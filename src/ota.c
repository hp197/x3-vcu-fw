/**
 * \file            ota.c
 * \brief           OTA / firmware-update receiver (x3 VCU)
 * \note            Two RAM structs hold the state: the session (ota_session_t / g_ota; byte +0 is
 *                      the re-entrancy busy flag) and the boot descriptor (ota_descriptor_t /
 *                      g_desc; first 12 bytes are the slot-commit descriptor, +0xc the 16-byte
 *                      running TEA key/IV). Only the descriptor's first 12 bytes reach flash.
 *                      Flash goes through the Artery BSP (as in config.c): unlock -> erase ->
 *                      halfword-program (+ read-back verify) -> lock.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "ota.h"
#include <stddef.h>
#include <string.h>
#include "at32f415.h"
#include "config.h"
#include "report.h" /* report_ninebot */
#include "usart.h"  /* vcu_usart_report_send (USART reply ring enqueue) */
#include "vcu_state.h"

/* Peripheral-context handles (owned by the CAN / USART subsystems; the dispatcher only
   passes them to the reply paths): g_uartctx and g_candev, the same objects the drivers
   and every report writer use. */
#define OTA_USART_CTX         ((uint32_t)&g_uartctx)
#define OTA_CAN_CTX           ((uint32_t)&g_candev)

/* OTA reply status codes the dispatcher returns. */
#define OTA_STATUS_OK         0x00u /* no error                                   */
#define OTA_STATUS_LEN        0x01u /* bad length / region overrun                */
#define OTA_STATUS_ERASE      0x02u /* slot-1 erase failed (ota_erase_slot1)      */
#define OTA_STATUS_FLASH      0x03u /* erase/program of flash failed              */
#define OTA_STATUS_SEQ        0x05u /* unexpected sequence byte                   */
#define OTA_STATUS_BUSY       0x06u /* session already in progress                */
#define OTA_STATUS_STATE      0x07u /* bad opcode / SP sanity / precondition      */
#define OTA_STATUS_CHECKSUM   0x08u /* trailing checksum mismatch                 */
#define OTA_STATUS_NO_PREPARE 0x09u /* DATA before PREPARE (declared_len == 0)    */

/* `payload` is the stored Ninebot frame (usart.h NB_OFF_*): [NB_OFF_LEN] = byte count of
   this chunk, [NB_OFF_SRC] = requester (the reply dst), [NB_OFF_CMD] = OTA command
   (NB_CMD_OTA_PREPARE..APPLY, ninebot.h), [NB_OFF_ARG] = sequence byte, [NB_OFF_BODY..] =
   data words. */

/* The running TEA key re-mutates each time the program offset crosses a 1 KB
   boundary. Keep the stride byte-exact. */
#define OTA_KEY_MUTATE_MASK   0x3FFu /* (1 KB - 1); offset & mask == 0 triggers the step */

/* Reply header the dispatcher emits: src NB_ADDR_VCU, dst = the
   requester, cmd OTA_REPLY_CMD, arg = status, body = the request's sequence byte. */
#define OTA_REPLY_CMD         0x0Bu /* reply cmd byte                                     */
#define OTA_ACK_LEN           0u    /* busy ACK: no body                                  */
#define OTA_REPLY_LEN         1u    /* status reply: 1-byte body (the sequence byte)      */

/**
 * \brief           TEA numeric key seed
 * \note            The only key. g_desc.key is loaded from this seed on every session reset
 *                      (PREPARE and after COMMIT), then mutates byte-wise (key[i] += i) each time
 *                      the program offset crosses a 1 KB boundary (see the DATA path).
 */
static const uint32_t OTA_TEA_KEY_SEED[4] = {0xB21C80FEu, 0xA641EFD1u, 0xF53117A4u, 0xF02468A0u};

/* The session state and the boot descriptor (BSS). */
static ota_session_t g_ota;
static ota_descriptor_t g_desc;

/* ------------------------------------------------------------------------- */
/* TEA 64-bit block decrypt                                                  */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Decrypt one 8-byte block in place
 * \param[in,out]   block: two little-endian 32-bit words (v0, v1)
 * \param[in]       key: 4-word running key (g_desc.key)
 * \note            Classic TEA, not XTEA: 32 rounds, sum = OTA_TEA_SUM0 decremented by
 *                      OTA_TEA_DELTA after both halves, fixed subkey indices (v1: key[2]/key[3],
 *                      v0: key[0]/key[1]) and the `(x<<4)+k ^ x+sum ^ (x>>5)+k` form - not the
 *                      XTEA `key[(sum>>11)&3]` schedule.
 */
static void ota_tea_decrypt(uint32_t* block, const uint32_t* key) {
    uint32_t v0 = block[0];
    uint32_t v1 = block[1];
    uint32_t sum = OTA_TEA_SUM0;

    for (uint32_t round = 0; round < OTA_TEA_ROUNDS; round++) {
        v1 -= ((key[OTA_TEA_KEY_IDX_V1] + (v0 << 4)) ^ (v0 + sum)) ^ (key[OTA_TEA_KEY_IDX_V1 + 1] + (v0 >> 5));
        v0 -= ((key[OTA_TEA_KEY_IDX_V0] + (v1 << 4)) ^ (v1 + sum)) ^ (key[OTA_TEA_KEY_IDX_V0 + 1] + (v1 >> 5));
        sum -= OTA_TEA_DELTA;
    }

    block[0] = v0;
    block[1] = v1;
}

/* ------------------------------------------------------------------------- */
/* Flash HAL (page erase, halfword program)                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Erase `len` bytes starting at `addr`, page-at-a-time
 * \param[in]       addr: page-aligned flash address
 * \param[in]       len: byte span; rounded up to whole 1 KB pages
 * \return          1 if every page erased, 0 on the first failure
 */
static int ota_flash_erase(uint32_t addr, uint32_t len) {
    uint32_t pages = (len + (OTA_PAGE_SIZE - 1u)) / OTA_PAGE_SIZE;
    flash_status_type st = FLASH_OPERATE_DONE;
    uint32_t i;

    flash_unlock();
    for (i = 0; (i < pages) && (st == FLASH_OPERATE_DONE); i++) {
        st = flash_sector_erase(addr + i * OTA_PAGE_SIZE);
    }
    flash_lock();
    return (st == FLASH_OPERATE_DONE) ? 1 : 0;
}

/**
 * \brief           Program `len` bytes as halfwords with read-back verify
 * \param[in]       dst: destination flash address
 * \param[in]       src: source halfwords
 * \param[in]       len: byte count (even)
 * \return          1 if the whole span programmed and verified, 0 otherwise
 * \note            Interrupts are disabled by the caller (the DATA path).
 */
static int ota_flash_program(uint32_t dst, const uint16_t* src, uint32_t len) {
    flash_status_type st = FLASH_OPERATE_DONE;
    uint32_t i = 0;

    flash_unlock();
    for (; (i < len) && (st == FLASH_OPERATE_DONE); i += 2) {
        st = flash_halfword_program(dst + i, *src);
        if ((st == FLASH_OPERATE_DONE) && (*(volatile uint16_t*)(dst + i) != *src)) {
            st = FLASH_PROGRAM_ERROR;
        }
        src++;
    }
    flash_lock();
    return (len <= i) ? 1 : 0;
}

/**
 * \brief           IRQ-guarded slot-1 erase
 * \param[in]       addr: slot-1 base
 * \param[in]       len: byte span to erase
 * \return          OTA_STATUS_OK on success, OTA_STATUS_ERASE on failure
 */
static uint8_t ota_erase_slot1(uint32_t addr, uint32_t len) {
    uint8_t status = OTA_STATUS_OK;

    __disable_irq();
    if (ota_flash_erase(addr, len) == 0) {
        status = OTA_STATUS_ERASE;
    }
    __enable_irq();
    return status;
}

/**
 * \brief           Write the 12-byte boot descriptor to flash
 * \return          1 on success, 0 if the erase or program failed
 * \note            Erase 12 bytes at OTA_BOOT_DESC_ADDR, then program {magic, commit, length} from
 *                      g_desc under disabled interrupts. Returns 0 if the in-RAM descriptor has no
 *                      valid magic, so an empty descriptor can't reach flash; the boot sync forces
 *                      the magic first, so the guard never fires in normal operation.
 */
int ota_write_descriptor(void) {
    int ok;

    if (g_desc.magic != OTA_DESC_MAGIC) {
        return 0;
    }
    __disable_irq();
    ok = ota_flash_erase(OTA_BOOT_DESC_ADDR, OTA_DESC_SIZE);
    if (ok != 0) {
        ok = ota_flash_program(OTA_BOOT_DESC_ADDR, (const uint16_t*)&g_desc, OTA_DESC_SIZE);
    }
    __enable_irq();
    return ok;
}

/* ------------------------------------------------------------------------- */
/* Session reset + key reseed                                                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Clear session state and reseed the TEA key
 * \note            Zeroes the session struct (busy flag and CBC blocks) and reloads g_desc.key
 *                      from OTA_TEA_KEY_SEED. Descriptor words 0/4/8 are left untouched.
 */
void ota_session_reset(void) {
    memset(&g_ota, 0, sizeof(g_ota));
    g_desc.key[0] = OTA_TEA_KEY_SEED[0];
    g_desc.key[1] = OTA_TEA_KEY_SEED[1];
    g_desc.key[2] = OTA_TEA_KEY_SEED[2];
    g_desc.key[3] = OTA_TEA_KEY_SEED[3];
}

/* ------------------------------------------------------------------------- */
/* System reset path (op10 APPLY tail)                                       */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Consolidate the identity record then software-reset (op10 tail)
 * \note            identity_store (re-consolidate) -> ~100 ms delay (config_delay_ms) ->
 *                      NVIC_SystemReset (DSB; AIRCR VECTKEY | SYSRESETREQ, PRIGROUP preserved; DSB;
 *                      spin). The const-cast hands the consolidator config.c's active record.
 */
/* Reg 0x31 (IN2+0x62) bit 14, cleared before the commit + reset. */
#define OTA_REG31_APPLY_BIT 0x4000U

static void ota_apply_reset(void) {
    g_in2.f.datetime_enable = (uint16_t)(g_in2.f.datetime_enable & (uint16_t)~OTA_REG31_APPLY_BIT);
    identity_store((identity_rec_t*)config_identity());
    config_delay_ms(100); /* SysTick millisecond delay */
    NVIC_SystemReset();   /* DSB; AIRCR SYSRESETREQ (PRIGROUP preserved); DSB; spin */
}

/* ------------------------------------------------------------------------- */
/* Sub-opcode handlers                                                       */
/* ------------------------------------------------------------------------- */

/**
 * \brief           op7 PREPARE: validate length, invalidate descriptor, erase slot 1
 * \param[in]       payload: command body (payload[0] must be 4)
 * \return          reply status code
 */
static uint8_t ota_op_prepare(const uint8_t* payload) {
    uint32_t count = payload[NB_OFF_LEN];
    uint32_t len = *(const uint32_t*)(payload + NB_OFF_BODY);
    uint8_t status;

    ota_session_reset();
    if (count != 4u) {
        return OTA_STATUS_STATE;
    }
    if (((len - 1u) < OTA_MAX_IMAGE_LEN) && ((len & 7u) == 0u)) {
        g_desc.commit = 0;
        if (ota_write_descriptor() == 0) {
            status = OTA_STATUS_FLASH;
        } else {
            g_ota.declared_len = len;
            if ((g_ota.declared_len & 7u) == 0u) {
                g_ota.block_count = (uint32_t)((int32_t)g_ota.declared_len / 8);
            } else {
                g_ota.block_count = (uint32_t)((int32_t)g_ota.declared_len / 8 + 1);
            }
            status = ota_erase_slot1(OTA_SLOT1_ADDR, g_ota.declared_len + OTA_HEADER_SLACK);
        }
    } else {
        status = OTA_STATUS_LEN;
    }
    return status;
}

/**
 * \brief           Decrypt + CBC-de-chain one DATA chunk in place
 * \param[in,out]   data: the chunk's data words (payload + NB_OFF_BODY)
 * \param[in]       count: chunk length in bytes (multiple of 8)
 * \note            For each 8-byte block: stash the ciphertext into the running CBC shuffle
 *                      (g_ota.cbc_*), TEA-decrypt in place, then XOR against the previous cipher
 *                      block - except the very first block of the image (program offset 0, block
 *                      0), which is not de-chained.
 */
static void ota_decrypt_chunk(uint8_t* data, uint32_t count) {
    for (uint32_t i = 0; i < count; i += 8) {
        uint8_t* block = data + i;

        g_ota.cbc_prev_lo = g_ota.cbc_cur_lo;
        g_ota.cbc_prev_hi = g_ota.cbc_cur_hi;
        g_ota.cbc_cur_lo = ((const uint32_t*)block)[0];
        g_ota.cbc_cur_hi = ((const uint32_t*)block)[1];
        ota_tea_decrypt((uint32_t*)block, g_desc.key);

        if (g_ota.offset != 0u || i != 0u) {
            const uint8_t* cbc_prev = (const uint8_t*)&g_ota.cbc_prev_lo;
            for (uint32_t j = 0; j < 8u; j++) {
                block[j] ^= cbc_prev[j];
            }
        }
    }
}

/**
 * \brief           Fold a programmed chunk into the running checksum
 * \param[in]       data: the chunk's data words (payload + NB_OFF_BODY)
 * \param[in]       count: chunk length in bytes (multiple of 4)
 * \return          OTA_STATUS_OK while summing or on a verified final word,
 *                      OTA_STATUS_CHECKSUM if the trailing word mismatches
 * \note            Sums 32-bit words, advancing g_ota.offset, until the trailing word is
 *                      reached (offset within 4 of declared_len). That word is the stored
 *                      image checksum: the accumulator is finalised as `~rol16(sum)` and
 *                      compared, marking the image complete either way.
 */
static uint8_t ota_accumulate_checksum(const uint8_t* data, uint32_t count) {
    for (uint32_t i = 0; i < count; i += 4) {
        int32_t word = *(const int32_t*)(data + i);

        if ((int32_t)(g_ota.declared_len - 4u) <= (int32_t)g_ota.offset) {
            g_ota.checksum = ~(((uint32_t)g_ota.checksum << 16) | ((uint32_t)g_ota.checksum >> 16));
            g_ota.complete = 1;
            if ((int32_t)g_ota.checksum == word) {
                g_ota.verified = 1;
                return OTA_STATUS_OK;
            }
            return OTA_STATUS_CHECKSUM;
        }
        g_ota.checksum = (uint32_t)(word + (int32_t)g_ota.checksum);
        g_ota.offset += 4;
    }
    return OTA_STATUS_OK;
}

/**
 * \brief           Mutate the running TEA key when the program offset crosses 1 KB
 * \note            Each key byte gets its own index added (kb[i] += i); keep byte-exact.
 */
static void ota_mutate_key_on_1k_boundary(void) {
    if ((g_ota.offset & OTA_KEY_MUTATE_MASK) == 0u) {
        uint8_t* kb = (uint8_t*)g_desc.key;
        for (uint32_t i = 0; i < sizeof(g_desc.key); i++) {
            kb[i] = (uint8_t)(kb[i] + (uint8_t)i);
        }
    }
}

/**
 * \brief           op8 DATA: decrypt, CBC-chain, program, accumulate the checksum
 * \param[in,out]   payload: command body; the data words are decrypted in place
 * \return          reply status code
 * \note            The DATA branch: the per-block cipher shuffle, the
 *                      "no XOR on the very first block" rule, the first-word vector-table
 *                      SP sanity gate, the program-then-accumulate order, the trailing-word
 *                      checksum finaliser, and the per-1 KB key byte mutation.
 */
static uint8_t ota_op_data(uint8_t* payload) {
    uint32_t count = payload[NB_OFF_LEN];
    uint8_t seq = payload[NB_OFF_ARG];
    uint8_t* data = payload + NB_OFF_BODY;
    uint8_t status = OTA_STATUS_OK;

    if (g_ota.declared_len == 0u) {
        return OTA_STATUS_NO_PREPARE;
    }
    if (g_ota.expected_seq != seq) {
        /* Tolerate a single retransmit of the previous chunk; reject anything else. The
           compare is 32-bit: with expected 0 the "previous" sequence is -1, which no byte
           matches. */
        if ((int32_t)seq == ((int32_t)g_ota.expected_seq - 1)) {
            return OTA_STATUS_OK;
        }
        return OTA_STATUS_SEQ;
    }
    if (count == 0u || (count & 7u) != 0u) {
        return OTA_STATUS_LEN;
    }

    ota_decrypt_chunk(data, count);
    memcpy(g_desc.scratch, payload, count); /* frame copy into the descriptor scratch */

    /* The first chunk carries the image vector table: its initial SP must point to RAM. */
    if (seq == 0u && g_ota.offset == 0u) {
        if ((*(const uint32_t*)data & OTA_SP_MASK) != OTA_SP_VALUE) {
            return OTA_STATUS_STATE;
        }
        g_ota.sp_valid = 1;
    }
    g_ota.started = 1;

    if ((int32_t)(g_ota.offset + count) >= (int32_t)(g_ota.declared_len + OTA_HEADER_SLACK)) {
        return OTA_STATUS_LEN; /* chunk would overrun the staged region */
    }
    if (g_ota.sp_valid == 0u) {
        return OTA_STATUS_STATE;
    }
    if (g_ota.complete != 0u) {
        return OTA_STATUS_OK; /* image already finalised: nothing more to do */
    }

    __disable_irq();
    if (ota_flash_program(OTA_SLOT1_ADDR + g_ota.offset, (const uint16_t*)data, count) == 0) {
        status = OTA_STATUS_FLASH;
    } else {
        g_ota.expected_seq++;
        status = ota_accumulate_checksum(data, count);
        ota_mutate_key_on_1k_boundary();
    }
    __enable_irq();
    return status;
}

/**
 * \brief           op9 COMMIT: write the slot-commit boot descriptor
 * \param[in]       payload: command body (payload[0] must be 4)
 * \return          reply status code
 */
static uint8_t ota_op_commit(const uint8_t* payload) {
    uint32_t count = payload[NB_OFF_LEN];
    uint8_t status;

    if ((count == 4u) && (g_ota.complete != 0u) && (g_ota.verified != 0u)) {
        g_desc.length = g_ota.declared_len;
        g_desc.commit = 1;
        if (ota_write_descriptor() == 0) {
            status = OTA_STATUS_FLASH;
        } else {
            ota_session_reset();
            status = OTA_STATUS_OK;
        }
    } else {
        status = OTA_STATUS_STATE;
    }
    return status;
}

/* ------------------------------------------------------------------------- */
/* Dispatcher + public API                                                   */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Emit the terminating OTA reply on the active transport
 * \param[in]       payload: the request frame (payload[NB_OFF_SRC] = reply dst)
 * \param[in]       transport: VCU_TRANSPORT_USART / VCU_TRANSPORT_CAN (other values: no reply)
 * \param[in]       status: reply status byte
 * \param[in]       tunnel: passed to the CAN reply (unused there)
 */
static void ota_send_reply(const uint8_t* payload, int transport, uint8_t status, uint32_t tunnel) {
    if (transport == (int)VCU_TRANSPORT_USART) {
        /* The USART reply goes straight into the reply ring: usart.c's
           vcu_usart_report_send(ctx, src, dst, len, cmd, arg, payload). */
        vcu_usart_report_send((volatile uint8_t*)OTA_USART_CTX, NB_ADDR_VCU, payload[NB_OFF_SRC],
                              OTA_REPLY_LEN, OTA_REPLY_CMD, status, payload + NB_OFF_ARG);
    } else if (transport == (int)VCU_TRANSPORT_CAN) {
        /* The CAN reply also arms the retransmit timer (ctx+0xe = 500) and sets
           ctx+0xc bit0 (g_candev event gate); ota_reply_can does that for a
           normal reply before sending. */
        ota_reply_can(NB_ADDR_VCU, payload[NB_OFF_SRC], OTA_REPLY_LEN, OTA_REPLY_CMD, status, payload + NB_OFF_ARG,
                      tunnel);
    }
}

void vcu_ota_dispatch(uint8_t* payload, int transport, uint32_t p3, uint32_t p4) {
    uint8_t cmd;
    uint8_t status = OTA_STATUS_OK;

    (void)p3;

    /* Re-entrancy guard: if a session is already running, ACK busy and bail. */
    if (g_ota.busy != 0u) {
        if (transport != (int)VCU_TRANSPORT_USART) { /* any non-USART transport */
            ota_reply_can(NB_ADDR_VCU, payload[NB_OFF_SRC], OTA_ACK_LEN, OTA_REPLY_CMD, OTA_STATUS_BUSY,
                          payload + NB_OFF_BODY, p4);
        } else {
            vcu_usart_report_send((volatile uint8_t*)OTA_USART_CTX, NB_ADDR_VCU, payload[NB_OFF_SRC],
                                  OTA_ACK_LEN, OTA_REPLY_CMD, OTA_STATUS_BUSY, NULL);
        }
        g_ota.busy = 0;
        return;
    }
    g_ota.busy = 1;

    cmd = payload[NB_OFF_CMD];
    if (cmd == NB_CMD_OTA_PREPARE) {
        status = ota_op_prepare(payload);
    } else if (cmd == NB_CMD_OTA_DATA) {
        status = ota_op_data(payload);
    } else if (cmd == NB_CMD_OTA_COMMIT) {
        status = ota_op_commit(payload);
    } else if (cmd == NB_CMD_OTA_APPLY) {
        if ((g_desc.commit != 0u) && (g_ota.verified == 0u)) {
            ota_apply_reset(); /* does not return */
        }
    } else {
        status = OTA_STATUS_STATE;
    }

    if (status != OTA_STATUS_OK) {
        ota_session_reset();
    }
    ota_send_reply(payload, transport, status, p4);
    g_ota.busy = 0;
}

/**
 * \brief           Boot-time descriptor sync (tail of config_identity_sync)
 */
void ota_desc_boot_sync(void) {
    memcpy((void*)&g_desc, (const void*)OTA_BOOT_DESC_ADDR, OTA_DESC_SIZE);
    if (g_desc.magic != OTA_DESC_MAGIC) {
        g_desc.commit = 0u;
        g_desc.length = 0u;
        g_desc.magic = OTA_DESC_MAGIC;
        (void)ota_write_descriptor();
    }
}

void vcu_ota_init(void) {
    /* g_desc is NOT cleared here: it already holds the descriptor loaded by
       ota_desc_boot_sync() (a zeroed copy would later be written to flash). */
    ota_session_reset();
}

/* ------------------------------------------------------------------------- */
/* CAN-transport reply                                                       */
/* ------------------------------------------------------------------------- */

/* Tunnel retransmit timer armed before a normal reply: ctx+0x0E = 500, ctx+0x0C |= 1 on
   the OTA_CAN_CTX block = g_candev event gate. */
#define OTA_RETX_TICKS   500
#define OTA_RETX_PENDING 0x0001u

/**
 * \brief           Send an OTA reply on the CAN transport (see ota.h)
 * \note            For a normal reply (\p len == OTA_REPLY_LEN; the busy ACK skips it)
 *                      it first arms the tunnel retransmit timer on the g_candev event
 *                      gate, then hands the reply to the Ninebot event-report sink
 *                      report_ninebot(src, dst, len, cmd, arg, data). \p tunnel is not
 *                      used.
 */
void ota_reply_can(uint8_t src, uint8_t dst, uint8_t len, uint8_t cmd, uint8_t arg, const void* data, uint32_t tunnel) {
    (void)tunnel;
    if (len == OTA_REPLY_LEN) {
        g_candev.event_wait = OTA_RETX_TICKS;
        g_candev.event_flags = (uint16_t)(g_candev.event_flags | OTA_RETX_PENDING);
    }
    report_ninebot(src, dst, len, cmd, arg, data);
}
