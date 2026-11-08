/**
 * \file            ota.h
 * \brief           OTA / firmware-update receiver for the x3 VCU
 * \note            The OTA image is streamed in 8-byte blocks, TEA-decrypted + CBC-chained,
 *                      programmed into slot 1 (0x08010000), checksum-verified, then committed by
 *                      writing a 12-byte boot descriptor to 0x0801F800. APPLY issues a SYSRESETREQ
 *                      so the 4 KB bootloader copies slot 1 -> slot 0.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef OTA_H
#define OTA_H

#include <stddef.h>
#include <stdint.h>
#include "ninebot.h" /* vcu_transport_t */

/* ------------------------------------------------------------------------- */
/* Flash geometry                                                            */
/* ------------------------------------------------------------------------- */

#define OTA_SLOT1_ADDR        0x08010000u /* OTA staging slot 1                       */
#define OTA_BOOT_DESC_ADDR    0x0801F800u /* 12-byte boot descriptor                  */
#define OTA_PAGE_SIZE         1024u       /* flash page / erase granularity = 1 KB    */
#define OTA_DESC_SIZE         12u         /* boot descriptor length in bytes          */
#define OTA_DESC_MAGIC        0x0000505Au /* boot descriptor magic 'Z''P'              */
#define OTA_HEADER_SLACK      128u        /* extra region erased ahead of the body    */

/* ------------------------------------------------------------------------- */
/* Image-header sanity limits                                                */
/* ------------------------------------------------------------------------- */

#define OTA_SP_MASK           0x2FFE0000u /* mask for the vector-table initial SP     */
#define OTA_SP_VALUE          0x20000000u /* required masked SP value (points to RAM)  */
#define OTA_MAX_IMAGE_LEN     0xF000u     /* declared length must satisfy len-1 < this */

/* The op10 APPLY tail resets via CMSIS NVIC_SystemReset() (SCB->AIRCR
   VECTKEY | SYSRESETREQ, PRIGROUP preserved), so no raw AIRCR literals are needed. */

/* ------------------------------------------------------------------------- */
/* TEA block cipher constants                                                */
/* ------------------------------------------------------------------------- */

#define OTA_TEA_SUM0          0xC6EF3720u /* initial sum = delta * 32                 */
#define OTA_TEA_DELTA         0x9E3779B9u /* golden-ratio delta                       */
#define OTA_TEA_ROUNDS        32u         /* decrypt rounds                           */

/* Classic TEA subkey pairing: every round feeds the v1 half-round from
   key[2],key[3] and the v0 half-round from key[0],key[1] (not XTEA's
   `key[(sum>>11)&3]`). These indices are load-bearing - do not change. */
#define OTA_TEA_KEY_IDX_V1    2u /* v1 half-round subkey pair: key[2], key[3] */
#define OTA_TEA_KEY_IDX_V0    0u /* v0 half-round subkey pair: key[0], key[1] */

/* ------------------------------------------------------------------------- */
/* OTA session state                                                         */
/* ------------------------------------------------------------------------- */

/**
 * \brief           OTA session state
 * \note            Field offsets are fixed. The CBC previous/current cipher blocks at
 *                      +0x1c..+0x28 are also read as a byte array during the CBC XOR,
 *                      so their layout is load-bearing.
 */
typedef struct {
    uint8_t busy;          /* +0x00 re-entrancy guard                               */
    uint8_t expected_seq;  /* +0x01 next expected sequence byte                     */
    uint8_t sp_valid;      /* +0x02 image vector-table SP sanity passed             */
    uint8_t verified;      /* +0x03 trailing checksum matched                       */
    uint8_t complete;      /* +0x04 final block reached                             */
    uint8_t started;       /* +0x05 at least one DATA block processed               */
    uint8_t reserved6[6];  /* +0x06 zeroed by the reset, otherwise unused           */
    uint32_t declared_len; /* +0x0c image length declared by PREPARE                 */
    uint32_t offset;       /* +0x10 running program offset into slot 1               */
    uint32_t checksum;     /* +0x14 running 32-bit sum, then finalised value         */
    uint32_t block_count;  /* +0x18 declared_len / 8 (informational)                */
    uint32_t cbc_prev_lo;  /* +0x1c previous cipher block, low word                  */
    uint32_t cbc_prev_hi;  /* +0x20 previous cipher block, high word                 */
    uint32_t cbc_cur_lo;   /* +0x24 current cipher block, low word                   */
    uint32_t cbc_cur_hi;   /* +0x28 current cipher block, high word                  */
} ota_session_t;

/**
 * \brief           Boot descriptor + running key
 * \note            Its first 12 bytes {magic, commit, length} are programmed verbatim
 *                      to OTA_BOOT_DESC_ADDR by the descriptor writer. The 16-byte key at
 *                      +0xc is the running TEA key/IV; `scratch` receives a copy of each
 *                      DATA frame and is sized to bound any chunk the gated protocol can
 *                      deliver.
 */
typedef struct {
    uint32_t magic;         /* +0x00 OTA_DESC_MAGIC; forced at boot by ota_desc_boot_sync */
    uint32_t commit;        /* +0x04 commit / valid flag                              */
    uint32_t length;        /* +0x08 committed image length                           */
    uint32_t key[4];        /* +0x0c 16-byte running TEA key/IV                        */
    uint8_t scratch[0x100]; /* +0x1c DATA frame copy                                 */
} ota_descriptor_t;

/* The field offsets must stay byte-exact. */

/* ------------------------------------------------------------------------- */
/* Transport selector: the shared \ref vcu_transport_t (ninebot.h).           */
/* VCU_TRANSPORT_USART (0) replies into the USART report ring (g_uartctx);    */
/* VCU_TRANSPORT_CAN (1) replies via report_ninebot after arming g_candev     */
/* (+0xE = 500, +0xC |= 1).                                                   */
/* The dispatcher is called DIRECTLY by the RX handlers; no fetch/poll shim.  */
/* ------------------------------------------------------------------------- */

/**
 * \brief           One-shot OTA bring-up (clears state, seeds the TEA key)
 * \note            Runs ota_session_reset; called once from main().
 */
void vcu_ota_init(void);

/**
 * \brief           OTA sub-command dispatcher
 * \param[in,out]   payload: Ninebot dst-0x16 request frame (decrypted in place)
 * \param[in]       transport: \ref vcu_transport_t (VCU_TRANSPORT_USART or VCU_TRANSPORT_CAN)
 * \param[in]       p3: unused by the state machine
 * \param[in]       p4: passed to the CAN reply (unused there)
 * \note            op7 PREPARE, op8 DATA, op9 COMMIT, op10 APPLY; see ota.c for the
 *                      field map.
 */
void vcu_ota_dispatch(uint8_t* payload, int transport, uint32_t p3, uint32_t p4);

/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* Reply paths. There is no fetch function: vcu_ota_dispatch is invoked       */
/* directly by the RX handlers.                                               */
/* ------------------------------------------------------------------------- */

/* The USART reply path is usart.c's vcu_usart_report_send; ota.c's dispatcher calls it
   directly. */

/**
 * \brief           Send an OTA reply over CAN (via report_ninebot, after arming the
 *                      g_candev retransmit timer)
 */
void ota_reply_can(uint8_t src, uint8_t dst, uint8_t len, uint8_t cmd, uint8_t arg, const void* data,
                   uint32_t tunnel);

/* ------------------------------------------------------------------------- */
/* Cross-module owners exported for the config identity-sync path.           */
/* These bodies live here; config.c's config_identity_sync calls them         */
/* directly.                                                                  */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Write the 12-byte boot descriptor to flash
 * \return          1 on success, 0 if the erase or program failed
 * \note            Erase OTA_DESC_SIZE bytes at OTA_BOOT_DESC_ADDR, then program the
 *                      first 12 bytes of the running boot descriptor, under disabled
 *                      interrupts. Owned by ota.c; config_identity_sync() calls it directly.
 */
int ota_write_descriptor(void);

/**
 * \brief           Boot-time descriptor sync (tail of config_identity_sync)
 * \note            Loads the 12-byte descriptor from flash into g_desc; if its magic is not
 *                      OTA_DESC_MAGIC, resets commit/length, sets the magic and writes it back.
 */
void ota_desc_boot_sync(void);

/**
 * \brief           Clear OTA session state and reseed the TEA key
 * \note            Zeroes the session struct (busy flag, CBC blocks, counters) and
 *                      reloads the running TEA key from its seed; descriptor words 0/4/8
 *                      are left untouched. Owned by ota.c; config_identity_sync() calls it
 *                      directly.
 */
void ota_session_reset(void);

#endif /* OTA_H */
