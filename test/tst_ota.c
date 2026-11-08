/**
 * \file            tst_ota.c
 * \brief           OTA test helpers: stream encoder and flash recorder, see the header
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_ota.h"
#include <string.h>
#include "at32f415.h"
#include "tst_bsp_fakes.h"

/** \brief TEA golden-ratio delta. */
#define TST_TEA_DELTA      0x9E3779B9u
/** \brief TEA rounds (0x20). */
#define TST_TEA_ROUNDS     32u
/** \brief Cipher block size. */
#define TST_OTA_BLOCK      8u
/** \brief Key bytes. */
#define TST_OTA_KEY_BYTES  16u
/** \brief Receiver key-step stride (the key steps when `(offset & 0x3ff) == 0`). */
#define TST_OTA_KEY_STRIDE 0x3FFu
/** \brief Image word size. */
#define TST_OTA_WORD       4u
/** \brief Halfword size. */
#define TST_OTA_HALF       2u
/** \brief Recorder "no address" marker. */
#define TST_OTA_NO_ADDR    0xFFFFFFFFu

/* TEA key seed: the receiver loads it into its running key at every session reset. */
static const uint32_t TST_OTA_SEED[4] = {0xB21C80FEu, 0xA641EFD1u, 0xF53117A4u, 0xF02468A0u};

/* The original BSP functions (GNU ld --wrap). */
flash_status_type __real_flash_sector_erase(uint32_t sector_address);
flash_status_type __real_flash_halfword_program(uint32_t address, uint16_t data);

static tst_ota_flash_log_t g_log;
static uint32_t g_protect_lo;
static uint32_t g_protect_hi;
static uint32_t g_fail_addr;
/* The fakes' own custom_fake (tst_bsp_fakes.c: operation log + forward) the recorder chains to. */
static flash_status_type (*g_next_erase)(uint32_t);
static flash_status_type (*g_next_program)(uint32_t, uint16_t);

/**
 * \brief           Classic TEA encryption of one block (the inverse of the receiver's decrypt)
 * \param[in,out]   block: v0, v1
 * \param[in]       key: k0..k3
 */
void tst_ota_tea_encrypt(uint32_t* block, const uint32_t* key) {
    uint32_t v0 = block[0];
    uint32_t v1 = block[1];
    uint32_t sum = 0u;
    uint32_t round;

    for (round = 0u; round < TST_TEA_ROUNDS; ++round) {
        sum += TST_TEA_DELTA;
        v0 += ((v1 << 4) + key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + key[1]);
        v1 += ((v0 << 4) + key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + key[3]);
    }
    block[0] = v0;
    block[1] = v1;
}

/**
 * \brief           Start a stream: seed key, zero chaining, nothing sent
 * \param[out]      enc: stream state
 * \param[in]       declared: image length PREPARE announces
 */
void tst_ota_enc_init(tst_ota_enc_t* enc, uint32_t declared) {
    memset(enc, 0, sizeof(*enc));
    memcpy(enc->key, TST_OTA_SEED, sizeof(enc->key));
    enc->declared = declared;
}

/**
 * \brief           Encrypt one DATA chunk in place, then follow the receiver's offset and key step
 * \param[in,out]   enc: stream state
 * \param[in,out]   data: plaintext in, ciphertext out (`len` bytes, a multiple of 8)
 * \param[in]       len: chunk length
 * \note            The XOR is skipped for block 0 of the chunk with receiver offset 0
 *                      (`if (offset != 0 || i != 0)`). After the chunk the receiver offset moves
 *                      4 per word until it reaches declared - 4, and a 1 KB-aligned offset steps
 *                      the key bytes by their index.
 */
void tst_ota_enc_chunk(tst_ota_enc_t* enc, uint8_t* data, uint32_t len) {
    uint32_t pos;
    uint8_t* key_bytes = (uint8_t*)enc->key;

    for (pos = 0u; pos < len; pos += TST_OTA_BLOCK) {
        uint32_t block[2];

        memcpy(block, &data[pos], sizeof(block));
        if ((enc->rx_offset != 0u) || (pos != 0u)) {
            block[0] ^= enc->prev[0];
            block[1] ^= enc->prev[1];
        }
        tst_ota_tea_encrypt(block, enc->key);
        memcpy(&data[pos], block, sizeof(block));
        enc->prev[0] = block[0];
        enc->prev[1] = block[1];
    }
    for (pos = 0u; pos < len; pos += TST_OTA_WORD) {
        if ((int32_t)enc->rx_offset < (int32_t)(enc->declared - TST_OTA_WORD)) {
            enc->rx_offset += TST_OTA_WORD;
        }
    }
    if ((enc->rx_offset & TST_OTA_KEY_STRIDE) == 0u) {
        for (pos = 0u; pos < TST_OTA_KEY_BYTES; ++pos) {
            key_bytes[pos] = (uint8_t)(key_bytes[pos] + pos);
        }
    }
    enc->offset += len;
}

/**
 * \brief           Image checksum the receiver expects in the last word
 * \param[in]       image: plaintext image
 * \param[in]       len: image length (multiple of 4, >= 8)
 * \return          ~rol16(sum of every word but the last)
 */
uint32_t tst_ota_checksum(const uint8_t* image, uint32_t len) {
    uint32_t sum = 0u;
    uint32_t pos;

    for (pos = 0u; (pos + TST_OTA_WORD) < len; pos += TST_OTA_WORD) {
        uint32_t word;

        memcpy(&word, &image[pos], sizeof(word));
        sum += word;
    }
    return ~((sum << 16) | (sum >> 16));
}

/**
 * \brief           True if `address` lies in the protected (record-only) range
 * \param[in]       address: flash address
 * \return          1 inside, 0 outside
 */
static uint8_t prv_protected(uint32_t address) {
    return ((address >= g_protect_lo) && (address < g_protect_hi)) ? 1u : 0u;
}

/**
 * \brief           Recording erase fake: log, fail on request, forward outside the protected range
 * \param[in]       sector_address: page to erase
 * \return          BSP status
 */
static flash_status_type prv_erase(uint32_t sector_address) {
    flash_status_type status = FLASH_OPERATE_DONE;

    if (g_log.erase_count < TST_OTA_LOG_MAX) {
        g_log.erase_addr[g_log.erase_count] = sector_address;
    }
    ++g_log.erase_count;
    if (sector_address == g_fail_addr) {
        status = FLASH_PROGRAM_ERROR;
    } else if (prv_protected(sector_address) == 0u) {
        status = (g_next_erase != NULL) ? g_next_erase(sector_address) : __real_flash_sector_erase(sector_address);
    } else if (g_next_erase != NULL) {
        tst_flash_forward(0u); /* logged in the operation log, flash untouched */
        status = g_next_erase(sector_address);
        tst_flash_forward(1u);
    }
    return status;
}

/**
 * \brief           Recording halfword-program fake: log, fail on request, forward outside the range
 * \param[in]       address: target address
 * \param[in]       data: halfword
 * \return          BSP status
 */
static flash_status_type prv_program(uint32_t address, uint16_t data) {
    flash_status_type status = FLASH_OPERATE_DONE;

    if (g_log.program_count < TST_OTA_LOG_MAX) {
        g_log.program_addr[g_log.program_count] = address;
    }
    ++g_log.program_count;
    if (address < g_log.program_min) {
        g_log.program_min = address;
    }
    if ((address > g_log.program_max) || (g_log.program_max == TST_OTA_NO_ADDR)) {
        g_log.program_max = address;
    }
    if (address == g_fail_addr) {
        status = FLASH_PROGRAM_ERROR;
    } else if (prv_protected(address) == 0u) {
        status = (g_next_program != NULL) ? g_next_program(address, data)
                                          : __real_flash_halfword_program(address, data);
    } else if (g_next_program != NULL) {
        tst_flash_forward(0u);
        status = g_next_program(address, data);
        tst_flash_forward(1u);
    }
    return status;
}

/**
 * \brief           Install the recorder on the erase / halfword-program fakes and clear its log
 * \param[in]       protect_lo: first address that is only recorded (reported done, not touched)
 * \param[in]       protect_hi: end of that range (exclusive); lo == hi protects nothing
 * \note            Call after tst_fakes_reset. Writes outside the range reach the emulated
 *                      flash (emulator memory only; the device backup is never written).
 *                      Every operation that does not fail on request is passed on to the
 *                      fakes' own handler, so it also shows in \ref tst_flash_op_get
 *                      (protected ones with forwarding off for that call).
 */
void tst_ota_flash_record(uint32_t protect_lo, uint32_t protect_hi) {
    memset(&g_log, 0, sizeof(g_log));
    g_log.program_min = TST_OTA_NO_ADDR;
    g_log.program_max = TST_OTA_NO_ADDR;
    g_protect_lo = protect_lo;
    g_protect_hi = protect_hi;
    g_fail_addr = TST_OTA_NO_ADDR;
    tst_ota_flash_resume();
}

/**
 * \brief           Re-install the recorder after a tst_fakes_reset, keeping its log and settings
 */
void tst_ota_flash_resume(void) {
    if (__wrap_flash_sector_erase_fake.custom_fake != prv_erase) {
        g_next_erase = __wrap_flash_sector_erase_fake.custom_fake;
    }
    if (__wrap_flash_halfword_program_fake.custom_fake != prv_program) {
        g_next_program = __wrap_flash_halfword_program_fake.custom_fake;
    }
    __wrap_flash_sector_erase_fake.custom_fake = prv_erase;
    __wrap_flash_halfword_program_fake.custom_fake = prv_program;
}

/**
 * \brief           Make the erase or program of one address report FLASH_PROGRAM_ERROR
 * \param[in]       address: the failing address (0xFFFFFFFF: none)
 */
void tst_ota_flash_fail_at(uint32_t address) { g_fail_addr = address; }

/**
 * \brief           The recorder's log
 * \return          log since the last \ref tst_ota_flash_record
 */
const tst_ota_flash_log_t* tst_ota_flash_log(void) { return &g_log; }

/**
 * \brief           Erase one page directly (test set-up, not logged)
 * \param[in]       address: page address
 * \return          1 on success
 */
uint8_t tst_ota_flash_raw_erase(uint32_t address) {
    flash_status_type status;

    flash_unlock();
    status = __real_flash_sector_erase(address);
    flash_lock();
    return (status == FLASH_OPERATE_DONE) ? 1u : 0u;
}

/**
 * \brief           Program halfwords directly into erased flash (test set-up, not logged)
 * \param[in]       address: first address
 * \param[in]       data: halfwords
 * \param[in]       halfwords: count
 * \return          1 on success
 */
uint8_t tst_ota_flash_raw_write(uint32_t address, const uint16_t* data, uint32_t halfwords) {
    flash_status_type status = FLASH_OPERATE_DONE;
    uint32_t idx;

    flash_unlock();
    for (idx = 0u; (idx < halfwords) && (status == FLASH_OPERATE_DONE); ++idx) {
        status = __real_flash_halfword_program(address + (idx * TST_OTA_HALF), data[idx]);
    }
    flash_lock();
    return (status == FLASH_OPERATE_DONE) ? 1u : 0u;
}

/* Linker symbols of the image's load region (emu-test.ld / startup). */
extern uint8_t _sidata[];
extern uint8_t _sdata[];
extern uint8_t _edata[];

/**
 * \brief           First flash address after the test image (code + .data load image)
 * \return          end address; slot-1 staging is only safe when it is <= 0x08010000
 */
uint32_t tst_ota_image_end(void) { return (uint32_t)(uintptr_t)_sidata + (uint32_t)(_edata - _sdata); }
