/**
 * \file            tst_ota.h
 * \brief           OTA test helpers: the sender side of the update stream and a flash recorder
 * \note            The encoder implements the update stream format (classic TEA, 32 rounds)
 *                      independently of ota.c:
 *                        - plaintext block P_i is XORed with the previous CIPHER block C_{i-1}
 *                          (not for the first block of the image), then TEA-encrypted;
 *                        - the key bytes get `key[i] += i` whenever the receiver's program
 *                          offset ends a chunk on a 1 KB boundary;
 *                        - the last image word is the checksum ~rol16(sum of the other words).
 *                      The recorder replaces the flash erase / halfword-program fakes: it logs
 *                      every address, forwards to the real BSP (emulator memory only) except
 *                      inside a protected range, and can fail one chosen address.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_OTA_H
#define TST_OTA_H

#include <stdint.h>

#ifndef TST_OTA_LOG_MAX
#define TST_OTA_LOG_MAX 80u /*!< erase / program addresses kept (later ones are only counted) */
#endif

/** \brief Sender-side state of one update stream. */
typedef struct {
    uint32_t key[4];    /*!< running TEA key (starts at the key seed)  */
    uint32_t prev[2];   /*!< previous cipher block                      */
    uint32_t offset;    /*!< bytes encrypted so far                     */
    uint32_t declared;  /*!< image length announced by PREPARE          */
    uint32_t rx_offset; /*!< the receiver's program offset (stops at declared - 4) */
} tst_ota_enc_t;

/** \brief What the flash recorder saw since \ref tst_ota_flash_record. */
typedef struct {
    uint32_t erase_count;                   /*!< flash_sector_erase calls          */
    uint32_t erase_addr[TST_OTA_LOG_MAX];   /*!< their addresses (first ones)      */
    uint32_t program_count;                 /*!< flash_halfword_program calls      */
    uint32_t program_min;                   /*!< lowest programmed address         */
    uint32_t program_max;                   /*!< highest programmed address        */
    uint32_t program_addr[TST_OTA_LOG_MAX]; /*!< their addresses (first ones)      */
} tst_ota_flash_log_t;

void tst_ota_tea_encrypt(uint32_t* block, const uint32_t* key);
void tst_ota_enc_init(tst_ota_enc_t* enc, uint32_t declared);
void tst_ota_enc_chunk(tst_ota_enc_t* enc, uint8_t* data, uint32_t len);
uint32_t tst_ota_checksum(const uint8_t* image, uint32_t len);

void tst_ota_flash_record(uint32_t protect_lo, uint32_t protect_hi);
void tst_ota_flash_resume(void);
void tst_ota_flash_fail_at(uint32_t address);
const tst_ota_flash_log_t* tst_ota_flash_log(void);
uint8_t tst_ota_flash_raw_erase(uint32_t address);
uint8_t tst_ota_flash_raw_write(uint32_t address, const uint16_t* data, uint32_t halfwords);
uint32_t tst_ota_image_end(void);

#endif /* TST_OTA_H */
