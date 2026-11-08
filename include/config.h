/**
 * \file            config.h
 * \brief           Identity record + configuration loader for the x3 VCU
 * \note            The identity sector is memory-mapped flash, so records are read with plain
 *                      pointer loads; only the consolidate path touches the flash controller.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stddef.h>
#include <stdint.h>
#include "model.h" /* VCU_MODEL_SERIAL_PREFIX, VCU_MODEL_DEFAULT_REGION */

/* ------------------------------------------------------------------------- */
/* Flash layout of the identity sector                                       */
/* ------------------------------------------------------------------------- */

#define IDENTITY_PAGE_SIZE      1024u                    /* flash page/erase granularity = 1 KB  */
#define IDENTITY_PAGE_A         0x0801F000u              /* identity record A (1 KB page)        */
#define IDENTITY_PAGE_B         (IDENTITY_PAGE_A + IDENTITY_PAGE_SIZE) /* 0x0801F400 record B (mirror) */
/* 0x0801F800 (the next page) is the OTA boot descriptor, OTA_BOOT_DESC_ADDR (ota.h). */
#define IDENTITY_REC_SIZE       512u                     /* record size checksum-covered         */
#define IDENTITY_REC_HALFWORDS  (IDENTITY_REC_SIZE / 2u) /* 256 int16s summed by the checksum */

/* Image header, placed by x3-vcu-fw.ld (.app_header.* sections): the banner at 0x08001400
   (src/app_header.c), the OTA auth key at 0x08001420 (NB_FLASH_AUTH_KEY, ninebot.c) and the build
   id at 0x08001430 (config.c). Tools identify the image by the banner (16 ASCII bytes at payload
   offset 0x400); the bootloader/OTA tooling reads the key and build id at these addresses. */
#define APP_HEADER_SECTION(part) __attribute__((section(".app_header." part), used))

/* Build id stamped into the record (+0x1C8..+0x1CD, BLE regs 0xE4..0xE6) at every boot: a
   string literal of exactly CONFIG_BUILD_ID_LEN characters (no terminator is stored). The build
   defines it (tools/build_id.py: the first 6 hex digits of the git commit, as text). With
   X3VCU_BUILD_ID=none it stays unset: 6 x 0xFF, the erased-flash value. */
#define CONFIG_BUILD_ID_LEN     6u
#ifndef CONFIG_BUILD_ID
#define CONFIG_BUILD_ID         "\xFF\xFF\xFF\xFF\xFF\xFF"
#endif

/* Record signature: word 0 of the factory-default record, checked by vcu_config_init. */
#define IDENTITY_MAGIC          0x505Cu

/* Field sizes written by the "not paired" default path. */
#define IDENTITY_SERIAL_LEN     14
#define IDENTITY_MODEL_LEN      16

/* Hard-coded factory / "not activated" identity. */
#define IDENTITY_DEFAULT_SERIAL VCU_MODEL_SERIAL_PREFIX VCU_MODEL_DEFAULT_REGION "0000000001" /* 14 chars */
#define IDENTITY_DEFAULT_MODEL  VCU_MODEL_BOARD_PREFIX "000000000001"                         /* 16 chars */

/**
 * \brief           512-byte identity / configuration record (A/B wear-levelled)
 * \note            Offsets are the byte offsets within the record. The
 *                      int16-sum checksum @+0x1F4 covers the whole record except
 *                      its own 32-bit word; `version` @+0x1FE selects the fresher
 *                      of the two pages. `reservedN` spans are not decoded here.
 */
typedef struct {
    uint16_t magic;                   /* +0x000 signature, IDENTITY_MAGIC class        */
    uint8_t reserved0[10];            /* +0x002                                        */
    int16_t throttle_cal_sel;         /* +0x00C reg 0x06 throttle cal select (==1)     */
    uint8_t reserved00e[18];          /* +0x00E                                        */
    char serial[IDENTITY_SERIAL_LEN]; /* +0x020 ASCII unit serial            */
    uint8_t reserved1[14];            /* +0x02E                                        */
    uint16_t cfg_flags2;              /* +0x03C reg 0x1E charge / gate bits            */
    uint8_t reserved03e[2];           /* +0x03E                                        */
    char model[IDENTITY_MODEL_LEN];   /* +0x040 component / model id         */
    uint8_t reserved2[48];            /* +0x050                                        */
    /* ---- per-mode / tune config window +0x080..+0x17F ---- */
    uint8_t config_80[6];             /* +0x080                                        */
    uint16_t prof_86;                 /* +0x086 reg 0x43 drive-profile cell            */
    uint16_t prof_88;                 /* +0x088 reg 0x44 drive-profile cell            */
    uint16_t prof_8a;                 /* +0x08A reg 0x45 drive-profile cell            */
    uint16_t speed_limit;             /* +0x08C reg 0x46 base speed ceiling (low byte) */
    uint8_t config_8e[4];             /* +0x08E                                        */
    uint16_t level;                   /* +0x092 reg 0x49 level / commloss scale        */
    uint8_t config_94[32];            /* +0x094                                        */
    int16_t gear;                     /* +0x0B4 reg 0x5A stored gear                   */
    uint8_t config_b6[12];            /* +0x0B6                                        */
    uint16_t passcode_num;            /* +0x0C2 reg 0x61 numeric passcode              */
    uint8_t config_c4[188];           /* +0x0C4                                        */
    /* ---- identity tail +0x180..+0x1F3 (registers 0xC0..0xF9) ---- */
    uint8_t mcu_id[20];               /* +0x180 reg 0xC0..0xC9: the motor controller's identity
                                         block. vcu_config_init asks CAN node 0x02 for 20 B of its
                                         reg 0xDA (0x18EF0216 `01 01 DA 14 00`); the reply (src 0x02,
                                         cmd 4, arg 0xDA) is copied here (IN2+0x180). First 12 B =
                                         MCU id. It equals the identity-backup `mcu_id`
                                         (683A10B8C0001104270305DC), word-swapped on the wire (reg
                                         read 3a68b81000c004110327dc05). */
    uint8_t reserved_194[32];         /* +0x194..+0x1B3 (no known reader or writer)    */
    uint8_t vcu_id[16];               /* +0x1B4 reg 0xDA..0xE1: this VCU's identity block, IN2
                                         `ident`. First 12 B = copy of the factory UID @0x1FFFF7E8
                                         (op 0x18; op 0x41 reads), word 2 doubles as the checksum
                                         salt. It equals the identity-backup `vcu_id`
                                         (359A005CC0000E06270305C4), word-swapped on the wire. */
    uint8_t reserved_1c4[4];          /* +0x1C4..+0x1C7                                */
    uint8_t build_id[6];              /* +0x1C8 reg 0xE4..0xE6 build id, stamped by
                                         config_identity_sync (IN2 build_id)           */
    uint16_t build_id_1ce;            /* +0x1CE cleared beside the build id (purpose unknown) */
    uint16_t rec_1d0;                 /* +0x1D0 set to 1 by vcu_config_init (purpose unknown) */
    uint16_t datetime_stored;         /* +0x1D2 report packed date-time (year/month/day) */
    uint8_t reserved_1d4[20];         /* +0x1D4..+0x1E7                                */
    uint16_t canrx_1e8_scratch;       /* +0x1E8 CAN RX scratch (echoed to report frame[0x2E]) */
    uint8_t reserved_1ea[10];         /* +0x1EA..+0x1F3                                */
    uint32_t checksum;                /* +0x1F4 int16-sum, excludes this word          */
    uint8_t reserved3[6];             /* +0x1F8                                        */
    uint16_t version;                 /* +0x1FE A/B version counter, higher wins       */
} identity_rec_t;

/* The on-flash layout must stay byte-exact. */

/**
 * \brief           Boot-time identity/config bring-up
 * \note            The first one-shot init in main(); also re-run on wake by the vehicle FSM.
 */
void vcu_config_init(void);

/**
 * \brief           Load the fresher valid identity record from the A/B pair
 * \param[out]      out: destination record (the RAM parameter buffer)
 * \return          0 if a record was selected, -1 if both pages failed
 * \note            Picks the higher `version`, gates
 *                      on the int16-sum checksum only (the magic/"paired" gate is
 *                      applied later in `vcu_config_init`), falls back to the
 *                      mirror page, and on total failure stamps `magic = 0xFFFF`.
 */
int identity_load(identity_rec_t* out);

/**
 * \brief           Consolidate a record to flash: dual-bank NVM store commit
 * \param[in,out]   rec: record to persist; its version + checksum are updated
 * \return          0 on success, -1 on erase/program/verify failure
 * \note            checksum -> erase -> program -> verify -> record sequence (flash
 *                      ping-pong writer). Public so other modules (e.g. ninebot.c
 *                      nb_state_refresh) can drive it.
 */
int identity_store(identity_rec_t* rec);

/**
 * \brief           Report whether a record carries a provisioned ("paired") magic
 * \param[in]       rec: record to test
 * \return          non-zero if paired, 0 if factory / "not paired"
 */
int identity_is_paired(const identity_rec_t* rec);

/**
 * \brief           Accessor for the active in-RAM identity/parameter block
 * \return          Pointer to the module-global record
 */
const identity_rec_t* config_identity(void);

/* ------------------------------------------------------------------------- */
/* Config accessors / validators (clamps + model-string parse)                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Clamp the record's level setting to [1,30], default 5
 * \note            Operates on the config window halfword at record +0x92.
 */
void config_validate_level(void);

/**
 * \brief           Latch + clamp the ride mode to {2,3,4}, default 2
 * \note            Copies the low byte of the config
 *                      halfword at record +0xB4 into the vehicle-state mode byte
 *                      and clamps it to the valid set.
 */
void config_validate_ride_mode(void);

/**
 * \brief           Parse a serial (family prefix + region code) into the drive row
 * \param[in]       model: NUL-free serial; bytes [0..3] are inspected
 * \note            The model's prefix plus a region code from CONFIG_REGION_CODES selects
 *                      that code's row (GT3: 03S + P/E/D -> 0/1/2); anything else sets the invalid
 *                      flag and row 0.
 */
void config_parse_model(const char* model);

/**
 * \brief           Validate + commit the stored serial against the factory template
 * \return          2 on a template-prefix match, 1 on full-length overflow, 0 on mismatch
 * \note            Validates g_rec.serial (record +0x20) against the embedded 14-char template
 *                      IDENTITY_DEFAULT_SERIAL (e.g. "03SP0000000001") with an alphanumeric-gated
 *                      walk, parses it as a model id (config_parse_model), and sets the
 *                      recognised/invalid latch on a committed serial. Used by identity_load as the
 *                      page-fallback gate; public for the pairing path.
 */
int config_serial_commit(void);

/**
 * \brief           Load/compare/consolidate the identity + boot-descriptor block
 * \return          First 8 bytes of the default config seed
 */
uint64_t config_identity_sync(void);

/* ------------------------------------------------------------------------- */
/* Init orchestrators (exposed as callable; main() wiring is the integrator's) */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Millisecond busy-wait (SysTick from HCLK)
 * \param[in]       ms: milliseconds to wait
 * \note            Public: the OTA apply tail and ninebot.c use the same delay.
 */
void config_delay_ms(uint32_t ms);

/**
 * \brief           Microsecond busy-wait (single SysTick run)
 * \param[in]       ticks: microseconds to wait
 */
void config_delay_ticks(uint32_t ticks);

/**
 * \brief           Master peripheral bring-up orchestrator
 * \note            Brings up every peripheral once, in this order: SysTick, SPI2,
 *                      GPIO pin groups, buzzer, ADC (vcu_control_init), USART2
 *                      (vcu_usart_init), CAN1, TMR5, ERTC, IWDG, then releases JTAG.
 *                      Called once from main() after vcu_config_init.
 */
void vcu_periph_init(void);

/**
 * \brief           Power-on timing / startup FSM step
 * \param[in,out]   counter: caller-owned 16-bit startup counter/state
 */
void vcu_startup_poll(uint16_t* counter);

/**
 * \brief           Low-power standby sleep/wake orchestrator
 * \return          The USART2 data-register word read at the end of the cycle
 * \note            Straight call sequence (no branching) + two clock-config snapshot
 *                      copies: arms the wake EXINT lines, resets CAN1, snapshots two CRM
 *                      words into the vehicle-state cache, selects low-power mode and
 *                      enters deep sleep (WFI), then on wake masks the EXINT lines,
 *                      restores the PLL system clock (clock_resume) and settles.
 */
uint32_t vcu_standby_cycle(void);

/**
 * \brief           Output/indicator reset dispatcher
 * \note            Drives the three indicator / anti-theft-latch primitives
 *                      (control_out_pb13/pa6/pb14) to 0 and clears the g_scr +0x42
 *                      status halfword, in that order.
 */
void vcu_outputs_reset(void);

#endif /* CONFIG_H */
