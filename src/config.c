/**
 * \file            config.c
 * \brief           Identity record + configuration loader (x3 VCU)
 * \note            The identity pages are memory-mapped flash, so reads are plain pointer loads.
 *                      Only identity_store() touches the flash controller, through the Artery BSP
 *                      (flash_unlock / flash_sector_erase / flash_halfword_program / flash_lock) in
 *                      an erase -> program -> verify sequence. Parameter writes go to fixed
 *                      record-relative offsets.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "config.h"
#include <stddef.h>
#include <string.h>
#include "at32f415.h"
#include "board.h"     /* PCLK1_HZ                                                 */
#include "buzzer.h"    /* buzzer_beep, buzzer_init                                 */
#include "can.h"       /* vcu_can_lowlevel_init                                    */
#include "clock.h"     /* clock_resume                                             */
#include "control.h"   /* vcu_ride_mode_t, control_out_pb13/pa6/pb14 indicator primitives   */
#include "gpio.h"      /* EXINT wake inits + the PB12 / PB13-PA6-PB14 / PC14 output inits */
#include "ninebot.h"   /* nb_key_load, NB_FLASH_AUTH_KEY                           */
#include "ota.h"       /* ota_write_descriptor / ota_session_reset                 */
#include "report.h"    /* report_uid_telemetry_copy                                */
#include "ridemode.h"  /* ridemode_led_matrix_update                               */
#include "sensor.h"    /* vcu_sensor_bus_init                                      */
#include "usart.h"     /* vcu_usart_init                                           */
#include "vcu_state.h" /* single shared SRAM backing store (blackboard aliasing)  */

/**
 * \brief           Active identity / parameter block
 * \note            Sits at +0x32 of a larger parameter block; the 0x32-byte header (g_in1) holds
 *                      control-loop state owned by other subsystems.
 * \note            Lives in the shared SRAM store as g_in2, so the record aliases the cells
 *                      ninebot/faultmon/control reach through g_in2. &g_rec, g_rec.field and the
 *                      g_in2 field views all resolve to the same memory.
 */
#define g_rec (*(identity_rec_t*)(void*)&g_in2) /* the 512-byte IN2 record union */

/**
 * \brief           A/B version bookkeeping
 */
static struct {
    uint16_t reserved0;  /* +0x00 */
    uint16_t reserved2;  /* +0x02 */
    uint16_t active_ver; /* +0x04 chosen/active version counter */
    uint16_t slot_a_ver; /* +0x06 page A version               */
    uint16_t slot_b_ver; /* +0x08 page B version               */
} g_ctrl;

/* The loader fallback gate is the serial/pairing commit, config_serial_commit() in the
   config-validators section below, declared public in config.h. */

/**
 * \brief           int16-sum record checksum
 * \param[in]       rec: record to sum (first 0x200 bytes)
 * \return          Signed 32-bit sum of all 256 halfwords, less the two of the +0x1F4 checksum word
 */
static int32_t identity_checksum(const identity_rec_t* rec) {
    const int16_t* hw = (const int16_t*)rec;
    const size_t cksum_hw = offsetof(identity_rec_t, checksum) / 2u; /* = 0xFA */
    int32_t sum = 0;

    for (size_t i = 0; i < IDENTITY_REC_HALFWORDS; i++) {
        sum += hw[i];
    }
    /* drop the two halfwords of the stored checksum word */
    return (sum - hw[cksum_hw]) - hw[cksum_hw + 1u];
}

/**
 * \brief           Load the fresher valid identity record
 * \param[out]      out: destination record (the RAM parameter buffer)
 * \return          0 if a record was selected, -1 if both pages failed
 * \note            Three exits: the "<=2 returns" guideline is relaxed here to keep each A/B
 *                      fallback case flat.
 */
int identity_load(identity_rec_t* out) {
    const identity_rec_t* page_a = (const identity_rec_t*)IDENTITY_PAGE_A;
    const identity_rec_t* page_b = (const identity_rec_t*)IDENTITY_PAGE_B;
    int fault_a = 0;
    int fault_b = 0;
    int state = 0;

    g_ctrl.slot_a_ver = page_a->version; /* page A +0x1FE */
    g_ctrl.slot_b_ver = page_b->version; /* page B +0x1FE */

    if (g_ctrl.slot_b_ver < g_ctrl.slot_a_ver) {
        memcpy(out, page_a, IDENTITY_REC_SIZE);
        if (identity_checksum(out) == (int32_t)out->checksum) {
            g_ctrl.active_ver = g_ctrl.slot_a_ver;
        } else {
            fault_a = config_serial_commit();
            state = 1;
        }
    } else if (g_ctrl.slot_a_ver < g_ctrl.slot_b_ver) {
        memcpy(out, page_b, IDENTITY_REC_SIZE);
        if (identity_checksum(out) == (int32_t)out->checksum) {
            g_ctrl.active_ver = g_ctrl.slot_b_ver;
        } else {
            fault_b = config_serial_commit();
            state = 2;
        }
    } else {
        state = 3;
    }

    if (state == 1) {
        memcpy(out, page_b, IDENTITY_REC_SIZE);
        if (identity_checksum(out) == (int32_t)out->checksum) {
            g_ctrl.active_ver = g_ctrl.slot_b_ver;
            state = 0;
            identity_store(out); /* rewrite stale mirror */
        } else {
            fault_b = config_serial_commit();
            state = 3;
        }
    } else if (state == 2) {
        memcpy(out, page_a, IDENTITY_REC_SIZE);
        if (identity_checksum(out) == (int32_t)out->checksum) {
            g_ctrl.active_ver = g_ctrl.slot_a_ver;
            state = 0;
            identity_store(out);
        } else {
            fault_a = config_serial_commit();
            state = 3;
        }
    }

    if (state == 3) {
        if ((fault_a == 0) || (fault_a == 1)) {
            g_ctrl.active_ver = g_ctrl.slot_a_ver;
            memcpy(out, page_a, IDENTITY_REC_SIZE);
            identity_store(out);
            return 0;
        }
        if (fault_b == 0) {
            g_ctrl.active_ver = g_ctrl.slot_b_ver;
            memcpy(out, page_b, IDENTITY_REC_SIZE);
            identity_store(out);
            return 0;
        }
        g_ctrl.active_ver = 0;
        out->magic = 0xFFFF; /* invalidate magic -> "not paired" in init */
        return -1;
    }

    return 0;
}

/**
 * \brief           Consolidate a record to flash: dual-bank writer
 * \param[in,out]   rec: record to persist; version + checksum are refreshed
 * \return          0 on success, -1 on erase/program/verify failure
 * \note            Bumps the version, writes it and a fresh checksum into the record, then writes
 *                      to the page picked by version parity (even -> A, odd -> B) so the loader
 *                      always re-selects the freshest page. Runs unlock -> erase -> halfword-program
 *                      (+ read-back verify) -> lock with interrupts disabled.
 */
int identity_store(identity_rec_t* rec) {
    const uint16_t* src = (const uint16_t*)rec;
    uint32_t page;
    flash_status_type st;
    uint32_t i;

    g_ctrl.active_ver += 1;                           /* ctrl+4 += 1          */
    rec->version = g_ctrl.active_ver;                 /* record +0x1FE        */
    rec->checksum = (uint32_t)identity_checksum(rec); /* record +0x1F4        */

    page = ((g_ctrl.active_ver & 1u) == 0u) ? IDENTITY_PAGE_A : IDENTITY_PAGE_B;

    __disable_irq();
    flash_unlock();                /* keys FLASH_UNLOCK_KEY1/2 (0x45670123/0xCDEF89AB) */
    st = flash_sector_erase(page); /* erase one 1 KB page */
    for (i = 0; (i < (IDENTITY_REC_SIZE / 2u)) && (st == FLASH_OPERATE_DONE); i++) {
        st = flash_halfword_program(page + i * 2u, src[i]);
        if ((st == FLASH_OPERATE_DONE) && (*(volatile uint16_t*)(page + i * 2u) != src[i])) {
            st = FLASH_PROGRAM_ERROR; /* read-back verify */
        }
    }
    flash_lock();
    __enable_irq();

    if (st == FLASH_OPERATE_DONE) {
        if ((g_ctrl.active_ver & 1u) == 0u) {
            g_ctrl.slot_a_ver = g_ctrl.active_ver;
        } else {
            g_ctrl.slot_b_ver = g_ctrl.active_ver;
        }
    }
    return (st == FLASH_OPERATE_DONE) ? 0 : -1;
}

/**
 * \brief           Report whether a record is provisioned ("paired")
 * \param[in]       rec: record to test
 * \return          non-zero if the magic matches, 0 if factory / "not paired"
 * \note            The gate vcu_config_init uses: the +0x00 magic must equal IDENTITY_MAGIC. A
 *                      blank, corrupt or checksum-failed sector leaves it unwritten or 0xFFFF, so
 *                      the unit falls back to the placeholder serial.
 */
int identity_is_paired(const identity_rec_t* rec) { return rec->magic == IDENTITY_MAGIC; }

/* Factory-default record cell values.
   Record byte offset = template offset = IN2 offset (Ninebot reg = offset / 2); the
   offsets are the identity_rec_t fields named next to each value. */
#define IDENTITY_DEF_THR_CAL      1u      /* +0x0C throttle_cal_sel: ==1 selects the 0xA1 window */
#define IDENTITY_DEF_CFG_FLAGS2   0x0300u /* +0x3C cfg_flags2: bit8 + bit9 (POWER / SPORT gate) */
#define IDENTITY_DEF_PROF_86      0x0505u /* +0x86 prof_86                                */
#define IDENTITY_DEF_PROF_88      0x0505u /* +0x88 prof_88                                */
#define IDENTITY_DEF_PROF_8A      25u     /* +0x8A prof_8a                                */
#define IDENTITY_DEF_SPEED_LIMIT  0x3232u /* +0x8C speed_limit: low byte = base ceiling 50   */
#define IDENTITY_DEF_LEVEL        5u      /* +0x92 level: = CONFIG_LEVEL_DEFAULT          */
#define IDENTITY_DEF_GEAR         2u      /* +0xB4 gear: VCU_GEAR_DRIVE_A (control.c)     */
#define IDENTITY_DEF_PASSCODE_NUM 1111u   /* +0xC2 passcode_num: default numeric passcode */

/**
 * \brief           Write the factory-default identity record (0x200 B)
 * \param[out]      rec: record to overwrite
 * \note            vcu_config_init writes this over the RAM record when the stored magic does not
 *                      match, then writes serial + model. The template is a zero fill plus the
 *                      magic and the nine non-zero cells set below.
 */
static void identity_set_defaults(identity_rec_t* rec) {
    memset(rec, 0, sizeof *rec);
    rec->magic = IDENTITY_MAGIC;
    rec->throttle_cal_sel = (int16_t)IDENTITY_DEF_THR_CAL;
    rec->cfg_flags2 = IDENTITY_DEF_CFG_FLAGS2;
    rec->prof_86 = IDENTITY_DEF_PROF_86;
    rec->prof_88 = IDENTITY_DEF_PROF_88;
    rec->prof_8a = IDENTITY_DEF_PROF_8A;
    rec->speed_limit = IDENTITY_DEF_SPEED_LIMIT;
    rec->level = IDENTITY_DEF_LEVEL;
    rec->gear = (int16_t)IDENTITY_DEF_GEAR;
    rec->passcode_num = IDENTITY_DEF_PASSCODE_NUM;
}

/** \brief Record +0x2E boot default: the firmware version the VCU reports (0x0170). */
#define CONFIG_SERIAL_MARK_DEFAULT 0x0170u

/** \brief TMR5 period: 10 counts of the 10 kHz counter = 1.0 ms tick. */
#define CONFIG_TICK_PERIOD         9u
/** \brief TMR5 prescaler: the TMR5 clock (2 x PCLK1, APB1 is /2) down to the 10 kHz counter. */
#define CONFIG_TICK_DIV            (((PCLK1_HZ * 2u) / 10000u) - 1u)

/** \brief IWDG reload at LICK / 256: 0x4E1 = 1250 counts, ~8 s. */
#define CONFIG_WDT_RELOAD          0x4E1u

/** \brief Warm-boot marker at g_scr+0x4A. */
#define CONFIG_SCR_WARM_MAGIC      0x000Bu

/** \brief Body length of the sub-node 0x02 component request: the count as a 16-bit LE value. */
#define CONFIG_SUB02_REQ_LEN       2u

/**
 * \brief           Boot identity/config bring-up
 */
void vcu_config_init(void) {
    const uint8_t sub02_req[CONFIG_SUB02_REQ_LEN] = {NB_SUB02_COMPONENT_LEN, 0u}; /* count 0x14, LE */

    /* Always clear the IN1 header (g_in1). Warm-boot guard on g_scr+0x4A (ui_screen): clear SCR
       unless it holds 0x000B. The RAM record is kept; each clear is bounded by sizeof. */
    memset((void*)&g_in1, 0, sizeof(g_in1));
    if ((uint16_t)g_scr.ui_screen != CONFIG_SCR_WARM_MAGIC) {
        memset((void*)&g_scr, 0, sizeof(g_scr));
    }

    identity_load(&g_rec);
    nb_key_load();         /* auth key: image header -> RAM */

    if (!identity_is_paired(&g_rec)) {
        /* "Not paired": write the factory-default record (including the magic), then
           overwrite serial + model. */
        identity_set_defaults(&g_rec);
        memcpy(g_rec.serial, IDENTITY_DEFAULT_SERIAL, IDENTITY_SERIAL_LEN);
        memcpy(g_rec.model, IDENTITY_DEFAULT_MODEL, IDENTITY_MODEL_LEN);
        identity_store(&g_rec); /* consolidate defaults to flash */
    }

    /* Order matters: ridemode_led_matrix_update reads the g_vs region that config_serial_commit
       (via config_parse_model) sets, and config_validate_ride_mode writes VS+1 after it. */
    (void)config_serial_commit();   /* serial commit + IN1+0x24 latch */
    ridemode_led_matrix_update();   /* LED/status matrix projector    */
    config_validate_level();        /* level clamp [1,30] -> 5        */
    config_validate_ride_mode();    /* VS+1 ride-mode latch {2,3,4}   */
    control_esc_staging_defaults(); /* g_mode defaults                */
    report_uid_telemetry_copy();    /* UID copy + uid_cksum           */

    /* Zero faultmon's UV pre-arm counter. It matters when the vehicle FSM re-runs this init
       on wake. */
    g_lowglob.fm_uv_cnt = 0;

    /* Tail defaults, written into the record (offsets are record-relative). */
    g_in2.f.serial_mark = CONFIG_SERIAL_MARK_DEFAULT; /* +0x2E reported firmware version */
    g_in2.f.trip_dist = 0;                            /* +0xD0                           */
    g_in2.f.trip_total_time = 0;                      /* +0xD2                           */
    g_in2.f.trip_ride_time = 0;                       /* +0xD4                           */
    g_in2.f.speed_src = 0;                            /* +0xAE                           */
    g_in2.rec_1d0 = 0x0001;                           /* +0x1D0                          */
    g_node2_push.state = NODE2_PUSH_IDLE;             /* no node-2 push pending          */

    /* Ask sub-node 0x02 for its component record: a reg read (cmd 0x01) of reg 0xDA, count 0x14 in
       a 2-byte body -> CAN 0x18EF0216 `01 01 DA 14 00`. The node replies with an op-4 write of reg
       0xDA that nb_sub02_command (ninebot.c) copies to IN2+0x180. The frame is enqueued, but boot
       clears the TX ring before CAN1 is up, so only the FSM re-run (vehicle.c) sends it. */
    report_ninebot(NB_ADDR_VCU, NB_ADDR_SUB02, CONFIG_SUB02_REQ_LEN, NB_CMD_READ, NB_REG_SUB02_COMPONENT, sub02_req);

    /* The CAN 0x100 frame counter (history +0x1C, u32) starts from 0. */
    g_telem_frame[REPORT_TELEM_TX100_LO] = 0;
    g_telem_frame[REPORT_TELEM_TX100_HI] = 0;
}

/**
 * \brief           Accessor for the active in-RAM identity/parameter block
 * \return          Pointer to the module-global record
 */
const identity_rec_t* config_identity(void) { return &g_rec; }

/* ======================================================================== */
/* Config accessors / validators + init orchestrator                        */
/* ------------------------------------------------------------------------ */
/* Config-window validators and the boot-init orchestrator around the       */
/* identity record above. Offsets in the record go through g_rec. These     */
/* also touch RAM blackboards outside it (the 0x32-byte param header, the   */
/* vehicle-state struct, two boot-descriptor blocks) via the vcu_state.h    */
/* globals, and call cross-module bring-up (CAN, SPI2, buzzer, GPIO)        */
/* through each module's owner.                                             */
/* ======================================================================== */

/* The build-id cell sits in the image header beside the auth-key cell (NB_FLASH_AUTH_KEY,
   ninebot.c); the linker places it, so this is a named constant, not an absolute read. Its 6
   bytes come from CONFIG_BUILD_ID (config.h): tools/build_id.py fills in the first 6 hex digits
   of the git commit as text (e.g. "99bc2c"), readable as BLE regs 0xE4..0xE6. Without git, or
   with X3VCU_BUILD_ID=none, it stays 6 x 0xFF. */
static const char k_config_build_id[CONFIG_BUILD_ID_LEN] APP_HEADER_SECTION("build_id") = CONFIG_BUILD_ID;

/* 16-byte default config seed (little-endian words). */
#define CONFIG_SEED_W0          0xB21C80FEu
#define CONFIG_SEED_W1          0xA641EFD1u
#define CONFIG_SEED_W2          0xF53117A4u
#define CONFIG_SEED_W3          0xF02468A0u

/* Level clamp (config_validate_level): valid range and the fallback default. */
#define CONFIG_LEVEL_MIN        1
#define CONFIG_LEVEL_MAX        30
#define CONFIG_LEVEL_DEFAULT    5

/* Model id format parsed by config_parse_model: a 3-char family then a 1-char variant. */
#define CONFIG_MODEL_PREFIX_LEN 3u /*!< family prefix length of a serial (VCU_MODEL_SERIAL_PREFIX) */
/** \brief Region code per row (index = \ref vcu_region_t): P Global 50 km/h, E Europe 25 km/h, D Germany 20 km/h */
#define CONFIG_REGION_CODES     "PED"

/* config_identity_sync helper. */
int config_block_cmp(const void* a, const void* b, uint32_t n) { /* 1 if equal, else 0 */
    const uint8_t* pa = (const uint8_t*)a;
    const uint8_t* pb = (const uint8_t*)b;
    uint32_t i;

    for (i = 0; i < n; i++) {
        if (pa[i] != pb[i]) {
            return 0;
        }
    }
    return 1;
}

/* ------------------------------------------------------------------------- */
/* SysTick busy-wait primitives. SysTick runs from HCLK; the per-ms / per-us  */
/* reload factor is derived from the live core clock; one SysTick tick == one */
/* HCLK cycle.                                                                */
/* ------------------------------------------------------------------------- */
#define CONFIG_DELAY_MS_STEP 50u /* chunk (0x32): keeps fac_ms * step inside the 24-bit reload */

/* Load SysTick, run it once down to zero from HCLK and spin on COUNTFLAG. */
static void config_systick_run(uint32_t reload) {
    SysTick->LOAD = reload;
    SysTick->VAL = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
        /* wait for the down-counter to wrap */
    }
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0u;
}

/* Millisecond busy-wait. Chunked at CONFIG_DELAY_MS_STEP ms so fac_ms * chunk stays
   inside SysTick's 24-bit reload. */
void config_delay_ms(uint32_t ms) {
    uint32_t fac_ms = SystemCoreClock / 1000u; /* HCLK ticks per millisecond */

    while (ms != 0u) {
        uint32_t chunk = (ms <= CONFIG_DELAY_MS_STEP) ? ms : CONFIG_DELAY_MS_STEP;
        config_systick_run(fac_ms * chunk);
        ms -= chunk;
    }
}

/* vcu_startup_poll callee. */
/* Charge-relay / startup GPIO drive: GPIOB pin 12. on == 1
   energizes the relay (SCR set); any other value releases it (CLR reset). */
void config_startup_gpio(int on) {
    if (on == 1) {
        gpio_bits_set(GPIOB, GPIO_PINS_12);
    } else {
        gpio_bits_reset(GPIOB, GPIO_PINS_12);
    }
}

/**
 * \brief           Clamp the record's level setting to [1,30], default 5
 * \note            The clamped field is the config halfword at record +0x92.
 */
void config_validate_level(void) {
    int16_t level = g_in2.f.auto_off_min_s; /* record +0x92 */

    if (level < CONFIG_LEVEL_MIN || level > CONFIG_LEVEL_MAX) {
        g_in2.f.auto_off_min_s = CONFIG_LEVEL_DEFAULT;
    }
}

/**
 * \brief           Latch + clamp the ride mode to {2,3,4}, default 2
 * \note            Takes the low byte of record +0xB4 into the vehicle-state mode byte g_vs +0x01,
 *                      then clamps.
 */
void config_validate_ride_mode(void) {
    volatile uint8_t* mode = &g_vs.gear_sel; /* g_vs +0x01 stored ride-mode latch */

    *mode = (uint8_t)g_in2.f.gear; /* low byte of record +0xB4 */
    if (*mode != VCU_MODE_ECO && *mode != VCU_MODE_SPORT && *mode != VCU_MODE_POWER) {
        *mode = VCU_MODE_ECO;
    }
}

/**
 * \brief           Parse a serial (family prefix + region code) into the drive row
 * \param[in]       model: NUL-free serial; bytes [0..3] are inspected
 * \note            The serial must start with the model's family prefix; its region code is
 *                      looked up in CONFIG_REGION_CODES, whose index is the region: P -> GLOBAL 0
 *                      (50 km/h), E -> EUROPE 1 (25 km/h), D -> GERMANY 2 (20 km/h).
 *                      Another prefix or an unknown region code sets the invalid flag
 *                      (g_in1 +0x24, dtc23_in) and row 0.
 */
void config_parse_model(const char* model) {
    static const char k_prefix[] = VCU_MODEL_SERIAL_PREFIX;
    static const char k_regions[] = CONFIG_REGION_CODES;
    uint32_t row = 0u;
    uint32_t idx;
    uint8_t found = 0u;

    if (strncmp(model, k_prefix, CONFIG_MODEL_PREFIX_LEN) == 0) { /* stops at a NUL in a short string */
        for (idx = 0u; idx < (sizeof(k_regions) - 1u); ++idx) {
            if ((found == 0u) && (model[CONFIG_MODEL_PREFIX_LEN] == k_regions[idx])) {
                row = idx;
                found = 1u;
            }
        }
    }
    if (found == 0u) {
        g_in1.dtc23_in = 1; /* g_in1+0x24 model-bad/serial latch */
    }
    g_vs.region = (uint8_t)row;
}

/* Serial-commit template: the embedded 14-char factory serial the stored serial is
   validated against (== IDENTITY_DEFAULT_SERIAL, e.g. "03SP0000000001"). */
#define CONFIG_SERIAL_TEMPLATE    IDENTITY_DEFAULT_SERIAL

/* Serial-commit return codes (config_serial_commit). */
#define CONFIG_SERIAL_MISMATCH    0 /* alnum char diverged from the template         */
#define CONFIG_SERIAL_MATCH       2 /* template-prefix matched, stopped at non-alnum */
#define CONFIG_SERIAL_OVERFLOW    1 /* all 14 template chars matched (index overran)  */

/* Flag set on a committed serial: g_in1 +0x24, the same cell config_parse_model stamps,
   the "serial/model recognised/invalid" latch. */
#define CONFIG_SERIAL_COMMIT_FLAG g_in1.dtc23_in /* g_in1+0x24 */

/**
 * \brief           Validate + commit the stored serial against the factory template
 * \return          2 on a template-prefix match, 1 on full-length overflow, 0 on mismatch
 * \note            Parses the serial (record +0x20) as a model id (config_parse_model), then
 *                      walks it byte-by-byte while alphanumeric: a byte that diverges from the
 *                      template returns 0 (no flag); matching all 14 bytes overruns the index
 *                      (>0xD) and returns 1; a non-alphanumeric terminator while still matching
 *                      returns 2. Both 1 and 2 set the recognised flag (g_in1 +0x24). The goto is
 *                      the shared overflow check (the same "<=2 returns" relaxation as identity_load).
 */
int config_serial_commit(void) {
    const char* serial = g_rec.serial;         /* record +0x20      */
    const char* tmpl = CONFIG_SERIAL_TEMPLATE; /* embedded template */
    int ret = CONFIG_SERIAL_MISMATCH;
    uint32_t i = 0;
    uint32_t c;

    config_parse_model(serial);

    for (;;) {
        c = (uint8_t)serial[i];
        /* Stop the walk at the first non-alphanumeric byte (0-9 / A-Z / a-z). */
        if (!((c - 0x30u < 10u) || (c - 0x41u < 26u) || (c - 0x61u < 26u))) {
            break;
        }
        if (c != (uint8_t)tmpl[i]) {
            goto commit; /* diverged: leave ret = 0, i <= 0xD */
        }
        i = (i + 1u) & 0xFFu;
        if (i > 0x0Du) {
            goto commit; /* all 14 template bytes matched: index overran */
        }
    }
    ret = CONFIG_SERIAL_MATCH;
    CONFIG_SERIAL_COMMIT_FLAG = 1;

commit:
    if (i > 0x0Du) {
        ret = CONFIG_SERIAL_OVERFLOW;
        CONFIG_SERIAL_COMMIT_FLAG = 1;
    }
    return ret;
}

/**
 * \brief           Load/compare/consolidate the identity + boot-descriptor block
 * \return          First 8 bytes of the default config seed (words 1:0)
 * \note            Loads the 16-byte seed and compares it against the flash auth-key cell (result
 *                      unused), stamps the build id (record +0x1C8) and clears +0x1CE, then
 *                      consolidates the record to flash twice (identity_store). Finally syncs the
 *                      OTA boot descriptor and resets the OTA session. In-record offsets go
 *                      through g_rec.
 */
uint64_t config_identity_sync(void) {
    uint32_t seed[4];

    seed[0] = CONFIG_SEED_W0;
    seed[1] = CONFIG_SEED_W1;
    seed[2] = CONFIG_SEED_W2;
    seed[3] = CONFIG_SEED_W3;
    (void)config_block_cmp(seed, NB_FLASH_AUTH_KEY, NB_KEY_LEN); /* auth-key cell, 16 bytes */

    g_in2.build_id_1ce = 0;
    /* the 6-byte build id from the image header (see k_config_build_id) */
    memcpy((void*)g_in2.build_id, k_config_build_id, sizeof(k_config_build_id));

    identity_store(&g_rec); /* stored twice: the version parity */
    identity_store(&g_rec); /* puts the record on both pages    */

    ota_desc_boot_sync(); /* load OTA_BOOT_DESC_ADDR into the OTA descriptor, force magic 0x505A */
    ota_session_reset();  /* owned by ota.c */

    return ((uint64_t)seed[1] << 32) | seed[0];
}

/* ======================================================================== */
/* Peripheral bring-up bodies called by vcu_periph_init.                    */
/* ======================================================================== */

/**
 * \brief           SysTick delay base (Artery `delay_init`)
 * \note            Selects the undivided core clock for SysTick. config_delay_ms /
 *                      config_delay_ticks derive fac_ms / fac_us live from SystemCoreClock,
 *                      so no globals are kept.
 */
static void config_delay_init(void) { SysTick->CTRL |= SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV; }

/**
 * \brief           PC14 power/fault latch output, released HIGH
 * \note            vcu_gpio_fault_latch_init is gpio.c's pin init (also used by the vehicle
 *                      FSM state-4 fault latch), then the pin is set.
 */
static void config_pc14_init(void) {
    vcu_gpio_fault_latch_init((uint8_t)GPIO_OUTPUT_PUSH_PULL);
    gpio_bits_set(GPIOC, GPIO_PINS_14);
}

/**
 * \brief           GPIOA pin(s) as input with pull-up, drive stronger
 * \param[in]       pins: pin mask on GPIOA
 * \note            Used for PA10 and PA9.
 */
static void config_gpioa_input_pu(uint16_t pins) {
    BOARD_CLOCK_ON(CRM_GPIOA_PERIPH_CLOCK);
    vcu_gpio_config(GPIOA, pins, VCU_GPIO_CFG_IN_PU);
}

/**
 * \brief           PB12 power latch output, asserted HIGH
 * \note            vcu_gpio_power_latch_init is gpio.c's pin init; setting the pin is what
 *                      holds power on (same pin as config_startup_gpio).
 */
static void config_power_latch_init(void) {
    vcu_gpio_power_latch_init();
    gpio_bits_set(GPIOB, GPIO_PINS_12); /* == config_startup_gpio(1) */
}

/**
 * \brief           PF6 aux output, driven LOW
 * \note            PF6 is the aux output control.c's debounce FSM drives (CTRL_GPIO_AUX).
 */
static void config_pf6_init(void) {
    BOARD_CLOCK_ON(CRM_GPIOF_PERIPH_CLOCK);
    vcu_gpio_config(GPIOF, GPIO_PINS_6, VCU_GPIO_CFG_OUT_PP);
    gpio_bits_reset(GPIOF, GPIO_PINS_6);
}

/**
 * \brief           Floating inputs for the faultmon node lines
 * \note            PA11 (node1) + PA12 (node3), then PB1 (node6) + PB2 (node5) + PB11 (node4).
 */
static void config_node_inputs_init(void) {
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_GPIOA_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_GPIOB_PERIPH_CLOCK));

    vcu_gpio_config(GPIOA, GPIO_PINS_11 | GPIO_PINS_12, VCU_GPIO_CFG_IN_FLOAT);
    vcu_gpio_config(GPIOB, GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_11, VCU_GPIO_CFG_IN_FLOAT);
}

/**
 * \brief           Pull-up inputs PB7 / PB6 for faultmon node2 + sensor2/3
 */
static void config_node2_inputs_init(void) {
    BOARD_CLOCK_ON(CRM_GPIOB_PERIPH_CLOCK);
    vcu_gpio_config(GPIOB, GPIO_PINS_6 | GPIO_PINS_7, VCU_GPIO_CFG_IN_PU);
}

/**
 * \brief           PB0 / PB10 subsystem-enable outputs, released HIGH
 * \note            Inits PB0 + PB10, then sets PB10, then PB0. Both are active-low
 *                      (control_subsys_a/b_enable), so HIGH means both subsystems released;
 *                      the charge FSM reads the latches back.
 */
static void config_subsys_outputs_init(void) {
    BOARD_CLOCK_ON(CRM_GPIOB_PERIPH_CLOCK);
    vcu_gpio_config(GPIOB, GPIO_PINS_0 | GPIO_PINS_10, VCU_GPIO_CFG_OUT_PP);

    gpio_bits_set(GPIOB, GPIO_PINS_10); /* subsystem A released (active-low) */
    gpio_bits_set(GPIOB, GPIO_PINS_0);  /* subsystem B released (active-low) */
}

/**
 * \brief           PB13 / PA6 / PB14 indicator outputs, driven LOW
 * \note            vcu_gpio_indicator_outputs_init is gpio.c's pin init; these are the pins
 *                      the control_out_pb13/pa6/pb14 indicator primitives drive.
 */
static void config_indicator_outputs_init(void) {
    vcu_gpio_indicator_outputs_init();
    gpio_bits_reset(GPIOB, GPIO_PINS_13);
    gpio_bits_reset(GPIOA, GPIO_PINS_6);
    gpio_bits_reset(GPIOB, GPIO_PINS_14);
}

/**
 * \brief           TMR5 base timer with overflow IRQ
 * \note            Counter at (2 x PCLK1) / 10000 = 10 kHz, PR = 9, so an overflow every
 *                      10 counts = 1.0 ms. The x2 is hard-coded (valid while APB1 is not /1).
 *                      The ISR is report.c's TMR5_GLOBAL_IRQHandler.
 */
static void config_tick_timer_init(void) {
    /* 10 kHz counter, period 9 = 10 counts: a 1.0 ms scheduler tick. */
    BOARD_CLOCK_ON(CRM_TMR5_PERIPH_CLOCK);
    /* tmr_base_init: pr, div, then a software overflow event loads them. */
    TMR5->pr = CONFIG_TICK_PERIOD;
    TMR5->div = CONFIG_TICK_DIV;
    TMR5->swevt_bit.ovfswtr = TRUE;
    TMR5->ctrl1_bit.cnt_dir = TMR_COUNT_UP;
    TMR5->iden |= TMR_OVF_INT;
    board_irq_enable(TMR5_GLOBAL_IRQn, BOARD_IRQ_PRIO(1, 0));
    TMR5->ctrl1_bit.tmren = TRUE;
}

/* ERTC register access, as the BSP ertc_* functions do it. */
#define CONFIG_ERTC_WP_KEY1  0xCAu       /*!< wp unlock, first key                              */
#define CONFIG_ERTC_WP_KEY2  0x53u       /*!< wp unlock, second key                             */
#define CONFIG_ERTC_WP_LOCK  0xFFu       /*!< wp: write protection on                           */
#define CONFIG_ERTC_POLLS    0x6C0000u   /*!< init-mode / update flag polls (2 x ERTC_TIMEOUT)  */
#define CONFIG_ERTC_STS_INIT 0xFFFFFFFFu /*!< sts write that requests init mode                 */
#define CONFIG_ERTC_STS_RUN  0xFFFFFF7Fu /*!< sts write that leaves init mode                   */
#define CONFIG_ERTC_IMEN     0x80u       /*!< sts imen [7]: init mode requested                 */
#define CONFIG_ERTC_DIV_MASK 0x007F7FFFu /*!< div diva [22:16] + divb [14:0]                    */
#define CONFIG_ERTC_DIV_1HZ  0x0063018Fu /*!< diva 99, divb 399: 40 kHz LICK / 100 / 400 = 1 Hz */
#define CONFIG_ERTC_WAT_CNT  2u          /*!< 1 Hz CK_B, counter 2 -> wakeup every 3 s          */

/**
 * \brief           Wait for an ERTC sts flag to be set
 * \param[in]       flag: sts flag mask
 * \return          1 when the flag was seen within \ref CONFIG_ERTC_POLLS polls, else 0
 */
static uint32_t prv_ertc_wait(uint32_t flag) {
    uint32_t polls = CONFIG_ERTC_POLLS;

    while (((ERTC->sts & flag) == 0u) && (polls != 0u)) {
        polls--;
    }
    return ((ERTC->sts & flag) != 0u) ? 1u : 0u;
}

/**
 * \brief           Enter ERTC init mode (ertc_init_mode_enter, write protection already off)
 * \return          1 in init mode, 0 on timeout
 * \note            On a timeout the BSP switches the write protection back on; the callers
 *                      here keep it off until their own lock, with no ERTC access in between.
 */
static uint32_t prv_ertc_init_mode(void) {
    if (ERTC->sts_bit.imf == 0u) {
        ERTC->sts = CONFIG_ERTC_STS_INIT;
    }
    return prv_ertc_wait(ERTC_IMF_FLAG);
}

/**
 * \brief           Clear an ERTC sts flag (ertc_flag_clear): write 0 to it, keep imen
 * \param[in]       flag: sts flag mask
 */
static void prv_ertc_flag_clear(uint32_t flag) {
    ERTC->sts = ~(flag | CONFIG_ERTC_IMEN) | ((uint32_t)ERTC->sts_bit.imen << 7);
}

/**
 * \brief           Wait for the ERTC calendar update (ertc_wait_update)
 */
static void prv_ertc_wait_update(void) {
    prv_ertc_flag_clear(ERTC_UPDF_FLAG);
    (void)prv_ertc_wait(ERTC_UPDF_FLAG);
}

/**
 * \brief           ERTC on LICK at 1 Hz plus a 3 s wakeup timer on EXINT22
 * \note            The battery-domain reset runs on every boot, so the ERTC is only a wakeup tick,
 *                      not a persistent clock: the calendar is never set or read, just the 1 Hz
 *                      prescaler and the wakeup timer. Write protection is lifted once for the final
 *                      programming rather than per register. The ISR is \ref ERTC_WKUP_IRQHandler.
 */
static void config_ertc_init(void) {
    /* ERTC config */
    BOARD_CLOCK_ON(CRM_PWC_PERIPH_CLOCK);
    PWC->ctrl_bit.bpwen = TRUE;  /* battery-powered domain write access */
    CRM->bpdc_bit.bpdrst = TRUE; /* battery-powered domain reset pulse */
    CRM->bpdc_bit.bpdrst = FALSE;
    CRM->ctrlsts_bit.licken = TRUE;
    while (CRM->ctrlsts_bit.lickstbl == 0u) {}
    CRM->bpdc_bit.ertcsel = CRM_ERTC_CLOCK_LICK;
    CRM->bpdc_bit.ertcen = TRUE;

    /* ertc_reset: every register back to its reset value, then the calendar update. */
    ERTC->wp = CONFIG_ERTC_WP_KEY1;
    ERTC->wp = CONFIG_ERTC_WP_KEY2;
    ERTC->ctrl = 0u;
    if (prv_ertc_init_mode() != 0u) {
        ERTC->time = 0u;
        ERTC->date = 0x00002101u;
        ERTC->ctrl = 0u;
        ERTC->div = 0x007F00FFu;
        ERTC->wat = 0x0000FFFFu;
        ERTC->ccal = 0u;
        ERTC->ala = 0u;
        ERTC->alb = 0u;
        ERTC->tadj = 0u;
        ERTC->scal = 0u;
        ERTC->tamp = 0u;
        ERTC->alasbs = 0u;
        ERTC->albsbs = 0u;
        ERTC->sts = 0u;
        prv_ertc_wait_update();
    }
    ERTC->wp = CONFIG_ERTC_WP_LOCK;
    prv_ertc_wait_update();

    /* 1 Hz prescaler (in init mode), then the wakeup timer. */
    ERTC->wp = CONFIG_ERTC_WP_KEY1;
    ERTC->wp = CONFIG_ERTC_WP_KEY2;
    if (prv_ertc_init_mode() != 0u) {
        ERTC->div = (ERTC->div & ~CONFIG_ERTC_DIV_MASK) | CONFIG_ERTC_DIV_1HZ;
        ERTC->sts = CONFIG_ERTC_STS_RUN;
    }
    ERTC->ctrl_bit.watclk = ERTC_WAT_CLK_CK_B_16BITS;
    ERTC->wat_bit.val = CONFIG_ERTC_WAT_CNT;
    ERTC->ctrl |= ERTC_WAT_INT;
    ERTC->ctrl_bit.waten = TRUE;
    ERTC->wp = CONFIG_ERTC_WP_LOCK;

    vcu_exint_arm(EXINT_LINE_22, 1u); /* ERTC wakeup line, rising edge */

    board_irq_enable(ERTC_WKUP_IRQn, BOARD_IRQ_PRIO(0, 1));
}

/**
 * \brief           ERTC wakeup ISR: latch the wake flag
 * \note            Sets g_vs +0x07, then clears the ERTC wakeup flag and the EXINT22
 *                      pending bit.
 */
void ERTC_WKUP_IRQHandler(void) {
    if ((ERTC->sts & ERTC_WATF_FLAG) != 0u) {
        g_vs.ertc_wake = 1; /* g_vs +0x07 */
        prv_ertc_flag_clear(ERTC_WATF_FLAG);
        EXINT->intsts = EXINT_LINE_22;
    }
}

/**
 * \brief           Independent watchdog: /256, reload 0x4E1
 * \note            1249 / (40 kHz / 256) = ~8.0 s. main's super-loop feeds it with
 *                      wdt_counter_reload() each time the 3 s ERTC wakeup ISR raises
 *                      g_vs.ertc_wake.
 */
static void config_wdt_init(void) {
    WDT->cmd = WDT_CMD_UNLOCK; /* div / rld write access */
    WDT->div_bit.div = WDT_CLK_DIV_256;
    WDT->rld = CONFIG_WDT_RELOAD;
    wdt_counter_reload();
    WDT->cmd = WDT_CMD_ENABLE;
}

/**
 * \brief           Master peripheral bring-up orchestrator
 * \note            Brings up every peripheral once, in this order; main() does not re-init any.
 *                      Callees owned by another module (SPI2, buzzer, ADC, USART, CAN, the GPIO
 *                      output inits) go through that module's owner.
 */
void vcu_periph_init(void) {
    NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUP_2); /* the CMSIS write of nvic_priority_group_config */
    config_delay_init();
    if (VCU_HAS_FEATURE(IMU)) {
        vcu_sensor_bus_init(); /* sensor.c: SPI2 for the IMU */
    }
    config_pc14_init();
    config_gpioa_input_pu(GPIO_PINS_10); /* PA10 */
    config_gpioa_input_pu(GPIO_PINS_9);  /* PA9  */
    config_power_latch_init();
    config_pf6_init();
    buzzer_init();           /* buzzer.c  */
    vcu_control_init();      /* control.c */
    vcu_usart_init();        /* usart.c   */
    vcu_can_lowlevel_init(); /* can.c     */
    config_node_inputs_init();
    config_node2_inputs_init();
    config_subsys_outputs_init();
    config_indicator_outputs_init();
    config_tick_timer_init();
    config_ertc_init();
    config_wdt_init();
    /* With PA12 or PA9 reading high the debug port is switched off completely, else JTAG off /
       SWD kept. */
    if (((GPIOA->idt & GPIO_PINS_12) != 0u) || ((GPIOA->idt & GPIO_PINS_9) != 0u)) {
        board_pin_remap(SWJTAG_GMUX_100);
    } else {
        board_pin_remap(SWJTAG_GMUX_010);
    }
    config_delay_ms(10);
}

/**
 * \brief           Power-on timing / startup FSM step
 * \param[in,out]   counter: caller-owned 16-bit startup counter/state
 * \note            Ramps the counter to 1000; at 999 drives the startup
 *                      GPIO off and beeps once (long). Once latched at >=1000, if PA10
 *                      (the EXINT10 wake line) is not active and the counter is < 0x44C
 *                      it waits 500 ms, drives the GPIO on and parks the counter at 0x4B0.
 */
void vcu_startup_poll(uint16_t* counter) {
    uint16_t v;

    v = *counter;
    if (v < 1000) {
        *counter = (uint16_t)(v + 1);
        if (v == 999) {
            config_startup_gpio(0);
            buzzer_beep(BEEP_LONG);
        }
    } else if (vcu_gpio_pa10_active() == 0 && *counter < 0x44C) {
        config_delay_ms(500);
        config_startup_gpio(1);
        *counter = 0x4B0;
    }
}

/* ======================================================================== */
/* Low-power standby + output-reset orchestrators                           */
/* ------------------------------------------------------------------------ */
/* Shared cells go through the vcu_state.h per-block globals. Cross-module  */
/* callees go through their module's public API or the BSP.                 */
/* ======================================================================== */

/* Standby runs through the Artery HAL / CMSIS. The pre-sleep CRM snapshots go into the VS cache
   (CRM->ctrl -> g_vs+0x5C, CRM->cfg -> g_vs+0x60), the low-power select is PWC->ctrl_bit.vrsel,
   deep sleep uses SCB_SCR_SLEEPDEEP_Msk, the standby status byte is g_vs+0x08, and the cycle ends
   reading USART2->dt. The output reset clears SCR->submode_code (g_scr+0x42). */

/* Dwell after resume, in microseconds (config_delay_ticks). */
#define CONFIG_STANDBY_DELAY_TICKS 120u

/* The indicator / anti-theft-latch primitives the reset dispatcher drives are control.c's
   control_out_pb13/pa6/pb14 (declared in control.h). The post-wake clock restore is
   clock.c's clock_resume. */

/* Microsecond busy-wait: single SysTick run of fac_us * n. The parameter is
   microseconds, not scheduler ticks. */
void config_delay_ticks(uint32_t ticks) {
    uint32_t fac_us = SystemCoreClock / 1000000u; /* HCLK ticks per microsecond */

    config_systick_run(fac_us * ticks);
}

/**
 * \brief           Low-power deepsleep voltage-regulator select
 * \param[in]       on: non-zero selects the low-power regulator for deep sleep
 * \note            PWC_CTRL bit0 (VRSEL) is the deepsleep voltage-regulator select,
 *                      written as pwc_voltage_regulate_set() does (the ctrl vrsel bitfield).
 */
static void config_pwr_lp_select(uint32_t on) {
    PWC->ctrl_bit.vrsel = ((on & 1u) != 0u) ? PWC_REGULATOR_LOW_POWER : PWC_REGULATOR_ON;
}

/**
 * \brief           Enter deep sleep and wait
 * \param[in]       wake_on_event: 1 -> WFE x2, 0 -> WFI (else neither)
 * \note            Sets SCB->SCR SLEEPDEEP (CMSIS), waits, then clears it. Hand-written
 *                      rather than pwc_deep_sleep_mode_enter() because the HAL has no
 *                      three-way branch (it only does WFI or an __SEV-prefixed WFE pair);
 *                      SLEEPDEEP uses the CMSIS SCB_SCR_SLEEPDEEP_Msk.
 */
static void config_deepsleep_wait(int wake_on_event) {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    if (wake_on_event == 1) {
        __WFE();
        __WFE();
    } else if (wake_on_event == 0) {
        __WFI();
    }
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

/**
 * \brief           Low-power standby sleep/wake orchestrator
 * \return          The USART2 data-register word read at the end of the cycle
 * \note            A low-power STANDBY (sleep/wake) cycle: arm the wake EXINT lines,
 *                      quiesce CAN, snapshot two clock-config words, select low-power
 *                      mode and WFI, then on wake mask the EXINT lines, restore the PLL
 *                      system clock and settle, in that order.
 */
uint32_t vcu_standby_cycle(void) {
    /* 1. Re-arm the two wake-EXINT groups (PA3, then PA10/PC13). */
    vcu_gpio_exint3_init();      /* owned by gpio.c */
    vcu_gpio_exint_10_13_init(); /* owned by gpio.c */

    /* 2. Quiesce CAN1 via its CRM peripheral reset. */
    vcu_can_periph_reset(); /* owned by can.c */

    /* 3. Snapshot two clock-config words into the vehicle-state cache, then clear status:
          CRM->cfg -> VS+0x60, CRM->ctrl -> VS+0x5C. */
    g_vs.crm_cfg_save = CRM->cfg;   /* g_vs+0x60 clock-cfg snapshot */
    g_vs.crm_ctrl_save = CRM->ctrl; /* g_vs+0x5C clock-ctrl snapshot */
    g_vs.wake_src = 0;              /* g_vs+0x08 standby status byte */

    /* 4. Select low-power mode and enter deep sleep (wake on interrupt). */
    config_pwr_lp_select(1);
    config_deepsleep_wait(0); /* __WFI */

    /* 5. On wake, mask the three wake-EXINT lines again (line3 / line10 / line13). */
    EXINT->inten &= ~(EXINT_LINE_3 | EXINT_LINE_10 | EXINT_LINE_13);

    /* 6. Restore HEXT + PLL as SYSCLK, settle, and return the USART2 data word. */
    clock_resume();
    config_delay_ticks(CONFIG_STANDBY_DELAY_TICKS);
    return USART2->dt; /* USART2 data register (+0x04), via CMSIS */
}

/**
 * \brief           Output/indicator reset dispatcher
 * \note            Drives the three indicator / anti-theft-latch primitives to 0 (the
 *                      same trio control.c's control_output_light uses), then clears the
 *                      g_scr +0x42 status halfword.
 */
void vcu_outputs_reset(void) {
    control_out_pb13(0);
    control_out_pa6(0);
    control_out_pb14(0);

    ((volatile vcu_scr_t*)((volatile uint8_t*)&g_scr))->submode_code = 0; /* clear g_scr +0x42 */
}
