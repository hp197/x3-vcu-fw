/**
 * \file            test_config.c
 * \brief           Boot configuration (config.c, without the identity store)
 * \note            Covers boot init vcu_config_init (block clears, factory defaults,
 *                      validators, tail defaults, the sub-node 0x02 request), the level clamp,
 *                      ride-mode latch, model parse, serial commit, startup FSM, output reset,
 *                      the ms / us delays, peripheral bring-up and standby. The identity A/B
 *                      store itself (load / save / checksum / build id / auth key / UID) is
 *                      test_identity's.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "at32f415.h"
#include "can.h"
#include "config.h"
#include "gpio.h"
#include "report.h"
#include "tst_bsp_fakes.h"
#include "tst_ota.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/* Identity sector (pages A / B) and record layout. */
#define PAGE_A           0x0801F000u
#define PAGE_B           0x0801F400u
#define REC_SIZE         512u
#define REC_CHECKSUM_OFF 0x1F4u
#define REC_VERSION_OFF  0x1FEu
#define REC_SERIAL_OFF   0x20u
#define REC_MODEL_OFF    0x40u
#define SERIAL_LEN       14u
#define MODEL_LEN        16u

/* Sub-node 0x02 component request: src 0x16, dst 2, len 2, cmd 1, reg 0xDA, body 0x14. */
#define SUB02_REQ_ID     0x18EF0216u
#define SUB02_REQ_DLC    5u

/* Startup FSM (vcu_startup_poll). */
#define STARTUP_RAMP_END 1000u
#define STARTUP_WAIT_MAX 1100u
#define STARTUP_PARKED   1200u

/* Clamp sets. */
#define LEVEL_DEFAULT    5
#define MODE_DEFAULT     2u
#define WARM_SCREEN      0x000B
#define SENTINEL         0x1234

/** \brief One register of a register-map check: its address and the whole-word value. */
typedef struct {
    const char* name;             /*!< register, for the failure message */
    const volatile uint32_t* reg; /*!< register address                  */
    uint32_t value;               /*!< expected 32-bit value             */
} reg_expect_t;

/** \brief Register-map entry of a 32-bit register */
#define REG(r, v)  {#r, &(r), (v)}

/** \brief Register-map entry of four NVIC priority bytes read as one word */
#define REGW(r, v) {#r, (const volatile uint32_t*)&(r), (v)}

/** \brief One non-zero 32-bit cell of the factory record (record offset + value). */
typedef struct {
    uint16_t offset; /*!< record offset           */
    uint32_t value;  /*!< little-endian cell value */
} rec_cell_t;

/* The factory record (all other cells of the 512-byte template are 0). */
static const rec_cell_t FACTORY_CELLS[] = {
    {0x000u, 0x0000505Cu}, {0x00Cu, 0x00000001u}, {0x03Cu, 0x00000300u}, {0x084u, 0x05050000u}, {0x088u, 0x00190505u},
    {0x08Cu, 0x00003232u}, {0x090u, 0x00050000u}, {0x0B4u, 0x00000002u}, {0x0C0u, 0x04570000u},
};

/**
 * \brief           The RAM record as a writable identity_rec_t (the IN2 register file)
 * \return          the record
 */
static identity_rec_t* prv_rec(void) { return (identity_rec_t*)(uintptr_t)config_identity(); }

/**
 * \brief           Put a serial into the record (14 bytes, NUL-padded)
 * \param[in]       text: up to 14 characters
 */
static void prv_set_serial(const char* text) {
    identity_rec_t* rec = prv_rec();
    size_t len = strlen(text);

    memset(rec->serial, 0, SERIAL_LEN);
    memcpy(rec->serial, text, (len < SERIAL_LEN) ? len : SERIAL_LEN);
}

/**
 * \brief           Configure PA10 (the wake line vcu_gpio_pa10_active reads) as an input with a pull
 * \param[in]       pull: GPIO_PULL_UP (reads 1, inactive) or GPIO_PULL_DOWN (reads 0, active)
 */
static void prv_pa10_pull(gpio_pull_type pull) {
    gpio_init_type gpio;

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&gpio);
    gpio.gpio_mode = GPIO_MODE_INPUT;
    gpio.gpio_pull = pull;
    gpio.gpio_pins = GPIO_PINS_10;
    gpio_init(GPIOA, &gpio);
}

/**
 * \brief           Fresh boot state for every test; CAN TX captured only; watchdog fed
 * \note            The last tests start the IWDG (~8 s): feeding it here is harmless before.
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    vcu_usart_init();
    wdt_counter_reload();
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* -- vcu_config_init ---------------------------------------------------------------------- */

/**
 * \brief           Boot init always clears the IN1 flag block
 * \note            The whole IN1 block is cleared. Only IN1+0x24 is written again afterwards
 *                      (serial commit / model parse latch).
 */
static void test_init_clears_in1(void) {
    volatile uint8_t* in1 = (volatile uint8_t*)&g_in1;
    uint32_t idx;

    for (idx = 0u; idx < sizeof(g_in1); ++idx) {
        in1[idx] = 0x55u;
    }
    vcu_config_init();
    for (idx = 0u; idx < sizeof(g_in1); ++idx) {
        if (idx != 0x24u) {
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(0u, in1[idx], "IN1 byte not cleared");
        }
    }
}

/**
 * \brief           SCR is cleared on a cold boot and kept when SCR+0x4A holds the warm marker 0x0B
 * \note            SCR (0x200 bytes) is cleared unless SCR+0x4A holds 0x000B.
 */
static void test_init_scr_warm_boot_guard(void) {
    g_scr.ui_screen = WARM_SCREEN;
    g_scr.distance_disp = SENTINEL;
    vcu_config_init();
    TEST_ASSERT_EQUAL_INT16(SENTINEL, g_scr.distance_disp);
    TEST_ASSERT_EQUAL_INT16(WARM_SCREEN, g_scr.ui_screen);

    g_scr.ui_screen = 0x0C;
    vcu_config_init();
    TEST_ASSERT_EQUAL_INT16(0, g_scr.distance_disp);
    TEST_ASSERT_EQUAL_INT16(0, g_scr.ui_screen);
}

/**
 * \brief           The tail defaults are written into the record on every boot
 * \note            Record +0x2E = 0x0170 (the firmware version the VCU reports),
 *                      +0xD0/+0xD2/+0xD4 = 0, +0xAE = 0, +0x1D0 = 1; the node 0x02 push state is
 *                      cleared and the CAN 0x100 frame counter (history +0x1C) zeroed. Before them
 *                      the faultmon UV pre-arm counter (g_lowglob.fm_uv_cnt) is zeroed.
 */
static void test_init_tail_defaults(void) {
    vcu_config_init();
    g_in2.f.serial_mark = 2u;
    g_in2.f.trip_dist = 3;
    g_in2.f.trip_total_time = 4;
    g_in2.f.trip_ride_time = 5;
    g_in2.f.speed_src = 6u;
    g_in2.rec_1d0 = 7u;
    g_lowglob.fm_uv_cnt = 123u;
    g_node2_push.state = NODE2_PUSH_DONE;
    g_telem_frame[REPORT_TELEM_TX100_LO] = 0x1234u;
    g_telem_frame[REPORT_TELEM_TX100_HI] = 0x5678u;
    vcu_config_init();
    TEST_ASSERT_EQUAL_HEX16(0x0170u, g_in2.f.serial_mark);
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_IDLE, g_node2_push.state);
    TEST_ASSERT_EQUAL_HEX16(0u, g_telem_frame[REPORT_TELEM_TX100_LO]);
    TEST_ASSERT_EQUAL_HEX16(0u, g_telem_frame[REPORT_TELEM_TX100_HI]);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.trip_dist);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.trip_total_time);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.trip_ride_time);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.speed_src);
    TEST_ASSERT_EQUAL_HEX16(1u, g_in2.rec_1d0);
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.fm_uv_cnt);
}

/**
 * \brief           Boot init asks sub-node 0x02 for its component record: 0x18EF0216 `01 01 DA 14 00`
 * \note            report_ninebot(0x16, 2, 2, 1, 0xDA, body) with the count 0x14 as two body
 *                      bytes. In the running system one 0x18EF0216 frame reaches the bus (from the FSM
 *                      re-run; the boot one is dropped with the ring before CAN1 is up). Here the
 *                      ring is pumped right after the call.
 */
static void test_init_requests_sub02_component(void) {
    static const uint8_t expect[SUB02_REQ_DLC] = {0x01u, 0x01u, 0xDAu, 0x14u, 0x00u};
    uint32_t found = 0u;
    uint32_t idx;

    vcu_config_init();
    vcu_can_poll();
    for (idx = 0u; idx < tst_can_tx_count(); ++idx) {
        const tst_can_frame_t* frame = tst_can_tx_get(idx);

        if ((frame != NULL) && (frame->ext != 0u) && (frame->id == SUB02_REQ_ID)) {
            ++found;
            TEST_ASSERT_EQUAL_UINT8(SUB02_REQ_DLC, frame->dlc);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame->data, SUB02_REQ_DLC);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(1u, found);
}

/**
 * \brief           With no valid record, the factory template + default serial/model is stored
 * \note            magic != 0x505C -> write the 0x200-byte factory template, serial
 *                      "03SP0000000001" to +0x20, model "Z03Q000000000001" to +0x40, then
 *                      identity_store. Both pages erased: the loader
 *                      rewrites page A (version 0), the defaults go to page B (version 1). Then
 *                      the validators run on the defaults: gear 2 -> VS+1 = 2, serial 'P' ->
 *                      VS+0 = 0, level 5, and the full serial match latches IN1+0x24.
 */
static void test_init_not_paired_stores_factory_record(void) {
    uint8_t expect[REC_SIZE];
    const uint8_t* page = (const uint8_t*)PAGE_B;
    uint32_t idx;

    memset(expect, 0, sizeof(expect));
    for (idx = 0u; idx < (sizeof(FACTORY_CELLS) / sizeof(FACTORY_CELLS[0])); ++idx) {
        memcpy(&expect[FACTORY_CELLS[idx].offset], &FACTORY_CELLS[idx].value, sizeof(uint32_t));
    }
    memcpy(&expect[REC_SERIAL_OFF], "03SP0000000001", SERIAL_LEN);
    memcpy(&expect[REC_MODEL_OFF], "Z03Q000000000001", MODEL_LEN);

    TEST_ASSERT_TRUE(tst_ota_flash_raw_erase(PAGE_A));
    TEST_ASSERT_TRUE(tst_ota_flash_raw_erase(PAGE_B));
    vcu_config_init();

    TEST_ASSERT_EQUAL_HEX16(1u, *(const volatile uint16_t*)(PAGE_B + REC_VERSION_OFF));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, page, REC_CHECKSUM_OFF);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&expect[REC_CHECKSUM_OFF + 4u], &page[REC_CHECKSUM_OFF + 4u],
                                 REC_VERSION_OFF - (REC_CHECKSUM_OFF + 4u));

    TEST_ASSERT_EQUAL_UINT8(MODE_DEFAULT, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.region);
    TEST_ASSERT_EQUAL_INT16(LEVEL_DEFAULT, g_in2.f.auto_off_min_s);
    TEST_ASSERT_EQUAL_UINT16(1111u, g_in2.f.passcode_num);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.dtc23_in);
}

/* -- validators ---------------------------------------------------------------------------- */

/**
 * \brief           Level (record +0x92) is kept in 1..30, anything else becomes 5
 * \note            A signed value below 1 or above 0x1E becomes 5.
 */
static void test_validate_level_range(void) {
    static const int16_t input[] = {-1, 0, 1, 17, 30, 31, 0x7FFF};
    static const int16_t output[] = {5, 5, 1, 17, 30, 5, 5};
    uint32_t idx;

    for (idx = 0u; idx < (sizeof(input) / sizeof(input[0])); ++idx) {
        g_in2.f.auto_off_min_s = input[idx];
        config_validate_level();
        TEST_ASSERT_EQUAL_INT16(output[idx], g_in2.f.auto_off_min_s);
    }
}

/**
 * \brief           Ride mode VS+1 = low byte of record +0xB4, kept in {2, 3, 4}, else 2
 * \note            VS+1 takes the low byte of record +0xB4, then anything not 2/3/4 -> 2.
 *                      Only the low byte counts (0x0103 -> 3).
 */
static void test_validate_ride_mode_set(void) {
    static const uint16_t input[] = {2u, 3u, 4u, 0u, 1u, 5u, 0x0103u, 0x0204u, 0xFF02u, 0x00FFu};
    static const uint8_t output[] = {2u, 3u, 4u, 2u, 2u, 2u, 3u, 4u, 2u, 2u};
    uint32_t idx;

    for (idx = 0u; idx < (sizeof(input) / sizeof(input[0])); ++idx) {
        g_in2.f.gear = (int16_t)input[idx];
        config_validate_ride_mode();
        TEST_ASSERT_EQUAL_UINT8(output[idx], g_vs.gear_sel);
    }
}

/**
 * \brief           Model "03S" + D/E/P selects drive row 2/1/0; anything else row 0 and IN1+0x24 = 1
 * \note            Family check on bytes 0..2, then D -> 2, E -> 1, P -> 0,
 *                      other -> 0 with the latch; the latch is not touched for a valid model.
 */
static void test_parse_model_variants(void) {
    static const char* const model[] = {"03SD", "03SE", "03SP", "03SX", "04SD", "03TD", "0"};
    static const uint8_t row[] = {2u, 1u, 0u, 0u, 0u, 0u, 0u};
    static const uint8_t latch[] = {0u, 0u, 0u, 1u, 1u, 1u, 1u};
    uint32_t idx;

    for (idx = 0u; idx < (sizeof(row) / sizeof(row[0])); ++idx) {
        g_in1.dtc23_in = 0u;
        g_vs.region = 0x55u;
        config_parse_model(model[idx]);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(row[idx], g_vs.region, model[idx]);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(latch[idx], g_in1.dtc23_in, model[idx]);
    }
}

/**
 * \brief           Serial commit: 1 for all 14 template bytes, 2 for a matching prefix, 0 on mismatch
 * \note            Parse the serial as a model, walk it while alphanumeric against
 *                      "03SP0000000001": diverge -> 0 (no latch); index > 0xD -> 1;
 *                      non-alphanumeric stop -> 2; 1 and 2 set IN1+0x24.
 *                      A serial that starts with a non-alphanumeric byte stops at once: 2.
 */
static void test_serial_commit_results(void) {
    g_in1.dtc23_in = 0u;
    prv_set_serial("03SP0000000001");
    TEST_ASSERT_EQUAL_INT(1, config_serial_commit());
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.dtc23_in);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.region);

    g_in1.dtc23_in = 0u;
    prv_set_serial("03SP00");
    TEST_ASSERT_EQUAL_INT(2, config_serial_commit());
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.dtc23_in);

    g_in1.dtc23_in = 0u;
    prv_set_serial("03SP0000000002");
    TEST_ASSERT_EQUAL_INT(0, config_serial_commit());
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.dtc23_in);

    g_in1.dtc23_in = 0u;
    prv_set_serial("03SD0000000001");
    TEST_ASSERT_EQUAL_INT(0, config_serial_commit());
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.dtc23_in);
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.region);

    prv_set_serial("");
    TEST_ASSERT_EQUAL_INT(2, config_serial_commit());
}

/* -- startup FSM (vcu_startup_poll) ------------------------------------------------------- */

/**
 * \brief           The counter ramps to 1000; at 999 PB12 is released and one long beep starts
 * \note            Below 1000 the counter increments; on the step from 999 the startup GPIO
 *                      (GPIOB pin 12) is reset and buzzer_beep(BEEP_LONG) starts.
 */
static void test_startup_ramp_releases_pb12_at_999(void) {
    uint16_t counter = 998u;

    vcu_gpio_power_latch_init();
    gpio_bits_set(GPIOB, GPIO_PINS_12);
    vcu_startup_poll(&counter);
    TEST_ASSERT_EQUAL_UINT16(999u, counter);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_12, GPIOB->odt & GPIO_PINS_12);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.beep_seq_id);

    vcu_startup_poll(&counter);
    TEST_ASSERT_EQUAL_UINT16(STARTUP_RAMP_END, counter);
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_12);
    TEST_ASSERT_EQUAL_UINT8(1u, g_lowglob.beep_seq_id); /* BEEP_LONG */
}

/**
 * \brief           At 1000 with PA10 high (wake line inactive): 500 ms, PB12 set, counter parked 0x4B0
 * \note            PA10 inactive and n < 0x44C -> config_delay_ms(500), PB12 set,
 *                      n = 0x4B0.
 */
static void test_startup_relatch_when_pa10_inactive(void) {
    uint16_t counter = STARTUP_RAMP_END;

    vcu_gpio_power_latch_init();
    gpio_bits_reset(GPIOB, GPIO_PINS_12);
    prv_pa10_pull(GPIO_PULL_UP);
    vcu_startup_poll(&counter);
    TEST_ASSERT_EQUAL_UINT16(STARTUP_PARKED, counter);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_12, GPIOB->odt & GPIO_PINS_12);

    vcu_startup_poll(&counter); /* parked: 0x4B0 >= 0x44C */
    TEST_ASSERT_EQUAL_UINT16(STARTUP_PARKED, counter);
}

/**
 * \brief           No re-latch while PA10 is low (active) or once the counter reached 0x44C
 * \note            Both conditions are required.
 */
static void test_startup_no_relatch(void) {
    uint16_t counter = STARTUP_RAMP_END;

    vcu_gpio_power_latch_init();
    gpio_bits_reset(GPIOB, GPIO_PINS_12);
    prv_pa10_pull(GPIO_PULL_DOWN);
    vcu_startup_poll(&counter);
    TEST_ASSERT_EQUAL_UINT16(STARTUP_RAMP_END, counter);
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_12);

    prv_pa10_pull(GPIO_PULL_UP);
    counter = STARTUP_WAIT_MAX;
    vcu_startup_poll(&counter);
    TEST_ASSERT_EQUAL_UINT16(STARTUP_WAIT_MAX, counter);
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_12);
}

/* -- outputs / delays ---------------------------------------------------------------------- */

/**
 * \brief           Output reset drives PB13, PA6, PB14 low and clears SCR+0x42
 * \note            control_out_pb13(0), control_out_pa6(0), control_out_pb14(0), then
 *                      SCR+0x42 = 0.
 */
static void test_outputs_reset(void) {
    vcu_gpio_indicator_outputs_init();
    gpio_bits_set(GPIOB, GPIO_PINS_13 | GPIO_PINS_14);
    gpio_bits_set(GPIOA, GPIO_PINS_6);
    g_scr.submode_code = 0x77u;
    vcu_outputs_reset();
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & (GPIO_PINS_13 | GPIO_PINS_14));
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOA->odt & GPIO_PINS_6);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.submode_code);
}

/**
 * \brief           The ms delay runs SysTick in chunks of 50 ms; the us delay is one run of n us
 * \note            config_delay_ms: LOAD = fac_ms * n for n <= 50, else fac_ms * 50 per chunk,
 *                      then wait COUNTFLAG, ENABLE cleared, VAL = 0. 120 ms = 50 + 50 + 20, so the
 *                      last reload is fac_ms * 20. config_delay_ticks: LOAD = fac_us * n.
 */
static void test_delays_systick_chunks(void) {
    uint32_t fac_ms = SystemCoreClock / 1000u;

    config_delay_ms(120u);
    TEST_ASSERT_EQUAL_UINT32(fac_ms * 20u, SysTick->LOAD);
    TEST_ASSERT_EQUAL_HEX32(0u, SysTick->CTRL & SysTick_CTRL_ENABLE_Msk);
    TEST_ASSERT_EQUAL_UINT32(0u, SysTick->VAL);

    config_delay_ms(51u);
    TEST_ASSERT_EQUAL_UINT32(fac_ms * 1u, SysTick->LOAD);

    config_delay_ticks(0x78u);
    TEST_ASSERT_EQUAL_UINT32((SystemCoreClock / 1000000u) * 0x78u, SysTick->LOAD);
    TEST_ASSERT_EQUAL_HEX32(0u, SysTick->CTRL & SysTick_CTRL_ENABLE_Msk);
}

/* -- bring-up / standby (run last: they start TMR5, the ERTC wakeup and the IWDG) ---------- */

/**
 * \brief           Smoke test of the peripheral bring-up: output levels, TMR5 period, IWDG, NVIC group
 * \note            PRIGROUP 5, PC14 set, PA10 / PA9 pull-up inputs, PB12 set, PF6 reset, PB10 and
 *                      PB0 set, PB13 / PA6 / PB14 reset, TMR5 PR 9 running, IWDG /256 with reload
 *                      0x4E1.
 */
static void test_periph_init_smoke(void) {
    vcu_periph_init();
    wdt_counter_reload();
    TEST_ASSERT_EQUAL_HEX32(0x500u, SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_14, GPIOC->odt & GPIO_PINS_14);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_9 | GPIO_PINS_10, GPIOA->idt & (GPIO_PINS_9 | GPIO_PINS_10));
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_12 | GPIO_PINS_10 | GPIO_PINS_0,
                            GPIOB->odt & (GPIO_PINS_12 | GPIO_PINS_10 | GPIO_PINS_0));
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOF->odt & GPIO_PINS_6);
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & (GPIO_PINS_13 | GPIO_PINS_14));
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOA->odt & GPIO_PINS_6);
    TEST_ASSERT_EQUAL_UINT32(9u, TMR5->pr);
    TEST_ASSERT_EQUAL_UINT32(14399u, TMR5->div); /* 144 MHz timer clock / 14400 = 10 kHz */
    TEST_ASSERT_EQUAL_HEX32(1u, TMR5->ctrl1 & 1u);
    TEST_ASSERT_EQUAL_HEX32(WDT_CLK_DIV_256, WDT->div & 7u);
    TEST_ASSERT_EQUAL_HEX32(0x4E1u, WDT->rld & 0xFFFu);
}

/**
 * \brief           Bring-up tail: SWJ fully off when PA9 or PA12 reads high, else JTAG off / SWD kept
 * \note            GPIOA idt bit12 or bit9 set -> SWJTAG_GMUX_100, else SWJTAG_GMUX_010. After
 *                      the bring-up PA9 (pull-up) reads high, so the debug port is switched off.
 */
static void test_periph_init_swj_off(void) {
    vcu_periph_init();
    wdt_counter_reload();
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_9, GPIOA->idt & GPIO_PINS_9);
    TEST_ASSERT_EQUAL_HEX32(0x4u, IOMUX->remap7_bit.swjtag_gmux);
}

/**
 * \brief           Smoke test of the standby cycle: clock snapshots, status cleared, wake lines masked
 * \note            CRM cfg -> VS+0x60, CRM ctrl -> VS+0x5C, VS+0x08 = 0, PWC vrsel set,
 *                      SLEEPDEEP + WFI (cleared after), EXINT lines 3/10/13 masked. TMR5 (started
 *                      by the bring-up test before) wakes the WFI.
 */
static void test_standby_cycle_smoke(void) {
    uint32_t cfg = CRM->cfg;
    uint32_t ctrl = CRM->ctrl;

    g_vs.wake_src = 0x55u;
    (void)vcu_standby_cycle();
    wdt_counter_reload();
    TEST_ASSERT_EQUAL_HEX32(cfg, g_vs.crm_cfg_save);
    TEST_ASSERT_EQUAL_HEX32(ctrl, g_vs.crm_ctrl_save);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.wake_src);
    TEST_ASSERT_EQUAL_HEX32(0u, SCB->SCR & SCB_SCR_SLEEPDEEP_Msk);
    TEST_ASSERT_EQUAL_HEX32(0u, EXINT->inten & (EXINT_LINE_3 | EXINT_LINE_10 | EXINT_LINE_13));
    TEST_ASSERT_EQUAL_HEX32(1u, PWC->ctrl & 1u);
}

/*
 * Peripheral state after vcu_periph_init (preceded in this suite by the bring-up tests and one
 * standby cycle): the GPIO modes and levels, the IOMUX remaps and EXINT routing, the EXINT and
 * NVIC lines and priorities, TMR1 (buzzer) / TMR5 (tick), SPI2, CAN1, ADC1 + DMA1 ch1, the
 * USART2 DMA channels, the clock tree, USART2, the ERTC wakeup and the IWDG. The DMA memory
 * addresses are RAM symbol addresses and are not pinned.
 */
static const reg_expect_t PERIPH_REGS[] = {
    REG(GPIOA->cfglr, 0x01004900u),
    REG(GPIOA->cfghr, 0x14444889u),
    REG(GPIOA->odt, 0x00008608u),
    REG(GPIOB->cfglr, 0x88989441u),
    REG(GPIOB->cfghr, 0x11114198u),
    REG(GPIOB->odt, 0x000015D1u),
    REG(GPIOC->cfglr, 0x44444444u),
    REG(GPIOC->cfghr, 0x41844444u),
    REG(GPIOC->odt, 0x00006000u),
    REG(GPIOD->cfglr, 0x44444444u),
    REG(GPIOD->cfghr, 0x44444444u),
    REG(GPIOD->odt, 0x00000000u),
    REG(GPIOF->cfglr, 0x11444444u),
    REG(GPIOF->cfghr, 0x44444444u),
    REG(GPIOF->odt, 0x00000080u),
    REG(IOMUX->remap, 0x00000000u),
    REG(IOMUX->exintc1, 0x00000000u),
    REG(IOMUX->exintc2, 0x00000000u),
    REG(IOMUX->exintc3, 0x00000000u),
    REG(IOMUX->exintc4, 0x00000000u),
    REG(IOMUX->remap2, 0x00000000u),
    REG(IOMUX->remap3, 0x00000000u),
    REG(IOMUX->remap4, 0x00000000u),
    REG(IOMUX->remap5, 0x00100000u),
    REG(IOMUX->remap6, 0x00000002u),
    REG(IOMUX->remap7, 0x00040000u),
    REG(IOMUX->remap8, 0x00000000u),
    REG(EXINT->inten, 0x00400000u),
    REG(EXINT->evten, 0x00000000u),
    REG(EXINT->polcfg1, 0x00400000u),
    REG(EXINT->polcfg2, 0x00000408u),
    REG(NVIC->ISER[0], 0x00500208u),
    REG(NVIC->ISER[1], 0x00040100u),
    REGW(NVIC->IP[0], 0x10000000u),
    REGW(NVIC->IP[4], 0x00000000u),
    REGW(NVIC->IP[8], 0x00004000u),
    REGW(NVIC->IP[12], 0x00000000u),
    REGW(NVIC->IP[16], 0x00000000u),
    REGW(NVIC->IP[20], 0x00000000u),
    REGW(NVIC->IP[24], 0x00000000u),
    REGW(NVIC->IP[28], 0x00000000u),
    REGW(NVIC->IP[32], 0x00000000u),
    REGW(NVIC->IP[36], 0x00000000u),
    REGW(NVIC->IP[40], 0x00000040u),
    REGW(NVIC->IP[44], 0x00000000u),
    REGW(NVIC->IP[48], 0x00400000u),
    REGW(NVIC->IP[52], 0x00000000u),
    REGW(NVIC->IP[56], 0x00000000u),
    REGW(NVIC->IP[60], 0x00000000u),
    REG(SCB->AIRCR, 0xFA050500u),
    REG(TMR1->ctrl1, 0x00000001u),
    REG(TMR1->ctrl2, 0x00000100u),
    REG(TMR1->iden, 0x00000000u),
    REG(TMR1->cm1, 0x00000060u),
    REG(TMR1->cm2, 0x00000000u),
    REG(TMR1->cctrl, 0x00000000u),
    REG(TMR1->pr, 0x00000001u),
    REG(TMR1->div, 0x0000008Fu),
    REG(TMR1->c1dt, 0x00000000u),
    REG(TMR1->brk, 0x00008000u),
    REG(TMR5->ctrl1, 0x00000001u),
    REG(TMR5->iden, 0x00000001u),
    REG(TMR5->pr, 0x00000009u),
    REG(TMR5->div, 0x0000383Fu),
    REG(SPI2->ctrl1, 0x00000357u),
    REG(SPI2->ctrl2, 0x00000000u),
    REG(SPI2->i2sctrl, 0x00000000u),
    REG(CAN1->mctrl, 0x00010020u),
    REG(CAN1->btmg, 0x013A0008u),
    REG(CAN1->inten, 0x00008C02u),
    REG(CAN1->fctrl, 0x00000000u),
    REG(CAN1->fmcfg, 0x00000000u),
    REG(CAN1->fbwcfg, 0x00000001u),
    REG(CAN1->frf, 0x00000000u),
    REG(CAN1->facfg, 0x00000001u),
    REG(CAN1->ffb[0].ffdb1, 0x00000000u),
    REG(CAN1->ffb[0].ffdb2, 0x00000000u),
    REG(ADC1->ctrl1, 0x00000100u),
    REG(ADC1->ctrl2, 0x001E0103u),
    REG(ADC1->spt1, 0x00000000u),
    REG(ADC1->spt2, 0x00E3F03Fu),
    REG(ADC1->osq1, 0x00400000u),
    REG(ADC1->osq2, 0x00000000u),
    REG(ADC1->osq3, 0x00729020u),
    REG(DMA1_CHANNEL1->ctrl, 0x000015A1u),
    REG(DMA1_CHANNEL1->paddr, 0x4001244Cu),
    REG(DMA1_CHANNEL6->ctrl, 0x000020A1u),
    REG(DMA1_CHANNEL6->paddr, 0x40004404u),
    REG(DMA1_CHANNEL7->ctrl, 0x00002090u),
    REG(DMA1_CHANNEL7->dtcnt, 0x00000000u),
    REG(DMA1_CHANNEL7->paddr, 0x40004404u),
    REG(CRM->cfg, 0x001DA40Au),
    REG(CRM->pll, 0x04001F10u),
    REG(CRM->misc1, 0x00100000u),
    REG(CRM->apb2en, 0x00000A9Du),
    REG(CRM->apb1en, 0x12024008u),
    REG(CRM->ahben, 0x00000015u),
    REG(CRM->bpdc, 0x00008200u),
    REG(USART2->baudr, 0x00000271u),
    REG(USART2->ctrl1, 0x0000200Cu),
    REG(USART2->ctrl2, 0x00000000u),
    REG(USART2->ctrl3, 0x000000C0u),
    REG(ERTC->ctrl, 0x00004404u),
    REG(ERTC->div, 0x0063018Fu),
    REG(ERTC->wat, 0x00000002u),
    REG(ERTC->wp, 0x000000FFu),
    REG(PWC->ctrl, 0x00000101u),
    REG(WDT->div, 0x00000006u),
    REG(WDT->rld, 0x000004E1u),
};

/* The registers the standby cycle changes or re-programs, after one more cycle: PA3 / PA10 /
   PC13 inputs, EXINT routing and lines, NVIC, CAN1 back at its reset values, the clock tree. */
static const reg_expect_t STANDBY_REGS[] = {
    REG(GPIOA->cfglr, 0x01008900u),       REG(GPIOA->cfghr, 0x14444489u),       REG(GPIOA->odt, 0x00008608u),
    REG(GPIOC->cfglr, 0x44444444u),       REG(GPIOC->cfghr, 0x41844444u),       REG(GPIOC->odt, 0x00006000u),
    REG(IOMUX->remap, 0x00000000u),       REG(IOMUX->exintc1, 0x00000000u),     REG(IOMUX->exintc2, 0x00000000u),
    REG(IOMUX->exintc3, 0x00000000u),     REG(IOMUX->exintc4, 0x00000000u),     REG(IOMUX->remap2, 0x00000000u),
    REG(IOMUX->remap3, 0x00000000u),      REG(IOMUX->remap4, 0x00000000u),      REG(IOMUX->remap5, 0x00100000u),
    REG(IOMUX->remap6, 0x00000002u),      REG(IOMUX->remap7, 0x00040000u),      REG(IOMUX->remap8, 0x00000000u),
    REG(EXINT->inten, 0x00400000u),       REG(EXINT->evten, 0x00000000u),       REG(EXINT->polcfg1, 0x00400000u),
    REG(EXINT->polcfg2, 0x00000408u),     REG(NVIC->ISER[0], 0x00500208u),      REG(NVIC->ISER[1], 0x00040100u),
    REGW(NVIC->IP[0], 0x10000000u),       REGW(NVIC->IP[4], 0x00000000u),       REGW(NVIC->IP[8], 0x00004000u),
    REGW(NVIC->IP[12], 0x00000000u),      REGW(NVIC->IP[16], 0x00000000u),      REGW(NVIC->IP[20], 0x00000000u),
    REGW(NVIC->IP[24], 0x00000000u),      REGW(NVIC->IP[28], 0x00000000u),      REGW(NVIC->IP[32], 0x00000000u),
    REGW(NVIC->IP[36], 0x00000000u),      REGW(NVIC->IP[40], 0x00000040u),      REGW(NVIC->IP[44], 0x00000000u),
    REGW(NVIC->IP[48], 0x00400000u),      REGW(NVIC->IP[52], 0x00000000u),      REGW(NVIC->IP[56], 0x00000000u),
    REGW(NVIC->IP[60], 0x00000000u),      REG(CAN1->mctrl, 0x00010002u),        REG(CAN1->btmg, 0x01230000u),
    REG(CAN1->inten, 0x00000000u),        REG(CAN1->fctrl, 0x00000000u),        REG(CAN1->fmcfg, 0x00000000u),
    REG(CAN1->fbwcfg, 0x00000000u),       REG(CAN1->frf, 0x00000000u),          REG(CAN1->facfg, 0x00000000u),
    REG(CAN1->ffb[0].ffdb1, 0x00000000u), REG(CAN1->ffb[0].ffdb2, 0x00000000u), REG(CRM->cfg, 0x001DA40Au),
    REG(CRM->pll, 0x04001F10u),           REG(CRM->misc1, 0x00100000u),         REG(PWC->ctrl, 0x00000101u),
};

/**
 * \brief           Assert every register of a register map
 * \param[in]       map: the expected registers
 * \param[in]       count: number of entries
 */
static void prv_assert_regs(const reg_expect_t* map, uint32_t count) {
    uint32_t idx;

    for (idx = 0u; idx < count; idx++) {
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(map[idx].value, *map[idx].reg, map[idx].name);
    }
}

/**
 * \brief           The whole peripheral register map after the bring-up
 * \note            Every init the bring-up runs ends in the values of \ref PERIPH_REGS.
 */
static void test_periph_init_register_map(void) {
    vcu_periph_init();
    wdt_counter_reload();
    prv_assert_regs(PERIPH_REGS, sizeof(PERIPH_REGS) / sizeof(PERIPH_REGS[0]));
}

/**
 * \brief           The registers a standby cycle re-programs
 * \note            The EXINT re-arm, the CAN1 reset and the clock resume end in \ref STANDBY_REGS.
 */
static void test_standby_register_map(void) {
    (void)vcu_standby_cycle();
    wdt_counter_reload();
    prv_assert_regs(STANDBY_REGS, sizeof(STANDBY_REGS) / sizeof(STANDBY_REGS[0]));
}

/** \brief The ERTC wakeup ISR (config.c; a vector, so no header declares it) */
void ERTC_WKUP_IRQHandler(void);

/**
 * \brief           A wakeup-timer expiry latches the wake flag and clears WATF and EXINT22
 * \note            The bring-up's ERTC wakeup timer (CK_B, counter 2) is switched to ERTCCLK / 2
 *                      with counter 1 for one expiry, then restored. ERTC_WKUP_IRQHandler sets
 *                      g_vs +0x07, clears sts WATF (imen kept) and the EXINT line 22 pending bit.
 */
static void test_ertc_wakeup_isr(void) {
    g_vs.ertc_wake = 0u;
    ERTC->wp = 0xCAu;
    ERTC->wp = 0x53u;
    ERTC->ctrl_bit.waten = FALSE;
    ERTC->ctrl_bit.watclk = ERTC_WAT_CLK_ERTCCLK_DIV2;
    ERTC->wat = 1u;
    ERTC->ctrl_bit.waten = TRUE;
    tst_delay_us(1000u);
    ERTC->ctrl_bit.waten = FALSE;
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.ertc_wake);
    TEST_ASSERT_EQUAL_HEX32(0u, ERTC->sts & ERTC_WATF_FLAG);
    TEST_ASSERT_EQUAL_HEX32(0u, ERTC->sts & 0x80u); /* imen not set by the flag clear */
    TEST_ASSERT_EQUAL_HEX32(0u, EXINT->intsts & EXINT_LINE_22);

    g_vs.ertc_wake = 0u;
    ERTC_WKUP_IRQHandler(); /* no WATF: nothing latched */
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.ertc_wake);

    ERTC->ctrl_bit.watclk = ERTC_WAT_CLK_CK_B_16BITS;
    ERTC->wat = 2u;
    ERTC->ctrl_bit.waten = TRUE;
    ERTC->wp = 0xFFu;
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_init_clears_in1);
    RUN_TEST(test_init_scr_warm_boot_guard);
    RUN_TEST(test_init_tail_defaults);
    RUN_TEST(test_init_requests_sub02_component);
    RUN_TEST(test_init_not_paired_stores_factory_record);
    RUN_TEST(test_validate_level_range);
    RUN_TEST(test_validate_ride_mode_set);
    RUN_TEST(test_parse_model_variants);
    RUN_TEST(test_serial_commit_results);
    RUN_TEST(test_startup_ramp_releases_pb12_at_999);
    RUN_TEST(test_startup_relatch_when_pa10_inactive);
    RUN_TEST(test_startup_no_relatch);
    RUN_TEST(test_outputs_reset);
    RUN_TEST(test_delays_systick_chunks);
    RUN_TEST(test_periph_init_smoke);
    RUN_TEST(test_periph_init_swj_off);
    RUN_TEST(test_standby_cycle_smoke);
    RUN_TEST(test_periph_init_register_map);
    RUN_TEST(test_standby_register_map);
    RUN_TEST(test_ertc_wakeup_isr);
    tst_exit(UNITY_END());
    return 0;
}
