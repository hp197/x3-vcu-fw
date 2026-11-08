/**
 * \file            test_ridemode.c
 * \brief           Ride mode, gear events, rear light and LED matrix (ridemode.c)
 * \note            Covers the rear-light mode, the sub-mode dispatcher, the gear/mode event
 *                      tick, the LED matrix and the rear-light clear. The tests describe the
 *                      gear transitions and their permissions only; no speed limit is set or
 *                      changed here.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "buzzer.h"
#include "can.h"
#include "control.h"
#include "faultmon.h"
#include "report.h"
#include "ridemode.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"
#include "vehicle.h"

#define GEAR_REPORT_ID  0x18EF0716u /*!< report_ninebot(0x16, 7, ..): prio 6, PF 0xEF, dst 7, src 0x16 */
#define REPORT_RELOAD   300u        /*!< VS+0x36 reload of control_mode_report_fsm */
#define DWELL_RELOAD    300u        /*!< sub-mode dwell reload (0x012C) */
#define LEDM_WORD_AUX_A 21u         /*!< matrix word +0x2A */
#define LEDM_WORD_AUX_B 22u         /*!< matrix word +0x2C */

/** \brief Rear-light mode \return MODE+0x10 bits 9..11 */
static uint16_t prv_light(void) { return (uint16_t)((g_mode.enable_flags & MODE_LIGHT_MASK) >> MODE_LIGHT_SHIFT); }

/** \brief LED matrix word \param[in] word: index from g_in2+0x36 \return its value */
static uint16_t prv_ledm(uint32_t word) { return ((volatile uint16_t*)(((volatile uint8_t*)&g_in2) + 0x36u))[word]; }

/**
 * \brief           Preset an LED matrix word
 * \param[in]       word: index from g_in2+0x36
 * \param[in]       value: new value
 */
static void prv_ledm_set(uint32_t word, uint16_t value) {
    ((volatile uint16_t*)(((volatile uint8_t*)&g_in2) + 0x36u))[word] = value;
}

/** \brief PB13 output latch \return 1 when set */
static uint32_t prv_pb13(void) { return (GPIOB->odt >> 13) & 1u; }

/** \brief PA6 output latch \return 1 when set */
static uint32_t prv_pa6(void) { return (GPIOA->odt >> 6) & 1u; }

/** \brief Drive both indicator latches low (PB13, PA6) */
static void prv_outputs_low(void) {
    gpio_bits_reset(GPIOB, GPIO_PINS_13);
    gpio_bits_reset(GPIOA, GPIO_PINS_6);
}

/**
 * \brief           Press gear up or down once with gear changes permitted
 * \param[in]       mon: FM_MON_GEAR_UP or FM_MON_GEAR_DOWN
 * \param[in]       gear: gear before the press
 * \return          gear after the press
 */
static uint8_t prv_gear_press(uint32_t mon, uint8_t gear) {
    g_vs.gear_sel = gear;
    g_in2.f.status_flags = SF_SESSION;
    g_evt.mon[mon].code = 1u;
    ridemode_gear_event_tick();
    return g_vs.gear_sel;
}

/**
 * \brief           Fresh boot state, both indicator latches low, CAN frames captured only
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    prv_outputs_low();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- rear-light mode ------------------------------------------------------------------------- */

/**
 * \brief           Regen without a brake lever: tail light; with a brake: brake or flashing brake
 * \note            IN1+0x26 == 1 -> IN1+0x0B == 0 -> `& 0xF1FF + 0x400`;
 *                      else IN2+0xBA != 0 -> `+ 0x800`, else `+ 0x600`.
 */
static void test_light_regen(void) {
    g_in1.regen_active = 1u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_TAIL, prv_light());

    g_in1.brake_any = 1u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE, prv_light());

    g_in2.f.brake_light_flash = 1u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE_FLASH, prv_light());
}

/**
 * \brief           A brake lever without regen: brake light, or the flashing one with reg 0x5D
 * \note            IN1+0x0B != 0 -> IN2+0xBA ? 0x800 : 0x600.
 */
static void test_light_brake(void) {
    g_in1.brake_any = 1u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE, prv_light());

    g_in2.f.brake_light_flash = 0x0101u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE_FLASH, prv_light());
}

/**
 * \brief           No demand: off when PB13 and PA6 are both low, tail light when either is set
 * \note            GPIOB odt bit 13 (0x40010C0C) and GPIOA odt bit 6 (0x4001080C) both clear ->
 *                      `& 0xF1FF`, else `+ 0x400`.
 */
static void test_light_idle_follows_indicator_latches(void) {
    mode_light_set(MODE_LIGHT_BRAKE);
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_OFF, prv_light());

    gpio_bits_set(GPIOB, GPIO_PINS_13);
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_TAIL, prv_light());

    prv_outputs_low();
    gpio_bits_set(GPIOA, GPIO_PINS_6);
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_TAIL, prv_light());
}

/**
 * \brief           A latched error code leaves the light to the error blink
 * \note            active_error_code != 0 (no charger) -> return without a store.
 */
static void test_light_error_code_untouched(void) {
    g_lowglob.active_error_code = 23u;
    mode_light_set(MODE_LIGHT_BRAKE_FLASH);
    g_in1.brake_any = 1u;

    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE_FLASH, prv_light());
}

/**
 * \brief           Charger connected: blink until the charge is complete, then steady; off without display
 * \note            IN1+0x25 != 0 -> IN2+0x3E bit 1 -> VS chg_full (packed VS+0x18 bit2) ?
 *                      `+ 0x600` : `+ 0x200`; bit 1 clear -> `& 0xF1FF`. The SoC does not matter.
 */
static void test_light_charging(void) {
    g_in1.charger_connected = 1u;
    g_in2.f.chg_cfg = CHG_DISP_EN;
    g_in2.f.soc_s = 100;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BLINK, prv_light());

    g_vs.chg_full = 1u;
    g_in2.f.soc_s = 50;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE, prv_light());

    g_in2.f.chg_cfg = 0u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_OFF, prv_light());
}

/**
 * \brief           The regen release turns the light off even when the error branch stores nothing
 * \note            led_submode == 1 && IN1+0x26 == 0 -> `& 0xF1FF` before the main branches;
 *                      then led_submode mirrors IN1+0x26.
 */
static void test_light_regen_release_clears(void) {
    g_in1.regen_active = 1u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_TAIL, prv_light());

    g_lowglob.active_error_code = 7u;
    g_in1.regen_active = 0u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_OFF, prv_light());
}

/**
 * \brief           The light field is bits 9..11 only; the rest of MODE+0x10 is kept
 * \note            ridemode_status_word_update / ridemode_led_indicator_clear: `& 0xF1FF` then
 *                      `+ (mode << 9)`.
 */
static void test_light_field_keeps_other_bits(void) {
    g_mode.enable_flags = 0xF1FFu;
    g_in1.brake_any = 1u;
    ridemode_status_word_update();
    TEST_ASSERT_EQUAL_HEX16(0xF7FFu, g_mode.enable_flags);

    ridemode_led_indicator_clear();
    TEST_ASSERT_EQUAL_HEX16(0xF1FFu, g_mode.enable_flags);
}

/* -- LED matrix ------------------------------------------------------------------------------ */

/**
 * \brief           Row 0 (Global), and any other row value: the Global annunciator pattern
 * \note            Common frame on words 0 / 0x15 / 0x16, then for `(char)VS+0 != 1 && != 2`:
 *                      |0x2000 &0xFFDF, &0xEFFF |0x8000, &0xFFDF |1 |4. The common frame also
 *                      clears bits 11..14 of word 0x16 (IN2+0x62) and stores word 0x15 (IN2+0x60)
 *                      = 0x20 before its bit sets; row 0 clears its bit 13.
 */
static void test_ledm_power_row(void) {
    prv_ledm_set(0u, 0u);
    prv_ledm_set(LEDM_WORD_AUX_A, 0u);
    prv_ledm_set(LEDM_WORD_AUX_B, 0xFFFFu);
    g_vs.region = VCU_REGION_GLOBAL;

    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x600Fu, prv_ledm(0u));
    TEST_ASSERT_EQUAL_HEX16(0x85E0u, prv_ledm(LEDM_WORD_AUX_A));
    TEST_ASSERT_EQUAL_HEX16(0x819Fu, prv_ledm(LEDM_WORD_AUX_B));

    g_vs.region = 7u;
    prv_ledm_set(0u, 0xFFFFu);
    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x600Fu, prv_ledm(0u));
}

/**
 * \brief           Row 1 (Europe) and row 2 (Germany) patterns
 * \note            Row 1: &0xDFFF &0xFFDF, |0x1000 &0x7FFF, &0xFFDF &0xFFFE &0xFFFB; row 2:
 *                      &0xDFFF |0x20, |0x1000 |0x8000, |0x20 &0xFFFE &0xFFFB. Word 0x15 also
 *                      |0x2000 in rows 1 and 2.
 */
static void test_ledm_sport_and_eco_rows(void) {
    prv_ledm_set(LEDM_WORD_AUX_B, 0u);
    g_vs.region = VCU_REGION_EUROPE;
    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x400Fu, prv_ledm(0u));
    TEST_ASSERT_EQUAL_HEX16(0x35E0u, prv_ledm(LEDM_WORD_AUX_A));
    TEST_ASSERT_EQUAL_HEX16(0x019Au, prv_ledm(LEDM_WORD_AUX_B));

    g_vs.region = VCU_REGION_GERMANY;
    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x402Fu, prv_ledm(0u));
    TEST_ASSERT_EQUAL_HEX16(0xB5E0u, prv_ledm(LEDM_WORD_AUX_A));
    TEST_ASSERT_EQUAL_HEX16(0x01BAu, prv_ledm(LEDM_WORD_AUX_B));
}

/**
 * \brief           Assist capability bits: regs 0x32 / 0x33 / 0x2F and reg 0x1E per row
 * \note            Common: reg 0x33 &~2 &~0x10, reg 0x2F &~1, reg 0x1E &~8; rows 1/2 (shared
 *                      tail): reg 0x32 &~4, reg 0x2F |4, reg 0x33 |1 |4 |8; row 0: reg 0x32 |4,
 *                      reg 0x33 &~1 &~4 &~8, reg 0x1E &~0x10 &~2 &~4.
 */
static void test_ledm_assist_caps(void) {
    g_in2.f.ride_caps_32 = 0u;
    g_in2.f.ride_caps_33 = 0xFFFFu;
    g_in2.f.in2_5e = 0xFFFFu;
    g_in2.f.cfg_flags2 = 0xFFFFu;
    g_vs.region = VCU_REGION_GLOBAL;
    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_in2.f.ride_caps_32);
    TEST_ASSERT_EQUAL_HEX16(0xFFE0u, g_in2.f.ride_caps_33);
    TEST_ASSERT_EQUAL_HEX16(0xFFFEu, g_in2.f.in2_5e);
    TEST_ASSERT_EQUAL_HEX16(0xFFE1u, g_in2.f.cfg_flags2);

    g_in2.f.ride_caps_32 = 0xFFFFu;
    g_in2.f.ride_caps_33 = 0u;
    g_in2.f.in2_5e = 0u;
    g_in2.f.cfg_flags2 = 0xFFFFu;
    g_vs.region = VCU_REGION_EUROPE;
    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0xFFFBu, g_in2.f.ride_caps_32);
    TEST_ASSERT_EQUAL_HEX16(0x000Du, g_in2.f.ride_caps_33);
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_in2.f.in2_5e);
    TEST_ASSERT_EQUAL_HEX16(0xFFF7u, g_in2.f.cfg_flags2);

    g_vs.region = VCU_REGION_GERMANY;
    g_in2.f.ride_caps_33 = 0x0012u;
    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x000Du, g_in2.f.ride_caps_33);
}

/**
 * \brief           Matrix words 1, 2 and 4..0x13 are not written; word 0x14 (reg 0x2F) only loses bit0
 * \note            The matrix stores go only to words 0, 0x15 and 0x16; the assist capability
 *                      bits add IN2+0x3C (word 3), IN2+0x5E (word 0x14), IN2+0x64 and IN2+0x66.
 */
static void test_ledm_other_words_untouched(void) {
    prv_ledm_set(1u, 0x1234u);
    prv_ledm_set(0x13u, 0x5678u);
    prv_ledm_set(0x14u, 0xABCDu);

    ridemode_led_matrix_update();
    TEST_ASSERT_EQUAL_HEX16(0x1234u, prv_ledm(1u));
    TEST_ASSERT_EQUAL_HEX16(0x5678u, prv_ledm(0x13u));
    TEST_ASSERT_EQUAL_HEX16(0xABCCu, prv_ledm(0x14u));
}

/* -- sub-mode dispatcher --------------------------------------------------------------------- */

/**
 * \brief           Sub-states 0..2 and regen select the output prep and the sub-mode code
 * \note            IN1+0x26 == 1 -> vehicle_lock_antitheft_latch (PB13 0, PA6 1), code 1;
 *                      VS+5 == 0 -> vehicle_event1_prep (PB13 0, PA6 0), code 0; 1 ->
 *                      vehicle_lock_antitheft_latch, code 1; 2 -> vehicle_event8_prep (PB13 1,
 *                      PA6 1), code 2. Code at SCR+0x42.
 */
static void test_submode_states_0_to_2(void) {
    g_vs.substate = 2u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(2u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_pb13());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_pa6());

    g_vs.substate = 1u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_pb13());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_pa6());

    g_vs.substate = 0u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_pa6());

    g_vs.substate = 2u;
    g_in1.regen_active = 1u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_pb13());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_pa6());
}

/**
 * \brief           Sub-state 3 needs IN2+0x3E bit 0, else it falls back to sub-state 0
 * \note            VS+5 == 3 and `(IN2+0x3E & 1) == 0` -> VS+5 = 0, no code.
 */
static void test_submode_3_needs_cfg_bit0(void) {
    g_vs.substate = 3u;
    g_scr.submode_code = 2u;

    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.substate);
    TEST_ASSERT_EQUAL_UINT16(2u, g_scr.submode_code);
}

/**
 * \brief           Sub-state 3, short range: standstill latches (code 3), speed >= 41 holds (code 4)
 * \note            VS+0x0E < 0x78 -> hold latch; VS+2 < 0x29 and dwell 0 ->
 *                      vehicle_lock_antitheft_latch, code 3; VS+2 >= 0x29 -> dwell 300,
 *                      vehicle_event8_prep, code 4.
 */
static void test_submode_3_short_range(void) {
    g_vs.substate = 3u;
    g_in2.f.chg_cfg = 1u;
    g_vs.distance_code = 0x77u;

    g_vs.speed_kmh = 0x28u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(3u, g_scr.submode_code);

    g_vs.speed_kmh = 0x29u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(4u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_pb13());

    g_vs.speed_kmh = 0u; /* dwell 300 now runs down before code 3 again */
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(4u, g_scr.submode_code);
}

/**
 * \brief           Sub-state 3 unwinds over a 300-tick dwell to code 6 once the range is back
 * \note            VS+0x0E > 200 with the hold latch set -> latch 0, dwell 300; dwell != 0 ->
 *                      dwell - 1; dwell 0 -> vehicle_event1_prep, code 6.
 */
static void test_submode_3_unwind_dwell(void) {
    uint32_t tick;

    g_vs.substate = 3u;
    g_in2.f.chg_cfg = 1u;
    g_vs.distance_code = 0x77u;
    g_vs.speed_kmh = 0x30u;
    ridemode_submode_dispatch(); /* code 4, latch, dwell 300 */

    g_vs.distance_code = 201u;
    for (tick = 0u; tick < DWELL_RELOAD; ++tick) {
        ridemode_submode_dispatch();
        TEST_ASSERT_EQUAL_UINT16(4u, g_scr.submode_code);
    }
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(6u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_pb13());
    TEST_ASSERT_EQUAL_UINT32(0u, prv_pa6());
}

/**
 * \brief           Sub-state 3 entered from code 0 or 2 inside the range band: code 5, which only
 *                      unwinds once the range is back above 200
 * \note            `SCR+0x42 == 0 || == 2` -> dwell 300, code 5, then the same
 *                      pass decrements the dwell (299). With code 5 a band value (120..200)
 *                      matches no branch, so nothing runs; VS+0x0E > 200 resumes the countdown
 *                      (no reload: the hold latch is clear) and dwell 0 gives code 6.
 */
static void test_submode_3_entry_from_code_0(void) {
    uint32_t tick;

    g_vs.substate = 3u;
    g_in2.f.chg_cfg = 1u;
    g_vs.distance_code = 150u;
    g_scr.submode_code = 0u;

    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.submode_code);
    for (tick = 0u; tick < 2u * DWELL_RELOAD; ++tick) {
        ridemode_submode_dispatch();
    }
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.submode_code);

    g_vs.distance_code = 201u;
    for (tick = 0u; tick < DWELL_RELOAD - 1u; ++tick) {
        ridemode_submode_dispatch();
    }
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.submode_code);
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(6u, g_scr.submode_code);
}

/**
 * \brief           Releasing regen in sub-state 0 restores the outputs and clears the code
 * \note            `cur_submode == 1 && IN1+0x26 == 0 && VS+5 == 0` -> control_output_light
 *                      (PB13 0, PA6 0), dwell 0, SCR+0x42 = 0.
 */
static void test_submode_regen_release(void) {
    g_in1.regen_active = 1u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT32(1u, prv_pa6());

    g_in1.regen_active = 0u;
    ridemode_submode_dispatch();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.submode_code);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_pa6());
}

/* -- gear events ----------------------------------------------------------------------------- */

/**
 * \brief           Gear up WALK -> ECO; gear down SPORT -> ECO; ECO -> walk needs bit 4 and < 5 km/h
 * \note            Up from 1 -> 2. Down from 3 -> 2. Down from 2: IN2+0x3A bit 4 and
 *                      VS+2 < 5 -> backup = gear, gear 1.
 */
static void test_gear_basic_transitions(void) {
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_WALK));
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_SPORT));

    g_in2.f.cfg_flags = CFG_WALK_EN;
    g_vs.speed_kmh = 5u;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_ECO));
    g_vs.speed_kmh = 4u;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_WALK, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_ECO));
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, *vcu_faultmon_gear_backup());
}

/**
 * \brief           Gear down from WALK and POWER honours the gear 3 / gear 4 enables
 * \note            Down from 1: bit 9 -> 4, bit 8 -> 3, else 2. Down from 4: bit 8 -> 3,
 *                      else 2.
 */
static void test_gear_down_enables(void) {
    g_in2.f.cfg_flags2 = CFG2_GEAR4_EN | CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_POWER, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_WALK));
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_POWER));
    g_in2.f.cfg_flags2 = CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_WALK));
    g_in2.f.cfg_flags2 = 0u;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_WALK));
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_POWER));
}

/**
 * \brief           Gear down from ECO: walk first, then gear 4, then gear 3; nothing enabled: stays
 * \note            Walk condition, then bit 9 -> 4, bit 8 -> 3,
 *                      else no store.
 */
static void test_gear_down_from_eco(void) {
    g_in2.f.cfg_flags2 = CFG2_GEAR4_EN | CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_POWER, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_ECO));
    g_in2.f.cfg_flags2 = CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_ECO));
    g_in2.f.cfg_flags2 = 0u;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_DOWN, VCU_MODE_ECO));
}

/**
 * \brief           Gear up from ECO: gear 3 (bit 8) first, then gear 4 (bit 9), then walk
 * \note            Bit 8 -> 3; bit 9 -> 4; IN2+0x3A bit 4 and VS+2 < 5 -> walk.
 */
static void test_gear_up_from_eco(void) {
    g_in2.f.cfg_flags2 = CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_ECO));
    g_in2.f.cfg_flags2 = CFG2_GEAR4_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_POWER, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_ECO));
    g_in2.f.cfg_flags2 = CFG2_GEAR4_EN | CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_ECO));
    g_in2.f.cfg_flags2 = 0u;
    g_in2.f.cfg_flags = CFG_WALK_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_WALK, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_ECO));
    g_in2.f.cfg_flags = 0u;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_ECO));
}

/**
 * \brief           Gear up from SPORT: gear 4 (bit 9), else walk, else back to ECO; POWER: walk or ECO
 * \note            From 3: bit 9 -> 4; walk condition; else 2. From 4: walk condition, else 2.
 */
static void test_gear_up_from_sport_and_power(void) {
    g_in2.f.cfg_flags2 = CFG2_GEAR3_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_SPORT));
    g_in2.f.cfg_flags2 = CFG2_GEAR4_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_POWER, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_SPORT));
    g_in2.f.cfg_flags2 = 0u;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_POWER));
    g_in2.f.cfg_flags = CFG_WALK_EN;
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_WALK, prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_POWER));
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_POWER, *vcu_faultmon_gear_backup());
}

/**
 * \brief           A handled gear press beeps, counts, and reports value 100 to node 7
 * \note            Gear-event counter (telemetry field +0x18) + 1, event cleared,
 *                      buzzer_beep(3), STAT gear_report (+0x1F4) = 100, report_ninebot(0x16, 7,
 *                      2, 3, 0xFA, &gear_report): one CAN frame `01 03 FA 64 00` (single-frame
 *                      builder, prio 6).
 */
static void test_gear_press_report(void) {
    const tst_can_frame_t* frame;

    (void)prv_gear_press(FM_MON_GEAR_UP, VCU_MODE_WALK);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_GEAR_UP].code);
    TEST_ASSERT_EQUAL_UINT16(1u, g_telem_frame[REPORT_TELEM_GEAR_EVENT]);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(100u, g_stat.gear_report);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_NOT_NULL(frame);
    TEST_ASSERT_EQUAL_HEX32(GEAR_REPORT_ID, frame->id);
    TEST_ASSERT_EQUAL_UINT8(5u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8(0x01u, frame->data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03u, frame->data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFAu, frame->data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x64u, frame->data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00u, frame->data[4]);
}

/**
 * \brief           Without IN2+0x38 bit 11 the gear stays, the mode report (8) runs, still reported
 * \note            Bit 11 clear -> control_mode_report_fsm(8) (status bit 3, VS+0x36 = 300);
 *                      no beep; the gear report still runs.
 */
static void test_gear_press_without_permission(void) {
    g_vs.gear_sel = VCU_MODE_ECO;
    g_in2.f.status_flags = 0u;
    g_evt.mon[FM_MON_GEAR_DOWN].code = 1u;

    ridemode_gear_event_tick();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_ECO, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0x0008u, g_scr.mode_status & 0x0018u);
    TEST_ASSERT_EQUAL_UINT16(REPORT_RELOAD, g_vs.mode_timeout);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
}

/**
 * \brief           Park gesture (code 3): walk with backup and back, below 5 km/h only
 * \note            EVT+0x50 == 3 -> cleared; VS+2 < 5 -> gear 1 restores the backup (faultmon
 *                      gear-backup cell), else backup = gear, gear 1; beep 3.
 */
static void test_park_toggle(void) {
    g_vs.gear_sel = VCU_MODE_SPORT;
    g_evt.mon[FM_MON_PARK].code = 3u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_WALK, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_PARK].code);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);

    g_evt.mon[FM_MON_PARK].code = 3u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, g_vs.gear_sel);

    g_vs.speed_kmh = 5u;
    g_evt.mon[FM_MON_PARK].code = 3u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(VCU_MODE_SPORT, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_PARK].code);
}

/**
 * \brief           Sub-state next / prev cycle 0..2, or 0..3 with IN2+0x3E bit 0
 * \note            EVT+0x14 == 1 -> VS+5 + 1, `2 <` -> bit 0 clear: 0,
 *                      `3 <` -> 0. EVT+0x20 == 1 -> 0 wraps to 2 (bit 0 clear) or 3, else - 1;
 *                      codes 2 / 3 are only cleared.
 */
static void test_substate_cycle(void) {
    g_vs.substate = 2u;
    g_evt.mon[FM_MON_SUBSTATE_NEXT].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.substate);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_NEXT].code);

    g_in2.f.chg_cfg = 1u;
    g_vs.substate = 2u;
    g_evt.mon[FM_MON_SUBSTATE_NEXT].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.substate);
    g_evt.mon[FM_MON_SUBSTATE_NEXT].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.substate);

    g_evt.mon[FM_MON_SUBSTATE_PREV].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.substate);
    g_in2.f.chg_cfg = 0u;
    g_vs.substate = 0u;
    g_evt.mon[FM_MON_SUBSTATE_PREV].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.substate);
    g_evt.mon[FM_MON_SUBSTATE_PREV].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.substate);

    g_evt.mon[FM_MON_SUBSTATE_PREV].code = 2u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.substate);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_PREV].code);
}

/**
 * \brief           Light enable event: code 1 sets MODE+0x0A bit 9, code 5 clears it
 * \note            EVT+0x74 == 1 -> `|= 0x200`; == 5 -> `&= 0xFDFF`.
 */
static void test_light_enable_event(void) {
    g_evt.mon[FM_MON_LIGHT_EN].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_HEX16(MF_LIGHT_EN, g_mode.mode_flags & MF_LIGHT_EN);

    g_evt.mon[FM_MON_LIGHT_EN].code = 5u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & MF_LIGHT_EN);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_EN].code);
}

/**
 * \brief           Light-hi press toggles outputs (0, 1); a second press turns both off
 * \note            EVT+0x68 == 1: g_chg +6/+7 = 0; +1 == 0 -> +2 = 0, +3 = 1, +1 = 1;
 *                      +1 == 1 -> buzzer_off, +1 = 0, +2/+3 = +4 ? 1 : 0.
 */
static void test_light_hi_toggle(void) {
    g_chg.f6 = 9u;
    g_evt.mon[FM_MON_LIGHT_HI].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f3);
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f1);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f6);

    g_evt.mon[FM_MON_LIGHT_HI].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f1);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f3);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_HI].code);
}

/**
 * \brief           Long press (code 3) of either light button latches both outputs on, again off
 * \note            Code 3 (g_chg offsets): +4 == 0 -> +2 = +3 = 1, +1 = +0 = 0, +4 = 1;
 *                      else +2 = +3 = 0, buzzer_off, +4 = 0. Low button (EVT+0x5C) press 1
 *                      while latched: +0 == 0 -> +2 = 1, +3 = 0, +0 = 1.
 */
static void test_light_long_press_latch(void) {
    g_evt.mon[FM_MON_LIGHT_LO].code = 3u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f3);
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg._rsv04);

    g_evt.mon[FM_MON_LIGHT_LO].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f3);
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f0);

    g_evt.mon[FM_MON_LIGHT_LO].code = 1u; /* second short press: back to the latched pair */
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f3);

    g_evt.mon[FM_MON_LIGHT_HI].code = 3u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f3);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg._rsv04);
}

/**
 * \brief           Power-button press in states 3 / 7 reports 0x61 = 0x20 and consumes a lock edge
 * \note            EVT+8 == 1 and FSM 3 or 7 -> report_u16(0x61, 0x20); VS+0x23 != 0 ->
 *                      VS+0x23 = 0, EVT+8 = 0. Without the edge the event stays.
 */
static void test_power_button_lock_report(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_evt.mon[FM_MON_POWER_BTN].code);

    g_vs.remote_lock_edge = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_edge);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_POWER_BTN].code);

    g_state_block.fsm_state = VCU_FSM_LOCKED;
    g_vs.remote_lock_edge = 1u;
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    ridemode_gear_event_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.remote_lock_edge);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_light_regen);
    RUN_TEST(test_light_brake);
    RUN_TEST(test_light_idle_follows_indicator_latches);
    RUN_TEST(test_light_error_code_untouched);
    RUN_TEST(test_light_charging);
    RUN_TEST(test_light_regen_release_clears);
    RUN_TEST(test_light_field_keeps_other_bits);
    RUN_TEST(test_ledm_power_row);
    RUN_TEST(test_ledm_sport_and_eco_rows);
    RUN_TEST(test_ledm_assist_caps);
    RUN_TEST(test_ledm_other_words_untouched);
    RUN_TEST(test_submode_states_0_to_2);
    RUN_TEST(test_submode_3_needs_cfg_bit0);
    RUN_TEST(test_submode_3_short_range);
    RUN_TEST(test_submode_3_unwind_dwell);
    RUN_TEST(test_submode_3_entry_from_code_0);
    RUN_TEST(test_submode_regen_release);
    RUN_TEST(test_gear_basic_transitions);
    RUN_TEST(test_gear_down_enables);
    RUN_TEST(test_gear_down_from_eco);
    RUN_TEST(test_gear_up_from_eco);
    RUN_TEST(test_gear_up_from_sport_and_power);
    RUN_TEST(test_gear_press_report);
    RUN_TEST(test_gear_press_without_permission);
    RUN_TEST(test_park_toggle);
    RUN_TEST(test_substate_cycle);
    RUN_TEST(test_light_enable_event);
    RUN_TEST(test_light_hi_toggle);
    RUN_TEST(test_light_long_press_latch);
    RUN_TEST(test_power_button_lock_report);
    tst_exit(UNITY_END());
    return 0;
}
