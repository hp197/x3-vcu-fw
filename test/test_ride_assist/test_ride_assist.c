/**
 * \file            test_ride_assist.c
 * \brief           Ride assists (ride_assist.c)
 * \note            Covers ride_assist_tick and what it runs: turn-signal auto-cancel, climb
 *                      boost, over-speed regen, the wobble warning and the resets, plus the
 *                      climb-boost override in control_build_drive_cmd. The tick-exact values
 *                      (cancel tick, float bit patterns) follow from the specified arithmetic:
 *                      double maths, f32 round-to-nearest stores, truncating conversion to int.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "buzzer.h"
#include "control.h"
#include "ride_assist.h"
#include "tst_bsp_fakes.h"
#include "tst_control.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/** \brief Float tolerance of the ride-assist outputs (float rounding, not behaviour) */
#define RA_FLOAT_TOL 1e-4f

#define CEIL_LEAD_25KMH 50u /* MODE cmd_lead: ceiling 25 km/h << 1 -> ceil10 = 250 */
#define FULL_THROTTLE   200u

/**
 * \brief           Float from its IEEE-754 bits
 * \param[in]       bits: the bit pattern
 * \return          the float
 */
static float prv_float(uint32_t bits) {
    float value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/**
 * \brief           Run the ride assists `count` times with the same inputs
 * \param[in]       count: ticks
 * \param[in]       throttle: throttle command
 * \param[in]       wobble_rate: gyro axis 1
 * \param[in]       yaw_rate: gyro axis 2
 */
static void prv_ticks(uint32_t count, uint32_t throttle, int32_t wobble_rate, int32_t yaw_rate) {
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        ride_assist_tick(throttle, 0u, wobble_rate, yaw_rate);
    }
}

/**
 * \brief           ride_assist_init sets the boot values
 * \note            Climb divider 10, over-speed divider 10, swing sign -1, swing gap 80, coast
 *                      count 10.
 */
static void test_init_boot_values(void) {
    const volatile ride_assist_state_t* st = ride_assist_state();

    TEST_ASSERT_EQUAL_UINT8(10u, st->climb_div);
    TEST_ASSERT_EQUAL_UINT8(10u, st->overspeed_div);
    TEST_ASSERT_EQUAL_INT8(-1, st->wobble_sign);
    TEST_ASSERT_EQUAL_UINT8(80u, st->wobble_decay);
    TEST_ASSERT_EQUAL_UINT16(10u, st->coast_ticks);
}

/** \brief Fresh boot state, fakes, USART2, ADC scan at 0; ceiling 25 km/h, SoC 100, gear 4 */
void setUp(void) {
    tst_reset_product_state();
    ride_assist_init(); /* main: the ride-assist boot values */
    tst_fakes_reset();
    vcu_usart_init();
    tst_control_adc_freeze();
    g_mode.cmd_lead = CEIL_LEAD_25KMH;
    g_scr.soc_combined = 100u;
    g_in2.f.gear = 4;
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- turn-signal auto-cancel ---------------------------------------------------------------- */

/**
 * \brief           Light-lo toggle at 10 km/h: arms (2500 ticks), tracks the turn, then
 *                      injects a light-lo press once the heading settles
 * \note            chg+0 -> signal 2, dir -1; yaw (gyro axis 2) low-pass
 *                      `yf = (int)(yf * 0.9 + dir * 0.1 * yaw)`, |yf| <= 19 -> 0; phase 1 at signal
 *                      1/2 (timeout 2500); speed > 40 and yf > 0 -> phase 2; heading += yf_prev,
 *                      peak = (int)(heading * 1.1) on a new maximum; 200 ticks after the last peak
 *                      the mean heading (x 0.001064 x 180 / pi x 0.01 degrees) > 15 -> phase 3;
 *                      phase 3 writes EVT+0x5C (LIGHT_LO code) = 1. Yaw -500 for 150 ticks then 0:
 *                      cancel on tick 358.
 */
static void test_turn_signal_cancels_after_turn(void) {
    const volatile ride_turn_cancel_t* turn = &ride_assist_state()->turn;

    g_chg.f0 = 1u;
    g_in2.f.speed_src = 100u;
    prv_ticks(1u, 0u, 0, -500);
    TEST_ASSERT_EQUAL_UINT8(1u, turn->phase);
    TEST_ASSERT_EQUAL_INT8(-1, turn->dir);
    TEST_ASSERT_EQUAL_INT32(50, turn->yaw_filt);
    TEST_ASSERT_EQUAL_UINT16(2500u, turn->timeout);
    prv_ticks(1u, 0u, 0, -500);
    TEST_ASSERT_EQUAL_UINT8(2u, turn->phase);
    prv_ticks(8u, 0u, 0, -500); /* tick 10 */
    TEST_ASSERT_EQUAL_INT32(322, turn->yaw_filt);
    TEST_ASSERT_EQUAL_INT32(1679, turn->heading);
    TEST_ASSERT_EQUAL_INT32(1846, turn->heading_peak);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3F900C2Du), turn->peak_deg);
    prv_ticks(140u, 0u, 0, -500); /* tick 150 */
    TEST_ASSERT_EQUAL_INT32(68878, turn->heading);
    TEST_ASSERT_EQUAL_INT32(70904, turn->heading_peak);
    TEST_ASSERT_EQUAL_UINT16(2351u, turn->timeout);
    prv_ticks(207u, 0u, 0, 0); /* tick 357 */
    TEST_ASSERT_EQUAL_UINT8(3u, turn->phase);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_LO].code);
    prv_ticks(1u, 0u, 0, 0); /* tick 358 */
    TEST_ASSERT_EQUAL_UINT8(1u, g_evt.mon[FM_MON_LIGHT_LO].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_HI].code);
    TEST_ASSERT_EQUAL_UINT8(0u, turn->phase);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x423EE74Cu), turn->peak_deg);
}

/**
 * \brief           Light-hi toggle at standstill: no turn, so the 2500-tick timeout cancels it
 *                      with a light-hi press
 * \note            chg+1 -> signal 1; phase 1 counts the timeout down each
 *                      tick, `if (timeout < 10) { timeout = 0; phase = 3; }`; phase 3 writes EVT+0x68
 *                      (LIGHT_HI code) = 1. First press on tick 2493.
 */
static void test_turn_signal_timeout(void) {
    g_chg.f1 = 1u;
    prv_ticks(2492u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_HI].code);
    TEST_ASSERT_EQUAL_UINT8(3u, ride_assist_state()->turn.phase);
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(1u, g_evt.mon[FM_MON_LIGHT_HI].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_LO].code);
}

/**
 * \brief           Reg 0x2F bit2 holds the auto-cancel in reset (the 0x2C-byte block cleared)
 * \note            With IN2+0x5E bit2 set the tick runs the auto-cancel reset (the whole
 *                      0x2C-byte block cleared) instead of the auto-cancel tick.
 */
static void test_turn_signal_reset_by_reg2f(void) {
    g_chg.f0 = 1u;
    prv_ticks(5u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(1u, ride_assist_state()->turn.phase);
    g_in2.f.in2_5e = REG2F_TURN_CANCEL_OFF;
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(0u, ride_assist_state()->turn.phase);
    TEST_ASSERT_EQUAL_UINT16(0u, ride_assist_state()->turn.timeout);
    TEST_ASSERT_EQUAL_UINT8(0u, ride_assist_state()->turn.signal);
}

/* -- climb boost ---------------------------------------------------------------------------- */

/**
 * \brief           Full throttle at 20 km/h (below 90 % of the 25 km/h ceiling, not speeding up)
 *                      ramps the target after 100 held ticks and engages at factor > 1.65
 * \note            With reg 0x1E bit2: throttle > 0xC3 and no brake counts climb_hold; every
 *                      15th tick (climb_div > 25 -> 14) climb_target += 0.15 (held > 100, speed
 *                      <= ref); climb_factor slews +0.04 / -0.01; > 1.01 -> ramp += 30; allowed
 *                      (vbus code < 0x7D), speed > 70, factor > 1.65f -> ramp 200, SCR+0x92 = 1,
 *                      OCFG_CLIMB_BOOST set; gear 4 energy = (int)((F - 1) / 5 + 1) * 140), max
 *                      160, current = (int)((E - 50) * 0.24 + 20) capped at 25 + 5.
 */
static void test_climb_boost_engages(void) {
    const volatile ride_assist_state_t* st = ride_assist_state();

    g_in2.f.cfg_flags2 = CFG2_CLIMB_BOOST;
    g_in2.f.speed_src = 200u;
    prv_ticks(100u, FULL_THROTTLE, 0, 0);
    TEST_ASSERT_EQUAL_UINT16(100u, st->climb_hold);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3F800000u), st->climb_target);
    TEST_ASSERT_EQUAL_UINT8(8u, st->climb_ramp);
    TEST_ASSERT_EQUAL_UINT16(140u, st->climb_energy);
    TEST_ASSERT_EQUAL_UINT8(30u, st->climb_current);
    prv_ticks(20u, FULL_THROTTLE, 0, 0); /* tick 120 */
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3F933333u), st->climb_target);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3F95C28Eu), st->climb_factor);
    TEST_ASSERT_EQUAL_UINT8(110u, st->climb_ramp);
    TEST_ASSERT_EQUAL_UINT16(144u, st->climb_energy);
    prv_ticks(40u, FULL_THROTTLE, 0, 0); /* tick 160 */
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3FCF5C24u), st->climb_factor);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.climb_boost_on);
    prv_ticks(1u, FULL_THROTTLE, 0, 0); /* tick 161 */
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3FD47ADCu), st->climb_factor);
    TEST_ASSERT_EQUAL_UINT8(200u, st->climb_ramp);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.climb_boost_on);
    TEST_ASSERT_EQUAL_HEX8(OCFG_CLIMB_BOOST, g_mode.output_cfg & OCFG_CLIMB_BOOST);
    TEST_ASSERT_EQUAL_UINT16(158u, st->climb_energy);

    prv_ticks(1u, 0u, 0, 0); /* throttle released: hold back to 10, factor slews down */
    TEST_ASSERT_EQUAL_UINT16(10u, st->climb_hold);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3FD3332Eu), st->climb_factor);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.climb_boost_on);
}

/**
 * \brief           While boosting, the drive command takes the climb current and energy
 * \note            With IN2+0x3C bit2, climb_factor > 1.0f and climb_allowed, the we_word low
 *                      byte is (u8)climb_energy and the current byte is climb_current; otherwise
 *                      the we_word low byte is derate * lo >> 7. Gear 4 row 0.
 */
static void test_climb_boost_overrides_drive_cmd(void) {
    g_in2.f.cfg_flags2 = CFG2_CLIMB_BOOST;
    g_in2.f.speed_src = 200u;
    prv_ticks(161u, FULL_THROTTLE, 0, 0);
    g_stat.soc = 100;
    g_stat.curr_limit = 10000;
    g_in2.f.status_flags = SF_SESSION;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(30u, g_mode.curr_word & 0xFFu);
    TEST_ASSERT_EQUAL_HEX16(0x5A9Eu, g_mode.we_word);

    g_in2.f.cfg_flags2 = 0u; /* boost disabled: table energy 0x8C, table current 0x28 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(0x28u, g_mode.curr_word & 0xFFu);
    TEST_ASSERT_EQUAL_HEX16(0x5A8Cu, g_mode.we_word);
}

/**
 * \brief           Battery-voltage gate: code < 0x7D allows, > 0x87 forbids, in between it holds
 *                      while the boost is engaged
 * \note            On A3C+2 >> 8 (vbus code) and OCFG_CLIMB_BOOST.
 */
static void test_climb_boost_voltage_gate(void) {
    g_in2.f.cfg_flags2 = CFG2_CLIMB_BOOST;
    g_fault_codes.vbus_code = 0x7C00u;
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(1u, ride_assist_state()->climb_allowed);
    g_fault_codes.vbus_code = 0x7D00u;
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(0u, ride_assist_state()->climb_allowed);
    g_mode.output_cfg = OCFG_CLIMB_BOOST;
    g_fault_codes.vbus_code = 0x8700u;
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(1u, ride_assist_state()->climb_allowed);
    g_mode.output_cfg = OCFG_CLIMB_BOOST;
    g_fault_codes.vbus_code = 0x8800u;
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(0u, ride_assist_state()->climb_allowed);
}

/**
 * \brief           Without reg 0x1E bit2 the climb boost is held in reset
 * \note            climb_target = climb_factor = 1.0f, climb_current = 0, climb_energy = 0,
 *                      climb_ramp = 10, climb_hold = 10, SCR+0x92 = 0, OCFG_CLIMB_BOOST cleared.
 */
static void test_climb_boost_disabled_resets(void) {
    const volatile ride_assist_state_t* st = ride_assist_state();

    g_mode.output_cfg = 0xFFu;
    g_scr.climb_boost_on = 1u;
    prv_ticks(1u, FULL_THROTTLE, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3F800000u), st->climb_target);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3F800000u), st->climb_factor);
    TEST_ASSERT_EQUAL_UINT8(10u, st->climb_ramp);
    TEST_ASSERT_EQUAL_UINT16(10u, st->climb_hold);
    TEST_ASSERT_EQUAL_UINT16(0u, st->climb_energy);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.climb_boost_on);
    TEST_ASSERT_EQUAL_HEX8(0u, g_mode.output_cfg & OCFG_CLIMB_BOOST);
}

/* -- over-speed regen ----------------------------------------------------------------------- */

/**
 * \brief           Coasting at 30 km/h over a 25 km/h ceiling raises the regen level; after the
 *                      500-tick hold-off it raises the over-speed flag at level > 0.25
 * \note            With reg 0x1E bit4 (OCFG_OVERSPEED_REGEN set by the caller): a new ceiling
 *                      (cmd_lead x 5) loads a 500-tick hold that counts only at or below it; coast
 *                      count +1 without throttle/brake; every 11th tick (overspeed_div >= 25 -> 14):
 *                      speed > ref and coast > 40 -> +0.03 (< 0.3) and +0.05 above 20 km/h; speed >
 *                      ceiling + 0.5 -> +0.08 (< 0.3); overspeed_regen = (u8)min(level * 50, 45);
 *                      speed > 12 km/h, hold 0, level > 0.25f -> SCR+0x96 = 1 and the climb reset.
 *                      Flag on tick 32 at 30 km/h.
 */
static void test_overspeed_regen_ramps(void) {
    const volatile ride_assist_state_t* st = ride_assist_state();

    g_in2.f.cfg_flags2 = CFG2_OVERSPEED_REGEN;
    g_in2.f.speed_src = 250u;
    prv_ticks(500u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT16(0u, st->overspeed_hold);
    TEST_ASSERT_EQUAL_HEX8(OCFG_OVERSPEED_REGEN, g_mode.output_cfg & OCFG_OVERSPEED_REGEN);
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.regen_level);

    g_in2.f.speed_src = 300u;
    prv_ticks(10u, 0u, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3E23D70Au), st->overspeed_level);
    TEST_ASSERT_EQUAL_UINT8(8u, st->overspeed_regen);
    prv_ticks(21u, 0u, 0, 0); /* tick 31 */
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.overspeed_regen_on);
    prv_ticks(1u, 0u, 0, 0); /* tick 32 */
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.overspeed_regen_on);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0x3EA3D70Au), st->overspeed_level);
    TEST_ASSERT_EQUAL_UINT8(16u, st->overspeed_regen);

    prv_ticks(1u, 100u, 0, 0); /* throttle: the coast count restarts at 10 */
    TEST_ASSERT_EQUAL_UINT16(10u, st->coast_ticks);
}

/**
 * \brief           Over-speed regen feeds the regen byte as max(level, setting)
 * \note            IN2+0xE0 == 0 -> regen_level = 0 (3 with OCFG_OVERSPEED_REGEN and reg 0x1E
 *                      bit4); with reg 0x1E bit4 regen_level = max(overspeed_regen, regen_level).
 */
static void test_overspeed_regen_drive_byte(void) {
    g_in2.f.cfg_flags2 = CFG2_OVERSPEED_REGEN;
    g_in2.f.speed_src = 250u;
    prv_ticks(500u, 0u, 0, 0);
    g_in2.f.speed_src = 300u;
    prv_ticks(32u, 0u, 0, 0);
    g_in2.f.dir_bits = 0u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(16u, g_mode.regen_level);
    g_in2.f.dir_bits = 2u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(45u, g_mode.regen_level);
}

/**
 * \brief           Disabled over-speed regen: level 0, strength 1, flag and OCFG_OVERSPEED_REGEN cleared
 * \note            overspeed_level = 0, overspeed_regen = 1, SCR+0x96 = 0, OCFG_OVERSPEED_REGEN
 *                      cleared.
 */
static void test_overspeed_regen_disabled(void) {
    g_mode.output_cfg = OCFG_OVERSPEED_REGEN;
    g_scr.overspeed_regen_on = 1u;
    prv_ticks(1u, 0u, 0, 0);
    TEST_ASSERT_EQUAL_UINT8(1u, ride_assist_state()->overspeed_regen);
    TEST_ASSERT_FLOAT_WITHIN(RA_FLOAT_TOL, prv_float(0u), ride_assist_state()->overspeed_level);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.overspeed_regen_on);
    TEST_ASSERT_EQUAL_HEX8(0u, g_mode.output_cfg & OCFG_OVERSPEED_REGEN);

    /* the reset strength 1 still lifts the regen byte when reg 0x1E bit4 is set (max with overspeed_regen) */
    ride_assist_reset_all();
    g_in2.f.cfg_flags2 = CFG2_OVERSPEED_REGEN;
    g_in2.f.dir_bits = 0u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(1u, g_mode.regen_level);
}

/* -- wobble warning ------------------------------------------------------------------------- */

/**
 * \brief           Reg 0x75 = 1: swings above 250 with alternating sign count up; over 25 the
 *                      warning beeps (pattern 3), sets SCR+0x98, OCFG_WOBBLE_ALERT and the score to 600
 * \note            IN2+0xEA 1 -> wobble_threshold = 250 (2 -> 170). A swing (|r| > threshold
 *                      and wobble_sign * r < 0) flips wobble_sign, sets the swing gap to 50 and
 *                      counts the score up. A score over 25 without OCFG_WOBBLE_ALERT beeps
 *                      pattern 3, sets the score to 600, SCR+0x98 = 1 and OCFG_WOBBLE_ALERT.
 *                      Rates +-300 from boot (score 0): warning on tick 26.
 */
static void test_wobble_warning(void) {
    uint32_t tick;

    g_in2.f.wobble_sens = 1u;
    for (tick = 1u; tick <= 25u; ++tick) {
        ride_assist_tick(0u, 0u, ((tick & 1u) != 0u) ? 300 : -300, 0);
    }
    TEST_ASSERT_EQUAL_UINT16(25u, ride_assist_state()->wobble_count);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.wobble_alert_on);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    ride_assist_tick(0u, 0u, -300, 0);
    TEST_ASSERT_EQUAL_UINT16(600u, ride_assist_state()->wobble_count);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.wobble_alert_on);
    TEST_ASSERT_EQUAL_HEX8(OCFG_WOBBLE_ALERT, g_mode.output_cfg & OCFG_WOBBLE_ALERT);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(250u, ride_assist_state()->wobble_threshold);

    /* calm again: the score decays 1 per tick once the swing gap is below 30; clears below 15 */
    for (tick = 0u; (tick < 700u) && (g_scr.wobble_alert_on != 0u); ++tick) {
        ride_assist_tick(0u, 0u, 0, 0);
    }
    TEST_ASSERT_EQUAL_UINT32(607u, tick);
    TEST_ASSERT_EQUAL_UINT16(14u, ride_assist_state()->wobble_count);
    TEST_ASSERT_EQUAL_HEX8(0u, g_mode.output_cfg & OCFG_WOBBLE_ALERT);
}

/**
 * \brief           Reg 0x75 = 2 lowers the threshold to 170; with 1, swings of 200 do not count
 * \note            IN2+0xEA == 2 -> wobble_threshold = 0xAA.
 */
static void test_wobble_threshold_by_reg75(void) {
    uint32_t tick;

    g_in2.f.wobble_sens = 1u;
    for (tick = 1u; tick <= 30u; ++tick) {
        ride_assist_tick(0u, 0u, ((tick & 1u) != 0u) ? 200 : -200, 0);
    }
    TEST_ASSERT_EQUAL_UINT16(0u, ride_assist_state()->wobble_count);

    tst_reset_product_state();
    ride_assist_init();
    g_in2.f.wobble_sens = 2u;
    for (tick = 1u; tick <= 26u; ++tick) {
        ride_assist_tick(0u, 0u, ((tick & 1u) != 0u) ? 200 : -200, 0);
    }
    TEST_ASSERT_EQUAL_UINT16(600u, ride_assist_state()->wobble_count);
    TEST_ASSERT_EQUAL_UINT16(170u, ride_assist_state()->wobble_threshold);
}

/**
 * \brief           Gear 1 (or SCR+0x52) resets every assist each tick
 * \note            SCR+0x52 != 0 or IN2+0xB4 == 1 runs ride_assist_reset_all: over-speed
 *                      reset, wobble reset (wobble_count = 10), climb reset.
 */
static void test_reset_all_on_gear1(void) {
    g_in2.f.gear = 1;
    g_in2.f.wobble_sens = 1u;
    prv_ticks(1u, 0u, 300, 0);
    TEST_ASSERT_EQUAL_UINT16(11u, ride_assist_state()->wobble_count); /* reset to 10, then one swing */
    TEST_ASSERT_EQUAL_UINT8(1u, ride_assist_state()->overspeed_regen);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_init_boot_values);
    RUN_TEST(test_turn_signal_cancels_after_turn);
    RUN_TEST(test_turn_signal_timeout);
    RUN_TEST(test_turn_signal_reset_by_reg2f);
    RUN_TEST(test_climb_boost_engages);
    RUN_TEST(test_climb_boost_overrides_drive_cmd);
    RUN_TEST(test_climb_boost_voltage_gate);
    RUN_TEST(test_climb_boost_disabled_resets);
    RUN_TEST(test_overspeed_regen_ramps);
    RUN_TEST(test_overspeed_regen_drive_byte);
    RUN_TEST(test_overspeed_regen_disabled);
    RUN_TEST(test_wobble_warning);
    RUN_TEST(test_wobble_threshold_by_reg75);
    RUN_TEST(test_reset_all_on_gear1);
    tst_exit(UNITY_END());
    return 0;
}
