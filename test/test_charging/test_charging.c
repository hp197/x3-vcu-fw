/**
 * \file            test_charging.c
 * \brief           Charge timer / fault-code annunciation (charging.c)
 * \note            The rear light is read from MODE+0x10 bits 9..11, beep requests from the
 *                      request block g_lowglob (cleared to BEEP_NONE before each tick).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "buzzer.h"
#include "charging.h"
#include "control.h"
#include "tst_bsp_fakes.h"
#include "tst_control.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define SWEEP_TICKS_23 340u   /*!< code 23: 100 lead-in + 2*60 + 30 gap + 3*30 */
#define BRAKE_HELD_RAW 2048u  /*!< ADC raw well above the 50 << 4 held threshold */
#define REPORT_RELOAD  300u   /*!< VS+0x36 reload of control_mode_report_fsm(4) */

/** \brief Rear-light mode currently published \return MODE+0x10 bits 9..11 */
static uint16_t prv_light(void) { return (uint16_t)((g_mode.enable_flags & MODE_LIGHT_MASK) >> MODE_LIGHT_SHIFT); }

/** \brief Run one tick with the beep request block cleared first \return the tick's result */
static uint32_t prv_tick(void) {
    g_lowglob.beep_seq_id = BEEP_NONE;
    return vcu_charge_timer_tick();
}

/**
 * \brief           Fresh boot state, USART2 ring ready for the report frames
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- abort and code changes ------------------------------------------------------------------ */

/**
 * \brief           A non-zero IN1+0x04 aborts: segment and cycle zeroed, the code returned
 * \note            IN1+0x04 != 0 -> seg_tick = 0, cycle = 0, return that byte. Nothing else is
 *                      touched (no snapshot update).
 */
static void test_abort_resets_and_returns_code(void) {
    g_in1.drive_active = 5u;
    g_charge_timer.seg_tick = 77u;
    g_charge_timer.cycle = 2u;
    g_lowglob.active_error_code = 23u;

    TEST_ASSERT_EQUAL_UINT32(5u, prv_tick());
    TEST_ASSERT_EQUAL_UINT16(0u, g_charge_timer.seg_tick);
    TEST_ASSERT_EQUAL_UINT8(0u, g_charge_timer.cycle);
    TEST_ASSERT_EQUAL_UINT8(0u, g_charge_timer.code_snapshot);
}

/**
 * \brief           A change of the latched error code restarts the annunciation
 * \note            `if (tmr[2] != ERROR_ACTIVE) { seg = 0; cycle = 0; }` then `tmr[2] = code`.
 */
static void test_code_change_restarts(void) {
    g_lowglob.active_error_code = 12u;
    g_charge_timer.code_snapshot = 23u;
    g_charge_timer.seg_tick = 200u;
    g_charge_timer.cycle = 2u;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_tick());
    TEST_ASSERT_EQUAL_UINT8(12u, g_charge_timer.code_snapshot);
    TEST_ASSERT_EQUAL_UINT16(1u, g_charge_timer.seg_tick);
    TEST_ASSERT_EQUAL_UINT8(0u, g_charge_timer.cycle);
}

/* -- charge timer (no error code) ------------------------------------------------------------ */

/**
 * \brief           No event: chirp held at 0, the minute count returned until it reaches 100
 * \note            Event byte 0 -> chirp_tick = 0; `99 < min` -> min = 0 and
 *                      `return control_mode_report_fsm(4)`, else return min.
 */
static void test_idle_returns_minutes(void) {
    g_charge_timer.chirp_tick = 33u;
    g_charge_timer.charge_min = 99u;

    TEST_ASSERT_EQUAL_UINT32(99u, prv_tick());
    TEST_ASSERT_EQUAL_UINT16(0u, g_charge_timer.chirp_tick);
    TEST_ASSERT_EQUAL_UINT16(99u, g_charge_timer.charge_min);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.mode_timeout);
}

/**
 * \brief           No event and 100 minutes: the counter restarts and the mode report (4) runs
 * \note            control_mode_report_fsm(4) (control.c) sets the status bit 2 and reloads
 *                      VS+0x36 = 300.
 */
static void test_idle_full_mark_dispatches_report(void) {
    g_charge_timer.charge_min = 100u;
    g_scr.mode_status = 0u;

    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_charge_timer.charge_min);
    TEST_ASSERT_EQUAL_UINT16(REPORT_RELOAD, g_vs.mode_timeout);
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_scr.mode_status & 0x001Cu);
}

/**
 * \brief           Event active and parked without brake: one minute counted per tick
 * \note            Speed byte 0 and IN1+0x0B 0 -> min = min + 1.
 */
static void test_event_parked_counts_minutes(void) {
    g_lowglob.active_event_id = VCU_CHG_EVT_CHARGING;
    g_charge_timer.charge_min = 10u;

    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(11u, g_charge_timer.charge_min);
    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(12u, g_charge_timer.charge_min);
}

/**
 * \brief           The clamp tests the value before the increment: 100 -> 101 -> 100
 * \note            `min = old + 1; if (100 < old) min = 100` with `old`
 *                      loaded before the store, so the counter alternates 101 / 100.
 */
static void test_event_minute_clamp_tests_old_value(void) {
    g_lowglob.active_event_id = 1u;
    g_charge_timer.charge_min = 100u;

    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(101u, g_charge_timer.charge_min);
    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(100u, g_charge_timer.charge_min);
}

/**
 * \brief           A brake lever holds the count, any speed restarts it
 * \note            Speed != 0 -> min = 0; IN1+0x0B != 0 -> no store.
 */
static void test_event_brake_holds_speed_resets(void) {
    g_lowglob.active_event_id = 1u;
    g_charge_timer.charge_min = 40u;
    g_in1.brake_any = 1u;

    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(40u, g_charge_timer.charge_min);

    g_vs.speed_kmh = 3u;
    (void)prv_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_charge_timer.charge_min);
}

/**
 * \brief           The chirp: a double beep and reload 1000 for the charging event, then count down
 * \note            Chirp 0 -> buzzer_beep(3), reload 1000 when the event byte is 9, else 500;
 *                      non-zero -> decrement. The value is returned.
 */
static void test_chirp_cadence_charging_event(void) {
    g_lowglob.active_event_id = VCU_CHG_EVT_CHARGING;

    TEST_ASSERT_EQUAL_UINT32(1000u, prv_tick());
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(999u, prv_tick());
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           Any other event chirps on the 500-tick reload
 * \note            Event byte != 9 -> 500.
 */
static void test_chirp_cadence_other_event(void) {
    g_lowglob.active_event_id = 2u;
    g_charge_timer.chirp_tick = 1u;

    TEST_ASSERT_EQUAL_UINT32(0u, prv_tick());
    TEST_ASSERT_EQUAL_UINT32(500u, prv_tick());
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
}

/**
 * \brief           At the full mark, a latched brake-held window counts; the third one reports
 * \note            min > 99 -> vcu_faultmon_sensor_debounce_a / _b (5 consecutive held readings
 *                      latch code 4 at EVT+0x80 / +0x8C); a code 4 clears both windows and bumps
 *                      tmr[1]; `2 < tmr[1]` -> tmr[1] = 0, control_mode_report_fsm(4).
 *                      The debouncer holds its counter while latched, so each confirm needs a
 *                      release in between.
 */
static void test_full_mark_brake_window_confirm(void) {
    uint32_t press;
    uint32_t tick;

    tst_control_adc_freeze();
    g_lowglob.active_event_id = 1u;
    g_charge_timer.charge_min = 100u;
    for (press = 0u; press < 3u; ++press) {
        TEST_ASSERT_EQUAL_UINT16(0u, g_vs.mode_timeout);
        tst_control_adc_set(VCU_ADC_SLOT_BRAKE_FULL, BRAKE_HELD_RAW);
        for (tick = 0u; tick < 5u; ++tick) {
            (void)prv_tick();
        }
        TEST_ASSERT_EQUAL_UINT8((press + 1u) % 3u, g_charge_timer.fault_debounce);
        TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_BRAKE_FULL].code);
        tst_control_adc_set(VCU_ADC_SLOT_BRAKE_FULL, 0u);
        (void)prv_tick();
    }
    TEST_ASSERT_EQUAL_UINT16(REPORT_RELOAD, g_vs.mode_timeout);
}

/* -- error-code annunciation ----------------------------------------------------------------- */

/**
 * \brief           Code 23 sweep: lead-in, 2 long pulses, gap, 3 short pulses, 340 ticks
 * \note            tick < 100 light off; tens window (tick - 100) % 30 == 0 -> on +
 *                      buzzer_beep(1) at % 60, else off; gap until tens*60 + 130; units window
 *                      (tick - tens*60 - 130) % 15 == 0 -> buzzer_beep(2) + on at % 30, else off;
 *                      then seg = 0, cycle + 1. The
 *                      light is MODE+0x10 `& 0xF1FF` (+ 0x600 = brake light while on).
 */
static void test_annunciation_code_23_sweep(void) {
    uint32_t tick;
    uint32_t longs = 0u;
    uint32_t shorts = 0u;
    uint32_t on_ticks = 0u;

    g_lowglob.active_error_code = 23u;
    for (tick = 1u; tick <= SWEEP_TICKS_23; ++tick) {
        TEST_ASSERT_EQUAL_UINT32(1u, prv_tick());
        longs += (g_lowglob.beep_seq_id == BEEP_LONG) ? 1u : 0u;
        shorts += (g_lowglob.beep_seq_id == BEEP_SHORT) ? 1u : 0u;
        on_ticks += (prv_light() == MODE_LIGHT_BRAKE) ? 1u : 0u;
        if ((tick == 99u) || (tick == 130u) || (tick == 220u) || (tick == 265u)) {
            TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_OFF, prv_light());
        }
        if ((tick == 100u) || (tick == 160u) || (tick == 250u) || (tick == 310u)) {
            TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE, prv_light());
        }
    }
    TEST_ASSERT_EQUAL_UINT32(2u, longs);
    TEST_ASSERT_EQUAL_UINT32(3u, shorts);
    TEST_ASSERT_EQUAL_UINT32(2u * 30u + 3u * 15u, on_ticks);
    TEST_ASSERT_EQUAL_UINT16(0u, g_charge_timer.seg_tick);
    TEST_ASSERT_EQUAL_UINT8(1u, g_charge_timer.cycle);
}

/**
 * \brief           After three sweeps the tick returns 0, yet the pattern keeps running
 * \note            `return tmr[0] < 3`; nothing stops the sweep, so the
 *                      fourth sweep still turns the light on at tick 100.
 */
static void test_annunciation_three_cycles_then_keeps_blinking(void) {
    uint32_t tick;
    uint32_t last = 1u;

    g_lowglob.active_error_code = 23u;
    for (tick = 0u; tick < 3u * SWEEP_TICKS_23; ++tick) {
        last = prv_tick();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, last);
    TEST_ASSERT_EQUAL_UINT8(3u, g_charge_timer.cycle);

    for (tick = 0u; tick < 100u; ++tick) {
        last = prv_tick();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, last);
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE, prv_light());
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
}

/**
 * \brief           A single-digit code has an empty tens window: units start at tick 130
 * \note            tens = code / 10 = 0, so the tens window
 *                      `tick < 0*60 + 100` is empty and the gap runs 100..129.
 */
static void test_annunciation_single_digit(void) {
    uint32_t tick;
    uint32_t shorts = 0u;

    g_lowglob.active_error_code = 5u;
    for (tick = 1u; tick <= 130u; ++tick) {
        (void)prv_tick();
        shorts += (g_lowglob.beep_seq_id == BEEP_SHORT) ? 1u : 0u;
        TEST_ASSERT_NOT_EQUAL(BEEP_LONG, g_lowglob.beep_seq_id);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, shorts);
    TEST_ASSERT_EQUAL_UINT16(MODE_LIGHT_BRAKE, prv_light());
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_abort_resets_and_returns_code);
    RUN_TEST(test_code_change_restarts);
    RUN_TEST(test_idle_returns_minutes);
    RUN_TEST(test_idle_full_mark_dispatches_report);
    RUN_TEST(test_event_parked_counts_minutes);
    RUN_TEST(test_event_minute_clamp_tests_old_value);
    RUN_TEST(test_event_brake_holds_speed_resets);
    RUN_TEST(test_chirp_cadence_charging_event);
    RUN_TEST(test_chirp_cadence_other_event);
    RUN_TEST(test_full_mark_brake_window_confirm);
    RUN_TEST(test_annunciation_code_23_sweep);
    RUN_TEST(test_annunciation_three_cycles_then_keeps_blinking);
    RUN_TEST(test_annunciation_single_digit);
    tst_exit(UNITY_END());
    return 0;
}
