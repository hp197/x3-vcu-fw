/**
 * \file            test_control_fsm.c
 * \brief           Control-loop timers and state machines (control.c)
 * \note            Covers the countdowns (auto-off, shutdown, countdown beep, chirp pulse), the
 *                      anti-theft motion alarm, the subsystem A/B outputs with the charge current
 *                      sense, the beep player, the mode/lighting report, the brake-hold fault, the
 *                      lock FSM, the power monitor and the drive-parameter seed. Outputs are
 *                      checked on the emulated GPIOs (odt), TMR1 (buzzer channel enable), the
 *                      USART2 reports and the blackboards.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "at32f415.h"
#include "buzzer.h"
#include "can.h"
#include "control.h"
#include "tst_bsp_fakes.h"
#include "tst_control.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define NODE_VCU         0x16u
#define NODE_APP         0x23u
#define CMD_WRITE_SILENT 0x03u
#define ARG_MODE_STATUS  0xD5u

#define PIN_SUBSYS_A     GPIO_PINS_10 /* PB10, active low */
#define PIN_SUBSYS_B     GPIO_PINS_0  /* PB0, active low  */
#define PIN_CHIRP        GPIO_PINS_6  /* PF6              */

#define MODE_LIGHT_BITS  0x0E00u
#define MODE_FLAG_LIGHT  0x0200u /* MODE+0x0A bit 9 */

#define ALARM_RAMP_TH0   120u /* ramp > 119 advances with sensitivity 0 */
#define ALARM_RAMP_TH2   15u  /* ramp > 14 advances with sensitivity 2 */
#define ALARM_WINDOW_A   550u
#define ALARM_GIVE_UP    500u

#define CHG_SENSE_OK     319u /* 5 samples summed, >> 2: 398 < 400 */
#define CHG_SENSE_HIGH   320u /* 400: not below 400                */
#define CHG_AVERAGES     6u   /* the 6th sub-400 average sets the latch */
#define CHG_SAMPLES      5u   /* samples per average                    */
#define CHG_EVENT        0x18u

/**
 * \brief           Output latch of a GPIO pin
 * \param[in]       port: GPIO port
 * \param[in]       pin: pin mask
 * \return          1 if the output data bit is set
 */
static uint8_t prv_pin(const gpio_type* port, uint16_t pin) { return ((port->odt & pin) != 0u) ? 1u : 0u; }

/**
 * \brief           Buzzer state: TMR1 channel 1 output enable (buzzer on/off)
 * \return          1 while the buzzer sounds
 */
static uint8_t prv_buzzer_on(void) { return (uint8_t)TMR1->cctrl_bit.c1en; }

/**
 * \brief           Run a state machine step `count` times
 * \param[in]       step: the product function
 * \param[in]       count: calls
 */
static void prv_repeat(void (*step)(void), uint32_t count) {
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        step();
    }
}

/**
 * \brief           Run the alarm FSM step `count` times
 * \param[in]       count: calls
 * \return          OR of the return values
 */
static uint32_t prv_alarm_steps(uint32_t count) {
    uint32_t idx;
    uint32_t done = 0u;

    for (idx = 0u; idx < count; ++idx) {
        done |= control_alarm_motion_tick(1);
    }
    return done;
}

/**
 * \brief           Parse captured USART frame `index` and check it is a mode-status report
 * \param[in]       index: capture index
 * \return          the reported status halfword
 */
static uint16_t prv_mode_report(uint32_t index) {
    const tst_usart_frame_t* raw = tst_usart_tx_get(index);
    tst_nb_frame_t frame;

    TEST_ASSERT_NOT_NULL(raw);
    TEST_ASSERT_TRUE(tst_nb_parse(raw->bytes, raw->len, &frame));
    TEST_ASSERT_TRUE(frame.checksum_ok);
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, frame.src);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(CMD_WRITE_SILENT, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(ARG_MODE_STATUS, frame.arg);
    TEST_ASSERT_EQUAL_UINT8(2u, frame.len);
    return (uint16_t)(frame.body[0] | ((uint16_t)frame.body[1] << 8));
}

/**
 * \brief           Fresh boot state, fakes, USART2 (reports), ADC scan with every input at 0, outputs released
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
    tst_control_adc_freeze();
    control_subsys_a_enable(0);
    control_subsys_b_enable(0);
    buzzer_off();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- countdowns ---------------------------------------------------------------------------- */

/**
 * \brief           At standstill the auto-off count loads `reload`, counts down and requests
 *                      shutdown at 0
 * \note            `if (VS+2 < 3 && VS+0x1D == 0) { ... if (VS+0x3C == 0)
 *                      { VS+0x3C = reload; return; } VS+0x3C--; ... if (VS+0x3C == 0) VS+0x1C = 1; }`.
 */
static void test_auto_off_counts_down(void) {
    uint32_t idx;

    for (idx = 0u; idx < 5u; ++idx) {
        control_auto_off_tick(5u);
    }
    TEST_ASSERT_EQUAL_UINT32(1u, g_vs.auto_off_ticks);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
    control_auto_off_tick(5u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_vs.auto_off_ticks);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
}

/**
 * \brief           Motion (speed >= 3 or VS+0x1D) re-arms the count and drops a long press
 * \note            Else branch: `VS+0x1D = 0; VS+0x3C = reload; if (EVT+8 == 3)
 *                      EVT+8 = 0;` (EVT+8 is the power-button monitor).
 */
static void test_auto_off_motion_rearms(void) {
    g_vs.auto_off_ticks = 3u;
    g_vs.speed_kmh = VCU_STANDSTILL_KMH;
    g_evt.mon[FM_MON_POWER_BTN].code = 3u;
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT32(6000u, g_vs.auto_off_ticks);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);

    g_vs.speed_kmh = 0u;
    g_vs.can_drive_active = 1u;
    g_vs.auto_off_ticks = 3u;
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT32(6000u, g_vs.auto_off_ticks);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.can_drive_active);
}

/**
 * \brief           At standstill a long press or a tripped tilt latch requests shutdown at once
 * \note            `if (EVT+8 == 3) { EVT+8 = 0; VS+0x1C = 1; return; }
 *                      if (VS+6 == 1) { VS+0x2A = 0; VS+6 = 0; VS+0x1C = 1; return; }`.
 */
static void test_auto_off_long_press_and_tilt(void) {
    g_vs.auto_off_ticks = 100u;
    g_evt.mon[FM_MON_POWER_BTN].code = 3u;
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT32(100u, g_vs.auto_off_ticks);

    g_vs.shutdown_req = 0u;
    g_vs.tilt_latch = 1u;
    g_vs.creep_enable = 1u;
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.tilt_latch);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.creep_enable);
}

/**
 * \brief           With a drive-fault request the last 99 ticks beep short
 * \note            `if (SCR+0x56 != 0 && VS+0x3C < 0x1771 && VS+0x3C < 100)
 *                      buzzer_beep(2);` after the decrement.
 */
static void test_auto_off_beeps_near_expiry(void) {
    g_scr.drive_fault_req = 1u;
    g_vs.auto_off_ticks = 101u;
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);

    g_lowglob.beep_seq_id = BEEP_NONE;
    g_scr.drive_fault_req = 0u;
    control_auto_off_tick(6000u);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           The countdown mirror is VS+0x3C / 100; a short beep on each whole 100 up to 500
 * \note            `SCR+200 = VS+0x3C / 100; if (VS+0x3C != 0 && VS+0x3C < 0x1f5
 *                      && VS+0x3C % 100 == 0) buzzer_beep(2);`.
 */
static void test_countdown_beep(void) {
    g_vs.auto_off_ticks = 501u;
    control_countdown_beep();
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.countdown_mirror);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    g_vs.auto_off_ticks = 600u;
    control_countdown_beep();
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    g_vs.auto_off_ticks = 450u;
    control_countdown_beep();
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    g_vs.auto_off_ticks = 500u;
    control_countdown_beep();
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.countdown_mirror);
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);

    g_lowglob.beep_seq_id = BEEP_NONE;
    g_vs.auto_off_ticks = 0u;
    control_countdown_beep();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.countdown_mirror);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           The one-shot countdown reloads at 0 and enters power-down when it reaches 0
 * \note            `if (VS+0x3C != 0) { VS+0x3C--; if (VS+0x3C == 0) fsm_state = 4;
 *                      return; } VS+0x3C = reload;`.
 */
static void test_shutdown_countdown(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    control_shutdown_countdown(2u);
    TEST_ASSERT_EQUAL_UINT32(2u, g_vs.auto_off_ticks);
    control_shutdown_countdown(2u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_RIDE, g_state_block.fsm_state);
    control_shutdown_countdown(2u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_vs.auto_off_ticks);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, g_state_block.fsm_state);
}

/**
 * \brief           A chirp request holds PF6 high for 10 ticks, then drops it and clears itself
 * \note            `if (VS+0x29) { n = VS+0x2E++; if (n < 10) { set(0x40011C00, 0x40);
 *                      return; } reset(0x40011C00, 0x40); VS+0x2E = 0; VS+0x29 = 0; }`.
 */
static void test_chirp_pulse_ten_ticks(void) {
    gpio_bits_reset(GPIOF, PIN_CHIRP);
    g_vs.chirp_req = 1u;
    prv_repeat(control_debounced_gpio_drive, 10u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOF, PIN_CHIRP));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chirp_req);
    control_debounced_gpio_drive();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOF, PIN_CHIRP));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chirp_req);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.chirp_tick);
    control_debounced_gpio_drive();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOF, PIN_CHIRP));
}

/* -- tilt latch ----------------------------------------------------------------------------- */

/* -- anti-theft motion alarm (control_alarm_motion_tick) ------------------------------------ */

/**
 * \brief           Teardown resets the step state, lights and buzzer and releases the outputs
 * \note            control_alarm_motion_tick(0): TMR+0x10 = 1, +0x20 = 0, +0x2C = 0, MODE+0xA &= 0xFDFF,
 *                      MODE+0x10 &= 0xF1FF, VS+0x2A = 1, buzzer_off, vcu_outputs_reset, control_mode_sync.
 */
static void test_alarm_teardown(void) {
    g_tmr.step_state = 4u;
    g_tmr.step_ramp = 9u;
    g_tmr.step_tick = 77u;
    g_mode.mode_flags = 0xFFFFu;
    g_mode.enable_flags = 0x0E03u;
    buzzer_on(BUZZER_TONE);
    control_subsys_a_enable(1);
    TEST_ASSERT_EQUAL_UINT32(0u, control_alarm_motion_tick(0));
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.step_state);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.step_ramp);
    TEST_ASSERT_EQUAL_UINT32(0u, g_tmr.step_tick);
    TEST_ASSERT_EQUAL_HEX16(0xFDFFu, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.enable_flags & (MODE_LIGHT_BITS | 3u));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.creep_enable);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_B));
}

/**
 * \brief           Step 1: motion (metric > 20.0) with the alarm enabled ramps; > 0x77 advances
 * \note            VS+0x10 is compared with 0x41A00000 (20.0f). State 1: `if (IN2+0x3A bit15 &&
 *                      thr < (int)VS+0x10 && TMR+0x20 < 500) TMR+0x20++; if (ramp > 0x77 || (IN2+0xE8 == 1
 *                      && ramp > 0x45) || (IN2+0xE8 == 2 && ramp > 0xE)) { state 2; ramp = tick = 0;
 *                      return 1; }`.
 */
static void test_alarm_arm_ramp_advances(void) {
    g_tmr.step_state = 1u;
    g_in2.f.cfg_flags = CFG_ALARM_EN;
    g_vs.sensor_delta_f = 20.5f;
    TEST_ASSERT_EQUAL_UINT32(0u, prv_alarm_steps(ALARM_RAMP_TH0 - 1u));
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.step_state);
    TEST_ASSERT_EQUAL_UINT32(1u, control_alarm_motion_tick(1));
    TEST_ASSERT_EQUAL_UINT8(2u, g_tmr.step_state);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.step_ramp);
    TEST_ASSERT_EQUAL_UINT32(0u, g_tmr.step_tick);

    g_tmr.step_state = 1u;
    g_in2.f.alarm_sensitivity = 2u;
    TEST_ASSERT_EQUAL_UINT32(0u, prv_alarm_steps(ALARM_RAMP_TH2 - 1u));
    TEST_ASSERT_EQUAL_UINT32(1u, control_alarm_motion_tick(1));
}

/**
 * \brief           Step 1 without motion (or with the alarm disabled) gives up at tick 500
 * \note            State 1: `if (499 < tick) { VS+0x1C = 1; VS+0x2B = 1; }`;
 *                      a metric of exactly 20.0 is not above the 0x41A00000 (20.0f) gate.
 */
static void test_alarm_arm_gives_up(void) {
    g_tmr.step_state = 1u;
    g_in2.f.cfg_flags = CFG_ALARM_EN;
    g_vs.sensor_delta_f = 20.0f;
    (void)prv_alarm_steps(ALARM_GIVE_UP - 1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.step_ramp);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
    (void)control_alarm_motion_tick(1);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);

    g_tmr.step_state = 1u;
    g_tmr.step_tick = 0u;
    g_in2.f.cfg_flags = 0u;
    g_vs.sensor_delta_f = 25.0f;
    (void)prv_alarm_steps(10u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.step_ramp);
}

/**
 * \brief           Step 2: siren on at ticks 1/150/300/450, off at 100/250/400/550; at 550 too
 *                      little motion requests shutdown, enough advances to step 3
 * \note            State 2: buzzer_on(0xA8C) / buzzer_off at those
 *                      ticks; `if (tick < 0x226) ramp++ (gated) else if (ramp below threshold)
 *                      VS+0x1C = 1; else state 3`. No alarm-enable gate on the ramp here.
 */
static void test_alarm_siren_window(void) {
    g_tmr.step_state = 2u;
    g_vs.sensor_delta_f = 25.0f;
    (void)control_alarm_motion_tick(1);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    (void)prv_alarm_steps(98u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    (void)control_alarm_motion_tick(1); /* tick 100 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    (void)prv_alarm_steps(ALARM_WINDOW_A - 101u); /* tick 0x225 */
    TEST_ASSERT_EQUAL_UINT8(2u, g_tmr.step_state);
    (void)control_alarm_motion_tick(1); /* tick 0x226: window evaluation */
    TEST_ASSERT_EQUAL_UINT8(3u, g_tmr.step_state);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);

    g_tmr.step_state = 2u;
    g_vs.sensor_delta_f = 0.0f;
    (void)prv_alarm_steps(ALARM_WINDOW_A);
    TEST_ASSERT_EQUAL_UINT8(2u, g_tmr.step_state);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
}

/**
 * \brief           Step 3 blinks: tick 1 latches the lights + subsystems, tick 100 drops them
 * \note            State 3: tick 1 -> vehicle_lock_antitheft_latch, report_subsys_enable, MODE+0xA |= 0x200,
 *                      MODE+0x10 = (MODE+0x10 & 0xF1FF) + 0x200; tick 100 -> vcu_outputs_reset,
 *                      control_mode_sync, MODE+0xA &= 0xFDFF.
 */
static void test_alarm_blink_phase(void) {
    g_tmr.step_state = 3u;
    (void)control_alarm_motion_tick(1);
    TEST_ASSERT_EQUAL_HEX16(MODE_FLAG_LIGHT, g_mode.mode_flags & MODE_FLAG_LIGHT);
    TEST_ASSERT_EQUAL_HEX16(0x0200u, g_mode.enable_flags & MODE_LIGHT_BITS);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_B));
    (void)prv_alarm_steps(99u);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & MODE_FLAG_LIGHT);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_B));
}

/**
 * \brief           Lock-out alarm (VS+0x42 == 2): siren only, shutdown after tick 0x225, phase back to 1
 * \note            `if (VS+0x42 == 2) { tick++; siren ticks; if (0x225 < tick)
 *                      { VS+0x1C = 1; VS+0x42 = 1; } }`, before the step-state switch.
 */
static void test_alarm_lockout_siren(void) {
    g_vs.lockout_phase = LOCKOUT_ALARM;
    g_tmr.step_state = 3u;
    (void)control_alarm_motion_tick(1);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & MODE_FLAG_LIGHT);
    (void)prv_alarm_steps(ALARM_WINDOW_A - 2u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
    (void)control_alarm_motion_tick(1);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_ARMED, g_vs.lockout_phase);
    TEST_ASSERT_EQUAL_UINT8(3u, g_tmr.step_state);
}

/* -- outputs ------------------------------------------------------------------------------- */

/**
 * \brief           Subsystem A = PB10, B = PB0, both active low
 * \note            Subsystem A: GPIOB (0x40010C00) mask 0x400; en != 0 writes the clear register
 *                      (+0x14), else the set register (+0x10). Subsystem B: the same with mask 1.
 */
static void test_subsys_enable_active_low(void) {
    control_subsys_a_enable(1);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_B));
    control_subsys_b_enable(1);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_B));
    control_subsys_a_enable(0);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_B));
    control_subsys_b_enable(0);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_B));
}

/**
 * \brief           Indicator leaves PB13 / PA6 / PB14; the mode light drops PB13+PA6 and sets
 *                      PB14 only in row 0
 * \note            control_out_pb13 (GPIOB 0x2000), control_out_pa6 (GPIOA 0x40), control_out_pb14
 *                      (GPIOB 0x4000): a != 0 sets. control_output_light: (0), (0), then `VS+0 != 0 ?
 *                      control_out_pb14(0) : control_out_pb14(1)`.
 */
static void test_indicator_outputs(void) {
    control_out_pb13(1);
    control_out_pa6(1);
    control_out_pb14(1);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, GPIO_PINS_13));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOA, GPIO_PINS_6));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, GPIO_PINS_14));

    g_vs.region = VCU_REGION_EUROPE;
    control_output_light();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, GPIO_PINS_13));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOA, GPIO_PINS_6));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, GPIO_PINS_14));

    g_vs.region = VCU_REGION_GLOBAL;
    control_output_light();
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, GPIO_PINS_14));
}

/**
 * \brief           Mode sync releases both subsystems and clears the MODE+0x10 enable bits
 * \note            control_subsys_a_enable(0), control_subsys_b_enable(0), then the mask-0 publish
 *                      (MODE+0x10 = (MODE+0x10 & 0xFFFC) + 0, SCR+0xC0 = 0).
 */
static void test_mode_sync_releases_subsystems(void) {
    control_subsys_a_enable(1);
    control_subsys_b_enable(1);
    g_mode.enable_flags = 0x1003u;
    g_scr.subsys_enable = 3u;
    control_mode_sync();
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_B));
    TEST_ASSERT_EQUAL_HEX16(0x1000u, g_mode.enable_flags);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.subsys_enable);
}

/**
 * \brief           Staging-frame boot defaults
 * \note            +0xA clears bits 0..9 and 11, sets 14; +0x10 sets 14 and 12, clears 0..11;
 *                      +0xC = 0x1400; +0xE = 0x5A00. It also clears the drive-fault state (staging
 *                      +0x19 bits 2..3) and sets the regen byte to 45.
 */
static void test_staging_defaults(void) {
    g_mode.mode_flags = 0xFFFFu;
    g_mode.enable_flags = 0xFFFFu;
    g_mode.curr_word = 0xFFFFu;
    g_mode.we_word = 0xFFFFu;
    g_mode.cfg_17 = 0xFFu;
    control_esc_staging_defaults();
    TEST_ASSERT_EQUAL_HEX8(0xF3u, g_mode.cfg_17);     /* +0x19 &= ~0xC */
    TEST_ASSERT_EQUAL_UINT8(45u, g_mode.regen_level); /* +0xE = 0x2D */
    TEST_ASSERT_EQUAL_HEX16(0xF400u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_HEX16(0xF000u, g_mode.enable_flags);
    TEST_ASSERT_EQUAL_HEX16(0x1400u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A00u, g_mode.we_word);

    g_mode.mode_flags = 0u;
    g_mode.enable_flags = 0u;
    control_esc_staging_defaults();
    TEST_ASSERT_EQUAL_HEX16(0x4000u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_HEX16(0x5000u, g_mode.enable_flags);
}

/**
 * \brief           IN2+0x3A bit0 is mirrored into MODE+0xA bit0, unless the climb boost is enabled
 * \note            `if (IN2+0x3A bit0 && !(IN2+0x3C bit2)) MODE+0xA |= 1; else &= ~1;`
 */
static void test_status_bit_mirror(void) {
    g_mode.mode_flags = 0x8000u;
    g_in2.f.cfg_flags = 1u;
    control_status_bit_mirror();
    TEST_ASSERT_EQUAL_HEX16(0x8001u, g_mode.mode_flags);
    g_in2.f.cfg_flags = 0xFFFEu;
    control_status_bit_mirror();
    TEST_ASSERT_EQUAL_HEX16(0x8000u, g_mode.mode_flags);
    g_in2.f.cfg_flags = 1u;
    g_in2.f.cfg_flags2 = CFG2_CLIMB_BOOST;
    control_status_bit_mirror();
    TEST_ASSERT_EQUAL_HEX16(0x8000u, g_mode.mode_flags);
}

/* -- charge detection with the subsystem current sense (control_charge_fsm) -------------- */

/**
 * \brief           Charger A: subsystem A on, PA5 sensed while PB10 is low; the 6th average
 *                      below 400 (5 samples summed, >> 2) sets the detect latch and event 0x18
 * \note            (VS+0x24 == 0, chg+2 == 1, chg+1 == 0): control_trigger_out,
 *                      vehicle_event9_prep (PB10 low, mask A); `if (!(*0x40010C0C bit10)) { s = sense A;
 *                      acc2 += s; if (3 < chg+6++) { acc2 >>= 2; if (acc2 < 400) { if (4 < acc0++)
 *                      { acc0 = 0; chg+5 = 1; } } else acc0 = 0; acc2 = 0; chg+6 = 0; } }`;
 *                      with chg+5 set control_event_set(0x18).
 */
static void test_charge_sense_detects(void) {
    g_chg.f2 = 1u;
    tst_control_adc_set(VCU_ADC_SLOT_ISENSE_PB10, CHG_SENSE_OK);
    tst_control_adc_set(VCU_ADC_SLOT_ISENSE_PB0, 0u);
    prv_repeat(control_charge_fsm, (CHG_AVERAGES * CHG_SAMPLES) - 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_B));
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f5);
    control_charge_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, g_chg.f5);
    TEST_ASSERT_EQUAL_HEX16(0x0080u, g_scr.event_mask_hi); /* event 0x18 */
    TEST_ASSERT_EQUAL_HEX16(0u, g_chgbuf.acc[1]);          /* PB0 high: sense B never sampled */
}

/**
 * \brief           An average of 400 or more restarts the count; after 43 ticks the window
 *                      closes and both subsystems are released
 * \note            `else acc0 = 0`; `if (chg+8 < 0x2b) {...} else { if (!chg+5)
 *                      buzzer_off(); control_mode_sync(); chg+6 = chg+7 = 0; ... }` and chg+8 counts
 *                      every tick while chg+5 is clear (reset past 0x55).
 */
static void test_charge_sense_high_current_window_closes(void) {
    g_chg.f2 = 1u;
    tst_control_adc_set(VCU_ADC_SLOT_ISENSE_PB10, CHG_SENSE_HIGH);
    prv_repeat(control_charge_fsm, 0x2Bu);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f5);
    TEST_ASSERT_EQUAL_UINT16(0u, g_chgbuf.acc[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    control_charge_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi);
}

/**
 * \brief           Charger gone with the timer at 0: outputs released, event 0x18 cleared
 * \note            `else if (chg+8 == 0) { control_mode_sync(); control_event_clear(0x18);
 *                      acc1 = acc0 = 0; chg+1 = chg+0 = chg+5 = chg+6 = chg+7 = 0; }`.
 */
static void test_charge_removed_clears(void) {
    control_event_set(CHG_EVENT);
    control_subsys_a_enable(1);
    g_chg.f5 = 1u;
    g_chg.f0 = 1u;
    g_chgbuf.acc[0] = 3u;
    control_charge_fsm();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f5);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f0);
    TEST_ASSERT_EQUAL_UINT16(0u, g_chgbuf.acc[0]);
}

/**
 * \brief           Lamp flash sequence: 250 count-down, both subsystems on 100..61 and 40..1,
 *                      off 60..41 and at 0 (sequence ends)
 * \note            (VS+0x24 != 0, IN2+0x3C bit14): state 1 -> count 0xFA,
 *                      state 2; state 2: `cnt--; if (cnt < 0x65) { if (0x3c < cnt) on; else if
 *                      (cnt < 0x29) { if (cnt == 0) { off; state 0 } else on } else off }`;
 *                      on/off = control_subsys_a/b_enable(1/0) + MODE+0x10 |= 3 / &= 0xFFFC.
 */
static void test_flash_sequence(void) {
    g_in2.f.cfg_flags2 = CFG2_LAMP_FLASH;
    g_vs.flash_seq = FLASH_SEQ_START;
    control_charge_fsm();
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_RUN, g_vs.flash_seq);
    TEST_ASSERT_EQUAL_UINT32(250u, g_chgbuf.count);

    prv_repeat(control_charge_fsm, 149u); /* count 101 */
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    control_charge_fsm(); /* 100 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_B));
    TEST_ASSERT_EQUAL_HEX16(3u, g_mode.enable_flags & 3u);
    prv_repeat(control_charge_fsm, 39u); /* 61 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    control_charge_fsm(); /* 60 */
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.enable_flags & 3u);
    prv_repeat(control_charge_fsm, 20u); /* 40 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    prv_repeat(control_charge_fsm, 39u); /* 1 */
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_RUN, g_vs.flash_seq);
    control_charge_fsm(); /* 0 */
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_IDLE, g_vs.flash_seq);
}

/**
 * \brief           Locate flash (0xEE): on for counts 1..19, off 20..39, wraps at 40; keep-awake 200;
 *                      without IN2+0x3C bit14 any other sequence aborts
 * \note            `cnt++; VS+0x34 = 200; if (0x13 < cnt) { if (0x27 < cnt) cnt = 0;
 *                      else off } else on`; `if (!(IN2+0x3C bit14) && VS+0x24 != 0xEE) VS+0x24 = 0`.
 */
static void test_locate_flash(void) {
    g_vs.flash_seq = FLASH_SEQ_LOCATE;
    control_charge_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT16(200u, g_vs.keepalive_ticks);
    prv_repeat(control_charge_fsm, 18u); /* 19 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, PIN_SUBSYS_A));
    control_charge_fsm(); /* 20 */
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    prv_repeat(control_charge_fsm, 20u); /* 40 */
    TEST_ASSERT_EQUAL_UINT32(0u, g_chgbuf.count);
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_LOCATE, g_vs.flash_seq);

    g_vs.flash_seq = FLASH_SEQ_RUN;
    control_charge_fsm();
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_IDLE, g_vs.flash_seq);
}

/* -- beep player (control_gesture_fsm) ------------------------------------------------------ */

/**
 * \brief           Short beep: on at tick 0, off at tick 10; long beep: off after tick 50
 * \note            Case 2: `if (t == 0) on; else if (9 < t) { id = 0; off; }
 *                      t++`; case 1: `... else if (0x32 < t) { t = 0; id = 0; off; } t++`.
 */
static void test_beep_short_and_long(void) {
    buzzer_beep(BEEP_SHORT);
    control_gesture_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    prv_repeat(control_gesture_fsm, 9u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    control_gesture_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    buzzer_beep(BEEP_LONG);
    prv_repeat(control_gesture_fsm, 51u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    control_gesture_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_UINT16(1u, g_lowglob.beep_seq_tick);
}

/**
 * \brief           Double beep: on 0..9, off 10..19, on 20..29, off at 30; stop silences at once
 * \note            Case 3 (on 0, off 10, on 0x14, end > 0x1D); case 6: id = 0, off.
 */
static void test_beep_double_and_stop(void) {
    buzzer_beep(BEEP_DOUBLE);
    prv_repeat(control_gesture_fsm, 10u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    control_gesture_fsm(); /* t = 10 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    prv_repeat(control_gesture_fsm, 10u); /* t = 20 */
    TEST_ASSERT_EQUAL_UINT8(1u, prv_buzzer_on());
    prv_repeat(control_gesture_fsm, 10u); /* t = 30 */
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    buzzer_on(BUZZER_TONE);
    buzzer_beep(BEEP_STOP);
    control_gesture_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           A charger present with IN2+0x3A bit11 holds the player reset
 * \note            `if ((chg+3 == 1 || chg+2 == 1) && IN2+0x3A bit11)
 *                      { id = 0; t = 0; }` (no buzzer call).
 */
static void test_beep_held_while_charging(void) {
    g_chg.f3 = 1u;
    g_in2.f.cfg_flags = 0x0800u;
    buzzer_beep(BEEP_SHORT);
    control_gesture_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_buzzer_on());
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.beep_seq_tick);
}

/* -- mode / lighting report (control_mode_report_fsm) -------------------------------------- */

/**
 * \brief           Levels 0/1/2 replace the status and report it; 3 and other codes do nothing
 * \note            `if (p < 5) { if (p not in {0,1,2}) return; SCR+0x1AA = p;
 *                      vcu_usart_report_send(ctx, 0x16, 0x23, 2, 3, 0xD5, &SCR+0x1AA); }`; > 4 other than 8 /
 *                      0x10 returns.
 */
static void test_mode_report_levels(void) {
    g_scr.mode_status = 0x18u;
    (void)control_mode_report_fsm(1u);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.mode_status);
    (void)control_mode_report_fsm(3u);
    (void)control_mode_report_fsm(0x20u);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.mode_status);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    TEST_ASSERT_EQUAL_HEX16(1u, prv_mode_report(0u));
}

/**
 * \brief           Flags 8 / 0x10 keep the level bits, reload VS+0x36 = 300 and report on a change
 *                      only; 4 reports the status with bit 2 set, then clears bit 2
 * \note            `if ((SCR+0x1AA & 0x18) != p) { SCR+0x1AA = (SCR+0x1AA & 3) | p;
 *                      VS+0x36 = 300; vcu_usart_report_send(...); }`; `p == 4: |= 4; vcu_usart_report_send; &= 0xFFFB`
 *                      then the same flag compare with p = 4.
 */
static void test_mode_report_flags(void) {
    g_scr.mode_status = 2u;
    (void)control_mode_report_fsm(8u);
    TEST_ASSERT_EQUAL_HEX16(0x0Au, g_scr.mode_status);
    TEST_ASSERT_EQUAL_UINT16(300u, g_vs.mode_timeout);
    g_vs.mode_timeout = 0u;
    (void)control_mode_report_fsm(8u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.mode_timeout);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    TEST_ASSERT_EQUAL_HEX16(0x0Au, prv_mode_report(0u));

    (void)control_mode_report_fsm(4u);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    TEST_ASSERT_EQUAL_HEX16(0x0Eu, prv_mode_report(1u));
    TEST_ASSERT_EQUAL_HEX16(0x06u, prv_mode_report(2u));
    TEST_ASSERT_EQUAL_HEX16(0x06u, g_scr.mode_status);
}

/* -- brake-hold fault FSM (control_fault_drive_fsm) ---------------------------------------- */

/**
 * \brief           Fault arms after 101 standstill ticks, trips after 300 more with the brake held;
 *                      a second press then a release recovers
 * \note            (IN2+0x3A bit5, IN1+0x1F/+0x2C clear): speed 0 -> TMR+0x1A
 *                      counts (reset by IN1+0xB), `100 < c` -> TMR+0xA = 1; with IN1+0xB == 1 and
 *                      VS+4 == 0: TMR+0x18 < 300 counts, else (SoC >= 6) buzzer_beep(3), TMR+9 = 1,
 *                      SCR+0x56 = 1, MODE+0xA |= 4|8, VS+4 = 1; brake released: TMR+0x18 = TMR+9 = 0;
 *                      VS+4 == 1 and TMR+9 == 0 on the next press -> 2; release in 2 -> buzzer_beep(1),
 *                      SCR+0x56 = 0, VS+4 = 0, MODE+0xA &= 0xFFF7, control_mode_report_fsm(0).
 */
static void test_brake_hold_fault_cycle(void) {
    g_in2.f.cfg_flags = 0x0020u;
    g_stat.soc = 50;
    prv_repeat(control_fault_drive_fsm, 101u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ovt_active);
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.ovt_active);

    g_in1.brake_any = 1u;
    prv_repeat(control_fault_drive_fsm, 300u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.drive_fault_state);
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.drive_fault_state);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.drive_fault_req);
    TEST_ASSERT_EQUAL_HEX16(0x000Cu, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX8(0x04u, g_mode.cfg_17 & CFG17_DRIVE_FAULT); /* +0x19 = (& ~0xC) + 4 */

    g_in1.brake_any = 0u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.drive_fault_state);
    g_in1.brake_any = 1u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.drive_fault_state);
    g_in1.brake_any = 0u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.drive_fault_state);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.drive_fault_req);
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.mode_status);
    TEST_ASSERT_EQUAL_HEX8(0u, g_mode.cfg_17 & CFG17_DRIVE_FAULT); /* +0x19 &= ~0xC */
}

/**
 * \brief           Tripped with the brake held: the drive-fault state goes to 3 while the vbus code
 *                      is above 150 (and stays 3 after it drops)
 * \note            VS+4 == 1 and brake: `if (150 < A3C+2 >> 8) +0x19 |= 0xC;
 *                      if (TMR+9 == 0) VS+4 = 2;`.
 */
static void test_brake_hold_fault_vbus_state(void) {
    g_in2.f.cfg_flags = 0x0020u;
    g_stat.soc = 50;
    g_tmr.ovt_active = 1u;
    g_tmr.ovt_flag = 1u;
    g_vs.drive_fault_state = 1u;
    g_mode.cfg_17 = 0x04u;
    g_in1.brake_any = 1u;
    g_fault_codes.vbus_code = 0x9600u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_HEX8(0x04u, g_mode.cfg_17 & CFG17_DRIVE_FAULT);
    g_fault_codes.vbus_code = 0x9700u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_HEX8(0x0Cu, g_mode.cfg_17 & CFG17_DRIVE_FAULT);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.drive_fault_state);
    g_fault_codes.vbus_code = 0u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_HEX8(0x0Cu, g_mode.cfg_17 & CFG17_DRIVE_FAULT);
}

/**
 * \brief           At SoC below 6 % the expiry only reports flag 8; a lever fault resets a trip
 * \note            `if (STAT+0x11E < 6) control_mode_report_fsm(8);`; with IN1+0x1F or
 *                      +0x2C set and VS+4 != 0: SCR+0x56 = 0, VS+4 = 0, MODE+0xA &= 0xFFF7,
 *                      control_mode_report_fsm(0).
 */
static void test_brake_hold_low_soc_and_lever_fault(void) {
    g_in2.f.cfg_flags = 0x0020u;
    g_stat.soc = 5;
    g_tmr.ovt_active = 1u;
    g_tmr.ovt_timer = 300u;
    g_in1.brake_any = 1u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.drive_fault_state);
    TEST_ASSERT_EQUAL_HEX16(8u, g_scr.mode_status);

    g_vs.drive_fault_state = 1u;
    g_mode.mode_flags = 0x000Cu;
    g_scr.drive_fault_req = 1u;
    g_in1.brake_half_range_fault = 1u;
    control_fault_drive_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.drive_fault_state);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.drive_fault_req);
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.mode_status);
}

/* -- lock FSM (control_lock_fsm) ---------------------------------------------------------- */

/**
 * \brief           Unlocked at standstill: VS+0x30 counts to 600 (long beep at 600), then the
 *                      lock request SCR+0x3E is raised
 * \note            `if ((short)IN2+0x84 < 3 && VS+2 < 3 && IN1+0x26 == 0) {
 *                      if (VS+0x30 < 600) { VS+0x30++; if (== 600) buzzer_beep(1); } else SCR+0x3E = 1; }
 *                      else VS+0x30 = 0;`. A brake lever holds the timer at 0 (`if (IN1+0xB)
 *                      VS+0x1E = 0; else VS+0x1E++;`).
 */
static void test_lock_timer_requests_lock(void) {
    prv_repeat(control_lock_fsm, 599u);
    TEST_ASSERT_EQUAL_UINT16(599u, g_vs.lock_timer);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.lock_req);
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.lock_req);

    g_vs.speed_kmh = VCU_STANDSTILL_KMH;
    g_scr.lock_req = 0u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.lock_timer);

    g_vs.speed_kmh = 0u;
    prv_repeat(control_lock_fsm, 5u);
    TEST_ASSERT_EQUAL_UINT16(5u, g_vs.lock_timer);
    g_in1.brake_any = 1u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.lock_timer);
}

/**
 * \brief           IN2+0x84 is compared signed: a negative value counts like 0..2
 * \note            Both the timer branch and the raise-to-3 branch compare IN2+0x84 as a signed
 *                      halfword (`>= 3`).
 */
static void test_lock_profile_reg_signed(void) {
    g_in2.f.fault_code = 0xFFFFu;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.lock_timer);

    g_vs.region = VCU_REGION_EUROPE;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(3u, g_in2.f.fault_code);
}

/**
 * \brief           Riding (VS+0 != 0) raises IN2+0x84 to 3 and drops the request
 * \note            `else { if ((short)IN2+0x84 < 3) IN2+0x84 = 3; SCR+0x3E = 0; }`.
 */
static void test_lock_riding_drops_request(void) {
    g_vs.region = VCU_REGION_EUROPE;
    g_scr.lock_req = 1u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(3u, g_in2.f.fault_code);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.lock_req);
}

/**
 * \brief           A requested lock: a brake press cancels it (double beep, status 0); the
 *                      throttle shows status 1, or 8 with a charger
 * \note            `SCR+0x3E == 1`: IN1+0xC == 0 or status 1 -> IN1+0xB set ->
 *                      (IN1+0x25 ? control_mode_report_fsm(8) : buzzer_beep(3), VS+0x30 = 0, SCR+0x3E = 0,
 *                      control_mode_report_fsm(0)); throttle -> IN1+0x25 ? 8 : control_mode_report_fsm(1).
 */
static void test_lock_request_cancel_and_throttle(void) {
    g_vs.lock_timer = 600u;
    g_scr.lock_req = 1u;
    g_in1.brake_any = 1u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.lock_req);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.lock_timer);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);

    g_in1.brake_any = 0u;
    g_vs.lock_timer = 600u;
    g_in1.throttle_flag = 1u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.lock_req);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.mode_status);

    g_scr.mode_status = 0u;
    g_in1.charger_connected = 1u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(8u, g_scr.mode_status);
}

/**
 * \brief           Lock-out gate: speed >= IN2+0x84 sets MODE+0xA bit2 at once; at standstill it is
 *                      cleared after the 0x32-tick divider, at once with the throttle
 * \note            Tail: `if (IN2+0xB0 == 0) { if ((short)VS+2 < IN2+0x84 &&
 *                      IN1+0x26 == 0) { if (IN1+0xB == 0) TMR+7 = 0; } else TMR+7 = 1; }`; TMR+7 == 0:
 *                      `TMR+8 < 0x32 ? TMR+8++ : MODE+0xA &= 0xFFFB`, else TMR+8 = 0, MODE+0xA |= 4.
 *                      The release needs VS+2 == 0 too (`else if (!IN1+0xB && VS+2 == 0)
 *                      TMR+8 = 0`); the throttle (IN1+0xC) clears bit2 without the divider.
 */
static void test_lock_out_gate(void) {
    g_in2.f.fault_code = 3u;
    g_vs.region = VCU_REGION_EUROPE; /* keep the timer branch out of the way */
    g_vs.speed_kmh = 3u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(4u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.lock_div);

    g_vs.speed_kmh = 2u; /* below IN2+0x84 but rolling: the gate holds */
    prv_repeat(control_lock_fsm, 0x40u);
    TEST_ASSERT_EQUAL_HEX16(4u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.lock_div);

    g_vs.speed_kmh = 0u;
    prv_repeat(control_lock_fsm, 0x32u);
    TEST_ASSERT_EQUAL_HEX16(4u, g_mode.mode_flags);
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags);

    g_vs.speed_kmh = 3u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(4u, g_mode.mode_flags);
    g_vs.speed_kmh = 0u;
    g_in1.throttle_flag = 1u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags);
}

/**
 * \brief           MODE+0x17 bit1 holds the standby timer at 0 and forces the lock-out bit
 * \note            `if (MODE+0x17 bit1) VS+0x1E = 0;` (before the timer count) and
 *                      `if (TMR+8 != 0 || MODE+0x17 bit1) { TMR+9 = 0; MODE+0xA |= 4; }`.
 */
static void test_lock_coast_preset_bit(void) {
    g_mode.output_cfg = OCFG_COAST_PRESET;
    prv_repeat(control_lock_fsm, 3u);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.lock_timer); /* cleared, then counted once */

    g_in2.f.fault_code = 3u; /* at rest below IN2+0x84: lock-out released, only bit1 holds the gate */
    g_tmr.lock_div = 7u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(4u, g_mode.mode_flags);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.lock_div);
    g_mode.output_cfg = 0u;
    control_lock_fsm();
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.lock_div);
}

/* -- power monitor (control_power_monitor_fsm) --------------------------------------------- */

/**
 * \brief           SoC mirror (0 reads 1) and, in ride, power = |I*V/100| / 2000 capped at 100
 * \note            SCR+0x50 = STAT+0x11E (single pack), 0 -> 1, IN2+0xAA and
 *                      SCR+0x2E mirror it; state 3: `p = I*V/100; if (p < 1) { p = -p; SCR+6 = 0; }
 *                      else SCR+6 = 1; SCR+2 = p / 2000; if (100 < SCR+2) SCR+2 = 100;` else SCR+2 = 0.
 */
static void test_power_monitor_power_and_soc(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_stat.soc = 0;
    g_stat.curr_sample = 1000;
    g_stat.volt_sample = 5000; /* 50000 -> 25 */
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.soc_combined);
    TEST_ASSERT_EQUAL_UINT16(1u, g_in2.f.soc);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.soc_report);
    TEST_ASSERT_EQUAL_UINT16(25u, g_scr.power_half);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.power_sign);

    g_stat.soc = 77;
    g_stat.curr_sample = -30000;
    g_stat.volt_sample = 30000;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT16(77u, g_in2.f.soc);
    TEST_ASSERT_EQUAL_UINT16(100u, g_scr.power_half);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.power_sign);

    g_state_block.fsm_state = VCU_FSM_IDLE;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.power_half);

    /* SCR+0x50 = max(STAT+0x11E, 1); no second-pack average, and a negative SoC reads 1 too. */
    g_in2.f.status_flags = SF_DUAL_PACK;
    g_stat.soc = 60;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT16(60u, g_scr.soc_combined);
    g_stat.soc = -3;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.soc_combined);
}

/**
 * \brief           The power monitor runs the battery-2 presence tick first: with battery 2 present
 *                      its SoC is copied to SCR+0x0E
 * \note            The battery-2 presence tick (SCR+0x0A != 0 -> SCR+0x0E = battery-2
 *                      STAT+0x11E) runs before the SoC mirror.
 */
static void test_power_monitor_runs_battery2_tick(void) {
    g_scr.bat2_present = 1u;
    g_stat2.soc = 42;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT16(42u, g_scr.bat2_soc);
}

/**
 * \brief           In ride, after 20 warm-up ticks a comm fault (A3C bit1) raises event 1 and
 *                      starts a 600-tick dwell; below 100 left the VCU requests shutdown
 * \note            `if (OS+0xC < 0x14) OS+0xC++; else { ... if (A3C bit1) {
 *                      if (OS+0xE == 0) OS+0xE = 600; control_event_set(1); } else { OS+0xE = 0; SCR+8 = 0;
 *                      control_event_clear(1); } if (OS+0xE) { t = --OS+0xE; SCR+8 = t / 100; if (t < 100)
 *                      { OS+0xE = 0; VS+0x1C = 1; } } }`; outside state 3: OS+0xC = 0, control_event_clear(1).
 */
static void test_power_monitor_comm_timeout(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_stat.soc = 50;
    g_fault_codes.comm_fault = 0x02u;
    prv_repeat(control_power_monitor_fsm, 0x14u);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo);
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.commto_mirror);
    prv_repeat(control_power_monitor_fsm, 499u); /* dwell 100 */
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
    control_power_monitor_fsm(); /* dwell 99 */
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);

    g_state_block.fsm_state = VCU_FSM_IDLE;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_UINT16(0u, g_oneshot.commto_tick);
}

/**
 * \brief           SoC falling from >= 6 to < 6 in ride beeps twice long and reports flag 0x10
 * \note            `if ((short)IN2+0xAA < 6 && 5 < OS+7) { OS+0xE = 0;
 *                      buzzer_beep(5); control_mode_report_fsm(0x10); } OS+7 = IN2+0xAA;`.
 */
static void test_power_monitor_low_soc_warning(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_stat.soc = 6;
    prv_repeat(control_power_monitor_fsm, 0x15u);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    g_stat.soc = 5;
    control_power_monitor_fsm();
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0x10u, g_scr.mode_status & 0x18u);
}

/* -- drive-parameter seed (control_drive_param_init) --------------------------------------- */

/**
 * \brief           Register seeds and the per-row profile table
 * \note            Base IN2: reg A (+0x3A) ends with bits 4 and 11 only, reg B
 *                      (+0x3C) sets 0,8,9,11 and clears 1..7,10,12..14, reg C (+0x3E) sets 0,1,8 and
 *                      clears 3..7,9,11; VS+5 = 3; +0x92 = 5, +0xE0 = 1, +0xE8 = 1, +0x96 = 0,
 *                      +0x98 = 0x100; IN1+0x28 = 1; STAT+0x104 low byte = 100; row 0: +0x84 = 0,
 *                      +0x8E = 0x19, +0x90 = +0x8C = 0x3232; row 1: 3, 0xF, 0x1919; row 2: 3, 0xF0F,
 *                      0x1414; +0x86 = +0x88 = 0x505, +0x8A = +0x8E. It also zeroes reg 0x75
 *                      (+0xEA) and reg 0x2E (+0x5C), clears reg 0x2F (+0x5E) bit0, and bit2 in row 0.
 */
static void test_drive_param_init(void) {
    g_in2.f.cfg_flags = 0xFFFFu;
    g_in2.f.cfg_flags2 = 0xFFFFu;
    g_in2.f.chg_cfg = 0xFFFFu;
    g_stat.chg_limit = 0xAB55u;
    g_in2.f.wobble_sens = 2u;
    g_in2.f.reg_2e = 0x1234u;
    g_in2.f.in2_5e = 0xFFFFu;
    control_drive_param_init();
    TEST_ASSERT_EQUAL_UINT16(0u, g_in2.f.wobble_sens);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.reg_2e);
    TEST_ASSERT_EQUAL_HEX16(0xFFFAu, g_in2.f.in2_5e);  /* bit0, and bit2 in row 0 */
    TEST_ASSERT_EQUAL_HEX16(0x0810u, g_in2.f.cfg_flags);
    TEST_ASSERT_EQUAL_HEX16(0x8B01u, g_in2.f.cfg_flags2);
    TEST_ASSERT_EQUAL_HEX16(0xF507u, g_in2.f.chg_cfg);
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.substate);
    TEST_ASSERT_EQUAL_UINT16(5u, g_in2.f.auto_off_min);
    TEST_ASSERT_EQUAL_UINT16(1u, g_in2.f.dir_bits);
    TEST_ASSERT_EQUAL_UINT16(1u, g_in2.f.alarm_sensitivity);
    TEST_ASSERT_EQUAL_HEX16(0x0100u, g_in2.f.chg_sched_end);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.bms_chg_cmd_req);
    TEST_ASSERT_EQUAL_HEX16(0xAB64u, g_stat.chg_limit);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.fault_code);
    TEST_ASSERT_EQUAL_HEX16(0x0019u, g_in2.f.gear_a_ceiling);
    TEST_ASSERT_EQUAL_HEX16(0x3232u, g_in2.f.speed_limit_2);
    TEST_ASSERT_EQUAL_HEX16(0x3232u, g_in2.f.speed_limit);
    TEST_ASSERT_EQUAL_HEX16(0x0505u, g_in2.f.prof_86);

    g_vs.region = VCU_REGION_EUROPE;
    g_in2.f.in2_5e = 0xFFFFu;
    control_drive_param_init();
    TEST_ASSERT_EQUAL_HEX16(0xFFFEu, g_in2.f.in2_5e); /* rows 1/2 keep bit2 */
    TEST_ASSERT_EQUAL_HEX16(3u, g_in2.f.fault_code);
    TEST_ASSERT_EQUAL_HEX16(0x000Fu, g_in2.f.prof_8a);
    TEST_ASSERT_EQUAL_HEX16(0x1919u, g_in2.f.speed_limit);

    g_vs.region = VCU_REGION_GERMANY;
    control_drive_param_init();
    TEST_ASSERT_EQUAL_HEX16(0x0F0Fu, g_in2.f.gear_a_ceiling);
    TEST_ASSERT_EQUAL_HEX16(0x1414u, g_in2.f.speed_limit_2);
}

/* -- charging helpers ------------------------------------------------------------------------ */

/**
 * \brief           Below 100 % and below the charge limit nothing is complete; at either the
 *                      charge is full, the refresh one-shot fires once and the 6-step beep runs
 * \note            `if (soc < 100 && soc < (short)(STAT+0x104 & 0xff) && VS+0x19 == 0)
 *                      { OS+5 = OS+6 = VS+0x1A = 0; return; }` SCR+0x10 = 3, VS+0x1A = 1, OS+6 one-shot
 *                      -> VS+0x20 = 1, IN2+0x3E &= ~0x10; step 0: beep 2 when the beep player is idle, steps 1..4
 *                      advance while idle, step 5: beep 1 -> step 6.
 */
static void test_charge_complete_beeps(void) {
    uint32_t idx;

    g_stat.chg_limit = 0x0050u;
    g_stat.soc = 79;
    g_oneshot.beep_counter = 3u;
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_full);
    TEST_ASSERT_EQUAL_UINT8(0u, g_oneshot.beep_counter);

    g_stat.soc = 80;
    g_in2.f.chg_cfg = CHG_SCHED_EN;
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_full);
    TEST_ASSERT_EQUAL_UINT16(3u, g_scr.drive_oneshot);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_disp_refresh_req);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.chg_cfg);
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT8(1u, g_oneshot.beep_counter);

    g_vs.chg_disp_refresh_req = 0u;
    charge_complete_tick(); /* beep still playing: no step */
    TEST_ASSERT_EQUAL_UINT8(1u, g_oneshot.beep_counter);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_disp_refresh_req);
    g_lowglob.beep_seq_id = BEEP_NONE;
    for (idx = 0u; idx < 4u; ++idx) {
        charge_complete_tick();
    }
    TEST_ASSERT_EQUAL_UINT8(5u, g_oneshot.beep_counter);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(6u, g_oneshot.beep_counter);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    g_lowglob.beep_seq_id = BEEP_NONE;
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(6u, g_oneshot.beep_counter);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           At the limit the charge is complete only once the charger reports done (IN1+0x25
 *                      clear); a schedule stop completes it regardless
 * \note            `if ((soc >= 100 || soc >= limit) && IN1+0x25 == 0) complete;
 *                      else if (VS sched-stop) complete; else reset;`. IN1+0x25 is charger_connected.
 */
static void test_charge_complete_waits_for_charger(void) {
    g_stat.chg_limit = 0x0050u;
    g_stat.soc = 100;
    g_in1.charger_connected = 1u;
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_full);
    g_vs.chg_sched_stop = 1u;
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_full);
    g_vs.chg_sched_stop = 0u;
    g_in1.charger_connected = 0u;
    charge_complete_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_full);
}

/**
 * \brief           The BMS charge-control push packs the schedule window (IN2+0x96/+0x98) and the
 *                      reg 0x1F bits 7/4/5 into a 4-byte op 4 arg 0x87 report to node 7
 * \note            word = start lo6 | start hi5 << 6 | end lo6 << 11 | end hi5 << 17 |
 *                      IN2+0x3E bit7 << 24 | bit4 << 22 | bit5 << 23; report_ninebot(0x16, 7, 4, 2, 0x87,
 *                      &word) on the 11th call (divider ONESHOT+1). The base register is IN2, not SCR.
 */
static void test_charger_report_packs_schedule(void) {
    static const uint8_t expect[7] = {0x01u, 0x02u, 0x87u, 0x97u, 0x2Du, 0xCCu, 0x00u};
    const tst_can_frame_t* frame;
    uint32_t pass;

    tst_can_tx_forward(0u);
    vcu_can_init();
    g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND;
    g_in2.f.chg_sched_start = 0x1617u;
    g_in2.f.chg_sched_end = 0x0605u;
    g_in2.f.chg_cfg = CHG_SCHED_EN | CHG_SCHED_ONCE;
    prv_repeat(charger_detect_tick, 10u);
    for (pass = 0u; pass < 8u; ++pass) {
        vcu_can_poll();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
    charger_detect_tick();
    for (pass = 0u; pass < 8u; ++pass) {
        vcu_can_poll();
    }
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_EQUAL_HEX32(0x18EF0716u, frame->id);
    TEST_ASSERT_EQUAL_UINT8(7u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame->data, sizeof(expect));
    TEST_ASSERT_EQUAL_UINT8(30u, g_oneshot.mode_hold_countdown);
}

/**
 * \brief           Without the BMS charger bit the schedule stop is dropped on every call
 * \note            `VS sched-stop = 0` runs whenever STAT+0x126 bit3 is clear.
 */
static void test_charger_absent_clears_schedule_stop(void) {
    g_vs.chg_sched_stop = 1u;
    charger_detect_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_stop);
}

/**
 * \brief           The schedule-cancel notice lasts 60 report periods, 30 with a charger connected
 * \note            `n = ++ONESHOT+2; if (n >= 60 || (IN1+0x25 == 1 && n >= 30))
 *                      { ONESHOT+2 = 0; VS cancel = 0; IN2+0x3E &= ~0x10; ... }`.
 */
static void test_charger_cancel_notice_length(void) {
    uint32_t idx;

    g_vs.chg_sched_cancel_notice = 1u;
    for (idx = 0u; idx < 59u; ++idx) {
        g_oneshot.report_divider = 10u;
        charger_detect_tick();
    }
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_sched_cancel_notice);
    g_oneshot.report_divider = 10u;
    charger_detect_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_cancel_notice);

    g_stat.status_word_a = 0x0040u; /* charger connected: STAT+0x124 bit6 + STAT+0x126 bit3 */
    g_stat.status_word_b = 0x0008u;
    g_vs.chg_sched_cancel_notice = 1u;
    for (idx = 0u; idx < 29u; ++idx) {
        g_oneshot.report_divider = 10u;
        charger_detect_tick();
    }
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_sched_cancel_notice);
    g_oneshot.report_divider = 10u;
    charger_detect_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_cancel_notice);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_disp_refresh_req);
}

/**
 * \brief           Scheduled charging: the hour (STAT+0x1FC bits 12..16) is checked every 52nd tick;
 *                      inside [end, start) of an overnight window charging stops
 * \note            `if (IN2+0x3E bit4 && IN1+0x22) { c = OS+8++; if (0x32 < c) {
 *                      OS+8 = 0; h = (STAT+0x1FC & 0x1ffff) >> 12; start < end ? (end <= h -> stop) :
 *                      (end <= h && h < start -> stop); ...} } else { OS+8 = 0; IN1+0x22 = 0; }`,
 *                      stop = IN1+0x28 = 1, IN1+0x22 = 0, VS+0x21 = 1, VS+0x19 = 1, IN2+0x3E &= ~0x10.
 */
static void test_charge_schedule_window(void) {
    g_in2.f.chg_cfg = CHG_SCHED_EN;
    g_in1.chg_sched_armed = 1u;
    g_in2.f.chg_sched_start = 0x1600u; /* 22:00 */
    g_in2.f.chg_sched_end = 0x0600u;   /* 06:00 */
    g_stat.status_packed = 7u << 12;
    prv_repeat(charge_schedule_tick, 0x33u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_stop);
    charge_schedule_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_sched_stop);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_disp_refresh);
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_SEND, g_in1.bms_chg_cmd_req);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.chg_cfg);
    TEST_ASSERT_EQUAL_UINT16(0u, g_oneshot.window_timer);

    /* Disabled: timer and arm cleared. */
    g_in1.chg_sched_armed = 1u;
    g_oneshot.window_timer = 9u;
    charge_schedule_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_oneshot.window_timer);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.chg_sched_armed);
}

/**
 * \brief           A single-shot schedule at/after its start hour clears bits 4, 5 and 7 without
 *                      stopping the charge; a same-day window stops at its end hour
 * \note            `if (start <= h && IN2+0x3E bit5) { IN1+0x28 = 1; IN1+0x22 = 0;
 *                      VS+0x21 = 1; IN2+0x3E &= 0xffef, 0xffdf, 0xff7f; }` (no VS+0x19).
 */
static void test_charge_schedule_single_shot(void) {
    g_in2.f.chg_cfg = CHG_SCHED_EN | CHG_SCHED_ONCE | CHG_KEYHOLD | CHG_TILT_LATCH_EN;
    g_in1.chg_sched_armed = 1u;
    g_in2.f.chg_sched_start = 0x1600u;
    g_in2.f.chg_sched_end = 0x0600u;
    g_stat.status_packed = 23u << 12;
    g_oneshot.window_timer = 0x33u;
    charge_schedule_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_stop);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_disp_refresh);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.chg_sched_armed);
    TEST_ASSERT_EQUAL_HEX16(CHG_TILT_LATCH_EN, g_in2.f.chg_cfg);

    g_in2.f.chg_cfg = CHG_SCHED_EN;
    g_in1.chg_sched_armed = 1u;
    g_in2.f.chg_sched_start = 0x0100u;
    g_in2.f.chg_sched_end = 0x0500u;
    g_stat.status_packed = 4u << 12;
    g_oneshot.window_timer = 0x33u;
    charge_schedule_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_stop);
    g_stat.status_packed = 5u << 12;
    g_oneshot.window_timer = 0x33u;
    charge_schedule_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_sched_stop);
}

/**
 * \brief           Charge-state reset releases the subsystems and clears the detect state
 * \note            control_mode_sync; chg+8 = 0, chg+4 = chg+3 = chg+2 = 0; EVT records 7 and 8
 *                      (+8) = 0; VS+0x24 = 0.
 */
static void test_charge_state_reset(void) {
    control_subsys_a_enable(1);
    g_chg.timer = 7u;
    g_chg.f2 = 1u;
    g_chg.f3 = 1u;
    g_chg.f4 = 1u;
    g_evt.mon[FM_MON_LIGHT_LO].code = 1u;
    g_evt.mon[FM_MON_LIGHT_HI].code = 1u;
    g_vs.flash_seq = FLASH_SEQ_LOCATE;
    control_charge_state_reset();
    TEST_ASSERT_EQUAL_UINT8(1u, prv_pin(GPIOB, PIN_SUBSYS_A));
    TEST_ASSERT_EQUAL_UINT16(0u, g_chg.timer);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f2);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f3);
    TEST_ASSERT_EQUAL_UINT8(0u, g_chg.f4); /* chg+4 = 0 */
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_LO].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_HI].code);
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_IDLE, g_vs.flash_seq);
}

/**
 * \brief           Diag reset drops the indicators and clears VS+5 unless it is 3
 * \note            vcu_outputs_reset (PB13/PA6/PB14 low); `if (VS+5 != 3) VS+5 = 0;`.
 */
static void test_diag_reset(void) {
    control_out_pb14(1);
    g_vs.substate = 2u;
    control_diag_reset();
    TEST_ASSERT_EQUAL_UINT8(0u, prv_pin(GPIOB, GPIO_PINS_14));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.substate);
    g_vs.substate = 3u;
    control_diag_reset();
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.substate);
}

/* -- speed / lock-arm supervisor (control_speed_lock_fsm) ---------------------------------- */

/**
 * \brief           Power-down clears the supervisor; otherwise the arm counter steps every 3rd tick
 *                      while -V/100 < MODE+0x13, clamped at 1200, mirrored to SCR+0x44
 * \note            State 4 -> TMR+0 = 0, MODE+0xA &= 0xFF7F, VS+0xC = 0, TMR+0x12 = 0;
 *                      `if (-V/100 < MODE+0x13 && 1 < TMR+0++) { TMR+0 = 0; if (1199 < VS+0xC++)
 *                      VS+0xC = 1200; }` (strict compare, every 3rd tick, 0x4B0); SCR+0x44 = TMR+0x12.
 */
static void test_speed_lock_arm_counter(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_mode.spd_cmp = 1u;
    prv_repeat(control_speed_lock_fsm, 2u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.lock_arm_ctr);
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.lock_arm_ctr);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.lock_subdiv);

    g_vs.lock_arm_ctr = 1200u;
    g_tmr.lock_subdiv = 2u;
    g_tmr.lock_packed_ctr = 0x1234u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(1200u, g_vs.lock_arm_ctr);
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_scr.lock_status_mirror);

    g_stat.volt_sample = -100; /* -V/100 = 1 == MODE+0x13: no stepping (strict) */
    g_tmr.lock_subdiv = 2u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_UINT8(2u, g_tmr.lock_subdiv);

    g_state_block.fsm_state = VCU_FSM_POWER_DOWN;
    g_mode.mode_flags = 0x0080u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.lock_arm_ctr);
    TEST_ASSERT_EQUAL_HEX16(0u, g_tmr.lock_packed_ctr);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags);
}

/**
 * \brief           A park ack in gear 4 with every guard met arms MODE+0xA bit7 and packs the
 *                      counter; a counter below 100 holds (dwell 200, long beep); other gears report 8
 * \note            EVT+0x50 == 1 and bit7 clear -> EVT+0x50 = 0; `VS+1 != 4 ->
 *                      control_mode_report_fsm(8)`; guards: active error code == 0, A3C halfword bit8,
 *                      (short)IN2+0xD6 < 0x1CC, VS+0xC >= 100, STAT+0x11E >= 0x14, STAT+0x122/100 >=
 *                      table col 0x10; pass -> bit7 set, buzzer_beep(3); fail -> TMR+0x14 = 200, bit7
 *                      cleared, TMR+0x12 = (& 0x8001) | 1, buzzer_beep(1); the armed tail packs
 *                      (VS+0xC/100) << 8 | (VS+0xC/12) << 1.
 */
static void test_speed_lock_park_ack(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_evt.mon[FM_MON_PARK].code = 1u;
    g_vs.gear_sel = 3u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_PARK].code);
    TEST_ASSERT_EQUAL_HEX16(8u, g_scr.mode_status);

    g_vs.gear_sel = 4u;
    g_fault_codes.comm_fault_word = 0x0100u;
    g_stat.soc = 0x14;
    g_stat.curr_limit = 4000;
    g_vs.lock_arm_ctr = 99u;
    g_evt.mon[FM_MON_PARK].code = 1u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_UINT16(200u, g_tmr.lock_ack_dwell);
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_tmr.lock_packed_ctr);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x80u);

    g_tmr.lock_ack_dwell = 0u;
    g_tmr.lock_packed_ctr = 0u;
    g_vs.lock_arm_ctr = 100u;
    g_evt.mon[FM_MON_PARK].code = 1u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(0x80u, g_mode.mode_flags & 0x80u);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0x0110u, g_tmr.lock_packed_ctr);
    TEST_ASSERT_EQUAL_HEX16(0x0110u, g_scr.lock_status_mirror);

    /* SCR+0x50 does not guard: STAT+0x11E below 20 releases the armed lock */
    g_scr.soc_combined = 100u;
    g_stat.soc = 19;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x80u);
}

/**
 * \brief           Live mode 3: the park-ack current guard uses row 0's accel base 0x28 (the row
 *                      index wraps)
 * \note            control.c indexes the table with VS[0] % 3. STAT+0x122 / 100 = 39 < 0x28 holds,
 *                      40 arms.
 */
static void test_speed_lock_mode3_wraps_to_row0(void) {
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_vs.region = 3u;
    g_vs.gear_sel = 4u;
    g_fault_codes.comm_fault_word = 0x0100u;
    g_stat.soc = 0x14;
    g_vs.lock_arm_ctr = 100u;
    g_stat.curr_limit = 3999;
    g_evt.mon[FM_MON_PARK].code = 1u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x80u);
    TEST_ASSERT_EQUAL_UINT16(200u, g_tmr.lock_ack_dwell);

    g_tmr.lock_ack_dwell = 0u;
    g_stat.curr_limit = 4000;
    g_evt.mon[FM_MON_PARK].code = 1u;
    control_speed_lock_fsm();
    TEST_ASSERT_EQUAL_HEX16(0x80u, g_mode.mode_flags & 0x80u);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_auto_off_counts_down);
    RUN_TEST(test_auto_off_motion_rearms);
    RUN_TEST(test_auto_off_long_press_and_tilt);
    RUN_TEST(test_auto_off_beeps_near_expiry);
    RUN_TEST(test_countdown_beep);
    RUN_TEST(test_shutdown_countdown);
    RUN_TEST(test_chirp_pulse_ten_ticks);
    RUN_TEST(test_alarm_teardown);
    RUN_TEST(test_alarm_arm_ramp_advances);
    RUN_TEST(test_alarm_arm_gives_up);
    RUN_TEST(test_alarm_siren_window);
    RUN_TEST(test_alarm_blink_phase);
    RUN_TEST(test_alarm_lockout_siren);
    RUN_TEST(test_subsys_enable_active_low);
    RUN_TEST(test_indicator_outputs);
    RUN_TEST(test_mode_sync_releases_subsystems);
    RUN_TEST(test_staging_defaults);
    RUN_TEST(test_status_bit_mirror);
    RUN_TEST(test_charge_sense_detects);
    RUN_TEST(test_charge_sense_high_current_window_closes);
    RUN_TEST(test_charge_removed_clears);
    RUN_TEST(test_flash_sequence);
    RUN_TEST(test_locate_flash);
    RUN_TEST(test_beep_short_and_long);
    RUN_TEST(test_beep_double_and_stop);
    RUN_TEST(test_beep_held_while_charging);
    RUN_TEST(test_mode_report_levels);
    RUN_TEST(test_mode_report_flags);
    RUN_TEST(test_brake_hold_fault_cycle);
    RUN_TEST(test_brake_hold_low_soc_and_lever_fault);
    RUN_TEST(test_brake_hold_fault_vbus_state);
    RUN_TEST(test_lock_timer_requests_lock);
    RUN_TEST(test_lock_profile_reg_signed);
    RUN_TEST(test_lock_riding_drops_request);
    RUN_TEST(test_lock_request_cancel_and_throttle);
    RUN_TEST(test_lock_out_gate);
    RUN_TEST(test_lock_coast_preset_bit);
    RUN_TEST(test_power_monitor_power_and_soc);
    RUN_TEST(test_power_monitor_runs_battery2_tick);
    RUN_TEST(test_power_monitor_comm_timeout);
    RUN_TEST(test_power_monitor_low_soc_warning);
    RUN_TEST(test_drive_param_init);
    RUN_TEST(test_charge_complete_beeps);
    RUN_TEST(test_charge_complete_waits_for_charger);
    RUN_TEST(test_charger_report_packs_schedule);
    RUN_TEST(test_charger_absent_clears_schedule_stop);
    RUN_TEST(test_charger_cancel_notice_length);
    RUN_TEST(test_charge_schedule_window);
    RUN_TEST(test_charge_schedule_single_shot);
    RUN_TEST(test_charge_state_reset);
    RUN_TEST(test_diag_reset);
    RUN_TEST(test_speed_lock_arm_counter);
    RUN_TEST(test_speed_lock_park_ack);
    RUN_TEST(test_speed_lock_mode3_wraps_to_row0);
    tst_exit(UNITY_END());
    return 0;
}
