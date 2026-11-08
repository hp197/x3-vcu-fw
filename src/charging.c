/**
 * \file            charging.c
 * \brief           Charge timer / fault-code annunciation tick (x3 VCU)
 * \note            Driven by the vehicle FSM in vehicle.c as `vcu_charge_timer_tick`. Shared
 *                      state comes from the vcu_state.h blackboards; this module's own working
 *                      state is the vcu_charge_timer_t overlay (charging.h).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "charging.h"
#include "control.h"  /* control_mode_report_fsm */
#include "faultmon.h" /* vcu_faultmon_sensor_debounce_a/b */
#include "vcu_state.h"

/* Shared blackboard blocks are plain globals in vcu_state.h: a write here is seen by every
   module that uses them. */

/* The two brake-lever held windows (g_evt.mon[FM_MON_BRAKE_FULL] PA4 / [FM_MON_BRAKE_HALF] PA1)
   are the last two fm_mon_t records of the event block (bases 0x78/0x84); the sensor
   debouncers latch them to code 4 (FM_CODE_LATCH), which this timer clears. */

/* ------------------------------------------------------------------------- */
/* Cadence / threshold constants.                                              */
/* ------------------------------------------------------------------------- */
#define CHG_MINUTE_FULL     100 /* parked minutes that mark a full charge        */
#define CHG_MINUTE_CLAMP    100 /* charge-minute counter clamp                    */
#define CHG_CYCLE_LIMIT     3   /* annunciation repeat cycles before it stops    */
#define CHG_FAULT_CONFIRM   3   /* sensor-fault latches needed to dispatch        */

#define CHG_SENS_FAULT_CODE 4 /* window `code` value == brake lever held (latched) */

#define CHG_CHIRP_SLOW      1000 /* charge-chirp reload while charging (event 9)   */
#define CHG_CHIRP_FAST      500  /* charge-chirp reload otherwise                  */

/* Fault-code annunciation (second body): tick cadence encoding a two-digit code. */
#define CHG_ANNUN_LEADIN    100 /* silent lead-in ticks before the tens digit    */
#define CHG_SEG_TENS        60  /* ticks per tens unit (one long pulse period)   */
#define CHG_SEG_UNITS       30  /* ticks per units unit (one short pulse period) */
#define CHG_TENS_GAP        130 /* lead-in + gap before the units digit (100+30) */
#define CHG_TENS_ON_MOD     60  /* tens pulse fires every 60 ticks                */
#define CHG_TENS_STEP_MOD   30  /* tens cadence step (off at the 30 mark)         */
#define CHG_UNITS_ON_MOD    30  /* units pulse fires every 30 ticks               */
#define CHG_UNITS_STEP_MOD  15  /* units cadence step (off at the 15 mark)        */

/* The error-code blink drives the rear-light mode (MODE->enable_flags bits 9..11,
   vcu_state.h mode_light_set): MODE_LIGHT_BRAKE for an on-pulse, MODE_LIGHT_OFF between. */

/* Beep / dispatch codes passed to the shared seams. */
#define CHG_MODE_LIGHT      4 /* control_mode_report_fsm(4): mode/lighting dispatch */

/* Cross-module callees: the sensor-fault debouncers vcu_faultmon_sensor_debounce_a/b
   (faultmon.h), buzzer_beep (buzzer.h), and control_mode_report_fsm (control.h), whose
   uint32_t result the charge timer passes on as its own. */
#include "buzzer.h"

/* ------------------------------------------------------------------------- */
/* Charge timer tick.                                                          */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Blink out the latched error code as a two-digit pulse pattern
 * \note            Drives the MODE->enable_flags LED pulses and the beeper on a tens-then-units
 *                      cadence derived from the error code's decimal digits, for three cycles.
 * \return          Non-zero (1) while still annunciating (cycle < 3), 0 once done.
 */
static uint32_t charge_annunciate_code(void) {
    uint32_t tens = (uint32_t)g_lowglob.active_error_code / 10u;
    uint32_t units = (uint32_t)g_lowglob.active_error_code % 10u;
    uint32_t tick;

    tick = (uint16_t)(g_charge_timer.seg_tick + 1u);
    g_charge_timer.seg_tick = (uint16_t)tick;

    if (tick < CHG_ANNUN_LEADIN) {
        /* Silent lead-in: hold the rear light off. */
        mode_light_set(MODE_LIGHT_OFF);
    } else if (tick < tens * CHG_SEG_TENS + CHG_ANNUN_LEADIN) {
        int32_t t = (int32_t)tick - CHG_ANNUN_LEADIN;
        if (t % CHG_TENS_STEP_MOD == 0) {
            if (t % CHG_TENS_ON_MOD == 0) {
                mode_light_set(MODE_LIGHT_BRAKE);
                buzzer_beep(BEEP_LONG);
            } else {
                mode_light_set(MODE_LIGHT_OFF);
            }
        }
    } else if (tick < tens * CHG_SEG_TENS + CHG_TENS_GAP) {
        /* Gap between the tens and units digits. */
        mode_light_set(MODE_LIGHT_OFF);
    } else if (tick < tens * CHG_SEG_TENS + units * CHG_SEG_UNITS + CHG_TENS_GAP) {
        int32_t u = (int32_t)tick - (int32_t)(tens * CHG_SEG_TENS) - CHG_TENS_GAP;
        if (u % CHG_UNITS_STEP_MOD == 0) {
            if (u % CHG_UNITS_ON_MOD == 0) {
                buzzer_beep(BEEP_SHORT);
                mode_light_set(MODE_LIGHT_BRAKE);
            } else {
                mode_light_set(MODE_LIGHT_OFF);
            }
        }
    } else {
        /* One full tens+units sweep done: restart the segment, bump the cycle. */
        g_charge_timer.seg_tick = 0;
        g_charge_timer.cycle = (uint8_t)(g_charge_timer.cycle + 1u);
    }

    return (uint32_t)(g_charge_timer.cycle < CHG_CYCLE_LIMIT);
}

/**
 * \brief           Charge timer / fault-code annunciation tick
 */
uint32_t vcu_charge_timer_tick(void) {
    uint8_t abort_code = g_in1.drive_active;
    uint32_t chirp;

    /* Abort: an upstream fault/abort code forces the timers back to zero. */
    if (abort_code != 0) {
        g_charge_timer.seg_tick = 0;
        g_charge_timer.cycle = 0;
        return abort_code;
    }

    /* A change in the latched error code re-zeros the segment + cycle state. */
    if (g_charge_timer.code_snapshot != g_lowglob.active_error_code) {
        g_charge_timer.seg_tick = 0;
        g_charge_timer.cycle = 0;
    }
    g_charge_timer.code_snapshot = g_lowglob.active_error_code;

    /* With an error code latched, blink it out; otherwise service the charge timer. */
    if (g_lowglob.active_error_code != 0) {
        return charge_annunciate_code();
    }

    chirp = g_charge_timer.charge_min;

    if (g_lowglob.active_event_id == VCU_CHG_EVT_NONE) {
        /* No event active: hold the chirp timer and roll the charge-minute counter. */
        g_charge_timer.chirp_tick = 0;
        if (chirp >= CHG_MINUTE_FULL) {
            g_charge_timer.charge_min = 0;
            chirp = control_mode_report_fsm(CHG_MODE_LIGHT);
            return chirp;
        }
        return chirp;
    }

    /* An event (e.g. charging) is active. */
    if (g_vs.speed_kmh == 0) {
        /* Parked / idle and no throttle -> count a charge minute (clamped). */
        uint32_t prev = g_charge_timer.charge_min;
        if (g_in1.brake_any == 0) {
            g_charge_timer.charge_min = (uint16_t)(prev + 1u);
            if (prev > CHG_MINUTE_CLAMP) {
                g_charge_timer.charge_min = CHG_MINUTE_CLAMP;
            }
        }
    } else {
        g_charge_timer.charge_min = 0;
    }

    /* At the full mark, run the two brake-lever held debouncers and confirm the latch. */
    if (g_charge_timer.charge_min >= CHG_MINUTE_FULL) {
        vcu_faultmon_sensor_debounce_a();
        vcu_faultmon_sensor_debounce_b();
        if ((g_evt.mon[FM_MON_BRAKE_FULL].code == CHG_SENS_FAULT_CODE)
            || (g_evt.mon[FM_MON_BRAKE_HALF].code == CHG_SENS_FAULT_CODE)) {
            g_evt.mon[FM_MON_BRAKE_FULL].code = 0;
            g_evt.mon[FM_MON_BRAKE_HALF].code = 0;
            g_charge_timer.fault_debounce = (uint8_t)(g_charge_timer.fault_debounce + 1u);
            if (g_charge_timer.fault_debounce >= CHG_FAULT_CONFIRM) {
                g_charge_timer.fault_debounce = 0;
                (void)control_mode_report_fsm(CHG_MODE_LIGHT);
            }
        }
    }

    /* Charge chirp: fire on reload, else count down. Event 9 uses the slow cadence. */
    if (g_charge_timer.chirp_tick == 0) {
        buzzer_beep(BEEP_DOUBLE);
        if (g_lowglob.active_event_id == VCU_CHG_EVT_CHARGING) {
            chirp = CHG_CHIRP_SLOW;
        } else {
            chirp = CHG_CHIRP_FAST;
        }
        g_charge_timer.chirp_tick = (uint16_t)chirp;
    } else {
        chirp = (uint32_t)(g_charge_timer.chirp_tick - 1u);
        g_charge_timer.chirp_tick = (uint16_t)chirp;
    }

    return chirp;
}
