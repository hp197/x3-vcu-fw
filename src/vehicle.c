/**
 * \file            vehicle.c
 * \brief           Vehicle finite-state machine (scheduler slot 0) and its latch/event helpers
 * \note            Slot-0 FSM task: nine per-state run handlers, the state-change latch, the
 *                      shared pre-pass and the input-test event-report path, plus the
 *                      FSM-coupled GPIO latches and event-prep leaves. Plain C over the
 *                      vcu_state.h overlays, dispatched on the vcu_fsm_state_t enum.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "vehicle.h"
#include "model.h" /* VCU_HAS_FEATURE */

#include <string.h>

#include "buzzer.h"
#include "can.h"
#include "charging.h"
#include "clock.h"
#include "config.h" /* vcu_standby_cycle, vcu_config_init, vcu_startup_poll */
#include "control.h"
#include "faultmon.h"
#include "gpio.h" /* vcu_gpio_fault_latch_init, VCU_GPIO_EVENT_* (g_vs.wake_src codes) */
#include "ninebot.h"
#include "odometer.h"
#include "passcode.h" /* passcode_set_confirm_tick, passcode_key_unlock_tick, passcode_digit_unlock_tick */
#include "report.h"
#include "ride_assist.h" /* ride_assist_tick */
#include "ridemode.h"
#include "sensor.h"
#include "usart.h"
#include "vcu_state.h"

/* Top-level FSM state byte in g_state_block (values 0..8 = vcu_fsm_state_t). */
#define FSM                      (g_state_block.fsm_state)

/* ------------------------------------------------------------------------- */
/* Named bit masks for the packed mode/BMS halfwords. The IN2 register bits  */
/* (SF_*, CFG_*, CFG2_*, CHG_*) live in vcu_state.h, MF_ACTIVE in vehicle.h. */
/* ------------------------------------------------------------------------- */
/* MODE->mode_flags (+0x0A) */
#define MF_ARMED                 0x0008u /*!< bit3                               */
#define MF_REGEN                 0x0010u /*!< bit4 regen/brake output           */
/* STAT->status_word_b (+0x126) */
#define BMS_ST_CHARGER           0x0008u /*!< bit3 charger present (BMS 0x422 d[4..7]) */

/* ------------------------------------------------------------------------- */
/* Loop periods / thresholds. STANDBY_OFF_TICKS is shared                    */
/* with the mileage pre-pass and lives in vehicle.h.                            */
/* ------------------------------------------------------------------------- */
#define LOCK_TIMER_INIT          600  /* VS->lock_timer reload on state change  */
#define POWER_ON_TICKS           11   /* state-1 power-on step count (0..10)    */
#define SESSION_SCREEN_TICKS     200  /* state-3 session-screen (0x14) dwell     */
#define POWER_DOWN_TICKS         300  /* state-4 power-down dwell              */
#define POWER_DOWN_COMMIT_PERIOD 20   /* state-4 screen-0 re-commit period     */
#define POWER_DOWN_COMMIT_LAST   100  /* state-4 last re-commit run            */
#define ALARM_GUARD_TICKS        500  /* state-6 motion-guard window            */
#define CHG_SCREEN_HOLD_TICKS    1000 /* state-8 charge-screen hold             */
#define CHG_KEYHOLD_TICKS        300  /* state-8 key hold on the charge screen */
#define FAULT_TIMEOUT_SHORT      3000 /* fault-report timeout reload (short)    */
#define FAULT_TIMEOUT_LONG       6000 /* fault-report timeout reload (long)     */
#define OFF_DELAY_TICKS          100  /* state-0 -> state-8 settle preset       */
#define TRAVEL_IN_LO             0x33u /* input test: lever travel (ADC >> 4) from here ... */
#define TRAVEL_FULL              0x9Bu /* ... to below here is in travel, from here full     */

/* IMU tilt thresholds, degrees. */
#define TILT_FALLEN_DEG          50.0f /* idle long press: tilt > 50 deg -> off            */
#define TILT_ALARM_DEG           50.0f /* alarm timeout: tilt > 50 deg ends the siren      */
#define IDLE_SCREEN              16    /* idle screen: a lapsed wake window stays on it    */
#define DRIVE_FAULT_ARMED        2u    /* cfg_17 drive-fault state set at a standstill */
#define DRIVE_FAULT_VBUS_LIMIT   150u  /* A3C vbus code hi byte above this escalates it to 3   */

/* GPIO ports the FSM latches drive (GPIOB 0x40010C00, GPIOC 0x40011000), driven     */
/* straight through the Artery gpio_bits_set / gpio_bits_reset HAL (one 32-bit store   */
/* to scr @+0x10 / clr @+0x14). The brake / indicator / anti-theft leaves (PB13, PA6,  */
/* PB14) go through control.c's control_out_* primitives.                             */
#define VCU_GPIO_LATCH           GPIOB /* drive kill latch           */
#define VCU_GPIO_PWR             GPIOC /* power/fault latch          */

/* GPIO pin masks the FSM latches drive. */
#define GPIO_LATCH_KILL          0x8000 /* GPIOB bit15 drive kill latch  */
#define GPIO_PWR_LATCH           0x4000 /* GPIOC bit14 power/fault latch  */

/* FSM pre-pass (skipped in state 5 = sleep). It is the IMU attitude update in sensor.c:  */
/* sample read, calibration step, quaternion attitude, then control_sensor_delta and the  */
/* tilt latch. The result is discarded.                                                   */
static void vehicle_fsm_prepass(void) {
    if (VCU_HAS_FEATURE(IMU)) {
        (void)vcu_sensor_attitude_update();
    }
}

/* Fault-detection aggregate: the ten input monitors in a fixed order (faultmon.c  */
/* vcu_faultmon_tick). Reached from the state-8 handler, the FSM input-test path and */
/* ridemode.                                                                         */
void vehicle_fault_aggregate(void) { vcu_faultmon_tick(); }

/**
 * \brief           Drive the brake-latch (PB13) and indicator (PA6) leaves, then the anti-theft leaf
 * \param[in]       brake: PB13 level
 * \param[in]       ind: PA6 level
 * \note            The anti-theft leaf (PB14) is the inverse of the vehicle lock flag (g_vs[0]):
 *                      locked (!=0) -> release (0), unlocked -> assert (1). The three primitives
 *                      are control.c's public output leaves.
 */
static void prv_event_outputs(int32_t brake, int32_t ind) {
    control_out_pb13(brake);
    control_out_pa6(ind);
    control_out_pb14((g_vs.region != 0) ? 0 : 1);
}

/* Event-1 output prep: brake-latch and indicator leaves low, anti-theft from the lock flag. */
void vehicle_event1_prep(void) { prv_event_outputs(0, 0); }

/* Event-8 output prep: twin of event-1 with the brake-latch and indicator leaves HIGH. */
void vehicle_event8_prep(void) { prv_event_outputs(1, 1); }

/* Event-9 subsystem-A prep: enable subsystem A only, then publish mask A           */
/* (report_subsys_mask_set). */
void vehicle_event9_prep(void) {
    control_subsys_a_enable(1);
    report_subsys_mask_set(VCU_SUBSYS_A);
}

/* Event-10 subsystem-B prep: twin of event-9 with subsystem B and mask B. */
void vehicle_event10_prep(void) {
    control_subsys_b_enable(1);
    report_subsys_mask_set(VCU_SUBSYS_B);
}

/* ------------------------------------------------------------------------- */
/* FSM-coupled latch / mode helpers.                                          */
/*                                                                            */
/* These are the short helpers the vehicle FSM calls directly for the brake /  */
/* lock / anti-theft / fault latches, the per-state event-block reset and the  */
/* drive-mode commit.                                                          */
/* ------------------------------------------------------------------------- */

/* On every state change, reset helper B wipes the whole 0x90-byte event-flag    */
/* block (g_evt). */
void vehicle_event_block_clear(void) { /* clear event-flag block */
    memset((void*)&g_evt, 0, sizeof(g_evt));
}

/* State-4 power/fault latch: disable drive, then assert (assert == 1) or release */
/* the GPIOC power-latch pin (bit 14). */
static void vehicle_power_fault_latch(int32_t assert_latch) { /* state-4 fault latch */
    vcu_gpio_fault_latch_init(0);
    if (assert_latch == 1) {
        gpio_bits_set(VCU_GPIO_PWR, GPIO_PWR_LATCH);
    } else {
        gpio_bits_reset(VCU_GPIO_PWR, GPIO_PWR_LATCH);
    }
}

/* Event-7 lock / anti-theft latch: brake latch (PB13) low, indicator (PA6) high, anti-theft */
/* (PB14) from the vehicle-state lock flag (g_vs[0]).                                       */
void vehicle_lock_antitheft_latch(void) { prv_event_outputs(0, 1); }

/* Commit drive/indicator mode is owned by ninebot.c as the public nb_commit_mode(); */
/* the FSM calls it through that prototype (see ninebot.h). */

/* Mode/enable flag reset: clears the drive/enable flag bits of MODE->mode_flags     */
/* (+0x0A: bits 0-9 and 11 -- bit10 is deliberately preserved) and                   */
/* MODE->enable_flags (+0x10: bits 0-11). */
static void vehicle_mode_flags_clear(void) {
    g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~0x0BFFu);     /* bits 0-9, 11 */
    g_mode.enable_flags = (uint16_t)(g_mode.enable_flags & ~0x0FFFu); /* bits 0-11    */
}

/* The counter-service step is the startup/power-on timing FSM step:                      */
/* config.c's vcu_startup_poll(uint16_t*), called from the state-4 site with the 16-bit  */
/* counter &PREV->ctr_d2. See config.h. */

/* ========================================================================= */
/* slot 0 per-state "run" handlers.                                          */
/*                                                                            */
/* The FSM dispatch below calls one of these every tick for the current state.*/
/* Each drives the control loop for that state via the cross-module calls      */
/* above and commits mode/report changes through the mapped report helpers      */
/* (nb_commit_mode, report_status, report_u16).                              */
/* ========================================================================= */

/**
 * \brief           Take a pending power-button press (code 1) and send the lock-event report
 * \note            report_u16(0x61, 0x20).
 */
static void prv_power_press_report(void) {
    if (g_evt.mon[FM_MON_POWER_BTN].code == 1) {
        g_evt.mon[FM_MON_POWER_BTN].code = 0;
        report_u16(VCU_RPT_U16_LOCK, 0x20);
    }
}

/**
 * \brief           Clear the ride-active bit (MODE mode_flags bit2)
 */
static void prv_ride_inactive(void) { g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~MF_ACTIVE); }

/**
 * \brief           Clear the run and idle bits of the status register (reg 0x1C b0 / b5)
 */
static void prv_status_run_idle_clear(void) {
    g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & ~(SF_RUN | SF_IDLE));
}

/**
 * \brief           Arm at a standstill: ride active + armed, drive-fault state (MODE cfg_17 bits 2..3) 2
 * \note            mode_flags |= MF_ACTIVE | MF_ARMED, then `(x & 0xF3) + 8` on cfg_17, in the locked
 *                      and the alarm state. control_fault_drive_fsm (control.c) consumes it.
 */
static void vehicle_drive_fault_arm(void) {
    g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | MF_ACTIVE | MF_ARMED);
    g_mode.cfg_17 = (uint8_t)((g_mode.cfg_17 & ~CFG17_DRIVE_FAULT) | (DRIVE_FAULT_ARMED << CFG17_DRIVE_FAULT_SHIFT));
}

/**
 * \brief           Raise the armed drive-fault state to 3 while the bus code is above 150
 * \param[in]       latch: the state's standstill latch (only 1 escalates)
 * \note            latch == 1 and A3C+2 hi byte `> 0x96` -> cfg_17 |= 0x0C.
 */
static void vehicle_drive_fault_escalate(uint8_t latch) {
    if ((latch == 1u) && ((g_fault_codes.vbus_code >> 8) > DRIVE_FAULT_VBUS_LIMIT)) {
        g_mode.cfg_17 = (uint8_t)(g_mode.cfg_17 | CFG17_DRIVE_FAULT);
    }
}

/**
 * \brief           Power the outputs back up: drive-kill latch off, CAN mode reset, power latch on
 */
static void prv_power_restore(void) {
    gpio_bits_reset(VCU_GPIO_LATCH, GPIO_LATCH_KILL);
    vcu_can_mode_reset();
    vehicle_power_fault_latch(1);
}

/**
 * \brief           Show the ride screen: screen 1, the status report and drive-mode report 1
 */
static void prv_ride_screen(void) {
    nb_commit_mode(1);
    report_status(&g_stat.status_packed);
    report_u16(VCU_RPT_U16_MODE, 1);
}

/**
 * \brief           Show the scheduled-charge screen (0x12) with the start time in minutes
 * \note            ridetime_disp = start hour (high byte) x 60 + start minute (low byte).
 */
static void prv_sched_screen(void) {
    uint16_t start;

    nb_commit_mode(0x12);
    start = g_in2.f.chg_sched_start;
    g_scr.ridetime_disp = (int16_t)((start & 0xffu) + ((uint32_t)start >> 8) * 0x3cu);
}

/**
 * \brief           Show the passcode-entry screen of the configured passcode kind
 * \note            Numeric passcode: screen 4; key sequence: screen 0x11 with the code length
 *                      in the high byte.
 */
static void prv_passcode_screen(void) {
    if ((g_in2.f.cfg_flags & CFG_PASSCODE_NUM) != 0) {
        nb_commit_mode(4);
    } else {
        nb_commit_mode((int16_t)(((uint32_t)g_vs.code_len << 8) | 0x11));
    }
}

/* state 0 = idle: powered, not started. The charger bit STAT+0x126 bit3 is left as it is */
/* for the plug-in edge.                                                                 */
static void vehicle_state_idle(void) {
    uint16_t cnt;

    g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_IDLE);
    vcu_faultmon_can_fault_escalate();
    control_deferred_action();
    charger_detect_tick();
    control_charge_fsm();
    if (g_scr.ui_screen == 0x10) {
        control_shutdown_countdown(3000);
    }
    if (g_scr.light_state != 2) {
        report_u16(VCU_RPT_U16_MODE, 2);
    }
    if (g_vs.app_reg_pending != 0) {
        cnt = g_prev.ctr_08;
        g_prev.ctr_08 = (uint16_t)(cnt + 1);
        if (cnt > STANDBY_OFF_TICKS) {
            g_prev.ctr_08 = 0;
            FSM = VCU_FSM_POWER_DOWN;
            g_vs.poweroff_beep = 1;
        }
    }
}

/* state 1 = power-on: config init, beep, first screen. Clears only the idle bit on entry and */
/* sets the run bit while it waits for the arm gate.                                         */
static void vehicle_state_power_on(void) {
    uint16_t cnt;
    uint8_t start;

    g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & ~SF_IDLE);
    g_vs.power_ready = 1;
    prv_ride_inactive();
    control_throttle_brake_scan(VCU_FSM_POWER_ON);
    report_fault_status(1);
    vcu_faultmon_fault_latch(1);
    vcu_faultmon_can_fault_escalate();
    control_auto_off_tick(FAULT_TIMEOUT_LONG);
    control_gesture_fsm();
    vcu_faultmon_retry_trigger();
    control_deferred_action();
    prv_power_press_report();
    cnt = g_prev.ctr_0a;
    if (cnt < POWER_ON_TICKS) {
        g_vs.power_ready = 0;
        if (cnt == 0) {
            vcu_config_init();
            buzzer_beep(BEEP_SHORT);
            if ((g_in2.f.status_flags & SF_SESSION) != 0) {
                start = g_vs.start_req;
                if (start == VS_START_NO_LOCK || start == VS_START_UNLOCKED) {
                    nb_commit_mode(1);
                } else if (start == VS_START_LOCKED) {
                    prv_passcode_screen();
                }
            }
        }
        g_prev.ctr_0a = (uint16_t)(g_prev.ctr_0a + 1);
    } else {
        g_prev.ctr_0a = 0;
        if (g_vs.shutdown_req == 0) {
            if (g_in1.arm_gate == 0) {
                g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_RUN);
                g_vs.power_ready = 0;
                g_prev.ctr_0a = 0x14;
            }
        } else {
            g_vs.power_ready = 1;
        }
    }
}

/* state 2 = locked: passcode screen + passcode entry. */
static void vehicle_state_locked(void) {
    prv_status_run_idle_clear();
    if (g_prev.commfault_a_arm == 0) {
        g_prev.commfault_a_arm = 1;
        passcode_entry_reset();
        prv_passcode_screen();
        report_status(&g_stat.status_packed);
    }
    if ((g_fault_codes.comm_fault & 1) == 0 && g_vs.speed_kmh < VCU_STANDSTILL_KMH && g_prev.commfault_a_latch == 0) {
        g_prev.commfault_a_latch = 1;
        vehicle_drive_fault_arm();
    } else if (g_prev.commfault_a_latch == 0) {
        prv_ride_inactive();
    } else {
        vehicle_drive_fault_escalate(g_prev.commfault_a_latch);
    }
    vcu_faultmon_can_fault_escalate();
    control_throttle_brake_scan(VCU_FSM_LOCKED);
    vcu_faultmon_fault_latch(2);
    report_fault_status(2);
    control_auto_off_tick(FAULT_TIMEOUT_SHORT);
    charger_detect_tick();
    vcu_charge_timer_tick();
    control_gesture_fsm();
    control_deferred_action();
    if (g_vs.remote_unlocked == 0 && g_vs.app_unlock_req == 0) {
        if ((g_in2.f.cfg_flags & CFG_PASSCODE_NUM) != 0) {
            g_vs.unlock_ok = passcode_digit_unlock_tick();
        } else if ((g_in2.f.cfg_flags & CFG_PASSCODE_KEY) != 0) {
            g_vs.unlock_ok = passcode_key_unlock_tick();
        }
    } else {
        g_vs.app_unlock_req = 0;
        g_vs.unlock_ok = 1;
    }
    if (g_vs.shutdown_req != 0 || g_vs.unlock_ok != 0 || g_vs.lockout_phase == LOCKOUT_ALARM) {
        g_prev.commfault_a_latch = 0;
        g_prev.commfault_a_arm = 0;
    }
}

/* state 3 = ride: the full drive pipeline, including the ride assists. */
static void vehicle_state_ride(void) {
    g_vs.lockout_phase = LOCKOUT_NONE;
    g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_RUN);
    if ((g_in2.f.status_flags & SF_SESSION) != 0) {
        if (g_prev.chk_c_latch == 0) {
            if (g_scr.ui_screen == 0x14) {
                g_prev.ctr_c = (uint16_t)(g_prev.ctr_c + 1);
            } else {
                g_prev.ctr_c = SESSION_SCREEN_TICKS;
            }
            if (g_prev.ctr_c > SESSION_SCREEN_TICKS - 1) {
                g_prev.chk_c_latch = 1;
                g_prev.ctr_c = 0;
                prv_ride_screen();
            }
        }
    } else {
        g_prev.chk_c_latch = 0;
    }
    vcu_faultmon_gesture_detect();
    ridemode_gear_event_tick();
    control_throttle_brake_scan(VCU_FSM_RIDE);
    vcu_faultmon_fault_latch(3);
    report_fault_status(3);
    /* The ride assists run on the staged throttle / brake (MODE+0x08 low / high byte) and
       the IMU's raw -GY / -GZ rates. */
    ride_assist_tick(g_mode.drive_output_word & 0xFFu, (uint32_t)g_mode.drive_output_word >> 8, vcu_sensor_gyro_raw(1u),
                     vcu_sensor_gyro_raw(2u));
    control_build_drive_cmd();
    vcu_faultmon_interlock_tick();
    control_lock_fsm();
    control_fault_drive_fsm();
    control_speed_lock_fsm();
    control_status_bit_mirror();
    control_charge_fsm();
    ridemode_submode_dispatch();
    charger_detect_tick();
    ridemode_status_word_update();
    control_auto_off_tick((int32_t)g_in2.f.auto_off_min_s * FAULT_TIMEOUT_LONG);
    report_countdown();
    vcu_faultmon_retry_trigger();
    report_datetime();
    vcu_charge_timer_tick();
    control_gesture_fsm();
    odometer_power_tick();
    control_deferred_action();
    if (g_vs.shutdown_req != 0 || g_in1.charger_connected != 0) {
        g_prev.chk_c_latch = 0;
    }
}

/**
 * \brief           State 4 = power-down: outputs off, screen 0, power-latch release
 * \note            While PA10 is released the dwell counts to 300 runs (re-committing screen 0
 *                      every 20th run up to run 100), then the power latch releases. PA10 held
 *                      pauses the dwell. A press that arrives after a release (`pd_button_held`
 *                      0) cancels the power-down: the latches re-assert and the FSM goes back to
 *                      idle with an app start pending.
 */
static void vehicle_state_power_down(void) {
    uint16_t cnt;

    prv_ride_inactive();
    g_vs.app_reg_pending = 0;
    if (g_prev.chk_d_latch == 0) {
        g_prev.chk_d_latch = 1;
        g_prev.pd_button_held = 1;
        control_diag_reset();
        vehicle_mode_flags_clear();
        control_dtc_mask_reset();
        control_event_mask_reset();
        ridemode_led_indicator_clear();
        control_charge_state_reset();
        control_speed_lock_fsm();
        nb_commit_mode(0);
        report_u16(VCU_RPT_U16_MODE, 0);
        if (g_vs.poweroff_beep != 0) {
            buzzer_beep(BEEP_NONE);
            g_vs.poweroff_beep = 0;
        } else {
            buzzer_beep(BEEP_LONG);
        }
        g_in1.recompute_req = 1;
    }
    vcu_faultmon_can_fault_escalate();
    control_throttle_brake_scan(VCU_FSM_POWER_DOWN);
    vcu_faultmon_fault_latch(4);
    charger_detect_tick();
    control_gesture_fsm();
    control_deferred_action();
    vcu_startup_poll((uint16_t*)&g_prev.ctr_d2);
    if (vcu_gpio_pa10_active() == 0) {
        g_prev.pd_button_held = 0;
        g_evt.mon[FM_MON_POWER_BTN].code = 0;
        cnt = (uint16_t)(g_prev.ctr_d + 1);
        g_prev.ctr_d = cnt;
        if (cnt <= POWER_DOWN_TICKS) {
            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | MF_REGEN);
            if ((cnt <= POWER_DOWN_COMMIT_LAST) && ((cnt % POWER_DOWN_COMMIT_PERIOD) == 0u)) {
                nb_commit_mode(0);
            }
        } else {
            g_prev.chk_d_latch = 0;
            g_prev.ctr_d2 = 0;
            g_prev.ctr_d = 0;
            buzzer_off();
            buzzer_beep(BEEP_NONE);
            gpio_bits_set(VCU_GPIO_LATCH, GPIO_LATCH_KILL);
            vehicle_power_fault_latch(0);
            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~MF_REGEN);
            g_vs.power_released = 1;
            prv_status_run_idle_clear();
            g_vs.wake_src = VCU_GPIO_EVENT_NONE;
        }
    } else {
        g_prev.ctr_d = 0;
        if ((g_evt.mon[FM_MON_POWER_BTN].code != 0) && (g_prev.pd_button_held == 0)) {
            g_prev.chk_d_latch = 0;
            g_prev.ctr_d2 = 0;
            g_prev.pd_button_held = 1;
            g_evt.mon[FM_MON_POWER_BTN].code = 0;
            g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~MF_REGEN);
            g_vs.chirp_req = 1;
            g_vs.app_start_req = 1;
            buzzer_off();
            buzzer_beep(BEEP_NONE);
            prv_power_restore();
            FSM = VCU_FSM_IDLE;
        }
    }
}

/* state 5 = sleep: low-power wait for a wake source. */
static void vehicle_state_sleep(void) {
    g_vs.report_enable = 0;
    g_vs.power_ready = 0;
    prv_status_run_idle_clear();
    if (g_vs.wake_src == VCU_GPIO_EVENT_NONE) {
        vcu_standby_cycle();
    } else if (g_vs.wake_src == VCU_GPIO_EVENT_PA10 && vcu_gpio_pa10_active() == 0) {
        g_vs.wake_src = VCU_GPIO_EVENT_NONE;
    } else {
        prv_power_restore();
        g_vs.power_ready = 1;
    }
    clock_restore(); /* CRM clock-register restore */
}

/* state 6 = alarm: anti-theft motion guard / siren. With the alarm timeout enabled a tilt */
/* above 50 degrees ends the siren.                                                         */
static void vehicle_state_alarm(void) {
    int32_t drive;

    prv_status_run_idle_clear();
    if ((g_fault_codes.comm_fault & 1) == 0 && g_vs.speed_kmh < VCU_STANDSTILL_KMH && g_prev.commfault_b_latch == 0) {
        g_prev.commfault_b_latch = 1;
        control_alarm_motion_tick(0);
        vehicle_drive_fault_arm();
        g_vs.chirp_req = 1;
        report_status(&g_stat.status_packed);
    } else if (g_prev.commfault_b_latch == 0) {
        prv_ride_inactive();
    } else {
        vehicle_drive_fault_escalate(g_prev.commfault_b_latch);
    }
    if (g_vs.lockout_phase == LOCKOUT_ALARM) {
        if (g_scr.ui_screen != 5) {
            nb_commit_mode(5);
        }
        control_alarm_motion_tick(2);
    } else if (g_prev.ctr_commfault_b < ALARM_GUARD_TICKS) {
        vcu_faultmon_can_fault_escalate();
        g_prev.ctr_commfault_b = (uint16_t)(g_prev.ctr_commfault_b + 1);
        drive = control_alarm_motion_tick(1);
        if (drive != 0) {
            g_prev.ctr_commfault_b = ALARM_GUARD_TICKS;
        }
    } else if ((g_in2.f.cfg_flags & CFG_ALARM_EN) != 0) {
        if (((g_in2.f.chg_cfg & ALARM_TIMEOUT_EN) != 0 && vcu_sensor_tilt_deg() > TILT_ALARM_DEG)
            || g_vs.remote_unlocked != 0) {
            g_vs.creep_enable = 0;
        }
        if (g_vs.creep_enable == 0) {
            g_vs.shutdown_req = 1;
            g_vs.poweroff_beep = 1;
        } else {
            vcu_faultmon_can_fault_escalate();
            if (g_scr.ui_screen != 5) {
                nb_commit_mode(5);
            }
            control_alarm_motion_tick(2);
        }
    } else {
        g_vs.shutdown_req = 1;
        g_vs.poweroff_beep = 1;
    }
    if (g_vs.shutdown_req == 0 && g_evt.mon[FM_MON_POWER_BTN].code != 1 && g_vs.tilt_latch != 1
        && g_vs.app_start_req == 0) {
        return;
    }
    g_prev.commfault_b_latch = 0;
    g_prev.ctr_commfault_b = 0;
    control_alarm_motion_tick(0);
}

/* state 7 = charging: charger on during a session, drive pipeline with the throttle */
/* blocked. Its entry one-shot is the private g_prev.charging_oneshot, not the        */
/* charger-detect flag SCR+0x10.                                                      */
static void vehicle_state_charging(void) {
    g_vs.lockout_phase = LOCKOUT_NONE;
    g_in2.f.status_flags = (uint16_t)((g_in2.f.status_flags & ~SF_IDLE) | SF_RUN);
    if (g_prev.charging_oneshot == 0) {
        g_prev.charging_oneshot = 1;
        prv_ride_screen();
    }
    ridemode_gear_event_tick();
    control_throttle_brake_scan(VCU_FSM_CHARGING);
    vcu_faultmon_fault_latch(7);
    report_fault_status(7);
    control_lock_fsm();
    control_fault_drive_fsm();
    control_speed_lock_fsm();
    control_build_drive_cmd();
    vcu_faultmon_interlock_tick();
    control_charge_fsm();
    ridemode_submode_dispatch();
    charger_detect_tick();
    charge_complete_tick();
    ridemode_status_word_update();
    control_auto_off_tick((int32_t)g_in2.f.auto_off_min_s * FAULT_TIMEOUT_LONG);
    vcu_faultmon_retry_trigger();
    control_gesture_fsm();
    odometer_power_tick();
    control_deferred_action();
    prv_ride_inactive();
    if (g_vs.shutdown_req != 0 || g_in1.charger_connected == 0) {
        g_prev.charging_oneshot = 0;
    }
}

/* state 8 = charge standby: awake for charging after power-down. */
static void vehicle_state_charge_standby(void) {
    uint16_t cnt;

    vehicle_fault_aggregate();
    charger_detect_tick();
    charge_complete_tick();
    control_gesture_fsm();
    control_charge_state_reset();
    ridemode_status_word_update();
    if (g_vs.postride_timer != 0) {
        g_vs.postride_timer = (int16_t)(g_vs.postride_timer - 1);
        return;
    }
    if (g_prev.chk_e_latch == 0) {
        g_prev.chk_e_latch = 1;
        g_prev.chk_e_phase = 1;
        report_status(&g_stat.status_packed);
        if ((g_in2.f.chg_cfg & CHG_SCHED_EN) != 0 && g_in1.charger_connected == 0) {
            prv_sched_screen();
        } else {
            nb_commit_mode(3);
        }
    }
    if (g_vs.chg_disp_refresh_req != 0) {
        g_prev.chk_e_phase = 0;
        g_vs.chg_disp_refresh = 1;
        g_vs.chg_disp_refresh_req = 0;
    }
    if (g_prev.chk_e_phase == 0) {
        if (g_vs.chg_disp_refresh != 0) {
            g_prev.ctr_e2 = 0;
            g_prev.ctr_e = 0;
            g_prev.chk_e_phase = 1;
            g_vs.chg_disp_refresh = 0;
            if (g_in1.charger_connected == 0 && g_vs.chg_full == 0) {
                if ((g_in2.f.chg_cfg & CHG_SCHED_EN) != 0) {
                    prv_sched_screen();
                } else {
                    g_prev.ctr_e = CHG_SCREEN_HOLD_TICKS;
                }
            } else {
                nb_commit_mode(3);
            }
            report_status(&g_stat.status_packed);
            vehicle_event_block_clear();
        }
    } else {
        cnt = g_prev.ctr_e;
        g_prev.ctr_e = (uint16_t)(cnt + 1);
        if (cnt > CHG_SCREEN_HOLD_TICKS - 1) {
            g_prev.ctr_e = 0;
            g_prev.chk_e_phase = 0;
            nb_commit_mode(0);
            g_vs.chg_disp_refresh = 0;
        }
    }
    if (g_scr.ui_screen == 0x12) {
        vcu_faultmon_sensor_debounce_a();
        vcu_faultmon_sensor_debounce_b();
        if (g_evt.mon[FM_MON_BRAKE_FULL].code == 4 || g_evt.mon[FM_MON_BRAKE_HALF].code == 4) {
            cnt = (uint16_t)(g_prev.ctr_e2 + 1);
            g_prev.ctr_e2 = cnt;
            if (cnt > CHG_KEYHOLD_TICKS) {
                g_prev.ctr_e2 = 0;
                if ((g_in2.f.chg_cfg & CHG_KEYHOLD) == 0) {
                    g_in2.f.chg_cfg = (uint16_t)(g_in2.f.chg_cfg | CHG_KEYHOLD);
                }
            }
        }
    }
    if (g_vs.app_start_req != 0 || g_evt.mon[FM_MON_POWER_BTN].code == 1
        || ((g_stat.status_word_b & BMS_ST_CHARGER) == 0 && g_vs.chg_sched_cancel_notice == 0)) {
        g_prev.chk_e_latch = 0;
        g_prev.ctr_e = 0;
        g_prev.chk_e_phase = 0;
    }
}

/**
 * \brief           Latch a charger plug-in edge
 * \note            Each normal run compares the BMS charger bit (STAT+0x126 bit3) with the
 *                      value it saw last time; a change is stored, and a change to "present"
 *                      sets the plug-in latch. The latch is cleared on every state change.
 */
static void vehicle_charger_edge_update(void) {
    uint16_t charger = (uint16_t)(g_stat.status_word_b & BMS_ST_CHARGER);

    if (charger != g_prev.chg_bit_prev) {
        g_prev.chg_bit_prev = charger;
        if (charger == BMS_ST_CHARGER) {
            g_prev.chg_plug_edge = 1;
        }
    }
}

/**
 * \brief           Lying on its side: the tilt the idle long press powers down at
 * \return          1 if the tilt is above 50 degrees
 */
static uint8_t vehicle_fallen(void) { return (vcu_sensor_tilt_deg() > TILT_FALLEN_DEG) ? 1u : 0u; }

/**
 * \brief           State-0 dispatch: wake window, start request or power-down
 * \note            A pending start (app start, power-button event or charger plug-in edge)
 *                      opens a 300-run wake window on screen 2; a long press lying on the side
 *                      powers down. After the window the start request is resolved. Without a
 *                      pending start a PA3 or PA10 wake runs the keep-alive down, and a window
 *                      that lapsed off the idle screen powers down.
 */
static void vehicle_idle_dispatch(void) {
    uint8_t pending = ((g_vs.app_start_req != 0) || (g_evt.mon[FM_MON_POWER_BTN].code != 0)
                       || (g_prev.chg_plug_edge != 0))
                          ? 1u
                          : 0u;
    uint16_t cnt;

    if ((pending != 0u) && (g_prev.mode_counter < STANDBY_OFF_TICKS)) {
        g_vs.app_reg_pending = 0;
        if (g_prev.mode_counter == 0) {
            nb_commit_mode(2);
            if (g_vs.app_start_req != 0) {
                g_vs.chirp_req = 1;
            }
        }
        cnt = (uint16_t)(g_prev.mode_counter + 1);
        g_prev.mode_counter = cnt;
        if ((cnt >= STANDBY_OFF_TICKS) && (vehicle_fallen() != 0u) && (g_evt.mon[FM_MON_POWER_BTN].code != 0)) {
            FSM = VCU_FSM_POWER_DOWN;
            g_vs.start_req = VS_START_NONE;
            g_vs.poweroff_beep = 1;
            g_evt.word[0] = 0; /* wipe the lock record (shift..code/phase) */
            g_evt.word[1] = 0;
            g_evt.word[2] = 0;
        }
        return;
    }

    if (pending != 0u) {
        if ((g_in2.f.cfg_flags & CFG_LOCK_EN) != 0) {
            if (g_vs.remote_lock_req == 0) {
                if (g_vs.remote_unlocked == 0 && g_vs.app_unlock_req == 0) {
                    g_vs.start_req = VS_START_LOCKED;
                } else {
                    g_vs.app_unlock_req = 0;
                    g_vs.start_req = VS_START_UNLOCKED;
                }
            } else {
                g_vs.remote_lock_req = 0;
                g_vs.start_req = VS_START_NONE;
                if ((g_stat.status_word_b & BMS_ST_CHARGER) == 0) {
                    nb_commit_mode(0x10);
                }
            }
        } else {
            g_vs.start_req = VS_START_NO_LOCK;
        }
        if ((g_prev.chg_plug_edge != 0) && (g_in2.f.status_flags & SF_SESSION) != 0) {
            FSM = VCU_FSM_CHARGE_STANDBY;
            g_vs.start_req = VS_START_NONE;
            g_vs.postride_timer = OFF_DELAY_TICKS;
        }
        g_evt.mon[FM_MON_POWER_BTN].code = 0;
        g_vs.app_start_req = 0;
        g_vs.wake_src = VCU_GPIO_EVENT_NONE;
    } else if ((g_vs.wake_src == VCU_GPIO_EVENT_PA3) || (g_vs.wake_src == VCU_GPIO_EVENT_PA10)) {
        if (g_vs.keepalive_ticks == 0) {
            FSM = VCU_FSM_POWER_DOWN;
            g_vs.poweroff_beep = 1;
        } else {
            g_vs.keepalive_ticks = (uint16_t)(g_vs.keepalive_ticks - 1);
        }
    } else if ((g_prev.mode_counter != 0) && (g_scr.ui_screen != IDLE_SCREEN)) {
        FSM = VCU_FSM_POWER_DOWN;
        g_vs.start_req = VS_START_NONE;
        g_vs.poweroff_beep = 1;
    }
    if (g_vs.start_req != VS_START_NONE) {
        FSM = VCU_FSM_POWER_ON;
    }
}

/** \brief Input monitors the input-test mode reports, in priority order: record i reports event i + 1. */
static const uint8_t prv_test_mon[VCU_EVT_10] = {FM_MON_POWER_BTN,     FM_MON_PARK,          FM_MON_GEAR_UP,
                                                 FM_MON_GEAR_DOWN,     FM_MON_LIGHT_EN,      FM_MON_INTERLOCK,
                                                 FM_MON_SUBSTATE_PREV, FM_MON_SUBSTATE_NEXT, FM_MON_LIGHT_LO,
                                                 FM_MON_LIGHT_HI};

/** \brief Travel latch of each lever, by ADC slot (throttle, half brake, full brake): slot i reports 0x0B + i. */
static volatile uint8_t* const prv_travel_latch[VCU_ADC_SLOT_BRAKE_FULL + 1u] = {
    &g_deb.thr_travel, &g_deb.brake_half_travel, &g_deb.brake_full_travel};

/** \brief Payload of an event report (two zero bytes). */
static const uint8_t prv_event_payload[VCU_RPT_EVT_LEN] = {0u, 0u};

/**
 * \brief           Input-test mode: report one pending input event or a lever end stop
 * \note            No state handler runs. The first pending input event (code 1, in the
 *                      \ref prv_test_mon order) is cleared, its output prep runs and one Ninebot
 *                      event-report frame goes out. Without one, the throttle / brake-lever
 *                      travel debounce runs: in travel [0x33, 0x9B) sets the lever's latch,
 *                      reaching >= 0x9B with the latch set clears it and reports the end stop
 *                      (0x0B PA0, 0x0C PA1, 0x0D PA4); the first lever that does either ends the run.
 */
static void prv_input_test(void) {
    uint32_t idx;
    uint32_t level;
    volatile uint8_t* latch;
    uint8_t code = 0u;
    uint8_t done;

    vehicle_fault_aggregate();
    control_gesture_fsm();
    for (idx = 0u; (idx < sizeof(prv_test_mon)) && (code == 0u); ++idx) {
        if (g_evt.mon[prv_test_mon[idx]].code == 1u) {
            g_evt.mon[prv_test_mon[idx]].code = 0u;
            code = (uint8_t)(idx + 1u);
        }
    }
    done = code;
    switch (code) {
        case VCU_EVT_LOCK: vehicle_event1_prep(); break;
        case VCU_EVT_LOCK_LATCH: vehicle_lock_antitheft_latch(); break;
        case VCU_EVT_08: vehicle_event8_prep(); break;
        case VCU_EVT_09: vehicle_event9_prep(); break;
        case VCU_EVT_10: vehicle_event10_prep(); break;
        default: break;
    }
    for (idx = 0u; (idx < sizeof(prv_travel_latch) / sizeof(prv_travel_latch[0])) && (done == 0u); ++idx) {
        level = (uint32_t)control_adc_raw((uint8_t)idx) >> 4;
        latch = prv_travel_latch[idx];
        if ((level - TRAVEL_IN_LO) < (TRAVEL_FULL - TRAVEL_IN_LO)) {
            *latch = 1u;
            done = 1u;
        } else if ((level >= TRAVEL_FULL) && (*latch == 1u)) {
            *latch = 0u;
            code = (uint8_t)(VCU_EVT_THROTTLE_ENDSTOP + idx);
            done = 1u;
        }
    }
    if (code != 0u) {
        report_ninebot(NB_ADDR_VCU, VCU_RPT_EVT_DST, VCU_RPT_EVT_LEN, VCU_RPT_EVT_CMD, code, prv_event_payload);
    }
}

/* ========================================================================= */
/* slot 0 (period 10): vehicle finite-state machine                          */
/* ========================================================================= */
void vehicle_task_fsm(void) {
    uint8_t start;

    /* State change latched? Clear the vehicle-state struct and notify helpers. */
    if (g_prev.prev_state != FSM) {
        g_prev.prev_state = FSM;
        passcode_entry_reset();
        vehicle_event_block_clear();
        g_prev.mode_counter = 0;
        g_prev.chg_plug_edge = 0;
        g_mode.cfg_17 = (uint8_t)(g_mode.cfg_17 & ~CFG17_DRIVE_FAULT); /* cfg_17 &= 0xF3 */
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~(MF_ACTIVE | MF_ARMED));
        g_vs.shutdown_req = 0;
        g_vs.auto_off_ticks = 0;
        g_vs.power_ready = 0;
        g_vs.unlock_ok = 0;
        g_vs.power_released = 0;
        g_vs.chg_full = 0;
        g_vs.chg_disp_refresh_req = 0;
        g_vs.can_drive_active = 0;
        g_vs.creep_enable = 1;
        g_vs.code_retries = 0;
        g_vs.drive_fault_state = 0;
        g_vs.chg_disp_refresh = 0;
        g_vs.fault_ack = 0;
        g_vs.input_test_mode = 0;
        g_vs.input_test_ea = 0;
        g_vs.lock_timer = LOCK_TIMER_INIT;
        if (FSM != VCU_FSM_POWER_ON) {
            g_vs.start_req = VS_START_NONE;
        }
    }

    /* Shared pre-pass (skipped in state 5 = sleep): attitude update, power monitor, then the
       mileage pre-pass. */
    if (FSM != VCU_FSM_SLEEP) {
        vehicle_fsm_prepass();
        control_power_monitor_fsm();
        odometer_mileage_prepass();
    }
    control_debounced_gpio_drive(); /* control.c */
    vcu_can_senders();
    report_telemetry_build();
    passcode_set_confirm_tick();

    if (g_vs.input_test_mode == 0) {
        vehicle_charger_edge_update();

        /* Normal FSM dispatch. */
        switch (FSM) {
            case VCU_FSM_IDLE:
                vehicle_state_idle();
                vehicle_idle_dispatch();
                break;

            case VCU_FSM_POWER_ON:
                vehicle_state_power_on();
                if (g_vs.power_ready != 0) {
                    if (g_vs.shutdown_req == 0) {
                        start = g_vs.start_req;
                        if (start == VS_START_NO_LOCK || start == VS_START_UNLOCKED) {
                            FSM = VCU_FSM_RIDE;
                        } else if (start == VS_START_LOCKED) {
                            FSM = VCU_FSM_LOCKED;
                        }
                    } else {
                        FSM = VCU_FSM_POWER_DOWN;
                    }
                }
                break;

            case VCU_FSM_LOCKED:
                vehicle_state_locked();
                if (g_vs.unlock_ok == 0) {
                    if (g_vs.shutdown_req == 0) {
                        if (g_vs.lockout_phase == LOCKOUT_ALARM) {
                            FSM = VCU_FSM_ALARM;
                        }
                    } else {
                        FSM = VCU_FSM_POWER_DOWN;
                    }
                } else {
                    FSM = VCU_FSM_RIDE;
                }
                break;

            case VCU_FSM_RIDE:
                vehicle_state_ride();
                if (g_vs.shutdown_req == 0) {
                    if (g_in1.charger_connected != 0 && (g_in2.f.status_flags & SF_SESSION) != 0) {
                        FSM = VCU_FSM_CHARGING;
                    }
                } else {
                    FSM = VCU_FSM_POWER_DOWN;
                }
                break;

            case VCU_FSM_POWER_DOWN:
                vehicle_state_power_down();
                if (g_vs.power_released != 0) {
                    if ((g_in1.charger_connected == 0 || (g_in2.f.status_flags & SF_SESSION) == 0)
                        && g_vs.chg_keep_awake == 0) {
                        FSM = VCU_FSM_SLEEP;
                    } else {
                        prv_power_restore();
                        FSM = VCU_FSM_CHARGE_STANDBY;
                        g_vs.chirp_req = 1;
                    }
                }
                break;

            case VCU_FSM_SLEEP:
                vehicle_state_sleep();
                if (g_vs.power_ready != 0 && g_vs.wake_src != VCU_GPIO_EVENT_NONE) {
                    if (g_vs.wake_src == VCU_GPIO_EVENT_PC13) {
                        FSM = VCU_FSM_ALARM;
                        g_vs.tilt_latch = 0;
                        g_vs.wake_src = VCU_GPIO_EVENT_NONE;
                    } else {
                        FSM = VCU_FSM_IDLE;
                    }
                }
                break;

            case VCU_FSM_ALARM:
                vehicle_state_alarm();
                if (g_evt.mon[FM_MON_POWER_BTN].code == 1 || g_vs.app_start_req != 0) {
                    FSM = VCU_FSM_IDLE;
                    g_evt.mon[FM_MON_POWER_BTN].code = 0;
                    g_vs.app_start_req = 1;
                    g_vs.lockout_phase = LOCKOUT_ARMED;
                } else if (g_vs.tilt_latch == 1 || g_vs.shutdown_req != 0 || g_vs.remote_unlocked != 0) {
                    FSM = VCU_FSM_POWER_DOWN;
                    g_vs.tilt_latch = 0;
                    if (g_vs.remote_unlocked != 0) {
                        g_vs.poweroff_beep = 1;
                    }
                }
                break;

            case VCU_FSM_CHARGING:
                vehicle_state_charging();
                if (g_in1.charger_connected == 0 || g_vs.shutdown_req != 0) {
                    FSM = VCU_FSM_POWER_DOWN;
                }
                break;

            case VCU_FSM_CHARGE_STANDBY:
                vehicle_state_charge_standby();
                if (g_vs.postride_timer == 0) {
                    if ((g_stat.status_word_b & BMS_ST_CHARGER) != 0 || g_vs.chg_sched_cancel_notice != 0) {
                        if (g_evt.mon[FM_MON_POWER_BTN].code == 1 || g_vs.app_start_req != 0) {
                            prv_power_press_report();
                            if ((g_in2.f.cfg_flags & CFG_LOCK_EN) != 0) {
                                FSM = VCU_FSM_LOCKED;
                            } else if (g_scr.ui_screen == 0x12 || g_in1.charger_connected == 0) {
                                FSM = VCU_FSM_RIDE;
                            } else {
                                FSM = VCU_FSM_CHARGING;
                                g_vs.app_start_req = 0;
                            }
                        }
                    } else {
                        FSM = VCU_FSM_POWER_DOWN;
                    }
                }
                break;

            default: break;
        }
    } else {
        prv_input_test();
    }
}
