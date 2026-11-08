/**
 * \file            faultmon.c
 * \brief           Fault monitor: sensor-fault debounce FSMs, per-node comms-loss
 *                      watchdogs, fault-code display builders, periodic status/telemetry
 *                      builder and the retry -> reset/erase trigger (x3 VCU)
 * \note            See faultmon.h for the monitor map. The SRAM blackboards are reached
 *                      through the vcu_state.h shared store, via the typed overlays
 *                      (VS/IN1/IN2/STAT/SCR/MODE/EVT/A3C/TMR/LOW and the per-monitor fm_mon_t
 *                      window). The hw-input flag accessors read the GPIO input-data register
 *                      directly; the cross-module DTC/status/shutdown callees go through their
 *                      owning modules' headers.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "faultmon.h"
#include "at32f415.h" /* CMSIS NVIC_SystemReset + Artery GPIO HAL (charge relay) */
#include "charging.h" /* g_charge_timer (FM_CHG_DWELL +3) */
#include "vcu_state.h"

/* ------------------------------------------------------------------------- */
/* Shared-blackboard views (like control.c): each typed overlay aliases the ONE */
/* backing store in vcu_state.h, so a write here is seen by every other module   */
/* that reads the same field. Offsets are pinned in the respective headers.  */
/* ------------------------------------------------------------------------- */

/* The monitors own records of the event block (vcu_state.h fm_mon_id_t). */
#define MON(id)     (&g_evt.mon[(id)])
#define WD_NODE1    MON(FM_MON_SUBSTATE_PREV) /* record 2  (report event 7)  */
#define WD_NODE2    MON(FM_MON_SUBSTATE_NEXT) /* record 1  (report event 8)  */
#define WD_NODE3    MON(FM_MON_INTERLOCK)     /* record 3  (report event 6)  */
#define WD_NODE4    MON(FM_MON_GEAR_UP)       /* record 5  (report event 3)  */
#define WD_NODE5    MON(FM_MON_GEAR_DOWN)     /* record 4  (report event 4)  */
#define WD_NODE6    MON(FM_MON_PARK)          /* record 6  (report event 2)  */
#define FSM_SENSOR1 MON(FM_MON_LIGHT_EN)      /* record 9  (report event 5)  */
#define FSM_SENSOR2 MON(FM_MON_LIGHT_HI)      /* record 8  (report event 10) */
#define FSM_SENSOR3 MON(FM_MON_LIGHT_LO)      /* record 7  (report event 9)  */

/* Scalar counters that no typed overlay names. */
static volatile uint16_t g_fm_retry;              /* fm_retry_cnt, a lone faultmon-owned global */
#define FM_CHG_DWELL (g_charge_timer.relay_dwell) /* charge-relay debounce dwell (charge-timer +3) */
#define FM_RETRY     (g_fm_retry)                 /* fm_retry_cnt (> 0x13 resets); lone faultmon cell */

/*
 * Gesture / sensor-fault block g_fm_gest.
 * faultmon owns the sensor-A/B confirm counters (+1/+2), the gesture confirm (+3) and
 * dwell (+4); byte +0 (gear_backup) is ridemode's parked-gear backup, shared through
 * vcu_faultmon_gear_backup() (faultmon.h). One typed block names every cell.
 */
typedef struct {
    uint8_t gear_backup; /*!< +0x00 ridemode parked-gear backup (park toggle); cross-module via accessor */
    uint8_t sens_a_cnt;  /*!< +0x01 faultmon sensor-A confirm counter (debounce A) */
    uint8_t sens_b_cnt;  /*!< +0x02 faultmon sensor-B confirm counter (debounce B) */
    uint8_t confirm;     /*!< +0x03 gesture hold-confirm up-counter (clamp 100)     */
    int16_t dwell;       /*!< +0x04 gesture dwell / action-window timer             */
} fm_gesture_t;

static volatile fm_gesture_t g_fm_gest; /* gesture/sensor-fault block, faultmon-owned global */

#define FM_SENS_A_CNT (g_fm_gest.sens_a_cnt) /* debounce A confirm counter (block +1) */
#define FM_SENS_B_CNT (g_fm_gest.sens_b_cnt) /* debounce B confirm counter (block +2) */

/* Their latch windows are the last two event-block records: debounce A writes the
   code byte of record 10 (g_evt+0x80), debounce B that of record 11 (+0x8C); only
   `code` is touched. Record 10's `hold` (+0x7C) is the gesture index below. */
#define BRAKE_FULL_WIN                                                                                                 \
    MON(FM_MON_BRAKE_FULL) /* PA4 full-weight brake lever held window (code @ +0x80) */
#define BRAKE_HALF_WIN                                                                                                 \
    MON(FM_MON_BRAKE_HALF)                   /* PA1 half-weight brake lever held window (code @ +0x8C) */
#define FM_GEST_INDEX (BRAKE_FULL_WIN->hold) /* g_evt+0x7C gesture index (0..GEST_INDEX_WRAP) */

/* ------------------------------------------------------------------------- */
/* Debounce / threshold constants.                                           */
/* ------------------------------------------------------------------------- */
#define FM_SHIFT_FULL 0x0F /* shift register saturated (4 consecutive lost reads)   */
#define FM_ESCALATE_HOLD                                                                                               \
    100 /* two-stage dwell ticks before escalating code 4 -> 3                                                         \
         */
#define FM_SENSOR1_TRIP        10   /* sensor FSM 1: counter value that latches a warn        */
#define FM_SENSOR1_MAX         500  /* sensor FSM 1: counter clamp                            */
#define FM_RETRY_LIMIT         19   /* retry counter -> system reset                         */

/* Brake-lever held debounce A / B. */
#define FM_SENS_ADC_SHIFT      4    /* raw ADC reading is scaled down by >>4                  */
#define FM_SENS_ADC_THRESH     50   /* scaled reading at/above 50 == fault reading            */
#define FM_SENS_CONFIRM        4    /* counter must pass 4 (5 consecutive) to latch code 4    */

/* Fault code values written into a window's `code` byte. */
#define FM_CODE_WARN           1 /* active warning (also the demoted value of a latch) */
#define FM_CODE_FAULT          3 /* escalated hard fault                               */
#define FM_CODE_LATCH          4 /* latched / confirmed, pending escalate-or-demote    */
#define FM_CODE_SENSOR1_LO     5 /* sensor FSM 1: cleared-after-fault code             */

/* Periodic status/telemetry builder thresholds. */
#define FM_ESC_COMMLOSS_LIMIT  200    /* ESC comm-loss ticks before DTC 10 + CAN mode reset */
#define FM_BMS_COMMLOSS_LIMIT  400    /* BMS comm-loss ticks before event 0x12              */
#define FM_UV_LIMIT_NORMAL     1000   /* under-volt debounce, normal                        */
#define FM_UV_LIMIT_CHARGE     6000   /* under-volt debounce, while charging                */
#define FM_CHG_RELAY_DWELL     200    /* charge-relay debounce span (relay on at the top)   */
#define FM_TEMP_HIGH           600    /* over-temperature threshold (0.1 degC)              */
#define FM_TEMP_WARN_ABOVE     579    /* battery-2 warning band: 579 < t < 600              */
#define FM_TEMP_LOW_DECI       (-12)  /* under-temperature threshold (value / 10 < -12)     */
#define FM_VBUS_SENTINEL_A     0xFD   /* battery-code sentinel A                            */
#define FM_VBUS_SENTINEL_B     0xFE   /* battery-code sentinel B                            */
#define FM_VBUS_THRESHOLD      191    /* battery-code high threshold                        */
#define FM_RAW_THRESHOLD       0x6E   /* raw fault-code threshold for status bit 0x16       */
#define FM_EVT_BATT2_TEMP_WARN 0x1A   /* battery-2 pack warm (580..599 x 0.1 degC)         */

/* ------------------------------------------------------------------------- */
/* Safety-interlock FSM state + inputs. The FSM keeps its state in the g_tmr */
/* block (vcu_state.h).                                                      */
/* ------------------------------------------------------------------------- */

#define ILK_ARM_TIMEOUT        300 /* state-1: ticks before the arm request times out     */
#define ILK_MISMATCH_TRIP      10  /* state-2: command-mismatch debounce limit            */
#define ILK_FAULT_CODE         8   /* mode/lighting report code raised on interlock fault */
#define ILK_MIN_SPEED_KMH      6   /* speed_kmh floor: arm only at/above it, abort below  */

/* 12-bit debounce shift register (CAN error-code monitor), vs the 4-bit FM_SHIFT_FULL above. */
#define FM_SHIFT_FULL12        0xFFF

/* ------------------------------------------------------------------------- */
/* hw-input accessors. Each reads the comms/sensor watchdog's discrete input   */
/* pin(s) off the Artery GPIO input-data register and returns the fault/absent  */
/* boolean the FSM consumes. Ports are GPIOA (0x40010800) and GPIOB          */
/* (0x40010C00); the masks per node/sensor are listed below.                 */
/* ------------------------------------------------------------------------- */

/* TRUE iff every pin in `mask` is driven high on `port`'s input-data register
   (IDT @ base+8): (mask & ~port->idt) == 0. */
static uint32_t fm_pins_all_high(gpio_type* port, uint32_t mask) {
    return ((mask & ~(uint32_t)port->idt) == 0u) ? 1u : 0u;
}

/* TRUE iff every pin in `mask` reads low on `port`'s input-data register. The
   per-node comms and sensor lines are wired active-low, so "all low" == "lost".
   The watchdog call sites test these two primitives inline (GPIOA 0x40010800,
   GPIOB 0x40010C00):
     node1 PA11, node2 PB7+PB6, node3 PA12, node4 PB11, node5 PB2, node6 PB1,
     sensor1 PA9, sensor2/3 PB7^PB6, CAN-err PA10. */
static uint32_t fm_pins_all_low(gpio_type* port, uint32_t mask) {
    return ((mask & (uint32_t)port->idt) == 0u) ? 1u : 0u;
}

/* ------------------------------------------------------------------------- */
/* Cross-module callees, reached through their owning modules' headers:        */
/* report framer (usart.h), mode/lighting reporter (control.h                */
/* control_mode_report_fsm), beeps (buzzer.h).                               */
/* ------------------------------------------------------------------------- */
#include "buzzer.h" /* buzzer_beep */
#include "can.h"    /* vcu_can_mode_reset */
#include "control.h"
#include "gpio.h"    /* vcu_gpio_pa10_active */
#include "ninebot.h" /* nb_commit_mode */
#include "report.h"  /* VCU_RPT_DST_U16 */
#include "usart.h"   /* vcu_usart_report_send, NB_ADDR_* / NB_CMD_WRITE3 */
#include "vehicle.h" /* vehicle_event_block_clear */

/* Charge-relay GPIO drive: GPIOB pin 12. on == 1 energizes the
   relay (SCR set); any other value releases it (CLR reset). */
void fm_charge_relay(int32_t on) {
    if (on == 1) {
        gpio_bits_set(GPIOB, GPIO_PINS_12);
    } else {
        gpio_bits_reset(GPIOB, GPIO_PINS_12);
    }
}

/* System reset: DSB, write SCB->AIRCR with VECTKEY|SYSRESETREQ, DSB, then spin until
   the core resets -- exactly CMSIS NVIC_SystemReset(). There is no flash erase (it is a
   plain AIRCR system-reset request). */
void fm_system_reset(void) { NVIC_SystemReset(); }

/* Set/clear a DTC or status code from a boolean condition (collapses the
   `if (cond) set(code); else clear(code);` pairs, which appear ~14 times below). */
static void fm_dtc_update(uint8_t code, int active) {
    if (active) {
        control_dtc_set(code);
    } else {
        control_dtc_clear(code);
    }
}

static void fm_status_update(uint8_t code, int active) {
    if (active) {
        control_event_set(code);
    } else {
        control_event_clear(code);
    }
}

/* ------------------------------------------------------------------------- */
/* Debounce engine. Every comms watchdog and sensor FSM feeds a 4-bit shift    */
/* register: a clean read arms the monitor and zeroes the register; a lost read */
/* shifts in a 1 and raises the global "fault seen" flag. Confirmation fires    */
/* only once the register saturates (0xF == four consecutive lost reads) AND    */
/* the monitor is armed.                                                     */
/* ------------------------------------------------------------------------- */

/* Shift the debounce register for one monitor from a lost/ok reading. */
static void fm_debounce_feed(volatile fm_mon_t* w, uint32_t lost) {
    if (lost == 0U) {
        w->armed = 1;
        w->shift = 0;
    } else {
        g_vs.chg_disp_refresh = 1;
        w->shift = (uint16_t)((w->shift * 2u + 1u) & 0xFu);
    }
}

/**
 * \brief           Single-stage monitor: node watchdogs 1-5
 * \param[in]       trip_code: code latched on confirmation (1 for most, 4 for node 3)
 * \note            On confirmation the monitor latches \p trip_code once; on recovery a
 *                      latched code 4 is demoted to the active warning 1 (node 3 only --
 *                      the others never hold code 4, so the demote is a no-op for them).
 */
static void fm_watchdog(volatile fm_mon_t* w, uint32_t lost, uint8_t trip_code) {
    fm_debounce_feed(w, lost);

    if (w->shift == FM_SHIFT_FULL) {
        if (w->armed == 0) {
            return;
        }
        if (w->phase == 0) {
            w->code = trip_code;
            w->phase = 1;
        }
    } else {
        w->phase = 0;
        if (w->code == FM_CODE_LATCH) {
            w->code = FM_CODE_WARN;
        }
    }
    if (w->code != 0) {
        g_vs.can_drive_active = 1;
    }
}

/**
 * \brief           Two-stage monitor: node watchdog 6 + sensor FSMs 2/3
 * \note            Confirmation latches code 4, then holds FM_ESCALATE_HOLD ticks before
 *                      escalating to the hard fault code 3. Recovery clears the dwell and
 *                      demotes a still-latched code 4 back to the active warning 1.
 */
static void fm_monitor_two_stage(volatile fm_mon_t* w, uint32_t lost) {
    fm_debounce_feed(w, lost);

    if (w->shift == FM_SHIFT_FULL) {
        if (w->armed == 0) {
            return;
        }
        if (w->phase == 0) {
            w->code = FM_CODE_LATCH;
            w->phase = 1;
        } else if (w->phase == 1) {
            if (w->hold < FM_ESCALATE_HOLD) {
                w->hold = (uint16_t)(w->hold + 1);
            } else {
                w->code = FM_CODE_FAULT;
                w->phase = 2;
            }
        }
    } else {
        w->hold = 0;
        w->phase = 0;
        if (w->code == FM_CODE_LATCH) {
            w->code = FM_CODE_WARN;
        }
    }
    if (w->code != 0) {
        g_vs.can_drive_active = 1;
    }
}

/**
 * \brief           Sensor FSM 1: count-up debounce, no shift register
 * \note            A clean read clears any fault (code 5 if the counter had advanced) and
 *                      re-arms; a sustained fault counts up (clamped at 500) and latches the
 *                      active warning once the counter reaches 10. `hold` is the counter and
 *                      `code` is the latch for this monitor.
 */
static void fm_sensor_fsm_1(void) {
    volatile fm_mon_t* w = FSM_SENSOR1;

    if (fm_pins_all_low(GPIOA, GPIO_PINS_9) == 0U) {
        if (w->hold != 0) {
            w->code = FM_CODE_SENSOR1_LO;
        }
        w->hold = 0;
        w->armed = 1;
    } else {
        g_vs.chg_disp_refresh = 1;
        if (w->armed == 0) {
            return;
        }
        if (w->hold < FM_SENSOR1_MAX) {
            w->hold = (uint16_t)(w->hold + 1);
            if (w->hold == FM_SENSOR1_TRIP) {
                w->code = FM_CODE_WARN;
            }
        }
    }
    if (w->code != 0) {
        g_vs.can_drive_active = 1;
    }
}

/* ------------------------------------------------------------------------- */
/* Brake-lever held debounce FSMs (A = PA4 full-weight lever, B = PA1        */
/* half-weight lever).                                                       */
/*                                                                           */
/* Each reads its lever ADC leaf, scales it down by >>4 and compares it to the  */
/* threshold (50). A reading below the threshold (lever released) zeroes the    */
/* confirm counter and clears the window's latched code. A reading at/above it   */
/* (lever held) counts up; once the counter passes 4 (the 5th consecutive held  */
/* reading) it latches the window code to 4 (FM_CODE_LATCH) and then holds there  */
/* until a released reading clears it.                                       */
/* The charge timer (charging.c) consumes the latched code-4 window state.         */
/* ------------------------------------------------------------------------- */

/** \brief PA4 full-weight brake lever held debounce (evt+0x80). */
void vcu_faultmon_sensor_debounce_a(void) {
    uint8_t cnt;

    if ((control_adc_raw(VCU_ADC_SLOT_BRAKE_FULL) >> FM_SENS_ADC_SHIFT) < FM_SENS_ADC_THRESH) {
        FM_SENS_A_CNT = 0;        /* released: reset the counter ... */
        BRAKE_FULL_WIN->code = 0; /* ... and clear the latched window code */
        return;
    }
    cnt = FM_SENS_A_CNT;
    if (cnt > FM_SENS_CONFIRM) {
        return; /* already latched -- hold until a healthy reading clears it */
    }
    cnt = (uint8_t)(cnt + 1);
    FM_SENS_A_CNT = cnt;
    if (cnt > FM_SENS_CONFIRM) {
        BRAKE_FULL_WIN->code = FM_CODE_LATCH; /* 5th consecutive held reading -> latch code 4 */
    }
}

/** \brief PA1 half-weight brake lever held debounce (evt+0x8C). */
void vcu_faultmon_sensor_debounce_b(void) {
    uint8_t cnt;

    if ((control_adc_raw(VCU_ADC_SLOT_BRAKE_HALF) >> FM_SENS_ADC_SHIFT) < FM_SENS_ADC_THRESH) {
        FM_SENS_B_CNT = 0;
        BRAKE_HALF_WIN->code = 0;
        return;
    }
    cnt = FM_SENS_B_CNT;
    if (cnt > FM_SENS_CONFIRM) {
        return;
    }
    cnt = (uint8_t)(cnt + 1);
    FM_SENS_B_CNT = cnt;
    if (cnt > FM_SENS_CONFIRM) {
        BRAKE_HALF_WIN->code = FM_CODE_LATCH;
    }
}

/* ------------------------------------------------------------------------- */
/* Retry -> reset/erase trigger + periodic status/telemetry builder.           */
/* ------------------------------------------------------------------------- */

/* Retry counter -> system-reset trigger. When the retry request flag is
   set, the retry counter is bumped; past FM_RETRY_LIMIT it is reset and the system reset
   fires. The recompute request is raised either way. Called from
   the vehicle FSM (vehicle.c). */
void vcu_faultmon_retry_trigger(void) {
    uint16_t next;

    if (g_in1.retry_req == 1) {
        next = (uint16_t)(FM_RETRY + 1);
        FM_RETRY = next;
        if (next > FM_RETRY_LIMIT) {
            FM_RETRY = 0;
            fm_system_reset();
        }
        g_in1.recompute_req = 1;
    }
}

/* Periodic status/telemetry flag + DTC report builder. For each monitored
   condition it sets or clears the matching DTC/status code, advances the pre-arm/report/
   under-volt debounce counters, drives the charge relay and re-inits CAN on ESC comm loss. */
void fm_status_report(void) {
    uint16_t uv_limit = FM_UV_LIMIT_NORMAL;
    uint8_t dwell;
    int16_t temp;

    fm_dtc_update(0x0E, g_in1.throttle_fault_latch != 0);
    fm_dtc_update(0x0F, g_in1.brake_full_range_fault != 0); /* PA4 full-weight brake lever out of band */
    fm_dtc_update(0x10, g_in1.brake_half_range_fault != 0); /* PA1 half-weight brake lever out of band */

    /* ESC comm loss: FM_ESC_COMMLOSS_LIMIT ticks without an ESC-group frame -> DTC 10 + CAN reset. */
    if (g_lowglob.esc_commloss_cnt < FM_ESC_COMMLOSS_LIMIT) {
        g_lowglob.esc_commloss_cnt = (uint16_t)(g_lowglob.esc_commloss_cnt + 1);
        control_dtc_clear(10);
    } else {
        vcu_can_mode_reset();
        control_dtc_set(10);
    }

    /* Mirror the active fault-level code into the per-code DTC flags. */
    fm_dtc_update(0x0B, (uint8_t)g_fault_codes.active_fault_code == 0x0B);
    fm_dtc_update(0x0D, (uint8_t)g_fault_codes.active_fault_code == 0x0D);
    fm_dtc_update(0x12, (uint8_t)g_fault_codes.active_fault_code == 0x12);
    fm_dtc_update(0x18, (uint8_t)g_fault_codes.active_fault_code == 0x18);
    fm_dtc_update(0x23, g_in1.dtc23_in != 0);
    fm_dtc_update(0x37, (uint8_t)g_fault_codes.active_fault_code == 0x37);
    fm_dtc_update(0x38, (uint8_t)g_fault_codes.active_fault_code == 0x38);
    fm_dtc_update(0x1E, ((int32_t)((uint32_t)g_stat.status_word_a << 0x13) < 0)          /* W124 bit12 */
                            && ((int32_t)((uint32_t)g_stat.status_word_b << 0x1C) < 0)); /* W126 bit3 */

    /* Publish the active-error code mirror where control.c reads it (IN2+0xB0). */
    g_in2.f.lock_state = g_lowglob.active_error_code;

    /* BMS comm loss: FM_BMS_COMMLOSS_LIMIT ticks without a BMS-group frame -> event 0x12. */
    if (g_bms_commloss.cnt < FM_BMS_COMMLOSS_LIMIT) {
        g_bms_commloss.cnt = (uint16_t)(g_bms_commloss.cnt + 1);
        control_event_clear(0x12);
    } else {
        control_event_set(0x12);
    }

    /* Under-volt debounce: a longer window while charging; on timeout, debounce the
       charge relay over FM_CHG_RELAY_DWELL ticks (relay off at 0, on at the top). */
    if (g_in1.uv_grace_ext != 0) {
        uv_limit = FM_UV_LIMIT_CHARGE;
    }
    if (g_lowglob.fm_uv_cnt < uv_limit) {
        g_lowglob.fm_uv_cnt = (uint16_t)(g_lowglob.fm_uv_cnt + 1);
        FM_CHG_DWELL = 0;
        control_event_clear(0x19);
    } else {
        dwell = FM_CHG_DWELL;
        if (dwell < FM_CHG_RELAY_DWELL) {
            if (dwell == 0) {
                fm_charge_relay(0);
            }
            dwell = (uint8_t)(dwell + 1);
            FM_CHG_DWELL = dwell;
            if (dwell == FM_CHG_RELAY_DWELL) {
                fm_charge_relay(1);
            }
        }
        control_event_set(0x19);
    }

    /* Temperature window: nothing is judged before the first BMS 0x425 frame. Event 4 = too
       hot, 5 = too cold; with battery 2 present the band just below the hot limit raises the warning 0x1A and
       leaves event 4 as it was. */
    if (g_in1.bms_temp_rx == 0) {
        control_event_clear(4);
        control_event_clear(5);
    } else {
        temp = (int16_t)g_in2.f.temp_425;
        if (temp >= FM_TEMP_HIGH) {
            control_event_set(4);
        } else if ((temp > FM_TEMP_WARN_ABOVE) && (g_scr.bat2_present != 0)) { /* battery 2 present */
            control_event_set(FM_EVT_BATT2_TEMP_WARN);
        } else {
            control_event_clear(4);
            control_event_clear(FM_EVT_BATT2_TEMP_WARN);
        }
        fm_status_update(5, (int32_t)temp / 10 < FM_TEMP_LOW_DECI);
    }

    /* Battery-voltage code (hi byte): sentinels force status bit 0x14; else threshold 0x15. */
    if (((g_fault_codes.vbus_code >> 8) == FM_VBUS_SENTINEL_A) || ((g_fault_codes.vbus_code >> 8) == FM_VBUS_SENTINEL_B)) {
        control_event_set(0x14);
    } else {
        control_event_clear(0x14);
        fm_status_update(0x15, (g_fault_codes.vbus_code >> 8) >= FM_VBUS_THRESHOLD);
    }

    /* The original firmware guards this 0x16 update with `raw_code == -3 && raw_code == -2`.
       A signed byte can't be both, so the return is dead and the update always runs - almost
       certainly a vendor bug, since the vbus block above uses `||` for the same 0xFD/0xFE
       (= -3/-2) sentinels. Define FM_RAW_SENTINEL_FIX to keep the firmware's construct verbatim. */
#ifdef FM_RAW_SENTINEL_FIX
    if ((g_fault_codes.raw_code == -3) && (g_fault_codes.raw_code == -2)) {
        return;
    }
#endif
    /* Unsigned byte compare against FM_RAW_THRESHOLD. */
    fm_status_update(0x16, FM_RAW_THRESHOLD < (uint8_t)g_fault_codes.raw_code);
}

/* ------------------------------------------------------------------------- */
/* CAN error-code debounce + two-stage escalation.                           */
/* ------------------------------------------------------------------------- */

/* Structurally the two-stage monitor, but with a 12-bit shift register (mask 0xFFF)
   fed from the CAN error-code input, operating on the lock record (g_evt+0). It
   does NOT raise the global fault flag. */
void vcu_faultmon_can_fault_escalate(void) {
    volatile fm_mon_t* w = MON(FM_MON_POWER_BTN);

    if (fm_pins_all_low(GPIOA, GPIO_PINS_10) == 0U) {
        w->shift = 0;
    } else {
        w->shift = (uint16_t)((w->shift * 2u + 1u) & FM_SHIFT_FULL12);
    }

    if (w->shift == FM_SHIFT_FULL12) {
        if (w->armed != 0) {
            if (w->phase == 0) {
                w->phase = 1;
                w->code = FM_CODE_LATCH; /* 4 */
            } else if (w->phase == 1) {
                if (w->hold < FM_ESCALATE_HOLD) {
                    w->hold = (uint16_t)(w->hold + 1);
                } else {
                    w->phase = 2;
                    w->code = FM_CODE_FAULT; /* 3 */
                }
            }
        }
    } else {
        w->hold = 0;
        w->phase = 0;
        w->armed = 1;
        if (w->code == FM_CODE_LATCH) {
            w->code = FM_CODE_WARN; /* demote 4 -> 1 */
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Safety-interlock state machine.                                           */
/* ------------------------------------------------------------------------- */

/* Three-state interlock (idle=0 / armed=1 / engaged=2) over the g_tmr block. While
   the entry gate is open it arms on a request, waits up to 300 ticks for bus confirmation,
   then watches the live command against the captured reference and aborts on mismatch.
   Raises fault 8 via the mode/lighting reporter and drives the buzzer; publishes the
   active flag at SCR+0x52. */
void vcu_faultmon_interlock_tick(void) {
    uint16_t next;

    if (((int32_t)((uint32_t)g_in2.f.status_flags << 0x14) < 0) /* IN2+0x38 bit11          */
        && (g_vs.region == 0)                                /* vehicle idle            */
        && (((g_vs.speed_kmh <= (uint8_t)(g_mode.cmd_lead >> 1)) && (g_in1.throttle_flag != 0))
            || ((int32_t)((uint32_t)g_mode.mode_flags << 0x1E) < 0))) { /* ...or MODE+0xa bit1  */

        /* Output-config bit4 (MODE+0x15) or an active error code in {0x0A,0x0E,0x0F,0x10} forces
           the FSM back to idle. */
        if (((g_mode.output_cfg & OCFG_BRAKE_CAP) != 0u) || (g_lowglob.active_error_code == 0x0A)
            || (g_lowglob.active_error_code == 0x0E) || (g_lowglob.active_error_code == 0x0F)
            || (g_lowglob.active_error_code == 0x10)) {
            g_tmr.ilk_state = 0;
            if (g_evt.mon[FM_MON_INTERLOCK].code == 1) {
                g_evt.mon[FM_MON_INTERLOCK].code = 0;
            }
        }

        switch (g_tmr.ilk_state) {
            case 0: /* idle: on a request, arm if permitted and above the speed floor */
                if (g_evt.mon[FM_MON_INTERLOCK].code == 1) {
                    g_evt.mon[FM_MON_INTERLOCK].code = 0;
                    if (((int32_t)((uint32_t)g_mode.mode_flags << 0x1D) < 0) /* MODE+0xa bit2 */
                        && (g_vs.speed_kmh >= ILK_MIN_SPEED_KMH)) {
                        g_tmr.ilk_state = 1;
                        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | 2u);
                    } else {
                        control_mode_report_fsm(ILK_FAULT_CODE);
                    }
                } else {
                    g_tmr.ilk_zero_flag = 0;
                    g_tmr.ilk_tmr = 0;
                    g_tmr.ilk_ref_cmd = 0;
                    g_tmr.ilk_fault_flag = 0;
                    g_tmr.ilk_cnt = 0;
                    g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~2u);
                }
                break;

            case 1: /* armed: engage on bus confirmation, else time out after 300 ticks */
                if ((int32_t)((uint32_t)g_fault_codes.comm_fault_word << 0x1D) < 0) { /* A3C+0 bit2 */
                    g_tmr.ilk_state = 2;
                    g_tmr.ilk_zero_flag = 0;
                    g_tmr.ilk_tmr = 0;
                    buzzer_beep(BEEP_DOUBLE);
                    g_tmr.ilk_ref_spd = g_vs.gear_sel;
                    g_tmr.ilk_ref_cmd = (uint8_t)g_mode.drive_output_word;
                } else {
                    next = (uint16_t)(g_tmr.ilk_tmr + 1);
                    g_tmr.ilk_tmr = next;
                    if (next > ILK_ARM_TIMEOUT) {
                        g_tmr.ilk_state = 0;
                        g_tmr.ilk_tmr = 0;
                        buzzer_beep(BEEP_LONG);
                    }
                }
                break;

            case 2: /* engaged: abort on command drift, release, abort input or speed drop */
                if ((uint32_t)(g_tmr.ilk_ref_cmd + 10) < (uint32_t)(uint8_t)g_mode.drive_output_word) {
                    next = (uint16_t)(g_tmr.ilk_cnt + 1);
                    g_tmr.ilk_cnt = next;
                    if (next > ILK_MISMATCH_TRIP) {
                        g_tmr.ilk_fault_flag = 1;
                        g_tmr.ilk_cnt = 0;
                    }
                } else if (g_evt.mon[FM_MON_INTERLOCK].code == 1) {
                    g_tmr.ilk_fault_flag = 1;
                    g_tmr.ilk_cnt = 0;
                    g_evt.mon[FM_MON_INTERLOCK].code = 0;
                }

                if (g_tmr.ilk_zero_flag == 0) {
                    if ((uint8_t)g_mode.drive_output_word == 0) {
                        g_tmr.ilk_zero_flag = 1;
                    }
                } else if ((uint8_t)g_mode.drive_output_word != 0) {
                    g_tmr.ilk_fault_flag = 1;
                }

                if ((g_in1.brake_any != 0) || (g_tmr.ilk_fault_flag != 0) || (g_vs.gear_sel != g_tmr.ilk_ref_spd)
                    || (g_vs.speed_kmh < ILK_MIN_SPEED_KMH)) {
                    g_tmr.ilk_state = 0;
                    buzzer_beep(BEEP_LONG);
                }
                break;

            default: break;
        }

        /* Publish the "interlock active" flag from MODE+0xa bit1. */
        if ((int32_t)((uint32_t)g_mode.mode_flags << 0x1E) < 0) {
            g_scr.ilk_publish = 1;
        } else {
            g_scr.ilk_publish = 0;
        }
    } else {
        /* Gate closed: drop to idle, clear the publish + allow-arm bits, ack any request. */
        g_tmr.ilk_state = 0;
        g_tmr.ilk_zero_flag = 0;
        g_scr.ilk_publish = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & ~2u);
        if (g_evt.mon[FM_MON_INTERLOCK].code == 1) {
            g_evt.mon[FM_MON_INTERLOCK].code = 0;
            control_mode_report_fsm(ILK_FAULT_CODE);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Shared report payload: both FSMs below enqueue the 14-byte fault frame whose */
/* body lives at IN2+0x20.                                                   */
/* ------------------------------------------------------------------------- */
#define FM_IN2_FRAME          (g_in2.f.serial) /* report body (IN2+0x20) */

/* ------------------------------------------------------------------------- */
/* Drive-enable fault latch + arm-timeout FSM.                               */
/*                                                                           */
/* A command-routed latch tied to the drive-output-enable gate (IN2+0x38 bit11, */
/* the same gate the interlock FSM watches). It keeps a one-shot "frame sent"    */
/* latch and a 300-tick arm timer in the g_tmr block (disjoint offsets from   */
/* the interlock cells above) and routes control event 0x0F set-vs-clear by the   */
/* command code.                                                             */
/* ------------------------------------------------------------------------- */

#define LATCH_CMD_ARM         1    /* command that primes the latch / fault frame     */
#define LATCH_CMD_RAISE       3    /* command that routes the event to the set path   */
#define LATCH_EVENT           0x0F /* control event raised(set)/cleared by this latch */
#define LATCH_ARM_TIMEOUT     299  /* arm timer trip (> 299 == 300 ticks)             */
#define LATCH_MODE_PRIME      11   /* mode committed on the arm edge                   */
#define LATCH_MODE_TIMEOUT    1    /* mode committed on arm timeout                    */
#define LATCH_VS_TIMEOUT_MARK 2    /* VS+0x01 value written on timeout                 */

/**
 * \brief           Drive-enable fault latch + arm timeout.
 * \param[in]       cmd: latch command / mode code (compared against 1 and 3)
 * \note            The event clear/raise calls are control.c's control_event_clear /
 *                      control_event_set; only the event code is passed.
 */
void vcu_faultmon_fault_latch(uint32_t cmd) {
    uint16_t tmr;

    /* Gate open (IN2+0x38 bit11): reset the latch, mark handled, clear event 0x0F. */
    if ((int32_t)((uint32_t)g_in2.f.status_flags << 0x14) < 0) {
        g_tmr.latch_arm_tmr = 0;
        g_tmr.latch_reported = 0;
        if (g_in1.arm_gate == 0) {
            g_in1.arm_gate = 1;
        }
        control_event_clear(LATCH_EVENT);
        return;
    }

    /* Gate closed: on the arm-command edge, enqueue the fault frame once + prime mode. */
    if ((cmd == LATCH_CMD_ARM) && (g_tmr.latch_reported == 0)) {
        g_tmr.latch_reported = 1;
        vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 0xe, NB_CMD_WRITE3, 0,
                              FM_IN2_FRAME);
        nb_commit_mode(LATCH_MODE_PRIME);
    } else if (cmd != LATCH_CMD_ARM) {
        g_tmr.latch_reported = 0;
    }

    /* Arm timer runs while unhandled and a brake lever is active; trips after 300 ticks. */
    if ((g_in1.arm_gate == 0) && (g_in1.brake_any == 1)) {
        tmr = (uint16_t)(g_tmr.latch_arm_tmr + 1);
        g_tmr.latch_arm_tmr = tmr;
        if (tmr > LATCH_ARM_TIMEOUT) {
            g_vs.gear_sel = LATCH_VS_TIMEOUT_MARK;
            g_in1.arm_gate = 1;
            nb_commit_mode(LATCH_MODE_TIMEOUT);
        }
    } else {
        g_tmr.latch_arm_tmr = 0;
    }

    /* Route the event: raise (set) on the report command, clear otherwise. */
    if (cmd == LATCH_CMD_RAISE) {
        control_event_set(LATCH_EVENT);
        return;
    }
    control_event_clear(LATCH_EVENT);
}

/* ------------------------------------------------------------------------- */
/* Throttle/brake gesture + command detection.                               */
/*                                                                           */
/* Watches the two throttle/brake ADC leaves while the rider holds them; a      */
/* sustained hold advances a 0..9 gesture index, and -- once the CAN input is    */
/* ready -- each index dispatches its own beeps, report frames and mode commits. */
/* A final stage keyed off the committed mode (SCR+0x4a) closes out the gesture.  */
/* ------------------------------------------------------------------------- */

/* The gesture / sensor-fault block g_fm_gest (fm_gesture_t) is defined up
   top with the other faultmon blackboard accessors. gear_backup (+0) is exposed to
   ridemode through vcu_faultmon_gear_backup() below. */

/** \brief Parked-gear backup cell (gesture block +0); shared with ridemode's park toggle. */
volatile uint8_t* vcu_faultmon_gear_backup(void) { return &g_fm_gest.gear_backup; }

/* The dispatched gesture indices (the switch over EVT+0x7c). */
enum fm_gesture_state {
    FM_GEST_PRESET = 3,  /* beep + sub-mode 0x3a frame                       */
    FM_GEST_ENTER = 5,   /* commit mode 0x0e, reload the short dwell window   */
    FM_GEST_CONFIRM = 8, /* fault frame + commit mode 0x0d + sub-mode 0x3b    */
    FM_GEST_RELEASE = 9  /* beep + sub-mode 0x20 frame                        */
};

#define GEST_ADC_MASK      0xFFF /* 12-bit ADC value mask                           */
#define GEST_ADC_SHIFT     4     /* raw ADC scaled down by >>4                      */
#define GEST_ADC_THRESH_A  50    /* channel A scaled-reading release threshold      */
#define GEST_ADC_THRESH_B  51    /* channel B scaled-reading release threshold      */
#define GEST_CONFIRM_CLAMP 100   /* confirm counter clamp                           */
#define GEST_CONFIRM_TRIP  5     /* confirm value that advances the index           */
#define GEST_INDEX_WRAP    9     /* gesture index ring wraps past 9                 */
#define GEST_WINDOW_LONG   300   /* dwell window loaded on a confirmed advance      */
#define GEST_WINDOW_SHORT  200   /* dwell window reloaded entering mode 0x0e        */
#define GEST_MODE_ENTER    14    /* mode committed on ENTER                         */
#define GEST_MODE_CONFIRM  13    /* mode committed on CONFIRM                       */
#define GEST_MODE_FINISH   1     /* mode committed on window expiry / clear         */
#define GEST_FINISH_MODE   14    /* SCR+0x4a value that drives the finish frame     */
#define GEST_CLEAR_MODE    13    /* SCR+0x4a value that drives the event-block clear */

/**
 * \brief           Throttle/brake gesture + command detect.
 * \note            Each report call passes the frame tuple (a, b, payload) as the 5th, 6th
 *                      and 7th arguments of vcu_usart_report_send.
 */
void vcu_faultmon_gesture_detect(void) {
    uint32_t adc_a;
    uint32_t adc_b;
    int16_t state;

    /* Only run un-inhibited (IN1+0x04), not drive-inhibited (VS+0x41), slot idle (SCR+0x40). */
    if ((g_in1.drive_active != 0) || (g_vs.passcode_set_pending != 0) || (g_scr.gest_busy != 0)) {
        return;
    }

    /* Both channels below their scaled thresholds == released; else count the hold up. */
    adc_a = control_adc_raw(VCU_ADC_SLOT_BRAKE_FULL); /* PA4 full-weight brake lever */
    adc_b = control_adc_raw(VCU_ADC_SLOT_BRAKE_HALF); /* PA1 half-weight brake lever */
    if ((((adc_a & GEST_ADC_MASK) >> GEST_ADC_SHIFT) < GEST_ADC_THRESH_A)
        && (((adc_b & GEST_ADC_MASK) >> GEST_ADC_SHIFT) < GEST_ADC_THRESH_B)) {
        g_fm_gest.confirm = 0;
    } else {
        if (g_fm_gest.confirm < GEST_CONFIRM_CLAMP) {
            g_fm_gest.confirm = (uint8_t)(g_fm_gest.confirm + 1);
        }
        if (g_fm_gest.confirm == GEST_CONFIRM_TRIP) {
            g_fm_gest.dwell = GEST_WINDOW_LONG;
            FM_GEST_INDEX = (uint16_t)(FM_GEST_INDEX + 1);
            if (FM_GEST_INDEX > GEST_INDEX_WRAP) {
                FM_GEST_INDEX = 0;
            }
        }
    }

    /* A pending index runs its dwell window down; on CAN-input ready, dispatch it. */
    if (FM_GEST_INDEX != 0) {
        if (g_fm_gest.dwell < 1) {
            FM_GEST_INDEX = 0;
        } else {
            g_fm_gest.dwell = (int16_t)(g_fm_gest.dwell - 1);
        }
        if (vcu_gpio_pa10_active() != 0) {
            g_fm_gest.dwell = 0;
            state = (int16_t)FM_GEST_INDEX;
            if (state == FM_GEST_PRESET) {
                buzzer_beep(BEEP_SHORT);
                g_scr.gest_payload = 1;
                vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, VCU_RPT_DST_U16, 2, NB_CMD_WRITE3,
                                      0x3A, ((const volatile void*)&g_scr.gest_payload));
            } else if (state == FM_GEST_ENTER) {
                g_fm_gest.dwell = GEST_WINDOW_SHORT;
                buzzer_beep(BEEP_DOUBLE);
                nb_commit_mode(GEST_MODE_ENTER);
            } else if (state == FM_GEST_CONFIRM) {
                buzzer_beep(BEEP_DOUBLE);
                vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 0xe, NB_CMD_WRITE3, 0,
                                      FM_IN2_FRAME);
                nb_commit_mode(GEST_MODE_CONFIRM);
                g_scr.gest_payload = 1;
                vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, VCU_RPT_DST_U16, 2, NB_CMD_WRITE3,
                                      0x3B, ((const volatile void*)&g_scr.gest_payload));
            } else if (state == FM_GEST_RELEASE) {
                buzzer_beep(BEEP_SHORT);
                g_scr.gest_payload = 2;
                vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, VCU_RPT_DST_U16, 2, 1, 0x20,
                                      ((const volatile void*)&g_scr.gest_payload));
            }
            FM_GEST_INDEX = 0;
        }
    }

    /* Finish / clear stage, gated on the currently-committed mode (SCR+0x4a). */
    if (g_scr.ui_screen == GEST_FINISH_MODE) {
        if (g_fm_gest.dwell < 1) {
            g_scr.gest_payload = 0;
            vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, VCU_RPT_DST_U16, 2, NB_CMD_WRITE3, 0x1F,
                                  ((const volatile void*)&g_scr.gest_payload));
            nb_commit_mode(GEST_MODE_FINISH);
            return;
        }
        g_fm_gest.dwell = (int16_t)(g_fm_gest.dwell - 1);
    } else if (g_vs.chg_disp_refresh != 0) {
        g_vs.chg_disp_refresh = 0;
        if (g_scr.ui_screen == GEST_CLEAR_MODE) {
            nb_commit_mode(GEST_MODE_FINISH);
            vehicle_event_block_clear();
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public module API.                                                         */
/* ------------------------------------------------------------------------- */

void vcu_faultmon_init(void) {
    uint32_t i;

    /* Node watchdogs 1-6 and sensor FSMs 2/3 (records 1..8): full record zeroed. */
    for (i = FM_MON_SUBSTATE_NEXT; i <= FM_MON_LIGHT_HI; ++i) {
        volatile fm_mon_t* w = MON(i);
        w->shift = 0;
        w->armed = 0;
        w->hold = 0;
        w->code = 0;
        w->phase = 0;
    }

    /* Sensor FSM 1 has no shift register (armed/counter/latch only). */
    FSM_SENSOR1->armed = 0;
    FSM_SENSOR1->hold = 0;
    FSM_SENSOR1->code = 0;

    FM_RETRY = 0;
}

/* Public per-node watchdog + sensor-FSM entry points. passcode.c drives these from its
   passcode unlock ticks; each forwards to the same fm_watchdog / fm_monitor_two_stage /
   sensor FSM that vcu_faultmon_tick uses. */
void fm_comms_wd_1(void) { fm_watchdog(WD_NODE1, fm_pins_all_low(GPIOA, GPIO_PINS_11), FM_CODE_WARN); }

void fm_comms_wd_2(void) { fm_watchdog(WD_NODE2, fm_pins_all_low(GPIOB, GPIO_PINS_7 | GPIO_PINS_6), FM_CODE_WARN); }

void fm_comms_wd_3(void) { fm_watchdog(WD_NODE3, fm_pins_all_low(GPIOA, GPIO_PINS_12), FM_CODE_LATCH); }

void fm_comms_wd_4(void) { fm_watchdog(WD_NODE4, fm_pins_all_low(GPIOB, GPIO_PINS_11), FM_CODE_WARN); }

void fm_comms_wd_5(void) { fm_watchdog(WD_NODE5, fm_pins_all_low(GPIOB, GPIO_PINS_2), FM_CODE_WARN); }

void fm_comms_wd_6(void) { fm_monitor_two_stage(WD_NODE6, fm_pins_all_low(GPIOB, GPIO_PINS_1)); }

void fm_sensor_fsm_2(void) {
    fm_monitor_two_stage(FSM_SENSOR2, fm_pins_all_high(GPIOB, GPIO_PINS_7) && fm_pins_all_low(GPIOB, GPIO_PINS_6));
}

void fm_sensor_fsm_3(void) {
    fm_monitor_two_stage(FSM_SENSOR3, fm_pins_all_low(GPIOB, GPIO_PINS_7) && fm_pins_all_high(GPIOB, GPIO_PINS_6));
}

/* The nine input monitors and the PA10 power-button monitor, in a fixed call order. */
void vcu_faultmon_tick(void) {
    fm_comms_wd_1();
    fm_comms_wd_2();
    fm_comms_wd_3();
    fm_sensor_fsm_1();
    fm_sensor_fsm_3();
    fm_sensor_fsm_2();
    fm_comms_wd_4();
    fm_comms_wd_5();
    fm_comms_wd_6();
    vcu_faultmon_can_fault_escalate();
}
