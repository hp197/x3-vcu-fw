/**
 * \file            ridemode.c
 * \brief           Ride-mode / gear / display-state machine (x3 VCU)
 * \note            See ridemode.h for the role of this unit. These functions are callees of
 *                      the vehicle FSM dispatch tick; vehicle.c calls them directly.
 *
 *                      The SRAM blackboards are reached through the vcu_state.h shared store:
 *                      the typed overlays (VS/IN1/IN2/SCR/STAT/EVT/MODE/STATE) plus two
 *                      unit-local overlays for the cells no shared overlay names, and the
 *                      named B8/U16/S16 macros for odd offsets. Writes here are seen by every
 *                      other module. The cross-module
 *                      callees are reached through their owners' public headers (vehicle.h,
 *                      buzzer.h, report.h, control.h).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "ridemode.h"
#include "at32f415.h"  /* Artery HAL: GPIOA, GPIO_PINS_6, gpio_output_data_bit_read */
#include "control.h"   /* vcu_ride_mode_t (WALK..POWER) + control_output_light() */
#include "faultmon.h"  /* vcu_faultmon_gear_backup() (gear-backup cell, gesture block +0) */
#include "vcu_state.h" /* shared SRAM store, overlays, enums, B8/U16/S16 macros   */

/* ------------------------------------------------------------------------- */
/* Shared-blackboard state: the blocks are plain globals declared in            */
/* vcu_state.h; this unit reads/writes them directly (g_vs, g_in1, ...).          */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* Unit-local overlays for the two shared blocks no vcu_state.h overlay names.  */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Ride sub-mode dispatcher state
 * \note            Owned by ridemode alone.
 */
typedef struct {
    uint8_t cur_submode; /*!< live sub-mode (mirror of g_in1.regen_active)   */
    uint8_t hold_latch;  /*!< range-hold latch (set entering the hold leg)   */
    uint8_t led_submode; /*!< sub-mode mirror consumed by the status word    */
    uint16_t dwell;      /*!< hold / unwind down-counter                     */
} ridemode_sub_t;

/** \brief Ride sub-mode dispatcher state (ridemode-private). */
static volatile ridemode_sub_t g_ride_sub;

/**
 * \brief           Headlight / light-button toggle view of the g_chg block
 * \note            Overlays the same g_chg bytes control.c reaches through \ref vcu_chg_t --
 *                      one shared subsystem, not reused scratch. This FSM (driven by the
 *                      FM_MON_LIGHT_LO/_HI events) writes out_lo/out_hi (+0x02/+0x03); control's
 *                      press-counting FSM is the only reader, as g_chg.f2/f3, and
 *                      control_charge_state_reset() clears f2/f3 with those two event codes. So
 *                      the overlay has to stay on &g_chg. Field map: tog_lo/tog_hi/out_lo/out_hi
 *                      == vcu_chg_t f0/f1/f2/f3, clr06/clr07 == f6/f7; together a light-button
 *                      multi-press detector.
 */
typedef struct {
    uint8_t tog_lo;    /*!< +0x00 low channel toggle latch  (LIGHT_LO) */
    uint8_t tog_hi;    /*!< +0x01 high channel toggle latch (LIGHT_HI) */
    uint8_t out_lo;    /*!< +0x02 output flag A                        */
    uint8_t out_hi;    /*!< +0x03 output flag B                        */
    uint8_t alt_latch; /*!< +0x04 alternate-state latch                */
    uint8_t _rsv05;    /*!< +0x05 unused                               */
    uint8_t clr06;     /*!< +0x06 cleared on each handled event        */
    uint8_t clr07;     /*!< +0x07 cleared on each handled event        */
} ridemode_hl_t;

/**
 * \brief           LED / status bit-matrix at g_in2 + 0x36
 * \note            Target of the mode->matrix projector, addressed as 16-bit words off this
 *                      base. Only three are written: the main indicator word at +0x00 and the
 *                      two auxiliary annunciator words at +0x2A (word 0x15) and +0x2C (word
 *                      0x16). A plain SRAM blackboard cell, not a peripheral, read by the
 *                      display service.
 */
typedef struct {
    uint16_t ind_main;     /*!< +0x00 main indicator word  (matrix word 0)    */
    uint16_t _rsv02[0x14]; /*!< +0x02..+0x29 words 1..0x14 (untouched here)  */
    uint16_t ind_aux_lo;   /*!< +0x2A auxiliary word A     (matrix word 0x15) */
    uint16_t ind_aux_hi;   /*!< +0x2C auxiliary word B     (matrix word 0x16) */
} ridemode_ledmatrix_t;

#define HL                   ((volatile ridemode_hl_t*)&g_chg) /* light-toggle view of g_chg (shared, see above) */
/* Mode LED/status matrix (overlays IN2+0x36). */
#define LEDM                 ((volatile ridemode_ledmatrix_t*)(((volatile uint8_t*)&g_in2) + 0x36u))

/* ------------------------------------------------------------------------- */
/* The blackboard cells this unit uses are reached through their shared           */
/* vcu_state.h overlay fields: VS->gear_sel @0x01 (stored WALK..POWER),            */
/* VS->substate @0x05, VS->remote_lock_edge @0x23, VS->distance_code @0x0E,        */
/* SCR->submode_code @0x42, MODE->enable_flags @0x10, STAT->gear_report @0x1F4,    */
/* and LOWGLOB->active_error_code. The only cell with a local macro is the         */
/* parked-gear backup below (a lone LOW cell that no shared overlay names).        */
/* ------------------------------------------------------------------------- */
/* Parked-gear backup for the park toggle: byte +0 of faultmon's gesture block,
   reached through faultmon's shared accessor. */
#define GEAR_BACKUP          (*vcu_faultmon_gear_backup())

/* Reads the throttle/idle line on GPIOA via the Artery HAL. The access is GPIOA+0x0C, the
 * output data register (odt), not idt (0x08) -- hence gpio_output_data_bit_read(), since
 * gpio_input_data_bit_read() would read the wrong register. The test checks bit 6 == RESET. */

/*
 * Gear-event counter: telemetry-frame field +0x18 of report.c's
 * g_telem_frame (the builder copies it to frame[0x2F]).
 */
#define GEAR_EVENT_COUNTER   (g_telem_frame[REPORT_TELEM_GEAR_EVENT])

/* ------------------------------------------------------------------------- */
/* Magic-number constants.                                                   */
/* ------------------------------------------------------------------------- */

/* LED/status matrix segment bits. Each op is a single-bit set (|mask) or clear
 * (&~mask) read-modify-write; the raw AND masks (0xffef, 0xffbf, ...) are the one's
 * complement of these. */
#define LEDM_B0              (1u << 0)  /* 0x0001 */
#define LEDM_B1              (1u << 1)  /* 0x0002 */
#define LEDM_B2              (1u << 2)  /* 0x0004 */
#define LEDM_B3              (1u << 3)  /* 0x0008 */
#define LEDM_B4              (1u << 4)  /* 0x0010 */
#define LEDM_B5              (1u << 5)  /* 0x0020 */
#define LEDM_B6              (1u << 6)  /* 0x0040 */
#define LEDM_B7              (1u << 7)  /* 0x0080 */
#define LEDM_B8              (1u << 8)  /* 0x0100 */
#define LEDM_B9              (1u << 9)  /* 0x0200 */
#define LEDM_B10             (1u << 10) /* 0x0400 */
#define LEDM_B11             (1u << 11) /* 0x0800 */
#define LEDM_B12             (1u << 12) /* 0x1000 */
#define LEDM_B13             (1u << 13) /* 0x2000 */
#define LEDM_B14             (1u << 14) /* 0x4000 */
#define LEDM_B15             (1u << 15) /* 0x8000 */

/* Single-bit RMW on a matrix word: one volatile load + one volatile store. */
#define LEDM_SET(field, bit) ((field) = (uint16_t)((field) | (uint16_t)(bit)))
#define LEDM_CLR(field, bit) ((field) = (uint16_t)((field) & (uint16_t)~(uint16_t)(bit)))

#define SUBMODE_DWELL_RELOAD 300u  /* hold / unwind dwell reload (0x012C)           */
#define HOLD_SPEED_MAX_KMH   41u   /* speed_kmh ceiling inside the hold leg          */
#define DIST_CODE_LOW        120u  /* range-code low threshold                       */
#define DIST_CODE_HIGH       200u  /* range-code high threshold (200)                */
#define GEAR_CHANGE_MAX_KMH  5u    /* speed_kmh ceiling for a gear change            */
#define GEAR_RPT_VALUE       100u  /* value stashed in STAT->gear_report             */
#define SECLOCK_RPT_IDX      0x61u /* report_u16 index for the secondary lock        */
#define SECLOCK_RPT_VAL      0x20u /* report_u16 value for the secondary lock        */

/* ------------------------------------------------------------------------- */
/* Cross-module callees via their canonical public owners.                     */
/*                                                                            */
/* vehicle.h owns the FSM/event leaves (vehicle_fault_aggregate,              */
/* vehicle_event1_prep, vehicle_lock_antitheft_latch, vehicle_event8_prep),   */
/* buzzer.h the beep/tone leaves (buzzer_beep, buzzer_off), report.h the      */
/* report writers (report_u16, report_ninebot); control.h owns                */
/* control_mode_report_fsm and control_output_light.                          */
/* ------------------------------------------------------------------------- */
#include "buzzer.h"
#include "report.h"
#include "usart.h" /* NB_ADDR_VCU / NB_CMD_WRITE3 (shared Ninebot header vocabulary) */
#include "vehicle.h"

/* ------------------------------------------------------------------------- */
/* Rear-light mode (MODE->enable_flags bits 9..11).                          */
/* ------------------------------------------------------------------------- */
void ridemode_status_word_update(void) {
    /* Leaving a regen indication with regen cleared: rear light off. */
    if ((g_ride_sub.led_submode == 1) && (g_in1.regen_active == 0)) {
        mode_light_set(MODE_LIGHT_OFF);
    }
    g_ride_sub.led_submode = g_in1.regen_active; /* mirror the live regen/neutral flag */

    if (g_in1.charger_connected == 0) {
        if (g_lowglob.active_error_code == 0) {
            if (g_in1.regen_active == 1) {
                if (g_in1.brake_any == 0) {
                    mode_light_set(MODE_LIGHT_TAIL);
                    return;
                }
                if (g_in2.f.brake_light_flash != 0) {
                    mode_light_set(MODE_LIGHT_BRAKE_FLASH);
                    return;
                }
                mode_light_set(MODE_LIGHT_BRAKE);
                return;
            }
            if (g_in1.brake_any != 0) {
                if (g_in2.f.brake_light_flash != 0) {
                    mode_light_set(MODE_LIGHT_BRAKE_FLASH);
                    return;
                }
                mode_light_set(MODE_LIGHT_BRAKE);
                return;
            }
            /* No ride demand: clear only when both indicator output latches are off
               (PB13 / PA6 odt read-back; GPIOA is read only when PB13 is clear). */
            if ((gpio_output_data_bit_read(GPIOB, GPIO_PINS_13) == RESET)
                && (gpio_output_data_bit_read(GPIOA, GPIO_PINS_6) == RESET)) {
                mode_light_set(MODE_LIGHT_OFF);
                return;
            }
            mode_light_set(MODE_LIGHT_TAIL);
            return;
        }
    } else {
        /* Charging / charge-display branch, gated on IN2->chg_cfg bit1. */
        if ((int32_t)((uint32_t)g_in2.f.chg_cfg << 0x1E) < 0) {
            /* steady once the charge is complete (VS chg_full) */
            if (g_vs.chg_full != 0u) {
                mode_light_set(MODE_LIGHT_BRAKE);
                return;
            }
            mode_light_set(MODE_LIGHT_BLINK);
            return;
        }
        mode_light_set(MODE_LIGHT_OFF);
    }
}

/* ------------------------------------------------------------------------- */
/* Ride sub-mode dispatcher.                                                 */
/* ------------------------------------------------------------------------- */
void ridemode_submode_dispatch(void) {
    /* Leaving the sub-mode on a clean regen release while no drive fault is latched. */
    if ((g_ride_sub.cur_submode == 1) && (g_in1.regen_active == 0) && (g_vs.substate == 0)) {
        control_output_light();
        g_ride_sub.dwell = 0;
        g_scr.submode_code = 0;
    }

    g_ride_sub.cur_submode = g_in1.regen_active; /* latch the live regen/neutral flag */

    if (g_in1.regen_active == 1) {
        vehicle_lock_antitheft_latch();
        g_scr.submode_code = 1;
        return;
    }

    switch (g_vs.substate) {
        case 0:
            vehicle_event1_prep();
            g_scr.submode_code = 0;
            break;
        case 1:
            vehicle_lock_antitheft_latch();
            g_scr.submode_code = 1;
            break;
        case 2:
            vehicle_event8_prep();
            g_scr.submode_code = 2;
            break;
        case 3:
            if ((g_in2.f.chg_cfg & 1u) == 0) {
                g_vs.substate = 0;
            } else if (g_vs.distance_code < DIST_CODE_LOW) {
                g_ride_sub.hold_latch = 1;
                if (g_vs.speed_kmh < HOLD_SPEED_MAX_KMH) {
                    if (g_ride_sub.dwell == 0) {
                        vehicle_lock_antitheft_latch();
                        g_scr.submode_code = 3;
                    } else {
                        g_ride_sub.dwell = (uint16_t)(g_ride_sub.dwell - 1);
                    }
                } else {
                    g_ride_sub.dwell = SUBMODE_DWELL_RELOAD;
                    vehicle_event8_prep();
                    g_scr.submode_code = 4;
                }
            } else if ((g_vs.distance_code > DIST_CODE_HIGH) || (g_scr.submode_code == 0)
                       || (g_scr.submode_code == 2)) {
                if ((g_scr.submode_code == 0) || (g_scr.submode_code == 2)) {
                    g_ride_sub.dwell = SUBMODE_DWELL_RELOAD;
                    g_scr.submode_code = 5;
                } else if (g_ride_sub.hold_latch != 0) {
                    g_ride_sub.hold_latch = 0;
                    g_ride_sub.dwell = SUBMODE_DWELL_RELOAD;
                }
                if (g_ride_sub.dwell == 0) {
                    vehicle_event1_prep();
                    g_scr.submode_code = 6;
                } else {
                    g_ride_sub.dwell = (uint16_t)(g_ride_sub.dwell - 1);
                }
            } else {
                /* in-range, nothing to do this tick */
            }
            break;
        default: break;
    }
}

/* ------------------------------------------------------------------------- */
/* Gear / mode change event state machine.                                  */
/* ------------------------------------------------------------------------- */

/* Emit the gear-change report frame shared by both gear event blocks. */
static void ridemode_gear_report(void) {
    g_stat.gear_report = GEAR_RPT_VALUE;
    report_ninebot(NB_ADDR_VCU, 7, 2, NB_CMD_WRITE3, 0xFA, (const void*)&g_stat.gear_report);
}

void ridemode_gear_event_tick(void) {
    uint8_t gear;

    vehicle_fault_aggregate(); /* fault-monitor aggregate */

    /* Secondary-lock report gate (only in the reportable FSM states 3/7). */
    if ((g_evt.mon[FM_MON_POWER_BTN].code == 1)
        && ((g_state_block.fsm_state == VCU_FSM_RIDE) || (g_state_block.fsm_state == VCU_FSM_CHARGING))) {
        report_u16(SECLOCK_RPT_IDX, SECLOCK_RPT_VAL);
        if (g_vs.remote_lock_edge != 0) {
            g_vs.remote_lock_edge = 0;
            g_evt.mon[FM_MON_POWER_BTN].code = 0;
        }
    }

    /* Park / neutral toggle gesture (FM_MON_PARK code 3): the event is always cleared, the
     * gear toggle runs only while the guard is clear. */
    if ((g_evt.mon[FM_MON_PARK].code != 1) && (g_evt.mon[FM_MON_PARK].code == 3)) {
        g_evt.mon[FM_MON_PARK].code = 0;
        if (g_vs.speed_kmh < GEAR_CHANGE_MAX_KMH) {
            if (g_vs.gear_sel == VCU_MODE_WALK) {
                g_vs.gear_sel = GEAR_BACKUP; /* restore the parked gear */
            } else {
                GEAR_BACKUP = g_vs.gear_sel; /* save, then drop to neutral/park */
                g_vs.gear_sel = VCU_MODE_WALK;
            }
            buzzer_beep(BEEP_DOUBLE);
        }
    }

    /* Gear-up event block (FM_MON_GEAR_UP). */
    if (g_evt.mon[FM_MON_GEAR_UP].code == 1) {
        GEAR_EVENT_COUNTER = (uint16_t)(GEAR_EVENT_COUNTER + 1);
        g_evt.mon[FM_MON_GEAR_UP].code = 0;
        if ((int32_t)((uint32_t)g_in2.f.status_flags << 0x14) < 0) { /* IN2+0x38 bit11: drive output enable */
            buzzer_beep(BEEP_DOUBLE);
            gear = g_vs.gear_sel;
            if (gear == VCU_MODE_WALK) {
                g_vs.gear_sel = VCU_MODE_ECO;
            } else if (gear == VCU_MODE_ECO) {
                /* bit8 -> 3 first, then bit9 -> 4 */
                if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x17) < 0) { /* +0x3C bit8 */
                    g_vs.gear_sel = VCU_MODE_SPORT;
                } else if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x16) < 0) { /* +0x3C bit9 */
                    g_vs.gear_sel = VCU_MODE_POWER;
                } else if (((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x1B) < 0) /* +0x3A bit4 */
                           && (g_vs.speed_kmh < GEAR_CHANGE_MAX_KMH)) {
                    GEAR_BACKUP = g_vs.gear_sel;
                    g_vs.gear_sel = VCU_MODE_WALK;
                }
            } else if (gear == VCU_MODE_SPORT) {
                /* bit9 -> 4 */
                if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x16) < 0) { /* +0x3C bit9 */
                    g_vs.gear_sel = VCU_MODE_POWER;
                } else if (((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x1B) < 0) /* +0x3A bit4 */
                           && (g_vs.speed_kmh < GEAR_CHANGE_MAX_KMH)) {
                    GEAR_BACKUP = g_vs.gear_sel;
                    g_vs.gear_sel = VCU_MODE_WALK;
                } else {
                    g_vs.gear_sel = VCU_MODE_ECO;
                }
            } else if (gear == VCU_MODE_POWER) {
                if (((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x1B) < 0) /* +0x3A bit4 */
                    && (g_vs.speed_kmh < GEAR_CHANGE_MAX_KMH)) {
                    GEAR_BACKUP = g_vs.gear_sel;
                    g_vs.gear_sel = VCU_MODE_WALK;
                } else {
                    g_vs.gear_sel = VCU_MODE_ECO;
                }
            } else {
                /* unknown gear value: leave as-is */
            }
        } else {
            control_mode_report_fsm(8);
        }
        ridemode_gear_report();
    }

    /* Gear-down event block (FM_MON_GEAR_DOWN). */
    if (g_evt.mon[FM_MON_GEAR_DOWN].code == 1) {
        GEAR_EVENT_COUNTER = (uint16_t)(GEAR_EVENT_COUNTER + 1);
        g_evt.mon[FM_MON_GEAR_DOWN].code = 0;
        if ((int32_t)((uint32_t)g_in2.f.status_flags << 0x14) < 0) { /* IN2+0x38 bit11: drive output enable */
            buzzer_beep(BEEP_DOUBLE);
            gear = g_vs.gear_sel;
            if (gear == VCU_MODE_WALK) {
                if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x16) < 0) { /* +0x3C bit9 */
                    g_vs.gear_sel = VCU_MODE_POWER;
                } else if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x17) < 0) { /* +0x3C bit8 */
                    g_vs.gear_sel = VCU_MODE_SPORT;
                } else {
                    g_vs.gear_sel = VCU_MODE_ECO;
                }
            } else if (gear == VCU_MODE_ECO) {
                if (((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x1B) < 0) /* +0x3A bit4 */
                    && (g_vs.speed_kmh < GEAR_CHANGE_MAX_KMH)) {
                    GEAR_BACKUP = g_vs.gear_sel;
                    g_vs.gear_sel = VCU_MODE_WALK;
                } else if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x16) < 0) { /* +0x3C bit9 */
                    g_vs.gear_sel = VCU_MODE_POWER;
                } else if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x17) < 0) { /* +0x3C bit8 */
                    g_vs.gear_sel = VCU_MODE_SPORT;
                }
            } else if (gear == VCU_MODE_SPORT) {
                g_vs.gear_sel = VCU_MODE_ECO;
            } else if (gear == VCU_MODE_POWER) {
                if ((int32_t)((uint32_t)g_in2.f.cfg_flags2 << 0x17) < 0) { /* +0x3C bit8 */
                    g_vs.gear_sel = VCU_MODE_SPORT;
                } else {
                    g_vs.gear_sel = VCU_MODE_ECO;
                }
            } else {
                /* unknown gear value: leave as-is */
            }
        } else {
            control_mode_report_fsm(8);
        }
        ridemode_gear_report();
    }

    /* Sub-state step-down (FM_MON_SUBSTATE_PREV). */
    switch (g_evt.mon[FM_MON_SUBSTATE_PREV].code) {
        case 1:
            g_evt.mon[FM_MON_SUBSTATE_PREV].code = 0;
            if (g_vs.substate == 0) {
                if ((g_in2.f.chg_cfg & 1u) == 0) {
                    g_vs.substate = 2;
                } else {
                    g_vs.substate = 3;
                }
            } else {
                g_vs.substate = (uint8_t)(g_vs.substate - 1);
            }
            break;
        case 2: g_evt.mon[FM_MON_SUBSTATE_PREV].code = 0; break;
        case 3: g_evt.mon[FM_MON_SUBSTATE_PREV].code = 0; break;
        default: break;
    }

    /* Sub-state cycle (FM_MON_SUBSTATE_NEXT). */
    if (g_evt.mon[FM_MON_SUBSTATE_NEXT].code == 1) {
        g_evt.mon[FM_MON_SUBSTATE_NEXT].code = 0;
        g_vs.substate = (uint8_t)(g_vs.substate + 1);
        if (g_vs.substate > 2) {
            if ((g_in2.f.chg_cfg & 1u) == 0) {
                g_vs.substate = 0;
            } else if (g_vs.substate > 3) {
                g_vs.substate = 0;
            }
        }
    }

    /* Light-enable toggle (FM_MON_LIGHT_EN) -> MODE->mode_flags bit9. */
    if (g_evt.mon[FM_MON_LIGHT_EN].code == 1) {
        g_evt.mon[FM_MON_LIGHT_EN].code = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags | MF_LIGHT_EN);
    } else if (g_evt.mon[FM_MON_LIGHT_EN].code == 5) {
        g_evt.mon[FM_MON_LIGHT_EN].code = 0;
        g_mode.mode_flags = (uint16_t)(g_mode.mode_flags & (uint16_t)~MF_LIGHT_EN);
    }

    /* High-channel headlight toggle (FM_MON_LIGHT_HI). */
    switch (g_evt.mon[FM_MON_LIGHT_HI].code) {
        case 1:
            HL->clr06 = 0;
            HL->clr07 = 0;
            if (HL->tog_hi == 0) {
                HL->out_lo = 0;
                HL->out_hi = 1;
                HL->tog_hi = 1;
            } else if (HL->tog_hi == 1) {
                buzzer_off();
                HL->tog_hi = 0;
                if (HL->alt_latch == 0) {
                    HL->out_lo = 0;
                    HL->out_hi = 0;
                } else {
                    HL->out_lo = 1;
                    HL->out_hi = 1;
                }
            }
            g_evt.mon[FM_MON_LIGHT_HI].code = 0;
            break;
        case 2: g_evt.mon[FM_MON_LIGHT_HI].code = 0; break;
        case 3:
            HL->clr06 = 0;
            HL->clr07 = 0;
            if (HL->alt_latch == 0) {
                HL->out_lo = 1;
                HL->out_hi = 1;
                HL->tog_hi = 0;
                HL->tog_lo = 0;
                HL->alt_latch = 1;
            } else {
                HL->out_lo = 0;
                HL->out_hi = 0;
                buzzer_off();
                HL->alt_latch = 0;
            }
            g_evt.mon[FM_MON_LIGHT_HI].code = 0;
            break;
        default: break;
    }

    /* Low-channel headlight toggle (FM_MON_LIGHT_LO). */
    switch (g_evt.mon[FM_MON_LIGHT_LO].code) {
        case 1:
            HL->clr06 = 0;
            HL->clr07 = 0;
            if (HL->tog_lo == 0) {
                HL->out_lo = 1;
                HL->out_hi = 0;
                HL->tog_lo = 1;
            } else if (HL->tog_lo == 1) {
                buzzer_off();
                HL->tog_lo = 0;
                if (HL->alt_latch == 0) {
                    HL->out_lo = 0;
                    HL->out_hi = 0;
                } else {
                    HL->out_lo = 1;
                    HL->out_hi = 1;
                }
            }
            g_evt.mon[FM_MON_LIGHT_LO].code = 0;
            break;
        case 2: g_evt.mon[FM_MON_LIGHT_LO].code = 0; break;
        case 3:
            HL->clr06 = 0;
            HL->clr07 = 0;
            if (HL->alt_latch == 0) {
                HL->out_lo = 1;
                HL->out_hi = 1;
                HL->tog_hi = 0;
                HL->tog_lo = 0;
                HL->alt_latch = 1;
            } else {
                HL->out_lo = 0;
                HL->out_hi = 0;
                buzzer_off();
                HL->alt_latch = 0;
            }
            g_evt.mon[FM_MON_LIGHT_LO].code = 0;
            break;
        default: break;
    }
}

/* ------------------------------------------------------------------------- */
/* Ride-mode -> LED / status bit matrix projector. Each line is one single-bit */
/* RMW; the per-region detail is on ridemode_led_matrix_update below.          */
/* ------------------------------------------------------------------------- */
/**
 * \brief           Tail shared by rows 1 and 2
 * \param[in,out]   m: the LED / status matrix
 * \note            Word 0x16 &0xfffe &0xfffb, then the assist capabilities:
 *                      reg 0x32 &0xfffb, reg 0x2F |0x0004, reg 0x33 |1 |4 |8.
 */
static void ridemode_ledm_assist_rows_1_2(volatile ridemode_ledmatrix_t* m) {
    LEDM_CLR(m->ind_aux_hi, LEDM_B0);                /* &0xfffe */
    LEDM_CLR(m->ind_aux_hi, LEDM_B2);                /* &0xfffb */
    LEDM_CLR(g_in2.f.ride_caps_32, LEDM_B2);         /* reg 0x32 &0xfffb */
    LEDM_SET(g_in2.f.in2_5e, REG2F_TURN_CANCEL_OFF); /* reg 0x2F |0x0004 */
    LEDM_SET(g_in2.f.ride_caps_33, LEDM_B0);         /* reg 0x33 |0x0001 */
    LEDM_SET(g_in2.f.ride_caps_33, LEDM_B2);         /* reg 0x33 |0x0004 */
    LEDM_SET(g_in2.f.ride_caps_33, LEDM_B3);         /* reg 0x33 |0x0008 */
}

/**
 * \brief           Project the ride mode onto the LED / status matrix
 * \note            Also writes the assist capability bits (regs 0x32 / 0x33 / 0x2F, reg 0x1E
 *                      bits 1..4), clears word 0x16 bits 11..14, stores word 0x15 = 0x20 before
 *                      its sets and drives its bit13 per row.
 */
void ridemode_led_matrix_update(void) {
    volatile ridemode_ledmatrix_t* m = LEDM; /* matrix base (g_in2+0x36) */

    /* --- Common frame: applied for every ride mode. --- */
    LEDM_SET(m->ind_main, LEDM_B0);  /* |0x0001 */
    LEDM_SET(m->ind_main, LEDM_B1);  /* |0x0002 */
    LEDM_SET(m->ind_main, LEDM_B2);  /* |0x0004 */
    LEDM_SET(m->ind_main, LEDM_B3);  /* |0x0008 */
    LEDM_CLR(m->ind_main, LEDM_B4);  /* &0xffef */
    LEDM_CLR(m->ind_main, LEDM_B6);  /* &0xffbf */
    LEDM_CLR(m->ind_main, LEDM_B7);  /* &0xff7f */
    LEDM_CLR(m->ind_main, LEDM_B8);  /* &0xfeff */
    LEDM_CLR(m->ind_main, LEDM_B9);  /* &0xfdff */
    LEDM_CLR(m->ind_main, LEDM_B10); /* &0xfbff */
    LEDM_CLR(m->ind_main, LEDM_B11); /* &0xf7ff */
    LEDM_CLR(m->ind_main, LEDM_B12); /* &0xefff */
    LEDM_SET(m->ind_main, LEDM_B14); /* |0x4000 */
    LEDM_CLR(m->ind_main, LEDM_B15); /* &0x7fff */

    /* clear bit1, then a plain store of 0x20 */
    LEDM_CLR(m->ind_aux_lo, LEDM_B1);  /* &0xfffd */
    m->ind_aux_lo = LEDM_B5;           /* = 0x0020 */
    LEDM_SET(m->ind_aux_lo, LEDM_B6);  /* |0x0040 */
    LEDM_SET(m->ind_aux_lo, LEDM_B7);  /* |0x0080 */
    LEDM_SET(m->ind_aux_lo, LEDM_B8);  /* |0x0100 */
    LEDM_CLR(m->ind_aux_lo, LEDM_B9);  /* &0xfdff */
    LEDM_SET(m->ind_aux_lo, LEDM_B10); /* |0x0400 */
    LEDM_CLR(m->ind_aux_lo, LEDM_B11); /* &0xf7ff */
    LEDM_CLR(m->ind_aux_lo, LEDM_B14); /* &0xbfff */

    LEDM_SET(m->ind_aux_hi, LEDM_B1);  /* |0x0002 */
    LEDM_SET(m->ind_aux_hi, LEDM_B3);  /* |0x0008 */
    LEDM_SET(m->ind_aux_hi, LEDM_B4);  /* |0x0010 */
    LEDM_CLR(m->ind_aux_hi, LEDM_B6);  /* &0xffbf */
    LEDM_SET(m->ind_aux_hi, LEDM_B7);  /* |0x0080 */
    LEDM_SET(m->ind_aux_hi, LEDM_B8);  /* |0x0100 */
    LEDM_CLR(m->ind_aux_hi, LEDM_B9);  /* &0xfdff */
    LEDM_CLR(m->ind_aux_hi, LEDM_B10); /* &0xfbff */
    LEDM_CLR(m->ind_aux_hi, LEDM_B11); /* &0xf7ff */
    LEDM_CLR(m->ind_aux_hi, LEDM_B12); /* &0xefff */
    LEDM_CLR(m->ind_aux_hi, LEDM_B13); /* &0xdfff */
    LEDM_CLR(m->ind_aux_hi, LEDM_B14); /* &0xbfff */

    /* Assist capability bits, common part. */
    LEDM_CLR(g_in2.f.ride_caps_33, LEDM_B1);      /* reg 0x33 &0xfffd */
    LEDM_CLR(g_in2.f.in2_5e, REG2F_BIT0);         /* reg 0x2F &0xfffe */
    LEDM_CLR(g_in2.f.ride_caps_33, LEDM_B4);      /* reg 0x33 &0xffef */
    LEDM_CLR(g_in2.f.cfg_flags2, CFG2_ASSIST_B3); /* reg 0x1E &0xfff7 */

    /* --- Per-mode override group (live drive-row at g_vs+0x00). --- */
    switch ((vcu_region_t)g_vs.region) {
        case VCU_REGION_EUROPE:                /* row 1 */
            LEDM_CLR(m->ind_main, LEDM_B13);   /* &0xdfff */
            LEDM_CLR(m->ind_main, LEDM_B5);    /* &0xffdf */
            LEDM_SET(m->ind_aux_lo, LEDM_B12); /* |0x1000 */
            LEDM_SET(m->ind_aux_lo, LEDM_B13); /* |0x2000 */
            LEDM_CLR(m->ind_aux_lo, LEDM_B15); /* &0x7fff */
            LEDM_CLR(m->ind_aux_hi, LEDM_B5);  /* &0xffdf */
            ridemode_ledm_assist_rows_1_2(m);
            break;

        case VCU_REGION_GERMANY:               /* row 2 */
            LEDM_CLR(m->ind_main, LEDM_B13);   /* &0xdfff */
            LEDM_SET(m->ind_main, LEDM_B5);    /* |0x0020 */
            LEDM_SET(m->ind_aux_lo, LEDM_B12); /* |0x1000 */
            LEDM_SET(m->ind_aux_lo, LEDM_B13); /* |0x2000 */
            LEDM_SET(m->ind_aux_lo, LEDM_B15); /* |0x8000 */
            LEDM_SET(m->ind_aux_hi, LEDM_B5);  /* |0x0020 */
            ridemode_ledm_assist_rows_1_2(m);
            break;

        case VCU_REGION_GLOBAL: /* row 0 (also any other value) */
        default:
            LEDM_SET(m->ind_main, LEDM_B13);   /* |0x2000 */
            LEDM_CLR(m->ind_main, LEDM_B5);    /* &0xffdf */
            LEDM_CLR(m->ind_aux_lo, LEDM_B12); /* &0xefff */
            LEDM_CLR(m->ind_aux_lo, LEDM_B13); /* &0xdfff */
            LEDM_SET(m->ind_aux_lo, LEDM_B15); /* |0x8000 */
            LEDM_CLR(m->ind_aux_hi, LEDM_B5);  /* &0xffdf */
            LEDM_SET(m->ind_aux_hi, LEDM_B0);  /* |0x0001 */
            LEDM_SET(m->ind_aux_hi, LEDM_B2);  /* |0x0004 */
            /* assist capabilities of row 0 */
            LEDM_SET(g_in2.f.ride_caps_32, LEDM_B2);            /* reg 0x32 |0x0004 */
            LEDM_CLR(g_in2.f.ride_caps_33, LEDM_B0);            /* reg 0x33 &0xfffe */
            LEDM_CLR(g_in2.f.ride_caps_33, LEDM_B2);            /* reg 0x33 &0xfffb */
            LEDM_CLR(g_in2.f.ride_caps_33, LEDM_B3);            /* reg 0x33 &0xfff7 */
            LEDM_CLR(g_in2.f.cfg_flags2, CFG2_OVERSPEED_REGEN); /* reg 0x1E &0xffef */
            LEDM_CLR(g_in2.f.cfg_flags2, CFG2_RIDE_ASSIST);     /* reg 0x1E &0xfffd */
            LEDM_CLR(g_in2.f.cfg_flags2, CFG2_CLIMB_BOOST);     /* reg 0x1E &0xfffb */
            break;
    }
}

/* Rear-light mode off: clear MODE->enable_flags bits 9..11 (g_mode +0x10), the field
   ridemode_status_word_update writes. Only caller: the vehicle FSM state-4 (power-down)
   entry. */
void ridemode_led_indicator_clear(void) { mode_light_set(MODE_LIGHT_OFF); }
