/**
 * \file            vehicle.h
 * \brief           Vehicle finite-state machine (x3 VCU) -- public API
 * \note            The slot-0 scheduler task and the FSM-coupled
 *                      latch / event-prep leaves other units call directly (ridemode.c,
 *                      control.c, faultmon.c). The FSM state byte lives in g_state_block
 *                      (vcu_state.h, values = vcu_fsm_state_t).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef VEHICLE_H
#define VEHICLE_H

#include "at32f415.h"

/* ------------------------------------------------------------------------- */
/* Shared mode bit mask + timing the FSM shares with the report and odometer   */
/* units (the IN2 register bits SF_* / CFG_* live in vcu_state.h).             */
/* ------------------------------------------------------------------------- */
#define MF_ACTIVE         0x0004u /*!< MODE->mode_flags (+0x0A) bit2 ride active            */
#define MF_LIGHT_EN       0x0200u /*!< MODE->mode_flags (+0x0A) bit9 light enable            */
#define STANDBY_OFF_TICKS 300     /*!< idle -> power-down count (also the mileage down-counter reload) */

/**
 * \brief           Vehicle finite-state machine / state-update task
 * \note            Slot-0 scheduler task, period 10 ticks. Latches state changes and clears
 *                      the vehicle-state struct, runs the shared sensor/CAN pre-pass, then
 *                      either steps the 9-state FSM (state byte in g_state_block) or, in
 *                      input-test mode (VS `+0x64` set), emits a Ninebot event-report frame
 *                      per pending input event.
 * \note            Nine per-state run handlers in vehicle.c (0 idle, 1 power-on, 2 locked,
 *                      3 ride, 4 power-down, 5 sleep, 6 alarm, 7 charging, 8 charge standby);
 *                      each drives control.c and commits via nb_commit_mode and the report.c writers.
 */
void vehicle_task_fsm(void);

/** \brief Fault-detection aggregate tick: runs vcu_faultmon_tick(). */
void vehicle_fault_aggregate(void);

/** \brief Event-1 / sub-mode-0 output prep. */
void vehicle_event1_prep(void);

/** \brief Event-7 lock / anti-theft GPIO latch. */
void vehicle_lock_antitheft_latch(void);

/** \brief Event-8 output prep. */
void vehicle_event8_prep(void);

/** \brief Event-9 / subsystem-A enable + mask-A report. */
void vehicle_event9_prep(void);

/** \brief Event-10 / subsystem-B enable + mask-B report. */
void vehicle_event10_prep(void);

/** \brief Zero the 0x90-byte event-flag block g_evt. */
void vehicle_event_block_clear(void);

#endif /* VEHICLE_H */
