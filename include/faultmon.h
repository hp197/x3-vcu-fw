/**
 * \file            faultmon.h
 * \brief           Fault monitor: sensor-fault debounce FSMs, per-node comms-loss
 *                      watchdogs, fault-code display builders and the periodic
 *                      status/telemetry builder + retry/reset trigger (x3 VCU)
 * \note            Entry points of this module:
 *                        fm_sensor_fsm_1  (sensor debounce, +0x70/+0x74)
 *                        fm_sensor_fsm_2  (sensor debounce, +0x60/+0x69)
 *                        fm_sensor_fsm_3  (sensor debounce, +0x54/+0x5d)
 *                        fm_comms_wd_1    (node watchdog, +0x18/+0x20)
 *                        fm_comms_wd_2    (node watchdog, +0x0C/+0x14)
 *                        fm_comms_wd_3    (node watchdog, +0x24/+0x2C)
 *                        fm_comms_wd_4    (node watchdog, +0x3C/+0x44)
 *                        fm_comms_wd_5    (node watchdog, +0x30/+0x38)
 *                        fm_comms_wd_6    (node watchdog, +0x48/+0x50 + 100-tick timer)
 *                        vcu_faultmon_interlock_tick     (safety-interlock FSM)
 *                        vcu_faultmon_fault_latch        (drive-enable fault latch + arm-timeout)
 *                        vcu_faultmon_gesture_detect     (throttle/brake gesture + command detect)
 *                        vcu_faultmon_can_fault_escalate (CAN err debounce/escalate)
 *                        vcu_faultmon_retry_trigger      (retry ctr -> system reset)
 *                        fm_status_report                (periodic DTC/telemetry builder)
 *
 *                      STATE: the monitors read and write the shared per-block globals of
 *                      vcu_state.h (g_evt, g_vs, g_in1, g_in2, g_scr, ...). The hw-input
 *                      reads are BSP GPIO calls; the cross-module callees (DTC/status,
 *                      mode commit, beeps, CAN re-init, report framer) go through their
 *                      owning modules' headers.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef FAULTMON_H
#define FAULTMON_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* The monitors keep their debounce state in the twelve fm_mon_t records of the  */
/* shared event block (g_evt, vcu_state.h: fm_mon_t / fm_mon_id_t).             */
/* The `code` byte of each record is what the other modules test and clear; the  */
/* shift / armed / hold / phase cells are written only here.                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* Module lifecycle.                                                         */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Reset the fault-monitor blackboard state to power-on zero
 * \note            The C runtime already zero-fills .bss; this re-zeros only the fields
 *                      these monitors own (the debounce / watchdog shift-register, enable,
 *                      hold-counter and latch bytes in the event blackboard, plus the retry
 *                      counter) so the module starts from a known state.
 */
void vcu_faultmon_init(void);

/**
 * \brief           Fault-detection aggregate: run every input monitor once
 * \note            Order: node watchdogs 1, 2, 3, sensor FSMs 1, 3, 2, node watchdogs 4,
 *                      5, 6 and the PA10 power-button monitor
 *                      (\ref vcu_faultmon_can_fault_escalate).
 */
void vcu_faultmon_tick(void);

/**
 * \brief           Periodic DTC / status / telemetry builder
 * \note            Called from the FSM task sites (report.c);
 *                      \ref vcu_faultmon_retry_trigger runs separately (vehicle.c).
 */
void fm_status_report(void);

/* ------------------------------------------------------------------------- */
/* Fault / event state machines.                                             */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Safety-interlock state machine (idle / armed / engaged)
 * \note            While the entry gate is open (IN2+0x38 bit11, vehicle idle, speed within
 *                      half the MODE threshold with the arm-permit input, or the MODE-active
 *                      bit set) it runs a 3-state machine (state block g_tmr): idle
 *                      waits for a request and arms; armed waits up to 300 ticks for bus
 *                      confirmation; engaged watches the command against the captured
 *                      reference and aborts on mismatch. Raises fault code 8 via the
 *                      mode/lighting reporter and drives the buzzer; publishes the active
 *                      flag at SCR+0x52.
 */
void vcu_faultmon_interlock_tick(void);

/**
 * \brief           Drive-enable fault latch + arm-timeout state machine
 * \note            Routed by a command code. While the drive-enable gate (IN2+0x38 bit11) is
 *                      open it resets the latch block (g_tmr +0x06 one-shot, +0x16 arm
 *                      timer), marks the handled flag (IN1+0x27) and clears control event
 *                      0x0F. While the gate is closed it, on the arm command (cmd 1) edge,
 *                      enqueues the fault frame (src 0x16 dst 0x23 len 0x0e) once and commits
 *                      mode 0x0B; it runs a 300-tick arm timer while a brake lever is active (IN1+0x0B) and
 *                      on timeout marks VS+0x01, latches handled and commits mode 1; finally
 *                      it routes the event -- raised (set) on the report command (cmd 3),
 *                      cleared otherwise.
 * \param[in]       cmd: latch command / mode code (compared against 1 and 3)
 */
void vcu_faultmon_fault_latch(uint32_t cmd);

/**
 * \brief           Throttle/brake gesture + command detection
 * \note            Gated on IN1+0x04, VS+0x41 and the SCR+0x40 busy word all clear. Reads the
 *                      two throttle/brake ADC leaves, scales
 *                      each by (&0xFFF)>>4 and, while both stay held, advances a confirm
 *                      counter; on the 5th hit it reloads a dwell window and steps a 0..9
 *                      gesture index. With a pending index and the CAN input ready
 *                      (vcu_gpio_pa10_active) it dispatches per index (states 3/5/8/9): beeps, report
 *                      frames and mode commits (0x0E/0x0D). A finish/clear stage, gated on the
 *                      committed mode (SCR+0x4a == 0x0E / 0x0D), runs the window down and
 *                      emits the closing frame or clears the event block.
 */
void vcu_faultmon_gesture_detect(void);

/**
 * \brief           CAN error-code debounce + two-stage fault escalation
 * \note            Feeds a 12-bit debounce shift register (mask 0xFFF) from the CAN
 *                      error-code input; on 12 consecutive lost reads it latches code 4,
 *                      holds 100 ticks, then escalates to hard-fault code 3. A clean read
 *                      re-arms and demotes a still-latched code 4 to the active warning 1.
 *                      Monitor window @ g_evt+0x00.
 */
void vcu_faultmon_can_fault_escalate(void);

/**
 * \brief           Retry counter -> system-reset trigger
 * \note            On a retry request the retry counter is bumped; past the limit (0x13) it
 *                      is reset and the system reset fires. The recompute request is raised
 *                      either way. Called from the vehicle FSM (vehicle.c).
 */
void vcu_faultmon_retry_trigger(void);

/**
 * \brief           Brake-lever held debounce A / B
 * \note            A reads the PA4 full-weight brake lever, B the PA1 half-weight brake
 *                      lever; both scale by >>4 and compare
 *                      against 50. A released reading clears the confirm counter and the
 *                      latched window code; a held reading counts up and, on the 5th
 *                      consecutive hit, latches the window code (g_evt+0x80 PA4 / +0x8C PA1)
 *                      to 4, holding there until a released reading clears it. The charge timer (charging.c)
 *                      consumes the latched code-4 state.
 */
void vcu_faultmon_sensor_debounce_a(void);
void vcu_faultmon_sensor_debounce_b(void);

/* Per-node comms watchdogs + sensor FSMs (individual entry points; the passcode unlock
   ticks call these directly; bodies forward to the fm_watchdog / two-stage / sensor FSMs
   that vcu_faultmon_tick drives). */
void fm_comms_wd_1(void);   /* node watchdog 1 */
void fm_comms_wd_2(void);   /* node watchdog 2 */
void fm_comms_wd_3(void);   /* node watchdog 3 */
void fm_comms_wd_4(void);   /* node watchdog 4 */
void fm_comms_wd_5(void);   /* node watchdog 5 */
void fm_comms_wd_6(void);   /* node watchdog 6 */
void fm_sensor_fsm_2(void); /* sensor debounce FSM 2 */
void fm_sensor_fsm_3(void); /* sensor debounce FSM 3 */

/**
 * \brief           Shared accessor for the parked-gear backup cell
 * \note            Byte +0 of faultmon's gesture block. faultmon owns the block;
 *                      ridemode's park toggle reads/writes this byte, so it is exposed here.
 * \return          Pointer to the single gear-backup byte
 */
volatile uint8_t* vcu_faultmon_gear_backup(void);

#endif /* FAULTMON_H */
