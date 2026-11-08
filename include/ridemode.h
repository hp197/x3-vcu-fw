/**
 * \file            ridemode.h
 * \brief           Ride-mode / gear / display-state machine (x3 VCU) -- public API
 * \note            These three callees sit UNDER the vehicle FSM (vehicle.c): the dispatch
 *                      tick calls them to advance the ride sub-mode, to service the
 *                      gear/mode change events and to recompute the LED/indicator status
 *                      word. vehicle.c calls the public functions declared here directly
 *                      (they are callees of the FSM, not top-level super-loop tasks, so there
 *                      is no main.c wiring).
 *
 *                      The SRAM blackboards are reached through the vcu_state.h shared store
 *                      (typed overlays + named B8/U16 macros), so a write here is seen by
 *                      every other module.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef RIDEMODE_H
#define RIDEMODE_H

/**
 * \brief           Advance the ride sub-mode dispatcher
 * \note            Mirrors the live regen/neutral flag (IN1+0x26) into the sub-mode
 *                      block, then dispatches on the current sub-mode selector (VS+0x05,
 *                      states 0..3) to the indicator / anti-theft / event-prep primitives
 *                      and publishes the resulting sub-mode output code 0..6 at SCR+0x42.
 *                      State 3 runs the range-gated hold/unwind timer (SUB+0x06 dwell).
 */
void ridemode_submode_dispatch(void);

/**
 * \brief           Service the gear/mode change events for one tick
 * \note            The big ride-mode state machine. Runs the fault-monitor aggregate,
 *                      the secondary-lock report gate and the park/neutral toggle, then
 *                      the two gear-change event blocks (up / down) that walk the gear
 *                      selector (VS+0x01, WALK..POWER) subject to the IN2 gate bits, then
 *                      the cruise / light / headlight-toggle event blocks. Each handled
 *                      event emits its Ninebot report frame.
 */
void ridemode_gear_event_tick(void);

/**
 * \brief           Recompute the rear-light mode
 * \note            Derives the rear-light mode (\ref vcu_mode_light_t, MODE+0x10 bits [11:9])
 *                      from regen / brake, reg 0x5D (flashing brake light), the light
 *                      outputs, and while charging the charge-display bit and the SoC.
 */
void ridemode_status_word_update(void);

/**
 * \brief           Project the ride mode onto the LED / status bit matrix
 * \note            Reads the live 3-way drive-row byte at g_vs+0x00 (vcu_region_t:
 *                      GLOBAL=0 / EUROPE=1 / GERMANY=2) and rewrites the LED / status bitfield
 *                      matrix held at g_in2+0x36 (an SRAM blackboard, NOT a peripheral). A common preset/clear frame is
 *                      applied to three 16-bit matrix words (+0x00 main, +0x2A and +0x2C
 *                      auxiliary), then a per-mode override group selects the annunciator
 *                      pattern for the current drive row.
 */
void ridemode_led_matrix_update(void);

/**
 * \brief           Rear-light mode off
 * \note            Clears MODE+0x10 bits [11:9] (the rear-light mode \ref ridemode_status_word_update
 *                      writes) via mode_light_set(MODE_LIGHT_OFF). Only caller: the vehicle FSM
 *                      state-4 (power-down) entry.
 */
void ridemode_led_indicator_clear(void);

#endif /* RIDEMODE_H */
