/**
 * \file            charging.h
 * \brief           Charge timer / fault-code annunciation tick (x3 VCU)
 * \note            Public entry `vcu_charge_timer_tick` (called from the vehicle FSM in
 *                      vehicle.c), plus the private SRAM overlay this module owns.
 *
 *                      Run once per periodic tick. With no error code latched it services the
 *                      charge timer -- counting parked/idle minutes toward a full mark,
 *                      debouncing the two sensor-fault FSMs and emitting a slow charge chirp.
 *                      With an error code latched it blinks that code out as a two-digit beep +
 *                      LED pulse pattern (tens on a 60-tick cadence, units on 30) for three
 *                      cycles.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef CHARGING_H
#define CHARGING_H

#include <stddef.h>
#include <stdint.h>

/* Module overlay: the charge timer's working state is the 10-byte g_charge_timer
   (vcu_charge_timer_t, defined in vcu_state.c), reached by named field. The one foreign cell is
   +0x03, faultmon's FM_CHG_DWELL (charge-relay debounce) -- the struct's align-pad hole between
   code_snapshot(+2) and seg_tick(+4); charging never touches it, faultmon owns it. */

/**
 * \brief           Charge-timer / annunciation working state (10 bytes)
 * \note            `cycle` and `fault_debounce` are small modulo-3 counters; `seg_tick`
 *                      is the free-running annunciation segment tick; `chirp_tick` is the
 *                      charge-chirp countdown; `charge_min` is the parked-minute counter
 *                      (clamped at 100).
 */
typedef struct {
    uint8_t  cycle;          /*!< +0x00 annunciation repeat-cycle counter (resets at 3)   */
    uint8_t  fault_debounce; /*!< +0x01 sensor-fault confirm debounce (resets at 3)       */
    uint8_t  code_snapshot;  /*!< +0x02 last ERROR_ACTIVE seen; a change re-zeros timers   */
    uint8_t  relay_dwell;    /*!< +0x03 faultmon FM_CHG_DWELL (relay debounce); align-pad hole, faultmon owns it */
    uint16_t seg_tick;       /*!< +0x04 annunciation segment tick (tens/units cadence)    */
    uint16_t chirp_tick;     /*!< +0x06 charge-chirp countdown (reload 500 / 1000)        */
    uint16_t charge_min;     /*!< +0x08 parked charge-minute counter (0..100)             */
} vcu_charge_timer_t;

/* g_charge_timer is a typed global, defined in vcu_state.c. */
extern volatile vcu_charge_timer_t g_charge_timer;


/**
 * \brief           Active-event selector values read from EVENT_ACTIVE
 * \note            Only the two values the charge timer distinguishes are named. 0 means
 *                      no event is active (charge timer idles); 9 is the charging report
 *                      event (slow chirp cadence). Any other non-zero event uses the fast
 *                      cadence. The same byte is control.c's EVENT_ACTIVE (latched event).
 */
typedef enum {
    VCU_CHG_EVT_NONE     = 0, /*!< no active event -- charge timer idles              */
    VCU_CHG_EVT_CHARGING = 9  /*!< charging event (report event 9) -- slow chirp      */
} vcu_charge_evt_t;

/**
 * \brief           Charge timer / fault-code annunciation tick
 * \note            Call once per periodic tick.
 * \return          The abort/fault code when aborted via IN1+0x04; the mode/lighting dispatch
 *                      result when a full/confirmed condition fires; the live chirp countdown
 *                      while charging; or a "still annunciating" flag (cycle < 3) while blinking
 *                      out an error code. Non-zero generally means busy/active.
 */
uint32_t vcu_charge_timer_tick(void);

#endif /* CHARGING_H */
