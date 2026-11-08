/**
 * \file            ride_assist.c
 * \brief           Ride assists: climb boost, over-speed regen, turn-signal auto-cancel,
 *                      wobble warning (x3 VCU)
 * \note            The assists the ride state runs every tick:
 *                        ride_assist_tick       dispatcher
 *                        prv_turn_cancel_tick   turn-signal auto-cancel
 *                        prv_climb_update       climb boost
 *                        prv_overspeed_update   over-speed regen
 *                        prv_wobble_update      wobble warning
 *                        ride_assist_reset_all  reset of climb boost, over-speed regen, wobble
 *                        prv_*_reset            the per-assist resets
 *                      Fractional maths runs in double and is stored as float or truncated to
 *                      int; the operand order and conversions are fixed, so the values are
 *                      repeatable. Stored floats hold non-negative values and compare as floats.
 *
 *                      None of this caps the vehicle speed: the over-speed regen only adds motor
 *                      braking while coasting above the ride-mode ceiling.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "ride_assist.h"
#include "model.h" /* VCU_HAS_FEATURE */
#include <string.h>
#include "buzzer.h"
#include "vcu_state.h"

/* ------------------------------------------------------------------------- */
/* Constants.                                                                 */
/* ------------------------------------------------------------------------- */

/* Shared sampling: both the climb boost and the over-speed regen re-sample the speed every */
/* 11th / 12th tick; the divider restarts at 14 and the reference speed is taken while it  */
/* is below 15.                                                                              */
#define RA_DIV_REF_BELOW     15u /*!< divider < 15: copy the speed into the reference      */
#define RA_DIV_RESTART       14u /*!< divider value after a sample                         */
#define RA_DIV_CLIMB_SAMPLE  25u /*!< climb samples when the divider is above this         */
#define RA_DIV_OS_SAMPLE     25u /*!< over-speed samples when the divider is at least this */

/* Climb boost. */
#define CLIMB_VBUS_ALLOW     125u  /*!< vbus code below this: boost allowed                   */
#define CLIMB_VBUS_FORBID    135u  /*!< vbus code above this: boost forbidden                 */
#define CLIMB_SOC_MIN        20u   /*!< SCR soc_combined below this resets the factors        */
#define CLIMB_FULL_THROTTLE  195u  /*!< throttle above this (>= 98 %) counts as held          */
#define CLIMB_HOLD_RESET     10u   /*!< hold count without full throttle                      */
#define CLIMB_HOLD_MAX       200u  /*!< hold count clamp                                      */
#define CLIMB_HOLD_ARMED     100u  /*!< the target only rises after this many held ticks      */
#define CLIMB_HOLD_LOW       11u   /*!< below this the target drops by 0.3                    */
#define CLIMB_RAMP_MIN       10u   /*!< engage-ramp floor                                     */
#define CLIMB_RAMP_MAX       80u   /*!< engage-ramp ceiling                                   */
#define CLIMB_RAMP_ENGAGED   200u  /*!< engage-ramp value when the boost engages              */
#define CLIMB_RAMP_RELEASE   14u   /*!< engage-ramp below this releases the boost             */
#define CLIMB_SPEED_MIN      30u   /*!< speed (0.1 km/h) at or below: no boost                */
#define CLIMB_ENGAGE_SPEED   70u   /*!< sampled speed above this may engage                   */
#define CLIMB_ENERGY_MAX     160u  /*!< energy override clamp                                 */
#define CLIMB_CURRENT_MARGIN 5u    /*!< current override cap = k + 5                          */
#define CLIMB_CURRENT_BASE   25u   /*!< the caller's k (ride_assist_tick passes 0x19)         */

/* Over-speed regen. */
#define OS_HOLD_TICKS        500u /*!< hold-off after a ceiling change                       */
#define OS_THROTTLE_IDLE     5u   /*!< throttle below this (and no brake) counts as coasting */
#define OS_COAST_RESET       10u  /*!< coast count with throttle or brake                    */
#define OS_COAST_MAX         200u /*!< coast count clamp                                     */
#define OS_COAST_ARMED       40u  /*!< the level only rises after this many coasting ticks   */
#define OS_COAST_RECENT      15u  /*!< at or below: throttle/brake used recently, level drops */
#define OS_SPEED_MIN         105u /*!< 10.5 km/h: below it the level decays                  */
#define OS_SPEED_FAST        200u /*!< 20.0 km/h: extra rise above it                        */
#define OS_CEIL_MARGIN       5u   /*!< 0.5 km/h above the ceiling                            */
#define OS_DECEL_MARGIN      9u   /*!< reference above speed + 0.9 km/h: level x 0.9         */
#define OS_FLAG_SPEED        120u /*!< 12.0 km/h: the flag needs more                        */
#define OS_REGEN_RESET       1u   /*!< regen strength after a reset                          */

/* Wobble warning. */
#define WOBBLE_THR_SENS1     250u /*!< reg 0x75 == 1                                         */
#define WOBBLE_THR_SENS2     170u /*!< reg 0x75 == 2                                         */
#define WOBBLE_SWING_GAP     50u  /*!< gap counter after a swing                             */
#define WOBBLE_GAP_DECAY     30u  /*!< gap counter below this: the score decays              */
#define WOBBLE_GAP_FLOOR     10u  /*!< gap counter floor                                     */
#define WOBBLE_COUNT_RESET   10u  /*!< score after a reset / floor                           */
#define WOBBLE_COUNT_MAX     300u /*!< score clamp while swinging                            */
#define WOBBLE_COUNT_WARN    25u  /*!< score above this raises the warning                   */
#define WOBBLE_COUNT_ALERT   600u /*!< score after a warning                                 */
#define WOBBLE_COUNT_CLEAR   15u  /*!< score below this clears the warning                   */

/* Turn-signal auto-cancel. */
#define TURN_YAW_DEADBAND    19    /*!< |filtered yaw| <= this reads 0                        */
#define TURN_TIMEOUT_ARM     2500u /*!< ticks from the light toggle to the cancel             */
#define TURN_TIMEOUT_PEAK    300u  /*!< at least this many ticks left after a new peak        */
#define TURN_TIMEOUT_END     10u   /*!< below this the cancel fires                           */
#define TURN_SPEED_MIN       40u   /*!< 4.0 km/h: the turn tracking starts above it           */
#define TURN_DEG_PER_LSB_TICK 6.096271e-4f /*!< 0.001064 rad/s per LSB x 180 / pi x 0.01 s tick */
#define TURN_SETTLE_TICKS    200u  /*!< ticks after the last peak before the mean is checked  */
#define TURN_SETTLE_DONE_DEG 15    /*!< mean heading above this: turn done                    */
#define TURN_SETTLE_IDLE_DEG 5     /*!< mean heading below this: back to armed                */
#define TURN_PEAK_DONE_DEG   70.0f /*!< peak above this: turn done                            */

/* Rider-input phases of the auto-cancel. */
enum {
    TURN_IDLE = 0,    /*!< no light toggle                    */
    TURN_ARMED = 1,   /*!< light on, waiting for the turn     */
    TURN_TURNING = 2, /*!< tracking the heading               */
    TURN_CANCEL = 3   /*!< inject the light-button press      */
};

/* Light state seen by the auto-cancel (g_chg toggles). */
enum {
    TURN_SIG_NONE = 0,  /*!< no light toggle             */
    TURN_SIG_HI = 1,    /*!< chg+1 (LIGHT_HI toggle)     */
    TURN_SIG_LO = 2,    /*!< chg+0 (LIGHT_LO toggle)     */
    TURN_SIG_HAZARD = 3 /*!< chg+4 (hazard latch)        */
};

/** \brief Event code an injected light-button press carries (fm_mon_t.code == 1, a short press). */
#define TURN_PRESS_CODE      1u

/* ------------------------------------------------------------------------- */
/* State.                                                                     */
/* ------------------------------------------------------------------------- */

/* Boot values set by ride_assist_init(). */
#define RA_BOOT_DIV          10u /*!< climb / over-speed sample divider */
#define RA_BOOT_WOBBLE_SIGN  -1  /*!< sign of the last wobble swing      */
#define RA_BOOT_WOBBLE_DECAY 80u /*!< swing-gap counter                  */
#define RA_BOOT_COAST_TICKS  10u /*!< ticks without throttle or brake    */

/**
 * \brief           Ride-assist state
 * \note            Boot values: climb divider 10, over-speed divider 10, swing sign -1,
 *                      swing gap 80, coast count 10; everything else 0. Only the vehicle FSM task
 *                      touches it (ride_assist_tick, ride_assist_reset_all and the drive-command
 *                      readers in control.c run in that task), so it is a plain object: the
 *                      compiler keeps the cells in registers between the reads and writes of one
 *                      call. Readers get it through the const volatile view ride_assist_state().
 */
static ride_assist_state_t prv_state;

/**
 * \brief           Set the ride-assist boot values
 * \note            main sets them before the scheduler starts. The rest of the state is .bss
 *                      (0).
 */
void ride_assist_init(void) {
    prv_state.climb_div = RA_BOOT_DIV;
    prv_state.overspeed_div = RA_BOOT_DIV;
    prv_state.wobble_sign = RA_BOOT_WOBBLE_SIGN;
    prv_state.wobble_decay = RA_BOOT_WOBBLE_DECAY;
    prv_state.coast_ticks = RA_BOOT_COAST_TICKS;
}

/* ------------------------------------------------------------------------- */
/* Resets.                                                                    */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Turn-signal auto-cancel reset
 * \note            Clears the whole auto-cancel block (0x2C bytes).
 */
static void prv_turn_cancel_reset(void) { memset((void*)&prv_state.turn, 0, sizeof(prv_state.turn)); }

/**
 * \brief           Over-speed regen reset
 * \note            overspeed_level = 0.0f, overspeed_regen = 1, SCR+0x96 = 0,
 *                      OCFG_OVERSPEED_REGEN cleared.
 */
static void prv_overspeed_reset(void) {
    prv_state.overspeed_level = 0.0f;
    prv_state.overspeed_regen = OS_REGEN_RESET;
    g_scr.overspeed_regen_on = 0u;
    g_mode.output_cfg = (uint8_t)(g_mode.output_cfg & ~OCFG_OVERSPEED_REGEN);
}

/**
 * \brief           Wobble warning reset
 * \note            wobble_count = 10, OCFG_WOBBLE_ALERT cleared, SCR+0x98 = 0.
 */
static void prv_wobble_reset(void) {
    prv_state.wobble_count = WOBBLE_COUNT_RESET;
    g_mode.output_cfg = (uint8_t)(g_mode.output_cfg & ~OCFG_WOBBLE_ALERT);
    g_scr.wobble_alert_on = 0u;
}

/**
 * \brief           Climb boost reset
 * \note            climb_target = climb_factor = 1.0f, climb_current = 0, climb_energy = 0,
 *                      climb_ramp = 10, climb_hold = 10, SCR+0x92 = 0, OCFG_CLIMB_BOOST cleared.
 */
static void prv_climb_reset(void) {
    prv_state.climb_target = 1.0f;
    prv_state.climb_factor = 1.0f;
    prv_state.climb_current = 0u;
    prv_state.climb_energy = 0u;
    prv_state.climb_ramp = CLIMB_RAMP_MIN;
    prv_state.climb_hold = CLIMB_HOLD_RESET;
    g_scr.climb_boost_on = 0u;
    g_mode.output_cfg = (uint8_t)(g_mode.output_cfg & ~OCFG_CLIMB_BOOST);
}

/**
 * \brief           Reset over-speed regen, wobble warning and climb boost
 */
void ride_assist_reset_all(void) {
    prv_overspeed_reset();
    prv_wobble_reset();
    prv_climb_reset();
}

/* ------------------------------------------------------------------------- */
/* Turn-signal auto-cancel.                                                   */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Convert a heading integral (raw yaw LSB x ticks) to degrees
 * \param[in]       raw: heading integral
 * \return          degrees: raw x 0.001064 (rad/s per LSB) x 180 / pi x 0.01 (tick period)
 */
static float prv_turn_degrees(int32_t raw) { return (float)raw * TURN_DEG_PER_LSB_TICK; }

/**
 * \brief           Phase 2: integrate the heading, track its peak and decide when the turn is over
 * \param[in]       timeout: the decremented timeout of this tick
 */
static void prv_turn_track(uint16_t timeout) {
    ride_turn_cancel_t* turn = &prv_state.turn;
    int32_t heading;
    int32_t mean;

    turn->timeout = timeout;
    heading = turn->heading + turn->yaw_prev;
    turn->heading = heading;
    turn->heading_deg = prv_turn_degrees(heading);
    if (heading > turn->heading_peak) {
        turn->settle_cnt = 0u;
        turn->settle_sum = 0;
        turn->heading_peak = (int32_t)((float)heading * 1.1f);
        turn->peak_deg = prv_turn_degrees(turn->heading_peak);
        if (timeout < TURN_TIMEOUT_PEAK) {
            turn->timeout = TURN_TIMEOUT_PEAK;
        }
    } else {
        turn->settle_cnt = (uint16_t)(turn->settle_cnt + 1u);
        turn->settle_sum = turn->settle_sum + heading;
        if (turn->settle_cnt > TURN_SETTLE_TICKS) {
            mean = turn->settle_sum / (int32_t)turn->settle_cnt;
            turn->settle_deg = (int32_t)prv_turn_degrees(mean);
            turn->settle_cnt = 0u;
            if ((turn->settle_deg > TURN_SETTLE_DONE_DEG) || (turn->peak_deg > TURN_PEAK_DONE_DEG)) {
                turn->phase = TURN_CANCEL;
            } else if (turn->settle_deg < TURN_SETTLE_IDLE_DEG) {
                turn->phase = TURN_ARMED;
            }
        }
    }
    /* Turned back below half the peak: re-arm (heading < peak / 2, exact in 64 bits). */
    if (((int64_t)heading * 2) < (int64_t)turn->heading_peak) {
        turn->phase = TURN_ARMED;
        turn->settle_cnt = 0u;
    }
}

/**
 * \brief           Turn-signal auto-cancel tick
 * \param[in]       speed: vehicle speed, 0.1 km/h
 * \param[in]       yaw_rate: gyro axis 2 (raw LSB)
 * \note            Starts on a light-lo / light-hi toggle (g_chg +0 / +1), low-passes the yaw rate
 *                      signed by the expected direction, tracks the heading while above 4 km/h and,
 *                      once the turn is done or after the timeout, injects a press of the same light
 *                      button into the input-monitor block (EVT LIGHT_LO / LIGHT_HI code = 1) so the
 *                      light FSM switches the indicator off.
 */
static void prv_turn_cancel_tick(uint32_t speed, int32_t yaw_rate) {
    ride_turn_cancel_t* turn = &prv_state.turn;
    uint8_t signal;
    uint16_t timeout;
    uint8_t check_timeout;

    if (g_chg.f0 != 0u) {
        signal = TURN_SIG_LO;
    } else if (g_chg.f1 != 0u) {
        signal = TURN_SIG_HI;
    } else if (g_chg.f4 != 0u) {
        signal = TURN_SIG_HAZARD;
    } else {
        signal = TURN_SIG_NONE;
    }
    turn->signal = signal;
    if (signal != turn->signal_prev) {
        turn->phase = TURN_IDLE;
        turn->heading = 0;
        turn->heading_peak = 0;
        turn->settle_cnt = 0u;
        if (signal == TURN_SIG_LO) {
            turn->dir = -1;
        } else if (signal == TURN_SIG_HI) {
            turn->dir = 1;
        }
    }

    /* yaw_filt = (yaw_filt * 0.9 + dir * 0.1 * yaw), truncated to 0 */
    turn->yaw_filt = ((turn->yaw_filt * 9) + ((int32_t)turn->dir * yaw_rate)) / 10;
    if ((turn->yaw_filt >= -TURN_YAW_DEADBAND) && (turn->yaw_filt <= TURN_YAW_DEADBAND)) {
        turn->yaw_filt = 0;
    }

    check_timeout = 0u;
    timeout = (uint16_t)(turn->timeout - 1u);
    switch (turn->phase) {
        case TURN_IDLE:
            if ((signal == TURN_SIG_LO) || (signal == TURN_SIG_HI)) {
                turn->timeout = TURN_TIMEOUT_ARM;
                turn->phase = TURN_ARMED;
            }
            break;
        case TURN_ARMED:
            turn->timeout = timeout;
            if ((speed > TURN_SPEED_MIN) && (turn->yaw_filt > 0)) {
                turn->phase = TURN_TURNING;
                turn->heading = 0;
                turn->heading_peak = 0;
            }
            check_timeout = 1u;
            break;
        case TURN_TURNING:
            prv_turn_track(timeout);
            check_timeout = (turn->phase != TURN_IDLE) ? 1u : 0u;
            break;
        case TURN_CANCEL:
            turn->phase = TURN_IDLE;
            if (g_chg.f0 != 0u) {
                g_evt.mon[FM_MON_LIGHT_LO].code = TURN_PRESS_CODE;
            } else if (g_chg.f1 != 0u) {
                g_evt.mon[FM_MON_LIGHT_HI].code = TURN_PRESS_CODE;
            }
            break;
        default: check_timeout = 1u; break;
    }
    if ((check_timeout != 0u) && (turn->timeout < TURN_TIMEOUT_END)) {
        turn->timeout = 0u;
        turn->phase = TURN_CANCEL;
    }
    turn->yaw_prev = turn->yaw_filt;
    turn->signal_prev = turn->signal;
}

/* ------------------------------------------------------------------------- */
/* Climb boost.                                                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Battery-voltage gate, engage-ramp clamp, low-SoC reset and the hold count
 * \param[in]       throttle: throttle command
 * \param[in]       brake: brake command
 * \param[in]       soc: SCR soc_combined
 */
static void prv_climb_inputs(uint32_t throttle, uint32_t brake, uint32_t soc) {
    uint32_t vcode = (uint32_t)(g_fault_codes.vbus_code >> 8);
    uint16_t hold;

    if (vcode > CLIMB_VBUS_FORBID) {
        prv_state.climb_allowed = 0u;
    } else if (vcode < CLIMB_VBUS_ALLOW) {
        prv_state.climb_allowed = 1u;
    } else {
        prv_state.climb_allowed = ((g_mode.output_cfg & OCFG_CLIMB_BOOST) != 0u) ? 1u : 0u;
    }
    if ((prv_state.climb_hold < CLIMB_HOLD_ARMED) || (prv_state.climb_ramp < CLIMB_RAMP_MIN)) {
        prv_state.climb_ramp = CLIMB_RAMP_MIN;
    } else if (prv_state.climb_ramp > CLIMB_RAMP_MAX) {
        prv_state.climb_ramp = CLIMB_RAMP_MAX;
    }
    if (soc < CLIMB_SOC_MIN) {
        prv_state.climb_target = 1.0f;
        prv_state.climb_factor = 1.0f;
        g_scr.climb_boost_on = 0u;
    }
    if ((throttle > CLIMB_FULL_THROTTLE) && (brake == 0u)) {
        hold = prv_state.climb_hold;
        prv_state.climb_hold = (uint16_t)(hold + 1u);
        if (hold > CLIMB_HOLD_MAX) {
            prv_state.climb_hold = CLIMB_HOLD_MAX;
        }
    } else {
        prv_state.climb_hold = CLIMB_HOLD_RESET;
    }
}

/**
 * \brief           New climb target from one speed sample
 * \param[in]       speed: sampled speed, 0.1 km/h
 * \param[in]       ceil10: ride-mode ceiling, 0.1 km/h
 * \param[in]       floor10: profile speed (IN2+0x84 x 10) at or below which nothing boosts
 * \return          the new target before the clamp
 */
static float prv_climb_target(uint32_t speed, uint32_t ceil10, uint32_t floor10) {
    float target = prv_state.climb_target;
    uint32_t ref = prv_state.climb_ref_speed;
    uint8_t held = (prv_state.climb_hold > CLIMB_HOLD_ARMED) ? 1u : 0u;

    if ((speed <= floor10) || (speed <= CLIMB_SPEED_MIN) || (g_scr.lock_req != 0u)) {
        target = target - 0.1f;
    } else if ((speed * 10u) < (ceil10 * 9u)) { /* speed < 0.9 x ceiling */
        if ((speed < (ref + 1u)) && (held != 0u)) {
            if (target < 2.4f) {
                if (((target + 0.15f) > 1.7f) && (target < 1.7f)) {
                    target = target + 0.02f;
                } else {
                    target = target + 0.15f;
                }
            } else {
                target = target + 0.1f;
            }
        } else {
            target = target - 0.06f;
            if (speed > (ref + 5u)) {
                target = target - 0.1f;
            }
        }
    } else {
        if ((speed < ref) && (held != 0u)) {
            target = target + ((target < 2.0f) ? 0.1f : 0.05f);
        } else {
            target = target - 0.05f;
            if (speed > (ref + 5u)) {
                target = target - 0.1f;
            }
            if (speed > (ceil10 + 3u)) {
                target = target - 0.1f;
            }
        }
        if ((int32_t)speed > ((int32_t)ceil10 - 15)) {
            target = target - 0.13f;
        }
    }
    if (prv_state.climb_hold < CLIMB_HOLD_LOW) {
        target = target - 0.3f;
    }
    return target;
}

/**
 * \brief           Slew the factor towards the target and move the engage ramp
 */
static void prv_climb_slew(void) {
    float factor = prv_state.climb_factor;

    if (factor < prv_state.climb_target) {
        factor = factor + 0.04f;
    } else if (prv_state.climb_target < factor) {
        factor = factor - 0.01f;
    }
    if (factor < 1.0f) {
        factor = 1.0f;
        prv_state.climb_ramp = (uint8_t)(prv_state.climb_ramp - 2u);
    } else if (factor > 4.0f) {
        factor = 4.0f;
        prv_state.climb_ramp = (uint8_t)(prv_state.climb_ramp + 30u);
    } else if (factor <= 1.01f) {
        prv_state.climb_ramp = (uint8_t)(prv_state.climb_ramp - 2u);
    } else {
        prv_state.climb_ramp = (uint8_t)(prv_state.climb_ramp + 30u);
    }
    prv_state.climb_factor = factor;
}

/**
 * \brief           Engage / release the boost and compute the gear's current / energy overrides
 * \param[in]       current_base: the caller's k (0x19)
 */
static void prv_climb_outputs(uint32_t current_base) {
    float gain;
    uint32_t current;
    int16_t gear;
    uint16_t energy;

    if ((prv_state.climb_allowed == 1u) && (prv_state.climb_speed > CLIMB_ENGAGE_SPEED)
        && ((g_mode.output_cfg & OCFG_CLIMB_BOOST) == 0u) && (prv_state.climb_factor > 1.65f)) {
        prv_state.climb_ramp = CLIMB_RAMP_ENGAGED;
        g_scr.climb_boost_on = 1u;
        g_mode.output_cfg = (uint8_t)(g_mode.output_cfg | OCFG_CLIMB_BOOST);
    } else if ((prv_state.climb_allowed == 0u)
               || ((prv_state.climb_ramp < CLIMB_RAMP_RELEASE) && ((g_mode.output_cfg & OCFG_CLIMB_BOOST) != 0u))) {
        g_scr.climb_boost_on = 0u;
        g_mode.output_cfg = (uint8_t)(g_mode.output_cfg & ~OCFG_CLIMB_BOOST);
    }
    if (prv_state.climb_factor >= 0.98f) {
        gain = prv_state.climb_factor - 1.0f;
        gear = g_in2.f.gear;
        energy = prv_state.climb_energy;
        if (gear == 2) {
            energy = (uint16_t)(int32_t)(((gain * 1.3f) + 1.0f) * 50.0f);
        } else if (gear == 3) {
            energy = (uint16_t)(int32_t)(((gain * 0.5f) + 1.0f) * 90.0f);
        } else if (gear == 4) {
            energy = (uint16_t)(int32_t)(((gain / 5.0f) + 1.0f) * 140.0f);
        }
        if (energy > CLIMB_ENERGY_MAX) {
            energy = CLIMB_ENERGY_MAX;
        }
        prv_state.climb_energy = energy;
        current = (uint8_t)(((((int32_t)energy - 50) * 24) + 2000) / 100); /* x 0.24 + 20 */
        if (current > (current_base + CLIMB_CURRENT_MARGIN)) {
            current = current_base + CLIMB_CURRENT_MARGIN;
        }
        prv_state.climb_current = (uint8_t)current;
    }
}

/**
 * \brief           Climb boost
 * \param[in]       throttle: throttle command
 * \param[in]       brake: brake command
 * \param[in]       speed: vehicle speed, 0.1 km/h
 * \param[in]       ceil10: ride-mode ceiling (MODE cmd_lead x 5), 0.1 km/h
 * \param[in]       floor10: IN2+0x84 x 10, truncated to 16 bits
 * \note            Full throttle held for 100 ticks without the speed rising ramps a 1.0..4.0
 *                      factor; above 1.65 the boost engages (OCFG_CLIMB_BOOST, SCR+0x92) and the drive
 *                      command takes the current / energy overrides.
 */
static void prv_climb_update(uint32_t throttle, uint32_t brake, uint32_t speed, uint32_t ceil10, uint32_t floor10) {
    uint8_t div;
    float target;

    prv_climb_inputs(throttle, brake, (uint32_t)g_scr.soc_combined);
    div = prv_state.climb_div;
    if (div < RA_DIV_REF_BELOW) {
        prv_state.climb_ref_speed = (uint16_t)speed;
    }
    prv_state.climb_div = (uint8_t)(div + 1u);
    if (div > RA_DIV_CLIMB_SAMPLE) {
        prv_state.climb_div = RA_DIV_RESTART;
        prv_state.climb_speed = (uint16_t)speed;
        target = prv_climb_target(speed, ceil10, floor10);
        if (target < 1.0f) {
            target = 1.0f;
        } else if (target > 4.0f) {
            target = 4.0f;
        }
        prv_state.climb_target = target;
    }
    prv_climb_slew();
    prv_climb_outputs(CLIMB_CURRENT_BASE);
}

/* ------------------------------------------------------------------------- */
/* Over-speed regen.                                                          */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Level change of one over-speed sample
 * \param[in]       speed: sampled speed, 0.1 km/h
 * \param[in]       limit: ceiling + 0.5 km/h
 * \return          the new level before the clamp
 * \note            The 0.3 / 0.7 / 0.9 bands are float compares on the level.
 */
static float prv_overspeed_level(uint32_t speed, uint32_t limit) {
    float level = prv_state.overspeed_level;
    uint32_t ref = prv_state.overspeed_ref;
    uint8_t coasting = (prv_state.coast_ticks > OS_COAST_ARMED) ? 1u : 0u;

    if (speed > OS_SPEED_MIN) {
        if ((speed > ref) && (coasting != 0u)) {
            level = level + ((level < 0.3f) ? 0.03f : 0.01f);
            if (speed > OS_SPEED_FAST) {
                level = level + 0.05f;
            }
        }
        if ((speed > limit) && (coasting != 0u)) {
            if (level < 0.3f) {
                level = level + 0.08f;
            } else if (level <= 0.7f) {
                level = level + 0.04f;
            } else if (level <= 0.9f) {
                level = level + 0.01f;
            } else {
                level = level + 0.001f;
            }
        }
    } else if (speed < OS_SPEED_MIN) {
        level = level - 0.03f;
    }
    if ((speed <= limit) && (speed <= ref)) {
        level = level - 0.01f;
        if (ref > (speed + OS_DECEL_MARGIN)) {
            level = level * 0.9f;
        }
    }
    if (prv_state.coast_ticks <= OS_COAST_RECENT) {
        level = level - 0.03f;
    }
    return level;
}

/**
 * \brief           Over-speed regen
 * \param[in]       throttle: throttle command
 * \param[in]       brake: brake command
 * \param[in]       speed: vehicle speed, 0.1 km/h
 * \param[in]       ceil10: ride-mode ceiling (MODE cmd_lead x 5), 0.1 km/h
 * \note            Coasting above the ceiling + 0.5 km/h raises a 0..1 level; the regen strength
 *                      overspeed_regen = min(level x 50, 45) feeds the drive command's regen byte. Above
 *                      12 km/h and level 0.25 the flag SCR+0x96 rises and the climb boost resets.
 */
static void prv_overspeed_update(uint32_t throttle, uint32_t brake, uint32_t speed, uint32_t ceil10) {
    uint16_t coast;
    uint8_t div;
    float level;
    float regen;
    uint32_t sampled;

    if ((ceil10 != prv_state.overspeed_ceil) && (prv_state.overspeed_hold == 0u)) {
        prv_state.overspeed_hold = OS_HOLD_TICKS;
    }
    prv_state.overspeed_ceil = (uint16_t)ceil10;
    if ((prv_state.overspeed_hold != 0u) && (speed <= ceil10)) {
        prv_state.overspeed_hold = (uint16_t)(prv_state.overspeed_hold - 1u);
    }
    if ((throttle >= OS_THROTTLE_IDLE) || (brake != 0u)) {
        prv_state.coast_ticks = OS_COAST_RESET;
    } else {
        coast = prv_state.coast_ticks;
        prv_state.coast_ticks = (uint16_t)(coast + 1u);
        if (coast > OS_COAST_MAX) {
            prv_state.coast_ticks = OS_COAST_MAX;
        }
    }
    div = prv_state.overspeed_div;
    if (div < RA_DIV_REF_BELOW) {
        prv_state.overspeed_ref = (uint16_t)speed;
    }
    prv_state.overspeed_div = (uint8_t)(div + 1u);
    if (div >= RA_DIV_OS_SAMPLE) {
        prv_state.overspeed_div = RA_DIV_RESTART;
        prv_state.overspeed_speed = (uint16_t)speed;
        level = prv_overspeed_level(speed, ceil10 + OS_CEIL_MARGIN);
        if (level > 1.0f) {
            level = 1.0f;
        } else if (level < 0.0f) {
            level = 0.0f;
        }
        prv_state.overspeed_level = level;
    }
    regen = prv_state.overspeed_level * 50.0f;
    if (regen >= 45.0f) {
        regen = 45.0f;
    }
    prv_state.overspeed_regen = (uint8_t)(uint32_t)regen;

    sampled = prv_state.overspeed_speed;
    if ((sampled > OS_FLAG_SPEED) && (prv_state.overspeed_hold == 0u)
        && ((g_scr.overspeed_regen_on == 0u) || ((sampled * 20u) > (ceil10 * 17u))) /* > 0.85 x ceiling */
        && (prv_state.overspeed_level > 0.25f)) {
        g_scr.overspeed_regen_on = 1u;
        prv_climb_reset();
    } else if ((g_scr.overspeed_regen_on == 1u) && (prv_state.overspeed_level < 0.05f)) {
        g_scr.overspeed_regen_on = 0u;
    }
}

/* ------------------------------------------------------------------------- */
/* Wobble warning (tail of ride_assist_tick).                                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Wobble warning tick
 * \param[in]       rate: gyro axis 1 (raw LSB)
 * \note            A swing is |rate| above the threshold with the sign opposite to the last swing.
 *                      Swings raise the score (max 300); quiet ticks with a swing gap below 30 lower
 *                      it. Over 25 the warning beeps (pattern 3), sets SCR+0x98 and OCFG_WOBBLE_ALERT
 *                      and the score to 600; below 15 it clears.
 */
static void prv_wobble_update(int32_t rate) {
    uint32_t magnitude = (rate < 0) ? (uint32_t)(-rate) : (uint32_t)rate;
    uint16_t count = prv_state.wobble_count;
    uint8_t gap;
    uint8_t warn = 0u;
    uint8_t clear = 0u;

    if (((int32_t)magnitude > (int32_t)prv_state.wobble_threshold) && (((int32_t)prv_state.wobble_sign * rate) < 0)) {
        prv_state.wobble_sign = (int8_t)(-prv_state.wobble_sign);
        prv_state.wobble_decay = WOBBLE_SWING_GAP;
        prv_state.wobble_count = (uint16_t)(count + 1u);
        if (count > WOBBLE_COUNT_MAX) {
            prv_state.wobble_count = WOBBLE_COUNT_MAX;
            warn = 1u;
        }
    } else {
        gap = prv_state.wobble_decay;
        prv_state.wobble_decay = (uint8_t)(gap - 1u);
        if (gap < WOBBLE_GAP_DECAY) {
            if ((uint8_t)(gap - 1u) < WOBBLE_GAP_FLOOR) {
                prv_state.wobble_decay = WOBBLE_GAP_FLOOR;
            }
            prv_state.wobble_count = (uint16_t)(count - 1u);
            if (count < WOBBLE_COUNT_RESET) {
                prv_state.wobble_count = WOBBLE_COUNT_RESET;
                clear = 1u;
            }
        }
    }
    if (clear == 0u) {
        if (((warn != 0u) || (prv_state.wobble_count > WOBBLE_COUNT_WARN))
            && ((g_mode.output_cfg & OCFG_WOBBLE_ALERT) == 0u)) {
            buzzer_beep(BEEP_DOUBLE);
            prv_state.wobble_count = WOBBLE_COUNT_ALERT;
            g_scr.wobble_alert_on = 1u;
            g_mode.output_cfg = (uint8_t)(g_mode.output_cfg | OCFG_WOBBLE_ALERT);
        }
        if (prv_state.wobble_count < WOBBLE_COUNT_CLEAR) {
            clear = 1u;
        }
    }
    if (clear != 0u) {
        g_scr.wobble_alert_on = 0u;
        g_mode.output_cfg = (uint8_t)(g_mode.output_cfg & ~OCFG_WOBBLE_ALERT);
    }
}

/* ------------------------------------------------------------------------- */
/* Dispatcher.                                                                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Run the ride assists for one tick
 * \param[in]       throttle: throttle command 0..200
 * \param[in]       brake: brake command 0..200
 * \param[in]       wobble_rate: gyro axis 1 (-GY, raw LSB)
 * \param[in]       yaw_rate: gyro axis 2 (-GZ, raw LSB)
 * \note            Order and gates: turn-signal auto-cancel unless reg 0x2F bit2; a reset of
 *                      all three assists with SCR+0x52 or gear 1; climb boost with reg 0x1E
 *                      bit2, no MODE+0xA bit7 and no OCFG_BRAKE_CAP; over-speed regen with reg
 *                      0x1E bit4 while the climb hold count is at most 180 (it sets
 *                      OCFG_OVERSPEED_REGEN first); the wobble warning while reg 0x75 is set or
 *                      the warning is up.
 */
void ride_assist_tick(uint32_t throttle, uint32_t brake, int32_t wobble_rate, int32_t yaw_rate) {
    uint32_t speed = (uint32_t)g_in2.f.speed_src;
    uint32_t ceil10 = (uint32_t)g_mode.cmd_lead * 5u;

    if (VCU_HAS_FEATURE(TURN_TRACKING) && ((g_in2.f.in2_5e & REG2F_TURN_CANCEL_OFF) == 0u)) {
        prv_turn_cancel_tick(speed, yaw_rate);
    } else {
        prv_turn_cancel_reset();
    }
    if ((g_scr.ilk_publish != 0u) || (g_in2.f.gear == 1)) {
        ride_assist_reset_all();
    }
    if (VCU_HAS_FEATURE(CLIMB_ASSIST) && ((g_in2.f.cfg_flags2 & CFG2_CLIMB_BOOST) != 0u) && ((g_mode.mode_flags & 0x0080u) == 0u)
        && ((g_mode.output_cfg & OCFG_BRAKE_CAP) == 0u)) {
        prv_climb_update(throttle, brake, speed, ceil10, (uint16_t)((int32_t)(int16_t)g_in2.f.fault_code * 10));
    } else {
        prv_climb_reset();
    }
    if (VCU_HAS_FEATURE(OVERSPEED_REGEN) && ((g_in2.f.cfg_flags2 & CFG2_OVERSPEED_REGEN) != 0u)
        && (prv_state.climb_hold <= 180u)) {
        g_mode.output_cfg = (uint8_t)(g_mode.output_cfg | OCFG_OVERSPEED_REGEN);
        prv_overspeed_update(throttle, brake, speed, ceil10);
    } else {
        prv_overspeed_reset();
    }
    if (!VCU_HAS_FEATURE(WOBBLE_ALERT) || (((g_mode.output_cfg & OCFG_WOBBLE_ALERT) == 0u) && (g_in2.f.wobble_sens == 0u))) {
        prv_wobble_reset();
    } else {
        if (g_in2.f.wobble_sens == 1u) {
            prv_state.wobble_threshold = WOBBLE_THR_SENS1;
        } else if (g_in2.f.wobble_sens == 2u) {
            prv_state.wobble_threshold = WOBBLE_THR_SENS2;
        }
        prv_wobble_update(wobble_rate);
    }
}

/**
 * \brief           Release the throttle re-arm latch (`throttle_rearm` = 0)
 */
void ride_assist_throttle_rearm_clear(void) { prv_state.throttle_rearm = 0u; }

/**
 * \brief           Read-only view of the ride-assist state
 * \return          the module's state
 */
const volatile ride_assist_state_t* ride_assist_state(void) { return &prv_state; }
