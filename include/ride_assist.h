/**
 * \file            ride_assist.h
 * \brief           Ride assists: climb boost, over-speed regen, turn-signal auto-cancel,
 *                      wobble warning (x3 VCU) -- public API
 * \note            The ride state (vehicle_state_ride) runs ride_assist_tick() once per tick
 *                      after the input scan and before the drive command;
 *                      control_build_drive_cmd() then applies the climb-boost current /
 *                      energy overrides and the over-speed regen level read through
 *                      ride_assist_state().
 *
 *                      The state is one module-private struct; ride_assist_init() sets its
 *                      boot values.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef RIDE_ASSIST_H
#define RIDE_ASSIST_H

#include <stdint.h>

/**
 * \brief           Turn-signal auto-cancel state
 * \note            The auto-cancel reset clears these 0x2C bytes as one block, so they stay
 *                      one struct.
 */
typedef struct {
    int32_t yaw_filt;     /*!< low-passed signed yaw rate (0.9 / 0.1 IIR), raw LSB          */
    int32_t yaw_prev;     /*!< last tick's yaw_filt                                          */
    int32_t heading;      /*!< heading integral (sum of yaw_prev while turning)              */
    float heading_deg;    /*!< heading integral in degrees (written, no reader)              */
    uint16_t settle_cnt;  /*!< ticks since the last new heading peak                         */
    uint8_t pad_a[2];     /*!< padding                                                 */
    int32_t heading_peak; /*!< heading peak x 1.1                                            */
    float peak_deg;       /*!< heading peak in degrees (> 70 cancels)                       */
    int32_t settle_sum;   /*!< heading sum since the last peak                               */
    int32_t settle_deg;   /*!< mean heading over the settle window, degrees                  */
    uint8_t phase;        /*!< 0 idle, 1 armed, 2 turning, 3 cancel                          */
    int8_t dir;           /*!< expected turn direction: -1 for signal 2, +1 for signal 1     */
    uint8_t signal;       /*!< light state: 0 none, 1 hi toggle, 2 lo toggle, 3 hazard      */
    uint8_t signal_prev;  /*!< last tick's signal                                            */
    uint16_t timeout;     /*!< ticks left before the cancel (2500 on arm, >= 300 on a peak)  */
    uint8_t pad_b[2];     /*!< padding (inside the cleared block)                    */
} ride_turn_cancel_t;

/**
 * \brief           Ride-assist working state
 * \note            Read by control.c (throttle scan, drive command) and by the tests. Two
 *                      cells have a reader but no writer: `throttle_rearm` and `coast_energy`.
 *                      They stay 0.
 */
typedef struct {
    /* Zero at boot. */
    uint8_t throttle_rearm;    /*!< throttle held off until it returns to 0 (no writer)       */
    uint8_t coast_energy;      /*!< energy byte with throttle 0 and OCFG_COAST_PRESET (no writer) */
    uint8_t climb_current;     /*!< climb-boost current override                              */
    uint16_t climb_energy;     /*!< climb-boost energy override (low byte used), max 160      */
    float climb_target;        /*!< climb-boost target factor 1.0 .. 4.0                      */
    float climb_factor;        /*!< climb-boost factor, slewed to the target (+0.04 / -0.01)  */
    uint8_t climb_ramp;        /*!< climb-boost engage ramp (10 .. 80, 200 when engaged)       */
    uint16_t climb_hold;       /*!< full-throttle hold count (10 .. 200)                      */
    uint8_t climb_allowed;     /*!< battery-voltage gate for the boost                         */
    uint8_t overspeed_regen;   /*!< over-speed regen strength min(level x 50, 45)             */
    float overspeed_level;     /*!< over-speed regen level 0.0 .. 1.0                          */
    uint16_t wobble_threshold; /*!< |rate| threshold for a wobble swing (250 / 170 by reg 0x75) */
    uint16_t wobble_count;     /*!< wobble score (10 .. 300, 600 after a warning)              */
    ride_turn_cancel_t turn;   /*!< turn-signal auto-cancel                             */
    /* Sample dividers and speed tracking (boot values set by ride_assist_init()). */
    uint8_t climb_div;        /*!< climb sample divider (boot 10, 14 after a sample)        */
    uint8_t overspeed_div;    /*!< over-speed sample divider (boot 10)                     */
    int8_t wobble_sign;       /*!< sign of the last wobble swing (boot -1)                  */
    uint8_t wobble_decay;     /*!< swing-gap counter (boot 80, 50 on a swing)               */
    uint16_t climb_ref_speed; /*!< climb reference speed (0.1 km/h)                         */
    uint16_t climb_speed;     /*!< climb sampled speed (0.1 km/h)                           */
    uint16_t coast_ticks;     /*!< ticks without throttle or brake (boot 10, max 200)       */
    uint16_t overspeed_ref;   /*!< over-speed reference speed (0.1 km/h)                    */
    uint16_t overspeed_speed; /*!< over-speed sampled speed (0.1 km/h)                      */
    uint16_t overspeed_ceil;  /*!< last ceiling seen (0.1 km/h)                             */
    uint16_t overspeed_hold;  /*!< hold-off after a ceiling change (500 ticks)              */
} ride_assist_state_t;

/**
 * \brief           Run the ride assists for one tick
 * \param[in]       throttle: throttle command 0..200 (MODE drive_output_word low byte)
 * \param[in]       brake: brake command 0..200 (MODE drive_output_word high byte)
 * \param[in]       wobble_rate: gyro axis 1 of the IMU sample (-GY, raw LSB)
 * \param[in]       yaw_rate: gyro axis 2 of the IMU sample (-GZ, raw LSB)
 * \note            The speed is IN2 speed_src (0.1 km/h).
 */
void ride_assist_tick(uint32_t throttle, uint32_t brake, int32_t wobble_rate, int32_t yaw_rate);

/**
 * \brief           Set the ride-assist boot values
 * \note            Called once by main before the scheduler starts. The values are set by
 *                      code, not as .data, to keep flash.
 */
void ride_assist_init(void);

/**
 * \brief           Reset climb boost, over-speed regen and the wobble warning
 * \note            Called by ride_assist_tick (gear 1 or SCR+0x52 set) and by the throttle scan
 *                      while OCFG_BRAKE_CAP is set.
 */
void ride_assist_reset_all(void);

/**
 * \brief           Release the throttle re-arm latch (`throttle_rearm` = 0)
 * \note            The throttle scan's only write to the latch, once the throttle reads 0.
 *                      No code sets the latch.
 */
void ride_assist_throttle_rearm_clear(void);

/**
 * \brief           Read-only view of the ride-assist state
 * \return          the module's state
 */
const volatile ride_assist_state_t* ride_assist_state(void);

#endif /* RIDE_ASSIST_H */
