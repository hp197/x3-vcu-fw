/**
 * \file            odometer.h
 * \brief           Trip / session distance + time counters, range estimate (x3 VCU) -- public API
 * \note            Driven from the vehicle FSM: the per-tick odo/range update from the
 *                      ride run handlers (states 3 and 7) and the mileage pre-pass from the
 *                      shared FSM pre-pass.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef ODOMETER_H
#define ODOMETER_H

/**
 * \brief           Per-tick odometer + range-estimate update
 * \note            Runs the distance integrator + range estimate and the trip/session time
 *                      counters, derives the trip average speed trip_dist*360/trip_ride_time
 *                      into g_in2+0xDA, then updates the distance display.
 */
void odometer_power_tick(void);

/**
 * \brief           Mileage report-on-change pre-pass
 * \note            Depending on the distance field g_vs+0x0E, reports arg 0x24 values
 *                      0x1E / 100 on change and maintains the standby-off down-counter.
 */
void odometer_mileage_prepass(void);

#endif /* ODOMETER_H */
