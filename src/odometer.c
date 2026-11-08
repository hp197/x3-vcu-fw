/**
 * \file            odometer.c
 * \brief           Trip / session distance + time counters, range estimate and the mileage pre-pass
 * \note            The per-tick odo/range update the ride run handlers call (odometer_power_tick:
 *                      distance integrator and range estimate, trip/session time counters, trip
 *                      average speed, distance display) and the mileage report-on-change pre-pass
 *                      of the FSM. Soft-float precision: single precision (fdiv/fmul/f2uiz) for the
 *                      imperial displays, double (dmul/d2iz) for the x1.5 dual-pack range.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "odometer.h"

#include "report.h"
#include "vcu_state.h"
#include "vehicle.h" /* MF_ACTIVE, STANDBY_OFF_TICKS */

/* Loop periods / scaling. */
#define ODO_SUBCOUNT_TICKS 100     /* ticks per time-counter unit */
#define DIST_UNIT_PERIOD   360000  /* speed_src*ticks per distance count */
#define SPEED_SCALE        360     /* trip_dist*360/trip_ride_time factor */

/* Imperial display: the count goes through single-precision soft-float, `__aeabi_fdiv` by 10.0f
   (0x41200000), then `__aeabi_fmul` by the km -> mile factor 0.621f (0x3F1EF9DB), then
   `__aeabi_f2uiz`. The range display divides by 10 in integers first and only multiplies. */
#define ODO_DISP_DIV       10.0f
#define ODO_KM_TO_MI       0.621f /* 0x3F1EF9DB */
#define ODO_DISP_DIV_INT   10     /* range / 10 (sdiv) */
#define ODO_DUAL_PACK_NUM  3u     /* second pack: full-charge range x 3 / 2 */
#define ODO_DUAL_PACK_DEN  2u

/* Last trip distance the average speed was computed for (boot 0). */
static volatile int16_t g_avg_speed_dist;

/* Mileage pre-pass down-counter, owned by the odometer (the report cache lives in report.c). */
static volatile int16_t g_mileage_down_counter; /*!< standby-off down-counter */

/**
 * \brief           Float-to-unsigned conversion into a 16-bit display cell
 * \param[in]       miles: the value
 * \return          the magnitude, truncated
 * \note            It truncates the MAGNITUDE (sign bit cleared first), so a negative value
 *                      shows as positive.
 */
static int16_t prv_f2uiz_disp(float miles) {
    if (miles < 0.0f) {
        miles = -miles;
    }
    return (int16_t)(uint16_t)(uint32_t)miles;
}

/**
 * \brief           Imperial display value of a 0.1 km count, computed in single-precision soft-float
 * \param[in]       count: the count, already converted to float (ui2f / i2f as the caller's type)
 * \return          the displayed miles, truncated to the 16-bit display cell
 * \note            (count / 10.0f) * 0.621f, then \ref prv_f2uiz_disp.
 */
static int16_t prv_imperial_disp(float count) { return prv_f2uiz_disp((count / ODO_DISP_DIV) * ODO_KM_TO_MI); }

/* Distance integrator + range estimate. Integrates the speed (IN2 speed_src, every tick)    */
/* into g_state_block.dist_subacc and bumps the session/trip distance counts each            */
/* DIST_UNIT_PERIOD; picks the full-charge range from the (model, variant) table (g_vs[0],    */
/* g_vs[1]), x1.5 when a second pack is present; then remaining range = full-charge range x   */
/* SoC / 100 (clamped >= 0), written to the display scratch and to regs 0x5F / 0x5E (x0.8).   */
/* The distance display is its own step (\ref odometer_distance_display).                    */
static void odometer_range_compute(void) {
    int32_t dist;
    uint32_t range_full;
    int32_t soc;
    int32_t range;
    int32_t range_tenth;

    /* Distance integrator: accumulate speed per tick; each DIST_UNIT_PERIOD is one count. */
    dist = (int32_t)g_in2.f.speed_src_s + (int32_t)g_state_block.dist_subacc;
    g_state_block.dist_subacc = (uint32_t)dist;
    if (dist >= DIST_UNIT_PERIOD) {
        g_state_block.dist_subacc = (uint32_t)(dist - DIST_UNIT_PERIOD);
        if ((g_mode.mode_flags & MF_ACTIVE) != 0) {
            if (g_in2.f.session_dist != 0xFFFFFFFFu) {
                g_in2.f.session_dist = g_in2.f.session_dist + 1; /* session distance count (u32) */
            }
            if (g_in2.f.trip_dist < 0x7FFF) {
                g_in2.f.trip_dist = (int16_t)(g_in2.f.trip_dist + 1); /* trip distance count (s16) */
            }
        }
    }

    /* Full-charge range from the (model, variant) table. */
    {
        uint8_t m = g_vs.region;
        uint8_t v = g_vs.gear_sel;
        if (m == 0) {
            range_full = (v == 3 || v == 4) ? 4000u : 0x1CACu;
        } else if (m == 1) {
            range_full = (v == 3) ? 0x1D4Cu : (v == 4) ? 0x1C20u : 0x251Cu;
        } else if (m == 2) {
            range_full = (v == 3) ? 0x20D0u : (v == 4) ? 0x1D4Cu : 0x251Cu;
        } else {
            range_full = 0x1CACu;
        }
    }

    /* Second pack present (IN2 status_flags SF_DUAL_PACK): the range is 1.5 times larger. */
    if ((g_in2.f.status_flags & SF_DUAL_PACK) != 0) {
        range_full = (uint16_t)(((uint32_t)range_full * ODO_DUAL_PACK_NUM) / ODO_DUAL_PACK_DEN);
    }

    /* Remaining range = full-charge range * SoC / 100, clamped >= 0. */
    soc = (int32_t)g_in2.f.soc_s;
    range = (int32_t)(range_full * (uint32_t)soc) / 100;
    if (range < 0) {
        range = 0;
    }
    range_tenth = range / ODO_DISP_DIV_INT;
    if ((g_in2.f.cfg_flags & CFG_IMPERIAL) != 0) {
        g_scr.range_disp = prv_f2uiz_disp((float)range_tenth * ODO_KM_TO_MI); /* i2f, fmul, f2uiz */
    } else {
        g_scr.range_disp = (int16_t)range_tenth;
    }
    g_in2.f.range_est = (int16_t)range;
    g_in2.f.range_reserve = (int16_t)((range << 3) / 10); /* range * 0.8 */
}

/* Trip / session time counters. Every 100 ticks of the g_state_block odometer subcounter,   */
/* advances the trip (s16) and session (u32) total time counters and, while the active flag   */
/* (MODE->mode_flags bit2) is set, the trip and session ride time counters. All four count    */
/* the same 100-tick unit.                                                                    */
static void odometer_trip_tick(void) {
    int32_t sub;
    int32_t sess;

    sub = (int32_t)g_state_block.odo_subcnt + 1;
    g_state_block.odo_subcnt = (uint32_t)sub;
    if (sub >= ODO_SUBCOUNT_TICKS) {
        g_state_block.odo_subcnt = 0;
        if (g_in2.f.trip_total_time < 0x7FFF) {
            g_in2.f.trip_total_time = (int16_t)(g_in2.f.trip_total_time + 1); /* trip total time (s16) */
        }
        sess = (int32_t)g_in2.f.session_total_time;
        if (sess != -1) {
            g_in2.f.session_total_time = (uint32_t)(sess + 1); /* session total time (u32) */
        }
        if ((g_mode.mode_flags & MF_ACTIVE) != 0) {
            if (g_in2.f.trip_ride_time < 0x7FFF) {
                g_in2.f.trip_ride_time = (int16_t)(g_in2.f.trip_ride_time + 1); /* trip ride time (s16) */
            }
            sess = (int32_t)g_in2.f.session_ride_time;
            if (sess != -1) {
                g_in2.f.session_ride_time = (uint32_t)(sess + 1); /* session ride time (u32) */
            }
        }
    }
}

/* Trip average speed: with ride time, recompute trip_dist * 360 / trip_ride_time into        */
/* g_in2+0xDA only when trip_dist moved since the last computation (the product is 16 x 16    */
/* signed, smulbb); without ride time it is 0 and the last distance is kept.                  */
static void odometer_gauge_update(void) {
    if (g_in2.f.trip_ride_time > 0) {
        if (g_in2.f.trip_dist != g_avg_speed_dist) {
            g_in2.f.derived_speed = (int16_t)(((int32_t)g_in2.f.trip_dist * SPEED_SCALE)
                                              / (int32_t)g_in2.f.trip_ride_time);
        }
        g_avg_speed_dist = g_in2.f.trip_dist;
    } else {
        g_in2.f.derived_speed = 0;
    }
}

/* Distance display (the tail of odometer_power_tick): clears reg 0x3A, then shows the        */
/* session total (IN2 cfg_flags2 bit13) or the trip, metric (/10) or imperial (km -> mi in    */
/* single precision).                                                                         */
static void odometer_distance_display(void) {
    g_scr.signal_74 = 0; /* reg 0x3A, report signal [21] */
    if ((g_in2.f.cfg_flags2 & CFG2_SHOW_TOTAL) != 0) {
        g_scr.dist_is_trip = 0;
        if ((g_in2.f.cfg_flags & CFG_IMPERIAL) != 0) {
            g_scr.distance_disp = prv_imperial_disp((float)g_in2.f.session_dist); /* ui2f */
        } else {
            g_scr.distance_disp = (int16_t)(g_in2.f.session_dist / 10u);
        }
    } else {
        g_scr.dist_is_trip = 1;
        if ((g_in2.f.cfg_flags & CFG_IMPERIAL) != 0) {
            g_scr.distance_disp = prv_imperial_disp((float)(int32_t)g_in2.f.trip_dist); /* i2f */
        } else {
            g_scr.distance_disp = (int16_t)((int32_t)g_in2.f.trip_dist / 10);
        }
    }
}

/* Per-tick odo/range + speed update: range, time counters, average speed, then the distance */
/* display (the steps share no data, so their order is not observable). Called from the FSM   */
/* ride run handlers (states 3 and 7).                                                        */
void odometer_power_tick(void) {
    odometer_range_compute();
    odometer_trip_tick();
    odometer_gauge_update();
    odometer_distance_display();
}

/* Mileage report-on-change pre-pass: by the field g_vs+0x0E, reports arg 0x24 values */
/* 0x1E / 100 on change and maintains the standby-off down-counter. Called from the   */
/* FSM pre-pass.                                                                      */
void odometer_mileage_prepass(void) {
    if (g_vs.distance_code < 0x78) {
        report_onchange_24(0x1E);
        g_mileage_down_counter = STANDBY_OFF_TICKS;
        return;
    }
    if (g_vs.distance_code < 0xC9) {
        if (g_scr.mileage == 0) {
            report_onchange_24(100);
        }
    } else if (g_mileage_down_counter == 0) {
        report_onchange_24(100);
    } else {
        g_mileage_down_counter = (int16_t)(g_mileage_down_counter - 1);
    }
}
