/**
 * \file            test_odometer.c
 * \brief           Odometer, trip/session counters and range estimate (odometer.c)
 * \note            Covers odometer_power_tick (range, time counters, average speed, distance
 *                      display) and the mileage pre-pass. The odometer keeps its counts in RAM
 *                      only (the IN2 register file); no flash is written.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <stddef.h>
#include <stdint.h>
#include <unity.h>
#include "odometer.h"
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"
#include "vehicle.h"

/* Blackboard offsets of the session distance (IN2) and the mode flags (MODE). */
#define IN2_SESSION_DIST_OFF 0xC4u
#define MODE_FLAGS_OFF       0x0Au

#define TICKS_PER_TIME_UNIT 100u    /* ticks per time-counter unit                       */
#define DIST_UNIT           360000u /* 0x57E40 speed counts per distance unit            */
#define SPEED_FACTOR        360     /* trip_dist * 0x168 / ride time                     */
#define S16_MAX             0x7FFF
#define U32_MAX             0xFFFFFFFFu
#define MILEAGE_LOW_CODE    120u  /* distance code < 120 -> report 30                    */
#define MILEAGE_HIGH_CODE   201u  /* distance code >= 201 -> down-counter band           */
#define MILEAGE_LOW_VALUE   30u
#define MILEAGE_HIGH_VALUE  100u
#define MILEAGE_RELOAD      300u /* down-counter reload value                           */
#define NB_ARG_MILEAGE      0x24u
#define SOC_FULL            100

/** \brief One (region, gear_sel) cell of the full-charge range table. */
typedef struct {
    uint8_t mode;  /*!< g_vs+0x00 */
    uint8_t gear;  /*!< g_vs+0x01 */
    uint16_t full; /*!< full-charge range, 0.1 km   */
    uint16_t dual; /*!< the same x 1.5 (two packs) */
} range_cell_t;

/* The range table and its x1.5 values (i2d, dmul 1.5 = 0x3FF80000, d2iz, uxth: 9500 -> 14250,
   7340 -> 11010, ...). */
static const range_cell_t RANGE_TABLE[] = {
    {0u, 0u, 0x1CACu, 11010u}, {0u, 2u, 0x1CACu, 11010u}, {0u, 3u, 4000u, 6000u},    {0u, 4u, 4000u, 6000u},
    {0u, 5u, 0x1CACu, 11010u}, {1u, 1u, 0x251Cu, 14250u}, {1u, 2u, 0x251Cu, 14250u}, {1u, 3u, 0x1D4Cu, 11250u},
    {1u, 4u, 0x1C20u, 10800u}, {2u, 0u, 0x251Cu, 14250u}, {2u, 2u, 0x251Cu, 14250u}, {2u, 3u, 0x20D0u, 12600u},
    {2u, 4u, 0x1D4Cu, 11250u}, {3u, 2u, 0x1CACu, 11010u}, {7u, 3u, 0x1CACu, 11010u},
};

/**
 * \brief           Fresh boot state, fakes and USART2 (the mileage reports go out on it)
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/**
 * \brief           Run the time-counter tick `count` times through the public per-tick update
 * \param[in]       count: number of odometer_power_tick calls
 */
static void prv_ticks(uint32_t count) {
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        odometer_power_tick();
    }
}

/**
 * \brief           Offset of a blackboard cell from its block, as an integer
 * \param[in]       cell: address of the field
 * \param[in]       base: address of the block
 * \return          byte offset
 */
static uint32_t prv_off(const volatile void* cell, const volatile void* base) {
    return (uint32_t)((uintptr_t)cell - (uintptr_t)base);
}

/* -- register map ------------------------------------------------------------------------- */

/**
 * \brief           The odometer cells sit at fixed blackboard offsets (they are Ninebot registers)
 * \note            IN2 (+0xAA SoC, +0xAE speed, +0xBC/+0xBE range, +0xC4 session distance,
 *                      +0xC8/+0xCC session times, +0xD0..+0xD4 trip, +0xDA speed), SCR (+0x16,
 *                      +0x18, +0x1E, +0x48), VS (+0x0E) and MODE flags (+0x0A). The register number
 *                      is offset / 2 (reg 0x62 = session distance). The two state-block counters
 *                      are internal, so their place is not pinned.
 */
static void test_cells_at_fixed_offsets(void) {
    TEST_ASSERT_EQUAL_HEX32(IN2_SESSION_DIST_OFF, prv_off(&g_in2.f.session_dist, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xAAu, prv_off(&g_in2.f.soc_s, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xAEu, prv_off(&g_in2.f.speed_src_s, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xBCu, prv_off(&g_in2.f.range_reserve, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xBEu, prv_off(&g_in2.f.range_est, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xC8u, prv_off(&g_in2.f.session_total_time, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xCCu, prv_off(&g_in2.f.session_ride_time, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xD0u, prv_off(&g_in2.f.trip_dist, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xD2u, prv_off(&g_in2.f.trip_total_time, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xD4u, prv_off(&g_in2.f.trip_ride_time, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0xDAu, prv_off(&g_in2.f.derived_speed, &g_in2));
    TEST_ASSERT_EQUAL_HEX32(0x16u, prv_off(&g_scr.dist_is_trip, &g_scr));
    TEST_ASSERT_EQUAL_HEX32(0x18u, prv_off(&g_scr.range_disp, &g_scr));
    TEST_ASSERT_EQUAL_HEX32(0x1Eu, prv_off(&g_scr.distance_disp, &g_scr));
    TEST_ASSERT_EQUAL_HEX32(0x48u, prv_off(&g_scr.mileage, &g_scr));
    TEST_ASSERT_EQUAL_HEX32(0x0Eu, prv_off(&g_vs.distance_code, &g_vs));
    TEST_ASSERT_EQUAL_HEX32(MODE_FLAGS_OFF, prv_off(&g_mode.mode_flags, &g_mode));
}

/* -- time counters ------------------------------------------------------------------------- */

/**
 * \brief           The time counters advance once per 100 ticks; ride times only while active
 * \note            The subcounter is incremented; at `n >= 100` it is zeroed and IN2+0xD2 (s16)
 *                      and IN2+0xC8 (u32) advance; IN2+0xD4 / +0xCC only when MODE+0x0A bit 2 is set.
 */
static void test_time_counters_every_100_ticks(void) {
    prv_ticks(TICKS_PER_TIME_UNIT - 1u);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.trip_total_time);
    TEST_ASSERT_EQUAL_UINT32(TICKS_PER_TIME_UNIT - 1u, g_state_block.odo_subcnt);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_state_block.odo_subcnt);
    TEST_ASSERT_EQUAL_INT16(1, g_in2.f.trip_total_time);
    TEST_ASSERT_EQUAL_UINT32(1u, g_in2.f.session_total_time);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.trip_ride_time);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_ride_time);

    g_mode.mode_flags = MF_ACTIVE;
    prv_ticks(TICKS_PER_TIME_UNIT);
    TEST_ASSERT_EQUAL_INT16(2, g_in2.f.trip_total_time);
    TEST_ASSERT_EQUAL_UINT32(2u, g_in2.f.session_total_time);
    TEST_ASSERT_EQUAL_INT16(1, g_in2.f.trip_ride_time);
    TEST_ASSERT_EQUAL_UINT32(1u, g_in2.f.session_ride_time);
}

/**
 * \brief           The time counters saturate: trip at 0x7FFF (s16), session at 0xFFFFFFFF (u32)
 * \note            Guards: trip `< 0x7fff` and session `!= -1`, the same pair for the ride times.
 */
static void test_time_counters_saturate(void) {
    g_mode.mode_flags = MF_ACTIVE;
    g_in2.f.trip_total_time = S16_MAX;
    g_in2.f.trip_ride_time = S16_MAX;
    g_in2.f.session_total_time = U32_MAX;
    g_in2.f.session_ride_time = U32_MAX - 1u;
    g_state_block.odo_subcnt = TICKS_PER_TIME_UNIT - 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(S16_MAX, g_in2.f.trip_total_time);
    TEST_ASSERT_EQUAL_INT16(S16_MAX, g_in2.f.trip_ride_time);
    TEST_ASSERT_EQUAL_HEX32(U32_MAX, g_in2.f.session_total_time);
    TEST_ASSERT_EQUAL_HEX32(U32_MAX, g_in2.f.session_ride_time);
}

/* -- distance integrator ------------------------------------------------------------------- */

/**
 * \brief           Speed is summed every tick; each 360000 counts one distance unit while active
 * \note            `acc = (short)speed + acc`; at `0x57E40 <= acc` acc -= 0x57E40 and, with
 *                      MODE+0x0A bit 2, IN2+0xC4 (u32) and IN2+0xD0 (s16) advance by one. The
 *                      remainder is kept.
 */
static void test_distance_counts_per_360000(void) {
    g_mode.mode_flags = MF_ACTIVE;
    g_in2.f.speed_src_s = 1000;
    g_state_block.dist_subacc = DIST_UNIT - 1500u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT32(DIST_UNIT - 500u, g_state_block.dist_subacc);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_dist);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT32(500u, g_state_block.dist_subacc);
    TEST_ASSERT_EQUAL_UINT32(1u, g_in2.f.session_dist);
    TEST_ASSERT_EQUAL_INT16(1, g_in2.f.trip_dist);
}

/**
 * \brief           While not active the integrator still wraps, but no distance is counted
 * \note            The bit-2 test sits inside the wrap branch.
 */
static void test_distance_not_counted_when_inactive(void) {
    g_in2.f.speed_src_s = 1000;
    g_state_block.dist_subacc = DIST_UNIT - 1000u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_state_block.dist_subacc);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_dist);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.trip_dist);
}

/**
 * \brief           A negative speed (signed IN2+0xAE) runs the integrator backwards
 * \note            The speed is read as signed 16-bit and added to the int sum.
 */
static void test_distance_negative_speed_subtracts(void) {
    g_mode.mode_flags = MF_ACTIVE;
    g_in2.f.speed_src_s = -100;
    g_state_block.dist_subacc = 50u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT32(-50, (int32_t)g_state_block.dist_subacc);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_dist);
}

/**
 * \brief           The distance counts saturate: session at 0xFFFFFFFF, trip at 0x7FFF
 * \note            `!= 0xffffffff` and `< 0x7fff` guards.
 */
static void test_distance_saturates(void) {
    g_mode.mode_flags = MF_ACTIVE;
    g_in2.f.speed_src_s = 1000;
    g_in2.f.session_dist = U32_MAX;
    g_in2.f.trip_dist = S16_MAX;
    g_state_block.dist_subacc = DIST_UNIT - 1000u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX32(U32_MAX, g_in2.f.session_dist);
    TEST_ASSERT_EQUAL_INT16(S16_MAX, g_in2.f.trip_dist);
}

/* -- distance display ---------------------------------------------------------------------- */

/**
 * \brief           Metric display: session total (cfg2 bit 13) / 10 unsigned, else trip / 10 signed
 * \note            Bit 13 of IN2+0x3C -> SCR+0x16 = 0 and SCR+0x1E = `udiv` session/10; else
 *                      SCR+0x16 = 1 and SCR+0x1E = `sdiv` trip/10.
 */
static void test_distance_display_metric(void) {
    g_in2.f.cfg_flags2 = CFG2_SHOW_TOTAL;
    g_in2.f.session_dist = 123456u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.dist_is_trip);
    TEST_ASSERT_EQUAL_INT16(12345, g_scr.distance_disp);

    g_in2.f.cfg_flags2 = 0u;
    g_in2.f.trip_dist = -15;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.dist_is_trip);
    TEST_ASSERT_EQUAL_INT16(-1, g_scr.distance_disp);
}

/**
 * \brief           Imperial display uses single-precision (x / 10.0f) * 0.621f, truncated
 * \note            ui2f (session) / i2f (trip), `__aeabi_fdiv` by 10.0f (0x41200000), `__aeabi_fmul`
 *                      by 0.621f (0x3F1EF9DB), `__aeabi_f2uiz`. 692 -> 69.2f * 0.621f = 42.97 -> 42.
 */
static void test_distance_display_imperial_single_precision(void) {
    g_in2.f.cfg_flags = CFG_IMPERIAL;
    g_in2.f.cfg_flags2 = CFG2_SHOW_TOTAL;
    g_in2.f.session_dist = 692u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(42, g_scr.distance_disp);
    g_in2.f.session_dist = 1000000u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(62100, (uint16_t)g_scr.distance_disp);

    g_in2.f.cfg_flags2 = 0u;
    g_in2.f.trip_dist = 1207;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(74, g_scr.distance_disp);
}

/**
 * \brief           Imperial trip display of a negative count shows its magnitude
 * \note            The conversion drops the sign, so -69.2f * 0.621f -> 42, not a negative or
 *                      zero value.
 */
static void test_distance_display_imperial_negative_trip(void) {
    g_in2.f.cfg_flags = CFG_IMPERIAL;
    g_in2.f.trip_dist = -692;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(42, g_scr.distance_disp);
}

/**
 * \brief           The distance display clears SCR+0x74 (Ninebot page-1 reg 0x3A) every tick
 * \note            SCR+0x74 is written 0 before the display.
 */
static void test_distance_display_clears_reg_3a(void) {
    g_scr.reg[0x3Au] = 0x1234u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.reg[0x3Au]);
}

/* -- range estimate ------------------------------------------------------------------------ */

/**
 * \brief           Full-charge range per (ride mode, gear) with SoC 100: est, x0.8 reserve, /10 display
 * \note            The table, `range = full * soc / 100`,
 *                      IN2+0xBE = range, IN2+0xBC = (range << 3) / 10, SCR+0x18 = range / 10.
 */
static void test_range_table_single_pack(void) {
    uint32_t idx;

    g_in2.f.soc_s = SOC_FULL;
    for (idx = 0u; idx < (sizeof(RANGE_TABLE) / sizeof(RANGE_TABLE[0])); ++idx) {
        const range_cell_t* cell = &RANGE_TABLE[idx];

        g_vs.region = cell->mode;
        g_vs.gear_sel = cell->gear;
        prv_ticks(1u);
        TEST_ASSERT_EQUAL_INT16_MESSAGE((int16_t)cell->full, g_in2.f.range_est, "range_est");
        TEST_ASSERT_EQUAL_INT16_MESSAGE((int16_t)((cell->full * 8u) / 10u), g_in2.f.range_reserve, "reserve");
        TEST_ASSERT_EQUAL_INT16_MESSAGE((int16_t)(cell->full / 10u), g_scr.range_disp, "range_disp");
    }
}

/**
 * \brief           A second pack (IN2+0x38 bit 9) scales the full-charge range by 1.5
 * \note            int->double, `__aeabi_dmul` by 1.5 (0x3FF80000), `__aeabi_d2iz`, `uxth`.
 */
static void test_range_table_dual_pack_x15(void) {
    uint32_t idx;

    g_in2.f.soc_s = SOC_FULL;
    g_in2.f.status_flags = SF_DUAL_PACK;
    for (idx = 0u; idx < (sizeof(RANGE_TABLE) / sizeof(RANGE_TABLE[0])); ++idx) {
        const range_cell_t* cell = &RANGE_TABLE[idx];

        g_vs.region = cell->mode;
        g_vs.gear_sel = cell->gear;
        prv_ticks(1u);
        TEST_ASSERT_EQUAL_INT16_MESSAGE((int16_t)cell->dual, g_in2.f.range_est, "range_est x1.5");
    }
}

/**
 * \brief           Range scales with SoC (integer, truncating) and is clamped at 0 for a negative SoC
 * \note            `(int)(full * (int)soc) / 100`, `if (r < 0) r = 0`.
 */
static void test_range_soc_scaling_and_clamp(void) {
    g_vs.region = 1u;
    g_vs.gear_sel = 2u; /* 9500 */
    g_in2.f.soc_s = 37;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(3515, g_in2.f.range_est);
    TEST_ASSERT_EQUAL_INT16(2812, g_in2.f.range_reserve);
    TEST_ASSERT_EQUAL_INT16(351, g_scr.range_disp);

    g_in2.f.soc_s = -5;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.range_est);
    TEST_ASSERT_EQUAL_INT16(0, g_in2.f.range_reserve);
    TEST_ASSERT_EQUAL_INT16(0, g_scr.range_disp);
}

/**
 * \brief           Imperial range display: (float)(range / 10) * 0.621f, truncated; est stays metric
 * \note            `sdiv` range / 10 first, then i2f, fmul 0.621f (0x3F1EF9DB), f2uiz to SCR+0x18.
 *                      3515 -> 351 * 0.621f = 217.97 -> 217; 6920 (4000 x SoC 173 / 100) ->
 *                      692 * 0.621f = 429.73 -> 429.
 */
static void test_range_display_imperial(void) {
    g_in2.f.cfg_flags = CFG_IMPERIAL;
    g_vs.region = 1u;
    g_vs.gear_sel = 2u;
    g_in2.f.soc_s = 37;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(3515, g_in2.f.range_est);
    TEST_ASSERT_EQUAL_INT16(217, g_scr.range_disp);

    g_vs.region = 0u;
    g_vs.gear_sel = 3u; /* 4000 */
    g_in2.f.soc_s = 173;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(6920, g_in2.f.range_est);
    TEST_ASSERT_EQUAL_INT16(429, g_scr.range_disp);
}

/* -- trip average speed -------------------------------------------------------------------- */

/**
 * \brief           Trip average speed = trip_dist * 360 / trip_ride_time, 0 without ride time;
 *                      recomputed only when trip_dist changed
 * \note            `if (0 < ride) { if (dist != last) +0xDA = (dist * 0x168) / ride; last = dist; }
 *                      else +0xDA = 0` (last boots at 0). A new ride time with the same distance
 *                      keeps the old value; after a 0 the value stays 0 until the distance moves.
 */
static void test_trip_average_speed(void) {
    g_in2.f.trip_dist = 50;
    g_in2.f.trip_ride_time = 7;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)((50 * SPEED_FACTOR) / 7), g_in2.f.derived_speed);
    g_in2.f.trip_ride_time = 10;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)((50 * SPEED_FACTOR) / 7), g_in2.f.derived_speed);
    g_in2.f.trip_dist = 51;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)((51 * SPEED_FACTOR) / 10), g_in2.f.derived_speed);
    g_in2.f.trip_ride_time = 0;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_in2.f.derived_speed);
    g_in2.f.trip_ride_time = 10;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_in2.f.derived_speed);

    g_in2.f.trip_ride_time = 0;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_in2.f.derived_speed);

    g_in2.f.derived_speed = 0x1234u;
    g_in2.f.trip_ride_time = -3;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_in2.f.derived_speed);
}

/* -- mileage pre-pass ---------------------------------------------------------------------- */

/**
 * \brief           Below distance code 0x78 the mileage 0x1E is reported (arg 0x24) once
 * \note            `if (code < 0x78) { report(0x1E); counter = 300; }`. The report goes out on
 *                      change only: the second pass sends nothing.
 */
static void test_mileage_low_band_reports_0x1e(void) {
    tst_nb_frame_t frame;
    const tst_usart_frame_t* raw;

    g_vs.distance_code = MILEAGE_LOW_CODE - 1u;
    odometer_mileage_prepass();
    odometer_mileage_prepass();
    TEST_ASSERT_EQUAL_INT16(MILEAGE_LOW_VALUE, g_scr.mileage);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    raw = tst_usart_tx_get(0u);
    TEST_ASSERT_NOT_NULL(raw);
    TEST_ASSERT_TRUE(tst_nb_parse(raw->bytes, raw->len, &frame));
    TEST_ASSERT_EQUAL_HEX8(NB_ADDR_APP, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(NB_CMD_WRITE3, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(NB_ARG_MILEAGE, frame.arg);
    TEST_ASSERT_EQUAL_UINT8(2u, frame.len);
    TEST_ASSERT_EQUAL_HEX8(MILEAGE_LOW_VALUE, frame.body[0]);
    TEST_ASSERT_EQUAL_HEX8(0u, frame.body[1]);
}

/**
 * \brief           In the middle band (0x78..0xC8) 100 is reported only while SCR+0x48 is 0
 * \note            `if (code < 0xC9) { if (*(short*)(SCR+0x48) == 0) report(100); }`.
 */
static void test_mileage_mid_band_only_when_zero(void) {
    g_vs.distance_code = MILEAGE_LOW_CODE;
    g_scr.mileage = 7;
    odometer_mileage_prepass();
    TEST_ASSERT_EQUAL_INT16(7, g_scr.mileage);

    g_vs.distance_code = MILEAGE_HIGH_CODE - 1u;
    g_scr.mileage = 0;
    odometer_mileage_prepass();
    TEST_ASSERT_EQUAL_INT16(MILEAGE_HIGH_VALUE, g_scr.mileage);
}

/**
 * \brief           From 0xC9 on, 100 is reported when the down-counter (boot 0, reload 300) is 0
 * \note            The counter (boot 0) is tested and decremented when non-zero. After a
 *                      low-band pass (reload 300) it takes 300 high-band passes before the report.
 */
static void test_mileage_high_band_down_counter(void) {
    uint32_t pass;

    g_vs.distance_code = MILEAGE_HIGH_CODE;
    odometer_mileage_prepass(); /* boot value 0: reports at once */
    TEST_ASSERT_EQUAL_INT16(MILEAGE_HIGH_VALUE, g_scr.mileage);

    g_vs.distance_code = 0u;
    odometer_mileage_prepass(); /* reload 300, report 0x1E */
    TEST_ASSERT_EQUAL_INT16(MILEAGE_LOW_VALUE, g_scr.mileage);

    g_vs.distance_code = MILEAGE_HIGH_CODE;
    for (pass = 0u; pass < MILEAGE_RELOAD; ++pass) {
        odometer_mileage_prepass();
    }
    TEST_ASSERT_EQUAL_INT16(MILEAGE_LOW_VALUE, g_scr.mileage);
    odometer_mileage_prepass();
    TEST_ASSERT_EQUAL_INT16(MILEAGE_HIGH_VALUE, g_scr.mileage);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_cells_at_fixed_offsets);
    RUN_TEST(test_time_counters_every_100_ticks);
    RUN_TEST(test_time_counters_saturate);
    RUN_TEST(test_distance_counts_per_360000);
    RUN_TEST(test_distance_not_counted_when_inactive);
    RUN_TEST(test_distance_negative_speed_subtracts);
    RUN_TEST(test_distance_saturates);
    RUN_TEST(test_distance_display_metric);
    RUN_TEST(test_distance_display_imperial_single_precision);
    RUN_TEST(test_distance_display_imperial_negative_trip);
    RUN_TEST(test_distance_display_clears_reg_3a);
    RUN_TEST(test_range_table_single_pack);
    RUN_TEST(test_range_table_dual_pack_x15);
    RUN_TEST(test_range_soc_scaling_and_clamp);
    RUN_TEST(test_range_display_imperial);
    RUN_TEST(test_trip_average_speed);
    RUN_TEST(test_mileage_low_band_reports_0x1e);
    RUN_TEST(test_mileage_mid_band_only_when_zero);
    RUN_TEST(test_mileage_high_band_down_counter);
    tst_exit(UNITY_END());
    return 0;
}
