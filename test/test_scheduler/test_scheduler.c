/**
 * \file            test_scheduler.c
 * \brief           Cooperative scheduler (scheduler.c) behaviour
 * \note            Covers the dispatcher (`tick - last >= period`), slot registration, the
 *                      slot-1 task and the task periods main registers. The tick is driven by
 *                      hand: the test image does not start TMR5.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "board.h"
#include "busproto.h"
#include "scheduler.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"

#define CALL_LOG_MAX 16u /*!< callback ids kept by the call log */

static uint8_t g_call_log[CALL_LOG_MAX];   /*!< slot ids in call order */
static uint32_t g_call_count;              /*!< callbacks run */
static uint32_t g_call_tick[CALL_LOG_MAX]; /*!< g_sched_tick at each callback */

/**
 * \brief           Record one callback run
 * \param[in]       slot: id of the task that ran
 */
static void prv_log(uint8_t slot) {
    if (g_call_count < CALL_LOG_MAX) {
        g_call_log[g_call_count] = slot;
        g_call_tick[g_call_count] = g_sched_tick;
    }
    ++g_call_count;
}

/** \brief Task callback of slot 0 */
static void prv_task0(void) { prv_log(0u); }

/** \brief Task callback of slot 1 */
static void prv_task1(void) { prv_log(1u); }

/** \brief Task callback of slot 2 */
static void prv_task2(void) { prv_log(2u); }

/** \brief Task callback of slot 3 */
static void prv_task3(void) { prv_log(3u); }

/** \brief Task callback of slot 4 */
static void prv_task4(void) { prv_log(4u); }

/**
 * \brief           Fresh boot state, empty slot table, tick 0, empty call log
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    sched_init();
    g_sched_tick = 0u;
    g_call_count = 0u;
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- slot table and dispatcher -------------------------------------------------------------- */

/**
 * \brief           A new slot starts with last = 0 and fires once `tick - last` reaches the period
 * \note            Registration stores 0 to `last`, not the current tick. A slot is skipped
 *                      while `tick - last < period`.
 */
static void test_register_seeds_last_zero_and_compare_is_inclusive(void) {
    sched_register(0u, 10u, 0u, prv_task0);

    tst_sched_run_ticks(9u); /* ticks 1..9: 9 - 0 < 10 */
    TEST_ASSERT_EQUAL_UINT32(0u, g_call_count);
    tst_sched_run_ticks(1u); /* tick 10 */
    TEST_ASSERT_EQUAL_UINT32(1u, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(10u, g_call_tick[0]);
}

/**
 * \brief           A periodic slot runs every `period` ticks and re-reads the tick for `last`
 * \note            A periodic slot (oneshot byte 0) stores the current tick into `last` before
 *                      the call, so period 10 runs at ticks 10, 20, 30.
 */
static void test_periodic_slot_runs_every_period(void) {
    sched_register(0u, 10u, 0u, prv_task0);

    tst_sched_run_ticks(33u);
    TEST_ASSERT_EQUAL_UINT32(3u, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(10u, g_call_tick[0]);
    TEST_ASSERT_EQUAL_UINT32(20u, g_call_tick[1]);
    TEST_ASSERT_EQUAL_UINT32(30u, g_call_tick[2]);
}

/**
 * \brief           Period 1 (the fast service slot) runs every tick
 * \note            main registers slot 3 with period 1; it runs whenever tick - last >= 1.
 */
static void test_period_one_runs_every_tick(void) {
    sched_register(3u, 1u, 0u, prv_task3);

    tst_sched_run_ticks(10u);
    TEST_ASSERT_EQUAL_UINT32(10u, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(1u, g_call_tick[0]);
    TEST_ASSERT_EQUAL_UINT32(10u, g_call_tick[9]);
}

/**
 * \brief           A one-shot slot runs once and is disabled before its callback
 * \note            oneshot byte != 0 clears the enable byte instead of updating `last`; the
 *                      callback still runs that pass, at tick 5 for period 5.
 */
static void test_oneshot_runs_once(void) {
    sched_register(2u, 5u, 1u, prv_task2);

    tst_sched_run_ticks(40u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_call_count);
    TEST_ASSERT_EQUAL_UINT32(5u, g_call_tick[0]);
}

/**
 * \brief           Due slots run in slot order 0..4 within one dispatch pass
 * \note            The dispatcher walks the 5 slots with an index 0..4.
 */
static void test_due_slots_run_in_slot_order(void) {
    static const uint8_t expect[5] = {0u, 1u, 2u, 3u, 4u};

    sched_register(4u, 3u, 0u, prv_task4);
    sched_register(2u, 3u, 0u, prv_task2);
    sched_register(0u, 3u, 0u, prv_task0);
    sched_register(3u, 3u, 0u, prv_task3);
    sched_register(1u, 3u, 0u, prv_task1);

    g_sched_tick = 4u;
    sched_dispatch();
    TEST_ASSERT_EQUAL_UINT32(5u, g_call_count);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, g_call_log, 5u);
}

/**
 * \brief           The elapsed time is an unsigned difference, so the tick may wrap
 * \note            An unsigned difference and an unsigned compare.
 */
static void test_tick_wraparound(void) {
    g_sched_tick = 0xFFFFFFF8u;
    sched_register(0u, 10u, 0u, prv_task0);
    sched_dispatch(); /* tick - 0 is huge: runs, last = 0xFFFFFFF8 */
    TEST_ASSERT_EQUAL_UINT32(1u, g_call_count);

    tst_sched_run_ticks(9u); /* 0x00000001 after the wrap: 9 elapsed, not due */
    TEST_ASSERT_EQUAL_UINT32(1u, g_call_count);
    tst_sched_run_ticks(1u); /* 0x00000002: 10 elapsed */
    TEST_ASSERT_EQUAL_UINT32(2u, g_call_count);
    TEST_ASSERT_EQUAL_HEX32(0x00000002u, g_call_tick[1]);
}

/**
 * \brief           A slot registered with no callback still advances without calling anything
 * \note            A non-null check guards the indirect call.
 */
static void test_null_callback_is_skipped(void) {
    sched_register(1u, 2u, 0u, 0);
    sched_register(0u, 2u, 0u, prv_task0);

    tst_sched_run_ticks(3u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_call_count);
    TEST_ASSERT_EQUAL_UINT8(0u, g_call_log[0]);
}

/**
 * \brief           sched_init leaves every slot disabled
 * \note            The table is .bss (all enable bytes 0) and only main registers slots;
 *                      sched_init makes that reset state explicit.
 */
static void test_init_disables_all_slots(void) {
    sched_register(0u, 1u, 0u, prv_task0);
    sched_init();

    tst_sched_run_ticks(10u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_call_count);
}

/**
 * \brief           The task periods are the raw counts main registers
 * \note            sched_register(0,10,..), (1,10,..), (2,10,..), (3,1,..), (4, 0x493E0 =
 *                      300000, ..), all oneshot 0, for the vehicle FSM, dispatch, CAN, fast and
 *                      slow tasks.
 */
static void test_main_task_periods(void) {
    TEST_ASSERT_EQUAL_UINT32(5u, SCHED_SLOTS);
    TEST_ASSERT_EQUAL_UINT32(10u, SCHED_PERIOD_TASK);
    TEST_ASSERT_EQUAL_UINT32(1u, SCHED_PERIOD_FAST);
    TEST_ASSERT_EQUAL_UINT32(300000u, SCHED_PERIOD_SLOW);
    TEST_ASSERT_EQUAL_UINT8(0u, SLOT_VEHICLE_FSM);
    TEST_ASSERT_EQUAL_UINT8(1u, SLOT_DISPATCH);
    TEST_ASSERT_EQUAL_UINT8(2u, SLOT_CAN);
    TEST_ASSERT_EQUAL_UINT8(3u, SLOT_FAST);
    TEST_ASSERT_EQUAL_UINT8(4u, SLOT_SLOW);
}

/* -- slot 1: dispatch / timeout task ---------------------------------------------------------- */

/**
 * \brief           Report period 0: the divider is held at 0
 * \note            report_period 0 -> divider stored 0.
 */
static void test_dispatch_tick_period_zero_holds_divider(void) {
    g_disp.divider = 7u;
    g_bp_node.report_period = 0u;

    sched_task_dispatch_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_disp.divider);
}

/**
 * \brief           The divider compares its pre-increment value x10 with the period: period 20
 *                      wraps on every third call
 * \note            due = (cnt * 5 << 1) & 0xFFFF from the value before the increment; on
 *                      `period == due` the divider is stored 0.
 */
static void test_dispatch_tick_divider_cadence(void) {
    g_bp_node.report_period = 20u;
    g_bp_node.cap[0] = 0u; /* no notify call */

    sched_task_dispatch_tick();
    TEST_ASSERT_EQUAL_UINT16(1u, g_disp.divider);
    sched_task_dispatch_tick();
    TEST_ASSERT_EQUAL_UINT16(2u, g_disp.divider);
    sched_task_dispatch_tick(); /* 2 * 10 == 20 */
    TEST_ASSERT_EQUAL_UINT16(0u, g_disp.divider);
}

/**
 * \brief           A period that is not a multiple of 10 never matches in the first cycles
 * \note            The compare is on exact equality with cnt*10.
 */
static void test_dispatch_tick_period_not_multiple_of_ten(void) {
    uint32_t idx;

    g_bp_node.report_period = 25u;
    for (idx = 0u; idx < 10u; ++idx) {
        sched_task_dispatch_tick();
    }
    TEST_ASSERT_EQUAL_UINT16(10u, g_disp.divider);
}

/**
 * \brief           The mode timeout counts down and on reaching 0 masks the status to bits 0..1
 * \note            VS+0x36 decremented when non-zero; at 0, `SCR+0x1AA &= 3`. A timeout already
 *                      at 0 leaves the status.
 */
static void test_dispatch_tick_mode_timeout(void) {
    g_vs.mode_timeout = 2u;
    g_scr.mode_status = 0xFFFFu;

    sched_task_dispatch_tick();
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.mode_timeout);
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, g_scr.mode_status);
    sched_task_dispatch_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.mode_timeout);
    TEST_ASSERT_EQUAL_HEX16(0x0003u, g_scr.mode_status);

    g_scr.mode_status = 0xFFFFu;
    sched_task_dispatch_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.mode_timeout);
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, g_scr.mode_status);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_register_seeds_last_zero_and_compare_is_inclusive);
    RUN_TEST(test_periodic_slot_runs_every_period);
    RUN_TEST(test_period_one_runs_every_tick);
    RUN_TEST(test_oneshot_runs_once);
    RUN_TEST(test_due_slots_run_in_slot_order);
    RUN_TEST(test_tick_wraparound);
    RUN_TEST(test_null_callback_is_skipped);
    RUN_TEST(test_init_disables_all_slots);
    RUN_TEST(test_main_task_periods);
    RUN_TEST(test_dispatch_tick_period_zero_holds_divider);
    RUN_TEST(test_dispatch_tick_divider_cadence);
    RUN_TEST(test_dispatch_tick_period_not_multiple_of_ten);
    RUN_TEST(test_dispatch_tick_mode_timeout);
    tst_exit(UNITY_END());
    return 0;
}
