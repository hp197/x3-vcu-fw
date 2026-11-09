/**
 * \file            scheduler.c
 * \brief           Cooperative software-timer scheduler
 * \note            A 5-slot software-timer table: each enabled task runs when `tick - last >=
 *                      period`. The tick is \ref g_sched_tick, the free-running count the TMR5
 *                      overflow ISR bumps every 1.0 ms (report.c); there is no dedicated
 *                      scheduler timer.
 */
#include "scheduler.h"
#include "board.h"
#include "busproto.h" /* bp_state_notify + g_bp_node / g_bp_rptr */
#include "vcu_state.h"

/**
 * \brief           One software-timer slot (functional layout, not byte-exact)
 */
typedef struct {
    sched_cb_t cb;   /*!< task callback                 */
    uint32_t period; /*!< run interval, in ticks        */
    uint32_t last;   /*!< tick at the last run          */
    uint8_t oneshot; /*!< disable after the first run   */
    uint8_t enabled; /*!< slot is active                */
} sched_task_t;

static sched_task_t g_tasks[SCHED_SLOTS];

/**
 * \brief           Clear the task table
 * \note            The table is plain .bss; this only makes the reset state explicit.
 *                      The TMR5 tick (\ref g_sched_tick) is left running.
 */
void sched_init(void) {
    uint8_t i;

    for (i = 0; i < SCHED_SLOTS; ++i) {
        g_tasks[i].enabled = 0;
    }
}

/**
 * \brief           Register a periodic (or one-shot) task in a slot
 * \param[in]       id: slot index, `0 .. SCHED_SLOTS-1`
 * \param[in]       period_ticks: run interval in ticks
 * \param[in]       oneshot: non-zero to disable the slot after its first run
 * \param[in]       cb: task callback
 */
void sched_register(uint8_t id, uint32_t period_ticks, uint8_t oneshot, sched_cb_t cb) {
    if (id >= SCHED_SLOTS) {
        return;
    }
    g_tasks[id].cb = cb;
    g_tasks[id].period = period_ticks;
    g_tasks[id].last = 0; /* seeds last=0 (not the current tick) */
    g_tasks[id].oneshot = oneshot;
    g_tasks[id].enabled = 1;
}

/**
 * \brief           Run every due task once; call this each super-loop pass
 */
void sched_dispatch(void) {
    uint8_t i;
    uint32_t now;

    for (i = 0; i < SCHED_SLOTS; ++i) {
        if (!g_tasks[i].enabled) {
            continue;
        }
        /* fires once `tick - last` reaches the period; a periodic slot re-reads the tick for
           `last`, a one-shot slot only disables. */
        now = g_sched_tick;
        if ((now - g_tasks[i].last) >= g_tasks[i].period) {
            if (g_tasks[i].oneshot) {
                g_tasks[i].enabled = 0;
            } else {
                g_tasks[i].last = g_sched_tick;
            }
            if (g_tasks[i].cb != 0) {
                g_tasks[i].cb();
            }
        }
    }
}

/* bp_state_notify (mgr vtable +0x0C) lives in busproto.c (serialize + emit the 0xFE
   state frame). The call site below passes the report self/context cell. */

/* ========================================================================= */
/* slot 1 (period 10): periodic dispatch + timeout                           */
/* ========================================================================= */
void sched_task_dispatch_tick(void) {
    uint16_t cnt = g_disp.divider;
    uint16_t due = (uint16_t)((cnt * 5u) << 1); /* cnt*10, truncated to 16 bits */
    uint8_t period = g_bp_node.report_period;

    if (period == 0) {
        g_disp.divider = 0;
    } else {
        g_disp.divider = (uint16_t)(cnt + 1);
        if (period == due) {
            g_disp.divider = 0;
            if (g_bp_node.cap[0] != 0) {
                bp_state_notify((void*)g_bp_rptr.last_node_handle);
            }
        }
    }

    if (g_vs.mode_timeout != 0) {
        g_vs.mode_timeout = (uint16_t)(g_vs.mode_timeout - 1);
        if (g_vs.mode_timeout == 0) {
            g_scr.mode_status = (uint16_t)(g_scr.mode_status & 3);
        }
    }
}
