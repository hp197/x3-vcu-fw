/**
 * \file            scheduler.h
 * \brief           Cooperative software-timer scheduler (no RTOS)
 */
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "at32f415.h"

typedef void (*sched_cb_t)(void);

/**
 * \brief           Scheduler time base: free-running TMR5 overflow count
 * \note            Incremented by TMR5_GLOBAL_IRQHandler (report.c) every 1.0 ms and read by
 *                      \ref sched_dispatch. TMR5 itself is started by vcu_periph_init.
 */
extern volatile uint32_t g_sched_tick;

void sched_init(void);
void sched_register(uint8_t id, uint32_t period_ticks, uint8_t oneshot, sched_cb_t cb);
void sched_dispatch(void);

/**
 * \brief           Periodic dispatch + timeout task
 * \note            Slot 1, period 10 ticks. Rate-divides an internal counter (g_disp.divider),
 *                      fires a registered state callback (vtable `[+0x0c]`) when due, and counts
 *                      down a timeout halfword in the vehicle-state struct (g_vs +0x36), masking
 *                      a status halfword (g_scr +0x1AA) to its low 2 bits on expiry. Acts as a
 *                      periodic notifier/heartbeat.
 */
void sched_task_dispatch_tick(void);

#endif /* SCHEDULER_H */
