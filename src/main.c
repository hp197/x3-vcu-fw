/**
 * \file            main.c
 * \brief           x3 VCU application entry: clock, subsystem init, super-loop
 * \note            main brings up the clock, runs the subsystem inits (all peripherals via
 *                      vcu_periph_init), registers the five software-timer tasks, then spins
 *                      the cooperative super-loop: each pass runs the scheduler dispatch + the
 *                      fixed poll list, and the IWDG is fed once per 3 s ERTC wakeup.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "at32f415.h"
#include "board.h"
#include "busproto.h"
#include "can.h"
#include "clock.h"
#include "config.h"
#include "control.h"
#include "faultmon.h"
#include "ota.h"
#include "report.h"
#include "ride_assist.h"
#include "scheduler.h"
#include "sensor.h"
#include "usart.h"
#include "vcu_state.h" /* g_vs.ertc_wake (ERTC wakeup -> IWDG feed) */
#include "vehicle.h"

/**
 * \brief           Application entry: init everything, then run the super-loop
 * \return          Never returns
 */
int main(void) {
    /* Point VTOR at the application vector table first. The BSP SystemInit points VTOR at the
       bootloader's table (0x08000000), so the app table is selected here. */
    SCB->VTOR = APP_VECTOR_TABLE;

    clock_config(); /* HEXT 16 MHz -> PLL -> 144 MHz SYSCLK */

    /* One-shot init. */
    vcu_config_init();            /* identity record + stored configuration */
    (void)config_identity_sync(); /* identity/boot-descriptor sync (flash writes) */

    /* Software-only state (rings, vtables, monitor state); no peripheral is touched
       here. Done before vcu_periph_init so the CAN RX ring is ready before the CAN IRQs
       are enabled. */
    vcu_can_init();      /* CAN rings + candev vtable                     */
    vcu_ota_init();      /* OTA descriptor + session state                */
    vcu_faultmon_init(); /* fault-debounce + comms-watchdog monitor state */
    ride_assist_init();  /* ride-assist boot values                       */

    /* Every peripheral, once, in this order: SPI2, GPIO groups, buzzer, ADC, USART2,
       CAN1, TMR5, ERTC, IWDG. Starts TMR5 (the scheduler tick) and the IWDG (fed by the
       loop below on the ERTC wakeup). */
    vcu_periph_init();
    if (VCU_HAS_FEATURE(IMU)) {
        vcu_sensor_init(); /* LSM6DS3 6-DoF IMU probe + config (SPI2 already up) */
    }
    vcu_busproto_init(); /* J1939-style BMS/peripheral TP engine */

    sched_init();
    sched_register(SLOT_VEHICLE_FSM, SCHED_PERIOD_TASK, 0, vehicle_task_fsm);
    sched_register(SLOT_DISPATCH, SCHED_PERIOD_TASK, 0, sched_task_dispatch_tick);
    sched_register(SLOT_CAN, SCHED_PERIOD_TASK, 0, vcu_can_tick);
    sched_register(SLOT_FAST, SCHED_PERIOD_FAST, 0, bp_task_fast_service);
    sched_register(SLOT_SLOW, SCHED_PERIOD_SLOW, 0, report_task_slow);

    debug_periph_mode_set(DEBUG_DEEPSLEEP, FALSE); /* DEBUG_CTRL &= ~DEEPSLEEP */

    /*
     * Super-loop: scheduler dispatch + the fixed poll list every pass; the IWDG is
     * fed only when the 3 s ERTC wakeup ISR has raised g_vs.ertc_wake, which is then
     * cleared. The scheduler is paced by the TMR5 tick (g_sched_tick), not by this
     * loop.
     *
     * The subsystem services (sensor update, faultmon debounce/report, busproto TP)
     * are NOT driven from here. They run from:
     *   - vcu_sensor_attitude_update -> FSM pre-pass                   (vehicle.c)
     *   - fm_status_report           -> per-state FSM handlers         (report.c)
     *   - vcu_faultmon_tick          -> fault-detection aggregator     (vehicle.c)
     *   - TP session service         -> bp_task_fast_service, slot 3   (busproto.c)
     */
    for (;;) {
        do {
            sched_dispatch();         /* run due software-timer tasks */
            vcu_can_poll();           /* CAN TX pump                  */
            vcu_can_rx_dispatch();    /* CAN RX ring consumer         */
            vcu_usart_tx_sequencer(); /* USART reply-ring drain       */
            vcu_usart_poll();         /* RX pump (g_uartctx, DMA1 ch6) */
        } while (g_vs.ertc_wake == 0);

        wdt_counter_reload(); /* feed the IWDG on the 3 s ERTC wakeup */
        g_vs.ertc_wake = 0;
    }
}
