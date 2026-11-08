/**
 * \file            test_harness.c
 * \brief           Self-tests of the on-target test support (tst_*.c) and worked examples
 * \note            One test per helper, so a suite author can copy the pattern: product
 *                      state reset, scheduler ticks, CAN RX injection (fake and through the
 *                      emulated controller), CAN TX capture, flash fakes.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "at32f415.h"
#include "board.h" /* SLOT_FAST */
#include "can.h"
#include "passcode.h" /* g_passcode_ch: a .data global */
#include "scheduler.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define CANID_ESC_D       0x344u      /* std frame whose bytes 0..1 land in g_in2.canrx_1e8_scratch */
#define SPARE_SECTOR      0x0801FC00u /* last 1 KB sector: erased, outside the firmware image */
#define CAN_FRAME_WAIT_US 1000u       /* > one frame at the configured bit rate */
#define TASK_PERIOD       2u

static uint32_t g_task_runs;

/**
 * \brief           Scheduler task that counts its runs
 */
static void prv_count_task(void) { ++g_task_runs; }

/**
 * \brief           Fresh boot state and fakes for every test
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    g_task_runs = 0u;
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/**
 * \brief           The reset puts .data globals back to their initialiser and zeroes .bss ones
 * \note            Run-time init values (vcu_usart_init's free-slot count) are not restored:
 *                      a suite re-runs the inits it needs after the reset.
 */
static void test_reset_restores_boot_values(void) {
    g_passcode_ch.reload[2] = 0u; /* .data, boot value 0x10 */
    g_vs.app_start_req = 1u;
    vcu_usart_init();
    TEST_ASSERT_EQUAL_UINT8(VCU_UART_SLOTS - 1u, g_uartctx.free);
    tst_reset_product_state();
    TEST_ASSERT_EQUAL_HEX16(0x0010u, g_passcode_ch.reload[2]);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.free);
}

/**
 * \brief           A task registered with period 2 first runs on tick 2, then every 2 ticks
 * \note            sched_dispatch fires on `tick - last >= period` with last = 0.
 */
static void test_sched_ticks_run_due_tasks(void) {
    sched_init();
    sched_register(SLOT_FAST, TASK_PERIOD, 0u, prv_count_task);
    tst_sched_run_ticks(TASK_PERIOD - 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_task_runs);
    tst_sched_run_ticks(1u);
    TEST_ASSERT_EQUAL_UINT32(1u, g_task_runs);
    tst_sched_run_ticks(3u * TASK_PERIOD);
    TEST_ASSERT_EQUAL_UINT32(4u, g_task_runs);
}

/**
 * \brief           An injected CAN frame goes through the RX ISR and the RX consumer
 */
static void test_can_rx_inject_reaches_decoder(void) {
    const uint8_t data[] = {0x34u, 0x12u};

    TEST_ASSERT_TRUE(tst_can_rx_inject(CANID_ESC_D, 0u, data, sizeof(data)));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_rx_deliver());
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_in2.canrx_1e8_scratch);
}

/**
 * \brief           A frame received by the emulated CAN1 raises RX0 and lands in the RX ring
 * \note            The path a device sees: bus -> FIFO0 -> CAN1_RX0_IRQHandler -> ring.
 */
static void test_emu_can_rx_through_controller(void) {
    const uint8_t data[] = {0x78u, 0x56u};

    vcu_can_init();
    vcu_can_lowlevel_init();
    tst_emu_can_rx(CANID_ESC_D, 0u, data, sizeof(data));
    tst_delay_us(CAN_FRAME_WAIT_US);
    while (vcu_can_rx_dispatch() != 0u) {
        /* drain */
    }
    TEST_ASSERT_EQUAL_HEX16(0x5678u, g_in2.canrx_1e8_scratch);
}

/**
 * \brief           CAN TX is captured at can_message_transmit; forwarding can be turned off
 */
static void test_can_tx_capture(void) {
    const uint8_t data[] = {0x01u, 0x01u, 0xDAu, 0x14u, 0x00u};
    const tst_can_frame_t* frame;

    tst_can_tx_forward(0u);
    vcu_can_init();
    vcu_can_send_ext(0x18EF0216u, data, sizeof(data));
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_UINT32(1u, __wrap_can_message_transmit_fake.call_count);
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_NOT_NULL(frame);
    TEST_ASSERT_EQUAL_UINT8(1u, frame->ext);
    TEST_ASSERT_EQUAL_HEX32(0x18EF0216u, frame->id);
    TEST_ASSERT_EQUAL_UINT8(sizeof(data), frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, frame->data, sizeof(data));
}

/**
 * \brief           Flash calls are recorded; forwarded they change the emulated flash, blocked not
 */
static void test_flash_fakes_record_and_block(void) {
    const volatile uint16_t* cell = (const volatile uint16_t*)SPARE_SECTOR;

    flash_unlock();
    TEST_ASSERT_EQUAL(FLASH_OPERATE_DONE, flash_sector_erase(SPARE_SECTOR));
    TEST_ASSERT_EQUAL(FLASH_OPERATE_DONE, flash_halfword_program(SPARE_SECTOR, 0x1234u));
    TEST_ASSERT_EQUAL_HEX16(0x1234u, cell[0]);
    tst_flash_forward(0u);
    TEST_ASSERT_EQUAL(FLASH_OPERATE_DONE, flash_halfword_program(SPARE_SECTOR + 2u, 0xABCDu));
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, cell[1]);
    flash_lock();
    TEST_ASSERT_EQUAL_UINT32(1u, __wrap_flash_sector_erase_fake.call_count);
    TEST_ASSERT_EQUAL_HEX32(SPARE_SECTOR, __wrap_flash_sector_erase_fake.arg0_val);
    TEST_ASSERT_EQUAL_UINT32(2u, __wrap_flash_halfword_program_fake.call_count);
    TEST_ASSERT_EQUAL_HEX32(SPARE_SECTOR + 2u, __wrap_flash_halfword_program_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_HEX16(0xABCDu, __wrap_flash_halfword_program_fake.arg1_history[1]);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_reset_restores_boot_values);
    RUN_TEST(test_sched_ticks_run_due_tasks);
    RUN_TEST(test_can_rx_inject_reaches_decoder);
    RUN_TEST(test_emu_can_rx_through_controller);
    RUN_TEST(test_can_tx_capture);
    RUN_TEST(test_flash_fakes_record_and_block);
    tst_exit(UNITY_END());
    return 0;
}
