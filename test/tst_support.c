/**
 * \file            tst_support.c
 * \brief           On-target test support: start-up, Unity output, exit, product state reset
 * \note            Unity prints through the Cortex-M ITM stimulus port 0 (CMSIS
 *                      `ITM_SendChar`); the run ends with an Arm semihosting exit
 *                      (`bkpt 0xAB`, SYS_EXIT_EXTENDED) whose status is the failure count.
 *                      x3emu (`python -m x3emu test <elf>`) models both; under a debugger
 *                      the same image would print over SWO and stop at the exit.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_support.h"
#include <string.h>
#include "at32f415.h"
#include "board.h"     /* APP_VECTOR_TABLE */
#include "clock.h"     /* clock_config */
#include "scheduler.h" /* g_sched_tick, sched_dispatch */

/** \brief ITM software lock key (ARMv7-M CoreSight LAR). */
#define TST_ITM_UNLOCK_KEY    0xC5ACCE55u
/** \brief ITM stimulus port used for the Unity console. */
#define TST_ITM_CONSOLE_PORT  0u
/** \brief Semihosting operation: exit with a reason and a subcode. */
#define TST_SYS_EXIT_EXTENDED 0x20u
/** \brief Semihosting exit reason: the application finished. */
#define TST_ADP_APP_EXIT      0x20026u
/** \brief x3emu semihosting extension: bytes arrive on USART2 RX. */
#define TST_X3EMU_USART_RX    0x100u
/** \brief x3emu semihosting extension: a frame arrives from the CAN bus. */
#define TST_X3EMU_CAN_RX      0x101u
/** \brief x3emu semihosting extension: an ADC1 channel input level (12-bit raw). */
#define TST_X3EMU_ADC_SET     0x102u
/** \brief x3emu semihosting extension: the IMU attitude and roll/pitch rates. */
#define TST_X3EMU_IMU_ATT     0x103u
/** \brief x3emu semihosting extension: the IMU physical sample in chip axes. */
#define TST_X3EMU_IMU_SAMPLE  0x104u
/** \brief ADC1 input channels the emulator accepts (ADC_CHANNEL_0..ADC_CHANNEL_17). */
#define TST_ADC_CHANNELS      18u
/** \brief Axes of the IMU accelerometer and gyroscope. */
#define TST_IMU_AXES          3u
/** \brief Flag of \ref TST_X3EMU_CAN_RX: extended identifier. */
#define TST_X3EMU_CAN_EXT     1u
/** \brief Classic CAN payload limit. */
#define TST_CAN_DLC_MAX       8u
/** \brief Hz per MHz: SystemCoreClock / this = core clocks per microsecond. */
#define TST_HZ_PER_MHZ        1000000u

/* Linker symbols: the startup's .data image and the product objects' state (emu-test.ld). */
extern uint8_t _sidata[];
extern uint8_t _sdata[];
extern uint8_t tst_product_data_start[];
extern uint8_t tst_product_data_end[];
extern uint8_t tst_product_bss_start[];
extern uint8_t tst_product_bss_end[];
extern uint8_t _ebss[];

/**
 * \brief           Issue an Arm semihosting call (`bkpt 0xAB`, r0 = operation, r1 = parameter)
 * \param[in]       operation: semihosting operation number
 * \param[in]       param: pointer to the operation's parameter block
 */
static void prv_semihost(uint32_t operation, const void* param) {
    register uint32_t reg_op __asm__("r0") = operation;
    register const void* reg_param __asm__("r1") = param;

    __asm__ volatile("bkpt 0xAB" : "+r"(reg_op) : "r"(reg_param) : "memory");
}

/**
 * \brief           Enable the ITM and its console port, as a debugger would for SWO
 * \note            DEMCR.TRCENA, the LAR unlock, TCR.ITMENA and TER bit 0: the conditions
 *                      CMSIS `ITM_SendChar` checks before it writes a character.
 */
static void prv_itm_enable(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR = TST_ITM_UNLOCK_KEY;
    ITM->TCR |= ITM_TCR_ITMENA_Msk;
    ITM->TER |= 1UL << TST_ITM_CONSOLE_PORT;
}

/**
 * \brief           Bring the test image up the way product `main` starts
 * \note            Selects the application vector table (BSP `SystemInit` points VTOR at
 *                      the bootloader's) and runs `clock_config` (144 MHz PLL), so product
 *                      code sees the clocks it expects. Call it first in the suite's `main`.
 */
void tst_init(void) {
    SCB->VTOR = APP_VECTOR_TABLE;
    clock_config();
    prv_itm_enable();
}

/**
 * \brief           End the run: report `failures` to the host and stop
 * \note            `tst_exit(UNITY_END())` is the last statement of every suite's `main`.
 *                      x3emu maps status 0 to exit code 0 and anything else to 1.
 * \param[in]       failures: Unity failure count (the exit status)
 */
void tst_exit(int32_t failures) {
    const uint32_t block[2] = {TST_ADP_APP_EXIT, (uint32_t)failures};

    prv_semihost(TST_SYS_EXIT_EXTENDED, block);
    for (;;) {
        /* no semihosting host: stay here */
    }
}

/**
 * \brief           Unity output start (UNITY_OUTPUT_START): make sure the ITM is enabled
 */
void tst_output_start(void) { prv_itm_enable(); }

/**
 * \brief           Unity character output (UNITY_OUTPUT_CHAR): one byte to ITM port 0
 * \param[in]       chr: character to print
 */
void tst_output_char(int32_t chr) { (void)ITM_SendChar((uint32_t)chr & 0xFFu); }

/**
 * \brief           Unity output flush (UNITY_OUTPUT_FLUSH): nothing is buffered
 */
void tst_output_flush(void) {}

/**
 * \brief           Put every product global and static back to its boot value
 * \note            Re-copies the product objects' .data from its flash image and zeroes
 *                      their .bss, i.e. what the C start-up did before `main`: the shared
 *                      blackboards (g_vs, g_in1, g_in2, g_scr, g_uartctx, ...) and every
 *                      module-private static. Unity's and the test support's own state is
 *                      linked outside that range (emu-test.ld) and is kept. Peripheral
 *                      registers and the NVIC are NOT reset, and nothing that a product
 *                      init sets up at run time (vcu_usart_init's free-slot count, ...) is
 *                      redone: call those inits again after the reset. Interrupts are masked
 *                      while the RAM is rewritten.
 */
void tst_reset_product_state(void) {
    uint32_t primask = __get_PRIMASK();
    uint32_t data_len = (uint32_t)(tst_product_data_end - tst_product_data_start);
    uint32_t bss_len = (uint32_t)(tst_product_bss_end - tst_product_bss_start);
    const uint8_t* data_image = _sidata + (tst_product_data_start - _sdata);

    __disable_irq();
    memcpy(tst_product_data_start, data_image, data_len);
    memset(tst_product_bss_start, 0, bss_len);
    if (primask == 0u) {
        __enable_irq();
    }
}

/**
 * \brief           Advance the scheduler time base and run the due tasks, one tick at a time
 * \note            Each step adds one TMR5 tick to `g_sched_tick` (what the TMR5 overflow ISR
 *                      does every 0.9 ms) and calls `sched_dispatch`, so tasks registered with
 *                      `sched_register` run exactly when they would on the device. TMR5 itself
 *                      need not run.
 * \param[in]       ticks: number of scheduler ticks to advance
 */
void tst_sched_run_ticks(uint32_t ticks) {
    uint32_t step;

    for (step = 0u; step < ticks; ++step) {
        g_sched_tick = g_sched_tick + 1u;
        sched_dispatch();
    }
}

/**
 * \brief           Busy-wait until any bit of `mask` is set in the register at `reg`
 * \param[in]       reg: register (or RAM word) to poll
 * \param[in]       mask: bits to wait for
 * \param[in]       max_loops: poll budget (\ref TST_WAIT_LOOPS_DEFAULT is ~10 ms at 144 MHz)
 * \return          `1` when a bit was seen set, `0` when the budget ran out
 */
uint8_t tst_wait_set(volatile const uint32_t* reg, uint32_t mask, uint32_t max_loops) {
    uint32_t loops = 0u;
    uint8_t seen = 0u;

    if (reg == NULL) {
        return 0u;
    }
    while ((seen == 0u) && (loops < max_loops)) {
        if ((*reg & mask) != 0u) {
            seen = 1u;
        }
        ++loops;
    }
    return seen;
}

/**
 * \brief           Busy-wait about `microseconds` of core time (SysTick is left alone)
 * \note            Counts SystemCoreClock / 1 MHz loop passes per microsecond; x3emu counts
 *                      one instruction per clock, so this is a lower bound of the emulated time.
 * \param[in]       microseconds: time to wait
 */
void tst_delay_us(uint32_t microseconds) {
    volatile uint32_t spins = (SystemCoreClock / TST_HZ_PER_MHZ) * microseconds / 4u;

    while (spins > 0u) {
        spins = spins - 1u;
    }
}

/**
 * \brief           Have bytes arrive on USART2 RX, from the bus side (x3emu only)
 * \note            x3emu semihosting extension 0x100: the emulator's USART2 model receives
 *                      the bytes at the configured baud rate (into the RX DMA ring once
 *                      `vcu_usart_init` has run). Give them time (\ref tst_delay_us) before
 *                      polling the product receiver. A debugger would reject the operation.
 * \param[in]       data: bytes to receive
 * \param[in]       len: number of bytes
 */
void tst_emu_usart_rx(const uint8_t* data, uint32_t len) {
    const uint32_t block[2] = {(uint32_t)(uintptr_t)data, len};

    if ((data != NULL) || (len == 0u)) {
        prv_semihost(TST_X3EMU_USART_RX, block);
    }
}

/**
 * \brief           Have a frame arrive from the CAN bus (x3emu only)
 * \note            x3emu semihosting extension 0x101: the emulator's CAN1 model receives
 *                      the frame into FIFO0 and raises the RX0 interrupt (CAN1 must be out
 *                      of sleep/freeze with the interrupt enabled, as after
 *                      `vcu_can_lowlevel_init`). For a direct, model-free path see
 *                      tst_can_rx_inject (tst_bsp_fakes.h).
 * \param[in]       id: identifier
 * \param[in]       ext: `1` for a 29-bit identifier
 * \param[in]       data: payload (`dlc` bytes)
 * \param[in]       dlc: payload length, 0..8
 */
void tst_emu_can_rx(uint32_t id, uint8_t ext, const uint8_t* data, uint8_t dlc) {
    uint32_t block[5] = {id, (ext != 0u) ? TST_X3EMU_CAN_EXT : 0u, dlc, 0u, 0u};

    if ((dlc <= TST_CAN_DLC_MAX) && ((data != NULL) || (dlc == 0u))) {
        if (dlc > 0u) {
            memcpy(&block[3], data, dlc);
        }
        prv_semihost(TST_X3EMU_CAN_RX, block);
    }
}

/**
 * \brief           Set the level of an ADC1 input channel (x3emu only)
 * \note            x3emu semihosting extension 0x102: from now on every conversion of
 *                      `channel` returns `raw`, so the running ordinary scan and its DMA
 *                      (vcu_control_init) deliver it with that channel's next conversion. The
 *                      level stays until it is set again (also across tests). A value above
 *                      0xFFF converts as 0xFFF (an input above VREF+).
 * \param[in]       channel: ADC1 input channel, 0..17 (`ADC_CHANNEL_*`)
 * \param[in]       raw: 12-bit conversion result
 */
void tst_emu_adc_set(uint8_t channel, uint16_t raw) {
    const uint32_t block[2] = {channel, raw};

    if (channel < TST_ADC_CHANNELS) {
        prv_semihost(TST_X3EMU_ADC_SET, block);
    }
}

/**
 * \brief           Present the board at an attitude to the emulated LSM6DS3 (x3emu only)
 * \note            x3emu semihosting extension 0x103, the scenario's `set_attitude`: gravity
 *                      for roll/pitch plus the given rotation rates, in the firmware's axes
 *                      (gravity along the board axes for that roll/pitch). The sample
 *                      is read on the product's next IMU burst read; the temperature is kept.
 * \param[in]       roll_mdeg: roll, millidegrees
 * \param[in]       pitch_mdeg: pitch, millidegrees
 * \param[in]       roll_rate_mdps: roll rate, millidegrees per second
 * \param[in]       pitch_rate_mdps: pitch rate, millidegrees per second
 */
void tst_emu_imu_set(int32_t roll_mdeg, int32_t pitch_mdeg, int32_t roll_rate_mdps, int32_t pitch_rate_mdps) {
    const int32_t block[4] = {roll_mdeg, pitch_mdeg, roll_rate_mdps, pitch_rate_mdps};

    prv_semihost(TST_X3EMU_IMU_ATT, block);
}

/**
 * \brief           Set the emulated LSM6DS3's physical sample in chip axes (x3emu only)
 * \note            x3emu semihosting extension 0x104, the scenario's `set_imu`. The chip
 *                      scales it with the full-scale range the product programmed.
 * \param[in]       accel_ug: X, Y, Z acceleration, micro-g (at rest and level: 0, 0, -1000000)
 * \param[in]       gyro_mdps: X, Y, Z rotation rate, millidegrees per second
 * \param[in]       temp_mdegc: die temperature, millidegrees Celsius
 */
void tst_emu_imu_sample_set(const int32_t accel_ug[3], const int32_t gyro_mdps[3], int32_t temp_mdegc) {
    int32_t block[(2u * TST_IMU_AXES) + 1u];
    uint32_t axis;

    if ((accel_ug != NULL) && (gyro_mdps != NULL)) {
        for (axis = 0u; axis < TST_IMU_AXES; ++axis) {
            block[axis] = accel_ug[axis];
            block[TST_IMU_AXES + axis] = gyro_mdps[axis];
        }
        block[2u * TST_IMU_AXES] = temp_mdegc;
        prv_semihost(TST_X3EMU_IMU_SAMPLE, block);
    }
}

/**
 * \brief           Fill the free stack below the caller with `pattern`
 * \note            Makes reads of uninitialised locals deterministic: product code called
 *                      next finds `pattern`, not whatever an earlier call left there, so a
 *                      stray stack read shows up in the result (e.g. a body byte sent from an
 *                      unset local). Paints from the end of .bss up to the current stack
 *                      pointer; nothing below the stack pointer is in use (AAPCS has no red
 *                      zone), and an interrupt simply overwrites part of the paint.
 * \param[in]       pattern: fill byte (pick one the expected result cannot contain)
 */
void tst_stack_paint(uint8_t pattern) {
    volatile uint8_t* cursor = _ebss;
    uint8_t* top = (uint8_t*)(uintptr_t)__get_MSP();

    while (cursor < top) {
        *cursor = pattern;
        ++cursor;
    }
}
