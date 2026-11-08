/**
 * \file            tst_support.h
 * \brief           On-target test support: start-up, Unity output, exit, product state reset
 * \note            Shared by every suite under test/ (PlatformIO compiles the test root files
 *                      into each test image). See test/README.md for the suite template.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_SUPPORT_H
#define TST_SUPPORT_H

#include <stdint.h>

/** \brief Loop budget of \ref tst_wait_set: ~10 ms of emulated time at 144 MHz. */
#define TST_WAIT_LOOPS_DEFAULT 300000u

void tst_init(void);
void tst_exit(int32_t failures);
void tst_reset_product_state(void);
void tst_sched_run_ticks(uint32_t ticks);
uint8_t tst_wait_set(volatile const uint32_t* reg, uint32_t mask, uint32_t max_loops);
void tst_delay_us(uint32_t microseconds);
void tst_stack_paint(uint8_t pattern);
void tst_emu_usart_rx(const uint8_t* data, uint32_t len);
void tst_emu_can_rx(uint32_t id, uint8_t ext, const uint8_t* data, uint8_t dlc);
void tst_emu_adc_set(uint8_t channel, uint16_t raw);
void tst_emu_imu_set(int32_t roll_mdeg, int32_t pitch_mdeg, int32_t roll_rate_mdps, int32_t pitch_rate_mdps);
void tst_emu_imu_sample_set(const int32_t accel_ug[3], const int32_t gyro_mdps[3], int32_t temp_mdegc);

#endif /* TST_SUPPORT_H */
