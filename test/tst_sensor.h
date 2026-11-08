/**
 * \file            tst_sensor.h
 * \brief           SPI2 / IMU fakes at the Artery BSP boundary (GNU ld `--wrap`) for sensor tests
 * \note            `[env:emu-test]` wraps `spi_i2s_data_transmit`, `spi_i2s_data_receive` and
 *                      `spi_i2s_flag_get`. By default every call forwards to the real BSP, so the
 *                      emulated SPI2 and its LSM6DS3 model answer as usual; only SPI2 transmits
 *                      are logged. A test can then:
 *                        - read the MOSI byte log (\ref tst_spi_mosi_get), i.e. every command and
 *                          data byte the product clocked out;
 *                        - present a raw data block (\ref tst_imu_sample_set): the 14 bytes of a
 *                          burst read from OUT_TEMP_L (command 0xA0) come from it instead of the
 *                          emulated chip, so any accel/gyro/temperature input can be tested;
 *                        - script the WHO_AM_I answers (\ref tst_imu_whoami_set, command 0x8F);
 *                        - stall the bus (\ref tst_spi_stall): the status flags never set.
 *                      For a physical attitude or sample through the emulated chip itself, see
 *                      tst_emu_imu_set / tst_emu_imu_sample_set (tst_support.h).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_SENSOR_H
#define TST_SENSOR_H

#include <stdint.h>

#ifndef TST_SPI_MOSI_MAX
#define TST_SPI_MOSI_MAX 128u /*!< SPI2 transmit bytes kept (later ones are only counted) */
#endif
#define TST_IMU_WHOAMI_MAX 8u /*!< scripted WHO_AM_I answers (the last one repeats) */
#define TST_IMU_WORDS      7u /*!< words of the OUT_TEMP_L block: TEMP, GX, GY, GZ, AX, AY, AZ */

void tst_sensor_fakes_reset(void);
uint32_t tst_spi_mosi_count(void);
uint8_t tst_spi_mosi_get(uint32_t index);
void tst_spi_stall(uint8_t stall);
void tst_imu_sample_set(const int16_t words[TST_IMU_WORDS]);
void tst_imu_sample_clear(void);
void tst_imu_whoami_set(const uint8_t* answers, uint8_t count);
void tst_imu_level_sample(int16_t* words, float roll_deg, float pitch_deg);

#endif /* TST_SENSOR_H */
