/**
 * \file            sensor.h
 * \brief           x3 VCU on-board IMU (6-axis accel+gyro) over SPI2 + shared SPI2 bus primitives
 * \note            Reads the on-board 6-axis IMU every FSM pass, compensates the accelerometer
 *                      for speed, moving-averages it and runs a Mahony filter that yields the
 *                      gravity direction, the quaternion and the tilt angle.
 *
 *                      Module layout:
 *                        vcu_sensor_bus_init        (pin mux + SPI2 + CS release)
 *                        vcu_sensor_bus_gpio_init   (SPI2 remap, CS pins)
 *                        vcu_sensor_spi_init        (SPI2 master mode 3 /8)
 *                        vcu_spi2_xfer              (1-byte SPI2 xfer + retry)
 *                        vcu_spi2_bus_select        (PA15/PF7 chip-select mux)
 *                        imu_reg_read               (1-byte, cmd | 0x80)
 *                        imu_reg_write              (1-byte, cmd & ~0x80)
 *                        imu_burst_read             (N-byte, cmd | 0x80)
 *                        imu_burst_write            (N-byte, cmd & ~0x80)
 *                        vcu_sensor_init            (clear, seeds, identity probe)
 *                        imu_state_clear
 *                        imu_identify               (CTRL readback compare)
 *                        imu_profile_apply          (CTRL1..4 profile burst)
 *                        imu_read_raw               (14-byte burst + axis remap)
 *                        imu_sample_update          (compensate, IIR, scale)
 *                        imu_accel_compensate       (speed terms, clamp, averages)
 *                        moving_avg_push            (100-sample integer mean)
 *                        imu_mahony_update          (Mahony filter + tilt, tilt average)
 *                        fast_inv_sqrt              (0x5F3759DF + one Newton step)
 *                        imu_motion_metric_update   (|a - a 6 passes ago| * 10000)
 *                        vcu_sensor_attitude_update (pass + inlined tilt class)
 *                        atan2f, sqrtf              (Arm mathlib algorithms)
 *
 *                      IMU identity: WHO_AM_I register 0x0F returns 0x69 ('i') or 0x6A ('j');
 *                      the data block starts at OUT_TEMP_L (0x20) with 14 bytes (TEMP, then
 *                      OUTX/Y/Z_G, then OUTX/Y/Z_XL); control registers at 0x10..0x19. That
 *                      register map is the ST LSM6DS3 (0x69) / LSM6DSL/DSM (0x6A) family.
 *                      Profile {0x51, 0x7C, 0x64, 0x04} = accel 208 Hz +-2 g (2^-14 g/LSB),
 *                      gyro 833 Hz 2000 dps (70 mdps/LSB).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef SENSOR_H
#define SENSOR_H

#include <stddef.h>
#include "at32f415.h"

/* ------------------------------------------------------------------------- */
/* ST LSM6DS3 / LSM6DSL register map. The part family follows from WHO_AM_I   */
/* (0x69/0x6A) + this map; register NAMES follow the ST LSM6DS3 datasheet.     */
/* ------------------------------------------------------------------------- */
typedef enum {
    LSM6DS3_WHO_AM_I = 0x0F,    /*!< identity: 0x69 ('i') / 0x6A ('j')              */
    LSM6DS3_CTRL1_XL = 0x10,    /*!< accel ODR/range; base of the CTRL1..4 burst    */
    LSM6DS3_CTRL3_C = 0x12,     /*!< common control (BDU / auto-increment)          */
    LSM6DS3_CTRL8_XL = 0x17,    /*!< accel filtering                                 */
    LSM6DS3_CTRL9_XL = 0x18,    /*!< accel axis enable (verify reads back 0x38)     */
    LSM6DS3_CTRL10_C = 0x19,    /*!< gyro axis enable (verify reads back 0x38)      */
    LSM6DS3_OUT_TEMP_L = 0x20,  /*!< data block start: TEMP + 3+3 axis words (14 B) */
    LSM6DS3_TAP_CFG = 0x58,     /*!< embedded-function / tap config                 */
    LSM6DS3_TAP_THS_6D = 0x59,  /*!< tap threshold / 6D (written at init)           */
    LSM6DS3_WAKE_UP_THS = 0x5B, /*!< wake-up threshold                              */
    LSM6DS3_WAKE_UP_DUR = 0x5C, /*!< wake-up / sleep duration                       */
    LSM6DS3_MD1_CFG = 0x5E      /*!< INT1 function routing                          */
} lsm6ds3_reg_t;

/**
 * \brief           Attitude outputs of the Mahony filter
 * \note            `grav` is the gravity direction the quaternion predicts, in the body frame
 *                      (written before the quaternion step). The tilt is the angle of gravity
 *                      in the body X/Z plane: atan2f(grav x, grav z) in degrees + 180, wrapped
 *                      from (180, 360] to (-180, 0]; a board with az = -1 g reads 0.
 */
typedef struct {
    float grav[3];       /*!< +0x00 predicted gravity x, y, z                         */
    float tilt_deg;      /*!< +0x0C tilt of the last update (deg)                     */
    float grav_x_norm;   /*!< +0x10 grav x / |(grav x, grav z)| (no reader found)     */
    float tilt_mean_deg; /*!< +0x14 mean of the last <= 100 tilts with |tilt| < 179   */
} imu_attitude_t;

/**
 * \brief           Probe, configure and verify the on-board IMU
 * \note            Clears the IMU state, seeds the temperature IIR (250), then polls
 *                      WHO_AM_I (reg 0x0F) up to twice for 0x6A ('j'),
 *                      writes the CTRL registers (0x12/0x18/0x19/0x59/0x17), runs the ODR/range
 *                      profile (reg 0x10 burst) and the identity probe. The probe counter is
 *                      never reset. main discards the result, so this returns nothing.
 */
void vcu_sensor_init(void);

/**
 * \brief           IMU SPI2 bus bring-up
 * \note            SPI2 remapped to PB3/PB4/PB5 with PA15/PF7 as GPIO chip selects, SPI2
 *                      master mode 3 /8 software-CS, then both chips deselected. Called once
 *                      from vcu_periph_init, before vcu_sensor_init probes the IMU.
 */
void vcu_sensor_bus_init(void);

/**
 * \brief           FSM pre-pass: read the IMU and update the attitude
 * \note            One sample (read, speed compensation, moving average, IIR), one Mahony
 *                      update, |a| and the motion metric (g_vs.sensor_delta_f), then, when
 *                      IN2+0x3E bit 8 is set, the tilt class g_vs.tilt_latch from the averaged
 *                      tilt. Skipped by the FSM in state 5.
 * \return          0, or the IMU bus error (the caller ignores it)
 */
int32_t vcu_sensor_attitude_update(void);

/**
 * \brief           Attitude outputs of the last pass
 * \return          Pointer to the attitude block
 */
const imu_attitude_t* vcu_sensor_attitude(void);

/**
 * \brief           Mahony filter quaternion (w x y z)
 * \return          Pointer to the four quaternion components
 */
const float* vcu_sensor_quaternion(void);

/**
 * \brief           Averaged tilt (`tilt_mean_deg` of the attitude block)
 * \note            What the tilt class (vcu_sensor_attitude_update), vehicle_task_fsm (`<= 50`)
 *                      and vehicle_state_alarm (`> 50`) compare. A compare of the float bits as
 *                      signed integers equals a float compare here.
 * \return          Tilt in degrees, (-180, 180]
 */
float vcu_sensor_tilt_deg(void);

/**
 * \brief           Raw gyro sample in the VCU frame
 * \note            turn_signal_auto_cancel_tick reads axis 2, ride_assist_tick axis 1.
 * \param[in]       axis: 0 = GX, 1 = -GY, 2 = -GZ (any other value reads 0)
 * \return          Gyro in LSB (70 mdps/LSB)
 */
int32_t vcu_sensor_gyro_raw(uint32_t axis);

/**
 * \brief           Acceleration magnitude
 * \return          |a| in g, sqrtf((ax^2 + ay^2) + az^2) of the filtered accel
 */
float vcu_sensor_accel_norm_g(void);

/**
 * \brief           Filtered die temperature
 * \return          Temperature in 0.1 degC (IIR, seeded at 250 = 25.0 degC)
 */
int16_t vcu_sensor_temp(void);

/* ------------------------------------------------------------------------- */
/* SPI2 bus primitives: the bus select over the PA15/PF7 chip selects and the */
/* byte transfer under the IMU register protocol (imu_reg_write / ...).      */
/* ------------------------------------------------------------------------- */

/** \brief vcu_spi2_bus_select modes. */
enum {
    VCU_SPI2_SEL_IDLE = 0,  /*!< PA15 set, PF7 set: bus released        */
    VCU_SPI2_SEL_DEV_A = 1, /*!< PA15 reset, PF7 set: device A selected */
    VCU_SPI2_SEL_IMU = 2    /*!< PF7 reset, PA15 set: IMU selected      */
};

/**
 * \brief           Select / release a device on the shared SPI2 bus
 * \param[in]       mode: VCU_SPI2_SEL_IDLE / _DEV_A / _IMU (any other value: no change)
 */
void vcu_spi2_bus_select(int32_t mode);

/**
 * \brief           Clock one byte through SPI2
 * \note            Poll transmit-ready, write DT, poll receive-ready, read DT.
 * \param[in]       tx: byte to send
 * \param[out]      rx: receives the byte clocked in (may be NULL)
 * \return          0 on success, 1 on a status-flag timeout
 */
uint8_t vcu_spi2_xfer(uint8_t tx, uint8_t* rx);

#endif /* SENSOR_H */
