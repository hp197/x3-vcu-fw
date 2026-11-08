/**
 * \file            sensor.c
 * \brief           x3 VCU on-board IMU over SPI2 + shared SPI2 bus primitives
 * \note            IMU driver and attitude estimator. See sensor.h for the module layout and
 *                      the LSM6DS3 register enum.
 *
 *                      PIPELINE (one vcu_sensor_attitude_update pass):
 *                        read 14 bytes, remap axes, filter temperature   (imu_read_raw)
 *                        add speed-derived accel terms, clamp, average   (imu_accel_compensate)
 *                        accel IIR 15/16, scale to g and rad/s           (imu_sample_update)
 *                        Mahony filter, tilt, 100-sample tilt mean       (imu_mahony_update)
 *                        |a|, motion metric, tilt class                  (imu_motion_metric_update)
 *
 *                      MATH: float ops are IEEE-754 with a fixed operand order, so the outputs
 *                      are bit-reproducible. atan2f and sqrtf are implemented here (Arm mathlib);
 *                      newlib's atan2f differs by 1 ulp on about 14 % of inputs.
 *
 *                      BUS: the LSM6DS3 IMU hangs off hardware SPI2 (base 0x40003800), with
 *                      the two chip selects PA15/PF7 driven as GPIO.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "sensor.h"
#include <math.h>
#include <string.h>
#include "board.h"     /* BOARD_APB2_CLOCKS_ON */
#include "gpio.h"      /* vcu_gpio_config */
#include "vcu_state.h" /* g_in2 (speed, tilt-class enable), g_vs (metric, class), FSM state */

/* ------------------------------------------------------------------------- */
/* Protocol constants.                                                        */
/* ------------------------------------------------------------------------- */

#define IMU_READ_FLAG         0x80U /*!< command MSB: set = read, clear = write */

#define IMU_WHOAMI_I          0x69U /*!< WHO_AM_I: LSM6DS3                                 */
#define IMU_WHOAMI_J          0x6AU /*!< WHO_AM_I: LSM6DSL / LSM6DSM / DS3TR-C             */
#define IMU_CTRL_VERIFY       0x38U /*!< CTRL9_XL / CTRL10_C expected readback ('8')      */

#define IMU_PROBE_TRIES       2U    /*!< WHO_AM_I reads per probe          */
#define IMU_XFER_TRIES        250U  /*!< per-byte bus-flag poll count      */
#define IMU_PROBE_FAIL        3     /*!< probe / identity mismatch result  */

#define IMU_DATA_BYTES        14    /*!< OUT_TEMP_L block: TEMP + 3 gyro + 3 accel words */
#define IMU_IIR_NUM           15    /*!< IIR numerator: y = (15 * y + x + 7) >> 4         */
#define IMU_IIR_ROUND         7     /*!< IIR rounding bias                                */
#define IMU_IIR_SHIFT         4     /*!< IIR denominator shift (>> 4 == / 16)             */
#define IMU_TEMP_SEED         250   /*!< temperature IIR seed / offset (25.0 degC)       */
#define IMU_TEMP_SCALE        10    /*!< temperature raw * 10 / temp_div (0.1 degC)      */
#define IMU_TEMP_DIV_I        16U   /*!< temp_div default                     */
#define IMU_TEMP_DIV_J        256U  /*!< temp_div written for a 'j' part      */
#define IMU_ACC_LSB_EXP       (-14) /*!< accel LSB -> g: scale by 2^-14 (+-2 g)         */

/* ------------------------------------------------------------------------- */
/* Estimator constants.                                                      */
/* ------------------------------------------------------------------------- */

#define IMU_GYRO_SCALE        0.0012217f /*!< rad/s per LSB (70 mdps)                */
#define IMU_MAHONY_KP         4.0f       /*!< proportional gain                      */
#define IMU_MAHONY_KI         0.1f       /*!< integral gain (no dt)                  */
#define IMU_MAHONY_HALFDT     0.005f     /*!< 0.5 * dt, dt = 10 ms (bits 0x3BA3D70A) */

#define IMU_INVSQRT_MAGIC     0x5F3759DFU /*!< initial-guess magic for 1 / sqrt(x) */
#define IMU_INVSQRT_3HALF     1.5f        /*!< Newton step constant (bits 0x3FC00000) */

/* IEEE-754 single-precision fields, for prv_sqrtf. */
#define IMU_F32_SIGN_BIT      0x80000000U /*!< sign */
#define IMU_F32_ABS_MASK      0x7FFFFFFFU /*!< everything but the sign */
#define IMU_F32_POS_INF       0x7F800000U /*!< +inf; a larger magnitude is a NaN */
#define IMU_F32_QUIET_BIT     0x00400000U /*!< quiet-NaN bit */
#define IMU_F32_DEFAULT_NAN   0x7FC00000U /*!< default quiet NaN */
#define IMU_F32_MANT_MASK     0x007FFFFFU /*!< stored mantissa */
#define IMU_F32_HIDDEN_BIT    0x00800000U /*!< implicit leading 1 */
#define IMU_F32_EXP_BIAS      127         /*!< exponent bias */
#define IMU_F32_HALF_BITS     0x3F000000U /*!< 0.5f: exponent 126, rebuilt with the hidden bit */
#define IMU_SQRT_STEP_TOP     0x01000000U /*!< first result bit (24 mantissa bits + guard) */

#define IMU_TILT_DEG_PER_RAD  57.29578f       /*!< 180 / pi                             */
#define IMU_TILT_HALF_TURN    180.0f           /*!< half a turn (deg)                    */
#define IMU_TILT_WRAP         180.0f           /*!< tilt > 180 -> tilt - 360             */
#define IMU_TILT_TURN         360.0f           /*!< one full turn (deg)                  */
#define IMU_TILT_AVG_LIMIT    179.0f           /*!< only |tilt| < 179 is averaged        */
#define IMU_TILT_AVG_LEN      100U             /*!< tilt mean window                     */

#define IMU_CLASS_UPRIGHT_DEG 30.0f /*!< mean <= 30 -> class 2           */
#define IMU_CLASS_FALLEN_DEG  90.0f /*!< mean > 90 after class 2 -> 1    */

#define IMU_COMP_TICK_RESET   10U  /*!< counter seed, and the value after a recompute     */
#define IMU_COMP_TICK_LAST    20U  /*!< recompute when the counter was 20 (period 11 passes)   */
#define IMU_COMP_GAIN_X       320  /*!< d * 5 << 6                                        */
#define IMU_COMP_GAIN_Z       240  /*!< d * 15 << 4                                       */
#define IMU_COMP_CLAMP        5000 /*!< +- around the last average (0x1388)              */
#define IMU_AVG_LEN           100U /*!< moving-average window                             */

#define IMU_METRIC_TICK_RESET 10U      /*!< counter seed / snapshot reload value */
#define IMU_METRIC_TICK_LAST  15U      /*!< snapshot when the counter passes 15 (6 passes) */
#define IMU_METRIC_SCALE      10000.0f /*!< metric scale                         */

/* Tilt class values in g_vs.tilt_latch (the names control.c uses for the same cell). */
enum {
    TILT_LATCH_CLEAR = 0,   /*!< states 4 / 5     */
    TILT_LATCH_TRIPPED = 1, /*!< fallen over      */
    TILT_LATCH_ARMED = 2    /*!< upright, armed   */
};

/* GPIO bus-select lines driven by the bus mux. */
#define IMU_CS1_PORT     GPIOA        /*!< 0x40010800 */
#define IMU_CS1_PIN      GPIO_PINS_15 /*!< PA15       */
#define IMU_CS2_PORT     GPIOF        /*!< 0x40011C00 */
#define IMU_CS2_PIN      GPIO_PINS_7  /*!< PF7        */

/* SPI2 signal pins after the SPI2_GMUX_0001 remap. */
#define IMU_SPI_PORT     GPIOB
#define IMU_SPI_SCK_PIN  GPIO_PINS_3 /*!< PB3 SPI2_SCK  */
#define IMU_SPI_MISO_PIN GPIO_PINS_4 /*!< PB4 SPI2_MISO */
#define IMU_SPI_MOSI_PIN GPIO_PINS_5 /*!< PB5 SPI2_MOSI */

#define VCU_IMU_SPI      SPI2 /*!< on-board IMU bus (base 0x40003800) */

/* SPI2 ctrl1 fields spi_init writes: clkpha [0], clkpol [1], msten [2], mdiv_l [5:3], ltf [7],
   swcsil [8], swcsen [9], ora [10], fbn [11], slbtd [14], slben [15]. */
#define IMU_SPI_CTRL1_MASK 0xCFBFu
/* Mode 3 (clkpol = clkpha = 1), master, /8 (mdiv_l 2), MSB first, 8-bit, full duplex, software
   CS with swcsil set. */
#define IMU_SPI_CTRL1_CFG                                                                                              \
    (SPI_CLOCK_PHASE_2EDGE | (SPI_CLOCK_POLARITY_HIGH << 1) | (SPI_MODE_MASTER << 2) | (SPI_MCLK_DIV_8 << 3)           \
     | (1u << 8) | (SPI_CS_SOFTWARE_MODE << 9))
#define IMU_SPI_CTRL2_MDIV_H 0x100u /*!< ctrl2 mdiv_h [8]: divider above /256         */
#define IMU_SPI_CTRL2_HWCSOE 0x004u /*!< ctrl2 hwcsoe [2]: hardware CS output enable  */

/* ------------------------------------------------------------------------- */
/* Module state.                                                             */
/* ------------------------------------------------------------------------- */

/** \brief One chip sample in the VCU frame */
typedef struct {
    int32_t gyro[3]; /*!< GX, -GY, -GZ (LSB)                                        */
    int32_t acc[3];  /*!< -AX, AY, AZ (LSB); replaced by the compensated, clamped value */
    int16_t temp;    /*!< temperature, 0.1 degC (IIR 15/16, seed 250)                */
} imu_raw_t;

/** \brief Compensated, averaged sample */
typedef struct {
    int32_t gyro[3]; /*!< copy of the raw gyro               */
    int32_t acc[3];  /*!< 100-sample moving average per axis */
} imu_avg_sample_t;

/** \brief Integer moving average (0x19C bytes) */
typedef struct {
    uint8_t count;             /*!< +0x000 samples held (<= window)  */
    uint8_t index;             /*!< +0x001 next write slot           */
    int32_t ring[IMU_AVG_LEN]; /*!< +0x004 samples                   */
    int32_t mean;              /*!< +0x194 sum / count (truncating)  */
    uint8_t window;            /*!< +0x198 window length (100)       */
} imu_movavg_t;

/** \brief Float tilt average */
typedef struct {
    float ring[IMU_TILT_AVG_LEN]; /*!< samples           */
    uint8_t count;                /*!< samples held      */
    uint8_t index;                /*!< next write slot   */
} imu_tilt_avg_t;

/** \brief Mahony filter state */
typedef struct {
    float quat[4];     /*!< w, x, y, z (initially 1, 0, 0, 0) */
    float integral[3]; /*!< integral feedback x, y, z         */
} imu_mahony_t;

/** \brief Accel compensation state */
typedef struct {
    uint8_t tick;       /*!< counts 10..20 (initially 10)           */
    int32_t speed_prev; /*!< speed at the last recompute            */
    int32_t comp_z;     /*!< (speed delta) * 240, added to raw Z    */
    int32_t comp_x;     /*!< (speed delta) * 320, added to raw X    */
} imu_comp_t;

/** \brief Motion metric state */
typedef struct {
    uint8_t tick;     /*!< counts 10..16 (initially 10)  */
    float acc_ref[3]; /*!< accel snapshot (g)            */
} imu_metric_t;

/** \brief Device record */
typedef struct {
    uint8_t whoami;    /*!< last WHO_AM_I byte                              */
    uint8_t retry;     /*!< probe counter (never reset)                     */
    uint16_t temp_div; /*!< temperature divisor (initially 16; 256 for 'j') */
} imu_dev_t;

static imu_raw_t g_imu_raw;            /*!< last chip sample                  */
static imu_avg_sample_t g_imu_avg;     /*!< compensated, averaged sample      */
static int32_t g_imu_acc_iir[3];       /*!< accel IIR state (LSB)             */
static float g_imu_gyro_rads[3];       /*!< gyro (rad/s)                      */
static float g_imu_acc_g[3];           /*!< accel (g)                         */
static float g_imu_acc_norm_g;         /*!< |a| (g)                           */
static imu_attitude_t g_imu_att;       /*!< attitude outputs                  */
static imu_tilt_avg_t g_imu_tilt_avg;  /*!< tilt average                      */
static imu_movavg_t g_imu_acc_mavg[3]; /*!< per-axis accel moving averages    */
static uint8_t g_imu_acc_iir_seeded;   /*!< accel IIR seeded                  */
static imu_mahony_t g_imu_mahony = {
    /*!< Mahony filter state               */
    .quat = {1.0f, 0.0f, 0.0f, 0.0f},
};
static imu_comp_t g_imu_comp = {
    /*!< accel compensation state          */
    .tick = IMU_COMP_TICK_RESET,
};
static imu_metric_t g_imu_metric = {
    /*!< motion metric state               */
    .tick = IMU_METRIC_TICK_RESET,
};
static imu_dev_t g_imu_dev = {
    /*!< device record                     */
    .temp_div = IMU_TEMP_DIV_I,
};

/* ------------------------------------------------------------------------- */
/* IMU bus primitives: hardware SPI2 byte transfer + GPIO chip-select.         */
/* ------------------------------------------------------------------------- */

/**
 * \brief           SPI2 pin mux: remap to PB3/PB4/PB5, CS lines PA15/PF7 as outputs
 * \note            Enables the GPIOB, GPIOA, GPIOF and IOMUX clocks, applies SPI2_GMUX_0001,
 *                      then: SCK PB3 + MOSI PB5 mux push-pull, MISO PB4 input pull-up, CS1 PA15
 *                      and CS2 PF7 push-pull outputs, all drive stronger.
 *                      PA15/PB3/PB4 are JTAG pins; vcu_periph_init releases them with
 *                      SWJTAG_GMUX_010 at its tail.
 */
static void vcu_sensor_bus_gpio_init(void) {
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_GPIOB_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_GPIOA_PERIPH_CLOCK)
                         | BOARD_APB2_CLK(CRM_GPIOF_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_IOMUX_PERIPH_CLOCK));
    board_pin_remap(SPI2_GMUX_0001); /* SPI2 -> PA15/PB3/PB4/PB5 */

    /* A mux pin ignores the pull setting (gpio_init writes no odt bit for it). */
    vcu_gpio_config(IMU_SPI_PORT, IMU_SPI_SCK_PIN | IMU_SPI_MOSI_PIN, VCU_GPIO_CFG_MUX_PP); /* SCK, MOSI */
    vcu_gpio_config(IMU_SPI_PORT, IMU_SPI_MISO_PIN, VCU_GPIO_CFG_IN_PU);                    /* MISO */
    vcu_gpio_config(IMU_CS1_PORT, IMU_CS1_PIN, VCU_GPIO_CFG_OUT_PP);                        /* CS1 PA15 */
    vcu_gpio_config(IMU_CS2_PORT, IMU_CS2_PIN, VCU_GPIO_CFG_OUT_PP);                        /* CS2 PF7  */
}

/**
 * \brief           Bring up the IMU SPI2 bus
 * \note            Enables the SPI2 clock, then (SPI disabled) sets master, full-duplex, 8-bit,
 *                      MSB-first, mode 3 (CPOL=CPHA=1), /8 clock and software CS. Clearing
 *                      CTRL2.hwcsoe keeps the hardware NSS idle so PA15/PF7 drive CS as GPIO.
 *                      Re-enables the SPI.
 */
static void vcu_sensor_spi_init(void) {
    BOARD_CLOCK_ON(CRM_SPI2_PERIPH_CLOCK);
    VCU_IMU_SPI->ctrl1_bit.spien = FALSE;

    /* What spi_init writes for these settings: SPI mode (not I2S), the ctrl1 fields, mdiv_h 0. */
    VCU_IMU_SPI->i2sctrl_bit.i2smsel = FALSE;
    VCU_IMU_SPI->ctrl1 = (VCU_IMU_SPI->ctrl1 & ~IMU_SPI_CTRL1_MASK) | IMU_SPI_CTRL1_CFG;
    /* mdiv_h 0 and hwcsoe 0: NSS idle, CS is GPIO (ctrl2 & 0xfffb) */
    VCU_IMU_SPI->ctrl2 &= ~(IMU_SPI_CTRL2_MDIV_H | IMU_SPI_CTRL2_HWCSOE);

    VCU_IMU_SPI->ctrl1_bit.spien = TRUE;
}

/**
 * \brief           Shared SPI2 bus select (see sensor.h)
 * \note            Drives the two chip-select lines PA15 (CS1) and PF7 (CS2).
 */
void vcu_spi2_bus_select(int32_t mode) {
    if (mode == VCU_SPI2_SEL_IDLE) {
        gpio_bits_set(IMU_CS1_PORT, IMU_CS1_PIN);
        gpio_bits_set(IMU_CS2_PORT, IMU_CS2_PIN);
    } else if (mode == VCU_SPI2_SEL_DEV_A) {
        gpio_bits_reset(IMU_CS1_PORT, IMU_CS1_PIN);
        gpio_bits_set(IMU_CS2_PORT, IMU_CS2_PIN);
    } else if (mode == VCU_SPI2_SEL_IMU) {
        gpio_bits_reset(IMU_CS2_PORT, IMU_CS2_PIN);
        gpio_bits_set(IMU_CS1_PORT, IMU_CS1_PIN);
    }
}

/**
 * \brief           IMU SPI2 bus bring-up (see sensor.h)
 * \note            Pin mux, SPI2 init, then vcu_spi2_bus_select(IDLE): both chip selects
 *                      high.
 */
void vcu_sensor_bus_init(void) {
    vcu_sensor_bus_gpio_init();
    vcu_sensor_spi_init();
    vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
}

/**
 * \brief           Wait for an SPI2 status flag (the two polls of vcu_spi2_xfer)
 * \param[in]       flag: SPI_I2S_TDBE_FLAG or SPI_I2S_RDBF_FLAG
 * \return          1 when the flag was seen with retries left, else 0
 * \note            Counts from 0xFA down and polls once per count, including at 0 (251
 *                      polls); a flag first seen at count 0 still times out.
 */
static uint8_t prv_spi2_wait(uint32_t flag) {
    uint32_t tries = IMU_XFER_TRIES;

    while ((spi_i2s_flag_get(VCU_IMU_SPI, flag) == RESET) && (tries != 0U)) {
        --tries;
    }
    return (uint8_t)(tries != 0U);
}

/**
 * \brief           SPI2 one-byte transfer (see sensor.h)
 * \note            Waits transmit-ready, sends, waits receive-ready, reads. The received
 *                      byte is always read (clearing RDBF) and stored only when \p rx is set.
 *                      The register helpers pass NULL for every command byte and every write
 *                      data byte.
 */
uint8_t vcu_spi2_xfer(uint8_t tx, uint8_t* rx) {
    uint8_t rc = 1U;
    uint8_t byte;

    if (prv_spi2_wait(SPI_I2S_TDBE_FLAG) != 0U) {
        spi_i2s_data_transmit(VCU_IMU_SPI, tx);
        if (prv_spi2_wait(SPI_I2S_RDBF_FLAG) != 0U) {
            byte = (uint8_t)spi_i2s_data_receive(VCU_IMU_SPI);
            if (rx != NULL) { /* NULL: the caller discards the byte */
                *rx = byte;
            }
            rc = 0U;
        }
    }
    return rc;
}

/* ------------------------------------------------------------------------- */
/* Register protocol. The command MSB (IMU_READ_FLAG) is the direction bit:  */
/* set = read, clear = write.                                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Read one register byte (cmd | 0x80)
 * \param[in]       reg: register address
 * \param[out]      val: receives the register byte
 * \return          0 on success, 2 if \p val is NULL, else the bus error code
 */
static int32_t imu_reg_read(uint32_t reg, uint8_t* val) {
    int32_t rc = 2;

    if (val != NULL) {
        vcu_spi2_bus_select(VCU_SPI2_SEL_IMU);
        rc = (int32_t)vcu_spi2_xfer((uint8_t)(reg | IMU_READ_FLAG), NULL);
        if (rc == 0) {
            rc = (int32_t)vcu_spi2_xfer(0U, val);
        }
        vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
    }
    return rc;
}

/**
 * \brief           Write one register byte (cmd & ~0x80)
 * \param[in]       reg: register address
 * \param[in]       val: value to write
 * \return          0 on success, else the bus error code
 */
static int32_t imu_reg_write(uint32_t reg, uint8_t val) {
    int32_t rc;

    vcu_spi2_bus_select(VCU_SPI2_SEL_IMU);
    rc = (int32_t)vcu_spi2_xfer((uint8_t)(reg & ~IMU_READ_FLAG), NULL);
    if (rc == 0) {
        rc = (int32_t)vcu_spi2_xfer(val, NULL);
    }
    vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
    return rc;
}

/**
 * \brief           Burst-read \p count register bytes (cmd | 0x80)
 * \param[in]       reg: start register (auto-incrementing)
 * \param[out]      buf: destination buffer
 * \param[in]       count: number of bytes
 * \return          0 on success, else the bus error code
 */
static int32_t imu_burst_read(uint32_t reg, uint8_t* buf, int32_t count) {
    int32_t rc;
    int32_t idx;

    vcu_spi2_bus_select(VCU_SPI2_SEL_IMU);
    rc = (int32_t)vcu_spi2_xfer((uint8_t)(reg | IMU_READ_FLAG), NULL);
    for (idx = 0; (rc == 0) && (idx < count); ++idx) {
        rc = (int32_t)vcu_spi2_xfer(0U, &buf[idx]);
    }
    vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
    return rc;
}

/**
 * \brief           Burst-write \p count register bytes (cmd & ~0x80)
 * \param[in]       reg: start register (auto-incrementing)
 * \param[in]       buf: source buffer
 * \param[in]       count: number of bytes
 * \return          0 on success, else the bus error code
 */
static int32_t imu_burst_write(uint32_t reg, const uint8_t* buf, int32_t count) {
    int32_t rc;
    int32_t idx;

    vcu_spi2_bus_select(VCU_SPI2_SEL_IMU);
    rc = (int32_t)vcu_spi2_xfer((uint8_t)(reg & ~IMU_READ_FLAG), NULL);
    for (idx = 0; (rc == 0) && (idx < count); ++idx) {
        rc = (int32_t)vcu_spi2_xfer(buf[idx], NULL);
    }
    vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
    return rc;
}

/* ------------------------------------------------------------------------- */
/* Float helpers with fixed bit-level semantics.                            */
/* ------------------------------------------------------------------------- */

/** \brief IEEE-754 bits of \p value \param[in] value: float \return its bit pattern */
static uint32_t prv_f2bits(float value) {
    uint32_t bits;

    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/** \brief Float with the bit pattern \p bits \param[in] bits: IEEE-754 bits \return the float */
static float prv_bits2f(uint32_t bits) {
    float value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/**
 * \brief           x * 2^n by exponent arithmetic
 * \param[in]       value: x
 * \param[in]       exp2: n
 * \return          ±0 for ±0; +0 if the result would not be normal; else x with n added to
 *                      its exponent (no overflow check)
 */
static float prv_scalbf(float value, int32_t exp2) {
    uint32_t bits = prv_f2bits(value);
    int32_t exponent = (int32_t)((bits >> 23) & 0xFFU);

    if ((bits << 1) != 0U) {
        bits = (exp2 > -exponent) ? (bits + ((uint32_t)exp2 << 23)) : 0U;
    }
    return prv_bits2f(bits);
}

/**
 * \brief           1 / sqrt(x): 0x5F3759DF guess and one Newton step
 * \param[in]       value: x (> 0)
 * \return          y * (1.5 - (x/2 * y) * y)
 */
static float fast_inv_sqrt(float value) {
    float half = prv_scalbf(value, -1);
    float guess = prv_bits2f(IMU_INVSQRT_MAGIC - (uint32_t)((int32_t)prv_f2bits(value) >> 1));

    return (IMU_INVSQRT_3HALF - ((half * guess) * guess)) * guess;
}

/**
 * \brief           Correctly rounded single-precision square root
 * \note            The classic restoring square root on the mantissa, rounded to
 *                      nearest-even: the exact IEEE-754 result, the same one newlib `sqrtf`
 *                      gives. It replaces newlib `sqrtf`, `errno` and `impure_data` (about
 *                      0.4 KB) to fit the 60 KB slot; nothing here reads `errno`.
 *                      Special cases: +-0 -> +-0, +inf -> +inf, NaN -> quiet NaN, x < 0 ->
 *                      default NaN.
 * \param[in]       value: x
 * \return          sqrt(x)
 */
static float prv_sqrtf(float value) {
    uint32_t bits = prv_f2bits(value);
    uint32_t mant;
    uint32_t root = 0U;
    uint32_t step = IMU_SQRT_STEP_TOP;
    int32_t exponent;

    if (((bits & IMU_F32_ABS_MASK) == 0U) || (bits == IMU_F32_POS_INF)) {
        return value; /* +-0 and +inf are their own root */
    }
    if ((bits & IMU_F32_ABS_MASK) > IMU_F32_POS_INF) {
        bits |= IMU_F32_QUIET_BIT; /* NaN in, quiet NaN out */
    } else if ((bits & IMU_F32_SIGN_BIT) != 0U) {
        bits = IMU_F32_DEFAULT_NAN; /* x < 0 */
    } else {
        exponent = (int32_t)(bits >> 23);
        mant = bits & IMU_F32_MANT_MASK;
        if (exponent == 0) { /* subnormal: normalise the mantissa */
            exponent = 1;
            while ((mant & IMU_F32_HIDDEN_BIT) == 0U) {
                mant <<= 1;
                exponent--;
            }
        }
        mant |= IMU_F32_HIDDEN_BIT;
        exponent -= IMU_F32_EXP_BIAS;
        if ((exponent & 1) != 0) {
            mant <<= 1; /* make the exponent even */
        }
        exponent >>= 1;
        mant <<= 1;
        /* One result bit per step: 24 mantissa bits plus a guard bit. */
        while (step != 0U) {
            uint32_t trial = root + step;

            if (trial <= mant) {
                mant -= trial;
                root = trial + step;
            }
            mant <<= 1;
            step >>= 1;
        }
        /* `root` is 2 * q, where q is the 24-bit result plus a guard bit. A non-zero remainder
           means the exact root lies above q, so round half to even on the guard bit. */
        if (mant != 0U) {
            root += root & 2U;
        }
        bits = (root >> 2) + IMU_F32_HALF_BITS + ((uint32_t)exponent << 23);
    }
    return prv_bits2f(bits);
}

/**
 * \brief           x <= -FLT_MIN, the "negative" test on float bits
 * \param[in]       value: x
 * \return          1 when the bits are >= 0x80800000 (a negative normal or worse)
 * \note            An unsigned compare of the bits. -0 and negative subnormals count as not
 *                      negative.
 */
static uint8_t prv_is_neg_normal(float value) { return (uint8_t)(prv_f2bits(value) >= 0x80800000U); }

/*
 * atan2f after the Arm mathlib algorithm: range reduction to |t| <= 1 by swapping
 * (pi/2 - ...), an extra reduction by atan(0.5) when the ratio lies in (0.5, 1], an odd
 * degree-11 polynomial and a two-part (hi + lo) offset. There is no errno handling: nothing
 * reads errno.
 */
#define ATAN_PI_2_F  1.5707963705062866f     /*!< 0x3FC90FDB */
#define ATAN_PI_F    3.1415927410125732f     /*!< 0x40490FDB */
#define ATAN_PI_2_HI 1.5703125f              /*!< 0x3FC90000 */
#define ATAN_PI_2_LO 0.0004838267923332751f  /*!< 0x39FDAA22 */
#define ATAN_PI_HI   3.140625f               /*!< 0x40490000 */
#define ATAN_PI_LO   0.0009676535846665502f  /*!< 0x3A7DAA22 */
#define ATAN_HALF_HI 0.463623046875f         /*!< 0x3EED6000, atan(0.5) high part */
#define ATAN_HALF_LO 2.4562124963267706e-05f /*!< 0x37CE0AC3, atan(0.5) low part */
#define ATAN_C0      (-0.05607335641980171f) /*!< 0xBD65AD2D */
#define ATAN_C1      0.10435592383146286f    /*!< 0x3DD5B88F */
#define ATAN_C2      (-0.14229224622249603f) /*!< 0xBE11B50F */
#define ATAN_C3      0.1999831348657608f     /*!< 0x3E4CC861 */
#define ATAN_C4      (-0.33333325386047363f) /*!< 0xBEAAAAA8 */
#define F32_SIGN     0x80000000U
#define F32_INF      0x7F800000U

/**
 * \brief           Core of atan2f once the operands are in range
 * \param[in]       y_bits: y as used for the decisions (exponent, sign, magnitude order)
 * \param[in]       x_bits: x as used for the decisions
 * \param[in]       y_val: y as used in the arithmetic
 * \param[in]       x_val: x as used in the arithmetic
 * \return          atan2(y, x)
 * \note            The special-case entries pass different decision and value operands (for
 *                      example y | inf for a zero x): the decision bits and the arithmetic
 *                      values are kept apart.
 */
static float prv_atan2f_core(uint32_t y_bits, uint32_t x_bits, float y_val, float x_val) {
    int32_t exp_diff = (int32_t)((y_bits >> 23) & 0xFFU) - (int32_t)((x_bits >> 23) & 0xFFU);
    uint32_t swap;
    float result;
    float num;
    float den;
    float hi;
    float lo;
    float tan_r;
    float sq;
    float poly;
    float half;

    if (exp_diff > 27) {
        result = ((y_bits & F32_SIGN) != 0U) ? -ATAN_PI_2_F : ATAN_PI_2_F;
    } else if (exp_diff < -26) {
        if ((x_bits & F32_SIGN) != 0U) {
            result = ((y_bits & F32_SIGN) == 0U) ? ATAN_PI_F : -ATAN_PI_F;
        } else {
            result = y_val / x_val;
        }
    } else {
        if ((y_bits << 1) > (x_bits << 1)) { /* |y| > |x|: atan(y/x) = +-pi/2 + atan(x / -y) */
            hi = ((y_bits & F32_SIGN) != 0U) ? -ATAN_PI_2_HI : ATAN_PI_2_HI;
            lo = ((y_bits & F32_SIGN) != 0U) ? -ATAN_PI_2_LO : ATAN_PI_2_LO;
            num = x_val;
            den = -y_val;
            swap = y_bits;
            y_bits = x_bits;
            x_bits = swap ^ F32_SIGN;
        } else if ((x_bits & F32_SIGN) == 0U) {
            hi = 0.0f;
            lo = 0.0f;
            num = y_val;
            den = x_val;
        } else { /* x < 0: +-pi + atan(y/x) */
            hi = ((y_bits & F32_SIGN) != 0U) ? -ATAN_PI_HI : ATAN_PI_HI;
            lo = ((y_bits & F32_SIGN) != 0U) ? -ATAN_PI_LO : ATAN_PI_LO;
            num = y_val;
            den = x_val;
        }

        if ((uint32_t)((x_bits - y_bits) << 1) >= 0x1000000U) {
            tan_r = num / den;
        } else { /* ratio in (0.5, 1]: atan(r) = atan(0.5) + atan((r - 0.5) / (1 + 0.5 r)) */
            if (((y_bits ^ x_bits) & F32_SIGN) != 0U) {
                half = -0.5f;
                hi = hi - ATAN_HALF_HI;
                lo = lo - ATAN_HALF_LO;
            } else {
                half = 0.5f;
                hi = hi + ATAN_HALF_HI;
                lo = lo + ATAN_HALF_LO;
            }
            tan_r = (num - (half * den)) / ((num * half) + den);
        }

        sq = tan_r * tan_r;
        poly = ((((((((sq * ATAN_C0) + ATAN_C1) * sq) + ATAN_C2) * sq) + ATAN_C3) * sq) + ATAN_C4);
        result = ((((tan_r * sq) * poly) + lo) + tan_r) + hi;
    }
    return result;
}

/**
 * \brief           atan2f (Arm mathlib algorithm)
 * \param[in]       y_in: y
 * \param[in]       x_in: x
 * \return          atan2(y, x) in [-pi, pi]
 * \note            Operands with an exponent outside [12, 252] (zero, tiny, huge, inf, NaN)
 *                      take the special-case path first: NaN -> y + x; both zero -> x
 *                      becomes +-inf; both inf -> +-1, +-1; one zero or inf -> the matching
 *                      +-0 / +-inf pair; otherwise both are scaled by 2^32 or 2^-32.
 */
static float prv_atan2f(float y_in, float x_in) {
    uint32_t y_bits = prv_f2bits(y_in);
    uint32_t x_bits = prv_f2bits(x_in);
    uint32_t y_mag = y_bits << 1;
    uint32_t x_mag = x_bits << 1;
    float y_val = y_in;
    float x_val = x_in;
    float scale;
    float result;

    if (((uint32_t)(0xE8000000U + y_mag) < 0xE5000000U) && ((uint32_t)(0xE8000000U + x_mag) < 0xE5000000U)) {
        result = prv_atan2f_core(y_bits, x_bits, y_val, x_val);
    } else if ((y_mag > 0xFF000000U) || (x_mag > 0xFF000000U)) {
        result = y_in + x_in; /* NaN */
    } else if ((y_mag | x_mag) == 0U) {
        x_bits |= F32_INF;
        result = prv_atan2f_core(y_bits, x_bits, y_val, prv_bits2f(x_bits));
    } else if ((y_mag == 0xFF000000U) && (x_mag == 0xFF000000U)) {
        y_bits &= ~0x40000000U;
        x_bits &= ~0x40000000U;
        result = prv_atan2f_core(y_bits, x_bits, prv_bits2f(y_bits), prv_bits2f(x_bits));
    } else if ((y_mag == 0xFF000000U) || (x_mag == 0U)) {
        result = prv_atan2f_core(y_bits | F32_INF, x_bits & F32_SIGN, y_val, x_val);
    } else if ((x_mag == 0xFF000000U) || (y_mag == 0U)) {
        result = prv_atan2f_core(y_bits & F32_SIGN, x_bits | F32_INF, y_val, x_val);
    } else if (((x_mag ^ y_mag) & F32_SIGN) != 0U) {
        result = prv_atan2f_core(y_bits, x_bits, y_val, x_val);
    } else {
        scale = ((x_mag & F32_SIGN) == 0U) ? 4294967296.0f : 2.3283064365386963e-10f; /* 2^32 / 2^-32 */
        y_val = y_in * scale;
        x_val = x_in * scale;
        result = prv_atan2f_core(prv_f2bits(y_val), prv_f2bits(x_val), y_val, x_val);
    }
    return result;
}

/* ------------------------------------------------------------------------- */
/* IMU helpers.                                                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Clear the IMU state
 * \note            Zeroes the raw sample, the IIR state, the scaled gyro and accel, |a| and
 *                      the three moving averages, then sets each window to 100. It leaves the
 *                      averaged sample, the attitude block and the filter state alone.
 */
static void imu_state_clear(void) {
    uint32_t axis;

    (void)memset(&g_imu_raw, 0, sizeof g_imu_raw);
    (void)memset(g_imu_acc_iir, 0, sizeof g_imu_acc_iir);
    (void)memset(g_imu_gyro_rads, 0, sizeof g_imu_gyro_rads);
    (void)memset(g_imu_acc_g, 0, sizeof g_imu_acc_g);
    g_imu_acc_norm_g = 0.0f;
    for (axis = 0U; axis < 3U; ++axis) {
        (void)memset(&g_imu_acc_mavg[axis], 0, sizeof g_imu_acc_mavg[axis]);
        g_imu_acc_mavg[axis].window = (uint8_t)IMU_AVG_LEN;
    }
}

/**
 * \brief           Push a sample into an integer moving average and update its mean
 * \param[in,out]   avg: the average
 * \param[in]       sample: new value
 */
static void moving_avg_push(imu_movavg_t* avg, int32_t sample) {
    int32_t sum = 0;
    uint8_t idx;

    avg->ring[avg->index] = sample;
    avg->index = (uint8_t)(avg->index + 1U);
    avg->count = (uint8_t)(avg->count + 1U);
    if (avg->index >= avg->window) {
        avg->index = 0U;
    }
    if (avg->count > avg->window) {
        avg->count = avg->window;
    }
    for (idx = 0U; idx < avg->count; ++idx) {
        sum += avg->ring[idx];
    }
    avg->mean = sum / (int32_t)avg->count;
}

/**
 * \brief           Burst-read the data block, remap axes and filter temperature
 * \return          0 on success, else the bus error code
 * \note            14-byte read from OUT_TEMP_L (0x20): word 0 = temperature, words 1..3 =
 *                      gyro X/Y/Z, words 4..6 = accel X/Y/Z. Stored as gyro `GX, -GY, -GZ`
 *                      and accel `-AX, AY, AZ`. Temperature IIR:
 *                      y = (15 * y + (int16)(raw * 10 / temp_div + 250) + 7) >> 4.
 */
static int32_t imu_read_raw(void) {
    uint8_t raw[IMU_DATA_BYTES] = {0};
    int16_t word[IMU_DATA_BYTES / 2];
    int16_t temp_new;
    int32_t rc;
    int32_t idx;

    rc = imu_burst_read(LSM6DS3_OUT_TEMP_L, raw, IMU_DATA_BYTES);
    if (rc == 0) {
        for (idx = 0; idx < (IMU_DATA_BYTES / 2); ++idx) {
            word[idx] = (int16_t)((uint16_t)raw[2 * idx] | ((uint16_t)raw[(2 * idx) + 1] << 8));
        }
        temp_new = (int16_t)((((int32_t)word[0] * IMU_TEMP_SCALE) / (int32_t)g_imu_dev.temp_div) + IMU_TEMP_SEED);
        g_imu_raw.temp = (int16_t)((((int32_t)g_imu_raw.temp * IMU_IIR_NUM) + (int32_t)temp_new + IMU_IIR_ROUND)
                                   >> IMU_IIR_SHIFT);

        g_imu_raw.gyro[0] = (int32_t)word[1];
        g_imu_raw.gyro[1] = -(int32_t)word[2];
        g_imu_raw.gyro[2] = -(int32_t)word[3];
        g_imu_raw.acc[0] = -(int32_t)word[4];
        g_imu_raw.acc[1] = (int32_t)word[5];
        g_imu_raw.acc[2] = (int32_t)word[6];
    }
    return rc;
}

/**
 * \brief           Identity probe
 * \param[in]       expect: 4-byte expected CTRL1_XL..CTRL4_C readback
 * \return          0 on match, 3 on mismatch, else the CTRL9_XL read error
 * \note            For an LSM6DS ('i'/'j') part, reads CTRL9_XL (expected 0x38, a read error
 *                      returns it) and CTRL10_C (expected 0x38; its read result is not
 *                      checked), then compares a 4-byte read of CTRL1_XL (result not checked)
 *                      against \p expect. Both reads go into one zeroed 4-byte buffer. The
 *                      caller passes a zeroed buffer, so a configured chip returns 3;
 *                      the result is discarded.
 */
static int32_t imu_identify(const uint8_t* expect) {
    uint8_t ctrl[4] = {0U, 0U, 0U, 0U};
    int32_t rc = 0;
    int32_t idx;

    if ((g_imu_dev.whoami == IMU_WHOAMI_I) || (g_imu_dev.whoami == IMU_WHOAMI_J)) {
        rc = imu_reg_read(LSM6DS3_CTRL9_XL, &ctrl[0]);
        if ((rc == 0) && (ctrl[0] != IMU_CTRL_VERIFY)) {
            rc = IMU_PROBE_FAIL;
        }
        if (rc == 0) {
            ctrl[0] = 0U;
            (void)imu_reg_read(LSM6DS3_CTRL10_C, &ctrl[0]);
            if (ctrl[0] != IMU_CTRL_VERIFY) {
                rc = IMU_PROBE_FAIL;
            }
        }
    }
    if (rc == 0) {
        (void)imu_burst_read(LSM6DS3_CTRL1_XL, ctrl, 4);
        for (idx = 0; (rc == 0) && (idx < 4); ++idx) {
            if (ctrl[idx] != expect[idx]) {
                rc = IMU_PROBE_FAIL;
            }
        }
    }
    return rc;
}

/**
 * \brief           Run the ODR/range profile burst
 * \note            Writes WAKE_UP_DUR 0, WAKE_UP_THS 4, TAP_CFG 0x90, the CTRL1..CTRL4 profile
 *                      {0x51, 0x7C, 0x64, 0x04} and MD1_CFG 0x20.
 */
static void imu_profile_apply(void) {
    static const uint8_t profile[4] = {0x51, 0x7C, 0x64, 0x04}; /* CTRL1_XL..CTRL4_C */

    (void)imu_reg_write(LSM6DS3_WAKE_UP_DUR, 0U);
    (void)imu_reg_write(LSM6DS3_WAKE_UP_THS, 4U);
    (void)imu_reg_write(LSM6DS3_TAP_CFG, 0x90U);
    (void)imu_burst_write(LSM6DS3_CTRL1_XL, profile, 4);
    (void)imu_reg_write(LSM6DS3_MD1_CFG, 0x20U);
}

/**
 * \brief           Add the speed-derived accel terms, clamp, and average the accel
 * \note            The counter (initially 10) counts up; on the pass it was 20 the change of
 *                      the speed (g_in2 speed_src_s) since the last recompute sets the terms
 *                      d * 320 (raw X) and d * 240 (raw Z), and the counter restarts at 10: a recompute every
 *                      11 passes. The terms are added on every pass; each raw axis is then
 *                      clamped to +-5000 around its last average, stored back, and pushed into
 *                      that axis' 100-sample moving average. The averaged sample takes the
 *                      averages and a copy of the raw gyro.
 */
static void imu_accel_compensate(void) {
    uint8_t tick = g_imu_comp.tick;
    int32_t delta;
    int32_t centre;
    uint32_t axis;

    g_imu_comp.tick = (uint8_t)(tick + 1U);
    if (tick == IMU_COMP_TICK_LAST) {
        delta = (int32_t)g_in2.f.speed_src_s - g_imu_comp.speed_prev;
        g_imu_comp.comp_z = delta * IMU_COMP_GAIN_Z;
        g_imu_comp.comp_x = delta * IMU_COMP_GAIN_X;
        g_imu_comp.tick = (uint8_t)IMU_COMP_TICK_RESET;
    }
    if (g_imu_comp.tick == IMU_COMP_TICK_RESET) {
        g_imu_comp.speed_prev = (int32_t)g_in2.f.speed_src_s;
    }

    g_imu_raw.acc[2] += g_imu_comp.comp_z;
    g_imu_raw.acc[0] += g_imu_comp.comp_x;
    for (axis = 0U; axis < 3U; ++axis) {
        centre = g_imu_avg.acc[axis];
        if (g_imu_raw.acc[axis] > (centre + IMU_COMP_CLAMP)) {
            g_imu_raw.acc[axis] = centre + IMU_COMP_CLAMP;
        } else if (g_imu_raw.acc[axis] < (centre - IMU_COMP_CLAMP)) {
            g_imu_raw.acc[axis] = centre - IMU_COMP_CLAMP;
        }
    }
    for (axis = 0U; axis < 3U; ++axis) {
        moving_avg_push(&g_imu_acc_mavg[axis], g_imu_raw.acc[axis]);
    }
    for (axis = 0U; axis < 3U; ++axis) {
        g_imu_avg.gyro[axis] = g_imu_raw.gyro[axis];
        g_imu_avg.acc[axis] = g_imu_acc_mavg[axis].mean;
    }
}

/**
 * \brief           Read the IMU once, compensate, filter and scale
 * \return          0 on success, else the bus error code from imu_read_raw
 * \note            Accel: IIR 15/16 on the averaged sample, seeded with its NEGATED first
 *                      value while the filter adds the value, then 2^-14 = g. Gyro:
 *                      (float)raw * 0.0012217 = rad/s, no bias, no slew.
 */
static int32_t imu_sample_update(void) {
    int32_t rc;
    uint32_t axis;

    rc = imu_read_raw();
    if (rc == 0) {
        imu_accel_compensate();
        for (axis = 0U; axis < 3U; ++axis) {
            if (g_imu_acc_iir_seeded == 0U) {
                g_imu_acc_iir[axis] = -g_imu_avg.acc[axis];
            } else {
                g_imu_acc_iir[axis] = ((g_imu_acc_iir[axis] * IMU_IIR_NUM) + IMU_IIR_ROUND + g_imu_avg.acc[axis])
                                      >> IMU_IIR_SHIFT;
            }
        }
        g_imu_acc_iir_seeded = 1U;
        for (axis = 0U; axis < 3U; ++axis) {
            g_imu_acc_g[axis] = prv_scalbf((float)g_imu_acc_iir[axis], IMU_ACC_LSB_EXP);
        }
        for (axis = 0U; axis < 3U; ++axis) {
            g_imu_gyro_rads[axis] = (float)g_imu_avg.gyro[axis] * IMU_GYRO_SCALE;
        }
    }
    return rc;
}

/**
 * \brief           Push a tilt into the float average and update its mean
 * \param[in]       tilt: tilt of this pass (deg)
 * \note            Mean = sum of ring[i] / count over the held samples, each divided before
 *                      it is added.
 */
static void imu_tilt_avg_push(float tilt) {
    imu_tilt_avg_t* avg = &g_imu_tilt_avg;
    float sum = 0.0f;
    uint8_t idx;

    avg->ring[avg->index] = tilt;
    avg->index = (uint8_t)(avg->index + 1U);
    avg->count = (uint8_t)(avg->count + 1U);
    if (avg->count >= IMU_TILT_AVG_LEN) {
        avg->count = (uint8_t)IMU_TILT_AVG_LEN;
    }
    if (avg->index >= IMU_TILT_AVG_LEN) {
        avg->index = 0U;
    }
    for (idx = 0U; idx < avg->count; ++idx) {
        sum = (avg->ring[idx] / (float)avg->count) + sum;
    }
    g_imu_att.tilt_mean_deg = sum;
}

/**
 * \brief           One Mahony filter step and the tilt
 * \param[in]       gyro: rate x, y, z (rad/s)
 * \param[in]       acc: accel x, y, z (g)
 * \note            Skipped entirely when (ax * ay) * az has a zero exponent (any axis 0).
 *                      The quaternion integration uses the already updated components (q0 for
 *                      q1..q3, q1 for q2 and q3, q2 for q3), and the renormalisation uses the
 *                      squares of the quaternion from before the step.
 *                      The gravity estimate is from the quaternion before the step. Tilt =
 *                      atan2f(gx, gz) in degrees + 180, minus 360 above 180; only
 *                      |tilt| < 179 enters the 100-sample mean.
 */
static void imu_mahony_update(const float gyro[3], const float acc[3]) {
    float* quat = g_imu_mahony.quat;
    float* integ = g_imu_mahony.integral;
    float q0q0 = quat[0] * quat[0];
    float q0q1 = quat[0] * quat[1];
    float q0q2 = quat[0] * quat[2];
    float q1q1 = quat[1] * quat[1];
    float q1q3 = quat[1] * quat[3];
    float q2q2 = quat[2] * quat[2];
    float q2q3 = quat[2] * quat[3];
    float q3q3 = quat[3] * quat[3];
    float acc_x = acc[0];
    float acc_y = acc[1];
    float acc_z = acc[2];
    float grav_x;
    float grav_y;
    float grav_z;
    float err_x;
    float err_y;
    float err_z;
    float rate_x;
    float rate_y;
    float rate_z;
    float q0;
    float q1;
    float q2;
    float q3;
    float recip;
    float tilt;

    if ((prv_f2bits((acc_x * acc_y) * acc_z) & F32_INF) != 0U) {
        recip = fast_inv_sqrt(((acc_x * acc_x) + (acc_y * acc_y)) + (acc_z * acc_z));
        acc_x = acc_x * recip;
        acc_y = acc_y * recip;
        acc_z = acc_z * recip;

        grav_x = prv_scalbf(q1q3 - q0q2, 1);
        grav_y = prv_scalbf(q0q1 + q2q3, 1);
        grav_z = ((q0q0 - q1q1) - q2q2) + q3q3;
        g_imu_att.grav[0] = grav_x;
        g_imu_att.grav[1] = grav_y;
        g_imu_att.grav[2] = grav_z;

        err_x = (acc_y * grav_z) - (acc_z * grav_y);
        err_y = (acc_z * grav_x) - (acc_x * grav_z);
        err_z = (acc_x * grav_y) - (acc_y * grav_x);

        integ[0] = (err_x * IMU_MAHONY_KI) + integ[0];
        integ[1] = (err_y * IMU_MAHONY_KI) + integ[1];
        integ[2] = (err_z * IMU_MAHONY_KI) + integ[2];
        rate_x = ((IMU_MAHONY_KP * err_x) + gyro[0]) + integ[0];
        rate_y = ((IMU_MAHONY_KP * err_y) + gyro[1]) + integ[1];
        rate_z = ((IMU_MAHONY_KP * err_z) + gyro[2]) + integ[2];

        q2 = quat[2];
        q3 = quat[3];
        q0 = (((((-quat[1]) * rate_x) - (q2 * rate_y)) - (q3 * rate_z)) * IMU_MAHONY_HALFDT) + quat[0];
        q1 = ((((q0 * rate_x) + (q2 * rate_z)) - (q3 * rate_y)) * IMU_MAHONY_HALFDT) + quat[1];
        q2 = ((((q0 * rate_y) - (q1 * rate_z)) + (q3 * rate_x)) * IMU_MAHONY_HALFDT) + q2;
        q3 = ((((q0 * rate_z) + (q1 * rate_y)) - (q2 * rate_x)) * IMU_MAHONY_HALFDT) + q3;

        recip = fast_inv_sqrt(((q0q0 + q1q1) + q2q2) + q3q3);
        quat[0] = q0 * recip;
        quat[1] = q1 * recip;
        quat[2] = q2 * recip;
        quat[3] = q3 * recip;

        g_imu_att.grav_x_norm = fast_inv_sqrt((grav_x * grav_x) + (grav_z * grav_z)) * grav_x;
        tilt = (prv_atan2f(grav_x, grav_z) * IMU_TILT_DEG_PER_RAD) + IMU_TILT_HALF_TURN;
        g_imu_att.tilt_deg = tilt;
        if (tilt > IMU_TILT_WRAP) {
            tilt = tilt - IMU_TILT_TURN;
            g_imu_att.tilt_deg = tilt;
        }
        if (prv_is_neg_normal(tilt) != 0U) {
            tilt = -tilt;
        }
        if (tilt < IMU_TILT_AVG_LIMIT) {
            imu_tilt_avg_push(g_imu_att.tilt_deg);
        }
    }
}

/**
 * \brief           |a - b| on the negative-normal test
 * \param[in]       now: a
 * \param[in]       ref: b
 * \return          a - b, or b - a when a - b is a negative normal
 */
static float prv_abs_diff(float now, float ref) {
    float diff = now - ref;

    if (prv_is_neg_normal(diff) != 0U) {
        diff = ref - now;
    }
    return diff;
}

/**
 * \brief           Motion metric: accel change against a snapshot
 * \note            g_vs.sensor_delta_f = |(|dx| + |dy|) + |dz|| * 10000, d = accel (g) minus
 *                      the snapshot. The counter (initially 10) counts up; when it passes 15 it
 *                      restarts at 10 and the snapshot takes this pass' accel, so the snapshot
 *                      is refreshed every 6 passes. control_alarm_motion_tick compares the
 *                      metric with 20.0.
 */
static void imu_motion_metric_update(void) {
    float diff_z = prv_abs_diff(g_imu_acc_g[2], g_imu_metric.acc_ref[2]);
    float diff_y = prv_abs_diff(g_imu_acc_g[1], g_imu_metric.acc_ref[1]);
    float diff_x = prv_abs_diff(g_imu_acc_g[0], g_imu_metric.acc_ref[0]);
    float sum = (diff_x + diff_y) + diff_z;

    if (prv_is_neg_normal(sum) != 0U) {
        sum = -sum;
    }
    g_vs.sensor_delta_f = sum * IMU_METRIC_SCALE;

    g_imu_metric.tick = (uint8_t)(g_imu_metric.tick + 1U);
    if (g_imu_metric.tick > IMU_METRIC_TICK_LAST) {
        g_imu_metric.tick = (uint8_t)IMU_METRIC_TICK_RESET;
        g_imu_metric.acc_ref[0] = g_imu_acc_g[0];
        g_imu_metric.acc_ref[1] = g_imu_acc_g[1];
        g_imu_metric.acc_ref[2] = g_imu_acc_g[2];
    }
}

/**
 * \brief           Tilt class from the averaged tilt (part of vcu_sensor_attitude_update)
 * \note            States 5 / 4: cleared. States 3 / 2 / 7: mean <= 30 -> armed (2); mean > 90
 *                      while armed -> tripped (1); else kept. Other states keep it.
 */
static void imu_tilt_class_update(void) {
    uint8_t state = g_state_block.fsm_state;
    float mean = g_imu_att.tilt_mean_deg;

    if ((state == VCU_FSM_SLEEP) || (state == VCU_FSM_POWER_DOWN)) {
        g_vs.tilt_latch = TILT_LATCH_CLEAR;
    } else if ((state == VCU_FSM_RIDE) || (state == VCU_FSM_LOCKED) || (state == VCU_FSM_CHARGING)) {
        if (mean <= IMU_CLASS_UPRIGHT_DEG) {
            g_vs.tilt_latch = TILT_LATCH_ARMED;
        } else if ((mean > IMU_CLASS_FALLEN_DEG) && (g_vs.tilt_latch == TILT_LATCH_ARMED)) {
            g_vs.tilt_latch = TILT_LATCH_TRIPPED;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public API.                                                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Probe and configure the IMU (see sensor.h)
 */
void vcu_sensor_init(void) {
    static const uint8_t expect[4] = {0U, 0U, 0U, 0U}; /* zeroed expected readback */
    uint8_t tries;
    uint8_t probing = 1U;
    uint8_t found = 0U;

    imu_state_clear();
    g_imu_raw.temp = (int16_t)IMU_TEMP_SEED;

    while (probing != 0U) {
        if (imu_reg_read(LSM6DS3_WHO_AM_I, &g_imu_dev.whoami) != 0) {
            probing = 0U; /* bus error: returned, the retry not counted */
        } else {
            g_imu_dev.retry = (uint8_t)(g_imu_dev.retry + 1U);
            tries = g_imu_dev.retry;
            if (g_imu_dev.whoami == IMU_WHOAMI_J) {
                found = (uint8_t)(tries < IMU_PROBE_TRIES);
                probing = 0U;
            } else if (tries >= IMU_PROBE_TRIES) {
                probing = 0U; /* no 'j' part found */
            }
        }
    }

    if (found != 0U) {
        g_imu_dev.temp_div = IMU_TEMP_DIV_J;
        (void)imu_reg_write(LSM6DS3_CTRL3_C, 1U);
        (void)imu_reg_write(LSM6DS3_CTRL9_XL, IMU_CTRL_VERIFY);
        (void)imu_reg_write(LSM6DS3_CTRL10_C, IMU_CTRL_VERIFY);
        (void)imu_reg_write(LSM6DS3_TAP_THS_6D, 0x40U);
        (void)imu_reg_write(LSM6DS3_CTRL8_XL, 1U);
        imu_profile_apply();
        (void)imu_identify(expect); /* main discards the result */
    }
}

int32_t vcu_sensor_attitude_update(void) {
    int32_t rc;

    rc = imu_sample_update();
    if (rc == 0) {
        imu_mahony_update(g_imu_gyro_rads, g_imu_acc_g);
        g_imu_acc_norm_g = prv_sqrtf(((g_imu_acc_g[0] * g_imu_acc_g[0]) + (g_imu_acc_g[1] * g_imu_acc_g[1]))
                                     + (g_imu_acc_g[2] * g_imu_acc_g[2]));
        imu_motion_metric_update();
        if ((g_in2.f.chg_cfg & CHG_TILT_LATCH_EN) != 0U) {
            imu_tilt_class_update();
        }
    }
    return rc;
}

const imu_attitude_t* vcu_sensor_attitude(void) { return &g_imu_att; }

const float* vcu_sensor_quaternion(void) { return g_imu_mahony.quat; }

float vcu_sensor_tilt_deg(void) { return g_imu_att.tilt_mean_deg; }

int32_t vcu_sensor_gyro_raw(uint32_t axis) { return (axis < 3U) ? g_imu_raw.gyro[axis] : 0; }

float vcu_sensor_accel_norm_g(void) { return g_imu_acc_norm_g; }

int16_t vcu_sensor_temp(void) { return g_imu_raw.temp; }
