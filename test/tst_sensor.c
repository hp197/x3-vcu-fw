/**
 * \file            tst_sensor.c
 * \brief           SPI2 / IMU fakes at the Artery BSP boundary (GNU ld `--wrap`), see the header
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_sensor.h"
#include <math.h>
#include "at32f415.h"

#define TST_IMU_CMD_DATA   0xA0u                 /*!< burst read from OUT_TEMP_L (0x20 | read flag) */
#define TST_IMU_CMD_WHOAMI 0x8Fu                 /*!< read of WHO_AM_I (0x0F | read flag) */
#define TST_IMU_DATA_BYTES 14u                   /*!< bytes of the OUT_TEMP_L block */
#define TST_IMU_LSB_PER_G  16384.0f              /*!< the firmware decodes accel as ldexpf(raw, -14) g */
#define TST_DEG2RAD        0.017453292519943295f /*!< degrees to radians */
#define TST_RX_REAL        (-1)                  /*!< next receive forwards to the real BSP */

/* The real BSP functions (GNU ld --wrap). */
void __real_spi_i2s_data_transmit(spi_type* spi_x, uint16_t tx_data);
uint16_t __real_spi_i2s_data_receive(spi_type* spi_x);
flag_status __real_spi_i2s_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag);

/* The wrappers the product's calls land in (prototypes for -Wmissing-prototypes). */
void __wrap_spi_i2s_data_transmit(spi_type* spi_x, uint16_t tx_data);
uint16_t __wrap_spi_i2s_data_receive(spi_type* spi_x);
flag_status __wrap_spi_i2s_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag);

static uint8_t g_mosi[TST_SPI_MOSI_MAX];     /*!< SPI2 transmit log */
static uint32_t g_mosi_count;                /*!< SPI2 transmits seen */
static uint8_t g_stall;                      /*!< status flags held clear */
static uint8_t g_sample_on;                  /*!< data block comes from g_sample */
static uint8_t g_sample[TST_IMU_DATA_BYTES]; /*!< little-endian data block */
static uint8_t g_whoami[TST_IMU_WHOAMI_MAX]; /*!< scripted WHO_AM_I answers */
static uint8_t g_whoami_count;               /*!< answers scripted (0 = real chip) */
static uint8_t g_whoami_next;                /*!< next answer index */
static uint8_t g_mode_cmd;                   /*!< command of the transaction being decoded */
static uint32_t g_data_index;                /*!< data bytes clocked since the command */
static int32_t g_rx_override = TST_RX_REAL;  /*!< value of the next receive, or real */

/**
 * \brief           Forward everything again and clear the logs and scripts
 */
void tst_sensor_fakes_reset(void) {
    g_mosi_count = 0u;
    g_stall = 0u;
    g_sample_on = 0u;
    g_whoami_count = 0u;
    g_whoami_next = 0u;
    g_mode_cmd = 0u;
    g_data_index = 0u;
    g_rx_override = TST_RX_REAL;
}

/** \brief SPI2 transmits seen since the reset \return the count (also beyond the log size) */
uint32_t tst_spi_mosi_count(void) { return g_mosi_count; }

/**
 * \brief           One logged SPI2 transmit byte
 * \param[in]       index: position in the log
 * \return          the byte, or 0 past the log
 */
uint8_t tst_spi_mosi_get(uint32_t index) {
    uint8_t value = 0u;

    if ((index < g_mosi_count) && (index < TST_SPI_MOSI_MAX)) {
        value = g_mosi[index];
    }
    return value;
}

/**
 * \brief           Hold the SPI2 status flags clear (a dead bus) or release them
 * \param[in]       stall: 1 = every `spi_i2s_flag_get` on SPI2 reports RESET
 */
void tst_spi_stall(uint8_t stall) { g_stall = stall; }

/**
 * \brief           Present this raw block on every burst read of OUT_TEMP_L
 * \param[in]       words: TEMP, GX, GY, GZ, AX, AY, AZ in chip LSB (little-endian on the wire)
 */
void tst_imu_sample_set(const int16_t words[TST_IMU_WORDS]) {
    uint32_t idx;

    for (idx = 0u; idx < TST_IMU_WORDS; ++idx) {
        g_sample[2u * idx] = (uint8_t)((uint16_t)words[idx] & 0xFFu);
        g_sample[(2u * idx) + 1u] = (uint8_t)((uint16_t)words[idx] >> 8);
    }
    g_sample_on = 1u;
}

/** \brief Let the emulated chip answer the data block again */
void tst_imu_sample_clear(void) { g_sample_on = 0u; }

/**
 * \brief           Script the answers of the next WHO_AM_I reads
 * \param[in]       answers: one byte per read; the last one repeats
 * \param[in]       count: number of answers (0 = the emulated chip answers)
 */
void tst_imu_whoami_set(const uint8_t* answers, uint8_t count) {
    uint8_t idx;

    if (count > TST_IMU_WHOAMI_MAX) {
        count = TST_IMU_WHOAMI_MAX;
    }
    for (idx = 0u; idx < count; ++idx) {
        g_whoami[idx] = answers[idx];
    }
    g_whoami_count = count;
    g_whoami_next = 0u;
}

/**
 * \brief           Chip-axis raw words of a board at rest at the given attitude
 * \param[out]      words: TEMP 0, gyro 0, accel (AX, AY, AZ) at 2^14 LSB/g
 * \param[in]       roll_deg: roll the firmware should decode
 * \param[in]       pitch_deg: pitch the firmware should decode
 * \note            The firmware reads accel as (-AX, AY, AZ), publishes g = (-AY, AX, -AZ)
 *                      and takes pitch = -asin(g0/|g|), roll = atan2(g1, g2). Inverted for a
 *                      still board: AX = cos(p) sin(r), AY = sin(p), AZ = -cos(p) cos(r)
 *                      (the same as x3emu's accel_for_attitude).
 */
void tst_imu_level_sample(int16_t* words, float roll_deg, float pitch_deg) {
    float roll = roll_deg * TST_DEG2RAD;
    float pitch = pitch_deg * TST_DEG2RAD;
    uint32_t idx;

    for (idx = 0u; idx < 4u; ++idx) {
        words[idx] = 0;
    }
    words[4] = (int16_t)lroundf(cosf(pitch) * sinf(roll) * TST_IMU_LSB_PER_G);
    words[5] = (int16_t)lroundf(sinf(pitch) * TST_IMU_LSB_PER_G);
    words[6] = (int16_t)lroundf(-cosf(pitch) * cosf(roll) * TST_IMU_LSB_PER_G);
}

/**
 * \brief           Decide what the receive after this transmit returns
 * \param[in]       byte: the byte clocked out
 * \note            A command byte starts a transaction: 0xA0 (data block) and 0x8F (WHO_AM_I)
 *                      may be overridden; the product clocks 0x00 for every read data byte, and
 *                      no register write it does uses 0xA0 or 0x8F as a value.
 */
static void prv_decode(uint8_t byte) {
    g_rx_override = TST_RX_REAL;
    if ((byte == TST_IMU_CMD_DATA) || (byte == TST_IMU_CMD_WHOAMI)) {
        g_mode_cmd = byte;
        g_data_index = 0u;
    } else if ((byte == 0u) && (g_mode_cmd == TST_IMU_CMD_DATA) && (g_data_index < TST_IMU_DATA_BYTES)) {
        if (g_sample_on != 0u) {
            g_rx_override = (int32_t)g_sample[g_data_index];
        }
        ++g_data_index;
    } else if ((byte == 0u) && (g_mode_cmd == TST_IMU_CMD_WHOAMI) && (g_data_index == 0u)) {
        if (g_whoami_count != 0u) {
            g_rx_override = (int32_t)g_whoami[g_whoami_next];
            if ((uint8_t)(g_whoami_next + 1u) < g_whoami_count) {
                ++g_whoami_next;
            }
        }
        ++g_data_index;
    } else {
        g_mode_cmd = 0u;
    }
}

/**
 * \brief           Wrapped `spi_i2s_data_transmit`: log SPI2 bytes, forward
 * \param[in]       spi_x: SPI instance
 * \param[in]       tx_data: data to send
 */
void __wrap_spi_i2s_data_transmit(spi_type* spi_x, uint16_t tx_data) {
    if (spi_x == SPI2) {
        if (g_mosi_count < TST_SPI_MOSI_MAX) {
            g_mosi[g_mosi_count] = (uint8_t)tx_data;
        }
        ++g_mosi_count;
        prv_decode((uint8_t)tx_data);
    }
    __real_spi_i2s_data_transmit(spi_x, tx_data);
}

/**
 * \brief           Wrapped `spi_i2s_data_receive`: the real byte, or the scripted one
 * \param[in]       spi_x: SPI instance
 * \return          received data (the real read always runs, so RDBF is cleared)
 */
uint16_t __wrap_spi_i2s_data_receive(spi_type* spi_x) {
    uint16_t value = __real_spi_i2s_data_receive(spi_x);

    if ((spi_x == SPI2) && (g_rx_override != TST_RX_REAL)) {
        value = (uint16_t)g_rx_override;
        g_rx_override = TST_RX_REAL;
    }
    return value;
}

/**
 * \brief           Wrapped `spi_i2s_flag_get`: RESET on SPI2 while stalled, else forward
 * \param[in]       spi_x: SPI instance
 * \param[in]       spi_i2s_flag: flag to test
 * \return          flag state
 */
flag_status __wrap_spi_i2s_flag_get(spi_type* spi_x, uint32_t spi_i2s_flag) {
    flag_status state = RESET;

    if ((spi_x != SPI2) || (g_stall == 0u)) {
        state = __real_spi_i2s_flag_get(spi_x, spi_i2s_flag);
    }
    return state;
}
