/**
 * \file            test_sensor.c
 * \brief           IMU over SPI2 and the attitude estimator (sensor.c)
 * \note            Covers the bus bring-up, bus select, byte transfer, register protocol,
 *                      probe + configure (profile, identity), read path, accel compensation +
 *                      moving averages, Mahony filter, motion metric and the attitude pass with
 *                      its tilt class.
 *
 *                      The estimator tests compare raw IEEE-754 bits with the fixed reference
 *                      outputs in sensor_gold.h: per scenario a chip sample, FSM state,
 *                      tilt-class enable and speed schedule, checked after a given number of
 *                      passes.
 *                      The emulated LSM6DS3 answers the bus by default; tst_sensor.h scripts the
 *                      data block, WHO_AM_I and bus stalls at the SPI2 BSP calls.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "sensor.h"
#include "sensor_gold.h"
#include "tst_bsp_fakes.h"
#include "tst_sensor.h"
#include "tst_support.h"
#include "vcu_state.h"

#define WHOAMI_LSM6DS3   0x69u /*!< 'i' */
#define WHOAMI_LSM6DSL   0x6Au /*!< 'j' */
#define INIT_MOSI_BYTES  34u   /*!< bytes vcu_sensor_init clocks out for a 'j' on the first read */
#define PROBE_MOSI_BYTES 2u    /*!< one WHO_AM_I read: command + dummy */
#define PROBE_TRIES      2u    /*!< WHO_AM_I reads per probe */
#define FSM_LOCKED       2u    /*!< vehicle FSM state 2 (tilt class active) */
#define FSM_RIDE         3u    /*!< vehicle FSM state 3 (tilt class active) */
#define TILT_UPRIGHT     2u    /*!< class 2: averaged tilt <= 30 deg */
#define TILT_FALLEN      1u    /*!< class 1: averaged tilt > 90 deg after class 2 */
#define MSG_LEN          48u   /*!< assertion message buffer */

/** \brief Tilt tolerance against the reference (degrees): float rounding, the rest is bit for bit */
#define GOLD_TILT_TOL_DEG 1e-3f

static char g_msg[MSG_LEN]; /*!< "<scenario> pass <n> <field>" */

/** \brief PA15 (chip select 1) output latch \return 1 when high */
static uint32_t prv_cs1(void) { return (GPIOA->odt >> 15) & 1u; }

/** \brief PF7 (chip select 2, the IMU) output latch \return 1 when high */
static uint32_t prv_cs2(void) { return (GPIOF->odt >> 7) & 1u; }

/**
 * \brief           IEEE-754 bits of a float
 * \param[in]       value: the float
 * \return          its bit pattern
 */
static uint32_t prv_bits(float value) {
    uint32_t bits;

    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/**
 * \brief           Float from its IEEE-754 bits
 * \param[in]       bits: the bit pattern
 * \return          the float
 */
static float prv_float(uint32_t bits) {
    float value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/**
 * \brief           Append a string to the message buffer
 * \param[in,out]   pos: write position
 * \param[in]       text: NUL-terminated text
 */
static void prv_msg_put(uint32_t* pos, const char* text) {
    while ((*text != '\0') && (*pos < (MSG_LEN - 1u))) {
        g_msg[*pos] = *text;
        ++(*pos);
        ++text;
    }
    g_msg[*pos] = '\0';
}

/**
 * \brief           Build "<name> pass <n> <field>" for an assertion
 * \param[in]       name: scenario name
 * \param[in]       pass: pass number
 * \param[in]       field: output name
 * \return          the message buffer
 */
static const char* prv_msg(const char* name, uint32_t pass, const char* field) {
    char digits[8];
    uint32_t pos = 0u;
    uint32_t len = 0u;

    do {
        digits[len] = (char)('0' + (pass % 10u));
        pass /= 10u;
        ++len;
    } while ((pass != 0u) && (len < sizeof digits));

    prv_msg_put(&pos, name);
    prv_msg_put(&pos, " pass ");
    while ((len != 0u) && (pos < (MSG_LEN - 1u))) {
        --len;
        g_msg[pos] = digits[len];
        ++pos;
    }
    g_msg[pos] = ' ';
    ++pos;
    g_msg[pos] = '\0';
    prv_msg_put(&pos, field);
    return g_msg;
}

/**
 * \brief           Run the estimator a number of times
 * \param[in]       passes: calls of vcu_sensor_attitude_update
 */
static void prv_passes(uint32_t passes) {
    uint32_t idx;

    for (idx = 0u; idx < passes; ++idx) {
        (void)vcu_sensor_attitude_update();
    }
}

/**
 * \brief           Read one register of the emulated chip through the product's bus primitives
 * \param[in]       reg: register address
 * \return          its value
 */
static uint8_t prv_chip_read(uint8_t reg) {
    uint8_t value = 0u;

    vcu_spi2_bus_select(VCU_SPI2_SEL_IMU);
    (void)vcu_spi2_xfer((uint8_t)(reg | 0x80u), &value);
    (void)vcu_spi2_xfer(0u, &value);
    vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
    return value;
}

/** \brief Probe and configure the IMU as main does, then clear the SPI log */
static void prv_init(void) {
    vcu_sensor_init();
    tst_sensor_fakes_reset();
}

/**
 * \brief           Speed of a reference scenario at a pass
 * \param[in]       scenario: the scenario
 * \param[in]       pass: pass number
 * \return          g_in2.f.speed_src_s for that pass
 */
static int16_t prv_speed_at(const gold_scenario_t* scenario, uint32_t pass) {
    int16_t speed = 0;
    uint32_t idx;

    for (idx = 0u; idx < scenario->speed_count; ++idx) {
        if (pass >= scenario->speed[idx].from_pass) {
            speed = scenario->speed[idx].speed;
        }
    }
    return speed;
}

/**
 * \brief           Compare the outputs after a pass with a reference checkpoint
 * \param[in]       name: scenario name
 * \param[in]       point: the expected outputs
 */
static void prv_check_point(const char* name, const gold_point_t* point) {
    const imu_attitude_t* att = vcu_sensor_attitude();
    const float* quat = vcu_sensor_quaternion();
    uint32_t idx;

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(GOLD_TILT_TOL_DEG, prv_float(point->tilt), att->tilt_deg,
                                     prv_msg(name, point->pass, "tilt"));
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(GOLD_TILT_TOL_DEG, prv_float(point->tilt_mean), vcu_sensor_tilt_deg(),
                                     prv_msg(name, point->pass, "tilt mean"));
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(point->norm, prv_bits(vcu_sensor_accel_norm_g()),
                                    prv_msg(name, point->pass, "|a|"));
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(point->metric, prv_bits(g_vs.sensor_delta_f), prv_msg(name, point->pass, "metric"));
    for (idx = 0u; idx < 4u; ++idx) {
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(point->quat[idx], prv_bits(quat[idx]), prv_msg(name, point->pass, "quat"));
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(point->cls, g_vs.tilt_latch, prv_msg(name, point->pass, "class"));
}

/**
 * \brief           Run a reference scenario and compare every checkpoint
 * \param[in]       scenario: the run to reproduce
 */
static void prv_run_gold(const gold_scenario_t* scenario) {
    uint32_t pass = 0u;
    uint32_t point = 0u;

    prv_init();
    g_state_block.fsm_state = scenario->fsm_state;
    g_in2.f.chg_cfg = (scenario->tilt_cfg != 0u) ? CHG_TILT_LATCH_EN : 0u;
    tst_imu_sample_set(scenario->sample);
    while (point < scenario->point_count) {
        ++pass;
        g_in2.f.speed_src_s = prv_speed_at(scenario, pass);
        TEST_ASSERT_EQUAL_INT32(0, vcu_sensor_attitude_update());
        if (pass == scenario->points[point].pass) {
            prv_check_point(scenario->name, &scenario->points[point]);
            ++point;
        }
    }
}

/**
 * \brief           The reference scenario with this name
 * \param[in]       name: scenario name
 * \return          the scenario (the test fails if it is missing)
 */
static const gold_scenario_t* prv_gold(const char* name) {
    const gold_scenario_t* found = NULL;
    uint32_t idx;

    for (idx = 0u; idx < (sizeof gold_scenarios / sizeof gold_scenarios[0]); ++idx) {
        if (strcmp(gold_scenarios[idx].name, name) == 0) {
            found = &gold_scenarios[idx];
        }
    }
    TEST_ASSERT_NOT_NULL_MESSAGE(found, name);
    return found;
}

/**
 * \brief           Fresh boot state, SPI2 brought up as vcu_periph_init does, fakes forwarding
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_sensor_fakes_reset();
    vcu_sensor_bus_init();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- SPI2 bus ------------------------------------------------------------------------------- */

/**
 * \brief           Bring-up: SPI2 master, mode 3, /8, software CS, both chip selects high
 * \note            Pins, SPI2 init (struct bytes 0,1,2,0,0,1,1,1 then ctrl2 & 0xFFFB), then
 *                      bus select 0.
 */
static void test_bus_init_smoke(void) {
    TEST_ASSERT_EQUAL_UINT32(1u, SPI2->ctrl1_bit.spien);
    TEST_ASSERT_EQUAL_UINT32(1u, SPI2->ctrl1_bit.msten);
    TEST_ASSERT_EQUAL_UINT32(1u, SPI2->ctrl1_bit.clkpol);
    TEST_ASSERT_EQUAL_UINT32(1u, SPI2->ctrl1_bit.clkpha);
    TEST_ASSERT_EQUAL_UINT32(2u, SPI2->ctrl1_bit.mdiv_l);
    TEST_ASSERT_EQUAL_UINT32(1u, SPI2->ctrl1_bit.swcsen);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs1());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs2());
}

/**
 * \brief           Bus select: 0 releases both, 1 selects PA15, 2 selects PF7, others change nothing
 * \note            Mode 0 -> set 0x8000 (GPIOA) and 0x80 (GPIOF); 1 -> reset
 *                      PA15, set PF7; 2 -> reset PF7, set PA15; any other mode returns.
 */
static void test_bus_select_modes(void) {
    vcu_spi2_bus_select(VCU_SPI2_SEL_DEV_A);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_cs1());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs2());

    vcu_spi2_bus_select(3);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_cs1());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs2());

    vcu_spi2_bus_select(VCU_SPI2_SEL_IMU);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs1());
    TEST_ASSERT_EQUAL_UINT32(0u, prv_cs2());

    vcu_spi2_bus_select(VCU_SPI2_SEL_IDLE);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs1());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs2());
}

/**
 * \brief           A register read is `reg | 0x80` then a dummy 0x00, the answer on the second byte
 * \note            Select 2, xfer(reg | 0x80, NULL), xfer(0, val), select 0. The emulated
 *                      chip answers WHO_AM_I 0x6A.
 */
static void test_xfer_register_read(void) {
    TEST_ASSERT_EQUAL_HEX8(WHOAMI_LSM6DSL, prv_chip_read(0x0Fu));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_spi_mosi_count());
    TEST_ASSERT_EQUAL_HEX8(0x8Fu, tst_spi_mosi_get(0u));
    TEST_ASSERT_EQUAL_HEX8(0x00u, tst_spi_mosi_get(1u));
}

/**
 * \brief           A transfer without a receive buffer still reads DT (RDBF cleared) and succeeds
 * \note            With a NULL rx the received byte is discarded: result 0, RDBF clear,
 *                      nothing stored.
 */
static void test_xfer_null_rx(void) {
    TEST_ASSERT_EQUAL_UINT8(0u, vcu_spi2_xfer(0x5Au, NULL));
    TEST_ASSERT_EQUAL_UINT32(0u, SPI2->sts_bit.rdbf);
    TEST_ASSERT_EQUAL_HEX8(0x5Au, tst_spi_mosi_get(0u));
}

/**
 * \brief           A stalled bus times out with 1 before anything is sent
 * \note            TDBE is polled while a count of 0xFA runs down to 0, then the transfer
 *                      returns 1 without writing DT.
 */
static void test_xfer_timeout(void) {
    uint8_t value = 0xEEu;

    tst_spi_stall(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, vcu_spi2_xfer(0x8Fu, &value));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_spi_mosi_count());
    TEST_ASSERT_EQUAL_HEX8(0xEEu, value);
}

/* -- probe and configuration ---------------------------------------------------------------- */

/**
 * \brief           An LSM6DSL ('j') is configured with the full register sequence
 * \note            WHO_AM_I read (0x0F); writes 0x12=1, 0x18=0x38, 0x19=0x38, 0x59=0x40,
 *                      0x17=1; profile: 0x5C=0, 0x5B=4, 0x58=0x90, burst 0x10 = 51 7C 64 04,
 *                      0x5E=0x20; identity: reads 0x18, 0x19, then a 4-byte read from 0x10.
 *                      Write commands have bit 7 clear.
 */
static void test_init_register_sequence(void) {
    static const uint8_t expect[INIT_MOSI_BYTES] = {0x8F, 0x00, 0x12, 0x01, 0x18, 0x38, 0x19, 0x38, 0x59,
                                                    0x40, 0x17, 0x01, 0x5C, 0x00, 0x5B, 0x04, 0x58, 0x90,
                                                    0x10, 0x51, 0x7C, 0x64, 0x04, 0x5E, 0x20, 0x98, 0x00,
                                                    0x99, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00};
    uint32_t idx;

    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(INIT_MOSI_BYTES, tst_spi_mosi_count());
    for (idx = 0u; idx < INIT_MOSI_BYTES; ++idx) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(expect[idx], tst_spi_mosi_get(idx), "MOSI byte differs");
    }
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs1());
    TEST_ASSERT_EQUAL_UINT32(1u, prv_cs2());
}

/**
 * \brief           The emulated chip holds the profile afterwards: accel 208 Hz 2 g, gyro 833 Hz 2000 dps
 * \note            Burst {0x51, 0x7C, 0x64, 0x04} to CTRL1_XL..CTRL4_C, 0x58 = 0x90,
 *                      0x5E = 0x20.
 */
static void test_init_profile_on_chip(void) {
    vcu_sensor_init();

    TEST_ASSERT_EQUAL_HEX8(0x51u, prv_chip_read(0x10u));
    TEST_ASSERT_EQUAL_HEX8(0x7Cu, prv_chip_read(0x11u));
    TEST_ASSERT_EQUAL_HEX8(0x64u, prv_chip_read(0x12u));
    TEST_ASSERT_EQUAL_HEX8(0x04u, prv_chip_read(0x13u));
    TEST_ASSERT_EQUAL_HEX8(0x01u, prv_chip_read(0x17u));
    TEST_ASSERT_EQUAL_HEX8(0x90u, prv_chip_read(0x58u));
    TEST_ASSERT_EQUAL_HEX8(0x20u, prv_chip_read(0x5Eu));
}

/**
 * \brief           An LSM6DS3 ('i') is probed twice and never configured
 * \note            Not 'j' and `retry < 2` -> probe again, else give up (3). No CTRL write.
 */
static void test_init_lsm6ds3_not_configured(void) {
    static const uint8_t answers[1] = {WHOAMI_LSM6DS3};
    uint32_t idx;

    tst_imu_whoami_set(answers, 1u);
    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(PROBE_TRIES * PROBE_MOSI_BYTES, tst_spi_mosi_count());
    for (idx = 0u; idx < PROBE_TRIES; ++idx) {
        TEST_ASSERT_EQUAL_HEX8(0x8Fu, tst_spi_mosi_get(idx * PROBE_MOSI_BYTES));
    }
}

/**
 * \brief           'j' on the first read is accepted, on the second it is not
 * \note            On 'j', `retry >= 2` -> give up (3). After the first read retry = 1 ->
 *                      configure; after the second retry = 2 -> give up.
 */
static void test_init_probe_retry_limit(void) {
    static const uint8_t second[2] = {0x00, WHOAMI_LSM6DSL};

    tst_imu_whoami_set(second, 2u);
    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(PROBE_TRIES * PROBE_MOSI_BYTES, tst_spi_mosi_count());
    TEST_ASSERT_EQUAL_HEX8(0x8Fu, tst_spi_mosi_get(PROBE_MOSI_BYTES));
}

/**
 * \brief           The probe counter is never reset: a second init probes once and gives up
 * \note            The retry counter (initially 0) is only incremented, so after a
 *                      configured first init (retry 1) the next read makes it 2 and the 'j'
 *                      branch gives up without writing a CTRL register.
 */
static void test_init_retry_counter_persists(void) {
    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(INIT_MOSI_BYTES, tst_spi_mosi_count());

    tst_sensor_fakes_reset();
    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(PROBE_MOSI_BYTES, tst_spi_mosi_count());
}

/**
 * \brief           A bus error on the probe returns at once and does not use up a retry
 * \note            The bus error skips the retry increment, so a later init still
 *                      configures the chip.
 */
static void test_init_bus_error(void) {
    tst_spi_stall(1u);
    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_spi_mosi_count());

    tst_spi_stall(0u);
    vcu_sensor_init();
    TEST_ASSERT_EQUAL_UINT32(INIT_MOSI_BYTES, tst_spi_mosi_count());
}

/* -- read path ------------------------------------------------------------------------------- */

/**
 * \brief           The data block is one 14-byte burst from 0x20 (command 0xA0)
 * \note            imu_burst_read(0x20, buf, 0xE).
 */
static void test_read_burst(void) {
    static const int16_t level[TST_IMU_WORDS] = {0, 0, 0, 0, 0, 0, -16384};
    uint32_t idx;

    prv_init();
    tst_imu_sample_set(level);
    (void)vcu_sensor_attitude_update();
    TEST_ASSERT_EQUAL_UINT32(15u, tst_spi_mosi_count());
    TEST_ASSERT_EQUAL_HEX8(0xA0u, tst_spi_mosi_get(0u));
    for (idx = 1u; idx < 15u; ++idx) {
        TEST_ASSERT_EQUAL_HEX8(0x00u, tst_spi_mosi_get(idx));
    }
}

/**
 * \brief           Temperature: raw * 10 / 256 + 250 for an 'j' part, through the 15/16 IIR
 * \note            `t = (t * 15 + (short)(raw * 10 / temp_div + 250) + 7) >> 4`; the probe
 *                      sets temp_div = 0x0100 for 'j', init seeds t = 0xFA. raw 1280 (30 C):
 *                      250 -> 253 -> 256.
 */
static void test_temperature_lsm6dsl(void) {
    static const int16_t words[TST_IMU_WORDS] = {1280, 0, 0, 0, 0, 0, -16384};

    prv_init();
    TEST_ASSERT_EQUAL_INT16(250, vcu_sensor_temp());
    tst_imu_sample_set(words);
    (void)vcu_sensor_attitude_update();
    TEST_ASSERT_EQUAL_INT16(253, vcu_sensor_temp());
    (void)vcu_sensor_attitude_update();
    TEST_ASSERT_EQUAL_INT16(256, vcu_sensor_temp());
}

/**
 * \brief           An unconfigured 'i' part keeps the default divisor 16
 * \note            The divisor starts at 16; only the 'j' branch writes 256.
 *                      raw 80 -> 80 * 10 / 16 + 250 = 300 -> IIR 253.
 */
static void test_temperature_lsm6ds3_divisor(void) {
    static const uint8_t answers[1] = {WHOAMI_LSM6DS3};
    static const int16_t words[TST_IMU_WORDS] = {80, 0, 0, 0, 0, 0, -16384};

    tst_imu_whoami_set(answers, 1u);
    vcu_sensor_init();
    tst_imu_sample_set(words);
    (void)vcu_sensor_attitude_update();
    TEST_ASSERT_EQUAL_INT16(253, vcu_sensor_temp());
}

/**
 * \brief           A bus error on the data read returns it and changes nothing
 * \note            imu_sample_update returns the read error first, and the attitude pass
 *                      returns it without running the filter.
 */
static void test_read_bus_error(void) {
    prv_init();
    tst_spi_stall(1u);
    TEST_ASSERT_EQUAL_INT32(1, vcu_sensor_attitude_update());
    TEST_ASSERT_EQUAL_INT16(250, vcu_sensor_temp());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vcu_sensor_accel_norm_g());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vcu_sensor_tilt_deg());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, g_vs.sensor_delta_f);
}

/**
 * \brief           Raw gyro in the VCU frame: GX, -GY, -GZ
 * \note            Block words 1..3 are stored as axis 0, axis 1 (negated), axis 2 (negated).
 *                      turn_signal_auto_cancel_tick reads axis 2, ride_assist_tick axis 1.
 */
static void test_gyro_raw_frame(void) {
    static const int16_t words[TST_IMU_WORDS] = {0, 300, -500, 200, 9000, 3000, -13000};

    prv_init();
    tst_imu_sample_set(words);
    (void)vcu_sensor_attitude_update();
    TEST_ASSERT_EQUAL_INT32(300, vcu_sensor_gyro_raw(0u));
    TEST_ASSERT_EQUAL_INT32(500, vcu_sensor_gyro_raw(1u));
    TEST_ASSERT_EQUAL_INT32(-200, vcu_sensor_gyro_raw(2u));
}

/* -- estimator against the reference outputs (sensor_gold.h) --------------------------------- */

/**
 * \brief           Board on its side (AX +14189, AZ 8192): tilt rises to +120 deg, class 2 -> 1
 * \note            Pass 1 has the start quaternion (tilt 180, not averaged: |tilt| < 179
 *                      fails), then the Mahony filter (Kp 4.0 Ki 0.1, half-dt 0.005) turns over
 *                      and the 100-sample mean passes 90 at pass 95 (class 1). Bits of tilt,
 *                      mean, |a|, metric, q, class.
 */
static void test_gold_tilt_positive(void) { prv_run_gold(prv_gold("tilt_pos")); }

/**
 * \brief           Mirror board (AX -14189): the start transient trips class 1, the -120 deg board re-arms 2
 * \note            Pass 2 averages +178 (> 90 after class 2 -> 1), at pass 21 the mean falls
 *                      to 28 (<= 30 -> 2), and the negative tilt afterwards always classes 2.
 */
static void test_gold_tilt_negative(void) { prv_run_gold(prv_gold("tilt_neg")); }

/**
 * \brief           Turning board (gyro 300, -500, 200 LSB) in FSM state 7
 * \note            rate = (float)gyro * 0.0012217 (70 mdps/LSB), fed with the proportional
 *                      and integral feedback into the quaternion integration.
 */
static void test_gold_gyro(void) { prv_run_gold(prv_gold("gyro")); }

/**
 * \brief           Speed steps feed the accel compensation, recomputed every 11 passes
 * \note            The counter (initially 10) counts 10..20; at 20 it takes d = speed -
 *                      previous speed, adds d*320 to raw X and d*240 to raw Z from then on, then
 *                      clamps each axis to +-5000 around the last averaged value and pushes it
 *                      into the 100-sample moving average. Tilt class off.
 */
static void test_gold_speed(void) { prv_run_gold(prv_gold("speed")); }

/**
 * \brief           A level board with zero X/Y never runs the filter; the metric still runs
 * \note            `(ax * ay) * az` with a zero exponent skips the whole update (q stays
 *                      1,0,0,0, tilt 0). Motion metric: |a - a snapshot| summed, * 10000,
 *                      snapshot refreshed every 6 passes (counter counts 10..16). The accel IIR
 *                      is seeded with -value but then adds +value.
 */
static void test_gold_level(void) { prv_run_gold(prv_gold("level")); }

/**
 * \brief           Tilt class: cleared in states 4/5, else only states 2/3/7 classify
 * \note            On the averaged tilt: states 5, 4 -> 0; states 3, 2, 7: <= 30 -> 2, > 90
 *                      and class 2 -> 1, otherwise kept; other states keep it. The state
 *                      sequence 5,1,7,3,4,2,0,6,8 with the mean at about 118 deg reads 0 after
 *                      every pass.
 */
static void test_tilt_class_gating(void) {
    static const uint8_t states[9] = {5u, 1u, 7u, 3u, 4u, 2u, 0u, 6u, 8u};
    static const int16_t side[TST_IMU_WORDS] = {0, 0, 0, 0, 14189, 200, 8192};
    uint32_t idx;

    prv_init();
    g_state_block.fsm_state = FSM_RIDE;
    g_in2.f.chg_cfg = CHG_TILT_LATCH_EN;
    tst_imu_sample_set(side);
    prv_passes(120u);
    TEST_ASSERT_EQUAL_UINT8(TILT_FALLEN, g_vs.tilt_latch);
    TEST_ASSERT_TRUE(vcu_sensor_tilt_deg() > 90.0f);

    for (idx = 0u; idx < 9u; ++idx) {
        g_state_block.fsm_state = states[idx];
        (void)vcu_sensor_attitude_update();
        TEST_ASSERT_EQUAL_UINT8(0u, g_vs.tilt_latch);
    }

    g_vs.tilt_latch = TILT_UPRIGHT;
    g_state_block.fsm_state = FSM_LOCKED;
    (void)vcu_sensor_attitude_update();
    TEST_ASSERT_EQUAL_UINT8(TILT_FALLEN, g_vs.tilt_latch);
}

/**
 * \brief           Without CHG_TILT_LATCH_EN the class is never written
 * \note            The enable bit is tested before the class code runs.
 */
static void test_tilt_class_disabled(void) {
    static const int16_t side[TST_IMU_WORDS] = {0, 0, 0, 0, 14189, 200, 8192};

    prv_init();
    g_state_block.fsm_state = FSM_RIDE;
    g_in2.f.chg_cfg = (uint16_t)~CHG_TILT_LATCH_EN;
    g_vs.tilt_latch = TILT_UPRIGHT;
    tst_imu_sample_set(side);
    prv_passes(120u);
    TEST_ASSERT_TRUE(vcu_sensor_tilt_deg() > 90.0f);
    TEST_ASSERT_EQUAL_UINT8(TILT_UPRIGHT, g_vs.tilt_latch);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_bus_init_smoke);
    RUN_TEST(test_bus_select_modes);
    RUN_TEST(test_xfer_register_read);
    RUN_TEST(test_xfer_null_rx);
    RUN_TEST(test_xfer_timeout);
    RUN_TEST(test_init_register_sequence);
    RUN_TEST(test_init_profile_on_chip);
    RUN_TEST(test_init_lsm6ds3_not_configured);
    RUN_TEST(test_init_probe_retry_limit);
    RUN_TEST(test_init_retry_counter_persists);
    RUN_TEST(test_init_bus_error);
    RUN_TEST(test_read_burst);
    RUN_TEST(test_temperature_lsm6dsl);
    RUN_TEST(test_temperature_lsm6ds3_divisor);
    RUN_TEST(test_read_bus_error);
    RUN_TEST(test_gyro_raw_frame);
    RUN_TEST(test_gold_tilt_positive);
    RUN_TEST(test_gold_tilt_negative);
    RUN_TEST(test_gold_gyro);
    RUN_TEST(test_gold_speed);
    RUN_TEST(test_gold_level);
    RUN_TEST(test_tilt_class_gating);
    RUN_TEST(test_tilt_class_disabled);
    tst_exit(UNITY_END());
    return 0;
}
