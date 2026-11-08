/**
 * \file            test_vcu_state.c
 * \brief           Shared blackboard (vcu_state.c / vcu_state.h): boot values and field aliases
 * \note            Boot values are the initialised cells set at reset or by the init
 *                      functions. The register-file aliases are the Ninebot wire layout
 *                      (register n at IN2 + 2n), which the register handlers address directly.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "can.h" /* vcu_can_init: the candev timers */
#include "charging.h"
#include "passcode.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"

/**
 * \brief           Fresh boot state
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/**
 * \brief           Initialised cells come back with their boot values after a reset
 * \note            Candev timers 200 at +0x12 / +0x14; passcode reloads 0x30, 0x20, 0x10. The
 *                      candev timers are .bss and set by vcu_can_init (main calls it before the
 *                      scheduler).
 */
static void test_boot_data_values(void) {
    g_candev.timer_a = 1;
    g_passcode_ch.reload[2] = 0u;
    tst_reset_product_state();
    vcu_can_init();

    TEST_ASSERT_EQUAL_INT16(200, g_candev.timer_a);
    TEST_ASSERT_EQUAL_INT16(200, g_candev.timer_b);
    TEST_ASSERT_EQUAL_HEX16(0x0030u, g_passcode_ch.reload[0]);
    TEST_ASSERT_EQUAL_HEX16(0x0020u, g_passcode_ch.reload[1]);
    TEST_ASSERT_EQUAL_HEX16(0x0010u, g_passcode_ch.reload[2]);
}

/**
 * \brief           Uninitialised blackboards start at zero
 * \note            These blocks (VS, charge timer, beep request) have no initialiser and the
 *                      .bss is zeroed.
 */
static void test_boot_bss_zero(void) {
    g_vs.speed_kmh = 9u;
    g_charge_timer.charge_min = 9u;
    g_lowglob.beep_seq_id = 9u;
    tst_reset_product_state();

    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.speed_kmh);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.region);
    TEST_ASSERT_EQUAL_UINT16(0u, g_charge_timer.charge_min);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(0u, g_mode.enable_flags);
}

/**
 * \brief           The IN2 named fields are Ninebot registers: 0x1C..0x1F, 0x55, 0x5D
 * \note            Register handlers address IN2 + 2n: status 0x1C (+0x38), settings 0x1D
 *                      (+0x3A), settings 2 0x1E (+0x3C), charge settings 0x1F (+0x3E, holds the
 *                      tilt latch), SoC 0x55 (+0xAA), brake flash 0x5D (+0xBA).
 */
static void test_in2_register_aliases(void) {
    g_in2.reg[0x1Cu] = 0x1111u;
    g_in2.reg[0x1Du] = 0x2222u;
    g_in2.reg[0x1Eu] = 0x3333u;
    g_in2.reg[0x1Fu] = 0x4444u;
    g_in2.reg[0x55u] = 0xFFFBu;
    g_in2.reg[0x5Du] = 0x0001u;

    TEST_ASSERT_EQUAL_HEX16(0x1111u, g_in2.f.status_flags);
    TEST_ASSERT_EQUAL_HEX16(0x2222u, g_in2.f.cfg_flags);
    TEST_ASSERT_EQUAL_HEX16(0x3333u, g_in2.f.cfg_flags2);
    TEST_ASSERT_EQUAL_HEX16(0x4444u, g_in2.f.chg_cfg);
    TEST_ASSERT_EQUAL_INT16(-5, g_in2.f.soc_s);
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_in2.f.brake_light_flash);
}

/**
 * \brief           mode_light_set writes bits 9..11 and keeps the rest of MODE+0x10
 * \note            Every writer does `(x & 0xF1FF) + (mode << 9)`.
 */
static void test_mode_light_set_field(void) {
    uint16_t mode;

    g_mode.enable_flags = 0xF1FFu;
    for (mode = MODE_LIGHT_OFF; mode <= MODE_LIGHT_BRAKE_FLASH; ++mode) {
        mode_light_set(mode);
        TEST_ASSERT_EQUAL_HEX16((uint16_t)(0xF1FFu | (mode << 9)), g_mode.enable_flags);
    }

    g_mode.enable_flags = 0x0000u;
    mode_light_set(MODE_LIGHT_BRAKE_FLASH);
    mode_light_set(MODE_LIGHT_TAIL);
    TEST_ASSERT_EQUAL_HEX16(0x0400u, g_mode.enable_flags);
}

/**
 * \brief           The rear-light codes are the adders 0, 0x200, 0x400, 0x600, 0x800
 * \note            `+ 0x200` charging blink, `+ 0x400` tail, `+ 0x600`
 *                      brake / charged, `+ 0x800` flashing brake.
 */
static void test_mode_light_codes(void) {
    TEST_ASSERT_EQUAL_HEX16(0x0E00u, MODE_LIGHT_MASK);
    TEST_ASSERT_EQUAL_UINT16(0x200u >> 9, MODE_LIGHT_BLINK);
    TEST_ASSERT_EQUAL_UINT16(0x400u >> 9, MODE_LIGHT_TAIL);
    TEST_ASSERT_EQUAL_UINT16(0x600u >> 9, MODE_LIGHT_BRAKE);
    TEST_ASSERT_EQUAL_UINT16(0x800u >> 9, MODE_LIGHT_BRAKE_FLASH);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_boot_data_values);
    RUN_TEST(test_boot_bss_zero);
    RUN_TEST(test_in2_register_aliases);
    RUN_TEST(test_mode_light_set_field);
    RUN_TEST(test_mode_light_codes);
    tst_exit(UNITY_END());
    return 0;
}
