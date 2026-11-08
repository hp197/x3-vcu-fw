/**
 * \file            test_clock.c
 * \brief           System clock bring-up (clock.c) behaviour (smoke)
 * \note            tst_init() has already run clock_config() when the tests start, as the
 *                      product main does. Read back from the emulated CRM and FLASH.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "at32f415.h"
#include "board.h"
#include "clock.h"
#include "tst_support.h"

#define PSR_EXPECTED   0x14u /* *0x40022000 = 0x14 (4 wait cycles + bit4) */
#define PSR_CFG_MASK   0x17u /* wtcyc [2:0] and bit4 (bit3 / bit5 not written) */
#define PLL_SRC_HEXT   1u    /* pll_config(1, 7): source 1 = HEXT */
#define PLL_MULT_9_LO  7u    /* pll_config(1, 7): multiplier code 7 = x9 */
#define BUS_DIV_2      4u    /* apb2_div(4) / apb1_div(4): APB2 / APB1 = AHB / 2 */
#define HICK_SPIN_LOOP 100000u

/**
 * \brief           Nothing to set up: the clock state is the one tst_init() left
 */
void setUp(void) {}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/**
 * \brief           clock_config ends on the PLL at 144 MHz: HEXT x9, AHB /1, APB1/APB2 /2, 4 wait states
 * \note            crm_reset, psr = 0x14, HEXT on + stable, pll_config(1, 7),
 *                      PLL on + stable (flag 0x19), ahb_div(0), apb2_div(4), apb1_div(4),
 *                      auto-step on, sclk_switch(2), wait sclksts == 2, auto-step off, then the
 *                      inline SystemCoreClock update.
 */
static void test_config_144mhz_pll(void) {
    TEST_ASSERT_EQUAL_UINT32(SYSCLK_HZ, SystemCoreClock);
    TEST_ASSERT_EQUAL(CRM_SCLK_PLL, crm_sysclk_switch_status_get());
    TEST_ASSERT_EQUAL_UINT32(PLL_SRC_HEXT, CRM->cfg_bit.pllrcs);
    TEST_ASSERT_EQUAL_UINT32(PLL_MULT_9_LO, CRM->cfg_bit.pllmult_l);
    TEST_ASSERT_EQUAL_UINT32(0u, CRM->cfg_bit.pllmult_h);
    TEST_ASSERT_EQUAL_UINT32(0u, CRM->cfg_bit.ahbdiv);
    TEST_ASSERT_EQUAL_UINT32(BUS_DIV_2, CRM->cfg_bit.apb1div);
    TEST_ASSERT_EQUAL_UINT32(BUS_DIV_2, CRM->cfg_bit.apb2div);
    TEST_ASSERT_EQUAL(SET, crm_flag_get(CRM_HEXT_STABLE_FLAG));
    TEST_ASSERT_EQUAL(SET, crm_flag_get(CRM_PLL_STABLE_FLAG));
    TEST_ASSERT_EQUAL_HEX32(PSR_EXPECTED, FLASH->psr & PSR_CFG_MASK);
}

/**
 * \brief           After a drop to HICK with HEXT and PLL off, clock_resume returns to the 144 MHz PLL
 * \note            HEXT on + stable, PLL on + stable, auto-step, sclk_switch(2), wait, auto-step
 *                      off. The dividers survive in CRM->cfg.
 */
static void test_resume_relocks_pll(void) {
    uint32_t spins = 0u;

    crm_sysclk_switch(CRM_SCLK_HICK);
    while ((crm_sysclk_switch_status_get() != CRM_SCLK_HICK) && (spins < HICK_SPIN_LOOP)) {
        ++spins;
    }
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, FALSE);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, FALSE);
    TEST_ASSERT_EQUAL(CRM_SCLK_HICK, crm_sysclk_switch_status_get());
    clock_resume();
    TEST_ASSERT_EQUAL(CRM_SCLK_PLL, crm_sysclk_switch_status_get());
    TEST_ASSERT_EQUAL(SET, crm_flag_get(CRM_PLL_STABLE_FLAG));
    TEST_ASSERT_EQUAL_UINT32(PLL_MULT_9_LO, CRM->cfg_bit.pllmult_l);
    TEST_ASSERT_EQUAL_UINT32(BUS_DIV_2, CRM->cfg_bit.apb1div);
}

/**
 * \brief           clock_resume re-programs the PLL (HEXT x9) before it re-enables it
 * \note            After HEXT is stable, crm_pll_config(HEXT, x9), then PLL on. The test leaves a
 *                      HICK x4 setting behind and expects the 144 MHz tree back.
 */
static void test_resume_reprograms_pll(void) {
    uint32_t spins = 0u;

    crm_sysclk_switch(CRM_SCLK_HICK);
    while ((crm_sysclk_switch_status_get() != CRM_SCLK_HICK) && (spins < HICK_SPIN_LOOP)) {
        ++spins;
    }
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, FALSE);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, FALSE);
    crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_4);
    TEST_ASSERT_EQUAL_UINT32(0u, CRM->cfg_bit.pllrcs);
    clock_resume();
    TEST_ASSERT_EQUAL(CRM_SCLK_PLL, crm_sysclk_switch_status_get());
    TEST_ASSERT_EQUAL_UINT32(PLL_SRC_HEXT, CRM->cfg_bit.pllrcs);
    TEST_ASSERT_EQUAL_UINT32(PLL_MULT_9_LO, CRM->cfg_bit.pllmult_l);
    TEST_ASSERT_EQUAL_UINT32(0u, CRM->cfg_bit.pllmult_h);
    clock_config(); /* whatever happened, leave the 144 MHz tree for the next test */
}

/**
 * \brief           clock_resume retries the PLL and the SYSCLK switch after 0x3000 failed polls
 * \note            PLL-stable poll count `>= 0x3000` -> count 0, crm_pll_config(1, 7) and PLL on
 *                      again; the same bound on the SYSCLK-switch poll -> sclk_switch(2) again.
 *                      HEXT: enable + crm_hext_stable_wait (bounded 0x3000) repeated until stable.
 */
static void test_resume_retries_bounded_waits(void) {
    TEST_IGNORE_MESSAGE("x3emu gap: HEXT/PLL always stabilise at once, no start-up failure injection");
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_config_144mhz_pll);
    RUN_TEST(test_resume_relocks_pll);
    RUN_TEST(test_resume_reprograms_pll);
    RUN_TEST(test_resume_retries_bounded_waits);
    tst_exit(UNITY_END());
    return 0;
}
