/**
 * \file            test_buzzer.c
 * \brief           Buzzer (buzzer.c) behaviour
 * \note            Covers bring-up (pin + timer), tone reconfigure, tone on, tone off and the
 *                      beep request. Read back from the emulated TMR1 / GPIOA.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "buzzer.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"

#define TIMER_CLK_HZ  1000000u /*!< effective TMR1 clock */
#define PA8_CFG_MASK  0xFu     /*!< cfghr bits for pin 8 */
#define PA8_CFG_AF_PP 0x9u     /*!< CNF 10 (mux push-pull), MODE 01 (drive "stronger") */

/**
 * \brief           Fresh boot state and the buzzer brought up as vcu_periph_init does
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    buzzer_init();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/**
 * \brief           Bring-up: PA8 TMR1_CH1 alternate function, 1 MHz tick, counter on, output gated
 * \note            TMR1 base init (pr 1, div 143), channel 1 with oc mode 6 (PWM A) and output
 *                      state 0, brkdt.oen and counter enable. PA8 init word 0x01080400 (push-pull,
 *                      no pull, mux, stronger).
 */
static void test_init_smoke(void) {
    TEST_ASSERT_EQUAL_HEX32(PA8_CFG_AF_PP, GPIOA->cfghr & PA8_CFG_MASK);
    TEST_ASSERT_EQUAL_UINT32(143u, TMR1->div);
    TEST_ASSERT_EQUAL_UINT32(1u, TMR1->ctrl1_bit.tmren);
    TEST_ASSERT_EQUAL_UINT32(6u, TMR1->cm1_output_bit.c1octrl);
    TEST_ASSERT_EQUAL_UINT32(0u, TMR1->cctrl_bit.c1en);
    TEST_ASSERT_EQUAL_UINT32(1u, TMR1->brk_bit.oen);
}

/**
 * \brief           The 2700 Hz tone: period 1e6 / 2700 = 370, compare 185, counter reset, output on
 * \note            pr (+0x2C) = 1000000 / tone, c1dt (+0x34) = that >> 1, cval (+0x24) = 0,
 *                      ctrl1 bit 0 cleared then set; then cctrl bit 0 set. 0xA8C is the tone every
 *                      buzzer_on caller passes.
 */
static void test_on_programs_tone_and_enables_output(void) {
    buzzer_on(BUZZER_TONE);

    TEST_ASSERT_EQUAL_UINT16(2700u, BUZZER_TONE);
    TEST_ASSERT_EQUAL_UINT32(TIMER_CLK_HZ / 2700u, TMR1->pr);
    TEST_ASSERT_EQUAL_UINT32(370u, TMR1->pr);
    TEST_ASSERT_EQUAL_UINT32(185u, TMR1->c1dt);
    TEST_ASSERT_EQUAL_UINT32(1u, TMR1->ctrl1_bit.tmren);
    TEST_ASSERT_EQUAL_UINT32(1u, TMR1->cctrl_bit.c1en);
}

/**
 * \brief           Another tone uses the same integer formula (truncating division, 50 % duty)
 * \note            Unsigned division of 1000000 by the tone, compare = period >> 1.
 */
static void test_on_period_formula_truncates(void) {
    buzzer_on(3000u);

    TEST_ASSERT_EQUAL_UINT32(333u, TMR1->pr);
    TEST_ASSERT_EQUAL_UINT32(166u, TMR1->c1dt);
}

/**
 * \brief           Off clears only the channel output enable; the timer keeps counting
 * \note            `cctrl &= ~1`, nothing else.
 */
static void test_off_gates_output_only(void) {
    buzzer_on(BUZZER_TONE);
    buzzer_off();

    TEST_ASSERT_EQUAL_UINT32(0u, TMR1->cctrl_bit.c1en);
    TEST_ASSERT_EQUAL_UINT32(1u, TMR1->ctrl1_bit.tmren);
    TEST_ASSERT_EQUAL_UINT32(370u, TMR1->pr);
}

/**
 * \brief           A new beep request latches the pattern id and rewinds the progress counter
 * \note            When byte [0] of the request block differs, store the id and zero bytes [2]
 *                      and [3] (the u16 progress counter).
 */
static void test_beep_new_pattern_rewinds(void) {
    g_lowglob.beep_seq_id = BEEP_SHORT;
    g_lowglob.beep_seq_tick = 42u;

    buzzer_beep(BEEP_DOUBLE);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.beep_seq_tick);
}

/**
 * \brief           Repeating the latched pattern does not restart it
 * \note            Equal id -> no store at all.
 */
static void test_beep_same_pattern_keeps_progress(void) {
    g_lowglob.beep_seq_id = BEEP_LONG;
    g_lowglob.beep_seq_tick = 42u;

    buzzer_beep(BEEP_LONG);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(42u, g_lowglob.beep_seq_tick);
}

/**
 * \brief           The request does not touch the buzzer hardware
 * \note            The request only writes the request block; the pattern player (control.c)
 *                      drives TMR1.
 */
static void test_beep_does_not_drive_tmr1(void) {
    buzzer_beep(BEEP_LONG);

    TEST_ASSERT_EQUAL_UINT32(0u, TMR1->cctrl_bit.c1en);
}

/**
 * \brief           The pattern id compare is on the low byte only
 * \note            The latched id byte is compared with the full argument, then the low byte is
 *                      stored. Because the compare is against the untruncated argument, 0x103
 *                      differs from a latched 3 and re-latches 3 (low byte) with a rewind.
 */
static void test_beep_stores_low_byte(void) {
    g_lowglob.beep_seq_id = BEEP_DOUBLE;
    g_lowglob.beep_seq_tick = 9u;

    buzzer_beep(0x103);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.beep_seq_tick);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_init_smoke);
    RUN_TEST(test_on_programs_tone_and_enables_output);
    RUN_TEST(test_on_period_formula_truncates);
    RUN_TEST(test_off_gates_output_only);
    RUN_TEST(test_beep_new_pattern_rewinds);
    RUN_TEST(test_beep_same_pattern_keeps_progress);
    RUN_TEST(test_beep_does_not_drive_tmr1);
    RUN_TEST(test_beep_stores_low_byte);
    tst_exit(UNITY_END());
    return 0;
}
