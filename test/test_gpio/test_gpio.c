/**
 * \file            test_gpio.c
 * \brief           GPIO + EXINT (gpio.c) behaviour
 * \note            Edges are raised with the EXINT software trigger (x3emu sets the pending
 *                      flag of an enabled line), so the product ISRs run through the NVIC as on
 *                      the device. Inputs without an applied level read their pull (x3emu GPIO).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "at32f415.h"
#include "gpio.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"

#define LINE_3         0x0008u /* EXINT line 3 (PA3) */
#define LINE_10        0x0400u /* EXINT line 10 (PA10) */
#define LINE_13        0x2000u /* EXINT line 13 (PC13) */
#define LINES_ALL      0x7FFFFu
#define IRQ_EXINT3     9
#define IRQ_EXINT15_10 40
#define CFG_IN_PULL    0x8u /* cfg field: iomc 0 (input), iofc 2 (pull) */
#define CFG_IN_FLOAT   0x4u /* cfg field: iomc 0 (input), iofc 1 (floating) */
#define IOFC_SHIFT     2u
#define CFG_FIELD_MASK 0xFu
#define PORT_A         0u /* IOMUX exintc source: GPIOA */
#define PORT_C         2u /* IOMUX exintc source: GPIOC */
#define CFG_PC13_BIT15 0x8000u
#define CFG_PC13_BIT2  0x0004u
#define ISR_SETTLE_US  100u

/**
 * \brief           The 4-bit cfglr/cfghr field of one pin
 * \param[in]       port: GPIO port
 * \param[in]       pin: pin number 0..15
 * \return          iofc << 2 | iomc
 */
static uint32_t prv_pin_cfg(const gpio_type* port, uint32_t pin) {
    uint32_t reg = (pin < 8u) ? port->cfglr : port->cfghr;

    return (reg >> ((pin % 8u) * 4u)) & CFG_FIELD_MASK;
}

/**
 * \brief           Assert a pin is a general-purpose push-pull output
 * \param[in]       port: GPIO port
 * \param[in]       pin: pin number
 */
static void prv_assert_output_pp(const gpio_type* port, uint32_t pin) {
    uint32_t cfg = prv_pin_cfg(port, pin);

    TEST_ASSERT_NOT_EQUAL_UINT32(0u, cfg & 0x3u);             /* iomc != 0: output */
    TEST_ASSERT_EQUAL_UINT32(0u, (cfg >> IOFC_SHIFT) & 0x3u); /* iofc 0: GP push-pull */
}

/**
 * \brief           Raise a software edge on EXINT lines and give the ISR time to run
 * \param[in]       lines: EXINT line mask
 */
static void prv_edge(uint32_t lines) {
    EXINT->swtrg = lines;
    tst_delay_us(ISR_SETTLE_US);
}

/**
 * \brief           Configure PA10 as an input with the given pull
 * \param[in]       pull: GPIO_PULL_UP / GPIO_PULL_DOWN
 */
static void prv_pa10_input(gpio_pull_type pull) {
    gpio_init_type init;

    gpio_default_para_init(&init);
    init.gpio_pins = GPIO_PINS_10;
    init.gpio_mode = GPIO_MODE_INPUT;
    init.gpio_pull = pull;
    gpio_init(GPIOA, &init);
}

/**
 * \brief           Fresh product state; EXINT lines, routing and the two NVIC lines idle
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    NVIC_DisableIRQ(EXINT3_IRQn);
    NVIC_DisableIRQ(EXINT15_10_IRQn);
    EXINT->inten = 0u;
    EXINT->polcfg2 = 0u;
    EXINT->swtrg = 0u;
    EXINT->intsts = LINES_ALL;
    NVIC_ClearPendingIRQ(EXINT3_IRQn);
    NVIC_ClearPendingIRQ(EXINT15_10_IRQn);
    IOMUX->exintc1 = 0u;
    IOMUX->exintc3 = 0u;
    IOMUX->exintc4 = 0xFFFFu; /* not GPIOC on line 13 unless the init routes it */
}

/**
 * \brief           Leave the EXINT lines quiet for the next test
 */
void tearDown(void) {
    EXINT->inten = 0u;
    EXINT->intsts = LINES_ALL;
}

/* -- EXINT3 / PA3 ------------------------------------------------------------------------- */

/**
 * \brief           EXINT3 init: PA3 pull-up input, line 3 from GPIOA, falling edge, IRQ9 on
 * \note            gpio struct {pins 8, pull 0x18, mode 0}, line config (0, 3), exint struct
 *                      {mode 0, line 8, polarity 1 = falling, enable 1}, NVIC 9 (1, 0), line 3
 *                      interrupt enabled, pending flag cleared.
 */
static void test_exint3_init(void) {
    vcu_gpio_exint3_init();
    TEST_ASSERT_EQUAL_HEX32(CFG_IN_PULL, prv_pin_cfg(GPIOA, 3u));
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_3, GPIOA->odt & GPIO_PINS_3);
    TEST_ASSERT_EQUAL_UINT32(PORT_A, IOMUX->exintc1_bit.exint3);
    TEST_ASSERT_EQUAL_HEX32(LINE_3, EXINT->inten & LINE_3);
    TEST_ASSERT_EQUAL_HEX32(LINE_3, EXINT->polcfg2 & LINE_3);
    TEST_ASSERT_EQUAL_UINT32(1u, NVIC_GetEnableIRQ(EXINT3_IRQn));
    TEST_ASSERT_EQUAL_HEX32(0u, EXINT->intsts & LINE_3);
}

/**
 * \brief           A PA3 edge latches wake code 4 and masks IRQ9: a second edge is not handled
 * \note            `if (intsts & 8) { intsts = 8; VS+8 = 4; NVIC_Disable(9); }`.
 */
static void test_exint3_isr_one_shot(void) {
    vcu_gpio_exint3_init();
    prv_edge(LINE_3);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_PA3, g_vs.wake_src);
    TEST_ASSERT_EQUAL_HEX32(0u, EXINT->intsts & LINE_3);
    TEST_ASSERT_EQUAL_UINT32(0u, NVIC_GetEnableIRQ(EXINT3_IRQn));
    g_vs.wake_src = VCU_GPIO_EVENT_NONE;
    prv_edge(LINE_3);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_NONE, g_vs.wake_src);
    vcu_gpio_exint3_init(); /* the standby path re-arms the line */
    prv_edge(LINE_3);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_PA3, g_vs.wake_src);
}

/* -- EXINT10 / EXINT13 -------------------------------------------------------------------- */

/**
 * \brief           The PC13 gate is IN2 +0x3A bit15 or bit2
 */
static void test_pc13_gate_bits(void) {
    volatile uint8_t* cfg = (volatile uint8_t*)&g_in2 + 0x3Au;

    cfg[0] = 0u;
    cfg[1] = 0u;
    TEST_ASSERT_EQUAL_UINT8(0u, vcu_board_pc13_wake_present());
    cfg[1] = (uint8_t)(CFG_PC13_BIT15 >> 8);
    TEST_ASSERT_NOT_EQUAL_UINT8(0u, vcu_board_pc13_wake_present());
    cfg[1] = 0u;
    cfg[0] = (uint8_t)CFG_PC13_BIT2;
    TEST_ASSERT_NOT_EQUAL_UINT8(0u, vcu_board_pc13_wake_present());
    cfg[0] = 0xFBu;
    cfg[1] = 0x7Fu;
    TEST_ASSERT_EQUAL_UINT8(0u, vcu_board_pc13_wake_present());
}

/**
 * \brief           Without the gate bits only line 10 is armed; PC13 is still a pull-up input
 * \note            PA10 {pull 4 = none, input}, PC13 {pull 0x18} always; line config (0, 10)
 *                      only; exint line 0x400; NVIC 0x28; clear 0x400.
 */
static void test_exint_10_13_without_pc13(void) {
    vcu_gpio_exint_10_13_init();
    TEST_ASSERT_EQUAL_HEX32(CFG_IN_FLOAT, prv_pin_cfg(GPIOA, 10u));
    TEST_ASSERT_EQUAL_HEX32(CFG_IN_PULL, prv_pin_cfg(GPIOC, 13u));
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_13, GPIOC->odt & GPIO_PINS_13);
    TEST_ASSERT_EQUAL_UINT32(PORT_A, IOMUX->exintc3_bit.exint10);
    TEST_ASSERT_NOT_EQUAL_UINT32(PORT_C, IOMUX->exintc4_bit.exint13);
    TEST_ASSERT_EQUAL_HEX32(LINE_10, EXINT->inten & (LINE_10 | LINE_13));
    TEST_ASSERT_EQUAL_HEX32(LINE_10, EXINT->polcfg2 & (LINE_10 | LINE_13));
    TEST_ASSERT_EQUAL_UINT32(1u, NVIC_GetEnableIRQ(EXINT15_10_IRQn));
}

/**
 * \brief           With IN2 +0x3A bit15 set, line 13 is routed to GPIOC and armed with line 10
 * \note            Line config (0, 10) + (2, 13), exint line 0x2400, clears 0x400 and 0x2000.
 */
static void test_exint_10_13_with_pc13(void) {
    ((volatile uint8_t*)&g_in2)[0x3Bu] = (uint8_t)(CFG_PC13_BIT15 >> 8);
    vcu_gpio_exint_10_13_init();
    TEST_ASSERT_EQUAL_UINT32(PORT_A, IOMUX->exintc3_bit.exint10);
    TEST_ASSERT_EQUAL_UINT32(PORT_C, IOMUX->exintc4_bit.exint13);
    TEST_ASSERT_EQUAL_HEX32(LINE_10 | LINE_13, EXINT->inten & (LINE_10 | LINE_13));
    TEST_ASSERT_EQUAL_HEX32(LINE_10 | LINE_13, EXINT->polcfg2 & (LINE_10 | LINE_13));
}

/**
 * \brief           A PA10 edge latches wake code 2 and masks IRQ40
 * \note            intsts & 0x400 -> VS+8 = 2, clear 0x400, NVIC_Disable(0x28).
 */
static void test_isr_pa10_code(void) {
    vcu_gpio_exint_10_13_init();
    prv_edge(LINE_10);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_PA10, g_vs.wake_src);
    TEST_ASSERT_EQUAL_HEX32(0u, EXINT->intsts & LINE_10);
    TEST_ASSERT_EQUAL_UINT32(0u, NVIC_GetEnableIRQ(EXINT15_10_IRQn));
    g_vs.wake_src = VCU_GPIO_EVENT_NONE;
    prv_edge(LINE_10);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_NONE, g_vs.wake_src);
}

/**
 * \brief           A PC13 edge latches wake code 1; with both pending, PC13's code is the one left
 * \note            Line 13 is tested after line 10 and writes VS+8 = 1.
 */
static void test_isr_pc13_code_and_order(void) {
    ((volatile uint8_t*)&g_in2)[0x3Au] = (uint8_t)CFG_PC13_BIT2;
    vcu_gpio_exint_10_13_init();
    prv_edge(LINE_13);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_PC13, g_vs.wake_src);
    TEST_ASSERT_EQUAL_UINT32(0u, NVIC_GetEnableIRQ(EXINT15_10_IRQn));
    g_vs.wake_src = VCU_GPIO_EVENT_NONE;
    vcu_gpio_exint_10_13_init();
    NVIC_DisableIRQ(EXINT15_10_IRQn);
    EXINT->swtrg = LINE_10 | LINE_13;
    NVIC_EnableIRQ(EXINT15_10_IRQn);
    tst_delay_us(ISR_SETTLE_US);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_PC13, g_vs.wake_src);
    TEST_ASSERT_EQUAL_HEX32(0u, EXINT->intsts & (LINE_10 | LINE_13));
}

/* -- PA10 level + outputs ----------------------------------------------------------------- */

/**
 * \brief           PA10 counts as active while it reads LOW
 * \note            Pin 10 high -> 0, else 1.
 */
static void test_pa10_active_low(void) {
    prv_pa10_input(GPIO_PULL_UP);
    TEST_ASSERT_EQUAL_INT32(0, vcu_gpio_pa10_active());
    prv_pa10_input(GPIO_PULL_DOWN);
    TEST_ASSERT_EQUAL_INT32(1, vcu_gpio_pa10_active());
}

/**
 * \brief           PB12 power latch: push-pull output, level left as it was
 * \note            gpio struct {pins 0x1000, out 0, pull 4, mode 0x10, drive 1} on GPIOB, no
 *                      level write (vcu_periph_init sets it afterwards).
 */
static void test_power_latch_init(void) {
    GPIOB->scr = GPIO_PINS_12;
    vcu_gpio_power_latch_init();
    prv_assert_output_pp(GPIOB, 12u);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_12, GPIOB->odt & GPIO_PINS_12);
    GPIOB->clr = GPIO_PINS_12;
    vcu_gpio_power_latch_init();
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_12);
}

/**
 * \brief           PB13 / PA6 / PB14 indicators: push-pull outputs, levels left as they were
 * \note            Three output inits; vcu_periph_init drives them low afterwards.
 */
static void test_indicator_outputs_init(void) {
    GPIOB->scr = GPIO_PINS_13 | GPIO_PINS_14;
    GPIOA->scr = GPIO_PINS_6;
    vcu_gpio_indicator_outputs_init();
    prv_assert_output_pp(GPIOB, 13u);
    prv_assert_output_pp(GPIOA, 6u);
    prv_assert_output_pp(GPIOB, 14u);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_13 | GPIO_PINS_14, GPIOB->odt & (GPIO_PINS_13 | GPIO_PINS_14));
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_6, GPIOA->odt & GPIO_PINS_6);
}

/**
 * \brief           PC14 fault latch: push-pull output for mode 0
 * \note            gpio struct {pins 0x4000, out = mode, pull 4, mode 0x10, drive 1} on GPIOC.
 */
static void test_fault_latch_init(void) {
    vcu_gpio_fault_latch_init(0u);
    prv_assert_output_pp(GPIOC, 14u);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_exint3_init);
    RUN_TEST(test_exint3_isr_one_shot);
    RUN_TEST(test_pc13_gate_bits);
    RUN_TEST(test_exint_10_13_without_pc13);
    RUN_TEST(test_exint_10_13_with_pc13);
    RUN_TEST(test_isr_pa10_code);
    RUN_TEST(test_isr_pc13_code_and_order);
    RUN_TEST(test_pa10_active_low);
    RUN_TEST(test_power_latch_init);
    RUN_TEST(test_indicator_outputs_init);
    RUN_TEST(test_fault_latch_init);
    tst_exit(UNITY_END());
    return 0;
}
