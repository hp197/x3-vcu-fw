/**
 * \file            tst_faultmon.c
 * \brief           Input stimuli for the fault-monitor and vehicle FSM suites (see tst_faultmon.h)
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_faultmon.h"
#include <stddef.h>
#include "control.h"
#include "tst_control.h"

/**
 * \brief           Present a level on input pin(s) through the internal pull resistor
 * \param[in]       port: GPIO port
 * \param[in]       pins: pin mask (GPIO_PINS_x)
 * \param[in]       level: \ref TST_LEVEL_LOW or \ref TST_LEVEL_HIGH
 */
void tst_fm_pin(gpio_type* port, uint16_t pins, uint8_t level) {
    gpio_init_type init;

    gpio_default_para_init(&init);
    init.gpio_pins = pins;
    init.gpio_mode = GPIO_MODE_INPUT;
    init.gpio_pull = (level != TST_LEVEL_LOW) ? GPIO_PULL_UP : GPIO_PULL_DOWN;
    gpio_init(port, &init);
}

/**
 * \brief           Every line the fault monitors read back to its released (high) level
 * \note            PA9..PA12, PB1, PB2, PB6, PB7, PB11 (faultmon.c accessors), and every ADC
 *                      input back to 0 once a test has started the scan (\ref tst_fm_adc_set).
 */
void tst_fm_release_all(void) {
    tst_fm_pin(GPIOA, GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12, TST_LEVEL_HIGH);
    tst_fm_pin(GPIOB, GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_11, TST_LEVEL_HIGH);
    tst_control_adc_release();
}

/**
 * \brief           Set one ADC scan-buffer reading through its channel's input level
 * \param[in]       slot: VCU_ADC_SLOT_* index
 * \param[in]       raw: 12-bit reading
 * \note            The first call brings the product ADC scan up (\ref tst_control_adc_freeze);
 *                      returns once the scan has delivered the reading.
 */
void tst_fm_adc_set(uint8_t slot, uint16_t raw) {
    if (tst_control_adc_buf() == NULL) {
        tst_control_adc_freeze();
    }
    tst_control_adc_set(slot, raw);
}
