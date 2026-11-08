/**
 * \file            tst_faultmon.h
 * \brief           Input stimuli for the fault-monitor and vehicle FSM suites: GPIO input levels
 *                      and ADC scan-buffer readings
 * \note            Inputs are applied the way the board would present them, without a seam
 *                      in src/: a monitored GPIO line is configured as an input with the
 *                      internal pull-up or pull-down, which the emulated input buffer reads
 *                      as that level (x3emu GpioPort.input_level); an ADC reading is applied as
 *                      the input level of its channel (x3emu ADC op, tst_control.h), which the
 *                      product's own ADC1 scan and DMA (vcu_control_init) deliver to the buffer.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_FAULTMON_H
#define TST_FAULTMON_H

#include <stdint.h>
#include "at32f415.h"

#define TST_LEVEL_LOW  0u /*!< pin reads 0 (an active-low line is asserted / "lost") */
#define TST_LEVEL_HIGH 1u /*!< pin reads 1 (released / healthy)                       */

void tst_fm_pin(gpio_type* port, uint16_t pins, uint8_t level);
void tst_fm_release_all(void);
void tst_fm_adc_set(uint8_t slot, uint16_t raw);

#endif /* TST_FAULTMON_H */
