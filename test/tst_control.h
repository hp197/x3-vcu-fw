/**
 * \file            tst_control.h
 * \brief           Test helpers for the control loop (control.c): ADC input levels
 * \note            control.c reads the ADC1 scan buffer (control_adc_raw) filled by the ADC1 ->
 *                      DMA1 channel 1 stream. \ref tst_control_adc_freeze runs the product bring-up
 *                      (vcu_control_init) with every input at 0 and takes the buffer address from
 *                      the DMA channel's memory-address register. \ref tst_control_adc_set then
 *                      applies a level through the x3emu ADC op (tst_emu_adc_set) and waits until
 *                      the running scan has delivered it, so the product reads it as it would on the
 *                      device. Levels stay frozen until a test sets them again. No product seam.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_CONTROL_H
#define TST_CONTROL_H

#include <stdint.h>

void tst_control_adc_freeze(void);
void tst_control_adc_set(uint8_t slot, uint16_t raw);
void tst_control_adc_set_codes(uint8_t throttle, uint8_t brake_full, uint8_t brake_half);
void tst_control_adc_release(void);
volatile uint16_t* tst_control_adc_buf(void);

#endif /* TST_CONTROL_H */
