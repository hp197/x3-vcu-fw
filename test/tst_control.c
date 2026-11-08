/**
 * \file            tst_control.c
 * \brief           Test helpers for the control loop (control.c): ADC input levels
 * \note            See tst_control.h. Call \ref tst_control_adc_freeze after
 *                      tst_fakes_reset() (the DMA enable is a recording fake that only forwards
 *                      to the BSP once the fakes are reset).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_control.h"
#include <stddef.h>
#include "at32f415.h"
#include "control.h"
#include "tst_support.h"

/** \brief Raw 12-bit code per 8-bit code step: control.c works on `raw >> 4`. */
#define TST_ADC_CODE_SHIFT 4u
/** \brief Largest 12-bit conversion result (a higher level converts as this). */
#define TST_ADC_FULL_SCALE 0x0FFFu

/** \brief ADC1 input channel scanned into each buffer slot (vcu_control_init sequence). */
static const uint8_t prv_slot_channel[VCU_ADC_SCAN_LEN] = {
    (uint8_t)VCU_ADC_CH_THROTTLE,    (uint8_t)VCU_ADC_CH_BRAKE_HALF, (uint8_t)VCU_ADC_CH_BRAKE_FULL,
    (uint8_t)VCU_ADC_CH_ISENSE_PB10, (uint8_t)VCU_ADC_CH_ISENSE_PB0,
};

/** \brief The product's scan buffer, as found in the DMA memory address register. */
static volatile uint16_t* prv_adc_buf;

/**
 * \brief           Apply a level to a slot's channel, without waiting for the scan
 * \param[in]       slot: `VCU_ADC_SLOT_*`
 * \param[in]       raw: 12-bit conversion result
 */
static void prv_level(uint8_t slot, uint16_t raw) {
    if ((prv_adc_buf != NULL) && (slot < VCU_ADC_SCAN_LEN)) {
        tst_emu_adc_set(prv_slot_channel[slot], raw);
    }
}

/**
 * \brief           Wait until the running scan has written `raw` into `slot`
 * \note            One five-channel scan at 239.5 sample cycles takes about 0.1 ms; the
 *                      budget (\ref TST_WAIT_LOOPS_DEFAULT, ~10 ms) covers many.
 * \param[in]       slot: `VCU_ADC_SLOT_*`
 * \param[in]       raw: level applied to the slot's channel
 */
static void prv_settle(uint8_t slot, uint16_t raw) {
    uint16_t expect = (raw > TST_ADC_FULL_SCALE) ? (uint16_t)TST_ADC_FULL_SCALE : raw;
    uint32_t loops = 0u;

    if ((prv_adc_buf != NULL) && (slot < VCU_ADC_SCAN_LEN)) {
        while ((prv_adc_buf[slot] != expect) && (loops < TST_WAIT_LOOPS_DEFAULT)) {
            ++loops;
        }
    }
}

/**
 * \brief           Bring the ADC front-end up as the product does, with every scanned input at 0
 * \note            After this the five slots read 0 until a test sets a level with \ref
 *                      tst_control_adc_set. The converter and DMA channel are stopped first so
 *                      vcu_control_init starts them from rest as at boot; re-initialising a running
 *                      scan would let a conversion land before the software trigger restarts the
 *                      sequence, shifting every slot by one.
 */
void tst_control_adc_freeze(void) {
    ADC1->ctrl2_bit.adcen = 0u;
    DMA1_CHANNEL1->ctrl_bit.chen = 0u;
    vcu_control_init();
    prv_adc_buf = (volatile uint16_t*)(uintptr_t)DMA1_CHANNEL1->maddr;
    tst_control_adc_release();
}

/**
 * \brief           Apply one input level and wait until the scan has delivered it
 * \param[in]       slot: `VCU_ADC_SLOT_*`
 * \param[in]       raw: 12-bit conversion result
 */
void tst_control_adc_set(uint8_t slot, uint16_t raw) {
    prv_level(slot, raw);
    prv_settle(slot, raw);
}

/**
 * \brief           Set throttle and both brake levers as 8-bit codes (the `raw >> 4` value)
 * \param[in]       throttle: PA0 code
 * \param[in]       brake_full: PA4 code (full-weight lever)
 * \param[in]       brake_half: PA1 code (half-weight lever)
 */
void tst_control_adc_set_codes(uint8_t throttle, uint8_t brake_full, uint8_t brake_half) {
    uint16_t thr_raw = (uint16_t)((uint16_t)throttle << TST_ADC_CODE_SHIFT);
    uint16_t full_raw = (uint16_t)((uint16_t)brake_full << TST_ADC_CODE_SHIFT);
    uint16_t half_raw = (uint16_t)((uint16_t)brake_half << TST_ADC_CODE_SHIFT);

    prv_level(VCU_ADC_SLOT_THROTTLE, thr_raw);
    prv_level(VCU_ADC_SLOT_BRAKE_FULL, full_raw);
    prv_level(VCU_ADC_SLOT_BRAKE_HALF, half_raw);
    prv_settle(VCU_ADC_SLOT_THROTTLE, thr_raw);
    prv_settle(VCU_ADC_SLOT_BRAKE_FULL, full_raw);
    prv_settle(VCU_ADC_SLOT_BRAKE_HALF, half_raw);
}

/**
 * \brief           Put every scanned input back to 0 and wait until the buffer reads 0
 * \note            The emulated levels outlive tst_reset_product_state (they are the board's
 *                      inputs, not product RAM); a suite whose tests set levels calls this in
 *                      its set-up so no level leaks into the next test. Does nothing before
 *                      \ref tst_control_adc_freeze.
 */
void tst_control_adc_release(void) {
    uint8_t slot;

    for (slot = 0u; slot < VCU_ADC_SCAN_LEN; ++slot) {
        prv_level(slot, 0u);
    }
    for (slot = 0u; slot < VCU_ADC_SCAN_LEN; ++slot) {
        prv_settle(slot, 0u);
    }
}

/**
 * \brief           The product's scan buffer (read only: the DMA overwrites any write)
 * \return          its address (NULL before \ref tst_control_adc_freeze)
 */
volatile uint16_t* tst_control_adc_buf(void) { return prv_adc_buf; }
