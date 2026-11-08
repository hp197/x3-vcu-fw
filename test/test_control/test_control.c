/**
 * \file            test_control.c
 * \brief           Control loop signal path (control.c)
 * \note            Covers the analog front-end (ADC1 scan + DMA bring-up, the throttle / brake
 *                      lever conditioning), the drive-command producer with its current/voltage
 *                      derate, and the event / DTC latches. The inputs are written into the
 *                      product's ADC scan buffer (tst_control.h), the results read from the
 *                      blackboards. The drive-command producer selects and corrects the per-mode
 *                      ceilings from IN2+0x8E/+0x90 before applying the low-SoC ramp.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "at32f415.h"
#include "control.h"
#include "tst_bsp_fakes.h"
#include "tst_control.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/* 8-bit lever / throttle codes (the `raw >> 4` value control.c works on). */
#define CODE_REST       32u  /* in the fault window, below the 50 travel floor   */
#define CODE_FLOOR      50u  /* first code with travel: command 0, active          */
#define CODE_MID        100u /* (100 - 50) * 200 / 105 = 95                         */
#define CMD_MID         95u
#define CODE_TOP        154u /* last scaled code: 104 * 200 / 105 = 198             */
#define CMD_TOP         198u
#define CODE_FULL       155u /* saturates at 200                                    */
#define CMD_FULL        200u
#define CODE_FAULT_LOW  24u  /* below the 25 fault window                           */
#define CODE_WIN_LOW    25u  /* first code inside the fault window                  */
#define CODE_WIN_HIGH   220u /* last code inside the 25 + 195 window                */
#define CODE_FAULT_HIGH 221u
#define CODE_CAL1_HIGH  185u /* last code of the throttle window with reg 0x06 == 1 */
#define CODE_CAL1_FAULT 186u

/* Drive-command inputs that keep every derate at unity. */
#define SOC_FULL        100
#define CURR_LIMIT_HIGH 10000 /* ceiling 100 after the /100            */
#define VBUS_CODE_LOW   0u    /* voltage code (A3C vbus_code >> 8) < 116 */
#define ACC_BASE        40u   /* table column +0x10 in every row         */

/**
 * \brief           Run the conditioning once with the given codes
 * \param[in]       fsm_state: vehicle FSM state argument
 * \param[in]       thr: throttle code
 * \param[in]       full: full-weight lever code
 * \param[in]       half: half-weight lever code
 */
static void prv_scan(uint8_t fsm_state, uint8_t thr, uint8_t full, uint8_t half) {
    tst_control_adc_set_codes(thr, full, half);
    control_throttle_brake_scan(fsm_state);
}

/**
 * \brief           Unity-gain drive inputs: full SoC, high current ceiling, low voltage code,
 *                      session active, stored speed-limit registers invalid (< 5)
 */
static void prv_drive_neutral(void) {
    g_stat.soc = SOC_FULL;
    g_stat.curr_limit = CURR_LIMIT_HIGH;
    g_fault_codes.vbus_code = (uint16_t)(VBUS_CODE_LOW << 8);
    g_in2.f.status_flags = SF_SESSION;
    g_in2.f.gear_a_ceiling = 0u;
    g_in2.f.speed_limit_2 = 0u;
}

/**
 * \brief           Fresh boot state, fakes, USART2 (reports) and the ADC scan with every input at 0
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
    tst_control_adc_freeze();
    g_in2.f.cfg_flags2 = CFG2_THROTTLE_PERMIT; /* keep the brake byte (IN2+0x3C bit11) */
}

/** \brief Nothing to undo */
void tearDown(void) {}

/* -- ADC front-end bring-up ---------------------------------------------------------------- */

/**
 * \brief           ADC1 scans channels 0,1,4,5,7 in that order, repeating, into DMA1 channel 1
 * \note            Base config: scan, repeat, length 5; ordinary channels 0,1,4,5,7 in
 *                      sequence slots 1..5 at 239.5 cycles; DMA mode + ADC enable. DMA1 channel 1:
 *                      peripheral 0x4001244C (ADC1 odt), 5 halfwords, memory increment, circular.
 *                      PA0/1/4/5/7 analog.
 */
static void test_adc_init_scan_geometry(void) {
    uint32_t osq3 = ADC1->osq3;

    TEST_ASSERT_EQUAL_UINT32(VCU_ADC_SCAN_LEN - 1u, ADC1->osq1_bit.oclen);
    TEST_ASSERT_EQUAL_UINT32(0u, osq3 & 0x1Fu);
    TEST_ASSERT_EQUAL_UINT32(1u, (osq3 >> 5) & 0x1Fu);
    TEST_ASSERT_EQUAL_UINT32(4u, (osq3 >> 10) & 0x1Fu);
    TEST_ASSERT_EQUAL_UINT32(5u, (osq3 >> 15) & 0x1Fu);
    TEST_ASSERT_EQUAL_UINT32(7u, (osq3 >> 20) & 0x1Fu);
    TEST_ASSERT_EQUAL_UINT32(1u, ADC1->ctrl1_bit.sqen);
    TEST_ASSERT_EQUAL_UINT32(1u, ADC1->ctrl2_bit.rpen);
    TEST_ASSERT_EQUAL_UINT32(1u, ADC1->ctrl2_bit.ocdmaen);
    TEST_ASSERT_EQUAL_HEX32((uint32_t)&ADC1->odt, DMA1_CHANNEL1->paddr);
    TEST_ASSERT_EQUAL_UINT32(1u, DMA1_CHANNEL1->ctrl_bit.lm);
    TEST_ASSERT_EQUAL_UINT32(1u, DMA1_CHANNEL1->ctrl_bit.mincm);
    TEST_ASSERT_EQUAL_UINT32(0u, DMA1_CHANNEL1->ctrl_bit.pincm);
    TEST_ASSERT_EQUAL_UINT32(0u, DMA1_CHANNEL1->ctrl_bit.dtd); /* peripheral -> memory */
    /* PA0, PA1, PA4, PA5, PA7: cfglr nibble 0 = analog input. */
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOA->cfglr & 0xF0FF00FFu);
}

/**
 * \brief           The leaf readers index the DMA buffer; an out-of-range slot reads 0
 * \note            Slots 0..4 read the DMA buffer cell at byte offset 2*slot; a slot past 4
 *                      reads 0.
 */
static void test_adc_raw_reads_scan_slots(void) {
    uint8_t slot;

    for (slot = 0u; slot < VCU_ADC_SCAN_LEN; ++slot) {
        tst_control_adc_set(slot, (uint16_t)(0x100u + slot));
    }
    for (slot = 0u; slot < VCU_ADC_SCAN_LEN; ++slot) {
        TEST_ASSERT_EQUAL_HEX16(0x100u + slot, control_adc_raw(slot));
    }
    TEST_ASSERT_EQUAL_HEX16(0u, control_adc_raw(VCU_ADC_SCAN_LEN));
}

/* -- throttle / brake conditioning (control_throttle_brake_scan) ------------------------ */

/**
 * \brief           Throttle code -> MODE+8 low byte: 0 below 0x32, (code-0x32)*200/105, 200 from 0x9B
 * \note            `if (thr < 0x9b) { if (thr - 0x32 < 0x69) { tmr+3 = 1;
 *                      lo = (thr-0x32)*200/0x69 } else { tmr+3 = 0; lo = 0 } } else { tmr+3 = 1;
 *                      lo = 200 }`, thr = slot-0 raw >> 4.
 */
static void test_scan_throttle_scaling(void) {
    prv_scan(VCU_FSM_RIDE, CODE_FLOOR - 1u, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.drive_output_word & 0xFFu);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.throttle_active);

    prv_scan(VCU_FSM_RIDE, CODE_FLOOR, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.drive_output_word & 0xFFu);
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.throttle_active);

    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(CMD_MID, g_mode.drive_output_word & 0xFFu);

    prv_scan(VCU_FSM_RIDE, CODE_TOP, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(CMD_TOP, g_mode.drive_output_word & 0xFFu);

    prv_scan(VCU_FSM_RIDE, CODE_FULL, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(CMD_FULL, g_mode.drive_output_word & 0xFFu);
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.throttle_active);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.drive_output_word & 0xFF00u);
}

/**
 * \brief           Brake byte (MODE+8 high) = max(PA4 command, PA1 command / 2)
 * \note            Both levers scaled like the throttle, then
 *                      `if (full <= half >> 1) full = half >> 1; hi = full & 0xff`; activity
 *                      flags at g_tmr +1 (PA4) / +2 (PA1).
 */
static void test_scan_brake_is_max_full_half_over_two(void) {
    prv_scan(VCU_FSM_RIDE, CODE_REST, CODE_MID, CODE_FULL);
    TEST_ASSERT_EQUAL_UINT8(CMD_FULL / 2u, g_mode.drive_output_word >> 8);

    prv_scan(VCU_FSM_RIDE, CODE_REST, CODE_FULL, CODE_FULL);
    TEST_ASSERT_EQUAL_UINT8(CMD_FULL, g_mode.drive_output_word >> 8);

    prv_scan(VCU_FSM_RIDE, CODE_REST, CODE_REST, CODE_MID);
    TEST_ASSERT_EQUAL_UINT8(CMD_MID / 2u, g_mode.drive_output_word >> 8);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.brake_full_active);
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.brake_half_active);

    prv_scan(VCU_FSM_RIDE, CODE_REST, CODE_MID, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(CMD_MID, g_mode.drive_output_word >> 8);
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.brake_full_active);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.brake_half_active);
}

/**
 * \brief           Out-of-window codes latch the channel fault; any fault zeroes the whole word
 * \note            Normal branch: latch = (code - 0x19 < 0xC4) ? 0 : 1 for
 *                      IN1+0x1E (PA0), +0x1F (PA4), +0x2C (PA1); the tail clears both bytes
 *                      when any of the three latches is set.
 */
static void test_scan_fault_window_zeroes_word(void) {
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_WIN_LOW, CODE_WIN_HIGH);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_full_range_fault);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_half_range_fault);
    TEST_ASSERT_EQUAL_UINT8(CMD_MID, g_mode.drive_output_word & 0xFFu);

    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_FAULT_LOW, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_full_range_fault);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.drive_output_word);

    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_FAULT_HIGH);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_full_range_fault);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_half_range_fault);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.drive_output_word);

    prv_scan(VCU_FSM_RIDE, CODE_WIN_HIGH, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_fault_latch);
    prv_scan(VCU_FSM_RIDE, CODE_FAULT_HIGH, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.throttle_fault_latch);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.drive_output_word);
}

/**
 * \brief           With reg 0x06 (IN2+0x0C) == 1 the throttle window narrows to 0x19..0xB9
 * \note            `if (IN2+0xC == 1) latch = thr - 0x19 < 0xa1 ? 0 : 1`.
 */
static void test_scan_throttle_window_cal_select(void) {
    g_in2.f.throttle_cal_sel = 1;
    prv_scan(VCU_FSM_RIDE, CODE_CAL1_HIGH, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_fault_latch);
    prv_scan(VCU_FSM_RIDE, CODE_CAL1_FAULT, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.throttle_fault_latch);
}

/**
 * \brief           A set stuck guard holds the fault until the code returns to 0x1A..0x31
 * \note            `if (IN1+0x29 != 0) { IN1+0x1E = 1; if (thr - 0x1a < 0x18)
 *                      IN1+0x29 = 0; }` (same for PA4 +0x2A/+0x1F and PA1 +0x2B/+0x2C).
 */
static void test_scan_stuck_guard_releases_at_rest(void) {
    g_in1.throttle_stuck_guard = 1u;
    g_in1.brake_full_cal_guard = 1u;
    g_in1.brake_half_cal_guard = 1u;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_MID, CODE_MID);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.throttle_fault_latch);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_full_range_fault);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_half_range_fault);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.throttle_stuck_guard);

    prv_scan(VCU_FSM_RIDE, 0x1Au, 0x31u, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.throttle_fault_latch); /* still latched this pass */
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_stuck_guard);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_full_cal_guard);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_half_cal_guard);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.drive_output_word);

    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_fault_latch);
    TEST_ASSERT_EQUAL_UINT8(CMD_MID, g_mode.drive_output_word & 0xFFu);
}

/**
 * \brief           Power-on with a session: a channel at rest clears its latch, else arms its guard
 * \note            `if (fsm_state == 1 && IN2+0x38 bit11) { if (thr - 0x19 < 0x1a)
 *                      IN1+0x1E = 0; else IN1+0x29 = 1; ... }` (dead band 0x19..0x32).
 */
static void test_scan_power_on_calibration_branch(void) {
    g_in2.f.status_flags = SF_SESSION;
    g_in1.throttle_fault_latch = 1u;
    g_in1.brake_full_range_fault = 1u;
    prv_scan(VCU_FSM_POWER_ON, CODE_WIN_LOW, CODE_FLOOR, 0x33u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_fault_latch);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_full_range_fault);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_stuck_guard);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_full_cal_guard);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_half_cal_guard);

    /* Without the session the same state takes the normal window. */
    g_in2.f.status_flags = 0u;
    prv_scan(VCU_FSM_POWER_ON, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_stuck_guard);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_half_range_fault); /* guard from the pass above */
}

/**
 * \brief           Output gates: charging drops the throttle, IN2+0x3C bit11 clear drops the brake
 * \note            Tail: `if (!(IN2+0x3C bit11)) w &= 0xff;` ...
 *                      `if (fsm_state == 7) w &= 0xff00;`
 */
static void test_scan_output_gates(void) {
    prv_scan(VCU_FSM_CHARGING, CODE_MID, CODE_MID, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16((uint16_t)(CMD_MID << 8), g_mode.drive_output_word);

    g_mode.output_cfg = 0u;
    g_in2.f.cfg_flags2 = 0u;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_MID, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(CMD_MID, g_mode.drive_output_word);
}

/**
 * \brief           Brake cap (MODE+0x17 bit4): throttle off, and brake 15 above 15.0 km/h when the
 *                      lever asks for less
 * \note            `if (MODE+0x17 bit4) { ride_assist_reset_all(); w &= 0xff00;
 *                      if ((short)IN2+0xAE > 150 && (w >> 8) < 15) w = (w & 0xff) + 0xf00; }`.
 */
static void test_scan_brake_cap(void) {
    g_mode.output_cfg = OCFG_BRAKE_CAP;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(0x0000u, g_mode.drive_output_word);

    g_in2.f.speed_src = 150u;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(0x0000u, g_mode.drive_output_word);

    g_in2.f.speed_src = 151u;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(0x0F00u, g_mode.drive_output_word);

    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_MID, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16((uint16_t)(CMD_MID << 8), g_mode.drive_output_word);
}

/**
 * \brief           Wobble alert (MODE+0x17 bit3): the throttle runs through a 16-sample average
 * \note            Every scan pushes the throttle byte into the 16-byte ring
 *                      (ring[idx++] = b, idx wraps at 16, returns (sum >> 4) & 0xff); with bit3 it
 *                      pushes again and takes the result. First scan with throttle 95: two pushes
 *                      of 95 -> 190 >> 4 = 11.
 */
static void test_scan_wobble_average(void) {
    g_mode.output_cfg = OCFG_WOBBLE_ALERT;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(11u, g_mode.drive_output_word);
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(23u, g_mode.drive_output_word); /* 4 x 95 >> 4 */

    g_mode.output_cfg = 0u; /* one push only, the command passes through */
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(CMD_MID, g_mode.drive_output_word);
    g_mode.output_cfg = OCFG_WOBBLE_ALERT;
    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_HEX16(41u, g_mode.drive_output_word); /* 7 x 95 >> 4 */
}

/**
 * \brief           A change of any activity flag refreshes IN1 brake_any / throttle_flag and
 *                      raises VS+0x1D; no change leaves them alone
 * \note            `if (tmr+1 != tmr+4 || tmr+2 != tmr+5 || IN1+0xC != tmr+3)
 *                      { IN1+0xC = tmr+3; tmr+4 = tmr+1; tmr+5 = tmr+2; VS+0x1D = 1;
 *                      IN1+0xB = (tmr+1 || tmr+2); }`.
 */
static void test_scan_activity_edge_detect(void) {
    prv_scan(VCU_FSM_RIDE, CODE_REST, CODE_REST, CODE_MID);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.brake_any);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.throttle_flag);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.can_drive_active);

    g_vs.can_drive_active = 0u;
    g_in1.brake_any = 0x55u;
    prv_scan(VCU_FSM_RIDE, CODE_REST, CODE_REST, CODE_MID);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.can_drive_active);
    TEST_ASSERT_EQUAL_UINT8(0x55u, g_in1.brake_any);

    prv_scan(VCU_FSM_RIDE, CODE_MID, CODE_REST, CODE_REST);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.brake_any);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.throttle_flag);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.can_drive_active);
}

/* -- drive command (control_build_drive_cmd, control_scale_current) ---------------------- */

/**
 * \brief           Accel limits are the table column +0x10 times 1.3 and 0.8, truncated
 * \note            VS+9 = (u8)(col10 * 1.3), VS+10 = (u8)(col10 * 0.8) (doubles
 *                      0x3FF4CCCC_CCCCCCCD / 0x3FE99999_9999999A); table col +0x10 = 0x28 in
 *                      every row.
 */
static void test_drive_accel_limits(void) {
    prv_drive_neutral();
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(52u, g_vs.accel_limit_fwd);
    TEST_ASSERT_EQUAL_UINT8(32u, g_vs.accel_limit_rev);
}

/**
 * \brief           Gear 1 (idle): current 10, word 10 (25 with throttle), lead 5 << 1, bit13 set, regen flag
 * \note            `IN2+0xB4 == 1`: +0x10 lo = (MODE+8 lo == 0) ? 10 : 0x19; +0xC lo =
 *                      control_scale_current(10), +0 = 5, +0x10 |= 0x2000, IN1+0x26 = 1; tail:
 *                      +2 = 0xC8, +0 <<= 1, +0xE hi = 0x5A, +0xC = (+0xC & 0xC0FF) + 0x1400 | dir << 14,
 *                      +6 = 0x0E.
 */
static void test_drive_gear_idle(void) {
    prv_drive_neutral();
    g_in2.f.gear = 1;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x140Au, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A0Au, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX8(0x0Au, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX8(0xC8u, g_mode.cmd_param);
    TEST_ASSERT_EQUAL_HEX16(0x000Eu, g_mode.word06);
    TEST_ASSERT_EQUAL_HEX16(0x2000u, g_mode.enable_flags & 0x2000u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.regen_active);

    g_mode.drive_output_word = 0x0001u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x5A19u, g_mode.we_word);
}

/**
 * \brief           Gear 2, row 0: table current 0x14, word 0x32, ceiling 0x19 -> lead 0x32, word04 0x62
 * \note            `IN2+0xB4 == 2`: +0xC lo = control_scale_current(col4), +0xE lo = col2,
 *                      ceiling = col0, >= 0xB -> control_map(soc,0,0xf,10,ceiling), +4 = 0x0062,
 *                      +0x10 &= 0xDFFF.
 */
static void test_drive_gear_a(void) {
    prv_drive_neutral();
    g_mode.enable_flags = 0x2000u;
    g_in1.regen_active = 1u;
    g_in2.f.gear = 2;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x1414u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A32u, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x0062u, g_mode.word04);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.enable_flags & 0x2000u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.regen_active);
}

/**
 * \brief           Gear 3 per row: word04 0x147 in every row; row-0 low-SoC ramp
 * \note            `IN2+0xB4 == 3`: current col 0xA, word col 8, ceiling col 6;
 *                      row 0 and ceiling > 0x1D: soc < 10 -> map(soc,0,10,10,0x1E), else
 *                      map(soc,10,0x14,0x1E,ceiling); other rows map(soc,0,0xF,10,ceiling).
 */
static void test_drive_gear_b_rows(void) {
    prv_drive_neutral();
    g_in2.f.gear = 3;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x141Eu, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A5Au, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX8(0x64u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x0147u, g_mode.word04);

    g_vs.region = VCU_REGION_EUROPE;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x1419u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x0147u, g_mode.word04);

    g_vs.region = VCU_REGION_GLOBAL;
    g_in2.f.speed_limit_2 = 0x3200u;
    g_stat.soc = 5; /* map(5,0,10,10,30) = 20 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(40u, g_mode.cmd_lead);
    g_stat.soc = 15; /* map(15,10,20,30,50) = 40 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(80u, g_mode.cmd_lead);
}

/**
 * \brief           Gear 4: selected ceiling, current col 0x10, word col 0xE
 * \note            `IN2+0xB4 == 4`: the IN2+0x90 low byte is selected with the IN2+0x8C low byte
 *                      as fallback; current = control_scale_current(col 0x10), word col 0xE (0x8C),
 *                      +4 = 0x01EB in every row. A ceiling < 0xB passes through.
 */
static void test_drive_gear_c(void) {
    prv_drive_neutral();
    g_in2.f.gear = 4;
    g_in2.f.speed_limit = 0x19u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x1428u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A8Cu, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x01EBu, g_mode.word04);

    g_vs.region = VCU_REGION_EUROPE;
    g_in2.f.speed_limit = 0x0Au;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x14u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x01EBu, g_mode.word04);
}

/**
 * \brief           Gear 2 selects the IN2+0x8E low byte, with table column 0 as fallback
 * \note            Values 5..25 are retained; 0..4 and values above 25 select 25 and are written
 *                      back without changing the high byte. At full SoC cmd_lead is twice the ceiling.
 */
static void test_drive_gear_a_ceiling_select(void) {
    prv_drive_neutral();
    g_in2.f.gear = 2;

    g_in2.f.gear_a_ceiling = 0xA510u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x20u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0xA510u, g_in2.f.gear_a_ceiling);

    g_in2.f.gear_a_ceiling = 0xA504u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0xA519u, g_in2.f.gear_a_ceiling);

    g_in2.f.gear_a_ceiling = 0xA51Au;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0xA519u, g_in2.f.gear_a_ceiling);
}

/**
 * \brief           Gear 3 selects the IN2+0x90 high byte, with table column 6 as fallback
 * \note            Values 5..50 are retained; 0..4 and values above 50 select 50 and are written
 *                      back without changing the low byte. At full SoC cmd_lead is twice the ceiling.
 */
static void test_drive_gear_b_ceiling_select(void) {
    prv_drive_neutral();
    g_in2.f.gear = 3;

    g_in2.f.speed_limit_2 = 0x28A5u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x50u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x28A5u, g_in2.f.speed_limit_2);

    g_in2.f.speed_limit_2 = 0x04A5u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x64u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x32A5u, g_in2.f.speed_limit_2);

    g_in2.f.speed_limit_2 = 0x33A5u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x64u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x32A5u, g_in2.f.speed_limit_2);
}

/**
 * \brief           Gear 4 selects the IN2+0x90 low byte, with the IN2+0x8C low byte as fallback
 * \note            Values 5..50 are retained; 0..4 and values above 50 select 50 and are written
 *                      back without changing the high byte. At full SoC cmd_lead is twice the ceiling.
 */
static void test_drive_gear_c_ceiling_select(void) {
    prv_drive_neutral();
    g_in2.f.gear = 4;
    g_in2.f.speed_limit = 0x32u;

    g_in2.f.speed_limit_2 = 0xA520u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x40u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0xA520u, g_in2.f.speed_limit_2);

    g_in2.f.speed_limit_2 = 0xA504u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x64u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0xA532u, g_in2.f.speed_limit_2);

    g_in2.f.speed_limit_2 = 0xA533u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x64u, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0xA532u, g_in2.f.speed_limit_2);
}

/**
 * \brief           Row 2 turns a doubled lead of 0x2C into 0x2B and sends word06 0x0A (0x0E otherwise)
 * \note            The lead is doubled; in row 2 word06 = 0x0A and a doubled lead of 0x2C
 *                      becomes 0x2B; in the other rows word06 = 0x0E.
 */
static void test_drive_eco_lead_quirk(void) {
    prv_drive_neutral();
    g_vs.region = VCU_REGION_GERMANY;
    g_in2.f.gear = 4;
    g_in2.f.speed_limit = 0x16u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x2Bu, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x000Au, g_mode.word06);

    g_vs.region = VCU_REGION_EUROPE;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x2Cu, g_mode.cmd_lead);
    TEST_ASSERT_EQUAL_HEX16(0x000Eu, g_mode.word06);
}

/**
 * \brief           Event 9 is raised at speed >= reg 0x46 low byte + 3 and cleared below
 * \note            `if (VS+2 < (u8)IN2+0x8C + 3) control_event_clear(9); else
 *                      control_event_set(9);` event 9 = SCR+0x2A bit 8.
 */
static void test_drive_overspeed_event9(void) {
    prv_drive_neutral();
    g_in2.f.speed_limit = 0x19u;
    g_vs.speed_kmh = 0x1Bu;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo);

    g_vs.speed_kmh = 0x1Cu;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x0100u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_UINT8(9u, g_lowglob.active_event_id);

    g_vs.speed_kmh = 0u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.active_event_id);
}

/**
 * \brief           The +0xE word low byte is voltage-derated: unity to code 0xA5, ramp to 0xBE, 0 above
 * \note            code = A3C+2 >> 8; `< 0xa6 -> 0x80; < 0xbf ->
 *                      control_map_dec(code,0xa5,0xaa,0xc,0x80); else 0`; lo = scale * lo >> 7.
 *                      Code 0xA8: 0x80 - 0x74*3/5 = 59 -> 0x32 * 59 >> 7 = 23.
 */
static void test_drive_we_voltage_derate(void) {
    prv_drive_neutral();
    g_in2.f.gear = 2;
    g_fault_codes.vbus_code = 0xA500u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.we_word & 0xFFu);
    g_fault_codes.vbus_code = 0xA800u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(23u, g_mode.we_word & 0xFFu);
    g_fault_codes.vbus_code = 0xBF00u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.we_word & 0xFFu);
}

/**
 * \brief           Current command = min(voltage-scaled, ceiling/100, SoC-scaled) of the base
 * \note            Voltage scale 0x80 (< 0x74), 0x74 (< 0x92),
 *                      control_map_dec(v,0x91,0x9b,0x40,0x74) (< 0x9C),
 *                      control_map_dec(v,0x9b,0xaa,0xc,0x40) (< 0xBF), else 0; codes 0xFD/0xFE
 *                      read as 0x9B; ceiling STAT+0x122 / 100; SoC scale
 *                      control_map(soc,0,0x1e,0x1e,0x80); each
 *                      scale is (base * s & 0x7FFFFF) >> 7. Base 0x14 (gear 2, row 0).
 */
static void test_drive_current_derate(void) {
    prv_drive_neutral();
    g_in2.f.gear = 2;
    g_fault_codes.vbus_code = 0x8000u; /* 0x74 -> 20 * 116 >> 7 = 18 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(18u, g_mode.curr_word & 0xFFu);
    g_fault_codes.vbus_code = 0x9600u; /* 0x74 - 0x34 * 5 / 10 = 90 -> 14 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(14u, g_mode.curr_word & 0xFFu);
    g_fault_codes.vbus_code = 0xFD00u; /* sentinel -> 0x9B -> 0x40 -> 10 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(10u, g_mode.curr_word & 0xFFu);
    g_fault_codes.vbus_code = 0xBF00u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.curr_word & 0xFFu);

    g_fault_codes.vbus_code = 0u;
    g_stat.curr_limit = 1500; /* ceiling 15 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(15u, g_mode.curr_word & 0xFFu);
    g_stat.curr_limit = CURR_LIMIT_HIGH;
    g_stat.soc = 15; /* (98 * 15) / 30 + 30 = 79 -> 20 * 79 >> 7 = 12 */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(12u, g_mode.curr_word & 0xFFu);
}

/**
 * \brief           Without a session the command falls back to the row's raw gear-2 values
 * \note            End of the producer: `if (!(IN2+0x38 bit11)) { +0xC lo = col4; +0xE lo = col2;
 *                      +0 = (short)col0 << 1; }`, after the +0xE high byte is set to 0x5A.
 */
static void test_drive_no_session_failsafe(void) {
    prv_drive_neutral();
    g_in2.f.status_flags = 0u;
    g_in2.f.gear = 4;
    g_in2.f.speed_limit = 0x19u;
    g_fault_codes.vbus_code = 0x8000u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x1414u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A32u, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
}

/**
 * \brief           Tail flags: mode bit7 -> word04 0x3FFF, cfg bit1 with throttle 0 -> the coast
 *                      energy preset (0: no writer), dir bits 15:14, gear 0 leaves the gear fields
 * \note            `if ((MODE+8 & 0xff) == 0 && MODE+0x17 bit1) +0x10 lo = coast_energy;` before the
 *                      voltage derate; `if (MODE+0xA bit7) +4 = 0x3FFF; +0xD = (+0xD & 0x3f) |
 *                      IN2+0xE0 << 6`; no case for IN2+0xB4 outside 1..4. `coast_energy` has no writer in
 *                      this firmware, so it reads 0.
 */
static void test_drive_tail_flags(void) {
    prv_drive_neutral();
    g_in2.f.gear = 2;
    g_mode.mode_flags = 0x0080u;
    g_mode.output_cfg = 0x02u;
    g_in2.f.dir_bits = 2u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x3FFFu, g_mode.word04);
    TEST_ASSERT_EQUAL_HEX16(0x5A00u, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX16(0x9414u, g_mode.curr_word);
    g_mode.drive_output_word = 0x0001u; /* throttle on: the table word stays */
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x5A32u, g_mode.we_word);
    g_mode.drive_output_word = 0u;

    g_in2.f.gear = 0;
    g_mode.mode_flags = 0u;
    g_mode.output_cfg = 0u;
    g_in2.f.dir_bits = 0u;
    g_mode.curr_word = 0x0077u;
    g_mode.word04 = 0x1234u;
    g_mode.cmd_lead = 0x11u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x1477u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_mode.word04);
    TEST_ASSERT_EQUAL_HEX8(0x22u, g_mode.cmd_lead);
}

/**
 * \brief           Regen byte (staging +0x0E): reg 0x70 (IN2+0xE0) 0 / 1 / 2 -> 0 / 22 / 45, other
 *                      values leave it; 3 for the over-speed regen; max with its level
 * \note            `if (IN2+0xE0 == 0) { +0xE = 0; if (MODE+0x17 bit2 &&
 *                      IN2+0x3C bit4) +0xE = 3; } else if (== 1) +0xE = 0x16; else if (== 2) +0xE = 0x2D;`
 *                      Then `if (IN2+0x3C bit4) +0xE = max(overspeed_regen, +0xE)` (`overspeed_regen` is 0 at boot).
 */
static void test_drive_regen_level(void) {
    prv_drive_neutral();
    g_in2.f.gear = 2;
    g_in2.f.dir_bits = 1u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(22u, g_mode.regen_level);
    g_in2.f.dir_bits = 2u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(45u, g_mode.regen_level);
    g_in2.f.dir_bits = 3u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(45u, g_mode.regen_level);
    g_in2.f.dir_bits = 0u;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.regen_level);

    g_mode.output_cfg = OCFG_OVERSPEED_REGEN;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(0u, g_mode.regen_level); /* bit2 alone is not enough */
    g_in2.f.cfg_flags2 = CFG2_OVERSPEED_REGEN;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_UINT8(3u, g_mode.regen_level);
}

/**
 * \brief           Low-SoC derate flag (+0x17 bits 0..1): below 20 % in row 0, below 15 % otherwise
 * \note            `if (VS+0 == 0 && soc < 0x14) +0x17 |= 3; else if (soc < 0xf)
 *                      +0x17 |= 3; else +0x17 &= 0xfc;`
 */
static void test_drive_low_soc_flag(void) {
    prv_drive_neutral();
    g_stat.soc = 19;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x03u, g_mode.cfg_17 & 0x03u);
    g_stat.soc = 20;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0u, g_mode.cfg_17 & 0x03u);
    g_vs.region = VCU_REGION_EUROPE;
    g_stat.soc = 19;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0u, g_mode.cfg_17 & 0x03u);
    g_stat.soc = 14;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX8(0x03u, g_mode.cfg_17 & 0x03u);
}

/**
 * \brief           Live mode 3 selects table row 0 (the row index wraps)
 * \note            control.c indexes the 3-row table with VS[0] % 3. Gear 2 then gives row 0's
 *                      values (current 0x14, word 0x32, ceiling 0x19 -> lead 0x32), not row 1's
 *                      (ceiling 0x0F).
 */
static void test_drive_mode3_wraps_to_row0(void) {
    prv_drive_neutral();
    g_vs.region = 3u;
    g_in2.f.gear = 2;
    control_build_drive_cmd();
    TEST_ASSERT_EQUAL_HEX16(0x1414u, g_mode.curr_word);
    TEST_ASSERT_EQUAL_HEX16(0x5A32u, g_mode.we_word);
    TEST_ASSERT_EQUAL_HEX8(0x32u, g_mode.cmd_lead);
}

/* -- event / DTC latches ------------------------------------------------------------------ */

/**
 * \brief           Events 1..16 / 17..32 set mask bits; the active id keeps the lowest one
 * \note            control_event_set stores min(active, ev) (ev if none), then ev 1..16 ->
 *                      SCR+0x2A bit ev-1, 17..32 -> SCR+0x2C bit (ev-1)&0xF, 0 and > 0x20 set no
 *                      bit. control_event_clear is the inverse (it drops the active id, and zeroes
 *                      IN2+0xB2, only when it equals ev).
 */
static void test_event_set_clear(void) {
    control_event_set(9u);
    TEST_ASSERT_EQUAL_UINT8(9u, g_lowglob.active_event_id);
    control_event_set(3u);
    control_event_set(20u);
    control_event_set(0x21u);
    TEST_ASSERT_EQUAL_UINT8(3u, g_lowglob.active_event_id);
    TEST_ASSERT_EQUAL_HEX16(0x0104u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_HEX16(0x0008u, g_scr.event_mask_hi);

    g_in2.f.event_ack_reset = 0x5555u;
    control_event_clear(9u);
    TEST_ASSERT_EQUAL_UINT8(3u, g_lowglob.active_event_id);
    TEST_ASSERT_EQUAL_HEX16(0x5555u, g_in2.f.event_ack_reset);
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_scr.event_mask_lo);
    control_event_clear(3u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.active_event_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.event_ack_reset);
    control_event_clear(20u);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi);
}

/**
 * \brief           Event 0 overwrites the active id with 0 (the latch keeps the smaller value)
 * \note            The latch keeps the smaller of the active id and ev, so a non-zero
 *                      active id >= 0 is replaced by 0; no mask bit for 0.
 */
static void test_event_set_zero_clears_active(void) {
    control_event_set(5u);
    control_event_set(0u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.active_event_id);
    TEST_ASSERT_EQUAL_HEX16(0x0010u, g_scr.event_mask_lo);
}

/**
 * \brief           DTC codes 10..25 / 26..41 / 42..57 map to SCR+0x24 / +0x26 / +0x28
 * \note            control_dtc_set latches min(active, code); code-10 < 16 -> +0x24 bit
 *                      code-10, else bit (code+6)&0xF in +0x26 (< 32) or +0x28 (< 48); codes
 *                      outside 10..57 set no bit. control_dtc_clear clears (it drops the active
 *                      code only on a match).
 */
static void test_dtc_set_clear(void) {
    control_dtc_set(10u);
    control_dtc_set(25u);
    control_dtc_set(26u);
    control_dtc_set(41u);
    control_dtc_set(42u);
    control_dtc_set(57u);
    control_dtc_set(58u);
    control_dtc_set(9u);
    TEST_ASSERT_EQUAL_HEX16(0x8001u, g_scr.dtc_mask0);
    TEST_ASSERT_EQUAL_HEX16(0x8001u, g_scr.dtc_mask1);
    TEST_ASSERT_EQUAL_HEX16(0x8001u, g_scr.dtc_mask2);
    TEST_ASSERT_EQUAL_UINT8(9u, g_lowglob.active_error_code);

    control_dtc_clear(25u);
    control_dtc_clear(26u);
    control_dtc_clear(57u);
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_scr.dtc_mask0);
    TEST_ASSERT_EQUAL_HEX16(0x8000u, g_scr.dtc_mask1);
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_scr.dtc_mask2);
    TEST_ASSERT_EQUAL_UINT8(9u, g_lowglob.active_error_code);
    control_dtc_clear(9u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.active_error_code);
}

/**
 * \brief           The mask resets clear their own block only
 * \note            control_event_mask_reset: active event id = 0, IN2+0xB2 = 0,
 *                      SCR+0x2A/+0x2C = 0. control_dtc_mask_reset: active error code = 0,
 *                      IN2+0xB0 = 0, SCR+0x24/+0x26/+0x28 = 0.
 */
static void test_event_dtc_mask_reset(void) {
    control_event_set(2u);
    control_dtc_set(12u);
    g_in2.f.event_ack_reset = 1u;
    g_in2.f.lock_state = 1u;

    control_event_mask_reset();
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.active_event_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.event_ack_reset);
    TEST_ASSERT_EQUAL_UINT8(12u, g_lowglob.active_error_code);
    TEST_ASSERT_EQUAL_HEX16(0x0004u, g_scr.dtc_mask0);

    control_dtc_mask_reset();
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.active_error_code);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.dtc_mask0);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.lock_state);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_adc_init_scan_geometry);
    RUN_TEST(test_adc_raw_reads_scan_slots);
    RUN_TEST(test_scan_throttle_scaling);
    RUN_TEST(test_scan_brake_is_max_full_half_over_two);
    RUN_TEST(test_scan_fault_window_zeroes_word);
    RUN_TEST(test_scan_throttle_window_cal_select);
    RUN_TEST(test_scan_stuck_guard_releases_at_rest);
    RUN_TEST(test_scan_power_on_calibration_branch);
    RUN_TEST(test_scan_output_gates);
    RUN_TEST(test_scan_brake_cap);
    RUN_TEST(test_scan_wobble_average);
    RUN_TEST(test_scan_activity_edge_detect);
    RUN_TEST(test_drive_accel_limits);
    RUN_TEST(test_drive_gear_idle);
    RUN_TEST(test_drive_gear_a);
    RUN_TEST(test_drive_gear_b_rows);
    RUN_TEST(test_drive_gear_c);
    RUN_TEST(test_drive_gear_a_ceiling_select);
    RUN_TEST(test_drive_gear_b_ceiling_select);
    RUN_TEST(test_drive_gear_c_ceiling_select);
    RUN_TEST(test_drive_eco_lead_quirk);
    RUN_TEST(test_drive_overspeed_event9);
    RUN_TEST(test_drive_we_voltage_derate);
    RUN_TEST(test_drive_current_derate);
    RUN_TEST(test_drive_no_session_failsafe);
    RUN_TEST(test_drive_tail_flags);
    RUN_TEST(test_drive_regen_level);
    RUN_TEST(test_drive_low_soc_flag);
    RUN_TEST(test_drive_mode3_wraps_to_row0);
    RUN_TEST(test_event_set_clear);
    RUN_TEST(test_event_set_zero_clears_active);
    RUN_TEST(test_dtc_set_clear);
    RUN_TEST(test_event_dtc_mask_reset);
    tst_exit(UNITY_END());
    return 0;
}
