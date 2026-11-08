/**
 * \file            test_passcode.c
 * \brief           Passcode set / key-sequence unlock / numeric unlock (passcode.c)
 * \note            Covers passcode_set_confirm_tick (set/confirm), passcode_key_unlock_tick (key
 *                      unlock), passcode_digit_unlock_tick (numeric unlock) and the entry helpers
 *                      passcode_digit_count / passcode_pack_keys / passcode_display_word /
 *                      passcode_bcd_to_decimal / passcode_entry_reset.
 *
 *                      Handlebar keys are pressed the way the hardware does it: the ticks run
 *                      the real faultmon monitors, which latch code 4 after four consecutive
 *                      low reads of the key line. A test drives a line low / high with the
 *                      GPIO input pull-down / pull-up (x3emu reads a pulled input as its pull
 *                      level). The brake-lever keys 1 (PA4) and 3 (PA1) are ADC inputs: a test
 *                      applies the lever level to the channel (x3emu ADC op, tst_control.h) and
 *                      the product's ADC1 scan + DMA deliver it to the reading the debouncers
 *                      vcu_faultmon_sensor_debounce_a / _b use.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "at32f415.h"
#include "buzzer.h"
#include "control.h"
#include "passcode.h"
#include "tst_bsp_fakes.h"
#include "tst_control.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define NODE_VCU       0x16u
#define NODE_APP       0x23u
#define NODE_RESULT    0x3Eu
#define CMD_REPORT     0x03u
#define ARG_KEYS       0xD3u
#define ARG_STATUS     0x62u
#define ARG_DISPLAY    0x63u
#define ARG_VALUE      0x64u
#define ARG_RESULT     0x73u
#define SCREEN_CONFIRM 17u
#define PRESS_TICKS    5u    /* > 4 consecutive low reads latch the monitor code */
#define ROTOR_PERIOD   16u   /* rotor 0..14, then one reset tick */
#define KEY_SEQ_2_4    0x68u /* keys 2, 4 packed: (2 + 4) << 4 | (4 + 4) */
#define KEY_SEQ_4_2    0x86u
#define STATUS_MATCH   8u
#define STATUS_WRONG   4u
#define STATUS_LOCKOUT 0x0Cu
#define TRY_BITS_1     0x20u   /* passcode_ch.reload[1] */
#define KEY_SEQ_1_3    0x57u   /* keys 1, 3 packed: (1 + 4) << 4 | (3 + 4) */
#define KEY_SEQ_1234   0x5678u /* keys 1, 2, 3, 4 */
#define LEVER_HELD_RAW 800u    /* 800 >> 4 = 50: the first "held" reading of the brake-lever debouncer */

/** \brief The handlebar inputs a test can drive. */
typedef enum {
    KEY_1,    /*!< PA4 lever held: key 1 (full-weight brake, vcu_faultmon_sensor_debounce_a, EVT+0x80) */
    KEY_3,    /*!< PA1 lever held: key 3 (half-weight brake, vcu_faultmon_sensor_debounce_b, EVT+0x8C) */
    KEY_2,    /*!< PA12 low: key 2 (interlock monitor, fm_comms_wd_3), numeric select              */
    KEY_4,    /*!< PB1 low: key 4 (park monitor, fm_comms_wd_6)                                    */
    KEY_UP,   /*!< PB7 low, PB6 high: numeric up (fm_sensor_fsm_3)                                 */
    KEY_DOWN, /*!< PB6 low, PB7 high: numeric down (fm_sensor_fsm_2)                               */
    KEY_PREV, /*!< PA11 low: numeric cursor back (fm_comms_wd_1)                                   */
    KEY_NEXT  /*!< PB6 and PB7 low: numeric cursor forward (fm_comms_wd_2)                         */
} prv_key_t;

/** \brief A passcode tick as a uniform function (the set/confirm tick returns nothing). */
typedef uint8_t (*prv_tick_fn)(void);

/**
 * \brief           Drive input pins low (pull-down) or high (pull-up)
 * \param[in]       port: GPIO port
 * \param[in]       pins: pin mask
 * \param[in]       low: 1 = pull the pins low
 */
static void prv_pins(gpio_type* port, uint16_t pins, uint8_t low) {
    gpio_init_type init;

    gpio_default_para_init(&init);
    init.gpio_pins = pins;
    init.gpio_mode = GPIO_MODE_INPUT;
    init.gpio_pull = (low != 0u) ? GPIO_PULL_DOWN : GPIO_PULL_UP;
    gpio_init(port, &init);
}

/**
 * \brief           Press or release one handlebar input
 * \param[in]       key: the input
 * \param[in]       pressed: 1 = press
 */
static void prv_key_level(prv_key_t key, uint8_t pressed) {
    switch (key) {
        case KEY_1: tst_control_adc_set(VCU_ADC_SLOT_BRAKE_FULL, (pressed != 0u) ? LEVER_HELD_RAW : 0u); break;
        case KEY_3: tst_control_adc_set(VCU_ADC_SLOT_BRAKE_HALF, (pressed != 0u) ? LEVER_HELD_RAW : 0u); break;
        case KEY_2: prv_pins(GPIOA, GPIO_PINS_12, pressed); break;
        case KEY_4: prv_pins(GPIOB, GPIO_PINS_1, pressed); break;
        case KEY_UP: prv_pins(GPIOB, GPIO_PINS_7, pressed); break;
        case KEY_DOWN: prv_pins(GPIOB, GPIO_PINS_6, pressed); break;
        case KEY_PREV: prv_pins(GPIOA, GPIO_PINS_11, pressed); break;
        default: prv_pins(GPIOB, GPIO_PINS_6 | GPIO_PINS_7, pressed); break;
    }
}

/**
 * \brief           Release every input
 */
static void prv_release_all(void) {
    prv_pins(GPIOA, GPIO_PINS_11 | GPIO_PINS_12, 0u);
    prv_pins(GPIOB, GPIO_PINS_1 | GPIO_PINS_6 | GPIO_PINS_7, 0u);
    tst_control_adc_release();
}

/**
 * \brief           Run `tick` `count` times
 * \param[in]       tick: the passcode tick
 * \param[in]       count: number of ticks
 * \return          OR of the tick results
 */
static uint8_t prv_ticks(prv_tick_fn tick, uint32_t count) {
    uint8_t result = 0u;
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        result = (uint8_t)(result | tick());
    }
    return result;
}

/**
 * \brief           One key press: held for PRESS_TICKS ticks, then released for one tick
 * \param[in]       key: the input
 * \param[in]       tick: the passcode tick that consumes it
 * \return          OR of the tick results
 */
static uint8_t prv_press(prv_key_t key, prv_tick_fn tick) {
    uint8_t result;

    prv_key_level(key, 1u);
    result = prv_ticks(tick, PRESS_TICKS);
    prv_key_level(key, 0u);
    return (uint8_t)(result | tick());
}

/**
 * \brief           The set/confirm tick as a \ref prv_tick_fn
 * \return          0
 */
static uint8_t prv_set_tick(void) {
    passcode_set_confirm_tick();
    return 0u;
}

/**
 * \brief           Empty the USART ring and the captures, so the next frames are captured from index 0
 */
static void prv_quiet(void) {
    (void)tst_usart_drain();
    tst_fakes_reset();
}

/**
 * \brief           Find the first captured USART frame with `dst`, `arg`
 * \param[in]       dst: destination
 * \param[in]       arg: argument
 * \param[out]      frame: the frame when found
 * \return          1 if found
 */
static uint8_t prv_find(uint8_t dst, uint8_t arg, tst_nb_frame_t* frame) {
    uint32_t count = tst_usart_tx_count();
    uint32_t idx;
    uint8_t found = 0u;

    for (idx = 0u; (idx < count) && (idx < TST_USART_CAPTURE_MAX) && (found == 0u); ++idx) {
        const tst_usart_frame_t* raw = tst_usart_tx_get(idx);

        if ((tst_nb_parse(raw->bytes, raw->len, frame) != 0u) && (frame->dst == dst) && (frame->arg == arg)
            && (frame->src == NODE_VCU) && (frame->cmd == CMD_REPORT)) {
            found = 1u;
        }
    }
    return found;
}

/**
 * \brief           Arm the monitors with every input released (one clean read each)
 * \param[in]       tick: the passcode tick to run
 */
static void prv_arm(prv_tick_fn tick) {
    prv_release_all();
    (void)tick();
}

/**
 * \brief           Fresh boot state, fakes, USART2, the ADC scan (brake levers) and released inputs
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
    tst_control_adc_freeze();
    prv_release_all();
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* -- entry helpers ----------------------------------------------------------------------------- */

/**
 * \brief           passcode_digit_count counts the non-zero nibbles of the 32-bit code
 * \note            8 nibbles, `(code & 0xF) != 0` counts.
 */
static void test_digit_count(void) {
    TEST_ASSERT_EQUAL_INT8(0, passcode_digit_count(0u));
    TEST_ASSERT_EQUAL_INT8(2, passcode_digit_count(KEY_SEQ_2_4));
    TEST_ASSERT_EQUAL_INT8(4, passcode_digit_count(0x10101010u));
    TEST_ASSERT_EQUAL_INT8(8, passcode_digit_count(0xFFFFFFFFu));
}

/**
 * \brief           passcode_pack_keys: raw nibbles (d0 lowest) to +0x10, keys + 4 packed MSB-first returned
 * \note            word = d0 | d1 << 4 | .. | d7 << 28 stored in g_passcode_ch.disp_word; the
 *                      return skips zero entries and packs (d + 4) with the first key highest.
 */
static void test_pack_keys(void) {
    const uint8_t keys[8] = {2u, 4u, 1u, 3u, 0u, 0u, 0u, 0u};
    const uint8_t gaps[8] = {2u, 0u, 4u, 0u, 0u, 0u, 0u, 8u};

    TEST_ASSERT_EQUAL_HEX32(0x6857u, passcode_pack_keys(keys));
    TEST_ASSERT_EQUAL_HEX32(0x00003142u, g_passcode_ch.disp_word);
    TEST_ASSERT_EQUAL_HEX32(0x68Cu, passcode_pack_keys(gaps));
    TEST_ASSERT_EQUAL_HEX32(0x80000402u, g_passcode_ch.disp_word);
}

/**
 * \brief           passcode_display_word blanks digits past the count and dashes all but the cursor
 * \note            0xB past count, digits[cursor] at the cursor, 0xA elsewhere up
 *                      to the count; SCR+0xC4 bit2 -> the raw nibbles of digits[0..3].
 */
static void test_display_word(void) {
    const uint8_t digits[4] = {3u, 7u, 1u, 2u};

    g_vs.code_count = 1u;
    g_vs.code_cursor = 1u;
    TEST_ASSERT_EQUAL_HEX16(0xBB7Au, passcode_display_word(digits));
    g_vs.code_count = 3u;
    g_vs.code_cursor = 0u;
    TEST_ASSERT_EQUAL_HEX16(0xAAA3u, passcode_display_word(digits));
    g_scr.passcode_status = 0x0004u;
    TEST_ASSERT_EQUAL_HEX16(0x2173u, passcode_display_word(digits));
}

/**
 * \brief           passcode_bcd_to_decimal is ((d0 * 10 + d1) * 10 + d2) * 10 + d3
 */
static void test_bcd_to_decimal(void) {
    const uint8_t one[4] = {1u, 2u, 3u, 4u};
    const uint8_t top[4] = {9u, 9u, 9u, 9u};
    const uint8_t low[4] = {0u, 0u, 0u, 7u};

    TEST_ASSERT_EQUAL_INT16(1234, passcode_bcd_to_decimal(one));
    TEST_ASSERT_EQUAL_INT16(9999, passcode_bcd_to_decimal(top));
    TEST_ASSERT_EQUAL_INT16(7, passcode_bcd_to_decimal(low));
}

/**
 * \brief           Entry reset, numeric passcode: 4 digits, cleared entry, status 0 reported as 0x62
 * \note            Clears VS+0x50/+0x52/+0x54 and VS+0x47..0x4E; reg 0x1D b7 -> VS+0x45 = 4;
 *                      SCR+0xC4 = 0 and vcu_usart_report_send(0x16, 0x23, 2, 3, 0x62, SCR+0xC4).
 */
static void test_entry_reset_numeric(void) {
    const uint8_t zero[2] = {0u, 0u};
    tst_nb_frame_t frame;

    g_in2.f.cfg_flags = CFG_PASSCODE_NUM;
    g_vs.code_entry[3] = 9u;
    g_vs.code_cursor = 2u;
    g_vs.code_count = 2u;
    g_vs.code_digit = 5u;
    g_scr.passcode_status = 0x24u;
    passcode_entry_reset();
    TEST_ASSERT_EQUAL_UINT8(4u, g_vs.code_len);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.code_entry[3]);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_count);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_digit);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
    TEST_ASSERT_EQUAL_UINT8(2u, frame.len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(zero, frame.body, sizeof(zero));
}

/**
 * \brief           Entry reset, key passcode: length of the stored code, end mark, 0xD3 then 0x62
 * \note            Reg 0x1D b8 -> VS+0x45 = passcode_digit_count(IN2+0xE2), entry[0] = 8,
 *                      passcode_pack_keys and vcu_usart_report_send(.., 4, 3, 0xD3, disp_word).
 */
static void test_entry_reset_key_sequence(void) {
    const uint8_t keys[4] = {8u, 0u, 0u, 0u};
    tst_nb_frame_t frame;

    g_in2.f.cfg_flags = CFG_PASSCODE_KEY;
    vcu_in2_passcode_set(g_in2.f, 0x5678u);
    passcode_entry_reset();
    TEST_ASSERT_EQUAL_UINT8(4u, g_vs.code_len);
    TEST_ASSERT_EQUAL_UINT8(8u, g_vs.code_entry[0]);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_KEYS, &frame));
    TEST_ASSERT_EQUAL_UINT8(4u, frame.len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(keys, frame.body, sizeof(keys));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
}

/**
 * \brief           Entry reset with no passcode kind selected keeps the length, sends only 0x62
 * \note            Neither b7 nor b8 -> VS+0x45 untouched.
 */
static void test_entry_reset_no_kind(void) {
    tst_nb_frame_t frame;

    g_vs.code_len = 3u;
    passcode_entry_reset();
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.code_len);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
}

/* -- set / confirm ---------------------------------------------------------------------------- */

/**
 * \brief           With no change pending the tick clears the confirm state and the cancel bit
 * \note            VS+0x41 == 0 -> ch+0 = 0, ch+0x0A/0x0B = 0, reg 0x1E &= ~0x40.
 */
static void test_set_idle_clears(void) {
    g_passcode_ch.active = 1u;
    g_passcode_ch.hs_timeout = 77u;
    g_in2.f.cfg_flags2 = (uint16_t)(CFG2_PASSCODE_CANCEL | CFG2_ACK_BEEP);
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_passcode_ch.active);
    TEST_ASSERT_EQUAL_UINT16(0u, g_passcode_ch.hs_timeout);
    TEST_ASSERT_EQUAL_HEX16(CFG2_ACK_BEEP, g_in2.f.cfg_flags2);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           A pending change outside the ride state is given up: beep, status 2 to node 0x3E
 * \note            passcode_entry_reset, beep(1), VS+0x41 = 0,
 *                      vcu_usart_report_send(0x16, 0x3E, 1, 3, 0x73, &2). The same tick then falls
 *                      into the running branch (`ch+0 == 0 && VS+0x41` is now false), so the
 *                      entry is checked once more: with VS+0x45 == 0 that is an empty match.
 */
static void test_set_gives_up_outside_ride(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_vs.code_len = 4u; /* a 0-length entry would complete on the same tick (see the note) */
    g_state_block.fsm_state = VCU_FSM_LOCKED;
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(2u, g_passcode_ch.hs_timeout);
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_RESULT, ARG_RESULT, &frame));
    TEST_ASSERT_EQUAL_UINT8(1u, frame.len);
    TEST_ASSERT_EQUAL_HEX8(2u, frame.body[0]);
}

/**
 * \brief           In the ride state the confirm window ends after 3000 ticks, back to screen 1
 * \note            `prev < 3000` keeps going; else give up, and the second increment with
 *                      prev > 2999 -> nb_commit_mode(1).
 */
static void test_set_window_timeout(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_passcode_ch.active = 1u;
    g_passcode_ch.hs_timeout = PASSCODE_SET_TIMEOUT - 1u;
    g_vs.code_len = 4u;
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.passcode_set_pending);
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.ui_screen_u);
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_RESULT, ARG_RESULT, &frame));
    TEST_ASSERT_EQUAL_HEX8(2u, frame.body[0]);
}

/**
 * \brief           The cancel bit (reg 0x1E b6) ends a change in the ride state without a result report
 * \note            Reset, clear b6, VS+0x41 = 0, nb_commit_mode(1).
 */
static void test_set_cancel(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_vs.code_len = 4u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_in2.f.cfg_flags2 = CFG2_PASSCODE_CANCEL;
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.cfg_flags2);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.ui_screen_u);
    (void)tst_usart_drain();
    TEST_ASSERT_FALSE(prv_find(NODE_RESULT, ARG_RESULT, &frame));
}

/**
 * \brief           The entry starts after the hold delay: screen 0 first, then 0x11 with the length
 * \note            ch+0x0C == 0 -> nb_commit_mode(0); `prev > 10` (the 12th tick) -> active,
 *                      VS+0x45 = passcode_digit_count(VS+0x56), screen len << 8 | 0x11.
 */
static void test_set_start_delay(void) {
    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_vs.passcode_new[0] = KEY_SEQ_2_4;
    g_scr.ui_screen_u = 0x10u;
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.ui_screen_u);
    (void)prv_ticks(prv_set_tick, PASSCODE_SET_DELAY);
    TEST_ASSERT_EQUAL_UINT8(0u, g_passcode_ch.active);
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_passcode_ch.active);
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.code_len);
    TEST_ASSERT_EQUAL_HEX16((2u << 8) | SCREEN_CONFIRM, g_scr.ui_screen_u);
}

/**
 * \brief           Repeating the staged code on the handlebar commits it and reports status 1
 * \note            Keys from the monitor codes (PA12 -> 2, PB1 -> 4), complete entry ->
 *                      passcode_pack_keys == VS+0x56 -> beep(3), vcu_usart_report_send(0x16, 0x3E,
 *                      1, 3, 0x73, &1), IN2+0xE2 = VS+0x56, IN1+0x10 = 1; then reset, screen 1, VS+0x41 = 0.
 */
static void test_set_confirm_match(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_vs.passcode_new[0] = KEY_SEQ_2_4;
    g_passcode_ch.active = 1u;
    g_vs.code_len = 2u;
    prv_arm(prv_set_tick);
    (void)prv_press(KEY_2, prv_set_tick);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.code_entry[0]);
    TEST_ASSERT_EQUAL_UINT8(8u, g_vs.code_entry[1]);
    prv_quiet();
    prv_key_level(KEY_4, 1u);
    (void)prv_ticks(prv_set_tick, PRESS_TICKS);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_HEX32(KEY_SEQ_2_4, vcu_in2_passcode_get(g_in2.f));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.ui_screen_u);
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_RESULT, ARG_RESULT, &frame));
    TEST_ASSERT_EQUAL_HEX8(1u, frame.body[0]);
}

/**
 * \brief           A staged code of the two brake keys is confirmed with the levers
 * \note            EVT+0x80 == 4 (vcu_faultmon_sensor_debounce_a, PA4) -> key 1 and EVT+0x8C
 *                      == 4 (vcu_faultmon_sensor_debounce_b, PA1) -> key 3 in the confirm entry;
 *                      the complete entry packs to 0x57 == VS+0x56 -> IN2+0xE2 = 0x57, report 0x73 = 1.
 */
static void test_set_confirm_brake_keys(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_vs.passcode_new[0] = KEY_SEQ_1_3;
    g_passcode_ch.active = 1u;
    g_vs.code_len = 2u;
    prv_arm(prv_set_tick);
    (void)prv_press(KEY_1, prv_set_tick);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_entry[0]);
    prv_quiet();
    prv_key_level(KEY_3, 1u);
    (void)prv_ticks(prv_set_tick, PRESS_TICKS);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_HEX32(KEY_SEQ_1_3, vcu_in2_passcode_get(g_in2.f));
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_RESULT, ARG_RESULT, &frame));
    TEST_ASSERT_EQUAL_HEX8(1u, frame.body[0]);
}

/**
 * \brief           A different code on the handlebar reports status 2 and keeps the stored code
 * \note            beep(1), vcu_usart_report_send(.., 0x73, &2), reg 0x1E &= ~0x40.
 */
static void test_set_confirm_mismatch(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_vs.passcode_new[0] = KEY_SEQ_4_2;
    vcu_in2_passcode_set(g_in2.f, 0x1234u);
    g_passcode_ch.active = 1u;
    g_vs.code_len = 2u;
    prv_arm(prv_set_tick);
    (void)prv_press(KEY_2, prv_set_tick);
    prv_quiet();
    prv_key_level(KEY_4, 1u);
    (void)prv_ticks(prv_set_tick, PRESS_TICKS);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX32(0x1234u, vcu_in2_passcode_get(g_in2.f));
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_RESULT, ARG_RESULT, &frame));
    TEST_ASSERT_EQUAL_HEX8(2u, frame.body[0]);
}

/**
 * \brief           While the entry runs, the rotor streams 0xD3 (keys), 0x64 (countdown), 0x62 (status)
 * \note            Rotor ch+1 at 0 / 5 / 10, wraps after 0x0E.
 */
static void test_set_rotor(void) {
    tst_nb_frame_t frame;

    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_passcode_ch.active = 1u;
    g_vs.code_len = 4u;
    (void)prv_ticks(prv_set_tick, PASSCODE_ROTOR_WRAP + 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_passcode_ch.rotor_a);
    TEST_ASSERT_EQUAL_UINT32(3u, tst_usart_drain());
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_KEYS, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_VALUE, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
}

/**
 * \brief           The confirm entry takes the park key on any pending park code; the unlock only on 4
 * \note            Confirm: EVT+0x50 != 0 -> key 4. Key unlock: EVT+0x50 == 4 only, so a short-press
 *                      code (1) is left pending and adds no key.
 */
static void test_park_key_codes(void) {
    g_vs.passcode_set_pending = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_passcode_ch.active = 1u;
    g_vs.code_len = 2u;
    prv_arm(prv_set_tick);
    g_evt.mon[FM_MON_PARK].code = 1u;
    passcode_set_confirm_tick();
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT8(4u, g_vs.code_entry[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_PARK].code);

    setUp();
    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_2_4);
    g_vs.code_len = 2u;
    prv_arm(passcode_key_unlock_tick);
    g_evt.mon[FM_MON_PARK].code = 1u;
    (void)passcode_key_unlock_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT8(1u, g_evt.mon[FM_MON_PARK].code);
}

/* -- key-sequence unlock ---------------------------------------------------------------------- */

/**
 * \brief           The stored key code entered on the handlebar unlocks: returns 1, status 8, screen 1
 * \note            Keys PA12 -> 2, PB1 -> 4 appended; complete entry and timer 0 ->
 *                      passcode_pack_keys == IN2+0xE2 -> beep(3), SCR+0xC4 = 8,
 *                      nb_commit_mode(1), return 1.
 */
static void test_key_unlock_match(void) {
    uint8_t result;

    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_2_4);
    g_vs.code_len = 2u;
    prv_arm(passcode_key_unlock_tick);
    result = prv_press(KEY_2, passcode_key_unlock_tick);
    TEST_ASSERT_EQUAL_UINT8(0u, result);
    result = prv_press(KEY_4, passcode_key_unlock_tick);
    TEST_ASSERT_EQUAL_UINT8(1u, result);
    TEST_ASSERT_EQUAL_UINT16(STATUS_MATCH, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.ui_screen_u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.code_retries);
}

/**
 * \brief           All four handlebar keys, brake levers included, enter a 4-key code and unlock
 * \note            EVT+0x80 (vcu_faultmon_sensor_debounce_a, PA4 `raw >> 4 >= 0x32` five
 *                      consecutive reads) -> key 1, EVT+0x2C -> 2, EVT+0x8C
 *                      (vcu_faultmon_sensor_debounce_b, PA1) -> 3, EVT+0x50 -> 4, each appended
 *                      with end mark 8; the complete entry packs to 0x5678 == IN2+0xE2 ->
 *                      SCR+0xC4 = 8, return 1.
 */
static void test_key_unlock_brake_keys(void) {
    uint8_t result;

    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_1234);
    g_vs.code_len = 4u;
    prv_arm(passcode_key_unlock_tick);
    result = prv_press(KEY_1, passcode_key_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_entry[0]);
    result = (uint8_t)(result | prv_press(KEY_2, passcode_key_unlock_tick));
    result = (uint8_t)(result | prv_press(KEY_3, passcode_key_unlock_tick));
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.code_entry[2]);
    TEST_ASSERT_EQUAL_UINT8(0u, result);
    result = prv_press(KEY_4, passcode_key_unlock_tick);
    TEST_ASSERT_EQUAL_UINT8(1u, result);
    TEST_ASSERT_EQUAL_UINT16(STATUS_MATCH, g_scr.passcode_status);
}

/**
 * \brief           A brake lever needs five consecutive held readings; four and a release add nothing
 * \note            vcu_faultmon_sensor_debounce_a: `raw >> 4 < 0x32` clears the counter and EVT+0x80; else the
 *                      counter counts and its 5th value (> 4) latches 4. A reading one LSB below
 *                      0x320 is a release.
 */
static void test_key_unlock_lever_debounce(void) {
    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_1_3);
    g_vs.code_len = 2u;
    prv_arm(passcode_key_unlock_tick);
    prv_key_level(KEY_1, 1u);
    (void)prv_ticks(passcode_key_unlock_tick, PRESS_TICKS - 1u);
    tst_control_adc_set(VCU_ADC_SLOT_BRAKE_FULL, LEVER_HELD_RAW - 1u);
    (void)prv_ticks(passcode_key_unlock_tick, 1u);
    prv_key_level(KEY_1, 1u);
    (void)prv_ticks(passcode_key_unlock_tick, PRESS_TICKS - 1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_cursor);
    (void)prv_ticks(passcode_key_unlock_tick, 1u);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_entry[0]);
}

/**
 * \brief           Both levers latched together enter key 1 first, then key 3 on the next tick
 * \note            The if-else chain consumes EVT+0x80 (key 1) before EVT+0x8C (key 3);
 *                      vcu_faultmon_sensor_debounce_b keeps EVT+0x8C at 4 while the lever stays
 *                      held (its counter is past 4), so key 3 follows one tick later.
 *                      vcu_faultmon_sensor_debounce_a does not latch key 1 again until the lever
 *                      is released.
 */
static void test_key_unlock_both_levers_order(void) {
    uint8_t result;

    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_1_3);
    g_vs.code_len = 2u;
    prv_arm(passcode_key_unlock_tick);
    prv_key_level(KEY_1, 1u);
    prv_key_level(KEY_3, 1u);
    result = prv_ticks(passcode_key_unlock_tick, PRESS_TICKS);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_entry[0]);
    result = (uint8_t)(result | prv_ticks(passcode_key_unlock_tick, 1u));
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.code_entry[1]);
    TEST_ASSERT_EQUAL_UINT8(1u, result);
    TEST_ASSERT_EQUAL_UINT16(STATUS_MATCH, g_scr.passcode_status);
}

/**
 * \brief           A wrong key code counts a try, reports 0x62 = 4 | try bits and 0xD3, waits 200 ticks
 * \note            beep(1), cursor 0, VS+0x46++, < 3 -> SCR+0xC4 = 4 | ch.reload[tries]
 *                      ({0x30, 0x20, 0x10}), reports 0x62 and 0xD3, timer 200 (not armed). The
 *                      timer running out resets the entry (passcode_entry_reset).
 */
static void test_key_unlock_wrong(void) {
    tst_nb_frame_t frame;

    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_4_2);
    g_vs.code_len = 2u;
    prv_arm(passcode_key_unlock_tick);
    (void)prv_press(KEY_2, passcode_key_unlock_tick);
    prv_quiet();
    prv_key_level(KEY_4, 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_ticks(passcode_key_unlock_tick, 4u));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_retries);
    TEST_ASSERT_EQUAL_UINT16(STATUS_WRONG | TRY_BITS_1, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_UINT16(PASSCODE_RETRY_TICKS - 1u, g_passcode_ch.timer_sib); /* runs the same tick */
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_cursor);
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
    TEST_ASSERT_EQUAL_HEX8(STATUS_WRONG | TRY_BITS_1, frame.body[0]);
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_KEYS, &frame));
    g_passcode_ch.timer_sib = 1u;
    (void)passcode_key_unlock_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_retries);
}

/**
 * \brief           The third wrong key code arms the lock-out (status 0x0C) and shuts down 300 ticks later
 * \note            Tries >= 3 -> VS+0x42 = 1, SCR+0xC4 = 0x0C, timer 300; then tries > 2 and
 *                      the timer at 0 -> VS+0x1C = 1.
 */
static void test_key_unlock_third_try_locks_out(void) {
    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_4_2);
    g_vs.code_len = 2u;
    g_vs.code_retries = 2u;
    prv_arm(passcode_key_unlock_tick);
    (void)prv_press(KEY_2, passcode_key_unlock_tick);
    (void)prv_press(KEY_4, passcode_key_unlock_tick);
    TEST_ASSERT_EQUAL_UINT8(3u, g_vs.code_retries);
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_ARMED, g_vs.lockout_phase);
    TEST_ASSERT_EQUAL_UINT16(STATUS_LOCKOUT, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_UINT16(PASSCODE_LOCKOUT_TICKS - 3u, g_passcode_ch.timer_sib); /* 3 ticks since */
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
    g_passcode_ch.timer_sib = 1u;
    (void)passcode_key_unlock_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
}

/**
 * \brief           A wrong code while the lock-out is armed waits 20 ticks, then raises the alarm
 * \note            VS+0x42 == 1 -> timer 0x14, nb_commit_mode(0); then the timer at 0 and
 *                      VS+0x42 == 1 -> VS+0x42 = 2, reset.
 */
static void test_key_unlock_wrong_while_armed_alarms(void) {
    vcu_in2_passcode_set(g_in2.f, KEY_SEQ_4_2);
    g_vs.code_len = 2u;
    g_vs.lockout_phase = LOCKOUT_ARMED;
    g_scr.ui_screen_u = 0x11u;
    prv_arm(passcode_key_unlock_tick);
    (void)prv_press(KEY_2, passcode_key_unlock_tick);
    prv_key_level(KEY_4, 1u);
    (void)prv_ticks(passcode_key_unlock_tick, 4u);
    TEST_ASSERT_EQUAL_UINT16(PASSCODE_LOCKOUT_GRACE - 1u, g_passcode_ch.timer_sib);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.ui_screen_u);
    g_passcode_ch.timer_sib = 1u;
    (void)passcode_key_unlock_tick();
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_ALARM, g_vs.lockout_phase);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
}

/**
 * \brief           In the alarm phase the try timer is cut to 0 on the next tick
 * \note            Timer != 0 -> timer - 1, VS+0x42 == 2 -> timer = 0.
 */
static void test_key_unlock_alarm_cuts_timer(void) {
    g_vs.code_len = 4u;
    g_vs.lockout_phase = LOCKOUT_ALARM;
    g_passcode_ch.timer_sib = 100u;
    (void)passcode_key_unlock_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_passcode_ch.timer_sib);
}

/**
 * \brief           Idle key unlock streams 0xD3, 0x64 (display) and 0x62 once per 16 ticks
 * \note            Rotor ch+8 < 0x0F and timer 0 -> step 0 / 5 / 10 reports, ch+8++; else
 *                      ch+8 = 0.
 */
static void test_key_unlock_rotor(void) {
    tst_nb_frame_t frame;

    g_vs.code_len = 4u;
    (void)prv_ticks(passcode_key_unlock_tick, ROTOR_PERIOD);
    TEST_ASSERT_EQUAL_UINT16(0u, g_passcode_ch.rotor_sib);
    TEST_ASSERT_EQUAL_UINT32(3u, tst_usart_drain());
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_KEYS, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_VALUE, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
}

/* -- numeric unlock --------------------------------------------------------------------------- */

/**
 * \brief           Up / down scroll the digit under the cursor 0..9 with wrap-around
 * \note            LIGHT_HI (PB6 low) -> digit - 1, 0 -> 9;
 *                      LIGHT_LO (PB7 low) -> digit + 1, 9 -> 0; entry[cursor] = digit.
 */
static void test_digit_scroll_wraps(void) {
    g_vs.code_len = 4u;
    prv_arm(passcode_digit_unlock_tick);
    (void)prv_press(KEY_DOWN, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(9u, g_vs.code_digit);
    TEST_ASSERT_EQUAL_UINT8(9u, g_vs.code_entry[0]);
    (void)prv_press(KEY_UP, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_digit);
    (void)prv_press(KEY_UP, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_digit);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_entry[0]);
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
}

/**
 * \brief           Select on the last digit compares d0..d3 with reg 0x61: a match unlocks
 * \note            Select (PA12 / brake levers) with timer 0 -> count/cursor + 1; count >= len
 *                      -> passcode_bcd_to_decimal == (s16)IN2+0xC2 -> beep(3), SCR+0xC4 = 8,
 *                      nb_commit_mode(1), return 1.
 */
static void test_digit_unlock_match(void) {
    g_in2.f.passcode_num = 1234u;
    g_vs.code_len = 4u;
    g_vs.code_entry[0] = 1u;
    g_vs.code_entry[1] = 2u;
    g_vs.code_entry[2] = 3u;
    g_vs.code_entry[3] = 4u;
    g_vs.code_count = 3u;
    g_vs.code_cursor = 3u;
    prv_arm(passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_press(KEY_2, passcode_digit_unlock_tick));
    TEST_ASSERT_EQUAL_UINT16(4u, g_vs.code_count);
    TEST_ASSERT_EQUAL_UINT16(STATUS_MATCH, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_HEX16(1u, g_scr.ui_screen_u);
}

/**
 * \brief           A wrong number counts a try: 0x62 = 4 | try bits and the 0x63 display, 200 ticks
 * \note            beep(1), VS+0x46++, < 3 -> SCR+0xC4 = 4 | reload[tries], 0x62 report,
 *                      SCR+0xC6 = passcode_display_word, 0x63 report, timer 200; the timer running
 *                      out (not armed) resets the entry.
 */
static void test_digit_unlock_wrong(void) {
    tst_nb_frame_t frame;

    g_in2.f.passcode_num = 1234u;
    g_in2.f.cfg_flags = CFG_PASSCODE_NUM;
    g_vs.code_len = 4u;
    g_vs.code_entry[0] = 1u;
    g_vs.code_count = 3u;
    g_vs.code_cursor = 3u;
    prv_arm(passcode_digit_unlock_tick);
    prv_quiet();
    prv_key_level(KEY_2, 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_ticks(passcode_digit_unlock_tick, 4u));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.code_retries);
    TEST_ASSERT_EQUAL_UINT16(STATUS_WRONG | TRY_BITS_1, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_UINT16(PASSCODE_RETRY_TICKS - 1u, g_passcode_ch.timer_menu);
    (void)tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_DISPLAY, &frame));
    g_passcode_ch.timer_menu = 1u;
    (void)passcode_digit_unlock_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_count);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.passcode_status);
}

/**
 * \brief           The third wrong number arms the lock-out and shuts down when the 300-tick timer ends
 * \note            Timer 300, VS+0x42 = 1, SCR+0xC4 = 0x0C, 0x62 report; then tries > 2 and
 *                      the timer at 0 -> VS+0x1C = 1.
 */
static void test_digit_unlock_third_try_locks_out(void) {
    g_in2.f.passcode_num = 1234u;
    g_vs.code_len = 4u;
    g_vs.code_retries = 2u;
    g_vs.code_count = 3u;
    g_vs.code_cursor = 3u;
    prv_arm(passcode_digit_unlock_tick);
    prv_key_level(KEY_2, 1u);
    (void)prv_ticks(passcode_digit_unlock_tick, 4u);
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_ARMED, g_vs.lockout_phase);
    TEST_ASSERT_EQUAL_UINT16(STATUS_LOCKOUT, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_UINT16(PASSCODE_LOCKOUT_TICKS - 1u, g_passcode_ch.timer_menu);
    g_passcode_ch.timer_menu = 1u;
    (void)passcode_digit_unlock_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
}

/**
 * \brief           A wrong number while armed waits 20 ticks, then raises the alarm without a reset
 * \note            VS+0x42 == 1 -> timer 0x14, nb_commit_mode(0); then the timer at 0,
 *                      VS+0x42 == 1 -> VS+0x42 = 2 (no passcode_entry_reset).
 */
static void test_digit_unlock_wrong_while_armed_alarms(void) {
    g_in2.f.passcode_num = 1234u;
    g_vs.code_len = 4u;
    g_vs.lockout_phase = LOCKOUT_ARMED;
    g_vs.code_count = 3u;
    g_vs.code_cursor = 3u;
    prv_arm(passcode_digit_unlock_tick);
    prv_key_level(KEY_2, 1u);
    (void)prv_ticks(passcode_digit_unlock_tick, 4u);
    TEST_ASSERT_EQUAL_UINT16(PASSCODE_LOCKOUT_GRACE - 1u, g_passcode_ch.timer_menu);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.ui_screen_u);
    g_passcode_ch.timer_menu = 1u;
    (void)passcode_digit_unlock_tick();
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_ALARM, g_vs.lockout_phase);
    TEST_ASSERT_EQUAL_UINT16(4u, g_vs.code_count);
}

/**
 * \brief           Prev / next move the cursor within the accepted digits and reload the digit
 * \note            After the keys: PREV (PA11, code 1) cursor - 1 when
 *                      non-zero; NEXT (PB6+PB7 low, code 1) cursor + 1 while below the count;
 *                      digit = entry[cursor].
 */
static void test_digit_cursor_moves(void) {
    g_vs.code_len = 4u;
    g_vs.code_entry[0] = 5u;
    g_vs.code_entry[1] = 6u;
    g_vs.code_entry[2] = 7u;
    g_vs.code_count = 2u;
    g_vs.code_cursor = 2u;
    prv_arm(passcode_digit_unlock_tick);
    (void)prv_press(KEY_PREV, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(1u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT16(6u, g_vs.code_digit);
    (void)prv_press(KEY_NEXT, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(2u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT16(7u, g_vs.code_digit);
    (void)prv_press(KEY_NEXT, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(2u, g_vs.code_cursor);
    g_vs.code_cursor = 0u;
    (void)prv_press(KEY_PREV, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.code_cursor);
}

/**
 * \brief           Select jumps a trailing cursor back to the count, and is ignored while the timer runs
 * \note            Timer ch+4 != 0 -> nothing; cursor == count -> both + 1;
 *                      else cursor = count.
 */
static void test_digit_select_rules(void) {
    g_vs.code_len = 4u;
    g_vs.code_count = 2u;
    g_vs.code_cursor = 0u;
    prv_arm(passcode_digit_unlock_tick);
    (void)prv_press(KEY_2, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(2u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT16(2u, g_vs.code_count);
    (void)prv_press(KEY_2, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(3u, g_vs.code_cursor);
    TEST_ASSERT_EQUAL_UINT16(3u, g_vs.code_count);
    g_passcode_ch.timer_menu = 100u;
    (void)prv_press(KEY_2, passcode_digit_unlock_tick);
    TEST_ASSERT_EQUAL_UINT16(3u, g_vs.code_count);
}

/**
 * \brief           Idle numeric unlock streams 0x63 (display), 0x64 and 0x62 (= the cursor)
 * \note            Rotor ch+2 at 0 -> SCR+0xC6 = passcode_display_word + 0x63;
 *                      5 -> 0x64 (SCR+0xC8); 10 -> SCR+0xC4 = cursor + 0x62.
 */
static void test_digit_rotor(void) {
    tst_nb_frame_t frame;

    g_vs.code_len = 4u;
    g_vs.code_count = 2u;
    g_vs.code_cursor = 2u;
    (void)prv_ticks(passcode_digit_unlock_tick, ROTOR_PERIOD);
    TEST_ASSERT_EQUAL_UINT16(0u, g_passcode_ch.rotor_menu);
    TEST_ASSERT_EQUAL_UINT16(2u, g_scr.passcode_status);
    TEST_ASSERT_EQUAL_UINT32(3u, tst_usart_drain());
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_DISPLAY, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_VALUE, &frame));
    TEST_ASSERT_TRUE(prv_find(NODE_APP, ARG_STATUS, &frame));
    TEST_ASSERT_EQUAL_HEX8(2u, frame.body[0]);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_digit_count);
    RUN_TEST(test_pack_keys);
    RUN_TEST(test_display_word);
    RUN_TEST(test_bcd_to_decimal);
    RUN_TEST(test_entry_reset_numeric);
    RUN_TEST(test_entry_reset_key_sequence);
    RUN_TEST(test_entry_reset_no_kind);
    RUN_TEST(test_set_idle_clears);
    RUN_TEST(test_set_gives_up_outside_ride);
    RUN_TEST(test_set_window_timeout);
    RUN_TEST(test_set_cancel);
    RUN_TEST(test_set_start_delay);
    RUN_TEST(test_set_confirm_match);
    RUN_TEST(test_set_confirm_brake_keys);
    RUN_TEST(test_set_confirm_mismatch);
    RUN_TEST(test_set_rotor);
    RUN_TEST(test_park_key_codes);
    RUN_TEST(test_key_unlock_match);
    RUN_TEST(test_key_unlock_brake_keys);
    RUN_TEST(test_key_unlock_lever_debounce);
    RUN_TEST(test_key_unlock_both_levers_order);
    RUN_TEST(test_key_unlock_wrong);
    RUN_TEST(test_key_unlock_third_try_locks_out);
    RUN_TEST(test_key_unlock_wrong_while_armed_alarms);
    RUN_TEST(test_key_unlock_alarm_cuts_timer);
    RUN_TEST(test_key_unlock_rotor);
    RUN_TEST(test_digit_scroll_wraps);
    RUN_TEST(test_digit_unlock_match);
    RUN_TEST(test_digit_unlock_wrong);
    RUN_TEST(test_digit_unlock_third_try_locks_out);
    RUN_TEST(test_digit_unlock_wrong_while_armed_alarms);
    RUN_TEST(test_digit_cursor_moves);
    RUN_TEST(test_digit_select_rules);
    RUN_TEST(test_digit_rotor);
    tst_exit(UNITY_END());
    return 0;
}
