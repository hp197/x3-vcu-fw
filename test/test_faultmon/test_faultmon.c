/**
 * \file            test_faultmon.c
 * \brief           Fault monitor (faultmon.c): input monitors, interlock, fault latch, status builder
 * \note            Inputs are presented on the real pins (internal pull-up / pull-down,
 *                      tst_faultmon.h) and in the ADC scan buffer; outputs are the event
 *                      records, the DTC / event masks, the beep request, the mode/lighting
 *                      status, the GPIO output latch and the Ninebot frames captured on USART2.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <unity.h>
#include "buzzer.h"
#include "charging.h"
#include "control.h"
#include "faultmon.h"
#include "tst_bsp_fakes.h"
#include "tst_faultmon.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"
#include "vehicle.h" /* MF_ACTIVE */

/* Debounce counts. */
#define SHIFT4_TRIP      4u   /* 4-bit shift register: 4 lost reads saturate 0xF          */
#define SHIFT12_TRIP     12u  /* 12-bit shift register (power button): 12 lost reads     */
#define ESCALATE_HOLD    100u /* 100 dwell ticks before code 4 -> 3                        */
#define SENSOR1_TRIP     10u  /* PA9 count-up monitor latches code 1 at 10               */
#define SENSOR1_CLAMP    500u /* PA9 counter clamp                                        */
#define LEVER_CONFIRM    5u   /* brake-lever held debounce: 5th held reading latches 4    */
#define LEVER_HELD_RAW   800u /* 800 >> 4 = 50: the first "held" reading                  */
#define RETRY_SAFE       19u  /* retry counter resets the MCU on the 20th request (> 0x13) */
#define LATCH_TIMEOUT    300u /* fault-latch arm timer trips on the 300th tick            */
#define ILK_TIMEOUT      300u /* interlock armed state times out after 300 ticks          */

/* Codes written into a record's `code` byte. */
#define CODE_WARN        1u
#define CODE_FAULT       3u
#define CODE_LATCH       4u
#define CODE_CLEAR       5u

/* Event / DTC mask bits (control_event_set: lo bit ev-1, hi bit ev-17; DTC 10..25: mask0 bit code-10). */
#define EVT_LO_TEMP_HIGH 0x0008u /* event 4    */
#define EVT_LO_TEMP_LOW  0x0010u /* event 5    */
#define EVT_LO_LATCH     0x4000u /* event 0x0F */
#define EVT_HI_BMS_LOSS  0x0002u /* event 0x12 */
#define EVT_HI_VBUS_SENT 0x0008u /* event 0x14 */
#define EVT_HI_VBUS_HIGH 0x0010u /* event 0x15 */
#define EVT_HI_RAW_CODE  0x0020u /* event 0x16 */
#define EVT_HI_UNDERVOLT 0x0100u /* event 0x19 */
#define EVT_HI_TEMP_WARN 0x0200u /* event 0x1A */
#define DTC0_ESC_LOSS    0x0001u /* DTC 10     */
#define DTC0_FAULT_0B    0x0002u /* DTC 0x0B   */
#define DTC0_THROTTLE    0x0010u /* DTC 0x0E   */

/* Ninebot frame fields on the wire. */
#define NODE_VCU         0x16u
#define NODE_APP         0x23u
#define CMD_WRITE3       0x03u
#define ARG_MODE         0x25u
#define FAULT_FRAME_LEN  14u

/** \brief A monitor entry point under test. */
typedef void (*prv_monitor_fn)(void);

/** \brief One single-stage node watchdog: entry point, input line, record and trip code. */
typedef struct {
    prv_monitor_fn run; /*!< monitor entry point */
    gpio_type* port;    /*!< input port        */
    uint16_t pins;      /*!< line(s) that must all read low */
    uint8_t record;     /*!< fm_mon_id_t       */
    uint8_t trip_code;  /*!< code latched on confirmation */
} prv_node_t;

/** \brief Fresh boot state, released inputs, fakes recording */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_fm_release_all();
    vcu_usart_init();
}

/** \brief Nothing to undo */
void tearDown(void) {}

/**
 * \brief           Call a monitor `count` times
 * \param[in]       run: monitor entry point
 * \param[in]       count: number of calls
 */
static void prv_run(prv_monitor_fn run, uint32_t count) {
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        run();
    }
}

/**
 * \brief           Parse captured USART frame `index`, failing the test if it is not a valid frame
 * \param[in]       index: capture index
 * \param[out]      frame: the parsed frame
 */
static void prv_usart_frame(uint32_t index, tst_nb_frame_t* frame) {
    const tst_usart_frame_t* raw = tst_usart_tx_get(index);

    TEST_ASSERT_NOT_NULL_MESSAGE(raw, "no USART frame captured at this index");
    TEST_ASSERT_TRUE_MESSAGE(tst_nb_parse(raw->bytes, raw->len, frame), "not a 5A A5 frame of its length");
    TEST_ASSERT_TRUE_MESSAGE(frame->checksum_ok, "bad checksum");
}

/**
 * \brief           Node watchdog 1 (PA11) latches code 1 on the 4th consecutive low read
 * \note            Clean read -> +0x1A armed = 1, shift = 0; low read (PA11 = GPIOA 0x800
 *                      not high) -> VS+0x21 = 1, shift = (shift*2+1)&0xF; shift == 0xF with
 *                      armed and phase 0 -> code 1, phase 1; code != 0 -> VS+0x1D = 1.
 */
static void test_wd1_trips_on_fourth_low_read(void) {
    fm_comms_wd_1();
    tst_fm_pin(GPIOA, GPIO_PINS_11, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_1, SHIFT4_TRIP - 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_PREV].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.can_drive_active);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_disp_refresh);
    fm_comms_wd_1();
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_SUBSTATE_PREV].code);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.can_drive_active);
}

/**
 * \brief           A monitor never confirms before it has seen one clean read
 * \note            shift == 0xF with +0x1A (armed) == 0 returns before
 *                      the latch; the low read still raises VS+0x21.
 */
static void test_wd_needs_a_clean_read_to_arm(void) {
    tst_fm_pin(GPIOA, GPIO_PINS_11, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_1, 3u * SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_HEX16(0x000Fu, g_evt.mon[FM_MON_SUBSTATE_PREV].shift);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_PREV].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.can_drive_active);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_disp_refresh);
}

/**
 * \brief           One event per low run: a consumed code is not raised again until a clean read
 * \note            The latch is gated by phase (+0x21) == 0 and only a
 *                      non-saturated register clears the phase.
 */
static void test_wd1_latches_once_per_low_run(void) {
    fm_comms_wd_1();
    tst_fm_pin(GPIOA, GPIO_PINS_11, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_1, SHIFT4_TRIP);
    g_evt.mon[FM_MON_SUBSTATE_PREV].code = 0u; /* consumer takes the event */
    prv_run(fm_comms_wd_1, 2u * SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_PREV].code);
    tst_fm_pin(GPIOA, GPIO_PINS_11, TST_LEVEL_HIGH);
    fm_comms_wd_1();
    tst_fm_pin(GPIOA, GPIO_PINS_11, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_1, SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_SUBSTATE_PREV].code);
}

/**
 * \brief           Each single-stage node watchdog reads its own line and record
 * \note            Node 1 (PA11, rec +0x18, code 1), node 2 (PB7 and PB6 both low, rec
 *                      +0x0C, code 1), node 3 (PA12, rec +0x24, code 4), node 4 (PB11, rec
 *                      +0x3C, code 1), node 5 (PB2, rec +0x30, code 1). Ports GPIOA 0x40010800
 *                      / GPIOB 0x40010C00.
 */
static void test_wd_nodes_read_their_lines(void) {
    static const prv_node_t nodes[] = {
        {fm_comms_wd_1, GPIOA, GPIO_PINS_11, FM_MON_SUBSTATE_PREV, CODE_WARN},
        {fm_comms_wd_2, GPIOB, GPIO_PINS_6 | GPIO_PINS_7, FM_MON_SUBSTATE_NEXT, CODE_WARN},
        {fm_comms_wd_3, GPIOA, GPIO_PINS_12, FM_MON_INTERLOCK, CODE_LATCH},
        {fm_comms_wd_4, GPIOB, GPIO_PINS_11, FM_MON_GEAR_UP, CODE_WARN},
        {fm_comms_wd_5, GPIOB, GPIO_PINS_2, FM_MON_GEAR_DOWN, CODE_WARN},
    };
    uint32_t idx;

    for (idx = 0u; idx < sizeof(nodes) / sizeof(nodes[0]); ++idx) {
        setUp();
        nodes[idx].run();
        tst_fm_pin(nodes[idx].port, nodes[idx].pins, TST_LEVEL_LOW);
        prv_run(nodes[idx].run, SHIFT4_TRIP);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(nodes[idx].trip_code, g_evt.mon[nodes[idx].record].code, "trip code");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(1u, g_vs.can_drive_active, "fault flag VS+0x1D");
    }
}

/**
 * \brief           Node watchdog 2 needs PB6 and PB7 low together
 * \note            The input reads lost only when GPIOB 0x80 and GPIOB 0x40 both read low.
 */
static void test_wd2_needs_both_lines_low(void) {
    fm_comms_wd_2();
    tst_fm_pin(GPIOB, GPIO_PINS_6, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_2, 2u * SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_NEXT].code);
    tst_fm_pin(GPIOB, GPIO_PINS_6, TST_LEVEL_HIGH);
    tst_fm_pin(GPIOB, GPIO_PINS_7, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_2, 2u * SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_SUBSTATE_NEXT].code);
}

/**
 * \brief           Node 3 (PA12) latches 4 and a clean read demotes it to the press code 1
 * \note            Saturated -> code (+0x2C) = 4; non-saturated ->
 *                      phase 0 and code 4 -> 1.
 */
static void test_wd3_release_demotes_latch(void) {
    fm_comms_wd_3();
    tst_fm_pin(GPIOA, GPIO_PINS_12, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_3, SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_INTERLOCK].code);
    tst_fm_pin(GPIOA, GPIO_PINS_12, TST_LEVEL_HIGH);
    fm_comms_wd_3();
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_INTERLOCK].code);
}

/**
 * \brief           Node 6 (PB1): code 4 on confirmation, code 3 after a 100-tick hold
 * \note            Phase 0 -> code (+0x50) 4, phase 1; phase 1 with hold (+0x4C) `< 100` ->
 *                      hold+1, else code 3, phase 2. So code 3 arrives on the 4 + 100 + 1 =
 *                      105th low read. A clean read clears hold/phase but
 *                      keeps the escalated code 3.
 */
static void test_wd6_escalates_after_hold(void) {
    fm_comms_wd_6();
    tst_fm_pin(GPIOB, GPIO_PINS_1, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_6, SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_PARK].code);
    prv_run(fm_comms_wd_6, ESCALATE_HOLD);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_PARK].code);
    TEST_ASSERT_EQUAL_UINT16(ESCALATE_HOLD, g_evt.mon[FM_MON_PARK].hold);
    fm_comms_wd_6();
    TEST_ASSERT_EQUAL_UINT8(CODE_FAULT, g_evt.mon[FM_MON_PARK].code);
    TEST_ASSERT_EQUAL_UINT8(2u, g_evt.mon[FM_MON_PARK].phase);
    tst_fm_pin(GPIOB, GPIO_PINS_1, TST_LEVEL_HIGH);
    fm_comms_wd_6();
    TEST_ASSERT_EQUAL_UINT8(CODE_FAULT, g_evt.mon[FM_MON_PARK].code);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_PARK].hold);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_PARK].phase);
}

/**
 * \brief           Node 6 released before the hold ends reports a short press (code 1)
 * \note            Non-saturated with code 4 -> code 1.
 */
static void test_wd6_short_low_reports_press(void) {
    fm_comms_wd_6();
    tst_fm_pin(GPIOB, GPIO_PINS_1, TST_LEVEL_LOW);
    prv_run(fm_comms_wd_6, SHIFT4_TRIP + 10u);
    tst_fm_pin(GPIOB, GPIO_PINS_1, TST_LEVEL_HIGH);
    fm_comms_wd_6();
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_PARK].code);
}

/**
 * \brief           Sensor FSMs 2/3 decode the PB7/PB6 pair into the high / low channel records
 * \note            Sensor FSM 2 reads PB7 high and PB6 low into record +0x60; sensor FSM 3
 *                      reads the mirror pair (PB7 low, PB6 high) into record +0x54. Both are
 *                      two-stage (code 4, demote 4 -> 1).
 */
static void test_sensor_pairs_decode_channels(void) {
    fm_sensor_fsm_2();
    fm_sensor_fsm_3();
    tst_fm_pin(GPIOB, GPIO_PINS_6, TST_LEVEL_LOW); /* PB7 high, PB6 low: high channel */
    prv_run(fm_sensor_fsm_2, SHIFT4_TRIP);
    prv_run(fm_sensor_fsm_3, SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_LIGHT_HI].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_LO].code);
    tst_fm_pin(GPIOB, GPIO_PINS_6, TST_LEVEL_HIGH);
    tst_fm_pin(GPIOB, GPIO_PINS_7, TST_LEVEL_LOW); /* PB7 low, PB6 high: low channel */
    prv_run(fm_sensor_fsm_2, 1u);
    prv_run(fm_sensor_fsm_3, SHIFT4_TRIP);
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_LIGHT_HI].code);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_LIGHT_LO].code);
}

/**
 * \brief           Sensor FSMs 2/3 escalate a held channel to code 3 after the 100-tick hold
 * \note            Records +0x60 and +0x54: phase 1 with hold `< 100` -> hold+1, else
 *                      phase 2, code 3.
 */
static void test_sensor_pairs_escalate_after_hold(void) {
    fm_sensor_fsm_2();
    fm_sensor_fsm_3();
    tst_fm_pin(GPIOB, GPIO_PINS_6, TST_LEVEL_LOW); /* PB7 high, PB6 low: high channel */
    prv_run(fm_sensor_fsm_2, SHIFT4_TRIP + ESCALATE_HOLD);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_LIGHT_HI].code);
    fm_sensor_fsm_2();
    TEST_ASSERT_EQUAL_UINT8(CODE_FAULT, g_evt.mon[FM_MON_LIGHT_HI].code);
    tst_fm_pin(GPIOB, GPIO_PINS_6, TST_LEVEL_HIGH);
    tst_fm_pin(GPIOB, GPIO_PINS_7, TST_LEVEL_LOW); /* PB7 low, PB6 high: low channel */
    prv_run(fm_sensor_fsm_3, SHIFT4_TRIP + ESCALATE_HOLD);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_LIGHT_LO].code);
    fm_sensor_fsm_3();
    TEST_ASSERT_EQUAL_UINT8(CODE_FAULT, g_evt.mon[FM_MON_LIGHT_LO].code);
    TEST_ASSERT_EQUAL_UINT8(2u, g_evt.mon[FM_MON_LIGHT_LO].phase);
}

/**
 * \brief           Sensor FSM 1 (PA9) counts low reads, latches 1 at 10 and clamps at 500
 * \note            Reached through vcu_faultmon_tick: clean -> code 5 if the
 *                      counter (+0x70) had advanced, counter 0, armed (+0x6E) 1; low -> VS+0x21,
 *                      armed ? counter < 500 ? ++ (== 10 -> code 1).
 */
static void test_sensor1_counts_to_ten(void) {
    vcu_faultmon_tick(); /* clean read arms every monitor */
    tst_fm_pin(GPIOA, GPIO_PINS_9, TST_LEVEL_LOW);
    prv_run(vcu_faultmon_tick, SENSOR1_TRIP - 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_LIGHT_EN].code);
    vcu_faultmon_tick();
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_LIGHT_EN].code);
    prv_run(vcu_faultmon_tick, SENSOR1_CLAMP);
    TEST_ASSERT_EQUAL_UINT16(SENSOR1_CLAMP, g_evt.mon[FM_MON_LIGHT_EN].hold);
    tst_fm_pin(GPIOA, GPIO_PINS_9, TST_LEVEL_HIGH);
    vcu_faultmon_tick();
    TEST_ASSERT_EQUAL_UINT8(CODE_CLEAR, g_evt.mon[FM_MON_LIGHT_EN].code);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_LIGHT_EN].hold);
}

/**
 * \brief           Power button (PA10): code 4 on the 12th low read, code 3 after the hold
 * \note            12-bit shift register (mask 0xFFF); saturated and armed: phase 0 ->
 *                      code 4; phase 1 hold `< 100` -> hold+1, else code 3. It neither raises
 *                      the fault-seen flag nor the drive-active flag.
 */
static void test_power_button_long_press(void) {
    vcu_faultmon_can_fault_escalate();
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_run(vcu_faultmon_can_fault_escalate, SHIFT12_TRIP - 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_POWER_BTN].code);
    vcu_faultmon_can_fault_escalate();
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_POWER_BTN].code);
    prv_run(vcu_faultmon_can_fault_escalate, ESCALATE_HOLD);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_POWER_BTN].code);
    vcu_faultmon_can_fault_escalate();
    TEST_ASSERT_EQUAL_UINT8(CODE_FAULT, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_disp_refresh);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.can_drive_active);
}

/**
 * \brief           A short power-button press reads as code 1 once released
 * \note            Every non-saturated read (also the first 11 low reads)
 *                      sets armed (+0x02) = 1, hold 0, phase 0 and demotes code 4 -> 1, so a
 *                      press held from the first read still latches.
 */
static void test_power_button_short_press(void) {
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_run(vcu_faultmon_can_fault_escalate, SHIFT12_TRIP + 5u);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_POWER_BTN].code);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_HIGH);
    vcu_faultmon_can_fault_escalate();
    TEST_ASSERT_EQUAL_UINT8(CODE_WARN, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_POWER_BTN].shift);
}

/**
 * \brief           The fault-detection aggregate also runs the power-button monitor
 * \note            vcu_faultmon_tick runs node watchdogs 1, 2, 3, sensor FSMs 1, 3, 2, node
 *                      watchdogs 4, 5, 6 and the PA10 power-button monitor, in that order.
 */
static void test_tick_runs_power_button_monitor(void) {
    vcu_faultmon_tick();
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_run(vcu_faultmon_tick, SHIFT12_TRIP);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_POWER_BTN].code);
}

/**
 * \brief           Full-weight brake lever (PA4): code 4 on the 5th held reading, released clears
 * \note            ADC scan slot 2 >> 4 < 0x32 -> counter 0,
 *                      EVT+0x80 = 0; else counter <= 4 -> ++, > 4 -> EVT+0x80 = 4.
 */
static void test_brake_full_hold_debounce(void) {
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, LEVER_HELD_RAW);
    prv_run(vcu_faultmon_sensor_debounce_a, LEVER_CONFIRM - 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_BRAKE_FULL].code);
    vcu_faultmon_sensor_debounce_a();
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_BRAKE_FULL].code);
    prv_run(vcu_faultmon_sensor_debounce_a, 20u);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_BRAKE_FULL].code);
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, LEVER_HELD_RAW - 1u);
    vcu_faultmon_sensor_debounce_a();
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_BRAKE_FULL].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_BRAKE_HALF].code);
}

/**
 * \brief           Half-weight brake lever (PA1) debounce writes its own record
 * \note            ADC scan slot 1, EVT+0x8C.
 */
static void test_brake_half_hold_debounce(void) {
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_HALF, 0x0FFFu);
    prv_run(vcu_faultmon_sensor_debounce_b, LEVER_CONFIRM);
    TEST_ASSERT_EQUAL_UINT8(CODE_LATCH, g_evt.mon[FM_MON_BRAKE_HALF].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_BRAKE_FULL].code);
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_HALF, 0u);
    vcu_faultmon_sensor_debounce_b();
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_BRAKE_HALF].code);
}

/**
 * \brief           A retry request raises the recompute request; 19 requests do not reset
 * \note            IN1+0x0E == 1 -> counter+1, > 0x13 -> counter 0 and fm_system_reset
 *                      (AIRCR system reset); IN1+0x10 = 1 either way. The reset
 *                      itself (20th request) is not run: it would restart the test image.
 */
static void test_retry_trigger_requests_recompute(void) {
    vcu_faultmon_retry_trigger();
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
    g_in1.retry_req = 1u;
    prv_run(vcu_faultmon_retry_trigger, RETRY_SAFE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
}

/**
 * \brief           Fault latch with the session gate open: reset, mark handled, clear event 0x0F
 * \note            IN2+0x38 bit11 -> TMR+0x16 = 0, TMR+0x06 = 0, IN1+0x27 = 1,
 *                      control_event_clear(0x0F), return (the cmd is not looked at).
 */
static void test_fault_latch_gate_open_resets(void) {
    g_in2.f.status_flags = SF_SESSION;
    g_tmr.latch_arm_tmr = 5u;
    g_tmr.latch_reported = 1u;
    g_scr.event_mask_lo = EVT_LO_LATCH;
    vcu_faultmon_fault_latch(3u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.latch_arm_tmr);
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.latch_reported);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.arm_gate);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & EVT_LO_LATCH);
}

/**
 * \brief           Fault latch arm command: one 14-byte fault frame and screen 0x0B, once
 * \note            cmd 1 with TMR+0x06 == 0 -> TMR+0x06 = 1,
 *                      vcu_usart_report_send(ctx, 0x16, 0x23, 0x0E, cmd 3, arg 0, IN2+0x20),
 *                      nb_commit_mode(0x0B); a repeated cmd 1 sends nothing.
 */
static void test_fault_latch_arm_sends_frame_once(void) {
    tst_nb_frame_t frame;

    g_in1.arm_gate = 1u;
    vcu_faultmon_fault_latch(1u);
    vcu_faultmon_fault_latch(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, frame.src);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame.dst);
    TEST_ASSERT_EQUAL_UINT8(FAULT_FRAME_LEN, frame.len);
    TEST_ASSERT_EQUAL_HEX8(CMD_WRITE3, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(0u, frame.arg);
    prv_usart_frame(1u, &frame);
    TEST_ASSERT_EQUAL_HEX8(ARG_MODE, frame.arg);
    TEST_ASSERT_EQUAL_HEX8(0x0Bu, frame.body[0]);
    TEST_ASSERT_EQUAL_INT16(0x0B, g_scr.ui_screen);
}

/**
 * \brief           Fault latch arm timer: unhandled with a brake lever held trips on tick 300
 * \note            IN1+0x27 == 0 and IN1+0x0B == 1 -> TMR+0x16 + 1, > 299 -> VS+0x01 = 2,
 *                      IN1+0x27 = 1, nb_commit_mode(1); else timer 0.
 */
static void test_fault_latch_arm_timeout(void) {
    uint32_t idx;

    g_in1.brake_any = 1u;
    g_vs.gear_sel = 4u;
    for (idx = 0u; idx < LATCH_TIMEOUT - 1u; ++idx) {
        vcu_faultmon_fault_latch(0u);
    }
    TEST_ASSERT_EQUAL_UINT8(4u, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.arm_gate);
    vcu_faultmon_fault_latch(0u);
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.arm_gate);
    TEST_ASSERT_EQUAL_INT16(1, g_scr.ui_screen);
}

/**
 * \brief           Fault latch routes event 0x0F: set on cmd 3, cleared on any other cmd
 * \note            Tail: cmd == 3 -> control_event_set(0x0F) else control_event_clear(0x0F).
 *                      A released brake lever keeps the arm timer at 0.
 */
static void test_fault_latch_routes_event(void) {
    g_in1.arm_gate = 1u;
    g_tmr.latch_arm_tmr = 7u;
    vcu_faultmon_fault_latch(3u);
    TEST_ASSERT_EQUAL_HEX16(EVT_LO_LATCH, g_scr.event_mask_lo & EVT_LO_LATCH);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.latch_arm_tmr);
    vcu_faultmon_fault_latch(2u);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & EVT_LO_LATCH);
}

/**
 * \brief           Open the interlock entry gate: session, idle row, speed within the ceiling
 * \param[in]       speed: vehicle speed km/h
 */
static void prv_ilk_gate(uint8_t speed) {
    g_in2.f.status_flags = SF_SESSION;
    g_vs.region = 0u;
    g_vs.speed_kmh = speed;
    g_mode.cmd_lead = 40u; /* >> 1 = 20 km/h */
    g_in1.throttle_flag = 1u;
}

/**
 * \brief           Interlock idle: a request at >= 6 km/h with MODE bit2 arms it
 * \note            State 0: EVT+0x2C == 1 -> clear; MODE+0x0A bit2 and
 *                      VS+0x02 >= 6 -> state 1, MODE+0x0A |= 2; the publish flag SCR+0x52
 *                      follows MODE+0x0A bit1.
 */
static void test_interlock_arms_on_request(void) {
    prv_ilk_gate(10u);
    g_mode.mode_flags = MF_ACTIVE;
    g_evt.mon[FM_MON_INTERLOCK].code = 1u;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_HEX16(0x0002u, g_mode.mode_flags & 0x0002u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_INTERLOCK].code);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.ilk_publish);
}

/**
 * \brief           Interlock request below 6 km/h raises the mode/lighting fault 8
 * \note            State 0: no permit -> control_mode_report_fsm(8): status bits
 *                      0x18 = 8 and VS+0x36 = 300.
 */
static void test_interlock_refuses_below_floor(void) {
    prv_ilk_gate(5u);
    g_mode.mode_flags = MF_ACTIVE;
    g_evt.mon[FM_MON_INTERLOCK].code = 1u;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_HEX16(0x0008u, g_scr.mode_status & 0x0018u);
    TEST_ASSERT_EQUAL_UINT16(300u, g_vs.mode_timeout);
}

/**
 * \brief           Interlock armed: times out on the 301st tick with a long beep
 * \note            State 1: no A3C bit2 -> timer+1, > 300 -> state 0, timer 0,
 *                      buzzer_beep(1).
 */
static void test_interlock_arm_timeout(void) {
    uint32_t idx;

    prv_ilk_gate(10u);
    g_mode.mode_flags = MF_ACTIVE | 0x0002u;
    g_tmr.ilk_state = 1u;
    for (idx = 0u; idx < ILK_TIMEOUT; ++idx) {
        vcu_faultmon_interlock_tick();
    }
    TEST_ASSERT_EQUAL_UINT8(1u, g_tmr.ilk_state);
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
}

/**
 * \brief           Interlock engages on bus confirmation and aborts on a brake
 * \note            State 1: A3C+0 bit2 -> state 2, double beep, refs
 *                      TMR+0x0B = VS+0x01 and TMR+0x0D = MODE+0x08 lo. State 2: IN1+0x0B
 *                      (brake) -> state 0 + long beep.
 */
static void test_interlock_engages_and_aborts(void) {
    prv_ilk_gate(10u);
    g_mode.mode_flags = MF_ACTIVE | 0x0002u;
    g_tmr.ilk_state = 1u;
    g_vs.gear_sel = 3u;
    g_mode.drive_output_word = 0x0040u;
    g_fault_codes.comm_fault_word = 0x0004u;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(2u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT8(3u, g_tmr.ilk_ref_spd);
    TEST_ASSERT_EQUAL_UINT8(0x40u, g_tmr.ilk_ref_cmd);
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(2u, g_tmr.ilk_state);
    g_in1.brake_any = 1u;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
}

/**
 * \brief           Interlock gate closed: back to idle, publish 0, a pending request raises fault 8
 * \note            Else-branch: TMR+0x0C = 0, +0x0F = 0, SCR+0x52 = 0, MODE+0x0A &= ~2;
 *                      EVT+0x2C == 1 -> 0 and control_mode_report_fsm(8).
 */
static void test_interlock_gate_closed(void) {
    g_tmr.ilk_state = 2u;
    g_scr.ilk_publish = 1u;
    g_mode.mode_flags = 0x0002u | MF_ACTIVE;
    g_vs.region = 1u; /* not idle: gate closed */
    g_in2.f.status_flags = SF_SESSION;
    g_evt.mon[FM_MON_INTERLOCK].code = 1u;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.ilk_publish);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x0002u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_INTERLOCK].code);
    TEST_ASSERT_EQUAL_HEX16(0x0008u, g_scr.mode_status & 0x0018u);
}

/**
 * \brief           An active error code 0x0E forces the interlock back to idle
 * \note            Active error code (g_lowglob) in {0x0A, 0x0E, 0x0F, 0x10}
 *                      -> state 0 before the switch (then state 0 with no request clears MODE bit1).
 */
static void test_interlock_error_code_forces_idle(void) {
    prv_ilk_gate(10u);
    g_mode.mode_flags = MF_ACTIVE | 0x0002u;
    g_tmr.ilk_state = 2u;
    g_lowglob.active_error_code = 0x0Eu;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x0002u);
}

/**
 * \brief           Output-config bit4 (MODE+0x15) forces the interlock back to idle
 * \note            MODE+0x15 bit4 set -> state (TMR+0x0C) 0 and a pending request
 *                      (EVT+0x2C == 1) dropped, before the switch, alongside the error-code
 *                      test.
 */
static void test_interlock_output_cfg_bit4_forces_idle(void) {
    prv_ilk_gate(10u);
    g_mode.mode_flags = MF_ACTIVE | 0x0002u;
    g_tmr.ilk_state = 2u;
    g_mode.output_cfg = OCFG_BRAKE_CAP; /* MODE+0x15 bit4 */
    g_evt.mon[FM_MON_INTERLOCK].code = 1u;
    vcu_faultmon_interlock_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_tmr.ilk_state);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_INTERLOCK].code);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x0002u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.ilk_publish);
}

/**
 * \brief           ESC comm loss: DTC 10 clear while counting, set at 200
 * \note            esc_commloss_cnt < 200 -> ++, control_dtc_clear(10); else
 *                      vcu_can_mode_reset and control_dtc_set(10).
 */
static void test_status_esc_comm_loss(void) {
    g_lowglob.esc_commloss_cnt = 199u;
    fm_status_report();
    TEST_ASSERT_EQUAL_UINT16(200u, g_lowglob.esc_commloss_cnt);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.dtc_mask0 & DTC0_ESC_LOSS);
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(DTC0_ESC_LOSS, g_scr.dtc_mask0 & DTC0_ESC_LOSS);
}

/**
 * \brief           Input fault latches and the active fault code map to their DTC bits
 * \note            IN1+0x1E -> DTC 0x0E; A3C+4 lo byte == 0x0B -> DTC 0x0B; the
 *                      active-error mirror IN2+0xB0 = g_lowglob.active_error_code.
 */
static void test_status_dtc_mirrors(void) {
    g_lowglob.esc_commloss_cnt = 0u;
    g_in1.throttle_fault_latch = 1u;
    g_fault_codes.active_fault_code = 0x0Bu;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(DTC0_THROTTLE | DTC0_FAULT_0B, g_scr.dtc_mask0 & (DTC0_THROTTLE | DTC0_FAULT_0B));
    TEST_ASSERT_EQUAL_UINT16(g_lowglob.active_error_code, g_in2.f.lock_state);
    g_in1.throttle_fault_latch = 0u;
    g_fault_codes.active_fault_code = 0u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.dtc_mask0 & (DTC0_THROTTLE | DTC0_FAULT_0B));
}

/**
 * \brief           BMS comm loss raises event 0x12 at 400
 * \note            g_bms_commloss < 400 -> ++, control_event_clear(0x12); else
 *                      control_event_set(0x12).
 */
static void test_status_bms_comm_loss(void) {
    g_bms_commloss.cnt = 399u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & EVT_HI_BMS_LOSS);
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_BMS_LOSS, g_scr.event_mask_hi & EVT_HI_BMS_LOSS);
}

/**
 * \brief           Pack temperature window: >= 600 hot (event 4), / 10 < -12 cold (event 5)
 * \note            With the 0x425-received flag (IN1+0x02) set: t < 600 and not (t > 0x243
 *                      with battery 2) -> clear 4 and 0x1A; t >= 600 -> set 4; `-0xd < t / 10`
 *                      -> clear 5 else set (-129 / 10 = -12: clear). 0x5AA5 = 23205 is not a
 *                      sentinel: it reads as hot.
 */
static void test_status_temperature_window(void) {
    g_in1.bms_temp_rx = 1u;
    g_in2.f.temp_425 = 600u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_LO_TEMP_HIGH, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
    g_in2.f.temp_425 = 599u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
    g_in2.f.temp_425 = (uint16_t)-130;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_LO_TEMP_LOW, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
    g_in2.f.temp_425 = (uint16_t)-129;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
    g_in2.f.temp_425 = 0x5AA5u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_LO_TEMP_HIGH, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
}

/**
 * \brief           Before the first 0x425 frame the temperature events are held clear
 * \note            IN1+0x02 == 0 -> control_event_clear(4), control_event_clear(5),
 *                      whatever IN2+0xD6 holds.
 */
static void test_status_temperature_needs_425(void) {
    g_in1.bms_temp_rx = 0u;
    g_in2.f.temp_425 = 700u;
    g_scr.event_mask_lo = EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
    g_in2.f.temp_425 = (uint16_t)-300;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & (EVT_LO_TEMP_HIGH | EVT_LO_TEMP_LOW));
}

/**
 * \brief           580..599 (0.1 degC) with battery 2 present raises event 0x1A
 * \note            t < 600, `0x243 < t` and the battery-2 present halfword (SCR+0x0A) != 0
 *                      -> control_event_set(0x1A),
 *                      event 4 left as it was; otherwise below 600 clear 4 and 0x1A. At >= 600
 *                      only event 4 is set, 0x1A is left as it was.
 */
static void test_status_battery2_temp_warning(void) {
    g_in1.bms_temp_rx = 1u;
    g_scr.bat2_present = 1u;
    g_in2.f.temp_425 = 580u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_TEMP_WARN, g_scr.event_mask_hi & EVT_HI_TEMP_WARN);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & EVT_LO_TEMP_HIGH);
    g_in2.f.temp_425 = 600u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_TEMP_WARN, g_scr.event_mask_hi & EVT_HI_TEMP_WARN);
    TEST_ASSERT_EQUAL_HEX16(EVT_LO_TEMP_HIGH, g_scr.event_mask_lo & EVT_LO_TEMP_HIGH);
    g_in2.f.temp_425 = 599u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_LO_TEMP_HIGH, g_scr.event_mask_lo & EVT_LO_TEMP_HIGH); /* 0x1A path keeps 4 */
    g_in2.f.temp_425 = 579u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & EVT_HI_TEMP_WARN);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_lo & EVT_LO_TEMP_HIGH);
    g_in2.f.temp_425 = 590u;
    g_scr.bat2_present = 0u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & EVT_HI_TEMP_WARN);
}

/**
 * \brief           Battery code hi byte: 0xFD/0xFE sentinel (event 0x14), >= 0xBF high (0x15)
 * \note            hi == 0xFD or 0xFE -> set 0x14; else clear 0x14 and hi <= 0xBE ->
 *                      clear 0x15, else set.
 */
static void test_status_vbus_code(void) {
    g_fault_codes.vbus_code = 0xFD00u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_VBUS_SENT, g_scr.event_mask_hi & (EVT_HI_VBUS_SENT | EVT_HI_VBUS_HIGH));
    g_fault_codes.vbus_code = 0xBF00u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_VBUS_HIGH, g_scr.event_mask_hi & (EVT_HI_VBUS_SENT | EVT_HI_VBUS_HIGH));
    g_fault_codes.vbus_code = 0xBEFFu;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & (EVT_HI_VBUS_SENT | EVT_HI_VBUS_HIGH));
}

/**
 * \brief           Raw fault code above 0x6E raises event 0x16, compared as an unsigned byte
 * \note            A3C+6 read as an unsigned byte <= 0x6E -> clear 0x16, else set.
 *                      Unsigned: 0x6F..0xFF set the event (0xFD/0xFE included, the
 *                      tautological early return never fires).
 */
static void test_status_raw_code_unsigned(void) {
    g_fault_codes.raw_code = 0x6E;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & EVT_HI_RAW_CODE);
    g_fault_codes.raw_code = 0x6F;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_RAW_CODE, g_scr.event_mask_hi & EVT_HI_RAW_CODE);
    g_scr.event_mask_hi = 0u;
    g_fault_codes.raw_code = (int8_t)0x90;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_RAW_CODE, g_scr.event_mask_hi & EVT_HI_RAW_CODE);
}

/**
 * \brief           Under-volt debounce: event 0x19 at 1000, charge relay PB12 off then on after 200
 * \note            fm_uv_cnt < limit (1000, 6000 with IN1+0x05) -> ++, dwell 0, clear
 *                      0x19; else dwell < 200: dwell 0 -> fm_charge_relay(0) (PB12 clear),
 *                      ++dwell == 200 -> fm_charge_relay(1) (PB12 set); set 0x19.
 */
static void test_status_undervolt_relay(void) {
    uint32_t idx;

    gpio_bits_set(GPIOB, GPIO_PINS_12);
    g_lowglob.fm_uv_cnt = 999u;
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & EVT_HI_UNDERVOLT);
    fm_status_report();
    TEST_ASSERT_EQUAL_HEX16(EVT_HI_UNDERVOLT, g_scr.event_mask_hi & EVT_HI_UNDERVOLT);
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_12);
    for (idx = 1u; idx < 199u; ++idx) {
        fm_status_report();
    }
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_12);
    fm_status_report();
    TEST_ASSERT_EQUAL_UINT8(200u, g_charge_timer.relay_dwell);
    TEST_ASSERT_EQUAL_HEX32(GPIO_PINS_12, GPIOB->odt & GPIO_PINS_12);
}

/**
 * \brief           The long under-volt grace raises the limit to 6000
 * \note            IN1+0x05 != 0 -> limit 6000.
 */
static void test_status_undervolt_long_grace(void) {
    g_in1.uv_grace_ext = 1u;
    g_lowglob.fm_uv_cnt = 1000u;
    fm_status_report();
    TEST_ASSERT_EQUAL_UINT16(1001u, g_lowglob.fm_uv_cnt);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.event_mask_hi & EVT_HI_UNDERVOLT);
}

/**
 * \brief           Gesture detect: a 5-tick hold of a brake lever advances the index and opens a 300-tick window
 * \note            Both levers below threshold -> confirm 0; else confirm
 *                      (clamp 100) + 1, == 5 -> dwell 300, index (EVT+0x7C) + 1. With an index
 *                      pending the dwell runs down one per call.
 */
static void test_gesture_hold_advances_index(void) {
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, LEVER_HELD_RAW);
    prv_run(vcu_faultmon_gesture_detect, 4u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_BRAKE_FULL].hold);
    vcu_faultmon_gesture_detect();
    TEST_ASSERT_EQUAL_UINT16(1u, g_evt.mon[FM_MON_BRAKE_FULL].hold);
    prv_run(vcu_faultmon_gesture_detect, 10u);
    TEST_ASSERT_EQUAL_UINT16(1u, g_evt.mon[FM_MON_BRAKE_FULL].hold);
}

/**
 * \brief           Gesture index 3 with the power button held: short beep and the 0x3A report
 * \note            Index != 0 and vcu_gpio_pa10_active (PA10 low) -> dwell 0; index 3 ->
 *                      buzzer_beep(2), SCR+0x46 = 1, frame dst 0x04 len 2 cmd 3
 *                      arg 0x3A; index 0.
 */
static void test_gesture_index3_dispatch(void) {
    tst_nb_frame_t frame;
    uint32_t press;

    for (press = 0u; press < 3u; ++press) {
        tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, LEVER_HELD_RAW);
        prv_run(vcu_faultmon_gesture_detect, LEVER_CONFIRM);
        tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, 0u);
        vcu_faultmon_gesture_detect();
    }
    TEST_ASSERT_EQUAL_UINT16(3u, g_evt.mon[FM_MON_BRAKE_FULL].hold);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    vcu_faultmon_gesture_detect();
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_BRAKE_FULL].hold);
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_HEX8(0x04u, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(0x3Au, frame.arg);
    TEST_ASSERT_EQUAL_HEX8(1u, frame.body[0]);
}

/**
 * \brief           Gesture detect does nothing while inhibited
 * \note            Entry: IN1+0x04, VS+0x41 or SCR+0x40 non-zero -> return.
 */
static void test_gesture_inhibited(void) {
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, LEVER_HELD_RAW);
    g_vs.passcode_set_pending = 1u;
    prv_run(vcu_faultmon_gesture_detect, 2u * LEVER_CONFIRM);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_BRAKE_FULL].hold);
}

/**
 * \brief           Init zeroes the monitor records it owns
 * \note            The C runtime zero-fills .bss at boot; this pins the helper: records
 *                      1..9 back to zero.
 */
static void test_init_zeroes_records(void) {
    g_evt.mon[FM_MON_PARK].code = 3u;
    g_evt.mon[FM_MON_PARK].shift = 0xFu;
    g_evt.mon[FM_MON_LIGHT_EN].hold = 9u;
    vcu_faultmon_init();
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_PARK].code);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_PARK].shift);
    TEST_ASSERT_EQUAL_UINT16(0u, g_evt.mon[FM_MON_LIGHT_EN].hold);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_wd1_trips_on_fourth_low_read);
    RUN_TEST(test_wd_needs_a_clean_read_to_arm);
    RUN_TEST(test_wd1_latches_once_per_low_run);
    RUN_TEST(test_wd_nodes_read_their_lines);
    RUN_TEST(test_wd2_needs_both_lines_low);
    RUN_TEST(test_wd3_release_demotes_latch);
    RUN_TEST(test_wd6_escalates_after_hold);
    RUN_TEST(test_wd6_short_low_reports_press);
    RUN_TEST(test_sensor_pairs_decode_channels);
    RUN_TEST(test_sensor_pairs_escalate_after_hold);
    RUN_TEST(test_sensor1_counts_to_ten);
    RUN_TEST(test_power_button_long_press);
    RUN_TEST(test_power_button_short_press);
    RUN_TEST(test_tick_runs_power_button_monitor);
    RUN_TEST(test_brake_full_hold_debounce);
    RUN_TEST(test_brake_half_hold_debounce);
    RUN_TEST(test_retry_trigger_requests_recompute);
    RUN_TEST(test_fault_latch_gate_open_resets);
    RUN_TEST(test_fault_latch_arm_sends_frame_once);
    RUN_TEST(test_fault_latch_arm_timeout);
    RUN_TEST(test_fault_latch_routes_event);
    RUN_TEST(test_interlock_arms_on_request);
    RUN_TEST(test_interlock_refuses_below_floor);
    RUN_TEST(test_interlock_arm_timeout);
    RUN_TEST(test_interlock_engages_and_aborts);
    RUN_TEST(test_interlock_gate_closed);
    RUN_TEST(test_interlock_error_code_forces_idle);
    RUN_TEST(test_interlock_output_cfg_bit4_forces_idle);
    RUN_TEST(test_status_esc_comm_loss);
    RUN_TEST(test_status_dtc_mirrors);
    RUN_TEST(test_status_bms_comm_loss);
    RUN_TEST(test_status_temperature_window);
    RUN_TEST(test_status_temperature_needs_425);
    RUN_TEST(test_status_battery2_temp_warning);
    RUN_TEST(test_status_vbus_code);
    RUN_TEST(test_status_raw_code_unsigned);
    RUN_TEST(test_status_undervolt_relay);
    RUN_TEST(test_status_undervolt_long_grace);
    RUN_TEST(test_gesture_hold_advances_index);
    RUN_TEST(test_gesture_index3_dispatch);
    RUN_TEST(test_gesture_inhibited);
    RUN_TEST(test_init_zeroes_records);
    tst_exit(UNITY_END());
    return 0;
}
