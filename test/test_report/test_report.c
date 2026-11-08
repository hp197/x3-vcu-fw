/**
 * \file            test_report.c
 * \brief           Ninebot report / telemetry layer (report.c) behaviour
 * \note            Reports are captured as USART2 frames at the DMA arm, CAN frames at
 *                      can_message_transmit (no controller, \ref tst_can_tx_forward).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "at32f415.h"
#include "busproto.h"
#include "buzzer.h"
#include "can.h"
#include "report.h"
#include "scheduler.h"
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define NODE_VCU       0x16u
#define NODE_APP       0x23u
#define NODE_CTRL      0x04u
#define CMD_WRITE3     0x03u
#define ARG_MILEAGE    0x24u
#define ARG_STATUS     0x3Du
#define ARG_CANACT     0x40u
#define ARG_SUBSYS     0x60u
#define ARG_IDLE       0xD1u
#define ARG_COUNTDOWN  0xD4u
#define ARG_LIGHT_MODE 0x51u
#define FSM_IDLE       1u
#define FSM_RIDE       3u
#define FSM_CHARGING   7u
#define TELEM_PERIOD   12u /* fires when the counter read is > 10 */
#define TELEM_MAGIC    0x515Cu
#define CAN_FLUSH_MAX  16u
#define UID_ADDR       0x1FFFF7E8u
#define PIN_SUBSYS_A   (1u << 10)  /* PB10, active low */
#define PIN_SUBSYS_B   (1u << 0)   /* PB0, active low */
#define ID_COUNTDOWN   0x18EF073Eu /* prio 6, PF 0xEF, dst 0x07, src 0x3E */
#define HIST_MAGIC_IDX 16u         /* magic at history +0x20 */
#define HIST_ZERO_LAST 215u        /* [16 + 1 .. 16 + 199] zeroed */
#define HIST_TX100_CNT 14u         /* u32 0x100 frame counter at +0x1C */
#define IN2_THR_PM     0x1E0u      /* throttle per mille */
#define IN2_BRK_PM     0x1E2u      /* brake per mille */
#define IN2_SPD_PM     0x1E4u      /* speed per mille */
#define ID_NODE2_RTS   0x18EC0216u /* TP.CM VCU 0x16 -> node 0x02 */
#define ID_NODE2_CTS   0x18EC1602u /* TP.CM node 0x02 -> VCU 0x16 */
#define ID_NODE2_DT    0x18EB0216u /* TP.DT VCU 0x16 -> node 0x02 */
#define NODE2_PUSH_LEN 18u         /* n = 0x12 */
#define NODE2_SEND_DIV 10u         /* send once the count read is >= 10 */
#define NODE2_TRIES    10u         /* the send that reads 10 ends the push */

/** \brief The 18 bytes the VCU pushes to node 0x02 reg 0xD0. */
static const uint8_t k_node2_table[NODE2_PUSH_LEN] = {0x00u, 0x06u, 0x00u, 0x0Au, 0x00u, 0x14u, 0x00u, 0x05u, 0x66u,
                                                      0x02u, 0x33u, 0x00u, 0x00u, 0x01u, 0x00u, 0x01u, 0x01u, 0x00u};

/* The TMR5 overflow ISR (report.c); a vector, so no header declares it. */
void TMR5_GLOBAL_IRQHandler(void);

/** \brief Byte offset of a field inside its block (`&block.field - &block`). */
#define OFF(block, field) ((uint32_t)((uintptr_t)&(block).field - (uintptr_t)&(block)))

/**
 * \brief           Pack a date the way the long-idle check reads it: year[15:10] month[9:6] day[5:1]
 * \param[in]       year: years
 * \param[in]       month: months
 * \param[in]       day: days
 * \return          packed date
 */
static uint16_t prv_date(uint16_t year, uint16_t month, uint16_t day) {
    return (uint16_t)((year << 10) | (month << 6) | (day << 1));
}

/**
 * \brief           Read a little-endian halfword at a byte offset of a block
 * \param[in]       base: block base
 * \param[in]       off: byte offset
 * \return          the halfword
 */
static uint16_t prv_get16(volatile const void* base, uint32_t off) {
    volatile const uint8_t* cell = (volatile const uint8_t*)base + off;

    return (uint16_t)(cell[0] | ((uint16_t)cell[1] << 8));
}

/**
 * \brief           Write a little-endian halfword at a byte offset of a block
 * \param[in]       base: block base
 * \param[in]       off: byte offset
 * \param[in]       val: value
 */
static void prv_put16(volatile void* base, uint32_t off, uint16_t val) {
    volatile uint8_t* cell = (volatile uint8_t*)base + off;

    cell[0] = (uint8_t)val;
    cell[1] = (uint8_t)(val >> 8);
}

/**
 * \brief           Parse captured USART frame `index` and check its header (src 0x16)
 * \param[in]       index: capture index
 * \param[in]       dst: expected destination
 * \param[in]       arg: expected argument (cmd is always 0x03)
 * \param[in]       len: expected body length
 * \param[out]      frame: the parsed frame
 */
static void prv_usart_frame(uint32_t index, uint8_t dst, uint8_t arg, uint8_t len, tst_nb_frame_t* frame) {
    const tst_usart_frame_t* raw = tst_usart_tx_get(index);

    TEST_ASSERT_NOT_NULL_MESSAGE(raw, "no USART frame captured at this index");
    TEST_ASSERT_TRUE_MESSAGE(tst_nb_parse(raw->bytes, raw->len, frame), "not a 5A A5 frame of its length");
    TEST_ASSERT_TRUE_MESSAGE(frame->checksum_ok, "bad checksum");
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, frame->src);
    TEST_ASSERT_EQUAL_HEX8(dst, frame->dst);
    TEST_ASSERT_EQUAL_HEX8(CMD_WRITE3, frame->cmd);
    TEST_ASSERT_EQUAL_HEX8(arg, frame->arg);
    TEST_ASSERT_EQUAL_UINT8(len, frame->len);
}

/**
 * \brief           Send what is queued and expect exactly one frame with a 2-byte LE body
 * \param[in]       dst: expected destination
 * \param[in]       arg: expected argument
 * \param[in]       value: expected 16-bit body
 */
static void prv_expect_u16_report(uint8_t dst, uint8_t arg, uint16_t value) {
    const uint8_t expect[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, dst, arg, 2u, &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
    tst_fakes_reset();
    tst_can_tx_forward(0u);
}

/**
 * \brief           Pump the CAN TX ring into the capture
 * \return          number of frames captured so far
 */
static uint32_t prv_can_flush(void) {
    uint32_t pass;

    for (pass = 0u; pass < CAN_FLUSH_MAX; ++pass) {
        vcu_can_poll();
    }
    return tst_can_tx_count();
}

/**
 * \brief           Fresh boot state: product RAM, fakes, USART2 + CAN rings
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    vcu_usart_init();
    vcu_can_init();
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* ---------------------------------------------------------------------------------------- */
/* Blackboard cells                                                                          */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           The cells report.c reads and writes sit at fixed offsets of their blocks
 * \note            SCR, VS, IN1, IN2, STAT, MODE, TMR and CHGBUF field offsets the report
 *                      writers use (e.g. SCR+0x80 the CAN toggle, SCR+0x1A8 the countdown, VS+0x02
 *                      the speed cache, SCR+0x48 the mileage payload).
 */
static void test_cells_at_fixed_offsets(void) {
    TEST_ASSERT_EQUAL_HEX32(0x3Cu, OFF(g_scr, telem_3c));
    TEST_ASSERT_EQUAL_HEX32(0x40u, OFF(g_scr, gest_busy));
    TEST_ASSERT_EQUAL_HEX32(0x48u, OFF(g_scr, mileage_u));
    TEST_ASSERT_EQUAL_HEX32(0x70u, OFF(g_scr, telem_70));
    TEST_ASSERT_EQUAL_HEX32(0x7Cu, OFF(g_scr, status_pack));
    TEST_ASSERT_EQUAL_HEX32(0x80u, OFF(g_scr, can_active));
    TEST_ASSERT_EQUAL_HEX32(0xC0u, OFF(g_scr, subsys_enable));
    TEST_ASSERT_EQUAL_HEX32(0x1A2u, OFF(g_scr, idle_report));
    TEST_ASSERT_EQUAL_HEX32(0x1A8u, OFF(g_scr, countdown_report));
    TEST_ASSERT_EQUAL_HEX32(0x01u, OFF(g_vs, gear_sel));
    TEST_ASSERT_EQUAL_HEX32(0x02u, OFF(g_vs, speed_kmh));
    TEST_ASSERT_EQUAL_HEX32(0x25u, OFF(g_vs, fault_ack));
    TEST_ASSERT_EQUAL_HEX32(0x26u, OFF(g_vs, report_enable));
    TEST_ASSERT_EQUAL_HEX32(0x2Du, OFF(g_vs, reboot_req));
    TEST_ASSERT_EQUAL_HEX32(0x01u, OFF(g_in1, remote_countdown_req));
    TEST_ASSERT_EQUAL_HEX32(0x0Cu, OFF(g_in1, throttle_flag));
    TEST_ASSERT_EQUAL_HEX32(0x10u, OFF(g_in1, recompute_req));
    TEST_ASSERT_EQUAL_HEX32(0x38u, OFF(g_in2, f.status_flags));
    TEST_ASSERT_EQUAL_HEX32(0x3Au, OFF(g_in2, f.cfg_flags));
    TEST_ASSERT_EQUAL_HEX32(0x62u, OFF(g_in2, f.datetime_enable));
    TEST_ASSERT_EQUAL_HEX32(0xAEu, OFF(g_in2, f.speed_src_s));
    TEST_ASSERT_EQUAL_HEX32(0xB4u, OFF(g_in2, f.gear));
    TEST_ASSERT_EQUAL_HEX32(0x1D2u, OFF(g_in2, datetime_stored));
    TEST_ASSERT_EQUAL_HEX32(0x1FCu, OFF(g_stat, status_packed));
    TEST_ASSERT_EQUAL_HEX32(0x1FEu, OFF(g_stat, status_hi));
    TEST_ASSERT_EQUAL_HEX32(0x10u, OFF(g_mode, enable_flags));
    TEST_ASSERT_EQUAL_HEX32(0x22u, OFF(g_tmr, poweroff_countdown));
    TEST_ASSERT_EQUAL_HEX32(0x08u, OFF(g_chgbuf, subsys_reported));
}

/* ---------------------------------------------------------------------------------------- */
/* Small report writers                                                                      */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           report_u16: register idx of page 1 = value, sent `16 -> 04 cmd 3 arg idx`
 * \note            SCR[idx*2] = v; vcu_usart_report_send(.., 0x16, 4, 2, 3, idx, SCR+idx*2).
 *                      The idle boot sends `16 04 03 51 0200`.
 */
static void test_u16_report(void) {
    report_u16(ARG_LIGHT_MODE, 0x0201u);
    TEST_ASSERT_EQUAL_HEX16(0x0201u, g_scr.reg[ARG_LIGHT_MODE]);
    prv_expect_u16_report(NODE_CTRL, ARG_LIGHT_MODE, 0x0201u);
}

/**
 * \brief           report_can_toggle: SCR+0x80 = value, sent to the app as arg 0x40
 * \note            vcu_usart_report_send(.., 0x16, 0x23, 2, 3, 0x40, SCR+0x80).
 */
static void test_can_toggle_report(void) {
    report_can_toggle(-2);
    TEST_ASSERT_EQUAL_INT16(-2, g_scr.can_active);
    prv_expect_u16_report(NODE_APP, ARG_CANACT, 0xFFFEu);
}

/**
 * \brief           report_onchange_24: sent only when the value differs from the last one
 * \note            `cache != v` -> SCR+0x48 = v, cache = v, send arg 0x24. The idle boot sends
 *                      `16 23 03 24 1e00` once.
 */
static void test_mileage_report_on_change(void) {
    report_onchange_24(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    report_onchange_24(30u);
    prv_expect_u16_report(NODE_APP, ARG_MILEAGE, 30u);
    report_onchange_24(30u);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    report_onchange_24(31u);
    prv_expect_u16_report(NODE_APP, ARG_MILEAGE, 31u);
}

/**
 * \brief           Subsystem mask: MODE enable_flags[1:0] and SCR+0xC0 = mask, sent on change
 * \note            Mask B / mask A / mask 0 writers: `(flags & 0xFFFC) + mask`,
 *                      SCR+0xC0 = mask, send arg 0x60 when it differs from CHGBUF+8, then cache it.
 */
static void test_subsys_mask_report_on_change(void) {
    g_mode.enable_flags = 0xA5A7u;
    report_subsys_mask_set(VCU_SUBSYS_A);
    TEST_ASSERT_EQUAL_HEX16(0xA5A6u, g_mode.enable_flags);
    TEST_ASSERT_EQUAL_HEX16(VCU_SUBSYS_A, g_scr.subsys_enable);
    TEST_ASSERT_EQUAL_HEX16(VCU_SUBSYS_A, g_chgbuf.subsys_reported);
    prv_expect_u16_report(NODE_APP, ARG_SUBSYS, VCU_SUBSYS_A);
    report_subsys_mask_set(VCU_SUBSYS_A);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    report_subsys_mask_set(0u);
    TEST_ASSERT_EQUAL_HEX16(0xA5A4u, g_mode.enable_flags);
    prv_expect_u16_report(NODE_APP, ARG_SUBSYS, 0u);
}

/**
 * \brief           report_subsys_enable drives PB10 and PB0 low and reports mask 3
 * \note            control_subsys_a_enable(1) / control_subsys_b_enable(1) (active-low pins),
 *                      enable_flags |= 3, SCR+0xC0 = 3, report arg 0x60 on change.
 */
static void test_subsys_enable_both(void) {
    GPIOB->odt |= PIN_SUBSYS_A | PIN_SUBSYS_B;
    report_subsys_enable();
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & (PIN_SUBSYS_A | PIN_SUBSYS_B));
    TEST_ASSERT_EQUAL_HEX16(3u, g_mode.enable_flags & 3u);
    prv_expect_u16_report(NODE_APP, ARG_SUBSYS, 3u);
}

/**
 * \brief           Bit-field packer: a & 0x3F at [27:22], b & 0x3F at [21:16], c << 8, d (no masks)
 * \note            `d | c << 8 | (b & 0x3F) << 16 | (a & 0x3F) << 22`; c and d
 *                      are not masked, so a wide c or d spills into the fields above it.
 */
static void test_pack_bitfields(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0FFFFFFFu, report_pack_bitfields(0x3Fu, 0x3Fu, 0xFFu, 0xFFu));
    TEST_ASSERT_EQUAL_HEX32(0x00C10000u, report_pack_bitfields(0x43u, 0x41u, 0u, 0u));
    TEST_ASSERT_EQUAL_HEX32(0x00012300u, report_pack_bitfields(0u, 0u, 0x123u, 0u));
    TEST_ASSERT_EQUAL_HEX32(0x00000301u, report_pack_bitfields(0u, 0u, 0x02u, 0x101u));
}

/* ---------------------------------------------------------------------------------------- */
/* Status reports                                                                            */
/* ---------------------------------------------------------------------------------------- */

/** \brief A status word with distinct values in the four fields ([21:17] [16:12] [11:6] [5:0]). */
#define STATUS_WORD ((0x15u << 17) | (0x0Au << 12) | (0x2Bu << 6) | 0x31u | 0xC0000000u)

/**
 * \brief           Expect one status frame: reg 0x3D, then the four fields one byte each
 */
static void prv_expect_status_frame(void) {
    const uint8_t expect[6] = {0xEFu, 0xBEu, 0x15u, 0x0Au, 0x2Bu, 0x31u};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, NODE_APP, ARG_STATUS, sizeof(expect), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
}

/**
 * \brief           report_status: gated on VS+0x26, sets the one-shot flag, 6-byte body
 * \note            `VS+0x26 != 0` -> one-shot flag = 1, SCR+0x7C = the bytes
 *                      [21:17], [16:12], [11:6], [5:0] of *src, send 6 bytes from SCR+0x7A, arg 0x3D.
 */
static void test_status_report(void) {
    const volatile uint32_t word = STATUS_WORD;

    g_scr.reg[ARG_STATUS] = 0xBEEFu;
    report_status(&word);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT8(0u, g_oneshot.report_flag);
    g_vs.report_enable = 1u;
    report_status(&word);
    TEST_ASSERT_EQUAL_UINT8(1u, g_oneshot.report_flag);
    prv_expect_status_frame();
}

/**
 * \brief           Slow task: nothing when disabled or outside states 3 / 7
 * \note            VS+0x26 == 0 or state not 3/7 -> return,
 *                      the one-shot flag untouched.
 */
static void test_slow_report_gates(void) {
    g_stat.status_packed = STATUS_WORD;
    g_state_block.fsm_state = FSM_RIDE;
    report_task_slow();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    g_vs.report_enable = 1u;
    g_state_block.fsm_state = FSM_IDLE;
    g_oneshot.report_flag = 1u;
    report_task_slow();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT8(1u, g_oneshot.report_flag);
}

/**
 * \brief           Slow task in state 3 / 7: a pending one-shot is consumed instead of a send
 * \note            flag != 0 -> flag = 0, return; else pack STAT+0x1FC as report_status does
 *                      and send it.
 */
static void test_slow_report_skips_after_oneshot(void) {
    g_vs.report_enable = 1u;
    g_stat.status_packed = STATUS_WORD;
    g_scr.reg[ARG_STATUS] = 0xBEEFu;
    g_state_block.fsm_state = FSM_RIDE;
    g_oneshot.report_flag = 1u;
    report_task_slow();
    TEST_ASSERT_EQUAL_UINT8(0u, g_oneshot.report_flag);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    g_state_block.fsm_state = FSM_CHARGING;
    report_task_slow();
    prv_expect_status_frame();
}

/* ---------------------------------------------------------------------------------------- */
/* Ninebot event-report sink                                                                 */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Up to 5 payload bytes: one 0x18EF<dst><src> frame `01 cmd arg payload`
 * \note            `n < 6`: vcu_can_ninebot_frame(6, 1, src, dst, cmd, arg, p, n), then
 *                      vcu_can_send_ext(id & 0x1FFFFFFF, data, dlc = n + 3).
 */
static void test_event_report_single_frame(void) {
    const uint8_t payload[5] = {0xA1u, 0xA2u, 0xA3u, 0xA4u, 0xA5u};
    const uint8_t expect[8] = {0x01u, 0x5Au, 0x6Bu, 0xA1u, 0xA2u, 0xA3u, 0xA4u, 0xA5u};
    const tst_can_frame_t* frame;

    report_ninebot(NODE_VCU, 0x3Du, sizeof(payload), 0x5Au, 0x6Bu, payload);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_EQUAL_UINT8(1u, frame->ext);
    TEST_ASSERT_EQUAL_HEX32(0x18EF3D16u, frame->id);
    TEST_ASSERT_EQUAL_UINT8(8u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame->data, sizeof(expect));
}

/**
 * \brief           6 payload bytes or more go through the TP tunnel: RTS for n + 5 bytes
 * \note            `n >= 6`: bp_tp_send(&g_bp_mgr, 7, 1, src, dst, cmd, arg, p, n, 0); the fast
 *                      service sends TP.CM RTS `10 <n+5> 00 <packets>`.
 */
static void test_event_report_tp_tunnel(void) {
    const uint8_t payload[6] = {1u, 2u, 3u, 4u, 5u, 6u};
    const uint8_t rts[8] = {0x10u, 11u, 0u, 2u, 0xFFu, 0x00u, 0xEFu, 0x00u};
    const tst_can_frame_t* frame;

    vcu_busproto_init();
    (void)tst_usart_drain();
    report_ninebot(NODE_VCU, 0x3Du, sizeof(payload), 0x5Au, 0x6Bu, payload);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
    bp_task_fast_service();
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_EQUAL_HEX32(0x18EC3D16u, frame->id);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rts, frame->data, sizeof(rts));
}

/* ---------------------------------------------------------------------------------------- */
/* Power-off countdown and long-idle date check                                              */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Countdown: 300 -> 0, every 100 a short beep + `arg D4` remaining / 100, at 0 the
 *                      gate cleared (no beep); at display 2 a CAN event 3E -> 07 cmd 0x7A
 * \note            IN1+1 == 0 -> TMR+0x22 = 300; else n % 100 == 0 -> buzzer_beep(2),
 *                      SCR+0x1A8 = n / 100, report (.., 0x23, 2, 3, 0xD4) and at 2
 *                      report_ninebot(0x3E, 7, 1, 0x7A, 0, &0); n--. n == 0 -> IN1+1 = 0 only,
 *                      without a beep.
 */
static void test_countdown(void) {
    const uint8_t event[4] = {0x01u, 0x7Au, 0x00u, 0x00u};
    uint32_t tick;

    g_tmr.poweroff_countdown = 7u;
    report_countdown();
    TEST_ASSERT_EQUAL_UINT16(300u, g_tmr.poweroff_countdown);
    g_in1.remote_countdown_req = 1u;
    report_countdown();
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(299u, g_tmr.poweroff_countdown);
    prv_expect_u16_report(NODE_APP, ARG_COUNTDOWN, 3u);
    for (tick = 0u; tick < 99u; ++tick) {
        report_countdown();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    report_countdown(); /* 200 */
    prv_expect_u16_report(NODE_APP, ARG_COUNTDOWN, 2u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    TEST_ASSERT_EQUAL_HEX32(ID_COUNTDOWN, tst_can_tx_get(0u)->id);
    TEST_ASSERT_EQUAL_UINT8(sizeof(event), tst_can_tx_get(0u)->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(event, tst_can_tx_get(0u)->data, sizeof(event));
    for (tick = 0u; tick < 100u; ++tick) {
        report_countdown();
    }
    prv_expect_u16_report(NODE_APP, ARG_COUNTDOWN, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
    for (tick = 0u; tick < 99u; ++tick) {
        report_countdown();
    }
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.poweroff_countdown);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.remote_countdown_req);
    report_countdown();
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.remote_countdown_req);
    TEST_ASSERT_EQUAL_UINT16(0u, g_tmr.poweroff_countdown);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           Arm every gate of the long-idle check and store a date
 * \param[in]       stored: stored packed date
 * \param[in]       now: RTC-sampled packed date
 */
static void prv_datetime_arm(uint16_t stored, uint16_t now) {
    g_in2.f.status_flags = SF_SESSION;
    g_in2.f.datetime_enable = 0x0008u;
    g_vs.report_enable = 1u;
    g_in2.datetime_stored = stored;
    g_stat.status_hi = now;
}

/**
 * \brief           Long idle: more than 89 days (year*365 + month*30 + day) -> flag + `arg D1 01 00`
 * \note            Gates IN2+0x38 bit 11, IN2+0x62 bit 3, VS+0x26; VS+0x25 == 0 and
 *                      `0x59 < delta` -> VS+0x25 = 1, SCR+0x1A2 = 1, IN2+0x38 |= 8, send arg 0xD1.
 */
static void test_datetime_long_idle_threshold(void) {
    prv_datetime_arm(prv_date(5u, 1u, 1u), prv_date(5u, 4u, 0u)); /* 120 - 31 = 89 days */
    report_datetime();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.fault_ack);
    g_stat.status_hi = prv_date(5u, 4u, 1u); /* 90 days */
    report_datetime();
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.fault_ack);
    TEST_ASSERT_EQUAL_HEX16(SF_SESSION | 0x0008u, g_in2.f.status_flags);
    prv_expect_u16_report(NODE_APP, ARG_IDLE, 1u);
    report_datetime();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           Long idle gates: any of the three gates clear -> nothing
 * \note            The three tests guard the whole body.
 */
static void test_datetime_gates(void) {
    prv_datetime_arm(prv_date(1u, 0u, 0u), prv_date(9u, 0u, 0u));
    g_vs.report_enable = 0u;
    report_datetime();
    prv_datetime_arm(prv_date(1u, 0u, 0u), prv_date(9u, 0u, 0u));
    g_in2.f.datetime_enable = 0x0007u;
    report_datetime();
    prv_datetime_arm(prv_date(1u, 0u, 0u), prv_date(9u, 0u, 0u));
    g_in2.f.status_flags = 0u;
    report_datetime();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.fault_ack);
}

/**
 * \brief           A date earlier than the stored one counts as ~65535 years: it reports
 * \note            The year difference is `& 0xFFFF` before `* 0x16D`.
 */
static void test_datetime_year_wrap_reports(void) {
    prv_datetime_arm(prv_date(6u, 0u, 0u), prv_date(5u, 0u, 0u));
    report_datetime();
    prv_expect_u16_report(NODE_APP, ARG_IDLE, 1u);
}

/**
 * \brief           Reset request: flag bit 3 cleared, the RTC date stored, IN1+0x10 = 1
 * \note            VS+0x2D != 0 -> VS+0x2D = 0, IN2+0x38 &= 0xFFF7,
 *                      IN2+0x1D2 = STAT+0x1FE, IN1+0x10 = 1 (inside the same gates).
 */
static void test_datetime_reset_request(void) {
    prv_datetime_arm(prv_date(5u, 1u, 1u), prv_date(5u, 2u, 0u));
    g_in2.f.status_flags = SF_SESSION | 0x0008u;
    g_vs.reboot_req = 1u;
    report_datetime();
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.reboot_req);
    TEST_ASSERT_EQUAL_HEX16(SF_SESSION, g_in2.f.status_flags);
    TEST_ASSERT_EQUAL_HEX16(prv_date(5u, 2u, 0u), g_in2.datetime_stored);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/* ---------------------------------------------------------------------------------------- */
/* Telemetry frame builder                                                                   */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Speed caches: VS+2 = speed / 10 (signed); display = VS+2 (km/h) or speed x 0.0621 (mph)
 * \note            VS+2 = (char)(s / 10); IN2+0x3A bit 3 -> SCR+0x70 = 1,
 *                      SCR+0x40 = __aeabi_d2iz((double)s * 0.0621) (0.0621 = 0x3FAFCB923A29C77A),
 *                      else SCR+0x70 = 0, SCR+0x40 = VS+2 (ldrb);
 *                      SCR+0x3C = VS+1; IN2+0xB4 = VS+1 while IN1+0x0C == 0.
 */
static void test_telemetry_speed_scaling(void) {
    g_in2.f.speed_src_s = 253;
    g_vs.gear_sel = 3u;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(25u, g_vs.speed_kmh);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.telem_70);
    TEST_ASSERT_EQUAL_INT16(25, g_scr.gest_busy);
    TEST_ASSERT_EQUAL_UINT16(3u, g_scr.telem_3c);
    TEST_ASSERT_EQUAL_INT16(3, g_in2.f.gear);
    g_in2.f.cfg_flags = CFG_IMPERIAL;
    g_in1.throttle_flag = 1u;
    g_vs.gear_sel = 2u;
    g_in2.f.speed_src_s = 1600;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.telem_70);
    TEST_ASSERT_EQUAL_INT16(99, g_scr.gest_busy); /* 1600 * 0.0621 = 99.36 */
    TEST_ASSERT_EQUAL_INT16(3, g_in2.f.gear);     /* not copied while the throttle flag is set */
    g_in2.f.speed_src_s = -25;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX8(0xFEu, g_vs.speed_kmh);               /* -2 */
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, (uint16_t)g_scr.gest_busy); /* d2iz(-1.5525) = -1 */
    g_in2.f.cfg_flags = 0u;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX16(0x00FEu, (uint16_t)g_scr.gest_busy); /* ldrb VS+2 */
}

/**
 * \brief           Gauge cells: throttle / brake per mille and speed per mille of the mode's column 6
 * \note            IN2+0x1E0 = (MODE+8 & 0xFF) * 1000 / 200,
 *                      IN2+0x1E2 = (MODE+8 >> 8) * 1000 / 200, IN2+0x1E4 = (u32)(s * 100) /
 *                      (u32)table[VS+0][6] (ride-mode table, row 0 = 50), then
 *                      `if ((s16)IN2+0x1E4 > 1000) = 1000`.
 */
static void test_telemetry_gauge_cells(void) {
    g_mode.drive_output_word = 0xC832u; /* throttle 50, brake 200 */
    g_in2.f.speed_src_s = 253;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT16(250u, prv_get16(&g_in2, IN2_THR_PM));
    TEST_ASSERT_EQUAL_UINT16(1000u, prv_get16(&g_in2, IN2_BRK_PM));
    TEST_ASSERT_EQUAL_UINT16(506u, prv_get16(&g_in2, IN2_SPD_PM));
    g_in2.f.speed_src_s = 600;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT16(1000u, prv_get16(&g_in2, IN2_SPD_PM));
    g_in2.f.speed_src_s = -25;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX16(0xB81Fu, prv_get16(&g_in2, IN2_SPD_PM)); /* 0xFFFFF63C / 50, not clamped */
}

/**
 * \brief           Status bit 1 is cleared in states 3 / 7 and set in every other state
 * \note            State 3 or 7 -> IN2+0x38 &= 0xFFFD, else |= 2.
 */
static void test_telemetry_hold_bit(void) {
    g_state_block.fsm_state = FSM_IDLE;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX16(0x0002u, g_in2.f.status_flags & 0x0002u);
    g_state_block.fsm_state = FSM_CHARGING;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.status_flags & 0x0002u);
}

/**
 * \brief           History block: lazy init once (magic 0x515C at [0x10], [0x0C] and [0x11..0xD7]
 *                      zeroed); the block is assembled on every 12th call
 * \note            `[0x10] != 0x515C` -> [0x0C] = 0, magic, [0x10 + 1 .. 0x10 + 199] = 0; the
 *                      divider is post-incremented, `old > 10` -> divider = 0 and the copy.
 */
static void test_telemetry_frame_period_and_init(void) {
    uint32_t call;

    g_telem_frame[0x0Cu] = 0x1111u;
    g_telem_frame[HIST_ZERO_LAST] = 0x2222u;
    g_telem_frame[HIST_ZERO_LAST + 1u] = 0x3333u;
    g_telem_frame[0u] = 0x0A0Au;
    g_stat.soc_u = 77u;
    for (call = 1u; call < TELEM_PERIOD; ++call) {
        report_telemetry_build();
    }
    TEST_ASSERT_EQUAL_HEX16(TELEM_MAGIC, g_telem_frame[HIST_MAGIC_IDX]);
    TEST_ASSERT_EQUAL_HEX16(0u, g_telem_frame[0x0Cu]);
    TEST_ASSERT_EQUAL_HEX16(0u, g_telem_frame[HIST_ZERO_LAST]);
    TEST_ASSERT_EQUAL_HEX16(0x3333u, g_telem_frame[HIST_ZERO_LAST + 1u]);
    TEST_ASSERT_EQUAL_HEX16(0u, g_telem_frame[0x35u]);
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX16(77u, g_telem_frame[0x35u]);
    TEST_ASSERT_EQUAL_HEX16(0x0A0Au, g_telem_frame[0x19u]);
    g_stat.soc_u = 78u;
    g_telem_frame[0x0Cu] = 0x4444u; /* magic intact: no second init */
    for (call = 1u; call < TELEM_PERIOD; ++call) {
        report_telemetry_build();
    }
    TEST_ASSERT_EQUAL_HEX16(77u, g_telem_frame[0x35u]);
    report_telemetry_build();
    TEST_ASSERT_EQUAL_HEX16(78u, g_telem_frame[0x35u]);
    TEST_ASSERT_EQUAL_HEX16(0x4444u, g_telem_frame[0x32u]); /* [0x32] = [0x0C] */
}

/**
 * \brief           History copy: CAN mirrors, scaled MODE words, SCR masks (dtc_mask2 twice), speed,
 *                      the 0x100 counter, range reserve and the battery-2 cells
 * \note            [0x11] = MODE curr_word & 0xFF, [0x12] = we_word &
 *                      0xFF, [0x13..0x18] = STAT +0x122/+0x12A/+0x1DA/+0x1D8/+0x118/+0x11A, [0x19..0x1C] =
 *                      [0..3], [0x1D] = A3C+2 & 0xFF, [0x1E] = [4], [0x1F] = [5], [0x20] = [0x0B], [0x21] =
 *                      [6], [0x22] = [7], [0x23] = IN2+0xAE, [0x24..0x26] = [8..0x0A], [0x27..0x2C] =
 *                      SCR+0x24, +0x26, +0x28, +0x28, +0x2A, +0x2C, [0x2D..0x30] = IN2+0x2E..+0x34,
 *                      [0x31] = IN2+0x1E8, [0x32] = [0x0C], u32 [0x33] = u32 [0x0E], [0x35] = STAT+0x11E,
 *                      [0x36] = IN2+0xBC, [0x37..0x3A] = B2 +0x11E/+0x118/+0x11A/+0x12C.
 */
static void test_telemetry_frame_fields(void) {
    static const uint8_t mirror_src[] = {0u, 1u, 2u, 3u, 4u, 5u, 0x0Bu, 6u, 7u, 8u, 9u, 0x0Au};
    static const uint8_t mirror_dst[] = {0x19u, 0x1Au, 0x1Bu, 0x1Cu, 0x1Eu, 0x1Fu,
                                         0x20u, 0x21u, 0x22u, 0x24u, 0x25u, 0x26u};
    uint32_t idx;

    for (idx = 0u; idx < sizeof(mirror_src); ++idx) {
        g_telem_frame[mirror_src[idx]] = (uint16_t)(0x0100u + idx);
    }
    g_telem_frame[HIST_TX100_CNT] = 0x5678u;
    g_telem_frame[HIST_TX100_CNT + 1u] = 0x1234u;
    g_mode.curr_word = 0x1234u;
    g_mode.we_word = 0x5A78u;
    g_fault_codes.vbus_code = 0x0A9Bu;
    g_in2.f.speed_src = 0x4321u;
    g_scr.dtc_mask0 = 0x1001u;
    g_scr.dtc_mask1 = 0x1002u;
    g_scr.dtc_mask2 = 0x1003u;
    g_scr.event_mask_lo = 0x1004u;
    g_scr.event_mask_hi = 0x1005u;
    g_stat.curr_limit = 0x2001;
    g_stat.curr_sample = 0x2005;
    g_in2.f.range_reserve = 0x3001;
    g_stat2.soc = 0x4001;
    g_stat2.curr_sample = 0x4002;
    g_stat2.volt_sample = 0x4003;
    g_stat2.temp_array[0] = 0x4004u;
    prv_put16(&g_in2, 0x34u, 0x3002u);
    for (idx = 0u; idx < TELEM_PERIOD; ++idx) {
        report_telemetry_build();
    }
    for (idx = 0u; idx < sizeof(mirror_src); ++idx) {
        TEST_ASSERT_EQUAL_HEX16(0x0100u + idx, g_telem_frame[mirror_dst[idx]]);
    }
    TEST_ASSERT_EQUAL_HEX16(0x0034u, g_telem_frame[0x11u]);
    TEST_ASSERT_EQUAL_HEX16(0x0078u, g_telem_frame[0x12u]);
    TEST_ASSERT_EQUAL_HEX16(0x2001u, g_telem_frame[0x13u]);
    TEST_ASSERT_EQUAL_HEX16(0x2005u, g_telem_frame[0x17u]);
    TEST_ASSERT_EQUAL_HEX16(0x009Bu, g_telem_frame[0x1Du]);
    TEST_ASSERT_EQUAL_HEX16(0x4321u, g_telem_frame[0x23u]);
    TEST_ASSERT_EQUAL_HEX16(0x1001u, g_telem_frame[0x27u]);
    TEST_ASSERT_EQUAL_HEX16(0x1002u, g_telem_frame[0x28u]);
    TEST_ASSERT_EQUAL_HEX16(0x1003u, g_telem_frame[0x29u]);
    TEST_ASSERT_EQUAL_HEX16(0x1003u, g_telem_frame[0x2Au]);
    TEST_ASSERT_EQUAL_HEX16(0x1004u, g_telem_frame[0x2Bu]);
    TEST_ASSERT_EQUAL_HEX16(0x1005u, g_telem_frame[0x2Cu]);
    TEST_ASSERT_EQUAL_HEX16(0x3002u, g_telem_frame[0x30u]);
    TEST_ASSERT_EQUAL_HEX16(0x5678u, g_telem_frame[0x33u]);
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_telem_frame[0x34u]);
    TEST_ASSERT_EQUAL_HEX16(0x3001u, g_telem_frame[0x36u]);
    TEST_ASSERT_EQUAL_HEX16(0x4001u, g_telem_frame[0x37u]);
    TEST_ASSERT_EQUAL_HEX16(0x4002u, g_telem_frame[0x38u]);
    TEST_ASSERT_EQUAL_HEX16(0x4003u, g_telem_frame[0x39u]);
    TEST_ASSERT_EQUAL_HEX16(0x4004u, g_telem_frame[0x3Au]);
}

/* ---------------------------------------------------------------------------------------- */
/* Node 0x02 register 0xD0 push                                                              */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Not pending: the send divider and the try count are held at 0, nothing is sent
 * \note            `ubfx(VS+0x18, 21, 2) != 1` -> state+2 = 0,
 *                      state+1 = 0, return.
 */
static void test_node2_push_idle_resets_counters(void) {
    vcu_busproto_init();
    (void)tst_usart_drain();
    g_node2_push.send_div = 7u;
    g_node2_push.tries = 4u;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(0u, g_node2_push.send_div);
    TEST_ASSERT_EQUAL_UINT8(0u, g_node2_push.tries);
    g_node2_push.state = NODE2_PUSH_DONE;
    g_node2_push.send_div = 7u;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(0u, g_node2_push.send_div);
    bp_task_fast_service();
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
}

/**
 * \brief           Pending: every 11th call sends the 18-byte table as node 0x02 cmd 2 arg 0xD0 (TP tunnel)
 * \note            old = state+1++, `old >= 10` -> state+1 = 0 and
 *                      report_ninebot(0x16, 2, 0x12, 2, 0xD0, table); 18 bytes go through
 *                      bp_tp_send: RTS 23 bytes / 4 packets, the DT frames carry 01 02 D0 + table + ~sum.
 */
static void test_node2_push_sends_table(void) {
    const uint8_t rts[8] = {0x10u, 23u, 0u, 4u, 0xFFu, 0x00u, 0xEFu, 0x00u};
    const uint8_t cts[8] = {0x11u, 4u, 1u, 0xFFu, 0xFFu, 0x00u, 0xEFu, 0x00u};
    uint8_t msg[28];
    uint16_t sum = 0u;
    uint32_t idx;

    msg[0] = 0x01u;
    msg[1] = 0x02u;
    msg[2] = 0xD0u;
    memcpy(&msg[3], k_node2_table, sizeof(k_node2_table));
    for (idx = 0u; idx < 3u + NODE2_PUSH_LEN; ++idx) {
        sum = (uint16_t)(sum + msg[idx]);
    }
    sum = (uint16_t)~sum;
    msg[21] = (uint8_t)sum;
    msg[22] = (uint8_t)(sum >> 8);
    memset(&msg[23], 0xFF, 5u);
    vcu_busproto_init();
    (void)tst_usart_drain();
    g_node2_push.state = NODE2_PUSH_PENDING;
    for (idx = 0u; idx < NODE2_SEND_DIV; ++idx) {
        report_telemetry_build();
    }
    TEST_ASSERT_EQUAL_UINT8(NODE2_SEND_DIV, g_node2_push.send_div);
    bp_task_fast_service();
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(0u, g_node2_push.send_div);
    TEST_ASSERT_EQUAL_UINT8(1u, g_node2_push.tries);
    bp_task_fast_service();
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    TEST_ASSERT_EQUAL_HEX32(ID_NODE2_RTS, tst_can_tx_get(0u)->id);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rts, tst_can_tx_get(0u)->data, sizeof(rts));
    (void)tst_can_rx_inject(ID_NODE2_CTS, 1u, cts, sizeof(cts));
    (void)tst_can_rx_deliver();
    bp_task_fast_service();
    TEST_ASSERT_EQUAL_UINT32(5u, prv_can_flush());
    for (idx = 0u; idx < 4u; ++idx) {
        const tst_can_frame_t* frame = tst_can_tx_get(1u + idx);

        TEST_ASSERT_EQUAL_HEX32(ID_NODE2_DT, frame->id);
        TEST_ASSERT_EQUAL_HEX8(idx + 1u, frame->data[0]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(&msg[7u * idx], &frame->data[1], 7u);
    }
}

/**
 * \brief           The send that finds 10 earlier tries ends the push (state 2), so 11 sends at most
 * \note            old = state+2++, `old >= 10` -> VS+0x18 field = 2.
 */
static void test_node2_push_gives_up(void) {
    vcu_busproto_init();
    (void)tst_usart_drain();
    g_node2_push.state = NODE2_PUSH_PENDING;
    g_node2_push.send_div = NODE2_SEND_DIV;
    g_node2_push.tries = NODE2_TRIES - 1u;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(NODE2_TRIES, g_node2_push.tries);
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_PENDING, g_node2_push.state);
    g_node2_push.send_div = NODE2_SEND_DIV;
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(NODE2_TRIES + 1u, g_node2_push.tries);
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_DONE, g_node2_push.state);
    report_telemetry_build();
    TEST_ASSERT_EQUAL_UINT8(0u, g_node2_push.tries);
}

/* ---------------------------------------------------------------------------------------- */
/* UID copy and the scheduler tick                                                           */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           UID copy: 3 words from 0x1FFFF7E8 into the snapshot and IN2 mirror, ~sum
 * \note            Copies 0x1FFFF7E8 x 3 into the snapshot, the
 *                      mirror at -0x44E, and STATE+0x0C = ~(w0 + w1 + w2).
 */
static void test_uid_copy_and_checksum(void) {
    const volatile uint32_t* uid = (const volatile uint32_t*)UID_ADDR;
    uint32_t sum = uid[0] + uid[1] + uid[2];

    report_uid_telemetry_copy();
    TEST_ASSERT_EQUAL_HEX32(uid[0], g_mcu_uid[0]);
    TEST_ASSERT_EQUAL_HEX32(uid[1], g_mcu_uid[1]);
    TEST_ASSERT_EQUAL_HEX32(uid[2], g_mcu_uid[2]);
    TEST_ASSERT_EQUAL_HEX32(uid[0], g_in2.ident_word[0]);
    TEST_ASSERT_EQUAL_HEX32(uid[2], g_in2.ident_word[2]);
    TEST_ASSERT_EQUAL_HEX32(~sum, g_state_block.uid_cksum);
}

/**
 * \brief           TMR5 ISR: counts a tick and clears the flag only when the overflow flag is set
 * \note            Vector IRQ50: tmr_flag_get(TMR5, overflow) -> tmr_flag_clear + ++tick.
 *                      The overflow flag is raised with a software update event (SWEVT).
 */
static void test_tmr5_isr_counts_overflows(void) {
    uint32_t start = g_sched_tick;

    TMR5->ists = 0u;
    TMR5_GLOBAL_IRQHandler();
    TEST_ASSERT_EQUAL_UINT32(start, g_sched_tick);
    TMR5->swevt_bit.ovfswtr = 1u;
    TEST_ASSERT_EQUAL_UINT32(1u, TMR5->ists_bit.ovfif);
    TMR5_GLOBAL_IRQHandler();
    TEST_ASSERT_EQUAL_UINT32(start + 1u, g_sched_tick);
    TEST_ASSERT_EQUAL_UINT32(0u, TMR5->ists_bit.ovfif);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_cells_at_fixed_offsets);
    RUN_TEST(test_u16_report);
    RUN_TEST(test_can_toggle_report);
    RUN_TEST(test_mileage_report_on_change);
    RUN_TEST(test_subsys_mask_report_on_change);
    RUN_TEST(test_subsys_enable_both);
    RUN_TEST(test_pack_bitfields);
    RUN_TEST(test_status_report);
    RUN_TEST(test_slow_report_gates);
    RUN_TEST(test_slow_report_skips_after_oneshot);
    RUN_TEST(test_event_report_single_frame);
    RUN_TEST(test_event_report_tp_tunnel);
    RUN_TEST(test_countdown);
    RUN_TEST(test_datetime_long_idle_threshold);
    RUN_TEST(test_datetime_gates);
    RUN_TEST(test_datetime_year_wrap_reports);
    RUN_TEST(test_datetime_reset_request);
    RUN_TEST(test_telemetry_speed_scaling);
    RUN_TEST(test_telemetry_gauge_cells);
    RUN_TEST(test_telemetry_hold_bit);
    RUN_TEST(test_telemetry_frame_period_and_init);
    RUN_TEST(test_telemetry_frame_fields);
    RUN_TEST(test_node2_push_idle_resets_counters);
    RUN_TEST(test_node2_push_sends_table);
    RUN_TEST(test_node2_push_gives_up);
    RUN_TEST(test_uid_copy_and_checksum);
    RUN_TEST(test_tmr5_isr_counts_overflows);
    tst_exit(UNITY_END());
    return 0;
}
