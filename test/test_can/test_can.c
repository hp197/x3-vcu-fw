/**
 * \file            test_can.c
 * \brief           CAN1 driver (can.c) behaviour
 * \note            Blackboard inputs and outputs are reached at their byte offsets from the
 *                      block bases (g_in2, g_scr, g_stat, g_mode, g_vs, g_candev, g_tmr, the
 *                      telemetry frame), so a renamed or misplaced field shows up. TX frames are
 *                      captured at can_message_transmit with forwarding off (tst_bsp_fakes.h).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "can.h"
#include "report.h"
#include "tst_bsp_fakes.h"
#include "tst_support.h"
#include "vcu_state.h"

/* Expected constants. */
#define RING_USABLE      31u     /* 32 slots, one kept empty */
#define TX_STATUS_NO_BOX 4u      /* "no free mailbox" status of the mailbox write */
#define HS_EVENT_WAIT    500     /* 0x7F1 handshake countdown */
#define SENDER_SUBDIV    50      /* 0x7F0 sender subdivider */
#define SENDER_RETRIG    400u    /* 0x7F0 sender retrigger window */
#define BUSOFF_STREAK    50u     /* re-init when the streak exceeds this */
#define TIMER_RELOAD     200     /* RX decoder tail reload */
#define OUT_ESC_BIT      0x02u   /* candev +0x18 bit1 */
#define OUT_BMS_BIT      0x04u   /* candev +0x18 bit2 */
#define CAN_INTEN_INIT   0x8C02u /* FMP0 | BOF | ETR | EOI */
#define TICK_SPAN        100u
#define TX_LOG_MAX       128u
#define STD_ID_PROBE     0x344u    /* ESC id whose d[0..1] land at IN2 +0x1E8 */
#define MSTS_CUSS        0x100u    /* MSTS bit 8, transmit in progress */
#define TSTS_TCF_ANY     0x010101u /* TSTS tm0tcf | tm1tcf | tm2tcf: a mailbox completed */

/* Byte offsets inside the blackboard blocks. */
#define IN2_GEAR         0xB4u
#define IN2_SPEED        0xAEu
#define IN2_SER_MARK     0x2Eu
#define IN2_REG_2F       0x5Eu
#define IN2_RX344        0x1E8u
#define IN2_TEMP425      0xD6u
#define SCR_SOC          0x50u
#define SCR_SOC_NZ       0x0Cu
#define SCR_DTC0         0x24u
#define STAT_SOC         0x11Eu
#define STAT_CURR        0x118u
#define STAT_VOLT        0x11Au
#define STAT_420A        0x1Eu
#define STAT_TEMPS       0x12Cu
#define STAT_311C        0x1DAu
#define STAT_311B        0x1D8u
#define STAT_311A        0x192u
#define VS_ACC_FWD       0x09u
#define VS_ACC_REV       0x0Au
#define CHG_LATCH        0x05u
#define TMR_RETRIG       0x24u
#define TMR_SUBDIV       0x28u
#define TELEM_100        0x10u
#define TELEM_301        0x12u
#define TELEM_425        0x16u
#define LOW_ERR_CODE     0x05u
#define BAT2_TIMEOUT     200u  /* ticks without a battery-2 frame */
#define BAT2_SOC_LOW     5u    /* SoC <= 5 -> low */
#define BEEP_BAT2_ON     3     /* buzzer_beep(3) when battery 2 appears */
#define BEEP_BAT2_OFF    1     /* buzzer_beep(1) when battery 2 is dropped */
#define TELEM_TX100_CNT  14u   /* u32 counter at history +0x1C (halfword index) */
#define STAT_310_421     0x1A2u
#define STAT_310A        0x10Cu
#define STAT_CURR_LIM    0x122u
#define STAT_310B        0x12Au

static uint32_t g_tx_log[TX_LOG_MAX];
static uint32_t g_tx_logged;
static uint32_t g_tx_odd; /* logged frames that are not standard data frames of DLC 8 */

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
 * \brief           Byte at an offset of a block
 * \param[in]       base: block base
 * \param[in]       off: byte offset
 * \return          the byte
 */
static uint8_t prv_get8(volatile const void* base, uint32_t off) { return ((volatile const uint8_t*)base)[off]; }

/**
 * \brief           Set a byte at an offset of a block
 * \param[in]       base: block base
 * \param[in]       off: byte offset
 * \param[in]       val: value
 */
static void prv_put8(volatile void* base, uint32_t off, uint8_t val) { ((volatile uint8_t*)base)[off] = val; }

/**
 * \brief           CAN TX fake that only logs the identifier (for runs longer than the capture)
 * \param[in]       can_x: unused
 * \param[in]       msg: frame handed to the mailbox
 * \return          mailbox 0
 */
static uint8_t prv_log_tx(can_type* can_x, can_tx_message_type* msg) {
    (void)can_x;
    if (g_tx_logged < TX_LOG_MAX) {
        g_tx_log[g_tx_logged] = (msg->id_type == CAN_ID_EXTENDED) ? msg->extended_id : msg->standard_id;
    }
    if ((msg->id_type != CAN_ID_STANDARD) || (msg->frame_type != CAN_TFT_DATA) || (msg->dlc != 8u)) {
        ++g_tx_odd;
    }
    ++g_tx_logged;
    return 0u;
}

/**
 * \brief           Count logged frames with one identifier
 * \param[in]       can_id: identifier
 * \return          number of frames
 */
static uint32_t prv_logged(uint32_t can_id) {
    uint32_t idx;
    uint32_t hits = 0u;

    for (idx = 0u; (idx < g_tx_logged) && (idx < TX_LOG_MAX); ++idx) {
        if (g_tx_log[idx] == can_id) {
            ++hits;
        }
    }
    return hits;
}

/**
 * \brief           Run the periodic scheduler `ticks` times, pumping the TX ring after each
 * \param[in]       ticks: number of vcu_can_tick calls
 */
static void prv_tick(uint32_t ticks) {
    uint32_t step;

    for (step = 0u; step < ticks; ++step) {
        uint32_t before;

        vcu_can_tick();
        do {
            before = __wrap_can_message_transmit_fake.call_count;
            vcu_can_poll();
        } while (__wrap_can_message_transmit_fake.call_count != before);
    }
}

/**
 * \brief           Enqueue a standard data frame whose id and first byte are `tag`
 * \param[in]       tag: identifier and data[0]
 */
static void prv_enqueue_tag(uint32_t tag) {
    vcu_can_msg_t msg;

    memset(&msg, 0, sizeof(msg));
    msg.std_id = tag;
    msg.dlc = 1u;
    msg.data[0] = (uint8_t)tag;
    vcu_can_tx_enqueue(&msg);
}

/**
 * \brief           Fetch the periodic payload of `can_id` and assert all 8 bytes
 * \param[in]       can_id: standard id
 * \param[in]       expect: the 8 expected bytes
 */
static void prv_assert_payload(uint32_t can_id, const uint8_t* expect) {
    uint8_t payload[8];

    memset(payload, 0xEE, sizeof(payload));
    vcu_can_payload_get(can_id, payload);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, payload, sizeof(payload));
}

/**
 * \brief           Fresh boot state, capture-only CAN TX, empty rings
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    vcu_can_init();
    g_tx_logged = 0u;
    g_tx_odd = 0u;
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* -- TX ring ------------------------------------------------------------------------------- */

/**
 * \brief           The TX ring takes 31 frames; the 32nd is dropped, the queued ones go out in order
 * \note            Enqueue only when `rd != (wr + 1) & 0x1F`, else drop. The pump pops
 *                      from `rd`.
 */
static void test_tx_ring_full_drops_newest(void) {
    uint32_t tag;

    for (tag = 0u; tag <= RING_USABLE; ++tag) {
        prv_enqueue_tag(0x100u + tag);
    }
    for (tag = 0u; tag < TST_CAN_CAPTURE_MAX; ++tag) {
        vcu_can_poll();
    }
    for (tag = 0u; tag < TST_CAN_CAPTURE_MAX; ++tag) {
        TEST_ASSERT_EQUAL_HEX32(0x100u + tag, tst_can_tx_get(tag)->id);
    }
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    for (tag = 0u; tag < TST_CAN_CAPTURE_MAX; ++tag) {
        vcu_can_poll();
    }
    TEST_ASSERT_EQUAL_UINT32(RING_USABLE - TST_CAN_CAPTURE_MAX, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x100u + RING_USABLE - 1u, tst_can_tx_get(RING_USABLE - TST_CAN_CAPTURE_MAX - 1u)->id);
}

/**
 * \brief           Indices wrap at 32: 40 frames through the ring arrive in order, unaltered
 * \note            Enqueue and pump mask both indices with 0x1F.
 */
static void test_tx_ring_wraps(void) {
    uint32_t tag;
    uint8_t ok = 1u;

    for (tag = 0u; tag < 40u; ++tag) {
        const tst_can_frame_t* frame;

        tst_fakes_reset();
        tst_can_tx_forward(0u);
        prv_enqueue_tag(0x200u + tag);
        vcu_can_poll();
        frame = tst_can_tx_get(0u);
        if ((frame == NULL) || (frame->id != 0x200u + tag) || (frame->data[0] != (uint8_t)(0x200u + tag))) {
            ok = 0u;
        }
    }
    TEST_ASSERT_TRUE(ok);
}

/**
 * \brief           One poll hands at most one frame to a mailbox
 * \note            One peek + one mailbox write per call.
 */
static void test_tx_pump_one_frame_per_poll(void) {
    prv_enqueue_tag(0x101u);
    prv_enqueue_tag(0x102u);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    vcu_can_poll();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(2u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x102u, tst_can_tx_get(1u)->id);
}

/**
 * \brief           With no free mailbox the frame stays queued and goes out on a later poll
 * \note            `rd` advances only when the mailbox write returns != 4.
 */
static void test_tx_pump_keeps_frame_without_mailbox(void) {
    __wrap_can_message_transmit_fake.custom_fake = NULL;
    __wrap_can_message_transmit_fake.return_val = TX_STATUS_NO_BOX;
    prv_enqueue_tag(0x123u);
    vcu_can_poll();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(2u, __wrap_can_message_transmit_fake.call_count);
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x123u, tst_can_tx_get(0u)->id);
}

/**
 * \brief           While CAN1 reports a transmit in progress the pump sends nothing
 * \note            The pump returns while MSTS bit 8 (cuss) is set, before the ring is peeked,
 *                      so the frame stays queued. The first frame goes to a real mailbox (CAN1
 *                      online after vcu_can_lowlevel_init); x3emu holds cuss from start of
 *                      frame until it completes (TSTS tmNtcf).
 */
static void test_tx_pump_waits_while_transmitting(void) {
    vcu_can_lowlevel_init();
    tst_can_tx_forward(1u);
    prv_enqueue_tag(0x101u);
    prv_enqueue_tag(0x102u);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_TRUE(tst_wait_set(&CAN1->msts, MSTS_CUSS, TST_WAIT_LOOPS_DEFAULT));
    vcu_can_poll();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_TRUE(tst_wait_set(&CAN1->tsts, TSTS_TCF_ANY, TST_WAIT_LOOPS_DEFAULT));
    TEST_ASSERT_EQUAL_UINT32(0u, CAN1->msts & MSTS_CUSS);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(2u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x102u, tst_can_tx_get(1u)->id);
}

/* -- generic senders ---------------------------------------------------------------------- */

/**
 * \brief           send_std with length 0 queues one empty standard frame
 * \note            len == 0 stores id, dlc = 0, ide = 0, enqueues.
 */
static void test_send_std_empty_frame(void) {
    const tst_can_frame_t* frame;

    vcu_can_send_std(0u, 0x7F0u, NULL);
    vcu_can_poll();
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_NOT_NULL(frame);
    TEST_ASSERT_EQUAL_HEX32(0x7F0u, frame->id);
    TEST_ASSERT_EQUAL_UINT8(0u, frame->ext);
    TEST_ASSERT_EQUAL_UINT8(0u, frame->rtr);
    TEST_ASSERT_EQUAL_UINT8(0u, frame->dlc);
}

/**
 * \brief           send_std splits a long payload into <= 8-byte frames, each from the payload start
 * \note            chunk = min(len - off, 8); the copy always reads from the payload
 *                      pointer itself (the source is never advanced), so a 12-byte payload
 *                      becomes {p0..p7} + {p0..p3}. Every product caller sends 8.
 */
static void test_send_std_fragments_from_payload_start(void) {
    const uint8_t payload[12] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B};

    vcu_can_send_std(sizeof(payload), 0x123u, payload);
    vcu_can_poll();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(2u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_UINT8(8u, tst_can_tx_get(0u)->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, tst_can_tx_get(0u)->data, 8u);
    TEST_ASSERT_EQUAL_UINT8(4u, tst_can_tx_get(1u)->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, tst_can_tx_get(1u)->data, 4u);
    TEST_ASSERT_EQUAL_HEX32(0x123u, tst_can_tx_get(1u)->id);
}

/**
 * \brief           send_ext queues one extended frame of `len` bytes; len > 8 is dropped
 * \note            ide = 1, rtr = 0, dlc = len, enqueued only `if (len < 9)`.
 *                      The id goes on the bus as its low 29 bits (TIxR = ExtId << 3).
 */
static void test_send_ext_single_frame(void) {
    const uint8_t data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    const tst_can_frame_t* frame;

    vcu_can_send_ext(0x18EF2316u, data, 5u);
    vcu_can_send_ext(0x18EF2316u, data, 9u);
    vcu_can_poll();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_EQUAL_UINT8(1u, frame->ext);
    TEST_ASSERT_EQUAL_HEX32(0x18EF2316u, frame->id & VCU_CAN_EXT_ID_MASK);
    TEST_ASSERT_EQUAL_UINT8(5u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, frame->data, 5u);
}

/**
 * \brief           The Ninebot single-frame builder: J1939 id, dlc = n + 3, header + payload
 * \note            id = (prev & 0xA0000000) | ((prio<<26 | dst<<8 | src) &
 *                      0x1FFFFFFF) | 0xEF0000 | 0x20000000; data = {ctl, cmd, arg, payload};
 *                      rejected unless payload/out are set, n < 6 and prio < 8.
 */
static void test_ninebot_frame_builder(void) {
    const uint8_t payload[5] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
    const uint8_t expect[8] = {0x01, 0x04, 0x10, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
    vcu_can_ext_frame_t out;

    memset(&out, 0, sizeof(out));
    out.id = 0x80000000u;
    TEST_ASSERT_EQUAL_INT(1, vcu_can_ninebot_frame(6u, 1u, 0x16u, 0x3Du, 0x04u, 0x10u, payload, 5u, &out));
    TEST_ASSERT_EQUAL_HEX32(0xB8EF3D16u, out.id);
    TEST_ASSERT_EQUAL_UINT8(8u, out.dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, out.data, sizeof(expect));
    out.id = 0u;
    TEST_ASSERT_EQUAL_INT(0, vcu_can_ninebot_frame(6u, 1u, 0x16u, 0x3Du, 4u, 0x10u, payload, 6u, &out));
    TEST_ASSERT_EQUAL_INT(0, vcu_can_ninebot_frame(8u, 1u, 0x16u, 0x3Du, 4u, 0x10u, payload, 1u, &out));
    TEST_ASSERT_EQUAL_INT(0, vcu_can_ninebot_frame(6u, 1u, 0x16u, 0x3Du, 4u, 0x10u, NULL, 1u, &out));
    TEST_ASSERT_EQUAL_INT(0, vcu_can_ninebot_frame(6u, 1u, 0x16u, 0x3Du, 4u, 0x10u, payload, 1u, NULL));
    TEST_ASSERT_EQUAL_HEX32(0u, out.id);
}

/* -- RX ring + dispatch ------------------------------------------------------------------- */

/**
 * \brief           The RX ring keeps 31 frames (32nd dropped); dispatch pops one per call
 * \note            The RX0 ISR pushes (drop when full); vcu_can_rx_dispatch pops one and
 *                      returns the remaining count.
 */
static void test_rx_ring_full_and_dispatch_count(void) {
    uint32_t idx;
    uint32_t pops = 0u;

    for (idx = 0u; idx <= RING_USABLE; ++idx) {
        const uint8_t data[2] = {(uint8_t)idx, 0x5Au};

        (void)tst_can_rx_inject(STD_ID_PROBE, 0u, data, sizeof(data));
        CAN1_RX0_IRQHandler();
    }
    TEST_ASSERT_EQUAL_UINT32(RING_USABLE - 1u, vcu_can_rx_dispatch());
    TEST_ASSERT_EQUAL_HEX16(0x5A00u, prv_get16(&g_in2, IN2_RX344));
    ++pops;
    while (vcu_can_rx_dispatch() != 0u) {
        ++pops;
    }
    ++pops; /* the call that returned 0 still popped the last frame */
    TEST_ASSERT_EQUAL_UINT32(RING_USABLE, pops);
    TEST_ASSERT_EQUAL_HEX16(0x5A00u | (RING_USABLE - 1u), prv_get16(&g_in2, IN2_RX344));
    TEST_ASSERT_EQUAL_UINT32(0u, vcu_can_rx_dispatch());
}

/**
 * \brief           An extended-id frame is not fed to the standard-id decoder
 * \note            Record +0x08 (ide) == 0 -> standard-id decoder, else extended-id handler.
 */
static void test_rx_ext_frame_skips_std_decoder(void) {
    const uint8_t data[2] = {0x34u, 0x12u};

    (void)tst_can_rx_inject(STD_ID_PROBE, 1u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0u, prv_get16(&g_in2, IN2_RX344));
}

/**
 * \brief           0x211: d[6..7] -> IN2 +0xAE, data[3] bit2 sets the charge latch
 * \note            Case 0x211: data[3] bit2 set -> chg +5 = 1.
 */
static void test_rx_211_speed_and_charge_latch(void) {
    const uint8_t data[8] = {0, 0, 0, 0x04, 0, 0, 0xCD, 0xAB};

    (void)tst_can_rx_inject(0x211u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0xABCDu, prv_get16(&g_in2, IN2_SPEED));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_get8(&g_chg, CHG_LATCH));
}

/**
 * \brief           0x211 arms the node 0x02 reg 0xD0 push once; a running or finished push is kept
 * \note            `if ((VS+0x18 & 0x600000) == 0)` the 2-bit field (bits 21..22) becomes 1;
 *                      any other value is left alone.
 */
static void test_rx_211_arms_node2_push(void) {
    const uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0x10, 0x00};

    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_IDLE, g_node2_push.state);
    (void)tst_can_rx_inject(0x211u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_PENDING, g_node2_push.state);
    g_node2_push.state = NODE2_PUSH_DONE;
    (void)tst_can_rx_inject(0x211u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_DONE, g_node2_push.state);
}

/**
 * \brief           0x42: two words stored, no comm-loss counter touched
 * \note            g_can_rx42[0] = rec u32 +0x0B, g_can_rx42[1] = rec u32 +0x0F, then straight
 *                      to the decoder tail.
 */
static void test_rx_42_two_words(void) {
    const uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

    g_lowglob.esc_commloss_cnt = 7u;
    g_bms_commloss.cnt = 9u;
    (void)tst_can_rx_inject(0x42u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX32(0x44332211u, g_can_rx42[0]);
    TEST_ASSERT_EQUAL_HEX32(0x88776655u, g_can_rx42[1]);
    TEST_ASSERT_EQUAL_UINT16(7u, g_lowglob.esc_commloss_cnt);
    TEST_ASSERT_EQUAL_UINT16(9u, g_bms_commloss.cnt);
}

/**
 * \brief           0x310: STAT +0x122/+0x12A/+0x10C; d[4..5] does not reach STAT +0x1A2
 * \note            Three stores (d0, d2, d6); d[4..5] is not stored.
 */
static void test_rx_310_skips_1a2(void) {
    const uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    prv_put16(&g_stat, STAT_310_421, 0xBEEFu);
    (void)tst_can_rx_inject(0x310u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0x0201u, prv_get16(&g_stat, STAT_CURR_LIM));
    TEST_ASSERT_EQUAL_HEX16(0x0403u, prv_get16(&g_stat, STAT_310B));
    TEST_ASSERT_EQUAL_HEX16(0x0807u, prv_get16(&g_stat, STAT_310A));
    TEST_ASSERT_EQUAL_HEX16(0xBEEFu, prv_get16(&g_stat, STAT_310_421));
    TEST_ASSERT_EQUAL_UINT16(0u, g_bms_commloss.cnt);
}

/**
 * \brief           0x420: currents/voltage copied, SoC = s16 d[4..5] / 10, SCR copy without the floor
 * \note            STAT +0x118/+0x11A/+0x11E (sdiv 10)/+0x1E, SCR +0xC = STAT +0x11E, a SoC
 *                      of 0 included.
 */
static void test_rx_420_soc_scaling(void) {
    const uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x9A, 0x03, 0x66, 0x77}; /* d[4..5] = 922 */
    const uint8_t low[8] = {0, 0, 0, 0, 0x09, 0x00, 0, 0};                    /* 9 / 10 = 0 */

    (void)tst_can_rx_inject(0x420u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0x2211u, prv_get16(&g_stat, STAT_CURR));
    TEST_ASSERT_EQUAL_HEX16(0x4433u, prv_get16(&g_stat, STAT_VOLT));
    TEST_ASSERT_EQUAL_UINT16(92u, prv_get16(&g_stat, STAT_SOC));
    TEST_ASSERT_EQUAL_HEX16(0x7766u, prv_get16(&g_stat, STAT_420A));
    TEST_ASSERT_EQUAL_UINT16(92u, prv_get16(&g_scr, SCR_SOC_NZ));
    (void)tst_can_rx_inject(0x420u, 0u, low, sizeof(low));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT16(0u, prv_get16(&g_stat, STAT_SOC));
    TEST_ASSERT_EQUAL_UINT16(0u, prv_get16(&g_scr, SCR_SOC_NZ));
}

/**
 * \brief           0x311: d[0]/d[1] clamped to the VS accel limits, words copied
 * \note            Case 0x311: MODE +0x12 = min(d0, VS+9), +0x13 = min(d1,
 *                      VS+0xA), +0x14 = d2; STAT +0x1DA/+0x1D8/+0x192 = d[0..1]/d[2..3]/d[4..5].
 */
static void test_rx_311_clamped_accel(void) {
    const uint8_t data[8] = {0x90, 0x10, 0x77, 0x01, 0x22, 0x33, 0, 0};

    prv_put8(&g_vs, VS_ACC_FWD, 0x40u);
    prv_put8(&g_vs, VS_ACC_REV, 0x20u);
    (void)tst_can_rx_inject(0x311u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX8(0x40u, g_mode.canrx_12);
    TEST_ASSERT_EQUAL_HEX8(0x10u, g_mode.spd_cmp);
    TEST_ASSERT_EQUAL_HEX8(0x77u, g_mode.canrx_14);
    TEST_ASSERT_EQUAL_HEX16(0x1090u, prv_get16(&g_stat, STAT_311C));
    TEST_ASSERT_EQUAL_HEX16(0x0177u, prv_get16(&g_stat, STAT_311B));
    TEST_ASSERT_EQUAL_HEX16(0x3322u, prv_get16(&g_stat, STAT_311A));
}

/**
 * \brief           0x500: eight cell temperatures, each byte minus 0x32; 0x501/0x502 are BMS keep-alives
 * \note            0x500 carries the temperatures, 0x501/0x502 zero the BMS count. 0x4DF is
 *                      not decoded: no store and no count change.
 */
static void test_rx_500_temperatures(void) {
    const uint8_t data[8] = {0x32, 0x33, 0x50, 0x00, 0xFF, 0x40, 0x31, 0x64};
    uint8_t idx;

    g_bms_commloss.cnt = 9u;
    (void)tst_can_rx_inject(0x4DFu, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0u, prv_get16(&g_stat, STAT_TEMPS));
    TEST_ASSERT_EQUAL_UINT16(9u, g_bms_commloss.cnt);
    (void)tst_can_rx_inject(0x500u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    for (idx = 0u; idx < 8u; ++idx) {
        TEST_ASSERT_EQUAL_HEX16((uint16_t)((int)data[idx] - 0x32), prv_get16(&g_stat, STAT_TEMPS + 2u * idx));
    }
    TEST_ASSERT_EQUAL_UINT16(0u, g_bms_commloss.cnt);
    g_bms_commloss.cnt = 9u;
    (void)tst_can_rx_inject(0x501u, 0u, NULL, 0u);
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT16(0u, g_bms_commloss.cnt);
    g_bms_commloss.cnt = 9u;
    (void)tst_can_rx_inject(0x502u, 0u, NULL, 0u);
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT16(0u, g_bms_commloss.cnt);
}

/**
 * \brief           0x425: received flag, temperature (d2 - 0x32) * 10 + telemetry mirror; no watchdog re-arm
 * \note            IN1 +0x02 = 1, IN2 +0xD6, TELEM +0x16 = d2 | d5 << 8, then straight to the
 *                      decoder tail without zeroing a comm-loss count.
 */
static void test_rx_425_temperature_no_watchdog(void) {
    const uint8_t data[8] = {0, 0, 0x3C, 0, 0, 0x99, 0, 0};

    g_lowglob.esc_commloss_cnt = 7u;
    g_bms_commloss.cnt = 9u;
    (void)tst_can_rx_inject(0x425u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT16(100u, prv_get16(&g_in2, IN2_TEMP425));
    TEST_ASSERT_EQUAL_HEX16(0x993Cu, prv_get16(g_telem_frame, TELEM_425));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.bms_temp_rx);
    TEST_ASSERT_EQUAL_UINT16(7u, g_lowglob.esc_commloss_cnt);
    TEST_ASSERT_EQUAL_UINT16(9u, g_bms_commloss.cnt);
}

/**
 * \brief           An ESC frame zeroes the ESC comm-loss count, reloads timer A, clears out bit1 only
 * \note            Case 0x342 (ESC comm-loss count = 0) + the decoder tail: each group whose
 *                      count is 0 reloads its timer (+0x12 / +0x14) to 200 and clears its out bit
 *                      (+0x18 bit1 / bit2). The BMS frame is the keep-alive 0x502.
 */
static void test_rx_watchdog_rearm(void) {
    g_lowglob.esc_commloss_cnt = 7u;
    g_bms_commloss.cnt = 9u;
    g_candev.timer_a = 5;
    g_candev.timer_b = 5;
    g_candev.out_bits = OUT_ESC_BIT | OUT_BMS_BIT;
    (void)tst_can_rx_inject(0x342u, 0u, NULL, 0u);
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.esc_commloss_cnt);
    TEST_ASSERT_EQUAL_INT16(TIMER_RELOAD, prv_get16(&g_candev, 0x12u));
    TEST_ASSERT_EQUAL_INT16(5, prv_get16(&g_candev, 0x14u));
    TEST_ASSERT_EQUAL_HEX8(OUT_BMS_BIT, prv_get8(&g_candev, 0x18u));
    (void)tst_can_rx_inject(0x502u, 0u, NULL, 0u);
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_INT16(TIMER_RELOAD, prv_get16(&g_candev, 0x14u));
    TEST_ASSERT_EQUAL_HEX8(0u, prv_get8(&g_candev, 0x18u));
}

/* -- battery 2 ----------------------------------------------------------------------------- */

/**
 * \brief           0x430: battery-2 currents/voltage/SoC, present flag, battery-2 count only
 * \note            B2 (g_stat2) +0x118/+0x11A/+0x11E (sdiv 10)/+0x1E, then the battery-2 tail:
 *                      SCR +0x0A = 1, g_bat2.commloss_cnt = 0. No SCR +0x0C copy and the ESC/BMS
 *                      counts are not touched.
 */
static void test_rx_430_battery2_soc(void) {
    const uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x9A, 0x03, 0x66, 0x77};

    g_lowglob.esc_commloss_cnt = 7u;
    g_bms_commloss.cnt = 9u;
    g_bat2.commloss_cnt = 150u;
    (void)tst_can_rx_inject(0x430u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0x2211u, prv_get16(&g_stat2, STAT_CURR));
    TEST_ASSERT_EQUAL_HEX16(0x4433u, prv_get16(&g_stat2, STAT_VOLT));
    TEST_ASSERT_EQUAL_UINT16(92u, prv_get16(&g_stat2, STAT_SOC));
    TEST_ASSERT_EQUAL_HEX16(0x7766u, prv_get16(&g_stat2, STAT_420A));
    TEST_ASSERT_EQUAL_UINT16(0u, prv_get16(&g_stat, STAT_SOC));
    TEST_ASSERT_EQUAL_UINT16(0u, prv_get16(&g_scr, SCR_SOC_NZ));
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.bat2_present);
    TEST_ASSERT_EQUAL_UINT16(0u, g_bat2.commloss_cnt);
    TEST_ASSERT_EQUAL_UINT16(7u, g_lowglob.esc_commloss_cnt);
    TEST_ASSERT_EQUAL_UINT16(9u, g_bms_commloss.cnt);
}

/**
 * \brief           0x431: battery-2 ASCII-hex version + two words, version mirrored to IN2 +0x34
 * \note            Four can_rx_hex_nibble calls (d3, d2, d1, d0) ->
 *                      B2 +0x1C, B2 +0x26 = d[4..5], B2 +0x1A2 = d[6..7], IN2 +0x34 = B2 +0x1C.
 */
static void test_rx_431_battery2_version(void) {
    const uint8_t data[8] = {'7', '0', 'a', 'F', 0x21, 0x43, 0x65, 0x87};

    (void)tst_can_rx_inject(0x431u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0xFA07u, prv_get16(&g_stat2, 0x1Cu));
    TEST_ASSERT_EQUAL_HEX16(0x4321u, prv_get16(&g_stat2, 0x26u));
    TEST_ASSERT_EQUAL_HEX16(0x8765u, prv_get16(&g_stat2, STAT_310_421));
    TEST_ASSERT_EQUAL_HEX16(0xFA07u, prv_get16(&g_in2, 0x34u));
    TEST_ASSERT_EQUAL_HEX16(0u, prv_get16(&g_in2, 0x32u));
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.bat2_present);
}

/**
 * \brief           0x432: battery-2 block C words; 0x435: battery-2 temperature d2 - 0x32
 * \note            0x432: B2 +0xB2/+0x1E8/+0x124/+0x126; 0x435: B2 +0x12C = d2 - 0x32; both
 *                      through the battery-2 tail.
 */
static void test_rx_432_435_battery2(void) {
    const uint8_t blk[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    const uint8_t temp[8] = {0, 0, 0x50, 0, 0, 0, 0, 0};

    g_bat2.commloss_cnt = 50u;
    (void)tst_can_rx_inject(0x432u, 0u, blk, sizeof(blk));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0x0201u, prv_get16(&g_stat2, 0xB2u));
    TEST_ASSERT_EQUAL_HEX16(0x0403u, prv_get16(&g_stat2, 0x1E8u));
    TEST_ASSERT_EQUAL_HEX16(0x0605u, prv_get16(&g_stat2, 0x124u));
    TEST_ASSERT_EQUAL_HEX16(0x0807u, prv_get16(&g_stat2, 0x126u));
    TEST_ASSERT_EQUAL_UINT16(0u, g_bat2.commloss_cnt);
    g_bat2.commloss_cnt = 50u;
    g_scr.bat2_present = 0u;
    (void)tst_can_rx_inject(0x435u, 0u, temp, sizeof(temp));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0x1Eu, prv_get16(&g_stat2, STAT_TEMPS));
    TEST_ASSERT_EQUAL_UINT16(0u, g_bat2.commloss_cnt);
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.bat2_present);
}

/**
 * \brief           Presence tick: SoC copied, low flag at <= 5 %, SF_DUAL_PACK raised once with beep 3 in RIDE
 * \note            SCR+0x0A != 0 -> SCR+0x0E = B2+0x11E; g_bat2.soc_low = (SCR+0x0E <= 5); if
 *                      IN2+0x38 bit9 clear: set it and, when the FSM state is 3, buzzer_beep(3).
 */
static void test_bat2_presence_raises_dual_pack(void) {
    g_scr.bat2_present = 1u;
    g_stat2.soc = 40;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT16(40u, g_scr.bat2_soc);
    TEST_ASSERT_EQUAL_UINT8(0u, g_bat2.soc_low);
    TEST_ASSERT_EQUAL_HEX16(SF_DUAL_PACK, g_in2.f.status_flags & SF_DUAL_PACK);
    TEST_ASSERT_EQUAL_UINT8(BEEP_BAT2_ON, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(1u, g_bat2.commloss_cnt);
    g_lowglob.beep_seq_id = 0u;
    g_stat2.soc = (int16_t)BAT2_SOC_LOW;
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT8(1u, g_bat2.soc_low);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.beep_seq_id);
    g_stat2.soc = (int16_t)(BAT2_SOC_LOW + 1u);
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_bat2.soc_low);
}

/**
 * \brief           Presence tick outside RIDE: no beep; nothing at all while not present
 * \note            The beep only when the FSM state is 3; no copy while SCR+0x0A == 0, and
 *                      with IN2+0x38 bit9 clear the count is not advanced.
 */
static void test_bat2_presence_quiet_cases(void) {
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_bat2.commloss_cnt);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.status_flags & SF_DUAL_PACK);
    g_scr.bat2_present = 1u;
    g_stat2.soc = 50;
    g_state_block.fsm_state = VCU_FSM_CHARGING;
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_HEX16(SF_DUAL_PACK, g_in2.f.status_flags & SF_DUAL_PACK);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.beep_seq_id);
}

/**
 * \brief           Presence timeout: 200 ticks without a frame drop the pack (beep 1 in RIDE)
 * \note            count = old + 1; when old >= 200 the count is held at 200 and, present with
 *                      SoC not low: SCR+0x0A = 0, IN2+0x38 bit9 cleared, buzzer_beep(1) in state 3.
 *                      A battery-2 frame zeroes the count.
 */
static void test_bat2_presence_timeout(void) {
    uint32_t step;
    const uint8_t data[8] = {0, 0, 0, 0, 0x90, 0x01, 0, 0}; /* SoC 40 */

    (void)tst_can_rx_inject(0x430u, 0u, data, sizeof(data));
    (void)tst_can_rx_deliver();
    g_state_block.fsm_state = VCU_FSM_RIDE;
    for (step = 0u; step < BAT2_TIMEOUT; ++step) {
        vcu_can_battery2_presence_tick();
    }
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.bat2_present);
    TEST_ASSERT_EQUAL_UINT16(BAT2_TIMEOUT, g_bat2.commloss_cnt);
    g_lowglob.beep_seq_id = 0u;
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.bat2_present);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.status_flags & SF_DUAL_PACK);
    TEST_ASSERT_EQUAL_UINT8(BEEP_BAT2_OFF, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT16(BAT2_TIMEOUT, g_bat2.commloss_cnt);
}

/**
 * \brief           Presence timeout with a low SoC: only state 4 drops the pack, the flag goes a tick later
 * \note            Low flag set -> `if (state == 4) SCR+0x0A = 0`, IN2+0x38 bit9 kept; the next
 *                      expiry finds SCR+0x0A == 0 and clears bit9 without a beep.
 */
static void test_bat2_presence_timeout_low_soc(void) {
    g_scr.bat2_present = 1u;
    g_stat2.soc = 3;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    vcu_can_battery2_presence_tick();
    g_bat2.commloss_cnt = BAT2_TIMEOUT;
    g_lowglob.beep_seq_id = 0u;
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT16(1u, g_scr.bat2_present);
    TEST_ASSERT_EQUAL_HEX16(SF_DUAL_PACK, g_in2.f.status_flags & SF_DUAL_PACK);
    g_state_block.fsm_state = VCU_FSM_POWER_DOWN;
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.bat2_present);
    TEST_ASSERT_EQUAL_HEX16(SF_DUAL_PACK, g_in2.f.status_flags & SF_DUAL_PACK);
    vcu_can_battery2_presence_tick();
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.status_flags & SF_DUAL_PACK);
    TEST_ASSERT_EQUAL_UINT8(0u, g_lowglob.beep_seq_id);
}

/* -- periodic scheduler ------------------------------------------------------------------- */

/**
 * \brief           100 ticks emit 0x100 x50, 0x203/0x20C x10, 0x485 x5, 0x20A/0x482/0x301 x1, all DLC 8
 * \note            tick++ then 0x100 when even, 0x203+0x20C when %10, 0x485 when %20,
 *                      0x20A+0x482+0x301 when %100 (in that order), each
 *                      vcu_can_send_std(8, id, payload).
 */
static void test_tick_cadence(void) {
    static const uint32_t order[7] = {0x100u, 0x203u, 0x20Cu, 0x485u, 0x20Au, 0x482u, 0x301u};
    uint32_t idx;

    __wrap_can_message_transmit_fake.custom_fake = prv_log_tx;
    prv_tick(TICK_SPAN);
    TEST_ASSERT_EQUAL_UINT32(50u, prv_logged(0x100u));
    TEST_ASSERT_EQUAL_UINT32(10u, prv_logged(0x203u));
    TEST_ASSERT_EQUAL_UINT32(10u, prv_logged(0x20Cu));
    TEST_ASSERT_EQUAL_UINT32(5u, prv_logged(0x485u));
    TEST_ASSERT_EQUAL_UINT32(1u, prv_logged(0x20Au));
    TEST_ASSERT_EQUAL_UINT32(1u, prv_logged(0x482u));
    TEST_ASSERT_EQUAL_UINT32(1u, prv_logged(0x301u));
    TEST_ASSERT_EQUAL_UINT32(78u, g_tx_logged);
    for (idx = 0u; idx < 7u; ++idx) {
        TEST_ASSERT_EQUAL_HEX32(order[idx], g_tx_log[g_tx_logged - 7u + idx]);
    }
    TEST_ASSERT_EQUAL_UINT32(0u, g_tx_odd);
}

/**
 * \brief           The two countdowns decrement per tick and raise out bit1 / bit2 at zero
 * \note            `if (+0x12 == 0) +0x18 |= 2; else +0x12--;` (same for
 *                      +0x14 / bit2).
 */
static void test_tick_timers_raise_out_bits(void) {
    g_candev.timer_a = 1;
    g_candev.timer_b = 2;
    vcu_can_tick();
    TEST_ASSERT_EQUAL_HEX8(0u, g_candev.out_bits);
    vcu_can_tick();
    TEST_ASSERT_EQUAL_HEX8(OUT_ESC_BIT, g_candev.out_bits);
    vcu_can_tick();
    TEST_ASSERT_EQUAL_HEX8(OUT_ESC_BIT | OUT_BMS_BIT, g_candev.out_bits);
}

/**
 * \brief           While IN1 drive-active is 1 the tick only zeroes the comm-loss/UV counters
 * \note            `if (IN1+4 == 1)` the ESC/BMS comm-loss counts and the UV count are zeroed
 *                      and nothing else happens (no tick, no frames, no timers).
 */
static void test_tick_suspended_while_drive_active(void) {
    g_in1.drive_active = 1u;
    g_lowglob.esc_commloss_cnt = 3u;
    g_bms_commloss.cnt = 4u;
    g_lowglob.fm_uv_cnt = 5u;
    g_candev.timer_a = 10;
    prv_tick(4u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.esc_commloss_cnt);
    TEST_ASSERT_EQUAL_UINT16(0u, g_bms_commloss.cnt);
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.fm_uv_cnt);
    TEST_ASSERT_EQUAL_INT16(10, g_candev.timer_a);
    TEST_ASSERT_EQUAL_UINT16(0u, g_can_sched.tick);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
}

/**
 * \brief           A 0x7F1 handshake with d0 bit0 pauses the periodic TX for 500 ticks, then restarts the phase
 * \note            Case 0x7F1: candev +0xE = 500, +0xC bit0 = d0 bit0. vcu_can_tick: while
 *                      +0xC bit0 and +0xE != 0, decrement +0xE and return (no timers, no
 *                      frames); at 0, tick (S-6) = 0, bit0 cleared, then the normal tick.
 */
static void test_rx_7f1_pauses_periodic_tx(void) {
    const uint8_t start[1] = {0x01u};
    const uint8_t stop[1] = {0x00u};

    g_can_sched.tick = 7u;
    (void)tst_can_rx_inject(0x7F1u, 0u, start, sizeof(start));
    (void)tst_can_rx_deliver();
    g_candev.timer_a = 50; /* the RX tail reloaded it to 200 */
    TEST_ASSERT_EQUAL_INT16(HS_EVENT_WAIT, g_candev.event_wait);
    TEST_ASSERT_EQUAL_HEX16(1u, g_candev.event_flags & 1u);
    prv_tick((uint32_t)HS_EVENT_WAIT);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_INT16(50, g_candev.timer_a);
    prv_tick(1u);
    TEST_ASSERT_EQUAL_UINT16(1u, g_can_sched.tick);
    TEST_ASSERT_EQUAL_HEX16(0u, g_candev.event_flags & 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
    prv_tick(1u);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x100u, tst_can_tx_get(0u)->id);
    g_candev.event_flags = 1u;
    (void)tst_can_rx_inject(0x7F1u, 0u, stop, sizeof(stop));
    (void)tst_can_rx_deliver();
    TEST_ASSERT_EQUAL_HEX16(0u, g_candev.event_flags & 1u);
}

/**
 * \brief           A bus-off streak of more than 0x32 ticks re-initialises CAN1, keeping the TX ring
 * \note            bus_off (+0xB) set -> S-4 counts; `0x32 < count` -> +0xB = 0, S-4 = 0,
 *                      +8 ++, vcu_can_mode_reset (can_reset, GPIO, hw init, then
 *                      +0x19/+0x1A/+9/+8 = 0, so the re-init count ends at 0 again).
 *                      vcu_can_mode_reset does not touch the software rings.
 */
static void test_busoff_streak_reinits_can(void) {
    uint32_t step;

    prv_enqueue_tag(0x0ABu);
    g_candev.bus_off = 1u;
    g_candev.tec = 0x11u;
    g_candev.rec = 0x22u;
    g_candev.lec = 3u;
    CAN1->inten = 0u;
    for (step = 0u; step < BUSOFF_STREAK; ++step) {
        vcu_can_tick();
    }
    TEST_ASSERT_EQUAL_UINT16(BUSOFF_STREAK, g_can_sched.fault_ctr);
    TEST_ASSERT_EQUAL_UINT8(1u, g_candev.bus_off);
    TEST_ASSERT_EQUAL_HEX32(0u, CAN1->inten);
    vcu_can_tick();
    TEST_ASSERT_EQUAL_UINT8(0u, g_candev.bus_off);
    TEST_ASSERT_EQUAL_UINT16(0u, g_can_sched.fault_ctr);
    TEST_ASSERT_EQUAL_UINT8(0u, g_candev.reinit_cnt);
    TEST_ASSERT_EQUAL_UINT8(0u, g_candev.tec);
    TEST_ASSERT_EQUAL_UINT8(0u, g_candev.rec);
    TEST_ASSERT_EQUAL_UINT8(0u, g_candev.lec);
    TEST_ASSERT_EQUAL_HEX32(CAN_INTEN_INIT, CAN1->inten);
    vcu_can_poll(); /* the frame queued before the re-init is still first in the ring */
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x0ABu, tst_can_tx_get(0u)->id);
}

/**
 * \brief           The SE ISR latches bus-off and snapshots TEC/REC/LEC when an error is recorded
 * \note            ESR bit2 (BOF) -> candev +0xB = 1; flag 4 (LEC
 *                      != 0) -> +0x19 = ESR >> 16, +0x1A = ESR >> 24, +9 = ESR bits 4..6, then ESR
 *                      cleared; without an error record ESR is left as it is.
 */
static void test_se_isr_busoff_and_error_snapshot(void) {
    CAN1->ests = (0x34u << 24) | (0x12u << 16) | (3u << 4) | (1u << 2);
    CAN1_SE_IRQHandler();
    TEST_ASSERT_EQUAL_UINT8(1u, g_candev.bus_off);
    TEST_ASSERT_EQUAL_HEX8(0x12u, g_candev.tec);
    TEST_ASSERT_EQUAL_HEX8(0x34u, g_candev.rec);
    TEST_ASSERT_EQUAL_UINT8(3u, g_candev.lec);
    TEST_ASSERT_EQUAL_HEX32(0u, CAN1->ests); /* the error record is cleared after the snapshot */
    g_candev.tec = 0xAAu;
    CAN1->ests = (0x56u << 16);
    CAN1_SE_IRQHandler();
    TEST_ASSERT_EQUAL_HEX8(0xAAu, g_candev.tec);
    TEST_ASSERT_EQUAL_HEX32(0x56u << 16, CAN1->ests); /* no error record: ests is not written */
    CAN1->ests = 0u;
}

/* -- periodic payloads -------------------------------------------------------------------- */

/**
 * \brief           0x100 = drive output, mode flags, current, energy word; TELEM +0x10 mirror; frame counter
 * \note            The four staging words, then the u32 at the history block +0x1C counts up
 *                      unless it is 0xFFFFFFFF, then TELEM +0x10 = payload[0..1]. The staging
 *                      cells are reached by name.
 */
static void test_payload_100(void) {
    const uint8_t expect[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t payload[8];

    g_mode.drive_output_word = 0x0201u;
    g_mode.mode_flags = 0x0403u;
    g_mode.curr_word = 0x0605u;
    g_mode.we_word = 0x0807u;
    prv_assert_payload(0x100u, expect);
    TEST_ASSERT_EQUAL_HEX16(0x0201u, prv_get16(g_telem_frame, TELEM_100));
    TEST_ASSERT_EQUAL_HEX32(1u, g_telem_frame[TELEM_TX100_CNT] | ((uint32_t)g_telem_frame[TELEM_TX100_CNT + 1u] << 16));
    vcu_can_payload_get(0x100u, payload);
    TEST_ASSERT_EQUAL_HEX16(2u, g_telem_frame[TELEM_TX100_CNT]);
    g_telem_frame[TELEM_TX100_CNT] = 0xFFFEu;
    g_telem_frame[TELEM_TX100_CNT + 1u] = 0xFFFFu;
    vcu_can_payload_get(0x100u, payload);
    vcu_can_payload_get(0x100u, payload);
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, g_telem_frame[TELEM_TX100_CNT]);
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, g_telem_frame[TELEM_TX100_CNT + 1u]);
    vcu_can_payload_get(0x203u, payload);
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, g_telem_frame[TELEM_TX100_CNT]);
}

/**
 * \brief           0x203 = IN2 +0xB4 low byte, 0, SCR +0x50, STAT +0x11E, IN2 +0xAE
 * \note            Only the low byte of IN2+0xB4 is sent, byte 1 = 0.
 */
static void test_payload_203(void) {
    const uint8_t expect[8] = {0x34, 0x00, 0x78, 0x56, 0xBC, 0x9A, 0xF0, 0xDE};

    prv_put16(&g_in2, IN2_GEAR, 0x1234u);
    prv_put16(&g_scr, SCR_SOC, 0x5678u);
    prv_put16(&g_stat, STAT_SOC, 0x9ABCu);
    prv_put16(&g_in2, IN2_SPEED, 0xDEF0u);
    prv_assert_payload(0x203u, expect);
}

/**
 * \brief           0x20A = SCR +0x24/+0x26/+0x28/+0x2A
 */
static void test_payload_20a(void) {
    const uint8_t expect[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

    prv_put16(&g_scr, SCR_DTC0, 0x2211u);
    prv_put16(&g_scr, SCR_DTC0 + 2u, 0x4433u);
    prv_put16(&g_scr, SCR_DTC0 + 4u, 0x6655u);
    prv_put16(&g_scr, SCR_DTC0 + 6u, 0x8877u);
    prv_assert_payload(0x20Au, expect);
}

/**
 * \brief           0x20C = enable flags (u16), cmd lead, 0x311 bytes, derate flags, 0; MODE +1 = error code
 * \note            MODE+1 = the active error code first, then the bytes; byte 6 is the derate
 *                      byte at MODE+0x19 and byte 7 is always 0 (independent of g_vs).
 */
static void test_payload_20c(void) {
    const uint8_t expect[8] = {0xB0, 0xB1, 0xC0, 0xC2, 0xC3, 0xC4, 0xC7, 0x00};

    g_mode.enable_flags = 0xB1B0u;
    g_mode.cmd_lead = 0xC0u;
    g_mode.canrx_12 = 0xC2u;
    g_mode.spd_cmp = 0xC3u;
    g_mode.canrx_14 = 0xC4u;
    g_mode.cfg_17 = 0xC7u;
    g_vs.region = 0x02u;
    prv_put8(&g_lowglob, LOW_ERR_CODE, 0x2Du);
    prv_assert_payload(0x20Cu, expect);
    TEST_ASSERT_EQUAL_HEX8(0x2Du, g_mode.error_code);
}

/**
 * \brief           0x301 = TEC, REC, LEC, re-init count, then zeros; TEC|REC<<8 into TELEM +0x12
 * \note            candev +0x19, +0x1A, +0x09, +0x08.
 */
static void test_payload_301(void) {
    const uint8_t expect[8] = {0x21, 0x43, 0x05, 0x02, 0, 0, 0, 0};

    prv_put8(&g_candev, 0x19u, 0x21u);
    prv_put8(&g_candev, 0x1Au, 0x43u);
    prv_put8(&g_candev, 0x09u, 0x05u);
    prv_put8(&g_candev, 0x08u, 0x02u);
    prv_assert_payload(0x301u, expect);
    TEST_ASSERT_EQUAL_HEX16(0x4321u, prv_get16(g_telem_frame, TELEM_301));
}

/**
 * \brief           0x482 = IN2 +0x2E, IN2 +0x5E, zeros
 * \note            IN2+0x2E, then IN2+0x5E (+0x30 from there).
 */
static void test_payload_482(void) {
    const uint8_t expect[8] = {0x2E, 0x01, 0x5E, 0x02, 0, 0, 0, 0};

    prv_put16(&g_in2, IN2_SER_MARK, 0x012Eu);
    prv_put16(&g_in2, IN2_REG_2F, 0x025Eu);
    prv_assert_payload(0x482u, expect);
}

/**
 * \brief           0x485 = word06 (u16), output_cfg, cfg_16, word04 (u16), cmd_param, regen level
 * \note            payload[0..1] = staging +0x06, [2..3] = +0x17 (output_cfg, cfg_16),
 *                      [4..5] = +0x04, [6] = +0x02, [7] = +0x0E, the regen-strength byte. The
 *                      staging cells are reached by name.
 */
static void test_payload_485(void) {
    const uint8_t expect[8] = {0x61, 0x62, 0x15, 0x16, 0x41, 0x42, 0xC8, 0x2D};

    g_mode.word06 = 0x6261u;
    g_mode.output_cfg = 0x15u;
    g_mode.cfg_16 = 0x16u;
    g_mode.word04 = 0x4241u;
    g_mode.cmd_param = 0xC8u;
    g_mode.regen_level = 45u;
    prv_assert_payload(0x485u, expect);
}

/**
 * \brief           0x7F0 = {0x1E, 0, 0, 0, 0, 0, 0, 0}
 */
static void test_payload_7f0(void) {
    const uint8_t expect[8] = {0x1E, 0, 0, 0, 0, 0, 0, 0};

    prv_assert_payload(0x7F0u, expect);
}

/* -- 0x7F0 sender + init ------------------------------------------------------------------ */

/**
 * \brief           An app request opens the 400-tick window, sends 0x7F0 every 0x33 calls and pauses the tick
 * \note            IN1+3 set -> TMR+0x24 = 400, IN1+3 = 0, IN1+4 = 1; while IN1+4:
 *                      VS+0x1D = 1; TMR+0x28 < 1 -> TMR+0x28 = 0x32, candev +0xE = 0x32,
 *                      +0xC |= 1, vcu_can_send_std(8, 0x7F0, getter(0x7F0)); else TMR+0x28--.
 */
static void test_senders_0x7f0_window(void) {
    uint32_t step;

    g_in1.can_active_req = 1u;
    TEST_ASSERT_EQUAL_UINT8(1u, vcu_can_senders());
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_HEX32(0x7F0u, tst_can_tx_get(0u)->id);
    TEST_ASSERT_EQUAL_UINT8(8u, tst_can_tx_get(0u)->dlc);
    TEST_ASSERT_EQUAL_HEX8(0x1Eu, tst_can_tx_get(0u)->data[0]);
    TEST_ASSERT_EQUAL_UINT32(SENDER_RETRIG, prv_get16(&g_tmr, TMR_RETRIG));
    TEST_ASSERT_EQUAL_UINT16(SENDER_SUBDIV, prv_get16(&g_tmr, TMR_SUBDIV));
    TEST_ASSERT_EQUAL_INT16(SENDER_SUBDIV, g_candev.event_wait);
    TEST_ASSERT_EQUAL_HEX16(1u, g_candev.event_flags & 1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.can_drive_active);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.can_active_req);
    for (step = 0u; step < (uint32_t)SENDER_SUBDIV; ++step) {
        (void)vcu_can_senders();
        vcu_can_poll();
    }
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    (void)vcu_can_senders();
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(2u, tst_can_tx_count());
}

/**
 * \brief           The window closes when the retrigger count runs out
 * \note            IN1+3 == 0 and TMR+0x24 < 1 -> IN1+4 = 0.
 */
static void test_senders_window_expires(void) {
    g_in1.can_active_req = 1u;
    (void)vcu_can_senders();
    prv_put16(&g_tmr, TMR_RETRIG, 1u);
    TEST_ASSERT_EQUAL_UINT8(1u, vcu_can_senders());
    TEST_ASSERT_EQUAL_UINT8(0u, vcu_can_senders());
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.drive_active);
}

/**
 * \brief           Boot bring-up: CAN1 on PB8/PB9, 500 kbit timing, accept-all, IRQs, PB15 low (smoke)
 * \note            CAN GPIO init + CAN hardware init + gpio_bits_reset(GPIOB, 0x8000); baud
 *                      struct {9,1,10,3}; four interrupt enables (IER 0x8C02).
 */
static void test_lowlevel_init_smoke(void) {
    GPIOB->scr = GPIO_PINS_15;
    vcu_can_lowlevel_init();
    TEST_ASSERT_EQUAL_HEX32(0u, GPIOB->odt & GPIO_PINS_15);
    TEST_ASSERT_EQUAL_UINT32(8u, CAN1->btmg_bit.brdiv);
    TEST_ASSERT_EQUAL_UINT32(10u, CAN1->btmg_bit.bts1);
    TEST_ASSERT_EQUAL_UINT32(3u, CAN1->btmg_bit.bts2);
    TEST_ASSERT_EQUAL_UINT32(1u, CAN1->btmg_bit.rsaw);
    TEST_ASSERT_EQUAL_HEX32(CAN_INTEN_INIT, CAN1->inten);
}

/**
 * \brief           Controller base config: auto wake-up on, automatic retransmission on (NART off)
 * \note            Base struct bytes [0..6] = {0, 0, 0, 1, 0, 0, 0}, so aeden = 1 and prsfen
 *                      (no retransmission) = 0.
 */
static void test_hw_init_retransmission_enabled(void) {
    vcu_can_lowlevel_init();
    TEST_ASSERT_EQUAL_UINT32(0u, CAN1->mctrl_bit.prsfen);
    TEST_ASSERT_EQUAL_UINT32(1u, CAN1->mctrl_bit.aeden);
    TEST_ASSERT_EQUAL_UINT32(0u, CAN1->mctrl_bit.ttcen);
    TEST_ASSERT_EQUAL_UINT32(0u, CAN1->mctrl_bit.aeboen);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_tx_ring_full_drops_newest);
    RUN_TEST(test_tx_ring_wraps);
    RUN_TEST(test_tx_pump_one_frame_per_poll);
    RUN_TEST(test_tx_pump_keeps_frame_without_mailbox);
    RUN_TEST(test_tx_pump_waits_while_transmitting);
    RUN_TEST(test_send_std_empty_frame);
    RUN_TEST(test_send_std_fragments_from_payload_start);
    RUN_TEST(test_send_ext_single_frame);
    RUN_TEST(test_ninebot_frame_builder);
    RUN_TEST(test_rx_ring_full_and_dispatch_count);
    RUN_TEST(test_rx_ext_frame_skips_std_decoder);
    RUN_TEST(test_rx_211_speed_and_charge_latch);
    RUN_TEST(test_rx_211_arms_node2_push);
    RUN_TEST(test_rx_42_two_words);
    RUN_TEST(test_rx_310_skips_1a2);
    RUN_TEST(test_rx_420_soc_scaling);
    RUN_TEST(test_rx_311_clamped_accel);
    RUN_TEST(test_rx_500_temperatures);
    RUN_TEST(test_rx_425_temperature_no_watchdog);
    RUN_TEST(test_rx_watchdog_rearm);
    RUN_TEST(test_rx_430_battery2_soc);
    RUN_TEST(test_rx_431_battery2_version);
    RUN_TEST(test_rx_432_435_battery2);
    RUN_TEST(test_bat2_presence_raises_dual_pack);
    RUN_TEST(test_bat2_presence_quiet_cases);
    RUN_TEST(test_bat2_presence_timeout);
    RUN_TEST(test_bat2_presence_timeout_low_soc);
    RUN_TEST(test_tick_cadence);
    RUN_TEST(test_tick_timers_raise_out_bits);
    RUN_TEST(test_tick_suspended_while_drive_active);
    RUN_TEST(test_rx_7f1_pauses_periodic_tx);
    RUN_TEST(test_busoff_streak_reinits_can);
    RUN_TEST(test_se_isr_busoff_and_error_snapshot);
    RUN_TEST(test_payload_100);
    RUN_TEST(test_payload_203);
    RUN_TEST(test_payload_20a);
    RUN_TEST(test_payload_20c);
    RUN_TEST(test_payload_301);
    RUN_TEST(test_payload_482);
    RUN_TEST(test_payload_485);
    RUN_TEST(test_payload_7f0);
    RUN_TEST(test_senders_0x7f0_window);
    RUN_TEST(test_senders_window_expires);
    RUN_TEST(test_lowlevel_init_smoke);
    RUN_TEST(test_hw_init_retransmission_enabled);
    tst_exit(UNITY_END());
    return 0;
}
