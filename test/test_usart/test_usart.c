/**
 * \file            test_usart.c
 * \brief           USART2 link (usart.c): TX ring, sequencer, DMA and RX parser behaviour
 * \note            Received bytes come from the bus side (x3emu USART2 model into the RX DMA
 *                      ring); sent frames are captured when the sequencer arms DMA1 channel 7
 *                      (tst_bsp_fakes.h). Frames are built and checked with the independent
 *                      tst_ninebot helpers.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <stddef.h>
#include <string.h>
#include <unity.h>
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define NODE_VCU       0x16u
#define NODE_APP       0x23u
#define NODE_INFO      0x3Du
#define CMD_READ       0x01u
#define CMD_WRITE3     0x03u
#define CMD_READ_REPLY 0x04u
#define REG_VERSION    0x1Au
#define FREE_AT_BOOT   7u     /* ctx +2 = 7 after init */
#define SLOT_STRIDE    0x100u /* slot frame = ctx + tail * 0x100 + 0xC */
#define SLOT_LEN_OFF   0x08u
#define SLOT_FRAME_OFF 0x0Cu
#define BODY_LIMIT     243u /* the enqueue accepts body_len < 243 */
#define LEN_FIELD_MAX  236u /* the parser rejects (len + 7) - 7 > 236 */
#define RX_SETTLE_US   3000u /* > 20 bytes at 115200 Bd */
#define RX_GAP_US      50000u
#define RX_RING_BYTES  250u
#define BAUD_DIV_72MHZ 625u /* 72 MHz PCLK1 / 115200 */

static uint8_t g_big_body[BODY_LIMIT];

/**
 * \brief           Build the reference request: read REG_VERSION, count 1, from the info node
 * \param[out]      wire: destination (>= 16 bytes)
 * \return          frame length on the wire
 */
static uint16_t prv_request(uint8_t* wire) {
    const uint8_t body[1] = {1u};

    return tst_nb_build(wire, 16u, NODE_INFO, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
}

/**
 * \brief           Have bytes arrive on USART2 RX and let them reach the DMA ring
 * \param[in]       data: bytes
 * \param[in]       len: count
 */
static void prv_rx(const uint8_t* data, uint32_t len) {
    tst_emu_usart_rx(data, len);
    tst_delay_us(RX_SETTLE_US + (len * 100u));
}

/**
 * \brief           Assert captured frame `index` is a version reply to the info node
 * \param[in]       index: capture index
 */
static void prv_assert_reply(uint32_t index) {
    const tst_usart_frame_t* raw = tst_usart_tx_get(index);
    tst_nb_frame_t frame;

    TEST_ASSERT_NOT_NULL(raw);
    TEST_ASSERT_TRUE(tst_nb_parse(raw->bytes, raw->len, &frame));
    TEST_ASSERT_TRUE(frame.checksum_ok);
    TEST_ASSERT_EQUAL_HEX8(NODE_INFO, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(CMD_READ_REPLY, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(REG_VERSION, frame.arg);
}

/**
 * \brief           Enqueue a 3-byte report from the VCU to the app node
 * \param[in]       tag: first body byte
 */
static void prv_report(uint8_t tag) {
    const uint8_t body[3] = {tag, 0x22u, 0x33u};

    vcu_usart_report_send((volatile uint8_t*)&g_uartctx, NODE_VCU, NODE_APP, sizeof(body), CMD_WRITE3, 0x10u, body);
}

/**
 * \brief           Fresh boot state, fakes, USART2 + its DMA
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
}

/**
 * \brief           Let the last burst finish so the next test starts idle
 */
void tearDown(void) { (void)tst_wait_set(&DMA1->sts, DMA1_FDT7_FLAG, TST_WAIT_LOOPS_DEFAULT); }

/* -- init + ring layout ------------------------------------------------------------------- */

/**
 * \brief           After init the TX ring is empty with 7 free slots
 * \note            Head 0, tail 0, free 7, busy 0. The enqueue gates on ctx +2 != 0.
 */
static void test_init_free_slots(void) {
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT, g_uartctx.free);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.head);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.tail);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.busy);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.lock);
}

/**
 * \brief           Slot i keeps its length at ctx + 0x08 + i*0x100 and its frame at ctx + 0x0C + i*0x100
 * \note            The enqueue and the sequencer address the slots this way (8 slots).
 */
static void test_slot_layout(void) {
    const volatile uint8_t* base = (const volatile uint8_t*)&g_uartctx;

    TEST_ASSERT_EQUAL_UINT32(SLOT_LEN_OFF, (uint32_t)((const volatile uint8_t*)&g_uartctx.slot[0].len - base));
    TEST_ASSERT_EQUAL_UINT32(SLOT_FRAME_OFF, (uint32_t)(g_uartctx.slot[0].frame - base));
    TEST_ASSERT_EQUAL_UINT32(SLOT_FRAME_OFF + 7u * SLOT_STRIDE, (uint32_t)(g_uartctx.slot[7].frame - base));
    TEST_ASSERT_EQUAL_UINT32(8u, VCU_UART_SLOTS);
}

/**
 * \brief           USART2 at 115200 8N1 with DMA TX/RX, RX DMA circular over 250 bytes (smoke)
 * \note            Controller: baud 0x1C200, TE, RE, DMAT, DMAR, UEN. RX DMA: DMA1 Ch6, 0xFA
 *                      bytes, circular, enabled. TX Ch7 left disabled.
 */
static void test_init_registers(void) {
    TEST_ASSERT_EQUAL_UINT32(BAUD_DIV_72MHZ, USART2->baudr);
    TEST_ASSERT_EQUAL_UINT32(1u, USART2->ctrl1_bit.ten);
    TEST_ASSERT_EQUAL_UINT32(1u, USART2->ctrl1_bit.ren);
    TEST_ASSERT_EQUAL_UINT32(1u, USART2->ctrl1_bit.uen);
    TEST_ASSERT_EQUAL_UINT32(1u, USART2->ctrl3_bit.dmaten);
    TEST_ASSERT_EQUAL_UINT32(1u, USART2->ctrl3_bit.dmaren);
    TEST_ASSERT_EQUAL_UINT32(1u, DMA1_CHANNEL6->ctrl_bit.chen);
    TEST_ASSERT_EQUAL_UINT32(1u, DMA1_CHANNEL6->ctrl_bit.lm);
    TEST_ASSERT_EQUAL_UINT32(RX_RING_BYTES, DMA1_CHANNEL6->dtcnt);
    TEST_ASSERT_EQUAL_UINT32(0u, DMA1_CHANNEL7->ctrl_bit.chen);
}

/* -- TX ring ------------------------------------------------------------------------------ */

/**
 * \brief           A report is framed into the tail slot: 5A A5 len src dst cmd arg body ck16
 * \note            Checksum = ~sum from the len byte; slot len = frame length, tail++, free--.
 */
static void test_enqueue_frames_into_tail_slot(void) {
    static const uint8_t head[10] = {0x5A, 0xA5, 0x03, NODE_VCU, NODE_APP, CMD_WRITE3, 0x10, 0x11, 0x22, 0x33};
    const volatile uint8_t* frame = g_uartctx.slot[0].frame;
    uint8_t copy[12];
    uint16_t cksum;
    uint8_t idx;

    prv_report(0x11u);
    for (idx = 0u; idx < sizeof(copy); ++idx) {
        copy[idx] = frame[idx];
    }
    cksum = tst_nb_checksum(&copy[2], 8u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(head, copy, sizeof(head));
    TEST_ASSERT_EQUAL_HEX8((uint8_t)cksum, copy[10]);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(cksum >> 8), copy[11]);
    TEST_ASSERT_EQUAL_UINT32(12u, g_uartctx.slot[0].len);
    TEST_ASSERT_EQUAL_UINT8(1u, g_uartctx.tail);
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT - 1u, g_uartctx.free);
}

/**
 * \brief           Seven reports fit, the eighth is dropped (free reaches 0)
 * \note            The enqueue runs only `if (body_len < 0xF3 && ctx[2] != 0)`.
 */
static void test_ring_holds_seven(void) {
    uint8_t tag;

    for (tag = 0u; tag < 8u; ++tag) {
        prv_report(tag);
    }
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.free);
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT, g_uartctx.tail);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.busy);
}

/**
 * \brief           A body of 0xF2 bytes is accepted (0xFB on the wire), 0xF3 is dropped
 * \note            The enqueue accepts `body_len < 0xF3`.
 */
static void test_enqueue_body_limit(void) {
    vcu_usart_report_send((volatile uint8_t*)&g_uartctx, NODE_VCU, NODE_APP, BODY_LIMIT, CMD_WRITE3, 0u, g_big_body);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.tail);
    vcu_usart_report_send((volatile uint8_t*)&g_uartctx, NODE_VCU, NODE_APP, BODY_LIMIT - 1u, CMD_WRITE3, 0u,
                          g_big_body);
    TEST_ASSERT_EQUAL_UINT8(1u, g_uartctx.tail);
    TEST_ASSERT_EQUAL_UINT32(BODY_LIMIT - 1u + TST_NB_OVERHEAD, g_uartctx.slot[0].len);
}

/**
 * \brief           The enqueue does nothing while the ring is busy or locked
 * \note            The enqueue runs only `if (ctx[3] == 0 && ctx[4] == 0)`.
 */
static void test_enqueue_blocked_by_busy_or_lock(void) {
    g_uartctx.busy = 1u;
    prv_report(1u);
    g_uartctx.busy = 0u;
    g_uartctx.lock = 1u;
    prv_report(2u);
    g_uartctx.lock = 0u;
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.tail);
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT, g_uartctx.free);
}

/* -- TX sequencer + DMA ------------------------------------------------------------------- */

/**
 * \brief           One frame per pass; a pass while the burst is in flight sends nothing
 * \note            The sequencer arms one DMA burst per pass (head++ and free++ on success).
 *                      The DMA arm returns 0 `if (FDT7 == 0 && busy == 1)`, else it latches busy,
 *                      clears FDT7, channel off, maddr = slot frame, count = len, channel on.
 */
static void test_sequencer_waits_for_dma(void) {
    const tst_usart_frame_t* raw;

    prv_report(0xA1u);
    prv_report(0xA2u);
    vcu_usart_tx_sequencer();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_tx_count());
    TEST_ASSERT_EQUAL_UINT8(1u, g_uartctx.head);
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT - 1u, g_uartctx.free);
    vcu_usart_tx_sequencer();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_tx_count());
    TEST_ASSERT_EQUAL_UINT8(1u, g_uartctx.head);
    TEST_ASSERT_TRUE(tst_wait_set(&DMA1->sts, DMA1_FDT7_FLAG, TST_WAIT_LOOPS_DEFAULT));
    vcu_usart_tx_sequencer();
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_tx_count());
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT, g_uartctx.free);
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.lock);
    raw = tst_usart_tx_get(1u);
    TEST_ASSERT_EQUAL_UINT16(12u, raw->len);
    TEST_ASSERT_EQUAL_HEX8(0xA2u, raw->bytes[7]);
    TEST_ASSERT_EQUAL_HEX32((uint32_t)(uintptr_t)g_uartctx.slot[1].frame, DMA1_CHANNEL7->maddr);
}

/**
 * \brief           The sequencer does nothing while the ring is busy or locked
 * \note            The sequencer runs only `if (ctx[3] == 0 && ctx[4] == 0 && tail != head)`.
 */
static void test_sequencer_blocked_by_busy_or_lock(void) {
    prv_report(1u);
    g_uartctx.busy = 1u;
    vcu_usart_tx_sequencer();
    g_uartctx.busy = 0u;
    g_uartctx.lock = 1u;
    vcu_usart_tx_sequencer();
    g_uartctx.lock = 0u;
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_tx_count());
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.head);
}

/**
 * \brief           Head and tail wrap at 8: nine report/send cycles leave both at 1, free back at 7
 * \note            Enqueue and sequencer both wrap: `if (++idx == 8) idx = 0`.
 */
static void test_ring_wraps_at_eight(void) {
    uint8_t cycle;

    for (cycle = 0u; cycle < 9u; ++cycle) {
        prv_report(cycle);
        TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    }
    TEST_ASSERT_EQUAL_UINT8(1u, g_uartctx.tail);
    TEST_ASSERT_EQUAL_UINT8(1u, g_uartctx.head);
    TEST_ASSERT_EQUAL_UINT8(FREE_AT_BOOT, g_uartctx.free);
    TEST_ASSERT_EQUAL_UINT32(9u, tst_usart_tx_count());
}

/* -- RX path ------------------------------------------------------------------------------ */

/**
 * \brief           A request split over two polls 50 ms apart is still assembled (no RX timeout)
 * \note            The parser keeps its 6-byte state between calls and has no timer: only a
 *                      byte mismatch, a length reject or a completed frame resets it.
 */
static void test_rx_split_frame_no_timeout(void) {
    uint8_t wire[16];
    uint16_t len = prv_request(wire);

    prv_rx(wire, 4u);
    vcu_usart_poll();
    tst_delay_us(RX_GAP_US);
    prv_rx(&wire[4], len - 4u);
    vcu_usart_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_assert_reply(0u);
}

/**
 * \brief           A doubled 0x5A header byte loses sync: the frame after it is not seen
 * \note            While hunting, 0x5A with got_header0 == 0 sets it; a second
 *                      0x5A matches neither branch and resets; the following 0xA5 then also
 *                      resets (got_header0 == 0).
 */
static void test_rx_double_header_loses_sync(void) {
    uint8_t wire[17];
    uint16_t len;

    wire[0] = 0x5Au;
    len = prv_request(&wire[1]);
    prv_rx(wire, len + 1u);
    vcu_usart_poll();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           A length field above 0xEC aborts the frame; the parser hunts again at once
 * \note            At index 0, `0xEC < ((len + 7) & 0xFF) - 7` resets the parser
 *                      state; the bytes after it are hunted for 5A A5.
 */
static void test_rx_length_reject_then_resync(void) {
    uint8_t wire[19] = {0x5Au, 0xA5u, LEN_FIELD_MAX + 1u};
    uint16_t len = prv_request(&wire[3]);

    prv_rx(wire, len + 3u);
    vcu_usart_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_assert_reply(0u);
}

/**
 * \brief           The RX pump does nothing while ctx busy (+3) or its own latch (+5) is set
 * \note            The pump runs only `if (ctx[3] == 0 && ctx[5] == 0)`; the bytes stay in the
 *                      DMA ring and are parsed by the next free poll.
 */
static void test_rx_poll_guarded(void) {
    uint8_t wire[16];
    uint16_t len = prv_request(wire);

    prv_rx(wire, len);
    g_uartctx.busy = 1u;
    vcu_usart_poll();
    g_uartctx.busy = 0u;
    g_uartctx.rx_busy = 1u;
    vcu_usart_poll();
    g_uartctx.rx_busy = 0u;
    TEST_ASSERT_EQUAL_UINT8(0u, g_uartctx.tail);
    vcu_usart_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_assert_reply(0u);
}

/**
 * \brief           Requests keep being parsed across the 250-byte DMA ring wrap
 * \note            Ring drain: head = 250 - CNDTR, copy read_idx..head, read_idx wraps
 *                      to 0 past 0xF9. 30 x 10-byte requests = 300 bytes, crossing the wrap.
 */
static void test_rx_ring_wrap(void) {
    uint8_t wire[16];
    uint16_t len = prv_request(wire);
    uint32_t idx;
    uint32_t replies = 0u;

    for (idx = 0u; idx < 30u; ++idx) {
        tst_fakes_reset();
        prv_rx(wire, len);
        vcu_usart_poll();
        replies += tst_usart_drain();
    }
    TEST_ASSERT_EQUAL_UINT32(30u, replies);
    prv_assert_reply(0u);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_init_free_slots);
    RUN_TEST(test_slot_layout);
    RUN_TEST(test_init_registers);
    RUN_TEST(test_enqueue_frames_into_tail_slot);
    RUN_TEST(test_ring_holds_seven);
    RUN_TEST(test_enqueue_body_limit);
    RUN_TEST(test_enqueue_blocked_by_busy_or_lock);
    RUN_TEST(test_sequencer_waits_for_dma);
    RUN_TEST(test_sequencer_blocked_by_busy_or_lock);
    RUN_TEST(test_ring_wraps_at_eight);
    RUN_TEST(test_rx_split_frame_no_timeout);
    RUN_TEST(test_rx_double_header_loses_sync);
    RUN_TEST(test_rx_length_reject_then_resync);
    RUN_TEST(test_rx_poll_guarded);
    RUN_TEST(test_rx_ring_wrap);
    tst_exit(UNITY_END());
    return 0;
}
