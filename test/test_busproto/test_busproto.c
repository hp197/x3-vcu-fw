/**
 * \file            test_busproto.c
 * \brief           Peripheral-bus TP engine and report manager (busproto.c)
 * \note            CAN frames are captured at can_message_transmit (no controller,
 *                      \ref tst_can_tx_forward), USART frames at the DMA arm.
 *
 *                      Addresses on the wire: the peer node is 0x3D, the VCU 0x16, the app 0x23.
 *                      A TP id is 0x18<PF><dst><src> (PF 0xEC = TP.CM, 0xEB = TP.DT).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "busproto.h"
#include "can.h"
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

#define NODE_VCU       0x16u
#define NODE_APP       0x23u
#define NODE_PEER      0x3Du
#define NODE_FOREIGN   0x10u
#define ID_CM_TO_VCU   0x18EC163Du /* TP.CM peer -> VCU */
#define ID_DT_TO_VCU   0x18EB163Du /* TP.DT peer -> VCU */
#define ID_CM_TO_PEER  0x18EC3D16u /* TP.CM VCU -> peer */
#define ID_DT_TO_PEER  0x18EB3D16u /* TP.DT VCU -> peer */
#define ID_CM_TO_APP   0x18EC233Du /* TP.CM peer -> app node */
#define ID_DT_TO_APP   0x18EB233Du /* TP.DT peer -> app node */
#define ID_CM_APP_REP  0x18EC3D23u /* TP.CM answered as the app node */
#define ID_EF_TO_VCU   0x18EF163Du /* Ninebot single frame peer -> VCU */
#define ID_EF_TO_PEER  0x18EF3D16u /* Ninebot single frame VCU -> peer */
#define TP_RTS         0x10u
#define TP_CTS         0x11u
#define TP_EOMA        0x13u
#define TP_ABORT       0xFFu
#define TP_CTL         0x01u
#define TP_CHUNK       7u
#define TP_MSG_MAX     64u
#define CMD_READ       0x01u
#define CMD_WRITE3     0x03u
#define REG_SERIAL     0x10u
#define REG_VERSION    0x1Au
#define SERIAL_LEN     14u
#define TMO_CTS_WAIT   1250u /* 0x4E2 / period 1: RTS sent / CTS sent, waiting */
#define TMO_DT_GAP     750u  /* 0x2EE / period 1: between TP.DT packets */
#define CAN_FLUSH_MAX  32u
#define NB_ARG_FC      0xFCu
#define NB_ARG_FD      0xFDu
#define NB_ARG_FE      0xFEu
#define NB_CMD_FD      0x05u
#define NB_CMD_FE      0x03u
#define NB_CMD_FC      0x04u
#define SIG_COUNT      38u
#define SIG_REC_LEN    5u
#define KEY_MAX        64u

/* Signal keys used below (\ref k_sig_expected). */
#define SIG_POWER_HALF 0x7E765289u /* [ 0] 16 bit, g_scr.reg[0x01] */
#define SIG_POWER_SIGN 0x42416F9Fu /* [22]  1 bit, g_scr.reg[0x03] */
#define SIG_TELEM_3C   0x8689B2DAu /* [17]  4 bit, g_scr.reg[0x1E] */

/** \brief One expected signal descriptor: key, bit width and g_scr halfword index. */
typedef struct {
    uint32_t key;  /*!< +0x00 */
    uint8_t width; /*!< +0x04 */
    uint8_t scr;   /*!< halfword index into g_scr */
} sig_expected_t;

/** \brief The 38 expected descriptors (count 0x26), in table order. */
static const sig_expected_t k_sig_expected[SIG_COUNT] = {
    {0x7E765289u, 16u, 0x01u}, {0xDDAA7A89u, 16u, 0x0Cu}, {0xAC440AC5u, 16u, 0x0Fu}, {0xA3DDD1B3u, 16u, 0x12u},
    {0xA3DDD1B2u, 16u, 0x13u}, {0xA3DDD1ADu, 16u, 0x14u}, {0xA3DDD1ACu, 16u, 0x15u}, {0xA3DDD1AFu, 16u, 0x16u},
    {0x6FE56C44u, 16u, 0x20u}, {0xAD4A48AEu, 16u, 0x10u}, {0x9096F99Eu, 16u, 0x0Du}, {0x9C9B65E4u, 16u, 0x22u},
    {0x229A0092u, 16u, 0x49u}, {0xA4F6D064u, 8u, 0x28u},  {0x616A4512u, 8u, 0x17u},  {0xB3E2E070u, 8u, 0x06u},
    {0xE799388Au, 8u, 0x07u},  {0x8689B2DAu, 4u, 0x1Eu},  {0x7E0124A1u, 4u, 0x08u},  {0xF8F9CB4Bu, 4u, 0x21u},
    {0xA2BC1D75u, 4u, 0x04u},  {0x46DB128Fu, 4u, 0x3Au},  {0x42416F9Fu, 1u, 0x03u},  {0x17B28D98u, 1u, 0x1Fu},
    {0xCFBC0A29u, 1u, 0x39u},  {0x88E7BACCu, 1u, 0x38u},  {0x0C99E9E5u, 1u, 0x0Bu},  {0x1841E961u, 1u, 0x09u},
    {0x16B5A88Au, 1u, 0x2Cu},  {0x70EFD7B3u, 1u, 0x2Du},  {0x751A2603u, 1u, 0x05u},  {0x06BFB039u, 1u, 0x29u},
    {0x20E8263Bu, 1u, 0x26u},  {0x7F1A2555u, 1u, 0x2Bu},  {0xF739712Bu, 1u, 0x4Au},  {0xAC4E646Bu, 1u, 0x4Bu},
    {0x0086BD19u, 1u, 0x4Cu},  {0x0C7D931Au, 1u, 0x4Du},
};

/** \brief Calls into the test report-node object (register_obj / broadcast tests). */
static uint32_t g_obj_calls[5];

/* ---------------------------------------------------------------------------------------- */
/* Helpers                                                                                   */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Feed one 8-byte ext-id frame to the engine, as the ext-id CAN handler does
 * \param[in]       id: 29-bit identifier
 * \param[in]       data: 8 data bytes
 */
static void prv_rx(uint32_t id, const uint8_t* data) {
    bp_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.dlc = 8u;
    memcpy(frame.data, data, 8u);
    vcu_busproto_rx(&frame);
}

/**
 * \brief           Feed a TP.CM frame: control byte, 4 parameter bytes, then PGN 0x00EF00
 * \param[in]       id: 29-bit identifier
 * \param[in]       ctrl: control byte
 * \param[in]       p1: data[1]
 * \param[in]       p2: data[2]
 * \param[in]       p3: data[3]
 */
static void prv_rx_cm(uint32_t id, uint8_t ctrl, uint8_t p1, uint8_t p2, uint8_t p3) {
    const uint8_t data[8] = {ctrl, p1, p2, p3, 0xFFu, 0x00u, 0xEFu, 0x00u};

    prv_rx(id, data);
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
 * \brief           Assert captured CAN frame `index`: extended id and 8 data bytes
 * \param[in]       index: capture index
 * \param[in]       id: expected identifier
 * \param[in]       data: expected 8 data bytes
 */
static void prv_assert_can(uint32_t index, uint32_t id, const uint8_t* data) {
    const tst_can_frame_t* frame = tst_can_tx_get(index);

    TEST_ASSERT_NOT_NULL_MESSAGE(frame, "no CAN frame at this index");
    TEST_ASSERT_EQUAL_UINT8(1u, frame->ext);
    TEST_ASSERT_EQUAL_HEX32(id, frame->id);
    TEST_ASSERT_EQUAL_UINT8(8u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, frame->data, 8u);
}

/**
 * \brief           Assert captured CAN frame `index` is a TP.CM frame `ctrl p1 p2 p3 FF 00 EF 00`
 * \param[in]       index: capture index
 * \param[in]       id: expected identifier
 * \param[in]       ctrl: control byte
 * \param[in]       p1: data[1]
 * \param[in]       p2: data[2]
 * \param[in]       p3: data[3]
 */
static void prv_assert_cm(uint32_t index, uint32_t id, uint8_t ctrl, uint8_t p1, uint8_t p2, uint8_t p3) {
    const uint8_t data[8] = {ctrl, p1, p2, p3, 0xFFu, 0x00u, 0xEFu, 0x00u};

    prv_assert_can(index, id, data);
}

/**
 * \brief           Assert captured CAN frame `index` is a TP.CM connection abort
 * \param[in]       index: capture index
 * \param[in]       id: expected identifier
 */
static void prv_assert_abort(uint32_t index, uint32_t id) { prv_assert_cm(index, id, TP_ABORT, 0xFFu, 0xFFu, 0xFFu); }

/**
 * \brief           Build a TP message: header ctl/cmd/arg, body, inverse 16-bit sum LE
 * \param[out]      out: message buffer (at least body_len + 5 bytes)
 * \param[in]       cmd: Ninebot command
 * \param[in]       arg: Ninebot argument
 * \param[in]       body: body bytes
 * \param[in]       body_len: body length
 * \return          message length
 */
static uint8_t prv_tp_message(uint8_t* out, uint8_t cmd, uint8_t arg, const uint8_t* body, uint8_t body_len) {
    uint16_t sum = 0u;
    uint8_t idx;
    uint8_t len = (uint8_t)(body_len + 3u);

    out[0] = TP_CTL;
    out[1] = cmd;
    out[2] = arg;
    memcpy(&out[3], body, body_len);
    for (idx = 0u; idx < len; ++idx) {
        sum = (uint16_t)(sum + out[idx]);
    }
    sum = (uint16_t)~sum;
    out[len] = (uint8_t)sum;
    out[len + 1u] = (uint8_t)(sum >> 8);
    return (uint8_t)(len + 2u);
}

/**
 * \brief           Send TP.DT packets `first..last` (1-based) of a message, 0xFF padded
 * \param[in]       id: TP.DT identifier
 * \param[in]       msg: message bytes
 * \param[in]       len: message length
 * \param[in]       first: first packet number
 * \param[in]       last: last packet number
 */
static void prv_rx_dt(uint32_t id, const uint8_t* msg, uint8_t len, uint8_t first, uint8_t last) {
    uint8_t pkt;

    for (pkt = first; pkt <= last; ++pkt) {
        uint8_t data[8];
        uint8_t off = (uint8_t)((pkt - 1u) * TP_CHUNK);
        uint8_t idx;

        data[0] = pkt;
        for (idx = 0u; idx < TP_CHUNK; ++idx) {
            data[1u + idx] = ((off + idx) < len) ? msg[off + idx] : 0xFFu;
        }
        prv_rx(id, data);
    }
}

/**
 * \brief           A whole inbound transfer: RTS, then every TP.DT packet
 * \param[in]       cm_id: TP.CM identifier
 * \param[in]       dt_id: TP.DT identifier
 * \param[in]       msg: message bytes
 * \param[in]       len: message length
 */
static void prv_rx_transfer(uint32_t cm_id, uint32_t dt_id, const uint8_t* msg, uint8_t len) {
    uint8_t npkt = (uint8_t)((len + TP_CHUNK - 1u) / TP_CHUNK);

    prv_rx_cm(cm_id, TP_RTS, len, 0u, npkt);
    prv_rx_dt(dt_id, msg, len, 1u, npkt);
}

/**
 * \brief           Run the slot-3 fast service `count` times
 * \param[in]       count: number of service passes
 */
static void prv_fast(uint32_t count) {
    uint32_t pass;

    for (pass = 0u; pass < count; ++pass) {
        bp_task_fast_service();
    }
}

/**
 * \brief           Fill registers `first..` of page 0 with an incrementing byte pattern
 * \param[in]       first: first register
 * \param[in]       bytes: number of bytes
 * \param[in]       seed: first byte value
 */
static void prv_fill_regs(uint8_t first, uint8_t bytes, uint8_t seed) {
    volatile uint8_t* dst = (volatile uint8_t*)&g_in2.reg[first];
    uint8_t idx;

    for (idx = 0u; idx < bytes; ++idx) {
        dst[idx] = (uint8_t)(seed + idx);
    }
}

/**
 * \brief           Start a VCU -> peer transfer of the 14-byte serial (reg 0x10 read over TP)
 * \note            The read reply has 14 body bytes, so the Ninebot sink takes the TP path:
 *                      a 19-byte buffer (`01 04 10` + 14 bytes + checksum), RTS on the next
 *                      fast-service pass. Leaves the capture empty, the session in state 4.
 */
static void prv_start_tx_serial(void) {
    const uint8_t body[] = {SERIAL_LEN};
    uint8_t msg[TP_MSG_MAX];
    uint8_t len = prv_tp_message(msg, CMD_READ, REG_SERIAL, body, sizeof(body));

    prv_fill_regs(REG_SERIAL, SERIAL_LEN + 2u, 0x30u);
    prv_rx_transfer(ID_CM_TO_VCU, ID_DT_TO_VCU, msg, len);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_flush()); /* CTS, EOMA, RTS */
    prv_assert_cm(2u, ID_CM_TO_PEER, TP_RTS, 19u, 0u, 3u);
    tst_fakes_reset();
    tst_can_tx_forward(0u);
}

/**
 * \brief           The report manager object (g_bp_rptr.mgr_ptr)
 * \return          the manager
 */
static volatile bp_report_mgr_t* prv_mgr(void) { return (volatile bp_report_mgr_t*)(uintptr_t)g_bp_rptr.mgr_ptr; }

/**
 * \brief           Build a registration key {id, period, N x {u32 key LE, u8 width}}
 * \param[out]      out: key buffer (KEY_MAX)
 * \param[in]       node: node id
 * \param[in]       period: requested period
 * \param[in]       keys: signal keys
 * \param[in]       widths: signal widths
 * \param[in]       count: number of signals
 * \return          key length
 */
static uint8_t prv_reg_key(uint8_t* out, uint8_t node, uint8_t period, const uint32_t* keys, const uint8_t* widths,
                           uint8_t count) {
    uint8_t idx;
    uint8_t len = 2u;

    out[0] = node;
    out[1] = period;
    for (idx = 0u; idx < count; ++idx) {
        out[len] = (uint8_t)keys[idx];
        out[len + 1u] = (uint8_t)(keys[idx] >> 8);
        out[len + 2u] = (uint8_t)(keys[idx] >> 16);
        out[len + 3u] = (uint8_t)(keys[idx] >> 24);
        out[len + 4u] = widths[idx];
        len = (uint8_t)(len + SIG_REC_LEN);
    }
    return len;
}

/**
 * \brief           Parse captured USART frame `index` and check src/dst/cmd/arg/len
 * \param[in]       index: capture index
 * \param[in]       cmd: expected command
 * \param[in]       arg: expected argument
 * \param[in]       len: expected body length
 * \param[out]      frame: the parsed frame
 */
static void prv_usart_frame(uint32_t index, uint8_t cmd, uint8_t arg, uint8_t len, tst_nb_frame_t* frame) {
    const tst_usart_frame_t* raw = tst_usart_tx_get(index);

    TEST_ASSERT_NOT_NULL_MESSAGE(raw, "no USART frame captured at this index");
    TEST_ASSERT_TRUE_MESSAGE(tst_nb_parse(raw->bytes, raw->len, frame), "not a 5A A5 frame of its length");
    TEST_ASSERT_TRUE_MESSAGE(frame->checksum_ok, "bad checksum");
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, frame->src);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame->dst);
    TEST_ASSERT_EQUAL_HEX8(cmd, frame->cmd);
    TEST_ASSERT_EQUAL_HEX8(arg, frame->arg);
    TEST_ASSERT_EQUAL_UINT8(len, frame->len);
}

/**
 * \brief           Test node object slot at +0x04: counts its calls
 * \param[in]       obj: the object
 */
static void prv_obj_slot04(void* obj) {
    (void)obj;
    ++g_obj_calls[1];
}

/**
 * \brief           Test node object slot at +0x0C: counts its calls
 * \param[in]       obj: the object
 */
static void prv_obj_slot0c(void* obj) {
    (void)obj;
    ++g_obj_calls[3];
}

/**
 * \brief           Test node object slot at +0x10: counts its calls
 * \param[in]       obj: the object
 */
static void prv_obj_slot10(void* obj) {
    (void)obj;
    ++g_obj_calls[4];
}

/** \brief A node object with a distinct code pointer in each of its first five words. */
static const uint32_t k_test_obj[5] = {
    0u,
    (uint32_t)(uintptr_t)&prv_obj_slot04,
    0u,
    (uint32_t)(uintptr_t)&prv_obj_slot0c,
    (uint32_t)(uintptr_t)&prv_obj_slot10,
};

/**
 * \brief           Fresh boot state: product RAM, fakes, USART2 + CAN rings, busproto init
 * \note            The init's own frames (mode report, 0xD0 announce) are drained and the
 *                      captures cleared, so every test starts with empty captures.
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    vcu_usart_init();
    vcu_can_init();
    vcu_busproto_init();
    (void)tst_usart_drain();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    memset(g_obj_calls, 0, sizeof(g_obj_calls));
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* ---------------------------------------------------------------------------------------- */
/* Init and the node-accept gate                                                             */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Init: 5 sessions on the manager, report manager 0x16 -> 0x23, 38 signals
 * \note            The manager init registers 5 channels (300-byte buffers, period 1) and
 *                      the handler table; the subsystem init stores the report manager built for
 *                      (0x16, 0x23) into g_bp_rptr.mgr_ptr, clears the 0x28-byte node record,
 *                      sends the 0xD0 announce and sets VS+0x27. The descriptor count is 0x26.
 */
static void test_init_builds_manager(void) {
    volatile bp_report_mgr_t* mgr = prv_mgr();

    TEST_ASSERT_EQUAL_UINT16(5u, g_bp_mgr.count);
    TEST_ASSERT_NOT_NULL(g_bp_mgr.base);
    TEST_ASSERT_NOT_NULL(mgr);
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, mgr->nb_src);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, mgr->nb_dst);
    TEST_ASSERT_EQUAL_UINT8(0u, mgr->node_count);
    TEST_ASSERT_EQUAL_UINT16(SIG_COUNT, mgr->desc_count);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_reg_pending);
    TEST_ASSERT_EQUAL_UINT8(0u, g_bp_node.node_id);
    TEST_ASSERT_EQUAL_UINT8(0u, g_bp_node.cap[0]);
}

/**
 * \brief           The boot announce: `16 -> 23 cmd 03 arg D0 body 01 00`
 * \note            The body is the first 2 bytes of a little-endian 32-bit 1. On the wire:
 *                      `5a a5 02 16 23 03 d0 01 00`.
 */
static void test_init_sends_announce(void) {
    const uint8_t expect[] = {0x01u, 0x00u};
    tst_nb_frame_t frame;
    uint32_t sent;
    uint32_t idx;
    uint8_t found = 0u;

    vcu_busproto_init();
    sent = tst_usart_drain();
    for (idx = 0u; idx < sent; ++idx) {
        const tst_usart_frame_t* raw = tst_usart_tx_get(idx);

        if ((raw != NULL) && tst_nb_parse(raw->bytes, raw->len, &frame) && (frame.arg == 0xD0u)) {
            prv_usart_frame(idx, CMD_WRITE3, 0xD0u, 2u, &frame);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
            found = 1u;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "no 0xD0 announce frame");
}

/**
 * \brief           An RTS opens a session only for the destinations 02 07 16 23 3D 3E 3F
 * \note            The node-accept gate (ops[0]) compares the PS byte with 0x16, 0x23, 0x3D,
 *                      0x3E, 0x3F, 2, 7; the source byte is not inspected.
 */
static void test_node_accept_destination_set(void) {
    uint32_t da;

    for (da = 0u; da < 0x100u; ++da) {
        uint8_t want = ((da == 0x02u) || (da == 0x07u) || (da == 0x16u) || (da == 0x23u) || (da == 0x3Du)
                        || (da == 0x3Eu) || (da == 0x3Fu))
                           ? 1u
                           : 0u;

        TEST_ASSERT_EQUAL_INT_MESSAGE(want, bp_node_accept(0x55u, (uint8_t)da), "destination");
    }
}

/**
 * \brief           An RTS to a destination outside the set gets no answer
 * \note            The dispatcher: ops[0] != 1 -> no session, no frame. RTS 0x18EC103D ->
 *                      nothing.
 */
static void test_rts_to_foreign_node_is_ignored(void) {
    prv_rx_cm(0x18EC0000u | ((uint32_t)NODE_FOREIGN << 8) | NODE_PEER, TP_RTS, 6u, 0u, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
}

/* ---------------------------------------------------------------------------------------- */
/* Frame builders                                                                            */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Abort frame: id dst | src<<8 | 0x18EC0000, bits 31/29 of the old id kept
 * \note            id = `*f & 0xA0000000 | (dst | src<<8 | 0x18EC0000) & 0x1FFFFFFF |
 *                      0x20000000`, dlc 8, data FF FF FF FF FF 00 EF 00.
 */
static void test_frame_bcast_layout(void) {
    const uint8_t expect[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0xEFu, 0x00u};
    bp_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.id = 0xFFFFFFFFu;
    TEST_ASSERT_EQUAL_INT(1, bp_frame_bcast(0x16u, 0x3Du, &frame));
    TEST_ASSERT_EQUAL_HEX32(0xB8EC3D16u, frame.id);
    TEST_ASSERT_EQUAL_UINT8(8u, frame.dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.data, 8u);
    TEST_ASSERT_EQUAL_INT(0, bp_frame_bcast(0x16u, 0x3Du, NULL));
}

/* ---------------------------------------------------------------------------------------- */
/* Inbound transfers (RX FSM)                                                                */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           RTS (6 bytes, 1 packet) -> CTS `11 01 01 FF FF 00 EF 00` from the VCU
 * \note            FSM RTS branch: CTS (da, sa, (n+6)/7, 1). On the wire:
 *                      `0x18EC3D16 110101ffff00ef00`.
 */
static void test_rts_is_answered_with_cts(void) {
    prv_rx_cm(ID_CM_TO_VCU, TP_RTS, 6u, 0u, 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_assert_cm(0u, ID_CM_TO_PEER, TP_CTS, 1u, 1u, 0xFFu);
}

/**
 * \brief           RTS whose packet count does not match ceil(total / 7) is aborted
 * \note            FSM: `(total + 6) / 7 != data[3]` -> state 1 + abort frame.
 *                      RTS `10 06 00 02` -> `0x18EC3D16 ffffffffff00ef00`.
 */
static void test_rts_packet_count_mismatch_aborts(void) {
    prv_rx_cm(ID_CM_TO_VCU, TP_RTS, 6u, 0u, 2u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_assert_abort(0u, ID_CM_TO_PEER);
}

/**
 * \brief           TP.CM with a PGN other than 0x00EF00 is ignored
 * \note            FSM: `data[6]<<8 | data[7]<<16 | data[5] == 0xEF00` gates every TP.CM
 *                      control byte.
 */
static void test_rts_with_other_pgn_is_ignored(void) {
    const uint8_t data[8] = {TP_RTS, 6u, 0u, 1u, 0xFFu, 0x00u, 0xEEu, 0x00u};

    prv_rx(ID_CM_TO_VCU, data);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
}

/**
 * \brief           A complete transfer is acknowledged (EOMA) and dispatched as a CAN request
 * \note            FSM TP.DT branch: on pos == total and a matching inverse checksum, EOMA
 *                      (da, sa, total) then ops[2], which frames `[total-5, sa, da, buf[1], buf[2],
 *                      buf+3]` for the Ninebot dispatcher with transport 1 (CAN). The 1-byte read
 *                      is answered as one 0x18EF frame: EOMA `13 06 00 01`; single-frame read
 *                      `01 04 1a 7a`.
 */
static void test_transfer_completes_and_dispatches_on_can(void) {
    const uint8_t body[] = {1u};
    const uint8_t reply[8] = {0x01u, 0x04u, REG_VERSION, 0x7Au, 0x00u, 0x00u, 0x00u, 0x00u};
    const tst_can_frame_t* frame;
    uint8_t msg[TP_MSG_MAX];
    uint8_t len = prv_tp_message(msg, CMD_READ, REG_VERSION, body, sizeof(body));

    g_in2.reg[REG_VERSION] = 0x397Au;
    prv_rx_transfer(ID_CM_TO_VCU, ID_DT_TO_VCU, msg, len);
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_flush());
    prv_assert_cm(0u, ID_CM_TO_PEER, TP_CTS, 1u, 1u, 0xFFu);
    prv_assert_cm(1u, ID_CM_TO_PEER, TP_EOMA, 6u, 0u, 1u);
    frame = tst_can_tx_get(2u);
    TEST_ASSERT_NOT_NULL(frame);
    TEST_ASSERT_EQUAL_HEX32(ID_EF_TO_PEER, frame->id);
    TEST_ASSERT_EQUAL_UINT8(4u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(reply, frame->data, 4u);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_tx_count());
}

/**
 * \brief           A 3-packet transfer to the app node is forwarded verbatim on USART
 * \note            The TP layer answers as the addressed node (CTS/EOMA id low byte = 0x23);
 *                      ops[2] dispatches with transport 1 and the Ninebot gateway sends a frame
 *                      for another node out on the other bus. CTS `0x18EC3D23 11 03 01`, EOMA
 *                      `13 11 00 03`, USART `5a a5 0c 3d 23 03 10 41..4c`.
 */
static void test_transfer_for_app_is_forwarded_to_usart(void) {
    uint8_t body[12];
    uint8_t msg[TP_MSG_MAX];
    uint8_t len;
    uint8_t idx;
    tst_nb_frame_t frame;

    for (idx = 0u; idx < sizeof(body); ++idx) {
        body[idx] = (uint8_t)(0x41u + idx);
    }
    len = prv_tp_message(msg, CMD_WRITE3, REG_SERIAL, body, sizeof(body));
    prv_rx_transfer(ID_CM_TO_APP, ID_DT_TO_APP, msg, len);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_assert_cm(0u, ID_CM_APP_REP, TP_CTS, 3u, 1u, 0xFFu);
    prv_assert_cm(1u, ID_CM_APP_REP, TP_EOMA, 17u, 0u, 3u);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    TEST_ASSERT_TRUE(tst_nb_parse(tst_usart_tx_get(0u)->bytes, tst_usart_tx_get(0u)->len, &frame));
    TEST_ASSERT_TRUE(frame.checksum_ok);
    TEST_ASSERT_EQUAL_HEX8(NODE_PEER, frame.src);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(CMD_WRITE3, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(REG_SERIAL, frame.arg);
    TEST_ASSERT_EQUAL_UINT8(sizeof(body), frame.len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(body, frame.body, sizeof(body));
}

/**
 * \brief           A wrong checksum aborts the transfer and dispatches nothing
 * \note            FSM: checksum mismatch -> abort frame, no ops[2] call. CTS then
 *                      `0x18EC3D16 ffffffffff00ef00`, no reply.
 */
static void test_transfer_bad_checksum_aborts(void) {
    const uint8_t body[] = {1u};
    uint8_t msg[TP_MSG_MAX];
    uint8_t len = prv_tp_message(msg, CMD_READ, REG_VERSION, body, sizeof(body));

    msg[len - 1u] ^= 0xFFu;
    prv_rx_transfer(ID_CM_TO_VCU, ID_DT_TO_VCU, msg, len);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_assert_cm(0u, ID_CM_TO_PEER, TP_CTS, 1u, 1u, 0xFFu);
    prv_assert_abort(1u, ID_CM_TO_PEER);
}

/**
 * \brief           A TP.DT with the wrong sequence number aborts the transfer
 * \note            FSM: `seq + 1 != data[0]` -> state 1 + abort frame. DT numbered 2 first
 *                      -> abort.
 */
static void test_transfer_out_of_sequence_aborts(void) {
    const uint8_t body[] = {1u};
    uint8_t msg[TP_MSG_MAX];
    uint8_t len = prv_tp_message(msg, CMD_READ, REG_VERSION, body, sizeof(body));

    prv_rx_cm(ID_CM_TO_VCU, TP_RTS, len, 0u, 1u);
    prv_rx_dt(ID_DT_TO_VCU, msg, len, 2u, 2u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_assert_abort(1u, ID_CM_TO_PEER);
}

/**
 * \brief           A TP.DT without an open session is ignored
 * \note            FSM: TP.DT needs a session in state 10 for (sa, da).
 */
static void test_dt_without_session_is_ignored(void) {
    const uint8_t msg[] = {1u, 2u, 3u, 4u, 5u, 6u};

    prv_rx_dt(ID_DT_TO_VCU, msg, sizeof(msg), 1u, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
}

/**
 * \brief           No TP.DT within 1250 service passes after the CTS: abort, ids reversed
 * \note            The RTS branch arms timeout 0x4E2 / period; the fast service runs the
 *                      pacing step in state 10: tick + 1 >= timeout -> abort frame (addr_a,
 *                      addr_b), i.e. the session's (peer, VCU) pair, so the abort carries id
 *                      0x18EC163D.
 */
static void test_rx_cts_timeout_aborts(void) {
    prv_rx_cm(ID_CM_TO_VCU, TP_RTS, 13u, 0u, 2u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_fast(TMO_CTS_WAIT - 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_assert_abort(1u, ID_CM_TO_VCU);
}

/**
 * \brief           Between TP.DT packets the window is 750 service passes
 * \note            FSM TP.DT branch: pos < total -> tick 0, timeout 0x2EE / period.
 */
static void test_rx_dt_gap_timeout_aborts(void) {
    const uint8_t msg[13] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u};

    prv_rx_cm(ID_CM_TO_VCU, TP_RTS, sizeof(msg), 0u, 2u);
    prv_rx_dt(ID_DT_TO_VCU, msg, sizeof(msg), 1u, 1u);
    prv_fast(TMO_DT_GAP - 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_assert_abort(1u, ID_CM_TO_VCU);
}

/* ---------------------------------------------------------------------------------------- */
/* Outbound transfers (TX FSM, slot-3 fast service)                                          */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           A 14-byte reply over CAN goes out as TP: RTS, CTS(3,1) -> 3 TP.DT, EOMA
 * \note            A CAN reply with n >= 6 goes through bp_tp_send -> bp_build_packet (buffer
 *                      `01 04 10` + body + ~sum LE, state 3); the fast service sends the RTS in
 *                      state 3, CTS -> state 5, then TP.DT (packet no., 7 bytes, FF pad).
 *                      RTS `10 13 00 03`.
 */
static void test_tx_transfer_streams_after_cts(void) {
    const uint8_t dt1[8] = {0x01u, 0x01u, 0x04u, 0x10u, 0x30u, 0x31u, 0x32u, 0x33u};
    const uint8_t dt2[8] = {0x02u, 0x34u, 0x35u, 0x36u, 0x37u, 0x38u, 0x39u, 0x3Au};
    uint16_t sum = 0x01u + 0x04u + 0x10u;
    uint8_t dt3[8] = {0x03u, 0x3Bu, 0x3Cu, 0x3Du, 0x00u, 0x00u, 0xFFu, 0xFFu};
    uint8_t idx;

    for (idx = 0u; idx < SERIAL_LEN; ++idx) {
        sum = (uint16_t)(sum + 0x30u + idx);
    }
    sum = (uint16_t)~sum;
    dt3[4] = (uint8_t)sum;
    dt3[5] = (uint8_t)(sum >> 8);
    prv_start_tx_serial();
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 3u, 1u, 0xFFu);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_flush());
    prv_assert_can(0u, ID_DT_TO_PEER, dt1);
    prv_assert_can(1u, ID_DT_TO_PEER, dt2);
    prv_assert_can(2u, ID_DT_TO_PEER, dt3);
}

/**
 * \brief           The EOMA closes the outbound session: no abort later
 * \note            FSM EOMA branch: state 9 and pos == data[1] -> state 1. Nothing is sent
 *                      after the EOMA.
 */
static void test_tx_eoma_closes_session(void) {
    prv_start_tx_serial();
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 3u, 1u, 0xFFu);
    prv_fast(1u);
    prv_rx_cm(ID_CM_TO_VCU, TP_EOMA, 19u, 0u, 3u);
    prv_fast(TMO_CTS_WAIT + 10u);
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_flush());
}

/**
 * \brief           Without the EOMA the sender gives up 1250 passes after the last TP.DT
 * \note            Fast service: state 5 -> state 9 with timeout 0x4E2 / period; state 9
 *                      runs the pacing step -> abort frame (addr_a, addr_b) = 0x18EC3D16.
 */
static void test_tx_missing_eoma_aborts(void) {
    prv_start_tx_serial();
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 3u, 1u, 0xFFu);
    prv_fast(TMO_CTS_WAIT);
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_flush());
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(4u, prv_can_flush());
    prv_assert_abort(3u, ID_CM_TO_PEER);
}

/**
 * \brief           No CTS after the RTS: abort on the 1250th service pass
 * \note            Fast service: state 3 arms 0x4E2 / period, state 4 runs the pacing step.
 *                      The abort is `0x18EC3D16`.
 */
static void test_tx_rts_timeout_aborts(void) {
    prv_start_tx_serial();
    prv_fast(TMO_CTS_WAIT - 1u); /* the pass that sent the RTS armed tick 0 */
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_assert_abort(0u, ID_CM_TO_PEER);
}

/**
 * \brief           CTS windows: after a window, a CTS with packets is ignored until a hold CTS
 * \note            FSM CTS branch: in state 6 (window done) only `data[1] == 0` is taken
 *                      (-> state 7); a new window is accepted in state 4 or 7 only.
 *                      CTS(2,1) -> DT 1, 2; CTS(1,3) -> nothing; CTS(0) -> nothing;
 *                      CTS(1,3) -> DT 3.
 */
static void test_tx_cts_window_needs_hold_between_windows(void) {
    prv_start_tx_serial();
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 2u, 1u, 0xFFu);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    TEST_ASSERT_EQUAL_HEX8(0x01u, tst_can_tx_get(0u)->data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02u, tst_can_tx_get(1u)->data[0]);
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 1u, 3u, 0xFFu);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 0u, 0u, 0xFFu);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 1u, 3u, 0xFFu);
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_flush());
    TEST_ASSERT_EQUAL_HEX8(0x03u, tst_can_tx_get(2u)->data[0]);
    TEST_ASSERT_EQUAL_HEX32(ID_DT_TO_PEER, tst_can_tx_get(2u)->id);
}

/**
 * \brief           A peer abort idles the outbound session: no TP.DT, no timeout abort
 * \note            FSM: control 0xFF on the reverse pair -> state 1.
 */
static void test_tx_peer_abort_idles_session(void) {
    prv_start_tx_serial();
    prv_rx_cm(ID_CM_TO_VCU, TP_ABORT, 0xFFu, 0xFFu, 0xFFu);
    prv_rx_cm(ID_CM_TO_VCU, TP_CTS, 3u, 1u, 0xFFu);
    prv_fast(TMO_CTS_WAIT + 10u);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_flush());
}

/**
 * \brief           bp_tp_send: buffer header + payload + inverse sum, rejects what cannot fit
 * \note            bp_tp_send -> bp_route_lookup(m, sa, da, 0) -> bp_build_packet: refuses
 *                      cap < len + 5 (300-byte buffers: len 296 fails, 295 fits) and a reverse
 *                      pair (da, sa) that is already busy.
 */
static void test_tp_send_bounds_and_reverse_pair(void) {
    static uint8_t payload[296];

    TEST_ASSERT_EQUAL_INT(0, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_VCU, NODE_PEER, 4u, 0x10u, payload, 296u, 0));
    TEST_ASSERT_EQUAL_INT(1, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_VCU, NODE_PEER, 4u, 0x10u, payload, 295u, 0));
    TEST_ASSERT_EQUAL_INT(0, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_PEER, NODE_VCU, 4u, 0x10u, payload, 8u, 0));
    TEST_ASSERT_EQUAL_INT(0, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_VCU, NODE_PEER, 4u, 0x10u, NULL, 8u, 0));
    prv_fast(1u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_flush());
    prv_assert_cm(0u, ID_CM_TO_PEER, TP_RTS, 44u, 1u, 43u); /* 300 = 0x012C bytes, 43 packets */
}

/**
 * \brief           Five sessions: a sixth concurrent transfer finds no free slot
 * \note            bp_route_lookup: forward match, else the first state-1 slot, else 0.
 */
static void test_tp_send_runs_out_of_sessions(void) {
    static const uint8_t payload[8] = {0};
    uint8_t node;

    for (node = 0u; node < 5u; ++node) {
        TEST_ASSERT_EQUAL_INT(1, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_VCU, (uint8_t)(0x40u + node), 4u, 0u, payload,
                                            sizeof(payload), 0));
    }
    TEST_ASSERT_EQUAL_INT(0, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_VCU, 0x50u, 4u, 0u, payload, sizeof(payload), 0));
    TEST_ASSERT_EQUAL_INT(1, bp_tp_send(&g_bp_mgr, 7u, 1u, NODE_VCU, 0x40u, 4u, 0u, payload, sizeof(payload), 0));
}

/* ---------------------------------------------------------------------------------------- */
/* Ext-id CAN routing (can.c ext-id RX handler -> busproto / ninebot)                       */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           Through the CAN RX path: PF 0xEF goes to the Ninebot framer, PF 0xEC to TP
 * \note            The ext-id RX handler: PF byte 0xEF -> the Ninebot framer (0, sa, da, data,
 *                      dlc), every other PF -> the TP dispatcher on g_bp_mgr.
 *                      `0x18EF163D 01 01 1a 01` -> `0x18EF3D16 01 04 1a 7a`; RTS -> CTS.
 */
static void test_ext_id_routing_through_can_rx(void) {
    const uint8_t read[] = {0x01u, CMD_READ, REG_VERSION, 0x01u};
    const uint8_t reply[] = {0x01u, 0x04u, REG_VERSION, 0x7Au};
    const uint8_t rts[8] = {TP_RTS, 6u, 0u, 1u, 0xFFu, 0x00u, 0xEFu, 0x00u};
    const tst_can_frame_t* frame;

    g_in2.reg[REG_VERSION] = 0x397Au;
    TEST_ASSERT_TRUE(tst_can_rx_inject(ID_EF_TO_VCU, 1u, read, sizeof(read)));
    TEST_ASSERT_TRUE(tst_can_rx_inject(ID_CM_TO_VCU, 1u, rts, sizeof(rts)));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_can_rx_deliver());
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_flush());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_EQUAL_HEX32(ID_EF_TO_PEER, frame->id);
    TEST_ASSERT_EQUAL_UINT8(sizeof(reply), frame->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(reply, frame->data, sizeof(reply));
    prv_assert_cm(1u, ID_CM_TO_PEER, TP_CTS, 1u, 1u, 0xFFu);
}

/* ---------------------------------------------------------------------------------------- */
/* Report manager: descriptor table, registration, state frames                              */
/* ---------------------------------------------------------------------------------------- */

/**
 * \brief           The descriptor table holds the expected entries (key, width, g_scr index)
 * \note            38 x 16 B (key u32, width u32, ptr into the index bytes, getter), count
 *                      0x26; the manager init stores both. The getter returns the u32 at
 *                      g_scr + idx * 2.
 */
static void test_descriptor_table_matches_flash_layout(void) {
    const bp_sig_desc_t* tbl = (const bp_sig_desc_t*)(uintptr_t)prv_mgr()->desc_table;
    uint32_t idx;

    TEST_ASSERT_NOT_NULL(tbl);
    for (idx = 0u; idx < SIG_COUNT; ++idx) {
        uint8_t scr = k_sig_expected[idx].scr;

        TEST_ASSERT_EQUAL_HEX32(k_sig_expected[idx].key, tbl[idx].key);
        TEST_ASSERT_EQUAL_UINT8(k_sig_expected[idx].width, tbl[idx].width);
        TEST_ASSERT_EQUAL_HEX8(scr, tbl[idx].scr_idx);
    }
}

/**
 * \brief           A good registration: 0xFD `<id> 00`, node record filled, 0xFE packs signals
 * \note            bp_node_register: resolve (bp_rpt_resolve -> bp_resolve_records, seq =
 *                      key[1] * 20) then the descriptor lookup; record = {id, handle[+2], 0x26 x 1}.
 *                      bp_state_notify packs LSB first after the key byte.
 *                      `05 03 | 7E765289 w16 | 42416F9F w1` -> 0xFD `05 00`, then 0xFE with the
 *                      packed signal values.
 */
static void test_register_and_state_frame(void) {
    const uint32_t keys[] = {SIG_POWER_HALF, SIG_POWER_SIGN};
    const uint8_t widths[] = {16u, 1u};
    const uint8_t ok[] = {0x05u, 0x00u};
    const uint8_t state[] = {0x05u, 0x34u, 0x12u, 0x01u};
    uint8_t key[KEY_MAX];
    uint8_t len = prv_reg_key(key, 0x05u, 0x03u, keys, widths, 2u);
    tst_nb_frame_t frame;
    uint32_t idx;

    bp_node_register(key, len);
    TEST_ASSERT_EQUAL_UINT8(0x05u, g_bp_node.node_id);
    TEST_ASSERT_EQUAL_UINT8(60u, g_bp_node.report_period); /* low byte of 3 * 20 */
    for (idx = 0u; idx < BP_NODE_CAP_SLOTS; ++idx) {
        TEST_ASSERT_EQUAL_UINT8(1u, g_bp_node.cap[idx]);
    }
    TEST_ASSERT_NOT_EQUAL(0u, g_bp_rptr.last_node_handle);
    g_scr.power_half = 0x1234u;
    g_scr.power_sign = 0x0003u; /* only bit 0 is packed */
    tst_stack_paint(0xA5u);     /* the state frame scratch must be cleared */
    bp_state_notify((void*)(uintptr_t)g_bp_rptr.last_node_handle);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, NB_CMD_FD, NB_ARG_FD, sizeof(ok), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(ok, frame.body, sizeof(ok));
    prv_usart_frame(1u, NB_CMD_FE, NB_ARG_FE, sizeof(state), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(state, frame.body, sizeof(state));
}

/**
 * \brief           Packer quirk: a narrow field across a byte boundary keeps only its low bits
 * \note            The bit packer touches `1 + width / 8` bytes from the field's start byte:
 *                      a 4-bit field at bit offset 5 writes bits 5..7 only, and the serializer still
 *                      advances the cursor by 4. Five 1-bit fields + a 4-bit 0xF = `09 ff 00`
 *                      (17 bits -> 3 bytes). The registration answers `09 00`.
 */
static void test_state_frame_straddling_field_loses_high_bits(void) {
    const uint32_t keys[] = {0x42416F9Fu, 0x17B28D98u, 0xCFBC0A29u, 0x88E7BACCu, 0x0C99E9E5u, SIG_TELEM_3C};
    const uint8_t widths[] = {1u, 1u, 1u, 1u, 1u, 4u};
    const uint8_t scr[] = {0x03u, 0x1Fu, 0x39u, 0x38u, 0x0Bu, 0x1Eu};
    const uint8_t state[] = {0x09u, 0xFFu, 0x00u};
    uint8_t key[KEY_MAX];
    uint8_t len = prv_reg_key(key, 0x09u, 0x01u, keys, widths, sizeof(widths));
    tst_nb_frame_t frame;
    uint32_t idx;

    for (idx = 0u; idx < sizeof(scr); ++idx) {
        g_scr.reg[scr[idx]] = 0xFFFFu;
    }
    bp_node_register(key, len);
    bp_state_notify((void*)(uintptr_t)g_bp_rptr.last_node_handle);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(1u, NB_CMD_FE, NB_ARG_FE, sizeof(state), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(state, frame.body, sizeof(state));
}

/**
 * \brief           Width mismatch: 0xFD `<id> 01`, no node record
 * \note            bp_resolve_records: `desc.width != key[off + 4]` -> -1; bp_rpt_resolve then
 *                      emits the 0xFD list (scratch{id, 1}, node.rec, 0). The descriptor lookup
 *                      still finds the node (allocated), so bp_node_register looks up id -1 =
 *                      0xFF: no handle. `06 01 | 7E765289 w8` -> `06 01`.
 */
static void test_register_width_mismatch(void) {
    const uint32_t keys[] = {SIG_POWER_HALF};
    const uint8_t widths[] = {8u};
    const uint8_t nak[] = {0x06u, 0x01u};
    uint8_t key[KEY_MAX];
    uint8_t len = prv_reg_key(key, 0x06u, 0x01u, keys, widths, 1u);
    tst_nb_frame_t frame;

    bp_node_register(key, len);
    TEST_ASSERT_EQUAL_UINT32(0u, g_bp_rptr.last_node_handle);
    TEST_ASSERT_EQUAL_UINT8(0u, g_bp_node.cap[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, NB_CMD_FD, NB_ARG_FD, sizeof(nak), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(nak, frame.body, sizeof(nak));
}

/**
 * \brief           Unknown key: 0xFD `<id> 01` + the keys that did resolve, LE
 * \note            bp_resolve_records keeps going after a miss (result -1); the 0xFD list with
 *                      a record list appends each record's u32 key (payload 2 + 4 n).
 *                      `07 01 | 7E765289 w16 | 11223344 w8` -> `07 01 89 52 76 7e`.
 */
static void test_register_unknown_key_lists_resolved(void) {
    const uint32_t keys[] = {SIG_POWER_HALF, 0x11223344u};
    const uint8_t widths[] = {16u, 8u};
    const uint8_t nak[] = {0x07u, 0x01u, 0x89u, 0x52u, 0x76u, 0x7Eu};
    uint8_t key[KEY_MAX];
    uint8_t len = prv_reg_key(key, 0x07u, 0x01u, keys, widths, 2u);
    tst_nb_frame_t frame;

    bp_node_register(key, len);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, NB_CMD_FD, NB_ARG_FD, sizeof(nak), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(nak, frame.body, sizeof(nak));
}

/**
 * \brief           A key length that is not 2 + 5 n: 0xFD `<id> 01`
 * \note            bp_resolve_records: `(len - 2) % 5 != 0` -> -1 before any record.
 *                      8-byte key -> `08 01`.
 */
static void test_register_bad_length(void) {
    const uint32_t keys[] = {SIG_POWER_HALF};
    const uint8_t widths[] = {16u};
    const uint8_t nak[] = {0x08u, 0x01u};
    uint8_t key[KEY_MAX];
    uint8_t len = prv_reg_key(key, 0x08u, 0x01u, keys, widths, 1u);
    tst_nb_frame_t frame;

    key[len] = 0u;
    bp_node_register(key, (uint8_t)(len + 1u));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, NB_CMD_FD, NB_ARG_FD, sizeof(nak), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(nak, frame.body, sizeof(nak));
}

/**
 * \brief           The node table holds 10 nodes; the 11th registration is refused
 * \note            bp_rpt_resolve: `node_count >= 10` -> 0xFD list (scratch{id, 1}) and -1,
 *                      checked before the lookup, so a known id is refused too.
 */
static void test_register_table_full(void) {
    const uint32_t keys[] = {SIG_POWER_SIGN};
    const uint8_t widths[] = {1u};
    const uint8_t nak[] = {0x0Au, 0x01u};
    uint8_t key[KEY_MAX];
    uint8_t len;
    uint8_t node;
    tst_nb_frame_t frame;

    for (node = 0u; node < 10u; ++node) {
        len = prv_reg_key(key, node, 1u, keys, widths, 1u);
        (void)prv_mgr()->vt_resolve(key, len);
    }
    TEST_ASSERT_EQUAL_UINT8(10u, prv_mgr()->node_count);
    (void)tst_usart_drain();
    tst_fakes_reset();
    len = prv_reg_key(key, 0x0Au, 1u, keys, widths, 1u);
    TEST_ASSERT_EQUAL_INT(-1, prv_mgr()->vt_resolve(key, len));
    key[0] = 0x03u;
    TEST_ASSERT_EQUAL_INT(-1, prv_mgr()->vt_resolve(key, len));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, NB_CMD_FD, NB_ARG_FD, sizeof(nak), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(nak, frame.body, sizeof(nak));
}

/**
 * \brief           register_obj fires the object's +0x04 word; broadcast key 0xFF its +0x0C
 * \note            register_obj: node.obj = obj, then calls the word at obj + 4. Resolve with
 *                      key 0xFF calls the word at obj + 0x0C for every node with an object.
 *                      Nothing in the firmware registers an object.
 */
static void test_register_obj_and_broadcast_slots(void) {
    const uint32_t keys[] = {SIG_POWER_SIGN};
    const uint8_t widths[] = {1u};
    uint8_t key[KEY_MAX];
    uint8_t len = prv_reg_key(key, 0x05u, 1u, keys, widths, 1u);

    (void)prv_mgr()->vt_resolve(key, len);
    TEST_ASSERT_EQUAL_INT(-1, prv_mgr()->vt_register_obj(0x06u, (void*)(uintptr_t)k_test_obj));
    TEST_ASSERT_EQUAL_INT(0, prv_mgr()->vt_register_obj(0x05u, (void*)(uintptr_t)k_test_obj));
    TEST_ASSERT_EQUAL_UINT32(1u, g_obj_calls[1]);
    TEST_ASSERT_EQUAL_UINT32(0u, g_obj_calls[4]);
    key[0] = 0xFFu;
    (void)prv_mgr()->vt_resolve(key, len);
    TEST_ASSERT_EQUAL_UINT32(1u, g_obj_calls[3]);
}

/**
 * \brief           0xFC command: flag 1, reply `16 -> 23 cmd 04 arg FC` "abcdef0123" + 6 x 00
 * \note            bp_handle_fc: data[0] == 0xFC -> mgr vt[+0x14](1) (notify), then vt[+0x04]
 *                      (emit, cmd 4 arg 0xFC) with the 16-byte reply payload.
 *                      On the wire: `5a a5 10 16 23 04 fc 61 62 63 64 65 66 30 31 32 33 00..`.
 */
static void test_fc_command_reply(void) {
    const uint8_t expect[16] = {'a', 'b', 'c', 'd', 'e', 'f', '0', '1', '2', '3', 0u, 0u, 0u, 0u, 0u, 0u};
    bp_frame_t cmd;
    tst_nb_frame_t frame;

    memset(&cmd, 0, sizeof(cmd));
    cmd.data[0] = 0xFBu;
    bp_handle_fc(NODE_APP, 0x02u, &cmd);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_mgr()->flag);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    cmd.data[0] = 0xFCu;
    bp_handle_fc(NODE_APP, 0x02u, &cmd);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_mgr()->flag);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, NB_CMD_FC, NB_ARG_FC, sizeof(expect), &frame);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_init_builds_manager);
    RUN_TEST(test_init_sends_announce);
    RUN_TEST(test_node_accept_destination_set);
    RUN_TEST(test_rts_to_foreign_node_is_ignored);
    RUN_TEST(test_frame_bcast_layout);
    RUN_TEST(test_rts_is_answered_with_cts);
    RUN_TEST(test_rts_packet_count_mismatch_aborts);
    RUN_TEST(test_rts_with_other_pgn_is_ignored);
    RUN_TEST(test_transfer_completes_and_dispatches_on_can);
    RUN_TEST(test_transfer_for_app_is_forwarded_to_usart);
    RUN_TEST(test_transfer_bad_checksum_aborts);
    RUN_TEST(test_transfer_out_of_sequence_aborts);
    RUN_TEST(test_dt_without_session_is_ignored);
    RUN_TEST(test_rx_cts_timeout_aborts);
    RUN_TEST(test_rx_dt_gap_timeout_aborts);
    RUN_TEST(test_tx_transfer_streams_after_cts);
    RUN_TEST(test_tx_eoma_closes_session);
    RUN_TEST(test_tx_missing_eoma_aborts);
    RUN_TEST(test_tx_rts_timeout_aborts);
    RUN_TEST(test_tx_cts_window_needs_hold_between_windows);
    RUN_TEST(test_tx_peer_abort_idles_session);
    RUN_TEST(test_tp_send_bounds_and_reverse_pair);
    RUN_TEST(test_tp_send_runs_out_of_sessions);
    RUN_TEST(test_ext_id_routing_through_can_rx);
    RUN_TEST(test_descriptor_table_matches_flash_layout);
    RUN_TEST(test_register_and_state_frame);
    RUN_TEST(test_state_frame_straddling_field_loses_high_bits);
    RUN_TEST(test_register_width_mismatch);
    RUN_TEST(test_register_unknown_key_lists_resolved);
    RUN_TEST(test_register_bad_length);
    RUN_TEST(test_register_table_full);
    RUN_TEST(test_register_obj_and_broadcast_slots);
    RUN_TEST(test_fc_command_reply);
    tst_exit(UNITY_END());
    return 0;
}
