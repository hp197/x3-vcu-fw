/**
 * \file            test_ninebot.c
 * \brief           Ninebot dispatcher (ninebot.c)
 * \note            Replies are captured at the BSP boundary: USART frames at the DMA1
 *                      channel 7 arm, CAN frames at can_message_transmit (tst_bsp_fakes.h).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "busproto.h"
#include "buzzer.h"
#include "can.h"
#include "control.h" /* VCU_ADC_SLOT_* for the 0x56 input-test gate */
#include "ninebot.h"
#include "report.h"
#include "tst_bsp_fakes.h"
#include "tst_control.h" /* tst_control_adc_freeze / tst_control_adc_set */
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/* Public in ninebot.c but not in a header: declared here to test it. */
uint32_t nb_report_frame(uint8_t n, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg, const uint8_t* payload);

/* Node addresses and commands as they appear on the wire. */
#define NODE_VCU         0x16u
#define NODE_APP         0x23u
#define NODE_INFO        0x3Du
#define CMD_READ         0x01u
#define CMD_WRITE_REPLY  0x02u
#define CMD_WRITE_SILENT 0x03u
#define CMD_READ_REPLY   0x04u
#define CMD_STATUS_REPLY 0x05u
#define CMD_IDENT_READ   0x41u
#define CMD_IDENT_REPLY  0x43u
#define REG_SERIAL       0x10u
#define REG_VERSION      0x1Au
#define REG_SPEED_LIMIT  0x46u
#define REG_AUTO_OFF     0x49u
#define ARG_APP_START    0x84u
#define ARG_MODE_REPORT  0x25u
#define ARG_ANNOUNCE     0xD0u
#define SERIAL_LEN       14u
#define CAN_REPLY_ID     0x18EF3D16u /* prio 6, PF 0xEF, dst 0x3D, src 0x16 */
#define CAN_REPLY_CTL    0x01u
#define RX_SETTLE_US     3000u /* > 20 bytes at 115200 Bd */

/**
 * \brief           Fill registers `first..` of the register file with a byte pattern
 * \param[in]       first: first register
 * \param[in]       bytes: number of bytes
 * \param[in]       seed: value of the first byte (incremented per byte)
 */
static void prv_fill_regs(uint8_t first, uint8_t bytes, uint8_t seed) {
    volatile uint8_t* dst = (volatile uint8_t*)&g_in2.reg[first];
    uint8_t idx;

    for (idx = 0u; idx < bytes; ++idx) {
        dst[idx] = (uint8_t)(seed + idx);
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
 * \brief           Assert the header of a parsed reply
 * \param[in]       frame: parsed frame
 * \param[in]       dst: expected destination
 * \param[in]       cmd: expected command
 * \param[in]       arg: expected argument
 * \param[in]       len: expected body length
 */
static void prv_assert_header(const tst_nb_frame_t* frame, uint8_t dst, uint8_t cmd, uint8_t arg, uint8_t len) {
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, frame->src);
    TEST_ASSERT_EQUAL_HEX8(dst, frame->dst);
    TEST_ASSERT_EQUAL_HEX8(cmd, frame->cmd);
    TEST_ASSERT_EQUAL_HEX8(arg, frame->arg);
    TEST_ASSERT_EQUAL_UINT8(len, frame->len);
}

/* Further wire values (the dispatcher / the node handlers). */
#define NODE_CTRL       0x04u
#define NODE_SUB02      0x02u
#define NODE_SUB06      0x06u
#define NODE_GW7        0x07u
#define NODE_INFO_3E    0x3Eu
#define CMD_WRITE_RAW   0x04u
#define CMD_IDENT_REPLY 0x43u
#define CMD_PAIR        0x18u
#define CMD_VIN         0x19u
#define REG_PASSCODE    0x71u
#define CAN_CTL         0x01u /* data[0] of every single-frame Ninebot CAN frame */
#define CAN_ID_PF_EF    0x18EF0000u
#define CAN_PUMP_ROUNDS 24u
#define PAIR_MOD        1000000u
#define TOKEN_C0        2u
#define TOKEN_C1        3u
#define TOKEN_C2        5u
#define SALT_MUL        0x11u
#define KEEPALIVE_TICKS 300u
#define AUTO_OFF_SCALE  6000u

/** \brief The registers a cmd 2/3 write may change (register-write permission table). */
static const uint8_t prv_writable[] = {0x1C, 0x1D, 0x1E, 0x1F, 0x2E, 0x2F, 0x42, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
                                       0x4F, 0x5B, 0x5C, 0x5D, 0x61, 0x70, 0x71, 0x72, 0x74, 0x75, 0xE0, 0xE1, 0xE9};

/**
 * \brief           Reflected IEEE CRC-32, written from the algorithm (independent of nb_crc32)
 * \param[in]       data: bytes
 * \param[in]       len: number of bytes
 * \return          the CRC-32
 */
static uint32_t prv_crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t idx;
    uint32_t bit;

    for (idx = 0u; idx < len; ++idx) {
        crc ^= data[idx];
        for (bit = 0u; bit < 8u; ++bit) {
            crc = ((crc & 1u) != 0u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return ~crc;
}

/**
 * \brief           Put a 32-bit value little-endian into a buffer
 * \param[out]      dst: 4 bytes
 * \param[in]       val: value
 */
static void prv_put_u32(uint8_t* dst, uint32_t val) {
    dst[0] = (uint8_t)val;
    dst[1] = (uint8_t)(val >> 8);
    dst[2] = (uint8_t)(val >> 16);
    dst[3] = (uint8_t)(val >> 24);
}

/**
 * \brief           Serial / VIN checksum (cmd 0x18 / 0x19 paths)
 * \note            acc = (old[i] + acc) * body[i] over `count` bytes, acc += salt * 0x11,
 *                      |acc| as a signed int, then % 1000000.
 * \param[in]       old: the stored bytes the checksum walks
 * \param[in]       body: the proposed bytes
 * \param[in]       count: byte count
 * \return          the value the request word must carry
 */
static uint32_t prv_pair_checksum(const volatile uint8_t* old, const uint8_t* body, uint32_t count) {
    uint32_t acc = 0u;
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        acc = ((uint32_t)old[idx] + acc) * (uint32_t)body[idx];
    }
    acc = g_in2.ident_word[2] * SALT_MUL + acc;
    if ((acc & 0x80000000u) != 0u) {
        acc = 0u - acc;
    }
    return acc % PAIR_MOD;
}

/**
 * \brief           Seed the three VCU-compat words and build a valid 8-byte session token
 * \note            word0 == ~(c0 + c1 + c2), word1 == ~(c0 * c1 * c2).
 * \param[out]      body: at least 8 bytes
 */
static void prv_token(uint8_t* body) {
    g_in2.ident_word[0] = TOKEN_C0;
    g_in2.ident_word[1] = TOKEN_C1;
    g_in2.ident_word[2] = TOKEN_C2;
    prv_put_u32(body, ~(TOKEN_C0 + TOKEN_C1 + TOKEN_C2));
    prv_put_u32(&body[4], ~(TOKEN_C0 * TOKEN_C1 * TOKEN_C2));
}

/**
 * \brief           Pump the CAN TX ring into the capture (several rounds, one frame each)
 * \return          frames captured so far
 */
static uint32_t prv_can_pump(void) {
    uint32_t round;

    for (round = 0u; round < CAN_PUMP_ROUNDS; ++round) {
        vcu_can_poll();
    }
    return tst_can_tx_count();
}

/**
 * \brief           Count the drained USART frames sent to `dst`
 * \param[in]       sent: frames drained
 * \param[in]       dst: destination node
 * \return          matching frames
 */
static uint32_t prv_count_to(uint32_t sent, uint8_t dst) {
    tst_nb_frame_t frame;
    uint32_t idx;
    uint32_t hits = 0u;

    for (idx = 0u; (idx < sent) && (idx < TST_USART_CAPTURE_MAX); ++idx) {
        prv_usart_frame(idx, &frame);
        if (frame.dst == dst) {
            ++hits;
        }
    }
    return hits;
}

/**
 * \brief           Find the first drained USART frame with `dst`, `cmd`, `arg`
 * \param[in]       sent: frames drained
 * \param[in]       dst: destination node
 * \param[in]       cmd: command
 * \param[in]       arg: argument
 * \param[out]      frame: the frame, valid when found
 * \return          1 if found
 */
static uint8_t prv_find(uint32_t sent, uint8_t dst, uint8_t cmd, uint8_t arg, tst_nb_frame_t* frame) {
    uint32_t idx;
    uint8_t found = 0u;

    for (idx = 0u; (idx < sent) && (idx < TST_USART_CAPTURE_MAX) && (found == 0u); ++idx) {
        prv_usart_frame(idx, frame);
        if ((frame->dst == dst) && (frame->cmd == cmd) && (frame->arg == arg)) {
            found = 1u;
        }
    }
    return found;
}

/**
 * \brief           Assert one captured single-frame Ninebot CAN frame
 * \param[in]       index: capture index
 * \param[in]       dst: destination node (id bits 15:8)
 * \param[in]       src: source node (id bits 7:0)
 * \param[in]       cmd: data[1]
 * \param[in]       arg: data[2]
 * \param[in]       payload: data[3..] (may be NULL when `len` is 0)
 * \param[in]       len: payload length
 */
static void prv_assert_can(uint32_t index, uint8_t dst, uint8_t src, uint8_t cmd, uint8_t arg, const uint8_t* payload,
                           uint8_t len) {
    const tst_can_frame_t* can_frame = tst_can_tx_get(index);

    TEST_ASSERT_NOT_NULL_MESSAGE(can_frame, "no CAN frame captured at this index");
    TEST_ASSERT_EQUAL_UINT8(1u, can_frame->ext);
    TEST_ASSERT_EQUAL_HEX32(CAN_ID_PF_EF | ((uint32_t)dst << 8) | src, can_frame->id);
    TEST_ASSERT_EQUAL_UINT8(len + 3u, can_frame->dlc);
    TEST_ASSERT_EQUAL_HEX8(CAN_CTL, can_frame->data[0]);
    TEST_ASSERT_EQUAL_HEX8(cmd, can_frame->data[1]);
    TEST_ASSERT_EQUAL_HEX8(arg, can_frame->data[2]);
    if (len > 0u) {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, &can_frame->data[3], len);
    }
}

/**
 * \brief           Whether an identity page (0x0801F000 / 0x0801F400) was erased after op `since`
 * \param[in]       since: flash op count before the request
 * \return          1 if a 1 KB identity-page erase was recorded, else 0
 */
static uint8_t prv_identity_erased_since(uint32_t since) {
    const tst_flash_op_t* op;
    uint8_t found = 0u;
    uint32_t idx;

    for (idx = since; idx < tst_flash_op_count(); ++idx) {
        op = tst_flash_op_get(idx);
        if ((op != NULL) && (op->kind == TST_FLASH_ERASE) && ((op->addr == 0x0801F000u) || (op->addr == 0x0801F400u))) {
            found = 1u;
        }
    }
    return found;
}

/**
 * \brief           Valid 10-byte body of a reg 0x46 write (CRC over body[0], the key, body[9])
 * \note            The 16-byte RAM key is the erased flash cell (16 x 0xFF) after nb_key_load.
 * \param[out]      body: 10 bytes
 * \param[in]       first: body[0]
 * \param[in]       flag: body[9] (frame[0x0E])
 */
static void prv_speed_body(uint8_t* body, uint8_t first, uint8_t flag) {
    uint8_t buf[18];
    uint32_t crc;

    memset(buf, 0xFF, sizeof(buf));
    buf[0] = first;
    buf[17] = flag;
    crc = prv_crc32(buf, sizeof(buf));
    memset(body, 0, 10u);
    body[0] = first;
    body[5] = (uint8_t)(crc >> 24);
    body[6] = (uint8_t)(crc >> 16);
    body[7] = (uint8_t)(crc >> 8);
    body[8] = (uint8_t)crc;
    body[9] = flag;
}

/**
 * \brief           Send a cmd 0x02 write and return the status of its `05 <arg> st 00` reply
 * \param[in]       arg: first register
 * \param[in]       body: body
 * \param[in]       len: body length
 * \return          the status byte
 */
static uint8_t prv_write_status(uint8_t arg, const uint8_t* body, uint8_t len) {
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, arg, body, len);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(tst_usart_tx_count() - 1u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_STATUS_REPLY, arg, 2u);
    TEST_ASSERT_EQUAL_HEX8(0u, frame.body[1]);
    return frame.body[0];
}

/**
 * \brief           Clear the captures between two requests of one test (fakes stay record-only)
 */
static void prv_recapture(void) {
    tst_fakes_reset();
    tst_flash_forward(0u);
    tst_can_tx_forward(0u);
}

/**
 * \brief           Fresh boot state for every test: product RAM, fakes, USART2 + its DMA, the key
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_flash_forward(0u);  /* identity commits record only               */
    tst_can_tx_forward(0u); /* CAN frames captured, mailbox 0 reported free */
    vcu_usart_init();
    nb_key_load(); /* boot copy of the 16-byte key */
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* -- frame format ------------------------------------------------------------------------- */

/**
 * \brief           The test helper builds the reference frame
 */
static void test_frame_build_reference_vector(void) {
    static const uint8_t expect[] = {0x5A, 0xA5, 0x01, 0x3D, 0x16, 0x01, 0x10, 0x0E, 0x8C, 0xFF};
    const uint8_t body[] = {SERIAL_LEN};
    uint8_t wire[16];

    TEST_ASSERT_EQUAL_UINT16(sizeof(expect), tst_nb_build(wire, sizeof(wire), NODE_INFO, NODE_VCU, CMD_READ, REG_SERIAL,
                                                          body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, wire, sizeof(expect));
}

/**
 * \brief           A corrupted checksum is detected by the parser helper
 */
static void test_frame_parse_flags_bad_checksum(void) {
    const uint8_t body[] = {SERIAL_LEN};
    tst_nb_frame_t frame;
    uint8_t wire[16];
    uint16_t len = tst_nb_build(wire, sizeof(wire), NODE_INFO, NODE_VCU, CMD_READ, REG_SERIAL, body, sizeof(body));

    wire[len - 1u] ^= 0xFFu;
    TEST_ASSERT_TRUE(tst_nb_parse(wire, len, &frame));
    TEST_ASSERT_FALSE(frame.checksum_ok);
}

/**
 * \brief           A request received on USART2 is parsed and answered with a valid frame
 * \note            Bytes go in from the bus side (x3emu USART2 model, RX DMA), the product
 *                      parser (vcu_usart_poll) dispatches them.
 */
static void test_usart_rx_request_is_answered(void) {
    const uint8_t body[] = {1u};
    tst_nb_frame_t frame;
    uint8_t wire[16];
    uint16_t len = tst_nb_build(wire, sizeof(wire), NODE_INFO, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));

    tst_emu_usart_rx(wire, len);
    tst_delay_us(RX_SETTLE_US);
    vcu_usart_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_READ_REPLY, REG_VERSION, 1u);
}

/**
 * \brief           A request with a bad checksum is dropped by the receiver: no reply
 */
static void test_usart_rx_bad_checksum_is_ignored(void) {
    const uint8_t body[] = {1u};
    uint8_t wire[16];
    uint16_t len = tst_nb_build(wire, sizeof(wire), NODE_INFO, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));

    wire[len - 1u] ^= 0xFFu;
    tst_emu_usart_rx(wire, len);
    tst_delay_us(RX_SETTLE_US);
    vcu_usart_poll();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/* -- register read ---------------------------------------------------------------------------- */

/**
 * \brief           Reg 0x10 read with count 14 returns the 14-byte serial, not 14 words
 * \note            Reply length = body[0] (bytes), data at IN2 + arg*2, cmd 0x04, arg =
 *                      request arg: 14 B.
 */
static void test_reg_read_count_is_bytes(void) {
    const uint8_t body[] = {SERIAL_LEN};
    tst_nb_frame_t frame;

    prv_fill_regs(REG_SERIAL, SERIAL_LEN + 2u, 0x30u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_READ, REG_SERIAL, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_READ_REPLY, REG_SERIAL, SERIAL_LEN);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t*)(const volatile void*)&g_in2.reg[REG_SERIAL], frame.body, SERIAL_LEN);
}

/**
 * \brief           Reg 0x1A read with count 1 returns one byte (the low byte of the register)
 * \note            The reply body is `7a`, 1 byte.
 */
static void test_reg_read_single_byte(void) {
    const uint8_t body[] = {1u};
    tst_nb_frame_t frame;

    g_in2.reg[REG_VERSION] = 0x397Au;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_READ_REPLY, REG_VERSION, 1u);
    TEST_ASSERT_EQUAL_HEX8(0x7Au, frame.body[0]);
}

/**
 * \brief           Over CAN, a 1-byte read is one 0x18EF<dst><src> frame `01 04 1A <byte>`
 * \note            report_ninebot(0x16, src, count, 4, arg, data); <= 5 payload bytes take
 *                      the single-frame builder (prio 6).
 */
static void test_reg_read_over_can_single_frame(void) {
    const uint8_t body[] = {1u};
    const tst_can_frame_t* can_frame;

    g_in2.reg[REG_VERSION] = 0x397Au;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_tx_count());
    can_frame = tst_can_tx_get(0u);
    TEST_ASSERT_NOT_NULL(can_frame);
    TEST_ASSERT_EQUAL_UINT8(1u, can_frame->ext);
    TEST_ASSERT_EQUAL_HEX32(CAN_REPLY_ID, can_frame->id);
    TEST_ASSERT_EQUAL_UINT8(4u, can_frame->dlc);
    TEST_ASSERT_EQUAL_HEX8(CAN_REPLY_CTL, can_frame->data[0]);
    TEST_ASSERT_EQUAL_HEX8(CMD_READ_REPLY, can_frame->data[1]);
    TEST_ASSERT_EQUAL_HEX8(REG_VERSION, can_frame->data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x7Au, can_frame->data[3]);
}

/**
 * \brief           cmd 0x41 identity reads: 0x10/0xDA reply body[0] bytes, 0x17 replies 2 bytes
 * \note            vcu_usart_report_send(.., len = f[5], 0x43, 0x10 / 0xDA, IN2+0x1B4) and
 *                      (.., len 2, 0x43, 0x17, IN2+0x2E). The 0x17 reply is `0x43 0x17 5801`.
 */
static void test_ident_read_lengths(void) {
    const uint8_t body[] = {12u};
    tst_nb_frame_t frame;

    g_in2.f.serial_mark = 0x0158u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_IDENT_READ, 0x10u, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_IDENT_READ, 0xDAu, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_IDENT_READ, 0x17u, NULL, 0u);
    TEST_ASSERT_EQUAL_UINT32(3u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0x10u, 12u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t*)(const volatile void*)g_in2.ident, frame.body, 12u);
    prv_usart_frame(1u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0xDAu, 12u);
    prv_usart_frame(2u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0x17u, 2u);
    TEST_ASSERT_EQUAL_HEX8(0x58u, frame.body[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01u, frame.body[1]);
}

/* -- register write status reply ------------------------------------------------------------- */

/**
 * \brief           A permitted cmd 0x02 write is applied and answered `05 <arg> 00 00`
 * \note            Status sent as 2 bytes, cmd 5, arg = request arg: reg 0x49 = 7 -> `0000`.
 */
static void test_reg_write_ok_status_is_two_bytes(void) {
    const uint8_t body[] = {7u, 0u};
    const uint8_t expect[] = {0x00u, 0x00u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, REG_AUTO_OFF, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(7u, g_in2.reg[REG_AUTO_OFF]);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_STATUS_REPLY, REG_AUTO_OFF, 2u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
}

/**
 * \brief           A reg 0x46 write with a bad CRC is rejected with status `04 00`
 * \note            Status 4 is sent as `0400`.
 */
static void test_reg_write_rejected_status_is_two_bytes(void) {
    const uint8_t body[10] = {0};
    const uint8_t expect[] = {0x04u, 0x00u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, REG_SPEED_LIMIT, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_STATUS_REPLY, REG_SPEED_LIMIT, 2u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
}

/**
 * \brief           cmd 0x03 writes the register without any reply
 * \note            `if (f[3] == 2)` guards the reply.
 */
static void test_reg_write_cmd3_is_silent(void) {
    const uint8_t body[] = {9u, 0u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, REG_AUTO_OFF, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(9u, g_in2.reg[REG_AUTO_OFF]);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/* -- app start -------------------------------------------------------------------------------- */

/**
 * \brief           App start (0x23, cmd 2, arg 0x84): mode report, then `05 84 00 00`
 * \note            Commit mode (nb_commit_mode report, arg 0x25), IDLE -> start request,
 *                      then reply cmd 5, arg 0x84, 2 zero bytes: `0x05 0x84 0000`.
 */
static void test_app_start_reply_is_status(void) {
    const uint8_t expect[] = {0x00u, 0x00u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_WRITE_REPLY, ARG_APP_START, NULL, 0u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_WRITE_SILENT, ARG_MODE_REPORT, 2u);
    prv_usart_frame(1u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_STATUS_REPLY, ARG_APP_START, 2u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
}

/**
 * \brief           The app handler answers on USART even when the request came over CAN
 * \note            The app handler ignores the transport and calls the USART ring sender
 *                      vcu_usart_report_send directly.
 */
static void test_app_reply_ignores_transport(void) {
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_CAN, NODE_APP, NODE_VCU, CMD_WRITE_REPLY, ARG_APP_START, NULL, 0u);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(1u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_STATUS_REPLY, ARG_APP_START, 2u);
}

/* -- boot announce ---------------------------------------------------------------------------- */

/**
 * \brief           The busproto boot announce 0xD0 carries the 16-bit value 1: `01 00`
 * \note            vcu_usart_report_send(.., 0x16, 0x23, len 2, cmd 3, arg 0xD0, &1): the
 *                      frame is `0x16 0x23 0x03 0xd0 0100`.
 */
static void test_boot_announce_body(void) {
    const uint8_t expect[] = {0x01u, 0x00u};
    tst_nb_frame_t frame;
    uint32_t sent;
    uint32_t idx;
    uint8_t found = 0u;

    tst_stack_paint(0xA5u); /* an uninitialised stack byte must not end up in body[1] */
    vcu_busproto_init();
    sent = tst_usart_drain();
    for (idx = 0u; idx < sent; ++idx) {
        prv_usart_frame(idx, &frame);
        if (frame.arg == ARG_ANNOUNCE) {
            prv_assert_header(&frame, NODE_APP, CMD_WRITE_SILENT, ARG_ANNOUNCE, 2u);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
            found = 1u;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "no 0xD0 announce frame");
}

/* -- primitives -------------------------------------------------------------------------------- */

/**
 * \brief           nb_crc32 is the reflected IEEE CRC-32 (check value of "123456789")
 * \note            Init 0xFFFFFFFF, poly 0xEDB88320, final ~.
 */
static void test_crc32_check_value(void) {
    static const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, nb_crc32(check, sizeof(check)));
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, nb_crc32(check, 0u));
}

/**
 * \brief           nb_keycmp returns the difference of the first mismatching bytes, 0 on a match
 * \note            a[i] - b[i], continuing while it is 0.
 */
static void test_keycmp_byte_difference(void) {
    static const uint8_t ref[] = {1u, 2u, 3u, 4u, 5u};
    static const uint8_t other[] = {1u, 2u, 3u, 9u, 5u};

    TEST_ASSERT_EQUAL_INT(0, nb_keycmp(ref, ref, sizeof(ref)));
    TEST_ASSERT_EQUAL_INT(4 - 9, nb_keycmp(ref, other, sizeof(ref)));
    TEST_ASSERT_EQUAL_INT(9 - 4, nb_keycmp(other, ref, sizeof(ref)));
    TEST_ASSERT_EQUAL_INT(0, nb_keycmp(ref, other, 3u));
    TEST_ASSERT_EQUAL_INT(0, nb_keycmp(ref, other, 0u));
}

/**
 * \brief           nb_commit_mode latches the screen id, reports it, and mode 0 clears the node record
 * \note            SCR+0x4A = mode, vcu_usart_report_send(0x16, 0x23, 2, 3, 0x25, SCR+0x4A),
 *                      mode 0 -> memset(g_bp_node, 0).
 */
static void test_commit_mode_reports_and_clears_node(void) {
    const uint8_t expect[] = {0x11u, 0x04u};
    tst_nb_frame_t frame;

    g_bp_node.node_id = 7u;
    nb_commit_mode(0x0411);
    TEST_ASSERT_EQUAL_HEX16(0x0411u, g_scr.ui_screen_u);
    TEST_ASSERT_EQUAL_UINT8(7u, g_bp_node.node_id);
    nb_commit_mode(0);
    TEST_ASSERT_EQUAL_UINT8(0u, g_bp_node.node_id);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_WRITE_SILENT, ARG_MODE_REPORT, 2u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
}

/**
 * \brief           nb_report_frame sends 8-byte chunks, advancing the id's arg by one word per 2 bytes
 * \note            n == 0 -> one empty frame, else chunks of <= 8 with arg' = arg + off/2
 *                      packed by report_pack_bitfields into the ext id.
 */
static void test_report_frame_chunks(void) {
    uint8_t payload[10];
    const tst_can_frame_t* first;
    const tst_can_frame_t* second;
    uint8_t idx;

    for (idx = 0u; idx < sizeof(payload); ++idx) {
        payload[idx] = (uint8_t)(0xA0u + idx);
    }
    TEST_ASSERT_EQUAL_UINT32(10u, nb_report_frame(10u, NODE_VCU, NODE_GW7, 0x57u, 0x10u, payload));
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_pump());
    first = tst_can_tx_get(0u);
    second = tst_can_tx_get(1u);
    TEST_ASSERT_EQUAL_UINT8(8u, first->dlc);
    TEST_ASSERT_EQUAL_UINT8(2u, second->dlc);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, first->data, 8u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&payload[8], second->data, 2u);
    TEST_ASSERT_EQUAL_HEX32(first->id + 4u, second->id);
    TEST_ASSERT_EQUAL_UINT32(0u, nb_report_frame(0u, NODE_VCU, NODE_GW7, 0x57u, 0x10u, payload));
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_pump());
    TEST_ASSERT_EQUAL_UINT8(0u, tst_can_tx_get(2u)->dlc);
}

/* -- routing ----------------------------------------------------------------------------------- */

/**
 * \brief           A NULL frame is ignored and a dst outside the node list gets no reply
 * \note            A dst other than 0/0x16 (local) or 2/4/6/7/0x23/0x3D..0x3F (gateway)
 *                      returns without a reply.
 */
static void test_unknown_dst_is_dropped(void) {
    const uint8_t body[] = {1u};

    vcu_ninebot_process(NULL, VCU_TRANSPORT_USART, 0u, 0u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, 0x55u, CMD_READ, REG_VERSION, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, 0x08u, CMD_READ, REG_VERSION, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
}

/**
 * \brief           dst 0 is handled as local, like dst 0x16
 * \note            dst 0 goes to the local handler.
 */
static void test_dst_zero_is_local(void) {
    const uint8_t body[] = {1u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, 0u, CMD_READ, REG_VERSION, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
}

/**
 * \brief           A local frame from an unknown source node gets no reply
 * \note            src 0x23, 2, 4, 6, 7, 0x3D..0x3F have handlers; others return.
 */
static void test_unknown_src_is_dropped(void) {
    const uint8_t body[] = {1u};

    tst_nb_request(VCU_TRANSPORT_USART, 0x30u, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, 0x01u, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, 0x40u, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
}

/**
 * \brief           A USART frame for another node is forwarded verbatim on CAN
 * \note            USART transport: report_ninebot(src, dst, len, cmd, arg, f+5). Nothing goes
 *                      back on USART.
 */
static void test_gateway_usart_to_can(void) {
    const uint8_t body[] = {0x11u, 0x22u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_INFO_3E, 0x01u, 0x1Au, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO_3E, NODE_INFO, 0x01u, 0x1Au, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           A CAN frame for another node is forwarded verbatim on USART
 * \note            CAN transport: vcu_usart_report_send(src, dst, len, cmd, arg, f+5).
 */
static void test_gateway_can_to_usart(void) {
    const uint8_t body[] = {0x33u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_APP, 0x03u, 0x40u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_HEX8(NODE_INFO, frame.src);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(0x03u, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(0x40u, frame.arg);
    TEST_ASSERT_EQUAL_UINT8(1u, frame.len);
    TEST_ASSERT_EQUAL_HEX8(0x33u, frame.body[0]);
}

/**
 * \brief           Gateway cmds 7..10 re-arm the CAN-active window, then forward
 * \note            `cmd - 7 < 4` -> IN1+3 = 1, then the forward.
 */
static void test_gateway_ota_cmds_rearm_can_active(void) {
    const uint8_t body[] = {0u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_INFO, 0x0Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.can_active_req);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    g_in1.can_active_req = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_INFO, 0x0Bu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.can_active_req);
}

/**
 * \brief           While the VCU's own OTA is busy, a forwarded cmd 7..10 is answered, not forwarded
 * \note            With g_fault_codes comm/OTA-busy bit0 set, the forward path replies on USART cmd
 *                      0x0B code 9, no body, src/dst swapped (src = the addressed node, dst = the
 *                      sender), and does NOT forward or re-arm can_active.
 */
static void test_gateway_ota_busy_replies(void) {
    const uint8_t body[] = {0u};
    tst_nb_frame_t frame;

    g_fault_codes.comm_fault = 1u; /* OTA busy */
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_INFO, 0x08u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.can_active_req); /* not re-armed */
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());      /* not forwarded on CAN */
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_HEX8(NODE_INFO, frame.src); /* reply src = request dst */
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame.dst);  /* reply dst = request src */
    TEST_ASSERT_EQUAL_HEX8(NB_CMD_OTA_REPLY, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(NB_OTA_BUSY_CODE, frame.arg);
    TEST_ASSERT_EQUAL_UINT8(0u, frame.len);
}

/**
 * \brief           Over CAN, node 7's `05 .. 82` frame beeps once when the ack beep is enabled
 * \note            src 7, cmd 5, body[0] 0x82, IN2+0x3C bit0 -> beep(2).
 */
static void test_gateway_can_bms_ack_beep(void) {
    const uint8_t body[] = {0x82u};

    tst_nb_request(VCU_TRANSPORT_CAN, NODE_GW7, NODE_APP, 0x05u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    g_in2.f.cfg_flags2 = CFG2_ACK_BEEP;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_GW7, NODE_APP, 0x05u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
}

/**
 * \brief           USART dst 7 cmd 0x2B arg 0x10 is forwarded twice (status pack + the generic forward)
 * \note            report_status(f+8) + report_ninebot, then cmd != 0x7A forwards again: the
 *                      frame is sent twice.
 */
static void test_gateway_dst7_status_double_send(void) {
    const uint8_t body[] = {0u, 0u, 0u, 1u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_GW7, 0x2Bu, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(2u, prv_can_pump());
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_GW7, 0x2Bu, 0x11u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(3u, prv_can_pump());
}

/**
 * \brief           USART dst 7 cmd 0x7A starts the remote countdown only at standstill in RIDE
 * \note            VS+2 == 0, !(STAT+0x126 bit3), STATE == 3 -> IN1+1 = 1 and
 *                      nb_commit_mode(0x0F); cmd 0x7A is never forwarded.
 */
static void test_gateway_dst7_countdown(void) {
    const uint8_t body[] = {0u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_GW7, 0x7Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.remote_countdown_req);
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_vs.speed_kmh = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_GW7, 0x7Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.remote_countdown_req);
    g_vs.speed_kmh = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_GW7, 0x7Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.remote_countdown_req);
    TEST_ASSERT_EQUAL_HEX16(0x000Fu, g_scr.ui_screen_u);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
}

/**
 * \brief           nb_reply_fc rebuilds the frame and dispatches it as a CAN request
 * \note            [total-3][src][dst][body1][body2][body+3..] then
 *                      vcu_ninebot_process(buf, 1): the reply goes out on CAN.
 */
static void test_reply_fc_dispatches_over_can(void) {
    const uint8_t body[] = {0u, CMD_READ, REG_VERSION, 1u};
    const uint8_t expect[] = {0x7Au};

    g_in2.reg[REG_VERSION] = 0x397Au;
    nb_reply_fc(NULL, NODE_INFO, NODE_VCU, body, (char)sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO, NODE_VCU, CMD_READ_REPLY, REG_VERSION, expect, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/* -- register write: permissions, ranges, gates, side effects ------------------------------------ */

/**
 * \brief           Exactly the registers of the permission table accept a cmd 0x03 write
 * \note            Every register written must be in the permission table (ninebot.c
 *                      k_nb_writable_regs). Reg 0x71 is staged in VS+0x56 instead. Reg 0x46 is
 *                      the CRC gate.
 */
static void test_reg_write_permission_table(void) {
    const uint8_t body[] = {0x5Au, 0x5Au};
    uint32_t reg;

    for (reg = 0u; reg < NB_REG_COUNT; ++reg) {
        uint8_t writable = 0u;
        uint32_t idx;

        if (reg == REG_SPEED_LIMIT) {
            continue;
        }
        for (idx = 0u; idx < sizeof(prv_writable); ++idx) {
            writable = (prv_writable[idx] == reg) ? 1u : writable;
        }
        g_in2.reg[reg] = 0xA5A5u;
        tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, (uint8_t)reg, body, sizeof(body));
        if ((writable != 0u) && (reg != REG_PASSCODE)) {
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(0x5A5Au, g_in2.reg[reg], "permitted register not written");
        } else {
            TEST_ASSERT_EQUAL_HEX16_MESSAGE(0xA5A5u, g_in2.reg[reg], "register written without permission");
        }
    }
}

/**
 * \brief           A run that crosses into a non-permitted register is denied as a whole (status 1)
 * \note            Any register not writable sets status 1 and nothing is copied.
 */
static void test_reg_write_partly_denied_range(void) {
    const uint8_t body[] = {1u, 0u, 2u, 0u};

    g_in2.reg[0x4Cu] = 0x1111u;
    TEST_ASSERT_EQUAL_HEX8(NB_ST_DENIED, prv_write_status(0x4Cu, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0x1111u, g_in2.reg[0x4Cu]);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_HEX8(NB_ST_OK, prv_write_status(0x4Bu, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_in2.reg[0x4Bu]);
    TEST_ASSERT_EQUAL_HEX16(0x0002u, g_in2.reg[0x4Cu]);
}

/**
 * \brief           An odd or empty write is rejected with status 4
 * \note            `(len & 1) == 0 && len != 0` else status 4.
 */
static void test_reg_write_odd_or_empty_rejected(void) {
    const uint8_t body[] = {7u, 0u, 0u};

    TEST_ASSERT_EQUAL_HEX8(NB_ST_REJECT, prv_write_status(REG_AUTO_OFF, body, 3u));
    TEST_ASSERT_EQUAL_HEX8(NB_ST_REJECT, prv_write_status(REG_AUTO_OFF, body, 1u));
    TEST_ASSERT_EQUAL_HEX8(NB_ST_REJECT, prv_write_status(REG_AUTO_OFF, NULL, 0u));
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.reg[REG_AUTO_OFF]);
}

/**
 * \brief           A write whose run passes register 0xFF is denied
 * \note            No register above 0xFF is writable (ninebot.c k_nb_writable_regs).
 */
static void test_reg_write_past_last_register_denied(void) {
    const uint8_t body[] = {1u, 0u, 2u, 0u};

    TEST_ASSERT_EQUAL_HEX8(NB_ST_DENIED, prv_write_status(0xFFu, body, sizeof(body)));
}

/**
 * \brief           The cmd 0x02 status reply goes back on CAN for a CAN request
 * \note            CAN transport -> report_ninebot(0x16, src, 2, 5, arg, &status).
 */
static void test_reg_write_status_over_can(void) {
    const uint8_t body[] = {7u, 0u};
    const uint8_t expect[] = {0u, 0u};

    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, REG_AUTO_OFF, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO, NODE_VCU, CMD_STATUS_REPLY, REG_AUTO_OFF, expect, sizeof(expect));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           A cmd 2/3 write beeps only with reg 0x1E bit0 set AND the FSM out of state 0/5
 * \note            `if (reg0x1E & 1)` then, unless fsm_state is 0 (IDLE) or 5 (SLEEP), beep(2).
 */
static void test_reg_write_ack_beep(void) {
    const uint8_t body[] = {7u, 0u};

    /* bit0 clear: no beep regardless of state */
    g_state_block.fsm_state = VCU_FSM_RIDE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, REG_AUTO_OFF, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    /* bit0 set, state 3 (RIDE): beeps */
    g_in2.f.cfg_flags2 = CFG2_ACK_BEEP;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);

    /* bit0 set, state 0 (IDLE): suppressed */
    g_lowglob.beep_seq_id = BEEP_NONE;
    g_state_block.fsm_state = VCU_FSM_IDLE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);

    /* bit0 set, state 5 (SLEEP): suppressed */
    g_state_block.fsm_state = VCU_FSM_SLEEP;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           A cmd 2/3 write to reg 0x75 toggles reg 0x1E (cfg_flags2) bit1 from body[0]
 * \note            Reg 0x75 is writable; after the register file write, body[0]==0 clears
 *                      reg 0x1E bit1 (0x0002), else sets it. The register itself is still
 *                      written. The bit is the ride-assist enable; nothing reads it.
 */
static void test_reg_write_ride_assist_toggles_reg1e(void) {
    const uint8_t set_body[] = {1u, 0u};
    const uint8_t clr_body[] = {0u, 0u};

    g_in2.f.cfg_flags2 = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, NB_REG_RIDE_ASSIST, set_body,
                   sizeof(set_body));
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_in2.reg[NB_REG_RIDE_ASSIST]); /* register file still written */
    TEST_ASSERT_BITS_HIGH(0x0002u, g_in2.f.cfg_flags2);              /* reg 0x1E bit1 set */

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, NB_REG_RIDE_ASSIST, clr_body,
                   sizeof(clr_body));
    TEST_ASSERT_BITS_LOW(0x0002u, g_in2.f.cfg_flags2); /* body[0]==0 clears it */
}

/**
 * \brief           An accepted write raises IN1+0x10; reg 0x71 is staged as the new passcode
 * \note            Reg 0x71 -> memcpy(VS+0x56, body, len), VS+0x41 = 1, then IN1+0x10 = 1 as
 *                      for every accepted write. The register itself is not written.
 */
static void test_reg_write_passcode_is_staged(void) {
    const uint8_t body[] = {0x56u, 0x78u, 0x34u, 0x12u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, REG_PASSCODE, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0x7856u, g_vs.passcode_new[0]);
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_vs.passcode_new[1]);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.passcode_set_pending);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.reg[REG_PASSCODE]);
}

/**
 * \brief           Reg 0x1F: a schedule-enable change pushes to the BMS and sets stop/cancel
 * \note            b4 changed -> IN1+0x28 = 1, then b4 set ->
 *                      VS+0x19 = 0, b4 clear -> VS+0x1F = 1; only b5 changed -> IN1+0x28 = 1.
 */
static void test_reg_side_effect_charge_schedule(void) {
    uint8_t body[2] = {(uint8_t)CHG_SCHED_EN, 0u};

    g_vs.chg_sched_stop = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Fu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_SEND, g_in1.bms_chg_cmd_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_sched_stop);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    g_in1.bms_chg_cmd_req = 0u;
    body[0] = (uint8_t)(CHG_SCHED_EN | CHG_SCHED_ONCE);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Fu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_SEND, g_in1.bms_chg_cmd_req);
    g_in1.bms_chg_cmd_req = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Fu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.bms_chg_cmd_req);
    body[0] = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Fu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_SEND, g_in1.bms_chg_cmd_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chg_sched_cancel_notice);
}

/**
 * \brief           Regs 0x1D / 0x1E re-map the stored gear when its enable bit changes
 * \note            Reg 0x1D b4 changed and gear 1 -> 2. Reg 0x1E b9 or b8 changed: gear 3 -> 4
 *                      (b9 set) / 2, gear 4 -> 2.
 */
static void test_reg_side_effect_gear_remap(void) {
    uint8_t body[2] = {(uint8_t)CFG_WALK_EN, 0u};

    g_vs.gear_sel = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Du, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.gear_sel);
    g_vs.gear_sel = 3u;
    body[0] = 0u;
    body[1] = (uint8_t)(CFG2_GEAR4_EN >> 8);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Eu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(4u, g_vs.gear_sel);
    body[1] = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Eu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.gear_sel);
    g_vs.gear_sel = 3u;
    body[1] = (uint8_t)(CFG2_GEAR3_EN >> 8);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x1Eu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.gear_sel);
}

/**
 * \brief           Regs 0x4B / 0x4C push the schedule to the BMS; 0x4B also disarms it
 * \note            0x4B -> IN1+0x28 = 1, IN1+0x22 = 0; 0x4C -> IN1+0x28 = 1.
 */
static void test_reg_side_effect_charge_window(void) {
    const uint8_t body[] = {0x1Eu, 0x08u};

    g_in1.chg_sched_armed = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x4Cu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_SEND, g_in1.bms_chg_cmd_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.chg_sched_armed);
    g_in1.bms_chg_cmd_req = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_SILENT, 0x4Bu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_SEND, g_in1.bms_chg_cmd_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.chg_sched_armed);
    TEST_ASSERT_EQUAL_HEX16(0x081Eu, g_in2.f.chg_sched_start);
}

/**
 * \brief           Reg 0x46 with a valid CRC: region Germany writes 20/22 km/h into both copies
 * \note            CRC-32 over body[0] + key + frame[0x0E]
 *                      against frame[0x0A..0x0D] BE; VS+0 == 2 -> low byte 0x14 (flag 0) / 0x16.
 */
static void test_speed_gate_region_germany(void) {
    uint8_t body[10];

    g_vs.region = 2u;
    g_in2.f.speed_limit = 0x0100u;
    prv_speed_body(body, 0x19u, 0u);
    TEST_ASSERT_EQUAL_HEX8(NB_ST_OK, prv_write_status(REG_SPEED_LIMIT, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0x0114u, g_in2.f.speed_limit);
    TEST_ASSERT_EQUAL_HEX16(0x0014u, g_in2.f.speed_limit_2);
    prv_recapture();
    prv_speed_body(body, 0x19u, 1u);
    TEST_ASSERT_EQUAL_HEX8(NB_ST_OK, prv_write_status(REG_SPEED_LIMIT, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0x0116u, g_in2.f.speed_limit);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
}

/**
 * \brief           Reg 0x46 in region Europe writes 25/35 km/h only with the flag set; Global rejects
 * \note            VS+0 == 1 and frame[0x0E] != 0 -> 0x19 when the
 *                      limit is 0, else 0x23; flag 0 -> status 0, nothing written; other modes -> 4.
 */
static void test_speed_gate_region_europe_and_global(void) {
    uint8_t body[10];

    g_vs.region = 1u;
    prv_speed_body(body, 0u, 0u);
    TEST_ASSERT_EQUAL_HEX8(NB_ST_OK, prv_write_status(REG_SPEED_LIMIT, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.speed_limit);
    prv_recapture();
    prv_speed_body(body, 0u, 1u);
    TEST_ASSERT_EQUAL_HEX8(NB_ST_OK, prv_write_status(REG_SPEED_LIMIT, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0x0019u, g_in2.f.speed_limit);
    prv_recapture();
    TEST_ASSERT_EQUAL_HEX8(NB_ST_OK, prv_write_status(REG_SPEED_LIMIT, body, sizeof(body)));
    TEST_ASSERT_EQUAL_HEX16(0x0023u, g_in2.f.speed_limit);
    TEST_ASSERT_EQUAL_HEX16(0x0023u, g_in2.f.speed_limit_2);
    prv_recapture();
    g_vs.region = 0u;
    TEST_ASSERT_EQUAL_HEX8(NB_ST_REJECT, prv_write_status(REG_SPEED_LIMIT, body, sizeof(body)));
}

/**
 * \brief           cmd 0x04 is a raw write: no permission check, no reply, no side effects
 * \note            memcpy(IN2 + arg*2, body, len) only; a write that runs past the register
 *                      file is dropped (nb_reg_write).
 */
static void test_reg_raw_write(void) {
    const uint8_t body[] = {0x34u, 0x12u, 0x78u, 0x56u};

    g_in2.f.cfg_flags2 = CFG2_ACK_BEEP;
    g_vs.gear_sel = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_RAW, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_in2.reg[0x10u]);
    TEST_ASSERT_EQUAL_HEX16(0x5678u, g_in2.reg[0x11u]);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_RAW, 0x1Du, body, 2u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    g_in2.reg[0xFFu] = 0xA5A5u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_RAW, 0xFFu, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0xA5A5u, g_in2.reg[0xFFu]);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           A read that would run past the register file gets no reply
 * \note            nb_reg_read drops the request. A read ending at the last byte is answered.
 */
static void test_reg_read_past_end_dropped(void) {
    const uint8_t over[] = {3u};
    const uint8_t fits[] = {2u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_READ, 0xFFu, over, sizeof(over));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_READ, 0xFFu, fits, sizeof(fits));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
}

/* -- info node 0x3D..0x3F ---------------------------------------------------------------------- */

/**
 * \brief           Every info frame clears the lock edge, reloads keep-awake and, in RIDE, auto-off
 * \note            VS+0x23 = 0, VS+0x34 = 300, STATE == 3 ->
 *                      VS+0x3C = (s16)reg 0x49 * 6000. Also for src 0x3E / 0x3F.
 */
static void test_info_preamble(void) {
    const uint8_t body[] = {0u};

    g_vs.remote_lock_edge = 1u;
    g_in2.f.auto_off_min = 5u;
    tst_nb_request(VCU_TRANSPORT_USART, 0x3Fu, NODE_VCU, 0x7Fu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_edge);
    TEST_ASSERT_EQUAL_UINT16(KEEPALIVE_TICKS, g_vs.keepalive_ticks);
    TEST_ASSERT_EQUAL_UINT32(0u, g_vs.auto_off_ticks);
    g_state_block.fsm_state = VCU_FSM_RIDE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO_3E, NODE_VCU, 0x7Fu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(5u * AUTO_OFF_SCALE, g_vs.auto_off_ticks);
}

/**
 * \brief           cmd 0x13 replies `body[0]` bytes of the info config block at arg*2
 * \note            vcu_usart_report_send(0x16, src, f[5], 4, arg, cfg block + arg*2): count 4
 *                      gives 4 bytes.
 */
static void test_info_cfg_read_count(void) {
    const uint8_t body[] = {4u};
    tst_nb_frame_t frame;

    g_telem_frame[REPORT_TELEM_CFG_BASE + 2u] = 0x2211u;
    g_telem_frame[REPORT_TELEM_CFG_BASE + 3u] = 0x4433u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x13u, 2u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_READ_REPLY, 2u, 4u);
    TEST_ASSERT_EQUAL_HEX8(0x11u, frame.body[0]);
    TEST_ASSERT_EQUAL_HEX8(0x44u, frame.body[3]);
}

/**
 * \brief           cmd 0x17 is an alias of 0x13, and both answer on USART even for a CAN request
 * \note            `cmd == 0x17` runs the case-0x13 body, which always enqueues on the USART
 *                      ring (vcu_usart_report_send(0x16, src, f[5], 4, arg, blk + arg*2)).
 *                      The block content is set by hand here; its writer is report.c.
 */
static void test_info_cfg_read_alias_usart_only(void) {
    const uint8_t body[] = {2u};
    tst_nb_frame_t frame;

    g_telem_frame[REPORT_TELEM_CFG_BASE + 5u] = 0xBEEFu;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x17u, 5u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_READ_REPLY, 5u, 2u);
    TEST_ASSERT_EQUAL_HEX8(0xEFu, frame.body[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBEu, frame.body[1]);

    tst_fakes_reset(); /* fresh USART capture */
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, 0x13u, 5u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump()); /* not on CAN */
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_READ_REPLY, 5u, 2u);
}

/**
 * \brief           cmd 0x41 identity reads: arg 0x10/0xDA body[0] bytes, 0x17 two bytes, 0x20 sixteen
 * \note            Data IN2+0x1B4 (0x10, 0xDA), IN2+0x2E (0x17),
 *                      IN2+0x40 (0x20); cmd 0x43. Unknown arg: no reply.
 */
static void test_info_ident_reads(void) {
    const uint8_t count[] = {3u};
    tst_nb_frame_t frame;
    uint8_t idx;

    for (idx = 0u; idx < 16u; ++idx) {
        g_in2.ident[idx] = (uint8_t)(0xC0u + idx);
        g_in2.f.vin[idx] = (uint8_t)(0x40u + idx);
    }
    g_in2.f.serial_mark = 0xBEEFu;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x41u, 0x10u, count, sizeof(count));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x41u, 0x17u, count, sizeof(count));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x41u, 0x20u, count, sizeof(count));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x41u, 0xDAu, count, sizeof(count));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x41u, 0x11u, count, sizeof(count));
    TEST_ASSERT_EQUAL_UINT32(4u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0x10u, 3u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t*)(const volatile void*)g_in2.ident, frame.body, 3u);
    prv_usart_frame(1u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0x17u, 2u);
    TEST_ASSERT_EQUAL_HEX8(0xEFu, frame.body[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBEu, frame.body[1]);
    prv_usart_frame(2u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0x20u, 16u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t*)(const volatile void*)g_in2.f.vin, frame.body, 16u);
    prv_usart_frame(3u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_IDENT_REPLY, 0xDAu, 3u);
}

/**
 * \brief           cmd 0x18 status = byte difference of the serial tail vs "0000000001", with the beep
 * \note            status = nb_keycmp(IN2+0x24, "0000000001", 10); beep(4) when 0 else beep(3);
 *                      reply cmd 0x18, arg = status, no body, on the request's bus. A 1-byte body
 *                      never matches the CRC-32 gate (see \ref test_info_pair_crc_commit). Over
 *                      the CAN tunnel the reply is `01 18 17` = 'G' - '0'.
 */
static void test_info_pair_status(void) {
    static const uint8_t serial_tpl[] = {'0', '3', 'S', 'P', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'};
    const uint8_t body[] = {0u};
    tst_nb_frame_t frame;

    memcpy((void*)g_in2.f.serial, serial_tpl, sizeof(serial_tpl));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_PAIR, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_TRIPLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_PAIR, 0u, 0u);
    g_in2.f.serial[5] = 'G';
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, CMD_PAIR, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO, NODE_VCU, CMD_PAIR, (uint8_t)('G' - '0'), NULL, 0u);
}

/**
 * \brief           cmd 0x18 serial provisioning: a "03S" serial with the right checksum is stored
 * \note            When the CRC gate fails, len 0x12, body "03S..", weighted checksum (salt
 *                      IN2+0x1BC * 0x11, |.|, % 1000000) == f[0x13]; then the serial is copied to
 *                      IN2+0x20, the UID to IN2+0x1B4 (3 words), status 1, then
 *                      config_serial_commit, the LED matrix, drive_param_init, the identity store
 *                      (an erase of an identity page) and IN1+0x10 = 1.
 */
static void test_info_pair_provision(void) {
    static const uint8_t new_serial[] = {'0', '3', 'S', 'P', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
    uint8_t body[18];
    tst_nb_frame_t frame;
    uint32_t ops;

    memset((void*)g_in2.f.serial, '7', sizeof(g_in2.f.serial));
    g_in2.ident_word[2] = 0x1234u;
    g_mcu_uid[0] = 0x11111111u;
    g_mcu_uid[1] = 0x22222222u;
    g_mcu_uid[2] = 0x33333333u;
    memcpy(body, new_serial, sizeof(new_serial));
    prv_put_u32(&body[14], prv_pair_checksum(g_in2.f.serial, new_serial, sizeof(new_serial)) + 1u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_PAIR, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8('7', g_in2.f.serial[0]);
    prv_put_u32(&body[14], prv_pair_checksum(g_in2.f.serial, new_serial, sizeof(new_serial)));
    g_in1.recompute_req = 0u;
    ops = tst_flash_op_count();
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_PAIR, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(new_serial, (const uint8_t*)(const volatile void*)g_in2.f.serial, sizeof(new_serial));
    TEST_ASSERT_EQUAL_HEX32(0x11111111u, g_in2.ident_word[0]);
    TEST_ASSERT_EQUAL_HEX32(0x33333333u, g_in2.ident_word[2]);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_TRUE(prv_find(tst_usart_drain(), NODE_INFO, CMD_PAIR, 1u, &frame));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_TRUE(prv_identity_erased_since(ops));
}

/**
 * \brief           cmd 0x18 CRC gate: a matching CRC-32 stores frame[0x0E..0x1B] as the serial
 * \note            CRC-32 over {frame[5], the 16-byte RAM key, frame[0x0E..0x1B]} (31 bytes)
 *                      == f[0x0D] | f[0x0A] << 24 | f[0x0B] << 16 | f[0x0C] << 8 -> status 1,
 *                      serial (IN2+0x20, 14 bytes) = frame[0x0E..0x1B], identity store,
 *                      IN1+0x10 = 1, beep(3), reply arg 1. A wrong CRC changes nothing and
 *                      replies the template-compare status.
 */
static void test_info_pair_crc_commit(void) {
    static const uint8_t new_serial[] = {'0', '3', 'S', 'P', '9', '8', '7', '6', '5', '4', '3', '2', '1', '0'};
    uint8_t body[23];
    uint8_t buf[31];
    tst_nb_frame_t frame;
    uint32_t crc;
    uint32_t ops;

    memset((void*)g_in2.f.serial, '7', sizeof(g_in2.f.serial));
    g_in1.recompute_req = 0u;
    memset(body, 0, sizeof(body));
    body[0] = 0x5Au;
    memcpy(&body[9], new_serial, sizeof(new_serial));
    buf[0] = body[0];
    memset(&buf[1], 0xFF, 16u); /* the RAM key: erased flash cell after nb_key_load */
    memcpy(&buf[17], new_serial, sizeof(new_serial));
    crc = prv_crc32(buf, sizeof(buf));
    body[5] = (uint8_t)(crc >> 24);
    body[6] = (uint8_t)(crc >> 16);
    body[7] = (uint8_t)(crc >> 8);
    body[8] = (uint8_t)(crc ^ 1u);

    ops = tst_flash_op_count();
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_PAIR, 0u, body, sizeof(body));
    TEST_ASSERT_EACH_EQUAL_HEX8('7', (const uint8_t*)(const volatile void*)g_in2.f.serial, sizeof(new_serial));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_UINT32(ops, tst_flash_op_count());
    TEST_ASSERT_TRUE(prv_find(tst_usart_drain(), NODE_INFO, CMD_PAIR, (uint8_t)('7' - '0'), &frame));

    body[8] = (uint8_t)crc;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_PAIR, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(new_serial, (const uint8_t*)(const volatile void*)g_in2.f.serial, sizeof(new_serial));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_TRUE(prv_identity_erased_since(ops));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(1u, &frame); /* the capture is cumulative: [0] is the rejected request's reply */
    prv_assert_header(&frame, NODE_INFO, CMD_PAIR, 1u, 0u);
}

/**
 * \brief           cmd 0x19 VIN write: checksum-gated, stores VIN + build id, replies the status
 * \note            len 0x2A, checksum over IN2+0x40 vs f[0x15]; then VIN = body[0..15],
 *                      IN2+0x1C8 = f[0x19] (u32), IN2+0x1CC = f[0x1D] (u16), status 1; reply cmd
 *                      0x19, arg = status, no body (also on CAN: the sink gets f + 5, so a short
 *                      request is answered `01 19 00`).
 */
static void test_info_vin_write(void) {
    uint8_t body[0x2A];
    tst_nb_frame_t frame;
    uint8_t idx;

    memset(body, 0, sizeof(body));
    for (idx = 0u; idx < 16u; ++idx) {
        body[idx] = (uint8_t)('A' + idx);
        g_in2.f.vin[idx] = (uint8_t)('a' + idx);
    }
    prv_put_u32(&body[20], 0xCAFEF00Du);
    body[24] = 0x34u;
    body[25] = 0x12u;
    prv_put_u32(&body[16], prv_pair_checksum(g_in2.f.vin, body, 16u) ^ 1u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_VIN, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8('a', g_in2.f.vin[0]);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_INFO, CMD_VIN, 0u, 0u);
    prv_recapture();
    prv_put_u32(&body[16], prv_pair_checksum(g_in2.f.vin, body, 16u));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_VIN, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(body, (const uint8_t*)(const volatile void*)g_in2.f.vin, 16u);
    /* body[20..25] = f[0x19..0x1E]: the 6 build-id bytes, in order */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&body[20], (const uint8_t*)(const volatile void*)g_in2.build_id, 6u);
    TEST_ASSERT_TRUE(prv_find(tst_usart_drain(), NODE_INFO, CMD_VIN, 1u, &frame));
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, CMD_VIN, 0u, body, 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO, NODE_VCU, CMD_VIN, 0u, NULL, 0u);
}

/**
 * \brief           cmd 0x59 with a valid token opens the rental session
 * \note            Case 0x59 (0x57 shares it): len 10, nb_validate_token token, IN2+0x38 b11
 *                      clear -> beep(3), nb_report_frame(2, 0x16, 7, 0x57, 0, f+9),
 *                      IN2+0x80 = f[0x0D], b11 set, screen 0x14, regs 0x62..0x67 = 0, VS+1 = 2,
 *                      VS+0x2D = 1, IN1+0x10 = 1. Otherwise beep(1).
 */
static void test_info_session_open(void) {
    uint8_t body[10];
    const tst_can_frame_t* report;
    uint32_t sent;
    uint32_t idx;
    uint8_t found = 0u;

    prv_token(body);
    body[8] = 0x22u;
    body[9] = 0x11u;
    g_in2.f.session_dist = 1234u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x59u, 0u, body, 9u);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.status_flags & SF_SESSION);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x59u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0x1122u, g_in2.f.session_token);
    TEST_ASSERT_EQUAL_HEX16(SF_SESSION, g_in2.f.status_flags & SF_SESSION);
    TEST_ASSERT_EQUAL_HEX16(0x0014u, g_scr.ui_screen_u);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_dist);
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.gear_sel);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.reboot_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    sent = prv_can_pump();
    for (idx = 0u; idx < sent; ++idx) {
        report = tst_can_tx_get(idx);
        if ((report->dlc == 2u) && (report->data[0] == body[4]) && (report->data[1] == body[5])) {
            found = 1u;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "no 2-byte 0x57 session report on CAN");
}

/**
 * \brief           cmd 0x57 shares the 0x59 handler and is refused while a session is open
 * \note            0x57 runs the 0x59 body, whose gate requires IN2+0x38 b11 clear.
 */
static void test_info_session_open_refused_when_active(void) {
    uint8_t body[10];

    prv_token(body);
    g_in2.f.status_flags = SF_SESSION;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x57u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
    g_in2.f.status_flags = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x57u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(SF_SESSION, g_in2.f.status_flags);
}

/**
 * \brief           cmd 0x58 with a valid token closes the session; a bad token beeps 3 times
 * \note            len 8 + token -> beep(3), IN2+0x80 = 0, b11 cleared,
 *                      regs 0x62..0x67 = 0, IN1+0x10 = 1; else beep(4).
 */
static void test_info_session_close(void) {
    uint8_t body[8];

    prv_token(body);
    g_in2.f.session_token = 0x5555u;
    g_in2.f.status_flags = (uint16_t)(SF_SESSION | SF_RUN);
    g_in2.f.session_dist = 99u;
    body[0] ^= 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x58u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_TRIPLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0x5555u, g_in2.f.session_token);
    body[0] ^= 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x58u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.session_token);
    TEST_ASSERT_EQUAL_HEX16(SF_RUN, g_in2.f.status_flags);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_dist);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
}

/**
 * \brief           cmd 0x58 over CAN with a pending app registration in IDLE requests the app start
 * \note            VS+0x27 != 0, STATE == 0 and transport CAN -> VS+0x28 = 1.
 */
static void test_info_session_close_app_start(void) {
    uint8_t body[8];

    prv_token(body);
    g_vs.app_reg_pending = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x58u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_start_req);
    g_state_block.fsm_state = VCU_FSM_RIDE;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, 0x58u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_start_req);
    g_state_block.fsm_state = VCU_FSM_IDLE;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_INFO, NODE_VCU, 0x58u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_start_req);
}

/**
 * \brief           cmd 0x5F writes the session distance when the PIN word matches the UID checksum
 * \note            len 8, *(u32*)(f+5) == the UID checksum -> beep(3),
 *                      IN2+0xC4 = *(u32*)(f+9); else beep(4).
 */
static void test_info_limit_set(void) {
    uint8_t body[8];

    g_state_block.uid_cksum = 0x10203040u;
    prv_put_u32(body, 0x10203041u);
    prv_put_u32(&body[4], 777u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x5Fu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_TRIPLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(0u, g_in2.f.session_dist);
    prv_put_u32(body, 0x10203040u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x5Fu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(777u, g_in2.f.session_dist);
}

/**
 * \brief           cmd 0x80 also emits a report frame to node 7
 * \note            After the ridemode/drive recompute, cmd 0x80 sends a single-frame Ninebot
 *                      CAN report to node 7 (src 0x16, cmd 0x80, arg 1, 2 bytes from
 *                      frame[9..10]), then the config write + beep(3).
 */
static void test_info_lock_commit_node7_frame(void) {
    const uint8_t body[] = {0xAAu, 0xBBu, 0xCCu, 0xDDu, 0x11u, 0x22u}; /* frame[9]=body[4], frame[10]=body[5] */
    const uint32_t want_id = CAN_ID_PF_EF | ((uint32_t)NODE_GW7 << 8) | NODE_VCU; /* dst 7, src 0x16 */
    uint32_t sent;
    uint32_t idx;
    uint8_t found = 0u;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x80u, 0u, body, sizeof(body));
    sent = prv_can_pump();
    for (idx = 0u; idx < sent; ++idx) {
        const tst_can_frame_t* f = tst_can_tx_get(idx);

        if ((f != NULL) && (f->id == want_id) && (f->data[1] == 0x80u) && (f->data[2] == 0x01u)) {
            TEST_ASSERT_EQUAL_UINT8(5u, f->dlc); /* ctl + cmd + arg + 2 payload bytes */
            TEST_ASSERT_EQUAL_HEX8(0x11u, f->data[3]);
            TEST_ASSERT_EQUAL_HEX8(0x22u, f->data[4]);
            found = 1u;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "no src-0x16 report frame to node 7");
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
}

/**
 * \brief           cmd 0x79: body 1 app start (+ unlock with the lock on), body 2 shutdown below 3 km/h
 * \note            body 1 -> VS+0x28 = 1, IN2+0x3A b2 -> VS+0x2C = 1;
 *                      body 2 -> VS+2 <= 2 -> VS+0x1C = 1.
 */
static void test_info_standby(void) {
    uint8_t body[] = {1u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x79u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_unlock_req);
    g_in2.f.cfg_flags = CFG_LOCK_EN;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x79u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_unlock_req);
    body[0] = 2u;
    g_vs.speed_kmh = 3u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x79u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
    g_vs.speed_kmh = 2u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x79u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.shutdown_req);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           cmds 0x11 / 0x5E / 0x80: reboot request, the 0x5E flag, commit + recompute
 * \note            0x11: arg 1 -> VS+0x2D = 1, SCR+0x1A2 = 0. 0x5E: IN1+0x23 = 1, beep(3).
 *                      0x80: LED matrix update, drive_param_init, identity store, IN1+0x10 = 1,
 *                      beep(3). No replies.
 */
static void test_info_state_cmds(void) {
    const uint8_t body[] = {0u};

    g_scr.idle_report = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x11u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.reboot_req);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x11u, 1u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.reboot_req);
    TEST_ASSERT_EQUAL_UINT16(0u, g_scr.idle_report);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x5Eu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.app_flag_5e);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    g_lowglob.beep_seq_id = BEEP_NONE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x80u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           cmd 0x56 arg 0 sets input-test mode at rest and reports it on CAN, whatever the bus
 * \note            beep(3); throttle and both levers < 0x32 (>>4) -> VS+0x64 = body[0] else 0;
 *                      report_ninebot(0x16, src, 1, 0x56, 0, &body[0]).
 */
static void test_info_input_test(void) {
    const uint8_t body[] = {1u};
    const uint8_t expect[] = {1u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x56u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.input_test_mode);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO, NODE_VCU, 0x56u, 0u, expect, sizeof(expect));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x56u, 0xEEu, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x56u, 0xEAu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.input_test_ee);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.input_test_ea);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x56u, 0x14u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.input_test_mode);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.ui_screen_u);
}

/**
 * \brief           cmd 0x56 arg 0 echoes the request byte, even when the input-test gate fails
 * \note            The reply carries frame[5] (body[0]), not the stored mode VS+0x64, which is
 *                      forced to 0 when a lever/throttle is off its rest. With the throttle raised
 *                      the stored mode is 0 but the reply still echoes body[0].
 */
static void test_info_input_test_echoes_request_byte(void) {
    const uint8_t body[] = {0x07u};
    const uint8_t expect[] = {0x07u};

    tst_control_adc_freeze(); /* bring up the ADC scan at rest */
    tst_control_adc_set(VCU_ADC_SLOT_THROTTLE, 0x60u << 4) /* >> 4 >= 0x32: gate fails */;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x56u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.input_test_mode); /* gate failed -> stored mode 0 */
    TEST_ASSERT_EQUAL_UINT32(1u, prv_can_pump());
    prv_assert_can(0u, NODE_INFO, NODE_VCU, 0x56u, 0u, expect, sizeof(expect)); /* echoes 0x07 */
    tst_control_adc_release();
}

/**
 * \brief           OTA cmds 7..10 are dropped while the OTA-busy bit is set, else re-arm CAN-active
 * \note            Case 7..10: `!(g_fault_codes.comm_fault & 1)` -> IN1+3 = 1 and vcu_ota_dispatch
 *                      (ota.c, not under test here).
 */
static void test_info_ota_gate(void) {
    const uint8_t body[] = {0u};

    g_fault_codes.comm_fault = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x0Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.can_active_req);
    g_fault_codes.comm_fault = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, 0x0Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.can_active_req);
}

/* -- control node 0x04 ------------------------------------------------------------------------- */

/**
 * \brief           A control frame raises keep-awake to at least 300 and leaves a higher value alone
 * \note            `if (VS+0x34 < 300) VS+0x34 = 300`.
 */
static void test_ctrl_keepalive(void) {
    const uint8_t body[] = {0u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x55u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(KEEPALIVE_TICKS, g_vs.keepalive_ticks);
    g_vs.keepalive_ticks = 500u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x55u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(500u, g_vs.keepalive_ticks);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           A control-node read arriving on CAN is answered on USART with body[0] bytes
 * \note            vcu_usart_report_send(0x16, src, f[5], 4, arg, IN2 + arg*2); the handler
 *                      ignores the transport. The reply has 3 bytes.
 */
static void test_ctrl_read_replies_on_usart(void) {
    const uint8_t body[] = {3u};
    tst_nb_frame_t frame;

    g_in2.reg[REG_VERSION] = 0x397Au;
    g_in2.reg[REG_VERSION + 1u] = 0x550Fu;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_CTRL, NODE_VCU, CMD_READ, REG_VERSION, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_CTRL, CMD_READ_REPLY, REG_VERSION, 3u);
    TEST_ASSERT_EQUAL_HEX8(0x0Fu, frame.body[2]);
}

/**
 * \brief           A long control-node read is not capped: 0x50 bytes come back as 0x50 bytes
 * \note            vcu_usart_report_send gets the pointer IN2 + arg*2 and body[0]; there is
 *                      no 0x40 limit. A count past the end of the register page is cut at the
 *                      end of g_in2 (nb_clip).
 */
static void test_ctrl_long_read_not_capped(void) {
    const uint8_t body[] = {0x50u};
    const uint8_t tail[] = {0xF0u};
    tst_nb_frame_t frame;

    prv_fill_regs(0x80u, 0x50u, 0x01u);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, CMD_READ, 0x80u, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, CMD_READ, 0xFFu, tail, sizeof(tail));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_CTRL, CMD_READ_REPLY, 0x80u, 0x50u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t*)(const volatile void*)&g_in2.reg[0x80u], frame.body, 0x50u);
    prv_usart_frame(1u, &frame);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(sizeof(g_in2) - 0x1FEu), frame.len);
}

/**
 * \brief           cmd 3 arg 0x86 starts / stops the locate tone and sends no reply
 * \note            body 1 -> buzzer_on(0xA8C), VS+0x29 = 1, VS+0x34 = 300, VS+0x24 = 0xEE;
 *                      body 0 -> buzzer_off, VS+0x34 = 0, VS+0x24 = 0; then return, no reply.
 */
static void test_ctrl_locate_tone(void) {
    uint8_t body[] = {1u};

    buzzer_init();
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x03u, 0x86u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chirp_req);
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_LOCATE, g_vs.flash_seq);
    TEST_ASSERT_EQUAL_UINT16(KEEPALIVE_TICKS, g_vs.keepalive_ticks);
    TEST_ASSERT_EQUAL_HEX32(1u, TMR1->cctrl & 1u);
    body[0] = 0u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x03u, 0x86u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_IDLE, g_vs.flash_seq);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.keepalive_ticks);
    TEST_ASSERT_EQUAL_HEX32(0u, TMR1->cctrl & 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           cmd 4 arg 0x20 maps the mode 0/2 -> 3, 1/3 -> 2 and replies cmd 3 arg 0x20
 * \note            SCR+0x46 = body u16 (mapped), then vcu_usart_report_send(0x16, 4, 2, 3,
 *                      0x20, SCR+0x46): `0x16 0x04 0x03 0x20 0200` for mode 1.
 */
static void test_ctrl_mode_register(void) {
    uint8_t body[] = {1u, 0u};
    const uint8_t expect[] = {2u, 0u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_CAN, NODE_CTRL, NODE_VCU, 0x04u, 0x20u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(2u, g_scr.gest_payload);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_CTRL, CMD_WRITE_SILENT, 0x20u, 2u);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, frame.body, sizeof(expect));
    body[0] = 2u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x04u, 0x20u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(3u, g_scr.gest_payload);
    body[0] = 5u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x04u, 0x20u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(5u, g_scr.gest_payload);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x04u, 0x21u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
}

/**
 * \brief           cmd 2 or 4 with arg 0x7E beeps and echoes body[0] as `02 7E <byte>`
 * \note            beep(3), vcu_usart_report_send(0x16, src, 1, 2, 0x7E, &body0):
 *                      `0x16 0x04 0x02 0x7e 05`.
 */
static void test_ctrl_echo_7e(void) {
    const uint8_t body[] = {5u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x02u, 0x7Eu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x04u, 0x7Eu, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x02u, 0x7Du, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_CTRL, CMD_WRITE_REPLY, 0x7Eu, 1u);
    TEST_ASSERT_EQUAL_HEX8(5u, frame.body[0]);
}

/**
 * \brief           Control node cmd 2/4 arg 0xE0 echoes like 0x7E only when body[0] is 1/3/5
 * \note            Arg 0xE0 with body[0] in {1,3,5} -> beep(3) then the same reply as 0x7E
 *                      (cmd 2, arg 0x7E, 1 byte = body[0]); other body values return with no beep
 *                      and no reply.
 */
static void test_ctrl_echo_e0(void) {
    const uint8_t body_ok[] = {3u};
    const uint8_t body_no[] = {2u};
    tst_nb_frame_t frame;

    /* body not in {1,3,5}: nothing happens */
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x02u, 0xE0u, body_no, sizeof(body_no));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());

    /* body in {1,3,5}: beep + echo as arg 0x7E */
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x02u, 0xE0u, body_ok, sizeof(body_ok));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_CTRL, CMD_WRITE_REPLY, 0x7Eu, 1u);
    TEST_ASSERT_EQUAL_HEX8(3u, frame.body[0]);
}

/**
 * \brief           cmds 7 and 0x0A clear the remote lock and send no control reply
 * \note            7: IN1+3 = 1, VS+0x23 = VS+0x22 = 0.
 *                      0x0A: IN1+5 = 1, report_can_toggle(0), clears; return.
 */
static void test_ctrl_session_ops(void) {
    const uint8_t body[] = {0u};

    g_vs.remote_lock_state = 4u;
    g_vs.remote_lock_edge = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x07u, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.can_active_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_state);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_edge);
    g_vs.remote_lock_state = 4u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x0Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.uv_grace_ext);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_state);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_count_to(tst_usart_drain(), NODE_CTRL));
}

/**
 * \brief           cmd 0x7A latches the remote lock value; a 0 -> non-zero change beeps and sets the edge
 * \note            Value differs -> VS+0x23 = 0, VS+0x22 = arg, non-zero ->
 *                      VS+0x23 = 1 + beep(3); reply cmd 0x7A, arg = VS+0x22, no body.
 */
static void test_ctrl_lock_state(void) {
    const uint8_t body[] = {0u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Au, 2u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(2u, g_vs.remote_lock_state);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.remote_lock_edge);
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    g_lowglob.beep_seq_id = BEEP_NONE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Au, 2u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.remote_lock_edge);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_CTRL, NODE_VCU, 0x7Au, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_state);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_edge);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
    TEST_ASSERT_EQUAL_UINT32(3u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_CTRL, 0x7Au, 2u, 0u);
}

/**
 * \brief           cmd 0x7B with the lock enabled: unlock (1), lock (4), clear (0 / 2 / 3)
 * \note            IN2+0x3A b2 set -> arg 1 (not yet unlocked):
 *                      VS+0x29 = 1, VS+0x40 = 0, VS+0x43 = 1, IDLE -> VS+0x24 = 1; arg 4: VS+0x40 = 1,
 *                      VS+0x43 = 0; arg 0: both 0; arg 2/3: VS+0x43 = 0. Reply cmd 0x7B arg = arg.
 */
static void test_ctrl_remote_lock_enabled(void) {
    const uint8_t body[] = {0u};
    tst_nb_frame_t frame;

    g_in2.f.cfg_flags = CFG_LOCK_EN;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 1u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.remote_unlocked);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chirp_req);
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_START, g_vs.flash_seq);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 4u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.remote_lock_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_unlocked);
    g_vs.remote_unlocked = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 2u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_unlocked);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.remote_lock_req);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_req);
    TEST_ASSERT_EQUAL_UINT32(4u, tst_usart_drain());
    prv_usart_frame(1u, &frame);
    prv_assert_header(&frame, NODE_CTRL, 0x7Bu, 4u, 0u);
}

/**
 * \brief           cmd 0x7B with the lock disabled clears the unlock and the flash (except locate)
 * \note            VS+0x43 = 0; VS+0x24 != 0xEE -> VS+0x24 = 0.
 */
static void test_ctrl_remote_lock_disabled(void) {
    const uint8_t body[] = {0u};

    g_vs.remote_unlocked = 1u;
    g_vs.flash_seq = FLASH_SEQ_RUN;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 1u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_unlocked);
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_IDLE, g_vs.flash_seq);
    g_vs.flash_seq = FLASH_SEQ_LOCATE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 1u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(FLASH_SEQ_LOCATE, g_vs.flash_seq);
}

/**
 * \brief           cmd 0x7B arg 7 body 1 beeps for the first three app-auth attempts only
 * \note            App-auth counter < 3 -> +1 and beep(3); body 2/3 reset it.
 */
static void test_ctrl_app_auth_counter(void) {
    uint8_t body[] = {1u};
    uint8_t round;

    for (round = 0u; round < 3u; ++round) {
        g_lowglob.beep_seq_id = BEEP_NONE;
        tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 7u, body, sizeof(body));
        TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
    }
    g_lowglob.beep_seq_id = BEEP_NONE;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 7u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    body[0] = 3u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 7u, body, sizeof(body));
    body[0] = 1u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_CTRL, NODE_VCU, 0x7Bu, 7u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(BEEP_DOUBLE, g_lowglob.beep_seq_id);
}

/* -- app node 0x23 ----------------------------------------------------------------------------- */

/**
 * \brief           App reads: arg 0x2F -> `04 17` from IN2+0x2E, arg 0x30 -> `04 20` from IN2+0x40
 * \note            body[0] bytes; other args no reply. Every app frame clears the shared UV
 *                      counter (g_lowglob.fm_uv_cnt, the whole halfword).
 */
static void test_app_reads(void) {
    const uint8_t body[] = {4u};
    tst_nb_frame_t frame;

    g_lowglob.fm_uv_cnt = 0x1234u;
    g_in2.f.serial_mark = 0x0201u;
    g_in2.f.vin[0] = 0xAAu;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_READ, 0x2Fu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT16(0u, g_lowglob.fm_uv_cnt);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_READ, 0x30u, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_READ, 0x31u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_READ_REPLY, 0x17u, 4u);
    TEST_ASSERT_EQUAL_HEX8(0x01u, frame.body[0]);
    prv_usart_frame(1u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_READ_REPLY, 0x20u, 4u);
    TEST_ASSERT_EQUAL_HEX8(0xAAu, frame.body[0]);
}

/**
 * \brief           App cmd 2 (not 0x84) writes register page 1 and replies `05 <arg> 00 00`
 * \note            memcpy(SCR + arg*2, body, len), then the status reply.
 */
static void test_app_page1_write(void) {
    const uint8_t body[] = {0x34u, 0x12u};
    tst_nb_frame_t frame;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_WRITE_REPLY, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0x1234u, g_scr.reg[0x10u]);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_APP, CMD_STATUS_REPLY, 0x10u, 2u);
}

/**
 * \brief           App start in screen 0x0B announces 14 bytes from IN2+0x20; outside IDLE no start
 * \note            IN1+5 = 0, nb_commit_mode(SCR+0x4A), screen 0x0B ->
 *                      vcu_usart_report_send(0x16, 0x23, 0x0E, 3, 0, IN2+0x20); STATE != 0 -> no
 *                      VS+0x28.
 */
static void test_app_start_announce(void) {
    tst_nb_frame_t frame;
    uint32_t sent;

    g_scr.ui_screen_u = 0x0Bu;
    g_in1.uv_grace_ext = 1u;
    g_state_block.fsm_state = VCU_FSM_RIDE;
    g_in2.f.serial[0] = '0';
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_WRITE_REPLY, ARG_APP_START, NULL, 0u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.uv_grace_ext);
    sent = tst_usart_drain();
    TEST_ASSERT_TRUE(prv_find(sent, NODE_APP, CMD_WRITE_SILENT, 0u, &frame));
    TEST_ASSERT_EQUAL_UINT8(0x0Eu, frame.len);
    TEST_ASSERT_EQUAL_HEX8('0', frame.body[0]);
}

/**
 * \brief           App cmd 3 arg 0xD2 writes VS+0x0E, cmd 4 writes IN2 + arg*2; neither replies
 */
static void test_app_writes(void) {
    const uint8_t body[] = {0x78u, 0x56u};

    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_WRITE_SILENT, 0xD2u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0x5678u, g_vs.distance_code);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_WRITE_SILENT, 0xD3u, body, 1u);
    TEST_ASSERT_EQUAL_HEX16(0x5678u, g_vs.distance_code);
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, CMD_WRITE_RAW, 0x4Du, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0x5678u, g_in2.f.in2_9a);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           App arg 0xFC with a pending registration in IDLE requests the app start
 * \note            VS+0x27 && STATE == 0 -> delay, nb_commit_mode(0), sequencer, delay,
 *                      VS+0x28 = 1, then bp_handle_fc (busproto).
 */
static void test_app_fc_registration_start(void) {
    const uint8_t body[] = {0xFCu, 1u, 2u, 3u};

    vcu_busproto_init();
    tst_usart_drain();
    prv_recapture();
    tst_flash_forward(0u);
    g_scr.ui_screen_u = 7u;
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, 0x01u, 0xFCu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_HEX16(0u, g_scr.ui_screen_u);
}

/**
 * \brief           App arg 0xFD hands the registration body and its length to busproto
 * \note            bp_node_register(f+5, len) resolves the node from the body pointer and its
 *                      length.
 */
static void test_app_fd_registration(void) {
    /* node 0x05, period 3, one signal: key 0x7E765289 (power half), 16 bit */
    const uint8_t body[] = {0x05u, 0x03u, 0x89u, 0x52u, 0x76u, 0x7Eu, 16u};
    const uint8_t ok[] = {0x05u, 0x00u};
    tst_nb_frame_t frame;

    vcu_busproto_init();
    tst_usart_drain();
    prv_recapture();
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, 0x01u, 0xFDu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0x05u, g_bp_node.node_id);
    TEST_ASSERT_EQUAL_UINT32(1u, tst_usart_drain());
    prv_usart_frame(0u, &frame);
    prv_assert_header(&frame, NODE_APP, 0x05u, 0xFDu, sizeof(ok));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(ok, frame.body, sizeof(ok));
}

/* -- sub-nodes 0x02 / 0x06 / 0x07 -------------------------------------------------------------- */

/**
 * \brief           Sub-node 2 writes only the component record (cmd 4, arg 0xDA) into IN2+0x180
 * \note            cmd 4 && arg 0xDA -> memcpy(IN2+0x180, body, len).
 */
static void test_sub02_component_write(void) {
    const uint8_t body[] = {0x11u, 0x22u, 0x33u};
    const volatile uint8_t* dst = (const volatile uint8_t*)&g_in2 + 0x180u;

    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB02, NODE_VCU, CMD_WRITE_RAW, 0xDBu, body, sizeof(body));
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB02, NODE_VCU, CMD_WRITE_SILENT, 0xDAu, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8(0u, dst[0]);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB02, NODE_VCU, CMD_WRITE_RAW, 0xDAu, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX8(0x11u, dst[0]);
    TEST_ASSERT_EQUAL_HEX8(0x33u, dst[2]);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           Node 0x02 cmd 5 arg 0xD0 marks the reg 0xD0 parameter push done
 * \note            src 2, cmd 5, arg 0xD0 -> the push state := 2 (NODE2_PUSH_DONE), no reply.
 *                      Other cmd/arg pairs leave it.
 */
static void test_sub02_push_ack(void) {
    const uint8_t body[] = {0u};

    g_node2_push.state = (uint8_t)NODE2_PUSH_PENDING;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB02, NODE_VCU, 0x05u, 0xD1u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_PENDING, g_node2_push.state);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB02, NODE_VCU, 0x04u, 0xD0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_PENDING, g_node2_push.state);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB02, NODE_VCU, 0x05u, 0xD0u, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(NODE2_PUSH_DONE, g_node2_push.state);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_can_pump());
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
}

/**
 * \brief           Sub-nodes 6/7: bounded write into page 2, the 0x87 ack, presence flag, comm-loss clear
 * \note            arg < 0xFF; cmd 4 and arg + len/2 <= 0xFF -> memcpy(STAT + arg*2, body, len);
 *                      cmd 5 body 0x87 arg 0 -> IN1+0x28 = 2; then IN1+0x18 = 1 and the whole
 *                      comm-loss halfword (g_bms_commloss) := 0.
 */
static void test_sub0607_writes(void) {
    const uint8_t body[] = {0x34u, 0x12u, 0x78u, 0x56u};
    const uint8_t ack[] = {0x87u};
    const volatile uint16_t* page = (const volatile uint16_t*)(const volatile void*)&g_stat;

    g_bms_commloss.cnt = 0x0123u;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB06, NODE_VCU, CMD_WRITE_RAW, 0x10u, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0x1234u, page[0x10u]);
    TEST_ASSERT_EQUAL_HEX16(0x5678u, page[0x11u]);
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.sub0607_rx);
    TEST_ASSERT_EQUAL_HEX16(0u, g_bms_commloss.cnt);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_GW7, NODE_VCU, CMD_WRITE_RAW, 0xFEu, body, sizeof(body));
    TEST_ASSERT_EQUAL_HEX16(0u, page[0xFEu]);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_GW7, NODE_VCU, 0x05u, 1u, ack, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.bms_chg_cmd_req);
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_GW7, NODE_VCU, 0x05u, 0u, ack, sizeof(ack));
    TEST_ASSERT_EQUAL_UINT8(BMS_CHG_CMD_ACKED, g_in1.bms_chg_cmd_req);
}

/**
 * \brief           A sub-node frame with arg 0xFF is ignored entirely
 * \note            `arg >= 0xFF` -> return before any write.
 */
static void test_sub0607_arg_ff_ignored(void) {
    const uint8_t body[] = {1u, 2u};

    g_bms_commloss.cnt = 5u;
    tst_nb_request(VCU_TRANSPORT_CAN, NODE_SUB06, NODE_VCU, CMD_WRITE_RAW, 0xFFu, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.sub0607_rx);
    TEST_ASSERT_EQUAL_HEX16(5u, g_bms_commloss.cnt);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_frame_build_reference_vector);
    RUN_TEST(test_frame_parse_flags_bad_checksum);
    RUN_TEST(test_usart_rx_request_is_answered);
    RUN_TEST(test_usart_rx_bad_checksum_is_ignored);
    RUN_TEST(test_reg_read_count_is_bytes);
    RUN_TEST(test_reg_read_single_byte);
    RUN_TEST(test_reg_read_over_can_single_frame);
    RUN_TEST(test_ident_read_lengths);
    RUN_TEST(test_reg_write_ok_status_is_two_bytes);
    RUN_TEST(test_reg_write_rejected_status_is_two_bytes);
    RUN_TEST(test_reg_write_cmd3_is_silent);
    RUN_TEST(test_app_start_reply_is_status);
    RUN_TEST(test_app_reply_ignores_transport);
    RUN_TEST(test_boot_announce_body);
    RUN_TEST(test_crc32_check_value);
    RUN_TEST(test_keycmp_byte_difference);
    RUN_TEST(test_commit_mode_reports_and_clears_node);
    RUN_TEST(test_report_frame_chunks);
    RUN_TEST(test_unknown_dst_is_dropped);
    RUN_TEST(test_dst_zero_is_local);
    RUN_TEST(test_unknown_src_is_dropped);
    RUN_TEST(test_gateway_usart_to_can);
    RUN_TEST(test_gateway_can_to_usart);
    RUN_TEST(test_gateway_ota_cmds_rearm_can_active);
    RUN_TEST(test_gateway_ota_busy_replies);
    RUN_TEST(test_gateway_can_bms_ack_beep);
    RUN_TEST(test_gateway_dst7_status_double_send);
    RUN_TEST(test_gateway_dst7_countdown);
    RUN_TEST(test_reply_fc_dispatches_over_can);
    RUN_TEST(test_reg_write_permission_table);
    RUN_TEST(test_reg_write_partly_denied_range);
    RUN_TEST(test_reg_write_odd_or_empty_rejected);
    RUN_TEST(test_reg_write_past_last_register_denied);
    RUN_TEST(test_reg_write_status_over_can);
    RUN_TEST(test_reg_write_ack_beep);
    RUN_TEST(test_reg_write_ride_assist_toggles_reg1e);
    RUN_TEST(test_reg_write_passcode_is_staged);
    RUN_TEST(test_reg_side_effect_charge_schedule);
    RUN_TEST(test_reg_side_effect_gear_remap);
    RUN_TEST(test_reg_side_effect_charge_window);
    RUN_TEST(test_speed_gate_region_germany);
    RUN_TEST(test_speed_gate_region_europe_and_global);
    RUN_TEST(test_reg_raw_write);
    RUN_TEST(test_reg_read_past_end_dropped);
    RUN_TEST(test_info_preamble);
    RUN_TEST(test_info_cfg_read_count);
    RUN_TEST(test_info_cfg_read_alias_usart_only);
    RUN_TEST(test_info_ident_reads);
    RUN_TEST(test_info_pair_status);
    RUN_TEST(test_info_pair_provision);
    RUN_TEST(test_info_pair_crc_commit);
    RUN_TEST(test_info_vin_write);
    RUN_TEST(test_info_session_open);
    RUN_TEST(test_info_session_open_refused_when_active);
    RUN_TEST(test_info_session_close);
    RUN_TEST(test_info_session_close_app_start);
    RUN_TEST(test_info_limit_set);
    RUN_TEST(test_info_lock_commit_node7_frame);
    RUN_TEST(test_info_standby);
    RUN_TEST(test_info_state_cmds);
    RUN_TEST(test_info_input_test);
    RUN_TEST(test_info_input_test_echoes_request_byte);
    RUN_TEST(test_info_ota_gate);
    RUN_TEST(test_ctrl_keepalive);
    RUN_TEST(test_ctrl_read_replies_on_usart);
    RUN_TEST(test_ctrl_long_read_not_capped);
    RUN_TEST(test_ctrl_locate_tone);
    RUN_TEST(test_ctrl_mode_register);
    RUN_TEST(test_ctrl_echo_7e);
    RUN_TEST(test_ctrl_echo_e0);
    RUN_TEST(test_ctrl_session_ops);
    RUN_TEST(test_ctrl_lock_state);
    RUN_TEST(test_ctrl_remote_lock_enabled);
    RUN_TEST(test_ctrl_remote_lock_disabled);
    RUN_TEST(test_ctrl_app_auth_counter);
    RUN_TEST(test_app_reads);
    RUN_TEST(test_app_page1_write);
    RUN_TEST(test_app_start_announce);
    RUN_TEST(test_app_writes);
    RUN_TEST(test_app_fc_registration_start);
    RUN_TEST(test_app_fd_registration);
    RUN_TEST(test_sub02_component_write);
    RUN_TEST(test_sub02_push_ack);
    RUN_TEST(test_sub0607_writes);
    RUN_TEST(test_sub0607_arg_ff_ignored);
    tst_exit(UNITY_END());
    return 0;
}
