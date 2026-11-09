/**
 * \file            test_backup.c
 * \brief           Read-only backup responder + node routing (backup.c, ninebot.c dispatch)
 * \note            Drives a NB_CMD_BACKUP frame in on USART2 (the app side) and checks the reply
 *                      captured at the BSP boundary. The first body byte selects the node: node 0
 *                      is answered locally from the VCU's own per-unit flash, node 1 is relayed to
 *                      the motor controller over CAN. Read-only: the responder only ever reads the
 *                      allow-listed regions and never writes.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "backup.h"
#include "can.h"
#include "ninebot.h"
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/* Node addresses and the backup sub-protocol as they appear on the wire. */
#define NODE_VCU        0x16u
#define NODE_APP        0x23u
#define REQ_COUNT       0xFFu
#define REQ_META        0xFEu

/* The VCU-local parts, their sizes and restore destinations. */
#define PART_IDENTITY_A 0u
#define PART_UID        3u
#define VCU_PART_COUNT  4u
#define PAGE_SIZE       1024u
#define UID_SIZE        12u
#define IDENTITY_A_ADDR 0x0801F000u
#define CAN_BACKUP_REQ  0x6E0u
#define CHUNK_DATA_LEN  5u

/**
 * \brief           Parse the USART reply frame captured at \p index, failing on a bad frame
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
 * \brief           Drive one backup request and return the number of USART replies it produced
 * \param[in]       node: node selector (first body byte)
 * \param[in]       req: the part-protocol bytes after the selector (may be NULL when \p req_len is 0)
 * \param[in]       req_len: number of bytes in \p req
 * \return          number of reply frames the VCU put on USART2
 */
static uint32_t prv_backup(uint8_t node, const uint8_t* req, uint8_t req_len) {
    uint8_t body[8];
    uint8_t i;

    body[0] = node;
    for (i = 0u; i < req_len; i++) {
        body[1u + i] = req[i];
    }
    tst_nb_request(VCU_TRANSPORT_USART, NODE_APP, NODE_VCU, NB_CMD_BACKUP, 0u, body, (uint8_t)(req_len + 1u));
    return tst_usart_drain();
}

void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_flash_forward(0u);  /* record flash ops only                       */
    tst_can_tx_forward(0u); /* CAN frames captured, mailbox 0 reported free */
    vcu_usart_init();
    nb_key_load();
}

void tearDown(void) {}

/* -- node 0: the VCU answers for its own per-unit data ------------------------------------ */

/**
 * \brief           The part-count query returns the VCU's own part count
 */
static void test_vcu_part_count(void) {
    const uint8_t req[] = {REQ_COUNT};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_HEX8(NODE_APP, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(NB_CMD_BACKUP, frame.cmd);
    TEST_ASSERT_EQUAL_UINT8(2u, frame.len);
    TEST_ASSERT_EQUAL_HEX8(REQ_COUNT, frame.body[0]);
    TEST_ASSERT_EQUAL_UINT8(VCU_PART_COUNT, frame.body[1]);
}

/**
 * \brief           The meta query returns identity A's page size and its flash restore address
 */
static void test_vcu_meta_identity_a(void) {
    const uint8_t req[] = {REQ_META, PART_IDENTITY_A};
    const uint8_t want[] = {REQ_META,
                            PART_IDENTITY_A,
                            (uint8_t)(PAGE_SIZE & 0xFFu),
                            (uint8_t)(PAGE_SIZE >> 8),
                            (uint8_t)(IDENTITY_A_ADDR & 0xFFu),
                            (uint8_t)((IDENTITY_A_ADDR >> 8) & 0xFFu),
                            (uint8_t)((IDENTITY_A_ADDR >> 16) & 0xFFu),
                            (uint8_t)((IDENTITY_A_ADDR >> 24) & 0xFFu)};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_HEX8(NB_CMD_BACKUP, frame.cmd);
    TEST_ASSERT_EQUAL_UINT8(8u, frame.len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, frame.body, 8u);
}

/**
 * \brief           The factory UID part reports a zero restore destination (not restorable)
 */
static void test_vcu_meta_uid_not_restorable(void) {
    const uint8_t req[] = {REQ_META, PART_UID};
    const uint8_t want[] = {REQ_META, PART_UID, UID_SIZE, 0u, 0u, 0u, 0u, 0u};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_UINT8(8u, frame.len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, frame.body, 8u);
}

/**
 * \brief           A chunk read echoes the part and offset and returns the identity flash bytes
 */
static void test_vcu_chunk_reads_identity_flash(void) {
    const uint8_t req[] = {PART_IDENTITY_A, 0u, 0u};
    const volatile uint8_t* id_a = (const volatile uint8_t*)IDENTITY_A_ADDR;
    tst_nb_frame_t frame;
    uint8_t i;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_UINT8(3u + CHUNK_DATA_LEN, frame.len);
    TEST_ASSERT_EQUAL_HEX8(PART_IDENTITY_A, frame.body[0]);
    TEST_ASSERT_EQUAL_HEX8(0u, frame.body[1]);
    TEST_ASSERT_EQUAL_HEX8(0u, frame.body[2]);
    for (i = 0u; i < CHUNK_DATA_LEN; i++) {
        TEST_ASSERT_EQUAL_HEX8(id_a[i], frame.body[3u + i]);
    }
}

/**
 * \brief           The final chunk of a part is clamped to the bytes that remain
 */
static void test_vcu_chunk_tail_is_clamped(void) {
    const uint16_t offset = (uint16_t)(PAGE_SIZE - 4u);
    const uint8_t req[] = {PART_IDENTITY_A, (uint8_t)(offset & 0xFFu), (uint8_t)(offset >> 8)};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_UINT8(3u + 4u, frame.len); /* only 4 bytes remain in the page */
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(offset & 0xFFu), frame.body[1]);
}

/**
 * \brief           A read at or past the end of a part returns a header with no data bytes
 */
static void test_vcu_chunk_past_end_is_empty(void) {
    const uint8_t req[] = {PART_IDENTITY_A, (uint8_t)(PAGE_SIZE & 0xFFu), (uint8_t)(PAGE_SIZE >> 8)};
    tst_nb_frame_t frame;

    TEST_ASSERT_EQUAL_UINT32(1u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
    prv_usart_frame(0u, &frame);
    TEST_ASSERT_EQUAL_UINT8(3u, frame.len);
}

/**
 * \brief           A part index past the allow-list is ignored (no reply)
 */
static void test_vcu_part_out_of_range_is_ignored(void) {
    const uint8_t req[] = {VCU_PART_COUNT, 0u, 0u};

    TEST_ASSERT_EQUAL_UINT32(0u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
}

/**
 * \brief           A meta query for a part past the allow-list is ignored (no reply)
 */
static void test_vcu_meta_out_of_range_is_ignored(void) {
    const uint8_t req[] = {REQ_META, VCU_PART_COUNT};

    TEST_ASSERT_EQUAL_UINT32(0u, prv_backup(BACKUP_NODE_VCU, req, sizeof(req)));
}

/**
 * \brief           A backup command with only the node selector and no request is ignored
 */
static void test_vcu_empty_request_is_ignored(void) {
    TEST_ASSERT_EQUAL_UINT32(0u, prv_backup(BACKUP_NODE_VCU, 0, 0u));
}

/* -- node 1: relayed to the motor controller over CAN ------------------------------------ */

/**
 * \brief           The MCU node enqueues a CAN backup request and sends no local reply yet
 */
static void test_mcu_node_relays_over_can(void) {
    const uint8_t req[] = {REQ_COUNT};
    const tst_can_frame_t* can_frame;
    uint32_t round;

    TEST_ASSERT_EQUAL_UINT32(0u, prv_backup(BACKUP_NODE_MCU, req, sizeof(req)));
    for (round = 0u; round < 8u; round++) {
        vcu_can_poll();
    }
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    can_frame = tst_can_tx_get(0u);
    TEST_ASSERT_NOT_NULL(can_frame);
    TEST_ASSERT_EQUAL_UINT8(0u, can_frame->ext);
    TEST_ASSERT_EQUAL_HEX32(CAN_BACKUP_REQ, can_frame->id);
    TEST_ASSERT_EQUAL_UINT8(1u, can_frame->dlc);
    TEST_ASSERT_EQUAL_HEX8(REQ_COUNT, can_frame->data[0]);
}

/**
 * \brief           A node that is neither VCU nor MCU is dropped: no reply, no relay
 */
static void test_unknown_node_is_ignored(void) {
    const uint8_t req[] = {REQ_COUNT};
    uint32_t round;

    TEST_ASSERT_EQUAL_UINT32(0u, prv_backup(0x02u, req, sizeof(req)));
    for (round = 0u; round < 8u; round++) {
        vcu_can_poll();
    }
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
}

/**
 * \brief           Run the suite
 * \return          0
 */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_vcu_part_count);
    RUN_TEST(test_vcu_meta_identity_a);
    RUN_TEST(test_vcu_meta_uid_not_restorable);
    RUN_TEST(test_vcu_chunk_reads_identity_flash);
    RUN_TEST(test_vcu_chunk_tail_is_clamped);
    RUN_TEST(test_vcu_chunk_past_end_is_empty);
    RUN_TEST(test_vcu_part_out_of_range_is_ignored);
    RUN_TEST(test_vcu_meta_out_of_range_is_ignored);
    RUN_TEST(test_vcu_empty_request_is_ignored);
    RUN_TEST(test_mcu_node_relays_over_can);
    RUN_TEST(test_unknown_node_is_ignored);
    tst_exit(UNITY_END());
    return 0;
}
