/**
 * \file            test_ota.c
 * \brief           OTA receiver (ota.c)
 * \note            Covers the dispatcher (op 7 PREPARE, 8 DATA, 9 COMMIT, 10 APPLY), the TEA
 *                      decrypt, the session reset, the descriptor writer, the page erase (with its
 *                      page count), program + verify, the slot-1 erase and the boot descriptor
 *                      sync.
 *                      Flash goes through a recorder (tst_ota.h) that logs every erase/program
 *                      address. Writes reach the emulated flash (emulator memory only) except in
 *                      a protected range; slot 1 is written for real only when the test image
 *                      ends below it (checked, else the test is ignored).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "can.h"
#include "config.h"
#include "ninebot.h"
#include "ota.h"
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_ota.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/* Wire values. */
#define NODE_VCU        0x16u
#define NODE_REQ        0x23u /* requester: the reply goes back to it */
#define CMD_PREPARE     0x07u
#define CMD_DATA        0x08u
#define CMD_COMMIT      0x09u
#define CMD_APPLY       0x0Au
#define CMD_REPLY       0x0Bu /* reply cmd byte */
#define ST_OK           0x00u
#define ST_LEN          0x01u
#define ST_ERASE        0x02u
#define ST_FLASH        0x03u
#define ST_SEQ          0x05u
#define ST_STATE        0x07u
#define ST_CHECKSUM     0x08u
#define ST_NO_PREPARE   0x09u
#define TRANSPORT_USART 0
#define TRANSPORT_CAN   1
#define TRANSPORT_OTHER 2

/* Flash layout, descriptor magic and image-header limits. */
#define SLOT1           0x08010000u
#define DESC_ADDR       0x0801F800u
#define IDENTITY_A      0x0801F000u
#define PAGE            1024u
#define DESC_HALFWORDS  6u
#define DESC_MAGIC      0x0000505Au
#define SLACK           128u
#define MAX_LEN         0xF000u
#define SP_GOOD         0x20003000u
#define SP_BAD          0x08001000u
#define RETX_TICKS      500
#define CAN_REPLY_ID    0x18EF2316u /* prio 6, PF 0xEF, dst 0x23, src 0x16 */
#define CAN_REPLY_CTL   0x01u

/* Test streams. */
#define CHUNK           128u
#define IMAGE_LEN       2056u /* 16 x 128 + 8: two 1 KB key steps, a short last chunk */
#define FRAME_MAX       (NB_OFF_BODY + 248u + 8u)

static uint8_t g_image[IMAGE_LEN];
static uint8_t g_frame[FRAME_MAX];

/**
 * \brief           Send one OTA request through the dispatcher, as the Ninebot RX path does
 * \param[in]       transport: 0 USART, 1 CAN, other: no reply
 * \param[in]       cmd: 7..10
 * \param[in]       seq: argument byte (the DATA sequence number)
 * \param[in]       body: request body (`len` bytes, may be NULL when 0)
 * \param[in]       len: body length (frame[NB_OFF_LEN])
 */
static void prv_send(int32_t transport, uint8_t cmd, uint8_t seq, const uint8_t* body, uint32_t len) {
    memset(g_frame, 0, sizeof(g_frame));
    g_frame[NB_OFF_LEN] = (uint8_t)len;
    g_frame[NB_OFF_SRC] = NODE_REQ;
    g_frame[NB_OFF_DST] = NODE_VCU;
    g_frame[NB_OFF_CMD] = cmd;
    g_frame[NB_OFF_ARG] = seq;
    if (len > 0u) {
        memcpy(&g_frame[NB_OFF_BODY], body, len);
    }
    vcu_ota_dispatch(g_frame, (int)transport, 0u, 0u);
}

/**
 * \brief           Send one request over USART and return the status of its single reply
 * \param[in]       cmd: 7..10
 * \param[in]       seq: argument byte
 * \param[in]       body: request body
 * \param[in]       len: body length
 * \return          reply status
 * \note            Asserts the reply shape: src 0x16, dst = requester, cmd 0x0B, one body
 *                      byte = the request's sequence byte.
 */
static uint8_t prv_ota(uint8_t cmd, uint8_t seq, const uint8_t* body, uint32_t len) {
    tst_nb_frame_t frame;
    const tst_usart_frame_t* raw;

    /* The USART capture keeps 8 frames per reset: start each request with a fresh one. */
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    tst_ota_flash_resume();
    prv_send(TRANSPORT_USART, cmd, seq, body, len);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, tst_usart_drain(), "one OTA reply");
    raw = tst_usart_tx_get(0u);
    TEST_ASSERT_NOT_NULL(raw);
    TEST_ASSERT_TRUE(tst_nb_parse(raw->bytes, raw->len, &frame));
    TEST_ASSERT_TRUE(frame.checksum_ok);
    TEST_ASSERT_EQUAL_HEX8(NODE_VCU, frame.src);
    TEST_ASSERT_EQUAL_HEX8(NODE_REQ, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(CMD_REPLY, frame.cmd);
    TEST_ASSERT_EQUAL_UINT8(1u, frame.len);
    TEST_ASSERT_EQUAL_HEX8(seq, frame.body[0]);
    return frame.arg;
}

/**
 * \brief           PREPARE with a 4-byte little-endian length
 * \param[in]       declared: image length
 * \return          reply status
 */
static uint8_t prv_prepare(uint32_t declared) {
    uint8_t body[4];

    memcpy(body, &declared, sizeof(body));
    return prv_ota(CMD_PREPARE, 0u, body, sizeof(body));
}

/**
 * \brief           COMMIT with a 4-byte body
 * \return          reply status
 */
static uint8_t prv_commit(void) {
    const uint8_t body[4] = {0u, 0u, 0u, 0u};

    return prv_ota(CMD_COMMIT, 0u, body, sizeof(body));
}

/**
 * \brief           Fill g_image with a recognisable plaintext: valid SP, pattern, checksum word
 * \param[in]       len: image length
 * \param[in]       first_word: word 0 (the vector table's initial SP)
 */
static void prv_make_image(uint32_t len, uint32_t first_word) {
    uint32_t pos;
    uint32_t sum;

    for (pos = 0u; pos < len; ++pos) {
        g_image[pos] = (uint8_t)((pos * 7u) ^ (pos >> 8) ^ 0xA5u);
    }
    memcpy(g_image, &first_word, sizeof(first_word));
    sum = tst_ota_checksum(g_image, len);
    memcpy(&g_image[len - 4u], &sum, sizeof(sum));
}

/**
 * \brief           Encrypt and send g_image[offset..offset+len) as DATA `seq`
 * \param[in,out]   enc: stream state
 * \param[in]       offset: plaintext offset
 * \param[in]       len: chunk length
 * \param[in]       seq: sequence byte
 * \return          reply status
 */
static uint8_t prv_data(tst_ota_enc_t* enc, uint32_t offset, uint32_t len, uint8_t seq) {
    uint8_t chunk[0xF8];

    memcpy(chunk, &g_image[offset], len);
    tst_ota_enc_chunk(enc, chunk, len);
    return prv_ota(CMD_DATA, seq, chunk, len);
}

/**
 * \brief           Stream the whole g_image in CHUNK-sized DATA requests, asserting each status
 * \param[in]       len: image length
 * \param[in]       last_status: expected status of the final chunk
 */
static void prv_stream(uint32_t len, uint8_t last_status) {
    tst_ota_enc_t enc;
    uint32_t offset = 0u;
    uint8_t seq = 0u;

    tst_ota_enc_init(&enc, len);
    while (offset < len) {
        uint32_t part = ((len - offset) < CHUNK) ? (len - offset) : CHUNK;
        uint8_t expect = ((offset + part) >= len) ? last_status : ST_OK;

        TEST_ASSERT_EQUAL_HEX8_MESSAGE(expect, prv_data(&enc, offset, part, seq), "DATA status");
        offset += part;
        ++seq;
    }
}

/**
 * \brief           Word `idx` of the boot descriptor in flash
 * \param[in]       idx: 0 magic, 1 commit, 2 length
 * \return          the word
 */
static uint32_t prv_desc(uint32_t idx) { return ((const volatile uint32_t*)DESC_ADDR)[idx]; }

/**
 * \brief           Program a boot descriptor into flash directly (test set-up)
 * \param[in]       magic: word 0
 * \param[in]       commit: word 1
 * \param[in]       length: word 2
 */
static void prv_put_desc(uint32_t magic, uint32_t commit, uint32_t length) {
    const uint32_t words[3] = {magic, commit, length};

    TEST_ASSERT_TRUE(tst_ota_flash_raw_erase(DESC_ADDR));
    TEST_ASSERT_TRUE(tst_ota_flash_raw_write(DESC_ADDR, (const uint16_t*)words, DESC_HALFWORDS));
}

/**
 * \brief           Assert flash operation `idx` (fakes' operation log) is an erase of `addr`
 * \param[in]       idx: operation index
 * \param[in]       addr: expected sector address
 */
static void prv_assert_erase_op(uint32_t idx, uint32_t addr) {
    const tst_flash_op_t* op = tst_flash_op_get(idx);

    TEST_ASSERT_NOT_NULL(op);
    TEST_ASSERT_EQUAL_INT(TST_FLASH_ERASE, op->kind);
    TEST_ASSERT_EQUAL_HEX32(addr, op->addr);
}

/**
 * \brief           Assert flash operation `idx` programs the 12-byte descriptor {magic, commit, length}
 * \param[in]       idx: operation index
 * \param[in]       commit: expected word 1
 * \param[in]       length: expected word 2
 */
static void prv_assert_desc_op(uint32_t idx, uint32_t commit, uint32_t length) {
    const uint32_t words[3] = {DESC_MAGIC, commit, length};
    const tst_flash_op_t* op = tst_flash_op_get(idx);

    TEST_ASSERT_NOT_NULL(op);
    TEST_ASSERT_EQUAL_INT(TST_FLASH_PROGRAM, op->kind);
    TEST_ASSERT_EQUAL_HEX32(DESC_ADDR, op->addr);
    TEST_ASSERT_EQUAL_UINT32(sizeof(words), op->len);
    TEST_ASSERT_NOT_NULL(op->data);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t*)words, op->data, sizeof(words));
}

/**
 * \brief           Start a fresh flash log; only the identity pages are record-only
 * \note            Slot 1 is erased and programmed for real (emulator memory), so the program
 *                      read-back verify sees what was written.
 */
static void prv_record(void) { tst_ota_flash_record(IDENTITY_A, DESC_ADDR); }

/**
 * \brief           Boot state as main() leaves it: descriptor synced, session reset, USART up
 * \note            Slot 1 is staged for real, which is only safe while the test image ends
 *                      below it: otherwise every test is ignored (and test_image_leaves_slot1_free
 *                      fails).
 */
void setUp(void) {
    if (tst_ota_image_end() > SLOT1) {
        TEST_IGNORE_MESSAGE("test image reaches into slot 1: cannot stage an image for real");
    }
    tst_reset_product_state();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    vcu_usart_init();
    prv_record();
    ota_desc_boot_sync();
    vcu_ota_init();
    prv_record();
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* -- helpers ------------------------------------------------------------------------------- */

/**
 * \brief           The test image ends below slot 1, so staging there cannot overwrite it
 */
static void test_image_leaves_slot1_free(void) { TEST_ASSERT_LESS_OR_EQUAL_HEX32(SLOT1, tst_ota_image_end()); }

/**
 * \brief           The test-side TEA encoder matches the published TEA vector
 * \note            enc(k = 0, v = 0) = 41EA3A0A 94BAA940 (Wheeler/Needham). ota.c's decrypt is
 *                      the inverse, checked by the full-image test.
 */
static void test_helper_tea_reference_vector(void) {
    const uint32_t key[4] = {0u, 0u, 0u, 0u};
    uint32_t block[2] = {0u, 0u};

    tst_ota_tea_encrypt(block, key);
    TEST_ASSERT_EQUAL_HEX32(0x41EA3A0Au, block[0]);
    TEST_ASSERT_EQUAL_HEX32(0x94BAA940u, block[1]);
}

/* -- PREPARE (op 7) ------------------------------------------------------------------------ */

/**
 * \brief           PREPARE clears the descriptor commit word, then erases slot 1 for len + 0x80
 * \note            desc.commit = 0, ota_write_descriptor (erase the descriptor page for 12
 *                      bytes = 1 page, program 6 halfwords), then the slot-1 erase for len + 0x80,
 *                      rounded up to pages -> 0x880 = 3 pages. The length word keeps its old
 *                      value.
 */
static void test_prepare_erases_descriptor_then_slot1(void) {
    const tst_ota_flash_log_t* log = tst_ota_flash_log();
    uint32_t idx;

    prv_put_desc(DESC_MAGIC, 1u, 0x1234u);
    ota_desc_boot_sync();
    prv_record();

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(0x800u));
    TEST_ASSERT_EQUAL_UINT32(4u, log->erase_count);
    TEST_ASSERT_EQUAL_HEX32(DESC_ADDR, log->erase_addr[0]);
    TEST_ASSERT_EQUAL_HEX32(SLOT1, log->erase_addr[1]);
    TEST_ASSERT_EQUAL_HEX32(SLOT1 + PAGE, log->erase_addr[2]);
    TEST_ASSERT_EQUAL_HEX32(SLOT1 + (2u * PAGE), log->erase_addr[3]);
    TEST_ASSERT_EQUAL_UINT32(DESC_HALFWORDS, log->program_count);
    for (idx = 0u; idx < DESC_HALFWORDS; ++idx) {
        TEST_ASSERT_EQUAL_HEX32(DESC_ADDR + (2u * idx), log->program_addr[idx]);
    }
    TEST_ASSERT_EQUAL_HEX32(DESC_MAGIC, prv_desc(0u));
    TEST_ASSERT_EQUAL_HEX32(0u, prv_desc(1u));
    TEST_ASSERT_EQUAL_HEX32(0x1234u, prv_desc(2u));

    /* The same sequence in the fakes' operation log, with the programmed bytes. */
    TEST_ASSERT_EQUAL_UINT32(5u, tst_flash_op_count());
    prv_assert_erase_op(0u, DESC_ADDR);
    prv_assert_desc_op(1u, 0u, 0x1234u);
    prv_assert_erase_op(2u, SLOT1);
    prv_assert_erase_op(3u, SLOT1 + PAGE);
    prv_assert_erase_op(4u, SLOT1 + (2u * PAGE));
}

/**
 * \brief           PREPARE accepts 1..0xF000 in multiples of 8; the maximum erases into identity page A
 * \note            `(len - 1) < 0xF000 && (len & 7) == 0`. 0xF000 + 0x80 is 0x3C.2 pages ->
 *                      61 pages from slot 1: the last one is identity page A (record-only here).
 */
static void test_prepare_max_length_page_span(void) {
    const tst_ota_flash_log_t* log = tst_ota_flash_log();

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(MAX_LEN));
    TEST_ASSERT_EQUAL_UINT32(1u + 61u, log->erase_count);
    TEST_ASSERT_EQUAL_HEX32(SLOT1, log->erase_addr[1]);
    TEST_ASSERT_EQUAL_HEX32(IDENTITY_A, log->erase_addr[61]);
    TEST_ASSERT_EQUAL_HEX32(0x0801EC00u, log->erase_addr[60]);
}

/**
 * \brief           PREPARE rejects length 0, a length not a multiple of 8 and lengths above 0xF000
 * \note            Status 1, nothing erased (the session reset runs first).
 */
static void test_prepare_rejects_bad_length(void) {
    const tst_ota_flash_log_t* log = tst_ota_flash_log();

    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_prepare(0u));
    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_prepare(0x804u));
    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_prepare(MAX_LEN + 8u));
    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_prepare(0xFFFFFFF8u));
    TEST_ASSERT_EQUAL_UINT32(0u, log->erase_count);
    TEST_ASSERT_EQUAL_UINT32(0u, log->program_count);
}

/**
 * \brief           PREPARE whose body is not exactly 4 bytes is a state error (7)
 * \note            Body length must be 4, else status 7.
 */
static void test_prepare_rejects_body_not_4(void) {
    const uint8_t body[8] = {0x00u, 0x08u, 0u, 0u, 0u, 0u, 0u, 0u};

    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_ota(CMD_PREPARE, 0u, body, 8u));
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_ota(CMD_PREPARE, 0u, body, 3u));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->erase_count);
}

/**
 * \brief           A failing descriptor erase answers 3; a failing slot-1 erase answers 2
 * \note            ota_write_descriptor == 0 -> 3. The slot-1 erase returns 2 when the page
 *                      erase stops early.
 */
static void test_prepare_flash_failures(void) {
    tst_ota_flash_fail_at(DESC_ADDR);
    TEST_ASSERT_EQUAL_HEX8(ST_FLASH, prv_prepare(0x800u));

    prv_record();
    tst_ota_flash_fail_at(SLOT1 + PAGE);
    TEST_ASSERT_EQUAL_HEX8(ST_ERASE, prv_prepare(0x800u));
    TEST_ASSERT_EQUAL_UINT32(1u + 2u, tst_ota_flash_log()->erase_count); /* stops at the failing page */
}

/* -- DATA (op 8) --------------------------------------------------------------------------- */

/**
 * \brief           DATA before PREPARE answers 9
 * \note            `declared_len == 0` -> status 9.
 */
static void test_data_before_prepare(void) {
    uint8_t chunk[8] = {0u};

    TEST_ASSERT_EQUAL_HEX8(ST_NO_PREPARE, prv_ota(CMD_DATA, 0u, chunk, sizeof(chunk)));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->program_count);
}

/**
 * \brief           A whole image streams, decrypts into slot 1, verifies and commits the descriptor
 * \note            DATA: TEA + CBC with the previous cipher block (none for block 0 of the
 *                      image), program at slot 1 + offset (read-back verified), checksum ~rol16
 *                      of the word sum against the last word, key bytes += index at each 1 KB
 *                      offset. COMMIT: desc = {0x505A, 1, len}.
 */
static void test_full_image_stages_and_commits(void) {
    const tst_ota_flash_log_t* log = tst_ota_flash_log();

    prv_make_image(IMAGE_LEN, SP_GOOD);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    prv_stream(IMAGE_LEN, ST_OK);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(g_image, (const uint8_t*)SLOT1, IMAGE_LEN);
    TEST_ASSERT_EQUAL_HEX32(SLOT1, log->program_min);
    TEST_ASSERT_EQUAL_HEX32(SLOT1 + IMAGE_LEN - 2u, log->program_max);

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_commit());
    TEST_ASSERT_EQUAL_HEX32(DESC_MAGIC, prv_desc(0u));
    TEST_ASSERT_EQUAL_HEX32(1u, prv_desc(1u));
    TEST_ASSERT_EQUAL_HEX32(IMAGE_LEN, prv_desc(2u));

    /* COMMIT resets the session: a second one is refused. */
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_commit());
}

/**
 * \brief           A wrong trailing checksum answers 8 and leaves nothing to commit
 * \note            `~rol16(sum) != word` -> status 8, then the
 *                      common error tail resets the session; COMMIT answers 7.
 */
static void test_checksum_mismatch(void) {
    prv_make_image(IMAGE_LEN, SP_GOOD);
    g_image[IMAGE_LEN - 1u] ^= 0x01u;
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_stream(IMAGE_LEN, ST_CHECKSUM);
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_commit());
    TEST_ASSERT_EQUAL_HEX32(0u, prv_desc(1u));
}

/**
 * \brief           The first block's initial SP must point to RAM; else 7 and nothing is programmed
 * \note            seq 0 at offset 0, `(word & 0x2FFE0000) != 0x20000000` -> status 7.
 */
static void test_data_rejects_bad_initial_sp(void) {
    tst_ota_enc_t enc;

    prv_make_image(IMAGE_LEN, SP_BAD);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    tst_ota_enc_init(&enc, IMAGE_LEN);
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_data(&enc, 0u, CHUNK, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->program_count);
    /* The error tail reset the session: the next DATA has no PREPARE. */
    TEST_ASSERT_EQUAL_HEX8(ST_NO_PREPARE, prv_data(&enc, 0u, CHUNK, 0u));
}

/**
 * \brief           Chunk length 0 or not a multiple of 8 answers 1
 * \note            `(len & 7) != 0 || len == 0` -> status 1.
 */
static void test_data_rejects_bad_chunk_length(void) {
    uint8_t chunk[16] = {0u};

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_ota(CMD_DATA, 0u, chunk, 12u));
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_ota(CMD_DATA, 0u, chunk, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->program_count);
}

/**
 * \brief           A chunk that would end at or past declared + 0x80 answers 1
 * \note            `offset + len < declared + 0x80` else status 1.
 *                      Declared 8, first chunk 0x88 bytes: 0x88 is not < 0x88.
 */
static void test_data_rejects_overrun(void) {
    tst_ota_enc_t enc;

    prv_make_image(0x88u, SP_GOOD);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(8u));
    prv_record();
    tst_ota_enc_init(&enc, 8u);
    TEST_ASSERT_EQUAL_HEX8(ST_LEN, prv_data(&enc, 0u, 0x88u, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->program_count);
}

/**
 * \brief           A retransmit of the previous sequence is answered 0 and not programmed again
 * \note            `seq == expected - 1` -> status stays 0, no
 *                      flash access; the session goes on with the expected sequence.
 */
static void test_data_retransmit_is_acknowledged(void) {
    tst_ota_enc_t enc;
    uint32_t programs;

    prv_make_image(IMAGE_LEN, SP_GOOD);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    tst_ota_enc_init(&enc, IMAGE_LEN);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_data(&enc, 0u, CHUNK, 0u));
    programs = tst_ota_flash_log()->program_count;
    TEST_ASSERT_EQUAL_UINT32(CHUNK / 2u, programs);

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_ota(CMD_DATA, 0u, g_image, CHUNK));
    TEST_ASSERT_EQUAL_UINT32(programs, tst_ota_flash_log()->program_count);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_data(&enc, CHUNK, CHUNK, 1u));
    TEST_ASSERT_EQUAL_HEX32(SLOT1 + (2u * CHUNK) - 2u, tst_ota_flash_log()->program_max);
}

/**
 * \brief           Any other sequence answers 5 and ends the session
 * \note            Status 5, then the session reset in the error tail.
 */
static void test_data_wrong_sequence(void) {
    tst_ota_enc_t enc;

    prv_make_image(IMAGE_LEN, SP_GOOD);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    tst_ota_enc_init(&enc, IMAGE_LEN);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_data(&enc, 0u, CHUNK, 0u));
    TEST_ASSERT_EQUAL_HEX8(ST_SEQ, prv_data(&enc, CHUNK, CHUNK, 2u));
    TEST_ASSERT_EQUAL_HEX8(ST_NO_PREPARE, prv_data(&enc, CHUNK, CHUNK, 1u));
}

/**
 * \brief           Right after PREPARE, sequence 0xFF is not "the previous one": it answers 5
 * \note            `seq == expected - 1` is a 32-bit compare, so expected 0 gives -1, which no
 *                      byte equals. (An 8-bit `expected - 1` would wrap to 0xFF and answer 0.)
 */
static void test_data_seq_ff_after_prepare_is_wrong(void) {
    uint8_t chunk[8] = {0u};

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    TEST_ASSERT_EQUAL_HEX8(ST_SEQ, prv_ota(CMD_DATA, 0xFFu, chunk, sizeof(chunk)));
}

/**
 * \brief           A failing halfword program answers 3 and ends the session
 * \note            A failed program -> status 3; the halfword loop stops at the first
 *                      failure.
 */
static void test_data_program_failure(void) {
    tst_ota_enc_t enc;

    prv_make_image(IMAGE_LEN, SP_GOOD);
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    tst_ota_flash_fail_at(SLOT1 + 6u);
    tst_ota_enc_init(&enc, IMAGE_LEN);
    TEST_ASSERT_EQUAL_HEX8(ST_FLASH, prv_data(&enc, 0u, CHUNK, 0u));
    TEST_ASSERT_EQUAL_UINT32(4u, tst_ota_flash_log()->program_count);
    TEST_ASSERT_EQUAL_HEX8(ST_NO_PREPARE, prv_data(&enc, CHUNK, CHUNK, 1u));
}

/* -- COMMIT (op 9) / APPLY (op 10) / others ------------------------------------------------ */

/**
 * \brief           COMMIT needs a 4-byte body and a complete, verified image; else 7, no flash
 * \note            `len == 4 && complete && verified` else 7.
 */
static void test_commit_preconditions(void) {
    const uint8_t body[4] = {0u};

    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_commit());
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    prv_record();
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_commit());
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_ota(CMD_COMMIT, 0u, body, 2u));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->erase_count);
}

/**
 * \brief           APPLY without a committed descriptor answers 0 and does nothing
 * \note            Reset only when `desc.commit != 0 && verified == 0`; otherwise the common
 *                      reply with status 0. (The reset path, identity_store + 100 ms +
 *                      SYSRESETREQ, ends the run and is not tested. Clearing reg 0x31 bit14 just
 *                      before it, in ota_apply_reset, sits in that same untested tail.)
 */
static void test_apply_without_commit(void) {
    prv_put_desc(DESC_MAGIC, 0u, 0u);
    ota_desc_boot_sync();
    prv_record();
    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_ota(CMD_APPLY, 0u, NULL, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->erase_count);
}

/**
 * \brief           An unknown OTA command answers 7
 * \note            Falls through to status 7.
 */
static void test_unknown_command(void) { TEST_ASSERT_EQUAL_HEX8(ST_STATE, prv_ota(0x0Cu, 0u, NULL, 0u)); }

/**
 * \brief           Over CAN the reply is 0x18EF2316 `01 0B <status> <seq>`, after arming the timer
 * \note            g_candev+0x0E = 500, +0x0C |= 1, then report_ninebot(0x16, src, 1, 0x0B,
 *                      status, &seq).
 */
static void test_reply_over_can(void) {
    const tst_can_frame_t* frame;

    g_candev.event_wait = 0;
    g_candev.event_flags = 0u;
    prv_send(TRANSPORT_CAN, 0x0Cu, 0x5Au, NULL, 0u);
    TEST_ASSERT_EQUAL_INT16(RETX_TICKS, g_candev.event_wait);
    TEST_ASSERT_EQUAL_HEX16(1u, g_candev.event_flags & 1u);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT32(1u, tst_can_tx_count());
    frame = tst_can_tx_get(0u);
    TEST_ASSERT_NOT_NULL(frame);
    TEST_ASSERT_EQUAL_UINT8(1u, frame->ext);
    TEST_ASSERT_EQUAL_HEX32(CAN_REPLY_ID, frame->id);
    TEST_ASSERT_EQUAL_UINT8(4u, frame->dlc);
    TEST_ASSERT_EQUAL_HEX8(CAN_REPLY_CTL, frame->data[0]);
    TEST_ASSERT_EQUAL_HEX8(CMD_REPLY, frame->data[1]);
    TEST_ASSERT_EQUAL_HEX8(ST_STATE, frame->data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x5Au, frame->data[3]);
}

/**
 * \brief           A transport other than 0 / 1 gets no reply at all
 * \note            `transport == 0` USART, `== 1` CAN, else nothing.
 */
static void test_no_reply_on_other_transport(void) {
    prv_send(TRANSPORT_OTHER, 0x0Cu, 0u, NULL, 0u);
    vcu_can_poll();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_usart_drain());
    TEST_ASSERT_EQUAL_UINT32(0u, tst_can_tx_count());
}

/**
 * \brief           vcu_ota_init ends a session in progress (session reset)
 * \note            ota_session_reset zeroes the session struct incl. declared_len (+0x0C).
 */
static void test_init_resets_session(void) {
    uint8_t chunk[8] = {0u};

    TEST_ASSERT_EQUAL_HEX8(ST_OK, prv_prepare(IMAGE_LEN));
    vcu_ota_init();
    TEST_ASSERT_EQUAL_HEX8(ST_NO_PREPARE, prv_ota(CMD_DATA, 0u, chunk, sizeof(chunk)));
}

/* -- boot descriptor ------------------------------------------------------------------------ */

/**
 * \brief           Boot sync rewrites a descriptor without magic as {0x505A, 0, 0}
 * \note            Copy the descriptor page (12 bytes) to RAM; if word 0 != 0x505A, set
 *                      {0x505A, 0, 0} and ota_write_descriptor (erase 1 page, program 6
 *                      halfwords).
 */
static void test_boot_sync_rewrites_bad_magic(void) {
    const tst_ota_flash_log_t* log = tst_ota_flash_log();

    prv_put_desc(0x1111u, 1u, 0x40u);
    tst_fakes_reset();
    prv_record();
    ota_desc_boot_sync();
    TEST_ASSERT_EQUAL_UINT32(2u, tst_flash_op_count());
    prv_assert_erase_op(0u, DESC_ADDR);
    prv_assert_desc_op(1u, 0u, 0u);
    TEST_ASSERT_EQUAL_UINT32(1u, log->erase_count);
    TEST_ASSERT_EQUAL_HEX32(DESC_ADDR, log->erase_addr[0]);
    TEST_ASSERT_EQUAL_UINT32(DESC_HALFWORDS, log->program_count);
    TEST_ASSERT_EQUAL_HEX32(DESC_MAGIC, prv_desc(0u));
    TEST_ASSERT_EQUAL_HEX32(0u, prv_desc(1u));
    TEST_ASSERT_EQUAL_HEX32(0u, prv_desc(2u));

    TEST_ASSERT_TRUE(tst_ota_flash_raw_erase(DESC_ADDR));
    prv_record();
    ota_desc_boot_sync();
    TEST_ASSERT_EQUAL_UINT32(1u, log->erase_count);
    TEST_ASSERT_EQUAL_HEX32(DESC_MAGIC, prv_desc(0u));
    TEST_ASSERT_EQUAL_HEX32(0u, prv_desc(2u));
}

/**
 * \brief           Boot sync leaves a descriptor with magic alone (no flash access)
 * \note            No rewrite when word 0 matches.
 */
static void test_boot_sync_keeps_valid_descriptor(void) {
    prv_put_desc(DESC_MAGIC, 1u, 0x2468u);
    ota_desc_boot_sync();
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->erase_count);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->program_count);
    TEST_ASSERT_EQUAL_HEX32(1u, prv_desc(1u));
    TEST_ASSERT_EQUAL_HEX32(0x2468u, prv_desc(2u));
}

/**
 * \brief           The descriptor writer refuses a RAM copy without magic
 * \note            ota_write_descriptor has a magic guard (the boot sync always sets the magic
 *                      first, so the guard does not fire in normal operation). Pinned here:
 *                      before any boot sync, nothing is erased or programmed and 0 is returned.
 */
static void test_write_descriptor_guard_without_magic(void) {
    tst_reset_product_state();
    prv_record();
    TEST_ASSERT_EQUAL_INT(0, ota_write_descriptor());
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->erase_count);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_ota_flash_log()->program_count);
}

/** \brief Suite entry \return never returns */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_image_leaves_slot1_free);
    RUN_TEST(test_helper_tea_reference_vector);
    RUN_TEST(test_prepare_erases_descriptor_then_slot1);
    RUN_TEST(test_prepare_max_length_page_span);
    RUN_TEST(test_prepare_rejects_bad_length);
    RUN_TEST(test_prepare_rejects_body_not_4);
    RUN_TEST(test_prepare_flash_failures);
    RUN_TEST(test_data_before_prepare);
    RUN_TEST(test_full_image_stages_and_commits);
    RUN_TEST(test_checksum_mismatch);
    RUN_TEST(test_data_rejects_bad_initial_sp);
    RUN_TEST(test_data_rejects_bad_chunk_length);
    RUN_TEST(test_data_rejects_overrun);
    RUN_TEST(test_data_retransmit_is_acknowledged);
    RUN_TEST(test_data_wrong_sequence);
    RUN_TEST(test_data_seq_ff_after_prepare_is_wrong);
    RUN_TEST(test_data_program_failure);
    RUN_TEST(test_commit_preconditions);
    RUN_TEST(test_apply_without_commit);
    RUN_TEST(test_unknown_command);
    RUN_TEST(test_reply_over_can);
    RUN_TEST(test_no_reply_on_other_transport);
    RUN_TEST(test_init_resets_session);
    RUN_TEST(test_boot_sync_rewrites_bad_magic);
    RUN_TEST(test_boot_sync_keeps_valid_descriptor);
    RUN_TEST(test_write_descriptor_guard_without_magic);
    tst_exit(UNITY_END());
    return 0;
}
