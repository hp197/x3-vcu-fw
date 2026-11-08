/**
 * \file            test_identity.c
 * \brief           Identity record store (config.c)
 * \note            Covers the checksum (int16 sum of the 256 halfwords, less the two of the +0x1F4
 *                      word), the A/B loader (higher +0x1FE version copied to IN2 and checked, with
 *                      config_serial_commit fallbacks) and the store (version += 1, even -> page A /
 *                      odd -> page B, one sector erase then 0x200 B programmed halfword by halfword
 *                      with read-back). Every write is checked twice: the BSP fake log
 *                      (tst_flash_op_get) and the emulated flash pages.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "at32f415.h"
#include "config.h"
#include "control.h"
#include "ninebot.h"
#include "report.h"
#include "tst_bsp_fakes.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"

/* Flash layout. */
#define PAGE_A           0x0801F000u
#define PAGE_B           0x0801F400u
#define SECTOR_SIZE      1024u
#define REC_SIZE         512u
#define DESC_PAGE        0x0801F800u /* OTA boot descriptor page, written by its own code */
#define FLASH_LO         0x08000000u
#define FLASH_HI         0x08020000u
/* Record byte offsets. */
#define OFF_MAGIC        0x000u
#define OFF_SERIAL       0x020u
#define OFF_REG49        0x092u /* Ninebot reg n = IN2 + 2n */
#define OFF_IDENT        0x1B4u /* reg 0xDA */
#define OFF_BUILD_ID     0x1C8u
#define OFF_CHECKSUM     0x1F4u
#define OFF_VERSION      0x1FEu
#define MAGIC            0x505Cu
#define SERIAL_TEMPLATE  "03SP0000000001"
#define SERIAL_LEN       14u
/* Factory UID of the backed-up device, served by x3emu at 0x1FFFF7E8. */
#define UID_ADDR         0x1FFFF7E8u
#define UID_W0           0x005C359Au
#define UID_W1           0x0E06C000u
#define UID_W2           0x05C42703u
/* Ninebot requests. */
#define NODE_INFO        0x3Du
#define NODE_VCU         0x16u
#define CMD_WRITE_REPLY  0x02u
#define CMD_STATUS_REPLY 0x05u
#define REG_SPEED_LIMIT  0x46u
#define REG_AUTO_OFF     0x49u
#define RIDE_ROW_GATE    2u /* g_vs.region the reg 0x46 gate accepts */
#define CRC_GATE_LEN     18u
#define KEY_LEN          16u

static uint8_t g_page[REC_SIZE];

/**
 * \brief           Independent record checksum of a 512-byte record
 * \param[in]       rec: record bytes
 * \return          int16 sum of all halfwords minus the two at +0x1F4/+0x1F6
 */
static uint32_t prv_checksum(const uint8_t* rec) {
    int32_t sum = 0;
    uint32_t off;

    for (off = 0u; off < REC_SIZE; off += 2u) {
        if ((off != OFF_CHECKSUM) && (off != (OFF_CHECKSUM + 2u))) {
            sum += (int16_t)(uint16_t)(rec[off] | ((uint16_t)rec[off + 1u] << 8));
        }
    }
    return (uint32_t)sum;
}

/**
 * \brief           Read a little-endian halfword
 * \param[in]       src: first byte
 * \return          the value
 */
static uint16_t prv_rd16(const volatile uint8_t* src) { return (uint16_t)(src[0] | ((uint16_t)src[1] << 8)); }

/**
 * \brief           Read a little-endian word
 * \param[in]       src: first byte
 * \return          the value
 */
static uint32_t prv_rd32(const volatile uint8_t* src) {
    return (uint32_t)prv_rd16(src) | ((uint32_t)prv_rd16(src + 2u) << 16);
}

/**
 * \brief           Store a little-endian halfword / word into a byte buffer
 * \param[out]      dst: first byte
 * \param[in]       value: value
 * \param[in]       len: 2 or 4
 */
static void prv_wr(uint8_t* dst, uint32_t value, uint32_t len) {
    uint32_t idx;

    for (idx = 0u; idx < len; ++idx) {
        dst[idx] = (uint8_t)(value >> (8u * idx));
    }
}

/**
 * \brief           Build a paired record in \ref g_page: magic, template serial, reg 0x49 = `marker`
 * \param[in]       version: version counter at +0x1FE
 * \param[in]       marker: reg 0x49 value, tells the pages apart
 * \param[in]       good_checksum: `1` valid checksum, `0` off by one
 */
static void prv_make(uint16_t version, uint8_t marker, uint8_t good_checksum) {
    memset(g_page, 0, sizeof(g_page));
    prv_wr(&g_page[OFF_MAGIC], MAGIC, 2u);
    memcpy(&g_page[OFF_SERIAL], SERIAL_TEMPLATE, SERIAL_LEN);
    g_page[OFF_REG49] = marker;
    prv_wr(&g_page[OFF_VERSION], version, 2u);
    prv_wr(&g_page[OFF_CHECKSUM], prv_checksum(g_page) + ((good_checksum != 0u) ? 0u : 1u), 4u);
}

/**
 * \brief           Erase a page and, unless `rec` is `NULL`, program the record into it
 * \note            Test set-up through the (recording) BSP; the log is cleared afterwards.
 * \param[in]       page: page address
 * \param[in]       rec: 512 record bytes, or `NULL` to leave the page erased
 */
static void prv_page_put(uint32_t page, const uint8_t* rec) {
    uint32_t off;

    flash_unlock();
    TEST_ASSERT_EQUAL(FLASH_OPERATE_DONE, flash_sector_erase(page));
    for (off = 0u; (rec != NULL) && (off < REC_SIZE); off += 2u) {
        TEST_ASSERT_EQUAL(FLASH_OPERATE_DONE, flash_halfword_program(page + off, prv_rd16(&rec[off])));
    }
    flash_lock();
    tst_fakes_reset();
}

/**
 * \brief           Word-sum hash of a flash range (emulated flash, memory-mapped)
 * \param[in]       lo: first address
 * \param[in]       hi: end address (exclusive)
 * \return          hash
 */
static uint32_t prv_hash(uint32_t lo, uint32_t hi) {
    uint32_t hash = 0u;
    uint32_t addr;

    for (addr = lo; addr < hi; addr += 4u) {
        hash = (hash * 31u) + *(const volatile uint32_t*)addr;
    }
    return hash;
}

/**
 * \brief           The live RAM record (IN2)
 * \return          its bytes
 */
static volatile uint8_t* prv_ram(void) { return (volatile uint8_t*)&g_in2; }

/**
 * \brief           Assert recorded flash operation `index`
 * \param[in]       index: operation index
 * \param[in]       kind: erase / program
 * \param[in]       addr: expected address
 * \param[in]       len: expected length
 */
static void prv_assert_op(uint32_t index, tst_flash_kind_t kind, uint32_t addr, uint32_t len) {
    const tst_flash_op_t* op = tst_flash_op_get(index);

    TEST_ASSERT_NOT_NULL_MESSAGE(op, "flash operation missing");
    TEST_ASSERT_EQUAL_INT(kind, op->kind);
    TEST_ASSERT_EQUAL_HEX32(addr, op->addr);
    TEST_ASSERT_EQUAL_UINT32(len, op->len);
    TEST_ASSERT_EQUAL(FLASH_OPERATE_DONE, op->status);
}

/**
 * \brief           Assert one save: erase + 512-byte program of `page`, bytes = RAM = flash
 * \param[in]       first_op: index of the erase in the operation log
 * \param[in]       page: expected page
 * \param[in]       version: expected version
 */
static void prv_assert_save(uint32_t first_op, uint32_t page, uint16_t version) {
    const volatile uint8_t* flash = (const volatile uint8_t*)page;
    const tst_flash_op_t* prog;
    uint32_t off;

    prv_assert_op(first_op, TST_FLASH_ERASE, page, SECTOR_SIZE);
    prv_assert_op(first_op + 1u, TST_FLASH_PROGRAM, page, REC_SIZE);
    prog = tst_flash_op_get(first_op + 1u);
    TEST_ASSERT_NOT_NULL(prog->data);
    TEST_ASSERT_EQUAL_HEX16(version, prv_rd16(&prog->data[OFF_VERSION]));
    TEST_ASSERT_EQUAL_HEX32(prv_checksum(prog->data), prv_rd32(&prog->data[OFF_CHECKSUM]));
    for (off = 0u; off < REC_SIZE; ++off) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(prog->data[off], flash[off], "flash differs from the programmed bytes");
    }
    for (off = REC_SIZE; off < SECTOR_SIZE; ++off) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xFFu, flash[off], "sector tail not erased");
    }
}

/**
 * \brief           Independent reflected CRC-32 (IEEE), as the reg 0x46 gate uses
 * \param[in]       data: bytes
 * \param[in]       len: length
 * \return          CRC-32
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
 * \brief           Fresh boot RAM and fakes for every test (the flash pages are set per test)
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
}

/**
 * \brief           Nothing to undo
 */
void tearDown(void) {}

/* -- loader (identity_load) -------------------------------------------------------------------- */

/**
 * \brief           A valid page with the higher version is loaded; nothing is written
 * \note            verB < verA -> copy page A, checksum ok -> active = verA, no identity_store
 *                      call. Both directions.
 */
static void test_load_takes_higher_version(void) {
    prv_make(6u, 0xA6u, 1u);
    prv_page_put(PAGE_A, g_page);
    prv_make(5u, 0xB5u, 1u);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX8(0xA6u, prv_ram()[OFF_REG49]);
    TEST_ASSERT_EQUAL_HEX16(6u, prv_rd16(&prv_ram()[OFF_VERSION]));

    prv_make(7u, 0xB7u, 1u);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX8(0xB7u, prv_ram()[OFF_REG49]);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_flash_op_count());
}

/**
 * \brief           Higher page with a bad checksum: the other page is loaded and rewritten
 * \note            State 2: B (5) fails the check -> config_serial_commit, copy A (4), ok ->
 *                      active 4, identity_store -> version 5 (odd) -> page B gets A's content.
 */
static void test_load_bad_checksum_falls_back(void) {
    prv_make(4u, 0xA4u, 1u);
    prv_page_put(PAGE_A, g_page);
    prv_make(5u, 0xB5u, 0u);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX8(0xA4u, prv_ram()[OFF_REG49]);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_B, 5u);
    TEST_ASSERT_EQUAL_HEX8(0xA4u, ((const volatile uint8_t*)PAGE_B)[OFF_REG49]);
}

/**
 * \brief           Erased page B (version 0xFFFF, bad checksum) is restored from page A
 * \note            0xFFFF > 4 selects B, its all-0xFF checksum fails, A is loaded and
 *                      saved as version 5 to page B.
 */
static void test_load_erased_page_is_restored(void) {
    prv_make(4u, 0xA4u, 1u);
    prv_page_put(PAGE_A, g_page);
    prv_page_put(PAGE_B, NULL);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX8(0xA4u, prv_ram()[OFF_REG49]);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_B, 5u);
}

/**
 * \brief           Equal versions: page A is taken WITHOUT a checksum check and re-saved
 * \note            State 3, fault codes still 0: active = verA, copy page A, identity_store ->
 *                      version 7 -> page B. So a corrupt A wins over a valid B of the same
 *                      version.
 */
static void test_load_equal_versions_takes_a_unchecked(void) {
    prv_make(6u, 0xA6u, 0u);
    prv_page_put(PAGE_A, g_page);
    prv_make(6u, 0xB6u, 1u);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX8(0xA6u, prv_ram()[OFF_REG49]);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_B, 7u);
}

/**
 * \brief           Both pages erased: page A (all 0xFF) is loaded, saved as version 0 to page A
 * \note            0xFFFF == 0xFFFF -> state 3 -> active 0xFFFF, +1 wraps to 0 (u16 at
 *                      STATE+4), even -> page A. The magic stays 0xFFFF (not paired), which
 *                      vcu_config_init then replaces with the factory defaults.
 */
static void test_load_both_erased(void) {
    prv_page_put(PAGE_A, NULL);
    prv_page_put(PAGE_B, NULL);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, prv_rd16(&prv_ram()[OFF_MAGIC]));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_A, 0u);
}

/**
 * \brief           Both pages corrupt and the serial blank: nothing is written, magic 0xFFFF
 * \note            B fails -> config_serial_commit = 2 (blank serial), A fails -> 2; state 3
 *                      with neither code 0/1 -> STATE+4 = 0, *IN2 = 0xFFFF, no identity_store.
 */
static void test_load_both_corrupt_writes_nothing(void) {
    prv_make(4u, 0xA4u, 0u);
    memset(&g_page[OFF_SERIAL], 0, SERIAL_LEN);
    prv_page_put(PAGE_A, g_page);
    prv_make(5u, 0xB5u, 0u);
    memset(&g_page[OFF_SERIAL], 0, SERIAL_LEN);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(-1, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, prv_rd16(&prv_ram()[OFF_MAGIC]));
    TEST_ASSERT_EQUAL_UINT32(0u, tst_flash_op_count());
}

/* -- store (identity_store) --------------------------------------------------------------------- */

/**
 * \brief           Saves alternate B (odd) / A (even) and touch nothing else in flash
 * \note            The rest of flash (bootloader, app slot, the
 *                      test image, slot 1, the boot descriptor page) hashes the same before and
 *                      after, in the emulated flash.
 */
static void test_store_alternates_pages_only(void) {
    uint32_t below;
    uint32_t above;

    prv_make(4u, 0xA4u, 1u);
    prv_page_put(PAGE_A, g_page);
    prv_make(3u, 0xB3u, 1u);
    prv_page_put(PAGE_B, g_page);
    below = prv_hash(FLASH_LO, PAGE_A);
    above = prv_hash(DESC_PAGE, FLASH_HI);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_INT(0, identity_store((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_INT(0, identity_store((identity_rec_t*)(void*)&g_in2));
    TEST_ASSERT_EQUAL_UINT32(4u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_B, 5u);
    prv_assert_save(2u, PAGE_A, 6u);
    TEST_ASSERT_EQUAL_HEX16(6u, prv_rd16(&prv_ram()[OFF_VERSION]));
    TEST_ASSERT_EQUAL_HEX32(below, prv_hash(FLASH_LO, PAGE_A));
    TEST_ASSERT_EQUAL_HEX32(above, prv_hash(DESC_PAGE, FLASH_HI));
}

/**
 * \brief           A reg 0x49 write is saved by the deferred action: 07 00 at +0x92 of page B
 * \note            The write path ends with IN1+0x10 = 1; control_deferred_action clears it
 *                      and calls identity_store: erase + program of page B with 07 00 at +0x92
 *                      after the write.
 */
static void test_reg_write_is_saved(void) {
    const uint8_t body[] = {7u, 0u};

    prv_make(4u, 0x01u, 1u);
    prv_page_put(PAGE_A, g_page);
    prv_make(3u, 0x01u, 1u);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, REG_AUTO_OFF, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT8(1u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_UINT32(0u, tst_flash_op_count());
    control_deferred_action();
    TEST_ASSERT_EQUAL_UINT8(0u, g_in1.recompute_req);
    TEST_ASSERT_EQUAL_UINT32(2u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_B, 5u);
    TEST_ASSERT_EQUAL_HEX16(7u, prv_rd16((const volatile uint8_t*)(PAGE_B + OFF_REG49)));
}

/* -- fixed-address reads -------------------------------------------------------------------------- */

/**
 * \brief           config_identity_sync stamps the build id from the app slot, then saves twice
 * \note            IN2+0x1CE = 0, IN2+0x1C8..+0x1CD = the 6-byte build id from the slot-0
 *                      header (CONFIG_BUILD_ID: the git short hash as text, or 6 x 0xFF when
 *                      unset), then two identity_store calls.
 */
static void test_sync_stamps_build_id(void) {
    static const char build_id[CONFIG_BUILD_ID_LEN] = CONFIG_BUILD_ID;
    uint8_t expect[CONFIG_BUILD_ID_LEN + 2u];
    const volatile uint8_t* page_a = (const volatile uint8_t*)PAGE_A;
    uint32_t idx;
    uint32_t op;

    memcpy(expect, build_id, CONFIG_BUILD_ID_LEN);
    expect[CONFIG_BUILD_ID_LEN] = 0x00u; /* +0x1CE cleared */
    expect[CONFIG_BUILD_ID_LEN + 1u] = 0x00u;
    prv_make(4u, 0xA4u, 1u);
    prv_page_put(PAGE_A, g_page);
    prv_make(3u, 0xB3u, 1u);
    prv_page_put(PAGE_B, g_page);
    TEST_ASSERT_EQUAL_INT(0, identity_load((identity_rec_t*)(void*)&g_in2));
    memcpy((void*)g_in2.build_id, "abcdef", sizeof(g_in2.build_id));
    g_in2.build_id_1ce = 0x5555u;
    (void)config_identity_sync();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4u, tst_flash_op_count());
    prv_assert_save(0u, PAGE_B, 5u);
    prv_assert_save(2u, PAGE_A, 6u);
    for (op = 4u; op < tst_flash_op_count(); ++op) {
        TEST_ASSERT_TRUE_MESSAGE(tst_flash_op_get(op)->addr >= DESC_PAGE, "write outside the boot descriptor");
    }
    for (idx = 0u; idx < sizeof(expect); ++idx) {
        TEST_ASSERT_EQUAL_HEX8(expect[idx], page_a[OFF_BUILD_ID + idx]);
    }
}

/**
 * \brief           The auth key loaded from 0x08001420 is 16 x 0xFF: the reg 0x46 CRC gate passes
 * \note            nb_key_load copies 0x10 bytes from 0x08001420 (16 x 0xFF, erased flash)
 *                      to the RAM key; the gate CRCs body[0] + key + body[9] against body[5..8].
 *                      Before nb_key_load the key is zero and the same frame fails.
 */
static void test_auth_key_is_erased_flash(void) {
    uint8_t gate[CRC_GATE_LEN];
    uint8_t body[10] = {25u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 1u};
    tst_nb_frame_t frame;
    uint32_t crc;

    gate[0] = body[0];
    memset(&gate[1], 0xFF, KEY_LEN);
    gate[1u + KEY_LEN] = body[9];
    crc = prv_crc32(gate, sizeof(gate));
    body[5] = (uint8_t)(crc >> 24);
    body[6] = (uint8_t)(crc >> 16);
    body[7] = (uint8_t)(crc >> 8);
    body[8] = (uint8_t)crc;
    vcu_usart_init();
    g_vs.region = RIDE_ROW_GATE;

    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, REG_SPEED_LIMIT, body, sizeof(body));
    nb_key_load();
    tst_nb_request(VCU_TRANSPORT_USART, NODE_INFO, NODE_VCU, CMD_WRITE_REPLY, REG_SPEED_LIMIT, body, sizeof(body));
    TEST_ASSERT_EQUAL_UINT32(2u, tst_usart_drain());
    TEST_ASSERT_TRUE(tst_nb_parse(tst_usart_tx_get(0u)->bytes, tst_usart_tx_get(0u)->len, &frame));
    TEST_ASSERT_EQUAL_HEX8(CMD_STATUS_REPLY, frame.cmd);
    TEST_ASSERT_EQUAL_HEX8(4u, frame.body[0]); /* zero key: rejected */
    TEST_ASSERT_TRUE(tst_nb_parse(tst_usart_tx_get(1u)->bytes, tst_usart_tx_get(1u)->len, &frame));
    TEST_ASSERT_EQUAL_HEX8(0u, frame.body[0]); /* 0xFF key: accepted */
}

/**
 * \brief           The factory UID (0x1FFFF7E8) is copied to IN2+0x1B4 (reg 0xDA)
 * \note            report_uid_telemetry_copy copies 3 words from 0x1FFFF7E8 to g_mcu_uid and
 *                      to IN2+0x1B4. x3emu serves the backed-up device's UID there.
 */
static void test_uid_copied_to_ident(void) {
    const volatile uint32_t* uid = (const volatile uint32_t*)UID_ADDR;

    TEST_ASSERT_EQUAL_HEX32(UID_W0, uid[0]);
    TEST_ASSERT_EQUAL_HEX32(UID_W1, uid[1]);
    TEST_ASSERT_EQUAL_HEX32(UID_W2, uid[2]);
    report_uid_telemetry_copy();
    TEST_ASSERT_EQUAL_HEX32(UID_W0, prv_rd32(&prv_ram()[OFF_IDENT]));
    TEST_ASSERT_EQUAL_HEX32(UID_W1, prv_rd32(&prv_ram()[OFF_IDENT + 4u]));
    TEST_ASSERT_EQUAL_HEX32(UID_W2, prv_rd32(&prv_ram()[OFF_IDENT + 8u]));
    TEST_ASSERT_EQUAL_HEX32(UID_W0, g_mcu_uid[0]);
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_load_takes_higher_version);
    RUN_TEST(test_load_bad_checksum_falls_back);
    RUN_TEST(test_load_erased_page_is_restored);
    RUN_TEST(test_load_equal_versions_takes_a_unchecked);
    RUN_TEST(test_load_both_erased);
    RUN_TEST(test_load_both_corrupt_writes_nothing);
    RUN_TEST(test_store_alternates_pages_only);
    RUN_TEST(test_reg_write_is_saved);
    RUN_TEST(test_sync_stamps_build_id);
    RUN_TEST(test_auth_key_is_erased_flash);
    RUN_TEST(test_uid_copied_to_ident);
    tst_exit(UNITY_END());
    return 0;
}
