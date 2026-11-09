/**
 * \file            backup.c
 * \brief           Read-only backup: the VCU's own per-unit data, plus a relay to the motor
 *                      controller, behind one Ninebot backup command
 * \note            \ref vcu_backup_local_request answers for the VCU's own per-unit flash (the
 *                      identity records, the OTA boot descriptor and the factory UID) directly, in
 *                      the same read-only part protocol the motor controller speaks. \ref
 *                      vcu_backup_app_request forwards the request to the MCU as a \ref
 *                      CANID_BACKUP_REQ frame and remembers the one pending request; the matching
 *                      \ref CANID_BACKUP_RSP is relayed straight back by \ref
 *                      vcu_backup_can_response. The app serialises its requests (one in flight),
 *                      and every response echoes the part and offset for the app to check -- no
 *                      timer or large buffer is needed. Read-only by construction: a fixed source
 *                      allow-list, no write and no restore path.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "backup.h"

#include <stdint.h>

#include "can.h"     /* CANID_BACKUP_REQ, vcu_can_msg_t, vcu_can_tx_enqueue */
#include "ninebot.h" /* NB_CMD_BACKUP, nb_reply_pub                         */

/** \brief Classic CAN data-frame payload limit. */
#define BACKUP_MAX_PAYLOAD    8u

/* Flash layout of the VCU's per-unit data: the identity records, the OTA boot descriptor page and
   the factory UID. The firmware image is not backed up -- it is the same for every unit and is
   reflashed from the published image, so only the per-unit data a blank chip would be missing is
   read. The calibration/tune values live inside the identity record, so there is no separate
   calibration page here. */
#define BACKUP_IDENTITY_A     0x0801F000u /*!< identity record, page A                 */
#define BACKUP_IDENTITY_B     0x0801F400u /*!< identity record, page B (mirror)        */
#define BACKUP_OTA_DESC       0x0801F800u /*!< OTA boot descriptor page                */
#define BACKUP_PAGE_SIZE      1024u       /*!< flash page / erase granularity          */
#define BACKUP_UID_ADDR       0x1FFFF7E8u /*!< 96-bit factory UID                      */
#define BACKUP_UID_SIZE       12u

/** \brief Number of VCU-local backup parts (identity A, identity B, OTA descriptor, factory UID). */
#define BACKUP_VCU_PART_COUNT 4u
/** \brief Payload data bytes carried per read response, after the 3-byte part/offset header. */
#define BACKUP_CHUNK_LEN      5u

/** \brief Request sub-command markers (first request byte, after the node selector). */
#define BACKUP_REQ_COUNT      0xFFu /*!< ask for the part count       */
#define BACKUP_REQ_META       0xFEu /*!< ask for one part's size/dest */

/**
 * \brief           One backup part: a contiguous source region and where a restore writes it
 * \note            \ref dest_addr is informational, for the restore tool; the UID part is factory
 *                      read-only, so its destination is 0 (not restorable).
 */
typedef struct {
    uint32_t src_addr;  /*!< memory-mapped source address to read from */
    uint32_t dest_addr; /*!< flash destination a restore would write (0 = not restorable) */
    uint16_t size;      /*!< region length in bytes                    */
} backup_part_t;

/** \brief The fixed set of VCU-local parts a blank-chip restore needs (the read allow-list). */
static const backup_part_t k_vcu_parts[BACKUP_VCU_PART_COUNT] = {
    {BACKUP_IDENTITY_A, BACKUP_IDENTITY_A, BACKUP_PAGE_SIZE},
    {BACKUP_IDENTITY_B, BACKUP_IDENTITY_B, BACKUP_PAGE_SIZE},
    {BACKUP_OTA_DESC, BACKUP_OTA_DESC, BACKUP_PAGE_SIZE},
    {BACKUP_UID_ADDR, 0u, BACKUP_UID_SIZE},
};

/** \brief The one outstanding app request awaiting the MCU response. */
static struct {
    uint8_t active;   /*!< a request is in flight                    */
    int transport;    /*!< bus to reply on (\ref vcu_transport_t)    */
    uint8_t app_addr; /*!< app node address to reply to              */
} prv_pending;

/**
 * \brief           Send a VCU-local part-count reply to the app
 * \param[in]       transport: bus to reply on
 * \param[in]       app_addr: app node address to reply to
 */
static void prv_local_count(int transport, uint8_t app_addr) {
    uint8_t rsp[2];

    rsp[0] = BACKUP_REQ_COUNT;
    rsp[1] = (uint8_t)BACKUP_VCU_PART_COUNT;
    nb_reply_pub(transport, app_addr, NB_CMD_BACKUP, rsp, 2u);
}

/**
 * \brief           Send one VCU-local part's size and restore destination
 * \param[in]       transport: bus to reply on
 * \param[in]       app_addr: app node address to reply to
 * \param[in]       part: part index, 0 .. BACKUP_VCU_PART_COUNT-1
 */
static void prv_local_meta(int transport, uint8_t app_addr, uint8_t part) {
    const backup_part_t* p = &k_vcu_parts[part];
    uint8_t rsp[8];

    rsp[0] = BACKUP_REQ_META;
    rsp[1] = part;
    rsp[2] = (uint8_t)(p->size & 0xFFu);
    rsp[3] = (uint8_t)(p->size >> 8);
    rsp[4] = (uint8_t)(p->dest_addr & 0xFFu);
    rsp[5] = (uint8_t)((p->dest_addr >> 8) & 0xFFu);
    rsp[6] = (uint8_t)((p->dest_addr >> 16) & 0xFFu);
    rsp[7] = (uint8_t)((p->dest_addr >> 24) & 0xFFu);
    nb_reply_pub(transport, app_addr, NB_CMD_BACKUP, rsp, 8u);
}

/**
 * \brief           Send one chunk of a VCU-local part's bytes at the requested offset
 * \param[in]       transport: bus to reply on
 * \param[in]       app_addr: app node address to reply to
 * \param[in]       part: part index
 * \param[in]       offset: byte offset into the part
 */
static void prv_local_chunk(int transport, uint8_t app_addr, uint8_t part, uint16_t offset) {
    const backup_part_t* p = &k_vcu_parts[part];
    const volatile uint8_t* src = (const volatile uint8_t*)p->src_addr;
    uint8_t rsp[3u + BACKUP_CHUNK_LEN];
    uint16_t remain;
    uint8_t n;
    uint8_t i;

    rsp[0] = part;
    rsp[1] = (uint8_t)(offset & 0xFFu);
    rsp[2] = (uint8_t)(offset >> 8);

    remain = (offset < p->size) ? (uint16_t)(p->size - offset) : 0u;
    n = (remain < BACKUP_CHUNK_LEN) ? (uint8_t)remain : (uint8_t)BACKUP_CHUNK_LEN;
    for (i = 0u; i < n; i++) {
        rsp[3u + i] = src[offset + i];
    }
    nb_reply_pub(transport, app_addr, NB_CMD_BACKUP, rsp, (uint8_t)(3u + n));
}

void vcu_backup_local_request(int transport, uint8_t app_addr, const uint8_t* req, uint8_t req_len) {
    uint8_t sel;
    uint16_t offset;

    if ((req == 0) || (req_len < 1u)) {
        return;
    }

    sel = req[0];
    if (sel == BACKUP_REQ_COUNT) {
        prv_local_count(transport, app_addr);
    } else if (sel == BACKUP_REQ_META) {
        if ((req_len >= 2u) && (req[1] < BACKUP_VCU_PART_COUNT)) {
            prv_local_meta(transport, app_addr, req[1]);
        }
    } else if ((sel < BACKUP_VCU_PART_COUNT) && (req_len >= 3u)) {
        offset = (uint16_t)((uint16_t)req[1] | ((uint16_t)req[2] << 8));
        prv_local_chunk(transport, app_addr, sel, offset);
    }
}

void vcu_backup_app_request(int transport, uint8_t app_addr, const uint8_t* req, uint8_t req_len) {
    vcu_can_msg_t msg;
    uint8_t n;
    uint8_t i;

    n = (req_len > BACKUP_MAX_PAYLOAD) ? (uint8_t)BACKUP_MAX_PAYLOAD : req_len;
    msg.std_id = CANID_BACKUP_REQ;
    msg.ext_id = 0u;
    msg.ide = 0u;
    msg.rtr = 0u;
    msg.dlc = n;
    for (i = 0u; i < BACKUP_MAX_PAYLOAD; i++) {
        msg.data[i] = ((i < n) && (req != 0)) ? req[i] : 0u;
    }
    msg.fmi = 0u;
    vcu_can_tx_enqueue(&msg);

    prv_pending.active = 1u;
    prv_pending.transport = transport;
    prv_pending.app_addr = app_addr;
}

void vcu_backup_can_response(const volatile uint8_t* data, uint8_t len) {
    uint8_t buf[BACKUP_MAX_PAYLOAD];
    uint8_t n;
    uint8_t i;

    if ((prv_pending.active == 0u) || (data == 0)) {
        return;
    }
    n = (len > BACKUP_MAX_PAYLOAD) ? (uint8_t)BACKUP_MAX_PAYLOAD : len;
    for (i = 0u; i < n; i++) {
        buf[i] = data[i];
    }
    prv_pending.active = 0u;
    nb_reply_pub(prv_pending.transport, prv_pending.app_addr, NB_CMD_BACKUP, buf, n);
}
