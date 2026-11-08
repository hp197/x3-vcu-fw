/**
 * \file            backup.c
 * \brief           Read-only backup relay: the service app reads the motor controller's
 *                      per-unit data through the VCU
 * \note            A transparent relay between the app's Ninebot backup command and the motor
 *                      controller's read-only CAN backup channel. An app request is forwarded to
 *                      the MCU as a \ref CANID_BACKUP_REQ frame and the one pending request is
 *                      remembered; the matching \ref CANID_BACKUP_RSP is relayed straight back to
 *                      the app. The app serialises its requests (one in flight), so a new request
 *                      simply replaces any stale pending one, and every response echoes the part
 *                      and offset for the app to check -- no timer or large buffer is needed.
 *                      It forwards read requests and relays bytes only: there is no write or
 *                      restore path here.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "backup.h"

#include <stdint.h>

#include "can.h"     /* CANID_BACKUP_REQ, vcu_can_msg_t, vcu_can_tx_enqueue */
#include "ninebot.h" /* NB_CMD_BACKUP, nb_reply_pub                         */

/** \brief Classic CAN data-frame payload limit. */
#define BACKUP_MAX_PAYLOAD 8u

/** \brief The one outstanding app request awaiting the MCU response. */
static struct {
    uint8_t active;   /*!< a request is in flight                    */
    int transport;    /*!< bus to reply on (\ref vcu_transport_t)    */
    uint8_t app_addr; /*!< app node address to reply to              */
} prv_pending;

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
