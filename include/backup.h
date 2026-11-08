/**
 * \file            backup.h
 * \brief           Read-only backup relay: the service app reads the motor controller's
 *                      per-unit data through the VCU
 * \note            The app speaks the motor controller's read-only backup protocol straight
 *                      through the VCU. It sends a Ninebot backup command (\ref NB_CMD_BACKUP)
 *                      whose body carries the raw MCU request bytes; the VCU forwards them to
 *                      the MCU as a \ref CANID_BACKUP_REQ frame and, when the matching
 *                      \ref CANID_BACKUP_RSP arrives, relays the response bytes back to the app.
 *                      The app drives it part by part; each response echoes the part and offset,
 *                      so the app validates every chunk itself. Strictly read-only: this only
 *                      forwards read requests and relays the bytes back -- it never writes, and
 *                      there is no restore path.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef BACKUP_H
#define BACKUP_H

#include <stdint.h>

/**
 * \brief           Forward an app backup request to the motor controller over CAN
 * \param[in]       transport: the bus the app request arrived on (reply goes back on it)
 * \param[in]       app_addr: the app node address to reply to
 * \param[in]       req: the raw MCU backup request bytes (the command body)
 * \param[in]       req_len: request length in bytes (clamped to the 8-byte frame limit)
 */
void vcu_backup_app_request(int transport, uint8_t app_addr, const uint8_t* req, uint8_t req_len);

/**
 * \brief           Relay a motor-controller backup response back to the app
 * \param[in]       data: the \ref CANID_BACKUP_RSP payload bytes
 * \param[in]       len: payload length in bytes
 */
void vcu_backup_can_response(const volatile uint8_t* data, uint8_t len);

#endif /* BACKUP_H */
