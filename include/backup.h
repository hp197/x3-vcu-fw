/**
 * \file            backup.h
 * \brief           Read-only backup: the service app reads the per-unit data of the VCU itself and
 *                      of the motor controller, both through the one Ninebot backup command
 * \note            The app sends \ref NB_CMD_BACKUP; the first body byte selects the node:
 *                      \ref BACKUP_NODE_VCU reads the VCU's own per-unit flash and \ref
 *                      BACKUP_NODE_MCU relays to the motor controller over CAN. Every node speaks
 *                      the same read-only part protocol on the bytes after the node selector:
 *                      - `[0xFF]`                 -> `[0xFF, num_parts]`
 *                      - `[0xFE, part]`           -> `[0xFE, part, size_lo, size_hi, dest0..dest3]`
 *                      - `[part, off_lo, off_hi]` -> `[part, off_lo, off_hi, bytes...]`
 *                      The app asks for the part count, then each part's size and restore
 *                      destination, then streams the bytes offset by offset, validating every
 *                      chunk by the echoed part and offset. Strictly read-only: it only forwards
 *                      read requests and relays or reads bytes back -- it never writes, and there
 *                      is no restore path.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef BACKUP_H
#define BACKUP_H

#include <stdint.h>

/** \brief Backup node selector: the first \ref NB_CMD_BACKUP body byte. */
#define BACKUP_NODE_VCU 0x00u /*!< read the VCU's own per-unit flash (local)        */
#define BACKUP_NODE_MCU 0x01u /*!< relay to the motor controller over CAN           */

/**
 * \brief           Answer an app backup request for the VCU's own per-unit data
 * \param[in]       transport: the bus the app request arrived on (reply goes back on it)
 * \param[in]       app_addr: the app node address to reply to
 * \param[in]       req: the part-protocol request bytes (after the node selector)
 * \param[in]       req_len: request length in bytes
 */
void vcu_backup_local_request(int transport, uint8_t app_addr, const uint8_t* req, uint8_t req_len);

/**
 * \brief           Forward an app backup request to the motor controller over CAN
 * \param[in]       transport: the bus the app request arrived on (reply goes back on it)
 * \param[in]       app_addr: the app node address to reply to
 * \param[in]       req: the raw MCU backup request bytes (after the node selector)
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
