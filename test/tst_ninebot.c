/**
 * \file            tst_ninebot.c
 * \brief           Ninebot 5A A5 frames for tests, see the header
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_ninebot.h"
#include <stddef.h>
#include <string.h>
#include "ninebot.h" /* vcu_ninebot_process */
#include "usart.h"   /* NB_OFF_* (stored-frame layout the dispatcher takes) */

/** \brief Wire offset of the `len` byte (first checksummed byte). */
#define TST_NB_WIRE_LEN  2u
/** \brief Wire offset of the first body byte. */
#define TST_NB_WIRE_BODY 7u

/**
 * \brief           Ninebot checksum: one's complement of the 16-bit byte sum
 * \param[in]       from_len: first checksummed byte (the `len` field)
 * \param[in]       count: number of bytes (5 header fields + body)
 * \return          checksum as sent (low byte first on the wire)
 */
uint16_t tst_nb_checksum(const uint8_t* from_len, uint16_t count) {
    uint16_t sum = 0u;
    uint16_t idx;

    for (idx = 0u; idx < count; ++idx) {
        sum = (uint16_t)(sum + from_len[idx]);
    }
    return (uint16_t)~sum;
}

/**
 * \brief           Build a wire frame
 * \param[out]      out: destination buffer
 * \param[in]       out_size: size of `out` (needs `body_len + TST_NB_OVERHEAD`)
 * \param[in]       src: source node
 * \param[in]       dst: destination node
 * \param[in]       cmd: command
 * \param[in]       arg: argument
 * \param[in]       body: body bytes (may be `NULL` when `body_len` is 0)
 * \param[in]       body_len: body length
 * \return          wire length, or 0 if it does not fit
 */
uint16_t tst_nb_build(uint8_t* out, uint16_t out_size, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg,
                      const uint8_t* body, uint8_t body_len) {
    uint16_t total = (uint16_t)(body_len + TST_NB_OVERHEAD);
    uint16_t cksum;

    if ((out == NULL) || (out_size < total) || ((body == NULL) && (body_len > 0u))) {
        return 0u;
    }
    out[0] = TST_NB_HDR0;
    out[1] = TST_NB_HDR1;
    out[TST_NB_WIRE_LEN] = body_len;
    out[TST_NB_WIRE_LEN + 1u] = src;
    out[TST_NB_WIRE_LEN + 2u] = dst;
    out[TST_NB_WIRE_LEN + 3u] = cmd;
    out[TST_NB_WIRE_LEN + 4u] = arg;
    if (body_len > 0u) {
        memcpy(&out[TST_NB_WIRE_BODY], body, body_len);
    }
    cksum = tst_nb_checksum(&out[TST_NB_WIRE_LEN], (uint16_t)(body_len + 5u));
    out[TST_NB_WIRE_BODY + body_len] = (uint8_t)(cksum & 0xFFu);
    out[TST_NB_WIRE_BODY + body_len + 1u] = (uint8_t)(cksum >> 8);
    return total;
}

/**
 * \brief           Parse one wire frame
 * \param[in]       wire: frame bytes, starting at 5A A5
 * \param[in]       wire_len: number of bytes available
 * \param[out]      frame: the decoded fields
 * \return          `1` if the header and the length are consistent (see `checksum_ok`), else `0`
 */
uint8_t tst_nb_parse(const uint8_t* wire, uint16_t wire_len, tst_nb_frame_t* frame) {
    uint8_t ok = 0u;

    if ((wire == NULL) || (frame == NULL) || (wire_len < TST_NB_OVERHEAD)) {
        return 0u;
    }
    if ((wire[0] == TST_NB_HDR0) && (wire[1] == TST_NB_HDR1)
        && ((uint16_t)(wire[TST_NB_WIRE_LEN] + TST_NB_OVERHEAD) == wire_len)) {
        uint8_t len = wire[TST_NB_WIRE_LEN];
        uint16_t expect = tst_nb_checksum(&wire[TST_NB_WIRE_LEN], (uint16_t)(len + 5u));
        uint16_t got = (uint16_t)(wire[TST_NB_WIRE_BODY + len] | ((uint16_t)wire[TST_NB_WIRE_BODY + len + 1u] << 8));

        frame->len = len;
        frame->src = wire[TST_NB_WIRE_LEN + 1u];
        frame->dst = wire[TST_NB_WIRE_LEN + 2u];
        frame->cmd = wire[TST_NB_WIRE_LEN + 3u];
        frame->arg = wire[TST_NB_WIRE_LEN + 4u];
        frame->body = &wire[TST_NB_WIRE_BODY];
        frame->checksum_ok = (expect == got) ? 1u : 0u;
        ok = 1u;
    }
    return ok;
}

/**
 * \brief           Hand a request to the product dispatcher, as the USART/CAN receivers do
 * \note            Builds the stored frame (`len src dst cmd arg body`, no header and no
 *                      checksum: the layout the RX parser hands on) and calls
 *                      `vcu_ninebot_process(frame, transport, 0, 0)`.
 * \param[in]       transport: `VCU_TRANSPORT_USART` or `VCU_TRANSPORT_CAN`
 * \param[in]       src: source node
 * \param[in]       dst: destination node
 * \param[in]       cmd: command
 * \param[in]       arg: argument
 * \param[in]       body: body bytes (may be `NULL` when `body_len` is 0)
 * \param[in]       body_len: body length, at most `TST_NB_BODY_MAX`
 */
void tst_nb_request(int32_t transport, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg, const uint8_t* body,
                    uint8_t body_len) {
    uint8_t stored[NB_OFF_BODY + TST_NB_BODY_MAX + 1u];

    if ((body_len > TST_NB_BODY_MAX) || ((body == NULL) && (body_len > 0u))) {
        return;
    }
    memset(stored, 0, sizeof(stored));
    stored[NB_OFF_LEN] = body_len;
    stored[NB_OFF_SRC] = src;
    stored[NB_OFF_DST] = dst;
    stored[NB_OFF_CMD] = cmd;
    stored[NB_OFF_ARG] = arg;
    if (body_len > 0u) {
        memcpy(&stored[NB_OFF_BODY], body, body_len);
    }
    vcu_ninebot_process(stored, (int)transport, 0u, 0u);
}
