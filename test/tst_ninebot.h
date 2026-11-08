/**
 * \file            tst_ninebot.h
 * \brief           Ninebot 5A A5 frames for tests: build, parse, checksum, send a request
 * \note            Written from the wire format, independently of usart.c, so a test checks
 *                      the product's framing instead of reusing it:
 *                        5A A5 | len | src | dst | cmd | arg | body[len] | ck_lo ck_hi
 *                      ck = ~(sum of the bytes from `len` to the last body byte), 16-bit LE.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_NINEBOT_H
#define TST_NINEBOT_H

#include <stdint.h>

#define TST_NB_HDR0     0x5Au /*!< wire byte 0 */
#define TST_NB_HDR1     0xA5u /*!< wire byte 1 */
#define TST_NB_OVERHEAD 9u    /*!< header 2 + len/src/dst/cmd/arg 5 + checksum 2 */
#define TST_NB_BODY_MAX 242u  /*!< longest body the product sends (usart.c `< 243`) */
#define TST_NB_WIRE_MAX (TST_NB_BODY_MAX + TST_NB_OVERHEAD)

/** \brief A parsed wire frame (`body` points into the parsed buffer). */
typedef struct {
    uint8_t len;         /*!< body length */
    uint8_t src;         /*!< source node */
    uint8_t dst;         /*!< destination node */
    uint8_t cmd;         /*!< command */
    uint8_t arg;         /*!< argument */
    const uint8_t* body; /*!< `len` body bytes */
    uint8_t checksum_ok; /*!< 1 if the trailing checksum matches */
} tst_nb_frame_t;

uint16_t tst_nb_checksum(const uint8_t* from_len, uint16_t count);
uint16_t tst_nb_build(uint8_t* out, uint16_t out_size, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg,
                      const uint8_t* body, uint8_t body_len);
uint8_t tst_nb_parse(const uint8_t* wire, uint16_t wire_len, tst_nb_frame_t* frame);
void tst_nb_request(int32_t transport, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg, const uint8_t* body,
                    uint8_t body_len);

#endif /* TST_NINEBOT_H */
