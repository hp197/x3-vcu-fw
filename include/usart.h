/**
 * \file            usart.h
 * \brief           USART2 BLE/dashboard link + Ninebot 5A A5 parser (x3 VCU)
 * \note            USART2 driver on the Artery AT32F415 BSP. Declares the link bring-up and
 *                      RX pump, the Ninebot frame model and parser state, plus the shared report
 *                      framer (vcu_usart_report_send) and TX sequencer that every reporting module
 *                      uses.
 *
 *                      Functions (usart.c):
 *                        vcu_usart_init         (top-level bring-up)
 *                        usart_gpio_init        (PA2 TX / PA3 RX)
 *                        usart_cfg              (115200 8N1, TE/RE/DMAT/DMAR/UEN)
 *                        usart_rx_dma_init      (DMA1 Ch6, circular 250 B ring)
 *                        usart_tx_dma_init      (DMA1 Ch7, on-demand)
 *                        vcu_usart_poll         (re-entrancy-guarded RX pump)
 *                        usart_ring_drain       (circular DMA ring drain)
 *                        usart_parser_feed      (5A A5 byte-wise state machine)
 *                        usart_frame_cksum16    (one's-complement additive sum)
 *                        usart_frame_assemble   (5A A5 field-wise builder)
 *                        usart_tx_enqueue       (enqueue reply into TX ring)
 *                        usart_tx_dma_arm       (arm DMA1 Ch7 for one frame)
 *                      The main-loop TX path is owned here too:
 *                        vcu_usart_tx_sequencer (super-loop TX ring drain)
 *                        vcu_usart_report_send  (report -> ring enqueue)
 *                      The CAN 29-bit extended-id tunnel (mask 0x1FFFFFFF) is the CAN counterpart
 *                      of this framer and is owned by report.c, not here.
 *
 *                      USART2 has no interrupt handler: RX and TX both run on DMA.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef USART_H
#define USART_H

#include "at32f415.h"

/* ------------------------------------------------------------------------- */
/* Link + DMA constants.                                                     */
/* ------------------------------------------------------------------------- */

/** \brief USART2 line rate (0x1C200 baud) */
#define VCU_USART_BAUD_RATE   115200U
/** \brief USART2 BRR divider: PCLK1 / baud, rounded to nearest (needs board.h) */
#define VCU_USART_BAUD_DIV    ((((PCLK1_HZ * 10U) / VCU_USART_BAUD_RATE) + 5U) / 10U)

/** \brief RX DMA ring depth, 0xFA = 250 bytes (circular) */
#define VCU_USART_RX_RING_LEN 250U

/** \brief TX DMA buffer, 0x10 = 16 bytes (DMA memory base set at init) */
#define VCU_USART_TX_BUF_LEN  16U

/* ------------------------------------------------------------------------- */
/* Ninebot 5A A5 framing, as accepted by the RX parser.                      */
/*                                                                           */
/* On the wire:  5A A5 | len | <body...> | cksum_lo cksum_hi                 */
/*   - the two header bytes are consumed by the state machine and NOT stored */
/*   - `len` (the first stored byte) is the Ninebot length field             */
/*   - the parser's expected final index = len + 7 (fixed-field overhead)    */
/*   - cksum16 = one's-complement of the 16-bit additive sum over the stored */
/*     bytes excluding the two checksum bytes (classic Ninebot, NOT a CRC)   */
/* ------------------------------------------------------------------------- */

/** \brief Fixed-field overhead added to `len` to get the expected final index */
#define VCU_NINEBOT_OVERHEAD  7U

/** \brief Maximum accepted `len` field (the range check rejects `len > 0xEC`) */
#define VCU_NINEBOT_LEN_MAX   236U

/** \brief Frame-assembly buffer capacity: max final index = 0xEC + 7 = 243 */
#define VCU_USART_FRAME_MAX   243U

/* ------------------------------------------------------------------------- */
/* Ninebot frame header: the one vocabulary every module uses (usart, ninebot, */
/* busproto, report, ota, passcode, faultmon, control, ...).                    */
/*                                                                            */
/*   wire:   5A A5 | len | src | dst | cmd | arg | body[len] | ck_lo ck_hi       */
/*                                                                            */
/* NB_OFF_* index the stored frame -- the parser's assembly buffer and the      */
/* frame pointer the dispatcher receives start at the `len` byte; the 5A A5     */
/* header and the checksum are not stored. NB_WIRE() converts to the on-wire   */
/* index. src/dst: every frame the VCU sends has src = NB_ADDR_VCU, and the      */
/* dispatcher accepts dst == NB_ADDR_VCU (replies swap src/dst). The field      */
/* names follow the classic Ninebot layout.                                     */
/* ------------------------------------------------------------------------- */
#define NB_HDR0      0x5AU /*!< wire byte 0                               */
#define NB_HDR1      0xA5U /*!< wire byte 1                               */
#define NB_OFF_LEN   0U    /*!< body length                               */
#define NB_OFF_SRC   1U    /*!< source node address                       */
#define NB_OFF_DST   2U    /*!< destination node address                  */
#define NB_OFF_CMD   3U    /*!< command (register operation)              */
#define NB_OFF_ARG   4U    /*!< argument (register index / sub-code)      */
#define NB_OFF_BODY  5U    /*!< first body byte                           */
#define NB_WIRE(off) ((off) + 2U) /*!< stored-frame offset -> on-wire offset */

/* Header values shared by every sender. */
#define NB_ADDR_VCU   0x16U /*!< this VCU's node address (src of every frame it sends) */
#define NB_ADDR_APP   0x23U /*!< BLE app / dashboard node (dst of the telemetry reports) */
#define NB_CMD_WRITE3 0x03U /*!< cmd 3: register write (every VCU report; ninebot RX "guarded write") */

/**
 * \brief           Ninebot RX parser state (6-byte struct)
 * \note            Field offsets:
 *                        +0 got_header0, +1 in_frame, +2 index, +3 expected,
 *                        +4..+5 running 16-bit additive checksum.
 */
typedef struct {
    uint8_t got_header0; /*!< +0x00 1 once the 0x5A header byte has been seen       */
    uint8_t in_frame;    /*!< +0x01 1 once 0x5A 0xA5 complete; body is accumulating */
    uint8_t index;       /*!< +0x02 write index into the frame-assembly buffer      */
    uint8_t expected;    /*!< +0x03 expected final index (= len + 7)                */
    uint16_t checksum;   /*!< +0x04 running 16-bit additive checksum                */
} vcu_ninebot_parser_t;

/**
 * \brief           Initialise USART2: 115200 8N1, PA2/PA3, DMA RX ring + DMA TX
 * \note            Called once from vcu_periph_init.
 */
void vcu_usart_init(void);

/**
 * \brief           Drain the RX DMA ring, parse frames, dispatch valid ones
 * \note            Super-loop poll.
 */
void vcu_usart_poll(void);

/**
 * \brief           Enqueue a 0x5A/0xA5 frame on the USART2 TX ring
 * \note            The one ring enqueue of the firmware: every unit's report / reply path
 *                      (including busproto's report manager) calls it; the frame is transmitted
 *                      later by \ref vcu_usart_tx_sequencer. Rejects a body longer than 0xF2 bytes.
 * \param[in,out]   ctx: TX ring context (&g_uartctx)
 * \param[in]       src: header src (NB_ADDR_VCU for every VCU frame)
 * \param[in]       dst: header dst
 * \param[in]       len: body length
 * \param[in]       cmd: header cmd
 * \param[in]       arg: header arg
 * \param[in]       payload: \p len body bytes
 */
void vcu_usart_report_send(volatile uint8_t* ctx, uint8_t src, uint8_t dst, uint8_t len, uint8_t cmd, uint8_t arg,
                           const volatile void* payload);

/**
 * \brief           USART reply-frame sequencer (super-loop poll #3)
 * \note            Drains the USART2 TX frame ring (g_uartctx: head+0, tail+1, counter+2, busy+3,
 *                      lock+4; 8 slots of stride 0x100, len@+8, payload@+0xC). Sends one queued
 *                      frame per pass via usart_tx_dma_arm when not busy/locked and head!=tail,
 *                      advancing head mod 8 on success. This is the multi-frame BLE/app TX drain.
 */
void vcu_usart_tx_sequencer(void);

#endif /* USART_H */
