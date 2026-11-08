/**
 * \file            tst_bsp_fakes.h
 * \brief           Recording fakes at the Artery BSP boundary (GNU ld `--wrap`)
 * \note            `[env:emu-test]` links with `-Wl,--wrap=<fn>` for the functions below, so
 *                      every product call of `<fn>` lands in `__wrap_<fn>` (an fff fake) and
 *                      the real function stays reachable as `__real_<fn>`. After
 *                      \ref tst_fakes_reset each fake records the call and forwards it to the
 *                      real BSP function, so the emulated peripheral still does its work.
 *                      Captured outputs:
 *                        - CAN TX: every `can_message_transmit` frame (\ref tst_can_tx_get);
 *                        - USART2 TX: every DMA1 channel 7 burst armed by
 *                          `dma_channel_enable(DMA1_CHANNEL7, TRUE)`, i.e. each Ninebot frame
 *                          the reply sequencer sends (\ref tst_usart_tx_get);
 *                        - flash: `flash_sector_erase` / `flash_halfword_program` /
 *                          `flash_word_program` as an operation log (\ref tst_flash_op_get:
 *                          erase address, program address + length + bytes, consecutive
 *                          programs coalesced) and in the fff argument history.
 *                      Injected inputs: CAN RX frames (\ref tst_can_rx_inject).
 *                      The `__wrap_` / `__real_` names are fixed by GNU ld.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef TST_BSP_FAKES_H
#define TST_BSP_FAKES_H

#include <stdint.h>
#include "at32f415.h"
#include "fff.h"

#ifndef TST_CAN_CAPTURE_MAX
#define TST_CAN_CAPTURE_MAX 16u /*!< CAN TX frames kept (later ones are only counted) */
#endif
#ifndef TST_CAN_RX_QUEUE_MAX
#define TST_CAN_RX_QUEUE_MAX 8u /*!< CAN RX frames that can wait for delivery */
#endif
#ifndef TST_USART_CAPTURE_MAX
#define TST_USART_CAPTURE_MAX 8u /*!< USART TX frames kept (later ones are only counted) */
#endif
#ifndef TST_USART_FRAME_MAX
#define TST_USART_FRAME_MAX 96u /*!< bytes kept per USART TX frame (longer ones are truncated) */
#endif

#ifndef TST_FLASH_OPS_MAX
#define TST_FLASH_OPS_MAX 16u /*!< flash operations kept (later ones are only counted) */
#endif
#ifndef TST_FLASH_DATA_MAX
#define TST_FLASH_DATA_MAX 1024u /*!< programmed bytes kept over all program operations */
#endif

/** \brief Kind of a recorded flash operation. */
typedef enum {
    TST_FLASH_ERASE = 1,  /*!< `flash_sector_erase`: `len` = one 1 KB sector */
    TST_FLASH_PROGRAM = 2 /*!< halfword/word programs at consecutive addresses, coalesced */
} tst_flash_kind_t;

/** \brief One recorded flash operation (forwarded or not). */
typedef struct {
    tst_flash_kind_t kind;    /*!< erase or program run */
    uint32_t addr;            /*!< first address */
    uint32_t len;             /*!< bytes erased / programmed */
    flash_status_type status; /*!< status the call(s) returned */
    const uint8_t* data;      /*!< programmed bytes (`len` of them), `NULL` for an erase or when the pool was full */
} tst_flash_op_t;

/** \brief One captured CAN frame. */
typedef struct {
    uint32_t id;     /*!< 11-bit standard or 29-bit extended identifier */
    uint8_t ext;     /*!< 1 = extended identifier */
    uint8_t rtr;     /*!< 1 = remote frame */
    uint8_t dlc;     /*!< data length */
    uint8_t data[8]; /*!< payload */
} tst_can_frame_t;

/** \brief One captured USART2 TX burst (one Ninebot frame, 5A A5 .. checksum). */
typedef struct {
    uint16_t len;                       /*!< bytes the DMA was armed for */
    uint8_t bytes[TST_USART_FRAME_MAX]; /*!< the first `TST_USART_FRAME_MAX` of them */
} tst_usart_frame_t;

DECLARE_FAKE_VALUE_FUNC(uint8_t, __wrap_can_message_transmit, can_type*, can_tx_message_type*);
DECLARE_FAKE_VALUE_FUNC(uint8_t, __wrap_can_receive_message_pending_get, can_type*, can_rx_fifo_num_type);
DECLARE_FAKE_VOID_FUNC(__wrap_can_message_receive, can_type*, can_rx_fifo_num_type, can_rx_message_type*);
DECLARE_FAKE_VOID_FUNC(__wrap_dma_channel_enable, dma_channel_type*, confirm_state);
DECLARE_FAKE_VALUE_FUNC(flash_status_type, __wrap_flash_sector_erase, uint32_t);
DECLARE_FAKE_VALUE_FUNC(flash_status_type, __wrap_flash_halfword_program, uint32_t, uint16_t);
DECLARE_FAKE_VALUE_FUNC(flash_status_type, __wrap_flash_word_program, uint32_t, uint32_t);

void tst_fakes_reset(void);

uint32_t tst_can_tx_count(void);
const tst_can_frame_t* tst_can_tx_get(uint32_t index);
void tst_can_tx_forward(uint8_t enable);
uint8_t tst_can_rx_inject(uint32_t id, uint8_t ext, const uint8_t* data, uint8_t dlc);
uint32_t tst_can_rx_deliver(void);

uint32_t tst_usart_tx_count(void);
const tst_usart_frame_t* tst_usart_tx_get(uint32_t index);
uint32_t tst_usart_drain(void);

void tst_flash_forward(uint8_t enable);
uint32_t tst_flash_op_count(void);
const tst_flash_op_t* tst_flash_op_get(uint32_t index);

#endif /* TST_BSP_FAKES_H */
