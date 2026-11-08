/**
 * \file            tst_bsp_fakes.c
 * \brief           Recording fakes at the Artery BSP boundary (GNU ld `--wrap`), see the header
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "tst_bsp_fakes.h"
#include <string.h>
#include "can.h"         /* vcu_can_rx_dispatch */
#include "tst_support.h" /* tst_wait_set */
#include "usart.h"       /* vcu_usart_tx_sequencer */
#include "vcu_state.h"   /* g_uartctx */

DEFINE_FFF_GLOBALS;

DEFINE_FAKE_VALUE_FUNC(uint8_t, __wrap_can_message_transmit, can_type*, can_tx_message_type*);
DEFINE_FAKE_VALUE_FUNC(uint8_t, __wrap_can_receive_message_pending_get, can_type*, can_rx_fifo_num_type);
DEFINE_FAKE_VOID_FUNC(__wrap_can_message_receive, can_type*, can_rx_fifo_num_type, can_rx_message_type*);
DEFINE_FAKE_VOID_FUNC(__wrap_dma_channel_enable, dma_channel_type*, confirm_state);
DEFINE_FAKE_VALUE_FUNC(flash_status_type, __wrap_flash_sector_erase, uint32_t);
DEFINE_FAKE_VALUE_FUNC(flash_status_type, __wrap_flash_halfword_program, uint32_t, uint16_t);
DEFINE_FAKE_VALUE_FUNC(flash_status_type, __wrap_flash_word_program, uint32_t, uint32_t);

/* The real BSP functions (GNU ld --wrap). */
uint8_t __real_can_message_transmit(can_type* can_x, can_tx_message_type* tx_message_struct);
uint8_t __real_can_receive_message_pending_get(can_type* can_x, can_rx_fifo_num_type fifo_number);
void __real_can_message_receive(can_type* can_x, can_rx_fifo_num_type fifo_number,
                                can_rx_message_type* rx_message_struct);
void __real_dma_channel_enable(dma_channel_type* dmax_channely, confirm_state new_state);
flash_status_type __real_flash_sector_erase(uint32_t sector_address);
flash_status_type __real_flash_halfword_program(uint32_t address, uint16_t data);
flash_status_type __real_flash_word_program(uint32_t address, uint32_t data);

/* The product's CAN RX FIFO0 interrupt handler (can.c), run by tst_can_rx_deliver. */
void CAN1_RX0_IRQHandler(void);

/** \brief Mailbox number a non-forwarding CAN TX fake reports. */
#define TST_CAN_FAKE_MAILBOX  0u
/** \brief Reply-ring passes \ref tst_usart_drain makes at most (8 slots, with margin). */
#define TST_USART_DRAIN_MAX   32u
/** \brief Bytes one `flash_sector_erase` clears (AT32F415CBT7: 1 KB sectors). */
#define TST_FLASH_SECTOR_SIZE 1024u

static tst_can_frame_t g_can_tx[TST_CAN_CAPTURE_MAX];
static uint32_t g_can_tx_count;
static uint8_t g_can_tx_forward;
static can_rx_message_type g_can_rx[TST_CAN_RX_QUEUE_MAX];
static uint32_t g_can_rx_head;
static uint32_t g_can_rx_count;
static tst_usart_frame_t g_usart_tx[TST_USART_CAPTURE_MAX];
static uint32_t g_usart_tx_count;
static uint8_t g_flash_forward;
static tst_flash_op_t g_flash_ops[TST_FLASH_OPS_MAX];
static uint32_t g_flash_op_count;
static uint8_t g_flash_data[TST_FLASH_DATA_MAX];
static uint32_t g_flash_data_used;

/**
 * \brief           Append an erase to the flash operation log
 * \param[in]       addr: sector address
 * \param[in]       status: status the call returned
 */
static void prv_flash_log_erase(uint32_t addr, flash_status_type status) {
    if (g_flash_op_count < TST_FLASH_OPS_MAX) {
        tst_flash_op_t* op = &g_flash_ops[g_flash_op_count];

        op->kind = TST_FLASH_ERASE;
        op->addr = addr;
        op->len = TST_FLASH_SECTOR_SIZE;
        op->status = status;
        op->data = NULL;
    }
    ++g_flash_op_count;
}

/**
 * \brief           Append programmed bytes to the log, extending the last run when contiguous
 * \param[in]       addr: first programmed address
 * \param[in]       bytes: little-endian value bytes
 * \param[in]       len: 2 (halfword) or 4 (word)
 * \param[in]       status: status the call returned
 */
static void prv_flash_log_program(uint32_t addr, const uint8_t* bytes, uint32_t len, flash_status_type status) {
    tst_flash_op_t* last = NULL;
    uint8_t room = (uint8_t)((g_flash_data_used + len) <= TST_FLASH_DATA_MAX);

    if ((g_flash_op_count > 0u) && (g_flash_op_count <= TST_FLASH_OPS_MAX)) {
        last = &g_flash_ops[g_flash_op_count - 1u];
    }
    if ((last != NULL) && (last->kind == TST_FLASH_PROGRAM) && (last->status == status)
        && ((last->addr + last->len) == addr)
        && ((last->data == NULL) || (last->data + last->len == &g_flash_data[g_flash_data_used]))) {
        if ((last->data != NULL) && (room != 0u)) {
            memcpy(&g_flash_data[g_flash_data_used], bytes, len);
            g_flash_data_used += len;
        } else {
            last->data = NULL;
        }
        last->len += len;
    } else {
        if (g_flash_op_count < TST_FLASH_OPS_MAX) {
            tst_flash_op_t* op = &g_flash_ops[g_flash_op_count];

            op->kind = TST_FLASH_PROGRAM;
            op->addr = addr;
            op->len = len;
            op->status = status;
            op->data = NULL;
            if (room != 0u) {
                memcpy(&g_flash_data[g_flash_data_used], bytes, len);
                op->data = &g_flash_data[g_flash_data_used];
                g_flash_data_used += len;
            }
        }
        ++g_flash_op_count;
    }
}

/**
 * \brief           CAN TX fake: capture the frame, then (by default) hand it to the mailbox
 * \param[in]       can_x: CAN controller
 * \param[in]       tx_message_struct: frame to send
 * \return          mailbox number from the real BSP, or \ref TST_CAN_FAKE_MAILBOX
 */
static uint8_t prv_can_transmit(can_type* can_x, can_tx_message_type* tx_message_struct) {
    uint8_t mailbox = TST_CAN_FAKE_MAILBOX;

    if (tx_message_struct == NULL) {
        return CAN_TX_STATUS_NO_EMPTY;
    }
    if (g_can_tx_count < TST_CAN_CAPTURE_MAX) {
        tst_can_frame_t* rec = &g_can_tx[g_can_tx_count];

        rec->ext = (tx_message_struct->id_type == CAN_ID_EXTENDED) ? 1u : 0u;
        rec->id = (rec->ext != 0u) ? tx_message_struct->extended_id : tx_message_struct->standard_id;
        rec->rtr = (tx_message_struct->frame_type == CAN_TFT_REMOTE) ? 1u : 0u;
        rec->dlc = tx_message_struct->dlc;
        memcpy(rec->data, tx_message_struct->data, sizeof(rec->data));
    }
    ++g_can_tx_count;
    if (g_can_tx_forward != 0u) {
        mailbox = __real_can_message_transmit(can_x, tx_message_struct);
    }
    return mailbox;
}

/**
 * \brief           CAN RX pending fake: report the injected frames, else ask the controller
 * \param[in]       can_x: CAN controller
 * \param[in]       fifo_number: receive FIFO
 * \return          number of frames pending
 */
static uint8_t prv_can_rx_pending(can_type* can_x, can_rx_fifo_num_type fifo_number) {
    uint8_t pending;

    if ((can_x == CAN1) && (fifo_number == CAN_RX_FIFO0) && (g_can_rx_count > 0u)) {
        pending = (uint8_t)g_can_rx_count;
    } else {
        pending = __real_can_receive_message_pending_get(can_x, fifo_number);
    }
    return pending;
}

/**
 * \brief           CAN RX read-out fake: pop the oldest injected frame, else read the FIFO
 * \param[in]       can_x: CAN controller
 * \param[in]       fifo_number: receive FIFO
 * \param[out]      rx_message_struct: the received frame
 */
static void prv_can_rx_read(can_type* can_x, can_rx_fifo_num_type fifo_number, can_rx_message_type* rx_message_struct) {
    if ((can_x == CAN1) && (fifo_number == CAN_RX_FIFO0) && (g_can_rx_count > 0u) && (rx_message_struct != NULL)) {
        *rx_message_struct = g_can_rx[g_can_rx_head];
        g_can_rx_head = (g_can_rx_head + 1u) % TST_CAN_RX_QUEUE_MAX;
        --g_can_rx_count;
    } else {
        __real_can_message_receive(can_x, fifo_number, rx_message_struct);
    }
}

/**
 * \brief           DMA channel enable fake: capture each USART2 TX burst, then enable for real
 * \note            usart.c arms DMA1 channel 7 (USART2_TX) with `maddr` = frame and `dtcnt` =
 *                      length, then enables it: the bytes at that moment are the frame on the wire.
 * \param[in]       dmax_channely: DMA channel
 * \param[in]       new_state: `TRUE` to enable
 */
static void prv_dma_channel_enable(dma_channel_type* dmax_channely, confirm_state new_state) {
    if ((dmax_channely == DMA1_CHANNEL7) && (new_state == TRUE)) {
        if (g_usart_tx_count < TST_USART_CAPTURE_MAX) {
            tst_usart_frame_t* rec = &g_usart_tx[g_usart_tx_count];
            const uint8_t* src = (const uint8_t*)(uintptr_t)dmax_channely->maddr;
            uint16_t len = (uint16_t)dmax_channely->dtcnt;

            rec->len = len;
            memcpy(rec->bytes, src, (len < TST_USART_FRAME_MAX) ? len : TST_USART_FRAME_MAX);
        }
        ++g_usart_tx_count;
    }
    __real_dma_channel_enable(dmax_channely, new_state);
}

/**
 * \brief           Flash erase fake: forward unless \ref tst_flash_forward turned it off
 * \param[in]       sector_address: sector to erase
 * \return          real status, or `FLASH_OPERATE_DONE`
 */
static flash_status_type prv_flash_sector_erase(uint32_t sector_address) {
    flash_status_type status = FLASH_OPERATE_DONE;

    if (g_flash_forward != 0u) {
        status = __real_flash_sector_erase(sector_address);
    }
    prv_flash_log_erase(sector_address, status);
    return status;
}

/**
 * \brief           Flash halfword program fake: forward unless turned off
 * \param[in]       address: target address
 * \param[in]       data: halfword to program
 * \return          real status, or `FLASH_OPERATE_DONE`
 */
static flash_status_type prv_flash_halfword_program(uint32_t address, uint16_t data) {
    flash_status_type status = FLASH_OPERATE_DONE;

    const uint8_t bytes[2] = {(uint8_t)(data & 0xFFu), (uint8_t)(data >> 8)};

    if (g_flash_forward != 0u) {
        status = __real_flash_halfword_program(address, data);
    }
    prv_flash_log_program(address, bytes, sizeof(bytes), status);
    return status;
}

/**
 * \brief           Flash word program fake: forward unless turned off
 * \param[in]       address: target address
 * \param[in]       data: word to program
 * \return          real status, or `FLASH_OPERATE_DONE`
 */
static flash_status_type prv_flash_word_program(uint32_t address, uint32_t data) {
    flash_status_type status = FLASH_OPERATE_DONE;

    const uint8_t bytes[4] = {(uint8_t)(data & 0xFFu), (uint8_t)((data >> 8) & 0xFFu), (uint8_t)((data >> 16) & 0xFFu),
                              (uint8_t)(data >> 24)};

    if (g_flash_forward != 0u) {
        status = __real_flash_word_program(address, data);
    }
    prv_flash_log_program(address, bytes, sizeof(bytes), status);
    return status;
}

/**
 * \brief           Reset every fake and capture; all fakes record and forward to the real BSP
 * \note            Call it in `setUp`. It does not touch product state (see
 *                      tst_reset_product_state) or peripheral registers.
 */
void tst_fakes_reset(void) {
    RESET_FAKE(__wrap_can_message_transmit);
    RESET_FAKE(__wrap_can_receive_message_pending_get);
    RESET_FAKE(__wrap_can_message_receive);
    RESET_FAKE(__wrap_dma_channel_enable);
    RESET_FAKE(__wrap_flash_sector_erase);
    RESET_FAKE(__wrap_flash_halfword_program);
    RESET_FAKE(__wrap_flash_word_program);
    FFF_RESET_HISTORY();
    __wrap_can_message_transmit_fake.custom_fake = prv_can_transmit;
    __wrap_can_receive_message_pending_get_fake.custom_fake = prv_can_rx_pending;
    __wrap_can_message_receive_fake.custom_fake = prv_can_rx_read;
    __wrap_dma_channel_enable_fake.custom_fake = prv_dma_channel_enable;
    __wrap_flash_sector_erase_fake.custom_fake = prv_flash_sector_erase;
    __wrap_flash_halfword_program_fake.custom_fake = prv_flash_halfword_program;
    __wrap_flash_word_program_fake.custom_fake = prv_flash_word_program;
    memset(g_can_tx, 0, sizeof(g_can_tx));
    memset(g_can_rx, 0, sizeof(g_can_rx));
    memset(g_usart_tx, 0, sizeof(g_usart_tx));
    g_can_tx_count = 0u;
    g_can_rx_head = 0u;
    g_can_rx_count = 0u;
    g_usart_tx_count = 0u;
    g_can_tx_forward = 1u;
    g_flash_forward = 1u;
    memset(g_flash_ops, 0, sizeof(g_flash_ops));
    g_flash_op_count = 0u;
    g_flash_data_used = 0u;
}

/**
 * \brief           Number of CAN frames handed to `can_message_transmit` since the reset
 * \return          frame count (may exceed \ref TST_CAN_CAPTURE_MAX)
 */
uint32_t tst_can_tx_count(void) { return g_can_tx_count; }

/**
 * \brief           Captured CAN TX frame `index` (0 = oldest)
 * \param[in]       index: frame index
 * \return          the frame, or `NULL` if it was not captured
 */
const tst_can_frame_t* tst_can_tx_get(uint32_t index) {
    const tst_can_frame_t* frame = NULL;

    if ((index < g_can_tx_count) && (index < TST_CAN_CAPTURE_MAX)) {
        frame = &g_can_tx[index];
    }
    return frame;
}

/**
 * \brief           Choose whether captured CAN TX frames also go to the real mailboxes
 * \param[in]       enable: `1` forward (default), `0` capture only and report mailbox 0
 */
void tst_can_tx_forward(uint8_t enable) { g_can_tx_forward = (enable != 0u) ? 1u : 0u; }

/**
 * \brief           Queue a CAN frame that the next \ref tst_can_rx_deliver hands to the product
 * \param[in]       id: identifier (11 or 29 bits)
 * \param[in]       ext: `1` for an extended identifier
 * \param[in]       data: payload (`dlc` bytes, may be `NULL` when `dlc` is 0)
 * \param[in]       dlc: payload length, 0..8
 * \return          `1` queued, `0` queue full or bad arguments
 */
uint8_t tst_can_rx_inject(uint32_t id, uint8_t ext, const uint8_t* data, uint8_t dlc) {
    uint8_t queued = 0u;

    if ((dlc > 8u) || ((data == NULL) && (dlc > 0u))) {
        return 0u;
    }
    if (g_can_rx_count < TST_CAN_RX_QUEUE_MAX) {
        can_rx_message_type* msg = &g_can_rx[(g_can_rx_head + g_can_rx_count) % TST_CAN_RX_QUEUE_MAX];

        memset(msg, 0, sizeof(*msg));
        msg->id_type = (ext != 0u) ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
        msg->frame_type = CAN_TFT_DATA;
        msg->standard_id = (ext != 0u) ? 0u : id;
        msg->extended_id = (ext != 0u) ? id : 0u;
        msg->dlc = dlc;
        if (dlc > 0u) {
            memcpy(msg->data, data, dlc);
        }
        ++g_can_rx_count;
        queued = 1u;
    }
    return queued;
}

/**
 * \brief           Deliver the injected CAN frames through the product's RX path
 * \note            Runs the product `CAN1_RX0_IRQHandler` once per queued frame (the fakes
 *                      make it read the injected frame instead of the FIFO), then the
 *                      super-loop consumer `vcu_can_rx_dispatch` until the RX ring is empty.
 * \return          number of frames delivered
 */
uint32_t tst_can_rx_deliver(void) {
    uint32_t delivered = 0u;

    while (g_can_rx_count > 0u) {
        CAN1_RX0_IRQHandler();
        ++delivered;
    }
    while (vcu_can_rx_dispatch() != 0u) {
        /* drain the ring */
    }
    return delivered;
}

/**
 * \brief           Number of USART2 TX bursts armed since the reset
 * \return          frame count (may exceed \ref TST_USART_CAPTURE_MAX)
 */
uint32_t tst_usart_tx_count(void) { return g_usart_tx_count; }

/**
 * \brief           Captured USART2 TX frame `index` (0 = oldest)
 * \param[in]       index: frame index
 * \return          the frame, or `NULL` if it was not captured
 */
const tst_usart_frame_t* tst_usart_tx_get(uint32_t index) {
    const tst_usart_frame_t* frame = NULL;

    if ((index < g_usart_tx_count) && (index < TST_USART_CAPTURE_MAX)) {
        frame = &g_usart_tx[index];
    }
    return frame;
}

/**
 * \brief           Send every frame queued in the USART reply ring, as the super-loop would
 * \note            Calls the product reply sequencer (`vcu_usart_tx_sequencer`, one frame per
 *                      pass) and waits for each DMA burst to finish (DMA1 FDT7), until the ring
 *                      is empty. Needs `vcu_usart_init` to have run (USART2 + DMA1 set up).
 * \return          number of frames sent by this call
 */
uint32_t tst_usart_drain(void) {
    uint32_t before = g_usart_tx_count;
    uint32_t passes = 0u;

    while ((g_uartctx.head != g_uartctx.tail) && (passes < TST_USART_DRAIN_MAX)) {
        vcu_usart_tx_sequencer();
        (void)tst_wait_set(&DMA1->sts, DMA1_FDT7_FLAG, TST_WAIT_LOOPS_DEFAULT);
        ++passes;
    }
    return g_usart_tx_count - before;
}

/**
 * \brief           Choose whether the flash erase/program fakes touch the flash
 * \param[in]       enable: `1` forward (default), `0` record only and report `FLASH_OPERATE_DONE`
 */
void tst_flash_forward(uint8_t enable) { g_flash_forward = (enable != 0u) ? 1u : 0u; }

/**
 * \brief           Number of flash operations since the reset (erases + coalesced program runs)
 * \return          operation count (may exceed \ref TST_FLASH_OPS_MAX)
 */
uint32_t tst_flash_op_count(void) { return g_flash_op_count; }

/**
 * \brief           Recorded flash operation `index` (0 = oldest)
 * \note            A program run is a sequence of halfword/word programs at consecutive addresses
 *                      with the same status, e.g. the 256 halfwords of one identity record become
 *                      one operation of 512 bytes.
 * \param[in]       index: operation index
 * \return          the operation, or `NULL` if it was not kept
 */
const tst_flash_op_t* tst_flash_op_get(uint32_t index) {
    const tst_flash_op_t* op = NULL;

    if ((index < g_flash_op_count) && (index < TST_FLASH_OPS_MAX)) {
        op = &g_flash_ops[index];
    }
    return op;
}
