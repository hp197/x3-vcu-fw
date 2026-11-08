/**
 * \file            usart.c
 * \brief           USART2 BLE/dashboard link driver + Ninebot 5A A5 parser (x3 VCU)
 * \note            USART2 subsystem on the Artery AT32F415 BSP.
 *
 *                      USART2 is DMA-driven, NOT interrupt-driven: its vector points at an
 *                      empty handler, so there is deliberately NO `USART2_IRQHandler` in this
 *                      file. RX is a hardware-filled circular DMA ring drained by the super-loop;
 *                      TX is an on-demand DMA burst.
 *
 *                      The DMA/GPIO setup uses the BSP `dma_init_type` / `gpio_init_type` named
 *                      fields; the baud + word/stop programming goes through the BSP `usart_init`.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "usart.h"
#include "board.h"   /* PCLK1_HZ */
#include "gpio.h"    /* vcu_gpio_config */
#include "ninebot.h" /* vcu_ninebot_dispatch */
#include "vcu_state.h"

/* ------------------------------------------------------------------------- */
/* Module state. The linker places these statics.                            */
/* ------------------------------------------------------------------------- */

/** \brief RX DMA ring, hardware-filled (circular buffer, 250 B) */
static uint8_t g_rx_ring[VCU_USART_RX_RING_LEN];

/** \brief Software ring read-index / tail */
static uint16_t g_rx_read_idx;

/** \brief Ninebot RX parser state (6-byte struct) */
static vcu_ninebot_parser_t g_parser;

/** \brief Frame-assembly buffer, bytes stored from the `len` byte on */
static uint8_t g_frame[VCU_USART_FRAME_MAX];

/** \brief TX DMA buffer bound at init (16 bytes). Every burst re-points the channel at its
           ring slot (usart_tx_dma_arm), so it is never sent. */
static uint8_t g_tx_buf[VCU_USART_TX_BUF_LEN];

/* DMA1 channel ctrl words (what dma_reset + dma_init leave, channel disabled): byte-wide both
   ends, memory increment [7], priority high [13:12]; RX peripheral -> memory, circular [5];
   TX memory -> peripheral (dtd [4]), normal mode. */
#define USART_DMA_RX_CTRL                                                                                              \
    ((uint32_t)DMA_DIR_PERIPHERAL_TO_MEMORY | (1u << 5) | (1u << 7) | ((uint32_t)DMA_PRIORITY_HIGH << 12))
#define USART_DMA_TX_CTRL  ((uint32_t)DMA_DIR_MEMORY_TO_PERIPHERAL | (1u << 7) | ((uint32_t)DMA_PRIORITY_HIGH << 12))
#define USART_DMA_RX_FLAGS (0xFu << 20) /*!< DMA1 clr flags of channel 6 */
#define USART_DMA_TX_FLAGS (0xFu << 24) /*!< DMA1 clr flags of channel 7 */
#define USART_CTRL1_TEN    0x0008u      /*!< ctrl1 ten [3]: transmitter enable     */
#define USART_CTRL1_REN    0x0004u      /*!< ctrl1 ren [2]: receiver enable        */
#define USART_CTRL3_DMATEN 0x0080u      /*!< ctrl3 dmaten [7]: DMA transmit enable */
#define USART_CTRL3_DMAREN 0x0040u      /*!< ctrl3 dmaren [6]: DMA receive enable  */

/** \brief TX-in-flight latch (1 byte) */
static uint8_t g_tx_busy;

/* ------------------------------------------------------------------------- */
/* Init chain: GPIO -> controller cfg -> RX DMA -> TX DMA                    */
/* ------------------------------------------------------------------------- */

/**
 * \brief           USART2 pin setup: PA2 = USART2_TX, PA3 = USART2_RX
 * \note            Pin configuration:
 *                        PA2 (mask 0x04) cfg 0x1080400 -> alternate-function push-pull = TX
 *                        PA3 (mask 0x08) cfg 0x1001800 -> floating input             = RX
 *                      These are the BSP named fields below (AT32 default USART2 map, no IOMUX
 *                      remap). The GPIOA clock and the AFIO/IOMUX router clock are enabled first,
 *                      following the project's gpio.c convention.
 */
static void usart_gpio_init(void) {
    /* GPIOA clock, then the AFIO/IOMUX router clock. */
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_GPIOA_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_IOMUX_PERIPH_CLOCK));

    vcu_gpio_config(GPIOA, GPIO_PINS_2, VCU_GPIO_CFG_MUX_PP);   /* PA2 : USART2_TX, mux push-pull */
    vcu_gpio_config(GPIOA, GPIO_PINS_3, VCU_GPIO_CFG_IN_FLOAT); /* PA3 : USART2_RX, floating input */
}

/**
 * \brief           USART2 controller config: 115200 8N1, TE/RE, DMA TX/RX, enable
 * \note            Register writes (those of the BSP usart_init and enable calls):
 *                        baud 115200 (0x1C200), M=0 = 8 data bits, stop field 0 = 1 stop
 *                        TE + RE     (CTRL1 bit3 + bit2)
 *                        DMAT + DMAR (CTRL3 bit7 + bit6)
 *                        UEN         (CTRL1 bit13, 0x2000)
 *                      Parity is never written -> none (reset default); no hardware flow ctrl.
 */
static void usart_cfg(void) {
    /* Enable the USART2 peripheral clock. */
    BOARD_CLOCK_ON(CRM_USART2_PERIPH_CLOCK);

    /* 115200 8N1. The divider is PCLK1 / baud, rounded to nearest (as the BSP's usart_init,
       without reading the clock tree back). */
    USART2->baudr_bit.div = VCU_USART_BAUD_DIV;
    USART2->ctrl1_bit.dbn = USART_DATA_8BITS;
    USART2->ctrl2_bit.stopbn = USART_STOP_1_BIT;
    USART2->ctrl1 |= USART_CTRL1_TEN | USART_CTRL1_REN;       /* TE + RE     */
    USART2->ctrl3 |= USART_CTRL3_DMATEN | USART_CTRL3_DMAREN; /* DMAT + DMAR */
    USART2->ctrl1_bit.uen = TRUE;                             /* UEN         */
}

/**
 * \brief           RX DMA setup: DMA1 Channel6, circular 250-byte ring, USART2_DT -> SRAM
 * \note            DMA parameters:
 *                        peripheral_base_addr = 0x40004404 (USART2 DT)
 *                        memory_base_addr     = g_rx_ring
 *                        direction            = 0  P->M
 *                        buffer_size          = 0xFA = 250
 *                        peripheral_inc       = 0 (disable)
 *                        memory_inc           = 1 (enable)
 *                        peripheral_width     = 0 (byte)
 *                        memory_width         = 0 (byte)
 *                        loop_mode            = 1 (circular)
 *                        priority             = 2 (high)
 *                      Channel is ENABLED at init (RX runs continuously). The registers get the
 *                      values dma_reset + dma_init leave for these settings.
 */
static void usart_rx_dma_init(void) {
    /* Enable the DMA1 peripheral clock. */
    BOARD_CLOCK_ON(CRM_DMA1_PERIPH_CLOCK);

    DMA1_CHANNEL6->ctrl = USART_DMA_RX_CTRL;
    DMA1->clr = USART_DMA_RX_FLAGS;
    DMA1_CHANNEL6->dtcnt = VCU_USART_RX_RING_LEN;
    DMA1_CHANNEL6->paddr = (uint32_t)&USART2->dt;
    DMA1_CHANNEL6->maddr = (uint32_t)g_rx_ring;

    dma_channel_enable(DMA1_CHANNEL6, TRUE);
}

/**
 * \brief           TX DMA setup: DMA1 Channel7, SRAM -> USART2_DT, armed on demand
 * \note            DMA parameters:
 *                        peripheral_base_addr = 0x40004404 (USART2 DT)
 *                        memory_base_addr     = g_tx_buf
 *                        direction            = 0x10 M->P
 *                        buffer_size          = 0 (set per transmit)
 *                        memory_inc           = 1 (enable)
 *                        peripheral/memory width = 0 (byte)
 *                        loop_mode            = 0 (normal)
 *                        priority             = 2 (high)
 *                      Channel is DISABLED at init (armed per frame by \ref usart_tx_dma_arm).
 *                      The registers get the values dma_reset + dma_init leave for these settings.
 *                      The USART2 clock is already on from usart_cfg.
 */
static void usart_tx_dma_init(void) {
    DMA1_CHANNEL7->ctrl = USART_DMA_TX_CTRL;
    DMA1->clr = USART_DMA_TX_FLAGS;
    DMA1_CHANNEL7->dtcnt = 0u;
    DMA1_CHANNEL7->paddr = (uint32_t)&USART2->dt;
    DMA1_CHANNEL7->maddr = (uint32_t)g_tx_buf;

    dma_channel_enable(DMA1_CHANNEL7, FALSE);
}

/**
 * \brief           Initialise the USART2 link (called once from vcu_periph_init)
 * \note            Order: GPIO -> controller cfg -> RX DMA -> TX DMA.
 */
void vcu_usart_init(void) {
    g_rx_read_idx = 0;
    /* The ring starts with free = 7 (head/tail/latches 0). A C initialiser would move the
       whole 0x808-byte ring into .data (+2 KB flash), so the one nonzero header byte is set
       here, before any producer can enqueue. */
    g_uartctx.free = VCU_UART_SLOTS - 1u;
    g_uartctx.rx_busy = 0;
    g_tx_busy = 0;
    g_parser.got_header0 = 0;
    g_parser.in_frame = 0;
    g_parser.index = 0;
    g_parser.expected = 0;
    g_parser.checksum = 0;

    usart_gpio_init();
    usart_cfg();
    usart_rx_dma_init();
    usart_tx_dma_init();
}

/* ------------------------------------------------------------------------- */
/* RX parser                                                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Reset the parser back to "hunting for 0x5A"
 * \note            Clears got_header0, in_frame and the running checksum. The stored index
 *                      is left as-is: on the header-mismatch / length-reject paths that use
 *                      this reset it is already 0.
 */
static void usart_parser_reset(void) {
    g_parser.in_frame = 0;
    g_parser.got_header0 = 0;
    g_parser.checksum = 0;
}

/**
 * \brief           Feed one received byte to the Ninebot 5A A5 state machine
 * \note            Control flow:
 *                        - not in-frame: hunt 0x5A then 0xA5 (header); any mismatch resets.
 *                          On 0xA5 the running checksum is cleared.
 *                        - in-frame: store the byte at `g_frame[index]`.
 *                          - index 0 is the `len` field: set expected = len + 7 and
 *                            range-check (reject if `((len+7)&0xFF) - 7 > 0xEC`).
 *                          - advance index; when it reaches `expected`, the last two stored
 *                            bytes are a little-endian cksum16. Validate against
 *                            `~(sum - cksum_lo)` (one's-complement additive checksum) and, on
 *                            a match, hand the frame to the shared dispatcher. Then reset.
 *                          - otherwise accumulate the byte into the running sum.
 * \param[in]       byte: the received byte value
 */
static void usart_parser_feed(uint8_t byte) {
    if (g_parser.in_frame == 0u) {
        if ((byte == NB_HDR0) && (g_parser.got_header0 == 0u)) {
            g_parser.got_header0 = 1u;
        } else if ((byte == NB_HDR1) && (g_parser.got_header0 != 0u)) {
            g_parser.in_frame = 1u;
            g_parser.checksum = 0u;
        } else {
            usart_parser_reset();
        }
    } else {
        uint8_t idx = g_parser.index;

        g_frame[idx] = byte;

        if (idx == 0u) {
            g_parser.expected = (uint8_t)(byte + VCU_NINEBOT_OVERHEAD);
            if ((uint8_t)((uint8_t)(byte + VCU_NINEBOT_OVERHEAD) - VCU_NINEBOT_OVERHEAD) > VCU_NINEBOT_LEN_MAX) {
                usart_parser_reset();
                return;
            }
        }

        {
            uint8_t next = (uint8_t)(idx + 1u);

            g_parser.index = next;
            if (next == g_parser.expected) {
                uint8_t cksum_lo = g_frame[next - 2u];
                uint8_t cksum_hi = g_frame[next - 1u];
                uint16_t computed = (uint16_t)~(uint16_t)(g_parser.checksum - cksum_lo);

                g_parser.checksum = computed;
                if (computed == (uint16_t)(cksum_lo + (uint16_t)(cksum_hi * 0x100U))) {
                    vcu_ninebot_dispatch(g_frame);
                }

                /* the reset on the completed-frame path also clears the index. */
                g_parser.in_frame = 0;
                g_parser.got_header0 = 0;
                g_parser.index = 0;
                g_parser.checksum = 0;
            } else {
                g_parser.checksum = (uint16_t)(g_parser.checksum + byte);
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* RX ring drain                                                             */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Copy newly-DMA'd bytes out of the circular ring into a linear buffer
 * \note            `head = 250 - DMA_remaining(ch6)`; if the ring is empty
 *                      (`head == read_idx`) or the count hit zero (`head == 250`) nothing is
 *                      copied. Otherwise bytes from `read_idx .. head` are copied one at a time,
 *                      advancing and wrapping the software read-index at 250.
 * \param[out]      out: linear destination (caller supplies >= 250 bytes)
 * \return          number of bytes copied
 */
static uint16_t usart_ring_drain(uint8_t* out) {
    uint16_t count = 0;
    uint16_t remaining = (uint16_t)DMA1_CHANNEL6->dtcnt; /* CNDTR */
    uint16_t head = (uint16_t)(VCU_USART_RX_RING_LEN - remaining);

    if (head == g_rx_read_idx) {
        return 0;
    }
    if (head == VCU_USART_RX_RING_LEN) {
        return 0;
    }

    while (head != g_rx_read_idx) {
        uint16_t tail = g_rx_read_idx;

        g_rx_read_idx = (uint16_t)(tail + 1u);
        out[count] = g_rx_ring[tail];
        count = (uint16_t)(count + 1u);
        if (g_rx_read_idx > (VCU_USART_RX_RING_LEN - 1u)) {
            g_rx_read_idx = 0;
        }
    }

    return count;
}

/* ------------------------------------------------------------------------- */
/* RX pump                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Drain the RX ring and parse every byte, until the ring is empty
 * \note            A re-entrancy-guarded pump that repeatedly drains the circular DMA ring
 *                      into a local buffer and feeds each byte to the parser, looping until a
 *                      drain yields nothing. Runs only when g_uartctx's +3 and +5 bytes are both
 *                      clear: +3 is the TX ring's enqueue latch (`busy`), +5 the pump's own latch
 *                      (`rx_busy`). The RX DMA ring is channel 6.
 *                      Called every super-loop pass from `main`.
 */
void vcu_usart_poll(void) {
    uint8_t scratch[VCU_USART_RX_RING_LEN + 2U]; /* 252 bytes */
    uint16_t n;
    uint16_t i;

    if ((g_uartctx.busy != 0u) || (g_uartctx.rx_busy != 0u)) {
        return;
    }

    g_uartctx.rx_busy = 1u;
    n = usart_ring_drain(scratch);
    while (n != 0u) {
        for (i = 0; i < n; i = (uint16_t)(i + 1u)) {
            usart_parser_feed(scratch[i]);
        }
        n = usart_ring_drain(scratch);
    }
    g_uartctx.rx_busy = 0u;
}

/* ------------------------------------------------------------------------- */
/* TX wire format + physical transmit                                         */
/*                                                                            */
/*   usart_frame_cksum16      (one's-complement additive sum)                  */
/*   usart_frame_assemble     (5A A5 field-wise frame builder)                 */
/*   usart_tx_enqueue         (enqueue reply into the TX ring)                 */
/*   usart_tx_dma_arm         (arm DMA1 Ch7 for one frame)                     */
/* The reply-frame sequencer (super-loop poll #3) drains the ring and calls     */
/* usart_tx_dma_arm; it sits at the bottom of this file                         */
/* (vcu_usart_tx_sequencer), next to the shared report framer                  */
/* vcu_usart_report_send, so the wire format is owned here.                   */
/*                                                                            */
/* TX frame ring context (g_uartctx, 8 slots):                                 */
/*   +0x00 head  +0x01 tail  +0x02 free-slot count  +0x03 busy  +0x04 lock      */
/*   slot stride 0x100; per slot: len (u32) @ +0x08, frame[0xFC] @ +0x0C.       */
/*   Object is 0x808 bytes (\ref vcu_uartctx_t); free starts at 7.             */
/*                                                                            */
/* Aliasing: this ctx is the shared store's g_uartctx, never a static here.     */
/* usart_tx_enqueue() takes it as `ctx`, and every producer passes that same     */
/* base: the report writers (vcu_usart_report_send(g_uartctx, ...)) and ota.c    */
/* (OTA_USART_CTX). So the reply ring aliases the shared store; no re-pointing.  */
/*                                                                            */
/* Module-private state (not in the shared store):                             */
/*   g_rx_ring      hardware-filled RX DMA ring (circular buffer)               */
/*   g_tx_buf       on-demand TX DMA burst buffer                               */
/*   g_frame        linear frame-assembly scratch                               */
/*   g_rx_read_idx / g_tx_busy  USART-private cursors                           */
/*   g_parser       single-producer/consumer within USART -- kept as a module   */
/*                  static.                                                     */
/* ------------------------------------------------------------------------- */

/* Ring header and slots are the named members of \ref vcu_uartctx_t:
   &slot[i].len == ctx + 0x08 + i*0x100, slot[i].frame == ctx + 0x0C + i*0x100. */

/** \brief Max accepted reply body length in \ref usart_tx_enqueue (`< 0xF3`) */
#define VCU_NINEBOT_REPLY_MAX 243U

/**
 * \brief           Ninebot additive one's-complement checksum
 * \note            16-bit additive sum over `len` bytes, bitwise-inverted. This is the single
 *                      Ninebot checksum primitive of the firmware (busproto uses it too) and the
 *                      inverse of the RX parser's validation in \ref vcu_usart_poll.
 * \param[in]       buf: first byte to sum (the stored-frame bytes, i.e. from the `len` field)
 * \param[in]       len: number of bytes to sum
 * \return          `~sum` (little-endian cksum16 as emitted on the wire)
 */
static uint16_t usart_frame_cksum16(const uint8_t* buf, uint16_t len) {
    uint16_t sum = 0;
    uint16_t i;

    for (i = 0; i < len; i = (uint16_t)(i + 1u)) {
        sum = (uint16_t)(sum + buf[i]);
    }
    return (uint16_t)~sum;
}

/**
 * \brief           Assemble a Ninebot 5A A5 frame from its fields
 * \note            On the wire:
 *                        5A A5 | len | src | dst | cmd | arg | body[len] | ck_lo ck_hi
 *                      The checksum covers everything from the `len` byte up to (not including)
 *                      the two checksum bytes. The byte cursors are `uint8_t` and wrap at 0xFF
 *                      (body_len < 0xF3, so no wrap occurs in practice).
 * \param[in]       src: header src (out[3])
 * \param[in]       dst: header dst (out[4])
 * \param[in]       body_len: Ninebot length field (number of body bytes)
 * \param[in]       cmd: header cmd (out[5])
 * \param[in]       arg: header arg (out[6])
 * \param[in]       body: body bytes (may be unused when body_len == 0)
 * \param[out]      out: destination, at least `body_len + 9` bytes
 * \return          total on-wire length written (`body_len + 9`, truncated to 8 bits)
 */
static uint16_t usart_frame_assemble(uint8_t src, uint8_t dst, uint8_t body_len, uint8_t cmd, uint8_t arg,
                                         const uint8_t* body, uint8_t* out) {
    uint8_t pos;
    uint8_t i;
    uint16_t cksum;

    out[0] = NB_HDR0;
    out[1] = NB_HDR1;
    out[NB_WIRE(NB_OFF_LEN)] = body_len;
    out[NB_WIRE(NB_OFF_SRC)] = src;
    out[NB_WIRE(NB_OFF_DST)] = dst;
    out[NB_WIRE(NB_OFF_CMD)] = cmd;
    out[NB_WIRE(NB_OFF_ARG)] = arg;

    pos = NB_WIRE(NB_OFF_BODY);
    for (i = 0; i < body_len; i = (uint8_t)(i + 1u)) {
        out[pos] = body[i];
        pos = (uint8_t)(pos + 1u);
    }

    /* checksum covers everything from the `len` byte up to (not incl.) the cksum bytes. */
    cksum = usart_frame_cksum16(out + NB_WIRE(NB_OFF_LEN), (uint16_t)(pos - NB_WIRE(NB_OFF_LEN)));
    out[pos] = (uint8_t)(cksum & 0xFFU);
    out[(uint8_t)(pos + 1u)] = (uint8_t)(cksum >> 8);
    return (uint16_t)(uint8_t)(pos + 2u);
}

/**
 * \brief           Arm DMA1 Channel7 to transmit one assembled frame
 * \note            Bails out while a previous burst is still in flight (TX-complete flag FDT7
 *                      clear AND the busy latch set). Otherwise it latches busy, clears FDT7,
 *                      points the Channel7 memory address straight at the caller's frame,
 *                      programs the byte count and re-enables the channel.
 *                      Hardware: DMA1 Ch7 base @ 0x40020080 (USART2_TX), flag bit
 *                      DMA1_FDT7_FLAG = 0x02000000.
 * \param[in]       payload: pointer to the on-wire frame bytes (DMA source)
 * \param[in]       len: number of bytes to transmit
 * \return          1 when the burst was armed, 0 when the previous burst is still busy
 */
static int32_t usart_tx_dma_arm(volatile uint8_t* payload, uint16_t len) {
    if (((DMA1->sts & DMA1_FDT7_FLAG) == 0u) && (g_tx_busy == 1u)) {
        return 0;
    }

    g_tx_busy = 1u;
    DMA1->clr = DMA1_FDT7_FLAG;
    dma_channel_enable(DMA1_CHANNEL7, FALSE);
    DMA1_CHANNEL7->maddr = (uint32_t)payload;
    DMA1_CHANNEL7->dtcnt = len;
    dma_channel_enable(DMA1_CHANNEL7, TRUE);
    return 1;
}

/**
 * \brief           Enqueue a Ninebot reply into the USART2 TX frame ring
 * \note            Guarded by the ring busy/lock bytes; when a free slot is available and the
 *                      body fits (< \ref VCU_NINEBOT_REPLY_MAX), it assembles the frame straight
 *                      into the tail slot's payload, stores the on-wire length, advances the tail
 *                      (mod 8) and consumes a free slot. The super-loop sequencer
 *                      (vcu_usart_tx_sequencer) later transmits the queued slot.
 * \param[in,out]   ctx: TX ring context (g_uartctx)
 * \param[in]       src: header src
 * \param[in]       dst: header dst
 * \param[in]       body_len: Ninebot length field
 * \param[in]       cmd: header cmd
 * \param[in]       arg: header arg
 * \param[in]       body: body bytes
 */
static void usart_tx_enqueue(volatile uint8_t* ctx, uint8_t src, uint8_t dst, uint8_t body_len, uint8_t cmd,
                                 uint8_t arg, const uint8_t* body) {
    volatile vcu_uartctx_t* ring = (volatile vcu_uartctx_t*)ctx;

    if ((ring->busy == 0u) && (ring->lock == 0u)) {
        ring->busy = 1u;
        if ((body_len < VCU_NINEBOT_REPLY_MAX) && (ring->free != 0u)) {
            uint8_t tail = ring->tail;
            uint16_t total = usart_frame_assemble(src, dst, body_len, cmd, arg, body,
                                                      (uint8_t*)ring->slot[tail].frame);

            ring->slot[tail].len = total;
            tail = (uint8_t)(tail + 1u);
            ring->tail = tail; /* advance, then wrap with a second store */
            if (tail == VCU_UART_SLOTS) {
                ring->tail = 0u;
            }
            ring->free = (uint8_t)(ring->free - 1u);
            ring->busy = 0u;
            return;
        }
        ring->busy = 0u;
    }
}

/* ------------------------------------------------------------------------- */
/* Shared report framer: vcu_usart_report_send (usart.h).                    */
/* ------------------------------------------------------------------------- */

void vcu_usart_report_send(volatile uint8_t* ctx, uint8_t src, uint8_t dst, uint8_t len, uint8_t cmd, uint8_t arg,
                           const volatile void* payload) {
    usart_tx_enqueue(ctx, src, dst, len, cmd, arg, (const uint8_t*)payload);
}

/* ========================================================================= */
/* super-loop poll #3: USART reply-frame sequencer                           */
/* ========================================================================= */
void vcu_usart_tx_sequencer(void) {
    /* Send one queued frame per pass when not busy/locked and head != tail. */
    if (g_uartctx.busy == 0 && g_uartctx.lock == 0 && g_uartctx.tail != g_uartctx.head) {
        uint8_t head = g_uartctx.head;
        g_uartctx.lock = 1;
        if (usart_tx_dma_arm(g_uartctx.slot[head].frame, (uint16_t)g_uartctx.slot[head].len) != 0) {
            uint8_t next = (uint8_t)(head + 1);
            g_uartctx.head = next;
            if (next == VCU_UART_SLOTS) {
                g_uartctx.head = 0; /* ring of 8 slots */
            }
            g_uartctx.free = (uint8_t)(g_uartctx.free + 1); /* release one free slot */
        }
        g_uartctx.lock = 0;
    }
}
