/**
 * \file            can.c
 * \brief           CAN1 motor/battery bus driver + Ninebot tunnel (x3 VCU)
 * \note            CAN subsystem of the VCU on the Artery AT32F415 BSP: CAN1 bring-up and
 *                      re-init, the RX/TX software rings and their ISR/poll ends, the generic
 *                      standard-id and extended-id senders, the Ninebot single-frame ext-id
 *                      builder, the ESC/BMS telemetry decoder, the periodic TX scheduler and
 *                      the battery-2 presence tick.
 *
 *                      The 20-byte frame record (\ref vcu_can_msg_t) is the ring format.
 *                      The hardware mailbox read/write goes through the BSP
 *                      `can_message_receive`/`can_message_transmit`, whose own
 *                      `can_rx/tx_message_type` structs have a different layout; the driver
 *                      converts at that single boundary.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "can.h"
#include "backup.h"   /* vcu_backup_can_response (ESC backup relay) */
#include "board.h"    /* board_irq_enable */
#include "busproto.h" /* bp_frame_t + vcu_busproto_rx (ext-id TP tunnel route) */
#include "buzzer.h"   /* buzzer_beep (battery-2 presence beeps) */
#include "gpio.h"     /* vcu_gpio_config */
#include "ninebot.h"  /* nb_reply_fc (0xEF route) */
#include "report.h"   /* report_can_toggle (drive-active report) */
#include "sensor.h"   /* vcu_sensor_temp (CAN 0x205 bytes 4..5) */
#include "vcu_state.h"

/* ------------------------------------------------------------------------- */
/* Bit timing (baud struct `{9,1,10,3}`)                                     */
/* prescaler /9, SJW 2 Tq, BS1 11 Tq, BS2 4 Tq => 1+11+4 = 16 Tq/bit, 75%    */
/* sample point. APB1 = SYSCLK/2 = 72 MHz, so 72 MHz / (9 * 16) = 500 kbit/s */
/* (500 kbit follows from the APB1 divider set in clock.c).                  */
/* ------------------------------------------------------------------------- */
#define CAN_BAUD_DIV             9u /* prescaler /9: btmg brdiv = div - 1 = 8 */

/* btmg fields written by the init: brdiv [11:0], bts1 [19:16], bts2 [22:20], rsaw [25:24],
   lben [30], loen [31] (lben = loen = 0: normal communication mode). */
#define CAN_BTMG_CFG_MASK        0xC37F0FFFu
#define CAN_BTMG_BRDIV           (CAN_BAUD_DIV - 1u)
#define CAN_BTMG_500K            (CAN_BTMG_BRDIV | (CAN_BTS1_11TQ << 16) | (CAN_BTS2_4TQ << 20) | (CAN_RSAW_2TQ << 24))

/* mctrl fields written by the init: mmssr [2], mdrsel [3], prsfen [4], aeden [5], aeboen [6],
   ttcen [7]. Only aeden (automatic exit from doze) is set. */
#define CAN_MCTRL_CFG_MASK       0xFCu
#define CAN_MCTRL_AEDEN          0x20u

/** \brief Polls of msts.fzc before a freeze-mode request or release gives up (the BSP's FZC_TIMEOUT). */
#define CAN_FZC_POLLS            0xFFFFu

/** \brief Interrupts enabled by the init: FIFO0 message, bus-off, error type record, error occur. */
#define CAN_INTEN_MASK           (CAN_RF0MIEN_INT | CAN_BOIEN_INT | CAN_ETRIEN_INT | CAN_EOIEN_INT)

/* CAN1 pins after the CAN1_GMUX_0010 remap. */
#define CAN_GPIO_PORT            GPIOB
#define CAN_TX_PIN               GPIO_PINS_9  /* CAN1_TX (remapped)                         */
#define CAN_RX_PIN               GPIO_PINS_8  /* CAN1_RX (remapped)                         */
#define CAN_STB_PIN              GPIO_PINS_15 /* transceiver STB/EN, active-low             */

/* ------------------------------------------------------------------------- */
/* Behavioural constants.                                                    */
/* ------------------------------------------------------------------------- */

/** \brief Telemetry frames are all full 8-byte frames (DLC 8). */
#define CAN_PERIODIC_DLC         VCU_CAN_DLEN

/** \brief Bus-off ticks before a CAN re-init (re-init once `streak > 0x32`). */
#define CAN_BUSOFF_REINIT_STREAK 50u

/* g_candev (CAN device object) and g_can_sched (periodic-tick scheduler state) are two
   globals in vcu_state.h. vcu_can_tick reads the SE-ISR diagnostics (bus_off, reinit_cnt)
   through g_candev; the RX decoder and senders drive the same object (event_flags,
   event_wait, payload getter). So SE-ISR -> tick -> sender all share one object. */

/* candev object embedded-vtable init. */
static void vcu_candev_init(void);

/* ------------------------------------------------------------------------- */
/* Software rings, 32 slots x 20-byte records, each                         */
/*   { u32 wr; u32 rd; u8 entries[32][20]; }  (wr @ +0x00, rd @ +0x04,       */
/* entries @ +0x08, stride 0x14), \ref vcu_can_ring_t (declared in can.h).   */
/*                                                                           */
/* RX ring: pushed from the RX0 ISR, popped in the super-loop poll #2.       */
/* Single producer (RX0 ISR) / single consumer.                              */
/* TX ring: enqueued by the senders, pumped by the super-loop poll #1.       */
/*                                                                           */
/* g_rx_ring is shared with CAN1_RX0_IRQHandler (producer) and the super-loop */
/* vcu_can_rx_dispatch (consumer), so the whole object is volatile: the       */
/* record copy and the index store that publishes it stay in program order.  */
/* g_tx_ring is enqueued and pumped from thread context only.                */
/* ------------------------------------------------------------------------- */
static volatile vcu_can_ring_t g_rx_ring;
static vcu_can_ring_t g_tx_ring;

/* ------------------------------------------------------------------------- */
/* Byte-wise copy of one 20-byte frame record.                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Copy a 20-byte CAN frame record
 * \note            Byte-wise, both sides volatile-qualified so the one helper serves the
 *                      volatile RX ring and the plain TX ring / stack records without casting
 *                      a qualifier away.
 * \param[out]      dst: destination record
 * \param[in]       src: source record
 */
static void can_frame_copy(volatile vcu_can_msg_t* dst, const volatile vcu_can_msg_t* src) {
    volatile uint8_t* d = (volatile uint8_t*)dst;
    const volatile uint8_t* p = (const volatile uint8_t*)src;
    uint8_t i;

    for (i = 0; i < sizeof(vcu_can_msg_t); i++) {
        d[i] = p[i];
    }
}

/* ------------------------------------------------------------------------- */
/* Init: GPIO / controller / re-init                                         */
/* ------------------------------------------------------------------------- */

/**
 * \brief           CAN1 pins: remap to PB9 = CAN_TX / PB8 = CAN_RX, plus PB15 output
 * \note            Enables the GPIOB and IOMUX clocks, applies `CAN1_GMUX_0010`
 *                      (0x2C000402: CAN1 on PB8/PB9), then runs three gpio_init calls on
 *                      GPIOB, all push-pull / drive stronger: PB9 mux no-pull (TX), PB8
 *                      input pull-up (RX), PB15 output no-pull. PB15 is the CAN-path pin the
 *                      boot bring-up drives low right after (transceiver STB/EN, active-low
 *                      standby). PA11/PA12 are NOT CAN pins on this board: the faultmon GPIO
 *                      init makes them floating node inputs.
 */
static void can_gpio_init(void) {
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_GPIOB_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_IOMUX_PERIPH_CLOCK));
    board_pin_remap(CAN1_GMUX_0010); /* CAN1 -> PB8 (RX) / PB9 (TX) */

    vcu_gpio_config(CAN_GPIO_PORT, CAN_TX_PIN, VCU_GPIO_CFG_MUX_PP);  /* PB9 : CAN_TX, mux push-pull   */
    vcu_gpio_config(CAN_GPIO_PORT, CAN_RX_PIN, VCU_GPIO_CFG_IN_PU);   /* PB8 : CAN_RX, input pulled up */
    vcu_gpio_config(CAN_GPIO_PORT, CAN_STB_PIN, VCU_GPIO_CFG_OUT_PP); /* PB15: transceiver standby     */
}

/**
 * \brief           Request or release CAN1 freeze mode and wait for the acknowledge
 * \param[in]       enter: 1 to enter freeze mode, 0 to leave it
 * \return          1 when msts.fzc reads `enter` within \ref CAN_FZC_POLLS polls, else 0
 */
static uint32_t prv_can_freeze(uint32_t enter) {
    uint32_t polls = 0u;

    CAN1->mctrl_bit.fzen = enter;
    while ((CAN1->msts_bit.fzc != enter) && (polls != CAN_FZC_POLLS)) {
        polls++;
    }
    return (CAN1->msts_bit.fzc == enter) ? 1u : 0u;
}

/**
 * \brief           CAN1 hardware init: base cfg, bit timing, filter, NVIC, interrupts
 * \note            Leaves doze mode and, in one freeze-mode window, sets the mctrl options and
 *                      the bit timing:
 *                        mode normal (btmg lben = loen = 0), TTCM off, auto bus-off off,
 *                        AWUM/auto-wake (aeden) on, retransmission on (prsfen = 0), RFLM off
 *                        (mdrsel), TXFP off (mmssr);
 *                        /9, SJW 2 Tq, BS1 11 Tq, BS2 4 Tq -> 500 kbit.
 *                      These are the values can_base_init + can_baudrate_set write; when freeze
 *                      mode is not acknowledged neither is written.
 */
static void can_hw_init(void) {
    /* Enable the CAN1 peripheral clock. */
    BOARD_CLOCK_ON(CRM_CAN1_PERIPH_CLOCK);

    CAN1->mctrl_bit.dzen = FALSE; /* exit doze mode */
    if (prv_can_freeze(1u) != 0u) {
        CAN1->mctrl = (CAN1->mctrl & ~CAN_MCTRL_CFG_MASK) | CAN_MCTRL_AEDEN;
        CAN1->btmg = (CAN1->btmg & ~CAN_BTMG_CFG_MASK) | CAN_BTMG_500K;
        (void)prv_can_freeze(0u);
    }

    /* Filter bank 0, 32-bit mask mode, id = mask = 0 -> accept ALL into FIFO0 (what
       can_filter_init writes for it). No hardware id filtering; routing is entirely in
       software. */
    CAN1->fctrl_bit.fcs = TRUE; /* filter configuration mode */
    CAN1->facfg &= ~1u;         /* bank 0 inactive           */
    CAN1->fbwcfg |= 1u;         /* 32-bit                    */
    CAN1->ffb[0].ffdb1 = 0u;    /* id 0                      */
    CAN1->ffb[0].ffdb2 = 0u;    /* mask 0                    */
    CAN1->fmcfg &= ~1u;         /* id/mask mode              */
    CAN1->frf &= ~1u;           /* FIFO0                     */
    CAN1->facfg |= 1u;          /* bank 0 active             */
    CAN1->fctrl_bit.fcs = FALSE;

    /* NVIC enable SE (IRQ22) then RX0 (IRQ20). */
    board_irq_enable(CAN1_SE_IRQn, BOARD_IRQ_PRIO(0, 0));
    board_irq_enable(CAN1_RX0_IRQn, BOARD_IRQ_PRIO(0, 0));

    /* Interrupt enables: inten |= FMP0(0x2) | BOF(0x400) | ETR/last-error(0x800) |
       EOI/error-occur(0x8000). */
    CAN1->inten |= CAN_INTEN_MASK;
}

/**
 * \brief           CAN software-state init: empty rings + candev embedded vtable
 * \note            No hardware here: empties the rings, seeds the candev countdowns and
 *                      installs the candev payload getter. The CAN1 boot bring-up is
 *                      \ref vcu_can_lowlevel_init (called from vcu_periph_init) and the
 *                      bus-off re-init is \ref vcu_can_mode_reset. Called once from main()
 *                      before vcu_periph_init, so the RX ring is ready before the CAN IRQs
 *                      are enabled.
 */
void vcu_can_init(void) {
    g_rx_ring.wr = 0;
    g_rx_ring.rd = 0;
    g_tx_ring.wr = 0;
    g_tx_ring.rd = 0;
    /* The two candev countdowns start at their reload value (200). */
    g_candev.timer_a = VCU_CANDEV_TIMER_RELOAD;
    g_candev.timer_b = VCU_CANDEV_TIMER_RELOAD;

    /* Populate the candev object's embedded vtable. */
    vcu_candev_init();
}

/**
 * \brief           CAN1 boot bring-up: GPIO + controller, transceiver enabled
 * \note            CAN GPIO init, CAN hardware init, then PB15 (GPIOB 0x8000) low.
 *                      No can_reset and no counter clear: that is what makes it distinct
 *                      from \ref vcu_can_mode_reset. Called once from vcu_periph_init.
 */
void vcu_can_lowlevel_init(void) {
    can_gpio_init();
    can_hw_init();
    gpio_bits_reset(CAN_GPIO_PORT, CAN_STB_PIN); /* PB15 low: transceiver out of standby */
}

/**
 * \brief           Pulse the CAN1 peripheral reset (see can.h)
 */
void vcu_can_periph_reset(void) {
    CRM->apb1rst_bit.can1rst = TRUE;
    CRM->apb1rst_bit.can1rst = FALSE;
}

/**
 * \brief           CAN1 bring-up + error/mode-field clear (declared in can.h)
 * \note            CAN1 reset, CAN GPIO init, CAN hardware init, then zero tec (+0x19),
 *                      rec (+0x1a), lec (+0x09) and reinit_cnt (+0x08), in that order.
 *
 *                      This is the bus-off / fault re-init path and is intentionally
 *                      DISTINCT from \ref vcu_can_init (software rings) and
 *                      \ref vcu_can_lowlevel_init (boot path): it clears only the four
 *                      error/mode fields in the shared CAN device block and does NOT
 *                      reset the software TX/RX rings.
 */
void vcu_can_mode_reset(void) {
    /* Reset/deinit CAN1 (0x40006400). */
    vcu_can_periph_reset();

    /* CAN GPIO. */
    can_gpio_init();

    /* Hardware init. */
    can_hw_init();

    /* zero the four error/mode fields in the shared handle. */
    g_candev.tec = 0;        /* +0x19 */
    g_candev.rec = 0;        /* +0x1a */
    g_candev.lec = 0;        /* +0x09 */
    g_candev.reinit_cnt = 0; /* +0x08 */
}

/* ------------------------------------------------------------------------- */
/* RX path: ring push (from the RX0 ISR)                                     */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Push a received frame onto the RX software ring
 * \note            Full-check is `rd != (wr + 1) & 0x1F`; on room, copy 20 bytes and advance
 *                      `wr = (wr + 1) & 0x1F`. Overrun silently drops the frame.
 * \param[in]       msg: frame just read out of the hardware FIFO
 */
static void can_rx_ring_push(const vcu_can_msg_t* msg) {
    if (g_rx_ring.rd != ((g_rx_ring.wr + 1u) & VCU_CAN_RING_MASK)) {
        can_frame_copy(&g_rx_ring.entry[g_rx_ring.wr], msg);
        g_rx_ring.wr = (g_rx_ring.wr + 1u) & VCU_CAN_RING_MASK;
    }
}

/* ------------------------------------------------------------------------- */
/* CAN RX decode: the standard-id decoder and the extended-id Ninebot/J1939  */
/* route. These are the RX consumers; they write the incoming ESC (motor) +  */
/* BMS (battery) telemetry into the status blackboard. The router that       */
/* selects std vs ext is vcu_can_rx_dispatch (below), keyed on the record's  */
/* ide/type byte (+0x08).                                                    */
/* ------------------------------------------------------------------------- */

/* Destination blackboards the std decoder writes: g_in2, g_telem_frame,    */
/* g_stat, g_vs, g_candev, g_fault_codes, g_chg, g_scr, g_mode, plus the two         */
/* comm-loss counters below.                                                 */
/* Comm-loss counters zeroed by every frame of their group. */
#define CANRX_ESC_WD        (g_lowglob.esc_commloss_cnt)
#define CANRX_BMS_WD        (g_bms_commloss.cnt)

/* Scheduler-timer re-arm masks cleared by the decoder tail (CANDEV->out_bits). */
#define CANRX_OUT_ESC_FAULT 0x02u                   /* bit1 : ESC comms-timeout flag */
#define CANRX_OUT_BMS_FAULT 0x04u                   /* bit2 : BMS comms-timeout flag */
#define CANRX_WD_RELOAD     VCU_CANDEV_TIMER_RELOAD /* scheduler-timer reload on a fresh frame */
#define CANRX_HS_EVENT_WAIT 500                     /* 0x7F1 handshake: event-gate countdown seed        */
#define CANRX_HS_EVENT_BIT  0x01u                   /* 0x7F1 handshake: event-pending flag (CANDEV+0x0c) */

/* Little-endian u16 from two payload bytes. */
static inline uint16_t can_rx_rd16(const volatile uint8_t* d, uint8_t i) {
    return (uint16_t)((uint16_t)d[i] | ((uint16_t)d[i + 1] << 8));
}

/* ASCII hex digit ('0'-'9','A'-'F','a'-'f') -> 0..15, else 0.
   Used by the 0x421 version frame to pack four nibbles into one halfword. */
static uint8_t can_rx_hex_nibble(uint8_t c) {
    if ((uint8_t)(c - 0x30u) > 9u && (uint8_t)(c - 0x41u) > 5u && (uint8_t)(c - 0x61u) > 5u) {
        return 0u; /* not a hex digit */
    }
    if (c < 0x41u) {
        return (uint8_t)(c - 0x30u); /* '0'..'9' */
    }
    if (c < 0x61u) {
        return (uint8_t)(c - 0x37u); /* 'A'..'F' -> 10..15 */
    }
    return (uint8_t)(c - 0x57u); /* 'a'..'f' -> 10..15 */
}

/* Battery-2 presence tick constants. */
#define BAT2_COMMLOSS_LIMIT 200u /* ticks without a battery-2 frame */
#define BAT2_SOC_LOW_MAX    5u   /* SoC <= 5 % is low               */

/* Words of the last CAN 0x42 frame, declared in can.h. */
volatile uint32_t g_can_rx42[2];

/* Little-endian u32 from four payload bytes. */
static inline uint32_t can_rx_rd32(const volatile uint8_t* d, uint8_t i) {
    return (uint32_t)can_rx_rd16(d, i) | ((uint32_t)can_rx_rd16(d, (uint8_t)(i + 2u)) << 16);
}

/* Four ASCII-hex digits d[3] d[2] d[1] d[0] -> one halfword (0x421 / 0x431 version frames). */
static uint16_t can_rx_hex_version(const volatile uint8_t* d) {
    return (uint16_t)(((uint16_t)can_rx_hex_nibble(d[3]) << 12) | ((uint16_t)can_rx_hex_nibble(d[2]) << 8)
                      | ((uint16_t)can_rx_hex_nibble(d[1]) << 4) | (uint16_t)can_rx_hex_nibble(d[0]));
}

/**
 * \brief           Battery-2 frame: battery 2 is present and its comm-loss count restarts
 * \note            The common tail of 0x430/0x431/0x432/0x435: SCR+0x0A = 1,
 *                      g_bat2.commloss_cnt = 0.
 */
static void can_rx_bat2_seen(void) {
    g_scr.bat2_present = 1u;
    g_bat2.commloss_cnt = 0u;
}

/**
 * \brief           Standard-id ESC/BMS telemetry decoder
 * \param[in]       msg: the 20-byte record popped from the RX ring
 * \note            Switches on the 11-bit id and copies payload halfwords/bytes into the status
 *                      blackboard, zeroing the matching comm-loss count; the tail re-arms the
 *                      scheduler timers. 0x42 and 0x430/0x431/0x432/0x435 (battery 2) carry no
 *                      ESC/BMS comm-loss count, 0x211 arms the node 0x02 push, 0x425 sets
 *                      IN1+0x02. The cell temperatures are id 0x500 and the BMS keep-alives
 *                      0x501/0x502.
 */
static void can_rx_std(const volatile uint8_t* msg) {
    const volatile vcu_canmsg_t* m = (const volatile vcu_canmsg_t*)msg;
    const volatile uint8_t* d = m->data; /* d[0..7] = record bytes +0x0B..+0x12 */
    uint32_t id = m->std_id;

    switch (id) {
        case CANID_RX_AUX_42: /* 0x42: two words, no comm-loss count */
            g_can_rx42[0] = can_rx_rd32(d, 0);
            g_can_rx42[1] = can_rx_rd32(d, 4);
            break;

        case CANID_RX_ESC_A:            /* 0x211 */
            if ((d[3] & 0x04u) != 0u) { /* bit2 of data[3]: charge-detected latch */
                g_chg.f5 = 1;
            }
            g_in2.f.speed_src = can_rx_rd16(d, 6);
            if (g_node2_push.state == NODE2_PUSH_IDLE) { /* arm the node 0x02 push */
                g_node2_push.state = NODE2_PUSH_PENDING;
            }
            CANRX_ESC_WD = 0;
            break;

        case CANID_RX_ESC_B: /* 0x212 */
            g_in2.f.canrx_212 = can_rx_rd16(d, 0);
            g_fault_codes.comm_fault_word = can_rx_rd16(d, 2);
            g_fault_codes.vbus_code = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[7] << 8));
            g_fault_codes.active_code_lo = d[6];
            g_telem_frame[REPORT_TELEM_CAN212_W1] = can_rx_rd16(d, 2);
            g_telem_frame[REPORT_TELEM_CAN212_W3] = can_rx_rd16(d, 6);
            CANRX_ESC_WD = 0;
            break;

        case CANID_RX_BMS_A: /* 0x310: d[4..5] is not stored */
            g_stat.curr_limit = can_rx_rd16(d, 0);
            g_stat.canrx_310b = can_rx_rd16(d, 2);
            g_stat.canrx_310a = can_rx_rd16(d, 6);
            CANRX_BMS_WD = 0;
            break;

        case CANID_RX_BMS_B: { /* 0x311 : accel-clamped current */
            uint8_t v0 = d[0];
            uint8_t v1 = d[1];

            if (g_vs.accel_limit_fwd <= v0) { /* clamp to VS->accel_limit_fwd */
                v0 = g_vs.accel_limit_fwd;
            }
            g_mode.canrx_12 = v0;
            if (g_vs.accel_limit_rev <= v1) { /* clamp to VS->accel_limit_rev */
                v1 = g_vs.accel_limit_rev;
            }
            g_mode.spd_cmp = v1;
            g_mode.canrx_14 = d[2];
            g_stat.canrx_311c = can_rx_rd16(d, 0);
            g_stat.canrx_311b = can_rx_rd16(d, 2);
            g_stat.canrx_311a = can_rx_rd16(d, 4);
            CANRX_BMS_WD = 0;
            break;
        }

        case CANID_RX_ESC_C: /* 0x341 */
            g_telem_frame[REPORT_TELEM_CAN341_W0] = can_rx_rd16(d, 0);
            CANRX_ESC_WD = 0;
            break;

        case CANID_RX_ESC_KEEPALV: /* 0x342 */ CANRX_ESC_WD = 0; break;

        case CANID_RX_ESC_D: /* 0x344 */
            g_in2.canrx_1e8_scratch = can_rx_rd16(d, 0);
            CANRX_ESC_WD = 0;
            break;

        case CANID_RX_ESC_E: /* 0x401 */
            g_fault_codes.vbus_code_hi = d[5];
            g_telem_frame[REPORT_TELEM_CAN401_D5] = (uint16_t)((uint16_t)d[5] | ((uint16_t)d[6] << 8));
            g_telem_frame[REPORT_TELEM_CAN401_W0] = can_rx_rd16(d, 0);
            g_telem_frame[REPORT_TELEM_CAN401_W1] = can_rx_rd16(d, 2);
            CANRX_ESC_WD = 0;
            break;

        case CANID_RX_BMS_SOC: /* 0x420: no floor on the SCR copy */
            g_stat.curr_sample = can_rx_rd16(d, 0);
            g_stat.volt_sample = can_rx_rd16(d, 2);
            g_stat.soc = (int16_t)((int16_t)can_rx_rd16(d, 4) / 10);
            g_stat.canrx_420a = can_rx_rd16(d, 6);
            g_scr.soc_raw_nz = (uint16_t)g_stat.soc;
            CANRX_BMS_WD = 0;
            break;

        case CANID_RX_BMS_VERSION: /* 0x421 : ASCII-hex version nibbles */
            g_stat.canrx_421a = can_rx_hex_version(d);
            g_stat.canrx_421b = can_rx_rd16(d, 4);
            g_stat.canrx_310_421 = can_rx_rd16(d, 6);
            g_in2.f.canrx_421 = g_stat.canrx_421a;
            CANRX_BMS_WD = 0;
            break;

        case CANID_RX_BMS_C: /* 0x422 */
            g_stat.canrx_422a = can_rx_rd16(d, 0);
            g_stat.canrx_422b = can_rx_rd16(d, 2);
            g_stat.status_word_a = can_rx_rd16(d, 4);
            g_stat.status_word_b = can_rx_rd16(d, 6);
            CANRX_BMS_WD = 0;
            break;

        case CANID_RX_BMS_D: /* 0x423 */
            g_stat.canrx_423a = can_rx_rd16(d, 0);
            g_stat.chg_limit = can_rx_rd16(d, 2);
            g_stat.canrx_423b = can_rx_rd16(d, 4);
            g_stat.canrx_423c = can_rx_rd16(d, 6);
            g_scr.canrx_423 = g_stat.canrx_423b;
            CANRX_BMS_WD = 0;
            break;

        case CANID_RX_BMS_E: /* 0x424: VS+0x18 |= 0x2000, the report-enable bit */
            g_stat.status_lo = can_rx_rd16(d, 0);
            g_stat.status_hi = can_rx_rd16(d, 2);
            g_vs.report_enable = 1; /* VS report-enable */
            CANRX_BMS_WD = 0;
            break;

        case CANID_RX_BMS_TEMP2: /* 0x425: received flag, temp + display, no count */
            g_in1.bms_temp_rx = 1u;
            g_in2.f.pack_temp = (uint16_t)(((int)(uint8_t)d[2] - (int)CAN_RX_TEMP_BIAS) * 10);
            g_telem_frame[REPORT_TELEM_CAN425] = (uint16_t)((uint16_t)d[2] | ((uint16_t)d[5] << 8));
            break;

        case CANID_RX_BAT2_SOC: /* 0x430: as 0x420 into battery 2, no SCR copy */
            g_stat2.curr_sample = can_rx_rd16(d, 0);
            g_stat2.volt_sample = can_rx_rd16(d, 2);
            g_stat2.soc = (int16_t)((int16_t)can_rx_rd16(d, 4) / 10);
            g_stat2.canrx_420a = can_rx_rd16(d, 6);
            can_rx_bat2_seen();
            break;

        case CANID_RX_BAT2_VER: /* 0x431: as 0x421 into battery 2, version -> IN2+0x34 */
            g_stat2.canrx_421a = can_rx_hex_version(d);
            g_stat2.canrx_421b = can_rx_rd16(d, 4);
            g_stat2.canrx_310_421 = can_rx_rd16(d, 6);
            g_in2.f.bat2_version = g_stat2.canrx_421a;
            can_rx_bat2_seen();
            break;

        case CANID_RX_BAT2_C: /* 0x432: as 0x422 into battery 2 */
            g_stat2.canrx_422a = can_rx_rd16(d, 0);
            g_stat2.canrx_422b = can_rx_rd16(d, 2);
            g_stat2.status_word_a = can_rx_rd16(d, 4);
            g_stat2.status_word_b = can_rx_rd16(d, 6);
            can_rx_bat2_seen();
            break;

        case CANID_RX_BAT2_TEMP: /* 0x435: one battery-2 temperature */
            g_stat2.temp_array[0] = (uint16_t)((int)(uint8_t)d[2] - (int)CAN_RX_TEMP_BIAS);
            can_rx_bat2_seen();
            break;

        case CANID_RX_BMS_TEMP: { /* 0x500 : 8 cell temperatures, each byte - 0x32 */
            uint8_t i;

            for (i = 0; i < VCU_CAN_DLEN; i++) {
                g_stat.temp_array[i] = (uint16_t)((int)(uint8_t)d[i] - (int)CAN_RX_TEMP_BIAS);
            }
            CANRX_BMS_WD = 0;
            break;
        }

        case CANID_RX_BMS_KEEPAL0: /* 0x501 */
        case CANID_RX_BMS_KEEPAL1: /* 0x502 */ CANRX_BMS_WD = 0; break;

        case CANID_RX_HANDSHAKE: /* 0x7F1 : drives the periodic-sender event gate */
            g_candev.event_wait = CANRX_HS_EVENT_WAIT;
            if ((d[0] & 1u) == 0u) {
                g_candev.event_flags = (uint16_t)(g_candev.event_flags & ~CANRX_HS_EVENT_BIT);
            } else {
                g_candev.event_flags = (uint16_t)(g_candev.event_flags | CANRX_HS_EVENT_BIT);
            }
            break;

        case CANID_BACKUP_RSP: /* 0x7DF : relay the MCU's read-only backup chunk to the app */
            vcu_backup_can_response(m->data, m->dlc);
            break;

        default: break;
    }

    /* Comms-timeout re-arm. While each group's watchdog is fresh (0),
       reload its scheduler countdown to 200 and clear its comms-fault out-bit. */
    if (CANRX_ESC_WD == 0) {
        g_candev.timer_a = CANRX_WD_RELOAD;
        g_candev.out_bits = (uint8_t)(g_candev.out_bits & ~CANRX_OUT_ESC_FAULT);
    }
    if (CANRX_BMS_WD == 0) {
        g_candev.timer_b = CANRX_WD_RELOAD;
        g_candev.out_bits = (uint8_t)(g_candev.out_bits & ~CANRX_OUT_BMS_FAULT);
    }
}

/**
 * \brief           Battery-2 presence and SoC-low tick (see can.h)
 * \note            Called from control_power_monitor_fsm. While present: SCR+0x0E = B2 SoC,
 *                      g_bat2.soc_low = (SoC <= 5), and the first time SF_DUAL_PACK (IN2+0x38
 *                      bit9) is raised it beeps(3) in state 3. While SF_DUAL_PACK is up the
 *                      count advances and is held at 200; there, not present clears the bit;
 *                      present with SoC not low drops the pack, clears the bit and beeps(1) in
 *                      state 3; present with SoC low drops it in state 4 only (the bit follows
 *                      on the next tick).
 */
void vcu_can_battery2_presence_tick(void) {
    uint16_t ticks;

    if (g_scr.bat2_present != 0u) {
        g_scr.bat2_soc = g_stat2.soc_u;
        g_bat2.soc_low = (g_scr.bat2_soc <= BAT2_SOC_LOW_MAX) ? 1u : 0u;
        if ((g_in2.f.status_flags & SF_DUAL_PACK) == 0u) {
            g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_DUAL_PACK);
            if (g_state_block.fsm_state == VCU_FSM_RIDE) {
                buzzer_beep(BEEP_DOUBLE);
            }
        }
    }
    if ((g_in2.f.status_flags & SF_DUAL_PACK) != 0u) {
        ticks = g_bat2.commloss_cnt;
        g_bat2.commloss_cnt = (uint16_t)(ticks + 1u);
        if (ticks >= BAT2_COMMLOSS_LIMIT) {
            g_bat2.commloss_cnt = BAT2_COMMLOSS_LIMIT;
            if (g_scr.bat2_present == 0u) {
                g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & ~SF_DUAL_PACK);
            } else if (g_bat2.soc_low == 0u) {
                g_scr.bat2_present = 0u;
                g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & ~SF_DUAL_PACK);
                if (g_state_block.fsm_state == VCU_FSM_RIDE) {
                    buzzer_beep(BEEP_LONG);
                }
            } else if (g_state_block.fsm_state == VCU_FSM_POWER_DOWN) {
                g_scr.bat2_present = 0u;
            } else {
                /* SoC low outside state 4: battery 2 stays present */
            }
        }
    }
}

/* Extended-id (29-bit) handler. Builds a bp_frame_t from the record and routes by
   the J1939 PGN-format byte: PF 0xEF -> the Ninebot report/log reply framer
   (nb_reply_fc); every other PF (TP.CM 0xEC / TP.DT 0xEB) -> the TP tunnel
   reassembly (vcu_busproto_rx). */
static void can_rx_ext(const volatile uint8_t* msg) {
    const volatile vcu_canmsg_t* m = (const volatile vcu_canmsg_t*)msg;
    uint32_t ext_id = m->ext_id;
    bp_frame_t frame;
    uint8_t i;

    frame.id = ext_id & VCU_CAN_EXT_ID_MASK;
    frame.dlc = m->dlc;
    for (i = 0; i < VCU_CAN_DLEN; i++) {
        frame.data[i] = m->data[i];
    }

    if (((ext_id >> 16) & 0xFFu) == CAN_EXT_PF_REPORT) {
        /* nb_reply_fc(NULL, src, dst, payload, dlc); the leading arg is unused. */
        nb_reply_fc(NULL, (uint8_t)(ext_id & 0xFFu), (uint8_t)((ext_id >> 8) & 0xFFu), (const uint8_t*)frame.data,
                    (char)m->dlc);
    } else {
        vcu_busproto_rx(&frame); /* TP.CM/TP.DT tunnel reassembly */
    }
}

/* CAN RX software-ring occupancy: the RX ring write index minus its read index,
   wrapped to a 0x20-entry ring (g_rx_ring). */
static int32_t can_rx_count(void) {
    /* one snapshot of each index: the RX0 ISR may advance wr between two reads */
    uint32_t wr = g_rx_ring.wr;
    uint32_t rd = g_rx_ring.rd;
    uint32_t used = wr - rd;

    if (rd <= wr) {
        return (int32_t)(used & 0xFFFFu);
    }
    return (int32_t)((used + VCU_CAN_RING_LEN) & 0xFFFFu);
}

/* ------------------------------------------------------------------------- */
/* TX path: enqueue / count / pump                                          */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Enqueue one frame onto the software TX ring (see can.h)
 * \note            Full-check `rd != (wr+1)&0x1F`, copy 20 bytes, `wr = (wr+1)&0x1F`.
 *                      Overrun drops the frame.
 */
void vcu_can_tx_enqueue(const vcu_can_msg_t* msg) {
    if (g_tx_ring.rd != ((g_tx_ring.wr + 1u) & VCU_CAN_RING_MASK)) {
        can_frame_copy(&g_tx_ring.entry[g_tx_ring.wr], msg);
        g_tx_ring.wr = (g_tx_ring.wr + 1u) & VCU_CAN_RING_MASK;
    }
}

/**
 * \brief           Count the frames currently queued on the software TX ring
 * \note            `n = wr - rd`; when the write index has not wrapped below the read
 *                      index (`rd <= wr`) the difference is the occupancy directly, otherwise
 *                      the ring depth (0x20) is added before masking to 16 bits. The pump
 *                      (`can_tx_pump`) calls it as its empty-gate.
 * \return          Number of frames queued (0 when empty)
 */
static uint16_t can_tx_ring_count(void) {
    uint32_t n = g_tx_ring.wr - g_tx_ring.rd;

    if (g_tx_ring.rd <= g_tx_ring.wr) {
        return (uint16_t)(n & 0xFFFFu);
    }
    return (uint16_t)((n + VCU_CAN_RING_LEN) & 0xFFFFu); /* (wr-rd+0x20)&0xffff */
}

/**
 * \brief           Hand the oldest queued frame to a free hardware TX mailbox
 * \note            If the ring is empty, or the peripheral is mid-transmit, do nothing;
 *                      else peek the oldest frame, write it to a free mailbox (the BSP
 *                      transmitter scans TSR for TME0/1/2 and sets TXRQ), and advance `rd`
 *                      only when a mailbox accepted it. At most one frame per call.
 *
 *                      The 20-byte frame record (\ref vcu_can_msg_t) is converted here into
 *                      the BSP `can_tx_message_type`, which the BSP writes to
 *                      TIR/TDTR/TDLR/TDHR.
 * \return          `1` if a frame was handed to hardware, `0` otherwise
 */
static uint8_t can_tx_pump(void) {
    can_tx_message_type tx;
    vcu_can_msg_t frame;
    uint8_t i;
    uint8_t mailbox;

    /* Entry gate: nothing queued -> nothing to do. */
    if (can_tx_ring_count() == 0u) {
        return 0;
    }
    /* Skip while the peripheral reports a transmit in progress. */
    if (CAN1->msts_bit.cuss != RESET) {
        return 0;
    }
    /* Peek the oldest frame. The ring is not empty here, so the copy always happens;
       the pump advances `rd`. */
    can_frame_copy(&frame, &g_tx_ring.entry[g_tx_ring.rd]);

    tx.id_type = (frame.ide != 0u) ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
    tx.frame_type = (frame.rtr != 0u) ? CAN_TFT_REMOTE : CAN_TFT_DATA;
    tx.standard_id = frame.std_id;
    tx.extended_id = frame.ext_id;
    tx.dlc = frame.dlc;
    for (i = 0; i < VCU_CAN_DLEN; ++i) {
        tx.data[i] = frame.data[i];
    }

    mailbox = can_message_transmit(CAN1, &tx);
    if (mailbox == CAN_TX_STATUS_NO_EMPTY) {
        return 0; /* all mailboxes busy: leave the frame queued */
    }
    g_tx_ring.rd = (g_tx_ring.rd + 1u) & VCU_CAN_RING_MASK; /* (rd+1)&0x1F */
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Poll: pump the TX ring (super-loop poll #1)                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           CAN TX pump: hand at most one queued frame to a mailbox
 * \note            Called every super-loop pass. The poll touches the TX ring only; the
 *                      RX ring is drained one frame per pass by the separate poll #2,
 *                      \ref vcu_can_rx_dispatch.
 */
void vcu_can_poll(void) { (void)can_tx_pump(); }

/* ------------------------------------------------------------------------- */
/* Generic senders: standard id / extended id                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Send a payload as standard-id data frame(s) (see can.h)
 * \note            `len==0` enqueues one empty frame; otherwise the payload is
 *                      fragmented into <=8-byte frames, each repeating the first bytes of
 *                      the payload (see the loop). IDE=0, RTR=0.
 */
void vcu_can_send_std(uint16_t len, uint32_t std_id, const uint8_t* payload) {
    vcu_can_msg_t msg;
    uint16_t off;
    uint8_t i;

    msg.ide = 0;
    msg.rtr = 0;
    msg.std_id = std_id;
    msg.ext_id = 0;
    msg.fmi = 0;

    if (len == 0u) {
        msg.dlc = 0;
        for (i = 0; i < VCU_CAN_DLEN; ++i) {
            msg.data[i] = 0;
        }
        vcu_can_tx_enqueue(&msg);
        return;
    }

    for (off = 0; off < len;) {
        uint16_t rem = (uint16_t)(len - off);
        uint8_t chunk = (uint8_t)((rem > VCU_CAN_DLEN) ? VCU_CAN_DLEN : rem);

        msg.dlc = chunk;
        /* Every fragment is copied from the payload START: the source pointer does not
           advance. Every product caller sends 8 bytes, so it is one frame in practice. */
        for (i = 0; i < VCU_CAN_DLEN; ++i) {
            msg.data[i] = (i < chunk && payload != 0) ? payload[i] : 0u;
        }
        vcu_can_tx_enqueue(&msg);
        off = (uint16_t)(off + chunk);
    }
}

/**
 * \brief           Send a single extended-id data frame (see can.h)
 * \note            Builds a 20-byte record with IDE=1, RTR=0, DLC=len, and enqueues it
 *                      only when len <= 8 (frames longer than 8 bytes are dropped).
 */
void vcu_can_send_ext(uint32_t ext_id, const uint8_t* payload, uint8_t len) {
    vcu_can_msg_t msg;
    uint8_t i;

    if (len > VCU_CAN_DLEN) {
        return; /* the `len < 9` guard drops over-length frames */
    }

    msg.ide = 1;
    msg.rtr = 0;
    msg.std_id = 0;
    msg.ext_id = ext_id & VCU_CAN_EXT_ID_MASK;
    msg.fmi = 0;
    msg.dlc = len;
    for (i = 0; i < VCU_CAN_DLEN; ++i) {
        msg.data[i] = (i < len && payload != 0) ? payload[i] : 0u;
    }
    vcu_can_tx_enqueue(&msg);
}

/* ------------------------------------------------------------------------- */
/* Ninebot single-frame ext-id builder                                       */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Build one Ninebot single-frame ext-id report (see can.h)
 * \note            The builder behind the short (n < 6) branch of the Ninebot
 *                      event-report sink (report_ninebot). It is the one Ninebot ext-id
 *                      builder. The id is J1939-shaped: priority [28:26], PF 0xEF
 *                      ("Proprietary A") [23:16], destination [15:8], source [7:0], plus
 *                      the bit29 marker; the sender masks the marker off.
 */
int vcu_can_ninebot_frame(uint8_t prio, uint8_t ctl, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg,
                          const uint8_t* payload, uint8_t n, vcu_can_ext_frame_t* out) {
    uint8_t i;

    if (payload == NULL || out == NULL || n > VCU_CAN_NB_MAX_PAYLOAD || prio >= VCU_CAN_NB_PRIO_MAX) {
        return 0;
    }
    out->id = (out->id & VCU_CAN_NB_ID_KEEP)
              | ((((uint32_t)prio << VCU_CAN_NB_PRIO_SHIFT) | ((uint32_t)dst << 8) | (uint32_t)src)
                 & VCU_CAN_EXT_ID_MASK)
              | VCU_CAN_NB_PF_EF | VCU_CAN_NB_ID_FLAG;
    out->dlc = (uint8_t)(n + VCU_CAN_NB_HDR_LEN);
    out->data[0] = ctl;
    out->data[1] = cmd;
    out->data[2] = arg;
    for (i = 0; i < n; i++) {
        out->data[VCU_CAN_NB_HDR_LEN + i] = payload[i];
    }
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Periodic TX scheduler                                                     */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Periodic-tick scheduler fields (shared block g_candev)
 * \note            Byte offsets from the g_candev base `S`. Offsets `-6`/`-4` are the two
 *                      halfwords just below `S`. The scheduler and the RX decoder / senders
 *                      share this store, so a write by the RX decoder / senders to g_candev
 *                      +0x0c / +0x0e is seen here. Several fields couple to other subsystems
 *                      (the event gate feeds the 0x7F0 sender / handshake).
 */
/* Output-flag bits raised by the two scheduler countdowns (CANDEV->out_bits). */
#define CAN_OUT_TIMER_A_BIT 2u /* S+0x18 bit1: timer_a expired */
#define CAN_OUT_TIMER_B_BIT 4u /* S+0x18 bit2: timer_b expired */

/* S+0x0c event-flag block: bit0 = a pending event holds off the tick. */
#define CAN_EVENT_PENDING   1u

/* ------------------------------------------------------------------------- */
/* Suspend gate (vcu_can_tick entry).                                        */
/*                                                                           */
/* When the handle flag IN1->drive_active is exactly 1, the periodic sender is  */
/* held off: the ESC / BMS comm-loss counters and the faultmon UV counter        */
/* (LOW->esc_commloss_cnt, g_bms_commloss, LOW->fm_uv_cnt) are zeroed and     */
/* the tick returns at once.                                                     */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Emit one standard-id telemetry frame via the payload getter
 * \param[in]       std_id: id to send (DLC fixed at 8)
 */
static void can_send_periodic(uint32_t std_id) {
    uint8_t payload[VCU_CAN_DLEN];

    vcu_can_payload_get(std_id, payload);                /* candev getter (S+0x1c)       */
    vcu_can_send_std(CAN_PERIODIC_DLC, std_id, payload); /* 8-byte standard-id frame     */
}

/* Store a 16-bit value little-endian into the 8-byte candev tx scratch. */
static inline void candev_put16(volatile uint8_t* p, uint32_t off, uint16_t v) {
    p[off] = (uint8_t)v;
    p[off + 1u] = (uint8_t)(v >> 8);
}

/**
 * \brief           candev CAN-id -> 8-byte telemetry payload getter
 * \param[in]       can_id: 11-bit standard id
 * \return          pointer to the 8-byte tx scratch at the candev base
 * \note            candev object +0x1C. Fills the first 8 bytes of g_candev from the
 *                      telemetry blackboards and returns that pointer. All frames are DLC 8;
 *                      an unknown id leaves the scratch stale. Sources: g_mode/g_in2/g_scr/
 *                      g_stat, the mirrors in g_telem_frame, and the IMU temperature.
 */
static volatile uint8_t* vcu_candev_payload_get(uint32_t can_id) {
    volatile uint8_t* p = g_candev.tx_payload; /* 8-byte tx scratch */

    switch (can_id) {
        case 0x100u:
            candev_put16(p, 0, g_mode.drive_output_word);
            candev_put16(p, 2, g_mode.mode_flags);
            candev_put16(p, 4, g_mode.curr_word);
            candev_put16(p, 6, g_mode.we_word);
            /* Count the frames in the history block (+0x1C, u32), saturating at
               0xFFFFFFFF. */
            {
                uint32_t sent = (uint32_t)g_telem_frame[REPORT_TELEM_TX100_LO]
                                | ((uint32_t)g_telem_frame[REPORT_TELEM_TX100_HI] << 16);

                if (sent != UINT32_MAX) {
                    sent++;
                    g_telem_frame[REPORT_TELEM_TX100_LO] = (uint16_t)sent;
                    g_telem_frame[REPORT_TELEM_TX100_HI] = (uint16_t)(sent >> 16);
                }
            }
            /* telemetry-frame mirror */
            g_telem_frame[REPORT_TELEM_CANDEV_100] = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
            break;
        case 0x203u:
            p[0] = (uint8_t)g_in2.f.gear; /* low byte */
            p[1] = 0;
            candev_put16(p, 2, g_scr.soc_combined);
            candev_put16(p, 4, g_stat.soc_u);
            candev_put16(p, 6, g_in2.f.speed_src);
            break;
        case 0x204u:
            candev_put16(p, 0, 0);
            candev_put16(p, 2, 0);
            candev_put16(p, 4, 0);
            candev_put16(p, 6, 0);
            break;
        case 0x205u:
            candev_put16(p, 0, 0);
            candev_put16(p, 2, 0);
            /* IMU temperature, 0.1 degC */
            candev_put16(p, 4, (uint16_t)vcu_sensor_temp());
            candev_put16(p, 6, 0);
            break;
        case 0x209u:
            candev_put16(p, 0, g_in2.f.led_ind_main);
            candev_put16(p, 2, g_in2.f.cfg_flags);
            candev_put16(p, 4, g_in2.f.status_flags);
            candev_put16(p, 6, 0);
            break;
        case 0x20Au:
            candev_put16(p, 0, g_scr.dtc_mask0);
            candev_put16(p, 2, g_scr.dtc_mask1);
            candev_put16(p, 4, g_scr.dtc_mask2);
            candev_put16(p, 6, g_scr.event_mask_lo);
            break;
        case 0x20Bu:
            candev_put16(p, 0, g_stat.canrx_423a);
            candev_put16(p, 2, g_stat.chg_limit);
            candev_put16(p, 4, g_stat.canrx_310a);
            candev_put16(p, 6, g_scr.drive_oneshot);
            break;
        case 0x20Cu:
            candev_put16(p, 0, g_mode.enable_flags);
            p[2] = g_mode.cmd_lead;
            p[3] = g_mode.canrx_12;
            p[4] = g_mode.spd_cmp;
            p[5] = g_mode.canrx_14;
            /* Byte 6 is the derate byte (staging +0x19), byte 7 is always 0. */
            p[6] = g_mode.cfg_17; /* derate flags */
            p[7] = 0;
            g_mode.error_code = g_lowglob.active_error_code;
            break;
        case 0x301u:
            p[0] = g_candev.tec;
            p[1] = g_candev.rec;
            p[2] = g_candev.lec;
            p[3] = g_candev.reinit_cnt;
            candev_put16(p, 4, 0);
            candev_put16(p, 6, 0);
            /* telemetry-frame mirror */
            g_telem_frame[REPORT_TELEM_CANDEV_301] = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
            break;
        case 0x302u:
            p[0] = 0x00;
            p[1] = 0xFF;
            p[2] = 0xFF;
            p[3] = 0xFF;
            p[4] = 0xFF;
            p[5] = 0xFF;
            p[6] = 0xFF;
            p[7] = 0xFF;
            break;
        case 0x480u:
            candev_put16(p, 0, (uint16_t)g_in2.f.trip_dist);
            candev_put16(p, 2, (uint16_t)g_in2.f.trip_ride_time);
            candev_put16(p, 4, g_in2.f.session_dist_reg[0]); /* lo */
            candev_put16(p, 6, g_in2.f.session_dist_reg[1]); /* hi */
            break;
        case 0x481u:
            candev_put16(p, 0, (uint16_t)g_in2.f.range_est);
            candev_put16(p, 2, (uint16_t)g_in2.f.range_reserve);
            candev_put16(p, 4, g_in2.f.in2_9a);
            candev_put16(p, 6, g_scr.power_half);
            break;
        case 0x482u:
            candev_put16(p, 0, g_in2.f.serial_mark);
            candev_put16(p, 2, g_in2.f.in2_5e);
            candev_put16(p, 4, 0);
            candev_put16(p, 6, 0);
            break;
        case 0x483u:
            candev_put16(p, 0, g_in2.f.serial_reg[0]); /* serial bytes 0..13 */
            candev_put16(p, 2, g_in2.f.serial_reg[1]);
            candev_put16(p, 4, g_in2.f.serial_reg[2]);
            candev_put16(p, 6, g_in2.f.serial_reg[3]);
            break;
        case 0x484u:
            candev_put16(p, 0, g_in2.f.serial_reg[4]);
            candev_put16(p, 2, g_in2.f.serial_reg[5]);
            candev_put16(p, 4, g_in2.f.serial_reg[6]);
            candev_put16(p, 6, 0);
            break;
        case 0x485u:
            candev_put16(p, 0, g_mode.word06);
            p[2] = g_mode.output_cfg; /* MODE+0x15 */
            p[3] = g_mode.cfg_16;
            candev_put16(p, 4, g_mode.word04);
            p[6] = g_mode.cmd_param;
            p[7] = g_mode.regen_level; /* staging +0x0E */
            break;
        case 0x7F0u:
            p[0] = 0x1E; /* drive-active frame (byte0 = 0x1E const) */
            p[1] = 0;
            p[2] = 0;
            p[3] = 0;
            p[4] = 0;
            p[5] = 0;
            p[6] = 0;
            p[7] = 0;
            break;
        default:
            /* unknown id: scratch left stale, pointer still returned */
            break;
    }
    return p;
}

/**
 * \brief           Install the candev payload getter (S+0x1C)
 * \note            The backing store is zero-init BSS, so the function pointer is written
 *                      here at init. The object ends at S+0x1F; the TP manager ops table
 *                      (busproto.c g_bp_ctx) is not part of candev.
 */
static void vcu_candev_init(void) { g_candev.payload_getter = vcu_candev_payload_get; }

/**
 * \brief           Fetch the 8-byte payload for a periodically-sent standard id (see can.h)
 * \note            Routes through the candev payload getter: it fills the
 *                      candev tx scratch for \p std_id, which is copied into \p out8.
 */
void vcu_can_payload_get(uint32_t std_id, uint8_t* out8) {
    volatile uint8_t* p = vcu_candev_payload_get(std_id);
    uint8_t i;

    for (i = 0; i < VCU_CAN_DLEN; ++i) {
        out8[i] = p[i];
    }
}

/**
 * \brief           Periodic CAN TX scheduler (see can.h)
 * \note            Control flow: suspend gate, event-gate reset, the two countdown timers,
 *                      the four cadence blocks, and the bus-off recovery that re-inits CAN
 *                      after the fault has persisted for > 50 ticks.
 */
void vcu_can_tick(void) {
    uint16_t tick;

    /* Suspend gate: while g_in1.drive_active is 1, zero the two comm-loss counters and
       the UV counter and bail out. */
    if (g_in1.drive_active == 1u) {
        g_lowglob.esc_commloss_cnt = 0;
        g_bms_commloss.cnt = 0;
        g_lowglob.fm_uv_cnt = 0;
        return;
    }

    /* A pending event holds off the tick until its countdown expires, then it resets
       the tick phase and clears the flag. */
    if ((g_candev.event_flags & CAN_EVENT_PENDING) != 0u) {
        if (g_candev.event_wait != 0) {
            g_candev.event_wait--;
            return;
        }
        g_can_sched.tick = 0;
        g_candev.event_flags &= (uint16_t)~CAN_EVENT_PENDING;
    }

    /* Two countdowns that raise output flags when they hit zero. */
    if (g_candev.timer_a == 0) {
        g_candev.out_bits |= CAN_OUT_TIMER_A_BIT;
    } else {
        g_candev.timer_a--;
    }
    if (g_candev.timer_b == 0) {
        g_candev.out_bits |= CAN_OUT_TIMER_B_BIT;
    } else {
        g_candev.timer_b--;
    }

    /* Advance the tick, then emit the due frames at their cadences. */
    g_can_sched.tick++;
    tick = g_can_sched.tick;

    if ((tick % 2u) == 0u) {
        can_send_periodic(CANID_DRIVE); /* 0x100, fastest */
    }
    if ((tick % 10u) == 0u) {
        can_send_periodic(CANID_ESC_A); /* 0x203 */
        can_send_periodic(CANID_ESC_B); /* 0x20C */
    }
    if ((tick % 20u) == 0u) {
        can_send_periodic(CANID_BMS_TEMP); /* 0x485 */
    }
    if ((tick % 100u) == 0u) {
        can_send_periodic(CANID_ESC_C);    /* 0x20A */
        can_send_periodic(CANID_BMS_VOLT); /* 0x482 */
        can_send_periodic(CANID_BMS_DATA); /* 0x301 */
    }

    /* Bus-off latch -> persistence streak; after a sustained streak, re-init the CAN
       controller (bump S+8, then vcu_can_mode_reset: CAN1 reset + GPIO + controller
       init). The software rings are kept, so queued TX/RX frames survive.
       vcu_can_mode_reset zeroes reinit_cnt (+0x08) again, so the bump is cancelled at
       once. */
    if (g_candev.bus_off == 0u) {
        g_can_sched.fault_ctr = 0;
    } else {
        g_can_sched.fault_ctr++;
        if (g_can_sched.fault_ctr > CAN_BUSOFF_REINIT_STREAK) {
            g_candev.bus_off = 0;
            g_can_sched.fault_ctr = 0;
            g_candev.reinit_cnt++;
            vcu_can_mode_reset();
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Interrupt handlers: RX0 / SE                                              */
/* Override the BSP weak CMSIS symbols.                                      */
/* ------------------------------------------------------------------------- */

/**
 * \brief           CAN1 RX FIFO0 interrupt (IRQ20)
 * \note            On FMP0 pending: read one frame out of FIFO0 into the 20-byte
 *                      record (with the BSP `can_message_receive`, which also releases the
 *                      FIFO slot) and push it onto the RX software ring. No per-id dispatch
 *                      happens in the ISR.
 */
void CAN1_RX0_IRQHandler(void) {
    if (can_receive_message_pending_get(CAN1, CAN_RX_FIFO0) != 0u) {
        can_rx_message_type rx;
        vcu_can_msg_t frame;
        uint8_t i;

        can_message_receive(CAN1, CAN_RX_FIFO0, &rx);

        frame.std_id = rx.standard_id;
        frame.ext_id = rx.extended_id;
        frame.ide = (uint8_t)rx.id_type;
        frame.rtr = (uint8_t)rx.frame_type;
        frame.dlc = rx.dlc;
        for (i = 0; i < VCU_CAN_DLEN; ++i) {
            frame.data[i] = rx.data[i];
        }
        frame.fmi = rx.filter_index;

        can_rx_ring_push(&frame);
    }
}

/**
 * \brief           CAN1 status-change / error interrupt (IRQ22)
 * \note            On bus-off (ESR bit2): latch `bus_off` in the RAM handle. On an
 *                      error record (ESR LEC bits 4-6 nonzero): snapshot TEC/REC/LEC.
 *                      Then clear the error + status flags: msts eoif (write 1) and, after an
 *                      error record, ests. These are the reads and writes of the BSP
 *                      can_flag_get / can_flag_clear / error-counter getters.
 */
void CAN1_SE_IRQHandler(void) {
    if (CAN1->ests_bit.bof != 0u) {
        g_candev.bus_off = 1; /* +0x0b */
    }

    if (CAN1->ests_bit.etr != 0u) {
        g_candev.tec = (uint8_t)CAN1->ests_bit.tec; /* +0x19 (ESR>>16) */
        g_candev.rec = (uint8_t)CAN1->ests_bit.rec; /* +0x1a (ESR>>24) */
        g_candev.lec = (uint8_t)CAN1->ests_bit.etr; /* +0x09 (ESR LEC, bits 4-6) */
        CAN1->msts = CAN_MSTS_EOIF_VAL;             /* clears EOIF ...           */
        CAN1->ests = 0u;                            /* ... + ESR                 */
    }

    /* clear the error-occur interrupt flag. */
    CAN1->msts = CAN_MSTS_EOIF_VAL;
}

/* ------------------------------------------------------------------------- */
/* Periodic drive-active sender and the super-loop RX consumer. The PA10     */
/* level read lives in gpio.c.                                               */
/* ------------------------------------------------------------------------- */
/* Loop periods. */
#define CAN_RETRIG_RELOAD 400 /* drive-active retrigger window (ticks)  */
#define CAN_SUBDIV_RELOAD 50  /* drive-active subdivider (ticks)        */

/* CAN tx-device vtable send. The device object is g_candev (this file). The vtable  */
/* call RETURNS A POINTER to the just-staged 8-byte payload buffer (it is handed      */
/* straight to vcu_can_send_std as the payload source), so the uint32_t return is     */
/* that buffer address.                                                               */
/* Routes through the candev payload getter stored at dev+0x1c (populated at init by
   vcu_candev_init -> vcu_candev_payload_get). The getter fills the 8-byte tx scratch
   at the candev base and returns a pointer to it, which vcu_can_senders forwards to
   vcu_can_send_std. */
static uint32_t can_candev_tx(uint32_t id) { /* (*(dev+0x1c))(id) */
    volatile uint8_t* (*get)(uint32_t can_id) = g_candev.payload_getter;

    if (get == NULL) {
        return 0;
    }
    return (uint32_t)(uintptr_t)get(id);
}

/* vcu_can_send_std(len, std_id, payload): arg3 is a PAYLOAD POINTER (copied into the */
/* frame). vcu_can_senders passes the candev payload.                                  */

/* Periodic CAN telemetry senders: a 400-tick retrigger                            */
/* (TMR->retrig_timer) gates a drive-active window; while active, a 0x32-tick      */
/* (TMR->subdiv_timer) subdivider drives the CAN tx device (CANDEV: event_wait     */
/* reload, event_flags bit0 request, vtable send of id 0x7F0) and forwards the     */
/* result via vcu_can_send_std, and toggles the g_scr+0x80 drive-active report.    */
/* Returns the active flag (IN1 drive-active). */
uint8_t vcu_can_senders(void) {
    volatile vcu_tmr_t* tmr = (volatile vcu_tmr_t*)((volatile uint8_t*)&g_tmr);

    if (g_in1.can_active_req == 0) {
        if ((int32_t)tmr->retrig_timer < 1) {
            g_in1.drive_active = 0;
            if (g_scr.can_active != 0) {
                report_can_toggle(0);
            }
        } else {
            tmr->retrig_timer = tmr->retrig_timer - 1;
        }
    } else {
        tmr->retrig_timer = CAN_RETRIG_RELOAD;
        g_in1.can_active_req = 0;
        g_in1.drive_active = 1;
    }
    if (g_in1.drive_active != 0) {
        g_vs.can_drive_active = 1;
        if ((int32_t)tmr->subdiv_timer < 1) {
            uint32_t ret;
            tmr->subdiv_timer = CAN_SUBDIV_RELOAD;
            g_candev.event_wait = CAN_SUBDIV_RELOAD;
            g_candev.event_flags = (uint16_t)(g_candev.event_flags | 1);
            ret = can_candev_tx(CANID_BROADCAST);
            /* arg3 is the staged payload buffer pointer; forward it as a */
            /* standard-id data frame through can.c's real sender. */
            vcu_can_send_std(VCU_CAN_DLEN, CANID_BROADCAST, (const uint8_t*)(uintptr_t)ret);
        } else {
            tmr->subdiv_timer = tmr->subdiv_timer - 1;
        }
        if (g_scr.can_active == 0) {
            report_can_toggle(1);
        }
    }
    return g_in1.drive_active;
}

/* ========================================================================= */
/* super-loop poll #2: CAN RX consumer / dispatcher                          */
/* ========================================================================= */
uint32_t vcu_can_rx_dispatch(void) {
    if (can_rx_count() != 0) {
        /* Pop one record into the scratch: copy entry[rd] and advance rd while the ring
           is non-empty. */
        if (g_rx_ring.rd != g_rx_ring.wr) {
            can_frame_copy((volatile vcu_can_msg_t*)&g_canmsg, &g_rx_ring.entry[g_rx_ring.rd]);
            g_rx_ring.rd = (g_rx_ring.rd + 1u) & VCU_CAN_RING_MASK;
        }
        if (g_canmsg.ide == 0) { /* 0 = standard id */
            can_rx_std(((volatile uint8_t*)&g_canmsg));
        } else {
            can_rx_ext(((volatile uint8_t*)&g_canmsg));
        }
        return (uint32_t)can_rx_count();
    }
    return 0;
}
