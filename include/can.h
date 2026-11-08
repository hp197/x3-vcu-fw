/**
 * \file            can.h
 * \brief           CAN1 subsystem internal API (motor/battery bus + Ninebot tunnel)
 * \note            Software-state init and TX pump (`vcu_can_init()` / `vcu_can_poll()`) plus
 *                      the CAN API the other modules use: the 20-byte message record, the rings
 *                      and the senders. Payload role names follow bxCAN and Ninebot bus
 *                      conventions.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef CAN_H
#define CAN_H

#include "at32f415.h"

/** \brief CAN software-state init (rings + candev vtable); called once from main(). */
void vcu_can_init(void);

/** \brief CAN TX pump, super-loop poll #1. */
void vcu_can_poll(void);

/* Software ring depth. Both the RX ring and the TX ring hold 32 slots and mask every
   index with 0x1F (so 31 are usable; one slot is always left empty to tell "full" from
   "empty"). */
#define VCU_CAN_RING_LEN     32u
#define VCU_CAN_RING_MASK    0x1Fu

/* CAN 2.0 payload limit: a classic data frame carries at most 8 bytes. */
#define VCU_CAN_DLEN         8u

/* 29-bit mask for an extended CAN identifier. */
#define VCU_CAN_EXT_ID_MASK  0x1FFFFFFFu

/* ------------------------------------------------------------------------- */
/* CAN message-ID map.                                                       */
/*                                                                           */
/* The 11-bit standard ids the VCU transmits (all DLC=8) from              */
/* `vcu_can_tick` / `vcu_can_senders`. The role in each name follows its   */
/* send cadence and Ninebot/Segway bus conventions.                        */
/* ------------------------------------------------------------------------- */
#define CANID_DRIVE          0x100U /* fastest (every 2 ticks): VCU -> motor drive cmd   */
#define CANID_ESC_A          0x203U /* every 10 ticks : ESC core status                  */
#define CANID_ESC_B          0x20CU /* every 10 ticks : ESC secondary status             */
#define CANID_ESC_C          0x20AU /* every 100 ticks: ESC / battery extended status    */
#define CANID_BMS_DATA       0x301U /* every 100 ticks: BMS / battery data block         */
#define CANID_BMS_VOLT       0x482U /* every 100 ticks: BMS cell-voltage block           */
#define CANID_BMS_TEMP       0x485U /* every 20 ticks : BMS temperature block            */
#define CANID_BROADCAST      0x7F0U /* event-gated    : broadcast / handshake frame      */

/* ------------------------------------------------------------------------- */
/* Inbound (RX) standard-id telemetry map.                                   */
/*                                                                           */
/* Each id is a `case` of the standard-frame RX decoder (can_rx_std),      */
/* which copies the ESC/BMS payload bytes into the status blackboard. The   */
/* 0x2xx band is the motor controller (ESC), 0x3xx/0x4xx is the battery     */
/* (BMS); the role labels follow from the fields each case writes. 0x7F1 is */
/* the ESC/BMS handshake that drives the periodic-sender event gate.        */
/* ------------------------------------------------------------------------- */
#define CANID_RX_AUX_42      0x042U /* two words, stored only                   */
#define CANID_RX_ESC_A       0x211U /* ESC: charge-detect bit + current limit   */
#define CANID_RX_ESC_B       0x212U /* ESC: bus voltage / current block         */
#define CANID_RX_BMS_A       0x310U /* BMS: pack status block                   */
#define CANID_RX_BMS_B       0x311U /* BMS: current (accel-clamped) block       */
#define CANID_RX_ESC_C       0x341U /* ESC: status word                         */
#define CANID_RX_ESC_KEEPALV 0x342U /* ESC: keep-alive (re-arms ESC watchdog)   */
#define CANID_RX_ESC_D       0x344U /* ESC: status word                         */
#define CANID_RX_ESC_E       0x401U /* ESC: fault / info block                  */
#define CANID_RX_BMS_SOC     0x420U /* BMS: state-of-charge / current block     */
#define CANID_RX_BMS_VERSION 0x421U /* BMS: ASCII-hex sw-version + id block     */
#define CANID_RX_BMS_C       0x422U /* BMS: cell/voltage block C                */
#define CANID_RX_BMS_D       0x423U /* BMS: cell/voltage block D                */
#define CANID_RX_BMS_E       0x424U /* BMS: status + report-enable block        */
#define CANID_RX_BMS_TEMP2   0x425U /* BMS: temperature (single + display)      */
#define CANID_RX_BAT2_SOC    0x430U /* battery 2: as 0x420                      */
#define CANID_RX_BAT2_VER    0x431U /* battery 2: as 0x421                      */
#define CANID_RX_BAT2_C      0x432U /* battery 2: as 0x422                      */
#define CANID_RX_BAT2_TEMP   0x435U /* battery 2: one temperature               */
#define CANID_RX_BMS_TEMP    0x500U /* BMS: 8 cell temperatures (-0x32 offset)  */
#define CANID_RX_BMS_KEEPAL0 0x501U /* BMS: keep-alive (re-arms BMS watchdog)   */
#define CANID_RX_BMS_KEEPAL1 0x502U /* BMS: keep-alive (re-arms BMS watchdog)   */
#define CANID_RX_HANDSHAKE   0x7F1U /* ESC/BMS handshake -> event gate          */
/* Custom diagnostic ids: the 0x6xx block is unused by the stock bus (drive/ESC/BMS in
   0x0xx-0x5xx, handshake at 0x7Fx), so these stay clear of a future stock id and are low
   priority -- they never disturb the real-time drive frames. Must match x3-mcu-fw's can.h. */
#define CANID_BACKUP_REQ     0x6E0U /* ESC read-only backup request (relay)     */
#define CANID_BACKUP_RSP     0x6E1U /* ESC read-only backup response (relay)    */

/* Temperature frames carry a raw byte biased by +0x32 (50) -- subtract to decode. */
#define CAN_RX_TEMP_BIAS     50U

/* J1939 PGN-format byte ((ext_id>>16)&0xFF) that selects the report/log path over */
/* the TP tunnel in the extended-id handler (0xEF, "Proprietary A").              */
#define CAN_EXT_PF_REPORT    0xEFU

/**
 * \brief           20-byte CAN message record
 * \note            One layout for both the RX FIFO read-out and the TX mailbox write:
 *                        +0x00 u32 std_id, +0x04 u32 ext_id, +0x08 ide, +0x09 rtr,
 *                        +0x0A dlc, +0x0B..0x12 data[8], +0x13 fmi -> 0x14 = 20 B.
 *                      Copied verbatim through the RX and TX rings, so the layout must stay exact.
 */
typedef struct {
    uint32_t std_id;            /*!< +0x00 11-bit standard id (used when ide==0)  */
    uint32_t ext_id;            /*!< +0x04 29-bit extended id (used when ide==1)  */
    uint8_t ide;                /*!< +0x08 0 = standard id, 1 = extended id       */
    uint8_t rtr;                /*!< +0x09 0 = data frame, 1 = remote frame       */
    uint8_t dlc;                /*!< +0x0A data length, 0..8                      */
    uint8_t data[VCU_CAN_DLEN]; /*!< +0x0B payload bytes                          */
    uint8_t fmi;                /*!< +0x13 filter-match index (RX only)           */
} vcu_can_msg_t;

/* The 20-byte record is copied verbatim through the rings, so the layout is load-bearing. */

/**
 * \brief           Software CAN frame ring (32 x 20-byte records)
 * \note            Both directions share the layout `{ u32 wr; u32 rd; record[32]; }`. 32
 *                      slots, every index masked with \ref VCU_CAN_RING_MASK so 31 are usable
 *                      (one slot stays empty to tell "full" from "empty"). Single-producer /
 *                      single-consumer. The RX ring is filled by the CAN1 RX0 ISR and drained
 *                      by the super loop, so the whole instance is `volatile` -- indices and
 *                      records both, or the compiler could move the record copy past the index
 *                      store that publishes it.
 */
typedef struct {
    uint32_t wr;                           /*!< +0x00 write index     */
    uint32_t rd;                           /*!< +0x04 read index      */
    vcu_can_msg_t entry[VCU_CAN_RING_LEN]; /*!< +0x08 20-byte records */
} vcu_can_ring_t;

/**
 * \brief           CAN1 bring-up + error/mode-field clear (no software-ring reset)
 * \note            Resets the CAN1 peripheral (base 0x40006400), re-inits the CAN GPIO and
 *                      hardware, then zeroes four fields in the shared CAN device block in
 *                      order: tec (+0x19), rec (+0x1a), lec (+0x09), reinit_cnt (+0x08). The
 *                      software rings are left alone, so queued frames survive. Callers are the
 *                      re-init paths: bus-off recovery in `vcu_can_tick`, the faultmon ESC
 *                      comm-loss path and vehicle power-down. Boot bring-up is
 *                      `vcu_can_lowlevel_init()`.
 */
void vcu_can_mode_reset(void);

/**
 * \brief           Pulse the CAN1 peripheral reset (CRM apb1rst can1rst set, then cleared)
 * \note            BSP can_reset(CAN1): every CAN1 register returns to its reset value. Used
 *                      by \ref vcu_can_mode_reset and the standby cycle.
 */
void vcu_can_periph_reset(void);

/**
 * \brief           CAN1 boot bring-up
 * \note            CAN GPIO (CAN1 remapped to PB9 TX / PB8 RX, PB15 output), controller
 *                      init (bit timing, filter, NVIC, interrupts), then PB15 driven low.
 *                      No can_reset and no error-field clear. Called once from
 *                      vcu_periph_init.
 */
void vcu_can_lowlevel_init(void);

/**
 * \brief           Enqueue one frame onto the software TX ring
 * \note            The frame is pumped into a hardware mailbox later by `vcu_can_poll()`;
 *                      app code never writes a mailbox directly.
 * \param[in]       msg: frame to send (20 bytes, copied into the ring)
 */
void vcu_can_tx_enqueue(const vcu_can_msg_t* msg);

/**
 * \brief           Send a payload as standard-id data frame(s)
 * \note            `len==0` emits one empty frame; otherwise the payload is split into
 *                      <=8-byte IDE=0 frames, each repeating the first bytes (the source offset
 *                      never advances).
 * \param[in]       len: total payload length in bytes
 * \param[in]       std_id: 11-bit standard id
 * \param[in]       payload: payload bytes (ignored when len==0)
 */
void vcu_can_send_std(uint16_t len, uint32_t std_id, const uint8_t* payload);

/**
 * \brief           Send a single extended-id data frame (<=8 bytes)
 * \note            Generic extended-id sender: IDE=1, one frame, dropped if len > 8.
 * \param[in]       ext_id: 29-bit extended id
 * \param[in]       payload: payload bytes
 * \param[in]       len: payload length, 0..8
 */
void vcu_can_send_ext(uint32_t ext_id, const uint8_t* payload, uint8_t len);

/* Single-frame Ninebot ext-id builder field constants. */
#define VCU_CAN_NB_PRIO_SHIFT  26u         /* J1939 priority field, id bits [28:26]          */
#define VCU_CAN_NB_PRIO_MAX    8u          /* priority must be < 8 (3-bit field)             */
#define VCU_CAN_NB_PF_EF       0x00EF0000u /* PF byte 0xEF ("Proprietary A") in id [23:16]   */
#define VCU_CAN_NB_ID_FLAG     0x20000000u /* bit29 marker set in the stored id word         */
#define VCU_CAN_NB_ID_KEEP     0xA0000000u /* bits kept from the prior id word (31, 29)      */
#define VCU_CAN_NB_HDR_LEN     3u          /* data[0..2] = ctl, grp, code                    */
#define VCU_CAN_NB_MAX_PAYLOAD 5u          /* single-frame payload limit (n < 6)             */

/**
 * \brief           29-bit extended-id frame staging record (13-byte layout)
 * \note            The id word, then the DLC byte, then the 8 data bytes -- the "out"
 *                      buffer the ext-id frame builders (Ninebot single-frame, J1939
 *                      TP.CM/TP.DT) fill and their callers hand to the ext-id sender.
 *                      busproto.h aliases it as bp_frame_t.
 */
typedef struct {
    uint32_t id;                /*!< +0x00 id word: 29-bit id + marker/flag bits  */
    uint8_t dlc;                /*!< +0x04 data length                           */
    uint8_t data[VCU_CAN_DLEN]; /*!< +0x05..+0x0C payload                        */
} vcu_can_ext_frame_t;

/**
 * \brief           Build one Ninebot single-frame ext-id report
 * \note            The one Ninebot ext-id builder. Rejects (returns 0, \p out untouched)
 *                      unless \p payload and \p out are set, \p n < 6 and \p prio < 8.
 *                      id = (prev & 0xA0000000) | ((prio<<26 | dst<<8 | src) & 0x1FFFFFFF)
 *                      | 0x00EF0000 (PF 0xEF) | 0x20000000 (bit29); dlc = n + 3;
 *                      data = { ctl, grp, code, payload[0..n-1] }.
 * \param[in]       prio: J1939 priority (the report caller passes 6)
 * \param[in]       ctl: data[0] control byte (the report caller passes 1)
 * \param[in]       src: source address, id bits [7:0]
 * \param[in]       dst: destination address, id bits [15:8]
 * \param[in]       cmd: data[1]
 * \param[in]       arg: data[2]
 * \param[in]       payload: \p n payload bytes -> data[3..]
 * \param[in]       n: payload length, 0..5
 * \param[in,out]   out: frame record (its prior id word's top bits are kept)
 * \return          1 when the frame was built, 0 when rejected
 */
int vcu_can_ninebot_frame(uint8_t prio, uint8_t ctl, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg,
                          const uint8_t* payload, uint8_t n, vcu_can_ext_frame_t* out);

/**
 * \brief           Periodic CAN TX scheduler (the fixed-cadence senders)
 * \note            Advances an internal tick and emits the due standard-id telemetry
 *                      frames at their fixed cadences, and re-inits CAN after a sustained
 *                      bus-off streak.
 *                      Registered on SLOT_CAN by main.c (its cadence is counted in
 *                      scheduler ticks, not in super-loop passes).
 */
void vcu_can_tick(void);

/**
 * \brief           Fetch the 8-byte payload for a periodically-sent standard id
 * \note            Used by `vcu_can_tick`: routes through the candev payload getter
 *                      (candev vtable +0x1C), which fills the frame from the live
 *                      blackboards (drive command, ESC/BMS mirrors).
 * \param[in]       std_id: the id whose payload is requested
 * \param[out]      out8: destination buffer, exactly 8 bytes
 */
void vcu_can_payload_get(uint32_t std_id, uint8_t* out8);

/**
 * \brief           Periodic drive-active CAN sender
 * \note            400-tick drive-active retrigger (TMR->retrig_timer) gating a 0x32-tick
 *                      subdivider that requests the candev payload for id 0x7F0 and sends
 *                      it as a standard-id frame, and toggles the g_scr+0x80 drive-active
 *                      report. Run from the vehicle FSM pre-pass.
 * \return          The IN1 drive-active flag
 */
uint8_t vcu_can_senders(void);

/**
 * \brief           CAN RX consumer / frame dispatcher (super-loop poll #2)
 * \note            If the CAN RX software ring is non-empty, pops one 20-byte message
 *                      record into the g_canmsg scratch and dispatches it by IDE:
 *                      standard-id frames to the ESC/BMS telemetry decoder, extended-id
 *                      (Ninebot tunnel) frames to the extended-id handler. Complements the
 *                      CAN TX pump (`vcu_can_poll`).
 * \return          Remaining RX ring occupancy after dispatch (0 when drained)
 */
uint32_t vcu_can_rx_dispatch(void);

/**
 * \brief           Battery-2 presence and SoC-low tick
 * \note            Called by control_power_monitor_fsm. While battery 2
 *                      is present (\ref vcu_scr_t.bat2_present) it copies the SoC to
 *                      g_scr.bat2_soc, sets g_bat2.soc_low at <= 5 % and raises SF_DUAL_PACK
 *                      (beep 3 in the ride state). Once SF_DUAL_PACK is up it counts
 *                      g_bat2.commloss_cnt; at 200 ticks without a battery-2 frame the pack
 *                      is dropped, unless its SoC is low, where only state 4 drops it.
 */
void vcu_can_battery2_presence_tick(void);

/**
 * \brief           The two words of the last CAN 0x42 frame
 * \note            Written by can_rx_std: [0] = data[0..3], [1] = data[4..7], little-endian.
 *                      No module reads them.
 */
extern volatile uint32_t g_can_rx42[2];

#endif /* CAN_H */
