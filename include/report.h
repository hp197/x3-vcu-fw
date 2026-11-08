/**
 * \file            report.h
 * \brief           Ninebot report / telemetry layer (x3 VCU) -- public API
 * \note            The report frame writers every unit reaches (event-report sink, u16 /
 *                      status / toggle / on-change reports), the telemetry/info-node frame
 *                      builder, the power-off countdown and long-idle date-time reports and
 *                      the slow telemetry task (scheduler slot 4).
 *                      Frames are enqueued on the USART2 TX ring through usart.c
 *                      (vcu_usart_report_send).
 *
 *                      Role names (e.g. "throttle", "LED", "report") follow Ninebot/AT32
 *                      semantics.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef REPORT_H
#define REPORT_H

#include "at32f415.h"

/* ------------------------------------------------------------------------- */
/* Telemetry / history frame (u16).                                         */
/* ------------------------------------------------------------------------- */

/** \name Telemetry-frame halfword indices reached outside the builder. */
/**@{*/
#define REPORT_TELEM_CAN212_W1  3u  /*!< +0x06 CAN 0x212 d[2..3] mirror (can.c)             */
#define REPORT_TELEM_CAN212_W3  4u  /*!< +0x08 CAN 0x212 d[6..7] mirror (can.c)             */
#define REPORT_TELEM_CAN401_D5  5u  /*!< +0x0A CAN 0x401 d[5..6] mirror (can.c)             */
#define REPORT_TELEM_CAN401_W0  6u  /*!< +0x0C CAN 0x401 d[0..1] mirror (can.c)             */
#define REPORT_TELEM_CAN401_W1  7u  /*!< +0x0E CAN 0x401 d[2..3] mirror (can.c)             */
#define REPORT_TELEM_CANDEV_100 8u  /*!< +0x10 candev id 0x100 payload mirror (can.c)      */
#define REPORT_TELEM_CANDEV_301 9u  /*!< +0x12 candev id 0x301 payload mirror (can.c)      */
#define REPORT_TELEM_CAN341_W0  10u /*!< +0x14 CAN 0x341 d[0..1] mirror (can.c)             */
#define REPORT_TELEM_CAN425     11u /*!< +0x16 CAN 0x425 d[2] | d[5] << 8 (can.c)            */
#define REPORT_TELEM_GEAR_EVENT 12u /*!< +0x18 gear-event counter (ridemode.c)            */
#define REPORT_TELEM_CFG_BASE   16u /*!< +0x20 magic + info-node config block (op 0x13, ninebot.c) */
/**@}*/

/**
 * \brief           Telemetry-frame length in halfwords
 * \note            Covers every access: the history init zeroes up to index CFG_BASE + 199,
 *                      and the info-node op 0x13 read takes `CFG_BASE + arg` for any u8 `arg`,
 *                      so the frame spans CFG_BASE + 256 halfwords (544 B) and every such read
 *                      stays inside this array.
 */
#define REPORT_TELEM_FRAME_LEN  (REPORT_TELEM_CFG_BASE + 256u)

/**
 * \brief           Telemetry / info-node frame, owned by report.c
 */
extern volatile uint16_t g_telem_frame[REPORT_TELEM_FRAME_LEN];

/**
 * \name            CAN 0x100 frame counter in the history block
 * \note            A u32 at history +0x1C as two halfwords: the candev 0x100 payload getter
 *                      (can.c) counts it up to 0xFFFFFFFF, vcu_config_init zeroes it, the
 *                      history copy reads it.
 */
/**@{*/
#define REPORT_TELEM_TX100_LO 14u /*!< +0x1C low half  */
#define REPORT_TELEM_TX100_HI 15u /*!< +0x1E high half */
/**@}*/

/* ------------------------------------------------------------------------- */
/* Report protocol constants + enums (frame bytes / report command + group     */
/* ids). The shared SRAM overlays live in vcu_state.h; these are owned by the   */
/* report layer and shared with the vehicle FSM and ninebot.c.                  */
/* ------------------------------------------------------------------------- */

/* The Ninebot frame header vocabulary (src / dst / cmd / arg, NB_ADDR_VCU,
   NB_ADDR_APP, NB_CMD_WRITE3) is shared in usart.h. Telemetry reports go
   NB_ADDR_VCU -> NB_ADDR_APP with cmd NB_CMD_WRITE3 and one of the arg bytes below. */

/** \name Report destinations / lengths. */
/**@{*/
#define VCU_RPT_DST_U16       0x04u /*!< dst of the one-u16 field reports (report_u16)  */
#define VCU_RPT_EVT_LEN       2u    /*!< event-report payload length (report_ninebot)   */
/**@}*/

/** \name Report arg bytes (register index the value is written to). */
/**@{*/
#define VCU_RPT_ARG_MILEAGE   0x24u /*!< odometer / mileage */
#define VCU_RPT_ARG_MODE      0x25u /*!< drive/indicator mode */
#define VCU_RPT_ARG_STATUS    0x3Du /*!< packed status word */
#define VCU_RPT_ARG_CANACT    0x40u /*!< CAN drive-active toggle */
#define VCU_RPT_ARG_SUBSYS    0x60u /*!< subsystem-enable word */
#define VCU_RPT_U16_MODE      0x51u /*!< u16 report: light/mode state */
#define VCU_RPT_U16_LOCK      0x61u /*!< u16 report: lock event */
#define VCU_RPT_EVT_DST       0x3Du /*!< event-report dst */
#define VCU_RPT_EVT_CMD       0x56u /*!< event-report cmd */
#define VCU_RPT_ARG_COUNTDOWN 0xD4u /*!< power-off countdown remaining */
#define VCU_RPT_ARG_IDLE      0xD1u /*!< long-idle / park report */
/**@}*/

/**
 * \brief           Ninebot event-report codes (the 'arg' byte of the input-test
 *                      event-report frames emitted by the FSM event path)
 * \note            The mapping from each pending event flag (g_evt) to its code is the
 *                      one the dispatcher uses. The three *_ENDSTOP codes come from the
 *                      throttle / brake-lever travel debounce (partial travel, then full
 *                      travel).
 */
typedef enum {
    VCU_EVT_LOCK = 1,                /*!< g_evt.mon[FM_MON_POWER_BTN].code (secondary lock) */
    VCU_EVT_02 = 2,                  /*!< g_evt.mon[FM_MON_PARK].code */
    VCU_EVT_03 = 3,                  /*!< g_evt.mon[FM_MON_GEAR_UP].code */
    VCU_EVT_04 = 4,                  /*!< g_evt.mon[FM_MON_GEAR_DOWN].code */
    VCU_EVT_05 = 5,                  /*!< g_evt.mon[FM_MON_LIGHT_EN].code */
    VCU_EVT_06 = 6,                  /*!< g_evt.mon[FM_MON_INTERLOCK].code */
    VCU_EVT_LOCK_LATCH = 7,          /*!< g_evt.mon[FM_MON_SUBSTATE_PREV].code (+ anti-theft latch) */
    VCU_EVT_08 = 8,                  /*!< g_evt.mon[FM_MON_SUBSTATE_NEXT].code */
    VCU_EVT_09 = 9,                  /*!< g_evt.mon[FM_MON_LIGHT_LO].code */
    VCU_EVT_10 = 10,                 /*!< g_evt.mon[FM_MON_LIGHT_HI].code */
    VCU_EVT_THROTTLE_ENDSTOP = 11,   /*!< throttle (PA0) reached full travel */
    VCU_EVT_BRAKE_HALF_ENDSTOP = 12, /*!< half-weight brake lever (PA1) full travel */
    VCU_EVT_BRAKE_FULL_ENDSTOP = 13  /*!< full-weight brake lever (PA4) full travel */
} vcu_report_event_t;

/**
 * \brief           Ninebot event-report sink
 * \note            \p n < 6: one 29-bit ext-id frame (vcu_can_ninebot_frame, id
 *                      0x18EF<dst><src>, data {1, cmd, arg, payload}); else the J1939 TP
 *                      tunnel (bp_tp_send).
 * \param[in]       n: payload byte count
 */
void report_ninebot(uint8_t src, uint8_t dst, uint8_t n, uint8_t cmd, uint8_t arg, const void* payload);

/** \brief Stash a u16 at g_scr[idx*2] and report it as cmd 4, group idx. */
void report_u16(uint8_t idx, uint16_t val);

/** \brief Latch the CAN drive-active halfword (g_scr+0x80) and report it. */
void report_can_toggle(int16_t val);

/**
 * \brief           On-demand packed status report
 * \note            When reporting is enabled (g_vs+0x26), packs the 30-bit status word at
 *                      \p src into four bitfields at g_scr+0x7C and reports it (dst 0x23,
 *                      arg 0x3D). Driven on FSM state transitions.
 * \param[in]       src: the packed status word (callers pass &g_stat.status_packed)
 */
void report_status(const volatile uint32_t* src);

/** \brief Periodic DTC/status report site: runs fm_status_report(). */
void report_fault_status(int32_t a);

/** \brief Mileage report-on-change writer, arg 0x24. */
void report_onchange_24(uint16_t val);

/** \name Subsystem enable mask bits (g_scr.subsys_enable, MODE enable_flags bits 0..1). */
/**@{*/
#define VCU_SUBSYS_B 0x1u /*!< subsystem B (PB0, control_subsys_b_enable)  */
#define VCU_SUBSYS_A 0x2u /*!< subsystem A (PB10, control_subsys_a_enable) */
/**@}*/

/**
 * \brief           Publish a subsystem A/B enable mask and report it on change
 * \note            The shared tail of the four subsystem mask writers (the callers drive
 *                      the PB10 / PB0 pins first): sets
 *                      MODE enable_flags bits 0..1 to \p mask, latches g_scr.subsys_enable,
 *                      enqueues it (arg VCU_RPT_ARG_SUBSYS) when it differs from
 *                      g_chgbuf.subsys_reported, then updates that cache.
 * \param[in]       mask: VCU_SUBSYS_A / VCU_SUBSYS_B bits (0..3)
 */
void report_subsys_mask_set(uint16_t mask);

/** \brief Subsystem A+B enable + subsystem-enable report on change. */
void report_subsys_enable(void);

/** \brief Pack a=[27:22] (6b), b=[21:16] (6b), c=[15:8], d=[7:0] into one id word. */
uint32_t report_pack_bitfields(uint32_t a, uint32_t b, uint32_t c, uint32_t d);

/**
 * \brief           Factory-UID telemetry copy + checksum
 * \note            Copies the 3-word UID into g_mcu_uid and its IN2 mirror and stores the
 *                      ~(w0+w1+w2) checksum. Called from vcu_config_init.
 */
void report_uid_telemetry_copy(void);

/**
 * \brief           Telemetry / info-node frame builder
 * \note            Every 11th tick assembles the ~40-field telemetry/info-node frame
 *                      \ref g_telem_frame from the SRAM blackboards (MODE/STAT/SCR/IN2/A3C/CNT),
 *                      with value scaling and a lazy one-time block init (magic 0x515C).
 *                      Base cell IN2+0xAE.
 */
void report_telemetry_build(void);

/**
 * \brief           Power-off countdown + periodic report
 * \note            300->0 countdown (@g_tmr+0x22): every 100 ticks beep(2) and
 *                      report the remaining/100 (dst 0x23, arg 0xD4), emitting a
 *                      Ninebot event when the displayed value reaches 2; clear the active
 *                      gate at 0. Gated on IN1+1.
 */
void report_countdown(void);

/**
 * \brief           Long-idle date-time check
 * \note            When drive-enable (IN2+0x38 bit11), the datetime-check enable
 *                      (IN2+0x62 bit3) and reporting (VS+0x26) are all set: computes the
 *                      packed date-time delta (stored IN2+0x1D2 vs RTC-sampled STAT+0x1FE)
 *                      and, past 89 days, latches a flag + idle report (arg 0xD1);
 *                      re-arms and restores the stored datetime on the reset request.
 */
void report_datetime(void);

/**
 * \brief           Slow telemetry/report task
 * \note            Scheduler slot 4, period 300000 ticks (the slowest). When enabled
 *                      (VS+0x26) and the FSM state is 3 or 7: if the one-shot flag (set by
 *                      report_status) is set it is cleared and nothing is sent; else the 32-bit
 *                      packed word (STAT+0x1FC) is unpacked into four bitfields and sent (src
 *                      0x16, dst 0x23, arg 0x3D) via vcu_usart_report_send. Other states:
 *                      nothing.
 */
void report_task_slow(void);

#endif /* REPORT_H */
