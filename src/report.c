/**
 * \file            report.c
 * \brief           Ninebot report / telemetry layer: report frames, telemetry builder, slow task
 * \note            The Ninebot event-report sink (CAN ext-id frame or J1939 TP tunnel), the
 *                      small u16/status/toggle/on-change report writers, the periodic
 *                      telemetry/info-node frame builder, the power-off countdown and
 *                      long-idle date-time reports, the slow telemetry task (scheduler slot 4)
 *                      and the TMR5 base-timer tick ISR. Every frame goes out through the
 *                      USART2 TX ring (usart.c vcu_usart_report_send).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "report.h"

#include "busproto.h" /* bp_tp_send + g_bp_mgr (J1939 TP tunnel for long reports) */
#include "buzzer.h"
#include "can.h"
#include "control.h" /* control_mode_param (ride-mode table column 6) */
#include "faultmon.h"
#include "scheduler.h" /* g_sched_tick (TMR5 tick, owned here next to its ISR) */
#include "usart.h"
#include "vcu_state.h"
#include "vehicle.h" /* SF_SESSION */

/* ------------------------------------------------------------------------- */
/* Telemetry / history frame, an allocated global (declared in report.h).     */
/* Writers: the history update below, the CAN RX decoder + candev mirrors and   */
/* the 0x100 frame counter (can.c), the gear-event counter (ridemode.c);        */
/* reader: the info-node op 0x13 config read (ninebot.c).                       */
/* ------------------------------------------------------------------------- */
volatile uint16_t g_telem_frame[REPORT_TELEM_FRAME_LEN];
#define TELEM_FRAME (g_telem_frame)
/* 11-call history divider. */
static volatile uint8_t g_report_hist_tick;
#define TELEM_TICK          (g_report_hist_tick) /* 0..10 then fire                  */
#define TELEM_MAGIC         0x515Cu              /* cfg-block lazy-init sentinel     */
#define TELEM_INIT_SPAN     200                  /* cfg-block lazy-zero word count   */
#define TELEM_TICK_FIRE     10u                  /* copy when the count read is > 10 */
#define PWROFF_COUNT_RELOAD 300                  /* report_countdown reload          */

/* Telemetry builder scaling. */
#define TELEM_MPH_NUM       621    /* 0.1 km/h -> mph: x 621 / 10000 (x 0.0621)          */
#define TELEM_MPH_DEN       10000
#define TELEM_PEDAL_FULL    200u   /* pedal byte full scale                               */
#define TELEM_PERMILLE      1000u  /* per-mille scale                                     */
#define TELEM_SPEED_PCT     100    /* speed scale factor                                  */
#define TELEM_SPEED_COL     6u     /* ride-mode table column read at row * 0x12 + 0xC     */

/* Node 0x02 register 0xD0 parameter push (tail of the builder). */
#define NODE2_ADDR          0x02u /* destination node                                 */
#define NODE2_CMD           0x02u /* cmd byte                                         */
#define NODE2_REG           0xD0u /* arg byte: the register                           */
#define NODE2_SEND_EVERY    10u   /* send when the divider read is >= 10              */
#define NODE2_MAX_TRIES     10u   /* the send that reads 10 earlier tries ends it     */
#define NODE2_TABLE_LEN     18u   /* bytes sent                                       */

/**
 * \brief           Node 0x02 register 0xD0 table (18 bytes)
 * \note            Sent as one 18-byte report. The bytes are parameters for the node
 *                      (likely motor-controller parameters).
 */
static const uint8_t k_node2_reg_d0[NODE2_TABLE_LEN] = {0x00u, 0x06u, 0x00u, 0x0Au, 0x00u, 0x14u, 0x00u, 0x05u, 0x66u,
                                                        0x02u, 0x33u, 0x00u, 0x00u, 0x01u, 0x00u, 0x01u, 0x01u, 0x00u};

/* IN2->status_flags (+0x38) bits only the report path tests (the shared SF_* /
   CFG_* register bits are in vcu_state.h). */
#define SF_TELEM_HOLD      0x0002u /*!< bit1 set unless FSM in RIDE_B/ANTITHEFT */
#define SF_IDLE_EXPIRED    0x0008u /*!< bit3 long-idle/park expired         */

/* AT32F415 unique-device-ID words (0x1FFFF7E8, the standard AT32/STM32       */
/* factory UID base).                                                           */
#define VCU_UID            0x1FFFF7E8u
/* Factory UID words are read straight from flash (0x1FFFF7E8). */
#define MMIO32(addr)       (*(volatile uint32_t*)(addr))

/* Ninebot event-report sink constants. */
#define NB_RPT_PRIO_SINGLE 6u /* J1939 priority of the single-frame report        */
#define NB_RPT_PRIO_TP     7u /* priority/arg2 handed to the TP session builder   */
#define NB_RPT_CTL         1u /* data[0] / TP header[0] control byte              */
#define NB_RPT_TP_SCALE    0  /* deadline scale handed to bp_build_packet         */

/* Ninebot event-report sink. A payload of up to 5 bytes goes out as ONE 29-bit ext-id
   frame: vcu_can_ninebot_frame builds id 0x18EF<dst><src> + {ctl, cmd, arg, payload}
   and vcu_can_send_ext sends it with the id masked to 29 bits. Six bytes or more go
   through the J1939 TP tunnel: bp_tp_send allocates the (src,dst) session on the
   fast-service manager and builds its buffer; the slot-3 fast service
   (bp_task_fast_service) then streams it. A frame the builder rejects (e.g. NULL
   payload) is not sent. */
void report_ninebot(uint8_t src, uint8_t dst, uint8_t n, uint8_t cmd, uint8_t arg, const void* payload) {
    if (n <= VCU_CAN_NB_MAX_PAYLOAD) {
        vcu_can_ext_frame_t frame = {0};

        if (vcu_can_ninebot_frame(NB_RPT_PRIO_SINGLE, NB_RPT_CTL, src, dst, cmd, arg, (const uint8_t*)payload, n,
                                  &frame)
            != 0) {
            vcu_can_send_ext(frame.id & VCU_CAN_EXT_ID_MASK, frame.data, frame.dlc);
        }
    } else {
        (void)bp_tp_send(&g_bp_mgr, NB_RPT_PRIO_TP, NB_RPT_CTL, src, dst, cmd, arg, (const uint8_t*)payload, n,
                         NB_RPT_TP_SCALE);
    }
}

/* Periodic DTC/status report site, called from 4 task-level sites in the per-state */
/* FSM handlers (states 7/3/2/1 in vehicle.c): runs faultmon's fm_status_report.     */
/* The retry trigger (vcu_faultmon_retry_trigger) runs separately at its own sites   */
/* (vehicle.c), so it is NOT folded in here. Callers pass the FSM state in 'a';      */
/* fm_status_report reads it from the blackboard, so 'a' is unused. */
void report_fault_status(int32_t a) {
    (void)a;
    fm_status_report();
}

/* ------------------------------------------------------------------------- */
/* Report / telemetry frame layer.                                           */
/* ------------------------------------------------------------------------- */

/* Small halfword report: stash a u16 at g_scr[idx*2] and send it as cmd 4,   */
/* group `idx`. */
void report_u16(uint8_t idx, uint16_t val) {
    g_scr.reg[idx] = val; /* page-1 register `idx` */
    vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, VCU_RPT_DST_U16, 2, NB_CMD_WRITE3, idx,
                          &g_scr.reg[idx]);
}

/* CAN drive-active report toggle: latch a halfword at g_scr+0x80 and emit it  */
/* as dst 0x23, arg 0x40. */
void report_can_toggle(int16_t val) {
    g_scr.can_active = val;
    vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3,
                          VCU_RPT_ARG_CANACT, &g_scr.can_active);
}

/* On-demand status report: when reporting is enabled                          */
/* (g_vs+0x26), set the one-shot flag and pack the 30-bit status word at `*src`  */
/* into four bitfields ([21:17],[16:12],[11:6],[5:0]) laid byte-per-field at      */
/* g_scr+0x7C, then send it (dst 0x23, arg 0x3D). Same packing as the slow     */
/* report, driven on state transitions. */
void report_status(const volatile uint32_t* src) {
    uint32_t v;

    if (g_vs.report_enable != 0) {
        g_oneshot.report_flag = 1;
        v = *src;
        g_scr.status_pack = ((v >> 0x11) & 0x1F) | (((v >> 0x0C) & 0x1F) << 8) | (((v >> 0x06) & 0x3F) << 16)
                            | ((v & 0x3F) << 24);
        vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 6, NB_CMD_WRITE3,
                              VCU_RPT_ARG_STATUS, &g_scr.reg[VCU_RPT_ARG_STATUS]); /* regs 0x3D..0x3F */
    }
}

/* Mileage report-on-change cache, a report-owned variable (the down-counter next to
   it in ridemode's sub-mode block lives in odometer.c). */
static volatile uint16_t g_mileage_report_cache; /*!< last mileage value reported (arg 0x24) */

/* Mileage report-on-change writer: if the cached value differs from `val`, latch */
/* it at g_scr+0x48, update the cache and emit a Ninebot frame (dst 0x23,          */
/* arg 0x24). */
void report_onchange_24(uint16_t val) {
    if (g_mileage_report_cache != val) {
        g_scr.mileage_u = val; /* report payload */
        g_mileage_report_cache = g_scr.mileage_u;
        vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3,
                              VCU_RPT_ARG_MILEAGE, &g_scr.mileage_u);
    }
}

/* Subsystem mask publish + report-on-change: the common tail of the four mask     */
/* writers (mask 0, mask B, mask A, mask A|B). MODE->enable_flags low 2 bits =     */
/* mask, g_scr.subsys_enable = mask, emitted (arg 0x60) when it differs from the   */
/* cache. */
void report_subsys_mask_set(uint16_t mask) {
    g_mode.enable_flags = (uint16_t)((g_mode.enable_flags & 0xFFFCu) + mask);
    g_scr.subsys_enable = mask;
    if (g_scr.subsys_enable != g_chgbuf.subsys_reported) {
        vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3,
                              VCU_RPT_ARG_SUBSYS, &g_scr.subsys_enable);
    }
    g_chgbuf.subsys_reported = g_scr.subsys_enable;
}

/* Subsystem-enable + report: enables subsystems A and B via gpio, then          */
/* publishes mask A|B. */
void report_subsys_enable(void) {
    control_subsys_a_enable(1);
    control_subsys_b_enable(1);
    report_subsys_mask_set(VCU_SUBSYS_A | VCU_SUBSYS_B);
}

/* Scheduler time base (declared in scheduler.h): the free-running TMR5 overflow
   count. Its only users are this ISR (32-bit RMW) and the scheduler dispatch, which
   it paces. */
volatile uint32_t g_sched_tick;

/**
 * \brief           TMR5 overflow ISR: count the 1.0 ms scheduler ticks
 * \note            The TMR5_GLOBAL_IRQHandler vector (IRQ50); it has no direct caller.
 *                      TMR5 and its overflow IRQ are brought up by config.c. Increments the
 *                      scheduler tick \ref g_sched_tick, the time base of every
 *                      software-timer slot. The tick is 1.0 ms (TMR5 period set up by
 *                      config.c). Every count in report.c is a count of calls, not of time,
 *                      so nothing here depends on the tick length.
 */
void TMR5_GLOBAL_IRQHandler(void) {
    if (tmr_flag_get(TMR5, TMR_OVF_FLAG) != RESET) {
        tmr_flag_clear(TMR5, TMR_OVF_FLAG);
        g_sched_tick = g_sched_tick + 1;
    }
}

/* UID telemetry buffer copy + checksum: copies the 3-word factory UID           */
/* (0x1FFFF7E8) into the telemetry buffer g_mcu_uid, mirrors it to IN2+0x1B4 and */
/* writes a checksum = ~(w0+w1+w2) at g_state_block+0x0C. Its only caller is     */
/* vcu_config_init.                                                              */
void report_uid_telemetry_copy(void) {
    uint32_t w0 = MMIO32(VCU_UID + 0u); /* 0x1FFFF7E8                          */
    uint32_t w1 = MMIO32(VCU_UID + 4u);
    uint32_t w2 = MMIO32(VCU_UID + 8u);

    g_mcu_uid[0] = w0;
    g_mcu_uid[1] = w1;
    g_mcu_uid[2] = w2;
    g_in2.ident_word[0] = w0; /* mirror in the IN2 identity region (IN2+0x1B4) */
    g_in2.ident_word[1] = w1;
    g_in2.ident_word[2] = w2;
    g_state_block.uid_cksum = w0;
    g_state_block.uid_cksum = w1 + g_state_block.uid_cksum;
    g_state_block.uid_cksum = w2 + g_state_block.uid_cksum;
    g_state_block.uid_cksum = ~g_state_block.uid_cksum;
}

/* Bit-field packer: packs a=6-bit [27:22], b=6-bit [21:16], c=8-bit [15:8],     */
/* d=8-bit [7:0] into one 32-bit word. Its caller is ninebot.c's nb_report_frame */
/* (the chunked CAN report ext id).                                             */
uint32_t report_pack_bitfields(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    return d | (c << 8) | ((b & 0x3Fu) << 0x10) | ((a & 0x3Fu) << 0x16);
}

/* ========================================================================= */
/* slot 4 (period 300000): slow telemetry report                             */
/* Nothing unless reporting is enabled (VS+0x26) and the FSM is in state 3   */
/* or 7. There, a pending one-shot flag (set by report_status when it        */
/* reported the transition) is cleared INSTEAD of sending, so the slow       */
/* report skips one period after an on-demand report; with the flag clear    */
/* the status word is packed and sent.                                       */
/* ========================================================================= */
void report_task_slow(void) {
    uint8_t fields[4];
    uint32_t packed;

    if (g_vs.report_enable == 0) {
        return; /* report subsystem disabled */
    }
    if (g_state_block.fsm_state != VCU_FSM_RIDE && g_state_block.fsm_state != VCU_FSM_CHARGING) {
        return; /* not a reportable state */
    }
    if (g_oneshot.report_flag != 0) {
        g_oneshot.report_flag = 0; /* an on-demand report just went out: skip this one */
    } else {
        /* Unpack the 32-bit status word (STAT+0x1FC) into 4 bitfields. */
        packed = g_stat.status_packed;
        fields[0] = (uint8_t)((packed >> 0x11) & 0x1f); /* bits [21:17] */
        fields[1] = (uint8_t)((packed >> 0x0c) & 0x1f); /* bits [16:12] */
        fields[2] = (uint8_t)((packed >> 0x06) & 0x3f); /* bits [11:6]  */
        fields[3] = (uint8_t)(packed & 0x3f);           /* bits [5:0]   */

        /* The four fields are stored contiguously then re-read as one word (@+0x7c). */
        g_scr.status_pack = (uint32_t)fields[0] | ((uint32_t)fields[1] << 8) | ((uint32_t)fields[2] << 16)
                            | ((uint32_t)fields[3] << 24);
        vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 6, NB_CMD_WRITE3,
                              VCU_RPT_ARG_STATUS, &g_scr.reg[VCU_RPT_ARG_STATUS]); /* regs 0x3D..0x3F */
    }
}

/* ========================================================================= */
/* Telemetry / periodic-report task bodies.                                  */
/*                                                                           */
/* The telemetry frame builder, the power-off countdown report and the       */
/* long-idle date-time report (the mode/enable flag reset of the same group  */
/* lives in vehicle.c).                                                      */
/* ========================================================================= */

/**
 * \brief           History block update: lazy init, then the copy on every 12th call
 * \note            `frame[0x10] != 0x515C` -> frame[0x0C] = 0, magic, frame[0x11 .. 0xD7] = 0.
 *                      Then the divider is post-incremented; when it read > 10 it restarts and
 *                      the blackboards are copied into frame[0x11..0x3A], including the 0x100
 *                      frame counter, the range reserve and four battery-2 cells.
 */
static void prv_history_update(void) {
    volatile uint16_t* frame = TELEM_FRAME;
    uint8_t tick;

    if (frame[REPORT_TELEM_CFG_BASE] != TELEM_MAGIC) {
        uint16_t i;

        frame[REPORT_TELEM_GEAR_EVENT] = 0;
        frame[REPORT_TELEM_CFG_BASE] = TELEM_MAGIC;
        for (i = 1; i < TELEM_INIT_SPAN; i = (uint16_t)(i + 1)) {
            frame[REPORT_TELEM_CFG_BASE + i] = 0;
        }
    }

    tick = (uint8_t)TELEM_TICK;
    TELEM_TICK = (uint8_t)(tick + 1);
    if (tick > TELEM_TICK_FIRE) {
        TELEM_TICK = 0;
        frame[0x11] = (uint16_t)(g_mode.curr_word & 0xFFu);
        frame[0x12] = (uint16_t)(g_mode.we_word & 0xFFu);
        frame[0x13] = (uint16_t)g_stat.curr_limit;
        frame[0x14] = g_stat.canrx_310b;
        frame[0x15] = g_stat.canrx_311c;
        frame[0x16] = g_stat.canrx_311b;
        frame[0x17] = (uint16_t)g_stat.curr_sample;
        frame[0x18] = (uint16_t)g_stat.volt_sample;
        frame[0x19] = frame[0];
        frame[0x1A] = frame[1];
        frame[0x1B] = frame[2];
        frame[0x1C] = frame[REPORT_TELEM_CAN212_W1];
        frame[0x1D] = (uint16_t)(g_fault_codes.vbus_code & 0xFFu);
        frame[0x1E] = frame[REPORT_TELEM_CAN212_W3];
        frame[0x1F] = frame[REPORT_TELEM_CAN401_D5];
        frame[0x20] = frame[REPORT_TELEM_CAN425];
        frame[0x21] = frame[REPORT_TELEM_CAN401_W0];
        frame[0x22] = frame[REPORT_TELEM_CAN401_W1];
        frame[0x23] = g_in2.f.speed_src;
        frame[0x24] = frame[REPORT_TELEM_CANDEV_100];
        frame[0x25] = frame[REPORT_TELEM_CANDEV_301];
        frame[0x26] = frame[REPORT_TELEM_CAN341_W0];
        frame[0x27] = g_scr.dtc_mask0;
        frame[0x28] = g_scr.dtc_mask1;
        frame[0x29] = g_scr.dtc_mask2;
        frame[0x2A] = g_scr.dtc_mask2;
        frame[0x2B] = g_scr.event_mask_lo;
        frame[0x2C] = g_scr.event_mask_hi;
        frame[0x2D] = g_in2.f.serial_mark;
        frame[0x2E] = g_in2.f.canrx_212;
        frame[0x2F] = g_in2.f.canrx_421;
        frame[0x30] = g_in2.f.bat2_version;
        frame[0x31] = g_in2.canrx_1e8_scratch;
        frame[0x32] = frame[REPORT_TELEM_GEAR_EVENT];
        frame[0x33] = frame[REPORT_TELEM_TX100_LO]; /* the u32 counter, low then high */
        frame[0x34] = frame[REPORT_TELEM_TX100_HI];
        frame[0x35] = g_stat.soc_u;
        frame[0x36] = (uint16_t)g_in2.f.range_reserve;
        frame[0x37] = g_stat2.soc_u;
        frame[0x38] = (uint16_t)g_stat2.curr_sample;
        frame[0x39] = (uint16_t)g_stat2.volt_sample;
        frame[0x3A] = g_stat2.temp_array[0];
    }
}

/**
 * \brief           Node 0x02 register 0xD0 parameter push, one step per telemetry call
 * \note            The builder tail-calls it. Not pending: both counters are cleared. Pending: the divider is
 *                      post-incremented and, when it read >= 10, restarts and the table goes out
 *                      as report_ninebot(0x16, 0x02, 18, cmd 2, arg 0xD0) (the TP tunnel); the try
 *                      count is then post-incremented and, when it read >= 10, the push ends
 *                      (state 2). So a push sends every 11th call, 11 times at most, until the
 *                      node's cmd 5 arg 0xD0 reply (ninebot.c) ends it first.
 */
static void prv_node2_push_step(void) {
    uint8_t count;

    if (g_node2_push.state != NODE2_PUSH_PENDING) {
        g_node2_push.tries = 0u;
        g_node2_push.send_div = 0u;
    } else {
        count = g_node2_push.send_div;
        g_node2_push.send_div = (uint8_t)(count + 1u);
        if (count >= NODE2_SEND_EVERY) {
            g_node2_push.send_div = 0u;
            report_ninebot(NB_ADDR_VCU, NODE2_ADDR, NODE2_TABLE_LEN, NODE2_CMD, NODE2_REG, k_node2_reg_d0);
            count = g_node2_push.tries;
            g_node2_push.tries = (uint8_t)(count + 1u);
            if (count >= NODE2_MAX_TRIES) {
                g_node2_push.state = NODE2_PUSH_DONE;
            }
        }
    }
}

/* Telemetry builder: the speed caches, the telemetry-hold bit, the gauge cells     */
/* IN2+0x1E0..0x1E4, then the history update and the node 0x02 push step. The      */
/* displayed speed is km/h (VS+2) or mph = speed x 0.0621.                          */
void report_telemetry_build(void) {
    int16_t speed = g_in2.f.speed_src_s;

    /* speed/10 cache (signed) + displayed speed (scale selected by status bit3). */
    g_vs.speed_kmh = (uint8_t)((int32_t)speed / 10);
    if ((g_in2.f.cfg_flags & CFG_IMPERIAL) != 0) {
        g_scr.telem_70 = 1;
        g_scr.gest_busy = (int16_t)(((int32_t)speed * TELEM_MPH_NUM) / TELEM_MPH_DEN); /* truncates to 0 */
    } else {
        g_scr.telem_70 = 0;
        g_scr.gest_busy = (int16_t)(uint16_t)g_vs.speed_kmh; /* ldrb VS+2 */
    }
    g_scr.telem_3c = (uint16_t)(uint8_t)g_vs.gear_sel;
    if (g_in1.throttle_flag == 0) {
        g_in2.f.gear = (uint16_t)(uint8_t)g_vs.gear_sel; /* == IN2->gear cell */
    }
    /* Telemetry-hold bit (status_flags bit1): cleared while reportable (RIDE_B /    */
    /* ANTITHEFT), set otherwise. */
    if (g_state_block.fsm_state == VCU_FSM_RIDE || g_state_block.fsm_state == VCU_FSM_CHARGING) {
        g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & ~SF_TELEM_HOLD);
    } else {
        g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_TELEM_HOLD);
    }

    /* Gauge cells: pedal bytes in per mille, speed in per mille of the
       ride mode's column 6. The speed division is unsigned on the signed product (udiv), and the
       clamp compares signed. */
    {
        uint16_t pedals = g_mode.drive_output_word;
        uint32_t den = (uint32_t)(int32_t)(int16_t)control_mode_param(g_vs.region, TELEM_SPEED_COL);

        g_in2.throttle_pm = (uint16_t)(((uint32_t)(pedals & 0xFFu) * TELEM_PERMILLE) / TELEM_PEDAL_FULL);
        g_in2.brake_pm = (uint16_t)(((uint32_t)(pedals >> 8) * TELEM_PERMILLE) / TELEM_PEDAL_FULL);
        g_in2.speed_pm = (uint16_t)((uint32_t)((int32_t)speed * TELEM_SPEED_PCT) / den);
        if ((int16_t)g_in2.speed_pm > (int16_t)TELEM_PERMILLE) {
            g_in2.speed_pm = TELEM_PERMILLE;
        }
    }

    prv_history_update();
    prv_node2_push_step();
}

/* Power-off countdown + periodic report:                                         */
/* a 300->0 countdown (@g_tmr+0x22) gated on the IN1+1 active flag. Every 100    */
/* ticks it beeps(2) and reports the remaining/100 (dst 0x23, arg 0xD4); when the  */
/* displayed value reaches 2 it emits a Ninebot event frame. At 0 it clears the  */
/* gate without a beep.                                                           */
void report_countdown(void) {
    volatile vcu_tmr_t* tmr = (volatile vcu_tmr_t*)((volatile uint8_t*)&g_tmr); /* power-off pacing block */
    uint16_t remaining;

    if (g_in1.remote_countdown_req == 0) { /* countdown-active gate */
        tmr->poweroff_countdown = PWROFF_COUNT_RELOAD;
        return;
    }
    remaining = tmr->poweroff_countdown;
    if (remaining != 0) {
        if (remaining % 100u == 0u) {
            buzzer_beep(BEEP_SHORT);
            g_scr.countdown_report = (uint16_t)(tmr->poweroff_countdown / 100u);
            vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3,
                                  VCU_RPT_ARG_COUNTDOWN, &g_scr.countdown_report);
            if ((int16_t)g_scr.countdown_report == 2) {
                uint32_t zero = 0; /* 4-byte zero payload */
                report_ninebot(0x3E, 7, 1, 0x7A, 0, &zero);
            }
        }
        tmr->poweroff_countdown = (uint16_t)(tmr->poweroff_countdown - 1);
        return;
    }
    g_in1.remote_countdown_req = 0;
}

/* Long-idle date-time check: when drive is enabled                                */
/* (IN2->status_flags bit11), the datetime-check enable (IN2+0x62 bit3) and         */
/* reporting (VS+0x26) are all set, compute the packed date-time delta between the  */
/* stored value (IN2+0x1D2) and the RTC-sampled value (STAT+0x1FE). The packing is   */
/* year[15:10] / month[9:6] / day[5:1]; the delta is year*365 + month*30 + day/2.    */
/* Past 89 (days), latch the one-shot and send the idle report (arg 0xD1). On the  */
/* reset request (VS+0x2D) re-arm: clear the flag and restore the stored datetime.    */
void report_datetime(void) {
    if ((g_in2.f.status_flags & SF_SESSION) != 0 && (g_in2.f.datetime_enable & 0x0008u) != 0
        && g_vs.report_enable != 0) {
        uint16_t cur = g_stat.status_hi;       /* RTC-sampled packed date-time */
        uint16_t prev = g_in2.datetime_stored; /* stored packed date-time   */
        int32_t delta = (int32_t)((((uint32_t)(cur >> 10) - (uint32_t)(prev >> 10)) & 0xFFFFu) * 0x16Du
                                  + (uint32_t)(((cur & 0x3FFu) >> 6) * 0x1Eu) + (uint32_t)((cur & 0x3Fu) >> 1)
                                  - (uint32_t)(((prev & 0x3Fu) >> 1) + ((prev & 0x3FFu) >> 6) * 0x1Eu));

        if (g_vs.fault_ack == 0 && delta > 0x59) { /* VS+0x25 one-shot latch */
            g_vs.fault_ack = 1;
            g_scr.idle_report = 1;
            g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_IDLE_EXPIRED);
            vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3,
                                  VCU_RPT_ARG_IDLE, &g_scr.idle_report);
        }
        if (g_vs.reboot_req != 0) { /* datetime-reset request */
            g_vs.reboot_req = 0;
            g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & ~SF_IDLE_EXPIRED);
            g_in2.datetime_stored = g_stat.status_hi;
            g_in1.recompute_req = 1; /* datetime-updated mark */
        }
    }
}
