/**
 * \file            ninebot.c
 * \brief           Shared Ninebot command dispatcher for the x3 VCU
 * \note            The single command processor that BOTH the USART2/BLE 5A A5 parser and
 *                      the CAN1 29-bit extended-id tunnel feed. See ninebot.h for the module
 *                      map.
 *
 *                      This unit owns the pairing key (vcu_ninebot_ctx_t). The register file
 *                      is the shared IN2 block (g_in2.reg[], register n = IN2 + 2n). Side
 *                      effects on the live drive/ESC/BMS state (owned by other modules) all go
 *                      through the \ref nb_vcu_event seam, keeping command routing separate
 *                      from the state it changes.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */

#include "ninebot.h"
#include "at32f415.h" /* Artery AT32F415 BSP / CMSIS (transitive peripheral + core types) */
#include "vcu_state.h"

#include "backup.h"   /* vcu_backup_app_request (ESC backup relay) */
#include "busproto.h" /* bp_handle_fc / bp_node_register */
#include "buzzer.h"
#include "can.h"
#include "config.h"  /* identity_store, config_identity, config_serial_commit, config_delay_ms */
#include "control.h" /* control_mode_sync, vcu_ride_mode_t (gear_sel values) */
#include "ota.h"
#include "report.h"
#include "ridemode.h" /* ridemode_led_matrix_update */
#include "usart.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/* RAM context. The register file is IN2 (g_in2.reg[], register n = IN2 + 2n);  */
/* the 16-byte pairing key lives in g_nb.                                       */
/* ------------------------------------------------------------------------- */

/** \brief Serial family prefix for this model (model.h), checked during serial provisioning */
static const char k_nb_serial_prefix[] = VCU_MODEL_SERIAL_PREFIX;

#define NB_SERIAL_PREFIX_LEN 3u /*!< family prefix length of a serial */

/** \brief Factory serial template "0000000001" (10 bytes) */
static const uint8_t NB_SERIAL_TEMPLATE[NB_SERIAL_LEN] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '1'};

/** \brief The Ninebot context (pairing key) */
static vcu_ninebot_ctx_t g_nb;

/* Flash auth-key cell (16 B; declared in ninebot.h): erased, 16 x 0xFF. Linked in the image
   header at 0x08001420 (config.h APP_HEADER_SECTION). */
const uint8_t NB_FLASH_AUTH_KEY[NB_KEY_LEN] APP_HEADER_SECTION("key") = {
    0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
};

/* Boot copy of the 16-byte auth key from the flash cell into the Ninebot context. */
void nb_key_load(void) { memcpy(g_nb.key, NB_FLASH_AUTH_KEY, NB_KEY_LEN); }

/* The Ninebot registers a cmd 2/3 write may change. Only the writable registers are listed,
   read-only in flash (26 bytes instead of a 256-entry table), to fit the 60 KB slot; every
   other register is read-only. Nothing above register 0xFF is writable, so a range that runs
   past register 0xFF is rejected. Only a nonsensical request gets there. */
static const uint8_t k_nb_writable_regs[] = {
    0x1CU, 0x1DU, 0x1EU, 0x1FU, 0x2EU, 0x2FU, 0x42U, 0x47U, 0x48U, 0x49U, 0x4AU, 0x4BU, 0x4CU,
    0x4FU, 0x5BU, 0x5CU, 0x5DU, 0x61U, 0x70U, 0x71U, 0x72U, 0x74U, 0x75U, 0xE0U, 0xE1U, 0xE9U,
};

/**
 * \brief           Whether a cmd 2/3 write may change Ninebot register \p reg
 * \param[in]       reg: register index; nothing above 0xFF is writable
 * \return          `1` if the register is writable, else `0`
 */
static uint8_t nb_reg_writable(uint32_t reg) {
    uint32_t i;
    uint8_t writable = 0U;

    for (i = 0U; i < sizeof(k_nb_writable_regs); i = i + 1U) {
        if ((uint32_t)k_nb_writable_regs[i] == reg) {
            writable = 1U;
        }
    }
    return writable;
}

/* op-0x7B app-auth retry counter. */
static volatile uint8_t g_nb_app_auth_ctr;

/* Reply framing constants. */
#define NB_FLAG_ACK         0x00U /* unsolicited / ack                 */
#define NB_FLAG_REPLY       0x02U /* solicited reply                   */
#define NB_CMD_REG_REPLY    0x04U /* reply cmd: register-block read   */
#define NB_CMD_STATUS_REPLY 0x05U /* reply cmd: write status (register write, app start) */
#define NB_STATUS_REPLY_LEN 2U    /* status reply body: 16-bit LE status code          */
#define NB_APP_ANNOUNCE_LEN 14U   /* app-mode 0x0B announce body length                */

/* Gateway USART dst==7 status forward: gate on arg f[4] == 0x10 and cmd f[3] == 0x2B;
   report_status packs the 32 bits at f + 8. */
#define NB_FWD_STATUS_ARG   0x10U
#define NB_FWD_STATUS_CMD   0x2BU
#define NB_FWD_STATUS_OFF   8U

/* ------------------------------------------------------------------------- */
/* Primitives.                                                                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           CRC-32: reflected IEEE, poly 0xEDB88320.
 */
uint32_t nb_crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;

    for (i = 0U; i < len; i = i + 1U) {
        uint32_t bit;

        crc = crc ^ data[i];
        for (bit = 0U; bit < 8U; bit = bit + 1U) {
            if ((crc & 1U) == 0U) {
                crc = crc >> 1;
            } else {
                crc = NB_CRC32_POLY ^ (crc >> 1);
            }
        }
    }
    return ~crc;
}

/**
 * \brief           Byte compare, memcmp-like
 * \note            The loop computes a[i] - b[i] and continues while it is 0. So the result
 *                      is the difference of the first mismatching bytes, or 0 when all `len`
 *                      bytes match (also for len 0). Example: cmd 0x18 with a serial tail that
 *                      starts with 'G' gives status 0x17 = 'G' - '0'.
 */
int nb_keycmp(const uint8_t* a, const uint8_t* b, uint32_t len) {
    uint32_t i;
    int diff = 0;

    for (i = 0U; (i < len) && (diff == 0); i = i + 1U) {
        diff = (int)a[i] - (int)b[i];
    }
    return diff;
}

/* ------------------------------------------------------------------------- */
/* Reply senders: vcu_usart_report_send (USART2 ring) / report_ninebot (CAN    */
/* ext-id). Local replies go back on the request's bus (nb_reply); frames for   */
/* other nodes are forwarded to the other bus (nb_forward). Every sender takes  */
/* the header as (dst, src, flag, cmd, arg, data, len); \p flag has no slot in  */
/* either sink ABI (kept so every call site has the same shape).               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Send a reply over the CAN 29-bit extended-id tunnel
 * \note            Emits through the one Ninebot ext-id sink, report_ninebot: a single
 *                      0x18EF<dst><src> frame for <= 5 payload bytes, else the J1939 TP tunnel.
 */
static void nb_reply_can(uint8_t dst, uint8_t src, uint8_t flag, uint8_t cmd, uint8_t arg, const uint8_t* data,
                         uint8_t data_len) {
    (void)flag;
    report_ninebot(src, dst, data_len, cmd, arg, data);
}

/**
 * \brief           Send a reply over USART2 (vcu_usart_report_send into the g_uartctx ring)
 * \note            Always enqueues; the super-loop TX sequencer drains
 *                      the ring to DMA1 ch7. A direct DMA arm could collide with a ring
 *                      burst already in flight. Same field order as \ref nb_reply_can.
 */
static void nb_reply_usart(uint8_t dst, uint8_t src, uint8_t flag, uint8_t cmd, uint8_t arg, const uint8_t* data,
                           uint8_t data_len) {
    (void)flag;
    vcu_usart_report_send((volatile uint8_t*)&g_uartctx, src, dst, data_len, cmd, arg, data);
}

/**
 * \brief           Local reply (frame for the VCU): back on the bus the request came from
 * \note            `transport == 1 ? report_ninebot (CAN) : vcu_usart_report_send (USART
 *                      ring)`.
 */
static void nb_reply(int transport, uint8_t dst, uint8_t src, uint8_t flag, uint8_t cmd, uint8_t arg,
                     const uint8_t* data, uint8_t data_len) {
    if (transport == (int)VCU_TRANSPORT_CAN) {
        nb_reply_can(dst, src, flag, cmd, arg, data, data_len);
    } else {
        nb_reply_usart(dst, src, flag, cmd, arg, data, data_len);
    }
}

/**
 * \brief           Send a solicited reply from the VCU to a node (public reply wrapper)
 * \param[in]       transport: the bus to reply on (\ref vcu_transport_t)
 * \param[in]       dst: destination node address
 * \param[in]       cmd: reply command code
 * \param[in]       data: reply body bytes (may be NULL when \p len is 0)
 * \param[in]       len: body length in bytes
 */
void nb_reply_pub(int transport, uint8_t dst, uint8_t cmd, const uint8_t* data, uint8_t len) {
    nb_reply(transport, dst, NB_ADDR_VCU, NB_FLAG_REPLY, cmd, 0U, data, len);
}

/**
 * \brief           Gateway forward (frame for another node): out on the other bus
 * \note            `transport == 1 ? vcu_usart_report_send (USART ring) : report_ninebot
 *                      (CAN)`.
 */
static void nb_forward(int transport, uint8_t dst, uint8_t src, uint8_t flag, uint8_t cmd, uint8_t arg,
                       const uint8_t* data, uint8_t data_len) {
    if (transport == (int)VCU_TRANSPORT_CAN) {
        nb_reply_usart(dst, src, flag, cmd, arg, data, data_len);
    } else {
        nb_reply_can(dst, src, flag, cmd, arg, data, data_len);
    }
}

/**
 * \brief           Gateway re-send of a frame for another node, verbatim
 * \note            Every forward passes
 *                      sink(src=f[1], dst=f[2], len=f[0], cmd=f[3], arg=f[4], data=&f[5]);
 *                      only the bus changes (\ref nb_forward picks the other one).
 */
static void nb_forward_frame(const uint8_t* f, int transport) {
    nb_forward(transport, f[NB_OFF_DST], f[NB_OFF_SRC], NB_FLAG_ACK, f[NB_OFF_CMD], f[NB_OFF_ARG], &f[NB_OFF_BODY],
               f[NB_OFF_LEN]);
}

/* ------------------------------------------------------------------------- */
/* VCU state blackboard accessors.                                            */
/*                                                                            */
/* The dispatcher reaches the live drive/ESC state through the per-block      */
/* globals of vcu_state.h (VS, IN2, IN1, SCR, STATE); the field offsets are   */
/* load-bearing. Every cell is reached through its one shared vcu_state.h name */
/* (g_vs / g_in1 / g_in2 / g_scr), so a write here is the write every other     */
/* module (vehicle/config/faultmon/...) reads.                                  */
/* ------------------------------------------------------------------------- */

/* Pairing/VIN salt (IN2+0x1BC): identity-record word 2. */
#define NB_IN2_SALT  (g_in2.ident_word[2])

/* Lone blackboard cells reached by a single named base (not part of a block). */
#define APP_AUTH_CTR (g_nb_app_auth_ctr) /* app-auth retry counter (cmd 0x7B); lone ninebot-owned global */

/* The info-node config read block is the tail of report.c's telemetry frame
   g_telem_frame, from REPORT_TELEM_CFG_BASE on. */
#define NB_CFGBLK    ((volatile uint8_t*)&g_telem_frame[REPORT_TELEM_CFG_BASE])

/* ------------------------------------------------------------------------- */
/* Helpers the dispatcher calls.                                              */
/* ------------------------------------------------------------------------- */
/* Commit drive/indicator mode: latch the mode halfword at g_scr+0x4a (read back   */
/* by the FSM state-8 handler), report it (dst 0x23, arg 0x25) via the shared       */
/* report framer, and on mode 0 (disable) clear the registered report-node record   */
/* g_bp_node. Called by the vehicle FSM (vehicle.c) and the config path (busproto.c). */
void nb_commit_mode(int16_t mode) {
    g_scr.ui_screen_u = (uint16_t)mode;
    vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, 2, NB_CMD_WRITE3, VCU_RPT_ARG_MODE,
                          (const volatile void*)&g_scr.ui_screen_u);
    if (g_scr.ui_screen_u == 0) {
        memset((void*)&g_bp_node, 0, sizeof(g_bp_node));
    }
}

/* Forward decl: little-endian u32 frame reader (defined below). */
static uint32_t nb_rd_u32(const uint8_t* p);

/* ------------------------------------------------------------------------- */
/* Fixed field offsets used by the helpers below.                             */
/* The three "VCU-compat" config words live at IN2+0x1B4/0x1B8/0x1BC, which is  */
/* the SAME backing store as config's identity record (g_rec.vcu_id, record     */
/* +0x1B4); ninebot and config share g_in2, so these are the same cells. */

/**
 * \brief           Validate a 2-word config token
 * \return          1 iff body[0]==~(c0+c1+c2) AND body[1]==~(c0*c1*c2) over the
 *                      three VCU-compat config words at IN2+0x1B4/0x1B8/0x1BC
 * \note            Done in uint32_t: the sum and product wrap by design, which would be
 *                      undefined behaviour in int32_t.
 */
static uint8_t nb_validate_token(const uint8_t* body) {
    const uint32_t c0 = g_in2.ident_word[0]; /* IN2+0x1B4 compat word 0 (ident[0..3]) */
    const uint32_t c1 = g_in2.ident_word[1]; /* IN2+0x1B8 compat word 1 (ident[4..7]) */
    const uint32_t c2 = NB_IN2_SALT;         /* IN2+0x1BC compat word 2 (ident[8..11] == salt) */

    if ((nb_rd_u32(body) == ~(c0 + c1 + c2)) && (nb_rd_u32(&body[4]) == ~(c0 * c1 * c2))) {
        return 1U;
    }
    return 0U;
}

/**
 * \brief           Commit the live identity record to flash
 * \note            Calls config.c's dual-bank identity store: the ping-pong writer
 *                      (checksum, erase, program, bank select by version parity) -- NOT a
 *                      derived-state refresh. config_identity() returns the global record
 *                      (&g_rec), which identity_store() updates (version + checksum).
 */
static void nb_state_refresh(void) { (void)identity_store((identity_rec_t*)config_identity()); }

/**
 * \brief           Reinitialise the drive-mode parameter block
 * \note            Calls control.c's control_drive_param_init (public via
 *                      control.h): seeds the g_in2 drive-parameter + per-mode profile
 *                      table (bitfield runs at IN2+0x3A/0x3C/0x3E, the 0x82-group report,
 *                      and the mode-branch accel/decel curves).
 */
static void nb_drive_recompute_b(void) { control_drive_param_init(); }

/* Chunked report constants. */
#define NB_RPT_CHUNK   8U /* payload bytes per report frame (CAN DLC limit)            */
#define NB_RPT_IDE_EXT 1U /* descriptor +0x08: extended-id frame                       */

/**
 * \brief           Emit a chunked Ninebot report into the CAN TX ring
 * \note            Builds one vcu_can_msg_t per frame (ext_id = report_pack_bitfields(src,
 *                      dst, cmd, arg'), ide = 1, dlc = chunk length) and enqueues it on the CAN
 *                      TX ring (vcu_can_tx_enqueue, drops when full). n == 0 sends one
 *                      header-only frame; otherwise the payload is split into 8-byte chunks,
 *                      with arg' = arg + off/2 (the register index steps one 16-bit word per
 *                      2 bytes).
 * \param[in]       n: payload byte count
 * \param[in]       src: id bits [27:22] (6 bits)
 * \param[in]       dst: id bits [21:16] (6 bits)
 * \param[in]       cmd: id bits [15:8]
 * \param[in]       arg: id bits [7:0], base register index
 * \param[in]       payload: \p n payload bytes
 * \return          \p n (0 for the header-only frame; the ring enqueue reports no status here)
 */
uint32_t nb_report_frame(uint8_t n, uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg, const uint8_t* payload) {
    vcu_can_msg_t d;
    uint16_t off;
    uint8_t chunk;

    memset(&d, 0, sizeof(d));
    d.ide = NB_RPT_IDE_EXT;
    if (n == 0U) {
        d.ext_id = report_pack_bitfields(src, dst, cmd, arg);
        d.dlc = 0U;
        vcu_can_tx_enqueue(&d);
        return 0U;
    }
    for (off = 0U; off < n; off = (uint16_t)(off + chunk)) {
        chunk = (uint8_t)(((uint16_t)(n - off) > NB_RPT_CHUNK) ? NB_RPT_CHUNK : (n - off));
        d.ext_id = report_pack_bitfields(src, dst, cmd, (uint32_t)arg + ((uint32_t)off >> 1));
        d.dlc = chunk;
        memcpy(d.data, &payload[off], chunk);
        vcu_can_tx_enqueue(&d);
    }
    return n;
}

/**
 * \brief           Validate + commit the stored serial
 * \note            Calls config.c's config_serial_commit (the serial/template
 *                      validator; config domain). It walks the stored serial at
 *                      IN2+0x20 (== g_rec.serial, the shared blackboard) against the
 *                      factory template "03SP0000000001" and latches the commit flag.
 *                      The dispatcher ignores the return code.
 */
static void nb_pair_commit(void) { (void)config_serial_commit(); }

/* APP-node (src 0x23) OTA/registration -> busproto.c node handlers.                     */
/* ADAPTER: builds a transient CAN-style bp_frame_t from the decoded Ninebot 5A A5 frame  */
/* and calls the busproto owners (bp_handle_fc / bp_node_register, public in busproto.h). */
/* da = 0x02, and the FC handler gates on frame->data[0] == 0xFC. The source address (sa) */
/* and bp_frame_t.id are taken from the Ninebot frame's src byte.                          */
static void nb_app_register_prepare(const uint8_t* frame) {
    bp_frame_t bf;
    uint8_t i;

    memset(&bf, 0, sizeof(bf));
    bf.id = frame[NB_OFF_SRC];
    bf.dlc = 8U;
    for (i = 0U; i < 8U; i = (uint8_t)(i + 1U)) {
        bf.data[i] = frame[NB_OFF_BODY + i];
    }
    bf.data[0] = NB_APP_CMD_FC; /* FC gate byte checked by bp_handle_fc (frame->data[0] == 0xFC) */
    bp_handle_fc(frame[NB_OFF_SRC], 0x02U, &bf);
}

static void nb_app_register_commit(const uint8_t* frame) {
    /* The registration takes f[0] (body length) and the body pointer &f[5];
       bp_node_register resolves the node from exactly those two. */
    bp_node_register(&frame[NB_OFF_BODY], frame[NB_OFF_LEN]);
}

/** \brief Previous register value + index latched by \ref nb_reg_write. */
static uint16_t g_nb_reg_prev;
static uint8_t g_nb_reg_idx;

/**
 * \brief           Read a little-endian u32 straight out of a frame offset
 */
static uint32_t nb_rd_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
 * \brief           Register-write side effects (info node, cmd 2/3 register-write tail:
 *                      regs 0x1D/0x1E/0x1F/0x4B/0x4C/0x75)
 * \note            Compares the just-written register's changed bits to the
 *                      previous value. Reg 0x1F b4/b5 are the scheduled-charging enable and
 *                      single-shot bits: a change requests the BMS charge-control push
 *                      (IN1+0x28); enabling clears the schedule stop (VS+0x19), disabling
 *                      raises the schedule-cancel notice (VS+0x1F). A write to the window
 *                      start/end (0x4B/0x4C) also pushes; 0x4B disarms the pending schedule.
 *                      Settings changes re-map the stored gear.
 * \param[in]       reg: register index written (frame arg)
 * \param[in]       old: previous 16-bit register value
 * \param[in]       cur: new 16-bit register value
 */
static void nb_reg_side_effects(uint8_t reg, uint16_t old, uint16_t cur) {
    switch (reg) {
        case NB_REG_CHG_CFG: /* charge settings: scheduled-charging enable / single-shot */
            if ((cur & CHG_SCHED_EN) == (old & CHG_SCHED_EN)) {
                if ((cur & CHG_SCHED_ONCE) != (old & CHG_SCHED_ONCE)) {
                    g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND; /* single-shot bit changed */
                }
            } else {
                g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND; /* schedule enable changed */
                if ((cur & CHG_SCHED_EN) != 0U) {
                    g_vs.chg_sched_stop = 0U;
                } else {
                    g_vs.chg_sched_cancel_notice = 1U;
                }
            }
            break;

        case NB_REG_CFG_FLAGS: /* settings: walk-bit change moves gear 1 -> 2 */
            if (((cur & CFG_WALK_EN) != (old & CFG_WALK_EN)) && (g_vs.gear_sel == VCU_MODE_WALK)) {
                g_vs.gear_sel = VCU_MODE_ECO;
            }
            break;

        case NB_REG_CFG_FLAGS2: /* settings 2: gear-enable change re-maps gears 3/4 */
            if (((cur & CFG2_GEAR4_EN) != (old & CFG2_GEAR4_EN)) || ((cur & CFG2_GEAR3_EN) != (old & CFG2_GEAR3_EN))) {
                if (g_vs.gear_sel == VCU_MODE_SPORT) {
                    g_vs.gear_sel = (uint8_t)(((cur & CFG2_GEAR4_EN) != 0U) ? VCU_MODE_POWER : VCU_MODE_ECO);
                } else if (g_vs.gear_sel == VCU_MODE_POWER) {
                    g_vs.gear_sel = VCU_MODE_ECO;
                }
            }
            break;

        case NB_REG_CHG_START: /* window start: push to the BMS, disarm the pending schedule */
            g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND;
            g_in1.chg_sched_armed = 0U;
            break;

        case NB_REG_CHG_END: /* window end: push to the BMS */ g_in1.bms_chg_cmd_req = BMS_CHG_CMD_SEND; break;

        case NB_REG_RIDE_ASSIST: /* 0x75: body[0]==0 clears reg 0x1E bit1, else sets it */
            if ((cur & 0x00FFU) == 0U) {
                g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 & (uint16_t)~CFG2_RIDE_ASSIST);
            } else {
                g_in2.f.cfg_flags2 = (uint16_t)(g_in2.f.cfg_flags2 | CFG2_RIDE_ASSIST);
            }
            break;

        default: break;
    }
    g_in1.recompute_req = 1U; /* raise the recompute flag */
}

/**
 * \brief           VCU control-state side-effect layer: the per-sub-command writes the
 *                      dispatcher makes into the live state block
 * \note            `event` carries the sub-command: `0x0100 | cmd` for the control/info
 *                      nodes (routed by the dispatcher), or a small housekeeping id for the
 *                      auth/register paths.
 * \param[in]       event: 0x0100|cmd for a control/info sub-command, else a housekeeping id
 * \param[in]       frame: the originating frame (starting at the Ninebot `len` byte)
 */
void nb_vcu_event(uint16_t event, const uint8_t* frame) {
    if (frame == 0) {
        return;
    }

    if ((event & NB_EVT_CTRL_CMD) != 0U) {
        uint8_t cmd = (uint8_t)(event & 0x00FFU);
        uint8_t arg = frame[NB_OFF_ARG];
        uint8_t body0 = frame[NB_OFF_BODY];

        switch (cmd) {
            /* ----------------------------- control node (src 0x04) ---------------- */
            case NB_CTRL_LOCK_STATE: /* 0x7A: remote lock value (edge -> beep) */
                if (g_vs.remote_lock_state != arg) {
                    g_vs.remote_lock_edge = 0U;
                    g_vs.remote_lock_state = arg;
                    if (arg != 0U) {
                        g_vs.remote_lock_edge = 1U; /* 0 -> non-zero edge */
                        buzzer_beep(BEEP_DOUBLE);
                    }
                }
                break;

            case NB_CTRL_REMOTE_LOCK: /* 0x7B: remote lock / unlock (arg 1 unlock, 4 lock, 0/2/3 clear) */
                if (arg == 0x07U) {
                    if (body0 == 1U) {
                        if (APP_AUTH_CTR < 3U) {
                            APP_AUTH_CTR = (uint8_t)(APP_AUTH_CTR + 1U);
                            buzzer_beep(BEEP_DOUBLE);
                        }
                    } else if ((body0 == 2U) || (body0 == 3U)) {
                        APP_AUTH_CTR = 0U;
                    }
                }
                if ((g_in2.f.cfg_flags & CFG_LOCK_EN) != 0U) {
                    switch (arg) {
                        case 0x00U:
                            g_vs.remote_lock_req = 0U;
                            g_vs.remote_unlocked = 0U;
                            break;
                        case 0x01U:
                            if (g_vs.remote_unlocked == 0U) {
                                g_vs.chirp_req = 1U;
                                g_vs.remote_lock_req = 0U;
                                g_vs.remote_unlocked = 1U;
                                if (g_state_block.fsm_state == VCU_FSM_IDLE) {
                                    g_vs.flash_seq = FLASH_SEQ_START;
                                }
                            }
                            break;
                        case 0x02U:
                        case 0x03U: g_vs.remote_unlocked = 0U; break;
                        case 0x04U:
                            g_vs.remote_lock_req = 1U;
                            g_vs.remote_unlocked = 0U;
                            break;
                        default: break;
                    }
                } else {
                    g_vs.remote_unlocked = 0U;
                    if (g_vs.flash_seq != FLASH_SEQ_LOCATE) {
                        g_vs.flash_seq = FLASH_SEQ_IDLE;
                    }
                }
                break;

            case NB_CTRL_CMD3: /* 0x03: arg 0x86 -> locate (tone + locate flash) */
                if (arg == NB_CTRL_ARG_BUZZER) {
                    if (body0 == 1U) {
                        buzzer_on(BUZZER_TONE); /* cmd 0x03 arg 0x86 body==1 */
                        g_vs.chirp_req = 1U;
                        g_vs.keepalive_ticks = 300U; /* stay awake while locating */
                        g_vs.flash_seq = FLASH_SEQ_LOCATE;
                    } else if (body0 == 0U) {
                        buzzer_off();
                        g_vs.keepalive_ticks = 0U;
                        g_vs.flash_seq = FLASH_SEQ_IDLE;
                    }
                }
                break;

            case NB_CTRL_CMD4: /* 0x04: arg 0x20 -> display / ride-mode register */
                if (arg == NB_CTRL_ARG_MODE) {
                    uint16_t mode = (uint16_t)(frame[NB_OFF_BODY] | ((uint16_t)frame[NB_OFF_BODY + 1U] << 8));

                    g_scr.gest_payload = mode;
                    if ((mode == 0U) || (mode == 2U)) {
                        g_scr.gest_payload = 3U;
                    } else if ((mode == 1U) || (mode == 3U)) {
                        g_scr.gest_payload = 2U;
                    }
                }
                break;

            case NB_CTRL_OP7_SESSION: /* 0x07: retrigger the bus-active hold, clear the remote lock */
                g_in1.can_active_req = 1U;
                g_vs.remote_lock_edge = 0U;
                g_vs.remote_lock_state = 0U;
                break;

            case NB_CTRL_OP0A: /* 0x0A: long UV grace, CAN-active word 0, clear the remote lock */
                g_in1.uv_grace_ext = 1U;
                report_can_toggle(0); /* CAN-active word := 0 + report */
                g_vs.remote_lock_edge = 0U;
                g_vs.remote_lock_state = 0U;
                break;

            /* ----------------------------- info node (src 0x3D..3F) --------------- */
            case NB_INFO_REBOOT: /* 0x11: arg==1 -> reboot request + clear scratch */
                if (arg == 1U) {
                    g_vs.reboot_req = 1U;
                    g_scr.idle_report = 0U;
                }
                break;

            case NB_INFO_INPUT_TEST: /* 0x56: input-test mode (only with throttle and both levers at rest) */
                if (arg == 0x00U) {
                    buzzer_beep(BEEP_DOUBLE);
                    if (((control_adc_raw(VCU_ADC_SLOT_THROTTLE) >> 4) < 0x32U)
                        && ((control_adc_raw(VCU_ADC_SLOT_BRAKE_FULL) >> 4) < 0x32U)
                        && ((control_adc_raw(VCU_ADC_SLOT_BRAKE_HALF) >> 4) < 0x32U)) {
                        g_vs.input_test_mode = body0; /* input-test mode (vehicle.c FSM +0x64 path) */
                    } else {
                        g_vs.input_test_mode = 0U;
                    }
                    /* The reply (0x16, src, 1, 0x56, 0, f+5) echoes the request byte body[0], not
                       the stored mode (VS+0x64), which differs when the input-test gate fails.
                       CAN sink on whatever bus the request came on. */
                    nb_reply_can(frame[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_ACK, NB_INFO_INPUT_TEST, 0U, &body0, 1U);
                } else if (arg == 0x14U) {
                    nb_commit_mode(0U);
                    control_mode_sync();
                    vcu_outputs_reset();
                    g_vs.input_test_mode = 0U;
                } else if (arg == 0xEEU) {
                    g_vs.input_test_ee = 1U;
                } else if (arg == 0xEAU) {
                    g_vs.input_test_ea = 1U;
                }
                break;

            case NB_INFO_RIDE_START: /* 0x57: shares the 0x59 unlock handler */
            case NB_INFO_UNLOCK:     /* 0x59: token-gated unlock / ride session open */
                if ((frame[NB_OFF_LEN] == 0x0AU) && (nb_validate_token(&frame[NB_OFF_BODY]) != 0U)
                    && ((g_in2.f.status_flags & SF_SESSION) == 0U)) {
                    buzzer_beep(BEEP_DOUBLE);
                    /* args: n = 2, src = 0x16, dst = 7, cmd = 0x57, arg = 0 */
                    (void)nb_report_frame(2U, NB_ADDR_VCU, 7U, NB_INFO_RIDE_START, 0U, &frame[0x09U]);
                    g_in2.f.session_token = (uint16_t)(frame[0x0DU] | ((uint16_t)frame[0x0EU] << 8));
                    g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags | SF_SESSION);
                    nb_commit_mode(0x14U);
                    ridemode_led_matrix_update();
                    nb_drive_recompute_b();
                    /* session_dist / _total_time / _ride_time (IN2+0xC4..+0xCF), cleared as 6 halfwords */
                    g_in2.reg[NB_REG_SESSION_DIST + 0U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 1U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 2U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 3U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 4U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 5U] = 0U;
                    g_vs.gear_sel = 2U;
                    g_vs.reboot_req = 1U;
                    nb_state_refresh();
                    g_in1.recompute_req = 1U;
                } else {
                    buzzer_beep(BEEP_LONG);
                }
                break;

            case NB_INFO_RIDE_STOP: /* 0x58: token-gated ride/session stop */
                if ((frame[NB_OFF_LEN] == 0x08U) && (nb_validate_token(&frame[NB_OFF_BODY]) != 0U)) {
                    /* (the CAN-only app start runs in nb_info_command) */
                    buzzer_beep(BEEP_DOUBLE);
                    g_in2.f.session_token = 0U;
                    g_in2.f.status_flags = (uint16_t)(g_in2.f.status_flags & (uint16_t)~SF_SESSION);
                    /* session_dist / _total_time / _ride_time (IN2+0xC4..+0xCF), cleared as 6 halfwords */
                    g_in2.reg[NB_REG_SESSION_DIST + 0U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 1U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 2U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 3U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 4U] = 0U;
                    g_in2.reg[NB_REG_SESSION_DIST + 5U] = 0U;
                    ridemode_led_matrix_update();
                    nb_drive_recompute_b();
                    nb_state_refresh();
                    g_in1.recompute_req = 1U;
                } else {
                    buzzer_beep(BEEP_TRIPLE);
                }
                break;

            case NB_INFO_FLAG_5E: /* 0x5E: set IN1+0x23 (no reader), ack */
                g_in1.app_flag_5e = 1U;
                buzzer_beep(BEEP_DOUBLE);
                break;

            case NB_INFO_LIMIT_SET: /* 0x5F: PIN-gated write of the session distance (IN2+0xC4) */
                if ((frame[NB_OFF_LEN] == 0x08U) && (nb_rd_u32(&frame[NB_OFF_BODY]) == g_state_block.uid_cksum)) {
                    buzzer_beep(BEEP_DOUBLE);
                    g_in2.f.session_dist = nb_rd_u32(&frame[0x09U]); /* regs 0x62/0x63 (IN2+0xC4, u32) */
                } else {
                    buzzer_beep(BEEP_TRIPLE);
                }
                break;

            case NB_INFO_STANDBY: /* 0x79: body 1 app start (+ unlock when the lock is on), body 2 shutdown */
                if (body0 == 1U) {
                    g_vs.app_start_req = 1U;
                    if ((g_in2.f.cfg_flags & CFG_LOCK_EN) != 0U) {
                        g_vs.app_unlock_req = 1U;
                    }
                } else if (body0 == 2U) {
                    if (g_vs.speed_kmh < VCU_STANDSTILL_KMH) {
                        g_vs.shutdown_req = 1U;
                    }
                }
                break;

            case NB_INFO_LOCK_COMMIT: /* 0x80: commit lock / recompute, ack */
                ridemode_led_matrix_update();
                nb_drive_recompute_b();
                /* Single-frame report to node 7 (cmd 0x80, arg 1, 2 bytes from frame[9..10]):
                   report_ninebot(src 0x16, dst 7, len 2). */
                nb_reply_can(7U, NB_ADDR_VCU, NB_FLAG_REPLY, NB_INFO_LOCK_COMMIT, 1U, &frame[0x09U], 2U);
                nb_state_refresh();
                g_in1.recompute_req = 1U;
                buzzer_beep(BEEP_DOUBLE);
                break;

            case NB_INFO_VIN_WRITE: /* 0x19: CRC-gated VIN write committed by caller */ nb_state_refresh(); break;

            default:
                /* 0x13 / 0x41 are pure reads; their replies are built in the dispatcher. */
                break;
        }
        return;
    }

    /* ------------------------- housekeeping / auth / register events ----------- */
    switch (event) {
        case NB_EVT_AUTH_OK: buzzer_beep(BEEP_DOUBLE); break;   /* auth OK beep   */
        case NB_EVT_AUTH_FAIL: buzzer_beep(BEEP_TRIPLE); break; /* auth reject beep */

        case NB_EVT_OTA_FLAG: g_in1.can_active_req = 1U; break; /* re-arm the CAN-active window (IN1+3) */

        case NB_EVT_REG_WRITTEN: /* register write committed -> apply the register side effects */
            nb_reg_side_effects(g_nb_reg_idx, g_nb_reg_prev, g_in2.reg[g_nb_reg_idx]);
            break;

        case NB_EVT_SERIAL_COMMIT: /* commit provisioned serial / unlock (identity store) */
            nb_state_refresh();
            g_in1.recompute_req = 1U;
            break;

            /* NB: the ride-mode (reg 0x46) duty write is state-dependent and is performed
               inline in nb_ridemode_gate(); that path does NOT raise IN1+0x10. */

        default: break;
    }
}

/* ------------------------------------------------------------------------- */
/* Pairing / auth (cmd 0x18 and reg 0x46).                                   */
/* CRC-32 challenge over {body[0] | 16-byte RAM key | frame tail}, compared to  */
/* a big-endian u32 scattered across frame[0x0A..0x0D].                         */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Tail of the serial / VIN weighted checksums: salt, abs, mod 1000000
 * \param[in]       acc: the weighted accumulator, `acc = (w[i] + acc) * body[i]` in uint32_t
 * \return          `|acc + salt * 0x11| % 1000000`, the value the frame word must equal
 * \note            All uint32_t: the accumulator wraps by design. The abs negates when bit31
 *                      is set (0u - acc), so 0x80000000 stays put, and the modulus is unsigned.
 */
static uint32_t nb_pair_checksum(uint32_t acc) {
    acc = NB_IN2_SALT * 0x11U + acc; /* salt = IN2+0x1BC */
    if ((acc & 0x80000000U) != 0U) { /* abs of the signed value */
        acc = 0U - acc;
    }
    return acc % NB_PAIR_MOD;
}

/** \brief Big-endian challenge word from frame[0x0A..0x0D] */
static uint32_t nb_challenge_word(const uint8_t* f) {
    return (uint32_t)f[0x0DU] | ((uint32_t)f[0x0AU] << 24) | ((uint32_t)f[0x0BU] << 16) | ((uint32_t)f[0x0CU] << 8);
}

/**
 * \brief           Store the identity record and raise the recompute request
 * \note            The shared tail of both cmd 0x18 success paths: identity store, then
 *                      IN1+0x10 = 1.
 */
static void nb_pair_store(void) {
    nb_state_refresh();       /* identity_store */
    g_in1.recompute_req = 1U; /* IN1+0x10       */
}

/**
 * \brief           cmd 0x18 serial commit / provisioning
 * \note            The status starts as nb_keycmp(serial tail, "0000000001", 10), the byte
 *                      difference at the first mismatch (0 when the tail is the template).
 *                      1. CRC gate: CRC-32 over {frame[5], the 16-byte RAM key, frame[0x0E..0x1B]}
 *                         (31 bytes) equal to the big-endian word in frame[0x0A..0x0D]: status 1,
 *                         the serial (IN2+0x20, 14 bytes) := frame[0x0E..0x1B], identity store,
 *                         IN1+0x10 = 1.
 *                      2. Otherwise the serial provisioning: len 0x12, body starting with the
 *                         model's serial prefix (VCU_MODEL_SERIAL_PREFIX, GT3 "03S"), the weighted
 *                         checksum of the proposed serial (mod 1000000) equal to frame[0x13]:
 *                         serial := body, status 1, the factory UID copied to IN2+0x1B4 (3 words),
 *                         then config_serial_commit, the LED matrix update,
 *                         control_drive_param_init, identity store, IN1+0x10 = 1.
 *                      Status 0 raises the auth-reject beep, any other value the auth-OK beep
 *                      (nb_vcu_event); the reply arg is the status.
 * \param[in,out]   f: the request frame
 * \return          the reply status byte
 */
static uint8_t nb_pair_challenge(uint8_t* f) {
    uint8_t buf[NB_CHALLENGE_LEN]; /* CRC'd as 31 bytes */
    uint32_t crc;
    uint8_t status;

    /* status = nb_keycmp(IN2+0x24, "0000000001", 10) */
    status = (uint8_t)nb_keycmp((const uint8_t*)&g_in2.f.serial[NB_SERIAL_PROV_LEN - NB_SERIAL_LEN], NB_SERIAL_TEMPLATE,
                                NB_SERIAL_LEN);

    buf[0] = f[NB_OFF_BODY];                                                      /* frame[5]               */
    memcpy(&buf[1], g_nb.key, NB_KEY_LEN);                                        /* 16-byte RAM key        */
    memcpy(&buf[1U + NB_KEY_LEN], &f[0x0EU], NB_CHALLENGE_LEN - 1U - NB_KEY_LEN); /* 14 bytes from frame+0x0E */
    crc = nb_crc32(buf, NB_CHALLENGE_LEN);

    if (crc == nb_challenge_word(f)) {
        /* status 1, serial := frame[0x0E..0x1B] */
        status = 1U;
        memcpy((void*)g_in2.f.serial, &f[0x0EU], NB_SERIAL_PROV_LEN);
        nb_pair_store();
    } else if ((f[NB_OFF_LEN] == 0x12U) && (memcmp(&f[NB_OFF_BODY], k_nb_serial_prefix, NB_SERIAL_PREFIX_LEN) == 0)) {
        /* Serial provisioning: a serial of this model's family with the weighted checksum. */
        uint32_t acc = 0U;
        uint32_t i;

        for (i = 0U; i < NB_SERIAL_PROV_LEN; i = i + 1U) { /* walks IN2+0x20+i, 14 bytes */
            acc = ((uint32_t)g_in2.f.serial[i] + acc) * (uint32_t)f[NB_OFF_BODY + i];
        }
        if (nb_rd_u32(&f[0x13U]) == nb_pair_checksum(acc)) {
            volatile uint8_t* prov_dst = NB_BB_PROV_DST;
            volatile uint8_t* prov_src = NB_BB_PROV_SRC;
            uint32_t j;

            memcpy((void*)g_in2.f.serial, &f[NB_OFF_BODY], NB_SERIAL_PROV_LEN); /* IN2+0x20 := body, 14 bytes */
            status = 1U;
            for (j = 0U; j < 12U; j = j + 1U) { /* 3 words: factory UID -> IN2+0x1B4 */
                prov_dst[j] = prov_src[j];
            }
            nb_pair_commit();             /* config_serial_commit     */
            ridemode_led_matrix_update(); /* LED matrix               */
            nb_drive_recompute_b();       /* control_drive_param_init */
            nb_pair_store();
        }
    }

    nb_vcu_event((status != 0U) ? NB_EVT_AUTH_OK : NB_EVT_AUTH_FAIL, f);
    return status;
}

/**
 * \brief           reg 0x46 ride-mode write gate
 * \return          0 if the CRC gate passed, else status 4 (reject)
 */
static uint8_t nb_ridemode_gate(uint8_t* f) {
    uint8_t buf[NB_RIDEMODE_LEN]; /* CRC'd as 18 bytes */
    uint32_t crc;
    uint8_t state;
    uint8_t duty;

    buf[0] = f[NB_OFF_BODY];               /* frame[5]        */
    memcpy(&buf[1], g_nb.key, NB_KEY_LEN); /* 16-byte RAM key */
    buf[1U + NB_KEY_LEN] = f[0x0EU];       /* frame[0x0E]     */

    crc = nb_crc32(buf, NB_RIDEMODE_LEN);
    if (crc != nb_challenge_word(f)) {
        return NB_ST_REJECT; /* CRC gate failed */
    }

    /* Once the CRC passes, the committed duty depends on the region (VS->region) and
       frame[0x0E]. Only the low byte of the two duty ceilings (IN2->speed_limit /
       speed_limit_2) is replaced; the region selects the value and gates whether anything
       is written at all:
         GERMANY : 20 km/h if frame[0x0E]==0 else 22 km/h
         EUROPE  : (only if frame[0x0E]!=0) 25 km/h if speed_limit==0 else 35 km/h
         GLOBAL  : reject                                                          */
    state = g_vs.region;
    if (state == (uint8_t)VCU_REGION_GERMANY) {
        duty = (uint8_t)((f[0x0EU] == 0U) ? 0x14U : 0x16U);
        g_in2.f.speed_limit = (uint16_t)((g_in2.f.speed_limit & 0xFF00U) | duty);
        g_in2.f.speed_limit_2 = (uint16_t)((g_in2.f.speed_limit_2 & 0xFF00U) | duty);
    } else if (state == (uint8_t)VCU_REGION_EUROPE) {
        if (f[0x0EU] != 0U) {
            duty = (uint8_t)((g_in2.f.speed_limit == 0U) ? 0x19U : 0x23U);
            g_in2.f.speed_limit = (uint16_t)((g_in2.f.speed_limit & 0xFF00U) | duty);
            g_in2.f.speed_limit_2 = (uint16_t)((g_in2.f.speed_limit_2 & 0xFF00U) | duty);
        }
    } else {
        return NB_ST_REJECT; /* not in a provisionable region */
    }
    return NB_ST_OK;
}

/* ------------------------------------------------------------------------- */
/* Register file read / write: the shared IN2 block, register n = IN2 + 2n.   */
/* Read  -> reply with IN2 + arg*2 for body[0] 16-bit words;                 */
/* write -> memcpy(IN2 + arg*2, body, len) with an arg+len range guard.       */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Register READ: reply with `body[0]` bytes starting at register `arg`
 * \note            The reply length is body[0] itself, a BYTE count, not a word count
 *                      (reg 0x10 read of 14 -> the 14-byte serial; reg 0x1A count 1 -> 1
 *                      byte). The data starts at IN2 + arg*2. The `arg*2 + count <= 0x200`
 *                      guard keeps a reply from running past g_in2.reg[].
 */
static void nb_reg_read(uint8_t* f, int transport) {
    uint8_t arg = f[NB_OFF_ARG];
    uint8_t count = f[NB_OFF_BODY]; /* body[0] = reply length in bytes */

    if (((uint32_t)arg * 2U + (uint32_t)count) <= (NB_REG_COUNT * 2U)) {
        nb_reply(transport, f[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_REPLY, NB_CMD_REG_REPLY, arg,
                 (const uint8_t*)(const void*)&g_in2.reg[arg], count);
    }
}

/**
 * \brief           Register WRITE (op 4, raw): copy `len` bytes of body to IN2 + arg*2
 */
static void nb_reg_write(uint8_t* f, int side_effects) {
    uint8_t arg = f[NB_OFF_ARG];
    uint8_t len = f[NB_OFF_LEN];

    /* A write that would run past the register file is dropped instead of corrupting
       unrelated RAM. */
    if (((uint32_t)arg * 2U + (uint32_t)len) <= (NB_REG_COUNT * 2U)) {
        g_nb_reg_idx = arg;             /* latch the written index    */
        g_nb_reg_prev = g_in2.reg[arg]; /* old value before the write */
        memcpy((void*)&g_in2.reg[arg], &f[NB_OFF_BODY], len);
        /* cmd 2/3 run the register side effects; cmd 4 is a RAW write. */
        if (side_effects != 0) {
            nb_vcu_event(NB_EVT_REG_WRITTEN, f); /* register-changed: side effects */
        }
    }
}

/**
 * \brief           Reply with \p nbytes read straight out of a state block
 * \note            The sink gets the block pointer and the requested count, no
 *                      copy and no cap (the USART framer drops a body >= VCU_NINEBOT_REPLY_MAX
 *                      itself, the CAN sink goes to the J1939 TP above 5 bytes). Callers whose
 *                      count comes from the request bound it with \ref nb_clip.
 */
static void nb_reply_block(int transport, uint8_t* f, uint8_t cmd, uint8_t arg, const volatile uint8_t* blk,
                           uint8_t nbytes) {
    nb_reply(transport, f[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_REPLY, cmd, arg, (const uint8_t*)(const void*)blk, nbytes);
}

/**
 * \brief           Bound a requested reply count to the end of the object it is read from
 * \note            A count running past \p obj is cut at its end, so a reply never reads
 *                      beyond the object.
 * \param[in]       blk: first byte of the reply
 * \param[in]       count: requested byte count
 * \param[in]       obj: the object \p blk points into
 * \param[in]       obj_size: size of \p obj in bytes
 * \return          \p count, or the bytes left in \p obj when fewer
 */
static uint8_t nb_clip(const volatile void* blk, uint8_t count, const volatile void* obj, uint32_t obj_size) {
    uint32_t room = obj_size - (uint32_t)((const volatile uint8_t*)blk - (const volatile uint8_t*)obj);

    return (uint8_t)(((uint32_t)count > room) ? room : count);
}

/**
 * \brief           Extended register write, cmd 2 / cmd 3 (info node)
 * \note            The gates:
 *                      - ride-mode register 0x46 -> \ref nb_ridemode_gate (status 0 or 4);
 *                      - permission gate: every register arg+i for i in [0,len/2) must be
 *                        writable (\ref nb_reg_writable), else status 1 (reject);
 *                      - len must be even and nonzero, else status 4;
 *                      - reg 0x71 special: raw write to VS+0x56 and set VS+0x41=1 (no reg-file
 *                        write, no side effects);
 *                      - otherwise write IN2 register + run the register side effects;
 *                      - reply ONLY for cmd==2: cmd 0x05, arg = request arg, body = the
 *                        status code as a 16-bit LE value (2 bytes);
 *                      - trailing `if (IN2+0x3C & 1) beep(2)`.
 */
static void nb_reg_cmd(uint8_t* f, int transport) {
    uint8_t cmd = f[NB_OFF_CMD];
    uint8_t arg = f[NB_OFF_ARG];
    uint8_t len = f[NB_OFF_LEN];
    uint8_t status = NB_ST_OK;

    if (arg == NB_REG_SPEED_LIMIT) {
        status = nb_ridemode_gate(f); /* 0x46 CRC gate -> OK or REJECT */
    } else if (((len & 1U) == 0U) && (len != 0U)) {
        uint32_t i;

        for (i = 0U; i < (uint32_t)(len >> 1); i = i + 1U) {
            if (!nb_reg_writable((uint32_t)arg + i)) {
                status = NB_ST_DENIED; /* the permission table rejects this register range */
            }
        }
        if (status == NB_ST_OK) {
            if (arg == NB_REG_PASSCODE) { /* 0x71: stage the new key passcode in VS+0x56, raise VS+0x41 */
                uint8_t k;

                for (k = 0U; k < len; k = (uint8_t)(k + 1U)) {
                    ((volatile uint8_t*)g_vs.passcode_new)[k] = f[NB_OFF_BODY + k];
                }
                g_vs.passcode_set_pending = 1U;
                g_in1.recompute_req = 1U; /* as every accepted write */
            } else {
                g_nb_reg_idx = arg;
                g_nb_reg_prev = g_in2.reg[arg];
                memcpy((void*)&g_in2.reg[arg], &f[NB_OFF_BODY], len);
                nb_reg_side_effects(arg, g_nb_reg_prev, g_in2.reg[arg]); /* ends with IN1->recompute_req=1 */
            }
        }
    } else {
        status = NB_ST_REJECT; /* len odd or zero */
    }

    if (cmd == NB_CMD_WRITE2) { /* reply only for cmd 2; body = status as LE16, arg = request arg */
        /* The reply body is the status as NB_STATUS_REPLY_LEN bytes (`04 00` / `00 00`). */
        const uint8_t data[NB_STATUS_REPLY_LEN] = {status, 0U};

        nb_reply(transport, f[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_REPLY, NB_CMD_STATUS_REPLY, arg, data,
                 NB_STATUS_REPLY_LEN);
    }
    /* The ack beep needs reg 0x1E bit0 set and is suppressed in FSM states 0 (IDLE) and
       5 (SLEEP). */
    if (((g_in2.f.cfg_flags2 & CFG2_ACK_BEEP) != 0U) && (g_state_block.fsm_state != VCU_FSM_IDLE)
        && (g_state_block.fsm_state != VCU_FSM_SLEEP)) {
        buzzer_beep(BEEP_SHORT);
    }
}

/**
 * \brief           cmd 0x19 VIN / serial write (info node)
 * \note            len==0x2A gate; a 16-iteration weighted checksum over IN2+0x40
 *                      (acc = (IN2[0x40+i] + acc) * body[i]) salted by IN2+0x1BC*0x11, abs,
 *                      mod 1000000, compared against frame[0x15] (u32). On a match: copy 16
 *                      body bytes to IN2+0x40, write the build id (IN2+0x1C8 u32, IN2+0x1CC
 *                      u16), refresh derived state. Replies with status (0/1).
 */
static void nb_vin_write(uint8_t* f, int transport) {
    uint8_t status = NB_ST_OK;

    if (f[NB_OFF_LEN] == 0x2AU) {
        uint32_t acc = 0U;
        uint32_t i;
        uint32_t expect;

        for (i = 0U; i < 0x10U; i = i + 1U) {
            acc = ((uint32_t)g_in2.f.vin[i] + acc) * (uint32_t)f[NB_OFF_BODY + i];
        }
        expect = nb_pair_checksum(acc);
        if (nb_rd_u32(&f[0x15U]) == expect) {
            for (i = 0U; i < 0x10U; i = i + 1U) {
                g_in2.f.vin[i] = f[NB_OFF_BODY + i];
            }
            /* the 6 build-id bytes (a u32 and a u16) */
            memcpy((void*)g_in2.build_id, &f[0x19U], sizeof(g_in2.build_id));
            status = 1U; /* VIN write committed */
            nb_state_refresh();
        }
    }
    /* Reply: src 0x16, flag 0, cmd = request cmd (0x19), arg = result, no body. The CAN
       sink gets the non-NULL pointer f + 5: the single-frame builder drops a frame with a
       NULL payload even for n == 0. */
    nb_reply(transport, f[NB_OFF_SRC], NB_ADDR_VCU, 0U, f[NB_OFF_CMD], status, &f[NB_OFF_BODY], 0U);
}

/**
 * \brief           Info-node entry preamble (src 0x3D..3F, BEFORE the cmd dispatch)
 * \note            Runs for EVERY info-node cmd: clears the remote-lock edge
 *                      (VS+0x23), reloads the keep-awake timer (VS+0x34 = 300) and, in the ride
 *                      state, restarts the auto power-off countdown (VS+0x3C = reg 0x49 * 6000).
 */
static void nb_info_preamble(void) {
    g_vs.remote_lock_edge = 0U;
    g_vs.keepalive_ticks = 300U;
    if (g_state_block.fsm_state == VCU_FSM_RIDE) {
        g_vs.auto_off_ticks = (uint32_t)((int32_t)g_in2.f.auto_off_min_s * 6000);
    }
}

/* ------------------------------------------------------------------------- */
/* Command handlers.                                                          */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Vehicle control node (src==0x04)
 * \note            The cmd switch: register read (0x01), buzzer (0x03/0x86),
 *                      display/ride-mode (0x04/0x20), the arg 0x7E echo (cmd 0x02/0x04), ops
 *                      0x07/0x0A, remote lock value (0x7A) and remote lock/unlock (0x7B). The
 *                      live state writes are performed by \ref nb_vcu_event, once for every cmd
 *                      but the register read and before any reply. Every reply goes to the
 *                      USART ring (vcu_usart_report_send), whatever bus the request came on
 *                      (\p transport is unused); cmds 0x03, 0x07 and 0x0A send no reply.
 * \param[in]       f: the frame, starting at the `len` byte
 * \param[in]       transport: unused (kept for the dispatcher's call shape)
 */
static void nb_ctrl_command(uint8_t* f, int transport) {
    uint8_t cmd = f[NB_OFF_CMD];
    uint8_t arg = f[NB_OFF_ARG];

    (void)transport;
    /* Control-node entry preamble: any control frame keeps the VCU awake (>= 300). */
    if (g_vs.keepalive_ticks < 300U) {
        g_vs.keepalive_ticks = 300U;
    }
    /* Every cmd but the register read changes live state first; the replies below follow it. */
    if (cmd != NB_CTRL_REG_READ) {
        nb_vcu_event((uint16_t)(NB_EVT_CTRL_CMD | cmd), f);
    }

    switch (cmd) {
        case NB_CTRL_REG_READ: /* 0x01: IN2 register block, body[0] BYTES */
            nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_REG_REPLY, arg, (volatile uint8_t*)&g_in2.reg[arg],
                           nb_clip(&g_in2.reg[arg], f[NB_OFF_BODY], &g_in2, sizeof(g_in2)));
            break;

        case NB_CMD_WRITE2: /* 0x02 and 0x04 share the arg 0x7E echo */
        case NB_CTRL_CMD4:
            if ((cmd == NB_CTRL_CMD4) && (arg == NB_CTRL_ARG_MODE)) {
                /* reply (0x16, 4, len 2, cmd 3, arg 0x20, SCR+0x46) */
                nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_WRITE3, NB_CTRL_ARG_MODE,
                               (const volatile uint8_t*)&g_scr.gest_payload, 2U);
            } else if ((arg == NB_CTRL_ARG_ECHO)
                       || ((arg == NB_CTRL_ARG_E0)
                           && ((f[NB_OFF_BODY] == 1U) || (f[NB_OFF_BODY] == 3U) || (f[NB_OFF_BODY] == 5U)))) {
                /* arg 0x7E, or arg 0xE0 with body in {1,3,5}: beep(3), then the reply cmd 2,
                   arg 0x7E, 1 byte = body[0]. */
                buzzer_beep(BEEP_DOUBLE);
                nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_WRITE2, NB_CTRL_ARG_ECHO, &f[NB_OFF_BODY], 1U);
            } else {
                /* no reply */
            }
            break;

        case NB_CTRL_LOCK_STATE:  /* 0x7A: reply arg = the latched lock value */
        case NB_CTRL_REMOTE_LOCK: /* 0x7B: reply arg = the request arg        */
            nb_reply_usart(f[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_ACK, cmd,
                           (cmd == NB_CTRL_LOCK_STATE) ? g_vs.remote_lock_state : arg, 0, 0U);
            break;

        default: /* 0x03 / 0x07 / 0x0A and the rest: state only, no reply */ break;
    }
}

/**
 * \brief           BLE app / dashboard node (src==0x23)
 * \note            A DISTINCT handler from the ESC/info node: it uses its own
 *                      config-register bases (NB_BB_APPCFG / SCR / VS / IN1) and maps cmd
 *                      1/2/3/4 with the reg 0x2F/0x30/0x84/0xD2 specials, PLUS arg 0xFC/0xFD
 *                      which rewrite the cmd to the OTA/registration sub-protocol (busproto
 *                      bp_handle_fc / bp_node_register). Every reply goes to the USART ring
 *                      whatever bus the request came on (\p transport is unused).
 */
static void nb_app_command(uint8_t* f, int transport) {
    const uint8_t status[NB_STATUS_REPLY_LEN] = {NB_ST_OK, 0U}; /* zeroed status                */
    volatile uint8_t* cfg = NB_BB_APPCFG;                       /* app-node config register base */
    uint8_t arg = f[NB_OFF_ARG];
    uint8_t len = f[NB_OFF_LEN];
    uint8_t cmd;
    uint8_t i;

    (void)transport;
    g_lowglob.fm_uv_cnt = 0U; /* BLE app-session clear of the shared fm_uv_cnt cell (u16, the
                            whole cell, not a byte; faultmon uses it as its UV counter at other times) */
    if (arg == NB_APP_CMD_FC) {
        f[NB_OFF_CMD] = NB_APP_CMD_FC;
    } else if (arg == NB_APP_CMD_FD) {
        f[NB_OFF_CMD] = NB_APP_CMD_FD;
    }
    cmd = f[NB_OFF_CMD];

    if (cmd == NB_CMD_BACKUP) { /* read-only ESC backup: forward the body to the MCU over CAN */
        vcu_backup_app_request(transport, f[NB_OFF_SRC], &f[NB_OFF_BODY], f[NB_OFF_LEN]);
        return;
    }
    if (cmd == NB_CMD_WRITE4) { /* 4: raw write to cfg base -0x40 + arg*2 */
        volatile uint8_t* dst = cfg - 0x40U + (uint32_t)arg * 2U;

        for (i = 0U; i < len; i = (uint8_t)(i + 1U)) {
            dst[i] = f[NB_OFF_BODY + i];
        }
        return;
    }
    if (cmd < 5U) {
        if (cmd == NB_CMD_READ) { /* 1: config register reads, body[0] bytes */
            if (arg == 0x2FU) {
                nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_REG_REPLY, 0x17U, cfg - 0x12U,
                               nb_clip(cfg - 0x12U, f[NB_OFF_BODY], &g_in2, sizeof(g_in2)));
            } else if (arg == 0x30U) {
                nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_REG_REPLY, 0x20U, cfg,
                               nb_clip(cfg, f[NB_OFF_BODY], &g_in2, sizeof(g_in2)));
            }
        } else if (cmd == NB_CMD_WRITE2) { /* 2 */
            if (arg == 0x84U) {
                g_in1.uv_grace_ext = 0U;
                nb_commit_mode((uint8_t)g_scr.ui_screen_u);
                if ((int16_t)g_scr.ui_screen_u == 0x0B) {
                    /* app-mode 0x0B announce: cmd 3, arg 0,
                       NB_APP_ANNOUNCE_LEN bytes from cfg - 0x20, to the app node */
                    nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_WRITE3, 0U, cfg - 0x20U, NB_APP_ANNOUNCE_LEN);
                }
                if (g_state_block.fsm_state == VCU_FSM_IDLE) {
                    g_vs.app_start_req = 1U;
                }
                report_status(&g_stat.status_packed); /* g_stat+0x1FC */
            } else {
                volatile uint8_t* dst = (volatile uint8_t*)&g_scr + (uint32_t)arg * 2U; /* register page 1 */

                for (i = 0U; i < len; i = (uint8_t)(i + 1U)) {
                    dst[i] = f[NB_OFF_BODY + i];
                }
            }
            /* Status reply cmd 0x05, arg = request arg, body = the zeroed status, 2 bytes:
               `00 00`. The app handler always replies on the USART ring. */
            nb_reply_usart(f[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_REPLY, NB_CMD_STATUS_REPLY, arg, status,
                           NB_STATUS_REPLY_LEN);
        } else { /* 3 */
            if (arg == 0xD2U) {
                volatile uint8_t* dst = (volatile uint8_t*)&g_vs.distance_code; /* VS+0x0E */

                for (i = 0U; i < len; i = (uint8_t)(i + 1U)) {
                    dst[i] = f[NB_OFF_BODY + i];
                }
            }
        }
        return;
    }
    /* cmd >= 5: OTA / registration into busproto.c. */
    if (cmd == NB_APP_CMD_FC) {
        if ((g_vs.app_reg_pending != 0U) && (g_state_block.fsm_state == VCU_FSM_IDLE)) {
            config_delay_ms(0x32U);
            nb_commit_mode(0U);
            vcu_usart_tx_sequencer();
            config_delay_ms(0x32U);
            g_vs.app_start_req = 1U;
        }
        nb_app_register_prepare(f);
    } else if (cmd == NB_APP_CMD_FD) {
        nb_app_register_commit(f);
    }
}

/**
 * \brief           Extended sub-node src==0x02
 * \note            op4/arg0xDA: a raw write of the body to NB_BB_SUB02. op5/arg0xD0: node
 *                      0x02 acknowledges the reg 0xD0 parameter push, so the 2-bit push state
 *                      := 2, \ref NODE2_PUSH_DONE. No reply.
 */
static void nb_sub02_command(uint8_t* f) {
    if ((f[NB_OFF_CMD] == NB_CMD_WRITE4) && (f[NB_OFF_ARG] == NB_REG_SUB02_COMPONENT)) {
        volatile uint8_t* dst = NB_BB_SUB02;
        uint8_t len = f[NB_OFF_LEN];
        uint8_t i;

        for (i = 0U; i < len; i = (uint8_t)(i + 1U)) {
            dst[i] = f[NB_OFF_BODY + i];
        }
    } else if ((f[NB_OFF_CMD] == NB_CMD_SUB02_ACK) && (f[NB_OFF_ARG] == NB_REG_SUB02_PUSH)) {
        g_node2_push.state = (uint8_t)NODE2_PUSH_DONE;
    } else {
        /* any other node 0x02 frame: nothing */
    }
}

/**
 * \brief           Extended sub-nodes src==0x06 / 0x07
 * \note            A bounded register-array write (op4, arg+len/2 < 0x100) into
 *                      NB_BB_SCR2 + arg*2, plus the op5/body0x87/arg0 special that sets
 *                      IN1+0x28=2; then IN1+0x18=1 and the clear of g_bms_commloss (halfword).
 */
static void nb_sub0607_command(uint8_t* f) {
    volatile uint8_t* base = NB_BB_SCR2;
    uint8_t arg = f[NB_OFF_ARG];
    uint8_t cmd = f[NB_OFF_CMD];
    uint8_t len = f[NB_OFF_LEN];

    if (arg < 0xFFU) {
        if (cmd == NB_CMD_WRITE4) {
            if (((uint32_t)arg + (uint32_t)(len >> 1)) < 0x100U) {
                volatile uint8_t* dst = base + (uint32_t)arg * 2U;
                uint8_t i;

                for (i = 0U; i < len; i = (uint8_t)(i + 1U)) {
                    dst[i] = f[NB_OFF_BODY + i];
                }
            }
        } else if ((cmd == 0x05U) && (f[NB_OFF_BODY] == 0x87U) && (arg == 0U)) {
            g_in1.bms_chg_cmd_req = 2U;
        }
        g_in1.sub0607_rx = 1U;
        g_bms_commloss.cnt = 0U; /* the whole BMS comm-loss counter (halfword) */
    }
}

/** \brief Reply length of a cmd 0x41 arg 0x17 read. */
#define NB_IDENT_SERIAL_MARK_LEN 2U
/** \brief Reply length of a cmd 0x41 arg 0x20 read. */
#define NB_IDENT_VIN_LEN         16U
/** \brief Reply cmd of the cmd 0x41 identity reads. */
#define NB_CMD_IDENT_REPLY       0x43U

/**
 * \brief           cmd 0x41 identity / version block read
 * \note            Reply cmd 0x43, arg = request arg, on the request's bus:
 *                        - 0x10 and 0xDA: body[0] bytes from IN2+0x1B4 (the UID-derived ident);
 *                        - 0x17: 2 bytes from IN2+0x2E;
 *                        - 0x20: 0x10 bytes, the VIN at IN2+0x40.
 *                      An empty-body 0x10 request takes its count from the stale byte at
 *                      body[0] (the checksum byte). When body[0] > 0x4C the reply stops at
 *                      the end of g_in2.
 * \param[in]       f: stored request frame
 * \param[in]       transport: bus the request came on
 */
static void nb_ident_read(uint8_t* f, int transport) {
    const uint8_t* ident = (const uint8_t*)(const void*)g_in2.ident;
    uint32_t room = (uint32_t)(((const uint8_t*)(const void*)&g_in2 + sizeof(g_in2)) - ident);
    uint8_t count = f[NB_OFF_BODY];
    uint8_t arg = f[NB_OFF_ARG];

    if ((uint32_t)count > room) {
        count = (uint8_t)room;
    }
    switch (arg) {
        case 0x10U:
        case 0xDAU:
            nb_reply(transport, f[NB_OFF_SRC], NB_ADDR_VCU, NB_FLAG_REPLY, NB_CMD_IDENT_REPLY, arg, ident, count);
            break;
        case 0x17U:
            nb_reply_block(transport, f, NB_CMD_IDENT_REPLY, arg, NB_BB_SERIAL, NB_IDENT_SERIAL_MARK_LEN);
            break;
        case 0x20U: nb_reply_block(transport, f, NB_CMD_IDENT_REPLY, arg, g_in2.f.vin, NB_IDENT_VIN_LEN); break;
        default: break;
    }
}

/**
 * \brief           ESC / info node (src 0x3D..0x3F)
 * \note            Runs the entry preamble, then the cmd dispatch. The register
 *                      write (op2/op3) goes through \ref nb_reg_cmd, the VIN write
 *                      (cmd 0x19) through \ref nb_vin_write, the pairing challenge
 *                      (cmd 0x18) through \ref nb_pair_challenge. The remaining state cmds
 *                      (0x11/0x56..0x59/0x5E/0x5F/0x79/0x80) poke the blackboard + beep in
 *                      \ref nb_vcu_event and send NO extended-tunnel reply.
 */
static void nb_info_command(uint8_t* f, int transport, uint32_t p3, uint32_t p4) {
    uint8_t cmd = f[NB_OFF_CMD];
    uint8_t arg = f[NB_OFF_ARG];

    nb_info_preamble();

    switch (cmd) {
        case NB_CMD_READ: nb_reg_read(f, transport); break;

        case NB_INFO_CFG_READ:   /* 0x13: config block (telemetry frame from REPORT_TELEM_CFG_BASE), body[0] bytes */
        case NB_INFO_CFG_READ_2: /* 0x17: alias of 0x13 */
            /* Both always reply through the USART ring. */
            nb_reply_block((int)VCU_TRANSPORT_USART, f, NB_CMD_REG_REPLY, arg, NB_CFGBLK + (uint32_t)arg * 2U,
                           nb_clip(NB_CFGBLK + (uint32_t)arg * 2U, f[NB_OFF_BODY], g_telem_frame,
                                   sizeof(g_telem_frame)));
            break;

        case NB_INFO_IDENT_READ: /* 0x41: identity / version block reads (nb_ident_read) */
            nb_ident_read(f, transport);
            break;

        case NB_CMD_WRITE2:
        case NB_CMD_WRITE3: nb_reg_cmd(f, transport); break;

        case NB_CMD_WRITE4: nb_reg_write(f, 0); break;

        case NB_INFO_VIN_WRITE: nb_vin_write(f, transport); break;

        case NB_CMD_OTA_PREPARE:
        case NB_CMD_OTA_DATA:
        case NB_CMD_OTA_COMMIT:
        case NB_CMD_OTA_APPLY:
            /* OTA sub-protocol -> ota.c, gated on OTA-busy bit0 clear. */
            if ((g_fault_codes.comm_fault & 1U) == 0U) {
                g_in1.can_active_req = 1U;
                vcu_ota_dispatch(f, transport, p3, p4);
            }
            break;

        case NB_CMD_PAIR_CHALLENGE: {
            uint8_t status = nb_pair_challenge(f);

            /* Reply: src 0x16, flag 0, cmd = request cmd (0x18), arg = status, no body; the CAN
               sink gets the non-NULL f + 5 (see nb_vin_write). */
            nb_reply(transport, f[NB_OFF_SRC], NB_ADDR_VCU, 0U, cmd, status, &f[NB_OFF_BODY], 0U);
            break;
        }

        case NB_INFO_RIDE_STOP:
            /* Once the token is accepted, a pending app registration in IDLE starts the scooter,
               but only for a stop that came over CAN.
               The token check is pure, so running it here first changes nothing else. */
            if ((transport == (int)VCU_TRANSPORT_CAN) && (f[NB_OFF_LEN] == 0x08U)
                && (nb_validate_token(&f[NB_OFF_BODY]) != 0U) && (g_vs.app_reg_pending != 0U)
                && (g_state_block.fsm_state == VCU_FSM_IDLE)) {
                g_vs.app_start_req = 1U;
            }
            /* fall through: the stop itself is a state sub-command like the default ones */
        default:
            /* Info-node state sub-commands: blackboard pokes + beep, no extended reply. */
            nb_vcu_event((uint16_t)(NB_EVT_CTRL_CMD | cmd), f);
            break;
    }
}

/**
 * \brief           dst != 0x16 generic tunnel path
 * \note            Frames for another node: re-sent verbatim on the other bus
 *                      (\ref nb_forward_frame). cmd 7..10 raises IN1 can_active_req then forwards.
 *                      For the CAN path, (src==7, cmd==5, body==0x82, IN2+0x3C bit0) -> beep(2).
 *                      For the USART path with dst==7: (arg f[4]==0x10, cmd==0x2B) -> status pack
 *                      of f+8 (report_status) + forward, then the cmd != 0x7A forward again
 *                      (the frame goes out twice);
 *                      (cmd==0x7A, gated by VS+2==0 && !(SCR2+0x126 bit3) && STATE==3) ->
 *                      IN1+1=1 + commit mode 0xF.
 */
static void nb_dispatch_generic(uint8_t* f, int transport) {
    uint8_t cmd = f[NB_OFF_CMD];
    uint8_t src = f[NB_OFF_SRC];
    uint8_t dst = f[NB_OFF_DST];

    if ((uint8_t)(cmd - 7U) < 4U) { /* OTA-adjacent cmd range (cmd 7..10) */
        if ((g_fault_codes.comm_fault & 1U) != 0U) {
            /* The VCU's own OTA is busy, so instead of forwarding the frame the VCU answers it:
               report_ninebot-style reply on USART, cmd 0x0B code 9, no body, with the frame's
               src/dst swapped (reply from the addressed node back to the sender). */
            nb_reply_usart(src, dst, NB_FLAG_ACK, NB_CMD_OTA_REPLY, NB_OTA_BUSY_CODE, 0, 0U);
        } else {
            g_in1.can_active_req = 1U;
            nb_forward_frame(f, transport);
        }
        return;
    }

    if (transport == (int)VCU_TRANSPORT_CAN) { /* CAN transport */
        nb_forward_frame(f, transport);
        if ((src == 0x07U) && (cmd == 0x05U) && (f[NB_OFF_BODY] == 0x82U) && ((g_in2.f.cfg_flags2 & 1U) != 0U)) {
            buzzer_beep(BEEP_SHORT);
        }
        return;
    }

    if (dst == 0x07U) { /* USART transport, dst==7 */
        if ((f[NB_OFF_ARG] == NB_FWD_STATUS_ARG) && (cmd == NB_FWD_STATUS_CMD)) {
            /* gate on f[4]/f[3]; report_status gets f + 8 */
            report_status((const volatile uint32_t*)(const void*)&f[NB_FWD_STATUS_OFF]);
            nb_forward_frame(f, transport);
        }
        if (cmd == 0x7AU) {
            if ((g_vs.speed_kmh == 0U) && ((g_stat.status_word_b & 0x08U) == 0U)
                && (g_state_block.fsm_state == VCU_FSM_RIDE)) {
                g_in1.remote_countdown_req = 1U; /* IN1+1 */
                nb_commit_mode(0x0FU);
            }
        } else {
            nb_forward_frame(f, transport);
        }
        return;
    }
    nb_forward_frame(f, transport);
}

/**
 * \brief           dst == 0x16 local protocol: route on the src byte
 * \note            Routes on the `src` byte FIRST: the BLE app node (0x23), the extended
 *                      sub-nodes (0x02 / 0x06 / 0x07), the vehicle control node (0x04) and the
 *                      ESC/info node (0x3D..0x3F) each carry a DISTINCT handler; any other src
 *                      is dropped with no reply.
 */
static void nb_dispatch_extended(uint8_t* f, int transport, uint32_t p3, uint32_t p4) {
    uint8_t src = f[NB_OFF_SRC];

    if (src == NB_ADDR_APP) { /* 0x23 */
        nb_app_command(f, transport);
        return;
    }
    if (src < 0x24U) {
        if (src == NB_ADDR_SUB02) {
            nb_sub02_command(f);
        } else if (src == NB_ADDR_CTRL) {
            nb_ctrl_command(f, transport);
        } else if ((src == NB_ADDR_SUB06) || (src == NB_ADDR_SUB07)) {
            nb_sub0607_command(f);
        }
        /* any other src < 0x24: no reply. */
        return;
    }
    if ((src == NB_ADDR_ESC_3D) || (src == NB_ADDR_ESC_3E) || (src == NB_ADDR_ESC_3F)) {
        nb_info_command(f, transport, p3, p4);
    }
    /* any other src >= 0x24: no reply. */
}

/* ------------------------------------------------------------------------- */
/* Public entry points.                                                       */
/* ------------------------------------------------------------------------- */

void vcu_ninebot_process(uint8_t* frame, int transport, uint32_t p3, uint32_t p4) {
    uint8_t dst;

    if (frame == 0) {
        return;
    }

    dst = frame[NB_OFF_DST];

    /* Outer route on the dst byte. */
    if ((dst == NB_ADDR_VCU) || (dst == 0U)) {
        nb_dispatch_extended(frame, transport, p3, p4);
        return;
    }
    if ((dst == 0x02U) || (dst == 0x04U) || (dst == 0x06U) || (dst == 0x07U) || (dst == NB_ADDR_APP)
        || (dst == NB_ADDR_ESC_3D) || (dst == NB_ADDR_ESC_3E) || (dst == NB_ADDR_ESC_3F)) {
        nb_dispatch_generic(frame, transport);
        return;
    }
    /* Unknown dst: return without replying. */
}

/**
 * \brief           USART entry into the Ninebot dispatcher (called by usart.c's parser)
 * \note            The USART parser owns `g_frame`; the dispatcher writes register state
 *                      (never the frame itself on the USART path), so the const is cast off
 *                      where the dispatcher treats the buffer as mutable.
 */
void vcu_ninebot_dispatch(const uint8_t* frame) {
    vcu_ninebot_process((uint8_t*)(const void*)frame, (int)VCU_TRANSPORT_USART, 0U, 0U);
}

/**
 * \brief           Build+send a 0xFC Ninebot reply
 * \note            candev handler +0x38 and the 0xEF route target of can.c's can_rx_ext.
 *                      Zeroes a 252-byte scratch, lays out [len-3][src][dst][body1][body2]
 *                      then the body, and dispatches via the shared Ninebot handler
 *                      (vcu_ninebot_process, VCU_TRANSPORT_CAN). The leading \p x arg is
 *                      unused.
 * \param[in]       src: source node address
 * \param[in]       dst: destination node address
 * \param[in]       body: command body (body[1], body[2] land in the header; body+3 is payload)
 * \param[in]       total_len: frame selector; body length = total_len - 3
 */
void nb_reply_fc(void* x, uint8_t src, uint8_t dst, const uint8_t* body, char total_len) {
    uint8_t buf[0x110];
    uint8_t n;
    uint8_t i;

    (void)x;
    for (i = 0; i < 0xFCu; i++) { /* memclr(buf, 0xFC): zero 252 bytes */
        buf[i] = 0;
    }
    n = (uint8_t)((int)total_len - 3);
    buf[0] = n;
    buf[1] = src;
    buf[2] = dst;
    buf[3] = body[1];
    buf[4] = body[2];
    for (i = 0; i < n; i++) { /* memcpy(&buf[5], body+3, total_len-3) */
        buf[5 + i] = body[3 + i];
    }
    vcu_ninebot_process(buf, (int)VCU_TRANSPORT_CAN, 0, 0); /* CAN 0xEF route: transport 1 */
}
