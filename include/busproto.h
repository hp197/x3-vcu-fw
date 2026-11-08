/**
 * \file            busproto.h
 * \brief           Peripheral-bus multi-packet transport engine (x3 VCU, wave3)
 * \note            The second Ninebot-style protocol engine: the peripheral / BMS comms
 *                      master that rides the internal CAN bus. It is a J1939-style Transport
 *                      Protocol (TP) master; multi-packet messages move over the CAN 29-bit
 *                      extended-id tunnel using two PGNs carried in the id:
 *                        - 0x18EC00xx  TP.CM (connection management) -- control bytes
 *                                            0x10 RTS, 0x11 CTS, 0x13 EndOfMsgAck, 0xFF Abort
 *                        - 0x18EB00xx  TP.DT (data transfer) -- 7 payload bytes/packet,
 *                                            preceded by a 1-byte sequence number
 *                      The transported PGN is 0x00EF00 (J1939 "Proprietary A"), checked on
 *                      every frame. Inbound transfers from a peripheral (BMS / dashboard) are
 *                      reassembled into a 300-byte buffer with a trailing 16-bit one's-complement
 *                      checksum, and the matching CTS / EndOfMsgAck / Abort replies are built.
 *                      The TX path builds its buffer with bp_build_packet and is driven by the
 *                      fast-service tick (bp_task_fast_service, end of busproto.c) over the same
 *                      session array.
 *
 *                      Transport: internal CAN bus, 29-bit extended id. Inbound frames come from
 *                      the ext-id CAN RX handler (can.c vcu_can_rx_dispatch); outbound frames
 *                      leave through the per-manager emit vtable, backed by can.c
 *                      vcu_can_send_ext(). A completed transfer is handed on as a 0xFC Ninebot
 *                      frame (bp_on_data -> ninebot.c nb_reply_fc, transport 1 = CAN).
 *
 *                      The J1939 / BMS role names are just labels; the numeric values, offsets,
 *                      ids and checksum are what the code uses.
 *
 *                      State owned here (module globals): the session state (5 x 0x24-byte
 *                      descriptors + 5 x 300-byte buffers + manager + emit vtable), the node
 *                      pairing/auth table, the report manager and the node-reg record. What it
 *                      shares with other modules (g_vs, the USART context g_uartctx, g_bp_rptr)
 *                      lives in vcu_state.h. Cross-module sends call the real can.c / ninebot.c /
 *                      usart.c APIs.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef BUSPROTO_H
#define BUSPROTO_H

#include <stddef.h>
#include <stdint.h>
#include "can.h" /* vcu_can_ext_frame_t (the 13-byte ext-id frame record) */

/* ========================================================================= */
/* J1939-style Transport-Protocol engine: named constants, enums and types.    */
/*                                                                             */
/* Numeric values, record strides, ids and the checksum used by busproto.c.    */
/* The J1939 / BMS role names are descriptive labels.                          */
/* ========================================================================= */

/* ------------------------------------------------------------------------- */
/* CAN 29-bit extended-id composition.                                        */
/* Composed id = (dst | src<<8 | base) & MASK | FLAG, with the top bits of the */
/* previous id word preserved through KEEP. The two PGN-carrying id bases are:  */
/*   TP.CM 0x18EC0000 (connection management), TP.DT 0x18EB0000 (data).        */
/* The TP.CM builders use the CM base; the TP.DT builder uses the DT base.     */
/* ------------------------------------------------------------------------- */
#define BP_EXTID_CM_BASE  0x18EC0000U /* TP.CM id base (connection management)  */
#define BP_EXTID_DT_BASE  0x18EB0000U /* TP.DT id base (data transfer)          */
#define BP_EXTID_KEEP     0xA0000000U /* top bits preserved from prior id word  */
#define BP_EXTID_MASK     0x1FFFFFFFU /* 29-bit id mask                         */
#define BP_EXTID_FLAG     0x20000000U /* extended-id marker bit (bit29)         */

/* Transported PGN carried in data[5..7]: 0x00EF00 (J1939 "Proprietary A").    */
#define BP_PGN_EF00       0xEF00U
#define BP_TP_PGN_LO      0x00U /* PGN 0x00EF00, little-endian bytes: data[5] */
#define BP_TP_PGN_MID     0xEFU /*                                    data[6] */
#define BP_TP_PGN_HI      0x00U /*                                    data[7] */
#define BP_TP_PAD         0xFFU /* reserved / pad byte in TP.CM and TP.DT frames */

/* ------------------------------------------------------------------------- */
/* Transport sizing constants.                                                 */
/* ------------------------------------------------------------------------- */
#define BP_TP_CHUNK       7U     /* TP.DT payload bytes per packet             */
#define BP_SESSIONS       5U     /* channels / session descriptors             */
#define BP_BUF_SIZE       300U   /* reassembly/transmit buffer per channel     */
#define BP_MAX_PAYLD      1785U  /* bp_build_packet payload cap                 */

/* Shared node pairing/auth table layout (two banks).                          */
#define BP_PAIR_KEY_LEN   20U /* bytes per key/credential slot                 */
#define BP_PAIR_SLOTS     32U /* key slots per bank (two banks)                */

/* Node-registration record.                                                   */
#define BP_NODE_REC_SIZE  40U /* total record size                          */
#define BP_NODE_CAP_SLOTS 38U /* capability-present flag slots              */

/* Report/log subsystem sizing.                                                */
#define BP_RPT_REPLY_LEN  16U     /* 0xFC-reply payload length                  */

/* Report/Ninebot-layer identifiers. The report frames go from NB_ADDR_VCU to
   NB_ADDR_APP (usart.h). */
#define BP_ANNOUNCE_ARG   0xD0U /* "device present" self-announce arg byte    */
#define BP_ANNOUNCE_LEN   2U    /* announce body: 16-bit LE value 1 (`01 00`)  */
#define BP_CMD_FC         0xFCU /* report/log command byte (0xFC handler)     */

/* Per-session timeout seeds, divided by the session period (1) on reseed: counts of */
/* the slot-3 fast service, which runs every scheduler tick (1.0 ms), so             */
/* 1250 / 1050 / 750 ticks = 1.25 / 1.05 / 0.75 s.                                   */
#define BP_TMO_RTS        1250 /* ticks: after sending CTS, await TP.DT              */
#define BP_TMO_CTS        1050 /* ticks                                              */
#define BP_TMO_DT         750  /* ticks: between TP.DT packets                       */

/* ------------------------------------------------------------------------- */
/* TP finite-state machine: states, control bytes, PGN-format bytes.           */
/* The numeric values are stored in / read from shared memory by the vehicle.c */
/* TX FSM, so the exact values are load-bearing.                               */
/* ------------------------------------------------------------------------- */

/** \brief Session FSM state (session descriptor +0x02). */
typedef enum {
    BP_ST_IDLE = 1,    /*!< free / closed                                    */
    BP_ST_BUILT = 3,   /*!< TX buffer built (bp_build_packet)                */
    BP_ST_TX_CTS = 4,  /*!< TX: awaiting / consuming CTS                     */
    BP_ST_TX_SEND = 5, /*!< TX: streaming TP.DT                              */
    BP_ST_TX_RTS = 6,  /*!< TX: RTS sent, awaiting first CTS                 */
    BP_ST_TX_WAIT = 7, /*!< TX: window sent, awaiting next CTS               */
    BP_ST_TX_DONE = 9, /*!< TX: all data sent, awaiting EndOfMsgAck          */
    BP_ST_RX_DATA = 10 /*!< RX: reassembling TP.DT                           */
} bp_state_t;

/** \brief TP.CM control byte (frame data[0]). */
typedef enum {
    BP_TP_RTS = 0x10,  /*!< request-to-send   (opens a session)             */
    BP_TP_CTS = 0x11,  /*!< clear-to-send     (peer grants a window)        */
    BP_TP_EOMA = 0x13, /*!< end-of-message ack                              */
    BP_TP_ABORT = 0xFF /*!< connection abort                                */
} bp_tp_ctrl_t;

/** \brief PGN-format byte of the id ((id>>16)&0xFF). */
typedef enum {
    BP_PF_DT = 0xEB, /*!< TP.DT data transfer                                */
    BP_PF_CM = 0xEC  /*!< TP.CM connection management                        */
} bp_pf_t;

/** \brief on_ack phase argument: EndOfMsgAck completion. */
typedef enum { BP_ACK_PHASE_EOMA = 3 } bp_ack_phase_t;

/* ------------------------------------------------------------------------- */
/* Engine-owned record layouts (byte offsets).                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           CAN 29-bit extended-id frame / mailbox record (13-byte layout)
 * \note            u32 id @+0x00, dlc @+0x04 (always 8 for TP), data[8] @+0x05. The same
 *                      record the Ninebot single-frame builder fills, so it is one type.
 */
typedef vcu_can_ext_frame_t bp_frame_t;

/** \brief J1939-TP session / reassembly descriptor (0x24-byte record). */
typedef struct {
    uint8_t addr_a;   /*!< +0x00 local/source address (SA)                   */
    uint8_t addr_b;   /*!< +0x01 peer/destination address (DA)               */
    uint8_t state;    /*!< +0x02 FSM state (\ref bp_state_t)                  */
    uint8_t pad3;     /*!< +0x03                                             */
    uint8_t* buf;     /*!< +0x04 reassembly/transmit buffer                  */
    uint16_t cap;     /*!< +0x08 buffer capacity                             */
    uint16_t pos;     /*!< +0x0A byte cursor (bytes received / sent)         */
    uint16_t total;   /*!< +0x0C declared total length                       */
    uint16_t seq;     /*!< +0x0E packet sequence number                      */
    uint16_t npkt;    /*!< +0x10 total packet count                          */
    uint16_t win_end; /*!< +0x12 window end position                         */
    uint16_t period;  /*!< +0x14 timer divisor base (ticks per ms unit)      */
    uint16_t tick;    /*!< +0x16 pacing tick (counts up to `timeout`; u16 cell) */
    int16_t timeout;  /*!< +0x18 timeout countdown (seed/period)             */
    uint16_t retry;   /*!< +0x1A retry/aux                                   */
    int16_t deadline; /*!< +0x1C deadline (bp_build_packet)                  */
    uint16_t csum;    /*!< +0x1E running 16-bit additive checksum            */
    void* ctx;        /*!< +0x20 per-session ctx (unused; registered as 0)   */
} bp_session_t;

/** \brief Handler / callback vtable (4 code pointers). */
typedef struct {
    int (*node_accept)(uint8_t sa, uint8_t da);                                        /*!< +0x00 */
    void (*on_ack)(bp_session_t* s, uint8_t sa, uint8_t da, int phase);                /*!< +0x04 */
    void (*on_data)(bp_session_t* s, uint8_t sa, uint8_t da, const uint8_t* b, int n); /*!< +0x08 */
    int (*emit)(bp_session_t* s, const bp_frame_t* f);                                 /*!< +0x0C */
} bp_ctx_t;

/** \brief Comms manager / handler table. */
typedef struct {
    bp_session_t* base; /*!< +0x00 session array                             */
    uint16_t count;     /*!< +0x04 session count                            */
    uint16_t pad6;      /*!< +0x06                                          */
    bp_ctx_t* ctx;      /*!< +0x08 handler/callback vtable                   */
} bp_mgr_t;

/* ------------------------------------------------------------------------- */
/* Pairing table and report-node record (busproto.c globals).                 */
/* Every named field is pinned to its byte offset below.                       */
/* ------------------------------------------------------------------------- */

/** \brief One pairing/auth key slot. */
typedef struct {
    uint8_t key[BP_PAIR_KEY_LEN]; /*!< 20-byte key/credential slot */
} bp_pair_slot_t;

/**
 * \brief           Node pairing/auth table
 * \note            Two banks of 32 x 20-byte key slots, each bank preceded by two
 *                      32-bit counter words.
 */
typedef struct {
    uint32_t count_a0;                    /*!< +0x000 bank-A counter word 0           */
    uint32_t count_a1;                    /*!< +0x004 bank-A counter word 1           */
    bp_pair_slot_t bank_a[BP_PAIR_SLOTS]; /*!< +0x008 bank A key slots                 */
    uint32_t count_b0;                    /*!< +0x288 bank-B counter word 0           */
    uint32_t count_b1;                    /*!< +0x28C bank-B counter word 1           */
    bp_pair_slot_t bank_b[BP_PAIR_SLOTS]; /*!< +0x290 bank B key slots                 */
} bp_pair_table_t;

/**
 * \brief           Registered report-node record (0x28 bytes)
 * \note            Filled by bp_node_register() and cleared (whole record) by it, by the
 *                      busproto init and by nb_commit_mode(0). The slot-1 dispatch tick uses the
 *                      same record to pace the node's state report: it fires
 *                      bp_state_notify() every report_period*10 ticks while cap[0] is set.
 *                      The dispatch tick reads +0x01/+0x02 as its divider period / enable.
 */
typedef struct {
    uint8_t node_id;                /*!< +0x00 resolved node id                                 */
    uint8_t report_period;          /*!< +0x01 state-report period (x10 ticks), from descriptor[+2] */
    uint8_t cap[BP_NODE_CAP_SLOTS]; /*!< +0x02..+0x27 capability flags; cap[0] also enables the periodic report */
} bp_node_rec_t;

/** \brief The registered report-node record (owned by busproto.c). */
extern volatile bp_node_rec_t g_bp_node;

/* ------------------------------------------------------------------------- */
/* Report-manager object.                                                     */
/*                                                                            */
/* The emit/register slots are the embedded vtable of one object, the report    */
/* manager, which also holds the node table: the 7-pointer vtable, the Ninebot   */
/* frame header, the table and its metadata all live in this one object.         */
/* g_report_mgr is zero-init BSS, so the vtable + mgr_ptr are filled in at init   */
/* (bp_report_mgr_init).                                                         */
/* ------------------------------------------------------------------------- */

/* Node-table geometry (\ref bp_report_node_t). */
#define BP_NODE_MAX        10u /* table size / table-full threshold (count >= 10) */
#define BP_NODE_REC_MAX    50u /* resolved-record slots per node                   */
#define BP_RMGR_SEQ_SCALE  20u /* resolve: node.seq *= 20                  */

/* Per-emit Ninebot frame cmd byte (mgr +0x1F) and arg byte (mgr +0x20). */
#define BP_RPT_CMD_FC      4u    /* 0xFC report cmd        */
#define BP_RPT_CMD_FD_ACK  1u    /* 0xFD ack   cmd         */
#define BP_RPT_CMD_FE      3u    /* 0xFE state cmd         */
#define BP_RPT_CMD_FD_LIST 5u    /* 0xFD list  cmd         */
#define BP_RPT_ARG_FC      0xFCu /* report frame arg       */
#define BP_RPT_ARG_FD      0xFDu /* ack/list frame arg     */
#define BP_RPT_ARG_FE      0xFEu /* state frame arg        */

/* Embedded report-manager vtable slot signatures. */
typedef int (*bp_vt_resolve_fn)(const uint8_t* key, uint32_t id);                          /* +0x00 */
typedef void (*bp_vt_emit_fn)(void* self, int arg, const uint8_t* payload, uint16_t plen); /* +0x04 */
typedef int (*bp_vt_emit_ack_fn)(uint8_t code, int ok);                                    /* +0x08 */
typedef void (*bp_vt_state_notify_fn)(void* node);                                         /* +0x0C */
typedef int (*bp_vt_register_obj_fn)(uint8_t id, void* obj);                               /* +0x10 */
typedef void (*bp_vt_notify_fn)(uint8_t flag);                                             /* +0x14 */
typedef void* (*bp_vt_register_desc_fn)(uint8_t id);                                       /* +0x18 */

/** \brief Number of signal descriptors (0x26). */
#define BP_SIG_DESC_COUNT 38u

/**
 * \brief           Report signal descriptor
 * \note            `key` is matched by bp_desc_find, `width` is checked by bp_resolve_records and
 *                      is the field width bp_state_serialize packs. The value is the u32 at g_scr
 *                      halfword `scr_idx` (two halfwords, low `width` bits packed).
 *                      bp_resolve_records stores the index of each matched entry in the node's
 *                      record table; the table itself stays in flash.
 */
typedef struct {
    uint32_t key;    /*!< 32-bit signal key (find-by-key) */
    uint8_t width;   /*!< bit width 1..16                 */
    uint8_t scr_idx; /*!< halfword index into g_scr       */
} bp_sig_desc_t;

/** \brief Registered node-object init slot (word obj+0x04); fired by register_obj. */
typedef void (*bp_obj_init_fn)(void* obj);

/**
 * \brief           Object registered on a report node (vt_register_obj)
 * \note            The report manager calls two words of the object, both code pointers
 *                      stored in the object itself: register_obj calls the word at obj+0x04, the
 *                      key-0xFF broadcast of the resolve slot calls the word at obj+0x0C, each
 *                      with the object as argument. Nothing in this firmware registers one.
 */
typedef struct {
    uint32_t _w00;                      /*!< +0x00 never called here             */
    bp_obj_init_fn init;                /*!< +0x04 init (fired by register_obj)  */
    uint32_t _w08;                      /*!< +0x08 never called here             */
    bp_vt_state_notify_fn state_notify; /*!< +0x0C state notify (key 0xFF broadcast) */
} bp_node_obj_t;

/**
 * \brief           One report-node record of the manager's node table
 * \note            Filled by the resolve slot from the requester's key: the node id,
 *                      the requested sequence (scaled by \ref BP_RMGR_SEQ_SCALE), and one
 *                      copy of every matched signal descriptor. Its address is the node
 *                      handle vt_register_desc returns.
 */
typedef struct {
    uint8_t id;                         /*!< node id (key[0])                            */
    uint8_t resolved;                   /*!< set once the records resolved              */
    uint16_t seq;                       /*!< sequence: key[1], then x BP_RMGR_SEQ_SCALE */
    uint8_t rec[BP_NODE_REC_MAX]; /*!< resolved records: indices into the descriptor table */
    uint8_t rec_count;                  /*!< resolved record count (cleared on re-resolve) */
    uint8_t _pad325[3];                 /*!<                                     */
    const volatile bp_node_obj_t* obj;  /*!< registered object (vt_register_obj)         */
} bp_report_node_t;

/**
 * \brief           Report-manager object layout
 * \note            The tail cells hold the node count and the desc_table/desc_count/send_obj
 *                      metadata.
 */
typedef struct {
    bp_vt_resolve_fn vt_resolve;             /*!< resolve / broadcast              */
    bp_vt_emit_fn vt_emit_fc;                /*!< emit 0xFC report                 */
    bp_vt_emit_ack_fn vt_emit_fd;            /*!< emit 0xFD ack                    */
    bp_vt_state_notify_fn vt_state_notify;   /*!< emit 0xFE state                  */
    bp_vt_register_obj_fn vt_register_obj;   /*!< register node object by id        */
    bp_vt_notify_fn vt_notify;               /*!< store flag byte                  */
    bp_vt_register_desc_fn vt_register_desc; /*!< lookup node descriptor by id      */
    uint8_t flag;                            /*!< status flag (vt_notify target)    */
    uint8_t nb_src;                          /*!< Ninebot header src (NB_OFF_SRC)    */
    uint8_t nb_dst;                          /*!< Ninebot header dst (NB_OFF_DST)    */
    uint8_t nb_cmd;                          /*!< Ninebot header cmd (per emit)      */
    uint8_t nb_arg;                          /*!< Ninebot header arg (FC/FD/FE)      */
    uint8_t _pad21[3];                       /*!< pad to node[0].id          */
    bp_report_node_t node[BP_NODE_MAX];      /*!< node table */
    uint8_t node_count;                      /*!< active node count (u8)          */
    uint8_t _pad1fdd[3];                     /*!<                        */
    uint32_t desc_table;                     /*!< descriptor-table base ptr        */
    uint16_t desc_count;                     /*!< descriptor count                */
    uint16_t _pad1fe6;                       /*!<                                 */
    uint32_t send_obj;                       /*!< send object = the USART2 TX ring ctx (&g_uartctx) */
} bp_report_mgr_t;

/** \brief Report descriptor / vtable. */
typedef struct {
    uint32_t descriptor; /*!< +0x00 descriptor word                       */
    uint32_t vfn;        /*!< +0x04 report descriptor vfn (bp_report_vfn) */
} bp_report_vtable_t;

/** \brief Report-manager handle/pointer pair. */
typedef struct {
    uint32_t last_node_handle; /*!< +0x00 last-resolved node handle */
    uint32_t mgr_ptr;          /*!< +0x04 report-mgr object pointer */
} bp_report_ptrs_t;

/* Backing storage lives in vcu_state.c; read by busproto and by the scheduler's
   dispatch tick. */
extern volatile bp_report_ptrs_t g_bp_rptr;

/* ------------------------------------------------------------------------- */
/* Secondary engine entry points.                                             */
/* ------------------------------------------------------------------------- */

/** \brief 0xFC report/log command handler. */
void bp_handle_fc(uint8_t sa, uint8_t da, const bp_frame_t* frame);

/**
 * \brief           Node / device registration handler
 * \param[in]       key: registration key (the Ninebot body of the app's 0xFD request)
 * \param[in]       len: key length in bytes
 */
void bp_node_register(const uint8_t* key, uint8_t len);

/**
 * \brief           Build a TP.CM connection-abort / broadcast frame
 * \note            id = \p dst | \p src<<8 | 0x18EC0000 (+bit29), dlc 8, data
 *                      FF FF FF FF FF 00 EF 00. Also used by the fast-service pacing step.
 * \return          1 when built, 0 if \p f is NULL
 */
int bp_frame_bcast(uint8_t dst, uint8_t src, bp_frame_t* f);

/**
 * \brief           Queue a multi-frame (J1939 TP) Ninebot report
 * \note            Finds or allocates the TX session for (\p sa, \p da) through the
 *                      session allocator bp_route_lookup and builds its transmit buffer with
 *                      bp_build_packet (header ctl/cmd/arg + payload + inverse checksum,
 *                      state BUILT). The slot-3 fast service then sends it (RTS, TP.DT).
 *                      The CAN report tunnel calls it as (m, 7, 1, src, dst, cmd, arg, p, n, 0).
 * \param[in]       m: comms manager (\ref g_bp_mgr)
 * \return          1 when a session was built, 0 when none was free or the build failed
 */
int bp_tp_send(bp_mgr_t* m, uint8_t prio, uint8_t ctl, uint8_t sa, uint8_t da, uint8_t cmd, uint8_t arg,
               const uint8_t* payload, uint16_t len, int scale);

/**
 * \brief           The comms manager
 * \note            The TP engine's manager and the slot-3 fast-service descriptor are one
 *                      object: {session array, u16 count, handler vtable}. It is a plain
 *                      busproto.c global, so the RX FSM here and the fast-service TX/pacing FSM
 *                      walk the same session array.
 */
extern bp_mgr_t g_bp_mgr; /* the comms/session manager (busproto.c) */

/* TP-manager ops installed by vcu_busproto_init (4-pointer handler table, installed
   by bp_manager_init via bp_handler_register). */

/** \brief Node-accept gate (ops[0]): 1 iff da is a VCU TP destination. */
int bp_node_accept(uint8_t sa, uint8_t da);

/** \brief Transfer-ack callback (ops[1]): no-op. */
void bp_on_ack(bp_session_t* s, uint8_t sa, uint8_t da, int phase);

/** \brief Data-ready callback (ops[2]): reassembled payload -> Ninebot dispatcher (CAN). */
void bp_on_data(bp_session_t* s, uint8_t sa, uint8_t da, const uint8_t* b, int n);

/**
 * \brief           Report-manager state-change notify (vtable +0x0C)
 * \note            Serializes the node's descriptor signals and emits a 0xFE state frame.
 *                      Reached as `mgr_ptr->vt[+0x0C](g_bp_rptr.last_node_handle)`; the arg is
 *                      the last-resolved node handle (NULL until a node is bound).
 *                      Home: busproto.c; called from scheduler.c's dispatch tick.
 * \param[in]       self: report node/self pointer (frame key @+0, descriptors @+4, count @+0x324)
 */
void bp_state_notify(void* self);

/**
 * \brief           Fast per-session service task
 * \note            Slot 3, period 1 tick = fastest. Thin wrapper that walks the session
 *                      manager (g_bp_mgr: base, count@+4, ctx@+8) and services every session
 *                      (stride 0x24) via bp_fast_service_entry: a per-session state machine
 *                      (+2 = state) that builds the initial TP.CM RTS (state 3), runs the
 *                      pacing/abort step (states 4/6/7/9/10 -> bp_fast_frame_cont) or streams the
 *                      body in 7-byte TP.DT chunks (state 5), emitting through the handler
 *                      vtable (+0x0C).
 */
void bp_task_fast_service(void);

/* ------------------------------------------------------------------------- */
/* Public API.                                                                 */
/* main.c wires these in; the per-session timeout/retransmit service is the    */
/* fast-service task (bp_task_fast_service, slot 3).                           */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Bring up the peripheral-bus TP engine
 * \note            Runs the report subsystem init + self-announce, then the comms-manager
 *                      init. Registers the five 300-byte RX buffers + their 0x24-byte session
 *                      descriptors, installs the handler table (5 sessions, emit vtable),
 *                      clears the shared node pairing/auth table, builds the report
 *                      manager, and enqueues the "device present" self-announce
 *                      (Ninebot src 0x16, dst 0x23, arg 0xD0).
 */
void vcu_busproto_init(void);

/**
 * \brief           Feed one inbound 29-bit extended-id CAN frame to the engine
 * \note            RX frame dispatcher. Routes by the id's PGN-format byte: TP.CM/RTS
 *                      (0xEC + control 0x10) allocates a session (via the node-accept callback
 *                      + the routing/pairing lookup bp_route_lookup) and runs the FSM on it;
 *                      every other TP.CM/TP.DT frame (0xEB/0xEC) is offered to all sessions, each
 *                      filtering on its own address pair. Call this from the ext-id CAN RX handler.
 * \param[in]       ext_frame: raw frame -- u32 id @+0, dlc @+4, 8 data bytes @+5
 *                      (the mailbox/`bp_frame_t` layout)
 */
void vcu_busproto_rx(const void* ext_frame);

#endif /* BUSPROTO_H */
