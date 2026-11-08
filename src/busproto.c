/**
 * \file            busproto.c
 * \brief           Peripheral-bus multi-packet transport engine (x3 VCU, wave3)
 * \note            The second Ninebot-style protocol engine: a J1939-style Transport Protocol
 *                      (TP) master for peripheral / BMS comms over the internal CAN 29-bit
 *                      extended-id bus. The pieces, roughly in call order:
 *
 *                        bp_manager_init        5 RX bufs + sessions
 *                        bp_route_lookup        session lookup by address pair
 *                        bp_dispatch            RX id 0xEB/0xEC router
 *                        bp_session_fsm         RTS/CTS/DT/EOMA/Abort FSM
 *                        bp_tp_send             TX session alloc + build
 *                        bp_build_packet        TX buffer + inverse checksum
 *                        bp_frame_bcast/cts/eoma  TP.CM abort / CTS 0x11 / EndOfMsgAck 0x13
 *                        bp_handle_fc           frame[5]==0xFC handler
 *                        bp_node_register       node/device registration
 *                        bp_subsystem_init      init + self-announce 0xD0
 *                        bp_report_mgr_init     report/log manager object
 *                        bp_rpt_send            report 5A A5 frame (via usart.c's ring)
 *                        bp_on_data             reassembled payload -> nb_reply_fc
 *
 *                      The J1939/BMS role names are descriptive labels; see busproto.h.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "busproto.h"
#include <stddef.h>
#include "can.h"
#include "ninebot.h"   /* nb_reply_fc, nb_commit_mode                          */
#include "usart.h"     /* vcu_usart_report_send, NB_ADDR_* / NB_CMD_*            */
#include "vcu_state.h" /* g_vs, g_uartctx, g_bp_rptr (shared per-block globals)  */

/* ------------------------------------------------------------------------- */
/* Engine-owned module statics (~1.7 KB total): sessions, buffers, the       */
/* manager g_bp_mgr and the handler vtable.                                    */
/* ------------------------------------------------------------------------- */

static bp_session_t g_bp_session[BP_SESSIONS];
static uint8_t g_bp_buf[BP_SESSIONS][BP_BUF_SIZE];
static bp_ctx_t g_bp_ctx;

/** \brief The comms/session manager. The slot-3 fast service (bp_task_fast_service)
           walks the same sessions through it. */
bp_mgr_t g_bp_mgr;

/* ------------------------------------------------------------------------- */
/* Report subsystem + pairing table (busproto-owned). What this unit shares    */
/* with other modules (g_vs, g_uartctx, g_bp_rptr) lives in vcu_state.h;       */
/* g_bp_rptr is also read by the scheduler's dispatch tick.                    */
/* ------------------------------------------------------------------------- */

/* The report subsystem's objects: descriptor, reply payload, tx context and the
   report manager. Nothing reaches across their boundaries, so they are separate
   globals. The node table is part of bp_report_mgr_t. */
static bp_pair_table_t g_bp_pair; /* node pairing/auth table */
/** \brief Report descriptor + its vfn. */
static volatile bp_report_vtable_t g_bp_rvtbl;
/* 0xFC-reply payload text; the 6 trailing bytes are 0. */
#define BP_RPT_REPLY_TEXT "abcdef0123"
/** \brief 0xFC-reply payload sent by the node-register path. */
static volatile uint8_t g_bp_reply_payload[BP_RPT_REPLY_LEN] = BP_RPT_REPLY_TEXT;

/** \brief Transmit context handed out by bp_tx_ctx_init: send fn + arg. */
static volatile struct {
    uint32_t send_fn; /*!< transport send function (written, never read) */
    uint32_t arg;     /*!< caller argument                          */
} g_bp_tx_ctx;

/** \brief The report-manager object. */
static volatile bp_report_mgr_t g_report_mgr;

/* g_report_mgr is also reached through the pointer cell g_bp_rptr.mgr_ptr; its
   embedded 7-pointer vtable is populated at init (bp_report_mgr_init). */

/* ------------------------------------------------------------------------- */
/* Cross-module APIs + the handler-table defaults.                            */
/* ------------------------------------------------------------------------- */

/* Cross-module APIs this engine calls:                                       */
/*   vcu_can_send_ext()      -- can.c     (CAN 29-bit ext-id transmit)         */
/*   nb_reply_fc()           -- ninebot.c (0xFC frame -> Ninebot dispatcher)   */
/*   vcu_usart_report_send() -- usart.c   (Ninebot ring enqueue)               */
/*   nb_commit_mode()        -- ninebot.c (drive/indicator mode commit)         */

/* TP RTS destinations the VCU accepts a session for; 0x16 / 0x23 are NB_ADDR_VCU /
   NB_ADDR_APP (usart.h). */
#define BP_DA_02 0x02u
#define BP_DA_07 0x07u
#define BP_DA_3D 0x3Du
#define BP_DA_3E 0x3Eu
#define BP_DA_3F 0x3Fu

/**
 * \brief           Node-accept gate (handler table ops[0])
 * \note            Called as accept(sa, da) on every TP.CM RTS (da = the PS byte of the id).
 *                      Accepts only the fixed destination set; \p sa is not inspected.
 */
int bp_node_accept(uint8_t sa, uint8_t da) {
    (void)sa;
    switch (da) {
        case BP_DA_02:
        case BP_DA_07:
        case NB_ADDR_VCU: /* 0x16 */
        case NB_ADDR_APP: /* 0x23 */
        case BP_DA_3D:
        case BP_DA_3E:
        case BP_DA_3F: return 1;
        default: return 0;
    }
}

/** \brief Transfer-ack callback (handler table ops[1]); does nothing. */
void bp_on_ack(bp_session_t* s, uint8_t sa, uint8_t da, int phase) {
    (void)s;
    (void)sa;
    (void)da;
    (void)phase;
}

/**
 * \brief           Data-ready callback (handler table ops[2])
 * \note            Called on TP EOMA as (s, sa, da, buf, total - 2). nb_reply_fc (ninebot.c)
 *                      frames the reassembled payload as a 0xFC Ninebot frame and hands it to
 *                      the Ninebot dispatcher with transport 1 (CAN).
 */
void bp_on_data(bp_session_t* s, uint8_t sa, uint8_t da, const uint8_t* b, int n) {
    nb_reply_fc((void*)s, sa, da, b, (char)n);
}

/**
 * \brief           Report pre-init context store
 * \note            Stores \p arg into the report pre-init context cell at +0x04 and returns
 *                      the cell base. Called as `bp_tx_ctx_init(0)` from the subsystem init; the
 *                      returned base is passed straight on as the descriptor of
 *                      \ref bp_report_vtable_init.
 */
uint32_t bp_tx_ctx_init(uint32_t arg) {
    /* The pre-init object:
       +0x00 = the report-descriptor method (the USART ring enqueue, vcu_usart_report_send),
       +0x04 = the runtime context arg. */
    g_bp_tx_ctx.send_fn = (uint32_t)(uintptr_t)&vcu_usart_report_send;
    g_bp_tx_ctx.arg = arg;
    return (uint32_t)(uintptr_t)&g_bp_tx_ctx;
}

/* ------------------------------------------------------------------------- */
/* The report-manager vtable bodies live in the "Report / log subsystem"     */
/* section further down and are wired into g_report_mgr's embedded vtable by  */
/* bp_report_mgr_init.                                                        */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Report descriptor vfn (stored at g_bp_rvtbl.vfn)
 * \note            A generic single-level vtable trampoline: with \p slot pointing at an
 *                      object pointer, it calls that object's vtable[0] with (a1,a2) and
 *                      returns 0. The dispatched leaf is wired up at run time, so the call is
 *                      NULL-guarded.
 */
typedef void (*bp_report_vcall_t)(uint32_t, uint32_t);

uint32_t bp_report_vfn(uint32_t* slot, uint32_t a1, uint32_t a2) {
    uint32_t obj;
    bp_report_vcall_t fn;

    if (slot == NULL) {
        return 0;
    }
    obj = *slot;
    if (obj == 0U) {
        return 0;
    }
    fn = *(bp_report_vcall_t*)(volatile void*)(uintptr_t)obj; /* vtable[0] = word at *slot (real addr) */
    if (fn != NULL) {
        fn(a1, a2);
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Local helpers.                                                             */
/* ------------------------------------------------------------------------- */

/** Byte-wise zero-fill. */
static void bp_block_clear(volatile uint8_t* p, uint32_t n) {
    while (n-- != 0U) {
        *p++ = 0;
    }
}

/**
 * \brief           Compose a 29-bit extended id from an address pair
 * \note            Shared by all TP.CM frame builders: keep the top bits of the prior id
 *                      word, merge in (dst | src<<8 | TP.CM base) masked to 29 bits, and
 *                      set the extended-id marker.
 */
static uint32_t bp_compose_extid(uint8_t dst, uint8_t src, uint32_t prev_id) {
    uint32_t pair = ((uint32_t)dst | ((uint32_t)src << 8) | BP_EXTID_CM_BASE) & BP_EXTID_MASK;

    return (prev_id & BP_EXTID_KEEP) | pair | BP_EXTID_FLAG;
}

/** Number of 7-byte TP.DT packets needed to carry \p nbytes (ceil division). */
static uint32_t bp_packet_count(uint32_t nbytes) { return (nbytes + (BP_TP_CHUNK - 1U)) / BP_TP_CHUNK; }

/* ------------------------------------------------------------------------- */
/* TX frame builders (29-bit ext-id, dlc=8).                                  */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Build a TP.CM broadcast/abort template
 * \note            All-0xFF control payload with the 0xEF00 PGN; used for the
 *                      connection-abort / error path.
 */
int bp_frame_bcast(uint8_t dst, uint8_t src, bp_frame_t* f) {
    if (f == NULL) {
        return 0;
    }
    f->id = bp_compose_extid(dst, src, f->id);
    f->dlc = 8;
    f->data[0] = BP_TP_ABORT;
    f->data[1] = BP_TP_PAD;
    f->data[2] = BP_TP_PAD;
    f->data[3] = BP_TP_PAD;
    f->data[4] = BP_TP_PAD;
    f->data[5] = BP_TP_PGN_LO;
    f->data[6] = BP_TP_PGN_MID;
    f->data[7] = BP_TP_PGN_HI;
    return 1;
}

/**
 * \brief           Build a TP.CM CTS frame, control 0x11
 * \param[in]       npkt: number of packets the peer may send now
 * \param[in]       next: next packet number to send
 */
static int bp_frame_cts(uint8_t dst, uint8_t src, uint8_t npkt, uint8_t next, bp_frame_t* f) {
    if (f == NULL) {
        return 0;
    }
    f->id = bp_compose_extid(dst, src, f->id);
    f->dlc = 8;
    f->data[0] = BP_TP_CTS;
    f->data[1] = npkt;
    f->data[2] = next;
    f->data[3] = BP_TP_PAD;
    f->data[4] = BP_TP_PAD;
    f->data[5] = BP_TP_PGN_LO;
    f->data[6] = BP_TP_PGN_MID;
    f->data[7] = BP_TP_PGN_HI;
    return 1;
}

/**
 * \brief           Build a TP.CM EndOfMsgAck frame, control 0x13
 * \param[in]       total: total transferred byte count
 */
static int bp_frame_eoma(uint8_t dst, uint8_t src, int total, bp_frame_t* f) {
    if (f == NULL) {
        return 0;
    }
    f->id = bp_compose_extid(dst, src, f->id);
    f->dlc = 8;
    f->data[0] = BP_TP_EOMA;
    f->data[1] = (uint8_t)total;
    f->data[2] = (uint8_t)((uint32_t)total >> 8);
    f->data[3] = (uint8_t)bp_packet_count((uint32_t)total);
    f->data[4] = BP_TP_PAD;
    f->data[5] = BP_TP_PGN_LO;
    f->data[6] = BP_TP_PGN_MID;
    f->data[7] = BP_TP_PGN_HI;
    return 1;
}

/**
 * \brief           Build a TP transmit buffer with trailing inverse checksum
 * \note            Lays out header[3] + payload in the session's own buffer, appends a
 *                      16-bit one's-complement (inverse) additive checksum, sets state BUILT
 *                      and the deadline. Returns 0 if the buffer cannot hold payload+5 bytes or
 *                      the payload exceeds the 0x6F9 cap. \p p2 is unused.
 *                      The session's address pair is (\p sa, \p da): the same +0x00/+0x01
 *                      pair the allocator bp_route_lookup matches, the RX FSM checks CTS/EOMA
 *                      against, and the TX FSM uses as the frame SA/DA.
 */
static int bp_build_packet(bp_session_t* s, uint8_t p2, uint8_t h0, uint8_t sa, uint8_t da, uint8_t h1, uint8_t h2,
                           const uint8_t* payload, uint16_t len, int scale) {
    uint16_t sum;
    uint16_t i;

    (void)p2;
    if (s == NULL || payload == NULL || s->cap < (uint32_t)len + 5U || len > BP_MAX_PAYLD) {
        return 0;
    }
    s->state = BP_ST_BUILT;
    s->addr_a = sa;
    s->addr_b = da;
    s->buf[0] = h0;
    s->buf[1] = h1;
    s->buf[2] = h2;

    sum = (uint16_t)(h1 + h0 + h2);
    for (i = 0; i < len; i++) {
        sum = (uint16_t)((sum & 0xFFFFU) + payload[i]);
        s->buf[i + 3U] = payload[i];
    }
    sum = (uint16_t)~(sum & 0xFFFFU);
    s->buf[len + 3U] = (uint8_t)sum;
    s->buf[len + 4U] = (uint8_t)(sum >> 8);

    s->total = (uint16_t)(len + 5U);
    s->retry = 0;
    s->deadline = (int16_t)(((uint32_t)s->period + (uint32_t)scale - 1U) / s->period);
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Init: channel register / handler register / manager + subsystem init.      */
/* ------------------------------------------------------------------------- */

/** Register one RX buffer / channel descriptor. */
static int bp_channel_register(bp_session_t* s, uint16_t period, uint8_t* buf, uint16_t cap, void* ctx) {
    if (s == NULL || buf == NULL || period == 0U) {
        return 0;
    }
    s->buf = buf;
    s->cap = cap;
    s->period = period;
    s->state = BP_ST_IDLE;
    s->ctx = ctx;
    return 1;
}

/**
 * \brief           Install the handler table into the manager
 * \note            Requires a valid session base, non-zero count, and a vtable whose
 *                      node-accept (+0x00) and emit (+0x0C) entries are both set.
 */
static int bp_handler_register(bp_mgr_t* m, bp_session_t* base, uint16_t count, bp_ctx_t* ctx) {
    if (m != NULL && base != NULL && count != 0U && ctx != NULL && ctx->node_accept != NULL && ctx->emit != NULL) {
        m->base = base;
        m->count = count;
        m->ctx = ctx;
        return 1;
    }
    return 0;
}

/** Clear the shared node pairing/auth table (two banks + counters). */
static void bp_pair_table_clear(void) {
    volatile bp_pair_table_t* t = &g_bp_pair;
    uint32_t i;

    /* Order: bank-B counters + slots, then bank-A. The key slots are cleared via
       explicit (base + offset) byte math rather than t->bank_x[i]; the field names
       still drive the offset/stride. */
    t->count_b1 = 0;
    t->count_b0 = 0;
    for (i = 0; i < BP_PAIR_SLOTS; i++) {
        bp_block_clear((volatile uint8_t*)&g_bp_pair + offsetof(bp_pair_table_t, bank_b) + i * sizeof(bp_pair_slot_t),
                       BP_PAIR_KEY_LEN);
    }
    t->count_a1 = 0;
    t->count_a0 = 0;
    for (i = 0; i < BP_PAIR_SLOTS; i++) {
        bp_block_clear((volatile uint8_t*)&g_bp_pair + offsetof(bp_pair_table_t, bank_a) + i * sizeof(bp_pair_slot_t),
                       BP_PAIR_KEY_LEN);
    }
}

/**
 * \brief           Comms-manager init: 5 RX buffers + sessions + pairing table
 * \note            Registers 5 channels (300-byte buffers, 0x24-byte descriptors, period 1),
 *                      installs the handler table, then clears the shared node pairing/auth
 *                      table.
 */
static void bp_manager_init(void) {
    uint32_t i;

    for (i = 0; i < BP_SESSIONS; i++) {
        bp_channel_register(&g_bp_session[i], 1, g_bp_buf[i], BP_BUF_SIZE, NULL);
    }
    bp_handler_register(&g_bp_mgr, g_bp_session, BP_SESSIONS, &g_bp_ctx);
    bp_pair_table_clear();
}

/* ------------------------------------------------------------------------- */
/* Routing / pairing lookup.                                                  */
/* ------------------------------------------------------------------------- */

/** Match an active session for the forward address pair (sa->da). */
static bp_session_t* bp_find_forward(bp_mgr_t* m, uint8_t sa, uint8_t da) {
    uint16_t i;

    for (i = 0; i < m->count; i++) {
        if (m->base[i].addr_a == sa && m->base[i].addr_b == da && m->base[i].state != BP_ST_IDLE) {
            return &m->base[i];
        }
    }
    return NULL;
}

/** Take the first idle (free) session slot. */
static bp_session_t* bp_find_free(bp_mgr_t* m) {
    uint16_t i;

    for (i = 0; i < m->count; i++) {
        if (m->base[i].state == BP_ST_IDLE) {
            return &m->base[i];
        }
    }
    return NULL;
}

/**
 * \brief           Find or allocate a session for an address pair
 * \note            For \p alloc==0: reject if the reverse pair is already busy, then match
 *                      the forward pair, else take the first free slot. For \p alloc!=0:
 *                      match the forward pair, else take the first free slot.
 */
static bp_session_t* bp_route_lookup(bp_mgr_t* m, uint8_t sa, uint8_t da, int alloc) {
    bp_session_t* s;
    uint16_t i;

    if (alloc == 0) {
        /* Reject when the reverse pair (da->sa) already owns a busy session. */
        for (i = 0; i < m->count; i++) {
            if (m->base[i].addr_a == da && m->base[i].addr_b == sa && m->base[i].state != BP_ST_IDLE) {
                return NULL;
            }
        }
    }

    s = bp_find_forward(m, sa, da);
    if (s != NULL) {
        return s;
    }
    return bp_find_free(m);
}

/* Multi-frame report entry: allocate the (sa,da) TX session (bp_route_lookup,
   alloc 0) and build its buffer (bp_build_packet). */
int bp_tp_send(bp_mgr_t* m, uint8_t prio, uint8_t ctl, uint8_t sa, uint8_t da, uint8_t cmd, uint8_t arg,
               const uint8_t* payload, uint16_t len, int scale) {
    bp_session_t* s;

    if (m == NULL) {
        return 0;
    }
    s = bp_route_lookup(m, sa, da, 0);
    if (s == NULL) {
        return 0;
    }
    return bp_build_packet(s, prio, ctl, sa, da, cmd, arg, payload, len, scale);
}

/* ------------------------------------------------------------------------- */
/* RX reassembly / TP FSM.                                                    */
/* ------------------------------------------------------------------------- */

/** Extract the transported PGN (data[5..7]) from a TP.CM frame. */
static uint32_t bp_frame_pgn(const bp_frame_t* m) {
    return ((uint32_t)m->data[6] << 8) | ((uint32_t)m->data[7] << 16) | (uint32_t)m->data[5];
}

/**
 * \brief           Handle an inbound TP.CM RTS (open an RX session)
 * \note            Accepts only when the session is IDLE or the frame matches this
 *                      session's existing address pair. On a size/packet-count mismatch
 *                      the session is idled and a Conn-Abort is emitted; otherwise it
 *                      replies CTS and enters RX_DATA.
 */
static void bp_fsm_rx_rts(bp_session_t* s, const bp_frame_t* m, bp_ctx_t* ctx, uint32_t sa, uint32_t da) {
    bp_frame_t out = {0};
    uint8_t total;
    uint32_t npkt;

    if (s->state != BP_ST_IDLE && !(s->addr_b == da && s->addr_a == sa)) {
        return;
    }

    s->addr_b = (uint8_t)(m->id >> 8);
    s->addr_a = (uint8_t)m->id;
    /* Only the low byte of the RTS size (data[1]) is taken. A transfer > 255 B then
       fails the (total+6)/7 == npkt check and is aborted, so this fails safe. */
    total = m->data[1];
    s->total = total;
    s->npkt = m->data[3];
    s->pos = 0;
    s->seq = 0;
    s->csum = 0;

    npkt = bp_packet_count((uint32_t)total);
    if (s->cap < (uint32_t)total || npkt != (uint32_t)s->npkt) {
        s->state = BP_ST_IDLE; /* RTS reject/abort path idles the session */
        bp_frame_bcast((uint8_t)da, (uint8_t)sa, &out);
        ctx->emit(s, &out);
    } else {
        bp_frame_cts((uint8_t)da, (uint8_t)sa, (uint8_t)npkt, 1, &out);
        ctx->emit(s, &out);
        s->state = BP_ST_RX_DATA;
        s->tick = 0;
        s->timeout = (int16_t)(BP_TMO_RTS / s->period);
    }
}

/** Advance the TX side on an inbound CTS (TP.CM control 0x11). */
static void bp_fsm_tx_cts(bp_session_t* s, const bp_frame_t* m) {
    if (s->state == BP_ST_TX_RTS) {
        if (m->data[1] == 0) {
            s->state = BP_ST_TX_WAIT;
            s->tick = 0;
            s->timeout = (int16_t)(BP_TMO_CTS / s->period);
        }
    } else if ((s->state == BP_ST_TX_CTS || s->state == BP_ST_TX_WAIT) && m->data[1] != 0 && m->data[2] != 0) {
        s->npkt = m->data[2];
        s->pos = (uint16_t)((m->data[2] - 1) * BP_TP_CHUNK);
        s->win_end = (uint16_t)(s->pos + (uint16_t)(m->data[1] * BP_TP_CHUNK));
        s->state = BP_ST_TX_SEND;
    }
}

/**
 * \brief           Append one inbound TP.DT data packet to the reassembly buffer
 * \note            Copies up to 7 bytes, accumulates the additive checksum (excluding the
 *                      trailing 2 bytes), and on completion verifies the inverse checksum
 *                      and replies EndOfMsgAck + fires the data-ready callback. An
 *                      out-of-sequence packet idles the session and aborts.
 */
static void bp_fsm_rx_dt(bp_session_t* s, const bp_frame_t* m, bp_ctx_t* ctx, uint32_t sa, uint32_t da) {
    bp_frame_t out = {0};
    uint32_t avail = (uint32_t)(s->total - s->pos) & 0xFFFFU;
    uint32_t i;

    if (avail > BP_TP_CHUNK) {
        avail = BP_TP_CHUNK;
    }

    if ((uint32_t)s->seq + 1U != (uint32_t)m->data[0]) {
        s->state = BP_ST_IDLE;
        bp_frame_bcast((uint8_t)da, (uint8_t)sa, &out);
        ctx->emit(s, &out);
        return;
    }

    for (i = 0; i < avail; i++) {
        uint8_t b = m->data[i + 1U];

        s->buf[s->pos] = b;
        if ((int)(uint32_t)s->pos < (int)(s->total - 2)) {
            s->csum = (uint16_t)(b + s->csum);
        }
        s->pos++;
    }
    s->seq++;

    if (s->pos < s->total) {
        s->tick = 0;
        s->timeout = (int16_t)(BP_TMO_DT / s->period);
    } else {
        uint16_t inv = (uint16_t)~s->csum;

        s->csum = inv;
        s->state = BP_ST_IDLE;
        if ((uint16_t)((uint16_t)s->buf[s->total - 2] + (uint16_t)(s->buf[s->total - 1] * 0x100)) == inv) {
            bp_frame_eoma((uint8_t)da, (uint8_t)sa, (int)s->total, &out); /* total = s->total */
            ctx->emit(s, &out);
            if (ctx->on_data != NULL) {
                ctx->on_data(s, (uint8_t)sa, (uint8_t)da, s->buf, s->total - 2);
            }
        } else {
            bp_frame_bcast((uint8_t)da, (uint8_t)sa, &out);
            ctx->emit(s, &out);
        }
    }
}

/**
 * \brief           Per-session TP state machine
 * \note            Drives one session on one inbound frame. TP.CM (0xEC): RTS opens a
 *                      session and replies CTS (or aborts); CTS advances the TX side;
 *                      EndOfMsgAck completes a TX; Abort idles. TP.DT (0xEB): reassembles
 *                      7-byte chunks.
 */
static void bp_session_fsm(bp_session_t* s, const bp_frame_t* m, bp_ctx_t* ctx) {
    uint32_t pf;
    uint32_t sa;
    uint32_t da;
    uint8_t ctrl;

    if (s == NULL || m == NULL) {
        return;
    }
    pf = (m->id & 0xFFFFFFU) >> 16;
    da = (m->id & 0xFFFFU) >> 8;
    sa = m->id & 0xFFU;

    if (pf == BP_PF_CM) {
        if (bp_frame_pgn(m) != BP_PGN_EF00) {
            return;
        }
        ctrl = m->data[0];
        if (ctrl == BP_TP_RTS) {
            bp_fsm_rx_rts(s, m, ctx, sa, da);
        } else if (s->addr_a == da && s->addr_b == sa) {
            /* TX-side replies are addressed back to us (reverse pair). */
            if (ctrl == BP_TP_CTS) {
                bp_fsm_tx_cts(s, m);
            } else if (ctrl == BP_TP_EOMA) {
                /* Only the low byte of the acked size (data[1]) is compared, as for RTS. */
                if (s->state == BP_ST_TX_DONE && s->pos == (uint16_t)m->data[1]) {
                    s->state = BP_ST_IDLE;
                    if (ctx->on_ack != NULL) {
                        ctx->on_ack(s, (uint8_t)sa, (uint8_t)da, BP_ACK_PHASE_EOMA);
                    }
                }
            } else if (ctrl == BP_TP_ABORT) {
                s->state = BP_ST_IDLE;
            }
        }
    } else if (pf == BP_PF_DT && s->addr_a == sa && s->addr_b == da && s->state == BP_ST_RX_DATA) {
        bp_fsm_rx_dt(s, m, ctx, sa, da);
    }
}

/* ------------------------------------------------------------------------- */
/* RX dispatcher.                                                             */
/* ------------------------------------------------------------------------- */

static void bp_dispatch(bp_mgr_t* m, const bp_frame_t* frame) {
    uint32_t pf = (frame->id & 0xFFFFFFU) >> 16;
    uint32_t da = (frame->id & 0xFFFFU) >> 8;
    uint32_t sa = frame->id & 0xFFU;
    uint16_t i;

    if (pf != BP_PF_CM && pf != BP_PF_DT) {
        return;
    }

    if (pf == BP_PF_CM && frame->data[0] == BP_TP_RTS) {
        /* A new RTS: gate on the node-accept callback, then allocate a session. */
        if (m->ctx->node_accept((uint8_t)sa, (uint8_t)da) == 1) {
            bp_session_t* s = bp_route_lookup(m, (uint8_t)sa, (uint8_t)da, 1);

            if (s != NULL) {
                bp_session_fsm(s, frame, m->ctx);
            }
        }
    } else {
        /* Every other TP.CM/TP.DT frame is offered to all sessions. */
        for (i = 0; i < m->count; i++) {
            bp_session_fsm(&m->base[i], frame, m->ctx);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Emit vtable backing: the real CAN ext-id transport seam.                   */
/* ------------------------------------------------------------------------- */

/** Emit (ctx[+0x0C]): push the built frame onto the CAN ext-id bus. */
int bp_emit(bp_session_t* s, const bp_frame_t* f) {
    (void)s;
    vcu_can_send_ext(f->id & BP_EXTID_MASK, f->data, 8);
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Report / log subsystem.                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* Report-manager vtable bodies and their helpers. The frame build / ring copy */
/* goes through usart.c (bp_rpt_send).                                        */
/* ------------------------------------------------------------------------- */

/**
 * \brief           LSB-first little-endian bit-field insert
 * \param[in,out]   dst: destination byte span (nbytes long)
 * \param[in]       nbytes: bytes of \p dst that may be touched
 * \param[in]       bitoff: starting bit offset within dst (0..7 of dst[0] from bp_state_serialize)
 * \param[in]       width: field width in bits
 * \param[in]       value: value whose low \p width bits are packed
 * \note            Reads `1 + width/8` bytes little-endian into a word, clears
 *                      `mask << (bitoff & 7)`, ORs `(v & mask) << (bitoff & 7)` and writes the
 *                      bytes back little-endian, so value bit 0 lands at bit \p bitoff.
 */
static void bp_bitfield_insert(volatile uint8_t* dst, uint8_t nbytes, uint8_t bitoff, uint8_t width, uint32_t value) {
    uint32_t span = 1u + ((uint32_t)width >> 3); /* bytes spanned */
    uint32_t base = (uint32_t)bitoff >> 3;       /* first byte (0 from bp_state_serialize) */
    uint32_t sh = (uint32_t)bitoff & 7u;
    uint32_t mask = (width >= 32u) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    uint32_t acc = 0u;
    uint32_t i;

    for (i = 0u; (i < span) && ((i + base) < nbytes); i++) {
        acc |= (uint32_t)dst[base + i] << (8u * i);
    }
    acc = (acc & ~(mask << sh)) | ((value & mask) << sh);
    for (i = 0u; (i < span) && ((i + base) < nbytes); i++) {
        dst[base + i] = (uint8_t)(acc >> (8u * i));
    }
}

/**
 * \brief           Frame + enqueue one report-manager frame on the mgr's send object
 * \note            Every emit (0xFE state, 0xFC reply, 0xFD ack, 0xFD list) sets the per-emit
 *                      cmd/arg header bytes, then sends nb_src/nb_dst/cmd/arg + payload through
 *                      the usart.c ring enqueue (\ref vcu_usart_report_send); send_obj is the
 *                      ring ctx. The enqueue frames the 5A A5 frame with its checksum and rejects
 *                      a body > 0xF2 bytes; no report path builds one.
 */
static void bp_rpt_send(uint8_t cmd, uint8_t arg, const uint8_t* payload, uint8_t plen) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);

    m->nb_cmd = cmd;
    m->nb_arg = arg;
    if (m->send_obj == 0u) {
        return;
    }
    vcu_usart_report_send((volatile uint8_t*)(uintptr_t)m->send_obj, m->nb_src, m->nb_dst, plen, m->nb_cmd, m->nb_arg,
                          payload);
}

/**
 * \brief           Value of a report signal
 * \param[in]       desc: the descriptor
 * \return          the u32 at g_scr halfword `desc->scr_idx`
 * \note            Built from two halfwords, so no unaligned word access is needed. Only the
 *                      low `width` (<= 16) bits are packed.
 */
static uint32_t bp_sig_value(const bp_sig_desc_t* desc) {
    uint32_t idx = desc->scr_idx;

    return (uint32_t)g_scr.reg[idx] | ((uint32_t)g_scr.reg[idx + 1u] << 16);
}

/**
 * \brief           Serialize a node's descriptor signals
 * \note            dst[0]=key, then each descriptor's value packed as a `width`-bit field,
 *                      LSB-first little-endian bit cursor. The records are the descriptor
 *                      indices bp_resolve_records stored.
 */
static int bp_state_serialize(uint8_t key, const bp_sig_desc_t* tbl, const volatile uint8_t* arr, int n,
                              volatile uint8_t* dst, volatile uint8_t* out_cnt) {
    uint32_t bitpos;
    int i;

    if (tbl == NULL || arr == NULL || n == 0 || dst == NULL || out_cnt == NULL) {
        return -1;
    }
    dst[0] = key;
    bitpos = 8; /* pack after byte 0 */
    for (i = 0; i < n; i++) {
        const bp_sig_desc_t* d = &tbl[arr[i]];
        uint8_t bitoff = (uint8_t)(bitpos & 7u);
        uint8_t byteoff = (uint8_t)((bitpos >> 3) & 0xFFu);
        uint8_t width = d->width;
        uint8_t nbytes = (uint8_t)(((uint32_t)width + bitoff + 7u) / 8u);
        uint32_t v = bp_sig_value(d);

        bp_bitfield_insert(dst + byteoff, nbytes, bitoff, width, v);
        bitpos += width;
    }
    *out_cnt = (uint8_t)(*out_cnt + (bitpos >> 3));
    if ((bitpos & 7u) != 0u) {
        (*out_cnt)++;
    }
    return 0;
}

/**
 * \brief           Report-manager state notify -> 0xFE state frame
 * \note            vtable +0x0C. Serializes the node's descriptors then emits a 0xFE frame.
 *                      self is the report node (g_bp_rptr.last_node_handle); NULL until a node
 *                      is bound.
 */
void bp_state_notify(void* self) {
    const volatile bp_report_node_t* node = (const volatile bp_report_node_t*)self;
    uint8_t buf[0x100];
    uint8_t cnt = 0;

    if (node == NULL) {
        return;
    }
    /* Clear the frame scratch first: the bit packer read-modify-writes each byte, so
       the bits after the last field must start at 0. */
    bp_block_clear(buf, sizeof buf);
    bp_state_serialize(node->id, (const bp_sig_desc_t*)(uintptr_t)g_report_mgr.desc_table, node->rec,
                       (int)node->rec_count, buf, &cnt);
    bp_rpt_send(BP_RPT_CMD_FE, BP_RPT_ARG_FE, buf, cnt);
}

/**
 * \brief           Emit a 0xFC report frame (vtable +0x04)
 */
static void bp_rpt_emit(void* self, int arg, const uint8_t* payload, uint16_t plen) {
    (void)self;
    (void)arg;
    bp_rpt_send(BP_RPT_CMD_FC, BP_RPT_ARG_FC, payload, (uint8_t)plen);
}

/**
 * \brief           Emit a 0xFD 1-byte ack frame (vtable +0x08)
 * \return          0 on send, -1 if \p ok is 0
 */
static int bp_rpt_emit_ack(uint8_t code, int ok) {
    if (ok == 0) {
        return -1;
    }
    bp_rpt_send(BP_RPT_CMD_FD_ACK, BP_RPT_ARG_FD, &code, 1);
    return 0;
}

/** Find a descriptor by its 32-bit key. */
static int bp_desc_find(const bp_sig_desc_t* tbl, uint16_t n, uint32_t key) {
    int i;

    for (i = 0; i < (int)n; i++) {
        if (tbl[i].key == key) {
            return i;
        }
    }
    return -1;
}

/**
 * \brief           Resolve a node's subscribe records against the descriptor table
 * \note            key = {id, period, N x {u32 key LE, u8 width}}. Every record that resolves is copied
 *                      as its index to out_recs and counted. Any miss or width mismatch makes the result -1,
 *                      but the loop goes on. id/period are stored before the %5 check.
 */
static int bp_resolve_records(const uint8_t* key, uint32_t id, uint32_t desc_table, uint16_t desc_count,
                              volatile uint8_t* out_id, volatile uint16_t* out_seq, volatile uint8_t* out_recs,
                              volatile uint8_t* out_cnt) {
    const bp_sig_desc_t* tbl = (const bp_sig_desc_t*)(uintptr_t)desc_table;
    uint32_t nrec;
    uint32_t r;
    uint32_t off = 2u;
    uint32_t k = 0u;
    int rc = 0;

    if (key == NULL || id < 4u || tbl == NULL || desc_count == 0u || out_id == NULL || out_seq == NULL
        || out_recs == NULL || out_cnt == NULL) {
        return -1;
    }
    *out_id = key[0];
    *out_seq = key[1]; /* period byte; the caller zeroed the halfword first */
    if (((id - 2u) % 5u) != 0u) {
        return -1;
    }
    nrec = (uint8_t)((id - 2u) / 5u);
    for (r = 0u; r < nrec; r++, off += 5u) {
        uint32_t want = (uint32_t)key[off] | ((uint32_t)key[off + 1u] << 8) | ((uint32_t)key[off + 2u] << 16)
                        | ((uint32_t)key[off + 3u] << 24);
        int j = bp_desc_find(tbl, desc_count, want);

        if ((j < 0) || (tbl[j].width != key[off + 4u])) {
            rc = -1;
            continue;
        }
        out_recs[k] = (uint8_t)j;
        k++;
        (*out_cnt)++;
    }
    return rc;
}

/**
 * \brief           Build+send a 0xFD list/ack frame
 * \note            cmd 5, arg 0xFD. With no list it packs scratch[0..1] (key+status)
 *                      as a 2-byte payload; else it appends \p count 4-byte records (opaque
 *                      passthrough). Sends via \ref bp_rpt_send.
 */
static void bp_rpt_emit_list(const uint8_t* scratch, const bp_sig_desc_t* tbl, const volatile uint8_t* recs,
                             uint8_t count) {
    uint8_t payload[0x100];
    uint8_t plen;

    payload[0] = scratch[0];
    payload[1] = scratch[1];
    plen = 2;
    if (tbl != NULL && recs != NULL && count != 0u) {
        uint32_t i;

        for (i = 0; i < count && plen <= (uint8_t)(sizeof payload - 4u); i++) {
            uint32_t b;

            for (b = 0; b < 4u; b++) { /* the record's 32-bit key, little-endian */
                payload[plen++] = (uint8_t)(tbl[recs[i]].key >> (8u * b));
            }
        }
    }
    bp_rpt_send(BP_RPT_CMD_FD_LIST, BP_RPT_ARG_FD, payload, plen);
}

/** Find a node by id, clearing its scratch; else allocate a new slot at the tail. */
static volatile bp_report_node_t* bp_find_or_alloc_node(uint8_t id) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);
    uint32_t i;
    volatile bp_report_node_t* node;

    for (i = 0; i < m->node_count; i++) {
        node = &m->node[i];
        if (node->id == id) {
            node->rec_count = 0;
            return node;
        }
    }
    node = &m->node[m->node_count];
    node->id = id;
    m->node_count = (uint8_t)(m->node_count + 1u);
    return node;
}

/**
 * \brief           Resolve / broadcast a node key then emit the 0xFD list
 * \note            vtable +0x00. key[0]==0xFF broadcasts the node objects' state-notify slot;
 *                      otherwise find/alloc the node, resolve its descriptor records, scale the
 *                      sequence and emit the 0xFD list. Returns key[0], or -1 on error.
 */
static int bp_rpt_resolve(const uint8_t* key, uint32_t id) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);
    uint8_t scratch[0x108];
    uint8_t k0;
    uint32_t i;
    volatile bp_report_node_t* node;
    int rc;

    if (key == NULL || id <= 2u) {
        return -1;
    }
    bp_block_clear(scratch, sizeof scratch);
    k0 = key[0];
    if (k0 == 0xFFu) { /* broadcast: fire each registered node object's state-notify slot */
        for (i = 0; i < m->node_count; i++) {
            const volatile bp_node_obj_t* obj = m->node[i].obj;

            if (obj != NULL) {
                bp_vt_state_notify_fn fn = obj->state_notify; /* word at obj+0x0C */

                if (fn != NULL) {
                    fn((void*)obj);
                }
            }
        }
    }
    scratch[0] = k0;
    if (m->node_count >= BP_NODE_MAX) { /* table full */
        scratch[1] = 1;
        bp_rpt_emit_list(scratch, NULL, NULL, 0);
        return -1;
    }
    node = bp_find_or_alloc_node(k0);
    node->seq = 0;
    rc = bp_resolve_records(key, id, m->desc_table, m->desc_count, &node->id, &node->seq, node->rec, &node->rec_count);
    if (rc != 0) {
        scratch[1] = 1;
        bp_rpt_emit_list(scratch, (const bp_sig_desc_t*)(uintptr_t)m->desc_table, node->rec, node->rec_count);
        return -1;
    }
    node->seq = (uint16_t)((uint32_t)node->seq * BP_RMGR_SEQ_SCALE);
    node->resolved = 1;
    scratch[1] = 0;
    bp_rpt_emit_list(scratch, NULL, NULL, 0);
    return (int)k0;
}

/**
 * \brief           Register a node OBJECT by id (vtable +0x10)
 * \note            Binds \p obj into the matching node (node.obj) and fires the object's
 *                      +0x04 word (init).
 */
static int bp_rpt_register_obj(uint8_t id, void* obj) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);
    uint32_t i;

    for (i = 0; i < m->node_count; i++) {
        volatile bp_report_node_t* node = &m->node[i];

        if (node->id == id) {
            const volatile bp_node_obj_t* o = (const volatile bp_node_obj_t*)obj;

            node->obj = o;
            if (o != NULL) {
                bp_obj_init_fn fn = o->init; /* word at obj+0x04 */

                if (fn != NULL) {
                    fn(obj);
                }
            }
            return 0;
        }
    }
    return -1;
}

/** Report notify: store a status/flag byte into the mgr (vtable +0x14). */
static void bp_rpt_notify(uint8_t flag) { g_report_mgr.flag = flag; /* mgr +0x1C */ }

/** Lookup a node DESCRIPTOR handle by id (vtable +0x18). */
static void* bp_rpt_register_desc(uint8_t id) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);
    uint32_t i;

    for (i = 0; i < m->node_count; i++) {
        volatile bp_report_node_t* node = &m->node[i];

        if (node->id == id) {
            return (void*)node; /* node handle */
        }
    }
    return NULL;
}

/** \brief g_scr halfword index of a named vcu_scr_t cell. */
#define BP_SCR_IDX(field) ((uint8_t)(offsetof(vcu_scr_t, field) / 2u))

/* Report signal descriptor table (38 x 8 B). Entries [12]/[35]/[36] are the ride-assist flags */
/* (climb boost, over-speed regen, wobble warning).                                              */
static const bp_sig_desc_t k_bp_sig_desc[BP_SIG_DESC_COUNT] = {
    {.key = 0x7E765289u, .width = 16u, .scr_idx = BP_SCR_IDX(power_half)},         /* [ 0] g_scr+0x02 */
    {.key = 0xDDAA7A89u, .width = 16u, .scr_idx = BP_SCR_IDX(range_disp)},         /* [ 1] g_scr+0x18 */
    {.key = 0xAC440AC5u, .width = 16u, .scr_idx = BP_SCR_IDX(distance_disp)},      /* [ 2] g_scr+0x1E */
    {.key = 0xA3DDD1B3u, .width = 16u, .scr_idx = BP_SCR_IDX(dtc_mask0)},          /* [ 3] g_scr+0x24 */
    {.key = 0xA3DDD1B2u, .width = 16u, .scr_idx = BP_SCR_IDX(dtc_mask1)},          /* [ 4] g_scr+0x26 */
    {.key = 0xA3DDD1ADu, .width = 16u, .scr_idx = BP_SCR_IDX(dtc_mask2)},          /* [ 5] g_scr+0x28 */
    {.key = 0xA3DDD1ACu, .width = 16u, .scr_idx = BP_SCR_IDX(event_mask_lo)},      /* [ 6] g_scr+0x2A */
    {.key = 0xA3DDD1AFu, .width = 16u, .scr_idx = BP_SCR_IDX(event_mask_hi)},      /* [ 7] g_scr+0x2C */
    {.key = 0x6FE56C44u, .width = 16u, .scr_idx = BP_SCR_IDX(gest_busy)},          /* [ 8] g_scr+0x40 */
    {.key = 0xAD4A48AEu, .width = 16u, .scr_idx = BP_SCR_IDX(canrx_423)},          /* [ 9] g_scr+0x20 */
    {.key = 0x9096F99Eu, .width = 16u, .scr_idx = BP_SCR_IDX(ridetime_disp)},      /* [10] g_scr+0x1A */
    {.key = 0x9C9B65E4u, .width = 16u, .scr_idx = BP_SCR_IDX(lock_status_mirror)}, /* [11] g_scr+0x44 */
    {.key = 0x229A0092u, .width = 16u, .scr_idx = BP_SCR_IDX(climb_boost_on)},     /* [12] g_scr+0x92 */
    {.key = 0xA4F6D064u, .width = 8u, .scr_idx = BP_SCR_IDX(soc_combined)},        /* [13] g_scr+0x50 */
    {.key = 0x616A4512u, .width = 8u, .scr_idx = BP_SCR_IDX(soc_report)},          /* [14] g_scr+0x2E */
    {.key = 0xB3E2E070u, .width = 8u, .scr_idx = BP_SCR_IDX(soc_raw_nz)},          /* [15] g_scr+0x0C */
    {.key = 0xE799388Au, .width = 8u, .scr_idx = BP_SCR_IDX(bat2_soc)},            /* [16] g_scr+0x0E */
    {.key = 0x8689B2DAu, .width = 4u, .scr_idx = BP_SCR_IDX(telem_3c)},            /* [17] g_scr+0x3C */
    {.key = 0x7E0124A1u, .width = 4u, .scr_idx = BP_SCR_IDX(drive_oneshot)},       /* [18] g_scr+0x10 */
    {.key = 0xF8F9CB4Bu, .width = 4u, .scr_idx = BP_SCR_IDX(submode_code)},        /* [19] g_scr+0x42 */
    {.key = 0xA2BC1D75u, .width = 4u, .scr_idx = BP_SCR_IDX(commto_mirror)},       /* [20] g_scr+0x08 */
    {.key = 0x46DB128Fu, .width = 4u, .scr_idx = BP_SCR_IDX(signal_74)},           /* [21] g_scr+0x74 */
    {.key = 0x42416F9Fu, .width = 1u, .scr_idx = BP_SCR_IDX(power_sign)},          /* [22] g_scr+0x06 */
    {.key = 0x17B28D98u, .width = 1u, .scr_idx = BP_SCR_IDX(lock_req)},            /* [23] g_scr+0x3E */
    {.key = 0xCFBC0A29u, .width = 1u, .scr_idx = BP_SCR_IDX(signal_72)},           /* [24] g_scr+0x72 */
    {.key = 0x88E7BACCu, .width = 1u, .scr_idx = BP_SCR_IDX(telem_70)},            /* [25] g_scr+0x70 */
    {.key = 0x0C99E9E5u, .width = 1u, .scr_idx = BP_SCR_IDX(dist_is_trip)},        /* [26] g_scr+0x16 */
    {.key = 0x1841E961u, .width = 1u, .scr_idx = BP_SCR_IDX(signal_12)},           /* [27] g_scr+0x12 */
    {.key = 0x16B5A88Au, .width = 1u, .scr_idx = BP_SCR_IDX(signal_58)},           /* [28] g_scr+0x58 */
    {.key = 0x70EFD7B3u, .width = 1u, .scr_idx = BP_SCR_IDX(signal_5a)},           /* [29] g_scr+0x5A */
    {.key = 0x751A2603u, .width = 1u, .scr_idx = BP_SCR_IDX(bat2_present)},        /* [30] g_scr+0x0A */
    {.key = 0x06BFB039u, .width = 1u, .scr_idx = BP_SCR_IDX(ilk_publish)},         /* [31] g_scr+0x52 */
    {.key = 0x20E8263Bu, .width = 1u, .scr_idx = BP_SCR_IDX(signal_4c)},           /* [32] g_scr+0x4C */
    {.key = 0x7F1A2555u, .width = 1u, .scr_idx = BP_SCR_IDX(drive_fault_req)},     /* [33] g_scr+0x56 */
    {.key = 0xF739712Bu, .width = 1u, .scr_idx = BP_SCR_IDX(signal_94)},           /* [34] g_scr+0x94 */
    {.key = 0xAC4E646Bu, .width = 1u, .scr_idx = BP_SCR_IDX(overspeed_regen_on)},  /* [35] g_scr+0x96 */
    {.key = 0x0086BD19u, .width = 1u, .scr_idx = BP_SCR_IDX(wobble_alert_on)},     /* [36] g_scr+0x98 */
    {.key = 0x0C7D931Au, .width = 1u, .scr_idx = BP_SCR_IDX(signal_9a)},           /* [37] g_scr+0x9A */
};
#undef BP_SCR_IDX

/**
 * \brief           Report/log manager object init
 * \note            Populates g_report_mgr: the embedded 7-pointer vtable, the Ninebot frame
 *                      header bytes (nb_src=src, nb_dst=dst) and the node-table metadata
 *                      (node_count, desc_table/count, send_obj), then zeroes the node table.
 *                      Returns the object base; the caller stores it into g_bp_rptr.mgr_ptr.
 */
static uint32_t bp_report_mgr_init(uint8_t src, uint8_t dst) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);

    /* Embedded vtable. */
    m->vt_resolve = bp_rpt_resolve;
    m->vt_emit_fc = bp_rpt_emit;
    m->vt_emit_fd = bp_rpt_emit_ack;
    m->vt_state_notify = bp_state_notify;
    m->vt_register_obj = bp_rpt_register_obj;
    m->vt_notify = bp_rpt_notify;
    m->vt_register_desc = bp_rpt_register_desc;

    /* Ninebot frame header (src -> nb_src @+0x1D, dst -> nb_dst @+0x1E). */
    m->flag = 0;
    m->nb_src = src;
    m->nb_dst = dst;
    m->nb_cmd = 0;
    m->nb_arg = 0;

    /* Zero the node table. */
    bp_block_clear((volatile uint8_t*)m->node, sizeof m->node);

    /* Node-table metadata. */
    m->node_count = 0;
    m->desc_table = (uint32_t)(uintptr_t)k_bp_sig_desc; /* descriptor table */
    m->desc_count = (uint16_t)BP_SIG_DESC_COUNT;        /* descriptor count (38) */
    m->send_obj = (uint32_t)(uintptr_t)&g_uartctx;      /* send object: the USART2 TX ring (bp_rpt_send) */
    return (uint32_t)(uintptr_t)(&g_report_mgr);        /* the object address */
}

/** Report descriptor / vtable init. */
static void bp_report_vtable_init(uint32_t descriptor) {
    g_bp_rvtbl.descriptor = descriptor;
    g_bp_rvtbl.vfn = (uint32_t)(uintptr_t)&bp_report_vfn;
}

/**
 * \brief           Command handler for frame data[0]==0xFC
 * \note            Notifies the report manager, then emits a 16-byte reply through the
 *                      report-manager vtable.
 */
void bp_handle_fc(uint8_t sa, uint8_t da, const bp_frame_t* frame) {
    volatile bp_report_mgr_t* m = (&g_report_mgr);

    (void)sa;
    (void)da;
    if (frame->data[0] == BP_CMD_FC) {
        m->vt_notify(1); /* mgr_ptr->vt[+0x14] */
        /* mgr_ptr->vt[+0x04]: emit the 0xFC reply; nb_src/nb_dst come from the mgr object. */
        m->vt_emit_fc((void*)m, 0, (const uint8_t*)(const volatile void*)g_bp_reply_payload, BP_RPT_REPLY_LEN);
    }
}

/** \brief The registered report-node record (see bp_node_rec_t). */
volatile bp_node_rec_t g_bp_node;

/**
 * \brief           Node / device registration handler
 * \note            Resolves the node from its registration key, looks up its handle and --
 *                      when one exists -- clears the 0x28-byte node record, stores the node
 *                      id and the handle's +2 byte as the report period, and marks all 0x26
 *                      capability slots present. The Ninebot 0xFD request handler passes the
 *                      request body as \p key and the body length f[0] as \p len.
 * \param[in]       key: registration key {node id, period, N x {u32 signal key LE, u8 width}}
 * \param[in]       len: key length in bytes (2 + 5 * N)
 */
void bp_node_register(const uint8_t* key, uint8_t len) {
    volatile bp_node_rec_t* rec = &g_bp_node;
    volatile bp_report_mgr_t* m = (&g_report_mgr);
    int nodeid;
    void* handle;
    int i;

    /* mgr_ptr->vt[+0x00] resolve (key, len), then vt[+0x18] look up the node handle by the
       resolved id. */
    nodeid = m->vt_resolve(key, len);
    handle = m->vt_register_desc((uint8_t)nodeid);
    g_bp_rptr.last_node_handle = (uint32_t)(uintptr_t)handle;
    if (handle != NULL) {
        bp_block_clear((volatile uint8_t*)rec, BP_NODE_REC_SIZE);
        rec->node_id = (uint8_t)nodeid;
        rec->report_period = *((volatile uint8_t*)handle + 2U); /* descriptor[+2] */
        for (i = 0; i < (int)BP_NODE_CAP_SLOTS; i++) {
            rec->cap[i] = 1;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Subsystem init + self-announce.                                            */
/* ------------------------------------------------------------------------- */

/**
 * Comms subsystem init + self-announce id 0xD0.
 * The announce body is the first BP_ANNOUNCE_LEN bytes of a little-endian 32-bit 1, so the
 * wire body is `01 00`.
 */
static void bp_subsystem_init(void) {
    const uint8_t announce[BP_ANNOUNCE_LEN] = {1U, 0U};

    bp_report_vtable_init(bp_tx_ctx_init(0)); /* the tx context base becomes the descriptor */
    /* The mgr-ptr cell holds &g_report_mgr. bp_report_mgr_init populates the object's
       embedded vtable + header + table metadata and returns that base. */
    g_bp_rptr.mgr_ptr = bp_report_mgr_init(NB_ADDR_VCU, NB_ADDR_APP);
    nb_commit_mode(0);
    bp_block_clear((volatile uint8_t*)&g_bp_node, BP_NODE_REC_SIZE);
    vcu_usart_report_send((volatile uint8_t*)&g_uartctx, NB_ADDR_VCU, NB_ADDR_APP, BP_ANNOUNCE_LEN, NB_CMD_WRITE3,
                          BP_ANNOUNCE_ARG, announce);
    g_vs.app_reg_pending = 1;
}

/* ------------------------------------------------------------------------- */
/* Public API.                                                                */
/* ------------------------------------------------------------------------- */

void vcu_busproto_init(void) {
    g_bp_ctx.node_accept = bp_node_accept;
    g_bp_ctx.on_ack = bp_on_ack;
    g_bp_ctx.on_data = bp_on_data;
    g_bp_ctx.emit = bp_emit;
    /* The subsystem init runs BEFORE the manager init. */
    bp_subsystem_init();
    bp_manager_init();
}

void vcu_busproto_rx(const void* ext_frame) {
    if (ext_frame != NULL) {
        bp_dispatch(&g_bp_mgr, (const bp_frame_t*)ext_frame);
    }
}

/* J1939 TP frames for the slot-3 fast service (RTS, TP.DT, pacing step and the
   [ctx+0x0C] emit). Same 13-byte record and id composition as the TP.CM
   builders above: keep bits 31/29 of the prior id word, merge (src | dst<<8 | base) masked
   to 29 bits, set the bit29 marker. Data[5..7] carry PGN 0x00EF00. */
#define TP_ABORT_PHASE   1     /* on_ack phase for a pacing timeout */
#define FAST_PACING_BASE 1250  /* fast-service pacing base (/ rate) */

static uint32_t bp_tp_extid(uint32_t prev, uint32_t base, uint8_t src, uint8_t dst) {
    return (prev & BP_EXTID_KEEP) | (((uint32_t)src | ((uint32_t)dst << 8) | base) & BP_EXTID_MASK) | BP_EXTID_FLAG;
}

/* TP.CM RTS: open a transfer of `len` (> 8) bytes from src to dst. */
static int bp_fast_frame_init(uint8_t src, uint8_t dst, uint16_t len, bp_frame_t* out) {
    if (out == NULL || len <= VCU_CAN_DLEN) {
        return 0;
    }
    out->id = bp_tp_extid(out->id, BP_EXTID_CM_BASE, src, dst);
    out->dlc = VCU_CAN_DLEN;
    out->data[0] = BP_TP_RTS;
    out->data[1] = (uint8_t)len;
    out->data[2] = (uint8_t)(len >> 8);
    out->data[3] = (uint8_t)((len + (BP_TP_CHUNK - 1U)) / BP_TP_CHUNK); /* packet count */
    out->data[4] = BP_TP_PAD;
    out->data[5] = BP_TP_PGN_LO;
    out->data[6] = BP_TP_PGN_MID;
    out->data[7] = BP_TP_PGN_HI;
    return 1;
}

/* TP.DT: packet number consumed/7 + 1, then up to 7 bytes from
   srcptr, 0xFF-padded. `consumed` must be a whole number of packets. On return
   *remaining holds the number of bytes copied. The caller (bp_fast_service_entry)
   passes remaining = total - consumed, so the copy never runs past the buffer. */
static int bp_fast_frame_stream(uint8_t src, uint8_t dst, uint16_t consumed, const uint8_t* srcptr, int32_t* remaining,
                                bp_frame_t* out) {
    int32_t c;
    int32_t i;

    if (out == NULL || remaining == NULL || (consumed % BP_TP_CHUNK) != 0U) {
        return 0;
    }
    c = *remaining;
    if (c > (int32_t)BP_TP_CHUNK) {
        c = (int32_t)BP_TP_CHUNK;
    }
    if (c < 0) {
        c = 0;
    }
    out->id = bp_tp_extid(out->id, BP_EXTID_DT_BASE, src, dst);
    out->dlc = VCU_CAN_DLEN;
    out->data[0] = (uint8_t)((consumed + (BP_TP_CHUNK - 1U)) / BP_TP_CHUNK + 1U); /* 1-based packet no. */
    for (i = 0; i < (int32_t)BP_TP_CHUNK; i++) {
        out->data[1 + i] = (i < c && srcptr != NULL) ? srcptr[i] : BP_TP_PAD;
    }
    *remaining = c;
    return 1;
}

/* Emit through the handler vtable's send slot [ctx+0x0C].
   The vtable is built at run time (busproto's handler table -> bp_emit), so the call
   is NULL-guarded. */
static void bp_fast_emit(int32_t ctx, volatile bp_session_t* s, const bp_frame_t* frame) {
    const bp_ctx_t* vt = (const bp_ctx_t*)(uintptr_t)ctx;

    if (vt != NULL && vt->emit != NULL) {
        (void)vt->emit((bp_session_t*)s, frame);
    }
}

/* Pacing / timeout step for the waiting states: advance the tick (+0x16); once it
   reaches the timeout window (+0x18), tell the ack callback [ctx+4] (phase 1), send a
   TP.CM abort (bp_frame_bcast) and idle the session. */
static void bp_fast_frame_cont(volatile bp_session_t* s, int32_t ctx) {
    const bp_ctx_t* vt = (const bp_ctx_t*)(uintptr_t)ctx;
    bp_frame_t frame = {0};
    uint16_t t = s->tick;

    s->tick = (uint16_t)(t + 1u);
    if ((int32_t)(uint16_t)(t + 1u) >= (int32_t)s->timeout) {
        if (vt != NULL && vt->on_ack != NULL) {
            vt->on_ack((bp_session_t*)s, s->addr_a, s->addr_b, TP_ABORT_PHASE);
        }
        (void)bp_frame_bcast(s->addr_a, s->addr_b, &frame);
        bp_fast_emit(ctx, s, &frame);
        s->state = BP_ST_IDLE;
    }
}

/* slot3 per-entry service: advances one 0x24-byte session                         */
/* (\ref bp_session_t) through its state machine. State 3 builds the initial frame */
/* and arms the pacing (timeout = 0x4E2 / period); states 4/6/7/9/10 run the        */
/* continuation (bp_fast_frame_cont); state 5 streams the body in 7-byte chunks,    */
/* emitting via the ctx vtable (+0x0C) until the window end or the total is         */
/* reached. The chunk pacing counts `retry` up to `deadline`. */
static void bp_fast_service_entry(volatile bp_session_t* s, int32_t ctx) {
    bp_frame_t buf = {0};
    uint16_t next;
    uint16_t base;
    int32_t remain;

    if (s == NULL) {
        return;
    }
    switch (s->state) {
        case 3:
            (void)bp_fast_frame_init(s->addr_a, s->addr_b, s->total, &buf);
            bp_fast_emit(ctx, s, &buf);
            s->tick = 0;
            s->timeout = (int16_t)(FAST_PACING_BASE / s->period);
            s->state = 4;
            break;
        case 4:
        case 6:
        case 7:
        case 9:
        case 10: bp_fast_frame_cont(s, ctx); break;
        case 5:
            do {
                uint16_t r = s->retry;
                s->retry = (uint16_t)(r + 1);
                if ((uint16_t)(r + 1) < (uint16_t)s->deadline) {
                    return;
                }
                base = s->pos;
                remain = (int32_t)s->total - (int32_t)base;
                (void)bp_fast_frame_stream(s->addr_a, s->addr_b, base, s->buf + base, &remain, &buf);
                bp_fast_emit(ctx, s, &buf);
                s->retry = 0;
                next = (uint16_t)(s->pos + 7);
                s->pos = next;
                if (s->total <= next) {
                    s->pos = s->total;
                    s->tick = 0;
                    s->timeout = (int16_t)(FAST_PACING_BASE / s->period);
                    s->state = 9;
                    return;
                }
                if (s->win_end <= next) {
                    s->tick = 0;
                    s->timeout = (int16_t)(FAST_PACING_BASE / s->period);
                    s->state = 6;
                    return;
                }
            } while (s->deadline == 0);
            break;
        default: break;
    }
}

/* ========================================================================= */
/* slot 3 (period 1): fast per-entry service                                 */
/* ========================================================================= */
void bp_task_fast_service(void) {
    /* The session manager: session array, count, handler ctx. */
    volatile bp_session_t* base = g_bp_mgr.base;
    uint16_t count = g_bp_mgr.count;
    int32_t ctx = (int32_t)(uintptr_t)g_bp_mgr.ctx;
    uint16_t i;

    for (i = 0; i < count; i = (uint16_t)(i + 1)) {
        bp_fast_service_entry(&base[i], ctx); /* 0x24-byte sessions */
    }
}
