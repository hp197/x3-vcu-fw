/**
 * \file            vcu_state.c
 * \brief           Definition of the per-block shared SRAM "blackboard" globals.
 * \note            See vcu_state.h for the block layout. This TU owns the backing storage:
 *                      one typed global per block, so every module's accesses alias the SAME
 *                      bytes. Ordinary .bss -- only the offsets inside a block matter, no linker
 *                      script or section attribute needed. The genuine overlaps (IN2 identity
 *                      record, LEDM in IN2, fm_mon over EVT, the light-toggle view over g_chg)
 *                      are typed views of the same global.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include "vcu_state.h"
#include "busproto.h" /* bp_report_ptrs_t: real type of the g_bp_rptr backing */
#include "charging.h" /* vcu_charge_timer_t: real type of the g_charge_timer backing */
#include "passcode.h" /* passcode_ch_t: real type of the g_passcode_ch backing */

/* g_in2 is also read through other typed views of the same bytes (identity_rec_t,
   the Ninebot register file, LEDM); its u32 members already give it natural 4-byte alignment. */
volatile vcu_vs_t          g_vs;
volatile vcu_in2_block_t   g_in2;
volatile vcu_in1_t         g_in1;
volatile vcu_scr_t         g_scr;
volatile vcu_stat_t        g_stat;
volatile vcu_evt_t         g_evt;
volatile vcu_mode_t        g_mode;
volatile vcu_chg_t         g_chg;
volatile vcu_chgbuf_t      g_chgbuf;
volatile vcu_prev_t        g_prev;
volatile vcu_tmr_t         g_tmr;
volatile vcu_candev_t      g_candev; /* timers set by vcu_can_init */
volatile vcu_can_sched_t   g_can_sched;
volatile vcu_oneshot_t     g_oneshot;
volatile vcu_fault_codes_t         g_fault_codes;
volatile vcu_deb_t         g_deb;
volatile vcu_disp_t        g_disp;
volatile vcu_uartctx_t     g_uartctx; /* .free = 7 set by vcu_usart_init */
volatile vcu_canmsg_t      g_canmsg;
volatile vcu_lowglob_t     g_lowglob;
volatile vcu_state_block_t g_state_block;
volatile vcu_bms_commloss_t g_bms_commloss;
volatile vcu_stat_t         g_stat2;
volatile vcu_bat2_t         g_bat2;
volatile vcu_node2_push_t   g_node2_push;

/* g_bp_rptr: two u32 words, read by busproto and by the scheduler's dispatch tick. */
volatile bp_report_ptrs_t g_bp_rptr;

/* Factory UID snapshot (3 words copied from 0x1FFFF7E8 at boot). */
volatile uint32_t g_mcu_uid[3];

/* Charge-timer and passcode-channel state. Both used to be untyped byte arrays that
   other modules reached into by offset; those reaches are now named fields
   (g_charge_timer.relay_dwell, g_passcode_ch.disp_word). */
volatile vcu_charge_timer_t g_charge_timer;
volatile passcode_ch_t          g_passcode_ch = {.reload = {0x0030u, 0x0020u, 0x0010u}};

