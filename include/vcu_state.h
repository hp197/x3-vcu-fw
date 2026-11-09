/**
 * \file            vcu_state.h
 * \brief           Shared SRAM "blackboard" per-block globals + accessors (x3 VCU)
 * \note            Each block below is a plain aligned C global of its vcu_*_t type (defined
 *                      in vcu_state.c, declared extern at the end of this header); modules use
 *                      the globals directly (g_vs.region, g_in2.f.gear, ...). The linker places
 *                      them -- only the offsets inside a block matter, never the addresses.
 *
 *                      Most blocks are independent. The overlaps that carry a real dependency
 *                      stay shared on one global: IN2 (its live bytes sit inside the 512-byte
 *                      identity record that is checksummed and flashed), LEDM inside IN2, the
 *                      fm_mon_t records of EVT (a monitor's code byte IS the event flag), and
 *                      the light-toggle view over g_chg.
 *
 *                      Nothing is packed: the blackboard is a 16-bit register file, so the odd
 *                      32-bit values are really halfword pairs (IN2.passcode_key[2],
 *                      VS.passcode_new[2]). Every cell is a named field -- no base+offset accessors.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef VCU_STATE_H
#define VCU_STATE_H

#include <stddef.h>
#include <stdint.h>

/*
 * Each blackboard block is its own aligned global of the matching vcu_*_t type (declared
 * below, after the struct typedefs); modules reach its cells through the named fields. The
 * genuine overlaps (IN2 identity-record union, LEDM in IN2, the HL view over g_chg, the
 * fm_mon_t records of EVT) are typed views of the SAME global, so cross-module sharing is
 * preserved. Only the offsets inside a block matter. The externs live at the end of this header.
 */

/* ========================================================================= */
/* Named layouts of the blackboard blocks.                                    */
/*                                                                           */
/* Each named field sits at the fixed offset in its comment. Reserved `_rsvNN` */
/* members only exist to place the next field at its offset; a cell with a    */
/* known reader or writer but no known purpose is named `<block>_<offset>`.   */
/* ========================================================================= */

/**
 * \brief           Main vehicle-state struct @ g_vs
 * \note            0x68 bytes; +0x42..+0x67 is the VS-tail scratch area. Unpacked aligned
 *                      struct: the one 2-misaligned 32-bit value, the staged passcode @ +0x56,
 *                      is two halfword slots (passcode_new[2]). Everything else is naturally
 *                      aligned, so sizeof stays 0x68 and every field keeps its offset.
 */
typedef struct {
    uint8_t region;            /*!< +0x00 region = live drive-table row (\ref vcu_region_t) */
    uint8_t gear_sel;          /*!< +0x01 stored gear WALK=1..POWER=4 (overload: speed-lock / model-variant) */
    uint8_t speed_kmh;         /*!< +0x02 vehicle speed, km/h (= IN2 speed_src / 10, written by report_telemetry_build) */
    uint8_t start_req;         /*!< +0x03 start request (\ref vcu_start_req_t)      */
    uint8_t drive_fault_state; /*!< +0x04 drive over-temp fault FSM state         */
    uint8_t substate;          /*!< +0x05 drive_param_init substate (==3)         */
    uint8_t tilt_latch;        /*!< +0x06 tilt latch (2 armed, 1 tripped -> power-down / alarm exit) */
    uint8_t ertc_wake;         /*!< +0x07 set by ERTC_WKUP_IRQHandler (3 s ERTC tick) */
    uint8_t wake_src;          /*!< +0x08 vehicle VS_WAKE_SRC                        */
    uint8_t accel_limit_fwd;   /*!< +0x09 forward accel limit (mp[ACC] * 1.3)     */
    uint8_t accel_limit_rev;   /*!< +0x0A reverse accel limit (mp[ACC] * 0.8)     */
    uint8_t _rsv0b;            /*!< +0x0B                                        */
    uint16_t lock_arm_ctr;     /*!< +0x0C standby-arm count-up (clamp 600)        */
    uint16_t distance_code;    /*!< +0x0E vehicle VS_DISTANCE_CODE / ridemode dist   */
    union {
        float sensor_delta_f;      /*!< +0x10 |sample - prev|, float channel      */
        int32_t sensor_delta_bits; /*!< +0x10 same cell, int-bits compare (step ramp advance) */
    };
    float sensor_delta_i;      /*!< +0x14 |sample - prev|, int channel            */
    uint8_t chg_keep_awake;    /*!< +0x18 charge keep-awake: charger + schedule + SoC below limit -> state 4 goes to 8 */
    uint8_t chg_sched_stop;    /*!< +0x19 charging stopped by the schedule window (set at window end; blocks IN1+0x25, forces "complete") */
    uint8_t chg_full;          /*!< +0x1A charge complete (SoC >= 100 or >= limit, or VS+0x19)          */
    uint8_t power_released;    /*!< +0x1B power latch released (end of state 4)      */
    uint8_t shutdown_req;      /*!< +0x1C shutdown request: every state -> 4 (auto-off, passcode lock-out, siren end, comm timeout, app) */
    uint8_t can_drive_active;  /*!< +0x1D UNION: drive-active writer (+) any-fault writer; published status, time-shared */
    uint8_t power_ready;       /*!< +0x1E power-on / wake done                       */
    uint8_t chg_sched_cancel_notice; /*!< +0x1F schedule-cancel notice: screen 0x13 + beep, keeps state 8 awake */
    uint8_t chg_disp_refresh_req; /*!< +0x20 request a state-8 charge-screen refresh (one-shot)       */
    uint8_t chg_disp_refresh;  /*!< +0x21 state-8 charge-screen refresh (set at window end and by VS+0x20) */
    uint8_t remote_lock_state; /*!< +0x22 ctrl op 0x7A lock value (edge -> beep); ctrl ops 7/0x0A clear */
    uint8_t remote_lock_edge;  /*!< +0x23 set on a 0 -> non-zero op 0x7A edge; consumed by ridemode       */
    uint8_t flash_seq;         /*!< +0x24 lamp flash sequence (\ref vcu_flash_seq_t); 0 runs the charger-detect half */
    uint8_t fault_ack;         /*!< +0x25 vehicle VS_FAULT_ACK                       */
    uint8_t report_enable;     /*!< +0x26 report VS_REPORT_ENABLE / nb report gate  */
    uint8_t app_reg_pending;   /*!< +0x27 set by the busproto APP register commit; state 0 shuts down 300 ticks later */
    uint8_t app_start_req;     /*!< +0x28 app power-on / start request: acts like a button press in states 0/6/8 */
    uint8_t chirp_req;         /*!< +0x29 10-tick chirp pulse request (unlock / charger / alarm) */
    uint8_t creep_enable;      /*!< +0x2A vehicle VS_CREEP_ENABLE                    */
    uint8_t poweroff_beep;     /*!< +0x2B vehicle VS_POWEROFF_BEEP                   */
    uint8_t app_unlock_req;    /*!< +0x2C app unlock bypass (info 0x79 with the lock enabled): state 0 -> start unlocked, state 2 -> unlocked */
    uint8_t reboot_req;        /*!< +0x2D report datetime-reset / reboot request    */
    uint16_t chirp_tick;       /*!< +0x2E chirp pulse tick (GPIO bit set/clear around it)              */
    uint16_t lock_timer;       /*!< +0x30 standby-arm count-up                   */
    uint8_t _rsv32[2];         /*!< +0x32..+0x33                                 */
    uint16_t keepalive_ticks;  /*!< +0x34 keep-awake timer: control frames raise it to >= 300; state 0 counts it down */
    uint16_t mode_timeout;     /*!< +0x36 mode/lighting reload                   */
    int16_t postride_timer;    /*!< +0x38 vehicle VS_POSTRIDE_TIMER                 */
    uint8_t _rsv3a[2];         /*!< +0x3A..+0x3B                                 */
    uint32_t auto_off_ticks;   /*!< +0x3C auto power-off countdown (reg 0x49 minutes x 6000)            */
    uint8_t remote_lock_req;   /*!< +0x40 remote lock request (ctrl op 0x7B arg 4, lock enabled)        */
    uint8_t passcode_set_pending; /*!< +0x41 passcode change pending (app wrote reg 0x71; confirm on the handlebar) */
    /* ---- VS-tail scratch +0x42..+0x67 ---- */
    uint8_t lockout_phase;     /*!< +0x42 passcode lock-out phase (\ref vcu_lockout_phase_t) */
    uint8_t remote_unlocked;   /*!< +0x43 remote unlocked (ctrl op 0x7B arg 1): bypasses the passcode    */
    uint8_t unlock_ok;         /*!< +0x44 passcode routine result (1 = correct code / unlocked) */
    uint8_t code_len;          /*!< +0x45 passcode length (non-zero nibbles of the code); passcode screen parameter */
    uint8_t code_retries;      /*!< +0x46 wrong passcode entries this session (3 -> lock-out) */
    uint8_t code_entry[8];     /*!< +0x47..+0x4E passcode entry buffer: one key / digit per byte, 8 = end mark (u32 block clears) */
    uint8_t _rsv4f;            /*!< +0x4F                                        */
    uint16_t code_digit;       /*!< +0x50 numeric passcode: digit value 0..9 under the cursor */
    uint16_t code_cursor;      /*!< +0x52 passcode entry cursor                              */
    uint16_t code_count;       /*!< +0x54 numeric passcode: digits accepted so far           */
    uint16_t passcode_new[2];  /*!< +0x56 staged new key passcode (app reg 0x71 write), two halfwords (lo, hi) */
    uint8_t _rsv5a[2];         /*!< +0x5A..+0x5B                                 */
    uint32_t crm_ctrl_save;    /*!< +0x5C CRM_CTRL register snapshot, saved before power-down and restored on wake (NOT odometer) */
    uint32_t crm_cfg_save;     /*!< +0x60 CRM_CFG register snapshot, saved before power-down and restored on wake (NOT odometer) */
    uint8_t input_test_mode;   /*!< +0x64 input-test mode (info 0x56): FSM reports every input event      */
    uint8_t input_test_ea;     /*!< +0x65 info 0x56 arg 0xEA sets; cleared on state change; no reader     */
    uint8_t input_test_ee;     /*!< +0x66 info 0x56 arg 0xEE sets; no reader (write-only)              */
    uint8_t _rsv67;            /*!< +0x67                                        */
} vcu_vs_t;

/**
 * \brief           Passcode lock-out phase @ g_vs.lockout_phase (VS+0x42)
 * \note            Written only by the passcode unlock routines, the alarm 6 -> 0 edge and the
 *                      siren: the third wrong entry arms the lock-out (and shuts down); a
 *                      wrong entry while armed raises the alarm (state 2 -> 6).
 */
typedef enum {
    LOCKOUT_NONE = 0,  /*!< no lock-out                                             */
    LOCKOUT_ARMED = 1, /*!< three wrong entries: the next wrong entry raises the alarm */
    LOCKOUT_ALARM = 2  /*!< lock-out alarm: siren, state 6                           */
} vcu_lockout_phase_t;

/** \brief Lamp flash sequence @ g_vs.flash_seq (VS+0x24). */
typedef enum {
    FLASH_SEQ_IDLE = 0,     /*!< no sequence: the charger-detect half runs            */
    FLASH_SEQ_START = 1,    /*!< start / ack flash (-> 2, 0xFA countdown)             */
    FLASH_SEQ_RUN = 2,      /*!< sequence running                                     */
    FLASH_SEQ_LOCATE = 0xEE /*!< locate flash (ctrl op 3 arg 0x86 body 1: tone + VS+0x34 = 300) */
} vcu_flash_seq_t;

/** \brief Standstill threshold: \ref vcu_vs_t.speed_kmh below this counts as standing still (lock / arm gates). */
#define VCU_STANDSTILL_KMH 3u


/** \brief Input-flag block @ g_in1, all byte flags. */
typedef struct {
    uint8_t _rsv00;              /*!< +0x00                              */
    uint8_t remote_countdown_req; /*!< +0x01 300-tick countdown with beep + report 0xD4 (generic cmd 0x7A in state 3 at standstill) */
    uint8_t bms_temp_rx;         /*!< +0x02 a BMS 0x425 temperature frame arrived (set by can_rx_std; faultmon reads it) */
    uint8_t can_active_req;      /*!< +0x03 re-arm request for the 400-tick CAN-active window: set by ninebot on every app/OTA session command, consumed + cleared by vcu_can_senders */
    uint8_t drive_active;        /*!< +0x04 can sole writer; consumers read (tick-suspend/gest-inhibit/chg-abort) */
    uint8_t uv_grace_ext;        /*!< +0x05 long UV grace (fault 0x19 debounce 6000 instead of 1000): ctrl op 0x0A sets, APP op 2 arg 0x84 clears */
    uint8_t _rsv06[5];           /*!< +0x06..+0x0A                       */
    uint8_t brake_any;           /*!< +0x0B either brake lever active (PA1 half / PA4 full weight) */
    uint8_t throttle_flag;       /*!< +0x0C throttle active, copy of g_tmr.throttle_active */
    uint8_t _rsv0d;              /*!< +0x0D                              */
    uint8_t retry_req;           /*!< +0x0E faultmon FM_RETRY_REQ        */
    uint8_t _rsv0f;              /*!< +0x0F                              */
    uint8_t recompute_req;       /*!< +0x10 cross-module req/ack (control/vehicle/faultmon/sensor) */
    uint8_t _rsv11[7];           /*!< +0x11..+0x17                       */
    uint8_t sub0607_rx;          /*!< +0x18 set on a sub-node 0x06/0x07 frame; no reader (write-only) */
    uint8_t _rsv19[5];           /*!< +0x19..+0x1D                       */
    uint8_t throttle_fault_latch; /*!< +0x1E control / faultmon FM_IN1_DTC0E */
    uint8_t brake_full_range_fault; /*!< +0x1F full-weight brake lever (PA4) out-of-band latch */
    uint8_t cal_req_a;           /*!< +0x20 sensor FLAG_CAL_A            */
    uint8_t cal_req_b;           /*!< +0x21 sensor FLAG_CAL_B            */
    uint8_t chg_sched_armed;     /*!< +0x22 scheduled charge armed (charger connect with reg 0x1F b4; cleared at window end / reg 0x4B write) */
    uint8_t app_flag_5e;         /*!< +0x23 set by info op 0x5E; no reader (write-only) */
    uint8_t dtc23_in;            /*!< +0x24 faultmon FM_IN1_DTC23 (== VS+0x8C, cross-struct) */
    uint8_t charger_connected;   /*!< +0x25 charger connected (BMS 0x422 charger bit, set by the charger detect) */
    uint8_t regen_active;        /*!< +0x26 regen/neutral this tick       */
    uint8_t arm_gate;            /*!< +0x27 vehicle IN1_ARM_GATE / faultmon LATCH_HANDLED */
    uint8_t bms_chg_cmd_req;     /*!< +0x28 BMS charge-control push (\ref vcu_bms_chg_cmd_t) */
    uint8_t throttle_stuck_guard; /*!< +0x29 control stuck guard          */
    uint8_t brake_full_cal_guard; /*!< +0x2A full-weight brake lever (PA4) calibration guard */
    uint8_t brake_half_cal_guard; /*!< +0x2B half-weight brake lever (PA1) calibration guard */
    uint8_t brake_half_range_fault; /*!< +0x2C half-weight brake lever (PA1) out-of-band latch */
} vcu_in1_t;


/** \brief BMS charge-control push @ g_in1.bms_chg_cmd_req (IN1+0x28). */
typedef enum {
    BMS_CHG_CMD_IDLE = 0,  /*!< nothing pending                                         */
    BMS_CHG_CMD_SEND = 1,  /*!< send Ninebot op 4 arg 0x87 to node 7 (charger detect)   */
    BMS_CHG_CMD_ACKED = 2  /*!< node 7 acked (sub-node op 5 body 0x87); hold-off back to 0 */
} vcu_bms_chg_cmd_t;

/**
 * \brief           Ninebot register file, page 0 @ g_in2: register n = IN2 + 2n
 * \note            This block IS the Ninebot 16-bit register file the dispatcher reads and
 *                      writes (`IN2 + arg*2`; page 1 = SCR, page 2 = STAT). It is also the 512-byte identity record
 *                      that is cleared, checksummed and persisted halfword by halfword
 *                      (\ref vcu_in2_block_t). Named fields carry their register number; the
 *                      only odd-looking 32-bit value, the key passcode at +0xE2, is really two
 *                      halfword registers (passcode_key[2]). The u32 counters at +0xC4/+0xC8/
 *                      +0xCC sit on 4-byte offsets and are naturally aligned.
 */
typedef struct {
    uint8_t _rsv00[0x0C];       /*!< +0x00..+0x0B                        */
    int16_t throttle_cal_sel;   /*!< +0x0C reg 0x06 control throttle cal select (==1) */
    uint8_t _rsv0e[0x12];       /*!< +0x0E..+0x1F                        */
    union {
        uint8_t serial[14];     /*!< +0x20..+0x2D reg 0x10..0x16 14-byte serial number (op 0x18 provisioning; tail +0x24 compared with "0000000001") */
        uint16_t serial_reg[7]; /*!< +0x20..+0x2D the same bytes as registers 0x10..0x16 (CAN 0x483/0x484) */
    };
    uint16_t serial_mark;       /*!< +0x2E reg 0x17 op 0x41 arg 0x17 reply base; written by vcu_config_init */
    uint16_t canrx_212;         /*!< +0x30 can rx 0x212 W            */
    uint16_t canrx_421;         /*!< +0x32 can rx 0x421 W            */
    uint16_t bat2_version;      /*!< +0x34 battery-2 version (CAN 0x431) */
    uint16_t led_ind_main;      /*!< +0x36 reg 0x1B mode LED main indicator word (ridemode writes it as LEDM.ind_main; CAN 0x209 bytes 0..1) */
    uint16_t status_flags;      /*!< +0x38 reg 0x1C status (\ref SF_RUN .. \ref SF_SESSION) */
    uint16_t cfg_flags;         /*!< +0x3A reg 0x1D settings (\ref CFG_LOCK_EN .. \ref CFG_ALARM_EN) */
    uint16_t cfg_flags2;        /*!< +0x3C reg 0x1E settings 2 (\ref CFG2_ACK_BEEP .. \ref CFG2_LAMP_FLASH) */
    uint16_t chg_cfg;           /*!< +0x3E reg 0x1F charge settings (\ref CHG_DISP_EN .. \ref ALARM_TIMEOUT_EN) */
    union {
        uint8_t vin[16];        /*!< +0x40..+0x4F reg 0x20..0x27 VIN (op 0x19 CRC-gated write, op 0x41 arg 0x20 read) */
        struct {
            uint8_t _vin_lo[0x0A];       /*!< +0x40..+0x49 VIN bytes 0..9          */
            uint16_t stored_mode_shadow; /*!< +0x4A VIN bytes 10..11: control stored mode shadow / config warm-boot guard */
            uint8_t _vin_hi[4];          /*!< +0x4C..+0x4F VIN bytes 12..15        */
        };
    };
    uint8_t _rsv50[0x0C];       /*!< +0x50..+0x5B                        */
    uint16_t reg_2e;            /*!< +0x5C reg 0x2E: zeroed by control_drive_param_init; no reader in the drive code */
    uint16_t in2_5e;            /*!< +0x5E reg 0x2F, sent in CAN 0x482 bytes 2..3; bits \ref REG2F_BIT0, \ref REG2F_TURN_CANCEL_OFF */
    uint8_t _rsv60[2];          /*!< +0x60..+0x61 (ridemode LEDM.ind_aux_lo private) */
    uint16_t datetime_enable;   /*!< +0x62 report bit3 (ridemode LEDM.ind_aux_hi private) */
    uint16_t ride_caps_32;      /*!< +0x64 reg 0x32: bit2 set by ridemode_led_matrix_update in row 0, cleared in rows 1/2 */
    uint16_t ride_caps_33;      /*!< +0x66 reg 0x33: bits 0/2/3 set in rows 1/2, cleared in row 0; bits 1/4 always cleared */
    uint8_t _rsv68[0x18];       /*!< +0x68..+0x7F                        */
    uint16_t session_token;     /*!< +0x80 reg 0x40 rental session token (info 0x59 sets, 0x58 clears) */
    uint8_t _rsv82[2];          /*!< +0x82..+0x83                        */
    /* ---- Speed-profile table +0x84..+0x93 (block-written + index-addressable) ---- */
    union {
        struct {
            uint16_t fault_code;    /*!< +0x84 reg 0x42 profile ceiling / DP_PROF_CEIL */
            uint16_t prof_86;       /*!< +0x86 reg 0x43 control DP profile cell */
            uint16_t prof_88;       /*!< +0x88 reg 0x44 control DP profile cell */
            uint16_t prof_8a;       /*!< +0x8A reg 0x45 control DP profile cell */
            uint16_t speed_limit;   /*!< +0x8C reg 0x46 base output ceiling, low byte km/h (20/22/25/35); CRC-gated,
                                         gear DRIVE_C clamps to it (control_build_drive_cmd) */
            uint16_t gear_a_ceiling; /*!< +0x8E reg 0x47 gear DRIVE_A output ceiling, low byte km/h (clamped to
                                        MP_CEIL2) */
            uint16_t speed_limit_2;  /*!< +0x90 reg 0x48 two packed per-gear ceilings: high byte = gear DRIVE_B (clamped
                                          to MP_CEIL3), low byte = gear DRIVE_C (clamped to \ref speed_limit); low byte
                                          also written by the reg 0x46 CRC gate alongside \ref speed_limit */

            union {
                uint16_t auto_off_min;  /*!< +0x92 reg 0x49 auto power-off delay, minutes (x6000 ticks) */
                int16_t auto_off_min_s; /*!< +0x92 signed view (countdown reload)   */
            };
        };
        uint16_t prof_tbl[8];       /*!< +0x84..+0x93 index window (control_drive_param_init) */
    };
    uint8_t _rsv94[2];          /*!< +0x94..+0x95                        */
    uint16_t chg_sched_start;   /*!< +0x96 reg 0x4B scheduled-charging window start, hi = hour, lo = minute */
    uint16_t chg_sched_end;     /*!< +0x98 reg 0x4C scheduled-charging window end, hi = hour, lo = minute */
    uint16_t in2_9a;            /*!< +0x9A reg 0x4D, sent in CAN 0x481 bytes 4..5 (purpose unknown) */
    uint8_t _rsv9c[0x0E];       /*!< +0x9C..+0xA9                        */
    union {
        uint16_t soc;           /*!< +0xAA reg 0x55 battery state of charge, % (both packs averaged when SF_DUAL_PACK; >= 1) */
        int16_t soc_s;          /*!< +0xAA signed view (range estimate, charge LED)          */
    };
    uint8_t _rsvac[2];          /*!< +0xAC..+0xAD                        */
    union {
        uint16_t speed_src;         /*!< +0xAE can rx 0x211 W            */
        int16_t speed_src_s;        /*!< +0xAE signed view: odometer distance integrand, report speed_kmh source */
    };
    uint16_t lock_state;        /*!< +0xB0 lock sub-state (faultmon FM_ERR_MIRROR) */
    uint16_t event_ack_reset;   /*!< +0xB2 control event unlatch                */
    int16_t gear;               /*!< +0xB4 gear / drive-profile selector (config ride-mode src) */
    uint16_t dp_b6;             /*!< +0xB6 control DP_WB6                 */
    uint16_t dp_b8;             /*!< +0xB8 control DP_WB8                 */
    uint16_t brake_light_flash; /*!< +0xBA reg 0x5D: non-zero selects the flashing brake light (MODE_LIGHT_BRAKE_FLASH) */
    int16_t range_reserve;      /*!< +0xBC reg 0x5E range estimate x 0.8                     */
    int16_t range_est;          /*!< +0xBE reg 0x5F remaining range = full-charge range x SoC / 100 */
    uint8_t _rsvc0[2];          /*!< +0xC0..+0xC1                        */
    uint16_t passcode_num;      /*!< +0xC2 reg 0x61 numeric passcode (4 digits, d0*1000+d1*100+d2*10+d3) */
    union {
        uint32_t session_dist;       /*!< +0xC4 regs 0x62/0x63 session distance count (u32, saturating; display = /10); zeroed by rental session start/stop (info 0x57/0x59, 0x58) */
        uint16_t session_dist_reg[2]; /*!< +0xC4 the same count as registers 0x62 (lo) / 0x63 (hi) (CAN 0x480) */
    };
    uint32_t session_total_time; /*!< +0xC8 session time in the ride run states (200-tick units, u32 saturating; cleared with session_dist) */
    uint32_t session_ride_time; /*!< +0xCC session ride time (MODE MF_ACTIVE set; 200-tick units, u32 saturating; cleared with session_dist) */
    int16_t trip_dist;          /*!< +0xD0 trip distance count (s16, saturating; zeroed at boot by vcu_config_init) */
    int16_t trip_total_time;    /*!< +0xD2 trip time in the ride run states (200-tick units, s16 saturating; zeroed at boot) */
    int16_t trip_ride_time;     /*!< +0xD4 trip ride time (MF_ACTIVE set; 200-tick units, s16 saturating; zeroed at boot) */
    uint16_t pack_temp;         /*!< +0xD6 pack thermistor temp; 0x5AA5 = invalid/init sentinel */
    uint8_t _rsvd8[2];          /*!< +0xD8..+0xD9                        */
    uint16_t derived_speed;     /*!< +0xDA trip average speed = trip_dist*360/trip_ride_time (odometer) / control DP_WDA */
    uint8_t _rsvdc[4];          /*!< +0xDC..+0xDF                        */
    uint16_t dir_bits;          /*!< +0xE0 direction bits (packed into wc) */
    /* ---- Passcode / anti-theft registers +0xE2..+0xEB ---- */
    uint16_t passcode_key[2];   /*!< +0xE2 regs 0x71/0x72 key-sequence passcode (packed key nibbles), two register slots (lo, hi); use vcu_in2_passcode_get/set */
    uint8_t _rsve6[2];          /*!< +0xE6..+0xE7                        */
    uint16_t alarm_sensitivity; /*!< +0xE8 reg 0x74 anti-theft sensitivity 0/1/2 (motion-count thresholds 0x77/0x45/0x0E) */
    uint16_t wobble_sens;       /*!< +0xEA reg 0x75 wobble warning: 0 off, 1 threshold 250, 2 threshold 170 (ride_assist_tick); zeroed by control_drive_param_init */
} vcu_in2_t;

/* IN2 register bits (reg 0x1C status, 0x1D / 0x1E settings, 0x1F charge settings). */
#define SF_RUN           0x0001u /*!< status_flags b0: set in ride / charging (drive pipeline running) */
#define SF_CHARGING      0x0004u /*!< status_flags b2: charging (charger detect)                   */
#define SF_IDLE          0x0020u /*!< status_flags b5: set in state 0 (idle)                       */
#define SF_DUAL_PACK     0x0200u /*!< status_flags b9: second battery pack present (SoC average, range x1.55) */
#define SF_SESSION       0x0800u /*!< status_flags b11: rental session active (info 0x59 sets, 0x58 clears) */
#define CFG_LOCK_EN      0x0004u /*!< cfg_flags b2: passcode lock enabled                          */
#define CFG_IMPERIAL     0x0008u /*!< cfg_flags b3: imperial units (km -> mi on the displays)       */
#define CFG_WALK_EN      0x0010u /*!< cfg_flags b4: walk-mode bit (write side effect: gear 1 -> 2)  */
#define CFG_PASSCODE_NUM 0x0080u /*!< cfg_flags b7: numeric passcode (4 digits, reg 0x61)           */
#define CFG_PASSCODE_KEY 0x0100u /*!< cfg_flags b8: key-sequence passcode (reg 0x71/0x72)           */
#define CFG_ALARM_EN     0x8000u /*!< cfg_flags b15: anti-theft alarm enabled                       */
#define CFG2_ACK_BEEP    0x0001u /*!< cfg_flags2 b0: ack beep on a register write                  */
#define CFG2_RIDE_ASSIST 0x0002u /*!< cfg_flags2 b1: toggled by a reg 0x75 write; cleared by the LED matrix in row 0 */
#define CFG2_CLIMB_BOOST 0x0004u /*!< cfg_flags2 b2: climb boost enabled (ride_assist_tick)       */
#define CFG2_ASSIST_B3   0x0008u /*!< cfg_flags2 b3: cleared by the LED matrix in every row; no reader  */
#define CFG2_OVERSPEED_REGEN 0x0010u /*!< cfg_flags2 b4: coasting over-speed regen enabled      */
#define CFG2_PASSCODE_CANCEL 0x0040u /*!< cfg_flags2 b6: cancels a passcode change in progress      */
#define CFG2_GEAR3_EN    0x0100u /*!< cfg_flags2 b8: gear 3 enabled                                */
#define CFG2_GEAR4_EN    0x0200u /*!< cfg_flags2 b9: gear 4 enabled                                */
#define CFG2_THROTTLE_PERMIT 0x0800u /*!< cfg_flags2 b11: throttle permitted                       */
#define CFG2_SHOW_TOTAL  0x2000u /*!< cfg_flags2 b13: distance display shows the session total, not the trip */
#define CFG2_LAMP_FLASH  0x4000u /*!< cfg_flags2 b14: lamp flash allowed                           */
#define REG2F_BIT0       0x0001u /*!< in2_5e (reg 0x2F) b0: cleared by the drive-param seed and the LED matrix */
#define REG2F_TURN_CANCEL_OFF 0x0004u /*!< in2_5e (reg 0x2F) b2: set holds the turn-signal auto-cancel reset; set in rows 1/2 */
#define CHG_DISP_EN      0x0002u /*!< chg_cfg b1: charge LED display enabled                       */
#define CHG_SCHED_EN     0x0010u /*!< chg_cfg b4: scheduled charging enabled                       */
#define CHG_SCHED_ONCE   0x0020u /*!< chg_cfg b5: single-shot schedule (cleared with b4/b7 at window start) */
#define CHG_SEEN         0x0040u /*!< chg_cfg b6: charger seen                                     */
#define CHG_KEYHOLD      0x0080u /*!< chg_cfg b7: set by a 30-tick key hold on the charge screen   */
#define CHG_TILT_LATCH_EN 0x0100u /*!< chg_cfg b8: IMU tilt latch runs (sensor.c)                     */
#define ALARM_TIMEOUT_EN 0x0200u /*!< chg_cfg b9: alarm auto-timeout                               */

/** \brief Read the key-sequence passcode from its two IN2 register slots (regs 0x71/0x72). */
#define vcu_in2_passcode_get(in2) \
    ((uint32_t)(in2).passcode_key[0] | ((uint32_t)(in2).passcode_key[1] << 16))

/** \brief Store a key-sequence passcode into the two IN2 register slots (lo, hi). */
#define vcu_in2_passcode_set(in2, code)                  \
    do {                                                 \
        uint32_t c_ = (uint32_t)(code);                  \
        (in2).passcode_key[0] = (uint16_t)(c_ & 0xFFFFu); \
        (in2).passcode_key[1] = (uint16_t)(c_ >> 16);     \
    } while (0)


/**
 * \brief           Scratch / build block @ g_scr: DTC + event masks, telem
 * \note            Not packed (every multi-byte field is naturally aligned). The low
 *                      block is ALSO the Ninebot register page 1, walked by halfword
 *                      index (`g_scr.reg[n]`, report_u16 and the register-write handler),
 *                      overlapping the named scalars; the +0x7A status-report payload
 *                      base pointer is handed to the framer by raw address.
 */
typedef struct {
    union {
        uint16_t reg[0xD6]; /*!< +0x000..+0x1AB Ninebot register page 1: reg n = SCR + 2n (report_u16, nb register writes) */
        struct {
            uint8_t _rsv00[2];        /*!< +0x00..+0x01                  */
            uint16_t power_half;      /*!< +0x02 control power/2000      */
            uint8_t _rsv04[2];        /*!< +0x04..+0x05                  */
            uint16_t power_sign;      /*!< +0x06 control power sign       */
            uint16_t commto_mirror;   /*!< +0x08 control commto mirror    */
            uint16_t bat2_present;    /*!< +0x0A battery 2 present (reg 5, busproto signal [29], 1 bit): set by its CAN frames, dropped by vcu_can_battery2_presence_tick */
            uint16_t soc_raw_nz;      /*!< +0x0C BMS SoC % (CAN 0x420 d[4..5] / 10), 0 -> 1        */
            uint16_t bat2_soc;        /*!< +0x0E battery-2 SoC % (reg 7, busproto signal [16], 8 bit): copy of g_stat2.soc while present */
            uint16_t drive_oneshot;   /*!< +0x10 control / vehicle SCR_DRIVE_ONESHOT */
            uint16_t signal_12;       /*!< +0x12 report signal (busproto descriptor [26], 1 bit) */
            uint8_t _rsv14[2];        /*!< +0x14..+0x15                  */
            uint16_t dist_is_trip;    /*!< +0x16 odometer SCR_DIST_IS_TRIP    */
            int16_t range_disp;       /*!< +0x18 displayed range: range_est / 10 (x 0.62137 in imperial) */
            int16_t ridetime_disp;    /*!< +0x1A vehicle SCR_RIDETIME_DISP   */
            uint8_t _rsv1c[2];        /*!< +0x1C..+0x1D                  */
            int16_t distance_disp;    /*!< +0x1E odometer SCR_DISTANCE_DISP   */
            uint16_t canrx_423;       /*!< +0x20 can rx 0x423         */
            uint8_t _rsv22[2];        /*!< +0x22..+0x23                  */
            uint16_t dtc_mask0;       /*!< +0x24 DTC codes 10..25         */
            uint16_t dtc_mask1;       /*!< +0x26 DTC codes 26..41         */
            uint16_t dtc_mask2;       /*!< +0x28 DTC codes 42..57         */
            uint16_t event_mask_lo;   /*!< +0x2A pending events 1..16     */
            uint16_t event_mask_hi;   /*!< +0x2C pending events 17..32    */
            uint16_t soc_report;      /*!< +0x2E reported SoC copy (= IN2->soc)                    */
            uint8_t _rsv30[0x0C];     /*!< +0x30..+0x3B                  */
            uint16_t telem_3c;        /*!< +0x3C report telem (== IN2+0x23C) */
            uint16_t lock_req;        /*!< +0x3E lock request latch       */
            int16_t gest_busy;        /*!< +0x40 gesture-in-progress latch (== IN2+0x240 telem mirror) */
            uint16_t submode_code;    /*!< +0x42 ridemode SCR_SUBMODE_CODE */
            uint16_t lock_status_mirror; /*!< +0x44 control lock status mirror */
            uint16_t gest_payload;    /*!< +0x46 page-1 reg 0x23: gesture/ride report payload (ctrl op 4 arg 0x20 writes 0..3) */
            union {
                int16_t mileage;      /*!< +0x48 odometer SCR_MILEAGE (signed test)        */
                uint16_t mileage_u;   /*!< +0x48 unsigned view (report_onchange_24 payload) */
            };
            union {
                int16_t ui_screen;    /*!< +0x4A page-1 reg 0x25: dashboard screen id sent by nb_commit_mode: 0x10 idle, 0x11 passcode, 4 numeric passcode, 5 alarm, 0x12 charging, 0x13, 0x14 session */
                uint16_t ui_screen_u; /*!< +0x4A unsigned view (the dispatcher's register write / commit) */
            };
            uint16_t signal_4c;       /*!< +0x4C report signal (busproto descriptor [31], 1 bit) */
            uint8_t _rsv4e[2];        /*!< +0x4E..+0x4F                  */
            uint16_t soc_combined;    /*!< +0x50 SoC % of both packs (average when SF_DUAL_PACK), >= 1 */
            uint16_t ilk_publish;     /*!< +0x52 faultmon ILK_PUBLISH      */
            uint8_t _rsv54[2];        /*!< +0x54..+0x55                  */
            uint16_t drive_fault_req; /*!< +0x56 drive-fault request       */
            uint16_t signal_58;       /*!< +0x58 report signal (busproto descriptor [27], 1 bit) */
            uint16_t signal_5a;       /*!< +0x5A report signal (busproto descriptor [28], 1 bit) */
            uint8_t _rsv5c[0x14];     /*!< +0x5C..+0x6F                  */
            uint16_t telem_70;        /*!< +0x70 report telem (== IN2+0x270) */
            uint16_t signal_72;       /*!< +0x72 report signal (busproto descriptor [23], 1 bit) */
            uint16_t signal_74;       /*!< +0x74 report signal reg 0x3A (busproto descriptor [21], 4 bit); the odometer display clears it */
            uint8_t _rsv76[0x06];     /*!< +0x76..+0x7B (+0x7A = status-report payload base ptr) */
            uint32_t status_pack;     /*!< +0x7C report/nb status-report pack word */
            int16_t can_active;       /*!< +0x80 report SCR_CAN_ACTIVE / nb OTA abort */
            uint8_t _rsv82[0x10];     /*!< +0x82..+0x91                  */
            union {
                uint16_t signal_92;      /*!< +0x92 report signal (busproto descriptor [12], 16 bit) */
                uint16_t climb_boost_on; /*!< +0x92 same cell: 1 while the climb boost is engaged */
            };
            uint16_t signal_94;       /*!< +0x94 report signal (busproto descriptor [33], 1 bit) */
            union {
                uint16_t lock_pack_src0;     /*!< +0x96 busproto signal [34] (1 bit)                        */
                uint16_t overspeed_regen_on; /*!< +0x96 same cell: 1 while the over-speed regen is up */
            };
            union {
                uint16_t lock_pack_src1;  /*!< +0x98 busproto signal [35] (1 bit)                           */
                uint16_t wobble_alert_on; /*!< +0x98 same cell: 1 while the wobble warning is up */
            };
            uint16_t signal_9a;       /*!< +0x9A report signal (busproto descriptor [36], 1 bit) */
            uint8_t _rsv9c[6];        /*!< +0x9C..+0xA1                  */
            int16_t light_state;      /*!< +0xA2 vehicle SCR_LIGHT_STATE      */
            uint8_t _rsva4[0x1C];     /*!< +0xA4..+0xBF                  */
            uint16_t subsys_enable;   /*!< +0xC0 live subsystem enable mask (bit1 = A / PB10, bit0 = B / PB0), reported as VCU_RPT_ARG_SUBSYS */
            uint8_t _rsvc2[2];        /*!< +0xC2..+0xC3                  */
            uint16_t passcode_status;  /*!< +0xC4 page-1 reg 0x62: passcode entry status word (8 = match, 4|try bits = wrong, 0x0C = lock-out) */
            uint16_t passcode_display; /*!< +0xC6 page-1 reg 0x63: numeric passcode display word     */
            uint16_t countdown_mirror; /*!< +0xC8 countdown / 100 mirror   */
            uint8_t _rsvca[0xD8];     /*!< +0xCA..+0x1A1                 */
            uint16_t idle_report;     /*!< +0x1A2 report idle flag; info op 0x11 (reboot) clears it */
            uint8_t _rsv1a4[4];       /*!< +0x1A4..+0x1A7                */
            uint16_t countdown_report; /*!< +0x1A8 report countdown report   */
            uint16_t mode_status;     /*!< +0x1AA mode/lighting status     */
        };
    };
} vcu_scr_t;


/**
 * \brief           Sensor/status block @ g_stat
 * \note            PLAIN (not packed). Its own aligned global places every multi-byte
 *                      member on its natural boundary (status_packed @+0x1FC is 4-aligned).
 */
typedef struct {
    uint8_t _rsv00[0x1C];   /*!< +0x00..+0x1B               */
    uint16_t canrx_421a;    /*!< +0x1C can rx 0x421      */
    uint16_t canrx_420a;    /*!< +0x1E can rx 0x420      */
    uint8_t _rsv20[6];      /*!< +0x20..+0x25               */
    uint16_t canrx_421b;    /*!< +0x26 can rx 0x421      */
    uint8_t _rsv28[0x8A];   /*!< +0x28..+0xB1               */
    uint16_t canrx_422a;    /*!< +0xB2 can rx 0x422      */
    uint8_t _rsvb4[0x30];   /*!< +0xB4..+0xE3               */
    uint16_t canrx_423a;    /*!< +0xE4 can rx 0x423      */
    uint8_t _rsve6[0x1E];   /*!< +0xE6..+0x103              */
    uint16_t chg_limit;     /*!< +0x104 charge limit, low byte % (CAN 0x423 d[2..3])      */
    uint8_t _rsv106[6];     /*!< +0x106..+0x10B             */
    uint16_t canrx_310a;    /*!< +0x10C can rx 0x310     */
    uint8_t _rsv10e[0x0A];  /*!< +0x10E..+0x117             */
    int16_t curr_sample;    /*!< +0x118 I sample (CAN 0x420 d[0..1])  */
    int16_t volt_sample;    /*!< +0x11A V/speed sample (CAN 0x420 d[2..3]) */
    uint8_t _rsv11c[2];     /*!< +0x11C..+0x11D             */
    union {
        int16_t soc;            /*!< +0x11E battery SoC %, CAN 0x420 d[4..5] / 10 (signed compares) */
        uint16_t soc_u;         /*!< +0x11E unsigned view (CAN 0x420 write; telemetry / CAN 0x203) */
    };
    uint8_t _rsv120[2];     /*!< +0x120..+0x121             */
    int16_t curr_limit;     /*!< +0x122 current ceiling (/100)  */
    uint16_t status_word_a; /*!< +0x124 drive-permit flags (bit6 = drive-permit) */
    uint16_t status_word_b; /*!< +0x126 drive-fault flags (bit3 = fault/limp)    */
    uint16_t canrx_423b;    /*!< +0x128 can rx 0x423 (W->SCR+0x20) */
    uint16_t canrx_310b;    /*!< +0x12A can rx 0x310     */
    uint16_t temp_array[8]; /*!< +0x12C..+0x13B can rx 0x500 temps (ARRAY, index-based) */
    uint8_t _rsv13c[0x56];  /*!< +0x13C..+0x191             */
    uint16_t canrx_311a;    /*!< +0x192 can rx 0x311     */
    uint8_t _rsv194[0x0E];  /*!< +0x194..+0x1A1             */
    uint16_t canrx_310_421; /*!< +0x1A2 can rx 0x310/0x421 */
    uint8_t _rsv1a4[0x34];  /*!< +0x1A4..+0x1D7             */
    uint16_t canrx_311b;    /*!< +0x1D8 can rx 0x311     */
    uint16_t canrx_311c;    /*!< +0x1DA can rx 0x311     */
    uint8_t _rsv1dc[0x0C];  /*!< +0x1DC..+0x1E7             */
    uint16_t canrx_422b;    /*!< +0x1E8 can rx 0x422     */
    uint8_t _rsv1ea[0x0A];  /*!< +0x1EA..+0x1F3             */
    uint16_t gear_report;   /*!< +0x1F4 ridemode / ridemode gear report */
    uint8_t _rsv1f6[4];     /*!< +0x1F6..+0x1F9             */
    uint16_t canrx_423c;    /*!< +0x1FA can rx 0x423     */
    union {
        uint32_t status_packed; /*!< +0x1FC packed status-report source word (== g_stat+0x1FC) */
        struct {
            uint16_t status_lo; /*!< +0x1FC canrx 0x424 low half  */
            uint16_t status_hi; /*!< +0x1FE canrx 0x424 high half (NOT OOB) */
        };
    };
} vcu_stat_t;


/**
 * \brief           Drive/charge pacing + faultmon interlock block @ g_tmr
 * \note            0x30 bytes. The +0x00..+0x06 and +0x0B..+0x22 cells are
 *                      the control input-edge latches and the faultmon drive-interlock
 *                      (ilk_*) state machine; +0x07..+0x0A, +0x18/+0x1A, +0x24/+0x28 keep
 *                      their existing names.
 */
typedef struct {
    uint8_t lock_subdiv;     /*!< +0x00 control lock subdivider        */
    uint8_t brake_full_active;    /*!< +0x01 full-weight brake lever (PA4) active      */
    uint8_t brake_half_active;    /*!< +0x02 half-weight brake lever (PA1) active      */
    uint8_t throttle_active;      /*!< +0x03 throttle (PA0) active                     */
    uint8_t edge_prev_brake_full; /*!< +0x04 previous brake_full_active (edge detect)  */
    uint8_t edge_prev_brake_half; /*!< +0x05 previous brake_half_active (edge detect)  */
    uint8_t latch_reported;  /*!< +0x06 faultmon LATCH_REPORTED        */
    uint8_t lock_out;        /*!< +0x07 lock output gate               */
    uint8_t lock_div;        /*!< +0x08 lock subdivider                */
    uint8_t ovt_flag;        /*!< +0x09 over-temp request flag         */
    uint8_t ovt_active;      /*!< +0x0A over-temp latch                */
    uint8_t ilk_ref_spd;     /*!< +0x0B faultmon interlock ref value (engage) */
    uint8_t ilk_state;       /*!< +0x0C faultmon interlock state 0/1/2  */
    uint8_t ilk_ref_cmd;     /*!< +0x0D faultmon interlock ref speed/cmd */
    uint8_t ilk_fault_flag;  /*!< +0x0E faultmon set on mismatch/abort (was ilk_flag_e) */
    uint8_t ilk_zero_flag;   /*!< +0x0F faultmon set when speed==0 (was ilk_flag_f) */
    uint8_t step_state;      /*!< +0x10 control STEP_STATE             */
    uint8_t _rsv11;          /*!< +0x11                                */
    uint16_t lock_packed_ctr; /*!< +0x12 control lock packed counter   */
    uint16_t lock_ack_dwell; /*!< +0x14 control lock ack dwell         */
    uint16_t latch_arm_tmr;  /*!< +0x16 faultmon LATCH_ARM_TMR         */
    uint16_t ovt_timer;      /*!< +0x18 over-temp dwell timer          */
    uint16_t guard_timer;    /*!< +0x1A guard debounce timer           */
    uint16_t ilk_cnt;        /*!< +0x1C faultmon interlock mismatch count */
    uint16_t ilk_tmr;        /*!< +0x1E faultmon interlock 300-tick timeout */
    uint16_t step_ramp;      /*!< +0x20 control STEP_RAMP              */
    uint16_t poweroff_countdown; /*!< +0x22 report power-off countdown   */
    uint32_t retrig_timer;   /*!< +0x24 drive-active retrigger window (reload 400) */
    uint32_t subdiv_timer;   /*!< +0x28 drive-active subdivider (reload 0x32)      */
    uint32_t step_tick;      /*!< +0x2C control STEP_TICK              */
} vcu_tmr_t;


/**
 * \brief           Charge-detect flag block @ chgstate (g_chg)
 * \note            f2/f3 = charger present A/B, f5 = charge-detected latch, f6/f7 =
 *                      per-channel sample counters. f0/f1 and f4 are raw cells (f4 unused by
 *                      the FSM).
 */
typedef struct {
    uint8_t f0;     /*!< +0x00                        */
    uint8_t f1;     /*!< +0x01                        */
    uint8_t f2;     /*!< +0x02 charger present A        */
    uint8_t f3;     /*!< +0x03 charger present B        */
    union {
        uint8_t _rsv04; /*!< +0x04 (unused by the charge FSM)                                     */
        uint8_t f4;     /*!< +0x04 same cell: light hazard latch (ridemode alt_latch); turn-signal auto-cancel */
    };
    uint8_t f5;     /*!< +0x05 charge-detected latch    */
    uint8_t f6;     /*!< +0x06 sample counter A         */
    uint8_t f7;     /*!< +0x07 sample counter B         */
    uint16_t timer; /*!< +0x08 debounce / dwell timer   */
} vcu_chg_t;


/**
 * \brief           Charge averaging accumulators @ chgbuf
 * \note            +0x08 holds the subsystem-mask report cache (written and read only by
 *                      report_subsys_mask_set).
 */
typedef struct {
    uint16_t acc[4];          /*!< +0x00..+0x07 charge FSM accumulators / counters (control.c) */
    uint16_t subsys_reported; /*!< +0x08 last reported subsystem A/B enable mask (report_subsys_mask_set) */
    uint8_t _rsv0a[2];        /*!< +0x0A..+0x0B                         */
    uint32_t count;           /*!< +0x0C relay dwell counter            */
} vcu_chgbuf_t;


/* ========================================================================= */
/* Typed layouts + enums for the remaining shared blackboard blocks.         */
/* ========================================================================= */

/* ------------------------------------------------------------------------- */
/* Shared enums.                                                             */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Top-level vehicle FSM state @ g_state_block +0x00
 * \note            The vehicle FSM task switches on this 0..8 byte; each state is named
 *                      after what its handler does. Only states 3 (ride) and 7 (charging) are
 *                      "reportable" (the vehicle.c / report.c FSM run gate).
 */
typedef enum {
    VCU_FSM_IDLE = 0,           /*!< powered, not started: wake/app/button -> 1, idle timeout -> 4, charger in a session -> 8 */
    VCU_FSM_POWER_ON = 1,       /*!< ~10-tick power-on: config init, beep, first screen; -> 3 (no lock / unlocked) or 2 (locked) */
    VCU_FSM_LOCKED = 2,         /*!< passcode entry (key-sequence or numeric); unlock -> 3, 3-try lock-out alarm -> 6 */
    VCU_FSM_RIDE = 3,           /*!< the full drive pipeline (reportable); charger in a session -> 7 */
    VCU_FSM_POWER_DOWN = 4,     /*!< outputs off, screen 0, power-latch release; -> 5, or -> 8 when charging */
    VCU_FSM_SLEEP = 5,          /*!< low-power wait for a wake source; -> 6 (motion wake) or 0 */
    VCU_FSM_ALARM = 6,          /*!< anti-theft motion guard / siren; button or app start -> 0, else -> 4 */
    VCU_FSM_CHARGING = 7,       /*!< charger on during a session: drive pipeline with the throttle blocked (reportable) */
    VCU_FSM_CHARGE_STANDBY = 8  /*!< awake for charging after power-down (charge screens, schedule start time) */
} vcu_fsm_state_t;

/** \brief Start request @ g_vs.start_req (VS+0x03), set in state 0 and read by state 1. */
typedef enum {
    VS_START_NONE = 0,     /*!< no start pending                                       */
    VS_START_UNLOCKED = 1, /*!< start, already unlocked (app / remote unlock)          */
    VS_START_LOCKED = 2,   /*!< start locked: passcode entry first (state 2)          */
    VS_START_NO_LOCK = 3   /*!< start with the passcode lock disabled                  */
} vcu_start_req_t;

/**
 * \brief           Region of the vehicle @ g_vs +0x00 (VS->region): the live row of the drive tables
 * \note            Selected at boot from the serial's region code (config_parse_model,
 *                      CONFIG_REGION_CODES): P -> GLOBAL, E -> EUROPE, D -> GERMANY. It indexes the
 *                      3-row drive tables (g_mode_table, the speed-limit profiles). It is NOT the
 *                      stored ride mode WALK=1/ECO=2/SPORT=3/POWER=4 (vcu_ride_mode_t in control.h,
 *                      IN2+0xB4); the stored-gear -> row map is in ridemode.c. The same cell also
 *                      reads as idle/unlocked (value 0) inside the lock FSM, so each access site
 *                      uses the name that fits its context. GLOBAL is also the default row (the
 *                      switch `default`).
 */
typedef enum {
    VCU_REGION_GLOBAL = 0, /*!< row 0: Global, 50 km/h (region code P; also the default / lock-idle row) */
    VCU_REGION_EUROPE = 1, /*!< row 1: Europe, 25 km/h (region code E)                                 */
    VCU_REGION_GERMANY = 2 /*!< row 2: Germany, 20 km/h (region code D)                                */
} vcu_region_t;

/* ------------------------------------------------------------------------- */
/* Overlay structs for the remaining shared bases.                            */
/* ------------------------------------------------------------------------- */

/**
 * \brief           FSM state block @ g_state_block: one 0..8 state byte + id/odo words
 * \note            0x18 bytes. +0x00 is the FSM state byte read by every
 *                      module; +0x0C..+0x17 are the report unit-id / odometer odo / charge words.
 */
typedef struct {
    uint8_t fsm_state;  /*!< +0x00 vehicle FSM state (see \ref vcu_fsm_state_t) */
    uint8_t _rsv01[0x0B]; /*!< +0x01..+0x0B                                   */
    uint32_t uid_cksum; /*!< +0x0C report STATE_UID_CKSUM (also the 0x5F unlock credential; nb reads same cell) */
    uint32_t odo_subcnt; /*!< +0x10 odometer STATE_ODO_SUBCNT                     */
    uint32_t dist_subacc; /*!< +0x14 odometer distance integrator (sum of speed_src per tick, mod DIST_UNIT_PERIOD) */
} vcu_state_block_t;


/**
 * \brief           LOW-region lone scalars @ g_lowglob (NOT part of g_state_block)
 * \note            These cells are shared cross-module but are their own named globals,
 *                      not fields of \ref vcu_state_block_t; they are modelled as one
 *                      block-global. +0x08..+0x09 is reserved.
 */
typedef struct {
    uint8_t beep_seq_id;       /*!< +0x00 beep pattern selector (buzzer_pattern_t) */
    uint8_t _rsv01;            /*!< +0x01                                   */
    uint16_t beep_seq_tick;    /*!< +0x02 buzzer sequence tick counter */
    uint8_t active_event_id;   /*!< +0x04 active event/alarm id (charging CHG_EVENT_ACTIVE) */
    uint8_t active_error_code; /*!< +0x05 active fault-code mirror (faultmon/charging/ridemode) */
    uint16_t esc_commloss_cnt; /*!< +0x06 ticks since the last ESC-group CAN frame (can.c zeroes on RX; faultmon counts to 200 -> DTC 10) */
    uint8_t _rsv08[2];         /*!< +0x08..+0x09 reserved                    */
    uint16_t fm_uv_cnt;        /*!< +0x0A faultmon fault-0x19 UV pre-arm counter (to 1000/6000) */
} vcu_lowglob_t;


/**
 * \brief           Latched prev-state / debounce block @ g_prev
 * \note            A bank of per-check latch flags (+0x00..+0x06) with matching
 *                      debounce counters, the latched previous FSM state (+0x07), and
 *                      the mode-change counter (+0x1a). Flag/counter pairings follow
 *                      vehicle.c's comm-fault / timeout guards.
 */
typedef struct {
    uint8_t commfault_a_latch; /*!< +0x00 comm/bus-fault guard A latch   */
    uint8_t commfault_a_arm;   /*!< +0x01 comm-fault guard A armed       */
    uint8_t commfault_b_latch; /*!< +0x02 comm/bus-fault guard B latch   */
    uint8_t chk_c_latch;       /*!< +0x03 timeout check C latch          */
    uint8_t chk_d_latch;       /*!< +0x04 timeout check D latch          */
    uint8_t chk_e_latch;       /*!< +0x05 timeout check E latch          */
    uint8_t chk_e_phase;       /*!< +0x06 check E phase companion        */
    uint8_t prev_state;        /*!< +0x07 latched previous FSM state     */
    uint16_t ctr_08;           /*!< +0x08 blink/divider counter          */
    uint16_t ctr_0a;           /*!< +0x0A counter (wraps at 0x14)        */
    uint16_t ctr_commfault_b;  /*!< +0x0C comm-fault B counter (clamp 500) */
    uint16_t ctr_c;            /*!< +0x0E check C counter (target 200)   */
    uint16_t charging_oneshot; /*!< +0x10 state-7 entry one-shot: screen 1 + status report */
    uint16_t ctr_d;            /*!< +0x12 check D counter                */
    uint16_t ctr_d2;           /*!< +0x14 check D secondary counter      */
    uint16_t ctr_e;            /*!< +0x16 check E counter (target 1000)  */
    uint16_t ctr_e2;           /*!< +0x18 check E secondary counter      */
    uint16_t mode_counter;     /*!< +0x1A mode-change counter (clamp 300) */
    /* ---- power-down button and charger-edge state ---- */
    uint8_t pd_button_held;    /*!< +0x1C power-down: PA10 not released since entry; a press cancels only once it is 0 */
    uint16_t chg_bit_prev;     /*!< +0x1E last STAT+0x126 bit3 (charger) value seen by the task, 0 or 8 */
    uint16_t chg_plug_edge;    /*!< +0x20 charger plugged-in edge latch: idle wake / charge standby; cleared on a state change */
} vcu_prev_t;


/**
 * \brief           ESC command-staging / mode register block @ g_mode
 * \note            The control pipeline stages the ESC drive command here through the
 *                      named fields; `frame[]` is the raw byte view of the same block.
 */
typedef struct {
    union {
        uint8_t frame[0x18]; /*!< +0x00..+0x17 byte view (control marshals drive_cmd[0..7]) */
        struct {
            uint8_t cmd_lead;     /*!< +0x00 command lead byte (output ceiling << 1) */
            uint8_t error_code;   /*!< +0x01 active error code (CAN 0x20C getter copies g_lowglob.active_error_code; byte 1 of the 0x100 frame) */
            uint8_t cmd_param;    /*!< +0x02 throttle full-scale param = 0xC8 (written unconditionally) */
            uint8_t _rsv03;       /*!< +0x03                                        */
            uint16_t word04;      /*!< +0x04 drive word (per-gear 0x62/0xA3/0x147/0x41/0x1EB, 0x3FFF when mode_flags b7) */
            uint16_t word06;      /*!< +0x06 drive word (control writes 0x0E)       */
            uint16_t drive_output_word; /*!< +0x08 lo=throttle 0..200, hi=brake     */
            uint16_t mode_flags;  /*!< +0x0A mode flag word (bit0/bit2/bit9)         */
            uint16_t curr_word;   /*!< +0x0C scaled current command (lo) + dir bits [15:14] / report read */
            uint16_t we_word;     /*!< +0x0E energy/limit word (lo, voltage-derated; hi = 0x5A) / report read */
            uint16_t enable_flags; /*!< +0x10 b0/b1 lamp A/B enable, b9..11 rear-light mode (\ref vcu_mode_light_t), b12/b14 set by staging */
            uint8_t canrx_12;     /*!< +0x12 can rx 0x311                        */
            uint8_t spd_cmp;      /*!< +0x13 speed threshold / can W                */
            uint8_t canrx_14;     /*!< +0x14 can rx 0x311                        */
            uint8_t output_cfg;   /*!< +0x15 \ref OCFG_CLIMB_BOOST .. \ref OCFG_BRAKE_CAP (was cfg_15) */
            uint8_t cfg_16;       /*!< +0x16 sent with output_cfg as CAN 0x485 bytes 2..3 (read as one u16 with +0x15; no writer) */
            uint8_t cfg_17;       /*!< +0x17 bits0-1 = soc-derived limp/derate flag, bits2-3 drive-fault state (\ref CFG17_DRIVE_FAULT) */
        };
    };
    uint8_t regen_level; /*!< +0x18 regen strength 0 / 22 / 45 from reg 0x70 (3 for over-speed regen), boot 45 */
} vcu_mode_t;

/* MODE.output_cfg bits. */
#define OCFG_CLIMB_BOOST     0x01u /*!< b0: climb boost engaged (ride_assist.c)                              */
#define OCFG_COAST_PRESET    0x02u /*!< b1: no writer found; with throttle 0 selects the coast energy preset  */
#define OCFG_OVERSPEED_REGEN 0x04u /*!< b2: over-speed regen running (ride_assist_tick)                     */
#define OCFG_WOBBLE_ALERT    0x08u /*!< b3: wobble warning up: throttle through the 16-sample average        */
#define OCFG_BRAKE_CAP       0x10u /*!< b4: no writer found; throttle off, brake 15 above 15.0 km/h           */

/* MODE.cfg_17 fields. */
#define CFG17_LOW_SOC           0x03u /*!< b0..1: low-SoC derate flag (control_build_drive_cmd)            */
#define CFG17_DRIVE_FAULT       0x0Cu /*!< b2..3: drive-fault state 0 / 1 tripped / 3 + vbus > 150 */
#define CFG17_DRIVE_FAULT_SHIFT 2u    /*!< bit position of \ref CFG17_DRIVE_FAULT                        */

/* MODE.enable_flags bits 9..11: one 3-bit rear-light mode value, sent in CAN 0x20C bytes
   0..1. Every writer does `(w & 0xF1FF) + (v << 9)` or `w & 0xF1FF`; the field is
   never set or tested bit by bit. Write it with \ref mode_light_set. */
#define MODE_LIGHT_MASK  0x0E00u /*!< enable_flags bits 9..11: rear-light mode */
#define MODE_LIGHT_SHIFT 9u      /*!< bit position of the rear-light mode      */

/** \brief Rear-light mode in MODE.enable_flags bits 9..11. */
typedef enum {
    MODE_LIGHT_OFF = 0,        /*!< off: idle with both light outputs off, error blink off-phase, teardown */
    MODE_LIGHT_BLINK = 1,      /*!< blink: charging below 100 % SoC, anti-theft siren on-phase         */
    MODE_LIGHT_TAIL = 2,       /*!< tail light: regen without brake, or a light output on            */
    MODE_LIGHT_BRAKE = 3,      /*!< brake light: brake, charged (SoC 100 %), error-code blink on-pulse */
    MODE_LIGHT_BRAKE_FLASH = 4 /*!< flashing brake light: brake with reg 0x5D set                    */
} vcu_mode_light_t;


/**
 * \brief           One fault-monitor / input debounce record (12 bytes; g_evt holds 12)
 * \note            Each monitored input owns one record. Not every monitor uses every
 *                      field: the single-stage monitors drive `shift`, the two-stage
 *                      monitors add `hold`, and the PA9 monitor uses `hold` as a 0..500
 *                      sample counter with no shift register. `code` (+0x08) is the
 *                      pending-event byte the consumers (vehicle dispatcher, ridemode,
 *                      control, passcode, charging) test and clear; `phase` (+0x09) is the
 *                      per-monitor latch / escalation phase.
 */
typedef struct {
    uint16_t shift;     /*!< +0x00 debounce shift register (<<1|1, mask 0xF or 0xFFF) */
    uint16_t armed;     /*!< +0x02 set after one clean read; gates confirmation    */
    uint16_t hold;      /*!< +0x04 escalation dwell / sample counter               */
    uint8_t _rsv06[2];  /*!< +0x06..+0x07                                          */
    uint8_t code;       /*!< +0x08 pending event / fault code (1 warn/short, 3 escalated/long, 4 latched) */
    uint8_t phase;      /*!< +0x09 latch / escalation phase                        */
    uint8_t _rsv0a[2];  /*!< +0x0A..+0x0B                                          */
} fm_mon_t;

/**
 * \brief           Index of each record in \ref vcu_evt_t.mon
 * \note            The role names follow the consumers of each `code` byte. "rpt N" =
 *                      the Ninebot event code the vehicle.c
 *                      dispatcher reports when `code == 1` (\ref vcu_report_event_t).
 */
typedef enum {
    FM_MON_POWER_BTN = 0,     /*!< +0x00 PA10 power button, 12-bit two-stage: 1 = press (wake / start / 6->0), 3 = long press (shutdown), rpt 1 */
    FM_MON_SUBSTATE_NEXT = 1, /*!< +0x0C PB6+PB7 both low: ridemode sub-state +1, passcode "next", rpt 8 */
    FM_MON_SUBSTATE_PREV = 2, /*!< +0x18 PA11: ridemode sub-state -1, passcode "prev", vehicle anti-theft latch, rpt 7 */
    FM_MON_INTERLOCK = 3,     /*!< +0x24 PA12, latches 4: faultmon interlock request (1), passcode key 2 (4), rpt 6 */
    FM_MON_GEAR_DOWN = 4,     /*!< +0x30 PB2: ridemode gear-down, rpt 4                         */
    FM_MON_GEAR_UP = 5,       /*!< +0x3C PB11: ridemode gear-up, rpt 3                          */
    FM_MON_PARK = 6,          /*!< +0x48 PB1, two-stage: ridemode park toggle (3), control lock-ack (1), passcode key 4, rpt 2 */
    FM_MON_LIGHT_LO = 7,      /*!< +0x54 PB6 high / PB7 low, two-stage: ridemode low-channel light toggle, numeric passcode "up", rpt 9 */
    FM_MON_LIGHT_HI = 8,      /*!< +0x60 PB7 high / PB6 low, two-stage: ridemode high-channel light toggle, numeric passcode "down", rpt 10 */
    FM_MON_LIGHT_EN = 9,      /*!< +0x6C PA9 count-up: ridemode light enable (1) / disable (5), rpt 5 */
    FM_MON_BRAKE_FULL = 10,   /*!< +0x78 PA4 full-weight brake lever held (code 4 only); `hold` = faultmon gesture index */
    FM_MON_BRAKE_HALF = 11,   /*!< +0x84 PA1 half-weight brake lever held (code 4 only)         */
    FM_MON_COUNT = 12
} fm_mon_id_t;

/**
 * \brief           Event / input-monitor block @ g_evt: 12 \ref fm_mon_t records
 * \note            faultmon owns the debounce cells; every other module only tests and
 *                      clears `mon[i].code`. `word[]` is the bulk-clear view: the vehicle
 *                      FSM wipes record 0 as three U32 stores (vehicle.c), and the event-
 *                      block reset clears all 0x90 bytes.
 */
typedef union {
    fm_mon_t mon[FM_MON_COUNT];          /*!< +0x00..+0x8F the twelve monitor records */
    uint32_t word[FM_MON_COUNT * 3u];    /*!< +0x00..+0x8F bulk-clear view (not a value) */
} vcu_evt_t;


/**
 * \brief           Partial-travel latches @ g_deb, one per analog input
 * \note            Set while the reading is in [0x33, 0x9B); reaching >= 0x9B with the latch
 *                      set clears it and reports the full-travel event.
 */
typedef struct {
    uint8_t thr_travel;        /*!< +0x00 throttle (PA0), event 0x0B                    */
    uint8_t brake_full_travel; /*!< +0x01 full-weight brake lever (PA4), event 0x0D     */
    uint8_t brake_half_travel; /*!< +0x02 half-weight brake lever (PA1), event 0x0C     */
} vcu_deb_t;


/**
 * \brief           Slot1 divider counter block @ g_disp
 * \note            Only +0x06 is used; +0x00..+0x05 are reserved.
 */
typedef struct {
    uint8_t _rsv00[6]; /*!< +0x00..+0x05                         */
    uint16_t divider;  /*!< +0x06 slot1 divider counter         */
} vcu_disp_t;





#define VCU_UART_SLOTS     8u    /*!< USART2 TX ring depth (wraps at 8) */
#define VCU_UART_FRAME_MAX 252u  /*!< slot frame bytes (stride 0x100 minus the u32 length) */

/**
 * \brief           One USART2 TX ring slot (stride 0x100, starts at g_uartctx +0x08)
 * \note            The length sits at ctx + i*0x100 + 0x08 and the frame at
 *                      ctx + i*0x100 + 0x0C. Both enqueuers accept at most 0xFB frame bytes (body 0xF2 + 9).
 */
typedef struct {
    uint32_t len;                      /*!< ctx +0x08 + i*0x100 : on-wire frame length          */
    uint8_t frame[VCU_UART_FRAME_MAX]; /*!< ctx +0x0C + i*0x100 : frame bytes, max 0xFB used    */
} vcu_txslot_t;

/**
 * \brief           USART2 TX frame-ring driver context @ g_uartctx
 * \note            0x808 bytes: an 8-byte header plus 8 slots of \ref vcu_txslot_t.
 *                      free starts at 7 (one slot reserved, classic ring); vcu_usart_init sets
 *                      it, so the ring stays in .bss.
 */
typedef struct {
    uint8_t head;                        /*!< +0x00 read cursor (sequencer advances)          */
    uint8_t tail;                        /*!< +0x01 write cursor (enqueue advances)           */
    uint8_t free;                        /*!< +0x02 free-slot count (enqueue gate), 7 at boot   */
    uint8_t busy;                        /*!< +0x03 enqueue latch (also gates the RX pump)    */
    uint8_t lock;                        /*!< +0x04 sequencer lock                            */
    uint8_t rx_busy;                     /*!< +0x05 RX pump re-entrancy latch                */
    uint8_t _rsv06[2];                   /*!< +0x06..+0x07 unused                             */
    vcu_txslot_t slot[VCU_UART_SLOTS];   /*!< +0x08 TX ring, stride 0x100                     */
} vcu_uartctx_t;


/**
 * \brief           CAN RX scratch message record @ g_canmsg, 20 bytes
 * \note            Byte-for-byte the same layout as \ref vcu_can_msg_t in can.h (the
 *                      record copied through the RX/TX software rings); typed here as an
 *                      overlay so the shared scratch cell can be reached without can.h.
 *
 *                      Our own compact 20-byte ring element, built from the bxCAN FIFO
 *                      registers with the classic StdPeriph CanRxMsg field set (all byte-sized).
 *                      Do NOT alias it to Artery's can_rx_message_type: that type is 28 bytes
 *                      (4-byte id_type/frame_type enums), and the 32-slot rings and this overlay
 *                      all depend on the 20-byte stride.
 */
typedef struct {
    uint32_t std_id; /*!< +0x00 11-bit standard id (used when ide==0) */
    uint32_t ext_id; /*!< +0x04 29-bit extended id (used when ide==1) */
    uint8_t ide;     /*!< +0x08 0 = standard id, 1 = extended id      */
    uint8_t rtr;     /*!< +0x09 0 = data frame, 1 = remote frame      */
    uint8_t dlc;     /*!< +0x0A data length, 0..8                     */
    uint8_t data[8]; /*!< +0x0B payload bytes                         */
    uint8_t fmi;     /*!< +0x13 filter-match index (RX only)          */
} vcu_canmsg_t;


/** \brief Slot4 one-shot report + charge-schedule timer block @ g_oneshot. */
typedef struct {
    uint8_t report_flag;        /*!< +0x00 slot4 one-shot report flag          */
    uint8_t report_divider;     /*!< +0x01 control 10-tick report divider      */
    uint8_t mode_apply_cnt;     /*!< +0x02 control mode-apply dwell/escalation count (os2) */
    uint8_t mode_hold_countdown; /*!< +0x03 control mode hold countdown (init 30) (os3) */
    uint8_t report_latch4;      /*!< +0x04 control one-shot latch (os4)        */
    uint8_t beep_counter;       /*!< +0x05 control beep counter (6-step)       */
    uint8_t report_latch6;      /*!< +0x06 control one-shot latch (os6)        */
    uint8_t chg_power_shadow;   /*!< +0x07 charge power/current shadow byte (os7) */
    uint16_t window_timer;      /*!< +0x08 scheduled-charging window sample timer (0x32 ticks) */
    uint16_t stored_mode_shadow; /*!< +0x0A control stored mode/err code shadow */
    uint16_t commto_tick;       /*!< +0x0C control comm-timeout tick (20-count) */
    uint16_t commto_dwell;      /*!< +0x0E control comm-timeout dwell (reload 600) */
} vcu_oneshot_t;


/**
 * \brief           Comm/bus-fault flag + bus-voltage block @ g_fault_codes
 * \note            0x08 bytes; +0x06 carries faultmon's raw-threshold code.
 *                      `vbus_code` and `active_fault_code` are modelled as byte-addressable
 *                      unions so can.c's RX byte pokes land on the named hi/lo byte
 *                      (vbus_code_hi @+0x03 from frame 0x401 d[5]; active_code_lo @+0x04
 *                      from frame 0x212 d[6]) instead of a raw g_fault_codes +0x03 / +0x04 reach.
 */
typedef struct {
    union {
        uint8_t comm_fault;      /*!< +0x00 bit0 = comm/bus fault (read by FSM run) */
        uint16_t comm_fault_word; /*!< +0x00 whole word: bit0 OTA-busy, bit1 charging, bit2 interlock, bit8 lock */
    };
    union {
        uint16_t vbus_code;      /*!< +0x02 battery-voltage code (derate code = hi byte) */
        struct {
            uint8_t vbus_code_lo; /*!< +0x02 low byte                               */
            uint8_t vbus_code_hi; /*!< +0x03 high byte (can rx 0x401 d[5])      */
        };
    };
    union {
        uint16_t active_fault_code; /*!< +0x04 can 0x212 code / faultmon FC->active_code */
        struct {
            uint8_t active_code_lo; /*!< +0x04 low byte (can rx 0x212 d[6])   */
            uint8_t active_code_hi; /*!< +0x05 high byte                          */
        };
    };
    int8_t raw_code;     /*!< +0x06 faultmon raw-threshold code (signed; FM_RAW_THRESHOLD cmp) */
    uint8_t _rsv07;      /*!< +0x07 pad to 0x08                              */
} vcu_fault_codes_t;


/**
 * \brief           Unified CAN tx-device + periodic-tick scheduler @ g_candev
 * \note            The device handle (reinit/lec/bus_off/tec/rec) and the scheduler state
 *                      (event gate, two countdown timers, out flags, payload getter at +0x1C)
 *                      share this one block so the ISR->tick->sender coupling aliases. The
 *                      scheduler tick and bus-off counter are their own global,
 *                      \ref vcu_can_sched_t.
 */
/** \brief candev timer_a / timer_b initial value and fresh-frame reload. */
#define VCU_CANDEV_TIMER_RELOAD 200

typedef struct {
    uint8_t  tx_payload[8];  /*!< +0x00 8-byte TX payload scratch filled by the payload getter */
    uint8_t  reinit_cnt;     /*!< +0x08 CAN re-init count                          */
    uint8_t  lec;            /*!< +0x09 last error code                            */
    uint8_t  _rsv0a;         /*!< +0x0A                                            */
    uint8_t  bus_off;        /*!< +0x0B bus-off latch                              */
    uint16_t event_flags;    /*!< +0x0C bit0 = event pending                       */
    int16_t  event_wait;     /*!< +0x0E event-gate countdown                       */
    uint8_t  _rsv10[2];      /*!< +0x10                                            */
    int16_t  timer_a;        /*!< +0x12 countdown -> out_bits bit1                 */
    int16_t  timer_b;        /*!< +0x14 countdown -> out_bits bit2                 */
    uint8_t  _rsv16[2];      /*!< +0x16 unused cell (no reader/writer)              */
    uint8_t  out_bits;       /*!< +0x18 event output flags                         */
    uint8_t  tec;            /*!< +0x19 transmit error counter                     */
    uint8_t  rec;            /*!< +0x1A receive error counter                      */
    uint8_t  _rsv1b;         /*!< +0x1B                                            */
    volatile uint8_t* (*payload_getter)(uint32_t can_id); /*!< +0x1C fills tx_payload for can_id, returns it (object ends at +0x1F) */
} vcu_candev_t;

/**
 * \brief           CAN periodic-tick scheduler state
 * \note            A separate variable from \ref vcu_candev_t. Keeping it apart lets the
 *                      device object start naturally aligned (payload_getter / the vtable
 *                      land on 4-byte boundaries).
 */
typedef struct {
    uint16_t tick;      /*!< free-running scheduler tick              */
    uint16_t fault_ctr; /*!< bus-off persistence counter              */
} vcu_can_sched_t;


/**
 * \brief           BMS comm-loss counter @ g_bms_commloss
 * \note            Ticks since the last BMS-group CAN frame: can.c zeroes the halfword
 *                      on RX, faultmon counts it to 400 (-> event 0x12). The Ninebot
 *                      sub-node 0x06/0x07 handler clears it too.
 */
typedef union {
    uint16_t cnt;   /*!< tick count                                   */
    uint8_t cnt_lo; /*!< low byte (no writer; the sub-node clear is a halfword store) */
} vcu_bms_commloss_t;


/* ========================================================================= */
/* Block globals.                                                            */
/*                                                                           */
/* One aligned global per blackboard block. The genuine overlaps are modelled */
/* on the SAME global (see each note).                                        */
/* ========================================================================= */

/* IN2 is a 512-byte union: the vcu_in2_t view, the Ninebot register file      */
/* (reg[]), the raw 0x200-byte identity record (config g_rec), the identity-   */
/* record tail cells past vcu_in2_t (+0x1B4..+0x1E9: ident, build id, the      */
/* report date-time and the CAN scratch) and the LEDM overlay (IN2+0x36) share */
/* these bytes. The tail cells are in the persisted record but past the        */
/* register window that vcu_in2_t names.                                       */
typedef union {
    vcu_in2_t f;           /*!< named register / state view (g_in2.f)                         */
    uint16_t reg[0x100];   /*!< Ninebot register file page 0: reg n = IN2 + 2n.
                                Key IDs the app/1dragon read: reg 0x17 = version id (IN2+0x2E);
                                reg 0xC0 = MCU id (IN2+0x180, identity_rec_t mcu_id); reg 0xDA = VCU id
                                (IN2+0x1B4, ident = identity_rec_t vcu_id, factory UID copy); reg 0xE4 = build id
                                (IN2+0x1C8, build_id). */
    uint8_t record[0x200]; /*!< 512-byte identity record (config g_rec)                       */
    struct {
        uint8_t _rsv_id[0x1B4]; /*!< +0x000..+0x1B3 (the f / record views)                     */
        union {
            uint8_t ident[16];      /*!< +0x1B4..+0x1C3 reg 0xDA..0xE1 identity record = VCU id:
                                         the first 12 B are the factory UID copy (@0x1FFFF7E8, op 0x18;
                                         op 0x41 reads), served as reg 0xDA (word-swapped on the wire). */
            uint32_t ident_word[4]; /*!< same bytes as words: [0..2] factory UID words, [2] = checksum salt (IN2+0x1BC) */
        };
        uint8_t _rsv1c4[4];         /*!< +0x1C4..+0x1C7                                          */
        uint8_t build_id[6];        /*!< +0x1C8..+0x1CD build id = BLE regs 0xE4..0xE6. Stamped every
                                         boot by config_identity_sync; op 0x19 VIN write also sets it (the
                                         next boot re-stamps it). The 1dragon backend reads reg 0xE4 for
                                         its compatibility gate, so a blank build id -> "firmware
                                         unsupported". This firmware stamps the git short hash as text
                                         (CONFIG_BUILD_ID, config.c). */
        uint16_t build_id_1ce;      /*!< +0x1CE cleared by config_identity_sync beside the build id (purpose unknown) */
        uint16_t rec_1d0;           /*!< +0x1D0 set to 1 by vcu_config_init (purpose unknown)     */
        uint16_t datetime_stored;   /*!< +0x1D2 report packed date-time (year/month/day); restored on reset */
        uint8_t _rsv1d4[0x0C];      /*!< +0x1D4..+0x1DF                                          */
        uint16_t throttle_pm;       /*!< +0x1E0 throttle, per mille of full (report_telemetry_build) */
        uint16_t brake_pm;          /*!< +0x1E2 brake, per mille of full (report_telemetry_build)    */
        uint16_t speed_pm;          /*!< +0x1E4 speed, per mille of the ride mode's column 6, <= 1000 (signed compare) */
        uint8_t _rsv1e6[2];         /*!< +0x1E6..+0x1E7                                          */
        uint16_t canrx_1e8_scratch; /*!< +0x1E8 can RX scratch (echoed to report frame[0x2E]) */
    };
} vcu_in2_block_t;

extern volatile vcu_vs_t          g_vs;
extern volatile vcu_in2_block_t   g_in2;
extern volatile vcu_in1_t g_in1;
extern volatile vcu_scr_t g_scr;
extern volatile vcu_stat_t g_stat;
extern volatile vcu_evt_t g_evt;
extern volatile vcu_mode_t g_mode;
extern volatile vcu_chg_t g_chg; /* chgstate; HL ride-mode overlay reuses it */
extern volatile vcu_chgbuf_t g_chgbuf;
extern volatile vcu_prev_t g_prev;
extern volatile vcu_tmr_t g_tmr;
extern volatile vcu_can_sched_t g_can_sched;
extern volatile vcu_candev_t g_candev; /* CAN device object + periodic-tick state */
extern volatile vcu_oneshot_t g_oneshot;
extern volatile vcu_fault_codes_t g_fault_codes;
extern volatile vcu_deb_t g_deb;
extern volatile vcu_disp_t g_disp;
extern volatile vcu_uartctx_t g_uartctx;
extern volatile vcu_canmsg_t g_canmsg;
extern volatile vcu_lowglob_t g_lowglob;
extern volatile vcu_state_block_t g_state_block;
extern volatile vcu_bms_commloss_t g_bms_commloss; /* BMS-group CAN comm-loss counter */

/**
 * \brief           Battery-2 (second pack) state that is not in \ref g_stat2
 * \note            can_rx_std zeroes `commloss_cnt` on every battery-2 frame
 *                      (0x430/0x431/0x432/0x435); vcu_can_battery2_presence_tick counts it to
 *                      200 and keeps the SoC-low verdict in `soc_low`. The pack is present
 *                      while \ref vcu_scr_t bat2_present is set; SF_DUAL_PACK follows it.
 */
typedef struct {
    uint16_t commloss_cnt; /*!< ticks since the last battery-2 frame (saturates at 200) */
    uint8_t soc_low;       /*!< 1 while the battery-2 SoC is <= 5 %                     */
} vcu_bat2_t;

/**
 * \brief           Node 0x02 register 0xD0 parameter push state (\ref vcu_node2_push_t state)
 * \note            The first CAN 0x211 frame arms it, the telemetry task sends the table
 *                      while it is pending, and the Ninebot reply handler (cmd 5 arg 0xD0) or
 *                      the 11th send ends it. vcu_config_init clears it. Value 3 is never
 *                      written.
 */
typedef enum {
    NODE2_PUSH_IDLE = 0,    /*!< not armed (boot, vcu_config_init)              */
    NODE2_PUSH_PENDING = 1, /*!< armed by CAN 0x211: the table is being sent     */
    NODE2_PUSH_DONE = 2     /*!< acknowledged (cmd 5 arg 0xD0) or sends used up  */
} vcu_node2_push_state_t;

/**
 * \brief           Node 0x02 register 0xD0 parameter push
 */
typedef struct {
    uint8_t state;    /*!< \ref vcu_node2_push_state_t                              */
    uint8_t send_div; /*!< telemetry-task calls counted towards the next send (sends at 10) */
    uint8_t tries;    /*!< sends so far; the send that finds 10 ends the push       */
} vcu_node2_push_t;

extern volatile vcu_stat_t         g_stat2;       /* battery-2 mirror of g_stat (CAN 0x430/431/432/435) */
extern volatile vcu_bat2_t         g_bat2;        /* battery-2 comm-loss count + SoC-low flag */
extern volatile vcu_node2_push_t   g_node2_push;  /* node 0x02 reg 0xD0 parameter push */

/* g_bp_rptr is typed as bp_report_ptrs_t beside its typedef in busproto.h; g_charge_timer /
   g_passcode_ch are typed globals declared beside their struct in charging.h / passcode.h;
   the ride sub-mode is a ridemode-local ridemode_sub_t object and its two mileage caches are
   separate report.c / odometer.c scalars. */
extern volatile uint32_t g_mcu_uid[3];        /* factory UID snapshot (report.c copies it in; ninebot serial-provision reads it) */

/**
 * \brief           Set the rear-light mode field (MODE.enable_flags bits 9..11)
 * \param[in]       v: \ref vcu_mode_light_t value
 * \note            Clears the field and adds the new value (`+` equals `|` after the
 *                      clear).
 */
static inline void mode_light_set(uint16_t v) {
    g_mode.enable_flags = (uint16_t)((g_mode.enable_flags & ~MODE_LIGHT_MASK) | ((uint32_t)v << MODE_LIGHT_SHIFT));
}

/* CANDEV note: g_candev is the CAN device object itself (no bias); g_can_sched holds */
/* the tick.                                                                         */

#endif /* VCU_STATE_H */
