/**
 * \file            test_vehicle.c
 * \brief           Vehicle FSM (vehicle.c): state-change latch, per-state handlers, input-test path
 * \note            Each test runs the real slot-0 task (vehicle_task_fsm) with every
 *                      cross-module callee, so a state is driven through its inputs (blackboard
 *                      cells, pins) and judged by its outputs: the next state, the blackboard,
 *                      the power latch PC14, the drive-kill latch PB15, the indicator /
 *                      anti-theft latches PB13 / PA6 / PB14, the dashboard screen and the
 *                      Ninebot event frames on CAN.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <string.h>
#include <unity.h>
#include "buzzer.h"
#include "can.h"
#include "control.h"
#include "gpio.h"
#include "sensor.h"
#include "tst_bsp_fakes.h"
#include "tst_faultmon.h"
#include "tst_ninebot.h"
#include "tst_support.h"
#include "usart.h"
#include "vcu_state.h"
#include "vehicle.h"

#define FSM_STATE           (g_state_block.fsm_state)

/* Pins the FSM latches drive. */
#define PIN_PWR_LATCH       GPIO_PINS_14 /* PC14 power / fault latch */
#define PIN_KILL            GPIO_PINS_15 /* PB15 drive-kill latch    */
#define PIN_BRAKE           GPIO_PINS_13 /* PB13 brake latch         */
#define PIN_THEFT           GPIO_PINS_14 /* PB14 anti-theft latch    */
#define PIN_IND             GPIO_PINS_6  /* PA6 indicator            */

/* Timing. */
#define POWER_ON_STEPS      11u  /* state 1: steps 0..10 (cnt < 0x0B)     */
#define IDLE_WAKE_TICKS     300u /* state 0: pending-event window          */
#define POWER_DOWN_TICKS    300u /* state 4: dwell before the latch release */
#define POWER_DOWN_COMMIT   20u  /* state 4: screen-0 re-commit period     */
#define CHG_KEYHOLD_RUNS    300u /* state 8: brake hold before the key-hold bit */
#define ALARM_GUARD         500u /* state 6: motion-guard window           */
#define POSTRIDE_PRESET     100u /* state 0 -> 8 settle preset            */
#define BUTTON_LATCH_READS  12u  /* PA10 12-bit debounce: 12 low reads latch 4 */

/* IMU tilt: lying on its side is a tilt above 50.0f degrees. The board is laid at a roll; the
   estimator's tilt (vcu_sensor_tilt_deg) follows it. */
#define TILT_FALLEN_MDEG    50000
#define ROLL_MARGIN_MDEG    2000  /* clear of the estimator's float rounding */
#define TILT_ALARM_LOW_MDEG 30000 /* a moderate tilt, well below 50 deg                  */
#define PRV_LIE_PITCH_MDEG  1000  /* keeps ay non-zero so the Mahony update runs            */
#define PRV_SETTLE_PASSES   400u  /* filter turn-over (~150) + a full 100-sample mean, with margin */

/* Ninebot event report on CAN (report_ninebot(0x16, 0x3D, 2, 0x56, code)). */
#define EVT_CAN_ID          0x18EF3D16u /* prio 6, PF 0xEF, dst 0x3D, src 0x16 */
#define EVT_CAN_CTL         0x01u
#define EVT_CMD             0x56u

/* Screen commit on USART (nb_commit_mode -> vcu_usart_report_send(.., 0x16, 0x23, 2, 3, 0x25, SCR+0x4A)). */
#define NODE_APP            0x23u
#define CMD_WRITE3          0x03u
#define ARG_MODE            0x25u

/* Screens committed through nb_commit_mode. */
#define SCREEN_OFF          0
#define SCREEN_RIDE         1
#define SCREEN_WAKE         2
#define SCREEN_CHARGE       3
#define SCREEN_NUMERIC      4
#define SCREEN_ALARM        5
#define SCREEN_IDLE         16

/* Battery state for every test: below the charge limit, so the charge-complete check
   (charge_complete_tick) does not request a charge-screen refresh. */
#define BATTERY_SOC         50
#define BATTERY_LIMIT       100u

/* BMS status bits the charger detect (charger_detect_tick) reads: STAT+0x124 bit6 and
   STAT+0x126 bit3 both set = charger connected. */
#define BMS_A_CHARGER       0x0040u
#define BMS_B_CHARGER       0x0008u

/** \brief One input-test event: the monitor record and the event code reported for it. */
typedef struct {
    uint8_t record; /*!< fm_mon_id_t      */
    uint8_t code;   /*!< reported arg byte */
} prv_event_t;

/**
 * \brief           Fresh boot state, released inputs, fakes recording, USART/CAN rings ready
 * \note            CAN TX is capture-only: a frame handed to a real mailbox would hold the
 *                      next one in the ring while it is on the bus (MSTS cuss gate), and
 *                      \ref prv_ticks pumps only once per task run.
 */
void setUp(void) {
    tst_reset_product_state();
    tst_fakes_reset();
    tst_can_tx_forward(0u);
    tst_fm_release_all();
    vcu_usart_init();
    vcu_can_init();
    g_stat.soc = BATTERY_SOC;
    g_stat.chg_limit = BATTERY_LIMIT;
}

/** \brief Nothing to undo */
void tearDown(void) {}

/**
 * \brief           Lay the board at `roll_mdeg` and let the IMU estimator settle on it
 * \note            x3emu IMU op (tst_emu_imu_set) + the product estimator (sensor.c): there
 *                      is no seed snap, the Mahony filter turns over from the
 *                      identity quaternion in about 100-150 passes and the tilt is the mean of the
 *                      last 100 samples, so the board is held for PRV_SETTLE_PASSES updates. A
 *                      small pitch keeps every accel axis non-zero (the filter skips an update
 *                      while ax * ay * az is 0). `prv_lie(0)` puts the emulated board back level.
 * \param[in]       roll_mdeg: roll, millidegrees
 */
static void prv_lie(int32_t roll_mdeg) {
    uint32_t pass;

    if (roll_mdeg == 0) {
        tst_emu_imu_set(0, 0, 0, 0);
    } else {
        tst_emu_imu_set(roll_mdeg, PRV_LIE_PITCH_MDEG, 0, 0);
        vcu_sensor_bus_init();
        vcu_sensor_init();
        for (pass = 0u; pass < PRV_SETTLE_PASSES; ++pass) {
            (void)vcu_sensor_attitude_update();
        }
    }
}

/**
 * \brief           Put the FSM in `state` without a state change pending
 * \param[in]       state: vcu_fsm_state_t
 */
static void prv_enter(uint8_t state) {
    FSM_STATE = state;
    g_prev.prev_state = state;
}

/**
 * \brief           Run the slot-0 task `count` times, pumping the CAN TX ring after each
 * \param[in]       count: number of task runs
 */
static void prv_ticks(uint32_t count) {
    uint32_t idx;

    for (idx = 0u; idx < count; ++idx) {
        vehicle_task_fsm();
        vcu_can_poll();
        (void)tst_usart_drain();
    }
}

/**
 * \brief           Plug the charger in or out as the BMS reports it
 * \param[in]       present: 1 = connected
 */
static void prv_charger(uint8_t present) {
    if (present != 0u) {
        g_stat.status_word_a = (uint16_t)(g_stat.status_word_a | BMS_A_CHARGER);
        g_stat.status_word_b = (uint16_t)(g_stat.status_word_b | BMS_B_CHARGER);
    } else {
        g_stat.status_word_a = (uint16_t)(g_stat.status_word_a & ~BMS_A_CHARGER);
        g_stat.status_word_b = (uint16_t)(g_stat.status_word_b & ~BMS_B_CHARGER);
    }
}

/**
 * \brief           Was `screen` committed (nb_commit_mode frame dst 0x23, cmd 3, arg 0x25)?
 * \param[in]       screen: screen id
 * \return          1 if a captured USART frame carries it
 */
static uint8_t prv_screen_sent(uint16_t screen) {
    const tst_usart_frame_t* raw;
    tst_nb_frame_t frame;
    uint32_t idx;
    uint8_t found = 0u;

    for (idx = 0u; idx < tst_usart_tx_count() && found == 0u; ++idx) {
        raw = tst_usart_tx_get(idx);
        if (raw != NULL && tst_nb_parse(raw->bytes, raw->len, &frame) && frame.dst == NODE_APP
            && frame.cmd == CMD_WRITE3 && frame.arg == ARG_MODE && frame.len == 2u
            && (uint16_t)(frame.body[0] | ((uint16_t)frame.body[1] << 8)) == screen) {
            found = 1u;
        }
    }
    return found;
}

/**
 * \brief           Level of an output latch
 * \param[in]       port: GPIO port
 * \param[in]       pin: pin mask
 * \return          1 if the output data register drives the pin high
 */
static uint8_t prv_out(const gpio_type* port, uint16_t pin) { return ((port->odt & pin) != 0u) ? 1u : 0u; }

/**
 * \brief           Find the captured Ninebot event report with event code `code`
 * \param[in]       code: event code (frame arg byte)
 * \return          the frame, or NULL
 */
static const tst_can_frame_t* prv_event_frame(uint8_t code) {
    const tst_can_frame_t* found = NULL;
    const tst_can_frame_t* frame;
    uint32_t idx;

    for (idx = 0u; idx < tst_can_tx_count() && found == NULL; ++idx) {
        frame = tst_can_tx_get(idx);
        if (frame != NULL && frame->ext != 0u && frame->id == EVT_CAN_ID && frame->data[1] == EVT_CMD
            && frame->data[2] == code) {
            found = frame;
        }
    }
    return found;
}

/**
 * \brief           Number of captured Ninebot event reports
 * \return          count of EVT_CAN_ID frames with cmd 0x56
 */
static uint32_t prv_event_count(void) {
    const tst_can_frame_t* frame;
    uint32_t idx;
    uint32_t count = 0u;

    for (idx = 0u; idx < tst_can_tx_count(); ++idx) {
        frame = tst_can_tx_get(idx);
        if (frame != NULL && frame->id == EVT_CAN_ID && frame->data[1] == EVT_CMD) {
            ++count;
        }
    }
    return count;
}

/* ------------------------------------------------------------------------- */
/* State-change latch and task frame                                         */
/* ------------------------------------------------------------------------- */

/**
 * \brief           A state change clears the session cells of the vehicle state
 * \note            prev_state = state, passcode_entry_reset, vehicle_event_block_clear, mode
 *                      counter 0, charger edge latch 0, cfg_17 &= 0xF3 (drive-fault state 0),
 *                      MODE+0x0A &= ~4, ~8, VS +0x1C/+0x3C/+0x1E/+0x44/+0x1B/+0x1A/+0x20/+0x1D/
 *                      +0x46/+0x04/+0x21/+0x25/+0x64/+0x65 = 0, +0x2A = 1, +0x30 = 600; +0x03 = 0
 *                      unless the new state is 1.
 */
static void test_state_change_latch(void) {
    prv_enter(VCU_FSM_ALARM);
    FSM_STATE = VCU_FSM_CHARGE_STANDBY;
    g_vs.postride_timer = 5; /* state 8 only counts it down this tick */
    g_vs.start_req = VS_START_LOCKED;
    g_vs.code_retries = 2u;
    g_vs.fault_ack = 1u;
    g_vs.input_test_ea = 1u;
    g_vs.lock_timer = 1u;
    g_vs.creep_enable = 0u;
    g_vs.power_released = 1u;
    g_vs.unlock_ok = 1u;
    g_vs.chg_full = 1u;
    g_vs.drive_fault_state = 3u;
    g_vs.auto_off_ticks = 1234u;
    g_prev.mode_counter = 77u;
    g_prev.chg_plug_edge = 1u;
    g_mode.cfg_17 = (uint8_t)(CFG17_DRIVE_FAULT | CFG17_LOW_SOC);
    g_mode.mode_flags = MF_ACTIVE | 0x0008u;
    g_evt.mon[FM_MON_GEAR_UP].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, g_prev.prev_state);
    TEST_ASSERT_EQUAL_UINT8(VS_START_NONE, g_vs.start_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.code_retries);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.fault_ack);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.input_test_ea);
    TEST_ASSERT_EQUAL_UINT16(600u, g_vs.lock_timer);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.creep_enable);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.power_released);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.unlock_ok);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.chg_full);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.drive_fault_state);
    TEST_ASSERT_EQUAL_UINT32(0u, g_vs.auto_off_ticks);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.mode_counter);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.chg_plug_edge);
    TEST_ASSERT_EQUAL_HEX8(CFG17_LOW_SOC, g_mode.cfg_17); /* cfg_17 &= 0xF3 */
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & (MF_ACTIVE | 0x0008u));
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_GEAR_UP].code);
    TEST_ASSERT_EQUAL_INT16(4, g_vs.postride_timer);
}

/**
 * \brief           A change into power-on keeps the start request
 * \note            A new state of 1 skips the VS+0x03 clear.
 */
static void test_state_change_into_power_on_keeps_start(void) {
    prv_enter(VCU_FSM_IDLE);
    FSM_STATE = VCU_FSM_POWER_ON;
    g_vs.start_req = VS_START_LOCKED;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VS_START_LOCKED, g_vs.start_req);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_ON, FSM_STATE);
}

/**
 * \brief           Event-block clear zeroes all twelve monitor records (0x90 bytes)
 * \note            The whole g_evt block is zeroed.
 */
static void test_event_block_clear(void) {
    memset((void*)&g_evt, 0xA5, sizeof(g_evt));
    vehicle_event_block_clear();
    TEST_ASSERT_EACH_EQUAL_HEX32(0u, (const uint32_t*)&g_evt, sizeof(g_evt) / sizeof(uint32_t));
}

/* ------------------------------------------------------------------------- */
/* State 0: idle (handler + dispatch case 0)                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Idle with a pending app start: wake screen 2, chirp, count the window
 * \note            (VS+0x28 || EVT+8 || charger plug-in edge) && counter < 300 -> VS+0x27 = 0;
 *                      counter 0 -> nb_commit_mode(2) and VS+0x28 -> VS+0x29 = 1; counter + 1. The
 *                      idle handler sets IN2+0x38 bit5.
 */
static void test_idle_pending_start_opens_window(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.app_start_req = 1u;
    g_vs.app_reg_pending = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_INT16(SCREEN_WAKE, g_scr.ui_screen);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chirp_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_reg_pending);
    TEST_ASSERT_EQUAL_UINT16(1u, g_prev.mode_counter);
    TEST_ASSERT_EQUAL_HEX16(SF_IDLE, g_in2.f.status_flags & SF_IDLE);
}

/**
 * \brief           Idle after the 300-tick window: an app start without the lock starts state 1
 * \note            Pending and counter >= 300 -> IN2+0x3A bit2
 *                      clear -> VS+0x03 = 3; EVT+8, VS+0x28, VS+0x08 = 0; VS+0x03 != 0 -> state 1.
 */
static void test_idle_start_without_lock(void) {
    prv_enter(VCU_FSM_IDLE);
    g_prev.mode_counter = IDLE_WAKE_TICKS;
    g_vs.app_start_req = 1u;
    g_vs.wake_src = VCU_GPIO_EVENT_PA10;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_ON, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(VS_START_NO_LOCK, g_vs.start_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_NONE, g_vs.wake_src);
}

/**
 * \brief           Idle start with the lock enabled: locked, or unlocked through the app bypass
 * \note            IN2+0x3A bit2 -> VS+0x40 == 0 -> (VS+0x43 ||
 *                      VS+0x2C) ? (VS+0x2C = 0, start 1) : start 2.
 */
static void test_idle_start_with_lock(void) {
    prv_enter(VCU_FSM_IDLE);
    g_in2.f.cfg_flags = CFG_LOCK_EN;
    g_prev.mode_counter = IDLE_WAKE_TICKS;
    g_vs.app_start_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_ON, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(VS_START_LOCKED, g_vs.start_req);

    setUp();
    prv_enter(VCU_FSM_IDLE);
    g_in2.f.cfg_flags = CFG_LOCK_EN;
    g_prev.mode_counter = IDLE_WAKE_TICKS;
    g_vs.app_start_req = 1u;
    g_vs.app_unlock_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VS_START_UNLOCKED, g_vs.start_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_unlock_req);
}

/**
 * \brief           Idle start with a remote lock request: stays idle on the idle screen
 * \note            VS+0x40 != 0 -> VS+0x40 = 0, start 0, no charger bit -> nb_commit_mode(0x10);
 *                      start == 0 keeps state 0.
 */
static void test_idle_remote_lock_refuses_start(void) {
    prv_enter(VCU_FSM_IDLE);
    g_in2.f.cfg_flags = CFG_LOCK_EN;
    g_prev.mode_counter = IDLE_WAKE_TICKS;
    g_vs.app_start_req = 1u;
    g_vs.remote_lock_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.remote_lock_req);
    TEST_ASSERT_EQUAL_INT16(SCREEN_IDLE, g_scr.ui_screen);
}

/**
 * \brief           Idle with the charger bit in a session goes to charge standby
 * \note            STAT+0x126 bit3 going 0 -> 8 latches the plug-in edge (chg_plug_edge = 1); the
 *                      edge and IN2+0x38 bit11 -> state 8, VS+0x03 = 0, post-ride timer = 100.
 */
static void test_idle_charger_in_session_to_charge_standby(void) {
    prv_enter(VCU_FSM_IDLE);
    g_scr.light_state = 2;
    g_stat.status_word_b = 0x0008u;
    g_in2.f.status_flags = SF_SESSION;
    g_prev.mode_counter = IDLE_WAKE_TICKS;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(VS_START_NONE, g_vs.start_req);
    TEST_ASSERT_EQUAL_INT16(POSTRIDE_PRESET, g_vs.postride_timer);
}

/**
 * \brief           Idle leaves the charger bit alone and reports light state 2
 * \note            SCR+0xA2 != 2 -> report_u16(0x51, 2) only; the charger bit STAT+0x126 bit3
 *                      is left alone, it feeds the task's charger edge.
 */
static void test_idle_keeps_charger_bit(void) {
    prv_enter(VCU_FSM_IDLE);
    g_stat.status_word_b = 0x0008u;
    g_prev.chg_bit_prev = 0x0008u; /* already seen: no edge */
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX16(0x0008u, g_stat.status_word_b & 0x0008u);
    TEST_ASSERT_EQUAL_UINT16(2u, g_scr.reg[0x51]);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
}

/**
 * \brief           Idle: only a charger plug-in edge opens the wake window, not the level
 * \note            (STAT+0x126 & 8) != chg_bit_prev -> chg_bit_prev = it, == 8 -> chg_plug_edge
 *                      = 1; the idle dispatch tests chg_plug_edge, not the bit.
 */
static void test_idle_charger_edge_opens_window(void) {
    prv_enter(VCU_FSM_IDLE);
    g_stat.status_word_b = 0x0008u;
    g_prev.chg_bit_prev = 0x0008u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.mode_counter);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.chg_plug_edge);
    g_stat.status_word_b = 0u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.chg_bit_prev);
    g_stat.status_word_b = 0x0008u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(1u, g_prev.chg_plug_edge);
    TEST_ASSERT_EQUAL_UINT16(1u, g_prev.mode_counter);
    TEST_ASSERT_EQUAL_INT16(SCREEN_WAKE, g_scr.ui_screen);
}

/**
 * \brief           Idle woken by the power button edge (PA10) runs the keep-alive like PA3
 * \note            Nothing pending and VS+0x08 == 4 or 2 -> keep-alive == 0 ? (state 4,
 *                      power-off beep) : keep-alive - 1.
 */
static void test_idle_button_wake_keepalive(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.wake_src = VCU_GPIO_EVENT_PA10;
    g_vs.keepalive_ticks = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.mode_counter);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.keepalive_ticks);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/**
 * \brief           Idle: a wake window that lapsed without a start powers down off the idle screen
 * \note            Nothing pending, no PA3/PA10 wake, mode counter != 0 and SCR+0x4A != 0x10 ->
 *                      state 4, VS+0x03 = 0, power-off beep. On the idle screen 0x10 the vehicle
 *                      stays.
 */
static void test_idle_lapsed_window_powers_down(void) {
    prv_enter(VCU_FSM_IDLE);
    g_prev.mode_counter = 5u;
    g_scr.ui_screen = SCREEN_IDLE;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    g_scr.ui_screen = SCREEN_WAKE;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(VS_START_NONE, g_vs.start_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/**
 * \brief           Idle woken by serial traffic (PA3) powers down once the keep-alive runs out
 * \note            Nothing pending and VS+0x08 == 4 -> VS+0x34
 *                      == 0 ? (state 4, VS+0x2B = 1) : VS+0x34 - 1.
 */
static void test_idle_serial_wake_keepalive(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.wake_src = VCU_GPIO_EVENT_PA3;
    g_vs.keepalive_ticks = 2u;
    prv_ticks(2u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(0u, g_vs.keepalive_ticks);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/**
 * \brief           An app register commit powers the idle vehicle down after 300 ticks
 * \note            VS+0x27 -> counter (g_prev +0x08) + 1, old value > 300 ->
 *                      counter 0, state 4, VS+0x2B = 1.
 */
static void test_idle_app_register_timeout(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.app_reg_pending = 1u;
    g_prev.ctr_08 = 300u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.ctr_08);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/**
 * \brief           Idle long press while lying on the side: power down, clear the start request
 * \note            Counter >= 300, tilt `> 50.0f` and EVT+8 != 0 -> state 4, VS+0x03 = 0,
 *                      power-off beep, EVT words 0..2 = 0.
 */
static void test_idle_fallen_power_down(void) {
    prv_lie(TILT_FALLEN_MDEG + ROLL_MARGIN_MDEG);
    TEST_ASSERT_TRUE(vcu_sensor_tilt_deg() > 50.0f);
    prv_enter(VCU_FSM_IDLE);
    g_prev.mode_counter = IDLE_WAKE_TICKS - 1u;
    g_vs.start_req = VS_START_NO_LOCK;
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    prv_ticks(1u);
    prv_lie(0);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(VS_START_NONE, g_vs.start_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
    TEST_ASSERT_EQUAL_HEX32(0u, g_evt.word[0] | g_evt.word[1] | g_evt.word[2]);
}

/**
 * \brief           The same long press at or below 50 degrees keeps the vehicle idle
 * \note            A tilt of at most 50.0f skips the power-down: the counter reaches 300,
 *                      VS+0x03 is kept.
 */
static void test_idle_long_press_upright_stays(void) {
    prv_lie(TILT_FALLEN_MDEG - ROLL_MARGIN_MDEG);
    TEST_ASSERT_TRUE(vcu_sensor_tilt_deg() <= 50.0f);
    prv_enter(VCU_FSM_IDLE);
    g_prev.mode_counter = IDLE_WAKE_TICKS - 1u;
    g_vs.start_req = VS_START_NO_LOCK;
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    prv_ticks(1u);
    prv_lie(0);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(IDLE_WAKE_TICKS, g_prev.mode_counter);
    TEST_ASSERT_EQUAL_UINT8(VS_START_NO_LOCK, g_vs.start_req);
}

/* ------------------------------------------------------------------------- */
/* State 1: power-on (handler + dispatch case 1)                             */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Power-on runs 11 steps, then goes to ride for a start without the lock
 * \note            g_prev +0x0A < 0x0B -> VS+0x1E = 0 (step 0: vcu_config_init, buzzer_beep(2)),
 *                      step + 1; else step 0 and, with IN1+0x27 set, VS+0x1E
 *                      stays 1. Dispatch: VS+0x1E, no shutdown, start 3 or 1 -> state 3.
 */
static void test_power_on_to_ride(void) {
    prv_enter(VCU_FSM_POWER_ON);
    g_vs.start_req = VS_START_NO_LOCK;
    g_in1.arm_gate = 1u;
    prv_ticks(POWER_ON_STEPS);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_ON, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(BEEP_SHORT, g_lowglob.beep_seq_id);
    g_in1.arm_gate = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_RIDE, FSM_STATE);
}

/**
 * \brief           Power-on with a locked start goes to the passcode state
 * \note            Dispatch case 1: VS+0x03 == 2 -> state 2.
 */
static void test_power_on_to_locked(void) {
    prv_enter(VCU_FSM_POWER_ON);
    g_vs.start_req = VS_START_LOCKED;
    prv_ticks(POWER_ON_STEPS);
    g_in1.arm_gate = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_LOCKED, FSM_STATE);
}

/**
 * \brief           Power-on with a shutdown request goes to power-down
 * \note            VS+0x1C -> VS+0x1E = 1; dispatch: VS+0x1C -> state 4.
 */
static void test_power_on_shutdown(void) {
    prv_enter(VCU_FSM_POWER_ON);
    g_vs.start_req = VS_START_NO_LOCK;
    prv_ticks(POWER_ON_STEPS);
    g_vs.shutdown_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
}

/**
 * \brief           Power-on waits while the arm gate is closed, with the run bit set
 * \note            Steps done, no shutdown, IN1+0x27 == 0 -> IN2+0x38 |= 1, power ready (VS+0x1E)
 *                      0, step counter = 0x14 (so the next run takes the same branch).
 */
static void test_power_on_waits_for_arm_gate(void) {
    prv_enter(VCU_FSM_POWER_ON);
    g_vs.start_req = VS_START_NO_LOCK;
    prv_ticks(POWER_ON_STEPS);
    g_in1.arm_gate = 0u;
    g_in2.f.status_flags = 0u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_ON, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(0x14u, g_prev.ctr_0a);
    TEST_ASSERT_EQUAL_HEX16(SF_RUN, g_in2.f.status_flags & SF_RUN);
}

/**
 * \brief           Power-on clears only the idle bit on entry, the run bit stays
 * \note            IN2+0x38 &= ~0x20 only.
 */
static void test_power_on_keeps_run_bit(void) {
    prv_enter(VCU_FSM_POWER_ON);
    g_vs.start_req = VS_START_NO_LOCK;
    g_prev.ctr_0a = 1u; /* past step 0: its vcu_config_init reloads IN2 from the identity */
    g_in2.f.status_flags = SF_RUN | SF_IDLE;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX16(SF_RUN, g_in2.f.status_flags & (SF_RUN | SF_IDLE));
}

/* ------------------------------------------------------------------------- */
/* State 2: locked (handler + dispatch case 2)                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Locked: the first run shows the passcode screen (numeric -> 4, key -> len|0x11)
 * \note            g_prev +0x01 == 0 -> 1, passcode_entry_reset, IN2+0x3A bit7 ? nb_commit_mode(4)
 *                      : nb_commit_mode(VS+0x45 << 8 | 0x11).
 */
static void test_locked_shows_passcode_screen(void) {
    prv_enter(VCU_FSM_LOCKED);
    g_in2.f.cfg_flags = CFG_LOCK_EN | CFG_PASSCODE_NUM;
    prv_ticks(1u);
    TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_NUMERIC));

    setUp();
    prv_enter(VCU_FSM_LOCKED);
    g_in2.f.cfg_flags = CFG_LOCK_EN | CFG_PASSCODE_KEY;
    prv_ticks(1u);
    /* passcode_entry_reset (passcode.c) recounts VS+0x45 from the stored key code before the commit. */
    TEST_ASSERT_TRUE(prv_screen_sent((uint16_t)(((uint16_t)g_vs.code_len << 8) | 0x11u)));
}

/**
 * \brief           Locked: an app unlock goes to ride
 * \note            VS+0x43 || VS+0x2C -> VS+0x2C = 0, VS+0x44 = 1;
 *                      dispatch: VS+0x44 -> state 3.
 */
static void test_locked_app_unlock_to_ride(void) {
    prv_enter(VCU_FSM_LOCKED);
    g_in2.f.cfg_flags = CFG_LOCK_EN | CFG_PASSCODE_KEY;
    g_vs.app_unlock_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_RIDE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_unlock_req);
}

/**
 * \brief           Locked: a shutdown goes to power-down, a lock-out alarm to state 6
 * \note            Dispatch case 2: VS+0x44 == 0 -> VS+0x1C ? 4 : (VS+0x42 == 2 ? 6).
 */
static void test_locked_shutdown_and_alarm(void) {
    prv_enter(VCU_FSM_LOCKED);
    g_vs.shutdown_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);

    setUp();
    prv_enter(VCU_FSM_LOCKED);
    g_vs.lockout_phase = LOCKOUT_ALARM;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_ALARM, FSM_STATE);
}

/**
 * \brief           Locked at a standstill arms the drive-fault state 2, and 3 above 150 on the bus
 * \note            Comm fault bit0 clear, speed < 3 and the latch (g_prev +0x00) 0 -> latch 1,
 *                      MODE+0x0A |= 4 | 8, cfg_17 = (x & 0xF3) + 8; latch 1 and A3C+2 hi byte `> 0x96`
 *                      -> cfg_17 |= 0x0C.
 */
static void test_locked_arms_drive_fault_state(void) {
    prv_enter(VCU_FSM_LOCKED);
    g_in2.f.cfg_flags = CFG_LOCK_EN; /* no passcode kind: the unlock routines do not run */
    g_mode.cfg_17 = CFG17_LOW_SOC;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(CFG17_LOW_SOC | (2u << CFG17_DRIVE_FAULT_SHIFT)), g_mode.cfg_17);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_LOCKED, FSM_STATE);
    g_fault_codes.vbus_code = 0x9600u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(CFG17_LOW_SOC | (2u << CFG17_DRIVE_FAULT_SHIFT)), g_mode.cfg_17);
    g_fault_codes.vbus_code = 0x9700u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(CFG17_LOW_SOC | CFG17_DRIVE_FAULT), g_mode.cfg_17);
}

/**
 * \brief           The alarm arms the same drive-fault state on its first standstill run
 * \note            As in the locked state, with the alarm latch g_prev +0x02.
 */
static void test_alarm_arms_drive_fault_state(void) {
    prv_enter(VCU_FSM_ALARM);
    g_mode.cfg_17 = 0u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(2u << CFG17_DRIVE_FAULT_SHIFT), g_mode.cfg_17);
    g_fault_codes.vbus_code = 0xFF00u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX8(CFG17_DRIVE_FAULT, g_mode.cfg_17);
}

/* ------------------------------------------------------------------------- */
/* State 3: ride / state 7: charging                                          */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Ride: marks the drive pipeline running; a shutdown request goes to power-down
 * \note            VS+0x42 = 0, IN2+0x38 |= 1. Dispatch case 3: VS+0x1C -> 4.
 */
static void test_ride_shutdown(void) {
    prv_enter(VCU_FSM_RIDE);
    g_vs.lockout_phase = LOCKOUT_ARMED;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_RIDE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_NONE, g_vs.lockout_phase);
    TEST_ASSERT_EQUAL_HEX16(SF_RUN, g_in2.f.status_flags & SF_RUN);
    g_vs.shutdown_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
}

/**
 * \brief           Ride: a charger in a session goes to charging, which leaves when it is unplugged
 * \note            Dispatch case 3: IN1+0x25 and IN2+0x38 bit11 -> state 7. Case 7: IN1+0x25 == 0
 *                      or VS+0x1C -> state 4. The charging handler commits screen 1 once through its
 *                      own one-shot (g_prev.charging_oneshot), not the charger-detect flag SCR+0x10.
 *                      The charger is presented as the BMS bits the charger detect
 *                      (charger_detect_tick) turns into IN1+0x25.
 */
static void test_ride_charger_to_charging(void) {
    prv_enter(VCU_FSM_RIDE);
    prv_charger(1u);
    g_in2.f.status_flags = SF_SESSION;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGING, FSM_STATE);
    prv_ticks(1u);
    TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_RIDE));
    TEST_ASSERT_EQUAL_UINT16(1u, g_prev.charging_oneshot);
    prv_charger(0u);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
}

/* ------------------------------------------------------------------------- */
/* State 4: power-down (handler + dispatch case 4)                           */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Power-down: screen 0, long beep, then after 300 runs PB15 high, PC14 low -> sleep
 * \note            First run (chk_d_latch 0) -> nb_commit_mode(0), no power-off beep -> long beep.
 *                      PA10 released -> counter (ctr_d) `< 300`: MODE+0x0A |= 0x10, + 1; else beep off,
 *                      PB15 set, power/fault latch released (PC14 push-pull output, low), power
 *                      released (VS+0x1B), VS+0x08 = 0. Dispatch: released, no charger -> state 5.
 */
static void test_power_down_releases_latch(void) {
    gpio_bits_set(GPIOC, PIN_PWR_LATCH);
    gpio_bits_reset(GPIOB, PIN_KILL);
    prv_enter(VCU_FSM_POWER_DOWN);
    g_vs.wake_src = VCU_GPIO_EVENT_PA3;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(SCREEN_OFF, g_scr.ui_screen);
    TEST_ASSERT_EQUAL_UINT8(BEEP_LONG, g_lowglob.beep_seq_id);
    prv_ticks(100u); /* part of the dwell (running on into state 5 would enter the WFI standby) */
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    prv_ticks(POWER_DOWN_TICKS - 101u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_HEX16(0x0010u, g_mode.mode_flags & 0x0010u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOC, PIN_PWR_LATCH));
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_SLEEP, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOC, PIN_PWR_LATCH));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOB, PIN_KILL));
    TEST_ASSERT_EQUAL_HEX32(0x1u, (GPIOC->cfghr >> 24) & 0x3u); /* PC14 output, push-pull */
    TEST_ASSERT_EQUAL_HEX32(0x0u, (GPIOC->cfghr >> 26) & 0x3u);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_NONE, g_vs.wake_src);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x0010u);
}

/**
 * \brief           Power-down after a power-off beep stops the player instead of the long beep
 * \note            Power-off beep (VS+0x2B) -> buzzer_beep(0), flag cleared; else the long beep.
 */
static void test_power_down_poweroff_beep_skips_long_beep(void) {
    prv_enter(VCU_FSM_POWER_DOWN);
    g_vs.poweroff_beep = 1u;
    g_lowglob.beep_seq_id = BEEP_SHORT;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.poweroff_beep);
    TEST_ASSERT_EQUAL_UINT8(BEEP_NONE, g_lowglob.beep_seq_id);
}

/**
 * \brief           Power-down re-commits screen 0 every 20 runs up to run 100, not on runs 1..6
 * \note            New counter `<= 100` and `% 20 == 0` -> nb_commit_mode(0).
 */
static void test_power_down_screen_commits(void) {
    uint32_t run;
    uint32_t commits = 0u;

    prv_enter(VCU_FSM_POWER_DOWN);
    prv_ticks(1u); /* entry commit + counter 1 */
    for (run = 2u; run <= 120u; ++run) {
        tst_fakes_reset();
        tst_can_tx_forward(0u);
        prv_ticks(1u);
        if (prv_screen_sent(SCREEN_OFF) != 0u) {
            TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, run % POWER_DOWN_COMMIT, "screen 0 committed off the 20-run grid");
            ++commits;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(5u, commits);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
}

/**
 * \brief           Power-down runs the power-button monitor every run
 * \note            vcu_faultmon_can_fault_escalate runs on every run. With PA10 held since the
 *                      entry the press is not a cancel (pd_button_held stays 1), so the 12-bit
 *                      debounce latches code 4.
 */
static void test_power_down_monitors_power_button(void) {
    prv_enter(VCU_FSM_POWER_DOWN);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_ticks(BUTTON_LATCH_READS);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_HIGH);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(4u, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT8(1u, g_prev.pd_button_held);
}

/**
 * \brief           A new power-button press during power-down cancels it and restarts the vehicle
 * \note            PA10 active, EVT+8 != 0 and pd_button_held == 0 (cleared while PA10 was
 *                      released) -> chk_d_latch 0, ctr_d2 0, pd_button_held 1, EVT+8 = 0, MODE+0x0A &=
 *                      ~0x10, chirp and app start requests, beep off, PB15 clear, vcu_can_mode_reset,
 *                      power/fault latch asserted (PC14 high), state 0.
 */
static void test_power_down_button_press_cancels(void) {
    gpio_bits_set(GPIOB, PIN_KILL);
    gpio_bits_reset(GPIOC, PIN_PWR_LATCH);
    prv_enter(VCU_FSM_POWER_DOWN);
    prv_ticks(5u); /* released: the guard drops */
    TEST_ASSERT_EQUAL_UINT8(0u, g_prev.pd_button_held);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_ticks(BUTTON_LATCH_READS - 1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    prv_ticks(1u);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_HIGH);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chirp_req);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT8(0u, g_prev.chk_d_latch);
    TEST_ASSERT_EQUAL_UINT8(1u, g_prev.pd_button_held);
    TEST_ASSERT_EQUAL_HEX16(0u, g_mode.mode_flags & 0x0010u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOC, PIN_PWR_LATCH));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, PIN_KILL));
}

/**
 * \brief           Power-down holds while the power button (PA10) is pressed
 * \note            PA10 low -> counter (ctr_d) 0; the countdown runs only while PA10 is released.
 */
static void test_power_down_holds_while_button_pressed(void) {
    prv_enter(VCU_FSM_POWER_DOWN);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_ticks(POWER_DOWN_TICKS + 10u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.ctr_d);
}

/**
 * \brief           Power-down with a charger in a session wakes into charge standby
 * \note            Case 4: VS+0x1B and IN1+0x25 with IN2+0x38 bit11 -> PB15 clear,
 *                      vcu_can_mode_reset, power/fault latch asserted (PC14 set), state 8, VS+0x29 = 1.
 */
static void test_power_down_with_charger_to_charge_standby(void) {
    prv_enter(VCU_FSM_POWER_DOWN);
    g_in2.f.status_flags = SF_SESSION;
    prv_charger(1u);
    prv_ticks(101u); /* part of the dwell */
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    prv_ticks(POWER_DOWN_TICKS - 101u);
    g_in2.f.status_flags = SF_SESSION;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOC, PIN_PWR_LATCH));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, PIN_KILL));
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.chirp_req);
}

/**
 * \brief           Power-down keeps the vehicle awake (state 8) for a charge keep-awake request
 * \note            Case 4: VS+0x18 != 0 -> the state-8 branch.
 */
static void test_power_down_keep_awake_to_charge_standby(void) {
    prv_enter(VCU_FSM_POWER_DOWN);
    prv_ticks(101u); /* part of the dwell */
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    prv_ticks(POWER_DOWN_TICKS - 101u);
    g_vs.chg_keep_awake = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, FSM_STATE);
}

/* ------------------------------------------------------------------------- */
/* State 5: sleep (handler + dispatch case 5)                                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Sleep woken by PC13 (motion): latches back on, goes to the alarm state
 * \note            VS+0x08 != 0 and not (2 with PA10 released) -> PB15 clear, vcu_can_mode_reset,
 *                      power/fault latch asserted, VS+0x1E = 1;
 *                      VS+0x26 = 0. Dispatch case 5: VS+0x08 == 1 -> state 6, VS+0x06 = 0,
 *                      VS+0x08 = 0.
 */
static void test_sleep_motion_wake_to_alarm(void) {
    gpio_bits_set(GPIOB, PIN_KILL);
    gpio_bits_reset(GPIOC, PIN_PWR_LATCH);
    prv_enter(VCU_FSM_SLEEP);
    g_vs.wake_src = VCU_GPIO_EVENT_PC13;
    g_vs.tilt_latch = 2u;
    g_vs.report_enable = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_ALARM, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.tilt_latch);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_NONE, g_vs.wake_src);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.report_enable);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOC, PIN_PWR_LATCH));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, PIN_KILL));
}

/**
 * \brief           Sleep woken by the power button (PA10 held) or serial traffic goes to idle
 * \note            Dispatch case 5: VS+0x1E and VS+0x08 not 0 and not 1 -> state 0.
 */
static void test_sleep_button_or_serial_wake_to_idle(void) {
    prv_enter(VCU_FSM_SLEEP);
    g_vs.wake_src = VCU_GPIO_EVENT_PA10;
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOC, PIN_PWR_LATCH));

    setUp();
    prv_enter(VCU_FSM_SLEEP);
    g_vs.wake_src = VCU_GPIO_EVENT_PA3;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
}

/**
 * \brief           Sleep: a PA10 edge with the button already released is dropped
 * \note            VS+0x08 == 2 and vcu_gpio_pa10_active() == 0 -> VS+0x08 = 0;
 *                      VS+0x1E stays 0, so the state stays 5 (the next run re-enters standby).
 */
static void test_sleep_released_button_edge_dropped(void) {
    prv_enter(VCU_FSM_SLEEP);
    g_vs.wake_src = VCU_GPIO_EVENT_PA10;
    g_vs.power_ready = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_SLEEP, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(VCU_GPIO_EVENT_NONE, g_vs.wake_src);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.power_ready);
}

/* ------------------------------------------------------------------------- */
/* State 6: alarm (handler + dispatch case 6)                                */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Alarm: a button press returns to idle as an app start, lock-out armed
 * \note            Dispatch case 6: EVT+8 == 1 || VS+0x28 -> state 0, EVT+8 = 0,
 *                      VS+0x28 = 1, VS+0x42 = 1.
 */
static void test_alarm_button_to_idle(void) {
    prv_enter(VCU_FSM_ALARM);
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_IDLE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_evt.mon[FM_MON_POWER_BTN].code);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.app_start_req);
    TEST_ASSERT_EQUAL_UINT8(LOCKOUT_ARMED, g_vs.lockout_phase);
}

/**
 * \brief           Alarm: a tripped tilt latch or a remote unlock powers down
 * \note            Dispatch case 6: VS+0x06 == 1 || VS+0x1C || VS+0x43 -> state 4,
 *                      VS+0x06 = 0, VS+0x43 -> VS+0x2B = 1.
 */
static void test_alarm_tilt_or_remote_unlock_to_power_down(void) {
    prv_enter(VCU_FSM_ALARM);
    g_vs.tilt_latch = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.tilt_latch);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.poweroff_beep);

    setUp();
    prv_enter(VCU_FSM_ALARM);
    g_vs.remote_unlocked = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/**
 * \brief           Alarm guard window: 500 runs, then with the alarm disabled it shuts down
 * \note            ctr_commfault_b < 500 -> vcu_faultmon_can_fault_escalate, + 1; else IN2+0x3A
 *                      bit15 clear -> VS+0x1C = 1, VS+0x2B = 1. Dispatch: VS+0x1C -> state 4.
 */
static void test_alarm_guard_then_shutdown(void) {
    prv_enter(VCU_FSM_ALARM);
    g_prev.ctr_commfault_b = ALARM_GUARD - 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_ALARM, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(ALARM_GUARD, g_prev.ctr_commfault_b);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/**
 * \brief           Alarm enabled after the guard window: siren screen 5, stays armed
 * \note            IN2+0x3A bit15, VS+0x2A (creep) set -> vcu_faultmon_can_fault_escalate,
 *                      SCR+0x4A != 5 -> nb_commit_mode(5), control_alarm_motion_tick(2).
 */
static void test_alarm_enabled_shows_siren(void) {
    prv_enter(VCU_FSM_ALARM);
    g_in2.f.cfg_flags = CFG_ALARM_EN;
    g_prev.ctr_commfault_b = ALARM_GUARD;
    g_vs.creep_enable = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_ALARM, FSM_STATE);
    TEST_ASSERT_EQUAL_INT16(SCREEN_ALARM, g_scr.ui_screen);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.shutdown_req);
}

/**
 * \brief           Alarm timeout: a moderate tilt keeps the siren, only above 50 degrees it ends
 * \note            IN2+0x3A bit15, IN2+0x3E bit9 and tilt `> 50.0f` -> creep (VS+0x2A) 0 ->
 *                      shutdown + power-off beep; else the siren screen 5.
 */
static void test_alarm_timeout_tilt(void) {
    prv_lie(TILT_ALARM_LOW_MDEG);
    prv_enter(VCU_FSM_ALARM);
    g_in2.f.cfg_flags = CFG_ALARM_EN;
    g_in2.f.chg_cfg = ALARM_TIMEOUT_EN;
    g_prev.ctr_commfault_b = ALARM_GUARD;
    g_vs.creep_enable = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.creep_enable);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_ALARM, FSM_STATE);
    TEST_ASSERT_EQUAL_INT16(SCREEN_ALARM, g_scr.ui_screen);
    prv_lie(0);

    setUp(); /* a fresh estimator: its 100-sample mean starts empty */
    prv_lie(TILT_FALLEN_MDEG + ROLL_MARGIN_MDEG);
    TEST_ASSERT_TRUE(vcu_sensor_tilt_deg() > 50.0f);
    prv_enter(VCU_FSM_ALARM);
    g_in2.f.cfg_flags = CFG_ALARM_EN;
    g_in2.f.chg_cfg = ALARM_TIMEOUT_EN;
    g_prev.ctr_commfault_b = ALARM_GUARD;
    g_vs.creep_enable = 1u;
    prv_ticks(1u);
    prv_lie(0);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(1u, g_vs.poweroff_beep);
}

/* ------------------------------------------------------------------------- */
/* State 8: charge standby (handler + dispatch case 8)                       */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Charge standby counts the settle preset down before anything else
 * \note            VS+0x38 != 0 -> VS+0x38 - 1 (and the dispatch waits
 *                      for VS+0x38 == 0).
 */
static void test_charge_standby_settle(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    g_vs.postride_timer = 2;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, FSM_STATE);
    TEST_ASSERT_EQUAL_INT16(1, g_vs.postride_timer);
}

/**
 * \brief           Charge standby without the charger bit powers down
 * \note            Dispatch case 8: VS+0x38 == 0 and neither STAT+0x126 bit3 nor
 *                      VS+0x1F -> state 4.
 */
static void test_charge_standby_charger_gone(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_POWER_DOWN, FSM_STATE);
}

/**
 * \brief           Charge standby: the first run shows the charge screen 3
 * \note            g_prev +0x05 == 0 -> +0x05 = 1, +0x06 = 1, report_status, no schedule ->
 *                      nb_commit_mode(3).
 */
static void test_charge_standby_charge_screen(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    g_stat.status_word_b = 0x0008u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, FSM_STATE);
    TEST_ASSERT_EQUAL_INT16(SCREEN_CHARGE, g_scr.ui_screen);
    TEST_ASSERT_EQUAL_UINT8(1u, g_prev.chk_e_latch);
}

/**
 * \brief           Charge standby with charge scheduling: the schedule screen shows the start in minutes
 * \note            CHG_SCHED_EN, charger unplugged -> nb_commit_mode(0x12) and SCR ridetime_disp =
 *                      hour (reg 0x4B hi) x 60 + minute (lo), on the first run and on a display refresh.
 */
static void test_charge_standby_schedule_screen(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    g_stat.status_word_b = 0x0008u;
    g_in2.f.chg_cfg = CHG_SCHED_EN;
    g_in2.f.chg_sched_start = 0x172Du; /* 23:45 */
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(0x12, g_scr.ui_screen);
    TEST_ASSERT_EQUAL_INT16(23 * 60 + 45, g_scr.ridetime_disp);

    g_in2.f.chg_sched_start = 0xFF3Bu; /* hour byte 255, minute 59 */
    g_prev.chk_e_phase = 0u;
    g_vs.chg_disp_refresh = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_INT16(255 * 60 + 59, g_scr.ridetime_disp);
}

/**
 * \brief           Charge standby: a button press starts riding (lock off, charger unplugged) or charging
 * \note            Dispatch case 8: charger bit and (EVT+8 == 1 || VS+0x28) -> EVT+8 = 0;
 *                      IN2+0x3A bit2 -> 2; SCR+0x4A == 0x12 || IN1+0x25 == 0 -> 3; else 7 and
 *                      VS+0x28 = 0.
 */
static void test_charge_standby_start(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    g_stat.status_word_b = 0x0008u;
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_RIDE, FSM_STATE);

    setUp();
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    prv_charger(1u);
    g_vs.app_start_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGING, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT8(0u, g_vs.app_start_req);

    setUp();
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    g_stat.status_word_b = 0x0008u;
    g_in2.f.cfg_flags = CFG_LOCK_EN;
    g_vs.app_start_req = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_LOCKED, FSM_STATE);
}

/**
 * \brief           Charge standby: a real press of the power button (PA10) starts the vehicle
 * \note            The handler runs vehicle_fault_aggregate, which includes the PA10 monitor: 12
 *                      low reads latch 4, the release demotes it to the press code 1, and dispatch
 *                      case 8 then leaves for state 3 (no lock, unplugged). Case 8 with EVT+8 == 1 ->
 *                      EVT+8 = 0 and report_u16(0x61, 0x20) (the lock-event report).
 */
static void test_charge_standby_power_button(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    g_stat.status_word_b = 0x0008u;
    prv_ticks(1u);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_LOW);
    prv_ticks(BUTTON_LATCH_READS);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_CHARGE_STANDBY, FSM_STATE);
    tst_fm_pin(GPIOA, GPIO_PINS_10, TST_LEVEL_HIGH);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(VCU_FSM_RIDE, FSM_STATE);
    TEST_ASSERT_EQUAL_UINT16(0x20u, g_scr.reg[0x61]);
}

/**
 * \brief           Charge standby: the charge screen is held 1000 runs, then screen 0
 * \note            Phase (g_prev +0x06) != 0 -> counter (+0x16) + 1, old value > 999 -> counter 0,
 *                      phase 0, nb_commit_mode(0), VS+0x21 = 0.
 */
static void test_charge_standby_screen_hold(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    prv_charger(1u);
    g_prev.chk_e_latch = 1u;
    g_prev.chk_e_phase = 1u;
    g_prev.ctr_e = 999u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(1000u, g_prev.ctr_e);
    TEST_ASSERT_FALSE(prv_screen_sent(SCREEN_OFF));
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT16(0u, g_prev.ctr_e);
    TEST_ASSERT_EQUAL_UINT8(0u, g_prev.chk_e_phase);
    TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_OFF));
}

/**
 * \brief           Charge standby on the schedule screen: a 300-run brake hold sets the key-hold bit
 * \note            SCR+0x4A == 0x12 -> vcu_faultmon_sensor_debounce_a / _b; EVT+0x80 or +0x8C ==
 *                      4 -> counter (ctr_e2) + 1, `300 <` it -> counter 0 and IN2+0x3E |= 0x80. The
 *                      lever latches 4 on its 5th held reading, so the bit is set on run 305.
 */
static void test_charge_standby_key_hold(void) {
    prv_enter(VCU_FSM_CHARGE_STANDBY);
    prv_charger(1u);
    g_prev.chk_e_latch = 1u;
    g_prev.chk_e_phase = 1u;
    g_scr.ui_screen = 0x12;
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, 0x0FFFu);
    prv_ticks(CHG_KEYHOLD_RUNS + 4u);
    TEST_ASSERT_EQUAL_HEX16(0u, g_in2.f.chg_cfg & CHG_KEYHOLD);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_HEX16(CHG_KEYHOLD, g_in2.f.chg_cfg & CHG_KEYHOLD);
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, 0u);
}

/**
 * \brief           Ride in a session: screen 1 after 200 runs on the session screen 0x14
 * \note            IN2+0x38 bit11 and g_prev +0x03 == 0 -> SCR+0x4A == 0x14 ? counter (+0x0E) + 1
 *                      : counter = 200; counter > 199 -> g_prev +0x03 = 1, counter 0,
 *                      nb_commit_mode(1), report_status, report_u16(0x51, 1).
 */
static void test_ride_session_screen(void) {
    prv_enter(VCU_FSM_RIDE);
    g_in2.f.status_flags = SF_SESSION;
    g_scr.ui_screen = 0x14;
    g_prev.ctr_c = 198u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_prev.chk_c_latch);
    TEST_ASSERT_EQUAL_UINT16(199u, g_prev.ctr_c);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_prev.chk_c_latch);
    TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_RIDE));
}

/**
 * \brief           Ride in a session off the session screen commits screen 1 at once
 * \note            SCR+0x4A != 0x14 -> counter = 200 -> > 199.
 */
static void test_ride_session_other_screen(void) {
    prv_enter(VCU_FSM_RIDE);
    g_in2.f.status_flags = SF_SESSION;
    g_scr.ui_screen = SCREEN_WAKE;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_prev.chk_c_latch);
    TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_RIDE));
}

/**
 * \brief           Power-on step 0 in a session shows the ride or passcode screen
 * \note            Step 0: IN2+0x38 bit11 -> start 3/1 -> nb_commit_mode(1); start 2 -> IN2+0x3A
 *                      bit7 ? nb_commit_mode(4) : (VS+0x45 << 8 | 0x11).
 */
static void test_power_on_first_screen(void) {
    prv_enter(VCU_FSM_POWER_ON);
    g_in2.f.status_flags = SF_SESSION;
    g_vs.start_req = VS_START_UNLOCKED;
    prv_ticks(1u);
    TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_RIDE));

    setUp();
    prv_enter(VCU_FSM_POWER_ON);
    g_in2.f.status_flags = SF_SESSION;
    g_vs.start_req = VS_START_LOCKED;
    prv_ticks(1u);
    /* vcu_config_init (config.c) reloads IN2+0x3A from the identity before the commit. */
    if ((g_in2.f.cfg_flags & CFG_PASSCODE_NUM) != 0u) {
        TEST_ASSERT_TRUE(prv_screen_sent(SCREEN_NUMERIC));
    } else {
        TEST_ASSERT_TRUE(prv_screen_sent((uint16_t)(((uint16_t)g_vs.code_len << 8) | 0x11u)));
    }
}

/* ------------------------------------------------------------------------- */
/* Input-test mode (task tail)                                               */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Input-test mode reports each pending input event as a Ninebot frame
 * \note            No state handler runs; for EVT+8/+0x50/+0x44/+0x38/+0x74/+0x2C/+0x20/+0x14/
 *                      +0x5C/+0x68 == 1 (records 0, 6, 5, 4, 9, 3, 2, 1, 7, 8) the code is cleared and
 *                      report_ninebot(0x16, 0x3D, 2, 0x56, 1..10)
 *                      sends one frame: CAN 0x18EF3D16, data 01 56 <code> 00 00.
 */
static void test_input_test_reports_events(void) {
    static const prv_event_t events[] = {
        {FM_MON_POWER_BTN, 1u}, {FM_MON_PARK, 2u},      {FM_MON_GEAR_UP, 3u},       {FM_MON_GEAR_DOWN, 4u},
        {FM_MON_LIGHT_EN, 5u},  {FM_MON_INTERLOCK, 6u}, {FM_MON_SUBSTATE_PREV, 7u}, {FM_MON_SUBSTATE_NEXT, 8u},
        {FM_MON_LIGHT_LO, 9u},  {FM_MON_LIGHT_HI, 10u},
    };
    const tst_can_frame_t* frame;
    uint32_t idx;

    for (idx = 0u; idx < sizeof(events) / sizeof(events[0]); ++idx) {
        setUp();
        prv_enter(VCU_FSM_RIDE);
        g_vs.input_test_mode = 1u;
        g_evt.mon[events[idx].record].code = 1u;
        prv_ticks(1u);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0u, g_evt.mon[events[idx].record].code, "code consumed");
        frame = prv_event_frame(events[idx].code);
        TEST_ASSERT_NOT_NULL_MESSAGE(frame, "event frame sent");
        TEST_ASSERT_EQUAL_UINT8(5u, frame->dlc);
        TEST_ASSERT_EQUAL_HEX8(EVT_CAN_CTL, frame->data[0]);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, prv_event_count(), "one frame per run");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(VCU_FSM_RIDE, FSM_STATE, "no state handler runs");
    }
}

/**
 * \brief           Input-test events have a priority: the power button wins, one per run
 * \note            Each `== 1` branch returns after its frame (events 1..7)
 *                      or ends the if-else chain (8..10).
 */
static void test_input_test_one_event_per_run(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.input_test_mode = 1u;
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    g_evt.mon[FM_MON_PARK].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_NOT_NULL(prv_event_frame(1u));
    TEST_ASSERT_NULL(prv_event_frame(2u));
    TEST_ASSERT_EQUAL_UINT8(1u, g_evt.mon[FM_MON_PARK].code);
    prv_ticks(1u);
    TEST_ASSERT_NOT_NULL(prv_event_frame(2u));
}

/**
 * \brief           Input-test event 1 and 8 output prep and the event-7 anti-theft latch
 * \note            vehicle_event1_prep (event 1): PB13 0, PA6 0, PB14 = !VS+0;
 *                      vehicle_lock_antitheft_latch (event 7): PB13 0, PA6 1, PB14 = !VS+0;
 *                      vehicle_event8_prep (event 8): PB13 1, PA6 1, PB14 = !VS+0.
 */
static void test_input_test_output_latches(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.input_test_mode = 1u;
    gpio_bits_set(GPIOB, PIN_BRAKE);
    gpio_bits_set(GPIOA, PIN_IND);
    g_evt.mon[FM_MON_POWER_BTN].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, PIN_BRAKE));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOA, PIN_IND));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOB, PIN_THEFT));

    g_vs.region = 1u;
    g_evt.mon[FM_MON_SUBSTATE_PREV].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, PIN_BRAKE));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOA, PIN_IND));
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, PIN_THEFT));

    g_vs.region = 0u;
    g_evt.mon[FM_MON_SUBSTATE_NEXT].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOB, PIN_BRAKE));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOA, PIN_IND));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOB, PIN_THEFT));
}

/**
 * \brief           Input-test events 9 and 10 enable subsystem A (PB10 low) / B (PB0 low)
 * \note            vehicle_event9_prep: control_subsys_a_enable(1) (PB10 clear), MODE+0x10 low
 *                      bits = 2, report 0x60 = 2. vehicle_event10_prep: control_subsys_b_enable(1) (PB0
 *                      clear), low bits = 1.
 */
static void test_input_test_subsystem_enables(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.input_test_mode = 1u;
    gpio_bits_set(GPIOB, GPIO_PINS_10 | GPIO_PINS_0);
    g_evt.mon[FM_MON_LIGHT_LO].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, GPIO_PINS_10));
    TEST_ASSERT_EQUAL_UINT8(1u, prv_out(GPIOB, GPIO_PINS_0));
    TEST_ASSERT_EQUAL_HEX16(0x0002u, g_mode.enable_flags & 0x0003u);
    g_evt.mon[FM_MON_LIGHT_HI].code = 1u;
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, prv_out(GPIOB, GPIO_PINS_0));
    TEST_ASSERT_EQUAL_HEX16(0x0001u, g_mode.enable_flags & 0x0003u);
}

/**
 * \brief           Input-test travel debounce: throttle in [0x33, 0x9B) then full travel reports 0x0B
 * \note            (thr >> 4) - 0x33 < 0x68 -> latch (g_deb +0x00) = 1, return; >= 0x9B with the
 *                      latch -> latch 0, report_ninebot(.., 0x0B). Same for PA1
 *                      (latch +2, event 0x0C) and PA4 (latch +1, event 0x0D).
 */
static void test_input_test_travel_endstops(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.input_test_mode = 1u;
    tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0x40u << 4);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_deb.thr_travel);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_event_count());
    tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0x9Bu << 4);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_deb.thr_travel);
    TEST_ASSERT_NOT_NULL(prv_event_frame(0x0Bu));
    tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0u);

    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_HALF, 0x33u << 4);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_deb.brake_half_travel);
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_HALF, 0x0FFFu);
    prv_ticks(1u);
    TEST_ASSERT_NOT_NULL(prv_event_frame(0x0Cu));
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_HALF, 0u);

    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, 0x9Au << 4);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_deb.brake_full_travel);
    tst_fm_adc_set(VCU_ADC_SLOT_BRAKE_FULL, 0x9Bu << 4);
    prv_ticks(1u);
    TEST_ASSERT_EQUAL_UINT8(0u, g_deb.brake_full_travel);
    TEST_ASSERT_NOT_NULL(prv_event_frame(0x0Du));
}

/**
 * \brief           Input-test full travel without a prior in-travel reading reports nothing
 * \note            The end-stop report needs the latch set (g_deb.thr_travel == 1).
 */
static void test_input_test_endstop_needs_travel(void) {
    prv_enter(VCU_FSM_IDLE);
    g_vs.input_test_mode = 1u;
    tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0x0FFFu);
    prv_ticks(2u);
    TEST_ASSERT_EQUAL_UINT32(0u, prv_event_count());
    tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0u);
}

/**
 * \brief           A run that reports an input event skips the lever travel debounce
 * \note            With the throttle in travel and an event pending (one with an output prep, one
 *                      without, one from the tail of the chain), the run sends only the event frame
 *                      and leaves the travel latch alone.
 */
static void test_input_test_event_skips_travel(void) {
    static const uint8_t records[] = {FM_MON_POWER_BTN, FM_MON_PARK, FM_MON_SUBSTATE_NEXT, FM_MON_LIGHT_HI};
    uint32_t idx;

    for (idx = 0u; idx < sizeof(records); ++idx) {
        setUp();
        prv_enter(VCU_FSM_IDLE);
        g_vs.input_test_mode = 1u;
        tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0x40u << 4);
        g_evt.mon[records[idx]].code = 1u;
        prv_ticks(1u);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0u, g_deb.thr_travel, "travel latch untouched");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(1u, prv_event_count(), "only the event frame");
        tst_fm_adc_set(VCU_ADC_SLOT_THROTTLE, 0u);
    }
}

/**
 * \brief           Suite entry: Unity over ITM, semihosting exit with the failure count
 * \return          never returns (tst_exit)
 */
int main(void) {
    tst_init();
    UNITY_BEGIN();
    RUN_TEST(test_state_change_latch);
    RUN_TEST(test_state_change_into_power_on_keeps_start);
    RUN_TEST(test_event_block_clear);
    RUN_TEST(test_idle_pending_start_opens_window);
    RUN_TEST(test_idle_start_without_lock);
    RUN_TEST(test_idle_start_with_lock);
    RUN_TEST(test_idle_remote_lock_refuses_start);
    RUN_TEST(test_idle_charger_in_session_to_charge_standby);
    RUN_TEST(test_idle_keeps_charger_bit);
    RUN_TEST(test_idle_charger_edge_opens_window);
    RUN_TEST(test_idle_button_wake_keepalive);
    RUN_TEST(test_idle_lapsed_window_powers_down);
    RUN_TEST(test_idle_serial_wake_keepalive);
    RUN_TEST(test_idle_app_register_timeout);
    RUN_TEST(test_idle_fallen_power_down);
    RUN_TEST(test_idle_long_press_upright_stays);
    RUN_TEST(test_power_on_to_ride);
    RUN_TEST(test_power_on_to_locked);
    RUN_TEST(test_power_on_shutdown);
    RUN_TEST(test_power_on_waits_for_arm_gate);
    RUN_TEST(test_power_on_keeps_run_bit);
    RUN_TEST(test_locked_shows_passcode_screen);
    RUN_TEST(test_locked_app_unlock_to_ride);
    RUN_TEST(test_locked_shutdown_and_alarm);
    RUN_TEST(test_locked_arms_drive_fault_state);
    RUN_TEST(test_alarm_arms_drive_fault_state);
    RUN_TEST(test_ride_shutdown);
    RUN_TEST(test_ride_charger_to_charging);
    RUN_TEST(test_power_down_releases_latch);
    RUN_TEST(test_power_down_poweroff_beep_skips_long_beep);
    RUN_TEST(test_power_down_screen_commits);
    RUN_TEST(test_power_down_monitors_power_button);
    RUN_TEST(test_power_down_button_press_cancels);
    RUN_TEST(test_power_down_holds_while_button_pressed);
    RUN_TEST(test_power_down_with_charger_to_charge_standby);
    RUN_TEST(test_power_down_keep_awake_to_charge_standby);
    RUN_TEST(test_sleep_motion_wake_to_alarm);
    RUN_TEST(test_sleep_button_or_serial_wake_to_idle);
    RUN_TEST(test_sleep_released_button_edge_dropped);
    RUN_TEST(test_alarm_button_to_idle);
    RUN_TEST(test_alarm_tilt_or_remote_unlock_to_power_down);
    RUN_TEST(test_alarm_guard_then_shutdown);
    RUN_TEST(test_alarm_enabled_shows_siren);
    RUN_TEST(test_alarm_timeout_tilt);
    RUN_TEST(test_charge_standby_settle);
    RUN_TEST(test_charge_standby_charger_gone);
    RUN_TEST(test_charge_standby_charge_screen);
    RUN_TEST(test_charge_standby_schedule_screen);
    RUN_TEST(test_charge_standby_start);
    RUN_TEST(test_charge_standby_power_button);
    RUN_TEST(test_charge_standby_screen_hold);
    RUN_TEST(test_charge_standby_key_hold);
    RUN_TEST(test_ride_session_screen);
    RUN_TEST(test_ride_session_other_screen);
    RUN_TEST(test_power_on_first_screen);
    RUN_TEST(test_input_test_reports_events);
    RUN_TEST(test_input_test_one_event_per_run);
    RUN_TEST(test_input_test_output_latches);
    RUN_TEST(test_input_test_subsystem_enables);
    RUN_TEST(test_input_test_travel_endstops);
    RUN_TEST(test_input_test_endstop_needs_travel);
    RUN_TEST(test_input_test_event_skips_travel);
    tst_exit(UNITY_END());
    return 0;
}
