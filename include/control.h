/**
 * \file            control.h
 * \brief           Control-loop / ADC front-end internal API (x3 VCU)
 * \note            VCU analog front-end and the input->drive-command pipeline: the ADC bring-up
 *                      and scan-buffer reader, plus the control-domain FSMs and helpers. The ADC
 *                      bring-up is vcu_control_init (configure + start), with control_adc_gpio_init
 *                      (PA0/1/4/5/7 analog) and control_adc_dma_init (DMA1_CH1 -> scan buffer).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "at32f415.h"

/**
 * \brief           Bring up the analog front-end: ADC1 5-channel scan + DMA1 ch1
 * \note            Called once from vcu_periph_init (config.c).
 */
void vcu_control_init(void);

/* ------------------------------------------------------------------------- */
/* ADC scan geometry.                                                        */
/*                                                                           */
/* The ordinary group is a 5-entry scan                                      */
/* (base-config ordinary_channel_length = 5) over ADC1 channels 0,1,4,5,7,   */
/* in sequence slots 1..5, each sampled at 239.5 cycles. DMA1 channel 1      */
/* copies the ADC1 ordinary data register into a 5-halfword                  */
/* circular buffer, so one buffer slot exists per sequence slot.             */
/* ------------------------------------------------------------------------- */
#define VCU_ADC_SCAN_LEN         5u /*!< ordinary-group length (base-config byte[3] = 5) */

/* PA1 and PA4 are both brake levers (physical side unknown, see pinmap-check.md):
   the brake byte is max(PA4, PA1 >> 1), so PA4 is the full-weight
   and PA1 the half-weight lever. */
#define VCU_ADC_CH_THROTTLE      ADC_CHANNEL_0 /*!< seq1: ADC1_IN0 / PA0, throttle                */
#define VCU_ADC_CH_BRAKE_HALF    ADC_CHANNEL_1 /*!< seq2: ADC1_IN1 / PA1, half-weight brake lever */
#define VCU_ADC_CH_BRAKE_FULL    ADC_CHANNEL_4 /*!< seq3: ADC1_IN4 / PA4, full-weight brake lever */
#define VCU_ADC_CH_ISENSE_PB10   ADC_CHANNEL_5 /*!< seq4: ADC1_IN5 / PA5, current sense, read while PB10 low */
#define VCU_ADC_CH_ISENSE_PB0    ADC_CHANNEL_7 /*!< seq5: ADC1_IN7 / PA7, current sense, read while PB0 low  */

/* Scan-buffer slot indices (the order the sequence fills the DMA buffer). */
#define VCU_ADC_SLOT_THROTTLE    0u /*!< DMA +0, PA0 */
#define VCU_ADC_SLOT_BRAKE_HALF  1u /*!< DMA +2, PA1 */
#define VCU_ADC_SLOT_BRAKE_FULL  2u /*!< DMA +4, PA4 */
#define VCU_ADC_SLOT_ISENSE_PB10 3u /*!< DMA +6, PA5 */
#define VCU_ADC_SLOT_ISENSE_PB0  4u /*!< DMA +8, PA7 */

/**
 * \brief           Read one raw cell of the ADC1 ordinary-scan DMA buffer
 * \param[in]       slot: scan-buffer slot index (`VCU_ADC_SLOT_*`, 0..VCU_ADC_SCAN_LEN-1)
 * \return          Latest 12-bit conversion DMA copied into that scan slot (0 for
 *                      an out-of-range slot)
 * \note            The analog-read path. The free-running ordinary scan fills a circular DMA
 *                      buffer (5 halfwords, stride 2); every leaf reader just indexes it through
 *                      this accessor, no live conversion. Slot order follows sequence slots 1..5
 *                      (channels 0,1,4,5,7 -> PA0,PA1,PA4,PA5,PA7): slot 0 throttle, slots 1/2
 *                      half/full brake levers, slots 3/4 current senses.
 */
uint16_t control_adc_raw(uint8_t slot);

/**
 * \brief           Number of drive-mode rows in the drive-mode parameter table
 * \note            ROM table, 3 rows x 9 `uint16_t` (0x12 bytes each), indexed by the live
 *                      drive row (g_vs.region, \ref vcu_region_t).
 */
#define VCU_MODE_ROWS 3u
#define VCU_MODE_COLS 9u

/**
 * \brief           One cell of the drive-mode parameter table
 * \param[in]       row: live drive row (g_vs.region)
 * \param[in]       col: column, 0..8
 * \return          the table halfword
 * \note            A row >= 3 wraps (% 3), as in the drive-command producer. Used by the
 *                      telemetry builder (column 6).
 */
uint16_t control_mode_param(uint8_t row, uint8_t col);

/**
 * \brief           Ride-mode identifiers (stored / app-facing values)
 * \note            Four ride modes, ascending: walk, eco, sport, power. Per config.h
 *                      `config_validate_ride_mode`, the stored ride mode (record `+0xB4`) is clamped
 *                      to {2,3,4} (default 2 = eco/sport/power); WALK (= 1) sits below the clamp
 *                      floor as the special low mode. The live drive row (g_vs.region, \ref
 *                      vcu_region_t) is a separate 0-based region index Global/Europe/Germany; the
 *                      stored-gear -> live-row map is in ridemode.c.
 * \note            control_mode_report_fsm's argument is a lighting/status code, not a ride mode
 *                      (see control.c).
 */
typedef enum {
    VCU_MODE_WALK = 1,  /*!< pedestrian / walk assist (below the {2,3,4} clamp floor) */
    VCU_MODE_ECO = 2,   /*!< clamp default                                            */
    VCU_MODE_SPORT = 3, /*!< sport                                                    */
    VCU_MODE_POWER = 4  /*!< power                                                    */
} vcu_ride_mode_t;

/* ------------------------------------------------------------------------- */
/* Control FSMs / helpers.                                                     */
/* ------------------------------------------------------------------------- */

void control_build_drive_cmd(void);             /*!< per-tick drive-command producer with per-mode ceiling selection */
void control_fault_drive_fsm(void);             /*!< drive overtemp fault + mode-report dispatch */
void control_lock_fsm(void);                    /*!< lock/standby arming state machine      */
void charge_schedule_tick(void);                /*!< scheduled-charging window tick           */
void control_charge_fsm(void);                  /*!< charge detect + relay state machine    */
void control_speed_lock_fsm(void);              /*!< over-speed / lock-arm supervisor       */
void charge_complete_tick(void);                /*!< charge-complete check + 6-step beep      */
void charger_detect_tick(void);                 /*!< charger detect + charge-control report   */
void control_charge_state_reset(void);          /*!< charge-detect / relay state reset      */
void control_gesture_fsm(void);                 /*!< timed button/gesture beep FSM          */
void control_deferred_action(void);             /*!< deferred-action trigger                */
uint32_t control_mode_report_fsm(uint32_t arg); /*!< mode/lighting status update + report (returns a status value) */
void control_countdown_beep(void);              /*!< countdown mirror + periodic beep       */
void control_dtc_set(uint32_t code);            /*!< latch active code + set DTC mask bit    */
void control_dtc_clear(uint32_t code);          /*!< clear active code + clear DTC mask bit  */
void control_output_light(void);                /*!< indicator output from the mode flag    */

/* ------------------------------------------------------------------------- */
/* Control-domain FSMs.                                                        */
/* ------------------------------------------------------------------------- */
void control_throttle_brake_scan(uint8_t fsm_state); /*!< 3-ch ADC scale/threshold -> fault + MODE+8 output word  */
void control_power_monitor_fsm(void);                /*!< I*V/100 power + 600-tick comm timeout + event FSM       */
void control_auto_off_tick(uint32_t reload);         /*!< standstill auto-off countdown -> VS shutdown request    */
void control_status_bit_mirror(void);                /*!< mirror IN2+0x3A bit0 into MODE+0x0A bit0                 */
void control_shutdown_countdown(uint32_t reload);    /*!< one-shot down-counter -> FSM state 4 (power-down)        */
void control_event_mask_reset(void);                 /*!< clear active-event byte + IN2+0xB2 + event masks        */
void control_dtc_mask_reset(void);                   /*!< clear active-error byte + lock_state + DTC masks        */
void control_diag_reset(void);                       /*!< reset wrapper (vcu_outputs_reset + conditional VS+5 clear) */
void control_mode_sync(void);                        /*!< subsystems A/B off + mask-0 report on change            */
void control_debounced_gpio_drive(void);             /*!< 10-tick debounce then aux-GPIO drive                    */
void control_esc_staging_defaults(void);             /*!< ESC staging-frame (g_mode) boot defaults               */

/* ------------------------------------------------------------------------- */
/* Drive-parameter init and anti-theft motion check.                           */
/* control_drive_param_init seeds the per-mode speed/limit profile registers; */
/* the drive producer selects and corrects the active ceiling from them.      */
/* ------------------------------------------------------------------------- */
void control_drive_param_init(void);             /*!< drive-parameter + per-mode profile-table init */
uint32_t control_alarm_motion_tick(int32_t run); /*!< anti-theft motion check / siren FSM (state 6); 1 at state-1 end */

/* Event / DTC latch primitives (canonical owners; faultmon.c calls these). */
void control_event_set(uint8_t ev);   /*!< raise event ev (latch + set mask bit)                   */
void control_event_clear(uint8_t ev); /*!< clear event ev (unlatch + clear mask bit)               */

/* Indicator-output GPIO primitives (shared leaf; config.c vcu_outputs_reset calls these). */
void control_out_pb13(int32_t a);         /*!< indicator output PB13 (a != 0 sets the pin)             */
void control_out_pa6(int32_t a);          /*!< indicator output PA6  (a != 0 sets the pin)             */
void control_out_pb14(int32_t a);         /*!< indicator output PB14 (a != 0 sets the pin)             */
void control_subsys_a_enable(int32_t en); /*!< subsystem A enable (GPIOB pin10, active-low) */
void control_subsys_b_enable(int32_t en); /*!< subsystem B enable (GPIOB pin0, active-low)  */

#endif /* CONTROL_H */
