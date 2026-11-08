/**
 * \file            buzzer.h
 * \brief           Buzzer (TMR1 ch1 PWM on PA8) + beep request (x3 VCU) -- public API
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef BUZZER_H
#define BUZZER_H

#include "at32f415.h"

/**
 * \brief           Buzzer hardware bring-up
 * \note            PA8 as TMR1_CH1 alternate function, then TMR1 as a 1 MHz PWM-A base
 *                      with the channel output gated off (buzzer_on enables it).
 *                      Called once from vcu_periph_init.
 */
void buzzer_init(void);

/** \brief Buzzer-ON leaf: program TMR1 ch1 PWM to \p tone Hz
 *         (period = 1e6/tone, 50% duty) and enable the output. */
void buzzer_on(uint16_t tone);

/** \brief Buzzer-OFF leaf: clears TMR1 ch1 PWM output enable
 *         (cctrl bit0), the inverse of buzzer_on. */
void buzzer_off(void);

/** \brief The one buzzer tone every caller plays: 0xA8C = 2700 Hz. */
#define BUZZER_TONE 2700u

/**
 * \brief           Beep pattern ids (the selector byte at g_lowglob.beep_seq_id)
 * \note            Names are the shapes the pattern player (control_gesture_fsm) emits for
 *                      each id; ticks = player calls.
 */
typedef enum {
    BEEP_NONE = 0,        /*!< no pattern / player idle                         */
    BEEP_LONG = 1,        /*!< one long beep (~50 ticks)                         */
    BEEP_SHORT = 2,       /*!< one short beep (~10 ticks)                        */
    BEEP_DOUBLE = 3,      /*!< two short beeps                                   */
    BEEP_TRIPLE = 4,      /*!< three short beeps                                 */
    BEEP_DOUBLE_LONG = 5, /*!< two long beeps (50 on, 50 off, 50 on)             */
    BEEP_STOP = 6         /*!< silence the buzzer and return to idle             */
} buzzer_pattern_t;

/** \brief Alert / beep-pattern request: latch \p pattern (a \ref buzzer_pattern_t) into
 *         the beep state block (g_lowglob) for the pattern player. */
void buzzer_beep(int32_t pattern);

#endif /* BUZZER_H */
