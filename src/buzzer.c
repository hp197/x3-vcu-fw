/**
 * \file            buzzer.c
 * \brief           Buzzer: TMR1 channel-1 PWM bring-up, tone on/off leaves and the beep request
 * \note            Bring-up, tone reconfigure, tone on, tone off and the shared beep-pattern
 *                      request every module posts. See the section note below for the register
 *                      map.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "buzzer.h"
#include "board.h" /* BOARD_CLOCK_ON */
#include "gpio.h"  /* vcu_gpio_config */

#include "vcu_state.h" /* g_lowglob beep request block */

/* ------------------------------------------------------------------------- */
/* Buzzer PWM (TMR1 channel 1). The config leaf programs TMR1 (base 0x40012C00) for */
/* a square wave at `tone` Hz and the start leaf enables the channel-1 output.       */
/* Register map (word/DMA offsets): ctrl1 @+0x00 (bit0 = counter enable), cval       */
/* @+0x24, pr @+0x2C, c1dt @+0x34, cctrl @+0x20 (bit0 = ch1 output enable). The      */
/* effective timer clock is 1,000,000 Hz, so period = timerclk/tone and the 50%-duty */
/* compare = period/2.                                                               */
/* ------------------------------------------------------------------------- */
#define BUZZER_TMR                TMR1     /* 0x40012C00                                    */
#define BUZZER_CTRL2_C1IOS        0x100u   /* ctrl2 c1ios [8]: channel 1 idle state high    */
#define BUZZER_CTRL2_C1_IDLE_MASK 0x300u   /* ctrl2 c1ios [8] + c1cios [9]                  */
#define BUZZER_CCTRL_C1_MASK      0xFu     /* cctrl c1en [0], c1p [1], c1cen [2], c1cp [3]  */
#define BUZZER_CM1_C1_MASK        0x73u    /* cm1 c1c [1:0] (0 = output), c1octrl [6:4]     */
#define BUZZER_TIMER_CLK_HZ       1000000u /* effective TMR1 clock                          */

/**
 * \brief           Buzzer bring-up: PA8 = TMR1_CH1 AF + TMR1 1 MHz PWM base
 * \note            Pin first, then timer. TMR1 runs off the APB2 timer clock (72 MHz x2 =
 *                      144 MHz), /144 gives the 1 MHz tick that BUZZER_TIMER_CLK_HZ assumes.
 *                      The period of 1 is a placeholder; each tone reloads pr/c1dt in
 *                      buzzer_config. OEN (brkdt) is set because TMR1 is an advanced timer, and the
 *                      channel output stays gated off until buzzer_on.
 */
void buzzer_init(void) {
    /* PA8 = TMR1_CH1 alternate function, push-pull. */
    BOARD_CLOCK_ON(CRM_GPIOA_PERIPH_CLOCK);
    vcu_gpio_config(GPIOA, GPIO_PINS_8, VCU_GPIO_CFG_MUX_PP);

    /* TMR1 1 MHz tick (144 MHz / 144), channel-1 PWM A, output gated off. */
    BOARD_CLOCK_ON(CRM_TMR1_PERIPH_CLOCK);
    /* tmr_base_init: pr = 1, div = 0x8F, then a software overflow event loads them. */
    BUZZER_TMR->pr = 1u;
    BUZZER_TMR->div = 143u;
    BUZZER_TMR->swevt_bit.ovfswtr = TRUE;
    BUZZER_TMR->ctrl1_bit.cnt_dir = TMR_COUNT_UP;
    /* Channel 1 output config (what tmr_output_channel_config writes for PWM A, idle high,
       active high, outputs off): ctrl2 c1ios = 1 / c1cios = 0, cctrl c1en / c1p / c1cen / c1cp
       = 0 (the channel output stays gated off), cm1 c1c = 0 (output) / c1octrl = PWM A. */
    BUZZER_TMR->ctrl2 = (BUZZER_TMR->ctrl2 & ~BUZZER_CTRL2_C1_IDLE_MASK) | BUZZER_CTRL2_C1IOS;
    BUZZER_TMR->cctrl &= ~BUZZER_CCTRL_C1_MASK;
    BUZZER_TMR->cm1 = (BUZZER_TMR->cm1 & ~BUZZER_CM1_C1_MASK) | ((uint32_t)TMR_OUTPUT_CONTROL_PWM_MODE_A << 4);
    BUZZER_TMR->c1dt = 0u;
    BUZZER_TMR->brk_bit.oen = TRUE; /* brkdt.oen (advanced timer)    */
    BUZZER_TMR->ctrl1_bit.tmren = TRUE;
}

/* Buzzer PWM reconfigure (registers only): stop the counter, load the period
   (timerclk/tone) and a 50%-duty compare (period/2), reset the counter value and
   restart. The channel-1 PWM mode and the output state stay as buzzer_init left them;
   the output is gated on cctrl bit0 by buzzer_on / buzzer_off.                        */
static void buzzer_config(uint16_t tone) {
    uint32_t period = BUZZER_TIMER_CLK_HZ / tone; /* pr = timerclk/tone */

    BUZZER_TMR->ctrl1_bit.tmren = FALSE;                            /* ctrl1 &= ~1 */
    BUZZER_TMR->pr = period;                                        /* pr   @+0x2C */
    BUZZER_TMR->c1dt = period >> 1;                                 /* c1dt @+0x34 (50%) */
    BUZZER_TMR->cval = 0u;                                          /* cval @+0x24 = 0   */
    BUZZER_TMR->ctrl1_bit.tmren = TRUE;                             /* ctrl1 |= 1 */
}

/* Buzzer on: reprogram the tone, then enable the channel-1 PWM output (cctrl @+0x20 bit0).
   Called from control.c and ninebot.c. */
void buzzer_on(uint16_t tone) {
    buzzer_config(tone);
    BUZZER_TMR->cctrl_bit.c1en = TRUE; /* cctrl bit0 = 1 (on)  */
}

/* Buzzer off: clear the TMR1 channel-1 PWM output enable (cctrl @+0x20 bit0), the inverse
   of buzzer_on. */
void buzzer_off(void) {                                    /* TMR1 ch1 output disable (buzzer off) */
    BUZZER_TMR->cctrl_bit.c1en = FALSE;                    /* cctrl bit0 = 0 (off) */
}

/* Beep request: the shared entry point every module calls. It doesn't drive the buzzer --
   it posts into the beep state block (g_lowglob). When the requested pattern id differs from
   the one latched, it stores the new id and rewinds the pattern-progress counter to 0. A
   separate pattern player reads this block and drives the TMR1 ch1 PWM via buzzer_on/off.    */

/* The compare uses the full argument; the store keeps its low byte. */
void buzzer_beep(int32_t pattern) {
    if ((int32_t)g_lowglob.beep_seq_id != pattern) {
        g_lowglob.beep_seq_id = (uint8_t)pattern;
        g_lowglob.beep_seq_tick = 0;
    }
}
