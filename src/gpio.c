/**
 * \file            gpio.c
 * \brief           GPIO + EXINT: edge-latched wake inputs + discrete outputs
 * \note            Wake inputs: EXINT3/PA3 (pull-up, falling edge, NVIC IRQ9) and EXINT10/PA10 +
 *                      EXINT13/PC13 (falling edge, shared NVIC IRQ40). Outputs: PB12 (power latch),
 *                      PB13/PA6/PB14 (indicators), PC14 (fault latch).
 *
 *                      Each ISR latches a distinct code into g_vs.wake_src (PA3=4, PA10=2, PC13=1)
 *                      and masks its own NVIC line; the vehicle FSM reads and clears it. The lines
 *                      aren't re-armed here -- the standby cycle re-enables them through the two
 *                      EXINT inits below. PC13 is routed only when IN2 cfg_flags bit15 or bit2 is
 *                      set (\ref vcu_board_pc13_wake_present). EXINT routing uses the IOMUX
 *                      exintc1..4 port fields (the AT32F415 has no SCFG block), gated by
 *                      CRM_IOMUX_PERIPH_CLOCK. Nothing here runs at boot: vcu_periph_init sets up
 *                      the outputs, standby arms the EXINT lines.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "gpio.h"
#include "at32f415.h"
#include "board.h"     /* board_irq_enable */
#include "vcu_state.h" /* g_in2.f.cfg_flags (PC13 gate), g_vs.wake_src (ISR latch) */

/** \brief NVIC preemption priority for both EXINT lines */
#define VCU_EXINT_PREEMPT_PRIORITY 1u

/** \brief NVIC sub-priority for both EXINT lines */
#define VCU_EXINT_SUB_PRIORITY     0u

/*
 * EXINT wake inputs. The per-pin roles in the comments are the likely board functions.
 * PA10 and PC13 share EXINT15_10 (NVIC IRQ40); PA3 has its own EXINT3 (NVIC IRQ9).
 */
#define VCU_WAKE_PA3_PORT          GPIOA       /* likely brake-lever / mode button   */
#define VCU_WAKE_PA3_PIN           GPIO_PINS_3 /* active-low, pull-up, falling edge   */
#define VCU_WAKE_PA3_PIN_SOURCE    GPIO_PINS_SOURCE3
#define VCU_WAKE_PA3_LINE          EXINT_LINE_3
#define VCU_WAKE_PA3_IRQ           EXINT3_IRQn /* NVIC IRQ9 */

#define VCU_WAKE_PA10_PORT         GPIOA        /* likely charger / dock detect, floating   */
#define VCU_WAKE_PA10_PIN          GPIO_PINS_10 /* externally driven, falling edge           */
#define VCU_WAKE_PA10_PIN_SOURCE   GPIO_PINS_SOURCE10
#define VCU_WAKE_PA10_LINE         EXINT_LINE_10

#define VCU_WAKE_PC13_PORT         GPIOC        /* likely power / wake button (HW variant)   */
#define VCU_WAKE_PC13_PIN          GPIO_PINS_13 /* pull-up, falling edge                      */
#define VCU_WAKE_PC13_PIN_SOURCE   GPIO_PINS_SOURCE13
#define VCU_WAKE_PC13_LINE         EXINT_LINE_13

#define VCU_WAKE_PA10_PC13_IRQ     EXINT15_10_IRQn /* NVIC IRQ40, shared by PA10 + PC13 */

/*
 * Discrete push-pull outputs (drive strength "stronger"). The loads named in the
 * comments are the likely board functions.
 */
#define VCU_OUT_POWER_LATCH_PORT   GPIOB
#define VCU_OUT_POWER_LATCH_PIN    GPIO_PINS_12 /* likely power-hold / self-latch */
#define VCU_OUT_LOAD1_PORT         GPIOB
#define VCU_OUT_LOAD1_PIN          GPIO_PINS_13 /* likely light / load enable */
#define VCU_OUT_AUX_PORT           GPIOA
#define VCU_OUT_AUX_PIN            GPIO_PINS_6  /* likely aux enable */
#define VCU_OUT_LOAD2_PORT         GPIOB
#define VCU_OUT_LOAD2_PIN          GPIO_PINS_14 /* likely light / load enable */

/**
 * \brief           Set the 4-bit config field of a group of pins (see gpio.h)
 * \param[in]       port: GPIO port
 * \param[in]       pins: pin mask within the port
 * \param[in]       cfg: field value, one of the VCU_GPIO_CFG_* values
 */
void vcu_gpio_config(gpio_type* port, uint32_t pins, uint32_t cfg) {
    volatile uint32_t* cfgr = &port->cfglr; /* cfglr (pins 0..7), then cfghr (pins 8..15) */
    uint32_t pin;
    uint32_t shift;

    if (cfg == VCU_GPIO_CFG_IN_PU) {
        port->scr = pins;
    }
    for (pin = 0u; pin < 16u; pin++) {
        if (((pins >> pin) & 1u) != 0u) {
            shift = (pin & 7u) * 4u;
            cfgr[pin >> 3] = (cfgr[pin >> 3] & ~(0xFu << shift)) | (cfg << shift);
        }
    }
}

/**
 * \brief           Arm EXINT lines as edge-triggered interrupts (see gpio.h)
 * \param[in]       lines: EXINT line mask
 * \param[in]       rising: 1 = rising edge, 0 = falling edge
 */
void vcu_exint_arm(uint32_t lines, uint32_t rising) {
    EXINT->inten &= ~lines;
    EXINT->evten &= ~lines;
    EXINT->inten |= lines;
    EXINT->polcfg1 &= ~lines;
    EXINT->polcfg2 &= ~lines;
    if (rising != 0u) {
        EXINT->polcfg1 |= lines;
    } else {
        EXINT->polcfg2 |= lines;
    }
}

/**
 * \brief           Whether the PC13 wake input is routed
 * \return          1 when bit15 or bit2 of g_in2.f.cfg_flags (IN2 +0x3A) is set, else 0
 * \note            A runtime flag in the persisted IN2 record, not a compile-time board variant.
 */
uint8_t vcu_board_pc13_wake_present(void) {
    return ((g_in2.f.cfg_flags & ((1u << 15) | (1u << 2))) != 0u) ? 1u : 0u;
}

/**
 * \brief           Configure EXINT3 / PA3 (pull-up, falling edge, IRQ9)
 * \note            Likely pin function: brake-lever / mode button (active-low, debounced one-shot).
 */
void vcu_gpio_exint3_init(void) {
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_GPIOA_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_IOMUX_PERIPH_CLOCK));

    vcu_gpio_config(VCU_WAKE_PA3_PORT, VCU_WAKE_PA3_PIN, VCU_GPIO_CFG_IN_PU);

    IOMUX->exintc1_bit.exint3 = GPIO_PORT_SOURCE_GPIOA; /* line 3 <- PA3 (gpio_exint_line_config) */
    vcu_exint_arm(VCU_WAKE_PA3_LINE, 0u);               /* falling edge, inten |= line3 */

    board_irq_enable(VCU_WAKE_PA3_IRQ, BOARD_IRQ_PRIO(VCU_EXINT_PREEMPT_PRIORITY, VCU_EXINT_SUB_PRIORITY));
    EXINT->intsts = VCU_WAKE_PA3_LINE; /* drop spurious pending */
}

/**
 * \brief           Configure EXINT10 / PA10 and (HW-variant) EXINT13 / PC13 (shared IRQ40)
 * \note            PA10 is floating (externally driven); PC13 is pull-up.
 *                      The PC13 GPIO pin is configured unconditionally; only its EXINT routing,
 *                      its bit in the combined line mask, and its pending-clear are gated on the
 *                      HW-variant flag. A single \ref vcu_exint_arm arms the group.
 *                      Likely functions: PA10 = charger/dock detect or wake/handshake;
 *                      PC13 = power/wake button (only populated on some revisions).
 */
void vcu_gpio_exint_10_13_init(void) {
    uint32_t lines = VCU_WAKE_PA10_LINE;

    /* Enable the clocks of the ports the pins below are on (GPIOA, GPIOC) plus IOMUX for the
       EXINT router. vcu_periph_init normally has these on already. */
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_IOMUX_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_GPIOA_PERIPH_CLOCK)
                         | BOARD_APB2_CLK(CRM_GPIOC_PERIPH_CLOCK));

    vcu_gpio_config(VCU_WAKE_PA10_PORT, VCU_WAKE_PA10_PIN, VCU_GPIO_CFG_IN_FLOAT); /* PA10: floating */
    vcu_gpio_config(VCU_WAKE_PC13_PORT, VCU_WAKE_PC13_PIN, VCU_GPIO_CFG_IN_PU);    /* PC13: pull-up, always */

    /* EXINT line routing (gpio_exint_line_config): line 10 <- PA10, line 13 <- PC13. */
    IOMUX->exintc3_bit.exint10 = GPIO_PORT_SOURCE_GPIOA;
    if (vcu_board_pc13_wake_present() != 0u) {
        IOMUX->exintc4_bit.exint13 = GPIO_PORT_SOURCE_GPIOC;
        lines |= VCU_WAKE_PC13_LINE;
    }

    vcu_exint_arm(lines, 0u); /* falling edge */
    board_irq_enable(VCU_WAKE_PA10_PC13_IRQ, BOARD_IRQ_PRIO(VCU_EXINT_PREEMPT_PRIORITY, VCU_EXINT_SUB_PRIORITY));
    EXINT->intsts = lines; /* drop spurious pending */
}

/**
 * \brief           PB12 power-latch pad as a push-pull output
 * \note            Only configures the pin; the caller drives it.
 */
void vcu_gpio_power_latch_init(void) {
    BOARD_CLOCK_ON(CRM_GPIOB_PERIPH_CLOCK);
    vcu_gpio_config(VCU_OUT_POWER_LATCH_PORT, VCU_OUT_POWER_LATCH_PIN, VCU_GPIO_CFG_OUT_PP);
}

/**
 * \brief           PB13 / PA6 / PB14 indicator outputs, push-pull
 * \note            PB13 + PB14, then PA6. Only configures the pins; the caller drives them low.
 */
void vcu_gpio_indicator_outputs_init(void) {
    BOARD_APB2_CLOCKS_ON(BOARD_APB2_CLK(CRM_GPIOB_PERIPH_CLOCK) | BOARD_APB2_CLK(CRM_GPIOA_PERIPH_CLOCK));

    vcu_gpio_config(VCU_OUT_LOAD1_PORT, VCU_OUT_LOAD1_PIN | VCU_OUT_LOAD2_PIN, VCU_GPIO_CFG_OUT_PP); /* GPIOB */
    vcu_gpio_config(VCU_OUT_AUX_PORT, VCU_OUT_AUX_PIN, VCU_GPIO_CFG_OUT_PP);
}

#define GPIO_PWR_LATCH_PIN GPIO_PINS_14 /* GPIOC pin 14 (0x4000) drive/power-latch pad */

/* (Re)configure the power/fault-latch pad (GPIOC pin 14) as an output before the vehicle FSM's */
/* latch toggles it. Field = 0x1 (output, drive stronger) | (mode & 0x04); mode picks the       */
/* output type (0 = push-pull, as both callers pass; 0x04 = open drain).                         */
void vcu_gpio_fault_latch_init(uint8_t mode) { /* drive disable */
    BOARD_CLOCK_ON(CRM_GPIOC_PERIPH_CLOCK);
    vcu_gpio_config(GPIOC, GPIO_PWR_LATCH_PIN, VCU_GPIO_CFG_OUT_PP | ((uint32_t)mode & GPIO_OUTPUT_OPEN_DRAIN));
}

/**
 * \brief           PA10 (EXINT10 wake line) level test, active-low
 * \note            Reads GPIOA (0x40010800) idt (@GPIO+0x08) with mask 0x400; the line counts as
 *                      active while pin 10 is LOW.
 * \return          `1` when PA10 is LOW, `0` when HIGH
 */
int32_t vcu_gpio_pa10_active(void) {
    if (gpio_input_data_bit_read(VCU_WAKE_PA10_PORT, VCU_WAKE_PA10_PIN) != RESET) {
        return 0; /* pin 10 HIGH -> inactive */
    }
    return 1;     /* pin 10 LOW  -> active   */
}

/**
 * \brief           EXINT line 3 ISR (PA3): clear pending, latch code 4 in g_vs.wake_src, mask IRQ9
 */
void EXINT3_IRQHandler(void) {
    if ((EXINT->intsts & VCU_WAKE_PA3_LINE) != 0u) {
        EXINT->intsts = VCU_WAKE_PA3_LINE;
        g_vs.wake_src = VCU_GPIO_EVENT_PA3; /* = 4 */
        NVIC_DisableIRQ(VCU_WAKE_PA3_IRQ);
    }
}

/**
 * \brief           EXINT lines 15..10 ISR (PA10, PC13): latch code, clear pending, mask own line
 * \note            Line 10 is checked first and latches 2; line 13 latches 1. Each clears its
 *                      pending bit and masks the shared IRQ40.
 */
void EXINT15_10_IRQHandler(void) {
    if ((EXINT->intsts & VCU_WAKE_PA10_LINE) != 0u) {
        g_vs.wake_src = VCU_GPIO_EVENT_PA10; /* = 2 */
        EXINT->intsts = VCU_WAKE_PA10_LINE;
        NVIC_DisableIRQ(VCU_WAKE_PA10_PC13_IRQ);
    }

    if ((EXINT->intsts & VCU_WAKE_PC13_LINE) != 0u) {
        g_vs.wake_src = VCU_GPIO_EVENT_PC13; /* = 1 */
        EXINT->intsts = VCU_WAKE_PC13_LINE;
        NVIC_DisableIRQ(VCU_WAKE_PA10_PC13_IRQ);
    }
}
