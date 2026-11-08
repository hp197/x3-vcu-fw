/**
 * \file            board.h
 * \brief           Shared configuration for the x3 VCU application
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef BOARD_H
#define BOARD_H

#include "at32f415.h"
#include "model.h" /* VCU_MODEL and the feature switches */

/* System clock: the 16 MHz HEXT crystal via `PLL` gives `HEXT * 9 = 144 MHz`
   (the `crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_9)` settings -- the PLL is fed from
   HEXT directly, no /2 predivider). `SystemInit` resets the tree to the internal HICK;
   `main()` then brings up HEXT + PLL in `clock_config()`. */
#define SYSCLK_HZ 144000000u
#define PCLK1_HZ  (SYSCLK_HZ / 2u) /*!< APB1 = HCLK / 2 = 72 MHz (USART2, TMR5) */
#define PCLK2_HZ  (SYSCLK_HZ / 2u) /*!< APB2 = HCLK / 2 = 72 MHz                */

/* The application uses a hand-rolled cooperative scheduler (no RTOS) with a
   5-slot software-timer table. Each slot is dedicated to one periodic task, in the
   fixed order `main` registers them. The terminal enumerator gives the table size,
   so the slot table and \ref SCHED_SLOTS can never drift apart. */
typedef enum {
    SLOT_VEHICLE_FSM = 0, /*!< vehicle state machine   */
    SLOT_DISPATCH = 1,    /*!< divider / timeout tick  */
    SLOT_CAN = 2,         /*!< CAN TX cadence          */
    SLOT_FAST = 3,        /*!< fast service            */
    SLOT_SLOW = 4,        /*!< slow report             */
    SCHED_SLOT_COUNT      /*!< number of task slots (== 5) */
} sched_slot_t;

#define SCHED_SLOTS       SCHED_SLOT_COUNT

/* Application vector table. The bootloader owns 0x08000000; the app image (and its
   vector table) starts at 0x08001000 (see x3-vcu-fw.ld). */
#define APP_VECTOR_TABLE  0x08001000u

/* Task periods, in scheduler ticks -- the raw counts `main` registers
   (10/10/10/1/300000). One tick is one TMR5 overflow: TMR5 counts at 10 kHz with
   pr = 9 (10 counts), so a tick is 1.0 ms. The dispatch compare is inclusive
   (`tick - last >= period`), so a slot runs every `period` ticks. */
#define SCHED_PERIOD_TASK 10u     /* FSM / dispatch / CAN cadence: every 10 ticks = 10 ms    */
#define SCHED_PERIOD_FAST 1u      /* fast service: every tick = 1 ms                         */
#define SCHED_PERIOD_SLOW 300000u /* slow report: every 300000 ticks = 300 s                 */

/** \brief Enable one peripheral clock: the write of crm_periph_clock_enable(periph, TRUE) */
#define BOARD_CLOCK_ON(periph) (CRM_REG(periph) |= CRM_REG_BIT(periph))

/** \brief CRM apb2en offset, the register field of a CRM_*_PERIPH_CLOCK value (MAKE_VALUE) */
#define BOARD_APB2_OFFSET      0x18u

/** \brief APB2 enable bit of a peripheral clock (a compile error when the clock is not on APB2) */
#define BOARD_APB2_CLK(periph)                                                                                         \
    (CRM_REG_BIT(periph) + 0u * sizeof(char[(((periph) >> 16) == BOARD_APB2_OFFSET) ? 1 : -1]))

/** \brief Enable APB2 peripheral clocks together (\ref BOARD_APB2_CLK bits): one apb2en write */
#define BOARD_APB2_CLOCKS_ON(bits) (CRM->apb2en |= (bits))

/**
 * \brief           Apply an IOMUX pin remap: what gpio_pin_remap_config(remap, TRUE) leaves
 * \param[in]       remap: a *_GMUX_* value: register offset [31:24], field offset [23:16],
 *                      field width [15:8], field value [7:0]
 * \note            One read-modify-write of the remap field; with a constant `remap` it folds to
 *                      a load, a mask and a store.
 */
static inline void board_pin_remap(uint32_t remap) {
    volatile uint32_t* reg = (volatile uint32_t*)(IOMUX_BASE + (remap >> 24));
    uint32_t shift = (remap >> 16) & 0xFFu;
    uint32_t mask = ((1u << ((remap >> 8) & 0xFFu)) - 1u) << shift;

    *reg = (*reg & ~mask) | ((remap & 0xFFu) << shift);
}

/**
 * \brief           NVIC priority byte of a (preemption, sub) priority pair
 * \note            The encoding of NVIC_PRIORITY_GROUP_2 (2 preemption + 2 sub-priority bits of
 *                      the 4 implemented ones), which vcu_periph_init selects before any IRQ is
 *                      enabled: what nvic_irq_enable stores for the pair under that group.
 */
#define BOARD_IRQ_PRIO(pre, sub) ((uint8_t)((((uint32_t)(pre) << 2) | (uint32_t)(sub)) << (8u - __NVIC_PRIO_BITS)))

/**
 * \brief           Set the priority of an IRQ and enable it (the writes of nvic_irq_enable)
 * \param[in]       irqn: device IRQ number (>= 0)
 * \param[in]       prio: priority byte, \ref BOARD_IRQ_PRIO
 */
static inline void board_irq_enable(IRQn_Type irqn, uint8_t prio) {
    NVIC->IP[(uint32_t)irqn] = prio;
    NVIC->ISER[(uint32_t)irqn >> 5] = 1u << ((uint32_t)irqn & 0x1Fu);
}

#endif /* BOARD_H */
