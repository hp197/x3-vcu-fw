/**
 * \file            gpio.h
 * \brief           GPIO + EXINT subsystem: edge-latched wake inputs and discrete outputs
 * \note            Three active EXINT input lines each behave as an edge-triggered one-shot:
 *                      on their first falling edge the ISR latches a distinct event code and
 *                      masks its own NVIC line. The code goes into `g_vs.wake_src`, which the
 *                      vehicle FSM reads and clears. The lines are re-armed only by the standby
 *                      cycle, through \ref vcu_gpio_exint3_init and \ref vcu_gpio_exint_10_13_init
 *                      (both re-enable their NVIC line). The per-pin *functions* are the likely
 *                      board roles; the pins, edges, pulls and latch codes are fixed.
 *
 * \par Board pin map (AT32F415CBT7, LQFP48)
 *
 * Built from every gpio_init / pin access in the firmware. Device roles marked `?` are
 * likely roles, not confirmed. Boot level H/L = level driven by vcu_periph_init.
 *
 * JTAG is released (the SWJTAG_GMUX_010 remap) as the LAST step of
 * vcu_periph_init, after the SPI2 pins, the SPI2 init and the PA15 write; SWD stays on
 * PA13/PA14 permanently (the G3 / U2 boards strap-disable SWD; GT3 never does).
 *
 * \verbatim
 *  GPIOA                                          GPIOB
 *  -----------------------------------------      -----------------------------------------
 *  PA0  AN   ADC1 ch0  -> throttle                PB0  OUT H subsys B enable (act-low) [ctl]
 *  PA1  AN   ADC1 ch1  -> brake lever (half wt)   PB1  IN    node-6 comms line          [fm]
 *  PA2  AF   USART2 TX -> dashboard / BLE         PB2  IN    node-5 comms line          [fm]
 *  PA3  IN   pull-up: USART2 RX <- dash / BLE     PB3  AF    SPI2 SCK  (remap)         [sens]
 *            standby only: +EXINT3 wake           PB4  IN    SPI2 MISO (remap, pull-up)[sens]
 *  PA4  AN   ADC1 ch4  -> brake lever (full wt)   PB5  AF    SPI2 MOSI (remap)         [sens]
 *  PA5  AN   ADC1 ch5  -> current sense of PB10   PB6  IN    node-2 / sensor line B     [fm]
 *  PA6  OUT L status LED half 2 ? (pair w/ PB13)  PB7  IN    node-2 / sensor line A     [fm]
 *  PA7  AN   ADC1 ch7  -> current sense of PB0    PB8  AF    CAN1 RX (remap, pull-up)  [can]
 *  PA8  AF   TMR1_CH1  -> buzzer PWM     [buzz]   PB9  AF    CAN1 TX (remap)           [can]
 *  PA9  IN   pull-up   sensor-fault 1      [fm]   PB10 OUT H subsys A enable (act-low) [ctl]
 *  PA10 IN   pull-up (boot) / floating (standby)  PB11 IN    node-4 comms line          [fm]
 *            +EXINT10 wake: charger/dock ?        PB12 OUT H power latch              [gpio]
 *  PA11 IN   node-1 comms line           [fm]     PB13 OUT L status LED half 1 ? (w/ PA6)
 *  PA12 IN   node-3 comms line           [fm]     PB14 OUT L indicator, own flag
 *  PA13 SWDIO  (debug)                            PB15 OUT L CAN xcvr standby ?     [can]
 *  PA14 SWCLK  (debug)                                       (low right after CAN init)
 *  PA15 OUT H SPI2 CS, device A (never asserted; SPI flash PY25Q64 ?)
 *
 *  GPIOC                                          GPIOD / GPIOF
 *  -----------------------------------------      -----------------------------------------
 *  PC13 not configured at boot; standby only:     PD0  OSC_IN   HEXT 16 MHz crystal [clock]
 *       IN pull-up +EXINT13 wake: power btn ?     PD1  OSC_OUT
 *       (routed only when IN2 status bits say so) PF6  OUT L aux output             [ctl]
 *  PC14 OUT H fault / drive-disable latch ?       PF7  OUT H SPI2 CS, IMU (active low) [sens]
 *  PC15 (unused)
 *
 *  Peripheral view
 *  ---------------
 *           +---------------------------+
 *  PA2/PA3  | USART2 ---- dashboard/BLE |       (Ninebot 5A A5 frames, DMA1)
 *  PB8/PB9  | CAN1   ---- ESC / BMS     |------ PB15 transceiver standby ?
 *  PB3..PB5 | SPI2  -+--- IMU LSM6DS3   |------ CS PF7  (WHO_AM_I 0x69 / 0x6A)
 *           |        +--- device A ?    |------ CS PA15 (deselected forever)
 *  PA8      | TMR1 ch1 -> buzzer        |
 *  PA0..PA7 | ADC1 (DMA): throttle, 2 brake levers, current sense of PB10 / PB0
 *           +---------------------------+
 * \endverbatim
 *
 * PA1 and PA4 are the two brake levers (left and right). Which hand is which is not in
 * the code; the brake output is max(PA4 %, PA1 % / 2), so PA4 is the full-scale lever
 * and PA1 contributes at most half. A bench test settles the side (ADC DMA buffer +2 is
 * PA1, +4 is PA4). PA5 / PA7 are sampled only while PB10 / PB0 is driven low, so each is
 * the current sense of the output it pairs with. PB13 and PA6 are always driven as a
 * pair ((0,0), (1,1), (0,1)) and read back together, which looks like a two-colour LED.
 *
 * PA3 is the USART2 RX pin (plain pull-up input at boot); EXINT3 is armed on it only by
 * the standby path, so that wake fires on incoming serial traffic, not on a button.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "at32f415.h"

/*
 * Pin configuration field values for \ref vcu_gpio_config: the 4-bit cfglr / cfghr field of a
 * pin, `iofc << 2 | iomc`. These are the values gpio_init writes for the BSP settings named.
 */
#define VCU_GPIO_CFG_ANALOG   0x0u /*!< analog input                                     */
#define VCU_GPIO_CFG_IN_FLOAT 0x4u /*!< input, no pull (GPIO_PULL_NONE)                   */
#define VCU_GPIO_CFG_IN_PU    0x8u /*!< input with pull-up (GPIO_PULL_UP: also sets odt)  */
#define VCU_GPIO_CFG_OUT_PP   0x1u /*!< output, push-pull, drive strength stronger        */
#define VCU_GPIO_CFG_MUX_PP   0x9u /*!< alternate function, push-pull, drive stronger     */

/**
 * \brief           Latched EXINT event codes (the values the ISRs write to `g_vs.wake_src`)
 * \note            Each line latches a distinct code, so the vehicle FSM can tell PA10 from
 *                      PC13 even though they share IRQ 40.
 */
typedef enum {
    VCU_GPIO_EVENT_NONE = 0, /*!< no event pending */
    VCU_GPIO_EVENT_PC13 = 1, /*!< EXINT13 / PC13 fired (likely power/wake button) */
    VCU_GPIO_EVENT_PA10 = 2, /*!< EXINT10 / PA10 fired (likely charger/dock detect) */
    VCU_GPIO_EVENT_PA3 = 4   /*!< EXINT3  / PA3  fired (serial traffic on USART2 RX) */
} vcu_gpio_event_t;

/**
 * \brief           Set the 4-bit config field of a group of pins
 * \param[in]       port: GPIO port
 * \param[in]       pins: pin mask within the port
 * \param[in]       cfg: field value, one of the VCU_GPIO_CFG_* values
 * \note            Read-modify-writes the 4-bit field of every pin in `pins`; the other pins keep
 *                      their configuration. A pull-up input first sets the pins' odt bits (scr).
 */
void vcu_gpio_config(gpio_type* port, uint32_t pins, uint32_t cfg);

/**
 * \brief           Arm EXINT lines as edge-triggered interrupts
 * \param[in]       lines: EXINT line mask
 * \param[in]       rising: 1 = rising edge, 0 = falling edge
 * \note            Enables the lines in inten (and clears evten), then sets the one edge in
 *                      polcfg1 / polcfg2. Pending flags are left alone.
 */
void vcu_exint_arm(uint32_t lines, uint32_t rising);

/**
 * \brief           PA10 (EXINT10 wake line) level test, active-low
 * \return          `1` when PA10 reads LOW, `0` when HIGH
 */
int32_t vcu_gpio_pa10_active(void);

/**
 * \brief           Whether this board revision populates the PC13 wake input
 * \return          Non-zero when EXINT13 / PC13 should be routed and armed
 * \note            Line 13 is routed only when bit15 or bit2 of g_in2.f.cfg_flags (IN2 +0x3A)
 *                      is set.
 */
uint8_t vcu_board_pc13_wake_present(void);

/**
 * \brief           PB12 power-latch pad as a push-pull output
 * \note            Configures the pin only; vcu_periph_init drives it high.
 */
void vcu_gpio_power_latch_init(void);

/**
 * \brief           PB13 / PA6 / PB14 indicator outputs, push-pull
 * \note            Configures the pins only; vcu_periph_init drives them low.
 */
void vcu_gpio_indicator_outputs_init(void);

/**
 * \brief           Arm EXINT3 / PA3 as a falling-edge wake input
 * \note            Also called standalone by the low-power standby orchestrator to
 *                      re-arm the PA3 wake line before deep sleep. Configures the pin
 *                      (pull-up), routes EXINT line 3, enables NVIC IRQ9 and clears any
 *                      pending edge. Idempotent.
 */
void vcu_gpio_exint3_init(void);

/**
 * \brief           Arm EXINT10 / PA10 and (HW-variant) EXINT13 / PC13
 * \note            Also called standalone by the low-power standby orchestrator to
 *                      re-arm the PA10/PC13 wake group (shared NVIC IRQ40) before deep
 *                      sleep. PC13 routing/arming is gated on \ref vcu_board_pc13_wake_present.
 */
void vcu_gpio_exint_10_13_init(void);

/**
 * \brief           PC14 power/fault-latch pad as an output
 * \param[in]       mode: output type (every caller passes 0 = push-pull)
 * \note            Shared by the boot bring-up and the vehicle FSM state-4 fault latch. Only
 *                      (re)configures the pin; it does not drive a level.
 */
void vcu_gpio_fault_latch_init(uint8_t mode);

#endif /* GPIO_H */
