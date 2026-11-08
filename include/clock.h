/**
 * \file            clock.h
 * \brief           System clock bring-up
 */
#ifndef CLOCK_H
#define CLOCK_H

/** \brief Bring up HEXT -> PLL x9 -> 144 MHz SYSCLK. */
void clock_config(void);

/** \brief Re-lock HEXT + PLL and switch SYSCLK back after deep sleep. */
void clock_resume(void);

/**
 * \brief           Restore a saved CRM clock snapshot
 * \note            Called from the vehicle FSM state-5 (power-off) handler. Writes the
 *                      clock-config / clock-control words saved by vcu_standby_cycle back
 *                      into CRM when the snapshot is valid and the live registers diverge
 *                      (or USART2->sts bit2 is set).
 */
void clock_restore(void);

#endif /* CLOCK_H */
