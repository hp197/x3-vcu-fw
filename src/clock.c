/**
 * \file            clock.c
 * \brief           Bring up the system clock: `HEXT` -> `PLL` -> 144 MHz
 * \note            Re-establishes HEXT + PLL after `SystemInit` has reset the tree to HICK.
 *                      Same 16 MHz crystal as the bootloader. PLL fed from HEXT (no /2) x 9 =
 *                      144 MHz, as crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_9) programs
 *                      it. (HEXT/2 x 18 gives the same SYSCLK but is not used.)
 */
#include "clock.h"

#include "at32f415.h"
#include "board.h" /* SYSCLK_HZ */
#include "vcu_state.h" /* g_vs.crm_cfg_save / crm_ctrl_save (clock snapshot) */

/*
 * Clock tree:
 *
 *   HEXT 16 MHz --> PLL (x9) --> SYSCLK 144 MHz --+--> AHB  /1 --> HCLK  144 MHz
 *                                                 +--> APB2 /2 --> PCLK2  72 MHz
 *                                                 +--> APB1 /2 --> PCLK1  72 MHz
 *                                                                  (timers x2 = 144 MHz)
 *
 * The PLL is fed from HEXT directly (no /2 predivider): 16 MHz x 9 = 144 MHz.
 * At 144 MHz the flash needs 4 wait states.
 */
#define CLOCK_PLL_SOURCE CRM_PLL_SOURCE_HEXT /* PLL input = 16 MHz HEXT, no predivide */
#define CLOCK_PLL_MULT   CRM_PLL_MULT_9      /* x9: 16 MHz x 9 = 144 MHz */
#define CLOCK_PLL_FREF   CRM_PLL_FREF_16M    /* PLL reference range of the 16 MHz input */
#define CLOCK_FLASH_WAIT FLASH_WAIT_CYCLE_4  /* 4 wait states, required at 144 MHz */
#define CLOCK_AHB_DIV    CRM_AHB_DIV_1       /* HCLK  = SYSCLK / 1 = 144 MHz */
#define CLOCK_APB2_DIV   CRM_APB2_DIV_2      /* PCLK2 = HCLK   / 2 =  72 MHz */
#define CLOCK_APB1_DIV   CRM_APB1_DIV_2      /* PCLK1 = HCLK   / 2 =  72 MHz */

/* crm_reset values: ctrl hexten / hextbyps / cfden / pllen cleared, pll / misc1 reset values,
   clkint: every interrupt off, every pending flag cleared. */
#define CLOCK_CTRL_RESET_MASK 0x010D0000u
#define CLOCK_PLL_RESET       0x00001F10u
#define CLOCK_MISC1_RESET     0x00100000u
#define CLOCK_CLKINT_RESET    0x009F0000u

/* CRM cfg bus divider fields: ahbdiv [7:4], apb1div [10:8], apb2div [13:11]. */
#define CLOCK_DIV_CFGMSK 0x3FF0u
#define CLOCK_DIV_CFGVAL                                                                                               \
    (((uint32_t)CLOCK_AHB_DIV << 4) | ((uint32_t)CLOCK_APB1_DIV << 8) | ((uint32_t)CLOCK_APB2_DIV << 11))

/* CRM cfg PLL fields: pllrcs [16], pllhextdiv [17], pllmult_l [21:18], pllmult_h [30:29]. */
#define CLOCK_PLL_CFGMSK 0x603F0000u
/* pllrcs = 1 (HEXT), pllhextdiv = 0 (no /2), pllmult = 7 (x9: low 4 bits, high bits 0). */
#define CLOCK_PLL_CFGVAL (((uint32_t)CLOCK_PLL_SOURCE << 16) | ((uint32_t)CLOCK_PLL_MULT << 18))

/**
 * \brief           Wait for HEXT to report stable, for at most HEXT_STARTUP_TIMEOUT polls
 * \return          1 when HEXT is stable, else 0 (crm_hext_stable_wait)
 */
static uint32_t prv_hext_stable_wait(void) {
    uint32_t polls = 0u;

    while ((CRM->ctrl_bit.hextstbl == 0u) && (polls < HEXT_STARTUP_TIMEOUT)) {
        polls++;
    }
    return CRM->ctrl_bit.hextstbl;
}

/** \brief Start HEXT and wait for the crystal to stabilise. */
static void clock_hext_start(void) {
    CRM->ctrl_bit.hexten = TRUE;
    while (prv_hext_stable_wait() == 0u) {}
}

/**
 * \brief           Program the PLL source, multiplier and reference range (PLL off)
 * \note            The values crm_pll_config(CRM_PLL_SOURCE_HEXT, CRM_PLL_MULT_9) writes for the
 *                      16 MHz HEXT: cfg pllrcs = 1, pllhextdiv = 0, pllmult = 7, pll pllfref = 16M.
 */
static void prv_pll_config(void) {
    CRM->cfg = (CRM->cfg & ~CLOCK_PLL_CFGMSK) | CLOCK_PLL_CFGVAL;
    CRM->pll_bit.pllfref = CLOCK_PLL_FREF;
}

/** \brief Enable the PLL (multiplier already programmed) and wait for lock. */
static void clock_pll_start(void) {
    CRM->ctrl_bit.pllen = TRUE;
    while (CRM->ctrl_bit.pllstbl == 0u) {}
}

/** \brief Switch SYSCLK onto the PLL; auto-step ramps it to stay within the flash timing. */
static void clock_sysclk_to_pll(void) {
    CRM->misc2_bit.auto_step_en = CRM_AUTO_STEP_MODE_ENABLE;
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (CRM->cfg_bit.sclksts != CRM_SCLK_PLL) {}
    CRM->misc2_bit.auto_step_en = CRM_AUTO_STEP_MODE_DISABLE;
}

/**
 * \brief           Configure the system clock to 144 MHz from the 16 MHz crystal
 */
void clock_config(void) {
    /* crm_reset: HICK on and SYSCLK back on it, then HEXT / CFD / PLL off and cfg, pll,
       misc1 and clkint at their reset values. */
    CRM->ctrl_bit.hicken = TRUE;
    while (CRM->ctrl_bit.hickstbl == 0u) {}
    CRM->cfg_bit.sclksel = CRM_SCLK_HICK;
    while (CRM->cfg_bit.sclksts != CRM_SCLK_HICK) {}
    CRM->ctrl &= ~CLOCK_CTRL_RESET_MASK;
    CRM->cfg = 0u;
    CRM->pll = CLOCK_PLL_RESET;
    CRM->misc1 = CLOCK_MISC1_RESET;
    CRM->clkint = CLOCK_CLKINT_RESET;
    flash_psr_set(CLOCK_FLASH_WAIT);

    clock_hext_start();

    /* Lock the PLL on HEXT x 9. */
    prv_pll_config();
    clock_pll_start();

    /* Program the bus dividers before switching SYSCLK onto the PLL (the fields
       crm_ahb_div_set / crm_apb2_div_set / crm_apb1_div_set write). */
    CRM->cfg = (CRM->cfg & ~CLOCK_DIV_CFGMSK) | CLOCK_DIV_CFGVAL;

    clock_sysclk_to_pll();

    system_core_clock = SYSCLK_HZ; /* the clock tree is fixed: no need to read it back */
}

/** \brief Failed polls of the PLL-stable flag or the SYSCLK switch before clock_resume retries. */
#define CLOCK_RESUME_POLLS 0x3000u

/**
 * \brief           Restore the 144 MHz clock after deep sleep
 * \note            Deep sleep drops SYSCLK to HICK and stops HEXT and the PLL. This restarts
 *                      HEXT, re-programs and re-locks the PLL and switches SYSCLK back; the
 *                      flash wait states and the bus dividers survive in CRM->cfg / FLASH. Every
 *                      wait is bounded and retried:
 *                      - HEXT: enable + a bounded stable wait until it reports stable;
 *                      - PLL: after 0x3000 failed polls, program and enable it again;
 *                      - SYSCLK: after 0x3000 failed polls, request the switch again.
 *                      `system_core_clock` keeps its value. Only caller:
 *                      vcu_standby_cycle, after the WFI wake.
 */
void clock_resume(void) {
    uint32_t polls = 0u;

    do {
        CRM->ctrl_bit.hexten = TRUE;
    } while (prv_hext_stable_wait() == 0u);

    prv_pll_config();
    CRM->ctrl_bit.pllen = TRUE;
    while (CRM->ctrl_bit.pllstbl == 0u) {
        if (polls < CLOCK_RESUME_POLLS) {
            ++polls;
        } else {
            polls = 0u;
            prv_pll_config();
            CRM->ctrl_bit.pllen = TRUE;
        }
    }

    CRM->misc2_bit.auto_step_en = CRM_AUTO_STEP_MODE_ENABLE;
    polls = 0u;
    crm_sysclk_switch(CRM_SCLK_PLL);
    while (CRM->cfg_bit.sclksts != CRM_SCLK_PLL) {
        if (polls < CLOCK_RESUME_POLLS) {
            ++polls;
        } else {
            polls = 0u;
            crm_sysclk_switch(CRM_SCLK_PLL);
        }
    }
    CRM->misc2_bit.auto_step_en = CRM_AUTO_STEP_MODE_DISABLE;
}

/* Clock save/restore across power-down. clock_restore (below) and the standby-time save
   (config.c vcu_standby_cycle) use the CRM->ctrl / CRM->cfg pair at 0x40021000 and poll
   USART2->sts. The AT32F415 has no backup domain, so the snapshot lives in g_vs. */

/* FSM state-5 (power-off) restore: with a valid snapshot (crm_cfg_save and crm_ctrl_save both
   non-zero), write the saved clock-config/control words back into CRM->cfg / CRM->ctrl when the
   live registers no longer match it, or USART2->sts bit2 is set. */
void clock_restore(void) {
    uint32_t cfg_save = g_vs.crm_cfg_save;
    uint32_t ctrl_save = g_vs.crm_ctrl_save;

    if (cfg_save != 0 && ctrl_save != 0
        && (((int32_t)(USART2->sts << 0x1d) < 0) || CRM->cfg != cfg_save || CRM->ctrl != ctrl_save)) {
        CRM->cfg = cfg_save;
        CRM->ctrl = ctrl_save;
    }
}
