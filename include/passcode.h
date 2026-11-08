/**
 * \file            passcode.h
 * \brief           Passcode: set (app + confirm on the handlebar) and unlock (key-sequence /
 *                      numeric), with the 3-try lock-out (x3 VCU)
 * \note            Three ticks make up this module:
 *                        passcode_set_confirm_tick  (confirm a new key passcode)
 *                        passcode_key_unlock_tick   (key-sequence unlock, state 2)
 *                        passcode_digit_unlock_tick (numeric unlock, state 2)
 *
 *                      The rider's lock passcode comes in two kinds, picked by the settings
 *                      register (reg 0x1D):
 *                        - key sequence (CFG_PASSCODE_KEY): a run of handlebar key presses
 *                          (g_evt records PA4/PA12/PA1/PB1, code 4 -> keys 1..4) appended to
 *                          the entry buffer (VS+0x47, cursor VS+0x52), packed as nibbles
 *                          (passcode_pack_keys, digit + 4 per nibble) and compared with the
 *                          stored code (reg 0x71/0x72, IN2+0xE2);
 *                        - numeric (CFG_PASSCODE_NUM): four digits 0..9 scrolled with the
 *                          up/down keys, the cursor moved with prev/next, the value
 *                          d0*1000+d1*100+d2*10+d3 compared with reg 0x61 (IN2+0xC2).
 *                      Setting a key passcode: an app write to reg 0x71 is staged in VS+0x56
 *                      and raises VS+0x41; in the ride state the rider then repeats the code
 *                      on the handlebar within 3000 ticks. A match beeps, reports status 1
 *                      (cmd 3 group 0x73 to 0x3E) and commits the code to IN2+0xE2 (persisted
 *                      through IN1+0x10); a mismatch reports status 2; reg 0x1E b6 cancels.
 *                      Unlocking (state 2): a match returns 1 (unlocked). Each mismatch counts
 *                      in VS+0x46; the third one arms the lock-out (VS+0x42 = 1) and shuts the
 *                      scooter down after 300 ticks (VS+0x1C). A further failure while the
 *                      lock-out is armed sets VS+0x42 = 2: anti-theft alarm (state 6).
 *                      Progress goes to the dashboard as cmd 3 groups 0x62 (status word),
 *                      0x63 (numeric display) and 0x64, and 0xD3 (packed keys).
 *
 *                      The bodies reach the shared blackboards through vcu_state.h. The
 *                      private control block is typed here as \ref passcode_ch_t.
 *                      Cross-module callees are reached through their public owners
 *                      (vcu_usart_report_send, nb_commit_mode, buzzer_beep, passcode_digit_count,
 *                      the passcode_* / vcu_faultmon_* packers). These three ticks are called
 *                      from the vehicle FSM.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#ifndef PASSCODE_H
#define PASSCODE_H

#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Module-specific overlay: the passcode control block.                      */
/*                                                                           */
/* The three routines share ONE small control block in low SRAM. Each owns a */
/* disjoint set of fields (set/confirm = +0x00/+0x01/+0x0A..+0x0D, key unlock */
/* = +0x06..+0x09, numeric unlock = +0x02..+0x05); they SHARE the packed-keys */
/* word at +0x10 (written by passcode_pack_keys) and the per-try status bits  */
/* at +0x14.                                                                 */
/* ------------------------------------------------------------------------- */

/** \brief Shared passcode control block (g_passcode_ch). */
typedef struct {
    uint8_t active;       /*!< +0x00 set/confirm: entry started                             */
    uint8_t rotor_a;      /*!< +0x01 set/confirm report rotor 0..0x0E                       */
    uint16_t rotor_menu;  /*!< +0x02 numeric-unlock report rotor 0..0x0E                    */
    uint16_t timer_menu;  /*!< +0x04 numeric-unlock retry / lock-out timer                  */
    uint16_t timer_sib;   /*!< +0x06 key-unlock retry / lock-out timer                      */
    uint16_t rotor_sib;   /*!< +0x08 key-unlock report rotor 0..0x0E                        */
    uint16_t hs_timeout;  /*!< +0x0A set/confirm 3000-tick window counter                   */
    uint16_t rpt_timeout; /*!< +0x0C set/confirm start delay                                */
    uint8_t _rsv0e[2];    /*!< +0x0E..+0x0F                                                 */
    uint32_t disp_word;   /*!< +0x10 packed keys (passcode_pack_keys) / 0xD3 report payload */
    uint16_t reload[3];   /*!< +0x14 per-try status bits OR-ed into SCR+0xC4, initialised to {0x30, 0x20, 0x10} */
} passcode_ch_t;

/* g_passcode_ch is a typed global (0x1A bytes, defined in vcu_state.c). disp_word @+0x10 is a
   shared field: passcode_pack_keys writes it, the passcode reports send it. */
extern volatile passcode_ch_t g_passcode_ch;

/* ------------------------------------------------------------------------- */
/* Enums + named constants.                                                  */
/* ------------------------------------------------------------------------- */

/* Report rotor emission points (shared by all three routines). */
#define PASSCODE_ROTOR_STEP_A  0u  /*!< emit the packed-keys (0xD3) or numeric display (99) report */
#define PASSCODE_ROTOR_STEP_B  5u  /*!< emit the group-100 report                               */
#define PASSCODE_ROTOR_STEP_C  10u /*!< emit the status-word report (0x62)                      */
#define PASSCODE_ROTOR_WRAP    14u /*!< rotor wraps back to 0 after this value                  */
#define PASSCODE_ROTOR_LIMIT   15u /*!< rotor runs only while < this                            */

/* Timeouts / limits. */
#define PASSCODE_SET_TIMEOUT   3000u /*!< ticks to confirm a new passcode on the handlebar        */
#define PASSCODE_SET_DELAY     10u   /*!< ticks to hold before the confirm entry starts (>10)     */
#define PASSCODE_TRIES         3u    /*!< wrong entries before the lock-out (tries < 3)           */
#define PASSCODE_DIGIT_MIN     0     /*!< numeric passcode: digit value lower bound               */
#define PASSCODE_DIGIT_MAX     9     /*!< numeric passcode: digit value upper bound               */

/* Per-try timer reloads. */
#define PASSCODE_LOCKOUT_GRACE 20u   /*!< reload on a wrong entry while the lock-out is armed (-> alarm) */
#define PASSCODE_RETRY_TICKS   200u  /*!< reload between tries                                    */
#define PASSCODE_LOCKOUT_TICKS 300u  /*!< reload on the third wrong entry (-> shutdown)           */

/* Ninebot report frame fields. The reports go src NB_ADDR_VCU, cmd
   NB_CMD_WRITE3 (usart.h) to dst NB_ADDR_APP, or to PASSCODE_NB_DST_STATUS. */
#define PASSCODE_NB_DST_STATUS 0x3Eu /*!< dst of the passcode-set result report                   */
#define PASSCODE_ARG_KEYS      0xD3u /*!< arg: packed entered keys (len 4)                        */
#define PASSCODE_ARG_STATUS    0x62u /*!< arg: entry status word, SCR+0xC4 (len 2)                */
#define PASSCODE_ARG_VALUE     100u  /*!< arg: group 0x64 value (len 2)                           */
#define PASSCODE_ARG_DISPLAY   99u   /*!< arg: numeric display word, SCR+0xC6 (len 2)             */
#define PASSCODE_ARG_RESULT    0x73u /*!< arg: passcode-set result, 1 = match, 2 = mismatch (len 1) */

/* ------------------------------------------------------------------------- */
/* Public ticks (callees of the vehicle FSM).                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Confirm a new key passcode on the handlebar
 * \note            Runs only while a passcode change is pending (VS->passcode_set_pending,
 *                      raised by an app write to reg 0x71). Outside the ride state, or after
 *                      3000 ticks, it gives up (status 2); reg 0x1E b6 cancels. Once started
 *                      it sizes the entry from the staged code (passcode_digit_count of
 *                      VS+0x56), shows the passcode screen, appends each handlebar key (1..4)
 *                      to the entry buffer and streams the progress reports. A complete entry
 *                      is packed and compared with the staged code: a match commits it to
 *                      IN2+0xE2 (reg 0x71/0x72) and reports status 1, a mismatch reports
 *                      status 2.
 */
void passcode_set_confirm_tick(void);

/**
 * \brief           Key-sequence passcode unlock (state 2)
 * \note            Appends each handlebar key (1..4) to the entry buffer; a complete entry
 *                      is packed and compared with the stored key passcode (IN2+0xE2). A
 *                      wrong entry counts a try and restarts the entry after 200 ticks; while
 *                      the lock-out is armed (VS+0x42 == 1) it goes to the alarm (VS+0x42 = 2)
 *                      instead; the third wrong entry arms the lock-out and requests a
 *                      shutdown (VS+0x1C) after 300 ticks. A report rotor streams the
 *                      progress frames while idle.
 * \return          1 when the code matched (unlocked), 0 while in progress.
 */
uint8_t passcode_key_unlock_tick(void);

/**
 * \brief           Numeric passcode unlock (state 2)
 * \note            Up/down keys scroll the digit under the cursor 0..9 (with wrap),
 *                      prev/next move the cursor, select advances to the next digit; after
 *                      the last digit the value d0*1000+d1*100+d2*10+d3
 *                      (passcode_bcd_to_decimal) is compared with reg 0x61 (IN2+0xC2),
 *                      with the same tries / lock-out logic as the key-sequence unlock. A
 *                      report rotor streams the display / status frames while idle.
 * \return          1 when the code matched (unlocked), 0 while in progress.
 */
uint8_t passcode_digit_unlock_tick(void);

/* ------------------------------------------------------------------------- */
/* Passcode entry helpers over the entry buffer VS+0x47.                      */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Passcode length: number of non-zero nibbles in \p codes
 * \param[in]       codes: packed passcode, one nibble per key / digit
 * \return          0..8
 */
int8_t passcode_digit_count(uint32_t codes);

/**
 * \brief           Pack the 8-entry passcode key buffer into nibble words
 * \note            Packs digits[7..0] low-to-high into a 32-bit word stored at the
 *                      packed-keys cell (g_passcode_ch.disp_word, the 0xD3 report payload),
 *                      then returns a second word in which each key (non-zero entry) is
 *                      offset by 4 and packed MSB-first: the form the key passcode is stored
 *                      and compared in.
 * \param[in]       digits: pointer to the 8 entry bytes
 * \return          Packed offset-by-4 nibble word of the entered keys
 */
uint32_t passcode_pack_keys(const uint8_t* digits);

/**
 * \brief           Build the 4-digit numeric passcode display word
 * \note            Blanks (0x0B) the digits past the active count (@g_vs+0x54), copies
 *                      the selected digit (@g_vs+0x52) from `digits`, dashes (0x0A) the rest
 *                      up to the count, and packs the four nibbles; if the raw-mode flag bit
 *                      (@g_scr+0xC4 bit2) is set, returns the raw 4-nibble value of
 *                      digits[0..3] instead.
 * \param[in]       digits: pointer to the 4 entered digits (bytes)
 * \return          16-bit packed 4-digit display word
 */
uint16_t passcode_display_word(const uint8_t* digits);

/**
 * \brief           Convert the 4 entered passcode digits to their decimal value
 * \note            Computes ((d0*10 + d1)*10 + d2)*10 + d3.
 * \param[in]       digits: pointer to 4 BCD digits (bytes)
 * \return          Decimal value as a signed 16-bit integer
 */
int16_t passcode_bcd_to_decimal(const uint8_t* digits);

/**
 * \brief           Reset the passcode entry and send its report frames
 * \note            Clears the passcode entry buffer, cursor, digit and count, then -- per
 *                      the passcode kind in reg 0x1D -- sets the length to 4 (numeric) or to
 *                      the stored key passcode's length (key sequence, plus the packed 0xD3
 *                      report frame); always clears the status word SCR+0xC4 and enqueues
 *                      the 0x62 report frame through the report framer.
 */
void passcode_entry_reset(void);

#endif /* PASSCODE_H */
