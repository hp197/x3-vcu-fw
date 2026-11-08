/**
 * \file            passcode.c
 * \brief           Passcode: set (app + confirm on the handlebar), key-sequence and numeric
 *                      unlock, 3-try lock-out (x3 VCU)
 * \note            The passcode ticks run by the vehicle FSM:
 *                        passcode_set_confirm_tick  (confirm a new key passcode)
 *                        passcode_key_unlock_tick   (key-sequence unlock)
 *                        passcode_digit_unlock_tick (numeric unlock)
 *                      and the entry helpers they share: passcode_digit_count,
 *                      passcode_pack_keys, passcode_display_word, passcode_bcd_to_decimal and
 *                      passcode_entry_reset. See passcode.h for how the feature works.
 *
 *                      The entry state lives in the vehicle-state block (vcu_state.h:
 *                      VS+0x45 code_len, +0x46 code_retries, +0x47 code_entry[], +0x50
 *                      code_digit, +0x52 code_cursor, +0x54 code_count, +0x56 passcode_new,
 *                      +0x42 lockout_phase); the private control block is \ref passcode_ch_t
 *                      (passcode.h). Cross-module callees go through their public owners.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 *                      The Artery AT32F415 BSP + CMSIS keep their own licenses.
 */
#include "passcode.h"
#include "buzzer.h"   /* buzzer_beep                                                    */
#include "control.h"  /* control_countdown_beep                                         */
#include "faultmon.h" /* the fm_* / vcu_faultmon_sensor_debounce_* ticks run by the unlocks */
#include "ninebot.h"  /* nb_commit_mode                                                 */
#include "usart.h"    /* vcu_usart_report_send                                          */
#include "vcu_state.h"

/* The staged new key passcode (VS+0x56, two halfword slots) as one 32-bit value. */
#define PASSCODE_NEW32 ((uint32_t)g_vs.passcode_new[0] | ((uint32_t)g_vs.passcode_new[1] << 16))
/* The entry buffer (VS+0x47, 8 bytes: one key 1..4 or digit 0..9 per byte, 8 = end mark). */
#define PASSCODE_ENTRY ((const uint8_t*)g_vs.code_entry)
/* g_scr+0xC8 == countdown mirror cell; the set/confirm routine reports it as group 100. */

/* ------------------------------------------------------------------------- */
/* Report frames to the app.                                                 */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Enqueue one passcode report frame (VCU -> app, cmd write-3)
 * \param[in]       len: payload length
 * \param[in]       arg: report argument (PASSCODE_ARG_*)
 * \param[in]       payload: \p len payload bytes
 */
static void prv_report_app(uint8_t len, uint8_t arg, const volatile void* payload) {
    vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, NB_ADDR_APP, len, NB_CMD_WRITE3, arg, payload);
}

/**
 * \brief           Report the entry status word SCR+0xC4 (arg 0x62)
 */
static void prv_report_status(void) { prv_report_app(2u, PASSCODE_ARG_STATUS, &g_scr.passcode_status); }

/**
 * \brief           Pack the entered keys into +0x10 and report them (arg 0xD3)
 */
static void prv_report_keys(void) {
    (void)passcode_pack_keys(PASSCODE_ENTRY);
    prv_report_app(4u, PASSCODE_ARG_KEYS, &g_passcode_ch.disp_word);
}

/**
 * \brief           Build the numeric display word SCR+0xC6 and report it (arg 99)
 */
static void prv_report_display(void) {
    g_scr.passcode_display = passcode_display_word(PASSCODE_ENTRY);
    prv_report_app(2u, PASSCODE_ARG_DISPLAY, &g_scr.passcode_display);
}

/**
 * \brief           Report the passcode-set result (arg 0x73, one byte)
 * \param[in]       result: 1 = match, 2 = mismatch / given up
 */
static void prv_report_result(uint32_t result) {
    uint32_t payload = result;

    vcu_usart_report_send(((volatile uint8_t*)&g_uartctx), NB_ADDR_VCU, PASSCODE_NB_DST_STATUS, 1, NB_CMD_WRITE3,
                          PASSCODE_ARG_RESULT, &payload);
}

/**
 * \brief           Send the report of one report-rotor step
 * \param[in]       step: rotor value
 * \param[in]       entry_report: report of step 0 (the entry)
 * \param[in]       value: group-100 payload of step 5
 * \param[in]       status_report: report of step 10 (the status word)
 * \note            The other steps send nothing.
 */
static void prv_rotor_report(uint32_t step, void (*entry_report)(void), const volatile void* value,
                             void (*status_report)(void)) {
    if (step == PASSCODE_ROTOR_STEP_A) {
        entry_report();
    } else if (step == PASSCODE_ROTOR_STEP_B) {
        prv_report_app(2u, PASSCODE_ARG_VALUE, value);
    } else if (step == PASSCODE_ROTOR_STEP_C) {
        status_report();
    }
}

/* ------------------------------------------------------------------------- */
/* Shared helper: append one key to the entry buffer, advancing the cursor and     */
/* laying down the 8 end mark (used by the confirm entry and the key unlock).      */
/* ------------------------------------------------------------------------- */
static inline void passcode_entry_push(uint8_t code) {
    uint16_t cursor = g_vs.code_cursor;

    g_vs.code_entry[cursor] = code;
    cursor = (uint16_t)(cursor + 1);
    g_vs.code_cursor = cursor;
    g_vs.code_entry[cursor] = 8;
}

/* One handlebar key of a key-sequence entry: input monitor, event code that enters it */
/* (0 = any pending code), key value 1..4.                                             */
#define PRV_KEY_MON  0u
#define PRV_KEY_CODE 1u
#define PRV_KEY_VAL  2u

/** \brief Keys of the confirm entry, in priority order (the park key takes any pending code). */
static const uint8_t prv_confirm_keys[4][3] = {
    {FM_MON_BRAKE_FULL, 4u, 1u}, {FM_MON_INTERLOCK, 4u, 2u}, {FM_MON_BRAKE_HALF, 4u, 3u}, {FM_MON_PARK, 0u, 4u}};

/** \brief Keys of the key-sequence unlock, in priority order. */
static const uint8_t prv_unlock_keys[4][3] = {
    {FM_MON_INTERLOCK, 4u, 2u}, {FM_MON_PARK, 4u, 4u}, {FM_MON_BRAKE_FULL, 4u, 1u}, {FM_MON_BRAKE_HALF, 4u, 3u}};

/**
 * \brief           Append the first pressed handlebar key of `keys` to the entry buffer
 * \param[in]       keys: four {monitor, code, key} rows in priority order
 * \note            The pressed key beeps (short) and its event code is cleared; at most one
 *                      key is taken per tick.
 */
static void prv_key_entry(const uint8_t (*keys)[3]) {
    uint32_t idx;
    uint8_t code;
    uint8_t done = 0u;

    for (idx = 0u; (idx < 4u) && (done == 0u); ++idx) {
        code = g_evt.mon[keys[idx][PRV_KEY_MON]].code;
        if ((code != 0u) && ((keys[idx][PRV_KEY_CODE] == 0u) || (code == keys[idx][PRV_KEY_CODE]))) {
            buzzer_beep(BEEP_SHORT);
            g_evt.mon[keys[idx][PRV_KEY_MON]].code = 0u;
            passcode_entry_push(keys[idx][PRV_KEY_VAL]);
            done = 1u;
        }
    }
}

/**
 * \brief           Clear the passcode-change cancel bit (reg 0x1E b6)
 */
static void prv_cancel_clear(void) { g_in2.f.cfg_flags2 &= (uint16_t)~CFG2_PASSCODE_CANCEL; }

/* ========================================================================= */
/* Confirm a new key passcode on the handlebar.                               */
/* ========================================================================= */
void passcode_set_confirm_tick(void) {
    uint16_t prev;
    uint8_t step;
    uint32_t staged;

    /* no passcode change pending: clear the confirm state and the cancel bit, done. */
    if (g_vs.passcode_set_pending == 0) {
        g_passcode_ch.active = 0;
        g_passcode_ch.hs_timeout = 0; /* clears +0x0A/+0x0B */
        prv_cancel_clear();
        return;
    }

    /* pending: advance the 3000-tick confirm window (ride state only); b6 cancels. */
    prev = g_passcode_ch.hs_timeout;
    g_passcode_ch.hs_timeout = (uint16_t)(prev + 1);
    if (prev < PASSCODE_SET_TIMEOUT && g_state_block.fsm_state == VCU_FSM_RIDE) {
        if ((g_in2.f.cfg_flags2 & CFG2_PASSCODE_CANCEL) != 0) {
            passcode_entry_reset();
            prv_cancel_clear();
            g_vs.passcode_set_pending = 0;
            nb_commit_mode(1);
        }
    } else {
        /* window expired (or not in the ride state): give up, report status 2. */
        passcode_entry_reset();
        buzzer_beep(BEEP_LONG);
        prv_cancel_clear();
        g_vs.passcode_set_pending = 0;
        prv_report_result(2u);
        prev = g_passcode_ch.hs_timeout;
        g_passcode_ch.hs_timeout = (uint16_t)(prev + 1);
        if (prev >= PASSCODE_SET_TIMEOUT) {
            nb_commit_mode(1);
        }
    }

    if (g_passcode_ch.active == 0 && g_vs.passcode_set_pending != 0) {
        /* not yet started: hold for a few ticks, then size the entry from the staged code. */
        if (g_passcode_ch.rpt_timeout == 0) {
            nb_commit_mode(0);
        }
        prev = g_passcode_ch.rpt_timeout;
        g_passcode_ch.rpt_timeout = (uint16_t)(prev + 1);
        if (prev > PASSCODE_SET_DELAY) {
            g_passcode_ch.active = 1;
            g_passcode_ch.rotor_a = 0;
            g_passcode_ch.rpt_timeout = 0; /* clears +0x0C/+0x0D */
            g_vs.code_len = (uint8_t)passcode_digit_count(PASSCODE_NEW32);
            nb_commit_mode((int16_t)(((uint16_t)g_vs.code_len << 8) | 0x0011u));
        }
    } else {
        /* running: service the key monitors, append one key, rotate the report frames. */
        vcu_faultmon_sensor_debounce_a();
        vcu_faultmon_sensor_debounce_b();
        fm_comms_wd_3();
        fm_comms_wd_6();

        prv_key_entry(prv_confirm_keys);

        step = g_passcode_ch.rotor_a;
        if (step < PASSCODE_ROTOR_LIMIT) {
            prv_rotor_report(step, prv_report_keys, &g_scr.countdown_mirror, prv_report_status);
            g_passcode_ch.rotor_a = (uint8_t)((step >= PASSCODE_ROTOR_WRAP) ? 0u : (step + 1u));
        }

        /* entry complete: terminate, pack, compare with the staged code, commit on a match. */
        if (g_vs.code_cursor >= (uint16_t)g_vs.code_len) {
            g_vs.code_entry[g_vs.code_cursor] = 0;
            staged = PASSCODE_NEW32;
            if ((int32_t)passcode_pack_keys(PASSCODE_ENTRY) == (int32_t)staged) {
                buzzer_beep(BEEP_DOUBLE);
                prv_report_result(1u);
                vcu_in2_passcode_set(g_in2.f, staged); /* commit the new key passcode (reg 0x71/0x72) */
                g_in1.recompute_req = 1;
            } else {
                buzzer_beep(BEEP_LONG);
                prv_report_result(2u);
                prv_cancel_clear();
            }
            passcode_entry_reset();
            nb_commit_mode(1);
            g_vs.passcode_set_pending = 0;
        }
    }
}

/* ========================================================================= */
/* Unlock ticks: shared judge, try / lock-out timer and idle report rotor.   */
/* ========================================================================= */

/**
 * \brief           What differs between the key-sequence and the numeric unlock
 */
typedef struct {
    volatile uint16_t* timer;    /*!< try / lock-out timer                                          */
    volatile uint16_t* rotor;    /*!< idle report rotor                                             */
    void (*entry_report)(void);  /*!< the entry report (wrong entry, rotor step A)                  */
    const volatile void* value;  /*!< group-100 payload (rotor step B)                              */
    void (*status_report)(void); /*!< the status report (rotor step C)                              */
    uint8_t key_seq;             /*!< 1 = key sequence: a wrong entry restarts the cursor and the
                                      timer expiry resets the entry also when it raises the alarm */
} prv_unlock_t;

/**
 * \brief           Report the cursor position as the status word (numeric rotor step C)
 */
static void prv_report_cursor(void) {
    g_scr.passcode_status = g_vs.code_cursor;
    prv_report_status();
}

/** \brief The key-sequence unlock. */
static const prv_unlock_t prv_key_unlock = {&g_passcode_ch.timer_sib, &g_passcode_ch.rotor_sib, prv_report_keys,
                                            &g_scr.passcode_display,  prv_report_status,        1u};

/** \brief The numeric unlock. */
static const prv_unlock_t prv_digit_unlock = {&g_passcode_ch.timer_menu, &g_passcode_ch.rotor_menu, prv_report_display,
                                              &g_scr.countdown_mirror,   prv_report_cursor,         0u};

/**
 * \brief           Judge a complete entry: unlock on a match, else count a try or arm the lock-out
 * \param[in]       kind: the unlock
 * \param[in]       match: 1 if the entry matches the stored passcode
 * \return          `match`
 * \note            Match: beep (double), status 8, screen 1. Wrong: beep (long); a try below
 *                      the third sets status 4 | the try bits, reports the status and the entry
 *                      and reloads the timer (200 ticks, or 0x14 and screen 0 while the lock-out
 *                      is armed); the third arms the lock-out: status 0x0C, timer 300.
 */
static uint8_t prv_entry_judge(const prv_unlock_t* kind, uint8_t match) {
    if (match != 0u) {
        buzzer_beep(BEEP_DOUBLE);
        g_scr.passcode_status = 8;
        nb_commit_mode(1);
    } else {
        buzzer_beep(BEEP_LONG);
        if (kind->key_seq != 0u) {
            g_vs.code_cursor = 0;
        }
        g_vs.code_retries = (uint8_t)(g_vs.code_retries + 1);
        if (g_vs.code_retries < PASSCODE_TRIES) {
            g_scr.passcode_status = 4;
            g_scr.passcode_status |= g_passcode_ch.reload[g_vs.code_retries];
            prv_report_status();
            kind->entry_report();
            if (g_vs.lockout_phase == LOCKOUT_ARMED) {
                *kind->timer = PASSCODE_LOCKOUT_GRACE;
                nb_commit_mode(0);
            } else {
                *kind->timer = PASSCODE_RETRY_TICKS;
            }
        } else {
            g_vs.lockout_phase = LOCKOUT_ARMED;
            g_scr.passcode_status = 0x0C;
            *kind->timer = PASSCODE_LOCKOUT_TICKS;
            prv_report_status();
        }
    }
    return match;
}

/**
 * \brief           Run the try / lock-out timer, then the idle report rotor
 * \param[in]       kind: the unlock
 * \note            The timer counts down (to 0 at once in the alarm phase). After the third try
 *                      its expiry requests the shutdown and the rotor does not run while it counts.
 *                      Otherwise the expiry raises the alarm while the lock-out is armed and
 *                      restarts the entry (key sequence: always; numeric: when not armed). The
 *                      rotor steps 0..14 while the timer is idle, reporting the entry at 0, the
 *                      group-100 value at 5 and the status at 10, and rests at 0 otherwise.
 */
static void prv_unlock_tail(const prv_unlock_t* kind) {
    volatile uint16_t* timer = kind->timer;
    uint16_t step;
    uint8_t hold = 0u;

    if (*timer != 0) {
        *timer = (uint16_t)(*timer - 1);
        if (g_vs.lockout_phase == LOCKOUT_ALARM) {
            *timer = 0;
        }
        if (g_vs.code_retries > 2) {
            if (*timer == 0) {
                g_vs.shutdown_req = 1;
            }
            hold = 1u;
        } else if (*timer == 0) {
            if (g_vs.lockout_phase == LOCKOUT_ARMED) {
                g_vs.lockout_phase = LOCKOUT_ALARM;
                if (kind->key_seq != 0u) {
                    passcode_entry_reset();
                }
            } else {
                passcode_entry_reset();
            }
        }
    }

    if (hold == 0u) {
        step = *kind->rotor;
        if (step < PASSCODE_ROTOR_LIMIT && *timer == 0) {
            prv_rotor_report(step, kind->entry_report, kind->value, kind->status_report);
            *kind->rotor = (uint16_t)(*kind->rotor + 1);
        } else {
            *kind->rotor = 0;
        }
    }
}

/* ========================================================================= */
/* Key-sequence passcode unlock, 3 tries then lock-out.                      */
/* ========================================================================= */
uint8_t passcode_key_unlock_tick(void) {
    uint8_t unlocked = 0u;

    vcu_faultmon_sensor_debounce_a();
    vcu_faultmon_sensor_debounce_b();
    fm_comms_wd_3();
    fm_comms_wd_6();
    control_countdown_beep();

    /* append one handlebar key to the entry (same key codes as the confirm entry). */
    prv_key_entry(prv_unlock_keys);

    /* entry complete and no timer pending: pack, compare, count a try or lock out. */
    if (g_vs.code_cursor >= (uint16_t)g_vs.code_len && g_passcode_ch.timer_sib == 0) {
        g_vs.code_entry[g_vs.code_cursor] = 0;
        unlocked = prv_entry_judge(
            &prv_key_unlock,
            ((int32_t)passcode_pack_keys(PASSCODE_ENTRY) == (int32_t)vcu_in2_passcode_get(g_in2.f)) ? 1u : 0u);
    }
    if (unlocked == 0u) {
        prv_unlock_tail(&prv_key_unlock);
    }
    return unlocked;
}

/**
 * \brief           Move the numeric entry cursor and load the digit under it
 * \param[in]       cursor: new cursor position
 */
static void prv_cursor_set(uint16_t cursor) {
    g_vs.code_cursor = cursor;
    g_vs.code_digit = (uint16_t)g_vs.code_entry[cursor];
}

/* ========================================================================= */
/* Numeric passcode unlock, 3 tries then lock-out.                           */
/* ========================================================================= */
uint8_t passcode_digit_unlock_tick(void) {
    uint8_t unlocked = 0u;

    vcu_faultmon_sensor_debounce_a();
    vcu_faultmon_sensor_debounce_b();
    fm_sensor_fsm_3();
    fm_sensor_fsm_2();
    fm_comms_wd_1();
    fm_comms_wd_2();
    fm_comms_wd_3();
    control_countdown_beep();

    if (g_evt.mon[FM_MON_LIGHT_HI].code == 0x04) {
        /* DOWN: decrement the digit under the cursor, wrapping 0 -> 9. */
        buzzer_beep(BEEP_SHORT);
        g_evt.mon[FM_MON_LIGHT_HI].code = 0;
        if (g_vs.code_digit == PASSCODE_DIGIT_MIN) {
            g_vs.code_digit = (uint16_t)PASSCODE_DIGIT_MAX;
        } else {
            g_vs.code_digit = (uint16_t)(g_vs.code_digit - 1);
        }
        g_vs.code_entry[g_vs.code_cursor] = (uint8_t)g_vs.code_digit;
    } else if (g_evt.mon[FM_MON_LIGHT_LO].code == 0x04) {
        /* UP: increment the digit under the cursor, wrapping 9 -> 0. */
        buzzer_beep(BEEP_SHORT);
        g_evt.mon[FM_MON_LIGHT_LO].code = 0;
        if (g_vs.code_digit < (uint16_t)PASSCODE_DIGIT_MAX) {
            g_vs.code_digit = (uint16_t)(g_vs.code_digit + 1);
        } else {
            g_vs.code_digit = PASSCODE_DIGIT_MIN;
        }
        g_vs.code_entry[g_vs.code_cursor] = (uint8_t)g_vs.code_digit;
    } else if (g_evt.mon[FM_MON_INTERLOCK].code == 0x04 || g_evt.mon[FM_MON_BRAKE_FULL].code == 0x04
               || g_evt.mon[FM_MON_BRAKE_HALF].code == 0x04) {
        /* SELECT: accept the digit; after the last digit compare with reg 0x61. */
        buzzer_beep(BEEP_SHORT);
        g_evt.mon[FM_MON_BRAKE_FULL].code = 0;
        g_evt.mon[FM_MON_BRAKE_HALF].code = 0;
        g_evt.mon[FM_MON_INTERLOCK].code = 0;
        if (g_passcode_ch.timer_menu == 0) {
            if (g_vs.code_cursor == g_vs.code_count) {
                g_vs.code_count = (uint16_t)(g_vs.code_count + 1);
            }
            prv_cursor_set(g_vs.code_count); /* the next free digit */
            if (g_vs.code_count >= (uint16_t)g_vs.code_len) {
                unlocked = prv_entry_judge(&prv_digit_unlock, ((int32_t)passcode_bcd_to_decimal(PASSCODE_ENTRY)
                                                               == (int32_t)(int16_t)g_in2.f.passcode_num)
                                                                  ? 1u
                                                                  : 0u);
            }
        }
    }

    if (unlocked == 0u) {
        if (g_evt.mon[FM_MON_SUBSTATE_PREV].code == 0x01) {
            /* PREV: step the cursor back one digit. */
            g_evt.mon[FM_MON_SUBSTATE_PREV].code = 0;
            if (g_vs.code_cursor != 0) {
                prv_cursor_set((uint16_t)(g_vs.code_cursor - 1));
            }
        } else if (g_evt.mon[FM_MON_SUBSTATE_NEXT].code == 0x01) {
            /* NEXT: step the cursor forward while it trails the read cursor. */
            g_evt.mon[FM_MON_SUBSTATE_NEXT].code = 0;
            if (g_vs.code_cursor < g_vs.code_count) {
                prv_cursor_set((uint16_t)(g_vs.code_cursor + 1));
            }
        }
        prv_unlock_tail(&prv_digit_unlock);
    }
    return unlocked;
}

/* ------------------------------------------------------------------------- */
/* Passcode length (pure).                                                   */
/* ------------------------------------------------------------------------- */

/** \brief Passcode length = count of the non-zero nibbles in `codes`. */
int8_t passcode_digit_count(uint32_t codes) {
    int8_t count;
    uint8_t i;

    count = 0;
    for (i = 0; i < 8u; ++i) {
        if ((codes & 0xfu) != 0u) {
            count = (int8_t)(count + 1);
        }
        codes >>= 4;
    }
    return count;
}

/* ------------------------------------------------------------------------- */
/* Passcode entry packers.                                                   */
/* ------------------------------------------------------------------------- */

/* Pack the 8-entry key buffer into nibble words. */
uint32_t passcode_pack_keys(const uint8_t* digits) {
    uint32_t packed;
    uint32_t i;
    uint32_t out;

    packed = (uint32_t)digits[7];
    i = 1;
    do {
        packed = (uint32_t)digits[7 - i] | (packed << 4);
        i = (i + 1) & 0xFF;
    } while (i < 8);
    g_passcode_ch.disp_word = packed;

    out = (uint32_t)digits[0];
    if (out != 0) {
        out = out + 4;
    }
    i = 1;
    do {
        if (digits[i] != 0) {
            out = (uint32_t)(digits[i] + 4) | (out << 4);
        }
        i = (i + 1) & 0xFF;
    } while (i < 8);
    return out;
}

/* Build the 4-digit display word. */
uint16_t passcode_display_word(const uint8_t* digits) {
    uint8_t local[4];
    uint32_t i;
    uint16_t word;

    local[0] = 0;
    local[1] = 0;
    local[2] = 0;
    local[3] = 0;

    /* Blank (0xB) every digit past the active count. */
    for (i = (uint32_t)(uint8_t)((char)g_vs.code_count + 1); i < 4; i = (i + 1) & 0xFF) {
        local[i] = 0xB;
    }
    /* Copy the selected digit; dash (0xA) the others up to the active count. */
    local[g_vs.code_cursor] = digits[g_vs.code_cursor];
    for (i = 0; i <= g_vs.code_count; i = (i + 1) & 0xFF) {
        if (g_vs.code_cursor != i) {
            local[i] = 10;
        }
    }

    word = (uint16_t)((uint16_t)local[0] | (uint16_t)(local[3] << 0xC) | (uint16_t)(local[2] << 8)
                      | (uint16_t)(local[1] << 4));

    /* Raw-mode (SCR->passcode_status bit2): emit the four raw digit nibbles instead. */
    if ((int32_t)((uint32_t)g_scr.passcode_status << 0x1D) < 0) { /* RAWCFG bit2 */
        word = (uint16_t)((uint16_t)digits[0]
                          | (uint16_t)(((uint16_t)digits[1]
                                        | (uint16_t)(((uint16_t)digits[2] | (uint16_t)((digits[3] & 0xF) << 4)) << 4))
                                       << 4));
    }
    return word;
}

/* BCD digits -> decimal value. */
int16_t passcode_bcd_to_decimal(const uint8_t* digits) {
    return (
        int16_t)((uint16_t)digits[3]
                 + (uint16_t)(((uint16_t)((uint16_t)digits[0] * 10 + (uint16_t)digits[1]) * 10 + (uint16_t)digits[2])
                              * 10));
}

/* ------------------------------------------------------------------------- */
/* Passcode entry reset + report.                                            */
/* ------------------------------------------------------------------------- */

/* Clear the passcode entry (buffer, cursor, digit, count), then size it for the selected
   passcode kind (reg 0x1D): numeric (b7) = 4 digits; key sequence (b8) = the length of the
   stored key passcode, with the end mark and the packed 0xD3 frame; always clear the status
   word SCR+0xC4 and enqueue the 0x62 frame. */
void passcode_entry_reset(void) {
    g_vs.code_count = 0;                          /* VS+0x54 */
    g_vs.code_cursor = 0;                         /* VS+0x52 */
    g_vs.code_digit = 0;                          /* VS+0x50 */
    *(volatile uint32_t*)g_vs.code_entry = 0;     /* VS+0x47..0x4A (digits 0..3) */
    *(volatile uint32_t*)&g_vs.code_entry[4] = 0; /* VS+0x4B..0x4E (digits 4..7) */

    if ((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x18) < 0) { /* CFG_PASSCODE_NUM */
        g_vs.code_len = 4;
    } else if ((int32_t)((uint32_t)g_in2.f.cfg_flags << 0x17) < 0) { /* CFG_PASSCODE_KEY */
        g_vs.code_len = (uint8_t)passcode_digit_count(vcu_in2_passcode_get(g_in2.f));
        g_vs.code_entry[g_vs.code_cursor] = 8;
        prv_report_keys();
    }

    g_scr.passcode_status = 0; /* SCR+0xC4 */
    prv_report_status();
}
