/**
 * \file            runtime.c
 * \brief           Compact replacements for libgcc / libc start-up helpers.
 * \note            `__libc_init_array`: the startup code (`Reset_Handler`) calls it between
 *                      `SystemInit` and `main` to run static constructors. The firmware has
 *                      none (C only, no `__attribute__((constructor))`, and the linker scripts
 *                      place no `.preinit_array` / `.init_array`), so this empty version
 *                      replaces the 72-byte libc one and keeps `lib_a-init.o` out of the link.
 *                      If constructor code is ever added, this function has to run the
 *                      `.preinit_array` and `.init_array` entries again.
 *
 *                      `__udivmoddi4`: only the BSP clock code divides 64-bit values
 *                      (`crm_clocks_freq_get` and `system_core_clock_update`, through
 *                      `__aeabi_uldivmod`). libgcc's generic `__udivmoddi4` is 716 bytes; this
 *                      shift-subtract version gives the same quotient and remainder in about a
 *                      tenth of that, and runs only during clock set-up, so the slower loop is
 *                      fine against the 60 KB flash budget.
 *
 *                      The symbol names are fixed by the libgcc / libc ABI, so they do not
 *                      follow the project naming rules. Defining them here keeps `_udivmoddi4.o`
 *                      and `lib_a-init.o` out of the link.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <stddef.h>
#include <stdint.h>

#define RT_U64_TOP_BIT 0x8000000000000000ULL

void __libc_init_array(void);
uint64_t __udivmoddi4(uint64_t num, uint64_t den, uint64_t* rem);

/**
 * \brief           Static constructor hook called by the startup code; empty (no constructors)
 */
void __libc_init_array(void) {}

/**
 * \brief           Unsigned 64-bit division with remainder (libgcc `__udivmoddi4` ABI)
 * \note            `__aeabi_uldivmod` handles a zero divisor before it calls this function.
 *                      For a direct call with `den == 0` this returns 0 and stores `num` as the
 *                      remainder rather than looping forever.
 * \param[in]       num: dividend
 * \param[in]       den: divisor
 * \param[out]      rem: remainder `num % den`; may be `NULL`
 * \return          Quotient `num / den`
 */
uint64_t __udivmoddi4(uint64_t num, uint64_t den, uint64_t* rem) {
    uint64_t quot = 0U;
    uint64_t bit = 1U;

    if (den == 0U) {
        if (rem != NULL) {
            *rem = num;
        }
        return 0U;
    }
    /* Align the divisor under the dividend's top bit, then subtract back down. */
    while ((den < num) && ((den & RT_U64_TOP_BIT) == 0U)) {
        den <<= 1;
        bit <<= 1;
    }
    while (bit != 0U) {
        if (num >= den) {
            num -= den;
            quot |= bit;
        }
        bit >>= 1;
        den >>= 1;
    }
    if (rem != NULL) {
        *rem = num;
    }
    return quot;
}
