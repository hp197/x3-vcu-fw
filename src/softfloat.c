/**
 * \file            softfloat.c
 * \brief           Compact IEEE-754 soft-float run-time (the `__aeabi_*` helpers GCC calls).
 * \note            The AT32F415 has no FPU, so every `float`/`double` op is a run-time call.
 *                      By default those calls resolve to libgcc's speed-tuned helpers. This file
 *                      is an opt-in replacement: define `VCU_SOFTFLOAT_COMPACT` and these smaller
 *                      C versions define the symbols instead, so the linker pulls no libgcc float
 *                      object. Without the define the whole body below drops out and libgcc wins.
 *
 *                      It trades a little speed for flash. Measured, the compact set saves ~568 B
 *                      on gt3 and ~736 B on zt3/f3, RAM unchanged. Reach for it only under flash
 *                      pressure; libgcc's helpers are faster, and the IMU filter runs about 300
 *                      float ops every 10 ms.
 *
 *                      Every operation is IEEE-754 binary64/binary32 with round to nearest,
 *                      ties to even, as libgcc's:
 *                      - Subnormals are kept. Overflow gives +-inf, an invalid operation
 *                        (inf - inf, 0 * inf, 0 / 0, inf / inf) gives the default quiet NaN, and
 *                        a NaN operand gives the default quiet NaN. NaN payloads are not
 *                        propagated, and no exception flags are kept.
 *                      - `float` add, subtract, multiply and divide have their own 32-bit code:
 *                        the IMU filter runs about 300 of them every 10 ms. The `float` compares
 *                        and conversions go through the exact widening to `double`.
 *                      - Conversions to an integer truncate toward zero and saturate, as
 *                        libgcc's: NaN -> 0, below the range -> INT32_MIN (unsigned: 0), above
 *                        -> INT32_MAX (unsigned: UINT32_MAX).
 *
 *                      The symbol names are fixed by the Arm run-time ABI (AAPCS RTABI), so they
 *                      do not follow the project naming rules.
 *
 *                      Each libgcc float object defines several helpers, so pulling one in for a
 *                      helper missing here would define the rest twice. This set therefore covers
 *                      all the int-returning compares, the reverse subtracts and the 64-bit int
 *                      to double conversions; a new helper is a `multiple definition` link error,
 *                      never a silent mix, so add it here. `--gc-sections` drops what is unused.
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <stdint.h>
#include <string.h>

#ifdef VCU_SOFTFLOAT_COMPACT /* opt-in; without it libgcc provides the __aeabi_* float helpers */

/* binary64 fields */
#define F64_SIGN     0x8000000000000000ULL /*!< sign bit */
#define F64_FRAC     0x000FFFFFFFFFFFFFULL /*!< stored significand */
#define F64_HIDDEN   0x0010000000000000ULL /*!< implicit leading 1 */
#define F64_DEF_NAN  0x7FF8000000000000ULL /*!< default quiet NaN */
#define F64_INF      0x7FF0000000000000ULL /*!< +inf */
#define F64_EXP_MAX  0x7FFU                /*!< all-ones exponent: inf / NaN */
#define F64_BIAS     1023                  /*!< exponent bias */
#define F64_RND_BITS 0x3FFU                /*!< bits below the result in the working significand */
#define F64_RND_HALF 0x200U                /*!< half an ulp of the working significand */

/* binary32 fields */
#define F32_SIGN     0x80000000U /*!< sign bit */
#define F32_FRAC     0x007FFFFFU /*!< stored significand */
#define F32_HIDDEN   0x00800000U /*!< implicit leading 1 */
#define F32_DEF_NAN  0x7FC00000U /*!< default quiet NaN */
#define F32_EXP_MAX  0xFFU       /*!< all-ones exponent: inf / NaN */
#define F32_BIAS     127         /*!< exponent bias */
#define F32_RND_BITS 0x7FU       /*!< bits below the result in the working significand */
#define F32_RND_HALF 0x40U       /*!< half an ulp of the working significand */

/* ------------------------------------------------------------------------- */
/* Bit access and helpers                                                     */
/* ------------------------------------------------------------------------- */

/** \brief Bits of a double \param[in] value: the double \return its IEEE-754 bits */
static uint64_t prv_d2bits(double value) {
    uint64_t bits;

    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/** \brief Double with the given bits \param[in] bits: IEEE-754 bits \return the double */
static double prv_bits2d(uint64_t bits) {
    double value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/** \brief Bits of a float \param[in] value: the float \return its IEEE-754 bits */
static uint32_t prv_f2bits(float value) {
    uint32_t bits;

    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/** \brief Float with the given bits \param[in] bits: IEEE-754 bits \return the float */
static float prv_bits2f(uint32_t bits) {
    float value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/** \brief Whether binary64 \p bits is a NaN \param[in] bits: IEEE-754 bits \return 1 for NaN */
static uint32_t prv_is_nan64(uint64_t bits) { return ((bits << 1) > (F64_INF << 1)) ? 1U : 0U; }

/**
 * \brief           Shift right, keeping a 1 in bit 0 if any 1 bit was shifted out ("jamming")
 * \param[in]       val: value
 * \param[in]       count: shift count, any size
 * \return          `val >> count` with the sticky bit
 */
static uint64_t prv_shr_jam(uint64_t val, uint32_t count) {
    uint64_t res = val;

    if (count >= 64U) {
        res = (val != 0U) ? 1U : 0U;
    } else if (count != 0U) {
        res = (val >> count) | (((val << (64U - count)) != 0U) ? 1U : 0U);
    }
    return res;
}

/**
 * \brief           Round and pack a binary64 result
 * \note            \p sig has its leading 1 at bit 62 (or is a subnormal result that will be
 *                      shifted down) and 10 bits below the result's last bit. \p exp is the
 *                      biased exponent minus 1: the leading 1 adds the 1 when it is packed.
 * \param[in]       sign: `F64_SIGN` or 0
 * \param[in]       exp: biased exponent minus 1
 * \param[in]       sig: working significand
 * \return          the rounded binary64 bits
 */
static uint64_t prv_pack64(uint64_t sign, int32_t exp, uint64_t sig) {
    uint64_t res;
    uint32_t low;

    if ((exp > 0x7FD) || ((exp == 0x7FD) && ((sig + F64_RND_HALF) >= F64_SIGN))) {
        res = sign | F64_INF; /* overflow */
    } else {
        if (exp < 0) {
            sig = prv_shr_jam(sig, (uint32_t)(-exp)); /* subnormal result */
            exp = 0;
        }
        low = (uint32_t)sig & F64_RND_BITS;
        sig = (sig + F64_RND_HALF) >> 10;
        if (low == F64_RND_HALF) {
            sig &= ~1ULL; /* tie: round to even */
        }
        if (sig == 0U) {
            exp = 0;
        }
        res = sign | (((uint64_t)(uint32_t)exp << 52) + sig);
    }
    return res;
}

/**
 * \brief           Normalise, round and pack a binary64 result
 * \param[in]       sign: `F64_SIGN` or 0
 * \param[in]       exp: biased exponent the value would have with its leading 1 at bit 62
 * \param[in]       sig: non-zero working significand, value = `sig * 2^(exp - 1023 - 62)`
 * \return          the rounded binary64 bits
 */
static uint64_t prv_norm_pack64(uint64_t sign, int32_t exp, uint64_t sig) {
    int32_t shift;

    if ((sig & F64_SIGN) != 0U) {
        sig = prv_shr_jam(sig, 1U);
        exp++;
    }
    shift = (int32_t)__builtin_clzll(sig) - 1;
    return prv_pack64(sign, exp - shift - 1, sig << shift);
}

/**
 * \brief           Split a finite non-zero binary64 into a normalised significand and exponent
 * \param[in]       bits: IEEE-754 bits (finite, not zero)
 * \param[out]      exp: biased exponent (below 1 for a subnormal)
 * \return          significand with the leading 1 at bit 52
 */
static uint64_t prv_unpack64(uint64_t bits, int32_t* exp) {
    uint64_t sig = bits & F64_FRAC;
    int32_t shift;

    *exp = (int32_t)((bits >> 52) & F64_EXP_MAX);
    if (*exp == 0) {
        shift = (int32_t)__builtin_clzll(sig) - 11;
        sig <<= shift;
        *exp = 1 - shift;
    } else {
        sig |= F64_HIDDEN;
    }
    return sig;
}

/* ------------------------------------------------------------------------- */
/* binary64 arithmetic                                                        */
/* ------------------------------------------------------------------------- */

/**
 * \brief           binary64 addition on bits
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          a + b
 */
static uint64_t prv_add64(uint64_t abits, uint64_t bbits) {
    uint64_t res;
    uint64_t asign = abits & F64_SIGN;
    uint64_t bsign = bbits & F64_SIGN;
    uint64_t asig;
    uint64_t bsig;
    uint64_t tmp;
    int32_t aexp = (int32_t)((abits >> 52) & F64_EXP_MAX);
    int32_t bexp = (int32_t)((bbits >> 52) & F64_EXP_MAX);

    if ((prv_is_nan64(abits) != 0U) || (prv_is_nan64(bbits) != 0U)) {
        res = F64_DEF_NAN;
    } else if (aexp == (int32_t)F64_EXP_MAX) {
        res = ((bexp == (int32_t)F64_EXP_MAX) && (asign != bsign)) ? F64_DEF_NAN : abits; /* inf - inf */
    } else if (bexp == (int32_t)F64_EXP_MAX) {
        res = bbits;
    } else if ((abits << 1) == 0U) {
        res = ((bbits << 1) == 0U) ? (abits & bbits) : bbits; /* -0 + -0 = -0, else +0 */
    } else if ((bbits << 1) == 0U) {
        res = abits;
    } else {
        /* Leading 1 at bit 61: one bit of headroom for the carry of an addition. */
        asig = ((aexp != 0) ? ((abits & F64_FRAC) | F64_HIDDEN) : (abits & F64_FRAC)) << 9;
        bsig = ((bexp != 0) ? ((bbits & F64_FRAC) | F64_HIDDEN) : (bbits & F64_FRAC)) << 9;
        aexp = (aexp != 0) ? aexp : 1;
        bexp = (bexp != 0) ? bexp : 1;
        if ((bexp > aexp) || ((bexp == aexp) && (bsig > asig))) { /* make |a| >= |b| */
            tmp = asig;
            asig = bsig;
            bsig = tmp;
            tmp = asign;
            asign = bsign;
            bsign = tmp;
            aexp ^= bexp;
            bexp ^= aexp;
            aexp ^= bexp;
        }
        bsig = prv_shr_jam(bsig, (uint32_t)(aexp - bexp));
        asig = (asign == bsign) ? (asig + bsig) : (asig - bsig);
        res = (asig == 0U) ? 0U : prv_norm_pack64(asign, aexp + 1, asig); /* x - x = +0 */
    }
    return res;
}

/**
 * \brief           binary64 multiplication on bits
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          a * b
 */
static uint64_t prv_mul64(uint64_t abits, uint64_t bbits) {
    uint64_t res;
    uint64_t sign = (abits ^ bbits) & F64_SIGN;
    uint64_t asig;
    uint64_t bsig;
    uint64_t p00;
    uint64_t p01;
    uint64_t p10;
    uint64_t mid;
    uint64_t high;
    uint32_t a_inf = (((abits << 1) >> 53) == F64_EXP_MAX) ? 1U : 0U;
    uint32_t b_inf = (((bbits << 1) >> 53) == F64_EXP_MAX) ? 1U : 0U;
    uint32_t a_zero = ((abits << 1) == 0U) ? 1U : 0U;
    uint32_t b_zero = ((bbits << 1) == 0U) ? 1U : 0U;
    int32_t aexp;
    int32_t bexp;

    if ((prv_is_nan64(abits) != 0U) || (prv_is_nan64(bbits) != 0U) || ((a_inf & b_zero) != 0U)
        || ((b_inf & a_zero) != 0U)) {
        res = F64_DEF_NAN; /* NaN, or 0 * inf */
    } else if ((a_inf | b_inf) != 0U) {
        res = sign | F64_INF;
    } else if ((a_zero | b_zero) != 0U) {
        res = sign;
    } else {
        /* 53 x 53 bits, each shifted up so its leading 1 is at bit 63: the top 64 bits of the
           128-bit product, with the low half folded into the sticky bit. */
        asig = prv_unpack64(abits, &aexp) << 11;
        bsig = prv_unpack64(bbits, &bexp) << 11;
        p00 = (uint64_t)(uint32_t)asig * (uint32_t)bsig;
        p01 = (uint64_t)(uint32_t)asig * (uint32_t)(bsig >> 32);
        p10 = (uint64_t)(uint32_t)(asig >> 32) * (uint32_t)bsig;
        mid = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;
        high = ((uint64_t)(uint32_t)(asig >> 32) * (uint32_t)(bsig >> 32)) + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
        if (((mid << 32) | (uint32_t)p00) != 0U) {
            high |= 1U;
        }
        res = prv_norm_pack64(sign, aexp + bexp - F64_BIAS, high);
    }
    return res;
}

/**
 * \brief           binary64 division on bits
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          a / b
 */
static uint64_t prv_div64(uint64_t abits, uint64_t bbits) {
    uint64_t res;
    uint64_t sign = (abits ^ bbits) & F64_SIGN;
    uint64_t asig;
    uint64_t bsig;
    uint64_t quot = 0U;
    uint32_t a_inf = (((abits << 1) >> 53) == F64_EXP_MAX) ? 1U : 0U;
    uint32_t b_inf = (((bbits << 1) >> 53) == F64_EXP_MAX) ? 1U : 0U;
    uint32_t a_zero = ((abits << 1) == 0U) ? 1U : 0U;
    uint32_t b_zero = ((bbits << 1) == 0U) ? 1U : 0U;
    uint32_t step;
    int32_t aexp;
    int32_t bexp;

    if ((prv_is_nan64(abits) != 0U) || (prv_is_nan64(bbits) != 0U) || ((a_inf & b_inf) != 0U)
        || ((a_zero & b_zero) != 0U)) {
        res = F64_DEF_NAN; /* NaN, inf / inf, 0 / 0 */
    } else if ((a_inf | b_zero) != 0U) {
        res = sign | F64_INF;
    } else if ((a_zero | b_inf) != 0U) {
        res = sign;
    } else {
        asig = prv_unpack64(abits, &aexp);
        bsig = prv_unpack64(bbits, &bexp);
        if (asig < bsig) {
            asig <<= 1; /* quotient in [1, 2) */
            aexp--;
        }
        /* Restoring division: 63 quotient bits, leading 1 at bit 62, then the sticky bit. */
        for (step = 0U; step < 63U; step++) {
            quot <<= 1;
            if (asig >= bsig) {
                asig -= bsig;
                quot |= 1U;
            }
            asig <<= 1;
        }
        if (asig != 0U) {
            quot |= 1U;
        }
        res = prv_norm_pack64(sign, aexp - bexp + F64_BIAS, quot);
    }
    return res;
}

/**
 * \brief           Three-way binary64 compare
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          -1 (a < b), 0 (equal, including +0 == -0), 1 (a > b), 2 (unordered: a NaN)
 */
static int32_t prv_cmp64(uint64_t abits, uint64_t bbits) {
    int32_t res;
    uint64_t akey;
    uint64_t bkey;

    if ((prv_is_nan64(abits) != 0U) || (prv_is_nan64(bbits) != 0U)) {
        res = 2;
    } else if (((abits | bbits) << 1) == 0U) {
        res = 0;
    } else {
        /* Map sign-magnitude to an unsigned order. */
        akey = ((abits & F64_SIGN) != 0U) ? ~abits : (abits | F64_SIGN);
        bkey = ((bbits & F64_SIGN) != 0U) ? ~bbits : (bbits | F64_SIGN);
        res = (akey < bkey) ? -1 : ((akey > bkey) ? 1 : 0);
    }
    return res;
}

/* ------------------------------------------------------------------------- */
/* binary32 <-> binary64                                                      */
/* ------------------------------------------------------------------------- */

/**
 * \brief           Widen binary32 to binary64 (exact)
 * \param[in]       bits: binary32 bits
 * \return          binary64 bits
 */
static uint64_t prv_f2d_bits(uint32_t bits) {
    uint64_t res;
    uint64_t sign = (uint64_t)(bits & F32_SIGN) << 32;
    uint32_t exp = (bits >> 23) & F32_EXP_MAX;
    uint32_t frac = bits & F32_FRAC;
    int32_t shift;

    if (exp == F32_EXP_MAX) {
        res = (frac != 0U) ? F64_DEF_NAN : (sign | F64_INF);
    } else if ((exp == 0U) && (frac == 0U)) {
        res = sign;
    } else if (exp == 0U) {
        shift = (int32_t)__builtin_clz(frac) - 8; /* subnormal: leading 1 to bit 23 */
        res = sign | ((uint64_t)(uint32_t)(F64_BIAS - F32_BIAS + 1 - shift) << 52)
              | (((uint64_t)(frac << shift) & F32_FRAC) << 29);
    } else {
        res = sign | ((uint64_t)(exp + (uint32_t)(F64_BIAS - F32_BIAS)) << 52) | ((uint64_t)frac << 29);
    }
    return res;
}

/* ------------------------------------------------------------------------- */
/* binary32 arithmetic (native: the IMU filter runs ~300 of these per 10 ms)  */
/* ------------------------------------------------------------------------- */

/** \brief Whether binary32 \p bits is a NaN \param[in] bits: IEEE-754 bits \return 1 for NaN */
static uint32_t prv_is_nan32(uint32_t bits) { return ((bits << 1) > 0xFF000000U) ? 1U : 0U; }

/**
 * \brief           32-bit shift right with a sticky bit, as \ref prv_shr_jam
 * \param[in]       val: value
 * \param[in]       count: shift count, any size
 * \return          `val >> count` with the sticky bit
 */
static uint32_t prv_shr_jam32(uint32_t val, uint32_t count) {
    uint32_t res = val;

    if (count >= 32U) {
        res = (val != 0U) ? 1U : 0U;
    } else if (count != 0U) {
        res = (val >> count) | (((val << (32U - count)) != 0U) ? 1U : 0U);
    }
    return res;
}

/**
 * \brief           Round and pack a binary32 result (as \ref prv_pack64)
 * \note            \p sig has its leading 1 at bit 30 and 7 bits below the result's last bit.
 * \param[in]       sign: `F32_SIGN` or 0
 * \param[in]       exp: biased exponent minus 1
 * \param[in]       sig: working significand
 * \return          the rounded binary32 bits
 */
static uint32_t prv_pack32(uint32_t sign, int32_t exp, uint32_t sig) {
    uint32_t res;
    uint32_t low;

    if ((exp > 0xFD) || ((exp == 0xFD) && ((sig + F32_RND_HALF) >= F32_SIGN))) {
        res = sign | (F32_EXP_MAX << 23); /* overflow */
    } else {
        if (exp < 0) {
            sig = prv_shr_jam32(sig, (uint32_t)(-exp)); /* subnormal result */
            exp = 0;
        }
        low = sig & F32_RND_BITS;
        sig = (sig + F32_RND_HALF) >> 7;
        if (low == F32_RND_HALF) {
            sig &= ~1U; /* tie: round to even */
        }
        if (sig == 0U) {
            exp = 0;
        }
        res = sign | (((uint32_t)exp << 23) + sig);
    }
    return res;
}

/**
 * \brief           Normalise, round and pack a binary32 result
 * \param[in]       sign: `F32_SIGN` or 0
 * \param[in]       exp: biased exponent the value would have with its leading 1 at bit 30
 * \param[in]       sig: non-zero working significand, value = `sig * 2^(exp - 127 - 30)`
 * \return          the rounded binary32 bits
 */
static uint32_t prv_norm_pack32(uint32_t sign, int32_t exp, uint32_t sig) {
    int32_t shift;

    if ((sig & F32_SIGN) != 0U) {
        sig = prv_shr_jam32(sig, 1U);
        exp++;
    }
    shift = (int32_t)__builtin_clz(sig) - 1;
    return prv_pack32(sign, exp - shift - 1, sig << shift);
}

/**
 * \brief           Split a finite non-zero binary32 into a normalised significand and exponent
 * \param[in]       bits: IEEE-754 bits (finite, not zero)
 * \param[out]      exp: biased exponent (below 1 for a subnormal)
 * \return          significand with the leading 1 at bit 23
 */
static uint32_t prv_unpack32(uint32_t bits, int32_t* exp) {
    uint32_t sig = bits & F32_FRAC;
    int32_t shift;

    *exp = (int32_t)((bits >> 23) & F32_EXP_MAX);
    if (*exp == 0) {
        shift = (int32_t)__builtin_clz(sig) - 8;
        sig <<= shift;
        *exp = 1 - shift;
    } else {
        sig |= F32_HIDDEN;
    }
    return sig;
}

/**
 * \brief           binary32 addition on bits (as \ref prv_add64)
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          a + b
 */
static uint32_t prv_add32(uint32_t abits, uint32_t bbits) {
    uint32_t res;
    uint32_t asign = abits & F32_SIGN;
    uint32_t bsign = bbits & F32_SIGN;
    uint32_t asig;
    uint32_t bsig;
    uint32_t tmp;
    int32_t aexp = (int32_t)((abits >> 23) & F32_EXP_MAX);
    int32_t bexp = (int32_t)((bbits >> 23) & F32_EXP_MAX);

    if ((prv_is_nan32(abits) != 0U) || (prv_is_nan32(bbits) != 0U)) {
        res = F32_DEF_NAN;
    } else if (aexp == (int32_t)F32_EXP_MAX) {
        res = ((bexp == (int32_t)F32_EXP_MAX) && (asign != bsign)) ? F32_DEF_NAN : abits; /* inf - inf */
    } else if (bexp == (int32_t)F32_EXP_MAX) {
        res = bbits;
    } else if ((abits << 1) == 0U) {
        res = ((bbits << 1) == 0U) ? (abits & bbits) : bbits; /* -0 + -0 = -0, else +0 */
    } else if ((bbits << 1) == 0U) {
        res = abits;
    } else {
        /* Leading 1 at bit 29: one bit of headroom for the carry of an addition. */
        asig = ((aexp != 0) ? ((abits & F32_FRAC) | F32_HIDDEN) : (abits & F32_FRAC)) << 6;
        bsig = ((bexp != 0) ? ((bbits & F32_FRAC) | F32_HIDDEN) : (bbits & F32_FRAC)) << 6;
        aexp = (aexp != 0) ? aexp : 1;
        bexp = (bexp != 0) ? bexp : 1;
        if ((bexp > aexp) || ((bexp == aexp) && (bsig > asig))) { /* make |a| >= |b| */
            tmp = asig;
            asig = bsig;
            bsig = tmp;
            tmp = asign;
            asign = bsign;
            bsign = tmp;
            aexp ^= bexp;
            bexp ^= aexp;
            aexp ^= bexp;
        }
        bsig = prv_shr_jam32(bsig, (uint32_t)(aexp - bexp));
        asig = (asign == bsign) ? (asig + bsig) : (asig - bsig);
        res = (asig == 0U) ? 0U : prv_norm_pack32(asign, aexp + 1, asig); /* x - x = +0 */
    }
    return res;
}

/**
 * \brief           binary32 multiplication on bits
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          a * b
 */
static uint32_t prv_mul32(uint32_t abits, uint32_t bbits) {
    uint64_t prod;
    uint32_t res;
    uint32_t sign = (abits ^ bbits) & F32_SIGN;
    uint32_t a_inf = (((abits << 1) >> 24) == F32_EXP_MAX) ? 1U : 0U;
    uint32_t b_inf = (((bbits << 1) >> 24) == F32_EXP_MAX) ? 1U : 0U;
    uint32_t a_zero = ((abits << 1) == 0U) ? 1U : 0U;
    uint32_t b_zero = ((bbits << 1) == 0U) ? 1U : 0U;
    int32_t aexp;
    int32_t bexp;

    if ((prv_is_nan32(abits) != 0U) || (prv_is_nan32(bbits) != 0U) || ((a_inf & b_zero) != 0U)
        || ((b_inf & a_zero) != 0U)) {
        res = F32_DEF_NAN; /* NaN, or 0 * inf */
    } else if ((a_inf | b_inf) != 0U) {
        res = sign | (F32_EXP_MAX << 23);
    } else if ((a_zero | b_zero) != 0U) {
        res = sign;
    } else {
        /* 24 x 24 bits, shifted to bits 30 and 31: one UMULL, top word plus sticky. */
        prod = (uint64_t)(prv_unpack32(abits, &aexp) << 7) * (prv_unpack32(bbits, &bexp) << 8);
        res = prv_norm_pack32(sign, aexp + bexp - 126, (uint32_t)(prod >> 32) | (((uint32_t)prod != 0U) ? 1U : 0U));
    }
    return res;
}

/**
 * \brief           binary32 division on bits
 * \param[in]       abits: a
 * \param[in]       bbits: b
 * \return          a / b
 */
static uint32_t prv_div32(uint32_t abits, uint32_t bbits) {
    uint32_t res;
    uint32_t sign = (abits ^ bbits) & F32_SIGN;
    uint32_t asig;
    uint32_t bsig;
    uint32_t quot;
    uint32_t digit;
    uint32_t step;
    uint32_t a_inf = (((abits << 1) >> 24) == F32_EXP_MAX) ? 1U : 0U;
    uint32_t b_inf = (((bbits << 1) >> 24) == F32_EXP_MAX) ? 1U : 0U;
    uint32_t a_zero = ((abits << 1) == 0U) ? 1U : 0U;
    uint32_t b_zero = ((bbits << 1) == 0U) ? 1U : 0U;
    int32_t aexp;
    int32_t bexp;

    if ((prv_is_nan32(abits) != 0U) || (prv_is_nan32(bbits) != 0U) || ((a_inf & b_inf) != 0U)
        || ((a_zero & b_zero) != 0U)) {
        res = F32_DEF_NAN; /* NaN, inf / inf, 0 / 0 */
    } else if ((a_inf | b_zero) != 0U) {
        res = sign | (F32_EXP_MAX << 23);
    } else if ((a_zero | b_inf) != 0U) {
        res = sign;
    } else {
        asig = prv_unpack32(abits, &aexp);
        bsig = prv_unpack32(bbits, &bexp);
        if (asig < bsig) {
            asig <<= 1; /* quotient in [1, 2) */
            aexp--;
        }
        /* Long division on the hardware UDIV: 7 quotient bits, then three digits of 8 bits,
           so 31 bits with the leading 1 at bit 30. Every dividend fits 32 bits: asig < 2^25
           and the remainder < bsig < 2^24. The final remainder is the sticky bit. */
        quot = (asig << 6) / bsig;
        asig = (asig << 6) - (quot * bsig);
        for (step = 0U; step < 3U; step++) {
            digit = (asig << 8) / bsig;
            asig = (asig << 8) - (digit * bsig);
            quot = (quot << 8) | digit;
        }
        res = prv_norm_pack32(sign, aexp - bexp + F32_BIAS, quot | ((asig != 0U) ? 1U : 0U));
    }
    return res;
}

/**
 * \brief           Narrow binary64 to binary32, rounded to nearest-even
 * \param[in]       bits: binary64 bits
 * \return          binary32 bits
 */
static uint32_t prv_d2f_bits(uint64_t bits) {
    uint32_t res;
    uint32_t sign = (uint32_t)(bits >> 32) & F32_SIGN;
    int32_t exp = (int32_t)((bits >> 52) & F64_EXP_MAX);

    if (prv_is_nan64(bits) != 0U) {
        res = F32_DEF_NAN;
    } else if (exp == (int32_t)F64_EXP_MAX) {
        res = sign | (F32_EXP_MAX << 23);
    } else if ((bits << 1) == 0U) {
        res = sign;
    } else {
        /* Working significand: leading 1 at bit 30, 7 bits below the result's last bit
           (a binary64 subnormal is far below the binary32 range and only sets sticky). */
        res = prv_pack32(sign, exp - ((F64_BIAS - F32_BIAS) + 1),
                         (uint32_t)prv_shr_jam(bits & F64_FRAC, 22) | ((exp != 0) ? 0x40000000U : 0U));
    }
    return res;
}

/**
 * \brief           Truncate binary64 toward zero into a saturated integer range
 * \param[in]       bits: binary64 bits
 * \param[in]       is_signed: 1 for int32, 0 for uint32
 * \return          the integer, as `uint32_t` bits
 */
static uint32_t prv_d2int(uint64_t bits, uint32_t is_signed) {
    uint32_t res;
    uint32_t neg = (uint32_t)(bits >> 63);
    uint64_t sig = (bits & F64_FRAC) | F64_HIDDEN;
    int32_t exp = (int32_t)((bits >> 52) & F64_EXP_MAX) - F64_BIAS; /* value = sig * 2^(exp - 52) */
    uint32_t limit = (is_signed != 0U) ? 31U : 32U;

    if (prv_is_nan64(bits) != 0U) {
        res = 0U;
    } else if (exp < 0) {
        res = 0U; /* |x| < 1, and zero / subnormals */
    } else if ((neg != 0U) && (is_signed == 0U)) {
        res = 0U; /* negative -> unsigned */
    } else if ((uint32_t)exp >= limit) {
        res = (is_signed == 0U) ? 0xFFFFFFFFU : ((neg != 0U) ? 0x80000000U : 0x7FFFFFFFU);
    } else {
        res = (uint32_t)(sig >> (52 - exp));
        res = (neg != 0U) ? (0U - res) : res;
    }
    return res;
}

/**
 * \brief           Truncate binary32 toward zero into a saturated integer range
 * \param[in]       bits: binary32 bits
 * \param[in]       is_signed: 1 for int32, 0 for uint32
 * \return          the integer, as `uint32_t` bits
 */
static uint32_t prv_f2int(uint32_t bits, uint32_t is_signed) {
    uint32_t res;
    uint32_t neg = bits >> 31;
    uint32_t sig = (bits & F32_FRAC) | F32_HIDDEN;
    int32_t exp = (int32_t)((bits >> 23) & F32_EXP_MAX) - F32_BIAS; /* value = sig * 2^(exp - 23) */
    uint32_t limit = (is_signed != 0U) ? 31U : 32U;

    if (prv_is_nan32(bits) != 0U) {
        res = 0U;
    } else if (exp < 0) {
        res = 0U; /* |x| < 1, and zero / subnormals */
    } else if ((neg != 0U) && (is_signed == 0U)) {
        res = 0U; /* negative -> unsigned */
    } else if ((uint32_t)exp >= limit) {
        res = (is_signed == 0U) ? 0xFFFFFFFFU : ((neg != 0U) ? 0x80000000U : 0x7FFFFFFFU);
    } else {
        res = (exp >= 23) ? (sig << (uint32_t)(exp - 23)) : (sig >> (uint32_t)(23 - exp));
        res = (neg != 0U) ? (0U - res) : res;
    }
    return res;
}

/**
 * \brief           Convert an integer magnitude to binary32, rounded to nearest-even
 * \param[in]       sign: `F32_SIGN` or 0
 * \param[in]       mag: magnitude
 * \return          binary32 bits
 */
static uint32_t prv_int2f(uint32_t sign, uint32_t mag) {
    return (mag == 0U) ? sign : prv_norm_pack32(sign, F32_BIAS + 30, mag);
}

/**
 * \brief           Convert an integer magnitude to binary64 (exact up to 53 bits, else rounded)
 * \param[in]       sign: `F64_SIGN` or 0
 * \param[in]       mag: magnitude
 * \return          binary64 bits
 */
static uint64_t prv_int2d(uint64_t sign, uint64_t mag) {
    return (mag == 0U) ? sign : prv_norm_pack64(sign, F64_BIAS + 62, mag);
}

/* ------------------------------------------------------------------------- */
/* Run-time ABI entry points                                                  */
/* ------------------------------------------------------------------------- */

double __aeabi_dadd(double lhs, double rhs);
double __aeabi_dsub(double lhs, double rhs);
double __aeabi_dmul(double lhs, double rhs);
double __aeabi_ddiv(double lhs, double rhs);
double __aeabi_drsub(double lhs, double rhs);
int __aeabi_dcmpeq(double lhs, double rhs);
int __aeabi_dcmplt(double lhs, double rhs);
int __aeabi_dcmple(double lhs, double rhs);
int __aeabi_dcmpge(double lhs, double rhs);
int __aeabi_dcmpgt(double lhs, double rhs);
int __aeabi_dcmpun(double lhs, double rhs);
float __aeabi_fadd(float lhs, float rhs);
float __aeabi_fsub(float lhs, float rhs);
float __aeabi_frsub(float lhs, float rhs);
float __aeabi_fmul(float lhs, float rhs);
float __aeabi_fdiv(float lhs, float rhs);
int __aeabi_fcmpeq(float lhs, float rhs);
int __aeabi_fcmplt(float lhs, float rhs);
int __aeabi_fcmple(float lhs, float rhs);
int __aeabi_fcmpgt(float lhs, float rhs);
int __aeabi_fcmpge(float lhs, float rhs);
int __aeabi_fcmpun(float lhs, float rhs);
double __aeabi_f2d(float value);
float __aeabi_d2f(double value);
int __aeabi_d2iz(double value);
unsigned __aeabi_d2uiz(double value);
int __aeabi_f2iz(float value);
unsigned __aeabi_f2uiz(float value);
double __aeabi_i2d(int value);
double __aeabi_ui2d(unsigned value);
double __aeabi_l2d(long long value);
double __aeabi_ul2d(unsigned long long value);
float __aeabi_i2f(int value);
float __aeabi_ui2f(unsigned value);

/** \brief binary64 add \param[in] lhs: a \param[in] rhs: b \return a + b */
double __aeabi_dadd(double lhs, double rhs) { return prv_bits2d(prv_add64(prv_d2bits(lhs), prv_d2bits(rhs))); }

/** \brief binary64 subtract \param[in] lhs: a \param[in] rhs: b \return a - b */
double __aeabi_dsub(double lhs, double rhs) {
    return prv_bits2d(prv_add64(prv_d2bits(lhs), prv_d2bits(rhs) ^ F64_SIGN));
}

/** \brief binary64 reverse subtract \param[in] lhs: a \param[in] rhs: b \return b - a */
double __aeabi_drsub(double lhs, double rhs) {
    return prv_bits2d(prv_add64(prv_d2bits(rhs), prv_d2bits(lhs) ^ F64_SIGN));
}

/** \brief binary64 multiply \param[in] lhs: a \param[in] rhs: b \return a * b */
double __aeabi_dmul(double lhs, double rhs) { return prv_bits2d(prv_mul64(prv_d2bits(lhs), prv_d2bits(rhs))); }

/** \brief binary64 divide \param[in] lhs: a \param[in] rhs: b \return a / b */
double __aeabi_ddiv(double lhs, double rhs) { return prv_bits2d(prv_div64(prv_d2bits(lhs), prv_d2bits(rhs))); }

/** \brief binary64 a == b \param[in] lhs: a \param[in] rhs: b \return 1 if a == b, else 0 */
int __aeabi_dcmpeq(double lhs, double rhs) { return (prv_cmp64(prv_d2bits(lhs), prv_d2bits(rhs)) == 0) ? 1 : 0; }

/** \brief binary64 a >= b \param[in] lhs: a \param[in] rhs: b \return 1 if a >= b, else 0 */
int __aeabi_dcmpge(double lhs, double rhs) {
    int32_t cmp = prv_cmp64(prv_d2bits(lhs), prv_d2bits(rhs));

    return ((cmp == 1) || (cmp == 0)) ? 1 : 0;
}

/** \brief binary64 unordered \param[in] lhs: a \param[in] rhs: b \return 1 if a or b is a NaN, else 0 */
int __aeabi_dcmpun(double lhs, double rhs) { return (prv_cmp64(prv_d2bits(lhs), prv_d2bits(rhs)) == 2) ? 1 : 0; }

/** \brief binary64 a < b \param[in] lhs: a \param[in] rhs: b \return 1 if a < b, else 0 */
int __aeabi_dcmplt(double lhs, double rhs) { return (prv_cmp64(prv_d2bits(lhs), prv_d2bits(rhs)) == -1) ? 1 : 0; }

/** \brief binary64 a <= b \param[in] lhs: a \param[in] rhs: b \return 1 if a <= b, else 0 */
int __aeabi_dcmple(double lhs, double rhs) {
    int32_t cmp = prv_cmp64(prv_d2bits(lhs), prv_d2bits(rhs));

    return ((cmp == -1) || (cmp == 0)) ? 1 : 0;
}

/** \brief binary64 a > b \param[in] lhs: a \param[in] rhs: b \return 1 if a > b, else 0 */
int __aeabi_dcmpgt(double lhs, double rhs) { return (prv_cmp64(prv_d2bits(lhs), prv_d2bits(rhs)) == 1) ? 1 : 0; }

/** \brief binary32 add \param[in] lhs: a \param[in] rhs: b \return a + b */
float __aeabi_fadd(float lhs, float rhs) { return prv_bits2f(prv_add32(prv_f2bits(lhs), prv_f2bits(rhs))); }

/** \brief binary32 subtract \param[in] lhs: a \param[in] rhs: b \return a - b */
float __aeabi_fsub(float lhs, float rhs) { return prv_bits2f(prv_add32(prv_f2bits(lhs), prv_f2bits(rhs) ^ F32_SIGN)); }

/** \brief binary32 reverse subtract \param[in] lhs: a \param[in] rhs: b \return b - a */
float __aeabi_frsub(float lhs, float rhs) { return __aeabi_fsub(rhs, lhs); }

/** \brief binary32 multiply \param[in] lhs: a \param[in] rhs: b \return a * b */
float __aeabi_fmul(float lhs, float rhs) { return prv_bits2f(prv_mul32(prv_f2bits(lhs), prv_f2bits(rhs))); }

/** \brief binary32 divide \param[in] lhs: a \param[in] rhs: b \return a / b */
float __aeabi_fdiv(float lhs, float rhs) { return prv_bits2f(prv_div32(prv_f2bits(lhs), prv_f2bits(rhs))); }

/**
 * \brief           Three-way binary32 compare
 * \param[in]       lhs: a
 * \param[in]       rhs: b
 * \return          as \ref prv_cmp64
 */
static int32_t prv_cmp32(float lhs, float rhs) {
    int32_t res;
    uint32_t abits = prv_f2bits(lhs);
    uint32_t bbits = prv_f2bits(rhs);
    uint32_t akey;
    uint32_t bkey;

    if ((prv_is_nan32(abits) != 0U) || (prv_is_nan32(bbits) != 0U)) {
        res = 2;
    } else if (((abits | bbits) << 1) == 0U) {
        res = 0;
    } else {
        /* Map sign-magnitude to an unsigned order. */
        akey = ((abits & F32_SIGN) != 0U) ? ~abits : (abits | F32_SIGN);
        bkey = ((bbits & F32_SIGN) != 0U) ? ~bbits : (bbits | F32_SIGN);
        res = (akey < bkey) ? -1 : ((akey > bkey) ? 1 : 0);
    }
    return res;
}

/** \brief binary32 a == b \param[in] lhs: a \param[in] rhs: b \return 1 if a == b, else 0 */
int __aeabi_fcmpeq(float lhs, float rhs) { return (prv_cmp32(lhs, rhs) == 0) ? 1 : 0; }

/** \brief binary32 unordered \param[in] lhs: a \param[in] rhs: b \return 1 if a or b is a NaN, else 0 */
int __aeabi_fcmpun(float lhs, float rhs) { return (prv_cmp32(lhs, rhs) == 2) ? 1 : 0; }

/** \brief binary32 a < b \param[in] lhs: a \param[in] rhs: b \return 1 if a < b, else 0 */
int __aeabi_fcmplt(float lhs, float rhs) { return (prv_cmp32(lhs, rhs) == -1) ? 1 : 0; }

/** \brief binary32 a <= b \param[in] lhs: a \param[in] rhs: b \return 1 if a <= b, else 0 */
int __aeabi_fcmple(float lhs, float rhs) {
    int32_t cmp = prv_cmp32(lhs, rhs);

    return ((cmp == -1) || (cmp == 0)) ? 1 : 0;
}

/** \brief binary32 a > b \param[in] lhs: a \param[in] rhs: b \return 1 if a > b, else 0 */
int __aeabi_fcmpgt(float lhs, float rhs) { return (prv_cmp32(lhs, rhs) == 1) ? 1 : 0; }

/** \brief binary32 a >= b \param[in] lhs: a \param[in] rhs: b \return 1 if a >= b, else 0 */
int __aeabi_fcmpge(float lhs, float rhs) {
    int32_t cmp = prv_cmp32(lhs, rhs);

    return ((cmp == 1) || (cmp == 0)) ? 1 : 0;
}

/** \brief Widen float to double \param[in] value: x \return (double)x */
double __aeabi_f2d(float value) { return prv_bits2d(prv_f2d_bits(prv_f2bits(value))); }

/** \brief Narrow double to float \param[in] value: x \return (float)x, rounded to nearest-even */
float __aeabi_d2f(double value) { return prv_bits2f(prv_d2f_bits(prv_d2bits(value))); }

/** \brief double to int32, toward zero, saturated \param[in] value: x \return (int)x */
int __aeabi_d2iz(double value) { return (int)(int32_t)prv_d2int(prv_d2bits(value), 1U); }

/** \brief double to uint32, toward zero, saturated \param[in] value: x \return (unsigned)x */
unsigned __aeabi_d2uiz(double value) { return (unsigned)prv_d2int(prv_d2bits(value), 0U); }

/** \brief float to int32, toward zero, saturated \param[in] value: x \return (int)x */
int __aeabi_f2iz(float value) { return (int)(int32_t)prv_f2int(prv_f2bits(value), 1U); }

/** \brief float to uint32, toward zero, saturated \param[in] value: x \return (unsigned)x */
unsigned __aeabi_f2uiz(float value) { return (unsigned)prv_f2int(prv_f2bits(value), 0U); }

/** \brief int32 to double (exact) \param[in] value: x \return (double)x */
double __aeabi_i2d(int value) {
    return prv_bits2d(prv_int2d((value < 0) ? F64_SIGN : 0U, (value < 0) ? (0U - (uint32_t)value) : (uint32_t)value));
}

/** \brief uint32 to double (exact) \param[in] value: x \return (double)x */
double __aeabi_ui2d(unsigned value) { return prv_bits2d(prv_int2d(0U, (uint32_t)value)); }

/** \brief int64 to double, rounded to nearest-even \param[in] value: x \return (double)x */
double __aeabi_l2d(long long value) {
    return prv_bits2d(prv_int2d((value < 0) ? F64_SIGN : 0U, (value < 0) ? (0U - (uint64_t)value) : (uint64_t)value));
}

/** \brief uint64 to double, rounded to nearest-even \param[in] value: x \return (double)x */
double __aeabi_ul2d(unsigned long long value) { return prv_bits2d(prv_int2d(0U, (uint64_t)value)); }

/** \brief int32 to float, rounded to nearest-even \param[in] value: x \return (float)x */
float __aeabi_i2f(int value) {
    return prv_bits2f(prv_int2f((value < 0) ? F32_SIGN : 0U, (value < 0) ? (0U - (uint32_t)value) : (uint32_t)value));
}

/** \brief uint32 to float, rounded to nearest-even \param[in] value: x \return (float)x */
float __aeabi_ui2f(unsigned value) { return prv_bits2f(prv_int2f(0U, (uint32_t)value)); }

#endif /* VCU_SOFTFLOAT_COMPACT */
