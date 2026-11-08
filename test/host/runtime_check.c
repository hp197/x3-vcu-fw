/**
 * \file            runtime_check.c
 * \brief           Host check of the run-time helpers against the host's IEEE-754 hardware.
 * \note            Builds src/softfloat.c and src/runtime.c for the host and compares every
 *                      helper with the host's own arithmetic. The host must round to nearest-even
 *                      with subnormals, and its float-to-integer conversions must saturate with
 *                      NaN -> 0; arm64 (Apple silicon) does all of this. See test/README.md.
 *                      - unary helpers: all 2^32 inputs;
 *                      - `x * 3`, `1 / x`, `x + 1`, `x * 2^-148`: all 2^32 values of x;
 *                      - binary helpers: random pairs drawn from any bit pattern, moderate,
 *                        subnormal, tiny, huge, special and few-bit (tie) values, plus
 *                        near-equal pairs for cancellation;
 *                      - `__udivmoddi4`: edge values and random pairs.
 *                      A NaN result only has to be a NaN (the helpers return the default NaN).
 *
 * \copyright       Copyright (c) 2026 hp197. Licensed under CC BY-NC-SA 4.0.
 *                      SPDX-License-Identifier: CC-BY-NC-SA-4.0
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHK_RANDOM_PAIRS64 16000000L /*!< double pairs                */
#define CHK_RANDOM_PAIRS32 30000000L /*!< float pairs                 */
#define CHK_RANDOM_DIV64   20000000L /*!< __udivmoddi4 random pairs   */
#define CHK_MAX_REPORTS    12L       /*!< failures printed in full    */

double __aeabi_dadd(double lhs, double rhs);
double __aeabi_dsub(double lhs, double rhs);
double __aeabi_drsub(double lhs, double rhs);
double __aeabi_dmul(double lhs, double rhs);
double __aeabi_ddiv(double lhs, double rhs);
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
int __aeabi_fcmpge(float lhs, float rhs);
int __aeabi_fcmpgt(float lhs, float rhs);
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
uint64_t __udivmoddi4(uint64_t num, uint64_t den, uint64_t* rem);

static uint64_t prv_seed = 0x243F6A8885A308D3ULL;
static long prv_bad;

/** \brief xorshift64 \return next pseudo-random value */
static uint64_t prv_rnd(void) {
    prv_seed ^= prv_seed << 13;
    prv_seed ^= prv_seed >> 7;
    prv_seed ^= prv_seed << 17;
    return prv_seed;
}

/** \brief Bits of a double \param[in] value: the double \return IEEE-754 bits */
static uint64_t prv_dbits(double value) {
    uint64_t bits;

    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/** \brief Double from bits \param[in] bits: IEEE-754 bits \return the double */
static double prv_bitsd(uint64_t bits) {
    double value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/** \brief Bits of a float \param[in] value: the float \return IEEE-754 bits */
static uint32_t prv_fbits(float value) {
    uint32_t bits;

    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/** \brief Float from bits \param[in] bits: IEEE-754 bits \return the float */
static float prv_bitsf(uint32_t bits) {
    float value;

    memcpy(&value, &bits, sizeof value);
    return value;
}

/**
 * \brief           Count (and print the first few) double mismatches
 * \param[in]       name: helper name
 * \param[in]       got: helper result
 * \param[in]       want: host result
 * \param[in]       lhs: first operand (for the report)
 * \param[in]       rhs: second operand (for the report)
 */
static void prv_chk_d(const char* name, double got, double want, double lhs, double rhs) {
    if (isnan(want) ? !isnan(got) : (prv_dbits(got) != prv_dbits(want))) {
        if (prv_bad < CHK_MAX_REPORTS) {
            printf("%s FAIL a=%016llx b=%016llx got %016llx want %016llx\n", name, (unsigned long long)prv_dbits(lhs),
                   (unsigned long long)prv_dbits(rhs), (unsigned long long)prv_dbits(got),
                   (unsigned long long)prv_dbits(want));
        }
        prv_bad++;
    }
}

/**
 * \brief           Count (and print the first few) float mismatches
 * \param[in]       name: helper name
 * \param[in]       got: helper result
 * \param[in]       want: host result
 * \param[in]       lhs: first operand (for the report)
 * \param[in]       rhs: second operand (for the report)
 */
static void prv_chk_f(const char* name, float got, float want, float lhs, float rhs) {
    if (isnan(want) ? !isnan(got) : (prv_fbits(got) != prv_fbits(want))) {
        if (prv_bad < CHK_MAX_REPORTS) {
            printf("%s FAIL a=%08x b=%08x got %08x want %08x\n", name, prv_fbits(lhs), prv_fbits(rhs), prv_fbits(got),
                   prv_fbits(want));
        }
        prv_bad++;
    }
}

/**
 * \brief           Count a mismatch of an integer result
 * \param[in]       name: helper name
 * \param[in]       ok: 1 if the result matched
 */
static void prv_chk_i(const char* name, int ok) {
    if (!ok) {
        if (prv_bad < CHK_MAX_REPORTS) {
            printf("%s FAIL\n", name);
        }
        prv_bad++;
    }
}

/**
 * \brief           A random double from one of eight classes
 * \param[in]       kind: class 0..7
 * \return          the double
 */
static double prv_gen_d(int kind) {
    static const uint64_t special[] = {
        0U, 0x8000000000000000ULL, 0x7FF0000000000000ULL, 0x7FF8000000000000ULL, 0x3FF0000000000000ULL,
        1U, 0x000FFFFFFFFFFFFFULL, 0x0010000000000000ULL, 0x7FEFFFFFFFFFFFFFULL, 0x4000000000000000ULL,
    };
    uint64_t rnd = prv_rnd();
    uint64_t sign_frac = rnd & 0x800FFFFFFFFFFFFFULL;
    uint64_t bits;

    switch (kind) {
        case 0: bits = rnd; break;
        case 1: bits = sign_frac | ((uint64_t)(1003U + (prv_rnd() % 40U)) << 52); break;
        case 2: bits = sign_frac; break;
        case 3: bits = sign_frac | ((prv_rnd() % 60U) << 52); break;
        case 4: bits = sign_frac | ((uint64_t)(0x7FEU - (prv_rnd() % 60U)) << 52); break;
        case 5: bits = special[prv_rnd() % 10U] ^ (((prv_rnd() & 1U) != 0U) ? 0x8000000000000000ULL : 0U); break;
        case 6: bits = (rnd & 0x8000000000FFFFFFULL) | 0x3FF0000000000000ULL | ((prv_rnd() & 0xFFU) << 44); break;
        default: bits = prv_dbits((double)(int32_t)rnd); break;
    }
    return prv_bitsd(bits);
}

/**
 * \brief           A random float from one of seven classes
 * \param[in]       kind: class 0..6
 * \return          the float
 */
static float prv_gen_f(int kind) {
    static const uint32_t special[] = {
        0U, 0x80000000U, 0x7F800000U, 0x7FC00000U, 0x3F800000U, 1U, 0x007FFFFFU, 0x00800000U, 0x7F7FFFFFU, 0x40000000U,
    };
    uint32_t rnd = (uint32_t)prv_rnd();
    uint32_t sign_frac = rnd & 0x807FFFFFU;
    uint32_t bits;

    switch (kind) {
        case 0: bits = rnd; break;
        case 1: bits = sign_frac | ((112U + (uint32_t)(prv_rnd() % 30U)) << 23); break;
        case 2: bits = sign_frac; break;
        case 3: bits = sign_frac | ((uint32_t)(prv_rnd() % 30U) << 23); break;
        case 4: bits = sign_frac | ((0xFEU - (uint32_t)(prv_rnd() % 30U)) << 23); break;
        case 5: bits = special[prv_rnd() % 10U] ^ (((prv_rnd() & 1U) != 0U) ? 0x80000000U : 0U); break;
        default: bits = (rnd & 0x800000FFU) | 0x3F800000U | ((uint32_t)(prv_rnd() & 0x3FU) << 17); break;
    }
    return prv_bitsf(bits);
}

/** \brief Random double pairs through every double helper */
static void prv_check_double(void) {
    long idx;

    for (idx = 0; idx < CHK_RANDOM_PAIRS64; idx++) {
        int k1 = (int)(prv_rnd() % 8U);
        int k2 = ((prv_rnd() & 3U) != 0U) ? k1 : (int)(prv_rnd() % 8U);
        double lhs = prv_gen_d(k1);
        double rhs = prv_gen_d(k2);
        volatile double vl;
        volatile double vr;

        if (((idx & 3) == 0) && (k1 == 1)) { /* near-equal: cancellation */
            rhs = lhs * (1.0 + (prv_bitsd(0x3CB0000000000000ULL) * (double)((int)(prv_rnd() % 5U) - 2)));
        }
        vl = lhs;
        vr = rhs;
        prv_chk_d("dadd", __aeabi_dadd(lhs, rhs), vl + vr, lhs, rhs);
        prv_chk_d("dsub", __aeabi_dsub(lhs, rhs), vl - vr, lhs, rhs);
        prv_chk_d("drsub", __aeabi_drsub(lhs, rhs), vr - vl, lhs, rhs);
        prv_chk_d("dmul", __aeabi_dmul(lhs, rhs), vl * vr, lhs, rhs);
        prv_chk_d("ddiv", __aeabi_ddiv(lhs, rhs), vl / vr, lhs, rhs);
        prv_chk_i("dcmp", (__aeabi_dcmpeq(lhs, rhs) == (vl == vr)) && (__aeabi_dcmplt(lhs, rhs) == (vl < vr))
                              && (__aeabi_dcmple(lhs, rhs) == (vl <= vr)) && (__aeabi_dcmpge(lhs, rhs) == (vl >= vr))
                              && (__aeabi_dcmpgt(lhs, rhs) == (vl > vr))
                              && (__aeabi_dcmpun(lhs, rhs) == (isnan(vl) || isnan(vr))));
        prv_chk_f("d2f", __aeabi_d2f(lhs), (float)vl, 0.0f, 0.0f);
        prv_chk_i("d2iz", __aeabi_d2iz(lhs) == (isnan(lhs) ? 0 : (int)vl));
        prv_chk_i("d2uiz", __aeabi_d2uiz(lhs) == (isnan(lhs) ? 0U : (unsigned)vl));
    }
}

/** \brief Random float pairs through every float helper */
static void prv_check_float(void) {
    long idx;

    for (idx = 0; idx < CHK_RANDOM_PAIRS32; idx++) {
        int k1 = (int)(prv_rnd() % 7U);
        int k2 = ((prv_rnd() & 3U) != 0U) ? k1 : (int)(prv_rnd() % 7U);
        float lhs = prv_gen_f(k1);
        float rhs = prv_gen_f(k2);
        volatile float vl;
        volatile float vr;

        if (((idx & 3) == 0) && (k1 == 1)) { /* a few ulps apart: cancellation */
            rhs = prv_bitsf(prv_fbits(lhs) + (uint32_t)(prv_rnd() % 5U) - 2U);
        }
        vl = lhs;
        vr = rhs;
        prv_chk_f("fadd", __aeabi_fadd(lhs, rhs), vl + vr, lhs, rhs);
        prv_chk_f("fsub", __aeabi_fsub(lhs, rhs), vl - vr, lhs, rhs);
        prv_chk_f("frsub", __aeabi_frsub(lhs, rhs), vr - vl, lhs, rhs);
        prv_chk_f("fmul", __aeabi_fmul(lhs, rhs), vl * vr, lhs, rhs);
        prv_chk_f("fdiv", __aeabi_fdiv(lhs, rhs), vl / vr, lhs, rhs);
        prv_chk_i("fcmp", (__aeabi_fcmpeq(lhs, rhs) == (vl == vr)) && (__aeabi_fcmplt(lhs, rhs) == (vl < vr))
                              && (__aeabi_fcmple(lhs, rhs) == (vl <= vr)) && (__aeabi_fcmpge(lhs, rhs) == (vl >= vr))
                              && (__aeabi_fcmpgt(lhs, rhs) == (vl > vr))
                              && (__aeabi_fcmpun(lhs, rhs) == (isnan(vl) || isnan(vr))));
    }
}

/** \brief All 2^32 inputs of the unary helpers, and four one-operand float sweeps */
static void prv_check_exhaustive(void) {
    volatile float three = 3.0f;
    volatile float one = 1.0f;
    volatile float tiny = prv_bitsf(0x00000003U);
    uint64_t word;

    for (word = 0U; word <= 0xFFFFFFFFULL; word++) {
        uint32_t bits = (uint32_t)word;
        float val = prv_bitsf(bits);
        volatile float vf = val;
        volatile int vi = (int)bits;
        volatile unsigned vu = bits;

        prv_chk_d("f2d", __aeabi_f2d(val), (double)vf, 0.0, 0.0);
        prv_chk_i("f2iz", __aeabi_f2iz(val) == (isnan(val) ? 0 : (int)vf));
        prv_chk_i("f2uiz", __aeabi_f2uiz(val) == (isnan(val) ? 0U : (unsigned)vf));
        prv_chk_f("i2f", __aeabi_i2f((int)bits), (float)vi, val, val);
        prv_chk_f("ui2f", __aeabi_ui2f(bits), (float)vu, val, val);
        prv_chk_d("i2d", __aeabi_i2d((int)bits), (double)vi, 0.0, 0.0);
        prv_chk_d("ui2d", __aeabi_ui2d(bits), (double)vu, 0.0, 0.0);
        prv_chk_f("fmul*3", __aeabi_fmul(val, 3.0f), vf * three, val, 3.0f);
        prv_chk_f("1/fdiv", __aeabi_fdiv(1.0f, val), one / vf, 1.0f, val);
        prv_chk_f("fadd+1", __aeabi_fadd(val, 1.0f), vf + one, val, 1.0f);
        prv_chk_f("fmul*tiny", __aeabi_fmul(val, prv_bitsf(0x00000003U)), vf * tiny, val, tiny);
    }
}

/** \brief 64-bit integer conversions and __udivmoddi4 */
static void prv_check_int64(void) {
    static const uint64_t edge[] = {
        1U,
        2U,
        3U,
        7U,
        8U,
        0xFFU,
        0x100U,
        0xFFFFFFFFULL,
        0x100000000ULL,
        0x7FFFFFFFFFFFFFFFULL,
        0x8000000000000000ULL,
        0xFFFFFFFFFFFFFFFFULL,
        0xFFFFFFFFFFFFFFFEULL,
        12000000U,
        144000000U,
        8000000U,
    };
    uint64_t rem;
    uint32_t idx;
    uint32_t jdx;
    long cnt;

    for (idx = 0U; idx < (sizeof(edge) / sizeof(edge[0])); idx++) {
        for (jdx = 0U; jdx < (sizeof(edge) / sizeof(edge[0])); jdx++) {
            prv_chk_i("udivmoddi4", (__udivmoddi4(edge[idx], edge[jdx], &rem) == (edge[idx] / edge[jdx]))
                                        && (rem == (edge[idx] % edge[jdx])));
        }
    }
    prv_chk_i("udivmoddi4/0", (__udivmoddi4(5U, 0U, &rem) == 0U) && (rem == 5U));
    for (cnt = 0; cnt < CHK_RANDOM_DIV64; cnt++) {
        uint64_t num = prv_rnd() >> (((cnt & 1) != 0) ? (prv_rnd() % 64U) : 0U);
        uint64_t den = prv_rnd() >> (prv_rnd() % 64U);
        long long sll = (long long)(prv_rnd() >> (prv_rnd() % 64U));
        volatile long long vll;
        volatile unsigned long long vull = num;

        den = (den == 0U) ? 1U : den;
        prv_chk_i("udivmoddi4", (__udivmoddi4(num, den, &rem) == (num / den)) && (rem == (num % den)));
        sll = ((prv_rnd() & 1U) != 0U) ? -sll : sll;
        vll = sll;
        prv_chk_d("l2d", __aeabi_l2d(sll), (double)vll, 0.0, 0.0);
        prv_chk_d("ul2d", __aeabi_ul2d(num), (double)vull, 0.0, 0.0);
    }
}

/**
 * \brief           Run every check
 * \return          0 when every helper matched the host
 */
int main(void) {
    prv_check_double();
    printf("double pairs: %ld mismatches\n", prv_bad);
    prv_check_float();
    printf("float pairs: %ld mismatches\n", prv_bad);
    prv_check_int64();
    printf("64-bit integer: %ld mismatches\n", prv_bad);
    prv_check_exhaustive();
    printf("exhaustive 2^32: %ld mismatches\n", prv_bad);
    return (prv_bad == 0) ? 0 : 1;
}
