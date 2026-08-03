/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Multiply, multiply-accumulate and vector operations.
 *
 * Semantics are from the Blackfin Processor Programming Reference rev 2.2:
 * chapter 2 "Computational Units" for the multiplier, the 40-bit
 * accumulators, saturation and rounding; chapter 15 for the per-option
 * tables 15-2 to 15-5; chapter 19 for the vector forms; and appendix D
 * "Numeric Formats" for the fractional and integer conventions.
 */

/*
 * Multiplier option encoding, the mmod field of table C-17. The values not
 * listed here are reserved and never reach a helper: the translator refuses
 * them.
 */
enum {
    BFIN_MMOD_DEFAULT = 0,
    BFIN_MMOD_S2RND = 1,
    BFIN_MMOD_T = 2,
    BFIN_MMOD_W32 = 3,
    BFIN_MMOD_FU = 4,
    BFIN_MMOD_TFU = 6,
    BFIN_MMOD_IS = 8,
    BFIN_MMOD_ISS2 = 9,
    BFIN_MMOD_IH = 11,
    BFIN_MMOD_IU = 12,
};

/*
 * An accumulator is 40 bits held right aligned in env->a[n] with bits 63 to
 * 40 zero, so a signed reading has to extend bit 39 by hand. Keeping the
 * spare bits clear rather than sign extending them matters because
 * write_creg() deposits A0.X and A1.X without touching them.
 */
#define BFIN_ACC_MASK 0xffffffffffULL

static int64_t mac_acc_signed(uint64_t a)
{
    return (int64_t)(a << 24) >> 24;
}

static uint32_t mac_sat_s16(int64_t v, bool *sat)
{
    if (v > INT16_MAX) {
        *sat = true;
        return (uint16_t)INT16_MAX;
    }
    if (v < INT16_MIN) {
        *sat = true;
        return (uint16_t)INT16_MIN;
    }
    return (uint16_t)v;
}

static uint32_t mac_sat_u16(uint64_t v, bool *sat)
{
    if (v > UINT16_MAX) {
        *sat = true;
        return UINT16_MAX;
    }
    return v;
}

static uint32_t mac_sat_s32(int64_t v, bool *sat)
{
    if (v > INT32_MAX) {
        *sat = true;
        return (uint32_t)INT32_MAX;
    }
    if (v < INT32_MIN) {
        *sat = true;
        return (uint32_t)INT32_MIN;
    }
    return (uint32_t)v;
}

static uint32_t mac_sat_u32(uint64_t v, bool *sat)
{
    if (v > UINT32_MAX) {
        *sat = true;
        return UINT32_MAX;
    }
    return v;
}

/* (FU), (TFU) and (IU) read both operands and the accumulator as unsigned. */
static bool mac_unsigned(int mmod)
{
    return mmod == BFIN_MMOD_FU || mmod == BFIN_MMOD_TFU ||
           mmod == BFIN_MMOD_IU;
}

/*
 * A product of two signed fractions carries two sign bits, one of them
 * redundant, so the 32-bit product is shifted left by one before it reaches
 * the accumulator: appendix D "Fractional Mode And Integer Mode" explains
 * why and figure 2-5 shows the resulting alignment. Unsigned fractional,
 * integer and mixed multiplies keep the product as it is.
 */
static bool mac_shift_correct(int mmod, bool mm)
{
    if (mm) {
        return false;
    }
    return mmod == BFIN_MMOD_DEFAULT || mmod == BFIN_MMOD_S2RND ||
           mmod == BFIN_MMOD_T || mmod == BFIN_MMOD_W32;
}

static uint32_t mac_multiply(uint32_t s0, uint32_t s1, int mmod, bool mm,
                             bool *sat)
{
    uint32_t p;

    if (mm) {
        /* (M): operand 0 is signed and operand 1 unsigned. MAC1 only. */
        p = (uint32_t)((int32_t)(int16_t)s0 * (int32_t)(uint16_t)s1);
    } else if (mac_unsigned(mmod)) {
        p = (uint32_t)(uint16_t)s0 * (uint32_t)(uint16_t)s1;
    } else {
        p = (uint32_t)((int32_t)(int16_t)s0 * (int32_t)(int16_t)s1);
    }

    if (mac_shift_correct(mmod, mm)) {
        if (p == 0x40000000u) {
            /*
             * 0x8000 * 0x8000 is +1.0, which 1.31 cannot represent. Table
             * 15-2 saturates it to the largest positive fraction instead of
             * letting the shift turn it into -1.0.
             */
            *sat = true;
            p = 0x7fffffffu;
        } else {
            p <<= 1;
        }
    }
    return p;
}

static uint64_t mac_accumulate(uint64_t acc, uint32_t prod, int op, int mmod,
                               bool *sat)
{
    int64_t v, lo, hi;

    if (mac_unsigned(mmod)) {
        uint64_t u = acc & BFIN_ACC_MASK;

        switch (op) {
        case 0:
            return prod;
        case 1:
            u += prod;
            if (u > BFIN_ACC_MASK) {
                *sat = true;
                u = BFIN_ACC_MASK;
            }
            return u;
        default:
            if (prod > u) {
                *sat = true;
                return 0;
            }
            return u - prod;
        }
    }

    v = (int32_t)prod;
    switch (op) {
    case 0:
        break;
    case 1:
        v += mac_acc_signed(acc);
        break;
    default:
        v = mac_acc_signed(acc) - v;
        break;
    }

    /*
     * (W32) and (IH) hold 1.31 and 32.0 accumulators and so saturate at bit
     * 31; every other signed option keeps the full 9.31 or 40.0 range.
     */
    if (mmod == BFIN_MMOD_W32 || mmod == BFIN_MMOD_IH) {
        lo = INT32_MIN;
        hi = INT32_MAX;
    } else {
        lo = -(INT64_C(1) << 39);
        hi = (INT64_C(1) << 39) - 1;
    }
    if (v < lo) {
        *sat = true;
        v = lo;
    } else if (v > hi) {
        *sat = true;
        v = hi;
    }
    return (uint64_t)v & BFIN_ACC_MASK;
}

/*
 * Round at the boundary between bits 15 and 16. Biased rounding just carries
 * one into bit 16; unbiased rounding additionally forces bit 16 to zero at
 * the exact midpoint, so that odd values round up and even values round
 * down and the mean error is zero. See figure 2-8 and table 2-7.
 */
static int64_t mac_round(int64_t v, bool biased)
{
    int64_t r = v + 0x8000;

    if (!biased && (v & 0xffff) == 0x8000) {
        r &= ~(int64_t)0x10000;
    }
    return r;
}

/* Table 15-4: copy the accumulator into a 16-bit register half. */
static uint32_t mac_extract_half(CPUBfinState *env, uint64_t acc, int mmod,
                                 bool *sat)
{
    bool biased = FIELD_EX32(env->astat, ASTAT, RND_MOD);
    int64_t v;

    switch (mmod) {
    case BFIN_MMOD_IS:
        return mac_sat_s16(mac_acc_signed(acc), sat);
    case BFIN_MMOD_ISS2:
        return mac_sat_s16(mac_acc_signed(acc) * 2, sat);
    case BFIN_MMOD_IU:
        return mac_sat_u16(acc & BFIN_ACC_MASK, sat);
    case BFIN_MMOD_T:
        return mac_sat_s16(mac_acc_signed(acc) >> 16, sat);
    case BFIN_MMOD_TFU:
        return mac_sat_u16((acc & BFIN_ACC_MASK) >> 16, sat);
    case BFIN_MMOD_FU:
        v = mac_round(acc & BFIN_ACC_MASK, biased);
        return mac_sat_u16(v >> 16, sat);
    case BFIN_MMOD_IH:
        /*
         * Integer with high half extraction: round at bit 16, saturate what
         * is left to 32 bits and hand back the top half of that.
         */
        v = mac_sat_s32(mac_round(mac_acc_signed(acc), biased), sat);
        return (v >> 16) & 0xffff;
    case BFIN_MMOD_S2RND:
        v = mac_round(mac_acc_signed(acc) * 2, biased);
        return mac_sat_s16(v >> 16, sat);
    default:
        v = mac_round(mac_acc_signed(acc), biased);
        return mac_sat_s16(v >> 16, sat);
    }
}

/*
 * Table 15-5: copy the accumulator into a 32-bit register. These forms never
 * round, only saturate.
 */
static uint32_t mac_extract_word(uint64_t acc, int mmod, bool *sat)
{
    switch (mmod) {
    case BFIN_MMOD_FU:
    case BFIN_MMOD_TFU:
    case BFIN_MMOD_IU:
        return mac_sat_u32(acc & BFIN_ACC_MASK, sat);
    case BFIN_MMOD_S2RND:
    case BFIN_MMOD_ISS2:
        return mac_sat_s32(mac_acc_signed(acc) * 2, sat);
    default:
        return mac_sat_s32(mac_acc_signed(acc), sat);
    }
}

static void mac_set_av(CPUBfinState *env, int which, bool v)
{
    if (which) {
        env->astat = FIELD_DP32(env->astat, ASTAT, AV1, v);
        if (v) {
            env->astat = FIELD_DP32(env->astat, ASTAT, AV1S, 1);
        }
    } else {
        env->astat = FIELD_DP32(env->astat, ASTAT, AV0, v);
        if (v) {
            env->astat = FIELD_DP32(env->astat, ASTAT, AV0S, 1);
        }
    }
}

/* V_COPY is architecturally required to read back the same as V. */
static void mac_set_v(CPUBfinState *env, bool v)
{
    env->astat = FIELD_DP32(env->astat, ASTAT, V, v);
    env->astat = FIELD_DP32(env->astat, ASTAT, V_COPY, v);
    if (v) {
        env->astat = FIELD_DP32(env->astat, ASTAT, VS, 1);
    }
}

/*
 * One multiplier unit.
 *
 * mode holds the instruction's mmod field in bits 3 to 0 and its MM bit in
 * bit 4. op is 0, 1 or 2 for "=", "+=" and "-="; 3 when the unit only
 * extracts an accumulator it did not compute; and 4 for the Multiply 16-Bit
 * Operands forms, which leave both accumulators untouched. dst bits 1 and 0
 * are 0 to write only the accumulator, 1 for a 16-bit register half and 2
 * for a 32-bit register, and bit 2 asks for V to be merged with what is
 * already there rather than replacing it, which is how a vector form that
 * extracts both accumulators reports either of them saturating. which
 * selects MAC0 with A0 or MAC1 with A1.
 */
uint32_t HELPER(mac)(CPUBfinState *env, uint32_t s0, uint32_t s1,
                     uint32_t mode, uint32_t op, uint32_t dst, uint32_t which)
{
    int mmod = mode & 0xf;
    bool mm = extract32(mode, 4, 1);
    bool asat = false, vsat = false;
    uint64_t acc;
    uint32_t res = 0;

    switch (op) {
    case 3:
        acc = env->a[which];
        break;
    case 4:
        acc = mac_accumulate(0, mac_multiply(s0, s1, mmod, mm, &asat), 0,
                             mmod, &asat);
        break;
    default:
        acc = mac_accumulate(env->a[which],
                             mac_multiply(s0, s1, mmod, mm, &asat), op,
                             mmod, &asat);
        env->a[which] = acc;
        mac_set_av(env, which, asat);
        break;
    }

    if (dst & 3) {
        if ((dst & 3) == 2) {
            res = mac_extract_word(acc, mmod, &vsat);
        } else {
            res = mac_extract_half(env, acc, mmod, &vsat);
        }
        /*
         * Multiply 16-Bit Operands has no accumulator to overflow, so a
         * product that had to saturate is reported in V rather than AV.
         */
        if (op == 4 && asat) {
            vsat = true;
        }
        if (dst & 4) {
            vsat |= FIELD_EX32(env->astat, ASTAT, V);
        }
        mac_set_v(env, vsat);
    }
    return res;
}

/*
 * Vector Add / Subtract, MAX and MIN. AZ is set if any result is zero and AN
 * if any is negative, so the flags are collected across every half the
 * instruction produces.
 */
static void vec_set_flags(CPUBfinState *env, bool zero, bool neg, bool ovf)
{
    env->astat = FIELD_DP32(env->astat, ASTAT, AZ, zero);
    env->astat = FIELD_DP32(env->astat, ASTAT, AN, neg);
    mac_set_v(env, ovf);
}

/*
 * Reduce a 17-bit sum or difference to 16 bits. Overflow is reported whether
 * or not the (S) option asked for the value to be clamped, because ASTAT.V
 * follows the arithmetic and not the option.
 */
static uint32_t vec_pack16(int32_t v, bool sat, bool *ovf)
{
    if (v > INT16_MAX || v < INT16_MIN) {
        *ovf = true;
        if (sat) {
            return v > 0 ? (uint16_t)INT16_MAX : (uint16_t)INT16_MIN;
        }
    }
    return (uint16_t)v;
}

static uint32_t vec_pack32(int64_t v, bool sat, bool *ovf)
{
    if (v > INT32_MAX || v < INT32_MIN) {
        *ovf = true;
        if (sat) {
            return v > 0 ? (uint32_t)INT32_MAX : (uint32_t)INT32_MIN;
        }
    }
    return (uint32_t)v;
}

/* A subtraction carries when it does not borrow, that is when a >= b. */
static bool vec_carry16(uint32_t a, uint32_t b, bool sub)
{
    a = (uint16_t)a;
    b = (uint16_t)b;
    return sub ? a >= b : a + b > UINT16_MAX;
}

/*
 * Dual 16-bit add and subtract. aop bit 1 subtracts the high halves and bit
 * 0 the low halves, so the four values spell +|+, +|-, -|+ and -|-. The (CO)
 * option swaps the two results as they are written back; the worked example
 * in chapter 19 pins the order down.
 */
uint32_t HELPER(vadd16)(CPUBfinState *env, uint32_t s0, uint32_t s1,
                        uint32_t aop, uint32_t sat, uint32_t co)
{
    int32_t a_hi = (int16_t)(s0 >> 16), b_hi = (int16_t)(s1 >> 16);
    int32_t a_lo = (int16_t)s0, b_lo = (int16_t)s1;
    bool sub_hi = aop & 2, sub_lo = aop & 1;
    int32_t hi = sub_hi ? a_hi - b_hi : a_hi + b_hi;
    int32_t lo = sub_lo ? a_lo - b_lo : a_lo + b_lo;
    bool ovf = false;
    uint32_t rh = vec_pack16(hi, sat, &ovf);
    uint32_t rl = vec_pack16(lo, sat, &ovf);

    /*
     * AC1 reports the carry out of the left-hand operation of the pair and
     * AC0 that of the right-hand one.
     */
    env->astat = FIELD_DP32(env->astat, ASTAT, AC1,
                            vec_carry16(a_hi, b_hi, sub_hi));
    env->astat = FIELD_DP32(env->astat, ASTAT, AC0,
                            vec_carry16(a_lo, b_lo, sub_lo));
    vec_set_flags(env, rh == 0 || rl == 0,
                  (rh & 0x8000) || (rl & 0x8000), ovf);

    return co ? (rl << 16) | rh : (rh << 16) | rl;
}

/* (ASR) halves and (ASL) doubles each result before it is saturated. */
static int32_t vec_scale16(int32_t v, uint32_t scale)
{
    if (scale == 2) {
        return v >> 1;
    }
    if (scale == 3) {
        return v << 1;
    }
    return v;
}

/*
 * Quad 16-bit add and subtract. hl selects which pair of sign patterns the
 * two destinations get: clear is "+|+ , -|-" and set is "+|- , -|+". The
 * result named first in the syntax, which table C-21 calls Dest 0, comes
 * back in the low half of the return value.
 */
uint64_t HELPER(vadd16_q)(CPUBfinState *env, uint32_t s0, uint32_t s1,
                          uint32_t hl, uint32_t scale, uint32_t sat,
                          uint32_t co)
{
    int32_t a_hi = (int16_t)(s0 >> 16), b_hi = (int16_t)(s1 >> 16);
    int32_t a_lo = (int16_t)s0, b_lo = (int16_t)s1;
    int32_t sum_hi = a_hi + b_hi, dif_hi = a_hi - b_hi;
    int32_t sum_lo = a_lo + b_lo, dif_lo = a_lo - b_lo;
    bool ovf = false;
    uint32_t r[4];
    uint32_t d0, d1;
    bool zero = false, neg = false;
    int i;

    /* Dest 0 high, Dest 0 low, Dest 1 high, Dest 1 low. */
    r[0] = vec_pack16(vec_scale16(sum_hi, scale), sat, &ovf);
    r[1] = vec_pack16(vec_scale16(hl ? dif_lo : sum_lo, scale), sat, &ovf);
    r[2] = vec_pack16(vec_scale16(dif_hi, scale), sat, &ovf);
    r[3] = vec_pack16(vec_scale16(hl ? sum_lo : dif_lo, scale), sat, &ovf);

    for (i = 0; i < 4; i++) {
        zero |= r[i] == 0;
        neg |= (r[i] & 0x8000) != 0;
    }
    /* AC0 and AC1 are documented as unaffected by a quad operation. */
    vec_set_flags(env, zero, neg, ovf);

    d0 = co ? (r[1] << 16) | r[0] : (r[0] << 16) | r[1];
    d1 = co ? (r[3] << 16) | r[2] : (r[2] << 16) | r[3];
    return ((uint64_t)d1 << 32) | d0;
}

/*
 * Dual 32-bit add and subtract. The sum is Dest 0 and comes back in the low
 * half of the return value; the difference is Dest 1.
 */
uint64_t HELPER(vadd32)(CPUBfinState *env, uint32_t s0, uint32_t s1,
                        uint32_t sat)
{
    int64_t sum = (int64_t)(int32_t)s0 + (int32_t)s1;
    int64_t dif = (int64_t)(int32_t)s0 - (int32_t)s1;
    bool ovf = false;
    uint32_t r0 = vec_pack32(sum, sat, &ovf);
    uint32_t r1 = vec_pack32(dif, sat, &ovf);

    env->astat = FIELD_DP32(env->astat, ASTAT, AC0,
                            (uint64_t)s0 + s1 > UINT32_MAX);
    env->astat = FIELD_DP32(env->astat, ASTAT, AC1, s0 >= s1);
    vec_set_flags(env, r0 == 0 || r1 == 0,
                  ((r0 | r1) & 0x80000000u) != 0, ovf);

    return ((uint64_t)r1 << 32) | r0;
}

/*
 * Vector Maximum and Vector Minimum compare the two halves independently.
 * Chapter 19 says V is cleared unconditionally.
 */
uint32_t HELPER(vminmax16)(CPUBfinState *env, uint32_t s0, uint32_t s1,
                           uint32_t min)
{
    int32_t a_hi = (int16_t)(s0 >> 16), b_hi = (int16_t)(s1 >> 16);
    int32_t a_lo = (int16_t)s0, b_lo = (int16_t)s1;
    int32_t hi = min ? MIN(a_hi, b_hi) : MAX(a_hi, b_hi);
    int32_t lo = min ? MIN(a_lo, b_lo) : MAX(a_lo, b_lo);

    vec_set_flags(env, hi == 0 || lo == 0, hi < 0 || lo < 0, false);

    return ((uint32_t)(uint16_t)hi << 16) | (uint16_t)lo;
}
