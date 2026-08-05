/*
 * C674x arithmetic that is easier to be right about in C than in TCG.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The long tail of this instruction set is packed SIMD, multiply variants and
 * double precision: about a hundred mnemonics, individually simple, most used
 * only a handful of times in the firmware. Writing each as hand-rolled TCG
 * would be a lot of code and a lot of places to get a saturation boundary or
 * a sign extension subtly wrong, so they dispatch on the mnemonic here
 * instead. Not the fastest arrangement - a switch per instruction - but these
 * are not where the time goes, and being able to read the semantics straight
 * off SPRUFE8B matters more.
 *
 * The hot and common instructions stay as inline TCG in translate.c.
 */

static int32_t sat32(int64_t v)
{
    if (v > INT32_MAX) {
        return INT32_MAX;
    }
    if (v < INT32_MIN) {
        return INT32_MIN;
    }
    return v;
}

static int16_t sat16(int32_t v)
{
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return v;
}

static uint8_t satu8(int32_t v)
{
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

static int16_t sext16(uint32_t v)
{
    return (int16_t)(v & 0xffff);
}

static int lo16(uint32_t v)
{
    return (int16_t)(v & 0xffff);
}

static int hi16(uint32_t v)
{
    return (int16_t)(v >> 16);
}

static unsigned ulo16(uint32_t v)
{
    return v & 0xffff;
}

static unsigned uhi16(uint32_t v)
{
    return v >> 16;
}

#define B(v, n) (((v) >> ((n) * 8)) & 0xff)
#define SB(v, n) ((int8_t)B(v, n))

static uint32_t pack16(int a, int b)   /* b is the low half */
{
    return ((uint32_t)(a & 0xffff) << 16) | (b & 0xffff);
}

static uint32_t pack8(int a, int b, int c, int d)  /* d is byte 0 */
{
    return ((uint32_t)(a & 0xff) << 24) | ((uint32_t)(b & 0xff) << 16) |
           ((uint32_t)(c & 0xff) << 8) | (d & 0xff);
}

/* Two 32-bit sources, one 32-bit result. */
uint32_t HELPER(alu2)(CPUTIC6XState *env, uint32_t which, uint32_t a,
                      uint32_t b)
{
    switch (which) {
    /* ---- plain and unsigned forms that differ only in name at 32 bits */
    case TIC6X_MNEM_addu:
        return a + b;
    case TIC6X_MNEM_subu:
        return a - b;
    case TIC6X_MNEM_neg:
        return -(int32_t)b;
    case TIC6X_MNEM_not:
        return ~b;
    case TIC6X_MNEM_abs: {
        int32_t v = b;
        return v == INT32_MIN ? INT32_MAX : (v < 0 ? -v : v);
    }

    /* ---- shifts and bit counting */
    case TIC6X_MNEM_rotl: {
        unsigned n = a & 31;
        return n ? ((b << n) | (b >> (32 - n))) : b;
    }
    case TIC6X_MNEM_sshl: {
        /* Shift left, saturating rather than losing the top bits. */
        unsigned n = a & 31;
        int64_t v = (int64_t)(int32_t)b << n;
        return sat32(v);
    }
    case TIC6X_MNEM_lmbd: {
        /* How many bits before the leftmost one that equals bit 0 of src1. */
        uint32_t bit = a & 1;
        uint32_t v = bit ? b : ~b;
        return v ? clz32(v) : 32;
    }
    case TIC6X_MNEM_norm: {
        int32_t v = b;
        uint32_t u = v < 0 ? ~(uint32_t)v : (uint32_t)v;
        return u ? clz32(u) - 1 : 31;
    }
    case TIC6X_MNEM_shlmb:
        /* Shift left one byte, bringing in the top byte of the other. */
        return (b << 8) | ((a >> 24) & 0xff);
    case TIC6X_MNEM_shrmb:
        return (b >> 8) | ((a & 0xff) << 24);

    /* ---- the divide step */
    case TIC6X_MNEM_subc: {
        /* One step of a restoring divide, SPRUFE8B: if the subtraction does
           not go negative, take it and set the low bit; otherwise shift. */
        if (a >= b) {
            return ((a - b) << 1) | 1;
        }
        return a << 1;
    }

    /* ---- packed halfword arithmetic */
    case TIC6X_MNEM_add2:
        return pack16(hi16(a) + hi16(b), lo16(a) + lo16(b));
    case TIC6X_MNEM_sub2:
        return pack16(hi16(a) - hi16(b), lo16(a) - lo16(b));
    case TIC6X_MNEM_sadd2:
        return pack16(sat16(hi16(a) + hi16(b)), sat16(lo16(a) + lo16(b)));
    case TIC6X_MNEM_ssub2:
        return pack16(sat16(hi16(a) - hi16(b)), sat16(lo16(a) - lo16(b)));
    case TIC6X_MNEM_saddus2:
    case TIC6X_MNEM_saddsu2:
        return pack16(sat16((int32_t)uhi16(a) + hi16(b)),
                      sat16((int32_t)ulo16(a) + lo16(b)));
    case TIC6X_MNEM_avg2:
        return pack16((hi16(a) + hi16(b) + 1) >> 1,
                      (lo16(a) + lo16(b) + 1) >> 1);
    case TIC6X_MNEM_min2:
        return pack16(MIN(hi16(a), hi16(b)), MIN(lo16(a), lo16(b)));
    case TIC6X_MNEM_max2:
        return pack16(MAX(hi16(a), hi16(b)), MAX(lo16(a), lo16(b)));
    case TIC6X_MNEM_shr2:
        return pack16(hi16(b) >> (a & 15), lo16(b) >> (a & 15));
    case TIC6X_MNEM_shru2:
        return pack16(uhi16(b) >> (a & 15), ulo16(b) >> (a & 15));
    case TIC6X_MNEM_packh2:
        return pack16(hi16(a), hi16(b));
    case TIC6X_MNEM_packl4:
        return pack8(B(a, 2), B(a, 0), B(b, 2), B(b, 0));
    case TIC6X_MNEM_packh4:
        return pack8(B(a, 3), B(a, 1), B(b, 3), B(b, 1));
    case TIC6X_MNEM_spack2:
        return pack16(sat16((int32_t)a), sat16((int32_t)b));
    case TIC6X_MNEM_spacku4:
        return pack8(satu8(hi16(a)), satu8(lo16(a)),
                     satu8(hi16(b)), satu8(lo16(b)));
    case TIC6X_MNEM_xpnd2: {
        /* Expand the two low bits into two halfword masks. */
        return pack16((b & 2) ? 0xffff : 0, (b & 1) ? 0xffff : 0);
    }
    case TIC6X_MNEM_xpnd4:
        return pack8((b & 8) ? 0xff : 0, (b & 4) ? 0xff : 0,
                     (b & 2) ? 0xff : 0, (b & 1) ? 0xff : 0);
    case TIC6X_MNEM_unpklu4:
        return pack16(B(b, 1), B(b, 0));

    /* ---- packed byte arithmetic */
    case TIC6X_MNEM_add4:
        return pack8(B(a, 3) + B(b, 3), B(a, 2) + B(b, 2),
                     B(a, 1) + B(b, 1), B(a, 0) + B(b, 0));
    case TIC6X_MNEM_sub4:
        return pack8(B(a, 3) - B(b, 3), B(a, 2) - B(b, 2),
                     B(a, 1) - B(b, 1), B(a, 0) - B(b, 0));
    case TIC6X_MNEM_saddu4:
        return pack8(satu8(B(a, 3) + B(b, 3)), satu8(B(a, 2) + B(b, 2)),
                     satu8(B(a, 1) + B(b, 1)), satu8(B(a, 0) + B(b, 0)));
    case TIC6X_MNEM_subabs4:
        return pack8(abs((int)B(a, 3) - (int)B(b, 3)),
                     abs((int)B(a, 2) - (int)B(b, 2)),
                     abs((int)B(a, 1) - (int)B(b, 1)),
                     abs((int)B(a, 0) - (int)B(b, 0)));
    case TIC6X_MNEM_minu4:
        return pack8(MIN(B(a, 3), B(b, 3)), MIN(B(a, 2), B(b, 2)),
                     MIN(B(a, 1), B(b, 1)), MIN(B(a, 0), B(b, 0)));
    case TIC6X_MNEM_maxu4:
        return pack8(MAX(B(a, 3), B(b, 3)), MAX(B(a, 2), B(b, 2)),
                     MAX(B(a, 1), B(b, 1)), MAX(B(a, 0), B(b, 0)));
    case TIC6X_MNEM_avgu4:
        return pack8((B(a, 3) + B(b, 3) + 1) >> 1,
                     (B(a, 2) + B(b, 2) + 1) >> 1,
                     (B(a, 1) + B(b, 1) + 1) >> 1,
                     (B(a, 0) + B(b, 0) + 1) >> 1);

    /* ---- packed compares: one bit of the result per lane */
    case TIC6X_MNEM_cmpeq2:
        return (hi16(a) == hi16(b) ? 2 : 0) | (lo16(a) == lo16(b) ? 1 : 0);
    case TIC6X_MNEM_cmpgt2:
        return (hi16(a) > hi16(b) ? 2 : 0) | (lo16(a) > lo16(b) ? 1 : 0);
    case TIC6X_MNEM_cmpeq4:
        return (B(a, 3) == B(b, 3) ? 8 : 0) | (B(a, 2) == B(b, 2) ? 4 : 0) |
               (B(a, 1) == B(b, 1) ? 2 : 0) | (B(a, 0) == B(b, 0) ? 1 : 0);
    case TIC6X_MNEM_cmpgtu4:
        return (B(a, 3) > B(b, 3) ? 8 : 0) | (B(a, 2) > B(b, 2) ? 4 : 0) |
               (B(a, 1) > B(b, 1) ? 2 : 0) | (B(a, 0) > B(b, 0) ? 1 : 0);

    /* ---- multiplies with a 32-bit result */
    case TIC6X_MNEM_mpyi:
        return (int32_t)a * (int32_t)b;
    case TIC6X_MNEM_mpyhi:
        return (int32_t)(((int64_t)hi16(a) * (int32_t)b) >> 16);
    case TIC6X_MNEM_mpyli:
        return (int32_t)(((int64_t)lo16(a) * (int32_t)b) >> 16);
    case TIC6X_MNEM_mpylshu:
        return (uint32_t)(lo16(a) * (int64_t)uhi16(b));
    case TIC6X_MNEM_mpyluhs:
        return (uint32_t)((int64_t)ulo16(a) * hi16(b));
    case TIC6X_MNEM_mpyhuls:
        return (uint32_t)((int64_t)uhi16(a) * lo16(b));
    case TIC6X_MNEM_mpyhslu:
        return (uint32_t)((int64_t)hi16(a) * ulo16(b));
    case TIC6X_MNEM_mpylhu:
        return ulo16(a) * uhi16(b);
    case TIC6X_MNEM_mpyhlu:
        return uhi16(a) * ulo16(b);
    case TIC6X_MNEM_mpyhsu:
        return (uint32_t)(hi16(a) * (int64_t)ulo16(b));
    case TIC6X_MNEM_mpyhus:
        return (uint32_t)((int64_t)uhi16(a) * lo16(b));
    case TIC6X_MNEM_smpylh:
        return sat32(2LL * lo16(a) * hi16(b));
    case TIC6X_MNEM_smpyhl:
        return sat32(2LL * hi16(a) * lo16(b));
    case TIC6X_MNEM_mpy32u:
    case TIC6X_MNEM_mpy32su:
    case TIC6X_MNEM_mpy32us:
        return a * b;                       /* low 32 bits are the same */

    /* ---- dot products */
    case TIC6X_MNEM_dotp2:
        return hi16(a) * hi16(b) + lo16(a) * lo16(b);
    case TIC6X_MNEM_dotpn2:
        return lo16(a) * lo16(b) - hi16(a) * hi16(b);
    case TIC6X_MNEM_dotprsu2:
        return sat32((((int64_t)hi16(a) * uhi16(b) +
                       (int64_t)lo16(a) * ulo16(b)) + 0x8000) >> 16);
    case TIC6X_MNEM_dotpnrsu2:
        return sat32((((int64_t)lo16(a) * ulo16(b) -
                       (int64_t)hi16(a) * uhi16(b)) + 0x8000) >> 16);
    case TIC6X_MNEM_dotpu4:
        return B(a, 0) * B(b, 0) + B(a, 1) * B(b, 1) +
               B(a, 2) * B(b, 2) + B(a, 3) * B(b, 3);
    case TIC6X_MNEM_dotpsu4:
        return SB(a, 0) * B(b, 0) + SB(a, 1) * B(b, 1) +
               SB(a, 2) * B(b, 2) + SB(a, 3) * B(b, 3);

    /* ---- Galois field multiply, used by the error correction code */
    case TIC6X_MNEM_sshvl: {
        /* Variable shift left, saturating; a negative count shifts right. */
        int32_t n = (int32_t)a;
        if (n < 0) {
            n = -n > 31 ? 31 : -n;
            return (int32_t)b >> n;
        }
        n = n > 31 ? 31 : n;
        return sat32((int64_t)(int32_t)b << n);
    }
    case TIC6X_MNEM_sshvr: {
        int32_t n = (int32_t)a;
        if (n < 0) {
            n = -n > 31 ? 31 : -n;
            return sat32((int64_t)(int32_t)b << n);
        }
        n = n > 31 ? 31 : n;
        return (int32_t)b >> n;
    }
    case TIC6X_MNEM_mpyhir:
        return sat32((((int64_t)hi16(a) * (int32_t)b) + 0x4000) >> 15);
    case TIC6X_MNEM_mpyilr:
        return sat32((((int64_t)lo16(a) * (int32_t)b) + 0x4000) >> 15);
    case TIC6X_MNEM_mpyil:
        return (int32_t)(((int64_t)lo16(a) * (int32_t)b) >> 16);

    case TIC6X_MNEM_gmpy4: {
        uint32_t poly = env->cr[TIC6X_CR_AMR];  /* GFPGFR is not modelled */
        uint32_t r = 0;
        int lane;

        for (lane = 0; lane < 4; lane++) {
            unsigned x = B(a, lane), y = B(b, lane), acc = 0, i;

            for (i = 0; i < 8; i++) {
                if (y & (1 << i)) {
                    acc ^= x << i;
                }
            }
            for (i = 15; i >= 8; i--) {
                if (acc & (1 << i)) {
                    acc ^= (poly | 0x100) << (i - 8);
                }
            }
            r |= (acc & 0xff) << (lane * 8);
        }
        return r;
    }

    default:
        break;
    }
    /*
     * Not reached for anything translate.c routes here; a mnemonic arriving
     * that is not in the switch is a bug in the routing rather than a gap in
     * the firmware's needs, so say so loudly.
     */
    qemu_log_mask(LOG_UNIMP, "tic6x: alu2 has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

/* Two 32-bit sources, a 64-bit result into a register pair. */
uint64_t HELPER(alu2_wide)(CPUTIC6XState *env, uint32_t which, uint32_t a,
                           uint32_t b)
{
    switch (which) {
    case TIC6X_MNEM_mpyid:
        return (uint64_t)((int64_t)(int32_t)a * (int32_t)b);
    case TIC6X_MNEM_mpy32:                  /* the 64-bit result form */
        return (uint64_t)((int64_t)(int32_t)a * (int32_t)b);
    case TIC6X_MNEM_mpy32u:
        return (uint64_t)a * b;
    case TIC6X_MNEM_mpy32su:
        return (uint64_t)((int64_t)(int32_t)a * (uint64_t)b);
    case TIC6X_MNEM_mpy32us:
        return (uint64_t)((uint64_t)a * (int64_t)(int32_t)b);
    case TIC6X_MNEM_mpy2:
        return ((uint64_t)(uint32_t)(hi16(a) * hi16(b)) << 32) |
               (uint32_t)(lo16(a) * lo16(b));
    case TIC6X_MNEM_smpy2:
        return ((uint64_t)(uint32_t)sat32(2LL * hi16(a) * hi16(b)) << 32) |
               (uint32_t)sat32(2LL * lo16(a) * lo16(b));
    case TIC6X_MNEM_mpyu4:
        return ((uint64_t)(B(a, 3) * B(b, 3)) << 48) |
               ((uint64_t)(B(a, 2) * B(b, 2)) << 32) |
               ((uint64_t)(B(a, 1) * B(b, 1)) << 16) |
               (uint64_t)(B(a, 0) * B(b, 0));
    case TIC6X_MNEM_mpysu4:
        return ((uint64_t)(uint16_t)(SB(a, 3) * B(b, 3)) << 48) |
               ((uint64_t)(uint16_t)(SB(a, 2) * B(b, 2)) << 32) |
               ((uint64_t)(uint16_t)(SB(a, 1) * B(b, 1)) << 16) |
               (uint16_t)(SB(a, 0) * B(b, 0));
    case TIC6X_MNEM_cmpy: {
        /* Complex multiply: real in the low word, imaginary in the high. */
        int32_t re = lo16(a) * lo16(b) - hi16(a) * hi16(b);
        int32_t im = lo16(a) * hi16(b) + hi16(a) * lo16(b);

        return ((uint64_t)(uint32_t)im << 32) | (uint32_t)re;
    }
    case TIC6X_MNEM_cmpyr1: {
        int32_t re = sat32((((int64_t)lo16(a) * lo16(b) -
                             (int64_t)hi16(a) * hi16(b)) + 0x4000) >> 15);
        int32_t im = sat32((((int64_t)lo16(a) * hi16(b) +
                             (int64_t)hi16(a) * lo16(b)) + 0x4000) >> 15);

        return pack16(sat16(im), sat16(re));
    }
    case TIC6X_MNEM_addsub:
        return ((uint64_t)(a - b) << 32) | (uint32_t)(a + b);
    case TIC6X_MNEM_addsub2:
        return ((uint64_t)(uint32_t)pack16(hi16(a) - hi16(b),
                                           lo16(a) - lo16(b)) << 32) |
               (uint32_t)pack16(hi16(a) + hi16(b), lo16(a) + lo16(b));
    case TIC6X_MNEM_saddsub:
        return ((uint64_t)(uint32_t)sat32((int64_t)(int32_t)a -
                                          (int32_t)b) << 32) |
               (uint32_t)sat32((int64_t)(int32_t)a + (int32_t)b);
    case TIC6X_MNEM_saddsub2:
        return ((uint64_t)(uint32_t)pack16(sat16(hi16(a) - hi16(b)),
                                           sat16(lo16(a) - lo16(b))) << 32) |
               (uint32_t)pack16(sat16(hi16(a) + hi16(b)),
                                sat16(lo16(a) + lo16(b)));
    case TIC6X_MNEM_dmv:
        return ((uint64_t)b << 32) | a;
    case TIC6X_MNEM_ddotp4:
        /* Two dot products of byte pairs, one per result half. */
        return ((uint64_t)(uint32_t)(SB(a, 3) * SB(b, 3) +
                                     SB(a, 2) * SB(b, 2)) << 32) |
               (uint32_t)(SB(a, 1) * SB(b, 1) + SB(a, 0) * SB(b, 0));
    case TIC6X_MNEM_ddotpl2:
        return ((uint64_t)(uint32_t)(hi16(a) * hi16(b) +
                                     lo16(a) * lo16(b)) << 32) |
               (uint32_t)(lo16(a) * lo16(b) + hi16(a) * hi16(b));
    case TIC6X_MNEM_ddotpl2r:
        return ((uint64_t)(uint32_t)pack16(
                    sat16((hi16(a) * hi16(b) + lo16(a) * lo16(b) +
                           0x8000) >> 16), 0) << 32) |
               (uint32_t)pack16(sat16((lo16(a) * lo16(b) +
                                       hi16(a) * hi16(b) + 0x8000) >> 16), 0);
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: alu2_wide has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

/* One source, one result. */
uint32_t HELPER(alu1)(uint32_t which, uint32_t b)
{
    switch (which) {
    case TIC6X_MNEM_neg:
        return -(int32_t)b;
    case TIC6X_MNEM_not:
        return ~b;
    case TIC6X_MNEM_abs: {
        int32_t v = b;
        return v == INT32_MIN ? INT32_MAX : (v < 0 ? -v : v);
    }
    default:
        break;
    }
    return 0;
}

/* ------------------------------------------------- double precision */

static float_status *dp_status(CPUTIC6XState *env)
{
    return sp_status(env);
}

uint64_t HELPER(dp2)(CPUTIC6XState *env, uint32_t which, uint64_t a,
                     uint64_t b)
{
    float64 fa = make_float64(a), fb = make_float64(b);

    switch (which) {
    case TIC6X_MNEM_adddp:
        return float64_val(float64_add(fa, fb, dp_status(env)));
    case TIC6X_MNEM_subdp:
        return float64_val(float64_sub(fa, fb, dp_status(env)));
    case TIC6X_MNEM_mpydp:
        return float64_val(float64_mul(fa, fb, dp_status(env)));
    case TIC6X_MNEM_absdp:
        return float64_val(float64_abs(fb));
    case TIC6X_MNEM_rcpdp:
        /* An approximation is all the architecture promises; the firmware
           refines it with Newton steps of its own. */
        return float64_val(float64_div(float64_one, fb, dp_status(env)));
    case TIC6X_MNEM_rsqrdp:
        return float64_val(float64_div(float64_one,
                                       float64_sqrt(fb, dp_status(env)),
                                       dp_status(env)));
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: dp2 has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

uint32_t HELPER(dp_cmp)(CPUTIC6XState *env, uint32_t which, uint64_t a,
                        uint64_t b)
{
    float64 fa = make_float64(a), fb = make_float64(b);

    switch (which) {
    case TIC6X_MNEM_cmpeqdp:
        return float64_eq_quiet(fa, fb, dp_status(env)) ? 1 : 0;
    case TIC6X_MNEM_cmpgtdp:
        return float64_lt(fb, fa, dp_status(env)) ? 1 : 0;
    case TIC6X_MNEM_cmpltdp:
        return float64_lt(fa, fb, dp_status(env)) ? 1 : 0;
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: dp_cmp has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

/* Mixed precision: a single and a double in, a double out. */
uint64_t HELPER(spdp2)(CPUTIC6XState *env, uint32_t which, uint32_t a,
                       uint64_t b)
{
    float64 fa = float32_to_float64(make_float32(a), dp_status(env));

    switch (which) {
    case TIC6X_MNEM_mpyspdp:
        return float64_val(float64_mul(fa, make_float64(b), dp_status(env)));
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: spdp2 has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

/* Two singles in, a double out. */
uint64_t HELPER(sp2dp)(CPUTIC6XState *env, uint32_t which, uint32_t a,
                       uint32_t b)
{
    float_status *st = dp_status(env);
    float64 fa = float32_to_float64(make_float32(a), st);
    float64 fb = float32_to_float64(make_float32(b), st);

    switch (which) {
    case TIC6X_MNEM_mpysp2dp:
        return float64_val(float64_mul(fa, fb, st));
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: sp2dp has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

/* Single-operand single-precision conversions. */
uint32_t HELPER(sp1)(CPUTIC6XState *env, uint32_t which, uint32_t a)
{
    float_status *st = sp_status(env);

    switch (which) {
    case TIC6X_MNEM_spint:
        return float32_to_int32(make_float32(a), st);
    case TIC6X_MNEM_sptrunc:
        return float32_to_int32_round_to_zero(make_float32(a), st);
    case TIC6X_MNEM_intsp:
        return float32_val(int32_to_float32((int32_t)a, st));
    case TIC6X_MNEM_intspu:
        return float32_val(uint32_to_float32(a, st));
    case TIC6X_MNEM_rsqrsp:
        return float32_val(float32_div(float32_one,
                                       float32_sqrt(make_float32(a), st), st));
    case TIC6X_MNEM_rcpsp:
        return float32_val(float32_div(float32_one, make_float32(a), st));
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: sp1 has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

uint32_t HELPER(dp_to_w)(CPUTIC6XState *env, uint32_t which, uint64_t a)
{
    float64 fa = make_float64(a);

    switch (which) {
    case TIC6X_MNEM_dpsp:
        return float32_val(float64_to_float32(fa, dp_status(env)));
    case TIC6X_MNEM_dpint:
        return float64_to_int32(fa, dp_status(env));
    case TIC6X_MNEM_dptrunc:
        return float64_to_int32_round_to_zero(fa, dp_status(env));
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: dp_to_w has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}

uint64_t HELPER(w_to_dp)(CPUTIC6XState *env, uint32_t which, uint32_t a)
{
    switch (which) {
    case TIC6X_MNEM_spdp:
        return float64_val(float32_to_float64(make_float32(a),
                                              dp_status(env)));
    case TIC6X_MNEM_intdp:
        return float64_val(int32_to_float64((int32_t)a, dp_status(env)));
    case TIC6X_MNEM_intdpu:
        return float64_val(uint32_to_float64(a, dp_status(env)));
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: w_to_dp has no case for %s\n",
                  tic6x_mnemonic_name(which));
    return 0;
}
