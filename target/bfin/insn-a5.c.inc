/* SPDX-License-Identifier: GPL-2.0-or-later */
/* ---------------------------------------------------------------- */
/* Multiply, multiply-accumulate and vector operations              */

/*
 * The arithmetic itself lives in helper.c: a MAC carries three mostly
 * independent option axes - operand format, accumulator width and the way
 * the result is extracted - and expressing those inline would cost far more
 * TCG than the handful of multiplies the firmware executes is worth.
 */

static bool trans_mul32_d_d(DisasContext *ctx, arg_mul32_d_d *a)
{
    /*
     * Dreg *= Dreg is a 32 by 32 integer multiply modulo 2^32. Chapter 15
     * notes that overflow is possible but not detected, that no status bit
     * moves and that neither accumulator is disturbed.
     */
    tcg_gen_mul_i32(dreg(a->dst), dreg(a->dst), dreg(a->src));
    return true;
}

/* Take the half of a data register named by one of the h.. option bits. */
static TCGv gen_mac_operand(int reg, int high)
{
    TCGv t = tcg_temp_new_i32();

    if (high) {
        tcg_gen_shri_i32(t, dreg(reg), 16);
    } else {
        tcg_gen_andi_i32(t, dreg(reg), 0xffff);
    }
    return t;
}

/*
 * mmod values 5, 7, 10 and 13 to 15 have no option assigned to them in table
 * C-17. Letting them through would silently pick a neighbouring format, so
 * treat them as undefined instead.
 */
static bool mac_mmod_valid(int mmod)
{
    return mmod != 5 && mmod != 7 && mmod != 10 && mmod < 13;
}

typedef struct {
    int mmod;
    int mm;                     /* mixed multiply, MAC1 only              */
    int p;                      /* destination is a 32-bit register       */
    int w0, op0, h00, h10;      /* MAC0 and A0                            */
    int w1, op1, h01, h11;      /* MAC1 and A1                            */
    int dst, src0, src1;
    bool acc;                   /* false for Multiply 16-Bit Operands     */
} BfinMacOp;

static bool gen_mac(DisasContext *ctx, const BfinMacOp *m)
{
    int dstmode = m->p ? 2 : 1;
    bool run0 = m->w0 || (m->acc && m->op0 != 3);
    bool run1 = m->w1 || (m->acc && m->op1 != 3);
    TCGv r0 = NULL, r1 = NULL;

    if (!mac_mmod_valid(m->mmod)) {
        return false;
    }

    /*
     * Both multipliers are offered the same pair of source registers, so
     * every operand has to be read before either destination is written -
     * the destination may well be one of the sources.
     */
    if (run0) {
        TCGv s0 = gen_mac_operand(m->src0, m->h00);
        TCGv s1 = gen_mac_operand(m->src1, m->h10);

        r0 = tcg_temp_new_i32();
        /* MAC0 has no mixed mode: it always uses the named format. */
        gen_helper_mac(r0, tcg_env, s0, s1, tcg_constant_i32(m->mmod),
                       tcg_constant_i32(m->acc ? m->op0 : 4),
                       tcg_constant_i32(m->w0 ? dstmode : 0),
                       tcg_constant_i32(0));
    }
    if (run1) {
        TCGv s0 = gen_mac_operand(m->src0, m->h01);
        TCGv s1 = gen_mac_operand(m->src1, m->h11);
        /*
         * MAC1 runs second, so when MAC0 also extracted a result it has
         * already written V; bit 2 of the destination mode asks for the two
         * saturation reports to be merged rather than the later one winning.
         */
        int mode1 = m->w1 ? dstmode | (m->w0 ? 4 : 0) : 0;

        r1 = tcg_temp_new_i32();
        gen_helper_mac(r1, tcg_env, s0, s1,
                       tcg_constant_i32(m->mmod | (m->mm << 4)),
                       tcg_constant_i32(m->acc ? m->op1 : 4),
                       tcg_constant_i32(mode1), tcg_constant_i32(1));
    }

    /*
     * A 32-bit result goes to the even register of the pair for MAC0 and to
     * the odd one for MAC1; a 16-bit result goes to the matching half of the
     * single named register.
     */
    if (m->w0) {
        if (m->p) {
            tcg_gen_mov_i32(dreg(m->dst & 6), r0);
        } else {
            tcg_gen_deposit_i32(dreg(m->dst), dreg(m->dst), r0, 0, 16);
        }
    }
    if (m->w1) {
        if (m->p) {
            tcg_gen_mov_i32(dreg(m->dst | 1), r1);
        } else {
            tcg_gen_deposit_i32(dreg(m->dst), dreg(m->dst), r1, 16, 16);
        }
    }
    return true;
}

static bool trans_mult_m0(DisasContext *ctx, arg_mult_m0 *a)
{
    BfinMacOp m = {
        .mmod = a->mmod, .mm = a->mm, .p = a->p,
        .w0 = 1, .h00 = a->h00, .h10 = a->h10,
        .dst = a->dst, .src0 = a->src0, .src1 = a->src1,
    };

    return gen_mac(ctx, &m);
}

static bool trans_mult_m1(DisasContext *ctx, arg_mult_m1 *a)
{
    BfinMacOp m = {
        .mmod = a->mmod, .mm = a->mm, .p = a->p,
        .w1 = 1, .h01 = a->h01, .h11 = a->h11,
        .dst = a->dst, .src0 = a->src0, .src1 = a->src1,
    };

    return gen_mac(ctx, &m);
}

static bool trans_mult_v(DisasContext *ctx, arg_mult_v *a)
{
    BfinMacOp m = {
        .mmod = a->mmod, .mm = a->mm, .p = a->p,
        .w0 = 1, .h00 = a->h00, .h10 = a->h10,
        .w1 = 1, .h01 = a->h01, .h11 = a->h11,
        .dst = a->dst, .src0 = a->src0, .src1 = a->src1,
    };

    return gen_mac(ctx, &m);
}

static bool gen_mac_pair(DisasContext *ctx, int mmod, int mm, int p,
                         int w0, int op0, int h00, int h10,
                         int w1, int op1, int h01, int h11,
                         int dst, int src0, int src1)
{
    BfinMacOp m = {
        .mmod = mmod, .mm = mm, .p = p,
        .w0 = w0, .op0 = op0, .h00 = h00, .h10 = h10,
        .w1 = w1, .op1 = op1, .h01 = h01, .h11 = h11,
        .dst = dst, .src0 = src0, .src1 = src1, .acc = true,
    };

    return gen_mac(ctx, &m);
}

static bool trans_mac_m1(DisasContext *ctx, arg_mac_m1 *a)
{
    return gen_mac_pair(ctx, a->mmod, a->mm, a->p,
                        a->w0, a->op0, a->h00, a->h10,
                        a->w1, a->op1, a->h01, a->h11,
                        a->dst, a->src0, a->src1);
}

static bool trans_mac_m1_sub(DisasContext *ctx, arg_mac_m1_sub *a)
{
    return gen_mac_pair(ctx, a->mmod, a->mm, a->p,
                        a->w0, a->op0, a->h00, a->h10,
                        a->w1, 2, a->h01, a->h11,
                        a->dst, a->src0, a->src1);
}

static bool trans_mac_m0(DisasContext *ctx, arg_mac_m0 *a)
{
    return gen_mac_pair(ctx, a->mmod, a->mm, a->p,
                        a->w0, a->op0, a->h00, a->h10,
                        0, 3, 0, 0,
                        a->dst, a->src0, a->src1);
}

static bool trans_mac_m0_sub(DisasContext *ctx, arg_mac_m0_sub *a)
{
    return gen_mac_pair(ctx, a->mmod, a->mm, a->p,
                        a->w0, 2, a->h00, a->h10,
                        0, 3, 0, 0,
                        a->dst, a->src0, a->src1);
}

/* ---------------------------------------------------------------- */
/* Vector add and subtract                                          */

static bool trans_vadd16(DisasContext *ctx, arg_vadd16 *a)
{
    gen_helper_vadd16(dreg(a->dst), tcg_env, dreg(a->src0), dreg(a->src1),
                      tcg_constant_i32(a->aop), tcg_constant_i32(a->s),
                      tcg_constant_i32(a->co));
    return true;
}

static bool gen_vadd16_q(DisasContext *ctx, int hl, int scale, int s, int co,
                         int dst0, int dst1, int src0, int src1)
{
    TCGv_i64 r = tcg_temp_new_i64();

    /* Only 0, 2 and 3 are assigned to the scaling option; 1 is reserved. */
    if (scale == 1) {
        return false;
    }
    gen_helper_vadd16_q(r, tcg_env, dreg(src0), dreg(src1),
                        tcg_constant_i32(hl), tcg_constant_i32(scale),
                        tcg_constant_i32(s), tcg_constant_i32(co));
    tcg_gen_extrl_i64_i32(dreg(dst0), r);
    tcg_gen_extrh_i64_i32(dreg(dst1), r);
    return true;
}

static bool trans_vadd16_q(DisasContext *ctx, arg_vadd16_q *a)
{
    return gen_vadd16_q(ctx, 0, a->aop, a->s, a->co,
                        a->dst0, a->dst1, a->src0, a->src1);
}

static bool trans_vadd16_qx(DisasContext *ctx, arg_vadd16_qx *a)
{
    return gen_vadd16_q(ctx, 1, a->aop, a->s, a->co,
                        a->dst0, a->dst1, a->src0, a->src1);
}

static bool gen_vadd32(DisasContext *ctx, int s, int dst0, int dst1,
                       int src0, int src1)
{
    TCGv_i64 r = tcg_temp_new_i64();

    gen_helper_vadd32(r, tcg_env, dreg(src0), dreg(src1),
                      tcg_constant_i32(s));
    tcg_gen_extrl_i64_i32(dreg(dst0), r);
    tcg_gen_extrh_i64_i32(dreg(dst1), r);
    return true;
}

static bool trans_vadd32(DisasContext *ctx, arg_vadd32 *a)
{
    return gen_vadd32(ctx, 0, a->dst0, a->dst1, a->src0, a->src1);
}

static bool trans_vadd32_s(DisasContext *ctx, arg_vadd32_s *a)
{
    return gen_vadd32(ctx, 1, a->dst0, a->dst1, a->src0, a->src1);
}

/* ---------------------------------------------------------------- */
/* Vector maximum, minimum and pack                                 */

static bool trans_vmax16(DisasContext *ctx, arg_vmax16 *a)
{
    gen_helper_vminmax16(dreg(a->dst), tcg_env, dreg(a->src0),
                         dreg(a->src1), tcg_constant_i32(0));
    return true;
}

static bool trans_vmin16(DisasContext *ctx, arg_vmin16 *a)
{
    gen_helper_vminmax16(dreg(a->dst), tcg_env, dreg(a->src0),
                         dreg(a->src1), tcg_constant_i32(1));
    return true;
}

/*
 * Table 19-19: the first half named becomes the upper half of the destination
 * and the second the lower half. The encoding numbers its two register fields
 * the other way about - the half named first is src1 - so the upper half of
 * the result is read from src1 and the lower half from src0. No status bit
 * moves.
 */
static bool gen_pack(DisasContext *ctx, int dst, int hi_src, bool hi_half,
                     int lo_src, bool lo_half)
{
    TCGv hi = tcg_temp_new_i32();
    TCGv lo = tcg_temp_new_i32();

    if (hi_half) {
        tcg_gen_andi_i32(hi, dreg(hi_src), 0xffff0000);
    } else {
        tcg_gen_shli_i32(hi, dreg(hi_src), 16);
    }
    if (lo_half) {
        tcg_gen_shri_i32(lo, dreg(lo_src), 16);
    } else {
        tcg_gen_andi_i32(lo, dreg(lo_src), 0xffff);
    }
    tcg_gen_or_i32(dreg(dst), hi, lo);
    return true;
}

static bool trans_pack_ll(DisasContext *ctx, arg_pack_ll *a)
{
    return gen_pack(ctx, a->dst, a->src1, false, a->src0, false);
}

static bool trans_pack_lh(DisasContext *ctx, arg_pack_lh *a)
{
    return gen_pack(ctx, a->dst, a->src1, false, a->src0, true);
}

static bool trans_pack_hl(DisasContext *ctx, arg_pack_hl *a)
{
    return gen_pack(ctx, a->dst, a->src1, true, a->src0, false);
}

static bool trans_pack_hh(DisasContext *ctx, arg_pack_hh *a)
{
    return gen_pack(ctx, a->dst, a->src1, true, a->src0, true);
}
