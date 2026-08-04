/*
 * Analog Devices Blackfin ALU, shifter and bit-field translation
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Fragment for translate.c; the encodings live in a4.decode. Semantics are
 * from the Blackfin Processor Programming Reference rev 2.2 chapters 13
 * (bit operations), 14 (shift/rotate) and 15 (arithmetic), and the status
 * bit summaries in chapter 2 of the ADSP-BF533 Blackfin Processor Hardware
 * Reference rev 3.6.
 *
 * Paste after gen_logic_flags(), which this fragment reuses for the AZ and
 * AN bits of a 32-bit result, and after ireg().
 *
 * Where the two manuals disagree about a status bit, the per-instruction
 * "Status Bits Affected" prose of the programming reference wins, because it
 * enumerates the bits and then says the rest are unaffected. Two such
 * disagreements are called out below.
 */

/* ---------------------------------------------------------------- */
/* Arithmetic status bits                                           */

static void gen_astat_set(int shift, TCGv val)
{
    tcg_gen_deposit_i32(cpu_astat, cpu_astat, val, shift, 1);
}

static void gen_astat_seti(int shift, int val)
{
    if (val) {
        tcg_gen_ori_i32(cpu_astat, cpu_astat, 1u << shift);
    } else {
        tcg_gen_andi_i32(cpu_astat, cpu_astat, ~(1u << shift));
    }
}

/* AC0_COPY is architecturally required to read back identical to AC0. */
static void gen_ac0_flag(TCGv val)
{
    gen_astat_set(R_ASTAT_AC0_SHIFT, val);
    gen_astat_set(R_ASTAT_AC0_COPY_SHIFT, val);
}

static void gen_ac0_flagi(int val)
{
    gen_astat_seti(R_ASTAT_AC0_SHIFT, val);
    gen_astat_seti(R_ASTAT_AC0_COPY_SHIFT, val);
}

/*
 * V has a copy at bit 3 and a sticky companion at bit 25. VS is set with V
 * and is cleared only by writing ASTAT, so it is never assigned zero here.
 */
static void gen_v_flag(TCGv val)
{
    TCGv t = tcg_temp_new_i32();

    gen_astat_set(R_ASTAT_V_SHIFT, val);
    gen_astat_set(R_ASTAT_V_COPY_SHIFT, val);
    tcg_gen_shli_i32(t, val, R_ASTAT_VS_SHIFT);
    tcg_gen_or_i32(cpu_astat, cpu_astat, t);
}

static void gen_v_flagi(int val)
{
    gen_astat_seti(R_ASTAT_V_SHIFT, val);
    gen_astat_seti(R_ASTAT_V_COPY_SHIFT, val);
    if (val) {
        tcg_gen_ori_i32(cpu_astat, cpu_astat, R_ASTAT_VS_MASK);
    }
}

/* AZ and AN for a result that is 16 bits wide, held in the low half. */
static void gen_logic_flags16(TCGv res)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_andi_i32(t, res, 0xffff);
    tcg_gen_setcondi_i32(TCG_COND_EQ, t, t, 0);
    gen_astat_set(R_ASTAT_AZ_SHIFT, t);
    tcg_gen_extract_i32(t, res, 15, 1);
    gen_astat_set(R_ASTAT_AN_SHIFT, t);
}

/* ---------------------------------------------------------------- */
/* Half register access                                             */

static TCGv gen_half_s(int rn, bool hi)
{
    TCGv t = tcg_temp_new_i32();

    if (hi) {
        tcg_gen_sari_i32(t, dreg(rn), 16);
    } else {
        tcg_gen_ext16s_i32(t, dreg(rn));
    }
    return t;
}

static TCGv gen_half_u(int rn, bool hi)
{
    TCGv t = tcg_temp_new_i32();

    if (hi) {
        tcg_gen_shri_i32(t, dreg(rn), 16);
    } else {
        tcg_gen_ext16u_i32(t, dreg(rn));
    }
    return t;
}

static void gen_put_half(int rn, bool hi, TCGv val)
{
    tcg_gen_deposit_i32(dreg(rn), dreg(rn), val, hi ? 16 : 0, 16);
}

/* ---------------------------------------------------------------- */
/* Add and subtract                                                 */

/*
 * 32-bit add and subtract. AC0 is the unsigned carry out of the most
 * significant bit and V the two's-complement overflow, both as described in
 * "ALU Data Types" in chapter 2 of the hardware reference. Subtract is
 * evaluated as a + ~b + 1, so its carry out is set exactly when the
 * unsigned minuend is not smaller than the subtrahend.
 *
 * The saturating forms clamp to the int32 range and still report the clamp
 * in V; the true result has the sign of the first operand, because overflow
 * requires the operands to agree in sign for an add and to disagree for a
 * subtract.
 */
static void gen_addsub32(TCGv dst, TCGv a, TCGv b, bool sub, bool sat,
                         bool carry)
{
    TCGv res = tcg_temp_new_i32();
    TCGv ovf = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    if (sub) {
        tcg_gen_sub_i32(res, a, b);
        tcg_gen_xor_i32(ovf, a, b);
    } else {
        tcg_gen_add_i32(res, a, b);
        tcg_gen_eqv_i32(ovf, a, b);
    }
    tcg_gen_xor_i32(t, a, res);
    tcg_gen_and_i32(ovf, ovf, t);
    tcg_gen_shri_i32(ovf, ovf, 31);

    if (carry) {
        if (sub) {
            tcg_gen_setcond_i32(TCG_COND_GEU, t, a, b);
        } else {
            tcg_gen_setcond_i32(TCG_COND_LTU, t, res, a);
        }
        gen_ac0_flag(t);
    }
    if (sat) {
        tcg_gen_sari_i32(t, a, 31);
        tcg_gen_xori_i32(t, t, INT32_MAX);
        tcg_gen_movcond_i32(TCG_COND_NE, res, ovf, tcg_constant_i32(0),
                            t, res);
    }
    gen_v_flag(ovf);
    tcg_gen_mov_i32(dst, res);
    gen_logic_flags(dst);
}

/*
 * The half-word forms of the same instructions. Both operands arrive sign
 * extended, so the sum is exact in 32 bits and overflow is a comparison
 * against its own 16-bit truncation.
 */
static void gen_addsub16(TCGv dst, TCGv a, TCGv b, bool sub, bool sat)
{
    TCGv res = tcg_temp_new_i32();
    TCGv sx = tcg_temp_new_i32();
    TCGv ovf = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    tcg_gen_ext16u_i32(t, a);
    tcg_gen_ext16u_i32(sx, b);
    if (sub) {
        tcg_gen_sub_i32(res, a, b);
        tcg_gen_setcond_i32(TCG_COND_GEU, t, t, sx);
    } else {
        tcg_gen_add_i32(res, a, b);
        tcg_gen_add_i32(t, t, sx);
        tcg_gen_extract_i32(t, t, 16, 1);
    }
    gen_ac0_flag(t);

    tcg_gen_ext16s_i32(sx, res);
    tcg_gen_setcond_i32(TCG_COND_NE, ovf, sx, res);
    if (sat) {
        tcg_gen_smax_i32(res, res, tcg_constant_i32(INT16_MIN));
        tcg_gen_smin_i32(res, res, tcg_constant_i32(INT16_MAX));
    } else {
        tcg_gen_mov_i32(res, sx);
    }
    gen_v_flag(ovf);
    tcg_gen_mov_i32(dst, res);
    gen_logic_flags16(dst);
}

static bool gen_addsub_hw(int dst, bool dst_hi, int src0, bool s0_hi,
                          int src1, bool s1_hi, bool sub, bool sat)
{
    TCGv a = gen_half_s(src0, s0_hi);
    TCGv b = gen_half_s(src1, s1_hi);
    TCGv res = tcg_temp_new_i32();

    gen_addsub16(res, a, b, sub, sat);
    gen_put_half(dst, dst_hi, res);
    return true;
}

/* ---------------------------------------------------------------- */
/* Shifts                                                           */

/*
 * A shift whose magnitude is only known at run time. The magnitude is the
 * low six bits of the register sign extended, so it runs from -32 to +31:
 * positive shifts left and negative shifts right (chapter 14, "Arithmetic
 * Shift" and "Logical Shift"). src must arrive sign extended for an
 * arithmetic shift and zero extended for a logical one; the result is
 * returned truncated to width bits.
 *
 * The whole thing is evaluated 64 bits wide so that a magnitude of the full
 * register width behaves - a 32-bit TCG shift by 32 is undefined - and so
 * that the bits a left shift pushes past the top are still available for the
 * overflow test.
 */
static void gen_shift_by_mag(TCGv dst, TCGv src, TCGv magreg, int width,
                             bool arith, bool sat)
{
    TCGv_i64 v = tcg_temp_new_i64();
    TCGv_i64 left = tcg_temp_new_i64();
    TCGv_i64 right = tcg_temp_new_i64();
    TCGv_i64 sh = tcg_temp_new_i64();
    TCGv_i64 t64 = tcg_temp_new_i64();
    TCGv mag = tcg_temp_new_i32();
    TCGv cnt = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();
    TCGv rl = tcg_temp_new_i32();
    TCGv rr = tcg_temp_new_i32();
    TCGv ovf = tcg_temp_new_i32();
    TCGv zero = tcg_constant_i32(0);

    tcg_gen_shli_i32(mag, magreg, 26);
    tcg_gen_sari_i32(mag, mag, 26);
    tcg_gen_abs_i32(cnt, mag);
    tcg_gen_extu_i32_i64(sh, cnt);

    if (arith) {
        tcg_gen_ext_i32_i64(v, src);
        tcg_gen_sar_i64(right, v, sh);
    } else {
        tcg_gen_extu_i32_i64(v, src);
        tcg_gen_shr_i64(right, v, sh);
    }
    tcg_gen_shl_i64(left, v, sh);

    tcg_gen_extrl_i64_i32(rl, left);
    tcg_gen_extrl_i64_i32(rr, right);
    tcg_gen_movcond_i32(TCG_COND_GE, res, mag, zero, rl, rr);

    /*
     * Only a left shift can lose significant bits, and it reports the loss in
     * V whether it is written as the logical or the arithmetic form: a left
     * shift is a right shift by a negative magnitude, and the two forms differ
     * only in what a right shift feeds in at the top. A right shift clears V.
     *
     * What is pushed past the top of the field has to be either nothing at
     * all, or - for a result that is itself negative - that result's sign
     * extension. Note the first case stands alone: a positive value shifted
     * until it reads as negative has still lost no bits, and does not set V.
     */
    {
        TCGv_i64 out = tcg_temp_new_i64();
        TCGv_i64 fill = tcg_temp_new_i64();
        TCGv_i64 fits = tcg_temp_new_i64();
        TCGv_i64 neg = tcg_temp_new_i64();
        TCGv sign = tcg_temp_new_i32();

        if (width == 32) {
            tcg_gen_extu_i32_i64(t64, src);
        } else {
            tcg_gen_ext16u_i32(sign, src);
            tcg_gen_extu_i32_i64(t64, sign);
        }
        tcg_gen_shl_i64(out, t64, sh);
        tcg_gen_shri_i64(out, out, width);

        tcg_gen_shl_i64(fill, tcg_constant_i64(1), sh);
        tcg_gen_subi_i64(fill, fill, 1);

        tcg_gen_shri_i32(sign, res, width - 1);
        tcg_gen_andi_i32(sign, sign, 1);
        tcg_gen_extu_i32_i64(neg, sign);

        tcg_gen_setcond_i64(TCG_COND_EQ, fits, out, fill);
        tcg_gen_and_i64(fits, fits, neg);
        tcg_gen_setcondi_i64(TCG_COND_EQ, neg, out, 0);
        tcg_gen_or_i64(fits, fits, neg);

        tcg_gen_extrl_i64_i32(ovf, fits);
        tcg_gen_xori_i32(ovf, ovf, 1);
        tcg_gen_movcond_i32(TCG_COND_GE, ovf, mag, zero, ovf, zero);
    }

    if (sat) {
        tcg_gen_sari_i32(rl, src, 31);
        tcg_gen_xori_i32(rl, rl, width == 32 ? INT32_MAX : INT16_MAX);
        tcg_gen_movcond_i32(TCG_COND_NE, res, ovf, zero, rl, res);
    }
    gen_v_flag(ovf);
    if (width == 32) {
        tcg_gen_mov_i32(dst, res);
    } else {
        tcg_gen_ext16u_i32(dst, res);
    }
}

/*
 * The 16-bit shift-by-register instructions are different: the magnitude is
 * the whole 32 bits taken as unsigned, and anything past 31 gives all zeros
 * for a logical shift or all sign for an arithmetic one (tables 14-1 and
 * 14-2). These clear V.
 */
static void gen_shift_by_reg32(TCGv dst, TCGv src, TCGv mag, bool arith,
                               bool left)
{
    TCGv cnt = tcg_temp_new_i32();
    TCGv res = tcg_temp_new_i32();

    tcg_gen_umin_i32(cnt, mag, tcg_constant_i32(31));
    if (arith) {
        tcg_gen_sar_i32(res, src, cnt);
    } else if (left) {
        tcg_gen_shl_i32(res, src, cnt);
        tcg_gen_movcond_i32(TCG_COND_LTU, res, mag, tcg_constant_i32(32),
                            res, tcg_constant_i32(0));
    } else {
        tcg_gen_shr_i32(res, src, cnt);
        tcg_gen_movcond_i32(TCG_COND_LTU, res, mag, tcg_constant_i32(32),
                            res, tcg_constant_i32(0));
    }
    tcg_gen_mov_i32(dst, res);
    gen_logic_flags(dst);
    gen_v_flagi(0);
}

/* ---------------------------------------------------------------- */
/* Bit reversed carry addition, used by the BREV modify forms       */

static void gen_bitrev32(TCGv dst, TCGv src)
{
    TCGv v = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    tcg_gen_shri_i32(t, src, 1);
    tcg_gen_andi_i32(t, t, 0x55555555);
    tcg_gen_andi_i32(v, src, 0x55555555);
    tcg_gen_shli_i32(v, v, 1);
    tcg_gen_or_i32(v, v, t);

    tcg_gen_shri_i32(t, v, 2);
    tcg_gen_andi_i32(t, t, 0x33333333);
    tcg_gen_andi_i32(v, v, 0x33333333);
    tcg_gen_shli_i32(v, v, 2);
    tcg_gen_or_i32(v, v, t);

    tcg_gen_shri_i32(t, v, 4);
    tcg_gen_andi_i32(t, t, 0x0f0f0f0f);
    tcg_gen_andi_i32(v, v, 0x0f0f0f0f);
    tcg_gen_shli_i32(v, v, 4);
    tcg_gen_or_i32(v, v, t);

    tcg_gen_bswap32_i32(dst, v);
}

/*
 * (BREV) propagates the carry from the most significant bit towards the
 * least, figure 15-1. Reversing both operands turns that back into an
 * ordinary addition.
 */
static void gen_add_brev(TCGv dst, TCGv a, TCGv b)
{
    TCGv x = tcg_temp_new_i32();
    TCGv y = tcg_temp_new_i32();

    gen_bitrev32(x, a);
    gen_bitrev32(y, b);
    tcg_gen_add_i32(x, x, y);
    gen_bitrev32(dst, x);
}

/* ---------------------------------------------------------------- */
/* 16-bit shifts and shift-with-add                                 */

static bool trans_asr_d_d(DisasContext *ctx, arg_asr_d_d *a)
{
    gen_shift_by_reg32(dreg(a->dst), dreg(a->dst), dreg(a->src), true, false);
    return true;
}

static bool trans_lsr_d_d(DisasContext *ctx, arg_lsr_d_d *a)
{
    gen_shift_by_reg32(dreg(a->dst), dreg(a->dst), dreg(a->src), false, false);
    return true;
}

static bool trans_lsl_d_d(DisasContext *ctx, arg_lsl_d_d *a)
{
    gen_shift_by_reg32(dreg(a->dst), dreg(a->dst), dreg(a->src), false, true);
    return true;
}

/*
 * Add with Shift. Chapter 14 lists AZ, AN, V and VS and says the rest are
 * unaffected; table 2-11 of the hardware reference also marks AC0. The
 * enumeration wins, so AC0 is left alone here. The addition and the shift
 * are one operation as far as V is concerned, so the overflow test is
 * against the shifted result.
 */
static void gen_add_shift_d(int dst, int src, int sh)
{
    TCGv_i64 wide = tcg_temp_new_i64();
    TCGv_i64 t64 = tcg_temp_new_i64();
    TCGv res = tcg_temp_new_i32();
    TCGv ovf = tcg_temp_new_i32();

    tcg_gen_ext_i32_i64(wide, dreg(dst));
    tcg_gen_ext_i32_i64(t64, dreg(src));
    tcg_gen_add_i64(wide, wide, t64);
    tcg_gen_shli_i64(wide, wide, sh);

    tcg_gen_extrl_i64_i32(res, wide);
    tcg_gen_ext32s_i64(t64, wide);
    tcg_gen_setcond_i64(TCG_COND_NE, t64, t64, wide);
    tcg_gen_extrl_i64_i32(ovf, t64);

    gen_v_flag(ovf);
    tcg_gen_mov_i32(dreg(dst), res);
    gen_logic_flags(dreg(dst));
}

static bool trans_add_sh1_d(DisasContext *ctx, arg_add_sh1_d *a)
{
    gen_add_shift_d(a->dst, a->src, 1);
    return true;
}

static bool trans_add_sh2_d(DisasContext *ctx, arg_add_sh2_d *a)
{
    gen_add_shift_d(a->dst, a->src, 2);
    return true;
}

/* ---------------------------------------------------------------- */
/* Divide primitives                                                */

/*
 * DIVS seeds a non-restoring division: AQ becomes the sign the quotient
 * will have, and the dividend is shifted up one place with AQ in the vacated
 * bit. Chapter 15 lists AQ as the only status bit these two touch; table 2-9
 * of the hardware reference also marks AZ, AN, AC0 and AV0, which the
 * enumeration contradicts, so only AQ is written.
 */
static bool trans_divs(DisasContext *ctx, arg_divs *a)
{
    TCGv aq = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    tcg_gen_shri_i32(aq, dreg(a->dst), 31);
    tcg_gen_extract_i32(t, dreg(a->src), 15, 1);
    tcg_gen_xor_i32(aq, aq, t);
    gen_astat_set(R_ASTAT_AQ_SHIFT, aq);

    tcg_gen_shli_i32(t, dreg(a->dst), 1);
    tcg_gen_or_i32(dreg(a->dst), t, aq);
    return true;
}

/*
 * One DIVQ step adds the divisor when AQ is set and subtracts it when it is
 * clear, re-derives AQ from the sign of that intermediate result, and shifts
 * the dividend up with the inverse of AQ in the vacated bit. The divisor is
 * the low half of its register, aligned against the top of the dividend.
 */
static bool trans_divq(DisasContext *ctx, arg_divq *a)
{
    TCGv d = tcg_temp_new_i32();
    TCGv r = tcg_temp_new_i32();
    TCGv aq = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    tcg_gen_shli_i32(r, dreg(a->src), 16);
    tcg_gen_extract_i32(aq, cpu_astat, R_ASTAT_AQ_SHIFT, 1);
    tcg_gen_add_i32(t, dreg(a->dst), r);
    tcg_gen_sub_i32(d, dreg(a->dst), r);
    tcg_gen_movcond_i32(TCG_COND_NE, d, aq, tcg_constant_i32(0), t, d);

    tcg_gen_shri_i32(aq, d, 31);
    tcg_gen_extract_i32(t, dreg(a->src), 15, 1);
    tcg_gen_xor_i32(aq, aq, t);
    gen_astat_set(R_ASTAT_AQ_SHIFT, aq);

    tcg_gen_xori_i32(t, aq, 1);
    tcg_gen_shli_i32(d, d, 1);
    tcg_gen_or_i32(dreg(a->dst), d, t);
    return true;
}

/* ---------------------------------------------------------------- */
/* Negate                                                           */

/*
 * The 16-bit Negate does not saturate, so 0x80000000 negates to itself and
 * only sets V. Chapter 15 also has AC0 set when the source is zero, which
 * falls out of evaluating the instruction as 0 - src.
 */
static bool trans_neg_d(DisasContext *ctx, arg_neg_d *a)
{
    gen_addsub32(dreg(a->dst), tcg_constant_i32(0), dreg(a->src),
                 true, false, true);
    return true;
}

/* ---------------------------------------------------------------- */
/* Pointer arithmetic. None of this touches the status bits.        */

static bool trans_sub_p_p(DisasContext *ctx, arg_sub_p_p *a)
{
    tcg_gen_sub_i32(preg(a->dst), preg(a->dst), preg(a->src));
    return true;
}

static bool trans_lsl2_p(DisasContext *ctx, arg_lsl2_p *a)
{
    tcg_gen_shli_i32(preg(a->dst), preg(a->src), 2);
    return true;
}

static bool trans_lsr2_p(DisasContext *ctx, arg_lsr2_p *a)
{
    tcg_gen_shri_i32(preg(a->dst), preg(a->src), 2);
    return true;
}

static bool trans_lsr1_p(DisasContext *ctx, arg_lsr1_p *a)
{
    tcg_gen_shri_i32(preg(a->dst), preg(a->src), 1);
    return true;
}

static bool trans_add_brev_p(DisasContext *ctx, arg_add_brev_p *a)
{
    gen_add_brev(preg(a->dst), preg(a->dst), preg(a->src));
    return true;
}

static void gen_add_shift_p(int dst, int src, int sh)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_add_i32(t, preg(dst), preg(src));
    tcg_gen_shli_i32(preg(dst), t, sh);
}

static bool trans_add_sh1_p(DisasContext *ctx, arg_add_sh1_p *a)
{
    gen_add_shift_p(a->dst, a->src, 1);
    return true;
}

static bool trans_add_sh2_p(DisasContext *ctx, arg_add_sh2_p *a)
{
    gen_add_shift_p(a->dst, a->src, 2);
    return true;
}

static bool trans_add_p_p1(DisasContext *ctx, arg_add_p_p1 *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_shli_i32(t, preg(a->src1), 1);
    tcg_gen_add_i32(preg(a->dst), preg(a->src0), t);
    return true;
}

/* ---------------------------------------------------------------- */
/* Index register modify. No status bits, and no circular wrap: see */
/* the note on ireg() about the L registers.                        */

static bool trans_imod_i_m(DisasContext *ctx, arg_imod_i_m *a)
{
    tcg_gen_add_i32(ireg(a->ireg), ireg(a->ireg),
                    cpu_gpr[BFIN_REG_M0 + a->mreg]);
    return true;
}

static bool trans_dmod_i_m(DisasContext *ctx, arg_dmod_i_m *a)
{
    tcg_gen_sub_i32(ireg(a->ireg), ireg(a->ireg),
                    cpu_gpr[BFIN_REG_M0 + a->mreg]);
    return true;
}

static bool trans_imod_i_m_brev(DisasContext *ctx, arg_imod_i_m_brev *a)
{
    gen_add_brev(ireg(a->ireg), ireg(a->ireg),
                 cpu_gpr[BFIN_REG_M0 + a->mreg]);
    return true;
}

static bool trans_addimm_i2(DisasContext *ctx, arg_addimm_i2 *a)
{
    tcg_gen_addi_i32(ireg(a->ireg), ireg(a->ireg), 2);
    return true;
}

static bool trans_subimm_i2(DisasContext *ctx, arg_subimm_i2 *a)
{
    tcg_gen_subi_i32(ireg(a->ireg), ireg(a->ireg), 2);
    return true;
}

static bool trans_addimm_i4(DisasContext *ctx, arg_addimm_i4 *a)
{
    tcg_gen_addi_i32(ireg(a->ireg), ireg(a->ireg), 4);
    return true;
}

static bool trans_subimm_i4(DisasContext *ctx, arg_subimm_i4 *a)
{
    tcg_gen_subi_i32(ireg(a->ireg), ireg(a->ireg), 4);
    return true;
}

/* ---------------------------------------------------------------- */
/* 32-bit word add and subtract                                     */

static bool trans_add_d_d_ns(DisasContext *ctx, arg_add_d_d_ns *a)
{
    gen_addsub32(dreg(a->dst), dreg(a->src0), dreg(a->src1),
                 false, false, true);
    return true;
}

static bool trans_add_d_d_s(DisasContext *ctx, arg_add_d_d_s *a)
{
    gen_addsub32(dreg(a->dst), dreg(a->src0), dreg(a->src1),
                 false, true, true);
    return true;
}

static bool trans_sub_d_d_ns(DisasContext *ctx, arg_sub_d_d_ns *a)
{
    gen_addsub32(dreg(a->dst), dreg(a->src0), dreg(a->src1),
                 true, false, true);
    return true;
}

static bool trans_sub_d_d_s(DisasContext *ctx, arg_sub_d_d_s *a)
{
    gen_addsub32(dreg(a->dst), dreg(a->src0), dreg(a->src1),
                 true, true, true);
    return true;
}

/* ---------------------------------------------------------------- */
/* Half-word add and subtract. The untouched half of the            */
/* destination keeps its value.                                     */

#define HWADDSUB(name, dh, s0h, s1h, sub, sat)                          \
    static bool trans_##name(DisasContext *ctx, arg_##name *a)          \
    {                                                                   \
        return gen_addsub_hw(a->dst, dh, a->src0, s0h, a->src1, s1h,    \
                             sub, sat);                                 \
    }

HWADDSUB(add16_l_ll,   false, false, false, false, false)
HWADDSUB(add16_l_ll_s, false, false, false, false, true)
HWADDSUB(add16_l_lh,   false, false, true,  false, false)
HWADDSUB(add16_l_lh_s, false, false, true,  false, true)
HWADDSUB(add16_l_hl,   false, true,  false, false, false)
HWADDSUB(add16_l_hl_s, false, true,  false, false, true)
HWADDSUB(add16_l_hh,   false, true,  true,  false, false)
HWADDSUB(add16_l_hh_s, false, true,  true,  false, true)

HWADDSUB(add16_h_ll,   true,  false, false, false, false)
HWADDSUB(add16_h_ll_s, true,  false, false, false, true)
HWADDSUB(add16_h_lh,   true,  false, true,  false, false)
HWADDSUB(add16_h_lh_s, true,  false, true,  false, true)
HWADDSUB(add16_h_hl,   true,  true,  false, false, false)
HWADDSUB(add16_h_hl_s, true,  true,  false, false, true)
HWADDSUB(add16_h_hh,   true,  true,  true,  false, false)
HWADDSUB(add16_h_hh_s, true,  true,  true,  false, true)

HWADDSUB(sub16_l_ll,   false, false, false, true,  false)
HWADDSUB(sub16_l_ll_s, false, false, false, true,  true)
HWADDSUB(sub16_l_lh,   false, false, true,  true,  false)
HWADDSUB(sub16_l_lh_s, false, false, true,  true,  true)
HWADDSUB(sub16_l_hl,   false, true,  false, true,  false)
HWADDSUB(sub16_l_hl_s, false, true,  false, true,  true)
HWADDSUB(sub16_l_hh,   false, true,  true,  true,  false)
HWADDSUB(sub16_l_hh_s, false, true,  true,  true,  true)

HWADDSUB(sub16_h_ll,   true,  false, false, true,  false)
HWADDSUB(sub16_h_ll_s, true,  false, false, true,  true)
HWADDSUB(sub16_h_lh,   true,  false, true,  true,  false)
HWADDSUB(sub16_h_lh_s, true,  false, true,  true,  true)
HWADDSUB(sub16_h_hl,   true,  true,  false, true,  false)
HWADDSUB(sub16_h_hl_s, true,  true,  false, true,  true)
HWADDSUB(sub16_h_hh,   true,  true,  true,  true,  false)
HWADDSUB(sub16_h_hh_s, true,  true,  true,  true,  true)

#undef HWADDSUB

/* ---------------------------------------------------------------- */
/* Maximum, minimum, absolute value, negate                         */

static bool trans_max_d_d(DisasContext *ctx, arg_max_d_d *a)
{
    tcg_gen_smax_i32(dreg(a->dst), dreg(a->src0), dreg(a->src1));
    gen_logic_flags(dreg(a->dst));
    gen_v_flagi(0);
    return true;
}

static bool trans_min_d_d(DisasContext *ctx, arg_min_d_d *a)
{
    tcg_gen_smin_i32(dreg(a->dst), dreg(a->src0), dreg(a->src1));
    gen_logic_flags(dreg(a->dst));
    gen_v_flagi(0);
    return true;
}

/*
 * ABS saturates unconditionally, so the largest negative value comes back as
 * the largest positive one and reports that in V. AN is cleared rather than
 * derived, because the result can never be negative.
 */
static bool trans_abs_d(DisasContext *ctx, arg_abs_d *a)
{
    TCGv res = tcg_temp_new_i32();
    TCGv v = tcg_temp_new_i32();

    tcg_gen_abs_i32(res, dreg(a->src0));
    tcg_gen_setcondi_i32(TCG_COND_EQ, v, dreg(a->src0), INT32_MIN);
    tcg_gen_movcond_i32(TCG_COND_NE, res, v, tcg_constant_i32(0),
                        tcg_constant_i32(INT32_MAX), res);
    gen_v_flag(v);
    tcg_gen_mov_i32(dreg(a->dst), res);

    tcg_gen_setcondi_i32(TCG_COND_EQ, v, dreg(a->dst), 0);
    gen_astat_set(R_ASTAT_AZ_SHIFT, v);
    gen_astat_seti(R_ASTAT_AN_SHIFT, 0);
    return true;
}

static bool trans_neg_d_ns(DisasContext *ctx, arg_neg_d_ns *a)
{
    gen_addsub32(dreg(a->dst), tcg_constant_i32(0), dreg(a->src0),
                 true, false, true);
    return true;
}

static bool trans_neg_d_s(DisasContext *ctx, arg_neg_d_s *a)
{
    gen_addsub32(dreg(a->dst), tcg_constant_i32(0), dreg(a->src0),
                 true, true, true);
    return true;
}

/* ---------------------------------------------------------------- */
/* Quad 8-bit pack                                                  */

/*
 * Bytes 0 and 2 of the destination come from the low bytes of each half of
 * src0 and src1; tables 18-22 and 18-23. No status bits.
 */
static bool trans_bytepack(DisasContext *ctx, arg_bytepack *a)
{
    TCGv res = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    tcg_gen_andi_i32(res, dreg(a->src0), 0x000000ff);
    tcg_gen_shri_i32(t, dreg(a->src0), 8);
    tcg_gen_andi_i32(t, t, 0x0000ff00);
    tcg_gen_or_i32(res, res, t);
    tcg_gen_shli_i32(t, dreg(a->src1), 16);
    tcg_gen_andi_i32(t, t, 0x00ff0000);
    tcg_gen_or_i32(res, res, t);
    tcg_gen_shli_i32(t, dreg(a->src1), 8);
    tcg_gen_andi_i32(t, t, 0xff000000);
    tcg_gen_or_i32(dreg(a->dst), res, t);
    return true;
}

/* ---------------------------------------------------------------- */
/* Shifter, register magnitude                                      */

static void gen_shift16_by_mag(int dst, bool dst_hi, int src, bool src_hi,
                               int mag, bool arith, bool sat)
{
    TCGv v = arith ? gen_half_s(src, src_hi) : gen_half_u(src, src_hi);
    TCGv res = tcg_temp_new_i32();

    gen_shift_by_mag(res, v, dreg(mag), 16, arith, sat);
    gen_put_half(dst, dst_hi, res);
    gen_logic_flags16(res);
}

static bool trans_ashift16_l_l(DisasContext *ctx, arg_ashift16_l_l *a)
{
    gen_shift16_by_mag(a->dst, false, a->src, false, a->mag, true, false);
    return true;
}

static bool trans_ashift16_l_h(DisasContext *ctx, arg_ashift16_l_h *a)
{
    gen_shift16_by_mag(a->dst, false, a->src, true, a->mag, true, false);
    return true;
}

static void gen_shift32_by_mag(int dst, int src, int mag, bool arith,
                               bool sat)
{
    TCGv res = tcg_temp_new_i32();

    gen_shift_by_mag(res, dreg(src), dreg(mag), 32, arith, sat);
    tcg_gen_mov_i32(dreg(dst), res);
    gen_logic_flags(dreg(dst));
}

static bool trans_ashift_d(DisasContext *ctx, arg_ashift_d *a)
{
    gen_shift32_by_mag(a->dst, a->src, a->mag, true, false);
    return true;
}

static bool trans_ashift_d_s(DisasContext *ctx, arg_ashift_d_s *a)
{
    gen_shift32_by_mag(a->dst, a->src, a->mag, true, true);
    return true;
}

static bool trans_lshift_d(DisasContext *ctx, arg_lshift_d *a)
{
    gen_shift32_by_mag(a->dst, a->src, a->mag, false, false);
    return true;
}

/* ---------------------------------------------------------------- */
/* Rotate through CC                                                */

/*
 * The rotate chain is 33 bits, with CC above bit 31, and helper_rot already
 * models it for the constant-magnitude form. The registered magnitude is the
 * low six bits of the register sign extended, which is the same -32 to +31
 * range the helper expects. Nothing but CC is affected.
 */
static bool trans_rot_d_d(DisasContext *ctx, arg_rot_d_d *a)
{
    TCGv mag = tcg_temp_new_i32();

    tcg_gen_shli_i32(mag, dreg(a->mag), 26);
    tcg_gen_sari_i32(mag, mag, 26);
    gen_helper_rot(dreg(a->dst), tcg_env, dreg(a->src), mag);
    return true;
}

/* ---------------------------------------------------------------- */
/* Sign bit and bit fields                                          */

/*
 * SIGNBITS returns the number of leading sign bits minus one, so an input of
 * all zeros or all ones gives 31. Exclusive-oring the value with its own
 * sign turns that into a leading zero count. No status bits.
 */
static bool trans_signbits_d(DisasContext *ctx, arg_signbits_d *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_sari_i32(t, dreg(a->src), 31);
    tcg_gen_xor_i32(t, t, dreg(a->src));
    tcg_gen_clzi_i32(t, t, 32);
    tcg_gen_subi_i32(t, t, 1);
    gen_put_half(a->dst, false, t);
    return true;
}

/*
 * DEPOSIT writes the top half of the foreground register over the background
 * at the position and length the foreground's low half carries, table 13-1.
 * Lengths of 16 and above are clamped to 16, and foreground bits pushed past
 * bit 31 are dropped.
 */
static bool trans_deposit(DisasContext *ctx, arg_deposit *a)
{
    TCGv pos = tcg_temp_new_i32();
    TCGv len = tcg_temp_new_i32();
    TCGv mask = tcg_temp_new_i32();
    TCGv val = tcg_temp_new_i32();

    tcg_gen_extract_i32(pos, dreg(a->fg), 8, 5);
    tcg_gen_extract_i32(len, dreg(a->fg), 0, 5);
    tcg_gen_umin_i32(len, len, tcg_constant_i32(16));
    tcg_gen_shl_i32(mask, tcg_constant_i32(1), len);
    tcg_gen_subi_i32(mask, mask, 1);

    tcg_gen_shri_i32(val, dreg(a->fg), 16);
    tcg_gen_and_i32(val, val, mask);
    tcg_gen_shl_i32(val, val, pos);
    tcg_gen_shl_i32(mask, mask, pos);
    tcg_gen_andc_i32(mask, dreg(a->bg), mask);
    tcg_gen_or_i32(dreg(a->dst), mask, val);

    gen_logic_flags(dreg(a->dst));
    gen_ac0_flagi(0);
    gen_v_flagi(0);
    return true;
}

/* ---------------------------------------------------------------- */
/* Shifter, constant magnitude                                      */

/*
 * Bits 8-3 of the second word are one signed magnitude, so each direction of
 * a shift is a separate row with different fixed high bits and the decoded
 * immediate is only the part that varies. Rebuild the six-bit field and hand
 * it to the same code path the register magnitude forms use.
 */
static void gen_shift32_by_mag_imm(int dst, int src, int field, bool arith,
                                   bool sat)
{
    TCGv res = tcg_temp_new_i32();

    gen_shift_by_mag(res, dreg(src), tcg_constant_i32(field), 32, arith,
                     sat);
    tcg_gen_mov_i32(dreg(dst), res);
    gen_logic_flags(dreg(dst));
}

static bool trans_asr_imm32(DisasContext *ctx, arg_asr_imm32 *a)
{
    gen_shift32_by_mag_imm(a->dst, a->src, 0x20 | a->imm, true, false);
    return true;
}

static bool trans_lsl_imm32(DisasContext *ctx, arg_lsl_imm32 *a)
{
    gen_shift32_by_mag_imm(a->dst, a->src, a->imm, false, false);
    return true;
}

static bool trans_lsr_imm32(DisasContext *ctx, arg_lsr_imm32 *a)
{
    gen_shift32_by_mag_imm(a->dst, a->src, 0x20 | a->imm, false, false);
    return true;
}

static void gen_shift16_by_mag_imm(int dst, bool dst_hi, int src,
                                   bool src_hi, int field)
{
    TCGv v = gen_half_u(src, src_hi);
    TCGv res = tcg_temp_new_i32();

    gen_shift_by_mag(res, v, tcg_constant_i32(field), 16, false, false);
    gen_put_half(dst, dst_hi, res);
    gen_logic_flags16(res);
}

#define HWLSHIFT(name, dh, sh, field)                                   \
    static bool trans_##name(DisasContext *ctx, arg_##name *a)          \
    {                                                                   \
        gen_shift16_by_mag_imm(a->dst, dh, a->src, sh, field);          \
        return true;                                                    \
    }

HWLSHIFT(lsl16_l_l, false, false, a->imm)
HWLSHIFT(lsr16_l_l, false, false, 0x30 | a->imm)
HWLSHIFT(lsl16_l_h, false, true,  a->imm)
HWLSHIFT(lsr16_l_h, false, true,  0x30 | a->imm)
HWLSHIFT(lsl16_h_l, true,  false, a->imm)
HWLSHIFT(lsr16_h_l, true,  false, 0x30 | a->imm)
HWLSHIFT(lsl16_h_h, true,  true,  a->imm)
HWLSHIFT(lsr16_h_h, true,  true,  0x30 | a->imm)

#undef HWLSHIFT

/*
 * The vector forms shift the two halves independently by a constant, so the
 * bits that would cross between them are masked off first. AZ is set if
 * either half is zero and AN if either is negative; V is cleared.
 */
static void gen_lshift_v(int dst, int src, int n, bool left)
{
    TCGv res = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();
    TCGv az = tcg_temp_new_i32();
    uint32_t lane = left ? (0xffffu >> n) : ((0xffffu << n) & 0xffff);
    uint32_t mask = lane | (lane << 16);

    tcg_gen_andi_i32(res, dreg(src), mask);
    if (left) {
        tcg_gen_shli_i32(res, res, n);
    } else {
        tcg_gen_shri_i32(res, res, n);
    }
    tcg_gen_mov_i32(dreg(dst), res);

    tcg_gen_andi_i32(t, res, 0x0000ffff);
    tcg_gen_setcondi_i32(TCG_COND_EQ, az, t, 0);
    tcg_gen_andi_i32(t, res, 0xffff0000);
    tcg_gen_setcondi_i32(TCG_COND_EQ, t, t, 0);
    tcg_gen_or_i32(az, az, t);
    gen_astat_set(R_ASTAT_AZ_SHIFT, az);

    tcg_gen_andi_i32(t, res, 0x80008000);
    tcg_gen_setcondi_i32(TCG_COND_NE, t, t, 0);
    gen_astat_set(R_ASTAT_AN_SHIFT, t);
    gen_v_flagi(0);
}

static bool trans_lsl_imm_v(DisasContext *ctx, arg_lsl_imm_v *a)
{
    gen_lshift_v(a->dst, a->src, a->imm, true);
    return true;
}

static bool trans_lsr_imm_v(DisasContext *ctx, arg_lsr_imm_v *a)
{
    /* The encoded field is the two's complement of the count in 16. */
    gen_lshift_v(a->dst, a->src, 16 - a->imm, false);
    return true;
}

/* ---------------------------------------------------------------- */
/* Accumulator shifts                                               */

/*
 * A0 and A1 are 40 bits held right aligned in env->a[]. These three rows are
 * all right shifts, by 32 down to 1 places. Chapter 14 gives AZ and AN from
 * the 40-bit result; the arithmetic form cannot overflow on a right shift
 * and the logical form clears the accumulator overflow bit outright, so
 * either way AV0 or AV1 ends up clear. The sticky AV0S and AV1S are left
 * alone.
 */
static void gen_acc_shift_right(int accno, int cnt, bool arith)
{
    TCGv_i64 v = tcg_temp_new_i64();
    TCGv_i64 t = tcg_temp_new_i64();
    TCGv f = tcg_temp_new_i32();

    tcg_gen_ld_i64(v, tcg_env, offsetof(CPUBfinState, a[0]) +
                   accno * sizeof(uint64_t));
    if (arith) {
        tcg_gen_shli_i64(v, v, 24);
        tcg_gen_sari_i64(v, v, 24);
        tcg_gen_sari_i64(v, v, cnt);
    } else {
        tcg_gen_andi_i64(v, v, 0xffffffffffULL);
        tcg_gen_shri_i64(v, v, cnt);
    }
    tcg_gen_andi_i64(v, v, 0xffffffffffULL);
    tcg_gen_st_i64(v, tcg_env, offsetof(CPUBfinState, a[0]) +
                   accno * sizeof(uint64_t));

    tcg_gen_setcondi_i64(TCG_COND_EQ, t, v, 0);
    tcg_gen_extrl_i64_i32(f, t);
    gen_astat_set(R_ASTAT_AZ_SHIFT, f);
    tcg_gen_extract_i64(t, v, 39, 1);
    tcg_gen_extrl_i64_i32(f, t);
    gen_astat_set(R_ASTAT_AN_SHIFT, f);
    gen_astat_seti(accno ? R_ASTAT_AV1_SHIFT : R_ASTAT_AV0_SHIFT, 0);
}

static bool trans_asr_imm_a0(DisasContext *ctx, arg_asr_imm_a0 *a)
{
    gen_acc_shift_right(0, 32 - a->imm, true);
    return true;
}

static bool trans_lsr_imm_a0(DisasContext *ctx, arg_lsr_imm_a0 *a)
{
    gen_acc_shift_right(0, 32 - a->imm, false);
    return true;
}

static bool trans_lsr_imm_a1(DisasContext *ctx, arg_lsr_imm_a1 *a)
{
    gen_acc_shift_right(1, 32 - a->imm, false);
    return true;
}

/* ================================================================ */
/*                                                                  */
/* Everything below this line REPLACES a same-named function that   */
/* is already in translate.c. Delete the old ones when merging, or  */
/* the build will fail on the duplicate definitions - which is the  */
/* intent, because the versions being replaced are wrong. The       */
/* matching decode changes are in the last section of a4.decode.    */
/*                                                                  */
/* ================================================================ */

/*
 * The three-operand 16-bit forms put their destination in bits 8-6, not
 * 2-0: table C-17 gives "Dest / Src 1 / Src 0" for 0x5000, 0x5200 and
 * 0x5A00 and table C-16 gives the same for 0x5C00 and 0x5E00, and the GUI
 * image agrees - over its 1391 0x5000-form adds the two low fields are never
 * equal to each other while the top one matches one of them three times in
 * four, which only holds if the top field is the destination. The operation
 * is dst = src0 OP src1 with src0 in bits 2-0, so for a subtract bits 2-0
 * hold the minuend; the image shows the destination register matching bits
 * 2-0 twice as often as bits 5-3 across 0x5200, which is the x = x - y
 * idiom and pins the operand order down.
 */
static bool trans_add_d_d(DisasContext *ctx, arg_add_d_d *a)
{
    gen_addsub32(dreg(a->dst), dreg(a->src0), dreg(a->src1),
                 false, false, true);
    return true;
}

static bool trans_sub_d_d(DisasContext *ctx, arg_sub_d_d *a)
{
    gen_addsub32(dreg(a->dst), dreg(a->src0), dreg(a->src1),
                 true, false, true);
    return true;
}

/* The logical operations clear AC0 and V; chapter 12, "& (AND)". */
static void gen_logic_op_flags(TCGv res)
{
    gen_logic_flags(res);
    gen_ac0_flagi(0);
    gen_v_flagi(0);
}

static bool trans_and_d_d(DisasContext *ctx, arg_and_d_d *a)
{
    tcg_gen_and_i32(dreg(a->dst), dreg(a->src0), dreg(a->src1));
    gen_logic_op_flags(dreg(a->dst));
    return true;
}

static bool trans_or_d_d(DisasContext *ctx, arg_or_d_d *a)
{
    tcg_gen_or_i32(dreg(a->dst), dreg(a->src0), dreg(a->src1));
    gen_logic_op_flags(dreg(a->dst));
    return true;
}

static bool trans_xor_d_d(DisasContext *ctx, arg_xor_d_d *a)
{
    tcg_gen_xor_i32(dreg(a->dst), dreg(a->src0), dreg(a->src1));
    gen_logic_op_flags(dreg(a->dst));
    return true;
}

/*
 * 0x5A00 is the Add instruction Preg = Preg + Preg. The Logical Shift
 * spelling Preg = Preg << 1 is the special case where both sources name the
 * same register, so one translator covers both and the old one - which
 * doubled bits 5-3 and ignored bits 2-0 - was wrong whenever the two
 * sources differed. Rename shl_p1 to add_p_p in insn16.decode.
 */
static bool trans_add_p_p(DisasContext *ctx, arg_add_p_p *a)
{
    tcg_gen_add_i32(preg(a->dst), preg(a->src0), preg(a->src1));
    return true;
}

static bool trans_add_p_p2(DisasContext *ctx, arg_add_p_p2 *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_shli_i32(t, preg(a->src1), 2);
    tcg_gen_add_i32(preg(a->dst), preg(a->src0), t);
    return true;
}

/* Add Immediate to a Dreg also reports carry and overflow; chapter 15. */
static bool trans_addimm_d7(DisasContext *ctx, arg_addimm_d7 *a)
{
    gen_addsub32(dreg(a->rd), dreg(a->rd), tcg_constant_i32(a->imm),
                 false, false, true);
    return true;
}

/*
 * The bit operations clear AC0 and V as well as setting AZ and AN; table
 * 2-11 of the hardware reference and chapter 13. BITSET's AZ is documented
 * as cleared outright, which is what the generic test produces anyway
 * because setting a bit cannot leave the register zero.
 */
static bool trans_bitset(DisasContext *ctx, arg_bitset *a)
{
    tcg_gen_ori_i32(dreg(a->rd), dreg(a->rd), 1u << a->imm);
    gen_logic_op_flags(dreg(a->rd));
    return true;
}

static bool trans_bitclr(DisasContext *ctx, arg_bitclr *a)
{
    tcg_gen_andi_i32(dreg(a->rd), dreg(a->rd), ~(1u << a->imm));
    gen_logic_op_flags(dreg(a->rd));
    return true;
}

static bool trans_bittgl(DisasContext *ctx, arg_bittgl *a)
{
    tcg_gen_xori_i32(dreg(a->rd), dreg(a->rd), 1u << a->imm);
    gen_logic_op_flags(dreg(a->rd));
    return true;
}

/*
 * The constant-magnitude shifts leave AC0 alone but do clear V: a right
 * shift cannot overflow and the left shift here is the logical one. Table
 * 2-11 marks all three as clearing V.
 */
static bool trans_asr_imm(DisasContext *ctx, arg_asr_imm *a)
{
    tcg_gen_sari_i32(dreg(a->rd), dreg(a->rd), a->imm);
    gen_logic_flags(dreg(a->rd));
    gen_v_flagi(0);
    return true;
}

static bool trans_lsr_imm(DisasContext *ctx, arg_lsr_imm *a)
{
    tcg_gen_shri_i32(dreg(a->rd), dreg(a->rd), a->imm);
    gen_logic_flags(dreg(a->rd));
    gen_v_flagi(0);
    return true;
}

static bool trans_lsl_imm(DisasContext *ctx, arg_lsl_imm *a)
{
    tcg_gen_shli_i32(dreg(a->rd), dreg(a->rd), a->imm);
    gen_logic_flags(dreg(a->rd));
    gen_v_flagi(0);
    return true;
}

