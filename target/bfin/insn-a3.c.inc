/*
 * Analog Devices Blackfin move and condition-code translation
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Semantics are from the Blackfin Processor Programming Reference rev 2.2,
 * chapters 9 (Move), 11 (Control Code Bit Management) and 16 (External Event
 * Management). Encodings live in the matching .decode fragment.
 */

/* ---------------------------------------------------------------- */
/* Compare                                                          */

/* The signed forms and the flag rules they share are in translate.c. */

static bool trans_cc_lt_dreg_iu(DisasContext *ctx, arg_cc_lt_dreg_iu *a)
{
    return gen_cmp_dreg(TCG_COND_LTU, dreg(a->dst), dreg(a->src), false);
}

static bool trans_cc_le_dreg_iu(DisasContext *ctx, arg_cc_le_dreg_iu *a)
{
    return gen_cmp_dreg(TCG_COND_LEU, dreg(a->dst), dreg(a->src), false);
}

/*
 * The immediate is signed for ==, < and <=, and unsigned for the two forms
 * the syntax marks (IU); the decode fragment sign extends only the former.
 */
static bool trans_cc_lt_imm_iu(DisasContext *ctx, arg_cc_lt_imm_iu *a)
{
    return gen_cmp_dreg(TCG_COND_LTU, dreg(a->dst),
                        tcg_constant_i32(a->imm), false);
}

static bool trans_cc_le_imm_iu(DisasContext *ctx, arg_cc_le_imm_iu *a)
{
    return gen_cmp_dreg(TCG_COND_LEU, dreg(a->dst),
                        tcg_constant_i32(a->imm), false);
}

static bool trans_cc_eq_preg(DisasContext *ctx, arg_cc_eq_preg *a)
{
    return gen_cc_cmp(TCG_COND_EQ, preg(a->dst), preg(a->src));
}

static bool trans_cc_lt_preg(DisasContext *ctx, arg_cc_lt_preg *a)
{
    return gen_cc_cmp(TCG_COND_LT, preg(a->dst), preg(a->src));
}

static bool trans_cc_le_preg(DisasContext *ctx, arg_cc_le_preg *a)
{
    return gen_cc_cmp(TCG_COND_LE, preg(a->dst), preg(a->src));
}

static bool trans_cc_lt_preg_iu(DisasContext *ctx, arg_cc_lt_preg_iu *a)
{
    return gen_cc_cmp(TCG_COND_LTU, preg(a->dst), preg(a->src));
}

static bool trans_cc_le_preg_iu(DisasContext *ctx, arg_cc_le_preg_iu *a)
{
    return gen_cc_cmp(TCG_COND_LEU, preg(a->dst), preg(a->src));
}

static bool trans_cc_eq_imm_p(DisasContext *ctx, arg_cc_eq_imm_p *a)
{
    return gen_cc_cmp(TCG_COND_EQ, preg(a->dst), tcg_constant_i32(a->imm));
}

static bool trans_cc_lt_imm_p(DisasContext *ctx, arg_cc_lt_imm_p *a)
{
    return gen_cc_cmp(TCG_COND_LT, preg(a->dst), tcg_constant_i32(a->imm));
}

static bool trans_cc_le_imm_p(DisasContext *ctx, arg_cc_le_imm_p *a)
{
    return gen_cc_cmp(TCG_COND_LE, preg(a->dst), tcg_constant_i32(a->imm));
}

static bool trans_cc_lt_imm_p_iu(DisasContext *ctx, arg_cc_lt_imm_p_iu *a)
{
    return gen_cc_cmp(TCG_COND_LTU, preg(a->dst), tcg_constant_i32(a->imm));
}

static bool trans_cc_le_imm_p_iu(DisasContext *ctx, arg_cc_le_imm_p_iu *a)
{
    return gen_cc_cmp(TCG_COND_LEU, preg(a->dst), tcg_constant_i32(a->imm));
}

/* ---------------------------------------------------------------- */
/* Move CC to and from a status bit                                 */

/*
 * ASTAT bits 2 and 3 are the ADSP-BF535 positions of AC0 and V and are
 * defined to read back identical to bits 12 and 24 (figure 2-8 of the BF533
 * Hardware Reference rev 3.6, which annotates them "identical to bit 12" and
 * "identical to bit 24"). Reads use the modern position and writes update
 * both, so the two halves of each pair cannot drift apart.
 */
static int astat_canon_bit(int bit)
{
    switch (bit) {
    case R_ASTAT_AC0_COPY_SHIFT:
        return R_ASTAT_AC0_SHIFT;
    case R_ASTAT_V_COPY_SHIFT:
        return R_ASTAT_V_SHIFT;
    default:
        return bit;
    }
}

/*
 * ASTAT.CC is held out of line in env->cc because it is hot, which leaves
 * bit 5 of the packed register stale; bfin_pack_astat() and
 * bfin_unpack_astat() reconcile the two whenever the whole register moves.
 * A statbit operand names one ASTAT bit by number, so the same reconciliation
 * applies here a bit at a time. CC is not a legal statbit operand (Move CC,
 * "Syntax Terminology"), but the bit number is a plain five bit field and
 * nothing in the encoding prevents it.
 */
static void gen_statbit_read(TCGv dst, int bit)
{
    if (bit == R_ASTAT_CC_SHIFT) {
        tcg_gen_andi_i32(dst, cpu_cc, 1);
    } else {
        tcg_gen_extract_i32(dst, cpu_astat, astat_canon_bit(bit), 1);
    }
}

static void gen_statbit_write(int bit, TCGv val)
{
    if (bit == R_ASTAT_CC_SHIFT) {
        tcg_gen_andi_i32(cpu_cc, val, 1);
        return;
    }
    tcg_gen_deposit_i32(cpu_astat, cpu_astat, val, astat_canon_bit(bit), 1);
    switch (astat_canon_bit(bit)) {
    case R_ASTAT_AC0_SHIFT:
        tcg_gen_deposit_i32(cpu_astat, cpu_astat, val,
                            R_ASTAT_AC0_COPY_SHIFT, 1);
        break;
    case R_ASTAT_V_SHIFT:
        tcg_gen_deposit_i32(cpu_astat, cpu_astat, val,
                            R_ASTAT_V_COPY_SHIFT, 1);
        break;
    default:
        break;
    }
}

static bool trans_cc_from_stat(DisasContext *ctx, arg_cc_from_stat *a)
{
    gen_statbit_read(cpu_cc, a->bit);
    return true;
}

/*
 * CC only ever holds 0 or 1, so the bitwise forms need no masking.
 */
static bool trans_cc_or_stat(DisasContext *ctx, arg_cc_or_stat *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_statbit_read(t, a->bit);
    tcg_gen_or_i32(cpu_cc, cpu_cc, t);
    return true;
}

static bool trans_cc_and_stat(DisasContext *ctx, arg_cc_and_stat *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_statbit_read(t, a->bit);
    tcg_gen_and_i32(cpu_cc, cpu_cc, t);
    return true;
}

static bool trans_cc_xor_stat(DisasContext *ctx, arg_cc_xor_stat *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_statbit_read(t, a->bit);
    tcg_gen_xor_i32(cpu_cc, cpu_cc, t);
    return true;
}

static bool trans_stat_from_cc(DisasContext *ctx, arg_stat_from_cc *a)
{
    gen_statbit_write(a->bit, cpu_cc);
    return true;
}

static bool trans_stat_or_cc(DisasContext *ctx, arg_stat_or_cc *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_statbit_read(t, a->bit);
    tcg_gen_or_i32(t, t, cpu_cc);
    gen_statbit_write(a->bit, t);
    return true;
}

static bool trans_stat_and_cc(DisasContext *ctx, arg_stat_and_cc *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_statbit_read(t, a->bit);
    tcg_gen_and_i32(t, t, cpu_cc);
    gen_statbit_write(a->bit, t);
    return true;
}

static bool trans_stat_xor_cc(DisasContext *ctx, arg_stat_xor_cc *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_statbit_read(t, a->bit);
    tcg_gen_xor_i32(t, t, cpu_cc);
    gen_statbit_write(a->bit, t);
    return true;
}

/* ---------------------------------------------------------------- */
/* Move Conditional                                                 */

/*
 * Move Conditional affects no status bits, so the untaken case has to leave
 * the destination alone rather than write it back through gen_logic_flags().
 * A select keeps the block straight-line.
 */
static bool gen_movecc(bool want, TCGv dst, TCGv src)
{
    tcg_gen_movcond_i32(want ? TCG_COND_NE : TCG_COND_EQ, dst,
                        cpu_cc, tcg_constant_i32(0), src, dst);
    return true;
}

static bool trans_movcc_d_d(DisasContext *ctx, arg_movcc_d_d *a)
{
    return gen_movecc(true, dreg(a->dst), dreg(a->src));
}

static bool trans_movcc_d_p(DisasContext *ctx, arg_movcc_d_p *a)
{
    return gen_movecc(true, dreg(a->dst), preg(a->src));
}

static bool trans_movcc_p_d(DisasContext *ctx, arg_movcc_p_d *a)
{
    return gen_movecc(true, preg(a->dst), dreg(a->src));
}

static bool trans_movcc_p_p(DisasContext *ctx, arg_movcc_p_p *a)
{
    return gen_movecc(true, preg(a->dst), preg(a->src));
}

static bool trans_movncc_d_d(DisasContext *ctx, arg_movncc_d_d *a)
{
    return gen_movecc(false, dreg(a->dst), dreg(a->src));
}

static bool trans_movncc_d_p(DisasContext *ctx, arg_movncc_d_p *a)
{
    return gen_movecc(false, dreg(a->dst), preg(a->src));
}

static bool trans_movncc_p_d(DisasContext *ctx, arg_movncc_p_d *a)
{
    return gen_movecc(false, preg(a->dst), dreg(a->src));
}

static bool trans_movncc_p_p(DisasContext *ctx, arg_movncc_p_p *a)
{
    return gen_movecc(false, preg(a->dst), preg(a->src));
}

/* ---------------------------------------------------------------- */
/* Move with extension                                              */

/*
 * Move Half to Full Word and Move Byte both clear AC0 and V on top of the
 * usual AZ and AN ("Status Bits Affected" of each). The copies of AC0 and V
 * have to follow. AN comes out of gen_logic_flags() as zero for the (Z)
 * forms without a special case, because a zero extended result has bit 31
 * clear, which is what the manual asks for.
 */
static void gen_move_ext_flags(TCGv res)
{
    gen_logic_flags(res);
    tcg_gen_andi_i32(cpu_astat, cpu_astat,
                     ~(R_ASTAT_AC0_MASK | R_ASTAT_AC0_COPY_MASK |
                       R_ASTAT_V_MASK | R_ASTAT_V_COPY_MASK));
}

static bool trans_mov_dl_x(DisasContext *ctx, arg_mov_dl_x *a)
{
    tcg_gen_ext16s_i32(dreg(a->dst), dreg(a->src));
    gen_move_ext_flags(dreg(a->dst));
    return true;
}

static bool trans_mov_dl_z(DisasContext *ctx, arg_mov_dl_z *a)
{
    tcg_gen_ext16u_i32(dreg(a->dst), dreg(a->src));
    gen_move_ext_flags(dreg(a->dst));
    return true;
}

static bool trans_mov_db_x(DisasContext *ctx, arg_mov_db_x *a)
{
    tcg_gen_ext8s_i32(dreg(a->dst), dreg(a->src));
    gen_move_ext_flags(dreg(a->dst));
    return true;
}

static bool trans_mov_db_z(DisasContext *ctx, arg_mov_db_z *a)
{
    tcg_gen_ext8u_i32(dreg(a->dst), dreg(a->src));
    gen_move_ext_flags(dreg(a->dst));
    return true;
}

/* ---------------------------------------------------------------- */
/* Move Register Half                                               */

static bool trans_mov_dl_a0x(DisasContext *ctx, arg_mov_dl_a0x *a)
{
    TCGv_i64 acc = tcg_temp_new_i64();
    TCGv t = tcg_temp_new_i32();

    /*
     * A0.X is the top eight bits of the 40-bit accumulator and is sign
     * extended into the low half of the destination, whose high half is
     * untouched. This version of Move Register Half affects no status bits.
     */
    tcg_gen_ld_i64(acc, tcg_env, offsetof(CPUBfinState, a[0]));
    tcg_gen_extrh_i64_i32(t, acc);
    tcg_gen_ext8s_i32(t, t);
    tcg_gen_deposit_i32(dreg(a->rd), dreg(a->rd), t, 0, 16);
    return true;
}

/* ---------------------------------------------------------------- */
/* Test and Set Byte (Atomic)                                       */

static bool trans_testset(DisasContext *ctx, arg_testset *a)
{
    TCGv old;

    /*
     * SP and FP are not allowed as the pointer, which is why the documented
     * opcode range stops at 0x00B5 rather than at the end of the register
     * field. The load, the test and the set of bit 7 are one atomic
     * transaction; CC reports whether the byte was zero beforehand.
     */
    if (a->pb >= 6) {
        return false;
    }
    old = tcg_temp_new_i32();
    tcg_gen_atomic_fetch_or_i32(old, preg(a->pb), tcg_constant_i32(0x80), 0,
                                MO_UB);
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cc, old, 0);
    return true;
}
