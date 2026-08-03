/*
 * Analog Devices Blackfin translation: 32-bit load and store, hardware loops,
 * and the remaining program flow and no-op forms.
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Semantics are from the Blackfin Processor Programming Reference rev 2.2.
 * Instruction encodings live in insn16.decode and insn32.decode.
 */

/* ---------------------------------------------------------------- */
/* Zero overhead loops                                              */

/*
 * LTx and LBx are written before LCx because that is the order the manual
 * requires when the three registers are set up by hand, and LSETUP is defined
 * as the one-instruction spelling of the same sequence (chapter 7, "LSETUP,
 * LOOP"). Both offsets are unsigned and forward-only: LT is 4 to 30 bytes
 * ahead and LB 4 to 2046, each stored pre-divided by two.
 */
static bool gen_lsetup(DisasContext *ctx, int n, int spc, int epc,
                       int preg_no, bool from_preg, bool halve)
{
    tcg_gen_movi_i32(cpu_lt[n], ctx->pc + spc * 2);
    tcg_gen_movi_i32(cpu_lb[n], ctx->pc + epc * 2);
    if (from_preg) {
        /*
         * LCx holds an unsigned 32-bit trip count, so the ">> 1" spelling
         * used to halve a count of bytes into a count of half words is a
         * logical shift. SP and FP are not allowed as the source.
         */
        if (halve) {
            tcg_gen_shri_i32(cpu_lc[n], preg(preg_no & 7), 1);
        } else {
            tcg_gen_mov_i32(cpu_lc[n], preg(preg_no & 7));
        }
    }
    /*
     * The loop registers are part of the translation key, so a block that
     * starts a loop must end here: what follows executes with the loop live.
     */
    ctx->base.is_jmp = DISAS_UPDATE;
    return true;
}

static bool trans_lsetup0(DisasContext *ctx, arg_lsetup0 *a)
{
    return gen_lsetup(ctx, 0, a->spc, a->epc, 0, false, false);
}

static bool trans_lsetup1(DisasContext *ctx, arg_lsetup1 *a)
{
    return gen_lsetup(ctx, 1, a->spc, a->epc, 0, false, false);
}

static bool trans_lsetup0_p(DisasContext *ctx, arg_lsetup0_p *a)
{
    return gen_lsetup(ctx, 0, a->spc, a->epc, a->preg, true, false);
}

static bool trans_lsetup1_p(DisasContext *ctx, arg_lsetup1_p *a)
{
    return gen_lsetup(ctx, 1, a->spc, a->epc, a->preg, true, false);
}

static bool trans_lsetup0_ph(DisasContext *ctx, arg_lsetup0_ph *a)
{
    return gen_lsetup(ctx, 0, a->spc, a->epc, a->preg, true, true);
}

static bool trans_lsetup1_ph(DisasContext *ctx, arg_lsetup1_ph *a)
{
    return gen_lsetup(ctx, 1, a->spc, a->epc, a->preg, true, true);
}

/* ---------------------------------------------------------------- */
/* Load and store with a large displacement                         */

/*
 * Bit 15 of the second word chooses whether the displacement is added or
 * subtracted and bits 14-0 hold its unsigned magnitude, pre-divided by the
 * access size. The magnitudes are therefore 0 to 131068 for a word, 0 to
 * 65534 for a half word and 0 to 32767 for a byte, which is exactly what the
 * manual calls uimm17m4, uimm16m2 and uimm15 (chapter 8, "Load Data
 * Register", "Load Half Word" and "Load Byte" syntax terminology). The pair
 * of translators per form matches the pair of rows per form in table C-10.
 *
 * Pointer arithmetic does not touch the arithmetic status bits, and neither
 * does a load, so nothing here sets ASTAT.
 */
static bool trans_ld_d_p_off17(DisasContext *ctx, arg_ld_d_p_off17 *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_ld_p_p_off17(DisasContext *ctx, arg_ld_p_p_off17 *a)
{
    gen_load(ctx, preg(a->rd), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_st_p_off17_d(DisasContext *ctx, arg_st_p_off17_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_st_p_off17_p(DisasContext *ctx, arg_st_p_off17_p *a)
{
    gen_store(ctx, preg(a->rs), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_ldw_d_p_off16_z(DisasContext *ctx, arg_ldw_d_p_off16_z *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off * 2, MO_LEUW);
    return true;
}

static bool trans_ldw_d_p_off16_x(DisasContext *ctx, arg_ldw_d_p_off16_x *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off * 2, MO_LESW);
    return true;
}

/* Only the low half of the source register reaches memory. */
static bool trans_stw_p_off16_d(DisasContext *ctx, arg_stw_p_off16_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), a->off * 2, MO_LEUW);
    return true;
}

static bool trans_ldb_d_p_off15_z(DisasContext *ctx, arg_ldb_d_p_off15_z *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off, MO_UB);
    return true;
}

static bool trans_ldb_d_p_off15_x(DisasContext *ctx, arg_ldb_d_p_off15_x *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off, MO_SB);
    return true;
}

static bool trans_stb_p_off15_d(DisasContext *ctx, arg_stb_p_off15_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), a->off, MO_UB);
    return true;
}

/* ---------------------------------------------------------------- */
/* Logic, no-op and population count                                */

static bool trans_not_d(DisasContext *ctx, arg_not_d *a)
{
    tcg_gen_not_i32(dreg(a->dst), dreg(a->src));
    gen_logic_flags(dreg(a->dst));
    return true;
}

/* The multiply unit's no-op, used to pad an otherwise empty MAC slot. */
static bool trans_mnop(DisasContext *ctx, arg_mnop *a)
{
    return true;
}

static bool trans_ones(DisasContext *ctx, arg_ones *a)
{
    TCGv t = tcg_temp_new_i32();

    /*
     * The population count is 0 to 32 and lands in the low half of the
     * destination, leaving the high half alone. Source and destination are
     * allowed to be the same register, so the count is formed before the
     * write. No status bits are affected.
     */
    tcg_gen_ctpop_i32(t, dreg(a->src));
    tcg_gen_deposit_i32(dreg(a->dst), dreg(a->dst), t, 0, 16);
    return true;
}

static bool trans_ld_a_zero(DisasContext *ctx, arg_ld_a_zero *a)
{
    /*
     * Both accumulators, all 40 bits of each including the sign-extending
     * A0.X and A1.X bytes. They are held in state rather than in a TCG
     * global, so the clear is a plain store.
     */
    tcg_gen_st_i64(tcg_constant_i64(0), tcg_env,
                   offsetof(CPUBfinState, a[0]));
    tcg_gen_st_i64(tcg_constant_i64(0), tcg_env,
                   offsetof(CPUBfinState, a[1]));
    return true;
}
