/* ---------------------------------------------------------------- */
/* Half word and byte access through a pointer                      */

static bool trans_ldw_d_p_z(DisasContext *ctx, arg_ldw_d_p_z *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), 0, MO_LEUW);
    return true;
}

static bool trans_ldw_d_p_x(DisasContext *ctx, arg_ldw_d_p_x *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), 0, MO_LESW);
    return true;
}

static bool trans_ldw_d_pp_z(DisasContext *ctx, arg_ldw_d_pp_z *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), 2, MO_LEUW);
    return true;
}

static bool trans_ldw_d_pp_x(DisasContext *ctx, arg_ldw_d_pp_x *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), 2, MO_LESW);
    return true;
}

static bool trans_ldw_d_pd_z(DisasContext *ctx, arg_ldw_d_pd_z *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), -2, MO_LEUW);
    return true;
}

static bool trans_ldw_d_pd_x(DisasContext *ctx, arg_ldw_d_pd_x *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), -2, MO_LESW);
    return true;
}

/* The half word stores take the low half of the source and never the high. */
static bool trans_stw_p_d(DisasContext *ctx, arg_stw_p_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), 0, MO_LEUW);
    return true;
}

static bool trans_stw_pp_d(DisasContext *ctx, arg_stw_pp_d *a)
{
    gen_store_pm(dreg(a->rs), preg(a->pb), 2, MO_LEUW);
    return true;
}

static bool trans_stw_pd_d(DisasContext *ctx, arg_stw_pd_d *a)
{
    gen_store_pm(dreg(a->rs), preg(a->pb), -2, MO_LEUW);
    return true;
}

static bool trans_ldb_d_p_z(DisasContext *ctx, arg_ldb_d_p_z *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), 0, MO_UB);
    return true;
}

static bool trans_ldb_d_p_x(DisasContext *ctx, arg_ldb_d_p_x *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), 0, MO_SB);
    return true;
}

static bool trans_ldb_d_pp_z(DisasContext *ctx, arg_ldb_d_pp_z *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), 1, MO_UB);
    return true;
}

static bool trans_ldb_d_pp_x(DisasContext *ctx, arg_ldb_d_pp_x *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), 1, MO_SB);
    return true;
}

static bool trans_ldb_d_pd_z(DisasContext *ctx, arg_ldb_d_pd_z *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), -1, MO_UB);
    return true;
}

static bool trans_ldb_d_pd_x(DisasContext *ctx, arg_ldb_d_pd_x *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), -1, MO_SB);
    return true;
}

static bool trans_stb_p_d(DisasContext *ctx, arg_stb_p_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), 0, MO_UB);
    return true;
}

static bool trans_stb_pp_d(DisasContext *ctx, arg_stb_pp_d *a)
{
    gen_store_pm(dreg(a->rs), preg(a->pb), 1, MO_UB);
    return true;
}

static bool trans_stb_pd_d(DisasContext *ctx, arg_stb_pd_d *a)
{
    gen_store_pm(dreg(a->rs), preg(a->pb), -1, MO_UB);
    return true;
}

/* ---------------------------------------------------------------- */
/* Post-modify index forms                                          */

/*
 * A half word load into one half of a data register has to leave the other
 * half alone, so it cannot land in the destination directly. Table C-10
 * sheets 6 and 7 give these two operation codes a plain [Preg] spelling as
 * the encoding whose two pointer fields name the same register.
 */
static void gen_index_load_half(int rd, int pb, int pi, bool hi)
{
    TCGv addr = tcg_temp_new_i32();
    TCGv t = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, preg(pb));
    if (pb != pi) {
        tcg_gen_add_i32(preg(pb), preg(pb), preg(pi));
    }
    tcg_gen_qemu_ld_i32(t, addr, 0, MO_LEUW);
    tcg_gen_deposit_i32(dreg(rd), dreg(rd), t, hi ? 16 : 0, 16);
}

static bool trans_ldw_pi_dl(DisasContext *ctx, arg_ldw_pi_dl *a)
{
    gen_index_load_half(a->rd, a->pb, a->pi, false);
    return true;
}

static bool trans_ldw_pi_dh(DisasContext *ctx, arg_ldw_pi_dh *a)
{
    gen_index_load_half(a->rd, a->pb, a->pi, true);
    return true;
}

static bool trans_ld_pi_d(DisasContext *ctx, arg_ld_pi_d *a)
{
    TCGv addr = tcg_temp_new_i32();

    /*
     * Table C-10 sheet 3 gives this operation code the whole opcode range and
     * documents no plain [Preg] spelling for it, so unlike the two half
     * register codes the pointer advances even when both fields name the same
     * register - which then doubles it.
     */
    tcg_gen_mov_i32(addr, preg(a->pb));
    tcg_gen_add_i32(preg(a->pb), preg(a->pb), preg(a->pi));
    tcg_gen_qemu_ld_i32(dreg(a->rd), addr, 0, MO_LEUL);
    return true;
}

/* ---------------------------------------------------------------- */
/* Access through an index register                                 */

static inline TCGv mreg(int n)
{
    return cpu_gpr[BFIN_REG_M0 + n];
}

static void gen_ld_half_i(int rd, int i, int delta, bool hi)
{
    TCGv t = tcg_temp_new_i32();

    gen_load_pm(t, ireg(i), delta, MO_LEUW);
    tcg_gen_deposit_i32(dreg(rd), dreg(rd), t, hi ? 16 : 0, 16);
}

static void gen_st_half_i(int rs, int i, int delta, bool hi)
{
    TCGv t = tcg_temp_new_i32();

    if (hi) {
        tcg_gen_shri_i32(t, dreg(rs), 16);
    } else {
        tcg_gen_mov_i32(t, dreg(rs));
    }
    gen_store_pm(t, ireg(i), delta, MO_LEUW);
}

static bool trans_ldw_dh_i(DisasContext *ctx, arg_ldw_dh_i *a)
{
    gen_ld_half_i(a->rd, a->ireg, 0, true);
    return true;
}

static bool trans_ldw_dh_ip(DisasContext *ctx, arg_ldw_dh_ip *a)
{
    gen_ld_half_i(a->rd, a->ireg, 2, true);
    return true;
}

static bool trans_ldw_dl_id(DisasContext *ctx, arg_ldw_dl_id *a)
{
    gen_ld_half_i(a->rd, a->ireg, -2, false);
    return true;
}

static bool trans_ldw_dh_id(DisasContext *ctx, arg_ldw_dh_id *a)
{
    gen_ld_half_i(a->rd, a->ireg, -2, true);
    return true;
}

static bool trans_ld_d_id(DisasContext *ctx, arg_ld_d_id *a)
{
    gen_load_pm(dreg(a->rd), ireg(a->ireg), -4, MO_LEUL);
    return true;
}

static bool trans_stw_i_dl(DisasContext *ctx, arg_stw_i_dl *a)
{
    gen_st_half_i(a->rs, a->ireg, 0, false);
    return true;
}

static bool trans_stw_i_dh(DisasContext *ctx, arg_stw_i_dh *a)
{
    gen_st_half_i(a->rs, a->ireg, 0, true);
    return true;
}

static bool trans_stw_ip_dl(DisasContext *ctx, arg_stw_ip_dl *a)
{
    gen_st_half_i(a->rs, a->ireg, 2, false);
    return true;
}

static bool trans_stw_ip_dh(DisasContext *ctx, arg_stw_ip_dh *a)
{
    gen_st_half_i(a->rs, a->ireg, 2, true);
    return true;
}

static bool trans_stw_id_dl(DisasContext *ctx, arg_stw_id_dl *a)
{
    gen_st_half_i(a->rs, a->ireg, -2, false);
    return true;
}

static bool trans_stw_id_dh(DisasContext *ctx, arg_stw_id_dh *a)
{
    gen_st_half_i(a->rs, a->ireg, -2, true);
    return true;
}

static bool trans_st_id_d(DisasContext *ctx, arg_st_id_d *a)
{
    gen_store_pm(dreg(a->rs), ireg(a->ireg), -4, MO_LEUL);
    return true;
}

/*
 * The two ++Mreg forms take the post-modify amount from an M register instead
 * of the access width. The index and data register files are disjoint, so the
 * update cannot disturb the value being transferred.
 */
static bool trans_ld_d_im(DisasContext *ctx, arg_ld_d_im *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, ireg(a->ireg));
    tcg_gen_add_i32(ireg(a->ireg), ireg(a->ireg), mreg(a->mreg));
    tcg_gen_qemu_ld_i32(dreg(a->rd), addr, 0, MO_LEUL);
    return true;
}

static bool trans_st_im_d(DisasContext *ctx, arg_st_im_d *a)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, ireg(a->ireg));
    tcg_gen_add_i32(ireg(a->ireg), ireg(a->ireg), mreg(a->mreg));
    tcg_gen_qemu_st_i32(dreg(a->rs), addr, 0, MO_LEUL);
    return true;
}

/* ---------------------------------------------------------------- */
/* Frame pointer relative                                           */

static bool trans_ld_d_fp_off(DisasContext *ctx, arg_ld_d_fp_off *a)
{
    gen_load(ctx, dreg(a->rd), cpu_gpr[BFIN_REG_FP], -(a->off * 4), MO_LEUL);
    return true;
}

static bool trans_ld_p_fp_off(DisasContext *ctx, arg_ld_p_fp_off *a)
{
    gen_load(ctx, preg(a->rd), cpu_gpr[BFIN_REG_FP], -(a->off * 4), MO_LEUL);
    return true;
}

static bool trans_st_fp_off_d(DisasContext *ctx, arg_st_fp_off_d *a)
{
    gen_store(ctx, dreg(a->rs), cpu_gpr[BFIN_REG_FP], -(a->off * 4), MO_LEUL);
    return true;
}

static bool trans_st_fp_off_p(DisasContext *ctx, arg_st_fp_off_p *a)
{
    gen_store(ctx, preg(a->rs), cpu_gpr[BFIN_REG_FP], -(a->off * 4), MO_LEUL);
    return true;
}

/* ---------------------------------------------------------------- */
/* Push and pop of a register range                                 */

/*
 * Push Multiple saves the lowest numbered register first and the data
 * registers before the pointers; Pop Multiple is its exact mirror. The stack
 * pointer is only committed once the last access has been issued, so a fault
 * part way through leaves SP where it started and the instruction can be
 * restarted, as the manual requires on page 10-16.
 *
 * Preglim runs to P5 and no further, and neither SP nor FP can take part.
 * The decoder cannot express that bound, so the two encodings above P5 are
 * rejected here.
 */
static bool gen_push_multiple(int dr, bool d, int pr, bool p)
{
    TCGv addr = tcg_temp_new_i32();
    int i;

    if (p && pr > 5) {
        return false;
    }
    tcg_gen_mov_i32(addr, cpu_gpr[BFIN_REG_SP]);
    if (d) {
        for (i = dr; i < 8; i++) {
            tcg_gen_subi_i32(addr, addr, 4);
            tcg_gen_qemu_st_i32(dreg(i), addr, 0, MO_LEUL);
        }
    }
    if (p) {
        for (i = pr; i < 6; i++) {
            tcg_gen_subi_i32(addr, addr, 4);
            tcg_gen_qemu_st_i32(preg(i), addr, 0, MO_LEUL);
        }
    }
    tcg_gen_mov_i32(cpu_gpr[BFIN_REG_SP], addr);
    return true;
}

static bool gen_pop_multiple(int dr, bool d, int pr, bool p)
{
    TCGv addr = tcg_temp_new_i32();
    int i;

    if (p && pr > 5) {
        return false;
    }
    tcg_gen_mov_i32(addr, cpu_gpr[BFIN_REG_SP]);
    if (p) {
        for (i = 5; i >= pr; i--) {
            tcg_gen_qemu_ld_i32(preg(i), addr, 0, MO_LEUL);
            tcg_gen_addi_i32(addr, addr, 4);
        }
    }
    if (d) {
        for (i = 7; i >= dr; i--) {
            tcg_gen_qemu_ld_i32(dreg(i), addr, 0, MO_LEUL);
            tcg_gen_addi_i32(addr, addr, 4);
        }
    }
    tcg_gen_mov_i32(cpu_gpr[BFIN_REG_SP], addr);
    return true;
}

static bool trans_push_mult(DisasContext *ctx, arg_push_mult *a)
{
    return gen_push_multiple(a->dr, true, a->pr, true);
}

static bool trans_push_mult_d(DisasContext *ctx, arg_push_mult_d *a)
{
    return gen_push_multiple(a->dr, true, 0, false);
}

static bool trans_push_mult_p(DisasContext *ctx, arg_push_mult_p *a)
{
    return gen_push_multiple(0, false, a->pr, true);
}

static bool trans_pop_mult(DisasContext *ctx, arg_pop_mult *a)
{
    return gen_pop_multiple(a->dr, true, a->pr, true);
}

static bool trans_pop_mult_d(DisasContext *ctx, arg_pop_mult_d *a)
{
    return gen_pop_multiple(a->dr, true, 0, false);
}

static bool trans_pop_mult_p(DisasContext *ctx, arg_pop_mult_p *a)
{
    return gen_pop_multiple(0, false, a->pr, true);
}

/* ---------------------------------------------------------------- */
/* Cache control                                                    */

static bool gen_cache_ctrl(int pb, bool post_inc)
{
    /*
     * Nothing here models a cache, so all that is left of PREFETCH, FLUSH,
     * FLUSHINV and IFLUSH is the optional pointer update. It advances by the
     * cache line size, which is 32 bytes on every Blackfin part; see the
     * instruction overview at the head of the cache control chapter.
     */
    if (post_inc) {
        tcg_gen_addi_i32(preg(pb), preg(pb), 32);
    }
    return true;
}

static bool trans_prefetch_p(DisasContext *ctx, arg_prefetch_p *a)
{
    return gen_cache_ctrl(a->pb, false);
}

static bool trans_prefetch_pp(DisasContext *ctx, arg_prefetch_pp *a)
{
    return gen_cache_ctrl(a->pb, true);
}

static bool trans_flush_p(DisasContext *ctx, arg_flush_p *a)
{
    return gen_cache_ctrl(a->pb, false);
}

static bool trans_flush_pp(DisasContext *ctx, arg_flush_pp *a)
{
    return gen_cache_ctrl(a->pb, true);
}

static bool trans_flushinv_p(DisasContext *ctx, arg_flushinv_p *a)
{
    return gen_cache_ctrl(a->pb, false);
}

static bool trans_flushinv_pp(DisasContext *ctx, arg_flushinv_pp *a)
{
    return gen_cache_ctrl(a->pb, true);
}

static bool trans_iflush_p(DisasContext *ctx, arg_iflush_p *a)
{
    return gen_cache_ctrl(a->pb, false);
}

static bool trans_iflush_pp(DisasContext *ctx, arg_iflush_pp *a)
{
    return gen_cache_ctrl(a->pb, true);
}
