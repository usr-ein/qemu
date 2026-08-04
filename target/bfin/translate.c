/*
 * Analog Devices Blackfin translation
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Semantics are from the Blackfin Processor Programming Reference rev 2.2.
 * Instruction encodings live in insn16.decode and insn32.decode.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/memop.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translation-block.h"
#include "exec/translator.h"
#include "exec/target_page.h"
#include "accel/tcg/cpu-ldst.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

/* Leave the block and re-enter, for state the translator cannot track. */
#define DISAS_UPDATE  DISAS_TARGET_0

typedef struct DisasContext {
    DisasContextBase base;
    uint32_t pc;            /* address of the instruction being translated */
    uint32_t pc_next;       /* address of the one after it                 */
    uint32_t tb_flags;
} DisasContext;

static TCGv cpu_gpr[BFIN_NUM_GPR];
static TCGv cpu_pc, cpu_astat, cpu_cc;
static TCGv cpu_lc[2], cpu_lt[2], cpu_lb[2];
static TCGv cpu_rets, cpu_reti, cpu_retx, cpu_retn, cpu_rete;

void bfin_translate_init(void)
{
    static const char *const gpr_names[BFIN_NUM_GPR] = {
        "R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
        "P0", "P1", "P2", "P3", "P4", "P5", "SP", "FP",
        "I0", "I1", "I2", "I3", "M0", "M1", "M2", "M3",
        "B0", "B1", "B2", "B3", "L0", "L1", "L2", "L3",
    };
    int i;

    for (i = 0; i < BFIN_NUM_GPR; i++) {
        cpu_gpr[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPUBfinState, gpr[i]),
                                            gpr_names[i]);
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env,
                                    offsetof(CPUBfinState, pc), "PC");
    cpu_astat = tcg_global_mem_new_i32(tcg_env,
                                       offsetof(CPUBfinState, astat), "ASTAT");
    cpu_cc = tcg_global_mem_new_i32(tcg_env,
                                    offsetof(CPUBfinState, cc), "CC");
    for (i = 0; i < 2; i++) {
        static const char *const lc_n[] = { "LC0", "LC1" };
        static const char *const lt_n[] = { "LT0", "LT1" };
        static const char *const lb_n[] = { "LB0", "LB1" };

        cpu_lc[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUBfinState, lc[i]),
                                           lc_n[i]);
        cpu_lt[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUBfinState, lt[i]),
                                           lt_n[i]);
        cpu_lb[i] = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUBfinState, lb[i]),
                                           lb_n[i]);
    }
    cpu_rets = tcg_global_mem_new_i32(tcg_env,
                                      offsetof(CPUBfinState, rets), "RETS");
    cpu_reti = tcg_global_mem_new_i32(tcg_env,
                                      offsetof(CPUBfinState, reti), "RETI");
    cpu_retx = tcg_global_mem_new_i32(tcg_env,
                                      offsetof(CPUBfinState, retx), "RETX");
    cpu_retn = tcg_global_mem_new_i32(tcg_env,
                                      offsetof(CPUBfinState, retn), "RETN");
    cpu_rete = tcg_global_mem_new_i32(tcg_env,
                                      offsetof(CPUBfinState, rete), "RETE");
}

/* Register file helpers. Data registers are 0-7 and pointers 8-15. */
static inline TCGv dreg(int n)
{
    return cpu_gpr[BFIN_REG_R0 + n];
}

static inline TCGv preg(int n)
{
    return cpu_gpr[BFIN_REG_P0 + n];
}

static void gen_goto_tb(DisasContext *ctx, int n, uint32_t dest)
{
    if (translator_use_goto_tb(&ctx->base, dest)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_exit_tb(ctx->base.tb, n);
    } else {
        tcg_gen_movi_i32(cpu_pc, dest);
        tcg_gen_lookup_and_goto_ptr();
    }
    ctx->base.is_jmp = DISAS_NORETURN;
}

static void gen_jump_reg(DisasContext *ctx, TCGv dest)
{
    tcg_gen_mov_i32(cpu_pc, dest);
    tcg_gen_lookup_and_goto_ptr();
    ctx->base.is_jmp = DISAS_NORETURN;
}

static void gen_exception(DisasContext *ctx, int excp, uint32_t excause)
{
    tcg_gen_movi_i32(cpu_pc, ctx->pc);
    gen_helper_raise_exception(tcg_env, tcg_constant_i32(excp),
                               tcg_constant_i32(excause));
    ctx->base.is_jmp = DISAS_NORETURN;
}

static void gen_undef(DisasContext *ctx, uint32_t opc, int len)
{
    qemu_log_mask(LOG_UNIMP,
                  "bfin: unimplemented %d-bit instruction 0x%0*x at 0x%08x\n",
                  len * 8, len == 2 ? 4 : 8, opc, ctx->pc);
    gen_exception(ctx, BFIN_EXCP_EVX, BFIN_EXCAUSE_UNDEF_INSN);
}

/*
 * ASTAT.AZ and ASTAT.AN follow the result of most ALU and shifter operations.
 * Carry and overflow are per-instruction and are set by the individual
 * translators that need them.
 */
static void gen_logic_flags(TCGv res)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_setcondi_i32(TCG_COND_EQ, t, res, 0);
    tcg_gen_deposit_i32(cpu_astat, cpu_astat, t, R_ASTAT_AZ_SHIFT, 1);
    tcg_gen_shri_i32(t, res, 31);
    tcg_gen_deposit_i32(cpu_astat, cpu_astat, t, R_ASTAT_AN_SHIFT, 1);
}

#include "decode-insn16.c.inc"
#include "decode-insn32.c.inc"

/* ---------------------------------------------------------------- */
/* Program flow control                                             */

static bool trans_nop(DisasContext *ctx, arg_nop *a)
{
    return true;
}

static bool trans_idle(DisasContext *ctx, arg_idle *a)
{
    tcg_gen_movi_i32(cpu_pc, ctx->pc_next);
    gen_helper_idle(tcg_env);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

static bool trans_csync(DisasContext *ctx, arg_csync *a)
{
    tcg_gen_mb(TCG_MO_ALL | TCG_BAR_SC);
    return true;
}

static bool trans_ssync(DisasContext *ctx, arg_ssync *a)
{
    tcg_gen_mb(TCG_MO_ALL | TCG_BAR_SC);
    return true;
}

static bool trans_emuexcpt(DisasContext *ctx, arg_emuexcpt *a)
{
    gen_exception(ctx, BFIN_EXCP_EMU, 0);
    return true;
}

static bool gen_return(DisasContext *ctx, TCGv src)
{
    gen_jump_reg(ctx, src);
    return true;
}

static bool trans_rts(DisasContext *ctx, arg_rts *a)
{
    return gen_return(ctx, cpu_rets);
}

static bool trans_rti(DisasContext *ctx, arg_rti *a)
{
    gen_helper_rti(tcg_env);
    return gen_return(ctx, cpu_reti);
}

static bool trans_rtx(DisasContext *ctx, arg_rtx *a)
{
    return gen_return(ctx, cpu_retx);
}

static bool trans_rtn(DisasContext *ctx, arg_rtn *a)
{
    return gen_return(ctx, cpu_retn);
}

static bool trans_rte(DisasContext *ctx, arg_rte *a)
{
    return gen_return(ctx, cpu_rete);
}

static bool trans_cli(DisasContext *ctx, arg_cli *a)
{
    gen_helper_cli(dreg(a->rd), tcg_env);
    return true;
}

static bool trans_sti(DisasContext *ctx, arg_sti *a)
{
    gen_helper_sti(tcg_env, dreg(a->rd));
    ctx->base.is_jmp = DISAS_UPDATE;
    return true;
}

static bool trans_jump_p(DisasContext *ctx, arg_jump_p *a)
{
    gen_jump_reg(ctx, preg(a->pb));
    return true;
}

static bool trans_call_p(DisasContext *ctx, arg_call_p *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_mov_i32(t, preg(a->pb));
    tcg_gen_movi_i32(cpu_rets, ctx->pc_next);
    gen_jump_reg(ctx, t);
    return true;
}

static bool trans_call_pc_p(DisasContext *ctx, arg_call_pc_p *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_addi_i32(t, preg(a->pb), ctx->pc);
    tcg_gen_movi_i32(cpu_rets, ctx->pc_next);
    gen_jump_reg(ctx, t);
    return true;
}

static bool trans_jump_pc_p(DisasContext *ctx, arg_jump_pc_p *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_addi_i32(t, preg(a->pb), ctx->pc);
    gen_jump_reg(ctx, t);
    return true;
}

static bool trans_raise(DisasContext *ctx, arg_raise *a)
{
    gen_helper_raise_ivg(tcg_env, tcg_constant_i32(a->imm));
    ctx->base.is_jmp = DISAS_UPDATE;
    return true;
}

static bool trans_excpt(DisasContext *ctx, arg_excpt *a)
{
    gen_exception(ctx, BFIN_EXCP_EVX, a->imm);
    return true;
}

static bool trans_jump_s(DisasContext *ctx, arg_jump_s *a)
{
    gen_goto_tb(ctx, 0, ctx->pc + a->off * 2);
    return true;
}

static bool trans_jump_l(DisasContext *ctx, arg_jump_l *a)
{
    int32_t off = (int32_t)((a->hi << 16) | a->lo) << 8 >> 8;

    gen_goto_tb(ctx, 0, ctx->pc + off * 2);
    return true;
}

static bool trans_call_l(DisasContext *ctx, arg_call_l *a)
{
    int32_t off = (int32_t)((a->hi << 16) | a->lo) << 8 >> 8;

    tcg_gen_movi_i32(cpu_rets, ctx->pc_next);
    gen_goto_tb(ctx, 0, ctx->pc + off * 2);
    return true;
}

static bool gen_cond_jump(DisasContext *ctx, bool want, int off)
{
    TCGLabel *taken = gen_new_label();

    tcg_gen_brcondi_i32(want ? TCG_COND_NE : TCG_COND_EQ, cpu_cc, 0, taken);
    gen_goto_tb(ctx, 0, ctx->pc_next);
    gen_set_label(taken);
    gen_goto_tb(ctx, 1, ctx->pc + off * 2);
    ctx->base.is_jmp = DISAS_NORETURN;
    return true;
}

static bool trans_if_cc_jump(DisasContext *ctx, arg_if_cc_jump *a)
{
    return gen_cond_jump(ctx, true, a->off);
}

static bool trans_if_cc_jump_bp(DisasContext *ctx, arg_if_cc_jump_bp *a)
{
    return gen_cond_jump(ctx, true, a->off);
}

static bool trans_if_ncc_jump(DisasContext *ctx, arg_if_ncc_jump *a)
{
    return gen_cond_jump(ctx, false, a->off);
}

static bool trans_if_ncc_jump_bp(DisasContext *ctx, arg_if_ncc_jump_bp *a)
{
    return gen_cond_jump(ctx, false, a->off);
}

/* ---------------------------------------------------------------- */
/* Zero overhead loops                                              */

/* ---------------------------------------------------------------- */
/* Move                                                             */

static bool trans_movereg(DisasContext *ctx, arg_movereg *a)
{
    int dst = (a->dg << 3) | a->dn;
    int src = (a->sg << 3) | a->sn;

    /*
     * Groups 0 to 3 are the general and DAG registers and map straight onto
     * gpr[]; anything above that is a system register and goes through a
     * helper, which also has to cope with the SP/USP banking and with writes
     * to ASTAT unpacking the out-of-line CC bit.
     */
    if (a->dg < 4 && a->sg < 4) {
        tcg_gen_mov_i32(cpu_gpr[dst], cpu_gpr[src]);
        return true;
    }
    if (a->sg < 4) {
        gen_helper_write_creg(tcg_env, tcg_constant_i32(dst), cpu_gpr[src]);
        return true;
    }
    if (a->dg < 4) {
        gen_helper_read_creg(cpu_gpr[dst], tcg_env, tcg_constant_i32(src));
        return true;
    }
    {
        TCGv t = tcg_temp_new_i32();

        gen_helper_read_creg(t, tcg_env, tcg_constant_i32(src));
        gen_helper_write_creg(tcg_env, tcg_constant_i32(dst), t);
    }
    return true;
}

/* ---------------------------------------------------------------- */
/* Control code bit                                                 */

static bool gen_cc_cmp(TCGCond cond, TCGv a, TCGv b)
{
    tcg_gen_setcond_i32(cond, cpu_cc, a, b);
    return true;
}

static bool trans_cc_eq_dreg(DisasContext *ctx, arg_cc_eq_dreg *a)
{
    return gen_cc_cmp(TCG_COND_EQ, dreg(a->dst), dreg(a->src));
}

static bool trans_cc_lt_dreg(DisasContext *ctx, arg_cc_lt_dreg *a)
{
    return gen_cc_cmp(TCG_COND_LT, dreg(a->dst), dreg(a->src));
}

static bool trans_cc_le_dreg(DisasContext *ctx, arg_cc_le_dreg *a)
{
    return gen_cc_cmp(TCG_COND_LE, dreg(a->dst), dreg(a->src));
}

static bool trans_cc_eq_imm(DisasContext *ctx, arg_cc_eq_imm *a)
{
    tcg_gen_setcondi_i32(TCG_COND_EQ, cpu_cc, dreg(a->dst), a->imm);
    return true;
}

static bool trans_cc_lt_imm(DisasContext *ctx, arg_cc_lt_imm *a)
{
    tcg_gen_setcondi_i32(TCG_COND_LT, cpu_cc, dreg(a->dst), a->imm);
    return true;
}

static bool trans_cc_le_imm(DisasContext *ctx, arg_cc_le_imm *a)
{
    tcg_gen_setcondi_i32(TCG_COND_LE, cpu_cc, dreg(a->dst), a->imm);
    return true;
}

static bool trans_mov_cc_dreg(DisasContext *ctx, arg_mov_cc_dreg *a)
{
    tcg_gen_mov_i32(dreg(a->rd), cpu_cc);
    return true;
}

static bool trans_mov_dreg_cc(DisasContext *ctx, arg_mov_dreg_cc *a)
{
    tcg_gen_setcondi_i32(TCG_COND_NE, cpu_cc, dreg(a->rd), 0);
    return true;
}

static bool trans_cc_not(DisasContext *ctx, arg_cc_not *a)
{
    tcg_gen_xori_i32(cpu_cc, cpu_cc, 1);
    return true;
}

/* ---------------------------------------------------------------- */
/* Load immediate                                                   */

static bool trans_ldimm_d7(DisasContext *ctx, arg_ldimm_d7 *a)
{
    tcg_gen_movi_i32(dreg(a->rd), a->imm);
    return true;
}

static bool trans_ldimm_p7(DisasContext *ctx, arg_ldimm_p7 *a)
{
    tcg_gen_movi_i32(preg(a->rd), a->imm);
    return true;
}

static bool trans_addimm_p7(DisasContext *ctx, arg_addimm_p7 *a)
{
    /* Pointer arithmetic does not touch the arithmetic status bits. */
    tcg_gen_addi_i32(preg(a->rd), preg(a->rd), a->imm);
    return true;
}

static bool trans_ldimm_lo(DisasContext *ctx, arg_ldimm_lo *a)
{
    tcg_gen_deposit_i32(cpu_gpr[a->reg], cpu_gpr[a->reg],
                        tcg_constant_i32(a->imm), 0, 16);
    return true;
}

static bool trans_ldimm_hi(DisasContext *ctx, arg_ldimm_hi *a)
{
    tcg_gen_deposit_i32(cpu_gpr[a->reg], cpu_gpr[a->reg],
                        tcg_constant_i32(a->imm), 16, 16);
    return true;
}

static bool trans_ldimm_x(DisasContext *ctx, arg_ldimm_x *a)
{
    tcg_gen_movi_i32(cpu_gpr[a->reg], (int32_t)(int16_t)a->imm);
    return true;
}

static bool trans_ldimm_z(DisasContext *ctx, arg_ldimm_z *a)
{
    tcg_gen_movi_i32(cpu_gpr[a->reg], a->imm);
    return true;
}

/* ---------------------------------------------------------------- */
/* Load and store                                                   */

static void gen_load(DisasContext *ctx, TCGv dst, TCGv base, int32_t off,
                     MemOp op)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, base, off);
    tcg_gen_qemu_ld_i32(dst, addr, 0, op);
}

static void gen_store(DisasContext *ctx, TCGv src, TCGv base, int32_t off,
                      MemOp op)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_addi_i32(addr, base, off);
    tcg_gen_qemu_st_i32(src, addr, 0, op);
}

static bool trans_ld_d_p(DisasContext *ctx, arg_ld_d_p *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), 0, MO_LEUL);
    return true;
}

static bool trans_ld_p_p(DisasContext *ctx, arg_ld_p_p *a)
{
    gen_load(ctx, preg(a->rd), preg(a->pb), 0, MO_LEUL);
    return true;
}

static bool trans_st_p_d(DisasContext *ctx, arg_st_p_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), 0, MO_LEUL);
    return true;
}

static bool trans_st_p_p(DisasContext *ctx, arg_st_p_p *a)
{
    gen_store(ctx, preg(a->rs), preg(a->pb), 0, MO_LEUL);
    return true;
}

static bool trans_ld_d_p_off(DisasContext *ctx, arg_ld_d_p_off *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_ld_p_p_off(DisasContext *ctx, arg_ld_p_p_off *a)
{
    gen_load(ctx, preg(a->rd), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_st_p_off_d(DisasContext *ctx, arg_st_p_off_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_st_p_off_p(DisasContext *ctx, arg_st_p_off_p *a)
{
    gen_store(ctx, preg(a->rs), preg(a->pb), a->off * 4, MO_LEUL);
    return true;
}

static bool trans_ldw_d_p_off_z(DisasContext *ctx, arg_ldw_d_p_off_z *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off * 2, MO_LEUW);
    return true;
}

static bool trans_ldw_d_p_off_x(DisasContext *ctx, arg_ldw_d_p_off_x *a)
{
    gen_load(ctx, dreg(a->rd), preg(a->pb), a->off * 2, MO_LESW);
    return true;
}

static bool trans_stw_p_off_d(DisasContext *ctx, arg_stw_p_off_d *a)
{
    gen_store(ctx, dreg(a->rs), preg(a->pb), a->off * 2, MO_LEUW);
    return true;
}

/*
 * Post-modify forms. The address used is the pointer's current value and the
 * pointer is updated afterwards, so the update must not be visible to the
 * access - which matters when the loaded register is the pointer itself.
 */
static void gen_load_pm(TCGv dst, TCGv ptr, int delta, MemOp op)
{
    TCGv addr = tcg_temp_new_i32();
    TCGv val = tcg_temp_new_i32();

    /* The pointer update follows the access; see gen_store_pm. */
    tcg_gen_mov_i32(addr, ptr);
    tcg_gen_qemu_ld_i32(val, addr, 0, op);
    tcg_gen_addi_i32(ptr, ptr, delta);
    tcg_gen_mov_i32(dst, val);
}

static void gen_store_pm(TCGv src, TCGv ptr, int delta, MemOp op)
{
    TCGv addr = tcg_temp_new_i32();

    /*
     * The pointer update has to follow the access, not precede it. An access
     * to a device rather than to RAM makes QEMU rewind the block and execute
     * it again, and the pointer is a global, so an update made before the
     * access is applied twice: the pointer advances by two elements per pass
     * and every other store goes missing.
     */
    tcg_gen_mov_i32(addr, ptr);
    tcg_gen_qemu_st_i32(src, addr, 0, op);
    tcg_gen_addi_i32(ptr, ptr, delta);
}

static bool trans_ld_d_pp(DisasContext *ctx, arg_ld_d_pp *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), 4, MO_LEUL);
    return true;
}

static bool trans_ld_p_pp(DisasContext *ctx, arg_ld_p_pp *a)
{
    gen_load_pm(preg(a->rd), preg(a->pb), 4, MO_LEUL);
    return true;
}

static bool trans_ld_d_pd(DisasContext *ctx, arg_ld_d_pd *a)
{
    gen_load_pm(dreg(a->rd), preg(a->pb), -4, MO_LEUL);
    return true;
}

static bool trans_ld_p_pd(DisasContext *ctx, arg_ld_p_pd *a)
{
    gen_load_pm(preg(a->rd), preg(a->pb), -4, MO_LEUL);
    return true;
}

static bool trans_st_pp_d(DisasContext *ctx, arg_st_pp_d *a)
{
    gen_store_pm(dreg(a->rs), preg(a->pb), 4, MO_LEUL);
    return true;
}

static bool trans_st_pp_p(DisasContext *ctx, arg_st_pp_p *a)
{
    gen_store_pm(preg(a->rs), preg(a->pb), 4, MO_LEUL);
    return true;
}

static bool trans_st_pd_d(DisasContext *ctx, arg_st_pd_d *a)
{
    gen_store_pm(dreg(a->rs), preg(a->pb), -4, MO_LEUL);
    return true;
}

static bool trans_st_pd_p(DisasContext *ctx, arg_st_pd_p *a)
{
    gen_store_pm(preg(a->rs), preg(a->pb), -4, MO_LEUL);
    return true;
}

/*
 * Indexed forms. Table C-10 notes that when both pointer fields name the same
 * register the encoding means the plain [Preg] access; only when they differ
 * does the pointer advance by the index register afterwards.
 */
static void gen_indexed_load(int rd_gpr, int pb, int pi, MemOp op, bool half_hi)
{
    TCGv addr = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, preg(pb));
    tcg_gen_qemu_ld_i32(cpu_gpr[rd_gpr], addr, 0, op);
    if (pb != pi) {
        tcg_gen_add_i32(preg(pb), preg(pb), preg(pi));
    }
}

static void gen_indexed_store(TCGv src, int pb, int pi, MemOp op)
{
    TCGv addr = tcg_temp_new_i32();
    TCGv val = tcg_temp_new_i32();

    tcg_gen_mov_i32(addr, preg(pb));
    tcg_gen_mov_i32(val, src);
    tcg_gen_qemu_st_i32(val, addr, 0, op);
    if (pb != pi) {
        tcg_gen_add_i32(preg(pb), preg(pb), preg(pi));
    }
}

static bool trans_ldw_pi_d_z(DisasContext *ctx, arg_ldw_pi_d_z *a)
{
    gen_indexed_load(BFIN_REG_R0 + a->rd, a->pb, a->pi, MO_LEUW, false);
    return true;
}

static bool trans_ldw_pi_d_x(DisasContext *ctx, arg_ldw_pi_d_x *a)
{
    gen_indexed_load(BFIN_REG_R0 + a->rd, a->pb, a->pi, MO_LESW, false);
    return true;
}

static bool trans_st_pi_d(DisasContext *ctx, arg_st_pi_d *a)
{
    gen_indexed_store(dreg(a->rs), a->pb, a->pi, MO_LEUL);
    return true;
}

static bool trans_stw_pi_dl(DisasContext *ctx, arg_stw_pi_dl *a)
{
    gen_indexed_store(dreg(a->rs), a->pb, a->pi, MO_LEUW);
    return true;
}

static bool trans_stw_pi_dh(DisasContext *ctx, arg_stw_pi_dh *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_shri_i32(t, dreg(a->rs), 16);
    gen_indexed_store(t, a->pb, a->pi, MO_LEUW);
    return true;
}

/*
 * Index register access. Circular addressing is not modelled yet: it applies
 * only when the matching L register is nonzero, and the firmware's reset stub
 * clears L0 to L3 before doing anything else, so straight-line pointer
 * arithmetic is correct for the boot path.
 */
static inline TCGv ireg(int n)
{
    return cpu_gpr[BFIN_REG_I0 + n];
}

static bool trans_ld_d_i(DisasContext *ctx, arg_ld_d_i *a)
{
    tcg_gen_qemu_ld_i32(dreg(a->rd), ireg(a->ireg), 0, MO_LEUL);
    return true;
}

static bool trans_ldw_dl_i(DisasContext *ctx, arg_ldw_dl_i *a)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_qemu_ld_i32(t, ireg(a->ireg), 0, MO_LEUW);
    tcg_gen_deposit_i32(dreg(a->rd), dreg(a->rd), t, 0, 16);
    return true;
}

static bool trans_ld_d_ip(DisasContext *ctx, arg_ld_d_ip *a)
{
    gen_load_pm(dreg(a->rd), ireg(a->ireg), 4, MO_LEUL);
    return true;
}

static bool trans_ldw_dl_ip(DisasContext *ctx, arg_ldw_dl_ip *a)
{
    TCGv t = tcg_temp_new_i32();

    gen_load_pm(t, ireg(a->ireg), 2, MO_LEUW);
    tcg_gen_deposit_i32(dreg(a->rd), dreg(a->rd), t, 0, 16);
    return true;
}

static bool trans_st_i_d(DisasContext *ctx, arg_st_i_d *a)
{
    tcg_gen_qemu_st_i32(dreg(a->rs), ireg(a->ireg), 0, MO_LEUL);
    return true;
}

static bool trans_st_ip_d(DisasContext *ctx, arg_st_ip_d *a)
{
    gen_store_pm(dreg(a->rs), ireg(a->ireg), 4, MO_LEUL);
    return true;
}

static bool trans_pop_reg(DisasContext *ctx, arg_pop_reg *a)
{
    TCGv sp = cpu_gpr[BFIN_REG_SP];

    if (a->reg < BFIN_NUM_GPR) {
        tcg_gen_qemu_ld_i32(cpu_gpr[a->reg], sp, 0, MO_LEUL);
    } else {
        TCGv t = tcg_temp_new_i32();

        tcg_gen_qemu_ld_i32(t, sp, 0, MO_LEUL);
        gen_helper_write_creg(tcg_env, tcg_constant_i32(a->reg), t);
    }
    tcg_gen_addi_i32(sp, sp, 4);
    return true;
}

static bool trans_push_reg(DisasContext *ctx, arg_push_reg *a)
{
    TCGv sp = cpu_gpr[BFIN_REG_SP];
    TCGv val = tcg_temp_new_i32();

    if (a->reg < BFIN_NUM_GPR) {
        tcg_gen_mov_i32(val, cpu_gpr[a->reg]);
    } else {
        gen_helper_read_creg(val, tcg_env, tcg_constant_i32(a->reg));
    }
    tcg_gen_subi_i32(sp, sp, 4);
    tcg_gen_qemu_st_i32(val, sp, 0, MO_LEUL);
    return true;
}

/* ---------------------------------------------------------------- */
/* Stack frames                                                     */

static bool trans_link(DisasContext *ctx, arg_link *a)
{
    TCGv sp = cpu_gpr[BFIN_REG_SP];
    TCGv fp = cpu_gpr[BFIN_REG_FP];

    /* [--SP] = RETS ; [--SP] = FP ; FP = SP ; SP -= framesize */
    tcg_gen_subi_i32(sp, sp, 4);
    tcg_gen_qemu_st_i32(cpu_rets, sp, 0, MO_LEUL);
    tcg_gen_subi_i32(sp, sp, 4);
    tcg_gen_qemu_st_i32(fp, sp, 0, MO_LEUL);
    tcg_gen_mov_i32(fp, sp);
    if (a->imm) {
        tcg_gen_subi_i32(sp, sp, a->imm * 4);
    }
    return true;
}

static bool trans_unlink(DisasContext *ctx, arg_unlink *a)
{
    TCGv sp = cpu_gpr[BFIN_REG_SP];
    TCGv fp = cpu_gpr[BFIN_REG_FP];

    /* SP = FP ; FP = [SP++] ; RETS = [SP++] */
    tcg_gen_mov_i32(sp, fp);
    tcg_gen_qemu_ld_i32(fp, sp, 0, MO_LEUL);
    tcg_gen_addi_i32(sp, sp, 4);
    tcg_gen_qemu_ld_i32(cpu_rets, sp, 0, MO_LEUL);
    tcg_gen_addi_i32(sp, sp, 4);
    return true;
}

/* ---------------------------------------------------------------- */
/* Arithmetic and logic                                             */

static bool gen_bittst(DisasContext *ctx, int rd, int imm, bool want)
{
    TCGv t = tcg_temp_new_i32();

    tcg_gen_andi_i32(t, dreg(rd), 1u << imm);
    tcg_gen_setcondi_i32(want ? TCG_COND_NE : TCG_COND_EQ, cpu_cc, t, 0);
    return true;
}

static bool trans_bittst(DisasContext *ctx, arg_bittst *a)
{
    return gen_bittst(ctx, a->rd, a->imm, true);
}

static bool trans_bittst_n(DisasContext *ctx, arg_bittst_n *a)
{
    return gen_bittst(ctx, a->rd, a->imm, false);
}

/* ---------------------------------------------------------------- */

/*
 * Hardware loop closure. If this instruction sits at the bottom of an active
 * loop, decrement the counter and go back to the top instead of falling
 * through. Whether a loop is live is part of the translation key, so this is
 * only emitted for blocks that were translated with one.
 */
static void gen_loop_end(DisasContext *ctx)
{
    int n;

    for (n = 0; n < 2; n++) {
        TCGLabel *skip;

        if (!(ctx->tb_flags & (1 << n))) {
            continue;
        }
        skip = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_NE, cpu_lb[n], ctx->pc, skip);
        tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_lc[n], 0, skip);
        /*
         * The counter is decremented at the bottom of the body and the branch
         * is taken only if it is still nonzero, so a loop set up with LC = N
         * executes the body exactly N times. Testing for zero before the
         * decrement instead runs it N + 1 times.
         */
        tcg_gen_subi_i32(cpu_lc[n], cpu_lc[n], 1);
        tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_lc[n], 0, skip);
        tcg_gen_mov_i32(cpu_pc, cpu_lt[n]);
        tcg_gen_lookup_and_goto_ptr();
        gen_set_label(skip);
    }
}

static int insn_len(uint16_t iw0)
{
    if ((iw0 & 0xc000) != 0xc000) {
        return 2;
    }
    if ((iw0 & 0xf800) == 0xc800) {
        return 8;
    }
    return 4;
}

static void bfin_tr_init_disas_context(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    ctx->tb_flags = ctx->base.tb->flags;
}

static void bfin_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void bfin_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(ctx->base.pc_next, 0, 0);
}

static void bfin_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    uint16_t iw0;
    uint32_t opc;
    int len;
    bool ok;

    ctx->pc = ctx->base.pc_next;
    iw0 = translator_lduw_end(cpu_env(cs), &ctx->base, ctx->pc, MO_LE);
    len = insn_len(iw0);
    ctx->pc_next = ctx->pc + len;

    opc = iw0;
    switch (len) {
    case 2:
        ok = decode_insn16(ctx, iw0);
        break;
    case 4: {
        uint16_t iw1 = translator_lduw_end(cpu_env(cs), &ctx->base,
                                          ctx->pc + 2, MO_LE);

        opc = ((uint32_t)iw0 << 16) | iw1;
        ok = decode_insn32(ctx, opc);
        break;
    }
    default: {
        /*
         * A 64-bit instruction is one 32-bit ALU/MAC operation issued in
         * parallel with two 16-bit instructions (chapter 20). Table C-23
         * lists only the non-parallel encodings, so bit 11 of the first word
         * is cleared before the wide half is looked up. Either 16-bit slot
         * may be a NOP, which is what the assembler inserts when the
         * programmer supplies only one.
         *
         * All three issue in the same cycle, so every slot reads the register
         * file as it stood before the instruction. Translating them one after
         * another does not do that, and the compiler relies on the
         * difference: it schedules a value's last use into a 16-bit store
         * while the 32-bit slot already overwrites the register with
         * something else. The firmware stores a thread identifier with
         *
         *     R0 = R0 -|- R0 || [P5] = R0 || NOP
         *
         * which saves the identifier and sets the zero success code in one
         * cycle. Run in sequence, the store saves the zero.
         *
         * Table 20-3 restricts the 16-bit slots to loads, stores and Ireg
         * arithmetic, so only the data registers can be written from both
         * halves. Snapshotting those, running the wide slot, putting the old
         * values back for the narrow slots and then reapplying only what the
         * wide slot changed gives every slot the pre-instruction values, in
         * both directions, whichever half writes.
         *
         * Comparing against the snapshot rather than tracking destinations
         * costs nothing in correctness: a wide slot that writes a register
         * its own value did not need to write it, and a register written by
         * both halves at once is a conflict the assembler rejects.
         */
        TCGv old[BFIN_NUM_DREG], new[BFIN_NUM_DREG];
        uint16_t iw1 = translator_lduw_end(cpu_env(cs), &ctx->base,
                                           ctx->pc + 2, MO_LE);
        uint16_t iw2 = translator_lduw_end(cpu_env(cs), &ctx->base,
                                           ctx->pc + 4, MO_LE);
        uint16_t iw3 = translator_lduw_end(cpu_env(cs), &ctx->base,
                                           ctx->pc + 6, MO_LE);
        int i;

        for (i = 0; i < BFIN_NUM_DREG; i++) {
            old[i] = tcg_temp_new_i32();
            new[i] = tcg_temp_new_i32();
            tcg_gen_mov_i32(old[i], cpu_gpr[i]);
        }

        opc = (((uint32_t)iw0 << 16) | iw1) & ~(0x0800u << 16);
        ok = decode_insn32(ctx, opc);

        for (i = 0; i < BFIN_NUM_DREG; i++) {
            tcg_gen_mov_i32(new[i], cpu_gpr[i]);
            tcg_gen_mov_i32(cpu_gpr[i], old[i]);
        }

        if (ok) {
            ok = decode_insn16(ctx, iw2);
            opc = iw2;
        }
        if (ok) {
            ok = decode_insn16(ctx, iw3);
            opc = iw3;
        }

        for (i = 0; i < BFIN_NUM_DREG; i++) {
            tcg_gen_movcond_i32(TCG_COND_NE, cpu_gpr[i], new[i], old[i],
                                new[i], cpu_gpr[i]);
        }
        break;
    }
    }

    if (!ok) {
        gen_undef(ctx, opc, len);
    }

    gen_loop_end(ctx);

    ctx->base.pc_next = ctx->pc_next;
    if (ctx->base.is_jmp == DISAS_NEXT &&
        ctx->base.pc_next - ctx->base.pc_first >= TARGET_PAGE_SIZE - 8) {
        ctx->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void bfin_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_TOO_MANY:
    case DISAS_NEXT:
        gen_goto_tb(ctx, 0, ctx->base.pc_next);
        break;
    case DISAS_UPDATE:
        tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps bfin_tr_ops = {
    .init_disas_context = bfin_tr_init_disas_context,
    .tb_start           = bfin_tr_tb_start,
    .insn_start         = bfin_tr_insn_start,
    .translate_insn     = bfin_tr_translate_insn,
    .tb_stop            = bfin_tr_tb_stop,
};

void bfin_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext ctx;

    translator_loop(cs, tb, max_insns, pc, host_pc, &bfin_tr_ops, &ctx.base);
}

/* ---- merged instruction families ---- */
#include "insn-a0.c.inc"
#include "insn-a1.c.inc"
#include "insn-a2.c.inc"
#include "insn-a3.c.inc"
#include "insn-a4.c.inc"
#include "insn-a5.c.inc"
/* ---- end merged instruction families ---- */
