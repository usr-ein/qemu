/*
 * TMS320C674x translation
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Three things about this architecture shape everything below.
 *
 * It is VLIW. Instructions arrive in fetch packets of eight words, and the
 * p-bit of each word says whether the next one executes in the same cycle.
 * Everything in one execute packet reads its sources before any of them
 * writes, so a packet has to be translated as a unit: results go to a
 * writeback list and land together at the end. Translating one instruction
 * at a time would be wrong whenever a packet writes a register another
 * instruction in the same packet reads, which compiled code does constantly.
 *
 * Branches are delayed by five execute packets and the compiler fills those
 * slots with real work. So a branch does not end a block here; it records
 * its target, translation carries on through the delay slots, and the jump
 * is emitted when the landing packet is reached. Whether a predicated
 * branch actually executed is only known at run time, so the target and a
 * taken flag go through env and the landing site reads them back.
 *
 * Nearly every instruction is predicated, on one of six registers, through
 * the creg and z fields. Rather than branch around each one, the predicate
 * selects between the new value and the old at writeback. Memory accesses
 * cannot be done that way and get a real branch.
 *
 * Instruction coverage is deliberately partial. Anything not implemented
 * traps by name, which is how the Blackfin's coverage was grown: run the
 * firmware, see what it stopped on, implement that, run again.
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

#include "insn-table.c.inc"

const char *tic6x_mnemonic_name(unsigned int mnem)
{
    return mnem < TIC6X_MNEM_COUNT ? tic6x_mnem_name[mnem] : "?";
}

static TCGv_i32 cpu_gpr[TIC6X_NUM_GPR];
static TCGv_i32 cpu_pc;
static TCGv_i32 cpu_br_target;
static TCGv_i32 cpu_br_taken;
static TCGv_i32 cpu_br_cnt;

#define TIC6X_MAX_WB 8

typedef struct DisasContext {
    DisasContextBase base;

    /* Address of the execute packet being translated. */
    uint32_t packet_pc;

    /*
     * PCE1: the address of the FETCH packet containing the execute packet,
     * which is what every PC-relative constant is measured from. SPRUFE8B
     * says it plainly for B - "added to the address of the first instruction
     * of the fetch packet that contains the branch instruction" - and again
     * for ADDKPC. It is not the execute packet address, and using that
     * instead is wrong by up to 28 bytes whenever an execute packet starts
     * part way into its fetch packet, which is most of them. That sent the
     * firmware's startup to the wrong place and left it calling a null
     * pointer a few hundred packets later.
     */
    uint32_t pce1;

    /*
     * Writeback list for the packet. Register results are computed into
     * temporaries and applied together, which is what makes the parallel
     * instructions in a packet independent.
     */
    struct {
        int reg;
        TCGv_i32 val;
    } wb[TIC6X_MAX_WB];
    int nwb;

    /*
     * Packets until a pending branch lands, or -1 for none. The count is in
     * cycles rather than packets because NOP n occupies n of them.
     */
    int br_countdown;

    /* A branch scheduled by the packet being translated right now does not
       consume one of its own delay slots. */
    bool br_just_set;

    /*
     * The address of the instruction being translated, as opposed to the
     * packet's. A trap that names only the packet sends you looking at the
     * wrong address - an execute packet can span two fetch packets, so the
     * instruction that trapped may be a long way from where the packet
     * started. That happened: a sploop reported at 0x118047c0 was at
     * 0x11804838.
     */
    uint32_t insn_pc;
} DisasContext;

static void wb_add(DisasContext *dc, int reg, TCGv_i32 val)
{
    if (dc->nwb >= TIC6X_MAX_WB) {
        /* Eight functional units, so eight results is the architectural
           maximum; more than that means the packet walk has gone wrong. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tic6x: more than %d results in one execute packet at "
                      "0x%08x\n", TIC6X_MAX_WB, dc->packet_pc);
        return;
    }
    dc->wb[dc->nwb].reg = reg;
    dc->wb[dc->nwb].val = val;
    dc->nwb++;
}

static void wb_flush(DisasContext *dc)
{
    int i;

    for (i = 0; i < dc->nwb; i++) {
        tcg_gen_mov_i32(cpu_gpr[dc->wb[i].reg], dc->wb[i].val);
    }
    dc->nwb = 0;
}

/* ------------------------------------------------------------ decoding */

static uint32_t field_get(const TIC6XFormat *f, TIC6XField id, uint32_t insn)
{
    const TIC6XFieldDef *d = &f->field[id];
    uint32_t v = 0;
    int i;

    for (i = 0; i < d->npieces; i++) {
        v |= ((insn >> d->piece[i].pos) &
              ((1u << d->piece[i].width) - 1)) << d->piece[i].shift;
    }
    return v;
}

static bool field_present(const TIC6XFormat *f, TIC6XField id)
{
    return f->field[id].npieces != 0;
}

/*
 * Find the encoding. binutils walks its table in order and takes the first
 * entry whose format matches and whose fixed fields agree, and so does this:
 * several encodings share a format and are told apart only by those fixed
 * fields, so order is part of the data.
 */
static const TIC6XOpcode *decode(uint32_t insn, int bits, uint32_t expansion,
                                 const TIC6XFormat **fmt_out)
{
    uint32_t match = bits == 16 ? (insn | (expansion << 16)) : insn;
    unsigned i, j;

    for (i = 0; i < ARRAY_SIZE(tic6x_opcodes); i++) {
        const TIC6XOpcode *op = &tic6x_opcodes[i];
        const TIC6XFormat *f = &tic6x_formats[op->fmt];
        bool ok = true;

        if (f->bits != bits || (match & f->mask) != f->cst) {
            continue;
        }
        for (j = 0; j < op->nfix; j++) {
            if (!field_present(f, op->fix[j].field) ||
                field_get(f, op->fix[j].field, insn) != op->fix[j].value) {
                ok = false;
                break;
            }
        }
        if (ok) {
            *fmt_out = f;
            return op;
        }
    }
    return NULL;
}

/* --------------------------------------------------------- predication */

/*
 * creg selects the predicate register and z the sense. Value 0 means the
 * instruction is unconditional, and 7 is reserved - the decoder rejects it,
 * so anything reaching here is one of the six real ones.
 */
static TCGv_i32 insn_pred(uint32_t creg, uint32_t z)
{
    static const int creg_reg[8] = {
        -1,
        TIC6X_REG_B(0), TIC6X_REG_B(1), TIC6X_REG_B(2),
        TIC6X_REG_A(1), TIC6X_REG_A(2), TIC6X_REG_A(0),
        -1,
    };
    TCGv_i32 pred;

    if (creg == 0 || creg_reg[creg] < 0) {
        return NULL;
    }
    pred = tcg_temp_new_i32();
    tcg_gen_setcondi_i32(z ? TCG_COND_EQ : TCG_COND_NE, pred,
                         cpu_gpr[creg_reg[creg]], 0);
    return pred;
}

/* Apply a result under a predicate: false leaves the register alone. */
static void wb_pred(DisasContext *dc, TCGv_i32 pred, int reg, TCGv_i32 val)
{
    if (pred) {
        TCGv_i32 sel = tcg_temp_new_i32();

        tcg_gen_movcond_i32(TCG_COND_NE, sel, pred, tcg_constant_i32(0),
                            val, cpu_gpr[reg]);
        wb_add(dc, reg, sel);
    } else {
        wb_add(dc, reg, val);
    }
}

static int reg_of(uint32_t side, uint32_t num);

/* ------------------------------------------------------ register pairs */

/*
 * A pair is written A5:A4 and the even register holds the low half. The
 * field names the even one; masking bit 0 off is defensive, since the
 * assembler will not emit an odd number here.
 */
static TCGv_i64 pair_get(uint32_t side, uint32_t num)
{
    TCGv_i64 v = tcg_temp_new_i64();

    tcg_gen_concat_i32_i64(v, cpu_gpr[reg_of(side, num & ~1u)],
                           cpu_gpr[reg_of(side, (num & ~1u) | 1u)]);
    return v;
}

static void pair_set(DisasContext *dc, TCGv_i32 pred, uint32_t side,
                     uint32_t num, TCGv_i64 val)
{
    TCGv_i32 lo = tcg_temp_new_i32();
    TCGv_i32 hi = tcg_temp_new_i32();

    tcg_gen_extr_i64_i32(lo, hi, val);
    wb_pred(dc, pred, reg_of(side, num & ~1u), lo);
    wb_pred(dc, pred, reg_of(side, (num & ~1u) | 1u), hi);
}

/*
 * The long tail of the instruction set - packed SIMD, the multiply variants,
 * double precision - is routed to helpers in alu.c.inc rather than written
 * as inline TCG, because getting a saturation boundary or a sign extension
 * wrong in a hundred hand-rolled cases is a certainty and these are not
 * where the time goes. What differs between them is only the shape of their
 * operands, which is what these four groups are.
 */
static bool routed_alu2(uint16_t m)
{
    switch (m) {
    case TIC6X_MNEM_addu: case TIC6X_MNEM_subu:
    case TIC6X_MNEM_rotl: case TIC6X_MNEM_sshl:
    case TIC6X_MNEM_lmbd: case TIC6X_MNEM_norm:
    case TIC6X_MNEM_shlmb: case TIC6X_MNEM_shrmb: case TIC6X_MNEM_subc:
    case TIC6X_MNEM_sadd2: case TIC6X_MNEM_ssub2: case TIC6X_MNEM_saddsu2:
    case TIC6X_MNEM_avg2: case TIC6X_MNEM_min2: case TIC6X_MNEM_max2:
    case TIC6X_MNEM_shr2: case TIC6X_MNEM_shru2: case TIC6X_MNEM_packh2:
    case TIC6X_MNEM_packl4: case TIC6X_MNEM_packh4:
    case TIC6X_MNEM_spack2: case TIC6X_MNEM_spacku4:
    case TIC6X_MNEM_xpnd2: case TIC6X_MNEM_xpnd4: case TIC6X_MNEM_unpklu4:
    case TIC6X_MNEM_add4: case TIC6X_MNEM_sub4: case TIC6X_MNEM_saddu4:
    case TIC6X_MNEM_subabs4: case TIC6X_MNEM_minu4: case TIC6X_MNEM_maxu4:
    case TIC6X_MNEM_avgu4:
    case TIC6X_MNEM_cmpeq2: case TIC6X_MNEM_cmpgt2:
    case TIC6X_MNEM_cmpeq4: case TIC6X_MNEM_cmpgtu4:
    case TIC6X_MNEM_mpyi: case TIC6X_MNEM_mpyhi: case TIC6X_MNEM_mpyli:
    case TIC6X_MNEM_mpylshu: case TIC6X_MNEM_mpyluhs:
    case TIC6X_MNEM_mpyhuls: case TIC6X_MNEM_mpyhslu:
    case TIC6X_MNEM_mpylhu: case TIC6X_MNEM_mpyhlu:
    case TIC6X_MNEM_mpyhsu: case TIC6X_MNEM_mpyhus:
    case TIC6X_MNEM_smpylh: case TIC6X_MNEM_smpyhl:
    case TIC6X_MNEM_dotp2: case TIC6X_MNEM_dotpn2:
    case TIC6X_MNEM_dotprsu2: case TIC6X_MNEM_dotpnrsu2:
    case TIC6X_MNEM_dotpu4: case TIC6X_MNEM_dotpsu4:
    case TIC6X_MNEM_gmpy4:
        return true;
    default:
        return false;
    }
}

static bool routed_alu2_wide(uint16_t m)
{
    switch (m) {
    case TIC6X_MNEM_mpyid: case TIC6X_MNEM_mpy32u:
    case TIC6X_MNEM_mpy32su: case TIC6X_MNEM_mpy32us:
    case TIC6X_MNEM_mpy2: case TIC6X_MNEM_smpy2:
    case TIC6X_MNEM_mpyu4: case TIC6X_MNEM_mpysu4:
    case TIC6X_MNEM_cmpy: case TIC6X_MNEM_cmpyr1:
    case TIC6X_MNEM_addsub: case TIC6X_MNEM_addsub2:
    case TIC6X_MNEM_saddsub: case TIC6X_MNEM_saddsub2:
    case TIC6X_MNEM_dmv:
    case TIC6X_MNEM_ddotp4: case TIC6X_MNEM_ddotpl2:
    case TIC6X_MNEM_ddotpl2r:
        return true;
    default:
        return false;
    }
}

/* ------------------------------------------------------ one instruction */

static int reg_of(uint32_t side, uint32_t num)
{
    return (side ? 32 : 0) + (num & 31);
}

/*
 * src1 as a value. It holds either a register number or a small constant
 * depending on the encoding, and the two forms share a format - ADD .L1
 * A3,A4,A5 and ADD .L1 5,A4,A5 differ only in the op field and in how src1
 * is read. The generated table carries that from binutils' ENC lists, which
 * is the only place it is written down; without it the constant forms
 * quietly compute with a register number as though it were a value.
 */
static TCGv_i32 src1_value(const TIC6XFormat *f, const TIC6XOpcode *op,
                           uint32_t insn, uint32_t s)
{
    uint32_t raw = field_get(f, TIC6X_FLD_src1, insn);
    TCGv_i32 v = tcg_temp_new_i32();

    switch (op->enc[TIC6X_FLD_src1]) {
    case TIC6X_ENC_SCST:
        /* Five bits signed, in every form the .L and .S units use. */
        tcg_gen_movi_i32(v, (int32_t)(raw << 27) >> 27);
        break;
    case TIC6X_ENC_UCST:
        tcg_gen_movi_i32(v, raw);
        break;
    case TIC6X_ENC_SCST_NEG:
        tcg_gen_movi_i32(v, -(int32_t)((int32_t)(raw << 27) >> 27));
        break;
    default:
        tcg_gen_mov_i32(v, cpu_gpr[reg_of(s, raw)]);
        break;
    }
    return v;
}

/* The addressing modes of the .D unit load and store, SPRUFE8B 3.8. */
static TCGv_i32 addr_mode(DisasContext *dc, TCGv_i32 pred,
                          const TIC6XFormat *f, uint32_t insn, int scale)
{
    uint32_t mode = field_get(f, TIC6X_FLD_mode, insn);
    uint32_t y = field_get(f, TIC6X_FLD_y, insn);
    uint32_t baseN = field_get(f, TIC6X_FLD_baseR, insn);
    uint32_t offN = field_get(f, TIC6X_FLD_offsetR, insn);
    int base = reg_of(y, baseN);
    TCGv_i32 off = tcg_temp_new_i32();
    TCGv_i32 addr = tcg_temp_new_i32();
    bool reg_offset = mode & 4;
    bool modify = mode & 8;
    bool post = modify && (mode & 2);
    bool add = mode & 1;

    if (reg_offset) {
        tcg_gen_shli_i32(off, cpu_gpr[reg_of(y, offN)], scale);
    } else {
        tcg_gen_movi_i32(off, offN << scale);
    }

    if (post) {
        /* The access uses the base, then the base moves. */
        tcg_gen_mov_i32(addr, cpu_gpr[base]);
    } else if (add) {
        tcg_gen_add_i32(addr, cpu_gpr[base], off);
    } else {
        tcg_gen_sub_i32(addr, cpu_gpr[base], off);
    }

    if (modify) {
        TCGv_i32 nb = tcg_temp_new_i32();

        if (post) {
            if (add) {
                tcg_gen_add_i32(nb, cpu_gpr[base], off);
            } else {
                tcg_gen_sub_i32(nb, cpu_gpr[base], off);
            }
        } else {
            tcg_gen_mov_i32(nb, addr);
        }
        wb_pred(dc, pred, base, nb);
    }
    return addr;
}

static void gen_load(DisasContext *dc, TCGv_i32 pred, int dst,
                     TCGv_i32 addr, MemOp op)
{
    TCGv_i32 val = tcg_temp_new_i32();
    TCGLabel *skip = NULL;

    if (pred) {
        skip = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, pred, 0, skip);
    }
    tcg_gen_qemu_ld_i32(val, addr, 0, op);
    tcg_gen_mov_i32(cpu_gpr[dst], val);
    if (skip) {
        gen_set_label(skip);
    }
}

static void gen_store(DisasContext *dc, TCGv_i32 pred, int src,
                      TCGv_i32 addr, MemOp op)
{
    TCGLabel *skip = NULL;

    if (pred) {
        skip = gen_new_label();
        tcg_gen_brcondi_i32(TCG_COND_EQ, pred, 0, skip);
    }
    tcg_gen_qemu_st_i32(cpu_gpr[src], addr, 0, op);
    if (skip) {
        gen_set_label(skip);
    }
}

/*
 * Translate one instruction of an execute packet. Returns the number of
 * cycles it occupies, which is one for everything except NOP n - and that
 * matters, because those cycles are what a branch's delay slots are counted
 * in.
 */
static int trans_one(DisasContext *dc, uint32_t insn, int bits,
                     uint32_t expansion)
{
    const TIC6XFormat *f;
    const TIC6XOpcode *op = decode(insn, bits, expansion, &f);
    TCGv_i32 pred;
    uint32_t creg = 0, z = 0, s, x;
    int cycles = 1;

    if (!op) {
        tcg_gen_movi_i32(cpu_pc, dc->insn_pc);
        gen_helper_illegal(tcg_env, tcg_constant_i32(insn));
        return 1;
    }

    if (field_present(f, TIC6X_FLD_creg)) {
        creg = field_get(f, TIC6X_FLD_creg, insn);
        z = field_get(f, TIC6X_FLD_z, insn);
    }
    pred = insn_pred(creg, z);
    s = field_present(f, TIC6X_FLD_s) ? field_get(f, TIC6X_FLD_s, insn) : 0;
    x = field_present(f, TIC6X_FLD_x) ? field_get(f, TIC6X_FLD_x, insn) : 0;

    /*
     * The routed groups first: they all take the same operand shape and
     * differ only in which helper case runs, so there is nothing to say
     * about them one at a time.
     */
    if (routed_alu2(op->mnem)) {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        gen_helper_alu2(v, tcg_env, tcg_constant_i32(op->mnem), a,
                        cpu_gpr[src2]);
        wb_pred(dc, pred, dst, v);
        return cycles;
    }
    if (routed_alu2_wide(op->mnem)) {
        uint32_t dstn = field_get(f, TIC6X_FLD_dst, insn);
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i64 v = tcg_temp_new_i64();

        gen_helper_alu2_wide(v, tcg_env, tcg_constant_i32(op->mnem), a,
                             cpu_gpr[src2]);
        pair_set(dc, pred, s, dstn, v);
        return cycles;
    }

    switch (op->mnem) {
    case TIC6X_MNEM_adddp:
    case TIC6X_MNEM_subdp:
    case TIC6X_MNEM_mpydp:
    case TIC6X_MNEM_absdp:
    case TIC6X_MNEM_rcpdp:
    case TIC6X_MNEM_rsqrdp: {
        /* Both operands and the result are register pairs. */
        uint32_t dstn = field_get(f, TIC6X_FLD_dst, insn);
        TCGv_i64 a = pair_get(s, field_get(f, TIC6X_FLD_src1, insn));
        TCGv_i64 b = pair_get(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i64 v = tcg_temp_new_i64();

        gen_helper_dp2(v, tcg_env, tcg_constant_i32(op->mnem), a, b);
        pair_set(dc, pred, s, dstn, v);
        break;
    }

    case TIC6X_MNEM_cmpeqdp:
    case TIC6X_MNEM_cmpgtdp:
    case TIC6X_MNEM_cmpltdp: {
        /* Two pairs in, a one-or-zero flag out. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        TCGv_i64 a = pair_get(s, field_get(f, TIC6X_FLD_src1, insn));
        TCGv_i64 b = pair_get(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        gen_helper_dp_cmp(v, tcg_env, tcg_constant_i32(op->mnem), a, b);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_mpyspdp: {
        /* A single times a double, into a double. */
        uint32_t dstn = field_get(f, TIC6X_FLD_dst, insn);
        int src1 = reg_of(s, field_get(f, TIC6X_FLD_src1, insn));
        TCGv_i64 b = pair_get(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i64 v = tcg_temp_new_i64();

        gen_helper_spdp2(v, tcg_env, tcg_constant_i32(op->mnem),
                         cpu_gpr[src1], b);
        pair_set(dc, pred, s, dstn, v);
        break;
    }

    case TIC6X_MNEM_mpysp2dp: {
        /* Two singles, into a double. */
        uint32_t dstn = field_get(f, TIC6X_FLD_dst, insn);
        int src1 = reg_of(s, field_get(f, TIC6X_FLD_src1, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i64 v = tcg_temp_new_i64();

        gen_helper_sp2dp(v, tcg_env, tcg_constant_i32(op->mnem),
                         cpu_gpr[src1], cpu_gpr[src2]);
        pair_set(dc, pred, s, dstn, v);
        break;
    }

    case TIC6X_MNEM_spint:
    case TIC6X_MNEM_sptrunc:
    case TIC6X_MNEM_intsp:
    case TIC6X_MNEM_intspu:
    case TIC6X_MNEM_rsqrsp:
    case TIC6X_MNEM_rcpsp: {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        gen_helper_sp1(v, tcg_env, tcg_constant_i32(op->mnem),
                       cpu_gpr[src2]);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_dpsp:
    case TIC6X_MNEM_dpint:
    case TIC6X_MNEM_dptrunc: {
        /* A pair in, a single register out. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        TCGv_i64 b = pair_get(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        gen_helper_dp_to_w(v, tcg_env, tcg_constant_i32(op->mnem), b);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_spdp:
    case TIC6X_MNEM_intdp:
    case TIC6X_MNEM_intdpu: {
        /* A single register in, a pair out. */
        uint32_t dstn = field_get(f, TIC6X_FLD_dst, insn);
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i64 v = tcg_temp_new_i64();

        gen_helper_w_to_dp(v, tcg_env, tcg_constant_i32(op->mnem),
                           cpu_gpr[src2]);
        pair_set(dc, pred, s, dstn, v);
        break;
    }

    case TIC6X_MNEM_neg:
    case TIC6X_MNEM_not:
    case TIC6X_MNEM_abs: {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        gen_helper_alu1(v, tcg_constant_i32(op->mnem), cpu_gpr[src2]);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_nop: {
        /*
         * NOP n idles for n cycles. Nothing here models cycles, but the
         * count still has to be honoured because branch delay slots are
         * measured in them: a NOP 5 in a delay slot fills all five.
         */
        uint32_t src = field_present(f, TIC6X_FLD_src) ?
                       field_get(f, TIC6X_FLD_src, insn) : 0;
        cycles = src + 1;
        break;
    }

    case TIC6X_MNEM_mvk: {
        /* Sign-extended 16-bit constant into a register. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int32_t cst = (int32_t)(field_get(f, TIC6X_FLD_cst, insn) << 16) >> 16;
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_movi_i32(v, cst);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_mvkh:
    case TIC6X_MNEM_mvklh: {
        /* Replace the top half, keep the bottom. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        uint32_t cst = field_get(f, TIC6X_FLD_cst, insn) & 0xffff;
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_andi_i32(v, cpu_gpr[dst], 0x0000ffff);
        tcg_gen_ori_i32(v, v, cst << 16);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_mv: {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_mov_i32(v, cpu_gpr[src2]);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_add:
    case TIC6X_MNEM_sub:
    case TIC6X_MNEM_and:
    case TIC6X_MNEM_or:
    case TIC6X_MNEM_xor: {
        /*
         * The three-operand arithmetic that dominates the histogram. src1
         * is either a register on the instruction's own side or a five-bit
         * constant, depending on the encoding; the opcode table tells them
         * apart by format, and the constant forms are the ones whose src1
         * field the manual calls scst5.
         */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        switch (op->mnem) {
        case TIC6X_MNEM_add:
            tcg_gen_add_i32(v, a, cpu_gpr[src2]);
            break;
        case TIC6X_MNEM_sub:
            tcg_gen_sub_i32(v, a, cpu_gpr[src2]);
            break;
        case TIC6X_MNEM_and:
            tcg_gen_and_i32(v, a, cpu_gpr[src2]);
            break;
        case TIC6X_MNEM_or:
            tcg_gen_or_i32(v, a, cpu_gpr[src2]);
            break;
        default:
            tcg_gen_xor_i32(v, a, cpu_gpr[src2]);
            break;
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_addk: {
        /* dst += a 16-bit signed constant. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int32_t cst = (int32_t)(field_get(f, TIC6X_FLD_cst, insn) << 16) >> 16;
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_addi_i32(v, cpu_gpr[dst], cst);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_ldw:
    case TIC6X_MNEM_ldh:
    case TIC6X_MNEM_ldhu:
    case TIC6X_MNEM_ldb:
    case TIC6X_MNEM_ldbu: {
        int dst = reg_of(field_get(f, TIC6X_FLD_s, insn),
                         field_get(f, TIC6X_FLD_srcdst, insn));
        int scale = op->mnem == TIC6X_MNEM_ldw ? 2 :
                    (op->mnem == TIC6X_MNEM_ldh ||
                     op->mnem == TIC6X_MNEM_ldhu) ? 1 : 0;
        MemOp mo = op->mnem == TIC6X_MNEM_ldw ? MO_LEUL :
                   op->mnem == TIC6X_MNEM_ldh ? MO_LESW :
                   op->mnem == TIC6X_MNEM_ldhu ? MO_LEUW :
                   op->mnem == TIC6X_MNEM_ldb ? MO_SB : MO_UB;
        TCGv_i32 addr = addr_mode(dc, pred, f, insn, scale);

        gen_load(dc, pred, dst, addr, mo);
        break;
    }

    case TIC6X_MNEM_stw:
    case TIC6X_MNEM_sth:
    case TIC6X_MNEM_stb: {
        int src = reg_of(field_get(f, TIC6X_FLD_s, insn),
                         field_get(f, TIC6X_FLD_srcdst, insn));
        int scale = op->mnem == TIC6X_MNEM_stw ? 2 :
                    op->mnem == TIC6X_MNEM_sth ? 1 : 0;
        MemOp mo = op->mnem == TIC6X_MNEM_stw ? MO_LEUL :
                   op->mnem == TIC6X_MNEM_sth ? MO_LEUW : MO_UB;
        TCGv_i32 addr = addr_mode(dc, pred, f, insn, scale);

        gen_store(dc, pred, src, addr, mo);
        break;
    }

    case TIC6X_MNEM_cmpeq:
    case TIC6X_MNEM_cmpgt:
    case TIC6X_MNEM_cmpgtu:
    case TIC6X_MNEM_cmplt:
    case TIC6X_MNEM_cmpltu: {
        /* dst is 1 or 0, which is what the predicate registers then test. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();
        TCGCond c;

        switch (op->mnem) {
        case TIC6X_MNEM_cmpeq:  c = TCG_COND_EQ;  break;
        case TIC6X_MNEM_cmpgt:  c = TCG_COND_LT;  break;
        case TIC6X_MNEM_cmpgtu: c = TCG_COND_LTU; break;
        case TIC6X_MNEM_cmplt:  c = TCG_COND_GT;  break;
        default:                c = TCG_COND_GTU; break;
        }
        /*
         * The operand order is src2 op src1 - CMPGT .L1 A3,A4,A5 sets A5
         * when A3 > A4, and src1 is the first written operand - so the
         * comparison is inverted relative to the register order here.
         */
        tcg_gen_setcond_i32(c, v, cpu_gpr[src2], a);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_extu:
    case TIC6X_MNEM_ext: {
        /*
         * Shift left by csta, then right by cstb, arithmetic for ext and
         * logical for extu. That is how the C6000 spells a bit-field
         * extract, and the two constants come as one ten-bit src1 in the
         * constant form.
         */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        if (field_present(f, TIC6X_FLD_csta)) {
            uint32_t csta = field_get(f, TIC6X_FLD_csta, insn);
            uint32_t cstb = field_get(f, TIC6X_FLD_cstb, insn);

            tcg_gen_shli_i32(v, cpu_gpr[src2], csta);
            if (op->mnem == TIC6X_MNEM_ext) {
                tcg_gen_sari_i32(v, v, cstb);
            } else {
                tcg_gen_shri_i32(v, v, cstb);
            }
        } else {
            /* Register form: src1 holds csta in 9:5 and cstb in 4:0. */
            TCGv_i32 amt = cpu_gpr[reg_of(s, field_get(f, TIC6X_FLD_src1,
                                                       insn))];
            TCGv_i32 sa = tcg_temp_new_i32();
            TCGv_i32 sb = tcg_temp_new_i32();

            tcg_gen_extract_i32(sa, amt, 5, 5);
            tcg_gen_extract_i32(sb, amt, 0, 5);
            tcg_gen_shl_i32(v, cpu_gpr[src2], sa);
            if (op->mnem == TIC6X_MNEM_ext) {
                tcg_gen_sar_i32(v, v, sb);
            } else {
                tcg_gen_shr_i32(v, v, sb);
            }
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_clr:
    case TIC6X_MNEM_set: {
        /* Clear or set the bits from csta to cstb inclusive. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 v = tcg_temp_new_i32();

        if (field_present(f, TIC6X_FLD_csta)) {
            uint32_t csta = field_get(f, TIC6X_FLD_csta, insn);
            uint32_t cstb = field_get(f, TIC6X_FLD_cstb, insn);
            uint32_t width = cstb >= csta ? cstb - csta + 1 : 0;
            uint32_t mask = width >= 32 ? 0xffffffffu
                                        : (((1u << width) - 1) << csta);

            if (op->mnem == TIC6X_MNEM_set) {
                tcg_gen_ori_i32(v, cpu_gpr[src2], mask);
            } else {
                tcg_gen_andi_i32(v, cpu_gpr[src2], ~mask);
            }
            wb_pred(dc, pred, dst, v);
        } else {
            gen_helper_unimplemented(tcg_env, tcg_constant_i32(insn),
                                     tcg_constant_i32(op->mnem));
        }
        break;
    }

    case TIC6X_MNEM_shl:
    case TIC6X_MNEM_shr:
    case TIC6X_MNEM_shru: {
        /*
         * The shift count is src1 and the value src2, which is the reverse
         * of the usual reading of the operand order.
         */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 amt = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();
        TCGv_i32 cap = tcg_temp_new_i32();

        /* A count above 31 saturates the result rather than wrapping. */
        tcg_gen_umin_i32(cap, amt, tcg_constant_i32(31));
        switch (op->mnem) {
        case TIC6X_MNEM_shl:
            tcg_gen_shl_i32(v, cpu_gpr[src2], cap);
            break;
        case TIC6X_MNEM_shr:
            tcg_gen_sar_i32(v, cpu_gpr[src2], cap);
            break;
        default:
            tcg_gen_shr_i32(v, cpu_gpr[src2], cap);
            break;
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_addkpc: {
        /*
         * dst = the address of this execute packet plus a scaled seven-bit
         * signed constant, then src2 cycles of nop. This is how a call
         * builds its return address: the branch goes out and addkpc, in one
         * of its delay slots, leaves the address to come back to.
         */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int32_t disp = (int32_t)(field_get(f, TIC6X_FLD_src1, insn) << 25) >> 25;
        uint32_t nops = field_get(f, TIC6X_FLD_src2, insn);
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_movi_i32(v, dc->pce1 + (disp << 2));
        wb_pred(dc, pred, dst, v);
        cycles = nops + 1;
        break;
    }

    case TIC6X_MNEM_mvc: {
        /*
         * Between a general register and a control register. The two
         * directions share a format and differ only in the op field, and
         * which field holds the control register number swaps with them:
         *
         *   op 0x0f  from_cr:  src2 is crlo, dst is the register
         *   op 0x0e  to_cr:    src2 is the register, dst is crlo
         *
         * Both are .S2 only, so the register side is always B.
         */
        uint32_t opf = field_get(f, TIC6X_FLD_op, insn);
        uint32_t f_src2 = field_get(f, TIC6X_FLD_src2, insn);
        uint32_t f_dst = field_get(f, TIC6X_FLD_dst, insn);

        if (opf == 0x0f) {
            TCGv_i32 v = tcg_temp_new_i32();

            gen_helper_read_creg(v, tcg_env, tcg_constant_i32(f_src2));
            wb_pred(dc, pred, reg_of(s, f_dst), v);
        } else {
            TCGLabel *skip = NULL;

            if (pred) {
                skip = gen_new_label();
                tcg_gen_brcondi_i32(TCG_COND_EQ, pred, 0, skip);
            }
            gen_helper_write_creg(tcg_env, tcg_constant_i32(f_dst),
                                  cpu_gpr[reg_of(s ^ x, f_src2)]);
            if (skip) {
                gen_set_label(skip);
            }
        }
        break;
    }

    case TIC6X_MNEM_lddw:
    case TIC6X_MNEM_ldndw:
    case TIC6X_MNEM_stdw:
    case TIC6X_MNEM_stndw: {
        /*
         * Doubleword, into or out of a register pair. The pair is written
         * A5:A4 and the even register holds the low half, so the field names
         * the even one and the odd one follows. The nonaligned forms differ
         * only in not requiring alignment, which this model never enforced
         * anyway - there is no alignment fault here to avoid.
         */
        bool is_load = op->mnem == TIC6X_MNEM_lddw ||
                       op->mnem == TIC6X_MNEM_ldndw;
        uint32_t side = field_get(f, TIC6X_FLD_s, insn);
        uint32_t num = field_get(f, TIC6X_FLD_srcdst, insn);
        int lo = reg_of(side, num & ~1u);
        int hi = reg_of(side, (num & ~1u) | 1u);
        TCGv_i32 addr = addr_mode(dc, pred, f, insn, 3);
        TCGv_i32 hiaddr = tcg_temp_new_i32();
        TCGLabel *skip = NULL;

        tcg_gen_addi_i32(hiaddr, addr, 4);
        if (pred) {
            skip = gen_new_label();
            tcg_gen_brcondi_i32(TCG_COND_EQ, pred, 0, skip);
        }
        if (is_load) {
            tcg_gen_qemu_ld_i32(cpu_gpr[lo], addr, 0, MO_LEUL);
            tcg_gen_qemu_ld_i32(cpu_gpr[hi], hiaddr, 0, MO_LEUL);
        } else {
            tcg_gen_qemu_st_i32(cpu_gpr[lo], addr, 0, MO_LEUL);
            tcg_gen_qemu_st_i32(cpu_gpr[hi], hiaddr, 0, MO_LEUL);
        }
        if (skip) {
            gen_set_label(skip);
        }
        break;
    }

    case TIC6X_MNEM_cmtl: {
        /*
         * Commit to memory and load - an atomic read for the shared-memory
         * protocol on parts that have one. With no second master inside this
         * model there is nothing to arbitrate against, so it is an ordinary
         * doubleword load; the linked-store side would need modelling before
         * that is more than a convenience.
         */
        uint32_t num = field_get(f, TIC6X_FLD_dst, insn);
        int lo = reg_of(s, num & ~1u);
        int hi = reg_of(s, (num & ~1u) | 1u);
        int base = reg_of(s, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 hiaddr = tcg_temp_new_i32();
        TCGLabel *skip = NULL;

        tcg_gen_addi_i32(hiaddr, cpu_gpr[base], 4);
        if (pred) {
            skip = gen_new_label();
            tcg_gen_brcondi_i32(TCG_COND_EQ, pred, 0, skip);
        }
        tcg_gen_qemu_ld_i32(cpu_gpr[lo], cpu_gpr[base], 0, MO_LEUL);
        tcg_gen_qemu_ld_i32(cpu_gpr[hi], hiaddr, 0, MO_LEUL);
        if (skip) {
            gen_set_label(skip);
        }
        break;
    }

    case TIC6X_MNEM_ldnw:
    case TIC6X_MNEM_stnw: {
        /* Nonaligned word; the same as ldw and stw to a model that does not
           fault on alignment. */
        int reg = reg_of(field_get(f, TIC6X_FLD_s, insn),
                         field_get(f, TIC6X_FLD_srcdst, insn));
        TCGv_i32 addr = addr_mode(dc, pred, f, insn, 2);

        if (op->mnem == TIC6X_MNEM_ldnw) {
            gen_load(dc, pred, reg, addr, MO_LEUL);
        } else {
            gen_store(dc, pred, reg, addr, MO_LEUL);
        }
        break;
    }

    case TIC6X_MNEM_addab:
    case TIC6X_MNEM_addah:
    case TIC6X_MNEM_addaw:
    case TIC6X_MNEM_addad:
    case TIC6X_MNEM_subab:
    case TIC6X_MNEM_subah:
    case TIC6X_MNEM_subaw: {
        /*
         * Address arithmetic on the .D unit: the offset is scaled by the
         * size the mnemonic names, which is what makes them different from
         * a plain add.
         */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();
        int scale = (op->mnem == TIC6X_MNEM_addab ||
                     op->mnem == TIC6X_MNEM_subab) ? 0 :
                    (op->mnem == TIC6X_MNEM_addah ||
                     op->mnem == TIC6X_MNEM_subah) ? 1 :
                    (op->mnem == TIC6X_MNEM_addad) ? 3 : 2;
        bool sub = op->mnem == TIC6X_MNEM_subab ||
                   op->mnem == TIC6X_MNEM_subah ||
                   op->mnem == TIC6X_MNEM_subaw;

        tcg_gen_shli_i32(v, a, scale);
        if (sub) {
            tcg_gen_sub_i32(v, cpu_gpr[src2], v);
        } else {
            tcg_gen_add_i32(v, cpu_gpr[src2], v);
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_andn: {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_andc_i32(v, a, cpu_gpr[src2]);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_mpy:
    case TIC6X_MNEM_mpyu:
    case TIC6X_MNEM_mpysu:
    case TIC6X_MNEM_mpyus:
    case TIC6X_MNEM_mpyh:
    case TIC6X_MNEM_mpyhu:
    case TIC6X_MNEM_mpylh:
    case TIC6X_MNEM_mpyhl:
    case TIC6X_MNEM_smpyh:
    case TIC6X_MNEM_smpy: {
        /*
         * The sixteen-by-sixteen multiplies. Which half of each source is
         * taken, and whether it is signed, is spelled out by the mnemonic:
         * the trailing h or l on each side, and u or s for the sign. The
         * smpy forms shift the result left by one and saturate, which is
         * the fractional Q15 convention.
         */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 lhs = tcg_temp_new_i32();
        TCGv_i32 rhs = tcg_temp_new_i32();
        TCGv_i32 v = tcg_temp_new_i32();
        bool hi1 = op->mnem == TIC6X_MNEM_mpyh ||
                   op->mnem == TIC6X_MNEM_mpyhu ||
                   op->mnem == TIC6X_MNEM_mpyhl ||
                   op->mnem == TIC6X_MNEM_smpyh;
        bool hi2 = op->mnem == TIC6X_MNEM_mpyh ||
                   op->mnem == TIC6X_MNEM_mpyhu ||
                   op->mnem == TIC6X_MNEM_mpylh ||
                   op->mnem == TIC6X_MNEM_smpyh;
        bool u1 = op->mnem == TIC6X_MNEM_mpyu ||
                  op->mnem == TIC6X_MNEM_mpyhu ||
                  op->mnem == TIC6X_MNEM_mpyus;
        bool u2 = op->mnem == TIC6X_MNEM_mpyu ||
                  op->mnem == TIC6X_MNEM_mpyhu ||
                  op->mnem == TIC6X_MNEM_mpysu;

        if (u1) {
            tcg_gen_extract_i32(lhs, a, hi1 ? 16 : 0, 16);
        } else {
            tcg_gen_sextract_i32(lhs, a, hi1 ? 16 : 0, 16);
        }
        if (u2) {
            tcg_gen_extract_i32(rhs, cpu_gpr[src2], hi2 ? 16 : 0, 16);
        } else {
            tcg_gen_sextract_i32(rhs, cpu_gpr[src2], hi2 ? 16 : 0, 16);
        }
        tcg_gen_mul_i32(v, lhs, rhs);
        if (op->mnem == TIC6X_MNEM_smpy ||
            op->mnem == TIC6X_MNEM_smpyh) {
            /* Left shift by one, saturating the one case that overflows:
               0x8000 * 0x8000 gives 0x40000000, which doubles out of range. */
            TCGv_i32 sat = tcg_temp_new_i32();

            tcg_gen_shli_i32(sat, v, 1);
            tcg_gen_movcond_i32(TCG_COND_EQ, v, v,
                                tcg_constant_i32(0x40000000),
                                tcg_constant_i32(0x7fffffff), sat);
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_mpy32: {
        /* The low 32 bits of a 32 by 32 product. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_mul_i32(v, a, cpu_gpr[src2]);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_add2:
    case TIC6X_MNEM_sub2: {
        /* Two sixteen-bit lanes, no carry between them. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 lo = tcg_temp_new_i32();
        TCGv_i32 hi = tcg_temp_new_i32();
        TCGv_i32 v = tcg_temp_new_i32();

        if (op->mnem == TIC6X_MNEM_add2) {
            tcg_gen_add_i32(lo, a, cpu_gpr[src2]);
            tcg_gen_add_i32(hi, a, cpu_gpr[src2]);
        } else {
            tcg_gen_sub_i32(lo, a, cpu_gpr[src2]);
            tcg_gen_sub_i32(hi, a, cpu_gpr[src2]);
        }
        tcg_gen_andi_i32(lo, lo, 0x0000ffff);
        tcg_gen_andi_i32(hi, hi, 0xffff0000);
        tcg_gen_or_i32(v, lo, hi);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_pack2:
    case TIC6X_MNEM_packlh2:
    case TIC6X_MNEM_packhl2: {
        /* Build a word from one half of each source. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();
        TCGv_i32 t = tcg_temp_new_i32();
        /* pack2 takes both low halves, packlh2 src1 low and src2 high,
           packhl2 src1 high and src2 low. The result's high half comes
           from src1. */
        bool hi_from_src1_high = op->mnem == TIC6X_MNEM_packhl2;
        bool lo_from_src2_high = op->mnem == TIC6X_MNEM_packlh2;

        tcg_gen_extract_i32(t, a, hi_from_src1_high ? 16 : 0, 16);
        tcg_gen_shli_i32(v, t, 16);
        tcg_gen_extract_i32(t, cpu_gpr[src2], lo_from_src2_high ? 16 : 0, 16);
        tcg_gen_or_i32(v, v, t);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_sadd:
    case TIC6X_MNEM_ssub: {
        /* Saturating 32-bit add and subtract. */
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        TCGv_i32 ovf = tcg_temp_new_i32();
        TCGv_i32 t = tcg_temp_new_i32();

        if (op->mnem == TIC6X_MNEM_sadd) {
            tcg_gen_add_i32(v, a, cpu_gpr[src2]);
            /* Overflow when both operands differ from the result in sign. */
            tcg_gen_xor_i32(ovf, a, v);
            tcg_gen_xor_i32(t, cpu_gpr[src2], v);
            tcg_gen_and_i32(ovf, ovf, t);
        } else {
            tcg_gen_sub_i32(v, a, cpu_gpr[src2]);
            /* And for subtract, when the operands differ and the result
               takes the subtrahend's sign. */
            tcg_gen_xor_i32(ovf, a, cpu_gpr[src2]);
            tcg_gen_xor_i32(t, a, v);
            tcg_gen_and_i32(ovf, ovf, t);
        }
        /* On overflow the result clamps to the end it ran off. */
        tcg_gen_sari_i32(t, v, 31);
        tcg_gen_xori_i32(t, t, 0x7fffffff);
        tcg_gen_movcond_i32(TCG_COND_LT, v, ovf, tcg_constant_i32(0), t, v);
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_addsp:
    case TIC6X_MNEM_subsp:
    case TIC6X_MNEM_mpysp: {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        switch (op->mnem) {
        case TIC6X_MNEM_addsp:
            gen_helper_addsp(v, tcg_env, a, cpu_gpr[src2]);
            break;
        case TIC6X_MNEM_subsp:
            gen_helper_subsp(v, tcg_env, a, cpu_gpr[src2]);
            break;
        default:
            gen_helper_mpysp(v, tcg_env, a, cpu_gpr[src2]);
            break;
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_cmpeqsp:
    case TIC6X_MNEM_cmpgtsp:
    case TIC6X_MNEM_cmpltsp: {
        int dst = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i32 v = tcg_temp_new_i32();

        switch (op->mnem) {
        case TIC6X_MNEM_cmpeqsp:
            gen_helper_cmpeqsp(v, tcg_env, a, cpu_gpr[src2]);
            break;
        case TIC6X_MNEM_cmpgtsp:
            gen_helper_cmpgtsp(v, tcg_env, a, cpu_gpr[src2]);
            break;
        default:
            gen_helper_cmpltsp(v, tcg_env, a, cpu_gpr[src2]);
            break;
        }
        wb_pred(dc, pred, dst, v);
        break;
    }

    case TIC6X_MNEM_bdec:
    case TIC6X_MNEM_bpos: {
        /*
         * Branch on the sign of a register, PC-relative by a ten-bit signed
         * displacement. BDEC also decrements the register, unconditionally -
         * the decrement happens whether or not the branch is taken, which is
         * what makes it a loop counter.
         */
        int src = reg_of(s, field_get(f, TIC6X_FLD_src2, insn));
        int32_t disp = (int32_t)(field_get(f, TIC6X_FLD_cst, insn) << 22) >> 22;
        TCGv_i32 take = tcg_temp_new_i32();
        TCGv_i32 target = tcg_temp_new_i32();

        tcg_gen_setcondi_i32(TCG_COND_GE, take, cpu_gpr[src], 0);
        if (pred) {
            tcg_gen_and_i32(take, take, pred);
        }
        tcg_gen_movi_i32(target, dc->pce1 + (disp << 2));
        tcg_gen_movcond_i32(TCG_COND_NE, cpu_br_target, take,
                            tcg_constant_i32(0), target, cpu_br_target);
        tcg_gen_movcond_i32(TCG_COND_NE, cpu_br_taken, take,
                            tcg_constant_i32(0), tcg_constant_i32(1),
                            cpu_br_taken);
        if (op->mnem == TIC6X_MNEM_bdec) {
            TCGv_i32 dec = tcg_temp_new_i32();

            tcg_gen_subi_i32(dec, cpu_gpr[src], 1);
            wb_pred(dc, pred, src, dec);
        }
        dc->br_countdown = TIC6X_BRANCH_DELAY;
        dc->br_just_set = true;
        break;
    }

    case TIC6X_MNEM_callp: {
        /*
         * A call that saves its own return address, and - unlike B - has no
         * visible delay slots: the instruction after it is the one that runs
         * on return. So the link register gets the address just past this
         * execute packet and the branch takes effect immediately.
         */
        int link = reg_of(s, field_get(f, TIC6X_FLD_dst, insn));
        int32_t disp = (int32_t)(field_get(f, TIC6X_FLD_cst, insn) << 11) >> 11;
        TCGv_i32 ret = tcg_temp_new_i32();

        tcg_gen_movi_i32(ret, dc->base.pc_next);
        wb_pred(dc, pred, link, ret);
        tcg_gen_movi_i32(cpu_br_target, dc->pce1 + (disp << 2));
        tcg_gen_movi_i32(cpu_br_taken, 1);
        dc->br_countdown = 0;       /* lands at the end of this packet */
        dc->br_just_set = false;
        break;
    }

    case TIC6X_MNEM_spmask:
    case TIC6X_MNEM_spmaskr:
    case TIC6X_MNEM_spkernel:
    case TIC6X_MNEM_spkernelr:
    case TIC6X_MNEM_sploop:
    case TIC6X_MNEM_sploopd:
    case TIC6X_MNEM_sploopw: {
        /*
         * The software-pipelined loop buffer, and the one thing here that is
         * deliberately left unimplemented rather than approximated.
         *
         * SPLOOP does not mean "run this body ILC times". It loads the body
         * into a buffer and replays it with a different set of stages active
         * each iteration, so several iterations are in flight at once and an
         * instruction's neighbours differ from one pass to the next. SPMASK
         * then suppresses individual units within that. Executing the body
         * serially gives the right answer only for loops that happen not to
         * depend on the overlap, and the compiler emits SPLOOP precisely
         * when it does depend on it.
         *
         * A wrong answer here would be wrong audio samples, quietly - the
         * failure mode this model has been careful to avoid everywhere else.
         * So it traps, and doing it properly means modelling the buffer and
         * its stage predicates. 87 instructions in this firmware use it.
         */
        tcg_gen_movi_i32(cpu_pc, dc->insn_pc);
        gen_helper_unimplemented(tcg_env, tcg_constant_i32(insn),
                                 tcg_constant_i32(op->mnem));
        break;
    }

    case TIC6X_MNEM_bnop:
    case TIC6X_MNEM_b: {
        /*
         * The target is recorded, not jumped to: five more execute packets
         * run first. A predicated branch only counts if the predicate held,
         * which is why the flag is a run-time value.
         */
        TCGv_i32 target = tcg_temp_new_i32();

        if (field_present(f, TIC6X_FLD_src2) &&
            !field_present(f, TIC6X_FLD_cst)) {
            tcg_gen_mov_i32(target,
                            cpu_gpr[reg_of(s ^ x,
                                           field_get(f, TIC6X_FLD_src2,
                                                     insn))]);
        } else {
            uint32_t raw = field_get(f, TIC6X_FLD_cst, insn);
            int32_t disp = (int32_t)(raw << 11) >> 11;   /* scst21 */

            tcg_gen_movi_i32(target, dc->pce1 + (disp << 2));
        }
        if (pred) {
            tcg_gen_movcond_i32(TCG_COND_NE, cpu_br_target, pred,
                                tcg_constant_i32(0), target, cpu_br_target);
            tcg_gen_movcond_i32(TCG_COND_NE, cpu_br_taken, pred,
                                tcg_constant_i32(0), tcg_constant_i32(1),
                                cpu_br_taken);
        } else {
            tcg_gen_mov_i32(cpu_br_target, target);
            tcg_gen_movi_i32(cpu_br_taken, 1);
        }
        if (dc->br_countdown >= 0) {
            qemu_log_mask(LOG_UNIMP,
                          "tic6x: a second branch at 0x%08x while one is "
                          "still in flight; only one is tracked\n",
                          dc->packet_pc);
        }
        dc->br_countdown = TIC6X_BRANCH_DELAY;
        dc->br_just_set = true;
        /*
         * BNOP is a branch with its own nop count, which fills some of the
         * delay slots the branch just opened. The count has to be honoured
         * for the same reason NOP n does: the slots are counted in cycles.
         */
        if (op->mnem == TIC6X_MNEM_bnop && field_present(f, TIC6X_FLD_n)) {
            cycles = field_get(f, TIC6X_FLD_n, insn) + 1;
        }
        break;
    }

    default:
        tcg_gen_movi_i32(cpu_pc, dc->insn_pc);
        gen_helper_unimplemented(tcg_env, tcg_constant_i32(insn),
                                 tcg_constant_i32(op->mnem));
        break;
    }

    return cycles;
}

/* ---------------------------------------------------------- the packet */

/*
 * Translate one execute packet, and return how many bytes of instruction
 * stream it consumed and how many cycles it took.
 *
 * A fetch packet is eight words on a 32-byte boundary. If its last word has
 * bits 31-28 set to 1110 it is a header rather than an instruction, and the
 * other seven words hold a mixture of 32-bit opcodes and pairs of 16-bit
 * compact ones, with the p-bits in the header instead of in the opcodes.
 *
 * AN EXECUTE PACKET CAN CROSS A FETCH PACKET BOUNDARY - SPRUFE8B 3.5 says so
 * plainly - and this has to follow it when it does, because the writeback
 * list is what makes the instructions in a packet independent and it is only
 * correct if it is flushed once, at the true end. Stopping at the fetch
 * packet boundary and letting the translator loop start a "new" packet for
 * the remainder flushes it in the middle, so the second half sees results
 * from the first half that the hardware would not have written yet. That is
 * silent whenever the two halves touch different registers and wrong the
 * moment they do not.
 *
 * An execute packet is at most eight instructions, so it can span at most
 * two fetch packets; going further means the walk has lost its place.
 */
static int translate_packet(CPUState *cs, DisasContext *dc, uint32_t pc,
                            int *cycles)
{
    uint32_t fp_base = pc & ~31u;
    int consumed = 0;
    int fetch_packets = 0;
    bool ended = false;

    dc->packet_pc = pc;
    dc->pce1 = fp_base;
    dc->nwb = 0;
    *cycles = 1;

    while (!ended) {
        uint32_t word[8];
        uint32_t header;
        bool have_header;
        int slot = fetch_packets ? 0 : (pc - fp_base) / 4;
        int n;

        if (++fetch_packets > 2) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "tic6x: execute packet at 0x%08x spans more than "
                          "two fetch packets\n", dc->packet_pc);
            break;
        }

        /*
         * Read this fetch packet's remaining words up front, in address
         * order. The translator records the bytes a block was built from and
         * asserts they are read contiguously and forwards, so the header -
         * which is the LAST word - cannot be peeked at before the
         * instructions preceding it. Reading the tail into an array first
         * satisfies that, and a fetch packet is 32 bytes on a 32-byte
         * boundary so it never straddles a page.
         */
        for (n = slot; n < 8; n++) {
            word[n] = translator_ldl_end(cpu_env(cs), &dc->base,
                                         fp_base + n * 4, MO_LE);
        }

        header = word[7];
        have_header = (header >> 28) == 0xE;

        if (have_header) {
            uint32_t layout = (header >> 21) & 0x7f;
            uint32_t expansion = (header >> 14) & 0x7f;
            uint32_t pbits = header & 0x3fff;
            int i;

            /*
             * Where each instruction's p-bit lives, which is not what it
             * looks like and was wrong here in two ways at once.
             *
             * The header's fourteen p-bits are indexed by WORD POSITION, two
             * per word - table 3-16 spells them out as "word N, least
             * significant sixteen bits" and "word N, most significant" - so
             * word i uses bits 2i and 2i+1. Walking a running counter that
             * advances by one for a 32-bit word and two for a compact one
             * gives the same answer only while every preceding word is
             * compact, and a different one as soon as a 32-bit word comes
             * first.
             *
             * And a 32-bit instruction inside a header-based packet keeps
             * its own p-bit in bit 0 of the opcode. The header's p-bits are
             * there for compact instructions, which have no room for one;
             * they do not displace the bit a full-width instruction already
             * carries. tic6x-dis.c is explicit about it - for a non-compact
             * previous word it reads prev_opcode & 1 rather than the header.
             *
             * Getting either of these wrong moves the end of the execute
             * packet, which moves where execution resumes.
             */
            for (i = slot; i < 7 && !ended; i++) {
                uint32_t w = word[i];
                bool compact = layout & (1u << i);
                int halves = compact ? 2 : 1;
                int h;

                for (h = 0; h < halves; h++) {
                    bool parallel;
                    int c;

                    dc->insn_pc = fp_base + i * 4;
                    if (compact) {
                        uint32_t op16 = h ? (w >> 16) : (w & 0xffff);

                        c = trans_one(dc, op16, 16, expansion);
                        parallel = pbits & (1u << (2 * i + h));
                    } else {
                        c = trans_one(dc, w, 32, 0);
                        parallel = w & 1;
                    }
                    if (c > *cycles) {
                        *cycles = c;
                    }
                    if (!parallel) {
                        consumed += (i - slot) * 4 + 4;
                        ended = true;
                        break;
                    }
                }
            }
            if (!ended) {
                /* Ran out of instructions here; the header is not one. */
                consumed += (7 - slot) * 4 + 4;
            }
        } else {
            for (n = slot; n < 8; n++) {
                uint32_t w = word[n];
                int c;

                dc->insn_pc = fp_base + n * 4;
                c = trans_one(dc, w, 32, 0);

                if (c > *cycles) {
                    *cycles = c;
                }
                if (!(w & 1)) {
                    consumed += (n - slot) * 4 + 4;
                    ended = true;
                    break;
                }
            }
            if (!ended) {
                consumed += (8 - slot) * 4;
            }
        }

        /* Carry on into the next fetch packet if the packet did not end. */
        fp_base += 32;
    }
    wb_flush(dc);
    return consumed ? consumed : 4;
}

/* ------------------------------------------------------- translator ops */

static void tic6x_tr_init_disas_context(DisasContextBase *dcbase,
                                        CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    /*
     * Pick the countdown up from where the last block left it. Zero means
     * nothing is in flight; otherwise this block starts part way through
     * some earlier branch's delay slots.
     */
    dc->br_countdown = dc->base.tb->flags ? (int)dc->base.tb->flags : -1;
    dc->br_just_set = false;
    dc->nwb = 0;
}

static void tic6x_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void tic6x_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    tcg_gen_insn_start(dc->base.pc_next, 0, 0);
}

static void tic6x_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);
    uint32_t pc = dc->base.pc_next;
    int cycles = 1;
    int len = translate_packet(cs, dc, pc, &cycles);

    dc->base.pc_next = pc + len;

    if (dc->br_countdown >= 0) {
        if (dc->br_just_set) {
            /* The branch's own packet is not one of its delay slots. */
            dc->br_just_set = false;
        } else {
            dc->br_countdown -= cycles;
        }
        /* Keep env in step, so a block ending here does not lose it. */
        tcg_gen_movi_i32(cpu_br_cnt, dc->br_countdown > 0
                                     ? dc->br_countdown : 0);
        if (dc->br_countdown <= 0) {
            /*
             * The delay slots are done. If the branch was taken, go to its
             * target; if it was predicated off, carry straight on.
             */
            TCGLabel *not_taken = gen_new_label();

            tcg_gen_brcondi_i32(TCG_COND_EQ, cpu_br_taken, 0, not_taken);
            tcg_gen_movi_i32(cpu_br_taken, 0);
            tcg_gen_mov_i32(cpu_pc, cpu_br_target);
            tcg_gen_exit_tb(NULL, 0);
            gen_set_label(not_taken);
            tcg_gen_movi_i32(cpu_br_cnt, 0);
            dc->br_countdown = -1;
            dc->base.is_jmp = DISAS_TOO_MANY;
        }
    }
}

static void tic6x_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    switch (dc->base.is_jmp) {
    case DISAS_TOO_MANY:
    case DISAS_NEXT:
        tcg_gen_movi_i32(cpu_pc, dc->base.pc_next);
        tcg_gen_exit_tb(NULL, 0);
        break;
    default:
        break;
    }
}

static const TranslatorOps tic6x_translator_ops = {
    .init_disas_context = tic6x_tr_init_disas_context,
    .tb_start           = tic6x_tr_tb_start,
    .insn_start         = tic6x_tr_insn_start,
    .translate_insn     = tic6x_tr_translate_insn,
    .tb_stop            = tic6x_tr_tb_stop,
};

void tic6x_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext dc;

    translator_loop(cs, tb, max_insns, pc, host_pc, &tic6x_translator_ops,
                    &dc.base);
}

void tic6x_translate_init(void)
{
    int i;

    for (i = 0; i < TIC6X_NUM_GPR; i++) {
        char name[8];

        snprintf(name, sizeof(name), "%c%d", i < 32 ? 'A' : 'B', i & 31);
        cpu_gpr[i] = tcg_global_mem_new_i32(tcg_env,
                                            offsetof(CPUTIC6XState, gpr[i]),
                                            g_strdup(name));
    }
    cpu_pc = tcg_global_mem_new_i32(tcg_env,
                                    offsetof(CPUTIC6XState, pc), "pc");
    cpu_br_target = tcg_global_mem_new_i32(tcg_env,
                                           offsetof(CPUTIC6XState, br_target),
                                           "br_target");
    cpu_br_taken = tcg_global_mem_new_i32(tcg_env,
                                          offsetof(CPUTIC6XState, br_taken),
                                          "br_taken");
    cpu_br_cnt = tcg_global_mem_new_i32(tcg_env,
                                        offsetof(CPUTIC6XState, br_cnt),
                                        "br_cnt");
}
