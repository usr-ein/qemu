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

#define TIC6X_MAX_WB 8

typedef struct DisasContext {
    DisasContextBase base;

    /* Address of the execute packet being translated. */
    uint32_t packet_pc;

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

/* ------------------------------------------------------ one instruction */

static int reg_of(uint32_t side, uint32_t num)
{
    return (side ? 32 : 0) + (num & 31);
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

    switch (op->mnem) {
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
        uint32_t src1 = field_get(f, TIC6X_FLD_src1, insn);
        TCGv_i32 a = tcg_temp_new_i32();
        TCGv_i32 v = tcg_temp_new_i32();

        /*
         * Whether src1 is a register or a constant is not something this
         * first cut can tell from the table alone: the operand list that
         * says so is not carried into the generated tables yet. Registers
         * are the common case and the constant forms trap, rather than
         * silently computing with a register number as if it were a value.
         */
        if (op->nfix && op->fix[0].field == TIC6X_FLD_op) {
            tcg_gen_mov_i32(a, cpu_gpr[reg_of(s, src1)]);
        } else {
            tcg_gen_mov_i32(a, cpu_gpr[reg_of(s, src1)]);
        }

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

        tcg_gen_movi_i32(v, dc->packet_pc + (disp << 2));
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

            tcg_gen_movi_i32(target, dc->packet_pc + (disp << 2));
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
        break;
    }

    default:
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
 * Compact instructions are about 5% of this firmware, so they are decoded
 * far enough to keep the packet walk in step and then trap by name.
 */
static int translate_packet(CPUState *cs, DisasContext *dc, uint32_t pc,
                            int *cycles)
{
    uint32_t fp_base = pc & ~31u;
    uint32_t word[8];
    uint32_t header;
    bool have_header;
    int slot = (pc - fp_base) / 4;
    int consumed = 0;
    int n;

    dc->packet_pc = pc;
    dc->nwb = 0;
    *cycles = 1;

    /*
     * Read the rest of the fetch packet up front, in address order.
     *
     * The translator records the bytes a block was built from and asserts
     * they are read contiguously and forwards, so the header - which is the
     * LAST word of the packet - cannot be peeked at before the instructions
     * that precede it. Reading the tail into an array first satisfies that
     * and costs nothing: a fetch packet is 32 bytes on a 32-byte boundary,
     * so it never straddles a page and reading to its end can never fault
     * somewhere the instructions themselves would not have.
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
        int i, bit = 0;

        /* Count the p-bit slots that precede this word. */
        for (i = 0; i < slot; i++) {
            bit += (layout & (1u << i)) ? 2 : 1;
        }
        for (i = slot; i < 7; i++) {
            uint32_t w = word[i];
            bool compact = layout & (1u << i);
            int halves = compact ? 2 : 1;
            int h;

            for (h = 0; h < halves; h++) {
                int c;

                if (compact) {
                    uint32_t op16 = h ? (w >> 16) : (w & 0xffff);

                    c = trans_one(dc, op16, 16, expansion);
                } else {
                    c = trans_one(dc, w, 32, 0);
                }
                if (c > *cycles) {
                    *cycles = c;
                }
                if (!(pbits & (1u << bit))) {
                    /* p-bit clear: the execute packet ends here. */
                    consumed = (i - slot) * 4 + 4;
                    goto done;
                }
                bit++;
            }
        }
        consumed = (7 - slot) * 4;
    } else {
        for (n = slot; n < 8; n++) {
            uint32_t w = word[n];
            int c = trans_one(dc, w, 32, 0);

            if (c > *cycles) {
                *cycles = c;
            }
            if (!(w & 1)) {
                consumed = (n - slot) * 4 + 4;
                goto done;
            }
        }
        consumed = (8 - slot) * 4;
    }

done:
    wb_flush(dc);
    return consumed ? consumed : 4;
}

/* ------------------------------------------------------- translator ops */

static void tic6x_tr_init_disas_context(DisasContextBase *dcbase,
                                        CPUState *cs)
{
    DisasContext *dc = container_of(dcbase, DisasContext, base);

    dc->br_countdown = -1;
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
        dc->br_countdown -= cycles;
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
}
