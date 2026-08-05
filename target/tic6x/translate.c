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
 * SPLOOP is the fourth thing, and it is why the packet walk is split into a
 * scan and an emit: a software-pipelined loop has to be generated as a
 * whole, from a body the scanner reads ahead. See gen_sploop().
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

/*
 * Results landing in one cycle. Eight functional units is the architectural
 * limit for one execute packet, but a cycle inside an SPLOOP holds several
 * stages at once, plus the address writebacks of their .D units and the
 * loads arriving from four cycles earlier.
 */
#define TIC6X_MAX_WB 32

/* An execute packet is eight instructions, but a compact one may be split
   across a header packet's fourteen slots; this is room to spare. */
#define TIC6X_MAX_PACKET 16

/*
 * A load's data lands four cycles after it issues - SPRUFE8B table 3-8,
 * "Load: delay slots 4, write cycles i, i + 4", the i write being the
 * address post-increment on a separate port.
 *
 * Everywhere else in this model a load writes its destination immediately,
 * which is harmless because compiled code respects the delay slots. Inside
 * an SPLOOP it is not harmless at all: the body is one iteration and the
 * same register serves every pass, so with ii = 2 the value the store at
 * body cycle 5 wants is the one loaded by ITS OWN iteration five cycles
 * back, not the one issued a cycle ago by the iteration two ahead. Writing
 * early scrambles the copy and says nothing about it.
 */
#define SP_LOAD_DELAY 4

/* The loop buffer holds a body of at most 48 cycles - SPRUFE8B 7.7.3.3. */
#define SP_MAX_CYCLES 48
#define SP_MAX_STAGES (SP_MAX_CYCLES + SP_LOAD_DELAY)

/* Two .D units per cycle, and a doubleword load writes a register pair. */
#define SP_LOADS_PER_CYCLE 4

/* One instruction, decoded. */
typedef struct {
    uint32_t insn;
    uint32_t pc;
    int bits;
    const TIC6XOpcode *op;
    const TIC6XFormat *fmt;
} DisasInsn;

/* One execute packet, decoded but not yet translated. */
typedef struct {
    uint32_t addr;              /* where it starts                        */
    uint32_t pce1;              /* the fetch packet it starts in          */
    uint32_t next;              /* the packet after it                    */
    int cycles;                 /* what it costs, NOP n included          */
    int n;
    bool present;
    DisasInsn ins[TIC6X_MAX_PACKET];
} DisasPacket;

/*
 * A load in an SPLOOP body: the value waits here for the four cycles it
 * takes to arrive. slot 0 is written when the load issues and the slots
 * shift along once per pass, so the commit reads slot[depth] - the value
 * from the pass that issued it, however many passes ago that was.
 */
typedef struct {
    bool present;
    int reg;                    /* what it writes                         */
    int depth;                  /* passes between issuing and landing     */
    TCGv_i32 val[SP_LOAD_DELAY + 1];
    TCGv_i32 valid[SP_LOAD_DELAY + 1];
} SPLoad;

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

    /*
     * An SPLOOP seen in the packet being translated. It cannot act where it
     * is decoded: its own execute packet has to finish first, because
     * SPLOOPD's iteration count arrives from an MVC in parallel with it and
     * the loop reads that count afterwards. So the decode records it and
     * the packet walk generates the loop once the writebacks are done.
     */
    struct {
        bool active;
        uint16_t mnem;
        uint32_t insn;
        int ii;
        uint32_t creg, z;
    } sp_start;

    /* State while the body of an SPLOOP is being emitted. */
    struct {
        bool active;
        TCGv_i32 gate;          /* is this stage running an iteration?    */
        int bc;                 /* the body cycle being emitted           */
        int nld;                /* loads emitted so far in that cycle     */
        SPLoad *ld;             /* SP_LOADS_PER_CYCLE entries per cycle   */
    } sp;
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

/* Which register each creg value names; -1 for none and for the reserved 7. */
static const int creg_reg[8] = {
    -1,
    TIC6X_REG_B(0), TIC6X_REG_B(1), TIC6X_REG_B(2),
    TIC6X_REG_A(1), TIC6X_REG_A(2), TIC6X_REG_A(0),
    -1,
};

/*
 * creg selects the predicate register and z the sense. Value 0 means the
 * instruction is unconditional, and 7 is reserved - the decoder rejects it,
 * so anything reaching here is one of the six real ones.
 *
 * Inside an SPLOOP body there is a second condition: whether the stage this
 * instruction belongs to is running an iteration at all. It is one more
 * term on the predicate the instruction already has, which is the whole
 * reason the loop can be generated as ordinary translated code.
 */
static TCGv_i32 insn_pred(DisasContext *dc, uint32_t creg, uint32_t z)
{
    TCGv_i32 pred = NULL;

    if (creg != 0 && creg_reg[creg] >= 0) {
        pred = tcg_temp_new_i32();
        tcg_gen_setcondi_i32(z ? TCG_COND_EQ : TCG_COND_NE, pred,
                             cpu_gpr[creg_reg[creg]], 0);
    }
    if (dc->sp.gate) {
        if (!pred) {
            /* Read only from here on; nothing modifies a predicate. */
            return dc->sp.gate;
        }
        tcg_gen_and_i32(pred, pred, dc->sp.gate);
    }
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

/*
 * The register a doubleword load or store names.
 *
 * Which field holds it, and how, is not the same across the forms. The
 * aligned LDDW puts the even register number straight into srcdst; the
 * nonaligned LDNDW puts the PAIR number - the register number halved, which
 * is binutils' reg_shift - into dst, and STNDW into src. Reading srcdst
 * unconditionally finds no such field in the nonaligned forms, so every one
 * of them loads or stores A1:A0 and says nothing about it.
 */
static int dword_reg(const TIC6XFormat *f, const TIC6XOpcode *op,
                     uint32_t insn, uint32_t side, bool high)
{
    static const TIC6XField cand[] = {
        TIC6X_FLD_srcdst, TIC6X_FLD_dst, TIC6X_FLD_src,
    };
    uint32_t num = 0;
    int i;

    for (i = 0; i < ARRAY_SIZE(cand); i++) {
        uint8_t enc = op->enc[cand[i]];

        if (!field_present(f, cand[i]) ||
            (enc != TIC6X_ENC_REG && enc != TIC6X_ENC_REG_SHIFT)) {
            continue;
        }
        num = field_get(f, cand[i], insn);
        if (enc == TIC6X_ENC_REG_SHIFT) {
            num <<= 1;
        }
        break;
    }
    /* The pair is written A5:A4 and the even register holds the low half. */
    return reg_of(side, high ? ((num & ~1u) | 1u) : (num & ~1u));
}

/*
 * Whether addr_mode() can read this encoding at all.
 *
 * The compact memory instructions are a different addressing family: they
 * name their pointer in ptr rather than baseR, their offset in cst, their
 * register file in t, and they carry the mode in the fetch packet header's
 * expansion bits rather than in a mode field. addr_mode() reads none of
 * those, so for a compact form every field it wants is absent and it builds
 * an address out of zeroes - a load from 0 that this machine answers with a
 * quiet nothing. There are 286 of them in this image; they need their own
 * decode, and until then they say so rather than pretend.
 */
static bool addr_mode_known(const TIC6XFormat *f)
{
    return field_present(f, TIC6X_FLD_mode) &&
           field_present(f, TIC6X_FLD_baseR);
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

/*
 * The next delay-line slot for a load in an SPLOOP body. The slots were
 * allocated by a scan of the same packets in the same order, so the nth
 * load emitted from a body cycle is the nth the scan found there.
 */
static SPLoad *sp_load_slot(DisasContext *dc, int reg)
{
    SPLoad *l;

    if (dc->sp.nld >= SP_LOADS_PER_CYCLE) {
        return NULL;
    }
    l = &dc->sp.ld[dc->sp.bc * SP_LOADS_PER_CYCLE + dc->sp.nld++];
    if (!l->present || l->reg != reg) {
        return NULL;
    }
    return l;
}

static void gen_load(DisasContext *dc, TCGv_i32 pred, int dst,
                     TCGv_i32 addr, MemOp op)
{
    TCGv_i32 val;
    TCGLabel *skip = NULL;

    if (dc->sp.active) {
        /*
         * In a loop body the value does not go to the register here. It is
         * four cycles in the air; gen_sploop() lands it at the cycle where
         * the architecture says it becomes readable.
         */
        SPLoad *l = sp_load_slot(dc, dst);

        if (l) {
            tcg_gen_movi_i32(l->valid[0], 0);
            if (pred) {
                skip = gen_new_label();
                tcg_gen_brcondi_i32(TCG_COND_EQ, pred, 0, skip);
            }
            tcg_gen_qemu_ld_i32(l->val[0], addr, 0, op);
            tcg_gen_movi_i32(l->valid[0], 1);
            if (skip) {
                gen_set_label(skip);
            }
            return;
        }
        /* The scan and the emit disagree, which they cannot; say so rather
           than write to the wrong register. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tic6x: load at 0x%08x has no delay slot in the "
                      "SPLOOP body\n", dc->insn_pc);
    }

    val = tcg_temp_new_i32();
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
 * How many cycles an instruction occupies. One for everything except the
 * multicycle NOPs, and that matters twice over: those cycles are what a
 * branch's delay slots are counted in, and what an SPLOOP body's stage
 * boundaries are measured in.
 *
 * The count lives in a different field in each encoding and is stored one
 * less than it means - binutils' ucst_minus_one. Reading a field the format
 * does not have, which is what this used to do, makes every NOP n a NOP 1.
 */
static int insn_cycles(const TIC6XOpcode *op, const TIC6XFormat *f,
                       uint32_t insn)
{
    switch (op->mnem) {
    case TIC6X_MNEM_nop:
        /* op in the 32-bit form, n in the compact one. */
        if (field_present(f, TIC6X_FLD_op)) {
            return field_get(f, TIC6X_FLD_op, insn) + 1;
        }
        if (field_present(f, TIC6X_FLD_n)) {
            return field_get(f, TIC6X_FLD_n, insn) + 1;
        }
        return 1;

    case TIC6X_MNEM_addkpc:
        /* dst = return address, then src2 cycles of nop. */
        return field_present(f, TIC6X_FLD_src2)
               ? field_get(f, TIC6X_FLD_src2, insn) + 1 : 1;

    case TIC6X_MNEM_bnop:
        /* A branch carrying the nops that fill its own delay slots. */
        return field_present(f, TIC6X_FLD_n)
               ? field_get(f, TIC6X_FLD_n, insn) + 1 : 1;

    default:
        return 1;
    }
}

/* Translate one instruction of an execute packet. */
static void trans_one(DisasContext *dc, const DisasInsn *di)
{
    uint32_t insn = di->insn;
    const TIC6XFormat *f = di->fmt;
    const TIC6XOpcode *op = di->op;
    TCGv_i32 pred;
    uint32_t creg = 0, z = 0, s, x;

    if (!op) {
        tcg_gen_movi_i32(cpu_pc, dc->insn_pc);
        gen_helper_illegal(tcg_env, tcg_constant_i32(insn));
        return;
    }

    if (field_present(f, TIC6X_FLD_creg)) {
        creg = field_get(f, TIC6X_FLD_creg, insn);
        z = field_get(f, TIC6X_FLD_z, insn);
    }
    pred = insn_pred(dc, creg, z);
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
        return;
    }
    if (routed_alu2_wide(op->mnem)) {
        uint32_t dstn = field_get(f, TIC6X_FLD_dst, insn);
        int src2 = reg_of(s ^ x, field_get(f, TIC6X_FLD_src2, insn));
        TCGv_i32 a = src1_value(f, op, insn, s);
        TCGv_i64 v = tcg_temp_new_i64();

        gen_helper_alu2_wide(v, tcg_env, tcg_constant_i32(op->mnem), a,
                             cpu_gpr[src2]);
        pair_set(dc, pred, s, dstn, v);
        return;
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

    case TIC6X_MNEM_nop:
        /* Its only effect is the cycles it eats, counted by insn_cycles(). */
        break;

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
        TCGv_i32 addr;

        if (!addr_mode_known(f)) {
            goto unimplemented;
        }
        addr = addr_mode(dc, pred, f, insn, scale);
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
        TCGv_i32 addr;

        if (!addr_mode_known(f)) {
            goto unimplemented;
        }
        addr = addr_mode(dc, pred, f, insn, scale);
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
        TCGv_i32 v = tcg_temp_new_i32();

        tcg_gen_movi_i32(v, dc->pce1 + (disp << 2));
        wb_pred(dc, pred, dst, v);
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
        int lo = dword_reg(f, op, insn, side, false);
        int hi = dword_reg(f, op, insn, side, true);
        /*
         * The nonaligned forms carry an sc bit saying whether the offset is
         * scaled by the access size or taken as bytes - SPRUFE8B's LDNDW
         * page, "if sc is 0 the offsetR/ucst5 is not shifted". The aligned
         * ones always scale.
         */
        int scale = field_present(f, TIC6X_FLD_sc)
                    ? (field_get(f, TIC6X_FLD_sc, insn) ? 3 : 0) : 3;
        TCGv_i32 addr, hiaddr;

        if (!addr_mode_known(f)) {
            goto unimplemented;
        }
        addr = addr_mode(dc, pred, f, insn, scale);
        hiaddr = tcg_temp_new_i32();
        tcg_gen_addi_i32(hiaddr, addr, 4);
        /* Two word accesses rather than one inline pair, so that a load in
           an SPLOOP body reaches its delay line like any other. */
        if (is_load) {
            gen_load(dc, pred, lo, addr, MO_LEUL);
            gen_load(dc, pred, hi, hiaddr, MO_LEUL);
        } else {
            gen_store(dc, pred, lo, addr, MO_LEUL);
            gen_store(dc, pred, hi, hiaddr, MO_LEUL);
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

        tcg_gen_addi_i32(hiaddr, cpu_gpr[base], 4);
        gen_load(dc, pred, lo, cpu_gpr[base], MO_LEUL);
        gen_load(dc, pred, hi, hiaddr, MO_LEUL);
        break;
    }

    case TIC6X_MNEM_ldnw:
    case TIC6X_MNEM_stnw: {
        /* Nonaligned word; the same as ldw and stw to a model that does not
           fault on alignment. */
        int reg = reg_of(field_get(f, TIC6X_FLD_s, insn),
                         field_get(f, TIC6X_FLD_srcdst, insn));
        TCGv_i32 addr;

        if (!addr_mode_known(f)) {
            goto unimplemented;
        }
        addr = addr_mode(dc, pred, f, insn, 2);
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

    case TIC6X_MNEM_sploop:
    case TIC6X_MNEM_sploopd:
    case TIC6X_MNEM_sploopw: {
        /*
         * Record it and let the packet walk act on it. The loop cannot be
         * generated here: SPLOOPD takes its iteration count from an MVC in
         * parallel with it, so the count is only in ILC once this execute
         * packet's writebacks have landed. See gen_sploop().
         */
        uint32_t ii_field = field_present(f, TIC6X_FLD_cstb)
                            ? TIC6X_FLD_cstb : TIC6X_FLD_ii;

        dc->sp_start.active = true;
        dc->sp_start.mnem = op->mnem;
        dc->sp_start.insn = insn;
        /* ii is encoded one less than it means. */
        dc->sp_start.ii = field_get(f, ii_field, insn) + 1;
        dc->sp_start.creg = creg;
        dc->sp_start.z = z;
        break;
    }

    case TIC6X_MNEM_spmask:
    case TIC6X_MNEM_spmaskr:
    case TIC6X_MNEM_spkernel:
    case TIC6X_MNEM_spkernelr:
        /*
         * These say things about the loop buffer rather than doing anything
         * to the register file, and gen_sploop() has already read what they
         * say. Outside a loop body SPRUFE8B has them do nothing.
         */
        break;

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
        break;
    }

    default:
    unimplemented:
        tcg_gen_movi_i32(cpu_pc, dc->insn_pc);
        gen_helper_unimplemented(tcg_env, tcg_constant_i32(insn),
                                 tcg_constant_i32(op->mnem));
        break;
    }
}

/* ---------------------------------------------------------- the packet */

/* Decode one instruction into a packet, and let it claim its cycles. */
static void scan_one(DisasPacket *pk, uint32_t insn, int bits,
                     uint32_t expansion, uint32_t pc)
{
    DisasInsn *di;

    if (pk->n >= TIC6X_MAX_PACKET) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tic6x: more than %d instructions in the execute "
                      "packet at 0x%08x\n", TIC6X_MAX_PACKET, pk->addr);
        return;
    }
    di = &pk->ins[pk->n++];
    di->insn = insn;
    di->bits = bits;
    di->pc = pc;
    di->op = decode(insn, bits, expansion, &di->fmt);
    if (di->op) {
        int c = insn_cycles(di->op, di->fmt, insn);

        if (c > pk->cycles) {
            pk->cycles = c;
        }
    }
}

/*
 * Read one execute packet and decode it, without emitting anything.
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
 *
 * Reading and translating are separate because an SPLOOP body has to be read
 * ahead in full before any of it can be generated, and doing that with a
 * second copy of this walk would mean two chances to get the p-bits wrong.
 */
static void scan_packet(CPUState *cs, DisasContext *dc, uint32_t pc,
                        DisasPacket *pk)
{
    uint32_t fp_base = pc & ~31u;
    int consumed = 0;
    int fetch_packets = 0;
    bool ended = false;

    memset(pk, 0, sizeof(*pk));
    pk->addr = pc;
    pk->pce1 = fp_base;
    pk->cycles = 1;
    pk->present = true;

    while (!ended) {
        uint32_t word[8];
        uint32_t header;
        bool have_header;
        int slot = fetch_packets ? 0 : (pc - fp_base) / 4;
        int n;

        if (++fetch_packets > 2) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "tic6x: execute packet at 0x%08x spans more than "
                          "two fetch packets\n", pk->addr);
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

                    if (compact) {
                        uint32_t op16 = h ? (w >> 16) : (w & 0xffff);

                        scan_one(pk, op16, 16, expansion, fp_base + i * 4);
                        parallel = pbits & (1u << (2 * i + h));
                    } else {
                        scan_one(pk, w, 32, 0, fp_base + i * 4);
                        parallel = w & 1;
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

                scan_one(pk, w, 32, 0, fp_base + n * 4);
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
    pk->next = pc + (consumed ? consumed : 4);
}

/* Translate every instruction of a scanned packet. The caller flushes. */
static void emit_packet(DisasContext *dc, const DisasPacket *pk)
{
    int i;

    dc->packet_pc = pk->addr;
    dc->pce1 = pk->pce1;
    for (i = 0; i < pk->n; i++) {
        dc->insn_pc = pk->ins[i].pc;
        trans_one(dc, &pk->ins[i]);
    }
}

/* ---------------------------------------------------------- SPLOOP */

/*
 * The software-pipelined loop buffer, SPRUFE8B chapter 7.
 *
 * SPLOOP does not mean "run this body ILC times". The body is one iteration
 * of a modulo-scheduled loop, divided into stages of ii cycles; iteration j
 * starts ii cycles after iteration j-1, so several iterations are in flight
 * at once and what executes alongside an instruction differs from one pass
 * to the next. Running the body serially gives the steady state with no
 * prolog and no epilog, and the compiler emits SPLOOP precisely when the
 * overlap is what makes the loop correct.
 *
 * Stated as a schedule it is simple enough to generate directly. At absolute
 * cycle t, the instructions that run are body cycle t - j*ii for every
 * iteration j that has started and not finished. Group the cycles into
 * passes of ii and that becomes: at pass p, offset c, stage k runs body
 * cycle k*ii + c on behalf of iteration p - k. So one pass of generated code
 * covers every stage, each gated on whether its iteration exists - which is
 * one more term on the predicate every instruction already has. The pass
 * repeats through a backward branch, and the whole loop is one translation
 * block.
 *
 * Two things make that gating insufficient on its own.
 *
 * A load's data lands four cycles after it issues. The body reuses one
 * register per iteration and only works because the value is consumed before
 * the next one lands, so the four cycles have to be modelled: see SPLoad.
 *
 * SPMASK marks instructions that are executed but not loaded into the
 * buffer, which is how setup code is overlaid on the loop's first stage.
 * Those run for iteration 0 only - that is, at pass p == k - and they
 * suppress anything from an older stage on the same unit in that cycle.
 *
 * What is not modelled, and would need to be if a firmware used it:
 *
 *   - reload, SPKERNELR and SPMASKR. Nothing in this image reloads.
 *   - interrupts during a loop. Hardware pipes the loop down, takes the
 *     interrupt, and pipes it back up; here the loop runs to completion
 *     first. The DSP has no interrupts wired up yet either way.
 *   - the SPKERNEL fstg/fcyc delay, which overlaps post-loop code with the
 *     epilog. Here the epilog finishes first. That is safe in the direction
 *     it errs: post-loop reads see final values rather than stale ones, and
 *     post-loop writes land after the epilog's reads rather than before.
 */

/* How the eight bits of an SPMASK unit mask are ordered: L1 L2 S1 S2 D1 D2
   M1 M2, SPRUFE8B figure H-8. The compact form has only the first six. */
static bool sp_unit_masked(const DisasInsn *di, uint32_t mask)
{
    static const int8_t first[TIC6X_UNIT_COUNT] = {
        [TIC6X_UNIT_l] = 0, [TIC6X_UNIT_s] = 2,
        [TIC6X_UNIT_d] = 4, [TIC6X_UNIT_m] = 6,
        [TIC6X_UNIT_nfu] = -1,
    };
    int base;
    uint32_t side;

    if (!di->op || di->op->unit >= TIC6X_UNIT_COUNT) {
        return false;
    }
    base = first[di->op->unit];
    if (base < 0) {
        return false;
    }
    /*
     * Which field says unit 1 or unit 2 is not always s. A load names its
     * unit in y and its register file in s, so LDW .D1T2 has y = 0 and
     * s = 1; masking on s would mask the wrong .D unit. The generator
     * carries binutils' ENC(field, fu, ...) through for exactly this.
     */
    if (di->op->unit_field >= TIC6X_FLD_COUNT ||
        !field_present(di->fmt, di->op->unit_field)) {
        return false;
    }
    side = field_get(di->fmt, di->op->unit_field, di->insn) & 1;
    return (mask >> (base + side)) & 1;
}

/* The registers a load writes, in the order gen_load() will be called for
   them. Zero for anything that is not a load. */
static int sp_load_regs(const DisasInsn *di, int *regs)
{
    const TIC6XFormat *f = di->fmt;
    uint32_t insn = di->insn;
    uint32_t side;

    if (!di->op) {
        return 0;
    }
    switch (di->op->mnem) {
    case TIC6X_MNEM_ldw:
    case TIC6X_MNEM_ldh:
    case TIC6X_MNEM_ldhu:
    case TIC6X_MNEM_ldb:
    case TIC6X_MNEM_ldbu:
    case TIC6X_MNEM_ldnw:
        regs[0] = reg_of(field_get(f, TIC6X_FLD_s, insn),
                         field_get(f, TIC6X_FLD_srcdst, insn));
        return 1;

    case TIC6X_MNEM_lddw:
    case TIC6X_MNEM_ldndw:
    case TIC6X_MNEM_cmtl:
        side = field_present(f, TIC6X_FLD_s)
               ? field_get(f, TIC6X_FLD_s, insn) : 0;
        regs[0] = dword_reg(f, di->op, insn, side, false);
        regs[1] = dword_reg(f, di->op, insn, side, true);
        return 2;

    default:
        return 0;
    }
}

/*
 * Things a loop body may not contain, because the machinery around them
 * assumes it is being driven one packet at a time. A branch would arm the
 * delay-slot countdown from inside a loop that the countdown knows nothing
 * about; a nested SPLOOP needs the second loop buffer this does not have.
 * SPRUFE8B forbids most of these in a body anyway unless they are SPMASKed.
 */
static bool sp_body_forbids(uint16_t mnem)
{
    switch (mnem) {
    case TIC6X_MNEM_b:
    case TIC6X_MNEM_bnop:
    case TIC6X_MNEM_bdec:
    case TIC6X_MNEM_bpos:
    case TIC6X_MNEM_callp:
    case TIC6X_MNEM_addkpc:
    case TIC6X_MNEM_sploop:
    case TIC6X_MNEM_sploopd:
    case TIC6X_MNEM_sploopw:
    case TIC6X_MNEM_spkernelr:
    case TIC6X_MNEM_spmaskr:
        return true;
    default:
        return false;
    }
}

/* A gate that also requires the pass number to match, or not to. */
static TCGv_i32 sp_gate_pass(TCGv_i32 live, TCGv_i32 pass, int k, TCGCond c)
{
    TCGv_i32 g = tcg_temp_new_i32();

    tcg_gen_setcondi_i32(c, g, pass, k);
    tcg_gen_and_i32(g, g, live);
    return g;
}

/* Stop before generating a loop that would be wrong, naming the reason. */
static uint32_t sp_refuse(DisasContext *dc, uint32_t resume, uint32_t insn,
                          uint16_t mnem)
{
    tcg_gen_movi_i32(cpu_pc, dc->packet_pc);
    gen_helper_unimplemented(tcg_env, tcg_constant_i32(insn),
                             tcg_constant_i32(mnem));
    return resume;
}

/*
 * Generate the whole loop, and return the address execution resumes at -
 * the execute packet after the one holding SPKERNEL.
 */
static uint32_t gen_sploop(CPUState *cs, DisasContext *dc, uint32_t body_pc)
{
    const int ii = dc->sp_start.ii;
    const uint16_t kind = dc->sp_start.mnem;
    const uint32_t sp_insn = dc->sp_start.insn;
    DisasPacket *body = NULL;
    SPLoad *loads = NULL;
    uint8_t *umask = NULL;
    TCGv_i32 live[SP_MAX_STAGES];
    TCGv_i32 wcond[SP_LOAD_DELAY + 1];
    TCGv_i32 pass, go, any, iters = NULL;
    TCGLabel *loop_top, *loop_done, *runaway;
    int dynlen = 0, span, nstages, wdepth = 0, wsample = 0, wskip;
    int bc, c, d, i, k, n;
    uint32_t pc = body_pc, resume = body_pc;
    bool found_kernel = false;

    if (ii < 1 || ii > 14) {
        return sp_refuse(dc, body_pc, sp_insn, kind);
    }
    if (kind == TIC6X_MNEM_sploopw && dc->sp_start.creg == 0) {
        /* SPLOOPW takes its exit condition from its own predicate; without
           one there is nothing to end the loop. SPRUFE8B 7.5.1.3. */
        return sp_refuse(dc, body_pc, sp_insn, kind);
    }
    if (dc->br_countdown >= 0) {
        qemu_log_mask(LOG_UNIMP,
                      "tic6x: SPLOOP at 0x%08x inside a branch's delay "
                      "slots; the loop's cycles are not counted\n",
                      dc->packet_pc);
    }

    body = g_new0(DisasPacket, SP_MAX_CYCLES);
    umask = g_new0(uint8_t, SP_MAX_CYCLES);
    loads = g_new0(SPLoad, SP_MAX_CYCLES * SP_LOADS_PER_CYCLE);

    /*
     * Read the body: every execute packet from here to the one holding
     * SPKERNEL, indexed by the cycle it starts on. dynlen counts execute
     * packets and NOP cycles alike, starting with the cycle after the
     * SPLOOP - SPRUFE8B 7.7.
     */
    while (dynlen < SP_MAX_CYCLES) {
        DisasPacket *b = &body[dynlen];

        scan_packet(cs, dc, pc, b);
        pc = b->next;
        for (i = 0; i < b->n; i++) {
            const DisasInsn *di = &b->ins[i];

            if (!di->op) {
                continue;
            }
            if (di->op->mnem == TIC6X_MNEM_spkernel ||
                di->op->mnem == TIC6X_MNEM_spkernelr) {
                found_kernel = true;
            } else if (di->op->mnem == TIC6X_MNEM_spmask ||
                       di->op->mnem == TIC6X_MNEM_spmaskr) {
                umask[dynlen] |= field_present(di->fmt, TIC6X_FLD_mask)
                                 ? field_get(di->fmt, TIC6X_FLD_mask, di->insn)
                                 : 0;
            } else if (sp_body_forbids(di->op->mnem)) {
                resume = sp_refuse(dc, pc, di->insn, di->op->mnem);
                goto out;
            }
        }
        dynlen += b->cycles;
        if (found_kernel) {
            break;
        }
    }
    resume = pc;
    if (!found_kernel) {
        /* Forty-eight cycles with no SPKERNEL is not a loop body. */
        resume = sp_refuse(dc, pc, sp_insn, kind);
        goto out;
    }

    /*
     * Loads issued in the last stages land after the body has ended, so the
     * pipeline has to run on past dynlen far enough for them to arrive.
     */
    span = dynlen;
    for (bc = 0; bc < dynlen; bc++) {
        if (!body[bc].present) {
            continue;
        }
        n = 0;
        for (i = 0; i < body[bc].n; i++) {
            int regs[2];
            int nr = sp_load_regs(&body[bc].ins[i], regs);

            for (d = 0; d < nr; d++) {
                SPLoad *l;

                if (n >= SP_LOADS_PER_CYCLE) {
                    resume = sp_refuse(dc, resume, body[bc].ins[i].insn,
                                       body[bc].ins[i].op->mnem);
                    goto out;
                }
                l = &loads[bc * SP_LOADS_PER_CYCLE + n++];
                l->present = true;
                l->reg = regs[d];
                l->depth = (bc + SP_LOAD_DELAY) / ii - bc / ii;
            }
            if (nr && bc + SP_LOAD_DELAY + 1 > span) {
                span = bc + SP_LOAD_DELAY + 1;
            }
        }
    }
    nstages = (span + ii - 1) / ii;
    if (nstages > SP_MAX_STAGES) {
        resume = sp_refuse(dc, resume, sp_insn, kind);
        goto out;
    }

    /* ---- the loop's state, all of it temporaries of this block ---- */

    for (bc = 0; bc < dynlen; bc++) {
        for (n = 0; n < SP_LOADS_PER_CYCLE; n++) {
            SPLoad *l = &loads[bc * SP_LOADS_PER_CYCLE + n];

            if (!l->present) {
                continue;
            }
            for (d = 0; d <= l->depth; d++) {
                l->val[d] = tcg_temp_new_i32();
                l->valid[d] = tcg_temp_new_i32();
                tcg_gen_movi_i32(l->val[d], 0);
                tcg_gen_movi_i32(l->valid[d], 0);
            }
        }
    }

    pass = tcg_temp_new_i32();
    go = tcg_temp_new_i32();
    any = tcg_temp_new_i32();
    tcg_gen_movi_i32(pass, 0);
    for (k = 0; k < nstages; k++) {
        live[k] = tcg_temp_new_i32();
        tcg_gen_movi_i32(live[k], 0);
    }

    /*
     * How many iterations, and when a new one stops starting.
     *
     * SPLOOP tests ILC at the instruction itself and at every stage
     * boundary, terminating when it reaches zero and decrementing otherwise,
     * which comes to exactly ILC iterations - zero of them if ILC is zero
     * (SPRUFE8B 7.9.1, 7.9.2).
     *
     * SPLOOPD skips both the initial test and every stage boundary in the
     * first three cycles, so it runs 1 + 3/ii iterations before ILC is
     * consulted at all. That reproduces table 7-4's minimum counts - four
     * for ii = 1, two for ii = 2 and 3, one above that - and the bias the
     * assembler applies when loading ILC.
     *
     * SPLOOPW has no count: it runs while its own predicate holds, sampled
     * four cycles before each stage boundary (7.10.2), with the same first
     * three cycles exempt.
     */
    wskip = 3 / ii;
    switch (kind) {
    case TIC6X_MNEM_sploop:
    case TIC6X_MNEM_sploopd:
        iters = tcg_temp_new_i32();
        tcg_gen_ld_i32(iters, tcg_env,
                       offsetof(CPUTIC6XState, cr[TIC6X_CR_ILC]));
        if (kind == TIC6X_MNEM_sploopd) {
            tcg_gen_addi_i32(iters, iters, 1 + wskip);
        }
        tcg_gen_setcond_i32(TCG_COND_LTU, go, pass, iters);
        break;

    default:                    /* sploopw */
        /*
         * The condition is read four cycles before the stage boundary that
         * acts on it, so it is sampled at whichever offset of an earlier
         * pass is four cycles back: wdepth passes and wsample cycles into
         * one. With ii >= 4 that is the pass just gone; with ii = 1 it is
         * four passes ago, which is why this is a shift register at all.
         *
         * Sampling at offset wsample of pass q lands in wcond[0], so at the
         * end of pass q - deciding the boundary before pass q + 1 at cycle
         * (q + 1) * ii - the wanted sample is the one from pass
         * q + 1 - wdepth, which is wcond[wdepth - 1].
         */
        wdepth = (SP_LOAD_DELAY + ii - 1) / ii;
        wsample = wdepth * ii - SP_LOAD_DELAY;
        for (d = 0; d < wdepth; d++) {
            wcond[d] = tcg_temp_new_i32();
            tcg_gen_movi_i32(wcond[d], 1);
        }
        tcg_gen_movi_i32(go, 1);
        break;
    }

    /* ---- one pass over the body ---- */

    loop_top = gen_new_label();
    loop_done = gen_new_label();
    runaway = gen_new_label();

    dc->sp.active = true;
    dc->sp.ld = loads;

    gen_set_label(loop_top);

    /* The stage window slides by one: what stage k held is now stage k+1,
       and the iteration starting this pass, if there is one, enters at 0. */
    for (k = nstages - 1; k >= 1; k--) {
        tcg_gen_mov_i32(live[k], live[k - 1]);
    }
    tcg_gen_mov_i32(live[0], go);

    /* Nothing in flight and nothing starting: the loop has drained. */
    tcg_gen_mov_i32(any, live[0]);
    for (k = 1; k < nstages; k++) {
        tcg_gen_or_i32(any, any, live[k]);
    }
    tcg_gen_brcondi_i32(TCG_COND_EQ, any, 0, loop_done);

    for (c = 0; c < ii; c++) {
        dc->nwb = 0;

        /* SPLOOPW reads its condition here, four cycles before the stage
           boundary that will act on it. */
        if (kind == TIC6X_MNEM_sploopw && c == wsample) {
            int reg = creg_reg[dc->sp_start.creg];

            for (d = wdepth - 1; d >= 1; d--) {
                tcg_gen_mov_i32(wcond[d], wcond[d - 1]);
            }
            tcg_gen_setcondi_i32(dc->sp_start.z ? TCG_COND_EQ : TCG_COND_NE,
                                 wcond[0], cpu_gpr[reg], 0);
        }

        /* Oldest iteration first, which is the order the .D units would
           reach memory in. */
        for (k = nstages - 1; k >= 0; k--) {
            int issue = k * ii + c;
            int landed = issue - SP_LOAD_DELAY;

            /* Loads issued four cycles ago arrive now. They go through the
               writeback list, so an instruction reading the register in
               this same cycle still sees the old value. */
            if (landed >= 0 && landed < dynlen) {
                for (n = 0; n < SP_LOADS_PER_CYCLE; n++) {
                    SPLoad *l = &loads[landed * SP_LOADS_PER_CYCLE + n];
                    TCGv_i32 sel;

                    if (!l->present) {
                        continue;
                    }
                    sel = tcg_temp_new_i32();
                    tcg_gen_movcond_i32(TCG_COND_NE, sel, l->valid[l->depth],
                                        tcg_constant_i32(0),
                                        l->val[l->depth], cpu_gpr[l->reg]);
                    wb_add(dc, l->reg, sel);
                }
            }

            if (issue >= dynlen || !body[issue].present) {
                continue;
            }
            dc->sp.bc = issue;
            dc->sp.nld = 0;
            dc->packet_pc = body[issue].addr;
            dc->pce1 = body[issue].pce1;

            for (i = 0; i < body[issue].n; i++) {
                const DisasInsn *di = &body[issue].ins[i];
                TCGv_i32 gate = live[k];
                int km;

                if (di->op && (di->op->mnem == TIC6X_MNEM_spkernel ||
                               di->op->mnem == TIC6X_MNEM_spmask)) {
                    continue;
                }
                if (umask[issue] && sp_unit_masked(di, umask[issue])) {
                    /* SPMASKed: executed once, as the buffer loads, which
                       is the pass where stage k holds iteration 0. */
                    gate = sp_gate_pass(live[k], pass, k, TCG_COND_EQ);
                } else {
                    /* And suppressed in the cycle where a younger stage's
                       SPMASK claims this unit. */
                    for (km = k + 1; km < nstages; km++) {
                        int at = km * ii + c;

                        if (at < dynlen && umask[at] &&
                            sp_unit_masked(di, umask[at])) {
                            gate = sp_gate_pass(gate, pass, km, TCG_COND_NE);
                        }
                    }
                }
                dc->sp.gate = gate;
                dc->insn_pc = di->pc;
                trans_one(dc, di);
                dc->sp.gate = NULL;
            }
        }
        wb_flush(dc);
    }

    dc->sp.active = false;
    dc->sp.ld = NULL;

    /* ---- the stage boundary ---- */

    tcg_gen_addi_i32(pass, pass, 1);

    /* The delay lines move with the pass. */
    for (bc = 0; bc < dynlen; bc++) {
        for (n = 0; n < SP_LOADS_PER_CYCLE; n++) {
            SPLoad *l = &loads[bc * SP_LOADS_PER_CYCLE + n];

            if (!l->present) {
                continue;
            }
            for (d = l->depth; d >= 1; d--) {
                tcg_gen_mov_i32(l->val[d], l->val[d - 1]);
                tcg_gen_mov_i32(l->valid[d], l->valid[d - 1]);
            }
        }
    }

    if (kind == TIC6X_MNEM_sploopw) {
        TCGv_i32 tested = tcg_temp_new_i32();
        TCGv_i32 keep = tcg_temp_new_i32();

        tcg_gen_setcondi_i32(TCG_COND_GTU, tested, pass, wskip);
        tcg_gen_movcond_i32(TCG_COND_NE, keep, tested, tcg_constant_i32(0),
                            wcond[wdepth - 1], tcg_constant_i32(1));
        tcg_gen_and_i32(go, go, keep);
        /*
         * SPLOOPW has no epilog. When the condition fails the whole
         * pipeline stops where it stands, part-finished iterations and all;
         * SPRUFE8B 7.10 says the body must be written to tolerate that.
         */
        tcg_gen_brcondi_i32(TCG_COND_EQ, go, 0, loop_done);
    } else {
        tcg_gen_setcond_i32(TCG_COND_LTU, go, pass, iters);
    }

    /* The whole loop runs inside one translation block, so a condition that
       never comes true would hang with no way out. Bound it and say so. */
    tcg_gen_brcondi_i32(TCG_COND_GEU, pass, TIC6X_SPLOOP_MAX_PASSES, runaway);
    tcg_gen_br(loop_top);

    gen_set_label(runaway);
    gen_helper_sploop_runaway(tcg_env, tcg_constant_i32(dc->packet_pc));

    gen_set_label(loop_done);
    if (kind != TIC6X_MNEM_sploopw) {
        /* ILC counted down to zero as the loop ran. */
        tcg_gen_st_i32(tcg_constant_i32(0), tcg_env,
                       offsetof(CPUTIC6XState, cr[TIC6X_CR_ILC]));
    }

out:
    dc->sp.active = false;
    dc->sp.gate = NULL;
    dc->sp.ld = NULL;
    g_free(body);
    g_free(umask);
    g_free(loads);
    return resume;
}

/*
 * Translate one execute packet, and return how many bytes of instruction
 * stream it consumed and how many cycles it took.
 */
static int translate_packet(CPUState *cs, DisasContext *dc, uint32_t pc,
                            int *cycles)
{
    DisasPacket pk;

    dc->nwb = 0;
    dc->sp_start.active = false;

    scan_packet(cs, dc, pc, &pk);
    emit_packet(dc, &pk);
    wb_flush(dc);
    *cycles = pk.cycles;

    if (dc->sp_start.active) {
        /*
         * The loop starts on the cycle after this packet, and takes over the
         * instruction stream as far as its SPKERNEL. Its own cycles are not
         * added to *cycles: nothing may branch across an SPLOOP.
         */
        dc->sp_start.active = false;
        return gen_sploop(cs, dc, pk.next) - pc;
    }
    return pk.next - pc;
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
    dc->sp_start.active = false;
    dc->sp.active = false;
    dc->sp.gate = NULL;
    dc->sp.ld = NULL;
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
