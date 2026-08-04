/*
 * Analog Devices Blackfin emulation
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register layout and semantics are from the Blackfin Processor Programming
 * Reference rev 2.2, which is architecture-wide and applies to every BF5xx
 * part. Part-specific detail (the memory map, which peripherals exist) lives
 * in the SoC model, not here.
 */

#ifndef BFIN_CPU_H
#define BFIN_CPU_H

#include "qemu/bitops.h"
#include "hw/core/registerfields.h"
#include "cpu-qom.h"

#include "exec/cpu-common.h"
#include "exec/cpu-defs.h"
#include "exec/cpu-interrupt.h"

#ifdef CONFIG_USER_ONLY
#error "Blackfin does not support user mode emulation"
#endif

/*
 * Core register encoding map, programming reference table C-4. Instructions
 * address core registers as a 3-bit group and a 3-bit number; the common
 * 5-bit "all registers" field used by the Load Immediate forms covers groups
 * 0 to 3, which is exactly the layout of gpr[] below.
 *
 *   group 0   R0  R1  R2  R3  R4  R5  R6  R7
 *   group 1   P0  P1  P2  P3  P4  P5  SP  FP
 *   group 2   I0  I1  I2  I3  M0  M1  M2  M3
 *   group 3   B0  B1  B2  B3  L0  L1  L2  L3
 */
#define BFIN_NUM_GPR 32
/* Group 0 on its own: the eight registers a parallel issue can write twice. */
#define BFIN_NUM_DREG 8

#define BFIN_REG_R0  0
#define BFIN_REG_P0  8
#define BFIN_REG_SP  14
#define BFIN_REG_FP  15
#define BFIN_REG_I0  16
#define BFIN_REG_M0  20
#define BFIN_REG_B0  24
#define BFIN_REG_L0  28

/*
 * ASTAT, the arithmetic status register. Field positions are from figure 2-8
 * of the ADSP-BF533 Blackfin Processor Hardware Reference rev 3.6. Note that
 * AC0_COPY and V_COPY are architecturally required to read back identical to
 * AC0 and V respectively.
 */
REG32(ASTAT, 0)
FIELD(ASTAT, AZ, 0, 1)
FIELD(ASTAT, AN, 1, 1)
FIELD(ASTAT, AC0_COPY, 2, 1)
FIELD(ASTAT, V_COPY, 3, 1)
FIELD(ASTAT, CC, 5, 1)
FIELD(ASTAT, AQ, 6, 1)
FIELD(ASTAT, RND_MOD, 8, 1)
FIELD(ASTAT, AC0, 12, 1)
FIELD(ASTAT, AC1, 13, 1)
FIELD(ASTAT, AV0, 16, 1)
FIELD(ASTAT, AV0S, 17, 1)
FIELD(ASTAT, AV1, 18, 1)
FIELD(ASTAT, AV1S, 19, 1)
FIELD(ASTAT, V, 24, 1)
FIELD(ASTAT, VS, 25, 1)

/*
 * Events, in priority order. EVT0 to EVT15 live in the core MMR event vector
 * table at 0xFFE02000; IVG7 upwards are the peripheral interrupts routed by
 * the system interrupt controller.
 */
enum {
    BFIN_EXCP_EMU = 0,      /* emulation                       */
    BFIN_EXCP_RST = 1,      /* reset                           */
    BFIN_EXCP_NMI = 2,      /* nonmaskable interrupt           */
    BFIN_EXCP_EVX = 3,      /* exception                       */
    BFIN_EXCP_IVHW = 5,     /* hardware error                  */
    BFIN_EXCP_IVTMR = 6,    /* core timer                      */
    BFIN_EXCP_IVG7 = 7,
    BFIN_EXCP_IVG8 = 8,
    BFIN_EXCP_IVG9 = 9,
    BFIN_EXCP_IVG10 = 10,
    BFIN_EXCP_IVG11 = 11,
    BFIN_EXCP_IVG12 = 12,
    BFIN_EXCP_IVG13 = 13,
    BFIN_EXCP_IVG14 = 14,
    BFIN_EXCP_IVG15 = 15,
    BFIN_NUM_EVT = 16,
};

/* Synchronous exception causes, reported in SEQSTAT.EXCAUSE. */
#define BFIN_EXCAUSE_SINGLE_STEP   0x10
#define BFIN_EXCAUSE_UNDEF_INSN    0x21
#define BFIN_EXCAUSE_ILLEGAL_INSN  0x22
#define BFIN_EXCAUSE_DATA_MISALIGN 0x24
#define BFIN_EXCAUSE_UNRECOVERED   0x25
#define BFIN_EXCAUSE_INSN_MISALIGN 0x2A

typedef struct CPUArchState {
    uint32_t gpr[BFIN_NUM_GPR];

    /*
     * The two 40-bit accumulators, held right aligned in the low 40 bits.
     * A0.w and A1.w are the low 32; A0.x and A1.x the sign-extending top 8.
     */
    uint64_t a[2];

    uint32_t pc;
    uint32_t astat;
    uint32_t cc;                /* ASTAT.CC, kept out of line: it is hot */

    /* Zero overhead hardware loops. */
    uint32_t lc[2];             /* loop counters                          */
    uint32_t lt[2];             /* loop tops                              */
    uint32_t lb[2];             /* loop bottoms                           */

    /* Return address registers, one per event class. */
    uint32_t rets;              /* subroutine                             */
    uint32_t reti;              /* interrupt                              */
    uint32_t retx;              /* exception                              */
    uint32_t retn;              /* NMI                                    */
    uint32_t rete;              /* emulation                              */

    uint32_t seqstat;
    uint32_t syscfg;
    uint32_t usp;               /* user stack pointer, banked against SP  */
    uint32_t emudat;

    /* Core event controller, core MMR 0xFFE02000. */
    uint32_t evt[BFIN_NUM_EVT];
    uint32_t imask;
    uint32_t ipend;
    uint32_t ilat;

    /* Core timer, core MMR 0xFFE03000. */
    uint32_t tcntl;
    uint32_t tperiod;
    uint32_t tscale;
    uint32_t tcount;

    /* Fields up to this point are cleared by a CPU reset. */
    struct {} end_reset_fields;

    uint64_t cycles;            /* CYCLES and CYCLES2 as one counter      */
} CPUBfinState;

struct ArchCPU {
    CPUState parent_obj;

    CPUBfinState env;
};

struct BfinCPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define CPU_RESOLVING_TYPE TYPE_BFIN_CPU

void bfin_cpu_do_interrupt(CPUState *cpu);
bool bfin_cpu_exec_interrupt(CPUState *cpu, int int_req);
hwaddr bfin_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
void bfin_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
int bfin_cpu_gdb_read_register(CPUState *cpu, GByteArray *buf, int reg);
int bfin_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
G_NORETURN void bfin_raise_exception(CPUBfinState *env, int excp,
                                     uint32_t excause, uintptr_t ra);

void bfin_translate_init(void);
void bfin_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc);

const char *bfin_reg_name(unsigned int regno);

/* Pack the out-of-line CC bit back into ASTAT. */
static inline uint32_t bfin_pack_astat(CPUBfinState *env)
{
    return FIELD_DP32(env->astat, ASTAT, CC, env->cc & 1);
}

static inline void bfin_unpack_astat(CPUBfinState *env, uint32_t astat)
{
    env->astat = astat;
    env->cc = FIELD_EX32(astat, ASTAT, CC);
}

#endif /* BFIN_CPU_H */
