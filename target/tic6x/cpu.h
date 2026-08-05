/*
 * Texas Instruments TMS320C674x emulation
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register layout and semantics are from the TMS320C674x DSP CPU and
 * Instruction Set Reference Guide, SPRUFE8B. Part-specific detail - the
 * memory map, which peripherals exist - lives in the SoC model, not here.
 */

#ifndef TIC6X_CPU_H
#define TIC6X_CPU_H

#include "qemu/bitops.h"
#include "hw/core/registerfields.h"
#include "cpu-qom.h"

#include "exec/cpu-common.h"
#include "exec/cpu-defs.h"
#include "exec/cpu-interrupt.h"

#ifdef CONFIG_USER_ONLY
#error "tic6x does not support user mode emulation"
#endif

/*
 * Two register files of 32, A and B, held here as one array: A0 to A31 are
 * 0 to 31 and B0 to B31 are 32 to 63. Instructions name a side and a
 * register number, and the cross paths let a unit on one side read one
 * operand from the other, so keeping both in one array and indexing by
 * (side << 5 | number) is what the encoding already does.
 */
#define TIC6X_NUM_GPR 64
#define TIC6X_REG_A(n) (n)
#define TIC6X_REG_B(n) (32 + (n))

/*
 * The functional units, as the opcode table names them. Each execute packet
 * may use each unit at most once, which is what makes a packet's
 * instructions independent.
 */
enum {
    TIC6X_UNIT_l = 0,
    TIC6X_UNIT_s,
    TIC6X_UNIT_m,
    TIC6X_UNIT_d,
    TIC6X_UNIT_nfu,             /* no functional unit: nop, and SPLOOP    */
    TIC6X_UNIT_COUNT,
};

/*
 * Control registers, indexed by their architectural crlo number so the
 * encoding indexes this array directly. The numbering is from binutils'
 * tic6x-control-registers.h, which agrees with SPRUFE8B chapter 2.8; taking
 * it from the hardware rather than inventing an order means mvc needs no
 * translation table.
 */
#define TIC6X_NUM_CR 32

enum {
    TIC6X_CR_AMR = 0x00,        /* addressing mode                        */
    TIC6X_CR_CSR = 0x01,        /* control status                         */
    TIC6X_CR_IFR = 0x02,        /* interrupt flag, read; ISR on write     */
    TIC6X_CR_ICR = 0x03,        /* interrupt clear                        */
    TIC6X_CR_IER = 0x04,        /* interrupt enable                       */
    TIC6X_CR_ISTP = 0x05,       /* interrupt service table pointer        */
    TIC6X_CR_IRP = 0x06,        /* interrupt return pointer               */
    TIC6X_CR_NRP = 0x07,        /* NMI return pointer                     */
    TIC6X_CR_TSCL = 0x0a,       /* time stamp counter, low                */
    TIC6X_CR_TSCH = 0x0b,       /* time stamp counter, high               */
    TIC6X_CR_ILC = 0x0d,        /* SPLOOP inner loop count                */
    TIC6X_CR_RILC = 0x0e,       /* SPLOOP reload inner loop count         */
    TIC6X_CR_REP = 0x0f,        /* restricted entry point                 */
    TIC6X_CR_DNUM = 0x11,       /* core number                            */
    TIC6X_CR_FADCR = 0x12,      /* floating point adder config            */
    TIC6X_CR_FAUCR = 0x13,      /* floating point auxiliary config        */
    TIC6X_CR_FMCR = 0x14,       /* floating point multiplier config       */
};

/* CSR, SPRUFE8B 2.8.4. GIE and PGIE are the two that gate interrupts. */
REG32(CSR, 0)
FIELD(CSR, GIE, 0, 1)
FIELD(CSR, PGIE, 1, 1)
FIELD(CSR, DCC, 2, 3)
FIELD(CSR, PCC, 5, 3)
FIELD(CSR, EN, 8, 1)
FIELD(CSR, SAT, 9, 1)
FIELD(CSR, PWRD, 10, 6)
FIELD(CSR, REVISION, 16, 8)
FIELD(CSR, CPU_ID, 24, 8)

/*
 * A branch does not take effect where it is written: the target is fetched
 * five execute packets later, and those five packets execute first. The
 * compiler fills them with real work, so this is not a detail that can be
 * rounded off - it is the normal way C6000 code is scheduled.
 *
 * The translator handles the common case by carrying on through the delay
 * slots and emitting the jump when it reaches the landing packet, which
 * keeps blocks whole. What it cannot know at translation time is whether a
 * predicated branch actually executed, so the branch writes its target and
 * a taken flag here and the landing site reads them back.
 */
#define TIC6X_BRANCH_DELAY 5

enum {
    TIC6X_EXCP_NONE = 0,
    TIC6X_EXCP_UNIMPLEMENTED,   /* an encoding this model does not run    */
    TIC6X_EXCP_ILLEGAL,         /* no encoding matched at all             */
    TIC6X_EXCP_INTERRUPT,
};

typedef struct CPUArchState {
    uint32_t gpr[TIC6X_NUM_GPR];

    uint32_t pc;

    uint32_t cr[TIC6X_NUM_CR];

    /* One branch in flight; see TIC6X_BRANCH_DELAY above. */
    uint32_t br_target;
    uint32_t br_taken;

    /*
     * What the translator could not run, kept so the exception handler can
     * say which instruction it was rather than just "something at this pc".
     * This is the whole coverage-growing loop: run, trap, look at the name,
     * implement it, run again.
     */
    uint32_t excp_insn;
    uint32_t excp_mnem;

    /* Fields up to this point are cleared by a CPU reset. */
    struct {} end_reset_fields;

    uint64_t cycles;
} CPUTIC6XState;

struct ArchCPU {
    CPUState parent_obj;

    CPUTIC6XState env;

    /* Where the core starts. A host boot writes this into L2 before
       releasing the core, so the board sets it. */
    uint32_t reset_pc;
};

struct TIC6XCPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define CPU_RESOLVING_TYPE TYPE_TIC6X_CPU

void tic6x_cpu_do_interrupt(CPUState *cpu);
bool tic6x_cpu_exec_interrupt(CPUState *cpu, int int_req);
hwaddr tic6x_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
void tic6x_cpu_dump_state(CPUState *cpu, FILE *f, int flags);
int tic6x_cpu_gdb_read_register(CPUState *cpu, GByteArray *buf, int reg);
int tic6x_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);

void tic6x_translate_init(void);
void tic6x_translate_code(CPUState *cs, TranslationBlock *tb,
                          int *max_insns, vaddr pc, void *host_pc);

const char *tic6x_reg_name(unsigned int regno);
const char *tic6x_mnemonic_name(unsigned int mnem);

#endif /* TIC6X_CPU_H */
