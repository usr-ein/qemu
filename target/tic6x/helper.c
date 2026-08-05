/*
 * TMS320C674x helpers
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/cpu-common.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "system/runstate.h"

/*
 * Growing instruction coverage works the same way it did for the Blackfin:
 * run the real firmware, stop on the first encoding the translator does not
 * implement, look at what it was, implement it, run again. For that to be
 * quick the trap has to name the instruction rather than just the address,
 * so the mnemonic is carried through to here.
 */
G_NORETURN void HELPER(unimplemented)(CPUTIC6XState *env, uint32_t insn,
                                      uint32_t mnem)
{
    CPUState *cs = env_cpu(env);

    env->excp_insn = insn;
    env->excp_mnem = mnem;
    qemu_log_mask(LOG_UNIMP,
                  "tic6x: unimplemented %s (0x%08x) at pc 0x%08x\n",
                  tic6x_mnemonic_name(mnem), insn, env->pc);
    cs->exception_index = TIC6X_EXCP_UNIMPLEMENTED;
    cpu_loop_exit(cs);
}

G_NORETURN void HELPER(illegal)(CPUTIC6XState *env, uint32_t insn)
{
    CPUState *cs = env_cpu(env);

    env->excp_insn = insn;
    env->excp_mnem = ~0u;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "tic6x: no encoding matches 0x%08x at pc 0x%08x\n",
                  insn, env->pc);
    cs->exception_index = TIC6X_EXCP_ILLEGAL;
    cpu_loop_exit(cs);
}

/*
 * The control register file. Reading one that is not modelled returns zero
 * and says so once: a control register quietly reading zero is a classic way
 * for a boot to hang with no clue why, and the log line is the clue.
 */
uint32_t HELPER(read_creg)(CPUTIC6XState *env, uint32_t crlo)
{
    if (crlo < TIC6X_NUM_CR) {
        return env->cr[crlo];
    }
    qemu_log_mask(LOG_UNIMP, "tic6x: read of unmodelled control register %u\n",
                  crlo);
    return 0;
}

void HELPER(write_creg)(CPUTIC6XState *env, uint32_t crlo, uint32_t val)
{
    if (crlo < TIC6X_NUM_CR) {
        env->cr[crlo] = val;
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "tic6x: write of unmodelled control register %u = 0x%08x\n",
                  crlo, val);
}

void tic6x_cpu_do_interrupt(CPUState *cs)
{
    CPUTIC6XState *env = cpu_env(cs);

    switch (cs->exception_index) {
    case TIC6X_EXCP_UNIMPLEMENTED:
    case TIC6X_EXCP_ILLEGAL:
        /*
         * Stop rather than pretend. A DSP that carries on past an
         * instruction it did not run produces wrong answers quietly, and
         * the whole point of this trap is to be told what to implement
         * next.
         */
        cpu_dump_state(cs, stderr, 0);
        qemu_system_guest_panicked(NULL);
        break;
    default:
        break;
    }
}

bool tic6x_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    /*
     * Interrupts are not delivered yet. The firmware sets up ISTP, IER and
     * CSR.GIE early, so this will be needed, but nothing raises a line into
     * the core until the peripherals exist - see task #28.
     */
    return false;
}
