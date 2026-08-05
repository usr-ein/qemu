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
#include "fpu/softfloat.h"
#include "qemu/host-utils.h"

#include "insn-mnem.h.inc"

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
 * An SPLOOP that never ends. The loop is one translation block with a branch
 * back to its own head, so this is the only way out; it means either a
 * termination condition the model gets wrong or a guest doing something
 * SPRUFE8B does not describe. Either way, stopping with a message beats
 * spinning forever inside a block the debugger cannot reach into.
 */
G_NORETURN void HELPER(sploop_runaway)(CPUTIC6XState *env, uint32_t pc)
{
    CPUState *cs = env_cpu(env);

    env->pc = pc;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "tic6x: the SPLOOP at 0x%08x ran %u passes without "
                  "reaching its termination condition\n",
                  pc, TIC6X_SPLOOP_MAX_PASSES);
    cs->exception_index = TIC6X_EXCP_SPLOOP_RUNAWAY;
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

/*
 * Single-precision arithmetic, through softfloat so the results are the ones
 * IEEE specifies rather than the host's.
 *
 * The rounding mode and the exception behaviour are configurable on this
 * part, through FADCR, FAUCR and FMCR - which the firmware writes at its
 * entry point, before anything else. Those are stored but not yet acted on:
 * round-to-nearest and quiet NaNs are what the reset values select, so this
 * is right until the firmware changes them, and wrong quietly if it does.
 * Worth revisiting when there is a reason to trust the numbers.
 */
static float_status *sp_status(CPUTIC6XState *env)
{
    static float_status st;

    set_float_rounding_mode(float_round_nearest_even, &st);
    set_flush_to_zero(false, &st);
    set_default_nan_mode(true, &st);
    return &st;
}

uint32_t HELPER(addsp)(CPUTIC6XState *env, uint32_t a, uint32_t b)
{
    return float32_val(float32_add(make_float32(a), make_float32(b),
                                   sp_status(env)));
}

uint32_t HELPER(subsp)(CPUTIC6XState *env, uint32_t a, uint32_t b)
{
    return float32_val(float32_sub(make_float32(a), make_float32(b),
                                   sp_status(env)));
}

uint32_t HELPER(mpysp)(CPUTIC6XState *env, uint32_t a, uint32_t b)
{
    return float32_val(float32_mul(make_float32(a), make_float32(b),
                                   sp_status(env)));
}

uint32_t HELPER(cmpeqsp)(CPUTIC6XState *env, uint32_t a, uint32_t b)
{
    return float32_eq_quiet(make_float32(a), make_float32(b),
                            sp_status(env)) ? 1 : 0;
}

uint32_t HELPER(cmpgtsp)(CPUTIC6XState *env, uint32_t a, uint32_t b)
{
    return float32_lt(make_float32(b), make_float32(a),
                      sp_status(env)) ? 1 : 0;
}

uint32_t HELPER(cmpltsp)(CPUTIC6XState *env, uint32_t a, uint32_t b)
{
    return float32_lt(make_float32(a), make_float32(b),
                      sp_status(env)) ? 1 : 0;
}

#include "alu.c.inc"

void tic6x_cpu_do_interrupt(CPUState *cs)
{
    CPUTIC6XState *env = cpu_env(cs);

    switch (cs->exception_index) {
    case TIC6X_EXCP_UNIMPLEMENTED:
    case TIC6X_EXCP_ILLEGAL:
    case TIC6X_EXCP_FETCH_ABORT:
    case TIC6X_EXCP_SPLOOP_RUNAWAY:
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
