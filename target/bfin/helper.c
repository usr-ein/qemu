/*
 * Analog Devices Blackfin helpers and event handling
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The event controller is described in chapter 4 of the ADSP-BF533 Blackfin
 * Processor Hardware Reference rev 3.6. Events are prioritised, EVT0 to EVT15
 * hold their handler addresses, and each class has its own return register.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "system/runstate.h"
#include "qemu/main-loop.h"

G_NORETURN void bfin_raise_exception(CPUBfinState *env, int excp,
                                     uint32_t excause, uintptr_t ra)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    env->seqstat = deposit32(env->seqstat, 0, 6, excause);
    cpu_loop_exit_restore(cs, ra);
}

void HELPER(raise_exception)(CPUBfinState *env, uint32_t excp, uint32_t excause)
{
    bfin_raise_exception(env, excp, excause, GETPC());
}

void HELPER(idle)(CPUBfinState *env)
{
    CPUState *cs = env_cpu(env);

    cs->halted = 1;
    cs->exception_index = EXCP_HLT;
    cpu_loop_exit(cs);
}

/*
 * Returning from an event clears the IPEND bit that event set, and there is a
 * separate instruction per event: RTI for the interrupt levels, RTX for an
 * exception, RTN for NMI, RTE for emulation. RTI clears the highest priority
 * interrupt standing in IPEND, which is the lowest numbered one, and must
 * leave the four low bits alone - they belong to the other three and to the
 * global disable that CLI sets.
 *
 * Only RTI cleared anything here, and it took the lowest set bit whatever it
 * was. An exception handler returning with RTX therefore left IPEND bit 3
 * standing for the rest of the run, and since an event cannot pre-empt one of
 * equal or higher priority, nothing at IVG3 or below could ever be accepted
 * again: the processor carried on with every interrupt latched in ILAT and
 * none of them delivered. The GUI processor was seen in exactly that state,
 * with IPEND 0x8 after an undefined-instruction exception.
 *
 * Clearing a bit can unblock something that was already latched, so the
 * pending check has to be redone. Translated code does not hold the big lock
 * and cpu_interrupt insists on it, hence the guard.
 */
static void bfin_event_return(CPUBfinState *env, uint32_t clear)
{
    env->ipend &= ~clear;

    if (env->ilat & env->imask) {
        BQL_LOCK_GUARD();
        cpu_interrupt(env_cpu(env), CPU_INTERRUPT_HARD);
    }
}

void HELPER(rti)(CPUBfinState *env)
{
    uint32_t levels = env->ipend & BFIN_IPEND_IVG_MASK;

    bfin_event_return(env, levels & -levels);
}

void HELPER(rtx)(CPUBfinState *env)
{
    bfin_event_return(env, 1u << BFIN_EXCP_EVX);
}

void HELPER(rtn)(CPUBfinState *env)
{
    bfin_event_return(env, 1u << BFIN_EXCP_NMI);
}

void HELPER(rte)(CPUBfinState *env)
{
    bfin_event_return(env, 1u << BFIN_EXCP_EMU);
}

/*
 * CLI returns the previous IMASK and clears it; STI restores one. Both are
 * supervisor-only on real hardware, which this model does not enforce.
 */
uint32_t HELPER(cli)(CPUBfinState *env)
{
    uint32_t old = env->imask;

    env->imask = 0;
    return old;
}

void HELPER(sti)(CPUBfinState *env, uint32_t value)
{
    env->imask = value;
    if (env->ilat & env->imask) {
        /*
         * Translated code runs without the big QEMU lock, and cpu_interrupt
         * asserts that it is held. Devices raise interrupts from a context
         * that already owns it; a helper has to take it.
         */
        BQL_LOCK_GUARD();
        cpu_interrupt(env_cpu(env), CPU_INTERRUPT_HARD);
    }
}

void HELPER(raise_ivg)(CPUBfinState *env, uint32_t n)
{
    env->ilat |= 1u << n;
    if (env->ilat & env->imask) {
        BQL_LOCK_GUARD();
        cpu_interrupt(env_cpu(env), CPU_INTERRUPT_HARD);
    }
}

/*
 * Core register access for the Move Register instruction, using the register
 * group and number encoding of programming reference table C-4. Groups 0 to 3
 * are handled inline by the translator and never reach here.
 */
uint32_t HELPER(read_creg)(CPUBfinState *env, uint32_t regno)
{
    unsigned grp = regno >> 3, num = regno & 7;

    switch (grp) {
    case 4:
        switch (num) {
        case 0:
            return (env->a[0] >> 32) & 0xff;
        case 1:
            return env->a[0] & 0xffffffff;
        case 2:
            return (env->a[1] >> 32) & 0xff;
        case 3:
            return env->a[1] & 0xffffffff;
        case 6:
            return bfin_pack_astat(env);
        case 7:
            return env->rets;
        }
        break;
    case 6:
        switch (num) {
        case 0:
            return env->lc[0];
        case 1:
            return env->lt[0];
        case 2:
            return env->lb[0];
        case 3:
            return env->lc[1];
        case 4:
            return env->lt[1];
        case 5:
            return env->lb[1];
        case 6:
            return env->cycles & 0xffffffff;
        case 7:
            return env->cycles >> 32;
        }
        break;
    case 7:
        switch (num) {
        case 0:
            return env->usp;
        case 1:
            return env->seqstat;
        case 2:
            return env->syscfg;
        case 3:
            return env->reti;
        case 4:
            return env->retx;
        case 5:
            return env->retn;
        case 6:
            return env->rete;
        case 7:
            return env->emudat;
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "bfin: read of reserved core register %u:%u\n",
                  grp, num);
    return 0;
}

void HELPER(write_creg)(CPUBfinState *env, uint32_t regno, uint32_t value)
{
    unsigned grp = regno >> 3, num = regno & 7;

    switch (grp) {
    case 4:
        switch (num) {
        case 0:
            env->a[0] = deposit64(env->a[0], 32, 8, value);
            return;
        case 1:
            env->a[0] = deposit64(env->a[0], 0, 32, value);
            return;
        case 2:
            env->a[1] = deposit64(env->a[1], 32, 8, value);
            return;
        case 3:
            env->a[1] = deposit64(env->a[1], 0, 32, value);
            return;
        case 6:
            bfin_unpack_astat(env, value);
            return;
        case 7:
            env->rets = value;
            return;
        }
        break;
    case 6:
        switch (num) {
        case 0:
            env->lc[0] = value;
            return;
        case 1:
            env->lt[0] = value;
            return;
        case 2:
            env->lb[0] = value;
            return;
        case 3:
            env->lc[1] = value;
            return;
        case 4:
            env->lt[1] = value;
            return;
        case 5:
            env->lb[1] = value;
            return;
        case 6:
            env->cycles = deposit64(env->cycles, 0, 32, value);
            return;
        case 7:
            env->cycles = deposit64(env->cycles, 32, 32, value);
            return;
        }
        break;
    case 7:
        switch (num) {
        case 0:
            env->usp = value;
            return;
        case 1:
            env->seqstat = value;
            return;
        case 2:
            env->syscfg = value;
            return;
        case 3:
            env->reti = value;
            return;
        case 4:
            env->retx = value;
            return;
        case 5:
            env->retn = value;
            return;
        case 6:
            env->rete = value;
            return;
        case 7:
            env->emudat = value;
            return;
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "bfin: write to reserved core register %u:%u\n",
                  grp, num);
}

/*
 * Deliver an event. The return address goes to the register belonging to the
 * event class, the pending bit is set, and control transfers to the vector in
 * the event vector table.
 */
void bfin_cpu_do_interrupt(CPUState *cs)
{
    CPUBfinState *env = cpu_env(cs);
    int excp = cs->exception_index;

    if (excp < 0 || excp >= BFIN_NUM_EVT) {
        qemu_log_mask(LOG_GUEST_ERROR, "bfin: bogus event %d\n", excp);
        return;
    }

    switch (excp) {
    case BFIN_EXCP_EMU:
        env->rete = env->pc;
        break;
    case BFIN_EXCP_NMI:
        env->retn = env->pc;
        break;
    case BFIN_EXCP_EVX:
        env->retx = env->pc;
        break;
    case BFIN_EXCP_RST:
        break;
    default:
        env->reti = env->pc;
        break;
    }

    env->ipend |= 1u << excp;
    env->ilat &= ~(1u << excp);
    env->pc = env->evt[excp];

    if (env->pc == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bfin: event %d taken with EVT%d unset\n", excp, excp);
    }
}

bool bfin_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUBfinState *env = cpu_env(cs);
    uint32_t pending;
    int n;

    if (!(interrupt_request & CPU_INTERRUPT_HARD)) {
        return false;
    }

    pending = env->ilat & env->imask;
    if (!pending) {
        return false;
    }

    /*
     * Lower numbered events have higher priority, and an event cannot
     * pre-empt one of the same or higher priority that is already active.
     */
    n = ctz32(pending);
    if (env->ipend && ctz32(env->ipend) <= n) {
        return false;
    }

    cs->exception_index = n;
    bfin_cpu_do_interrupt(cs);
    return true;
}

/*
 * Rotate a data register through CC, which acts as a 33rd bit above bit 31.
 * The count is signed: positive rotates left, negative right. Chapter 2 of
 * the ADSP-BF533 hardware reference describes the shifter.
 */
uint32_t HELPER(rot)(CPUBfinState *env, uint32_t value, int32_t count)
{
    uint64_t wide = ((uint64_t)(env->cc & 1) << 32) | value;
    unsigned n = count < 0 ? -count : count;

    n %= 33;
    if (n) {
        if (count > 0) {
            wide = (wide << n) | (wide >> (33 - n));
        } else {
            wide = (wide >> n) | (wide << (33 - n));
        }
    }
    env->cc = (wide >> 32) & 1;
    return wide & 0xffffffff;
}

/*
 * Bit field extraction. The pattern register holds the position of the field
 * in bits 12-8 and its length in bits 4-0 (table 13-2 of the programming
 * reference). A length of zero yields zero, and bits above the top of the
 * scene register are treated as zero.
 */
uint32_t HELPER(bitextract)(uint32_t scene, uint32_t pattern, uint32_t sign)
{
    unsigned pos = (pattern >> 8) & 0x1f;
    unsigned len = pattern & 0x1f;
    uint32_t field;

    if (len == 0) {
        return 0;
    }
    field = pos >= 32 ? 0 : scene >> pos;
    if (len < 32) {
        field &= (1u << len) - 1;
        if (sign && (field & (1u << (len - 1)))) {
            field |= ~((1u << len) - 1);
        }
    }
    return field;
}

#include "helper-mac.c.inc"
