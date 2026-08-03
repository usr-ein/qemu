/*
 * Analog Devices Blackfin gdb server stubs
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register order matches gdb/bfin-tdep.c and gdb-xml/bfin-core.xml.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "gdbstub/helpers.h"

enum {
    GDB_A0X = 32, GDB_A0W, GDB_A1X, GDB_A1W,
    GDB_ASTAT, GDB_RETS,
    GDB_LC0, GDB_LT0, GDB_LB0, GDB_LC1, GDB_LT1, GDB_LB1,
    GDB_CYCLES, GDB_CYCLES2,
    GDB_USP, GDB_SEQSTAT, GDB_SYSCFG,
    GDB_RETI, GDB_RETX, GDB_RETN, GDB_RETE,
    GDB_PC,
    GDB_NUM_REGS,
};

int bfin_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPUBfinState *env = cpu_env(cs);
    uint32_t val;

    if (n < BFIN_NUM_GPR) {
        val = env->gpr[n];
    } else {
        switch (n) {
        case GDB_A0X:
            val = (env->a[0] >> 32) & 0xff;
            break;
        case GDB_A0W:
            val = env->a[0] & 0xffffffff;
            break;
        case GDB_A1X:
            val = (env->a[1] >> 32) & 0xff;
            break;
        case GDB_A1W:
            val = env->a[1] & 0xffffffff;
            break;
        case GDB_ASTAT:
            val = bfin_pack_astat(env);
            break;
        case GDB_RETS:
            val = env->rets;
            break;
        case GDB_LC0:
            val = env->lc[0];
            break;
        case GDB_LT0:
            val = env->lt[0];
            break;
        case GDB_LB0:
            val = env->lb[0];
            break;
        case GDB_LC1:
            val = env->lc[1];
            break;
        case GDB_LT1:
            val = env->lt[1];
            break;
        case GDB_LB1:
            val = env->lb[1];
            break;
        case GDB_CYCLES:
            val = env->cycles & 0xffffffff;
            break;
        case GDB_CYCLES2:
            val = env->cycles >> 32;
            break;
        case GDB_USP:
            val = env->usp;
            break;
        case GDB_SEQSTAT:
            val = env->seqstat;
            break;
        case GDB_SYSCFG:
            val = env->syscfg;
            break;
        case GDB_RETI:
            val = env->reti;
            break;
        case GDB_RETX:
            val = env->retx;
            break;
        case GDB_RETN:
            val = env->retn;
            break;
        case GDB_RETE:
            val = env->rete;
            break;
        case GDB_PC:
            val = env->pc;
            break;
        default:
            return 0;
        }
    }

    return gdb_get_reg32(mem_buf, val);
}

int bfin_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUBfinState *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);

    if (n < BFIN_NUM_GPR) {
        env->gpr[n] = val;
        return 4;
    }

    switch (n) {
    case GDB_A0X:
        env->a[0] = deposit64(env->a[0], 32, 8, val);
        break;
    case GDB_A0W:
        env->a[0] = deposit64(env->a[0], 0, 32, val);
        break;
    case GDB_A1X:
        env->a[1] = deposit64(env->a[1], 32, 8, val);
        break;
    case GDB_A1W:
        env->a[1] = deposit64(env->a[1], 0, 32, val);
        break;
    case GDB_ASTAT:
        bfin_unpack_astat(env, val);
        break;
    case GDB_RETS:
        env->rets = val;
        break;
    case GDB_LC0:
        env->lc[0] = val;
        break;
    case GDB_LT0:
        env->lt[0] = val;
        break;
    case GDB_LB0:
        env->lb[0] = val;
        break;
    case GDB_LC1:
        env->lc[1] = val;
        break;
    case GDB_LT1:
        env->lt[1] = val;
        break;
    case GDB_LB1:
        env->lb[1] = val;
        break;
    case GDB_CYCLES:
        env->cycles = deposit64(env->cycles, 0, 32, val);
        break;
    case GDB_CYCLES2:
        env->cycles = deposit64(env->cycles, 32, 32, val);
        break;
    case GDB_USP:
        env->usp = val;
        break;
    case GDB_SEQSTAT:
        env->seqstat = val;
        break;
    case GDB_SYSCFG:
        env->syscfg = val;
        break;
    case GDB_RETI:
        env->reti = val;
        break;
    case GDB_RETX:
        env->retx = val;
        break;
    case GDB_RETN:
        env->retn = val;
        break;
    case GDB_RETE:
        env->rete = val;
        break;
    case GDB_PC:
        env->pc = val;
        break;
    default:
        return 0;
    }

    return 4;
}
