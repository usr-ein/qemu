/*
 * TMS320C674x gdb server stubs
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register order matches gdb/tic6x-tdep.c and gdb-xml/tic6x-core.xml: the
 * low sixteen of each file first, because that is the set the ABI passes
 * arguments in and gdb has always listed first, then CSR and PC, then the
 * upper halves.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "gdbstub/helpers.h"

enum {
    GDB_A0 = 0,                 /* A0..A15  */
    GDB_B0 = 16,                /* B0..B15  */
    GDB_CSR = 32,
    GDB_PC = 33,
    GDB_A16 = 34,               /* A16..A31 */
    GDB_B16 = 50,               /* B16..B31 */
    GDB_NUM_REGS = 66,
};

static int gdb_to_gpr(int n)
{
    if (n >= GDB_A0 && n < GDB_B0) {
        return TIC6X_REG_A(n - GDB_A0);
    }
    if (n >= GDB_B0 && n < GDB_CSR) {
        return TIC6X_REG_B(n - GDB_B0);
    }
    if (n >= GDB_A16 && n < GDB_B16) {
        return TIC6X_REG_A(16 + n - GDB_A16);
    }
    if (n >= GDB_B16 && n < GDB_NUM_REGS) {
        return TIC6X_REG_B(16 + n - GDB_B16);
    }
    return -1;
}

int tic6x_cpu_gdb_read_register(CPUState *cs, GByteArray *mem_buf, int n)
{
    CPUTIC6XState *env = cpu_env(cs);
    int gpr = gdb_to_gpr(n);

    if (gpr >= 0) {
        return gdb_get_reg32(mem_buf, env->gpr[gpr]);
    }
    switch (n) {
    case GDB_CSR:
        return gdb_get_reg32(mem_buf, env->cr[TIC6X_CR_CSR]);
    case GDB_PC:
        return gdb_get_reg32(mem_buf, env->pc);
    }
    return 0;
}

int tic6x_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    CPUTIC6XState *env = cpu_env(cs);
    uint32_t val = ldl_p(mem_buf);
    int gpr = gdb_to_gpr(n);

    if (gpr >= 0) {
        env->gpr[gpr] = val;
        return 4;
    }
    switch (n) {
    case GDB_CSR:
        env->cr[TIC6X_CR_CSR] = val;
        return 4;
    case GDB_PC:
        env->pc = val;
        return 4;
    }
    return 0;
}
