/*
 * Analog Devices Blackfin CPU
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "qapi/error.h"
#include "cpu.h"
#include "migration/vmstate.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/translation-block.h"
#include "exec/target_page.h"
#include "hw/core/sysemu-cpu-ops.h"
#include "accel/tcg/cpu-ops.h"

/*
 * The Blackfin has no MMU. Addresses issued by the core are physical; the
 * CPLBs provide protection and cacheability attributes only, and this model
 * does not enforce them.
 */

static void bfin_cpu_set_pc(CPUState *cs, vaddr value)
{
    CPUBfinState *env = cpu_env(cs);

    env->pc = value;
}

static vaddr bfin_cpu_get_pc(CPUState *cs)
{
    CPUBfinState *env = cpu_env(cs);

    return env->pc;
}

static TCGTBCPUState bfin_get_tb_cpu_state(CPUState *cs)
{
    CPUBfinState *env = cpu_env(cs);

    /*
     * Hardware loop state has to be part of the translation key. LC0/LC1
     * being nonzero changes what the instruction at LB does, so a block
     * translated with a loop active must not be reused without one.
     */
    uint32_t flags = 0;

    flags |= env->lc[0] ? 1 : 0;
    flags |= env->lc[1] ? 2 : 0;

    return (TCGTBCPUState){ .pc = env->pc, .flags = flags };
}

static void bfin_cpu_synchronize_from_tb(CPUState *cs,
                                         const TranslationBlock *tb)
{
    CPUBfinState *env = cpu_env(cs);

    tcg_debug_assert(!tcg_cflags_has(cs, CF_PCREL));
    env->pc = tb->pc;
}

static void bfin_restore_state_to_opc(CPUState *cs,
                                      const TranslationBlock *tb,
                                      const uint64_t *data)
{
    CPUBfinState *env = cpu_env(cs);

    env->pc = data[0];
}

static int bfin_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return 0;
}

static bool bfin_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                              MMUAccessType access_type, int mmu_idx,
                              bool probe, uintptr_t retaddr)
{
    tlb_set_page(cs, address & TARGET_PAGE_MASK,
                 address & TARGET_PAGE_MASK,
                 PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                 mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

static bool bfin_cpu_has_work(CPUState *cs)
{
    return cs->interrupt_request & CPU_INTERRUPT_HARD;
}

static void bfin_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    BfinCPUClass *bcc = BFIN_CPU_GET_CLASS(obj);
    CPUBfinState *env = cpu_env(cs);

    if (bcc->parent_phases.hold) {
        bcc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPUBfinState, end_reset_fields));

    /*
     * The core comes out of reset executing the on-chip boot ROM, which reads
     * the boot source selected by the BMODE pins and processes an LDR stream.
     * A board that loads the LDR itself overrides this.
     */
    env->pc = 0xef000000;
    env->seqstat = 0;
    env->syscfg = 0;
}

static void bfin_cpu_disas_set_info(const CPUState *cs, disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
    info->print_insn = NULL;
}

hwaddr bfin_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    /* No MMU: the core issues physical addresses. */
    return addr;
}

static const struct SysemuCPUOps bfin_sysemu_ops = {
    .has_work = bfin_cpu_has_work,
    .get_phys_page_debug = bfin_cpu_get_phys_page_debug,
};

static void bfin_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    BfinCPUClass *bcc = BFIN_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    bcc->parent_realize(dev, errp);
}

static void bfin_cpu_initfn(Object *obj)
{
    /* Nothing per-instance yet; state is established by reset. */
}

void bfin_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUBfinState *env = cpu_env(cs);
    int i;

    qemu_fprintf(f, "PC  %08x  ASTAT %08x  CC %d  SEQSTAT %08x\n",
                 env->pc, bfin_pack_astat(env), env->cc & 1, env->seqstat);
    for (i = 0; i < 8; i++) {
        qemu_fprintf(f, "R%d  %08x%s", i, env->gpr[BFIN_REG_R0 + i],
                     (i % 4) == 3 ? "\n" : "  ");
    }
    for (i = 0; i < 6; i++) {
        qemu_fprintf(f, "P%d  %08x%s", i, env->gpr[BFIN_REG_P0 + i],
                     (i % 4) == 3 ? "\n" : "  ");
    }
    qemu_fprintf(f, "SP  %08x  FP  %08x  USP %08x\n",
                 env->gpr[BFIN_REG_SP], env->gpr[BFIN_REG_FP], env->usp);
    for (i = 0; i < 4; i++) {
        qemu_fprintf(f, "I%d  %08x  M%d  %08x  B%d  %08x  L%d  %08x\n",
                     i, env->gpr[BFIN_REG_I0 + i],
                     i, env->gpr[BFIN_REG_M0 + i],
                     i, env->gpr[BFIN_REG_B0 + i],
                     i, env->gpr[BFIN_REG_L0 + i]);
    }
    qemu_fprintf(f, "A0  %010" PRIx64 "  A1  %010" PRIx64 "\n",
                 env->a[0] & 0xffffffffffULL, env->a[1] & 0xffffffffffULL);
    qemu_fprintf(f, "RETS %08x RETI %08x RETX %08x RETN %08x RETE %08x\n",
                 env->rets, env->reti, env->retx, env->retn, env->rete);
    for (i = 0; i < 2; i++) {
        qemu_fprintf(f, "LC%d %08x  LT%d %08x  LB%d %08x\n",
                     i, env->lc[i], i, env->lt[i], i, env->lb[i]);
    }
    qemu_fprintf(f, "IMASK %08x IPEND %08x ILAT %08x\n",
                 env->imask, env->ipend, env->ilat);
}

const char *bfin_reg_name(unsigned int regno)
{
    static const char *const names[BFIN_NUM_GPR] = {
        "R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
        "P0", "P1", "P2", "P3", "P4", "P5", "SP", "FP",
        "I0", "I1", "I2", "I3", "M0", "M1", "M2", "M3",
        "B0", "B1", "B2", "B3", "L0", "L1", "L2", "L3",
    };

    return regno < BFIN_NUM_GPR ? names[regno] : "??";
}

static const VMStateDescription vmstate_bfin_cpu = {
    .name = "cpu/bfin",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(env.gpr, ArchCPU, BFIN_NUM_GPR),
        VMSTATE_UINT64_ARRAY(env.a, ArchCPU, 2),
        VMSTATE_UINT32(env.pc, ArchCPU),
        VMSTATE_UINT32(env.astat, ArchCPU),
        VMSTATE_UINT32(env.cc, ArchCPU),
        VMSTATE_UINT32_ARRAY(env.lc, ArchCPU, 2),
        VMSTATE_UINT32_ARRAY(env.lt, ArchCPU, 2),
        VMSTATE_UINT32_ARRAY(env.lb, ArchCPU, 2),
        VMSTATE_UINT32(env.rets, ArchCPU),
        VMSTATE_UINT32(env.reti, ArchCPU),
        VMSTATE_UINT32(env.retx, ArchCPU),
        VMSTATE_UINT32(env.retn, ArchCPU),
        VMSTATE_UINT32(env.rete, ArchCPU),
        VMSTATE_UINT32(env.seqstat, ArchCPU),
        VMSTATE_UINT32(env.syscfg, ArchCPU),
        VMSTATE_UINT32(env.usp, ArchCPU),
        VMSTATE_UINT32_ARRAY(env.evt, ArchCPU, BFIN_NUM_EVT),
        VMSTATE_UINT32(env.imask, ArchCPU),
        VMSTATE_UINT32(env.ipend, ArchCPU),
        VMSTATE_UINT32(env.ilat, ArchCPU),
        VMSTATE_UINT32(env.tcntl, ArchCPU),
        VMSTATE_UINT32(env.tperiod, ArchCPU),
        VMSTATE_UINT32(env.tscale, ArchCPU),
        VMSTATE_UINT32(env.tcount, ArchCPU),
        VMSTATE_END_OF_LIST()
    }
};

static const TCGCPUOps bfin_tcg_ops = {
    /* MTTCG not yet supported: require strict ordering */
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = bfin_translate_init,
    .translate_code = bfin_translate_code,
    .get_tb_cpu_state = bfin_get_tb_cpu_state,
    .synchronize_from_tb = bfin_cpu_synchronize_from_tb,
    .restore_state_to_opc = bfin_restore_state_to_opc,
    .mmu_index = bfin_cpu_mmu_index,
    .tlb_fill = bfin_cpu_tlb_fill,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .cpu_exec_interrupt = bfin_cpu_exec_interrupt,
    .cpu_exec_halt = bfin_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = bfin_cpu_do_interrupt,
};

static void bfin_cpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    CPUClass *cc = CPU_CLASS(klass);
    BfinCPUClass *bcc = BFIN_CPU_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_parent_realize(dc, bfin_cpu_realizefn,
                                    &bcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, bfin_cpu_reset_hold, NULL,
                                       &bcc->parent_phases);

    cc->dump_state = bfin_cpu_dump_state;
    cc->set_pc = bfin_cpu_set_pc;
    cc->get_pc = bfin_cpu_get_pc;

    cc->sysemu_ops = &bfin_sysemu_ops;
    cc->gdb_read_register = bfin_cpu_gdb_read_register;
    cc->gdb_write_register = bfin_cpu_gdb_write_register;
    cc->disas_set_info = bfin_cpu_disas_set_info;

    dc->vmsd = &vmstate_bfin_cpu;
    cc->gdb_core_xml_file = "bfin-core.xml";
    cc->tcg_ops = &bfin_tcg_ops;
}

static const TypeInfo bfin_cpu_types[] = {
    {
        .name           = TYPE_BFIN_CPU,
        .parent         = TYPE_CPU,
        .instance_size  = sizeof(BfinCPU),
        .instance_align = __alignof(BfinCPU),
        .instance_init  = bfin_cpu_initfn,
        .abstract       = true,
        .class_size     = sizeof(BfinCPUClass),
        .class_init     = bfin_cpu_class_init,
    },
    {
        .name           = TYPE_BF531_CPU,
        .parent         = TYPE_BFIN_CPU,
    },
};

DEFINE_TYPES(bfin_cpu_types)
