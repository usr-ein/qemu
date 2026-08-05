/*
 * Texas Instruments TMS320C674x CPU
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
#include "hw/core/qdev-properties.h"
#include "hw/core/sysemu-cpu-ops.h"
#include "accel/tcg/cpu-ops.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "qemu/log.h"
#include "qemu/rcu.h"

/*
 * The C674x has no MMU. There is a memory protection unit and caches, but
 * addresses issued by the core are physical and this model does not enforce
 * protection.
 */

static void tic6x_cpu_set_pc(CPUState *cs, vaddr value)
{
    CPUTIC6XState *env = cpu_env(cs);

    env->pc = value;
}

static vaddr tic6x_cpu_get_pc(CPUState *cs)
{
    CPUTIC6XState *env = cpu_env(cs);

    return env->pc;
}

static TCGTBCPUState tic6x_get_tb_cpu_state(CPUState *cs)
{
    CPUTIC6XState *env = cpu_env(cs);

    return (TCGTBCPUState){ .pc = env->pc, .flags = env->br_cnt };
}

static void tic6x_cpu_synchronize_from_tb(CPUState *cs,
                                          const TranslationBlock *tb)
{
    CPUTIC6XState *env = cpu_env(cs);

    tcg_debug_assert(!tcg_cflags_has(cs, CF_PCREL));
    env->pc = tb->pc;
}

static void tic6x_restore_state_to_opc(CPUState *cs,
                                       const TranslationBlock *tb,
                                       const uint64_t *data)
{
    CPUTIC6XState *env = cpu_env(cs);

    env->pc = data[0];
}

static int tic6x_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return 0;
}

/* Is there real memory behind this address, or would a read invent zeros? */
static bool tic6x_addr_is_ram(CPUState *cs, vaddr addr)
{
    AddressSpace *as = cpu_get_address_space(cs, 0);
    hwaddr xlat, len = 1;
    MemoryRegion *mr;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(as, addr, &xlat, &len, false,
                                 MEMTXATTRS_UNSPECIFIED);
    return mr && (memory_region_is_ram(mr) || memory_region_is_romd(mr));
}

static bool tic6x_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                               MMUAccessType access_type, int mmu_idx,
                               bool probe, uintptr_t retaddr)
{
    /*
     * Fetching from somewhere with no memory behind it is always a bug, and
     * without this it is a silent one: an unmapped read returns zero, zero
     * decodes as NOP, and a core that has jumped somewhere wild walks
     * forward through the whole address space executing nothing for as long
     * as it is left running. That happened, and it buried the evidence -
     * millions of log lines of a runaway doing wild loads, on top of the few
     * real accesses that mattered. Stopping at the first bad fetch says
     * where the jump went instead.
     *
     * Data accesses are left alone. Peripherals that are not modelled yet
     * are legitimately not RAM, and they have their own logging.
     */
    if (access_type == MMU_INST_FETCH && !tic6x_addr_is_ram(cs, address)) {
        CPUTIC6XState *env = cpu_env(cs);

        if (probe) {
            return false;
        }
        env->excp_insn = 0;
        env->excp_mnem = ~0u;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tic6x: fetch from 0x%08x, which is not memory\n",
                      (uint32_t)address);
        cs->exception_index = TIC6X_EXCP_FETCH_ABORT;
        cpu_loop_exit_restore(cs, retaddr);
    }

    tlb_set_page(cs, address & TARGET_PAGE_MASK,
                 address & TARGET_PAGE_MASK,
                 PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                 mmu_idx, TARGET_PAGE_SIZE);
    return true;
}

static bool tic6x_cpu_has_work(CPUState *cs)
{
    return cs->interrupt_request & CPU_INTERRUPT_HARD;
}

static void tic6x_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    TIC6XCPUClass *tcc = TIC6X_CPU_GET_CLASS(obj);
    CPUTIC6XState *env = cpu_env(cs);
    ArchCPU *cpu = TIC6X_CPU(obj);

    if (tcc->parent_phases.hold) {
        tcc->parent_phases.hold(obj, type);
    }

    memset(env, 0, offsetof(CPUTIC6XState, end_reset_fields));

    /*
     * On this board the DSP does not fetch a reset vector of its own: the
     * main processor downloads a program over the host port, writes the
     * entry point as a single word at the base of L2 RAM and then releases
     * the core. The board reads that word and sets reset_pc, so the core
     * starts where the host said.
     */
    env->pc = cpu->reset_pc;

    /* Interrupts are globally disabled out of reset, SPRUFE8B 2.8.4. */
    env->cr[TIC6X_CR_CSR] = 0;
}

static void tic6x_cpu_disas_set_info(const CPUState *cs,
                                     disassemble_info *info)
{
    info->endian = BFD_ENDIAN_LITTLE;
    info->print_insn = NULL;
}

hwaddr tic6x_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr;
}

static const struct SysemuCPUOps tic6x_sysemu_ops = {
    .has_work = tic6x_cpu_has_work,
    .get_phys_page_debug = tic6x_cpu_get_phys_page_debug,
};

static const Property tic6x_cpu_properties[] = {
    DEFINE_PROP_UINT32("reset-pc", ArchCPU, reset_pc, 0),
};

static void tic6x_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    TIC6XCPUClass *tcc = TIC6X_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }

    qemu_init_vcpu(cs);
    cpu_reset(cs);

    tcc->parent_realize(dev, errp);
}

void tic6x_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUTIC6XState *env = cpu_env(cs);
    int i;

    qemu_fprintf(f, "PC  %08x  CSR %08x  IER %08x  IFR %08x  ISTP %08x\n",
                 env->pc, env->cr[TIC6X_CR_CSR], env->cr[TIC6X_CR_IER],
                 env->cr[TIC6X_CR_IFR], env->cr[TIC6X_CR_ISTP]);
    qemu_fprintf(f, "IRP %08x  NRP %08x  AMR %08x  branch %s -> %08x\n",
                 env->cr[TIC6X_CR_IRP], env->cr[TIC6X_CR_NRP],
                 env->cr[TIC6X_CR_AMR],
                 env->br_taken ? "pending" : "idle", env->br_target);
    for (i = 0; i < 32; i++) {
        qemu_fprintf(f, "A%-2d %08x%s", i, env->gpr[TIC6X_REG_A(i)],
                     (i % 4) == 3 ? "\n" : "  ");
    }
    for (i = 0; i < 32; i++) {
        qemu_fprintf(f, "B%-2d %08x%s", i, env->gpr[TIC6X_REG_B(i)],
                     (i % 4) == 3 ? "\n" : "  ");
    }
}

const char *tic6x_reg_name(unsigned int regno)
{
    static char buf[4][8];
    static int which;
    char *s;

    if (regno >= TIC6X_NUM_GPR) {
        return "??";
    }
    s = buf[which++ & 3];
    snprintf(s, 8, "%c%u", regno < 32 ? 'A' : 'B', regno & 31);
    return s;
}

static const VMStateDescription vmstate_tic6x_cpu = {
    .name = "cpu/tic6x",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(env.gpr, ArchCPU, TIC6X_NUM_GPR),
        VMSTATE_UINT32(env.pc, ArchCPU),
        VMSTATE_UINT32_ARRAY(env.cr, ArchCPU, TIC6X_NUM_CR),
        VMSTATE_UINT32(env.br_target, ArchCPU),
        VMSTATE_UINT32(env.br_taken, ArchCPU),
        VMSTATE_UINT32(env.br_cnt, ArchCPU),
        VMSTATE_END_OF_LIST()
    }
};

static const TCGCPUOps tic6x_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = tic6x_translate_init,
    .translate_code = tic6x_translate_code,
    .get_tb_cpu_state = tic6x_get_tb_cpu_state,
    .synchronize_from_tb = tic6x_cpu_synchronize_from_tb,
    .restore_state_to_opc = tic6x_restore_state_to_opc,
    .mmu_index = tic6x_cpu_mmu_index,
    .tlb_fill = tic6x_cpu_tlb_fill,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .cpu_exec_interrupt = tic6x_cpu_exec_interrupt,
    .cpu_exec_halt = tic6x_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = tic6x_cpu_do_interrupt,
};

static void tic6x_cpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    CPUClass *cc = CPU_CLASS(klass);
    TIC6XCPUClass *tcc = TIC6X_CPU_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, tic6x_cpu_properties);
    device_class_set_parent_realize(dc, tic6x_cpu_realizefn,
                                    &tcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, tic6x_cpu_reset_hold, NULL,
                                       &tcc->parent_phases);

    cc->dump_state = tic6x_cpu_dump_state;
    cc->set_pc = tic6x_cpu_set_pc;
    cc->get_pc = tic6x_cpu_get_pc;

    cc->sysemu_ops = &tic6x_sysemu_ops;
    cc->gdb_read_register = tic6x_cpu_gdb_read_register;
    cc->gdb_write_register = tic6x_cpu_gdb_write_register;
    cc->disas_set_info = tic6x_cpu_disas_set_info;

    dc->vmsd = &vmstate_tic6x_cpu;
    cc->gdb_core_xml_file = "tic6x-core.xml";
    cc->tcg_ops = &tic6x_tcg_ops;
}

static const TypeInfo tic6x_cpu_types[] = {
    {
        .name           = TYPE_TIC6X_CPU,
        .parent         = TYPE_CPU,
        .instance_size  = sizeof(TIC6XCPU),
        .instance_align = __alignof(TIC6XCPU),
        .abstract       = true,
        .class_size     = sizeof(TIC6XCPUClass),
        .class_init     = tic6x_cpu_class_init,
    },
    {
        .name           = TYPE_C6747_CPU,
        .parent         = TYPE_TIC6X_CPU,
    },
};

DEFINE_TYPES(tic6x_cpu_types)
