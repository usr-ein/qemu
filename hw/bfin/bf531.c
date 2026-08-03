/*
 * Analog Devices ADSP-BF531 system on chip
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "hw/bfin/bf531.h"

/*
 * Core memory mapped registers. These live inside the core rather than on the
 * system bus, and the ones that matter for bringing firmware up are the event
 * vector table, the interrupt mask and latch, and the core timer, all of which
 * are CPU state. Routing them through a memory region keeps the CPU model free
 * of device code.
 */
static uint64_t bf531_core_mmr_read(void *opaque, hwaddr offset, unsigned size)
{
    BF531State *s = opaque;
    CPUBfinState *env = &s->cpu.env;
    hwaddr addr = BF531_CORE_MMR_BASE + offset;

    if (addr >= BF531_EVT0 && addr < BF531_EVT0 + BFIN_NUM_EVT * 4) {
        return env->evt[(addr - BF531_EVT0) / 4];
    }

    switch (addr) {
    case BF531_IMASK:
        return env->imask;
    case BF531_IPEND:
        return env->ipend;
    case BF531_ILAT:
        return env->ilat;
    case BF531_TCNTL:
        return env->tcntl;
    case BF531_TPERIOD:
        return env->tperiod;
    case BF531_TSCALE:
        return env->tscale;
    case BF531_TCOUNT:
        return env->tcount;
    case BF531_DMEM_CONTROL:
    case BF531_IMEM_CONTROL:
    case BF531_DTEST_COMMAND:
    case BF531_ITEST_COMMAND:
        /* Cache and L1 test control: accepted, no behaviour modelled. */
        return 0;
    }

    qemu_log_mask(LOG_UNIMP, "bf531: unmodelled core MMR read 0x%08" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

static void bf531_core_mmr_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    BF531State *s = opaque;
    CPUBfinState *env = &s->cpu.env;
    hwaddr addr = BF531_CORE_MMR_BASE + offset;

    if (addr >= BF531_EVT0 && addr < BF531_EVT0 + BFIN_NUM_EVT * 4) {
        env->evt[(addr - BF531_EVT0) / 4] = value;
        return;
    }

    switch (addr) {
    case BF531_IMASK:
        env->imask = value;
        if (env->ilat & env->imask) {
            cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
        }
        return;
    case BF531_IPEND:
        env->ipend = value;
        return;
    case BF531_ILAT:
        /* Writing a one clears a latched event. */
        env->ilat &= ~(uint32_t)value;
        return;
    case BF531_TCNTL:
        env->tcntl = value;
        return;
    case BF531_TPERIOD:
        env->tperiod = value;
        return;
    case BF531_TSCALE:
        env->tscale = value;
        return;
    case BF531_TCOUNT:
        env->tcount = value;
        return;
    case BF531_DMEM_CONTROL:
    case BF531_IMEM_CONTROL:
    case BF531_DTEST_COMMAND:
    case BF531_ITEST_COMMAND:
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "bf531: unmodelled core MMR write 0x%08" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", addr, value);
}

static const MemoryRegionOps bf531_core_mmr_ops = {
    .read = bf531_core_mmr_read,
    .write = bf531_core_mmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

static void bf531_realize(DeviceState *dev, Error **errp)
{
    BF531State *s = BF531(dev);
    MemoryRegion *sysmem = s->sysmem ?: get_system_memory();
    unsigned i;

    if (!qdev_realize(DEVICE(&s->cpu), NULL, errp)) {
        return;
    }

    memory_region_init_ram(&s->sdram, OBJECT(dev), "bf531.sdram",
                           s->sdram_size, &error_fatal);
    memory_region_add_subregion(sysmem, BF531_SDRAM_BASE, &s->sdram);

    memory_region_init_ram(&s->l1_data_a, OBJECT(dev), "bf531.l1-data-a",
                           BF531_L1_DATA_A_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BF531_L1_DATA_A_BASE, &s->l1_data_a);

    memory_region_init_ram(&s->l1_inst, OBJECT(dev), "bf531.l1-inst",
                           BF531_L1_INST_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BF531_L1_INST_BASE, &s->l1_inst);

    memory_region_init_ram(&s->l1_inst_c, OBJECT(dev), "bf531.l1-inst-cache",
                           BF531_L1_INST_C_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BF531_L1_INST_C_BASE, &s->l1_inst_c);

    memory_region_init_ram(&s->l1_scratch, OBJECT(dev), "bf531.l1-scratch",
                           BF531_L1_SCRATCH_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BF531_L1_SCRATCH_BASE,
                                &s->l1_scratch);

    memory_region_init_io(&s->core_mmr, OBJECT(dev), &bf531_core_mmr_ops, s,
                          "bf531.core-mmr", BF531_CORE_MMR_SIZE);
    memory_region_add_subregion(sysmem, BF531_CORE_MMR_BASE, &s->core_mmr);

    /*
     * The system peripherals are not modelled yet. Give each page the firmware
     * is known to touch its own catch-all so that -d unimp names the block
     * rather than reporting one anonymous 2 MB region, and back the whole MMR
     * window at lower priority so nothing faults.
     */
    {
        static const struct {
            hwaddr base;
            const char *name;
        } pages[] = {
            { BF531_SIC_BASE,    "bf531.sic" },
            { BF531_WDOG_BASE,   "bf531.wdog" },
            { BF531_RTC_BASE,    "bf531.rtc" },
            { BF531_UART_BASE,   "bf531.uart" },
            { BF531_SPI_BASE,    "bf531.spi" },
            { BF531_TIMER_BASE,  "bf531.timer" },
            { BF531_GPIO_BASE,   "bf531.gpio" },
            { BF531_SPORT0_BASE, "bf531.sport0" },
            { BF531_SPORT1_BASE, "bf531.sport1" },
            { BF531_EBIU_BASE,   "bf531.ebiu" },
            { BF531_DMA_TC_BASE, "bf531.dma-tc" },
        };

        for (i = 0; i < ARRAY_SIZE(pages); i++) {
            create_unimplemented_device(pages[i].name, pages[i].base,
                                        BF531_PERIPH_PAGE);
        }
        create_unimplemented_device("bf531.sys-mmr", BF531_SYS_MMR_BASE,
                                    BF531_SYS_MMR_SIZE);
    }

    /*
     * DMA and the PPI are modelled, because between them they are the panel:
     * the PPI is the parallel port that clocks pixels out, and a DMA channel
     * streams the frame buffer into it.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->dma), errp)) {
        return;
    }
    memory_region_add_subregion(sysmem, BF531_DMA_BASE,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dma),
                                                       0));

    object_property_set_link(OBJECT(&s->ppi), "dma", OBJECT(&s->dma),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ppi), errp)) {
        return;
    }
    memory_region_add_subregion(sysmem, BF531_PPI_BASE,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->ppi),
                                                       0));

    for (i = 0; i < BF531_ASYNC_BANKS; i++) {
        g_autofree char *name = g_strdup_printf("bf531.async-bank%u", i);

        hwaddr base = BF531_ASYNC_BASE + i * BF531_ASYNC_BANK_SIZE;

        create_unimplemented_device(name, base, BF531_ASYNC_BANK_SIZE);
    }
}

static void bf531_init(Object *obj)
{
    BF531State *s = BF531(obj);

    object_initialize_child(obj, "cpu", &s->cpu, TYPE_BF531_CPU);
    object_initialize_child(obj, "dma", &s->dma, TYPE_BFIN_DMA);
    object_initialize_child(obj, "ppi", &s->ppi, TYPE_BFIN_PPI);
}

static const Property bf531_properties[] = {
    DEFINE_PROP_UINT64("sdram-size", BF531State, sdram_size, 32 * MiB),
};

static void bf531_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bf531_realize;
    device_class_set_props(dc, bf531_properties);
    /* The SoC is not user-creatable: a board wires it up. */
    dc->user_creatable = false;
}

static const TypeInfo bf531_types[] = {
    {
        .name          = TYPE_BF531,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(BF531State),
        .instance_init = bf531_init,
        .class_init    = bf531_class_init,
    },
};

DEFINE_TYPES(bf531_types)
