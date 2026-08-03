/*
 * Pioneer CDJ-2000NXS board (Renesas SH7764)
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A minimal board for the SH7764-based main processor of the Pioneer
 * CDJ-2000NXS DJ player. It provides NOR flash at physical 0, 128 MB of DDR at
 * physical 0x04000000, and the SH7764 SoC peripherals.
 *
 * No firmware is distributed with QEMU. Supply your own flash image with
 * -bios; it is loaded at physical address 0 and executed from the P2 reset
 * vector 0xa0000000, exactly as the hardware does.
 *
 * The memory map was established by executing a real boot ROM under an
 * instrumented SH-4A interpreter:
 *
 *   0x00000000  NOR flash, ~3 MB, reached at reset through P2 (0xa0000000)
 *   0x04000000  DDR, 128 MB. The boot ROM sizes it by probing 0x04000000,
 *               0x06000000, 0x07fff000, 0x08000000 and 0x0a000000, DMAs the
 *               compressed application to 0x07a00000, decompresses it to
 *               0x04000000, relocates its final stage to 0x0bffd000, and
 *               hands over at 0xa4000800. The initial stack pointer is
 *               0xac000000, i.e. the top of DDR, growing down.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/sh4/sh7764.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/reset.h"
#include "target/sh4/cpu.h"

/*
 * Area 3 external bus, still unidentified. Once the application is running it
 * both reads sequentially from 0x0c000000 (memory-like) and repeatedly DMAs
 * blocks to a *fixed* 0x0c080000 (port-like), so we deliberately do not claim
 * which it is. Backing it with RAM was tried and changed nothing, so it is not
 * the current blocker. Stubbed so the accesses show up under `-d unimp`.
 */
#define CDJ2KNXS_EXTBUS_BASE    0x0c000000
#define CDJ2KNXS_EXTBUS_SIZE    (64 * MiB)

#define CDJ2KNXS_FLASH_BASE     0x00000000
#define CDJ2KNXS_FLASH_SIZE     (4 * MiB)
#define CDJ2KNXS_DRAM_BASE      0x04000000
#define CDJ2KNXS_DRAM_SIZE      (128 * MiB)

/* SH-4 reset vector: the start of the P2 (uncached) window onto flash. */
#define CDJ2KNXS_RESET_VECTOR   0xa0000000

typedef struct ResetData {
    SuperHCPU *cpu;
    uint32_t vector;
} ResetData;

static void cdj2knxs_cpu_reset(void *opaque)
{
    ResetData *s = opaque;
    CPUSH4State *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->pc = s->vector;
}

static void cdj2knxs_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    ResetData *reset_info;
    SuperHCPU *cpu;
    DeviceState *soc;

    cpu = SUPERH_CPU(cpu_create(machine->cpu_type));

    reset_info = g_new0(ResetData, 1);
    reset_info->cpu = cpu;
    reset_info->vector = CDJ2KNXS_RESET_VECTOR;
    qemu_register_reset(cdj2knxs_cpu_reset, reset_info);

    memory_region_init_ram(dram, NULL, "cdj2knxs.dram",
                           CDJ2KNXS_DRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CDJ2KNXS_DRAM_BASE, dram);

    memory_region_init_rom(flash, NULL, "cdj2knxs.flash",
                           CDJ2KNXS_FLASH_SIZE, &error_fatal);
    /*
     * Unprogrammed NOR reads back as 0xff, not 0x00. This is not cosmetic:
     * the boot ROM DMAs a fixed 3.8 MB out of flash regardless of how long
     * the compressed image actually is, and the decompressor consumes the
     * padding past its end. Zero-filling here produces a different image and
     * the machine jumps to a bogus entry point.
     */
    memset(memory_region_get_ram_ptr(flash), 0xff, CDJ2KNXS_FLASH_SIZE);
    memory_region_add_subregion(sysmem, CDJ2KNXS_FLASH_BASE, flash);

    if (machine->firmware) {
        ssize_t size = load_image_mr(machine->firmware, flash);

        if (size < 0) {
            error_report("cdj2knxs: could not load flash image '%s'",
                         machine->firmware);
            exit(1);
        }
    } else {
        error_report("cdj2knxs: no flash image; supply one with -bios");
        exit(1);
    }

    create_unimplemented_device("cdj2knxs.area3", CDJ2KNXS_EXTBUS_BASE,
                                CDJ2KNXS_EXTBUS_SIZE);

    soc = qdev_new(TYPE_SH7764);
    object_property_set_link(OBJECT(soc), "cpu", OBJECT(cpu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(soc), &error_fatal);
}

static void cdj2knxs_machine_init(MachineClass *mc)
{
    mc->desc = "Pioneer CDJ-2000NXS (SH7764)";
    mc->init = cdj2knxs_init;
    mc->is_default = false;
    /*
     * Pioneer document the CDJ-2000NXS main processor as an
     * R5S77641N300BG, i.e. an SH77641 (SH7764 group) at 300 MHz.
     */
    mc->default_cpu_type = TYPE_SH7764_CPU;
    mc->default_ram_size = CDJ2KNXS_DRAM_SIZE;
    mc->no_parallel = true;
    mc->no_floppy = true;
    mc->no_cdrom = true;
}

DEFINE_MACHINE("cdj2knxs", cdj2knxs_machine_init)
