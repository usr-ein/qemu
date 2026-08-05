/*
 * Pioneer CDJ-2000NXS MAIN DSP (IC301, D810K013CZKB400 = TMS320C6745/C6747)
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The DSP has no boot ROM path of its own on this board. The main processor
 * downloads a program over the host port into L2 RAM, writes the entry point
 * as a single word at the base of L2, and then releases the core. That was
 * measured, not assumed - CDJ_DSP_DUMP in hw/sh4/cdj2knxs.c writes out
 * everything the port received, and a boot produces:
 *
 *   0x11800000   one word, 0x11801da0   the entry point, written last
 *   0x11801da0   57,952 bytes           the program
 *   0x11837000   36,864 bytes           data
 *
 * So this machine's job is to present L2 RAM and start the core at whatever
 * the base word holds.
 *
 * The two processors share that memory rather than a wire, which is why this
 * is a separate QEMU process joined by a shared file rather than the chardev
 * arrangement the GUI processor uses: QEMU builds one target per binary, so
 * a C674x core cannot live inside qemu-system-sh4, and the host port is a
 * memory interface, not a serial one. Point both sides at the same file with
 * -global cdj2knxs-dsp.l2-file=... and the SH-4's host port model.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/reset.h"
#include "exec/cpu-common.h"
#include "system/memory.h"
#include "target/tic6x/cpu.h"

/*
 * The C6745/C6747 memory map, SPRS377. Only what the firmware uses is here;
 * peripherals come with task #28.
 */
#define CDJ_DSP_L2_BASE     0x11800000
#define CDJ_DSP_L2_SIZE     (256 * KiB)
#define CDJ_DSP_L1P_BASE    0x11e00000
#define CDJ_DSP_L1P_SIZE    (32 * KiB)
#define CDJ_DSP_L1D_BASE    0x11f00000
#define CDJ_DSP_L1D_SIZE    (32 * KiB)

static void cdj2knxs_dsp_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *l2 = g_new(MemoryRegion, 1);
    MemoryRegion *l1p = g_new(MemoryRegion, 1);
    MemoryRegion *l1d = g_new(MemoryRegion, 1);
    ArchCPU *cpu;
    uint32_t entry = 0;

    memory_region_init_ram(l2, NULL, "cdj2knxs-dsp.l2", CDJ_DSP_L2_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, CDJ_DSP_L2_BASE, l2);

    memory_region_init_ram(l1p, NULL, "cdj2knxs-dsp.l1p", CDJ_DSP_L1P_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, CDJ_DSP_L1P_BASE, l1p);

    memory_region_init_ram(l1d, NULL, "cdj2knxs-dsp.l1d", CDJ_DSP_L1D_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, CDJ_DSP_L1D_BASE, l1d);

    /*
     * Standalone use: -bios takes the image the host port would have
     * written, loaded at the address the download used. Without it the
     * machine waits with L2 zeroed, which is what the real part does until
     * the main processor has fed it.
     */
    if (machine->firmware) {
        ssize_t size = load_image_targphys(machine->firmware,
                                           CDJ_DSP_L2_BASE,
                                           CDJ_DSP_L2_SIZE, &error_fatal);
        char *head = NULL;
        gsize len = 0;

        if (size < 0) {
            error_report("could not load '%s'", machine->firmware);
            exit(1);
        }
        /*
         * Take the entry point from the file rather than from memory.
         * load_image_targphys stages the image as a ROM blob that is not
         * written into RAM until the machine resets, so reading L2 here
         * returns zeros and the core would start at address 0 - which it
         * duly did, walking forward through unmapped memory for as long as
         * it was left running.
         */
        if (g_file_get_contents(machine->firmware, &head, &len, NULL) &&
            len >= 4) {
            entry = ldl_le_p(head);
        }
        g_free(head);
    }

    cpu = TIC6X_CPU(object_new(machine->cpu_type));
    /*
     * The entry point is the word the host writes at the base of L2. Taking
     * it from memory rather than hard-coding it means the model follows the
     * firmware rather than a note about one particular build.
     */
    object_property_set_uint(OBJECT(cpu), "reset-pc", entry, &error_fatal);
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);
}

static void cdj2knxs_dsp_machine_init(MachineClass *mc)
{
    mc->desc = "Pioneer CDJ-2000NXS MAIN DSP (TMS320C6747)";
    mc->init = cdj2knxs_dsp_init;
    mc->default_cpu_type = TYPE_C6747_CPU;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

DEFINE_MACHINE("cdj2knxs-dsp", cdj2knxs_dsp_machine_init)
