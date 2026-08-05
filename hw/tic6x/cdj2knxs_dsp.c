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
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
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

/*
 * Everything outside RAM, logged rather than silently reading zero.
 *
 * The point is to be told what the firmware wants. A model that answers every
 * address with zero looks like it is working right up until the firmware
 * waits for a status bit that will never set, and then there is nothing to
 * go on. This says which register, which access and from where, which is how
 * the SH7764 and BF531 peripheral sets were built - let the firmware ask, and
 * implement what it asks for.
 */
static uint64_t cdj2knxs_dsp_unimp_read(void *opaque, hwaddr off, unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;

    qemu_log_mask(LOG_UNIMP, "dsp: read  0x%08" HWADDR_PRIx " (%u bytes)\n",
                  base + off, size);
    return 0;
}

static void cdj2knxs_dsp_unimp_write(void *opaque, hwaddr off, uint64_t val,
                                     unsigned size)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;

    qemu_log_mask(LOG_UNIMP,
                  "dsp: write 0x%08" HWADDR_PRIx " = 0x%08" PRIx64
                  " (%u bytes)\n", base + off, val, size);
}

static const MemoryRegionOps cdj2knxs_dsp_unimp_ops = {
    .read = cdj2knxs_dsp_unimp_read,
    .write = cdj2knxs_dsp_unimp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void cdj2knxs_dsp_watch(MemoryRegion *sysmem, const char *name,
                               hwaddr base, uint64_t size)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);

    memory_region_init_io(mr, NULL, &cdj2knxs_dsp_unimp_ops,
                          (void *)(uintptr_t)base, name, size);
    memory_region_add_subregion_overlap(sysmem, base, mr, -1000);
}

/*
 * Hold the core until the main processor has written an entry point, then
 * start it there. The timer runs on the machine's clock, so it costs nothing
 * while the DSP is halted.
 */
typedef struct {
    ArchCPU *cpu;
    QEMUTimer *timer;
    int delay_ms;
    bool waited;
} CDJ2KNXSDSPStart;

static void cdj2knxs_dsp_poll_entry(void *opaque)
{
    CDJ2KNXSDSPStart *st = opaque;
    CPUState *cs = CPU(st->cpu);
    uint8_t word[4];
    uint32_t entry;

    cpu_physical_memory_read(CDJ_DSP_L2_BASE, word, sizeof(word));
    entry = ldl_le_p(word);

    if (entry >= CDJ_DSP_L2_BASE &&
        entry < CDJ_DSP_L2_BASE + CDJ_DSP_L2_SIZE) {
        /*
         * The host writes 41 blocks of 32 KB to 0x11837800 AFTER the DSPINT
         * that releases the core, so with shared memory the DSP races them.
         * Real hardware has the same race and the firmware presumably copes,
         * but CDJ_DSP_DELAY_MS holds the core back so the theory can be
         * tested rather than argued about.
         */
        if (st->delay_ms && !st->waited) {
            st->waited = true;
            timer_mod(st->timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + st->delay_ms);
            return;
        }
        cpu_env(cs)->pc = entry;
        cs->halted = 0;
        cpu_resume(cs);
        qemu_log_mask(LOG_UNIMP,
                      "dsp: released by the host, starting at 0x%08x\n", entry);
        timer_free(st->timer);
        g_free(st);
        return;
    }
    timer_mod(st->timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
}

static void cdj2knxs_dsp_wait_for_entry(ArchCPU *cpu)
{
    CDJ2KNXSDSPStart *st = g_new0(CDJ2KNXSDSPStart, 1);

    CPU(cpu)->halted = 1;
    st->cpu = cpu;
    st->delay_ms = getenv("CDJ_DSP_DELAY_MS")
                   ? atoi(getenv("CDJ_DSP_DELAY_MS")) : 0;
    st->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, cdj2knxs_dsp_poll_entry, st);
    timer_mod(st->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
}

static void cdj2knxs_dsp_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *l2 = g_new(MemoryRegion, 1);
    MemoryRegion *l1p = g_new(MemoryRegion, 1);
    MemoryRegion *l1d = g_new(MemoryRegion, 1);
    ArchCPU *cpu;
    uint32_t entry = 0;

    /*
     * L2 is where the two processors meet.
     *
     * On the board the main processor writes this memory through the DSP's
     * host port: it is a memory interface, not a wire, so the two machines
     * cannot be joined by a chardev the way the GUI processor is. QEMU
     * builds one target per binary, so they cannot be one process either.
     * What is left is to put L2 in a file and let both map it.
     *
     * Set CDJ_DSP_L2FILE to the same path on both sides. Without it this
     * falls back to private memory, which is what the standalone -bios mode
     * needs and is how the DSP was brought up.
     */
    if (getenv("CDJ_DSP_L2FILE")) {
        memory_region_init_ram_from_file(l2, NULL, "cdj2knxs-dsp.l2",
                                         CDJ_DSP_L2_SIZE, 0,
                                         RAM_SHARED, getenv("CDJ_DSP_L2FILE"),
                                         0, &error_fatal);
    } else {
        memory_region_init_ram(l2, NULL, "cdj2knxs-dsp.l2", CDJ_DSP_L2_SIZE,
                               &error_fatal);
    }
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

    /*
     * The peripheral windows, from the C6745/C6747 memory map in SPRS377.
     * None are modelled yet; each one logs what the firmware asks of it so
     * the set that actually matters can be built from evidence rather than
     * from reading the datasheet front to back.
     */
    cdj2knxs_dsp_watch(sysmem, "dsp.cfg0", 0x01c00000, 0x00200000);
    cdj2knxs_dsp_watch(sysmem, "dsp.cfg1", 0x01e00000, 0x00200000);
    cdj2knxs_dsp_watch(sysmem, "dsp.intc", 0x01800000, 0x00001000);
    cdj2knxs_dsp_watch(sysmem, "dsp.emifa", 0x68000000, 0x00008000);

    /*
     * Shared RAM and the EMIFB SDRAM, SPRS377: 128 KB at 0x80000000 and
     * 256 MB at 0xc0000000. Watched rather than backed with RAM, on purpose
     * and for now - the question being asked is whether the firmware touches
     * them at all, and giving them memory would answer it by hiding it.
     */
    cdj2knxs_dsp_watch(sysmem, "dsp.shram", 0x80000000, 0x00020000);
    cdj2knxs_dsp_watch(sysmem, "dsp.ddr", 0xc0000000, 0x10000000);

    cpu = TIC6X_CPU(object_new(machine->cpu_type));

    /*
     * With shared L2 there is no -bios and no entry point yet: the main
     * processor has not written one. Hold the core and watch the word at the
     * base of L2, which is the last thing the download writes and is
     * immediately followed by the DSPINT that lets the real core go. So that
     * word appearing IS the release, and it needs no side channel.
     *
     * Polling rather than a callback because the writer is another process
     * writing through a shared mapping; there is nothing to hook.
     */

    /*
     * The entry point is the word the host writes at the base of L2. Taking
     * it from memory rather than hard-coding it means the model follows the
     * firmware rather than a note about one particular build.
     */
    object_property_set_uint(OBJECT(cpu), "reset-pc", entry, &error_fatal);
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);

    if (getenv("CDJ_DSP_L2FILE") && !entry) {
        cdj2knxs_dsp_wait_for_entry(cpu);
    }
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
