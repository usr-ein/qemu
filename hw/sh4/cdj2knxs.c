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
#include "qemu/log.h"
#include "target/sh4/cpu.h"

/*
 * Area 3. The SH7764 address map gives area 3 to 64 MB of SRAM, and the
 * firmware treats it that way: once running it reads all 8388608 eight-byte
 * words of the window in sequence, which is a memory scan, not a device
 * probe. It also DMAs blocks to a fixed 0x0c080000, presumably a mailbox for
 * the separate GUI processor, but that lands in the same memory.
 */

/*
 * The MAIN DSP, IC301 (D810K013CZKB400), through its host port.
 *
 * Schematic sheet 10.5 shows the part with three blocks - EMIFA labelled
 * "U-HPI", EMIF-B, and power - and it is the U-HPI that faces the main
 * processor: sixteen data lines CPU_DATA0D to CPU_DATA15D, address lines into
 * UHPI_HCNTL0, UHPI_HCNTL1, UHPI_HAS and UHPI_HHWIL, UHPI_HCS off
 * CPU_DSP_ENABLE, and UHPI_HRDY coming back as DSP_RDY. That is a Texas
 * Instruments style host port, four registers selected by HCNTL.
 *
 * The SH7764 only has two normal-space areas, CS0 and CS3, and CS0 is the
 * flash, so the DSP is in area 3. Which four addresses it answers on was
 * measured rather than read off the schematic - a logging overlay on area 3
 * showed the firmware touching exactly four, a quarter of a megabyte apart,
 * so HCNTL is wired to address bits 19 and 18:
 *
 *     0x0C000000   34 reads, 2 writes of 0x4 and 0x00010001   HPIC
 *     0x0C040000   one write of 0x11801DA0                    HPIA
 *     0x0C080000   3997 writes                                HPID, download
 *     0x0C0C0000   3966 reads                                 HPID, verify
 *
 * The 0x00010001 is HPIC's giveaway: the register is sixteen bits mirrored
 * into both halves of the word. The rest is a program download - set the
 * address once, write the image, then read it back and check it. Backed by
 * plain RAM the readback returned zeroes, the check failed, and the firmware
 * declared the DSP dead: "Downloading of programs is not possible" in section
 * [12-2] of the service manual, E-7010 on the screen.
 *
 * This models the port, not the DSP: an address register, a data port that
 * auto-increments it, and memory behind them, so a download reads back as
 * what was written. Nothing executes it. That is enough to satisfy the check
 * and is honest about being a stub - a player emulated this way will not make
 * a sound, which for the purpose here is fine.
 */
#define CDJ2KNXS_DSP_BASE       0x0c000000
#define CDJ2KNXS_DSP_WINDOW     0x00100000
#define CDJ2KNXS_DSP_STRIDE     0x00040000
#define CDJ2KNXS_DSP_MEM        (16 * MiB)

/*
 * HPIC, as the C6000 host port defines it. The host sets DSPINT to interrupt
 * the DSP; a running DSP takes the interrupt, which clears DSPINT, and
 * answers by raising HINT, which the host clears by writing a one back. A
 * stub that only stores the bits never answers, and the firmware is left
 * waiting for a DSP that appears not to be running.
 */
#define CDJ2KNXS_HPIC_HWOB      0x0001
#define CDJ2KNXS_HPIC_DSPINT    0x0002      /* host to DSP, DSP clears it   */
#define CDJ2KNXS_HPIC_HINT      0x0004      /* DSP to host, host clears it  */
#define CDJ2KNXS_HPIC_HRDY      0x0008      /* the port is ready for a word */

typedef struct CDJ2KNXSDSP {
    MemoryRegion iomem;
    uint32_t hpic;
    uint32_t hpia;
    uint32_t words;
    uint32_t mbox_log;
    uint32_t mbox_armed;
    uint8_t *mem;
} CDJ2KNXSDSP;

static uint64_t cdj2knxs_dsp_read(void *opaque, hwaddr off, unsigned size)
{
    CDJ2KNXSDSP *s = opaque;
    uint32_t v = 0;

    switch (off / CDJ2KNXS_DSP_STRIDE) {
    case 0:                             /* HPIC, mirrored, always ready */
        v = (s->hpic | CDJ2KNXS_HPIC_HRDY) & 0xffff;
        if (getenv("CDJ_DSP_TRACE")) {
            qemu_log("dsp: read HPIC = 0x%08x\n", v | (v << 16));
        }
        return v | (v << 16);
    case 1:
        return s->hpia;
    default:                            /* HPID, either window */
        memcpy(&v, s->mem + (s->hpia & (CDJ2KNXS_DSP_MEM - 1)), 4);
        if (s->mbox_armed && getenv("CDJ_DSP_MBOX") && s->mbox_log++ < 200) {
            qemu_log("mbox: read  0x%08x = 0x%08x\n", s->hpia, v);
        }
        if (s->mbox_armed) {
            s->mbox_armed--;
        }
        s->hpia += 4;
        s->words++;
        return v;
    }
}

static void cdj2knxs_dsp_write(void *opaque, hwaddr off, uint64_t value,
                               unsigned size)
{
    CDJ2KNXSDSP *s = opaque;
    uint32_t v = value;

    switch (off / CDJ2KNXS_DSP_STRIDE) {
    case 0:
        if (getenv("CDJ_DSP_TRACE")) {
            qemu_log("dsp: write HPIC = 0x%08x\n", v);
        }
        s->hpic = (s->hpic & ~CDJ2KNXS_HPIC_HWOB) | (v & CDJ2KNXS_HPIC_HWOB);
        if (v & CDJ2KNXS_HPIC_HINT) {
            s->hpic &= ~CDJ2KNXS_HPIC_HINT;     /* host acknowledges */
        }
        if (v & CDJ2KNXS_HPIC_DSPINT) {
            /* Taken and answered at once: nothing here runs the DSP. */
            s->hpic &= ~CDJ2KNXS_HPIC_DSPINT;
            s->hpic |= CDJ2KNXS_HPIC_HINT;
        }
        return;
    case 1:
        if (getenv("CDJ_DSP_TRACE")) {
            qemu_log("dsp: write HPIA = 0x%08x (after %u data words)\n",
                     v, s->words);
            s->words = 0;
        }
        s->hpia = v;
        /* A short burst after a mailbox address is the poll, not the image. */
        if (v >= 0x11837b80 && v < 0x11837c00) {
            s->mbox_armed = 3;
        }
        return;
    default:
        if (s->mbox_armed && getenv("CDJ_DSP_MBOX") && s->mbox_log++ < 200) {
            qemu_log("mbox: write 0x%08x = 0x%08x\n", s->hpia, v);
        }
        if (s->mbox_armed) {
            s->mbox_armed--;
        }
        memcpy(s->mem + (s->hpia & (CDJ2KNXS_DSP_MEM - 1)), &v, 4);
        s->hpia += 4;
        s->words++;
        return;
    }
}

static const MemoryRegionOps cdj2knxs_dsp_ops = {
    .read = cdj2knxs_dsp_read,
    .write = cdj2knxs_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void cdj2knxs_dsp_init(MemoryRegion *sysmem)
{
    CDJ2KNXSDSP *s = g_new0(CDJ2KNXSDSP, 1);

    s->mem = g_malloc0(CDJ2KNXS_DSP_MEM);
    memory_region_init_io(&s->iomem, NULL, &cdj2knxs_dsp_ops, s,
                          "cdj2knxs.dsp-hpi", CDJ2KNXS_DSP_WINDOW);
    memory_region_add_subregion_overlap(sysmem, CDJ2KNXS_DSP_BASE,
                                        &s->iomem, 1);
}

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
    MemoryRegion *area3 = g_new(MemoryRegion, 1);
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

    memory_region_init_ram(area3, NULL, "cdj2knxs.area3",
                           CDJ2KNXS_EXTBUS_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CDJ2KNXS_EXTBUS_BASE, area3);
    cdj2knxs_dsp_init(sysmem);

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
