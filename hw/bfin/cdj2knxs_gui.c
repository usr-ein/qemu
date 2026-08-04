/*
 * Pioneer CDJ-2000NXS GUI processor board
 *
 * Copyright (c) 2026 cdj2knxs contributors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The CDJ-2000NXS draws its main colour LCD on a separate ADSP-BF531 running
 * ThreadX. This board runs that processor's firmware, which the vendor ships
 * as segment 0 of the update file in Blackfin LDR boot stream form.
 *
 * On hardware the on-chip boot ROM reads the LDR over SPI and then enters L1
 * instruction SRAM. This board does the same block processing directly, which
 * avoids needing a boot ROM image and makes the load visible to -d unimp.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "hw/bfin/bf531.h"

#define CDJ2KNXS_GUI_SDRAM_SIZE   (32 * MiB)
#define CDJ2KNXS_GUI_PANEL_LINES  234

/*
 * Pioneer prefix each update segment with a 32-byte ASCII banner, for example
 * "CDJ-2000NXS GUI Ver1.200 0234561". The LDR stream starts after it.
 */
#define VENDOR_BANNER_SIZE 32

/* Block flags, from the VisualDSP++ loader documentation. */
#define BFLAG_ZEROFILL 0x0001
#define BFLAG_INIT     0x0008
#define BFLAG_IGNORE   0x0010
#define BFLAG_FINAL    0x8000

#define LDR_HEADER_SIZE 10

static bool looks_like_banner(const uint8_t *buf, size_t len)
{
    size_t i;

    if (len < VENDOR_BANNER_SIZE) {
        return false;
    }
    for (i = 0; i < 16; i++) {
        if (buf[i] < 0x20 || buf[i] > 0x7e) {
            return false;
        }
    }
    return true;
}

/*
 * Walk the boot stream and place each block. Returns the number of blocks
 * processed, or a negative value if the stream is malformed.
 */
static int cdj2knxs_gui_load_ldr(const uint8_t *buf, size_t len)
{
    size_t off = looks_like_banner(buf, len) ? VENDOR_BANNER_SIZE : 0;
    int blocks = 0;

    while (off + LDR_HEADER_SIZE <= len) {
        uint32_t addr = ldl_le_p(buf + off);
        uint32_t count = ldl_le_p(buf + off + 4);
        uint16_t flags = lduw_le_p(buf + off + 8);

        off += LDR_HEADER_SIZE;
        blocks++;

        if (flags & BFLAG_ZEROFILL) {
            g_autofree uint8_t *zero = g_malloc0(count);

            if (!(flags & BFLAG_IGNORE) && count) {
                cpu_physical_memory_write(addr, zero, count);
            }
        } else {
            if (off + count > len) {
                error_report("cdj2knxs-gui: block %d runs past end of file "
                             "(want %u bytes at offset %zu of %zu)",
                             blocks, count, off, len);
                return -1;
            }
            if (!(flags & BFLAG_IGNORE) && count) {
                cpu_physical_memory_write(addr, buf + off, count);
            }
            off += count;
        }

        if (flags & BFLAG_FINAL) {
            return blocks;
        }
    }

    error_report("cdj2knxs-gui: boot stream ended without a final block");
    return -1;
}

static void cdj2knxs_gui_cpu_reset(void *opaque)
{
    BF531State *soc = opaque;

    cpu_reset(CPU(&soc->cpu));
    /*
     * The boot ROM enters L1 instruction SRAM once the stream is loaded. On
     * the BF531 that is 0xFFA08000, which is where this firmware's final two
     * blocks land.
     */
    soc->cpu.env.pc = BF531_L1_INST_BASE;
}

static void cdj2knxs_gui_init(MachineState *machine)
{
    BF531State *soc;
    const char *fw = machine->firmware ?: machine->kernel_filename;
    g_autofree uint8_t *buf = NULL;
    gsize len;
    int blocks;

    soc = BF531(object_new(TYPE_BF531));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_property_set_uint(OBJECT(soc), "sdram-size", machine->ram_size,
                             &error_fatal);
    /*
     * The panel is the 6.1 inch wide TFT of section 6.2 of the service
     * manual, 480 by 234. The firmware sends 255 lines per frame - timer 2,
     * which drives the vertical sync from the pixel clock, has a period of
     * exactly 255 line times and a pulse 21 lines wide - so the top 21 lines
     * of every frame are blanking and never reach the glass.
     */
    object_property_set_uint(OBJECT(soc), "panel-lines",
                             CDJ2KNXS_GUI_PANEL_LINES, &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(soc), &error_fatal);

    if (!fw) {
        error_report("cdj2knxs-gui: no firmware given; use -bios with the "
                     "GUI LDR extracted from your own update file");
        exit(1);
    }

    if (!g_file_get_contents(fw, (gchar **)&buf, &len, NULL)) {
        error_report("cdj2knxs-gui: cannot read '%s'", fw);
        exit(1);
    }

    /*
     * The image is what is programmed into the GUI processor's own flash,
     * DYW1815 at IC4004 on the TFTA assembly, which sits on the Blackfin's
     * asynchronous memory bus. The boot stream is only the first part of it:
     * past the final block the same image carries the fonts and bitmaps, and
     * the firmware reads those straight out of flash while it runs. So place
     * the whole file in asynchronous memory as well as processing its blocks.
     */
    cpu_physical_memory_write(BF531_ASYNC_BASE, buf, len);

    blocks = cdj2knxs_gui_load_ldr(buf, len);
    if (blocks < 0) {
        exit(1);
    }

    qemu_register_reset(cdj2knxs_gui_cpu_reset, soc);
}

static void cdj2knxs_gui_machine_init(MachineClass *mc)
{
    mc->desc = "Pioneer CDJ-2000NXS GUI processor (ADSP-BF531)";
    mc->init = cdj2knxs_gui_init;
    mc->default_cpu_type = TYPE_BF531_CPU;
    mc->default_ram_size = CDJ2KNXS_GUI_SDRAM_SIZE;
    /*
     * No default_ram_id: the SoC allocates SDRAM itself, sized from the
     * machine's ram_size, so letting the machine pre-create a block of the
     * same name would collide.
     */
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

DEFINE_MACHINE("cdj2knxs-gui", cdj2knxs_gui_machine_init)
