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
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "hw/bfin/bf531.h"
#include "qemu/timer.h"

/*
 * Core memory mapped registers. These live inside the core rather than on the
 * system bus, and the ones that matter for bringing firmware up are the event
 * vector table, the interrupt mask and latch, and the core timer, all of which
 * are CPU state. Routing them through a memory region keeps the CPU model free
 * of device code.
 */
#define BF531_TCNTL_TMPWR    (1u << 0)
#define BF531_TCNTL_TMREN    (1u << 1)
#define BF531_TCNTL_TAUTORLD (1u << 2)
#define BF531_TCNTL_TINT     (1u << 3)

/*
 * The core timer decrements TCOUNT once every TSCALE + 1 core clocks and
 * raises IVTMR when it reaches zero, reloading from TPERIOD if TAUTORLD is
 * set. This is the tick the firmware's kernel schedules on, so without it
 * nothing beyond the initial thread ever runs.
 */
static void bf531_core_timer_update(BF531State *s)
{
    CPUBfinState *env = &s->cpu.env;
    uint64_t ticks, ns;

    if (!(env->tcntl & BF531_TCNTL_TMPWR) ||
        !(env->tcntl & BF531_TCNTL_TMREN) || env->tcount == 0) {
        timer_del(s->core_timer);
        return;
    }

    ticks = (uint64_t)env->tcount * (env->tscale + 1);
    ns = muldiv64(ticks, NANOSECONDS_PER_SECOND, s->cclk_hz);
    s->core_timer_next = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns;
    timer_mod(s->core_timer, s->core_timer_next);
}

static void bf531_core_timer_expire(void *opaque)
{
    BF531State *s = opaque;
    CPUBfinState *env = &s->cpu.env;

    env->tcntl |= BF531_TCNTL_TINT;

    if (env->tcntl & BF531_TCNTL_TAUTORLD) {
        env->tcount = env->tperiod;
    } else {
        env->tcount = 0;
    }

    env->ilat |= 1u << BFIN_EXCP_IVTMR;
    if (env->ilat & env->imask) {
        cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
    }

    bf531_core_timer_update(s);
}

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
        /* TINT is sticky and cleared by writing a one to it. */
        if (value & BF531_TCNTL_TINT) {
            env->tcntl &= ~BF531_TCNTL_TINT;
        }
        env->tcntl = (env->tcntl & BF531_TCNTL_TINT) |
                     (value & ~BF531_TCNTL_TINT);
        bf531_core_timer_update(s);
        return;
    case BF531_TPERIOD:
        env->tperiod = value;
        return;
    case BF531_TSCALE:
        env->tscale = value;
        return;
    case BF531_TCOUNT:
        env->tcount = value;
        bf531_core_timer_update(s);
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

/*
 * System interrupt controller, chapter 4.
 *
 * The core has nine general purpose interrupt levels and the part has far more
 * peripherals than that, so the SIC sits between them: SIC_ISR latches each
 * peripheral source, SIC_IMASK selects which ones are allowed through, and
 * SIC_IAR gives each source a four bit level, counted from IVG7. Several
 * sources can share a level, which is why the assignment registers exist and
 * why the handler has to ask the peripherals who interrupted.
 *
 * The lines are level sensitive, so a source that goes away has to take its
 * core latch bit with it - but only its own. The firmware raises IVG15 in
 * software for its kernel level, and clearing that from here would lose it.
 */
static unsigned bf531_sic_level(BF531State *s, unsigned id)
{
    unsigned word = id / 8;
    unsigned nibble = (id % 8) * 4;

    return 7 + ((s->sic_iar[word] >> nibble) & 0xf);
}

static void bf531_sic_update(BF531State *s)
{
    CPUBfinState *env = &s->cpu.env;
    uint32_t active = s->sic_isr & s->sic_imask;
    uint32_t levels = 0;
    unsigned id;

    for (id = 0; id < BF531_SIC_SOURCES; id++) {
        if (active & (1u << id)) {
            levels |= 1u << bf531_sic_level(s, id);
        }
    }

    env->ilat &= ~(s->sic_levels & ~levels);
    env->ilat |= levels;
    s->sic_levels = levels;

    if (env->ilat & env->imask) {
        cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
    }
}

static void bf531_sic_set_irq(void *opaque, int id, int level)
{
    BF531State *s = opaque;

    if (level) {
        s->sic_isr |= 1u << id;
    } else {
        s->sic_isr &= ~(1u << id);
    }
    bf531_sic_update(s);
}

static uint64_t bf531_sic_read(void *opaque, hwaddr offset, unsigned size)
{
    BF531State *s = opaque;

    switch (offset) {
    case BF531_SWRST:
        return s->swrst;
    case BF531_SYSCR:
        return s->syscr;
    case BF531_SIC_RVECT:
        return s->cpu.env.evt[BFIN_EXCP_IVG15];
    case BF531_SIC_IMASK:
        return s->sic_imask;
    case BF531_SIC_IAR0:
    case BF531_SIC_IAR1:
    case BF531_SIC_IAR2:
        return s->sic_iar[(offset - BF531_SIC_IAR0) / 4];
    case BF531_SIC_ISR:
        return s->sic_isr;
    case BF531_SIC_IWR:
        return s->sic_iwr;
    }

    qemu_log_mask(LOG_UNIMP, "bf531-sic: read from 0x%02" HWADDR_PRIx "\n",
                  offset);
    return 0;
}

static void bf531_sic_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    BF531State *s = opaque;

    switch (offset) {
    case BF531_SWRST:
        s->swrst = value;
        return;
    case BF531_SYSCR:
        s->syscr = value;
        return;
    case BF531_SIC_IMASK:
        s->sic_imask = value;
        bf531_sic_update(s);
        return;
    case BF531_SIC_IAR0:
    case BF531_SIC_IAR1:
    case BF531_SIC_IAR2:
        s->sic_iar[(offset - BF531_SIC_IAR0) / 4] = value;
        bf531_sic_update(s);
        return;
    case BF531_SIC_IWR:
        s->sic_iwr = value;
        return;
    case BF531_SIC_ISR:
        /* Read only: a source is cleared at the peripheral that raised it. */
        return;
    }

    qemu_log_mask(LOG_UNIMP, "bf531-sic: write to 0x%02" HWADDR_PRIx
                  " = 0x%" PRIx64 "\n", offset, value);
}

static const MemoryRegionOps bf531_sic_ops = {
    .read = bf531_sic_read,
    .write = bf531_sic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

/*
 * SPI, chapter 10. The Blackfin boots from SPI flash and the firmware keeps
 * using the port afterwards, polling SPI_STAT between words. Nothing is
 * attached in this model, so transfers are reported as having completed the
 * instant they are started: without that the firmware spins on SPI_STAT
 * forever and never reaches its own initialisation.
 */
#define BF531_SPI_CTL    0x00
#define BF531_SPI_FLG    0x04
#define BF531_SPI_STAT   0x08
#define BF531_SPI_TDBR   0x0c
#define BF531_SPI_RDBR   0x10
#define BF531_SPI_BAUD   0x14
#define BF531_SPI_SHADOW 0x18

#define BF531_SPI_STAT_SPIF (1u << 0)
#define BF531_SPI_STAT_TXS  (1u << 3)
#define BF531_SPI_STAT_RXS  (1u << 5)

static uint64_t bf531_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    BF531State *s = opaque;

    switch (offset) {
    case BF531_SPI_CTL:
        return s->spi_ctl;
    case BF531_SPI_FLG:
        return s->spi_flg;
    case BF531_SPI_STAT:
        /*
         * Always finished, transmit buffer drained, receive buffer holding
         * the word that a transfer with nothing on the bus produces.
         */
        return BF531_SPI_STAT_SPIF | BF531_SPI_STAT_RXS;
    case BF531_SPI_RDBR:
    case BF531_SPI_SHADOW:
        /* An idle bus floats high, which is what an absent device reads as. */
        return 0xffff;
    case BF531_SPI_TDBR:
        return s->spi_tdbr;
    case BF531_SPI_BAUD:
        return s->spi_baud;
    }
    return 0;
}

static void bf531_spi_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    BF531State *s = opaque;

    switch (offset) {
    case BF531_SPI_CTL:
        s->spi_ctl = value;
        return;
    case BF531_SPI_FLG:
        s->spi_flg = value;
        return;
    case BF531_SPI_TDBR:
        s->spi_tdbr = value;
        return;
    case BF531_SPI_BAUD:
        s->spi_baud = value;
        return;
    case BF531_SPI_STAT:
        /* The error bits are sticky and write-one-to-clear. */
        return;
    }
}

static const MemoryRegionOps bf531_spi_ops = {
    .read = bf531_spi_read,
    .write = bf531_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

/*
 * Port F, chapter 14. Pins configured as outputs read back what was driven;
 * pins configured as inputs read the value on the pin. Boards strap those to
 * identify fitted options, so the value an input reads is a property.
 */
#define BF531_FIO_FLAG_D 0x00
#define BF531_FIO_FLAG_C 0x04
#define BF531_FIO_FLAG_S 0x08
#define BF531_FIO_FLAG_T 0x0c
#define BF531_FIO_DIR    0x30
#define BF531_FIO_INEN   0x40

static uint64_t bf531_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    BF531State *s = opaque;

    switch (offset) {
    case BF531_FIO_FLAG_D:
    case BF531_FIO_FLAG_C:
    case BF531_FIO_FLAG_S:
    case BF531_FIO_FLAG_T:
        return (s->gpio_out & s->gpio_dir) | (s->gpio_in & ~s->gpio_dir);
    case BF531_FIO_DIR:
        return s->gpio_dir;
    case BF531_FIO_INEN:
        return s->gpio_inen;
    }
    return 0;
}

static void bf531_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    BF531State *s = opaque;

    switch (offset) {
    case BF531_FIO_FLAG_D:
    case BF531_FIO_FLAG_T:
        s->gpio_out = value;
        return;
    case BF531_FIO_FLAG_C:
        s->gpio_out &= ~(uint16_t)value;
        return;
    case BF531_FIO_FLAG_S:
        s->gpio_out |= (uint16_t)value;
        return;
    case BF531_FIO_DIR:
        s->gpio_dir = value;
        return;
    case BF531_FIO_INEN:
        s->gpio_inen = value;
        return;
    }
}

static const MemoryRegionOps bf531_gpio_ops = {
    .read = bf531_gpio_read,
    .write = bf531_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
};

/*
 * A debugging overlay. Placed over a range of SDRAM at higher priority, it
 * logs every access and then performs it against the memory underneath, which
 * makes it possible to find out what writes a particular location - something
 * neither the monitor nor -d can do, because there are no guest watchpoints.
 * Off unless trace-size is set.
 */
static uint8_t *bf531_trace_host(BF531State *s, hwaddr offset)
{
    return (uint8_t *)memory_region_get_ram_ptr(&s->sdram)
           + s->trace_base + offset;
}

static uint64_t bf531_trace_read(void *opaque, hwaddr offset, unsigned size)
{
    BF531State *s = opaque;

    return ldn_le_p(bf531_trace_host(s, offset), size);
}

static void bf531_trace_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BF531State *s = opaque;

    qemu_log_mask(LOG_UNIMP,
                  "TRACE %08x <= %0*" PRIx64 " size %u pc %08x rets %08x\n",
                  (uint32_t)(s->trace_base + offset), size * 2, value, size,
                  s->cpu.env.pc, s->cpu.env.rets);
    stn_le_p(bf531_trace_host(s, offset), size, value);
}

static const MemoryRegionOps bf531_trace_ops = {
    .read = bf531_trace_read,
    .write = bf531_trace_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
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

    s->core_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 bf531_core_timer_expire, s);

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
            { BF531_WDOG_BASE,   "bf531.wdog" },
            { BF531_RTC_BASE,    "bf531.rtc" },
            { BF531_UART_BASE,   "bf531.uart" },
            { BF531_TIMER_BASE,  "bf531.timer" },
            { BF531_SPORT0_BASE, "bf531.sport0" },
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

    if (s->trace_size) {
        memory_region_init_io(&s->trace, OBJECT(dev), &bf531_trace_ops, s,
                              "bf531.trace", s->trace_size);
        memory_region_add_subregion_overlap(sysmem, s->trace_base,
                                            &s->trace, 1);
    }

    /*
     * DMA and the PPI are modelled, because between them they are the panel:
     * the PPI is the parallel port that clocks pixels out, and a DMA channel
     * streams the frame buffer into it.
     */
    memory_region_init_io(&s->sic, OBJECT(dev), &bf531_sic_ops, s,
                          "bf531.sic", BF531_PERIPH_PAGE);
    memory_region_add_subregion(sysmem, BF531_SIC_BASE, &s->sic);
    s->sic_in = qemu_allocate_irqs(bf531_sic_set_irq, s, BF531_SIC_SOURCES);

    memory_region_init_io(&s->gpio, OBJECT(dev), &bf531_gpio_ops, s,
                          "bf531.gpio", BF531_PERIPH_PAGE);
    memory_region_add_subregion(sysmem, BF531_GPIO_BASE, &s->gpio);

    memory_region_init_io(&s->spi, OBJECT(dev), &bf531_spi_ops, s,
                          "bf531.spi", BF531_PERIPH_PAGE);
    memory_region_add_subregion(sysmem, BF531_SPI_BASE, &s->spi);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->dma), errp)) {
        return;
    }
    memory_region_add_subregion(sysmem, BF531_DMA_BASE,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dma),
                                                       0));
    /* Peripheral DMA channels 0 to 7 are SIC sources 8 to 15, table 4-4. */
    for (i = 0; i < BFIN_DMA_CHANNELS; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->dma), i,
                           s->sic_in[BF531_SIC_ID_DMA0 + i]);
    }

    /*
     * SPORT1 is the link to the main processor. Its two DMA channels are 3
     * for receive and 4 for transmit, which is what the GUI firmware
     * programs and what SIC bits 11 and 12 report.
     */
    object_property_set_link(OBJECT(&s->sport1), "dma", OBJECT(&s->dma),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sport1), errp)) {
        return;
    }
    memory_region_add_subregion(sysmem, BF531_SPORT1_BASE,
                                sysbus_mmio_get_region(
                                    SYS_BUS_DEVICE(&s->sport1), 0));

    object_property_set_link(OBJECT(&s->ppi), "dma", OBJECT(&s->dma),
                             &error_abort);
    object_property_set_uint(OBJECT(&s->ppi), "panel-lines", s->panel_lines,
                             &error_abort);
    if (s->panel_hz) {
        object_property_set_uint(OBJECT(&s->ppi), "refresh-hz", s->panel_hz,
                                 &error_abort);
    }
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ppi), errp)) {
        return;
    }
    memory_region_add_subregion(sysmem, BF531_PPI_BASE,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->ppi),
                                                       0));

    /*
     * The four asynchronous memory banks. On this board they carry the flash
     * the GUI firmware reads its fonts and bitmaps from, so back them with
     * memory rather than leaving them unimplemented; a board that has the
     * flash contents can load them here.
     */
    memory_region_init_ram(&s->async, OBJECT(dev), "bf531.async",
                           BF531_ASYNC_BANKS * BF531_ASYNC_BANK_SIZE,
                           &error_fatal);
    /* Erased NOR reads as all ones, which is not what fresh RAM contains. */
    memset(memory_region_get_ram_ptr(&s->async), 0xff,
           BF531_ASYNC_BANKS * BF531_ASYNC_BANK_SIZE);
    memory_region_add_subregion(sysmem, BF531_ASYNC_BASE, &s->async);
}

static void bf531_init(Object *obj)
{
    BF531State *s = BF531(obj);

    object_initialize_child(obj, "cpu", &s->cpu, TYPE_BF531_CPU);
    object_initialize_child(obj, "dma", &s->dma, TYPE_BFIN_DMA);
    object_initialize_child(obj, "ppi", &s->ppi, TYPE_BFIN_PPI);
    object_initialize_child(obj, "sport1", &s->sport1, TYPE_BFIN_SPORT);
}

static const Property bf531_properties[] = {
    DEFINE_PROP_UINT64("sdram-size", BF531State, sdram_size, 32 * MiB),
    /* What port F reads on pins configured as inputs. */
    DEFINE_PROP_UINT16("gpio-in", BF531State, gpio_in, 0),
    /* ADSP-BF531SBSTZ400: the core clock is 400 MHz. */
    DEFINE_PROP_UINT32("cclk-hz", BF531State, cclk_hz, 400000000),
    DEFINE_PROP_UINT32("panel-lines", BF531State, panel_lines, 0),
    DEFINE_PROP_UINT32("panel-hz", BF531State, panel_hz, 0),
    DEFINE_PROP_UINT32("trace-base", BF531State, trace_base, 0),
    DEFINE_PROP_UINT32("trace-size", BF531State, trace_size, 0),
};

/*
 * The interrupt assignment registers do not reset to zero. Figures 4-9 to
 * 4-11 give them a default spread of the peripherals across the general
 * purpose levels, and firmware that is content with that spread never writes
 * them - this one does not. Resetting them to zero instead puts every
 * peripheral on IVG7, where the frame interrupt arrives at whatever handler
 * happens to own the lowest level and the display never advances.
 */
#define BF531_SIC_IAR0_RESET 0x10000000
#define BF531_SIC_IAR1_RESET 0x33322221
#define BF531_SIC_IAR2_RESET 0x66655444

static void bf531_reset_hold(Object *obj, ResetType type)
{
    BF531State *s = BF531(obj);

    s->sic_imask = 0;
    s->sic_isr = 0;
    s->sic_iwr = 0xffffffff;
    s->sic_levels = 0;
    s->sic_iar[0] = BF531_SIC_IAR0_RESET;
    s->sic_iar[1] = BF531_SIC_IAR1_RESET;
    s->sic_iar[2] = BF531_SIC_IAR2_RESET;
}

static void bf531_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = bf531_realize;
    device_class_set_props(dc, bf531_properties);
    rc->phases.hold = bf531_reset_hold;
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
