/*
 * QEMU support for Microchip PIC32 microcontroller family
 *
 * Copyright (c) 2015 Serge Vakulenko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* Only 32-bit little endian mode supported. */
#if !defined TARGET_MIPS64 && !defined TARGET_WORDS_BIGENDIAN

#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/mips/cpudevs.h"
#include "sysemu/char.h"
#include "hw/loader.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/error-report.h"
#include "hw/empty_slot.h"

/* Hardware addresses */
#define FLASH_ADDRESS   0x1d000000ULL
#define FPGA_ADDRESS    0x1f000000ULL
#define RESET_ADDRESS   0x1fc00000ULL

#define FLASH_SIZE      (512*1024)
#define BOOT_SIZE       (12*1024)

#define TYPE_MIPS_PIC32 "mips-pic32"

typedef struct {
    SysBusDevice    parent_obj;
    //qemu_irq        *i8259;
    MemoryRegion    iomem;
    SerialState     *uart;
} PIC32State;

static uint64_t pic32_fpga_read(void *opaque, hwaddr addr, unsigned size)
{
    //PIC32State *s = opaque;
    uint32_t val = 0;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {

    /* SWITCH Register */
    case 0x00200:
        val = 0x00000000;		/* All switches closed */
        break;

    /* STATUS Register */
    case 0x00208:
        val = 0x00000010;
        break;

    /* JMPRS Register */
    case 0x00210:
        val = 0x00;
        break;

    /* UART Registers are handled directly by the serial device */

    default:
        printf ("pic32_fpga_read: Bad register offset 0x" TARGET_FMT_lx "\n",
            (unsigned) addr);
        break;
    }
    return val;
}

static void pic32_fpga_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    //PIC32State *s = opaque;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {

    /* SWITCH Register */
    case 0x00200:
        break;

    /* JMPRS Register */
    case 0x00210:
        break;

    /* SOFTRES Register */
    case 0x00500:
        if (val == 0x42)
            qemu_system_reset_request ();
        break;

    /* UART Registers are handled directly by the serial device */

    default:
        printf ("pic32_fpga_write: Bad register offset 0x" TARGET_FMT_lx "\n",
            (unsigned) addr);
        break;
    }
}

static const MemoryRegionOps pic32_fpga_ops = {
    .read       = pic32_fpga_read,
    .write      = pic32_fpga_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

#if 0
static void cpu_request_exit(void *opaque, int irq, int level)
{
    CPUState *cpu = current_cpu;

    if (cpu && level) {
        cpu_exit(cpu);
    }
}
#endif

static void mips_pic32_init(MachineState *machine)
{
    const char *cpu_model = machine->cpu_model;
    unsigned ram_size = 128*1024;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *ram_high = g_new(MemoryRegion, 1);
    MemoryRegion *ram_low = g_new(MemoryRegion, 1);
    MemoryRegion *prog_mem = g_new(MemoryRegion, 1);
    MemoryRegion *boot_mem = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    CPUMIPSState *env;
    int i;

    DeviceState *dev = qdev_create(NULL, TYPE_MIPS_PIC32);
    PIC32State *s = OBJECT_CHECK(PIC32State, dev, TYPE_MIPS_PIC32);

    /* The whole address space doesn't generate exception when
     * accessing invalid memory. Create an empty slot to
     * emulate this feature. */
    empty_slot_init(0, 0x20000000);

    qdev_init_nofail(dev);

    /* Make sure the first 3 serial ports are associated with a device. */
    for(i = 0; i < 3; i++) {
        if (!serial_hds[i]) {
            char label[32];
            snprintf(label, sizeof(label), "serial%d", i);
            serial_hds[i] = qemu_chr_new(label, "null", NULL);
        }
    }

    /* Init CPU. */
    if (! cpu_model) {
        cpu_model = "4KEc";
    }
    printf("Processor: %s\n", cpu_model);
    cpu = cpu_mips_init(cpu_model);
    if (! cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    env = &cpu->env;

    /* Init internal devices */
    cpu_mips_irq_init_cpu(env);
    cpu_mips_clock_init(env);
    qemu_register_reset(main_cpu_reset, cpu);

    cpu = MIPS_CPU(first_cpu);
    env = &cpu->env;

    /* register RAM at high address */
    printf("RAM size: %u kbytes\n", ram_size / 1024);
    memory_region_init_ram(ram_high, NULL, "mips_pic32.ram",
        ram_size, &error_abort);
    vmstate_register_ram_global(ram_high);
    memory_region_add_subregion(system_memory, 0x80000000, ram_high);

    /* alias for user space 96 kbytes */
    memory_region_init_alias(ram_low, NULL, "mips_pic32_low.ram",
        ram_high, 0x8000, ram_size - 0x8000);
    memory_region_add_subregion(system_memory, 0xbf000000 + 0x8000, ram_low);

    /* FPGA */
    memory_region_init_io(&s->iomem, NULL, &pic32_fpga_ops, s,
                          "pic32-fpga", 0x100000);

    /* The CBUS UART is attached to the CPU INT2 pin, ie interrupt 4 */
    s->uart = serial_mm_init(system_memory, FPGA_ADDRESS + 0x900, 3, env->irq[4],
                             230400, serial_hds[2], DEVICE_NATIVE_ENDIAN);

    /*
     * Map the flash memory.
     */
    memory_region_init_ram(boot_mem, NULL, "flash.1fc", BOOT_SIZE, &error_abort);
    memory_region_init_ram(prog_mem, NULL, "flash.1d0", FLASH_SIZE, &error_abort);

    /* Load a Flash memory image. */
    if (! machine->kernel_filename) {
        error_report("No -kernel argument was specified.");
        exit(1);
    }
    printf("Load file: %s\n", machine->kernel_filename);
    int nbytes = load_image_targphys(machine->kernel_filename, FLASH_ADDRESS, BOOT_SIZE);
    if (nbytes < 0) {
        error_report("Could not load PIC32 firmware '%s'",
            machine->kernel_filename);
        exit(1);
    }
    printf("Image size: %d bytes\n", nbytes);

    memory_region_set_readonly(boot_mem, true);
    memory_region_set_readonly(prog_mem, true);
    memory_region_add_subregion(system_memory, RESET_ADDRESS, boot_mem);
    memory_region_add_subregion(system_memory, FLASH_ADDRESS, prog_mem);

    /* Init internal devices */
    cpu_mips_irq_init_cpu(env);
    cpu_mips_clock_init(env);
#if 0
    /*
     * We have a circular dependency problem: pci_bus depends on isa_irq,
     * isa_irq is provided by i8259, i8259 depends on ISA, ISA depends
     * on piix4, and piix4 depends on pci_bus.  To stop the cycle we have
     * qemu_irq_proxy() adds an extra bit of indirection, allowing us
     * to resolve the isa_irq -> i8259 dependency after i8259 is initialized.
     */
    int piix4_devfn;
    qemu_irq *cpu_exit_irq;
    PCIBus *pci_bus;
    ISABus *isa_bus;
    qemu_irq *isa_irq;

    isa_irq = qemu_irq_proxy(&s->i8259, 16);

    /* Northbridge */
    pci_bus = gt64120_register(isa_irq);

    /* Southbridge */
    //ide_drive_get(hd, ARRAY_SIZE(hd));

    piix4_devfn = piix4_init(pci_bus, &isa_bus, 80);

    /* Interrupt controller */
    /* The 8259 is attached to the MIPS CPU INT0 pin, ie interrupt 2 */
    s->i8259 = i8259_init(isa_bus, env->irq[2]);
    isa_bus_irqs(isa_bus, s->i8259);

    //pci_piix4_ide_init(pci_bus, hd, piix4_devfn + 1);
    pci_create_simple(pci_bus, piix4_devfn + 2, "piix4-usb-uhci");
    cpu_exit_irq = qemu_allocate_irqs(cpu_request_exit, NULL, 1);
    DMA_init(0, cpu_exit_irq);

    serial_isa_init(isa_bus, 0, serial_hds[0]);
    serial_isa_init(isa_bus, 1, serial_hds[1]);
#endif
}

static int mips_pic32_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void mips_pic32_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = mips_pic32_sysbus_device_init;
}

static const TypeInfo mips_pic32_device = {
    .name          = TYPE_MIPS_PIC32,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PIC32State),
    .class_init    = mips_pic32_class_init,
};

static QEMUMachine mips_pic32_machine = {
    .name           = "pic32",
    .desc           = "Microchip PIC32 microcontroller",
    .init           = mips_pic32_init,
    .max_cpus       = 1,
    .is_default     = 1,
};

static void mips_pic32_register_types(void)
{
    type_register_static(&mips_pic32_device);
}

static void mips_pic32_machine_init(void)
{
    qemu_register_machine(&mips_pic32_machine);
}

type_init(mips_pic32_register_types)
machine_init(mips_pic32_machine_init);

#endif /* !TARGET_MIPS64 && !TARGET_WORDS_BIGENDIAN */
