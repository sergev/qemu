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

#define PIC32MX7

/* Hardware addresses */
#define PROGRAM_FLASH_START 0x1d000000
#define BOOT_FLASH_START    0x1fc00000
#define DATA_MEM_START      0x00000000
#define IO_MEM_START        0x1f800000
#define IO_MEM_SIZE         (1024*1024)         // 1 Mbyte

#ifdef PIC32MX7
#define PROGRAM_FLASH_SIZE  (512*1024)          // 512 kbytes
#define BOOT_FLASH_SIZE     (12*1024)           // 12 kbytes
#define DATA_MEM_SIZE       (128*1024)          // 128 kbytes
#define USER_MEM_START      0xbf000000
#endif

#ifdef PIC32MZ
#define PROGRAM_FLASH_SIZE  (2*1024*1024)       // 2 Mbytes
#define BOOT_FLASH_SIZE     (64*1024)           // 64 kbytes
#define DATA_MEM_SIZE       (512*1024)          // 512 kbytes
#endif

#define IN_PROGRAM_MEM(addr) (addr >= PROGRAM_FLASH_START && \
                             addr < PROGRAM_FLASH_START+PROGRAM_FLASH_SIZE)
#define IN_BOOT_MEM(addr)   (addr >= BOOT_FLASH_START && \
                             addr < BOOT_FLASH_START+BOOT_FLASH_SIZE)

/* Macros for converting between hex and binary. */
#define NIBBLE(x)       (isdigit(x) ? (x)-'0' : tolower(x)+10-'a')
#define HEX(buffer)     ((NIBBLE((buffer)[0])<<4) + NIBBLE((buffer)[1]))

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

static void store_byte (char *progmem, char *bootmem, unsigned address, unsigned char byte)
{
    if (IN_PROGRAM_MEM(address)) {
        //printf("Store %02x to program memory %08x\n", byte, address);
        progmem[address & 0xfffff] = byte;
    }
    else if (IN_BOOT_MEM(address)) {
        //printf("Store %02x to boot memory %08x\n", byte, address);
        bootmem[address & 0xffff] = byte;
    }
}

static unsigned virt_to_phys (unsigned address)
{
    if (address >= 0xa0000000 && address <= 0xbfffffff)
        return address - 0xa0000000;
    if (address >= 0x80000000 && address <= 0x9fffffff)
        return address - 0x80000000;
    return address;
}

/*
 * Read the S record file.
 */
static int load_srec (void *progmem, void *bootmem, const char *filename)
{
    FILE *fd;
    unsigned char buf [256];
    unsigned char *data;
    unsigned address;
    int bytes, output_len;

    fd = fopen (filename, "r");
    if (! fd) {
        perror (filename);
        exit (1);
    }
    output_len = 0;
    while (fgets ((char*) buf, sizeof(buf), fd)) {
        if (buf[0] == '\n')
            continue;
        if (buf[0] != 'S') {
            if (output_len == 0)
                break;
            printf("%s: bad file format\n", filename);
            exit (1);
        }

        /* Starting an S-record.  */
        if (! isxdigit (buf[2]) || ! isxdigit (buf[3])) {
            printf("%s: bad record: %s\n", filename, buf);
            exit (1);
        }
        bytes = HEX (buf + 2);

        /* Ignore the checksum byte.  */
        --bytes;

        address = 0;
        data = buf + 4;
        switch (buf[1]) {
        case '7':
            address = HEX (data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '8':
            address = (address << 8) | HEX (data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '9':
            address = (address << 8) | HEX (data);
            data += 2;
            address = (address << 8) | HEX (data);
            data += 2;
            bytes -= 2;
            if (bytes == 0) {
                //printf("%s: start address = %08x\n", filename, address);
                //TODO: set start address
            }
            goto done;

        case '3':
            address = HEX (data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '2':
            address = (address << 8) | HEX (data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '1':
            address = (address << 8) | HEX (data);
            data += 2;
            address = (address << 8) | HEX (data);
            data += 2;
            bytes -= 2;

            address = virt_to_phys (address);
            if (! IN_PROGRAM_MEM(address) && ! IN_BOOT_MEM(address)) {
                printf("%s: incorrect address %08X, must be %08X-%08X or %08X-%08X\n",
                    filename, address, PROGRAM_FLASH_START,
                    PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE - 1,
                    BOOT_FLASH_START, BOOT_FLASH_START + BOOT_FLASH_SIZE - 1);
                exit (1);
            }
            output_len += bytes;
            while (bytes-- > 0) {
                store_byte (progmem, bootmem, address++, HEX (data));
                data += 2;
            }
            break;
        }
    }
done:
    fclose (fd);
    return output_len;
}

/*
 * Read HEX file.
 */
static int load_hex (void *progmem, void *bootmem, const char *filename)
{
    FILE *fd;
    unsigned char buf [256], data[16], record_type, sum;
    unsigned address, high;
    int bytes, output_len, i;

    fd = fopen (filename, "r");
    if (! fd) {
        perror (filename);
        exit (1);
    }
    output_len = 0;
    high = 0;
    while (fgets ((char*) buf, sizeof(buf), fd)) {
        if (buf[0] == '\n')
            continue;
        if (buf[0] != ':') {
            if (output_len == 0)
                break;
            printf("%s: bad HEX file format\n", filename);
            exit (1);
        }
        if (! isxdigit (buf[1]) || ! isxdigit (buf[2]) ||
            ! isxdigit (buf[3]) || ! isxdigit (buf[4]) ||
            ! isxdigit (buf[5]) || ! isxdigit (buf[6]) ||
            ! isxdigit (buf[7]) || ! isxdigit (buf[8])) {
            printf("%s: bad record: %s\n", filename, buf);
            exit (1);
        }
	record_type = HEX (buf+7);
	if (record_type == 1) {
	    /* End of file. */
            break;
        }
	bytes = HEX (buf+1);
	if (strlen ((char*) buf) < bytes * 2 + 11) {
            printf("%s: too short hex line\n", filename);
            exit (1);
        }
	address = high << 16 | HEX (buf+3) << 8 | HEX (buf+5);
        if (address & 3) {
            printf("%s: odd address\n", filename);
            exit (1);
        }

	sum = 0;
	for (i=0; i<bytes; ++i) {
            data [i] = HEX (buf+9 + i + i);
	    sum += data [i];
	}
	sum += record_type + bytes + (address & 0xff) + (address >> 8 & 0xff);
	if (sum != (unsigned char) - HEX (buf+9 + bytes + bytes)) {
            printf("%s: bad hex checksum\n", filename);
            printf("Line %s", buf);
            exit (1);
        }

	if (record_type == 5) {
	    /* Start address. */
            if (bytes != 4) {
                printf("%s: invalid length of hex start address record: %d bytes\n",
                    filename, bytes);
                exit (1);
            }
	    address = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
            //printf("%s: start address = %08x\n", filename, address);
            //TODO: set start address
	    continue;
	}
	if (record_type == 4) {
	    /* Extended address. */
            if (bytes != 2) {
                printf("%s: invalid length of hex linear address record: %d bytes\n",
                    filename, bytes);
                exit (1);
            }
	    high = data[0] << 8 | data[1];
	    continue;
	}
	if (record_type != 0) {
            printf("%s: unknown hex record type: %d\n",
                filename, record_type);
            exit (1);
        }

        /* Data record found. */
        address = virt_to_phys (address);
        if (! IN_PROGRAM_MEM(address) && ! IN_BOOT_MEM(address)) {
            printf("%s: incorrect address %08X, must be %08X-%08X or %08X-%08X\n",
                filename, address, PROGRAM_FLASH_START,
                PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE - 1,
                BOOT_FLASH_START, BOOT_FLASH_START + BOOT_FLASH_SIZE - 1);
            exit (1);
        }
        output_len += bytes;
        for (i=0; i<bytes; i++) {
            store_byte (progmem, bootmem, address++, data [i]);
        }
    }
    fclose (fd);
    return output_len;
}

static int load_file(MemoryRegion *prog_mem, MemoryRegion *boot_mem,
    const char *filename)
{
    void *prog = memory_region_get_ram_ptr(prog_mem);
    void *boot = memory_region_get_ram_ptr(boot_mem);

    int memory_len = load_srec (prog, boot, filename);
    if (memory_len == 0) {
        memory_len = load_hex (prog, boot, filename);
        if (memory_len == 0) {
            return 0;
        }
    }
    printf("Load file: '%s', %d bytes\n", filename, memory_len);
    return 1;
}

static void mips_pic32_init(MachineState *machine)
{
    const char *cpu_model = machine->cpu_model;
    unsigned ram_size = 128*1024;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *ram_main = g_new(MemoryRegion, 1);
    MemoryRegion *ram_user = g_new(MemoryRegion, 1);
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
    //empty_slot_init(0, 0x20000000);

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

    /* Register RAM */
    printf("RAM size: %u kbytes\n", ram_size / 1024);
    memory_region_init_ram(ram_main, NULL, "kernel.ram",
        ram_size, &error_abort);
    vmstate_register_ram_global(ram_main);
    memory_region_add_subregion(system_memory, DATA_MEM_START, ram_main);

    /* Alias for user space 96 kbytes */
    memory_region_init_alias(ram_user, NULL, "user.ram",
        ram_main, 0x8000, ram_size - 0x8000);
    memory_region_add_subregion(system_memory, USER_MEM_START + 0x8000, ram_user);

    /* FPGA */
    memory_region_init_io(&s->iomem, NULL, &pic32_fpga_ops, s,
                          "pic32-fpga", 0x100000);

    /* The CBUS UART is attached to the CPU INT2 pin, ie interrupt 4 */
    s->uart = serial_mm_init(system_memory, IO_MEM_START + 0x900, 3, env->irq[4],
                             230400, serial_hds[2], DEVICE_NATIVE_ENDIAN);

    /*
     * Map the flash memory.
     */
    memory_region_init_ram(boot_mem, NULL, "boot.flash", BOOT_FLASH_SIZE, &error_abort);
    memory_region_init_ram(prog_mem, NULL, "prog.flash", PROGRAM_FLASH_SIZE, &error_abort);

    /* Load a Flash memory image. */
    if (! machine->kernel_filename) {
        error_report("No -kernel argument was specified.");
        exit(1);
    }
    if (bios_name) {
        load_file(prog_mem, boot_mem, bios_name);
    }
    load_file(prog_mem, boot_mem, machine->kernel_filename);

    memory_region_set_readonly(boot_mem, true);
    memory_region_set_readonly(prog_mem, true);
    memory_region_add_subregion(system_memory, BOOT_FLASH_START, boot_mem);
    memory_region_add_subregion(system_memory, PROGRAM_FLASH_START, prog_mem);

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
