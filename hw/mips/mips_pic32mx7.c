/*
 * QEMU support for Microchip PIC32MX7 microcontroller.
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
#include "config.h"
#if !defined TARGET_MIPS64 && !defined TARGET_WORDS_BIGENDIAN

#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/mips/cpudevs.h"
#include "sysemu/char.h"
#include "hw/loader.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"
#include <termios.h>

#define PIC32MX7
#include "pic32mx.h"
#include "pic32_peripherals.h"

/* Hardware addresses */
#define PROGRAM_FLASH_START 0x1d000000
#define BOOT_FLASH_START    0x1fc00000
#define DATA_MEM_START      0x00000000
#define IO_MEM_START        0x1f800000

#define PROGRAM_FLASH_SIZE  (512*1024)          // 512 kbytes
#define BOOT_FLASH_SIZE     (12*1024)           // 12 kbytes
#define DATA_MEM_SIZE       (128*1024)          // 128 kbytes
#define USER_MEM_START      0xbf000000

#define TYPE_MIPS_PIC32     "mips-pic32mx7"

/*
 * Board variants.
 */
enum {
    BOARD_MAX32,            /* chipKIT Max32 board */
    BOARD_MAXIMITE,         /* Geoff's Maximite board */
    BOARD_EXPLORER16,       /* Microchip Explorer-16 board */
};

static const char *board_name[] = {
    "chipKIT Max32",
    "Geoff's Maximite Computer",
    "Microchip Explorer16",
};

/*
 * Pointers to Flash memory contents.
 */
static char *prog_ptr;
static char *boot_ptr;

#define BOOTMEM(addr) ((uint32_t*) boot_ptr) [(addr & 0xffff) >> 2]

/*
 * PIC32MX7 specific table:
 * translate IRQ number to interrupt vector.
 */
static const int irq_to_vector[] = {
    PIC32_VECT_CT,      /* 0  - Core Timer Interrupt */
    PIC32_VECT_CS0,     /* 1  - Core Software Interrupt 0 */
    PIC32_VECT_CS1,     /* 2  - Core Software Interrupt 1 */
    PIC32_VECT_INT0,    /* 3  - External Interrupt 0 */
    PIC32_VECT_T1,      /* 4  - Timer1 */
    PIC32_VECT_IC1,     /* 5  - Input Capture 1 */
    PIC32_VECT_OC1,     /* 6  - Output Compare 1 */
    PIC32_VECT_INT1,    /* 7  - External Interrupt 1 */
    PIC32_VECT_T2,      /* 8  - Timer2 */
    PIC32_VECT_IC2,     /* 9  - Input Capture 2 */
    PIC32_VECT_OC2,     /* 10 - Output Compare 2 */
    PIC32_VECT_INT2,    /* 11 - External Interrupt 2 */
    PIC32_VECT_T3,      /* 12 - Timer3 */
    PIC32_VECT_IC3,     /* 13 - Input Capture 3 */
    PIC32_VECT_OC3,     /* 14 - Output Compare 3 */
    PIC32_VECT_INT3,    /* 15 - External Interrupt 3 */
    PIC32_VECT_T4,      /* 16 - Timer4 */
    PIC32_VECT_IC4,     /* 17 - Input Capture 4 */
    PIC32_VECT_OC4,     /* 18 - Output Compare 4 */
    PIC32_VECT_INT4,    /* 19 - External Interrupt 4 */
    PIC32_VECT_T5,      /* 20 - Timer5 */
    PIC32_VECT_IC5,     /* 21 - Input Capture 5 */
    PIC32_VECT_OC5,     /* 22 - Output Compare 5 */
    PIC32_VECT_SPI1,    /* 23 - SPI1 Fault */
    PIC32_VECT_SPI1,    /* 24 - SPI1 Transfer Done */
    PIC32_VECT_SPI1,    /* 25 - SPI1 Receive Done */

    PIC32_VECT_U1     | /* 26 - UART1 Error */
    PIC32_VECT_SPI3   | /* 26 - SPI3 Fault */
    PIC32_VECT_I2C3,    /* 26 - I2C3 Bus Collision Event */

    PIC32_VECT_U1     | /* 27 - UART1 Receiver */
    PIC32_VECT_SPI3   | /* 27 - SPI3 Transfer Done */
    PIC32_VECT_I2C3,    /* 27 - I2C3 Slave Event */

    PIC32_VECT_U1     | /* 28 - UART1 Transmitter */
    PIC32_VECT_SPI3   | /* 28 - SPI3 Receive Done */
    PIC32_VECT_I2C3,    /* 28 - I2C3 Master Event */

    PIC32_VECT_I2C1,    /* 29 - I2C1 Bus Collision Event */
    PIC32_VECT_I2C1,    /* 30 - I2C1 Slave Event */
    PIC32_VECT_I2C1,    /* 31 - I2C1 Master Event */
    PIC32_VECT_CN,      /* 32 - Input Change Interrupt */
    PIC32_VECT_AD1,     /* 33 - ADC1 Convert Done */
    PIC32_VECT_PMP,     /* 34 - Parallel Master Port */
    PIC32_VECT_CMP1,    /* 35 - Comparator Interrupt */
    PIC32_VECT_CMP2,    /* 36 - Comparator Interrupt */

    PIC32_VECT_U3     | /* 37 - UART3 Error */
    PIC32_VECT_SPI2   | /* 37 - SPI2 Fault */
    PIC32_VECT_I2C4,    /* 37 - I2C4 Bus Collision Event */

    PIC32_VECT_U3     | /* 38 - UART3 Receiver */
    PIC32_VECT_SPI2   | /* 38 - SPI2 Transfer Done */
    PIC32_VECT_I2C4,    /* 38 - I2C4 Slave Event */

    PIC32_VECT_U3     | /* 39 - UART3 Transmitter */
    PIC32_VECT_SPI2   | /* 39 - SPI2 Receive Done */
    PIC32_VECT_I2C4,    /* 39 - I2C4 Master Event */

    PIC32_VECT_U2     | /* 40 - UART2 Error */
    PIC32_VECT_SPI4   | /* 40 - SPI4 Fault */
    PIC32_VECT_I2C5,    /* 40 - I2C5 Bus Collision Event */

    PIC32_VECT_U2     | /* 41 - UART2 Receiver */
    PIC32_VECT_SPI4   | /* 41 - SPI4 Transfer Done */
    PIC32_VECT_I2C5,    /* 41 - I2C5 Slave Event */

    PIC32_VECT_U2     | /* 42 - UART2 Transmitter */
    PIC32_VECT_SPI4   | /* 42 - SPI4 Receive Done */
    PIC32_VECT_I2C5,    /* 42 - I2C5 Master Event */

    PIC32_VECT_I2C2,    /* 43 - I2C2 Bus Collision Event */
    PIC32_VECT_I2C2,    /* 44 - I2C2 Slave Event */
    PIC32_VECT_I2C2,    /* 45 - I2C2 Master Event */
    PIC32_VECT_FSCM,    /* 46 - Fail-Safe Clock Monitor */
    PIC32_VECT_RTCC,    /* 47 - Real-Time Clock and Calendar */
    PIC32_VECT_DMA0,    /* 48 - DMA Channel 0 */
    PIC32_VECT_DMA1,    /* 49 - DMA Channel 1 */
    PIC32_VECT_DMA2,    /* 50 - DMA Channel 2 */
    PIC32_VECT_DMA3,    /* 51 - DMA Channel 3 */
    PIC32_VECT_DMA4,    /* 52 - DMA Channel 4 */
    PIC32_VECT_DMA5,    /* 53 - DMA Channel 5 */
    PIC32_VECT_DMA6,    /* 54 - DMA Channel 6 */
    PIC32_VECT_DMA7,    /* 55 - DMA Channel 7 */
    PIC32_VECT_FCE,     /* 56 - Flash Control Event */
    PIC32_VECT_USB,     /* 57 - USB */
    PIC32_VECT_CAN1,    /* 58 - Control Area Network 1 */
    PIC32_VECT_CAN2,    /* 59 - Control Area Network 2 */
    PIC32_VECT_ETH,     /* 60 - Ethernet Interrupt */
    PIC32_VECT_IC1,     /* 61 - Input Capture 1 Error */
    PIC32_VECT_IC2,     /* 62 - Input Capture 2 Error */
    PIC32_VECT_IC3,     /* 63 - Input Capture 3 Error */
    PIC32_VECT_IC4,     /* 64 - Input Capture 4 Error */
    PIC32_VECT_IC5,     /* 65 - Input Capture 5 Error */
    PIC32_VECT_PMP,     /* 66 - Parallel Master Port Error */
    PIC32_VECT_U4,      /* 67 - UART4 Error */
    PIC32_VECT_U4,      /* 68 - UART4 Receiver */
    PIC32_VECT_U4,      /* 69 - UART4 Transmitter */
    PIC32_VECT_U6,      /* 70 - UART6 Error */
    PIC32_VECT_U6,      /* 71 - UART6 Receiver */
    PIC32_VECT_U6,      /* 72 - UART6 Transmitter */
    PIC32_VECT_U5,      /* 73 - UART5 Error */
    PIC32_VECT_U5,      /* 74 - UART5 Receiver */
    PIC32_VECT_U5,      /* 75 - UART5 Transmitter */
};

static void update_irq_status(pic32_t *s)
{
    /* Assume no interrupts pending. */
    int cause_ripl = 0;
    int vector = 0;
    CPUMIPSState *env = &s->cpu->env;
    int current_ripl = (env->CP0_Cause >> (CP0Ca_IP + 2)) & 0x3f;

    VALUE(INTSTAT) = 0;

    if ((VALUE(IFS0) & VALUE(IEC0)) ||
        (VALUE(IFS1) & VALUE(IEC1)) ||
        (VALUE(IFS2) & VALUE(IEC2)))
    {
        /* Find the most prioritive pending interrupt,
         * it's vector and level. */
        int irq;
        for (irq=0; irq<sizeof(irq_to_vector)/sizeof(int); irq++) {
            int n = irq >> 5;

            if (((VALUE(IFS(n)) & VALUE(IEC(n))) >> (irq & 31)) & 1) {
                /* Interrupt is pending. */
                int v = irq_to_vector [irq];
                if (v < 0)
                    continue;

                int level = VALUE(IPC(v >> 2));
                level >>= 2 + (v & 3) * 8;
                level &= 7;
                if (level > cause_ripl) {
                    vector = v;
                    cause_ripl = level;
                }
            }
        }
        VALUE(INTSTAT) = vector | (cause_ripl << 8);
    }

    if (cause_ripl == current_ripl)
        return;

    if (qemu_loglevel_mask(CPU_LOG_INSTR))
        fprintf(qemu_logfile, "--- Priority level Cause.RIPL = %u\n",
            cause_ripl);

    /*
     * Modify Cause.RIPL field and take EIC interrupt.
     */
    env->CP0_Cause &= ~(0x3f << (CP0Ca_IP + 2));
    env->CP0_Cause |= cause_ripl << (CP0Ca_IP + 2);
    cpu_interrupt(CPU(s->cpu), CPU_INTERRUPT_HARD);
}

/*
 * Set interrupt flag status
 */
static void irq_raise(pic32_t *s, int irq)
{
    if (VALUE(IFS(irq >> 5)) & (1 << (irq & 31)))
        return;

    VALUE(IFS(irq >> 5)) |= 1 << (irq & 31);
    update_irq_status(s);
}

/*
 * Clear interrupt flag status
 */
static void irq_clear(pic32_t *s, int irq)
{
    if (! (VALUE(IFS(irq >> 5)) & (1 << (irq & 31))))
        return;

    VALUE(IFS(irq >> 5)) &= ~(1 << (irq & 31));
    update_irq_status(s);
}

/*
 * Timer interrupt.
 */
static void pic32_timer_irq(CPUMIPSState *env, int raise)
{
    pic32_t *s = env->eic_context;

    if (raise) {
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- %08x: Timer interrupt\n",
                env->active_tc.PC);
        irq_raise(s, 0);
    } else {
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- Clear timer interrupt\n");
        irq_clear(s, 0);
    }
}

/*
 * Software interrupt.
 */
static void pic32_soft_irq(CPUMIPSState *env, int num)
{
    pic32_t *s = env->eic_context;

    if (qemu_loglevel_mask(CPU_LOG_INSTR))
        fprintf(qemu_logfile, "--- %08x: Soft interrupt %u\n",
            env->active_tc.PC, num);
    irq_raise(s, num + 1);
}

/*
 * Perform an assign/clear/set/invert operation.
 */
static inline unsigned write_op(int a, int b, int op)
{
    switch (op & 0xc) {
    case 0x0: a = b;   break;   // Assign
    case 0x4: a &= ~b; break;   // Clear
    case 0x8: a |= b;  break;   // Set
    case 0xc: a ^= b;  break;   // Invert
    }
    return a;
}

static void io_reset(pic32_t *s)
{
    int i;

    /*
     * Bus matrix control registers.
     */
    VALUE(BMXCON)    = 0x001f0041;      // Bus Matrix Control
    VALUE(BMXDKPBA)  = 0;               // Data RAM kernel program base address
    VALUE(BMXDUDBA)  = 0;               // Data RAM user data base address
    VALUE(BMXDUPBA)  = 0;               // Data RAM user program base address
    VALUE(BMXPUPBA)  = 0;               // Program Flash user program base address
    VALUE(BMXDRMSZ)  = 128 * 1024;      // Data RAM memory size
    VALUE(BMXPFMSZ)  = 512 * 1024;      // Program Flash memory size
    VALUE(BMXBOOTSZ) = 12 * 1024;       // Boot Flash size

    /*
     * Prefetch controller.
     */
    VALUE(CHECON) = 0x00000007;

    /*
     * System controller.
     */
    VALUE(OSCTUN) = 0;
    VALUE(DDPCON) = 0;
    VALUE(SYSKEY) = 0;
    VALUE(RCON)   = 0;
    VALUE(RSWRST) = 0;
    s->syskey_unlock = 0;

    /*
     * Analog to digital converter.
     */
    VALUE(AD1CON1) = 0;                 // Control register 1
    VALUE(AD1CON2) = 0;                 // Control register 2
    VALUE(AD1CON3) = 0;                 // Control register 3
    VALUE(AD1CHS)  = 0;                 // Channel select
    VALUE(AD1CSSL) = 0;                 // Input scan selection
    VALUE(AD1PCFG) = 0;                 // Port configuration

    /*
     * General purpose IO signals.
     * All pins are inputs, high, open drains and pullups disabled.
     * No interrupts on change.
     */
    VALUE(TRISA) = 0xFFFF;              // Port A: mask of inputs
    VALUE(PORTA) = 0xFFFF;              // Port A: read inputs, write outputs
    VALUE(LATA)  = 0xFFFF;              // Port A: read/write outputs
    VALUE(ODCA)  = 0;                   // Port A: open drain configuration
    VALUE(TRISB) = 0xFFFF;              // Port B: mask of inputs
    VALUE(PORTB) = 0xFFFF;              // Port B: read inputs, write outputs
    VALUE(LATB)  = 0xFFFF;              // Port B: read/write outputs
    VALUE(ODCB)  = 0;                   // Port B: open drain configuration
    VALUE(TRISC) = 0xFFFF;              // Port C: mask of inputs
    VALUE(PORTC) = 0xFFFF;              // Port C: read inputs, write outputs
    VALUE(LATC)  = 0xFFFF;              // Port C: read/write outputs
    VALUE(ODCC)  = 0;                   // Port C: open drain configuration
    VALUE(TRISD) = 0xFFFF;              // Port D: mask of inputs
    VALUE(PORTD) = 0xFFFF;              // Port D: read inputs, write outputs
    VALUE(LATD)  = 0xFFFF;              // Port D: read/write outputs
    VALUE(ODCD)  = 0;                   // Port D: open drain configuration
    VALUE(TRISE) = 0xFFFF;              // Port E: mask of inputs
    VALUE(PORTE) = 0xFFFF;              // Port D: read inputs, write outputs
    VALUE(LATE)  = 0xFFFF;              // Port E: read/write outputs
    VALUE(ODCE)  = 0;                   // Port E: open drain configuration
    VALUE(TRISF) = 0xFFFF;              // Port F: mask of inputs
    VALUE(PORTF) = 0xFFFF;              // Port F: read inputs, write outputs
    VALUE(LATF)  = 0xFFFF;              // Port F: read/write outputs
    VALUE(ODCF)  = 0;                   // Port F: open drain configuration
    VALUE(TRISG) = 0xFFFF;              // Port G: mask of inputs
    VALUE(PORTG) = 0xFFFF;              // Port G: read inputs, write outputs
    VALUE(LATG)  = 0xFFFF;              // Port G: read/write outputs
    VALUE(ODCG)  = 0;                   // Port G: open drain configuration
    VALUE(CNCON) = 0;                   // Interrupt-on-change control
    VALUE(CNEN)  = 0;                   // Input change interrupt enable
    VALUE(CNPUE) = 0;                   // Input pin pull-up enable

    /*
     * Reset UARTs.
     */
    VALUE(U1MODE)  = 0;
    VALUE(U1STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U1TXREG) = 0;
    VALUE(U1RXREG) = 0;
    VALUE(U1BRG)   = 0;
    VALUE(U2MODE)  = 0;
    VALUE(U2STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U2TXREG) = 0;
    VALUE(U2RXREG) = 0;
    VALUE(U2BRG)   = 0;
    VALUE(U3MODE)  = 0;
    VALUE(U3STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U3TXREG) = 0;
    VALUE(U3RXREG) = 0;
    VALUE(U3BRG)   = 0;
    VALUE(U4MODE)  = 0;
    VALUE(U4STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U4TXREG) = 0;
    VALUE(U4RXREG) = 0;
    VALUE(U4BRG)   = 0;
    VALUE(U5MODE)  = 0;
    VALUE(U5STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U5TXREG) = 0;
    VALUE(U5RXREG) = 0;
    VALUE(U5BRG)   = 0;
    VALUE(U6MODE)  = 0;
    VALUE(U6STA)   = PIC32_USTA_RIDLE | PIC32_USTA_TRMT;
    VALUE(U6TXREG) = 0;
    VALUE(U6RXREG) = 0;
    VALUE(U6BRG)   = 0;

    /*
     * Reset SPI.
     */
    VALUE(SPI1CON)  = 0;
    VALUE(SPI1STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI1BRG)  = 0;

    VALUE(SPI2CON)  = 0;
    VALUE(SPI2STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI2BRG)  = 0;

    VALUE(SPI3CON)  = 0;
    VALUE(SPI3STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI3BRG)  = 0;

    VALUE(SPI4CON)  = 0;
    VALUE(SPI4STAT) = PIC32_SPISTAT_SPITBE;     // Transmit buffer is empty
    VALUE(SPI4BRG)  = 0;

    for (i=0; i<NUM_SPI; i++) {
        s->spi[i].rfifo = 0;
        s->spi[i].wfifo = 0;
    }
}

static unsigned io_read32(pic32_t *s, unsigned offset, const char **namep)
{
    unsigned *bufp = &VALUE(offset);

    switch (offset) {
    /*-------------------------------------------------------------------------
     * Bus matrix control registers.
     */
    STORAGE(BMXCON); break;     // Bus Mmatrix Control
    STORAGE(BMXDKPBA); break;   // Data RAM kernel program base address
    STORAGE(BMXDUDBA); break;   // Data RAM user data base address
    STORAGE(BMXDUPBA); break;   // Data RAM user program base address
    STORAGE(BMXPUPBA); break;   // Program Flash user program base address
    STORAGE(BMXDRMSZ); break;   // Data RAM memory size
    STORAGE(BMXPFMSZ); break;   // Program Flash memory size
    STORAGE(BMXBOOTSZ); break;  // Boot Flash size

    /*-------------------------------------------------------------------------
     * Interrupt controller registers.
     */
    STORAGE(INTCON); break;     // Interrupt Control
    STORAGE(INTSTAT); break;    // Interrupt Status
    STORAGE(IFS0); break;       // IFS(0..2) - Interrupt Flag Status
    STORAGE(IFS1); break;
    STORAGE(IFS2); break;
    STORAGE(IEC0); break;       // IEC(0..2) - Interrupt Enable Control
    STORAGE(IEC1); break;
    STORAGE(IEC2); break;
    STORAGE(IPC0); break;       // IPC(0..11) - Interrupt Priority Control
    STORAGE(IPC1); break;
    STORAGE(IPC2); break;
    STORAGE(IPC3); break;
    STORAGE(IPC4); break;
    STORAGE(IPC5); break;
    STORAGE(IPC6); break;
    STORAGE(IPC7); break;
    STORAGE(IPC8); break;
    STORAGE(IPC9); break;
    STORAGE(IPC10); break;
    STORAGE(IPC11); break;
    STORAGE(IPC12); break;

    /*-------------------------------------------------------------------------
     * Prefetch controller.
     */
    STORAGE(CHECON); break;     // Prefetch Control

    /*-------------------------------------------------------------------------
     * System controller.
     */
    STORAGE(OSCCON); break;     // Oscillator Control
    STORAGE(OSCTUN); break;     // Oscillator Tuning
    STORAGE(DDPCON); break;     // Debug Data Port Control
    STORAGE(DEVID); break;      // Device Identifier
    STORAGE(SYSKEY); break;     // System Key
    STORAGE(RCON); break;       // Reset Control
    STORAGE(RSWRST);            // Software Reset
        if ((VALUE(RSWRST) & 1) && s->stop_on_reset) {
            exit(0);
        }
        break;

    /*-------------------------------------------------------------------------
     * DMA controller.
     */
    STORAGE(DMACON); break;     // DMA Control
    STORAGE(DMASTAT); break;    // DMA Status
    STORAGE(DMAADDR); break;    // DMA Address

    /*-------------------------------------------------------------------------
     * Analog to digital converter.
     */
    STORAGE(AD1CON1); break;    // Control register 1
    STORAGE(AD1CON2); break;    // Control register 2
    STORAGE(AD1CON3); break;    // Control register 3
    STORAGE(AD1CHS); break;     // Channel select
    STORAGE(AD1CSSL); break;    // Input scan selection
    STORAGE(AD1PCFG); break;    // Port configuration
    STORAGE(ADC1BUF0); break;   // Result words
    STORAGE(ADC1BUF1); break;
    STORAGE(ADC1BUF2); break;
    STORAGE(ADC1BUF3); break;
    STORAGE(ADC1BUF4); break;
    STORAGE(ADC1BUF5); break;
    STORAGE(ADC1BUF6); break;
    STORAGE(ADC1BUF7); break;
    STORAGE(ADC1BUF8); break;
    STORAGE(ADC1BUF9); break;
    STORAGE(ADC1BUFA); break;
    STORAGE(ADC1BUFB); break;
    STORAGE(ADC1BUFC); break;
    STORAGE(ADC1BUFD); break;
    STORAGE(ADC1BUFE); break;
    STORAGE(ADC1BUFF); break;

    /*--------------------------------------
     * USB registers.
     */
    STORAGE(U1OTGIR); break;    // OTG interrupt flags
    STORAGE(U1OTGIE); break;    // OTG interrupt enable
    STORAGE(U1OTGSTAT); break;  // Comparator and pin status
    STORAGE(U1OTGCON); break;   // Resistor and pin control
    STORAGE(U1PWRC); break;     // Power control
    STORAGE(U1IR); break;       // Pending interrupt
    STORAGE(U1IE); break;       // Interrupt enable
    STORAGE(U1EIR); break;      // Pending error interrupt
    STORAGE(U1EIE); break;      // Error interrupt enable
    STORAGE(U1STAT); break;     // Status FIFO
    STORAGE(U1CON); break;      // Control
    STORAGE(U1ADDR); break;     // Address
    STORAGE(U1BDTP1); break;    // Buffer descriptor table pointer 1
    STORAGE(U1FRML); break;     // Frame counter low
    STORAGE(U1FRMH); break;     // Frame counter high
    STORAGE(U1TOK); break;      // Host control
    STORAGE(U1SOF); break;      // SOF counter
    STORAGE(U1BDTP2); break;    // Buffer descriptor table pointer 2
    STORAGE(U1BDTP3); break;    // Buffer descriptor table pointer 3
    STORAGE(U1CNFG1); break;    // Debug and idle
    STORAGE(U1EP(0)); break;    // Endpoint control
    STORAGE(U1EP(1)); break;
    STORAGE(U1EP(2)); break;
    STORAGE(U1EP(3)); break;
    STORAGE(U1EP(4)); break;
    STORAGE(U1EP(5)); break;
    STORAGE(U1EP(6)); break;
    STORAGE(U1EP(7)); break;
    STORAGE(U1EP(8)); break;
    STORAGE(U1EP(9)); break;
    STORAGE(U1EP(10)); break;
    STORAGE(U1EP(11)); break;
    STORAGE(U1EP(12)); break;
    STORAGE(U1EP(13)); break;
    STORAGE(U1EP(14)); break;
    STORAGE(U1EP(15)); break;

    /*-------------------------------------------------------------------------
     * General purpose IO signals.
     */
    STORAGE(TRISA); break;      // Port A: mask of inputs
    STORAGE(PORTA); break;      // Port A: read inputs
    STORAGE(LATA); break;       // Port A: read outputs
    STORAGE(ODCA); break;       // Port A: open drain configuration
    STORAGE(TRISB); break;      // Port B: mask of inputs
    STORAGE(PORTB); break;      // Port B: read inputs
    STORAGE(LATB); break;       // Port B: read outputs
    STORAGE(ODCB); break;       // Port B: open drain configuration
    STORAGE(TRISC); break;      // Port C: mask of inputs
    STORAGE(PORTC); break;      // Port C: read inputs
    STORAGE(LATC); break;       // Port C: read outputs
    STORAGE(ODCC); break;       // Port C: open drain configuration
    STORAGE(TRISD); break;      // Port D: mask of inputs
    STORAGE(PORTD); break;      // Port D: read inputs
    STORAGE(LATD); break;       // Port D: read outputs
    STORAGE(ODCD); break;       // Port D: open drain configuration
    STORAGE(TRISE); break;      // Port E: mask of inputs
    STORAGE(PORTE); break;      // Port E: read inputs
    STORAGE(LATE); break;       // Port E: read outputs
    STORAGE(ODCE); break;       // Port E: open drain configuration
    STORAGE(TRISF); break;      // Port F: mask of inputs
    STORAGE(PORTF); break;      // Port F: read inputs
    STORAGE(LATF); break;       // Port F: read outputs
    STORAGE(ODCF); break;       // Port F: open drain configuration
    STORAGE(TRISG); break;      // Port G: mask of inputs
    STORAGE(PORTG); break;      // Port G: read inputs
    STORAGE(LATG); break;       // Port G: read outputs
    STORAGE(ODCG); break;       // Port G: open drain configuration
    STORAGE(CNCON); break;      // Interrupt-on-change control
    STORAGE(CNEN); break;       // Input change interrupt enable
    STORAGE(CNPUE); break;      // Input pin pull-up enable

    /*-------------------------------------------------------------------------
     * UART 1.
     */
    STORAGE(U1RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 0);
        break;
    STORAGE(U1BRG); break;                      // Baud rate
    STORAGE(U1MODE); break;                     // Mode
    STORAGE(U1STA);                             // Status and control
        pic32_uart_poll_status(s, 0);
        break;
    STORAGE(U1TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U1MODECLR); *bufp = 0; break;
    STORAGE(U1MODESET); *bufp = 0; break;
    STORAGE(U1MODEINV); *bufp = 0; break;
    STORAGE(U1STACLR);  *bufp = 0; break;
    STORAGE(U1STASET);  *bufp = 0; break;
    STORAGE(U1STAINV);  *bufp = 0; break;
    STORAGE(U1BRGCLR);  *bufp = 0; break;
    STORAGE(U1BRGSET);  *bufp = 0; break;
    STORAGE(U1BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 2.
     */
    STORAGE(U2RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 1);
        break;
    STORAGE(U2BRG); break;                      // Baud rate
    STORAGE(U2MODE); break;                     // Mode
    STORAGE(U2STA);                             // Status and control
        pic32_uart_poll_status(s, 1);
        break;
    STORAGE(U2TXREG);   *bufp = 0; break;      // Transmit
    STORAGE(U2MODECLR); *bufp = 0; break;
    STORAGE(U2MODESET); *bufp = 0; break;
    STORAGE(U2MODEINV); *bufp = 0; break;
    STORAGE(U2STACLR);  *bufp = 0; break;
    STORAGE(U2STASET);  *bufp = 0; break;
    STORAGE(U2STAINV);  *bufp = 0; break;
    STORAGE(U2BRGCLR);  *bufp = 0; break;
    STORAGE(U2BRGSET);  *bufp = 0; break;
    STORAGE(U2BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 3.
     */
    STORAGE(U3RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 2);
        break;
    STORAGE(U3BRG); break;                      // Baud rate
    STORAGE(U3MODE); break;                     // Mode
    STORAGE(U3STA);                             // Status and control
        pic32_uart_poll_status(s, 2);
        break;
    STORAGE(U3TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U3MODECLR); *bufp = 0; break;
    STORAGE(U3MODESET); *bufp = 0; break;
    STORAGE(U3MODEINV); *bufp = 0; break;
    STORAGE(U3STACLR);  *bufp = 0; break;
    STORAGE(U3STASET);  *bufp = 0; break;
    STORAGE(U3STAINV);  *bufp = 0; break;
    STORAGE(U3BRGCLR);  *bufp = 0; break;
    STORAGE(U3BRGSET);  *bufp = 0; break;
    STORAGE(U3BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 4.
     */
    STORAGE(U4RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 3);
        break;
    STORAGE(U4BRG); break;                      // Baud rate
    STORAGE(U4MODE); break;                     // Mode
    STORAGE(U4STA);                             // Status and control
        pic32_uart_poll_status(s, 3);
        break;
    STORAGE(U4TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U4MODECLR); *bufp = 0; break;
    STORAGE(U4MODESET); *bufp = 0; break;
    STORAGE(U4MODEINV); *bufp = 0; break;
    STORAGE(U4STACLR);  *bufp = 0; break;
    STORAGE(U4STASET);  *bufp = 0; break;
    STORAGE(U4STAINV);  *bufp = 0; break;
    STORAGE(U4BRGCLR);  *bufp = 0; break;
    STORAGE(U4BRGSET);  *bufp = 0; break;
    STORAGE(U4BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 5.
     */
    STORAGE(U5RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 4);
        break;
    STORAGE(U5BRG); break;                      // Baud rate
    STORAGE(U5MODE); break;                     // Mode
    STORAGE(U5STA);                             // Status and control
        pic32_uart_poll_status(s, 4);
        break;
    STORAGE(U5TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U5MODECLR); *bufp = 0; break;
    STORAGE(U5MODESET); *bufp = 0; break;
    STORAGE(U5MODEINV); *bufp = 0; break;
    STORAGE(U5STACLR);  *bufp = 0; break;
    STORAGE(U5STASET);  *bufp = 0; break;
    STORAGE(U5STAINV);  *bufp = 0; break;
    STORAGE(U5BRGCLR);  *bufp = 0; break;
    STORAGE(U5BRGSET);  *bufp = 0; break;
    STORAGE(U5BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * UART 6.
     */
    STORAGE(U6RXREG);                           // Receive data
        *bufp = pic32_uart_get_char(s, 5);
        break;
    STORAGE(U6BRG); break;                      // Baud rate
    STORAGE(U6MODE); break;                     // Mode
    STORAGE(U6STA);                             // Status and control
        pic32_uart_poll_status(s, 5);
        break;
    STORAGE(U6TXREG);   *bufp = 0; break;       // Transmit
    STORAGE(U6MODECLR); *bufp = 0; break;
    STORAGE(U6MODESET); *bufp = 0; break;
    STORAGE(U6MODEINV); *bufp = 0; break;
    STORAGE(U6STACLR);  *bufp = 0; break;
    STORAGE(U6STASET);  *bufp = 0; break;
    STORAGE(U6STAINV);  *bufp = 0; break;
    STORAGE(U6BRGCLR);  *bufp = 0; break;
    STORAGE(U6BRGSET);  *bufp = 0; break;
    STORAGE(U6BRGINV);  *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 1.
     */
    STORAGE(SPI1CON); break;                    // Control
    STORAGE(SPI1CONCLR); *bufp = 0; break;
    STORAGE(SPI1CONSET); *bufp = 0; break;
    STORAGE(SPI1CONINV); *bufp = 0; break;
    STORAGE(SPI1STAT); break;                   // Status
    STORAGE(SPI1STATCLR); *bufp = 0; break;
    STORAGE(SPI1STATSET); *bufp = 0; break;
    STORAGE(SPI1STATINV); *bufp = 0; break;
    STORAGE(SPI1BUF);                           // Buffer
        *bufp = pic32_spi_readbuf(s, 0);
        break;
    STORAGE(SPI1BRG); break;                    // Baud rate
    STORAGE(SPI1BRGCLR); *bufp = 0; break;
    STORAGE(SPI1BRGSET); *bufp = 0; break;
    STORAGE(SPI1BRGINV); *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 2.
     */
    STORAGE(SPI2CON); break;                    // Control
    STORAGE(SPI2CONCLR); *bufp = 0; break;
    STORAGE(SPI2CONSET); *bufp = 0; break;
    STORAGE(SPI2CONINV); *bufp = 0; break;
    STORAGE(SPI2STAT); break;                   // Status
    STORAGE(SPI2STATCLR); *bufp = 0; break;
    STORAGE(SPI2STATSET); *bufp = 0; break;
    STORAGE(SPI2STATINV); *bufp = 0; break;
    STORAGE(SPI2BUF);                           // Buffer
        *bufp = pic32_spi_readbuf(s, 1);
        break;
    STORAGE(SPI2BRG); break;                    // Baud rate
    STORAGE(SPI2BRGCLR); *bufp = 0; break;
    STORAGE(SPI2BRGSET); *bufp = 0; break;
    STORAGE(SPI2BRGINV); *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 3.
     */
    STORAGE(SPI3CON); break;                    // Control
    STORAGE(SPI3CONCLR); *bufp = 0; break;
    STORAGE(SPI3CONSET); *bufp = 0; break;
    STORAGE(SPI3CONINV); *bufp = 0; break;
    STORAGE(SPI3STAT); break;                   // Status
    STORAGE(SPI3STATCLR); *bufp = 0; break;
    STORAGE(SPI3STATSET); *bufp = 0; break;
    STORAGE(SPI3STATINV); *bufp = 0; break;
    STORAGE(SPI3BUF);                           // SPIx Buffer
        *bufp = pic32_spi_readbuf(s, 2);
        break;
    STORAGE(SPI3BRG); break;                    // Baud rate
    STORAGE(SPI3BRGCLR); *bufp = 0; break;
    STORAGE(SPI3BRGSET); *bufp = 0; break;
    STORAGE(SPI3BRGINV); *bufp = 0; break;

    /*-------------------------------------------------------------------------
     * SPI 4.
     */
    STORAGE(SPI4CON); break;                    // Control
    STORAGE(SPI4CONCLR); *bufp = 0; break;
    STORAGE(SPI4CONSET); *bufp = 0; break;
    STORAGE(SPI4CONINV); *bufp = 0; break;
    STORAGE(SPI4STAT); break;                   // Status
    STORAGE(SPI4STATCLR); *bufp = 0; break;
    STORAGE(SPI4STATSET); *bufp = 0; break;
    STORAGE(SPI4STATINV); *bufp = 0; break;
    STORAGE(SPI4BUF);                           // Buffer
        *bufp = pic32_spi_readbuf(s, 3);
        break;
    STORAGE(SPI4BRG); break;                    // Baud rate
    STORAGE(SPI4BRGCLR); *bufp = 0; break;
    STORAGE(SPI4BRGSET); *bufp = 0; break;
    STORAGE(SPI4BRGINV); *bufp = 0; break;

    default:
        printf("--- Read 1f8%05x: peripheral register not supported\n",
            offset);
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- Read 1f8%05x: peripheral register not supported\n",
                offset);
        exit(1);
    }
    return *bufp;
}

static void io_write32(pic32_t *s, unsigned offset, unsigned data, const char **namep)
{
    unsigned *bufp = &VALUE(offset);

    switch (offset) {
    /*-------------------------------------------------------------------------
     * Bus matrix control registers.
     */
    WRITEOP(BMXCON); return;    // Bus Matrix Control
    STORAGE(BMXDKPBA); break;   // Data RAM kernel program base address
    STORAGE(BMXDUDBA); break;   // Data RAM user data base address
    STORAGE(BMXDUPBA); break;   // Data RAM user program base address
    STORAGE(BMXPUPBA); break;   // Program Flash user program base address
    READONLY(BMXDRMSZ);         // Data RAM memory size
    READONLY(BMXPFMSZ);         // Program Flash memory size
    READONLY(BMXBOOTSZ);        // Boot Flash size

    /*-------------------------------------------------------------------------
     * Interrupt controller registers.
     */
    WRITEOP(INTCON); return;    // Interrupt Control
    READONLY(INTSTAT);          // Interrupt Status
    WRITEOP(IPTMR);  return;    // Temporal Proximity Timer
    WRITEOP(IFS0); goto irq;    // IFS(0..2) - Interrupt Flag Status
    WRITEOP(IFS1); goto irq;
    WRITEOP(IFS2); goto irq;
    WRITEOP(IEC0); goto irq;    // IEC(0..2) - Interrupt Enable Control
    WRITEOP(IEC1); goto irq;
    WRITEOP(IEC2); goto irq;
    WRITEOP(IPC0); goto irq;    // IPC(0..11) - Interrupt Priority Control
    WRITEOP(IPC1); goto irq;
    WRITEOP(IPC2); goto irq;
    WRITEOP(IPC3); goto irq;
    WRITEOP(IPC4); goto irq;
    WRITEOP(IPC5); goto irq;
    WRITEOP(IPC6); goto irq;
    WRITEOP(IPC7); goto irq;
    WRITEOP(IPC8); goto irq;
    WRITEOP(IPC9); goto irq;
    WRITEOP(IPC10); goto irq;
    WRITEOP(IPC11); goto irq;
    WRITEOP(IPC12);
irq:    update_irq_status(s);
        return;

    /*-------------------------------------------------------------------------
     * Prefetch controller.
     */
    WRITEOP(CHECON); return;    // Prefetch Control

    /*-------------------------------------------------------------------------
     * System controller.
     */
    STORAGE(OSCCON); break;     // Oscillator Control
    STORAGE(OSCTUN); break;     // Oscillator Tuning
    STORAGE(DDPCON); break;     // Debug Data Port Control
    READONLY(DEVID);            // Device Identifier
    STORAGE(SYSKEY);            // System Key
        /* Unlock state machine. */
        if (s->syskey_unlock == 0 && VALUE(SYSKEY) == 0xaa996655)
            s->syskey_unlock = 1;
        if (s->syskey_unlock == 1 && VALUE(SYSKEY) == 0x556699aa)
            s->syskey_unlock = 2;
        else
            s->syskey_unlock = 0;
        break;
    STORAGE(RCON); break;       // Reset Control
    WRITEOP(RSWRST);            // Software Reset
        if (s->syskey_unlock == 2 && (VALUE(RSWRST) & 1)) {
            /* Reset CPU. */
            qemu_system_reset_request();

            /* Reset all devices */
            io_reset(s);
            pic32_sdcard_reset(s);
        }
        break;

    /*-------------------------------------------------------------------------
     * DMA controller.
     */
    WRITEOP(DMACON); return;    // DMA Control
    STORAGE(DMASTAT); break;    // DMA Status
    STORAGE(DMAADDR); break;    // DMA Address

    /*-------------------------------------------------------------------------
     * Analog to digital converter.
     */
    WRITEOP(AD1CON1); return;   // Control register 1
    WRITEOP(AD1CON2); return;   // Control register 2
    WRITEOP(AD1CON3); return;   // Control register 3
    WRITEOP(AD1CHS); return;    // Channel select
    WRITEOP(AD1CSSL); return;   // Input scan selection
    WRITEOP(AD1PCFG); return;   // Port configuration
    READONLY(ADC1BUF0);         // Result words
    READONLY(ADC1BUF1);
    READONLY(ADC1BUF2);
    READONLY(ADC1BUF3);
    READONLY(ADC1BUF4);
    READONLY(ADC1BUF5);
    READONLY(ADC1BUF6);
    READONLY(ADC1BUF7);
    READONLY(ADC1BUF8);
    READONLY(ADC1BUF9);
    READONLY(ADC1BUFA);
    READONLY(ADC1BUFB);
    READONLY(ADC1BUFC);
    READONLY(ADC1BUFD);
    READONLY(ADC1BUFE);
    READONLY(ADC1BUFF);

    /*--------------------------------------
     * USB registers.
     */
    STORAGE(U1OTGIR);           // OTG interrupt flags
        VALUE(U1OTGIR) = 0;
        return;
    STORAGE(U1OTGIE); break;    // OTG interrupt enable
    READONLY(U1OTGSTAT);        // Comparator and pin status
    STORAGE(U1OTGCON); break;   // Resistor and pin control
    STORAGE(U1PWRC); break;     // Power control
    STORAGE(U1IR);              // Pending interrupt
        VALUE(U1IR) = 0;
        return;
    STORAGE(U1IE); break;       // Interrupt enable
    STORAGE(U1EIR);             // Pending error interrupt
        VALUE(U1EIR) = 0;
        return;
    STORAGE(U1EIE); break;      // Error interrupt enable
    READONLY(U1STAT);           // Status FIFO
    STORAGE(U1CON); break;      // Control
    STORAGE(U1ADDR); break;     // Address
    STORAGE(U1BDTP1); break;    // Buffer descriptor table pointer 1
    READONLY(U1FRML);           // Frame counter low
    READONLY(U1FRMH);           // Frame counter high
    STORAGE(U1TOK); break;      // Host control
    STORAGE(U1SOF); break;      // SOF counter
    STORAGE(U1BDTP2); break;    // Buffer descriptor table pointer 2
    STORAGE(U1BDTP3); break;    // Buffer descriptor table pointer 3
    STORAGE(U1CNFG1); break;    // Debug and idle
    STORAGE(U1EP(0)); break;    // Endpoint control
    STORAGE(U1EP(1)); break;
    STORAGE(U1EP(2)); break;
    STORAGE(U1EP(3)); break;
    STORAGE(U1EP(4)); break;
    STORAGE(U1EP(5)); break;
    STORAGE(U1EP(6)); break;
    STORAGE(U1EP(7)); break;
    STORAGE(U1EP(8)); break;
    STORAGE(U1EP(9)); break;
    STORAGE(U1EP(10)); break;
    STORAGE(U1EP(11)); break;
    STORAGE(U1EP(12)); break;
    STORAGE(U1EP(13)); break;
    STORAGE(U1EP(14)); break;
    STORAGE(U1EP(15)); break;

    /*-------------------------------------------------------------------------
     * General purpose IO signals.
     */
    WRITEOP(TRISA); return;         // Port A: mask of inputs
    WRITEOPX(PORTA, LATA);          // Port A: write outputs
    WRITEOP(LATA);                  // Port A: write outputs
        pic32_gpio_write(s, 0, VALUE(LATA));
        return;
    WRITEOP(ODCA); return;          // Port A: open drain configuration
    WRITEOP(TRISB); return;         // Port B: mask of inputs
    WRITEOPX(PORTB, LATB);          // Port B: write outputs
    WRITEOP(LATB);                  // Port B: write outputs
        pic32_gpio_write(s, 1, VALUE(LATB));
        return;
    WRITEOP(ODCB); return;          // Port B: open drain configuration
    WRITEOP(TRISC); return;         // Port C: mask of inputs
    WRITEOPX(PORTC, LATC);          // Port C: write outputs
    WRITEOP(LATC);                  // Port C: write outputs
        pic32_gpio_write(s, 2, VALUE(LATC));
        return;
    WRITEOP(ODCC); return;          // Port C: open drain configuration
    WRITEOP(TRISD); return;         // Port D: mask of inputs
    WRITEOPX(PORTD, LATD);          // Port D: write outputs
    WRITEOP(LATD);                  // Port D: write outputs
        pic32_gpio_write(s, 3, VALUE(LATD));
        return;
    WRITEOP(ODCD); return;          // Port D: open drain configuration
    WRITEOP(TRISE); return;         // Port E: mask of inputs
    WRITEOPX(PORTE, LATE);          // Port E: write outputs
    WRITEOP(LATE);                  // Port E: write outputs
        pic32_gpio_write(s, 4, VALUE(LATE));
        return;
    WRITEOP(ODCE); return;          // Port E: open drain configuration
    WRITEOP(TRISF); return;         // Port F: mask of inputs
    WRITEOPX(PORTF, LATF);          // Port F: write outputs
    WRITEOP(LATF);                  // Port F: write outputs
        pic32_gpio_write(s, 5, VALUE(LATF));
        return;
    WRITEOP(ODCF); return;          // Port F: open drain configuration
    WRITEOP(TRISG); return;         // Port G: mask of inputs
    WRITEOPX(PORTG, LATG);          // Port G: write outputs
    WRITEOP(LATG);                  // Port G: write outputs
        pic32_gpio_write(s, 6, VALUE(LATG));
        return;
    WRITEOP(ODCG); return;          // Port G: open drain configuration
    WRITEOP(CNCON); return;         // Interrupt-on-change control
    WRITEOP(CNEN); return;          // Input change interrupt enable
    WRITEOP(CNPUE); return;         // Input pin pull-up enable

    /*-------------------------------------------------------------------------
     * UART 1.
     */
    STORAGE(U1TXREG);                               // Transmit
        pic32_uart_put_char(s, 0, data);
        break;
    WRITEOP(U1MODE);                                // Mode
        pic32_uart_update_mode(s, 0);
        return;
    WRITEOPR(U1STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 0);
        return;
    WRITEOP(U1BRG); return;                         // Baud rate
    READONLY(U1RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 2.
     */
    STORAGE(U2TXREG);                               // Transmit
        pic32_uart_put_char(s, 1, data);
        break;
    WRITEOP(U2MODE);                                // Mode
        pic32_uart_update_mode(s, 1);
        return;
    WRITEOPR(U2STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 1);
        return;
    WRITEOP(U2BRG); return;                         // Baud rate
    READONLY(U2RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 3.
     */
    STORAGE(U3TXREG);                               // Transmit
        pic32_uart_put_char(s, 2, data);
        break;
    WRITEOP(U3MODE);                                // Mode
        pic32_uart_update_mode(s, 2);
        return;
    WRITEOPR(U3STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 2);
        return;
    WRITEOP(U3BRG); return;                         // Baud rate
    READONLY(U3RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 4.
     */
    STORAGE(U4TXREG);                               // Transmit
        pic32_uart_put_char(s, 3, data);
        break;
    WRITEOP(U4MODE);                                // Mode
        pic32_uart_update_mode(s, 3);
        return;
    WRITEOPR(U4STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 3);
        return;
    WRITEOP(U4BRG); return;                         // Baud rate
    READONLY(U4RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 5.
     */
    STORAGE(U5TXREG);                               // Transmit
        pic32_uart_put_char(s, 4, data);
        break;
    WRITEOP(U5MODE);                                // Mode
        pic32_uart_update_mode(s, 4);
        return;
    WRITEOPR(U5STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 4);
        return;
    WRITEOP(U5BRG); return;                         // Baud rate
    READONLY(U5RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * UART 6.
     */
    STORAGE(U6TXREG);                               // Transmit
        pic32_uart_put_char(s, 5, data);
        break;
    WRITEOP(U6MODE);                                // Mode
        pic32_uart_update_mode(s, 5);
        return;
    WRITEOPR(U6STA,                                 // Status and control
        PIC32_USTA_URXDA | PIC32_USTA_FERR | PIC32_USTA_PERR |
        PIC32_USTA_RIDLE | PIC32_USTA_TRMT | PIC32_USTA_UTXBF);
        pic32_uart_update_status(s, 5);
        return;
    WRITEOP(U6BRG); return;                         // Baud rate
    READONLY(U6RXREG);                              // Receive

    /*-------------------------------------------------------------------------
     * SPI.
     */
    WRITEOP(SPI1CON);                               // Control
        pic32_spi_control(s, 0);
        return;
    WRITEOPR(SPI1STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI1BUF);                               // Buffer
        pic32_spi_writebuf(s, 0, data);
        return;
    WRITEOP(SPI1BRG); return;                       // Baud rate
    WRITEOP(SPI2CON);                               // Control
        pic32_spi_control(s, 1);
        return;
    WRITEOPR(SPI2STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI2BUF);                               // Buffer
        pic32_spi_writebuf(s, 1, data);
        return;
    WRITEOP(SPI2BRG); return;                       // Baud rate
    WRITEOP(SPI3CON);                               // Control
        pic32_spi_control(s, 2);
        return;
    WRITEOPR(SPI3STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI3BUF);                               // Buffer
        pic32_spi_writebuf(s, 2, data);
        return;
    WRITEOP(SPI3BRG); return;                       // Baud rate
    WRITEOP(SPI4CON);                               // Control
        pic32_spi_control(s, 3);
        return;
    WRITEOPR(SPI4STAT, ~PIC32_SPISTAT_SPIROV);      // Status
        return;                                     // Only ROV bit is writable
    STORAGE(SPI4BUF);                               // Buffer
        pic32_spi_writebuf(s, 3, data);
        return;
    WRITEOP(SPI4BRG); return;                       // Baud rate

    default:
        printf("--- Write %08x to 1f8%05x: peripheral register not supported\n",
            data, offset);
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- Write %08x to 1f8%05x: peripheral register not supported\n",
                data, offset);
        exit(1);
readonly:
        printf("--- Write %08x to %s: readonly register\n",
            data, *namep);
        if (qemu_loglevel_mask(CPU_LOG_INSTR))
            fprintf(qemu_logfile, "--- Write %08x to %s: readonly register\n",
                data, *namep);
        *namep = 0;
        return;
    }
    *bufp = data;
}

static uint64_t pic32_io_read(void *opaque, hwaddr addr, unsigned bytes)
{
    pic32_t *s = opaque;
    uint32_t offset = addr & 0xfffff;
    const char *name = "???";
    uint32_t data = 0;

    data = io_read32(s, offset & ~3, &name);
    switch (bytes) {
    case 1:
        if ((offset &= 3) != 0) {
            // Unaligned read.
            data >>= offset * 8;
        }
        data = (uint8_t) data;
        if (qemu_loglevel_mask(CPU_LOG_INSTR)) {
            fprintf(qemu_logfile, "--- I/O Read  %02x from %s\n", data, name);
        }
        break;
    case 2:
        if (offset & 2) {
            // Unaligned read.
            data >>= 16;
        }
        data = (uint16_t) data;
        if (qemu_loglevel_mask(CPU_LOG_INSTR)) {
            fprintf(qemu_logfile, "--- I/O Read  %04x from %s\n", data, name);
        }
        break;
    default:
        if (qemu_loglevel_mask(CPU_LOG_INSTR)) {
            fprintf(qemu_logfile, "--- I/O Read  %08x from %s\n", data, name);
        }
        break;
    }
    return data;
}

static void pic32_io_write(void *opaque, hwaddr addr, uint64_t data, unsigned bytes)
{
    pic32_t *s = opaque;
    uint32_t offset = addr & 0xfffff;
    const char *name = "???";

    // Fetch data and align to word format.
    switch (bytes) {
    case 1:
        data = (uint8_t) data;
        data <<= (offset & 3) * 8;
        break;
    case 2:
        data = (uint16_t) data;
        data <<= (offset & 2) * 8;
        break;
    }
    io_write32(s, offset & ~3, data, &name);

    if (qemu_loglevel_mask(CPU_LOG_INSTR) && name != 0) {
        fprintf(qemu_logfile, "--- I/O Write %08x to %s\n",
            (uint32_t) data, name);
    }
}

static const MemoryRegionOps pic32_io_ops = {
    .read       = pic32_io_read,
    .write      = pic32_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;
    int i;

    cpu_reset(CPU(cpu));

    /* Adjust the initial configuration for M4K core. */
    env->CP0_IntCtl = 0;
    env->CP0_Debug = (1 << CP0DB_CNT) | (3 << CP0DB_VER);
    for (i=0; i<7; i++)
        env->CP0_WatchHi[i] = 0;
}

static void store_byte(unsigned address, unsigned char byte)
{
    if (address >= PROGRAM_FLASH_START &&
        address < PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE)
    {
        //printf("Store %02x to program memory %08x\n", byte, address);
        prog_ptr[address & 0xfffff] = byte;
    }
    else if (address >= BOOT_FLASH_START &&
             address < BOOT_FLASH_START + BOOT_FLASH_SIZE)
    {
        //printf("Store %02x to boot memory %08x\n", byte, address);
        boot_ptr[address & 0xffff] = byte;
    }
    else {
        printf("Bad hex file: incorrect address %08X, must be %08X-%08X or %08X-%08X\n",
            address, PROGRAM_FLASH_START,
            PROGRAM_FLASH_START + PROGRAM_FLASH_SIZE - 1,
            BOOT_FLASH_START, BOOT_FLASH_START + BOOT_FLASH_SIZE - 1);
        exit(1);
    }
}

/*
 * Ignore ^C and ^\ signals and pass these characters to the target.
 */
static void pic32_pass_signal_chars(void)
{
    struct termios tty;

    tcgetattr(0, &tty);
    tty.c_lflag &= ~ISIG;
    tcsetattr(0, TCSANOW, &tty);
}

static void pic32_init(MachineState *machine, int board_type)
{
    const char *cpu_model = machine->cpu_model;
    unsigned ram_size = DATA_MEM_SIZE;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *ram_main = g_new(MemoryRegion, 1);
    MemoryRegion *prog_mem = g_new(MemoryRegion, 1);
    MemoryRegion *boot_mem = g_new(MemoryRegion, 1);
    MemoryRegion *io_mem = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    CPUMIPSState *env;

    DeviceState *dev = qdev_create(NULL, TYPE_MIPS_PIC32);
    pic32_t *s = OBJECT_CHECK(pic32_t, dev, TYPE_MIPS_PIC32);
    s->board_type = board_type;
    s->stop_on_reset = 1;               /* halt simulation on soft reset */
    s->iomem = g_malloc0(IO_MEM_SIZE);  /* backing storage for I/O area */

    qdev_init_nofail(dev);

    /* Init CPU. */
    if (! cpu_model) {
        cpu_model = "M4K";
    }
    printf("Board: %s\n", board_name[board_type]);
    if (qemu_logfile)
        fprintf(qemu_logfile, "Board: %s\n", board_name[board_type]);

    printf("Processor: %s\n", cpu_model);
    if (qemu_logfile)
        fprintf(qemu_logfile, "Processor: %s\n", cpu_model);

    cpu = cpu_mips_init(cpu_model);
    if (! cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    s->cpu = cpu;
    env = &cpu->env;

    /* Register RAM */
    printf("RAM size: %u kbytes\n", ram_size / 1024);
    if (qemu_logfile)
        fprintf(qemu_logfile, "RAM size: %u kbytes\n", ram_size / 1024);

    memory_region_init_ram(ram_main, NULL, "kernel.ram",
        ram_size, &error_abort);
    vmstate_register_ram_global(ram_main);
    memory_region_add_subregion(system_memory, DATA_MEM_START, ram_main);

    /* Alias for user space 96 kbytes.
     * For MX family only. */
    MemoryRegion *ram_user = g_new(MemoryRegion, 1);
    memory_region_init_alias(ram_user, NULL, "user.ram",
        ram_main, 0x8000, ram_size - 0x8000);
    memory_region_add_subregion(system_memory, USER_MEM_START + 0x8000, ram_user);

    /* Special function registers. */
    memory_region_init_io(io_mem, NULL, &pic32_io_ops, s,
                          "io", IO_MEM_SIZE);
    memory_region_add_subregion(system_memory, IO_MEM_START, io_mem);

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
    prog_ptr = memory_region_get_ram_ptr(prog_mem);
    boot_ptr = memory_region_get_ram_ptr(boot_mem);
    if (bios_name)
        pic32_load_hex_file(bios_name, store_byte);
    pic32_load_hex_file(machine->kernel_filename, store_byte);

    memory_region_set_readonly(boot_mem, true);
    memory_region_set_readonly(prog_mem, true);
    memory_region_add_subregion(system_memory, BOOT_FLASH_START, boot_mem);
    memory_region_add_subregion(system_memory, PROGRAM_FLASH_START, prog_mem);

    /* Init internal devices */
    s->irq_raise = irq_raise;
    s->irq_clear = irq_clear;
    qemu_register_reset(main_cpu_reset, cpu);

    /* Setup interrupt controller in EIC mode. */
    env->CP0_Config3 |= 1 << CP0C3_VEIC;
    cpu_mips_irq_init_cpu(env);
    env->eic_timer_irq = pic32_timer_irq;
    env->eic_soft_irq = pic32_soft_irq;
    env->eic_context = s;

    /* CPU runs at 80MHz.
     * Count register increases at half this rate. */
    cpu_mips_clock_init(env, 40*1000*1000);

    /*
     * Initialize board-specific parameters.
     */
    int cs0_port, cs0_pin, cs1_port, cs1_pin;
    switch (board_type) {
    default:
    case BOARD_MAX32:
        BOOTMEM(DEVCFG0) = 0xffffff7f;      // Max32 board
        BOOTMEM(DEVCFG1) = 0x5bfd6aff;      // console on UART1
        BOOTMEM(DEVCFG2) = 0xd979f8f9;
        BOOTMEM(DEVCFG3) = 0xffff0722;
        VALUE(DEVID)     = 0x04307053;      // MX795F512L
        VALUE(OSCCON)    = 0x01453320;      // external oscillator 8MHz
        s->sdcard_spi_port = 3;             // SD card at SPI4,
        cs0_port = 3;  cs0_pin = 3;         // select0 at D3,
        cs1_port = 3;  cs1_pin = 4;         // select1 at D4
        break;
    case BOARD_MAXIMITE:
        BOOTMEM(DEVCFG0) = 0xffffff7f;      // TODO: get real data from Maximite board
        BOOTMEM(DEVCFG1) = 0x5bfd6aff;      // console on UART1
        BOOTMEM(DEVCFG2) = 0xd979f8f9;
        BOOTMEM(DEVCFG3) = 0xffff0722;
        VALUE(DEVID)     = 0x04307053;      // MX795F512L
        VALUE(OSCCON)    = 0x01453320;      // external oscillator 8MHz
        s->sdcard_spi_port = 3;             // SD card at SPI4,
        cs0_port = 4;  cs0_pin = 0;         // select0 at E0,
        cs1_port = -1; cs1_pin = -1;        // select1 not available
        break;
    case BOARD_EXPLORER16:
        BOOTMEM(DEVCFG0) = 0xffffff7f;      // TODO: get real data from Explorer16 board
        BOOTMEM(DEVCFG1) = 0x5bfd6aff;      // console on UART2
        BOOTMEM(DEVCFG2) = 0xd979f8f9;
        BOOTMEM(DEVCFG3) = 0xffff0722;
        VALUE(DEVID)     = 0x04307053;      // MX795F512L
        VALUE(OSCCON)    = 0x01453320;      // external oscillator 8MHz
        s->sdcard_spi_port = 0;             // SD card at SPI1,
        cs0_port = 1;  cs0_pin = 1;         // select0 at B1,
        cs1_port = 1;  cs1_pin = 2;         // select1 at B2
        break;
    }

    /* UARTs */
    pic32_uart_init(s, 0, PIC32_IRQ_U1E, U1STA, U1MODE);
    pic32_uart_init(s, 1, PIC32_IRQ_U2E, U2STA, U2MODE);
    pic32_uart_init(s, 2, PIC32_IRQ_U3E, U3STA, U3MODE);
    pic32_uart_init(s, 3, PIC32_IRQ_U4E, U4STA, U4MODE);
    pic32_uart_init(s, 4, PIC32_IRQ_U5E, U5STA, U5MODE);
    pic32_uart_init(s, 5, PIC32_IRQ_U6E, U6STA, U6MODE);

    /* SPIs */
    pic32_spi_init(s, 0, PIC32_IRQ_SPI1E, SPI1CON, SPI1STAT);
    pic32_spi_init(s, 1, PIC32_IRQ_SPI2E, SPI2CON, SPI2STAT);
    pic32_spi_init(s, 2, PIC32_IRQ_SPI3E, SPI3CON, SPI3STAT);
    pic32_spi_init(s, 3, PIC32_IRQ_SPI4E, SPI4CON, SPI4STAT);

    /*
     * Load SD card images.
     * Use options:
     *      -sd filename
     * or   -hda filename
     * and  -hdb filename
     */
    const char *sd0_file = 0, *sd1_file = 0;
    DriveInfo *dinfo = drive_get(IF_IDE, 0, 0);
    if (dinfo) {
        sd0_file = qemu_opt_get(dinfo->opts, "file");
        dinfo->is_default = 1;

        dinfo = drive_get(IF_IDE, 0, 1);
        if (dinfo) {
            sd1_file = qemu_opt_get(dinfo->opts, "file");
            dinfo->is_default = 1;
        }
    }
    if (! sd0_file) {
        dinfo = drive_get(IF_SD, 0, 0);
        if (dinfo) {
            sd0_file = qemu_opt_get(dinfo->opts, "file");
            dinfo->is_default = 1;
        }
    }
    pic32_sdcard_init(s, 0, "sd0", sd0_file, cs0_port, cs0_pin);
    pic32_sdcard_init(s, 1, "sd1", sd1_file, cs1_port, cs1_pin);

    io_reset(s);
    pic32_sdcard_reset(s);
    pic32_pass_signal_chars();
}

static void pic32_init_max32(MachineState *machine)
{
    pic32_init(machine, BOARD_MAX32);
}

static void pic32_init_maximite(MachineState *machine)
{
    pic32_init(machine, BOARD_MAXIMITE);
}

static void pic32_init_explorer16(MachineState *machine)
{
    pic32_init(machine, BOARD_EXPLORER16);
}

static int pic32_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void pic32_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = pic32_sysbus_device_init;
}

static const TypeInfo pic32_device = {
    .name          = TYPE_MIPS_PIC32,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pic32_t),
    .class_init    = pic32_class_init,
};

static void pic32_register_types(void)
{
    type_register_static(&pic32_device);
}

static QEMUMachine pic32_board[3] = {
    {
        .name       = "pic32mx7-max32",
        .desc       = "PIC32MX7 microcontroller on chipKIT Max32 board",
        .init       = pic32_init_max32,
        .max_cpus   = 1,
    },
    {
        .name       = "pic32mx7-maximite",
        .desc       = "PIC32MX7 microcontroller on Geoff's Maximite board",
        .init       = pic32_init_maximite,
        .max_cpus   = 1,
    },
    {
        .name       = "pic32mx7-explorer16",
        .desc       = "PIC32MX7 microcontroller on Microchip Explorer-16 board",
        .init       = pic32_init_explorer16,
        .max_cpus   = 1,
    },
};

static void pic32_machine_init(void)
{
    qemu_register_machine(&pic32_board[0]);
    qemu_register_machine(&pic32_board[1]);
    qemu_register_machine(&pic32_board[2]);
}

type_init(pic32_register_types)
machine_init(pic32_machine_init);

#endif /* !TARGET_MIPS64 && !TARGET_WORDS_BIGENDIAN */
