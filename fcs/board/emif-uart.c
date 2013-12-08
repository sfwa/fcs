/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <assert.h>

#include <c6x.h>

#include "../c66x-csl/ti/csl/cslr.h"
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_emif16.h"
#include "../c66x-csl/ti/csl/cslr_tpcc.h"
#include "../c66x-csl/ti/csl/cslr_tmr.h"
#include "../c66x-csl/ti/csl/cslr_gpio.h"

#include "board.h"
#include "emif-uart.h"

/*
Use EDMA for EMIFs, and run without FIFOs. Use interrupts for read, and rate
limit in software for writes (1 byte per 20us or so).

For an external UART, on startup we configure it with FIFOs disabled, an
RHR interrupt, and no THR interrupt. The RHR interrupt triggers an
AB-synchronised DMA transfer with ACNT = 1, BCNT = 1 and CCNT > 2. When
CCNT gets too low we increase it; the number of bytes read is indicated by
the change in CCNT.

For TX in external UARTs, we set up a timer firing every ~ 1.5 byte
periods (1/16th of baud rate), which triggers an AB-synchronised transfer
with ACNT = 1, BCNT = 1 and CCNT = number of bytes to send. The same
output is sent to both external UARTs so only one needs to be written.

EMIF UART crystal is 14.7456MHz. The EMIF is clocked at CPU_FREQ/6 (166.7MHz).

Connections are:

EMIF signal         UART signal        Description
D[7:0]              D[7:0]             8-bit parallel data bus
A0                  A2                 Address bus
A22                 A0                 Address bus
A23                 A1                 Address bus
/OE                 /IOR
/WE                 /IOW
/CS1                /CSA               UART A select (active low)
/CS2                /CSB               UART B select (active low)
GPIOn               /INTA
GPIOn               /INTB
GPIOn               RESET              UART reset (active high)

Interface implementation follows the example for 8-bit ASRAM/NOR interface
in section 2.5 of SPRUGZ3A.

Timing is as follows:

EMIF parameter      UART parameter     EMIF setting         UART spec value
SETUP               tAS                0+1 cycles           5ns
STROBE              tCS/tRD            11+1 cycles          60ns
HOLD                tDY                7+1 cycles           60ns
TURNAROUND          -                  -                    -

Since each cycle is 6ns, EMIF SETUP is set to 0 (+1 internally), EMIF STROBE
is set to 11 (+ 1 internally) and EMIF HOLD is set to 7 (+1 internally).

The mapping isn't exactly 1:1, but basically tRD + tDY must be > 120ns, SETUP
must be > 5ns, and STROBE must be > 60ns.

Write timings are the same.
*/

/*
See int-uart.c for a detailed description of the EDMA3 configuration. This
file only includes descriptions of the differences in configuration between
the internal and external UARTs.
*/
static volatile CSL_TpccRegs* edma3 = (CSL_TpccRegs*)CSL_EDMA2CC_REGS;

/* Pointer to global EMIF16 object */
static volatile CSL_Emif16Regs* emif16 = (CSL_Emif16Regs*)CSL_EMIF16_REGS;

/*
Pointer to global GPIO peripheral. The GPIO peripheral has a few global config
options, and registers for four GPIO banks. The GPIO bank number is the GPIO
index divided by 32 and rounded down, while the position of the control bit is
the GPIO index mod 32.

SPRUGV1 appears to be incorrect in relation to the C6657, in that the upper
16 bits of the GPIO control registers are marked as reserved. The C6657 CSL
writes to them though, and TI support say (seemingly with some trepidation)
that the CSL is more likely to be correct than the documentation.

This is of course largely irrelevant to us as we only need GPIO0 and GPIO1.
*/
static volatile CSL_GpioRegs* gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;

/*
Pointer to TIMER4 and TIMER5 peripherals. These are configured in 64-bit mode
(the default), with continuous mode enabled. The configured period is equal to
the (input clock frequency / baud rate) * 16.

This means that the maximum TX throughput for a 230400 baud link is 14.4KB/s.
*/
static volatile CSL_TmrRegs* timer[2] = {
    (CSL_TmrRegs*)CSL_TIMER_4_REGS,
    (CSL_TmrRegs*)CSL_TIMER_5_REGS
};

/*
GPINT0 and GPINT1 are used as RX events -- we configure the external UART to
send an interrupt for each byte received.

For TX events, we set a timer that transfers each byte at an appropriate rate
(depending on the configured baud rate of the UART). The LO signals of
TIMER4 and TIMER5 are used.
*/
static uint16_t rx_edma_event[2] = { 6u, 7u },
                tx_edma_event[2] = { 22u, 24u };

static uint16_t rx_last_buf_size[2] = { 0, 0 },
                tx_last_buf_size[2] = { 0, 0 };

static uint32_t uart_baud[2] = { 115200u, 115200u };

/*
Configuration structure for the UART. This doesn't actually match the memory
map, because we drive everything through DMA and need to perform writes in a
particular sequence to get access to some of the registers.

Re-configuring the UART involves setting up a PaRAM set pushing LCR[7] high,
then setting the DLL/DLM/DLD bytes, then clearing LCR[7], then configuring
each byte in order: IER, FCR, LCR, MCR. This means 4 chained PaRAM sets, and
about 1188 cycles.

It may actually be easier just to set each byte via the memory map -- check
performance to make sure it's as expected.
*/
struct emif16_xr16m752_uart_config_t {
    /* These registers should be configured when LCR[7] is high. */

    /* Divisor latch LSB */
    uint8_t DLL;
    /* Divisor latch MSB */
    uint8_t DLM;
    /* Divisor latch fractional component */
    uint8_t DLD;

    /* These registers should be configured when LCR[7] is low. */

    /*
    IER: Interrupt Enable Register

    Bit   Field          Value         Description
    7     CTS_INT_EN     0             Only valid when EFR[4] = 1. Unused.
    6     RTS_INT_EN     0             Only valid when EFR[4] = 1. Unused.
    5     XOFF_INT_EN    0             Only valid when EFR[4] = 1. Unused.
    4     SLEEP_MODE_EN  0             Only valid when EFR[4] = 1. Unused.
    3     MODEM_INT_EN                 Modem status interrupt enable.
                                       0 = disabled
                                       1 = enabled
    2     RX_LINE_STAT_INT_EN          RX line status interrupt enable.
                                       0 = disabled
                                       1 = enabled
    1     TX_EMPTY_INT_EN              TX empty interrupt enable.
                                       0 = disabled
                                       1 = enabled
    0     RX_DATA_INT_EN               RX data interrupt enable.
                                       0 = disabled
                                       1 = enabled
    */
    uint8_t IER;

    /*
    FCR: FIFO Control Register

    Bit   Field          Value         Description
    7:6   RXFIFO_TRIGGER               RX FIFO trigger
    5:4   TXFIFO_TRIGGER 0             Only valid when EFR[4] = 1. Unused.
    3     DMAMODE_EN                   DMA mode enable.
                                       0 = disabled
                                       1 = enabled
    2     TXFIFO_RESET                 TX FIFO reset.
                                       0 = ?
                                       1 = ?
    1     RXFIFO_RESET                 RX FIFO reset.
                                       0 = ?
                                       1 = ?
    0     FIFO_EN                      FIFO enable.
                                       0 = disabled
                                       1 = enabled
    */
    uint8_t FCR;

    /*
    LCR: Line Control Register

    Bit   Field          Value         Description
    7     DIVISOR_EN                   Enable access to DLL, DLM and DLH
                                       instead of RHR, IER and FCR.
                                       0 = disabled (no access to latch)
                                       1 = enabled (access to latch)
    6     SET_TX_BREAK                 Set TX break.
                                       0 = no break condition
                                       1 = break condition
    5     SET_PARITY                   Set parity.
                                       0 = parity not forced
                                       1 = parity forced to 1 if even, 0 if
                                           odd
    4     EVEN_PARITY                  Even parity.
                                       0 = odd parity
                                       1 = even parity
    3     PARITY_EN                    Parity enable.
                                       0 = no parity
                                       1 = parity
    2     STOP_BITS                    Stop bits.
                                       0 = 1 stop bit
                                       1 = 1.5 stop bits for WORD_LEN=0, 2
                                           otherwise
    1:0   WORD_LEN                     Word length.
                                       0 = 5 bits
                                       1 = 6 bits
                                       2 = 7 bits
                                       3 = 8 bits
    */
    uint8_t LCR;

    /*
    MCR: Modem Control Register

    Bit   Field          Value         Description
    7     PRESCALER_SEL  0             Clock prescaler select. Only valid when
                                       EFR[4] = 1. Unused.
    6     TCL_TLR_EN     0             TCL and TLR enable. Only valid when
                                       EFR[4] = 1. Unused.
    5     XON_ANY        0             XON any. Only valid when EFR[4] = 1.
                                       Unused.
    4     LOOPBACK_EN                  Loop TX back to RX when enabled.
                                       0 = loopback disabled (normal)
                                       1 = loopback enabled
    3     INT_OE                       OP2#/Interrupt output enable.
    2     FIFO_RDY                     OP1#/FIFO ready.
    1     RTS_OC                       RTS# output control.
    0     DTR_OC                       DTR# output control.
    */
    uint8_t MCR;

    /* There are a bunch of other registers but we don't use them */
};

#define XR16M752_RHR 0x0u
#define XR16M752_THR 0x0u
#define XR16M752_DLL 0x0u
#define XR16M752_DLM 0x1u
#define XR16M752_DLD 0x2u
#define XR16M752_IER 0x1u
#define XR16M752_ISR 0x2u
#define XR16M752_FCR 0x2u
#define XR16M752_LCR 0x3u
#define XR16M752_MCR 0x4u
#define XR16M752_LSR 0x5u
#define XR16M752_MSR 0x6u
#define XR16M752_SPR 0x7u
#define XR16M752_TCR 0x6u
#define XR16M752_TLR 0x7u
#define XR16M752_FIFO_RDY 0x7u
#define XR16M752_EFR 0x2u

/*
Logical addresses for EMIF16 CE1 and CE2 data space.

In TI's EMIF16 addressing convention, A0 is always 32-bit aligned, and A23:22
are byte selects in the case of an 8-bit interface. Since EMIF A22 is wired to
UART A0, EMIF A23 is wired to UART A1, and EMIF A0 is wired to UART A2, we can
just access the UARTs as regular bytes and the EMIF will operate
transparently.
*/
#define EMIF16_UART0_BASE_ADDR 0x74000000
#define EMIF16_UART1_BASE_ADDR 0x78000000

static struct emif16_xr16m752_uart_config_t uart[2];
static volatile uint8_t *uart_regs[2] = {
    (uint8_t*)EMIF16_UART0_BASE_ADDR,
    (uint8_t*)EMIF16_UART1_BASE_ADDR
};

static void _fcs_emif_uart_write_config(uint8_t uart_idx);

static void _fcs_emif_uart_write_config(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Configuring the UART involves the following steps:
    - Set LCR[7]
    - Write DLL, DLM and DLD
    - Clear LCR[7] / write configured LCR
    - Write IER, FCR, [LCR,] MCR

    We could do this via DMA (4 PaRAM sets in a chained transfer) but since
    each write takes < 150ns, we're looking at ~1350 cycles maximum to
    configure by just writing each value to the appropriate EMIF location.

    Obviously we couldn't re-configure the UART hundreds of times in a single
    tick, but once or twice is probably fine.
    */

    volatile uint8_t *restrict uart_mem = uart_regs[uart_idx];

    uart_mem[XR16M752_LCR] = 0x80u;
    uart_mem[XR16M752_DLL] = uart[uart_idx].DLL;
    uart_mem[XR16M752_DLM] = uart[uart_idx].DLM;
    uart_mem[XR16M752_DLD] = uart[uart_idx].DLD;
    uart_mem[XR16M752_LCR] = uart[uart_idx].LCR & 0x7Fu;
    uart_mem[XR16M752_IER] = uart[uart_idx].IER;
    uart_mem[XR16M752_FCR] = uart[uart_idx].FCR;
    uart_mem[XR16M752_MCR] = uart[uart_idx].MCR;
}

void fcs_emif_uart_reset(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Configure the EMIF CE1 (for UART 0) or CE2 (for UART 1), via A1CR and A2CR
    respectively.

    A0CR/A1CR/A2CR/A3CR: Async n Config Register

    Bit   Field          Value         Description
    31    SS             0             Select strobe (SS) mode.
                                       0 = select strobe mode disabled
                                       1 = select strobe mode enabled
    30    EW             0             Extended wait (EW) mode.
                                       0 = extended wait disabled
                                       1 = extended wait enabled (via WAIT)
    29:26 W_SETUP        Fh            Write strobe setup cycles. The write
                                       strobe setup time will be 1 + this
                                       number (from /CE assert to /WE assert).
    25:20 W_STROBE       3Fh           Write strobe duration cycles. /WE will
                                       be active for 1 + this number of cycles
                                       (must not be zero when EW = 1).
    19:17 W_HOLD         7             Write strobe hold cycles. /CE etc will
                                       be held for 1 + this number of cycles
                                       after /WE has been deasserted.
    16:13 R_SETUP        Fh            Read strobe setup cycles. The read
                                       strobe setup time will be 1 + this
                                       number (from /CE asert to /OE assert).
    12:7  R_STROBE       3Fh           Read strobe duration cycles. /OE will
                                       be active for 1 + this number of cycles
                                       (must not be 0 when EW = 1).
    6:4   R_HOLD         7             Read strobe hold cycles. /CE etc will
                                       be held for 1 + this number of cycles
                                       after /OE has been deasserted.
    3:2   TA             3             Turnaround cycles. The number of cycles
                                       between the end of one access and the
                                       start of the next will be 1 + this
                                       value, provided the two accesses are
                                       of different types (e.g. write -> read,
                                       or read -> write).
    1:0   ASIZE                        Width of data bus.
                                       0 = 8-bit
                                       1 = 16-bit

    (Note that all cycle counts above are based on the EMIF16 clock rate,
    which is 1/6th the CPU clock rate.)

    We're using ASIZE=0 (8-bit), TA=0, R_HOLD=11, R_STROBE=7, R_SETUP=0,
    write setup/strobe/hold the same.
    */
    emif16->A1CR = (0x7u << 4) + (0xAu << 7) + (0x7u << 17) + (0xAu << 20);

    /*
    PMCR is the Page Mode Control Register. We're not using NOR flash, so set
    it all to 0
    */
    emif16->PMCR = 0;

    /* EDMA3 reset */
    edma3->TPCC_EECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_EECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_SECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_SECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(rx_edma_event[uart_idx],
                               rx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(tx_edma_event[uart_idx],
                               tx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;

    /*
    Calculate baud rate divisor. The XR16M752 has a 14.7456MHz clock input,
    which is divided by 1 or 4 by the prescaler (MCR bit 7).

    The BRG divides the prescaler output by 1 to (2^16 - 0.0625) in increments
    of 0.0625; that is then used as the input for a 16x, 8x or 4x sampling
    bit clock.

    The DLL and DLM registers make up the integer part of the divisor, and
    the lower niblle of the DLD register makes up the fractional part of the
    divisor (from 0 to 0.9375, 0000 to 1111).

    The following equation (from page 14 of the XR16M752 series datasheet) is
    used to calculate DLM, DLL and DLD:

        divisor = (input_freq / prescaler) / (baud * oversampling_rate)
        DLM = floor(divisor) >> 8
        DLL = floor(divisor) & 0xFF
        DLD = round((divisor - floor(divisor)) * 16)

    where prescaler = 1 or 4, and oversampling_rate = 4, 8 or 16.

    In this application we only need fairly low rates of 57600 for the Piksi
    and 230400 for the CPU, so we'll use oversampling_rate = 16.
    */
    assert(2400 <= uart_baud[uart_idx] && uart_baud[uart_idx] <= 3000000);

    float divisor = 14745600.0f / (float)(uart_baud[uart_idx] * 16);
    uint16_t divisor_floor = (uint16_t)divisor, dld;
    dld = (uint16_t)((divisor - (float)divisor_floor) * 16.0 + 0.5);
    assert(dld < 0x10u);

    /* Configure the divisor latch values */
    uart[uart_idx].DLM = (divisor_floor >> 8) & 0xFFu;
    uart[uart_idx].DLL = divisor_floor & 0xFFu;
    uart[uart_idx].DLD = dld & 0x0Fu;

    /*
    See comments for `struct emif16_xr16m752_uart_config_t` above for register
    details. Here, we want to configure the following:
    - RX interrupt on data
    - Loopback if debugging
    - 8 bits, no parity, 1 stop bit
    -> MCR = 0x18 for loopback, 0x08 without
    */
    uart[uart_idx].IER = 0x01u; /* RX interrupt only */
    uart[uart_idx].MCR = 0x08u; /* 0x18 for loopback + INT_OE */
    uart[uart_idx].LCR = 0x03u; /* 8 bit, 1 stop bit, no parity */

    /* Send configuration to the UART */
    _fcs_emif_uart_write_config(uart_idx);

    /*
    Enable interrupts on the GPIO bank -- set lowest bit (only one in that
    register)
    */
    gpio->BINTEN = 1u;

    /*
    Each GPIO bank has DIR, OUT_DATA, SET_DATA, CLR_DATA, IN_DATA,
    SET_RIS_TRIG, CLR_RIS_TRIG, SET_FAL_TRIG and CLR_FAL_TRIG. Those registers
    are 32 bits wide, one bit per pin (31-0 maps to GPIO31-GPIO0). DATA is 0
    low, 1 high; DIR is 0 for output, and 1 for input. The RIS_TRIG and
    FAL_TRIG register pairs determine whether the GPIO interrupts are
    triggered on the rising edge or falling edge, or both (write 1 to
    SET_*_TRIG to enable triggering, write 1 to CLR_*_TRIG to disable it).

    Don't bother working out exactly which bit to set based on uart_idx, it's
    as quick to set both.

    We want to trigger on the negative-going edge, since the UART interrupts
    are active low.
    */
    gpio->BANK_REGISTERS[0].DIR |= 0x3u;
    gpio->BANK_REGISTERS[0].SET_FAL_TRIG |= 0x3u;

    /*
    For 64-bit timers in continuous mode, we need TCR ENAMODE = 0x2u and
    TGCR TIMHIRS = TGCR TIMLORS = 1 (see SPRUGV5A table 3-2).

    Since we're using the internal clock without gating, we need TCR CLKSRC =
    TCR TIEN_LO = 0.

    We want to ignore emulation events, so set FREE = 1 in EMUMGT_CLKSPD
    (SOFT is a don't-care).

    The timer needs to be stopped before writing to PRDLO, PRDHI, TGCR (except
    TIMLORS and TIMHIRS) and TCR (except ENAMODE). We can stop by setting
    TCR ENAMODE = 0.

    The initialization process is as follows:
    - Write 0 to CNTHI:CNTLO and RELHI:RELLO; write desired period to
      PRDHI:PRDLO
    - Write 0 to TDDRHI and PSCHI to disable prescaler
    - Set control bits in TCR/TGCR
    - Set ENAMODE to 2u

    TCR: Timer Control Register (SPRUGV5A section 5.5)

    Bit   Field          Value         Description
    31:27 Reserved
    26    READRSTMODE_HI               Read reset mode enable. Only in 32-bit
                                       unchained (not used).
    25:24 Reserved
    23:22 ENAMODE_HI                   Timer mode for TIMHI.
                                       0 = disabled
                                       1 = one-time mode (stop after period is
                                           reached)
                                       2 = continuous mode (reset counter once
                                           period is reached)
                                       3 = continuous mode with period reload
                                           (reset counter to 0 but reload the
                                           period from RELHI:RELLO)
    21:20 PWID_HI                      Pulse width for TIMHI output.
                                       0 = 1 clock cycle
                                       1 = 2 cycles
                                       2 = 3 cycles
                                       3 = 4 cycles
    19    CP_HI                        Clock/pulse mode bit for TIMHI.
                                       0 = pulse mode
                                       1 = clock mode (50% duty cycle)
    17    INVOUTP_HI                   Timer output inverter control.
                                       0 = output not inverted
                                       1 = output inverted
    16    TSTAT_HI                     Timer status bit (matches timer output,
                                       read-only)
                                       0 = output is currently low
                                       1 = output is currently high
    15:14 Reserved
    13:12 CAPTEVTMODE_LO               Capture event mode.
                                       0 = event occurs on rising edge of
                                           timer input
                                       1 = event occurs on falling edge of
                                           timer input
                                       2 = event occurs on both rising and
                                           falling edges
    11    CAPMODE_LO                   Capture mode enable. Only available
                                       in 32-bit unchained (unused).
    10    READRSTMODE_LO               See READRSTMODE_HI.
    9     TIEN_LO                      Timer input enable mode.
                                       0 = clock not gated by timer input
                                       1 = clock gated by timer input
    8     CLKSRC_LO                    Clock source.
                                       0 = internal clock
                                       1 = clock source is timer input signal
    7:6   ENAMODE_LO                   See ENAMODE_HI.
    5:4   PWID_LO                      See PWID_HI.
    3     CP_LO                        See CP_HI.
    2     INVINP_LO                    Timer input inverter control.
                                       0 = input not inverted
                                       1 = input inverted
    1     INVOUTP_LO                   See INVOUTP_HI.
    0     TSTAT_LO                     See TSTAT_HI.

    Initially we need to configure this such that the timers are disabled
    (ENAMODE = 0). As it happens, all other values in this register need to
    be 0 in our configuration, so just set the whole thing to 0.
    */
    timer[uart_idx]->TCR = 0;

    /*
    TGCR: Timer Global Control Register (SPRUGV5A section 3-1)

    Bit   Field          Value         Description
    31:16 Reserved
    15:12 TDDRHI                       Timer divide-down ratio. Prescale
                                       counter for dual 32-bit timers in
                                       unchained mode (unused).
    11:8  PSCHI                        Prescale period bits for TIMHI in
                                       dual 32-bit unchained mode.
    7:5   Reserved
    4     PLUSEN                       Enable Timer Plus features.
                                       0 = disabled
                                       1 = enabled
    3:2   TIMMODE                      Configure the timer mode.
                                       0 = 64-bit general-purpose
                                       1 = 2x 32-bit unchained timers
                                       2 = 64-bit watchdog timer
                                       3 = 2x 32-bit chained timers
    1     TIMHRS                       Timer reset for TIMHI.
                                       0 = TIMHI is in reset
                                       1 = TIMHI operating normally
    0     TIMLORS                      Timer reset for TIMLO.
                                       0 = TIMLO is in reset
                                       1 = TIMLO operating normally

    Initially we'll hold both of these in reset by setting the register to 0.
    */
    timer[uart_idx]->TGCR = 0;


    /* Now clear the count registers */
    timer[uart_idx]->CNTHI = 0;
    timer[uart_idx]->CNTLO = 0;

    /*
    Set the desired period. The timer peripheral runs at 1/6th CPU frequency,
    or 166.67MHz; the period is (input freq / baud rate) * 16
    */
    divisor = (166666666.67f / (float)uart_baud[uart_idx]) * 16.0;

    /* This more than covers the range from 2400-3Mbaud */
    assert(8000.0f <= divisor && divisor <= 2000000.0f);

    timer[uart_idx]->PRDHI = 0;
    timer[uart_idx]->PRDLO = (uint32_t)divisor;

    /*
    Take timer out of reset and set the mode enables (both the only bits we
    need to set in those registers).

    ENAMODE needs to be 2u (in bits 23:22 and 7:6, although 23:22 is actually
    ignored since we're running in 64-bit mode).

    TIMHRS and TIMLORS need to be 1 to take the timer out of reset.
    */
    timer[uart_idx]->TCR = 0x00800080u;
    timer[uart_idx]->TGCR = 0x2u;

    /* The timer should now be running, and triggering events */
}

void fcs_emif_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud) {
    assert(uart_idx == 0 || uart_idx == 1);
    assert(2400 <= uart_baud[uart_idx] && uart_baud[uart_idx] <= 3000000);

    uart_baud[uart_idx] = baud;
}

uint32_t fcs_emif_uart_check_error(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    volatile uint8_t *restrict uart_mem = uart_regs[uart_idx];
    uint8_t lsr = uart_mem[XR16M752_LSR];

    /*
    LSR: Line Status Register (table 8 of datasheet)

    Bit   Field          Value         Description
    31:8  Reserved
    7     RXFIFOE                      RX FIFO error
                                       0 = no errors
                                       1 = at least one parity error, framing
                                           error or break indicator in the
                                           FIFO
    6     TEMT                         THR & TSR empty
                                       0 = THR or TSR contains a character
                                       1 = Neither THR nor TSR contains a
                                           character
    5     THRE                         THR empty
                                       0 = THR contains a character
                                       1 = THR empty
    4     BI                           Break indicator. Set when the RX input
                                       is held low for more than a full word
                                       time.
                                       0 = RBR is not a break
                                       1 = RBR is a break
    3     FE                           Framing error in RBR.
                                       0 = RBR does not have a framing error
                                       1 = RBR has a framing error
    2     PE                           Parity error in RBR.
                                       0 = RBR does not have a parity error
                                       1 = RBR has a parity error
    1     OE                           Overrun error in RBR.
                                       0 = No overrun error
                                       1 = Overrun error
    0     DR                           Data ready in RX.
                                       0 = No byte in RBR
                                       1 = Byte in RBR
    */

    /* Error if BI, FE or OE was set */
    return lsr & 0x1Au;
}

void fcs_emif_uart_start_rx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Track this so we can return how many bytes have been written to the RX
    buffer by the EDMA engine
    */
    rx_last_buf_size[uart_idx] = buf_size;

    /*
    See implementation of fcs_int_uart_start_rx_edma (in int-uart.c) for a
    detailed description of the EDMA3 stuff.
    */
    edma3->TPCC_EECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);

    /*
    Configure the DMA queue number -- we use 1 to separate this out from the
    internal UART traffic on queue 0
    */
    uint8_t dma_register_idx = rx_edma_event[uart_idx] >> 3;
    uint8_t dma_register_bit = (rx_edma_event[uart_idx] -
                                (dma_register_idx << 3)) << 2;
    CSL_FINSR(edma3->TPCC_DMAQNUM[dma_register_idx], dma_register_bit + 2,
              dma_register_bit, 1);

    /* PaRAM set -> channel association */
    edma3->TPCC_DCHMAP[rx_edma_event[uart_idx]] =
        rx_edma_event[uart_idx] << 5u;

    #define primary (edma3->PARAMSET[rx_edma_event[uart_idx]])
    primary.OPT = 0;
    primary.SRC = (uint32_t)uart_regs[uart_idx];
    primary.A_B_CNT = (buf_size << 16) | 1u;
    /* FIXME: make sure buf is actually a L2 local address */
    primary.DST = GLOBAL_FROM_L2_ADDRESS(buf);
    primary.SRC_DST_BIDX = 1u << 16;
    primary.LINK_BCNTRLD = (100u + rx_edma_event[uart_idx]) << 5;
    primary.SRC_DST_CIDX = 0;
    primary.CCNT = 1u;
    #undef primary

    #define reload (edma3->PARAMSET[100u + rx_edma_event[uart_idx]])
    reload.OPT = 0;
    reload.SRC = (uint32_t)uart_regs[uart_idx];
    reload.A_B_CNT = (buf_size << 16) | 1u;
    /* FIXME: make sure buf is actually a L2 local address */
    reload.DST = GLOBAL_FROM_L2_ADDRESS(buf);
    reload.SRC_DST_BIDX = 1u << 16;
    reload.LINK_BCNTRLD = (100u + rx_edma_event[uart_idx]) << 5;
    reload.SRC_DST_CIDX = 0;
    reload.CCNT = 1u;
    #undef reload

    /* Reset DMA and enable the transfer on the next interrupt */
    edma3->TPCC_SECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(rx_edma_event[uart_idx],
                               rx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;
    edma3->TPCC_EESR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
}

void fcs_emif_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Track buffer size so we can return number of bytes read from TX buffer
    based on the current BCNT value
    */
    tx_last_buf_size[uart_idx] = buf_size;

    /* Disable DMA events for this channel */
    edma3->TPCC_EECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);


    /* Set up DMA channel -> queue mapping: Q1 like above */
    uint8_t dma_register_idx = tx_edma_event[uart_idx] >> 3;
    uint8_t dma_register_bit = (tx_edma_event[uart_idx] -
                                (dma_register_idx << 3)) << 2;
    CSL_FINSR(edma3->TPCC_DMAQNUM[dma_register_idx], dma_register_bit + 2,
              dma_register_bit, 1);

    /* Map PaRAM set to channel */
    edma3->TPCC_DCHMAP[tx_edma_event[uart_idx]] =
      tx_edma_event[uart_idx] << 5u;

    #define primary (edma3->PARAMSET[tx_edma_event[uart_idx]])
    primary.OPT = 0;
    /* FIXME: make sure buf is actually a L2 local address */
    primary.SRC = GLOBAL_FROM_L2_ADDRESS(buf); /* Read from buf */
    primary.A_B_CNT = (buf_size << 16) | 1u;
    primary.DST = (uint32_t)uart_regs[uart_idx];
    primary.SRC_DST_BIDX = 1u; /* Increment src address, not dest */
    primary.LINK_BCNTRLD = 0xFFFFu; /* NULL PaRAM set for link */
    primary.SRC_DST_CIDX = 0;
    primary.CCNT = 1u;
    #undef primary

    /* Reset DMA and enable the transfer */
    edma3->TPCC_SECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(tx_edma_event[uart_idx],
                               tx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;
    edma3->TPCC_EESR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
}

uint16_t fcs_emif_uart_get_rx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    if (rx_last_buf_size[uart_idx] == 0) {
        return 0;
    } else {
        /*
        Subtract BCNT from last buffer size to get the number of bytes written
        to RX buffer
        */
        return rx_last_buf_size[uart_idx] -
               (edma3->PARAMSET[rx_edma_event[uart_idx]].A_B_CNT >> 16);
    }
}

uint16_t fcs_emif_uart_get_tx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    if (tx_last_buf_size[uart_idx] == 0) {
        return 0;
    } else {
        /*
        Subtract BCNT from last buffer size to get the number of bytes read
        from TX buffer
        */
        return tx_last_buf_size[uart_idx] -
               (edma3->PARAMSET[tx_edma_event[uart_idx]].A_B_CNT >> 16);
    }
}
