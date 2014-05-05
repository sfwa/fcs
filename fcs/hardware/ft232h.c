/*
Copyright (C) 2014 Ben Dyer

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
#include <stddef.h>

#include <c6x.h>

#include "../c66x-csl/ti/csl/cslr.h"
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_emif16.h"
#include "../c66x-csl/ti/csl/cslr_tpcc.h"
#include "../c66x-csl/ti/csl/cslr_tmr.h"
#include "../c66x-csl/ti/csl/cslr_gpio.h"
#include "../c66x-csl/ti/csl/cslr_cgem.h"

#include "board.h"
#include "ft232h.h"

/*
Use EDMA for FT232H connected via EMIF in CPU FIFO mode. Timers are used for
writes (1 byte per 2us) since interrupts are not available in this mode.

For TX we set up a timer firing at 500kHz, which triggers an AB-synchronised
transfer with ACNT = 1, BCNT = 1 and CCNT = number of bytes to send.

The EMIF is clocked at CPU_FREQ/6 (166.7MHz).

Connections are:

EMIF signal         FT232H signal      Description
D[7:0]              DBUS[7:0]          8-bit parallel data bus
/OE                 RD#
/WE                 WR#
/CS3                CS#                Chip select (active low)
GPIOn               RESET              FT232H reset (active high)

Interface implementation follows the example for 8-bit ASRAM/NOR interface
in section 2.5 of SPRUGZ3A.

Timing is as follows:

EMIF parameter      FT232H parameter   EMIF setting         FT232H spec value
SETUP               t1/t3              0+1 cycles           5ns
STROBE              t8/t9              5+1 cycles          30ns
HOLD                t2/t4              0+1 cycles           5ns
TURNAROUND          -                  -                    -

Since each cycle is 6ns, EMIF SETUP is set to 0 (+1 internally), EMIF STROBE
is set to 5 (+ 1 internally) and EMIF HOLD is set to 0 (+1 internally).

Write timings are the same.

See int-uart.c for a detailed description of the EDMA3 configuration. This
file only includes descriptions of the differences in configuration between
the internal and external UARTs.
*/

/*
For TX events, we set a timer that transfers each byte at an appropriate rate.
The LO signal of TIMER6 is used.
*/
static const uint16_t tx_edma_event = 26u;
static uint16_t tx_last_buf_size;

/*
Logical addresses for EMIF16 CE0 data space.
*/
#define EMIF16_FT232H_BASE_ADDR 0x70000000

void fcs_ft232h_reset(void) {
    /*
    Prior to accessing the EMIF16 we need to configure its cacheability in the
    appropriate MAR (see SPRUGW0C section 4.4.4).

    The MAR for the FT232H is 124 (for locations 7C00 0000 - 7CFF FFFF).

    MARn: Memory Attribute Register (SPRUGW0C section 4.4.5)

    Bit   Field          Value         Description
    31:4  Reserved
    3     PFX                          Enable/disable prefetchability of the
                                       address range.
                                       0 = not prefetchable
                                       1 = prefetchable
    2     Reserved
    1     Reserved
    0     PC                           Enable/disable cacheability of the
                                       address range.
                                       0 = not cacheable
                                       1 = cacheable

    For EMIF16 we want to configure the MARs as non-cacheable,
    non-prefetchable -- so set them to 0.
    */
    volatile CSL_CgemRegs *const cgem =
        (CSL_CgemRegs*)CSL_CGEM0_5_LOCAL_L2_SRAM_REGS;
    cgem->MAR[124] = 0;

    /*
    Configure the EMIF CE0 via A0CR.

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

    We're using ASIZE=0 (8-bit), TA=0, R_HOLD=0, R_STROBE=5, R_SETUP=0,
    write setup/strobe/hold the same.
    */
    volatile CSL_Emif16Regs *const emif16 = (CSL_Emif16Regs*)CSL_EMIF16_REGS;
    emif16->A0CR = (0x5u << 7u) + (0x5u << 20u);

    /*
    PMCR is the Page Mode Control Register. We're not using NOR flash, so set
    it all to 0
    */
    emif16->PMCR = 0;

    /* EDMA3 reset */
    volatile CSL_TpccRegs *const edma3 = (CSL_TpccRegs*)CSL_EDMA2CC_REGS;
    edma3->TPCC_EECR = 1u << tx_edma_event;
    edma3->TPCC_SECR = 1u << tx_edma_event;
    edma3->TPCC_ECR = 1u << tx_edma_event;
    edma3->TPCC_EMCR = 1u << tx_edma_event;

    /*
    Pointer to TIMER6 peripheral. This is configured in 64-bit mode (the
    default), with continuous mode enabled. The configured period is 2 times
    the maximum transfer rate (1M/s).

    This means that the maximum TX throughput is 4MB/s.
    */
    volatile CSL_TmrRegs *const timer = (CSL_TmrRegs*)CSL_TIMER_6_REGS;

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
    timer->TCR = 0;

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
    timer->TGCR = 0;


    /* Now clear the count registers */
    timer->CNTHI = 0;
    timer->CNTLO = 0;

    /*
    Set the desired period. The timer peripheral runs at 1/6th CPU frequency,
    or 166.67MHz; the period is (input freq / max transfer rate) * 2
    */
    float divisor;
    divisor = (166666666.67f / 1000000.0f) * 2.0;

    timer->PRDHI = 0;
    timer->PRDLO = (uint32_t)divisor;

    /*
    Take timer out of reset and set the mode enables (both the only bits we
    need to set in those registers).

    ENAMODE needs to be 2u (in bits 23:22 and 7:6, although 23:22 is actually
    ignored since we're running in 64-bit mode).

    TIMHRS and TIMLORS need to be 1 to take the timer out of reset.
    */
    timer->TCR = 0x00800080u;
    timer->TGCR = 0x3u;

    /* The timer should now be running, and triggering events */
}

void fcs_ft232h_start_tx_edma(uint8_t *restrict buf, uint16_t buf_size) {
    /*
    Track buffer size so we can return number of bytes read from TX buffer
    based on the current BCNT value
    */
    tx_last_buf_size = buf_size;

    /* Disable DMA events for this channel */
    volatile CSL_TpccRegs *const edma3 = (CSL_TpccRegs*)CSL_EDMA2CC_REGS;
    edma3->TPCC_EECR = CSL_FMKR(tx_edma_event, tx_edma_event, 1u);

    /* Set up DMA channel -> queue mapping: Q1 like above */
    uint8_t dma_register_idx = tx_edma_event >> 3;
    uint8_t dma_register_bit = (tx_edma_event - (dma_register_idx << 3)) << 2;
    CSL_FINSR(edma3->TPCC_DMAQNUM[dma_register_idx], dma_register_bit + 2,
              dma_register_bit, 1);

    /* Map PaRAM set to channel */
    edma3->TPCC_DCHMAP[tx_edma_event] = tx_edma_event << 5u;

    #define primary (edma3->PARAMSET[tx_edma_event])
    primary.OPT = 0;
    /* FIXME: make sure buf is actually a L2 local address */
    primary.SRC = GLOBAL_FROM_L2_ADDRESS(buf); /* Read from buf */
    primary.A_B_CNT = (buf_size << 16) | 1u;
    primary.DST = (uint32_t)EMIF16_FT232H_BASE_ADDR;
    primary.SRC_DST_BIDX = 1u; /* Increment src address, not dest */
    primary.LINK_BCNTRLD = 0xFFFFu; /* NULL PaRAM set for link */
    primary.SRC_DST_CIDX = 0;
    primary.CCNT = 1u;
    #undef primary

    /* Reset DMA and enable the transfer */
    edma3->TPCC_SECR = CSL_FMKR(tx_edma_event, tx_edma_event, 1u);
    edma3->TPCC_ECR = CSL_FMKR(tx_edma_event, tx_edma_event, 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;
    edma3->TPCC_EESR = CSL_FMKR(tx_edma_event, tx_edma_event, 1u);
}

uint16_t fcs_ft232h_get_tx_edma_count(void) {
    volatile CSL_TpccRegs *const edma3 = (CSL_TpccRegs*)CSL_EDMA2CC_REGS;

    if (tx_last_buf_size == 0) {
        return 0;
    } else {
        /*
        Subtract BCNT from last buffer size to get the number of bytes read
        from TX buffer
        */
        return tx_last_buf_size -
               (edma3->PARAMSET[tx_edma_event].A_B_CNT >> 16);
    }
}
