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
SETUP               tAS                N+1 cycles           5ns
STROBE              tCS/tRD            N+1 cycles           60ns
HOLD                tDY                N+1 cycles           60ns
TURNAROUND          -                  -                    -

Since each cycle is 6ns, EMIF SETUP is set to 0 (+1 internally), EMIF STROBE
is set to 9 (+ 1 internally) and EMIF HOLD is set to 9 (+1 internally).

Write timings are the same.
*/

/*
See int-uart.c for a detailed description of the EDMA3 configuration. This
file only includes descriptions of the differences in configuration between
the internal and external UARTs.
*/
static volatile CSL_TpccRegs* edma3 = (CSL_TpccRegs*)CSL_EDMA2CC_REGS;

/*
GPINT0 and GPINT1 are used as RX events -- we configure the external UART to
send an interrupt for each byte received.

For TX events, we set a timer that transfers each byte at an appropriate rate
(depending on the configured baud rate of the UART). The HIGH signals of
TIMER4 and TIMER5 are used.
*/
static uint16_t rx_edma_event[2] = { 6u, 7u },
                tx_edma_event[2] = { 23u, 25u };

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
    */
    uint8_t LCR;

    /*
    MCR: Modem Control Register

    Bit   Field          Value         Description
    */
    uint8_t MCR;

    /* There are a bunch of other registers but we don't use them */
};

static struct emif16_xr16m752_uart_t uart[2];

void fcs_emif_uart_reset(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    /* TODO: reset external UART */

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

    In this application we need fairly low rates of 57600 for the Piksi and
    230400 for the CPU, so we'll use prescaler = 4 and oversampling_rate = 16.
    */
    assert(2400 <= uart_baud[uart_idx] && uart_baud[uart_idx] <= 3000000);

    float divisor = (14745600.0f / 4.0f) / (float)(uart_baud[uart_idx] * 16);
    uint16_t divisor_floor = (uint16_t)divisor, dld;
    dld = (uint16_t)((divisor - (float)divisor_floor) * 16.0 + 0.5);
    assert(dld < 0x10u);

    /*
    Set the divisor latch values, as well as bit 7 of LCR to enable them.
    */
    uart[uart_idx].LCR != 0x80u;
    uart[uart_idx].DLM_IER = (divisor_floor >> 8) & 0xFFu;
    uart[uart_idx].DLL = divisor_floor & 0xFFu;
    uart[uart_idx].DLD_FCR = dld & 0x0Fu;

    /* TODO: trigger a DMA of these three bytes to c
}

void fcs_emif_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud) {
    assert(uart_idx == 0 || uart_idx == 1);
    assert(2400 <= uart_baud[uart_idx] && uart_baud[uart_idx] <= 3000000);

    uart_baud[uart_idx] = baud;
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
    TODO:
    - disable external UART RX (if possible)
    - update mapping for GPINT0/GPINT1 so we don't get UART RX interrupts
    - configure external UART RX with appropriate settings for its interrupt
      pin
    */

    /*
    See implementation of fcs_int_uart_start_rx_edma (in int-uart.c) for a
    detailed description of the EDMA3 stuff.
    */
    edma3->TPCC_EECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_SECR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(rx_edma_event[uart_idx],
                               rx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;

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
    CSL_FINS(edma3->TPCC_DCHMAP[rx_edma_event[uart_idx]],
             TPCC_TPCC_DCHMAP0_PAENTRY, rx_edma_event[uart_idx]);

    #define primary (edma3->PARAMSET[rx_edma_event[uart_idx]])
    primary.OPT = 0;
    primary.SRC = (uint32_t)&(uart[uart_idx]->RBR); /* FIXME: use EMIF16 address */
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
    reload.SRC = (uint32_t)&(uart[uart_idx]->RBR); /* FIXME: use EMIF16 address */
    reload.A_B_CNT = (buf_size << 16) | 1u;
    /* FIXME: make sure buf is actually a L2 local address */
    reload.DST = GLOBAL_FROM_L2_ADDRESS(buf);
    reload.SRC_DST_BIDX = 1u << 16;
    reload.LINK_BCNTRLD = (100u + rx_edma_event[uart_idx]) << 5;
    reload.SRC_DST_CIDX = 0;
    reload.CCNT = 1u;
    #undef reload

    /* Enable DMA */
    edma3->TPCC_EESR = CSL_FMKR(rx_edma_event[uart_idx],
                                rx_edma_event[uart_idx], 1u);

    /*
    TODO:
    - enable external UART RX (if possible)
    - update mapping for GPINT0/GPINT1 so we get UART RX interrupts
    */
}

void fcs_emif_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Track buffer size so we can return number of bytes read from TX buffer
    based on the current BCNT value
    */
    tx_last_buf_size[uart_idx] = buf_size;

    /*
    TODO:
    - disable TIMER4/TIMER5
    - reset external UART TX buffers
    */

    edma3->TPCC_EECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_SECR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);
    edma3->TPCC_ECR = CSL_FMKR(tx_edma_event[uart_idx],
                               tx_edma_event[uart_idx], 1u);
    edma3->TPCC_EMCR = 0xFFFFFFFFu;
    edma3->TPCC_EMCRH = 0xFFFFFFFFu;

    /* Set up DMA channel -> queue mapping: Q1 like above */
    uint8_t dma_register_idx = tx_edma_event[uart_idx] >> 3;
    uint8_t dma_register_bit = (tx_edma_event[uart_idx] -
                                (dma_register_idx << 3)) << 2;
    CSL_FINSR(edma3->TPCC_DMAQNUM[dma_register_idx], dma_register_bit + 2,
              dma_register_bit, 1);

    /* Map PaRAM set to channel */
    CSL_FINS(edma3->TPCC_DCHMAP[tx_edma_event[uart_idx]],
             TPCC_TPCC_DCHMAP0_PAENTRY, tx_edma_event[uart_idx]);

    #define primary (edma3->PARAMSET[tx_edma_event[uart_idx]])
    primary.OPT = 0;
    /* FIXME: make sure buf is actually a L2 local address */
    primary.SRC = GLOBAL_FROM_L2_ADDRESS(buf); /* Read from buf */
    primary.A_B_CNT = (buf_size << 16) | 1u;
    primary.DST = (uint32_t)&(uart[uart_idx]->THR); /* FIXME: external UART addr */
    primary.SRC_DST_BIDX = 1u; /* Increment src address, not dest */
    primary.LINK_BCNTRLD = 0xFFFFu; /* NULL PaRAM set for link */
    primary.SRC_DST_CIDX = 0;
    primary.CCNT = 1u;
    #undef primary

    /* Enable the transfer */
    edma3->TPCC_EESR = CSL_FMKR(tx_edma_event[uart_idx],
                                tx_edma_event[uart_idx], 1u);

    /*
    TODO:
    - configure TIMER4/TIMER5 based on current baud rate, to trigger TX events
    */
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
