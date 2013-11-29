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

#include "../c66x-csl/ti/csl/csl.h"
#include "../c66x-csl/ti/csl/cslr.h"
#include "../c66x-csl/ti/csl/cslr_uart.h"
#include "../c66x-csl/ti/csl/csl_edma3.h"
#include "../c66x-csl/ti/csl/csl_edma3Aux.h"

#include "int-uart.h"

/*
EDMA3 configuration

RX buffer transfers are A-synchronised, i.e. one transfer request [TR] per
event, which is triggered on every byte received. Two PaRAM entries are used
per UART: a primary entry and a reload entry. The primary entry is linked to
the reload entry, such that on each completion of the primary entry the reload
entry's destination address and transfer count is copied across. The reload
entry is linked-to-self to avoid the need for further updates.

For the RX PaRAM entries, the base destination address is the address of byte
0 in the RX buffer, and the transfer byte count (BCNT) is the size of the
buffer.

TX buffer transfers are also A-synchronised, but because the number of bytes
to be transferred varies, and only one write request at a time is permitted,
it's somewhat simpler -- just set up a single PaRAM entry per write and don't
accept further writes until it's complete.
*/

CSL_Edma3ParamSetup *rx_edma_primary_param[2] = {},
                    *rx_edma_reload_param[2] = {},
                    *tx_edma_primary_param[2] = {};

CSL_Edma3ChannelObj rx_edma_channel[2],
                    tx_edma_channel[2];

/*
UART registers (table 3-1 in SPRUGP1):

RBR: Receive Buffer Register (read-only)
THR: Transmitter Holding Register (write-only)
IER: Interrupt Enable Register
IIR: Interrupt Identification Register
FCR: FIFO Control Register (write-only)
LCR: Line Control Register
MCR: Modem Control Register
LSR: Line Status Register
MSR: Modem Status Register
SCR: Scratch Pad Register
DLL: Divisor LSB Latch
DLH: Divisor MSB Latch
REVID1: Revision Identification Register 1
REVID2: Revision Identification Register 2
PWREMU_MGMT: Power and Emulation Management Register
MDR: Mode Definition Register

If using the shared addresses, RBR and THR can only be accessed when the DLAB
bit in LCR is low; DLL can only be accessed when the DLAB bit in LCR is high.

IER can only be accessed when DLAB is low. DLH can only be accessed when DLAB
is high.

HOWEVER, the C66 CSL uses *dedicated* addresses for DLH and DLL, so we don't
need to mess around with that -- they can be written at any time.
*/

static CSL_UartRegs *uart[2] = {
    (CSL_UartRegs*)CSL_UART_REGS,
    (CSL_UartRegs*)CSL_UART_B_REGS
};

static uint32_t uart_baud[2] = { 115200u, 115200u };

void fcs_int_uart_reset(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    /* Initialization process as described in part 2.7 of SPRUGP1 */

    /*
    PWREMU_MGMT: Power and Emulation Management Register (section 3.13 in
    SPRUGP1)

    Bit   Field          Value         Description
    14    UTRST                        UART transmitter reset.
                                       0 = transmitter in reset state
                                       1 = transmitter enabled
    13    URRST                        UART receiver reset.
                                       0 = receiver in reset state
                                       1 = receiver enabled
    0     FREE                         Free running enable for emulation.
                                       0 = halt after current transmission if
                                           emulation even received
                                       1 = keep running regardless

    Reset the UART RX/TX, and halt in emulation -- write 0, then 0x6000u after
    the setup process is complete.
    */
    uart[uart_idx]->PWREMU_MGMT = 0;

    /*
    The UART baud rate generator is derived from SYSCLK7 via PLLOUT->PLLDIV7.
    SYSCLK7 is always 1/6th the rate of SYSCLK1.

    Set the baud rate divisor:
        DLH:DLL = SYSCLK7_FREQ_HZ / (UART_BAUD * 13)

    so with a 100MHz clock and e.g. a desired baud rate of 921600 we'd set
        DLH:DLL = 166666667 / (921600 * 13) = 11

    which would result in an actual baud rate of
        ACTUAL_UART_BAUD = SYSCLK7_FREQ_HZ / (DLH:DLL * 13)
        915750 = 166666667 / (11 * 13)

    for an error of 0.63%.
    */
    assert(2400 <= uart_baud[baud_idx] && uart_baud[baud_idx] <= 3000000);

    float divisor = 166666666.67f / (float)(uart_baud[baud_idx] * 13);
    uint16_t divisor_int = (uint16_t)(divisor + 0.5f);
    uart[uart_idx]->DLL = divisor_int & 0xFFu;
    uart[uart_idx]->DLH = (divisor_int >> 8) & 0xFFu;

    /*
    MDR: Mode Definition Register (section 3.14 in SPRUGP1)

    Bit   Field          Value         Description
    0     OSM_SEL                      0 = 16x oversampling
                                       1 = 13x oversampling

    Set to 0 for 13x oversampling (better for 230400, 921600 etc).
    */
    uart[uart_idx]->MDR = 0x01u;

    /*
    FCR: FIFO Control Register (section 3.5 in SPRUGP1)

    Bit   Field          Value         Description
    7:6   RXFIFTL        0-3           Receiver FIFO interrupt trigger level.
                                       0 = 1 byte
                                       1 = 4 bytes
                                       2 = 8 bytes
                                       3 = 14 bytes
    3     DMAMODE1                     Must be 1 for EDMA to work.
                                       0 = DMA mode disabled
                                       1 = DMA mode enabled
    2     TXCLR                        Clears transmitter FIFO.
                                       0 = no change
                                       1 = clear transmitter FIFO
    1     RXCLR                        Clears receiver FIFO.
                                       0 = no change
                                       1 = clear receiver FIFO
    0     FIFOEN                       Enables the RX/TX FIFOs. Must be set
                                       *before* other bits are written, or
                                       they will be ignored.
                                       0 = FIFO disabled
                                       1 = FIFO enabled

    First, set FCR to 0x01 to enable.
    Then, set 0x0F for 1-byte FIFO trigger level, RX/TX FIFO clear, and FIFO
    enable.
    */
    uart[uart_idx]->FCR = 0x01u;
    uart[uart_idx]->FCR = 0x0Fu;

    /*
    LCR: Line Control Register (section 3.6 in SPRUGP1)

    Bit   Field          Value         Description
    7     DLAB                         Divisor latch access bit.
                                       0 = acccess to THR/RBR and IER,
                                           *no* access to DLL and DLH on
                                           shared address
                                       1 = access to DLL and DLH, *no* access
                                           to THR/RBR and IER on shared
                                           address
    6     BC                           Break control.
                                       0 = break condition disabled
                                       1 = break condition transmitted to
                                           receiving UART
    5     SP                           Stick parity. If PEN=1, EPS=0 and SP=1,
                                       the parity bit is active high. If
                                       PEN=1, EPS=1 and SP=1, the parity bit
                                       is active low. No effect if PEN=0.
                                       0 = stick parity disabled
                                       1 = stick parity enabled
    4     EPS                          Even parity select. Ignored if PEN=0.
                                       0 = odd parity (odd number of 1s)
                                       1 = even parity (even number of 1s)
    3     PEN                          Parity enable.
                                       0 = no parity transmitted or checked
                                       1 = parity generated and checked
    2     STB                          Number of stop bit generated.
                                       0 = one stop bit generated
                                       1 = 1.5 stop bits generated when WLS=0,
                                           2 stop bits generated when WLS>0
    1:0   WLS            0-3           Word length select.
                                       0 = 5 bits
                                       1 = 6 bits
                                       2 = 7 bits
                                       3 = 8 bits

    Since we're using dedicated addresses for DLL/DLH, we don't need to set
    DLAB.

    Set LCR to 0x03 to run with no parity, one stop bit, and 8 data bits.
    */
    uart[uart_idx]->LCR = 0x03u;

    /*
    MCR: Modem Control Register (section 3.7 in SPRUGP1)

    Not used -- set to 0.
    */
    uart[uart_idx]->MCR = 0;

    /*
    IER: Interrupt Enable Register (section 3.3 in SPRUGP1)

    Bit   Field          Value         Description
    3     EDSSI          0             Enable modem status interrupt.
    2     ELSI                         Reeiver line status interrupt enable.
                                       0 = disabled
                                       1 = enabled
    1     ETBEI                        THR empty interrupt enable.
                                       0 = disabled
                                       1 = enabled
    0     ERBI                         Receiver data available/character
                                       timeout interrupt enable.
                                       0 = disabled
                                       1 = enabled

    We don't want any interrupts, so set to 0.
    */
    uart[uart_idx]->IER = 0;

    /* Enable the UART (see first step above) */
    uart[uart_idx]->PWREMU_MGMT = 0x6000u;
}

void fcs_int_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud) {
    assert(uart_idx == 0 || uart_idx == 1);
    assert(2400 <= uart_baud[baud_idx] && uart_baud[baud_idx] <= 3000000);

    uart_baud[uart_idx] = baud;
}

void fcs_int_uart_start_rx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);
    /*
    For an internal UART, we configure it in the mode required for DMA,
    set up a single DMA transfer per buffer with ACNT=1, BCNT=256, CCNT=1.

    A linked PaRAM set is configured to reload the transfer with the base
    buffer index and BCNT=256.

    The next destination byte index can be read directly to get the count of
    bytes transferred; if it's lower than the read pointer, add 256. (And
    subtract 256 once the read pointer reaches 256.)
    */
}

void fcs_int_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);
    /*
    For TX in internal UARTs, we QDMA-trigger an A-synchronised transfer with
    ACNT = 1, BCNT = number of bytes to send, and CCNT = 1. If there's a
    transfer currently in progress, we fail.

    The next source byte index can be read directly to get the count of bytes
    transferred.
    */
}

uint16_t fcs_int_uart_get_rx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);
}

uint16_t fcs_int_uart_get_tx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);
}
