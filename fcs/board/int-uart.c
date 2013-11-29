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

Some code is copyright (c) 2011-2012 Texas Instruments Incorporated -
http://www.ti.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the
  distribution.

  Neither the name of Texas Instruments Incorporated nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
For an internal UART, we configure it in the mode required for DMA,
set up a single DMA transfer per buffer with ACNT = 1, BCNT = 1, CCNT > 2,
and poll periodically to see if the buffer is full based on changes in
CCNT and/or the destination index. The transfer mode is set to
AB-synchronised, with no interrupts; when CCNT gets too low we increase it.

For TX in internal UARTs, we QDMA-trigger an A-synchronised transfer with
ACNT = 1, BCNT = number of bytes to send, and CCNT = 1.
*/

static CSL_UartRegs *uart[2] = {
    (CSL_UartRegs*)CSL_UART_REGS,
    (CSL_UartRegs*)CSL_UART_B_REGS
};

void fcs_int_uart_reset(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);

    /*
    Allows access to the divisor latches of the baud generator during a
    read or write operation (DLL and DLH)
    */
    CSL_FINS(uart[uart_idx]->LCR, UART_LCR_DLAB, CSL_UART_LCR_DLAB_ENABLE);

    /* Break condition is disabled. */
    CSL_FINS(uart[uart_idx]->LCR, UART_LCR_BC, CSL_UART_LCR_BC_DISABLE);

    /* Stick parity is disabled. */
    CSL_FINS(uart[uart_idx]->LCR, UART_LCR_SP, CSL_UART_LCR_SP_DISABLE);

    /* Odd parity is selected */
    CSL_FINS(uart[uart_idx]->LCR, UART_LCR_EPS, CSL_UART_LCR_EPS_ODD);

    /* No PARITY bit is transmitted or checked */
    CSL_FINS(uart[uart_idx]->LCR, UART_LCR_PEN, CSL_UART_LCR_PEN_DISABLE);

    /* Set the baudrate, for accessing LCR[7] should be enable */
    uart[uart_idx]->DLL = DLL_VAL;
    uart[uart_idx]->DLH = DLM_VAL;

    /*
    Allows access to the receiver buffer register (RBR), the transmitter
    holding register (THR), and the interrupt enable register (IER) selected.
    */
    CSL_FINS(uart[uart_idx]->LCR, UART_LCR_DLAB, CSL_UART_LCR_DLAB_DISABLE);

    /*
    Disable THR, RHR, Receiver line status interrupts
    */
    CSL_FINS(uart[uart_idx]->IER, UART_IER_ERBI,  CSL_UART_IER_ERBI_DISABLE);
    CSL_FINS(uart[uart_idx]->IER, UART_IER_ETBEI, CSL_UART_IER_ETBEI_DISABLE);
    CSL_FINS(uart[uart_idx]->IER, UART_IER_ELSI,  CSL_UART_IER_ELSI_DISABLE);
    CSL_FINS(uart[uart_idx]->IER, UART_IER_EDSSI, CSL_UART_IER_EDSSI_DISABLE);

    /*
    If autoflow control is desired,
    write appropriate values to the modem
    control register (MCR). Note that all UARTs
    do not support autoflow control, see
    the device-specific data manual for supported features.

    MCR
    ====================================================
    Bit  Field   Value   Description
    5    AFE     0       Autoflow control is disabled
    4    LOOP    0       Loop back mode is disabled.
    1    RTS     0       RTS control (UARTn_RTS is disabled,
                         UARTn_CTS is only enabled.)
    =====================================================
    */

    uart[uart_idx]->MCR = 0;

    /*
    Choose the desired response to
    emulation suspend events by configuring
    the FREE bit and enable the UART by setting
    the UTRST and URRST bits in the power and
    emulation management register (PWREMU_MGMT).


    PWREMU_MGMT
    =================================================
    Bit  Field   Value   Description
    14   UTRST   1       Transmitter is enabled
    13   URRST   1       Receiver is enabled
    0    FREE    1       Free-running mode is enabled
    ===================================================
    */
    uart[uart_idx]->PWREMU_MGMT = 0x6001;

    /* Cleanup previous data (rx trigger is also set to 0)*/
    /* Set FCR = 0x07; */
    CSL_FINS(uart[uart_idx]->FCR, UART_FCR_FIFOEN,
             CSL_UART_FCR_FIFOEN_ENABLE);
    CSL_FINS(uart[uart_idx]->FCR, UART_FCR_TXCLR, CSL_UART_FCR_TXCLR_CLR);
    CSL_FINS(uart[uart_idx]->FCR, UART_FCR_RXCLR, CSL_UART_FCR_RXCLR_CLR);
    CSL_FINS(uart[uart_idx]->FCR, UART_FCR_DMAMODE1,
             CSL_UART_FCR_DMAMODE1_DISABLE);
    CSL_FINS(uart[uart_idx]->FCR, UART_FCR_RXFIFTL,
             CSL_UART_FCR_RXFIFTL_CHAR1);
}

void fcs_int_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud) {
    assert(uart_idx == 0 || uart_idx == 1);

    uint8_t dll, dlh;

    /* TODO: work out baud rate properly, based on PLL/clock speed */
    dll = (uint8_t)(baud & 0xFFu);
    dlh = (uint8_t)((baud >> 8) & 0xFFu);

    /* Set the baud rate divisor -- LCR[7] should be enabled for access */
    uart[uart_idx]->LCR = 0x80;
    uart[uart_idx]->DLL = dll;
    uart[uart_idx]->DLH = dlh;
    uart[uart_idx]->LCR = 0x03;
}

void fcs_int_uart_start_rx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);
}

void fcs_int_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {
    assert(uart_idx == 0 || uart_idx == 1);
}

uint16_t fcs_int_uart_get_rx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);
}

uint16_t fcs_int_uart_get_tx_edma_count(uint8_t uart_idx) {
    assert(uart_idx == 0 || uart_idx == 1);
}
