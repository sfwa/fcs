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
*/

void fcs_emif_uart_reset(uint8_t uart_idx) {

}

void fcs_emif_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud) {

}

void fcs_emif_uart_start_rx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {

}

void fcs_emif_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size) {

}

uint16_t fcs_emif_uart_get_rx_edma_count(uint8_t uart_idx) {
	return 0;
}

uint16_t fcs_emif_uart_get_tx_edma_count(uint8_t uart_idx) {
	return 0;
}
