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
#ifndef _FCS_EMIF_UART_H
#define _FCS_EMIF_UART_H

/*
Reset the external UART identified by `uart_idx`, along with any associated
DMA channels and in-flight transfers.
*/
void fcs_emif_uart_reset(uint8_t uart_idx);

/*
Set the baud rate of the external UART identified by `uart_idx` to `baud`.
The external UART must be reset for the new configuration to take effect.
*/
void fcs_emif_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud);

/*
Start an ongoing DMA transfer from the UART `uart_idx` to the RX buffer `buf`,
with `buf_size` bytes being written to the buffer before the write pointer
wraps around to buf[0].

The transfer re-triggers each time it completes, so `fcs_emif_uart_reset` must
be called to stop it.
*/
void fcs_emif_uart_start_rx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size);

/*
Start a one-shot DMA transfer from the TX buffer `buf` to the UART `uart_idx`.
`buf_size` bytes will be transferred.

The transfer must complete before `fcs_emif_uart_start_tx_edma` is called
again (use `fcs_emif_uart_get_tx_edma_count` to check).
*/
void fcs_emif_uart_start_tx_edma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size);

/*
Get the count of bytes written to the RX EDMA buffer. If the write pointer has
wrapped, this count will too. Be sure to check the count frequently enough to
detect that condition.
*/
uint16_t fcs_emif_uart_get_rx_edma_count(uint8_t uart_idx);

/*
Get the count of bytes read from the TX EDMA buffer. Once this count reaches
the number of bytes passed to the previous call to
`fcs_emif_uart_start_tx_edma`, it's safe to start another write.
*/
uint16_t fcs_emif_uart_get_tx_edma_count(uint8_t uart_idx);

#endif
