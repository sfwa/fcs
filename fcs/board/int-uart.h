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

#ifndef _FCS_INT_UART_H
#define _FCS_INT_UART_H

void fcs_int_uart_reset(uint8_t uart_idx);
void fcs_int_uart_set_baud_rate(uint8_t uart_idx, uint32_t baud);

void fcs_int_uart_start_rx_dma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t buf_size);
void fcs_int_uart_start_tx_dma(uint8_t uart_idx, uint8_t *restrict buf,
uint16_t transfer_bytes);

uint16_t fcs_int_uart_get_rx_dma_count(uint8_t uart_idx);
uint16_t fcs_int_uart_get_tx_dma_count(uint8_t uart_idx);

#endif
