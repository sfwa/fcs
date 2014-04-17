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
#ifndef _FCS_FT232H_H
#define _FCS_FT232H_H

/* Reset the FT232H and clear any internal status. */
void fcs_ft232h_reset(void);

/*
Start a one-shot DMA transfer from the TX buffer `buf` to the FT232H.
`buf_size` bytes will be transferred.

The transfer must complete before `fcs_ft232h_start_tx_edma` is called
again (use `fcs_ft232h_get_tx_edma_count` to check).
*/
void fcs_ft232h_start_tx_edma(uint8_t *restrict buf, uint16_t buf_size);

/*
Get the count of bytes read from the TX EDMA buffer. Once this count reaches
the number of bytes passed to the previous call to `fcs_ft232h_start_tx_edma`,
it's safe to start another write.
*/
uint16_t fcs_ft232h_get_tx_edma_count(void);

#endif
