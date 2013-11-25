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
#include <stdbool.h>
#include <assert.h>

#include "stream.h"

static uint8_t rx_buffers[FCS_STREAM_NUM_DEVICES][FCS_STREAM_BUFFER_SIZE];
static uint8_t tx_buffers[FCS_STREAM_NUM_DEVICES][FCS_STREAM_BUFFER_SIZE];

enum fcs_stream_result_t fcs_stream_set_rate(enum fcs_stream_device_t dev,
uint32_t baud) {
    if (dev != FCS_STREAM_UART_INT0 && dev != FCS_STREAM_UART_INT1 &&
            dev != FCS_STREAM_UART_EXT0 && dev != FCS_STREAM_UART_EXT1) {
        return FCS_STREAM_ERROR;
    } else if (baud < 57600 || baud > 3000000) {
        return FCS_STREAM_ERROR;
    }

    if (dev == FCS_STREAM_UART_INT0 || dev == FCS_STREAM_UART_INT1) {
        /* TODO: configure internal UART baud rate */
    } else if (dev == FCS_STREAM_UART_EXT0 || dev == FCS_STREAM_UART_EXT1) {
        /* TODO: configure external UART baud rate */
    } else {
        assert(false);
    }

    return FCS_STREAM_OK;
}


uint32_t fcs_stream_read(enum fcs_stream_device_t dev, uint8_t *buf,
uint32_t nbytes) {
    /*
    For an internal UART, we configure it in the mode required for DMA, and
    set up a single DMA transfer per buffer with ACNT = 1, BCNT = 1, CCNT > 2,
    and we poll periodically to see if the buffer is full based on changes in
    CCNT and/or the destination index. The transfer mode is set to
    AB-synchronised, with no interrupts; when CCNT gets too low we increase it.

    For an external UART, on startup we configure it with FIFOs disabled, an
    RHR interrupt, and no THR interrupt. The RHR interrupt triggers an
    AB-synchronised DMA transfer with ACNT = 1, BCNT = 1 and CCNT > 2. When
    CCNT gets too low we increase it; the number of bytes read is indicated by
    the change in CCNT.

    For TX in interal UARTs, we QDMA-trigger an A-synchronised transfer with
    ACNT = 1, BCNT = number of bytes to send, and CCNT = 1.

    For TX in external UARTs, we set up a timer firing every ~ 1.5 byte
    periods (1/16th of baud rate), which triggers an AB-synchronised transfer
    with ACNT = 1, BCNT = 1 and CCNT = number of bytes to send. The same
    output is sent to both external UARTs so only one needs to be written.
    */
    return 0;
}

uint32_t fcs_stream_skip_until_after(enum fcs_stream_device_t dev,
uint8_t ch) {
    return 0;
}

uint32_t fcs_stream_read_until_after(enum fcs_stream_device_t dev,
uint8_t ch, uint8_t *restrict buf, uint32_t nbytes) {
    return 0;
}

int16_t fcs_stream_peek(enum fcs_stream_device_t dev) {
    return -1;
}

uint32_t fcs_stream_write(enum fcs_stream_device_t dev,
const uint8_t *restrict buf, uint32_t nbytes) {
    return 0;
}
