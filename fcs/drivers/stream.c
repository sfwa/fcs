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
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "stream.h"

/*
RX/TX buffers per serial link, with read/write indices -- these form circular
buffers.
*/
static uint8_t rx_buffers[FCS_STREAM_NUM_DEVICES][FCS_STREAM_BUFFER_SIZE];
static uint8_t tx_buffers[FCS_STREAM_NUM_DEVICES][FCS_STREAM_BUFFER_SIZE];
static uint16_t rx_read_idx[FCS_STREAM_NUM_DEVICES];
static uint16_t rx_write_idx[FCS_STREAM_NUM_DEVICES];
static uint16_t tx_read_idx[FCS_STREAM_NUM_DEVICES];
static uint16_t tx_write_idx[FCS_STREAM_NUM_DEVICES];

size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, uint8_t *val,
size_t len);
int32_t _fcs_stream_check_overrun(enum fcs_stream_device_t dev);


/* Buffer access functions for the test harness */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len) {
    assert(buffer_idx < FCS_STREAM_NUM_DEVICES);
    assert(len < FCS_STREAM_BUFFER_SIZE);

    size_t i;
    for (i = 0; i < len; rx_write_idx[buffer_idx]++, i++) {
        rx_buffers[buffer_idx][
            rx_write_idx[buffer_idx] % FCS_STREAM_BUFFER_SIZE] = val[i];
        assert(!_fcs_stream_check_overrun(buffer_idx));
    }

    return len;
}

size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, uint8_t *val,
size_t len) {
    assert(buffer_idx < FCS_STREAM_NUM_DEVICES);
    assert(len <= FCS_STREAM_BUFFER_SIZE);

    size_t i;
    for (i = 0; i < len && tx_read_idx[buffer_idx] - tx_write_idx[buffer_idx];
            tx_read_idx[buffer_idx]++, i++) {
        val[i] = tx_buffers[buffer_idx][
            tx_write_idx[buffer_idx] % FCS_STREAM_BUFFER_SIZE];
        assert(!_fcs_stream_check_overrun(buffer_idx));
    }

    return len;
}

/* Stream maintenance -- update DMA pointers and check for overrun */
int32_t _fcs_stream_check_overrun(enum fcs_stream_device_t dev) {
    assert(dev < FCS_STREAM_NUM_DEVICES);

    /* TODO: update rx_write_idx / tx_read_idx based on current DMA state */

    /*
    If the write index gets more than 255 values ahead of the read index,
    there's been an overrun -- disable DMA and reset the buffers
    */
    if (rx_write_idx[dev] - rx_read_idx[dev] >= FCS_STREAM_BUFFER_SIZE ||
            tx_write_idx[dev] - tx_read_idx[dev] >= FCS_STREAM_BUFFER_SIZE) {
        fcs_stream_open(dev);
        return 1;
    } else {
        return 0;
    }
}

/*
fcs_stream_set_rate - if the stream device is a UART, sets the connection
baud rate to the value provided in "baud".

Returns FCS_STREAM_OK if OK, or FCS_STREAM_ERROR if the device is not a UART
or the baud rate is not valid.
*/
enum fcs_stream_result_t fcs_stream_set_rate(enum fcs_stream_device_t dev,
uint32_t baud) {
    assert(dev < FCS_STREAM_NUM_DEVICES);

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

enum fcs_stream_result_t fcs_stream_open(enum fcs_stream_device_t dev) {
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

    assert(dev < FCS_STREAM_NUM_DEVICES);

    rx_read_idx[dev] = 0;
    rx_write_idx[dev] = 0;
    tx_read_idx[dev] = 0;
    tx_write_idx[dev] = 0;

    return FCS_STREAM_OK;
}

/*
fcs_stream_bytes_available - returns the number of bytes available to read in
"dev". This count won't necessarily be accurate for long since the bytes are
transferred via DMA.
*/
uint32_t fcs_stream_bytes_available(enum fcs_stream_device_t dev) {
    assert(FCS_STREAM_BUFFER_SIZE == 256u); /* & 0xFF is faster than % 256 */
    assert(dev < FCS_STREAM_NUM_DEVICES);

    /* Reset the stream and abort if there's an overrun */
    if (_fcs_stream_check_overrun(dev)) {
        return 0;
    }

    return rx_write_idx[dev] - rx_read_idx[dev];
}

/*
fcs_stream_read - reads up to "nbytes" from the device's input buffer into
"buf".

Returns the number of bytes actually read, in the range [0, nbytes].
*/
uint32_t fcs_stream_read(enum fcs_stream_device_t dev, uint8_t *restrict buf,
uint32_t nbytes) {
    assert(FCS_STREAM_BUFFER_SIZE == 256u); /* & 0xFF is faster than % 256 */
    assert(dev < FCS_STREAM_NUM_DEVICES);
    assert(nbytes);
    assert(buf);

    /* Reset the stream and abort if there's an overrun */
    if (_fcs_stream_check_overrun(dev)) {
        return 0;
    }

    size_t i;
    for (i = 0; i < nbytes && rx_read_idx[dev] - rx_write_idx[dev];
            rx_read_idx[dev]++, i++) {
        buf[i] = rx_buffers[dev][rx_read_idx[dev] & 0xFFu];
    }

    return i;
}

/*
fcs_stream_skip_until_after - discard bytes from the device's input buffer
until (and including) the first occurrence of "ch".

If "ch" is not present in the input buffer, the discarded bytes are restored.

Returns the number of bytes skipped, in the range [0, nbytes].
*/
uint32_t fcs_stream_skip_until_after(enum fcs_stream_device_t dev,
uint8_t ch) {
    assert(dev < FCS_STREAM_NUM_DEVICES);

    /* Reset the stream and abort if there's an overrun */
    if (_fcs_stream_check_overrun(dev)) {
        return 0;
    }

    uint8_t cur_ch;
    size_t i, initial_read_idx = rx_read_idx[dev];
    for (i = 0; rx_read_idx[dev] - rx_write_idx[dev];
            rx_read_idx[dev]++, i++) {
        cur_ch = rx_buffers[dev][rx_read_idx[dev] & 0xFFu];
        if (cur_ch == ch) {
            /* Increment these because we want to read until after "ch" */
            rx_read_idx[dev]++;
            i++;
            return i;
        }
    }

    /* Character not found -- restore the initial read index */
    rx_read_idx[dev] = initial_read_idx;

    return 0;
}

/*
fcs_stream_read_until_after - reads up to "nbytes" bytes from the device's
input buffer up to (and including) the first occurrence of "ch".

If "ch" is not present in the input buffer, the discarded bytes are restored,
and no bytes are read.

Returns the number of bytes read, in the range [0, nbytes].
*/
uint32_t fcs_stream_read_until_after(enum fcs_stream_device_t dev,
uint8_t ch, uint8_t *restrict buf, uint32_t nbytes) {
    assert(dev < FCS_STREAM_NUM_DEVICES);
    assert(nbytes);
    assert(buf);

    /* Reset the stream and abort if there's an overrun */
    if (_fcs_stream_check_overrun(dev)) {
        return 0;
    }

    uint8_t cur_ch;
    size_t i, initial_read_idx = rx_read_idx[dev];
    for (i = 0; i < nbytes && rx_read_idx[dev] - rx_write_idx[dev];
            rx_read_idx[dev]++, i++) {
        cur_ch = rx_buffers[dev][rx_read_idx[dev] & 0xFFu];
        buf[i] = cur_ch;

        if (cur_ch == ch) {
            /* Increment these because we want to read until after "ch" */
            rx_read_idx[dev]++;
            i++;
            return i;
        }
    }

    /* Character not found -- restore the initial read index */
    rx_read_idx[dev] = initial_read_idx;

    return 0;
}

/*
fcs_stream_peek - return the next character in the stream, or -1 if none. Does
not consume the character, so it will be returned by future peeks/reads.
*/
int16_t fcs_stream_peek(enum fcs_stream_device_t dev) {
    assert(dev < FCS_STREAM_NUM_DEVICES);

    /* Reset the stream and abort if there's an overrun */
    if (_fcs_stream_check_overrun(dev)) {
        return -1;
    }

    if (rx_read_idx[dev] != rx_write_idx[dev]) {
        return rx_buffers[dev][rx_read_idx[dev] & 0xFFu];
    } else {
        return -1;
    }
}

/*
fcs_stream_write - writes up to "nbytes" from "buf" into the device's output
buffer.

Returns the number of bytes actually written, in the range [0, nbytes].
Assuming the caller is writing at a rate lower than the maximum send rate of
the device, the return value will always be equal to "nbytes".
*/
uint32_t fcs_stream_write(enum fcs_stream_device_t dev,
const uint8_t *restrict buf, uint32_t nbytes) {
    assert(dev < FCS_STREAM_NUM_DEVICES);
    assert(nbytes);
    assert(buf);

    /* Reset the stream and abort if there's an overrun */
    if (_fcs_stream_check_overrun(dev)) {
        return 0;
    }

    size_t i;
    for (i = 0; i < nbytes && tx_write_idx[dev] - tx_read_idx[dev] < 256u;
            tx_write_idx[dev]++, i++) {
        tx_buffers[dev][tx_write_idx[dev] & 0xFFu] = buf[i];
    }

    /* TODO: trigger a DMA transfer / copy the number of bytes to the PaRAM */

    return i;
}
