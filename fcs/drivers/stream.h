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

#ifndef _FCS_STREAM_H
#define _FCS_STREAM_H

#define FCS_STREAM_BUFFER_SIZE 1024u
#define FCS_STREAM_BUFFER_MASK (FCS_STREAM_BUFFER_SIZE - 1u)
#define FCS_STREAM_MAX_RATE 24000000u

enum fcs_stream_device_t {
    FCS_STREAM_UART_INT0,
    FCS_STREAM_UART_INT1,
    FCS_STREAM_UART_EXT0,
    FCS_STREAM_UART_EXT1,
    FCS_STREAM_USB,

    /* Track the number of devices */
    FCS_STREAM_NUM_DEVICES
};

enum fcs_stream_result_t {
    FCS_STREAM_OK,
    FCS_STREAM_ERROR
};

/*
fcs_stream_open -- configure a stream and reset its buffer state
*/
enum fcs_stream_result_t fcs_stream_open(enum fcs_stream_device_t dev);

/*
fcs_stream_set_rate - if the stream device is a UART, sets the connection
baud rate to the value provided in "baud".

Returns FCS_STREAM_OK if OK, or FCS_STREAM_ERROR if the device is not a UART
or the baud rate is not valid.
*/
enum fcs_stream_result_t fcs_stream_set_rate(enum fcs_stream_device_t dev,
uint32_t baud);

/*
fcs_stream_check_error -- checks if an error has occurred
*/
enum fcs_stream_result_t fcs_stream_check_error(enum fcs_stream_device_t dev);

/*
fcs_stream_read - copy up to nbytes characters to the output buffer.

Returns the number of bytes copied, in the range [0, nbytes].
*/
size_t fcs_stream_read(enum fcs_stream_device_t dev, uint8_t *restrict buf,
size_t nbytes);

/*
fcs_stream_write - writes up to "nbytes" from "buf" into the device's output
buffer.

Returns the number of bytes actually written, in the range [0, nbytes].
Assuming the caller is writing at a rate lower than the maximum send rate of
the device, the return value will always be equal to "nbytes".
*/
size_t fcs_stream_write(enum fcs_stream_device_t dev,
const uint8_t *restrict buf, size_t nbytes);

#endif
