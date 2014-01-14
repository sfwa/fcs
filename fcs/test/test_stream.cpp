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

#include "test.h"

#include <cstdlib>

extern "C" {
#include "config/config.h"
#include "util/util.h"
#include "drivers/stream.h"

/* Prototypes for private test hooks */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t maxlen);
}

TEST(StreamRX, BasicRead) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Read it back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 255);
    EXPECT_EQ(8, len);
    EXPECT_STREQ((char*)s, (char*)buf);

    len = fcs_stream_consume(FCS_STREAM_UART_INT0, 255);
    EXPECT_EQ(8, len);
}

TEST(StreamRX, PartialReads) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Read the first part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    buf[4] = 0;
    EXPECT_EQ(4, len);
    EXPECT_STREQ("test", (char*)buf);

    len = fcs_stream_consume(FCS_STREAM_UART_INT0, 4);
    EXPECT_EQ(4, len);

    /* Read the second part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    EXPECT_EQ(4, len);
    EXPECT_STREQ("ing", (char*)buf);

    len = fcs_stream_consume(FCS_STREAM_UART_INT0, 255);
    EXPECT_EQ(4, len);
}

TEST(StreamRX, MixedReadsWrites) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Read the first part back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    EXPECT_EQ(4, len);
    buf[4] = 0;

    EXPECT_STREQ("test", (char*)buf);

    len = fcs_stream_consume(FCS_STREAM_UART_INT0, 4);
    EXPECT_EQ(4, len);

    /* Write another test string */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 8);
    EXPECT_EQ(8, len);

    /* Read the remainder back */
    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 4);
    EXPECT_EQ(4, len);
    EXPECT_STREQ("ing", (char*)buf);

    len = fcs_stream_consume(FCS_STREAM_UART_INT0, 4);
    EXPECT_EQ(4, len);

    len = fcs_stream_read(FCS_STREAM_UART_INT0, buf, 255);
    EXPECT_EQ(8, len);
    EXPECT_STREQ("testing", (char*)buf);
}

TEST(StreamTX, BasicWrite) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state*/
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = fcs_stream_write(FCS_STREAM_UART_INT0, s, sizeof(s));
    EXPECT_EQ(8, len);

    /* Read back and confirm */
    len = _fcs_stream_read_from_tx_buffer(FCS_STREAM_UART_INT0, buf, 255);
    EXPECT_EQ(8, len);
    EXPECT_STREQ("testing", (char*)buf);
}

TEST(StreamTX, MultipleWrites) {
    size_t len;
    uint8_t s[] = "testing",
            buf[256];

    /* Reset stream state*/
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = fcs_stream_write(FCS_STREAM_UART_INT0, s, sizeof(s));
    EXPECT_EQ(8, len);

    /* Read back and confirm */
    len = _fcs_stream_read_from_tx_buffer(FCS_STREAM_UART_INT0, buf, 255);
    EXPECT_EQ(8, len);
    EXPECT_STREQ("testing", (char*)buf);

    /* Write another test string */
    len = fcs_stream_write(FCS_STREAM_UART_INT0, s, sizeof(s));
    EXPECT_EQ(8, len);

    /* Read back and confirm */
    len = _fcs_stream_read_from_tx_buffer(FCS_STREAM_UART_INT0, buf, 255);
    EXPECT_EQ(8, len);
    EXPECT_STREQ("testing", (char*)buf);
}

TEST(StreamTX, WriteBufferFull) {
    size_t len;
    uint8_t s[] = "0123456789abcdef0123456789abcdef"
                  "0123456789abcdef0123456789abcdef"
                  "0123456789abcdef0123456789abcdef"
                  "0123456789abcdef0123456789abcdef",
            buf[512];

    /* Reset stream state*/
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write the test string */
    len = fcs_stream_write(FCS_STREAM_UART_INT0, s, 128);
    EXPECT_EQ(128, len);

    /* Write a second which overflows */
    len = fcs_stream_write(FCS_STREAM_UART_INT0, s, 129);
    EXPECT_EQ(0, len);

    /* Read back and confirm */
    len = _fcs_stream_read_from_tx_buffer(FCS_STREAM_UART_INT0, buf, 256);
    EXPECT_EQ(128, len);
}
