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
#include <cmath>

extern "C" {
#include "config/config.h"
#include "util/util.h"
#include "util/3dmath.h"
#include "comms/comms.h"
#include "ukf/cukf.h"
#include "TRICAL/TRICAL.h"
#include "ahrs/measurement.h"
#include "ahrs/ahrs.h"
#include "drivers/stream.h"
#include "hardware/board.h"

/* Prototypes for private test hooks */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t maxlen);

bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements);
uint32_t _fcs_format_control_packet(uint8_t *buf, uint8_t tick,
const double *restrict control_values);
}

TEST(Hardware, ReadIOBoardPacket) {
    bool result;
    struct fcs_measurement_log_t measurements;
    size_t len;
    uint8_t s[] = "\x00\x03\x61\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
                  "\x02\x08\x02\t\x02\x10\x02\x11\x02\x12\x02\x13\x02\x14\x02"
                  "\x15\x02\x16\x02\x17\x03\x18\x19\x02 \x02!\x02\"\x01\x01"
                  "\x02#\x01\x01\x02$\x01\x01\x02%\x02&\x02')(\x00";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 62u);
    ASSERT_EQ(62u, len);

    /* Read back and confirm results */
    fcs_measurement_log_init(&measurements, 0);
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, 0, &measurements);
    ASSERT_TRUE(result);

    /* Check a measurement */
    struct fcs_measurement_t measurement;
    double value[4];

    result = fcs_measurement_log_find(&measurements,
                                      FCS_MEASUREMENT_TYPE_GYROSCOPE, 0,
                                      &measurement);
    ASSERT_TRUE(result);
    EXPECT_EQ(2048, measurement.data.i16[1]);
    EXPECT_EQ(2304, measurement.data.i16[0]);
    EXPECT_EQ(4096, -measurement.data.i16[2]);

    len = fcs_measurement_get_values(&measurement, value);
    ASSERT_EQ(3u, len);
    ASSERT_EQ(16u, fcs_measurement_get_precision_bits(&measurement));
    EXPECT_FLOAT_EQ(2048.0 / 32768.0, value[1]);
    EXPECT_FLOAT_EQ(2304.0 / 32768.0, value[0]);
    EXPECT_FLOAT_EQ(4096.0 / 32768.0, -value[2]);
}

TEST(Hardware, ReadIOBoardPacketPartial) {
    bool result;
    size_t len;
    struct fcs_measurement_log_t measurements;
    uint8_t s[] = "\x00\x03\x61\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
                  "\x02\x08\x02\t\x02\x10\x02\x11\x02\x12\x02\x13\x02\x14\x02"
                  "\x15\x02\x16\x02\x17\x03\x18\x19\x02 \x02!\x02\"\x01\x01"
                  "\x02#\x01\x01\x02$\x01\x01\x02%\x02&\x02')(\x00";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write a partial packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 31);
    ASSERT_EQ(31, len);

    /* Read back and confirm failure */
    fcs_measurement_log_init(&measurements, 0);
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, 0, &measurements);
    ASSERT_FALSE(result);

    /* Write the rest of the packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, &s[31], 31);
    ASSERT_EQ(31, len);

    /* Read back and confirm success */
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, 0, &measurements);
    ASSERT_TRUE(result);

    /* Check a measurement */
    struct fcs_measurement_t measurement;
    double value[4];

    result = fcs_measurement_log_find(&measurements,
                                      FCS_MEASUREMENT_TYPE_GYROSCOPE, 0,
                                      &measurement);
    ASSERT_TRUE(result);
    EXPECT_EQ(2048, measurement.data.i16[1]);
    EXPECT_EQ(2304, measurement.data.i16[0]);
    EXPECT_EQ(4096, -measurement.data.i16[2]);

    len = fcs_measurement_get_values(&measurement, value);
    ASSERT_EQ(3u, len);
    ASSERT_EQ(16u, fcs_measurement_get_precision_bits(&measurement));
    EXPECT_FLOAT_EQ(2048.0 / 32768.0, value[1]);
    EXPECT_FLOAT_EQ(2304.0 / 32768.0, value[0]);
    EXPECT_FLOAT_EQ(4096.0 / 32768.0, -value[2]);
}

TEST(Hardware, ReadIOBoardPacketAfterGarbage) {
    bool result;
    size_t len;
    struct fcs_measurement_log_t measurements;
    uint8_t s[] = "\x00\x03\x61\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
                  "\x02\x08\x02\t\x02\x10\x02\x11\x02\x12\x02\x13\x02\x14\x02"
                  "\x15\x02\x16\x02\x17\x03\x18\x19\x02 \x02!\x02\"\x01\x01"
                  "\x02#\x01\x01\x02$\x01\x01\x02%\x02&\x02')(\x00";
    uint8_t garbage[] = "abcdefg";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write some NUL-terminated garbage */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, garbage, 8);
    ASSERT_EQ(8, len);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 62);
    ASSERT_EQ(62, len);

    /* Read back and confirm results */
    fcs_measurement_log_init(&measurements, 0);
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, 0, &measurements);
    ASSERT_TRUE(result);

    /* Check a measurement */
    struct fcs_measurement_t measurement;
    double value[4];

    result = fcs_measurement_log_find(&measurements,
                                      FCS_MEASUREMENT_TYPE_GYROSCOPE, 0,
                                      &measurement);
    ASSERT_TRUE(result);
    EXPECT_EQ(2048, measurement.data.i16[1]);
    EXPECT_EQ(2304, measurement.data.i16[0]);
    EXPECT_EQ(4096, -measurement.data.i16[2]);

    len = fcs_measurement_get_values(&measurement, value);
    ASSERT_EQ(3u, len);
    ASSERT_EQ(16u, fcs_measurement_get_precision_bits(&measurement));
    EXPECT_FLOAT_EQ(2048.0 / 32768.0, value[1]);
    EXPECT_FLOAT_EQ(2304.0 / 32768.0, value[0]);
    EXPECT_FLOAT_EQ(4096.0 / 32768.0, -value[2]);
}

TEST(Hardware, ReadIOBoardPacketAfterPacket) {
    bool result;
    size_t len;
    struct fcs_measurement_log_t measurements;
    uint8_t s[] = "\x00\x03\x61\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
                  "\x02\x08\x02\t\x02\x10\x02\x11\x02\x12\x02\x13\x02\x14\x02"
                  "\x15\x02\x16\x02\x17\x03\x18\x19\x02 \x02!\x02\"\x01\x01"
                  "\x02#\x01\x01\x02$\x01\x01\x02%\x02&\x02')(\x00";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write some NUL-terminated garbage */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 62);
    ASSERT_EQ(62, len);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 62);
    ASSERT_EQ(62, len);

    /* Read back and confirm results */
    fcs_measurement_log_init(&measurements, 0);
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, 0, &measurements);
    ASSERT_TRUE(result);

    /* Check a measurement */
    struct fcs_measurement_t measurement;
    double value[4];

    result = fcs_measurement_log_find(&measurements,
                                      FCS_MEASUREMENT_TYPE_GYROSCOPE, 0,
                                      &measurement);
    ASSERT_TRUE(result);
    EXPECT_EQ(2048, measurement.data.i16[1]);
    EXPECT_EQ(2304, measurement.data.i16[0]);
    EXPECT_EQ(4096, -measurement.data.i16[2]);

    len = fcs_measurement_get_values(&measurement, value);
    ASSERT_EQ(3u, len);
    EXPECT_FLOAT_EQ(2048.0 / 32768.0, value[1]);
    EXPECT_FLOAT_EQ(2304.0 / 32768.0, value[0]);
    EXPECT_FLOAT_EQ(4096.0 / 32768.0, -value[2]);

    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, 1, &measurements);
    ASSERT_TRUE(result);

    /* Check a measurement */
    result = fcs_measurement_log_find(&measurements,
                                      FCS_MEASUREMENT_TYPE_GYROSCOPE, 1,
                                      &measurement);
    ASSERT_TRUE(result);
    EXPECT_EQ(2048, measurement.data.i16[1]);
    EXPECT_EQ(2304, measurement.data.i16[0]);
    EXPECT_EQ(4096, -measurement.data.i16[2]);

    len = fcs_measurement_get_values(&measurement, value);
    ASSERT_EQ(3u, len);
    ASSERT_EQ(16u, fcs_measurement_get_precision_bits(&measurement));
    EXPECT_FLOAT_EQ(2048.0 / 32768.0, value[1]);
    EXPECT_FLOAT_EQ(2304.0 / 32768.0, value[0]);
    EXPECT_FLOAT_EQ(4096.0 / 32768.0, -value[2]);
}

TEST(Hardware, SerializeControlPacket) {
    uint32_t len;
    uint8_t buf[16];
    double values[4] = { 0.1, 0.2, 0.3, 0.4 };
    uint8_t tick = 1;

    len = _fcs_format_control_packet(buf, tick, values);
    EXPECT_EQ(14u, len);

    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[14]);
    EXPECT_STREQ("\x4" "B\x1\x1" "f\x19\x99" "33L\xCC" "f", (char*)&buf[1]);
}
