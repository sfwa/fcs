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

extern "C" {
#include <math.h>
#include "config/config.h"
#include "util/util.h"
#include "comms/comms.h"
#include "drivers/stream.h"
#include "TRICAL/TRICAL.h"
#include "ahrs/measurement.h"
#include "ahrs/ahrs.h"

/* Prototypes for private test functions */

/* from drivers/stream.c */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, uint8_t *val,
size_t len);

/* from comms/comms.c */
size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf);
}

TEST(Comms, Initialisation) {
    fcs_comms_init();
}

TEST(Comms, ReadPacketSimpleCPU) {
    size_t length;
    uint8_t buf[256], msg1[] = "$PSFWAG,abcdefghijklmnopqrstuvwxyz*20\r\n";

    length = _fcs_stream_write_to_rx_buffer(0, msg1, sizeof(msg1) - 1u);
    ASSERT_EQ(sizeof(msg1) - 1u, length);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg1) - 1u, length);
    EXPECT_STREQ((char*)msg1, (char*)buf);

    fcs_stream_consume((enum fcs_stream_device_t)0, 255u);
}

TEST(Comms, ReadPacketSimpleRadio) {
    size_t length;
    uint8_t buf[256],
            msg1[] =
                "\x00\x03\x11\x12\n\x01\x02\x03\x04\x05\x06\x07\x08\t\x00";

    length = _fcs_stream_write_to_rx_buffer(0, msg1, sizeof(msg1) - 1u);
    ASSERT_EQ(sizeof(msg1) - 1u, length);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg1) - 1u, length);
    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[length - 1]);
    EXPECT_STREQ((char*)&msg1[1], (char*)&buf[1]);

    fcs_stream_consume((enum fcs_stream_device_t)0, 255u);
}

TEST(Comms, ReadPacketCPUThenRadio) {
    size_t length;
    uint8_t buf[256], msg1[] = "$PSFWAG,abcdefghijklmnopqrstuvwxyz*20\r\n",
            msg2[] =
                "\x00\x03\x11\x12\n\x01\x02\x03\x04\x05\x06\x07\x08\t\x00";

    length = _fcs_stream_write_to_rx_buffer(0, msg1, sizeof(msg1) - 1u);
    ASSERT_EQ(sizeof(msg1) - 1u, length);

    length = _fcs_stream_write_to_rx_buffer(0, msg2, sizeof(msg2) - 1u);
    ASSERT_EQ(sizeof(msg2) - 1u, length);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg1) - 1u, length);
    EXPECT_STREQ((char*)msg1, (char*)buf);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg2) - 1u, length);
    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[length - 1]);
    EXPECT_STREQ((char*)&msg2[1], (char*)&buf[1]);

    fcs_stream_consume((enum fcs_stream_device_t)0, 255u);
}

TEST(Comms, ReadPacketRadioThenCPU) {
    size_t length;
    uint8_t buf[256], msg2[] = "$PSFWAG,abcdefghijklmnopqrstuvwxyz*20\r\n",
            msg1[] =
                "\x00\x03\x11\x12\n\x01\x02\x03\x04\x05\x06\x07\x08\t\x00";

    length = _fcs_stream_write_to_rx_buffer(0, msg1, sizeof(msg1) - 1u);
    ASSERT_EQ(sizeof(msg1) - 1u, length);

    length = _fcs_stream_write_to_rx_buffer(0, msg2, sizeof(msg2) - 1u);
    ASSERT_EQ(sizeof(msg2) - 1u, length);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg1) - 1u, length);
    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[length - 1]);
    EXPECT_STREQ((char*)&msg1[1], (char*)&buf[1]);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg2) - 1u, length);
    EXPECT_STREQ((char*)msg2, (char*)buf);

    fcs_stream_consume((enum fcs_stream_device_t)0, 255u);
}

TEST(Comms, ReadPacketPartialRadioThenCPU) {
    size_t length;
    uint8_t buf[256], msg2[] = "$PSFWAG,abcdefghijklmnopqrstuvwxyz*20\r\n",
            msg1[] =
                "\x03\x04\x05\x06\x07\x08\t\x00";

    length = _fcs_stream_write_to_rx_buffer(0, msg1, sizeof(msg1) - 1u);
    ASSERT_EQ(sizeof(msg1) - 1u, length);

    length = _fcs_stream_write_to_rx_buffer(0, msg2, sizeof(msg2) - 1u);
    ASSERT_EQ(sizeof(msg2) - 1u, length);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg2) - 1u, length);
    EXPECT_STREQ((char*)msg2, (char*)buf);

    fcs_stream_consume((enum fcs_stream_device_t)0, 255u);
}

TEST(Comms, ReadPacketPartialCPUThenRadio) {
    size_t length;
    uint8_t buf[256], msg1[] = "qrstuvwxyz*20\r\n",
            msg2[] =
                "\x00\x03\x11\x12\n\x01\x02\x03\x04\x05\x06\x07\x08\t\x00";

    length = _fcs_stream_write_to_rx_buffer(0, msg1, sizeof(msg1) - 1u);
    ASSERT_EQ(sizeof(msg1) - 1u, length);

    length = _fcs_stream_write_to_rx_buffer(0, msg2, sizeof(msg2) - 1u);
    ASSERT_EQ(sizeof(msg2) - 1u, length);

    length = _fcs_comms_read_packet((enum fcs_stream_device_t)0, buf);
    buf[length] = 0;

    EXPECT_EQ(sizeof(msg2) - 1u, length);
    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[length - 1]);
    EXPECT_STREQ((char*)&msg2[1], (char*)&buf[1]);

    fcs_stream_consume((enum fcs_stream_device_t)0, 255u);
}
