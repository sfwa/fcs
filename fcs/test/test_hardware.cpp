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
    size_t len;
    struct sensor_packet_t packet;
    uint8_t s[] = "\x00\x03\x61\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
                  "\x02\x08\x02\t\x02\x10\x02\x11\x02\x12\x02\x13\x02\x14\x02"
                  "\x15\x02\x16\x02\x17\x03\x18\x19\x02 \x02!\x02\"\x01\x01"
                  "\x02#\x01\x01\x02$\x01\x01\x02%\x02&\x02')(\x00";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, s, 62);
    ASSERT_EQ(62, len);

    /* Read back and confirm results */
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_TRUE(result);

    EXPECT_EQ(0x61, packet.crc);
    EXPECT_EQ(0x0100, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0400, packet.status);
    EXPECT_EQ(0x0500, packet.accel.x);
    EXPECT_EQ(0x0600, packet.accel.y);
    EXPECT_EQ(0x0700, packet.accel.z);
    EXPECT_EQ(0x0800, packet.gyro.x);
    EXPECT_EQ(0x0900, packet.gyro.y);
    EXPECT_EQ(0x1000, packet.gyro.z);
    EXPECT_EQ(0x1100, packet.accel_gyro_temp);
    EXPECT_EQ(0x1200, packet.pressure);
    EXPECT_EQ(0x1300, packet.barometer_temp);
    EXPECT_EQ(0x1400, packet.pitot);
    EXPECT_EQ(0x1500, packet.i);
    EXPECT_EQ(0x1600, packet.v);
    EXPECT_EQ(0x1700, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x1900, packet.mag.x);
    EXPECT_EQ(0x2000, packet.mag.y);
    EXPECT_EQ(0x2100, packet.mag.z);
    EXPECT_EQ(0x22000000, packet.gps.position.lat);
    EXPECT_EQ(0x23000000, packet.gps.position.lng);
    EXPECT_EQ(0x24000000, packet.gps.position.alt);
    EXPECT_EQ(0x2500, packet.gps.velocity.n);
    EXPECT_EQ(0x2600, packet.gps.velocity.e);
    EXPECT_EQ(0x2700, packet.gps.velocity.d);
    EXPECT_EQ(0x28, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x29, packet.gps_info.pos_err);
}

TEST(Hardware, ReadIOBoardPacketPartial) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
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
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_FALSE(result);

    /* Write the rest of the packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, &s[31], 31);
    ASSERT_EQ(31, len);

    /* Read back and confirm success */
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_TRUE(result);

    EXPECT_EQ(0x61, packet.crc);
    EXPECT_EQ(0x0100, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0400, packet.status);
    EXPECT_EQ(0x0500, packet.accel.x);
    EXPECT_EQ(0x0600, packet.accel.y);
    EXPECT_EQ(0x0700, packet.accel.z);
    EXPECT_EQ(0x0800, packet.gyro.x);
    EXPECT_EQ(0x0900, packet.gyro.y);
    EXPECT_EQ(0x1000, packet.gyro.z);
    EXPECT_EQ(0x1100, packet.accel_gyro_temp);
    EXPECT_EQ(0x1200, packet.pressure);
    EXPECT_EQ(0x1300, packet.barometer_temp);
    EXPECT_EQ(0x1400, packet.pitot);
    EXPECT_EQ(0x1500, packet.i);
    EXPECT_EQ(0x1600, packet.v);
    EXPECT_EQ(0x1700, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x1900, packet.mag.x);
    EXPECT_EQ(0x2000, packet.mag.y);
    EXPECT_EQ(0x2100, packet.mag.z);
    EXPECT_EQ(0x22000000, packet.gps.position.lat);
    EXPECT_EQ(0x23000000, packet.gps.position.lng);
    EXPECT_EQ(0x24000000, packet.gps.position.alt);
    EXPECT_EQ(0x2500, packet.gps.velocity.n);
    EXPECT_EQ(0x2600, packet.gps.velocity.e);
    EXPECT_EQ(0x2700, packet.gps.velocity.d);
    EXPECT_EQ(0x28, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x29, packet.gps_info.pos_err);
}

TEST(Hardware, ReadIOBoardPacketAfterGarbage) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
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

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(70, len);

    /* Read back and confirm results */
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_TRUE(result);

    EXPECT_EQ(0x61, packet.crc);
    EXPECT_EQ(0x0100, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0400, packet.status);
    EXPECT_EQ(0x0500, packet.accel.x);
    EXPECT_EQ(0x0600, packet.accel.y);
    EXPECT_EQ(0x0700, packet.accel.z);
    EXPECT_EQ(0x0800, packet.gyro.x);
    EXPECT_EQ(0x0900, packet.gyro.y);
    EXPECT_EQ(0x1000, packet.gyro.z);
    EXPECT_EQ(0x1100, packet.accel_gyro_temp);
    EXPECT_EQ(0x1200, packet.pressure);
    EXPECT_EQ(0x1300, packet.barometer_temp);
    EXPECT_EQ(0x1400, packet.pitot);
    EXPECT_EQ(0x1500, packet.i);
    EXPECT_EQ(0x1600, packet.v);
    EXPECT_EQ(0x1700, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x1900, packet.mag.x);
    EXPECT_EQ(0x2000, packet.mag.y);
    EXPECT_EQ(0x2100, packet.mag.z);
    EXPECT_EQ(0x22000000, packet.gps.position.lat);
    EXPECT_EQ(0x23000000, packet.gps.position.lng);
    EXPECT_EQ(0x24000000, packet.gps.position.alt);
    EXPECT_EQ(0x2500, packet.gps.velocity.n);
    EXPECT_EQ(0x2600, packet.gps.velocity.e);
    EXPECT_EQ(0x2700, packet.gps.velocity.d);
    EXPECT_EQ(0x28, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x29, packet.gps_info.pos_err);
}

TEST(Hardware, ReadIOBoardPacketAfterPacket) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
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

    len = fcs_stream_bytes_available(FCS_STREAM_UART_INT0);
    EXPECT_EQ(124, len);

    /* Read back and confirm results */
    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_TRUE(result);

    EXPECT_EQ(0x61, packet.crc);
    EXPECT_EQ(0x0100, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0400, packet.status);
    EXPECT_EQ(0x0500, packet.accel.x);
    EXPECT_EQ(0x0600, packet.accel.y);
    EXPECT_EQ(0x0700, packet.accel.z);
    EXPECT_EQ(0x0800, packet.gyro.x);
    EXPECT_EQ(0x0900, packet.gyro.y);
    EXPECT_EQ(0x1000, packet.gyro.z);
    EXPECT_EQ(0x1100, packet.accel_gyro_temp);
    EXPECT_EQ(0x1200, packet.pressure);
    EXPECT_EQ(0x1300, packet.barometer_temp);
    EXPECT_EQ(0x1400, packet.pitot);
    EXPECT_EQ(0x1500, packet.i);
    EXPECT_EQ(0x1600, packet.v);
    EXPECT_EQ(0x1700, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x1900, packet.mag.x);
    EXPECT_EQ(0x2000, packet.mag.y);
    EXPECT_EQ(0x2100, packet.mag.z);
    EXPECT_EQ(0x22000000, packet.gps.position.lat);
    EXPECT_EQ(0x23000000, packet.gps.position.lng);
    EXPECT_EQ(0x24000000, packet.gps.position.alt);
    EXPECT_EQ(0x2500, packet.gps.velocity.n);
    EXPECT_EQ(0x2600, packet.gps.velocity.e);
    EXPECT_EQ(0x2700, packet.gps.velocity.d);
    EXPECT_EQ(0x28, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x29, packet.gps_info.pos_err);

    memset(&packet, 0, sizeof(packet));

    result = _fcs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_TRUE(result);

    EXPECT_EQ(0x61, packet.crc);
    EXPECT_EQ(0x0100, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0400, packet.status);
    EXPECT_EQ(0x0500, packet.accel.x);
    EXPECT_EQ(0x0600, packet.accel.y);
    EXPECT_EQ(0x0700, packet.accel.z);
    EXPECT_EQ(0x0800, packet.gyro.x);
    EXPECT_EQ(0x0900, packet.gyro.y);
    EXPECT_EQ(0x1000, packet.gyro.z);
    EXPECT_EQ(0x1100, packet.accel_gyro_temp);
    EXPECT_EQ(0x1200, packet.pressure);
    EXPECT_EQ(0x1300, packet.barometer_temp);
    EXPECT_EQ(0x1400, packet.pitot);
    EXPECT_EQ(0x1500, packet.i);
    EXPECT_EQ(0x1600, packet.v);
    EXPECT_EQ(0x1700, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x1900, packet.mag.x);
    EXPECT_EQ(0x2000, packet.mag.y);
    EXPECT_EQ(0x2100, packet.mag.z);
    EXPECT_EQ(0x22000000, packet.gps.position.lat);
    EXPECT_EQ(0x23000000, packet.gps.position.lng);
    EXPECT_EQ(0x24000000, packet.gps.position.alt);
    EXPECT_EQ(0x2500, packet.gps.velocity.n);
    EXPECT_EQ(0x2600, packet.gps.velocity.e);
    EXPECT_EQ(0x2700, packet.gps.velocity.d);
    EXPECT_EQ(0x28, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x29, packet.gps_info.pos_err);
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
