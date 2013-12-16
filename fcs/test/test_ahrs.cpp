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
#include "ahrs/ahrs.h"
#include "drivers/stream.h"

/* Prototypes for private test functions */

/* from drivers/stream.c */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, uint8_t *val,
size_t len);

/* from ahrs/ahrs.c */
void _fcs_ahrs_update_global_state(struct ukf_state_t *restrict s,
double *restrict covariance);
bool _fcs_ahrs_read_ioboard_packet(enum fcs_stream_device_t dev,
struct sensor_packet_t *dest);
bool _fcs_ahrs_process_accelerometers(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_gyroscopes(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_magnetometers(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_gps(double *restrict p_output,
float *restrict v_output, const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_pitots(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_barometers(float *restrict output,
const struct sensor_packet_t *restrict packets);
void _fcs_ahrs_ioboard_reset_geometry(
struct fcs_ahrs_sensor_geometry_t *restrict geometry);
void _fcs_ahrs_ioboard_reset_calibration(
struct fcs_ahrs_sensor_calibration_t *restrict calibration);
uint32_t _fcs_ahrs_format_control_packet(uint8_t *buf, uint8_t tick,
const double *restrict control_values);
}

/* See ahrs/ahrs.c:40 */
struct sensor_packet_t {
    uint8_t crc;
    uint16_t tick;
    uint8_t sensor_update_flags;
    uint8_t cpu_load;
    uint16_t status;
    struct {
        int16_t x, y, z;
    } __attribute__ ((packed)) accel;
    struct {
        int16_t x, y, z;
    } __attribute__ ((packed)) gyro;
    int16_t accel_gyro_temp;
    uint16_t pressure;
    uint16_t barometer_temp;
    int16_t pitot;
    int16_t i;
    int16_t v;
    int16_t range;
    uint8_t gpin_state;
    struct {
        int16_t x, y, z;
    } __attribute__ ((packed)) mag;
    struct {
        struct {
            int32_t lat, lng, alt;
        } position;
        struct {
            int16_t n, e, d;
        } __attribute__ ((packed)) velocity;
    } __attribute__ ((packed)) gps;
    struct {
        uint8_t fix_mode_num_satellites;
        uint8_t pos_err;
    } __attribute__ ((packed)) gps_info;
} __attribute__ ((packed));

#define ACCEL_SENSITIVITY 4096.0
#define GYRO_SENSITIVITY 65.5
#define MAG_SENSITIVITY 1090.0


TEST(AHRS, Initialisation) {
    fcs_ahrs_init();
}

TEST(AHRSIO, ReadIOBoardPacket) {
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
    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
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

TEST(AHRSIO, ReadIOBoardPacketPartial) {
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
    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_FALSE(result);

    /* Write the rest of the packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_INT0, &s[31], 31);
    ASSERT_EQ(31, len);

    /* Read back and confirm success */
    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
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

TEST(AHRSIO, ReadIOBoardPacketAfterGarbage) {
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
    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
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

TEST(AHRSIO, ReadIOBoardPacketAfterPacket) {
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
    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
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

    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
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

/*
TEST(AHRSIO, WriteIOBoardPackets) {
    EXPECT_TRUE(false);
}
*/

TEST(AHRSMath, ProcessAccelDataPacketsNone) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* No data available -- should return false */
    memset(output, 0, sizeof(output));
    packets[0].sensor_update_flags = 0;
    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_accelerometers(output, packets);
    EXPECT_FALSE(result);
}

TEST(AHRSMath, ProcessAccelDataPackets0) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Packet 0 has data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].accel.x = 1234;
    packets[0].accel.y = 5678;
    packets[0].accel.z = 9012;

    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_accelerometers(output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(1234.0 / ACCEL_SENSITIVITY, output[0]);
    EXPECT_FLOAT_EQ(5678.0 / ACCEL_SENSITIVITY, output[1]);
    EXPECT_FLOAT_EQ(9012.0 / ACCEL_SENSITIVITY, output[2]);
}

TEST(AHRSMath, ProcessAccelDataPackets1) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Packet 1 has data available */
    packets[0].sensor_update_flags = 0;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].accel.x = 1234;
    packets[1].accel.y = 5678;
    packets[1].accel.z = 9012;

    result = _fcs_ahrs_process_accelerometers(output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(1234.0 / ACCEL_SENSITIVITY, output[0]);
    EXPECT_FLOAT_EQ(5678.0 / ACCEL_SENSITIVITY, output[1]);
    EXPECT_FLOAT_EQ(9012.0 / ACCEL_SENSITIVITY, output[2]);
}

TEST(AHRSMath, ProcessAccelDataPacketsBoth) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Both packets have data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].accel.x = 9012;
    packets[0].accel.y = 5678;
    packets[0].accel.z = 1234;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].accel.x = 1234;
    packets[1].accel.y = 5678;
    packets[1].accel.z = 9012;

    result = _fcs_ahrs_process_accelerometers(output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(5123.0 / ACCEL_SENSITIVITY, output[0]);
    EXPECT_FLOAT_EQ(5678.0 / ACCEL_SENSITIVITY, output[1]);
    EXPECT_FLOAT_EQ(5123.0 / ACCEL_SENSITIVITY, output[2]);

    /* TODO: check orientation / bias support */
}

TEST(AHRSMath, ProcessGyroDataPacketsNone) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* No data available -- should return false */
    packets[0].sensor_update_flags = 0;
    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_gyroscopes(output, packets);
    EXPECT_FALSE(result);
}

TEST(AHRSMath, ProcessGyroDataPackets0) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Packet 0 has data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].gyro.x = 1;
    packets[0].gyro.y = 2;
    packets[0].gyro.z = 3;

    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_gyroscopes(output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(1.0 / GYRO_SENSITIVITY, output[0]);
    EXPECT_FLOAT_EQ(2.0 / GYRO_SENSITIVITY, output[1]);
    EXPECT_FLOAT_EQ(3.0 / GYRO_SENSITIVITY, output[2]);
}

TEST(AHRSMath, ProcessGyroDataPackets1) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Packet 1 has data available */
    packets[0].sensor_update_flags = 0;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].gyro.x = 4;
    packets[1].gyro.y = 5;
    packets[1].gyro.z = 6;

    result = _fcs_ahrs_process_gyroscopes(output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(4.0 / GYRO_SENSITIVITY, output[0]);
    EXPECT_FLOAT_EQ(5.0 / GYRO_SENSITIVITY, output[1]);
    EXPECT_FLOAT_EQ(6.0 / GYRO_SENSITIVITY, output[2]);
}

TEST(AHRSMath, ProcessGyroDataPacketsBoth) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Both packets have data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].gyro.x = 1;
    packets[0].gyro.y = 2;
    packets[0].gyro.z = 3;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].gyro.x = 4;
    packets[1].gyro.y = 5;
    packets[1].gyro.z = 6;

    result = _fcs_ahrs_process_gyroscopes(output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(2.5 / GYRO_SENSITIVITY, output[0]);
    EXPECT_FLOAT_EQ(3.5 / GYRO_SENSITIVITY, output[1]);
    EXPECT_FLOAT_EQ(4.5 / GYRO_SENSITIVITY, output[2]);

    /* TODO: check orientation support */
}

TEST(AHRSMath, ProcessMagDataPacketsNone) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* No data available -- should return false */
    packets[0].sensor_update_flags = 0;
    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_magnetometers(output, packets);
    EXPECT_FALSE(result);
}

TEST(AHRSMath, ProcessMagDataPackets0) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Packet 0 has data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].mag.x = 123;
    packets[0].mag.y = 234;
    packets[0].mag.z = 345;

    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_magnetometers(output, packets);
    EXPECT_TRUE(result);
    EXPECT_NEAR(123.0 / MAG_SENSITIVITY, output[0], 2e-3);
    EXPECT_NEAR(234.0 / MAG_SENSITIVITY, output[1], 2e-3);
    EXPECT_NEAR(345.0 / MAG_SENSITIVITY, output[2], 2e-3);
}

TEST(AHRSMath, ProcessMagDataPackets1) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Packet 1 has data available */
    packets[0].sensor_update_flags = 0;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].mag.x = 456;
    packets[1].mag.y = 567;
    packets[1].mag.z = 678;

    result = _fcs_ahrs_process_magnetometers(output, packets);
    EXPECT_TRUE(result);
    EXPECT_NEAR(456.0 / MAG_SENSITIVITY, output[0], 5e-3);
    EXPECT_NEAR(567.0 / MAG_SENSITIVITY, output[1], 5e-3);
    EXPECT_NEAR(678.0 / MAG_SENSITIVITY, output[2], 5e-3);
}

TEST(AHRSMath, ProcessMagDataPacketsBoth) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output[3];

    /* Both packets have data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].mag.x = 123;
    packets[0].mag.y = 234;
    packets[0].mag.z = 345;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].mag.x = 456;
    packets[1].mag.y = 567;
    packets[1].mag.z = 678;

    result = _fcs_ahrs_process_magnetometers(output, packets);
    EXPECT_TRUE(result);
    EXPECT_NEAR(289.5 / MAG_SENSITIVITY, output[0], 3e-3);
    EXPECT_NEAR(400.5 / MAG_SENSITIVITY, output[1], 3e-3);
    EXPECT_NEAR(511.5 / MAG_SENSITIVITY, output[2], 3e-3);

    /* TODO: check bias and calibration support */
}

TEST(AHRSMath, ProcessGPSDataPacketsNone) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float velocity[3];
    double position[3];

    /* No data available -- should return false */
    packets[0].sensor_update_flags = 0;
    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_gps(position, velocity, packets);
    EXPECT_FALSE(result);
}

TEST(AHRSMath, ProcessGPSDataPackets0) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float velocity[3];
    double position[3];

    /* Packet 0 has data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].gps.position.lat = 1000000;
    packets[0].gps.position.lng = 2000000;
    packets[0].gps.position.alt = 3000;
    packets[0].gps.velocity.n = 123;
    packets[0].gps.velocity.e = 234;
    packets[0].gps.velocity.d = 345;

    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_gps(position, velocity, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(0.1 * M_PI / 180.0, position[0]);
    EXPECT_FLOAT_EQ(0.2 * M_PI / 180.0, position[1]);
    EXPECT_FLOAT_EQ(30.0, position[2]);
    EXPECT_FLOAT_EQ(123.0 * 1e-2, velocity[0]);
    EXPECT_FLOAT_EQ(234.0 * 1e-2, velocity[1]);
    EXPECT_FLOAT_EQ(345.0 * 1e-2, velocity[2]);
}

TEST(AHRSMath, ProcessGPSDataPackets1) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float velocity[3];
    double position[3];

    /* Packet 1 has data available */
    packets[0].sensor_update_flags = 0;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].gps.position.lat = 2000000;
    packets[1].gps.position.lng = 3000000;
    packets[1].gps.position.alt = 4000;
    packets[1].gps.velocity.n = 456;
    packets[1].gps.velocity.e = 567;
    packets[1].gps.velocity.d = 678;

    result = _fcs_ahrs_process_gps(position, velocity, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(0.2 * M_PI / 180.0, position[0]);
    EXPECT_FLOAT_EQ(0.3 * M_PI / 180.0, position[1]);
    EXPECT_FLOAT_EQ(40.0, position[2]);
    EXPECT_FLOAT_EQ(456.0 * 1e-2, velocity[0]);
    EXPECT_FLOAT_EQ(567.0 * 1e-2, velocity[1]);
    EXPECT_FLOAT_EQ(678.0 * 1e-2, velocity[2]);
}

TEST(AHRSMath, ProcessGPSDataPacketsBoth) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float velocity[3];
    double position[3];

    /* Both packets have data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].gps.position.lat = 1000000;
    packets[0].gps.position.lng = 2000000;
    packets[0].gps.position.alt = 3000;
    packets[0].gps.velocity.n = 123;
    packets[0].gps.velocity.e = 234;
    packets[0].gps.velocity.d = 345;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].gps.position.lat = 2000000;
    packets[1].gps.position.lng = 3000000;
    packets[1].gps.position.alt = 4000;
    packets[1].gps.velocity.n = 456;
    packets[1].gps.velocity.e = 567;
    packets[1].gps.velocity.d = 678;

    result = _fcs_ahrs_process_gps(position, velocity, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(0.15 * M_PI / 180.0, position[0]);
    EXPECT_FLOAT_EQ(0.25 * M_PI / 180.0, position[1]);
    EXPECT_FLOAT_EQ(35.0, position[2]);
    EXPECT_FLOAT_EQ(289.5 * 1e-2, velocity[0]);
    EXPECT_FLOAT_EQ(400.5 * 1e-2, velocity[1]);
    EXPECT_FLOAT_EQ(511.5 * 1e-2, velocity[2]);
}

TEST(AHRSMath, ProcessPitotDataPacketsNone) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* No data available -- should return false */
    packets[0].sensor_update_flags = 0;
    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_pitots(&output, packets);
    EXPECT_FALSE(result);
}

TEST(AHRSMath, ProcessPitotDataPackets0) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* Packet 0 has data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].pitot = 123;

    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_pitots(&output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(123.0 / 65535.0, output);
}

TEST(AHRSMath, ProcessPitotDataPackets1) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* Packet 1 has data available */
    packets[0].sensor_update_flags = 0;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].pitot = 456;

    result = _fcs_ahrs_process_pitots(&output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(456.0 / 65535.0, output);
}

TEST(AHRSMath, ProcessPitotDataPacketsBoth) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* Both packets have data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].pitot = 123;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].pitot = 456;

    result = _fcs_ahrs_process_pitots(&output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(289.5 / 65535.0, output);

    /* TODO: check bias support */
}

TEST(AHRSMath, ProcessBarometerDataPacketsNone) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* No data available -- should return false */
    packets[0].sensor_update_flags = 0;
    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_barometers(&output, packets);
    EXPECT_FALSE(result);
}

TEST(AHRSMath, ProcessBarometerDataPackets0) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* Packet 0 has data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].pressure = 123;

    packets[1].sensor_update_flags = 0;

    result = _fcs_ahrs_process_barometers(&output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(123.0 * 0.02, output);
}

TEST(AHRSMath, ProcessBarometerDataPackets1) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* Packet 1 has data available */
    packets[0].sensor_update_flags = 0;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].pressure = 456;

    result = _fcs_ahrs_process_barometers(&output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(456.0 * 0.02, output);
}

TEST(AHRSMath, ProcessBarometerDataPacketsBoth) {
    fcs_ahrs_init();

    bool result;
    struct sensor_packet_t packets[2];
    float output;

    /* Both packets have data available */
    packets[0].sensor_update_flags = 0xFFu;
    packets[0].pressure = 123;

    packets[1].sensor_update_flags = 0xFFu;
    packets[1].pressure = 456;

    result = _fcs_ahrs_process_barometers(&output, packets);
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(289.5 * 0.02, output);

    /* TODO: check bias support */
}

TEST(AHRSOutput, UpdateGlobalState) {
    struct ukf_state_t s = {
        .position = { M_PI / 2.0, -M_PI / 4.0, 30.0 },
        .velocity = { 1.0, -2.0, 3.0 },
        .acceleration = { 0.0, 0.0, 0.0 },
        .attitude = { 0.0, 0.0, 0.0, 1.0 },
        .angular_velocity = { -0.1 * M_PI, 0.2 * M_PI, -0.3 * M_PI },
        .angular_acceleration = { 0.0, 0.0, 0.0 },
        .wind_velocity = { -1.0, 2.0, -3.0 },
        .gyro_bias = { 0.0, 0.0, 0.0 }
    };
    double covariance[24] = {
        0.01 * 1e-8, 0.02 * 1e-8, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09,
        0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20,
        0.21, 0.22, 0.23, 0.24
    };

    fcs_global_state.solution_time = 1234u;
    fcs_global_state.next_waypoint_id[0] = 't';
    fcs_global_state.next_waypoint_id[1] = 'e';
    fcs_global_state.next_waypoint_id[2] = 's';
    fcs_global_state.next_waypoint_id[3] = 't';
    memset(fcs_global_state.flags, 0, 4);

    _fcs_ahrs_update_global_state(&s, covariance);
    EXPECT_EQ(1235u, fcs_global_state.solution_time);
    EXPECT_DOUBLE_EQ(90.0, fcs_global_state.lat);
    EXPECT_DOUBLE_EQ(-45.0, fcs_global_state.lon);
    EXPECT_DOUBLE_EQ(30.0, fcs_global_state.alt);
    EXPECT_DOUBLE_EQ(1.0, fcs_global_state.velocity[0]);
    EXPECT_DOUBLE_EQ(-2.0, fcs_global_state.velocity[1]);
    EXPECT_DOUBLE_EQ(3.0, fcs_global_state.velocity[2]);
    EXPECT_DOUBLE_EQ(-1.0, fcs_global_state.wind_velocity[0]);
    EXPECT_DOUBLE_EQ(2.0, fcs_global_state.wind_velocity[1]);
    EXPECT_DOUBLE_EQ(-3.0, fcs_global_state.wind_velocity[2]);
    EXPECT_DOUBLE_EQ(0.0, fcs_global_state.yaw);
    EXPECT_DOUBLE_EQ(0.0, fcs_global_state.pitch);
    EXPECT_DOUBLE_EQ(0.0, fcs_global_state.roll);
    EXPECT_DOUBLE_EQ(-18.0, fcs_global_state.angular_velocity[0]);
    EXPECT_DOUBLE_EQ(36.0, fcs_global_state.angular_velocity[1]);
    EXPECT_DOUBLE_EQ(-54.0, fcs_global_state.angular_velocity[2]);
    /* TODO: check these results against theory and practical UKF output */
    EXPECT_NEAR(176.789, fcs_global_state.lat_lon_uncertainty, 1e-3);
    EXPECT_NEAR(0.339, fcs_global_state.alt_uncertainty, 1e-3);
    EXPECT_NEAR(0.392, fcs_global_state.velocity_uncertainty[0], 1e-3);
    EXPECT_NEAR(0.438, fcs_global_state.velocity_uncertainty[1], 1e-3);
    EXPECT_NEAR(0.480, fcs_global_state.velocity_uncertainty[2], 1e-3);
    EXPECT_NEAR(0.854, fcs_global_state.wind_velocity_uncertainty[0], 1e-3);
    EXPECT_NEAR(0.876, fcs_global_state.wind_velocity_uncertainty[1], 1e-3);
    EXPECT_NEAR(0.898, fcs_global_state.wind_velocity_uncertainty[2], 1e-3);
    EXPECT_NEAR(38.902, fcs_global_state.yaw_uncertainty, 1e-3);
    EXPECT_NEAR(37.246, fcs_global_state.pitch_uncertainty, 1e-3);
    EXPECT_NEAR(35.512, fcs_global_state.roll_uncertainty, 1e-3);
    EXPECT_NEAR(40.490, fcs_global_state.angular_velocity_uncertainty[0],
                1e-3);
    EXPECT_NEAR(42.019, fcs_global_state.angular_velocity_uncertainty[1],
                1e-3);
    EXPECT_NEAR(43.493, fcs_global_state.angular_velocity_uncertainty[2],
                1e-3);
    EXPECT_EQ('A', fcs_global_state.mode_indicator);
}

TEST(AHRSOutput, UpdateGlobalStateAttitude1) {
    struct ukf_state_t s = {
        .position = { 0.0, 0.0, 0.0 },
        .velocity = { 0.0, 0.0, 0.0 },
        .acceleration = { 0.0, 0.0, 0.0 },
        .attitude = { 0.0, 0.0, 0.707107, 0.707107 },
        .angular_velocity = { 0.0, 0.0, 0.0 },
        .angular_acceleration = { 0.0, 0.0, 0.0 },
        .wind_velocity = { 0.0, 0.0, 0.0 },
        .gyro_bias = { 0.0, 0.0, 0.0 }
    };
    double covariance[24];

    memset(covariance, 0, sizeof(covariance));

    fcs_global_state.solution_time = 0;
    _fcs_ahrs_update_global_state(&s, covariance);
    ASSERT_EQ(1u, fcs_global_state.solution_time);

    EXPECT_NEAR(90.0, fcs_global_state.yaw, 1e-3);
    EXPECT_NEAR(0.0, fcs_global_state.pitch, 1e-3);
    EXPECT_NEAR(0.0, fcs_global_state.roll, 1e-3);
}

TEST(AHRSControl, SerializeControlPacket) {
    uint32_t len;
    uint8_t buf[16];
    double values[4] = { 0.1, 0.2, 0.3, 0.4 };
    uint8_t tick = 1;

    len = _fcs_ahrs_format_control_packet(buf, tick, values);
    EXPECT_EQ(14u, len);

    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[14]);
    EXPECT_STREQ("\x4" "B\x1\x1" "f\x19\x99" "33L\xCC" "f", (char*)&buf[1]);
}
