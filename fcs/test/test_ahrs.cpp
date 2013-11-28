#include "test.h"

#include <cstdlib>
#include <cmath>

extern "C" {
#include "config/config.h"
#include "util/util.h"
#include "util/3dmath.h"
#include "comms/comms.h"
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
bool _fcs_ahrs_process_accelerometers(float *restrict output);
bool _fcs_ahrs_process_gyroscopes(float *restrict output);
bool _fcs_ahrs_process_magnetometers(float *restrict output);
bool _fcs_ahrs_process_gps(double *restrict p_output,
float *restrict v_output);
bool _fcs_ahrs_process_pitots(float *restrict output);
bool _fcs_ahrs_process_barometers(float *restrict output);
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

TEST(AHRSIO, ReadIOBoardPacket) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
    uint8_t s[] = "\x00\x01\x02\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
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

    EXPECT_EQ(0x00, packet.crc);
    EXPECT_EQ(0x0001, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0004, packet.status);
    EXPECT_EQ(0x0005, packet.accel.x);
    EXPECT_EQ(0x0006, packet.accel.y);
    EXPECT_EQ(0x0007, packet.accel.z);
    EXPECT_EQ(0x0008, packet.gyro.x);
    EXPECT_EQ(0x0009, packet.gyro.y);
    EXPECT_EQ(0x0010, packet.gyro.z);
    EXPECT_EQ(0x0011, packet.accel_gyro_temp);
    EXPECT_EQ(0x0012, packet.pressure);
    EXPECT_EQ(0x0013, packet.barometer_temp);
    EXPECT_EQ(0x0014, packet.pitot);
    EXPECT_EQ(0x0015, packet.i);
    EXPECT_EQ(0x0016, packet.v);
    EXPECT_EQ(0x0017, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x0019, packet.mag.x);
    EXPECT_EQ(0x0020, packet.mag.y);
    EXPECT_EQ(0x0021, packet.mag.z);
    EXPECT_EQ(0x00000022, packet.gps.position.lat);
    EXPECT_EQ(0x00000023, packet.gps.position.lng);
    EXPECT_EQ(0x00000024, packet.gps.position.alt);
    EXPECT_EQ(0x0025, packet.gps.velocity.n);
    EXPECT_EQ(0x0026, packet.gps.velocity.e);
    EXPECT_EQ(0x0027, packet.gps.velocity.d);
    EXPECT_EQ(0x0028, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x0029, packet.gps_info.pos_err);
}

TEST(AHRSIO, ReadIOBoardPacketPartial) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
    uint8_t s[] = "\x00\x01\x02\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
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

    EXPECT_EQ(0x00, packet.crc);
    EXPECT_EQ(0x0001, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0004, packet.status);
    EXPECT_EQ(0x0005, packet.accel.x);
    EXPECT_EQ(0x0006, packet.accel.y);
    EXPECT_EQ(0x0007, packet.accel.z);
    EXPECT_EQ(0x0008, packet.gyro.x);
    EXPECT_EQ(0x0009, packet.gyro.y);
    EXPECT_EQ(0x0010, packet.gyro.z);
    EXPECT_EQ(0x0011, packet.accel_gyro_temp);
    EXPECT_EQ(0x0012, packet.pressure);
    EXPECT_EQ(0x0013, packet.barometer_temp);
    EXPECT_EQ(0x0014, packet.pitot);
    EXPECT_EQ(0x0015, packet.i);
    EXPECT_EQ(0x0016, packet.v);
    EXPECT_EQ(0x0017, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x0019, packet.mag.x);
    EXPECT_EQ(0x0020, packet.mag.y);
    EXPECT_EQ(0x0021, packet.mag.z);
    EXPECT_EQ(0x00000022, packet.gps.position.lat);
    EXPECT_EQ(0x00000023, packet.gps.position.lng);
    EXPECT_EQ(0x00000024, packet.gps.position.alt);
    EXPECT_EQ(0x0025, packet.gps.velocity.n);
    EXPECT_EQ(0x0026, packet.gps.velocity.e);
    EXPECT_EQ(0x0027, packet.gps.velocity.d);
    EXPECT_EQ(0x0028, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x0029, packet.gps_info.pos_err);
}

TEST(AHRSIO, ReadIOBoardPacketAfterGarbage) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
    uint8_t s[] = "\x00\x01\x02\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
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

    EXPECT_EQ(0x00, packet.crc);
    EXPECT_EQ(0x0001, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0004, packet.status);
    EXPECT_EQ(0x0005, packet.accel.x);
    EXPECT_EQ(0x0006, packet.accel.y);
    EXPECT_EQ(0x0007, packet.accel.z);
    EXPECT_EQ(0x0008, packet.gyro.x);
    EXPECT_EQ(0x0009, packet.gyro.y);
    EXPECT_EQ(0x0010, packet.gyro.z);
    EXPECT_EQ(0x0011, packet.accel_gyro_temp);
    EXPECT_EQ(0x0012, packet.pressure);
    EXPECT_EQ(0x0013, packet.barometer_temp);
    EXPECT_EQ(0x0014, packet.pitot);
    EXPECT_EQ(0x0015, packet.i);
    EXPECT_EQ(0x0016, packet.v);
    EXPECT_EQ(0x0017, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x0019, packet.mag.x);
    EXPECT_EQ(0x0020, packet.mag.y);
    EXPECT_EQ(0x0021, packet.mag.z);
    EXPECT_EQ(0x00000022, packet.gps.position.lat);
    EXPECT_EQ(0x00000023, packet.gps.position.lng);
    EXPECT_EQ(0x00000024, packet.gps.position.alt);
    EXPECT_EQ(0x0025, packet.gps.velocity.n);
    EXPECT_EQ(0x0026, packet.gps.velocity.e);
    EXPECT_EQ(0x0027, packet.gps.velocity.d);
    EXPECT_EQ(0x0028, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x0029, packet.gps_info.pos_err);
}

TEST(AHRSIO, ReadIOBoardPacketAfterPacket) {
    bool result;
    size_t len;
    struct sensor_packet_t packet;
    uint8_t s[] = "\x00\x01\x02\x01\x04\x02\x03\x04\x02\x05\x02\x06\x02\x07"
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

    EXPECT_EQ(0x00, packet.crc);
    EXPECT_EQ(0x0001, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0004, packet.status);
    EXPECT_EQ(0x0005, packet.accel.x);
    EXPECT_EQ(0x0006, packet.accel.y);
    EXPECT_EQ(0x0007, packet.accel.z);
    EXPECT_EQ(0x0008, packet.gyro.x);
    EXPECT_EQ(0x0009, packet.gyro.y);
    EXPECT_EQ(0x0010, packet.gyro.z);
    EXPECT_EQ(0x0011, packet.accel_gyro_temp);
    EXPECT_EQ(0x0012, packet.pressure);
    EXPECT_EQ(0x0013, packet.barometer_temp);
    EXPECT_EQ(0x0014, packet.pitot);
    EXPECT_EQ(0x0015, packet.i);
    EXPECT_EQ(0x0016, packet.v);
    EXPECT_EQ(0x0017, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x0019, packet.mag.x);
    EXPECT_EQ(0x0020, packet.mag.y);
    EXPECT_EQ(0x0021, packet.mag.z);
    EXPECT_EQ(0x00000022, packet.gps.position.lat);
    EXPECT_EQ(0x00000023, packet.gps.position.lng);
    EXPECT_EQ(0x00000024, packet.gps.position.alt);
    EXPECT_EQ(0x0025, packet.gps.velocity.n);
    EXPECT_EQ(0x0026, packet.gps.velocity.e);
    EXPECT_EQ(0x0027, packet.gps.velocity.d);
    EXPECT_EQ(0x0028, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x0029, packet.gps_info.pos_err);

    memset(&packet, 0, sizeof(packet));

    result = _fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &packet);
    ASSERT_TRUE(result);

    EXPECT_EQ(0x00, packet.crc);
    EXPECT_EQ(0x0001, packet.tick);
    EXPECT_EQ(0x02, packet.sensor_update_flags);
    EXPECT_EQ(0x03, packet.cpu_load);
    EXPECT_EQ(0x0004, packet.status);
    EXPECT_EQ(0x0005, packet.accel.x);
    EXPECT_EQ(0x0006, packet.accel.y);
    EXPECT_EQ(0x0007, packet.accel.z);
    EXPECT_EQ(0x0008, packet.gyro.x);
    EXPECT_EQ(0x0009, packet.gyro.y);
    EXPECT_EQ(0x0010, packet.gyro.z);
    EXPECT_EQ(0x0011, packet.accel_gyro_temp);
    EXPECT_EQ(0x0012, packet.pressure);
    EXPECT_EQ(0x0013, packet.barometer_temp);
    EXPECT_EQ(0x0014, packet.pitot);
    EXPECT_EQ(0x0015, packet.i);
    EXPECT_EQ(0x0016, packet.v);
    EXPECT_EQ(0x0017, packet.range);
    EXPECT_EQ(0x18, packet.gpin_state);
    EXPECT_EQ(0x0019, packet.mag.x);
    EXPECT_EQ(0x0020, packet.mag.y);
    EXPECT_EQ(0x0021, packet.mag.z);
    EXPECT_EQ(0x00000022, packet.gps.position.lat);
    EXPECT_EQ(0x00000023, packet.gps.position.lng);
    EXPECT_EQ(0x00000024, packet.gps.position.alt);
    EXPECT_EQ(0x0025, packet.gps.velocity.n);
    EXPECT_EQ(0x0026, packet.gps.velocity.e);
    EXPECT_EQ(0x0027, packet.gps.velocity.d);
    EXPECT_EQ(0x0028, packet.gps_info.fix_mode_num_satellites);
    EXPECT_EQ(0x0029, packet.gps_info.pos_err);
}

/*
TEST(AHRSIO, WriteIOBoardPackets) {
    EXPECT_TRUE(false);
}
*/

TEST(AHRSMath, ProcessAccelData) {
    EXPECT_TRUE(false);
}

TEST(AHRSMath, ProcessGyroData) {
    EXPECT_TRUE(false);
}

TEST(AHRSMath, ProcessMagData) {
    EXPECT_TRUE(false);
}

TEST(AHRSMath, ProcessGPSData) {
    EXPECT_TRUE(false);
}

TEST(AHRSMath, ProcessPitotData) {
    EXPECT_TRUE(false);
}

TEST(AHRSMath, ProcessBarometerData) {
    EXPECT_TRUE(false);
}

TEST(AHRSOutput, UpdateGlobalState) {
    EXPECT_TRUE(false);
}

