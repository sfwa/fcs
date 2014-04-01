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
#include "util/util.h"
#include "util/3dmath.h"
#include "ukf/cukf.h"
#include "TRICAL/TRICAL.h"
#include "ahrs/measurement.h"
#include "ahrs/ahrs.h"
#include "drivers/stream.h"

/* Prototypes for private test functions */

/* from drivers/stream.c */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, uint8_t *val,
size_t len);
}

TEST(AHRS, Initialisation) {
    fcs_ahrs_init();
}

TEST(MeasurementLog, InitPacket) {
    struct fcs_measurement_log_t packet;
    fcs_measurement_log_init(&packet, 1234u);
    EXPECT_EQ(FCS_MEASUREMENT_LOG_TYPE, packet.data[0]);
    EXPECT_EQ(0, packet.data[1]);
    EXPECT_EQ(0, packet.data[2]);
    EXPECT_EQ(1234u, (packet.data[4] << 8u) + packet.data[3]);
    EXPECT_EQ(5u, packet.length);
}

TEST(MeasurementLog, InitPacketMissing) {
    EXPECT_DEATH({
        fcs_measurement_log_init(NULL, 1234u);
    }, "Assertion.*failed");
}

TEST(MeasurementLog, AddSensorSingle) {
    struct fcs_measurement_log_t packet;
    fcs_measurement_log_init(&packet, 1u);

    bool result;
    struct fcs_measurement_t measurement;

    measurement.header = sizeof(int16_t);
    fcs_measurement_set_sensor(&measurement, 2u, FCS_MEASUREMENT_TYPE_PITOT);
    measurement.data.i16[0] = 1234;

    result = fcs_measurement_log_add(&packet, &measurement);

    EXPECT_EQ(true, result);
    EXPECT_EQ(0x02u, packet.data[5]);
    EXPECT_EQ(0x44u, packet.data[6]);
    EXPECT_EQ(1234, (int16_t)((packet.data[8] << 8u) + packet.data[7]));
}

TEST(MeasurementLog, AddSensorMultiple) {
    struct fcs_measurement_log_t packet;
    fcs_measurement_log_init(&packet, 1u);

    bool result;
    struct fcs_measurement_t measurement;

    fcs_measurement_set_header(&measurement, 16u, 1u);
    fcs_measurement_set_sensor(&measurement, 2u, FCS_MEASUREMENT_TYPE_PITOT);
    measurement.data.i16[0] = 1234;
    result = fcs_measurement_log_add(&packet, &measurement);

    ASSERT_EQ(true, result);

    fcs_measurement_set_header(&measurement, 16u, 3u);
    fcs_measurement_set_sensor(&measurement, 0,
                               FCS_MEASUREMENT_TYPE_ACCELEROMETER);
    measurement.data.i16[0] = 0;
    measurement.data.i16[1] = 1;
    measurement.data.i16[2] = -1;
    result = fcs_measurement_log_add(&packet, &measurement);

    EXPECT_EQ(true, result);

    EXPECT_EQ(0x18u, packet.data[5]);
    EXPECT_EQ(0x44u, packet.data[6]);
    EXPECT_EQ(1234, (int16_t)((packet.data[8] << 8u) + packet.data[7]));

    EXPECT_EQ(0x1Au, packet.data[9]);
    EXPECT_EQ(0x01u, packet.data[10]);
    EXPECT_EQ(0, (int16_t)((packet.data[12] << 8u) + packet.data[11]));
    EXPECT_EQ(1, (int16_t)((packet.data[14] << 8u) + packet.data[13]));
    EXPECT_EQ(-1, (int16_t)((packet.data[16] << 8u) + packet.data[15]));
}

TEST(MeasurementLog, AddSensorTooMany) {
    struct fcs_measurement_log_t packet;
    fcs_measurement_log_init(&packet, 1u);

    bool result;
    struct fcs_measurement_t measurement;

    fcs_measurement_set_header(&measurement, 16u, 7u);
    fcs_measurement_set_sensor(&measurement, 2u, FCS_MEASUREMENT_TYPE_PITOT);

    uint8_t i;
    for (i = 0; i < 15u; i++) {
        result = fcs_measurement_log_add(&packet, &measurement);
        EXPECT_EQ(true, result);
    }

    result = fcs_measurement_log_add(&packet, &measurement);
    EXPECT_EQ(false, result);
}

TEST(MeasurementLog, AddSensorInvalidType) {
    EXPECT_DEATH({
        struct fcs_measurement_log_t packet;
        fcs_measurement_log_init(&packet, 1u);

        struct fcs_measurement_t measurement;
        fcs_measurement_set_header(&measurement, 16u, 1u);
        measurement.sensor = FCS_MEASUREMENT_TYPE_LAST;
        measurement.data.i16[0] = 1234;
        fcs_measurement_log_add(&packet, &measurement);
    }, "Assertion.*failed");
}

TEST(MeasurementLog, AddSensorTooLong) {
    EXPECT_DEATH({
        struct fcs_measurement_log_t packet;
        fcs_measurement_log_init(&packet, 1u);

        struct fcs_measurement_t measurement;
        measurement.header = 0x3Fu;
        measurement.sensor = 0x01u;
        measurement.data.i16[0] = 1234;
        fcs_measurement_log_add(&packet, &measurement);
    }, "Assertion.*failed");
}

TEST(MeasurementLog, AddSensorInvalidID) {
    EXPECT_DEATH({
        struct fcs_measurement_log_t packet;
        fcs_measurement_log_init(&packet, 1u);

        struct fcs_measurement_t measurement;
        fcs_measurement_set_header(&measurement, 16u, 1u);
        measurement.sensor = 0xE1u;
        measurement.data.i16[0] = 1234;
        fcs_measurement_log_add(&packet, &measurement);
    }, "Assertion.*failed");
}

TEST(MeasurementLog, SerializePacket) {
    struct fcs_measurement_log_t packet;
    fcs_measurement_log_init(&packet, 1u);

    bool result;
    struct fcs_measurement_t measurement;

    fcs_measurement_set_header(&measurement, 16u, 1u);
    fcs_measurement_set_sensor(&measurement, 2u, FCS_MEASUREMENT_TYPE_PITOT);
    measurement.data.i16[0] = 1234;
    result = fcs_measurement_log_add(&packet, &measurement);

    ASSERT_EQ(true, result);

    fcs_measurement_set_header(&measurement, 16u, 3u);
    fcs_measurement_set_sensor(&measurement, 0,
                               FCS_MEASUREMENT_TYPE_ACCELEROMETER);
    measurement.data.i16[0] = 0;
    measurement.data.i16[1] = 1;
    measurement.data.i16[2] = -1;
    result = fcs_measurement_log_add(&packet, &measurement);

    ASSERT_EQ(true, result);

    uint8_t out_buf[256];
    size_t len;
    len = fcs_measurement_log_serialize(out_buf, sizeof(out_buf), &packet);
    EXPECT_EQ(21u, len);
    EXPECT_EQ(0, out_buf[0]);
    EXPECT_EQ(0, out_buf[len + 1u]);
    EXPECT_STREQ(
        "\x2\x1\x1\x2\x1\a\x18" "D\xD2\x4\x1A\x1\x1\x2\x1\xAA\xFF\xFF\xCA",
        (char*)&out_buf[1]);
}
