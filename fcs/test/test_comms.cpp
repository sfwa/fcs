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
#include "config/config.h"
#include "util/util.h"
#include "comms/comms.h"
#include "drivers/stream.h"

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

TEST(CommsSerializeState, NoBuffers) {
    EXPECT_DEATH(
        { fcs_comms_serialize_state(NULL, NULL); }, "Assertion.*failed");

    EXPECT_DEATH(
        { uint8_t x; fcs_comms_serialize_state(&x, NULL); },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        { struct fcs_packet_state_t x; fcs_comms_serialize_state(NULL, &x); },
        "Assertion.*failed"
    );
}

TEST(CommsSerializeState, InvalidSolutionTime) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.solution_time = -1;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidLat) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.lat = -95.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidLon) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.lon = -190.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidAlt) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.alt = -1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidVelocityN) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.velocity[0] = 1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidVelocityE) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.velocity[1] = 1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidVelocityD) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.velocity[2] = 1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidWindN) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.wind_velocity[0] = 1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidWindE) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.wind_velocity[1] = 1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidWindD) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.wind_velocity[2] = 1000.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidYaw) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.yaw = 400.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidPitch) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.pitch = 100.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidRoll) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.roll = 200.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidAngularVelocityX) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.angular_velocity[0] = 500.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidAngularVelocityY) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.angular_velocity[1] = 500.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, InvalidAngularVelocityZ) {
    uint8_t x[256];
    struct fcs_packet_state_t y;
    size_t len;
    memset(&y, 0, sizeof(y));
    y.angular_velocity[2] = 500.0;
    y.mode_indicator = 'N';
    len = fcs_comms_serialize_state(x, &y);
    EXPECT_EQ('N', x[len - 23]);
}

TEST(CommsSerializeState, AllOK) {
    struct fcs_packet_state_t state = {
        .solution_time = 200501234,
        .next_waypoint_id = { 't', 'e', 's', 't'},
        .lat = -30.0987654321,
        .lon = 145.0123456789,
        .alt = 100.5,
        .velocity = { 1.0, -2.0, 3.0 },
        .wind_velocity = { -10.0, 20.0, -30.0 },
        .yaw = 45.6789,
        .pitch = 80.1234,
        .roll = -175.0987,
        .angular_velocity = { 0.0, 0.0, 0.0 },
        .lat_lon_uncertainty = 99.9,
        .alt_uncertainty = 0.1,
        .velocity_uncertainty = { 0.0, 2.0, 5.0 },
        .wind_velocity_uncertainty = { 0.1, 2.1, 9.9 },
        .yaw_uncertainty = 12.34,
        .pitch_uncertainty = 2.345,
        .roll_uncertainty = 3.456,
        .angular_velocity_uncertainty = { 0.0, 0.5, 1.0 },
        .mode_indicator = 'A',
        .flags = { 't', 'e', 's', 't', '1', '2', '3' }
    };
    uint8_t result[256];
    size_t result_len;

    result_len = fcs_comms_serialize_state(result, &state);
    ASSERT_TRUE(result_len > 0);
    ASSERT_TRUE(result_len < 256u);

    result[result_len] = 0;

    EXPECT_STREQ(
        "$PSFWAS,200501234,test,-30.0987654,145.0123457,100.50,"
        "1.00,-2.00,3.00,-10.00,20.00,-30.00,"
        "45.68,80.12,-175.10,.00,.00,.00,"
        "100,.1,0,2,5,0,2,10,12,2,3,0,1,1,"
        "A,test123,BEC3EDED*18\r\n",
        (char*)result
    );
}

TEST(CommsDeserializeState, AllOK) {
    struct fcs_packet_state_t state;
    uint8_t input[] =
        "$PSFWAS,200501234,test,-30.0987654,145.0123457,100.50,"
        "1.00,-2.00,3.00,-10.00,20.00,-30.00,"
        "45.68,80.12,-175.10,.00,.00,.00,"
        "100,.1,0,2,5,0,2,10,12,2,3,0,1,1,"
        "A,test123,BEC3EDED*18\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_state(&state, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_OK, status);

    EXPECT_EQ(200501234, state.solution_time);
}

TEST(CommsDeserializeState, InvalidChecksum) {
    struct fcs_packet_state_t state;
    uint8_t input[] =
        "$PSFWAS,200501234,test,-30.0987654,145.0123457,100.50,"
        "1.00,-2.00,3.00,-10.00,20.00,-30.00,"
        "45.68,80.12,-175.10,.00,.00,.00,"
        "100,.1,0,2,5,0,2,10,12,2,3,0,1,1,"
        "A,test123,BEC3EDED*00\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_state(&state, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeState, InvalidTalker) {
    struct fcs_packet_state_t state;
    uint8_t input[] =
        "$PSFWAs,200501234,test,-30.0987654,145.0123457,100.50,"
        "1.00,-2.00,3.00,-10.00,20.00,-30.00,"
        "45.68,80.12,-175.10,.00,.00,.00,"
        "100,.1,0,2,5,0,2,10,12,2,3,0,1,1,"
        "A,test123,BEC3EDED*53\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_state(&state, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeState, InvalidFieldData) {
    struct fcs_packet_state_t state;
    uint8_t input[] =
        "$PSFWAS,200501234,test,-3q.0987654,145.0123457,100.50,"
        "1.00,-2.00,3.00,-10.00,20.00,-30.00,"
        "45.68,80.12,-175.10,.00,.00,.00,"
        "100,.1,0,2,5,0,2,10,12,2,3,0,1,1,"
        "A,test123,BEC3EDED*32\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_state(&state, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsSerializeWaypoint, AllOK) {
    struct fcs_packet_waypoint_t waypoint = {
        .waypoint_id = { 't', 'e', 's', 't'},
        .waypoint_role = 'q',
        .flags = { 't', 'e', 's' },
        .target_lat = -30.0987654321,
        .target_lon = 145.0123456789,
        .target_alt = 100.5,
        .target_yaw = 45.6789,
        .target_pitch = 80.1234,
        .target_roll = -175.0987,
        .target_airspeed = 54.321
    };
    uint8_t result[256];
    size_t result_len;

    result_len = fcs_comms_serialize_waypoint(result, &waypoint);
    ASSERT_TRUE(result_len > 0);
    ASSERT_TRUE(result_len <= 93u);

    result[result_len] = 0;

    EXPECT_STREQ(
        "$PSFWAP,test,q,tes,-30.0987654,145.0123457,100.50,"
        "45.68,80.12,-175.10,54.32,7808F414*42\r\n",
        (char*)result
    );
}

TEST(CommsSerializeWaypoint, InvalidLat) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_lat = -135.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsSerializeWaypoint, InvalidLon) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_lon = 235.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsSerializeWaypoint, InvalidAlt) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_alt = -3500.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsSerializeWaypoint, InvalidYaw) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_yaw = -5.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsSerializeWaypoint, InvalidPitch) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_pitch = -135.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsSerializeWaypoint, InvalidRoll) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_roll = -235.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsSerializeWaypoint, InvalidAirspeed) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_waypoint_t y;
        memset(&y, 0, sizeof(y));
        y.target_airspeed = 3000.0;
        fcs_comms_serialize_waypoint(x, &y);
    }, "Assertion.*failed");
}

TEST(CommsDeserializeWaypoint, AllOK) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,tes,-30.0987654,145.0123457,100.50,"
        "45.68,80.12,-175.10,54.32,7808F414*42\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_OK, status);

    EXPECT_NEAR(-30.0987654, waypoint.target_lat, 1e-9);
    EXPECT_NEAR(145.0123457, waypoint.target_lon, 1e-9);
    EXPECT_NEAR(100.50, waypoint.target_alt, 1e-9);
    EXPECT_NEAR(45.68, waypoint.target_yaw, 1e-9);
    EXPECT_NEAR(80.12, waypoint.target_pitch, 1e-9);
    EXPECT_NEAR(-175.10, waypoint.target_roll, 1e-9);
    EXPECT_NEAR(54.32, waypoint.target_airspeed, 1e-9);
}

TEST(CommsDeserializeWaypoint, InvalidChecksum) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "45.68,80.12,-175.10,54.32,FFAC0890*00\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidTalker) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAS,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "45.68,80.12,-175.10,54.32,6C29D7A6*52\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidFieldData) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "45.q8,80.12,-175.10,54.32,DD3C2C49*60\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidLat) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-92.0987654,145.0123457,100.50,"
        "45.68,80.12,-175.10,54.32,6C29D7A6*59\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidLon) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,185.0123457,100.50,"
        "45.68,80.12,-175.10,54.32,DD3C2C49*2B\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidAlt) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100000.50,"
        "45.68,80.12,-175.10,54.32,F9DA84BA*6E\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidYaw) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "-45.68,80.12,-175.10,54.32,45209290*07\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidPitch) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "45.68,90.12,-175.10,54.32,9ACB33D6*21\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidRoll) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "45.68,80.12,-185.10,54.32,8D3897E1*29\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeWaypoint, InvalidAirspeed) {
    struct fcs_packet_waypoint_t waypoint;
    uint8_t input[] =
        "$PSFWAP,test,q,xyz,-30.0987654,145.0123457,100.50,"
        "45.68,80.12,-175.10,-54.32,D2E7E8A7*09\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_waypoint(&waypoint, input,
                                            sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, AllOK) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
        "1034.68,BD739552*0A\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_OK, status);

    EXPECT_EQ(12345678, gcs.solution_time);
    EXPECT_NEAR(-30.0987654, gcs.lat, 1e-9);
    EXPECT_NEAR(145.0123457, gcs.lon, 1e-9);
    EXPECT_NEAR(100.50, gcs.alt, 1e-9);
    EXPECT_NEAR(1034.68, gcs.pressure, 1e-9);
}

TEST(CommsDeserializeGCS, InvalidChecksum) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
        "1034.68,BD739552*00\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, InvalidTalker) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAg,12345678,test,-30.0987654,145.0123457,100.50,"
        "1034.68,BD739552*35\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, InvalidSolutionTime) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,-50,test,-30.0987654,145.0123457,100.50,"
        "1034.68,BD739552*35\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, InvalidLat) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,12345678,test,130.0987654,145.0123457,100.50,"
        "1034.68,764B452A*0A\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, InvalidLon) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,12345678,test,-30.0987654,245.0123457,100.50,"
        "1034.68,EE7B891*16\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, InvalidAlt) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,12345678,test,-30.0987654,145.0123457,-600,"
        "1034.68,F5C38FE*14\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsDeserializeGCS, InvalidPressure) {
    struct fcs_packet_gcs_t gcs;
    uint8_t input[] =
        "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
        "50.00,7481820D*7A\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_gcs(&gcs, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_ERROR, status);
}

TEST(CommsIO, ReadPacket) {
    bool result;
    size_t len;
    uint8_t buf[256];
    uint8_t s[] = "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
                  "1034.68,BD739552*15\r\n";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, s, 74);
    ASSERT_EQ(74, len);

    /* Read back and confirm results */
    len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, buf);
    ASSERT_EQ(74, len);

    buf[len] = 0;
    ASSERT_STREQ((char*)s, (char*)buf);
}

TEST(CommsIO, ReadPacketPartial) {
    bool result;
    size_t len;
    uint8_t buf[256];
    uint8_t s[] = "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
                  "1034.68,BD739552*15\r\n";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write a partial packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, s, 30);
    ASSERT_EQ(30, len);

    /* Read back and confirm there's no packet */
    len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, buf);
    ASSERT_EQ(0, len);

    /* Write the remainder */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, &s[30], 44);
    ASSERT_EQ(44, len);

    /* Read the full packet */
    len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, buf);
    ASSERT_EQ(74, len);

    buf[len] = 0;
    ASSERT_STREQ((char*)s, (char*)buf);
}

TEST(CommsIO, ReadPacketAfterGarbage) {
    bool result;
    size_t len;
    uint8_t buf[256];
    uint8_t s[] = "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
                  "1034.68,BD739552*15\r\n";
    uint8_t garbage[] = "garbage\r\n";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write some garbage */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, garbage, 9);
    ASSERT_EQ(9, len);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, s, 74);
    ASSERT_EQ(74, len);

    /* Read back and confirm results */
    len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, buf);
    ASSERT_EQ(74, len);

    buf[len] = 0;
    ASSERT_STREQ((char*)s, (char*)buf);
}

TEST(CommsIO, ReadPacketAfterPacket) {
    bool result;
    size_t len;
    uint8_t buf[256];
    uint8_t s[] = "$PSFWAG,12345678,test,-30.0987654,145.0123457,100.50,"
                  "1034.68,BD739552*15\r\n";

    /* Reset stream state */
    fcs_stream_open(FCS_STREAM_UART_INT0);

    /* Write a full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, s, 74);
    ASSERT_EQ(74, len);

    /* Write another full packet */
    len = _fcs_stream_write_to_rx_buffer(FCS_STREAM_UART_EXT0, s, 74);
    ASSERT_EQ(74, len);

    /* Read first packet back and confirm results */
    len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, buf);
    ASSERT_EQ(74, len);

    buf[len] = 0;
    ASSERT_STREQ((char*)s, (char*)buf);

    memset(buf, 0, sizeof(buf));

    /* Read second packet back and confirm results */
    len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, buf);
    ASSERT_EQ(74, len);

    buf[len] = 0;
    ASSERT_STREQ((char*)s, (char*)buf);
}

TEST(CommsLog, InitPacket) {
    struct fcs_packet_log_t packet;
    fcs_comms_init_log(&packet, 1234u);
    EXPECT_EQ(FCS_PACKET_LOG_TYPE, packet.buf[0]);
    EXPECT_EQ(0, packet.buf[1u]);
    EXPECT_EQ(0, packet.buf[2u]);
    EXPECT_EQ(1234u, (packet.buf[4u] << 8u) + packet.buf[3u]);
    EXPECT_EQ(5u, packet.len);
}

TEST(CommsLog, InitPacketMissing) {
    EXPECT_DEATH({ fcs_comms_init_log(NULL, 1234u); }, "Assertion.*failed");
}

TEST(CommsLog, AddSensorSingle) {
    struct fcs_packet_log_t packet;
    fcs_comms_init_log(&packet, 1u);

    int16_t value = 1234;
    bool result;
    result = fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT,
                                            1u, (uint8_t*)&value,
                                            sizeof(value));
    EXPECT_EQ(true, result);
    EXPECT_EQ(0x02u, packet.buf[5u]);
    EXPECT_EQ(0x23u, packet.buf[6u]);
    EXPECT_EQ(1234, (int16_t)((packet.buf[8u] << 8u) + packet.buf[7u]));
}

TEST(CommsLog, AddSensorMultiple) {
    struct fcs_packet_log_t packet;
    fcs_comms_init_log(&packet, 1u);

    int16_t value1 = 1234, value2[3] = { 0, 1, -1 };
    bool result;
    result = fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT,
                                            1u, (uint8_t*)&value1,
                                            sizeof(value1));
    ASSERT_EQ(true, result);

    result = fcs_comms_add_log_sensor_value(&packet,
                                            FCS_SENSOR_TYPE_ACCELEROMETER,
                                            0, (uint8_t*)&value2,
                                            sizeof(value2));
    ASSERT_EQ(true, result);

    EXPECT_EQ(0x02u, packet.buf[5u]);
    EXPECT_EQ(0x23u, packet.buf[6u]);
    EXPECT_EQ(1234, (int16_t)((packet.buf[8u] << 8u) + packet.buf[7u]));

    EXPECT_EQ(0x06u, packet.buf[9u]);
    EXPECT_EQ(0, packet.buf[10u]);
    EXPECT_EQ(0, (int16_t)((packet.buf[12u] << 8u) + packet.buf[11u]));
    EXPECT_EQ(1, (int16_t)((packet.buf[14u] << 8u) + packet.buf[13u]));
    EXPECT_EQ(-1, (int16_t)((packet.buf[16u] << 8u) + packet.buf[15u]));
}

TEST(CommsLog, AddSensorTooMany) {
    struct fcs_packet_log_t packet;
    fcs_comms_init_log(&packet, 1u);

    int16_t value[7];
    bool result;
    uint8_t i;

    for (i = 0; i < 15; i++) {
        result = fcs_comms_add_log_sensor_value(&packet,
                                            FCS_SENSOR_TYPE_PITOT, 1u,
                                            (uint8_t*)&value, sizeof(value));
        EXPECT_EQ(true, result);
    }

    result = fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT,
                                            1u, (uint8_t*)&value,
                                            sizeof(value));
    EXPECT_EQ(false, result);
}

TEST(CommsLog, AddSensorInvalidType) {
    EXPECT_DEATH({
        struct fcs_packet_log_t packet;
        fcs_comms_init_log(&packet, 1u);

        int16_t value = 1234;
        fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_LAST, 0,
                                       (uint8_t*)&value, sizeof(value));
    }, "Assertion.*failed");
}

TEST(CommsLog, AddSensorTooLong) {
    EXPECT_DEATH({
        struct fcs_packet_log_t packet;
        fcs_comms_init_log(&packet, 1u);

        int16_t value[30];
        fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT, 0,
                                       (uint8_t*)&value, sizeof(value));
    }, "Assertion.*failed");
}

TEST(CommsLog, AddSensorInvalidID) {
    EXPECT_DEATH({
        struct fcs_packet_log_t packet;
        fcs_comms_init_log(&packet, 1u);

        int16_t value = 1234;
        fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT, 15,
                                       (uint8_t*)&value, sizeof(value));
    }, "Assertion.*failed");
}

TEST(CommsLog, AddSensorNoLength) {
    EXPECT_DEATH({
        struct fcs_packet_log_t packet;
        fcs_comms_init_log(&packet, 1u);

        int16_t value = 1234;
        fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT, 0,
                                       (uint8_t*)&value, 0);
    }, "Assertion.*failed");
}

TEST(CommsLog, AddSensorNoData) {
    EXPECT_DEATH({
        struct fcs_packet_log_t packet;
        fcs_comms_init_log(&packet, 1u);

        fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT, 0,
                                      NULL, 2u);
    }, "Assertion.*failed");
}

TEST(CommsLog, SerializePacket) {
    struct fcs_packet_log_t packet;
    fcs_comms_init_log(&packet, 1u);

    int16_t value1 = 1234, value2[3] = { 0, 1, -1 };
    bool result;
    result = fcs_comms_add_log_sensor_value(&packet, FCS_SENSOR_TYPE_PITOT,
                                            1u, (uint8_t*)&value1,
                                            sizeof(value1));
    ASSERT_EQ(true, result);

    result = fcs_comms_add_log_sensor_value(&packet,
                                            FCS_SENSOR_TYPE_ACCELEROMETER,
                                            0, (uint8_t*)&value2,
                                            sizeof(value2));
    ASSERT_EQ(true, result);

    uint8_t out_buf[256u];
    size_t len;
    len = fcs_comms_serialize_log(out_buf, sizeof(out_buf), &packet);
    EXPECT_EQ(21u, len);
    EXPECT_EQ(0, out_buf[0]);
    EXPECT_EQ(0, out_buf[len + 1u]);
    EXPECT_STREQ(
        "\x2\x1\x1\x2\x1\x6\x2#\xD2\x4\x6\x1\x1\x2\x1\xD3\xFF\xFF\xA8",
        (char*)&out_buf[1u]);
}

