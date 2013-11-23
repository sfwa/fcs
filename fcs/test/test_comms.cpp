#include "test.h"

extern "C" {
#include "config/config.h"
#include "util/util.h"
#include "comms/comms.h"
}

TEST(SerializeState, NoBuffers) {
    EXPECT_DEATH({ fcs_comms_serialize_state(NULL, NULL); }, "^Assertion");

    EXPECT_DEATH(
        { uint8_t x; fcs_comms_serialize_state(&x, NULL); }, "^Assertion");

    EXPECT_DEATH(
        { struct fcs_packet_state_t x; fcs_comms_serialize_state(NULL, &x); },
        "^Assertion"
    );
}

TEST(SerializeState, InvalidStates) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.solution_time = -1;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.lat = -95.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.lon = -190.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.alt = -1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.velocity[0] = 1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.velocity[1] = 1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.velocity[2] = 1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.wind_velocity[0] = 1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.wind_velocity[1] = 1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.wind_velocity[2] = 1000.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.attitude[0] = 400.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.attitude[1] = 100.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.attitude[2] = 200.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.angular_velocity[0] = 500.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.angular_velocity[1] = 500.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");

    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y;
        memset(&y, 0, sizeof(y));
        y.angular_velocity[2] = 500.0;
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");
}

TEST(SerializeState, AllOK) {
    struct fcs_packet_state_t state = {
        .solution_time = 200501234,
        .next_waypoint_id = { 't', 'e', 's', 't'},
        .lat = -30.0987654321,
        .lon = 145.0123456789,
        .alt = 100.5,
        .velocity = { 1.0, -2.0, 3.0 },
        .wind_velocity = { -10.0, 20.0, -30.0 },
        .attitude = { 45.6789, 80.1234, -175.0987 },
        .angular_velocity = { 0.0, 0.0, 0.0 },
        .lat_lon_uncertainty = 99.9,
        .alt_uncertainty = 0.1,
        .velocity_uncertainty = { 0.0, 2.0, 5.0 },
        .wind_velocity_uncertainty = { 0.1, 2.1, 9.9 },
        .attitude_uncertainty = { 12.34, 2.345, 3.456 },
        .angular_velocity_uncertainty = { 0.0, 0.5, 1.0 },
        .mode_indicator = 'A',
        .flags = { 't', 'e', 's', 't' }
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
        "A,test*73\r\n",
        (char*)result
    );
}

TEST(DeserializeState, AllOK) {
    struct fcs_packet_state_t state;
    uint8_t input[] =
        "$PSFWAS,200501234,test,-30.0987654,145.0123457,100.50,"
        "1.00,-2.00,3.00,-10.00,20.00,-30.00,"
        "45.68,80.12,-175.10,.00,.00,.00,"
        "100,.1,0,2,5,0,2,10,12,2,3,0,1,1,"
        "A,test*73\r\n";
    enum fcs_deserialization_result_t status;

    status = fcs_comms_deserialize_state(&state, input, sizeof(input) - 1);
    EXPECT_EQ(FCS_DESERIALIZATION_OK, status);

    EXPECT_EQ(200501234, state.solution_time);
}
