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

TEST(SerializeState, InvalidState) {
    EXPECT_DEATH({
        uint8_t x[256];
        struct fcs_packet_state_t y = {};
        fcs_comms_serialize_state(x, &y);
    }, "^Assertion");
}
