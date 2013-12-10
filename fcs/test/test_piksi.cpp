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

/* Data structures from piksi/piksi.c */
enum fcs_piksi_packet_type_t {
    FCS_PIKSI_TRACKING_STATE = 0x22u,

    FCS_PIKSI_SOLUTION = 0x50u,
    FCS_PIKSI_DOPS = 0x51u,
    FCS_PIKSI_OBSERVATIONS = 0x52u,
    FCS_PIKSI_OBSERVATION_HDR = 0x53u
};

enum fcs_piksi_packet_parse_state_t {
    FCS_PIKSI_EMPTY,
    FCS_PIKSI_GOT_BE,
    FCS_PIKSI_GOT_EF,
    FCS_PIKSI_GOT_TYPE,
    FCS_PIKSI_GOT_LEN,
    FCS_PIKSI_GOT_PAYLOAD,
    FCS_PIKSI_GOT_CRC16_0,
    FCS_PIKSI_DONE,
    FCS_PIKSI_INVALID
};

struct fcs_piksi_packet_t {
    uint8_t type;
    uint8_t len;
    uint8_t payload[255];
    uint16_t crc16;

    enum fcs_piksi_packet_parse_state_t _state;
    uint8_t _payload_byte_count;
};


/* Prototypes for private test functions */

/* from drivers/stream.c */
size_t _fcs_stream_write_to_rx_buffer(uint8_t buffer_idx, const uint8_t *val,
size_t len);
size_t _fcs_stream_read_from_tx_buffer(uint8_t buffer_idx, uint8_t *val,
size_t len);

/* from piksi/piksi.c */
enum fcs_piksi_packet_parse_state_t _fcs_piksi_read_packet(
enum fcs_stream_device_t dev, struct fcs_piksi_packet_t *packet);
}

TEST(Piksi, Initialisation) {
    fcs_piksi_init();
}

/* TODO: get some SBP packet samples for test cases. */
