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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../drivers/stream.h"
#include "../stats/stats.h"
#include "piksi.h"

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

#define FCS_PIKSI_MIN_PACKET_SIZE 6u

static uint32_t tick, piksi_last_packet;
static struct fcs_piksi_packet_t piksi_packet;

struct fcs_piksi_solution_t fcs_global_piksi_solution;

enum fcs_piksi_packet_parse_state_t _fcs_piksi_read_packet(
enum fcs_stream_device_t dev, struct fcs_piksi_packet_t *packet);

void fcs_piksi_init(void) {
    /* Open the Piksi stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT1, 115200u) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT1) == FCS_STREAM_OK);

    /* Make sure the solution isn't full of temptingly nearly-valid zeros */
    memset(&fcs_global_piksi_solution, 0xFFu,
           sizeof(fcs_global_piksi_solution));
    fcs_global_piksi_solution.updated = false;
}

void fcs_piksi_tick(void) {
    /* Check for packets */
    enum fcs_piksi_packet_parse_state_t state;
    state = _fcs_piksi_read_packet(FCS_STREAM_UART_EXT1, &piksi_packet);

    if (state == FCS_PIKSI_DONE) {
        switch (piksi_packet.type) {
            case FCS_PIKSI_TRACKING_STATE:
                fcs_global_counters.piksi_packet_rx++;
                break;
            case FCS_PIKSI_SOLUTION:
                fcs_global_piksi_solution.updated = true;

                /* TODO: extract solution */
                fcs_global_piksi_solution.lat = 0.0;
                fcs_global_piksi_solution.lon = 0.0;
                fcs_global_piksi_solution.alt = 0.0;
                fcs_global_piksi_solution.velocity[0] = 0.0;
                fcs_global_piksi_solution.velocity[1] = 0.0;
                fcs_global_piksi_solution.velocity[2] = 0.0;
                fcs_global_piksi_solution.h_covariance = 1.0;
                fcs_global_piksi_solution.v_covariance = 100.0;

                /* Update counters */
                piksi_last_packet = tick;
                fcs_global_counters.piksi_packet_rx++;
                break;
            case FCS_PIKSI_DOPS:
                fcs_global_counters.piksi_packet_rx++;
                break;
            case FCS_PIKSI_OBSERVATIONS:
                fcs_global_counters.piksi_packet_rx++;
                break;
            case FCS_PIKSI_OBSERVATION_HDR:
                fcs_global_counters.piksi_packet_rx++;
                break;
            default:
                fcs_global_counters.piksi_packet_rx_err++;
                break;
        }
    }

    /* TODO: Restart Piksi if it's been too long since the last solution */

    /*
    Increment tick, but wrap to 0 at a (decimal) round number in order to keep
    the output packet rate steady.
    */
    tick++;
    if (tick == 4000000000u) {
        tick = 0;
    }
}

/*
Read a Piksi packet from `dev` incrementally. Returns FCS_PIKSI_DONE once the
parse is complete and the packet is valid, or FCS_PIKSI_INVALID if the packet
is not valid.

Unlike the AHRS I/O board packet or comms message read code, this uses
persistent state for each packet and reads/deserializes/validates the packet
incrementally. The difference in approach is because the Piksi link is much
slower, and arrival times of packets much further apart. Also, the SwiftNav
Binary Protocol doesn't have an unambiguous interpretation (the frame start
bytes are permitted in data), so we need to be somewhat tolerant of that.
*/
enum fcs_piksi_packet_parse_state_t _fcs_piksi_read_packet(
enum fcs_stream_device_t dev, struct fcs_piksi_packet_t *packet) {
    assert(packet);
    assert(packet->_state != FCS_PIKSI_DONE &&
           packet->_state != FCS_PIKSI_INVALID);

    uint32_t nbytes;

    /*
    If the packet processing hasn't started, find the next 0xBE.

    Initialize the packet members to appropriate values once we start, in
    order to minimize the chance of any data being mistaken for valid.
    */
    if (packet->_state == FCS_PIKSI_EMPTY) {
        nbytes = fcs_stream_skip_until_after(dev, 0xBEu);
        if (nbytes) {
            packet->_state = FCS_PIKSI_GOT_BE;
            packet->type = 0;
            packet->len = 0;
            memset(packet->payload, 0xFFu, sizeof(packet->payload));
            packet->crc16 = 0xFFFFu;
            packet->_payload_byte_count = 0;
        }
    }

    /*
    If we found an 0xBE, the next byte should be an 0xEF. If not, the packet
    is invalid.
    */
    if (packet->_state == FCS_PIKSI_GOT_BE) {
        uint8_t ef;
        nbytes = fcs_stream_read(dev, &ef, 1u);
        if (nbytes && ef == 0xEFu) {
            packet->_state = FCS_PIKSI_GOT_EF;
        } else if (nbytes && ef != 0xEFu) {
            packet->_state = FCS_PIKSI_INVALID;
        }
    }

    /*
    If we found an 0xEF, the next byte is the packet type. If the packet type
    is 0, it's invalid (probably?).
    */
    if (packet->_state == FCS_PIKSI_GOT_EF) {
        nbytes = fcs_stream_read(dev, &packet->type, 1u);
        if (nbytes && packet->type) {
            packet->_state = FCS_PIKSI_GOT_TYPE;
        } else if (nbytes && !packet->type) {
            packet->_state = FCS_PIKSI_INVALID;
        }
    }

    /*
    If we found a valid type, the next byte is the length. If the length is 0,
    the packet is invalid.
    */
    if (packet->_state == FCS_PIKSI_GOT_TYPE) {
        nbytes = fcs_stream_read(dev, &packet->len, 1u);
        if (nbytes && packet->len) {
            packet->_state = FCS_PIKSI_GOT_LEN;
        } else if (nbytes && !packet->len) {
            packet->_state = FCS_PIKSI_INVALID;
        }
    }

    /*
    If we found a packet length, start reading the payload. The number of
    payload bytes read so far is stored in _payload_byte_count; once that is
    equal to len, the payload read is complete.
    */
    if (packet->_state == FCS_PIKSI_GOT_LEN) {
        nbytes = fcs_stream_read(
            dev,
            &packet->payload[packet->_payload_byte_count],
            packet->len - packet->_payload_byte_count
        );
        assert(nbytes + packet->_payload_byte_count <= packet->len);

        packet->_payload_byte_count += nbytes;
        if (packet->_payload_byte_count == packet->len) {
            packet->_state = FCS_PIKSI_GOT_PAYLOAD;
        }
    }

    /*
    If the payload read is complete, we're now looking for the first CRC16
    byte.
    */
    if (packet->_state == FCS_PIKSI_GOT_PAYLOAD) {
        uint8_t crc;
        nbytes = fcs_stream_read(dev, &crc, 1u);
        if (nbytes) {
            packet->crc16 = crc;
            packet->_state = FCS_PIKSI_GOT_CRC16_0;
        }
    }

    /*
    If the first CRC byte read is complete, try to read the second, then
    validate the CRC16 against the message type, length and payload. If it's
    not valid, update the packet accordingly; otherwise, we're done.
    */
    if (packet->_state == FCS_PIKSI_GOT_PAYLOAD) {
        uint8_t crc;
        nbytes = fcs_stream_read(dev, &crc, 1u);
        if (nbytes) {
            packet->crc16 |= (crc << 8u);

            /* These are checked above, but just to be sure... */
            assert(packet->type);
            assert(packet->len);
            assert(packet->len == packet->_payload_byte_count);

            /*
            Calculate the SBP CRC16 -- initial value is 0 for some reaosn
            */
            if (packet->crc16 == fcs_crc16_sbp(&packet->type,
                                               packet->len + 2u, 0)) {
                packet->_state = FCS_PIKSI_DONE;
            } else {
                packet->_state = FCS_PIKSI_INVALID;
            }
        }
    }

    return packet->_state;
}
