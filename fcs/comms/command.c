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
#include <math.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../TRICAL/TRICAL.h"
#include "../ukf/cukf.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "../stats/stats.h"
#include "../drivers/peripheral.h"
#include "comms.h"


#define MODE_CMD "set mode to "


enum fcs_deserialization_result_t fcs_comms_deserialize_command(
const uint8_t *packet, size_t packet_length) {
    assert(packet);
    assert(
        FCS_COMMS_MIN_PACKET_SIZE <= packet_length && packet_length < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'C' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(packet, "$PSFWAC,", 8u) != 0) {
        return FCS_DESERIALIZATION_ERROR;
    }

    const uint8_t *command;
    size_t command_length;

    /*
    Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values.

    Stop once we've processed all the fields, hit the end of the buffer, or
    hit the checksum marker ('*').
    */
    for (field = 0; field < 3u && idx < packet_length && packet[idx] != '*';
         field++) {
        size_t field_start = idx;
        for (; idx < packet_length; idx++) {
            if (packet[idx] == '*') {
                /* End of message -- break without updating checksum value */
                idx++;
                break;
            }

            /* Update checksum value with current byte */
            checksum ^= packet[idx];
            if (packet[idx] == ',') {
                /* End of field -- stop looping and parse it */
                idx++;
                break;
            }
        }

        /* Work out the current field length */
        size_t field_len = idx - field_start - 1u;
        if (field_len == 0) {
            continue;
        }
        assert(field_len < 256u);

        /* Handle the field data based on the current field index */
        switch (field) {
            case 0:
                /* TODO: Packet time -- ignore for now */
                if (field_len > 9u) {
                    return FCS_DESERIALIZATION_ERROR;
                }
                break;
            case 1u:
                /* Command string, up to 192 bytes long */
                if (field_len > 192u) {
                    return FCS_DESERIALIZATION_ERROR;
                }
                command = &packet[field_start];
                command_length = field_len;
                break;
            case 2u:
                if (field_len != 8u) {
                    return FCS_DESERIALIZATION_ERROR;
                }
                /* Read the message CRC and compare with the actual CRC */
                uint32_t expected_crc, actual_crc;
                result = fcs_uint32_from_ascii_hex(
                    &expected_crc, &packet[field_start], field_len);
                actual_crc = fcs_crc32(packet, field_start, 0xFFFFFFFFu);
                /* Fail if mismatched */
                if (actual_crc != expected_crc) {
                    return FCS_DESERIALIZATION_ERROR;
                }
                break;
            default:
                assert(false);
                break;
        }
    }

    /*
    Check that the full message was parsed, and that the checksum and CRLF
    markers are in the expected places
    */
    if (field != 3u || idx != packet_length - 4u || packet[idx - 1u] != '*' ||
            packet[idx + 2u] != '\r' || packet[idx + 3u] != '\n') {
        return FCS_DESERIALIZATION_ERROR;
    }

    /* Checksum validity */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &packet[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
        return FCS_DESERIALIZATION_ERROR;
    }

    /* Now that we know everything's valid, handle the actual command */
    if (command_length > sizeof(MODE_CMD) - 1 &&
            memcmp(MODE_CMD, command, sizeof(MODE_CMD) - 1) == 0) {
        uint8_t new_mode = command[sizeof(MODE_CMD) - 1];
        fcs_ahrs_set_mode((enum fcs_mode_t)new_mode);
    }

    return FCS_DESERIALIZATION_OK;
}
