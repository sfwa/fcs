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
#include "../nmpc/cnmpc.h"
#include "../control/control.h"
#include "../exports/exports.h"
#include "comms.h"


enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
const uint8_t *restrict packet, size_t packet_length) {
    assert(packet);
    assert(
        FCS_COMMS_MIN_PACKET_SIZE <= packet_length && packet_length < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'W' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result = FCS_CONVERSION_ERROR;
    double tmp;
    uint32_t nav_state_version = 0;
    uint16_t waypoint_id = FCS_CONTROL_INVALID_WAYPOINT_ID;
    struct fcs_waypoint_t waypoint;
    int32_t packet_time;
    uint8_t flags[4];

    if (packet[0] != '$' || packet[1] != 'P' || packet[2] != 'S' ||
            packet[3] != 'F' || packet[4] != 'W' || packet[5] != 'A' ||
            packet[6] != 'W' || packet[7] != ',') {
        return FCS_DESERIALIZATION_ERROR;
    }

    /* Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values */
    for (field = 0; field < 11u && idx < packet_length && packet[idx] != '*';
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

        /*
        Handle the field data appropriately, based on the current field index
        */
        switch (field) {
            case 0:
                result = fcs_int32_from_ascii(
                    &packet_time, &packet[field_start], field_len);
                /*
                TODO: check that solution time is greater than the last
                observed value
                */
                break;
            case 1u:
                if (field_len != 8u) {
                    return FCS_DESERIALIZATION_ERROR;
                }

                result = fcs_uint32_from_ascii_hex(
                    &nav_state_version, &packet[field_start], field_len);
                break;
            case 2u:
                if (field_len != 4u) {
                    return FCS_DESERIALIZATION_ERROR;
                }

                result = fcs_uint16_from_ascii_hex(
                    &waypoint_id, &packet[field_start], field_len);
                if (waypoint_id >= FCS_CONTROL_MAX_WAYPOINTS) {
                    return FCS_DESERIALIZATION_ERROR;
                }
                break;
            case 3u:
                if (field_len != 4u) {
                    return FCS_DESERIALIZATION_ERROR;
                }

                memcpy(flags, &packet[field_start], field_len);
                result = FCS_CONVERSION_OK;
                /* TODO */
                break;
            case 4u:
                result = fcs_double_from_ascii_fixed(
                    &tmp, &packet[field_start], field_len);
                waypoint.lat = tmp;
                break;
            case 5u:
                result = fcs_double_from_ascii_fixed(
                    &tmp, &packet[field_start], field_len);
                waypoint.lon = tmp;
                break;
            case 6u:
                result = fcs_double_from_ascii_fixed(
                    &tmp, &packet[field_start], field_len);
                waypoint.alt = (float)tmp;
                break;
            case 7u:
                result = fcs_double_from_ascii_fixed(
                    &tmp, &packet[field_start], field_len);
                waypoint.yaw = (float)tmp;
                break;
            case 8u:
                result = fcs_double_from_ascii_fixed(
                    &tmp, &packet[field_start], field_len);
                waypoint.pitch = (float)tmp;
                break;
            case 9u:
                result = fcs_double_from_ascii_fixed(
                    &tmp, &packet[field_start], field_len);
                waypoint.roll = (float)tmp;
                break;
            case 10u:
                if (field_len != 8u) {
                    return FCS_DESERIALIZATION_ERROR;
                }

                /*
                Read the CRC in the message, and calculate the actual CRC
                */
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

        if (result != FCS_CONVERSION_OK) {
            return FCS_DESERIALIZATION_ERROR;
        }
    }

    /*
    Check that the full message was parsed, and that the checksum and CRLF
    markers are in the expected places
    */
    if (field != 7u || idx != packet_length - 4u || packet[idx - 1u] != '*' ||
            packet[idx + 2u] != '\r' || packet[idx + 3u] != '\n') {
        return FCS_DESERIALIZATION_ERROR;
    }

    /* Checksum validity */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &packet[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
        return FCS_DESERIALIZATION_ERROR;
    }

    /*
    TODO
    Check data for validity -- return an error if any values are out of range
    */

    /*
    Everything's valid -- send a message to the control core to update the
    relevant waypoint
    */
    fcs_exports_send_waypoint_update(nav_state_version, waypoint_id,
                                     &waypoint);

    return FCS_DESERIALIZATION_OK;
}
