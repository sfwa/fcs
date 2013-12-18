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
#include "comms.h"
#include "../ahrs/ahrs.h"
#include "../stats/stats.h"

size_t fcs_comms_serialize_waypoint(uint8_t *restrict buf,
const struct fcs_packet_waypoint_t *restrict waypoint) {
    assert(buf);
    assert(waypoint);
    assert(fcs_comms_validate_waypoint(waypoint) == FCS_VALIDATION_OK);

    size_t index = 0;

    memcpy(buf, "$PSFWAP,", 8u);
    index += 8u;

    memcpy(&buf[index], waypoint->waypoint_id, 4u);
    index += 4u;
    buf[index++] = ',';

    buf[index++] = waypoint->waypoint_role;
    buf[index++] = ',';

    memcpy(&buf[index], waypoint->flags, 3u);
    index += 3u;
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], waypoint->target_lat,
                                         2u, 7u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], waypoint->target_lon,
                                         3u, 7u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], waypoint->target_alt,
                                         4u, 2u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], waypoint->target_yaw,
                                         3u, 2u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], waypoint->target_pitch,
                                         2u, 2u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], waypoint->target_roll,
                                         3u, 2u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index],
                                         waypoint->target_airspeed, 3u, 2u);
    buf[index++] = ',';

    /*
    Calculate a CRC32 over the entire message. Note that the CRC32 in the
    serialized packet is the CRC32 of the serialized message, not the CRC32 of
    the underlying data.
    */
    uint32_t crc = fcs_crc32(buf, index, 0xFFFFFFFFu);
    index += fcs_ascii_hex_from_uint32(&buf[index], crc);

    /* Exclude initial $ from the checksum calculation */
    uint8_t checksum = fcs_text_checksum(&buf[1u], index - 1u);
    buf[index++] = '*';
    index += fcs_ascii_hex_from_uint8(&buf[index], checksum);

    buf[index++] = '\r';
    buf[index++] = '\n';

    return index;
}

enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len) {
    assert(waypoint);
    assert(buf);
    assert(FCS_COMMS_MIN_PACKET_SIZE <= len && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'P' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAP,", 8u) != 0) {
        goto invalid;
    }

    /*
    Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values.

    Stop once we've processed all the fields, hit the end of the buffer, or
    hit the checksum marker ('*').
    */
    for (field = 0; field < 11u && idx < len && buf[idx] != '*'; field++) {
        size_t field_start = idx;
        for (; idx < len; idx++) {
            if (buf[idx] == '*') {
                /* End of message -- break without updating checksum value */
                idx++;
                break;
            }

            /* Update checksum value with current byte */
            checksum ^= buf[idx];
            if (buf[idx] == ',') {
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
                if (field_len != 4u) {
                    goto invalid;
                }

                memcpy(waypoint->waypoint_id, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 1u:
                if (field_len != 1u) {
                    goto invalid;
                }
                waypoint->waypoint_role = buf[field_start];
                break;
            case 2u:
                if (field_len != 3u) {
                    goto invalid;
                }

                memcpy(waypoint->flags, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 10u:
                if (field_len != 8u) {
                    goto invalid;
                }

                /*
                Read the CRC in the message, and calculate the actual CRC
                */
                uint32_t expected_crc, actual_crc;
                result = fcs_uint32_from_ascii_hex(
                    &expected_crc, &buf[field_start], field_len);
                actual_crc = fcs_crc32(buf, field_start, 0xFFFFFFFFu);
                /* Fail if mismatched */
                if (actual_crc != expected_crc) {
                    goto invalid;
                }
                break;
            default:
                /*
                Handle fields 3-9 -- just a bunch of double conversions, and
                order is the same between the waypoint structure and the
                message
                */
                assert(3u <= field && field <= 9u);
                double *data_ptr = &waypoint->target_lat;
                result = fcs_double_from_ascii_fixed(
                    &data_ptr[field - 3u], &buf[field_start], field_len);
                break;
        }

        if (result != FCS_CONVERSION_OK) {
            goto invalid;
        }
    }

    /*
    Check that the full message was parsed, and that the checksum and CRLF
    markers are in the expected places
    */
    if (field != 11u || idx != len - 4u || buf[idx - 1u] != '*' ||
            buf[idx + 2u] != '\r' || buf[idx + 3u] != '\n') {
        goto invalid;
    }

    /* Checksum validity */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &buf[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
        goto invalid;
    }

    /* Check data validity */
    if (fcs_comms_validate_waypoint(waypoint) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    return FCS_DESERIALIZATION_OK;

invalid:
    memset(waypoint, 0xFFu, sizeof(struct fcs_packet_waypoint_t));
    return FCS_DESERIALIZATION_ERROR;
}

enum fcs_validation_result_t fcs_comms_validate_waypoint(
const struct fcs_packet_waypoint_t *restrict waypoint) {
    assert(waypoint);

    if (-90.0 <= waypoint->target_lat && waypoint->target_lat <= 90.0 &&
        -180.0 <= waypoint->target_lon && waypoint->target_lon <= 180.0 &&
        -500.0 <= waypoint->target_alt && waypoint->target_alt < 10000.0 &&
        0.0 <= waypoint->target_yaw && waypoint->target_yaw < 360.0 &&
        -90.0 <= waypoint->target_pitch && waypoint->target_pitch <= 90.0 &&
        -180.0 <= waypoint->target_roll && waypoint->target_roll <= 180.0 &&
        0.0 < waypoint->target_airspeed &&
            waypoint->target_airspeed < 1000.0) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}
