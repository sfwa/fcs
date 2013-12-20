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

size_t fcs_comms_serialize_state(uint8_t *restrict buf,
const struct fcs_packet_state_t *restrict state) {
    assert(buf);
    assert(state);
    /* If the state is not valid, the mode indicator should be 'N' */
    assert(fcs_comms_validate_state(state) == FCS_VALIDATION_OK ||
           state->mode_indicator == 'N');

    size_t index = 0;

    memcpy(buf, "$PSFWAS,", 8u);
    index += 8u;

    index += fcs_ascii_from_int32(&buf[index], state->solution_time, 9u);
    buf[index++] = ',';

    memcpy(&buf[index], state->next_waypoint_id, 4u);
    index += 4u;
    buf[index++] = ',';

    /*
    Serialize the data fields. In most cases we check that the field
    uncertainty is below a certain threshold in order to output the data at
    all.
    */
    if (state->lat_lon_uncertainty < FCS_STATE_MAX_LAT_LON_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->lat, 2u, 7u);
    }
    buf[index++] = ',';

    if (state->lat_lon_uncertainty < FCS_STATE_MAX_LAT_LON_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->lon, 3u, 7u);
    }
    buf[index++] = ',';

    if (state->alt_uncertainty < FCS_STATE_MAX_ALT_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->alt, 4u, 2u);
    }
    buf[index++] = ',';

    if (state->velocity_uncertainty[0] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[0], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->velocity_uncertainty[1] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[1], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->velocity_uncertainty[2] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[2], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->wind_velocity_uncertainty[0] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[0], 2u, 2u);
    }
    buf[index++] = ',';

    if (state->wind_velocity_uncertainty[1] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[1], 2u, 2u);
    }
    buf[index++] = ',';

    if (state->wind_velocity_uncertainty[2] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[2], 2u, 2u);
    }
    buf[index++] = ',';

    if (state->yaw_uncertainty < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(&buf[index], state->yaw, 3u, 2u);
    }
    buf[index++] = ',';

    if (state->pitch_uncertainty < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->pitch, 2u, 2u);
    }
    buf[index++] = ',';

    if (state->roll_uncertainty < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->roll, 3u, 2u);
    }
    buf[index++] = ',';

    /*
    angular_velocity[0] is around the body x axis (roll), angular_velocity[1]
    is around the body y axis (pitch), and angular_velocity[2] is around the
    body z axis (yaw)
    */
    if (state->angular_velocity_uncertainty[0] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[0], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->angular_velocity_uncertainty[1] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[1], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->angular_velocity_uncertainty[2] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY &&
            state->mode_indicator != 'N') {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[2], 3u, 2u);
    }
    buf[index++] = ',';

    /*
    Uncertainty values are half the 95th percentile confidence interval, i.e.
    the extent of the interval away from the midpoint (being the field value
    output above).
    */
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->lat_lon_uncertainty, 3u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->alt_uncertainty, 2u, 1u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->yaw_uncertainty, 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->pitch_uncertainty, 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->roll_uncertainty, 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    buf[index++] = state->mode_indicator;
    buf[index++] = ',';

    memcpy(&buf[index], state->flags, 7u);
    index += 7u;
    buf[index++] = ',';

    /*
    Calculate a CRC32 over the entire message. Note that the CRC32 in the
    serialized packet is the CRC32 of the serialized message, not the CRC32 of
    the state structure.
    */
    uint32_t crc = fcs_crc32(buf, index, 0xFFFFFFFFu);
    index += fcs_ascii_hex_from_uint32(&buf[index], crc);

    /* Exclude initial $ from the checksum calculation */
    uint8_t checksum = fcs_text_checksum(&buf[1], index - 1u);
    buf[index++] = '*';
    index += fcs_ascii_hex_from_uint8(&buf[index], checksum);

    buf[index++] = '\r';
    buf[index++] = '\n';

    return index;
}

enum fcs_deserialization_result_t fcs_comms_deserialize_state(
struct fcs_packet_state_t *restrict state, uint8_t *restrict buf,
size_t len) {
    assert(state);
    assert(buf);
    assert(FCS_COMMS_MIN_PACKET_SIZE <= len && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'S' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAS,", 8u) != 0) {
        goto invalid;
    }

    /*
    Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values.

    Stop once we've processed all the fields, hit the end of the buffer, or
    hit the checksum marker ('*').
    */
    for (field = 0; field < 34u && idx < len && buf[idx] != '*'; field++) {
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
                result = fcs_int32_from_ascii(
                    &state->solution_time, &buf[field_start], field_len);
                break;
            case 1u:
                if (field_len != 4u) {
                    goto invalid;
                }

                memcpy(state->next_waypoint_id, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 31u:
                /* 'N' mode indicator means not valid */
                if (field_len != 1u || buf[field_start] == 'N') {
                    goto invalid;
                }
                state->mode_indicator = buf[field_start];
                break;
            case 32u:
                if (field_len != 7u) {
                    goto invalid;
                }

                memcpy(state->flags, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 33u:
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
                Handle fields 2-30 -- just a bunch of double conversions, and
                order is the same between the state structure and the message
                */
                assert(2u <= field && field <= 30u);
                double *data_ptr = &state->lat;
                result = fcs_double_from_ascii_fixed(
                    &data_ptr[field - 2u], &buf[field_start], field_len);
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
    if (field != 34u || idx != len - 4u || buf[idx - 1u] != '*' ||
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
    if (fcs_comms_validate_state(state) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    return FCS_DESERIALIZATION_OK;

invalid:
    memset(state, 0xFFu, sizeof(struct fcs_packet_state_t));
    return FCS_DESERIALIZATION_ERROR;
}

enum fcs_validation_result_t fcs_comms_validate_state(
const struct fcs_packet_state_t *restrict state) {
    assert(state);

    if (0 <= state->solution_time &&
        -90.0 <= state->lat && state->lat <= 90.0 &&
        -180.0 <= state->lon && state->lon <= 180.0 &&
        -500.0 <= state->alt && state->alt < 10000.0 &&
        -1000.0 < state->velocity[0] && state->velocity[0] < 1000.0 &&
        -1000.0 < state->velocity[1] && state->velocity[1] < 1000.0 &&
        -1000.0 < state->velocity[2] && state->velocity[2] < 1000.0 &&
        -100.0 < state->wind_velocity[0] &&
            state->wind_velocity[0] < 100.0 &&
        -100.0 < state->wind_velocity[1] &&
            state->wind_velocity[1] < 100.0 &&
        -100.0 < state->wind_velocity[2] &&
            state->wind_velocity[2] < 100.0 &&
        0.0 <= state->yaw && state->yaw < 360.0 &&
        -90.0 <= state->pitch && state->pitch <= 90.0 &&
        -180.0 <= state->roll && state->roll <= 180.0 &&
        -360.0 <= state->angular_velocity[0] &&
            state->angular_velocity[0] <= 360.0 &&
        -360.0 <= state->angular_velocity[1] &&
            state->angular_velocity[1] <= 360.0 &&
        -360.0 <= state->angular_velocity[2] &&
            state->angular_velocity[2] <= 360.0 &&
        0.0 <= state->lat_lon_uncertainty &&
        0.0 <= state->alt_uncertainty &&
        0.0 <= state->velocity_uncertainty[0] &&
        0.0 <= state->velocity_uncertainty[1] &&
        0.0 <= state->velocity_uncertainty[2] &&
        0.0 <= state->wind_velocity_uncertainty[0] &&
        0.0 <= state->wind_velocity_uncertainty[1] &&
        0.0 <= state->wind_velocity_uncertainty[2] &&
        0.0 <= state->yaw_uncertainty &&
        0.0 <= state->pitch_uncertainty &&
        0.0 <= state->roll_uncertainty &&
        0.0 <= state->angular_velocity_uncertainty[0] &&
        0.0 <= state->angular_velocity_uncertainty[1] &&
        0.0 <= state->angular_velocity_uncertainty[2]) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}
