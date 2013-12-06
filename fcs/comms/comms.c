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
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../drivers/stream.h"
#include "comms.h"
#include "../ahrs/ahrs.h"

static uint32_t tick;
static uint8_t comms_buf[256u];

void fcs_comms_init(void) {
    /* Open the CPU comms stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT0, 230400u) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT0) == FCS_STREAM_OK);
}

void fcs_comms_tick(void) {
    /* Send a state update packet to the CPU every 20ms (50Hz) */
    if (tick % 20u == 0) {
        size_t packet_len, write_len;
        packet_len = fcs_comms_serialize_state(comms_buf, &global_state);
        assert(packet_len && packet_len < 256u);

        write_len = fcs_stream_write(FCS_STREAM_UART_EXT0, comms_buf,
                                     packet_len);

        /* We should definitely have enough room in the write buffer */
        assert(packet_len == write_len);
    }

    /*
    Increment tick, but wrap to 0 at a (decimal) round number in order to keep
    the output packet rate steady.
    */
    tick++;
    if (tick == 4000000000u) {
        tick = 0;
    }
}

size_t fcs_comms_serialize_state(uint8_t *restrict buf,
const struct fcs_packet_state_t *restrict state) {
    assert(buf);
    assert(state);
    assert(fcs_comms_validate_state(state) == FCS_VALIDATION_OK);

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
    if (state->lat_lon_uncertainty < FCS_STATE_MAX_LAT_LON_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->lat, 2u, 7u);
    }
    buf[index++] = ',';

    if (state->lat_lon_uncertainty < FCS_STATE_MAX_LAT_LON_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->lon, 3u, 7u);
    }
    buf[index++] = ',';

    if (state->alt_uncertainty < FCS_STATE_MAX_ALT_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->alt, 4u, 2u);
    }
    buf[index++] = ',';

    if (state->velocity_uncertainty[0] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[0], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->velocity_uncertainty[1u] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[1u], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->velocity_uncertainty[2u] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[2u], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->wind_velocity_uncertainty[0] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[0], 2u, 2u);
    }
    buf[index++] = ',';

    if (state->wind_velocity_uncertainty[1u] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[1u], 2u, 2u);
    }
    buf[index++] = ',';

    if (state->wind_velocity_uncertainty[2u] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[2u], 2u, 2u);
    }
    buf[index++] = ',';

    if (state->yaw_uncertainty < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(&buf[index], state->yaw, 3u, 2u);
    }
    buf[index++] = ',';

    if (state->pitch_uncertainty < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->pitch, 2u, 2u);
    }
    buf[index++] = ',';

    if (state->roll_uncertainty < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY) {
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
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[0], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->angular_velocity_uncertainty[1u] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[1u], 3u, 2u);
    }
    buf[index++] = ',';

    if (state->angular_velocity_uncertainty[2u] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[2u], 3u, 2u);
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
        &buf[index], state->velocity_uncertainty[1u], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[2u], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[1u], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[2u], 2u, 0);
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
        &buf[index], state->angular_velocity_uncertainty[1u], 2u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[2u], 2u, 0);
    buf[index++] = ',';

    buf[index++] = state->mode_indicator;
    buf[index++] = ',';

    memcpy(&buf[index], state->flags, 4u);
    index += 4u;

    /* Exclude initial $ from the checksum calculation */
    uint8_t checksum = fcs_text_checksum(&buf[1u], index - 1u);
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
    assert(len > 10u && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'S' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAS,", 8u) != 0) {
        goto invalid;
    }

    /* Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values */
    for (field = 0; field < 33u && idx < len && buf[idx] != '*'; field++) {
        size_t field_start = idx;
        for (; idx < len; idx++) {
            if (buf[idx] == '*') {
                idx++;
                break;
            }

            checksum ^= buf[idx];
            if (buf[idx] == ',') {
                idx++;
                break;
            }
        }

        size_t field_len = idx - field_start - 1u;
        if (field_len == 0) {
            continue;
        }
        assert(field_len < 256u);

        /*
        After deserializing each field, we check that the value is within the
        valid range described by the assertions in the serialize function.
        If not, we return a deserialization error.
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
            case 2u:
                result = fcs_double_from_ascii_fixed(
                    &state->lat, &buf[field_start], field_len);
                break;
            case 3u:
                result = fcs_double_from_ascii_fixed(
                    &state->lon, &buf[field_start], field_len);
                break;
            case 4u:
                result = fcs_double_from_ascii_fixed(
                    &state->alt, &buf[field_start], field_len);
                break;
            case 5u:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity[0], &buf[field_start], field_len);
                break;
            case 6u:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity[1u], &buf[field_start], field_len);
                break;
            case 7u:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity[2u], &buf[field_start], field_len);
                break;
            case 8u:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity[0], &buf[field_start], field_len);
                break;
            case 9u:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity[1u], &buf[field_start], field_len);
                break;
            case 10u:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity[2u], &buf[field_start], field_len);
                break;
            case 11u:
                result = fcs_double_from_ascii_fixed(
                    &state->yaw, &buf[field_start], field_len);
                break;
            case 12u:
                result = fcs_double_from_ascii_fixed(
                    &state->pitch, &buf[field_start], field_len);
                break;
            case 13u:
                result = fcs_double_from_ascii_fixed(
                    &state->roll, &buf[field_start], field_len);
                break;
            case 14u:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity[0], &buf[field_start], field_len
                );
                break;
            case 15u:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity[1u], &buf[field_start], field_len
                );
                break;
            case 16u:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity[2u], &buf[field_start], field_len
                );
                break;
            case 17u:
                result = fcs_double_from_ascii_fixed(
                    &state->lat_lon_uncertainty, &buf[field_start], field_len
                );
                break;
            case 18u:
                result = fcs_double_from_ascii_fixed(
                    &state->alt_uncertainty, &buf[field_start], field_len);
                break;
            case 19u:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity_uncertainty[0], &buf[field_start],
                    field_len
                );
                break;
            case 20u:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity_uncertainty[1u], &buf[field_start],
                    field_len
                );
                break;
            case 21u:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity_uncertainty[2u], &buf[field_start],
                    field_len
                );
                break;
            case 22u:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity_uncertainty[0], &buf[field_start],
                    field_len
                );
                break;
            case 23u:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity_uncertainty[1u], &buf[field_start],
                    field_len
                );
                break;
            case 24u:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity_uncertainty[2u], &buf[field_start],
                    field_len
                );
                break;
            case 25u:
                result = fcs_double_from_ascii_fixed(
                    &state->yaw_uncertainty, &buf[field_start], field_len);
                break;
            case 26u:
                result = fcs_double_from_ascii_fixed(
                    &state->pitch_uncertainty, &buf[field_start], field_len);
                break;
            case 27u:
                result = fcs_double_from_ascii_fixed(
                    &state->roll_uncertainty, &buf[field_start], field_len);
                break;
            case 28u:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity_uncertainty[0],
                    &buf[field_start], field_len
                );
                break;
            case 29u:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity_uncertainty[1u],
                    &buf[field_start], field_len
                );
                break;
            case 30u:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity_uncertainty[2u],
                    &buf[field_start], field_len
                );
                break;
            case 31u:
                if (field_len != 1u) {
                    goto invalid;
                }
                state->mode_indicator = buf[field_start];
                break;
            case 32u:
                if (field_len != 4u) {
                    goto invalid;
                }

                memcpy(state->flags, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            default:
                assert(false);
                break;
        }

        if (result != FCS_CONVERSION_OK) {
            goto invalid;
        }
    }

    /* Make sure the full message was parsed */
    if (field != 33u || idx != len - 4u || buf[idx - 1u] != '*' ||
            buf[idx + 2u] != '\r' || buf[idx + 3u] != '\n') {
        goto invalid;
    }

    /* State validity */
    if (fcs_comms_validate_state(state) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    /* Checksum validity */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &buf[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
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
        -1000.0 < state->velocity[1u] && state->velocity[1] < 1000.0 &&
        -1000.0 < state->velocity[2u] && state->velocity[2] < 1000.0 &&
        -100.0 < state->wind_velocity[0] &&
            state->wind_velocity[0] < 100.0 &&
        -100.0 < state->wind_velocity[1u] &&
            state->wind_velocity[1u] < 100.0 &&
        -100.0 < state->wind_velocity[2u] &&
            state->wind_velocity[2u] < 100.0 &&
        0.0 <= state->yaw && state->yaw < 360.0 &&
        -90.0 <= state->pitch && state->pitch <= 90.0 &&
        -180.0 <= state->roll && state->roll <= 180.0 &&
        -360.0 <= state->angular_velocity[0] &&
            state->angular_velocity[0] <= 360.0 &&
        -360.0 <= state->angular_velocity[1u] &&
            state->angular_velocity[1u] <= 360.0 &&
        -360.0 <= state->angular_velocity[2u] &&
            state->angular_velocity[2u] <= 360.0 &&
        0.0 <= state->lat_lon_uncertainty &&
        0.0 <= state->alt_uncertainty &&
        0.0 <= state->velocity_uncertainty[0] &&
        0.0 <= state->velocity_uncertainty[1u] &&
        0.0 <= state->velocity_uncertainty[2u] &&
        0.0 <= state->wind_velocity_uncertainty[0] &&
        0.0 <= state->wind_velocity_uncertainty[1u] &&
        0.0 <= state->wind_velocity_uncertainty[2u] &&
        0.0 <= state->yaw_uncertainty &&
        0.0 <= state->pitch_uncertainty &&
        0.0 <= state->roll_uncertainty &&
        0.0 <= state->angular_velocity_uncertainty[0] &&
        0.0 <= state->angular_velocity_uncertainty[1u] &&
        0.0 <= state->angular_velocity_uncertainty[2u]) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}

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

    memcpy(&buf[index], waypoint->flags, 5u);
    index += 5u;

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
    assert(len > 10u && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'P' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAP,", 8u) != 0) {
        goto invalid;
    }

    /* Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values */
    for (field = 0; field < 10u && idx < len && buf[idx] != '*'; field++) {
        size_t field_start = idx;
        for (; idx < len; idx++) {
            if (buf[idx] == '*') {
                idx++;
                break;
            }

            checksum ^= buf[idx];
            if (buf[idx] == ',') {
                idx++;
                break;
            }
        }

        size_t field_len = idx - field_start - 1u;
        if (field_len == 0) {
            continue;
        }
        assert(field_len < 256u);

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
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_lat, &buf[field_start], field_len);
                break;
            case 3u:
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_lon, &buf[field_start], field_len);
                break;
            case 4u:
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_alt, &buf[field_start], field_len);
                break;
            case 5u:
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_yaw, &buf[field_start], field_len);
                break;
            case 6u:
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_pitch, &buf[field_start], field_len);
                break;
            case 7u:
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_roll, &buf[field_start], field_len);
                break;
            case 8u:
                result = fcs_double_from_ascii_fixed(
                    &waypoint->target_airspeed, &buf[field_start], field_len);
                break;
            case 9u:
                if (field_len != 5u) {
                    goto invalid;
                }

                memcpy(waypoint->flags, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            default:
                assert(false);
                break;
        }

        if (result != FCS_CONVERSION_OK) {
            goto invalid;
        }
    }

    /* Make sure the full message was parsed */
    if (field != 10u || idx != len - 4u || buf[idx - 1u] != '*' ||
            buf[idx + 2u] != '\r' || buf[idx + 3u] != '\n') {
        goto invalid;
    }

    /* Check data validity */
    if (fcs_comms_validate_waypoint(waypoint) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    /* Checksum */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &buf[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
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

size_t fcs_comms_serialize_config(uint8_t *restrict buf,
const struct fcs_packet_config_t *restrict config) {
    assert(buf);
    assert(config);
    assert(fcs_comms_validate_config(config) == FCS_VALIDATION_OK);

    size_t index = 0;

    memcpy(buf, "$PSFWAC,", 8u);
    index += 8u;

    memcpy(&buf[index], config->param_name, config->param_name_len);
    index += config->param_name_len;
    buf[index++] = ',';

    /* Base64-encode config value */
    uint8_t len = fcs_base64_from_data(&buf[index], 192u, config->param_value,
                                       config->param_value_len);
    assert(0 < len && len < 192u);
    index += len;
    buf[index++] = ',';

    /* Exclude initial $ from the checksum calculation */
    uint8_t checksum = fcs_text_checksum(&buf[1], index - 1u);
    buf[index++] = '*';
    index += fcs_ascii_hex_from_uint8(&buf[index], checksum);

    buf[index++] = '\r';
    buf[index++] = '\n';

    return index;
}

enum fcs_deserialization_result_t fcs_comms_deserialize_config(
struct fcs_packet_config_t *restrict config, uint8_t *restrict buf,
size_t len) {
    assert(config);
    assert(buf);
    assert(len > 10u && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'C' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAC,", 8u) != 0) {
        goto invalid;
    }

    /* Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values */
    for (field = 0; field < 2u && idx < len && buf[idx] != '*'; field++) {
        size_t field_start = idx;
        for (; idx < len; idx++) {
            if (buf[idx] == '*') {
                idx++;
                break;
            }

            checksum ^= buf[idx];
            if (buf[idx] == ',') {
                idx++;
                break;
            }
        }

        size_t field_len = idx - field_start - 1u;
        if (field_len == 0) {
            continue;
        }
        assert(field_len < 256u);

        switch (field) {
            case 0:
                /* Param name -- just copy the string */
                if (field_len == 0 || field_len > 24u) {
                    goto invalid;
                }

                config->param_name_len = field_len;
                memcpy(config->param_name, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 1u:
                /* Param data conversion -- Base64 decode */
                if (field_len < 4u || field_len > 192u) {
                    goto invalid;
                }

                config->param_value_len = fcs_data_from_base64(
                    config->param_value, 128u, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            default:
                assert(false);
                break;
        }

        if (result != FCS_CONVERSION_OK) {
            goto invalid;
        }
    }

    /* Make sure the full message was parsed */
    if (field != 2u || idx != len - 4u || buf[idx - 1u] != '*' ||
            buf[idx + 2u] != '\r' || buf[idx + 3u] != '\n') {
        goto invalid;
    }

    /* Check data validity */
    if (fcs_comms_validate_config(config) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    /* Checksum */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &buf[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
        goto invalid;
    }

    return FCS_DESERIALIZATION_OK;

invalid:
    memset(config, 0xFFu, sizeof(struct fcs_packet_config_t));
    return FCS_DESERIALIZATION_ERROR;
}

enum fcs_validation_result_t fcs_comms_validate_config(
const struct fcs_packet_config_t *restrict config) {
    assert(config);

    if (0 < config->param_name_len && config->param_name_len <= 24u &&
        0 < config->param_value_len && config->param_value_len <= 128u) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}

enum fcs_deserialization_result_t fcs_comms_deserialize_gcs(
struct fcs_packet_gcs_t *restrict gcs, uint8_t *restrict buf, size_t len) {
    assert(gcs);
    assert(buf);
    assert(len > 10u && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'G' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAG,", 8u) != 0) {
        goto invalid;
    }

    /* Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values */
    for (field = 0; field < 5u && idx < len && buf[idx] != '*'; field++) {
        size_t field_start = idx;
        for (; idx < len; idx++) {
            if (buf[idx] == '*') {
                idx++;
                break;
            }

            checksum ^= buf[idx];
            if (buf[idx] == ',') {
                idx++;
                break;
            }
        }

        size_t field_len = idx - field_start - 1u;
        if (field_len == 0) {
            continue;
        }
        assert(field_len < 256u);

        switch (field) {
            case 0:
                result = fcs_int32_from_ascii(
                    &gcs->solution_time, &buf[field_start], field_len);
                if (gcs->solution_time < 0) {
                    goto invalid;
                }
                break;
            case 1u:
                result = fcs_double_from_ascii_fixed(
                    &gcs->lat, &buf[field_start], field_len);
                break;
            case 2u:
                result = fcs_double_from_ascii_fixed(
                    &gcs->lon, &buf[field_start], field_len);
                break;
            case 3u:
                result = fcs_double_from_ascii_fixed(
                    &gcs->alt, &buf[field_start], field_len);
                break;
            case 4u:
                result = fcs_double_from_ascii_fixed(
                    &gcs->pressure, &buf[field_start], field_len);
                break;
            default:
                assert(false);
                break;
        }

        if (result != FCS_CONVERSION_OK) {
            goto invalid;
        }
    }

    /* Make sure the full message was parsed */
    if (field != 5u || idx != len - 4u || buf[idx - 1u] != '*' ||
            buf[idx + 2u] != '\r' || buf[idx + 3u] != '\n') {
        goto invalid;
    }

    /* Check data validity */
    if (fcs_comms_validate_gcs(gcs) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    /* Checksum */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &buf[idx], 2u);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
        goto invalid;
    }

    return FCS_DESERIALIZATION_OK;

invalid:
    memset(gcs, 0xFFu, sizeof(struct fcs_packet_gcs_t));
    return FCS_DESERIALIZATION_ERROR;
}

enum fcs_validation_result_t fcs_comms_validate_gcs(
const struct fcs_packet_gcs_t *restrict gcs) {
    assert(gcs);

    if (0 <= gcs->solution_time &&
        -90.0 <= gcs->lat && gcs->lat <= 90.0 &&
        -180.0 <= gcs->lon && gcs->lon <= 180.0 &&
        -500.0 <= gcs->alt && gcs->alt < 10000.0 &&
        700.0 <= gcs->pressure && gcs->pressure < 1200.0) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}
