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
static uint8_t comms_buf[256];

void fcs_comms_init(void) {
    /* Open the CPU comms stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT0, 230400) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT0) == FCS_STREAM_OK);
}

void fcs_comms_tick(void) {
    /* Send a state update packet to the CPU every 20ms (50Hz) */
    if (tick % 20 == 0) {
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
    assert(state->solution_time >= 0);

    size_t index = 0;

    memcpy(buf, "$PSFWAS,", 8u);
    index += 8u;

    index += fcs_ascii_from_int32(&buf[index], state->solution_time, 9u);
    buf[index++] = ',';

    memcpy(&buf[index], state->next_waypoint_id, 4u);
    index += 4u;
    buf[index++] = ',';

    assert(-90.0 <= state->lat && state->lat <= 90.0);
    if (state->lat_lon_uncertainty < FCS_STATE_MAX_LAT_LON_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->lat, 2u, 7u);
    }
    buf[index++] = ',';

    assert(-180.0 <= state->lon && state->lon <= 180.0);
    if (state->lat_lon_uncertainty < FCS_STATE_MAX_LAT_LON_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->lon, 3u, 7u);
    }
    buf[index++] = ',';

    assert(-500.0 <= state->alt && state->alt < 10000.0);
    if (state->alt_uncertainty < FCS_STATE_MAX_ALT_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->alt, 4u, 2u);
    }
    buf[index++] = ',';

    assert(-1000.0 < state->velocity[0] && state->velocity[0] < 1000.0);
    if (state->velocity_uncertainty[0] < FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[0], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-1000.0 < state->velocity[1] && state->velocity[1] < 1000.0);
    if (state->velocity_uncertainty[1] < FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[1], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-1000.0 < state->velocity[2] && state->velocity[2] < 1000.0);
    if (state->velocity_uncertainty[2] < FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[2], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-100.0 < state->wind_velocity[0] &&
           state->wind_velocity[0] < 100.0);
    if (state->wind_velocity_uncertainty[0] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[0], 2u, 2u);
    }
    buf[index++] = ',';

    assert(-100.0 < state->wind_velocity[1] &&
           state->wind_velocity[1] < 100.0);
    if (state->wind_velocity_uncertainty[1] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[1], 2u, 2u);
    }
    buf[index++] = ',';

    assert(-100.0 < state->wind_velocity[2] &&
           state->wind_velocity[2] < 100.0);
    if (state->wind_velocity_uncertainty[2] <
            FCS_STATE_MAX_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[2], 2u, 2u);
    }
    buf[index++] = ',';

    assert(0.0 <= state->attitude[0] && state->attitude[0] < 360.0);
    if (state->attitude_uncertainty[2] < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->attitude[0], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-90.0 <= state->attitude[1] && state->attitude[1] <= 90.0);
    if (state->attitude_uncertainty[2] < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->attitude[1], 2u, 2u);
    }
    buf[index++] = ',';

    assert(-180.0 <= state->attitude[2] && state->attitude[2] <= 180.0);
    if (state->attitude_uncertainty[2] < FCS_STATE_MAX_ATTITUDE_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->attitude[2], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-360.0 <= state->angular_velocity[0] &&
           state->angular_velocity[0] <= 360.0);
    if (state->angular_velocity_uncertainty[0] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[0], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-360.0 <= state->angular_velocity[1] &&
           state->angular_velocity[1] <= 360.0);
    if (state->angular_velocity_uncertainty[1] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[1], 3u, 2u);
    }
    buf[index++] = ',';

    assert(-360.0 <= state->angular_velocity[2] &&
           state->angular_velocity[2] <= 360.0);
    if (state->angular_velocity_uncertainty[2] <
            FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[2], 3u, 2u);
    }
    buf[index++] = ',';

    assert(0.0 <= state->lat_lon_uncertainty);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->lat_lon_uncertainty, 3u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->alt_uncertainty);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->alt_uncertainty, 2u, 1u);
    buf[index++] = ',';

    assert(0.0 <= state->velocity_uncertainty[0]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->velocity_uncertainty[1]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->velocity_uncertainty[2]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->velocity_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->wind_velocity_uncertainty[0]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->wind_velocity_uncertainty[1]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->wind_velocity_uncertainty[2]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->wind_velocity_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->attitude_uncertainty[0]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->attitude_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->attitude_uncertainty[1]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->attitude_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->attitude_uncertainty[2]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->attitude_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->angular_velocity_uncertainty[0]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[0], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->angular_velocity_uncertainty[1]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[1], 2u, 0);
    buf[index++] = ',';

    assert(0.0 <= state->angular_velocity_uncertainty[2]);
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->angular_velocity_uncertainty[2], 2u, 0);
    buf[index++] = ',';

    buf[index++] = state->mode_indicator;
    buf[index++] = ',';

    memcpy(&buf[index], state->flags, 4);
    index += 4;

    /* Exclude initial $ from the checksum calculation */
    uint8_t checksum = fcs_text_checksum(&buf[1], index - 1);
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
    assert(len > 10 && len < 256);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'S' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAS,", 8) != 0) {
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
        assert(field_len < 256);

        switch (field) {
            case 0:
                result = fcs_int32_from_ascii(
                    &state->solution_time, &buf[field_start], field_len);
                break;
            case 1:
                if (field_len != 4) {
                    goto invalid;
                }

                memcpy(state->next_waypoint_id, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 2:
                result = fcs_double_from_ascii_fixed(
                    &state->lat, &buf[field_start], field_len);
                break;
            case 3:
                result = fcs_double_from_ascii_fixed(
                    &state->lon, &buf[field_start], field_len);
                break;
            case 4:
                result = fcs_double_from_ascii_fixed(
                    &state->alt, &buf[field_start], field_len);
                break;
            case 5:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity[0], &buf[field_start], field_len);
                break;
            case 6:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity[1], &buf[field_start], field_len);
                break;
            case 7:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity[2], &buf[field_start], field_len);
                break;
            case 8:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity[0], &buf[field_start], field_len);
                break;
            case 9:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity[1], &buf[field_start], field_len);
                break;
            case 10:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity[2], &buf[field_start], field_len);
                break;
            case 11:
                result = fcs_double_from_ascii_fixed(
                    &state->attitude[0], &buf[field_start], field_len);
                break;
            case 12:
                result = fcs_double_from_ascii_fixed(
                    &state->attitude[1], &buf[field_start], field_len);
                break;
            case 13:
                result = fcs_double_from_ascii_fixed(
                    &state->attitude[2], &buf[field_start], field_len);
                break;
            case 14:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity[0], &buf[field_start], field_len
                );
                break;
            case 15:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity[1], &buf[field_start], field_len
                );
                break;
            case 16:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity[2], &buf[field_start], field_len
                );
                break;
            case 17:
                result = fcs_double_from_ascii_fixed(
                    &state->lat_lon_uncertainty, &buf[field_start], field_len
                );
                break;
            case 18:
                result = fcs_double_from_ascii_fixed(
                    &state->alt_uncertainty, &buf[field_start], field_len);
                break;
            case 19:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity_uncertainty[0], &buf[field_start],
                    field_len
                );
                break;
            case 20:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity_uncertainty[1], &buf[field_start],
                    field_len
                );
                break;
            case 21:
                result = fcs_double_from_ascii_fixed(
                    &state->velocity_uncertainty[2], &buf[field_start],
                    field_len
                );
                break;
            case 22:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity_uncertainty[0], &buf[field_start],
                    field_len
                );
                break;
            case 23:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity_uncertainty[1], &buf[field_start],
                    field_len
                );
                break;
            case 24:
                result = fcs_double_from_ascii_fixed(
                    &state->wind_velocity_uncertainty[2], &buf[field_start],
                    field_len
                );
                break;
            case 25:
                result = fcs_double_from_ascii_fixed(
                    &state->attitude_uncertainty[0], &buf[field_start],
                    field_len
                );
                break;
            case 26:
                result = fcs_double_from_ascii_fixed(
                    &state->attitude_uncertainty[1], &buf[field_start],
                    field_len
                );
                break;
            case 27:
                result = fcs_double_from_ascii_fixed(
                    &state->attitude_uncertainty[2], &buf[field_start],
                    field_len
                );
                break;
            case 28:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity_uncertainty[0],
                    &buf[field_start], field_len
                );
                break;
            case 29:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity_uncertainty[1],
                    &buf[field_start], field_len
                );
                break;
            case 30:
                result = fcs_double_from_ascii_fixed(
                    &state->angular_velocity_uncertainty[2],
                    &buf[field_start], field_len
                );
                break;
            case 31:
                if (field_len != 1) {
                    goto invalid;
                }
                state->mode_indicator = buf[field_start];
                break;
            case 32:
                if (field_len != 4) {
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
    if (field != 33 || idx != len - 4 || buf[idx - 1] != '*' ||
            buf[idx + 2] != '\r' || buf[idx + 3] != '\n') {
        goto invalid;
    }

    /* Check validity */
    uint8_t message_checksum;
    result = fcs_uint8_from_ascii_hex(&message_checksum, &buf[idx], 2);
    if (result != FCS_CONVERSION_OK || message_checksum != checksum) {
        goto invalid;
    }

    return FCS_DESERIALIZATION_OK;

invalid:
    memset(state, 0, sizeof(struct fcs_packet_state_t));
    return FCS_DESERIALIZATION_ERROR;
}

size_t fcs_comms_serialize_waypoint(uint8_t *restrict buf,
const struct fcs_packet_waypoint_t *restrict waypoint) {
    return 0;
}

enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len) {
    return FCS_DESERIALIZATION_ERROR;
}

size_t fcs_comms_serialize_config(uint8_t *restrict buf,
const struct fcs_packet_config_t *restrict config) {
    return 0;
}

enum fcs_deserialization_result_t fcs_comms_deserialize_config(
struct fcs_packet_config_t *restrict config, uint8_t *restrict buf,
size_t len) {
    return FCS_DESERIALIZATION_ERROR;
}

enum fcs_deserialization_result_t fcs_comms_deserialize_gcs(
struct fcs_packet_gcs_t *restrict gcs, uint8_t *restrict buf, size_t len) {
    return FCS_DESERIALIZATION_ERROR;
}
