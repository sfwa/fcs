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
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "comms.h"

void fcs_comms_init(void) {

}

void fcs_comms_tick(void) {

}

void fcs_comms_serialize_state(uint8_t *restrict buf,
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

    buf[index++] = state->mode_indicator;

    memcpy(&buf[index], state->flags, 4);
    index += 4;

    uint8_t checksum = fcs_text_checksum(buf, index);
    buf[index++] = '*';
    index += fcs_ascii_hex_from_uint8(&buf[index], checksum);
}

enum fcs_deserialization_result_t fcs_comms_deserialize_state(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len) {
    return FCS_DESERIALIZATION_ERROR;
}

void fcs_comms_serialize_waypoint(uint8_t *restrict buf,
const struct fcs_packet_state_t *restrict state) {

}

enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len) {
    return FCS_DESERIALIZATION_ERROR;
}

void fcs_comms_serialize_config(uint8_t *restrict buf,
const struct fcs_packet_config_t *restrict config) {

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
