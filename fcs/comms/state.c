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

size_t fcs_comms_serialize_state(uint8_t *restrict buf,
const struct fcs_ahrs_state_t *restrict state) {
    assert(buf);
    assert(state);

    size_t index = 0, i;

    memcpy(buf, "$PSFWAS,", 8u);
    index += 8u;

    /* Output solution timestamp. Wrap-around at 30 bits (1073741.823s). */
    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)(state->solution_time & 0x3FFFFFFFu), 9u);
    buf[index++] = ',';

    /* Current path ID -- convert to 4-character hex */
    uint16_t path_id = fcs_global_nav_state.reference_path_id[0];
    index += fcs_ascii_hex_from_uint8(
        &buf[index], (uint8_t)(path_id & 0xFF00u) >> 8u);
    index += fcs_ascii_hex_from_uint8(
        &buf[index], (uint8_t)(path_id & 0x00FFu));
    buf[index++] = ',';

    /* Serialize position -- convert lat and lon to degrees first */
    index += fcs_ascii_fixed_from_double(
        &buf[index], state->lat * (180.0/M_PI), 2u, 7u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(
        &buf[index], state->lon * (180.0/M_PI), 3u, 7u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], state->alt, 4u, 2u);
    buf[index++] = ',';

    /* Serialize velocity and wind velocity components */
    for (i = 0; i < 3; i++) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->velocity[i], 3u, 2u);
        buf[index++] = ',';
    }

    for (i = 0; i < 3; i++) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->wind_velocity[i], 2u, 2u);
        buf[index++] = ',';
    }

    float yaw, pitch, roll, q[4];
    q[X] = (float)state->attitude[X];
    q[Y] = (float)state->attitude[Y];
    q[Z] = (float)state->attitude[Z];
    q[W] = (float)state->attitude[W];

    yaw_pitch_roll_from_quaternion_f(&yaw, &pitch, &roll, q);

    yaw *= (180.0 / M_PI);
    pitch *= (180.0 / M_PI);
    roll *= (180.0 / M_PI);

    if (yaw < 0.0) {
        yaw += 360.0;
    }
    assert(0.0 <= yaw && yaw <= 360.0);
    assert(-90.0 <= pitch && pitch <= 90.0);
    assert(-180.0 <= roll && roll <= 180.0);

    index += fcs_ascii_fixed_from_double(&buf[index], yaw, 3u, 1u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], pitch, 2u, 1u);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], roll, 3u, 1u);
    buf[index++] = ',';

    /*
    angular_velocity[0] is around the body x axis (roll), angular_velocity[1]
    is around the body y axis (pitch), and angular_velocity[2] is around the
    body z axis (yaw)
    */
    for (i = 0; i < 3; i++) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], state->angular_velocity[i] * (180.0/M_PI), 3u, 1u);
        buf[index++] = ',';
    }

    /*
    Work out 95th percentile confidence intervals for each of the output
    values, based on the current state error vector and the assumption that
    error will follow a Gaussian distribution.

    Uncertainty values are half the 95th percentile confidence interval, i.e.
    the extent of the interval away from the midpoint (being the field value
    output above).
    */

    /*
    Ignore changes in lon with varying lat -- small angles and all that.

    Formula for m per degree latitude is approx (2 * pi / 360) * r
    */
    double ci[14];
    ci[0] = 1.96 * 6378000.0 * max(state->lat_error, state->lon_error);
    ci[1] = 1.96 * state->alt_error;

    #pragma MUST_ITERATE(3, 3)
    for (i = 0; i < 3u; i++) {
        ci[2 + i] = 1.96 * state->velocity_error[i];
        ci[5 + i] = 1.96 * state->wind_velocity_error[i];

        /* Rotation around +X, +Y and +Z -- roll, pitch, yaw */
        ci[10 - i] = 1.96 * (180.0/M_PI) * state->attitude_error[i];
        ci[13 - i] = 1.96 * (180.0/M_PI) * state->angular_velocity_error[i];
    }

    index += fcs_ascii_fixed_from_double(&buf[index], ci[0], 3u, 0);
    buf[index++] = ',';

    index += fcs_ascii_fixed_from_double(&buf[index], ci[1], 2u, 1u);
    buf[index++] = ',';

    #pragma MUST_ITERATE(12, 12)
    for (i = 2u; i < 14u; i++) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], ci[i], 2u, 0);
        buf[index++] = ',';
    }

    buf[index++] = (uint8_t)state->mode;
    buf[index++] = ',';

    if (state->payload_present) {
        buf[index++] = 'P';
    } else {
        buf[index++] = 'M';
    }

    struct fcs_control_output_t control;
    fcs_exports_recv_control(&control);
    if (control.mode == FCS_CONTROL_MODE_MANUAL) {
        buf[index++] = 'R';
    } else {
        buf[index++] = 'F';
    }

    memset(&buf[index], '-', 5u);
    index += 5u;
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
