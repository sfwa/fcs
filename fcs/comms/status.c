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

size_t fcs_comms_serialize_status(uint8_t *restrict buf,
const struct fcs_ahrs_state_t *restrict state,
const struct fcs_stats_counter_t *restrict counters,
const struct fcs_peripheral_state_t *restrict peripheral_state,
bool counter_reset) {
    static uint64_t last_ioboard_resets[2], last_trical_resets[2],
                    last_ukf_resets, last_nmpc_resets;

    assert(buf);
    assert(state);
    assert(counters);
    assert(peripheral_state);

    uint64_t count;
    size_t index = 0, i;

    memcpy(buf, "$PSFWAT,", 8u);
    index += 8u;

    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)(state->solution_time & 0x3FFFFFFFu), 9u);
    buf[index++] = ',';

    memset(&buf[index], '-', 4u);
    index += 4u;
    buf[index++] = ',';

    /*
    Output the change in I/O board count since the last packet was generated
    */
    for (i = 0; i < 2u; i++) {
        count = counters->ioboard_resets[i] - last_ioboard_resets[i];
        index += fcs_ascii_from_int32(
            &buf[index], (int32_t)(count & 0x0FFFFFFFu), 3u);
        buf[index++] = ',';

        if (counter_reset) {
            last_ioboard_resets[i] = counters->ioboard_resets[i];
        }
    }

    /*
    Same, but for TRICAL resets
    */
    for (i = 0; i < 2u; i++) {
        count = counters->trical_resets[i] - last_trical_resets[i];
        index += fcs_ascii_from_int32(
            &buf[index], (int32_t)(count & 0x0FFFFFFFu), 3u);
        buf[index++] = ',';

        if (counter_reset) {
            last_trical_resets[i] = counters->trical_resets[i];
        }
    }

    /* And again, for UKF resets */
    count = counters->ukf_resets - last_ukf_resets;
    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)(count & 0x0FFFFFFFu), 3u);
    buf[index++] = ',';

    if (counter_reset) {
        last_ukf_resets = counters->ukf_resets;
    }

    /* And for NMPC resets */
    count = counters->nmpc_resets - last_nmpc_resets;
    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)(count & 0x0FFFFFFFu), 3u);
    buf[index++] = ',';

    if (counter_reset) {
        last_nmpc_resets = counters->nmpc_resets;
    }

    /* Output the peak core cycle counts */
    for (i = 0; i < 2u; i++) {
        index += fcs_ascii_from_int32(
            &buf[index],
            (int32_t)(counters->main_loop_cycle_max[i] & 0x0FFFFFFFu), 8u);
        buf[index++] = ',';
    }

    /* Output CPU packet status */
    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)(counters->cpu_packet_rx & 0x0FFFFFFFu), 9u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)(counters->cpu_packet_rx_err & 0x0FFFFFFFu),
        9u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index],
        (int32_t)max(state->gps_num_svs[0], state->gps_num_svs[1]), 2u);
    buf[index++] = ',';

    /* Output the telemetry status */
    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)peripheral_state->telemetry_rssi, 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], (int32_t)peripheral_state->telemetry_noise, 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index],
        (int32_t)(peripheral_state->telemetry_packets & 0x7FFFFFFFu), 6u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index],
        (int32_t)(peripheral_state->telemetry_errors & 0x7FFFFFFFu), 6u);
    buf[index++] = ',';

    /*
    Calculate a CRC32 over the entire message. Note that the CRC32 in the
    serialized packet is the CRC32 of the serialized message, not the CRC32 of
    the underlying data.
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
