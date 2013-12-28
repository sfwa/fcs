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
#include "comms.h"
#include "../stats/stats.h"

size_t fcs_comms_serialize_status(uint8_t *restrict buf,
const struct fcs_packet_status_t *restrict status) {
    assert(buf);
    assert(status);
    assert(fcs_comms_validate_status(status) == FCS_VALIDATION_OK);

    size_t index = 0;

    memcpy(buf, "$PSFWAT,", 8u);
    index += 8u;

    index += fcs_ascii_from_int32(&buf[index], status->solution_time, 9u);
    buf[index++] = ',';

    memcpy(&buf[index], status->flags, 4u);
    index += 4u;
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->ioboard_resets[0], 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->ioboard_resets[1], 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->trical_resets[0], 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->trical_resets[1], 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->ukf_resets, 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->main_loop_cycle_max[0], 8u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->main_loop_cycle_max[1], 8u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->cpu_packet_rx, 9u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->cpu_packet_rx_err, 9u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(&buf[index], status->gps_num_svs, 2u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->telemetry_signal_db, 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->telemetry_noise_db, 3u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->telemetry_packet_rx, 6u);
    buf[index++] = ',';

    index += fcs_ascii_from_int32(
        &buf[index], status->telemetry_packet_rx_err, 6u);
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

enum fcs_validation_result_t fcs_comms_validate_status(
const struct fcs_packet_status_t *restrict status) {
    assert(status);

    if (0 <= status->solution_time && 0 <= status->ioboard_resets[0] &&
            0 <= status->ioboard_resets[1] &&
            0 <= status->trical_resets[0] &&
            0 <= status->trical_resets[1] && 0 <= status->ukf_resets &&
            0 <= status->main_loop_cycle_max[0] &&
            0 <= status->main_loop_cycle_max[1] &&
            0 <= status->cpu_packet_rx && 0 <= status->cpu_packet_rx_err &&
            0 <= status->gps_num_svs && status->gps_num_svs <= 99u &&
            -200 <= status->telemetry_signal_db &&
            status->telemetry_signal_db <= 100 &&
            -200 <= status->telemetry_noise_db &&
            status->telemetry_noise_db <= 100 &&
            0 <= status->telemetry_packet_rx &&
            0 <= status->telemetry_packet_rx_err) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}
