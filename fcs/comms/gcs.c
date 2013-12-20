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

enum fcs_deserialization_result_t fcs_comms_deserialize_gcs(
struct fcs_packet_gcs_t *restrict gcs, uint8_t *restrict buf, size_t len) {
    assert(gcs);
    assert(buf);
    assert(FCS_COMMS_MIN_PACKET_SIZE <= len && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'G' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAG,", 8u) != 0) {
        goto invalid;
    }

    /* Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values */
    for (field = 0; field < 7u && idx < len && buf[idx] != '*'; field++) {
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
                    &gcs->solution_time, &buf[field_start], field_len);
                if (gcs->solution_time < 0) {
                    goto invalid;
                }
                break;
            case 1u:
                if (field_len != 4u) {
                    goto invalid;
                }

                memcpy(gcs->flags, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 6u:
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
                Handle fields 2-5 -- just a bunch of double conversions, and
                order is the same between the GCS structure and the message
                */
                assert(2u <= field && field <= 5u);
                double *data_ptr = &gcs->lat;
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
    if (field != 7u || idx != len - 4u || buf[idx - 1u] != '*' ||
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
    if (fcs_comms_validate_gcs(gcs) != FCS_VALIDATION_OK) {
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
