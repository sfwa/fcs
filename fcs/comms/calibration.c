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
#include <float.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../drivers/stream.h"
#include "comms.h"
#include "../ahrs/measurement.h"

size_t fcs_comms_serialize_calibration(uint8_t *restrict buf,
const struct fcs_packet_calibration_t *restrict calibration) {
    assert(buf);
    assert(calibration);
    assert(fcs_comms_validate_calibration(calibration) == FCS_VALIDATION_OK);

    size_t index = 0, i;

    memcpy(buf, "$PSFWAL,", 8u);
    index += 8u;

    buf[index++] = calibration->sensor_type + 'A';
    buf[index++] = ',';

    buf[index++] = calibration->sensor_id + '0';
    buf[index++] = ',';

    memcpy(&buf[index], calibration->flags, 5u);
    index += 5u;
    buf[index++] = ',';

    buf[index++] = calibration->calibration_type + 'a';
    buf[index++] = ',';

    /* Output error, 9x params, 4x orientation, 3x offset */
    for (i = 0; i < 17u; i++) {
        index += fcs_ascii_fixed_from_double(
            &buf[index], (&calibration->error)[i], 3u, 7u);
        buf[index++] = ',';
    }

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

enum fcs_deserialization_result_t fcs_comms_deserialize_calibration(
struct fcs_packet_calibration_t *restrict calibration, uint8_t *restrict buf,
size_t len) {
    assert(calibration);
    assert(buf);
    assert(FCS_COMMS_MIN_PACKET_SIZE <= len && len < 256u);

    uint8_t field = 0,
            checksum = 'P' ^ 'S' ^ 'F' ^ 'W' ^ 'A' ^ 'L' ^ ',';
    size_t idx = 8u;
    enum fcs_conversion_result_t result;

    if (memcmp(buf, "$PSFWAL,", 8u) != 0) {
        goto invalid;
    }

    /*
    Loop over the buffer and update the checksum. Every time a comma is
    encountered, process the field corresponding to the preceding values.

    Stop once we've processed all the fields, hit the end of the buffer, or
    hit the checksum marker ('*').
    */
    for (field = 0; field < 24u && idx < len && buf[idx] != '*'; field++) {
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
                /* Sensor type -- one character */
                if (field_len != 1u) {
                    goto invalid;
                }

                calibration->sensor_type = buf[field_start] - 'A';
                result = FCS_CONVERSION_OK;
                break;
            case 1u:
                /* Sensor ID -- one character */
                if (field_len != 1u) {
                    goto invalid;
                }

                calibration->sensor_id = buf[field_start] - '0';
                result = FCS_CONVERSION_OK;
                break;
            case 2u:
                /* Flags -- 5 characters */
                if (field_len != 5u) {
                    goto invalid;
                }

                memcpy(calibration->flags, &buf[field_start], field_len);
                result = FCS_CONVERSION_OK;
                break;
            case 3u:
                /* Calibration type -- one character */
                if (field_len != 1u) {
                    goto invalid;
                }

                calibration->calibration_type = buf[field_start] - 'a';
                result = FCS_CONVERSION_OK;
                break;
            case 23u:
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
            default:
                /*
                Handle fields 4-22 -- just a bunch of double conversions, and
                order is the same between the calibration structure and the
                message
                */
                assert(4u <= field && field <= 22u);
                double *data_ptr = &calibration->error;
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
    if (field != 3u || idx != len - 4u || buf[idx - 1u] != '*' ||
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
    if (fcs_comms_validate_calibration(calibration) != FCS_VALIDATION_OK) {
        goto invalid;
    }

    return FCS_DESERIALIZATION_OK;

invalid:
    memset(calibration, 0xFFu, sizeof(struct fcs_packet_calibration_t));
    return FCS_DESERIALIZATION_ERROR;
}

enum fcs_validation_result_t fcs_comms_validate_calibration(
const struct fcs_packet_calibration_t *restrict calibration) {
    assert(calibration);

    if (calibration->sensor_type < (uint8_t)FCS_MEASUREMENT_TYPE_LAST &&
        calibration->sensor_id <= FCS_MEASUREMENT_SENSOR_ID_MAX &&
        calibration->calibration_type < (uint8_t)FCS_CALIBRATION_LAST &&
        0.0 < calibration->error && calibration->error <= FLT_MAX) {
        return FCS_VALIDATION_OK;
    } else {
        return FCS_VALIDATION_ERROR;
    }
}
