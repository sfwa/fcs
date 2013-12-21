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
#include "../util/3dmath.h"
#include "../util/util.h"
#include "../drivers/stream.h"
#include "../comms/comms.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "measurement.h"
#include "ahrs.h"

enum fcs_measurement_type_t fcs_measurement_get_sensor_type(
const struct fcs_measurement_t *measurement) {
    assert(measurement);

    enum fcs_measurement_type_t type;
    type = (enum fcs_measurement_type_t)(
        (measurement->sensor & FCS_MEASUREMENT_SENSOR_TYPE_MASK)
        >> FCS_MEASUREMENT_SENSOR_TYPE_OFFSET
    );

    assert(type >= FCS_MEASUREMENT_TYPE_INVALID);
    assert(type < FCS_MEASUREMENT_TYPE_LAST);

    return type;
}

uint8_t fcs_measurement_get_sensor_id(
const struct fcs_measurement_t *restrict measurement) {
    assert(measurement);

    uint8_t sensor_id;
    sensor_id = (
        (measurement->sensor & FCS_MEASUREMENT_SENSOR_ID_MASK)
        >> FCS_MEASUREMENT_SENSOR_ID_OFFSET
    );

    assert(sensor_id <= FCS_MEASUREMENT_SENSOR_ID_MAX);

    return sensor_id;
}

size_t fcs_measurement_get_length(
const struct fcs_measurement_t *restrict measurement) {
    assert(measurement);

    size_t length;
    length = (
        (measurement->header & FCS_MEASUREMENT_HEADER_LENGTH_MASK)
        >> FCS_MEASUREMENT_HEADER_LENGTH_OFFSET
    );

    assert(length);
    assert(length <= FCS_MEASUREMENT_LENGTH_MAX);

    return length;
}

void fcs_measurement_set_sensor(
struct fcs_measurement_t *restrict measurement, uint8_t sensor_id,
enum fcs_measurement_type_t type) {
    assert(measurement);
    assert(sensor_id < FCS_MEASUREMENT_SENSOR_ID_MAX);
    assert(type >= FCS_MEASUREMENT_TYPE_INVALID);
    assert(type < FCS_MEASUREMENT_TYPE_LAST);

    measurement->sensor =
        ((sensor_id << FCS_MEASUREMENT_SENSOR_ID_OFFSET) &
         FCS_MEASUREMENT_SENSOR_ID_MASK) |
        ((type << FCS_MEASUREMENT_SENSOR_TYPE_OFFSET)
         & FCS_MEASUREMENT_SENSOR_TYPE_MASK);
}

enum fcs_calibration_type_t fcs_calibration_get_type(
const struct fcs_calibration_t *restrict calibration) {
    assert(calibration);

    enum fcs_calibration_type_t type;
    type = (enum fcs_calibration_type_t)(
        (calibration->type & FCS_CALIBRATION_TYPE_MASK)
        >> FCS_CALIBRATION_TYPE_OFFSET
    );

    assert(type >= FCS_CALIBRATION_NONE);
    assert(type < FCS_CALIBRATION_LAST);

    return type;
}

/* Initialize a log packet with a packet index of `frame_id` */
void fcs_measurement_log_init(struct fcs_measurement_log_t *restrict log_rec,
uint16_t frame_id) {
    assert(log_rec);

    log_rec->data[0] = FCS_MEASUREMENT_LOG_TYPE;
    log_rec->data[1] = 0;
    log_rec->data[2] = 0;
    log_rec->data[3] = (frame_id & 0x00FFu);
    log_rec->data[4] = (frame_id & 0xFF00u) >> 8u;
    log_rec->length = 5u;
}

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log_rec` to include a CRC16SBP.
*/
size_t fcs_measurement_log_serialize(uint8_t *restrict out_buf,
size_t out_buf_length, struct fcs_measurement_log_t *restrict log_rec) {
    assert(out_buf);
    assert(out_buf_length);
    assert(log_rec);
    /* 2 bytes for CRC, 3 bytes for COBS-R + NUL start/end */
    assert(out_buf_length >= log_rec->length + 2u + 3u);

    /* Calculate checksum and update the packet with the result */
    uint16_t crc = fcs_crc16_sbp(log_rec->data, log_rec->length, 0xFFFFu);
    log_rec->data[log_rec->length + 0] = (crc & 0x00FFu);
    log_rec->data[log_rec->length + 1u] = (crc & 0xFF00u) >> 8u;

    /* Write COBS-R encoded result to out_buf */
    struct fcs_cobsr_encode_result result;
    result = fcs_cobsr_encode(&out_buf[1], out_buf_length - 2u, log_rec->data,
                              log_rec->length + 2u);
    assert(result.status == FCS_COBSR_ENCODE_OK);

    /* Add NUL start/end bytes */
    out_buf[0] = 0;
    out_buf[result.out_len + 1u] = 0;

    /* Return the COBS-R encoded length, plus the length of the NUL bytes */
    return result.out_len + 2u;
}

/*
Add a sensor value entry to a log packet. Returns true if the sensor value
could be added, or false if it couldn't.
*/
bool fcs_measurement_log_add(struct fcs_measurement_log_t *restrict log_rec,
struct fcs_measurement_t *restrict measurement) {
    assert(log_rec);
    /* Call these to validate sensor type and ID before copying */
    fcs_measurement_get_sensor_type(measurement);
    fcs_measurement_get_sensor_id(measurement);

    /*
    Length + 2 because the length field doesn't include the measurement header
    or sensor info
    */
    size_t length = fcs_measurement_get_length(measurement) + 2u;

    /*
    If there's not enough space in the log record to save the value, return
    false. 250 bytes to allow space for the CRC16, the COBS-R encoding and
    two NUL bytes within a 256-byte packet.
    */
    if (log_rec->length + length > 250u) {
        return false;
    }

    memcpy(&log_rec->data[log_rec->length], measurement, length);
    log_rec->length += length;

    return true;
}

/*
Finds a measurement with a given ID and type in the log, and copies the result
to `out_measurement`.

Returns true if a measurement with matching ID and type was found, and false
if not.
*/
bool fcs_measurement_log_find(
const struct fcs_measurement_log_t *restrict log_rec,
enum fcs_measurement_type_t type, uint8_t measurement_id,
struct fcs_measurement_t *restrict out_measurement) {
    assert(log_rec);
    assert(5u <= log_rec->length && log_rec->length <= 256u);
    assert(out_measurement);

    uint8_t search_key;
    size_t i, measurement_length;

    search_key = (measurement_id << FCS_MEASUREMENT_SENSOR_ID_OFFSET) &
                 FCS_MEASUREMENT_SENSOR_ID_MASK;
    search_key |= ((uint8_t)type << FCS_MEASUREMENT_SENSOR_TYPE_OFFSET) &
                   FCS_MEASUREMENT_SENSOR_TYPE_MASK;

    for (i = 5u; i < log_rec->length;) {
        measurement_length =
            ((log_rec->data[i] & FCS_MEASUREMENT_HEADER_LENGTH_MASK)
             >> FCS_MEASUREMENT_HEADER_LENGTH_OFFSET) + 2u;

        if (log_rec->data[i + 1u] == search_key) {
            memcpy(out_measurement, &log_rec->data[i], measurement_length);
            return true;
        }

        i += measurement_length;
    }

    return false;
}

/*
Retrieve a calibrated measurement for a given sensor. If multiple measurements
are available, they are individually calibrated and then averaged based on the
error of each sensor.

If the sensor calibration includes an offset component and `out_offset` is not
NULL, the offsets of the sensors are averaged based on the same weighting
factor as the measurement, and the resulting offset is copied into
`out_offset`.

Returns the number of raw measurements included in the output.
*/
size_t fcs_measurement_log_get_calibrated_value(
const struct fcs_measurement_log_t *restrict log_rec,
const struct fcs_calibration_map_t *restrict calibration_map,
enum fcs_measurement_type_t type, double out_value[4], double *out_error,
double out_offset[3]) {
    assert(log_rec);
    assert(5u <= log_rec->length && log_rec->length <= 256u);
    assert(calibration_map);
    assert(out_value);
    assert(out_error);
    assert(out_value != out_error);
    assert(out_value != out_offset);
    assert(out_error != out_offset);

    double accum_value[4], accum_error, accum_offset[3];
    double temp_value[4], temp_error, temp_offset[3];
    size_t i, n_measurements, measurement_length, measurement_type;

    /* Start scanning at index 5, first byte after the log record header */
    for (i = 5u, n_measurements = 0; i < log_rec->length;) {
        measurement_length =
            ((log_rec->data[i] & FCS_MEASUREMENT_HEADER_LENGTH_MASK)
             >> FCS_MEASUREMENT_HEADER_LENGTH_OFFSET) + 2u;
        measurement_type =
            (log_rec->data[i + 1u] & FCS_MEASUREMENT_SENSOR_TYPE_MASK)
             >> FCS_MEASUREMENT_SENSOR_TYPE_OFFSET;

        if (measurement_type == type) {
            struct fcs_measurement_t measurement;
            /* Copy the measurement data to an actual measurement structure */
            memcpy(&measurement, &log_rec->data[i], measurement_length);
            /* Process the reading and accumulate the output */
            fcs_measurement_calibrate(&measurement, calibration_map,
                                      temp_value, &temp_error, temp_offset);

            accum_value[0] += temp_value[0];
            accum_value[1] += temp_value[1];
            accum_value[2] += temp_value[2];
            accum_value[3] += temp_value[3];
            accum_error += temp_error;
            accum_offset[0] += temp_offset[0];
            accum_offset[1] += temp_offset[1];
            accum_offset[2] += temp_offset[2];

            n_measurements++;
        }

        i += measurement_length;
    }

    if (n_measurements > 0) {
        double scale;

        /* Get the mean of the results */
        scale = 1.0 / (double)n_measurements;
        out_value[0] = accum_value[0] * scale;
        out_value[1] = accum_value[1] * scale;
        out_value[2] = accum_value[2] * scale;
        out_value[3] = accum_value[3] * scale;
        *out_error = accum_error * scale;

        if (out_offset) {
            out_offset[0] = accum_offset[0] * scale;
            out_offset[1] = accum_offset[1] * scale;
            out_offset[2] = accum_offset[2] * scale;
        }
    }

    return n_measurements;
}

/*
Convert the values associated with a measurement into an array of doubles.
Returns the number of values in the measurement.
*/
size_t fcs_measurement_get_values(
const struct fcs_measurement_t *restrict measurement, double out_value[4]) {
    assert(measurement);
    assert(out_value);

    size_t n;
    memset(out_value, 0, sizeof(double) * 4u);

    /* Convert the raw sensor data to floating point */
    switch (fcs_measurement_get_sensor_type(measurement)) {
        case FCS_MEASUREMENT_TYPE_CONTROL_POS:
            /* 4x 16-bit signed values */
            n = 4u;
            out_value[3] = measurement->data.i16[3];
        case FCS_MEASUREMENT_TYPE_ACCELEROMETER:
        case FCS_MEASUREMENT_TYPE_GYROSCOPE:
        case FCS_MEASUREMENT_TYPE_MAGNETOMETER:
        case FCS_MEASUREMENT_TYPE_GPS_VELOCITY:
            /* 3x 16-bit signed values */
            n = 3u;
            out_value[2] = measurement->data.i16[2];
        case FCS_MEASUREMENT_TYPE_PRESSURE_TEMP:
        case FCS_MEASUREMENT_TYPE_IV:
            /* 2x 16-bit signed values */
            n = 2u;
            out_value[1] = measurement->data.i16[1];
        case FCS_MEASUREMENT_TYPE_PITOT:
        case FCS_MEASUREMENT_TYPE_RANGEFINDER:
            /* 1x 16-bit signed values */
            n = 1u;
            out_value[0] = measurement->data.i16[0];
            break;
        case FCS_MEASUREMENT_TYPE_GPS_POSITION:
            /* 3x 32-bit signed, with preset scaling */
            n = 3u;
            out_value[0] = measurement->data.i32[0] * 1e-7 * (M_PI/180.0);
            out_value[1] = measurement->data.i32[1] * 1e-7 * (M_PI/180.0);
            out_value[2] = measurement->data.i32[2] * 1e-2;
            break;
        case FCS_MEASUREMENT_TYPE_GPS_INFO:
            /* byte 0 7:4 fix mode, byte 0 3:0 num SVs, byte 1 dop */
            n = 3u;
            out_value[0] = measurement->data.u8[0] >> 4u;
            out_value[1] = measurement->data.u8[0] & 0xFu;
            out_value[2] = measurement->data.u8[1];
        case FCS_MEASUREMENT_TYPE_RADIO:
            /* 4x 8-bit signed */
            n = 4u;
            out_value[0] = measurement->data.i8[0];
            out_value[1] = measurement->data.i8[1];
            out_value[2] = measurement->data.i8[2];
            out_value[3] = measurement->data.i8[3];
            break;
        default:
            /*
            Invalid measurement type, or one that can't meaningfully be
            calibrated
            */
            assert(false);
            break;
    }

    return n;
}

/*
Calibrate a single measurement based on the calibration map parameters.
*/
void fcs_measurement_calibrate(
const struct fcs_measurement_t *restrict measurement,
const struct fcs_calibration_map_t *restrict calibration_map,
double out_value[4], double *out_error, double out_offset[3]) {
    assert(measurement);
    assert(calibration_map);
    assert(out_value);
    assert(out_error);
    assert(out_value != out_error);
    assert(out_value != out_offset);
    assert(out_error != out_offset);

    double temp_value[4];
    fcs_measurement_get_values(measurement, temp_value);

    /*
    Look up the calibration entry based on the combination of sensor ID and
    type
    */
    uint8_t sensor_key = measurement->sensor &
        (FCS_MEASUREMENT_SENSOR_ID_MASK | FCS_MEASUREMENT_SENSOR_TYPE_MASK);
    const struct fcs_calibration_t *calibration =
        &calibration_map->sensor_calibration[sensor_key];

    if (!calibration->header || !calibration->sensor) {
        /*
        header or sensor = 0 means there is no calibration for this sensor --
        return the raw measurement values
        */
        *out_error = 0.0;
        out_offset[0] = out_offset[1] = out_offset[2] = 0.0;
        memcpy(out_value, temp_value, sizeof(double) * 4u);
    } else {
        /* Set error and offset based on calibration values */
        *out_error = calibration->error;
        out_offset[0] = calibration->offset[0];
        out_offset[1] = calibration->offset[1];
        out_offset[2] = calibration->offset[2];

        /* Run the appropriate calibration routine */
        uint8_t i;
        const float *restrict p = calibration->params;
        double c[3]; /* centered value */

        switch (fcs_calibration_get_type(calibration)) {
            case FCS_CALIBRATION_NONE:
                break;
            case FCS_CALIBRATION_BIAS_SCALE_1D:
                temp_value[0] -= p[0];
                temp_value[0] *= p[1];
                break;
            case FCS_CALIBRATION_BIAS_SCALE_3D:
                for (i = 0; i < 3u; i++) {
                    temp_value[i] -= p[i];
                    temp_value[i] *= p[i + 3u];
                }
                break;
            case FCS_CALIBRATION_BIAS_SCALE_3X3:
                /*
                Implements
                B' = DB - b

                where B' is the calibrated measurement, D is the (symmetric)
                scale calibration matrix, B is the raw measurement, and b is
                the bias vector.

                We only store D's upper triangle values in the calibration
                params, like this:
                3  4  5
                   6  7
                      8

                This is based on the TRICAL calibration code, except in this
                case the identity matrix is not added in to D before
                multiplying it by B.
                */
                c[0] = temp_value[0] - p[0];
                c[1] = temp_value[1] - p[1];
                c[2] = temp_value[2] - p[2];

                /* Symmetric matrix multiply */
                temp_value[0] = c[0] * p[3] + c[1] * p[4] + c[2] * p[5];
                temp_value[1] = c[0] * p[4] + c[1] * p[6] + c[2] * p[7];
                temp_value[2] = c[0] * p[5] + c[1] * p[7] + c[2] * p[8];
                break;
            default:
                /* Invalid calibration type */
                assert(false);
                break;
        }

        if (calibration->type & FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION) {
            /* Transform calibrated value based on sensor orientation */
            double orientation[4];
            orientation[0] = calibration->orientation[0];
            orientation[1] = calibration->orientation[1];
            orientation[2] = calibration->orientation[2];
            orientation[3] = calibration->orientation[3];
            quaternion_vector3_multiply_d(
                out_value, orientation, temp_value);
            out_value[3] = 0.0;
        } else {
            out_value[0] = temp_value[0];
            out_value[1] = temp_value[1];
            out_value[2] = temp_value[2];
            out_value[3] = temp_value[3];
        }
    }
}
