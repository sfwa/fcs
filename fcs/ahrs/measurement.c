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
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "measurement.h"
#include "ahrs.h"

/* Internal API for making and reading fields of various types */
static inline enum fcs_measurement_type_t _fcs_extract_sensor_type(
const uint8_t *buf) {
    assert(buf);

    enum fcs_measurement_type_t type;
    type = (enum fcs_measurement_type_t)(
        (buf[1] & FCS_MEASUREMENT_SENSOR_TYPE_MASK)
        >> FCS_MEASUREMENT_SENSOR_TYPE_OFFSET
    );

    assert(type < FCS_MEASUREMENT_TYPE_LAST);

    return type;
}

static inline uint8_t _fcs_extract_sensor_id(const uint8_t *buf) {
    assert(buf);

    uint8_t sensor_id;
    sensor_id = (
        (buf[1] & FCS_MEASUREMENT_SENSOR_ID_MASK)
        >> FCS_MEASUREMENT_SENSOR_ID_OFFSET
    );

    assert(sensor_id <= FCS_MEASUREMENT_SENSOR_ID_MAX);

    return sensor_id;
}

static inline size_t _fcs_extract_num_values(const uint8_t *buf) {
    assert(buf);

    size_t num_values;
    num_values = (
        (buf[0] & FCS_MEASUREMENT_HEADER_NUM_VALUES_MASK)
        >> FCS_MEASUREMENT_HEADER_NUM_VALUES_OFFSET
    ) + 1u;

    assert(num_values <= FCS_MEASUREMENT_NUM_VALUES_MAX);

    return num_values;
}

static inline size_t _fcs_extract_precision_bits(const uint8_t *buf) {
    assert(buf);

    size_t precision_bits;
    precision_bits = ((
        (buf[0] & FCS_MEASUREMENT_HEADER_PRECISION_BITS_MASK)
        >> FCS_MEASUREMENT_HEADER_PRECISION_BITS_OFFSET
    ) + 1u) << 2u;

    assert(precision_bits <= FCS_MEASUREMENT_PRECISION_BITS_MAX);

    return precision_bits;
}

static inline size_t _fcs_extract_measurement_length(const uint8_t *buf) {
    assert(buf);

    size_t num_values, precision_bits, length;
    num_values = _fcs_extract_num_values(buf);
    precision_bits = _fcs_extract_precision_bits(buf);

    /*
    Header length + num values * bytes required to contain precision_bits
    */
    length = 2u + num_values * ((precision_bits + 7u) >> 3u);
    assert(length <= 16u);

    return length;
}

static inline uint8_t _fcs_make_measurement_header(size_t precision_bits,
size_t num_values) {
    assert(precision_bits <= FCS_MEASUREMENT_PRECISION_BITS_MAX);
    assert(num_values <= FCS_MEASUREMENT_NUM_VALUES_MAX);

    size_t length;
    length = 2u + num_values * ((precision_bits + 7u) >> 3u);
    assert(length <= 16u);

    precision_bits >>= 2u;
    precision_bits -= 1u;

    num_values -= 1u;

    return
        ((precision_bits << FCS_MEASUREMENT_HEADER_PRECISION_BITS_OFFSET) &
         FCS_MEASUREMENT_HEADER_PRECISION_BITS_MASK) |
        ((num_values << FCS_MEASUREMENT_HEADER_NUM_VALUES_OFFSET)
         & FCS_MEASUREMENT_HEADER_NUM_VALUES_MASK);
}

static inline uint8_t _fcs_make_measurement_sensor(uint8_t sensor_id,
enum fcs_measurement_type_t type) {
    assert(sensor_id <= FCS_MEASUREMENT_SENSOR_ID_MAX);
    assert(type < FCS_MEASUREMENT_TYPE_LAST);

    return
        ((sensor_id << FCS_MEASUREMENT_SENSOR_ID_OFFSET) &
         FCS_MEASUREMENT_SENSOR_ID_MASK) |
        ((type << FCS_MEASUREMENT_SENSOR_TYPE_OFFSET)
         & FCS_MEASUREMENT_SENSOR_TYPE_MASK);
}

/* Public, typed wrappers for the above functions */
enum fcs_measurement_type_t fcs_measurement_get_sensor_type(
const struct fcs_measurement_t *measurement) {
    return _fcs_extract_sensor_type((const uint8_t*)measurement);
}

uint8_t fcs_measurement_get_sensor_id(
const struct fcs_measurement_t *restrict measurement) {
    return _fcs_extract_sensor_id((const uint8_t*)measurement);
}

size_t fcs_measurement_get_num_values(
const struct fcs_measurement_t *restrict measurement) {
    return _fcs_extract_num_values((const uint8_t*)measurement);
}

size_t fcs_measurement_get_precision_bits(
const struct fcs_measurement_t *restrict measurement) {
    return _fcs_extract_precision_bits((const uint8_t*)measurement);
}

size_t fcs_measurement_get_length(
const struct fcs_measurement_t *restrict measurement) {
    return _fcs_extract_measurement_length((const uint8_t*)measurement);
}

void fcs_measurement_set_header(
struct fcs_measurement_t *restrict measurement, size_t precision_bits,
size_t num_values) {
    assert(measurement);
    measurement->header = _fcs_make_measurement_header(precision_bits,
                                                       num_values);
}

void fcs_measurement_set_sensor(
struct fcs_measurement_t *restrict measurement, uint8_t sensor_id,
enum fcs_measurement_type_t type) {
    assert(measurement);
    measurement->sensor = _fcs_make_measurement_sensor(sensor_id, type);
}

enum fcs_calibration_type_t fcs_calibration_get_type(
const struct fcs_calibration_t *restrict calibration) {
    assert(calibration);

    enum fcs_calibration_type_t type;
    type = (enum fcs_calibration_type_t)(
        (calibration->type & FCS_CALIBRATION_TYPE_MASK)
        >> FCS_CALIBRATION_TYPE_OFFSET
    );

    assert(type < FCS_CALIBRATION_LAST);

    return type;
}

/* Initialize a log packet with a packet index of `frame_id` */
void fcs_measurement_log_init(struct fcs_measurement_log_t *restrict mlog,
uint16_t frame_id) {
    assert(mlog);

    mlog->data[0] = FCS_MEASUREMENT_LOG_TYPE;
    mlog->data[1] = 0;
    mlog->data[2] = 0;
    mlog->data[3] = (frame_id & 0x00FFu);
    mlog->data[4] = (frame_id & 0xFF00u) >> 8u;
    mlog->length = 5u;
}

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `mlog` to include a CRC16SBP.
*/
size_t fcs_measurement_log_serialize(uint8_t *restrict out_buf,
size_t out_buf_length, struct fcs_measurement_log_t *restrict mlog) {
    assert(out_buf);
    assert(out_buf_length);
    assert(mlog);
    /* 2 bytes for CRC, 3 bytes for COBS-R + NUL start/end */
    assert(out_buf_length >= mlog->length + 2u + 3u);

    /* Calculate checksum and update the packet with the result */
    uint16_t crc = fcs_crc16_sbp(mlog->data, mlog->length, 0xFFFFu);
    mlog->data[mlog->length + 0] = (crc & 0x00FFu);
    mlog->data[mlog->length + 1u] = (crc & 0xFF00u) >> 8u;

    /* Write COBS-R encoded result to out_buf */
    struct fcs_cobsr_encode_result result;
    result = fcs_cobsr_encode(&out_buf[1], out_buf_length - 2u, mlog->data,
                              mlog->length + 2u);
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
bool fcs_measurement_log_add(struct fcs_measurement_log_t *restrict mlog,
struct fcs_measurement_t *restrict measurement) {
    assert(mlog);
    /* Call these to validate sensor type and ID before copying */
    fcs_measurement_get_sensor_type(measurement);
    fcs_measurement_get_sensor_id(measurement);

    /*
    If there's not enough space in the log record to save the value, return
    false. 250 bytes to allow space for the CRC16, the COBS-R encoding and
    two NUL bytes within a 256-byte packet.
    */
    size_t length = fcs_measurement_get_length(measurement);
    if (mlog->length + length > 250u) {
        return false;
    }

    memcpy(&mlog->data[mlog->length], measurement, length);
    mlog->length += length;

    return true;
}

/*
Finds a measurement with a given ID and type in the log, and copies the result
to `out_measurement`.

Returns true if a measurement with matching ID and type was found, and false
if not.
*/
bool fcs_measurement_log_find(
const struct fcs_measurement_log_t *restrict mlog,
enum fcs_measurement_type_t type, uint8_t measurement_id,
struct fcs_measurement_t *restrict out_measurement) {
    assert(mlog);
    assert(5u <= mlog->length && mlog->length <= 256u);
    assert(out_measurement);

    uint8_t search_key;
    size_t i, measurement_length;

    search_key = _fcs_make_measurement_sensor(measurement_id, type);

    for (i = 5u; i < mlog->length;) {
        measurement_length = _fcs_extract_measurement_length(&mlog->data[i]);
        if (mlog->data[i + 1u] == search_key) {
            memcpy(out_measurement, &mlog->data[i], measurement_length);
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

Raw readings are scaled by `prescale` prior to calibration parameters being
applied. This can be used for variable-range sensors, or for sensors measuring
time-varying (but theoretically known) fields, like magnetometers. If
prescaling is not required, pass 1.0 for this value.

If the sensor calibration includes an offset component and `out_offset` is not
NULL, the offsets of the sensors are averaged based on the same weighting
factor as the measurement, and the resulting offset is copied into
`out_offset`.

Returns the number of raw measurements included in the output.
*/
size_t fcs_measurement_log_get_calibrated_value(
const struct fcs_measurement_log_t *restrict mlog,
const struct fcs_calibration_map_t *restrict cmap,
enum fcs_measurement_type_t type, double out_value[4], double *out_error,
double *out_offset, double prescale) {
    assert(mlog);
    assert(5u <= mlog->length && mlog->length <= 256u);
    assert(cmap);
    assert(out_value);
    assert(out_error);
    assert(out_value != out_error);
    assert(out_value != out_offset);
    assert(out_error != out_offset);
    _nassert((size_t)out_value % 8 == 0);
    _nassert((size_t)out_error % 8 == 0);
    _nassert((size_t)out_offset % 8 == 0);

    double accum_value[4], accum_error, accum_offset[3];
    double temp_value[4], temp_error, temp_offset[3];
    size_t i, j, n_measurements, measurement_length, measurement_type;
    struct fcs_measurement_t measurement;

    vector_set_d(accum_value, 0, 4u);
    vector_set_d(accum_offset, 0, 3u);
    accum_error = 0.0;

    /* Start scanning at index 5, first byte after the log record header */
    for (i = 5u, n_measurements = 0; i < mlog->length;
            i += measurement_length) {
        measurement_length = _fcs_extract_measurement_length(&mlog->data[i]);
        measurement_type = _fcs_extract_sensor_type(&mlog->data[i]);

        if (measurement_type != type) {
            continue;
        }

        /* Copy the measurement data to an actual measurement structure */
        memcpy(&measurement, &mlog->data[i], measurement_length);

        /* Process the reading and accumulate the output */
        fcs_measurement_calibrate(&measurement, cmap, temp_value, &temp_error,
                                  temp_offset, prescale);

        for (j = 0; j < 3u; j++) {
            accum_value[j] += temp_value[j];
            accum_offset[j] += temp_offset[j];
        }
        accum_value[3] += temp_value[3];
        accum_error += temp_error;

        n_measurements++;
    }

    if (n_measurements > 0) {
        double scale;

        /* Get the mean of the results */
        scale = 1.0 / (double)n_measurements;
        for (i = 0; i < 3u; i++) {
            accum_value[i] *= scale;
            accum_offset[i] *= scale;
        }
        accum_value[3] *= scale;

        if (out_offset) {
            vector_copy_d(out_offset, accum_offset, 3u);
        }
        vector_copy_d(out_value, accum_value, 4u);
        *out_error = accum_error * scale;
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
    _nassert((size_t)out_value % 8 == 0);

    vector_set_d(out_value, 0, 4u);

    enum fcs_measurement_type_t type =
        fcs_measurement_get_sensor_type(measurement);
    uint8_t precision = fcs_measurement_get_precision_bits(measurement);
    double scale = 1.0 / (double)(1u << (precision - 1u));
    size_t n = fcs_measurement_get_num_values(measurement), i;

    if (type == FCS_MEASUREMENT_TYPE_PRESSURE_TEMP) {
        /* Special-cased due to a mix of signed and unsigned values */
        out_value[0] = (double)measurement->data.u16[0] * scale * 0.5;
        out_value[1] = (double)measurement->data.i16[1] * scale;
    } else if (type == FCS_MEASUREMENT_TYPE_GPS_POSITION) {
        /* Special-cased due to fixed scaling and higher precision */
        out_value[0] = measurement->data.i32[0] * 1e-7 * (M_PI/180.0);
        out_value[1] = measurement->data.i32[1] * 1e-7 * (M_PI/180.0);
        out_value[2] = measurement->data.i32[2] * 1e-3;
    } else if (type == FCS_MEASUREMENT_TYPE_GPS_INFO) {
        /* Special-cased due to packed values in first byte */
        n = 3u;
        out_value[0] = measurement->data.u8[0] >> 4u;
        out_value[1] = measurement->data.u8[0] & 0xFu;
        out_value[2] = measurement->data.u8[1];
    } else if (precision <= 8u) {
        /* Handle 1-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = (double)measurement->data.i8[i] * scale;
        }
    } else if (precision <= 16u) {
        /* Handle 2-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = (double)measurement->data.i16[i] * scale;
        }
    } else if (precision <= 32u) {
        /* Handle 4-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = (double)measurement->data.i32[i] * scale;
        }
    } else {
        assert(false);
    }

    return n;
}

/*
Calibrate a single measurement based on the calibration map parameters, with
sensor readings scaled by `prescale` prior to calibration parameters being
applied.
*/
void fcs_measurement_calibrate(
const struct fcs_measurement_t *restrict measurement,
const struct fcs_calibration_map_t *restrict cmap, double out_value[4],
double *out_error, double out_offset[3], double prescale) {
    assert(measurement);
    assert(cmap);
    assert(out_value);
    assert(out_error);
    assert(out_value != out_error);
    assert(out_value != out_offset);
    assert(out_error != out_offset);
    _nassert((size_t)out_value % 8 == 0);
    _nassert((size_t)out_error % 8 == 0);
    _nassert((size_t)out_offset % 8 == 0);

    size_t i;
    double temp_value[4], c[3], orientation[4];
    fcs_measurement_get_values(measurement, temp_value);

    /*
    Look up the calibration entry based on the combination of sensor ID and
    type
    */
    uint8_t sensor_key = measurement->sensor &
        (FCS_MEASUREMENT_SENSOR_ID_MASK | FCS_MEASUREMENT_SENSOR_TYPE_MASK);
    const struct fcs_calibration_t *calibration =
        &cmap->sensor_calibration[sensor_key];
    const float *restrict p = calibration->params;

    assert(calibration->header || calibration->sensor);

    /* Apply prescaling and scale factor */
    prescale *= calibration->scale_factor;
    for (i = 0; i < 4u; i++) {
        temp_value[i] *= prescale;
    }

    /* Set error and offset based on calibration values */
    *out_error = calibration->error;
    vector_d_from_f(out_offset, calibration->offset, 3u);

    /* Run the appropriate calibration routine */
    switch (fcs_calibration_get_type(calibration)) {
        case FCS_CALIBRATION_NONE:
            break;
        case FCS_CALIBRATION_BIAS_SCALE_1D:
            temp_value[0] -= p[0];
            temp_value[0] *= p[1];
            break;
        case FCS_CALIBRATION_BIAS_SCALE_3X3:
            /*
            Implements
            B' = (I_{3x3} + D)B - b

            where B' is the calibrated measurement, I_{3x3} is the 3x3
            identity matrix, D is the scale calibration matrix, B is the
            raw measurement, and b is the bias vector.

            This is exactly the same as the equivalent TRICAL function
            (and for magnetometers we use the TRICAL calibration state
            estimate directly).
            */
            c[0] = temp_value[0] - p[0];
            c[1] = temp_value[1] - p[1];
            c[2] = temp_value[2] - p[2];

            /* Symmetric matrix multiply */
            temp_value[0] = c[0] * (p[3] + 1.0) + c[1] * p[4] + c[2] * p[5];
            temp_value[1] = c[0] * p[6] + c[1] * (p[7] + 1.0) + c[2] * p[8];
            temp_value[2] = c[0] * p[9] + c[1] * p[10] + c[2] * (p[11] + 1.0);
            break;
        case FCS_CALIBRATION_BIAS_SCALE_PITOT:
            /*
            FIXME: this is really hacky, clean it up.

            Convert pitot reading to kPa -- 0.2533 = 0 kPa, 0 = 2 kPa
            From the datasheet, Vout = Vs * (0.2 * P + 0.5),
            where Vs is 3.3V. At Vout = 1.65V (P = 0), the reading is
            0.2533; at Vout = 3.3V, the reading is 0.
            Vout = -6.514 * Rd + 3.3;
            P = (Vout / Vs - 0.5) * 5
            IAS = sqrt(2 * P / 1.225)
            Thus:
            IAS = sqrt(2 * (((-6.514 * Rd + 3.3) / 3.3 - 0.5) * 5) / 1.225)

            p[0] is the 0-pressure reading (0.2533); p[1] is the ADC scale
            factor (-6.514).
            */
            if (temp_value[0] >= p[0]) {
                temp_value[0] = 0.0;
            } else {
                temp_value[0] = sqrt(
                    2.0 * ((p[1] * temp_value[0] + 3.3) / 3.3 - 0.5) *
                    5000 / 1.225
                );
            }
        default:
            /* Invalid calibration type */
            assert(false);
            break;
    }

    if (calibration->type & FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION) {
        /* Transform calibrated value based on sensor orientation */
        vector_d_from_f(orientation, calibration->orientation, 4u);
        quaternion_vector3_multiply_d(out_value, orientation, temp_value);
    } else {
        vector_copy_d(out_value, temp_value, 4u);
    }
}
