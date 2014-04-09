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

#include "parameter.h"
#include "../util/3dmath.h"

/* Internal API for making and reading fields of various types */
static inline size_t _extract_num_values(uint8_t header) {
    size_t num_values;

    if (header & FCS_PARAMETER_HEADER_MODE_MASK) {
        num_values = (
            (header & FCS_PARAMETER_HEADER_DATA_LENGTH_MASK)
            >> FCS_PARAMETER_HEADER_DATA_LENGTH_OFFSET
        );

        if (num_values > FCS_PARAMETER_DATA_LENGTH_MAX) {
            num_values = 0;
        }
    } else {
        num_values = (
            (header & FCS_PARAMETER_HEADER_NUM_VALUES_MASK)
            >> FCS_PARAMETER_HEADER_NUM_VALUES_OFFSET
        ) + 1u;

        if (num_values > FCS_PARAMETER_NUM_VALUES_MAX) {
            num_values = 0;
        }
    }

    return num_values;
}

static inline size_t _extract_precision_bits(uint8_t header) {
    size_t precision_bits;

    if (header & FCS_PARAMETER_HEADER_MODE_MASK) {
        return 8u;
    } else {
        precision_bits = ((
            (header & FCS_PARAMETER_HEADER_PRECISION_BITS_MASK)
            >> FCS_PARAMETER_HEADER_PRECISION_BITS_OFFSET
        ) + 1u) << 2u;

        if (precision_bits > FCS_PARAMETER_PRECISION_BITS_MAX) {
            precision_bits = 0;
        }
    }

    return precision_bits;
}

static inline size_t _extract_length(uint8_t header) {
    size_t num_values, precision_bits, length;

    num_values = _extract_num_values(header);
    precision_bits = _extract_precision_bits(header);

    if (!num_values || !precision_bits) {
        length = 0;
    } else {
        /*
        Header length + num values * bytes required to contain precision_bits
        */
        length = 3u + num_values * ((precision_bits + 7u) >> 3u);
    }

    return length;
}

static inline uint8_t _make_parameter_header(size_t precision_bits,
size_t num_values) {
    assert(precision_bits <= FCS_PARAMETER_PRECISION_BITS_MAX);
    assert(num_values <= FCS_PARAMETER_NUM_VALUES_MAX);

    size_t length;
    length = 3u + num_values * ((precision_bits + 7u) >> 3u);
    assert(length <= 16u);

    precision_bits >>= 2u;
    precision_bits -= 1u;

    num_values -= 1u;

    return
        ((precision_bits << FCS_PARAMETER_HEADER_PRECISION_BITS_OFFSET) &
         FCS_PARAMETER_HEADER_PRECISION_BITS_MASK) |
        ((num_values << FCS_PARAMETER_HEADER_NUM_VALUES_OFFSET)
         & FCS_PARAMETER_HEADER_NUM_VALUES_MASK);
}

static inline bool _validate_parameter(
const struct fcs_parameter_t *parameter) {
    bool valid = true;
    if (!parameter) {
        valid = false;
    } else if (!_extract_num_values(parameter->header)) {
        valid = false;
    } else if (!_extract_precision_bits(parameter->header)) {
        valid = false;
    } else if (parameter->type == FCS_PARAMETER_INVALID) {
        valid = false;
    } else if (parameter->type >= FCS_PARAMETER_LAST) {
        valid = false;
    }
    return valid;
}

/* Public, typed wrappers for the above functions */
enum fcs_parameter_type_t fcs_parameter_get_type(
const struct fcs_parameter_t *parameter) {
    assert(_validate_parameter(parameter));
    return (enum fcs_parameter_type_t)parameter->type;
}

uint8_t fcs_parameter_get_device_id(
const struct fcs_parameter_t *restrict parameter) {
    assert(_validate_parameter(parameter));
    return parameter->device;
}

size_t fcs_parameter_get_num_values(
const struct fcs_parameter_t *restrict parameter) {
    assert(_validate_parameter(parameter));
    return _extract_num_values(parameter->header);
}

size_t fcs_parameter_get_precision_bits(
const struct fcs_parameter_t *restrict parameter) {
    assert(_validate_parameter(parameter));
    return _extract_precision_bits(parameter->header);
}

size_t fcs_parameter_get_length(
const struct fcs_parameter_t *restrict parameter) {
    assert(_validate_parameter(parameter));
    return _extract_length(parameter->header);
}

void fcs_parameter_set_header(
struct fcs_parameter_t *restrict parameter, size_t precision_bits,
size_t num_values) {
    assert(_validate_parameter(parameter));
    parameter->header = _make_parameter_header(precision_bits, num_values);
}

void fcs_parameter_set_device(
struct fcs_parameter_t *restrict parameter, uint8_t device) {
    assert(_validate_parameter(parameter));
    parameter->device = device;
}

void fcs_parameter_set_type(
struct fcs_parameter_t *restrict parameter, enum fcs_parameter_type_t type) {
    assert(_validate_parameter(parameter));
    parameter->type = (uint8_t)type;
}

size_t fcs_parameter_get_key_value(uint8_t key[4], uint8_t *restrict value,
size_t value_length, const struct fcs_parameter_t *restrict parameter) {
    assert(_validate_parameter(parameter));
    assert(key);
    assert((value && value_length) || (!value && !value_length));

    size_t data_length = _extract_length(parameter->header);
    assert(data_length > 7u);
    data_length -= 7u;  /* header + key */

    key[0] = parameter->data.u8[0];
    key[1] = parameter->data.u8[1];
    key[2] = parameter->data.u8[2];
    key[3] = parameter->data.u8[3];

    if (value) {
        memcpy(value, &parameter->data.u8[4],
               data_length < value_length ? data_length : value_length);

        return data_length;
    } else {
        return 0;
    }
}

void fcs_parameter_set_key_value(struct fcs_parameter_t *restrict parameter,
uint8_t key[4], uint8_t *restrict value, size_t value_length) {
    assert(_validate_parameter(parameter));
    assert(key);
    assert(value);
    assert(value_length < FCS_PARAMETER_DATA_LENGTH_MAX - 4u);

    parameter->data.u8[0] = key[0];
    parameter->data.u8[1] = key[1];
    parameter->data.u8[2] = key[2];
    parameter->data.u8[3] = key[3];

    memcpy(&parameter->data.u8[4], value, value_length);

    parameter->header =
        (uint8_t)(FCS_PARAMETER_HEADER_MODE_MASK | value_length);
    parameter->type = FCS_PARAMETER_KEY_VALUE;
}

/*
Convert the values associated with a parameter into an array of doubles
normalized to the range [0.0, 1.0).

Returns the number of values in the parameter.
*/
size_t fcs_parameter_get_values_d(
const struct fcs_parameter_t *restrict parameter, double *restrict out_value,
size_t out_value_length) {
    assert(_validate_parameter(parameter));
    assert(out_value);
    _nassert((size_t)out_value % 8 == 0);

    enum fcs_parameter_type_t type = fcs_parameter_get_type(parameter);
    size_t precision = fcs_parameter_get_precision_bits(parameter);
    size_t n = fcs_parameter_get_num_values(parameter), i;

    if (n > out_value_length) {
        n = out_value_length;
    }

    if (type == FCS_PARAMETER_PRESSURE_TEMP && n == 2u) {
        /* Special-cased due to a mix of signed and unsigned values */
        out_value[0] = (double)parameter->data.u16[0];
        out_value[1] = (double)parameter->data.i16[1];
    } else if (type == FCS_PARAMETER_GPS_POSITION_LLA && n == 3u) {
        /* Special-cased due to fixed scaling and higher precision */
        out_value[0] = parameter->data.i32[0] * 1e-7 * (M_PI/180.0);
        out_value[1] = parameter->data.i32[1] * 1e-7 * (M_PI/180.0);
        out_value[2] = parameter->data.i32[2] * 1e-3;
    } else if (type == FCS_PARAMETER_GPS_INFO && n == 3u) {
        /* Special-cased due to packed values in first byte */
        out_value[0] = parameter->data.u8[0] >> 4u;
        out_value[1] = parameter->data.u8[0] & 0xFu;
        out_value[2] = parameter->data.u8[1];
    } else if (precision <= 8u) {
        /* Handle 1-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = (double)parameter->data.i8[i];
        }
    } else if (precision <= 16u) {
        /* Handle 2-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = (double)parameter->data.i16[i];
        }
    } else if (precision <= 32u) {
        /* Handle 4-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = (double)parameter->data.i32[i];
        }
    } else {
        assert(false);
    }

    return n;
}

size_t fcs_parameter_get_values_u32(
const struct fcs_parameter_t *restrict parameter,
uint32_t *restrict out_value, size_t out_value_length) {
    assert(_validate_parameter(parameter));
    assert(out_value);
    _nassert((size_t)out_value % 4 == 0);

    size_t precision = fcs_parameter_get_precision_bits(parameter);
    size_t n = fcs_parameter_get_num_values(parameter), i;

    if (n > out_value_length) {
        n = out_value_length;
    }

     if (precision <= 8u) {
        /* Handle 1-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = parameter->data.u8[i];
        }
    } else if (precision <= 16u) {
        /* Handle 2-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = parameter->data.u16[i];
        }
    } else if (precision <= 32u) {
        /* Handle 4-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = parameter->data.u32[i];
        }
    } else {
        assert(false);
    }

    return n;
}

size_t fcs_parameter_get_values_s32(
const struct fcs_parameter_t *restrict parameter, int32_t *restrict out_value,
size_t out_value_length) {
    assert(_validate_parameter(parameter));
    assert(out_value);
    _nassert((size_t)out_value % 4 == 0);

    size_t precision = fcs_parameter_get_precision_bits(parameter);
    size_t n = fcs_parameter_get_num_values(parameter), i;

    if (n > out_value_length) {
        n = out_value_length;
    }

     if (precision <= 8u) {
        /* Handle 1-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = parameter->data.i8[i];
        }
    } else if (precision <= 16u) {
        /* Handle 2-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = parameter->data.i16[i];
        }
    } else if (precision <= 32u) {
        /* Handle 4-byte values */
        for (i = 0; i < n; i++) {
            out_value[i] = parameter->data.i32[i];
        }
    } else {
        assert(false);
    }

    return n;
}

/*
Add a parameter entry to a log packet. Returns `true` if the parameter could
be added, or `false` if it couldn't.
*/
bool fcs_log_add_parameter(struct fcs_log_t *restrict plog,
struct fcs_parameter_t *restrict parameter) {
    assert(plog);
    assert(_validate_parameter(parameter));

    /*
    If there's not enough space in the log record to save the value, return
    false. Allow space for the CRC32, the COBS-R encoding and two NUL bytes
    within the packet.
    */
    size_t length = fcs_parameter_get_length(parameter);
    if (plog->length + length > FCS_LOG_LENGTH) {
        return false;
    }

    memcpy(&plog->data[plog->length], parameter, length);
    plog->length += length;

    return true;
}

/*
Set the device ID of all parameters in the log to `device_id`
*/
void fcs_log_set_parameter_device_id(struct fcs_log_t *restrict plog,
uint8_t device_id) {
    assert(plog);
    assert(plog->data[0] > (uint8_t)FCS_LOG_TYPE_INVALID);
    assert(plog->data[0] < (uint8_t)FCS_LOG_TYPE_LAST);

    size_t i, length = 0;

    for (i = 5u; i < plog->length; i += length) {
        length = _extract_length(plog->data[i]);
        plog->data[i + offsetof(struct fcs_parameter_t, device)] = device_id;
    }
}

/*
Finds a parameter with a given device and type in the log, and copies the
result to `out_parameter`.

Returns true if a parameter with matching device and type was found, and false
if not.
*/
bool fcs_parameter_find_by_type_and_device(
const struct fcs_log_t *restrict plog, enum fcs_parameter_type_t type,
uint8_t device_id, struct fcs_parameter_t *restrict out_parameter) {
    assert(plog);
    assert(5u <= plog->length && plog->length <= FCS_LOG_LENGTH);
    assert(out_parameter);

    size_t i, length;

    for (i = 5u; i < plog->length;) {
        length = _extract_length(plog->data[i]);

        if (plog->data[i + 1u] == device_id &&
                plog->data[i + 2u] == (uint8_t)type) {
            memcpy(out_parameter, &plog->data[i], length);
            return true;
        }

        i += length;
    }

    return false;
}

/*
Finds all parameters with a given type, and copies the first `max_parameters`
of them to the array `out_parameters`.

Returns the number of parameters found
*/
size_t fcs_parameter_find_all_by_type(const struct fcs_log_t *restrict plog,
enum fcs_parameter_type_t type,
struct fcs_parameter_t *restrict out_parameters, size_t max_parameters) {
    assert(plog);
    assert(5u <= plog->length && plog->length <= FCS_LOG_LENGTH);
    assert(out_parameters);

    size_t i, length, count = 0;

    for (i = 5u; i < plog->length && count < max_parameters;) {
        length = _extract_length(plog->data[i]);

        if (plog->data[i + 2u] == (uint8_t)type) {
            memcpy(&out_parameters[count], &plog->data[i], length);
            count++;
        }

        i += length;
    }

    return count;
}
