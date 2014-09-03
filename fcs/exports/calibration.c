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
#include <math.h>

#include "calibration.h"
#include "../util/util.h"
#include "../util/3dmath.h"


void fcs_calibration_map_init(struct fcs_calibration_map_t *restrict cmap) {
    memset(cmap, 0xFFu, sizeof(struct fcs_calibration_map_t));
    cmap->num_entries = 0;
}

/*
Register a calibration map entry for a given parameter type and device.
Returns false if no more entries are available, or true otherwise.
*/
bool fcs_calibration_map_register_calibration(
struct fcs_calibration_map_t *restrict cmap,
struct fcs_calibration_t *restrict calibration) {
    fcs_assert(cmap);
    fcs_assert(cmap->num_entries <= FCS_CALIBRATION_COUNT);
    fcs_assert(calibration);

    if (cmap->num_entries == FCS_CALIBRATION_COUNT) {
        return false;
    }

    cmap->sensor_calibration[cmap->num_entries] = *calibration;
    cmap->num_entries++;

    return true;
}

enum fcs_calibration_type_t fcs_calibration_get_type(
const struct fcs_calibration_t *restrict calibration) {
    fcs_assert(calibration);

    enum fcs_calibration_type_t type;
    type = (enum fcs_calibration_type_t)(
        (calibration->calibration_type & FCS_CALIBRATION_TYPE_MASK)
        >> FCS_CALIBRATION_TYPE_OFFSET
    );

    return type;
}

/*
Return a calibration map entry given a calibration map and `parameter`.
Returns NULL if the calibration for `parameter` was not found.
*/
struct fcs_calibration_t* fcs_parameter_get_calibration(
struct fcs_calibration_map_t *restrict cmap,
const struct fcs_parameter_t *restrict parameter) {
    fcs_assert(cmap);
    fcs_assert(cmap->num_entries <= FCS_CALIBRATION_COUNT);
    fcs_assert(parameter);

    size_t i;

    for (i = 0; i < cmap->num_entries; i++) {
        if (cmap->sensor_calibration[i].device == parameter->device &&
                cmap->sensor_calibration[i].type == parameter->type) {
            return &(cmap->sensor_calibration[i]);
        }
    }

    return NULL;
}

/*
Calibrate a single measurement based on the calibration map parameters, with
sensor readings scaled by `prescale` prior to calibration parameters being
applied.
*/
void fcs_parameter_calibrate(
const struct fcs_parameter_t *restrict parameter,
struct fcs_calibration_map_t *cmap, double out_value[4], double *out_error,
double *restrict out_offset, double prescale) {
    fcs_assert(parameter);
    fcs_assert(cmap);
    fcs_assert(out_value);
    fcs_assert(out_error);
    fcs_assert(out_value != out_error);
    fcs_assert(out_value != out_offset);
    fcs_assert(out_error != out_offset);
    _nassert((size_t)out_value % 8 == 0);
    _nassert((size_t)out_error % 8 == 0);
    _nassert((size_t)out_offset % 8 == 0);

    size_t i;
    double temp_value[4], c[3], orientation[4];
    fcs_parameter_get_values_d(parameter, temp_value, 4u);

    /*
    Look up the calibration entry based on the combination of sensor ID and
    type
    */
    const struct fcs_calibration_t *calibration =
        fcs_parameter_get_calibration(cmap, parameter);
    const float *restrict p = calibration->params;

    fcs_assert(calibration->type);

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

            if (parameter->type == FCS_PARAMETER_PITOT) {
                temp_value[0] = absval(temp_value[0]);
            }
            break;
        case FCS_CALIBRATION_BIAS_SCALE_2D:
            temp_value[0] -= p[0];
            temp_value[0] *= p[1];
            temp_value[1] -= p[2];
            temp_value[1] *= p[3];
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
        case FCS_CALIBRATION_LAST:
            /* Invalid calibration type */
            fcs_assert(false);
    }

    if (calibration->type & FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION) {
        /* Transform calibrated value based on sensor orientation */
        vector_d_from_f(orientation, calibration->orientation, 4u);
        quaternion_vector3_multiply_d(out_value, orientation, temp_value);
    } else {
        vector_copy_d(out_value, temp_value, 4u);
    }
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
size_t fcs_log_get_calibrated_value(
const struct fcs_log_t *restrict plog, struct fcs_calibration_map_t *cmap,
enum fcs_parameter_type_t type, double out_value[4], double *out_error,
double *restrict out_offset, double prescale) {
    fcs_assert(plog);
    fcs_assert(cmap);
    fcs_assert(out_value);
    fcs_assert(out_error);
    fcs_assert(out_value != out_error);
    fcs_assert(out_value != out_offset);
    fcs_assert(out_error != out_offset);
    _nassert((size_t)out_value % 8 == 0);
    _nassert((size_t)out_error % 8 == 0);
    _nassert((size_t)out_offset % 8 == 0);

    double accum_value[4], accum_error, accum_offset[3];
    double temp_value[4], temp_error, temp_offset[3], scale;
    size_t i, j, n_parameters;
    struct fcs_parameter_t parameters[4];

    n_parameters = fcs_parameter_find_all_by_type(plog, type, parameters, 4u);

    if (n_parameters > 0) {
        vector_set_d(accum_value, 0, 4u);
        vector_set_d(accum_offset, 0, 3u);
        accum_error = 0.0;

        for (i = 0; i < n_parameters; i++) {
            /* Process the reading and accumulate the output */
            fcs_parameter_calibrate(&parameters[i], cmap, temp_value,
                                    &temp_error, temp_offset, prescale);

            for (j = 0; j < 3u; j++) {
                accum_value[j] += temp_value[j];
                accum_offset[j] += temp_offset[j];
            }
            accum_value[3] += temp_value[3];
            accum_error += temp_error;
        }

        /* Get the mean of the results */
        scale = 1.0 / (double)n_parameters;
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

    return n_parameters;
}
