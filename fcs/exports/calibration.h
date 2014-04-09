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

#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include "parameter.h"

enum fcs_calibration_type_t {
    FCS_CALIBRATION_NONE,
    FCS_CALIBRATION_BIAS_SCALE_1D, /* params[0] is bias, params[1] is scale */
    FCS_CALIBRATION_BIAS_SCALE_2D, /* params[0] is bias 0, params[1] is scale
                                      0, params[2] is bias 1, params[3] is
                                      scale 1 */
    FCS_CALIBRATION_BIAS_SCALE_3X3, /* params[0:3] are bias, [3:11] specify a
                                       3x3 matrix of scale factors */
    FCS_CALIBRATION_LAST
};

/*
fcs_calibration_t specifies the calibration to be applied to a particular
sensor (by ID and type).

Calibration header:
7:6 = reserved
5:0 = length of calibration data in bytes (0-63)

Calibration device/type: same as parameter device/type

Type:
5 = ignore sensor value
4 = transform by orientation quaternion?
3:0 = calibration type (see enum fcs_calibration_type_t)

Error: the 1-sigma residual error after calibration has been applied
Params: the calibration parameters
*/
struct fcs_calibration_t {
    uint8_t header; /* 7:6 = reserved; 5:0 = length of data in bytes */
    uint8_t device; /* device ID */
    uint8_t type;   /* 7:5 = reserved, 4:0 = parameter type */
    uint8_t calibration_type;   /* 7:4 = flags, 3:0 = calibration type */
    float error;
    float params[12];
    float orientation[4];
    float offset[3];
    float scale_factor;
} __attribute__ ((packed));

#define FCS_CALIBRATION_LENGTH_MAX 63u
#define FCS_CALIBRATION_FLAGS_MASK 0xF0u
#define FCS_CALIBRATION_FLAGS_OFFSET 4u
#define FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION 0x10u
#define FCS_CALIBRATION_TYPE_MASK 0x0Fu
#define FCS_CALIBRATION_TYPE_OFFSET 0

#define FCS_CALIBRATION_COUNT 32

/*
fcs_calibration_map_t contains calibration data for all sensors permitted in
an fcs_measurement_log_t.
*/
struct fcs_calibration_map_t {
    struct fcs_calibration_t sensor_calibration[FCS_CALIBRATION_COUNT];
    size_t num_entries;
};

void fcs_calibration_map_init(struct fcs_calibration_map_t *restrict cmap);

/*
Register a calibration map entry for a given parameter type and device.
Returns false if no more entries are available, or true otherwise.
*/
bool fcs_calibration_map_register_calibration(
struct fcs_calibration_map_t *restrict cmap,
struct fcs_calibration_t *restrict calibration);

/* Calibration field accessors */
enum fcs_calibration_type_t fcs_calibration_get_type(
const struct fcs_calibration_t *restrict calibration);

/*
Retrieve a calibrated value for a given sensor. If multiple measurements
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
double *restrict out_offset, double prescale);

/*
Calibrate a single measurement based on the calibration map parameters, with
sensor readings scaled by `prescale` prior to calibration parameters being
applied.
*/
void fcs_parameter_calibrate(
const struct fcs_parameter_t *restrict parameter,
struct fcs_calibration_map_t *cmap, double out_value[4], double *out_error,
double *restrict out_offset, double prescale);

/*
Return a calibration map entry given a calibration map and a parameter
*/
struct fcs_calibration_t* fcs_parameter_get_calibration(
struct fcs_calibration_map_t *restrict cmap,
const struct fcs_parameter_t *restrict parameter);

#endif
