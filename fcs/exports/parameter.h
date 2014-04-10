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

#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include "log.h"

enum fcs_parameter_type_t {
    FCS_PARAMETER_INVALID,
    /* Raw sensor readings */
    FCS_PARAMETER_ACCELEROMETER_XYZ,
    FCS_PARAMETER_GYROSCOPE_XYZ,
    FCS_PARAMETER_MAGNETOMETER_XYZ,
    FCS_PARAMETER_PITOT,
    FCS_PARAMETER_PRESSURE_TEMP,
    FCS_PARAMETER_IV,
    FCS_PARAMETER_GPS_POSITION_LLA,
    FCS_PARAMETER_GPS_VELOCITY_NED,
    FCS_PARAMETER_GPS_INFO,
    FCS_PARAMETER_CONTROL_POS,
    FCS_PARAMETER_CONTROL_MODE,
    FCS_PARAMETER_RADIO,
    /* Calibration/sensor merge output */
    FCS_PARAMETER_HAL_ACCELEROMETER_XYZ,
    FCS_PARAMETER_HAL_ACCELEROMETER_VARIANCE,
    FCS_PARAMETER_HAL_ACCELEROMETER_OFFSET_XYZ,
    FCS_PARAMETER_HAL_GYROSCOPE_XYZ,
    FCS_PARAMETER_HAL_GYROSCOPE_VARIANCE,
    FCS_PARAMETER_HAL_MAGNETOMETER_XYZ,
    FCS_PARAMETER_HAL_MAGNETOMETER_VARIANCE,
    FCS_PARAMETER_HAL_AIRSPEED,
    FCS_PARAMETER_HAL_AIRSPEED_VARIANCE,
    FCS_PARAMETER_HAL_PRESSURE_ALTITUDE,
    FCS_PARAMETER_HAL_PRESSURE_ALTITUDE_VARIANCE,
    FCS_PARAMETER_HAL_POSITION_LAT_LON,
    FCS_PARAMETER_HAL_POSITION_LAT_LON_VARIANCE,
    FCS_PARAMETER_HAL_POSITION_ALT,
    FCS_PARAMETER_HAL_POSITION_ALT_VARIANCE,
    FCS_PARAMETER_HAL_VELOCITY_NED,
    FCS_PARAMETER_HAL_VELOCITY_NED_VARIANCE,
    /* Derived parameters */
    FCS_PARAMETER_DERIVED_REFERENCE_PRESSURE,
    FCS_PARAMETER_DERIVED_REFERENCE_ALT,
    /* AHRS output */
    FCS_PARAMETER_ESTIMATED_POSITION_LLA,
    FCS_PARAMETER_ESTIMATED_VELOCITY_NED,
    FCS_PARAMETER_ESTIMATED_ATTITUDE_Q,
    FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ,
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED,
    FCS_PARAMETER_ESTIMATED_GYRO_BIAS_XYZ,
    FCS_PARAMETER_ESTIMATED_POSITION_SD,
    FCS_PARAMETER_ESTIMATED_VELOCITY_SD,
    FCS_PARAMETER_ESTIMATED_ATTITUDE_SD,
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_SD,
    FCS_PARAMETER_ESTIMATED_STATIC_PRESSURE,
    FCS_PARAMETER_ESTIMATED_STATIC_TEMP,
    FCS_PARAMETER_ESTIMATED_DYNAMIC_PRESSURE,
    FCS_PARAMETER_ESTIMATED_WMM_FIELD,
    /* NMPC output */
    FCS_PARAMETER_CONTROL_SETPOINT,
    /* Status */
    FCS_PARAMETER_AHRS_MODE,
    FCS_PARAMETER_GP_OUT,
    FCS_PARAMETER_NAV_VERSION,
    FCS_PARAMETER_NAV_PATH,
    FCS_PARAMETER_AHRS_STATUS,
    FCS_PARAMETER_CONTROL_STATUS,
    /* General-purpose */
    FCS_PARAMETER_KEY_VALUE,
    /* Sentinel */
    FCS_PARAMETER_LAST
};

enum fcs_value_type_t {
    FCS_VALUE_UNSIGNED,
    FCS_VALUE_SIGNED,
    FCS_VALUE_FLOAT,
    FCS_VALUE_RESERVED
};

/*
- accel: 3x int16 (x, y, z)
- gyro: 3x int16 (x, y, z)
- mag: 3x int16 (x, y, z)
- pitot: 1x int16 (v)
- pressure/temp: 2x int16 (pressure, temp)
- rangefinder: 1x int16 (range)
- current/voltage: 2x int16 (i, v)
- gps position: 3x int32 (lat, lon, alt)
- gps velocity: 3x int16 (n, e, d)
- gps info: 3x uint8 (fix mode, num SVs, dop)
- message: text
- control position: 4x int16 (0, 1, 2, 3)
- radio: 4x int8 (RSSI, noise, # packets rx, # errors rx)
*/

struct fcs_parameter_t {
    uint8_t header; /* 7 = mode; 6:5 = value type (0 = unsigned, 1 = signed,
                       2 = float, 3 = reserved); 4:3 = log2(bytes per value);
                       2:0 = number of values */
    uint8_t device; /* device ID */
    uint8_t type;   /* data type */
    union { /* The data type is implicit based on parameter type */
        uint8_t u8[61];
        int8_t i8[61];
        uint16_t u16[4];
        int16_t i16[4];
        uint32_t u32[4];
        int32_t i32[4];
        float16_t f16[4];
        float f32[4];
        uint64_t u64[2];
        int64_t i64[2];
        double f64[2];
    } __attribute__ ((packed)) data;
} __attribute__ ((packed));

#define FCS_PARAMETER_PRECISION_MAX 4u
#define FCS_PARAMETER_HEADER_TYPE_MASK 0x60u
#define FCS_PARAMETER_HEADER_TYPE_OFFSET 5u
#define FCS_PARAMETER_HEADER_PRECISION_MASK 0x18u
#define FCS_PARAMETER_HEADER_PRECISION_OFFSET 3u
#define FCS_PARAMETER_HEADER_NUM_VALUES_MASK 0x07u
#define FCS_PARAMETER_HEADER_NUM_VALUES_OFFSET 0
#define FCS_PARAMETER_HEADER_DATA_LENGTH_MASK 0x3Fu
#define FCS_PARAMETER_HEADER_DATA_LENGTH_OFFSET 0
#define FCS_PARAMETER_HEADER_MODE_MASK 0x80u
#define FCS_PARAMETER_NUM_VALUES_MAX 4u
#define FCS_PARAMETER_DATA_LENGTH_MAX 61u
#define FCS_PARAMETER_TYPE_MASK 0x7Fu
#define FCS_PARAMETER_TYPE_OFFSET 0

/* Parameter field accessors */
enum fcs_parameter_type_t fcs_parameter_get_type(
const struct fcs_parameter_t *restrict parameter);

uint8_t fcs_parameter_get_device_id(
const struct fcs_parameter_t *restrict parameter);

size_t fcs_parameter_get_num_values(
const struct fcs_parameter_t *restrict parameter);

size_t fcs_parameter_get_precision_bits(
const struct fcs_parameter_t *restrict parameter);

size_t fcs_parameter_get_length(
const struct fcs_parameter_t *restrict parameter);

enum fcs_value_type_t fcs_parameter_get_value_type(
const struct fcs_parameter_t *restrict parameter);

void fcs_parameter_set_header(
struct fcs_parameter_t *restrict parameter, enum fcs_value_type_t type,
size_t precision_bits, size_t num_values);

void fcs_parameter_set_device(
struct fcs_parameter_t *restrict parameter, uint8_t device_id);

void fcs_parameter_set_type(
struct fcs_parameter_t *restrict parameter,
enum fcs_parameter_type_t type);

size_t fcs_parameter_get_key_value(uint8_t key[4], uint8_t *restrict value,
size_t value_length, const struct fcs_parameter_t *restrict parameter);

void fcs_parameter_set_key_value(struct fcs_parameter_t *restrict parameter,
uint8_t key[4], uint8_t *restrict value, size_t value_length);

/*
Convert the values associated with a parameter into an array of doubles.
Returns the number of values in the parameter.
*/
size_t fcs_parameter_get_values_d(
const struct fcs_parameter_t *restrict parameter, double *restrict out_value,
size_t out_value_length);

/*
Add a parameter entry to a log packet. Returns `true` if the parameter could
be added, or `false` if it couldn't.
*/
bool fcs_log_add_parameter(struct fcs_log_t *restrict plog,
struct fcs_parameter_t *restrict parameter);

/*
Finds a parameter with a given device and type in the log, and copies the
result to `out_parameter`.

Returns true if a parameter with matching device and type was found, and false
if not.
*/
bool fcs_parameter_find_by_type_and_device(
const struct fcs_log_t *restrict plog, enum fcs_parameter_type_t type,
uint8_t device_id, struct fcs_parameter_t *restrict out_parameter);

/*
Finds all parameters with a given type, and copies the first `max_parameters`
of them to the array `out_parameters`.

Returns the number of parameters found
*/
size_t fcs_parameter_find_all_by_type(const struct fcs_log_t *restrict plog,
enum fcs_parameter_type_t type,
struct fcs_parameter_t *restrict out_parameters, size_t max_parameters);

/*
Set the device ID of all parameters in the log to `device_id`
*/
void fcs_log_set_parameter_device_id(struct fcs_log_t *restrict plog,
uint8_t device_id);

#endif
