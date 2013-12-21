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

#ifndef _MEASUREMENT_H_
#define _MEASUREMENT_H_

enum fcs_measurement_type_t {
    FCS_MEASUREMENT_TYPE_INVALID,
    FCS_MEASUREMENT_TYPE_ACCELEROMETER,
    FCS_MEASUREMENT_TYPE_GYROSCOPE,
    FCS_MEASUREMENT_TYPE_MAGNETOMETER,
    FCS_MEASUREMENT_TYPE_PITOT,
    FCS_MEASUREMENT_TYPE_PRESSURE_TEMP,
    FCS_MEASUREMENT_TYPE_RANGEFINDER,
    FCS_MEASUREMENT_TYPE_IV,
    FCS_MEASUREMENT_TYPE_GPS_POSITION,
    FCS_MEASUREMENT_TYPE_GPS_VELOCITY,
    FCS_MEASUREMENT_TYPE_GPS_INFO,
    FCS_MEASUREMENT_TYPE_MESSAGE,
    FCS_MEASUREMENT_TYPE_CONTROL_POS,
    FCS_MEASUREMENT_TYPE_RADIO,
    FCS_MEASUREMENT_TYPE_LAST
};

enum fcs_calibration_type_t {
    FCS_CALIBRATION_NONE,
    FCS_CALIBRATION_BIAS_SCALE_1D, /* params[0] is bias, params[1] is scale */
    FCS_CALIBRATION_BIAS_SCALE_3D, /* params[0:3] are bias, [3:6] are scale */
    FCS_CALIBRATION_BIAS_SCALE_3X3, /* params[0:3] are bias, [3:9] specify a
                                       symmetric 3x3 matrix of scale factors
                                       */
    FCS_CALIBRATION_LAST
};

/*
fcs_measurement_t is a record of a single measurement, including a sensor ID
and type, as well as the raw measurement data.

Measurement header:
7:4 = reserved (0s)
3:0 = length in bytes

Measurement sensor:
7:5 = sensor ID (0-3, 4+ reserved)
4:0 = sensor type (see enum fcs_measurement_type_t)

Data:
- accel: 3x int16 (x, y, z)
- gyro: 3x int16 (x, y, z)
- mag: 3x int16 (x, y, z)
- pitot: 1x int16 (v)
- pressure/temp: 2x int16 (pressure, temp)
- rangefinder: 1x int16 (range)
- current/voltage: 2x int16 (i, v)
- gps position: 3x int32 (lat, lon, alt)
- gps velocity: 3x int16 (n, e, d)
- gps info: 2x int8 (7:4 fix mode, 3:0 num SVs, dop)
- message: text
- control position: 4x int16 (0, 1, 2, 3)
- radio: 4x int8 (RSSI, noise, # packets rx, # errors rx)
*/
struct fcs_measurement_t {
    uint8_t header; /* 7:4 = reserved; 3:0 = length of data in bytes */
    uint8_t sensor; /* 7:5 = sensor ID (0-3), 4:0 = sensor type */
    union { /* The data type is implicit based on sensor type */
        uint8_t u8[14];
        int8_t i8[14];
        uint16_t u16[7];
        int16_t i16[7];
        uint32_t u32[3];
        int32_t i32[3];
    } __attribute__ ((packed)) data;
} __attribute__ ((packed));

#define FCS_MEASUREMENT_LENGTH_MAX 14u
#define FCS_MEASUREMENT_HEADER_LENGTH_MASK 0x0Fu
#define FCS_MEASUREMENT_HEADER_LENGTH_OFFSET 0
#define FCS_MEASUREMENT_SENSOR_ID_MAX 3u
#define FCS_MEASUREMENT_SENSOR_ID_MASK 0xE0u
#define FCS_MEASUREMENT_SENSOR_ID_OFFSET 5u
#define FCS_MEASUREMENT_SENSOR_TYPE_MASK 0x1Fu
#define FCS_MEASUREMENT_SENSOR_TYPE_OFFSET 0

/* Measurement field accessors */
enum fcs_measurement_type_t fcs_measurement_get_sensor_type(
const struct fcs_measurement_t *measurement);

uint8_t fcs_measurement_get_sensor_id(
const struct fcs_measurement_t *restrict measurement);

size_t fcs_measurement_get_length(
const struct fcs_measurement_t *restrict measurement);

void fcs_measurement_set_sensor(
struct fcs_measurement_t *restrict measurement, uint8_t sensor_id,
enum fcs_measurement_type_t type);

/*
fcs_calibration_t specifies the calibration to be applied to a particular
sensor (by ID and type).

Calibration header:
7:6 = reserved
5:0 = length of calibration data in bytes (0-63)

Calibration sensor: same as "Measurement sensor" above

Type:
4 = transform by orientation quaternion?
3:0 = calibration type (see enum fcs_calibration_type_t)

Error: the 1-sigma residual error after calibration has been applied
Params: the calibration parameters
*/
struct fcs_calibration_t {
    uint8_t header; /* 7:6 = reserved; 5:0 = length of data in bytes */
    uint8_t sensor; /* 7:5 = sensor ID (0-3), 4:0 = sensor type */
    uint8_t type;   /* 7:4 = flags, 3:0 = calibration type */
    uint8_t reserved; /* set to 0 */
    float error;
    float params[9];
    float orientation[4];
    float offset[3];
} __attribute__ ((packed));

#define FCS_CALIBRATION_LENGTH_MAX 63u
#define FCS_CALIBRATION_FLAGS_MASK 0xF0u
#define FCS_CALIBRATION_FLAGS_OFFSET 4u
#define FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION 0x10u
#define FCS_CALIBRATION_TYPE_MASK 0x0Fu
#define FCS_CALIBRATION_TYPE_OFFSET 0

/* Calibration field accessors */
enum fcs_calibration_type_t fcs_calibration_get_type(
const struct fcs_calibration_t *restrict calibration);

/*
fcs_calibration_map_t contains calibration data for all sensors permitted in
an fcs_measurement_log_t. Entries are arranged by ID and type for easy access.
*/
struct fcs_calibration_map_t {
    struct fcs_calibration_t sensor_calibration[128];
};

/*
Binary log packet (all multi-byte values are LE):
1 byte type
2 bytes reserved (0)
2 bytes frame index
... (sensor readings -- fcs_measurement_t)
2 bytes CRC16

Max usage:
1 type
1 reserved (0)
2 index
8 accel 1 x, y, z
8 accel 2 x, y, z
8 gyro 1 x, y, z
8 gyro 2 x, y, z
6 pressure/temp 1
6 pressure/temp 2
6 i/v 1
6 i/v 2
4 pitot 1
4 pitot 2
4 range 1
4 range 2
8 magnetometer 1 x, y, z
8 magnetometer 2 x, y, z
14 gps pos 1 lat, lon, alt
8 gps velocity 1 n, e, d
4 gps fix mode, num SVs, error
10 control pos
6 radio
2 bytes CRC16

Total: 136 + COBS-R + NUL + NUL = 139
*/
struct fcs_measurement_log_t {
    uint8_t data[256];
    size_t length;
};

/* TODO: if there are other binary/log packet types, this should be an enum */
#define FCS_MEASUREMENT_LOG_TYPE 1u

/*
Measurement log packets -- used to send raw sensor data at 1000Hz
*/

/* Initialize a measurement log packet with a packet index of `frame_id` */
void fcs_measurement_log_init(struct fcs_measurement_log_t *restrict log_rec,
uint16_t frame_id);

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log_rec` to include a CRC16SBP.
*/
size_t fcs_measurement_log_serialize(uint8_t *restrict out_buf,
size_t out_buf_len, struct fcs_measurement_log_t *log_rec);

/*
Add a sensor value entry to a log packet. Returns true if the sensor value
could be added, or false if it couldn't.
*/
bool fcs_measurement_log_add(struct fcs_measurement_log_t *restrict log_rec,
struct fcs_measurement_t *restrict measurement);

/*
Finds a measurement with a given ID and type in the log, and copies the result
to `out_measurement`.

Returns true if a measurement with matching ID and type was found, and false
if not.
*/
bool fcs_measurement_log_find(
const struct fcs_measurement_log_t *restrict log_rec,
enum fcs_measurement_type_t type, uint8_t measurement_id,
struct fcs_measurement_t *restrict out_measurement);

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
double out_offset[3]);

/*
Calibrate a single measurement based on the calibration map parameters.
*/
void fcs_measurement_calibrate(
const struct fcs_measurement_t *restrict measurement,
const struct fcs_calibration_map_t * restrict calibration_map,
double out_value[4], double *out_error, double out_offset[3]);

/*
Convert the values associated with a measurement into an array of doubles.
Returns the number of values in the measurement.
*/
size_t fcs_measurement_get_values(
const struct fcs_measurement_t *restrict measurement, double out_value[4]);

#endif
