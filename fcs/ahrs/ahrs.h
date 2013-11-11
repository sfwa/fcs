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

#ifndef _FCS_AHRS_H
#define _FCS_AHRS_H

/*
Configuration data types for the AHRS
*/

#define FCS_AHRS_SENSOR_GEOMETRY_KEY "ahrs.sensgeom__"
#define FCS_AHRS_SENSOR_GEOMETRY_VERSION 1u

struct fcs_ahrs_sensor_geometry_v1_t {
    float accel_1_orientation[4];
    float accel_2_orientation[4];
    float mag_1_orientation[4];
    float mag_2_orientation[4];
    float accel_1_position[3];
    float accel_2_position[3];
};

#define FCS_AHRS_SENSOR_CALIBRATION_KEY "ahrs.senscalib_"
#define FCS_AHRS_SENSOR_CALIBRATION_VERSION 1u

struct fcs_ahrs_sensor_calibration_v1_t {
    float accel_1_calibration[16];
    float accel_2_calibration[16];
    float mag_calibration[16];
    float gyro_1_scale[3];
    float gyro_2_scale[3];
    float pitot_bias;
    float pitot_scale;
    float barometer_bias;
    float barometer_scale;
};

#define FCS_AHRS_SENSOR_COVARIANCE_KEY "ahrs.senscovar_"
#define FCS_AHRS_SENSOR_COVARIANCE_VERSION 1u

struct fcs_ahrs_sensor_covariance_v1_t {
    float accel_covariance;
    float gyro_covariance;
    float mag_covariance;
    float gps_position_covariance_h;
    float gps_position_covariance_v;
    float gps_velocity_covariance_h;
    float gps_velocity_covariance_v;
    float pitot_covariance;
    float barometer_amsl_covariance;
};

#define FCS_AHRS_WMM_FIELD_KEY "ahrs.wmmfield__"
#define FCS_AHRS_WMM_FIELD_VERSION 1u

struct fcs_ahrs_wmm_field_v1_t {
    float mag_field[3];
};

#define FCS_AHRS_DYNAMICS_MODEL_KEY "ahrs.dynmodel__"
#define FCS_AHRS_DYNAMICS_MODEL_VERSION 1u

struct fcs_ahrs_dynamics_model_v1_t {
    float process_noise[24];

    uint8_t model; /* 0, 1 or 2 */
};

void fcs_ahrs_init(void);
void fcs_ahrs_tick(void);
enum fcs_config_validation_result_t fcs_ahrs_validate_config(
fcs_config_t new_config);
void fcs_ahrs_load_config(fcs_config_t new_config);

#endif
