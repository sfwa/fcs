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

struct fcs_ahrs_sensor_geometry_t {
    float accel_orientation[4];
    float gyro_orientation[4];
    float mag_orientation[4];
    float accel_position[3];
};

struct fcs_ahrs_sensor_calibration_t {
    int16_t accel_bias[3];
    float accel_scale[3];
    float gyro_scale[3];
    int16_t pitot_bias;
    float pitot_scale;
    int16_t barometer_bias;
    float barometer_scale;
};

struct fcs_ahrs_sensor_covariance_t {
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

struct fcs_ahrs_wmm_field_t {
    float mag_field[3];
};

struct fcs_ahrs_dynamics_model_t {
    float process_noise[24];
    uint8_t model; /* 0, 1 or 2 */
};

void fcs_ahrs_init(void);
void fcs_ahrs_tick(void);

/*
Updated every AHRS tick -- contains the current state and uncertainty
estimates
*/
extern struct fcs_packet_state_t fcs_global_state;

/* Mirror of the state structure */
extern struct fcs_packet_state_t fcs_global_state_mirror;

#endif
