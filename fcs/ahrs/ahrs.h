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

#define FCS_AHRS_NUM_TRICAL_INSTANCES 2u

/*
AHRS global state
*/
struct fcs_ahrs_state_t {
    uint64_t solution_time;
    struct fcs_measurement_log_t measurements;

    /* State estimate values */
    double lat, lon, alt; /* rad, rad, m above ellipsoid*/
    double velocity[3]; /* NED, m/s */
    double acceleration[3]; /* body x, y, z in m/s/s */
    double attitude[4]; /* quaternion, body to world */
    double angular_velocity[3]; /* clockwise around x, y, z in rad/s */
    double angular_acceleration[3]; /* clockwise around x, y, z in rad/s/s */
    double wind_velocity[3]; /* NED, m/s */
    double gyro_bias[3]; /* x, y, z in rad/s */
    double control_pos[4]; /* arbitrary control units */

    /* State estimate covariance */
    double lat_covariance, lon_covariance, alt_covariance;
    double velocity_covariance[3];
    double acceleration_covariance[3];
    double attitude_covariance[3];
    double angular_velocity_covariance[3];
    double angular_acceleration_covariance[3];
    double wind_velocity_covariance[3];
    double gyro_bias_covariance[3];

    /* Configuration */
    double wmm_field[3];
    double ukf_process_noise[24];
    uint32_t ukf_dynamics_model;
    struct fcs_calibration_map_t calibration;
    TRICAL_instance_t trical_instances[FCS_AHRS_NUM_TRICAL_INSTANCES];
};

void fcs_ahrs_init(void);
void fcs_ahrs_tick(void);

/*
Updated every AHRS tick -- contains the current state and uncertainty
estimates
*/
extern struct fcs_ahrs_state_t fcs_global_ahrs_state;

#endif
