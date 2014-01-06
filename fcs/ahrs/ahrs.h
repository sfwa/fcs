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

#define FCS_AHRS_NUM_TRICAL_INSTANCES 4u

enum fcs_mode_t {
    /* Placeholder */
    FCS_MODE_STARTUP_VALUE,
    /*
    State: initializing (I)
    Previous states: none
    Entry condition: startup
    Control output: none
    */
    FCS_MODE_INITIALIZING,
    /*
    State: calibrating (C)
    Previous states: initializing, safe
    Entry condition: received first GPS packet ||
        calibration command from GCS
    Control output: none
    */
    FCS_MODE_CALIBRATING,
    /*
    State: safe (S)
    Previous states: calibrating, armed, active
    Entry condition: time since last GPS packet received < 1s &&
        (time since entered calibration > 30s ||
        safe command from GCS)
    Control output: none
    */
    FCS_MODE_SAFE,
    /*
    State: armed (R)
    Previous states: safe
    Entry condition: arm command from GCS && all sensor health == OK &&
        mission boundary defined && home waypoint defined &&
        recovery waypoint defined
    Control output: take-off, failsafe device arm asserted
    */
    FCS_MODE_ARMED,
    /*
    State: active (A)
    Previous states: armed, holding
    Entry condition: freefall detected (take-off) || activate command from GCS
    Control output: from NMPC
    Waypoint mode: normal
    */
    FCS_MODE_ACTIVE,
    /*
    State: holding (H)
    Previous states: active
    Entry condition: lost GPS || hold command from GCS
    Control output: from NMPC
    Waypoint mode: hold at current position
    */
    FCS_MODE_HOLDING,
    /*
    State: abort (F)
    Previous states: armed, active, holding
    Entry condition: mission boundary crossed ||
        (lost data link && lost gps) ||
        termination requested || sensor health != OK
    Control output: failsafe (all min), failsafe device abort asserted
    */
    FCS_MODE_ABORT,
    /* Sentinel */
    FCS_MODE_INVALID
};

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
    double wmm_field_dir[3];
    double wmm_field_norm;
    double ukf_process_noise[24];
    uint32_t ukf_dynamics_model;
    struct fcs_calibration_map_t calibration;
    TRICAL_instance_t trical_instances[FCS_AHRS_NUM_TRICAL_INSTANCES];
    double trical_update_attitude[4][FCS_AHRS_NUM_TRICAL_INSTANCES];

    /* Mode */
    enum fcs_mode_t mode;
    uint64_t mode_start_time;
};

void fcs_ahrs_init(void);
void fcs_ahrs_tick(void);

/*
Change the AHRS mode. Returns false if the mode change is rejected, or true
if successful.
*/
bool fcs_ahrs_set_mode(enum fcs_mode_t mode);

/*
Updated every AHRS tick -- contains the current state and uncertainty
estimates
*/
extern struct fcs_ahrs_state_t fcs_global_ahrs_state;

#endif
