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

#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../stats/stats.h"
#include "../ukf/cukf.h"
#include "../control/control.h"
#include "../exports/exports.h"
#include "../exports/calibration.h"
#include "wmm.h"
#include "ahrs.h"

#define AHRS_DELTA 0.001

/* Global FCS state structure */
static double ahrs_process_noise[] = {
    1e-17, 1e-17, 1e-4, /* lat, lon, alt */
    2e-3, 2e-3, 2e-3, /* velocity N, E, D */
    2e-2, 2e-2, 2e-2, /* acceleration x, y, z */
    7e-8, 7e-8, 7e-8, /* attitude roll, pitch, yaw */
    1e-3, 1e-3, 1e-3, /* angular velocity roll, pitch, yaw */
    1e-3, 1e-3, 1e-3, /* angular acceleration roll, pitch, yaw */
    3e-4, 3e-4, 3e-4, /* wind velocity N, E, D --
                         NOTE: overridden in fcs_ahrs_set_mode */
    1e-12, 1e-12, 1e-12 /* gyro bias x, y, z --
                           NOTE: overridden in fcs_ahrs_set_mode */
};
static double ahrs_wmm_field[3];
static uint64_t ahrs_solution_time;
static enum ukf_model_t ahrs_dynamics_model;
static uint64_t ahrs_mode_start_time;
static enum fcs_mode_t ahrs_mode;
static uint64_t ahrs_last_gps_time;
static uint64_t ahrs_last_baro_time;
static uint64_t ahrs_last_comms_time;
static uint16_t ahrs_last_gcs_packet_tick;

/*
Check a vector for NaN values; return true if any are found, and false
otherwise
*/
inline static bool _vec_hasnan_d(const double *vec, size_t n) {
    size_t i;
    for (i = 0; i < n; i++) {
        if (isnan(vec[i])) {
            return true;
        }
    }
    return false;
}

static void _update_wmm(double lla[3], double field[3]);
static bool _get_hal_sensor_value(const struct fcs_log_t *plog,
enum fcs_parameter_type_t param_type, double *restrict value,
double *restrict variance, size_t n);
static void _reset_state(void);
static void _set_estimate_value_i32(struct fcs_log_t *elog,
enum fcs_parameter_type_t param_type, uint8_t value_size,
const int32_t *restrict v, size_t n);
static void _populate_estimate_log(
const struct ukf_state_t *restrict state,
const double *restrict error, const double *restrict field,
enum fcs_mode_t mode, double static_pressure, double static_temp);
static void _populate_ahrs_status(struct fcs_log_t *estimate_log,
enum fcs_mode_t ahrs_mode);


void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    fcs_assert(ukf_config_get_state_dim() == 24u);
    fcs_assert(ukf_config_get_control_dim() == 4u);
    fcs_assert(ukf_config_get_measurement_dim() == 20u);
    fcs_assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    ahrs_solution_time = 0;
    ahrs_mode_start_time = 0;
    ahrs_wmm_field[0] = 1.0;
    ahrs_wmm_field[1] = ahrs_wmm_field[2] = 0.0;
    ahrs_last_gps_time = ahrs_last_baro_time = 0;

    fcs_wmm_init();

    /* Initialize AHRS mode */
    ahrs_dynamics_model = UKF_MODEL_X8;
    fcs_ahrs_set_mode(FCS_MODE_INITIALIZING);
}

#include <stdio.h>
void fcs_ahrs_tick(void) {
    /*
    While copying measurement data to the UKF, get sensor error and geometry
    information so the sensor model parameters can be updated.
    */
    static double control_pos[4];  /* Static so that missing values in a given
                                      frame don't disrupt the filter */
    double v[3], speed, wmm_field_inv;
    bool got_result, got_gps = false, got_reference_alt = false;
    struct ukf_ioboard_params_t params;
    struct fcs_parameter_t parameter;
    struct fcs_log_t *hal_log, *measurement_log, *estimate_log;
    struct ukf_state_t state_values;
    double error[24];

    /* Increment solution time */
    ahrs_solution_time++;

    /* Read virtual sensor values from the HAL log and pass them to the UKF */
    hal_log = fcs_exports_log_open(FCS_LOG_TYPE_SENSOR_HAL, FCS_MODE_READ);
    fcs_assert(hal_log);

    memset(&params, 0, sizeof(params));
    params.accel_orientation[W] = 1.0;
    params.gyro_orientation[W] = 1.0;
    params.mag_orientation[W] = 1.0;

    /* Use the latest WMM field vector (unit length) */
    wmm_field_inv = 1.0 / vector3_norm_d(ahrs_wmm_field);
    vector3_scale_d(ahrs_wmm_field, wmm_field_inv);
    vector_copy_d(params.mag_field, ahrs_wmm_field, 3u);

    /* Read sensor data from the measurement log, and pass it to the UKF */
    ukf_sensor_clear();

    /* Read accelerometer */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_ACCELEROMETER_XYZ, v,
                              params.accel_covariance, 3u)) {
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);

        got_result = fcs_parameter_find_by_type_and_device(
            hal_log, FCS_PARAMETER_HAL_ACCELEROMETER_OFFSET_XYZ, 0,
            &parameter);
        fcs_assert(got_result);

        fcs_parameter_get_values_d(&parameter, params.accel_offset, 3u);
    }

    /* Read gyro */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_GYROSCOPE_XYZ, v,
                              params.gyro_covariance, 3u)) {
        ukf_sensor_set_gyroscope(v[0], v[1], v[2]);
    }

    /* Read magnetometer */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_MAGNETOMETER_XYZ, v,
                              params.mag_covariance, 3u)) {
        vector3_scale_d(v, wmm_field_inv);
        ukf_sensor_set_magnetometer(v[0], v[1], v[2]);
    }

    /* Read pitot-derived airspeed */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_AIRSPEED, v,
                              &params.pitot_covariance, 1u)) {
        ukf_sensor_set_pitot_tas(v[0]);
    }

    /* Read barometer-derived altitude */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_PRESSURE_ALTITUDE, v,
                              &params.barometer_amsl_covariance, 1u)) {
        ukf_sensor_set_barometer_amsl(v[0]);
        ahrs_last_baro_time = ahrs_solution_time;
    }

    /* Read GPS position and velocity */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_POSITION_LAT_LON, v,
                             params.gps_position_covariance, 2u) &&
           _get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_POSITION_ALT,
                                 &v[2], &params.gps_position_covariance[2],
                                 1u)) {
        ukf_sensor_set_gps_position(v[0], v[1], v[2]);
        ahrs_last_gps_time = ahrs_solution_time;
        got_gps = true;
    }
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_VELOCITY_NED, v,
                              params.gps_velocity_covariance, 3u)) {
        ukf_sensor_set_gps_velocity(v[0], v[1], v[2]);
    }

    hal_log = fcs_exports_log_close(hal_log);
    fcs_assert(!hal_log);

    /*
    Clear accelerometer offset during calibration and safe mode; the vehicle
    shouldn't be moving so it's not needed, and can cause convergence issues
    */
    if (ahrs_mode != FCS_MODE_ACTIVE && ahrs_mode != FCS_MODE_ARMED) {
        vector_set_d(params.accel_offset, 0.0, 3u);
    }

    /*
    Run the UKF, taking sensor readings and current control position into
    account
    */
    ukf_set_params(&params);
    ukf_set_process_noise(ahrs_process_noise);

    measurement_log = fcs_exports_log_open(FCS_LOG_TYPE_MEASUREMENT,
                                           FCS_MODE_READ);
    fcs_assert(measurement_log);

    /* Don't update the filter during initialization */
    if (ahrs_mode != FCS_MODE_INITIALIZING) {
        got_result = fcs_parameter_find_by_type_and_device(
        	measurement_log, FCS_PARAMETER_CONTROL_POS, 1u, &parameter);
        if (got_result) {
            fcs_parameter_get_values_d(&parameter, control_pos, 4u);
            vector3_scale_d(control_pos, 1.0 / 65535.0);
        }

        ukf_iterate((float)AHRS_DELTA, control_pos);
    }

    /* Check if we've received a new packet from the GCS recently */
    got_result = fcs_parameter_find_by_type_and_device(
        measurement_log, FCS_PARAMETER_IO_STATUS, 1u, &parameter);
    if (got_result) {
        if (ahrs_last_gcs_packet_tick != parameter.data.u16[1]) {
            ahrs_last_comms_time = ahrs_solution_time;
            got_reference_alt = true;
        }

        ahrs_last_gcs_packet_tick = parameter.data.u16[1];
    }

    measurement_log = fcs_exports_log_close(measurement_log);
    fcs_assert(!measurement_log);

    /* Copy the state out of the UKF */
    ukf_get_state(&state_values);
    ukf_get_state_error(error);

    /* Validate the UKF state; if any values are NaN, reset it */
    if (_vec_hasnan_d(state_values.position, 25u) ||
            _vec_hasnan_d(error, 24u)) {
        _reset_state();
    } else {
        /* Clear the downward component of wind. */
        state_values.wind_velocity[2] = 0.0;
        ukf_set_state(&state_values);
    }

    /*
    Don't update the estimate log or the WMM field when simulating, since the
    relevant values will be provided by the simulation driver.
    */
    if (ahrs_mode != FCS_MODE_SIMULATING) {
        /* Get a new WMM field estimate */
        _update_wmm((double*)state_values.position, ahrs_wmm_field);

        _populate_estimate_log(&state_values, error, ahrs_wmm_field,
                               ahrs_mode, STANDARD_PRESSURE, STANDARD_TEMP);
    } else {
        estimate_log = fcs_exports_log_open(FCS_LOG_TYPE_ESTIMATE, FCS_MODE_APPEND);
        fcs_assert(estimate_log);
        _populate_ahrs_status(estimate_log, ahrs_mode);
        estimate_log = fcs_exports_log_close(estimate_log);
        fcs_assert(!estimate_log);
    }

    /* Check the current mode and transition if necessary */
    if (ahrs_mode == FCS_MODE_STARTUP_VALUE) {
        fcs_ahrs_set_mode(FCS_MODE_INITIALIZING);
    } else if (ahrs_mode == FCS_MODE_INITIALIZING) {
        /*
        Transition out of initializing if we've received a GPS packet as well
        as a reference pressure from the GCS
        */
        if (got_gps && ahrs_last_gcs_packet_tick != 0) {
            fcs_ahrs_set_mode(FCS_MODE_CALIBRATING);
        }
    } else if (ahrs_mode == FCS_MODE_SIMULATING) {
        /*
        Transition out of simulation mode if no packet has been received for
        more than 30s.
        */
        if (ahrs_solution_time - ahrs_mode_start_time > 30000) {
            fcs_ahrs_set_mode(FCS_MODE_CALIBRATING);
        }
    } else if (ahrs_mode == FCS_MODE_CALIBRATING) {
        /* Transition out of calibration mode after 30s */
        if (ahrs_solution_time - ahrs_mode_start_time > 30000) {
            fcs_ahrs_set_mode(FCS_MODE_SAFE);
        }
    } else if (ahrs_mode == FCS_MODE_SAFE) {
        /* Transition out of safe mode once throttle exceeds 50% */
        if (control_pos[0] > 0.5) {
            fcs_ahrs_set_mode(FCS_MODE_ARMED);
        }
    } else if (ahrs_mode == FCS_MODE_ARMED) {
        /* Transition to active once speed exceeds 8m/s */
        speed = vector3_norm_d((double*)&state_values.velocity);
        if (speed > 8.0) {
            fcs_ahrs_set_mode(FCS_MODE_ACTIVE);
        }
    }
    /* TODO: transition from safe to armed once input throttle hits 100% */
}

static void _reset_state(void) {
    size_t i;
    struct ukf_state_t reset_state, current_state;

    ukf_get_state(&current_state);

    /*
    Reset critical UKF parameters (position, attitude, gyro bias) based on
    their previous values, and clear everything else.
    */
    memset(&reset_state, 0, sizeof(reset_state));

    /*
    Copy the last position and attitude; if gyro bias is sane, copy that too
    */
    if (!_vec_hasnan_d(current_state.position, 3u)) {
        vector_copy_d(reset_state.position, current_state.position, 3u);
    }

    if (!_vec_hasnan_d(current_state.attitude, 4u)) {
        vector_copy_d(reset_state.attitude, current_state.attitude, 4u);
    } else {
        reset_state.attitude[3] = 1.0;
    }

    double bias;
    #pragma MUST_ITERATE(3, 3)
    for (i = 0; i < 3u; i++) {
        bias = current_state.gyro_bias[i];
        if (!isnan(bias) && fabs(bias) < M_PI * 0.1) {
            reset_state.gyro_bias[i] = bias;
        }
    }

    ukf_init();
    ukf_set_state(&reset_state);

    fcs_global_counters.ukf_resets++;
}

static void _update_wmm(double lla[3], double field[3]) {
    /* Calculate WMM field at current lat/lon/alt/time */
    if (!fcs_wmm_calculate_field(lla[0], lla[1], lla[2], 2014.0, field)) {
        fcs_global_counters.wmm_errors++;
    } else {
        /*
        WMM returns the field in nT; the magnetometer sensitivity is in Gauss
        (1G = 100 000nT). Convert the field to G.
        */
        vector3_scale_d(field, 1.0 / 100000.0);
    }
}

static void _populate_estimate_log(
const struct ukf_state_t *restrict state,
const double *restrict error, const double *restrict field,
enum fcs_mode_t mode, double static_pressure, double static_temp) {
    struct fcs_parameter_t param;
    struct fcs_log_t *estimate_log;
    int32_t tmp[4];

    estimate_log = fcs_exports_log_open(FCS_LOG_TYPE_ESTIMATE,
    		                            FCS_MODE_WRITE);
    fcs_assert(estimate_log);

    /* Lat/lon in (INT32_MAX/PI); alt in cm */
    tmp[0] = (int32_t)(state->position[0] * ((double)INT32_MAX / M_PI));
    tmp[1] = (int32_t)(state->position[1] * ((double)INT32_MAX / M_PI));
    tmp[2] = (int32_t)(state->position[2] * 1e2);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_POSITION_LLA, 4u, tmp, 3u);

    /* Velocity in cm/s */
    tmp[0] = (int32_t)(state->velocity[0] * 1e2);
    tmp[1] = (int32_t)(state->velocity[1] * 1e2);
    tmp[2] = (int32_t)(state->velocity[2] * 1e2);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_VELOCITY_NED, 2u, tmp, 3u);

    /* Quaternion values with [-1, 1] scaled to [-32767, 32767] */
    tmp[0] = (int32_t)(state->attitude[0] * (double)INT16_MAX);
    tmp[1] = (int32_t)(state->attitude[1] * (double)INT16_MAX);
    tmp[2] = (int32_t)(state->attitude[2] * (double)INT16_MAX);
    tmp[3] = (int32_t)(state->attitude[3] * (double)INT16_MAX);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_ATTITUDE_Q, 2u, tmp, 4u);

    /* Angular velocity with +/- 4pi rad/s scaled to [-32767, 32767] */
    tmp[0] = (int32_t)(state->angular_velocity[0] *
                       ((double)INT16_MAX / M_PI) * 0.25);
    tmp[1] = (int32_t)(state->angular_velocity[1] *
                       ((double)INT16_MAX / M_PI) * 0.25);
    tmp[2] = (int32_t)(state->angular_velocity[2] *
                       ((double)INT16_MAX / M_PI) * 0.25);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ, 2u, tmp,
        3u);

    /* Wind velocity in cm/s */
    tmp[0] = (int32_t)(state->wind_velocity[0] * 1e2);
    tmp[1] = (int32_t)(state->wind_velocity[1] * 1e2);
    tmp[2] = (int32_t)(state->wind_velocity[2] * 1e2);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED, 2u, tmp, 3u);

    /* Gyro bias with +/- pi/2 rad/s scaled to [-32767, 32767] */
    tmp[0] = (int32_t)(state->gyro_bias[0] *
                       ((double)INT16_MAX / M_PI) * 2.0);
    tmp[1] = (int32_t)(state->gyro_bias[1] *
                       ((double)INT16_MAX / M_PI) * 2.0);
    tmp[2] = (int32_t)(state->gyro_bias[2] *
                       ((double)INT16_MAX / M_PI) * 2.0);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_GYRO_BIAS_XYZ, 2u, tmp, 3u);

    /* Horizontal and vertical error std dev in cm */
    tmp[0] = (int32_t)(6378000.0 * max(error[0], error[1]) * 1e2);
    tmp[1] = (int32_t)(error[2] * 1e2);
    //_set_estimate_value_i32(
    //    estimate_log, FCS_PARAMETER_ESTIMATED_POSITION_SD, 2u, tmp, 2u);

    /* Velocity error std dev in cm/s */
    tmp[0] = (int32_t)(vector3_norm_d(&error[3]) * 1e2);
    //_set_estimate_value_i32(
    //    estimate_log, FCS_PARAMETER_ESTIMATED_VELOCITY_SD, 2u, tmp, 1u);

    /* Attitude error std dev with +/- 2pi rad scaled to [-32767, 32767] */
    tmp[0] = (int32_t)(error[9] * ((double)INT16_MAX / M_PI) * 0.5);
    tmp[1] = (int32_t)(error[10] * ((double)INT16_MAX / M_PI) * 0.5);
    tmp[2] = (int32_t)(error[11] * ((double)INT16_MAX / M_PI) * 0.5);
    //_set_estimate_value_i32(
    //    estimate_log, FCS_PARAMETER_ESTIMATED_ATTITUDE_SD, 2u, tmp, 3u);

    /* Wind velocity error std dev in cm/s */
    tmp[0] = (int32_t)(vector3_norm_d(&error[18]) * 1e2);
    //_set_estimate_value_i32(
    //    estimate_log, FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_SD, 2u, tmp, 1u);

    /* Static pressure in Pa */
    /* TODO -- determine based on reference pressure/alt */
    tmp[0] = (int32_t)static_pressure;
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_STATIC_PRESSURE, 4u, tmp, 1u);

    /* Static temperature in Kelvin */
    tmp[0] = (int32_t)static_temp;
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_ESTIMATED_STATIC_TEMP, 2u, tmp, 1u);

    /* Set WMM field */
    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, 3u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_ESTIMATED_WMM_FIELD);
    fcs_parameter_set_device_id(&param, 0);

    param.data.f32[X] = (float)field[X];
    param.data.f32[Y] = (float)field[Y];
    param.data.f32[Z] = (float)field[Z];

    fcs_log_add_parameter(estimate_log, &param);

    /* AHRS mode and status */
    tmp[0] = (int32_t)mode;
    tmp[1] = 0;
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_AHRS_MODE, 1u, tmp, 2u);

    _populate_ahrs_status(estimate_log, mode);

    estimate_log = fcs_exports_log_close(estimate_log);
    fcs_assert(!estimate_log);
}

static void _populate_ahrs_status(struct fcs_log_t *estimate_log,
enum fcs_mode_t ahrs_mode) {
    int32_t tmp[4];

    /*
    AHRS status 0 is the maximum time since last critical sensor update (for
    GPS and barometric pressure) -- if it exceeds a certain value flight
    should be terminated by the control system.
    */
    uint64_t t_since_last_sensor;
    t_since_last_sensor = ahrs_solution_time - ahrs_last_gps_time;
    if (ahrs_solution_time - ahrs_last_baro_time > t_since_last_sensor) {
        t_since_last_sensor = ahrs_solution_time - ahrs_last_baro_time;
    }

    /* Ignore sensor timeouts in any mode but active */
    if (ahrs_mode != FCS_MODE_ACTIVE) {
        t_since_last_sensor = 0;
    }

    tmp[0] = (int16_t)(t_since_last_sensor > INT16_MAX ?
                       INT16_MAX : t_since_last_sensor);

    /*
    AHRS status 1 is the time since the last reference pressure/GCS update --
    if it exceeds a certain value the control system should follow the loss
    of data link procedure.
    */
    uint64_t t_since_last_comms;
    t_since_last_comms = ahrs_solution_time - ahrs_last_comms_time;

    /* Ignore sensor timeouts in any mode but active */
    if (ahrs_mode != FCS_MODE_ACTIVE && ahrs_mode != FCS_MODE_SIMULATING) {
        t_since_last_comms = 0;
    }

    tmp[1] = (int16_t)(t_since_last_comms > INT16_MAX ?
                       INT16_MAX : t_since_last_comms);
    _set_estimate_value_i32(
        estimate_log, FCS_PARAMETER_AHRS_STATUS, 2u, tmp, 2u);
}

static bool _get_hal_sensor_value(const struct fcs_log_t *plog,
enum fcs_parameter_type_t param_type, double *restrict value,
double *restrict variance, size_t n) {
    bool got_variance;
    struct fcs_parameter_t param;

    if (fcs_parameter_find_by_type_and_device(plog, param_type, 0, &param)) {
        fcs_parameter_get_values_d(&param, value, n);

        got_variance = fcs_parameter_find_by_type_and_device(
            plog, (enum fcs_parameter_type_t)(param_type + 1u), 0, &param);
        fcs_assert(got_variance);

        fcs_parameter_get_values_d(&param, variance, n);

        return true;
    } else {
        return false;
    }
}

static void _set_estimate_value_i32(struct fcs_log_t *elog,
enum fcs_parameter_type_t param_type, uint8_t value_size,
const int32_t *restrict v, size_t n) {
    fcs_assert(elog);
    fcs_assert(v);
    fcs_assert(value_size == 1u || value_size == 2u || value_size == 4u);

    size_t i;
    struct fcs_parameter_t param;

    fcs_parameter_set_header(&param, FCS_VALUE_SIGNED,
                             (uint32_t)value_size << 3u, n);
    fcs_parameter_set_type(&param, param_type);
    fcs_parameter_set_device_id(&param, 0);

#define CLAMPVAL(x, mi, ma) (x >= mi ? (x <= ma ? x : ma) : mi)

    if (value_size == 1u) {
        for (i = 0; i < n; i++) {
            param.data.i8[i] = (int8_t)CLAMPVAL(v[i], INT8_MIN, INT8_MAX);
        }
    } else if (value_size == 2u) {
        for (i = 0; i < n; i++) {
            param.data.i16[i] = (int16_t)CLAMPVAL(v[i], INT16_MIN, INT16_MAX);
        }
    } else if (value_size == 4u) {
        for (i = 0; i < n; i++) {
            param.data.i32[i] = (int32_t)CLAMPVAL(v[i], INT32_MIN, INT32_MAX);
        }
    }

#undef CLAMPVAL

    fcs_log_add_parameter(elog, &param);
}

bool fcs_ahrs_set_mode(enum fcs_mode_t mode) {
    enum fcs_mode_t previous_mode = ahrs_mode;

    if (mode == ahrs_mode) {
        /* Always allow re-entering the same mode */
    } else if (mode == FCS_MODE_STARTUP_VALUE) {
        /* Invalid mode requested */
        return false;
    } else if (mode == FCS_MODE_INITIALIZING &&
            previous_mode != FCS_MODE_STARTUP_VALUE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_SIMULATING &&
            previous_mode != FCS_MODE_INITIALIZING &&
            previous_mode != FCS_MODE_CALIBRATING &&
            previous_mode != FCS_MODE_SAFE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_CALIBRATING &&
            previous_mode != FCS_MODE_INITIALIZING &&
            previous_mode != FCS_MODE_SIMULATING &&
            previous_mode != FCS_MODE_SAFE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_SAFE &&
            previous_mode != FCS_MODE_CALIBRATING &&
            previous_mode != FCS_MODE_ARMED &&
            previous_mode != FCS_MODE_ACTIVE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_ARMED &&
            previous_mode != FCS_MODE_SAFE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_ACTIVE &&
            previous_mode != FCS_MODE_ARMED &&
            previous_mode != FCS_MODE_HOLDING) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_HOLDING &&
            previous_mode != FCS_MODE_ACTIVE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_ABORT &&
            previous_mode != FCS_MODE_ARMED &&
            previous_mode != FCS_MODE_ACTIVE &&
            previous_mode != FCS_MODE_HOLDING) {
        /* Invalid mode transition */
        return false;
    }

    ahrs_mode = mode;
    ahrs_mode_start_time = ahrs_solution_time;

    switch (mode) {
        case FCS_MODE_INITIALIZING:
            _reset_state();
            ukf_choose_dynamics(UKF_MODEL_NONE);
            break;
        case FCS_MODE_SIMULATING:
            break;
        case FCS_MODE_CALIBRATING:
            /* Start up clean */
            _reset_state();
            ukf_choose_dynamics(UKF_MODEL_NONE);
            /* Trust gyro bias and attitude predictor less. */
            vector_set_d(&ahrs_process_noise[9], 1e-6, 3u);
            /* And wind velocity more. */
            vector_set_d(&ahrs_process_noise[18], 1e-9, 3u);
            vector_set_d(&ahrs_process_noise[21], 1e-6, 3u);
            break;
        case FCS_MODE_SAFE:
            ukf_choose_dynamics(UKF_MODEL_NONE);
            vector_set_d(&ahrs_process_noise[9], 7e-8, 3u);
            vector_set_d(&ahrs_process_noise[18], 1e-9, 3u);
            vector_set_d(&ahrs_process_noise[21], 1e-12, 3u);
            break;
        case FCS_MODE_ARMED:
            vector_set_d(&ahrs_process_noise[9], 7e-8, 3u);
            vector_set_d(&ahrs_process_noise[18], 1e-7, 3u);
            vector_set_d(&ahrs_process_noise[21], 1e-12, 3u);
            break;
        case FCS_MODE_ACTIVE:
            ukf_choose_dynamics(ahrs_dynamics_model);
            break;
        case FCS_MODE_HOLDING:
            break;
        case FCS_MODE_ABORT:
            break;
        case FCS_MODE_STARTUP_VALUE:
            fcs_assert(false);
    }

    return true;
}
