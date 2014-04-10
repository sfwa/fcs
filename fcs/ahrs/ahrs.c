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
#include <assert.h>

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
    1e-15, 1e-15, 1e-5, /* lat, lon, alt */
    7e-5, 7e-5, 7e-5, /* velocity N, E, D */
    2e-4, 2e-4, 2e-4, /* acceleration x, y, z */
    1e-9, 1e-9, 1e-9, /* attitude roll, pitch, yaw */
    3e-6, 3e-6, 3e-6, /* angular velocity roll, pitch, yaw */
    1e-3, 1e-3, 1e-3, /* angular acceleration roll, pitch, yaw */
    1e-5, 1e-5, 1e-5, /* wind velocity N, E, D */
    1.5e-12, 1.5e-12, 1.5e-12 /* gyro bias x, y, z */
};
static double ahrs_wmm_field_dir[3], ahrs_wmm_field_norm;
static uint64_t ahrs_solution_time;
static enum ukf_model_t ahrs_dynamics_model;
static uint64_t ahrs_mode_start_time;
static enum fcs_mode_t ahrs_mode;

/*
Check a vector for NaN values; return true if any are found, and false
otherwise
*/
inline static bool _vec_hasnan_d(const double *vec, size_t n) {
    for (; n; n--) {
        if (isnan(vec[n])) {
            return true;
        }
    }
    return false;
}

static void _update_wmm(double lla[3], double field[3], double *norm);
static bool _get_hal_sensor_value(const struct fcs_log_t *plog,
enum fcs_parameter_type_t param_type, double *restrict value,
double *restrict variance, size_t n);
static void _reset_state(void);

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24u);
    assert(ukf_config_get_control_dim() == 4u);
    assert(ukf_config_get_measurement_dim() == 20u);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    ahrs_solution_time = 0;
    ahrs_mode_start_time = 0;
    ahrs_wmm_field_dir[0] = 1.0;
    ahrs_wmm_field_dir[1] = ahrs_wmm_field_dir[2] = 0.0;
    ahrs_wmm_field_norm = 1.0;

    fcs_wmm_init();

    /* Reset/init the UKF */
    _reset_state();

    /* Initialize AHRS mode */
    ahrs_dynamics_model = UKF_MODEL_X8;
    fcs_ahrs_set_mode(FCS_MODE_INITIALIZING);
}

void fcs_ahrs_tick(void) {
    /*
    While copying measurement data to the UKF, get sensor error and geometry
    information so the sensor model parameters can be updated.
    */
    double v[3], speed;
    bool got_result, got_gps = false, got_reference_alt = false;
    struct ukf_ioboard_params_t params;
    struct fcs_parameter_t parameter;
    struct fcs_log_t *hal_log, *control_log, *estimate_log;

    /* Increment solution time */
    ahrs_solution_time++;

    /* Read virtual sensor values from the HAL log and pass them to the UKF */
    hal_log = fcs_exports_log_open(FCS_LOG_TYPE_SENSOR_HAL, 'r');
    assert(hal_log);

    memset(&params, 0, sizeof(params));
    params.accel_orientation[W] = 1.0;
    params.gyro_orientation[W] = 1.0;
    params.mag_orientation[W] = 1.0;

    /* Read sensor data from the measurement log, and pass it to the UKF */
    ukf_sensor_clear();

    /* Read accelerometer */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_ACCELEROMETER_XYZ, v,
                              params.accel_covariance, 3u)) {
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);

        got_result = fcs_parameter_find_by_type_and_device(
            hal_log, FCS_PARAMETER_HAL_ACCELEROMETER_OFFSET_XYZ, 0,
            &parameter);
        assert(got_result);

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
    }

    /* Read GPS position and velocity */
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_POSITION_LAT_LON, v,
                              params.gps_position_covariance, 2u) &&
            _get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_POSITION_ALT,
                                  &v[2], &params.gps_position_covariance[2],
                                  1u)) {
        ukf_sensor_set_gps_position(v[0], v[1], v[2]);
        got_gps = true;
    }
    if (_get_hal_sensor_value(hal_log, FCS_PARAMETER_HAL_VELOCITY_NED, v,
                              params.gps_velocity_covariance, 3u)) {
        ukf_sensor_set_gps_velocity(v[0], v[1], v[2]);
    }

    hal_log = fcs_exports_log_close(hal_log);
    assert(!hal_log);

    /* Use the latest WMM field vector (unit length) */
    vector_copy_d(params.mag_field, ahrs_wmm_field_dir, 3u);

    /*
    Run the UKF, taking sensor readings and current control position into
    account
    */
    ukf_set_params(&params);
    ukf_set_process_noise(ahrs_process_noise);

    /* Don't update the filter during initialization */
    if (ahrs_mode != FCS_MODE_INITIALIZING) {
        control_log = fcs_exports_log_open(FCS_LOG_TYPE_CONTROL, 'r');
        assert(control_log);

        got_result = fcs_parameter_find_by_type_and_device(
            control_log, FCS_PARAMETER_CONTROL_POS, 0, &parameter);
        if (got_result) {
            fcs_parameter_get_values_d(&parameter, v, 4u);
            ukf_iterate((float)AHRS_DELTA, v);
        }

        control_log = fcs_exports_log_close(control_log);
        assert(!control_log);
    }

    /* Copy the state out of the UKF */
    double state_values[25], error[24];
    ukf_get_state((struct ukf_state_t*)state_values);
    ukf_get_state_error(error);

    /* Validate the UKF state; if any values are NaN, reset it */
    if (_vec_hasnan_d(state_values, 25u) || _vec_hasnan_d(error, 24u)) {
        _reset_state();
    } else {
        /*
        Clear the wind speed estimate if we're not in active mode, or just the
        downward component of the wind if we are.
        */
        if (ahrs_mode != FCS_MODE_ACTIVE) {
            vector_set_d(&state_values[19], 0.0, 3u);
        } else {
            state_values[21] = 0.0;
        }
        ukf_set_state((struct ukf_state_t*)state_values);
    }

    /* Get a new WMM field estimate */
    _update_wmm(state_values, ahrs_wmm_field_dir, &ahrs_wmm_field_norm);

    /* TODO: Update the estimate log */
    estimate_log = fcs_exports_log_open(FCS_LOG_TYPE_ESTIMATE, 'w');
    assert(estimate_log);

/*
    FCS_PARAMETER_ESTIMATED_LAT_LON
    FCS_PARAMETER_ESTIMATED_ALT
    FCS_PARAMETER_ESTIMATED_VELOCITY_NED
    FCS_PARAMETER_ESTIMATED_ATTITUDE_ZYX
    FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED
    FCS_PARAMETER_ESTIMATED_GYRO_BIAS_XYZ
    FCS_PARAMETER_ESTIMATED_POS_SD
    FCS_PARAMETER_ESTIMATED_VELOCITY_SD
    FCS_PARAMETER_ESTIMATED_ATTITUDE_SD
    FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_SD
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_SD
    FCS_PARAMETER_ESTIMATED_STATIC_PRESSURE
    FCS_PARAMETER_ESTIMATED_STATIC_TEMP

    FCS_PARAMETER_AHRS_MODE
    FCS_PARAMETER_AHRS_STATUS
*/

    estimate_log = fcs_exports_log_close(estimate_log);
    assert(!estimate_log);

    /* Check the current mode and transition if necessary */
    if (ahrs_mode == FCS_MODE_STARTUP_VALUE) {
        fcs_ahrs_set_mode(FCS_MODE_INITIALIZING);
    } else if (ahrs_mode == FCS_MODE_INITIALIZING) {
        /*
        Transition out of initializing if we've received a GPS packet as well
        as a reference pressure from the GCS
        */
        if (got_gps && got_reference_alt) {
            fcs_ahrs_set_mode(FCS_MODE_CALIBRATING);
        }
    } else if (ahrs_mode == FCS_MODE_CALIBRATING) {
        /* Transition out of calibration mode after 30s */
        if (ahrs_solution_time - ahrs_mode_start_time > 30000) {
            fcs_ahrs_set_mode(FCS_MODE_SAFE);
        }
    } else if (ahrs_mode == FCS_MODE_ARMED) {
        /* Transition to active once speed exceeds 8m/s */
        speed = vector3_norm_d(&state_values[3]);
        if (speed > 8.0) {
            fcs_ahrs_set_mode(FCS_MODE_ACTIVE);
        }
    }
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

static void _update_wmm(double lla[3], double field[3], double *norm) {
    /* Calculate WMM field at current lat/lon/alt/time */
    if (!fcs_wmm_calculate_field(lla[0], lla[1], lla[2], 2014.0, field)) {
        fcs_global_counters.wmm_errors++;
    } else {
        *norm = vector3_norm_d(field);
        vector3_scale_d(field, 1.0 / *norm);

        /*
        WMM returns the field in nT; the magnetometer sensitivity is in Gauss
        (1G = 100 000nT). Convert the field norm to G.
        */
        *norm *= (1.0 / 100000.0);
    }
}

static bool _get_hal_sensor_value(const struct fcs_log_t *plog,
enum fcs_parameter_type_t param_type, double *restrict value,
double *restrict variance, size_t n) {
    bool got_variance;
    struct fcs_parameter_t param;

    if (fcs_parameter_find_by_type_and_device(plog, param_type, 0, &param)) {
        fcs_parameter_get_values_d(&param, value, n);

        got_variance = fcs_parameter_find_by_type_and_device(plog,
                                                             param_type + 1u,
                                                             0, &param);
        assert(got_variance);

        fcs_parameter_get_values_d(&param, variance, n);

        return true;
    } else {
        return false;
    }
}

bool fcs_ahrs_set_mode(enum fcs_mode_t mode) {
    enum fcs_mode_t previous_mode = ahrs_mode;

    if (mode == FCS_MODE_STARTUP_VALUE) {
        /* Invalid mode requested */
        return false;
    } else if (mode == FCS_MODE_INITIALIZING &&
            previous_mode != FCS_MODE_STARTUP_VALUE) {
        /* Invalid mode transition */
        return false;
    } else if (mode == FCS_MODE_CALIBRATING &&
            previous_mode != FCS_MODE_INITIALIZING &&
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
            ukf_choose_dynamics(UKF_MODEL_NONE);
            break;
        case FCS_MODE_CALIBRATING:
            /* Start up clean */
            _reset_state();
            /* Trust attitude and gyro bias predictors less. */
            vector_set_d(&ahrs_process_noise[9], 1e-5, 3u);
            vector_set_d(&ahrs_process_noise[21], 1e-7, 3u);
            break;
        case FCS_MODE_SAFE:
            ukf_choose_dynamics(UKF_MODEL_NONE);

            vector_set_d(&ahrs_process_noise[9], 1e-8, 3u);
            vector_set_d(&ahrs_process_noise[21], 1e-9, 3u);
            break;
        case FCS_MODE_ARMED:
            break;
        case FCS_MODE_ACTIVE:
            ukf_choose_dynamics(ahrs_dynamics_model);
            break;
        case FCS_MODE_HOLDING:
            break;
        case FCS_MODE_ABORT:
            break;
        case FCS_MODE_STARTUP_VALUE:
            assert(false);
    }

    return true;
}
