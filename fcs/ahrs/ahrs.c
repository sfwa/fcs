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

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "../ukf/cukf.h"
#include "../nmpc/cnmpc.h"
#include "../control/control.h"
#include "../exports/exports.h"
#include "measurement.h"
#include "wmm.h"
#include "ahrs.h"

#define AHRS_DELTA 0.001

/* Global FCS state structure */
struct fcs_ahrs_state_t fcs_global_ahrs_state;

/* Macro to limit the absolute value of x to l, but preserve the sign */
#define limitabs(x, l) (x < 0.0 && x < -l ? -l : x > 0.0 && x > l ? l : x)

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

inline static bool _vec_hasnan_f(const float *vec, size_t n) {
    for (; n; n--) {
        if (isnan(vec[n])) {
            return true;
        }
    }
    return false;
}

static void _update_wmm(struct fcs_ahrs_state_t *state);
static void _set_ukf_accelerometer(struct fcs_ahrs_state_t *state,
double *variance, double *offset);
static void _set_ukf_gyroscope(struct fcs_ahrs_state_t *state,
double *variance);
static void _set_ukf_magnetometer(struct fcs_ahrs_state_t *state,
double *variance);
static void _set_ukf_pitot(struct fcs_ahrs_state_t *state, double *variance);
static void _set_ukf_barometer(struct fcs_ahrs_state_t *state,
double *variance);
static void _set_ukf_gps_position(struct fcs_ahrs_state_t *state,
double *variance);
static void _set_ukf_gps_velocity(struct fcs_ahrs_state_t *state,
double *variance);
static void _update_gps_info(struct fcs_ahrs_state_t *state);
static void _update_control_positions(struct fcs_ahrs_state_t* state);
static void _pitot_calibration(struct fcs_ahrs_state_t *state);
static void _barometer_calibration(struct fcs_ahrs_state_t *state);
static void _magnetometer_calibration(struct fcs_ahrs_state_t *state);
static void _accelerometer_calibration(struct fcs_ahrs_state_t *state);
static void _reset_state(struct fcs_ahrs_state_t *state);
static void _update_state(struct fcs_ahrs_state_t *state,
double *restrict ukf_state, double *restrict ukf_error);

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24u);
    assert(ukf_config_get_control_dim() == 4u);
    assert(ukf_config_get_measurement_dim() == 20u);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    fcs_wmm_init();

    /* TODO: don't reset position if any of the entries are non-zero */
    fcs_global_ahrs_state.lat = -37.8136 * M_PI / 180.0;
    fcs_global_ahrs_state.lon = 144.9631 * M_PI / 180.0;
    fcs_global_ahrs_state.alt = 100.0;

    /* TODO: don't reset attitude if any of the entries are non-zero */
    vector_set_d(fcs_global_ahrs_state.attitude, 0, 4u);
    fcs_global_ahrs_state.attitude[3] = 1.0;

    /* Reset/init the UKF */
    _reset_state(&fcs_global_ahrs_state);

    /* Set the UKF model */
    double default_process_noise[] = {
        1e-15, 1e-15, 1e-5, /* lat, lon, alt */
        7e-5, 7e-5, 7e-5, /* velocity N, E, D */
        2e-4, 2e-4, 2e-4, /* acceleration x, y, z */
        1e-9, 1e-9, 1e-9, /* attitude roll, pitch, yaw */
        3e-3, 3e-3, 3e-3, /* angular velocity roll, pitch, yaw */
        1e-3, 1e-3, 1e-3, /* angular acceleration roll, pitch, yaw */
        1e-5, 1e-5, 1e-5, /* wind velocity N, E, D */
        1.5e-12, 1.5e-12, 1.5e-12 /* gyro bias x, y, z */
    };
    vector_copy_d(
        fcs_global_ahrs_state.ukf_process_noise, default_process_noise, 24u);

    /* Initialize AHRS mode */
    fcs_global_ahrs_state.ukf_dynamics_model = UKF_MODEL_X8;
    fcs_ahrs_set_mode(FCS_MODE_INITIALIZING);

    /*
    Reset reference pressure and altitude. Until reference_pressure is set to
    a non-zero value (generally by the GCS), we don't leave the initializing
    state.
    */
    fcs_global_ahrs_state.reference_pressure = 0.0;
    fcs_global_ahrs_state.reference_alt = 0.0;

    /* Reset the vehicle aerodynamic environment state */
    fcs_global_ahrs_state.aero_static_pressure = STANDARD_PRESSURE;
    fcs_global_ahrs_state.aero_static_temp = STANDARD_TEMP;
    fcs_global_ahrs_state.aero_dynamic_pressure = 0.0;

    /*
    Update the TRICAL instance parameters. Instances 0 and 1 are
    magnetometers; instances 2 and 3 are accelerometers.
    */
    uint8_t i;
    for (i = 0; i < FCS_AHRS_NUM_TRICAL_INSTANCES; i++) {
        TRICAL_init(&fcs_global_ahrs_state.trical_instances[i]);

        /*
        Set the norm to 1.0, because the sensor calibration scale factor is
        set such that TRICAL will always work with (theoretically) unit
        vectors.
        */
        TRICAL_norm_set(&fcs_global_ahrs_state.trical_instances[i], 1.0f);
        TRICAL_noise_set(&fcs_global_ahrs_state.trical_instances[i],
                         i < 2u ? 1e-3f : 1.0f);
    }
}

void fcs_ahrs_tick(void) {
    /* Increment solution time */
    fcs_global_ahrs_state.solution_time++;

    _update_wmm(&fcs_global_ahrs_state);
    _magnetometer_calibration(&fcs_global_ahrs_state);

    /*
    Run TRICAL on the current accelerometer results when in calibration mode,
    and call the other sensor calibration handlers
    */
    if (fcs_global_ahrs_state.mode == FCS_MODE_CALIBRATING) {
        _accelerometer_calibration(&fcs_global_ahrs_state);
        _pitot_calibration(&fcs_global_ahrs_state);
        _barometer_calibration(&fcs_global_ahrs_state);
    }

    /*
    While copying measurement data to the UKF, get sensor error and geometry
    information so the sensor model parameters can be updated.
    */
    struct ukf_ioboard_params_t params;
    memset(&params, 0, sizeof(params));
    params.accel_orientation[W] = 1.0;
    params.gyro_orientation[W] = 1.0;
    params.mag_orientation[W] = 1.0;

    /* Read sensor data from the measurement log, and pass it to the UKF */
    ukf_sensor_clear();

    _set_ukf_accelerometer(&fcs_global_ahrs_state, params.accel_covariance,
                           params.accel_offset);
    _set_ukf_gyroscope(&fcs_global_ahrs_state, params.gyro_covariance);
    _set_ukf_magnetometer(&fcs_global_ahrs_state, params.mag_covariance);
    _set_ukf_pitot(&fcs_global_ahrs_state, &params.pitot_covariance);
    _set_ukf_barometer(
        &fcs_global_ahrs_state, &params.barometer_amsl_covariance);
    _set_ukf_gps_position(
        &fcs_global_ahrs_state, params.gps_position_covariance);
    _set_ukf_gps_velocity(
        &fcs_global_ahrs_state, params.gps_velocity_covariance);

    _update_gps_info(&fcs_global_ahrs_state);
    _update_control_positions(&fcs_global_ahrs_state);

    /* Use the latest WMM field vector (unit length) */
    vector_copy_d(params.mag_field, fcs_global_ahrs_state.wmm_field_dir, 3u);

    /*
    Run the UKF, taking sensor readings and current control position into
    account
    */
    ukf_set_params(&params);
    ukf_set_process_noise(fcs_global_ahrs_state.ukf_process_noise);

    /* Don't update the filter during initialization */
    if (fcs_global_ahrs_state.mode != FCS_MODE_INITIALIZING) {
        ukf_iterate((float)AHRS_DELTA, fcs_global_ahrs_state.control_pos);
    }

    /* Copy the global state out of the UKF */
    double state_values[25], error[24];
    ukf_get_state((struct ukf_state_t*)state_values);
    ukf_get_state_error(error);

    /* Clear the wind speed estimate if we're not in active mode */
    if (fcs_global_ahrs_state.mode != FCS_MODE_ACTIVE) {
        vector_set_d(&state_values[19], 0.0, 3u);
        ukf_set_state((struct ukf_state_t*)state_values);
    }

    /* Validate the UKF state; if any values are NaN, reset it */
    if (_vec_hasnan_d(state_values, 25u) || _vec_hasnan_d(error, 24u)) {
        _reset_state(&fcs_global_ahrs_state);
    } else {
        /* Update the global state structure */
        _update_state(&fcs_global_ahrs_state, state_values, error);
    }

    /* Check the current mode and transition if necessary */
    if (fcs_global_ahrs_state.mode == FCS_MODE_STARTUP_VALUE) {
        fcs_ahrs_set_mode(FCS_MODE_INITIALIZING);
    } else if (fcs_global_ahrs_state.mode == FCS_MODE_INITIALIZING) {
        /*
        Transition out of initializing if we've received a GPS packet as well
        as a reference pressure from the GCS
        */
        if (fcs_global_ahrs_state.last_gps_time > 0 &&
                fcs_global_ahrs_state.reference_pressure > 0.0) {
            fcs_ahrs_set_mode(FCS_MODE_CALIBRATING);
        }
    } else if (fcs_global_ahrs_state.mode == FCS_MODE_CALIBRATING) {
        /* Transition out of calibration mode after 30s */
        if (fcs_global_ahrs_state.solution_time -
                fcs_global_ahrs_state.mode_start_time > 30000) {
            fcs_ahrs_set_mode(FCS_MODE_SAFE);
        }
    } else if (fcs_global_ahrs_state.mode == FCS_MODE_ARMED) {
        /* Transition to active once speed exceeds 5m/s */
        double speed = vector3_norm_d(fcs_global_ahrs_state.velocity);
        if (speed > 5.0) {
            fcs_ahrs_set_mode(FCS_MODE_ACTIVE);
        }
    }
}

static void _reset_state(struct fcs_ahrs_state_t *state) {
    size_t i;

    /*
    Reset critical UKF parameters (position, attitude, gyro bias) based on
    their previous values, and clear everything else.
    */
    struct ukf_state_t reset_state;
    memset(&reset_state, 0, sizeof(reset_state));

    /*
    Copy the last position and attitude; if gyro bias is sane, copy that too
    */
    if (!_vec_hasnan_d(&state->lat, 3u)) {
        vector_copy_d(reset_state.position, &state->lat, 3u);
    }

    if (!_vec_hasnan_d(state->attitude, 4u)) {
        vector_copy_d(reset_state.attitude, state->attitude, 4u);
    }

    double bias;
    #pragma MUST_ITERATE(3, 3)
    for (i = 0; i < 3u; i++) {
        bias = state->gyro_bias[i];
        if (!isnan(bias) && fabs(bias) < M_PI * 0.1) {
            reset_state.gyro_bias[i] = bias;
        }
    }

    ukf_init();
    ukf_set_state(&reset_state);

    double error[24];
    ukf_get_state_error(error);

    /* Update the output state and error with the latest values */
    _update_state(&fcs_global_ahrs_state, (double*)&reset_state, error);

    fcs_global_counters.ukf_resets++;
}

static void _update_state(struct fcs_ahrs_state_t *state,
double *restrict ukf_state, double *restrict ukf_error) {
    vector_copy_d(&state->lat, ukf_state, 25u);
    vector_copy_d(&state->lat_error, ukf_error, 24u);

    fcs_exports_send_state();
}

static void _update_wmm(struct fcs_ahrs_state_t *state) {
    /* Calculate WMM field at current lat/lon/alt/time */
    bool result;
    result = fcs_wmm_calculate_field(state->lat, state->lon, state->alt,
                                     2014.0, state->wmm_field_dir);
    if (!result) {
        fcs_global_counters.wmm_errors++;
    } else {
        state->wmm_field_norm = vector3_norm_d(state->wmm_field_dir);
        vector3_scale_d(state->wmm_field_dir, 1.0 / state->wmm_field_norm);

        /*
        WMM returns the field in nT; the magnetometer sensitivity is in Gauss
        (1G = 100 000nT). Convert the field norm to G.
        */
        state->wmm_field_norm *= (1.0 / 100000.0);
    }
}

static void _set_ukf_accelerometer(struct fcs_ahrs_state_t *state,
double *variance, double *offset) {
    double v[4], err;
    bool got_values;

    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_ACCELEROMETER, v, &err, offset, 1.0);
    if (got_values) {
        /* Accelerometer output is in g, convert to m/s^2 */
        vector3_scale_d(v, G_ACCEL);
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);

        /*
        Add a relative error term to the covariance to account for scale
        factor error when the accelerometer reading differs from the bias
        calibration point (0, 0, -1).

        This isn't the right way to do it as the UKF assumes zero-mean error,
        but in practice it's OK.
        */
        vector3_scale_d(v, 0.1);
        variance[0] = err + absval(v[0]);
        variance[0] *= variance[0];

        variance[1] = err + absval(v[1]);
        variance[1] *= variance[1];

        variance[2] = err + absval(v[2] + G_ACCEL * 0.1);
        variance[2] *= variance[2];

        state->last_accelerometer_time = state->solution_time;
    }
}

static void _set_ukf_gyroscope(struct fcs_ahrs_state_t *state,
double *variance) {
    double v[4], err;
    bool got_values;

    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_GYROSCOPE, v, &err, NULL, 1.0);
    if (got_values) {
        ukf_sensor_set_gyroscope(v[0], v[1], v[2]);

        /*
        Add a relative error term, as above. Remove this if gyro scale factor
        error is included in the process model.
        */
        vector3_scale_d(v, 0.03);
        variance[0] = err + absval(v[0]);
        variance[0] *= variance[0];

        variance[1] = err + absval(v[1]);
        variance[1] *= variance[1];

        variance[2] = err + absval(v[2]);
        variance[2] *= variance[2];

        state->last_gyroscope_time = state->solution_time;
    }
}

static void _set_ukf_magnetometer(struct fcs_ahrs_state_t *state,
double *variance) {
    double v[4], err;
    bool got_values;
    double field_norm_inv = 1.0 / state->wmm_field_norm;

    /*
    We need to pre-scale the sensor reading by the current WMM field magnitude
    to work with the calibration params.
    */
    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_MAGNETOMETER, v, &err, NULL, field_norm_inv);
    if (got_values) {
        /*
        The calibration scales the magnetometer value to unity expectation.
        Scale error by the same amount, so the units of error are Gauss.
        */
        ukf_sensor_set_magnetometer(v[0], v[1], v[2]);
        err *= field_norm_inv;
        vector_set_d(variance, err * err, 3);

        state->last_magnetometer_time = state->solution_time;
    }
}

static void _set_ukf_pitot(struct fcs_ahrs_state_t *state, double *variance) {
    double v[4], err, tas;
    bool got_values;

    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_PITOT, v, &err, NULL, 1.0);
    if (got_values) {
        tas = airspeed_from_pressure_temp(state->aero_static_pressure, v[0],
                                          state->aero_static_temp);
        ukf_sensor_set_pitot_tas(tas);

        /* Allow for 10% scale factor error */
        err += absval(v[0]) * 0.1;
        *variance = err * err;

        state->last_pitot_time = state->solution_time;

        /* Update the dynamic pressure -- simple moving average for now */
        state->aero_dynamic_pressure +=
            0.5 * (v[0] - state->aero_dynamic_pressure);
    }
}

static void _set_ukf_barometer(struct fcs_ahrs_state_t *state,
double *variance) {
    double v[4], err, alt;
    bool got_values;

    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_PRESSURE_TEMP, v, &err, NULL, 1.0);
    if (got_values) {
        /*
        Calculate pressure differential between measured pressure and GCS
        pressure, then convert that to metres by dividing by 0.12.

        Add the GCS altitude above the ellipsoid to the result, so the final
        value we pass to the UKF is our altitude above ellipsoid referenced
        to current GCS pressure.
        */
        alt = altitude_diff_from_pressure_diff(state->reference_pressure,
                                               v[0], state->aero_static_temp);
        ukf_sensor_set_barometer_amsl(alt + state->reference_alt);

        /* Allow for 3% scale factor error in altitude */
        err += absval(alt) * 0.03;
        *variance = err * err;

        state->last_barometer_time = state->solution_time;

        /* Update the static pressure -- moving average for now */
        state->aero_static_pressure +=
            0.5 * (v[0] - state->aero_static_pressure);
    }
}

static void _set_ukf_gps_position(struct fcs_ahrs_state_t *state,
double *variance) {
    double v[4], err;
    bool got_values;

    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_GPS_POSITION, v, &err, NULL, 1.0);
    if (got_values) {
        ukf_sensor_set_gps_position(v[0], v[1], v[2]);
        vector_set_d(variance, err * err, 2);
        variance[2] = 225.0;

        state->last_gps_time = state->solution_time;
    }
}

static void _set_ukf_gps_velocity(struct fcs_ahrs_state_t *state,
double *variance) {
    double v[4], err;
    bool got_values;

    got_values = fcs_measurement_log_get_calibrated_value(
        &state->measurements, &state->calibration,
        FCS_MEASUREMENT_TYPE_GPS_VELOCITY, v, &err, NULL, 1.0);
    if (got_values) {
        ukf_sensor_set_gps_velocity(v[0], v[1], v[2]);
        vector_set_d(variance, err * err, 2);
        variance[2] = 49.0;
    }
}

static void _update_gps_info(struct fcs_ahrs_state_t *state) {
    /*
    Update GPS info parameters -- store the number of SVs tracked by connected
    units, and the PDOP for each unit
    */
    struct fcs_measurement_t gps_info_measurement;
    bool result;
    uint8_t i;
    for (i = 0; i < 2u; i++) {
        result = fcs_measurement_log_find(&state->measurements,
                                          FCS_MEASUREMENT_TYPE_GPS_INFO, i,
                                          &gps_info_measurement);
        if (result) {
            state->gps_num_svs[i] = gps_info_measurement.data.u8[1];
            /* PDOP is in metres */
            state->gps_pdop[i] = gps_info_measurement.data.u8[2];
        }
    }
}

static void _update_control_positions(struct fcs_ahrs_state_t* state) {
    /*
    Work out the nominal current control position, taking into account the
    control response time configured in control_rates.
    */
    uint8_t i;
    struct fcs_control_output_t control;

    fcs_exports_recv_control(&control);

    #pragma MUST_ITERATE(FCS_CONTROL_CHANNELS, FCS_CONTROL_CHANNELS)
    for (i = 0; i < FCS_CONTROL_CHANNELS; i++) {
        state->control_pos[i] += limitabs(
            control.values[i] - state->control_pos[i],
            control.rates[i] * AHRS_DELTA);
    }
}

static void _pitot_calibration(struct fcs_ahrs_state_t *state) {
    struct fcs_measurement_log_t *mlog = &state->measurements;
    struct fcs_calibration_t *calibration;
    struct fcs_measurement_t measurement;

    double pitot_value[4];
    uint8_t i;

    for (i = 0; i < 2u; i++) {
        if (fcs_measurement_log_find(mlog, FCS_MEASUREMENT_TYPE_PITOT, i,
                                     &measurement)) {
            /* Get the measurement value */
            fcs_measurement_get_values(&measurement, pitot_value);

            /* Find the calibration parameters */
            calibration = fcs_measurement_get_calibration(
                &state->calibration, FCS_MEASUREMENT_TYPE_PITOT, i);

            /*
            Update bias based on current sensor value, assuming the true
            reading should be 0. This is just a weighted moving average, with
            convergence taking a few seconds.
            */
            calibration->params[0] +=
                0.001 * ((pitot_value[0] * calibration->scale_factor) -
                         calibration->params[0]);
        }
    }
}

static void _barometer_calibration(struct fcs_ahrs_state_t *state) {
    struct fcs_measurement_log_t *mlog = &state->measurements;
    struct fcs_calibration_t *calibration;
    struct fcs_measurement_t measurement;

    double barometer_value[4];
    uint8_t i;

    for (i = 0; i < 2u; i++) {
        if (fcs_measurement_log_find(mlog, FCS_MEASUREMENT_TYPE_PRESSURE_TEMP,
                                     i, &measurement)) {
            /* Get the measurement value */
            fcs_measurement_get_values(&measurement, barometer_value);

            /* Find the calibration parameters */
            calibration = fcs_measurement_get_calibration(
                &state->calibration, FCS_MEASUREMENT_TYPE_PRESSURE_TEMP, i);

            /*
            Update bias based on current sensor value, assuming the true
            reading should be the same as the current reference pressure
            (valid since we're looking for height above our starting
            position).

            The GCS sensor will generally be elevated by a metre or two, but
            this code will simply treat that as a sensor bias and compensate
            automatically.
            */
            calibration->params[0] +=
                0.001 * ((barometer_value[0] * calibration->scale_factor) -
                         state->reference_pressure - calibration->params[0]);
        }
    }
}

static void _magnetometer_calibration(struct fcs_ahrs_state_t *state) {
    /*
    Handle magnetometer calibration update based on new readings -- we do this
    regardless of calibration mode
    */
    struct fcs_measurement_log_t *mlog = &state->measurements;
    struct fcs_calibration_t *calibration;
    struct fcs_measurement_t measurement;
    TRICAL_instance_t *instance;

    double mag_value[4], expected_field[3], scale_factor, delta_angle,
           field_norm_inv = 1.0f / state->wmm_field_norm;
    float mag_value_f[3], expected_field_f[3];
    uint8_t i;

    /*
    Rotate the WMM field by the current attitude to get the expected field
    direction for these readings
    */
    quaternion_vector3_multiply_d(expected_field, state->attitude,
                                  state->wmm_field_dir);
    vector_f_from_d(expected_field_f, expected_field, 3u);

    for (i = 0; i < 2u; i++) {
        if (fcs_measurement_log_find(mlog, FCS_MEASUREMENT_TYPE_MAGNETOMETER,
                                     i, &measurement)) {
            /*
            If the current attitude is too close to the attitude at which this
            TRICAL instance was last updated, skip calibration this time
            */
            delta_angle = quaternion_quaternion_angle_d(
                    state->attitude, state->trical_update_attitude[i]);
            if (delta_angle < 3.0 * M_PI / 180.0) {
                continue;
            }

            /* Get the measurement value */
            fcs_measurement_get_values(&measurement, mag_value);

            /* Find the calibration parameters */
            calibration = fcs_measurement_get_calibration(
                &state->calibration, FCS_MEASUREMENT_TYPE_MAGNETOMETER, i);

            /*
            Update TRICAL instance parameters with the latest results. Scale
            the magnetometer reading such that the expected magnitude is the
            unit vector, by dividing by the current WMM field strength in
            Gauss.
            */
            instance = &state->trical_instances[i];

            /*
            Copy the current sensor calibration to the TRICAL instance state
            so that any external changes to the calibration are captured.
            */
            vector_copy_f(instance->state, calibration->params, 9u);

            scale_factor = field_norm_inv * calibration->scale_factor;
            vector3_scale_d(mag_value, scale_factor);
            vector_f_from_d(mag_value_f, mag_value, 3u);

            TRICAL_estimate_update(instance, mag_value_f, expected_field_f);

            if (_vec_hasnan_f(instance->state, 12u)) {
                /* TRICAL has blown up -- reset this instance. */
                TRICAL_reset(instance);
                fcs_global_counters.trical_resets[i + 2u]++;
            }

            /*
            Copy the TRICAL calibration estimate to the magnetometer
            calibration
            */
            vector_copy_f(calibration->params, instance->state, 9u);

            /*
            Record the attitude at which this TRICAL instance was last updated
            so that we can space out calibration updates
            */
            vector_copy_d(
                state->trical_update_attitude[i], state->attitude, 4u);
        }
    }
}

static void _accelerometer_calibration(struct fcs_ahrs_state_t *state) {
    struct fcs_measurement_log_t *mlog = &state->measurements;
    struct fcs_calibration_t *calibration;
    struct fcs_measurement_t measurement;
    TRICAL_instance_t *instance;

    double accel_value[4];
    float accel_value_f[3], g_field[] = { 0.0, 0.0, -1.0 };
    uint8_t i;

    for (i = 0; i < 2u; i++) {
        if (fcs_measurement_log_find(mlog, FCS_MEASUREMENT_TYPE_ACCELEROMETER,
                                     i, &measurement)) {
            /* Get the accelerometer value */
            fcs_measurement_get_values(&measurement, accel_value);

            /* Find the calibration parameters */
            calibration = fcs_measurement_get_calibration(
                &state->calibration, FCS_MEASUREMENT_TYPE_ACCELEROMETER, i);

            /*
            Update TRICAL instance parameters with the latest results.

            The accelerometer TRICAL instances are 3 and 4.
            */
            instance = &state->trical_instances[i + 2u];

            /*
            Copy the current sensor calibration to the TRICAL instance
            state so that any external changes to the calibration are
            captured.
            */
            vector_copy_f(instance->state, calibration->params, 9u);

            vector3_scale_d(accel_value, calibration->scale_factor);
            vector_f_from_d(accel_value_f, accel_value, 3u);

            /*
            If the vehicle is level, we can assume that bias accounts for
            essentially the entire deviation from a reading of (0, 0, -1).

            Strictly that's not quite true, as the Z-axis may have scale
            error, but it'll get us pretty close.

            Since we're level, we can also set the expected field
            direction to straight down.
            */
            if (instance->state[0] == 0.0 || instance->state[1] == 0.0 ||
                    instance->state[2] == 0.0) {
                /*
                TODO: average this out over a few ticks to avoid sensitivity
                to noise on startup
                */
                vector_copy_f(instance->state, accel_value_f, 3u);
                instance->state[2] += 1.0f;
            }

            TRICAL_estimate_update(instance, accel_value_f, g_field);

            if (_vec_hasnan_f(instance->state, 12u)) {
                /* TRICAL has blown up -- reset this instance. */
                TRICAL_reset(instance);
                fcs_global_counters.trical_resets[i + 2u]++;
            }

            /*
            Copy the TRICAL calibration estimate to the accelerometer
            calibration
            */
            vector_copy_f(calibration->params, instance->state, 9u);
        }
    }
}

bool fcs_ahrs_set_mode(enum fcs_mode_t mode) {
    enum fcs_mode_t previous_mode = fcs_global_ahrs_state.mode;

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

    fcs_global_ahrs_state.mode = mode;
    fcs_global_ahrs_state.mode_start_time =
            fcs_global_ahrs_state.solution_time;

    switch (mode) {
        case FCS_MODE_INITIALIZING:
            ukf_choose_dynamics(UKF_MODEL_NONE);
            break;
        case FCS_MODE_CALIBRATING:
            /* Start up clean */
            _reset_state(&fcs_global_ahrs_state);
            /* Trust attitude and gyro bias predictors less. */
            vector_set_d(
                &fcs_global_ahrs_state.ukf_process_noise[9], 1e-5, 3u);
            vector_set_d(
                &fcs_global_ahrs_state.ukf_process_noise[21], 1e-7, 3u);
            break;
        case FCS_MODE_SAFE:
            ukf_choose_dynamics(UKF_MODEL_NONE);

            vector_set_d(
                &fcs_global_ahrs_state.ukf_process_noise[9], 1e-8, 3u);
            vector_set_d(
                &fcs_global_ahrs_state.ukf_process_noise[21], 1e-9, 3u);
            break;
        case FCS_MODE_ARMED:
            break;
        case FCS_MODE_ACTIVE:
            ukf_choose_dynamics(fcs_global_ahrs_state.ukf_dynamics_model);
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
