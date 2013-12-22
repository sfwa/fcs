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

#ifdef __TI_COMPILER_VERSION__
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_sem.h"
#endif

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "measurement.h"
#include "wmm.h"
#include "ahrs.h"

#define AHRS_DELTA 0.001

/*
Track the "actual" control position -- NMPC outputs desired position but
we limit the rate at which that can change to the relevant control_rate
value.

The control_scale value is the magnitude of the maximum control input.
*/
static double control_rate[4];
static double control_scale[4];

/* Global FCS state structure */
struct fcs_ahrs_state_t fcs_global_ahrs_state;

/* Macro to limit the absolute value of x to l, but preserve the sign */
#define limitabs(x, l) (x < 0.0 && x < -l ? -l : x > 0.0 && x > l ? l : x)

static void _fcs_ahrs_update_wmm(void);

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24);
    assert(ukf_config_get_control_dim() == 4);
    assert(ukf_config_get_measurement_dim() == 20);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);


#ifdef __TI_COMPILER_VERSION__
    /* Release the global state semaphore */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE] = 1u;
#endif

    fcs_wmm_init();

    /*
    TODO: read state back from fcs_global_ahrs_state, just in case we're
    booting up after an in-flight reset
    */
    struct ukf_state_t initial_state = {
        {-37.0 * (M_PI/180.0), 145.0 * (M_PI/180.0), 10.0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };

    /* Set up UKF */
    ukf_init();
    ukf_set_state(&initial_state);

    /*
    Copy the UKF default state to the global AHRS state structure to make sure
    everything's in sync
    */
    ukf_get_state((struct ukf_state_t*)&fcs_global_ahrs_state.lat);

    /*
    TODO: Configure default UKF covariance and sensor offsets
    covariance.accel_covariance = 81.0;
    covariance.gyro_covariance = 0.5 * (M_PI / 180.0);
    covariance.mag_covariance = 1.5;
    covariance.gps_position_covariance_h = 1e-11;
    covariance.gps_position_covariance_v = 225.0;
    covariance.gps_velocity_covariance_h = 9.0;
    covariance.gps_velocity_covariance_v = 49.0;
    covariance.pitot_covariance = 100.0;
    covariance.barometer_amsl_covariance = 4.0;
    */

    /* Calculate WMM field at current lat/lon/alt/time */
    _fcs_ahrs_update_wmm();

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
    memcpy(fcs_global_ahrs_state.ukf_process_noise, default_process_noise,
           sizeof(default_process_noise));
    fcs_global_ahrs_state.ukf_dynamics_model = UKF_MODEL_X8;

    ukf_set_process_noise(fcs_global_ahrs_state.ukf_process_noise);
    ukf_choose_dynamics(fcs_global_ahrs_state.ukf_dynamics_model);

    /* Update the TRICAL instance parameters based on the UKF configuration */
    uint8_t i;
    for (i = 0; i < FCS_AHRS_NUM_TRICAL_INSTANCES; i++) {
        TRICAL_init(&fcs_global_ahrs_state.trical_instances[i]);

        /*
        Set the norm to 1.0 because we pass a unit field vector to the UKF.
        */
        TRICAL_norm_set(&fcs_global_ahrs_state.trical_instances[i], 1.0f);
        TRICAL_noise_set(&fcs_global_ahrs_state.trical_instances[i], 1e-2f);
    }
}

void fcs_ahrs_tick(void) {
    /* Increment solution time */
    fcs_global_ahrs_state.solution_time++;

    _fcs_ahrs_update_wmm();

    /* Handle magnetometer calibration update based on new readings */
    struct fcs_calibration_t *restrict sensor_calibration_map =
        fcs_global_ahrs_state.calibration.sensor_calibration;
    TRICAL_instance_t *restrict instance;
    struct fcs_measurement_t mag_measurement;
    double mag_value[4];
    float mag_value_f[3];
    uint8_t i, j;
    for (i = 0; i < 2u; i++) {
        if (fcs_measurement_log_find(
                &fcs_global_ahrs_state.measurements,
                FCS_MEASUREMENT_TYPE_MAGNETOMETER, i, &mag_measurement)) {
            /* Get the measurement value */
            fcs_measurement_get_values(&mag_measurement, mag_value);

            /* Determine the sensor key for the calibration parameters */
            uint8_t sensor_key =
                ((i << FCS_MEASUREMENT_SENSOR_ID_OFFSET) &
                 FCS_MEASUREMENT_SENSOR_ID_MASK) |
                ((FCS_MEASUREMENT_TYPE_MAGNETOMETER <<
                  FCS_MEASUREMENT_SENSOR_TYPE_OFFSET) &
                 FCS_MEASUREMENT_SENSOR_TYPE_MASK);

            /* Update TRICAL instance parameters with the latest results */
            instance = &fcs_global_ahrs_state.trical_instances[i];

            /*
            FIXME: work out a better way to specify sensor sensitivity (or
            get TRICAL working with measurements further from 1.0)
            */
            mag_value_f[0] = (mag_value[0] / 2048.0);
            mag_value_f[1] = (mag_value[1] / 2048.0);
            mag_value_f[2] = (mag_value[2] / 2048.0);
            TRICAL_estimate_update(instance, mag_value_f);

            for (j = 0; j < 9u; j++) {
                if (isnan(instance->state[j])) {
                    /*
                    TRICAL has blown up -- reset this instance and ignore the
                    current reading.
                    */
                    TRICAL_reset(instance);
                    fcs_global_counters.trical_resets[i]++;
                    break;
                }
            }

            /*
            Copy the TRICAL calibration estimate to the magnetometer
            calibration
            */
            memcpy(sensor_calibration_map[sensor_key].params,
                   instance->state, 9u * sizeof(float));
        }
    }

    /*
    While copying measurement data to the UKF, get sensor error and geometry
    information so the sensor model parameters can be updated.
    */
    struct ukf_ioboard_params_t params = {
        {0, 0, 0, 1}, /* accel_orientation */
        {0, 0, 0}, /* accel_offset */
        {0, 0, 0, 1}, /* gyro_orientation */
        {0, 0, 0, 1} /* mag_orientation */
    };

    /* Use the latest WMM field vector */
    params.mag_field[0] = fcs_global_ahrs_state.wmm_field_dir[0];
    params.mag_field[1] = fcs_global_ahrs_state.wmm_field_dir[1];
    params.mag_field[2] = fcs_global_ahrs_state.wmm_field_dir[2];

    /* Read sensor data from the measurement log, and pass it to the UKF */
    double v[4], err, offset[3];

    ukf_sensor_clear();

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_ACCELEROMETER, v, &err, offset)) {
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);
        params.accel_covariance[0] = params.accel_covariance[1] =
            params.accel_covariance[2] = err * err;
        params.accel_offset[0] = offset[0];
        params.accel_offset[1] = offset[1];
        params.accel_offset[2] = offset[2];
    }

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_GYROSCOPE, v, &err, offset)) {
        ukf_sensor_set_gyroscope(v[0], v[1], v[2]);
        params.gyro_covariance[0] = params.gyro_covariance[1] =
            params.gyro_covariance[2] = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_MAGNETOMETER, v, &err, offset)) {
        ukf_sensor_set_magnetometer(v[0], v[1], v[2]);
        params.mag_covariance[0] = params.mag_covariance[1] =
            params.mag_covariance[2] = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_PITOT, v, &err, offset)) {
        ukf_sensor_set_pitot_tas(v[0]);
        params.pitot_covariance = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_PRESSURE_TEMP, v, &err, offset)) {
        ukf_sensor_set_barometer_amsl(v[0]);
        params.barometer_amsl_covariance = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_GPS_POSITION, v, &err, offset)) {
        ukf_sensor_set_gps_position(v[0], v[1], v[2]);
        params.gps_position_covariance[0] =
            params.gps_position_covariance[1] = err * err;
        params.gps_position_covariance[2] = 1600.0;
    }

    if (fcs_measurement_log_get_calibrated_value(
            &fcs_global_ahrs_state.measurements,
            &fcs_global_ahrs_state.calibration,
            FCS_MEASUREMENT_TYPE_GPS_VELOCITY, v, &err, offset)) {
        ukf_sensor_set_gps_velocity(v[0], v[1], v[2]);
        params.gps_velocity_covariance[0] =
            params.gps_velocity_covariance[1] = err * err;
        params.gps_velocity_covariance[2] = 49.0;
    }

    /* TODO: Read control set values from NMPC */
    double control_set[4] = { 0.0, 0.0, 0.0, 0.0 };

    /*
    Work out the nominal current control position, taking into account the
    control response time configured in control_rates. Log the result.
    */
    struct fcs_measurement_t control_log;
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4; i++) {
        double delta = control_set[i] - fcs_global_ahrs_state.control_pos[i],
               limit = control_rate[i] * AHRS_DELTA;
        fcs_global_ahrs_state.control_pos[i] += limitabs(delta, limit);
        control_log.data.i16[i] =
            (int16_t)(fcs_global_ahrs_state.control_pos[i] / control_scale[i]
                      * INT16_MAX);
    }

    control_log.header = (8u << FCS_MEASUREMENT_HEADER_LENGTH_OFFSET)
                         & FCS_MEASUREMENT_HEADER_LENGTH_MASK;
    control_log.sensor = FCS_MEASUREMENT_TYPE_CONTROL_POS;
    fcs_measurement_log_add(&fcs_global_ahrs_state.measurements,
                            &control_log);

    /*
    Run the UKF, taking sensor readings and current control position into
    account
    */
    ukf_set_params(&params);
    ukf_iterate(AHRS_DELTA, fcs_global_ahrs_state.control_pos);

    /* Copy the global state out of the UKF and validate it */
    bool ukf_valid = true;

    double state_values[25], covariance[24];
    ukf_get_state((struct ukf_state_t*)state_values);
    ukf_get_state_covariance_diagonal(covariance);

    #pragma MUST_ITERATE(24, 24);
    for (i = 0; i < 24u; i++) {
        if (isnan(state_values[i]) || isnan(covariance[i])) {
            ukf_valid = false;
        }
    }
    if (isnan(state_values[24])) {
        ukf_valid = false;
    }

    if (ukf_valid) {
#ifdef __TI_COMPILER_VERSION__
        /*
        Use a semaphore to prevent the NMPC code accessing the state while we're
        updating it.
        */
        volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
        uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE];
        assert(sem_val == 1u);
#endif

        /* If that's all OK, update the global state */
        memcpy(&fcs_global_ahrs_state.lat, state_values,
               sizeof(state_values));
        memcpy(&fcs_global_ahrs_state.lat_covariance, covariance,
               sizeof(covariance));

#ifdef __TI_COMPILER_VERSION__
        /* Release the semaphore */
        semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE] = 1u;
#endif
    } else {
        struct ukf_state_t reset_state = {
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0, 1},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}
        };

        /* Copy the last position and attitude */
        reset_state.position[0] = fcs_global_ahrs_state.lat;
        reset_state.position[1] = fcs_global_ahrs_state.lon;
        reset_state.position[2] = fcs_global_ahrs_state.alt;
        memcpy(reset_state.attitude, fcs_global_ahrs_state.attitude,
               sizeof(double) * 4u);

        /* TODO: if gyro bias is sane, copy that too */

        fcs_global_counters.ukf_resets++;
        ukf_init();
        ukf_set_state(&reset_state);

        /* Update the output state and covariance with the latest values */
        memcpy(&fcs_global_ahrs_state.lat, &state_values,
               sizeof(state_values));
        ukf_get_state_covariance_diagonal(
            &fcs_global_ahrs_state.lat_covariance);
    }
}

static void _fcs_ahrs_update_wmm(void) {
    /* Calculate WMM field at current lat/lon/alt/time */
    bool result;
    result = fcs_wmm_calculate_field(
        fcs_global_ahrs_state.lat, fcs_global_ahrs_state.lon,
        fcs_global_ahrs_state.alt, 2014.0,
        fcs_global_ahrs_state.wmm_field_dir);
    if (!result) {
        fcs_global_counters.wmm_errors++;
    } else {
        fcs_global_ahrs_state.wmm_field_norm =
            vector3_norm_d(fcs_global_ahrs_state.wmm_field_dir);

        double norm_inv;
        norm_inv = 1.0 / fcs_global_ahrs_state.wmm_field_norm;
        fcs_global_ahrs_state.wmm_field_dir[0] *= norm_inv;
        fcs_global_ahrs_state.wmm_field_dir[1] *= norm_inv;
        fcs_global_ahrs_state.wmm_field_dir[2] *= norm_inv;
    }
}
