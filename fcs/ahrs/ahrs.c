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
#include "../comms/comms.h"
#include "../ukf/cukf.h"
#include "../drivers/stream.h"
#include "../stats/stats.h"
#include "../piksi/piksi.h"
#include "../TRICAL/TRICAL.h"
#include "measurement.h"
#include "ahrs.h"

#define AHRS_DELTA 0.001

/*
Track the "actual" control position -- NMPC outputs desired position but
we limit the rate at which that can change to the relevant control_rate
value.

The control_scale value is the magnitude of the maximum control input.
*/
static double control_rate[4];
static double control_pos[4];
static double control_scale[4];

/* Log packet tick count -- used by the CPU to detect missing log packets */
static uint16_t log_tick;

/* Global FCS state structure */
struct fcs_packet_state_t fcs_global_state;

/* Macro to limit the absolute value of x to l, but preserve the sign */
#define limitabs(x, l) (x < 0.0 && x < -l ? -l : x > 0.0 && x > l ? l : x)

/* Internal functions */
void _fcs_ahrs_update_global_state(struct ukf_state_t *restrict s,
double *restrict covariance);

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24);
    assert(ukf_config_get_control_dim() == 4);
    assert(ukf_config_get_measurement_dim() == 20);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    /* Set up global state */
    memset(&fcs_global_state, 0, sizeof(fcs_global_state));
    fcs_global_state.mode_indicator = 'N';

    /*
    TODO: read state back from fcs_global_state, just in case we're booting up
    after an in-flight reset
    */
    struct ukf_state_t initial_state = {
        {0, 0, 0},
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
        TRICAL_norm_set(&fcs_global_ahrs_state.trical_instances[i],
                        vector3_norm_f(fcs_global_ahrs_state.wmm_field));
        TRICAL_noise_set(&fcs_global_ahrs_state.trical_instances[i],
                         (float)sqrt(1.5));
    }
}

void fcs_ahrs_tick(void) {
    /* Handle magnetometer calibration update based on new readings */
    struct fcs_measurement_t mag_measurement;
    double mag_value[4];
    uint8_t i, j;
    for (i = 0; i < 2u; i++) {
        if (fcs_measurement_log_find(
                measurements, FCS_MEASUREMENT_TYPE_MAGNETOMETER, i,
                &mag_measurement)) {
            fcs_measurement_get_values(&measurement, mag_value);
            TRICAL_estimate_update(&magnetometer_calibration[i], mag_value);

            for (j = 0; j < 9u; j++) {
                if (isnan(magnetometer_calibration[i].state)) {
                    /*
                    TRICAL has blown up -- reset this instance and ignore the
                    current reading.
                    */
                    TRICAL_reset(&magnetometer_calibration[i]);
                    fcs_global_counters.trical_resets[i]++;
                    break;
                }
            }
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

    /* FIXME: get these from WMM or something */
    params.mag_field[0] = 0.6;
    params.mag_field[1] = 0.0;
    params.mag_field[2] = 0.0;

    /* Read sensor data from the measurement log, and pass it to the UKF */
    double v[4], err, offset[3];

    ukf_sensor_clear();

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_ACCELEROMETER, v,
            &err, offset)) {
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);
        params.accel_covariance[0] = params.accel_covariance[1] =
            params.accel_covariance[2] = err * err;
        params.accel_offset[0] = offset[0];
        params.accel_offset[1] = offset[1];
        params.accel_offset[2] = offset[2];
    }

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_GYROSCOPE, v,
            &error, offset)) {
        ukf_sensor_set_gyroscope(v[0], v[1], v[2]);
        params.gyro_covariance[0] = params.gyro_covariance[1] =
            params.gyro_covariance[2] = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_MAGNETOMETER, v,
            &error, offset)) {
        ukf_sensor_set_magnetometer(v[0], v[1], v[2]);
        params.mag_covariance[0] = params.mag_covariance[1] =
            params.mag_covariance[2] = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_PITOT, v,
            &error, offset)) {
        ukf_sensor_set_pitot_tas(v[0]);
        params.pitot_covariance = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_PRESSURE_TEMP, v,
            &error, offset)) {
        ukf_sensor_set_barometer_amsl(v[0]);
        params.barometer_amsl_covariance = err * err;
    }

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_GPS_POSITION, v,
            &error, offset)) {
        ukf_sensor_set_gps_position(v[0], v[1], v[2]);
        params.gps_position_covariance[0] =
            params.gps_position_covariance[1] = err * err;
        params.gps_position_covariance[2] = 1600.0;
    }

    if (fcs_measurement_log_get_calibrated_value(
            measurements, calibration, FCS_MEASUREMENT_TYPE_GPS_VELOCITY, v,
            &error, offset)) {
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
    uint8_t i;
    struct fcs_measurement_t control_log;
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4; i++) {
        double delta = control_set[i] - fcs_global_ahrs_state.control_pos[i],
               limit = control_rate[i] * AHRS_DELTA;
        fcs_global_ahrs_state.control_pos[i] += limitabs(delta, limit);
        control_log.data.i16[i] =
            (int16_t)(control_pos[i] / control_scale[i] * INT16_MAX);
    }

    control_log.header = (8u << FCS_MEASUREMENT_HEADER_LENGTH_OFFSET)
                         & FCS_MEASUREMENT_HEADER_LENGTH_MASK;
    control_log.sensor = FCS_MEASUREMENT_TYPE_CONTROL_POS;
    fcs_measurement_log_add(measurements, &control_log);

    /*
    Run the UKF, taking sensor readings and current control position into
    account
    */
    ukf_set_params(&params);
    ukf_iterate(AHRS_DELTA, fcs_global_ahrs_state.control_pos);


#ifdef __TI_COMPILER_VERSION__
    /*
    Use a semaphore to prevent the NMPC code accessing the state while we're
    updating it.
    */
    volatile CSL_SemRegs *const semaphore = (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE];
    assert(sem_val == 1u);
#endif

    /* Write current state to global state store */
    ukf_get_state((struct ukf_state_t*)&fcs_global_ahrs_state.lat);

    /* Get the state covariance matrix */
    ukf_get_state_covariance_diagonal(&fcs_global_ahrs_state.lat_covariance);

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE] = 1u;
#endif
}
