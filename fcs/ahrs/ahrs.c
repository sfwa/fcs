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
#include <stdlib.h>
#include <assert.h>

#include "../config/config.h"
#include "ahrs.h"
#include "../ukf/cukf.h"

#define AHRS_DELTA 0.001

struct sensor_packet_t {
    /* Base fields */
    uint8_t crc;
    uint16_t tick;
    uint8_t sensor_update_flags;
    uint8_t cpu_load;
    uint16_t status;

    /* Sensor fields */
    struct {
        int16_t x, y, z; /* [-4,4]g, 8192LSb/g */
    } __attribute__ ((packed)) accel;
    struct {
        int16_t x, y, z; /* [-500,500]deg/s, 65.5LSb/(deg/s) */
    } __attribute__ ((packed)) gyro;
    int16_t accel_gyro_temp; /* -40 to 85degC, 340LSb/degC,
        -512 = 35degC */
    uint16_t pressure; /* 10-1200mbar, 1LSb = 0.02mbar */
    uint16_t barometer_temp; /* -40 to 125degC, in 0.01degC increments,
        0 = -40degC */
    int16_t pitot; /* 16-bit ADC reading -- pitot sensor */
    int16_t i; /* 16-bit ADC reading -- current sensor */
    int16_t v; /* 16-bit ADC reading -- voltage sensor */
    int16_t range; /* 16-bit ADC reading -- rangefinder */
    uint8_t gpin_state; /* & 0x0f for pin state, & 0xf0 for PWM read */

    /* Magnetometer */
    struct {
        int16_t x, y, z; /* [-2,2]Ga, 1090LSb/Ga */
    } __attribute__ ((packed)) mag;

    /* GPS fields */
    struct {
        struct {
            int32_t lat, lng; /* lat, lng in 10^-7 degress */
            int32_t alt; /* alt above msl in  cm */
        } position;
        struct {
            int16_t n, e, d; /* NED in cm/s */
        } __attribute__ ((packed)) velocity;
    } __attribute__ ((packed)) gps;

    struct {
        uint8_t fix_mode_num_satellites; /* 2x 4-bit values */
        uint8_t pos_err; /* error estimate in metres */
    } __attribute__ ((packed)) gps_info;
} __attribute__ ((packed));

#define SENSOR_PACKET_LEN 60u

#define SENSOR_STATUS_TXERR_MASK   0x00000001u
#define SENSOR_STATUS_RXERR_MASK   0x00000002u
#define SENSOR_STATUS_GPS_MASK     0x0000001cu
#define SENSOR_STATUS_GPS_OFFSET   2u
#define SENSOR_STATUS_BAROMETER_MASK  0x000000e0u
#define SENSOR_STATUS_BAROMETER_OFFSET 5u
#define SENSOR_STATUS_ACCEL_GYRO_MASK 0x00000700u
#define SENSOR_STATUS_ACCEL_GYRO_OFFSET 8u
#define SENSOR_STATUS_MAGNETOMETER_MASK 0x00003800u
#define SENSOR_STATUS_MAGNETOMETER_OFFSET 11u
#define SENSOR_STATUS_UNUSED_MASK  0xC000u

#define UPDATED_ACCEL 0x01u
#define UPDATED_GYRO 0x02u
#define UPDATED_BAROMETER 0x04u
#define UPDATED_MAG 0x08u
#define UPDATED_GPS_POS 0x10u
#define UPDATED_GPS_INFO 0x20u
#define UPDATED_ADC_GPIO 0x40u

#define CMD_KEY_LEN 8u
#define TICK_MAX 65535u
/* From http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
   0x97 in Koopman notation = 0x12F in MSB-first notation, so excluing implicit
   x^n term we get 2F. */
#define COMM_CRC8_POLY 0x2Fu

struct control_packet_t {
    uint8_t crc;
    uint8_t tick;
    uint8_t msg_type;
    uint8_t gpout;
    uint16_t pwm[4];
} __attribute__ ((packed));

struct cmd_packet_t {
    uint8_t crc;
    uint8_t tick;
    uint8_t msg_type;
    uint8_t cmd[CMD_KEY_LEN];
    uint8_t val;
} __attribute__ ((packed));

struct firmware_packet_t {
    uint8_t crc;
    uint8_t tick;
    uint8_t msg_type;
    uint16_t addr;
    uint32_t data;
};

enum msg_type_t {
    MSG_TYPE_NONE = 0,
    MSG_TYPE_CONTROL = 1,
    MSG_TYPE_FIRMWARE = 2,
    MSG_TYPE_CMD = 3
};

/*
We need to hold on to these parts of the configuration -- the others are
provided directly to the UKF
*/
static struct fcs_ahrs_sensor_geometry_v1_t sensor_geometry;
static struct fcs_ahrs_sensor_calibration_v1_t sensor_calibration;

/*
Track the "actual" control position -- NMPC outputs desired position but
we limit the rate at which that can change to the relevant control_rate
value
*/
static double control_rate[4];
static double control_pos[4];

/* Macro to limit the absolute value of x to l, but preserve the sign */
#define limitabs(x, l) (x < 0.0 && x < -l ? -l : x > 0.0 && x > l ? l : x)

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24);
    assert(ukf_config_get_control_dim() == 4);
    assert(ukf_config_get_measurement_dim() == 20);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    /*
    TODO: read state from NMPC shared memory, just in case we're booting up
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

    ukf_init();
    ukf_set_state(&initial_state);
    fcs_ahrs_load_config(current_config);
}

void fcs_ahrs_tick(void) {
    ukf_sensor_clear();

    /* TODO: read sensor data from latest I/O board packets, and convert */

    /* TODO: load converted sensor data */
    /*
    ukf_sensor_set_accelerometer(x, y, z);
    ukf_sensor_set_gyroscope(x, y, z);
    ukf_sensor_set_magnetometer(x, y, z);
    ukf_sensor_set_gps_position(lat, lon, alt);
    ukf_sensor_set_gps_velocity(n, e, d);
    ukf_sensor_set_pitot_tas(tas);
    ukf_sensor_set_barometer_amsl(alt_amsl);
    */

    /* TODO: read control set values from NMPC */
    double control_set[4] = { 0.0, 0.0, 0.0, 0.0 };
    uint32_t i;

    /*
    Work out the nominal current control position, taking into account the
    response time (configured in control_rates)
    */
    #pragma MUST_ITERATE(4)
    for (i = 0; i < 4; i++) {
        double delta = control_set[i] - control_pos[i],
               limit = control_rate[i] * AHRS_DELTA;
        control_pos[i] += limitabs(delta, limit);
    }

    /* Run the UKF */
    ukf_iterate(AHRS_DELTA, control_pos);

    /* TODO: write current state to NMPC */
    struct ukf_state_t state;
    ukf_get_state(&state);
}

void fcs_ahrs_update_state(const struct fcs_state_t *new_state) {
    assert(new_config);

    enum fcs_config_param_result_t result;

    /* Load the AHRS settings */
    result = fcs_config_get_param_block(
        new_config, sizeof(sensor_geometry), &sensor_geometry,
        FCS_AHRS_SENSOR_GEOMETRY_KEY, FCS_AHRS_SENSOR_GEOMETRY_VERSION);
    assert(result == FCS_CONFIG_PARAM_OK);

    result = fcs_config_get_param_block(
        new_config, sizeof(sensor_calibration), &sensor_calibration,
        FCS_AHRS_SENSOR_CALIBRATION_KEY, FCS_AHRS_SENSOR_CALIBRATION_VERSION);
    assert(result == FCS_CONFIG_PARAM_OK);

    /* Set up the I/O board params */
    {
        struct fcs_ahrs_sensor_covariance_v1_t covariance;
        result = fcs_config_get_param_block(
            new_config, sizeof(covariance), &covariance,
            FCS_AHRS_SENSOR_COVARIANCE_KEY,
            FCS_AHRS_SENSOR_COVARIANCE_VERSION);
        assert(result == FCS_CONFIG_PARAM_OK);

        struct fcs_ahrs_wmm_field_v1_t field;
        result = fcs_config_get_param_block(
            new_config, sizeof(field), &field,
            FCS_AHRS_WMM_FIELD_KEY,
            FCS_AHRS_WMM_FIELD_VERSION);
        assert(result == FCS_CONFIG_PARAM_OK);

        struct ukf_ioboard_params_t ioboard_params = {
            /*
            Orientations and offsets are 0 because we're using two I/O boards
            and fusing their measurements before handing them off to the UKF
            */
            {0, 0, 0, 1}, /* accel_orientation */
            {0, 0, 0}, /* accel_offset */
            {0, 0, 0, 1}, /* gyro_orientation */
            {0, 0, 0, 1} /* mag_orientation */
        };
        ioboard_params.mag_field[0] = field.mag_field[0];
        ioboard_params.mag_field[1] = field.mag_field[1];
        ioboard_params.mag_field[2] = field.mag_field[2];

        ioboard_params.accel_covariance[0] = covariance.accel_covariance[0];
        ioboard_params.accel_covariance[1] = covariance.accel_covariance[1];
        ioboard_params.accel_covariance[2] = covariance.accel_covariance[2];

        ioboard_params.gyro_covariance[0] = covariance.gyro_covariance[0];
        ioboard_params.gyro_covariance[1] = covariance.gyro_covariance[1];
        ioboard_params.gyro_covariance[2] = covariance.gyro_covariance[2];

        ioboard_params.mag_covariance[0] = covariance.mag_covariance[0];
        ioboard_params.mag_covariance[1] = covariance.mag_covariance[1];
        ioboard_params.mag_covariance[2] = covariance.mag_covariance[2];

        ioboard_params.gps_position_covariance[0] = covariance.gps_position_covariance[0];
        ioboard_params.gps_position_covariance[1] = covariance.gps_position_covariance[1];
        ioboard_params.gps_position_covariance[2] = covariance.gps_position_covariance[2];

        ioboard_params.gps_velocity_covariance[0] = covariance.gps_velocity_covariance[0];
        ioboard_params.gps_velocity_covariance[1] = covariance.gps_velocity_covariance[1];
        ioboard_params.gps_velocity_covariance[2] = covariance.gps_velocity_covariance[2];

        ioboard_params.pitot_covariance = covariance.pitot_covariance;
        ioboard_params.barometer_amsl_covariance = covariance.barometer_amsl_covariance;

        ukf_set_params(&ioboard_params);
    }

    /* Set up the dynamics model */
    {
        struct fcs_ahrs_dynamics_model_v1_t model;
        result = fcs_config_get_param_block(
            new_config, sizeof(model), &model,
            FCS_AHRS_DYNAMICS_MODEL_KEY,
            FCS_AHRS_DYNAMICS_MODEL_VERSION);
        assert(result == FCS_CONFIG_PARAM_OK);

        ukf_set_process_noise(model.process_noise);
        switch (model.model) {
            case 0u:
                ukf_choose_dynamics(UKF_MODEL_NONE);
                break;
            case 1u:
                ukf_choose_dynamics(UKF_MODEL_CENTRIPETAL);
                break;
            case 2u:
                ukf_choose_dynamics(UKF_MODEL_FIXED_WING);
                break;
            default:
                assert(0);
                break;
        }

        /* Set the dynamics model parameters, if present */
        struct fcs_model_dynamics_params_v1_t opts;
        result = fcs_config_get_param_block(
            new_config, sizeof(opts), &opts,
            FCS_MODEL_DYNAMICS_PARAMS_KEY,
            FCS_MODEL_DYNAMICS_PARAMS_VERSION);
        if (result == FCS_CONFIG_PARAM_OK) {
            ukf_fixedwingdynamics_set_mass(opts.mass);
            ukf_fixedwingdynamics_set_inertia_tensor(opts.inertia_tensor);
            ukf_fixedwingdynamics_set_prop_coeffs(opts.prop_area,
                                                  opts.prop_cve);
            ukf_fixedwingdynamics_set_lift_coeffs(opts.lift_coeffs);
            ukf_fixedwingdynamics_set_drag_coeffs(opts.drag_coeffs);
            ukf_fixedwingdynamics_set_side_coeffs(
                opts.side_coeffs, opts.side_control_coeffs);
            ukf_fixedwingdynamics_set_yaw_moment_coeffs(
                opts.yaw_moment_coeffs,
                opts.yaw_moment_control_coeffs);
            ukf_fixedwingdynamics_set_pitch_moment_coeffs(
                opts.pitch_moment_coeffs,
                opts.pitch_moment_control_coeffs);
            ukf_fixedwingdynamics_set_roll_moment_coeffs(
                opts.roll_moment_coeff,
                opts.roll_momentcontrol_coeffs);

            uint32_t i;
            #pragma MUST_ITERATE(4)
            for (i = 0; i < 4; i++) {
                control_rate[i] = opts.control_rate[i];
            }
        }
    }
}
