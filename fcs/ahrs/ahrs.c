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
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../comms/comms.h"
#include "../ukf/cukf.h"
#include "ahrs.h"

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

#define ACCEL_SENSITIVITY 4096.0 /* LSB/g @ ±8g FS */
#define GYRO_SENSITIVITY 65.5 /* LSB/(deg/s) @ 500deg/s FS */
#define MAG_SENSITIVITY 1090.0 /* LSB/G @ ±2G FS */
#define G 9.80665

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
static struct fcs_ahrs_sensor_geometry_t ioboard_geometry[2];
static struct fcs_ahrs_sensor_calibration_t ioboard_calibration[2];

/*
Track the "actual" control position -- NMPC outputs desired position but
we limit the rate at which that can change to the relevant control_rate
value
*/
static double control_rate[4];
static double control_pos[4];

/* Latest I/O board state packets */
static struct sensor_packet_t ioboard[2];
static uint16_t ioboard_last_tick[2];

/* Global FCS state structure */
struct fcs_packet_state_t global_state;

/* Macro to limit the absolute value of x to l, but preserve the sign */
#define limitabs(x, l) (x < 0.0 && x < -l ? -l : x > 0.0 && x > l ? l : x)

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24);
    assert(ukf_config_get_control_dim() == 4);
    assert(ukf_config_get_measurement_dim() == 20);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    /* Set up global state */
    memset(&global_state, 0, sizeof(global_state));

    /* Set default geometry and calibration */
    memset(ioboard_calibration[0].accel_bias, 0, sizeof(int16_t) * 3);
    memset(ioboard_calibration[1].accel_bias, 0, sizeof(int16_t) * 3);

    ioboard_calibration[0].accel_scale[0] =
        ioboard_calibration[0].accel_scale[1] =
        ioboard_calibration[0].accel_scale[2] = 1.0 / ACCEL_SENSITIVITY;
    ioboard_calibration[1].accel_scale[0] =
        ioboard_calibration[1].accel_scale[1] =
        ioboard_calibration[1].accel_scale[2] = 1.0 / ACCEL_SENSITIVITY;

    ioboard_calibration[0].gyro_scale[0] =
        ioboard_calibration[0].gyro_scale[1] =
        ioboard_calibration[0].gyro_scale[2] = 1.0 / GYRO_SENSITIVITY;
    ioboard_calibration[1].gyro_scale[0] =
        ioboard_calibration[1].gyro_scale[1] =
        ioboard_calibration[1].gyro_scale[2] = 1.0 / GYRO_SENSITIVITY;

    /* Load identity matrices for magnetometer calibration */
    memset(ioboard_calibration[0].mag_bias, 0, sizeof(float) * 3);
    memset(ioboard_calibration[1].mag_bias, 0, sizeof(float) * 3);

    memset(ioboard_calibration[0].mag_scale, 0, sizeof(float) * 9);
    memset(ioboard_calibration[1].mag_scale, 0, sizeof(float) * 9);

    ioboard_calibration[0].mag_scale[0] =
        ioboard_calibration[0].mag_scale[4] =
        ioboard_calibration[0].mag_scale[8] = 1.0 / MAG_SENSITIVITY;
    ioboard_calibration[1].mag_scale[0] =
        ioboard_calibration[1].mag_scale[4] =
        ioboard_calibration[1].mag_scale[8] = 1.0 / MAG_SENSITIVITY;

    ioboard_calibration[0].pitot_bias =
        ioboard_calibration[1].pitot_bias = 0;

    ioboard_calibration[0].pitot_scale =
        ioboard_calibration[1].pitot_scale = 1.0 / 65535.0;

    ioboard_calibration[0].barometer_bias =
        ioboard_calibration[1].barometer_bias = 0;

    ioboard_calibration[0].barometer_scale =
        ioboard_calibration[1].barometer_scale = 0.02;

    ioboard_geometry[0].accel_orientation[0] = 0.0;
    ioboard_geometry[0].accel_orientation[1] = 0.0;
    ioboard_geometry[0].accel_orientation[2] = 0.0;
    ioboard_geometry[0].accel_orientation[3] = 1.0;
    ioboard_geometry[1].accel_orientation[0] = 0.0;
    ioboard_geometry[1].accel_orientation[1] = 0.0;
    ioboard_geometry[1].accel_orientation[2] = 0.0;
    ioboard_geometry[1].accel_orientation[3] = 1.0;

    ioboard_geometry[0].gyro_orientation[0] = 0.0;
    ioboard_geometry[0].gyro_orientation[1] = 0.0;
    ioboard_geometry[0].gyro_orientation[2] = 0.0;
    ioboard_geometry[0].gyro_orientation[3] = 1.0;
    ioboard_geometry[1].gyro_orientation[0] = 0.0;
    ioboard_geometry[1].gyro_orientation[1] = 0.0;
    ioboard_geometry[1].gyro_orientation[2] = 0.0;
    ioboard_geometry[1].gyro_orientation[3] = 1.0;

    ioboard_geometry[0].accel_position[0] = 0.0;
    ioboard_geometry[0].accel_position[1] = 0.0;
    ioboard_geometry[0].accel_position[2] = 0.0;
    ioboard_geometry[1].accel_position[0] = 0.0;
    ioboard_geometry[1].accel_position[1] = 0.0;
    ioboard_geometry[1].accel_position[2] = 0.0;

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

    /* Set up UKF */
    ukf_init();
    ukf_set_state(&initial_state);

    /* Configure UKF covariance and sensor offsets */
    struct fcs_ahrs_sensor_covariance_t covariance;
    struct fcs_ahrs_wmm_field_t field;

    struct ukf_ioboard_params_t params = {
        /*
        Orientations should represent the enclosure-to-fuselage rotation of
        the CPU board. The accel_offset value should be the position of the
        enclosure reference point relative to the CoM plus the offset from the
        enclosure reference point to the mean of the two I/O board
        accelerometer positions.
        */
        {0, 0, 0, 1}, /* accel_orientation */
        {0, 0, 0}, /* accel_offset */
        {0, 0, 0, 1}, /* gyro_orientation */
        {0, 0, 0, 1} /* mag_orientation */
    };
    params.mag_field[0] = field.mag_field[0];
    params.mag_field[1] = field.mag_field[1];
    params.mag_field[2] = field.mag_field[2];

    params.accel_covariance[0] = covariance.accel_covariance;
    params.accel_covariance[1] = covariance.accel_covariance;
    params.accel_covariance[2] = covariance.accel_covariance;

    params.gyro_covariance[0] = covariance.gyro_covariance;
    params.gyro_covariance[1] = covariance.gyro_covariance;
    params.gyro_covariance[2] = covariance.gyro_covariance;

    params.mag_covariance[0] = covariance.mag_covariance;
    params.mag_covariance[1] = covariance.mag_covariance;
    params.mag_covariance[2] = covariance.mag_covariance;

    params.gps_position_covariance[0] = covariance.gps_position_covariance_h;
    params.gps_position_covariance[1] = covariance.gps_position_covariance_h;
    params.gps_position_covariance[2] = covariance.gps_position_covariance_v;

    params.gps_velocity_covariance[0] = covariance.gps_velocity_covariance_h;
    params.gps_velocity_covariance[1] = covariance.gps_velocity_covariance_h;
    params.gps_velocity_covariance[2] = covariance.gps_velocity_covariance_v;

    params.pitot_covariance = covariance.pitot_covariance;
    params.barometer_amsl_covariance = covariance.barometer_amsl_covariance;

    ukf_set_params(&params);
}

void fcs_ahrs_tick(void) {
    /* Read latest I/O board packets from the UART streams */
    uint8_t buf[64];
    uint32_t nbytes;
    struct fcs_cobsr_decode_result decode_result;
    uint8_t i;
    bool ioboard_packet_read[2] = { false, false };

    for (i = 0; i < 2; i++) {
        /*
        Skip until after the next NUL byte -- if there was a successful read
        last tick, this will bring us to the first byte of the message
        */
        nbytes = fcs_stream_skip_until_after(
            FCS_STREAM_UART_INT0 + i, (uint8_t)0);

        /*
        If no bytes were skipped, there must have been no NUL byte in the
        buffer, and therefore no complete message -- don't bother trying to
        read anything yet.
        */
        if (!nbytes) {
            continue;
        }

        if (fcs_stream_peek(FCS_STREAM_UART_INT0 + i) == 0) {
            /*
            Must have skipped over some (incomplete) message bytes, so we're
            now at the terminating NUL byte of the message we skipped -- skip
            again to get past the starting NUL byte of the next message
            */
            fcs_stream_skip_until_after(FCS_STREAM_UART_INT0 + i, (uint8_t)0);
        }

        /* Read until after the next (terminating) NUL */
        nbytes = fcs_stream_read_until_after(
            FCS_STREAM_UART_INT0 + i, (uint8_t)0, buf, 64u);
        if (nbytes >= sizeof(struct sensor_packet_t) + 1) {
            /* Decode the message into the packet buffer */
            decode_result = fcs_cobsr_decode(
                (uint8_t*)&ioboard[i],
                sizeof(struct sensor_packet_t),
                buf,
                nbytes - 1
            );

            if (decode_result.status == FCS_COBSR_DECODE_OK &&
                    decode_result.out_len == sizeof(struct sensor_packet_t)) {
                ioboard_packet_read[i] = true;
            }
        }
    }

    /* Read sensor data from latest I/O board packets, and convert */
    float accel_data[3] = {0.0, 0.0, 0.0},
          gyro_data[3] = {0.0, 0.0, 0.0},
          mag_data[3] = {0.0, 0.0, 0.0},
          gps_position_data[3] = {0.0, 0.0, 0.0},
          gps_velocity_data[3] = {0.0, 0.0, 0.0},
          pitot_data = 0.0,
          barometer_data = 0.0;
    uint8_t n_accel = 0, n_gyro = 0, n_mag = 0,
            n_gps = 0, n_pitot = 0,
            n_barometer = 0;
    float v[3], rv[3];

    for (i = 0; i < 2u; i++) {
        if (!ioboard_packet_read[i] ||
                ioboard[i].tick == ioboard_last_tick[i]) {
            continue;
        }

        if (ioboard[i].sensor_update_flags & UPDATED_ACCEL) {
            v[0] = ((float)ioboard[i].accel.x -
                    (float)ioboard_calibration[i].accel_bias[0]) *
                   ioboard_calibration[i].accel_scale[0];

            v[1] = ((float)ioboard[i].accel.y -
                    (float)ioboard_calibration[i].accel_bias[1]) *
                   ioboard_calibration[i].accel_scale[1];

            v[2] = ((float)ioboard[i].accel.z -
                    (float)ioboard_calibration[i].accel_bias[2]) *
                   ioboard_calibration[i].accel_scale[2];

            /* Transform v by accelerometer orientation */
            quaternion_vector3_multiply_f(
                rv, ioboard_geometry[i].accel_orientation, v);

            accel_data[0] += rv[0];
            accel_data[1] += rv[1];
            accel_data[2] += rv[2];

            n_accel++;
        }

        if (ioboard[i].sensor_update_flags & UPDATED_GYRO) {
            v[0] = (float)ioboard[i].gyro.x *
                   ioboard_calibration[i].gyro_scale[0];

            v[1] = (float)ioboard[i].gyro.y *
                   ioboard_calibration[i].gyro_scale[1];

            v[2] = (float)ioboard[i].gyro.z *
                   ioboard_calibration[i].gyro_scale[2];

            /* Transform v by gyro orientation */
            quaternion_vector3_multiply_f(
                rv, ioboard_geometry[i].gyro_orientation, v);

            gyro_data[0] += rv[0];
            gyro_data[1] += rv[1];
            gyro_data[2] += rv[2];

            n_gyro++;
        }

        if (ioboard[i].sensor_update_flags & UPDATED_MAG) {
            v[0] = (float)ioboard[i].mag.x -
                   ioboard_calibration[i].mag_bias[0];
            v[1] = (float)ioboard[i].mag.y -
                   ioboard_calibration[i].mag_bias[1];
            v[2] = (float)ioboard[i].mag.z -
                   ioboard_calibration[i].mag_bias[2];

            /* Multiply v by magnetometer calibration matrix */
            /* FIXME: confirm order of parameters etc */
            matrix_multiply_f(rv, ioboard_calibration[i].mag_scale,
                              v, 1, 3, 3, 3, 1.0);

            mag_data[0] += rv[0];
            mag_data[1] += rv[1];
            mag_data[2] += rv[2];

            n_mag++;
        }

        if (ioboard[i].sensor_update_flags & UPDATED_GPS_POS) {
            gps_position_data[0] += (float)ioboard[i].gps.position.lat * 1e-7;
            gps_position_data[1] += (float)ioboard[i].gps.position.lng * 1e-7;
            gps_position_data[2] += (float)ioboard[i].gps.position.alt * 1e-2;

            gps_velocity_data[0] += (float)ioboard[i].gps.velocity.n * 1e-2;
            gps_velocity_data[1] += (float)ioboard[i].gps.velocity.e * 1e-2;
            gps_velocity_data[2] += (float)ioboard[i].gps.velocity.d * 1e-2;

            n_gps++;
        }

        if (ioboard[i].sensor_update_flags & UPDATED_BAROMETER) {
            barometer_data += ((float)ioboard[i].pressure +
                               (float)ioboard_calibration[i].barometer_bias) *
                              ioboard_calibration[i].barometer_scale;

            n_barometer++;
        }

        if (ioboard[i].sensor_update_flags & UPDATED_ADC_GPIO) {
            pitot_data += ((float)ioboard[i].pitot +
                           (float)ioboard_calibration[i].pitot_bias) *
                          ioboard_calibration[i].pitot_scale;

            n_pitot++;
        }

        ioboard_last_tick[i] = ioboard[i].tick;
    }

    /* Load converted/averaged sensor data */
    ukf_sensor_clear();

    float scale[3] = {0.0, 0.5, 1.0};
    if (n_accel) {
        ukf_sensor_set_accelerometer(
            accel_data[0] * scale[n_accel],
            accel_data[1] * scale[n_accel],
            accel_data[2] * scale[n_accel]
        );
    }
    if (n_gyro) {
        ukf_sensor_set_gyroscope(
            gyro_data[0] * scale[n_gyro],
            gyro_data[1] * scale[n_gyro],
            gyro_data[2] * scale[n_gyro]
        );
    }
    if (n_mag) {
        ukf_sensor_set_magnetometer(
            mag_data[0] * scale[n_mag],
            mag_data[1] * scale[n_mag],
            mag_data[2] * scale[n_mag]
        );
    }
    if (n_gps) {
        ukf_sensor_set_gps_position(
            gps_position_data[0] * scale[n_gps],
            gps_position_data[1] * scale[n_gps],
            gps_position_data[2] * scale[n_gps]
        );
        ukf_sensor_set_gps_velocity(
            gps_velocity_data[0] * scale[n_gps],
            gps_velocity_data[1] * scale[n_gps],
            gps_velocity_data[2] * scale[n_gps]
        );
    }
    if (n_pitot) {
        ukf_sensor_set_pitot_tas(pitot_data * scale[n_pitot]);
    }
    if (n_barometer) {
        ukf_sensor_set_barometer_amsl(barometer_data * scale[n_barometer]);
    }

    /* TODO: Read control set values from NMPC */
    double control_set[4] = { 0.0, 0.0, 0.0, 0.0 };

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

    /* TODO: Write current control values to I/O boards */

    /* Run the UKF */
    ukf_iterate(AHRS_DELTA, control_pos);

    /* Write current state to global state store */
    struct ukf_state_t s;
    ukf_get_state(&s);

    if (global_state.solution_time < INT32_MAX) {
        global_state.solution_time++;
    } else {
        global_state.solution_time = 0;
    }

    /* Convert lat/lon to degrees */
    global_state.lat = s.position[0] * (180.0 / M_PI);
    global_state.lon = s.position[1] * (180.0 / M_PI);
    global_state.alt = s.position[2];

    memcpy(global_state.velocity, s.velocity, 3 * sizeof(double));
    memcpy(global_state.wind_velocity, s.wind_velocity,
           3 * sizeof(double));

    /* Convert angular velocity to degrees/s */
    global_state.angular_velocity[0] = s.angular_velocity[0] * (180.0 / M_PI);
    global_state.angular_velocity[1] = s.angular_velocity[1] * (180.0 / M_PI);
    global_state.angular_velocity[2] = s.angular_velocity[2] * (180.0 / M_PI);

    /* Convert state.attitude to yaw/pitch/roll */
    double yaw, pitch, roll;

    #define q s.attitude
    yaw = atan2(2.0f * (q[W] * q[Z] + q[X] * q[Y]),
                1.0f - 2.0f * (q[Y] * q[Y] + q[Z] * q[Z]));
    pitch = asin(2.0f * (q[W] * q[X] - q[Z] * q[X]));
    roll = atan2(2.0f * (q[W] * q[X] + q[Y] * q[Z]),
                 1.0f - 2.0f * (q[X] * q[X] + q[Y] * q[Y]));
    #undef q

    if (yaw < 0.0) {
        yaw += 360.0;
    }
    assert(0.0 <= yaw && yaw <= 360.0);
    assert(-90.0 <= pitch && pitch <= 90.0);
    assert(-180.0 <= roll && roll <= 180.0);

    global_state.attitude[0] = yaw;
    global_state.attitude[1] = pitch;
    global_state.attitude[2] = roll;

    /*
    Get the state covariance matrix, and work out the 95th percentile
    confidence interval assuming Gaussian distribution
    */
    double covariance[UKF_STATE_DIM];
    ukf_get_state_covariance_diagonal(covariance);

    /*
    Ignore changes in lon with varying lat -- this should be close enough
    */
    global_state.lat_lon_uncertainty = 1.96 *
        sqrt(fmax(covariance[0], covariance[1])) * (40008000.0 / 360.0);
    global_state.alt_uncertainty = 1.96 * sqrt(covariance[2]);

    global_state.velocity_uncertainty[0] = 1.96 * sqrt(covariance[3]);
    global_state.velocity_uncertainty[1] = 1.96 * sqrt(covariance[4]);
    global_state.velocity_uncertainty[2] = 1.96 * sqrt(covariance[5]);

    global_state.wind_velocity_uncertainty[0] = 1.96 * sqrt(covariance[18]);
    global_state.wind_velocity_uncertainty[1] = 1.96 * sqrt(covariance[19]);
    global_state.wind_velocity_uncertainty[2] = 1.96 * sqrt(covariance[20]);

    global_state.attitude_uncertainty[0] = 1.96 *
        sqrt(covariance[9]) * (180.0 / M_PI);
    global_state.attitude_uncertainty[1] = 1.96 *
        sqrt(covariance[10]) * (180.0 / M_PI);
    global_state.attitude_uncertainty[2] = 1.96 *
        sqrt(covariance[11]) * (180.0 / M_PI);

    global_state.angular_velocity_uncertainty[0] = 1.96 *
        sqrt(covariance[12]) * (180.0 / M_PI);
    global_state.angular_velocity_uncertainty[1] = 1.96 *
        sqrt(covariance[13]) * (180.0 / M_PI);
    global_state.angular_velocity_uncertainty[2] = 1.96 *
        sqrt(covariance[14]) * (180.0 / M_PI);

    /* TODO: Update state mode indicator based on confidence in filter lock */
    global_state.mode_indicator = 'A';
}
