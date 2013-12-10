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
static uint8_t ioboard_last_tick[2];

/* Global FCS state structure */
struct fcs_packet_state_t fcs_global_state;

/* Macro to limit the absolute value of x to l, but preserve the sign */
#define limitabs(x, l) (x < 0.0 && x < -l ? -l : x > 0.0 && x > l ? l : x)

/* Endian swap functions -- AVR32s are big-endian */
static inline uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8 );
}

static inline int16_t swap_int16(int16_t val) {
    return (val << 8) | ((val >> 8) & 0xFF);
}

static inline uint32_t swap_uint32(uint32_t val) {
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
}

static inline int32_t swap_int32(int32_t val) {
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

/* Internal functions */
void _fcs_ahrs_update_global_state(struct ukf_state_t *restrict s,
double *restrict covariance);
bool _fcs_ahrs_read_ioboard_packet(enum fcs_stream_device_t dev,
struct sensor_packet_t *dest);
bool _fcs_ahrs_process_accelerometers(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_gyroscopes(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_magnetometers(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_gps(double *restrict p_output,
float *restrict v_output, const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_pitots(float *restrict output,
const struct sensor_packet_t *restrict packets);
bool _fcs_ahrs_process_barometers(float *restrict output,
const struct sensor_packet_t *restrict packets);
void _fcs_ahrs_ioboard_reset_geometry(
struct fcs_ahrs_sensor_geometry_t *restrict geometry);
void _fcs_ahrs_ioboard_reset_calibration(
struct fcs_ahrs_sensor_calibration_t *restrict calibration);
uint32_t _fcs_ahrs_format_control_packet(uint8_t *buf, uint8_t tick,
const double *restrict control_values);

void fcs_ahrs_init(void) {
    /* Ensure UKF library is configured correctly */
    assert(ukf_config_get_state_dim() == 24);
    assert(ukf_config_get_control_dim() == 4);
    assert(ukf_config_get_measurement_dim() == 20);
    assert(ukf_config_get_precision() == UKF_PRECISION_DOUBLE);

    /* Open I/O board streams */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_INT0, 921600) == FCS_STREAM_OK);
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_INT1, 921600) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_INT0) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_INT1) == FCS_STREAM_OK);

    /* Set up global state */
    memset(&fcs_global_state, 0, sizeof(fcs_global_state));

    /* Set default geometry and calibration */
    _fcs_ahrs_ioboard_reset_calibration(&ioboard_calibration[0]);
    _fcs_ahrs_ioboard_reset_calibration(&ioboard_calibration[1]);

    _fcs_ahrs_ioboard_reset_geometry(&ioboard_geometry[0]);
    _fcs_ahrs_ioboard_reset_geometry(&ioboard_geometry[1]);

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

    /* TODO: read these from config or something */
    memset(&covariance, 0, sizeof(covariance));
    memset(&field, 0, sizeof(field));

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
    uint8_t t = (uint8_t)(fcs_global_state.solution_time & 0xFFu);

    if (_fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &ioboard[0])) {
        ioboard_last_tick[0] = t;
    } else {
        ioboard[0].sensor_update_flags = 0;
    }

    if (_fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT1, &ioboard[1])) {
        ioboard_last_tick[1] = t;
    } else {
        ioboard[1].sensor_update_flags = 0;
    }

    /*
    TODO: Reset I/O board if time since last tick > 20ms and time since last
    reset > 20ms. Also reset the UART and EDMA3 at that point.
    */
    if (fcs_stream_check_error(FCS_STREAM_UART_INT0) == FCS_STREAM_ERROR ||
            ((t - ioboard_last_tick[0]) & 0xFFu) > 20u) {
        assert(fcs_stream_open(FCS_STREAM_UART_INT0) == FCS_STREAM_OK);

        ioboard_last_tick[0] = t;
        fcs_global_counters.ioboard_reset[0]++;
    }

    if (fcs_stream_check_error(FCS_STREAM_UART_INT1) == FCS_STREAM_ERROR ||
            ((t - ioboard_last_tick[1]) & 0xFFu) > 20u) {
        assert(fcs_stream_open(FCS_STREAM_UART_INT1) == FCS_STREAM_OK);

        ioboard_last_tick[1] = t;
        fcs_global_counters.ioboard_reset[1]++;
    }

    /*
    Read sensor data from latest I/O board packets, and convert to UKF
    input format/units. These procedures combine multiple simultaneous sensor
    readings from the two boards into a single virtual reading, generally by
    applying board calibration to each and then averaging the result.
    */
    float v[3];
    double p[3];

    ukf_sensor_clear();

    if (_fcs_ahrs_process_accelerometers(v, ioboard)) {
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_gyroscopes(v, ioboard)) {
        ukf_sensor_set_gyroscope(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_magnetometers(v, ioboard)) {
        ukf_sensor_set_magnetometer(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_gps(p, v, ioboard)) {
        ukf_sensor_set_gps_position(p[0], p[1], p[2]);
        ukf_sensor_set_gps_velocity(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_pitots(v, ioboard)) {
        ukf_sensor_set_pitot_tas(v[0]);
    }
    if (_fcs_ahrs_process_barometers(v, ioboard)) {
        ukf_sensor_set_barometer_amsl(v[0]);
    }

    /* TODO: Read control set values from NMPC */
    double control_set[4] = { 0.0, 0.0, 0.0, 0.0 };
    uint8_t i;

    /*
    Work out the nominal current control position, taking into account the
    control response time configured in control_rates.
    */
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4; i++) {
        double delta = control_set[i] - control_pos[i],
               limit = control_rate[i] * AHRS_DELTA;
        control_pos[i] += limitabs(delta, limit);
    }

    /*
    Write current control values to I/O boards -- the CPLD replicates the I/O
    board output stream so we only need to write to one.
    */
    uint32_t control_len;
    uint8_t control_buf[16];
    control_len = _fcs_ahrs_format_control_packet(
        control_buf,
        (uint8_t)(fcs_global_state.solution_time & 0xFFu),
        control_set
    );
    assert(control_len < 16);

    fcs_stream_write(FCS_STREAM_UART_INT0, control_buf, control_len);
    fcs_stream_write(FCS_STREAM_UART_INT1, control_buf, control_len);

    fcs_global_counters.ioboard_packet_tx[0]++;
    fcs_global_counters.ioboard_packet_tx[1]++;

    /* Run the UKF */
    ukf_iterate(AHRS_DELTA, control_pos);

    /* Write current state to global state store */
    struct ukf_state_t s;
    ukf_get_state(&s);

    /* Get the state covariance matrix */
    double covariance[UKF_STATE_DIM];
    ukf_get_state_covariance_diagonal(covariance);

    /*
    Process the current state and write it out to the global state vector,
    which is transferred to the CPU at 50Hz and also used by the NMPC code.
    */
    _fcs_ahrs_update_global_state(&s, covariance);
}

void _fcs_ahrs_update_global_state(struct ukf_state_t *restrict s,
double *restrict covariance) {
#ifdef __TI_COMPILER_VERSION__
    /*
    Use a semaphore to prevent the NMPC code accessing the state while we're
    updating it.
    */
    volatile CSL_SemRegs *semaphore = (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE];
    assert(sem_val == 1u);
#endif

    if (fcs_global_state.solution_time < INT32_MAX) {
        fcs_global_state.solution_time++;
    } else {
        fcs_global_state.solution_time = 0;
    }

    /* Convert lat/lon to degrees */
    fcs_global_state.lat = s->position[0] * (180.0/M_PI);
    fcs_global_state.lon = s->position[1] * (180.0/M_PI);
    fcs_global_state.alt = s->position[2];

    memcpy(fcs_global_state.velocity, s->velocity, 3 * sizeof(double));
    memcpy(fcs_global_state.wind_velocity, s->wind_velocity,
           3 * sizeof(double));

    /* Convert angular velocity to degrees/s */
    fcs_global_state.angular_velocity[0] = s->angular_velocity[0] *
                                           (180.0/M_PI);
    fcs_global_state.angular_velocity[1] = s->angular_velocity[1] *
                                           (180.0/M_PI);
    fcs_global_state.angular_velocity[2] = s->angular_velocity[2] *
                                           (180.0/M_PI);

    /* Convert state.attitude to yaw/pitch/roll, in degrees */
    double yaw, pitch, roll;

    #define q s->attitude
    yaw = atan2(2.0f * (q[W] * q[Z] + q[X] * q[Y]),
                1.0f - 2.0f * (q[Y] * q[Y] + q[Z] * q[Z])) * (180.0/M_PI);
    pitch = asin(2.0f * (q[W] * q[X] - q[Z] * q[X])) * (180.0/M_PI);
    roll = atan2(2.0f * (q[W] * q[X] + q[Y] * q[Z]),
                 1.0f - 2.0f * (q[X] * q[X] + q[Y] * q[Y])) * (180.0/M_PI);
    #undef q

    if (yaw < 0.0) {
        yaw += 360.0;
    }
    assert(0.0 <= yaw && yaw <= 360.0);
    assert(-90.0 <= pitch && pitch <= 90.0);
    assert(-180.0 <= roll && roll <= 180.0);

    fcs_global_state.yaw = yaw;
    fcs_global_state.pitch = pitch;
    fcs_global_state.roll = roll;

    /*
    Work out 95th percentile confidence intervals for each of the output
    values, based on the current state covariance matrix and the assumption
    that error will follow a Gaussian distribution
    */

    /* Ignore changes in lon with varying lat -- small angles and all that */
    fcs_global_state.lat_lon_uncertainty = 1.96 *
        sqrt(max(covariance[0], covariance[1])) * (40008000.0 / 360.0);
    fcs_global_state.alt_uncertainty = 1.96 * sqrt(covariance[2]);

    fcs_global_state.velocity_uncertainty[0] = 1.96 * sqrt(covariance[3]);
    fcs_global_state.velocity_uncertainty[1] = 1.96 * sqrt(covariance[4]);
    fcs_global_state.velocity_uncertainty[2] = 1.96 * sqrt(covariance[5]);

    fcs_global_state.wind_velocity_uncertainty[0] = 1.96 *
                                                    sqrt(covariance[18]);
    fcs_global_state.wind_velocity_uncertainty[1] = 1.96 *
                                                    sqrt(covariance[19]);
    fcs_global_state.wind_velocity_uncertainty[2] = 1.96 *
                                                    sqrt(covariance[20]);

    /* Rotation around +X, +Y and +Z -- roll, pitch, yaw */
    fcs_global_state.roll_uncertainty = 1.96 *
        sqrt(covariance[9]) * (180.0/M_PI);
    fcs_global_state.pitch_uncertainty = 1.96 *
        sqrt(covariance[10]) * (180.0/M_PI);
    fcs_global_state.yaw_uncertainty= 1.96 *
        sqrt(covariance[11]) * (180.0/M_PI);

    fcs_global_state.angular_velocity_uncertainty[0] = 1.96 *
        sqrt(covariance[12]) * (180.0/M_PI);
    fcs_global_state.angular_velocity_uncertainty[1] = 1.96 *
        sqrt(covariance[13]) * (180.0/M_PI);
    fcs_global_state.angular_velocity_uncertainty[2] = 1.96 *
        sqrt(covariance[14]) * (180.0/M_PI);

    /* TODO: Update state mode indicator based on confidence in filter lock */
    fcs_global_state.mode_indicator = 'A';

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE] = 1u;
#endif
}

/*
Read, deserialize and validate a full I/O board packet from `dev`. Since we
get two of these every tick there's no point doing this incrementally; we just
need to make sure we can deal with partial/corrupted packets.
*/
bool _fcs_ahrs_read_ioboard_packet(enum fcs_stream_device_t dev,
struct sensor_packet_t *dest) {
    assert(dest);

    /* Read latest I/O board packets from the UART streams */
    uint8_t buf[64], i = 0, checksum;
    uint32_t nbytes;
    struct fcs_cobsr_decode_result result;

    nbytes = fcs_stream_bytes_available(dev);

    /*
    If the initial byte is not 0, we haven't synchronised with the start of a
    message -- try to do that now by skipping until we see a zero followed by
    another zero.

    Give up after 4 tries.
    */
    while (i < 4 && fcs_stream_peek(dev) > 0 &&
           nbytes > sizeof(struct sensor_packet_t) + 1) {
        fcs_stream_skip_until_after(dev, (uint8_t)0);
        nbytes = fcs_stream_bytes_available(dev);
        i++;
    }

    /* Give up if there aren't enough bytes remaining */
    if (nbytes < sizeof(struct sensor_packet_t) + 2 || i == 4) {
        return false;
    }

    /*
    Skip until after the next NUL byte -- if there was a successful read
    last tick, this will bring us to the first byte of the message
    */
    nbytes = fcs_stream_skip_until_after(dev, (uint8_t)0);

    /*
    If no bytes were skipped, there must have been no NUL byte in the
    buffer, and therefore no complete message -- don't bother trying to
    read anything yet.
    */
    if (!nbytes) {
        return false;
    }

    /* Read until after the next (terminating) NUL */
    nbytes = fcs_stream_read_until_after(dev, (uint8_t)0, buf, 64u);
    if (nbytes < sizeof(struct sensor_packet_t)) {
        goto invalid;
    }

    /* Decode the message into the packet buffer */
    result = fcs_cobsr_decode((uint8_t*)dest,
                              sizeof(struct sensor_packet_t),
                              buf, nbytes - 1);

    /* Confirm decode was successful */
    if (result.status != FCS_COBSR_DECODE_OK) {
        goto invalid;
    }

    /* Confirm packet size is what we expect */
    if (result.out_len != sizeof(struct sensor_packet_t)) {
        goto invalid;
    }

    /* Validate the packet checksum */
    checksum = fcs_crc8(
        (uint8_t*)&dest->tick, result.out_len - 1, 0x0);

    if (checksum != dest->crc) {
        goto invalid;
    }

    /* Swap bytes for multi-byte values -- AVR32 is big-endian */
    dest->tick = swap_uint16(dest->tick);
    dest->status = swap_uint16(dest->status);
    dest->accel.x = swap_int16(dest->accel.x);
    dest->accel.y = swap_int16(dest->accel.y);
    dest->accel.z = swap_int16(dest->accel.z);
    dest->gyro.x = swap_int16(dest->gyro.x);
    dest->gyro.y = swap_int16(dest->gyro.y);
    dest->gyro.z = swap_int16(dest->gyro.z);
    dest->accel_gyro_temp = swap_int16(dest->accel_gyro_temp);
    dest->pressure = swap_uint16(dest->pressure);
    dest->barometer_temp = swap_uint16(dest->barometer_temp);
    dest->pitot = swap_int16(dest->pitot);
    dest->i = swap_int16(dest->i);
    dest->v = swap_int16(dest->v);
    dest->range = swap_int16(dest->range);
    dest->mag.x = swap_int16(dest->mag.x);
    dest->mag.y = swap_int16(dest->mag.y);
    dest->mag.z = swap_int16(dest->mag.z);
    dest->gps.position.lat = swap_int32(dest->gps.position.lat);
    dest->gps.position.lng = swap_int32(dest->gps.position.lng);
    dest->gps.position.alt = swap_int32(dest->gps.position.alt);
    dest->gps.velocity.n = swap_int16(dest->gps.velocity.n);
    dest->gps.velocity.e = swap_int16(dest->gps.velocity.e);
    dest->gps.velocity.d = swap_int16(dest->gps.velocity.d);

    fcs_global_counters.ioboard_packet_rx[dev]++;
    return true;

invalid:
    fcs_global_counters.ioboard_packet_rx_err[dev]++;
    return false;
}

bool _fcs_ahrs_process_accelerometers(float *restrict output,
const struct sensor_packet_t *restrict packets) {
    float rv[3], v[3];
    uint8_t i;
    bool read = false;

    for (i = 0; i < 2; i++) {
        if (!(packets[i].sensor_update_flags & UPDATED_ACCEL)) {
            continue;
        }

        v[0] = ((float)packets[i].accel.x -
                (float)ioboard_calibration[i].accel_bias[0]) *
               ioboard_calibration[i].accel_scale[0];

        v[1] = ((float)packets[i].accel.y -
                (float)ioboard_calibration[i].accel_bias[1]) *
               ioboard_calibration[i].accel_scale[1];

        v[2] = ((float)packets[i].accel.z -
                (float)ioboard_calibration[i].accel_bias[2]) *
               ioboard_calibration[i].accel_scale[2];

        /* Transform v by accelerometer orientation */
        quaternion_vector3_multiply_f(
            rv, ioboard_geometry[i].accel_orientation, v);

        if (!read) {
            output[0] = rv[0];
            output[1] = rv[1];
            output[2] = rv[2];
            read = true;
        } else {
            output[0] = (output[0] + rv[0]) * 0.5f;
            output[1] = (output[1] + rv[1]) * 0.5f;
            output[2] = (output[2] + rv[2]) * 0.5f;
        }
    }

    return read;
}

bool _fcs_ahrs_process_gyroscopes(float *restrict output,
const struct sensor_packet_t *restrict packets) {
    float rv[3], v[3];
    uint8_t i;
    bool read = false;

    for (i = 0; i < 2; i++) {
        if (!(packets[i].sensor_update_flags & UPDATED_GYRO)) {
            continue;
        }

        v[0] = (float)packets[i].gyro.x *
               ioboard_calibration[i].gyro_scale[0];

        v[1] = (float)packets[i].gyro.y *
               ioboard_calibration[i].gyro_scale[1];

        v[2] = (float)packets[i].gyro.z *
               ioboard_calibration[i].gyro_scale[2];

        /* Transform v by gyro orientation */
        quaternion_vector3_multiply_f(
            rv, ioboard_geometry[i].gyro_orientation, v);

        if (!read) {
            output[0] = rv[0];
            output[1] = rv[1];
            output[2] = rv[2];
            read = true;
        } else {
            output[0] = (output[0] + rv[0]) * 0.5f;
            output[1] = (output[1] + rv[1]) * 0.5f;
            output[2] = (output[2] + rv[2]) * 0.5f;
        }
    }

    return read;
}

bool _fcs_ahrs_process_magnetometers(float *restrict output,
const struct sensor_packet_t *restrict packets) {
    float rv[3], v[3];
    uint8_t i;
    bool read = false;

    for (i = 0; i < 2; i++) {
        if (!(packets[i].sensor_update_flags & UPDATED_MAG)) {
            continue;
        }

        v[0] = (float)packets[i].mag.x -
               ioboard_calibration[i].mag_bias[0];
        v[1] = (float)packets[i].mag.y -
               ioboard_calibration[i].mag_bias[1];
        v[2] = (float)packets[i].mag.z -
               ioboard_calibration[i].mag_bias[2];

        /*
        Multiply v by 3x3 magnetometer calibration matrix -- needs to be
        generated by TWOSTEP or similar
        */
        matrix_multiply_f(rv, ioboard_calibration[i].mag_scale,
                          v, 3, 1, 3, 3, 1.0f);

        if (!read) {
            output[0] = rv[0];
            output[1] = rv[1];
            output[2] = rv[2];
            read = true;
        } else {
            output[0] = (output[0] + rv[0]) * 0.5f;
            output[1] = (output[1] + rv[1]) * 0.5f;
            output[2] = (output[2] + rv[2]) * 0.5f;
        }
    }

    return read;
}

bool _fcs_ahrs_process_gps(double *restrict p_output,
float *restrict v_output, const struct sensor_packet_t *restrict packets) {
    double deg_mul = 1e-7;
    float cm_mul = 1e-2f;
    bool read = false;

    /*
    No calibration/bias/scale available for GPS, just average if necessary and
    pass the values through.
    */
    if (packets[0].sensor_update_flags & UPDATED_GPS_POS) {
        p_output[0] = (double)packets[0].gps.position.lat;
        p_output[1] = (double)packets[0].gps.position.lng;
        p_output[2] = (double)packets[0].gps.position.alt;

        v_output[0] = (float)packets[0].gps.velocity.n;
        v_output[1] = (float)packets[0].gps.velocity.e;
        v_output[2] = (float)packets[0].gps.velocity.d;

        read = true;
    }

    if (packets[1].sensor_update_flags & UPDATED_GPS_POS) {
        if (read) {
            p_output[0] += (double)packets[1].gps.position.lat;
            p_output[1] += (double)packets[1].gps.position.lng;
            p_output[2] += (double)packets[1].gps.position.alt;

            v_output[0] += (float)packets[1].gps.velocity.n;
            v_output[1] += (float)packets[1].gps.velocity.e;
            v_output[2] += (float)packets[1].gps.velocity.d;

            deg_mul = 0.5e-7;
            cm_mul = 0.5e-2f;
        } else {
            p_output[0] = (double)packets[1].gps.position.lat;
            p_output[1] = (double)packets[1].gps.position.lng;
            p_output[2] = (double)packets[1].gps.position.alt;

            v_output[0] = (float)packets[1].gps.velocity.n;
            v_output[1] = (float)packets[1].gps.velocity.e;
            v_output[2] = (float)packets[1].gps.velocity.d;

            read = true;
        }
    }

    p_output[0] *= deg_mul;
    p_output[1] *= deg_mul;
    p_output[2] *= (double)cm_mul;

    v_output[0] *= cm_mul;
    v_output[1] *= cm_mul;
    v_output[2] *= cm_mul;

    return read;
}

bool _fcs_ahrs_process_pitots(float *restrict output,
const struct sensor_packet_t *restrict packets) {
    bool read = false;
    float v;

    if (packets[0].sensor_update_flags & UPDATED_ADC_GPIO) {
        *output = ((float)packets[0].pitot +
                   (float)ioboard_calibration[0].pitot_bias) *
                  ioboard_calibration[0].pitot_scale;
        read = true;
    }

    if (packets[1].sensor_update_flags & UPDATED_ADC_GPIO) {
        v = ((float)packets[1].pitot +
             (float)ioboard_calibration[1].pitot_bias) *
            ioboard_calibration[1].pitot_scale;

        if (read) {
            *output = (*output + v) * 0.5f;
        } else {
            *output = v;
            read = true;
        }
    }

    return read;
}

bool _fcs_ahrs_process_barometers(float *restrict output,
const struct sensor_packet_t *restrict packets) {
    bool read = false;
    float v;

    if (packets[0].sensor_update_flags & UPDATED_BAROMETER) {
        *output = ((float)packets[0].pressure +
                   (float)ioboard_calibration[0].barometer_bias) *
                  ioboard_calibration[0].barometer_scale;
        read = true;
    }

    if (packets[1].sensor_update_flags & UPDATED_BAROMETER) {
        v = ((float)packets[1].pressure +
             (float)ioboard_calibration[1].barometer_bias) *
            ioboard_calibration[1].barometer_scale;

        if (read) {
            *output = (*output + v) * 0.5f;
        } else {
            *output = v;
            read = true;
        }
    }

    return read;
}

void _fcs_ahrs_ioboard_reset_geometry(
struct fcs_ahrs_sensor_geometry_t *restrict geometry) {
    geometry->accel_orientation[0] = 0.0;
    geometry->accel_orientation[1] = 0.0;
    geometry->accel_orientation[2] = 0.0;
    geometry->accel_orientation[3] = 1.0;

    geometry->gyro_orientation[0] = 0.0;
    geometry->gyro_orientation[1] = 0.0;
    geometry->gyro_orientation[2] = 0.0;
    geometry->gyro_orientation[3] = 1.0;

    geometry->accel_position[0] = 0.0;
    geometry->accel_position[1] = 0.0;
    geometry->accel_position[2] = 0.0;
}

void _fcs_ahrs_ioboard_reset_calibration(
struct fcs_ahrs_sensor_calibration_t *restrict calibration) {
    memset(calibration->accel_bias, 0, sizeof(int16_t) * 3);
    calibration->accel_scale[0] =
        calibration->accel_scale[1] =
        calibration->accel_scale[2] = 1.0 / ACCEL_SENSITIVITY;

    calibration->gyro_scale[0] =
        calibration->gyro_scale[1] =
        calibration->gyro_scale[2] = 1.0 / GYRO_SENSITIVITY;

    /* Load identity matrices for magnetometer calibration */
    memset(calibration->mag_bias, 0, sizeof(float) * 3);
    memset(calibration->mag_scale, 0, sizeof(float) * 9);
    calibration->mag_scale[0] =
        calibration->mag_scale[4] =
        calibration->mag_scale[8] = 1.0 / MAG_SENSITIVITY;

    calibration->pitot_bias = 0;
    calibration->pitot_scale = 1.0 / 65535.0;

    calibration->barometer_bias = 0;
    calibration->barometer_scale = 0.02;
}

uint32_t _fcs_ahrs_format_control_packet(uint8_t *buf, uint8_t tick,
const double *restrict control_values) {
    assert(buf);
    assert(control_values);

    struct control_packet_t packet;

    packet.tick = tick;
    packet.msg_type = MSG_TYPE_CONTROL;
    packet.gpout = 0;

    double val;
    uint8_t i;
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4u; i++) {
        val = min(max(1.0, control_values[i] * 65535.0), 65535.0);
        /* Swap bytes for big-endian AVR32 */
        packet.pwm[i] = swap_uint16((uint16_t)val);
    }

    /* Calculate the packet's CRC8 */
    packet.crc = fcs_crc8(&(packet.tick), sizeof(packet) - 1u, 0);

    /* Set the packet start/end and COBS-R encode the result */
    struct fcs_cobsr_encode_result result;
    result = fcs_cobsr_encode(&buf[1], sizeof(packet) + 3u, (uint8_t*)&packet,
                              sizeof(packet));
    assert(result.status == FCS_COBSR_ENCODE_OK);

    /* Set the NUL packet delimiters */
    buf[0] = 0;
    buf[result.out_len + 1u] = 0;

    /* Return the total length */
    return result.out_len + 2u;
}
