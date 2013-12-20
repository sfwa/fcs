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
#include "../c66x-csl/ti/csl/cslr_gpio.h"
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
/*
From:
http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf

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
static TRICAL_instance_t magnetometer_calibration[2];

/*
Track the "actual" control position -- NMPC outputs desired position but
we limit the rate at which that can change to the relevant control_rate
value.

The control_scale value is the magnitude of the maximum control input.
*/
static double control_rate[4];
static double control_pos[4];
static double control_scale[4];

/* Latest I/O board state packets */
static struct sensor_packet_t ioboard[2];
static int16_t ioboard_timeout[2];

/* Log packet tick count -- used by the CPU to detect missing log packets */
static uint16_t log_tick;

#define FCS_IOBOARD_RESET_TIMEOUT 50
#define FCS_IOBOARD_PACKET_TIMEOUT 5
#define FCS_IOBOARD_RESTART_WINDOW 10

/* Global FCS state structure */
struct fcs_packet_state_t fcs_global_state;

/* Mirror of the global FCS state structure */
struct fcs_packet_state_t fcs_global_state_mirror;

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
const struct sensor_packet_t *restrict packets, struct fcs_packet_log_t *log);
bool _fcs_ahrs_process_gyroscopes(float *restrict output,
const struct sensor_packet_t *restrict packets, struct fcs_packet_log_t *log);
bool _fcs_ahrs_process_magnetometers(float *restrict output,
const struct sensor_packet_t *restrict packets, struct fcs_packet_log_t *log);
bool _fcs_ahrs_process_gps(double *restrict p_output,
float *restrict v_output, const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log);
bool _fcs_ahrs_process_pitots(float *restrict output,
const struct sensor_packet_t *restrict packets, struct fcs_packet_log_t *log);
bool _fcs_ahrs_process_barometers(float *restrict output,
const struct sensor_packet_t *restrict packets, struct fcs_packet_log_t *log);
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

    /*
    Set I/O board serial baud rates. We don't actually open the streams here
    -- we wait for the I/O board comms to time out and reset the links to
    ensure we get a clean start.
    */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_INT0, 3000000u) == FCS_STREAM_OK);
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_INT1, 3000000u) == FCS_STREAM_OK);
    ioboard_timeout[0] = ioboard_timeout[1] = 0;
    log_tick = 0;

    /* Set up global state */
    memset(&fcs_global_state, 0, sizeof(fcs_global_state));
    fcs_global_state.mode_indicator = 'N';

    memset(&fcs_global_state_mirror, 0, sizeof(fcs_global_state_mirror));
    fcs_global_state_mirror.mode_indicator = 'N';

    fcs_global_counters.ukf_resets++;

    /* Set default geometry and calibration */
    _fcs_ahrs_ioboard_reset_calibration(&ioboard_calibration[0]);
    _fcs_ahrs_ioboard_reset_calibration(&ioboard_calibration[1]);

    _fcs_ahrs_ioboard_reset_geometry(&ioboard_geometry[0]);
    _fcs_ahrs_ioboard_reset_geometry(&ioboard_geometry[1]);

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

    /* Configure UKF covariance and sensor offsets */
    struct fcs_ahrs_sensor_covariance_t covariance;
    struct fcs_ahrs_wmm_field_t field;

    /* TODO: read these from config or something */
    field.mag_field[0] = 0.4;
    field.mag_field[1] = 0.0;
    field.mag_field[2] = 0.0;

    covariance.accel_covariance = 81.0;
    covariance.gyro_covariance = 0.5 * (M_PI / 180.0);
    covariance.mag_covariance = 1.5;
    covariance.gps_position_covariance_h = 1e-11;
    covariance.gps_position_covariance_v = 225.0;
    covariance.gps_velocity_covariance_h = 9.0;
    covariance.gps_velocity_covariance_v = 49.0;
    covariance.pitot_covariance = 100.0;
    covariance.barometer_amsl_covariance = 4.0;

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

    /* Set the UKF model */
    struct fcs_ahrs_dynamics_model_t model = {
        {
            1e-15, 1e-15, 1e-5, /* lat, lon, alt */
            7e-5, 7e-5, 7e-5, /* velocity N, E, D */
            2e-4, 2e-4, 2e-4, /* acceleration x, y, z */
            1e-9, 1e-9, 1e-9, /* attitude roll, pitch, yaw */
            3e-3, 3e-3, 3e-3, /* angular velocity roll, pitch, yaw */
            1e-3, 1e-3, 1e-3, /* angular acceleration roll, pitch, yaw */
            1e-5, 1e-5, 1e-5, /* wind velocity N, E, D */
            1.5e-12, 1.5e-12, 1.5e-12 /* gyro bias x, y, z */
        },
        UKF_MODEL_X8
    };
    ukf_set_process_noise(model.process_noise);
    ukf_choose_dynamics(model.model);

    /* Update the TRICAL instance parameters based on the UKF configuration */
    TRICAL_init(&magnetometer_calibration[0]);
    TRICAL_norm_set(&magnetometer_calibration[0],
                    vector3_norm_f(field.mag_field));
    TRICAL_noise_set(&magnetometer_calibration[0],
                     (float)sqrt(covariance.mag_covariance));

    TRICAL_init(&magnetometer_calibration[1]);
    TRICAL_norm_set(&magnetometer_calibration[1],
                    vector3_norm_f(field.mag_field));
    TRICAL_noise_set(&magnetometer_calibration[1],
                     (float)sqrt(covariance.mag_covariance));

#ifdef __TI_COMPILER_VERSION__
    volatile CSL_GpioRegs *const gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;

    /*
    Set I/O board reset line directions -- 0 for output, 1 for input. We're
    using GPIOs 2 and 3 for IOBOARD_1_RESET_OUT and IOBOARD_2_RESET_OUT
    respectively.
    */
    gpio->BANK_REGISTERS[0].DIR &= ~0xFFFFFFFCu;
#endif
}

void fcs_ahrs_tick(void) {
    if (_fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT0, &ioboard[0])) {
        ioboard_timeout[0] = FCS_IOBOARD_PACKET_TIMEOUT;
    } else {
        ioboard[0].sensor_update_flags = 0;
    }

    if (_fcs_ahrs_read_ioboard_packet(FCS_STREAM_UART_INT1, &ioboard[1])) {
        ioboard_timeout[1] = FCS_IOBOARD_PACKET_TIMEOUT;
    } else {
        ioboard[1].sensor_update_flags = 0;
    }

    /*
    Handle I/O board comms errors. Log and clear any UART errors, but the I/O
    board itself should not reset in that circumstance.

    If the time since last I/O board tick/reset is too high, both the I/O
    board and the stream should be reset.

    HOWEVER, the two I/O boards should never be reset at the same time, since
    that will increase the likelihood of the failsafe system activating.
    */
    if (fcs_stream_check_error(FCS_STREAM_UART_INT0) == FCS_STREAM_ERROR) {
        fcs_global_counters.ioboard_packet_rx_err[0]++;
    }
    if (fcs_stream_check_error(FCS_STREAM_UART_INT1) == FCS_STREAM_ERROR) {
        fcs_global_counters.ioboard_packet_rx_err[1]++;
    }

#ifdef __TI_COMPILER_VERSION__
    volatile CSL_GpioRegs *const gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;
#endif

    /*
    If board 0 has timed out, and board 1 has had a while since reset, start
    the reset for board 0. Otherwise, just decrement the timeout counter.
    */
    if (ioboard_timeout[0] <= 0 &&
            ioboard_timeout[1] < FCS_IOBOARD_RESTART_WINDOW) {
        ioboard_timeout[0] = FCS_IOBOARD_RESET_TIMEOUT;
        fcs_global_counters.ioboard_resets[0]++;

#ifdef __TI_COMPILER_VERSION__
        /* Assert IOBOARD_1_RESET_OUT on GPIO 2 */
        gpio->BANK_REGISTERS[0].OUT_DATA |= 1u << 2u;
#endif

        assert(fcs_stream_open(FCS_STREAM_UART_INT0) == FCS_STREAM_OK);
    } else if (ioboard_timeout[0] > INT16_MIN) {
        ioboard_timeout[0]--;

#ifdef __TI_COMPILER_VERSION__
        /* De-assert IOBOARD_1_RESET_OUT on GPIO 2 */
        gpio->BANK_REGISTERS[0].OUT_DATA &= ~(1u << 2u);
#endif
    }

    /*
    If board 1 has timed out, and board 0 has had a while since reset but has
    not timed out itself, reset board 1. This avoids the case where both
    boards are reset at the same time.
    */
    if (ioboard_timeout[1u] <= 0 &&
            0 < ioboard_timeout[0] &&
            ioboard_timeout[0] < FCS_IOBOARD_RESTART_WINDOW) {
        ioboard_timeout[1u] = FCS_IOBOARD_RESET_TIMEOUT;
        fcs_global_counters.ioboard_resets[1u]++;

#ifdef __TI_COMPILER_VERSION__
        /* Assert IOBOARD_2_RESET_OUT on GPIO 3 */
        gpio->BANK_REGISTERS[0].OUT_DATA |= 1u << 3u;
#endif

        assert(fcs_stream_open(FCS_STREAM_UART_INT1) == FCS_STREAM_OK);
    } else if (ioboard_timeout[1u] > INT16_MIN) {
        ioboard_timeout[1u]--;

#ifdef __TI_COMPILER_VERSION__
        /* De-assert IOBOARD_2_RESET_OUT on GPIO 3 */
        gpio->BANK_REGISTERS[0].OUT_DATA &= ~(1u << 3u);
#endif
    }

    struct fcs_packet_log_t log_packet;
    fcs_comms_init_log(&log_packet, log_tick++);

    /*
    Read sensor data from latest I/O board packets, and convert to UKF
    input format/units. These procedures combine multiple simultaneous sensor
    readings from the two boards into a single virtual reading, generally by
    applying board calibration to each and then averaging the result.
    */
    float v[3];
    double p[3];

    ukf_sensor_clear();

    if (_fcs_ahrs_process_accelerometers(v, ioboard, &log_packet)) {
        ukf_sensor_set_accelerometer(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_gyroscopes(v, ioboard, &log_packet)) {
        ukf_sensor_set_gyroscope(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_magnetometers(v, ioboard, &log_packet)) {
        ukf_sensor_set_magnetometer(v[0], v[1], v[2]);
    }
    if (_fcs_ahrs_process_pitots(v, ioboard, &log_packet)) {
        ukf_sensor_set_pitot_tas(v[0]);
    }
    if (_fcs_ahrs_process_barometers(v, ioboard, &log_packet)) {
        ukf_sensor_set_barometer_amsl(v[0]);
    }

    /* Only read I/O board GPS if we don't have a Piksi solution this frame */
    if (fcs_global_piksi_solution.updated) {
        fcs_global_piksi_solution.updated = false;

        ukf_sensor_set_gps_position(fcs_global_piksi_solution.lat,
                                    fcs_global_piksi_solution.lon,
                                    fcs_global_piksi_solution.alt);
        ukf_sensor_set_gps_velocity(fcs_global_piksi_solution.velocity[0],
                                    fcs_global_piksi_solution.velocity[1u],
                                    fcs_global_piksi_solution.velocity[2u]);

        /* TODO: update sensor covariance for GPS based on Piksi status */
    } else if (_fcs_ahrs_process_gps(p, v, ioboard, &log_packet)) {
        ukf_sensor_set_gps_position(p[0], p[1u], p[2u]);
        ukf_sensor_set_gps_velocity(v[0], v[1u], v[2u]);

        /* TODO: update sensor covariance for GPS based on DOP */
    }

    /* TODO: Read control set values from NMPC */
    double control_set[4] = { 0.0, 0.0, 0.0, 0.0 };

    /*
    Write current control values to I/O boards -- the CPLD replicates the I/O
    board output stream so we only need to write to one.
    */
    size_t control_len;
    uint8_t control_buf[16];
    control_len = _fcs_ahrs_format_control_packet(
        control_buf,
        (uint8_t)(fcs_global_state.solution_time & 0xFFu),
        control_set
    );
    assert(control_len < 16u);
    fcs_stream_write(FCS_STREAM_UART_INT0, control_buf, control_len);

    /* Increment transmit counters */
    fcs_global_counters.ioboard_packet_tx[0]++;
    fcs_global_counters.ioboard_packet_tx[1u]++;

    /*
    Work out the nominal current control position, taking into account the
    control response time configured in control_rates. Log the result.
    */
    uint8_t i;
    int16_t control_pos_log[4];
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4; i++) {
        double delta = control_set[i] - control_pos[i],
               limit = control_rate[i] * AHRS_DELTA;
        control_pos[i] += limitabs(delta, limit);
        control_pos_log[i] = (int16_t)(control_pos[i] / control_scale[i] *
                                       INT16_MAX);
    }

    fcs_comms_add_log_sensor_value(&log_packet, FCS_SENSOR_TYPE_CONTROL_POS,
                                   0, (uint8_t*)control_pos_log,
                                   sizeof(control_pos_log));

    /*
    Write the log values to stream 1 -- the CPLD will route this to the CPU
    UART
    */
    size_t log_len;
    uint8_t log_buf[256];
    log_len = fcs_comms_serialize_log(log_buf, sizeof(log_buf), &log_packet);
    fcs_stream_write(FCS_STREAM_UART_INT1, log_buf, log_len);

    /*
    Run the UKF, taking sensor readings and current control position into
    account
    */
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
    struct fcs_packet_state_t state;

    /* Increment solution time with wrap-around at 30 bits (1073741.823s) */
    state.solution_time = fcs_global_state.solution_time + 1u;
    state.solution_time &= 0x3FFFFFFF;

    /* Convert lat/lon to degrees */
    state.lat = s->position[0] * (180.0/M_PI);
    state.lon = s->position[1] * (180.0/M_PI);
    state.alt = s->position[2];

    memcpy(state.velocity, s->velocity, 3 * sizeof(double));
    memcpy(state.wind_velocity, s->wind_velocity, 3 * sizeof(double));

    /* Convert angular velocity to degrees/s */
    state.angular_velocity[0] = s->angular_velocity[0] * (180.0/M_PI);
    state.angular_velocity[1] = s->angular_velocity[1] * (180.0/M_PI);
    state.angular_velocity[2] = s->angular_velocity[2] * (180.0/M_PI);

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

    state.yaw = yaw;
    state.pitch = pitch;
    state.roll = roll;

    /*
    Work out 95th percentile confidence intervals for each of the output
    values, based on the current state covariance matrix and the assumption
    that error will follow a Gaussian distribution
    */

    /*
    Ignore changes in lon with varying lat -- small angles and all that.

    Formula for m per degree latitude is approx (2 * pi / 360) * r
    */
    state.lat_lon_uncertainty = 1.96 *
        sqrt(max(covariance[0], covariance[1])) * 6378000.0;
    state.alt_uncertainty = 1.96 * sqrt(covariance[2]);

    state.velocity_uncertainty[0] = 1.96 * sqrt(covariance[3]);
    state.velocity_uncertainty[1] = 1.96 * sqrt(covariance[4]);
    state.velocity_uncertainty[2] = 1.96 * sqrt(covariance[5]);

    state.wind_velocity_uncertainty[0] = 1.96 * sqrt(covariance[18]);
    state.wind_velocity_uncertainty[1] = 1.96 * sqrt(covariance[19]);
    state.wind_velocity_uncertainty[2] = 1.96 * sqrt(covariance[20]);

    /* Rotation around +X, +Y and +Z -- roll, pitch, yaw */
    state.roll_uncertainty = 1.96 * sqrt(covariance[9]) * (180.0/M_PI);
    state.pitch_uncertainty = 1.96 * sqrt(covariance[10]) * (180.0/M_PI);
    state.yaw_uncertainty = 1.96 * sqrt(covariance[11]) * (180.0/M_PI);

    state.angular_velocity_uncertainty[0] =
        1.96 * sqrt(covariance[12]) * (180.0/M_PI);
    state.angular_velocity_uncertainty[1] =
        1.96 * sqrt(covariance[13]) * (180.0/M_PI);
    state.angular_velocity_uncertainty[2] =
        1.96 * sqrt(covariance[14]) * (180.0/M_PI);

    /* Check that the state is valid */
    enum fcs_validation_result_t valid;
    valid = fcs_comms_validate_state(&state);

    if (valid != FCS_VALIDATION_OK) {
        /*
        TODO:
        The state is invalid. Reset the UKF's state vector and covariance
        matrix.
        */

        state.mode_indicator = 'N';
        fcs_global_counters.ukf_resets++;
    } else {
        state.mode_indicator = 'A';
    }

#ifdef __TI_COMPILER_VERSION__
    /*
    Use a semaphore to prevent the NMPC code accessing the state while we're
    updating it.
    */
    volatile CSL_SemRegs *const semaphore = (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_GLOBAL_STATE];
    assert(sem_val == 1u);
#endif

    /* Copy the state to the global state and its mirror */
    memcpy(&fcs_global_state, &state, sizeof(state));
    memcpy(&fcs_global_state_mirror, &state, sizeof(state));

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

/*
Apply scale and bias calibration to the accelerometer measurements, then apply
the rotation configured in the I/O board geometry. If multiple measurements
are available, they are averaged after scaling.

Return true if any values were read, and false otherwise.
*/
bool _fcs_ahrs_process_accelerometers(float *restrict output,
const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log_rec) {
    assert(output && packets && log_rec);

    float rv[3], v[3];
    uint8_t i;
    bool read = false;

    for (i = 0; i < 2; i++) {
        if (!(packets[i].sensor_update_flags & UPDATED_ACCEL)) {
            continue;
        }

        /* Log sensor values */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_ACCELEROMETER, i,
            (uint8_t*)&packets[i].accel, sizeof(packets[i].accel)
        );

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

/*
Apply the rotation configured in the I/O board geometry, then apply scale
calibration to the gyro measurements. If multiple measurements are available,
they are averaged after scaling.

Gyro bias estimation and compensation is handled by the AHRS UKF itself.

Return true if any values were read, and false otherwise.
*/
bool _fcs_ahrs_process_gyroscopes(float *restrict output,
const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log_rec) {
    assert(output && packets && log_rec);

    float rv[3], v[3];
    uint8_t i;
    bool read = false;

    for (i = 0; i < 2; i++) {
        if (!(packets[i].sensor_update_flags & UPDATED_GYRO)) {
            continue;
        }

        /* Log sensor values */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_GYROSCOPE, i,
            (uint8_t*)&packets[i].gyro, sizeof(packets[i].gyro)
        );

        /* Apply scale calibration */
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

/*
Apply the rotation configured in the I/O board geemoetry, then run TRICAL
calibration on the resulting measurement. Take the mean of the calibrated
measurements if there are multiple values available.

Return true if any values were read, and false otherwise.
*/
bool _fcs_ahrs_process_magnetometers(float *restrict output,
const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log_rec) {
    assert(output && packets && log_rec);

    float rv[3], v[3];
    uint8_t i;
    bool read = false;

    for (i = 0; i < 2; i++) {
        if (!(packets[i].sensor_update_flags & UPDATED_MAG)) {
            continue;
        }

        /* Log sensor values */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_MAGNETOMETER, i,
            (uint8_t*)&packets[i].mag, sizeof(packets[i].mag)
        );

        /* Scale based on sensitivity */
        v[0] = (float)packets[i].mag.x * (1.0f / MAG_SENSITIVITY);
        v[1] = (float)packets[i].mag.y * (1.0f / MAG_SENSITIVITY);
        v[2] = (float)packets[i].mag.z * (1.0f / MAG_SENSITIVITY);

        /* Transform v by magnetometer orientation */
        quaternion_vector3_multiply_f(
            rv, ioboard_geometry[i].mag_orientation, v);

        TRICAL_estimate_update(&magnetometer_calibration[i], rv);
        TRICAL_measurement_calibrate(&magnetometer_calibration[i], rv, v);

        if (isnan(v[0]) || isnan(v[1]) || isnan(v[2])) {
            /*
            TRICAL has blown up -- reset this instance and ignore the current
            reading.
            */
            TRICAL_reset(&magnetometer_calibration[i]);
            fcs_global_counters.trical_resets[i]++;
        } else if (!read) {
            output[0] = v[0];
            output[1] = v[1];
            output[2] = v[2];
            read = true;
        } else {
            output[0] = (output[0] + v[0]) * 0.5f;
            output[1] = (output[1] + v[1]) * 0.5f;
            output[2] = (output[2] + v[2]) * 0.5f;
        }
    }

    return read;
}

bool _fcs_ahrs_process_gps(double *restrict p_output,
float *restrict v_output, const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log_rec) {
    assert(p_output && v_output && packets && log_rec);

    double deg_mul = 1e-7 * (M_PI/180.0);
    float cm_mul = 1e-2f;
    bool read = false;

    /*
    No calibration/bias/scale available for GPS, just average if necessary and
    pass the values through.
    */
    if (packets[0].sensor_update_flags & UPDATED_GPS_POS) {
        /* Log sensor values */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_GPS_POSITION, 0,
            (uint8_t*)&packets[0].gps.position,
            sizeof(packets[0].gps.position)
        );
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_GPS_VELOCITY, 0,
            (uint8_t*)&packets[0].gps.velocity,
            sizeof(packets[0].gps.velocity)
        );

        p_output[0] = (double)packets[0].gps.position.lat;
        p_output[1] = (double)packets[0].gps.position.lng;
        p_output[2] = (double)packets[0].gps.position.alt;

        v_output[0] = (float)packets[0].gps.velocity.n;
        v_output[1] = (float)packets[0].gps.velocity.e;
        v_output[2] = (float)packets[0].gps.velocity.d;

        read = true;
    }

    if (packets[1].sensor_update_flags & UPDATED_GPS_POS) {
        /* Log sensor values */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_GPS_POSITION, 1,
            (uint8_t*)&packets[1].gps.position,
            sizeof(packets[1].gps.position)
        );
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_GPS_VELOCITY, 1,
            (uint8_t*)&packets[1].gps.velocity,
            sizeof(packets[1].gps.velocity)
        );

        if (read) {
            p_output[0] += (double)packets[1].gps.position.lat;
            p_output[1] += (double)packets[1].gps.position.lng;
            p_output[2] += (double)packets[1].gps.position.alt;

            v_output[0] += (float)packets[1].gps.velocity.n;
            v_output[1] += (float)packets[1].gps.velocity.e;
            v_output[2] += (float)packets[1].gps.velocity.d;

            deg_mul = 0.5e-7 * (M_PI/180.0);
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

    /* Scale the inputs based on the I/O board GPS units */
    p_output[0] *= deg_mul;
    p_output[1] *= deg_mul;
    p_output[2] *= (double)cm_mul;

    v_output[0] *= cm_mul;
    v_output[1] *= cm_mul;
    v_output[2] *= cm_mul;

    return read;
}

/*
Apply bias and scale calibration to pitot data from the I/O boards, then
take the mean of the results if there are multiple values available.

Return true if any values were read, and false otherwise.
*/
bool _fcs_ahrs_process_pitots(float *restrict output,
const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log_rec) {
    assert(output && packets && log_rec);

    bool read = false;
    float v;

    if (packets[0].sensor_update_flags & UPDATED_ADC_GPIO) {
        /* Log sensor value */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_PRESSURE_TEMP, 0,
            (uint8_t*)&packets[0].pitot, sizeof(packets[0].pitot)
        );

        /* Apply calibration to the output value */
        *output = ((float)packets[0].pitot +
                   (float)ioboard_calibration[0].pitot_bias) *
                  ioboard_calibration[0].pitot_scale;
        read = true;
    }

    if (packets[1].sensor_update_flags & UPDATED_ADC_GPIO) {
        /* Log sensor value */
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_PRESSURE_TEMP, 1,
            (uint8_t*)&packets[1].pitot, sizeof(packets[1].pitot)
        );

        /* Apply calibration to the output value */
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

/*
Apply bias and scale calibration to the barometric pressure data from the
I/O boards, then take the mean of the results if there are multiple values
available.

Return true if any values were read, and false otherwise.
*/
bool _fcs_ahrs_process_barometers(float *restrict output,
const struct sensor_packet_t *restrict packets,
struct fcs_packet_log_t *log_rec) {
    assert(output && packets && log_rec);

    bool read = false;
    float v;

    if (packets[0].sensor_update_flags & UPDATED_BAROMETER) {
        /* Log sensor values */
        int16_t val[2u] = { packets[0].pressure, packets[0].barometer_temp };
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_PRESSURE_TEMP, 0, (uint8_t*)val,
            sizeof(val)
        );

        /* Apply calibration to the output value */
        *output = ((float)packets[0].pressure +
                   (float)ioboard_calibration[0].barometer_bias) *
                  ioboard_calibration[0].barometer_scale;
        read = true;
    }

    if (packets[1].sensor_update_flags & UPDATED_BAROMETER) {
        /* Log sensor values */
        int16_t val[2u] = { packets[1].pressure, packets[1].barometer_temp };
        fcs_comms_add_log_sensor_value(
            log_rec, FCS_SENSOR_TYPE_PRESSURE_TEMP, 1u, (uint8_t*)val,
            sizeof(val)
        );

        /* Apply calibration to the output value */
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

/*
Reset the I/O board geometry configuration. The default configuration has
all sensors located at the centre of mass, and pointing along the +x
axis (forwards).
*/
void _fcs_ahrs_ioboard_reset_geometry(
struct fcs_ahrs_sensor_geometry_t *restrict geometry) {
    assert(geometry);

    memset(geometry->accel_orientation, 0,
           sizeof(geometry->accel_orientation));
    geometry->accel_orientation[3] = 1.0;

    memset(geometry->gyro_orientation, 0,
           sizeof(geometry->gyro_orientation));
    geometry->gyro_orientation[3] = 1.0;

    memset(geometry->mag_orientation, 0,
           sizeof(geometry->mag_orientation));
    geometry->mag_orientation[3] = 1.0;

    memset(geometry->accel_position, 0,
           sizeof(geometry->accel_position));
}

/*
Reset the I/O board sensor calibration, including the TRICAL state for
the magnetometers
*/
void _fcs_ahrs_ioboard_reset_calibration(
struct fcs_ahrs_sensor_calibration_t *restrict calibration) {
    assert(calibration);

    memset(calibration->accel_bias, 0, sizeof(int16_t) * 3);
    calibration->accel_scale[0] =
        calibration->accel_scale[1] =
        calibration->accel_scale[2] = 1.0 / ACCEL_SENSITIVITY;

    calibration->gyro_scale[0] =
        calibration->gyro_scale[1] =
        calibration->gyro_scale[2] = 1.0 / GYRO_SENSITIVITY;

    calibration->pitot_bias = 0;
    calibration->pitot_scale = 1.0 / 65535.0;

    calibration->barometer_bias = 0;
    calibration->barometer_scale = 0.02;

    TRICAL_reset(&magnetometer_calibration[0]);
    fcs_global_counters.trical_resets[0]++;

    TRICAL_reset(&magnetometer_calibration[1]);
    fcs_global_counters.trical_resets[1]++;
}

/*
Serialize a control packet containing `control_values` into `buf`.
*/
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
