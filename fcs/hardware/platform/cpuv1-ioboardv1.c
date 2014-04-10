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

#include "../board.h"
#include "../../util/util.h"
#include "../../util/3dmath.h"
#include "../../ukf/cukf.h"
#include "../../nmpc/cnmpc.h"
#include "../../drivers/stream.h"
#include "../../stats/stats.h"
#include "../../TRICAL/TRICAL.h"
#include "../../ahrs/ahrs.h"
#include "../../control/control.h"
#include "../../exports/exports.h"
#include "../../exports/parameter.h"
#include "../../exports/calibration.h"


/*
Check a vector for NaN values; return true if any are found, and false
otherwise
*/
inline static bool _vec_hasnan_f(const float *vec, size_t n) {
    for (; n; n--) {
        if (isnan(vec[n])) {
            return true;
        }
    }
    return false;
}


#define ACCEL_SENSITIVITY 4096.0f /* LSB/g @ ±8g FS */
#define GYRO_SENSITIVITY 65.5f /* LSB/(deg/s) @ 500deg/s FS */
#define MAG_SENSITIVITY 1090.0f /* LSB/G @ ±2G FS */

#define FCS_IOBOARD_COUNT 2
#define FCS_IOBOARD_RESET_TIMEOUT 50
#define FCS_IOBOARD_PACKET_TIMEOUT 5

static struct fcs_calibration_map_t board_calibration;
static TRICAL_instance_t board_accel_trical_instances[FCS_IOBOARD_COUNT];
static TRICAL_instance_t board_mag_trical_instances[FCS_IOBOARD_COUNT];
static double board_mag_trical_update_attitude[FCS_IOBOARD_COUNT];
static int16_t board_timeout[FCS_IOBOARD_COUNT];
static double board_reference_pressure, board_reference_alt;


/* Prototypes of internal functions */
bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, size_t board_id,
struct fcs_log_t *out_measurements);
static void _update_pitot_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, size_t i);
static void _update_barometer_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, double reference_pressure, size_t i);
static void _update_magnetometer_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, TRICAL_instance_t *instance,
const double *attitude, double *last_update_attitude, const double *wmm_field,
double wmm_field_norm, size_t i);
static void _update_accelerometer_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, TRICAL_instance_t *instance, size_t i);
static void _apply_accelerometer_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap);
static void _apply_gyroscope_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap);
static void _apply_magnetometer_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap,
double wmm_field_norm);
static void _apply_pitot_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap,
double aero_static_pressure, double aero_static_temp);
static void _apply_barometer_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap,
double reference_pressure, double aero_static_temp, double reference_alt);
static void _set_gps_position(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap);
static void _set_gps_velocity(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap);
static void _set_hal_sensor_value_f32(struct fcs_log_t *plog,
enum fcs_parameter_type_t param_type, const double *restrict value,
const double *restrict variance, size_t n);


void fcs_board_init_platform(void) {
    /*
    Set I/O board serial baud rates. We don't actually open the streams here
    -- we wait for the I/O board comms to time out and reset the links to
    ensure we get a clean start.
    */
    enum fcs_stream_result_t result;
    result = fcs_stream_set_rate(FCS_STREAM_UART_INT0, 2604168u);
    assert(result == FCS_STREAM_OK);
    result = fcs_stream_set_rate(FCS_STREAM_UART_INT1, 2604168u);
    assert(result == FCS_STREAM_OK);

#ifdef __TI_COMPILER_VERSION__
    volatile CSL_GpioRegs *const gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;

    /*
    Set I/O board reset line directions -- 0 for output, 1 for input. We're
    using GPIOs 2 and 3 for IOBOARD_1_RESET_OUT and IOBOARD_2_RESET_OUT
    respectively.
    */
    gpio->BANK_REGISTERS[0].DIR &= 0xFFFFFFFCu;
#endif

    /* Set up default sensor calibration */
    struct fcs_calibration_t accel_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_ACCELEROMETER_XYZ,
        .calibration_type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
                            FCS_CALIBRATION_BIAS_SCALE_3X3,
        .error = 0.98f, /* about 0.1g */
        .params = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        },
        .orientation = { 0.0f, 0.0f, 0.0f, 1.0f },
        .offset = { 0.0f, 0.0f, 0.0f },
        .scale_factor = 1.0f / ACCEL_SENSITIVITY
    };
    struct fcs_calibration_t gyro_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_GYROSCOPE_XYZ,
        .calibration_type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
                            FCS_CALIBRATION_BIAS_SCALE_3X3,
        .error = 0.0349f, /* approx 2 degrees */
        .params = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        },
        .orientation = { 0.0f, 0.0f, 0.0f, 1.0f },
        .offset = { 0.0f, 0.0f, 0.0f },
        .scale_factor = (float)(M_PI/180.0) / GYRO_SENSITIVITY
    };
    struct fcs_calibration_t mag_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_MAGNETOMETER_XYZ,
        .calibration_type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
                            FCS_CALIBRATION_BIAS_SCALE_3X3,
        .error = 0.1f, /* = 0.1 Gauss */
        .params = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        },
        .orientation = { 0.0f, 0.0f, 0.0f, 1.0f },
        .offset = { 0.0f, 0.0f, 0.0f },
        .scale_factor = 1.0f / MAG_SENSITIVITY
    };
    struct fcs_calibration_t gps_position_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_GPS_POSITION_LLA,
        .calibration_type = FCS_CALIBRATION_NONE,
        .error = 3.1623e-7f,
        .scale_factor = 1.0f
    };
    struct fcs_calibration_t gps_velocity_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_GPS_VELOCITY_NED,
        .calibration_type = FCS_CALIBRATION_BIAS_SCALE_3X3,
        .error = 3.0f,
        .params = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        },
        .scale_factor = 1e-3f
    };
    struct fcs_calibration_t pitot_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_PITOT,
        .calibration_type = FCS_CALIBRATION_BIAS_SCALE_1D,
        .error = 3.0f,
        /*
        Scale pitot output voltage from -2000.0 to 2000.0Pa, with 0.2533 being
        1.65V (=0kPa)
        */
        .params = { 0.2533f, 4000.0f / 0.5066f },
        .scale_factor = 1.0f
    };
    struct fcs_calibration_t barometer_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_PRESSURE_TEMP,
        .calibration_type = FCS_CALIBRATION_BIAS_SCALE_1D,
        .error = 1.0f,
        /*
        0.02 is the sensor scale factor for conversion to mbar; multiply by
        100 for Pa.
        */
        .params = { 0.0, 1.0f },
        .scale_factor = 0.02f * 100.0f
    };
    struct fcs_calibration_t iv_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .device = 0,
        .type = FCS_PARAMETER_IV,
        .calibration_type = FCS_CALIBRATION_BIAS_SCALE_2D,
        .error = 1.0f,
        /*
        To convert from 16-bit unsigned ints to input voltage (ie. voltage
        into the op-amp), divide by 65536 then multiply by (VCC*2). So, for
        5V VCC, divide by 65536 and multiply by 10.

        Then, for the Attopilot 180A I/V sensor, the scaling factors are as
        follows:
        Voltage: 63.69mV/V (multiply by a factor of 15.70)
        Current: 18.3mV/A (multiply by a factor of 54.64)

        So for a raw reading of 4114 for the voltage input with a 5V VCC,
        divide by (65536 / 10) to get 0.628V in, and then multiply by the I/V
        sensor voltage scale factor of 15.7 to get an input voltage of 9.86V.
        */
        .params = { 0.0, 15.70f, 0.0, 54.64f },
        .scale_factor = 10.0f / 65535.0f
    };

    uint8_t i;
    for (i = 0; i < FCS_IOBOARD_COUNT; i++) {
        accel_calibration.device = i;
        gyro_calibration.device = i;
        mag_calibration.device = i;
        gps_position_calibration.device = i;
        gps_velocity_calibration.device = i;
        pitot_calibration.device = i;
        barometer_calibration.device = i;
        iv_calibration.device = i;

        fcs_calibration_map_register_calibration(
            &board_calibration, &accel_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &gyro_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &mag_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &gps_position_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &gps_velocity_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &pitot_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &barometer_calibration);
        fcs_calibration_map_register_calibration(
            &board_calibration, &iv_calibration);
    }

    /*
    Update the TRICAL instance parameters.
    */
    for (i = 0; i < FCS_IOBOARD_COUNT; i++) {
        TRICAL_init(&board_mag_trical_instances[i]);
        /*
        Set the norm to 1.0, because the sensor calibration scale factor is
        set such that TRICAL will always work with (theoretically) unit
        vectors.
        */
        TRICAL_norm_set(&board_mag_trical_instances[i], 1.0f);
        TRICAL_noise_set(&board_mag_trical_instances[i], 1e-3f);

        TRICAL_init(&board_accel_trical_instances[i]);
        TRICAL_norm_set(&board_accel_trical_instances[i], 1.0f);
        TRICAL_noise_set(&board_accel_trical_instances[i], 1.0f);
    }

    /* Update reference values */
    board_reference_pressure = STANDARD_PRESSURE;
    board_reference_alt = 0.0;
}

void fcs_board_tick(void) {
    uint8_t out_buf[FCS_LOG_SERIALIZED_LENGTH];
    size_t out_buf_len, write_len, i;
    struct fcs_log_t *measurement_log, *hal_log, *estimate_log, *control_log;
    struct fcs_log_t out_log;
    double attitude[4], wmm_field[3], wmm_field_norm, wmm_field_norm_inv;
    double static_pressure, static_temp;
    struct fcs_parameter_t param;
    enum fcs_mode_t ahrs_mode;
    bool got_result;

    /*
    Read attitude, WMM field, static pressure, static temp and AHRS mode from
    old estimate log
    */
    estimate_log = fcs_exports_log_open(FCS_LOG_TYPE_ESTIMATE, 'r');
    assert(estimate_log);

    got_result = fcs_parameter_find_by_type_and_device(
        estimate_log, FCS_PARAMETER_ESTIMATED_ATTITUDE_Q, 0, &param);
    if (got_result) {
        fcs_parameter_get_values_d(&param, attitude, 4u);
    } else {
        attitude[X] = attitude[Y] = attitude[Z] = 0.0;
        attitude[W] = 1.0;
    }

    got_result = fcs_parameter_find_by_type_and_device(
        estimate_log, FCS_PARAMETER_ESTIMATED_WMM_FIELD, 0, &param);
    if (got_result) {
        fcs_parameter_get_values_d(&param, wmm_field, 3u);
        wmm_field_norm = vector3_norm_d(wmm_field);
        wmm_field_norm_inv = 1.0 / wmm_field_norm;
        wmm_field[X] *= wmm_field_norm_inv;
        wmm_field[Y] *= wmm_field_norm_inv;
        wmm_field[Z] *= wmm_field_norm_inv;
    } else {
        wmm_field[X] = 1.0;
        wmm_field[Y] = wmm_field[Z] = 0.0;
        wmm_field_norm = 1.0;
    }

    got_result = fcs_parameter_find_by_type_and_device(
        estimate_log, FCS_PARAMETER_ESTIMATED_STATIC_PRESSURE, 0, &param);
    if (got_result) {
        fcs_parameter_get_values_d(&param, &static_pressure, 1u);
    } else {
        static_pressure = board_reference_pressure;
    }

    got_result = fcs_parameter_find_by_type_and_device(
        estimate_log, FCS_PARAMETER_ESTIMATED_STATIC_TEMP, 0, &param);
    if (got_result) {
        fcs_parameter_get_values_d(&param, &static_temp, 1u);
    } else {
        static_temp = STANDARD_TEMP;
    }

    got_result = fcs_parameter_find_by_type_and_device(
        estimate_log, FCS_PARAMETER_AHRS_MODE, 0, &param);
    if (got_result) {
        ahrs_mode = (enum fcs_mode_t)param.data.u8[0];
    } else {
        ahrs_mode = FCS_MODE_INITIALIZING;
    }

    /* Read the latest measurements from the I/O boards */
    measurement_log = fcs_exports_log_open(FCS_LOG_TYPE_MEASUREMENT, 'w');
    assert(measurement_log);

    for (i = 0; i < FCS_IOBOARD_COUNT; i++){
        if (_fcs_read_ioboard_packet(
                (enum fcs_stream_device_t)(FCS_STREAM_UART_INT0 + i), i,
                measurement_log)) {
            board_timeout[i] = FCS_IOBOARD_PACKET_TIMEOUT;

            /* Get reference pressure and altitude */
            got_result = fcs_parameter_find_by_type_and_device(
                measurement_log, FCS_PARAMETER_DERIVED_REFERENCE_PRESSURE,
                (uint8_t)i, &param);
            if (got_result) {
                fcs_parameter_get_values_d(&param, &board_reference_pressure,
                                           1u);
            }

            got_result = fcs_parameter_find_by_type_and_device(
                measurement_log, FCS_PARAMETER_DERIVED_REFERENCE_ALT,
                (uint8_t)i, &param);
            if (got_result) {
                fcs_parameter_get_values_d(&param, &board_reference_alt, 1u);
            }

            /* Continuously update the magnetometer calibration */
            _update_magnetometer_calibration(
                measurement_log, &board_calibration,
                &board_mag_trical_instances[i], attitude,
                &board_mag_trical_update_attitude[i], wmm_field,
                wmm_field_norm, i);

            /*
            Run TRICAL on the current accelerometer results when in
            calibration mode, and call the other sensor calibration handlers.
            */
            if (ahrs_mode == FCS_MODE_CALIBRATING) {
                _update_pitot_calibration(
                    measurement_log, &board_calibration, i);
                _update_barometer_calibration(
                    measurement_log, &board_calibration,
                    board_reference_pressure, i);
                _update_accelerometer_calibration(
                    measurement_log, &board_calibration,
                    &board_accel_trical_instances[i], i);
            }
        }

        /*
        Handle I/O board comms errors. Log and clear any UART errors, but the
        I/O board itself should not reset in that circumstance.

        If the time since last I/O board tick/reset is too high, the stream
        should be reset.
        */
        if (fcs_stream_check_error((enum fcs_stream_device_t)
        		(FCS_STREAM_UART_INT0 + i)) == FCS_STREAM_ERROR) {
            fcs_global_counters.ioboard_packet_rx_err[i]++;
        }

        /*
        If the board has timed out, start the reset for stream 0. Otherwise,
        just decrement the timeout counter.
        */
        if (board_timeout[i] <= 0) {
            board_timeout[i] = FCS_IOBOARD_RESET_TIMEOUT;
            fcs_global_counters.ioboard_resets[i]++;

            enum fcs_stream_result_t result;
            result = fcs_stream_open(
                (enum fcs_stream_device_t)(FCS_STREAM_UART_INT0 + i));
            assert(result == FCS_STREAM_OK);
        } else if (board_timeout[i] > INT16_MIN) {
            board_timeout[i]--;
        }
    }

    hal_log = fcs_exports_log_open(FCS_LOG_TYPE_SENSOR_HAL, 'w');
    assert(hal_log);

    /* Add calibrated virtual sensor values to the HAL log */
    _apply_accelerometer_calibration(measurement_log, hal_log,
                                     &board_calibration);
    _apply_gyroscope_calibration(measurement_log, hal_log,
                                 &board_calibration);
    _apply_magnetometer_calibration(measurement_log, hal_log,
                                    &board_calibration, wmm_field_norm);
    _apply_pitot_calibration(measurement_log, hal_log, &board_calibration,
                             static_pressure, static_temp);
    _apply_barometer_calibration(measurement_log, hal_log, &board_calibration,
                                 board_reference_pressure, static_temp,
                                 board_reference_alt);
    _set_gps_position(measurement_log, hal_log, &board_calibration);
    _set_gps_velocity(measurement_log, hal_log, &board_calibration);

    hal_log = fcs_exports_log_close(hal_log);
    assert(!hal_log);

    /*
    Merge the AHRS estimate log and the NMPC control log and send them to the
    I/O boards.
    */
    control_log = fcs_exports_log_open(FCS_LOG_TYPE_CONTROL, 'r');
    assert(control_log);

    /*
    Copy control log then merge estimate log so that if there's not enough
    space for whatever reason, the control output still gets through.
    */
    memcpy(&out_log, control_log, sizeof(struct fcs_log_t));
    (void)fcs_log_merge(&out_log, estimate_log);

    control_log = fcs_exports_log_close(control_log);
    assert(!control_log);

    measurement_log = fcs_exports_log_close(measurement_log);
    assert(!measurement_log);

    estimate_log = fcs_exports_log_close(estimate_log);
    assert(!estimate_log);

    out_buf_len = fcs_log_serialize(out_buf, sizeof(out_buf), &out_log);
    /* FIXME: limitation of current stream driver */
    assert(out_buf_len < 255u);

    write_len = fcs_stream_write(FCS_STREAM_UART_INT0, out_buf, out_buf_len);
    assert(out_buf_len == write_len);

    /* Increment transmit counters */
    fcs_global_counters.ioboard_packet_tx[0]++;
    fcs_global_counters.ioboard_packet_tx[1]++;
}

/*
Read, deserialize and validate a full I/O board packet from `dev`. Since we
get two of these every tick there's no point doing this incrementally; we just
need to make sure we can deal with partial/corrupted packets.
*/
bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, size_t board_id,
struct fcs_log_t *out_measurements) {
    assert(out_measurements);

    /* Read the latest I/O board packet from the UART streams */
    uint8_t buf[256];
    struct fcs_log_t plog;
    size_t nbytes, i, packet_start = 0, packet_end = 0;
    enum {
        UNKNOWN,
        GOT_ZERO,
        IN_PACKET,
        ENDED_PACKET
    } state = UNKNOWN;
    bool result = false;

    nbytes = fcs_stream_read(dev, buf, 255u);
    while (!result && nbytes) {
        state = UNKNOWN;
        packet_start = packet_end = 0;
        for (i = 0; i < nbytes && state != ENDED_PACKET; i++) {
            switch (state) {
                case UNKNOWN:
                    if (buf[i] == 0) {
                        state = GOT_ZERO;
                    }
                    break;
                case GOT_ZERO:
                    if (buf[i] != 0) {
                        state = IN_PACKET;
                        packet_start = i - 1u;
                    }
                    break;
                case IN_PACKET:
                    if (buf[i] == 0) {
                        state = ENDED_PACKET;
                        packet_end = i;
                    }
                    break;
                case ENDED_PACKET:
                    assert(false);
                    break;
            }
        }

        if (state == ENDED_PACKET) {
            result = fcs_log_deserialize(&plog, &buf[packet_start],
                                         packet_end - packet_start);
            if (result) {
                fcs_log_set_parameter_device_id(&plog, (uint8_t)board_id);
                fcs_log_merge(out_measurements, &plog);
            }

            /* Consume all bytes until the end of the packet */
            fcs_stream_consume(dev, packet_end + 1u);
        } else if (state == IN_PACKET) {
            /*
            Consume bytes until the start of the packet, so we can parse the
            full packet next time
            */
            if (packet_start > 0) {
                fcs_stream_consume(dev, packet_start);
            }
        } else if (state == GOT_ZERO) {
            /* Consume bytes up to the current 0 */
            fcs_stream_consume(dev, nbytes - 1u);
        } else {
            /*
            Something has gone badly wrong. Consume the whole buffer and hope
            we get actual data next time.
            */
            fcs_stream_consume(dev, nbytes);
        }

        if (!result) {
            nbytes = fcs_stream_read(dev, buf, 255u);
        }
    }

    return result;
}

static void _update_pitot_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, size_t i) {
    struct fcs_calibration_t *calibration;
    struct fcs_parameter_t parameter;
    double pitot_value;

    if (!fcs_parameter_find_by_type_and_device(
            plog, FCS_PARAMETER_PITOT, (uint8_t)i, &parameter)) {
        return;
    }

    /* Get the parameter value */
    fcs_parameter_get_values_d(&parameter, &pitot_value, 1u);

    /* Find the calibration parameters */
    calibration = fcs_parameter_get_calibration(cmap, &parameter);

    /*
    Update bias based on current sensor value, assuming the true
    reading should be 0. This is just a weighted moving average, with
    convergence taking a few seconds.
    */
    calibration->params[0] +=
        0.001 * ((pitot_value * calibration->scale_factor) -
                 calibration->params[0]);
}

static void _update_barometer_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, double reference_pressure, size_t i) {
    struct fcs_calibration_t *calibration;
    struct fcs_parameter_t parameter;
    double barometer_value;

    if (!fcs_parameter_find_by_type_and_device(
            plog, FCS_PARAMETER_PRESSURE_TEMP, (uint8_t)i, &parameter)) {
        return;
    }

    /* Get the parameter value */
    fcs_parameter_get_values_d(&parameter, &barometer_value, 1u);

    /* Find the calibration parameters */
    calibration = fcs_parameter_get_calibration(cmap, &parameter);

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
        0.001 * ((barometer_value * calibration->scale_factor) -
                 reference_pressure - calibration->params[0]);
}

static void _update_magnetometer_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, TRICAL_instance_t *instance,
const double *attitude, double *last_update_attitude, const double *wmm_field,
double wmm_field_norm, size_t i) {
    /*
    Handle magnetometer calibration update based on new readings -- we do this
    regardless of calibration mode
    */
    struct fcs_calibration_t *calibration;
    struct fcs_parameter_t parameter;
    double mag_value[3], expected_field[3], scale_factor, delta_angle;
    float mag_value_f[3], expected_field_f[3];

    if (!fcs_parameter_find_by_type_and_device(
            plog, FCS_PARAMETER_MAGNETOMETER_XYZ, (uint8_t)i, &parameter)) {
        return;
    }

    /*
    If the current attitude is too close to the attitude at which this
    TRICAL instance was last updated, skip calibration this time
    */
    delta_angle = quaternion_quaternion_angle_d(attitude,
                                                last_update_attitude);
    if (delta_angle < 3.0 * M_PI / 180.0) {
        return;
    }

    /*
    Rotate the WMM field by the current attitude to get the expected field
    direction for these readings
    */
    quaternion_vector3_multiply_d(expected_field, attitude, wmm_field);
    vector_f_from_d(expected_field_f, expected_field, 3u);

    /* Get the parameter value */
    fcs_parameter_get_values_d(&parameter, mag_value, 3u);

    /* Find the calibration parameters */
    calibration = fcs_parameter_get_calibration(cmap, &parameter);

    /*
    Update TRICAL instance parameters with the latest results. Scale
    the magnetometer reading such that the expected magnitude is the
    unit vector, by dividing by the current WMM field strength in
    Gauss.

    Copy the current sensor calibration to the TRICAL instance state
    so that any external changes to the calibration are captured.
    */
    vector_copy_f(instance->state, calibration->params, 9u);

    scale_factor = calibration->scale_factor / wmm_field_norm;
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
    vector_copy_d(last_update_attitude, attitude, 4u);
}

static void _update_accelerometer_calibration(const struct fcs_log_t *plog,
struct fcs_calibration_map_t *cmap, TRICAL_instance_t *instance, size_t i) {
    struct fcs_calibration_t *calibration;
    struct fcs_parameter_t parameter;
    double accel_value[3];
    float accel_value_f[3], g_field[] = { 0.0, 0.0, -1.0 };

    if (!fcs_parameter_find_by_type_and_device(
            plog, FCS_PARAMETER_ACCELEROMETER_XYZ, (uint8_t)i, &parameter)) {
        return;
    }

    /* Get the accelerometer value */
    fcs_parameter_get_values_d(&parameter, accel_value, 3u);

    /* Find the calibration parameters */
    calibration = fcs_parameter_get_calibration(cmap, &parameter);

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

static void _apply_accelerometer_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap) {
    double v[4], variance[3], offset[4], err;
    struct fcs_parameter_t param;

    if (!fcs_log_get_calibrated_value(plog, cmap,
                                      FCS_PARAMETER_ACCELEROMETER_XYZ, v,
                                      &err, offset, 1.0)) {
        return;
    }

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, 3u);
    fcs_parameter_set_type(&param,
                           FCS_PARAMETER_HAL_ACCELEROMETER_OFFSET_XYZ);
    fcs_parameter_set_device(&param, 0);

    param.data.f32[X] = (float)offset[X];
    param.data.f32[Y] = (float)offset[Y];
    param.data.f32[Z] = (float)offset[Z];

    fcs_log_add_parameter(hlog, &param);

    /* Accelerometer output is in g, convert to m/s^2 */
    vector3_scale_d(v, G_ACCEL);

    /*
    Add a relative error term to the covariance to account for scale
    factor error when the accelerometer reading differs from the bias
    calibration point (0, 0, -1).

    This isn't the right way to do it as the UKF assumes zero-mean error,
    but in practice it's OK.
    */
    vector3_scale_d(v, 0.1);
    variance[X] = err + absval(v[X]);
    variance[X] *= variance[X];

    variance[Y] = err + absval(v[Y]);
    variance[Y] *= variance[Y];

    variance[Z] = err + absval(v[Z] + G_ACCEL * 0.1);
    variance[Z] *= variance[Z];

    _set_hal_sensor_value_f32(hlog, FCS_PARAMETER_HAL_ACCELEROMETER_XYZ, v,
                              variance, 3u);
}

static void _apply_gyroscope_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap) {
    double v[4], variance[3], err;

    if (!fcs_log_get_calibrated_value(plog, cmap, FCS_PARAMETER_GYROSCOPE_XYZ,
                                      v, &err, NULL, 1.0)) {
        return;
    }

    /*
    Add a relative error term, as above. Remove this if gyro scale factor
    error is included in the process model.
    */
    vector3_scale_d(v, 0.03);
    variance[X] = err + absval(v[X]);
    variance[X] *= variance[X];

    variance[Y] = err + absval(v[Y]);
    variance[Y] *= variance[Y];

    variance[Z] = err + absval(v[Z]);
    variance[Z] *= variance[Z];

    _set_hal_sensor_value_f32(hlog, FCS_PARAMETER_HAL_GYROSCOPE_XYZ, v,
                              variance, 3u);
}

static void _apply_magnetometer_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap,
double wmm_field_norm) {
    double v[4], variance[3], err;
    double field_norm_inv = 1.0 / wmm_field_norm;

    /*
    We need to pre-scale the sensor reading by the current WMM field magnitude
    to work with the calibration params.
    */
    if (!fcs_log_get_calibrated_value(plog, cmap,
                                      FCS_PARAMETER_MAGNETOMETER_XYZ, v, &err,
                                      NULL, field_norm_inv)) {
        return;
    }

    /*
    The calibration scales the magnetometer value to unity expectation.
    Scale error by the same amount, so the units of error are Gauss.
    */
    err *= field_norm_inv;
    vector_set_d(variance, err * err, 3);

    _set_hal_sensor_value_f32(hlog, FCS_PARAMETER_HAL_MAGNETOMETER_XYZ, v,
                              variance, 3u);
}

static void _apply_pitot_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap,
double static_pressure, double static_temp) {
    double v[4], err, tas;

    if (!fcs_log_get_calibrated_value(plog, cmap, FCS_PARAMETER_PITOT, v,
                                      &err, NULL, 1.0)) {
        return;
    }

    tas = airspeed_from_pressure_temp(static_pressure, v[0], static_temp);

    /* Allow for 2% scale factor error */
    err += absval(v[0]) * 0.02;
    err *= err;

    _set_hal_sensor_value_f32(hlog, FCS_PARAMETER_HAL_AIRSPEED, &tas, &err,
                              1u);
}

static void _apply_barometer_calibration(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap,
double reference_pressure, double static_temp, double reference_alt) {
    double v[4], err, alt;

    if (!fcs_log_get_calibrated_value(plog, cmap, FCS_PARAMETER_PRESSURE_TEMP,
                                      v, &err, NULL, 1.0)) {
        return;
    }

    /*
    Calculate pressure differential between measured pressure and GCS
    pressure, then convert that to metres by dividing by 0.12.

    Add the GCS altitude above the ellipsoid to the result, so the final
    value we pass to the UKF is our altitude above ellipsoid referenced
    to current GCS pressure.
    */
    alt = altitude_diff_from_pressure_diff(reference_pressure, v[0],
                                           static_temp) + reference_alt;

    /* Allow for 3% scale factor error in altitude */
    err += absval(alt) * 0.03;
    err *= err;

    _set_hal_sensor_value_f32(hlog, FCS_PARAMETER_HAL_PRESSURE_ALTITUDE, &alt,
                              &err, 1u);
}

static void _set_gps_position(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap) {
    double v[4], err;
    struct fcs_parameter_t param;

    if (!fcs_log_get_calibrated_value(plog, cmap,
                                      FCS_PARAMETER_GPS_POSITION_LLA, v, &err,
                                      NULL, 1.0)) {
        return;
    }

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 64u, 2u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_HAL_POSITION_LAT_LON);
    fcs_parameter_set_device(&param, 0);

    param.data.f64[0] = v[0];
    param.data.f64[1] = v[1];

    fcs_log_add_parameter(hlog, &param);

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, 3u);
    fcs_parameter_set_type(&param,
                           FCS_PARAMETER_HAL_POSITION_LAT_LON_VARIANCE);
    fcs_parameter_set_device(&param, 0);

    param.data.f32[0] = (float)(err * err);
    param.data.f32[1] = (float)(err * err);

    fcs_log_add_parameter(hlog, &param);

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_HAL_POSITION_ALT);
    fcs_parameter_set_device(&param, 0);

    param.data.f32[0] = (float)v[2];

    fcs_log_add_parameter(hlog, &param);

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_HAL_POSITION_ALT_VARIANCE);
    fcs_parameter_set_device(&param, 0);

    param.data.f32[0] = 225.0f;

    fcs_log_add_parameter(hlog, &param);
}

static void _set_gps_velocity(const struct fcs_log_t *plog,
struct fcs_log_t *hlog, struct fcs_calibration_map_t *cmap) {
    double v[4], variance[3], err;

    if (!fcs_log_get_calibrated_value(plog, cmap,
                                      FCS_PARAMETER_GPS_VELOCITY_NED, v, &err,
                                      NULL, 1.0)) {
        return;
    }

    variance[X] = variance[Y] = err * err;
    variance[Z] = 25.0;

    _set_hal_sensor_value_f32(hlog, FCS_PARAMETER_HAL_VELOCITY_NED, v,
                              variance, 3u);
}

static void _set_hal_sensor_value_f32(struct fcs_log_t *plog,
enum fcs_parameter_type_t param_type, const double *restrict value,
const double *restrict variance, size_t n) {
    assert(plog);
    assert(value);

    size_t i;
    struct fcs_parameter_t param;

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, n);
    fcs_parameter_set_type(&param, param_type);
    fcs_parameter_set_device(&param, 0);

    for (i = 0; i < n; i++) {
        param.data.f32[i] = (float)value[i];
    }

    fcs_log_add_parameter(plog, &param);

    if (variance) {
        fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, n);
        fcs_parameter_set_type(&param, param_type + 1u);
        fcs_parameter_set_device(&param, 0);

        for (i = 0; i < n; i++) {
            param.data.f32[i] = (float)variance[i];
        }

        fcs_log_add_parameter(plog, &param);
    }
}
