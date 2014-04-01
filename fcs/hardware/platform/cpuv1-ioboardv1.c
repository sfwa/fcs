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
#include "../../ahrs/measurement.h"
#include "../../ahrs/ahrs.h"
#include "../../control/control.h"
#include "../../exports/exports.h"


#define ACCEL_SENSITIVITY 4096.0f /* LSB/g @ ±8g FS */
#define GYRO_SENSITIVITY 65.5f /* LSB/(deg/s) @ 500deg/s FS */
#define MAG_SENSITIVITY 1090.0f /* LSB/G @ ±2G FS */

#define FCS_IOBOARD_RESET_TIMEOUT 50
#define FCS_IOBOARD_PACKET_TIMEOUT 5

static uint16_t pwm_state[2][FCS_CONTROL_CHANNELS];
static float pwm_neutral[FCS_CONTROL_CHANNELS];

/* Prototypes of internal functions */
bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements);
bool _fcs_decode_packet(const uint8_t *buf, size_t nbytes,
enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements);
size_t _fcs_format_control_packet(uint8_t *buf, uint8_t tick,
const uint16_t *restrict control_values, uint8_t gpout);

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
    struct fcs_calibration_t *restrict map =
        fcs_global_ahrs_state.calibration.sensor_calibration;

    struct fcs_calibration_t accel_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_ACCELEROMETER,
        .type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
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
        .scale_factor = 1.0f / ACCEL_SENSITIVITY * 32767.0f
    };
    struct fcs_calibration_t gyro_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_GYROSCOPE,
        .type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
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
        .scale_factor = (float)(M_PI/180.0) / GYRO_SENSITIVITY * 32767.0f
    };
    struct fcs_calibration_t mag_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_MAGNETOMETER,
        .type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
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
        .scale_factor = 1.0f / MAG_SENSITIVITY * 2047.0f
    };
    struct fcs_calibration_t gps_position_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_GPS_POSITION,
        .type = FCS_CALIBRATION_NONE,
        .error = 3.1623e-7f,
        .scale_factor = 1.0f
    };
    struct fcs_calibration_t gps_velocity_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_GPS_VELOCITY,
        .type = FCS_CALIBRATION_BIAS_SCALE_3X3,
        .error = 3.0f,
        .params = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        },
        .scale_factor = 1e-3f * 32767.0f
    };
    struct fcs_calibration_t pitot_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_PITOT,
        .type = FCS_CALIBRATION_BIAS_SCALE_1D,
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
        .sensor = FCS_MEASUREMENT_TYPE_PRESSURE_TEMP,
        .type = FCS_CALIBRATION_BIAS_SCALE_1D,
        .error = 1.0f,
        /*
        0.02 is the sensor scale factor for conversion to mbar; multiply by
        100 for Pa.
        */
        .params = { 0.0, 1.0f },
        .scale_factor = 0.02f * 65535.0f * 100.0f
    };
    struct fcs_calibration_t iv_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_IV,
        .type = FCS_CALIBRATION_BIAS_SCALE_2D,
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
        .scale_factor = 10.0f
    };

    /*
    FIXME: Should update the calibration sensor ID for each of these so they
    match the calibration slot for I/O board 1.
    */
    uint8_t sensor_id_bits, i;
    for (i = 0; i < 2u; i++) {
        sensor_id_bits = (uint8_t)(i << FCS_MEASUREMENT_SENSOR_ID_OFFSET);

        memcpy(&map[accel_calibration.sensor | sensor_id_bits],
               &accel_calibration, sizeof(accel_calibration));
        memcpy(&map[gyro_calibration.sensor | sensor_id_bits],
               &gyro_calibration, sizeof(gyro_calibration));
        memcpy(&map[mag_calibration.sensor | sensor_id_bits],
               &mag_calibration, sizeof(mag_calibration));
        memcpy(&map[gps_position_calibration.sensor | sensor_id_bits],
               &gps_position_calibration, sizeof(gps_position_calibration));
        memcpy(&map[gps_velocity_calibration.sensor | sensor_id_bits],
               &gps_velocity_calibration, sizeof(gps_velocity_calibration));
        memcpy(&map[pitot_calibration.sensor | sensor_id_bits],
               &pitot_calibration, sizeof(pitot_calibration));
        memcpy(&map[barometer_calibration.sensor | sensor_id_bits],
               &barometer_calibration, sizeof(barometer_calibration));
        memcpy(&map[iv_calibration.sensor | sensor_id_bits],
                       &iv_calibration, sizeof(iv_calibration));
    }
}

void fcs_board_tick(void) {
    static int16_t ioboard_timeout[2];
    uint16_t pwm_out[4];

    uint8_t i;
    for (i = 0; i < 2; i++){
        if (_fcs_read_ioboard_packet(
                (enum fcs_stream_device_t)(FCS_STREAM_UART_INT0 + i), i,
                &fcs_global_ahrs_state.measurements)) {
            ioboard_timeout[i] = FCS_IOBOARD_PACKET_TIMEOUT;
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
        if (ioboard_timeout[i] <= 0) {
            ioboard_timeout[i] = FCS_IOBOARD_RESET_TIMEOUT;
            fcs_global_counters.ioboard_resets[i]++;

            enum fcs_stream_result_t result;
            result = fcs_stream_open(
                (enum fcs_stream_device_t)(FCS_STREAM_UART_INT0 + i));
            assert(result == FCS_STREAM_OK);
        } else if (ioboard_timeout[i] > INT16_MIN) {
            ioboard_timeout[i]--;
        }
    }

    /*
    Work out the nominal current control position, taking into account the
    control response time configured in control_rates. Log the result.

    We don't need to worry about mutexes for this -- all reads on these data
    types are atomic, so there's no risk of reading a corrupted value. It's
    possible that not all control values will be part of the same control
    output frame, but that's not going to make a difference to anything.
    */
    struct fcs_measurement_t control_log;
    struct fcs_control_output_t control;
    float val;

    fcs_exports_recv_control(&control);

    /* Throttle */
    pwm_out[0] = (uint16_t)(control.values[0] * UINT16_MAX);

    /* Left elevon */
    val = control.values[1] + (pwm_neutral[1] - 0.5f);
    if (val > 1.0f) {
        val = 1.0f;
    } else if (val < 0.0f) {
        val = 0.0f;
    }
    pwm_out[1] = (uint16_t)(val * UINT16_MAX);

    /* Right elevon */
    val = 1.0f - control.values[2] + (pwm_neutral[2] - 0.5f);
    if (val > 1.0f) {
        val = 1.0f;
    } else if (val < 0.0f) {
        val = 0.0f;
    }
    pwm_out[2] = (uint16_t)(val * UINT16_MAX);

    pwm_out[3] = 0;

    /* Log to sensor ID 1, because sensor ID 0 is the RC PWM input. */
    memcpy(&control_log.data, pwm_out, sizeof(pwm_out));
    fcs_measurement_set_header(&control_log, 16u, 4u);
    fcs_measurement_set_sensor(&control_log, 1u,
                               FCS_MEASUREMENT_TYPE_CONTROL_POS);
    fcs_measurement_log_add(&fcs_global_ahrs_state.measurements,
                            &control_log);

    /*
    Write current control values to I/O boards -- the CPLD replicates the I/O
    board output stream so we only need to write to one.
    */
    size_t control_len;
    uint8_t control_buf[16];
    control_len = _fcs_format_control_packet(
        control_buf,
        (uint8_t)(fcs_global_ahrs_state.solution_time & 0xFFu),
        pwm_out,
        fcs_global_control_state.gpio_state
    );
    assert(control_len < 16u);
    fcs_stream_write(FCS_STREAM_UART_INT0, control_buf, control_len);

    /* Increment transmit counters */
    fcs_global_counters.ioboard_packet_tx[0]++;
    fcs_global_counters.ioboard_packet_tx[1]++;

    /*
    TODO: check DSP GPIO for the FCS_CONTROL signal from the failsafe device;
    if that's enabled then we're under autonomous control, so
    fcs_global_control_state.mode should be set to FCS_CONTROL_MODE_AUTO.

    Otherwise, it should be set to FCS_CONTROL_MODE_MANUAL.
    */

    /*
    Write the log values to internal stream 1 -- the CPLD will route this to
    the CPU UART
    */

    comms_buf_len = fcs_measurement_log_serialize(
        comms_buf, sizeof(comms_buf), &fcs_global_ahrs_state.measurements);
    write_len = fcs_stream_write(FCS_STREAM_UART_INT1, comms_buf,
                                 comms_buf_len);
    assert(comms_buf_len == write_len);
}

/*
Read, deserialize and validate a full I/O board packet from `dev`. Since we
get two of these every tick there's no point doing this incrementally; we just
need to make sure we can deal with partial/corrupted packets.
*/
bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements) {
    assert(out_measurements);

    /* Read the latest I/O board packet from the UART streams */
    uint8_t buf[64];
    size_t nbytes, i, packet_start = 0, packet_end = 0;
    enum {
        UNKNOWN,
        GOT_ZERO,
        IN_PACKET,
        ENDED_PACKET
    } state = UNKNOWN;
    bool result = false;

    nbytes = fcs_stream_read(dev, buf, 64u);
    while (!result && nbytes >= sizeof(struct sensor_packet_t) + 2u) {
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
            result = _fcs_decode_packet(&buf[packet_start + 1u],
                                        packet_end - packet_start,
                                        dev, board_id, out_measurements);

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
            nbytes = fcs_stream_read(dev, buf, 64u);
        }
    }

    return result;
}

/* Decode an input packet from `buf` */
bool _fcs_decode_packet(const uint8_t *buf, size_t nbytes,
enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements) {
    uint8_t checksum;
    size_t i;
    struct sensor_packet_t packet;
    struct fcs_cobsr_decode_result result;

    /* Decode the message into the packet buffer */
    result = fcs_cobsr_decode((uint8_t*)&packet, sizeof(packet), buf,
                              nbytes - 1u);

    /* Confirm decode was successful */
    if (result.status != FCS_COBSR_DECODE_OK) {
        goto invalid;
    }

    /* Confirm packet size is what we expect */
    if (result.out_len != sizeof(packet)) {
        goto invalid;
    }

    /* Validate the packet checksum */
    checksum = fcs_crc8(
        (uint8_t*)&packet.tick, (size_t)result.out_len - 1u, 0x0);

    if (checksum != packet.crc) {
        goto invalid;
    }

    /* Copy sensor readings to the system measurement log */
    struct fcs_measurement_t measurement;

    if (packet.sensor_update_flags & UPDATED_ACCEL) {
        fcs_measurement_set_header(&measurement, 16u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_ACCELEROMETER);

        measurement.data.i16[0] = swap_int16(packet.accel.y);
        measurement.data.i16[1] = swap_int16(packet.accel.x);
        measurement.data.i16[2] = -swap_int16(packet.accel.z);
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    if (packet.sensor_update_flags & UPDATED_GYRO) {
        fcs_measurement_set_header(&measurement, 16u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_GYROSCOPE);

        measurement.data.i16[0] = swap_int16(packet.gyro.y);
        measurement.data.i16[1] = swap_int16(packet.gyro.x);
        measurement.data.i16[2] = -swap_int16(packet.gyro.z);
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    if (packet.sensor_update_flags & UPDATED_BAROMETER) {
        fcs_measurement_set_header(&measurement, 16u, 2u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_PRESSURE_TEMP);

        measurement.data.u16[0] = swap_uint16(packet.pressure);
        measurement.data.u16[1] = swap_uint16(packet.barometer_temp);
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    if (packet.sensor_update_flags & UPDATED_ADC_GPIO) {
        /* Update the pitot */
        fcs_measurement_set_header(&measurement, 16u, 1u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_PITOT);

        measurement.data.i16[0] = swap_int16(packet.pitot);
        fcs_measurement_log_add(out_measurements, &measurement);

        /* Update current/voltage */
        fcs_measurement_set_header(&measurement, 16u, 2u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_IV);

        measurement.data.i16[0] = swap_int16(packet.i);
        measurement.data.i16[1] = swap_int16(packet.v);
        fcs_measurement_log_add(out_measurements, &measurement);


        /* Update ultransonic rangefinder */
        fcs_measurement_set_header(&measurement, 16u, 1u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_RANGEFINDER);

        measurement.data.i16[0] = swap_int16(packet.range);
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    if (packet.sensor_update_flags & UPDATED_MAG) {
        fcs_measurement_set_header(&measurement, 12u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_MAGNETOMETER);

        measurement.data.i16[0] = swap_int16(packet.mag.x);
        measurement.data.i16[1] = -swap_int16(packet.mag.y);
        measurement.data.i16[2] = -swap_int16(packet.mag.z);
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    if (packet.sensor_update_flags & UPDATED_GPS_POS) {
        fcs_measurement_set_header(&measurement, 32u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_GPS_POSITION);

        measurement.data.i32[0] = swap_int32(packet.gps.position.lat);
        measurement.data.i32[1] = swap_int32(packet.gps.position.lng);
        measurement.data.i32[2] = swap_int32(packet.gps.position.alt);
        fcs_measurement_log_add(out_measurements, &measurement);

        fcs_measurement_set_header(&measurement, 16u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_GPS_VELOCITY);

        measurement.data.i16[0] = swap_int16(packet.gps.velocity.n);
        measurement.data.i16[1] = swap_int16(packet.gps.velocity.e);
        measurement.data.i16[2] = swap_int16(packet.gps.velocity.d);
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    if (packet.sensor_update_flags & UPDATED_GPS_INFO) {
        fcs_measurement_set_header(&measurement, 8u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_GPS_INFO);

        measurement.data.u8[0] =
            packet.gps_info.fix_mode_num_satellites >> 4u;
        measurement.data.u8[1] =
            packet.gps_info.fix_mode_num_satellites & 0xFu;
        measurement.data.u8[2] = packet.gps_info.pos_err;
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    /*
    Update the current PWM state for the appropriate channel, then save all
    PWM values in the measurement control log.
    */
    pwm_state[board_id][(packet.gpin_state & 0xF0u) >> 4u] =
        swap_uint16(packet.pwm_in);
	for (i = 0; i < FCS_CONTROL_CHANNELS; i++) {
	    measurement.data.u16[i] = pwm_state[board_id][i];

        /* Set neutral value during calibration */
        if (fcs_global_ahrs_state.mode == FCS_MODE_CALIBRATING) {
            pwm_neutral[i] += (float)pwm_state[board_id][i] *
                          (1.0f / 65535.0f);
            pwm_neutral[i] *= 0.5;
        }
	}
	fcs_measurement_set_header(&measurement, 16u, 4u);
	fcs_measurement_set_sensor(&measurement, 0,
	                           FCS_MEASUREMENT_TYPE_CONTROL_POS);
	fcs_measurement_log_add(out_measurements, &measurement);

    fcs_global_counters.ioboard_packet_rx[dev]++;
    return true;

invalid:
    fcs_global_counters.ioboard_packet_rx_err[dev]++;
    return false;
}


#endif
