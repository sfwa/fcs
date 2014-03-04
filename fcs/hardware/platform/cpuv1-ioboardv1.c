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

#include "../../config/config.h"
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

#if !defined(FCS_COMPILE_BOARD_HITL) && !defined(FCS_COMPILE_BOARD_LOG)

struct sensor_packet_t {
    /* Base fields */
    uint8_t crc;
    uint16_t tick;
    uint8_t sensor_update_flags;
    uint8_t cpu_load;
    uint16_t pwm_in; /* PWM input value; channel is tick % 4 */

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

#define UPDATED_ACCEL 0x01u
#define UPDATED_GYRO 0x02u
#define UPDATED_BAROMETER 0x04u
#define UPDATED_MAG 0x08u
#define UPDATED_GPS_POS 0x10u
#define UPDATED_GPS_INFO 0x20u
#define UPDATED_ADC_GPIO 0x40u

#define ACCEL_SENSITIVITY 4096.0f /* LSB/g @ ±8g FS */
#define GYRO_SENSITIVITY 65.5f /* LSB/(deg/s) @ 500deg/s FS */
#define MAG_SENSITIVITY 1090.0f /* LSB/G @ ±2G FS */

struct control_packet_t {
    uint8_t crc;
    uint8_t tick;
    uint8_t msg_type;
    uint8_t gpout;
    uint16_t pwm[4];
} __attribute__ ((packed));

enum msg_type_t {
    MSG_TYPE_NONE = 0,
    MSG_TYPE_CONTROL = 1,
    MSG_TYPE_FIRMWARE = 2,
    MSG_TYPE_CMD = 3
};

#define FCS_IOBOARD_RESET_TIMEOUT 50
#define FCS_IOBOARD_PACKET_TIMEOUT 5

static uint16_t pwm_state[2][FCS_CONTROL_CHANNELS];

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
        .error = 3.1623e-6f,
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
    }
}

void fcs_board_tick(void) {
    static int16_t ioboard_timeout[2];

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
    const struct fcs_control_channel_t *restrict control;
    float proportional_pos;

    #pragma MUST_ITERATE(FCS_CONTROL_CHANNELS, FCS_CONTROL_CHANNELS)
    for (i = 0; i < FCS_CONTROL_CHANNELS; i++) {
        control = &fcs_global_control_state.controls[i];
        control_log.data.u16[i] = (uint16_t)(control->setpoint * UINT16_MAX);
    }

    /* Log to sensor ID 1, because sensor ID 0 is the RC PWM input. */
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
        control_log.data.u16,
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
    pwm_state[board_id][(packet.gpin_state & 0xF0u) >> 4u] = swap_uint16(packet.pwm_in);
    if (board_id == 0) {
    	for (i = 0; i < FCS_CONTROL_CHANNELS; i++) {
    	    measurement.data.u16[i] = pwm_state[board_id][i];
    	}
    	fcs_measurement_set_header(&measurement, 16u, 4u);
    	fcs_measurement_set_sensor(&measurement, 0,
    	                           FCS_MEASUREMENT_TYPE_CONTROL_POS);
    	fcs_measurement_log_add(out_measurements, &measurement);
    } else if (board_id == 1u) {
        /* Set payload presence based on GPIN 0 (high == present). */
        if (packet.gpin_state & 0x1u) {
            fcs_global_ahrs_state.payload_present = false;
        } else {
            fcs_global_ahrs_state.payload_present = true;
        }

        /*
        FIXME: this is a temporary addition to enable payload release via
        RC PWM control. Should be removed once we have waypoint-based support
        for payload release.
        */

        /* Trigger payload release after 25 consecutive pulses > 1.5ms. */
        static uint16_t payload_release_ticks;
        if (pwm_state[board_id][2] > 32767) {
            payload_release_ticks++;
        } else {
            payload_release_ticks = 0;
        }

        if (payload_release_ticks > 25u && payload_release_ticks < 525u) {
            /* Re-trigger to keep magnet going for up to half a second. */
            fcs_global_control_state.gpio_state =
                payload_release_ticks & 0x1u;
        } else {
            fcs_global_control_state.gpio_state = 0;
        }
    }

    fcs_global_counters.ioboard_packet_rx[dev]++;
    return true;

invalid:
    fcs_global_counters.ioboard_packet_rx_err[dev]++;
    return false;
}

/*
Serialize a control packet containing `control_values` into `buf`.
*/
size_t _fcs_format_control_packet(uint8_t *buf, uint8_t tick,
const uint16_t *restrict control_values, uint8_t gpout) {
    assert(buf);
    assert(control_values);

    struct control_packet_t packet;

    packet.tick = tick;
    packet.msg_type = MSG_TYPE_CONTROL;
    packet.gpout = gpout;

    uint8_t i;
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4u; i++) {
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
    assert(result.out_len > 0 && (size_t)result.out_len < SIZE_MAX - 2u);
    return (size_t)(result.out_len + 2u);
}

#endif
