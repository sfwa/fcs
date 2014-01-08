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
#include "../../drivers/stream.h"
#include "../../stats/stats.h"
#include "../../TRICAL/TRICAL.h"
#include "../../ahrs/measurement.h"
#include "../../ahrs/ahrs.h"

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

enum msg_type_t {
    MSG_TYPE_NONE = 0,
    MSG_TYPE_CONTROL = 1,
    MSG_TYPE_FIRMWARE = 2,
    MSG_TYPE_CMD = 3
};

#define FCS_IOBOARD_RESET_TIMEOUT 50
#define FCS_IOBOARD_PACKET_TIMEOUT 5

/* Endian swap functions -- AVR32s are big-endian */
static inline uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | ((val >> 8) & 0xFF);
}

static inline int16_t swap_int16(int16_t val) {
    return (val << 8) | ((val >> 8) & 0xFF);
}

static inline int32_t swap_int32(int32_t val) {
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

/* Prototypes of internal functions */
bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements);
uint32_t _fcs_format_control_packet(uint8_t *buf, uint8_t tick,
const double *restrict control_values);

void fcs_board_init_platform(void) {
    /*
    Set I/O board serial baud rates. We don't actually open the streams here
    -- we wait for the I/O board comms to time out and reset the links to
    ensure we get a clean start.
    */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_INT0, 868056u) == FCS_STREAM_OK);
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_INT1, 868056u) == FCS_STREAM_OK);

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
        .scale_factor = 1.0 / ACCEL_SENSITIVITY * 32767.0f
    };
    struct fcs_calibration_t gyro_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_GYROSCOPE,
        .type = FCS_CALIBRATION_FLAGS_APPLY_ORIENTATION |
                FCS_CALIBRATION_BIAS_SCALE_3X3,
        .error = 0.0349, /* approx 2 degrees */
        .params = {
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        },
        .orientation = { 0.0f, 0.0f, 0.0f, 1.0f },
        .offset = { 0.0f, 0.0f, 0.0f },
        .scale_factor = (M_PI/180.0f) / GYRO_SENSITIVITY * 32767.0f
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
        .error = 10.0f,
        .params = { 0.2533f, -6.514f },
        .scale_factor = 1.0f
    };
    struct fcs_calibration_t barometer_calibration = {
        .header = sizeof(struct fcs_calibration_t) - 1u,
        .sensor = FCS_MEASUREMENT_TYPE_PRESSURE_TEMP,
        .type = FCS_CALIBRATION_BIAS_SCALE_1D,
        .error = 2.0f,
        /*
        1013.25 is a guess at the GCS pressure; 0.02 is the sensor scale
        factor.
        */
        .params = { FCS_STANDARD_PRESSURE, 1.0f },
        .scale_factor = 0.02f * 65535.0f
    };

    /*
    FIXME: Should update the calibration sensor ID for each of these so they
    match the calibration slot for I/O board 1.
    */
    uint8_t sensor_id_bits, i;
    for (i = 0; i < 2u; i++) {
        sensor_id_bits = (i << FCS_MEASUREMENT_SENSOR_ID_OFFSET);

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
                FCS_STREAM_UART_INT0 + i, i,
                &fcs_global_ahrs_state.measurements)) {
            ioboard_timeout[i] = FCS_IOBOARD_PACKET_TIMEOUT;
        }

        /*
        Handle I/O board comms errors. Log and clear any UART errors, but the
        I/O board itself should not reset in that circumstance.

        If the time since last I/O board tick/reset is too high, the stream
        should be reset.
        */
        if (fcs_stream_check_error(FCS_STREAM_UART_INT0 + i) ==
                FCS_STREAM_ERROR) {
            fcs_global_counters.ioboard_packet_rx_err[i]++;
        }

        /*
        If the board has timed out, start the reset for stream 0. Otherwise,
        just decrement the timeout counter.
        */
        if (ioboard_timeout[i] <= 0) {
            ioboard_timeout[i] = FCS_IOBOARD_RESET_TIMEOUT;
            fcs_global_counters.ioboard_resets[i]++;

            assert(fcs_stream_open(FCS_STREAM_UART_INT0 + i) ==
                   FCS_STREAM_OK);
        } else if (ioboard_timeout[i] > INT16_MIN) {
            ioboard_timeout[i]--;
        }
    }

    /*
    Write current control values to I/O boards -- the CPLD replicates the I/O
    board output stream so we only need to write to one.
    */

    /* TODO: Read control set values from NMPC */
    double control_set[4] = { 0.0, 0.0, 0.0, 0.0 };

    size_t control_len;
    uint8_t control_buf[16];
    control_len = _fcs_format_control_packet(
        control_buf,
        (uint8_t)(fcs_global_ahrs_state.solution_time & 0xFFu),
        control_set
    );
    assert(control_len < 16u);
    fcs_stream_write(FCS_STREAM_UART_INT0, control_buf, control_len);

    /* Increment transmit counters */
    fcs_global_counters.ioboard_packet_tx[0]++;
    fcs_global_counters.ioboard_packet_tx[1]++;
}

/*
Read, deserialize and validate a full I/O board packet from `dev`. Since we
get two of these every tick there's no point doing this incrementally; we just
need to make sure we can deal with partial/corrupted packets.
*/
bool _fcs_read_ioboard_packet(enum fcs_stream_device_t dev, uint8_t board_id,
struct fcs_measurement_log_t *out_measurements) {
    assert(out_measurements);

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
    while (i < 4u && fcs_stream_peek(dev) > 0 &&
           nbytes > sizeof(struct sensor_packet_t) + 1u) {
        fcs_stream_skip_until_after(dev, (uint8_t)0);
        nbytes = fcs_stream_bytes_available(dev);
        i++;
    }

    /* Give up if there aren't enough bytes remaining */
    if (nbytes < sizeof(struct sensor_packet_t) + 2u || i == 4u) {
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
    if (nbytes < sizeof(struct sensor_packet_t) ||
            nbytes > sizeof(struct sensor_packet_t) + 2u) {
        goto invalid;
    }

    /* Decode the message into the packet buffer */
    struct sensor_packet_t packet;
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
    checksum = fcs_crc8((uint8_t*)&packet.tick, result.out_len - 1u, 0x0);

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
        measurement.data.i16[1] = swap_int16(packet.barometer_temp);
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

    /* FIXME: TESTING */
    if (true) {
        fcs_measurement_set_header(&measurement, 32u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_GPS_POSITION);

        measurement.data.i32[0] = -378136000;
        measurement.data.i32[1] = 1449631000;
        measurement.data.i32[2] = 60000;
        fcs_measurement_log_add(out_measurements, &measurement);

        fcs_measurement_set_header(&measurement, 16u, 3u);
        fcs_measurement_set_sensor(&measurement, board_id,
                                   FCS_MEASUREMENT_TYPE_GPS_VELOCITY);

        measurement.data.i16[0] = 0;
        measurement.data.i16[1] = 0;
        measurement.data.i16[2] = 0;
        fcs_measurement_log_add(out_measurements, &measurement);
    }

    /* TODO: Log GPS info updates */

    fcs_global_counters.ioboard_packet_rx[dev]++;
    return true;

invalid:
    fcs_global_counters.ioboard_packet_rx_err[dev]++;
    return false;
}

/*
Serialize a control packet containing `control_values` into `buf`.
*/
uint32_t _fcs_format_control_packet(uint8_t *buf, uint8_t tick,
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
