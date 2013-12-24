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
#include <assert.h>
#include <math.h>

#include "../config/config.h"
#include "../util/3dmath.h"
#include "../util/util.h"
#include "../drivers/stream.h"
#include "../drivers/peripheral.h"
#include "comms.h"
#include "../TRICAL/TRICAL.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "../stats/stats.h"

struct fcs_rfd900_status_packet_t {
    uint8_t type; /* always 0x11 */
    uint8_t length; /* always 12 */
    uint8_t crc; /* always 0 */
    uint8_t rssi;
    uint8_t remote_rssi;
    uint8_t tx_buffer_length;
    uint8_t noise;
    uint8_t remote_noise;
    uint16_t rx_errors;
    uint16_t rx_errors_fixed;
};

size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf);
void _fcs_comms_generate_status_packet(struct fcs_packet_status_t *out_status,
const struct fcs_ahrs_state_t *ahrs_state);
void _fcs_comms_generate_state_packet(struct fcs_packet_state_t *out_state,
const struct fcs_ahrs_state_t *ahrs_state);

void fcs_comms_init(void) {
    /* Open the CPU comms stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT0, 921600u) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT0) == FCS_STREAM_OK);

    /* Open the RFD900 comms stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT1, 57600u) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT1) == FCS_STREAM_OK);
}

void fcs_comms_tick(void) {
    static uint32_t tick;

    uint8_t comms_buf[256];
    size_t comms_buf_len, write_len;

    /* Generate a state packet for delivery to the CPU and RFD900a */
    struct fcs_packet_state_t state;
    _fcs_comms_generate_state_packet(&state, &fcs_global_ahrs_state);
    comms_buf_len = fcs_comms_serialize_state(comms_buf, &state);
    assert(comms_buf_len && comms_buf_len < 256u);

    /* Send a state update packet to the CPU every 20ms (50Hz) */
    if (tick % 20u == 0) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT0, comms_buf,
                                     comms_buf_len);
        fcs_global_counters.cpu_packet_tx++;
        /* We should definitely have enough room in the write buffer */
        assert(comms_buf_len == write_len);
    }

    /* Send a state update packet to the RFD900a every 1s */
    if (tick % 1000u == 0) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT1, comms_buf,
                                     comms_buf_len);
        assert(comms_buf_len == write_len);
    }

    /* Generate a status packet */
    struct fcs_packet_status_t status;
    _fcs_comms_generate_status_packet(&status, &fcs_global_ahrs_state);
    comms_buf_len = fcs_comms_serialize_status(comms_buf, &status);
    assert(comms_buf_len && comms_buf_len < 256u);

    /*
    Send a status update packet to the CPU every 100ms (10Hz), but long enough
    after the state update packet that the writes won't collide
    */
    if (tick % 100u == 12u) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT0, comms_buf,
                                     comms_buf_len);
        fcs_global_counters.cpu_packet_tx++;
        assert(comms_buf_len == write_len);
    }

    /*
    Send a status update packet to the RFD900a every 1s, 500ms after the state
    packet was sent
    */
    if (tick % 1000u == 500u) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT1, comms_buf,
                                     comms_buf_len);
        assert(comms_buf_len == write_len);
    }

    /*
    Write the log values to internal stream 1 -- the CPLD will route this to
    the CPU UART
    */
    comms_buf_len = fcs_measurement_log_serialize(
        comms_buf, sizeof(comms_buf), &fcs_global_ahrs_state.measurements);
    fcs_stream_write(FCS_STREAM_UART_INT1, comms_buf, comms_buf_len);

    /* Check for packets */
    comms_buf_len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, comms_buf);
    assert(comms_buf_len < 256u);

    if (comms_buf_len && comms_buf[0] == 0) {
        /* Initial byte is NUL, so this is an RFD900 status packet */
        struct fcs_rfd900_status_packet_t packet;
        struct fcs_cobsr_decode_result result;

        result = fcs_cobsr_decode((uint8_t*)&packet, sizeof(packet),
                                  &comms_buf[1], comms_buf_len - 2u);

        /* If decode is successful, update the counters */
        if (result.status == FCS_COBSR_DECODE_OK &&
                result.out_len == sizeof(packet)) {
            fcs_global_peripheral_state.telemetry_rssi = packet.rssi;
            fcs_global_peripheral_state.telemetry_noise = packet.noise;

            /*
            FIXME: is errors a delta, or a count wrapping to 16-bit? Also,
            check endianness.
            */
            fcs_global_peripheral_state.telemetry_errors += packet.rx_errors;
            fcs_global_peripheral_state.telemetry_errors_corrected +=
                packet.rx_errors_fixed;
        }
    } else if (comms_buf_len && comms_buf[0] == '$') {
        /* Initial byte is '$', so this is a control packet */
        enum fcs_deserialization_result_t result;

        /* TODO: do something with incoming packets */
        struct fcs_packet_state_t comms_state_in;
        bool comms_state_valid;
        struct fcs_packet_config_t comms_config_in;
        bool comms_config_valid;
        struct fcs_packet_gcs_t comms_gcs_in;
        bool comms_gcs_valid;
        struct fcs_packet_waypoint_t comms_waypoint_in;
        bool comms_waypoint_valid;

        switch (comms_buf[6]) {
            case 'S':
                result = fcs_comms_deserialize_state(
                    &comms_state_in, comms_buf, comms_buf_len);
                comms_state_valid = (result == FCS_DESERIALIZATION_OK);
                fcs_global_counters.cpu_packet_rx++;
                break;
            case 'P':
                result = fcs_comms_deserialize_waypoint(
                    &comms_waypoint_in, comms_buf, comms_buf_len);
                comms_waypoint_valid = (result == FCS_DESERIALIZATION_OK);
                fcs_global_counters.cpu_packet_rx++;
                break;
            case 'G':
                result = fcs_comms_deserialize_gcs(
                    &comms_gcs_in, comms_buf, comms_buf_len);
                comms_gcs_valid = (result == FCS_DESERIALIZATION_OK);
                fcs_global_counters.cpu_packet_rx++;
                break;
            case 'C':
                result = fcs_comms_deserialize_config(
                    &comms_config_in, comms_buf, comms_buf_len);
                comms_config_valid = (result == FCS_DESERIALIZATION_OK);
                fcs_global_counters.cpu_packet_rx++;
                break;
            default:
                fcs_global_counters.cpu_packet_rx_err++;
                break;
        }
    }

    /*
    Increment tick, but wrap to 0 at a (decimal) round number in order to keep
    the output packet rate steady.
    */
    tick++;
    if (tick == 4000000000u) {
        tick = 0;
    }
}

/*
Read a full message from `dev`, starting with $ and ending with \n. Neither
of those characters can appear in the message itself, so we don't need to do
any message parsing at this level.

The complication is that the RFD900 can inject NUL-terminated packets
containing link status information. That means we need to check for $ and \n
as well as NUL and NUL.
*/
size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf) {
    assert(buf);

    uint8_t i = 0, ch;
    uint32_t nbytes;

    nbytes = fcs_stream_bytes_available(dev);

    /*
    If the initial byte is not $ or NUL, we haven't synchronised with the
    start of a message -- try to do that now by skipping until we see a \n
    (which should be the end of \r\n), or a NUL (which would be the end of an
    RFD900 status packet).

    Give up after 4 tries.
    */
    ch = fcs_stream_peek(dev);
    while (i < 4u && (ch != '$' &&  ch != 0) &&
           nbytes >= FCS_COMMS_MIN_PACKET_SIZE) {
        /* Try to skip to the end of the next control packet */
        nbytes = fcs_stream_skip_until_after(dev, (uint8_t)'\n');
        if (!nbytes) {
            /*
            That didn't work, so try to skip to the end of the next RFD900
            status packet
            */
            fcs_stream_skip_until_after(dev, 0);
        }
        nbytes = fcs_stream_bytes_available(dev);
        ch = fcs_stream_peek(dev);
        i++;
    }

    /*
    Give up if there aren't enough bytes remaining, or we're not currently at
    a '$'/NUL character
    */
    if (nbytes < FCS_COMMS_MIN_PACKET_SIZE || i == 4) {
        return false;
    }

    /*
    Read until after the next (terminating) '\n' or NUL, depending on whether
    the packet start character was a '$' or NUL
    */
    nbytes = fcs_stream_read_until_after(
        dev, (uint8_t)ch == '$' ? (uint8_t)'\n' : 0, buf, 255u);
    if (nbytes >= FCS_COMMS_MIN_PACKET_SIZE) {
        return nbytes;
    } else {
        return 0;
    }
}

void _fcs_comms_generate_status_packet(struct fcs_packet_status_t *out_status,
const struct fcs_ahrs_state_t *ahrs_state) {
    assert(out_status);

    out_status->solution_time = ahrs_state->solution_time;
    out_status->solution_time &= 0x3FFFFFFFu;

    memset(out_status->flags, '-', sizeof(out_status->flags));

    uint8_t i;
    for (i = 0; i < 2; i++) {
        out_status->ioboard_resets[i] =
            fcs_global_counters.ioboard_resets[i] & 0x0FFFFFFFu;
        out_status->trical_resets[i] =
            fcs_global_counters.trical_resets[i] & 0x0FFFFFFFu;
        out_status->main_loop_cycle_max[i] =
            fcs_global_counters.main_loop_cycle_max[i] & 0x0FFFFFFFu;
    }
    out_status->ukf_resets = fcs_global_counters.ukf_resets;

    out_status->cpu_packet_rx =
        fcs_global_counters.cpu_packet_rx & 0x0FFFFFFFu;
    out_status->cpu_packet_rx_err =
        fcs_global_counters.cpu_packet_rx_err & 0x0FFFFFFFu;

    /*
    Work out the maximum number of SVs tracked by any of the GPS units
    connected
    */
    uint8_t gps_num_svs = 0;
    struct fcs_measurement_t gps_info_measurement;
    double value[4];
    bool result;

    for (i = 0; i < 4u; i++) {
        result = fcs_measurement_log_find(
            &fcs_global_ahrs_state.measurements,
            FCS_MEASUREMENT_TYPE_GPS_INFO, i, &gps_info_measurement);
        if (result) {
            fcs_measurement_get_values(&gps_info_measurement, value);
            if (0.0 <= value[0] && value[0] < 16.0) {
                gps_num_svs = max(gps_num_svs, (uint8_t)value[0]);
            }
        }
    }
    assert(gps_num_svs <= 0xFu);

    out_status->gps_num_svs = gps_num_svs;

    out_status->telemetry_signal_db =
        fcs_global_peripheral_state.telemetry_rssi;
    out_status->telemetry_noise_db =
        fcs_global_peripheral_state.telemetry_noise;
    out_status->telemetry_packet_rx =
        fcs_global_peripheral_state.telemetry_packets;
    out_status->telemetry_packet_rx_err =
        fcs_global_peripheral_state.telemetry_errors;
}

void _fcs_comms_generate_state_packet(struct fcs_packet_state_t *out_state,
const struct fcs_ahrs_state_t *ahrs_state) {
    assert(ahrs_state);
    assert(out_state);

    /* Wrap-around at 30 bits (1073741.823s) */
    out_state->solution_time = ahrs_state->solution_time;
    out_state->solution_time &= 0x3FFFFFFFu;

    memset(out_state->next_waypoint_id, '-',
           sizeof(out_state->next_waypoint_id));

    /* Convert lat/lon to degrees */
    out_state->lat = ahrs_state->lat * (180.0/M_PI);
    out_state->lon = ahrs_state->lon * (180.0/M_PI);
    out_state->alt = ahrs_state->alt;

    memcpy(out_state->velocity, ahrs_state->velocity, 3u * sizeof(double));
    memcpy(out_state->wind_velocity, ahrs_state->wind_velocity,
           3u * sizeof(double));

    /* Convert angular velocity to degrees/s */
    out_state->angular_velocity[0] =
        ahrs_state->angular_velocity[0] * (180.0/M_PI);
    out_state->angular_velocity[1] =
        ahrs_state->angular_velocity[1] * (180.0/M_PI);
    out_state->angular_velocity[2] =
        ahrs_state->angular_velocity[2] * (180.0/M_PI);

    /* Convert state.attitude to yaw/pitch/roll, in degrees */
    double yaw, pitch, roll;

    #define q ahrs_state->attitude
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

    out_state->yaw = yaw;
    out_state->pitch = pitch;
    out_state->roll = roll;

    /*
    Work out 95th percentile confidence intervals for each of the output
    values, based on the current state covariance matrix and the assumption
    that error will follow a Gaussian distribution
    */

    /*
    Ignore changes in lon with varying lat -- small angles and all that.

    Formula for m per degree latitude is approx (2 * pi / 360) * r
    */
    out_state->lat_lon_uncertainty = 1.96 *
                                     sqrt(max(ahrs_state->lat_covariance,
                                              ahrs_state->lon_covariance)) *
                                     6378000.0;
    out_state->alt_uncertainty = 1.96 * sqrt(ahrs_state->alt_covariance);

    /* Rotation around +X, +Y and +Z -- roll, pitch, yaw */
    out_state->roll_uncertainty =
        1.96 * sqrt(ahrs_state->attitude_covariance[0]) * (180.0/M_PI);
    out_state->pitch_uncertainty =
        1.96 * sqrt(ahrs_state->attitude_covariance[1]) * (180.0/M_PI);
    out_state->yaw_uncertainty =
        1.96 * sqrt(ahrs_state->attitude_covariance[2]) * (180.0/M_PI);

    uint8_t i;
    for (i = 0; i < 3; i++) {
        out_state->velocity_uncertainty[i] =
            1.96 * sqrt(ahrs_state->velocity_covariance[i]);
        out_state->wind_velocity_uncertainty[i] =
            1.96 * sqrt(ahrs_state->wind_velocity_covariance[i]);
        out_state->angular_velocity_uncertainty[i] =
            1.96 *
            sqrt(ahrs_state->angular_velocity_covariance[i]) * (180.0/M_PI);
    }

    memset(out_state->flags, '-', sizeof(out_state->flags));

    /* Check that the state is valid */
    enum fcs_validation_result_t valid;
    valid = fcs_comms_validate_state(out_state);

    if (valid != FCS_VALIDATION_OK) {
        out_state->mode_indicator = 'N';
    } else {
        out_state->mode_indicator = 'A';
    }
}
