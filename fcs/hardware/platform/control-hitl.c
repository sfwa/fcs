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

#ifdef FCS_COMPILE_BOARD_HITL

struct control_hitl_state_packet_t {
    /* Base fields */
    uint8_t crc;
    uint16_t tick;
    double lat, lon, alt;
    double velocity[3];
    double attitude[4];
    double angular_velocity[3];
    double wind_velocity[3];
} __attribute__ ((packed));

struct control_hitl_out_packet_t {
    uint8_t crc;
    uint16_t tick;
    uint16_t pwm[4];
    float objective_val;
    uint32_t cycles;
    double reference_lat;
    double reference_lon;
    float reference_alt;
    float reference_airspeed;
    float reference_yaw;
    float reference_pitch;
    float reference_roll;
} __attribute__ ((packed));

enum msg_type_t {
    MSG_TYPE_NONE = 0,
    MSG_TYPE_CONTROL = 1,
    MSG_TYPE_FIRMWARE = 2,
    MSG_TYPE_CMD = 3
};

#define FCS_HITL_PACKET_TIMEOUT 50
#define FCS_HITL_RESET_TIMEOUT 500


/* Prototypes of internal functions */
bool _fcs_read_control_hitl_state_packet(enum fcs_stream_device_t dev);
bool _fcs_decode_packet(const uint8_t *buf, size_t nbytes);
size_t _fcs_format_control_hitl_out_packet(uint8_t *buf, uint16_t tick,
const uint16_t *restrict control_values);

void fcs_board_init_platform(void) {
    /*
    Set I/O board serial baud rates. We don't actually open the streams here
    -- we wait for the I/O board comms to time out and reset the links to
    ensure we get a clean start.
    */
    enum fcs_stream_result_t result;
    result = fcs_stream_set_rate(FCS_STREAM_UART_EXT0, 921600u);
    assert(result == FCS_STREAM_OK);
}

void fcs_board_tick(void) {
    static int16_t comms_timeout;
    size_t i;

    /*
    Write one output packet per input packet to make sure things don't get
    out of hand.
    */
    if (_fcs_read_control_hitl_state_packet(FCS_STREAM_UART_EXT0)) {
        comms_timeout = FCS_HITL_PACKET_TIMEOUT;

        /* Work out the current control position */
        const struct fcs_control_channel_t *restrict control;
        float proportional_pos;
        uint16_t controls[FCS_CONTROL_CHANNELS];

        #pragma MUST_ITERATE(FCS_CONTROL_CHANNELS, FCS_CONTROL_CHANNELS)
        for (i = 0; i < FCS_CONTROL_CHANNELS; i++) {
            control = &fcs_global_control_state.controls[i];
            proportional_pos = (control->setpoint - control->min) /
                               (control->max - control->min);
            controls[i] = (uint16_t)(proportional_pos * UINT16_MAX);
        }

        /*
        Write current control values back to the HITL host
        */
        size_t control_len;
        uint8_t control_buf[16];
        control_len = _fcs_format_control_hitl_out_packet(
            control_buf,
            (uint16_t)(fcs_global_ahrs_state.solution_time & 0xFFFFu),
            controls
        );
        assert(control_len < 16u);
        fcs_stream_write(FCS_STREAM_UART_EXT0, control_buf, control_len);
    }

    /*
    If the stream has timed out, start the reset. Otherwise, just decrement
    the timeout counter.
    */
    if (comms_timeout <= 0) {
        comms_timeout = FCS_HITL_RESET_TIMEOUT;

        enum fcs_stream_result_t result;
        result = fcs_stream_open(FCS_STREAM_UART_EXT0);
        assert(result == FCS_STREAM_OK);
    } else if (comms_timeout > INT16_MIN) {
        comms_timeout--;
    }
}

bool _fcs_read_control_hitl_state_packet(enum fcs_stream_device_t dev) {
    /* Read the latest packet from the UART streams */
    uint8_t buf[192];
    size_t nbytes, i, packet_start = 0, packet_end = 0;
    enum {
        UNKNOWN,
        GOT_ZERO,
        IN_PACKET,
        ENDED_PACKET
    } state = UNKNOWN;
    bool result = false;

    nbytes = fcs_stream_read(dev, buf, 192u);
    while (!result &&
            nbytes >= sizeof(struct control_hitl_out_packet_t) + 2u) {
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
                                        packet_end - packet_start);

            /* Consume all bytes until the end of the packet */
            fcs_stream_consume(dev, packet_end + 1u);
        } else if (state == IN_PACKET && packet_start > 0) {
            /*
            Consume bytes until the start of the packet, so we can parse the
            full packet next time
            */
            fcs_stream_consume(dev, packet_start);
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
            nbytes = fcs_stream_read(dev, buf, 192u);
        }
    }

    return result;
}

/* Decode an input packet from `buf` */
bool _fcs_decode_packet(const uint8_t *buf, size_t nbytes) {
    uint8_t checksum;
    struct control_hitl_state_packet_t packet;
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

    /*
    Write the data in to the AHRS state structure directly, bypassing the UKF
    */
    fcs_global_ahrs_state.lat = packet.lat;
    fcs_global_ahrs_state.lon = packet.lon;
    fcs_global_ahrs_state.alt = packet.alt;
    fcs_global_ahrs_state.velocity[0] = packet.velocity[0];
    fcs_global_ahrs_state.velocity[1] = packet.velocity[1];
    fcs_global_ahrs_state.velocity[2] = packet.velocity[2];
    fcs_global_ahrs_state.attitude[0] = packet.attitude[0];
    fcs_global_ahrs_state.attitude[1] = packet.attitude[1];
    fcs_global_ahrs_state.attitude[2] = packet.attitude[2];
    fcs_global_ahrs_state.attitude[3] = packet.attitude[3];
    fcs_global_ahrs_state.angular_velocity[0] = packet.angular_velocity[0];
    fcs_global_ahrs_state.angular_velocity[1] = packet.angular_velocity[1];
    fcs_global_ahrs_state.angular_velocity[2] = packet.angular_velocity[2];
    fcs_global_ahrs_state.wind_velocity[0] = packet.wind_velocity[0];
    fcs_global_ahrs_state.wind_velocity[1] = packet.wind_velocity[1];
    fcs_global_ahrs_state.wind_velocity[2] = packet.wind_velocity[2];
    fcs_global_ahrs_state.mode = FCS_MODE_ACTIVE;
    fcs_global_control_state.mode = FCS_CONTROL_MODE_AUTO;

    return true;

invalid:
    return false;
}

/*
Serialize a control packet containing `control_values` into `buf`.
*/
size_t _fcs_format_control_hitl_out_packet(uint8_t *buf, uint16_t tick,
const uint16_t *restrict control_values) {
    assert(buf);
    assert(control_values);

    struct control_hitl_out_packet_t packet;

    packet.tick = tick;

    uint8_t i;
    #pragma MUST_ITERATE(4, 4)
    for (i = 0; i < 4u; i++) {
        packet.pwm[i] = (uint16_t)control_values[i];
    }

    packet.objective_val = fcs_global_counters.nmpc_objective_value;
    packet.cycles = fcs_global_counters.nmpc_last_cycle_count;

    /* Copy the current reference point into the packet as well */
    packet.reference_lat = fcs_global_nav_state.reference_trajectory[0].lat;
    packet.reference_lon = fcs_global_nav_state.reference_trajectory[0].lon;
    packet.reference_alt = fcs_global_nav_state.reference_trajectory[0].alt;
    packet.reference_airspeed =
        fcs_global_nav_state.reference_trajectory[0].airspeed;
    packet.reference_yaw = fcs_global_nav_state.reference_trajectory[0].yaw;
    packet.reference_pitch =
        fcs_global_nav_state.reference_trajectory[0].pitch;
    packet.reference_roll = fcs_global_nav_state.reference_trajectory[0].roll;

    /* Calculate the packet's CRC8 */
    packet.crc = fcs_crc8((uint8_t*)&packet.tick, sizeof(packet) - 1u, 0);

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
