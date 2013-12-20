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

#include "../config/config.h"
#include "../util/util.h"
#include "../drivers/stream.h"
#include "comms.h"
#include "../ahrs/ahrs.h"
#include "../stats/stats.h"

static struct fcs_packet_state_t comms_state_in;
static bool comms_state_valid;
static struct fcs_packet_config_t comms_config_in;
static bool comms_config_valid;
static struct fcs_packet_gcs_t comms_gcs_in;
static bool comms_gcs_valid;
static struct fcs_packet_waypoint_t comms_waypoint_in;
static bool comms_waypoint_valid;

size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf);

void fcs_comms_init(void) {
    /* Open the CPU comms stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT0, 921600u) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT0) == FCS_STREAM_OK);

    /* Open the RFD900 comms stream */
    assert(
        fcs_stream_set_rate(FCS_STREAM_UART_EXT1, 230400u) == FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT1) == FCS_STREAM_OK);
}

void fcs_comms_tick(void) {
    static uint32_t tick;

    uint8_t comms_buf[256u];
    size_t comms_buf_len, write_len;

    /* Generate a state packet for delivery to the CPU and RFD900a */
    comms_buf_len = fcs_comms_serialize_state(comms_buf, &fcs_global_state);
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
    status.solution_time = fcs_global_state.solution_time;
    status.ioboard_resets[0] =
        fcs_global_counters.ioboard_resets[0] <= INT32_MAX ?
        fcs_global_counters.ioboard_resets[0] : INT32_MAX;
    status.ioboard_resets[1u] =
        fcs_global_counters.ioboard_resets[1u] <= INT32_MAX ?
        fcs_global_counters.ioboard_resets[1u] : INT32_MAX;
    status.trical_resets[0] =
        fcs_global_counters.trical_resets[0] <= INT32_MAX ?
        fcs_global_counters.trical_resets[0] : INT32_MAX;
    status.trical_resets[1u] =
        fcs_global_counters.trical_resets[1u] <= INT32_MAX ?
        fcs_global_counters.trical_resets[1u] : INT32_MAX;
    status.ukf_resets =
        fcs_global_counters.ukf_resets <= INT32_MAX ?
        fcs_global_counters.ukf_resets : INT32_MAX;
    status.main_loop_cycle_max[0] =
        fcs_global_counters.main_loop_cycle_max[0] <= INT32_MAX ?
        fcs_global_counters.main_loop_cycle_max[0] : INT32_MAX;
    status.main_loop_cycle_max[1u] =
        fcs_global_counters.main_loop_cycle_max[1u] <= INT32_MAX ?
        fcs_global_counters.main_loop_cycle_max[1u] : INT32_MAX;
    status.cpu_packet_rx =
        fcs_global_counters.cpu_packet_rx <= INT32_MAX ?
        fcs_global_counters.cpu_packet_rx : INT32_MAX;
    status.cpu_packet_rx_err =
        fcs_global_counters.cpu_packet_rx_err <= INT32_MAX ?
        fcs_global_counters.cpu_packet_rx_err : INT32_MAX;
    status.gps_num_svs = 0; /* TODO */
    status.telemetry_signal_db = 0; /* TODO */
    status.telemetry_noise_db = 0; /* TODO */
    status.telemetry_packet_rx = 0; /* TODO */
    status.telemetry_packet_rx_err = 0; /* TODO */
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

    /* Check for packets */
    comms_buf_len = _fcs_comms_read_packet(FCS_STREAM_UART_EXT0, comms_buf);
    assert(comms_buf_len < 256u);

    if (comms_buf_len >= FCS_COMMS_MIN_PACKET_SIZE) {
        enum fcs_deserialization_result_t result;
        switch (comms_buf[6u]) {
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
*/
size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf) {
    assert(buf);

    uint8_t i = 0;
    uint32_t nbytes;

    nbytes = fcs_stream_bytes_available(dev);

    /*
    If the initial byte is not $, we haven't synchronised with the start of a
    message -- try to do that now by skipping until we see a \n (which should
    be the end of \r\n).

    Give up after 4 tries.
    */
    while (i < 4u && fcs_stream_peek(dev) != '$' &&
           nbytes >= FCS_COMMS_MIN_PACKET_SIZE) {
        fcs_stream_skip_until_after(dev, (uint8_t)'\n');
        nbytes = fcs_stream_bytes_available(dev);
        i++;
    }

    /*
    Give up if there aren't enough bytes remaining, or we're not currently at
    a '$' character
    */
    if (nbytes < FCS_COMMS_MIN_PACKET_SIZE || i == 4) {
        return false;
    }

    /* Read until after the next (terminating) '\n' */
    nbytes = fcs_stream_read_until_after(dev, (uint8_t)'\n', buf, 255u);
    if (nbytes >= FCS_COMMS_MIN_PACKET_SIZE) {
        return nbytes;
    } else {
        return 0;
    }
}
