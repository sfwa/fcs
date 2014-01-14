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
#include "../TRICAL/TRICAL.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "../stats/stats.h"
#include "comms.h"

#define UART0_RATE 921600u
#define UART0_STATE_INTERVAL 20u
#define UART0_STATUS_INTERVAL 100u
#define UART0_STATUS_OFFSET 12u

#define UART1_RATE 57600u
#define UART1_STATE_INTERVAL 500u
#define UART1_STATUS_INTERVAL 500u
#define UART1_STATUS_OFFSET 250u

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
} __attribute__ ((packed));

void _fcs_comms_parse_packets(enum fcs_stream_device_t dev);
size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf);

static inline uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | ((val >> 8) & 0xFF);
}

void fcs_comms_init(void) {
    /* Open the CPU comms stream */
    assert(fcs_stream_set_rate(FCS_STREAM_UART_EXT0, UART0_RATE) ==
           FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT0) == FCS_STREAM_OK);

    /* Open the RFD900 comms stream */
    assert(fcs_stream_set_rate(FCS_STREAM_UART_EXT1, UART1_RATE) ==
           FCS_STREAM_OK);
    assert(fcs_stream_open(FCS_STREAM_UART_EXT1) == FCS_STREAM_OK);
}

void fcs_comms_tick(void) {
    static uint32_t tick;

    uint8_t comms_buf[256];
    size_t comms_buf_len, write_len;

    /* Generate a state packet for delivery to the CPU and RFD900a */
    comms_buf_len = fcs_comms_serialize_state(
        comms_buf, &fcs_global_ahrs_state);
    assert(comms_buf_len && comms_buf_len < 256u);

    /* Send a state update packet to the CPU every 20ms (50Hz) */
    if (tick % UART0_STATE_INTERVAL == 0) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT0, comms_buf,
                                     comms_buf_len);
        fcs_global_counters.cpu_packet_tx++;
        /* We should definitely have enough room in the write buffer */
        assert(comms_buf_len == write_len);
    }

    /* Send a state update packet to the RFD900a every 1s */
    if (tick % UART1_STATE_INTERVAL == 0) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT1, comms_buf,
                                     comms_buf_len);
        /* Ignore a buffer full result */
    }

    /* Generate a status packet, resetting event counts every second */
    comms_buf_len = fcs_comms_serialize_status(
        comms_buf, &fcs_global_ahrs_state, &fcs_global_counters,
        &fcs_global_peripheral_state, tick % 1000u == 0 ? true : false);
    assert(comms_buf_len && comms_buf_len < 256u);

    /*
    Send a status update packet to the CPU every 100ms (10Hz), but long enough
    after the state update packet that the writes won't collide
    */
    if (tick % UART0_STATUS_INTERVAL == UART0_STATUS_OFFSET) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT0, comms_buf,
                                     comms_buf_len);
        fcs_global_counters.cpu_packet_tx++;
        assert(comms_buf_len == write_len);
    }

    /*
    Send a status update packet to the RFD900a every 1s, 500ms after the state
    packet was sent
    */
    if (tick % UART1_STATUS_INTERVAL == UART1_STATUS_OFFSET) {
        write_len = fcs_stream_write(FCS_STREAM_UART_EXT1, comms_buf,
                                     comms_buf_len);
        /* Ignore a buffer full result */
    }

    /*
    Write the log values to internal stream 1 -- the CPLD will route this to
    the CPU UART

    comms_buf_len = fcs_measurement_log_serialize(
        comms_buf, sizeof(comms_buf), &fcs_global_ahrs_state.measurements);
    write_len = fcs_stream_write(FCS_STREAM_UART_INT1, comms_buf,
                                 comms_buf_len);
    assert(comms_buf_len == write_len);
    */

    /* Check for packets from both the CPU and comms UARTs */
    _fcs_comms_parse_packets(FCS_STREAM_UART_EXT0);
    _fcs_comms_parse_packets(FCS_STREAM_UART_EXT1);

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
Receive and parse all comms packets received on the UART identified by `dev`.
*/
void _fcs_comms_parse_packets(enum fcs_stream_device_t dev) {
    uint8_t comms_buf[256];
    size_t comms_buf_len;

    comms_buf_len = _fcs_comms_read_packet(dev, comms_buf);
    assert(comms_buf_len < 256u);

    if (comms_buf_len > 3u && comms_buf[0] == 0) {
        /* Initial byte is NUL, so this is an RFD900 status packet */
        struct fcs_rfd900_status_packet_t packet;
        struct fcs_cobsr_decode_result result;

        result = fcs_cobsr_decode((uint8_t*)&packet, sizeof(packet),
                                  &comms_buf[1], comms_buf_len - 2u);

        /* If decode is successful, update the counters */
        if (result.status == FCS_COBSR_DECODE_OK &&
                result.out_len == sizeof(packet)) {
            /*
            RSSI/noise calcs per
            http://code.google.com/p/ardupilot-mega/wiki/3DRadio
            */
            fcs_global_peripheral_state.telemetry_rssi =
                (float)packet.rssi / 1.9f - 127.0f;
            fcs_global_peripheral_state.telemetry_noise =
                (float)packet.noise / 1.9f - 127.0f;

            /*
            FIXME:
            These are counts wrapped to 16 bits; we should really look for the
            change between this packet and last packet, and increment our
            counts accordingly.
            */
            fcs_global_peripheral_state.telemetry_errors =
                swap_uint16(packet.rx_errors);
            fcs_global_peripheral_state.telemetry_errors_corrected =
                swap_uint16(packet.rx_errors_fixed);
        }
    } else if (comms_buf_len && comms_buf[0] == '$') {
        /* Initial byte is '$', so this is a control packet */
        enum fcs_deserialization_result_t result;

        switch (comms_buf[6]) {
            case 'S':
                /* TODO */
                break;
            case 'G':
                result = fcs_comms_deserialize_gcs(comms_buf, comms_buf_len);
                break;
            case 'T':
                /* TODO */
                break;
            case 'C':
                result = fcs_comms_deserialize_command(comms_buf,
                                                       comms_buf_len);
                break;
            default:
                result = FCS_DESERIALIZATION_ERROR;
                break;
        }

        if (result == FCS_DESERIALIZATION_OK) {
            fcs_global_counters.cpu_packet_rx++;
        } else {
            fcs_global_counters.cpu_packet_rx_err++;
        }
    }
}

/*
Read a full message from `dev`, starting with $ and ending with \n. Neither
of those characters can appear in the message itself, so we don't need to do
any message parsing at this level.

The complication is that the RFD900 can inject NUL-terminated packets
containing link status information. That means we need to check for $ and \n
as well as NUL and NUL.

`buf` must be at least 256 characters long.
*/
size_t _fcs_comms_read_packet(enum fcs_stream_device_t dev, uint8_t *buf) {
    assert(buf);

    size_t nbytes, i, packet_start = 0, packet_end = 0;
    enum {
        UNKNOWN,
        GOT_ZERO,
        IN_RFD900_PACKET,
        ENDED_RFD900_PACKET,
        GOT_DOLLARS,
        IN_CPU_PACKET,
        ENDED_CPU_PACKET
    } state = UNKNOWN;
    bool result = false;

    nbytes = fcs_stream_read(dev, buf, 255u);
    for (i = 0; i < nbytes && state != ENDED_RFD900_PACKET &&
            state != ENDED_CPU_PACKET; i++) {
        switch (state) {
            case UNKNOWN:
                if (buf[i] == 0) {
                    state = GOT_ZERO;
                } else if (buf[i] == '$') {
                    state = GOT_DOLLARS;
                }
                break;
            case GOT_ZERO:
                if (buf[i] == '$' && buf[i - 1u] == 0) {
                    state = GOT_DOLLARS;
                    packet_start = 0;
                } else if (buf[i] != 0) {
                    state = IN_RFD900_PACKET;
                    packet_start = i - 1;
                }
                break;
            case IN_RFD900_PACKET:
                if (buf[i] == 0) {
                    state = ENDED_RFD900_PACKET;
                    packet_end = i;
                }
                break;
            case GOT_DOLLARS:
                if (buf[i] == 0) {
                    state = GOT_ZERO;
                } else if (buf[i] == 'P') {
                    state = IN_CPU_PACKET;
                    packet_start = i - 1;
                }
                break;
            case IN_CPU_PACKET:
                if (buf[i] == 0) {
                    state = GOT_ZERO;
                    packet_start = 0;
                } else if (buf[i] == '\n') {
                    state = ENDED_CPU_PACKET;
                    packet_end = i;
                }
                break;
            default:
                assert(false);
                break;
        }
    }

    if (state == ENDED_RFD900_PACKET || state == ENDED_CPU_PACKET) {
        memmove(buf, &buf[packet_start], packet_end - packet_start + 1u);

        /* Consume all bytes until the end of the packet */
        fcs_stream_consume(dev, packet_end + 1u);

        /* Return the number of bytes read */
        return packet_end - packet_start + 1u;
    } else if ((state == IN_RFD900_PACKET || state == IN_CPU_PACKET) &&
            i < 255u) {
        /*
        Consume bytes until the start of the packet, so we can parse the
        full packet next time
        */
        fcs_stream_consume(dev, packet_start);
        return 0;
    } else if (state == GOT_ZERO || state == GOT_DOLLARS) {
        /* Consume bytes up to the current start character */
        fcs_stream_consume(dev, nbytes - 1u);
        return 0;
    } else {
        /*
        Something has gone badly wrong. Consume the whole buffer and hope
        we get actual data next time.
        */
        fcs_stream_consume(dev, nbytes);
        return 0;
    }
}
