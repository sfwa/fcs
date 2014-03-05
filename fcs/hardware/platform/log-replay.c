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

#ifdef FCS_COMPILE_BOARD_LOG

#define FCS_HITL_PACKET_TIMEOUT 50
#define FCS_HITL_RESET_TIMEOUT 500


/* Prototypes of internal functions */
bool _fcs_read_measurement_packet(enum fcs_stream_device_t dev);
bool _fcs_decode_packet(const uint8_t *buf, size_t nbytes);

void fcs_board_init_platform(void) {
    /*
    Set I/O board serial baud rates. We don't actually open the streams here
    -- we wait for the I/O board comms to time out and reset the links to
    ensure we get a clean start.
    */
    enum fcs_stream_result_t result;
    result = fcs_stream_set_rate(FCS_STREAM_UART_EXT1, 2604166u);
    assert(result == FCS_STREAM_OK);
}

void fcs_board_tick(void) {
    static int16_t comms_timeout;

    /*
    Write one output packet per input packet to make sure things don't get
    out of hand.
    */
    if (_fcs_read_measurement_packet(FCS_STREAM_UART_EXT1)) {
        comms_timeout = FCS_HITL_PACKET_TIMEOUT;
    }

    /*
    If the stream has timed out, start the reset. Otherwise, just decrement
    the timeout counter.
    */
    if (comms_timeout <= 0) {
        comms_timeout = FCS_HITL_RESET_TIMEOUT;

        enum fcs_stream_result_t result;
        result = fcs_stream_open(FCS_STREAM_UART_EXT1);
        assert(result == FCS_STREAM_OK);
    } else if (comms_timeout > INT16_MIN) {
        comms_timeout--;
    }
}

bool _fcs_read_measurement_packet(enum fcs_stream_device_t dev) {
    /* Read the latest packet from the UART streams */
    uint8_t buf[256];
    size_t nbytes, i, packet_start = 0, packet_end = 0;
    enum {
        UNKNOWN,
        GOT_ZERO,
        IN_PACKET,
        ENDED_PACKET
    } state = UNKNOWN;
    bool result = false;

    nbytes = fcs_stream_read(dev, buf, 255u);
    while (!result && nbytes >= 8u) {
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

/* Decode an input packet from `buf` */
bool _fcs_decode_packet(const uint8_t *buf, size_t nbytes) {
    uint16_t crc, packet_crc;
    struct fcs_measurement_log_t mlog;
    struct fcs_cobsr_decode_result result;

    /* Decode the message into the packet buffer */
    result = fcs_cobsr_decode((uint8_t*)&mlog, 256u, buf, nbytes - 1u);

    /* Confirm decode was successful */
    if (result.status != FCS_COBSR_DECODE_OK || result.out_len < 2u) {
        goto invalid;
    }

    mlog.length = (size_t)result.out_len - 2u;

    /* Validate CRC */
    crc = fcs_crc16_sbp(mlog.data, mlog.length, 0xFFFFu);
    packet_crc = (uint16_t)(mlog.data[mlog.length + 0] |
                            (mlog.data[mlog.length + 1u] << 8u));
    if (crc != packet_crc) {
        goto invalid;
    }

    /* Write the result into the AHRS measurement log */
    memcpy(&fcs_global_ahrs_state.measurements, &mlog, sizeof(mlog));

    return true;

invalid:
    return false;
}

#endif
