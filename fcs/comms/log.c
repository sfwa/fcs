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

/* Initialize a log packet with a packet index of `frame_id` */
void fcs_comms_init_log(struct fcs_packet_log_t *log_rec, uint16_t frame_id) {
    assert(log_rec);

    log_rec->buf[0] = FCS_PACKET_LOG_TYPE;
    log_rec->buf[1u] = 0;
    log_rec->buf[2u] = 0;
    log_rec->buf[3u] = (frame_id & 0x00FFu);
    log_rec->buf[4u] = (frame_id & 0xFF00u) >> 8u;
    log_rec->len = 5u;
}

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log_rec` to include a CRC16SBP.
*/
size_t fcs_comms_serialize_log(uint8_t *restrict out_buf, size_t out_buf_len,
struct fcs_packet_log_t *log_rec) {
    assert(out_buf);
    assert(out_buf_len);
    assert(log_rec);
    /* 2 bytes for CRC, 3 bytes for COBS-R + NUL start/end */
    assert(out_buf_len >= log_rec->len + 2u + 3u);

    /* Calculate checksum and update the packet with the result */
    uint16_t crc = fcs_crc16_sbp(log_rec->buf, log_rec->len, 0xFFFFu);
    log_rec->buf[log_rec->len + 0] = (crc & 0x00FFu);
    log_rec->buf[log_rec->len + 1u] = (crc & 0xFF00u) >> 8u;

    /* Write COBS-R encoded result to out_buf */
    struct fcs_cobsr_encode_result result;
    result = fcs_cobsr_encode(&out_buf[1u], out_buf_len - 2u, log_rec->buf,
                              log_rec->len + 2u);
    assert(result.status == FCS_COBSR_ENCODE_OK);

    /* Add NUL start/end bytes */
    out_buf[0] = 0;
    out_buf[result.out_len + 1u] = 0;

    /* Return the COBS-R encoded length, plus the length of the NUL bytes */
    return result.out_len + 2u;
}

/*
Add a sensor value entry to a log packet. Returns true if the sensor value
could be added, or false if it couldn't.
*/
bool fcs_comms_add_log_sensor_value(struct fcs_packet_log_t *log_rec,
enum fcs_sensor_type_t sensor_type, uint8_t sensor_id,
const uint8_t *restrict sensor_val, size_t sensor_val_len) {
    assert(log_rec);
    assert(sensor_type < FCS_SENSOR_TYPE_LAST);
    assert(sensor_id <= FCS_SENSOR_ID_MAX);
    assert(sensor_val);
    assert(sensor_val_len);
    assert(sensor_val_len <= FCS_SENSOR_LEN_MAX);

    /*
    If there's not enough space in the log record to save the value, return
    false. 250 bytes to allow space for the CRC16, the COBS-R encoding and
    two NUL bytes within a 256-byte packet.
    */
    if (log_rec->len + sensor_val_len + 2u > 250u) {
        return false;
    }

    /* Set up the sensor value header */
    uint8_t *restrict buf = &log_rec->buf[log_rec->len];
    buf[0] = sensor_val_len;
    buf[1u] = (sensor_id << 5u) + sensor_type;

    /* Copy the sensor data */
    memcpy(&buf[2u], sensor_val, sensor_val_len);

    /* Update the log record's length */
    log_rec->len += 2u + sensor_val_len;

    return true;
}
