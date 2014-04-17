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

#include "log.h"
#include "../util/util.h"


/* Initialize a log packet with a type and packet index */
void fcs_log_init(struct fcs_log_t *restrict plog, enum fcs_log_type_t type,
uint16_t frame_id) {
    assert(plog);
    assert(type > FCS_LOG_TYPE_INVALID);
    assert(type < FCS_LOG_TYPE_LAST);

    plog->data[0] = (uint8_t)type;
    plog->data[1] = 0;
    plog->data[2] = 0;
    plog->data[3] = (frame_id >> 0u) & 0xFFu;
    plog->data[4] = (frame_id >> 8u) & 0xFFu;
    plog->length = FCS_LOG_MIN_LENGTH;
}

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log` to include a CRC32.
*/
size_t fcs_log_serialize(uint8_t *restrict out_buf, size_t out_buf_length,
struct fcs_log_t *plog) {
    assert(out_buf);
    assert(out_buf_length);
    assert(plog);
    assert(FCS_LOG_MIN_LENGTH <= plog->length &&
           plog->length <= FCS_LOG_MAX_LENGTH);
    assert(plog->data[0] > (uint8_t)FCS_LOG_TYPE_INVALID);
    assert(plog->data[0] < (uint8_t)FCS_LOG_TYPE_LAST);

    /*
    4 bytes for CRC, 2 + ceil((len+4) / 256) bytes for COBS-R + NUL start/end
    */
    assert(out_buf_length >= plog->length + 4u + 2u +
                             ((plog->length + 259u) >> 8u));

    /* Calculate checksum and update the packet with the result */
    uint32_t crc = fcs_crc32(plog->data, plog->length, 0xFFFFFFFFu);
    plog->data[plog->length + 0u] = (crc >> 0u) & 0xFFu;
    plog->data[plog->length + 1u] = (crc >> 8u) & 0xFFu;
    plog->data[plog->length + 2u] = (crc >> 16u) & 0xFFu;
    plog->data[plog->length + 3u] = (crc >> 24u) & 0xFFu;

    /* Write COBS-R encoded result to out_buf */
    struct fcs_cobsr_encode_result result;
    result = fcs_cobsr_encode(&out_buf[1], out_buf_length - 2u, plog->data,
                              plog->length + 4u);
    assert(result.status == FCS_COBSR_ENCODE_OK);

    /* Add NUL start/end bytes */
    out_buf[0] = 0;
    out_buf[result.out_len + 1u] = 0;

    /* Return the COBS-R encoded length, plus the length of the NUL bytes */
    assert(result.out_len > 0 && (size_t)result.out_len < SIZE_MAX - 2u);
    return (size_t)(result.out_len + 2u);
}

/* Deserialize a log frame */
bool fcs_log_deserialize(
struct fcs_log_t *plog, const uint8_t *restrict in_buf, size_t in_buf_len) {
    assert(plog);
    assert(in_buf);
    assert(in_buf_len);

    uint32_t crc, packet_crc;
    struct fcs_cobsr_decode_result result;

    /* Validate buffer length */
    if (in_buf_len < 9u || in_buf_len > FCS_LOG_MAX_LENGTH + 10u) {
        goto invalid;
    }

    result = fcs_cobsr_decode(plog->data, FCS_LOG_MAX_LENGTH + 4u, &in_buf[1],
                              in_buf_len - 2u);
    if (result.status != FCS_COBSR_DECODE_OK || result.out_len < 9u ||
            result.out_len - 4u > FCS_LOG_MAX_LENGTH) {
        goto invalid;
    }

    plog->length = (size_t)result.out_len - 4u;

    crc = fcs_crc32(plog->data, plog->length, 0xFFFFFFFFu);
    packet_crc = (uint32_t)plog->data[plog->length + 0u] +
                 ((uint32_t)plog->data[plog->length + 1u] << 8u) +
                 ((uint32_t)plog->data[plog->length + 2u] << 16u) +
                 ((uint32_t)plog->data[plog->length + 3u] << 24u);

    if (crc != packet_crc) {
        goto invalid;
    }

    return true;

invalid:
    /*
    Since the log is invalid, set all data to values which will cause an
    assertion failure as quickly as possible if the caller doesn't check our
    return value;
    */
    memset(plog, 0xFF, sizeof(struct fcs_log_t));
    return false;
}

/*
Merge the logs `src` and `dst`, with the result stored in `dst`.

If `dst` has insufficient space available, return `false`, otherwise `true`.
*/
bool fcs_log_merge(struct fcs_log_t *restrict dst,
const struct fcs_log_t *restrict src) {
    assert(dst);
    assert(FCS_LOG_MIN_LENGTH <= dst->length &&
           dst->length <= FCS_LOG_MAX_LENGTH);
    assert(src);
    assert(FCS_LOG_MIN_LENGTH <= src->length &&
           src->length <= FCS_LOG_MAX_LENGTH);
    assert(dst->data[0] > (uint8_t)FCS_LOG_TYPE_INVALID);
    assert(dst->data[0] < (uint8_t)FCS_LOG_TYPE_LAST);
    assert(src->data[0] > (uint8_t)FCS_LOG_TYPE_INVALID);
    assert(src->data[0] < (uint8_t)FCS_LOG_TYPE_LAST);

    /* Make sure there's enough room in dst */
    if (dst->length + src->length - 5u > FCS_LOG_MAX_LENGTH) {
        return false;
    }

    /* If the types differ, set the dst type to FCS_LOG_TYPE_COMBINED */
    if (dst->data[0] != dst->data[1]) {
        dst->data[0] = (uint8_t)FCS_LOG_TYPE_COMBINED;
    }

    memcpy(&dst->data[dst->length], &src->data[5], src->length - 5u);
    dst->length += src->length - 5u;

    return true;
}