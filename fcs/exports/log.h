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

#ifndef _LOG_H_
#define _LOG_H_

typedef uint16_t float16_t;

#define FCS_LOG_LENGTH 1016u
#define FCS_LOG_SERIALIZED_LENGTH 1024u

enum fcs_log_type_t {
    FCS_LOG_TYPE_INVALID,
    FCS_LOG_TYPE_MEASUREMENT,
    FCS_LOG_TYPE_SENSOR_HAL,
    FCS_LOG_TYPE_ESTIMATE,
    FCS_LOG_TYPE_CONTROL,
    FCS_LOG_TYPE_COMBINED,
    FCS_LOG_TYPE_LAST
};

/*
Binary log packet (all multi-byte values are LE):
1 byte type
2 bytes reserved (0)
2 bytes frame index
... (sensor readings -- fcs_measurement_t)
4 bytes CRC32
*/
struct fcs_log_t {
    uint8_t data[FCS_LOG_LENGTH + 4u];
    size_t length;
};

/* Initialize a log packet with a type and packet index */
void fcs_log_init(struct fcs_log_t *restrict plog, enum fcs_log_type_t type,
uint16_t frame_id);

/*
Serialize and add COBS-R + framing to log packet, and copy the result to
`out_buf`. Returns the length of the serialized data.

Modifies `log` to include a CRC32.
*/
size_t fcs_log_serialize(uint8_t *restrict out_buf, size_t out_buf_len,
struct fcs_log_t *plog);

/* Deserialize a log frame */
bool fcs_log_deserialize(struct fcs_log_t *plog,
const uint8_t *restrict in_buf, size_t in_buf_len);

/*
Merge the logs `src` and `dst`, with the result stored in `dst`.

If `dst` has insufficient space available, return `false`, otherwise `true`.
*/
bool fcs_log_merge(struct fcs_log_t *restrict dst,
const struct fcs_log_t *restrict src);

#endif
