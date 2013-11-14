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

#ifndef _FCS_UTIL_H
#define _FCS_UTIL_H

void fcs_util_init(void);
void fcs_util_tick(void);
void fcs_util_update_state(const struct fcs_state_t *new_state);

/*
http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
0x97 in Koopman notation = 0x12F in MSB-first notation, so excluing implicit
x^n term we get 2F.
*/
#define FCS_CRC8_POLY 0x2Fu

/*
fcs_crc8_init - fill crc table for given polynomial in regular bit
order (omitting leading 1 for implicit x^n).

polynomial:  polynomial for which table is to be filled.
*/
void fcs_crc8_init(uint8_t polynomial);

/*
fcs_crc8 - calculate a CRC8 over the given input data using the polynomial
previously supplied to fcs_crc8_init.

pdata: pointer to data buffer.
nbytes: number of bytes in data buffer.
crc: previous returned crc8 value.
*/
uint8_t fcs_crc8(const uint8_t *restrict pdata, uint32_t nbytes,
uint8_t crc);

/*
fcs_crc32 - calculate a CRC32 over the given input data using the EDB88320
polynomial (ANSI X3.66, ITU-T V.42, Ethernet, Gzip, PNG etc).

pdata: pointer to data buffer.
nbytes: number of bytes in data buffer.
crc: starting value; must be 0xffffffffu for standards compliance.
*/
uint32_t fcs_crc32(const uint8_t *restrict pdata, uint32_t nbytes,
uint32_t crc);

/* COBS-R prototypes */
enum fcs_cobsr_encode_status{
    FCS_COBSR_ENCODE_OK = 0x00u,
    FCS_COBSR_ENCODE_NULL_POINTER = 0x01u,
    FCS_COBSR_ENCODE_OUT_BUFFER_OVERFLOW = 0x02u
};

struct fcs_cobsr_encode_result {
    uint32_t out_len;
    enum fcs_cobsr_encode_status status;
};

enum fcs_cobsr_decode_status {
    FCS_COBSR_DECODE_OK = 0x00u,
    FCS_COBSR_DECODE_NULL_POINTER = 0x01u,
    FCS_COBSR_DECODE_OUT_BUFFER_OVERFLOW = 0x02u,
    FCS_COBSR_DECODE_ZERO_BYTE_IN_INPUT = 0x04u
};

struct fcs_cobsr_decode_result {
    uint32_t out_len;
    enum fcs_cobsr_decode_status status;
};

struct fcs_cobsr_encode_result fcs_cobsr_encode(uint8_t *dst_buf_ptr,
uint32_t dst_buf_len, const uint8_t * src_ptr, uint32_t src_len);
struct fcs_cobsr_decode_result fcs_cobsr_decode(uint8_t *dst_buf_ptr,
uint32_t dst_buf_len, const uint8_t * src_ptr, uint32_t src_len);

#endif
