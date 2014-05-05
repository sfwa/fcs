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

#define fcs_assert(x) if (!(x)) { while (1); }

/*
fcs_crc32 - calculate a CRC32(B) over the given input data using the EDB88320
polynomial (ANSI X3.66, ITU-T V.42, Ethernet, Gzip, PNG etc).

pdata: pointer to data buffer.
nbytes: number of bytes in data buffer.
crc: starting value; must be 0xFFFFFFFFu for standards compliance.
*/
#pragma FUNC_IS_PURE(fcs_crc32);
uint32_t fcs_crc32(const uint8_t *restrict pdata, size_t nbytes,
uint32_t crc);


/* COBS-R prototypes */
enum fcs_cobsr_encode_status{
    FCS_COBSR_ENCODE_OK = 0x00u,
    FCS_COBSR_ENCODE_NULL_POINTER = 0x01u,
    FCS_COBSR_ENCODE_OUT_BUFFER_OVERFLOW = 0x02u
};

struct fcs_cobsr_encode_result {
    ptrdiff_t out_len;
    enum fcs_cobsr_encode_status status;
};

enum fcs_cobsr_decode_status {
    FCS_COBSR_DECODE_OK = 0x00u,
    FCS_COBSR_DECODE_NULL_POINTER = 0x01u,
    FCS_COBSR_DECODE_OUT_BUFFER_OVERFLOW = 0x02u,
    FCS_COBSR_DECODE_ZERO_BYTE_IN_INPUT = 0x04u
};

struct fcs_cobsr_decode_result {
    ptrdiff_t out_len;
    enum fcs_cobsr_decode_status status;
};

#define FCS_COBSR_SIZE_LIMIT 1024u

/*
COBS-R encode/decode routines. The format is described here:
http://pythonhosted.org/cobs/cobsr-intro.html

COBS-R is used by the I/O board serial protocol to ensure that NUL bytes never
appear within packets, allowing them to be used as packet delimiters. Encoding
packets < 256 bytes with COBS-R results in an overhead of at most one byte.
*/
struct fcs_cobsr_encode_result fcs_cobsr_encode(uint8_t *dst_buf_ptr,
size_t dst_buf_len, const uint8_t * src_ptr, size_t src_len);
struct fcs_cobsr_decode_result fcs_cobsr_decode(uint8_t *dst_buf_ptr,
size_t dst_buf_len, const uint8_t * src_ptr, size_t src_len);

#endif
