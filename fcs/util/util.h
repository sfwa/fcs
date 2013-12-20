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

/*
fcs_util_init -- performs setup required for utility functions to work
*/
void fcs_util_init(void);

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
#pragma FUNC_IS_PURE(fcs_crc8);
uint8_t fcs_crc8(const uint8_t *restrict pdata, uint32_t nbytes,
uint8_t crc);

/*
fcs_crc16 - calculate a so-called CRC16 over the given input data. This
implementation is specifically designed for compatibility with the CRC16 used
in the SwiftNav Binary Protocol for the Piksi GPS. According to their
documentation, it's a CRC16-CCITT, but it's really not.

pdata: pointer to data buffer.
nbytes: number of bytes in data buffer.
crc: starting value; must be 0xFFFFu for standards compliance.
*/
#pragma FUNC_IS_PURE(fcs_crc16_sbp);
uint16_t fcs_crc16_sbp(const uint8_t *restrict pdata, uint32_t nbytes,
uint16_t crc);

/*
fcs_crc32 - calculate a CRC32(B) over the given input data using the EDB88320
polynomial (ANSI X3.66, ITU-T V.42, Ethernet, Gzip, PNG etc).

pdata: pointer to data buffer.
nbytes: number of bytes in data buffer.
crc: starting value; must be 0xFFFFFFFFu for standards compliance.
*/
#pragma FUNC_IS_PURE(fcs_crc32);
uint32_t fcs_crc32(const uint8_t *restrict pdata, uint32_t nbytes,
uint32_t crc);

/*
fcs_text_checksum - calculate an NMEA0183-compatible checksum over the given
input data.
*/
#pragma FUNC_IS_PURE(fcs_text_checksum);
uint8_t fcs_text_checksum(const uint8_t *restrict pdata, uint32_t nbytes);

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

/*
COBS-R encode/decode routines. The format is described here:
http://pythonhosted.org/cobs/cobsr-intro.html

COBS-R is used by the I/O board serial protocol to ensure that NUL bytes never
appear within packets, allowing them to be used as packet delimiters. Encoding
packets < 256 bytes with COBS-R results in an overhead of at most one byte.
*/
struct fcs_cobsr_encode_result fcs_cobsr_encode(uint8_t *dst_buf_ptr,
uint32_t dst_buf_len, const uint8_t * src_ptr, uint32_t src_len);
struct fcs_cobsr_decode_result fcs_cobsr_decode(uint8_t *dst_buf_ptr,
uint32_t dst_buf_len, const uint8_t * src_ptr, uint32_t src_len);

/*
fcs_ascii_fixed_from_double -- convert a double to ASCII, with output written
to the result buffer. Returns the number of bytes written.

max_integer_digits controls the number of digits before the decimal place;
max_fractional_digits controls the number of digits after.

max_fractional_digits must be <= 7; max_integer_digits must be <= 6. Either
max_fractional_digits or max_integer_digits must be greater than zero. Numbers
too large to be represented will output "OF" or "-OF".

The buffer length must be at least (1 + max_integer_digits + 1 +
max_fractional_digits).
*/
size_t fcs_ascii_fixed_from_double(uint8_t *restrict result, double value,
uint8_t max_integer_digits, uint8_t max_fractional_digits);

/*
fcs_ascii_from_int32 -- convert an integer to ASCII. Restricts the number of
digits output to max_digits; numbers greater in magnitude will output "OF" or
"-OF". Returns the number of bytes written.

The buffer length must be at least (1 + max_digits).
*/
size_t fcs_ascii_from_int32(uint8_t *result, int32_t value,
uint8_t max_digits);

enum fcs_conversion_result_t {
    FCS_CONVERSION_OK,
    FCS_CONVERSION_ERROR
};

/*
fcs_double_from_ascii_fixed -- convert an ASCII string (nnnn.nnn, nnn, or
.nnn) to double. Returns FCS_CONVERSION_OK if the result is valid, or
 FCS_CONVERSION_ERROR if not.

The integral part of the fixed-point value must have 6 digits or fewer, and
the fractional part must have 7 digits or fewer.
*/
enum fcs_conversion_result_t fcs_double_from_ascii_fixed(
double *restrict result, const uint8_t *restrict value, size_t len);

/*
fcs_int32_from_ascii -- convert an ASCII string (nnnn) to int32_t. Returns
FCS_CONVERSION_OK if the result is valid, or FCS_CONVERSION_ERROR if not.
*/
enum fcs_conversion_result_t fcs_int32_from_ascii(int32_t *restrict result,
const uint8_t *restrict value, size_t len);

/*
fcs_ascii_hex_from_uint8 -- convert a uint8_t value to two uppercase hex
digits
*/
size_t fcs_ascii_hex_from_uint8(uint8_t *restrict result, uint8_t value);

/*
fcs_uint8_from_ascii_hex -- convert two uppercase hex digits to a uint8.
The len parameter must be 2.
*/
enum fcs_conversion_result_t fcs_uint8_from_ascii_hex(uint8_t *result,
uint8_t *restrict value, size_t len);

/*
fcs_ascii_hex_from_uint32 -- convert a uint32_t value to eight uppercase
hex digits (MSB first)
*/
size_t fcs_ascii_hex_from_uint32(uint8_t *restrict result, uint32_t value);

/*
fcs_uint32_from_ascii_hex -- convert eight uppercase hex digits (MSB first)
to a uint32. The len parameter must be 8.
*/
enum fcs_conversion_result_t fcs_uint32_from_ascii_hex(uint32_t *result,
uint8_t *restrict value, size_t len);

/* Base64 encode/decode routines */

ptrdiff_t fcs_base64_from_data(uint8_t *restrict base64, size_t base64len,
const uint8_t *restrict data, size_t datalen);
ptrdiff_t fcs_data_from_base64(uint8_t *restrict data, size_t datalen,
const uint8_t *restrict base64, size_t base64len);

#endif
