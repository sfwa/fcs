/*
----------------------------------------------------------------------------
Copyright (c) 2010 Craig McQueen

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
----------------------------------------------------------------------------
*/

/*
 * cobsr.c
 *
 * Consistent Overhead Byte Stuffing--Reduced (COBS/R)
 */

#include <stdint.h>
#include <stddef.h>

#include "util.h"

struct fcs_cobsr_encode_result fcs_cobsr_encode(uint8_t *dst_buf_ptr,
size_t dst_buf_len, const uint8_t * src_ptr, size_t src_len) {
    /* Asserts ensure that the main loop terminates, and that buffers do not
    overlap */
    fcs_assert(dst_buf_ptr);
    fcs_assert(src_ptr);
    fcs_assert(dst_buf_len <= FCS_COBSR_SIZE_LIMIT);
    fcs_assert(src_len <= FCS_COBSR_SIZE_LIMIT);
    fcs_assert(dst_buf_ptr + dst_buf_len < src_ptr ||
               dst_buf_ptr > src_ptr + src_len);


    struct fcs_cobsr_encode_result result = { 0, FCS_COBSR_ENCODE_OK };
    const uint8_t *     src_end_ptr         = src_ptr + src_len;
    uint8_t *           dst_buf_end_ptr     = dst_buf_ptr + dst_buf_len;
    uint8_t *           dst_code_write_ptr  = dst_buf_ptr;
    uint8_t *           dst_write_ptr       = dst_code_write_ptr + 1u;
    uint8_t             src_byte            = 0;
    uint8_t             search_len          = 1u;

    /* Iterate over the source bytes */
    for (;;) {
        /* Check for running out of output buffer space */
        if (dst_write_ptr >= dst_buf_end_ptr) {
            result.status |= FCS_COBSR_ENCODE_OUT_BUFFER_OVERFLOW;
            break;
        }

        src_byte = *src_ptr++;
        if (src_byte == 0) {
            /* We found a zero byte */
            *dst_code_write_ptr = search_len;
            dst_code_write_ptr = dst_write_ptr++;
            search_len = 1u;
            if (src_ptr >= src_end_ptr) {
                break;
            }
        } else {
            /* Copy the non-zero byte to the destination buffer */
            *dst_write_ptr++ = src_byte;
            search_len++;
            if (src_ptr >= src_end_ptr) {
                break;
            }
            if (search_len == 0xFFu) {
                /* We have a long string of non-zero bytes, so we need
                 * to write out a length code of 0xFF. */
                *dst_code_write_ptr = 0xFFu;
                dst_code_write_ptr = dst_write_ptr++;
                search_len = 1u;
            }
        }
    }

    /* We've reached the end of the source data (or possibly run out of output buffer)
     * Finalise the remaining output. In particular, write the code (length) byte.
     *
     * For COBS/R, the final code (length) byte is special: if the final data byte is
     * greater than or equal to what would normally be the final code (length) byte,
     * then replace the final code byte with the final data byte, and remove the final
     * data byte from the end of the sequence. This saves one byte in the output.
     *
     * Update the pointer to calculate the final output length.
     */
    if (dst_code_write_ptr >= dst_buf_end_ptr) {
        /* We've run out of output buffer to write the code byte. */
        result.status |= FCS_COBSR_ENCODE_OUT_BUFFER_OVERFLOW;
        dst_write_ptr = dst_buf_end_ptr;
    } else {
        if (src_byte < search_len) {
            /* Encoding same as plain COBS */
            fcs_assert(search_len <= 0xFFu);
            *dst_code_write_ptr = search_len;
        } else {
            /* Special COBS/R encoding: length code is final byte,
             * and final byte is removed from data sequence. */
            *dst_code_write_ptr = src_byte;
            dst_write_ptr--;
        }
    }

    /* Calculate the output length, from the value of dst_code_write_ptr */
    result.out_len = dst_write_ptr - dst_buf_ptr;

    return result;
}


struct fcs_cobsr_decode_result fcs_cobsr_decode(uint8_t *dst_buf_ptr,
size_t dst_buf_len, const uint8_t * src_ptr, size_t src_len) {
    /* Asserts ensure that the main loop terminates, and that buffers do not
    overlap */
    fcs_assert(dst_buf_ptr);
    fcs_assert(src_ptr);
    fcs_assert(dst_buf_len <= FCS_COBSR_SIZE_LIMIT);
    fcs_assert(src_len <= FCS_COBSR_SIZE_LIMIT);
    fcs_assert(dst_buf_ptr + dst_buf_len < src_ptr ||
               dst_buf_ptr > src_ptr + src_len);

    struct fcs_cobsr_decode_result result = { 0, FCS_COBSR_DECODE_OK };
    const uint8_t *     src_end_ptr         = src_ptr + src_len;
    uint8_t *           dst_buf_end_ptr     = dst_buf_ptr + dst_buf_len;
    uint8_t *           dst_write_ptr       = dst_buf_ptr;
    size_t              remaining_input_bytes;
    size_t              remaining_output_bytes;
    size_t              num_output_bytes;
    uint8_t             src_byte;
    size_t              i;
    uint8_t             len_code;

    for (;;) {
        len_code = *src_ptr++;
        if (len_code == 0) {
            result.status |= FCS_COBSR_DECODE_ZERO_BYTE_IN_INPUT;
            break;
        }

        /* Calculate remaining input bytes */
        fcs_assert(src_end_ptr - src_ptr >= 0);
        remaining_input_bytes = (size_t)(src_end_ptr - src_ptr);

        if ((len_code - 1u) < remaining_input_bytes) {
            num_output_bytes = len_code - 1u;

            /* Check length code against remaining output buffer space */
            fcs_assert(dst_buf_end_ptr - dst_write_ptr >= 0);
            remaining_output_bytes =
                (size_t)(dst_buf_end_ptr - dst_write_ptr);
            if (num_output_bytes > remaining_output_bytes) {
                result.status |= FCS_COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                num_output_bytes = remaining_output_bytes;
            }

            fcs_assert(num_output_bytes <= FCS_COBSR_SIZE_LIMIT);
            for (i = num_output_bytes; i != 0; i--) {
                src_byte = *src_ptr++;
                if (src_byte == 0) {
                    result.status |= FCS_COBSR_DECODE_ZERO_BYTE_IN_INPUT;
                }
                *dst_write_ptr++ = src_byte;
            }

            /* Add a zero to the end */
            if (len_code != 0xFFu) {
                if (dst_write_ptr >= dst_buf_end_ptr) {
                    result.status |= FCS_COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                    break;
                }
                *dst_write_ptr++ = 0;
            }
        } else {
            /* We've reached the last length code, so write the remaining
             * bytes and then exit the loop. */

            num_output_bytes = remaining_input_bytes;

            /* Check length code against remaining output buffer space */
            fcs_assert(dst_buf_end_ptr - dst_write_ptr >= 0);
            remaining_output_bytes =
                (size_t)(dst_buf_end_ptr - dst_write_ptr);
            if (num_output_bytes > remaining_output_bytes) {
                result.status |= FCS_COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                num_output_bytes = remaining_output_bytes;
            }

            fcs_assert(num_output_bytes <= FCS_COBSR_SIZE_LIMIT);
            for (i = num_output_bytes; i != 0; i--) {
                src_byte = *src_ptr++;
                if (src_byte == 0) {
                    result.status |= FCS_COBSR_DECODE_ZERO_BYTE_IN_INPUT;
                }
                *dst_write_ptr++ = src_byte;
            }

            /* Write final data byte, if applicable for COBS/R encoding. */
            if (len_code - 1 > remaining_input_bytes) {
                if (dst_write_ptr >= dst_buf_end_ptr) {
                    result.status |= FCS_COBSR_DECODE_OUT_BUFFER_OVERFLOW;
                } else {
                    *dst_write_ptr++ = len_code;
                }
            }

            /* Exit the loop */
            break;
        }
    }

    result.out_len = dst_write_ptr - dst_buf_ptr;

    return result;
}
