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
#include <assert.h>
#include <math.h>

#include "../config/config.h"
#include "util.h"

/* Positive and negative powers of 10 for clamping/rounding */
static const double _fcs_exp10[14] = {
    1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9,
    1e10, 1e11, 1e12, 1e13
};

static const double _fcs_exp10neg[7] = {
    1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7
};

/*
Same deal, but integers. These are all negative because we do the base
conversion on negative numbers to avoid the -INT_MIN undefinedness.
*/
static const int32_t _fcs_exp10i[11] = {
    -1, -10, -100, -1000, -10000, -100000, -1000000, -10000000, -100000000,
    -1000000000
};

/* For hex conversion code */
static const uint8_t _fcs_hex_digits[] = "0123456789ABCDEF";

void fcs_util_init(void) {
	fcs_crc8_init(FCS_CRC8_POLY);
}

/*
fcs_text_checksum - calculate an NMEA0183-compatible checksum over the given
input data.
*/
uint8_t fcs_text_checksum(const uint8_t *restrict pdata, uint32_t nbytes) {
    assert(pdata);
    assert(nbytes);
    assert(nbytes <= 256);

    uint8_t result = 0x00, i;
    #pragma MUST_ITERATE(1, 256)
    for (i = 0; i < nbytes; i++) {
        result ^= pdata[i];
    }

    return result;
}

/*
Convert an integer to ASCII. Restricts the number of digits output to
max_digits; numbers greater in magnitude will output "OF" or "-OF". Returns
the number of bytes written.

The buffer length must be at least (1 + max_digits).
*/
size_t fcs_ascii_from_int32(uint8_t *restrict result, int32_t value,
uint8_t max_digits) {
    assert(max_digits && max_digits <= 10u);
    assert(result);

    size_t out_len = 0;

    /*
    Since -INT_MIN is undefined but -INT_MAX is OK, do the math on negative
    numbers.
    */
    if (value >= 0) {
        value = -value;
    } else {
        result[0] = '-';
        out_len++;
    }

    /* Check for overflow -- output "OF" or "-OF" */
    if (max_digits < 10u && value <= _fcs_exp10i[max_digits]) {
        result[out_len++] = 'O';
        result[out_len++] = 'F';
    } else if (value == 0) {
        result[out_len++] = '0';
    } else {
        int8_t place = max_digits - 1u;
        /* Skip ahead to the first non-zero place */
        while (place > 0 && _fcs_exp10i[place] < value) {
            place--;
        }

        /*
        For each place, subtract the (negated) value of a unit in that place
        while incrementing the output digit until the value is smaller than
        one unit.

        This is faster than dividing by 10 each time through the loop, since
        the C66 has no divide instruction.
        */
        int32_t place_value;
        uint8_t digit;
        for (; place >= 0; place--) {
            digit = '0';
            place_value = _fcs_exp10i[place];
            while (value <= place_value) {
                digit++;
                value -= place_value;
            }
            result[out_len++] = digit;
        }

        /* Sanity check */
        assert(value == 0);
    }

    return out_len;
}

/*
Convert a double to ASCII, with output written to the result buffer. Returns
the number of bytes written.

max_integer_digits controls the number of digits before the decimal place;
max_fractional_digits controls the number of digits after.

max_fractional_digits must be <= 7; max_integer_digits must be <= 6. Either
max_fractional_digits or max_integer_digits must be greater than zero. Numbers
too large to be represented will output "OF" or "-OF".

The buffer length must be at least (1 + max_integer_digits + 1 +
max_fractional_digits).
*/
size_t fcs_ascii_fixed_from_double(uint8_t *restrict result, double value,
uint8_t max_integer_digits, uint8_t max_fractional_digits) {
    assert(max_integer_digits <= 6u);
    assert(max_fractional_digits <= 7u);
    assert(max_integer_digits || max_fractional_digits);
    assert(result);

    size_t out_len = 0;

    /* Handle the sign */
    if (value < 0.0) {
        value = -value;
        result[0] = '-';
        out_len++;
    }

    /*
    Multiply by the maximum number of fractional digits, then round to the
    nearest integer (ties away from zero).

    This obviously leads to a loss of precision for values >= 2^53, but since
    the maximum number we can represent in 7 + 6 digits is < 2^44, we don't
    need to worry.
    */
    value = round(value * _fcs_exp10[max_fractional_digits]);

    if (isnan(value)) {
        /* Not a number -- output NaN */
        result[out_len++] = 'N';
        result[out_len++] = 'a';
        result[out_len++] = 'N';
    } else if (value >= _fcs_exp10[max_integer_digits +
                                   max_fractional_digits]) {
         /* Overflow -- output "OF" or "-OF" */
        result[out_len++] = 'O';
        result[out_len++] = 'F';
    } else {
        /* Find the highest place used by the number */
        int8_t place = max_integer_digits + max_fractional_digits - 1u;
        while (place >= max_fractional_digits && _fcs_exp10[place] > value) {
            place--;
        }

        /*
        For each place, subtract the value of a unit in that place while
        incrementing the output digit until the value is smaller than
        one unit.

        This is faster than dividing by 10 each time through the loop, since
        the C66 has no divide instruction.
        */
        double place_value;
        uint8_t digit;
        for (; place >= 0; place--) {
            if (max_fractional_digits > 0 && place == max_fractional_digits - 1) {
                result[out_len++] = '.';
            }

            digit = '0';
            place_value = _fcs_exp10[place];
            while (value >= place_value) {
                digit++;
                value -= place_value;
            }
            result[out_len++] = digit;
        }

        /* Handle zero values where max_fractional_digits == 0 */
        if (out_len == 0 || (out_len == 1 && result[0] == '-')) {
            result[out_len++] = '0';
        }

        /* Sanity check */
        assert(value == 0.0);
    }

    return out_len;
}

/*
Convert an ASCII string (nnnn.nnn, nnn, or .nnn) to double. Returns
FCS_CONVERSION_OK if the result is valid, or FCS_CONVERSION_ERROR if not.

The integral part of the fixed-point value must have 6 digits or fewer, and
the fractional part must have 7 digits or fewer.
*/
enum fcs_conversion_result_t fcs_double_from_ascii_fixed(
double *restrict result, const uint8_t *restrict value, size_t len) {
    assert(len);
    assert(value);
    assert(result);

    uint8_t integral_length, fractional_length;
    int32_t integral, fractional;
    double output = 0.0;

    for (integral_length = 0; integral_length < len; integral_length++) {
        if (value[integral_length] == '.') {
            break;
        }
    }

    /* Don't allow a decimal place in last position */
    if (integral_length == len - 1) {
        goto invalid;
    } else if (value[0] == '-' && integral_length > 7u) {
        goto invalid;
    } else if (value[0] != '-' && integral_length > 6u) {
        goto invalid;
    }

    /* Convert the integral part of the number */
    if (integral_length != 0) {
        if (fcs_int32_from_ascii(&integral, value, integral_length) !=
            FCS_CONVERSION_OK) {
            goto invalid;
        }

        output = integral;
    }

    /* If there's a fractional part, convert that too */
    fractional_length = len - integral_length;
    if (fractional_length > 8u) { /* allow an extra 1 for the DP */
        goto invalid;
    } else if (fractional_length > 1u) {
        if (fcs_int32_from_ascii(&fractional, &value[integral_length + 1],
                                 fractional_length - 1) != FCS_CONVERSION_OK) {
            goto invalid;
        }

        if (value[0] == '-') {
            fractional = -fractional;
        }

        /*
        Since the fractional part was converted to an integer, we need to
        determine the place value of the integer component.
        */
        output += (double)fractional * _fcs_exp10neg[fractional_length - 2];
    }

    *result = output;
    return FCS_CONVERSION_OK;

invalid:
    *result = 0.0;
    return FCS_CONVERSION_ERROR;
}

/*
Convert an ASCII string (nnnn) to int32_t. Returns FCS_CONVERSION_OK if the
result is valid, or FCS_CONVERSION_ERROR if not.
*/
enum fcs_conversion_result_t fcs_int32_from_ascii(int32_t *restrict result,
const uint8_t *restrict value, size_t len) {
    assert(len);
    assert(value);
    assert(result);

    bool negative = false;
    uint32_t output = 0;

    /*
    If the number is negative, keep track of that and deal with the sign at
    the end.
    */
    if (value[0] == '-') {
        len--;
        value++;
        negative = true;
    }

    /*
    uint32 can hold up to 4294967295, so we don't need to worry about overflow
    during calculation as long as the most significant digit is smaller than
    or equal to 2
    */
    if (len > 10 || (len == 10 && value[0] > '2')) {
        goto invalid;
    }

    uint8_t i;
    for (i = len; i >= 1; i--) {
        if (value[i - 1] < '0' || value[i - 1] > '9') {
            /* Invalid digit */
            goto invalid;
        }
        output += (uint32_t)(value[i - 1] - '0') *
                  (uint32_t)(-_fcs_exp10i[len - i]);
    }

    if (!negative && output <= INT32_MAX) {
        /* All OK, positive result */
        *result = (int32_t)output;
        return FCS_CONVERSION_OK;
    } else if (negative && output <= 2147483648u) {
        /* All OK, negative result though, so negate it here */
        *result = (int32_t)-output;
        return FCS_CONVERSION_OK;
    }

invalid:
    *result = INT32_MIN;
    return FCS_CONVERSION_ERROR;
}

/* fcs_ascii_hex_from_uint8 -- convert a uint8_t value to two uppercase hex
digits */
size_t fcs_ascii_hex_from_uint8(uint8_t *restrict result, uint8_t value) {
    assert(result);

    result[0] = _fcs_hex_digits[(value & 0xF0u) >> 4];
    result[1] = _fcs_hex_digits[value & 0x0Fu];

    return 2u;
}

/* fcs_uint8_from_ascii_hex-- convert two uppercase hex digits to a uint8 */
enum fcs_conversion_result_t fcs_uint8_from_ascii_hex(uint8_t *result,
const uint8_t *restrict value, size_t len) {
    assert(result);
    assert(value);
    assert(len == 2u);

    uint8_t output;
    if ('0' <= value[0] && value[0] <= '9') {
        output = value[0] - '0';
    } else if ('A' <= value[0] && value[0] <= 'F') {
        output = value[0] - 'A' + 10u;
    } else {
        goto invalid;
    }

    output <<= 4;

    if ('0' <= value[1] && value[1] <= '9') {
        output += value[1] - '0';
    } else if ('A' <= value[1] && value[1] <= 'F') {
        output += value[1] - 'A' + 10u;
    } else {
        goto invalid;
    }

    *result = output;
    return FCS_CONVERSION_OK;

invalid:
    *result = 0xFFu;
    return FCS_CONVERSION_ERROR;
}

/* fcs_ascii_hex_from_uint32 -- convert a uint32_t value to two uppercase hex
digits */
size_t fcs_ascii_hex_from_uint32(uint8_t *restrict result, uint32_t value) {
    assert(result);

    uint8_t i, shift;
    #pragma MUST_ITERATE(8, 8)
    for (i = 0, shift = 28u; i < 8u; i++, shift -= 4u) {
        result[i] = _fcs_hex_digits[(value >> shift) & 0xFu];
    }

    return 8u;
}

/* fcs_uint32_from_ascii_hex -- convert two uppercase hex digits to a uint32.
The len parameter must be 8. */
enum fcs_conversion_result_t fcs_uint32_from_ascii_hex(uint32_t *result,
const uint8_t *restrict value, size_t len) {
    assert(result);
    assert(value);
    assert(len == 8u);

    uint32_t output = 0;
    uint8_t i;
    #pragma MUST_ITERATE(8, 8)
    for (i = 0; i < 8u; i++) {
        output <<= 4u;

        if ('0' <= value[i] && value[i] <= '9') {
            output += value[i] - '0';
        } else if ('A' <= value[i] && value[i] <= 'F') {
            output += value[i] - 'A' + 10u;
        } else {
            goto invalid;
        }
    }

    *result = output;
    return FCS_CONVERSION_OK;

invalid:
    *result = 0xFFFFFFFFu;
    return FCS_CONVERSION_ERROR;
}
