/*
base64encode.c - modified by David Lazar

The encoder implementation has been formally verified as described in this
post:
http://corp.galois.com/blog/2013/9/11/high-assurance-base64.html

Originally:
cencode.c - c source to a base64 encoding algorithm implementation

This is part of the libb64 project, and has been placed in the public domain.
For details, see http://sourceforge.net/projects/libb64
*/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#define BASE64LEN_FROM_DATALEN(x) (((x + (x % 3 ? 3 - (x % 3) : 0)) / 3) << 2)
#define DATALEN_FROM_BASE64LEN(x) (((x + 3) >> 2) * 3)

typedef enum {
    step_A, step_B, step_C
} base64_encodestep;

typedef struct {
    base64_encodestep step;
    uint8_t result;
} base64_encodestate;

typedef enum {
    step_a, step_b, step_c, step_d
} base64_decodestep;

typedef struct {
    base64_decodestep step;
    uint8_t plainchar;
} base64_decodestate;

static ptrdiff_t _fcs_base64_encode_update(base64_encodestate *S,
const uint8_t *data, size_t datalen, uint8_t *encoded);
static ptrdiff_t _fcs_base64_encode_final(base64_encodestate *S,
uint8_t *encoded);
static ptrdiff_t _fcs_base64_decode_block(const uint8_t* code_in,
const size_t length_in, uint8_t* plaintext_out);

static inline void _fcs_base64_encode_init(base64_encodestate *S) {
    assert(S);

    S->step = step_A;
    S->result = 0;
}

static const uint8_t encoding[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnop"
                                  "qrstuvwxyz0123456789+/";

static inline uint8_t _fcs_base64_encode_value(uint8_t value) {
    if (value > 63u) {
        return '=';
    }
    return encoding[value];
}

static ptrdiff_t _fcs_base64_encode_update(base64_encodestate *S,
const uint8_t *data, size_t datalen, uint8_t *encoded) {
    assert(S);
    assert(data);
    assert(encoded);
    assert(datalen);

    uint8_t *encoded_begin = encoded;
    const uint8_t *currbyte = data;
    const uint8_t *data_end = data + datalen;
    uint8_t result;
    uint8_t fragment;

    result = S->result;

    switch (S->step) {
        while (1) {
    case step_A:
            if (currbyte == data_end) {
                S->result = result;
                S->step = step_A;
                return encoded - encoded_begin;
            }
            fragment = *currbyte++;
            result = (fragment & 0x0fcu) >> 2u;
            *encoded++ = _fcs_base64_encode_value(result);
            result = (fragment & 0x003u) << 4u;
    case step_B:
            if (currbyte == data_end) {
                S->result = result;
                S->step = step_B;
                return encoded - encoded_begin;
            }
            fragment = *currbyte++;
            result |= (fragment & 0x0f0u) >> 4u;
            *encoded++ = _fcs_base64_encode_value(result);
            result = (fragment & 0x00fu) << 2u;
    case step_C:
            if (currbyte == data_end) {
                S->result = result;
                S->step = step_C;
                return encoded - encoded_begin;
            }
            fragment = *currbyte++;
            result |= (fragment & 0x0c0u) >> 6u;
            *encoded++ = _fcs_base64_encode_value(result);
            result = (fragment & 0x03fu);
            *encoded++ = _fcs_base64_encode_value(result);
        }
    }

    /* control flow shouldn't reach here */
    assert(false);
    return encoded - encoded_begin;
}

static ptrdiff_t _fcs_base64_encode_final(base64_encodestate *S,
uint8_t *encoded) {
    assert(S);
    assert(encoded);

    uint8_t *encoded_begin = encoded;

    switch (S->step) {
    case step_B:
        *encoded++ = _fcs_base64_encode_value(S->result);
        *encoded++ = '=';
        *encoded++ = '=';
        break;
    case step_C:
        *encoded++ = _fcs_base64_encode_value(S->result);
        *encoded++ = '=';
        break;
    case step_A:
        break;
    }

    return encoded - encoded_begin;
}

ptrdiff_t fcs_base64_from_data(uint8_t *restrict base64, size_t base64len,
const uint8_t *restrict data, size_t datalen) {
    assert(base64);
    assert(base64len);
    assert(data);
    assert(datalen);
    assert(base64len >= BASE64LEN_FROM_DATALEN(datalen));

    memset(base64, 0xFFu, base64len);

    ptrdiff_t c = 0;
    base64_encodestate S;
    _fcs_base64_encode_init(&S);
    c += _fcs_base64_encode_update(&S, data, datalen, base64);
    c += _fcs_base64_encode_final(&S, base64 + c);

    return c;
}

/*
cdecode.c -- from libb64 project, in the public domain. For details, see
http://sourceforge.net/projects/libb64
*/

static const uint8_t decoding[] = {
    62,255,255,255,63,52,53,54,55,56,57,58,59,60,61,255,255,255,254,255,
    255,255,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,
    24,25,255,255,255,255,255,255,26,27,28,29,30,31,32,33,34,35,36,37,38,
    39,40,41,42,43,44,45,46,47,48,49,50,51
};
static const size_t decoding_size = sizeof(decoding);

static inline uint8_t _fcs_base64_decode_value(uint8_t value_in) {
    if (value_in < 43u || value_in >= decoding_size + 43u) {
        return 255u;
    } else {
        return decoding[value_in - 43u];
    }
}

static ptrdiff_t _fcs_base64_decode_block(const uint8_t *restrict code_in,
const size_t length_in, uint8_t *plaintext_out) {
    assert(plaintext_out);
    assert(code_in);
    assert(length_in);

    /* Valid Base64-encoded string lengths must be a multiple of 4 */
    if (length_in & 0x03) {
        return 0;
    }

    uint8_t* plainchar = plaintext_out;
    uint8_t fragment;
    size_t i;
    bool ending = false;

    for (i = 0; i < length_in && !ending;) {
        fragment = _fcs_base64_decode_value(code_in[i++]);
        if (fragment >= 254u) {
            return 0;
        }
        *plainchar = (fragment & 0x03fu) << 2u;

        fragment = _fcs_base64_decode_value(code_in[i++]);
        if (fragment == 255u) {
            return 0;
        } else if (fragment == 254u) {
            ending = true;
        } else {
            *plainchar++ |= (fragment & 0x030u) >> 4u;
            *plainchar = (fragment & 0x00fu) << 4u;
        }

        fragment = _fcs_base64_decode_value(code_in[i++]);
        if (fragment == 255u) {
            return 0;
        } else if (fragment == 254u) {
            ending = true;
        } else if (ending) {
            return 0;
        } else {
            *plainchar++ |= (fragment & 0x03cu) >> 2u;
            *plainchar    = (fragment & 0x003u) << 6u;
        }

        fragment = _fcs_base64_decode_value(code_in[i++]);
        if (fragment == 255u) {
            return 0;
        } else if (fragment == 254u) {
            ending = true;
        } else if (ending) {
            return 0;
        } else {
            *plainchar++   |= (fragment & 0x03fu);
        }
    }

    if (i != length_in) {
        return 0;
    } else {
        return plainchar - plaintext_out;
    }
}

ptrdiff_t fcs_data_from_base64(uint8_t *restrict data, size_t datalen,
const uint8_t *restrict base64, size_t base64len) {
    assert(data);
    assert(datalen);
    assert(base64);
    assert(base64len);
    assert(datalen >= DATALEN_FROM_BASE64LEN(base64len));

    memset(data, 0xFFu, datalen);

    ptrdiff_t c = _fcs_base64_decode_block(base64, base64len, data);

    if (c == 0) {
        /*
        Invalid Base64 sequence; clear output so we're not tempted to use it
        anyway
        */
        memset(data, 0xFFu, datalen);
    }
    return c;
}
