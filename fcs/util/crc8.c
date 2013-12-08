/*
 * Copyright (c) 2011 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

#include "util.h"

static uint8_t crc_lookup[256];
static bool crc_inited;

/*
 * crc8_init - fill crc table for given polynomial in regular bit order.
 *
 * polynomial:  polynomial for which table is to be filled.
 */
void fcs_crc8_init(uint8_t polynomial) {
    uint32_t i, j;
    uint8_t t = 0;

    #pragma MUST_ITERATE(256,256)
    for (i = 0; i < 256u; i++) {
        t = i;
        #pragma MUST_ITERATE(8,8)
        for (j = 0; j < 8u; j++) {
            t = (t << 1u) ^ ((t & 0x80u) ? polynomial : 0);
        }
        crc_lookup[i] = t;
    }

    crc_inited = true;
}

/*
 * crc8 - calculate a crc8 over the given input data.
 *
 * pdata: pointer to data buffer.
 * nbytes: number of bytes in data buffer.
 * crc: previous returned crc8 value.
 */
uint8_t fcs_crc8(const uint8_t *restrict pdata, uint32_t nbytes,
uint8_t crc) {
    uint32_t i;

    assert(nbytes && nbytes <= 256u);
    assert(pdata);
    assert(crc_inited);

    /* loop over the buffer data */
    #pragma MUST_ITERATE(1,256)
    for (i = 0; i < nbytes; i++) {
        crc = crc_lookup[(crc ^ pdata[i]) & 0xffu];
    }

    return crc;
}
