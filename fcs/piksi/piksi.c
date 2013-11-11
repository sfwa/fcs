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
#include <stdlib.h>
#include <assert.h>

#include "../config/config.h"
#include "../nmpc/nmpc.h"
#include "piksi.h"

void fcs_piksi_init(void) {
    /*
    Ensure this isn't called more than once, and the initial configuration is
    good
    */
    fcs_config_t current_config;
    current_config = fcs_config_get_current();
    assert(fcs_nmpc_validate_config(current_config) == FCS_CONFIG_OK);
    fcs_piksi_load_config(current_config);

    /* TODO */
}

void fcs_piksi_tick(void) {
    /* TODO */
}

enum fcs_config_validation_result_t fcs_piksi_validate_config(
fcs_config_t new_config) {
    /* TODO */

    return FCS_CONFIG_OK;
}

void fcs_piksi_load_config(fcs_config_t new_config) {
    assert(new_config);

    /* TODO */
}
