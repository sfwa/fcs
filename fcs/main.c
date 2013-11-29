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
#include <assert.h>
#include <c6x.h>
#include <ti/csl/csl.h>
#include <ti/csl/csl_chip.h>

#include "config/config.h"
#include "ahrs/ahrs.h"
#include "nmpc/nmpc.h"
#include "util/util.h"
#include "piksi/piksi.h"
#include "comms/comms.h"

int main(void);
uint32_t fcs_main_init_core0(void);
uint32_t fcs_main_init_core1(void);
void fcs_main_init_common(void);

uint32_t fcs_main_init_core0(void) {
    /*
    TODO: we're done with the NOR flash, so set the CPLD GPIO to return the
    SPI bus to the CPU
    */

    return (FCS_CLOCK_HZ / FCS_CORE0_TICK_HZ);
}

uint32_t fcs_main_init_core1(void) {
    return (FCS_CLOCK_HZ / FCS_CORE1_TICK_HZ);
}

void fcs_main_init_common(void) {
    /* Start TSC, if it's not already running */
    TSCL = 1u;
}

int main(void) {
    fcs_main_init_common();

    uint32_t core = CSL_chipReadReg(CSL_CHIP_DNUM);
    uint32_t cycles_per_tick = 0;

    if (core == 0u) {
        fcs_main_init_core0();
    } else if (core == 1u) {
        fcs_main_init_core1();
    }

    if (core == FCS_CORE_CONFIG) {
        fcs_config_init();
    }
    if (core == FCS_CORE_UTIL) {
        fcs_util_init();
    }
    if (core == FCS_CORE_COMMS) {
        fcs_comms_init();
    }
    if (core == FCS_CORE_PIKSI) {
        fcs_piksi_init();
    }
    if (core == FCS_CORE_AHRS) {
        fcs_ahrs_init();
    }
    if (core == FCS_CORE_NMPC) {
        fcs_nmpc_init();
    }

    uint32_t frame = TSCL / cycles_per_tick;
    while (1) {
        if (core == FCS_CORE_COMMS) {
            fcs_comms_tick();
        }
        if (core == FCS_CORE_CONFIG) {
            fcs_config_tick();
        }
        if (core == FCS_CORE_PIKSI) {
            fcs_piksi_tick();
        }
        if (core == FCS_CORE_AHRS) {
            fcs_ahrs_tick();
        }
        if (core == FCS_CORE_NMPC) {
            fcs_nmpc_tick();
        }

        /*
        Wait until next frame start time
        */
        uint32_t start_t = frame * cycles_per_tick;
        frame++;

        /* FIXME: use idle/sleep instead of busy loop */
        while (TSCL - start_t < cycles_per_tick);

        if (TSCL - start_t > (5u * cycles_per_tick) >> 2u) {
            /* Lost an entire frame! */
            assert(0);
        }
    }

	return 0;
}
