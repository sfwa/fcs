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
#include <stddef.h>

#include <c6x.h>
#include "c66x-csl/ti/csl/cslr_device.h"
#include "c66x-csl/ti/csl/cslr_bootcfg.h"
#include "c66x-csl/ti/csl/cslr_gpio.h"

#include "config/config.h"
#include "ahrs/ahrs.h"
#include "nmpc/nmpc.h"
#include "util/util.h"
#include "piksi/piksi.h"
#include "comms/comms.h"
#include "stats/stats.h"

int main(void);
uint32_t fcs_main_init_core0(void);
uint32_t fcs_main_init_core1(void);
void fcs_main_init_common(void);

uint32_t fcs_main_init_core0(void) {
    /*
    EMIF16 fix recommended here:
    http://e2e.ti.com/support/dsp/c6000_multi-core_dsps/f/639/t/248412.aspx
    */
    *(uint32_t*)0x20C00008 |= 0x80000000;

    /*
    Set UART CTS/RTS and TIMI/O pins as GPIOs
    CHIP_PIN_CONTROL_0: Pin Control 0 (section 3.3.20 in SPRS814A)

    Bit   Field          Value         Description
    31    GPIO31_SPIDOUT_MUX           SPI or GPIO mux control
                                       0 = SPIDOUT enabled
                                       1 = GPIO31 enabled
    30    GPIO30_SPIDIN_MUX            SPI or GPIO mux control
                                       0 = SPIDIN enabled
                                       1 = GPIO30 enabled
    29    GPIO29_SPICS1_MUX            SPI or GPIO mux control
                                       0 = SPICS1 enabled
                                       1 = GPIO29 enabled
    28    GPIO28_SPICS0_MUX            SPI or GPIO mux control
                                       0 = SPICS0 enabled
                                       1 = GPIO28 enabled
    27    GPIO27_UARTRTS1_MUX          UART or GPIO mux control
                                       0 = UARTRTS1 enabled
                                       1 = GPIO27 enabled
    26    GPIO26_UARTCTS1_MUX          UART or GPIO mux control
                                       0 = UARTCTS1 enabled
                                       1 = GPIO26 enabled
    25    GPIO25_UARTTX1_MUX           UART or GPIO mux control
                                       0 = UARTTX1 enabled
                                       1 = GPIO25 enabled
    24    GPIO24_UARTRX1_MUX           UART or GPIO mux control
                                       0 = UARTRX1 enabled
                                       1 = GPIO24 enabled
    23    GPIO23_UARTRTS0_MUX          UART or GPIO mux control
                                       0 = UARTRTS0 enabled
                                       1 = GPIO23 enabled
    22    GPIO22_UARTCTS0_MUX          UART or GPIO mux control
                                       0 = UARTCTS0 enabled
                                       1 = GPIO22 enabled
    21    GPIO21_UARTTX0_MUX           UART or GPIO mux control
                                       0 = UARTTX0 enabled
                                       1 = GPIO21 enabled
    20    GPIO20_UARTRX0_MUX           UART or GPIO mux control
                                       0 = UARTRX0 enabled
                                       1 = GPIO20 enabled
    19    GPIO19_TIMO1_MUX             TIMER or GPIO mux control
                                       0 = TIMO1 enabled
                                       1 = GPIO19 enabled
    18    GPIO18_TIMO0_MUX             TIMER or GPIO mux control
                                       0 = TIMO0 enabled
                                       1 = GPIO18 enabled
    17    GPIO17_TIMI1_MUX             TIMER or GPIO mux control
                                       0 = TIMI1 enabled
                                       1 = GPIO17 enabled
    16    GPIO16_TIMI0_MUX             TIMER or GPIO mux control
                                       0 = TIMI0 enabled
                                       1 = GPIO16 enabled
    15:0  Reserved

    We want GPIO27_UARTRTS1_MUX, GPIO26_UARTCTS1_MUX, GPIO23_UARTRTS0_MUX,
    GPIO22_UARTCTS0_MUX, GPIO19_TIMO1_MUX, GPIO18_TIMO0_MUX, GPIO17_TIMI1_MUX,
    and GPIO16_TIMI0_MUX high.
    */
    volatile CSL_BootcfgRegs *cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;
    cfg->CHIP_PIN_CONTROL_0 = 0x0CCF0000u;

    /*
    Each GPIO bank has DIR, OUT_DATA, SET_DATA, CLR_DATA, IN_DATA,
    SET_RIS_TRIG, CLR_RIS_TRIG, SET_FAL_TRIG and CLR_FAL_TRIG. Those registers
    are 32 bits wide, one bit per pin (31-0 maps to GPIO31-GPIO0). DATA is 0
    low, 1 high; DIR is 0 for output, and 1 for input.

    Clear bits 27, 26, 23 and 22 of DIR to set them as outputs, and then set
    the value to 0 to turn off the LEDs.
    */
    volatile CSL_GpioRegs* gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;
    gpio->BANK_REGISTERS[0].DIR &= 0xF33FFFFFu;
    gpio->BANK_REGISTERS[0].OUT_DATA = 0x0CC00000u;

    return (FCS_CLOCK_HZ / FCS_CORE0_TICK_HZ);
}

uint32_t fcs_main_init_core1(void) {
    return (FCS_CLOCK_HZ / FCS_CORE1_TICK_HZ);
}

void fcs_main_init_common(void) {
    /* Start TSC, if it's not already running */
    TSCL = 1u;
}

#pragma FUNC_NEVER_RETURNS(main);
int main(void) {
    fcs_main_init_common();

    uint32_t core = DNUM & 0xFFu;
    uint32_t cycles_per_tick = 0;

    if (core == 0u) {
    	cycles_per_tick = fcs_main_init_core0();
    } else if (core == 1u) {
    	cycles_per_tick = fcs_main_init_core1();
    } else {
    	assert(0);
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

        fcs_global_counters.main_loop_count++;
        if (TSCL - start_t > fcs_global_counters.main_loop_cycle_max) {
            fcs_global_counters.main_loop_cycle_max = TSCL - start_t;
        }

        /* FIXME: use idle/sleep instead of busy loop */
        while (TSCL - start_t < cycles_per_tick);

        if (TSCL - start_t > (5u * cycles_per_tick) >> 2u) {
            /* Lost an entire frame! */
            assert(0);
        }
    }

	return 0;
}
