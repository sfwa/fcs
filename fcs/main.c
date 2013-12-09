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

The PLL and DDR3 setup code is

Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the
  distribution.

  Neither the name of Texas Instruments Incorporated nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>

#include <c6x.h>
#include "c66x-csl/ti/csl/cslr_device.h"
#include "c66x-csl/ti/csl/cslr_bootcfg.h"
#include "c66x-csl/ti/csl/cslr_gpio.h"
#include "c66x-csl/ti/csl/cslr_cgem.h"
#include "c66x-csl/ti/csl/cslr_msmc.h"
#include "c66x-csl/ti/csl/cslr_pllc.h"
#include "c66x-csl/ti/csl/cslr_emif4f.h"
#include "c66x-csl/ti/csl/cslr_psc.h"
#include "c66x-csl/ti/csl/cslr_xmc.h"
#include "c66x-csl/ti/csl/cslr_sem.h"

#include "config/config.h"
#include "ahrs/ahrs.h"
#include "nmpc/nmpc.h"
#include "util/util.h"
#include "piksi/piksi.h"
#include "comms/comms.h"
#include "stats/stats.h"

static volatile CSL_BootcfgRegs *cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;
static volatile CSL_SemRegs *semaphore = (CSL_SemRegs*)CSL_SEMAPHORE_REGS;

static void _fcs_core_pll_setup(void);
static void _fcs_ddr3_pll_setup(void);
static void _fcs_ddr3_emif_setup(void);
static bool _fcs_ddr3_test(uint32_t start_idx, uint32_t nwords);
static void _fcs_enable_edc(void);

int main(void);
uint32_t fcs_main_init_core0(void);
uint32_t fcs_main_init_core1(void);
void fcs_main_init_common(void);

/*
Delay by a certain number of cycles. Note that if this is called before the
main PLL is configured, we'll only be running at 100MHz rather than 1GHz, so
delays should be reduced by a factor of 10.

TODO: maybe handle that automatically?
*/
static inline void _fcs_delay_cycles(uint32_t cycles) {
    uint32_t start_t = TSCL;
    while (TSCL - start_t < cycles);
}

/*
The boot config registers are protected by the kicker mechanism described on
page 72 of SPRS814A. The magic values 0x83E70B13 and 0x95A4F1E0 need to be
written to KICK0 and KICK1 respectively before modification can take place,
and then any other value must be written to either those registers to re-lock
the boot config registers.
*/
#define KICK_UNLOCK() { cfg->KICK_REG0 = 0x83E70B13; \
                        cfg->KICK_REG1 = 0x95A4F1E0; }
#define KICK_LOCK() { cfg->KICK_REG0 = 0; }

/* 1.0GHz clock from 100MHz input */
#define FCS_CORE_PLL_MUL 19
#define FCS_CORE_PLL_DIV 0

/* 667MHz clock from 50MHz input */
#define FCS_DDR3_PLL_MUL 39
#define FCS_DDR3_PLL_DIV 2

static void _fcs_core_pll_setup(void) {
    assert(DNUM == 0);

    volatile CSL_PllcRegs* pll = (CSL_PllcRegs*)CSL_PLL_CONTROLLER_REGS;

    /* 1. Wait for Stabilization time (min 100 us) */
    /* This will either be 100us or 1ms depending on PLL bypass status. */
    _fcs_delay_cycles(100000);

    /*
    2. Check the status of BYPASS bit in SECCTL register,
    execute following steps if
    BYPASS == 1 (if bypass enabled), if BYPASS==0 then Jump to Step 3
    */
    if (pll->SECCTL & 0x00800000) {
        /* PLL BYPASS is enabled, we assume if not in Bypass ENSAT = 1 */

        /*
        2a. Usage Note 9: For optimal PLL operation, the ENSAT bit in the PLL
        control registers for the Main PLL, DDR3 PLL, and PA PLL should be set
        to 1. The PLL initialization sequence in the boot ROM sets this bit to
        0 and could lead to non-optimal PLL operation. Software can set the
        bit to the optimal value of 1 after boot
        Ref: http://www.ti.com/lit/er/sprz334b/sprz334b.pdf
        |31...7   |6     |5 4       |3...0      |
        |Reserved |ENSAT |Reserved  |BWADJ[11:8]|
        */
        cfg->CORE_PLL_CTL1 = cfg->CORE_PLL_CTL1 | 0x00000040;

        /* 2b. Clear PLLEN bit (bypass enabled in PLL controller mux) */
        pll->PLLCTL &= ~(1 << 0);

        /*
        2c. Clear PLLENSRC bit (enable PLLEN to control PLL controller mux)
        */
        pll->PLLCTL &= ~(1 << 5);

        /*
        2d. Wait for 4 RefClks (to make sure the PLL controller mux switches
        properly to the bypass).
        Assuming 100Mhz RefClk, this is a 40ns delay
        */
        _fcs_delay_cycles(100); /* 100ns or 1us depending on freq */

        /*
        2e. Bypass needed to perform PWRDN cycle for C6670 and C6657
        Needed on all devices when in NOBOOT, I2C or SPI boot modes
        Ref: Figure 4-2 of http://www.ti.com/lit/ug/sprugv2a/sprugv2a.pdf
        PLL Secondary Control Register (SECCTL)  Layout
        |31...24  |23     |22...19       |18...0   |
        |Reserved |BYPASS |OUTPUT DIVIDE |Reserved |
        */
        pll->SECCTL |= 0x00800000; /* Set the Bit 23 */

        /*
        2f. Advisory 8: Multiple PLLs May Not Lock After Power-on Reset Issue
        In order to ensure proper PLL startup, the PLL power_down pin needs to
        be toggled. This is accomplished by toggling the PLLPWRDN bit in the
        PLLCTL register. This needs to be done before the main PLL
        initialization sequence
        Ref: Figure 4-1 of http://www.ti.com/lit/ug/sprugv2a/sprugv2a.pdf
        PLL Control Register (PLLCTL)  Layout
        |31...4   |3      |2        |1        |0        |
        |Reserved |PLLRST |Reserved |PLLPWRDN |Reserved |
        */
        pll->PLLCTL |= 0x00000002; /*Power Down the PLL */

        /*
        2g. Stay in a loop such that the bit is set for 5 µs (minimum) and
        then clear the bit.
        */
        _fcs_delay_cycles(5000); /* 5us or 50us depending on freq */

        /* 2h. Power up the PLL */
        pll->PLLCTL &= ~(0x00000002);
    } else {
        /* 3. Enable BYPASS in the PLL controller */
        /* 3a. Clear PLLEN bit (bypass enabled in PLL controller mux) */
        pll->PLLCTL &= ~(1 << 0);

        /*
        3b. Clear PLLENSRC bit (enable PLLEN to control PLL controller mux)
        */
        pll->PLLCTL &= ~(1 << 5);

        /*
        3c. Wait for 4 RefClks (to make sure the PLL controller mux switches
        properly to the bypass).
        Assuming 100Mhz RefClk, this is a 40ns delay
        */
        _fcs_delay_cycles(100); /* 100ns or 1us depending on freq */
    }

    /*
    4. Set the PLL Multiplier, Divider, BWADJ
    The PLLM[5:0] bits of the multiplier are controlled by the PLLM Register
    inside the PLL Controller and the PLLM[12:6] bits are controlled by the
    chip-level MAINPLLCTL0 Register.
    PLL Control Register (PLLM)  Layout
    |31...6   |5...0        |
    |Reserved |PLLM         |

    Main PLL Control Register (MAINPLLCTL0)
    |31...24   |23...19   |18...12    | 11...6   |5...0 |
    |BWADJ[7:0]| Reserved |PLLM[12:6] | Reserved | PLLD |
    */

    uint32_t temp;

    /* Set pll multipler (13 bit field) */
    pll->PLLM = (FCS_CORE_PLL_MUL & 0x0000003F); /* bits[5:0]  */
    temp = (FCS_CORE_PLL_MUL & 0x1FC0) >> 6; /* bits[12:6] */
    cfg->CORE_PLL_CTL0 &= ~(0x0007F000);                /*Clear PLLM field */
    cfg->CORE_PLL_CTL0 |= ((temp << 12) & 0x0007F000);

    /*
    5. Set the BWADJ     (12 bit field)
    BWADJ[11:8] and BWADJ[7:0] are located in MAINPLLCTL0 and MAINPLLCTL1
    registers. BWADJ[11:0] should be programmed to a value equal to half of
    PLLM[12:0] value (round down if PLLM has an odd value)
    Example: If PLLM = 15, then BWADJ = 7
    */
    temp = ((FCS_CORE_PLL_MUL + 1) >> 1) - 1; /* Divide the pllm by 2 */
    cfg->CORE_PLL_CTL0 &=~(0xFF000000);  /* Clear the BWADJ Field */
    cfg->CORE_PLL_CTL0 |=  ((temp << 24) & 0xFF000000);
    cfg->CORE_PLL_CTL1 &=~(0x0000000F);   /* Clear the BWADJ field */
    cfg->CORE_PLL_CTL1 |= ((temp >> 8) & 0x0000000F);

    /*
    6. Set the pll divider (6 bit field)
    PLLD[5:0] is located in MAINPLLCTL0
    */
    cfg->CORE_PLL_CTL0 &= ~(0x0000003F);    /* Clear the Field */
    cfg->CORE_PLL_CTL0 |= (FCS_CORE_PLL_DIV & 0x0000003F);

    /* 7. Set the OUTPUT DIVIDE (4 bit field) in SECCTL -- value is 1 */
    pll->SECCTL &= ~(0x00780000);     /* Clear the field */
    pll->SECCTL |= ((1 << 19) & 0x00780000) ;

    /* 8. Set PLL dividers if needed */
    uint32_t div2 = 3, div5 = 5, div8 = 64;
    pll->PLLDIV1_3[1] = (0x8000) | (div2 - 1);
    pll->PLLDIV4_16[1] = (0x8000) | (div5 - 1);
    pll->PLLDIV4_16[4] = (0x8000) | (div8 - 1);

    /* part of 8, Program ALNCTLn */
    /* Set bit 1, 4 and 7 */
    pll->ALNCTL |= ( (1 << 1) | (1 << 4) | (1 << 7));

    /*
    part of 8, Set GOSET bit in PLLCMD to initiate the GO operation to change
    the divide values and align the SYSCLKs as programmed
    */
    pll->PLLCMD |= 0x00000001;

    /* part of 8, go stat bit needs to be zero here */
    /*
    Read the GOSTAT bit in PLLSTAT to make sure the bit returns to 0 to
    indicate that the GO operation has completed
    */
    /* wait for the GOSTAT, but don't trap if lock is never read */
    uint32_t i;
    for (i = 0; i < 1000; i++) {
        if ((pll->PLLSTAT & 0x00000001) == 0) {
            break;
        }
    }
    /* Error if it doesn't lock */
    assert(i != 1000);

    /* 9. Place PLL in Reset, In PLLCTL, write PLLRST = 1 (PLL is reset) */
    pll->PLLCTL |= 0x00000008;

    /* 10. Wait for PLL Reset assertion Time (min: 7 us) */
    _fcs_delay_cycles(7000); /* 7us or 70us depending on freq */

    /* 11. In PLLCTL, write PLLRST = 0 (PLL reset is de-asserted) */
    pll->PLLCTL &= ~(0x00000008);

    /*
    12. PLL Lock Delay needs to be 500 RefClk periods * (PLLD + 1)
    i.e., Wait for at least 500 * CLKIN cycles * (PLLD + 1) (PLL lock time)
    */
    _fcs_delay_cycles(500 * 10 * (FCS_CORE_PLL_DIV + 1)); /* 5us delay */

    /*
    13. In SECCTL, write BYPASS = 0 (enable PLL mux to switch to PLL mode)
    */
    pll->SECCTL &= ~(0x00800000); /* Release Bypass */

    /*
    14. In PLLCTL, write PLLEN = 1 (enable PLL controller mux to switch to PLL
    mode)
    */
    pll->PLLCTL |= (1 << 0);

    /* 15. The PLL and PLL Controller are now initialized in PLL mode */

/*
    int dsp_freq;
    int dsp_freM,dsp_freD;

    // Compute the real dsp freq (*100)
    dsp_freq = (((REF_CLOCK_KHZ/10) * ((pll_mult+1)/2))/(pll_div+1));

    // Displayed frequency setup
    // dsp freq in MHz
    dsp_freM = dsp_freq / 100;

    // dsp freq first decimal if freq expressed in MHz
    dsp_freD = ((dsp_freq - dsp_freM * 100) + 5) / 10;

    // Add roundup unit to MHz displayed and reajust decimal value if
    // necessary...
    if (dsp_freD > 9)
    {
        dsp_freD = dsp_freD - 10;
        dsp_freM = dsp_freM + 1;
    }

    // Print freq info...
    GEL_TextOut("PLL1 Setup for DSP @ %d.%d MHz.\n",,,,,
                dsp_freM, dsp_freD );
    GEL_TextOut("           SYSCLK2 = %f MHz, SYSCLK5 = %f MHz.\n",,,,, (
                (float)(dsp_freq/100)/div2), ((float)(dsp_freq/100)/div5));
    GEL_TextOut("           SYSCLK8 = %f MHz.\n",,,,,
                ((float)(dsp_freq/100)/div8));
    GEL_TextOut("PLL1 Setup... Done.\n" );
*/
}

static void _fcs_ddr3_pll_setup(void) {
    assert(DNUM == 0);

    /*
    1. In DDR3PLLCTL1, write ENSAT = 1 (for optimal PLL operation)
    Usage Note 9: For optimal PLL operation, the ENSAT bit in the PLL control
    registers for the Main PLL, DDR3 PLL, and PA PLL should be set to 1.
    The PLL initialization sequence in the boot ROM sets this bit to 0 and
    could lead to non-optimal PLL operation. Software can set the bit to the
    optimal value of 1 after boot
    DDR3PLLCTL1 Bit map
    |31...7   |6     |5 4       |3...0      |
    |Reserved |ENSAT |Reserved  |BWADJ[11:8]|
    */
    cfg->DDR3_PLL_CTL1 |= 0x00000040;

    /*
    2. Put the PLL in PLL Mode
    DDR3PLLCTL0 Bit map
    |31...24    |23     |22...19       |18...6   |5...0 |
    |BWADJ[7:0] |BYPASS |Reserved      |PLLM     |PLLD  |
    */
    cfg->DDR3_PLL_CTL0 |= 0x00800000; /* Set the Bit 23 */

    /*
    NOTE: moved before step 3 for consistency with evm6657.c in the EVM PDK.

    5. In PLL Controller, reset the PLL (bit 13 in DDR3PLLCTL1 register)
    */
    cfg->DDR3_PLL_CTL1 |= 0x00002000;

    /* 3. Program the necessary multipliers/dividers and BW adjustments */
    /* part of 3, Set the Multipler values */
    cfg->DDR3_PLL_CTL0 &= ~(0x0007FFC0);
    cfg->DDR3_PLL_CTL0 |= ((FCS_DDR3_PLL_MUL << 6) & 0x0007FFC0 );
    /* part of 3, Set the divider values */
    cfg->DDR3_PLL_CTL0 &= ~(0x0000003F);
    cfg->DDR3_PLL_CTL0 |= (FCS_DDR3_PLL_DIV & 0x0000003F);

    /* 4. Set the BWADJ */
    uint32_t temp = ((FCS_DDR3_PLL_MUL + 1) >> 1) - 1;
    cfg->DDR3_PLL_CTL0 &= ~(0xFF000000);
    cfg->DDR3_PLL_CTL0 |= ((temp << 24) & 0xFF000000);
    cfg->DDR3_PLL_CTL1 &= ~(0x0000000F);
    cfg->DDR3_PLL_CTL1 |= ((temp >> 8) & 0x0000000F);

    /* 6. Wait for the PLL Reset time (min: 5 us)  */
    _fcs_delay_cycles(7000); /* 7us delay */

    /* 7. In DDR3PLLCTL1, write PLLRST = 0 to bring PLL out of reset */
    cfg->DDR3_PLL_CTL1 &= ~(0x00002000);

    /*
    8. Wait at least 500 * REFCLK cycles * PLLD (this is the PLL lock time)
    */
    _fcs_delay_cycles(70000); /* 70us delay */

    /*
    9. Put the PLL in PLL Mode
    DDR3PLLCTL0 Bit map
    |31...24    |23     |22...19       |18...6   |5...0 |
    |BWADJ[7:0] |BYPASS |Reserved      |PLLM     |PLLD  |
    */
    cfg->DDR3_PLL_CTL0 &= ~(0x00800000); /* Clear bit 23 */
}

static void _fcs_ddr3_emif_setup(void) {
    assert(DNUM == 0);

    /* First, configure XMC MPAX to allow access to DDR3 config space */
    volatile CSL_XmcRegs* xmc = (CSL_XmcRegs*)CSL_XMC_CONFIG_REGS;

    /* Replacement addr + permissions (0xFFu = R/W/X for all) */
    xmc->XMPAX[2].XMPAXL = 0x100000FFu;
    /* Base addr + seg size (64KB) */
    xmc->XMPAX[2].XMPAXH = 0x2100000Bu;

    volatile CSL_Emif4fRegs* ddr3 =
        (CSL_Emif4fRegs*)CSL_DDR3_EMIF_CONFIG_REGS;

    /*
    DDR3 setup -- same as on the EVM6657L. Unfortunately cslr_bootcfg.h only
    provides a DDR3_CONFIG_REG[64] member for access to the necessary
    registers, while the platform file defines them all by address.

    Based on the relative offsets of OBSCLKCTL/OBSCLK_CTL (presumably the same
    thing), and Table 4-2 in SPRUGV8C, I believe the mapping is as follows:

    EVM6657 name                        DDR3_CONFIG_REG index
    DATA0_GTLVL_INIT_RATIO              14
    DATA1_GTLVL_INIT_RATIO              15
    DATA2_GTLVL_INIT_RATIO              16
    DATA3_GTLVL_INIT_RATIO              17
    DATA4_GTLVL_INIT_RATIO              18
    DATA5_GTLVL_INIT_RATIO              19
    DATA6_GTLVL_INIT_RATIO              20
    DATA7_GTLVL_INIT_RATIO              21
    DATA8_GTLVL_INIT_RATIO              22
    DATA0_WRLVL_INIT_RATIO              2
    DATA1_WRLVL_INIT_RATIO              3
    DATA2_WRLVL_INIT_RATIO              4
    DATA3_WRLVL_INIT_RATIO              5
    DATA4_WRLVL_INIT_RATIO              6
    DATA5_WRLVL_INIT_RATIO              7
    DATA6_WRLVL_INIT_RATIO              8
    DATA7_WRLVL_INIT_RATIO              9
    DATA8_WRLVL_INIT_RATIO              10
    DDR3_CONFIG_REG_0                   0
    DDR3_CONFIG_REG_12                  12
    DDR3_CONFIG_REG_23                  23
    DDR3_CONFIG_REG_24                  24

    This DDR3 config code is adapted from the EVM6657 GEL file; I haven't
    documented the how/what/why as much as elsewhere because it Definitely
    Works on our board and I don't want to have to understand it.
    */

    /*
    Wait for PLL to lock = min 500 ref clock cycles.
    With refclk = 100MHz, = 5000 ns = 5us.
    */
    _fcs_delay_cycles(7000); /* Actually a 7us delay */

    /**************** 3.2 DDR3 PLL Configuration ****************************/
    /* Done before */

    /**************** 3.0 Leveling Register Configuration *******************/
    /* Using partial automatic leveling due to errata */

    /**************** 3.3 Leveling register configuration *******************/
    /* clear ctrl_slave_ratio field */
    cfg->DDR3_CONFIG_REG[0] &= ~(0x007FE000);
    /* set ctrl_slave_ratio to 0x100 */
    cfg->DDR3_CONFIG_REG[0] |= 0x00200000;
    /* Set invert_clkout = 1 */
    cfg->DDR3_CONFIG_REG[12] |= 0x08000000;
    /* set dll_lock_diff to 15 */
    cfg->DDR3_CONFIG_REG[0] |= 0xF;

    /* From 4.2.1 Executing Partial Automatic Leveling -- Start */
    /* Set bit 9 = 1 to use forced ratio leveling for read DQS */
    cfg->DDR3_CONFIG_REG[23] |= 0x00000200;
    /* From 4.2.1 Executing Partial Automatic Leveling -- End */

    /* Values with invertclkout = 1 */
    /**************** 3.3 Partial Automatic Leveling ************************/
    cfg->DDR3_CONFIG_REG[2] = 0;
    cfg->DDR3_CONFIG_REG[3] = 0;
    cfg->DDR3_CONFIG_REG[4] = 0;
    cfg->DDR3_CONFIG_REG[5] = 0;
    cfg->DDR3_CONFIG_REG[6] = 0x33;
    cfg->DDR3_CONFIG_REG[7] = 0x3A;
    cfg->DDR3_CONFIG_REG[8] = 0x2C;
    cfg->DDR3_CONFIG_REG[9] = 0x2C;
    cfg->DDR3_CONFIG_REG[10] = 0x21;

    cfg->DDR3_CONFIG_REG[14] = 0;
    cfg->DDR3_CONFIG_REG[15] = 0;
    cfg->DDR3_CONFIG_REG[16] = 0;
    cfg->DDR3_CONFIG_REG[17] = 0;
    cfg->DDR3_CONFIG_REG[18] = 0xB7;
    cfg->DDR3_CONFIG_REG[19] = 0xB1;
    cfg->DDR3_CONFIG_REG[20] = 0xA4;
    cfg->DDR3_CONFIG_REG[21] = 0xA4;
    cfg->DDR3_CONFIG_REG[22] = 0x98;

    /* Do a PHY reset. Toggle DDR_PHY_CTRL_1 bit 15 0->1->0 */
    ddr3->DDR_PHY_CTRL_1 &= ~(0x00008000);
    ddr3->DDR_PHY_CTRL_1 |= (0x00008000);
    ddr3->DDR_PHY_CTRL_1 &= ~(0x00008000);

    /***************** 3.4 Basic Controller and DRAM Configuration **********/
    ddr3->SDRAM_REF_CTRL = 0x0000515C;    /* enable configuration */

    uint32_t temp;

    /* DDR_SDTIM1 = 0x1557B9BD; */
    temp = 0;
    temp |= 0x09 << 25; /* T_RP bit field 28:25 */
    temp |= 0x09 << 21; /* T_RCD bit field 24:21 */
    temp |= 0x09 << 17; /* T_WR bit field 20:17 */
    temp |= 0x17 << 12; /* T_RAS bit field 16:12 */
    temp |= 0x20 << 6; /* T_RC bit field 11:6 */
    temp |= 0x1 << 3; /* T_RRD bit field 5:3 */
    temp |= 0x4; /* T_WTR bit field 2:0 */
    ddr3->SDRAM_TIM_1 = temp;

    /* DDR_SDTIM2 = 0x304F7FE3; */
    temp = 0;
    temp |= 0x3 << 28; /* T_XP bit field 30:28 */
    temp |= 0x71 << 16; /* T_XSNR bit field 24:16 */
    temp |= 0x1ff << 6; /* T_XSRD bit field 15:6 */
    temp |= 0x4 << 3; /* T_RTP bit field 5:3 */
    temp |= 0x3; /* T_CKE bit field 2:0 */
    ddr3->SDRAM_TIM_2 = temp;

    /* DDR_SDTIM3 = 0x559F849F; */
    temp = 0;
    temp |= 0x5 << 28; /* T_PDLL_UL bit field 31:28 (fixed value) */
    temp |= 0x5 << 24; /* T_CSTA bit field 27:24 (fixed value) */
    temp |= 0x4 << 21; /* T_CKESR bit field 23:21 */
    temp |= 0x3f << 15; /* T_ZQCS bit field 20:15 */
    temp |= 0x6A << 4; /* T_RFC bit field 12:4 */
    temp |= 0xf; /* T_RAS_MAX bit field 3:0 (fixed value) */
    ddr3->SDRAM_TIM_3 = temp;

    ddr3->DDR_PHY_CTRL_1 = 0x0010010F;

    ddr3->ZQ_CONFIG = 0x70074c1f;

    ddr3->PWR_MGMT_CTRL = 0x0;

    /* DDR_SDCFG = 0x63077AB3; */
    /* New value with DYN_ODT disabled and SDRAM_DRIVE = RZQ/7
       //0x63222A32;    // last config write DRAM init occurs */
    temp = 0;
    temp |= 0x3 << 29; /* SDRAM_TYPE bit field 31:29 (fixed value) */
    temp |= 0x0 << 27; /* IBANK_POS bit field 28:27 */
    temp |= 0x2 << 24; /* DDR_TERM bit field 26:24 */
    temp |= 0x2 << 21; /* DYN_ODT bit field 22:21 */
    temp |= 0x1 << 18; /* SDRAM_DRIVE bit field 19:18 */
    temp |= 0x3 << 16; /* CWL bit field 17:16 */
    temp |= 0x1 << 14; /* NM bit field 15:14 */
    temp |= 0xE << 10; /* CL bit field 13:10 */
    temp |= 0x5 << 7; /* ROWSIZE bit field 9:7 */
    temp |= 0x3 << 4; /* IBANK bit field 6:4 */
    temp |= 0x0 << 3; /* EBANK bit field 3:3 */
    temp |= 0x2; /* PAGESIZE bit field 2:0 */
    ddr3->SDRAM_CONFIG = temp;

    /* Wait 600us for HW init to complete */
    _fcs_delay_cycles(600000);

    ddr3->SDRAM_REF_CTRL = 0x0000144F;       /* Refresh rate = (7.8*666MHz) */

    /**************** 4.2.1 Executing Partial Automatic Leveling ************/

    ddr3->RDWR_LVL_RMP_CTRL = 0x80000000; /* enable full leveling */

    ddr3->RDWR_LVL_CTRL = 0x80000000; /* Trigger full leveling - This ignores
                                         read DQS leveling result and uses
                                         ratio forced value */

    /* (0x34) instead */
    /*
    Wait for min 1048576 DDR clock cycles for leveling to complete =
    1048576 * 1.5ns = 1572864ns = 1.57ms.
    */
    _fcs_delay_cycles(5000000); /* 5ms delay */
}

static bool _fcs_ddr3_test(uint32_t start_idx, uint32_t nwords) {
    volatile uint32_t *ddr3_mem = (volatile uint32_t*)0x80000000;
    uint32_t i, value;

    /* Write a pattern */
    for (i = start_idx; i < start_idx + nwords; i++) {
        ddr3_mem[i] = i + 0xAAAAAAAAu;
    }

    /* Read and check the pattern */
    for (i = start_idx; i < start_idx + nwords; i++) {
        value = ddr3_mem[i];

        if (value != i + 0xAAAAAAAAu) {
            return false;
        }
    }

    /* Write a pattern for complementary values */
    for (i = start_idx; i < start_idx + nwords; i++) {
        ddr3_mem[i] = ~(i + 0xAAAAAAAAu);
    }

    /* Read and check the pattern */
    for (i = start_idx; i < start_idx + nwords; i++) {
        value = ddr3_mem[i];

        if (value != ~(i + 0xAAAAAAAAu)) {
            return false;
        }
    }

    return true;
}

static void _fcs_enable_edc(void) {
    volatile CSL_CgemRegs* cgem =
        (CSL_CgemRegs*)CSL_CGEM0_5_LOCAL_L2_SRAM_REGS;

    /* L1P EDC enable */
    cgem->L1PEDCMD = 1u;
    assert(cgem->L1PEDSTAT);

    /* L2 EDC enable */
    cgem->L2EDCMD = 1u;
    assert(cgem->L2EDSTAT);

    cgem->L2EDCEN |= 0x1Fu;

    /* MSMC EDC enable -- clear SEN (bit 31) and set ECM (bit 30). */
    volatile CSL_MsmcRegs* msmc = (CSL_MsmcRegs*)CSL_MSMC_CONFIG_REGS;
    msmc->SMEDCC &= 0x7FFFFFFFu;
    msmc->SMEDCC |= 0x40000000u;

    /* Check MSMC status */
    assert((msmc->SMEDCC >> 30u) == 1u);
}

uint32_t fcs_main_init_core0(void) {
    KICK_UNLOCK();

    /*
    Perform PSC setup. Basically everything we care about is always-on, with
    the exception of the MSMC RAM on PD7/LPSC14, and maybe CorePac 1 on
    PD14/LPSC24.

    (See SPRS814A section 7.3.1 for details.)

    For now just power up the MSMC.
    */
    volatile CSL_PscRegs* psc = (CSL_PscRegs*)CSL_PSC_REGS;

    /* Skip if MSMC module (14) state is already ENABLE (3) */
    if ((psc->MDSTAT[14] & 0x1Fu) != 3u) {
        /* Set power domain control for domain 7 */
        psc->PDCTL[7] |= 1u;

        /* Set MDCTL NEXT to ENABLE (3) */
        psc->MDCTL[14u] = ((psc->MDCTL[14u]) & ~(0x1f)) | 3u;

        /* Start transition on power domain 7 */
        psc->PTCMD |= 1u << 7u;

        /* Wait for PTSTAT.GOSTAT to clear */
        _fcs_delay_cycles(150u);
        assert((psc->PTSTAT & (1u << 7u)) == 0);

        /* Verify that the state has changed to ENABLE (3) */
        _fcs_delay_cycles(150u);
        assert((psc->MDSTAT[14] & 0x1Fu) == 3u );
    }

    /*
    This stuff is in a loop because that's how the EVM6657 GEL and evm6657.c
    init scripts do it. No, it doesn't seem like a good idea to me either.
    */
    uint8_t tries = 0;
    do {
        tries++;

        /*
        PLL setup -- same as on the EVM6657, based on code from the GEL file.
        */
        _fcs_core_pll_setup();
        _fcs_ddr3_pll_setup();

        /*
        Set up the DDR3 EMIF, including read/write levelling (same as EVM6657
        GEL)
        */
        _fcs_ddr3_emif_setup();

        /* And wait for the DDR3 test to pass... */
    } while (!_fcs_ddr3_test(0, 1024) && tries < 10u);

    assert(tries < 10u);


    /*
    Clear PFX bits in MAR registers to work around Advisory 14 in SPRZ381A.

    If we don't do this, we might get read corruption when accessing MSMC SRAM
    as we don't use L2 SRAM as cache. This workaround disables prefetching
    for MSMC, but leaves caching (in L1D) enabled.

    Address range affected is 0x0C000000 - 0x0FFFFFFF, so MARs 12-15 need to
    be updated.

    MARn: Memory Attribute Register (SPRUGW0C section 4.4.5)

    Bit   Field          Value         Description
    31:4  Reserved
    3     PFX                          Enable/disable prefetchability of the
                                       address range.
                                       0 = not prefetchable
                                       1 = prefetchable
    2     Reserved
    1     Reserved
    0     PC                           Enable/disable cacheability of the
                                       address range.
                                       0 = not cacheable
                                       1 = cacheable
    */
    volatile CSL_CgemRegs* cgem =
        (CSL_CgemRegs*)CSL_CGEM0_5_LOCAL_L2_SRAM_REGS;
    cgem->MAR[12] = 0x1u;
    cgem->MAR[13] = 0x1u;
    cgem->MAR[14] = 0x1u;
    cgem->MAR[15] = 0x1u;

    /*
    Now that the MSMC cache issue has been worked around, enable error
    detection and correction.
    */
    _fcs_enable_edc();

    /*
    EMIF16 fix recommended here:
    http://e2e.ti.com/support/dsp/c6000_multi-core_dsps/f/639/t/248412.aspx

    Also see Usage Note 17 in SPRZ381A.
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

    KICK_LOCK();

    return (FCS_CLOCK_HZ / FCS_CORE0_TICK_HZ);
}

uint32_t fcs_main_init_core1(void) {
    /*
    Most of the platform config has been done by core 0. Here we just need to
    wait until core 0 is done, then enable EDC on the local SRAMs.

    Query core 0's boot semaphore until it's released, then start ourselves.
    If we reach 100000000 cycles (0.1s or 1s depending on PLL), abort.
    */
    uint32_t start_t = TSCL;
    while (!semaphore->QSEM[FCS_SEMAPHORE_CORE0_BOOT] &&
           TSCL - start_t < 100000000u);
    assert(TSCL - start_t < 100000000u);

    KICK_UNLOCK();
    _fcs_enable_edc();
    KICK_LOCK();

    return (FCS_CLOCK_HZ / FCS_CORE1_TICK_HZ);
}

void fcs_main_init_common(void) {
    /* Start TSC, if it's not already running */
    TSCL = 1u;
}

#pragma FUNC_NEVER_RETURNS(main);
int main(void) {
    /* Perform common initialization */
    fcs_main_init_common();

    uint32_t core = DNUM & 0xFFu,
             cycles_per_tick = 0,
             start_t = TSCL;

    /* Wait for the semaphore module to be ready, or for 1us/10us to elapse */
    while (!(semaphore->SEM_RST_RUN & 1u) && TSCL - start_t < 1000u);
    assert(TSCL - start_t < 1000u);

    /*
    Acquire the boot semaphore for our core by reading it. See SPRUGS3A for
    details on how this process works, but basically if the read returns 1
    we've got it.

    For some reason semaphore 0 is always busy, so use 1-31.
    */
    uint32_t sem_val = semaphore->SEM[core + 1];
    assert(sem_val == 1u);

    /* Wait a little to make sure both semaphores have been acquired */
    _fcs_delay_cycles(1000u); /* 1us to 10us depending on PLL state */

    /* Perform core-specific initialization */
    if (core == 0u) {
        cycles_per_tick = fcs_main_init_core0();
    } else if (core == 1u) {
        cycles_per_tick = fcs_main_init_core1();
    } else {
        assert(0);
    }

    /* Release the boot semaphore by writing 1 back to the register */
    semaphore->SEM[core + 1] = 1u;

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

        /* Wait until next frame start time */
        start_t = frame * cycles_per_tick;
        frame++;

        fcs_global_counters.main_loop_count[core]++;
        if (TSCL - start_t > fcs_global_counters.main_loop_cycle_max[core]) {
            fcs_global_counters.main_loop_cycle_max[core] = TSCL - start_t;
        }

        if (TSCL - start_t > cycles_per_tick) {
            /* Lost an entire frame! */
            assert(0);
        } else {
            /* FIXME: use idle/sleep instead of busy loop */
            while (TSCL - start_t < cycles_per_tick);
        }
    }

    return 0;
}
