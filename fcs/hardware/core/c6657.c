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
#include <stdbool.h>

#include <c6x.h>
#include "../../c66x-csl/ti/csl/cslr_device.h"
#include "../../c66x-csl/ti/csl/cslr_bootcfg.h"
#include "../../c66x-csl/ti/csl/cslr_gpio.h"
#include "../../c66x-csl/ti/csl/cslr_cgem.h"
#include "../../c66x-csl/ti/csl/cslr_msmc.h"
#include "../../c66x-csl/ti/csl/cslr_pllc.h"
#include "../../c66x-csl/ti/csl/cslr_emif4f.h"
#include "../../c66x-csl/ti/csl/cslr_psc.h"
#include "../../c66x-csl/ti/csl/cslr_xmc.h"
#include "../../c66x-csl/ti/csl/cslr_sem.h"

#include "../../config/config.h"
#include "../board.h"

static void _fcs_core_pll_setup(void);
static void _fcs_ddr3_pll_setup(void);
static void _fcs_ddr3_emif_setup(void);
static bool _fcs_ddr3_test(uint32_t nwords);
static void _fcs_enable_edc(void);
static uint32_t _fcs_init_core0(void);
static uint32_t _fcs_init_core1(void);

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

/*
Use the following for 1333MHz:
#define FCS_DDR3_PLL_MUL 79
#define FCS_DDR3_PLL_DIV 2
*/

static void _fcs_core_pll_setup(void) {
    assert(DNUM == 0);

    volatile CSL_PllcRegs *const pll = (CSL_PllcRegs*)CSL_PLL_CONTROLLER_REGS;
    volatile CSL_BootcfgRegs *const cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;

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
    cfg->CORE_PLL_CTL0 &= ~(0xFF000000);  /* Clear the BWADJ Field */
    cfg->CORE_PLL_CTL0 |=  ((temp << 24) & 0xFF000000);
    cfg->CORE_PLL_CTL1 &= ~(0x0000000F);   /* Clear the BWADJ field */
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
}

static void _fcs_ddr3_pll_setup(void) {
    assert(DNUM == 0);

    volatile CSL_BootcfgRegs *const cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;

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

    volatile CSL_BootcfgRegs *const cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;

    /* First, configure XMC MPAX to allow access to DDR3 config space */
    volatile CSL_XmcRegs *const xmc = (CSL_XmcRegs*)CSL_XMC_CONFIG_REGS;

    /* Replacement addr + permissions (0xFFu = R/W/X for all) */
    xmc->XMPAX[2].XMPAXL = 0x100000FFu;
    /* Base addr + seg size (64KB) */
    xmc->XMPAX[2].XMPAXH = 0x2100000Bu;

    volatile CSL_Emif4fRegs *const ddr3 =
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

    ddr3->SDRAM_TIM_1 = 0x1333780C;
    ddr3->SDRAM_TIM_2 = 0x30717FE3;
    ddr3->SDRAM_TIM_3 = 0x559F86AF;
    ddr3->DDR_PHY_CTRL_1 = 0x0010010F;
    ddr3->ZQ_CONFIG = 0x70074C1F;
    ddr3->PWR_MGMT_CTRL = 0;
    ddr3->SDRAM_CONFIG = 0x62477AB2;

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

/*
Check DDR3 data bus, address bus and memory integrity. Uses techniques from
http://www.esacademy.com/en/library/technical-articles-and-documents/\
miscellaneous/software-based-memory-testing.html
*/
static bool _fcs_ddr3_test(uint32_t nwords) {
    volatile uint32_t *const ddr3_mem = (volatile uint32_t*)0x80000000;
    uint32_t i, j, value;

    /*
    The DDR3 address range is non-cacheable by default, so no need to disable
    that here.
    */

    /* Walking 1s test to check data bus -- all at the start address */
    for (value = 1u; value != 0; value <<= 1u) {
        ddr3_mem[0] = value;
        if (ddr3_mem[0] != value) {
            return false;
        }
    }

    /* Address bus test -- walking 1s over the set of addresses */
    for (i = 1u; (i & (nwords - 1u)) != 0; i <<= 1u) {
        ddr3_mem[i] = 0xAAAAAAAAu;
    }

    /* Check for high address bits */
    ddr3_mem[0] = 0x55555555u;
    for (i = 1u; (i & (nwords - 1u)) != 0; i <<= 1u) {
        if (ddr3_mem[i] != 0xAAAAAAAAu) {
            return false;
        }
    }

    /* Check for low/shorted address bits */
    ddr3_mem[0] = 0xAAAAAAAAu;
    for (j = 1u; (j & (nwords - 1u)) != 0; j <<= 1u) {
        ddr3_mem[j] = 0x55555555u;
        if (ddr3_mem[0] != 0xAAAAAAAAu) {
            return false;
        }
        for (i = 1u; (i & (nwords - 1u)) != 0; i <<= 1u) {
            if (ddr3_mem[i] != 0xAAAAAAAAu && j != i) {
                return false;
            }
        }
        ddr3_mem[j] = 0xAAAAAAAAu;
    }

    /*
    Check data integrity -- try to stress the data bus by writing
    complementary patterns
    */
    for (i = 0; i < nwords; i++) {
        ddr3_mem[i] = (i & 1u) ? 0x55555555u : 0xAAAAAAAAu;
    }
    for (i = 0; i < nwords; i++) {
        if (ddr3_mem[i] != ((i & 1u) ? 0x55555555u : 0xAAAAAAAAu)) {
            return false;
        }
        ddr3_mem[i] = (i & 1u) ? 0xAAAAAAAAu : 0x55555555u;
    }
    for (i = 0; i < nwords; i++) {
        if (ddr3_mem[i] != ((i & 1u) ? 0xAAAAAAAAu : 0x55555555u)) {
            return false;
        }
        ddr3_mem[i] = (i & 1u) ? 0xFFFFFFFFu : 0x00000000u;
    }
    for (i = 0; i < nwords; i++) {
        if (ddr3_mem[i] != ((i & 1u) ? 0xFFFFFFFFu : 0x00000000u)) {
            return false;
        }
        ddr3_mem[i] = (i & 1u) ? 0x00000000u : 0xFFFFFFFFu;
    }
    for (i = 0; i < nwords; i++) {
        if (ddr3_mem[i] != ((i & 1u) ? 0x00000000u : 0xFFFFFFFFu)) {
            return false;
        }
    }

    return true;
}

static void _fcs_enable_edc(void) {
    volatile CSL_CgemRegs *const cgem =
        (CSL_CgemRegs*)CSL_CGEM0_5_LOCAL_L2_SRAM_REGS;

    /* L1P EDC enable */
    cgem->L1PEDCMD = 1u;
    assert(cgem->L1PEDSTAT);

    /* L2 EDC enable */
    cgem->L2EDCMD = 1u;
    assert(cgem->L2EDSTAT);

    cgem->L2EDCEN |= 0x1Fu;

    /* MSMC EDC enable -- clear SEN (bit 31) and set ECM (bit 30). */
    volatile CSL_MsmcRegs *const msmc = (CSL_MsmcRegs*)CSL_MSMC_CONFIG_REGS;
    msmc->SMEDCC &= 0x7FFFFFFFu;
    msmc->SMEDCC |= 0x40000000u;

    /* Check MSMC status */
    assert((msmc->SMEDCC >> 30u) == 1u);
}

uint32_t _fcs_init_core0(void) {
    volatile CSL_BootcfgRegs *const cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;

    KICK_UNLOCK();

    /*
    Perform PSC setup. Basically everything we care about is always-on, with
    the exception of the MSMC RAM on PD7/LPSC14, and maybe CorePac 1 on
    PD14/LPSC24.

    (See SPRS814A section 7.3.1 for details.)

    For now just power up the MSMC.
    */
    volatile CSL_PscRegs *const psc = (CSL_PscRegs*)CSL_PSC_REGS;

    /* Skip if MSMC module (14) state is already ENABLE (3) */
    if ((psc->MDSTAT[14] & 0x1Fu) != 3u) {
        /* Set power domain control for domain 7 */
        psc->PDCTL[7] |= 1u;

        /* Set MDCTL NEXT to ENABLE (3) */
        psc->MDCTL[14] = ((psc->MDCTL[14]) & ~(0x1f)) | 3u;

        /* Start transition on power domain 7 */
        psc->PTCMD |= 1u << 7u;

        /* Wait for PTSTAT.GOSTAT to clear */
        _fcs_delay_cycles(150u);
        assert((psc->PTSTAT & (1u << 7u)) == 0);

        /* Verify that the state has changed to ENABLE (3) */
        _fcs_delay_cycles(150u);
        assert((psc->MDSTAT[14] & 0x1Fu) == 3u);
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
    } while (!_fcs_ddr3_test(1048576) && tries < 10u);
    /* TODO: skip/reduce extent of memory test on restart */

    /*
    FIXME: allow DDR3 to fail for now since it's broken on the second
    prototype board.
    assert(tries < 10u);
    */


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
    volatile CSL_CgemRegs *const cgem =
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

    KICK_UNLOCK();

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

    Now, you'd think this would be accessible via cfg->CHIP_PIN_CONTROL_0,
    but somehow TI have messed up the offset in the CSL_BootcfgRegs, so we
    have to use the address directly. See this thread for details:
    http://e2e.ti.com/support/dsp/c6000_multi-core_dsps/f/639/t/215377.aspx
    */
    *(volatile uint32_t*)0x02620580 = 0x0CCF0000u;

    /*
    Each GPIO bank has DIR, OUT_DATA, SET_DATA, CLR_DATA, IN_DATA,
    SET_RIS_TRIG, CLR_RIS_TRIG, SET_FAL_TRIG and CLR_FAL_TRIG. Those registers
    are 32 bits wide, one bit per pin (31-0 maps to GPIO31-GPIO0). DATA is 0
    low, 1 high; DIR is 0 for output, and 1 for input.

    Clear bits 27, 26, 23 and 22 of DIR to set them as outputs, and then set
    the value to 0 to turn off the LEDs.
    */
    volatile CSL_GpioRegs *const gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;
    gpio->BANK_REGISTERS[0].DIR &= 0xF33FFFFFu;
    gpio->BANK_REGISTERS[0].OUT_DATA = 0x0CC00000u;

    /*
    Start booting core 1:
    - Populate BOOT_MAGIC_ADDRESS for CorePac 1, which should be the same as
      for CorePac 0.
    - Send an IPC interrupt to CorePac 1 (IPCGR1.IPCG) to wake it up.

    0x0087FFFCu is the boot magic address for the local core (it's at the end
    of L2 SRAM). Here, we convert that local address for CorePac 1's L2 SRAM
    to a global address, then copy the value of CorePac 0's boot magic address
    to it.
    */
    *(volatile uint32_t*)(GLOBAL_FROM_CORE_L2_ADDRESS(1u, 0x0087FFFCu)) =
        *(volatile uint32_t*)0x0087FFFCu;
    /*
    IPCGRn: IPC Generation Registers (section 3.3.12 in SPRS814A)

    Bit   Field          Value         Description
    31:4  SRCSx          0             Interrupt source indication
    3:1   Reserved
    0     IPCG           0             Generate an IPC interrupt
                                       0 = no interrupt
                                       1 = generate an interrupt
    */
    cfg->IPCGR[1] |= 0x1u;

    KICK_LOCK();

    return (FCS_CLOCK_HZ / FCS_CORE0_TICK_HZ);
}

uint32_t _fcs_init_core1(void) {
    /*
    Most of the platform config has been done by core 0. Here we just need to
    enable EDC on the local SRAMs.
    */
    volatile CSL_BootcfgRegs *const cfg = (CSL_BootcfgRegs*)CSL_BOOT_CFG_REGS;

    KICK_UNLOCK();
    _fcs_enable_edc();
    KICK_LOCK();

    return (FCS_CLOCK_HZ / FCS_CORE1_TICK_HZ);
}

uint32_t fcs_board_init_core(void) {
    uint8_t core = DNUM & 0xFFu;
    uint32_t result_cycles;

    /* Start TSC, if it's not already running */
    TSCL = 1u;

    if (core == 0) {
        result_cycles = _fcs_init_core0();
    } else if (core == 1u) {
        result_cycles = _fcs_init_core1();
    } else {
        assert(false);
    }

    /*
    Return the number of cycles per tick which should be run by this core
    */
    return result_cycles;
}
