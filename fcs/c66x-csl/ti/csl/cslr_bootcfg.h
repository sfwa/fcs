/********************************************************************
 * Copyright (C) 2003-2011 Texas Instruments Incorporated.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#ifndef _CSLR_BOOTCFG_H_
#define _CSLR_BOOTCFG_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Header files are included as per RTSC guidelines
 *      b) Changed BOOTADDR_GEM0_REG-BOOTADDR_GEM3_REG to an array BOOTADDR_GEM_REG[4]
 *      c) Changed RSTMUX0-RSTMUX7 to an array RSTMUX[8]
 *      d) Changed NMIGR_0-NMIGR_3 to an array NMIGR[4]
 *      e) Changed IPCGR0-IPCGR3 to an array IPCGR[4]
 *      f) Changed IPCAR0-IPCAR3 to an array IPCAR[4]
 *      g) Changed DDR3_CONFIG_REG_0-DDR3_CONFIG_REG_60 to an array DDR3_CONFIG_REG[61] - note that there are some gaps
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 REVISION_REG;
    volatile Uint8 RSVD0[4];
    volatile Uint32 DIE_ID_REG0;
    volatile Uint32 DIE_ID_REG1;
    volatile Uint32 DIE_ID_REG2;
    volatile Uint32 DIE_ID_REG3;
    volatile Uint32 DEVICE_ID_REG0;
    volatile Uint32 DEVICE_ID_REG1;
    volatile Uint32 BOOT_REG0;
    volatile Uint8 RSVD1[20];
    volatile Uint32 KICK_REG0;
    volatile Uint32 KICK_REG1;
#ifdef CSL_MODIFICATION
    volatile Uint32 BOOTADDR_GEM0_REG;
    volatile Uint32 BOOTADDR_GEM1_REG;
    volatile Uint32 RSVD_GS1;
    volatile Uint32 RSVD_GS2;
#else
    volatile Uint32 BOOTADDR_GEM_REG[4];
#endif
    volatile Uint8 RSVD2[144];
    volatile Uint32 INTR_RAW_STATUS_REG;
    volatile Uint32 INTR_ENABLED_STATUS_REG;
    volatile Uint32 INTR_ENABLE_REG;
    volatile Uint32 INTR_ENABLE_CLR_REG;
    volatile Uint32 EOI_REG;
    volatile Uint32 FAULT_ADDRESS_REG;
    volatile Uint32 FAULT_STATUS_REG;
    volatile Uint32 FAULT_CLEAR_REG;
    volatile Uint8 RSVD3[16];
    volatile Uint32 EMAC_ID1;
    volatile Uint32 EMAC_ID2;
    volatile Uint8 RSVD4[24];
    volatile Uint32 LRSTNMISTAT_CLR;
    volatile Uint32 RESET_STAT_CLR;
    volatile Uint8 RSVD5[4];
    volatile Uint32 BOOT_COMPLETE;
    volatile Uint32 BOOT_PROGRESS;
    volatile Uint32 RESET_STAT;
    volatile Uint32 LRSTNMISTAT;
    volatile Uint32 DEVCFG;
    volatile Uint32 PWR_STAT;
    volatile Uint32 STS_SRIO;
    volatile Uint32 STS_SGMII;
    volatile Uint32 STS_PCIE;
    volatile Uint32 STS_VUSR;
    volatile Uint32 RSVD_GS3;
    volatile Uint32 RSVD_GS4;
    volatile Uint8 RSVD6[20];
    volatile Uint32 CLASS0_EFUSE_REG;
    volatile Uint8 RSVD7[12];
    volatile Uint32 EFUSE_REG0;
    volatile Uint32 EFUSE_REG1;
    volatile Uint32 EFUSE_REG2;
    volatile Uint32 EFUSE_REG3;
    volatile Uint32 EFUSE_REG4;
    volatile Uint32 EFUSE_REG5;
    volatile Uint32 EFUSE_REG6;
    volatile Uint32 EFUSE_REG7;
    volatile Uint32 EFUSE_REG8;
    volatile Uint32 EFUSE_REG9;
    volatile Uint32 EFUSE_REG10;
    volatile Uint32 EFUSE_REG11;
    volatile Uint32 EFUSE_REG12;
    volatile Uint32 EFUSE_REG13;
    volatile Uint32 EFUSE_REG14;
    volatile Uint32 EFUSE_REG15;
    volatile Uint8 RSVD8[48];
#ifdef CSL_MODIFICATION
    volatile Uint32 NMIGR_0;
    volatile Uint32 NMIGR_1;
    volatile Uint32 RSVD_GS5;
    volatile Uint32 RSVD_GS6;
#else
    volatile Uint32 NMIGR[4];
#endif
    volatile Uint8 RSVD9[48];
#ifdef CSL_MODIFICATION
    volatile Uint32 IPCGR0;
    volatile Uint32 IPCGR1;
    volatile Uint32 RSVD_GS7;
    volatile Uint32 RSVD_GS8;
#else
    volatile Uint32 IPCGR[4];
#endif
    volatile Uint8 RSVD10[44];
    volatile Uint32 IPCGRH;
#ifdef CSL_MODIFICATION
    volatile Uint32 IPCAR0;
    volatile Uint32 IPCAR1;
    volatile Uint32 RSVD_GS9;
    volatile Uint32 RSVD_GS10;
#else
    volatile Uint32 IPCAR[4];
#endif
    volatile Uint8 RSVD11[44];
    volatile Uint32 IPCARH;
    volatile Uint8 RSVD12[64];
    volatile Uint32 TINPSEL;
    volatile Uint32 TOUTSEL;
#ifdef CSL_MODIFICATION
    volatile Uint32 RSTMUX0;
    volatile Uint32 RSTMUX1;
    volatile Uint32 RSVD_GS11;
    volatile Uint32 RSVD_GS12;
#else
    volatile Uint32 RSTMUX[4];
#endif
    volatile Uint8 RSVD13[16];
    volatile Uint32 CORE_PLL_CTL0;
    volatile Uint32 CORE_PLL_CTL1;
    volatile Uint32 DDR3_PLL_CTL0;
    volatile Uint32 DDR3_PLL_CTL1;
    volatile Uint32 RSVD_GS13;
    volatile Uint32 RSVD_GS14;
    volatile Uint32 SGMII_CFGPLL;
    volatile Uint32 SGMII_CFGRX0;
    volatile Uint32 SGMII_CFGTX0;
    volatile Uint32 RSVD_GS15;
    volatile Uint32 RSVD_GS16;
    volatile Uint32 SGMII_RSVD;
    volatile Uint32 PCIE_CFGPLL;
    volatile Uint32 PCIE_SERDES_RSVD;
    volatile Uint32 SRIO_SERDES_CFGPLL;
    volatile Uint32 SRIO_SERDES_CFGRX0;
    volatile Uint32 SRIO_SERDES_CFGTX0;
    volatile Uint32 SRIO_SERDES_CFGRX1;
    volatile Uint32 SRIO_SERDES_CFGTX1;
    volatile Uint32 SRIO_SERDES_CFGRX2;
    volatile Uint32 SRIO_SERDES_CFGTX2;
    volatile Uint32 SRIO_SERDES_CFGRX3;
    volatile Uint32 SRIO_SERDES_CFGTX3;
    volatile Uint32 SRIO_SERDES_RSVD;
    volatile Uint32 LED_GEM_PASSDONE;
    volatile Uint32 LED_PLLLOCK;
    volatile Uint32 LED_CHIP_PASSDONE;
    volatile Uint32 TDIODE;
    volatile Uint32 MARGIN;
    volatile Uint32 SECURE_CONTROL;
    volatile Uint8 RSVD14[4];
    volatile Uint32 EFUSE_SECROM_CHKSUM0;
    volatile Uint32 EFUSE_SECROM_CHKSUM1;
    volatile Uint32 OBSCLK_CTL;
    volatile Uint32 AIF_SERDES_RSVD;
    volatile Uint32 VUSR_CFGPLL;
    volatile Uint32 VUSR_CFGRX0;
    volatile Uint32 VUSR_CFGTX0;
    volatile Uint32 VUSR_CFGRX1;
    volatile Uint32 VUSR_CFGTX1;
    volatile Uint32 VUSR_CFGRX2;
    volatile Uint32 VUSR_CFGTX2;
    volatile Uint32 VUSR_CFGRX3;
    volatile Uint32 VUSR_CFGTX3;
    volatile Uint32 VUSR_SERDES_RSVD;
    volatile Uint32 LED_GPIO_CLR;
    volatile Uint32 LED_GPIO;
    volatile Uint8 RSVD15[12];
    volatile Uint32 EFUSE_RSVD0;
    volatile Uint32 EFUSE_RSVD1;
    volatile Uint32 EFUSE_RSVD2;
    volatile Uint32 EFUSE_RSVD3;
    volatile Uint32 CHIP_MISC;
#ifdef CSL_MODIFICATION
    volatile Uint32 DDR3_CONFIG_REG_0;
    volatile Uint32 DDR3_CONFIG_REG_1;
    volatile Uint32 DDR3_CONFIG_REG_2;
    volatile Uint32 DDR3_CONFIG_REG_3;
    volatile Uint32 DDR3_CONFIG_REG_4;
    volatile Uint32 DDR3_CONFIG_REG_5;
    volatile Uint32 DDR3_CONFIG_REG_6;
    volatile Uint32 DDR3_CONFIG_REG_7;
    volatile Uint32 DDR3_CONFIG_REG_8;
    volatile Uint32 DDR3_CONFIG_REG_9;
    volatile Uint32 DDR3_CONFIG_REG_10;
    volatile Uint32 DDR3_CONFIG_REG_11;
    volatile Uint32 DDR3_CONFIG_REG_12;
    volatile Uint32 DDR3_CONFIG_REG_13;
    volatile Uint32 DDR3_CONFIG_REG_14;
    volatile Uint32 DDR3_CONFIG_REG_15;
    volatile Uint32 DDR3_CONFIG_REG_16;
    volatile Uint32 DDR3_CONFIG_REG_17;
    volatile Uint32 DDR3_CONFIG_REG_18;
    volatile Uint32 DDR3_CONFIG_REG_19;
    volatile Uint32 DDR3_CONFIG_REG_20;
    volatile Uint32 DDR3_CONFIG_REG_21;
    volatile Uint32 DDR3_CONFIG_REG_22;
    volatile Uint32 DDR3_CONFIG_REG_23;
    volatile Uint32 DDR3_CONFIG_REG_24; // last of Ny
    volatile Uint32 DDR3_CONFIG_REG_25;
    volatile Uint32 DDR3_CONFIG_REG_26;
    volatile Uint32 DDR3_CONFIG_REG_27;
    volatile Uint32 DDR3_CONFIG_REG_28;
    volatile Uint32 DDR3_CONFIG_REG_29;
    volatile Uint32 DDR3_CONFIG_REG_30;
    volatile Uint32 DDR3_CONFIG_REG_31;
    volatile Uint32 DDR3_CONFIG_REG_32;
    volatile Uint32 DDR3_CONFIG_REG_33;
    volatile Uint32 DDR3_CONFIG_REG_34;
    volatile Uint32 DDR3_CONFIG_REG_35;
    volatile Uint32 DDR3_CONFIG_REG_36;
    volatile Uint32 DDR3_CONFIG_REG_37;
    volatile Uint32 DDR3_CONFIG_REG_38;
    volatile Uint32 DDR3_CONFIG_REG_39;
    volatile Uint32 DDR3_CONFIG_REG_40;
    volatile Uint32 DDR3_CONFIG_REG_41;
    volatile Uint32 DDR3_CONFIG_REG_42;
    volatile Uint32 DDR3_CONFIG_REG_43;
    volatile Uint32 DDR3_CONFIG_REG_44;
    volatile Uint32 DDR3_CONFIG_REG_45;
    volatile Uint32 DDR3_CONFIG_REG_46;
    volatile Uint32 DDR3_CONFIG_REG_47;
    volatile Uint32 DDR3_CONFIG_REG_48;
    volatile Uint32 DDR3_CONFIG_REG_49;
    volatile Uint32 DDR3_CONFIG_REG_50;
    volatile Uint32 DDR3_CONFIG_REG_51;
    volatile Uint32 DDR3_CONFIG_REG_52;
    volatile Uint32 DDR3_CONFIG_REG_53;
    volatile Uint32 DDR3_CONFIG_REG_54;
    volatile Uint32 DDR3_CONFIG_REG_55;
    volatile Uint32 DDR3_CONFIG_REG_56;
    volatile Uint32 DDR3_CONFIG_REG_57;
    volatile Uint32 DDR3_CONFIG_REG_58;
    volatile Uint32 DDR3_CONFIG_REG_59;
    volatile Uint32 DDR3_CONFIG_REG_60;
    volatile Uint32 DDR3_CONFIG_REG_RESV1[3];
#else
    volatile Uint32 DDR3_CONFIG_REG[64];
#endif
    volatile Uint32 MARGIN1;
    volatile Uint32 MARGIN2; // new
    volatile Uint32 MARGIN3;
    volatile Uint32 MARGIN4;
    volatile Uint32 MARGIN5;
    volatile Uint32 CHIP_PIN_CONTROL_0;
    volatile Uint32 CHIP_PIN_CONTROL_1;
    volatile Uint32 REGS_UID0;
    volatile Uint32 REGS_UID1;
    volatile Uint32 REGS_UID2;
    volatile Uint32 REGS_UID3;
    volatile Uint32 REGS_CUST_EFUSE_RSVD_0;
    volatile Uint32 REGS_CUST_EFUSE_RSVD_1;
    volatile Uint32 REGS_CUST_EFUSE_RSVD_2;
    volatile Uint32 REGS_CUST_EFUSE_RSVD_3;
    volatile Uint32 REGS_CONTROL_SEC_EMU;
    volatile Uint32 REGS_CONTROL_SEC_STATUS;
    volatile Uint32 REGS_CONTROL_KEK_0;
    volatile Uint32 REGS_CONTROL_KEK_1;
    volatile Uint32 REGS_CONTROL_KEK_2;
    volatile Uint32 REGS_CONTROL_KEK_3;
    volatile Uint32 REGS_CONTROL_CEK_0;
    volatile Uint32 REGS_CONTROL_CEK_1;
    volatile Uint32 REGS_CONTROL_CEK_2;
    volatile Uint32 REGS_CONTROL_CEK_3;
    volatile Uint32 REGS_CONTROL_MPK_0;
    volatile Uint32 REGS_CONTROL_MPK_1;
    volatile Uint32 REGS_CONTROL_MPK_2;
    volatile Uint32 REGS_CONTROL_MPK_3;
    volatile Uint32 REGS_CONTROL_MPK_4;
    volatile Uint32 REGS_CONTROL_MPK_5;
    volatile Uint32 REGS_CONTROL_MPK_6;
    volatile Uint32 REGS_CONTROL_MPK_7;
    volatile Uint32 REGS_CONTROL_CEK_BCH_0;
    volatile Uint32 REGS_CONTROL_CEK_BCH_1;
    volatile Uint32 REGS_CONTROL_CEK_BCH_2;
    volatile Uint32 REGS_CONTROL_CEK_BCH_3;
    volatile Uint32 REGS_CONTROL_CEK_BCH_4;
    volatile Uint32 REGS_CONTROL_MPK_BCH_0;
    volatile Uint32 REGS_CONTROL_MPK_BCH_1;
    volatile Uint32 REGS_CONTROL_MPK_BCH_2;
    volatile Uint32 REGS_CONTROL_MPK_BCH_3;
    volatile Uint32 REGS_CONTROL_MPK_BCH_4;
    volatile Uint32 REGS_CONTROL_MPK_BCH_5;
    volatile Uint32 REGS_CONTROL_MPK_BCH_6;
    volatile Uint32 REGS_CONTROL_MPK_BCH_7;
    volatile Uint32 REGS_CONTROL_MPK_BCH_8;
    volatile Uint32 REGS_CONTROL_MPK_BCH_9;
    volatile Uint8 RSVD16[6580];
    volatile Uint32 END_POINT;
} CSL_BootcfgRegs;

#endif
