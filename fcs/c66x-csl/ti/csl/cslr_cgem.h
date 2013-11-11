/********************************************************************
* Copyright (C) 2003-2010 Texas Instruments Incorporated.
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
#ifndef _CSLR_CGEM_H_
#define _CSLR_CGEM_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 *      b) The MAR0-MAR255 have been made into an array of MAR[256]
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 EVTFLAG[4];
    volatile Uint8 RSVD0[16];
    volatile Uint32 EVTSET[4];
    volatile Uint8 RSVD1[16];
    volatile Uint32 EVTCLR[4];
    volatile Uint8 RSVD2[48];
    volatile Uint32 EVTMASK[4];
    volatile Uint8 RSVD3[16];
    volatile Uint32 MEVTFLAG[4];
    volatile Uint8 RSVD4[16];
    volatile Uint32 EXPMASK[4];
    volatile Uint8 RSVD5[16];
    volatile Uint32 MEXPFLAG[4];
    volatile Uint8 RSVD6[20];
    volatile Uint32 INTMUX1;
    volatile Uint32 INTMUX2;
    volatile Uint32 INTMUX3;
    volatile Uint8 RSVD7[48];
    volatile Uint32 AEGMUX0;
    volatile Uint32 AEGMUX1;
    volatile Uint8 RSVD8[56];
    volatile Uint32 INTXSTAT;
    volatile Uint32 INTXCLR;
    volatile Uint32 INTDMASK;
    volatile Uint8 RSVD9[52];
    volatile Uint32 EVTASRT;
    volatile Uint8 RSVD10[65084];
    volatile Uint32 PDCCMD;
    volatile Uint8 RSVD11[8188];
    volatile Uint32 MM_REVID;
    volatile Uint8 RSVD12[57340];
    volatile Uint32 IDMA0_STAT;
    volatile Uint32 IDMA0_MASK;
    volatile Uint32 IDMA0_SOURCE;
    volatile Uint32 IDMA0_DEST;
    volatile Uint32 IDMA0_COUNT;
    volatile Uint8 RSVD13[236];
    volatile Uint32 IDMA1_STAT;
    volatile Uint8 RSVD14[4];
    volatile Uint32 IDMA1_SOURCE;
    volatile Uint32 IDMA1_DEST;
    volatile Uint32 IDMA1_COUNT;
    volatile Uint8 RSVD15[236];
    volatile Uint32 CPUARBE;
    volatile Uint32 IDMAARBE;
    volatile Uint32 SDMAARBE;
    volatile Uint8 RSVD16[4];
    volatile Uint32 ECFGARBE;
    volatile Uint8 RSVD17[236];
    volatile Uint32 ICFGMPFAR;
    volatile Uint32 ICFGMPFSR;
    volatile Uint32 ICFGMPFCR;
    volatile Uint8 RSVD18[252];
    volatile Uint32 ECFGERR;
    volatile Uint32 ECFGERRCLR;
    volatile Uint8 RSVD19[240];
    volatile Uint32 PAMAP0;
    volatile Uint32 PAMAP1;
    volatile Uint32 PAMAP2;
    volatile Uint32 PAMAP3;
    volatile Uint32 PAMAP4;
    volatile Uint32 PAMAP5;
    volatile Uint32 PAMAP6;
    volatile Uint32 PAMAP7;
    volatile Uint32 PAMAP8;
    volatile Uint32 PAMAP9;
    volatile Uint32 PAMAP10;
    volatile Uint32 PAMAP11;
    volatile Uint32 PAMAP12;
    volatile Uint32 PAMAP13;
    volatile Uint32 PAMAP14;
    volatile Uint32 PAMAP15;
    volatile Uint8 RSVD20[129728];
    volatile Uint32 L2CFG;
    volatile Uint8 RSVD21[28];
    volatile Uint32 L1PCFG;
    volatile Uint32 L1PCC;
    volatile Uint8 RSVD22[24];
    volatile Uint32 L1DCFG;
    volatile Uint32 L1DCC;
    volatile Uint8 RSVD23[4024];
    volatile Uint32 CPUARBU;
    volatile Uint32 IDMAARBU;
    volatile Uint32 SDMAARBU;
    volatile Uint32 UCARBU;
    volatile Uint32 MDMAARBU;
    volatile Uint8 RSVD24[44];
    volatile Uint32 CPUARBD;
    volatile Uint32 IDMAARBD;
    volatile Uint32 SDMAARBD;
    volatile Uint32 UCARBD;
    volatile Uint8 RSVD25[12208];
    volatile Uint32 L2WBAR;
    volatile Uint32 L2WWC;
    volatile Uint8 RSVD26[8];
    volatile Uint32 L2WIBAR;
    volatile Uint32 L2WIWC;
    volatile Uint32 L2IBAR;
    volatile Uint32 L2IWC;
    volatile Uint32 L1PIBAR;
    volatile Uint32 L1PIWC;
    volatile Uint8 RSVD27[8];
    volatile Uint32 L1DWIBAR;
    volatile Uint32 L1DWIWC;
    volatile Uint8 RSVD28[8];
    volatile Uint32 L1DWBAR;
    volatile Uint32 L1DWWC;
    volatile Uint32 L1DIBAR;
    volatile Uint32 L1DIWC;
    volatile Uint8 RSVD29[4016];
    volatile Uint32 L2WB;
    volatile Uint32 L2WBINV;
    volatile Uint32 L2INV;
    volatile Uint8 RSVD30[28];
    volatile Uint32 L1PINV;
    volatile Uint8 RSVD31[20];
    volatile Uint32 L1DWB;
    volatile Uint32 L1DWBINV;
    volatile Uint32 L1DINV;
    volatile Uint8 RSVD32[4024];
    volatile Uint32 L2EDSTAT;
    volatile Uint32 L2EDCMD;
    volatile Uint32 L2EDADDR;
    volatile Uint8 RSVD33[8];
    volatile Uint32 L2EDCPEC;
    volatile Uint32 L2EDCNEC;
    volatile Uint32 MDMAERR;
    volatile Uint32 MDMAERRCLR;
    volatile Uint8 RSVD34[8];
    volatile Uint32 L2EDCEN;
    volatile Uint8 RSVD35[976];
    volatile Uint32 L1PEDSTAT;
    volatile Uint32 L1PEDCMD;
    volatile Uint32 L1PEDADDR;
    volatile Uint8 RSVD36[7152];
#ifdef CSL_MODIFICATION    
    volatile Uint32 MAR0;
    volatile Uint32 MAR1;
    volatile Uint32 MAR2;
    volatile Uint32 MAR3;
    volatile Uint32 MAR4;
    volatile Uint32 MAR5;
    volatile Uint32 MAR6;
    volatile Uint32 MAR7;
    volatile Uint32 MAR8;
    volatile Uint32 MAR9;
    volatile Uint32 MAR10;
    volatile Uint32 MAR11;
    volatile Uint32 MAR12;
    volatile Uint32 MAR13;
    volatile Uint32 MAR14;
    volatile Uint32 MAR15;
    volatile Uint32 MAR16;
    volatile Uint32 MAR17;
    volatile Uint32 MAR18;
    volatile Uint32 MAR19;
    volatile Uint32 MAR20;
    volatile Uint32 MAR21;
    volatile Uint32 MAR22;
    volatile Uint32 MAR23;
    volatile Uint32 MAR24;
    volatile Uint32 MAR25;
    volatile Uint32 MAR26;
    volatile Uint32 MAR27;
    volatile Uint32 MAR28;
    volatile Uint32 MAR29;
    volatile Uint32 MAR30;
    volatile Uint32 MAR31;
    volatile Uint32 MAR32;
    volatile Uint32 MAR33;
    volatile Uint32 MAR34;
    volatile Uint32 MAR35;
    volatile Uint32 MAR36;
    volatile Uint32 MAR37;
    volatile Uint32 MAR38;
    volatile Uint32 MAR39;
    volatile Uint32 MAR40;
    volatile Uint32 MAR41;
    volatile Uint32 MAR42;
    volatile Uint32 MAR43;
    volatile Uint32 MAR44;
    volatile Uint32 MAR45;
    volatile Uint32 MAR46;
    volatile Uint32 MAR47;
    volatile Uint32 MAR48;
    volatile Uint32 MAR49;
    volatile Uint32 MAR50;
    volatile Uint32 MAR51;
    volatile Uint32 MAR52;
    volatile Uint32 MAR53;
    volatile Uint32 MAR54;
    volatile Uint32 MAR55;
    volatile Uint32 MAR56;
    volatile Uint32 MAR57;
    volatile Uint32 MAR58;
    volatile Uint32 MAR59;
    volatile Uint32 MAR60;
    volatile Uint32 MAR61;
    volatile Uint32 MAR62;
    volatile Uint32 MAR63;
    volatile Uint32 MAR64;
    volatile Uint32 MAR65;
    volatile Uint32 MAR66;
    volatile Uint32 MAR67;
    volatile Uint32 MAR68;
    volatile Uint32 MAR69;
    volatile Uint32 MAR70;
    volatile Uint32 MAR71;
    volatile Uint32 MAR72;
    volatile Uint32 MAR73;
    volatile Uint32 MAR74;
    volatile Uint32 MAR75;
    volatile Uint32 MAR76;
    volatile Uint32 MAR77;
    volatile Uint32 MAR78;
    volatile Uint32 MAR79;
    volatile Uint32 MAR80;
    volatile Uint32 MAR81;
    volatile Uint32 MAR82;
    volatile Uint32 MAR83;
    volatile Uint32 MAR84;
    volatile Uint32 MAR85;
    volatile Uint32 MAR86;
    volatile Uint32 MAR87;
    volatile Uint32 MAR88;
    volatile Uint32 MAR89;
    volatile Uint32 MAR90;
    volatile Uint32 MAR91;
    volatile Uint32 MAR92;
    volatile Uint32 MAR93;
    volatile Uint32 MAR94;
    volatile Uint32 MAR95;
    volatile Uint32 MAR96;
    volatile Uint32 MAR97;
    volatile Uint32 MAR98;
    volatile Uint32 MAR99;
    volatile Uint32 MAR100;
    volatile Uint32 MAR101;
    volatile Uint32 MAR102;
    volatile Uint32 MAR103;
    volatile Uint32 MAR104;
    volatile Uint32 MAR105;
    volatile Uint32 MAR106;
    volatile Uint32 MAR107;
    volatile Uint32 MAR108;
    volatile Uint32 MAR109;
    volatile Uint32 MAR110;
    volatile Uint32 MAR111;
    volatile Uint32 MAR112;
    volatile Uint32 MAR113;
    volatile Uint32 MAR114;
    volatile Uint32 MAR115;
    volatile Uint32 MAR116;
    volatile Uint32 MAR117;
    volatile Uint32 MAR118;
    volatile Uint32 MAR119;
    volatile Uint32 MAR120;
    volatile Uint32 MAR121;
    volatile Uint32 MAR122;
    volatile Uint32 MAR123;
    volatile Uint32 MAR124;
    volatile Uint32 MAR125;
    volatile Uint32 MAR126;
    volatile Uint32 MAR127;
    volatile Uint32 MAR128;
    volatile Uint32 MAR129;
    volatile Uint32 MAR130;
    volatile Uint32 MAR131;
    volatile Uint32 MAR132;
    volatile Uint32 MAR133;
    volatile Uint32 MAR134;
    volatile Uint32 MAR135;
    volatile Uint32 MAR136;
    volatile Uint32 MAR137;
    volatile Uint32 MAR138;
    volatile Uint32 MAR139;
    volatile Uint32 MAR140;
    volatile Uint32 MAR141;
    volatile Uint32 MAR142;
    volatile Uint32 MAR143;
    volatile Uint32 MAR144;
    volatile Uint32 MAR145;
    volatile Uint32 MAR146;
    volatile Uint32 MAR147;
    volatile Uint32 MAR148;
    volatile Uint32 MAR149;
    volatile Uint32 MAR150;
    volatile Uint32 MAR151;
    volatile Uint32 MAR152;
    volatile Uint32 MAR153;
    volatile Uint32 MAR154;
    volatile Uint32 MAR155;
    volatile Uint32 MAR156;
    volatile Uint32 MAR157;
    volatile Uint32 MAR158;
    volatile Uint32 MAR159;
    volatile Uint32 MAR160;
    volatile Uint32 MAR161;
    volatile Uint32 MAR162;
    volatile Uint32 MAR163;
    volatile Uint32 MAR164;
    volatile Uint32 MAR165;
    volatile Uint32 MAR166;
    volatile Uint32 MAR167;
    volatile Uint32 MAR168;
    volatile Uint32 MAR169;
    volatile Uint32 MAR170;
    volatile Uint32 MAR171;
    volatile Uint32 MAR172;
    volatile Uint32 MAR173;
    volatile Uint32 MAR174;
    volatile Uint32 MAR175;
    volatile Uint32 MAR176;
    volatile Uint32 MAR177;
    volatile Uint32 MAR178;
    volatile Uint32 MAR179;
    volatile Uint32 MAR180;
    volatile Uint32 MAR181;
    volatile Uint32 MAR182;
    volatile Uint32 MAR183;
    volatile Uint32 MAR184;
    volatile Uint32 MAR185;
    volatile Uint32 MAR186;
    volatile Uint32 MAR187;
    volatile Uint32 MAR188;
    volatile Uint32 MAR189;
    volatile Uint32 MAR190;
    volatile Uint32 MAR191;
    volatile Uint32 MAR192;
    volatile Uint32 MAR193;
    volatile Uint32 MAR194;
    volatile Uint32 MAR195;
    volatile Uint32 MAR196;
    volatile Uint32 MAR197;
    volatile Uint32 MAR198;
    volatile Uint32 MAR199;
    volatile Uint32 MAR200;
    volatile Uint32 MAR201;
    volatile Uint32 MAR202;
    volatile Uint32 MAR203;
    volatile Uint32 MAR204;
    volatile Uint32 MAR205;
    volatile Uint32 MAR206;
    volatile Uint32 MAR207;
    volatile Uint32 MAR208;
    volatile Uint32 MAR209;
    volatile Uint32 MAR210;
    volatile Uint32 MAR211;
    volatile Uint32 MAR212;
    volatile Uint32 MAR213;
    volatile Uint32 MAR214;
    volatile Uint32 MAR215;
    volatile Uint32 MAR216;
    volatile Uint32 MAR217;
    volatile Uint32 MAR218;
    volatile Uint32 MAR219;
    volatile Uint32 MAR220;
    volatile Uint32 MAR221;
    volatile Uint32 MAR222;
    volatile Uint32 MAR223;
    volatile Uint32 MAR224;
    volatile Uint32 MAR225;
    volatile Uint32 MAR226;
    volatile Uint32 MAR227;
    volatile Uint32 MAR228;
    volatile Uint32 MAR229;
    volatile Uint32 MAR230;
    volatile Uint32 MAR231;
    volatile Uint32 MAR232;
    volatile Uint32 MAR233;
    volatile Uint32 MAR234;
    volatile Uint32 MAR235;
    volatile Uint32 MAR236;
    volatile Uint32 MAR237;
    volatile Uint32 MAR238;
    volatile Uint32 MAR239;
    volatile Uint32 MAR240;
    volatile Uint32 MAR241;
    volatile Uint32 MAR242;
    volatile Uint32 MAR243;
    volatile Uint32 MAR244;
    volatile Uint32 MAR245;
    volatile Uint32 MAR246;
    volatile Uint32 MAR247;
    volatile Uint32 MAR248;
    volatile Uint32 MAR249;
    volatile Uint32 MAR250;
    volatile Uint32 MAR251;
    volatile Uint32 MAR252;
    volatile Uint32 MAR253;
    volatile Uint32 MAR254;
    volatile Uint32 MAR255;
#else
    volatile Uint32 MAR[256];
#endif    
    volatile Uint8 RSVD37[7168];
    volatile Uint32 L2MPFAR;
    volatile Uint32 L2MPFSR;
    volatile Uint32 L2MPFCR;
    volatile Uint8 RSVD38[500];
    volatile Uint32 L2MPPA[32];
    volatile Uint8 RSVD39[384];
    volatile Uint32 L1PMPFAR;
    volatile Uint32 L1PMPFSR;
    volatile Uint32 L1PMPFCR;
    volatile Uint8 RSVD40[564];
    volatile Uint32 L1PMPPA[16];
    volatile Uint8 RSVD41[1408];
    volatile Uint32 L1DMPFAR;
    volatile Uint32 L1DMPFSR;
    volatile Uint32 L1DMPFCR;
    volatile Uint8 RSVD42[244];
    volatile Uint32 MPLK[4];
    volatile Uint32 MPLKCMD;
    volatile Uint32 MPLKSTAT;
    volatile Uint8 RSVD43[296];
    volatile Uint32 L1DMPPA[16];
} CSL_CgemRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* EVTFLAG */

#define CSL_CGEM_EVTFLAG_EF_MASK         (0xFFFFFFFFu)
#define CSL_CGEM_EVTFLAG_EF_SHIFT        (0x00000000u)
#define CSL_CGEM_EVTFLAG_EF_RESETVAL     (0x00000000u)

#define CSL_CGEM_EVTFLAG_RESETVAL        (0x00000000u)

/* EVTSET */

#define CSL_CGEM_EVTSET_ES_MASK          (0xFFFFFFFFu)
#define CSL_CGEM_EVTSET_ES_SHIFT         (0x00000000u)
#define CSL_CGEM_EVTSET_ES_RESETVAL      (0x00000000u)

#define CSL_CGEM_EVTSET_RESETVAL         (0x00000000u)

/* EVTCLR */

#define CSL_CGEM_EVTCLR_EC_MASK          (0xFFFFFFFFu)
#define CSL_CGEM_EVTCLR_EC_SHIFT         (0x00000000u)
#define CSL_CGEM_EVTCLR_EC_RESETVAL      (0x00000000u)

#define CSL_CGEM_EVTCLR_RESETVAL         (0x00000000u)

/* EVTMASK */

#define CSL_CGEM_EVTMASK_EM_MASK         (0xFFFFFFFFu)
#define CSL_CGEM_EVTMASK_EM_SHIFT        (0x00000000u)
#define CSL_CGEM_EVTMASK_EM_RESETVAL     (0x00000000u)

#define CSL_CGEM_EVTMASK_RESETVAL        (0x00000000u)

/* MEVTFLAG */

#define CSL_CGEM_MEVTFLAG_MEVTFLAG_MASK  (0xFFFFFFFFu)
#define CSL_CGEM_MEVTFLAG_MEVTFLAG_SHIFT (0x00000000u)
#define CSL_CGEM_MEVTFLAG_MEVTFLAG_RESETVAL (0x00000000u)

#define CSL_CGEM_MEVTFLAG_RESETVAL       (0x00000000u)

/* EXPMASK */

#define CSL_CGEM_EXPMASK_XM_MASK         (0xFFFFFFFFu)
#define CSL_CGEM_EXPMASK_XM_SHIFT        (0x00000000u)
#define CSL_CGEM_EXPMASK_XM_RESETVAL     (0x00000000u)

#define CSL_CGEM_EXPMASK_RESETVAL        (0xFFFFFFFFu)

/* MEXPFLAG */

#define CSL_CGEM_MEXPFLAG_MEXPFLAG_MASK  (0xFFFFFFFFu)
#define CSL_CGEM_MEXPFLAG_MEXPFLAG_SHIFT (0x00000000u)
#define CSL_CGEM_MEXPFLAG_MEXPFLAG_RESETVAL (0x00000000u)

#define CSL_CGEM_MEXPFLAG_RESETVAL       (0x00000000u)

/* INTMUX1 */





#define CSL_CGEM_INTMUX1_INTSEL7_MASK    (0x7F000000u)
#define CSL_CGEM_INTMUX1_INTSEL7_SHIFT   (0x00000018u)
#define CSL_CGEM_INTMUX1_INTSEL7_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX1_INTSEL6_MASK    (0x007F0000u)
#define CSL_CGEM_INTMUX1_INTSEL6_SHIFT   (0x00000010u)
#define CSL_CGEM_INTMUX1_INTSEL6_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX1_INTSEL5_MASK    (0x00007F00u)
#define CSL_CGEM_INTMUX1_INTSEL5_SHIFT   (0x00000008u)
#define CSL_CGEM_INTMUX1_INTSEL5_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX1_INTSEL4_MASK    (0x0000007Fu)
#define CSL_CGEM_INTMUX1_INTSEL4_SHIFT   (0x00000000u)
#define CSL_CGEM_INTMUX1_INTSEL4_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX1_RESETVAL        (0x07060504u)

/* INTMUX2 */





#define CSL_CGEM_INTMUX2_INTSEL11_MASK   (0x7F000000u)
#define CSL_CGEM_INTMUX2_INTSEL11_SHIFT  (0x00000018u)
#define CSL_CGEM_INTMUX2_INTSEL11_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX2_INTSEL10_MASK   (0x007F0000u)
#define CSL_CGEM_INTMUX2_INTSEL10_SHIFT  (0x00000010u)
#define CSL_CGEM_INTMUX2_INTSEL10_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX2_INTSEL9_MASK    (0x00007F00u)
#define CSL_CGEM_INTMUX2_INTSEL9_SHIFT   (0x00000008u)
#define CSL_CGEM_INTMUX2_INTSEL9_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX2_INTSEL8_MASK    (0x0000007Fu)
#define CSL_CGEM_INTMUX2_INTSEL8_SHIFT   (0x00000000u)
#define CSL_CGEM_INTMUX2_INTSEL8_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX2_RESETVAL        (0x0B0A0908u)

/* INTMUX3 */





#define CSL_CGEM_INTMUX3_INTSEL15_MASK   (0x7F000000u)
#define CSL_CGEM_INTMUX3_INTSEL15_SHIFT  (0x00000018u)
#define CSL_CGEM_INTMUX3_INTSEL15_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX3_INTSEL14_MASK   (0x007F0000u)
#define CSL_CGEM_INTMUX3_INTSEL14_SHIFT  (0x00000010u)
#define CSL_CGEM_INTMUX3_INTSEL14_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX3_INTSEL13_MASK   (0x00007F00u)
#define CSL_CGEM_INTMUX3_INTSEL13_SHIFT  (0x00000008u)
#define CSL_CGEM_INTMUX3_INTSEL13_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX3_INTSEL12_MASK   (0x0000007Fu)
#define CSL_CGEM_INTMUX3_INTSEL12_SHIFT  (0x00000000u)
#define CSL_CGEM_INTMUX3_INTSEL12_RESETVAL (0x00000000u)

#define CSL_CGEM_INTMUX3_RESETVAL        (0x0F0E0D0Cu)

/* AEGMUX0 */

#define CSL_CGEM_AEGMUX0_AEGSEL3_MASK    (0x7F000000u)
#define CSL_CGEM_AEGMUX0_AEGSEL3_SHIFT   (0x00000018u)
#define CSL_CGEM_AEGMUX0_AEGSEL3_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX0_AEGSEL2_MASK    (0x007F0000u)
#define CSL_CGEM_AEGMUX0_AEGSEL2_SHIFT   (0x00000010u)
#define CSL_CGEM_AEGMUX0_AEGSEL2_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX0_AEGSEL1_MASK    (0x00007F00u)
#define CSL_CGEM_AEGMUX0_AEGSEL1_SHIFT   (0x00000008u)
#define CSL_CGEM_AEGMUX0_AEGSEL1_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX0_AEGSEL0_MASK    (0x0000007Fu)
#define CSL_CGEM_AEGMUX0_AEGSEL0_SHIFT   (0x00000000u)
#define CSL_CGEM_AEGMUX0_AEGSEL0_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX0_RESETVAL        (0x03020100u)

/* AEGMUX1 */

#define CSL_CGEM_AEGMUX1_AEGSEL7_MASK    (0x7F000000u)
#define CSL_CGEM_AEGMUX1_AEGSEL7_SHIFT   (0x00000018u)
#define CSL_CGEM_AEGMUX1_AEGSEL7_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX1_AEGSEL6_MASK    (0x007F0000u)
#define CSL_CGEM_AEGMUX1_AEGSEL6_SHIFT   (0x00000010u)
#define CSL_CGEM_AEGMUX1_AEGSEL6_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX1_AEGSEL5_MASK    (0x00007F00u)
#define CSL_CGEM_AEGMUX1_AEGSEL5_SHIFT   (0x00000008u)
#define CSL_CGEM_AEGMUX1_AEGSEL5_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX1_AEGSEL4_MASK    (0x0000007Fu)
#define CSL_CGEM_AEGMUX1_AEGSEL4_SHIFT   (0x00000000u)
#define CSL_CGEM_AEGMUX1_AEGSEL4_RESETVAL (0x00000000u)

#define CSL_CGEM_AEGMUX1_RESETVAL        (0x07060504u)

/* INTXSTAT */

#define CSL_CGEM_INTXSTAT_SYSINT_MASK    (0xFF000000u)
#define CSL_CGEM_INTXSTAT_SYSINT_SHIFT   (0x00000018u)
#define CSL_CGEM_INTXSTAT_SYSINT_RESETVAL (0x00000000u)

#define CSL_CGEM_INTXSTAT_CPUINT_MASK    (0x00FF0000u)
#define CSL_CGEM_INTXSTAT_CPUINT_SHIFT   (0x00000010u)
#define CSL_CGEM_INTXSTAT_CPUINT_RESETVAL (0x00000000u)


#define CSL_CGEM_INTXSTAT_DROP_MASK      (0x00000001u)
#define CSL_CGEM_INTXSTAT_DROP_SHIFT     (0x00000000u)
#define CSL_CGEM_INTXSTAT_DROP_RESETVAL  (0x00000000u)

#define CSL_CGEM_INTXSTAT_RESETVAL       (0x00000000u)

/* INTXCLR */


#define CSL_CGEM_INTXCLR_CLEAR_MASK      (0x00000001u)
#define CSL_CGEM_INTXCLR_CLEAR_SHIFT     (0x00000000u)
#define CSL_CGEM_INTXCLR_CLEAR_RESETVAL  (0x00000000u)

#define CSL_CGEM_INTXCLR_RESETVAL        (0x00000000u)

/* INTDMASK */


#define CSL_CGEM_INTDMASK_IDM_MASK       (0x0000FFF0u)
#define CSL_CGEM_INTDMASK_IDM_SHIFT      (0x00000004u)
#define CSL_CGEM_INTDMASK_IDM_RESETVAL   (0x00000000u)


#define CSL_CGEM_INTDMASK_RESETVAL       (0x00000000u)

/* EVTASRT */

#define CSL_CGEM_EVTASRT_MRA_MASK        (0x80000000u)
#define CSL_CGEM_EVTASRT_MRA_SHIFT       (0x0000001Fu)
#define CSL_CGEM_EVTASRT_MRA_RESETVAL    (0x00000000u)

#define CSL_CGEM_EVTASRT_SRA_MASK        (0x40000000u)
#define CSL_CGEM_EVTASRT_SRA_SHIFT       (0x0000001Eu)
#define CSL_CGEM_EVTASRT_SRA_RESETVAL    (0x00000000u)

#define CSL_CGEM_EVTASRT_CRA_MASK        (0x20000000u)
#define CSL_CGEM_EVTASRT_CRA_SHIFT       (0x0000001Du)
#define CSL_CGEM_EVTASRT_CRA_RESETVAL    (0x00000000u)

#define CSL_CGEM_EVTASRT_LRA_MASK        (0x10000000u)
#define CSL_CGEM_EVTASRT_LRA_SHIFT       (0x0000001Cu)
#define CSL_CGEM_EVTASRT_LRA_RESETVAL    (0x00000000u)


#define CSL_CGEM_EVTASRT_EA_MASK         (0x000000FFu)
#define CSL_CGEM_EVTASRT_EA_SHIFT        (0x00000000u)
#define CSL_CGEM_EVTASRT_EA_RESETVAL     (0x00000000u)

#define CSL_CGEM_EVTASRT_RESETVAL        (0x03020100u)

/* PDCCMD */


#define CSL_CGEM_PDCCMD_GEMPD_MASK       (0x00010000u)
#define CSL_CGEM_PDCCMD_GEMPD_SHIFT      (0x00000010u)
#define CSL_CGEM_PDCCMD_GEMPD_RESETVAL   (0x00000000u)


#define CSL_CGEM_PDCCMD_EMCLOG_MASK      (0x00003000u)
#define CSL_CGEM_PDCCMD_EMCLOG_SHIFT     (0x0000000Cu)
#define CSL_CGEM_PDCCMD_EMCLOG_RESETVAL  (0x00000000u)


#define CSL_CGEM_PDCCMD_UMCLOG_MASK      (0x00000300u)
#define CSL_CGEM_PDCCMD_UMCLOG_SHIFT     (0x00000008u)
#define CSL_CGEM_PDCCMD_UMCLOG_RESETVAL  (0x00000000u)

#define CSL_CGEM_PDCCMD_DMCMEM_MASK      (0x000000C0u)
#define CSL_CGEM_PDCCMD_DMCMEM_SHIFT     (0x00000006u)
#define CSL_CGEM_PDCCMD_DMCMEM_RESETVAL  (0x00000000u)

#define CSL_CGEM_PDCCMD_DMCLOG_MASK      (0x00000030u)
#define CSL_CGEM_PDCCMD_DMCLOG_SHIFT     (0x00000004u)
#define CSL_CGEM_PDCCMD_DMCLOG_RESETVAL  (0x00000000u)

#define CSL_CGEM_PDCCMD_PMCMEM_MASK      (0x0000000Cu)
#define CSL_CGEM_PDCCMD_PMCMEM_SHIFT     (0x00000002u)
#define CSL_CGEM_PDCCMD_PMCMEM_RESETVAL  (0x00000000u)

#define CSL_CGEM_PDCCMD_PMCLOG_MASK      (0x00000003u)
#define CSL_CGEM_PDCCMD_PMCLOG_SHIFT     (0x00000000u)
#define CSL_CGEM_PDCCMD_PMCLOG_RESETVAL  (0x00000000u)

#define CSL_CGEM_PDCCMD_RESETVAL         (0x00000000u)

/* MM_REVID */

#define CSL_CGEM_MM_REVID_MM_REVID_MASK  (0xFFFFFFFFu)
#define CSL_CGEM_MM_REVID_MM_REVID_SHIFT (0x00000000u)
#define CSL_CGEM_MM_REVID_MM_REVID_RESETVAL (0x00000000u)

#define CSL_CGEM_MM_REVID_RESETVAL       (0x00000000u)

/* IDMA0_STAT */


#define CSL_CGEM_IDMA0_STAT_PEND_MASK    (0x00000002u)
#define CSL_CGEM_IDMA0_STAT_PEND_SHIFT   (0x00000001u)
#define CSL_CGEM_IDMA0_STAT_PEND_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA0_STAT_ACTV_MASK    (0x00000001u)
#define CSL_CGEM_IDMA0_STAT_ACTV_SHIFT   (0x00000000u)
#define CSL_CGEM_IDMA0_STAT_ACTV_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA0_STAT_RESETVAL     (0x00000000u)

/* IDMA0_MASK */

#define CSL_CGEM_IDMA0_MASK_MASK_MASK    (0xFFFFFFFFu)
#define CSL_CGEM_IDMA0_MASK_MASK_SHIFT   (0x00000000u)
#define CSL_CGEM_IDMA0_MASK_MASK_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA0_MASK_RESETVAL     (0x00000000u)

/* IDMA0_SOURCE */

#define CSL_CGEM_IDMA0_SOURCE_SOURCEADDR_MASK (0xFFFFFFE0u)
#define CSL_CGEM_IDMA0_SOURCE_SOURCEADDR_SHIFT (0x00000005u)
#define CSL_CGEM_IDMA0_SOURCE_SOURCEADDR_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA0_SOURCE_RESETVAL   (0x00000000u)

/* IDMA0_DEST */

#define CSL_CGEM_IDMA0_DEST_DESTADDR_MASK (0xFFFFFFE0u)
#define CSL_CGEM_IDMA0_DEST_DESTADDR_SHIFT (0x00000005u)
#define CSL_CGEM_IDMA0_DEST_DESTADDR_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA0_DEST_RESETVAL     (0x00000000u)

/* IDMA0_COUNT */


#define CSL_CGEM_IDMA0_COUNT_INT_MASK    (0x10000000u)
#define CSL_CGEM_IDMA0_COUNT_INT_SHIFT   (0x0000001Cu)
#define CSL_CGEM_IDMA0_COUNT_INT_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA0_COUNT_COUNT_MASK  (0x0000000Fu)
#define CSL_CGEM_IDMA0_COUNT_COUNT_SHIFT (0x00000000u)
#define CSL_CGEM_IDMA0_COUNT_COUNT_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA0_COUNT_RESETVAL    (0x00000000u)

/* IDMA1_STAT */


#define CSL_CGEM_IDMA1_STAT_PEND_MASK    (0x00000002u)
#define CSL_CGEM_IDMA1_STAT_PEND_SHIFT   (0x00000001u)
#define CSL_CGEM_IDMA1_STAT_PEND_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA1_STAT_ACTV_MASK    (0x00000001u)
#define CSL_CGEM_IDMA1_STAT_ACTV_SHIFT   (0x00000000u)
#define CSL_CGEM_IDMA1_STAT_ACTV_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA1_STAT_RESETVAL     (0x00000000u)

/* IDMA1_SOURCE */

#define CSL_CGEM_IDMA1_SOURCE_SOURCEADDR_MASK (0xFFFFFFFCu)
#define CSL_CGEM_IDMA1_SOURCE_SOURCEADDR_SHIFT (0x00000002u)
#define CSL_CGEM_IDMA1_SOURCE_SOURCEADDR_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA1_SOURCE_RESETVAL   (0x00000000u)

/* IDMA1_DEST */

#define CSL_CGEM_IDMA1_DEST_DESTADDR_MASK (0xFFFFFFFCu)
#define CSL_CGEM_IDMA1_DEST_DESTADDR_SHIFT (0x00000002u)
#define CSL_CGEM_IDMA1_DEST_DESTADDR_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA1_DEST_RESETVAL     (0x00000000u)

/* IDMA1_COUNT */

#define CSL_CGEM_IDMA1_COUNT_PRI_MASK    (0xE0000000u)
#define CSL_CGEM_IDMA1_COUNT_PRI_SHIFT   (0x0000001Du)
#define CSL_CGEM_IDMA1_COUNT_PRI_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA1_COUNT_INT_MASK    (0x10000000u)
#define CSL_CGEM_IDMA1_COUNT_INT_SHIFT   (0x0000001Cu)
#define CSL_CGEM_IDMA1_COUNT_INT_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA1_COUNT_FILL_MASK   (0x00010000u)
#define CSL_CGEM_IDMA1_COUNT_FILL_SHIFT  (0x00000010u)
#define CSL_CGEM_IDMA1_COUNT_FILL_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMA1_COUNT_COUNT_MASK  (0x0000FFFFu)
#define CSL_CGEM_IDMA1_COUNT_COUNT_SHIFT (0x00000000u)
#define CSL_CGEM_IDMA1_COUNT_COUNT_RESETVAL (0x00000000u)


#define CSL_CGEM_IDMA1_COUNT_RESETVAL    (0x00000000u)

/* CPUARBE */


#define CSL_CGEM_CPUARBE_PRI_MASK        (0x00070000u)
#define CSL_CGEM_CPUARBE_PRI_SHIFT       (0x00000010u)
#define CSL_CGEM_CPUARBE_PRI_RESETVAL    (0x00000000u)


#define CSL_CGEM_CPUARBE_MAXWAIT_MASK    (0x0000003Fu)
#define CSL_CGEM_CPUARBE_MAXWAIT_SHIFT   (0x00000000u)
#define CSL_CGEM_CPUARBE_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_CPUARBE_RESETVAL        (0x00010010u)

/* IDMAARBE */


#define CSL_CGEM_IDMAARBE_MAXWAIT_MASK   (0x0000003Fu)
#define CSL_CGEM_IDMAARBE_MAXWAIT_SHIFT  (0x00000000u)
#define CSL_CGEM_IDMAARBE_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMAARBE_RESETVAL       (0x00000010u)

/* SDMAARBE */


#define CSL_CGEM_SDMAARBE_MAXWAIT_MASK   (0x0000003Fu)
#define CSL_CGEM_SDMAARBE_MAXWAIT_SHIFT  (0x00000000u)
#define CSL_CGEM_SDMAARBE_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_SDMAARBE_RESETVAL       (0x00000001u)

/* ECFGARBE */


#define CSL_CGEM_ECFGARBE_PRI_MASK       (0x00070000u)
#define CSL_CGEM_ECFGARBE_PRI_SHIFT      (0x00000010u)
#define CSL_CGEM_ECFGARBE_PRI_RESETVAL   (0x00000000u)


#define CSL_CGEM_ECFGARBE_RESETVAL       (0x00070000)

/* ICFGMPFAR */

#define CSL_CGEM_ICFGMPFAR_ADDR_MASK     (0xFFFFFFFFu)
#define CSL_CGEM_ICFGMPFAR_ADDR_SHIFT    (0x00000000u)
#define CSL_CGEM_ICFGMPFAR_ADDR_RESETVAL (0x00000000u)

#define CSL_CGEM_ICFGMPFAR_RESETVAL      (0x00000000u)

/* ICFGMPFSR */


#define CSL_CGEM_ICFGMPFSR_FID_MASK      (0x0000FF00u)
#define CSL_CGEM_ICFGMPFSR_FID_SHIFT     (0x00000008u)
#define CSL_CGEM_ICFGMPFSR_FID_RESETVAL  (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_SECE_MASK     (0x00000080u)
#define CSL_CGEM_ICFGMPFSR_SECE_SHIFT    (0x00000007u)
#define CSL_CGEM_ICFGMPFSR_SECE_RESETVAL (0x00000000u)


#define CSL_CGEM_ICFGMPFSR_SR_MASK       (0x00000020u)
#define CSL_CGEM_ICFGMPFSR_SR_SHIFT      (0x00000005u)
#define CSL_CGEM_ICFGMPFSR_SR_RESETVAL   (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_SW_MASK       (0x00000010u)
#define CSL_CGEM_ICFGMPFSR_SW_SHIFT      (0x00000004u)
#define CSL_CGEM_ICFGMPFSR_SW_RESETVAL   (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_SX_MASK       (0x00000008u)
#define CSL_CGEM_ICFGMPFSR_SX_SHIFT      (0x00000003u)
#define CSL_CGEM_ICFGMPFSR_SX_RESETVAL   (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_UR_MASK       (0x00000004u)
#define CSL_CGEM_ICFGMPFSR_UR_SHIFT      (0x00000002u)
#define CSL_CGEM_ICFGMPFSR_UR_RESETVAL   (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_UW_MASK       (0x00000002u)
#define CSL_CGEM_ICFGMPFSR_UW_SHIFT      (0x00000001u)
#define CSL_CGEM_ICFGMPFSR_UW_RESETVAL   (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_UX_MASK       (0x00000001u)
#define CSL_CGEM_ICFGMPFSR_UX_SHIFT      (0x00000000u)
#define CSL_CGEM_ICFGMPFSR_UX_RESETVAL   (0x00000000u)

#define CSL_CGEM_ICFGMPFSR_RESETVAL      (0x00000000u)

/* ICFGMPFCR */


#define CSL_CGEM_ICFGMPFCR_MPFCLR_MASK   (0x00000001u)
#define CSL_CGEM_ICFGMPFCR_MPFCLR_SHIFT  (0x00000000u)
#define CSL_CGEM_ICFGMPFCR_MPFCLR_RESETVAL (0x00000000u)

#define CSL_CGEM_ICFGMPFCR_RESETVAL      (0x00000000u)

/* ECFGERR */

#define CSL_CGEM_ECFGERR_ERR_MASK        (0xE0000000u)
#define CSL_CGEM_ECFGERR_ERR_SHIFT       (0x0000001Du)
#define CSL_CGEM_ECFGERR_ERR_RESETVAL    (0x00000000u)


#define CSL_CGEM_ECFGERR_XID_MASK        (0x00000F00u)
#define CSL_CGEM_ECFGERR_XID_SHIFT       (0x00000008u)
#define CSL_CGEM_ECFGERR_XID_RESETVAL    (0x00000000u)


#define CSL_CGEM_ECFGERR_STAT_MASK       (0x00000007u)
#define CSL_CGEM_ECFGERR_STAT_SHIFT      (0x00000000u)
#define CSL_CGEM_ECFGERR_STAT_RESETVAL   (0x00000000u)

#define CSL_CGEM_ECFGERR_RESETVAL        (0x00000000u)

/* ECFGERRCLR */


#define CSL_CGEM_ECFGERRCLR_CLR_MASK     (0x00000001u)
#define CSL_CGEM_ECFGERRCLR_CLR_SHIFT    (0x00000000u)
#define CSL_CGEM_ECFGERRCLR_CLR_RESETVAL (0x00000000u)

#define CSL_CGEM_ECFGERRCLR_RESETVAL     (0x00000000u)

/* PAMAP0 */


#define CSL_CGEM_PAMAP0_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP0_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP0_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP0_RESETVAL         (0x00000000u)

/* PAMAP1 */


#define CSL_CGEM_PAMAP1_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP1_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP1_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP1_RESETVAL         (0x00000000u)

/* PAMAP2 */


#define CSL_CGEM_PAMAP2_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP2_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP2_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP2_RESETVAL         (0x00000000u)

/* PAMAP3 */


#define CSL_CGEM_PAMAP3_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP3_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP3_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP3_RESETVAL         (0x00000000u)

/* PAMAP4 */


#define CSL_CGEM_PAMAP4_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP4_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP4_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP4_RESETVAL         (0x00000000u)

/* PAMAP5 */


#define CSL_CGEM_PAMAP5_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP5_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP5_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP5_RESETVAL         (0x00000000u)

/* PAMAP6 */


#define CSL_CGEM_PAMAP6_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP6_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP6_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP6_RESETVAL         (0x00000000u)

/* PAMAP7 */


#define CSL_CGEM_PAMAP7_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP7_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP7_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP7_RESETVAL         (0x00000000u)

/* PAMAP8 */


#define CSL_CGEM_PAMAP8_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP8_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP8_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP8_RESETVAL         (0x00000000u)

/* PAMAP9 */


#define CSL_CGEM_PAMAP9_AID_MASK         (0x00000007u)
#define CSL_CGEM_PAMAP9_AID_SHIFT        (0x00000000u)
#define CSL_CGEM_PAMAP9_AID_RESETVAL     (0x00000000u)

#define CSL_CGEM_PAMAP9_RESETVAL         (0x00000000u)

/* PAMAP10 */


#define CSL_CGEM_PAMAP10_AID_MASK        (0x00000007u)
#define CSL_CGEM_PAMAP10_AID_SHIFT       (0x00000000u)
#define CSL_CGEM_PAMAP10_AID_RESETVAL    (0x00000000u)

#define CSL_CGEM_PAMAP10_RESETVAL        (0x00000000u)

/* PAMAP11 */


#define CSL_CGEM_PAMAP11_AID_MASK        (0x00000007u)
#define CSL_CGEM_PAMAP11_AID_SHIFT       (0x00000000u)
#define CSL_CGEM_PAMAP11_AID_RESETVAL    (0x00000000u)

#define CSL_CGEM_PAMAP11_RESETVAL        (0x00000000u)

/* PAMAP12 */


#define CSL_CGEM_PAMAP12_AID_MASK        (0x00000007u)
#define CSL_CGEM_PAMAP12_AID_SHIFT       (0x00000000u)
#define CSL_CGEM_PAMAP12_AID_RESETVAL    (0x00000000u)

#define CSL_CGEM_PAMAP12_RESETVAL        (0x00000000u)

/* PAMAP13 */


#define CSL_CGEM_PAMAP13_AID_MASK        (0x00000007u)
#define CSL_CGEM_PAMAP13_AID_SHIFT       (0x00000000u)
#define CSL_CGEM_PAMAP13_AID_RESETVAL    (0x00000000u)

#define CSL_CGEM_PAMAP13_RESETVAL        (0x00000000u)

/* PAMAP14 */


#define CSL_CGEM_PAMAP14_AID_MASK        (0x00000007u)
#define CSL_CGEM_PAMAP14_AID_SHIFT       (0x00000000u)
#define CSL_CGEM_PAMAP14_AID_RESETVAL    (0x00000000u)

#define CSL_CGEM_PAMAP14_RESETVAL        (0x00000000u)

/* PAMAP15 */


#define CSL_CGEM_PAMAP15_AID_MASK        (0x00000007u)
#define CSL_CGEM_PAMAP15_AID_SHIFT       (0x00000000u)
#define CSL_CGEM_PAMAP15_AID_RESETVAL    (0x00000000u)

#define CSL_CGEM_PAMAP15_RESETVAL        (0x00000000u)

/* L2CFG */


#define CSL_CGEM_L2CFG_NUM_MM_MASK       (0x0F000000u)
#define CSL_CGEM_L2CFG_NUM_MM_SHIFT      (0x00000018u)
#define CSL_CGEM_L2CFG_NUM_MM_RESETVAL   (0x00000000u)


#define CSL_CGEM_L2CFG_MMID_MASK         (0x000F0000u)
#define CSL_CGEM_L2CFG_MMID_SHIFT        (0x00000010u)
#define CSL_CGEM_L2CFG_MMID_RESETVAL     (0x00000000u)


#define CSL_CGEM_L2CFG_L2MODE_MASK       (0x00000007u)
#define CSL_CGEM_L2CFG_L2MODE_SHIFT      (0x00000000u)
#define CSL_CGEM_L2CFG_L2MODE_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2CFG_RESETVAL          (0x01000000u)

/* L1PCFG */


#define CSL_CGEM_L1PCFG_L1PMODE_MASK     (0x00000007u)
#define CSL_CGEM_L1PCFG_L1PMODE_SHIFT    (0x00000000u)
#define CSL_CGEM_L1PCFG_L1PMODE_RESETVAL (0x00000000u)

#define CSL_CGEM_L1PCFG_RESETVAL         (0x00000007u)

/* L1PCC */


#define CSL_CGEM_L1PCC_POPER_MASK        (0x00070000u)
#define CSL_CGEM_L1PCC_POPER_SHIFT       (0x00000010u)
#define CSL_CGEM_L1PCC_POPER_RESETVAL    (0x00000000u)


#define CSL_CGEM_L1PCC_OPER_MASK         (0x00000007u)
#define CSL_CGEM_L1PCC_OPER_SHIFT        (0x00000000u)
#define CSL_CGEM_L1PCC_OPER_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PCC_RESETVAL          (0x00000000u)

/* L1DCFG */


#define CSL_CGEM_L1DCFG_L1DMODE_MASK     (0x00000007u)
#define CSL_CGEM_L1DCFG_L1DMODE_SHIFT    (0x00000000u)
#define CSL_CGEM_L1DCFG_L1DMODE_RESETVAL (0x00000000u)

#define CSL_CGEM_L1DCFG_RESETVAL         (0x00000007u)

/* L1DCC */


#define CSL_CGEM_L1DCC_POPER_MASK        (0x00070000u)
#define CSL_CGEM_L1DCC_POPER_SHIFT       (0x00000010u)
#define CSL_CGEM_L1DCC_POPER_RESETVAL    (0x00000000u)


#define CSL_CGEM_L1DCC_OPER_MASK         (0x00000007u)
#define CSL_CGEM_L1DCC_OPER_SHIFT        (0x00000000u)
#define CSL_CGEM_L1DCC_OPER_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DCC_RESETVAL          (0x00000000u)

/* CPUARBU */


#define CSL_CGEM_CPUARBU_PRI_MASK        (0x00070000u)
#define CSL_CGEM_CPUARBU_PRI_SHIFT       (0x00000010u)
#define CSL_CGEM_CPUARBU_PRI_RESETVAL    (0x00000000u)


#define CSL_CGEM_CPUARBU_MAXWAIT_MASK    (0x0000003Fu)
#define CSL_CGEM_CPUARBU_MAXWAIT_SHIFT   (0x00000000u)
#define CSL_CGEM_CPUARBU_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_CPUARBU_RESETVAL        (0x00010010u)

/* IDMAARBU */


#define CSL_CGEM_IDMAARBU_MAXWAIT_MASK   (0x0000003Fu)
#define CSL_CGEM_IDMAARBU_MAXWAIT_SHIFT  (0x00000000u)
#define CSL_CGEM_IDMAARBU_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMAARBU_RESETVAL       (0x00000010u)

/* SDMAARBU */


#define CSL_CGEM_SDMAARBU_MAXWAIT_MASK   (0x0000003Fu)
#define CSL_CGEM_SDMAARBU_MAXWAIT_SHIFT  (0x00000000u)
#define CSL_CGEM_SDMAARBU_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_SDMAARBU_RESETVAL       (0x00000001u)

/* UCARBU */


#define CSL_CGEM_UCARBU_MAXWAIT_MASK     (0x0000003Fu)
#define CSL_CGEM_UCARBU_MAXWAIT_SHIFT    (0x00000000u)
#define CSL_CGEM_UCARBU_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_UCARBU_RESETVAL         (0x00000020u)

/* MDMAARBU */


#define CSL_CGEM_MDMAARBU_UPRI_MASK      (0x07000000u)
#define CSL_CGEM_MDMAARBU_UPRI_SHIFT     (0x00000018u)
#define CSL_CGEM_MDMAARBU_UPRI_RESETVAL  (0x00000000u)


#define CSL_CGEM_MDMAARBU_PRI_MASK       (0x00070000u)
#define CSL_CGEM_MDMAARBU_PRI_SHIFT      (0x00000010u)
#define CSL_CGEM_MDMAARBU_PRI_RESETVAL   (0x00000000u)


#define CSL_CGEM_MDMAARBU_RESETVAL       (0x06070000u)

/* CPUARBD */


#define CSL_CGEM_CPUARBD_PRI_MASK        (0x00070000u)
#define CSL_CGEM_CPUARBD_PRI_SHIFT       (0x00000010u)
#define CSL_CGEM_CPUARBD_PRI_RESETVAL    (0x00000000u)


#define CSL_CGEM_CPUARBD_MAXWAIT_MASK    (0x0000003Fu)
#define CSL_CGEM_CPUARBD_MAXWAIT_SHIFT   (0x00000000u)
#define CSL_CGEM_CPUARBD_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_CPUARBD_RESETVAL        (0x00010010u)

/* IDMAARBD */


#define CSL_CGEM_IDMAARBD_MAXWAIT_MASK   (0x0000003Fu)
#define CSL_CGEM_IDMAARBD_MAXWAIT_SHIFT  (0x00000000u)
#define CSL_CGEM_IDMAARBD_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_IDMAARBD_RESETVAL       (0x00000010u)

/* SDMAARBD */


#define CSL_CGEM_SDMAARBD_MAXWAIT_MASK   (0x0000003Fu)
#define CSL_CGEM_SDMAARBD_MAXWAIT_SHIFT  (0x00000000u)
#define CSL_CGEM_SDMAARBD_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_SDMAARBD_RESETVAL       (0x00000001u)

/* UCARBD */


#define CSL_CGEM_UCARBD_MAXWAIT_MASK     (0x0000003Fu)
#define CSL_CGEM_UCARBD_MAXWAIT_SHIFT    (0x00000000u)
#define CSL_CGEM_UCARBD_MAXWAIT_RESETVAL (0x00000000u)

#define CSL_CGEM_UCARBD_RESETVAL         (0x00000020u)

/* L2WBAR */

#define CSL_CGEM_L2WBAR_ADDR_MASK        (0xFFFFFFFFu)
#define CSL_CGEM_L2WBAR_ADDR_SHIFT       (0x00000000u)
#define CSL_CGEM_L2WBAR_ADDR_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2WBAR_RESETVAL         (0x00000000u)

/* L2WWC */


#define CSL_CGEM_L2WWC_WC_MASK           (0x0000FFFFu)
#define CSL_CGEM_L2WWC_WC_SHIFT          (0x00000000u)
#define CSL_CGEM_L2WWC_WC_RESETVAL       (0x00000000u)

#define CSL_CGEM_L2WWC_RESETVAL          (0x00000000u)

/* L2WIBAR */

#define CSL_CGEM_L2WIBAR_ADDR_MASK       (0xFFFFFFFFu)
#define CSL_CGEM_L2WIBAR_ADDR_SHIFT      (0x00000000u)
#define CSL_CGEM_L2WIBAR_ADDR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2WIBAR_RESETVAL        (0x00000000u)

/* L2WIWC */


#define CSL_CGEM_L2WIWC_WC_MASK          (0x0000FFFFu)
#define CSL_CGEM_L2WIWC_WC_SHIFT         (0x00000000u)
#define CSL_CGEM_L2WIWC_WC_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2WIWC_RESETVAL         (0x00000000u)

/* L2IBAR */

#define CSL_CGEM_L2IBAR_ADDR_MASK        (0xFFFFFFFFu)
#define CSL_CGEM_L2IBAR_ADDR_SHIFT       (0x00000000u)
#define CSL_CGEM_L2IBAR_ADDR_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2IBAR_RESETVAL         (0x00000000u)

/* L2IWC */


#define CSL_CGEM_L2IWC_WC_MASK           (0x0000FFFFu)
#define CSL_CGEM_L2IWC_WC_SHIFT          (0x00000000u)
#define CSL_CGEM_L2IWC_WC_RESETVAL       (0x00000000u)

#define CSL_CGEM_L2IWC_RESETVAL          (0x00000000u)

/* L1PIBAR */

#define CSL_CGEM_L1PIBAR_ADDR_MASK       (0xFFFFFFFFu)
#define CSL_CGEM_L1PIBAR_ADDR_SHIFT      (0x00000000u)
#define CSL_CGEM_L1PIBAR_ADDR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PIBAR_RESETVAL        (0x00000000u)

/* L1PIWC */


#define CSL_CGEM_L1PIWC_WC_MASK          (0x0000FFFFu)
#define CSL_CGEM_L1PIWC_WC_SHIFT         (0x00000000u)
#define CSL_CGEM_L1PIWC_WC_RESETVAL      (0x00000000u)

#define CSL_CGEM_L1PIWC_RESETVAL         (0x00000000u)

/* L1DWIBAR */

#define CSL_CGEM_L1DWIBAR_ADDR_MASK      (0xFFFFFFFFu)
#define CSL_CGEM_L1DWIBAR_ADDR_SHIFT     (0x00000000u)
#define CSL_CGEM_L1DWIBAR_ADDR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L1DWIBAR_RESETVAL       (0x00000000u)

/* L1DWIWC */


#define CSL_CGEM_L1DWIWC_WC_MASK         (0x0000FFFFu)
#define CSL_CGEM_L1DWIWC_WC_SHIFT        (0x00000000u)
#define CSL_CGEM_L1DWIWC_WC_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DWIWC_RESETVAL        (0x00000000u)

/* L1DWBAR */

#define CSL_CGEM_L1DWBAR_ADDR_MASK       (0xFFFFFFFFu)
#define CSL_CGEM_L1DWBAR_ADDR_SHIFT      (0x00000000u)
#define CSL_CGEM_L1DWBAR_ADDR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DWBAR_RESETVAL        (0x00000000u)

/* L1DWWC */


#define CSL_CGEM_L1DWWC_WC_MASK          (0x0000FFFFu)
#define CSL_CGEM_L1DWWC_WC_SHIFT         (0x00000000u)
#define CSL_CGEM_L1DWWC_WC_RESETVAL      (0x00000000u)

#define CSL_CGEM_L1DWWC_RESETVAL         (0x00000000u)

/* L1DIBAR */

#define CSL_CGEM_L1DIBAR_ADDR_MASK       (0xFFFFFFFFu)
#define CSL_CGEM_L1DIBAR_ADDR_SHIFT      (0x00000000u)
#define CSL_CGEM_L1DIBAR_ADDR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DIBAR_RESETVAL        (0x00000000u)

/* L1DIWC */


#define CSL_CGEM_L1DIWC_WC_MASK          (0x0000FFFFu)
#define CSL_CGEM_L1DIWC_WC_SHIFT         (0x00000000u)
#define CSL_CGEM_L1DIWC_WC_RESETVAL      (0x00000000u)

#define CSL_CGEM_L1DIWC_RESETVAL         (0x00000000u)

/* L2WB */


#define CSL_CGEM_L2WB_C_MASK             (0xFFFFFFFFu)
#define CSL_CGEM_L2WB_C_SHIFT            (0x00000000u)
#define CSL_CGEM_L2WB_C_RESETVAL         (0x00000000u)

#define CSL_CGEM_L2WB_RESETVAL           (0x00000000u)

/* L2WBINV */


#define CSL_CGEM_L2WBINV_C_MASK          (0xFFFFFFFFu)
#define CSL_CGEM_L2WBINV_C_SHIFT         (0x00000000u)
#define CSL_CGEM_L2WBINV_C_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2WBINV_RESETVAL        (0x00000000u)

/* L2INV */


#define CSL_CGEM_L2INV_I_MASK            (0xFFFFFFFFu)
#define CSL_CGEM_L2INV_I_SHIFT           (0x00000000u)
#define CSL_CGEM_L2INV_I_RESETVAL        (0x00000000u)

#define CSL_CGEM_L2INV_RESETVAL          (0x00000000u)

/* L1PINV */


#define CSL_CGEM_L1PINV_I_MASK           (0xFFFFFFFFu)
#define CSL_CGEM_L1PINV_I_SHIFT          (0x00000000u)
#define CSL_CGEM_L1PINV_I_RESETVAL       (0x00000000u)

#define CSL_CGEM_L1PINV_RESETVAL         (0x00000000u)

/* L1DWB */


#define CSL_CGEM_L1DWB_C_MASK            (0xFFFFFFFFu)
#define CSL_CGEM_L1DWB_C_SHIFT           (0x00000000u)
#define CSL_CGEM_L1DWB_C_RESETVAL        (0x00000000u)

#define CSL_CGEM_L1DWB_RESETVAL          (0x00000000u)

/* L1DWBINV */


#define CSL_CGEM_L1DWBINV_C_MASK         (0xFFFFFFFFu)
#define CSL_CGEM_L1DWBINV_C_SHIFT        (0x00000000u)
#define CSL_CGEM_L1DWBINV_C_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DWBINV_RESETVAL       (0x00000000u)

/* L1DINV */


#define CSL_CGEM_L1DINV_I_MASK           (0xFFFFFFFFu)
#define CSL_CGEM_L1DINV_I_SHIFT          (0x00000000u)
#define CSL_CGEM_L1DINV_I_RESETVAL       (0x00000000u)

#define CSL_CGEM_L1DINV_RESETVAL         (0x00000000u)

/* L2EDSTAT */


#define CSL_CGEM_L2EDSTAT_BITPOS_MASK    (0x00FF0000u)
#define CSL_CGEM_L2EDSTAT_BITPOS_SHIFT   (0x00000010u)
#define CSL_CGEM_L2EDSTAT_BITPOS_RESETVAL (0x00000000u)


#define CSL_CGEM_L2EDSTAT_NERR_MASK      (0x00000300u)
#define CSL_CGEM_L2EDSTAT_NERR_SHIFT     (0x00000008u)
#define CSL_CGEM_L2EDSTAT_NERR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2EDSTAT_VERR_MASK      (0x00000080u)
#define CSL_CGEM_L2EDSTAT_VERR_SHIFT     (0x00000007u)
#define CSL_CGEM_L2EDSTAT_VERR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2EDSTAT_DMAERR_MASK    (0x00000040u)
#define CSL_CGEM_L2EDSTAT_DMAERR_SHIFT   (0x00000006u)
#define CSL_CGEM_L2EDSTAT_DMAERR_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDSTAT_PERR_MASK      (0x00000020u)
#define CSL_CGEM_L2EDSTAT_PERR_SHIFT     (0x00000005u)
#define CSL_CGEM_L2EDSTAT_PERR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2EDSTAT_DERR_MASK      (0x00000010u)
#define CSL_CGEM_L2EDSTAT_DERR_SHIFT     (0x00000004u)
#define CSL_CGEM_L2EDSTAT_DERR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2EDSTAT_SUSP_MASK      (0x00000008u)
#define CSL_CGEM_L2EDSTAT_SUSP_SHIFT     (0x00000003u)
#define CSL_CGEM_L2EDSTAT_SUSP_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2EDSTAT_DIS_MASK       (0x00000004u)
#define CSL_CGEM_L2EDSTAT_DIS_SHIFT      (0x00000002u)
#define CSL_CGEM_L2EDSTAT_DIS_RESETVAL   (0x00000000u)


#define CSL_CGEM_L2EDSTAT_EN_MASK        (0x00000001u)
#define CSL_CGEM_L2EDSTAT_EN_SHIFT       (0x00000000u)
#define CSL_CGEM_L2EDSTAT_EN_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2EDSTAT_RESETVAL       (0x00000004u)

/* L2EDCMD */


#define CSL_CGEM_L2EDCMD_VCLR_MASK       (0x00000080u)
#define CSL_CGEM_L2EDCMD_VCLR_SHIFT      (0x00000007u)
#define CSL_CGEM_L2EDCMD_VCLR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDCMD_DMACLR_MASK     (0x00000040u)
#define CSL_CGEM_L2EDCMD_DMACLR_SHIFT    (0x00000006u)
#define CSL_CGEM_L2EDCMD_DMACLR_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDCMD_PCLR_MASK       (0x00000020u)
#define CSL_CGEM_L2EDCMD_PCLR_SHIFT      (0x00000005u)
#define CSL_CGEM_L2EDCMD_PCLR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDCMD_DCLR_MASK       (0x00000010u)
#define CSL_CGEM_L2EDCMD_DCLR_SHIFT      (0x00000004u)
#define CSL_CGEM_L2EDCMD_DCLR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDCMD_SUSP_MASK       (0x00000008u)
#define CSL_CGEM_L2EDCMD_SUSP_SHIFT      (0x00000003u)
#define CSL_CGEM_L2EDCMD_SUSP_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDCMD_DIS_MASK        (0x00000004u)
#define CSL_CGEM_L2EDCMD_DIS_SHIFT       (0x00000002u)
#define CSL_CGEM_L2EDCMD_DIS_RESETVAL    (0x00000000u)


#define CSL_CGEM_L2EDCMD_EN_MASK         (0x00000001u)
#define CSL_CGEM_L2EDCMD_EN_SHIFT        (0x00000000u)
#define CSL_CGEM_L2EDCMD_EN_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2EDCMD_RESETVAL        (0x00000001u)

/* L2EDADDR */

#define CSL_CGEM_L2EDADDR_ADDR_MASK      (0xFFFFFFE0u)
#define CSL_CGEM_L2EDADDR_ADDR_SHIFT     (0x00000005u)
#define CSL_CGEM_L2EDADDR_ADDR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2EDADDR_WAY_MASK       (0x00000018u)
#define CSL_CGEM_L2EDADDR_WAY_SHIFT      (0x00000003u)
#define CSL_CGEM_L2EDADDR_WAY_RESETVAL   (0x00000000u)


#define CSL_CGEM_L2EDADDR_RAM_MASK       (0x00000001u)
#define CSL_CGEM_L2EDADDR_RAM_SHIFT      (0x00000000u)
#define CSL_CGEM_L2EDADDR_RAM_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDADDR_RESETVAL       (0x00000000u)

/* L2EDCPEC */


#define CSL_CGEM_L2EDCPEC_CNT_MASK       (0x000000FFu)
#define CSL_CGEM_L2EDCPEC_CNT_SHIFT      (0x00000000u)
#define CSL_CGEM_L2EDCPEC_CNT_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDCPEC_RESETVAL       (0x00000000u)

/* L2EDCNEC */


#define CSL_CGEM_L2EDCNEC_CNT_MASK       (0x000000FFu)
#define CSL_CGEM_L2EDCNEC_CNT_SHIFT      (0x00000000u)
#define CSL_CGEM_L2EDCNEC_CNT_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2EDCNEC_RESETVAL       (0x00000000u)

/* MDMAERR */

#define CSL_CGEM_MDMAERR_ERR_MASK        (0xE0000000u)
#define CSL_CGEM_MDMAERR_ERR_SHIFT       (0x0000001Du)
#define CSL_CGEM_MDMAERR_ERR_RESETVAL    (0x00000000u)


#define CSL_CGEM_MDMAERR_XID_MASK        (0x00000F00u)
#define CSL_CGEM_MDMAERR_XID_SHIFT       (0x00000008u)
#define CSL_CGEM_MDMAERR_XID_RESETVAL    (0x00000000u)


#define CSL_CGEM_MDMAERR_STAT_MASK       (0x00000007u)
#define CSL_CGEM_MDMAERR_STAT_SHIFT      (0x00000000u)
#define CSL_CGEM_MDMAERR_STAT_RESETVAL   (0x00000000u)

#define CSL_CGEM_MDMAERR_RESETVAL        (0x00000000u)

/* MDMAERRCLR */


#define CSL_CGEM_MDMAERRCLR_CLR_MASK     (0x00000001u)
#define CSL_CGEM_MDMAERRCLR_CLR_SHIFT    (0x00000000u)
#define CSL_CGEM_MDMAERRCLR_CLR_RESETVAL (0x00000000u)

#define CSL_CGEM_MDMAERRCLR_RESETVAL     (0x00000000u)

/* L2EDCEN */


#define CSL_CGEM_L2EDCEN_SDMAEN_MASK     (0x00000010u)
#define CSL_CGEM_L2EDCEN_SDMAEN_SHIFT    (0x00000004u)
#define CSL_CGEM_L2EDCEN_SDMAEN_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDCEN_PL2SEN_MASK     (0x00000008u)
#define CSL_CGEM_L2EDCEN_PL2SEN_SHIFT    (0x00000003u)
#define CSL_CGEM_L2EDCEN_PL2SEN_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDCEN_DL2SEN_MASK     (0x00000004u)
#define CSL_CGEM_L2EDCEN_DL2SEN_SHIFT    (0x00000002u)
#define CSL_CGEM_L2EDCEN_DL2SEN_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDCEN_PL2CEN_MASK     (0x00000002u)
#define CSL_CGEM_L2EDCEN_PL2CEN_SHIFT    (0x00000001u)
#define CSL_CGEM_L2EDCEN_PL2CEN_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDCEN_DL2CEN_MASK     (0x00000001u)
#define CSL_CGEM_L2EDCEN_DL2CEN_SHIFT    (0x00000000u)
#define CSL_CGEM_L2EDCEN_DL2CEN_RESETVAL (0x00000000u)

#define CSL_CGEM_L2EDCEN_RESETVAL        (0x0000001Fu)

/* L1PEDSTAT */


#define CSL_CGEM_L1PEDSTAT_DMAERR_MASK   (0x00000040u)
#define CSL_CGEM_L1PEDSTAT_DMAERR_SHIFT  (0x00000006u)
#define CSL_CGEM_L1PEDSTAT_DMAERR_RESETVAL (0x00000000u)

#define CSL_CGEM_L1PEDSTAT_PERR_MASK     (0x00000020u)
#define CSL_CGEM_L1PEDSTAT_PERR_SHIFT    (0x00000005u)
#define CSL_CGEM_L1PEDSTAT_PERR_RESETVAL (0x00000000u)


#define CSL_CGEM_L1PEDSTAT_SUSP_MASK     (0x00000008u)
#define CSL_CGEM_L1PEDSTAT_SUSP_SHIFT    (0x00000003u)
#define CSL_CGEM_L1PEDSTAT_SUSP_RESETVAL (0x00000000u)

#define CSL_CGEM_L1PEDSTAT_DIS_MASK      (0x00000004u)
#define CSL_CGEM_L1PEDSTAT_DIS_SHIFT     (0x00000002u)
#define CSL_CGEM_L1PEDSTAT_DIS_RESETVAL  (0x00000000u)


#define CSL_CGEM_L1PEDSTAT_EN_MASK       (0x00000001u)
#define CSL_CGEM_L1PEDSTAT_EN_SHIFT      (0x00000000u)
#define CSL_CGEM_L1PEDSTAT_EN_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PEDSTAT_RESETVAL      (0x00000004u)

/* L1PEDCMD */


#define CSL_CGEM_L1PEDCMD_DMACLR_MASK    (0x00000040u)
#define CSL_CGEM_L1PEDCMD_DMACLR_SHIFT   (0x00000006u)
#define CSL_CGEM_L1PEDCMD_DMACLR_RESETVAL (0x00000000u)

#define CSL_CGEM_L1PEDCMD_PCLR_MASK      (0x00000020u)
#define CSL_CGEM_L1PEDCMD_PCLR_SHIFT     (0x00000005u)
#define CSL_CGEM_L1PEDCMD_PCLR_RESETVAL  (0x00000000u)


#define CSL_CGEM_L1PEDCMD_SUSP_MASK      (0x00000008u)
#define CSL_CGEM_L1PEDCMD_SUSP_SHIFT     (0x00000003u)
#define CSL_CGEM_L1PEDCMD_SUSP_RESETVAL  (0x00000000u)

#define CSL_CGEM_L1PEDCMD_DIS_MASK       (0x00000004u)
#define CSL_CGEM_L1PEDCMD_DIS_SHIFT      (0x00000002u)
#define CSL_CGEM_L1PEDCMD_DIS_RESETVAL   (0x00000000u)


#define CSL_CGEM_L1PEDCMD_EN_MASK        (0x00000001u)
#define CSL_CGEM_L1PEDCMD_EN_SHIFT       (0x00000000u)
#define CSL_CGEM_L1PEDCMD_EN_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PEDCMD_RESETVAL       (0x00000000u)

/* L1PEDADDR */

#define CSL_CGEM_L1PEDADDR_ADDR_MASK     (0xFFFFFFE0u)
#define CSL_CGEM_L1PEDADDR_ADDR_SHIFT    (0x00000005u)
#define CSL_CGEM_L1PEDADDR_ADDR_RESETVAL (0x00000000u)


#define CSL_CGEM_L1PEDADDR_RAM_MASK      (0x00000001u)
#define CSL_CGEM_L1PEDADDR_RAM_SHIFT     (0x00000000u)
#define CSL_CGEM_L1PEDADDR_RAM_RESETVAL  (0x00000000u)

#define CSL_CGEM_L1PEDADDR_RESETVAL      (0x00000000u)

/* MAR0 */


#define CSL_CGEM_MAR0_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR0_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR0_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR0_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR0_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR0_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR0_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR0_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR0_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR0_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR0_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR0_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR0_RESETVAL           (0x00000000u)

/* MAR1 */


#define CSL_CGEM_MAR1_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR1_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR1_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR1_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR1_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR1_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR1_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR1_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR1_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR1_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR1_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR1_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR1_RESETVAL           (0x00000000u)

/* MAR2 */


#define CSL_CGEM_MAR2_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR2_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR2_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR2_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR2_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR2_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR2_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR2_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR2_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR2_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR2_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR2_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR2_RESETVAL           (0x00000000u)

/* MAR3 */


#define CSL_CGEM_MAR3_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR3_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR3_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR3_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR3_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR3_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR3_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR3_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR3_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR3_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR3_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR3_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR3_RESETVAL           (0x00000000u)

/* MAR4 */


#define CSL_CGEM_MAR4_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR4_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR4_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR4_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR4_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR4_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR4_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR4_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR4_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR4_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR4_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR4_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR4_RESETVAL           (0x00000000u)

/* MAR5 */


#define CSL_CGEM_MAR5_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR5_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR5_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR5_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR5_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR5_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR5_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR5_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR5_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR5_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR5_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR5_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR5_RESETVAL           (0x00000000u)

/* MAR6 */


#define CSL_CGEM_MAR6_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR6_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR6_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR6_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR6_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR6_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR6_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR6_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR6_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR6_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR6_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR6_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR6_RESETVAL           (0x00000000u)

/* MAR7 */


#define CSL_CGEM_MAR7_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR7_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR7_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR7_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR7_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR7_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR7_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR7_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR7_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR7_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR7_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR7_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR7_RESETVAL           (0x00000000u)

/* MAR8 */


#define CSL_CGEM_MAR8_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR8_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR8_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR8_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR8_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR8_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR8_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR8_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR8_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR8_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR8_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR8_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR8_RESETVAL           (0x00000000u)

/* MAR9 */


#define CSL_CGEM_MAR9_PFX_MASK           (0x00000008u)
#define CSL_CGEM_MAR9_PFX_SHIFT          (0x00000003u)
#define CSL_CGEM_MAR9_PFX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR9_PCX_MASK           (0x00000004u)
#define CSL_CGEM_MAR9_PCX_SHIFT          (0x00000002u)
#define CSL_CGEM_MAR9_PCX_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR9_WTE_MASK           (0x00000002u)
#define CSL_CGEM_MAR9_WTE_SHIFT          (0x00000001u)
#define CSL_CGEM_MAR9_WTE_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR9_PC_MASK            (0x00000001u)
#define CSL_CGEM_MAR9_PC_SHIFT           (0x00000000u)
#define CSL_CGEM_MAR9_PC_RESETVAL        (0x00000000u)

#define CSL_CGEM_MAR9_RESETVAL           (0x00000000u)

/* MAR10 */


#define CSL_CGEM_MAR10_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR10_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR10_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR10_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR10_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR10_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR10_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR10_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR10_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR10_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR10_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR10_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR10_RESETVAL          (0x00000000u)

/* MAR11 */


#define CSL_CGEM_MAR11_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR11_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR11_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR11_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR11_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR11_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR11_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR11_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR11_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR11_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR11_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR11_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR11_RESETVAL          (0x00000000u)

/* MAR12 */


#define CSL_CGEM_MAR12_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR12_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR12_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR12_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR12_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR12_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR12_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR12_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR12_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR12_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR12_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR12_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR12_RESETVAL          (0x00000000u)

/* MAR13 */


#define CSL_CGEM_MAR13_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR13_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR13_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR13_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR13_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR13_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR13_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR13_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR13_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR13_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR13_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR13_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR13_RESETVAL          (0x00000000u)

/* MAR14 */


#define CSL_CGEM_MAR14_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR14_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR14_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR14_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR14_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR14_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR14_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR14_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR14_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR14_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR14_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR14_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR14_RESETVAL          (0x00000000u)

/* MAR15 */


#define CSL_CGEM_MAR15_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR15_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR15_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR15_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR15_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR15_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR15_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR15_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR15_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR15_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR15_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR15_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR15_RESETVAL          (0x00000000u)

/* MAR16 */


#define CSL_CGEM_MAR16_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR16_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR16_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR16_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR16_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR16_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR16_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR16_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR16_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR16_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR16_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR16_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR16_RESETVAL          (0x00000000u)

/* MAR17 */


#define CSL_CGEM_MAR17_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR17_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR17_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR17_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR17_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR17_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR17_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR17_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR17_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR17_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR17_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR17_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR17_RESETVAL          (0x00000000u)

/* MAR18 */


#define CSL_CGEM_MAR18_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR18_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR18_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR18_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR18_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR18_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR18_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR18_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR18_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR18_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR18_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR18_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR18_RESETVAL          (0x00000000u)

/* MAR19 */


#define CSL_CGEM_MAR19_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR19_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR19_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR19_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR19_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR19_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR19_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR19_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR19_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR19_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR19_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR19_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR19_RESETVAL          (0x00000000u)

/* MAR20 */


#define CSL_CGEM_MAR20_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR20_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR20_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR20_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR20_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR20_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR20_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR20_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR20_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR20_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR20_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR20_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR20_RESETVAL          (0x00000000u)

/* MAR21 */


#define CSL_CGEM_MAR21_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR21_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR21_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR21_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR21_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR21_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR21_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR21_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR21_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR21_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR21_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR21_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR21_RESETVAL          (0x00000000u)

/* MAR22 */


#define CSL_CGEM_MAR22_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR22_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR22_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR22_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR22_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR22_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR22_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR22_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR22_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR22_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR22_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR22_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR22_RESETVAL          (0x00000000u)

/* MAR23 */


#define CSL_CGEM_MAR23_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR23_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR23_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR23_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR23_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR23_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR23_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR23_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR23_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR23_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR23_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR23_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR23_RESETVAL          (0x00000000u)

/* MAR24 */


#define CSL_CGEM_MAR24_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR24_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR24_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR24_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR24_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR24_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR24_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR24_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR24_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR24_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR24_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR24_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR24_RESETVAL          (0x00000000u)

/* MAR25 */


#define CSL_CGEM_MAR25_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR25_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR25_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR25_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR25_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR25_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR25_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR25_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR25_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR25_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR25_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR25_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR25_RESETVAL          (0x00000000u)

/* MAR26 */


#define CSL_CGEM_MAR26_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR26_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR26_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR26_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR26_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR26_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR26_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR26_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR26_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR26_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR26_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR26_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR26_RESETVAL          (0x00000000u)

/* MAR27 */


#define CSL_CGEM_MAR27_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR27_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR27_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR27_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR27_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR27_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR27_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR27_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR27_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR27_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR27_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR27_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR27_RESETVAL          (0x00000000u)

/* MAR28 */


#define CSL_CGEM_MAR28_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR28_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR28_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR28_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR28_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR28_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR28_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR28_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR28_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR28_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR28_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR28_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR28_RESETVAL          (0x00000000u)

/* MAR29 */


#define CSL_CGEM_MAR29_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR29_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR29_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR29_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR29_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR29_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR29_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR29_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR29_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR29_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR29_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR29_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR29_RESETVAL          (0x00000000u)

/* MAR30 */


#define CSL_CGEM_MAR30_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR30_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR30_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR30_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR30_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR30_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR30_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR30_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR30_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR30_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR30_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR30_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR30_RESETVAL          (0x00000000u)

/* MAR31 */


#define CSL_CGEM_MAR31_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR31_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR31_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR31_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR31_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR31_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR31_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR31_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR31_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR31_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR31_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR31_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR31_RESETVAL          (0x00000000u)

/* MAR32 */


#define CSL_CGEM_MAR32_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR32_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR32_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR32_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR32_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR32_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR32_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR32_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR32_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR32_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR32_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR32_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR32_RESETVAL          (0x00000000u)

/* MAR33 */


#define CSL_CGEM_MAR33_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR33_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR33_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR33_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR33_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR33_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR33_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR33_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR33_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR33_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR33_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR33_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR33_RESETVAL          (0x00000000u)

/* MAR34 */


#define CSL_CGEM_MAR34_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR34_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR34_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR34_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR34_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR34_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR34_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR34_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR34_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR34_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR34_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR34_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR34_RESETVAL          (0x00000000u)

/* MAR35 */


#define CSL_CGEM_MAR35_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR35_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR35_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR35_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR35_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR35_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR35_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR35_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR35_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR35_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR35_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR35_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR35_RESETVAL          (0x00000000u)

/* MAR36 */


#define CSL_CGEM_MAR36_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR36_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR36_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR36_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR36_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR36_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR36_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR36_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR36_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR36_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR36_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR36_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR36_RESETVAL          (0x00000000u)

/* MAR37 */


#define CSL_CGEM_MAR37_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR37_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR37_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR37_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR37_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR37_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR37_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR37_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR37_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR37_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR37_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR37_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR37_RESETVAL          (0x00000000u)

/* MAR38 */


#define CSL_CGEM_MAR38_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR38_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR38_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR38_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR38_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR38_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR38_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR38_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR38_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR38_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR38_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR38_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR38_RESETVAL          (0x00000000u)

/* MAR39 */


#define CSL_CGEM_MAR39_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR39_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR39_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR39_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR39_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR39_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR39_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR39_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR39_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR39_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR39_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR39_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR39_RESETVAL          (0x00000000u)

/* MAR40 */


#define CSL_CGEM_MAR40_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR40_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR40_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR40_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR40_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR40_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR40_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR40_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR40_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR40_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR40_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR40_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR40_RESETVAL          (0x00000000u)

/* MAR41 */


#define CSL_CGEM_MAR41_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR41_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR41_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR41_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR41_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR41_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR41_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR41_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR41_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR41_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR41_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR41_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR41_RESETVAL          (0x00000000u)

/* MAR42 */


#define CSL_CGEM_MAR42_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR42_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR42_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR42_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR42_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR42_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR42_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR42_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR42_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR42_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR42_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR42_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR42_RESETVAL          (0x00000000u)

/* MAR43 */


#define CSL_CGEM_MAR43_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR43_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR43_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR43_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR43_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR43_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR43_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR43_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR43_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR43_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR43_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR43_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR43_RESETVAL          (0x00000000u)

/* MAR44 */


#define CSL_CGEM_MAR44_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR44_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR44_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR44_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR44_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR44_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR44_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR44_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR44_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR44_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR44_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR44_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR44_RESETVAL          (0x00000000u)

/* MAR45 */


#define CSL_CGEM_MAR45_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR45_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR45_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR45_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR45_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR45_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR45_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR45_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR45_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR45_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR45_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR45_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR45_RESETVAL          (0x00000000u)

/* MAR46 */


#define CSL_CGEM_MAR46_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR46_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR46_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR46_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR46_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR46_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR46_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR46_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR46_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR46_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR46_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR46_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR46_RESETVAL          (0x00000000u)

/* MAR47 */


#define CSL_CGEM_MAR47_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR47_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR47_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR47_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR47_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR47_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR47_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR47_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR47_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR47_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR47_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR47_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR47_RESETVAL          (0x00000000u)

/* MAR48 */


#define CSL_CGEM_MAR48_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR48_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR48_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR48_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR48_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR48_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR48_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR48_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR48_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR48_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR48_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR48_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR48_RESETVAL          (0x00000000u)

/* MAR49 */


#define CSL_CGEM_MAR49_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR49_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR49_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR49_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR49_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR49_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR49_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR49_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR49_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR49_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR49_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR49_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR49_RESETVAL          (0x00000000u)

/* MAR50 */


#define CSL_CGEM_MAR50_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR50_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR50_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR50_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR50_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR50_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR50_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR50_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR50_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR50_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR50_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR50_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR50_RESETVAL          (0x00000000u)

/* MAR51 */


#define CSL_CGEM_MAR51_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR51_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR51_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR51_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR51_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR51_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR51_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR51_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR51_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR51_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR51_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR51_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR51_RESETVAL          (0x00000000u)

/* MAR52 */


#define CSL_CGEM_MAR52_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR52_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR52_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR52_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR52_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR52_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR52_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR52_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR52_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR52_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR52_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR52_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR52_RESETVAL          (0x00000000u)

/* MAR53 */


#define CSL_CGEM_MAR53_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR53_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR53_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR53_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR53_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR53_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR53_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR53_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR53_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR53_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR53_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR53_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR53_RESETVAL          (0x00000000u)

/* MAR54 */


#define CSL_CGEM_MAR54_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR54_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR54_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR54_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR54_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR54_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR54_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR54_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR54_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR54_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR54_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR54_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR54_RESETVAL          (0x00000000u)

/* MAR55 */


#define CSL_CGEM_MAR55_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR55_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR55_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR55_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR55_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR55_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR55_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR55_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR55_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR55_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR55_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR55_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR55_RESETVAL          (0x00000000u)

/* MAR56 */


#define CSL_CGEM_MAR56_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR56_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR56_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR56_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR56_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR56_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR56_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR56_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR56_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR56_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR56_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR56_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR56_RESETVAL          (0x00000000u)

/* MAR57 */


#define CSL_CGEM_MAR57_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR57_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR57_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR57_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR57_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR57_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR57_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR57_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR57_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR57_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR57_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR57_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR57_RESETVAL          (0x00000000u)

/* MAR58 */


#define CSL_CGEM_MAR58_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR58_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR58_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR58_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR58_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR58_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR58_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR58_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR58_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR58_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR58_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR58_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR58_RESETVAL          (0x00000000u)

/* MAR59 */


#define CSL_CGEM_MAR59_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR59_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR59_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR59_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR59_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR59_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR59_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR59_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR59_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR59_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR59_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR59_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR59_RESETVAL          (0x00000000u)

/* MAR60 */


#define CSL_CGEM_MAR60_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR60_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR60_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR60_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR60_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR60_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR60_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR60_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR60_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR60_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR60_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR60_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR60_RESETVAL          (0x00000000u)

/* MAR61 */


#define CSL_CGEM_MAR61_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR61_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR61_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR61_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR61_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR61_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR61_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR61_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR61_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR61_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR61_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR61_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR61_RESETVAL          (0x00000000u)

/* MAR62 */


#define CSL_CGEM_MAR62_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR62_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR62_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR62_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR62_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR62_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR62_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR62_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR62_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR62_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR62_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR62_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR62_RESETVAL          (0x00000000u)

/* MAR63 */


#define CSL_CGEM_MAR63_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR63_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR63_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR63_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR63_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR63_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR63_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR63_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR63_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR63_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR63_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR63_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR63_RESETVAL          (0x00000000u)

/* MAR64 */


#define CSL_CGEM_MAR64_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR64_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR64_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR64_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR64_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR64_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR64_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR64_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR64_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR64_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR64_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR64_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR64_RESETVAL          (0x00000000u)

/* MAR65 */


#define CSL_CGEM_MAR65_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR65_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR65_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR65_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR65_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR65_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR65_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR65_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR65_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR65_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR65_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR65_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR65_RESETVAL          (0x00000000u)

/* MAR66 */


#define CSL_CGEM_MAR66_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR66_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR66_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR66_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR66_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR66_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR66_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR66_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR66_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR66_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR66_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR66_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR66_RESETVAL          (0x00000000u)

/* MAR67 */


#define CSL_CGEM_MAR67_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR67_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR67_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR67_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR67_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR67_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR67_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR67_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR67_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR67_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR67_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR67_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR67_RESETVAL          (0x00000000u)

/* MAR68 */


#define CSL_CGEM_MAR68_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR68_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR68_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR68_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR68_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR68_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR68_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR68_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR68_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR68_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR68_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR68_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR68_RESETVAL          (0x00000000u)

/* MAR69 */


#define CSL_CGEM_MAR69_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR69_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR69_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR69_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR69_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR69_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR69_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR69_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR69_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR69_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR69_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR69_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR69_RESETVAL          (0x00000000u)

/* MAR70 */


#define CSL_CGEM_MAR70_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR70_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR70_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR70_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR70_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR70_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR70_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR70_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR70_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR70_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR70_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR70_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR70_RESETVAL          (0x00000000u)

/* MAR71 */


#define CSL_CGEM_MAR71_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR71_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR71_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR71_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR71_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR71_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR71_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR71_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR71_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR71_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR71_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR71_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR71_RESETVAL          (0x00000000u)

/* MAR72 */


#define CSL_CGEM_MAR72_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR72_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR72_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR72_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR72_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR72_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR72_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR72_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR72_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR72_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR72_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR72_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR72_RESETVAL          (0x00000000u)

/* MAR73 */


#define CSL_CGEM_MAR73_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR73_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR73_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR73_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR73_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR73_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR73_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR73_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR73_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR73_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR73_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR73_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR73_RESETVAL          (0x00000000u)

/* MAR74 */


#define CSL_CGEM_MAR74_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR74_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR74_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR74_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR74_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR74_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR74_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR74_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR74_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR74_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR74_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR74_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR74_RESETVAL          (0x00000000u)

/* MAR75 */


#define CSL_CGEM_MAR75_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR75_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR75_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR75_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR75_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR75_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR75_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR75_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR75_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR75_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR75_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR75_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR75_RESETVAL          (0x00000000u)

/* MAR76 */


#define CSL_CGEM_MAR76_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR76_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR76_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR76_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR76_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR76_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR76_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR76_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR76_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR76_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR76_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR76_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR76_RESETVAL          (0x00000000u)

/* MAR77 */


#define CSL_CGEM_MAR77_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR77_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR77_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR77_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR77_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR77_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR77_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR77_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR77_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR77_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR77_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR77_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR77_RESETVAL          (0x00000000u)

/* MAR78 */


#define CSL_CGEM_MAR78_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR78_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR78_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR78_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR78_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR78_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR78_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR78_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR78_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR78_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR78_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR78_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR78_RESETVAL          (0x00000000u)

/* MAR79 */


#define CSL_CGEM_MAR79_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR79_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR79_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR79_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR79_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR79_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR79_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR79_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR79_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR79_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR79_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR79_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR79_RESETVAL          (0x00000000u)

/* MAR80 */


#define CSL_CGEM_MAR80_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR80_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR80_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR80_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR80_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR80_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR80_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR80_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR80_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR80_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR80_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR80_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR80_RESETVAL          (0x00000000u)

/* MAR81 */


#define CSL_CGEM_MAR81_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR81_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR81_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR81_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR81_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR81_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR81_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR81_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR81_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR81_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR81_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR81_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR81_RESETVAL          (0x00000000u)

/* MAR82 */


#define CSL_CGEM_MAR82_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR82_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR82_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR82_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR82_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR82_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR82_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR82_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR82_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR82_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR82_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR82_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR82_RESETVAL          (0x00000000u)

/* MAR83 */


#define CSL_CGEM_MAR83_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR83_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR83_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR83_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR83_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR83_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR83_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR83_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR83_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR83_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR83_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR83_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR83_RESETVAL          (0x00000000u)

/* MAR84 */


#define CSL_CGEM_MAR84_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR84_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR84_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR84_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR84_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR84_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR84_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR84_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR84_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR84_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR84_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR84_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR84_RESETVAL          (0x00000000u)

/* MAR85 */


#define CSL_CGEM_MAR85_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR85_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR85_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR85_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR85_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR85_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR85_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR85_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR85_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR85_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR85_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR85_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR85_RESETVAL          (0x00000000u)

/* MAR86 */


#define CSL_CGEM_MAR86_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR86_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR86_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR86_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR86_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR86_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR86_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR86_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR86_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR86_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR86_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR86_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR86_RESETVAL          (0x00000000u)

/* MAR87 */


#define CSL_CGEM_MAR87_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR87_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR87_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR87_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR87_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR87_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR87_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR87_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR87_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR87_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR87_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR87_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR87_RESETVAL          (0x00000000u)

/* MAR88 */


#define CSL_CGEM_MAR88_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR88_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR88_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR88_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR88_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR88_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR88_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR88_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR88_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR88_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR88_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR88_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR88_RESETVAL          (0x00000000u)

/* MAR89 */


#define CSL_CGEM_MAR89_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR89_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR89_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR89_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR89_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR89_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR89_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR89_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR89_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR89_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR89_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR89_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR89_RESETVAL          (0x00000000u)

/* MAR90 */


#define CSL_CGEM_MAR90_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR90_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR90_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR90_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR90_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR90_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR90_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR90_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR90_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR90_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR90_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR90_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR90_RESETVAL          (0x00000000u)

/* MAR91 */


#define CSL_CGEM_MAR91_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR91_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR91_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR91_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR91_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR91_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR91_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR91_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR91_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR91_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR91_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR91_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR91_RESETVAL          (0x00000000u)

/* MAR92 */


#define CSL_CGEM_MAR92_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR92_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR92_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR92_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR92_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR92_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR92_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR92_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR92_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR92_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR92_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR92_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR92_RESETVAL          (0x00000000u)

/* MAR93 */


#define CSL_CGEM_MAR93_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR93_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR93_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR93_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR93_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR93_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR93_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR93_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR93_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR93_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR93_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR93_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR93_RESETVAL          (0x00000000u)

/* MAR94 */


#define CSL_CGEM_MAR94_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR94_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR94_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR94_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR94_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR94_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR94_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR94_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR94_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR94_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR94_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR94_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR94_RESETVAL          (0x00000000u)

/* MAR95 */


#define CSL_CGEM_MAR95_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR95_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR95_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR95_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR95_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR95_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR95_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR95_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR95_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR95_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR95_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR95_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR95_RESETVAL          (0x00000000u)

/* MAR96 */


#define CSL_CGEM_MAR96_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR96_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR96_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR96_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR96_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR96_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR96_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR96_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR96_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR96_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR96_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR96_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR96_RESETVAL          (0x00000000u)

/* MAR97 */


#define CSL_CGEM_MAR97_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR97_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR97_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR97_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR97_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR97_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR97_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR97_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR97_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR97_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR97_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR97_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR97_RESETVAL          (0x00000000u)

/* MAR98 */


#define CSL_CGEM_MAR98_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR98_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR98_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR98_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR98_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR98_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR98_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR98_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR98_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR98_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR98_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR98_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR98_RESETVAL          (0x00000000u)

/* MAR99 */


#define CSL_CGEM_MAR99_PFX_MASK          (0x00000008u)
#define CSL_CGEM_MAR99_PFX_SHIFT         (0x00000003u)
#define CSL_CGEM_MAR99_PFX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR99_PCX_MASK          (0x00000004u)
#define CSL_CGEM_MAR99_PCX_SHIFT         (0x00000002u)
#define CSL_CGEM_MAR99_PCX_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR99_WTE_MASK          (0x00000002u)
#define CSL_CGEM_MAR99_WTE_SHIFT         (0x00000001u)
#define CSL_CGEM_MAR99_WTE_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR99_PC_MASK           (0x00000001u)
#define CSL_CGEM_MAR99_PC_SHIFT          (0x00000000u)
#define CSL_CGEM_MAR99_PC_RESETVAL       (0x00000000u)

#define CSL_CGEM_MAR99_RESETVAL          (0x00000000u)

/* MAR100 */


#define CSL_CGEM_MAR100_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR100_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR100_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR100_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR100_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR100_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR100_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR100_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR100_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR100_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR100_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR100_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR100_RESETVAL         (0x00000000u)

/* MAR101 */


#define CSL_CGEM_MAR101_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR101_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR101_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR101_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR101_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR101_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR101_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR101_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR101_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR101_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR101_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR101_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR101_RESETVAL         (0x00000000u)

/* MAR102 */


#define CSL_CGEM_MAR102_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR102_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR102_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR102_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR102_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR102_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR102_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR102_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR102_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR102_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR102_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR102_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR102_RESETVAL         (0x00000000u)

/* MAR103 */


#define CSL_CGEM_MAR103_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR103_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR103_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR103_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR103_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR103_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR103_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR103_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR103_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR103_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR103_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR103_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR103_RESETVAL         (0x00000000u)

/* MAR104 */


#define CSL_CGEM_MAR104_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR104_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR104_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR104_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR104_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR104_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR104_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR104_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR104_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR104_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR104_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR104_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR104_RESETVAL         (0x00000000u)

/* MAR105 */


#define CSL_CGEM_MAR105_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR105_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR105_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR105_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR105_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR105_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR105_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR105_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR105_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR105_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR105_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR105_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR105_RESETVAL         (0x00000000u)

/* MAR106 */


#define CSL_CGEM_MAR106_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR106_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR106_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR106_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR106_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR106_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR106_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR106_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR106_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR106_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR106_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR106_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR106_RESETVAL         (0x00000000u)

/* MAR107 */


#define CSL_CGEM_MAR107_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR107_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR107_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR107_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR107_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR107_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR107_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR107_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR107_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR107_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR107_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR107_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR107_RESETVAL         (0x00000000u)

/* MAR108 */


#define CSL_CGEM_MAR108_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR108_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR108_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR108_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR108_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR108_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR108_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR108_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR108_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR108_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR108_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR108_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR108_RESETVAL         (0x00000000u)

/* MAR109 */


#define CSL_CGEM_MAR109_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR109_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR109_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR109_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR109_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR109_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR109_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR109_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR109_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR109_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR109_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR109_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR109_RESETVAL         (0x00000000u)

/* MAR110 */


#define CSL_CGEM_MAR110_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR110_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR110_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR110_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR110_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR110_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR110_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR110_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR110_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR110_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR110_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR110_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR110_RESETVAL         (0x00000000u)

/* MAR111 */


#define CSL_CGEM_MAR111_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR111_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR111_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR111_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR111_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR111_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR111_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR111_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR111_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR111_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR111_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR111_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR111_RESETVAL         (0x00000000u)

/* MAR112 */


#define CSL_CGEM_MAR112_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR112_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR112_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR112_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR112_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR112_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR112_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR112_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR112_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR112_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR112_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR112_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR112_RESETVAL         (0x00000000u)

/* MAR113 */


#define CSL_CGEM_MAR113_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR113_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR113_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR113_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR113_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR113_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR113_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR113_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR113_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR113_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR113_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR113_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR113_RESETVAL         (0x00000000u)

/* MAR114 */


#define CSL_CGEM_MAR114_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR114_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR114_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR114_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR114_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR114_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR114_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR114_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR114_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR114_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR114_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR114_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR114_RESETVAL         (0x00000000u)

/* MAR115 */


#define CSL_CGEM_MAR115_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR115_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR115_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR115_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR115_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR115_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR115_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR115_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR115_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR115_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR115_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR115_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR115_RESETVAL         (0x00000000u)

/* MAR116 */


#define CSL_CGEM_MAR116_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR116_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR116_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR116_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR116_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR116_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR116_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR116_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR116_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR116_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR116_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR116_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR116_RESETVAL         (0x00000000u)

/* MAR117 */


#define CSL_CGEM_MAR117_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR117_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR117_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR117_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR117_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR117_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR117_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR117_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR117_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR117_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR117_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR117_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR117_RESETVAL         (0x00000000u)

/* MAR118 */


#define CSL_CGEM_MAR118_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR118_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR118_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR118_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR118_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR118_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR118_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR118_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR118_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR118_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR118_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR118_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR118_RESETVAL         (0x00000000u)

/* MAR119 */


#define CSL_CGEM_MAR119_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR119_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR119_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR119_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR119_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR119_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR119_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR119_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR119_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR119_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR119_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR119_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR119_RESETVAL         (0x00000000u)

/* MAR120 */


#define CSL_CGEM_MAR120_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR120_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR120_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR120_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR120_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR120_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR120_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR120_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR120_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR120_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR120_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR120_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR120_RESETVAL         (0x00000000u)

/* MAR121 */


#define CSL_CGEM_MAR121_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR121_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR121_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR121_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR121_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR121_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR121_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR121_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR121_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR121_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR121_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR121_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR121_RESETVAL         (0x00000000u)

/* MAR122 */


#define CSL_CGEM_MAR122_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR122_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR122_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR122_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR122_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR122_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR122_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR122_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR122_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR122_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR122_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR122_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR122_RESETVAL         (0x00000000u)

/* MAR123 */


#define CSL_CGEM_MAR123_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR123_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR123_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR123_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR123_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR123_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR123_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR123_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR123_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR123_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR123_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR123_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR123_RESETVAL         (0x00000000u)

/* MAR124 */


#define CSL_CGEM_MAR124_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR124_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR124_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR124_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR124_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR124_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR124_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR124_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR124_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR124_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR124_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR124_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR124_RESETVAL         (0x00000000u)

/* MAR125 */


#define CSL_CGEM_MAR125_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR125_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR125_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR125_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR125_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR125_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR125_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR125_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR125_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR125_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR125_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR125_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR125_RESETVAL         (0x00000000u)

/* MAR126 */


#define CSL_CGEM_MAR126_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR126_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR126_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR126_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR126_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR126_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR126_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR126_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR126_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR126_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR126_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR126_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR126_RESETVAL         (0x00000000u)

/* MAR127 */


#define CSL_CGEM_MAR127_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR127_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR127_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR127_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR127_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR127_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR127_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR127_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR127_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR127_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR127_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR127_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR127_RESETVAL         (0x00000000u)

/* MAR128 */


#define CSL_CGEM_MAR128_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR128_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR128_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR128_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR128_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR128_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR128_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR128_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR128_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR128_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR128_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR128_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR128_RESETVAL         (0x00000000u)

/* MAR129 */


#define CSL_CGEM_MAR129_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR129_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR129_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR129_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR129_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR129_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR129_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR129_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR129_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR129_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR129_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR129_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR129_RESETVAL         (0x00000000u)

/* MAR130 */


#define CSL_CGEM_MAR130_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR130_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR130_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR130_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR130_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR130_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR130_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR130_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR130_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR130_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR130_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR130_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR130_RESETVAL         (0x00000000u)

/* MAR131 */


#define CSL_CGEM_MAR131_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR131_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR131_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR131_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR131_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR131_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR131_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR131_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR131_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR131_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR131_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR131_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR131_RESETVAL         (0x00000000u)

/* MAR132 */


#define CSL_CGEM_MAR132_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR132_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR132_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR132_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR132_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR132_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR132_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR132_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR132_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR132_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR132_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR132_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR132_RESETVAL         (0x00000000u)

/* MAR133 */


#define CSL_CGEM_MAR133_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR133_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR133_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR133_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR133_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR133_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR133_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR133_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR133_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR133_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR133_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR133_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR133_RESETVAL         (0x00000000u)

/* MAR134 */


#define CSL_CGEM_MAR134_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR134_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR134_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR134_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR134_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR134_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR134_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR134_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR134_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR134_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR134_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR134_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR134_RESETVAL         (0x00000000u)

/* MAR135 */


#define CSL_CGEM_MAR135_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR135_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR135_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR135_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR135_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR135_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR135_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR135_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR135_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR135_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR135_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR135_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR135_RESETVAL         (0x00000000u)

/* MAR136 */


#define CSL_CGEM_MAR136_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR136_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR136_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR136_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR136_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR136_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR136_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR136_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR136_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR136_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR136_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR136_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR136_RESETVAL         (0x00000000u)

/* MAR137 */


#define CSL_CGEM_MAR137_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR137_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR137_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR137_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR137_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR137_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR137_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR137_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR137_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR137_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR137_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR137_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR137_RESETVAL         (0x00000000u)

/* MAR138 */


#define CSL_CGEM_MAR138_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR138_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR138_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR138_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR138_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR138_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR138_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR138_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR138_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR138_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR138_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR138_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR138_RESETVAL         (0x00000000u)

/* MAR139 */


#define CSL_CGEM_MAR139_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR139_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR139_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR139_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR139_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR139_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR139_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR139_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR139_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR139_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR139_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR139_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR139_RESETVAL         (0x00000000u)

/* MAR140 */


#define CSL_CGEM_MAR140_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR140_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR140_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR140_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR140_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR140_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR140_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR140_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR140_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR140_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR140_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR140_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR140_RESETVAL         (0x00000000u)

/* MAR141 */


#define CSL_CGEM_MAR141_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR141_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR141_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR141_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR141_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR141_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR141_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR141_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR141_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR141_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR141_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR141_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR141_RESETVAL         (0x00000000u)

/* MAR142 */


#define CSL_CGEM_MAR142_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR142_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR142_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR142_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR142_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR142_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR142_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR142_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR142_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR142_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR142_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR142_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR142_RESETVAL         (0x00000000u)

/* MAR143 */


#define CSL_CGEM_MAR143_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR143_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR143_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR143_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR143_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR143_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR143_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR143_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR143_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR143_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR143_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR143_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR143_RESETVAL         (0x00000000u)

/* MAR144 */


#define CSL_CGEM_MAR144_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR144_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR144_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR144_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR144_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR144_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR144_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR144_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR144_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR144_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR144_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR144_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR144_RESETVAL         (0x00000000u)

/* MAR145 */


#define CSL_CGEM_MAR145_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR145_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR145_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR145_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR145_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR145_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR145_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR145_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR145_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR145_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR145_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR145_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR145_RESETVAL         (0x00000000u)

/* MAR146 */


#define CSL_CGEM_MAR146_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR146_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR146_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR146_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR146_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR146_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR146_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR146_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR146_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR146_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR146_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR146_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR146_RESETVAL         (0x00000000u)

/* MAR147 */


#define CSL_CGEM_MAR147_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR147_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR147_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR147_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR147_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR147_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR147_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR147_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR147_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR147_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR147_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR147_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR147_RESETVAL         (0x00000000u)

/* MAR148 */


#define CSL_CGEM_MAR148_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR148_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR148_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR148_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR148_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR148_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR148_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR148_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR148_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR148_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR148_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR148_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR148_RESETVAL         (0x00000000u)

/* MAR149 */


#define CSL_CGEM_MAR149_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR149_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR149_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR149_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR149_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR149_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR149_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR149_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR149_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR149_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR149_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR149_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR149_RESETVAL         (0x00000000u)

/* MAR150 */


#define CSL_CGEM_MAR150_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR150_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR150_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR150_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR150_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR150_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR150_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR150_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR150_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR150_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR150_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR150_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR150_RESETVAL         (0x00000000u)

/* MAR151 */


#define CSL_CGEM_MAR151_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR151_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR151_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR151_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR151_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR151_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR151_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR151_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR151_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR151_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR151_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR151_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR151_RESETVAL         (0x00000000u)

/* MAR152 */


#define CSL_CGEM_MAR152_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR152_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR152_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR152_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR152_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR152_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR152_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR152_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR152_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR152_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR152_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR152_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR152_RESETVAL         (0x00000000u)

/* MAR153 */


#define CSL_CGEM_MAR153_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR153_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR153_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR153_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR153_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR153_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR153_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR153_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR153_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR153_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR153_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR153_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR153_RESETVAL         (0x00000000u)

/* MAR154 */


#define CSL_CGEM_MAR154_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR154_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR154_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR154_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR154_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR154_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR154_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR154_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR154_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR154_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR154_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR154_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR154_RESETVAL         (0x00000000u)

/* MAR155 */


#define CSL_CGEM_MAR155_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR155_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR155_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR155_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR155_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR155_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR155_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR155_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR155_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR155_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR155_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR155_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR155_RESETVAL         (0x00000000u)

/* MAR156 */


#define CSL_CGEM_MAR156_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR156_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR156_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR156_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR156_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR156_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR156_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR156_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR156_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR156_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR156_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR156_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR156_RESETVAL         (0x00000000u)

/* MAR157 */


#define CSL_CGEM_MAR157_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR157_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR157_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR157_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR157_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR157_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR157_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR157_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR157_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR157_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR157_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR157_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR157_RESETVAL         (0x00000000u)

/* MAR158 */


#define CSL_CGEM_MAR158_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR158_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR158_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR158_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR158_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR158_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR158_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR158_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR158_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR158_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR158_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR158_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR158_RESETVAL         (0x00000000u)

/* MAR159 */


#define CSL_CGEM_MAR159_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR159_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR159_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR159_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR159_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR159_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR159_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR159_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR159_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR159_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR159_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR159_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR159_RESETVAL         (0x00000000u)

/* MAR160 */


#define CSL_CGEM_MAR160_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR160_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR160_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR160_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR160_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR160_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR160_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR160_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR160_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR160_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR160_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR160_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR160_RESETVAL         (0x00000000u)

/* MAR161 */


#define CSL_CGEM_MAR161_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR161_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR161_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR161_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR161_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR161_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR161_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR161_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR161_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR161_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR161_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR161_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR161_RESETVAL         (0x00000000u)

/* MAR162 */


#define CSL_CGEM_MAR162_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR162_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR162_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR162_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR162_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR162_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR162_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR162_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR162_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR162_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR162_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR162_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR162_RESETVAL         (0x00000000u)

/* MAR163 */


#define CSL_CGEM_MAR163_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR163_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR163_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR163_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR163_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR163_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR163_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR163_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR163_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR163_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR163_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR163_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR163_RESETVAL         (0x00000000u)

/* MAR164 */


#define CSL_CGEM_MAR164_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR164_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR164_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR164_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR164_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR164_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR164_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR164_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR164_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR164_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR164_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR164_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR164_RESETVAL         (0x00000000u)

/* MAR165 */


#define CSL_CGEM_MAR165_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR165_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR165_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR165_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR165_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR165_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR165_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR165_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR165_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR165_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR165_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR165_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR165_RESETVAL         (0x00000000u)

/* MAR166 */


#define CSL_CGEM_MAR166_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR166_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR166_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR166_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR166_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR166_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR166_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR166_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR166_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR166_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR166_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR166_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR166_RESETVAL         (0x00000000u)

/* MAR167 */


#define CSL_CGEM_MAR167_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR167_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR167_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR167_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR167_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR167_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR167_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR167_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR167_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR167_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR167_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR167_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR167_RESETVAL         (0x00000000u)

/* MAR168 */


#define CSL_CGEM_MAR168_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR168_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR168_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR168_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR168_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR168_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR168_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR168_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR168_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR168_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR168_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR168_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR168_RESETVAL         (0x00000000u)

/* MAR169 */


#define CSL_CGEM_MAR169_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR169_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR169_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR169_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR169_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR169_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR169_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR169_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR169_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR169_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR169_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR169_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR169_RESETVAL         (0x00000000u)

/* MAR170 */


#define CSL_CGEM_MAR170_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR170_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR170_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR170_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR170_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR170_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR170_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR170_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR170_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR170_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR170_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR170_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR170_RESETVAL         (0x00000000u)

/* MAR171 */


#define CSL_CGEM_MAR171_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR171_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR171_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR171_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR171_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR171_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR171_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR171_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR171_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR171_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR171_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR171_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR171_RESETVAL         (0x00000000u)

/* MAR172 */


#define CSL_CGEM_MAR172_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR172_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR172_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR172_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR172_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR172_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR172_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR172_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR172_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR172_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR172_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR172_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR172_RESETVAL         (0x00000000u)

/* MAR173 */


#define CSL_CGEM_MAR173_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR173_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR173_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR173_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR173_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR173_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR173_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR173_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR173_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR173_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR173_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR173_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR173_RESETVAL         (0x00000000u)

/* MAR174 */


#define CSL_CGEM_MAR174_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR174_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR174_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR174_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR174_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR174_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR174_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR174_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR174_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR174_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR174_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR174_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR174_RESETVAL         (0x00000000u)

/* MAR175 */


#define CSL_CGEM_MAR175_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR175_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR175_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR175_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR175_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR175_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR175_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR175_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR175_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR175_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR175_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR175_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR175_RESETVAL         (0x00000000u)

/* MAR176 */


#define CSL_CGEM_MAR176_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR176_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR176_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR176_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR176_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR176_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR176_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR176_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR176_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR176_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR176_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR176_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR176_RESETVAL         (0x00000000u)

/* MAR177 */


#define CSL_CGEM_MAR177_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR177_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR177_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR177_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR177_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR177_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR177_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR177_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR177_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR177_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR177_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR177_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR177_RESETVAL         (0x00000000u)

/* MAR178 */


#define CSL_CGEM_MAR178_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR178_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR178_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR178_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR178_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR178_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR178_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR178_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR178_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR178_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR178_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR178_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR178_RESETVAL         (0x00000000u)

/* MAR179 */


#define CSL_CGEM_MAR179_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR179_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR179_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR179_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR179_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR179_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR179_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR179_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR179_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR179_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR179_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR179_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR179_RESETVAL         (0x00000000u)

/* MAR180 */


#define CSL_CGEM_MAR180_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR180_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR180_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR180_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR180_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR180_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR180_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR180_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR180_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR180_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR180_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR180_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR180_RESETVAL         (0x00000000u)

/* MAR181 */


#define CSL_CGEM_MAR181_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR181_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR181_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR181_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR181_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR181_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR181_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR181_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR181_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR181_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR181_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR181_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR181_RESETVAL         (0x00000000u)

/* MAR182 */


#define CSL_CGEM_MAR182_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR182_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR182_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR182_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR182_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR182_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR182_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR182_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR182_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR182_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR182_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR182_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR182_RESETVAL         (0x00000000u)

/* MAR183 */


#define CSL_CGEM_MAR183_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR183_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR183_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR183_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR183_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR183_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR183_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR183_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR183_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR183_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR183_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR183_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR183_RESETVAL         (0x00000000u)

/* MAR184 */


#define CSL_CGEM_MAR184_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR184_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR184_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR184_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR184_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR184_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR184_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR184_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR184_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR184_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR184_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR184_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR184_RESETVAL         (0x00000000u)

/* MAR185 */


#define CSL_CGEM_MAR185_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR185_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR185_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR185_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR185_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR185_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR185_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR185_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR185_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR185_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR185_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR185_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR185_RESETVAL         (0x00000000u)

/* MAR186 */


#define CSL_CGEM_MAR186_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR186_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR186_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR186_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR186_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR186_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR186_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR186_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR186_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR186_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR186_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR186_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR186_RESETVAL         (0x00000000u)

/* MAR187 */


#define CSL_CGEM_MAR187_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR187_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR187_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR187_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR187_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR187_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR187_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR187_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR187_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR187_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR187_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR187_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR187_RESETVAL         (0x00000000u)

/* MAR188 */


#define CSL_CGEM_MAR188_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR188_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR188_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR188_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR188_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR188_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR188_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR188_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR188_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR188_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR188_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR188_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR188_RESETVAL         (0x00000000u)

/* MAR189 */


#define CSL_CGEM_MAR189_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR189_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR189_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR189_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR189_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR189_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR189_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR189_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR189_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR189_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR189_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR189_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR189_RESETVAL         (0x00000000u)

/* MAR190 */


#define CSL_CGEM_MAR190_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR190_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR190_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR190_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR190_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR190_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR190_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR190_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR190_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR190_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR190_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR190_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR190_RESETVAL         (0x00000000u)

/* MAR191 */


#define CSL_CGEM_MAR191_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR191_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR191_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR191_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR191_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR191_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR191_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR191_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR191_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR191_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR191_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR191_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR191_RESETVAL         (0x00000000u)

/* MAR192 */


#define CSL_CGEM_MAR192_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR192_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR192_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR192_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR192_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR192_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR192_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR192_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR192_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR192_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR192_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR192_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR192_RESETVAL         (0x00000000u)

/* MAR193 */


#define CSL_CGEM_MAR193_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR193_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR193_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR193_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR193_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR193_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR193_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR193_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR193_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR193_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR193_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR193_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR193_RESETVAL         (0x00000000u)

/* MAR194 */


#define CSL_CGEM_MAR194_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR194_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR194_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR194_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR194_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR194_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR194_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR194_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR194_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR194_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR194_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR194_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR194_RESETVAL         (0x00000000u)

/* MAR195 */


#define CSL_CGEM_MAR195_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR195_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR195_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR195_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR195_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR195_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR195_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR195_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR195_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR195_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR195_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR195_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR195_RESETVAL         (0x00000000u)

/* MAR196 */


#define CSL_CGEM_MAR196_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR196_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR196_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR196_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR196_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR196_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR196_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR196_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR196_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR196_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR196_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR196_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR196_RESETVAL         (0x00000000u)

/* MAR197 */


#define CSL_CGEM_MAR197_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR197_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR197_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR197_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR197_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR197_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR197_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR197_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR197_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR197_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR197_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR197_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR197_RESETVAL         (0x00000000u)

/* MAR198 */


#define CSL_CGEM_MAR198_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR198_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR198_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR198_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR198_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR198_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR198_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR198_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR198_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR198_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR198_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR198_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR198_RESETVAL         (0x00000000u)

/* MAR199 */


#define CSL_CGEM_MAR199_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR199_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR199_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR199_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR199_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR199_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR199_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR199_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR199_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR199_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR199_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR199_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR199_RESETVAL         (0x00000000u)

/* MAR200 */


#define CSL_CGEM_MAR200_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR200_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR200_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR200_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR200_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR200_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR200_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR200_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR200_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR200_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR200_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR200_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR200_RESETVAL         (0x00000000u)

/* MAR201 */


#define CSL_CGEM_MAR201_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR201_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR201_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR201_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR201_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR201_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR201_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR201_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR201_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR201_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR201_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR201_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR201_RESETVAL         (0x00000000u)

/* MAR202 */


#define CSL_CGEM_MAR202_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR202_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR202_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR202_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR202_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR202_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR202_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR202_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR202_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR202_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR202_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR202_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR202_RESETVAL         (0x00000000u)

/* MAR203 */


#define CSL_CGEM_MAR203_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR203_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR203_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR203_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR203_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR203_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR203_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR203_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR203_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR203_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR203_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR203_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR203_RESETVAL         (0x00000000u)

/* MAR204 */


#define CSL_CGEM_MAR204_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR204_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR204_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR204_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR204_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR204_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR204_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR204_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR204_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR204_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR204_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR204_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR204_RESETVAL         (0x00000000u)

/* MAR205 */


#define CSL_CGEM_MAR205_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR205_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR205_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR205_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR205_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR205_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR205_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR205_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR205_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR205_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR205_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR205_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR205_RESETVAL         (0x00000000u)

/* MAR206 */


#define CSL_CGEM_MAR206_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR206_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR206_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR206_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR206_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR206_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR206_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR206_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR206_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR206_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR206_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR206_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR206_RESETVAL         (0x00000000u)

/* MAR207 */


#define CSL_CGEM_MAR207_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR207_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR207_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR207_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR207_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR207_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR207_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR207_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR207_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR207_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR207_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR207_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR207_RESETVAL         (0x00000000u)

/* MAR208 */


#define CSL_CGEM_MAR208_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR208_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR208_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR208_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR208_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR208_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR208_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR208_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR208_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR208_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR208_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR208_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR208_RESETVAL         (0x00000000u)

/* MAR209 */


#define CSL_CGEM_MAR209_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR209_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR209_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR209_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR209_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR209_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR209_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR209_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR209_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR209_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR209_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR209_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR209_RESETVAL         (0x00000000u)

/* MAR210 */


#define CSL_CGEM_MAR210_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR210_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR210_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR210_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR210_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR210_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR210_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR210_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR210_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR210_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR210_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR210_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR210_RESETVAL         (0x00000000u)

/* MAR211 */


#define CSL_CGEM_MAR211_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR211_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR211_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR211_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR211_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR211_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR211_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR211_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR211_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR211_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR211_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR211_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR211_RESETVAL         (0x00000000u)

/* MAR212 */


#define CSL_CGEM_MAR212_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR212_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR212_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR212_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR212_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR212_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR212_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR212_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR212_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR212_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR212_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR212_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR212_RESETVAL         (0x00000000u)

/* MAR213 */


#define CSL_CGEM_MAR213_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR213_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR213_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR213_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR213_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR213_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR213_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR213_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR213_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR213_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR213_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR213_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR213_RESETVAL         (0x00000000u)

/* MAR214 */


#define CSL_CGEM_MAR214_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR214_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR214_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR214_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR214_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR214_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR214_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR214_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR214_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR214_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR214_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR214_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR214_RESETVAL         (0x00000000u)

/* MAR215 */


#define CSL_CGEM_MAR215_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR215_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR215_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR215_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR215_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR215_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR215_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR215_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR215_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR215_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR215_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR215_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR215_RESETVAL         (0x00000000u)

/* MAR216 */


#define CSL_CGEM_MAR216_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR216_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR216_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR216_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR216_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR216_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR216_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR216_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR216_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR216_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR216_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR216_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR216_RESETVAL         (0x00000000u)

/* MAR217 */


#define CSL_CGEM_MAR217_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR217_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR217_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR217_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR217_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR217_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR217_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR217_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR217_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR217_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR217_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR217_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR217_RESETVAL         (0x00000000u)

/* MAR218 */


#define CSL_CGEM_MAR218_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR218_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR218_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR218_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR218_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR218_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR218_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR218_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR218_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR218_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR218_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR218_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR218_RESETVAL         (0x00000000u)

/* MAR219 */


#define CSL_CGEM_MAR219_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR219_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR219_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR219_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR219_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR219_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR219_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR219_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR219_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR219_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR219_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR219_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR219_RESETVAL         (0x00000000u)

/* MAR220 */


#define CSL_CGEM_MAR220_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR220_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR220_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR220_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR220_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR220_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR220_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR220_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR220_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR220_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR220_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR220_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR220_RESETVAL         (0x00000000u)

/* MAR221 */


#define CSL_CGEM_MAR221_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR221_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR221_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR221_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR221_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR221_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR221_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR221_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR221_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR221_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR221_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR221_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR221_RESETVAL         (0x00000000u)

/* MAR222 */


#define CSL_CGEM_MAR222_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR222_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR222_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR222_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR222_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR222_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR222_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR222_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR222_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR222_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR222_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR222_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR222_RESETVAL         (0x00000000u)

/* MAR223 */


#define CSL_CGEM_MAR223_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR223_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR223_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR223_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR223_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR223_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR223_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR223_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR223_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR223_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR223_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR223_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR223_RESETVAL         (0x00000000u)

/* MAR224 */


#define CSL_CGEM_MAR224_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR224_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR224_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR224_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR224_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR224_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR224_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR224_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR224_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR224_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR224_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR224_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR224_RESETVAL         (0x00000000u)

/* MAR225 */


#define CSL_CGEM_MAR225_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR225_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR225_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR225_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR225_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR225_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR225_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR225_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR225_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR225_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR225_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR225_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR225_RESETVAL         (0x00000000u)

/* MAR226 */


#define CSL_CGEM_MAR226_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR226_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR226_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR226_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR226_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR226_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR226_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR226_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR226_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR226_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR226_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR226_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR226_RESETVAL         (0x00000000u)

/* MAR227 */


#define CSL_CGEM_MAR227_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR227_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR227_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR227_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR227_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR227_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR227_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR227_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR227_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR227_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR227_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR227_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR227_RESETVAL         (0x00000000u)

/* MAR228 */


#define CSL_CGEM_MAR228_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR228_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR228_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR228_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR228_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR228_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR228_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR228_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR228_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR228_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR228_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR228_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR228_RESETVAL         (0x00000000u)

/* MAR229 */


#define CSL_CGEM_MAR229_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR229_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR229_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR229_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR229_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR229_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR229_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR229_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR229_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR229_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR229_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR229_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR229_RESETVAL         (0x00000000u)

/* MAR230 */


#define CSL_CGEM_MAR230_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR230_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR230_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR230_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR230_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR230_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR230_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR230_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR230_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR230_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR230_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR230_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR230_RESETVAL         (0x00000000u)

/* MAR231 */


#define CSL_CGEM_MAR231_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR231_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR231_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR231_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR231_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR231_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR231_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR231_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR231_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR231_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR231_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR231_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR231_RESETVAL         (0x00000000u)

/* MAR232 */


#define CSL_CGEM_MAR232_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR232_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR232_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR232_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR232_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR232_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR232_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR232_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR232_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR232_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR232_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR232_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR232_RESETVAL         (0x00000000u)

/* MAR233 */


#define CSL_CGEM_MAR233_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR233_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR233_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR233_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR233_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR233_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR233_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR233_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR233_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR233_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR233_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR233_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR233_RESETVAL         (0x00000000u)

/* MAR234 */


#define CSL_CGEM_MAR234_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR234_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR234_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR234_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR234_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR234_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR234_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR234_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR234_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR234_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR234_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR234_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR234_RESETVAL         (0x00000000u)

/* MAR235 */


#define CSL_CGEM_MAR235_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR235_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR235_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR235_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR235_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR235_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR235_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR235_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR235_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR235_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR235_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR235_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR235_RESETVAL         (0x00000000u)

/* MAR236 */


#define CSL_CGEM_MAR236_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR236_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR236_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR236_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR236_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR236_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR236_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR236_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR236_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR236_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR236_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR236_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR236_RESETVAL         (0x00000000u)

/* MAR237 */


#define CSL_CGEM_MAR237_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR237_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR237_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR237_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR237_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR237_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR237_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR237_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR237_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR237_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR237_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR237_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR237_RESETVAL         (0x00000000u)

/* MAR238 */


#define CSL_CGEM_MAR238_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR238_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR238_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR238_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR238_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR238_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR238_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR238_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR238_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR238_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR238_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR238_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR238_RESETVAL         (0x00000000u)

/* MAR239 */


#define CSL_CGEM_MAR239_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR239_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR239_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR239_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR239_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR239_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR239_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR239_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR239_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR239_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR239_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR239_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR239_RESETVAL         (0x00000000u)

/* MAR240 */


#define CSL_CGEM_MAR240_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR240_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR240_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR240_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR240_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR240_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR240_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR240_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR240_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR240_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR240_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR240_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR240_RESETVAL         (0x00000000u)

/* MAR241 */


#define CSL_CGEM_MAR241_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR241_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR241_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR241_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR241_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR241_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR241_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR241_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR241_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR241_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR241_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR241_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR241_RESETVAL         (0x00000000u)

/* MAR242 */


#define CSL_CGEM_MAR242_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR242_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR242_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR242_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR242_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR242_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR242_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR242_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR242_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR242_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR242_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR242_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR242_RESETVAL         (0x00000000u)

/* MAR243 */


#define CSL_CGEM_MAR243_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR243_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR243_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR243_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR243_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR243_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR243_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR243_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR243_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR243_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR243_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR243_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR243_RESETVAL         (0x00000000u)

/* MAR244 */


#define CSL_CGEM_MAR244_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR244_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR244_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR244_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR244_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR244_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR244_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR244_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR244_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR244_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR244_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR244_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR244_RESETVAL         (0x00000000u)

/* MAR245 */


#define CSL_CGEM_MAR245_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR245_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR245_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR245_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR245_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR245_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR245_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR245_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR245_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR245_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR245_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR245_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR245_RESETVAL         (0x00000000u)

/* MAR246 */


#define CSL_CGEM_MAR246_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR246_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR246_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR246_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR246_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR246_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR246_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR246_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR246_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR246_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR246_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR246_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR246_RESETVAL         (0x00000000u)

/* MAR247 */


#define CSL_CGEM_MAR247_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR247_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR247_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR247_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR247_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR247_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR247_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR247_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR247_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR247_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR247_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR247_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR247_RESETVAL         (0x00000000u)

/* MAR248 */


#define CSL_CGEM_MAR248_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR248_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR248_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR248_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR248_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR248_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR248_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR248_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR248_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR248_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR248_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR248_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR248_RESETVAL         (0x00000000u)

/* MAR249 */


#define CSL_CGEM_MAR249_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR249_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR249_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR249_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR249_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR249_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR249_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR249_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR249_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR249_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR249_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR249_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR249_RESETVAL         (0x00000000u)

/* MAR250 */


#define CSL_CGEM_MAR250_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR250_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR250_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR250_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR250_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR250_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR250_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR250_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR250_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR250_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR250_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR250_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR250_RESETVAL         (0x00000000u)

/* MAR251 */


#define CSL_CGEM_MAR251_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR251_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR251_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR251_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR251_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR251_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR251_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR251_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR251_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR251_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR251_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR251_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR251_RESETVAL         (0x00000000u)

/* MAR252 */


#define CSL_CGEM_MAR252_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR252_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR252_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR252_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR252_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR252_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR252_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR252_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR252_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR252_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR252_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR252_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR252_RESETVAL         (0x00000000u)

/* MAR253 */


#define CSL_CGEM_MAR253_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR253_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR253_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR253_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR253_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR253_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR253_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR253_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR253_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR253_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR253_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR253_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR253_RESETVAL         (0x00000000u)

/* MAR254 */


#define CSL_CGEM_MAR254_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR254_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR254_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR254_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR254_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR254_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR254_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR254_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR254_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR254_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR254_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR254_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR254_RESETVAL         (0x00000000u)

/* MAR255 */


#define CSL_CGEM_MAR255_PFX_MASK         (0x00000008u)
#define CSL_CGEM_MAR255_PFX_SHIFT        (0x00000003u)
#define CSL_CGEM_MAR255_PFX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR255_PCX_MASK         (0x00000004u)
#define CSL_CGEM_MAR255_PCX_SHIFT        (0x00000002u)
#define CSL_CGEM_MAR255_PCX_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR255_WTE_MASK         (0x00000002u)
#define CSL_CGEM_MAR255_WTE_SHIFT        (0x00000001u)
#define CSL_CGEM_MAR255_WTE_RESETVAL     (0x00000000u)

#define CSL_CGEM_MAR255_PC_MASK          (0x00000001u)
#define CSL_CGEM_MAR255_PC_SHIFT         (0x00000000u)
#define CSL_CGEM_MAR255_PC_RESETVAL      (0x00000000u)

#define CSL_CGEM_MAR255_RESETVAL         (0x00000000u)

/* L2MPFAR */

#define CSL_CGEM_L2MPFAR_ADDR_MASK       (0xFFFFFFFFu)
#define CSL_CGEM_L2MPFAR_ADDR_SHIFT      (0x00000000u)
#define CSL_CGEM_L2MPFAR_ADDR_RESETVAL   (0x00000000u)

#define CSL_CGEM_L2MPFAR_RESETVAL        (0x00000000u)

/* L2MPFSR */


#define CSL_CGEM_L2MPFSR_FID_MASK        (0x0000FE00u)
#define CSL_CGEM_L2MPFSR_FID_SHIFT       (0x00000009u)
#define CSL_CGEM_L2MPFSR_FID_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPFSR_LOCAL_MASK      (0x00000100u)
#define CSL_CGEM_L2MPFSR_LOCAL_SHIFT     (0x00000008u)
#define CSL_CGEM_L2MPFSR_LOCAL_RESETVAL  (0x00000000u)

#define CSL_CGEM_L2MPFSR_NS_MASK         (0x00000080u)
#define CSL_CGEM_L2MPFSR_NS_SHIFT        (0x00000007u)
#define CSL_CGEM_L2MPFSR_NS_RESETVAL     (0x00000000u)


#define CSL_CGEM_L2MPFSR_SR_MASK         (0x00000020u)
#define CSL_CGEM_L2MPFSR_SR_SHIFT        (0x00000005u)
#define CSL_CGEM_L2MPFSR_SR_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2MPFSR_SW_MASK         (0x00000010u)
#define CSL_CGEM_L2MPFSR_SW_SHIFT        (0x00000004u)
#define CSL_CGEM_L2MPFSR_SW_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2MPFSR_SX_MASK         (0x00000008u)
#define CSL_CGEM_L2MPFSR_SX_SHIFT        (0x00000003u)
#define CSL_CGEM_L2MPFSR_SX_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2MPFSR_UR_MASK         (0x00000004u)
#define CSL_CGEM_L2MPFSR_UR_SHIFT        (0x00000002u)
#define CSL_CGEM_L2MPFSR_UR_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2MPFSR_UW_MASK         (0x00000002u)
#define CSL_CGEM_L2MPFSR_UW_SHIFT        (0x00000001u)
#define CSL_CGEM_L2MPFSR_UW_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2MPFSR_UX_MASK         (0x00000001u)
#define CSL_CGEM_L2MPFSR_UX_SHIFT        (0x00000000u)
#define CSL_CGEM_L2MPFSR_UX_RESETVAL     (0x00000000u)

#define CSL_CGEM_L2MPFSR_RESETVAL        (0x00000000u)

/* L2MPFCR */


#define CSL_CGEM_L2MPFCR_MPFCLR_MASK     (0x00000001u)
#define CSL_CGEM_L2MPFCR_MPFCLR_SHIFT    (0x00000000u)
#define CSL_CGEM_L2MPFCR_MPFCLR_RESETVAL (0x00000000u)

#define CSL_CGEM_L2MPFCR_RESETVAL        (0x00000000u)

/* L2MPPA */


#define CSL_CGEM_L2MPPA_AID5_MASK        (0x00008000u)
#define CSL_CGEM_L2MPPA_AID5_SHIFT       (0x0000000Fu)
#define CSL_CGEM_L2MPPA_AID5_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_AID4_MASK        (0x00004000u)
#define CSL_CGEM_L2MPPA_AID4_SHIFT       (0x0000000Eu)
#define CSL_CGEM_L2MPPA_AID4_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_AID3_MASK        (0x00002000u)
#define CSL_CGEM_L2MPPA_AID3_SHIFT       (0x0000000Du)
#define CSL_CGEM_L2MPPA_AID3_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_AID2_MASK        (0x00001000u)
#define CSL_CGEM_L2MPPA_AID2_SHIFT       (0x0000000Cu)
#define CSL_CGEM_L2MPPA_AID2_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_AID1_MASK        (0x00000800u)
#define CSL_CGEM_L2MPPA_AID1_SHIFT       (0x0000000Bu)
#define CSL_CGEM_L2MPPA_AID1_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_AID0_MASK        (0x00000400u)
#define CSL_CGEM_L2MPPA_AID0_SHIFT       (0x0000000Au)
#define CSL_CGEM_L2MPPA_AID0_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_AIDX_MASK        (0x00000200u)
#define CSL_CGEM_L2MPPA_AIDX_SHIFT       (0x00000009u)
#define CSL_CGEM_L2MPPA_AIDX_RESETVAL    (0x00000000u)

#define CSL_CGEM_L2MPPA_LOCAL_MASK       (0x00000100u)
#define CSL_CGEM_L2MPPA_LOCAL_SHIFT      (0x00000008u)
#define CSL_CGEM_L2MPPA_LOCAL_RESETVAL   (0x00000000u)



#define CSL_CGEM_L2MPPA_SR_MASK          (0x00000020u)
#define CSL_CGEM_L2MPPA_SR_SHIFT         (0x00000005u)
#define CSL_CGEM_L2MPPA_SR_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2MPPA_SW_MASK          (0x00000010u)
#define CSL_CGEM_L2MPPA_SW_SHIFT         (0x00000004u)
#define CSL_CGEM_L2MPPA_SW_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2MPPA_SX_MASK          (0x00000008u)
#define CSL_CGEM_L2MPPA_SX_SHIFT         (0x00000003u)
#define CSL_CGEM_L2MPPA_SX_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2MPPA_UR_MASK          (0x00000004u)
#define CSL_CGEM_L2MPPA_UR_SHIFT         (0x00000002u)
#define CSL_CGEM_L2MPPA_UR_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2MPPA_UW_MASK          (0x00000002u)
#define CSL_CGEM_L2MPPA_UW_SHIFT         (0x00000001u)
#define CSL_CGEM_L2MPPA_UW_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2MPPA_UX_MASK          (0x00000001u)
#define CSL_CGEM_L2MPPA_UX_SHIFT         (0x00000000u)
#define CSL_CGEM_L2MPPA_UX_RESETVAL      (0x00000000u)

#define CSL_CGEM_L2MPPA_RESETVAL         (0x0000FFFFu)

/* L1PMPFAR */

#define CSL_CGEM_L1PMPFAR_ADDR_MASK      (0xFFFFFFFFu)
#define CSL_CGEM_L1PMPFAR_ADDR_SHIFT     (0x00000000u)
#define CSL_CGEM_L1PMPFAR_ADDR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L1PMPFAR_RESETVAL       (0x00000000u)

/* L1PMPFSR */


#define CSL_CGEM_L1PMPFSR_FID_MASK       (0x0000FE00u)
#define CSL_CGEM_L1PMPFSR_FID_SHIFT      (0x00000009u)
#define CSL_CGEM_L1PMPFSR_FID_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPFSR_LOCAL_MASK     (0x00000100u)
#define CSL_CGEM_L1PMPFSR_LOCAL_SHIFT    (0x00000008u)
#define CSL_CGEM_L1PMPFSR_LOCAL_RESETVAL (0x00000000u)

#define CSL_CGEM_L1PMPFSR_NS_MASK        (0x00000080u)
#define CSL_CGEM_L1PMPFSR_NS_SHIFT       (0x00000007u)
#define CSL_CGEM_L1PMPFSR_NS_RESETVAL    (0x00000000u)


#define CSL_CGEM_L1PMPFSR_SR_MASK        (0x00000020u)
#define CSL_CGEM_L1PMPFSR_SR_SHIFT       (0x00000005u)
#define CSL_CGEM_L1PMPFSR_SR_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PMPFSR_SW_MASK        (0x00000010u)
#define CSL_CGEM_L1PMPFSR_SW_SHIFT       (0x00000004u)
#define CSL_CGEM_L1PMPFSR_SW_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PMPFSR_SX_MASK        (0x00000008u)
#define CSL_CGEM_L1PMPFSR_SX_SHIFT       (0x00000003u)
#define CSL_CGEM_L1PMPFSR_SX_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PMPFSR_UR_MASK        (0x00000004u)
#define CSL_CGEM_L1PMPFSR_UR_SHIFT       (0x00000002u)
#define CSL_CGEM_L1PMPFSR_UR_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PMPFSR_UW_MASK        (0x00000002u)
#define CSL_CGEM_L1PMPFSR_UW_SHIFT       (0x00000001u)
#define CSL_CGEM_L1PMPFSR_UW_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PMPFSR_UX_MASK        (0x00000001u)
#define CSL_CGEM_L1PMPFSR_UX_SHIFT       (0x00000000u)
#define CSL_CGEM_L1PMPFSR_UX_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1PMPFSR_RESETVAL       (0x00000000u)

/* L1PMPFCR */


#define CSL_CGEM_L1PMPFCR_MPFCLR_MASK    (0x00000001u)
#define CSL_CGEM_L1PMPFCR_MPFCLR_SHIFT   (0x00000000u)
#define CSL_CGEM_L1PMPFCR_MPFCLR_RESETVAL (0x00000000u)

#define CSL_CGEM_L1PMPFCR_RESETVAL       (0x00000000u)

/* L1PMPPA */


#define CSL_CGEM_L1PMPPA_AID5_MASK       (0x00008000u)
#define CSL_CGEM_L1PMPPA_AID5_SHIFT      (0x0000000Fu)
#define CSL_CGEM_L1PMPPA_AID5_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_AID4_MASK       (0x00004000u)
#define CSL_CGEM_L1PMPPA_AID4_SHIFT      (0x0000000Eu)
#define CSL_CGEM_L1PMPPA_AID4_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_AID3_MASK       (0x00002000u)
#define CSL_CGEM_L1PMPPA_AID3_SHIFT      (0x0000000Du)
#define CSL_CGEM_L1PMPPA_AID3_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_AID2_MASK       (0x00001000u)
#define CSL_CGEM_L1PMPPA_AID2_SHIFT      (0x0000000Cu)
#define CSL_CGEM_L1PMPPA_AID2_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_AID1_MASK       (0x00000800u)
#define CSL_CGEM_L1PMPPA_AID1_SHIFT      (0x0000000Bu)
#define CSL_CGEM_L1PMPPA_AID1_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_AID0_MASK       (0x00000400u)
#define CSL_CGEM_L1PMPPA_AID0_SHIFT      (0x0000000Au)
#define CSL_CGEM_L1PMPPA_AID0_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_AIDX_MASK       (0x00000200u)
#define CSL_CGEM_L1PMPPA_AIDX_SHIFT      (0x00000009u)
#define CSL_CGEM_L1PMPPA_AIDX_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1PMPPA_LOCAL_MASK      (0x00000100u)
#define CSL_CGEM_L1PMPPA_LOCAL_SHIFT     (0x00000008u)
#define CSL_CGEM_L1PMPPA_LOCAL_RESETVAL  (0x00000000u)



#define CSL_CGEM_L1PMPPA_SR_MASK         (0x00000020u)
#define CSL_CGEM_L1PMPPA_SR_SHIFT        (0x00000005u)
#define CSL_CGEM_L1PMPPA_SR_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PMPPA_SW_MASK         (0x00000010u)
#define CSL_CGEM_L1PMPPA_SW_SHIFT        (0x00000004u)
#define CSL_CGEM_L1PMPPA_SW_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PMPPA_SX_MASK         (0x00000008u)
#define CSL_CGEM_L1PMPPA_SX_SHIFT        (0x00000003u)
#define CSL_CGEM_L1PMPPA_SX_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PMPPA_UR_MASK         (0x00000004u)
#define CSL_CGEM_L1PMPPA_UR_SHIFT        (0x00000002u)
#define CSL_CGEM_L1PMPPA_UR_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PMPPA_UW_MASK         (0x00000002u)
#define CSL_CGEM_L1PMPPA_UW_SHIFT        (0x00000001u)
#define CSL_CGEM_L1PMPPA_UW_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PMPPA_UX_MASK         (0x00000001u)
#define CSL_CGEM_L1PMPPA_UX_SHIFT        (0x00000000u)
#define CSL_CGEM_L1PMPPA_UX_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1PMPPA_RESETVAL        (0x0000FFFFu)

/* L1DMPFAR */

#define CSL_CGEM_L1DMPFAR_ADDR_MASK      (0xFFFFFFFFu)
#define CSL_CGEM_L1DMPFAR_ADDR_SHIFT     (0x00000000u)
#define CSL_CGEM_L1DMPFAR_ADDR_RESETVAL  (0x00000000u)

#define CSL_CGEM_L1DMPFAR_RESETVAL       (0x00000000u)

/* L1DMPFSR */


#define CSL_CGEM_L1DMPFSR_FID_MASK       (0x0000FE00u)
#define CSL_CGEM_L1DMPFSR_FID_SHIFT      (0x00000009u)
#define CSL_CGEM_L1DMPFSR_FID_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPFSR_LOCAL_MASK     (0x00000100u)
#define CSL_CGEM_L1DMPFSR_LOCAL_SHIFT    (0x00000008u)
#define CSL_CGEM_L1DMPFSR_LOCAL_RESETVAL (0x00000000u)

#define CSL_CGEM_L1DMPFSR_NS_MASK        (0x00000080u)
#define CSL_CGEM_L1DMPFSR_NS_SHIFT       (0x00000007u)
#define CSL_CGEM_L1DMPFSR_NS_RESETVAL    (0x00000000u)


#define CSL_CGEM_L1DMPFSR_SR_MASK        (0x00000020u)
#define CSL_CGEM_L1DMPFSR_SR_SHIFT       (0x00000005u)
#define CSL_CGEM_L1DMPFSR_SR_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1DMPFSR_SW_MASK        (0x00000010u)
#define CSL_CGEM_L1DMPFSR_SW_SHIFT       (0x00000004u)
#define CSL_CGEM_L1DMPFSR_SW_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1DMPFSR_SX_MASK        (0x00000008u)
#define CSL_CGEM_L1DMPFSR_SX_SHIFT       (0x00000003u)
#define CSL_CGEM_L1DMPFSR_SX_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1DMPFSR_UR_MASK        (0x00000004u)
#define CSL_CGEM_L1DMPFSR_UR_SHIFT       (0x00000002u)
#define CSL_CGEM_L1DMPFSR_UR_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1DMPFSR_UW_MASK        (0x00000002u)
#define CSL_CGEM_L1DMPFSR_UW_SHIFT       (0x00000001u)
#define CSL_CGEM_L1DMPFSR_UW_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1DMPFSR_UX_MASK        (0x00000001u)
#define CSL_CGEM_L1DMPFSR_UX_SHIFT       (0x00000000u)
#define CSL_CGEM_L1DMPFSR_UX_RESETVAL    (0x00000000u)

#define CSL_CGEM_L1DMPFSR_RESETVAL       (0x00000000u)

/* L1DMPFCR */


#define CSL_CGEM_L1DMPFCR_MPFCLR_MASK    (0x00000001u)
#define CSL_CGEM_L1DMPFCR_MPFCLR_SHIFT   (0x00000000u)
#define CSL_CGEM_L1DMPFCR_MPFCLR_RESETVAL (0x00000000u)

#define CSL_CGEM_L1DMPFCR_RESETVAL       (0x00000000u)

/* MPLK */

#define CSL_CGEM_MPLK_MPLK_MASK          (0xFFFFFFFFu)
#define CSL_CGEM_MPLK_MPLK_SHIFT         (0x00000000u)
#define CSL_CGEM_MPLK_MPLK_RESETVAL      (0x00000000u)

#define CSL_CGEM_MPLK_RESETVAL           (0x00000000u)

/* MPLKCMD */


#define CSL_CGEM_MPLKCMD_KEYR_MASK       (0x00000004u)
#define CSL_CGEM_MPLKCMD_KEYR_SHIFT      (0x00000002u)
#define CSL_CGEM_MPLKCMD_KEYR_RESETVAL   (0x00000000u)

#define CSL_CGEM_MPLKCMD_LOCK_MASK       (0x00000002u)
#define CSL_CGEM_MPLKCMD_LOCK_SHIFT      (0x00000001u)
#define CSL_CGEM_MPLKCMD_LOCK_RESETVAL   (0x00000000u)

#define CSL_CGEM_MPLKCMD_UNLOCK_MASK     (0x00000001u)
#define CSL_CGEM_MPLKCMD_UNLOCK_SHIFT    (0x00000000u)
#define CSL_CGEM_MPLKCMD_UNLOCK_RESETVAL (0x00000000u)

#define CSL_CGEM_MPLKCMD_RESETVAL        (0x00000000u)

/* MPLKSTAT */


#define CSL_CGEM_MPLKSTAT_NSL_MASK       (0x00000002u)
#define CSL_CGEM_MPLKSTAT_NSL_SHIFT      (0x00000001u)
#define CSL_CGEM_MPLKSTAT_NSL_RESETVAL   (0x00000000u)

#define CSL_CGEM_MPLKSTAT_LK_MASK        (0x00000001u)
#define CSL_CGEM_MPLKSTAT_LK_SHIFT       (0x00000000u)
#define CSL_CGEM_MPLKSTAT_LK_RESETVAL    (0x00000000u)

#define CSL_CGEM_MPLKSTAT_RESETVAL       (0x00000002u)

/* L1DMPPA */


#define CSL_CGEM_L1DMPPA_AID5_MASK       (0x00008000u)
#define CSL_CGEM_L1DMPPA_AID5_SHIFT      (0x0000000Fu)
#define CSL_CGEM_L1DMPPA_AID5_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_AID4_MASK       (0x00004000u)
#define CSL_CGEM_L1DMPPA_AID4_SHIFT      (0x0000000Eu)
#define CSL_CGEM_L1DMPPA_AID4_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_AID3_MASK       (0x00002000u)
#define CSL_CGEM_L1DMPPA_AID3_SHIFT      (0x0000000Du)
#define CSL_CGEM_L1DMPPA_AID3_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_AID2_MASK       (0x00001000u)
#define CSL_CGEM_L1DMPPA_AID2_SHIFT      (0x0000000Cu)
#define CSL_CGEM_L1DMPPA_AID2_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_AID1_MASK       (0x00000800u)
#define CSL_CGEM_L1DMPPA_AID1_SHIFT      (0x0000000Bu)
#define CSL_CGEM_L1DMPPA_AID1_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_AID0_MASK       (0x00000400u)
#define CSL_CGEM_L1DMPPA_AID0_SHIFT      (0x0000000Au)
#define CSL_CGEM_L1DMPPA_AID0_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_AIDX_MASK       (0x00000200u)
#define CSL_CGEM_L1DMPPA_AIDX_SHIFT      (0x00000009u)
#define CSL_CGEM_L1DMPPA_AIDX_RESETVAL   (0x00000000u)

#define CSL_CGEM_L1DMPPA_LOCAL_MASK      (0x00000100u)
#define CSL_CGEM_L1DMPPA_LOCAL_SHIFT     (0x00000008u)
#define CSL_CGEM_L1DMPPA_LOCAL_RESETVAL  (0x00000000u)



#define CSL_CGEM_L1DMPPA_SR_MASK         (0x00000020u)
#define CSL_CGEM_L1DMPPA_SR_SHIFT        (0x00000005u)
#define CSL_CGEM_L1DMPPA_SR_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DMPPA_SW_MASK         (0x00000010u)
#define CSL_CGEM_L1DMPPA_SW_SHIFT        (0x00000004u)
#define CSL_CGEM_L1DMPPA_SW_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DMPPA_SX_MASK         (0x00000008u)
#define CSL_CGEM_L1DMPPA_SX_SHIFT        (0x00000003u)
#define CSL_CGEM_L1DMPPA_SX_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DMPPA_UR_MASK         (0x00000004u)
#define CSL_CGEM_L1DMPPA_UR_SHIFT        (0x00000002u)
#define CSL_CGEM_L1DMPPA_UR_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DMPPA_UW_MASK         (0x00000002u)
#define CSL_CGEM_L1DMPPA_UW_SHIFT        (0x00000001u)
#define CSL_CGEM_L1DMPPA_UW_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DMPPA_UX_MASK         (0x00000001u)
#define CSL_CGEM_L1DMPPA_UX_SHIFT        (0x00000000u)
#define CSL_CGEM_L1DMPPA_UX_RESETVAL     (0x00000000u)

#define CSL_CGEM_L1DMPPA_RESETVAL        (0x0000FFFFu)

#endif
