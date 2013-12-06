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

#endif
