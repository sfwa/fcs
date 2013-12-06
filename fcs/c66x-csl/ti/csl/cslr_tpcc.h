/********************************************************************
* Copyright (C) 2003-2008 Texas Instruments Incorporated.
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
#ifndef _CSLR_TPCC_H_
#define _CSLR_TPCC_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified Registers TPCC_DCHMAP0-TPCC_DCHMAP63 to an array of TPCC_DCHMAP[64]
 *      b) Modified Registers TPCC_QCHMAP0-TPCC_QCHMAP7 to an array of TPCC_QCHMAP[8]
 *      c) Modified Registers TPCC_DMAQNUM0-TPCC_DMAQNUM7 to an array of TPCC_DMAQNUM[8]
 *      d) Replaced TPCC_DRAE0/TPCC_DRAEH0 to TPCC_DRAE7/TPCC_DRAEH7 with an array of
 *         this CSL_EDMA3_TPCC_DRA structure
 *      e) Modified Registers TPCC_QRAE0-TPCC_QRAE6 to an array of TPCC_QRAE[8]
 *      f) Modified Registers TPCC_Q0E0-TPCC_Q7E15 to an array of TPCC_QE[8]
 *      g) Modified Registers TPCC_MPPA0-TPCC_MPPA7 to an array of TPCC_MPPA[8]
 *      h) Added Register Overlay structure for the PaRAM-Set
 *      i) Added Register Overlay structure for SHADOW
 *      j) Appended the SHADOW and PARAM SET to the CC Register Overlay structure.
 *      k) Modified registers TPCC_QSTAT0-TPCC_QSTAT3 to an array TPCC_QSTAT[4]
 *      l) Modified header file includes as per the RTSC specification
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

typedef struct CSL_EDMA3_TPCC_DRA
{
    volatile Uint32 DRAE;
    volatile Uint32 DRAEH;
}CSL_EDMA3_TPCC_DRA;

/**************************************************************************\
* Register Overlay Structure for SHADOW
\**************************************************************************/
typedef struct  {
    volatile Uint32 TPCC_ER;
    volatile Uint32 TPCC_ERH;
    volatile Uint32 TPCC_ECR;
    volatile Uint32 TPCC_ECRH;
    volatile Uint32 TPCC_ESR;
    volatile Uint32 TPCC_ESRH;
    volatile Uint32 TPCC_CER;
    volatile Uint32 TPCC_CERH;
    volatile Uint32 TPCC_EER;
    volatile Uint32 TPCC_EERH;
    volatile Uint32 TPCC_EECR;
    volatile Uint32 TPCC_EECRH;
    volatile Uint32 TPCC_EESR;
    volatile Uint32 TPCC_EESRH;
    volatile Uint32 TPCC_SER;
    volatile Uint32 TPCC_SERH;
    volatile Uint32 TPCC_SECR;
    volatile Uint32 TPCC_SECRH;
    volatile Uint8 RSVD0[8];
    volatile Uint32 TPCC_IER;
    volatile Uint32 TPCC_IERH;
    volatile Uint32 TPCC_IECR;
    volatile Uint32 TPCC_IECRH;
    volatile Uint32 TPCC_IESR;
    volatile Uint32 TPCC_IESRH;
    volatile Uint32 TPCC_IPR;
    volatile Uint32 TPCC_IPRH;
    volatile Uint32 TPCC_ICR;
    volatile Uint32 TPCC_ICRH;
    volatile Uint32 TPCC_IEVAL;
    volatile Uint8 RSVD1[4];
    volatile Uint32 TPCC_QER;
    volatile Uint32 TPCC_QEER;
    volatile Uint32 TPCC_QEECR;
    volatile Uint32 TPCC_QEESR;
    volatile Uint32 TPCC_QSER;
    volatile Uint32 TPCC_QSECR;
    volatile Uint8 RSVD2[360];
} CSL_TPCC_ShadowRegs;

/**************************************************************************\
* Register Overlay Structure for PARAMSET
\**************************************************************************/
typedef struct  {
    volatile Uint32 OPT;
    volatile Uint32 SRC;
    volatile Uint32 A_B_CNT;
    volatile Uint32 DST;
    volatile Uint32 SRC_DST_BIDX;
    volatile Uint32 LINK_BCNTRLD;
    volatile Uint32 SRC_DST_CIDX;
    volatile Uint32 CCNT;
} CSL_TPCC_ParamsetRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 TPCC_PID;
    volatile Uint32 TPCC_CFG;
    volatile Uint8 RSVD0[244];
    volatile Uint32 TPCC_CLKGDIS;
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_DCHMAP0;
    volatile Uint32 TPCC_DCHMAP1;
    volatile Uint32 TPCC_DCHMAP2;
    volatile Uint32 TPCC_DCHMAP3;
    volatile Uint32 TPCC_DCHMAP4;
    volatile Uint32 TPCC_DCHMAP5;
    volatile Uint32 TPCC_DCHMAP6;
    volatile Uint32 TPCC_DCHMAP7;
    volatile Uint32 TPCC_DCHMAP8;
    volatile Uint32 TPCC_DCHMAP9;
    volatile Uint32 TPCC_DCHMAP10;
    volatile Uint32 TPCC_DCHMAP11;
    volatile Uint32 TPCC_DCHMAP12;
    volatile Uint32 TPCC_DCHMAP13;
    volatile Uint32 TPCC_DCHMAP14;
    volatile Uint32 TPCC_DCHMAP15;
    volatile Uint32 TPCC_DCHMAP16;
    volatile Uint32 TPCC_DCHMAP17;
    volatile Uint32 TPCC_DCHMAP18;
    volatile Uint32 TPCC_DCHMAP19;
    volatile Uint32 TPCC_DCHMAP20;
    volatile Uint32 TPCC_DCHMAP21;
    volatile Uint32 TPCC_DCHMAP22;
    volatile Uint32 TPCC_DCHMAP23;
    volatile Uint32 TPCC_DCHMAP24;
    volatile Uint32 TPCC_DCHMAP25;
    volatile Uint32 TPCC_DCHMAP26;
    volatile Uint32 TPCC_DCHMAP27;
    volatile Uint32 TPCC_DCHMAP28;
    volatile Uint32 TPCC_DCHMAP29;
    volatile Uint32 TPCC_DCHMAP30;
    volatile Uint32 TPCC_DCHMAP31;
    volatile Uint32 TPCC_DCHMAP32;
    volatile Uint32 TPCC_DCHMAP33;
    volatile Uint32 TPCC_DCHMAP34;
    volatile Uint32 TPCC_DCHMAP35;
    volatile Uint32 TPCC_DCHMAP36;
    volatile Uint32 TPCC_DCHMAP37;
    volatile Uint32 TPCC_DCHMAP38;
    volatile Uint32 TPCC_DCHMAP39;
    volatile Uint32 TPCC_DCHMAP40;
    volatile Uint32 TPCC_DCHMAP41;
    volatile Uint32 TPCC_DCHMAP42;
    volatile Uint32 TPCC_DCHMAP43;
    volatile Uint32 TPCC_DCHMAP44;
    volatile Uint32 TPCC_DCHMAP45;
    volatile Uint32 TPCC_DCHMAP46;
    volatile Uint32 TPCC_DCHMAP47;
    volatile Uint32 TPCC_DCHMAP48;
    volatile Uint32 TPCC_DCHMAP49;
    volatile Uint32 TPCC_DCHMAP50;
    volatile Uint32 TPCC_DCHMAP51;
    volatile Uint32 TPCC_DCHMAP52;
    volatile Uint32 TPCC_DCHMAP53;
    volatile Uint32 TPCC_DCHMAP54;
    volatile Uint32 TPCC_DCHMAP55;
    volatile Uint32 TPCC_DCHMAP56;
    volatile Uint32 TPCC_DCHMAP57;
    volatile Uint32 TPCC_DCHMAP58;
    volatile Uint32 TPCC_DCHMAP59;
    volatile Uint32 TPCC_DCHMAP60;
    volatile Uint32 TPCC_DCHMAP61;
    volatile Uint32 TPCC_DCHMAP62;
    volatile Uint32 TPCC_DCHMAP63;
#else
    volatile Uint32 TPCC_DCHMAP[64];
#endif
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_QCHMAP0;
    volatile Uint32 TPCC_QCHMAP1;
    volatile Uint32 TPCC_QCHMAP2;
    volatile Uint32 TPCC_QCHMAP3;
    volatile Uint32 TPCC_QCHMAP4;
    volatile Uint32 TPCC_QCHMAP5;
    volatile Uint32 TPCC_QCHMAP6;
    volatile Uint32 TPCC_QCHMAP7;
#else
    volatile Uint32 TPCC_QCHMAP[8];
#endif
    volatile Uint8 RSVD1[32];
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_DMAQNUM0;
    volatile Uint32 TPCC_DMAQNUM1;
    volatile Uint32 TPCC_DMAQNUM2;
    volatile Uint32 TPCC_DMAQNUM3;
    volatile Uint32 TPCC_DMAQNUM4;
    volatile Uint32 TPCC_DMAQNUM5;
    volatile Uint32 TPCC_DMAQNUM6;
    volatile Uint32 TPCC_DMAQNUM7;
#else
    volatile Uint32 TPCC_DMAQNUM[8];
#endif
    volatile Uint32 TPCC_QDMAQNUM;
    volatile Uint8 RSVD2[28];
    volatile Uint32 TPCC_QUETCMAP;
    volatile Uint32 TPCC_QUEPRI;
    volatile Uint8 RSVD3[120];
    volatile Uint32 TPCC_EMR;
    volatile Uint32 TPCC_EMRH;
    volatile Uint32 TPCC_EMCR;
    volatile Uint32 TPCC_EMCRH;
    volatile Uint32 TPCC_QEMR;
    volatile Uint32 TPCC_QEMCR;
    volatile Uint32 TPCC_CCERR;
    volatile Uint32 TPCC_CCERRCLR;
    volatile Uint32 TPCC_EEVAL;
    volatile Uint8 RSVD4[28];
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_DRAE0;
    volatile Uint32 TPCC_DRAEH0;
    volatile Uint32 TPCC_DRAE1;
    volatile Uint32 TPCC_DRAEH1;
    volatile Uint32 TPCC_DRAE2;
    volatile Uint32 TPCC_DRAEH2;
    volatile Uint32 TPCC_DRAE3;
    volatile Uint32 TPCC_DRAEH3;
    volatile Uint32 TPCC_DRAE4;
    volatile Uint32 TPCC_DRAEH4;
    volatile Uint32 TPCC_DRAE5;
    volatile Uint32 TPCC_DRAEH5;
    volatile Uint32 TPCC_DRAE6;
    volatile Uint32 TPCC_DRAEH6;
    volatile Uint32 TPCC_DRAE7;
    volatile Uint32 TPCC_DRAEH7;
#else
    CSL_EDMA3_TPCC_DRA  TPCC_DRA[8];
#endif
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_QRAE0;
    volatile Uint32 TPCC_QRAE1;
    volatile Uint32 TPCC_QRAE2;
    volatile Uint32 TPCC_QRAE3;
    volatile Uint32 TPCC_QRAE4;
    volatile Uint32 TPCC_QRAE5;
    volatile Uint32 TPCC_QRAE6;
    volatile Uint32 TPCC_QRAE7;
#else
    volatile Uint32 TPCC_QRAE[8];
#endif
    volatile Uint8 RSVD5[96];
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_Q0E0;
    volatile Uint32 TPCC_Q0E1;
    volatile Uint32 TPCC_Q0E2;
    volatile Uint32 TPCC_Q0E3;
    volatile Uint32 TPCC_Q0E4;
    volatile Uint32 TPCC_Q0E5;
    volatile Uint32 TPCC_Q0E6;
    volatile Uint32 TPCC_Q0E7;
    volatile Uint32 TPCC_Q0E8;
    volatile Uint32 TPCC_Q0E9;
    volatile Uint32 TPCC_Q0E10;
    volatile Uint32 TPCC_Q0E11;
    volatile Uint32 TPCC_Q0E12;
    volatile Uint32 TPCC_Q0E13;
    volatile Uint32 TPCC_Q0E14;
    volatile Uint32 TPCC_Q0E15;
    volatile Uint32 TPCC_Q1E0;
    volatile Uint32 TPCC_Q1E1;
    volatile Uint32 TPCC_Q1E2;
    volatile Uint32 TPCC_Q1E3;
    volatile Uint32 TPCC_Q1E4;
    volatile Uint32 TPCC_Q1E5;
    volatile Uint32 TPCC_Q1E6;
    volatile Uint32 TPCC_Q1E7;
    volatile Uint32 TPCC_Q1E8;
    volatile Uint32 TPCC_Q1E9;
    volatile Uint32 TPCC_Q1E10;
    volatile Uint32 TPCC_Q1E11;
    volatile Uint32 TPCC_Q1E12;
    volatile Uint32 TPCC_Q1E13;
    volatile Uint32 TPCC_Q1E14;
    volatile Uint32 TPCC_Q1E15;
    volatile Uint32 TPCC_Q2E0;
    volatile Uint32 TPCC_Q2E1;
    volatile Uint32 TPCC_Q2E2;
    volatile Uint32 TPCC_Q2E3;
    volatile Uint32 TPCC_Q2E4;
    volatile Uint32 TPCC_Q2E5;
    volatile Uint32 TPCC_Q2E6;
    volatile Uint32 TPCC_Q2E7;
    volatile Uint32 TPCC_Q2E8;
    volatile Uint32 TPCC_Q2E9;
    volatile Uint32 TPCC_Q2E10;
    volatile Uint32 TPCC_Q2E11;
    volatile Uint32 TPCC_Q2E12;
    volatile Uint32 TPCC_Q2E13;
    volatile Uint32 TPCC_Q2E14;
    volatile Uint32 TPCC_Q2E15;
    volatile Uint32 TPCC_Q3E0;
    volatile Uint32 TPCC_Q3E1;
    volatile Uint32 TPCC_Q3E2;
    volatile Uint32 TPCC_Q3E3;
    volatile Uint32 TPCC_Q3E4;
    volatile Uint32 TPCC_Q3E5;
    volatile Uint32 TPCC_Q3E6;
    volatile Uint32 TPCC_Q3E7;
    volatile Uint32 TPCC_Q3E8;
    volatile Uint32 TPCC_Q3E9;
    volatile Uint32 TPCC_Q3E10;
    volatile Uint32 TPCC_Q3E11;
    volatile Uint32 TPCC_Q3E12;
    volatile Uint32 TPCC_Q3E13;
    volatile Uint32 TPCC_Q3E14;
    volatile Uint32 TPCC_Q3E15;
    volatile Uint32 TPCC_Q4E0;
    volatile Uint32 TPCC_Q4E1;
    volatile Uint32 TPCC_Q4E2;
    volatile Uint32 TPCC_Q4E3;
    volatile Uint32 TPCC_Q4E4;
    volatile Uint32 TPCC_Q4E5;
    volatile Uint32 TPCC_Q4E6;
    volatile Uint32 TPCC_Q4E7;
    volatile Uint32 TPCC_Q4E8;
    volatile Uint32 TPCC_Q4E9;
    volatile Uint32 TPCC_Q4E10;
    volatile Uint32 TPCC_Q4E11;
    volatile Uint32 TPCC_Q4E12;
    volatile Uint32 TPCC_Q4E13;
    volatile Uint32 TPCC_Q4E14;
    volatile Uint32 TPCC_Q4E15;
    volatile Uint32 TPCC_Q5E0;
    volatile Uint32 TPCC_Q5E1;
    volatile Uint32 TPCC_Q5E2;
    volatile Uint32 TPCC_Q5E3;
    volatile Uint32 TPCC_Q5E4;
    volatile Uint32 TPCC_Q5E5;
    volatile Uint32 TPCC_Q5E6;
    volatile Uint32 TPCC_Q5E7;
    volatile Uint32 TPCC_Q5E8;
    volatile Uint32 TPCC_Q5E9;
    volatile Uint32 TPCC_Q5E10;
    volatile Uint32 TPCC_Q5E11;
    volatile Uint32 TPCC_Q5E12;
    volatile Uint32 TPCC_Q5E13;
    volatile Uint32 TPCC_Q5E14;
    volatile Uint32 TPCC_Q5E15;
    volatile Uint32 TPCC_Q6E0;
    volatile Uint32 TPCC_Q6E1;
    volatile Uint32 TPCC_Q6E2;
    volatile Uint32 TPCC_Q6E3;
    volatile Uint32 TPCC_Q6E4;
    volatile Uint32 TPCC_Q6E5;
    volatile Uint32 TPCC_Q6E6;
    volatile Uint32 TPCC_Q6E7;
    volatile Uint32 TPCC_Q6E8;
    volatile Uint32 TPCC_Q6E9;
    volatile Uint32 TPCC_Q6E10;
    volatile Uint32 TPCC_Q6E11;
    volatile Uint32 TPCC_Q6E12;
    volatile Uint32 TPCC_Q6E13;
    volatile Uint32 TPCC_Q6E14;
    volatile Uint32 TPCC_Q6E15;
    volatile Uint32 TPCC_Q7E0;
    volatile Uint32 TPCC_Q7E1;
    volatile Uint32 TPCC_Q7E2;
    volatile Uint32 TPCC_Q7E3;
    volatile Uint32 TPCC_Q7E4;
    volatile Uint32 TPCC_Q7E5;
    volatile Uint32 TPCC_Q7E6;
    volatile Uint32 TPCC_Q7E7;
    volatile Uint32 TPCC_Q7E8;
    volatile Uint32 TPCC_Q7E9;
    volatile Uint32 TPCC_Q7E10;
    volatile Uint32 TPCC_Q7E11;
    volatile Uint32 TPCC_Q7E12;
    volatile Uint32 TPCC_Q7E13;
    volatile Uint32 TPCC_Q7E14;
    volatile Uint32 TPCC_Q7E15;
#else
    volatile Uint32 TPCC_QE[128];
#endif
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_QSTAT0;
    volatile Uint32 TPCC_QSTAT1;
    volatile Uint32 TPCC_QSTAT2;
    volatile Uint32 TPCC_QSTAT3;
    volatile Uint32 TPCC_QSTAT4;
    volatile Uint32 TPCC_QSTAT5;
    volatile Uint32 TPCC_QSTAT6;
    volatile Uint32 TPCC_QSTAT7;
#else
    volatile Uint32 TPCC_QSTAT[8];
#endif
    volatile Uint32 TPCC_QWMTHRA;
    volatile Uint32 TPCC_QWMTHRB;
    volatile Uint8 RSVD6[24];
    volatile Uint32 TPCC_CCSTAT;
    volatile Uint8 RSVD7[188];
    volatile Uint32 TPCC_AETCTL;
    volatile Uint32 TPCC_AETSTAT;
    volatile Uint32 TPCC_AETCMD;
    volatile Uint8 RSVD8[244];
    volatile Uint32 TPCC_MPFAR;
    volatile Uint32 TPCC_MPFSR;
    volatile Uint32 TPCC_MPFCR;
    volatile Uint32 TPCC_MPPAG;
#ifdef CSL_MODIFICATION
    volatile Uint32 TPCC_MPPA0;
    volatile Uint32 TPCC_MPPA1;
    volatile Uint32 TPCC_MPPA2;
    volatile Uint32 TPCC_MPPA3;
    volatile Uint32 TPCC_MPPA4;
    volatile Uint32 TPCC_MPPA5;
    volatile Uint32 TPCC_MPPA6;
    volatile Uint32 TPCC_MPPA7;
#else
    volatile Uint32 TPCC_MPPA[8];
#endif
    volatile Uint8 RSVD9[2000];
    volatile Uint32 TPCC_ER;
    volatile Uint32 TPCC_ERH;
    volatile Uint32 TPCC_ECR;
    volatile Uint32 TPCC_ECRH;
    volatile Uint32 TPCC_ESR;
    volatile Uint32 TPCC_ESRH;
    volatile Uint32 TPCC_CER;
    volatile Uint32 TPCC_CERH;
    volatile Uint32 TPCC_EER;
    volatile Uint32 TPCC_EERH;
    volatile Uint32 TPCC_EECR;
    volatile Uint32 TPCC_EECRH;
    volatile Uint32 TPCC_EESR;
    volatile Uint32 TPCC_EESRH;
    volatile Uint32 TPCC_SER;
    volatile Uint32 TPCC_SERH;
    volatile Uint32 TPCC_SECR;
    volatile Uint32 TPCC_SECRH;
    volatile Uint8 RSVD10[8];
    volatile Uint32 TPCC_IER;
    volatile Uint32 TPCC_IERH;
    volatile Uint32 TPCC_IECR;
    volatile Uint32 TPCC_IECRH;
    volatile Uint32 TPCC_IESR;
    volatile Uint32 TPCC_IESRH;
    volatile Uint32 TPCC_IPR;
    volatile Uint32 TPCC_IPRH;
    volatile Uint32 TPCC_ICR;
    volatile Uint32 TPCC_ICRH;
    volatile Uint32 TPCC_IEVAL;
    volatile Uint8 RSVD11[4];
    volatile Uint32 TPCC_QER;
    volatile Uint32 TPCC_QEER;
    volatile Uint32 TPCC_QEECR;
    volatile Uint32 TPCC_QEESR;
    volatile Uint32 TPCC_QSER;
    volatile Uint32 TPCC_QSECR;
#ifndef CSL_MODIFICATION
    volatile Uint8 RSVD14[3944];
    CSL_TPCC_ShadowRegs SHADOW[8];
    volatile Uint8 RSVD15[4096];
    CSL_TPCC_ParamsetRegs PARAMSET[512];
#endif
} CSL_TpccRegs;

#endif


