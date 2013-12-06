/********************************************************************
* Copyright (C) 2010 Texas Instruments Incorporated.
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
#ifndef _CSLR_PLLC_H_
#define _CSLR_PLLC_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint8 RSVD0[228];
    volatile Uint32 RSTYPE;
    volatile Uint32 RSTCTRL;
    volatile Uint32 RSTCFG;
    volatile Uint32 RSISO;
    volatile Uint8 RSVD1[12];
    volatile Uint32 PLLCTL;
    volatile Uint8 RSVD2[4];
    volatile Uint32 SECCTL;
    volatile Uint8 RSVD3[4];
    volatile Uint32 PLLM;
    volatile Uint32 PREDIV;
    volatile Uint32 PLLDIV1_3[3];
    volatile Uint8 RSVD4[4];
    volatile Uint32 POSTDIV;
    volatile Uint8 RSVD5[12];
    volatile Uint32 PLLCMD;
    volatile Uint32 PLLSTAT;
    volatile Uint32 ALNCTL;
    volatile Uint32 DCHANGE;
    volatile Uint32 CKEN;
    volatile Uint32 CKSTAT;
    volatile Uint32 SYSTAT;
    volatile Uint8 RSVD6[12];
    volatile Uint32 PLLDIV4_16[13];
} CSL_PllcRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_PllcRegs               *CSL_PllcRegsOvly;

#endif
