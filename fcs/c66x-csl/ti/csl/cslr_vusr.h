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
#ifndef _CSLR_VUSR_H_
#define _CSLR_VUSR_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 REV;
    volatile Uint32 CTL;
    volatile Uint32 STS;
    volatile Uint32 INT_PRI_VEC;
    volatile Uint32 INT_CLR;
    volatile Uint32 INT_SET;
    volatile Uint32 SW_INT;
    volatile Uint32 TX_SEL_CTL;
    volatile Uint8 RSVD0[12];
    volatile Uint32 RX_SEL_CTL;
    volatile Uint32 RX_PRIV_IDX;
    volatile Uint32 RX_PRIV_VAL;
    volatile Uint32 RX_SEG_IDX;
    volatile Uint32 RX_SEG_VAL;
    volatile Uint32 CHIP_ID_VER;
    volatile Uint32 PWR;
    volatile Uint8 RSVD1[4];
    volatile Uint32 ECC_CNTR;
    volatile Uint8 RSVD2[8];
    volatile Uint32 LINK_STS;
    volatile Uint8 RSVD3[4];
    volatile Uint32 INT_CTL_IDX;
    volatile Uint32 INT_CTL_VAL;
    volatile Uint32 INT_PTR_IDX;
    volatile Uint32 INT_PTR_VAL;
    volatile Uint32 SERDES_CTL_STS1;
    volatile Uint32 SERDES_CTL_STS2;
    volatile Uint32 SERDES_CTL_STS3;
    volatile Uint32 SERDES_CTL_STS4;
    volatile Uint32 REM_REGS[32];
} CSL_VusrRegs;

#endif
