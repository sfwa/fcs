/*  ===========================================================================
 *  Copyright (c) Texas Instruments Incorporated 2011
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

/** ============================================================================
 *   @file  cslr_emif16.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  This file contains the Register Desciptions for EMIF16
 *
 */
#ifndef _CSLR_EMIF16_H_
#define _CSLR_EMIF16_H_

#include <ti/csl/cslr.h>

#include <ti/csl/tistdtypes.h>


/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 RCSR;
    volatile Uint32 AWCCR;
    volatile Uint8 RSVD0[8];
    volatile Uint32 A0CR;
    volatile Uint32 A1CR;
    volatile Uint32 A2CR;
    volatile Uint32 A3CR;
    volatile Uint8 RSVD1[32];
    volatile Uint32 IRR;
    volatile Uint32 IMR;
    volatile Uint32 IMSR;
    volatile Uint32 IMCR;
    volatile Uint32 IOCR;
    volatile Uint32 IOSR;
    volatile Uint8 RSVD2[8];
    volatile Uint32 NANDFCTL;
    volatile Uint32 NANDFSR;
    volatile Uint32 PMCR;
    volatile Uint8 RSVD3[4];
    volatile Uint32 NFECCCE0;
    volatile Uint32 NFECCCE1;
    volatile Uint32 NFECCCE2;
    volatile Uint32 NFECCCE3;
    volatile Uint8 RSVD4[4];
    volatile Uint32 IODFTEXECNT;
    volatile Uint32 IODFTGBLCTRL;
    volatile Uint8 RSVD5[4];
    volatile Uint32 IODFTTLAMISR;
    volatile Uint32 IODFTTLDMISR;
    volatile Uint32 IODFTTLDCMISR;
    volatile Uint8 RSVD6[20];
    volatile Uint32 MODRELNUM;
    volatile Uint8 RSVD7[8];
    volatile Uint32 NANDF4BECCLR;
    volatile Uint32 NANDF4BECC1R;
    volatile Uint32 NANDF4BECC2R;
    volatile Uint32 NANDF4BECC3R;
    volatile Uint32 NANDF4BECC4R;
    volatile Uint32 NANDFEA1R;
    volatile Uint32 NANDFEA2R;
    volatile Uint32 NANDFEV1R;
    volatile Uint32 NANDFEV2R;
} CSL_Emif16Regs;

#endif
