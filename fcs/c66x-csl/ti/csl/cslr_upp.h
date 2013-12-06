/*
 * cslr_upp.h
 *
 * This file contains the macros for Register Chip Support Library (CSL) which
 * can be used for operations on the respective underlying hardware/peripheral
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#ifndef _CSLR_UPP_H_
#define _CSLR_UPP_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a)  Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 UPPID;
    volatile Uint32 UPPCR;
    volatile Uint32 UPDLB;
    volatile Uint8 RSVD0[4];
    volatile Uint32 UPCTL;
    volatile Uint32 UPICR;
    volatile Uint32 UPIVR;
    volatile Uint32 UPTCR;
    volatile Uint32 UPISR;
    volatile Uint32 UPIER;
    volatile Uint32 UPIES;
    volatile Uint32 UPIEC;
    volatile Uint32 UPEOI;
    volatile Uint8 RSVD1[12];
    volatile Uint32 UPID0;
    volatile Uint32 UPID1;
    volatile Uint32 UPID2;
    volatile Uint8 RSVD2[4];
    volatile Uint32 UPIS0;
    volatile Uint32 UPIS1;
    volatile Uint32 UPIS2;
    volatile Uint8 RSVD3[4];
    volatile Uint32 UPQD0;
    volatile Uint32 UPQD1;
    volatile Uint32 UPQD2;
    volatile Uint8 RSVD4[4];
    volatile Uint32 UPQS0;
    volatile Uint32 UPQS1;
    volatile Uint32 UPQS2;
} CSL_UppRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_UppRegs         *CSL_UppRegsOvly;

#endif
