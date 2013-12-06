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
#ifndef _CSLR_MPU_H_
#define _CSLR_MPU_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified header file includes as per the RTSC specification
 *      b) Removed Interrupt vector.
 */
#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for prog_region
\**************************************************************************/
typedef struct  {
    volatile Uint32 PROG_START_ADDRESS;
    volatile Uint32 PROG_END_ADDRESS;
    volatile Uint32 PROG_MPPA;
    volatile Uint8 RSVD2[4];
} CSL_MpuProg_regionRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 REVISION;
    volatile Uint32 CONFIG;
    volatile Uint8 RSVD0[8];
    volatile Uint32 INT_RAW_STATUS_SET;
    volatile Uint32 INT_ENABLED_STATUS_CLEAR;
    volatile Uint32 INT_ENABLE;
    volatile Uint32 INT_ENABLE_CLEAR;
    volatile Uint32 EOI;
    volatile Uint8  RSVD[4];
    volatile Uint8 RSVD1[216];
    volatile Uint32 FIXED_START_ADDRESS;
    volatile Uint32 FIXED_END_ADDRESS;
    volatile Uint32 FIXED_MPPA;
    volatile Uint8 RSVD3[244];
    CSL_MpuProg_regionRegs PROG_REGION[16];
    volatile Uint32 FAULT_ADDRESS;
    volatile Uint32 FAULT_STATUS;
    volatile Uint32 FAULT_CLEAR;
} CSL_MpuRegs;

#endif
