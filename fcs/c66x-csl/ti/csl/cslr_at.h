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
#ifndef _CSLR_AT_H_
#define _CSLR_AT_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for PI_Data
\**************************************************************************/
typedef struct  {
    volatile Uint32 AT_PIMAX_LK;
    volatile Uint32 AT_PIMMIN_LK;
    volatile Uint32 AT_PIVALUE_LK;
} CSL_AtPi_dataRegs;

/**************************************************************************\
* Register Overlay Structure for at_events
\**************************************************************************/
typedef struct  {
    volatile Uint32 AT_EVENT_OFFSET;
    volatile Uint32 AT_EVENT_MOD_TC;
    volatile Uint32 AT_EVENT_MASK_LSBS;
    volatile Uint32 AT_EVENT_MASK_MSBS;
} CSL_AtAt_eventsRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 AT_CONTROL1;
    volatile Uint32 AT_CONTROL2;
    volatile Uint32 AT_SW_SYNC;
    volatile Uint8 RSVD0[4];
    volatile Uint32 AT_RP1_TYPE;
    volatile Uint8 RSVD1[12];
    volatile Uint32 AT_RP1_TYPE_CAPTURE;
    volatile Uint32 AT_RP1_TOD_CAPTURE_L;
    volatile Uint32 AT_RP1_TOD_CAPTURE_H;
    volatile Uint32 AT_RP1_RP3_CAPTURE_L;
    volatile Uint32 AT_RP1_RP3_CAPTURE_H;
    volatile Uint32 AT_RP1_RAD_CAPTURE_L;
    volatile Uint32 AT_RP1_RAD_CAPTURE_H;
    volatile Uint8 RSVD2[4];
    volatile Uint32 AT_PHYT_CLKCNT_VALUE;
    volatile Uint32 AT_PHYT_FRM_VALUE_LSBS;
    volatile Uint32 AT_PHYT_FRM_VALUE_MSBS;
    volatile Uint32 AT_RADT_VALUE_LSBS;
    volatile Uint32 AT_RADT_VALUE_MID;
    volatile Uint32 AT_RADT_VALUE_MSBS;
    volatile Uint32 AT_ULRADT_VALUE_LSBS;
    volatile Uint32 AT_ULRADT_VALUE_MID;
    volatile Uint32 AT_ULRADT_VALUE_MSBS;
    volatile Uint32 AT_DLRADT_VALUE_LSBS;
    volatile Uint32 AT_DLRADT_VALUE_MID;
    volatile Uint32 AT_DLRADT_VALUE_MSBS;
    volatile Uint32 AT_RADT_WCDMA_VALUE;
    volatile Uint32 AT_ULRADT_WCDMA_VALUE;
    volatile Uint32 AT_DLRADT_WCDMA_VALUE;
    volatile Uint8 RSVD3[4];
    volatile Uint32 AT_PHYT_INIT_LSBS;
    volatile Uint32 AT_PHYT_INIT_MID;
    volatile Uint32 AT_PHYT_INIT_MSBS;
    volatile Uint32 AT_PHYT_TC_LSBS;
    volatile Uint32 AT_PHYT_FRAME_TC_LSBS;
    volatile Uint32 AT_PHYT_FRAME_TC_MSBS;
    volatile Uint32 AT_RADT_INIT_LSBS;
    volatile Uint32 AT_RADT_INIT_MID;
    volatile Uint32 AT_RADT_INIT_MSBS;
    volatile Uint32 AT_ULRADT_INIT_LSBS;
    volatile Uint32 AT_RADT_TSTAMP_VALUE;
    volatile Uint8 RSVD4[4];
    volatile Uint32 AT_DLRADT_INIT_LSBS;
    volatile Uint32 AT_GSM_TCOUNT;
    volatile Uint8 RSVD5[4];
    volatile Uint32 AT_RADT_SYMB_LUT_INDEX_TC;
    volatile Uint32 AT_ULRADT_SYMB_LUT_INDEX_TC;
    volatile Uint32 AT_DLRADT_SYMB_LUT_INDEX_TC;
    volatile Uint32 AT_RADT_FRAME_TC_MSBS;
    volatile Uint32 AT_RADT_FRAME_TC_LSBS;
    volatile Uint32 AT_ULRADT_FRAME_TC_MSBS;
    volatile Uint32 AT_ULRADT_FRAME_TC_LSBS;
    volatile Uint32 AT_DLRADT_FRAME_TC_MSBS;
    volatile Uint32 AT_DLRADT_FRAME_TC_LSBS;
    volatile Uint8 RSVD6[32];
    volatile Uint32 AT_RADT_SYM_LUT_RAM[128];
    volatile Uint32 AT_ULRADT_SYM_LUT_RAM[128];
    volatile Uint32 AT_DLRADT_SYM_LUT_RAM[128];
    volatile Uint8 RSVD7[256];
    CSL_AtPi_dataRegs PI_DATA[6];
    volatile Uint32 AT_EVT_ENABLE;
    volatile Uint32 AT_EVT_FORCE;
    CSL_AtAt_eventsRegs AT_EVENTS[8];
} CSL_AtRegs;

#endif
