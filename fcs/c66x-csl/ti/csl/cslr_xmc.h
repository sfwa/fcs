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
#ifndef _CSLR_XMC_H_
#define _CSLR_XMC_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) 	Modified the 'CSL_XmcRegs' register overlay structure. Merged
 * 			the XMPAXL0, XMPAXH0, XMPAXL1, XMPAXH1 into the XMPAX array
 * 			and made XMPAX[] array an array of 16 such register pairs. This
 * 			was done because all the XMPAXL/H register pairs 0-15 had the
 * 			exact same bit fields and masks and only diferred in the
 * 			reset values (which we do not use programatically). This
 * 			simplifies the code and use of the APIs lot more simpler and
 * 			more consistent.
 * 	    b) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for XMPAX
\**************************************************************************/
typedef struct  {
    volatile Uint32 XMPAXL;
    volatile Uint32 XMPAXH;
} CSL_XmcXmpaxRegs;


/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    CSL_XmcXmpaxRegs XMPAX[16];
    volatile Uint8 RSVD0[384];
    volatile Uint32 XMPFAR;
    volatile Uint32 XMPFSR;
    volatile Uint32 XMPFCR;
    volatile Uint8 RSVD1[116];
    volatile Uint32 MDMAARBX;
    volatile Uint8 RSVD2[124];
    volatile Uint32 XPFCMD;
    volatile Uint32 XPFACS;
    volatile Uint8 RSVD3[8];
    volatile Uint32 XPFAC0;
    volatile Uint32 XPFAC1;
    volatile Uint32 XPFAC2;
    volatile Uint32 XPFAC3;
    volatile Uint8 RSVD4[224];
    volatile Uint32 XPFADDR[8];
} CSL_XmcRegs;

#endif
