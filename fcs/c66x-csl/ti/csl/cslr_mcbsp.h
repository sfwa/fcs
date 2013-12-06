/*
 * cslr_mcbsp.h
 *
 * This file contains the macros for Register Chip Support Library (CSL) which
 * can be used for operations on the respective underlying hardware/peripheral
 *
 * Copyright (C) 2009-2012 Texas Instruments Incorporated - http://www.ti.com/
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


/*********************************************************************
* file: cslr_mcbsp.h
*
* Brief: This file contains the Register Description for mcbsp
*
*********************************************************************/
#ifndef _CSLR_MCBSP_H_
#define _CSLR_MCBSP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a)  Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 DRR;
    volatile Uint32 DXR;
    volatile Uint32 SPCR;
    volatile Uint32 RCR;
    volatile Uint32 XCR;
    volatile Uint32 SRGR;
    volatile Uint32 MCR;
    volatile Uint32 RCERE0;
    volatile Uint32 XCERE0;
    volatile Uint32 PCR;
    volatile Uint32 RCERE1;
    volatile Uint32 XCERE1;
    volatile Uint32 RCERE2;
    volatile Uint32 XCERE2;
    volatile Uint32 RCERE3;
    volatile Uint32 XCERE3;
} CSL_McbspRegs;


/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 REVID;
    volatile Uint8 RSVD0[12];
    volatile Uint32 WFIFOCTL;
    volatile Uint32 WFIFOSTS;
    volatile Uint32 RFIFOCTL;
    volatile Uint32 RFIFOSTS;
} CSL_BfifoRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 DATA_PORT;
    volatile Uint32 RSVD_04;
    volatile Uint32 RSVD_08;
    volatile Uint32 RSVD_0C;
} CSL_BdataRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_McbspRegs          *CSL_McbspRegsOvly;
typedef volatile CSL_BfifoRegs          *CSL_BfifoRegsOvly;
typedef volatile CSL_BdataRegs          *CSL_BdataRegsOvly;


#ifdef __cplusplus
}
#endif

#endif

