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
#ifndef _CSLR_MSMC_H_
#define _CSLR_MSMC_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Removed the redundant 'CSL_MsmcSbndRegs' structure for SBND
 *         since this was a data structure with a single field.
 *      b) Modified the 'CSL_MsmcSms_mpax_prividRegs' to the the new
 *         counterpart 'CSL_MsmcSms_mpax_per_prividRegs' which in turn
 *         now includes an array of 8 MPAXH and MPAXL registers.
 *      c) Modified the 'CSL_MsmcSes_mpax_prividRegs' to the the new
 *         counterpart 'CSL_MsmcSes_mpax_per_prividRegs' which in turn
 *         now includes an array of 8 MPAXH and MPAXL registers.
 *      d) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for SBND
\**************************************************************************/

typedef struct
{
    volatile Uint32 MPAXL;
    volatile Uint32 MPAXH;
}CSL_Msmc_Control_Regs;

/**************************************************************************\
* Register Overlay Structure for SMS_MPAX_per_PrivID
\**************************************************************************/
typedef struct
{
    CSL_Msmc_Control_Regs SMS[8];
} CSL_MsmcSms_mpax_per_prividRegs;

/**************************************************************************\
* Register Overlay Structure for SES_MPAX_per_PrivID
\**************************************************************************/
typedef struct
{
      CSL_Msmc_Control_Regs SES[8];
} CSL_MsmcSes_mpax_per_prividRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 PID;
    volatile Uint32 SMCC;
    volatile Uint32 SMCERRAR;
    volatile Uint32 SMCERRXR;
    volatile Uint32 SMEDCC;
    volatile Uint32 SMCEA;
    volatile Uint32 SMSECC;
    volatile Uint32 SMPFAR;
    volatile Uint32 SMPFXR;
    volatile Uint32 SMPFR;
    volatile Uint32 SMPFCR;
    volatile Uint32 SMSTA;
    volatile Uint32 SBND[8];
    volatile Uint32 SBNDM;
    volatile Uint32 SBNDE;
    volatile Uint32 SMEDCTST;
    volatile Uint32 CFGLCK;
    volatile Uint32 CFGULCK;
    volatile Uint32 CFGLCKSTAT;
    volatile Uint32 SMS_MPAX_LCK;
    volatile Uint32 SMS_MPAX_ULCK;
    volatile Uint32 SMS_MPAX_LCKSTAT;
    volatile Uint32 SES_MPAX_LCK;
    volatile Uint32 SES_MPAX_ULCK;
    volatile Uint32 SES_MPAX_LCKSTAT;
    volatile Uint32 SMESTAT;
    volatile Uint32 SMIRSTAT;
    volatile Uint32 SMIRC;
    volatile Uint32 SMIESTAT;
    volatile Uint32 SMIEC;
    volatile Uint8 RSVD0[12];
    volatile Uint32 SMATH;
    volatile Uint32 SMAH;
    volatile Uint32 SMARM;
    volatile Uint32 SMAWM;
    volatile Uint32 SMAV;
    volatile Uint32 SMAMP;
    volatile Uint32 SMAEP;
    volatile Uint32 SMAC;
    volatile Uint32 SMCFG;
    volatile Uint32 SMNCERRAR;
    volatile Uint32 SMNCERRXR;
    volatile Uint32 SMNCEA;
    volatile Uint8 RSVD1[304];
    CSL_MsmcSms_mpax_per_prividRegs SMS_MPAX_PER_PRIVID[16];
    CSL_MsmcSes_mpax_per_prividRegs SES_MPAX_PER_PRIVID[16];
} CSL_MsmcRegs;

#endif

