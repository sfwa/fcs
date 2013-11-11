/********************************************************************
* Copyright (C) 2002-2010 Texas Instruments Incorporated.
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
#ifndef _CSLR_PSC_H_
#define _CSLR_PSC_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 PID;
    volatile Uint8 RSVD0[16];
    volatile Uint32 VCNTLID;
    volatile Uint8 RSVD1[264];
    volatile Uint32 PTCMD;
    volatile Uint8 RSVD2[4];
    volatile Uint32 PTSTAT;
    volatile Uint8 RSVD3[212];
    volatile Uint32 PDSTAT[32];
    volatile Uint8 RSVD4[128];
    volatile Uint32 PDCTL[32];
    volatile Uint8 RSVD5[1152];
    volatile Uint32 MDSTAT[32];
    volatile Uint8 RSVD6[384];
    volatile Uint32 MDCTL[32];
} CSL_PscRegs;


/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_PscRegs             *CSL_PscRegsOvly;


/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* PID */

#define CSL_PSC_PID_PID_MASK (0xFFFFFFFFu)
#define CSL_PSC_PID_PID_SHIFT (0x00000000u)
#define CSL_PSC_PID_PID_RESETVAL (0x44823A00u)

#define CSL_PSC_PID_SCHEME_MASK (0xC0000000u)
#define CSL_PSC_PID_SCHEME_SHIFT (0x0000001Eu)
#define CSL_PSC_PID_SCHEME_RESETVAL (0x40000000u)

#define CSL_PSC_PID_FUNC_MASK (0x0FFF0000u)
#define CSL_PSC_PID_FUNC_SHIFT (0x00000010u)
#define CSL_PSC_PID_FUNC_RESETVAL (0x04820000u)

#define CSL_PSC_PID_RTL_MASK (0x0000F800u)
#define CSL_PSC_PID_RTL_SHIFT (0x0000000Bu)
#define CSL_PSC_PID_RTL_RESETVAL (0x00000000u)

#define CSL_PSC_PID_MAJOR_MASK (0x00000700u)
#define CSL_PSC_PID_MAJOR_SHIFT (0x00000008u)
#define CSL_PSC_PID_MAJOR_RESETVAL (0x00000200u)

#define CSL_PSC_PID_CUSTOM_MASK (0x000000C0u)
#define CSL_PSC_PID_CUSTOM_SHIFT (0x00000006u)
#define CSL_PSC_PID_CUSTOM_RESETVAL (0x00000000u)

#define CSL_PSC_PID_MINOR_MASK (0x0000003Fu)
#define CSL_PSC_PID_MINOR_SHIFT (0x00000000u)
#define CSL_PSC_PID_MINOR_RESETVAL (0x00000000u)

#define CSL_PSC_PID_RESETVAL (0x44823A00u)


/* VCNTLID */

#define CSL_PSC_VCNTLID_VCNTL_MASK (0x003F0000u)
#define CSL_PSC_VCNTLID_VCNTL_SHIFT (0x00000010u)
#define CSL_PSC_VCNTLID_VCNTL_RESETVAL (0x00000000u)

#define CSL_PSC_VCNTLID_RESETVAL (0x00000000u)

/* PTCMD */


#define CSL_PSC_PTCMD_GO_MASK (0xFFFFFFFFu)
#define CSL_PSC_PTCMD_GO_SHIFT (0x00000000u)
#define CSL_PSC_PTCMD_GO_RESETVAL (0x00000000u)
/*----GO Tokens----*/
#define CSL_PSC_PTCMD_GO_SET (0x00000001u)

#define CSL_PSC_PTCMD_RESETVAL (0x00000000u)

/* PTSTAT */


#define CSL_PSC_PTSTAT_GOSTAT_MASK (0xFFFFFFFFu)
#define CSL_PSC_PTSTAT_GOSTAT_SHIFT (0x00000000u)
#define CSL_PSC_PTSTAT_GOSTAT_RESETVAL (0x00000000u)
/*----GOSTAT Tokens----*/
#define CSL_PSC_PTSTAT_GOSTAT_NO_PWRDMN_TRANS (0x00000000u)
#define CSL_PSC_PTSTAT_GOSTAT_PWRDMN_TRANS_PROG (0x00000001u)

#define CSL_PSC_PTSTAT_RESETVAL (0x00000000u)

/* PDSTAT */


#define CSL_PSC_PDSTAT_STATE_MASK (0x00000003u)
#define CSL_PSC_PDSTAT_STATE_SHIFT (0x00000000u)
#define CSL_PSC_PDSTAT_STATE_RESETVAL (0x00000001u)
/*----STATE Tokens----*/
#define CSL_PSC_PDSTAT_STATE_OFF (0x00000000u)
#define CSL_PSC_PDSTAT_STATE_ON (0x00000001u)

#define CSL_PSC_PDSTAT_RESETVAL (0x00000301u)

/* PDCTL */


#define CSL_PSC_PDCTL_NEXT_MASK (0x00000001u)
#define CSL_PSC_PDCTL_NEXT_SHIFT (0x00000000u)
#define CSL_PSC_PDCTL_NEXT_RESETVAL (0x00000001u)
/*----NEXT Tokens----*/
#define CSL_PSC_PDCTL_NEXT_OFF (0x00000000u)
#define CSL_PSC_PDCTL_NEXT_ON (0x00000001u)

#define CSL_PSC_PDCTL_RESETVAL (0x0001B001u)

/* MDSTAT */


#define CSL_PSC_MDSTAT_MCKOUT_MASK (0x00001000u)
#define CSL_PSC_MDSTAT_MCKOUT_SHIFT (0x0000000Cu)
#define CSL_PSC_MDSTAT_MCKOUT_RESETVAL (0x00000000u)
/*----MCKOUT Tokens----*/
#define CSL_PSC_MDSTAT_MCKOUT_OFF (0x00000000u)
#define CSL_PSC_MDSTAT_MCKOUT_ON (0x00000001u)

#define CSL_PSC_MDSTAT_MRSTDONE_MASK (0x00000800u)
#define CSL_PSC_MDSTAT_MRSTDONE_SHIFT (0x0000000Bu)
#define CSL_PSC_MDSTAT_MRSTDONE_RESETVAL (0x00000001u)
/*----MRSTDONE Tokens----*/
#define CSL_PSC_MDSTAT_MRSTDONE_NO (0x00000000u)
#define CSL_PSC_MDSTAT_MRSTDONE_YES (0x00000001u)

#define CSL_PSC_MDSTAT_MRST_MASK (0x00000400u)
#define CSL_PSC_MDSTAT_MRST_SHIFT (0x0000000Au)
#define CSL_PSC_MDSTAT_MRST_RESETVAL (0x00000000u)
/*----MRST Tokens----*/
#define CSL_PSC_MDSTAT_MRST_ASSERT (0x00000000u)
#define CSL_PSC_MDSTAT_MRST_DE_ASSERT (0x00000001u)

#define CSL_PSC_MDSTAT_LRSTDONE_MASK (0x00000200u)
#define CSL_PSC_MDSTAT_LRSTDONE_SHIFT (0x00000009u)
#define CSL_PSC_MDSTAT_LRSTDONE_RESETVAL (0x00000001u)
/*----LRSTDONE Tokens----*/
#define CSL_PSC_MDSTAT_LRSTDONE_NO (0x00000000u)
#define CSL_PSC_MDSTAT_LRSTDONE_YES (0x00000001u)

#define CSL_PSC_MDSTAT_LRST_MASK (0x00000100u)
#define CSL_PSC_MDSTAT_LRST_SHIFT (0x00000008u)
#define CSL_PSC_MDSTAT_LRST_RESETVAL (0x00000000u)
/*----LRST Tokens----*/
#define CSL_PSC_MDSTAT_LRST_ASSERT (0x00000000u)
#define CSL_PSC_MDSTAT_LRST_DE_ASSERT (0x00000001u)

#define CSL_PSC_MDSTAT_STATE_MASK (0x0000003Fu)
#define CSL_PSC_MDSTAT_STATE_SHIFT (0x00000000u)
#define CSL_PSC_MDSTAT_STATE_RESETVAL (0x00000000u)
/*----STATE Tokens----*/
#define CSL_PSC_MDSTAT_STATE_SWRSTDISABLE (0x00000000u)
#define CSL_PSC_MDSTAT_STATE_SYNCRST (0x00000001u)
#define CSL_PSC_MDSTAT_STATE_DISABLE (0x00000002u)
#define CSL_PSC_MDSTAT_STATE_ENABLE (0x00000003u)
#define CSL_PSC_MDSTAT_STATE_AUTOSLP (0x00000004u)
#define CSL_PSC_MDSTAT_STATE_AUTOWK (0x00000005u)

#define CSL_PSC_MDSTAT_RESETVAL (0x00000A00u)

/* MDCTL */
#define CSL_PSC_MDCTL_RSTISO_MASK (0x00001000u)
#define CSL_PSC_MDCTL_RSTISO_SHIFT (0x0000000Cu)
#define CSL_PSC_MDCTL_RSTISO_RESETVAL (0x00000000u)
/*----LRST Tokens----*/
#define CSL_PSC_MDCTL_RSTISO_DISABLE (0x00000000u)
#define CSL_PSC_MDCTL_RSTISO_ENABLE (0x00000001u)

#define CSL_PSC_MDCTL_LRST_MASK (0x00000100u)
#define CSL_PSC_MDCTL_LRST_SHIFT (0x00000008u)
#define CSL_PSC_MDCTL_LRST_RESETVAL (0x00000001u)
/*----LRST Tokens----*/
#define CSL_PSC_MDCTL_LRST_ASSERT (0x00000000u)
#define CSL_PSC_MDCTL_LRST_DE_ASSERT (0x00000001u)


#define CSL_PSC_MDCTL_NEXT_MASK (0x0000001Fu)
#define CSL_PSC_MDCTL_NEXT_SHIFT (0x00000000u)
#define CSL_PSC_MDCTL_NEXT_RESETVAL (0x00000000u)
/*----NEXT Tokens----*/
#define CSL_PSC_MDCTL_NEXT_SWRSTDISABLE (0x00000000u)
#define CSL_PSC_MDCTL_NEXT_SYNCRST (0x00000001u)
#define CSL_PSC_MDCTL_NEXT_DISABLE (0x00000002u)
#define CSL_PSC_MDCTL_NEXT_ENABLE (0x00000003u)
#define CSL_PSC_MDCTL_NEXT_AUTOSLP (0x00000004u)
#define CSL_PSC_MDCTL_NEXT_AUTOWK (0x00000005u)

#define CSL_PSC_MDCTL_RESETVAL (0x00000100u)

#endif
