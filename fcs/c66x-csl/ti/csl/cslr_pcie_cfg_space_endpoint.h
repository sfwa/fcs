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
#ifndef _CSLR_PCIE_CFG_SPACE_ENDPOINT_H_
#define _CSLR_PCIE_CFG_SPACE_ENDPOINT_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>



/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 VENDOR_DEVICE_ID;
    volatile Uint32 STATUS_COMMAND;
    volatile Uint32 CLASSCODE_REVID;
    volatile Uint32 BIST_HEADER;
    volatile Uint32 BAR[6];
    volatile Uint32 CARDBUS;
    volatile Uint32 SUBSYS_VNDR_ID;
    volatile Uint32 EXPNSN_ROM;
    volatile Uint32 CAP_PTR;
    volatile Uint8 RSVD0[4];
    volatile Uint32 INT_PIN;
    volatile Uint32 PMCAP;
    volatile Uint32 PM_CTL_STAT;
    volatile Uint8 RSVD1[8];
    volatile Uint32 MSI_CAP;
    volatile Uint32 MSI_LOW32;
    volatile Uint32 MSI_UP32;
    volatile Uint32 MSI_DATA;
    volatile Uint8 RSVD2[16];
    volatile Uint32 PCIES_CAP;
    volatile Uint32 DEVICE_CAP;
    volatile Uint32 DEV_STAT_CTRL;
    volatile Uint32 LINK_CAP;
    volatile Uint32 LINK_STAT_CTRL;
    volatile Uint8 RSVD3[16];
    volatile Uint32 DEV_CAP2;
    volatile Uint32 DEV_STAT_CTRL2;
    volatile Uint8 RSVD4[4];
    volatile Uint32 LINK_CTRL2;
    volatile Uint8 RSVD5[92];
    volatile Uint32 PCIE_EXTCAP;
    volatile Uint32 PCIE_UNCERR;
    volatile Uint32 PCIE_UNCERR_MASK;
    volatile Uint32 PCIE_UNCERR_SVRTY;
    volatile Uint32 PCIE_CERR;
    volatile Uint32 PCIE_CERR_MASK;
    volatile Uint32 PCIE_ACCR;
    volatile Uint32 HDR_LOG[4];
    volatile Uint32 RC_ERR_CMD;
    volatile Uint32 RC_ERR_ST;
    volatile Uint32 ERR_SRC_ID;
    volatile Uint8 RSVD6[1480];
    volatile Uint32 PL_ACKTIMER;
    volatile Uint32 PL_OMSG;
    volatile Uint32 PL_FORCE_LINK;
    volatile Uint32 ACK_FREQ;
    volatile Uint32 PL_LINK_CTRL;
    volatile Uint32 LANE_SKEW;
    volatile Uint32 SYM_NUM;
    volatile Uint32 SYMTIMER_FLTMASK;
    volatile Uint32 FLT_MASK2;
    volatile Uint8 RSVD7[4];
    volatile Uint32 DEBUG0;
    volatile Uint32 DEBUG1;
    volatile Uint8 RSVD8[220];
    volatile Uint32 PL_GEN2;
} CSL_Pcie_cfg_space_endpointRegs;

#endif
