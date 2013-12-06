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
#ifndef _CSLR_PCIESS_APP_H_
#define _CSLR_PCIESS_APP_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>


/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for MSIX_IRQ
\**************************************************************************/
typedef struct  {
    volatile Uint32 MSI_IRQ_STATUS_RAW;
    volatile Uint32 MSI_IRQ_STATUS;
    volatile Uint32 MSI_IRQ_ENABLE_SET;
    volatile Uint32 MSI_IRQ_ENABLE_CLR;
} CSL_Pciess_appMsix_irqRegs;

/**************************************************************************\
* Register Overlay Structure for LEGACY_X_IRQ
\**************************************************************************/
typedef struct  {
    volatile Uint32 LEGACY_IRQ_STATUS_RAW;
    volatile Uint32 LEGACY_IRQ_STATUS;
    volatile Uint32 LEGACY_IRQ_ENABLE_SET;
    volatile Uint32 LEGACY_IRQ_ENABLE_CLR;
} CSL_Pciess_appLegacy_x_irqRegs;

/**************************************************************************\
* Register Overlay Structure for OUTBOUND_TRANSLATION
\**************************************************************************/
typedef struct  {
    volatile Uint32 OB_OFFSET_INDEX;
    volatile Uint32 OB_OFFSET_HI;
} CSL_Pciess_appOutbound_translationRegs;

/**************************************************************************\
* Register Overlay Structure for INBOUND_TRANSLATION
\**************************************************************************/
typedef struct  {
    volatile Uint32 IB_BAR;
    volatile Uint32 IB_START_LO;
    volatile Uint32 IB_START_HI;
    volatile Uint32 IB_OFFSET;
} CSL_Pciess_appInbound_translationRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 PID;
    volatile Uint32 CMD_STATUS;
    volatile Uint32 CFG_SETUP;
    volatile Uint32 IOBASE;
    volatile Uint32 TLPCFG;
    volatile Uint32 RSTCMD;
    volatile Uint8 RSVD0[8];
    volatile Uint32 PMCMD;
    volatile Uint32 PMCFG;
    volatile Uint32 ACT_STATUS;
    volatile Uint8 RSVD2[4];
    volatile Uint32 OB_SIZE;
    volatile Uint32 DIAG_CTRL;
    volatile Uint32 ENDIAN;
    volatile Uint32 PRIORITY;
    volatile Uint8 RSVD3[16];
    volatile Uint32 IRQ_EOI;
    volatile Uint32 MSI_IRQ;
    volatile Uint8 RSVD4[12];
    volatile Uint32 EP_IRQ_SET;
    volatile Uint32 EP_IRQ_CLR;
    volatile Uint32 EP_IRQ_STATUS;
    volatile Uint32 GPR[4];
    volatile Uint8 RSVD5[128];
    CSL_Pciess_appMsix_irqRegs MSIX_IRQ[8];
    CSL_Pciess_appLegacy_x_irqRegs LEGACY_X_IRQ[4];
    volatile Uint32 ERR_IRQ_STATUS_RAW;
    volatile Uint32 ERR_IRQ_STATUS;
    volatile Uint32 ERR_IRQ_ENABLE_SET;
    volatile Uint32 ERR_IRQ_ENABLE_CLR;
    volatile Uint32 PMRST_IRQ_STATUS_RAW;
    volatile Uint32 PMRST_IRQ_STATUS;
    volatile Uint32 PMRST_ENABLE_SET;
    volatile Uint32 PMRST_ENABLE_CLR;
    volatile Uint8 RSVD6[32];
    CSL_Pciess_appOutbound_translationRegs OUTBOUND_TRANSLATION[8];
    volatile Uint8 RSVD7[192];
    CSL_Pciess_appInbound_translationRegs INBOUND_TRANSLATION[4];
    volatile Uint8 RSVD8[64];
    volatile Uint32 PCS_CFG0;
    volatile Uint32 PCS_CFG1;
    volatile Uint32 PCS_STATUS;
    volatile Uint8 RSVD9[4];
    volatile Uint32 SERDES_CFG0;
    volatile Uint32 SERDES_CFG1;
} CSL_Pciess_appRegs;

#endif
