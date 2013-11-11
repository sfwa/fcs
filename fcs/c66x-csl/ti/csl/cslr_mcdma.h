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
#ifndef _CSLR_MCDMA_H_
#define _CSLR_MCDMA_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for MCDMA_CHANNEL_CONFIG_GROUP
\**************************************************************************/
typedef struct  {
    volatile Uint32 SRC_ADDR_REG;
    volatile Uint32 DST_ADDR_REG;
    volatile Uint32 XFER_CTRL_REG;
    volatile Uint8 RSVD0[52];
} CSL_McdmaMcdma_channel_config_groupRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    CSL_McdmaMcdma_channel_config_groupRegs MCDMA_CHANNEL_CONFIG_GROUP[4];
} CSL_McdmaRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* SRC_ADDR_REG */

#define CSL_MCDMA_SRC_ADDR_REG_SRC_ADDRESS_MASK (0xFFFFFFFFu)
#define CSL_MCDMA_SRC_ADDR_REG_SRC_ADDRESS_SHIFT (0x00000000u)
#define CSL_MCDMA_SRC_ADDR_REG_SRC_ADDRESS_RESETVAL (0x00000000u)

#define CSL_MCDMA_SRC_ADDR_REG_RESETVAL  (0x00000000u)

/* DST_ADDR_REG */

#define CSL_MCDMA_DST_ADDR_REG_DST_ADDRESS_MASK (0xFFFFFFFFu)
#define CSL_MCDMA_DST_ADDR_REG_DST_ADDRESS_SHIFT (0x00000000u)
#define CSL_MCDMA_DST_ADDR_REG_DST_ADDRESS_RESETVAL (0x00000000u)

#define CSL_MCDMA_DST_ADDR_REG_RESETVAL  (0x00000000u)

/* XFER_CTRL_REG */

#define CSL_MCDMA_XFER_CTRL_REG_TRANSFER_LENGTH_MASK (0x0003FFFFu)
#define CSL_MCDMA_XFER_CTRL_REG_TRANSFER_LENGTH_SHIFT (0x00000000u)
#define CSL_MCDMA_XFER_CTRL_REG_TRANSFER_LENGTH_RESETVAL (0x00000000u)

#define CSL_MCDMA_XFER_CTRL_REG_BURST_MODE_MASK (0x00300000u)
#define CSL_MCDMA_XFER_CTRL_REG_BURST_MODE_SHIFT (0x00000014u)
#define CSL_MCDMA_XFER_CTRL_REG_BURST_MODE_RESETVAL (0x00000000u)

#define CSL_MCDMA_XFER_CTRL_REG_SRC_ADDR_MODE_MASK (0x00C00000u)
#define CSL_MCDMA_XFER_CTRL_REG_SRC_ADDR_MODE_SHIFT (0x00000016u)
#define CSL_MCDMA_XFER_CTRL_REG_SRC_ADDR_MODE_RESETVAL (0x00000000u)

#define CSL_MCDMA_XFER_CTRL_REG_DST_ADDR_MODE_MASK (0x03000000u)
#define CSL_MCDMA_XFER_CTRL_REG_DST_ADDR_MODE_SHIFT (0x00000018u)
#define CSL_MCDMA_XFER_CTRL_REG_DST_ADDR_MODE_RESETVAL (0x00000000u)

#define CSL_MCDMA_XFER_CTRL_REG_PRIORITY_MASK (0x1C000000u)
#define CSL_MCDMA_XFER_CTRL_REG_PRIORITY_SHIFT (0x0000001Au)
#define CSL_MCDMA_XFER_CTRL_REG_PRIORITY_RESETVAL (0x00000007u)

#define CSL_MCDMA_XFER_CTRL_REG_ACTIVE_MASK (0x40000000u)
#define CSL_MCDMA_XFER_CTRL_REG_ACTIVE_SHIFT (0x0000001Eu)
#define CSL_MCDMA_XFER_CTRL_REG_ACTIVE_RESETVAL (0x00000000u)

#define CSL_MCDMA_XFER_CTRL_REG_START_STOP_MASK (0x80000000u)
#define CSL_MCDMA_XFER_CTRL_REG_START_STOP_SHIFT (0x0000001Fu)
#define CSL_MCDMA_XFER_CTRL_REG_START_STOP_RESETVAL (0x00000000u)

#define CSL_MCDMA_XFER_CTRL_REG_RESETVAL (0x1C000000u)

#endif
