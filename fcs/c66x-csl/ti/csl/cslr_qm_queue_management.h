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
#ifndef _CSLR_QM_QUEUE_MANAGEMENT_H_
#define _CSLR_QM_QUEUE_MANAGEMENT_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for QUEUE_MGMT_GROUP
\**************************************************************************/
typedef struct  {
    volatile Uint32 QUEUE_REG_A;
    volatile Uint32 QUEUE_REG_B;
    volatile Uint32 QUEUE_REG_C;
    volatile Uint32 QUEUE_REG_D;
} CSL_Qm_queue_managementQueue_mgmt_groupRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    CSL_Qm_queue_managementQueue_mgmt_groupRegs QUEUE_MGMT_GROUP[8192];
} CSL_Qm_queue_managementRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* QUEUE_REG_A */

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_A_QUEUE_ENTRY_COUNT_MASK (0x0007FFFFu)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_A_QUEUE_ENTRY_COUNT_SHIFT (0x00000000u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_A_QUEUE_ENTRY_COUNT_RESETVAL (0x00000000u)

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_A_RESETVAL (0x00000000u)

/* QUEUE_REG_B */

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_B_QUEUE_BYTE_COUNT_MASK (0xFFFFFFFFu)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_B_QUEUE_BYTE_COUNT_SHIFT (0x00000000u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_B_QUEUE_BYTE_COUNT_RESETVAL (0x00000000u)

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_B_RESETVAL (0x00000000u)

/* QUEUE_REG_C */

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_HEAD_TAIL_MASK (0x80000000u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_HEAD_TAIL_SHIFT (0x0000001Fu)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_HEAD_TAIL_RESETVAL (0x00000000u)

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_PACKET_SIZE_MASK (0x0001FFFFu)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_PACKET_SIZE_SHIFT (0x00000000u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_PACKET_SIZE_RESETVAL (0x00000000u)

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_C_RESETVAL (0x00000000u)

/* QUEUE_REG_D */

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_DESC_PTR_MASK (0xFFFFFFF0u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_DESC_PTR_SHIFT (0x00000004u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_DESC_PTR_RESETVAL (0x00000000u)

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_DESC_INFO_MASK (0x0000000Fu)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_DESC_INFO_SHIFT (0x00000000u)
#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_DESC_INFO_RESETVAL (0x00000000u)

#define CSL_QM_QUEUE_MANAGEMENT_QUEUE_REG_D_RESETVAL (0x00000000u)

#endif
