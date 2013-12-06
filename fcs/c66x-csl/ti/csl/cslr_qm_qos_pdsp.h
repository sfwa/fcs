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
#ifndef _CSLR_QM_QOS_PDSP_H_
#define _CSLR_QM_QOS_PDSP_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>


/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for COMMAND_BUFFER
\**************************************************************************/
typedef struct  {
    volatile Uint32 COMMAND_BUFFER_WORD0;
    volatile Uint32 COMMAND_BUFFER_WORD1;
    volatile Uint8 RSVD0[8];
} CSL_Qm_qos_pdspCommand_bufferRegs;

/**************************************************************************\
* Register Overlay Structure for QOS_CLUSTER
\**************************************************************************/
typedef struct  {
    volatile Uint32 QOS_CLUSTER_WORD0;
    volatile Uint32 QOS_CLUSTER_WORD1;
    volatile Uint32 QOS_CLUSTER_WORD2;
    volatile Uint32 QOS_CLUSTER_WORD3;
    volatile Uint32 QOS_CLUSTER_WORD4;
    volatile Uint32 QOS_CLUSTER_WORD5;
    volatile Uint32 QOS_CLUSTER_WORD6;
    volatile Uint32 QOS_CLUSTER_WORD7;
    volatile Uint32 QOS_CLUSTER_WORD8;
    volatile Uint32 QOS_CLUSTER_WORD9;
    volatile Uint32 QOS_CLUSTER_WORD10;
    volatile Uint32 QOS_CLUSTER_WORD11;
    volatile Uint32 QOS_CLUSTER_WORD12;
    volatile Uint32 QOS_CLUSTER_WORD13;
} CSL_Qm_qos_pdspQos_clusterRegs;

/**************************************************************************\
* Register Overlay Structure for QOS_QUEUE
\**************************************************************************/
typedef struct  {
    volatile Uint32 QOS_QUEUE_WORD0;
    volatile Uint32 QOS_QUEUE_WORD1;
    volatile Uint32 QOS_QUEUE_WORD2;
    volatile Uint32 QOS_QUEUE_WORD3;
    volatile Uint32 QOS_QUEUE_WORD4;
    volatile Uint32 QOS_QUEUE_WORD5;
} CSL_Qm_qos_pdspQos_queueRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    CSL_Qm_qos_pdspCommand_bufferRegs COMMAND_BUFFER;
    volatile Uint8 RSVD1[48];
    CSL_Qm_qos_pdspQos_clusterRegs QOS_CLUSTER[8];
    CSL_Qm_qos_pdspQos_queueRegs QOS_QUEUE[64];
} CSL_Qm_qos_pdspRegs;

#endif
