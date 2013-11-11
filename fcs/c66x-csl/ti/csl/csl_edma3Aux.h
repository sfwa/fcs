/**
 *   @file  csl_edma3Aux.h
 *
 *   @brief   
 *      This is the EDMA Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the EDMA Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2002, 2003, 2004, 2005, 2008, 2009, Texas Instruments, Inc.
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

#ifndef _CSL_EDMA3AUX_H_
#define _CSL_EDMA3AUX_H_

#include <ti/csl/csl_edma3.h>

/** @addtogroup CSL_EDMA3_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_edma3GetInfo
 *
 *   @b Description
 *   @n The function gets the EDMA Channel Controller Configuration Information 
 *      which includes reading the peripheral revision register and configuration
 *      register.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        response    Output parameter populated with the configuration information.      
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_CFG,TPCC_TPCC_PID 
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_Edma3QueryInfo      info;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get Module Info
            CSL_edma3GetInfo(hModule,&info);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3GetInfo 
(
    CSL_Edma3Handle     hModule,
    CSL_Edma3QueryInfo* response
)
{
    /* Populate the configuration and peripheral id. */
    response->config   = hModule->regs->TPCC_CFG;
    response->revision = hModule->regs->TPCC_PID;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3MapDMAChannelToParamBlock
 *
 *   @b Description
 *   @n The function maps the DMA Channel to the specified PARAM Entry Block.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        dmaChannel  DMA Channel Number which is to be mapped.
        paramId     Parameter Identifier to be mapped to.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n DMA Channel is mapped to the specified PARAM Block.
 *
 *   @b Writes
 *   @n TPCC_TPCC_DCHMAP_PAENTRY
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Maps DMA Channel 1 to Param ID Block 5.
            CSL_edma3MapDMAChannelToParamBlock(hModule, 1, 5);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3MapDMAChannelToParamBlock
(
    CSL_Edma3Handle     hModule,
    Uint8               dmaChannel,
    Uint16              paramId
)
{
    /* Map the DMA channel to the parameter block. */
    CSL_FINS(hModule->regs->TPCC_DCHMAP[dmaChannel], TPCC_TPCC_DCHMAP0_PAENTRY, paramId);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetDMAChannelToParamBlockMapping
 *
 *   @b Description
 *   @n The function gets the PARAM Entry ID to which a specific DMA Channel 
 *      is mapped.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        dmaChannel  DMA Channel Number whose mapping is to be found.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Paramater ID to which the specific DMA Channel is mapped to.
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_DCHMAP_PAENTRY
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint16                  paramId;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the mapping information for DMA channel 1.
            paramId = CSL_edma3GetDMAChannelToParamBlockMapping(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint16 CSL_edma3GetDMAChannelToParamBlockMapping 
(
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel
)
{
    /* Get the Param Block to which the DMA channel is mapped to. */
    return (Uint16) CSL_FEXT(hModule->regs->TPCC_DCHMAP[dmaChannel], TPCC_TPCC_DCHMAP0_PAENTRY);
}

/** ============================================================================
 *   @n@b CSL_edma3MapQDMAChannelToParamBlock
 *
 *   @b Description
 *   @n The function maps the QDMA Channel to the specified PARAM Entry Block.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        qdmaChannel QDMA Channel Number which is to be mapped.
        paramId     Parameter Identifier to be mapped to.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n QDMA Channel is mapped to the specified PARAM Block.
 *
 *   @b Writes
 *   @n TPCC_TPCC_QCHMAP_PAENTRY
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Maps QDMA Channel 1 to Param ID Block 5.
            CSL_edma3MapQDMAChannelToParamBlock(hModule, 1, 5);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3MapQDMAChannelToParamBlock 
(
    CSL_Edma3Handle     hModule,
    Uint8               qdmaChannel,
    Uint16              paramId
)
{
    CSL_FINS(hModule->regs->TPCC_QCHMAP[qdmaChannel], TPCC_TPCC_QCHMAP0_PAENTRY, paramId);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetQDMAChannelToParamBlockMapping
 *
 *   @b Description
 *   @n The function gets the PARAM Entry ID to which a specific QDMA Channel 
 *      is mapped.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        qdmaChannel QDMA Channel Number whose mapping is to be found.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Paramater ID to which the specific QDMA Channel is mapped to.
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QCHMAP_PAENTRY
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the mapping information for QDMA channel 1.
            paramId = CSL_edma3GetQDMAChannelMap(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint16 CSL_edma3GetQDMAChannelToParamBlockMapping 
(
    CSL_Edma3Handle     hModule,
    Uint8               qdmaChannel
)
{   
    return (Uint16) CSL_FEXT(hModule->regs->TPCC_QCHMAP[qdmaChannel], TPCC_TPCC_QCHMAP0_PAENTRY);
}

/** ============================================================================
 *   @n@b CSL_edma3SetQDMATriggerWord
 *
 *   @b Description
 *   @n The function sets the trigger word of the PaRAM Entry block.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        qdmaChannel QDMA Channel Number which is to be configured.
        trword      Trigger Word to be configured.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n QDMA Channel is mapped to the specified PARAM Block.
 *
 *   @b Writes
 *   @n TPCC_TPCC_QCHMAP_TRWORD
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Configure QDMA Channel 1 Trigger Word as 0
            CSL_edma3SetQDMATriggerWord(hModule, 1, 0);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3SetQDMATriggerWord
(
    CSL_Edma3Handle     hModule,
    Uint8               qdmaChannel,
    Uint8               trword
)
{
    CSL_FINS(hModule->regs->TPCC_QCHMAP[qdmaChannel], TPCC_TPCC_QCHMAP0_TRWORD, trword);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetQDMATriggerWord
 *
 *   @b Description
 *   @n The function gets the trigger word of the PaRAM Entry block.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        qdmaChannel QDMA Channel Number which is to be configured.
        trword      Trigger Word to be retreived populated by this API.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n QDMA Channel is mapped to the specified PARAM Block.
 *
 *   @b Reads
 *   @n TPCC_TPCC_QCHMAP_TRWORD
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   trWord;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the QDMA Channel 1 Trigger Word
            trWord = CSL_edma3SetQDMATriggerWord(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetQDMATriggerWord
(
    CSL_Edma3Handle     hModule,
    Uint8               qdmaChannel,
    Uint8*              trword
)
{
    /* Extract the trigger word. */
    *trword = CSL_FEXT(hModule->regs->TPCC_QCHMAP[qdmaChannel], TPCC_TPCC_QCHMAP0_TRWORD);
}

/** ============================================================================
 *   @n@b CSL_edma3MapDMAChannelToEventQueue
 *
 *   @b Description
 *   @n The function maps the event ID to the specific DMA Queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        dmaChannel  DMA Channel to which the event is mapped.
        eventQueue  Event Queue which is to be mapped.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n DMA Channel is mapped to the specified Event Queue.
 *
 *   @b Writes
 *   @n TPCC_TPCC_DMAQNUM_E0;TPCC_TPCC_DMAQNUM_E1;TPCC_TPCC_DMAQNUM_E2;
 *      TPCC_TPCC_DMAQNUM_E3;TPCC_TPCC_DMAQNUM_E4;TPCC_TPCC_DMAQNUM_E5;
 *      TPCC_TPCC_DMAQNUM_E6;TPCC_TPCC_DMAQNUM_E7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Maps DMA Channel 1 to Event Queue 2
            CSL_edma3MapDMAChannelToEventQueue(hModule, 1, 2);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3MapDMAChannelToEventQueue
(
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel,
    Uint8           eventQueue    
)
{
    Uint8   dmaRegIndex;
    Uint8   lsb;

    /* There are 8 channels per register; use this to determine the DMAQNUM register Index. */
    dmaRegIndex = dmaChannel >> 3;

    /* Compute the bit position where the value is to be written. */
    lsb = (dmaChannel - (dmaRegIndex * 8)) << 2;

    /* Write the event Queue */
    CSL_FINSR(hModule->regs->TPCC_DMAQNUM[dmaRegIndex], lsb + 2, lsb, eventQueue);
}

/** ============================================================================
 *   @n@b CSL_edma3GetDMAChannelToEventQueueMapping
 *
 *   @b Description
 *   @n The function gets the mapping of the DMA Channel to the Event Queue 
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        dmaChannel  DMA Channel for which the mapping is to be retreived.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Event Queue to which the DMA channel is mapped to
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_DMAQNUM_E0;TPCC_TPCC_DMAQNUM_E1;TPCC_TPCC_DMAQNUM_E2;
 *      TPCC_TPCC_DMAQNUM_E3;TPCC_TPCC_DMAQNUM_E4;TPCC_TPCC_DMAQNUM_E5;
 *      TPCC_TPCC_DMAQNUM_E6;TPCC_TPCC_DMAQNUM_E7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   eventQueue;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the Event Queue mapping of DMA Channel 1
            eventQueue = CSL_edma3GetDMAChannelToEventQueueMapping(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_edma3GetDMAChannelToEventQueueMapping
(
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel
)
{
    Uint8   dmaRegIndex;
    Uint8   lsb;

    /* There are 8 channels per register; use this to determine the DMAQNUM register Index. */
    dmaRegIndex = dmaChannel >> 3;

    /* Compute the bit position from where the value is to be retreived. */
    lsb = (dmaChannel - (dmaRegIndex * 8)) << 2;

    /* Get the event queue. */
    return CSL_FEXTR(hModule->regs->TPCC_DMAQNUM[dmaRegIndex], lsb + 2, lsb);
}

/** ============================================================================
 *   @n@b CSL_edma3MapQDMAChannelToEventQueue
 *
 *   @b Description
 *   @n The function maps the event ID to the specific DMA Queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        qdmaChannel QDMA Channel to which the event is mapped.
        eventQueue  Event Queue which is to be mapped.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n DMA Channel is mapped to the specified Event Queue.
 *
 *   @b Writes
 *   @n TPCC_TPCC_QDMAQNUM_E0;TPCC_TPCC_QDMAQNUM_E1;TPCC_TPCC_QDMAQNUM_E2;
 *      TPCC_TPCC_QDMAQNUM_E3;TPCC_TPCC_QDMAQNUM_E4;TPCC_TPCC_QDMAQNUM_E5;
 *      TPCC_TPCC_QDMAQNUM_E6;TPCC_TPCC_QDMAQNUM_E7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Maps QDMA Channel 1 to Event Queue 2
            CSL_edma3MapQDMAChannelToEventQueue(hModule, 1, 2);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3MapQDMAChannelToEventQueue
(
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel,
    Uint8           eventQueue    
)
{
    Uint8   lsb;

    /* Compute the bit position where the value is to be written. */
    lsb = qdmaChannel << 2;

    /* Write the event Queue */
    CSL_FINSR(hModule->regs->TPCC_QDMAQNUM, lsb + 2, lsb, eventQueue);
}

/** ============================================================================
 *   @n@b CSL_edma3GetQDMAChannelToEventQueueMapping
 *
 *   @b Description
 *   @n The function gets the mapping of the QDMA Channel to the Event Queue 
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        qdmaChannel QDMA Channel for which the mapping is to be retreived.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Event Queue to which the QDMA channel is mapped to
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QDMAQNUM_E0;TPCC_TPCC_QDMAQNUM_E1;TPCC_TPCC_QDMAQNUM_E2;
 *      TPCC_TPCC_QDMAQNUM_E3;TPCC_TPCC_QDMAQNUM_E4;TPCC_TPCC_QDMAQNUM_E5;
 *      TPCC_TPCC_QDMAQNUM_E6;TPCC_TPCC_QDMAQNUM_E7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   eventQueue;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the Event Queue mapping of QDMA Channel 1
            eventQueue = CSL_edma3GetQDMAChannelToEventQueueMapping(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_edma3GetQDMAChannelToEventQueueMapping
(
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel
)
{
    Uint8   lsb;

    /* Compute the bit position from where the value is to be retreived. */
    lsb = qdmaChannel << 2;

    /* Get the event Queue */
    return CSL_FEXTR(hModule->regs->TPCC_QDMAQNUM, lsb + 2, lsb);
}

/** ============================================================================
 *   @n@b CSL_edma3MapEventQueueToTC
 *
 *   @b Description
 *   @n The function maps the event queue to a specific TC
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle        
        eventQueue  Event Queue which is to be mapped.
        tcNum       TC to which the queue is to be mapped to.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Event Queue is mapped to the specific TC
 *
 *   @b Writes
 *   @n TPCC_TPCC_QUETCMAP_TCNUMQ0;TPCC_TPCC_QUETCMAP_TCNUMQ1;TPCC_TPCC_QUETCMAP_TCNUMQ2;
 *      TPCC_TPCC_QUETCMAP_TCNUMQ3;TPCC_TPCC_QUETCMAP_TCNUMQ4;TPCC_TPCC_QUETCMAP_TCNUMQ5;
 *      TPCC_TPCC_QUETCMAP_TCNUMQ6;TPCC_TPCC_QUETCMAP_TCNUMQ7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Maps Event Queue 1 to TC0
            CSL_edma3MapEventQueueToTC(hModule, 1, 0);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3MapEventQueueToTC
(
    CSL_Edma3Handle hModule,
    Uint8           eventQueue,
    Uint8           tcNum
)
{
    Uint8   lsb;

    /* Compute the bit position where the value is to be written. */
    lsb = eventQueue << 2;

    /* Write the event Queue */
    CSL_FINSR(hModule->regs->TPCC_QUETCMAP, lsb + 2, lsb, tcNum);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetEventQueueToTCMapping
 *
 *   @b Description
 *   @n The function gets the TC mapping for the specific event queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle        
        eventQueue  Event Queue which for which the mapping is needed.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TC Number to which the event queue is mapped to
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Event Queue is mapped to the specific TC
 *
 *   @b Reads
 *   @n TPCC_TPCC_QUETCMAP_TCNUMQ0;TPCC_TPCC_QUETCMAP_TCNUMQ1;TPCC_TPCC_QUETCMAP_TCNUMQ2;
 *      TPCC_TPCC_QUETCMAP_TCNUMQ3;TPCC_TPCC_QUETCMAP_TCNUMQ4;TPCC_TPCC_QUETCMAP_TCNUMQ5;
 *      TPCC_TPCC_QUETCMAP_TCNUMQ6;TPCC_TPCC_QUETCMAP_TCNUMQ7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   tcNum;

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the TC mapping for Event Queue 1 
            tcNum = CSL_edma3GetEventQueueToTCMapping(hModule, 1, 0);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_edma3GetEventQueueToTCMapping
(
    CSL_Edma3Handle hModule,
    Uint8           eventQueue
)
{
    Uint8   lsb;

    /* Compute the bit position where the value is to be written. */
    lsb = eventQueue << 2;

    /* Get the TC Number */
    return CSL_FEXTR(hModule->regs->TPCC_QUETCMAP, lsb + 2, lsb);
}

/** ============================================================================
 *   @n@b CSL_edma3SetEventQueuePriority
 *
 *   @b Description
 *   @n The function sets the priority of the specific event queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle        
        eventQueue  Event Queue whose priority is to be configured.
        priority    Priority to be configured.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Event Queue is configured to the specific priority.
 *
 *   @b Writes
 *   @n TPCC_TPCC_QUEPRI_PRIQ0;TPCC_TPCC_QUEPRI_PRIQ1;TPCC_TPCC_QUEPRI_PRIQ2;
 *      TPCC_TPCC_QUEPRI_PRIQ3;TPCC_TPCC_QUEPRI_PRIQ4;TPCC_TPCC_QUEPRI_PRIQ5;
 *      TPCC_TPCC_QUEPRI_PRIQ6;TPCC_TPCC_QUEPRI_PRIQ7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;            

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Maps Event Queue 2 to Priority 4 
            CSL_edma3SetEventQueuePriority(hModule, 2, 4);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3SetEventQueuePriority
(
    CSL_Edma3Handle hModule,
    Uint8           eventQueue,
    Uint8           priority
)
{
    Uint8   lsb;

    /* Compute the bit position where the value is to be written. */
    lsb = eventQueue << 2;

    /* Write the priority. */
    CSL_FINSR(hModule->regs->TPCC_QUEPRI, lsb + 2, lsb, priority);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetEventQueuePriority
 *
 *   @b Description
 *   @n The function gets the priority of the specific event queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle        
        eventQueue  Event Queue whose priority is to be retrieved.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Priority to which the Event Queue is mapped to.
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Event Queue is configured to the specific priority.
 *
 *   @b Reads
 *   @n TPCC_TPCC_QUEPRI_PRIQ0;TPCC_TPCC_QUEPRI_PRIQ1;TPCC_TPCC_QUEPRI_PRIQ2;
 *      TPCC_TPCC_QUEPRI_PRIQ3;TPCC_TPCC_QUEPRI_PRIQ4;TPCC_TPCC_QUEPRI_PRIQ5;
 *      TPCC_TPCC_QUEPRI_PRIQ6;TPCC_TPCC_QUEPRI_PRIQ7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   priority;         

            // Module Initialization
            CSL_edma3Init(&context);

            // Module Level Open
            hModule = CSL_edma3Open(&edmaObj, CSL_EDMA3, NULL, &status);

            // Get the priority of Event Queue 2.
            priority = CSL_edma3GetEventQueuePriority(hModule, 2);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_edma3GetEventQueuePriority
(
    CSL_Edma3Handle hModule,
    Uint8           eventQueue
)
{
    Uint8   lsb;

    /* Compute the bit position where the value is to be written. */
    lsb = eventQueue << 2;

    /* Get the priority. */
    return CSL_FEXTR(hModule->regs->TPCC_QUEPRI, lsb + 2, lsb);
}

/** ============================================================================
 *   @n@b CSL_edma3GetEventMissed
 *
 *   @b Description
 *   @n Queries all the events missed.Since there may be upto 64 EDMA 
 *      channels + upto 8 QDMA channels,this points to an  array of 3, 
 *      32 bit elements.Gets the status of the missed events.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        missedLo          missed [0] - holds status from EMR
        missedHi          missed [1] - holds status from EMRH
        missedQdma        missed [2] - holds status from QEMR
     @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_EMR,TPCC_TPCC_EMRH,TPCC_TPCC_QEMR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           missedLo, missedHi, missedQdma;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get the missed events 
            CSL_edma3GetEventMissed(hModule, &missedEdma, &missedEdmaHi, &missedQdma);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetEventMissed 
(
    CSL_Edma3Handle hModule,
    CSL_BitMask32*  missedLo,
    CSL_BitMask32*  missedHi,
    CSL_BitMask32*  missedQdma
)
{
    /* Reading the missed Status registers */
    *missedLo    = hModule->regs->TPCC_EMR;
    *missedHi    = hModule->regs->TPCC_EMRH;
    *missedQdma  = hModule->regs->TPCC_QEMR;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3IsDMAChannelMissedEventSet
 *
 *   @b Description
 *   @n The API checks determines if there is a missed Event for a specific 
 *      DMA channel
 *
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        dmaChannel      DMA Channel to be checked
        response        This is populated by the API and returns TRUE if the
                        event was missed else it returns FALSE.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_EMR_EMR0;TPCC_TPCC_EMR_EMR1;TPCC_TPCC_EMR_EMR2;
 *      TPCC_TPCC_EMR_EMR3;TPCC_TPCC_EMR_EMR4;TPCC_TPCC_EMR_EMR5;
 *      TPCC_TPCC_EMR_EMR6;TPCC_TPCC_EMR_EMR7;TPCC_TPCC_EMR_EMR8;
 *      TPCC_TPCC_EMR_EMR9;TPCC_TPCC_EMR_EMR10;TPCC_TPCC_EMR_EMR11;
 *      TPCC_TPCC_EMR_EMR12;TPCC_TPCC_EMR_EMR13;TPCC_TPCC_EMR_EMR14;
 *      TPCC_TPCC_EMR_EMR15;TPCC_TPCC_EMR_EMR16;TPCC_TPCC_EMR_EMR17;
 *      TPCC_TPCC_EMR_EMR18;TPCC_TPCC_EMR_EMR19;TPCC_TPCC_EMR_EMR20;
 *      TPCC_TPCC_EMR_EMR21;TPCC_TPCC_EMR_EMR22;TPCC_TPCC_EMR_EMR23;
 *      TPCC_TPCC_EMR_EMR24;TPCC_TPCC_EMR_EMR25;TPCC_TPCC_EMR_EMR26;
 *      TPCC_TPCC_EMR_EMR27;TPCC_TPCC_EMR_EMR28;TPCC_TPCC_EMR_EMR29;
 *      TPCC_TPCC_EMR_EMR30;TPCC_TPCC_EMR_EMR31;
 *      TPCC_TPCC_EMRH_EMR32;TPCC_TPCC_EMRH_EMR33;TPCC_TPCC_EMRH_EMR34;
 *      TPCC_TPCC_EMRH_EMR35;TPCC_TPCC_EMRH_EMR36;TPCC_TPCC_EMRH_EMR37;
 *      TPCC_TPCC_EMRH_EMR38;TPCC_TPCC_EMRH_EMR39;TPCC_TPCC_EMRH_EMR40;
 *      TPCC_TPCC_EMRH_EMR41;TPCC_TPCC_EMRH_EMR42;TPCC_TPCC_EMRH_EMR43;
 *      TPCC_TPCC_EMRH_EMR44;TPCC_TPCC_EMRH_EMR45;TPCC_TPCC_EMRH_EMR46;
 *      TPCC_TPCC_EMRH_EMR47;TPCC_TPCC_EMRH_EMR48;TPCC_TPCC_EMRH_EMR49;
 *      TPCC_TPCC_EMRH_EMR50;TPCC_TPCC_EMRH_EMR51;TPCC_TPCC_EMRH_EMR52;
 *      TPCC_TPCC_EMRH_EMR53;TPCC_TPCC_EMRH_EMR54;TPCC_TPCC_EMRH_EMR55;
 *      TPCC_TPCC_EMRH_EMR56;TPCC_TPCC_EMRH_EMR57;TPCC_TPCC_EMRH_EMR58;
 *      TPCC_TPCC_EMRH_EMR59;TPCC_TPCC_EMRH_EMR60;TPCC_TPCC_EMRH_EMR61;
 *      TPCC_TPCC_EMRH_EMR62;TPCC_TPCC_EMRH_EMR63;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    missed;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Check if DMA Channel 1 has an event missed.
            CSL_edma3IsDMAChannelMissedEventSet(hModule, 1, &missed);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3IsDMAChannelMissedEventSet
(
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel,
    Bool*           response
)
{
    /* Determine which register needs to be looked into. */
    if (dmaChannel < 32) 
    {
        /* EMR: Extract the appropriate bit. */
        if (CSL_FEXTR(hModule->regs->TPCC_EMR, dmaChannel, dmaChannel))
            *response = TRUE;
        else
            *response = FALSE;
    } 
    else 
    {
        /* EMRH: Extract the appropriate bit. */
        if (CSL_FEXTR(hModule->regs->TPCC_EMRH, dmaChannel-32, dmaChannel-32))
            *response = TRUE;
        else
            *response = FALSE;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3IsQDMAChannelMissedEventSet
 *
 *   @b Description
 *   @n The API checks determines if there is a missed Event for a specific 
 *      DMA channel
 *
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaChannel     QDMA Channel to be checked
        response        This is populated by the API and returns TRUE if the
                        event was missed else it returns FALSE.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QEMR_QEMR0;TPCC_TPCC_QEMR_QEMR1;TPCC_TPCC_QEMR_QEMR2;
 *      TPCC_TPCC_QEMR_QEMR3;TPCC_TPCC_QEMR_QEMR4;TPCC_TPCC_QEMR_QEMR5;
 *      TPCC_TPCC_QEMR_QEMR6;TPCC_TPCC_QEMR_QEMR7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    missed;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Check if DMA Channel 0 has an event missed.
            CSL_edma3IsQDMAChannelMissedEventSet(hModule, 0, &missed);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3IsQDMAChannelMissedEventSet
(
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel,
    Bool*           response
)
{
    /* Extract the appropriate QDMA Channel bit. */
    if (CSL_FEXTR(hModule->regs->TPCC_QEMR, qdmaChannel, qdmaChannel))
        *response = TRUE;
    else
        *response = FALSE;
    
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3EventMissedClear
 *
 *   @b Description
 *   @n Clear the Event missed errors
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        missedLo          Lower 32 of of the Event Missed register needing to 
                          be cleared (This is the same value as EMR)
        missedHi          Upper 32 of of the Event Missed register needing to 
                          be cleared (This is the same value as EMRH)
        missedQdma        Bit mask of Qdma events missed needing to be cleared
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Clears all the missed events
 *
 *   @b Writes
 *   @n TPCC_TPCC_EMCR,TPCC_TPCC_EMCRH,TPCC_TPCC_QEMCR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           missedLo, missedHi, missedQdma;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...            
            // Get the missed events 
            CSL_edma3GetEventMissed(hModule, &missedEdma, &missedEdmaHi, &missedQdma);
            ...
            // Clear the error 
            CSL_edma3EventMissedClear(hModule,missedLo, missedHi,qdmamissed);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3EventsMissedClear 
(
    CSL_Edma3Handle hModule,
    CSL_BitMask32   missedLo,
    CSL_BitMask32   missedHi,
    CSL_BitMask32   missedQdma
)
{
    /* Clear the reported missed events. */
    hModule->regs->TPCC_EMCR   = missedLo;
    hModule->regs->TPCC_EMCRH  = missedHi;
    hModule->regs->TPCC_QEMCR  = missedQdma;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearDMAMissedEvent
 *
 *   @b Description
 *   @n The API clears the missed event for the specific DMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        dmaChannel      DMA Channel for which the event is cleared.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Clears all the missed events
 *
 *   @b Writes
 *   @n TPCC_TPCC_EMCR_EMCR0;TPCC_TPCC_EMCR_EMCR1;TPCC_TPCC_EMCR_EMCR2;
 *      TPCC_TPCC_EMCR_EMCR3;TPCC_TPCC_EMCR_EMCR4;TPCC_TPCC_EMCR_EMCR5;
 *      TPCC_TPCC_EMCR_EMCR6;TPCC_TPCC_EMCR_EMCR7;TPCC_TPCC_EMCR_EMCR8;
 *      TPCC_TPCC_EMCR_EMCR9;TPCC_TPCC_EMCR_EMCR10;TPCC_TPCC_EMCR_EMCR11;
 *      TPCC_TPCC_EMCR_EMCR12;TPCC_TPCC_EMCR_EMCR13;TPCC_TPCC_EMCR_EMCR14;
 *      TPCC_TPCC_EMCR_EMCR15;TPCC_TPCC_EMCR_EMCR16;TPCC_TPCC_EMCR_EMCR17;
 *      TPCC_TPCC_EMCR_EMCR18;TPCC_TPCC_EMCR_EMCR19;TPCC_TPCC_EMCR_EMCR20;
 *      TPCC_TPCC_EMCR_EMCR21;TPCC_TPCC_EMCR_EMCR22;TPCC_TPCC_EMCR_EMCR23;
 *      TPCC_TPCC_EMCR_EMCR24;TPCC_TPCC_EMCR_EMCR25;TPCC_TPCC_EMCR_EMCR26;
 *      TPCC_TPCC_EMCR_EMCR27;TPCC_TPCC_EMCR_EMCR28;TPCC_TPCC_EMCR_EMCR29;
 *      TPCC_TPCC_EMCR_EMCR30;TPCC_TPCC_EMCR_EMCR31;
 *      TPCC_TPCC_EMCRH_EMCR32;TPCC_TPCC_EMCRH_EMCR33;TPCC_TPCC_EMCRH_EMCR34
 *      TPCC_TPCC_EMCRH_EMCR35;TPCC_TPCC_EMCRH_EMCR36;TPCC_TPCC_EMCRH_EMCR37
 *      TPCC_TPCC_EMCRH_EMCR38;TPCC_TPCC_EMCRH_EMCR39;TPCC_TPCC_EMCRH_EMCR40
 *      TPCC_TPCC_EMCRH_EMCR41;TPCC_TPCC_EMCRH_EMCR42;TPCC_TPCC_EMCRH_EMCR43
 *      TPCC_TPCC_EMCRH_EMCR44;TPCC_TPCC_EMCRH_EMCR45;TPCC_TPCC_EMCRH_EMCR46
 *      TPCC_TPCC_EMCRH_EMCR47;TPCC_TPCC_EMCRH_EMCR48;TPCC_TPCC_EMCRH_EMCR49
 *      TPCC_TPCC_EMCRH_EMCR50;TPCC_TPCC_EMCRH_EMCR51;TPCC_TPCC_EMCRH_EMCR52
 *      TPCC_TPCC_EMCRH_EMCR53;TPCC_TPCC_EMCRH_EMCR54;TPCC_TPCC_EMCRH_EMCR55
 *      TPCC_TPCC_EMCRH_EMCR56;TPCC_TPCC_EMCRH_EMCR57;TPCC_TPCC_EMCRH_EMCR58
 *      TPCC_TPCC_EMCRH_EMCR59;TPCC_TPCC_EMCRH_EMCR60;TPCC_TPCC_EMCRH_EMCR61
 *      TPCC_TPCC_EMCRH_EMCR62;TPCC_TPCC_EMCRH_EMCR63;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           missedLo, missedHi, missedQdma;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...            
            // Clear missed DMA 1 channel event.
            CSL_edma3ClearDMAMissedEvent(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearDMAMissedEvent
(
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel
)
{
    /* Determine which register needs to be looked into. */
	if (dmaChannel < 32)
    {
        /* EMCR: Set the appropriate DMA Channel bit.  */
        hModule->regs->TPCC_EMCR  = CSL_FMKR (dmaChannel, dmaChannel, 1);
    }
    else
	{
        /* EMCRH: Set the appropriate DMA Channel bit. */
        dmaChannel = dmaChannel - 32;
        hModule->regs->TPCC_EMCRH = CSL_FMKR (dmaChannel, dmaChannel, 1);
	}
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearQDMAMissedEvent
 *
 *   @b Description
 *   @n The API clears the missed event for the specific QDMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaChannel     QDMA Channel for which the event is cleared.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n Clears all the missed events
 *
 *   @b Writes
 *   @n TPCC_TPCC_QEMCR_QEMCR0;TPCC_TPCC_QEMCR_QEMCR1;TPCC_TPCC_QEMCR_QEMCR2;
 *      TPCC_TPCC_QEMCR_QEMCR3;TPCC_TPCC_QEMCR_QEMCR4;TPCC_TPCC_QEMCR_QEMCR5;
 *      TPCC_TPCC_QEMCR_QEMCR6;TPCC_TPCC_QEMCR_QEMCR7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           missedLo, missedHi, missedQdma;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...            
            // Clear missed QDMA Channel 1 event.
            CSL_edma3ClearQDMAMissedEvent(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearQDMAMissedEvent
(
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel
)
{    
    /* Set the appropriate QDMA Channel bit.  */
    CSL_FINSR(hModule->regs->TPCC_QEMCR, qdmaChannel, qdmaChannel, 1);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetControllerError
 *
 *   @b Description
 *   @n The function gets the status of the controller error.
 *
 *   @b Arguments
 *   @verbatim
        hModule     Module Handle
        ccStat      Controller Error populated by this API 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_CCERR_QTHRXD0;TPCC_TPCC_CCERR_QTHRXD1;TPCC_TPCC_CCERR_QTHRXD2;
 *      TPCC_TPCC_CCERR_QTHRXD3;TPCC_TPCC_CCERR_QTHRXD4;TPCC_TPCC_CCERR_QTHRXD5;
 *      TPCC_TPCC_CCERR_QTHRXD6;TPCC_TPCC_CCERR_QTHRXD7,
 *      TPCC_TPCC_CCERR_TCCERR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_Edma3CtrlErrStat    ccError;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get Controller Error
            status = CSL_edma3GetControllerError(hModule,&ccError);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetControllerError 
(
    CSL_Edma3Handle         hModule,
    CSL_Edma3CtrlErrStat*   ccStat
)
{
    /* Extract the Queue Thresholds and TCC-Error. */
    ccStat->error     = hModule->regs->TPCC_CCERR & 0xFF ; 
    ccStat->exceedTcc = (Bool)(CSL_FEXT(hModule->regs->TPCC_CCERR, TPCC_TPCC_CCERR_TCCERR));
    return;     
}

/** ============================================================================
 *   @n@b CSL_edma3ClearControllerError
 *
 *   @b Description
 *   @n Channel Controller Error Fault.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle        
        ccStat            Error Status which is to be cleared.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_CCERRCLR_QTHRXD0;TPCC_TPCC_CCERRCLR_QTHRXD1;TPCC_TPCC_CCERRCLR_QTHRXD2;
 *      TPCC_TPCC_CCERRCLR_QTHRXD3;TPCC_TPCC_CCERRCLR_QTHRXD4;TPCC_TPCC_CCERRCLR_QTHRXD5;
 *      TPCC_TPCC_CCERRCLR_QTHRXD6;TPCC_TPCC_CCERRCLR_QTHRXD7,
 *      TPCC_TPCC_CCERR_TCCERR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_Edma3CtrlErrStat    ccError;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get Controller Error
            status = CSL_edma3GetControllerError(hModule,&ccError);
            ...
            // Clear the error. 
            CSL_edma3ClearControllerError(hModule,&ccError);
            ...
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void CSL_edma3ClearControllerError 
(
    CSL_Edma3Handle         hModule,
    CSL_Edma3CtrlErrStat*   ccStat
)
{
    /* Clears the errors */
    hModule->regs->TPCC_CCERRCLR = CSL_FMK(TPCC_TPCC_CCERR_TCCERR, ccStat->exceedTcc) | ccStat->error;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ErrorEval
 *
 *   @b Description
 *   @n This API enables enables evaluation of errros for the specified 
 *      view/shadow region.Sets EVAL bit of the EEVAL register in the Global 
 *      register space
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_EEVAL_EVAL=1
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Set the Error Interrupt Evaluation 
            CSL_edma3ErrorEval(hModule);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3ErrorEval (CSL_Edma3Handle hModule)
{
    hModule->regs->TPCC_EEVAL = CSL_FMK(TPCC_TPCC_EEVAL_EVAL, 1);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3DmaRegionAccessEnable
 *
 *   @b Description
 *   @n This API enables read/write access to the shadow regions for the 
 *      specific DMA channels.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        edmaRegion        Shadow Region 
        access            Region bits to be programmed
        drae              Bitmask to be enabled in DRAE
        draeh             Bitmask to be enabled in DRAEH
     @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_DRA_DRAE_E0=1;TPCC_TPCC_DRA_DRAE_E1=1;TPCC_TPCC_DRA_DRAE_E2=1;
 *      TPCC_TPCC_DRA_DRAE_E3=1;TPCC_TPCC_DRA_DRAE_E4=1;TPCC_TPCC_DRA_DRAE_E5=1;
 *      TPCC_TPCC_DRA_DRAE_E6=1;TPCC_TPCC_DRA_DRAE_E7=1;TPCC_TPCC_DRA_DRAE_E8=1;
 *      TPCC_TPCC_DRA_DRAE_E9=1;TPCC_TPCC_DRA_DRAE_E10=1;TPCC_TPCC_DRA_DRAE_E11=1;
 *      TPCC_TPCC_DRA_DRAE_E12=1;TPCC_TPCC_DRA_DRAE_E13=1;TPCC_TPCC_DRA_DRAE_E14=1;
 *      TPCC_TPCC_DRA_DRAE_E15=1;TPCC_TPCC_DRA_DRAE_E16=1;TPCC_TPCC_DRA_DRAE_E17=1;
 *      TPCC_TPCC_DRA_DRAE_E18=1;TPCC_TPCC_DRA_DRAE_E19=1;TPCC_TPCC_DRA_DRAE_E20=1;
 *      TPCC_TPCC_DRA_DRAE_E21=1;TPCC_TPCC_DRA_DRAE_E22=1;TPCC_TPCC_DRA_DRAE_E23=1;
 *      TPCC_TPCC_DRA_DRAE_E24=1;TPCC_TPCC_DRA_DRAE_E25=1;TPCC_TPCC_DRA_DRAE_E26=1;
 *      TPCC_TPCC_DRA_DRAE_E27=1;TPCC_TPCC_DRA_DRAE_E28=1;TPCC_TPCC_DRA_DRAE_E29=1;
 *      TPCC_TPCC_DRA_DRAE_E30=1;TPCC_TPCC_DRA_DRAE_E31=1;
 *   @n TPCC_TPCC_DRA_DRAEH_E32=1;TPCC_TPCC_DRA_DRAEH_E33=1;TPCC_TPCC_DRA_DRAEH_E34=1;
 *      TPCC_TPCC_DRA_DRAEH_E35=1;TPCC_TPCC_DRA_DRAEH_E36=1;TPCC_TPCC_DRA_DRAEH_E37=1;
 *      TPCC_TPCC_DRA_DRAEH_E38=1;TPCC_TPCC_DRA_DRAEH_E39=1;TPCC_TPCC_DRA_DRAEH_E40=1;
 *      TPCC_TPCC_DRA_DRAEH_E41=1;TPCC_TPCC_DRA_DRAEH_E42=1;TPCC_TPCC_DRA_DRAEH_E43=1;
 *      TPCC_TPCC_DRA_DRAEH_E44=1;TPCC_TPCC_DRA_DRAEH_E45=1;TPCC_TPCC_DRA_DRAEH_E46=1;
 *      TPCC_TPCC_DRA_DRAEH_E47=1;TPCC_TPCC_DRA_DRAEH_E48=1;TPCC_TPCC_DRA_DRAEH_E49=1;
 *      TPCC_TPCC_DRA_DRAEH_E50=1;TPCC_TPCC_DRA_DRAEH_E51=1;TPCC_TPCC_DRA_DRAEH_E52=1;
 *      TPCC_TPCC_DRA_DRAEH_E53=1;TPCC_TPCC_DRA_DRAEH_E54=1;TPCC_TPCC_DRA_DRAEH_E55=1;
 *      TPCC_TPCC_DRA_DRAEH_E56=1;TPCC_TPCC_DRA_DRAEH_E57=1;TPCC_TPCC_DRA_DRAEH_E58=1;
 *      TPCC_TPCC_DRA_DRAEH_E59=1;TPCC_TPCC_DRA_DRAEH_E60=1;TPCC_TPCC_DRA_DRAEH_E61=1;
 *      TPCC_TPCC_DRA_DRAEH_E62=1;TPCC_TPCC_DRA_DRAEH_E63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Enable read/write access in Region 0 for DMA Channel 0 to 7
            CSL_edma3DmaRegionAccessEnable(hModule, 0, 0x000000FF, 0x0);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3DmaRegionAccessEnable 
(
    CSL_Edma3Handle       hModule,
    Int                   edmaRegion,
    CSL_BitMask32         drae,
    CSL_BitMask32         draeh
)
{
    /* Set the appropriate bit masks. */
    hModule->regs->TPCC_DRA[edmaRegion].DRAE  |= drae;
    hModule->regs->TPCC_DRA[edmaRegion].DRAEH |= draeh;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3DmaRegionAccessDisable
 *
 *   @b Description
 *   @n This API disables read/write access to the shadow regions for the 
 *      specific DMA channels.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        edmaRegion        Shadow Region 
        access            Region bits to be programmed
        drae              Bitmask to be disabled in DRAE
        draeh             Bitmask to be disabled in DRAEH
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_DRA_DRAE_E0=0;TPCC_TPCC_DRA_DRAE_E1=0;TPCC_TPCC_DRA_DRAE_E2=0;
 *      TPCC_TPCC_DRA_DRAE_E3=0;TPCC_TPCC_DRA_DRAE_E4=0;TPCC_TPCC_DRA_DRAE_E5=0;
 *      TPCC_TPCC_DRA_DRAE_E6=0;TPCC_TPCC_DRA_DRAE_E7=0;TPCC_TPCC_DRA_DRAE_E8=0;
 *      TPCC_TPCC_DRA_DRAE_E9=0;TPCC_TPCC_DRA_DRAE_E10=0;TPCC_TPCC_DRA_DRAE_E11=0;
 *      TPCC_TPCC_DRA_DRAE_E12=0;TPCC_TPCC_DRA_DRAE_E13=0;TPCC_TPCC_DRA_DRAE_E14=0;
 *      TPCC_TPCC_DRA_DRAE_E15=0;TPCC_TPCC_DRA_DRAE_E16=0;TPCC_TPCC_DRA_DRAE_E17=0;
 *      TPCC_TPCC_DRA_DRAE_E18=0;TPCC_TPCC_DRA_DRAE_E19=0;TPCC_TPCC_DRA_DRAE_E20=0;
 *      TPCC_TPCC_DRA_DRAE_E21=0;TPCC_TPCC_DRA_DRAE_E22=0;TPCC_TPCC_DRA_DRAE_E23=0;
 *      TPCC_TPCC_DRA_DRAE_E24=0;TPCC_TPCC_DRA_DRAE_E25=0;TPCC_TPCC_DRA_DRAE_E26=0;
 *      TPCC_TPCC_DRA_DRAE_E27=0;TPCC_TPCC_DRA_DRAE_E28=0;TPCC_TPCC_DRA_DRAE_E29=0;
 *      TPCC_TPCC_DRA_DRAE_E30=0;TPCC_TPCC_DRA_DRAE_E31=0;
 *   @n TPCC_TPCC_DRA_DRAEH_E32=0;TPCC_TPCC_DRA_DRAEH_E33=0;TPCC_TPCC_DRA_DRAEH_E34=0;
 *      TPCC_TPCC_DRA_DRAEH_E35=0;TPCC_TPCC_DRA_DRAEH_E36=0;TPCC_TPCC_DRA_DRAEH_E37=0;
 *      TPCC_TPCC_DRA_DRAEH_E38=0;TPCC_TPCC_DRA_DRAEH_E39=0;TPCC_TPCC_DRA_DRAEH_E40=0;
 *      TPCC_TPCC_DRA_DRAEH_E41=0;TPCC_TPCC_DRA_DRAEH_E42=0;TPCC_TPCC_DRA_DRAEH_E43=0;
 *      TPCC_TPCC_DRA_DRAEH_E44=0;TPCC_TPCC_DRA_DRAEH_E45=0;TPCC_TPCC_DRA_DRAEH_E46=0;
 *      TPCC_TPCC_DRA_DRAEH_E47=0;TPCC_TPCC_DRA_DRAEH_E48=0;TPCC_TPCC_DRA_DRAEH_E49=0;
 *      TPCC_TPCC_DRA_DRAEH_E50=0;TPCC_TPCC_DRA_DRAEH_E51=0;TPCC_TPCC_DRA_DRAEH_E52=0;
 *      TPCC_TPCC_DRA_DRAEH_E53=0;TPCC_TPCC_DRA_DRAEH_E54=0;TPCC_TPCC_DRA_DRAEH_E55=0;
 *      TPCC_TPCC_DRA_DRAEH_E56=0;TPCC_TPCC_DRA_DRAEH_E57=0;TPCC_TPCC_DRA_DRAEH_E58=0;
 *      TPCC_TPCC_DRA_DRAEH_E59=0;TPCC_TPCC_DRA_DRAEH_E60=0;TPCC_TPCC_DRA_DRAEH_E61=0;
 *      TPCC_TPCC_DRA_DRAEH_E62=0;TPCC_TPCC_DRA_DRAEH_E63=0;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Disable read/write access in Region 0 for DMA Channel 0 to 7
            CSL_edma3DmaRegionAccessDisable(hModule, 0, 0x000000FF, 0x0);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3DmaRegionAccessDisable 
(
    CSL_Edma3Handle hModule,
    Int             edmaRegion,
    CSL_BitMask32   drae,
    CSL_BitMask32   draeh
)
{
    /* Clear the appropriate bit masks. */ 
    hModule->regs->TPCC_DRA[edmaRegion].DRAE  &= ~drae;
    hModule->regs->TPCC_DRA[edmaRegion].DRAEH &= ~draeh;    
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3QdmaRegionAccessEnable
 *
 *   @b Description
 *   @n This API enables read/write access to the shadow regions for the 
 *      specific QDMA channels.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        edmaRegion        Shadow Region
        qrae              Bitmask to be enabled in QRAE
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QRAE_E0=1;TPCC_TPCC_QRAE_E1=1;TPCC_TPCC_QRAE_E2=1;
 *      TPCC_TPCC_QRAE_E3=1;TPCC_TPCC_QRAE_E4=1;TPCC_TPCC_QRAE_E5=1;
 *      TPCC_TPCC_QRAE_E6=1;TPCC_TPCC_QRAE_E7=1
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Enable read/write access in Region 0 for QDMA 0 to 3
            CSL_edma3QdmaRegionAccessEnable(hModule, 0, 0x0000000F);

     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3QdmaRegionAccessEnable 
(
    CSL_Edma3Handle hModule,
    Int             edmaRegion,
    CSL_BitMask32   qrae
)
{
    /* Set the appropriate bit masks. */
    hModule->regs->TPCC_QRAE[edmaRegion] |= qrae;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3QdmaRegionAccessDisable
 *
 *   @b Description
 *   @n This API disables read/write access to the shadow regions for the 
 *      specific QDMA channels.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle        
        edmaRegion        Shadow Region.                          
        qrae              Bitmask to be enabled in QRAE
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QRAE_E0=0;TPCC_TPCC_QRAE_E1=0;TPCC_TPCC_QRAE_E2=0;
 *      TPCC_TPCC_QRAE_E3=0;TPCC_TPCC_QRAE_E4=0;TPCC_TPCC_QRAE_E5=0;
 *      TPCC_TPCC_QRAE_E6=0;TPCC_TPCC_QRAE_E7=0
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Disable read/write access in Region 0 for QDMA 0 to 3
            CSL_edma3QdmaRegionAccessDisable(hModule, 0, 0x0000000F);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3QdmaRegionAccessDisable 
(
    CSL_Edma3Handle        hModule,
    Int                    edmaRegion,
    CSL_BitMask32          qrae
)
{ 
    /* Clear the appropriate bits */       
    hModule->regs->TPCC_QRAE[edmaRegion] &= ~qrae;    
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetWaterMark
 *
 *   @b Description
 *   @n The function gets the Queue Watermark for the specific event queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        eventQueue        Event queue number for which the watermark is retreived.
        waterMark         This is populated by the API to the configured water mark 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QSTAT_WM
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   waterMark;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Get the Water Mark Queue for event queue 0
            CSL_edma3GetWaterMark(hModule, 0, &waterMark);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetWaterMark 
(
    CSL_Edma3Handle  hModule,
    Uint8            eventQueue,
    Uint8*           waterMark
)
{
    /* Extract the watermark from the appropriate event queue. */
    *waterMark = CSL_FEXT(hModule->regs->TPCC_QSTAT[eventQueue], TPCC_TPCC_QSTAT0_WM);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetNumberValidEntries
 *
 *   @b Description
 *   @n The function gets the Number of valid entries for the specific event queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        eventQueue        Event queue number for which the watermark is retreived.
        numValidEntries   This is populated by the API to the number of valid entries
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QSTAT_NUMVAL
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   numVal;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Get the Number of valid entries in event queue 0.
            CSL_edma3GetNumberValidEntries(hModule, 0, &numVal);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetNumberValidEntries
(
    CSL_Edma3Handle  hModule,
    Uint8            eventQueue,
    Uint8*           numValidEntries
)
{
    /* Extract the number of valid entries from the appropriate event queue. */
    *numValidEntries = CSL_FEXT(hModule->regs->TPCC_QSTAT[eventQueue], TPCC_TPCC_QSTAT0_NUMVAL);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetStartPointer
 *
 *   @b Description
 *   @n The function gets the Number of valid entries for the specific event queue.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        eventQueue        Event queue number for which the watermark is retreived.
        startPtr          This is populated by the API to the start pointer
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QSTAT_STRPTR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   startPtr;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Get the Number of valid entries in event queue 0.
            CSL_edma3GetStartPointer(hModule, 0, &startPtr);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetStartPointer
(
    CSL_Edma3Handle  hModule,
    Uint8            eventQueue,
    Uint8*           startPtr
)
{
    /* Extract the start pointer from the appropriate event queue. */
    *startPtr = CSL_FEXT(hModule->regs->TPCC_QSTAT[eventQueue], TPCC_TPCC_QSTAT0_STRPTR);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetThresholdExceeded
 *
 *   @b Description
 *   @n The function gets the threshold exceeded flag for the specific event queue.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        eventQueue        Event queue number for which the watermark is retreived.
        thresholdExceeded This is populated by the API to the threshold exceeded flag
                          for the specific event queue.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QSTAT_THRXCD
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Uint8                   thresholdExceeded;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Determine if the threshold has been exceeded or not for Queue 1
            CSL_edma3GetThresholdExceeded(hModule, 1, &thresholdExceeded);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetThresholdExceeded
(
    CSL_Edma3Handle  hModule,
    Uint8            eventQueue,
    Bool*            thresholdExceeded
)
{
    /* Extract the threshold exceeded from the appropriate event queue. */
    *thresholdExceeded = (Bool)CSL_FEXT(hModule->regs->TPCC_QSTAT[eventQueue], TPCC_TPCC_QSTAT0_THRXCD);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3EventQueueThresholdSet
 *
 *   @b Description
 *   @n The function configures the queue threshold.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        eventQueue        Event queue for which the threshold is configured                          
        threshold         Target threshold value.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QWMTHRA_Q0;TPCC_TPCC_QWMTHRA_Q1;TPCC_TPCC_QWMTHRA_Q2;
 *      TPCC_TPCC_QWMTHRA_Q3;
 *   @n TPCC_TPCC_QWMTHRB_Q4;TPCC_TPCC_QWMTHRB_Q1;TPCC_TPCC_QWMTHRB_Q2;
 *      TPCC_TPCC_QWMTHRB_Q3
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
         
            // Set the Queue threshold for Event Queue 0 to be 9
            CSL_edma3EventQueueThresholdSet(hModule, 0, 9);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3EventQueueThresholdSet 
(
    CSL_Edma3Handle  hModule,
    Uint8            eventQueue,
    Uint8            threshold
)
{
    /* Determine which register needs to be configured: QWMTHRA or QWMTHRB */
	if (eventQueue < 4)
    {
        /* TPCC_QWMTHRA: Set the correct bits with the threshold value. */
		CSL_FINSR(hModule->regs->TPCC_QWMTHRA, (8 * eventQueue + 4), (8 * eventQueue),threshold);
    }
    else
	{
        /* TPCC_QWMTHRB: Set the correct bits with the threshold value. */
		eventQueue -= 4;
		CSL_FINSR(hModule->regs->TPCC_QWMTHRB, (8 * eventQueue + 4), (8 * eventQueue), threshold);
	}
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetActivityStatus
 *
 *   @b Description
 *   @n Obtains the Channel Controller Activity Status
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        activityStat      Activity Status populated by this API.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_CCSTAT_EVTACTV,TPCC_TPCC_CCSTAT_QEVTACTV,TPCC_TPCC_CCSTAT_TRACTV,
 *      TPCC_TPCC_CCSTAT_ACTV,TPCC_TPCC_CCSTAT_COMP_ACTV
 *
 *   @b Example
 *   @verbatim

            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_Edma3ActivityStat   activityStat;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get the CC activity status.
            CSL_edma3GetActivityStatus(hModule,&activityStat);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3GetActivityStatus 
(
    CSL_Edma3Handle         hModule,
    CSL_Edma3ActivityStat*  activityStat
)
{
    Uint32 value = hModule->regs->TPCC_CCSTAT;

    /* Populate the activity status structure. */
    activityStat->evtActive      = (Bool)CSL_FEXT(value, TPCC_TPCC_CCSTAT_EVTACTV);
    activityStat->qevtActive     = (Bool)CSL_FEXT(value, TPCC_TPCC_CCSTAT_QEVTACTV);
    activityStat->trActive       = (Bool)CSL_FEXT(value, TPCC_TPCC_CCSTAT_TRACTV); 
    activityStat->active         = (Bool)CSL_FEXT(value, TPCC_TPCC_CCSTAT_ACTV);
    activityStat->outstandingTcc = CSL_FEXT(value, TPCC_TPCC_CCSTAT_COMP_ACTV);
    activityStat->queActive      = CSL_FEXTR(value, 23,16);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetMemoryFaultError
 *
 *   @b Description
 *   @n The function gets the Controllers memory fault error and the error 
 *      attributes.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        memFault          The structure is populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>  
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_MPFAR_FADDR,TPCC_TPCC_MPFSR_FID,
 *      TPCC_TPCC_MPFSR_UXE,TPCC_TPCC_MPFSR_UWE,TPCC_TPCC_MPFSR_URE,
 *      TPCC_TPCC_MPFSR_SXE,TPCC_TPCC_MPFSR_SWE,TPCC_TPCC_MPFSR_SRE,
 *      TPCC_TPCC_MPFSR_SECE
 *
 *   @b Example
 *   @verbatim

            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_Edma3MemFaultStat   memFault;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get memory protection fault 
            CSL_edma3GetMemoryFaultError(hModule, &memFault);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetMemoryFaultError 
(
    CSL_Edma3Handle             hModule,
    CSL_Edma3MemFaultStat*      memFault
)
{
    Uint32 value = hModule->regs->TPCC_MPFSR;

    /* Extract the memory fault address. */
    memFault->addr  = CSL_FEXT(hModule->regs->TPCC_MPFAR, TPCC_TPCC_MPFAR_FADDR); 

    /* Extract the fault ID */
    memFault->fid   = CSL_FEXT(value, TPCC_TPCC_MPFSR_FID);

    /* Extract the error access bits. */
    memFault->error = CSL_FEXTR(value, 8, 0);
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3MemFaultClear
 *
 *   @b Description
 *   @n The function clears the memory fault.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_MPFCR_MPFCLR=1
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Clear the memory protection fault 
            CSL_edma3MemFaultClear(hModule);
            ...
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void CSL_edma3MemFaultClear (CSL_Edma3Handle hModule)
{
    /* Write a 1 to clear the memory fault. */
    hModule->regs->TPCC_MPFCR = CSL_FMK(TPCC_TPCC_MPFCR_MPFCLR, 1);
    return; 
}

/** ============================================================================
 *   @n@b CSL_edma3GetMemoryProtectionAttrib
 *
 *   @b Description
 *   @n The function gets the memory access/protection attributes of the 
 *   specific region.
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region being queried.
        mppa              Memory Access/Protection Attributes populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_MPPAG;TPCC_TPCC_MPPA
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           mppa;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get memory protection attributes for the Global Region.
            CSL_edma3GetMemoryProtectionAttrib(hModule, -1, &mppa);
            ...
            // Get memory protection attributes for region 2 
            CSL_edma3GetMemoryProtectionAttrib(hModule, 2, &mppa);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetMemoryProtectionAttrib 
(
    CSL_Edma3Handle     hModule,
    Int                 region,
    CSL_BitMask32*      mppa
)
{
    /* Determine which region is being queried. */
    if (region < 0)
    {
        /* Get the Global Memory Protection Attributes */
        *mppa = hModule->regs->TPCC_MPPAG;
    }
    else
    {
        /* Get the Memory Protection Attributes for the specific region. */
        *mppa = hModule->regs->TPCC_MPPA[region];
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3SetMemoryProtectionAttrib
 *
 *   @b Description
 *   @n This API sets the memory protection attributes for the specified region.
 *
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region being configured.                          
        mpa             Value to be programmed into the MPPAG/MPPA[0/1/2/../n]
                        This is a Bitmask of the protection attributes.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_MPPAG;TPCC_TPCC_MPPA
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Set the memory protection attributes of region 0. 
            CSL_edma3SetMemoryProtectionAttrib (hModule, 0, CSL_EDMA3_MEMACCESS_UX |
                                                            CSL_EDMA3_MEMACCESS_UW |
                                                            CSL_EDMA3_MEMACCESS_UR |
                                                            CSL_EDMA3_MEMACCESS_AID2));
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3SetMemoryProtectionAttrib
(
    CSL_Edma3Handle        hModule,
    Int                    region,
    CSL_BitMask32          mppa
)
{   
    /* Determine which region is being configured.*/
    if (region < 0) 
    {
        /* Set the Global Memory Protection Attributes */
        hModule->regs->TPCC_MPPAG = mppa;
    }
    else 
    {
        /* Set the Memory Protection Attributes for the specific region. */
        hModule->regs->TPCC_MPPA[region] = mppa;
    }
    return;
}    

/** ============================================================================
 *   @n@b CSL_edma3IsDMAChannelEventPending
 *
 *   @b Description
 *   @n The function gets the status of the specified DMA channel i.e. if
 *      there is a pending event on the specific channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        dmaChannel      DMA Channel for which status is being inquired.
        response        Place holder for whether an event is set(TRUE) or not (FALSE)
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None. 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_ER_E0;TPCC_TPCC_ER_E1;TPCC_TPCC_ER_E2;TPCC_TPCC_ER_E3;
 *      TPCC_TPCC_ER_E4;TPCC_TPCC_ER_E5;TPCC_TPCC_ER_E6;TPCC_TPCC_ER_E7;
 *      TPCC_TPCC_ER_E8;TPCC_TPCC_ER_E9;TPCC_TPCC_ER_E10;TPCC_TPCC_ER_E11;
 *      TPCC_TPCC_ER_E12;TPCC_TPCC_ER_E13;TPCC_TPCC_ER_E14;TPCC_TPCC_ER_E15;
 *      TPCC_TPCC_ER_E16;TPCC_TPCC_ER_E17;TPCC_TPCC_ER_E18;TPCC_TPCC_ER_E19;
 *      TPCC_TPCC_ER_E20;TPCC_TPCC_ER_E21;TPCC_TPCC_ER_E22;TPCC_TPCC_ER_E23;
 *      TPCC_TPCC_ER_E24;TPCC_TPCC_ER_E25;TPCC_TPCC_ER_E26;TPCC_TPCC_ER_E27;
 *      TPCC_TPCC_ER_E28;TPCC_TPCC_ER_E29;TPCC_TPCC_ER_E30;TPCC_TPCC_ER_E31;
 *   @n TPCC_TPCC_ERH_E32;TPCC_TPCC_ERH_E33;TPCC_TPCC_ERH_E34;TPCC_TPCC_ERH_E35;
 *      TPCC_TPCC_ERH_E36;TPCC_TPCC_ERH_E37;TPCC_TPCC_ERH_E38;TPCC_TPCC_ERH_E39;
 *      TPCC_TPCC_ERH_E40;TPCC_TPCC_ERH_E41;TPCC_TPCC_ERH_E42;TPCC_TPCC_ERH_E43;
 *      TPCC_TPCC_ERH_E44;TPCC_TPCC_ERH_E45;TPCC_TPCC_ERH_E46;TPCC_TPCC_ERH_E47;
 *      TPCC_TPCC_ERH_E48;TPCC_TPCC_ERH_E49;TPCC_TPCC_ERH_E50;TPCC_TPCC_ERH_E51;
 *      TPCC_TPCC_ERH_E52;TPCC_TPCC_ERH_E53;TPCC_TPCC_ERH_E54;TPCC_TPCC_ERH_E55;
 *      TPCC_TPCC_ERH_E56;TPCC_TPCC_ERH_E57;TPCC_TPCC_ERH_E58;TPCC_TPCC_ERH_E59;
 *      TPCC_TPCC_ERH_E60;TPCC_TPCC_ERH_E61;TPCC_TPCC_ERH_E62;TPCC_TPCC_ERH_E63;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    dmaStatus;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Determine if there is an event pending on DMA Channel 0.
            CSL_edma3IsDMAChannelEventPending(hModule, 0, &dmaStatus);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3IsDMAChannelEventPending
(
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel,
    Bool*           response
)
{
    /* Determine which register needs to be looked into. */
    if (dmaChannel < 32) 
    {
        /* ER: Read the specific DMA Channel bits */
        if (CSL_FEXTR(hModule->regs->TPCC_ER, dmaChannel, dmaChannel))
            *response = TRUE;
        else
            *response = FALSE;
    }
    else 
    {
        /* ERH: Read the specific DMA Channel bits */
        if (CSL_FEXTR(hModule->regs->TPCC_ERH, dmaChannel - 32, dmaChannel - 32))
            *response = TRUE;
        else
            *response = FALSE;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearDMAChannelEvent
 *
 *   @b Description
 *   @n This API clears the event for the specific DMA channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region (Shadow or Global)
        dmaChannel      DMA Channel for which the event is cleared. 
    @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_ECR_E0=1;TPCC_TPCC_ECR_E1=1;TPCC_TPCC_ECR_E2=1;TPCC_TPCC_ECR_E3=1;
 *      TPCC_TPCC_ECR_E4=1;TPCC_TPCC_ECR_E5=1;TPCC_TPCC_ECR_E6=1;TPCC_TPCC_ECR_E7=1;
 *      TPCC_TPCC_ECR_E8=1;TPCC_TPCC_ECR_E9=1;TPCC_TPCC_ECR_E10=1;TPCC_TPCC_ECR_E11=1;
 *      TPCC_TPCC_ECR_E12=1;TPCC_TPCC_ECR_E13=1;TPCC_TPCC_ECR_E14=1;TPCC_TPCC_ECR_E15=1;
 *      TPCC_TPCC_ECR_E16=1;TPCC_TPCC_ECR_E17=1;TPCC_TPCC_ECR_E18=1;TPCC_TPCC_ECR_E19=1;
 *      TPCC_TPCC_ECR_E20=1;TPCC_TPCC_ECR_E21=1;TPCC_TPCC_ECR_E22=1;TPCC_TPCC_ECR_E23=1;
 *      TPCC_TPCC_ECR_E24=1;TPCC_TPCC_ECR_E25=1;TPCC_TPCC_ECR_E26=1;TPCC_TPCC_ECR_E27=1;
 *      TPCC_TPCC_ECR_E28=1;TPCC_TPCC_ECR_E29=1;TPCC_TPCC_ECR_E30=1;TPCC_TPCC_ECR_E31=1;
 *   @n TPCC_TPCC_ECRH_E32=1;TPCC_TPCC_ECRH_E33=1;TPCC_TPCC_ECRH_E34=1;TPCC_TPCC_ECRH_E35=1;
 *      TPCC_TPCC_ECRH_E36=1;TPCC_TPCC_ECRH_E37=1;TPCC_TPCC_ECRH_E38=1;TPCC_TPCC_ECRH_E39=1;
 *      TPCC_TPCC_ECRH_E40=1;TPCC_TPCC_ECRH_E41=1;TPCC_TPCC_ECRH_E42=1;TPCC_TPCC_ECRH_E43=1;
 *      TPCC_TPCC_ECRH_E44=1;TPCC_TPCC_ECRH_E45=1;TPCC_TPCC_ECRH_E46=1;TPCC_TPCC_ECRH_E47=1;
 *      TPCC_TPCC_ECRH_E48=1;TPCC_TPCC_ECRH_E49=1;TPCC_TPCC_ECRH_E50=1;TPCC_TPCC_ECRH_E51=1;
 *      TPCC_TPCC_ECRH_E52=1;TPCC_TPCC_ECRH_E53=1;TPCC_TPCC_ECRH_E54=1;TPCC_TPCC_ECRH_E55=1;
 *      TPCC_TPCC_ECRH_E56=1;TPCC_TPCC_ECRH_E57=1;TPCC_TPCC_ECRH_E58=1;TPCC_TPCC_ECRH_E59=1;
 *      TPCC_TPCC_ECRH_E60=1;TPCC_TPCC_ECRH_E61=1;TPCC_TPCC_ECRH_E62=1;TPCC_TPCC_ECRH_E63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    dmaStatus;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get DMA Channel 0 Status 
            CSL_edma3GetDMAChannelEvent(hModule, 0, &dmaStatus);
            if (dmaStatus == TRUE)
            {
                // DMA Channel 0 is active... 
                ...
                // Clear DMA Channel 0.
                CSL_edma3ClearDMAChannelEvent (hModule, CSL_EDMA3_REGION_GLOBAL, 0);
            }
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearDMAChannelEvent 
(
    CSL_Edma3Handle hModule,
    Int             region,
    Uint8           dmaChannel
)
{
    /* Determine the region for which the event is to be cleared. */
    if (region == CSL_EDMA3_REGION_GLOBAL)
    {
        /* Global: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* ECR: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->TPCC_ECR, dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* ECRH: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->TPCC_ECRH, dmaChannel - 32, dmaChannel - 32, 1);
        }
    }
    else
    {
        /* Shadow: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* ECR: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->SHADOW[region].TPCC_ECR, dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* ECRH: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->SHADOW[region].TPCC_ECRH, dmaChannel - 32, dmaChannel - 32, 1);
        }            
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3SetDMAChannelEvent
 *
 *   @b Description
 *   @n This API sets the event for the specific DMA channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region (Shadow or Global)
        dmaChannel      DMA Channel for which the event is to be set
    @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.   
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_ESR_E0=1;TPCC_TPCC_ESR_E1=1;TPCC_TPCC_ESR_E2=1;TPCC_TPCC_ESR_E3=1;
 *      TPCC_TPCC_ESR_E4=1;TPCC_TPCC_ESR_E5=1;TPCC_TPCC_ESR_E6=1;TPCC_TPCC_ESR_E7=1;
 *      TPCC_TPCC_ESR_E8=1;TPCC_TPCC_ESR_E9=1;TPCC_TPCC_ESR_E10=1;TPCC_TPCC_ESR_E11=1;
 *      TPCC_TPCC_ESR_E12=1;TPCC_TPCC_ESR_E13=1;TPCC_TPCC_ESR_E14=1;TPCC_TPCC_ESR_E15=1;
 *      TPCC_TPCC_ESR_E16=1;TPCC_TPCC_ESR_E17=1;TPCC_TPCC_ESR_E18=1;TPCC_TPCC_ESR_E19=1;
 *      TPCC_TPCC_ESR_E20=1;TPCC_TPCC_ESR_E21=1;TPCC_TPCC_ESR_E22=1;TPCC_TPCC_ESR_E23=1;
 *      TPCC_TPCC_ESR_E24=1;TPCC_TPCC_ESR_E25=1;TPCC_TPCC_ESR_E26=1;TPCC_TPCC_ESR_E27=1;
 *      TPCC_TPCC_ESR_E28=1;TPCC_TPCC_ESR_E29=1;TPCC_TPCC_ESR_E30=1;TPCC_TPCC_ESR_E31=1;
 *   @n TPCC_TPCC_ESRH_E32=1;TPCC_TPCC_ESRH_E33=1;TPCC_TPCC_ESRH_E34=1;TPCC_TPCC_ESRH_E35=1;
 *      TPCC_TPCC_ESRH_E36=1;TPCC_TPCC_ESRH_E37=1;TPCC_TPCC_ESRH_E38=1;TPCC_TPCC_ESRH_E39=1;
 *      TPCC_TPCC_ESRH_E40=1;TPCC_TPCC_ESRH_E41=1;TPCC_TPCC_ESRH_E42=1;TPCC_TPCC_ESRH_E43=1;
 *      TPCC_TPCC_ESRH_E44=1;TPCC_TPCC_ESRH_E45=1;TPCC_TPCC_ESRH_E46=1;TPCC_TPCC_ESRH_E47=1;
 *      TPCC_TPCC_ESRH_E48=1;TPCC_TPCC_ESRH_E49=1;TPCC_TPCC_ESRH_E50=1;TPCC_TPCC_ESRH_E51=1;
 *      TPCC_TPCC_ESRH_E52=1;TPCC_TPCC_ESRH_E53=1;TPCC_TPCC_ESRH_E54=1;TPCC_TPCC_ESRH_E55=1;
 *      TPCC_TPCC_ESRH_E56=1;TPCC_TPCC_ESRH_E57=1;TPCC_TPCC_ESRH_E58=1;TPCC_TPCC_ESRH_E59=1;
 *      TPCC_TPCC_ESRH_E60=1;TPCC_TPCC_ESRH_E61=1;TPCC_TPCC_ESRH_E62=1;TPCC_TPCC_ESRH_E63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Set the DMA Channel 0 Event for the Global Region. 
            CSL_edma3SetDMAChannelEvent(hModule, CSL_EDMA3_REGION_GLOBAL, 0);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3SetDMAChannelEvent 
(
    CSL_Edma3Handle hModule,
    Int             region,
    Uint8           dmaChannel
)
{
    /* Determine the region for which the event is to be set. */
    if (region == CSL_EDMA3_REGION_GLOBAL)
    {
        /* Global: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* ESR: Write to the specific DMA Channel bits */
            hModule->regs->TPCC_ESR = CSL_FMKR(dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* ESRH: Write to the specific DMA Channel bits */
            hModule->regs->TPCC_ESRH = CSL_FMKR(dmaChannel - 32, dmaChannel - 32, 1);
        }
    }
    else
    {
        /* Shadow: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* ESR: Write to the specific DMA Channel bits */
            hModule->regs->SHADOW[region].TPCC_ESR = CSL_FMKR(dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* ESRH: Write to the specific DMA Channel bits */
            hModule->regs->SHADOW[region].TPCC_ESRH = CSL_FMKR(dmaChannel - 32, dmaChannel - 32, 1);
        }
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3DMAChannelDisable
 *
 *   @b Description
 *   @n This API disables the specified DMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region (Shadow or Global)
        dmaChannel      DMA Channel to be disabled. 
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_EECR_E0=1;TPCC_TPCC_EECR_E1=1;TPCC_TPCC_EECR_E2=1;
 *      TPCC_TPCC_EECR_E3=1;TPCC_TPCC_EECR_E4=1;TPCC_TPCC_EECR_E5=1;
 *      TPCC_TPCC_EECR_E6=1;TPCC_TPCC_EECR_E7=1;TPCC_TPCC_EECR_E8=1;
 *      TPCC_TPCC_EECR_E9=1;TPCC_TPCC_EECR_E10=1;TPCC_TPCC_EECR_E11=1;
 *      TPCC_TPCC_EECR_E12=1;TPCC_TPCC_EECR_E13=1;TPCC_TPCC_EECR_E14=1;
 *      TPCC_TPCC_EECR_E15=1;TPCC_TPCC_EECR_E16=1;TPCC_TPCC_EECR_E17=1;
 *      TPCC_TPCC_EECR_E18=1;TPCC_TPCC_EECR_E19=1;TPCC_TPCC_EECR_E20=1;
 *      TPCC_TPCC_EECR_E21=1;TPCC_TPCC_EECR_E22=1;TPCC_TPCC_EECR_E23=1;
 *      TPCC_TPCC_EECR_E24=1;TPCC_TPCC_EECR_E25=1;TPCC_TPCC_EECR_E26=1;
 *      TPCC_TPCC_EECR_E27=1;TPCC_TPCC_EECR_E28=1;TPCC_TPCC_EECR_E29=1;
 *      TPCC_TPCC_EECR_E30=1;TPCC_TPCC_EECR_E31=1;
 *   @n TPCC_TPCC_EECRH_E32=1;TPCC_TPCC_EECRH_E33=1;TPCC_TPCC_EECRH_E34=1;
 *      TPCC_TPCC_EECRH_E35=1;TPCC_TPCC_EECRH_E36=1;TPCC_TPCC_EECRH_E37=1;
 *      TPCC_TPCC_EECRH_E38=1;TPCC_TPCC_EECRH_E39=1;TPCC_TPCC_EECRH_E40=1;
 *      TPCC_TPCC_EECRH_E41=1;TPCC_TPCC_EECRH_E42=1;TPCC_TPCC_EECRH_E43=1;
 *      TPCC_TPCC_EECRH_E44=1;TPCC_TPCC_EECRH_E45=1;TPCC_TPCC_EECRH_E46=1;
 *      TPCC_TPCC_EECRH_E47=1;TPCC_TPCC_EECRH_E48=1;TPCC_TPCC_EECRH_E49=1;
 *      TPCC_TPCC_EECRH_E50=1;TPCC_TPCC_EECRH_E51=1;TPCC_TPCC_EECRH_E52=1;
 *      TPCC_TPCC_EECRH_E53=1;TPCC_TPCC_EECRH_E54=1;TPCC_TPCC_EECRH_E55=1;
 *      TPCC_TPCC_EECRH_E56=1;TPCC_TPCC_EECRH_E57=1;TPCC_TPCC_EECRH_E58=1;
 *      TPCC_TPCC_EECRH_E59=1;TPCC_TPCC_EECRH_E60=1;TPCC_TPCC_EECRH_E61=1;
 *      TPCC_TPCC_EECRH_E62=1;TPCC_TPCC_EECRH_E63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
          
            // Disables DMA Channel 0
            CSL_edma3DMAChannelDisable(hModule, 0);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3DMAChannelDisable
(
    CSL_Edma3Handle hModule,
    Int             region,
    Uint8           dmaChannel
)
{
    /* Determine the region for which the DMA channel is to be disabled. */
    if (region == CSL_EDMA3_REGION_GLOBAL)
    {
        /* Global: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* EECR: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->TPCC_EECR, dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* EECRH: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->TPCC_EECRH, dmaChannel - 32, dmaChannel - 32, 1);
        }
    }
    else
    {
        /* Shadow: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* EECR: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->SHADOW[region].TPCC_EECR, dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* EECRH: Write to the specific DMA Channel bits */
            CSL_FINSR(hModule->regs->SHADOW[region].TPCC_EECRH, dmaChannel - 32, dmaChannel - 32, 1);
        }            
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3DMAChannelEnable
 *
 *   @b Description
 *   @n This API enables the specified DMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region (Shadow or Global)
        dmaChannel      DMA Channel to be enabled. 
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_EESR_E0=1;TPCC_TPCC_EESR_E1=1;TPCC_TPCC_EESR_E2=1;
 *      TPCC_TPCC_EESR_E3=1;TPCC_TPCC_EESR_E4=1;TPCC_TPCC_EESR_E5=1;
 *      TPCC_TPCC_EESR_E6=1;TPCC_TPCC_EESR_E7=1;TPCC_TPCC_EESR_E8=1;
 *      TPCC_TPCC_EESR_E9=1;TPCC_TPCC_EESR_E10=1;TPCC_TPCC_EESR_E11=1;
 *      TPCC_TPCC_EESR_E12=1;TPCC_TPCC_EESR_E13=1;TPCC_TPCC_EESR_E14=1;
 *      TPCC_TPCC_EESR_E15=1;TPCC_TPCC_EESR_E16=1;TPCC_TPCC_EESR_E17=1;
 *      TPCC_TPCC_EESR_E18=1;TPCC_TPCC_EESR_E19=1;TPCC_TPCC_EESR_E20=1;
 *      TPCC_TPCC_EESR_E21=1;TPCC_TPCC_EESR_E22=1;TPCC_TPCC_EESR_E23=1;
 *      TPCC_TPCC_EESR_E24=1;TPCC_TPCC_EESR_E25=1;TPCC_TPCC_EESR_E26=1;
 *      TPCC_TPCC_EESR_E27=1;TPCC_TPCC_EESR_E28=1;TPCC_TPCC_EESR_E29=1;
 *      TPCC_TPCC_EESR_E30=1;TPCC_TPCC_EESR_E31=1;
 *   @n TPCC_TPCC_EESRH_E32=1;TPCC_TPCC_EESRH_E33=1;TPCC_TPCC_EESRH_E34=1;
 *      TPCC_TPCC_EESRH_E35=1;TPCC_TPCC_EESRH_E36=1;TPCC_TPCC_EESRH_E37=1;
 *      TPCC_TPCC_EESRH_E38=1;TPCC_TPCC_EESRH_E39=1;TPCC_TPCC_EESRH_E40=1;
 *      TPCC_TPCC_EESRH_E41=1;TPCC_TPCC_EESRH_E42=1;TPCC_TPCC_EESRH_E43=1;
 *      TPCC_TPCC_EESRH_E44=1;TPCC_TPCC_EESRH_E45=1;TPCC_TPCC_EESRH_E46=1;
 *      TPCC_TPCC_EESRH_E47=1;TPCC_TPCC_EESRH_E48=1;TPCC_TPCC_EESRH_E49=1;
 *      TPCC_TPCC_EESRH_E50=1;TPCC_TPCC_EESRH_E51=1;TPCC_TPCC_EESRH_E52=1;
 *      TPCC_TPCC_EESRH_E53=1;TPCC_TPCC_EESRH_E54=1;TPCC_TPCC_EESRH_E55=1;
 *      TPCC_TPCC_EESRH_E56=1;TPCC_TPCC_EESRH_E57=1;TPCC_TPCC_EESRH_E58=1;
 *      TPCC_TPCC_EESRH_E59=1;TPCC_TPCC_EESRH_E60=1;TPCC_TPCC_EESRH_E61=1;
 *      TPCC_TPCC_EESRH_E62=1;TPCC_TPCC_EESRH_E63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
          
            // Enables DMA Channel 0 for the global region.
            CSL_edma3DMAChannelEnable(hModule, CSL_EDMA3_REGION_GLOBAL, 0);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3DMAChannelEnable
(    
    CSL_Edma3Handle hModule,
    Int             region,
    Uint8           dmaChannel
)
{
    /* Determine the region for which the DMA channel is to be enabled. */
    if (region == CSL_EDMA3_REGION_GLOBAL)
    {
        /* Global: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* EESR: Write to the specific DMA Channel bits */
            hModule->regs->TPCC_EESR = CSL_FMKR(dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* EESRH: Write to the specific DMA Channel bits */
            hModule->regs->TPCC_EESRH = CSL_FMKR(dmaChannel - 32, dmaChannel - 32, 1);
        }
    }
    else
    {
        /* Shadow: Determine which register needs to be looked into. */
        if (dmaChannel < 32) 
        {
            /* EESR: Write to the specific DMA Channel bits */
            hModule->regs->SHADOW[region].TPCC_EESR = CSL_FMKR(dmaChannel, dmaChannel, 1);
        }
        else 
        {
            /* EESRH: Write to the specific DMA Channel bits */
            hModule->regs->SHADOW[region].TPCC_EESRH = CSL_FMKR(dmaChannel - 32, dmaChannel - 32, 1);
        }
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetDMASecondaryEvents
 *
 *   @b Description
 *   @n This API gets the DMA secondary events 
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        secEventLo      Lower order 32 bits of secondary events populated by the API
        secEventHi      Higher order 32 bits of secondary events populated by the API
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_SER,TPCC_TPCC_SERH
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           secEventLo;
            CSL_BitMask32           secEventHi;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);

            // Get the DMA Secondary Events.
            CSL_edma3GetDMASecondaryEvents(hModule, &secEventLo, &secEventHi);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3GetDMASecondaryEvents
(    
    CSL_Edma3Handle hModule,
    CSL_BitMask32*  secEventLo,
    CSL_BitMask32*  secEventHi
)
{
    /* Read the Secondary Events */
    *secEventLo = hModule->regs->TPCC_SER;
    *secEventHi = hModule->regs->TPCC_SERH;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3IsDMAChannelSecondaryEventSet
 *
 *   @b Description
 *   @n This API is used to determine if the secondary Event for a specific DMA
 *      channel is set or not?
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        dmaChannel      DMA Channel for which secondary Events are being checked
        response        Status of the check populated by the API (TRUE if event
                        is missed else FALSE)
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.   
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_SER_SER0;TPCC_TPCC_SER_SER1;TPCC_TPCC_SER_SER2;
 *      TPCC_TPCC_SER_SER3;TPCC_TPCC_SER_SER4;TPCC_TPCC_SER_SER5;
 *      TPCC_TPCC_SER_SER6;TPCC_TPCC_SER_SER7;TPCC_TPCC_SER_SER8;
 *      TPCC_TPCC_SER_SER9;TPCC_TPCC_SER_SER10;TPCC_TPCC_SER_SER11;
 *      TPCC_TPCC_SER_SER12;TPCC_TPCC_SER_SER13;TPCC_TPCC_SER_SER14;
 *      TPCC_TPCC_SER_SER15;TPCC_TPCC_SER_SER16;TPCC_TPCC_SER_SER17;
 *      TPCC_TPCC_SER_SER18;TPCC_TPCC_SER_SER19;TPCC_TPCC_SER_SER20;
 *      TPCC_TPCC_SER_SER21;TPCC_TPCC_SER_SER22;TPCC_TPCC_SER_SER23;
 *      TPCC_TPCC_SER_SER24;TPCC_TPCC_SER_SER25;TPCC_TPCC_SER_SER26;
 *      TPCC_TPCC_SER_SER27;TPCC_TPCC_SER_SER28;TPCC_TPCC_SER_SER29;
 *      TPCC_TPCC_SER_SER30;TPCC_TPCC_SER_SER31;
 *   @n TPCC_TPCC_SERH_SER32;TPCC_TPCC_SERH_SER33;TPCC_TPCC_SERH_SER34;
 *      TPCC_TPCC_SERH_SER35;TPCC_TPCC_SERH_SER36;TPCC_TPCC_SERH_SER37;
 *      TPCC_TPCC_SERH_SER38;TPCC_TPCC_SERH_SER39;TPCC_TPCC_SERH_SER40;
 *      TPCC_TPCC_SERH_SER41;TPCC_TPCC_SERH_SER42;TPCC_TPCC_SERH_SER43;
 *      TPCC_TPCC_SERH_SER44;TPCC_TPCC_SERH_SER45;TPCC_TPCC_SERH_SER46;
 *      TPCC_TPCC_SERH_SER47;TPCC_TPCC_SERH_SER48;TPCC_TPCC_SERH_SER49;
 *      TPCC_TPCC_SERH_SER50;TPCC_TPCC_SERH_SER51;TPCC_TPCC_SERH_SER52;
 *      TPCC_TPCC_SERH_SER53;TPCC_TPCC_SERH_SER54;TPCC_TPCC_SERH_SER55;
 *      TPCC_TPCC_SERH_SER56;TPCC_TPCC_SERH_SER57;TPCC_TPCC_SERH_SER58;
 *      TPCC_TPCC_SERH_SER59;TPCC_TPCC_SERH_SER60;TPCC_TPCC_SERH_SER61;
 *      TPCC_TPCC_SERH_SER62;TPCC_TPCC_SERH_SER63;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    response;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);

            // Check if the DMA Channel 1 Secondary Event is set or not? 
            CSL_edma3IsDMAChannelSecondaryEventSet(hModule, 1, &response);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3IsDMAChannelSecondaryEventSet
(    
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel,
    Bool*           response
)
{
    /* Determine which register needs to be looked into. */
    if (dmaChannel < 32) 
    {
        /* EMR: Extract the appropriate bit. */
        if (CSL_FEXTR(hModule->regs->TPCC_SER, dmaChannel, dmaChannel))
            *response = TRUE;
        else
            *response = FALSE;
    } 
    else 
    {
        /* EMRH: Extract the appropriate bit. */
        if (CSL_FEXTR(hModule->regs->TPCC_SERH, dmaChannel-32, dmaChannel-32))
            *response = TRUE;
        else
            *response = FALSE;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearDMASecondaryEvents
 *
 *   @b Description
 *   @n This API clears the DMA secondary events 
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        secEventLo      Lower order 32 bits of secondary events to be cleared
        secEventHi      Higher order 32 bits of secondary events to be cleared
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_SECR,TPCC_TPCC_SECRH
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           secEventLo;
            CSL_BitMask32           secEventHi;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);

            // Get the DMA Secondary Events.
            CSL_edma3GetDMASecondaryEvents(hModule, &secEventLo, &secEventHi);            
            ...
            // Clear the DMA Secondary Events
            CSL_edma3ClearDMASecondaryEvents(hModule, secEventLo, secEventHi);
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearDMASecondaryEvents
(    
    CSL_Edma3Handle hModule,
    CSL_BitMask32   secEventLo,
    CSL_BitMask32   secEventHi
)
{
    /* Clear the Secondary Events */
    hModule->regs->TPCC_SECR  = secEventLo;
    hModule->regs->TPCC_SECRH = secEventHi;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearDMAChannelSecondaryEvents
 *
 *   @b Description
 *   @n This API clears the DMA Secondary Event for a specific DMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaChannel     DMA Channel for which the secondary event is to be cleared.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_SECR_SECR0=1;TPCC_TPCC_SECR_SECR1=1;TPCC_TPCC_SECR_SECR2=1;
 *      TPCC_TPCC_SECR_SECR3=1;TPCC_TPCC_SECR_SECR4=1;TPCC_TPCC_SECR_SECR5=1;
 *      TPCC_TPCC_SECR_SECR6=1;TPCC_TPCC_SECR_SECR7=1;TPCC_TPCC_SECR_SECR8=1;
 *      TPCC_TPCC_SECR_SECR9=1;TPCC_TPCC_SECR_SECR10=1;TPCC_TPCC_SECR_SECR11=1;
 *      TPCC_TPCC_SECR_SECR12=1;TPCC_TPCC_SECR_SECR13=1;TPCC_TPCC_SECR_SECR14=1;
 *      TPCC_TPCC_SECR_SECR15=1;TPCC_TPCC_SECR_SECR16=1;TPCC_TPCC_SECR_SECR17=1;
 *      TPCC_TPCC_SECR_SECR18=1;TPCC_TPCC_SECR_SECR19=1;TPCC_TPCC_SECR_SECR20=1;
 *      TPCC_TPCC_SECR_SECR21=1;TPCC_TPCC_SECR_SECR22=1;TPCC_TPCC_SECR_SECR23=1;
 *      TPCC_TPCC_SECR_SECR24=1;TPCC_TPCC_SECR_SECR25=1;TPCC_TPCC_SECR_SECR26=1;
 *      TPCC_TPCC_SECR_SECR27=1;TPCC_TPCC_SECR_SECR28=1;TPCC_TPCC_SECR_SECR29=1;
 *      TPCC_TPCC_SECR_SECR30=1;TPCC_TPCC_SECR_SECR31=1;
 *   @n TPCC_TPCC_SECRH_SECR32=1;TPCC_TPCC_SECRH_SECR33=1;TPCC_TPCC_SECRH_SECR34=1;
 *      TPCC_TPCC_SECRH_SECR35=1;TPCC_TPCC_SECRH_SECR36=1;TPCC_TPCC_SECRH_SECR37=1;
 *      TPCC_TPCC_SECRH_SECR38=1;TPCC_TPCC_SECRH_SECR39=1;TPCC_TPCC_SECRH_SECR40=1;
 *      TPCC_TPCC_SECRH_SECR41=1;TPCC_TPCC_SECRH_SECR42=1;TPCC_TPCC_SECRH_SECR43=1;
 *      TPCC_TPCC_SECRH_SECR44=1;TPCC_TPCC_SECRH_SECR45=1;TPCC_TPCC_SECRH_SECR46=1;
 *      TPCC_TPCC_SECRH_SECR47=1;TPCC_TPCC_SECRH_SECR48=1;TPCC_TPCC_SECRH_SECR49=1;
 *      TPCC_TPCC_SECRH_SECR50=1;TPCC_TPCC_SECRH_SECR51=1;TPCC_TPCC_SECRH_SECR52=1;
 *      TPCC_TPCC_SECRH_SECR53=1;TPCC_TPCC_SECRH_SECR54=1;TPCC_TPCC_SECRH_SECR55=1;
 *      TPCC_TPCC_SECRH_SECR56=1;TPCC_TPCC_SECRH_SECR57=1;TPCC_TPCC_SECRH_SECR58=1;
 *      TPCC_TPCC_SECRH_SECR59=1;TPCC_TPCC_SECRH_SECR60=1;TPCC_TPCC_SECRH_SECR61=1;
 *      TPCC_TPCC_SECRH_SECR62=1;TPCC_TPCC_SECRH_SECR63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            Uint32                  qdmaSecEvent;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Clear the DMA Secondary Event for DMA channel 1
            CSL_edma3ClearDMAChannelSecondaryEvents(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearDMAChannelSecondaryEvents
(    
    CSL_Edma3Handle hModule,
    Uint8           dmaChannel
)
{
    /* Determine which register needs to be looked into. */
    if (dmaChannel < 32)
    {
        /* SECR: Write to the appropriate channel. */
        hModule->regs->TPCC_SECR = CSL_FMKR (dmaChannel, dmaChannel, 1);
    }
    else
    {
        /* SECRH: Write to the appropriate channel. */
        dmaChannel = dmaChannel - 32;
        hModule->regs->TPCC_SECRH = CSL_FMKR (dmaChannel, dmaChannel, 1);
    }
}

/** ============================================================================
 *   @n@b CSL_edma3InterruptLoDisable
 *
 *   @b Description
 *   @n The API disables the specified low interrupt Number.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadow or Global)
        intrLo            Interrupt 0-31 (BitMask32) to be disabled
    @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_IECR_IECR0=1;TPCC_TPCC_IECR_IECR1=1;TPCC_TPCC_IECR_IECR2=1;
 *      TPCC_TPCC_IECR_IECR3=1;TPCC_TPCC_IECR_IECR4=1;TPCC_TPCC_IECR_IECR5=1;
 *      TPCC_TPCC_IECR_IECR6=1;TPCC_TPCC_IECR_IECR7=1;TPCC_TPCC_IECR_IECR8=1;
 *      TPCC_TPCC_IECR_IECR9=1;TPCC_TPCC_IECR_IECR10=1;TPCC_TPCC_IECR_IECR11=1;
 *      TPCC_TPCC_IECR_IECR12=1;TPCC_TPCC_IECR_IECR13=1;TPCC_TPCC_IECR_IECR14=1;
 *      TPCC_TPCC_IECR_IECR15=1;TPCC_TPCC_IECR_IECR16=1;TPCC_TPCC_IECR_IECR17=1;
 *      TPCC_TPCC_IECR_IECR18=1;TPCC_TPCC_IECR_IECR19=1;TPCC_TPCC_IECR_IECR20=1;
 *      TPCC_TPCC_IECR_IECR21=1;TPCC_TPCC_IECR_IECR22=1;TPCC_TPCC_IECR_IECR23=1;
 *      TPCC_TPCC_IECR_IECR24=1;TPCC_TPCC_IECR_IECR25=1;TPCC_TPCC_IECR_IECR26=1;
 *      TPCC_TPCC_IECR_IECR27=1;TPCC_TPCC_IECR_IECR28=1;TPCC_TPCC_IECR_IECR29=1;
 *      TPCC_TPCC_IECR_IECR30=1;TPCC_TPCC_IECR_IECR31=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
        
            // Interrupts 5-7 disabled for Global Region.
            CSL_edma3InterruptLoDisable(hModule, CSL_EDMA3_REGION_GLOBAL, 0x000000E0);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3InterruptLoDisable 
(
    CSL_Edma3Handle hModule,
    Int             region,
    CSL_BitMask32   intrLo
)
{
    /* Disable the interrupts depending on the region. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {   
        /* Shadow Region */
        hModule->regs->SHADOW[region].TPCC_IECR |= intrLo;
    } 
    else
    {
        /* Global Region */
        hModule->regs->TPCC_IECR |= intrLo;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3InterruptHiDisable
 *
 *   @b Description
 *   @n The API disables the specified high interrupt Number.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadow or Global)
        intrHi            Interrupt 32-63 (BitMask32) to be disabled
    @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_IECRH_IECR32=1;TPCC_TPCC_IECRH_IECR33=1;TPCC_TPCC_IECRH_IECR34=1;
 *      TPCC_TPCC_IECRH_IECR35=1;TPCC_TPCC_IECRH_IECR36=1;TPCC_TPCC_IECRH_IECR37=1;
 *      TPCC_TPCC_IECRH_IECR38=1;TPCC_TPCC_IECRH_IECR39=1;TPCC_TPCC_IECRH_IECR40=1;
 *      TPCC_TPCC_IECRH_IECR41=1;TPCC_TPCC_IECRH_IECR42=1;TPCC_TPCC_IECRH_IECR43=1;
 *      TPCC_TPCC_IECRH_IECR44=1;TPCC_TPCC_IECRH_IECR45=1;TPCC_TPCC_IECRH_IECR46=1;
 *      TPCC_TPCC_IECRH_IECR47=1;TPCC_TPCC_IECRH_IECR48=1;TPCC_TPCC_IECRH_IECR49=1;
 *      TPCC_TPCC_IECRH_IECR50=1;TPCC_TPCC_IECRH_IECR51=1;TPCC_TPCC_IECRH_IECR52=1;
 *      TPCC_TPCC_IECRH_IECR53=1;TPCC_TPCC_IECRH_IECR54=1;TPCC_TPCC_IECRH_IECR55=1;
 *      TPCC_TPCC_IECRH_IECR56=1;TPCC_TPCC_IECRH_IECR57=1;TPCC_TPCC_IECRH_IECR58=1;
 *      TPCC_TPCC_IECRH_IECR59=1;TPCC_TPCC_IECRH_IECR60=1;TPCC_TPCC_IECRH_IECR61=1;
 *      TPCC_TPCC_IECRH_IECR62=1;TPCC_TPCC_IECRH_IECR63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
        
            // Interrupts 32 disabled for Global Region.
            CSL_edma3InterruptHiDisable(hModule, CSL_EDMA3_REGION_GLOBAL, 0x1);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3InterruptHiDisable 
(
    CSL_Edma3Handle hModule,
    Int             region,
    CSL_BitMask32   intrHi
)
{
    /* Disable the interrupts depending on the region. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {   
        /* Shadow Region */            
        hModule->regs->SHADOW[region].TPCC_IECRH |= intrHi;
    } 
    else 
    {
        /* Global Region */
        hModule->regs->TPCC_IECRH |= intrHi;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3InterruptLoEnable
 *
 *   @b Description
 *   @n The API enables the specific lower interrupts
 * 
 *   @b Arguments
 *   @verbatim
        hModule          Module Handle
        region           Region (Shadow or Global) 
        intrLo           Interrupt 0-31 (BitMask32) to be enabled
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_IESR_IESR0=1;TPCC_TPCC_IESR_IESR1=1;TPCC_TPCC_IESR_IESR2=1;
 *      TPCC_TPCC_IESR_IESR3=1;TPCC_TPCC_IESR_IESR4=1;TPCC_TPCC_IESR_IESR5=1;
 *      TPCC_TPCC_IESR_IESR6=1;TPCC_TPCC_IESR_IESR7=1;TPCC_TPCC_IESR_IESR8=1;
 *      TPCC_TPCC_IESR_IESR9=1;TPCC_TPCC_IESR_IESR10=1;TPCC_TPCC_IESR_IESR11=1;
 *      TPCC_TPCC_IESR_IESR12=1;TPCC_TPCC_IESR_IESR13=1;TPCC_TPCC_IESR_IESR14=1;
 *      TPCC_TPCC_IESR_IESR15=1;TPCC_TPCC_IESR_IESR16=1;TPCC_TPCC_IESR_IESR17=1;
 *      TPCC_TPCC_IESR_IESR18=1;TPCC_TPCC_IESR_IESR19=1;TPCC_TPCC_IESR_IESR20=1;
 *      TPCC_TPCC_IESR_IESR21=1;TPCC_TPCC_IESR_IESR22=1;TPCC_TPCC_IESR_IESR23=1;
 *      TPCC_TPCC_IESR_IESR24=1;TPCC_TPCC_IESR_IESR25=1;TPCC_TPCC_IESR_IESR26=1;
 *      TPCC_TPCC_IESR_IESR27=1;TPCC_TPCC_IESR_IESR28=1;TPCC_TPCC_IESR_IESR29=1;
 *      TPCC_TPCC_IESR_IESR30=1;TPCC_TPCC_IESR_IESR31=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
        
            // Interrupts 5-7 enabled for the global region.
            CSL_edma3InterruptLoEnable(hModule,CSL_EDMA3_REGION_GLOBAL, 0x000000E0);
            ...
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void CSL_edma3InterruptLoEnable 
(
    CSL_Edma3Handle hModule,
    Int             region,
    CSL_BitMask32   intrLo
)
{
    /* Enable the interrupts depending on the region. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region */
        hModule->regs->SHADOW[region].TPCC_IESR |= intrLo;
    } 
    else 
    {
        /* Global Region */
        hModule->regs->TPCC_IESR |= intrLo;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3InterruptHiEnable
 *
 *   @b Description
 *   @n The API enables the specific High interrupt.
 * 
 *   @b Arguments
 *   @verbatim
        hModule          Module Handle
        region           Region (Shadow or Global) 
        intrHi           Interrupt 32-63 (BitMask32) to be enabled
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_IESRH_IESR32=1;TPCC_TPCC_IESRH_IESR33=1;TPCC_TPCC_IESRH_IESR34=1;
 *      TPCC_TPCC_IESRH_IESR35=1;TPCC_TPCC_IESRH_IESR36=1;TPCC_TPCC_IESRH_IESR37=1;
 *      TPCC_TPCC_IESRH_IESR38=1;TPCC_TPCC_IESRH_IESR39=1;TPCC_TPCC_IESRH_IESR40=1;
 *      TPCC_TPCC_IESRH_IESR41=1;TPCC_TPCC_IESRH_IESR42=1;TPCC_TPCC_IESRH_IESR43=1;
 *      TPCC_TPCC_IESRH_IESR44=1;TPCC_TPCC_IESRH_IESR45=1;TPCC_TPCC_IESRH_IESR46=1;
 *      TPCC_TPCC_IESRH_IESR47=1;TPCC_TPCC_IESRH_IESR48=1;TPCC_TPCC_IESRH_IESR49=1;
 *      TPCC_TPCC_IESRH_IESR50=1;TPCC_TPCC_IESRH_IESR51=1;TPCC_TPCC_IESRH_IESR52=1;
 *      TPCC_TPCC_IESRH_IESR53=1;TPCC_TPCC_IESRH_IESR54=1;TPCC_TPCC_IESRH_IESR55=1;
 *      TPCC_TPCC_IESRH_IESR56=1;TPCC_TPCC_IESRH_IESR57=1;TPCC_TPCC_IESRH_IESR58=1;
 *      TPCC_TPCC_IESRH_IESR59=1;TPCC_TPCC_IESRH_IESR60=1;TPCC_TPCC_IESRH_IESR61=1;
 *      TPCC_TPCC_IESRH_IESR62=1;TPCC_TPCC_IESRH_IESR63=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
        
            // Interrupts 32 enabled for the global region.
            CSL_edma3InterruptHiEnable(hModule,CSL_EDMA3_REGION_GLOBAL, 0x1);
            ...
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void CSL_edma3InterruptHiEnable 
(
    CSL_Edma3Handle hModule,
    Int             region,
    CSL_BitMask32   intrHi
)
{
    /* Enable the interrupts depending on the region. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region */
        hModule->regs->SHADOW[region].TPCC_IESRH |= intrHi;
    } 
    else 
    {
        /* Global Region */
        hModule->regs->TPCC_IESRH |= intrHi;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetLoPendingInterrupts
 *
 *   @b Description
 *   @n The API gets a bitmask of all low pending interrupts. 
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadown Region or Global)
        intrLo            Status 0-31 of the interrupts
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_IPR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           edmaIntrLo;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get all low pending interrupts for the global region.
            CSL_edma3GetLoPendingInterrupts(hModule, CSL_EDMA3_REGION_GLOBAL, &edmaIntrLo);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetLoPendingInterrupts
(
    CSL_Edma3Handle hModule,
    Int             region,
    CSL_BitMask32*  intrLo
)
{
    /* Get the pending interrupts depending on the region */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region. */
        *intrLo = hModule->regs->SHADOW[region].TPCC_IPR;
    } 
    else 
    {
        /* Global Region. */
        *intrLo = hModule->regs->TPCC_IPR;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetHiPendingInterrupts
 *
 *   @b Description
 *   @n The API gets a bitmask of all high pending interrupts. 
 *
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadown Region or Global)
        intrHi            Status 32-63 of the interrupts       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_IPRH
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           edmaIntrHi;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get all the high pending interrupts for the global region.
            CSL_edma3GetHiPendingInterrupts(hModule, CSL_EDMA3_REGION_GLOBAL, &edmaIntrHi);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3GetHiPendingInterrupts
(
    CSL_Edma3Handle hModule,
    Int             region,
    CSL_BitMask32*  intrHi
)
{
    /* Get the pending interrupts depending on the region */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region. */
        *intrHi = hModule->regs->SHADOW[region].TPCC_IPRH;
    } 
    else 
    {
        /* Global Region. */
        *intrHi = hModule->regs->TPCC_IPRH;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearLoPendingInterrupts
 *
 *   @b Description
 *   @n This API clears the low pending interrupts using the interrupt bitmasks 
 *      provided
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadown Region or Global)
        intrLo            Interrupt 0-31 (BitMask32) to be cleared        
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_ICR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           edmaIntrLo;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get all the pending interrupts for the global region. 
            CSL_edma3GetLoPendingInterrupts(hModule, CSL_EDMA3_REGION_GLOBAL, &edmaIntrLo);
            ...
            // Clear the pending interrupts for the global region.
            CSL_edma3ClearLoPendingInterrupts(hModule, CSL_EDMA3_REGION_GLOBAL, edmaIntrLo);
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void CSL_edma3ClearLoPendingInterrupts 
(
    CSL_Edma3Handle        hModule,
    Int                    region,
    CSL_BitMask32          intrLo
)
{
    /* Clear the pending interrupts depending on the region. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region */
        hModule->regs->SHADOW[region].TPCC_ICR  = intrLo;
    } 
    else 
    {
        /* Global Region */
        hModule->regs->TPCC_ICR  = intrLo;
    }   
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearHiPendingInterrupts
 *
 *   @b Description
 *   @n This API clears the High pending interrupts using the interrupt bitmasks 
 *      provided
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadown Region or Global)
        intrHi            Interrupt 32-63 (BitMask32) to be cleared
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_ICRH
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           edmaIntrHi;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get all the pending interrupts for the global region. 
            CSL_edma3GetHiPendingInterrupts(hModule, CSL_EDMA3_REGION_GLOBAL, &edmaIntrHi);
            ...
            // Clear the pending interrupts for the global region.
            CSL_edma3ClearHiPendingInterrupts(hModule, CSL_EDMA3_REGION_GLOBAL, edmaIntrHi);
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void CSL_edma3ClearHiPendingInterrupts 
(
    CSL_Edma3Handle        hModule,
    Int                    region,
    CSL_BitMask32          intrHi
)
{
    /* Clear the pending interrupts depending on the region. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region */
        hModule->regs->SHADOW[region].TPCC_ICRH = intrHi;
    } 
    else 
    {
        /* Global Region */
        hModule->regs->TPCC_ICRH = intrHi;
    }   
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3InterruptEval
 *
 *   @b Description
 *   @n The API is used to set the EVAL bit which will cause an interrupt to be 
 *      generated if any enabled interrupts are still pending.
 * 
 *   @b Arguments
 *   @verbatim
        hModule           Module Handle
        region            Region (Shadown Region or Global)
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_IEVAL_EVAL=1
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            CSL_BitMask32           edmaIntrLo;
            CSL_BitMask32           edmaIntrHi;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...        
            // Interrupt Evaluate for Global Region.
            status = CSL_edma3InterruptEval(hModule, CSL_EDMA3_REGION_GLOBAL);
            ...
     @endverbatim
 * ===========================================================================
 */    
CSL_IDEF_INLINE void  CSL_edma3InterruptEval 
(
    CSL_Edma3Handle hModule,
    Int             region
)
{
    /* Determine the region for which the interrupt evaluate needs to be done. */
    if (region != CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Shadow Region. */
        hModule->regs->SHADOW[region].TPCC_IEVAL = CSL_FMK(TPCC_TPCC_IEVAL_EVAL, 1);
    }
    else 
    {
        /* Global Region. */
        hModule->regs->TPCC_IEVAL = CSL_FMK(TPCC_TPCC_IEVAL_EVAL, 1);
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3IsQDMAChannelEventPending
 *
 *   @b Description
 *   @n The function gets the status of the specified QDMA channel i.e. if
 *      there is a pending event on the specific channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaChannel     QDMA Channel for which status is being inquired.
        response        Place holder for whether an event is set(TRUE) or not (FALSE)
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None. 
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QER_QER0;TPCC_TPCC_QER_QER1;TPCC_TPCC_QER_QER2;TPCC_TPCC_QER_QER3;
 *      TPCC_TPCC_QER_QER4;TPCC_TPCC_QER_QER5;TPCC_TPCC_QER_QER6;TPCC_TPCC_QER_QER7
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    qdmaStatus;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Is QDMA Channel 1 event pending.
            CSL_edma3IsQDMAChannelEventPending(hModule, 1, &qdmaStatus);
            ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_edma3IsQDMAChannelEventPending
(
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel,    
    Bool*           response
)
{
    /* Read the specific QDMA channel bits. */
    if (CSL_FEXTR(hModule->regs->TPCC_QER, qdmaChannel, qdmaChannel))
        *response = TRUE;
    else
        *response = FALSE;

    return;
}

/** ============================================================================
 *   @n@b CSL_edma3QDMAChannelEnable
 *
 *   @b Description
 *   @n This API enables the specified QDMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region (Shadown Region or Global)
        qdmaChannel     QDMA Channel to be enabled. 
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QEESR_QEESR0=1;TPCC_TPCC_QEESR_QEESR1=1;TPCC_TPCC_QEESR_QEESR2=1;
 *      TPCC_TPCC_QEESR_QEESR3=1;TPCC_TPCC_QEESR_QEESR4=1;TPCC_TPCC_QEESR_QEESR5=1;
 *      TPCC_TPCC_QEESR_QEESR6=1;TPCC_TPCC_QEESR_QEESR7=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
          
            // Enables QDMA Channel 1 for Global Region.
            CSL_edma3QDMAChannelEnable(hModule, CSL_EDMA3_REGION_GLOBAL, 1);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3QDMAChannelEnable
(    
    CSL_Edma3Handle hModule,
    Int             region,
    Uint8           qdmaChannel
)
{
    /* Determine the region for which the QDMA channel is to be enabled. */
    if (region == CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Global: Write to the specific QDMA Channel bits */
        hModule->regs->TPCC_QEESR = CSL_FMKR(qdmaChannel, qdmaChannel, 1);
    }
    else
    {
        /* Shadow: Write to the specific QDMA Channel bits. */
        hModule->regs->SHADOW[region].TPCC_QEESR = CSL_FMKR(qdmaChannel, qdmaChannel, 1);
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3QDMAChannelDisable
 *
 *   @b Description
 *   @n This API disables the specified QDMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        region          Region (Shadown Region or Global)
        qdmaChannel     QDMA Channel to be disabled. 
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QEECR_QEECR0=1;TPCC_TPCC_QEECR_QEECR1=1;TPCC_TPCC_QEECR_QEECR2=1;
 *      TPCC_TPCC_QEECR_QEECR3=1;TPCC_TPCC_QEECR_QEECR4=1;TPCC_TPCC_QEECR_QEECR5=1;
 *      TPCC_TPCC_QEECR_QEECR6=1;TPCC_TPCC_QEECR_QEECR7=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
          
            // Disables QDMA Channel 0
            CSL_edma3QDMAChannelDisable(hModule, 0);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3QDMAChannelDisable
(    
    CSL_Edma3Handle hModule,
    Int             region,
    Uint8           qdmaChannel
)
{
    /* Determine the region for which the QDMA channel is to be disabled. */
    if (region == CSL_EDMA3_REGION_GLOBAL) 
    {
        /* Global: Write to the specific QDMA Channel bits */
        CSL_FINSR(hModule->regs->TPCC_QEECR, qdmaChannel, qdmaChannel, 1);
    }
    else
    {
        /* Shadow: Write to the specific QDMA Channel bits. */
        CSL_FINSR(hModule->regs->SHADOW[region].TPCC_QEECR, qdmaChannel, qdmaChannel, 1);
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3GetQDMASecondaryEvents
 *
 *   @b Description
 *   @n This API reads the QDMA Secondary Event.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaSecEvent    QDMA Secondary Event which is populated by this API
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QSER
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            Uint32                  qdmaSecEvent;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get the QDMA Secondary Event
            CSL_edma3GetQDMASecondaryEvents(hModule, &qdmaSecEvent);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3GetQDMASecondaryEvents
(    
    CSL_Edma3Handle hModule,
    Uint32*         qdmaSecEvent
)
{
    /* Read the QDMA Secondary Event. */
    *qdmaSecEvent = hModule->regs->TPCC_QSER;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3IsDMAChannelSecondaryEventSet
 *
 *   @b Description
 *   @n This API is used to determine if the secondary Event for a specific DMA
 *      channel is set or not?
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaChannel     QDMA Channel for which secondary Events are being checked
        response        Status of the check populated by the API (TRUE if event
                        is missed else FALSE)
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TPCC_TPCC_QSER_QSER0;TPCC_TPCC_QSER_QSER1;TPCC_TPCC_QSER_QSER2;
 *      TPCC_TPCC_QSER_QSER3;TPCC_TPCC_QSER_QSER4;TPCC_TPCC_QSER_QSER5;
 *      TPCC_TPCC_QSER_QSER6;TPCC_TPCC_QSER_QSER7;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            CSL_Status              status;
            Bool                    response;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);

            // Check if the QDMA Channel 1 Secondary Event is set or not? 
            CSL_edma3IsQDMAChannelSecondaryEventSet(hModule, 1, &response);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3IsQDMAChannelSecondaryEventSet
(    
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel,
    Bool*           response
)
{
    /* Check if the QDMA channel bit is set or not? */
    if (CSL_FEXTR(hModule->regs->TPCC_QSER, qdmaChannel, qdmaChannel))
        *response = TRUE;
    else
        *response = FALSE;
    return;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearQDMASecondaryEvents
 *
 *   @b Description
 *   @n This API clears the QDMA Secondary Event.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaSecEvent    QDMA Secondary Event to be cleared.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QSECR
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            Uint32                  qdmaSecEvent;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Get the QDMA Secondary Event
            CSL_edma3GetQDMASecondaryEvents(hModule, &qdmaSecEvent);
            ...            
            // Clear the QDMA Secondary Event
            CSL_edma3ClearQDMASecondaryEvents(hModule, qdmaSecEvent);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearQDMASecondaryEvents
(    
    CSL_Edma3Handle hModule,
    Uint32          qdmaSecEvent
)
{
    /* Clears the QDMA Secondary Event. */
    hModule->regs->TPCC_QSECR = qdmaSecEvent;
}

/** ============================================================================
 *   @n@b CSL_edma3ClearQDMAChannelSecondaryEvents
 *
 *   @b Description
 *   @n This API clears the QDMA Secondary Event for a specific QDMA Channel.
 * 
 *   @b Arguments
 *   @verbatim
        hModule         Module Handle
        qdmaChannel     QDMA Channel for which the secondary event is to be cleared.
    @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_edma3Init() and @a CSL_edma3Open() must be called.
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n TPCC_TPCC_QSECR_QSECR0=1;TPCC_TPCC_QSECR_QSECR1=1;TPCC_TPCC_QSECR_QSECR2=1;
 *      TPCC_TPCC_QSECR_QSECR3=1;TPCC_TPCC_QSECR_QSECR4=1;TPCC_TPCC_QSECR_QSECR5=1;
 *      TPCC_TPCC_QSECR_QSECR6=1;TPCC_TPCC_QSECR_QSECR7=1;
 *
 *   @b Example
 *   @verbatim
            CSL_Edma3Handle         hModule;
            CSL_Edma3Obj            edmaObj;
            CSL_Edma3Context        context;
            Uint32                  qdmaSecEvent;

            // Module Initialization
            CSL_edma3Init(&context);
            
            // Module Level Open    
            hModule = CSL_edma3Open(&edmaObj,CSL_EDMA3,NULL,&status);
            ...
            // Clear the QDMA Secondary Event for QDMA channel 1
            CSL_edma3ClearQDMAChannelSecondaryEvents(hModule, 1);
            ...
     @endverbatim
 * ===========================================================================
 */   
CSL_IDEF_INLINE void CSL_edma3ClearQDMAChannelSecondaryEvents
(    
    CSL_Edma3Handle hModule,
    Uint8           qdmaChannel
)
{
    /* Clears the QDMA Secondary Event. */
    CSL_FINSR (hModule->regs->TPCC_QSECR, qdmaChannel, qdmaChannel, 1);
}

/**
@}
*/

#endif /* _CSL_EDMA3AUX_H_ */
