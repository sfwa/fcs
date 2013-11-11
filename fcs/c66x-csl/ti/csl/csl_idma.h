/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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
/** 
 *   @file  csl_idma.h
 *
 *   @brief  IDMA CSL Implementation on DSP side
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par 
 */

/** @defgroup CSL_IDMA_API IDMA
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 * The iDMA is a simple DMA engine that can be used to perform block transfers 
 * between any two memory locations local to the GEM. A "local" memory is simply
 * one whose controller is inside the GEM, which can be L1P, L1D, L2 (port A and
 * port B), or peripheral configuration port (CFG). The iDMA controller allows 
 * rapid data paging between all local memories. To fully support this, the iDMA
 * consists of two orthogonal channels caoable of working concurrently. The two
 * channels are:
 *      a) IDMA Channel 0:- Intended for quick programming of configuration 
 *         registers through external CFG port of GEM.
 *      b) IDMA Channel 1:- Intended for data paging between local memories.  
 *
 * @subsection References
 *   -# GEM Internal DMA (IDMA) Specification. 
 *
 * @subsection Assumptions
 *    The abbreviations IDMA, idma and Idma have been used throughout this
 *    document to refer to iDMA Controller.
 */
#ifndef CSL_IDMA_H_
#define CSL_IDMA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_cgem.h>

/**
@defgroup CSL_IDMA_SYMBOL  IDMA Symbols Defined
@ingroup CSL_IDMA_API
*/
/**
@defgroup CSL_IDMA_DATASTRUCT  IDMA Data Structures
@ingroup CSL_IDMA_API
*/
/**
@defgroup CSL_IDMA_FUNCTION  IDMA Functions
@ingroup CSL_IDMA_API
*/
/**
@defgroup CSL_IDMA_ENUM  IDMA Enumerated Data Types
@ingroup CSL_IDMA_API
*/

/**
@addtogroup CSL_IDMA_SYMBOL
@{
*/

#define ZERO    0
#define ONE     1

/**
@}
*/

/** @addtogroup CSL_IDMA_ENUM
 @{ */

/** Specifies what priority level the IDMA channel is set to.
 *  Used to specify what priority level the IDMA channel is set to.
 */
typedef enum {
    /** Set Priority level 0 */
    IDMA_PRI_0 = 0,
    /** Set Priority level 1 */
    IDMA_PRI_1,
    /** Set Priority level 2 */
    IDMA_PRI_2,
    /** Set Priority level 3 */
    IDMA_PRI_3,
    /** Set Priority level 4 */
    IDMA_PRI_4,
    /** Set Priority level 5 */
    IDMA_PRI_5,
    /** Set Priority level 6 */
    IDMA_PRI_6,
    /** Set Priority level 7 */
    IDMA_PRI_7,
    /** NoPriority level */
    IDMA_PRI_NULL = -1
}IDMA_priSet;

/**
@}
*/

/** @addtogroup CSL_IDMA_DATASTRUCT
 @{ */


/** @brief      This structure holds the information required
 *              to initiate a iDMA Channel 1 Block Fill/Transfer
 *              request in the GEM.
 */
typedef struct {

    /** @brief  IDMA channel 1 Source Address.
     * 
     *  @details
     *  The source address must point to a word-aligned 
     *  memory location local to GEM. When performing a 
     *  block fill, all 32 bits of the address specified 
     *  are considered; While, for block transfers, the 
     *  2 LSBs are ignored and the higher order 30 bits 
     *  are read as the valid Source Address for transfer.  
     */
    Uint32*     source;
    
    /** @brief  IDMA channel 1 Destination Address.
     * 
     *  @details
     *  The destination address must point to a 32 bit 
     *  word-aligned memory location local to GEM.
     *  This address must be local to GEM, either in L1P,
     *  L1D, L2 or CFG and also must be different port 
     *  than the source address to obtain full throughput.    
     */
    Uint32*     destn;
    
    /** @brief  Number of bytes to be transfered
     * 
     *  @details
     *  The count signifies the number of bytes to be 
     *  transferred using iDMA channel 1. This must be 
     *  a multiple of 4 bytes. A count of zero will not
     *  transfer any data, but will generate an interrupt
     *  if requested.  
     */
    Uint16      count;

    /** @brief  Transfer Priority.
     *
     *  @details
     *  The transfer priority is used for arbitration between
     *  the CPU and DMA accesses when there are conflicts.
     *  Valid values for the priority range between 0 and 7. 
     */
    Uint32      priority:3;

    /** @brief  Boolean Flag to enable/disable CPU interrupt.
     *
     *  @details
     *  When this interrupt flag is set, a CPU Interrupt IDMA_INT1
     *  is raised on completion of the block transfer/fill request. 
     */
    Uint32      intEnable:1;
 
}CSL_IDMA_IDMA1CONFIG;


/** @brief      This structure holds the information required
 *              to initiate a iDMA Channel 0 Configuration(CFG) space
 *              Transfer request from the GEM.
 */
typedef struct {
    /** @brief  IDMA channel 0 Mask.
     * 
     *  @details
     *  The mask allows unwanted registers within the window
     *  to be blocked from access, facilitating multiple read/write
     *  transactions to be completed with a single transfer command
     *  by the CPU. Each of the 32 bits of the mask correspond to 
     *  a single register in the CFG space identified by the source/
     *  destination address registers.
     */
    Uint32      mask;

    /** @brief  IDMA channel 0 Source Address.
     * 
     *  @details
     *  The source address must point to a 32-byte-aligned 
     *  memory location local to GEM or to a valid configuration
     *  register space.   
     */
    Uint32*      source;
    
    /** @brief  IDMA channel 0 Destination Address.
     * 
     *  @details
     *  The destination address must point to a 32-byte 
     *  -aligned memory location local to GEM or to a valid
     *  configuration register space.     
     */
    Uint32*      destn;
    
    /** @brief  Number of 32-word windows to be transfered
     * 
     *  @details
     *  The count signifies the number of windows to be accessed
     *  during data transfer. Upto 16 contiguous 32-word regions 
     *  can be specified using this field. 
     */
    Uint32      count:4;

    /** @brief  Boolean Flag to enable/disable CPU interrupt.
     *
     *  @details
     *  When this interrupt flag is set, a CPU Interrupt IDMA_INT0
     *  is raised on completion of the block transfer/fill request. 
     */
    Uint32      intEnable:1;
 
}CSL_IDMA_IDMA0CONFIG;

/** @brief      This structure holds the information required
 *              to interpret the IDMA Channel 0/1 Transfer
 *              Status.
 */
typedef struct {
    /** @brief  Boolean Flag that indicates if any pending transfers
     *          exist on the iDMA channel 1/channel 0.          
     *
     *  @details
     *  Set when control registers are written to by the CPU and there
     *  is already an active transfer in progress (ACTV=1) and cleared
     *  when the pending transfer becomes active.
     */
    Uint32      isPending:1;

    /** @brief  Boolean Flag that indicates if any active transfers
     *          exist on the iDMA channel 1/channel 0.          
     *
     *  @details
     *  Set when channel 0/1 begins reading data from the source address
     *  and cleared following the write to the destination address.
     */
    Uint32      isActive:1;
}CSL_IDMA_STATUS;

/**
 *  Handle to access IDMA registers.
 */
#define hIdma     ((CSL_CgemRegs *) CSL_CGEM0_5_REG_BASE_ADDRESS_REGS)

/* @} */


#ifdef __cplusplus
}
#endif

#endif /*CSL_IDMA_H_*/

