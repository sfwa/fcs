/**
 *   @file  csl_cache.h
 *
 *   @brief   
 *      This is the CACHE Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the CACHE Module.
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

/** @defgroup CSL_CACHE_API CACHE
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This page describes the Functions, Data Structures, Enumerations and Macros
 * within CACHE module.
 *
 * This module uses three cache architectures, Level 1 Program (L1P), Level 1
 * Data (L1D) and Level 2 CACHE architectures, The L1P and L1D can be
 * configured as 0K, 4K, 8K, 16K, or 32K CACHE size. The L2 can be configured
 * as 32KB, 64KB, 128KB, or 256KB CACHE size. This CACHE module supports the
 * Block and Global Coherence Operations.
 *
 * @subsection References
 *    -# TMS320TCI6484 Data Sheet SPRS438 (Dec 2007) 
 *
 */
 
#ifndef _CSL_CACHE_H_
#define _CSL_CACHE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/csl/cslr_cgem.h>
#include <ti/csl/soc.h>

/**
@defgroup CSL_CACHE_SYMBOL  CACHE Symbols Defined
@ingroup CSL_CACHE_API
*/
/**
@defgroup CSL_CACHE_ENUM  CACHE Enumerated Data Types
@ingroup CSL_CACHE_API
*/
/**
@defgroup CSL_CACHE_FUNCTION  CACHE Functions
@ingroup CSL_CACHE_API
*/


/**
@addtogroup CSL_CACHE_SYMBOL
@{
*/
/** L2 Line Size */
#define CACHE_L2_LINESIZE    128 
/** L1D Line Size */
#define CACHE_L1D_LINESIZE    64
/** L1P Line Size */
#define CACHE_L1P_LINESIZE    32

/** Handle to the CGEM Register Layer */
#define hCache      ((CSL_CgemRegs *)CSL_CGEM0_5_REG_BASE_ADDRESS_REGS)

/** Cache Round to Line size */        
#define CACHE_ROUND_TO_LINESIZE(CACHE,ELCNT,ELSIZE)\
( ( ( ( (ELCNT) * (ELSIZE)\
        + CACHE_##CACHE##_LINESIZE - 1\
      ) / CACHE_##CACHE##_LINESIZE\
      * CACHE_##CACHE##_LINESIZE\
    ) + (ELSIZE) - 1\
  ) / (ELSIZE)\
)
/**
@}
*/

/******************************************************************************\
* global macro declarations
\******************************************************************************/

/** @addtogroup CSL_CACHE_ENUM
 @{ */
/** @brief Enumeration for Cache wait flags
 *
 *  This is used for specifying whether the cache operations should block till
 *  the desired operation is complete.
 */
typedef enum 
{
    /** No blocking, the call exits after programmation of the 
     *  control registers 
     */
    CACHE_NOWAIT = 0,
                         
    /** Blocking Call, the call exits after the relevant cache 
     *  status registers indicate completion. For block coherence
     *  this waits on the Word count register to be come 0.
     */
    CACHE_WAIT   = 1,

    /** Blocking Call, For block coherence this uses the MFENCE to 
     * wait for completion
     */
    CACHE_FENCE_WAIT  = 2    
}CACHE_Wait;

/** @brief Enumeration for L1 (P or D) Sizes */

typedef enum { 
    /** No Cache    */                 
    CACHE_L1_0KCACHE  = 0,
    /** 4KB Cache   */                               
    CACHE_L1_4KCACHE  = 1,
    /** 8KB Cache   */                 
    CACHE_L1_8KCACHE  = 2,
    /** 16KB Cache  */                                 
    CACHE_L1_16KCACHE = 3,
    /** 32KB Cache  */                
    CACHE_L1_32KCACHE = 4,
    /** MAX Cache Size */                
    CACHE_L1_MAXIM1   = 5,
    /** MAX Cache Size */                
    CACHE_L1_MAXIM2   = 6,      
    /** MAX Cache Size */                
    CACHE_L1_MAXIM3   = 7          
} CACHE_L1Size;

/** @brief Enumeration for L2 Sizes */

    /** For devices that have maximum caches less 
     * than the defined values, setting a higher value 
     * will map to maximum cache 
     */
typedef enum { 
    /** No Cache     */
    CACHE_0KCACHE   = 0,  
    /** 32KB Cache   */
    CACHE_32KCACHE  = 1, 
    /** 64KB Cache   */
    CACHE_64KCACHE  = 2, 
    /** 128KB Cache  */
    CACHE_128KCACHE = 3,
    /** 256KB Cache  */
    CACHE_256KCACHE = 4,
    /** 512KB Cache  */
    CACHE_512KCACHE = 5,
    /* 1024KB Cache  */
    CACHE_1024KCACHE = 6 
} CACHE_L2Size; 

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /*_CSL_CACHE_H_*/
