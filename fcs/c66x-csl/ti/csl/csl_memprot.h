/* ============================================================================
 * Copyright (c) Texas Instruments Incorporated 2008, 2009                
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
 * @file csl_memprot.h
 *
 * @brief 
 *  Header file for functional layer of CSL. It contains the various enumerations, 
 *  structure definitions and function declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par 
 */

/** ============================================================================ 
 *
 * @defgroup CSL_MEMPROT_API MEMPROT
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * Memory protection used to support resources (L1P, L2, L1D not an Intenal CFG space). 
 * Memory protection provides many benefits to a system.
 * Memory protection functionality can:
 * · Protect operating system data structures from poorly behaving code.
 * · Aid in debugging by providing greater information about illegal memory accesses.
 * · Allow the operating system to enforce clearly defined boundaries between supervisor and
 *   user modeaccesses, leading to greater system robustness.
 *
 * @subsection References
 *    -# TMS320C64x+ DSP Megamodule SPRU871I.pdf (May 2008)
 * ============================================================================
 */ 

#ifndef _CSL_MEMPROT_H_
#define _CSL_MEMPROT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_cgem.h>

/**
@defgroup CSL_MEMPROT_SYMBOL  MEMPROT Symbols Defined
@ingroup CSL_MEMPROT_API
*/
/**
@defgroup CSL_MEMPROT_DATASTRUCT  MEMPROT Data Structures
@ingroup CSL_MEMPROT_API
*/
/**
@defgroup CSL_MEMPROT_FUNCTION  MEMPROT Functions
@ingroup CSL_MEMPROT_API
*/
/**
@defgroup CSL_MEMPROT_ENUM MEMPROT Enumerated Data Types
@ingroup CSL_MEMPROT_API
*/
    
/** @addtogroup CSL_MEMPROT_DATASTRUCT
 @{ */

/** Constants for passing parameters to the functions.
 */
/** @brief Pointer to the memeory protection overlay registers */
#define hCgemRegs       ((CSL_CgemRegs *)CSL_CGEM0_5_REG_BASE_ADDRESS_REGS)

/** @brief This will be used to lock/unlock/reset a memory region.
 */

typedef struct {
	/** Indicates whether Emulation and Non secure may 
	 *	 manipulate the lock hardware.
	 */
	Uint32 nsl:1;

	/** Indicates the Lock Status; 1 indicates Locked, 0 means disengaged.
	 */					
	Uint32 lk:1;
}CSL_MEMPROT_MPLKSTAT;

/** @brief This will be used to query the memory fault status.
 */
typedef struct {
	/** Faulted ID */	
	Uint32 fid:7;		
			
	/** Local Access ? */
	Uint32 local:1;				
	
	/** When set, indicates a Non-secure access */
	Uint32 ns:1;				
	
	/** When set, indicates a Supervisor Read Request */
	Uint32 sr:1;				
	
	/** When set, indicates a Supervisor Write Request */
	Uint32 sw:1;				
	
	/** When set, indicates a Supervisor program fetch Request */
	Uint32 sx:1;				
	
	/** When set, indicates a User Read Request */
	Uint32 ur:1;				
	
	/** When set, indicates a User Write Request */
	Uint32 uw:1;				
	
	/** When set, indicates a User program fetch Request */
	Uint32 ux:1;				
}CSL_MEMPROT_MPFSR;

/** @brief This will be used to set/query the memory page attributes
 */
typedef struct {
	/** Allow access from VBUS PrivID=5 */
	Uint32 aid5:1;

	/** Allow access from VBUS PrivID=4 */	
	Uint32 aid4:1;
	
	/** Allow access from VBUS PrivID=3 */	
	Uint32 aid3:1;
	
	/** Allow access from VBUS PrivID=2 */	
	Uint32 aid2:1;
	
	/** Allow access from VBUS PrivID=1 */	
	Uint32 aid1:1;
	
	/** Allow access from VBUS PrivID=0 */	
	Uint32 aid0:1;
	
	/** Allow access from VBUS PrivID>=6 */	
	Uint32 aidx:1;	
	
	/** Allow access from CPU to its local memories (L1/L2 only) */	
	Uint32 local:1;		
	
	/** Supervisor may read */	
	Uint32 sr:1;
	
	/** Supervisor may write */	
	Uint32 sw:1;
	
	/** Supervisor may execute */	
	Uint32 sx:1;

	/** User may read */	
	Uint32 ur:1;
	
	/** User may write */	
	Uint32 uw:1;

	/** User may execute */	
	Uint32 ux:1;
}CSL_MEMPROT_MPPA;

/**
@}
*/


/** @addtogroup CSL_MEMPROT_FUNCTION
 @{ */

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _CSL_MEMPROT_H */
