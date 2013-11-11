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
 * @file csl_bwmngmt.h
 *
 * @brief 
 *  Header file for functional layer of CSL. It contains the various enumerations, 
 *  structure definitions and function declaration
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par
 */

/** @defgroup CSL_BWMNGMT_API BWMNGMT    
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This page describes the Functions, Data Structures, Enumerations and Macros
 * within BWMNGMT module.
 * 
 * The Bandwidth management module is used to avoid the case of a requestor
 * (CPU, SDMA,IDMA, and Coherence Operations) being blocked from accessing a
 * resource (L1P, L1D, L2, and configuration bus) for a long period of time.
 *
 * The following four resources are managed by the BWM control hardware:
 *    -Level 1 Program (L1P) SRAM/Cache
 *    -Level 1 Data (L1D) SRAM/Cache
 *    -Level 2 (L2) SRAM/Cache
 *    -Memory-mapped registers configuration bus
 *
 * @subsection References
 *    -# TMS320C64x+ DSP Megamodule Reference Guide SPRU871I (May 2008)
 *
 * @subsection Assumptions
 *     The abbreviation BWMNGMT has been used throughout this document
 *     to refer to the C64Plus Bandwidth Management Module.
 * ============================================================================
 */
 
#ifndef CSL_BWMNGMT_H_
#define CSL_BWMNGMT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/csl/cslr_cgem.h>
#include <ti/csl/soc.h>

/**
@defgroup CSL_BWMNGMT_ENUM  BWMNGMT Enumerated Data Types
@ingroup CSL_BWMNGMT_API
*/
/**
@defgroup CSL_BWMNGMT_DATASTRUCT  BWMNGMT Data Structures
@ingroup CSL_BWMNGMT_API
*/
/**
@defgroup CSL_BWMNGMT_SYMBOL  BWMNGMT Symbols Defined
@ingroup CSL_BWMNGMT_API
*/
/**
@defgroup CSL_BWMNGMT_FUNCTION  BWMNGMT Functions
@ingroup CSL_BWMNGMT_API
*/

/** @addtogroup CSL_BWMNGMT_SYMBOL
 @{ */


/** Priority Encodings for BWMNGMT 
 * 
 */
/** @brief Priority arbitration setting 0 - Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI0 	(0x00000000u)
/** @brief Priority arbitration setting 1 - 2nd Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI1 	(0x00000001u)
/** @brief Priority arbitration setting 2 - 3rd Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI2 	(0x00000002u)
/** @brief Priority arbitration setting 3 - 4th Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI3 	(0x00000003u)
/** @brief Priority arbitration setting 4 - 5th Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI4 	(0x00000004u)
/** @brief Priority arbitration setting 5 - 6th Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI5 	(0x00000005u)
/** @brief Priority arbitration setting 6 - 7th Highest priority requestor */
#define CSL_BWMNGMT_PRI_PRI6 	(0x00000006u)
/** @brief Priority arbitration setting 7 - Lowest priority requestor */
#define CSL_BWMNGMT_PRI_PRI7 	(0x00000007u)


/** Max Wait Encodings for BWMNGMT 
 *
 */
/** @brief Maxwait arbitration setting 0 - Always stall due to higher priority 
 *  requestor 
 */ 
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT0 	(0x00000000u)
/** @brief Maxwait arbitration setting 1 - Stall max of 1 cycle due to higher 
 *  priority requestor 
 */
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT1 	(0x00000001u)
/** @brief Maxwait arbitration setting 2 - Stall max of 2 cycle due to higher 
 *  priority requestor 
 */
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT2 	(0x00000002u)
/** @brief Maxwait arbitration setting 4 - Stall max of 4 cycle due to higher 
 *  priority requestor 
 */
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT4 	(0x00000004u)
/** @brief Maxwait arbitration setting 8 - Stall max of 8 cycle due to higher 
 *  priority requestor 
 */
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT8 	(0x00000008u)
/** @brief Maxwait arbitration setting 16 - Stall max of 16 cycle due to higher 
 *  priority requestor 
 */
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT16 	(0x00000010u)
/** @brief Maxwait arbitration setting 32 - Stall max of 32 cycle due to higher 
 *  priority requestor 
 */
#define CSL_BWMNGMT_MAXWAIT_MAXWAIT32 	(0x00000020u)

/**
@}*/


/** @addtogroup CSL_BWMNGMT_DATASTRUCT 
 * 
@{*/

/** @brief CSL_BWMNGMT_CPUARB_SETUP has all the fields required to configure 
 * 	the CPU Arbitration Control Register of BWMNGMT for any given memory 
 * 	control block (L1D/L2/EMC).
 */
typedef struct {
    /** CPU - Requestor Arbitration Settings - PRI */   
    Uint32		priority;       
    
    /** CPU - Requestor Arbitration Settings - MAXWAIT */   
    Uint32 		maxWait;
	
} CSL_BWMNGMT_CPUARB_SETUP;


/** @brief CSL_BWMNGMT_MDMAPRI_SETUP has all the fields required to configure 
 * 	Master DMA (MDMA) Arbitration Control Register of BWMNGMT for L2/UMC memory
 * 	control block.
 */

typedef struct {
    /** MDMA (Master DMA) Requestor Arbitration Settings - PRI */   
    Uint32 		priority;          
    
    /** MDMA (Master DMA) Requestor Arbitration Settings - EPRI (Urgent Priority) */   
    Uint32 		uPriority;
	
} CSL_BWMNGMT_MDMAPRI_SETUP;


/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
/**
 *  Handle to access Bandwidth Management registers accessible through config bus.
 */
#define hCgem     ((CSL_CgemRegs*)CSL_CGEM0_5_REG_BASE_ADDRESS_REGS)

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /*CSL_BWMNGMT_H_*/
