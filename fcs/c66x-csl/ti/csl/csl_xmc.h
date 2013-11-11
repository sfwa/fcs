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
 *   @file  csl_xmc.h
 *
 *   @brief  
 *      XMC CSL Implementation on DSP side
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par  
 */

/** @defgroup CSL_XMC_API XMC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 * The XMC takes on the following roles:
 * 		a) UMC to VBusM conversion
 * 		b) Shared memory access path
 * 		c) Prefetch support
 * 		d) Address extension/translation
 * 		e) Memory protection for addresses outside CGEM eg. MSMC RAM or EMIF
 *
 * @subsection References
 *   -# XMC User's Guide
 *
 * @subsection Assumptions
 *    The abbreviations XMC, xmc and Xmc have been used throughout this
 *    document to refer to CGEM eXtended Memory Controller
 */

#ifndef CSL_XMC_H
#define CSL_XMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_xmc.h>

/**
@defgroup CSL_XMC_SYMBOL  XMC Symbols Defined
@ingroup CSL_XMC_API
*/
/**
@defgroup CSL_XMC_DATASTRUCT  XMC Data Structures
@ingroup CSL_XMC_API
*/
/**
@defgroup CSL_XMC_FUNCTION  XMC Functions
@ingroup CSL_XMC_API
*/

/** @addtogroup CSL_XMC_DATASTRUCT
 @{ */

/** @brief This is the definition of CSL_XMC_XMPAXH */
typedef struct CSL_XMC_XMPAXH
{
	/** Base Address */	
	Uint32 bAddr;
	
	/** Encoded Segment Size */	
	Uint8 segSize;
}CSL_XMC_XMPAXH;

/** @brief This is the definition of CSL_XMC_XMPAXL */
typedef struct CSL_XMC_XMPAXL 
{
	/** Replacement Address */
	Uint32 rAddr;
	
	/** When set, supervisor may read from segment */	
	Uint32 sr;

	/** When set, supervisor may write to segment */	
	Uint32 sw;
	
	/** When set, supervisor may execute from segment */	
	Uint32 sx;
	
	/** When set, user may read from segment */	
	Uint32 ur;

	/** When set, user may write to segment */	
	Uint32 uw;

	/** When set, user may execute from segment */	
	Uint32 ux;
}CSL_XMC_XMPAXL;

/** @brief This is the definition of CSL_XMC_MPFSR */
typedef struct CSL_XMC_MPFSR 
{
	/** Local Access ? */	
	Uint32 local;
	
	/** When set, indicates a Supervisor Read Request */
	Uint32 sr;
	
	/** When set, indicates a Supervisor Write Request */	
	Uint32 sw;
	
	/** When set, indicates a Supervisor program fetch Request */	
	Uint32 sx;
	
	/** When set, indicates a User Read Request */	
	Uint32 ur;
	
	/** When set, indicates a User Write Request */	
	Uint32 uw;

	/** When set, indicates a User program fetch Request */	
	Uint32 ux;			
}CSL_XMC_MPFSR;

/** @brief This is the definition of CSL_XMC_XPFADDR */
typedef struct CSL_XMC_XPFADDR 
{
	/** Stream address (128-bit aligned) */	
	Uint32 addr;

	/** Sign bit of stream direction (0=fwd, 1=rev) */	
	Uint32 dir;
	
	/** Data pending for upper ("high") half */	
	Uint32 dph;
	
	/** Data valid for upper half */	
	Uint32 dvh;
	
	/** Address valid for upper half */	
	Uint32 avh;
	
	/** Data pending for lower half */	
	Uint32 dpl;
	
	/** Data valid for lower half */	
	Uint32 dvl;
	
	/** Address valid for lower half */	
	Uint32 avl;
}CSL_XMC_XPFADDR;

/** @brief This is the definition of CSL_XMC_ACEN_MODE */
typedef enum {
    /** Disable analysis counters. */        
    CSL_XMC_ACEN_MODE_COUNT_DISABLE = 0,
    /** Count Program events only. */        
    CSL_XMC_ACEN_MODE_COUNT_PROGEVENTS = 1,
    /** Count Data events only. */        
    CSL_XMC_ACEN_MODE_COUNT_DATAEVENTS = 0,
    /** Count both program and data events. */        
    CSL_XMC_ACEN_MODE_COUNT_ALL = 0
}CSL_XMC_ACEN_MODE;
	
/**
 *  Handle to access XMC registers accessible through config bus.
 */
#define hXmc     ((CSL_XmcRegs*)CSL_XMC_CONFIG_REGS)

/**
@}
*/


#ifdef __cplusplus
}
#endif
#endif /* CSL_XMC_H */

