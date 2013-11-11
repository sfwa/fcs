/**
 *   @file  csl_msmc.h
 *
 *   @brief   
 *      This is the main header file for the MSMC Module which defines
 *      all the data structures and exported API.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
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

/** @defgroup CSL_MSMC_API MSMC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * The MSMC is a module designed to manage traffic between multiple processor cores, other
 * mastering peripherals or DMA and the EMIF in a multi-core SoC. The MSMC provides a shared
 * on-chip memory that can be used either as a shared on-chip SRAM or a cache for external
 * memory traffic. It is part of the compute infrastructure for CGEM 0.5 and hence is designed to
 * work with the CGEM 0.5 processor cores.
 *
 * @subsection References
 *   -# MSMC User's Guide
 *
 * @subsection Assumptions
 *    The abbreviations MSMC, msmc and Msmc have been used throughout this
 *    document to refer to Multi-Core Shared Memory Controller
 */

#ifndef CSL_MSMC_H
#define CSL_MSMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_msmc.h>

/**
@defgroup CSL_MSMC_SYMBOL  MSMC Symbols Defined
@ingroup CSL_MSMC_API
*/
/**
@defgroup CSL_MSMC_DATASTRUCT  MSMC Data Structures
@ingroup CSL_MSMC_API
*/
/**
@defgroup CSL_MSMC_FUNCTION  MSMC Functions
@ingroup CSL_MSMC_API
*/

/**
@addtogroup CSL_MSMC_SYMBOL
@{
*/

/**
 *  Handle to access MSMC registers accessible through config bus.
 */
#define hMsmc     ((CSL_MsmcRegs*)CSL_MSMC_CONFIG_REGS)

/**
@}
*/

/** @addtogroup CSL_MSMC_DATASTRUCT
 @{ */

typedef struct {
	Uint16 baddr;
	Uint8  segSz;
}CSL_MSMC_SMSMPAXH;

typedef struct {
	Uint16  raddr;
	Uint32  sr;
	Uint32  sw;
	Uint32  sx;
	Uint32  ur;
	Uint32  uw;
	Uint32  ux;
}CSL_MSMC_SMSMPAXL;

typedef struct {
	Uint32 baddr;
	Uint32 be;
	Uint32 ai;
	Uint8  segSz;
}CSL_MSMC_SESMPAXH;

typedef struct {
	Uint32 raddr;
	Uint32 sr;
	Uint32 sw;
	Uint32 sx;
	Uint32 ur;
	Uint32 uw;
	Uint32 ux;
}CSL_MSMC_SESMPAXL;

/* @} */

/** @addtogroup CSL_MSMC_FUNCTION
 @{ */

/* @} */


#ifdef __cplusplus
}
#endif
#endif /* CSL_MSMC_H */
