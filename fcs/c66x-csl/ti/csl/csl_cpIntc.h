/**
 *   @file  csl_cpIntc.h
 *
 *   @brief   
 *      This is the main header file for the CPINTC Module which defines
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

/** @defgroup CSL_CPINTC_API CPINTC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *  The CPINTC is the interrupt controller which handles the system interrupts
 *  for the host, processes & prioritizes them and then is responsible for 
 *  delivering these to the host.
 *
 * @subsection References
 *   -# CPINTC Architecture Specification.
 *
 * @subsection Assumptions
 *    The abbreviations CPINTC, cpintc have been used throughout this
 *    document to refer to Interrupt Controller.
 */
 
#ifndef _CSL_CPINTC_H_
#define _CSL_CPINTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_cpintc.h>

/**
@defgroup CSL_CPINTC_SYMBOL  CPINTC Symbols Defined
@ingroup CSL_CPINTC_API
*/
/**
@defgroup CSL_CPINTC_DATASTRUCT  CPINTC Data Structures
@ingroup CSL_CPINTC_API
*/
/**
@defgroup CSL_CPINTC_FUNCTION  CPINTC Functions
@ingroup CSL_CPINTC_API
*/

/** @addtogroup CSL_CPINTC_DATASTRUCT
 @{ */
 
/** @brief Register Overlay Memory map for the CPINTC0 Registers. */
typedef volatile CSL_CPINTCRegs*  CSL_CPINTC_RegsOvly;

/** @brief This is the handle to the CPINTC instance */
typedef Uint32   CSL_CPINTC_Handle;

/** @brief This defines the system interrupt */
typedef Uint32   CSL_CPINTCSystemInterrupt;

/** @brief This defines the host interrupt */
typedef Uint32   CSL_CPINTCHostInterrupt;

/** @brief This defines the channels */
typedef Uint32   CSL_CPINTCChannel;

/** @brief This defines the nesting level */
typedef Uint16   CSL_CPINTCNestingLevel;

/** @brief Enumeration defines the type of Nesting Modes which is supported by the CPINTC */
typedef enum CSL_CPINTCNestingMode
{
    CPINTC_NO_NESTING            = 0x0,
    CPINTC_AUTOMATIC_GLB_NESTING = 0x1,
    CPINTC_AUTOMATIC_IND_NESTING = 0x2,
    CPINTC_MANUAL_NESTING        = 0x3
}CSL_CPINTCNestingMode;

/**
@}
*/

/* Device specific API which opens the CPINTC instance and returns a handle used in all subsequent calls */
extern CSL_CPINTC_Handle CSL_CPINTC_open (Int32 instNum);

#ifdef __cplusplus
}
#endif

#endif /* _CSL_CPINTC_H_ */

