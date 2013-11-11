/**
 *   @file  csl_pllc.h
 *
 *   @brief
 *      This is the main Header File for the PLLC Module which defines all
 *      the data structures and exported API.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010 Texas Instruments, Inc.
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
/** @defgroup CSL_PLLC_API PLLC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * 
 * The Phase-Locked Loop (PLL) controller offers flexibility and 
 * convenience by way of software configurable multipliers and
 * dividers to modify the clock input signal internally. The 
 * resulting clock outputs are passed to the DSP core, peripherals 
 * and other modules in the DSP.
 *
 * @subsection References
 *   -# PLLC User Guide. 
 *
 * @subsection Assumptions
 *    The abbreviations PLLC, pllc and Pllc have been used throughout this
 *    document to refer to PLL Controller.
 */
#ifndef _CSL_PLLC_H_
#define _CSL_PLLC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_pllc.h>

/**
@defgroup CSL_PLLC_SYMBOL  PLLC Symbols Defined
@ingroup CSL_PLLC_API
*/
/**
@defgroup CSL_PLLC_DATASTRUCT  PLLC Data Structures
@ingroup CSL_PLLC_API
*/
/**
@defgroup CSL_PLLC_FUNCTION  PLLC Functions
@ingroup CSL_PLLC_API
*/
/**
@defgroup CSL_PLLC_ENUM PLLC Enumerated Data Types
@ingroup CSL_PLLC_API
*/

/**
@addtogroup CSL_PLLC_SYMBOL
@{
*/

/**
@}
*/

/** @addtogroup CSL_PLLC_DATASTRUCT
 @{ */

/** @brief This is the handle to the PLLC instance */
typedef volatile CSL_PllcRegs*                  CSL_PllcHandle;

/** @brief PLLC Valid Key for configure Reset Control and Reset Cfg registers */
#define CSL_PLLC_RSTCTRL_VALID_KEY	            0x5A69


/**
@}
*/

/* Device specific API which opens the PLLC instance and returns a handle used in all subsequent calls */
extern CSL_PllcHandle CSL_PLLC_open (Int32 instNum);        


#ifdef __cplusplus
}
#endif

#endif  /* _CSL_PLLC_H_ */

