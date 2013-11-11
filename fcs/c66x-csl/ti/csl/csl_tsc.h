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
 *   @file  csl_tsc.h
 *
 *   @brief  
 *      Header file for functional layer of TSC CSL
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par 
 */

 /** @defgroup CSL_TSC_API TSC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 *  Time Stamp Counter is a free running 64-bit CPU counter that advances each CPU 
 *  clock after counting is enabled. The counter is accessed using two 32-bit 
 *  read-only control registers, Time Stamp Counter Registers – Low (TSCL) and 
 *  Time Stamp Counter Registers – High (TSCH). The counter is enabled by writing to 
 *  TSCL. The value written is ignored. Once enabled, counting cannot be disabled under 
 *  program control. Counting is disabled in the following cases:
 *          a.  After exiting the reset state.
 *          b.  When the CPU is fully powered down.
 *
 * @subsection References
 *   -# Joule CPU Architecture Spec
 *
 * @subsection Assumptions
 *    The abbreviations TSC, tsc and Tsc have been used throughout this
 *    document to refer to C64+ Time Stamp Counter
 */
 
#ifndef _CSL_TSC_H_
#define _CSL_TSC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>

/**
@defgroup CSL_TSC_FUNCTION  TSC Functions
@ingroup CSL_TSC_API
*/

/** @addtogroup CSL_TSC_FUNCTION
 @{ */

/** ===========================================================================
 *   @n@b CSL_tscEnable
 *
 *   @b Description
 *   @n This API enables the 64 bit time stamp counter. The Time Stamp Counter
 *      (TSC) stops only upon Reset or powerdown. When time stamp counter is 
 *      enabled (following a reset or powerdown of the CPU) it will initialize 
 *      to 0 and begin incrementing once per CPU cycle. You cannot reset the 
 *      time stamp counter. 
 *
 *   @b Arguments          
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n Time Stamp Counter value starts incrementing
 *
 *   @b Writes
 *   @n TSCL=0,
 *      TSCH=0
 *
 *   @b Example
 *   @verbatim
        
        CSL_tscEnable ();
        
 *   @endverbatim
 * ============================================================================
 */
extern void  CSL_tscEnable(void);


/** ============================================================================
 *   @n@b CSL_tscRead
 *
 *   @b Description
 *   @n Reads the 64 bit timer stamp conter and return the 64-bit counter value.
 *
 *   @b Arguments   None
 * 
 *   <b> Return Value </b>  CSL_Uint64
 *   @li                    64 Bit Time Stamp Counter value
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads            
 *      TSCL,
 *      TSCH
 *
 *   @b Example
 *   @verbatim
        CSL_Uint64        counterVal;
        
        ...
        
        CSL_tscStart();
        
        ...
        
        counterVal = CSL_tscRead();
        
 *   @endverbatim
 * ============================================================================
 */
extern CSL_Uint64 CSL_tscRead (void);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif
