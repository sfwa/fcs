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
 *   @file  csl_ipc.h
 *
 *   @brief  
 *      IPC CSL Implementation on DSP side
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par
 */

/** @defgroup CSL_IPC_API IPC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 * Each of the GEM cores can communicate with one another in a variety of ways.
 * Inter-Processor interrupts (IPC) is one of the ways used for core synchronization,
 * allowing for direct notification from one GEM core to another.
 * Three different IPC mechanisms are defined:
 * 		a)	Inter-DSP Interrupts.
 * 		b)	Host CPU Interrupts (interrupts to external host).
 * 		c)	Non Maskable Interrupts (NMI) to GEMs. 
 *
 * @subsection References
 *   -# Inter-Processor Communications (IPC) Architecture 
 *
 * @subsection Assumptions
 *    The abbreviations IPC, ipc and Ipc have been used throughout this
 *    document to refer to Inter-Processor Communications (IPC) Architecture
 */
#ifndef CSL_IPC_H_
#define CSL_IPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_ipc.h>

/**
@defgroup CSL_IPC_SYMBOL  IPC Symbols Defined
@ingroup CSL_IPC_API
*/
/**
@defgroup CSL_IPC_DATASTRUCT  IPC Data Structures
@ingroup CSL_IPC_API
*/
/**
@defgroup CSL_IPC_FUNCTION  IPC Functions
@ingroup CSL_IPC_API
*/

/**
@addtogroup CSL_IPC_SYMBOL
@{
*/

/**
@}
*/


/** @addtogroup CSL_IPC_DATASTRUCT
 @{ */

/**
 *  Handle to access IPC registers accessible through config bus.
 */
#define hIpc     ((CSL_IPCRegs *) (CSL_BOOT_CFG_REGS + 0x200))

/* @} */

/** @addtogroup CSL_IPC_FUNCTION
 @{ */

/* @} */


#ifdef __cplusplus
}
#endif

#endif /*CSL_IPC_H_*/
