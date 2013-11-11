/**
 *   @file  csl_mpu.h
 *
 *   @brief
 *      This is the header file for the MPU CSL Functional Layer.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2011 Texas Instruments, Inc.
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

/** @defgroup CSL_MPU_API MPU
 *
 * @section Introduction
 *  The MPU performs memory protection checking for a CBA bus. It inputs a VBUSM 
 *  or VBUSP bus, checks the address against the fixed and programmable regions 
 *  to see if the access is allowed. If allowed the transfer is passed unmodified
 *  to the output VBUSM or VBUSP bus. If the transfer is illegal (fails the 
 *  protection check) then the MPU does not pass the transfer to the output bus 
 *  but rather services the transfer internally back to the input bus (to prevent
 *  a hang) returning the fault status to the requestor as well as generating an 
 *  interrupt about the fault. 
 *
 * @subsection References
 *   -# MPU Module Specification.
 */
#ifndef _CSL_MPU_H_
#define _CSL_MPU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_mpu.h>

/**
@defgroup CSL_MPU_SYMBOL  MPU Symbols Defined
@ingroup CSL_MPU_API
*/
/**
@defgroup CSL_MPU_DATASTRUCT  MPU Data Structures
@ingroup CSL_MPU_API
*/
/**
@defgroup CSL_MPU_FUNCTION  MPU Functions
@ingroup CSL_MPU_API
*/

/** @addtogroup CSL_MPU_DATASTRUCT
 @{ */
        
/** @brief This is the handle to the MPU Register */
typedef volatile CSL_MpuRegs*   CSL_MpuHandle;

/**
@}
*/

/* Device specific API which opens the MPU instance and returns a handle used in all subsequent calls */
extern CSL_MpuHandle CSL_MPU_Open (Int32 instNum);

#ifdef __cplusplus
}
#endif

#endif /* _CSL_MPU_H_ */

