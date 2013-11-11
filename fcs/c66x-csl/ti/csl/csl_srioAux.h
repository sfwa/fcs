/**
 *   @file  csl_srioAux.h
 *
 *   @brief   
 *      This is the SRIO Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the SRIO Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008-2011, Texas Instruments, Inc.
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

#ifndef _CSL_SRIOAUX_H_
#define _CSL_SRIOAUX_H_

#include <ti/csl/soc.h>
#include <ti/csl/csl_srio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_SRIO_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_SRIO_GetPID
 *
 *   @b Description
 *   @n This function reads the peripheral ID register which identifies the 
 *      peripheral number for the peripheral.
 *
 *   @b Arguments
     @verbatim
          revInfo      Peripheral Revision populated by this API
          classInfo    Peripheral class populated by this API
          typeInfo     Peripheral Type populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PID
 *
 *   @b Example
 *   @verbatim
        Uint32          pid;
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the SRIO Peripheral Identification.
        CSL_SRIO_GetPID (hSrio, &pid);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPID 
(
    CSL_SrioHandle  hSrio,
    Uint32*         pid
)
{
    *pid = hSrio->RIO_PID;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePeripheral
 *
 *   @b Description
 *   @n This function sets the PEREN bit in the Peripheral Control register and
 *      if not set this will prevent the peripheral from participating in any
 *      transmit transactions and will disable all incoming requests. This should
 *      be the last enable bit when bringing the device out of reset.
 *
 *   @b Arguments
     @verbatim
          hSrio   Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PCR_PEREN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the SRIO peripheral. 
        CSL_SRIO_EnablePeripheral (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePeripheral (CSL_SrioHandle hSrio)
{
    CSL_FINS(hSrio->RIO_PCR, SRIO_RIO_PCR_PEREN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePeripheral
 *
 *   @b Description
 *   @n This function resets the PEREN bit in the Peripheral Control register 
 *      and disables the SRIO peripheral.
 *
 *   @b Arguments
     @verbatim
          hSrio   Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PCR_PEREN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the SRIO peripheral. 
        CSL_SRIO_DisablePeripheral (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePeripheral (CSL_SrioHandle hSrio)
{
    CSL_FINS(hSrio->RIO_PCR, SRIO_RIO_PCR_PEREN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetMAUSwappingMode
 *
 *   @b Description
 *   @n This function sets the Little Endian Swapping Mode for the Memory Access
 *      Unit.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the MAU.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Ensure that the Memory Access Unit operates in mode A
        CSL_SRIO_SetMAUSwappingMode (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetMAUSwappingMode (CSL_SrioHandle hSrio, Uint8 mode)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE, mode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetMAUSwappingMode
 *
 *   @b Description
 *   @n This function gets the Little Endian Swapping Mode for the Memory Access
 *      Unit.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the MAU populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Memory Access Unit Swapping Mode.
        CSL_SRIO_GetMAUSwappingMode (hSrio, &mode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetMAUSwappingMode (CSL_SrioHandle hSrio, Uint8* mode)
{
    *mode = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUSwappingMode
 *
 *   @b Description
 *   @n This function sets the Little Endian Swapping Mode for the Load Store 
 *      Unit
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the LSU.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Ensure that the LSU operates in mode A
        CSL_SRIO_SetLSUSwappingMode (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUSwappingMode (CSL_SrioHandle hSrio, Uint8 mode)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE, mode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLSUSwappingMode
 *
 *   @b Description
 *   @n This function gets the Little Endian Swapping Mode for the Load Store 
 *      Unit.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the LSU populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the LSU Swapping Mode.
        CSL_SRIO_GetLSUSwappingMode (hSrio, &mode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLSUSwappingMode (CSL_SrioHandle hSrio, Uint8* mode)
{
    *mode = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePromiscuous
 *
 *   @b Description
 *   @n This function enables the LOG_TGT_ID_DIS bit which ensures that the SRIO
 *      packets are accepted at the logical layer even if there is no match with 
 *      the destination identifier.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Operate the peripheral in PROMISCUOUS mode.
        CSL_SRIO_EnablePromiscuous (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePromiscuous (CSL_SrioHandle hSrio)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePromiscuous
 *
 *   @b Description
 *   @n This function disables the LOG_TGT_ID_DIS bit which ensures that the SRIO
 *      packets are accepted at the logical layer only if there is a match with the
 *      specific destination identifier.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Operate the peripheral in non-PROMISC mode.
        CSL_SRIO_DisablePromiscuous (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePromiscuous (CSL_SrioHandle hSrio)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPromiscuous
 *
 *   @b Description
 *   @n This function returns the status of the peripheral as TRUE if the device
 *      is operating in PROMISCUOUS mode else FALSE.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE  - Device is in PROMISCUOUS Mode.
 *   @n  FALSE - Device is in NON-PROMISCUOUS Mode.
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if we are operating in PROMISCUOUS mode or not?
        if (CSL_SRIO_IsPromiscuous (hSrio) == TRUE)
        {
            // Device is in PROMISCUOUS Mode.
        }
        else
        {
            // Device is in Non-PROMISCUOUS Mode.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPromiscuous (CSL_SrioHandle hSrio)
{
    if (CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetAMUSwappingMode
 *
 *   @b Description
 *   @n This function sets the Little Endian Swapping Mode for the AMU
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the AMU
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Ensure that the AMU operates in mode A
        CSL_SRIO_SetAMUSwappingMode (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetAMUSwappingMode (CSL_SrioHandle hSrio, Uint8 mode)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE, mode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUSwappingMode
 *
 *   @b Description
 *   @n This function gets the Little Endian Swapping Mode for the AMU
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the AMU populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the AMU Swapping Mode.
        CSL_SRIO_GetAMUSwappingMode (hSrio, &mode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUSwappingMode (CSL_SrioHandle hSrio, Uint8* mode)
{
    *mode = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetBootComplete
 *
 *   @b Description
 *   @n This function sets the boot complete flag as specified.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          bootCompleteFlag  Boot Complete Flag to be set in the register.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Boot Complete Flag to 1 
        CSL_SRIO_SetBootComplete (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetBootComplete (CSL_SrioHandle hSrio, Uint8 bootCompleteFlag)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE, bootCompleteFlag);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetBootComplete
 *
 *   @b Description
 *   @n This function gets the boot complete flag.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          bootCompleteFlag  Boot Complete Flag populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           bootCompleteFlag;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Boot Complete Flag
        bootCompleteFlag = CSL_SRIO_GetBootComplete (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetBootComplete (CSL_SrioHandle hSrio, Uint8* bootCompleteFlag)
{
    *bootCompleteFlag = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTXURXUSwappingMode
 *
 *   @b Description
 *   @n This function sets the Little Endian Swapping Mode for the TXU and RXU
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the TXU and RXU.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Ensure that the TXU and RXU operates in mode A
        CSL_SRIO_SetTXURXUSwappingMode (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTXURXUSwappingMode (CSL_SrioHandle hSrio, Uint8 mode)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE, mode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTXURXUSwappingMode
 *
 *   @b Description
 *   @n This function gets the Little Endian Swapping Mode for the Memory Access
 *      Unit.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          mode      Little Endian Swapping Mode for the TXU and RXU populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TXU and RXU Swapping Mode.
        CSL_SRIO_GetTXURXUSwappingMode (hSrio, &mode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTXURXUSwappingMode (CSL_SrioHandle hSrio, Uint8* mode)
{
    *mode = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableAutomaticPriorityPromotion
 *
 *   @b Description
 *   @n This function enables the automatic promotion of response priority by 
 *      RXU and MAU.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Automatic Priority Promotion.
        CSL_SRIO_EnableAutomaticPriorityPromotion (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableAutomaticPriorityPromotion (CSL_SrioHandle hSrio)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableAutomaticPriorityPromotion
 *
 *   @b Description
 *   @n This function disables the automatic promotion of response priority by 
 *      RXU and MAU.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Automatic Priority Promotion.
        CSL_SRIO_DisableAutomaticPriorityPromotion (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableAutomaticPriorityPromotion (CSL_SrioHandle hSrio)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsAutomaticPriorityPromotionEnabled
 *
 *   @b Description
 *   @n This function check if the automatic promotion of response priority by 
 *      RXU and MAU is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE  - Automatic Priority Promotion is enabled
 *   @n  FALSE - Automatic Priority Promotion is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Automatic Priority Promotion.
        if (CSL_SRIO_IsAutomaticPriorityPromotionEnabled (hSrio) == TRUE)
        {
            // Automatic Priority Promotion is enabled.
        }
        else
        {
            // Automatic Priority Promotion is disabled.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsAutomaticPriorityPromotionEnabled (CSL_SrioHandle hSrio)
{
    if (CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS) == 0)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTxCreditThreshold
 *
 *   @b Description
 *   @n This function sets the transmit credit threshold for priority packets
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          priority  Priority of the Transmit Buffers
          credit    Credit Threshold for the priority which is to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM;SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM;
 *      SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Transmit Credit Threshold as 8 for priority 2
        CSL_SRIO_SetTxCreditThreshold (hSrio, 2, 8);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTxCreditThreshold (CSL_SrioHandle hSrio,Uint8 priority,Uint8 credit)
{
    if (priority == 2)
    {
        CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM, credit);
    }
    else if (priority == 1)
    {
        CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM, credit);
    }
    else if (priority == 0)
    {
        CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM, credit);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTxCreditThreshold
 *
 *   @b Description
 *   @n This function gets the transmit credit threshold for priority packets
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          priority  Priority of the Transmit Buffers
          credit    Credit Threshold for the priority which is populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM;SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM;
 *      SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           credit;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Transmit Credit Threshold for priority 2
        CSL_SRIO_GetTxCreditThreshold (hSrio, 2, &credit);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTxCreditThreshold (CSL_SrioHandle hSrio,Uint8 priority,Uint8* credit)
{
    if (priority == 2)
    {
        *credit = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM);
    }
    else if (priority == 1)
    {
        *credit = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM);
    }
    else if (priority == 0)
    {
        *credit = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTransactionPriority
 *
 *   @b Description
 *   @n This function sets the DSP System transaction priority. 0 implies 
 *   Highest Priority while 7 implies lowest.
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          priority  Priority to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the transaction priority to be HIGHEST
        CSL_SRIO_SetTransactionPriority (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTransactionPriority (CSL_SrioHandle hSrio,Uint8 priority)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI, priority);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTransactionPriority
 *
 *   @b Description
 *   @n This function gets the DSP System transaction priority. 
 *
 *   @b Arguments
     @verbatim
          hSrio     Handle of the SRIO device
          priority  Priority to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           priority;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the transaction priority 
        CSL_SRIO_GetTransactionPriority (hSrio, &priority);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTransactionPriority (CSL_SrioHandle hSrio,Uint8 *priority)
{
    *priority = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPrescalarSelect
 *
 *   @b Description
 *   @n This function sets the prescalar select for SRIO
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          prescalarSelect   Prescalar Select which is to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the SRIO Prescalar select to operate in the range of 44.7 to 89.5
        CSL_SRIO_SetPrescalarSelect (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPrescalarSelect (CSL_SrioHandle hSrio, Uint8 prescalarSelect)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT, prescalarSelect);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPrescalarSelect
 *
 *   @b Description
 *   @n This function gets the prescalar select for SRIO
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          prescalarSelect   Prescalar Select which is to be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           prescalarSelect;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the SRIO Prescalar select.
        CSL_SRIO_GetPrescalarSelect (hSrio, &prescalarSelect);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPrescalarSelect (CSL_SrioHandle hSrio, Uint8* prescalarSelect)
{
    *prescalarSelect = CSL_FEXT(hSrio->RIO_PER_SET_CNTL, SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetResponseCRF
 *
 *   @b Description
 *   @n This function sets the CRF value in the response
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          value             CRF value to be set in the response.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_CRF
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Response CRF to be always set to 1
        CSL_SRIO_SetResponseCRF (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetResponseCRF (CSL_SrioHandle hSrio, Uint8 value)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_CRF, value);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetResponseCRF
 *
 *   @b Description
 *   @n This function gets the CRF value in the response
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          value             CRF value populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL1_CRF
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           crfValue;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Response CRF 
        CSL_SRIO_GetResponseCRF (hSrio, &crfValue);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetResponseCRF (CSL_SrioHandle hSrio, Uint8* value)
{
    *value = CSL_FEXT(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_CRF);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetRXUWatermark
 *
 *   @b Description
 *   @n This function sets the RXU Watermark value.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          value             0/1 to disable/enable RXU context assignment based on 
                            priority.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the RXU Watermark to to 1
        CSL_SRIO_SetRXUWatermark (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetRXUWatermark (CSL_SrioHandle hSrio, Uint8 value)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK, value);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetRXUWatermark
 *
 *   @b Description
 *   @n This function gets the RXU Watermark setting
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          value             RXU Watermark setting read 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           value;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the RXU Watermark
        CSL_SRIO_GetRXUWatermark (hSrio, &value);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetRXUWatermark (CSL_SrioHandle hSrio, Uint8* value)
{
    *value = CSL_FEXT(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK);
}


/** ============================================================================
 *   @n@b CSL_SRIO_SetSysClkSel
 *
 *   @b Description
 *   @n This function sets the SYS_CLK source
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          value             SYS_CLK source to configure
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the SYS_CLK source to 1
        CSL_SRIO_SetSysClkSel (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetSysClkSel (CSL_SrioHandle hSrio, Uint8 value)
{
    CSL_FINS(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL, value);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetSysClkSel
 *
 *   @b Description
 *   @n This function retrieves the SYS_CLK source for the port.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          value             SYS_CLK source read
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           value;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the SYS_CLK source
        CSL_SRIO_GetSysClkSel (hSrio, &value);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetSysClkSel (CSL_SrioHandle hSrio, Uint8* value)
{
    *value = CSL_FEXT(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLoopbackMode
 *
 *   @b Description
 *   @n This function sets the port to operate in loopback mode.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           Port Number for which loopback mode must be enabled
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_LOOPBACK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Port 0 to work in loopback mode.
        CSL_SRIO_SetLoopbackMode (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLoopbackMode (CSL_SrioHandle hSrio,Uint8 portNum)
{
    CSL_FINSR (hSrio->RIO_PER_SET_CNTL1, 
               CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT + portNum, CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT + portNum, 
               1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetNormalMode
 *
 *   @b Description
 *   @n This function sets the port to operate in Normal mode.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           Port Number for which normal mode must be enabled
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_LOOPBACK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the port 0 to work in normal mode.
        CSL_SRIO_SetNormalMode (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetNormalMode (CSL_SrioHandle hSrio,Uint8 portNum)
{
    CSL_FINSR (hSrio->RIO_PER_SET_CNTL1, 
               CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT + portNum, CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT + portNum, 
               0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsLoopbackMode
 *
 *   @b Description
 *   @n This function checks if the port specified is operating in loopback 
 *      mode or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           Port Number for which mode settings must be retrieved
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE   -   Port is operating in loopback mode
 *   @n  FALSE  -   Port is operating in Normal Mode.
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL1_LOOPBACK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the port 0 to work in normal mode.
        if (CSL_SRIO_IsLoopbackMode(hSrio, 0) == TRUE)
        {
            // Port 0 is operating in loopback mode.
        }
        else
        {
            // Port 0 is operating in normal mode.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsLoopbackMode (CSL_SrioHandle hSrio,Uint8 portNum)
{
    if (CSL_FEXTR (hSrio->RIO_PER_SET_CNTL1, 
                   CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT + portNum, CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT + portNum) == 1)
    {
        return TRUE;
    }
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableCOS
 *
 *   @b Description
 *   @n This function enables Class of Service
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_COS_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Class of Service
        CSL_SRIO_EnableCOS (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableCOS (CSL_SrioHandle hSrio)
{
    CSL_FINS (hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_COS_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableCOS
 *
 *   @b Description
 *   @n This function disables Class of Service for the device.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PER_SET_CNTL1_COS_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable COS.
        CSL_SRIO_DisableCOS (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableCOS (CSL_SrioHandle hSrio)
{
    CSL_FINS (hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_COS_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsCOSEnabled
 *
 *   @b Description
 *   @n This function checks if COS has been enabled or not for the device?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - COS is enabled
 *   @n FALSE - COS is disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PER_SET_CNTL1_COS_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if COS is enabled
        if (CSL_SRIO_IsCOSEnabled (hSrio) == TRUE)
        {
            // COS is ENABLED
        }
        else
        {
            // COS is DISABLED
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsCOSEnabled (CSL_SrioHandle hSrio)
{
    if (CSL_FEXT(hSrio->RIO_PER_SET_CNTL1, SRIO_RIO_PER_SET_CNTL1_COS_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GlobalEnable
 *
 *   @b Description
 *   @n This function globally enables the SRIO peripheral and all blocks.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_GBL_EN_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Globally Enable the SRIO peripheral.
        CSL_SRIO_GlobalEnable (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GlobalEnable (CSL_SrioHandle hSrio)
{
    CSL_FINS (hSrio->RIO_GBL_EN, SRIO_RIO_GBL_EN_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GlobalDisable
 *
 *   @b Description
 *   @n This function globally disables the SRIO peripheral and all blocks.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_GBL_EN_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Globally Disable the SRIO peripheral.
        CSL_SRIO_GlobalDisable (hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GlobalDisable (CSL_SrioHandle hSrio)
{
    CSL_FINS (hSrio->RIO_GBL_EN, SRIO_RIO_GBL_EN_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsSRIOEnabled
 *
 *   @b Description
 *   @n This function checks if the SRIO peripheral has been globally enabled or
 *      not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - SRIO Peripheral is enabled
 *   @n FALSE - SRIO Peripheral is disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_GBL_EN_STAT_GBL_EN_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Ensure that the SRIO Peripheral is enabled or disabled.
        if (CSL_SRIO_IsSRIOEnabled (hSrio) == TRUE)
        {
            // SRIO Peripheral is ENABLED
        }
        else
        {
            // SRIO Peripheral is DISABLED
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsSRIOEnabled (CSL_SrioHandle hSrio)
{
    if (CSL_FEXT(hSrio->RIO_GBL_EN_STAT, SRIO_RIO_GBL_EN_STAT_GBL_EN_STAT) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableBlock
 *
 *   @b Description
 *   @n This function enables the specific block in the SRIO peripheral.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          blockNum          Block Number to be enabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_BLK_EN_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the SRIO block for LSU .
        CSL_SRIO_GlobalEnable (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableBlock (CSL_SrioHandle hSrio, Uint16 blockNumber)
{
    CSL_FINS (hSrio->BLOCK_ENABLE_STATUS[blockNumber].RIO_BLK_EN, SRIO_RIO_BLK_EN_EN, 1); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableBlock
 *
 *   @b Description
 *   @n This function disables the specific block in the SRIO peripheral.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          blockNum          Block Number to be disabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_BLK_EN_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the SRIO Block 1 for LSU.
        CSL_SRIO_DisableBlock (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableBlock (CSL_SrioHandle hSrio, Uint16 blockNumber)
{
    CSL_FINS (hSrio->BLOCK_ENABLE_STATUS[blockNumber].RIO_BLK_EN, SRIO_RIO_BLK_EN_EN, 0); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsSRIOBlockEnabled
 *
 *   @b Description
 *   @n This function checks if the SRIO block is enabled or disabled?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          blockNumber       SRIO Block Number to be checked.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - SRIO Block is enabled
 *   @n FALSE - SRIO Block is disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_BLK_EN_STAT_EN_STATUS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Ensure that the SRIO Block 1 for LSU is enabled or not? 
        if (CSL_SRIO_IsSRIOBlockEnabled (hSrio, 1) == TRUE)
        {
            // SRIO Block 1 for LSU is ENABLED
        }
        else
        {
            // SRIO Block 1 for LSU is DISABLED
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsSRIOBlockEnabled (CSL_SrioHandle hSrio, Uint16 blockNumber)
{
    return (Bool)CSL_FEXT(hSrio->BLOCK_ENABLE_STATUS[blockNumber].RIO_BLK_EN_STAT,
                          SRIO_RIO_BLK_EN_STAT_EN_STATUS);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDeviceID
 *
 *   @b Description
 *   @n This function sets the DEVICE ID Register. There are multiple device
 *      ID registers which can exist in the system. The index parameter is used
 *      to indicate which register is to be configured.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the Device ID register to be configured (0-Based)
          NodeId8bit        8 Bit Device Identifier to be configured
          NodeId16bit       16 Bit Device Identifier to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_MULTIID_REG_8B_NODEID,SRIO_RIO_MULTIID_REG_16B_NODEID 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Configure the First Device ID register with a 16 bit identifer as 0xDEAD
        // and no 8 bit identifier.
        CSL_SRIO_SetDeviceID (hSrio, 0, 0x0, 0xDEAD);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDeviceID 
(
    CSL_SrioHandle  hSrio, 
    Uint8           index,
    Uint8           NodeId8bit,
    Uint16          NodeId16bit
)
{
    /* Set the 8bit and 16bit device identifiers. */
    hSrio->RIO_MULTIID_REG[index] = 
                CSL_FMK(SRIO_RIO_MULTIID_REG_8B_NODEID,  NodeId8bit) | 
                CSL_FMK(SRIO_RIO_MULTIID_REG_16B_NODEID, NodeId16bit);

}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDeviceID
 *
 *   @b Description
 *   @n This function gets the DEVICE ID Register. There are multiple device
 *      ID registers which can exist in the system. The index parameter is used
 *      to indicate which register is to be retreived.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the Device ID register to be configured (0-Based)
          NodeId8bit        8 Bit Device Identifier to be populated by this API
          NodeId16bit       16 Bit Device Identifier to be populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_MULTIID_REG_8B_NODEID,SRIO_RIO_MULTIID_REG_16B_NODEID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           NodeId8bit;
        Uint16          NodeId16bit;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Device ID register contents 
        CSL_SRIO_GetDeviceID (hSrio, 0, &NodeId8bit, &NodeId16bit);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDeviceID 
(
    CSL_SrioHandle  hSrio, 
    Uint8           index,
    Uint8*          NodeId8bit,
    Uint16*         NodeId16bit
)
{
    Uint32 value;

    /* Get the value from the register */
    value = hSrio->RIO_MULTIID_REG[index];

    /* Populate the 8bit and 16bit device identifiers. */
    *NodeId8bit  = CSL_FEXT(value, SRIO_RIO_MULTIID_REG_8B_NODEID);
    *NodeId16bit = CSL_FEXT(value, SRIO_RIO_MULTIID_REG_16B_NODEID);
    return;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPacketForwarding
 *
 *   @b Description
 *   @n The function sets the packet forwarding for the SRIO peripheral. There
 *   exist multiple packet forwarding registers and the parameter index is to used
 *   to select which register is to be configured.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the Packet forwarding register to be configured (0-Based)
          low16bitDeviceID  Lower bound 16bit Device Identifier which can use this
          up16bitDeviceID   Upper bound 16bit Device Identifier which can use this
          low8bitDeviceID   Lower bound 8bit Device Identifier which can use this
          up8bitDeviceID    Upper bound 8bit Device Identifier which can use this
          outPort           Outgoing SRIO port to be used.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP, SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO,
 *      SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO,SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP,
 *      SRIO_RIO_PF_8B_CNTL_OUT_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Create a packet forwarding entry for device ID 0xBEEF to be forwarded
        // to port 1; we dont care about the 8-bit identifiers hence these are 
        // set to reset values.
        CSL_SRIO_SetPacketForwarding (hSrio, 0, 0xBEEF, 0xBEEF, 0xFF, 0xFF, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPacketForwarding 
(
    CSL_SrioHandle  hSrio, 
    Uint8           index,
    Uint16          low16bitDeviceID,
    Uint16          up16bitDeviceID,
    Uint8           low8bitDeviceID,
    Uint8           up8bitDeviceID,
    Uint8           outPort
)
{
    hSrio->PF_CNTL[index].RIO_PF_16B_CNTL = 
            CSL_FMK(SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP, up16bitDeviceID) |
            CSL_FMK(SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO, low16bitDeviceID);

    hSrio->PF_CNTL[index].RIO_PF_8B_CNTL = 
            CSL_FMK(SRIO_RIO_PF_8B_CNTL_OUT_PORT,    outPort)           |
            CSL_FMK(SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP, up8bitDeviceID)    |
            CSL_FMK(SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO, low8bitDeviceID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPacketForwarding
 *
 *   @b Description
 *   @n The function gets the packet forwarding for the SRIO peripheral. There
 *   exist multiple packet forwarding registers and the parameter index is to used
 *   to select which register is to be retreived.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the Packet forwarding register to be configured (0-Based)
          low16bitDeviceID  Lower bound 16bit Device Identifier populated by this API
          up16bitDeviceID   Upper bound 16bit Device Identifier populated by this API
          low8bitDeviceID   Lower bound 8bit Device Identifier populated by this API
          up8bitDeviceID    Upper bound 8bit Device Identifier populated by this API
          outPort           Outgoing SRIO port populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO, SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP, 
 *      SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO,SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP,
 *      SRIO_RIO_PF_8B_CNTL_OUT_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          low16bitDeviceID;
        Uint16          up16bitDeviceID;
        Uint8           low8bitDeviceID;
        Uint8           up8bitDeviceID;
        Uint8           outPort;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the packet forwarding entry 0.
        CSL_SRIO_GetPacketForwarding (hSrio, 0, &low16bitDeviceID, &up16bitDeviceID, 
                                      &low8bitDeviceID, &up8bitDeviceID, &outPort);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPacketForwarding 
(
    CSL_SrioHandle  hSrio, 
    Uint8           index,
    Uint16*         low16bitDeviceID,
    Uint16*         up16bitDeviceID,
    Uint8*          low8bitDeviceID,
    Uint8*          up8bitDeviceID,
    Uint8*          outPort
)
{
    Uint32 value = hSrio->PF_CNTL[index].RIO_PF_16B_CNTL;
    *up16bitDeviceID  = CSL_FEXT(value, SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP);
    *low16bitDeviceID = CSL_FEXT(value, SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO);

    value = hSrio->PF_CNTL[index].RIO_PF_8B_CNTL;
    *outPort         = CSL_FEXT(value, SRIO_RIO_PF_8B_CNTL_OUT_PORT);
    *up8bitDeviceID  = CSL_FEXT(value, SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP);
    *low8bitDeviceID = CSL_FEXT(value, SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDoorbellPendingInterrupt
 *
 *   @b Description
 *   @n The function reads the DOORBELL status register to determine if there
 *      are any pending door bell interrupts or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          port              SRIO Port (Zero Based) for which the doorbell status is required.
          doorbellStatus    Door Bell Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_DOORBELL_ICSR_RIO_DOORBELL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          doorbellStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the status of doorbell interrupts for port 0
        CSL_SRIO_GetDoorbellPendingInterrupt (hSrio, 0, &doorbellStatus);
        if (doorbellStatus != 0)
        {
            // Door bell Interrupts were pending and need to be serviced.
        }
        else
        {
            // There are no door bell interrupts pending.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDoorbellPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint8           port,
    Uint16*         doorbellStatus
)
{
    *doorbellStatus = CSL_FEXT(hSrio->DOORBELL_ICSR_ICCR[port].RIO_DOORBELL_ICSR,
                               SRIO_RIO_DOORBELL_ICSR_RIO_DOORBELL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearDoorbellPendingInterrupt
 *
 *   @b Description
 *   @n The function clears a specific door bell interrupt in the specified door
 *   bell interrupt register.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          port              SRIO Port (Zero Based) for which the doorbell status is required.
          doorbellInterrupt Door Bell Interrupt to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_DOORBELL_ICCR
 *
 *   @b Affects
 *   @n SRIO_RIO_DOORBELL_ICSR_RIO_DOORBELL=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          doorbellStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the status of doorbell interrupts for port 0
        CSL_SRIO_GetDoorbellPendingInterrupt (hSrio, 0, &doorbellStatus);
        if (doorbellStatus != 0)
        {
            // Door bell Interrupts were pending and need to be serviced.
            ...
            // Clear the Door Bell Pending Interrupts.
            CSL_SRIO_ClearDoorbellPendingInterrupt (hSrio, 0, doorbellStatus);
        }
        else
        {
            // There are no door bell interrupts pending.
        }        
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearDoorbellPendingInterrupt 
(
    CSL_SrioHandle  hSrio, 
    Uint8           port,
    Uint16          doorbellInterrupt
)
{
    /* Clear all the specified doorbell interrupts. */ 
    hSrio->DOORBELL_ICSR_ICCR[port].RIO_DOORBELL_ICCR = doorbellInterrupt;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLSUPendingInterrupt
 *
 *   @b Description
 *   @n The function returns the LSU pending interrupt status.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          lsuStatus1        LSU Pending Status populated by this API
          lsuStatus2        LSU Pending Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_ICSR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          lsuStatus1;
        Uint32          lsuStatus2;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check the LSU Pending Interrupt Status.
        CSL_SRIO_GetLSUPendingInterrupt (hSrio, &lsuStatus1, &lsuStatus2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLSUPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint32*         lsuStatus1,
    Uint32*         lsuStatus2
)
{
    *lsuStatus1 = hSrio->LSU_ICSR_ICCR[0].RIO_LSU_ICSR;
    *lsuStatus2 = hSrio->LSU_ICSR_ICCR[1].RIO_LSU_ICSR;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearLSUPendingInterrupt
 *
 *   @b Description
 *   @n The function clears the LSU pending interrupt 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          lsuInterrupt1     LSU Interrupt to be cleared.
          lsuInterrupt2     LSU Interrupt to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_ICCR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          lsuStatus1;
        Uint32          lsuStatus2;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check the LSU Pending Interrupt Status.
        CSL_SRIO_GetLSUPendingInterrupt (hSrio, &lsuStatus1, &lsuStatus2);
        ...
        // Clear the pending interrupts.
        CSL_SRIO_ClearLSUPendingInterrupt (hSrio, lsuStatus1, lsuStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearLSUPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint32          lsuInterrupt1,
    Uint32          lsuInterrupt2
)
{
    hSrio->LSU_ICSR_ICCR[0].RIO_LSU_ICCR = lsuInterrupt1;
    hSrio->LSU_ICSR_ICCR[1].RIO_LSU_ICCR = lsuInterrupt2;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearLSU0PendingInterrupt
 *
 *   @b Description
 *   @n The function clears the LSU0 pending interrupt 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          lsuInterrupt      LSU Interrupt to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_ICCR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Clear the ICS0 indicating that the SRCID0 Transaction was successfully complete.
        CSL_SRIO_ClearLSU0PendingInterrupt (hSrio, 0x1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearLSU0PendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint32          lsuInterrupt
)
{
    hSrio->LSU_ICSR_ICCR[0].RIO_LSU_ICCR = lsuInterrupt;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorPendingInterrupt
 *
 *   @b Description
 *   @n The function returns the interrupt status of any error interrupts 
 *      which are pending.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          errStatus         Error Pending Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_ERR_RST_EVNT_ICSR_MCAST_INT_RECEIVED,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT_WRITE_IN_RECEIVED,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_LLERR_CAPTURE,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT0_ERR,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT1_ERR,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT2_ERR,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT3_ERR,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_DEVICE_RST_INT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get any pending errors 
        CSL_SRIO_GetErrorPendingInterrupt (hSrio, &errStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint32*         errStatus
)
{
    /* All the errors are present in the lower order 17 bits. */
    *errStatus = CSL_FEXTR(hSrio->RIO_ERR_RST_EVNT_ICSR, 16, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearErrorPendingInterrupt
 *
 *   @b Description
 *   @n The function clears the pending error interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          errStatus         Error Status to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_ERR_RST_EVNT_ICCR_MCAST_INT_RECEIVED,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_PORT_WRITE_IN_REQUEST_RECEIVED,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_LOGICAL_LAYER_ERROR_MANAGEMENT_EVENT_CAPTURE,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_PORT0_ERROR,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_PORT1_ERROR,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_PORT2_ERROR,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_PORT3_ERROR,
 *      SRIO_RIO_ERR_RST_EVNT_ICCR_DEVICE_RST_INT
 *
 *   @b Affects
 *   @n SRIO_RIO_ERR_RST_EVNT_ICSR_MCAST_INT_RECEIVED=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT_WRITE_IN_RECEIVED=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_LLERR_CAPTURE=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT0_ERR=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT1_ERR=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT2_ERR=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_PORT3_ERR=0,
 *      SRIO_RIO_ERR_RST_EVNT_ICSR_DEVICE_RST_INT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get any pending error status.
        CSL_SRIO_GetErrorPendingInterrupt (hSrio,&errStatus);
        if (errStatus != 0)
        {
            // Pending error interrupts detected.
            ...
            // Clear the pending interrupts.
            CSL_SRIO_ClearErrorPendingInterrupt (hSrio, errStatus);
        }
        else
        {
            // No pending error status
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearErrorPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint32          errStatus
)
{
    CSL_FINSR(hSrio->RIO_ERR_RST_EVNT_ICCR, 16, 0, errStatus);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUPendingInterrupt
 *
 *   @b Description
 *   @n The function returns the interrupt status of any AMU interrupts.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          amuIntrStatus     AMU Interrupt Pending Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_ICSR_CPRIVID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          amuIntrStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the pending AMU interrupt status
        CSL_SRIO_GetAMUPendingInterrupt (hSrio, &amuIntrStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint16*         amuIntrStatus
)
{
    *amuIntrStatus = CSL_FEXT(hSrio->RIO_AMU_ICSR, SRIO_RIO_AMU_ICSR_CPRIVID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearAMUPendingInterrupt
 *
 *   @b Description
 *   @n The function clears the AMU pending interrupts.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device          
          amuIntrStatus     AMU pending interrupt status to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_ICCR_CPRIVID
 *
 *   @b Affects
 *   @n SRIO_RIO_AMU_ICSR_CPRIVID=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          amuIntrStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the pending AMU interrupt status
        CSL_SRIO_GetAMUPendingInterrupt (hSrio, &amuIntrStatus);
        if (amuIntrStatus != 0)
        {
            // Pending AMU interrupts detected.
            ...
            // Clear the pending interrupts.
            CSL_SRIO_ClearAMUPendingInterrupt (hSrio, amuIntrStatus);
        }
        else
        {
            // No AMU interrupts were pending.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearAMUPendingInterrupt
(
    CSL_SrioHandle  hSrio, 
    Uint16          amuIntrStatus
)
{
    CSL_FINS(hSrio->RIO_AMU_ICCR, SRIO_RIO_AMU_ICCR_CPRIVID, amuIntrStatus);
}

/** ============================================================================
 *   @n@b CSL_SRIO_RouteDoorbellInterrupts
 *
 *   @b Description
 *   @n The function routes the doorbell interrupts for the specified doorbell
 *      register to a specific destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          regNum            Doorbell Register Number
          doorBellBit       Doorbell bit which is to be routed
          destination       Destination to which the interrupt bit is routed.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_DOORBELL_ICRR1_ICR0,SRIO_RIO_DOORBELL_ICRR1_ICR1,
 *      SRIO_RIO_DOORBELL_ICRR1_ICR2,SRIO_RIO_DOORBELL_ICRR1_ICR3,
 *      SRIO_RIO_DOORBELL_ICRR1_ICR4,SRIO_RIO_DOORBELL_ICRR1_ICR5,
 *      SRIO_RIO_DOORBELL_ICRR1_ICR6,SRIO_RIO_DOORBELL_ICRR1_ICR7;
 *   @n SRIO_RIO_DOORBELL_ICRR2_ICR8,SRIO_RIO_DOORBELL_ICRR2_ICR9,
 *      SRIO_RIO_DOORBELL_ICRR2_ICR10,SRIO_RIO_DOORBELL_ICRR2_ICR11,
 *      SRIO_RIO_DOORBELL_ICRR2_ICR12,SRIO_RIO_DOORBELL_ICRR2_ICR13,
 *      SRIO_RIO_DOORBELL_ICRR2_ICR14,SRIO_RIO_DOORBELL_ICRR2_ICR15
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Route the Doorbell bits 1 for Doorbell 2 to destination 2
        CSL_SRIO_RouteDoorbellInterrupts (hSrio, 2, 1, 2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_RouteDoorbellInterrupts
(
    CSL_SrioHandle  hSrio, 
    Uint8           regNum,
    Uint8           doorBellBit,
    Uint8           destination
)
{
    if (doorBellBit < 8)
    {
        /* This handles interrupts requests between 0 and 7 */
        CSL_FINSR (hSrio->DOORBELL_ICRR[regNum].RIO_DOORBELL_ICRR1, 
                   ((doorBellBit << 2) + 3), (doorBellBit << 2), destination);
    }
    else
    {
        /* This handles interrupts requests between 8 and 15 */
        doorBellBit = doorBellBit - 8;
        CSL_FINSR (hSrio->DOORBELL_ICRR[regNum].RIO_DOORBELL_ICRR2, 
                   ((doorBellBit << 2) + 3), (doorBellBit << 2), destination);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_RouteLSUInterrupts
 *
 *   @b Description
 *   @n The function routes the LSU interrupts to a specific destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsuIntrReq        LSU Interrupt request which is to be routed
          destination       Destination to which the interrupt bit is routed.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_LSU_ICRR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Route the LSU Interrupts 1 to destination 6
        CSL_SRIO_RouteLSUInterrupts (hSrio, 1, 6);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_RouteLSUInterrupts
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsuIntrReq,
    Uint8           destination
)
{
    if (lsuIntrReq < 8)
    {
        /* This handles interrupts requests between 0 and 7 */
        CSL_FINSR(hSrio->RIO_LSU0_MODULE_ICRR[0], ((lsuIntrReq << 2) + 3), (lsuIntrReq << 2), destination);
    }
    else if (lsuIntrReq < 16)
    {
        /* This handles interrupts requests between 8 and 15 */
        lsuIntrReq = lsuIntrReq - 8;
        CSL_FINSR(hSrio->RIO_LSU0_MODULE_ICRR[1], ((lsuIntrReq << 2) + 3), (lsuIntrReq << 2), destination);
    }
    else if (lsuIntrReq < 24)
    {
        /* This handles interrupts requests between 16 and 23 */
        lsuIntrReq = lsuIntrReq - 16;
        CSL_FINSR(hSrio->RIO_LSU0_MODULE_ICRR[2], ((lsuIntrReq << 2) + 3), (lsuIntrReq << 2), destination);
    }
    else
    {
        /* This handles interrupts requests between 24 and 31 */
        lsuIntrReq = lsuIntrReq - 24;
        CSL_FINSR(hSrio->RIO_LSU0_MODULE_ICRR[3], ((lsuIntrReq << 2) + 3), (lsuIntrReq << 2), destination);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_RouteErrorInterrupts
 *
 *   @b Description
 *   @n The function routes the error interrupts to a specific destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          errIntrReq        Error Interrupt request which is to be routed
          destination       Destination to which the interrupt bit is routed.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_ERR_RST_EVNT_ICRR_ICR0;SRIO_RIO_ERR_RST_EVNT_ICRR_ICR1;
 *      SRIO_RIO_ERR_RST_EVNT_ICRR_ICR2;
 *   @n SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR8;SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR9;
 *      SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR10;SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR11;
 *   @n SRIO_RIO_ERR_RST_EVNT_ICRR3_ICR16
 *
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Route the Error Interrupts 1 to destination 6
        CSL_SRIO_RouteErrorInterrupts (hSrio, 1, 6);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_RouteErrorInterrupts
(
    CSL_SrioHandle  hSrio, 
    Uint8           errIntrReq,
    Uint8           destination
)
{
    if (errIntrReq < 3)
    {
        /* This handles interrupts requests between 0 and 2 */
        CSL_FINSR(hSrio->RIO_ERR_RST_EVNT_ICRR, ((errIntrReq << 2) + 3), (errIntrReq << 2), destination);
    }
    else if (errIntrReq < 12)
    {
        /* This handles interrupts requests between 8 and 15; note that 3 to 7 are not used. */
        errIntrReq = errIntrReq - 8;
        CSL_FINSR(hSrio->RIO_ERR_RST_EVNT_ICRR2, ((errIntrReq << 2) + 3), (errIntrReq << 2), destination);
    }
    else
    {
        /* This handles interrupts request 16; all else is reserved.*/
        errIntrReq = errIntrReq - 16;
        CSL_FINSR(hSrio->RIO_ERR_RST_EVNT_ICRR3, ((errIntrReq << 2) + 3), (errIntrReq << 2), destination);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_RouteAMUInterrupts
 *
 *   @b Description
 *   @n The function routes the AMU interrupts to a specific destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          amuIntr           AMU Interrupt request which is to be routed
          destination       Destination to which the interrupt bit is routed.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_ICRR1_ICR0;SRIO_RIO_AMU_ICRR1_ICR1;SRIO_RIO_AMU_ICRR1_ICR2;
 *      SRIO_RIO_AMU_ICRR1_ICR3;SRIO_RIO_AMU_ICRR1_ICR4;SRIO_RIO_AMU_ICRR1_ICR5;
 *      SRIO_RIO_AMU_ICRR1_ICR6;SRIO_RIO_AMU_ICRR1_ICR7;
 *      SRIO_RIO_AMU_ICRR2_ICR8;SRIO_RIO_AMU_ICRR2_ICR9;SRIO_RIO_AMU_ICRR2_ICR10;
 *      SRIO_RIO_AMU_ICRR2_ICR11;SRIO_RIO_AMU_ICRR2_ICR12;SRIO_RIO_AMU_ICRR2_ICR13;
 *      SRIO_RIO_AMU_ICRR2_ICR14;SRIO_RIO_AMU_ICRR2_ICR15
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Route the AMU Interrupts 1 to destination 6
        CSL_SRIO_RouteAMUInterrupts (hSrio, 1, 6);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_RouteAMUInterrupts
(
    CSL_SrioHandle  hSrio, 
    Uint8           amuIntr,
    Uint8           destination
)
{
    if (amuIntr < 8)
    {
        /* This handles AMU interrupts requests between 0 and 7 */
        CSL_FINSR(hSrio->RIO_AMU_ICRR1, ((amuIntr << 2) + 3), (amuIntr << 2), destination);
    }
    else
    {
        /* This handles AMU interrupts requests between 8 and 15.*/
        amuIntr = amuIntr - 8;
        CSL_FINSR(hSrio->RIO_AMU_ICRR2, ((amuIntr << 2) + 3), (amuIntr << 2), destination);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDoorbellRoute
 *
 *   @b Description
 *   @n The function gets the doorbell routing information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          route             Routing Information populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           routeInfo;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Doorbell routing information
        CSL_SRIO_GetDoorbellRoute (hSrio, &routeInfo);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDoorbellRoute
(
    CSL_SrioHandle  hSrio, 
    Uint8*          routeInfo
)
{
    *routeInfo = CSL_FEXT(hSrio->RIO_INTERRUPT_CTL, SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDoorbellRoute
 *
 *   @b Description
 *   @n The function sets the doorbell routing information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          route             Routing Information to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           routeInfo;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Doorbell routing information to use dedicated interrupts.
        CSL_SRIO_SetDoorbellRoute (hSrio, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDoorbellRoute
(
    CSL_SrioHandle  hSrio, 
    Uint8           routeInfo
)
{
    hSrio->RIO_INTERRUPT_CTL = CSL_FMK(SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE, routeInfo);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetInterruptStatusDecode
 *
 *   @b Description
 *   @n The function gets the interrupt status for a specified interrupt 
 *      destination
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          intrDst           Interrupt Destination for which the status is required.
          status            Decode Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_INTDST_DECODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          status;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Interrupt Status for destination 0
        CSL_SRIO_GetInterruptStatusDecode (hSrio, 0, &status);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetInterruptStatusDecode
(
    CSL_SrioHandle  hSrio, 
    Uint8           intrDst,
    Uint32*         status
)
{
    *status = hSrio->RIO_INTDST_DECODE[intrDst];
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableInterruptPacing
 *
 *   @b Description
 *   @n The function enables interrupt pacing for the specific destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          destination       Destination Address for which interrupt pacing is enabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;
 *   SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_IO_INTDST_RATE_DIS_RATEN_DIS=0;
 *   SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;
 *   SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=0;
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          rate;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Interrupt Pacing for Destination 4.
        CSL_SRIO_EnableInterruptPacing (hSrio, 4);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableInterruptPacing
(
    CSL_SrioHandle  hSrio, 
    Uint8           destination
)
{
    CSL_FINSR(hSrio->RIO_INTDST_RATE_DIS, destination, destination, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableInterruptPacing
 *
 *   @b Description
 *   @n The function disables interrupt pacing for the specific destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          destination       Destination Address for which interrupt pacing is disabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;
 *   SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;
 *   SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;
 *   SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1;SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS=1
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          rate;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Interrupt Pacing for Destination 5.
        CSL_SRIO_DisableInterruptPacing (hSrio, 5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableInterruptPacing
(
    CSL_SrioHandle  hSrio, 
    Uint8           destination
)
{
    CSL_FINSR(hSrio->RIO_INTDST_RATE_DIS, destination, destination, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetInterruptRate
 *
 *   @b Description
 *   @n The function sets the rate for each interrupt destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          destination       Destination Address for which the status is read.
          rate              Interrupt Rate to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_INTDST_RATE_CNT_COUNT_DOWN_VALUE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Interrupt Pacing for Destination 2.
        CSL_SRIO_EnableInterruptPacing (hSrio, 2);
        
        // Set the slowest Interrupt Rate for destination 2.
        CSL_SRIO_SetInterruptRate (hSrio, 2, 0xFFFFFFFF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetInterruptPacing
(
    CSL_SrioHandle  hSrio, 
    Uint8           destination,
    Uint32          rate
)
{
    hSrio->RIO_INTDST_RATE_CNT[destination] = rate; 
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetInterruptPacing
 *
 *   @b Description
 *   @n The function gets the rate for each interrupt destination.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          destination       Destination Address for which the status is read.
          rate              Interrupt Rate to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_INTDST_RATE_CNT_COUNT_DOWN_VALUE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          rate;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Interrupt Pacing for Destination 4.
        CSL_SRIO_EnableInterruptPacing (hSrio, 4);
        ...
        // Get the Interrupt rate for destination 4
        CSL_SRIO_GetInterruptRate (hSrio, 4, &rate);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetInterruptRate
(
    CSL_SrioHandle  hSrio, 
    Uint8           destination,
    Uint32*         rate
)
{
    *rate = hSrio->RIO_INTDST_RATE_CNT[destination];
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetType11PSInfo
 *
 *   @b Description
 *   @n The function creates the type11 protocol specific information.
 *
 *   @b Arguments
     @verbatim
        psInfo      - 8 bytes of PS information populated by this API
        dstID       - Destination Identifier.
        srcId       - Source Identifier.
        mbox        - Mail Box
        ltr         - Letter 
        tt          - Identifies if 8 bit or 16 bit identifiers are to be used.
        ssize       - Standard Message Payload Size
        retryCount  - Total Number of retries allowed for this message.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          psInfo[2];

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
       
        // Create the Protocol Specific Information.
        CSL_SRIO_SetType11PSInfo(psInfo, 0xDEAD, 0xBEED, 0x1, 0x2, 0x1, 0xe, 0x0);

        // Set the PS Information into the Buffer Descriptor.
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetType11PSInfo
(
    Uint32  psInfo[2],
    Uint16  dstId,
    Uint16  srcId,
    Uint8   mbox,
    Uint8   letter,
    Uint8   tt,
    Uint8   ssize,
    Uint8   retryCount
)
{
    psInfo[0] = CSL_FMKR(31, 16, srcId) |
                CSL_FMKR(15, 0,  dstId);

    psInfo[1] = CSL_FMKR(5,  0,  mbox)      |
                CSL_FMKR(8,  6,  letter)    |
                CSL_FMKR(10, 9,  tt)        |
                CSL_FMKR(20, 17, ssize)     |
                CSL_FMKR(26, 21, retryCount);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ExtractType11PSInfo
 *
 *   @b Description
 *   @n The function extracts the type11 fields from the Protocol Specific 
 *      information.
 *
 *   @b Arguments
     @verbatim
        psInfo      - 8 bytes of PS information from the descriptor
        dstID       - Destination Identifier populated by the API
        srcId       - Source Identifier populated by the API
        mbox        - Mail Box populated by the API
        ltr         - Letter populated by the API
        tt          - Identifies if 8 bit or 16 bit identifiers are being used.
        pri         - Message Priority populated by the API
        cc          - Completion Code populated by the API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          psInfo[2];
        Uint16          dstId;
        Uint16          srcId;
        Uint8           mbox;
        Uint8           letter;
        Uint8           tt;
        Uint8           pri;
        Uint8           cc;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PS Information from the buffer descriptor.
        ...

        // Populate the Type11 fields from the PS Information.
        CSL_SRIO_ExtractType11PSInfo(psInfo, &dstId, &srcId, &mbox, &letter, &tt, &pri, &cc);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ExtractType11PSInfo
(
    Uint32  psInfo[2],
    Uint16* dstId,
    Uint16* srcId,
    Uint8*  mbox,
    Uint8*  letter,
    Uint8*  tt,
    Uint8*  pri,
    Uint8*  cc
)
{
    *srcId    = CSL_FEXTR(psInfo[0], 31, 16);
    *dstId    = CSL_FEXTR(psInfo[0], 15, 0);
    *mbox     = CSL_FEXTR(psInfo[1], 5,  0);
    *letter   = CSL_FEXTR(psInfo[1], 8,  6);
    *tt       = CSL_FEXTR(psInfo[1], 10, 9);
    *pri      = CSL_FEXTR(psInfo[1], 14, 11);
    *cc       = CSL_FEXTR(psInfo[1], 16, 15);
}

/** ============================================================================
 *   @n@b CSL_SRIO_MapMessageToQueue
 *
 *   @b Description
 *   @n The function maps a specific RIO message to the specific queue
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the MAP register 
          message           Message Information which is compared with incoming
                            packet.
          queueId           Destination Queue to which the message is sent
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_RXU_MAP_L_SRCID,SRIO_RIO_RXU_MAP_L_MBX,SRIO_RIO_RXU_MAP_L_LTR
 *      SRIO_RIO_RXU_MAP_L_MBX_MASK,SRIO_RIO_RXU_MAP_L_LTR_MASK,
 *   @n SRIO_RIO_RXU_MAP_H_SEG_MAP,SRIO_RIO_RXU_MAP_H_SRC_PROM,SRIO_RIO_RXU_MAP_H_TT,
 *      SRIO_RIO_RXU_MAP_H_DEST_PROM,SRIO_RIO_RXU_MAP_H_DEST_ID,
 *   @n SRIO_RIO_RXU_MAP_QID_DEST_QID,SRIO_RIO_RXU_MAP_QID_FLOWID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_MESSAGE    message;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        
        message.srcID   = 0xBEEF;
        message.mbx     = 0;
        message.ltr     = 0;
        message.mbxMask = 0;
        message.ltrMask = 0;
        message.segMap  = 0;     // Single Segment 
        message.srcProm = 1;     // Full Access to the Queue for any Source ID
        message.tt      = 1;     // Match 16bits for Source ID.
        message.dstProm = 0;     // Access to the Queue for the specified Destination ID
        message.dstId   = 0xDEAD;// The destination ID which indicates us.
        message.flowId  = 0x0;
               
        // Map the above message to Queue 4. This is the first mapping which is being done
        CSL_SRIO_MapMessageToQueue (hSrio, 0, &message, 4);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_MapMessageToQueue
(
    CSL_SrioHandle  hSrio, 
    Uint8           index,
    SRIO_MESSAGE*   ptrMessage,
    Uint16          queueId
)
{
    /* Initialize the RXU MAPL register. */
    hSrio->RXU_MAP[index].RIO_RXU_MAP_L = 
            CSL_FMK(SRIO_RIO_RXU_MAP_L_SRCID, ptrMessage->srcId)            |
            CSL_FMK(SRIO_RIO_RXU_MAP_L_MBX,   ptrMessage->mbx)              |
            CSL_FMK(SRIO_RIO_RXU_MAP_L_LTR,   ptrMessage->ltr)              |
            CSL_FMK(SRIO_RIO_RXU_MAP_L_MBX_MASK, ptrMessage->mbxMask)       |
            CSL_FMK(SRIO_RIO_RXU_MAP_L_LTR_MASK, ptrMessage->ltrMask);

    /* Initialize the RXU MAPH register. */
    hSrio->RXU_MAP[index].RIO_RXU_MAP_H =  
            CSL_FMK(SRIO_RIO_RXU_MAP_H_SEG_MAP, ptrMessage->segMap)  |
            CSL_FMK(SRIO_RIO_RXU_MAP_H_SRC_PROM, ptrMessage->srcProm)       |
            CSL_FMK(SRIO_RIO_RXU_MAP_H_TT, ptrMessage->tt)                  |
            CSL_FMK(SRIO_RIO_RXU_MAP_H_DEST_PROM, ptrMessage->dstProm)      |
            CSL_FMK(SRIO_RIO_RXU_MAP_H_DEST_ID, ptrMessage->dstId);

    /* Initialize the RXU QID register. */
    hSrio->RXU_MAP[index].RIO_RXU_MAP_QID = 
            CSL_FMK(SRIO_RIO_RXU_MAP_QID_DEST_QID, queueId)                 |
            CSL_FMK(SRIO_RIO_RXU_MAP_QID_FLOWID, ptrMessage->flowId);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetMessageToQueueMapping
 *
 *   @b Description
 *   @n The function gets information about the SRIO Message and Queue to which
 *      it is mapped by reading the specified MAP 'index' register.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the MAP register 
          message           Message Information populated by this API
          queueId           Destination Queue to which the message is mapped 
                            populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_RXU_MAP_L_SRCID,SRIO_RIO_RXU_MAP_L_MBX,SRIO_RIO_RXU_MAP_L_LTR
 *      SRIO_RIO_RXU_MAP_L_MBX_MASK,SRIO_RIO_RXU_MAP_L_LTR_MASK,
 *   @n SRIO_RIO_RXU_MAP_H_SEG_MAP,SRIO_RIO_RXU_MAP_H_SRC_PROM,SRIO_RIO_RXU_MAP_H_TT,
 *      SRIO_RIO_RXU_MAP_H_DEST_PROM,SRIO_RIO_RXU_MAP_H_DEST_ID,
 *   @n SRIO_RIO_RXU_MAP_QID_DEST_QID,SRIO_RIO_RXU_MAP_QID_FLOWID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_MESSAGE    message;
        Uint16          queueId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the mapping information for the 'first' mapping.
        CSL_SRIO_GetMessageToQueueMapping (hSrio, 0, &message, &queueId);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetMessageToQueueMapping
(
    CSL_SrioHandle  hSrio, 
    Uint8           index,
    SRIO_MESSAGE*   ptrMessage,
    Uint16*         queueId
)
{
    Uint32  value;

    /* Get the RXU MAPL register. */
    value = hSrio->RXU_MAP[index].RIO_RXU_MAP_L;
    ptrMessage->srcId   = CSL_FEXT(value, SRIO_RIO_RXU_MAP_L_SRCID);
    ptrMessage->mbx     = CSL_FEXT(value, SRIO_RIO_RXU_MAP_L_MBX);
    ptrMessage->ltr     = CSL_FEXT(value, SRIO_RIO_RXU_MAP_L_LTR);
    ptrMessage->mbxMask = CSL_FEXT(value, SRIO_RIO_RXU_MAP_L_MBX_MASK);
    ptrMessage->ltrMask = CSL_FEXT(value, SRIO_RIO_RXU_MAP_L_LTR_MASK);

    /* Get the RXU MAPH register. */
    value = hSrio->RXU_MAP[index].RIO_RXU_MAP_H;
    ptrMessage->segMap  = CSL_FEXT(value, SRIO_RIO_RXU_MAP_H_SEG_MAP);
    ptrMessage->srcProm = CSL_FEXT(value, SRIO_RIO_RXU_MAP_H_SRC_PROM);
    ptrMessage->tt      = CSL_FEXT(value, SRIO_RIO_RXU_MAP_H_TT);
    ptrMessage->dstProm = CSL_FEXT(value, SRIO_RIO_RXU_MAP_H_DEST_PROM);
    ptrMessage->dstId   = CSL_FEXT(value, SRIO_RIO_RXU_MAP_H_DEST_ID);

    /* Get the RXU QID register. */
    value = hSrio->RXU_MAP[index].RIO_RXU_MAP_QID;
    *queueId = CSL_FEXT(value, SRIO_RIO_RXU_MAP_QID_DEST_QID);
    ptrMessage->flowId = CSL_FEXT(value, SRIO_RIO_RXU_MAP_QID_FLOWID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetType9PSInfo
 *
 *   @b Description
 *   @n The function creates the type9 protocol specific information.
 *
 *   @b Arguments
     @verbatim
        psInfo      - 8 bytes of PS information populated by this API
        dstID       - Destination Identifier.
        srcId       - Source Identifier.
        cos         - Class of Service 
        cosMask     - Class of Service Mask
        tt          - Identifies if 8 bit or 16 bit identifiers are to be used.
        streamId    - Stream ID for the transaction
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          psInfo[2];

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
       
        // Create the Protocol Specific Information.
        CSL_SRIO_SetType9PSInfo(psInfo, 0xDEAD, 0xBEED, 0x4, 0x1, 0x1, 0xABCD);

        // Set the PS Information into the Buffer Descriptor.
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetType9PSInfo
(
    Uint32  psInfo[2],
    Uint16  dstId,
    Uint16  srcId,
    Uint8   cos,
    Uint8   cosMask,
    Uint8   tt,
    Uint16  streamId
)
{
    psInfo[0] = CSL_FMKR(31, 16, srcId)     |
                CSL_FMKR(15, 0,  dstId);

    psInfo[1] = CSL_FMKR(7,  0,  cos)       |
                CSL_FMKR(8,  8,  cosMask)   |
                CSL_FMKR(10, 9,  tt)        |
                CSL_FMKR(31, 16, streamId);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ExtractType9PSInfo
 *
 *   @b Description
 *   @n The function extracts the type9 protocol specific information.
 *
 *   @b Arguments
     @verbatim
        psInfo      - 8 bytes of PS information
        dstID       - Destination Identifier populated by this API
        srcId       - Source Identifier populated by this API
        cos         - Class of Service populated by this API
        cc          - Completion Code populated by this API
        tt          - Identifies if 8 bit or 16 bit identifiers are to be used.
        pri         - Message Priority populated by this API
        streamId    - Stream Id populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          psInfo[2];
        Uint16          dstId;
        Uint16          srcId;
        Uint8           cos;
        Uint8           cc;
        Uint8           tt;
        Uint8           pri;
        Uint16          streamId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PS Information into the Buffer Descriptor.
        ...

        // Extract the Type9 fields from the PS Information.
        CSL_SRIO_ExtractType9PSInfo(psInfo, &dstId, &srcId, &cos, &cc, &tt, &pri, &streamId);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ExtractType9PSInfo
(
    Uint32  psInfo[2],
    Uint16* dstId,
    Uint16* srcId,
    Uint8*  cos,
    Uint8*  cc,
    Uint8*  tt,
    Uint8*  pri,
    Uint16* streamId
)
{
    *srcId    = CSL_FEXTR(psInfo[0], 31, 16);
    *dstId    = CSL_FEXTR(psInfo[0], 15, 0);
    *cos      = CSL_FEXTR(psInfo[1], 7,  0);
    *cc       = CSL_FEXTR(psInfo[1], 9,  8);
    *tt       = CSL_FEXTR(psInfo[1], 10, 10);
    *pri      = CSL_FEXTR(psInfo[1], 14, 11);
    *streamId = CSL_FEXTR(psInfo[1], 31, 16);
}

/** ============================================================================
 *   @n@b CSL_SRIO_MapType9MessageToQueue
 *
 *   @b Description
 *   @n The function maps the specific TYPE9 message to the queue
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the MAP register 
          message           Type9-Message Information 
          queueId           Destination Queue 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_RXU_TYPE9_MAP0_SRCID,SRIO_RIO_RXU_TYPE9_MAP0_COS,
 *      SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK,
 *   @n SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM,SRIO_RIO_RXU_TYPE9_MAP1_TT,
 *      SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM,SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID,
 *   @n SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID,SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK,
 *   @n SRIO_RIO_RXU_MAP_QID_DEST_QID,SRIO_RIO_RXU_MAP_QID_FLOWID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_TYPE9_MESSAGE  message;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        message.srcId      = 0xBEEF;
        message.cos        = 0;
        message.cosMask    = 0;
        message.srcProm    = 0;
        message.tt         = 1; // Match 16bits for Source ID.
        message.dstProm    = 0;
        message.dstId      = 0xDEAD;
        message.streamId   = 0;
        message.streamMask = 0;
        message.flowId     = 0;
               
        // Map the above message to Queue 5. This is the first mapping which is being done
        CSL_SRIO_MapType9MessageToQueue (hSrio, 0, &message, 5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_MapType9MessageToQueue
(
    CSL_SrioHandle        hSrio, 
    Uint8                 index,
    SRIO_TYPE9_MESSAGE*   ptrMessage,
    Uint16                queueId
)
{
    /* Initialize the RXU TYPE9 MAP0 register. */
    hSrio->RXU_TYPE9_MAP[index].RIO_RXU_TYPE9_MAP0 = 
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP0_SRCID,    ptrMessage->srcId)     |
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP0_COS,      ptrMessage->cos)       |
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK, ptrMessage->cosMask);

    /* Initialize the RXU TYPE9 MAP1 register. */
    hSrio->RXU_TYPE9_MAP[index].RIO_RXU_TYPE9_MAP1 = 
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM, ptrMessage->srcProm)   |
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP1_TT,       ptrMessage->tt)        |
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM,ptrMessage->dstProm)   |
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID,  ptrMessage->dstId);

    /* Initialize the RXU TYPE9 MAP2 register. */
    hSrio->RXU_TYPE9_MAP[index].RIO_RXU_TYPE9_MAP2 =
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID,  ptrMessage->streamId)  |
                CSL_FMK(SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK,ptrMessage->streamMask);

    /* Initialize the RXU QID register: The specification states that this is shared between Type9 and Type11 */
    hSrio->RXU_MAP[index].RIO_RXU_MAP_QID = 
            CSL_FMK(SRIO_RIO_RXU_MAP_QID_DEST_QID, queueId)                      |
            CSL_FMK(SRIO_RIO_RXU_MAP_QID_FLOWID, ptrMessage->flowId);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetType9MessageToQueueMapping
 *
 *   @b Description
 *   @n The function gets information about the SRIO TYPE9 Message and Queue 
 *      to which it is mapped by reading the specified MAP 'index' register.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             Index of the MAP register 
          message           Type9-Message Information populated by this API 
          queueId           Destination Queue populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_RXU_TYPE9_MAP0_SRCID,SRIO_RIO_RXU_TYPE9_MAP0_COS,
 *      SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK,
 *   @n SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM,SRIO_RIO_RXU_TYPE9_MAP1_TT,
 *      SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM,SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID,
 *   @n SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID,SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK,
 *   @n SRIO_RIO_RXU_MAP_QID_DEST_QID,SRIO_RIO_RXU_MAP_QID_FLOWID   
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_TYPE9_MESSAGE  message;
        Uint16              queueId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the first mapping information.
        CSL_SRIO_GetType9MessageToQueueMapping (hSrio, 0, &message, &queueId);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetType9MessageToQueueMapping
(
    CSL_SrioHandle        hSrio, 
    Uint8                 index,
    SRIO_TYPE9_MESSAGE*   ptrMessage,
    Uint16*               queueId
)
{
    Uint32  value;

    /* Get the RXU TYPE9 MAP0 register. */
    value = hSrio->RXU_TYPE9_MAP[index].RIO_RXU_TYPE9_MAP0;
    ptrMessage->srcId   = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP0_SRCID);
    ptrMessage->cos     = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP0_COS);
    ptrMessage->cosMask = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK);

    /* Get the RXU TYPE9 MAP1 register. */
    value = hSrio->RXU_TYPE9_MAP[index].RIO_RXU_TYPE9_MAP1;
    ptrMessage->srcProm = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM);
    ptrMessage->tt      = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP1_TT);
    ptrMessage->dstProm = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM);
    ptrMessage->dstId   = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID);

    /* Get the RXU TYPE9 MAP2 register. */
    value = hSrio->RXU_TYPE9_MAP[index].RIO_RXU_TYPE9_MAP2;
    ptrMessage->streamId   = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID);
    ptrMessage->streamMask = CSL_FEXT(value, SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK);

    /* Get the RXU QID register: This is shared between the Type9 and Type11 */
    value = hSrio->RXU_MAP[index].RIO_RXU_MAP_QID;
    *queueId = CSL_FEXT(value, SRIO_RIO_RXU_MAP_QID_DEST_QID);
    ptrMessage->flowId = CSL_FEXT(value, SRIO_RIO_RXU_MAP_QID_FLOWID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_MapAMUPrivIdToSourceId
 *
 *   @b Description
 *   @n The function maps the AMU Priv ID to a specific Source ID.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          privId            AMU Priv ID which is being mapped
          srcId             Source ID to which the priv ID is being mapped to.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID0_8;SRIO_RIO_AMU_SRCID_MAP_CPRIVID1_9; 
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID2_10;SRIO_RIO_AMU_SRCID_MAP_CPRIVID3_11;
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID4_12;SRIO_RIO_AMU_SRCID_MAP_CPRIVID5_13;
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID6_14;SRIO_RIO_AMU_SRCID_MAP_CPRIVID7_15;
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Map the PRIV-ID 1 to Source ID 3
        CSL_SRIO_MapAMUPrivIdToSourceId (hSrio, 1, 5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_MapAMUPrivIdToSourceId
(
    CSL_SrioHandle  hSrio, 
    Uint8           privId,
    Uint8           srcId
)
{        
    if (privId < 8)
    {
        CSL_FINSR (hSrio->RIO_AMU_SRCID_MAP[0], (privId << 2) + 3, (privId << 2), srcId);
    }
    else
    {
        privId = privId - 8;
        CSL_FINSR (hSrio->RIO_AMU_SRCID_MAP[1], (privId << 2) + 3, (privId << 2), srcId);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUPrivIdToSourceIdMapping
 *
 *   @b Description
 *   @n The function gets the mapping of a specific AMU privID to source Id
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          privId            AMU Priv ID for which the mapping is required.
          srcId             Source ID to which the priv ID has been mapped to
                            populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID0_8;SRIO_RIO_AMU_SRCID_MAP_CPRIVID1_9; 
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID2_10;SRIO_RIO_AMU_SRCID_MAP_CPRIVID3_11;
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID4_12;SRIO_RIO_AMU_SRCID_MAP_CPRIVID5_13;
 *   @n SRIO_RIO_AMU_SRCID_MAP_CPRIVID6_14;SRIO_RIO_AMU_SRCID_MAP_CPRIVID7_15;
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               srcId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the mapping for Priv ID 1
        CSL_SRIO_GetAMUPrivIdToSourceIdMapping (hSrio, 1, &srcId);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUPrivIdToSourceIdMapping
(
    CSL_SrioHandle  hSrio, 
    Uint8           privId,
    Uint8*          srcId
)
{
    if (privId < 8)
    {
        *srcId = CSL_FEXTR (hSrio->RIO_AMU_SRCID_MAP[0], (privId << 2) + 3, (privId << 2));
    }
    else
    {
        privId = privId - 8;
        *srcId = CSL_FEXTR (hSrio->RIO_AMU_SRCID_MAP[1], (privId << 2) + 3, (privId << 2));
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_MapVBUSPriorityToRIO
 *
 *   @b Description
 *   @n The function maps the VBUS Priority to RIO Priority.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          vbusPriority      VBUS Priority which is to be mapped
          priority          RIO Priority to be mapped to.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI0;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI1;
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI2;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI3;
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI4;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI5;
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI6;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI7
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Map the VBUS Priority 0 to CRITICAL Flow High Priority i.e. 7
        CSL_SRIO_MapVBUSPriorityToRIO (hSrio, 0, 7);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_MapVBUSPriorityToRIO
(
    CSL_SrioHandle  hSrio, 
    Uint8           vbusPriority,
    Uint8           priority
)
{
    CSL_FINSR (hSrio->RIO_AMU_PRIORITY_MAP, (vbusPriority << 2) + 3, (vbusPriority << 2), priority);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetVBUSPriorityToRIOMapping
 *
 *   @b Description
 *   @n The function gets the mapping between the VBUS Priority to RIO Priority.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          vbusPriority      VBUS Priority for which the mapping is required.
          priority          Priority level populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI0;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI1;
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI2;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI3;
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI4;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI5;
 *   @n SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI6;SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI7
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               priority;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the RIO Priority Level for VBUS Priority 0
        CSL_SRIO_GetVBUSPriorityToRIOMapping (hSrio, 0, &priority);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetVBUSPriorityToRIOMapping
(
    CSL_SrioHandle  hSrio,
    Uint8           vbusPriority,
    Uint8*          priority
)
{
    *priority = CSL_FEXTR (hSrio->RIO_AMU_PRIORITY_MAP, (vbusPriority << 2) + 3, (vbusPriority << 2));
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUErrorCaptureInfo
 *
 *   @b Description
 *   @n The function is used to get access to the AMU Error capture information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          transAddress      Transaction Address which caused the error.
          privId            Priv ID which caused the error
          mstID             Master ID which caused the error.
          doorbellInfo      Doorbell Info
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_CAPT0_MAP,SRIO_RIO_AMU_CAPT1_MAP_CPRIVID,
 *      SRIO_RIO_AMU_CAPT1_MAP_CMSTID,SRIO_RIO_AMU_CAPT1_MAP_DOORBELL_INFO
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              transAddress;
        Uint8               privId;
        Uint8               mstID;
        Uint16              doorbellInfo;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the AMU Error Capture Information
        CSL_SRIO_GetAMUErrorCaptureInfo (hSrio, &transAddress, &privId, &mstID, &doorbellInfo);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUErrorCaptureInfo
(
    CSL_SrioHandle  hSrio,
    Uint32*         transAddress,
    Uint8*          privId,
    Uint8*          mstID,
    Uint16*         doorbellInfo
)
{
    *transAddress = hSrio->RIO_AMU_CAPT0_MAP;
    *privId = CSL_FEXT (hSrio->RIO_AMU_CAPT1_MAP, SRIO_RIO_AMU_CAPT1_MAP_CPRIVID);
    *mstID = CSL_FEXT (hSrio->RIO_AMU_CAPT1_MAP, SRIO_RIO_AMU_CAPT1_MAP_CMSTID);
    *doorbellInfo = CSL_FEXT (hSrio->RIO_AMU_CAPT1_MAP, SRIO_RIO_AMU_CAPT1_MAP_DOORBELL_INFO);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetAMUWindow
 *
 *   @b Description
 *   @n The function configures the specific AMU Window register
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             AMU Window to be configured
          amuWindow         AMU Window Properties to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_WINDOW_REG0_XAMBS,SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT,
 *      SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE,SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE,
 *      SRIO_RIO_AMU_WINDOW_REG1,SRIO_RIO_AMU_WINDOW_REG2
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_AMU_WINDOW     amuWindow;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure the AMU Window Properties for AMU Window 1
        CSL_SRIO_SetAMUWindow (hSrio, 1, &amuWindow);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetAMUWindow
(
    CSL_SrioHandle      hSrio, 
    Uint8               index,
    SRIO_AMU_WINDOW*    amuWindow
)
{
    /* Configure the Window0 Register. */
    hSrio->AMU_WINDOW[index].RIO_AMU_WINDOW_REG0 = 
                CSL_FMK (SRIO_RIO_AMU_WINDOW_REG0_XAMBS,      amuWindow->xambs)     |
                CSL_FMK (SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT, amuWindow->paneCount) |
                CSL_FMK (SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE,  amuWindow->paneSize)  |
                CSL_FMK (SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE,amuWindow->winSize);

    /* Configure the WINDOW1 and WINDOW2 registers. */
    hSrio->AMU_WINDOW[index].RIO_AMU_WINDOW_REG1 = amuWindow->rapidIOMsb;
    hSrio->AMU_WINDOW[index].RIO_AMU_WINDOW_REG2 = amuWindow->rapidIOLsb;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUWindow
 *
 *   @b Description
 *   @n The function gets the specific AMU Window register
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             AMU Window Index 
          amuWindow         AMU Window Properties populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_WINDOW_REG0_XAMBS,SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT,
 *      SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE,SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE,
 *      SRIO_RIO_AMU_WINDOW_REG1,SRIO_RIO_AMU_WINDOW_REG2
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_AMU_WINDOW     amuWindow;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the AMU Window Properties for AMU Window 1
        CSL_SRIO_GetAMUWindow (hSrio, 1, &amuWindow);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUWindow
(
    CSL_SrioHandle      hSrio, 
    Uint8               index,
    SRIO_AMU_WINDOW*    amuWindow
)
{
    Uint32 value = hSrio->AMU_WINDOW[index].RIO_AMU_WINDOW_REG0;

    /* Get the Window0 Register. */
    amuWindow->xambs     = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_REG0_XAMBS);
    amuWindow->paneCount = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT);
    amuWindow->paneSize  = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE);
    amuWindow->winSize   = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE);

    /* Get the WINDOW1 and WINDOW2 registers. */
    amuWindow->rapidIOMsb = hSrio->AMU_WINDOW[index].RIO_AMU_WINDOW_REG1;
    amuWindow->rapidIOLsb = hSrio->AMU_WINDOW[index].RIO_AMU_WINDOW_REG2;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetAMUPane
 *
 *   @b Description
 *   @n The function configures the specific AMU Pane
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             AMU Pane to be configured
          amuPane           AMU Pane Properties to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC,SRIO_RIO_AMU_WINDOW_PANE_PORT_ID,
 *      SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE,SRIO_RIO_AMU_WINDOW_PANE_DESTID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_AMU_PANE       amuPane;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure the AMU Pane Properties for AMU Pane 1
        CSL_SRIO_SetAMUPane (hSrio, 1, &amuPane);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetAMUPane
(
    CSL_SrioHandle      hSrio, 
    Uint8               index,
    SRIO_AMU_PANE*      amuPane
)
{
    /* Configure the Pane Register. */
    hSrio->RIO_AMU_WINDOW_PANE[index] = 
            CSL_FMK (SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC,amuPane->cmdEnc)    |
            CSL_FMK (SRIO_RIO_AMU_WINDOW_PANE_PORT_ID,amuPane->portId)    |
            CSL_FMK (SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE,amuPane->idSize)    |
            CSL_FMK (SRIO_RIO_AMU_WINDOW_PANE_DESTID, amuPane->dstID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUPane
 *
 *   @b Description
 *   @n The function gets the specific AMU Pane properties
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             AMU Pane Index
          amuPane           AMU Pane Properties populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC,SRIO_RIO_AMU_WINDOW_PANE_PORT_ID,
 *      SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE,SRIO_RIO_AMU_WINDOW_PANE_DESTID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_AMU_PANE       amuPane;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the AMU Pane Properties for AMU Pane 2
        CSL_SRIO_GetAMUPane (hSrio, 2, &amuPane);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUPane
(
    CSL_SrioHandle      hSrio, 
    Uint8               index,
    SRIO_AMU_PANE*      amuPane
)
{
    Uint32 value = hSrio->RIO_AMU_WINDOW_PANE[index];

    /* Get the Pane properties. */
    amuPane->cmdEnc = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC);
    amuPane->portId = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_PANE_PORT_ID);
    amuPane->idSize = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE);
    amuPane->dstID  = CSL_FEXT (value, SRIO_RIO_AMU_WINDOW_PANE_DESTID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetAMUFlowMask
 *
 *   @b Description
 *   @n The function configures the AMU Flow Mask
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          flowMask          Flow Mask to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure the AMU Flow Mask
        CSL_SRIO_SetAMUFlowMask (hSrio, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetAMUFlowMask
(
    CSL_SrioHandle      hSrio, 
    Uint16              flowMask
)
{
    hSrio->RIO_AMU_FLOW_MASKS0 = CSL_FMK(SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK, flowMask);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAMUFlowMask
 *
 *   @b Description
 *   @n The function gets the AMU Flow Mask
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          flowMask          Flow Mask to be populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              flowMask;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the AMU Flow Mask
        CSL_SRIO_GetAMUFlowMask (hSrio, &flowMask);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAMUFlowMask
(
    CSL_SrioHandle      hSrio, 
    Uint16*             flowMask
)
{
    *flowMask = CSL_FEXT(hSrio->RIO_AMU_FLOW_MASKS0, SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUTransfer
 *
 *   @b Description
 *   @n The function sets the specified LSU for a DIO Transfer.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number which is to be used for the transfer.
          lsuTransfer       LSU Transfer Information 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG0_RIO_ADDRESS_MSB,
 *   @n SRIO_RIO_LSU_REG1_RIO_ADDRESS_LSB,
 *   @n SRIO_RIO_LSU_REG2_DSP_ADDRESS,
 *   @n SRIO_RIO_LSU_REG3_BYTE_COUNT,SRIO_RIO_LSU_REG3_DRBLL_VALUE,
 *   @n SRIO_RIO_LSU_REG4_INT_REQ,SRIO_RIO_LSU_REG4_SUP_GINT,SRIO_RIO_LSU_REG4_XAMBS,
 *   @n SRIO_RIO_LSU_REG4_PRIORITY,SRIO_RIO_LSU_REG4_OUTPORTID,SRIO_RIO_LSU_REG4_ID_SIZE,
 *   @n SRIO_RIO_LSU_REG4_SRCID_MAP,SRIO_RIO_LSU_REG4_DESTID,
 *   @n SRIO_RIO_LSU_REG5_TTYPE,SRIO_RIO_LSU_REG5_FTYPE,SRIO_RIO_LSU_REG5_HOP_COUNT,
 *      SRIO_RIO_LSU_REG5_DRBLL_INFO
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        SRIO_LSU_TRANSFER  lsuTransfer;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Populate the Transfer Information. 
        lsuTransfer.rapidIOMSB    = 0x0;
        lsuTransfer.rapidIOLSB    = (Uint32)&tx_buffer[0];
        lsuTransfer.dspAddress    = (Uint32)&rx_buffer[0];
        lsuTransfer.bytecount     = 256;
        lsuTransfer.doorbellValid = 0;
        lsuTransfer.intrRequest   = 1;
        lsuTransfer.supInt        = 0;
        lsuTransfer.xambs         = 0;
        lsuTransfer.priority      = 2;
        lsuTransfer.outPortID     = 1;
        lsuTransfer.idSize        = 1;
        lsuTransfer.srcIDMap      = 0;
        lsuTransfer.dstID         = 0xDEAD;
        lsuTransfer.ttype         = 4;
        lsuTransfer.ftype         = 5;
        lsuTransfer.hopCount      = 0;
        lsuTransfer.doorbellInfo  = 0;

        // Send the transfer on LSU 1 
        CSL_SRIO_SetLSUTransfer (hSrio, 1, &lsuTransfer);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUTransfer
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    SRIO_LSU_TRANSFER*  lsuTransfer
)
{
    /* Configure the Source and Destination Address. */
    hSrio->LSU_CMD[lsu].RIO_LSU_REG0 = lsuTransfer->rapidIOMSB;
    hSrio->LSU_CMD[lsu].RIO_LSU_REG1 = lsuTransfer->rapidIOLSB;
    hSrio->LSU_CMD[lsu].RIO_LSU_REG2 = lsuTransfer->dspAddress;

    /* Configure the number of bytes which need to be transferred. */
    hSrio->LSU_CMD[lsu].RIO_LSU_REG3 = 
                CSL_FMK (SRIO_RIO_LSU_REG3_BYTE_COUNT, lsuTransfer->bytecount)      |
                CSL_FMK (SRIO_RIO_LSU_REG3_DRBLL_VALUE,lsuTransfer->doorbellValid);

    /* Configure outgoing port ID, Priority and other RIO parameters. */
    hSrio->LSU_CMD[lsu].RIO_LSU_REG4 = 
            CSL_FMK(SRIO_RIO_LSU_REG4_INT_REQ,   lsuTransfer->intrRequest)  |
            CSL_FMK(SRIO_RIO_LSU_REG4_SRCID_MAP, lsuTransfer->srcIDMap)     |
            CSL_FMK(SRIO_RIO_LSU_REG4_SUP_GINT,  lsuTransfer->supInt)       |
            CSL_FMK(SRIO_RIO_LSU_REG4_DESTID,    lsuTransfer->dstID)        |
            CSL_FMK(SRIO_RIO_LSU_REG4_ID_SIZE,   lsuTransfer->idSize)       |
            CSL_FMK(SRIO_RIO_LSU_REG4_XAMBS,     lsuTransfer->xambs)        |
            CSL_FMK(SRIO_RIO_LSU_REG4_PRIORITY,  lsuTransfer->priority)     |
            CSL_FMK(SRIO_RIO_LSU_REG4_OUTPORTID, lsuTransfer->outPortID);

    /* Configure the type of request, doorbell information etc. */
    hSrio->LSU_CMD[lsu].RIO_LSU_REG5 = 
            CSL_FMK(SRIO_RIO_LSU_REG5_TTYPE,      lsuTransfer->ttype)       |
		    CSL_FMK(SRIO_RIO_LSU_REG5_FTYPE,      lsuTransfer->ftype)       |
            CSL_FMK(SRIO_RIO_LSU_REG5_HOP_COUNT,  lsuTransfer->hopCount)    |
            CSL_FMK(SRIO_RIO_LSU_REG5_DRBLL_INFO, lsuTransfer->doorbellInfo);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUReg0
 *
 *   @b Description
 *   @n The function configures the REG0 of the specified LSU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number
          rapidIOMSB        Address to be configured in the REG0
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG0_RIO_ADDRESS_MSB
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure REG0 of LSU 0
        CSL_SRIO_SetLSUReg0(hSrio, 0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUReg0
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    Uint32              rapidIOMSB
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG0 = rapidIOMSB;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUReg1
 *
 *   @b Description
 *   @n The function configures the REG1 of the specified LSU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number
          rapidIOLSB        Address to be configured in the REG1
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG1_RIO_ADDRESS_LSB
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure REG1 of LSU 0
        CSL_SRIO_SetLSUReg1(hSrio, 0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUReg1
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    Uint32              rapidIOLSB
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG1 = rapidIOLSB;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUReg2
 *
 *   @b Description
 *   @n The function configures the REG2 of the specified LSU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number
          dspAddress        Address to be configured in the REG2
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG2_DSP_ADDRESS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;       

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure REG2 of LSU 0
        CSL_SRIO_SetLSUReg2(hSrio, 0, &myBuffer);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUReg2
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    Uint32              dspAddress
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG2 = dspAddress;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUReg3
 *
 *   @b Description
 *   @n The function configures the REG3 of the specified LSU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number
          byteCount         Byte Count
          doorbellValid     Doorbell valid flag 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG3_BYTE_COUNT,SRIO_RIO_LSU_REG3_DRBLL_VALUE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;       

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure REG3 of LSU 0
        CSL_SRIO_SetLSUReg3(hSrio, 0, 100, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUReg3
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    Uint32              byteCount,
    Uint8               doorbellValid
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG3 = 
                CSL_FMK (SRIO_RIO_LSU_REG3_BYTE_COUNT, byteCount)      |
                CSL_FMK (SRIO_RIO_LSU_REG3_DRBLL_VALUE,doorbellValid);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUReg4
 *
 *   @b Description
 *   @n The function configures the REG4 of the specified LSU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number
          byteCount         Byte Count
          doorbellValid     Doorbell valid flag 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG4_DESTID, SRIO_RIO_LSU_REG4_SRCID_MAP, SRIO_RIO_LSU_REG4_ID_SIZE,
 *   @n SRIO_RIO_LSU_REG4_OUTPORTID, SRIO_RIO_LSU_REG4_PRIORITY, SRIO_RIO_LSU_REG4_XAMBS,
 *   @n SRIO_RIO_LSU_REG4_SUP_GINT, SRIO_RIO_LSU_REG4_INT_REQ
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;       

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure REG4 of LSU 0
        CSL_SRIO_SetLSUReg4(hSrio, 0, 0xBEEF, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUReg4
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    Uint16              dstId,
    Uint8               srcIdMap,
    Uint8               idSize,
    Uint8               outPortId,
    Uint8               priority,
    Uint8               xambs,
    Uint8               supGoodInt,
    Uint8               intRequest
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG4 = 
            CSL_FMK(SRIO_RIO_LSU_REG4_INT_REQ,   intRequest)    |
            CSL_FMK(SRIO_RIO_LSU_REG4_SRCID_MAP, srcIdMap)      |
            CSL_FMK(SRIO_RIO_LSU_REG4_SUP_GINT,  supGoodInt)    |
            CSL_FMK(SRIO_RIO_LSU_REG4_DESTID,    dstId)         |
            CSL_FMK(SRIO_RIO_LSU_REG4_ID_SIZE,   idSize)        |
            CSL_FMK(SRIO_RIO_LSU_REG4_XAMBS,     xambs)         |
            CSL_FMK(SRIO_RIO_LSU_REG4_PRIORITY,  priority)      |
            CSL_FMK(SRIO_RIO_LSU_REG4_OUTPORTID, outPortId);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUReg5
 *
 *   @b Description
 *   @n The function configures the REG5 of the specified LSU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number
          ttype             TTYPE value
          ftype             FTYPE value
          hopCount          Hop Count
          doorBellInfo      Doorbell Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG5_TTYPE,     SRIO_RIO_LSU_REG5_FTYPE,
 *   @n SRIO_RIO_LSU_REG5_HOP_COUNT, SRIO_RIO_LSU_REG5_DRBLL_INFO
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;       

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Configure REG5 of LSU 0
        CSL_SRIO_SetLSUReg5(hSrio, 0, 4, 5, 1, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUReg5
(
    CSL_SrioHandle      hSrio, 
    Uint8               lsu,
    Uint8               ttype,
    Uint8               ftype,
    Uint8               hopCount,
    Uint16              doorbellInfo
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG5 = 
            CSL_FMK(SRIO_RIO_LSU_REG5_TTYPE,      ttype)       |
		    CSL_FMK(SRIO_RIO_LSU_REG5_FTYPE,      ftype)       |
            CSL_FMK(SRIO_RIO_LSU_REG5_HOP_COUNT,  hopCount)    |
            CSL_FMK(SRIO_RIO_LSU_REG5_DRBLL_INFO, doorbellInfo);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsLSUFull
 *
 *   @b Description
 *   @n The function checks if the specific LSU is BUSY or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number which is to checked
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - All the LSU Shadow Registers are being used
 *   @n FALSE - There is space in the LSU Shadow Registers
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_REG6_FULL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        SRIO_LSU_TRANSFER  lsuTransfer;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...

        // Wait around till there is space to write 
        while (1)
        {
            if (CSL_SRIO_IsLSUFull (hSrio, 1) == FALSE)
                break;
        }
        
        // Populate the Transfer Information. 
        lsuTransfer.rapidIOMSB    = 0x0;
        lsuTransfer.rapidIOLSB    = (Uint32)&tx_buffer[0];
        lsuTransfer.dspAddress    = (Uint32)&rx_buffer[0];
        lsuTransfer.bytecount     = 256;
        lsuTransfer.doorbellValid = 0;
        lsuTransfer.intrRequest   = 1;
        lsuTransfer.supInt        = 0;
        lsuTransfer.xambs         = 0;
        lsuTransfer.priority      = 2;
        lsuTransfer.outPortID     = 1;
        lsuTransfer.idSize        = 1;
        lsuTransfer.srcIDMap      = 0;
        lsuTransfer.dstID         = 0xDEAD;
        lsuTransfer.ttype         = 4;
        lsuTransfer.ftype         = 5;
        lsuTransfer.hopCount      = 0;
        lsuTransfer.doorbellInfo  = 0;

        // Send the transfer on LSU 1 
        CSL_SRIO_SetLSUTransfer (hSrio, 1, &lsuTransfer);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsLSUFull
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu
)
{
    if (CSL_FEXT(hSrio->LSU_CMD[lsu].RIO_LSU_REG6, SRIO_RIO_LSU_REG6_FULL) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsLSUBusy
 *
 *   @b Description
 *   @n The function checks if the specific LSU is BUSY or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number which is to checked
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - LSU is Busy
 *   @n FALSE - LSU is Free
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_REG6_BUSY
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        SRIO_LSU_TRANSFER  lsuTransfer;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Populate the Transfer Information. 
        lsuTransfer.rapidIOMSB    = 0x0;
        lsuTransfer.rapidIOLSB    = (Uint32)&tx_buffer[0];
        lsuTransfer.dspAddress    = (Uint32)&rx_buffer[0];
        lsuTransfer.bytecount     = 256;
        lsuTransfer.doorbellValid = 0;
        lsuTransfer.intrRequest   = 1;
        lsuTransfer.supInt        = 0;
        lsuTransfer.xambs         = 0;
        lsuTransfer.priority      = 2;
        lsuTransfer.outPortID     = 1;
        lsuTransfer.idSize        = 1;
        lsuTransfer.srcIDMap      = 0;
        lsuTransfer.dstID         = 0xDEAD;
        lsuTransfer.ttype         = 4;
        lsuTransfer.ftype         = 5;
        lsuTransfer.hopCount      = 0;
        lsuTransfer.doorbellInfo  = 0;

        // Send the transfer on LSU 1 
        CSL_SRIO_SetLSUTransfer (hSrio, 1, &lsuTransfer);

        // Wait around till the transfer is completed.
        while (1)
        {
            if (CSL_SRIO_IsLSUBusy (hSrio, 1) == FALSE)
                break;
        }

        // Control comes here indicating that the transfer was completed.
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsLSUBusy
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu
)
{
    if (CSL_FEXT(hSrio->LSU_CMD[lsu].RIO_LSU_REG6, SRIO_RIO_LSU_REG6_BUSY) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLSUContextTransaction
 *
 *   @b Description
 *   @n The function gets the LSU Context bit which can be used by the 
 *      transaction to identify if the context of the completion code is with 
 *      respect to the current transaction or not
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Index
          context           Context Information populated by this API 
          transID           Transaction ID populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_REG6_LCB,SRIO_RIO_LSU_REG6_LTID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              context;
        Uint8              transID;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the LSU Context and Transaction Information.
        CSL_SRIO_GetLSUContextTransaction (hSrio, &context, &transID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLSUContextTransaction
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu,
    Uint8*          context,
    Uint8*          transID
)
{
    Uint32 value = hSrio->LSU_CMD[lsu].RIO_LSU_REG6;

    /* Populate the context and transaction identifier. */
    *context = CSL_FEXT (value, SRIO_RIO_LSU_REG6_LCB);
    *transID = CSL_FEXT (value, SRIO_RIO_LSU_REG6_LTID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_CancelLSUTransaction
 *
 *   @b Description
 *   @n The function is used to cancel an LSU Transaction previously submitted. 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Index
          privID            Priv ID of the original transaction submitter.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG6_CBUSY = 1,
 *   @n SRIO_RIO_LSU_REG6_PRIVID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Cancel the LSU Transaction for LSU 0 for PRIV-ID 1
        CSL_SRIO_CancelLSUTransaction (hSrio, 0, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_CancelLSUTransaction
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu,
    Uint8           privID
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG6 = 
            CSL_FMK(SRIO_RIO_LSU_REG6_CBUSY,  1)       |
            CSL_FMK(SRIO_RIO_LSU_REG6_PRIVID, privID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_RestartLSUTransaction
 *
 *   @b Description
 *   @n The function is used to restart an LSU Transaction which had been halted
 *      due to an error condition.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Index which needs to be restarted
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG6_RESTART=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Restart the LSU transaction 1
        CSL_SRIO_RestartLSUTransaction (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_RestartLSUTransaction
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu
)
{
    hSrio->LSU_CMD[lsu].RIO_LSU_REG6 = CSL_FMK (SRIO_RIO_LSU_REG6_RESTART, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_FlushLSUTransaction
 *
 *   @b Description
 *   @n The function is used to flush an LSU Transaction previously submitted 
 *      and which matches the SRCID specified here.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Index which needs to be flushed
          srcIDMap          Source ID 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_REG6_SCRID_MAP, SRIO_RIO_LSU_REG6_FLUSH=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Flush the LSU transaction on LSU 1 with SRC ID 0x0 
        CSL_SRIO_FlushLSUTransaction (hSrio, 1, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_FlushLSUTransaction
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu,
    Uint8           srcIDMap
)
{
    CSL_FINS (hSrio->LSU_CMD[lsu].RIO_LSU_REG6, SRIO_RIO_LSU_REG6_SCRID_MAP, srcIDMap);
    CSL_FINS (hSrio->LSU_CMD[lsu].RIO_LSU_REG6, SRIO_RIO_LSU_REG6_FLUSH, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUShadowRegs
 *
 *   @b Description
 *   @n The function is used to configure the number of LSU Shadow Registers.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          shadowGrp0        Number of Shadow Registers associated with LSU0-3
          shadowGrp1        Number of Shadow Registers associated with LSU4-7
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP0, SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the LSU Shadow Regs as 4 for LSU 0-3 and 4 for LSU 4-7
        CSL_SRIO_SetLSUShadowRegs (hSrio, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUShadowRegs
(
    CSL_SrioHandle  hSrio,
    Uint32          shadowGrp0,
    Uint32          shadowGrp1
)
{
    hSrio->RIO_LSU_SETUP_REG0 = CSL_FMK(SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP0, shadowGrp0) |
                                CSL_FMK(SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP1, shadowGrp1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLSUShadowRegs
 *
 *   @b Description
 *   @n The function is used to get the number of shadow registers associated
 *      with each LSU.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Index for which the count is required
          count             Number of Shadow Registers associated with the specified LSU
                            populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_SETUP_REG0_LSU0_CNT;SRIO_RIO_LSU_SETUP_REG0_LSU1_CNT;
 *      SRIO_RIO_LSU_SETUP_REG0_LSU2_CNT;SRIO_RIO_LSU_SETUP_REG0_LSU3_CNT;
 *      SRIO_RIO_LSU_SETUP_REG0_LSU4_CNT;SRIO_RIO_LSU_SETUP_REG0_LSU5_CNT;
 *      SRIO_RIO_LSU_SETUP_REG0_LSU6_CNT;SRIO_RIO_LSU_SETUP_REG0_LSU7_CNT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              count;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the number of shadow register for LSU 1 
        CSL_SRIO_GetLSUShadowRegs (hSrio, 1, &count);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLSUShadowRegs
(
    CSL_SrioHandle  hSrio,
    Uint8           index,
    Uint8*          count
)
{
    if (index == 0)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU0_CNT);
    else if (index == 1)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU1_CNT);
    else if (index == 2)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU2_CNT);
    else if (index == 3)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU3_CNT);
    else if (index == 4)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU4_CNT);
    else if (index == 5)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU5_CNT);
    else if (index == 6)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU6_CNT);
    else if (index == 7)
       *count = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG0, SRIO_RIO_LSU_SETUP_REG0_LSU7_CNT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableSrcIDInterrupts
 *
 *   @b Description
 *   @n The function is used to enable the LSU interrupts to based on the SRCID
 *      and not on the LSU specific interrupt bit.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             LSU Index to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=0;SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=0;
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=0;SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=0;
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=06;SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=0;
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Enable Source ID interrupts for LSU 1
        CSL_SRIO_EnableSrcIDInterrupts (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableSrcIDInterrupts
(
    CSL_SrioHandle  hSrio,
    Uint8           index
)
{
    if (index == 0)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 0, 0, 0);
    else if (index == 1)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 1, 1, 0);
    else if (index == 2)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 2, 2, 0);
    else if (index == 3)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 3, 3, 0);
    else if (index == 4)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 4, 4, 0);
    else if (index == 5)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 5, 5, 0);
    else if (index == 6)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 6, 6, 0);
    else if (index == 7)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 7, 7, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableSrcIDInterrupts
 *
 *   @b Description
 *   @n The function is used to enable the LSU interrupts to based on the LSU 
 *      specific interrupt bit and not on the SRCID.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          index             LSU Index to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=1;SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=2;
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=4;SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=8;
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=16;SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=32;
 *   @n SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA=64
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Disable SRC ID interrupts for LSU 2
        CSL_SRIO_DisableSrcIDInterrupts (hSrio, 2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableSrcIDInterrupts
(
    CSL_SrioHandle  hSrio,
    Uint8           index
)
{
    if (index == 0)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 0, 0, 1);
    else if (index == 1)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 1, 1, 1);
    else if (index == 2)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 2, 2, 1);
    else if (index == 3)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 3, 3, 1);
    else if (index == 4)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 4, 4, 1);
    else if (index == 5)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 5, 5, 1);
    else if (index == 6)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 6, 6, 1);
    else if (index == 7)
       CSL_FINSR (hSrio->RIO_LSU_SETUP_REG1, 7, 7, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTimeoutCnt
 *
 *   @b Description
 *   @n The function is used to set the number of timecode changes after an error
 *      condition before the current LSU transaction is discarded and a new 
 *      transaction is loaded from the shadow register.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          timeoutCnt        Time out count to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the Timeout Count to be 1
        CSL_SRIO_SetTimeoutCnt (hSrio, 2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTimeoutCnt
(
    CSL_SrioHandle  hSrio,
    Uint8           timeoutCnt
)
{
    CSL_FINS (hSrio->RIO_LSU_SETUP_REG1, SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT, timeoutCnt);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTimeoutCnt
 *
 *   @b Description
 *   @n The function is used to get the number of timecode changes after an error
 *      condition before the current LSU transaction is discarded and a new 
 *      transaction is loaded from the shadow register.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          timeoutCnt        Time out count to be populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              timeoutCnt;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the Timeout Count
        CSL_SRIO_GetTimeoutCnt (hSrio, &timeoutCnt);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTimeoutCnt
(
    CSL_SrioHandle  hSrio,
    Uint8*          timeoutCnt
)
{
    *timeoutCnt = CSL_FEXT (hSrio->RIO_LSU_SETUP_REG1, SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLSUCompletionCode
 *
 *   @b Description
 *   @n The function gets the completion code for a previous executed transfer.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number which is to checked
          transId           Transaction ID for which the completion code is
                            required.
          compCode          Completion Code populated by this API
          contextBit        Context Bit populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  SRIO_LSU_STAT_REG
 
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              context;
        Uint8              transID;
        SRIO_LSU_TRANSFER  lsuTransfer;
        Uint8              compCode;
        Uint8              contextBit;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the LSU Context and Transaction Information.
        CSL_SRIO_GetLSUContextTransaction (hSrio, &context, &transID);
        ...
        // Send the transfer on LSU 1 
        CSL_SRIO_SetLSUTransfer (hSrio, 1, &lsuTransfer);

        // Wait around till the transfer is completed.
        while (1)
        {
            if (CSL_SRIO_IsLSUBusy (hSrio, 1) == FALSE)
                break;
        }
        // Get the completion code.
        CSL_SRIO_GetLSUCompletionCode (hSrio, 1, transID, &compCode, &contextBit);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLSUCompletionCode
(
    CSL_SrioHandle  hSrio, 
    Uint8           lsu,
    Uint8           transID,
    Uint8*          compCode,
    Uint8*          contextBit
)
{
    Uint8   lsuRegIndex;

    /* Check the LSU for which the completion code is required. */
    if ((lsu == 0) || (lsu == 4))
    {
        /* LSU0 or LSU4: Determine which register index we need to access.
         *  For LSU0 the register index is 0 and 1; but for LSU4 the register index is 3 and 4 */
        if (lsu == 0)
            lsuRegIndex = 0;
        else
            lsuRegIndex = 3;

        /* There are 9 pending LSU Transaction ID for this. */
        if (transID < 8)
        {
            /* Transaction ID 0-7 are handled here. */
            *compCode   = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], ((transID << 2) + 3), ((transID << 2) + 1));
            *contextBit = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], (transID << 2), (transID << 2));
        }
        else
        {
            /* Transaction ID 8 is handled here. */
            *compCode   = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex+1], 3, 1);
            *contextBit = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex+1], 0, 0);
        }
    }
    else if ((lsu == 1) || (lsu == 5))
    {
        /* LSU1 or LSU5: Determine which register index we need to access.
         *  For LSU1 the register index is 1; but for LSU5 the register index is 4 */
        if (lsu == 1)
            lsuRegIndex = 1;
        else
            lsuRegIndex = 4;

        /* There are 6 pending LSU Transaction ID for this. */
        *compCode   = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], ((transID << 2) + 7), ((transID << 2) + 5));
        *contextBit = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], ((transID << 2) + 4), ((transID << 2) + 4));
    }
    else if ((lsu == 2) || (lsu == 6))
    {
        /* LSU2 or LSU6: Determine which register index we need to access.
         *  For LSU2 the register index is 1 and 2; but for LSU5 the register index is 4 and 5 */
        if (lsu == 2)
            lsuRegIndex = 1;
        else
            lsuRegIndex = 4;

        /* There are 5 pending LSU Transaction ID for this. */
        if (transID == 0)
        {
            /* Transaction ID 0 is handled here. */
            *compCode   = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], 31, 29);
            *contextBit = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], 28, 28);
        }
        else
        {
            /* Transaction ID 1-4 are handled here. */
            *compCode   = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex+1], ((transID << 2) - 1), ((transID << 2) - 3));
            *contextBit = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex+1], ((transID << 2) - 4), ((transID << 2) - 4));
        }
    }
    else if ((lsu == 3) || (lsu == 7))
    {
        /* LSU3 or LSU7: Determine which register index we need to access.
         *  For LSU3 the register index is 2; but for LSU5 the register index is 5 */
        if (lsu == 3)
            lsuRegIndex = 2;
        else
            lsuRegIndex = 5;

        /* There are 4 pending LSU Transaction ID for this. */
        *compCode   = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], ((transID << 2) + 19), ((transID << 2) + 17));
        *contextBit = CSL_FEXTR(hSrio->LSU_STAT_REG[lsuRegIndex], ((transID << 2) + 16), ((transID << 2) + 16));
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLSUFlowMask
 *
 *   @b Description
 *   @n The function is used to set the LSU Flow Masks
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number for which flow mask is to be configured          
          flowMask          Flow Mask to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_LSU_FLOW_MASKS_LSU_EVEN_FLOW_MASK;SRIO_RIO_LSU_FLOW_MASKS_LSU_ODD_FLOW_MASK 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the LSU Flow Mask for LSU 0 to use Flow 1
        CSL_SRIO_SetLSUFlowMask (hSrio, 0, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLSUFlowMask
(
    CSL_SrioHandle  hSrio,
    Uint8           lsu,
    Uint16          flowMask
)
{
    switch (lsu)
    {
        case 0:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[0], 15, 0, flowMask);
            break;
        }
        case 1:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[0], 31, 16,flowMask);
            break;
        }
        case 2:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[1], 15, 0, flowMask);
            break;
        }
        case 3:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[1], 31, 16,flowMask);
            break;
        }
        case 4:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[2], 15, 0, flowMask);
            break;
        }
        case 5:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[2], 31, 16,flowMask);
            break;
        }
        case 6:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[3], 15, 0, flowMask);
            break;
        }
        case 7:
        {
            CSL_FINSR(hSrio->RIO_LSU_FLOW_MASKS[3], 31, 16,flowMask);
            break;
        }
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLSUFlowMask
 *
 *   @b Description
 *   @n The function is used to get the LSU Flow Masks
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          lsu               LSU Number for which flow mask is to be retreived.
          flowMask          Flow Mask to be populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_LSU_FLOW_MASKS_LSU_FLOW_MASK 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint16             flowMask;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the LSU Flow Mask for LSU 0 
        CSL_SRIO_GetLSUFlowMask (hSrio, 0, &flowMask);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLSUFlowMask
(
    CSL_SrioHandle  hSrio,
    Uint8           lsu,
    Uint16*         flowMask
)
{
    switch (lsu)
    {
        case 0:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[0], 15, 0);
            break;
        }
        case 1:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[0], 31, 16);
            break;
        }
        case 2:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[1], 15, 0);
            break;
        }
        case 3:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[1], 31, 16);
            break;
        }
        case 4:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[2], 15, 0);
            break;
        }
        case 5:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[2], 31, 16);
            break;
        }
        case 6:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[3], 15, 0);
            break;
        }
        case 7:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_LSU_FLOW_MASKS[3], 31, 16);
            break;
        }
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetSupervisorPerms
 *
 *   @b Description
 *   @n The function is used to set the supervisor permissions
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          supervisor16ID    16 bit Supervisor ID
          supervisor8ID     8 bit Supervisor ID
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID,SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the Supervisor Permissions for Source ID 0xBEEF
        CSL_SRIO_SetSupervisorPerms (hSrio, 0xBEEF, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetSupervisorPerms
(
    CSL_SrioHandle  hSrio,
    Uint16          supervisor16ID,
    Uint8           supervisor8ID
)
{
    hSrio->RIO_SUPERVISOR_ID = CSL_FMK(SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID,supervisor16ID) | 
                               CSL_FMK(SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID, supervisor8ID); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetSupervisorPerms
 *
 *   @b Description
 *   @n The function is used to get the supervisor source ID.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          supervisor16ID    16 bit Supervisor ID populated by this API
          supervisor8ID     8 bit Supervisor ID populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID,SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              super8ID;
        Uint16             super16ID;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the Supervisor Permissions.
        CSL_SRIO_GetSupervisorPerms (hSrio, &super16ID, &super8ID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetSupervisorPerms
(
    CSL_SrioHandle  hSrio,
    Uint16*         supervisor16ID,
    Uint8*          supervisor8ID
)
{
    *supervisor16ID = CSL_FEXT (hSrio->RIO_SUPERVISOR_ID, SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID);
    *supervisor8ID  = CSL_FEXT (hSrio->RIO_SUPERVISOR_ID, SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetFlowControl
 *
 *   @b Description
 *   @n The function is used to set the RIO flow control.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          flowControlIdx    Flow Control Index which is to be configured. 
          tt                Flag which defines 16 bit or 8 bit identifiers. Set to
                            1 for 16 bit identifiers 0 for 8 bit.
          dstID             Destination ID matching the flow
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_FLOW_CNTL_TT, SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the SRIO Flow Control 0 for Destination ID 0xBEEF
        // since the destination ID is 16 bit we set the TT flag as 1
        CSL_SRIO_SetFlowControl (hSrio, 0, 1, 0xBEEF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetFlowControl
(
    CSL_SrioHandle  hSrio,
    Uint8           flowControlIdx,
    Uint8           tt,
    Uint16          dstID
)
{
    hSrio->RIO_FLOW_CNTL[flowControlIdx] = CSL_FMK(SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID,dstID) | 
                                           CSL_FMK(SRIO_RIO_FLOW_CNTL_TT, tt); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetFlowControl
 *
 *   @b Description
 *   @n The function is used to get the RIO flow control.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          flowControlIdx    Flow Control Index for which the configuration is required.
          tt                Flag which defines 16 bit or 8 bit identifiers populated by
                            the API
          dstID             Destination ID matching the flow populated by the API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID,SRIO_RIO_FLOW_CNTL_TT 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              tt;
        Uint16             dstID;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the SRIO Flow Control 0 
        CSL_SRIO_GetFlowControl (hSrio, 0, &tt, &dstID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetFlowControl
(
    CSL_SrioHandle  hSrio,
    Uint8           flowControlIdx,
    Uint8*          tt,
    Uint16*         dstID
)
{
    Uint32 value = hSrio->RIO_FLOW_CNTL[flowControlIdx];

    /* Extract the fields from the RIO Flow Control */
    *dstID = CSL_FEXT(value, SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID);
    *tt    = CSL_FEXT(value, SRIO_RIO_FLOW_CNTL_TT); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTxCPPIFlowMask
 *
 *   @b Description
 *   @n The function is used to set the Transmit CPPI Flow Masks
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          txCPPI            Transmit CPPI Flow Index 
          flowMask          Flow Mask to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_TX_CPPI_FLOW_MASKS_TX_QUEUE_EVEN_FLOW_MASK;
 *   @n SRIO_RIO_TX_CPPI_FLOW_MASKS_TX_QUEUE_ODD_FLOW_MASK 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the Transmit CPPI Flow Mask for 0 to use Flow 1
        CSL_SRIO_SetTxCPPIFlowMask (hSrio, 0, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTxCPPIFlowMask
(
    CSL_SrioHandle  hSrio,
    Uint8           txCPPI,
    Uint16          flowMask
)
{
    switch (txCPPI)
    {
        case 0:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[0], 15, 0, flowMask);
            break;
        }
        case 1:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[0], 31, 16, flowMask);
            break;
        }
        case 2:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[1], 15, 0, flowMask);
            break;
        }
        case 3:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[1], 31, 16, flowMask);
            break;
        }
        case 4:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[2], 15, 0, flowMask);
            break;
        }
        case 5:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[2], 31, 16, flowMask);
            break;
        }
        case 6:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[3], 15, 0, flowMask);
            break;
        }
        case 7:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[3], 31, 16, flowMask);
            break;
        }
        case 8:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[4], 15, 0, flowMask);
            break;
        }
        case 9:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[4], 31, 16, flowMask);
            break;
        }
        case 10:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[5], 15, 0, flowMask);
            break;
        }
        case 11:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[5], 31, 16, flowMask);
            break;
        }
        case 12:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[6], 15, 0, flowMask);
            break;
        }
        case 13:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[6], 31, 16, flowMask);
            break;
        }
        case 14:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[7], 15, 0, flowMask);
            break;
        }
        case 15:
        {
            CSL_FINSR(hSrio->RIO_TX_CPPI_FLOW_MASKS[7], 31, 16, flowMask);
            break;
        }        
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTxCPPIFlowMask
 *
 *   @b Description
 *   @n The function is used to get the Transmit CPPI Flow Masks
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          txCPPI            Transmit CPPI Flow Index 
          flowMask          Flow Mask to be populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_TX_CPPI_FLOW_MASKS_TX_QUEUE_FLOW_MASK 
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint16             flowMask;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the Transmit CPPI Flow Mask for 0 to use Flow 1
        CSL_SRIO_GetTxCPPIFlowMask (hSrio, 0, &flowMask);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTxCPPIFlowMask
(
    CSL_SrioHandle  hSrio,
    Uint8           txCPPI,
    Uint16*         flowMask
)
{
    switch (txCPPI)
    {
        case 0:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[0], 15, 0);
            break;
        }
        case 1:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[0], 31, 16);
            break;
        }
        case 2:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[1], 15, 0);
            break;
        }
        case 3:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[1], 31, 16);
            break;
        }
        case 4:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[2], 15, 0);
            break;
        }
        case 5:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[2], 31, 16);
            break;
        }
        case 6:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[3], 15, 0);
            break;
        }
        case 7:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[3], 31, 16);
            break;
        }
        case 8:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[4], 15, 0);
            break;
        }
        case 9:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[4], 31, 16);
            break;
        }
        case 10:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[5], 15, 0);
            break;
        }
        case 11:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[5], 31, 16);
            break;
        }
        case 12:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[6], 15, 0);
            break;
        }
        case 13:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[6], 31, 16);
            break;
        }
        case 14:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[7], 15, 0);
            break;
        }
        case 15:
        {
            *flowMask = CSL_FEXTR(hSrio->RIO_TX_CPPI_FLOW_MASKS[7], 31, 16);
            break;
        }
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTxQueueSchedInfo
 *
 *   @b Description
 *   @n The function is used to set the port and scheduler information for a 
 *      specific transmit queue.
 *
 *   @b Arguments
     @verbatim
          hSrio       Handle of the SRIO device
          txQueue     Transmit Queue Index for which information is configured.
          portNum     Port Number to which the queue is mapped.
          crf         Critical Resource Flow Value              
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO; 
 *    SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO;
 *    SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO;
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the Tx Queue 0 Scheduler Information
        //  - The Queue needs to be mapped to SRIO Port 1
        //  - The Queue sets the CRF Bit.
        CSL_SRIO_SetTxQueueSchedInfo (hSrio, 0, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTxQueueSchedInfo
(
    CSL_SrioHandle  hSrio,
    Uint8           txQueue,
    Uint8           portNum,
    Uint8           crf
)
{
    if (txQueue < 4)
    {
        /* Transmit Queue 0, 1, 2 and 3 are handed in RIO_TX_QUEUE_SCH_INFO[0] */
        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[0], (txQueue << 3), (txQueue << 3), crf);
        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[0], (txQueue << 3) + 7, (txQueue << 3) + 4, portNum);
    }
    else if (txQueue < 8)
    {
        /* Transmit Queue 4, 5, 6 and 7 are handed in RIO_TX_QUEUE_SCH_INFO[1] */
        txQueue = txQueue - 4;

        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[1], (txQueue << 3), (txQueue << 3), crf);
        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[1], (txQueue << 3) + 7, (txQueue << 3) + 4, portNum);
    } 
    else if (txQueue < 12)
    {
        /* Transmit Queue 8, 9, 10 and 11 are handed in RIO_TX_QUEUE_SCH_INFO[2] */
        txQueue = txQueue - 8;

        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[2], (txQueue << 3), (txQueue << 3), crf);
        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[2], (txQueue << 3) + 7, (txQueue << 3) + 4, portNum);
    } 
    else if (txQueue < 16)
    {
        /* Transmit Queue 12, 13, 14 and 15 are handed in RIO_TX_QUEUE_SCH_INFO[2] */
        txQueue = txQueue - 12;

        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[3], (txQueue << 3), (txQueue << 3), crf);
        CSL_FINSR(hSrio->RIO_TX_QUEUE_SCH_INFO[3], (txQueue << 3) + 7, (txQueue << 3) + 4, portNum);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTxQueueSchedInfo
 *
 *   @b Description
 *   @n The function is used to get the port and scheduler information for a 
 *      specific transmit queue.
 *
 *   @b Arguments
     @verbatim
          hSrio       Handle of the SRIO device
          txQueue     Transmit Queue Index for which information is configured.
          portNum     Port Number to which the queue is mapped populated by this API
          priority    Priority of the Transmit Queue populated by this API
          crf         Critical Resource Flow Value populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO; 
 *    SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO;
 *    SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO,SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO;
 *
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle     hSrio;
        Uint8              portNum;
        Uint8              priority;
        Uint8              crf;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the Tx Queue 1 Information.
        CSL_SRIO_GetTxQueueSchedInfo (hSrio, 1, &portNum, &priority, &crf);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTxQueueSchedInfo
(
    CSL_SrioHandle  hSrio,
    Uint8           txQueue,
    Uint8*          portNum,
    Uint8*          priority,
    Uint8*          crf
)
{
    Uint32 value;

    if (txQueue < 4)
    {
        /* Transmit Queue 0, 1, 2 and 3 are handed in RIO_TX_QUEUE_SCH_INFO[0] */
        value = hSrio->RIO_TX_QUEUE_SCH_INFO[0];
        *crf      = CSL_FEXTR(value, (txQueue << 3), (txQueue << 3));
        *priority = CSL_FEXTR(value, (txQueue << 3) + 2, (txQueue << 3) + 1);
        *portNum  = CSL_FEXTR(value, (txQueue << 3) + 7, (txQueue << 3) + 4);
    }
    else if (txQueue < 8)
    {
        /* Transmit Queue 4, 5, 6 and 7 are handed in RIO_TX_QUEUE_SCH_INFO[1] */
        txQueue = txQueue - 4;
        value = hSrio->RIO_TX_QUEUE_SCH_INFO[1];

        *crf      = CSL_FEXTR(value, (txQueue << 3), (txQueue << 3));
        *priority = CSL_FEXTR(value, (txQueue << 3) + 2, (txQueue << 3) + 1);
        *portNum  = CSL_FEXTR(value, (txQueue << 3) + 7, (txQueue << 3) + 4);
    } 
    else if (txQueue < 12)
    {
        /* Transmit Queue 8, 9, 10 and 11 are handed in RIO_TX_QUEUE_SCH_INFO[2] */
        txQueue = txQueue - 8;
        value = hSrio->RIO_TX_QUEUE_SCH_INFO[2];

        *crf      = CSL_FEXTR(value, (txQueue << 3), (txQueue << 3));
        *priority = CSL_FEXTR(value, (txQueue << 3) + 2, (txQueue << 3) + 1);
        *portNum  = CSL_FEXTR(value, (txQueue << 3) + 7, (txQueue << 3) + 4);
    } 
    else if (txQueue < 16)
    {
        /* Transmit Queue 12, 13, 14 and 15 are handed in RIO_TX_QUEUE_SCH_INFO[3] */
        txQueue = txQueue - 12;
        value = hSrio->RIO_TX_QUEUE_SCH_INFO[3];

        *crf      = CSL_FEXTR(value, (txQueue << 3), (txQueue << 3));
        *priority = CSL_FEXTR(value, (txQueue << 3) + 2, (txQueue << 3) + 1);
        *portNum  = CSL_FEXTR(value, (txQueue << 3) + 7, (txQueue << 3) + 4);
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTxGarbageCollectionInfo
 *
 *   @b Description
 *   @n The function sets the Transmit Garbage collection information. 
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          qidLength     Length mismatch between size in the UDI to be configured. 
          qidTimeout    Timeout on receiving the segments to be configured.
          qidRetry      Excessive retries to be configured
          qidTransErr   Transaction Error to be configured
          qidProg       Programming Error queue to be configured
          qidSsize      Queue ID Ssize to be configured

     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN, 
 *   @n SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT,
 *   @n SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY,
 *   @n SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR,
 *   @n SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG,
 *   @n SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the Transmit Garbage Collection Information.
        CSL_SRIO_SetTxGarbageCollectionInfo (hSrio, 2, 3, 4, 5, 6, 10);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTxGarbageCollectionInfo
(
    CSL_SrioHandle  hSrio,
    Uint16          qidLength,
    Uint16          qidTimeout,
    Uint16          qidRetry,
    Uint16          qidTransErr,
    Uint16          qidProg,
    Uint16          qidSsize
)
{
    hSrio->RIO_GARBAGE_COLL_QID0 = 
            CSL_FMK (SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN,       qidLength)   | 
            CSL_FMK (SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT,      qidTimeout);

    hSrio->RIO_GARBAGE_COLL_QID1 = 
            CSL_FMK (SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY,     qidRetry)    |
            CSL_FMK (SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR, qidTransErr);

    hSrio->RIO_GARBAGE_COLL_QID2 = 
            CSL_FMK (SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG, qidProg) |
            CSL_FMK (SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE, qidSsize);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTxGarbageCollectionInfo
 *
 *   @b Description
 *   @n The function gets the Transmit Garbage collection information. Descriptors
 *      are placed into the Garbage collection queue when there is an error 
 *      response detected.
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          qidLength     Length mismatch between size in the UDI 
                        packet and receieved payload populated by this API
          qidTimeout    Timeout on receiving the segments populated by this API
          qidRetry      Excessive retries populated by this API
          qidTransErr   Transaction Error populated by this API
          qidProg       Programming error queue read
          qidSsize      Queue ID Ssize populated by this API

     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN, 
 *   @n SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT,
 *   @n SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY,
 *   @n SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR,
 *   @n SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG, 
 *   @n SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          qidLength;
        Uint16          qidTimeout;
        Uint16          qidRetry;
        Uint16          qidTransErr;
        Uint16          qidProg;
        Uint16          qidSsize;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the Transmit Garbage Collection Information.
        CSL_SRIO_GetTxGarbageCollectionInfo (hSrio, &qidLength, &qidTimeout, 
                                             &qidRetry, &qidTransErr, &qidProg, &qidSsize);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTxGarbageCollectionInfo
(
    CSL_SrioHandle  hSrio,
    Uint16*         qidLength,
    Uint16*         qidTimeout,
    Uint16*         qidRetry,
    Uint16*         qidTransErr,
    Uint16*         qidProg,
    Uint16*         qidSsize
)
{
    *qidLength  = CSL_FEXT (hSrio->RIO_GARBAGE_COLL_QID0, SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN);
    *qidTimeout = CSL_FEXT (hSrio->RIO_GARBAGE_COLL_QID0, SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT);
    *qidRetry   = CSL_FEXT (hSrio->RIO_GARBAGE_COLL_QID1, SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY);
    *qidTransErr= CSL_FEXT (hSrio->RIO_GARBAGE_COLL_QID1, SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR);
    *qidProg    = CSL_FEXT (hSrio->RIO_GARBAGE_COLL_QID2, SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG);
    *qidSsize   = CSL_FEXT (hSrio->RIO_GARBAGE_COLL_QID2, SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetQMBaseAddress
 *
 *   @b Description
 *   @n The function gets the QM base address
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          qmIndex           Index of the Queue Manager whose base address is needed
          qmBaseAddress     QM Base Address populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_QM_BASE_ADDRESS_REG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          qmBaseAddress;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Get the QM Base Address information for 0
        CSL_SRIO_GetQMBaseAddress(hSrio, 0, &qmBaseAddress);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetQMBaseAddress
(
    CSL_SrioHandle  hSrio,
    Uint8           qmIndex,
    Uint32*         qmBaseAddress
)
{
    *qmBaseAddress = hSrio->QM_BASE_ADDRESS_REG[qmIndex];
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetQMBaseAddress
 *
 *   @b Description
 *   @n The function sets the QM base address
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          qmIndex           Index of the Queue Manager whose base address is to be set
          qmBaseAddress     QM Base Address to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n SRIO_QM_BASE_ADDRESS_REG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);
        ...
        // Set the QM Base Address information for 0
        CSL_SRIO_SetQMBaseAddress(hSrio, 0, 0x10800000);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetQMBaseAddress
(
    CSL_SrioHandle  hSrio,
    Uint8           qmIndex,
    Uint32          qmBaseAddress
)
{
    hSrio->QM_BASE_ADDRESS_REG[qmIndex] = qmBaseAddress;
}


/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_SRIOAUX_H_ */


