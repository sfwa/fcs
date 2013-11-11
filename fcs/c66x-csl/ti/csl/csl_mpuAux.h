/**
 *   @file  csl_mpuAux.h
 *
 *   @brief   
 *      This is the MPU Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the MPU Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2011, Texas Instruments, Inc.
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

#ifndef _CSL_MPUAUX_H_
#define _CSL_MPUAUX_H_

#include <ti/csl/soc.h>
#include <ti/csl/csl_mpu.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_MPU_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_MPU_GetRevision
 *
 *   @b Description
 *   @n This function reads the revision register for the MPU Module.
 *
 *   @b Arguments
     @verbatim
        revisionInfo    - Revision Information populated by the API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_REVISION
 *
 *   @b Example
 *   @verbatim
        Uint32          revisionInfo;
        CSL_MpuHandle   hMpu;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);

        // Get the MPU Peripheral Identification.
        CSL_MPU_GetRevision (hMpu, &revisionInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetRevision 
(
    CSL_MpuHandle   hMpu,
    Uint32*         revisionInfo
)
{
    *revisionInfo = hMpu->REVISION;
}

/** ============================================================================
 *   @n@b CSL_MPU_GetConfiguration
 *
 *   @b Description
 *   @n This function gets the configured values of the MPU Module.
 *
 *   @b Arguments
     @verbatim
        addressAlign    - Address alignment for range checking
        numFixed        - Number of fixed address ranges
        numProg         - Number of programmable address ranges
        numFixedAID     - Number of supported AIDs
        assumedAllowed  - Assumed Allowed Mode 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_CONFIG_ADDRESS_ALIGN, MPU_CONFIG_NUM_FIXED
 *   @n MPU_CONFIG_NUM_PROG, MPU_CONFIG_NUM_FIXED_AIDS
 *   @n MPU_CONFIG_ASSUMED_ALLOWED
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8           addressAlign,
        Uint8           numFixed,
        Uint8           numProg,
        Uint8           numFixedAID,
        Uint8           assumedAllowed

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);

        // Get the MPU Configuration.
        CSL_MPU_GetConfiguration (hMpu, &addressAlign, &numFixed, 
                                  &numProg, &numFixedAID, &assumedAllowed);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetConfiguration 
(
    CSL_MpuHandle   hMpu,
    Uint8*          addressAlign,
    Uint8*          numFixed,
    Uint8*          numProg,
    Uint8*          numFixedAID,
    Uint8*          assumedAllowed
)
{
    Uint32  value = hMpu->CONFIG;

    *addressAlign   = CSL_FEXT (value, MPU_CONFIG_ADDRESS_ALIGN);
    *numFixed       = CSL_FEXT (value, MPU_CONFIG_NUM_FIXED);
    *numProg        = CSL_FEXT (value, MPU_CONFIG_NUM_PROG);
    *numFixedAID    = CSL_FEXT (value, MPU_CONFIG_NUM_FIXED_AIDS);
    *assumedAllowed = CSL_FEXT (value, MPU_CONFIG_ASSUMED_ALLOWED);
}

/** ============================================================================
 *   @n@b CSL_MPU_SetInterruptRawStatus
 *
 *   @b Description
 *   @n This function sets the interrupt raw status of the MPU
 *
 *   @b Arguments
     @verbatim
        addressViolation      - Address  Violation Error
        protocolViolation     - Protocol Violation Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_INT_RAW_STATUS_SET_ADDR_ERR, MPU_INT_RAW_STATUS_SET_PROT_ERR
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);

        // Set the address violation error
        CSL_MPU_SetInterruptRawStatus (hMpu, 1, 0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_SetInterruptRawStatus 
(
    CSL_MpuHandle   hMpu,
    Uint8           addressViolation,
    Uint8           protocolViolation
)
{
    hMpu->INT_RAW_STATUS_SET = 
        CSL_FMK(MPU_INT_RAW_STATUS_SET_ADDR_ERR,  addressViolation)     |
        CSL_FMK(MPU_INT_RAW_STATUS_SET_PROT_ERR,  protocolViolation);
}

/** ============================================================================
 *   @n@b CSL_MPU_GetInterruptStatus
 *
 *   @b Description
 *   @n This function gets the interrupt status of the MPU
 *
 *   @b Arguments
     @verbatim
        addressViolation      - Address  Violation Error
        protocolViolation     - Protocol Violation Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR, 
 *   @n MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8           addressViolation;
        Uint8           protocolViolation;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the status of the MPU Interrupts.
        CSL_MPU_GetInterruptStatus (hMpu, &addressViolation, &protocolViolation);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetInterruptStatus 
(
    CSL_MpuHandle   hMpu,
    Uint8*          addressViolation,
    Uint8*          protocolViolation
)
{
    Uint32  value = hMpu->INT_ENABLED_STATUS_CLEAR;

    *addressViolation  = CSL_FEXT (value, MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR);
    *protocolViolation = CSL_FEXT (value, MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR);
}

/** ============================================================================
 *   @n@b CSL_MPU_ClearInterruptStatus
 *
 *   @b Description
 *   @n This function clears the interrupt status of the MPU
 *
 *   @b Arguments
     @verbatim
        addressViolation      - Address  Violation Error
        protocolViolation     - Protocol Violation Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR, 
 *   @n MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8           addressViolation;
        Uint8           protocolViolation;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the status of the MPU Interrupts.
        CSL_MPU_GetInterruptStatus (hMpu, &addressViolation, &protocolViolation);
        ...
        // Clear the MPU Interrupt status
        CSL_MPU_ClearInterruptStatus(hMpu, addressViolation, protocolViolation);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_ClearInterruptStatus 
(
    CSL_MpuHandle   hMpu,
    Uint8           addressViolation,
    Uint8           protocolViolation
)
{
    hMpu->INT_ENABLED_STATUS_CLEAR = 
            CSL_FMK (MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR, addressViolation) |
            CSL_FMK (MPU_INT_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR, protocolViolation);
}

/** ============================================================================
 *   @n@b CSL_MPU_EnableInterrupts
 *
 *   @b Description
 *   @n This function enables the interrupt for the MPU
 *
 *   @b Arguments
     @verbatim
        addressViolation      - Address  Violation Error
        protocolViolation     - Protocol Violation Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_INT_ENABLE_ADDR_ERR_EN, 
 *   @n MPU_INT_ENABLE_PROT_ERR_EN
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8           addressViolation;
        Uint8           protocolViolation;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Enable both the Address & Protocol Violation Interrupts in the MPU
        CSL_MPU_EnableInterrupts(hMpu, 1, 1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_EnableInterrupts 
(
    CSL_MpuHandle   hMpu,
    Uint8           addressViolation,
    Uint8           protocolViolation
)
{
    hMpu->INT_ENABLE = 
            CSL_FMK (MPU_INT_ENABLE_ADDR_ERR_EN, addressViolation) |
            CSL_FMK (MPU_INT_ENABLE_PROT_ERR_EN, protocolViolation);
}

/** ============================================================================
 *   @n@b CSL_MPU_DisableInterrupts
 *
 *   @b Description
 *   @n This function disables the interrupt for the MPU
 *
 *   @b Arguments
     @verbatim
        addressViolation      - Address  Violation Error
        protocolViolation     - Protocol Violation Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_INT_ENABLE_CLEAR_ADDR_ERR_EN_CLR, 
 *   @n MPU_INT_ENABLE_CLEAR_PROT_ERR_EN_CLR
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8           addressViolation;
        Uint8           protocolViolation;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Disable both the Address & Protocol Violation Interrupts in the MPU
        CSL_MPU_DisableInterrupts(hMpu, 1, 1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_DisableInterrupts 
(
    CSL_MpuHandle   hMpu,
    Uint8           addressViolation,
    Uint8           protocolViolation
)
{
    hMpu->INT_ENABLE_CLEAR = 
            CSL_FMK (MPU_INT_ENABLE_CLEAR_ADDR_ERR_EN_CLR, addressViolation) |
            CSL_FMK (MPU_INT_ENABLE_CLEAR_PROT_ERR_EN_CLR, protocolViolation);
}

/** ============================================================================
 *   @n@b CSL_MPU_GetInterruptEnableStatus
 *
 *   @b Description
 *   @n This function gets the interrupt enable status of the MPU interrupts.
 *
 *   @b Arguments
     @verbatim
        addressViolation      - Address  Violation Error
        protocolViolation     - Protocol Violation Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_INT_ENABLE_CLEAR_ADDR_ERR_EN_CLR, 
 *   @n MPU_INT_ENABLE_CLEAR_PROT_ERR_EN_CLR
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8*          addressViolation;
        Uint8*          protocolViolation;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the Interrupt Enable status for the MPU.
        CSL_MPU_GetInterruptEnableStatus(hMpu, &addressViolation, &protocolViolation);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetInterruptEnableStatus 
(
    CSL_MpuHandle   hMpu,
    Uint8*          addressViolation,
    Uint8*          protocolViolation
)
{
    Uint32  value = hMpu->INT_ENABLE_CLEAR;

    *addressViolation  = CSL_FEXT (value, MPU_INT_ENABLE_CLEAR_ADDR_ERR_EN_CLR);
    *protocolViolation = CSL_FEXT (value, MPU_INT_ENABLE_CLEAR_PROT_ERR_EN_CLR);
}

/** ============================================================================
 *   @n@b CSL_MPU_SetEOI
 *
 *   @b Description
 *   @n This function sets the EOI register to indicate the end of the interrupt
 *      service.
 *
 *   @b Arguments
     @verbatim
        eoiVector     - EOI Vector Value
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_EOI_EOI_VECTOR
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint8           eoiVector;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Set the EOI Vector.
        CSL_MPU_SetEOI(hMpu, eoiVector);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_SetEOI 
(
    CSL_MpuHandle   hMpu,
    Uint8           eoiVector
)
{
    hMpu->EOI = CSL_FMK (MPU_EOI_EOI_VECTOR, eoiVector);
}

/** ============================================================================
 *   @n@b CSL_MPU_GetFixedAddressInfo
 *
 *   @b Description
 *   @n This function gets the fixed address information.
 *
 *   @b Arguments
     @verbatim
        startAddress    - Fixed Start Address
        endAddress      - Fixed End Address
        permissions     - Fixed region permissions.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_FIXED_START_ADDRESS, MPU_FIXED_END_ADDRESS, MPU_FIXED_MPPA
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint32          startAddress,
        Uint32          endAddress,
        Uint32          permissions

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the MPU Fixed Address Information.
        CSL_MPU_GetFixedAddressInfo (hMpu, &startAddress, &endAddress, &permissions);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetFixedAddressInfo 
(
    CSL_MpuHandle   hMpu,
    Uint32*         startAddress,
    Uint32*         endAddress,
    Uint32*         permissions
)
{
    *startAddress = hMpu->FIXED_START_ADDRESS;
    *endAddress   = hMpu->FIXED_END_ADDRESS;
    *permissions  = hMpu->FIXED_MPPA;
}

/** ============================================================================
 *   @n@b CSL_MPU_SetFixedAddressPermissions
 *
 *   @b Description
 *   @n This function sets the fixed address permissions.
 *
 *   @b Arguments
     @verbatim
        permissions     - Fixed region permissions.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_FIXED_MPPA
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint32          startAddress,
        Uint32          endAddress,
        Uint32          permissions

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the MPU Fixed Address Information.
        CSL_MPU_GetFixedAddressInfo (hMpu, &startAddress, &endAddress, &permissions);

        // Allow user executable permissions.
        permissions = permissions | CSL_FMK (MPU_FIXED_MPPA_UX, 1); 

        // Set the permissions for the fixed address
        CSL_MPU_SetFixedAddressPermissions (permissions);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_SetFixedAddressPermissions 
(
    CSL_MpuHandle   hMpu,
    Uint32          permissions
)
{
    hMpu->FIXED_MPPA = permissions;
}

/** ============================================================================
 *   @n@b CSL_MPU_GetProgrammableAddressInfo
 *
 *   @b Description
 *   @n This function gets the fixed address information.
 *
 *   @b Arguments
     @verbatim
        addressIndex    - Programmable Region Index.
        startAddress    - Programmable Start Address
        endAddress      - Programmable End Address
        permissions     - Programmable region permissions.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_PROG_START_ADDRESS, MPU_PROG_END_ADDRESS, MPU_PROG_MPPA
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint32          startAddress,
        Uint32          endAddress,
        Uint32          permissions

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the MPU Programmable Address for Region 1
        CSL_MPU_GetProgrammableAddressInfo (hMpu, 1, &startAddress, &endAddress, &permissions);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetProgrammableAddressInfo 
(
    CSL_MpuHandle   hMpu,
    Uint8           addressIndex,
    Uint32*         startAddress,
    Uint32*         endAddress,
    Uint32*         permissions
)
{
    *startAddress = hMpu->PROG_REGION[addressIndex].PROG_START_ADDRESS;
    *endAddress   = hMpu->PROG_REGION[addressIndex].PROG_END_ADDRESS;
    *permissions  = hMpu->PROG_REGION[addressIndex].PROG_MPPA;
}

/** ============================================================================
 *   @n@b CSL_MPU_SetProgrammableAddressInfo
 *
 *   @b Description
 *   @n This function sets the fixed address information.
 *
 *   @b Arguments
     @verbatim
        addressIndex    - Programmable Region Index.
        startAddress    - Programmable Start Address
        endAddress      - Programmable End Address
        permissions     - Programmable region permissions.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_PROG_START_ADDRESS, MPU_PROG_END_ADDRESS, MPU_PROG_MPPA
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint32          startAddress,
        Uint32          endAddress,
        Uint32          permissions

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the MPU Programmable Address for Region 1
        CSL_MPU_GetProgrammableAddressInfo (hMpu, 1, &startAddress, &endAddress, &permissions);

        // Allow user executable permissions.
        permissions = permissions | CSL_FMK (MPU_FIXED_MPPA_UX, 1);         

        // Set the permissions for the Region 1.
        CSL_MPU_SetProgrammableAddressInfo (hMpu, 1, startAddress, endAddress, permissions);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_SetProgrammableAddressInfo 
(
    CSL_MpuHandle   hMpu,
    Uint8           addressIndex,
    Uint32          startAddress,
    Uint32          endAddress,
    Uint32          permissions
)
{
    hMpu->PROG_REGION[addressIndex].PROG_START_ADDRESS = startAddress;
    hMpu->PROG_REGION[addressIndex].PROG_END_ADDRESS   = endAddress;
    hMpu->PROG_REGION[addressIndex].PROG_MPPA          = permissions;
}

/** ============================================================================
 *   @n@b CSL_MPU_GetFaultAddress
 *
 *   @b Description
 *   @n This function gets the address of the first protecton fault transfer.
 *
 *   @b Arguments
     @verbatim
        faultAddress     - Fault Address.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_FAULT_ADDRESS
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint32          faultAddress;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the Fault Address
        CSL_MPU_GetFaultAddress(hMpu, &faultAddress);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetFaultAddress 
(
    CSL_MpuHandle   hMpu,
    Uint32*         faultAddress
)
{
    *faultAddress = hMpu->FAULT_ADDRESS; 
}

/** ============================================================================
 *   @n@b CSL_MPU_GetFaultStatus
 *
 *   @b Description
 *   @n This function gets the fault status and attributes of the first 
 *      protection fault transfer
 *
 *   @b Arguments
     @verbatim
        faultStatus     - Fault Status
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MPU_FAULT_STATUS
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;
        Uint32          faultStatus;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Get the Fault Status
        CSL_MPU_GetFaultStatus(hMpu, &faultStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_GetFaultStatus 
(
    CSL_MpuHandle   hMpu,
    Uint32*         faultStatus
)
{
    *faultStatus = hMpu->FAULT_STATUS; 
}

/** ============================================================================
 *   @n@b CSL_MPU_ClearFaultStatus
 *
 *   @b Description
 *   @n This function clears the current fault so that another can be captured.
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_MPU_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MPU_FAULT_CLEAR
 *
 *   @b Affects
 *   @n MPU_FAULT_STATUS=0
 *
 *   @b Example
 *   @verbatim
        CSL_MpuHandle   hMpu;

        // Open the MPU MPU Module 0
        hMpu = CSL_MPU_Open (0);
        ...
        // Clear the Fault.
        CSL_MPU_ClearFaultStatus(hMpu);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MPU_ClearFaultStatus 
(
    CSL_MpuHandle   hMpu
)
{
    hMpu->FAULT_CLEAR = CSL_FMK(MPU_FAULT_CLEAR_FAULT_CLR, 1); 
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _CSL_MPUAUX_H_ */



