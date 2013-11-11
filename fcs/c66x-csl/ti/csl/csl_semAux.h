/**
 *   @file  csl_semAux.h
 *
 *   @brief   
 *      This is the SEM Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the SEM Module.
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

#ifndef _CSL_SEMAUX_H_
#define _CSL_SEMAUX_H_

#include <ti/csl/csl_sem.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_SEM_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_semSetEoi
 *
 *   @b Description
 *   @n This function sets the SEM Module EOI Register to rearm the interrupt line
 *      after processing an existing interrupt.
 *
 *   @b Arguments
     @verbatim
          eoiVal   Value to be written to the EOI register.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n SEM_SEM_EOI_SELECT_INTERRUPT_REARM
 *
 *   <b> Post Condition </b>
 *   @n Semaphore interrupts to the specific Master are reenabled.
 *
 *   @b Example
 *   @verbatim

        // Reenable Semaphore interrupts for CPU Core 0.
        CSL_semSetEoi (semHandle, CSL_SEM_REARM_SEMINT0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_semSetEoi (Uint8 eoiVal)
{
    hSEM->SEM_EOI = CSL_FMK(SEM_SEM_EOI_SELECT_INTERRUPT_REARM, eoiVal);
}         

/** ============================================================================
 *   @n@b CSL_semAcquireDirect
 *
 *   @b Description
 *   @n This function acquires a semaphore for direct access.
 *
 *   @b Arguments
     @verbatim
          semNum      Semaphore which is to be acquired.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  1 - Resource was available and has been allocated to the Master
 *   @n  0 - Resource was NOT available and has NOT been allocated to the Master
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n Semaphore is allocated to the Master on Success.
 *
 *   @b Reads
 *   @n SEM_SEM_FREE
 *
 *   @b Example
 *   @verbatim

        Uint8 status;

        // Acquire the semaphore number 2.
        status = CSL_semAcquireDirect (2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_semAcquireDirect (Uint8 semNum)
{
    /* Direct Access Request: Read from the SEMn register */
    return CSL_FEXT (hSEM->SEM[semNum], SEM_SEM_FREE);
}

/** ============================================================================
 *   @n@b CSL_semAcquireIndirect
 *
 *   @b Description
 *   @n This function acquires a semaphore for indirect access.
 *
 *   @b Arguments
     @verbatim
          semNum      Semaphore which is to be acquired.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n Semaphore is placed into the request queue to be acquired.
 *
 *   @b Writes
 *   @n SEM_SEM_FREE=0
 *
 *   @b Example
 *   @verbatim

        // Acquire the semaphore number 2.
        CSL_semAcquireIndirect (2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_semAcquireIndirect (Uint8 semNum)
{
    /* Indirect Access Request: Write a 0 to the SEMn register; to request for the resource. */
    hSEM->SEM[semNum] = CSL_FMK (SEM_SEM_FREE, 0);
}

/** ============================================================================
 *   @n@b CSL_semAcquireCombined
 *
 *   @b Description
 *   @n This function acquires a semaphore for combined access. 
 *
 *   @b Arguments
     @verbatim
          semNum      Semaphore which is to be acquired.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n Semaphore is acquired immediately if free else it is  placed into the request 
 *      queue to be acquired.
 *
 *   @b Reads
 *   @n SEM_ISEM_FREE
 *
 *   @b Example
 *   @verbatim

        Uint8 status;

        // Acquire the semaphore number 2.
        status = CSL_semAcquireCombined (2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_semAcquireCombined(Uint8 semNum)
{
    /* Combined Access Request: Read from the Indirect register */
    return CSL_FEXT (hSEM->ISEM[semNum], SEM_ISEM_FREE);
}

/** ============================================================================
 *   @n@b CSL_semReleaseSemaphore
 *
 *   @b Description
 *   @n This function is used to release a sempahore which had been acquired
 *      previously.
 *
 *   @b Arguments
     @verbatim
          semNum      Semaphore which is to be released.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n @a CSL_semAcquireDirect() or @a CSL_semAcquireCombined() @a CSL_semAcquireIndirect()
 *      must be called.
 *
 *   <b> Post Condition </b>
 *   @n Semaphore resource is freed up 
 *
 *   @b Writes
 *   @n SEM_SEM_FREE=1
 *
 *   @b Example
 *   @verbatim

        // Acquire the semaphore using the direct method. 
        CSL_semAcquireDirect (2);
        ...
        // Release the semaphore.
        CSL_semReleaseSemaphore (2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_semReleaseSemaphore (Uint8 semNum)
{
    /* Semaphore is released up by writing a 1 to the SEMn register */
    hSEM->SEM[semNum] = CSL_FMK (SEM_SEM_FREE, 1);
}

/** ============================================================================
 *   @n@b CSL_semIsFree
 *
 *   @b Description
 *   @n This function checks if the specified semaphore is acquired or not?
 *
 *   @b Arguments
     @verbatim
          semNum      Semaphore which is to be queried.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE  - Semaphore is Free
 *   @n  FALSE - Semaphore is Available   
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SEM_QSEM_FREE
 *
 *   @b Example
 *   @verbatim

        Bool isFree;

        // Check if Semaphore is available.
        isFree = CSL_semIsFree (2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_semIsFree(Uint8 semNum)
{
    /* Read the Query Status register to determine the semaphore status. */
    return (Bool)CSL_FEXT (hSEM->QSEM[semNum], SEM_QSEM_FREE);
}

/** ============================================================================
 *   @n@b CSL_semGetResourceOwner
 *
 *   @b Description
 *   @n This function returns the resource owner for the specific semaphore.
 *
 *   @b Arguments
     @verbatim
          semNum      Semaphore Number whose resource owner is to be retreived.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Semaphore Resource Owner.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SEM_QSEM_OWNER
 *
 *   @b Example
 *   @verbatim

        Bool    isFree;
        Uint8   owner;

        // Check if Semaphore is available.
        isFree = CSL_semIsFree (2);
        if (isFree == FALSE)
        {
            // Resource is acquired. Get the owner 
            owner = CSL_semGetResourceOwner(2);
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_semGetResourceOwner(Uint8 semNum)
{
    return (Uint8)CSL_FEXT(hSEM->QSEM[semNum], SEM_QSEM_OWNER);
}

/** ============================================================================
 *   @n@b CSL_semGetFlags
 *
 *   @b Description
 *   @n This function returns the contents of the SEMFLAG register for a specific
 *      master. This is used in the indirect or combined access method to determine
 *      if a resource has been acquired. The corresponding bit indicates that the 
 *      specific resource number which had been requested is now available.
 *
 *   @b Arguments
     @verbatim
          masterId      Master ID for which the SEMFLAG register is read.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SEM_SEMFLAGL_CLEAR
 *
 *   @b Example
 *   @verbatim

        Uint32  semFlags;

        // Read the bits of the semaphore flag register for master 0.
        semFlags = CSL_semGetFlags(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_semGetFlags(Uint8 masterId)
{
    return (Uint32)hSEM->SEMFLAGL_CLEAR[masterId];
}

/** ============================================================================
 *   @n@b CSL_semClearFlags
 *
 *   @b Description
 *   @n This function is used by the host by the host to inform the SEM Module 
 *      that the resource which had been marked as FREE has been processed by
 *      the host.
 *
 *   @b Arguments
     @verbatim
          masterId      Master ID for which the SEMFLAG register is read.
          semFlags      Semaphore Flags retreived through the CSL_semGetFlags API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n SEM_SEMFLAGL_CLEAR
 *
 *   @b Affects
 *   @n SEM_SEMFLAGL_CLEAR
 *
 *   @b Example
 *   @verbatim

        Uint32  semFlags;

        // Read the bits of the semaphore flag register for master 0.
        semFlags = CSL_semGetFlags(0);
        ...
        // Clear the bits of the semaphore flag register for master 0.
        CSL_semClearFlags (0, semFlags);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_semClearFlags(Uint8 masterId, Uint32 semFlags)
{
    /* Write the value to the CLEAR Register. */ 
    hSEM->SEMFLAGL_CLEAR[masterId] = semFlags;
}

/** ============================================================================
 *   @n@b CSL_semGetErrorCode
 *
 *   @b Description
 *   @n This function is used to get the Error code.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *   @n  Error Code 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SEM_SEMERR_ERR
 *
 *   @b Example
 *   @verbatim

        Uint8  errCode;

        // Check if there is an error which has been reported.
        errCode = CSL_semGetErrorCode();
        if (errCode != 0)
        {
            // Semaphore Module has reported an Error. 
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_semGetErrorCode (void)
{
    return (Uint8)CSL_FEXT(hSEM->SEMERR, SEM_SEMERR_ERR);
}

/** ============================================================================
 *   @n@b CSL_semGetErrorSemaphoreNumber
 *
 *   @b Description
 *   @n This function is used to get the semaphore number which is in error.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *   @n  Semaphore Number which caused the error.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SEM_SEMERR_SEMNUM
 *
 *   @b Example
 *   @verbatim

        Uint8  errCode;
        Uint8  semNumber;

        // Check if there is an error which has been reported.
        errCode = CSL_semGetErrorCode();
        if (errCode != 0)
        {
            // Semaphore Module has reported an error; get the semaphore number. 
            semNumber = CSL_semGetErrorSemaphoreNumber(); 
            ...
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_semGetErrorSemaphoreNumber (void)
{
    return (Uint8)CSL_FEXT(hSEM->SEMERR, SEM_SEMERR_SEMNUM);
}

/** ============================================================================
 *   @n@b CSL_semGetErrorFaultID
 *
 *   @b Description
 *   @n This function is used to get the resource owner which caused the error.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *   @n  Resource Number which caused the error.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SEM_SEMERR_FAULTID
 *
 *   @b Example
 *   @verbatim

        Uint8  errCode;
        Uint8  semNumber;
        Uint8  faultId;

        // Check if there is an error which has been reported.
        errCode = CSL_semGetErrorCode();
        if (errCode != 0)
        {
            // Semaphore Module has reported an error; get the semaphore number. 
            semNumber = CSL_semGetErrorSemaphoreNumber(); 

            // Get the fault id. 
            faultId = CSL_semGetErrorFaultID();
            ...
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_semGetErrorFaultID (void)
{
    return (Uint8)CSL_FEXT(hSEM->SEMERR, SEM_SEMERR_FAULTID);
}

/** ============================================================================
 *   @n@b CSL_semClearError
 *
 *   @b Description
 *   @n This function is used to clear the semaphore module error.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n SEM_SEMERR_CLEAR_CLRERR=1
 *
 *   @b Example
 *   @verbatim

        Uint8  errCode;
        Uint8  semNumber;
        Uint8  faultId;

        // Check if there is an error which has been reported.
        errCode = CSL_semGetErrorCode();
        if (errCode != 0)
        {
            // Semaphore Module has reported an error; get the semaphore number. 
            semNumber = CSL_semGetErrorSemaphoreNumber(); 

            // Get the fault id. 
            faultId = CSL_semGetErrorFaultID();
            ...

            // Clear the error.
            CSL_semClearError ();
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_semClearError (void)
{
    hSEM->SEMERR_CLEAR = CSL_FMK(SEM_SEMERR_CLEAR_CLRERR, 1);
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_SEMAUX_H_ */

