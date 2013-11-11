/**
 *   @file  csl_cgemAux.h
 *
 *   @brief   
 *      This is the CGEM Auxilary Header file.
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

#ifndef _CSL_CGEMAUX_H_
#define _CSL_CGEMAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_cgem.h>

/** @addtogroup CSL_CGEM_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_CGEM_getECFGPriorityLevel
 *
 *   @b Description
 *   @n This function gets the UMC priority level of MDMA requests.
 *
 *   @b Arguments
     @verbatim
        priorityLevel   - Priority Level populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_ECFGARBE_PRI
 *
 *   @b Example
 *   @verbatim
        Uint8   priorityLevel;
 
        CSL_CGEM_getECFGPriorityLevel (&priorityLevel);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getECFGPriorityLevel(Uint8* priorityLevel)
{
    *priorityLevel = CSL_FEXT(hCGEM->ECFGARBE, CGEM_ECFGARBE_PRI);
}

/** ============================================================================
 *   @n@b CSL_CGEM_setECFGPriorityLevel
 *
 *   @b Description
 *   @n This function sets the UMC priority level of MDMA requests.
 *
 *   @b Arguments
     @verbatim
        priorityLevel   - Priority Level to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_ECFGARBE_PRI
 *
 *   @b Example
 *   @verbatim
 
        CSL_CGEM_setECFGPriorityLevel (1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_setECFGPriorityLevel(Uint8 priorityLevel)
{
    CSL_FINS(hCGEM->ECFGARBE, CGEM_ECFGARBE_PRI, priorityLevel);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getEMCFaultAddress
 *
 *   @b Description
 *   @n This function gets the EMC Fault address 
 *
 *   @b Arguments
     @verbatim
        faultAddress   - Fault Address populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_ICFGMPFAR
 *
 *   @b Example
 *   @verbatim
        Uint32   faultAddress;
 
        CSL_CGEM_getEMCFaultAddress (&faultAddress);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getEMCFaultAddress(Uint32* faultAddress)
{
    *faultAddress = hCGEM->ICFGMPFAR;
}

/** ============================================================================
 *   @n@b CSL_CGEM_getEMCFaultStatus
 *
 *   @b Description
 *   @n This function gets the EMC Fault status
 *
 *   @b Arguments
     @verbatim
        faultStatus   - Fault Status populated by this API
        faultId       - Fault Identifier populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_ICFGMPFSR_FID
 *
 *   @b Example
 *   @verbatim
        Uint8   faultStatus;
        Uint8   faultId;

        // Get the EMC Fault Status  
        CSL_CGEM_getEMCFaultStatus (&faultStatus, &faultId);
        if (faultStatus != 0)
        {
            // Fault Detected
        }
        else
        {
            // No Fault 
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getEMCFaultStatus(Uint8* faultStatus, Uint8* faultId)
{
    Uint32  value = hCGEM->ICFGMPFSR;

    *faultStatus = CSL_FEXTR(value, 7, 0);
    *faultId     = CSL_FEXT(value, CGEM_ICFGMPFSR_FID);
}

/** ============================================================================
 *   @n@b CSL_CGEM_clearEMCFault
 *
 *   @b Description
 *   @n This function clears the EMC Fault 
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_ICFGMPFCR_MPFCLR=1
 *
 *   @b Affects
 *   @n CGEM_ICFGMPFSR=0
 *
 *   @b Example
 *   @verbatim
        Uint8   faultStatus;
        Uint8   faultId;

        // Get the EMC Fault Status  
        CSL_CGEM_getEMCFaultStatus (&faultStatus, &faultId);
        if (faultStatus != 0)
        {
            // Fault Detected
            ...
            // Clear the EMC Fault
            CSL_CGEM_clearEMCFault();
        }
        else
        {
            // No Fault 
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_clearEMCFault(void)
{
    hCGEM->ICFGMPFCR = CSL_FMK(CGEM_ICFGMPFCR_MPFCLR, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getECFGErrorStatus
 *
 *   @b Description
 *   @n This function gets the ECFG Error Status
 *
 *   @b Arguments
     @verbatim
        stat    -   Transaction Status populated by this API
        xid     -   Transaction ID populated by this API
        err     -   Error Detected populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_ECFGERR_STAT, CGEM_ECFGERR_XID, CGEM_ECFGERR_ERR
 *
 *   @b Example
 *   @verbatim
        Uint8   stat;
        Uint8   xid;
        Uint8   err;

        // Get the ECFG Error Status 
        CSL_CGEM_getECFGErrorStatus (&stat, &xid, &err);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getECFGErrorStatus
(
    Uint8*  stat,
    Uint8*  xid,
    Uint8*  err
)
{
    Uint32  value = hCGEM->ECFGERR;

    *stat = CSL_FEXT(value, CGEM_ECFGERR_STAT);
    *xid  = CSL_FEXT(value, CGEM_ECFGERR_XID);
    *err  = CSL_FEXT(value, CGEM_ECFGERR_ERR);
}

/** ============================================================================
 *   @n@b CSL_CGEM_clearECFGError
 *
 *   @b Description
 *   @n This function clears the ECFG Error Status
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_ECFGERRCLR_CLR=1
 *
 *   @b Affects
 *   @n CGEM_ECFGERR=0
 *
 *   @b Example
 *   @verbatim
        Uint8   stat;
        Uint8   xid;
        Uint8   err;

        // Get the ECFG Error Status 
        CSL_CGEM_getECFGErrorStatus (&stat, &xid, &err);
        ...
        // Clear the ECFG Error
        CSL_CGEM_clearECFGError();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_clearECFGError(void)
{
    hCGEM->ECFGERRCLR = CSL_FMK(CGEM_ECFGERRCLR_CLR, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_isL2EDCEnabled
 *
 *   @b Description
 *   @n This function is used to check if the L2 EDC is enabled or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE  - L2 EDC is enabled.
 *   @n  FALSE - L2 EDC is disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L2EDSTAT_EN
 *
 *   @b Example
 *   @verbatim

        // Check if L2-EDC is enabled or not?
        if (CSL_CGEM_isL2EDCEnabled() == TRUE)
        {
            // L2 EDC is enabled
        }
        else
        {
            // L2 EDC is disabled.        
        }
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_CGEM_isL2EDCEnabled(void)
{
    return (Bool)CSL_FEXT(hCGEM->L2EDSTAT, CGEM_L2EDSTAT_EN);
}

/** ============================================================================
 *   @n@b CSL_CGEM_isL2EDCSuspended
 *
 *   @b Description
 *   @n This function is used to check if the L2 EDC is suspended or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  TRUE  - L2 EDC is suspended.
 *   @n  FALSE - L2 EDC is NOT suspended.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L2EDSTAT_SUSP
 *
 *   @b Example
 *   @verbatim

        // Check if L2-EDC is suspended or not?
        if (CSL_CGEM_isL2EDCSuspended() == TRUE)
        {
            // L2 EDC is suspended
        }
        else
        {
            // L2 EDC is not suspended.        
        }
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_CGEM_isL2EDCSuspended(void)
{
    return (Bool)CSL_FEXT(hCGEM->L2EDSTAT, CGEM_L2EDSTAT_SUSP);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getL2EDCErrorStatus
 *
 *   @b Description
 *   @n This function gets the error status associated with the L2 EDC.
 *
 *   @b Arguments
     @verbatim
        derr        - DERR field populated by this API
        perr        - Parity Check error field populated by this API
        dmaerr      - DMA Error populated by this API
        verr        - Victim Error field populated by this API
        nerr        - NERR field populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L2EDSTAT_DERR, CGEM_L2EDSTAT_PERR, CGEM_L2EDSTAT_DMAERR,
 *   @n CGEM_L2EDSTAT_VERR, CGEM_L2EDSTAT_NERR 
 *
 *   @b Example
 *   @verbatim

        Uint8  derr;
        Uint8  perr;
        Uint8  dmaerr;
        Uint8  verr;
        Uint8  nerr;
 
        // Get the L2 EDC Error Status
        CSL_CGEM_getL2EDCErrorStatus(&derr, &perr, &dmaerr, &verr, &nerr);
 
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getL2EDCErrorStatus
(
    Uint8*  derr,
    Uint8*  perr,
    Uint8*  dmaerr,
    Uint8*  verr,
    Uint8*  nerr
)
{
    Uint32  value = hCGEM->L2EDSTAT;

    *derr   = CSL_FEXT(value, CGEM_L2EDSTAT_DERR);
    *perr   = CSL_FEXT(value, CGEM_L2EDSTAT_PERR);
    *dmaerr = CSL_FEXT(value, CGEM_L2EDSTAT_DMAERR);
    *verr   = CSL_FEXT(value, CGEM_L2EDSTAT_VERR);
    *nerr   = CSL_FEXT(value, CGEM_L2EDSTAT_NERR);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getL2EDCBitPosition
 *
 *   @b Description
 *   @n This function gets the bit position on which 128bit half of the 256bit 
 *   word caused the error.
 *
 *   @b Arguments
     @verbatim
        bitPos       - BitPos populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L2EDSTAT_BITPOS
 *
 *   @b Example
 *   @verbatim

        Uint8  bitPos;
 
        // Get the L2 EDC Bit Position
        CSL_CGEM_getL2EDCBitPosition(&bitPos);
 
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getL2EDCBitPosition (Uint8* bitPos)
{
    *bitPos = CSL_FEXT(hCGEM->L2EDSTAT, CGEM_L2EDSTAT_BITPOS);
}

/** ============================================================================
 *   @n@b CSL_CGEM_clearL2EDCErrorStatus
 *
 *   @b Description
 *   @n This function clears the error status associated with the L2 EDC.
 *
 *   @b Arguments
     @verbatim
        derr        - DERR field to be cleared.
        perr        - Parity Check Error field to be cleared.
        dmaerr      - DMA Error to be cleared.
        verr        - Victim Error to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCMD_DCLR, CGEM_L2EDCMD_PCLR, CGEM_L2EDCMD_DMACLR,
 *   @n CGEM_L2EDCMD_VCLR 
 *  
 *   @b Affects
 *   @n CGEM_L2EDSTAT_DERR=0, CGEM_L2EDSTAT_PERR=0, CGEM_L2EDSTAT_DMAERR=0,
 *   @n CGEM_L2EDSTAT_VERR=0, CGEM_L2EDSTAT_NERR=0 
 *
 *   @b Example
 *   @verbatim

        Uint8  derr;
        Uint8  perr;
        Uint8  dmaerr;
        Uint8  verr;
        Uint8  nerr;
 
        // Get the L2 EDC Error Status
        CSL_CGEM_getL2EDCErrorStatus(&derr, &perr, &dmaerr, &verr, &nerr);
        ...
        // Clear the L2 EDC Error Status
        CSL_CGEM_clearL2EDCErrorStatus(derr, perr, dmaerr, verr);
        ... 
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_clearL2EDCErrorStatus
(
    Uint8  derr,
    Uint8  perr,
    Uint8  dmaerr,
    Uint8  verr
)
{
    Uint32 value = 0;

    /* We dont want to touch the SUSP, DIS or EN bits in the L2EDCMD register */
    CSL_FINS(value, CGEM_L2EDCMD_DCLR, derr);
    CSL_FINS(value, CGEM_L2EDCMD_PCLR, perr);
    CSL_FINS(value, CGEM_L2EDCMD_DMACLR, dmaerr);
    CSL_FINS(value, CGEM_L2EDCMD_VCLR, verr);

    /* Since we are configuring the SUSP, DIS or EN bits i.e. they are set to 0. The 
     * write here to the register should not have any affect on the ED State. */
    hCGEM->L2EDCMD = value;
}

/** ============================================================================
 *   @n@b CSL_CGEM_enableL2EDC
 *
 *   @b Description
 *   @n This function enables the L2 EDC 
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCMD_EN=1
 *   
 *   @b Affects
 *   @n CGEM_L2EDSTAT_EN=1
 *
 *   @b Example
 *   @verbatim

        // Enable L2 EDC
        CSL_CGEM_enableL2EDC();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enableL2EDC(void)
{
    hCGEM->L2EDCMD = CSL_FMK(CGEM_L2EDCMD_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disableL2EDC
 *
 *   @b Description
 *   @n This function disables the L2 EDC 
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCMD_DIS=1
 *   
 *   @b Affects
 *   @n CGEM_L2EDSTAT_DIS=1
 *
 *   @b Example
 *   @verbatim

        // Disable L2 EDC
        CSL_CGEM_disableL2EDC();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disableL2EDC(void)
{
    hCGEM->L2EDCMD = CSL_FMK (CGEM_L2EDCMD_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_suspendL2EDC
 *
 *   @b Description
 *   @n This function suspends the L2 EDC 
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCMD_SUSP=1
 *   
 *   @b Affects
 *   @n CGEM_L2EDSTAT_SUSP=1
 *
 *   @b Example
 *   @verbatim

        // Suspend L2 EDC
        CSL_CGEM_suspendL2EDC();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_suspendL2EDC(void)
{
    hCGEM->L2EDCMD = CSL_FMK (CGEM_L2EDCMD_SUSP, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getL2EDCErrorInfo
 *
 *   @b Description
 *   @n This function gets the L2 EDC Error Information
 *
 *   @b Arguments
     @verbatim
        errAddr     - Error Address populated by this API
        l2way       - The way in the cache which caused the error.
        ram         - Flag which indicates if the error was detected in CACHE or RAM
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L2EDADDR_ADDR,CGEM_L2EDADDR_WAY,CGEM_L2EDADDR_RAM 
 *   
 *   @b Example
 *   @verbatim

        Uint32 errAddr;
        Uint8  l2way;
        Uint8  ram;

        // Get the L2 EDC Error Information.
        CSL_CGEM_getL2EDCErrorInfo(&errAddr, &l2way, &ram);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getL2EDCErrorInfo(Uint32* errAddr, Uint8* l2way, Uint8* ram)
{
    Uint32  value = hCGEM->L2EDADDR;

    *errAddr = CSL_FEXT(value, CGEM_L2EDADDR_ADDR);
    *l2way   = CSL_FEXT(value, CGEM_L2EDADDR_WAY);
    *ram     = CSL_FEXT(value, CGEM_L2EDADDR_RAM);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getL2EDCCounters
 *
 *   @b Description
 *   @n This function gets the L2 EDC Error event counters
 *
 *   @b Arguments
     @verbatim
        correctParityCnt    -   Correctable Parity Error Counters
        nonCorrectParityCnt -   Non-Correctable Parity Error Counters
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L2EDADDR_ADDR,CGEM_L2EDADDR_WAY,CGEM_L2EDADDR_RAM 
 *   
 *   @b Example
 *   @verbatim

        Uint8  correctParityCnt;
        Uint8  nonCorrectParityCnt;

        // Get the L2 EDC Error event counters.
        CSL_CGEM_getL2EDCCounters(&correctParityCnt, &nonCorrectParityCnt);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getL2EDCCounters(Uint8* correctParityCnt, Uint8* nonCorrectParityCnt)
{
    *correctParityCnt    = CSL_FEXT(hCGEM->L2EDCPEC, CGEM_L2EDCPEC_CNT);
    *nonCorrectParityCnt = CSL_FEXT(hCGEM->L2EDCNEC, CGEM_L2EDCNEC_CNT);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getMDMAErrorStatus
 *
 *   @b Description
 *   @n This function gets the MDMA Error Status
 *
 *   @b Arguments
     @verbatim
        stat    -   Transaction Status populated by this API
        xid     -   Transaction ID populated by this API
        err     -   Error Detected populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_MDMAERR_STAT, CGEM_MDMAERR_XID, CGEM_MDMAERR_ERR
 *
 *   @b Example
 *   @verbatim
        Uint8   stat;
        Uint8   xid;
        Uint8   err;

        // Get the MDMA Error Status
        CSL_CGEM_getMDMAErrorStatus(&stat, &xid, &err);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getMDMAErrorStatus(Uint8* stat, Uint8* xid, Uint8* err)
{
    Uint32  value = hCGEM->MDMAERR;

    *stat = CSL_FEXT(value, CGEM_MDMAERR_STAT);
    *xid  = CSL_FEXT(value, CGEM_MDMAERR_XID);
    *err  = CSL_FEXT(value, CGEM_MDMAERR_ERR);
}

/** ============================================================================
 *   @n@b CSL_CGEM_clearMDMAError
 *
 *   @b Description
 *   @n This function clears the MDMA Error Status
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_MDMAERRCLR_CLR=1
 *
 *   @b Affects
 *   @n CGEM_MDMAERR_STAT=0, CGEM_MDMAERR_XID=0, CGEM_MDMAERR_ERR=0
 *
 *   @b Example
 *   @verbatim
        Uint8   stat;
        Uint8   xid;
        Uint8   err;

        // Get the MDMA Error Status
        CSL_CGEM_getMDMAErrorStatus(&stat, &xid, &err);
        ...
        // Clear the MDMA Error Status
        CSL_CGEM_clearMDMAError();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_clearMDMAError(void)
{
    hCGEM->MDMAERRCLR = CSL_FMK(CGEM_MDMAERRCLR_CLR, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_enableEDCDMCExternal
 *
 *   @b Description
 *   @n This function is used to enable the EDC on DMC reads from an external 
 *      address(Hits L2 cache) if L2EDCMD is enabled.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_DL2CEN=1
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_enableEDCDMCExternal();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enableEDCDMCExternal(void)
{
    hCGEM->L2EDCEN = CSL_FMK (CGEM_L2EDCEN_DL2CEN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disableEDCDMCExternal
 *
 *   @b Description
 *   @n This function is used to disable the EDC on DMC reads from an external 
 *      address(Hits L2 cache) if L2EDCMD is enabled.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_DL2CEN=0
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_disableEDCDMCExternal();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disableEDCDMCExternal(void)
{
    hCGEM->L2EDCEN = CSL_FMK (CGEM_L2EDCEN_DL2CEN, 0);
}

/** ============================================================================
 *   @n@b CSL_CGEM_enableEDCPMCExternal
 *
 *   @b Description
 *   @n This function is used to enable the EDC on PMC reads from an external 
 *      address(Hits L2 cache) if L2EDCMD is enabled.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_PL2CEN=1
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_enableEDCPMCExternal();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enableEDCPMCExternal(void)
{
    hCGEM->L2EDCEN = CSL_FMK (CGEM_L2EDCEN_PL2CEN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disableEDCPMCExternal
 *
 *   @b Description
 *   @n This function is used to disable the EDC on PMC reads from an external 
 *      address(Hits L2 cache) if L2EDCMD is enabled.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_PL2CEN=0
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_disableEDCPMCExternal();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disableEDCPMCExternal(void)
{
    hCGEM->L2EDCEN = CSL_FMK (CGEM_L2EDCEN_PL2CEN, 0);
}

/** ============================================================================
 *   @n@b CSL_CGEM_enableEDCDMCL2SRAM
 *
 *   @b Description
 *   @n This function is used to enable the EDC on DMC reads from a L2SRAM.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_DL2SEN=1
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_enableEDCDMCL2SRAM();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enableEDCDMCL2SRAM(void)
{
    hCGEM->L2EDCEN = CSL_FMK(CGEM_L2EDCEN_DL2SEN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disableEDCDMCL2SRAM
 *
 *   @b Description
 *   @n This function is used to disable the EDC on DMC reads from a L2SRAM.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_DL2SEN=0
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_disableEDCDMCL2SRAM();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disableEDCDMCL2SRAM(void)
{
    hCGEM->L2EDCEN = CSL_FMK(CGEM_L2EDCEN_DL2SEN, 0);
}

/** ============================================================================
 *   @n@b CSL_CGEM_enableEDCPMCL2SRAM
 *
 *   @b Description
 *   @n This function is used to enable the EDC on PMC reads from a L2SRAM.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_PL2SEN=1
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_enableEDCPMCL2SRAM();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enableEDCPMCL2SRAM(void)
{
    hCGEM->L2EDCEN = CSL_FMK(CGEM_L2EDCEN_PL2SEN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disableEDCPMCL2SRAM
 *
 *   @b Description
 *   @n This function is used to disable the EDC on PMC reads from a L2SRAM.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_PL2SEN=0
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_disableEDCPMCL2SRAM();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disableEDCPMCL2SRAM(void)
{
    hCGEM->L2EDCEN = CSL_FMK(CGEM_L2EDCEN_PL2SEN, 0);
}

/** ============================================================================
 *   @n@b CSL_CGEM_enableEDCSDMAL2SRAM
 *
 *   @b Description
 *   @n This function is used to enable the EDC on SDMA reads from L2SRAM.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_SDMAEN=1
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_enableEDCSDMAL2SRAM();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enableEDCSDMAL2SRAM(void)
{
    hCGEM->L2EDCEN = CSL_FMK(CGEM_L2EDCEN_SDMAEN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disableEDCSDMAL2SRAM
 *
 *   @b Description
 *   @n This function is used to disable the EDC on SDMA reads from L2SRAM.
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L2EDCEN_SDMAEN=0
 *
 *   @b Example
 *   @verbatim
        
        CSL_CGEM_disableEDCSDMAL2SRAM();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disableEDCSDMAL2SRAM(void)
{
    hCGEM->L2EDCEN = CSL_FMK (CGEM_L2EDCEN_SDMAEN, 0);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getPMCErrorDetectionStatus
 *
 *   @b Description
 *   @n This function is used to get the PMC error detection status
 *
 *   @b Arguments
     @verbatim
        dmaErr      - DMA Error Status populated by this API
        perr        - PERR Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L1PEDSTAT_DMAERR, CGEM_L1PEDSTAT_PERR
 *
 *   @b Example
 *   @verbatim
       
        Uint8 dmaErr;
        Uint8 perr

        CSL_CGEM_getPMCErrorDetectionStatus(&dmaErr, &perr);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getPMCErrorDetectionStatus(Uint8* dmaErr, Uint8* perr)
{
    Uint32  value = hCGEM->L1PEDSTAT;

    *dmaErr = CSL_FEXT (value, CGEM_L1PEDSTAT_DMAERR);
    *perr   = CSL_FEXT (value, CGEM_L1PEDSTAT_PERR);
}

/** ============================================================================
 *   @n@b CSL_CGEM_isPMCErrorDetectionEnabled
 *
 *   @b Description
 *   @n This function is used to check if the PMC Error Detection is enabled or
 *      not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   PMC Error Detection is enabled
 *   @n FALSE   -   PMC Error Detection is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L1PEDSTAT_EN
 *
 *   @b Example
 *   @verbatim
        
        if (CSL_CGEM_isPMCErrorDetectionEnabled() == TRUE)
        {
            // PMC Error Detection is enabled
        }
        else
        {
            // PMC Error Detection is disabled
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_CGEM_isPMCErrorDetectionEnabled(void)
{
    return CSL_FEXT(hCGEM->L1PEDSTAT, CGEM_L1PEDSTAT_EN);
}

/** ============================================================================
 *   @n@b CSL_CGEM_isPMCErrorDetectionSuspended
 *
 *   @b Description
 *   @n This function is used to check if the PMC Error Detection is suspended or
 *      not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   PMC Error Detection is suspended
 *   @n FALSE   -   PMC Error Detection is not suspended
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L1PEDSTAT_SUSP
 *
 *   @b Example
 *   @verbatim
        
        if (CSL_CGEM_isPMCErrorDetectionSuspended() == TRUE)
        {
            // PMC Error Detection is suspended
        }
        else
        {
            // PMC Error Detection is not suspended
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_CGEM_isPMCErrorDetectionSuspended(void)
{
    return CSL_FEXT(hCGEM->L1PEDSTAT, CGEM_L1PEDSTAT_SUSP);
}

/** ============================================================================
 *   @n@b CSL_CGEM_clearPMCErrorDetectionStatus
 *
 *   @b Description
 *   @n This function is used to clear the PMC error detection status
 *
 *   @b Arguments
     @verbatim
        dmaErr      - DMA Error status to be cleared
        perr        - PERR status to be cleared
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L1PEDCMD_DMACLR, CGEM_L1PEDCMD_PCLR
 *
 *   @b Affects
 *   @n CGEM_L1PEDSTAT_DMAERR=0, CGEM_L1PEDSTAT_PERR=0
 *
 *   @b Example
 *   @verbatim
       
        Uint8 dmaErr;
        Uint8 perr

        // Get the PMC Error Detection status
        CSL_CGEM_getPMCErrorDetectionStatus(&dmaErr, &perr);
        ...
        // Clear the PMC errors.
        CSL_CGEM_clearPMCErrorDetectionStatus(dmaErr, perr);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_clearPMCErrorDetectionStatus(Uint8 dmaErr, Uint8 perr)
{
    Uint32 value = 0;

    /* We dont want to touch the EN, SUSP or DIS bits in the L1PEDCMD register */
    CSL_FINS (value, CGEM_L1PEDCMD_DMACLR, dmaErr);
    CSL_FINS (value, CGEM_L1PEDCMD_PCLR,   perr);

    /* Since we are writing 0 to the SUSP, DIS or EN bits we should preserve the ED 
     * state as indicated in the specification. */
    hCGEM->L1PEDCMD = value;
}

/** ============================================================================
 *   @n@b CSL_CGEM_enablePMCErrorDetection
 *
 *   @b Description
 *   @n This function is used to enable the PMC error detection status
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L1PEDCMD_EN=1
 *
 *   @b Affects
 *   @n CGEM_L1PEDSTAT_EN=1
 *
 *   @b Example
 *   @verbatim
       
        // Enable the PMC Error Detection
        CSL_CGEM_enablePMCErrorDetection();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_enablePMCErrorDetection(void)
{
    hCGEM->L1PEDCMD = CSL_FMK (CGEM_L1PEDCMD_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_disablePMCErrorDetection
 *
 *   @b Description
 *   @n This function is used to disable the PMC error detection status
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L1PEDCMD_DIS=1
 *
 *   @b Affects
 *   @n CGEM_L1PEDSTAT_DIS=1
 *
 *   @b Example
 *   @verbatim
       
        // Disable the PMC Error Detection
        CSL_CGEM_disablePMCErrorDetection();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_disablePMCErrorDetection(void)
{
    hCGEM->L1PEDCMD = CSL_FMK (CGEM_L1PEDCMD_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_suspendPMCErrorDetection
 *
 *   @b Description
 *   @n This function is used to suspend the PMC error detection status
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
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_L1PEDCMD_SUSP=1
 *
 *   @b Affects
 *   @n CGEM_L1PEDSTAT_SUSP=1
 *
 *   @b Example
 *   @verbatim
       
        // Suspend the PMC Error Detection
        CSL_CGEM_suspendPMCErrorDetection();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_suspendPMCErrorDetection(void)
{
    hCGEM->L1PEDCMD = CSL_FMK (CGEM_L1PEDCMD_SUSP, 1);
}

/** ============================================================================
 *   @n@b CSL_CGEM_getPMCErrorInfo
 *
 *   @b Description
 *   @n This function is used to get the PMC Error Detection information
 *
 *   @b Arguments
     @verbatim
        errAddress  -   Error Address which is populated by this API
        errRAM      -   Error RAM status which is populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_L1PEDADDR_ADDR, CGEM_L1PEDADDR_RAM
 *
 *   @b Example
 *   @verbatim
      
        Uint32  errAddress;
        Uint8   errRAM;

        // Get the PMC Error Detection Information
        CSL_CGEM_getPMCErrorInfo(&errAddress, &errRAM);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_CGEM_getPMCErrorInfo(Uint32* errAddress, Uint8* errRAM)
{
    Uint32 value = hCGEM->L1PEDADDR;

    *errAddress = CSL_FEXT (value, CGEM_L1PEDADDR_ADDR);
    *errRAM     = CSL_FEXT (value, CGEM_L1PEDADDR_RAM);
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _CSL_CGEMAUX_H_ */


