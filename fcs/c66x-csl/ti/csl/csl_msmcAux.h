/**
 *   @file  csl_msmcAux.h
 *
 *   @brief   
 *      This is the MSMC Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the MSMC Module.
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

#ifndef _CSL_MSMCAUX_H_
#define _CSL_MSMCAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_msmc.h>

/** @addtogroup CSL_MSMC_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_MSMC_getPID
 *
 *   @b Description
 *   @n This function gets the peripheral ID register for the MSMC IP.
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_PID
 *
 *   @b Example
 *   @verbatim
        Uint32   pid;

        // Get the Mode change progress flag
        mcpFlag = CSL_MSMC_getModeChangeProgress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getPID (void)
{
	return hMsmc->PID;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getModeChangeProgress
 *
 *   @b Description
 *   @n This function gets the mode change progress flag
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMCC_MCP
 *
 *   @b Example
 *   @verbatim
        Uint8   mcpFlag;

        // Get the Mode change progress flag
        mcpFlag = CSL_MSMC_getModeChangeProgress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getModeChangeProgress (void)
{
	return CSL_FEXT (hMsmc->SMCC, MSMC_SMCC_MCP);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSoftErrorAddress
 *
 *   @b Description
 *   @n When ECM bit is 1 in SMCFG register this function gets the corrupted
 *      location error address.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *      Sof Error Address.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMCERRAR_SEADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 errAddr;

        errAddr = CSL_MSMC_getSoftErrorAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getSoftErrorAddress (void)
{
	return CSL_FEXT (hMsmc->SMCERRAR, MSMC_SMCERRAR_SEADDR);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSoftErrorAddressExtension
 *
 *   @b Description
 *   @n This function gets the address extension used for the faulting address.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *      Soft Address Extension.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMCERRXR_ESYN, MSMC_SMCERRXR_SEEADDR, MSMC_SMCERRXR_SEPID,MSMC_SMCERRXR_SER
 *
 *   @b Example
 *   @verbatim
    	Uint8   esyn;
    	Uint8   errAddrExt;
    	Uint8   sepId;
    	Uint8   ser;

        CSL_MSMC_getSoftErrorAddressExtension (&esyn, &errAddrExt, &sepId, &ser);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getSoftErrorAddressExtension 
(
    Uint8* esyn, 
    Uint8* address, 
    Uint8* sepId, 
    Uint8* ser
)
{
    Uint32  value = hMsmc->SMCERRXR;

    *esyn    = CSL_FEXT (value, MSMC_SMCERRXR_ESYN);
    *address = CSL_FEXT (value, MSMC_SMCERRXR_SEEADDR);
    *sepId   = CSL_FEXT (value, MSMC_SMCERRXR_SEPID);
    *ser     = CSL_FEXT (value, MSMC_SMCERRXR_SER);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setCounterBankRefreshRead
 *
 *   @b Description
 *   @n This function programs REFDEL counter to the number of MSMC clock cycles
 *      between each bank refresh read access.
 *
 *   @b Arguments
     @verbatim
          refdel      number of MSMC clock cycles
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMEDCC_REFDEL
 *
 *   @b Example
 *   @verbatim

        CSL_MSMC_setCounterBankRefreshRead (1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setCounterBankRefreshRead (Uint32 refdel)
{
	CSL_FINS (hMsmc->SMEDCC, MSMC_SMEDCC_REFDEL, refdel);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getCounterBankRefreshRead
 *
 *   @b Description
 *   @n This function gets REFDEL counter to the number of MSMC clock cycles
 *      between each bank refresh read access.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *      REFDEL counter
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMEDCC_REFDEL
 *
 *   @b Example
 *   @verbatim
       	Uint32 refdel;

        refdel = CSL_MSMC_getCounterBankRefreshRead ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getCounterBankRefreshRead (void)
{
	return CSL_FEXT (hMsmc->SMEDCC, MSMC_SMEDCC_REFDEL);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getParityRAM
 *
 *   @b Description
 *   @n This function programs gets the parity RAM status
 *
 *   @b Arguments
     @verbatim
        None
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 Parity RAM
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMEDCC_PRR
 *
 *   @b Example
 *   @verbatim

        Uint8 parityRAM;

        parityRAM = CSL_MSMC_getParityRAM ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getParityRAM (void)
{
	return CSL_FEXT (hMsmc->SMEDCC, MSMC_SMEDCC_PRR);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getCSE
 *
 *   @b Description
 *   @n This function programs reads the CSE bit to determine if there is a parity
 *   mismatch
 *
 *   @b Arguments
     @verbatim
        None
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 CSE Status
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMEDCC_CSE
 *
 *   @b Example
 *   @verbatim

        Uint8 cse;

        cse = CSL_MSMC_getCSE ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getCSE (void)
{
	return CSL_FEXT (hMsmc->SMEDCC, MSMC_SMEDCC_CSE);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setCSE
 *
 *   @b Description
 *   @n This function programs sets the CSE bit 
 *
 *   @b Arguments
     @verbatim
        None
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMEDCC_CSE
 *
 *   @b Example
 *   @verbatim

        // Clear the CSE Bit.
        CSL_MSMC_setCSE (0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setCSE (Uint8 cse)
{
	CSL_FINS(hMsmc->SMEDCC, MSMC_SMEDCC_CSE, cse);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getECM
 *
 *   @b Description
 *   @n This function programs gets the status of the error correcting mode
 *
 *   @b Arguments
     @verbatim
        None
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 ECM Mode
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMEDCC_ECM
 *
 *   @b Example
 *   @verbatim

        Uint8 ecm;

        ecm = CSL_MSMC_getECM ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getECM (void)
{
	return CSL_FEXT (hMsmc->SMEDCC, MSMC_SMEDCC_ECM);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setECM
 *
 *   @b Description
 *   @n This function programs sets the error correcting mode.
 *
 *   @b Arguments
     @verbatim
        None
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMEDCC_ECM
 *
 *   @b Example
 *   @verbatim

        // Set Error Correcting Mode
        CSL_MSMC_setECM (0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setECM (Uint8 ecm)
{
	CSL_FINS(hMsmc->SMEDCC, MSMC_SMEDCC_ECM, ecm);
}

/** ============================================================================
 *   @n@b CSL_MSMC_enableScrubbingEngine
 *
 *   @b Description
 *   @n This function enables the scrubbing engine.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	Scrubbing Engine is enabled.
 *
 *   @b Writes
 *   @n MSMC_SMEDCC_SEN=0
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_enableScrubbingEngine ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_enableScrubbingEngine (void)
{
    CSL_FINS (hMsmc->SMEDCC, MSMC_SMEDCC_SEN, 0x0);
}

/** ============================================================================
 *   @n@b CSL_MSMC_disableScrubbingEngine
 *
 *   @b Description
 *   @n This function disables the scrubbing engine.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	Scrubbing Engine is disabled.
 *
 *   @b Writes
 *   @n MSMC_SMEDCC_SEN=1
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_disableScrubbingEngine ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_disableScrubbingEngine (void)
{
    CSL_FINS (hMsmc->SMEDCC, MSMC_SMEDCC_SEN, (Uint32)1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getScrubbingEngineState
 *
 *   @b Description
 *   @n This function gets the state of scrubbing engine, if it is
 *      enabled/disabled.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *   @li  1   -   Scrubbing Engine is enabled.
 *   @li  0   -   Scrubbing Engine is disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMEDCC_SEN
 *
 *   @b Example
 *   @verbatim
     	Uint32 stateSE;

        stateSE = CSL_MSMC_getScrubbingEngineState ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getScrubbingEngineState (void)
{
	return CSL_FEXT (hMsmc->SMEDCC, MSMC_SMEDCC_SEN);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getScrubErrorCorrectedAddress
 *
 *   @b Description
 *   @n This function gets address of the location whose contents have been
 *      corrected by the scrubbing engine.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMCEA_SECA
 *
 *   @b Example
 *   @verbatim
        Uint32 correctedAddr;

        correctedAddr = CSL_MSMC_getScrubErrorCorrectedAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getScrubErrorCorrectedAddress (void)
{
	return CSL_FEXT (hMsmc->SMCEA, MSMC_SMCEA_SECA);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSyndromeValue
 *
 *   @b Description
 *   @n This function gets the syndrome value that identifies the erroneous bit
 *   in the data which the scrubbing engine has corrected.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 ESYN Field
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	SMSECA register configured with the value zero.
 *
 *   @b Reads
 *   @n MSMC_SMCEA_ESYN
 *
 *   @b Example
 *   @verbatim
        Uint8   esynValue;

        esynValue = CSL_MSMC_getSyndromeValue ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getSyndromeValue (void)
{
	return CSL_FEXT (hMsmc->SMCEA, MSMC_SMCEA_ESYN);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getScrubCorrectableErrorCounter
 *
 *   @b Description
 *   @n This function gets the Scrubbing Engine Correctable Error Counter.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMSECC_SCEC
 *
 *   @b Example
 *   @verbatim
        Uint32 cCounter;

        cCounter = CSL_MSMC_getScrubCorrectableErrorCounter ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getScrubCorrectableErrorCounter (void)
{
	return CSL_FEXT (hMsmc->SMSECC, MSMC_SMSECC_SCEC);
}

/** ============================================================================
 *   @n@b CSL_MSMC_clearScrubCorrectableErrorCounter
 *
 *   @b Description
 *   @n This function clears the Scrub Correctable Error Counter.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n SMSECC register configured with the value passed.
 *
 *   @b Writes
 *   @n MSMC_SMSECC_SCEC=0
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_clearScrubCorrectableErrorCounter ();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_clearScrubCorrectableErrorCounter (void)
{
    CSL_FINS (hMsmc->SMSECC, MSMC_SMSECC_SCEC, 0);
    return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getScrubNonCorrectableErrorCounter
 *
 *   @b Description
 *   @n This function gets the Scrubbing Engine NonCorrectable Error Counter.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMSECC_SNCEC
 *
 *   @b Example
 *   @verbatim
        Uint32 ncCounter;

        ncCounter = CSL_MSMC_getScrubNonCorrectableErrorCounter ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getScrubNonCorrectableErrorCounter (void)
{
	return CSL_FEXT (hMsmc->SMSECC, MSMC_SMSECC_SNCEC);
}

/** ============================================================================
 *   @n@b CSL_MSMC_clearScrubNonCorrectableErrorCounter
 *
 *   @b Description
 *   @n his function clears the Scrub Non Correctable Error Counter.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	SMSECC register configured with the value passed.
 *
 *   @b Writes
 *   @n MSMC_SMSECC_SNCEC=0
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_clearScrubNonCorrectableErrorCounter ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_clearScrubNonCorrectableErrorCounter (void)
{
	CSL_FINS (hMsmc->SMSECC, MSMC_SMSECC_SNCEC, 0);
	return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getFaultAddress
 *
 *   @b Description
 *   @n This function gets the access address causing the fault.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *      Fault Address
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMPFAR_FAULT_ADDRESS
 *
 *   @b Example
 *   @verbatim
        Uint32 faultAddr;

        faultAddr = CSL_MSMC_getFaultAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getFaultAddress (void)
{
	return CSL_FEXT (hMsmc->SMPFAR, MSMC_SMPFAR_FAULT_ADDRESS);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getFaultAddressNMStatus
 *
 *   @b Description
 *   @n This function gets the NM Status which is set if the access address 
 *      causing the fault does not match any of the segment BADDR
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *      Fault Address NM Status
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMPFXR_NM
 *
 *   @b Example
 *   @verbatim
        Uint8 faultNMStatus;

        faultNMStatus = CSL_MSMC_getFaultAddressNMStatus ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getFaultAddressNMStatus(void)
{
	return CSL_FEXT (hMsmc->SMPFXR, MSMC_SMPFXR_NM);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getFaultInfo
 *
 *   @b Description
 *   @n This function gets the MSMC fault information. This include the event 
 *      bits  event bits for all PrivID's and the Master ID which caused the 
 *      fault.
 *
 *   @b Arguments
 *	 @verbatim
          faultPrivID      Fault Priv ID populated by this API.
          faultMstID       Fault Master ID populated by this API.
     @endverbatim
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
 *   @b Reads
 *   @n MSMC_SMPFR_FPID, MSMC_SMPFR_FMSTID
 *
 *   @b Example
 *   @verbatim
        Uint8   faultPrivID;
        Uint8   faultMstID;

        // Get the fault information.
        CSL_MSMC_getFaultInfo &faultPrivID, &faultMstID);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getFaultInfo (Uint8* faultPrivID, Uint8* faultMstID)
{
    Uint32 value = hMsmc->SMPFR;

	*faultPrivID = CSL_FEXT (value, MSMC_SMPFR_FPID);
	*faultMstID  = CSL_FEXT (value, MSMC_SMPFR_FMSTID);
}

/** ============================================================================
 *   @n@b CSL_MSMC_clearFaultPrivID
 *
 *   @b Description
 *   @n This function clears the faulted PrivID bit set in SMPFV register.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	Fault ID is cleared.
 *
 *   @b Writes
 *   @n MSMC_SMPFCR_CLR=1
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_clearFaultPrivID ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_clearFaultPrivID (void)
{
	CSL_FINS (hMsmc->SMPFCR, MSMC_SMPFCR_CLR, 1);
	return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSecurityThreshold
 *
 *   @b Description
 *   @n This function gets the Security Threshold Address register. All addresses
 *      above (and including) the address contained in the SMSTA register are 
 *      deemed as being secure and can only be accessed by secure mode code
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMSTA_SECURITY_THRESHOLD_SEGMENT_ADDRESS
 *
 *   @b Example
 *   @verbatim
        Uint32  secThreshold;

        // Get the security Threshold. 
        secThreshold = CSL_MSMC_getSecurityThreshold ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getSecurityThreshold (void)
{
	return CSL_FEXT (hMsmc->SMSTA, MSMC_SMSTA_SECURITY_THRESHOLD_SEGMENT_ADDRESS);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setSecurityThreshold
 *
 *   @b Description
 *   @n This function sets the Security Threshold Address register. All addresses
 *      above (and including) the address contained in the SMSTA register are 
 *      deemed as being secure and can only be accessed by secure mode code
 *
 *   @b Arguments
 *	 @verbatim
          securityThreshold      Security Threshold to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMSTA_SECURITY_THRESHOLD_SEGMENT_ADDRESS
 *
 *   @b Example
 *   @verbatim

        // Set the security Threshold to 0x0; implies that all MSMC SRAM is secure
        CSL_MSMC_setSecurityThreshold (0x0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setSecurityThreshold (Uint32 securityThreshold)
{
	CSL_FINS (hMsmc->SMSTA, MSMC_SMSTA_SECURITY_THRESHOLD_SEGMENT_ADDRESS, securityThreshold);
}

/** ============================================================================
 *   @n@b CSL_MSMC_IsNonSecure
 *
 *   @b Description
 *   @n This function checks if the MSMC SRAM is secure or not?
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE   - MSMC SRAM is non-secure
 *	 @n	 FALSE  - MSMC SRAM is secure
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMSTA_NS
 *
 *   @b Example
 *   @verbatim
        
        // Check if the MSMC SRAM is secure or not?
        if (CSL_MSMC_IsNonSecure () == TRUE)
        {
            // MSMC SRAM is Non-Secure
            ...
        }
        else
        {
            // MSMC SRAM is Secure
            ...
        }        

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_MSMC_IsNonSecure (void)
{
	if(CSL_FEXT (hMsmc->SMSTA, MSMC_SMSTA_NS) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_MSMC_SecureMSMC
 *
 *   @b Description
 *   @n This function secures the MSMC SRAM. This can only be called by a secure
 *      supervisor.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMSTA_NS=0
 *
 *   @b Example
 *   @verbatim

        // Secure the MSMC SRAM access
        CSL_MSMC_SecureMSMC ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_SecureMSMC (void)
{
	CSL_FINS (hMsmc->SMSTA, MSMC_SMSTA_NS, 0);
}

/** ============================================================================
 *   @n@b CSL_MSMC_NonSecureMSMC
 *
 *   @b Description
 *   @n This function non-secures the MSMC SRAM. This can only be called by a secure
 *      supervisor.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMSTA_NS=1
 *
 *   @b Example
 *   @verbatim

        // Ensure that access to all MSMC SRAM is non secure.
        CSL_MSMC_NonSecureMSMC ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_NonSecureMSMC (void)
{
	CSL_FINS (hMsmc->SMSTA, MSMC_SMSTA_NS, (Uint32)1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setStarvationBoundCGEM
 *
 *   @b Description
 *   @n This function sets the starvation bound for CCGEM slaves.
 *
 *   @b Arguments
     @verbatim
          index	            CGEM slave to configure starvation bound for
          strvEmifArbiter   Starvation bound to be configured for the EMIF Arbiter
          strvRAMArbiter    Starvation bound to be configured for the RAM Bank Arbiter
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SBNDC0_SCNTCE, MSMC_SBNDC0_SCNTCM
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_setStarvationBoundCGEM (1,1,1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setStarvationBoundCGEM 
(
	Uint32  index,
	Uint8   strvEmifArbiter,
	Uint8   strvRAMArbiter
)
{
    hMsmc->SBND[index] = CSL_FMK (MSMC_SBNDC0_SCNTCE, strvEmifArbiter) |
                         CSL_FMK (MSMC_SBNDC0_SCNTCM, strvRAMArbiter);
	return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getStarvationBoundCGEM
 *
 *   @b Description
 *   @n This function gets the starvation bound for CCGEM slaves.
 *
 *   @b Arguments
     @verbatim
          index	            CGEM slave index
          strvEmifArbiter   Starvation bound for the EMIF Arbiter
          strvRAMArbiter    Starvation bound for the RAM Bank Arbiter          
	 @endverbatim
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
 *   @b Reads
 *   @n MSMC_SBNDC0_SCNTCE, MSMC_SBNDC0_SCNTCM
 *
 *   @b Example
 *   @verbatim
        Uint8 strvEMIF;
        Uint8 strvRAM;

        CSL_MSMC_getStarvationBoundCGEM (1, &strvEMIF, &strvRAM);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getStarvationBoundCGEM 
(
    Uint32  index, 
	Uint8*  strvEmifArbiter,
	Uint8*  strvRAMArbiter
)
{
    Uint32 value = hMsmc->SBND[index];
    *strvEmifArbiter = CSL_FEXT (value, MSMC_SBNDC0_SCNTCE);
    *strvRAMArbiter  = CSL_FEXT (value, MSMC_SBNDC0_SCNTCM);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setStarvationBoundSMS
 *
 *   @b Description
 *   @n This function sets the starvation bound for SMS port.
 *
 *   @b Arguments
     @verbatim
          sbnd      starvation bound to be configured
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	Starvation Count for the SMS Port is configured.
 *
 *   @b Writes
 *   @n MSMC_SBNDM_SCNTMM
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_setStarvationBoundSMS (1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setStarvationBoundSMS (Uint32 sbnd)
{
	CSL_FINS (hMsmc->SBNDM, MSMC_SBNDM_SCNTMM, sbnd);
	return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getStarvationBoundSMS
 *
 *   @b Description
 *   @n This function gets the starvation bound for SMS port.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b> 
 *      Starvation Bound for the SMS Port.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SBNDM_SCNTMM
 *
 *   @b Example
 *   @verbatim
        Uint32 sbndm;

        sbndm = CSL_MSMC_getStarvationBoundSMS ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getStarvationBoundSMS (void)
{
	return CSL_FEXT (hMsmc->SBNDM, MSMC_SBNDM_SCNTMM);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setStarvationBoundSES
 *
 *   @b Description
 *   @n This function sets the starvation bound for SES port.
 *
 *   @b Arguments
     @verbatim
          sbnd      starvation bound to be configured
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	Starvation count for the SES port is configured
 *
 *   @b Writes
 *   @n MSMC_SBNDE_SCNTEM
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_setStarvationBoundSES (1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setStarvationBoundSES (Uint32 sbnd)
{
	CSL_FINS (hMsmc->SBNDE, MSMC_SBNDE_SCNTEM, sbnd);
	return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getStarvationBoundSES
 *
 *   @b Description
 *   @n This function gets the starvation bound for SES port.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b> 
 *      Starvation Bound for the SES Port.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SBNDE_SCNTEM
 *
 *   @b Example
 *   @verbatim
        Uint32 sbnde;

        sbnde = CSL_MSMC_getStarvationBoundSES ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getStarvationBoundSES (void)
{
	return CSL_FEXT (hMsmc->SBNDE, MSMC_SBNDE_SCNTEM);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setSMSMPAXH
 *
 *   @b Description
 *   @n This function sets the contents of SMS_MPAXH register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxh       CSL_MSMC_SMSMPAXH structure that needs to be set into the
                      register
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 SMS MPAXH register for the PrivID configured with the value passed.
 *
 *   @b Writes
 *   @n MSMC_SMS_MPAXH_0_SEGSZ..MSMC_SMS_MPAXH_7_SEGSZ,
 *      MSMC_SMS_MPAXH_0_BADDR..MSMC_SMS_MPAXH_7_BADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SMSMPAXH mpaxh;

        mpaxh.segSz = 4;
        mpaxh.baddr = 0x10;

        CSL_MSMC_setSMSMPAXH (privid, index, &mpaxh);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setSMSMPAXH 
(
	Uint32             privid,
	Uint32             index,
	CSL_MSMC_SMSMPAXH* mpaxh
)
{
    hMsmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH =
            CSL_FMK (MSMC_SMS_MPAXH_0_SEGSZ, mpaxh->segSz) |
            CSL_FMK (MSMC_SMS_MPAXH_0_BADDR, mpaxh->baddr);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSMSMPAXH
 *
 *   @b Description
 *   @n This function gets the contents of SMS_MPAXH register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxh       CSL_MSMC_SMSMPAXH structure that needs to be set into the
                      register
     @endverbatim
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
 *   @b Reads
 *   @n MSMC_SMS_MPAXH_0_SEGSZ..MSMC_SMS_MPAXH_7_SEGSZ,
 *      MSMC_SMS_MPAXH_0_BADDR..MSMC_SMS_MPAXH_7_BADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SMSMPAXH mpaxh;

        CSL_MSMC_getSMSMPAXH (privid, index, &mpaxh);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getSMSMPAXH (
    Uint32              privid,
    Uint32              index,
    CSL_MSMC_SMSMPAXH*  mpaxh
)
{
    Uint32 value = hMsmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH;
    mpaxh->segSz = CSL_FEXT (value, MSMC_SMS_MPAXH_0_SEGSZ);
    mpaxh->baddr = CSL_FEXT (value, MSMC_SMS_MPAXH_0_BADDR);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setSMSMPAXL
 *
 *   @b Description
 *   @n This function sets the contents of SMS_MPAXL register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxl       CSL_MSMC_SMSMPAXL structure that needs to be set into the
                      register
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	SMS MPAXL register for the PrivID configured with the value passed.
 *
 *   @b Writes
 *   @n MSMC_SMS_MPAXL_0_UX..MSMC_SMS_MPAXL_7_UX,
 *      MSMC_SMS_MPAXL_0_UW..MSMC_SMS_MPAXL_7_UW,
 *      MSMC_SMS_MPAXL_0_UR..MSMC_SMS_MPAXL_7_UR,
 *      MSMC_SMS_MPAXL_0_SX..MSMC_SMS_MPAXL_7_SX,
 *      MSMC_SMS_MPAXL_0_SW..MSMC_SMS_MPAXL_7_SW,
 *      MSMC_SMS_MPAXL_0_SR..MSMC_SMS_MPAXL_7_SR,
 *      MSMC_SMS_MPAXL_0_RADDR..MSMC_SMS_MPAXL_7_RADDR,
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SMSMPAXL mpaxl;

        mpaxl.ux    = 1;
        mpaxl.uw    = 1;
        mpaxl.ur    = 1;
        mpaxl.sx    = 1;
        mpaxl.sw    = 1;
        mpaxl.sr    = 1;
        mpaxl.emu   = 0;
        mpaxl.ns    = 1;
        mpaxl.raddr = 0x100;

        CSL_MSMC_setSMSMPAXL (privid, index, &mpaxl);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setSMSMPAXL 
(
	Uint32              privid,
	Uint32              index,
	CSL_MSMC_SMSMPAXL*  mpaxl
)
{
    Uint32 value = 0;

	CSL_FINS (value, MSMC_SMS_MPAXL_0_UX, mpaxl->ux);
	CSL_FINS (value, MSMC_SMS_MPAXL_0_UW, mpaxl->uw);
	CSL_FINS (value, MSMC_SMS_MPAXL_0_UR, mpaxl->ur);
	CSL_FINS (value, MSMC_SMS_MPAXL_0_SX, mpaxl->sx);
	CSL_FINS (value, MSMC_SMS_MPAXL_0_SW, mpaxl->sw);
	CSL_FINS (value, MSMC_SMS_MPAXL_0_SR, mpaxl->sr);
	CSL_FINS (value, MSMC_SMS_MPAXL_0_RADDR, mpaxl->raddr);

    hMsmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXL = value;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSMSMPAXL
 *
 *   @b Description
 *   @n This function gets the contents of SMS_MPAXL register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxl       CSL_MSMC_SMSMPAXL structure that needs to be set into the 
                      register
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMS_MPAXL_0_UX..MSMC_SMS_MPAXL_7_UX,
 *      MSMC_SMS_MPAXL_0_UW..MSMC_SMS_MPAXL_7_UW,
 *      MSMC_SMS_MPAXL_0_UR..MSMC_SMS_MPAXL_7_UR,
 *      MSMC_SMS_MPAXL_0_SX..MSMC_SMS_MPAXL_7_SX,
 *      MSMC_SMS_MPAXL_0_SW..MSMC_SMS_MPAXL_7_SW,
 *      MSMC_SMS_MPAXL_0_SR..MSMC_SMS_MPAXL_7_SR,
 *      MSMC_SMS_MPAXL_0_RADDR..MSMC_SMS_MPAXL_7_RADDR,
 *
 *   @b Example
 *   @verbatim
		Uint32 privid = 1;
		Uint32 index = 0;
		CSL_MSMC_SMSMPAXL mpaxl;

		CSL_MSMC_getSMSMPAXL (privid, index, &mpaxl);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getSMSMPAXL 
(
	Uint32              privid,
	Uint32              index,
	CSL_MSMC_SMSMPAXL*  mpaxl
)
{
    Uint32 value = hMsmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXL;

	mpaxl->raddr = CSL_FEXT (value, MSMC_SMS_MPAXL_0_RADDR);
	mpaxl->sr    = CSL_FEXT (value, MSMC_SMS_MPAXL_0_SR);
	mpaxl->sw    = CSL_FEXT (value, MSMC_SMS_MPAXL_0_SW);
	mpaxl->sx    = CSL_FEXT (value, MSMC_SMS_MPAXL_0_SX);
	mpaxl->ur    = CSL_FEXT (value, MSMC_SMS_MPAXL_0_UR);
	mpaxl->uw    = CSL_FEXT (value, MSMC_SMS_MPAXL_0_UW);
	mpaxl->ux    = CSL_FEXT (value, MSMC_SMS_MPAXL_0_UX);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setSESMPAXH
 *
 *   @b Description
 *   @n This function sets the contents of SES_MPAXH register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxh       CSL_MSMC_SESMPAXH structure that needs to be set into the
                      register
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	SES MPAXH register for the PrivID configured with the value passed.
 *
 *   @b Writes
 *   @n MSMC_SES_MPAXH_0_BE..MSMC_SES_MPAXH_7_BE,
 *      MSMC_SES_MPAXH_0_AI..MSMC_SES_MPAXH_7_AI,
 *      MSMC_SES_MPAXH_0_SEGSZ..MSMC_SES_MPAXH_7_SEGSZ,
 *      MSMC_SES_MPAXH_0_BADDR..MSMC_SES_MPAXH_7_BADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SESMPAXH mpaxh;

        mpaxh.segSize = 4;
        mpaxh.baseAddress = 0x100;
        mpaxh.be = 1;
        mpaxh.ai = 1;

        CSL_MSMC_setSESMPAXH (privid, index, &mpaxh);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setSESMPAXH 
(
	Uint32             privid,
	Uint32             index,
	CSL_MSMC_SESMPAXH* mpaxh
)
{
    Uint32 value = 0;

	CSL_FINS (value, MSMC_SES_MPAXH_0_BE, mpaxh->be);
    CSL_FINS (value, MSMC_SES_MPAXH_0_AI, mpaxh->ai);
    CSL_FINS (value, MSMC_SES_MPAXH_0_SEGSZ, mpaxh->segSz);
    CSL_FINS (value, MSMC_SES_MPAXH_0_BADDR, mpaxh->baddr);

 	hMsmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH = value;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSESMPAXH
 *
 *   @b Description
 *   @n This function gets the contents of SES_MPAXH register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxh       CSL_MSMC_SESMPAXH structure that needs to be set into the
                      register
     @endverbatim
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
 *   @b Reads
 *   @n MSMC_SES_MPAXH_0_BE..MSMC_SES_MPAXH_7_BE,
 *      MSMC_SES_MPAXH_0_AI..MSMC_SES_MPAXH_7_AI,
 *      MSMC_SES_MPAXH_0_SEGSZ..MSMC_SES_MPAXH_7_SEGSZ,
 *      MSMC_SES_MPAXH_0_BADDR..MSMC_SES_MPAXH_7_BADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SESMPAXH mpaxh;

        CSL_MSMC_getSESMPAXH (privid, index, &mpaxh);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getSESMPAXH 
(
    Uint32              privid,
    Uint32              index,
    CSL_MSMC_SESMPAXH*  mpaxh
)
{
    Uint32 value = hMsmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH;
    mpaxh->be    = CSL_FEXT (value, MSMC_SES_MPAXH_0_BE);
    mpaxh->ai    = CSL_FEXT (value, MSMC_SES_MPAXH_0_AI);
    mpaxh->segSz = CSL_FEXT (value, MSMC_SES_MPAXH_0_SEGSZ);
    mpaxh->baddr = CSL_FEXT (value, MSMC_SES_MPAXH_0_BADDR);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setSESMPAXL
 *
 *   @b Description
 *   @n This function sets the contents of SES_MPAXL register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxl       CSL_MSMC_SESMPAXL structure that needs to be set into the
                      register
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	SES MPAXL register for the PrivID configured with the value passed.
 *
 *   @b Writes
 *   @n MSMC_SES_MPAXL_0_UX..MSMC_SES_MPAXL_7_UX,
 *      MSMC_SES_MPAXL_0_UW..MSMC_SES_MPAXL_7_UW,
 *      MSMC_SES_MPAXL_0_UR..MSMC_SES_MPAXL_7_UR,
 *      MSMC_SES_MPAXL_0_SX..MSMC_SES_MPAXL_7_SX,
 *      MSMC_SES_MPAXL_0_SW..MSMC_SES_MPAXL_7_SW,
 *      MSMC_SES_MPAXL_0_SR..MSMC_SES_MPAXL_7_SR,
 *      MSMC_SES_MPAXL_0_RADDR..MSMC_SES_MPAXL_7_RADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SESMPAXL mpaxl;

        mpaxl.ux    = 1;
        mpaxl.uw    = 1;
        mpaxl.ur    = 1;
        mpaxl.sx    = 1;
        mpaxl.sw    = 1;
        mpaxl.sr    = 1;
        mpaxl.raddr = 0x4000;

        CSL_MSMC_setSESMPAXL (privid, index, &mpaxl);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setSESMPAXL 
(
	Uint32              privid,
	Uint32              index,
	CSL_MSMC_SESMPAXL*  mpaxl
)
{
    Uint32 value = 0;

	CSL_FINS (value, MSMC_SES_MPAXL_0_UX, mpaxl->ux);
	CSL_FINS (value, MSMC_SES_MPAXL_0_UW, mpaxl->uw);
	CSL_FINS (value, MSMC_SES_MPAXL_0_UR, mpaxl->ur);
	CSL_FINS (value, MSMC_SES_MPAXL_0_SX, mpaxl->sx);
	CSL_FINS (value, MSMC_SES_MPAXL_0_SW, mpaxl->sw);
	CSL_FINS (value, MSMC_SES_MPAXL_0_SR, mpaxl->sr);
	CSL_FINS (value, MSMC_SES_MPAXL_0_RADDR, mpaxl->raddr);

	hMsmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXL = value;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSESMPAXL
 *
 *   @b Description
 *   @n This function gets the contents of SES_MPAXL register.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID
          index       Index into the set of 8 registers for this particular
                      PrivID
          mpaxl       CSL_MSMC_SESMPAXL structure that needs to be set into the
                      register
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SES_MPAXL_0_UX..MSMC_SES_MPAXL_7_UX,
 *      MSMC_SES_MPAXL_0_UW..MSMC_SES_MPAXL_7_UW,
 *      MSMC_SES_MPAXL_0_UR..MSMC_SES_MPAXL_7_UR,
 *      MSMC_SES_MPAXL_0_SX..MSMC_SES_MPAXL_7_SX,
 *      MSMC_SES_MPAXL_0_SW..MSMC_SES_MPAXL_7_SW,
 *      MSMC_SES_MPAXL_0_SR..MSMC_SES_MPAXL_7_SR,
 *      MSMC_SES_MPAXL_0_RADDR..MSMC_SES_MPAXL_7_RADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 privid = 1;
        Uint32 index = 0;
        CSL_MSMC_SESMPAXL mpaxl;

        CSL_MSMC_getSESMPAXL (privid, index, &mpaxl);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getSESMPAXL 
(
	Uint32              privid,
	Uint32              index,
	CSL_MSMC_SESMPAXL*  mpaxl
)
{
    Uint32 value = hMsmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXL;
	mpaxl->raddr = CSL_FEXT (value, MSMC_SES_MPAXL_0_RADDR);
	mpaxl->sr    = CSL_FEXT (value, MSMC_SES_MPAXL_0_SR);
	mpaxl->sw    = CSL_FEXT (value, MSMC_SES_MPAXL_0_SW);
	mpaxl->sx    = CSL_FEXT (value, MSMC_SES_MPAXL_0_SX);
	mpaxl->ur    = CSL_FEXT (value, MSMC_SES_MPAXL_0_UR);
	mpaxl->uw    = CSL_FEXT (value, MSMC_SES_MPAXL_0_UW);
	mpaxl->ux    = CSL_FEXT (value, MSMC_SES_MPAXL_0_UX);
	return;
}

/** ============================================================================
 *   @n@b CSL_MSMC_lockNonMPAX
 *
 *   @b Description
 *   @n This function locks the write access for all non MPAX registers.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  All the non MPAX registers are locked for write-access.
 *
 *   @b Writes
 *   @n MSMC_CFGLCK_MGCID=0x2CD00000,MSMC_CFGLCK_WLCK=1
 *
 *   @b Example
 *   @verbatim

        CSL_MSMC_lockNonMPAX ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_lockNonMPAX (void)
{
    hMsmc->CFGLCK = CSL_FMK(MSMC_CFGLCK_MGCID, 0x2CD0) | CSL_FMK(MSMC_CFGLCK_WLCK, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_isNonMPAXLocked
 *
 *   @b Description
 *   @n This function gets the status of the lock for non MPAX registers.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE  - Non MPAX Register Write Access is locked
 *   @n	 FALSE - Non MPAX Register Write Access is unlocked
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_CFGLCKSTAT_WSTAT 
 *
 *   @b Example
 *   @verbatim

        if (CSL_MSMC_isNonMPAXLocked () == TRUE)
        {
            // Non MPAX Registers are locked.
        }
        else
        {
           // MPAX Registers are not locked.
        }

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_MSMC_isNonMPAXLocked (void)
{
    return (Bool)CSL_FEXT (hMsmc->CFGLCKSTAT, MSMC_CFGLCKSTAT_WSTAT);
}

/** ============================================================================
 *   @n@b CSL_MSMC_unlockNonMPAX
 *
 *   @b Description
 *   @n This function unlocks the write access for all non MPAX registers.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_MSMC_lockNonMPAX() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  All the non MPAX registers are available for write-access.
 *
 *   @b Writes
 *   @n MSMC_CFGULCK_MGCID=0x2CD00000,MSMC_CFGULCK_WEN=1
 *
 *   @b Example
 *   @verbatim

        if (CSL_MSMC_isNonMPAXLocked () == TRUE)
        {
            // Non MPAX Registers are locked
            ...
            // Unlock the Non-MPAX registers.
            CSL_MSMC_unlockNonMPAX ();
        }

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_unlockNonMPAX (void)
{
    hMsmc->CFGULCK = CSL_FMK(MSMC_CFGULCK_MGCID, 0x2CD0) | CSL_FMK(MSMC_CFGULCK_WEN, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_lockSMS
 *
 *   @b Description
 *   @n This function locks the write access for all SMS MPAX registers.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID for which the SMS MPAX registers will be locked.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  All the SMS MPAX registers for the specific PrivID are locked for 
 *       write-access.
 *
 *   @b Writes
 *   @n MSMC_SMS_MPAX_LCK_MGCID=0x2CD10000, MSMC_SMS_MPAX_LCK_WLCK
 *
 *   @b Example
 *   @verbatim

        // Lock SMS MPAX registers for priv-id 1.
        CSL_MSMC_lockSMS (1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_lockSMS (Uint32 privid)
{
    hMsmc->SMS_MPAX_LCK = CSL_FMK(MSMC_SMS_MPAX_LCK_MGCID, 0x2CD1) | CSL_FMKR (privid, privid, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_isSMSLocked
 *
 *   @b Description
 *   @n This function gets the status of the lock for SMS MPAX registers for the 
 *      specific priv-id
 *
 *   @b Arguments
     @verbatim
          privid      PrivID for which the status is required.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE  - SMS MPAX Register Write Access is locked for the priv-id
 *   @n	 FALSE - SMS MPAX Register Write Access is unlocked for the priv-id
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMS_MPAX_LCKSTAT_WSTAT 
 *
 *   @b Example
 *   @verbatim

        if (CSL_MSMC_isSMSLocked (1) == TRUE)
        {
            // SMS MPAX Registers for Priv-ID 1 are locked.
        }
        else
        {
           // SMS MPAX Registers for Priv-ID 1 are not locked.
        }

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_MSMC_isSMSLocked (Uint32 privId)
{
    return (Bool)CSL_FEXTR (hMsmc->SMS_MPAX_LCKSTAT, privId, privId);
}

/** ============================================================================
 *   @n@b CSL_MSMC_unlockSMS
 *
 *   @b Description
 *   @n This function unlocks the write access for all SMS MPAX registers.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID for which the SMS MPAX registers will be unlocked.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_MSMC_lockSMS() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  All the SMS MPAX registers for the specific PrivID are unlocked for 
 *       write-access.
 *
 *   @b Writes
 *   @n MSMC_SMS_MPAX_ULCK_MGCID=0x2CD10000,MSMC_SMS_MPAX_ULCK_WEN
 *
 *   @b Example
 *   @verbatim
        ...
        if (CSL_MSMC_isSMSLocked (1) == TRUE)
        {
            // SMS MPAX Registers for Priv-ID 1 are locked.
            ...
            // Unlock SMS MPAX registers for priv-id 1.
            CSL_MSMC_unlockSMS (1);
        }
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_unlockSMS (Uint32 privid)
{
    hMsmc->SMS_MPAX_ULCK = CSL_FMK(MSMC_SMS_MPAX_ULCK_MGCID, 0x2CD1) | CSL_FMKR (privid, privid, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_lockSES
 *
 *   @b Description
 *   @n This function locks the write access for all SES MPAX registers.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID for which the SES MPAX registers will be locked.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  All the SES MPAX registers for the specific PrivID are locked for 
 *       write-access.
 *
 *   @b Writes
 *   @n MSMC_SES_MPAX_LCK_MGCID=0x2CD20000,MSMC_SES_MPAX_LCK_WLCK
 *
 *   @b Example
 *   @verbatim

        // Lock SES MPAX registers for priv-id 1.
        CSL_MSMC_lockSES (1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_lockSES (Uint32 privid)
{
    hMsmc->SES_MPAX_LCK = CSL_FMK(MSMC_SES_MPAX_LCK_MGCID, 0x2CD2) | CSL_FMKR (privid, privid, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_isSESLocked
 *
 *   @b Description
 *   @n This function gets the status of the lock for SES MPAX registers for the 
 *      specific priv-id
 *
 *   @b Arguments
     @verbatim
          privid      PrivID for which the status is required.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE  - SES MPAX Register Write Access is locked for the priv-id
 *   @n	 FALSE - SES MPAX Register Write Access is unlocked for the priv-id
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SES_MPAX_LCKSTAT_WSTAT 
 *
 *   @b Example
 *   @verbatim

        if (CSL_MSMC_isSESLocked (1) == TRUE)
        {
            // SES MPAX Registers for Priv-ID 1 are locked.
        }
        else
        {
           // SES MPAX Registers for Priv-ID 1 are not locked.
        }

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_MSMC_isSESLocked (Uint32 privId)
{
    return (Bool)CSL_FEXTR (hMsmc->SES_MPAX_LCKSTAT, privId, privId);
}

/** ============================================================================
 *   @n@b CSL_MSMC_unlockSES
 *
 *   @b Description
 *   @n This function unlocks the write access for all SES MPAX registers.
 *
 *   @b Arguments
     @verbatim
          privid      PrivID for which the SES MPAX registers will be unlocked.
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_MSMC_lockSES() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  All the SES MPAX registers for the specific PrivID are unlocked for 
 *       write-access.
 *
 *   @b Writes
 *   @n MSMC_SES_MPAX_ULCK_MGCID=0x2CD20000,MSMC_SES_MPAX_ULCK_WEN
 *
 *   @b Example
 *   @verbatim
        ...
        if (CSL_MSMC_isSESLocked (1) == TRUE)
        {
            // SES MPAX Registers for Priv-ID 1 are locked.
            ...
            // Unlock SES MPAX registers for priv-id 1.
            CSL_MSMC_unlockSES (1);
        }
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_unlockSES (Uint32 privid)
{
    hMsmc->SES_MPAX_ULCK = CSL_FMK(MSMC_SES_MPAX_ULCK_MGCID, 0x2CD2) | CSL_FMKR (privid, privid, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getPendingInterrupts
 *
 *   @b Description
 *   @n This function gets the status of all pending interrupts i.e. which have 
 *   been enabled and an interrupt has occurred.
 *
 *   @b Arguments
     @verbatim
          pfeStat      Protection Fault Status
          cees         Correctable EDC Error
          ncees        Non-Correctable EDC Error
          cses         Correctable Scrubbing Error
          nces         Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMESTAT_PFESTAT, MSMC_SMESTAT_CEES, MSMC_SMESTAT_NCEES, 
 *   @n MSMC_SMESTAT_CSES, MSMC_SMESTAT_NCSES
 *
 *   @b Example
 *   @verbatim
        Uint16  pfeStat;
        Uint8   cees;
        Uint8   ncees;
        Uint8   cses;
        Uint8   ncses;

        // Get the MSMC Pending Interrupt Status
        CSL_MSMC_getPendingInterrupts (&pfeStat,&cees, &ncees, &cses, &ncses);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getPendingInterrupts
(
    Uint16*  pfeStat,
    Uint8*   cees,
    Uint8*   ncees,
    Uint8*   cses,
    Uint8*   ncses
)
{
    Uint32 value = hMsmc->SMESTAT;

    *pfeStat = CSL_FEXT (value, MSMC_SMESTAT_PFESTAT);    
    *cees    = CSL_FEXT (value, MSMC_SMESTAT_CEES);
    *ncees   = CSL_FEXT (value, MSMC_SMESTAT_NCEES);
    *cses    = CSL_FEXT (value, MSMC_SMESTAT_CSES);
    *ncses   = CSL_FEXT (value, MSMC_SMESTAT_NCSES);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getRawInterruptStatus
 *
 *   @b Description
 *   @n This function gets the RAW Interrupt Status. 
 *
 *   @b Arguments
     @verbatim
          pfeStat      Protection Fault Status
          cees         Correctable EDC Error
          ncees        Non-Correctable EDC Error
          cses         Correctable Scrubbing Error
          nces         Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMIRSTAT_PFESTAT, MSMC_SMIRSTAT_CEES, MSMC_SMIRSTAT_NCEES, 
 *   @n MSMC_SMIRSTAT_CSES, MSMC_SMIRSTAT_NCSES
 *
 *   @b Example
 *   @verbatim
        Uint16  pfeStat;
        Uint8   cees;
        Uint8   ncees;
        Uint8   cses;
        Uint8   ncses;

        // Get the MSMC Raw Interrupt Status
        CSL_MSMC_getRawInterruptStatus (&pfeStat, &cees, &ncees, &cses, &ncses);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getRawInterruptStatus
(
    Uint16*  pfeStat,
    Uint8*   cees,
    Uint8*   ncees,
    Uint8*   cses,
    Uint8*   ncses
)
{
    Uint32 value = hMsmc->SMIRSTAT;

    *pfeStat = CSL_FEXT (value, MSMC_SMIRSTAT_PFESTAT);
    *cees    = CSL_FEXT (value, MSMC_SMIRSTAT_CEES);
    *ncees   = CSL_FEXT (value, MSMC_SMIRSTAT_NCEES);
    *cses    = CSL_FEXT (value, MSMC_SMIRSTAT_CSES);
    *ncses   = CSL_FEXT (value, MSMC_SMIRSTAT_NCSES);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setRawInterruptStatus
 *
 *   @b Description
 *   @n This function sets the RAW Interrupt Status.
 *
 *   @b Arguments
     @verbatim
          pfiStat       Protection Fault Status
          cei           Correctable EDC Error
          ncei          Non-Correctable EDC Error
          csi           Correctable Scrubbing Error
          ncsi          Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n MSMC_SMIRSTAT_PFESTAT, MSMC_SMIRSTAT_CEES, MSMC_SMIRSTAT_NCEES, 
 *   @n MSMC_SMIRSTAT_CSES, MSMC_SMIRSTAT_NCSES
 *
 *   @b Example
 *   @verbatim

        // Ensure only the Correctable EDC Errors is set
        CSL_MSMC_setRawInterruptStatus (0x0, 0x1, 0x0, 0x0, 0x0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setRawInterruptStatus
(
    Uint16  pfiStat,
    Uint8   cei,
    Uint8   ncei,
    Uint8   csi,
    Uint8   ncsi 
)
{
    hMsmc->SMIRSTAT = CSL_FMK (MSMC_SMIRSTAT_PFESTAT, pfiStat) |
                      CSL_FMK (MSMC_SMIRSTAT_CEES,    cei)     |
                      CSL_FMK (MSMC_SMIRSTAT_NCEES,   ncei)    |
                      CSL_FMK (MSMC_SMIRSTAT_CSES,    csi)     |
                      CSL_FMK (MSMC_SMIRSTAT_NCSES,   ncsi);
}

/** ============================================================================
 *   @n@b CSL_MSMC_clearRawInterruptStatus
 *
 *   @b Description
 *   @n This function clears the RAW Interrupt Status.
 *
 *   @b Arguments
     @verbatim
          pfiStat       Protection Fault Status
          cei           Correctable EDC Error
          ncei          Non-Correctable EDC Error
          csi           Correctable Scrubbing Error
          ncsi          Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n MSMC_SMIRC_PFIC, MSMC_SMIRC_MPEC, MSMC_SMIRC_CEC,
 *   @n MSMC_SMIRC_NCEC, MSMC_SMIRC_CSC,  MSMC_SMIRC_NCSC
 *
 *   @b Example
 *   @verbatim
        Uint16  pfiStat;
        Uint8   cei;
        Uint8   ncei;
        Uint8   csi;
        Uint8   ncsi;

        // Get the MSMC Raw Interrupt Status
        CSL_MSMC_getRawInterruptStatus (&pfiStat, &cei, &ncei, &csi, &ncsi);
        ...
        // Clear the MSMC Interrupt Status.
        CSL_MSMC_clearRawInterruptStatus (pfiStat, cei, ncei, csi, ncsi);
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_clearRawInterruptStatus
(
    Uint16  pfiStat,
    Uint8   cei,
    Uint8   ncei,
    Uint8   csi,
    Uint8   ncsi 
)
{
    hMsmc->SMIRC = CSL_FMK (MSMC_SMIRC_PFIC, pfiStat) | 
                   CSL_FMK (MSMC_SMIRC_CEC,  cei)     |
                   CSL_FMK (MSMC_SMIRC_NCEC, ncei)    |
                   CSL_FMK (MSMC_SMIRC_CSC,  csi)     |
                   CSL_FMK (MSMC_SMIRC_NCSC, ncsi);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getInterruptEnableStatus
 *
 *   @b Description
 *   @n This function is used to get the interrupt enable/disable status
 *
 *   @b Arguments
     @verbatim
          pfiStat       Protection Fault Status
          cei           Correctable EDC Error
          ncei          Non-Correctable EDC Error
          csi           Correctable Scrubbing Error
          ncsi          Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMIESTAT_PFIESTAT, MSMC_SMIESTAT_CEIE, MSMC_SMIESTAT_NCEIE,
 *   @n MSMC_SMIESTAT_CSIE,MSMC_SMIESTAT_NCSIE 
 *
 *   @b Example
 *   @verbatim

        Uint16  pfiStat;
        Uint8   cei;
        Uint8   ncei;
        Uint8   csi;
        Uint8   ncsi;

        // Get the interrupt enable status.
        CSL_MSMC_getInterruptEnableStatus (&pfiStat, &cei, &ncei, &csi, &ncsi);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getInterruptEnableStatus
(
    Uint16* pfiStat,
    Uint8*  cei,
    Uint8*  ncei,
    Uint8*  csi,
    Uint8*  ncsi 
)
{
    Uint32  value = hMsmc->SMIESTAT;

    *pfiStat  = CSL_FEXT (value, MSMC_SMIESTAT_PFIESTAT);
    *cei      = CSL_FEXT (value, MSMC_SMIESTAT_CEIE);
    *ncei     = CSL_FEXT (value, MSMC_SMIESTAT_NCEIE);
    *csi      = CSL_FEXT (value, MSMC_SMIESTAT_CSIE);
    *ncsi     = CSL_FEXT (value, MSMC_SMIESTAT_NCSIE);
}

/** ============================================================================
 *   @n@b CSL_MSMC_enableInterrupts
 *
 *   @b Description
 *   @n This function enables the MSMC interrupts
 *
 *   @b Arguments
     @verbatim
          pfiStat       Protection Fault Status
          cei           Correctable EDC Error
          ncei          Non-Correctable EDC Error
          csi           Correctable Scrubbing Error
          ncsi          Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n MSMC_SMIESTAT_PFIESTAT, MSMC_SMIESTAT_CEIE, MSMC_SMIESTAT_NCEIE,
 *   @n MSMC_SMIESTAT_CSIE,MSMC_SMIESTAT_NCSIE 
 *
 *   @b Example
 *   @verbatim

        // Enable all interrupts except the scrubbing errors.
        CSL_MSMC_enableInterrupts (0xFFFF, 0x1, 0x1, 0x0, 0x0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_enableInterrupts
(
    Uint16  pfiStat,
    Uint8   cei,
    Uint8   ncei,
    Uint8   csi,
    Uint8   ncsi 
)
{
    hMsmc->SMIESTAT = CSL_FMK (MSMC_SMIESTAT_PFIESTAT, pfiStat) | 
                      CSL_FMK (MSMC_SMIESTAT_CEIE,     cei)     |
                      CSL_FMK (MSMC_SMIESTAT_NCEIE,    ncei)    |
                      CSL_FMK (MSMC_SMIESTAT_CSIE,     csi)     |
                      CSL_FMK (MSMC_SMIESTAT_NCSIE,    ncsi);
}

/** ============================================================================
 *   @n@b CSL_MSMC_disableInterrupts
 *
 *   @b Description
 *   @n This function disables the MSMC interrupts
 *
 *   @b Arguments
     @verbatim
          pfiStat       Protection Fault Status
          cei           Correctable EDC Error
          ncei          Non-Correctable EDC Error
          csi           Correctable Scrubbing Error
          ncsi          Non-Correctable Scrubbing Error
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n MSMC_SMIEC_PFIEC, MSMC_SMIEC_CEEC, MSMC_SMIEC_NCEEC, 
 *   @n MSMC_SMIEC_CSEC,  MSMC_SMIEC_NCSEC
 *
 *   @b Example
 *   @verbatim

        // Disable all interrupts except the scrubbing errors.
        CSL_MSMC_disableInterrupts (0xFFFF, 0x1, 0x1, 0x0, 0x0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_disableInterrupts
(
    Uint16  pfiStat,
    Uint8   cei,
    Uint8   ncei,
    Uint8   csi,
    Uint8   ncsi 
)
{
    hMsmc->SMIEC = CSL_FMK (MSMC_SMIEC_PFIEC, pfiStat) | 
                   CSL_FMK (MSMC_SMIEC_CEEC,  cei)     |
                   CSL_FMK (MSMC_SMIEC_NCEEC, ncei)    |
                   CSL_FMK (MSMC_SMIEC_CSEC,  csi)     |
                   CSL_FMK (MSMC_SMIEC_NCSEC, ncsi);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getTransmitHeader
 *
 *   @b Description
 *   @n This function gets the transmit header for analysis counter state 
 *      export through STM
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	Transmit Header
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMATH
 *
 *   @b Example
 *   @verbatim
        Uint32  transmitHeader;

        // Get the Transmit Header
        transmitHeader = CSL_MSMC_getTransmitHeader();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getTransmitHeader (void)
{
    return hMsmc->SMATH;
}

/** ============================================================================
 *   @n@b CSL_MSMC_setTransmitHeader
 *
 *   @b Description
 *   @n This function sets the transmit header for analysis counter state 
 *      export through STM
 *
 *   @b Arguments
     @verbatim
          transmitHeader      Transmit header to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMATH
 *
 *   @b Example
 *   @verbatim

        // Set the Transmit Header
        CSL_MSMC_getTransmitHeader(0x100);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setTransmitHeader (Uint32 transmitHeader)
{
    hMsmc->SMATH = transmitHeader;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getHitCounter
 *
 *   @b Description
 *   @n This function gets the hit counter for the cache. If the HM bit in SMAC 
 *      is set, it is a counter for Cache hits+SRAM accesses
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	Hit Counter
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMAH_HCNT
 *
 *   @b Example
 *   @verbatim
        Uint32  hitCounter;

        // Get the Hit Counter
        hitCounter = CSL_MSMC_getHitCounter();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getHitCounter (void)
{
    return CSL_FEXT (hMsmc->SMAH, MSMC_SMAH_HCNT);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setHitCounter
 *
 *   @b Description
 *   @n This function sets the hit counter for the cache. If the HM bit in SMAC 
 *      is set, it is a counter for Cache hits+SRAM accesses
 *
 *   @b Arguments
 *   @n hitCounter  -   Hit Counter to be configured.
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAH_HCNT
 *
 *   @b Example
 *   @verbatim

        // Reset the hit counter
        CSL_MSMC_setHitCounter(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setHitCounter (Uint32 hitCounter)
{
    CSL_FINS (hMsmc->SMAH, MSMC_SMAH_HCNT, hitCounter);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getMissCounter
 *
 *   @b Description
 *   @n This function gets the reads/write miss counters
 *
 *   @b Arguments
     @verbatim
          readMissCntr      Read Miss Counter
          writeMissCntr     Write Miss Counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMARM_RMCNT,MSMC_SMAWM_WMCNT
 *
 *   @b Example
 *   @verbatim
        Uint32  readMiss;
        Uint32  writeMiss;

        // Get the Read and Write Miss Counters
        CSL_MSMC_getMissCounter(&readMiss, &writeMiss);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getMissCounter (Uint32* readMiss, Uint32* writeMiss)
{
    *readMiss  = CSL_FEXT (hMsmc->SMARM, MSMC_SMARM_RMCNT);
    *writeMiss = CSL_FEXT (hMsmc->SMAWM, MSMC_SMAWM_WMCNT);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setMissCounter
 *
 *   @b Description
 *   @n This function sets the reads/write miss counters
 *
 *   @b Arguments
     @verbatim
          readMissCntr      Read Miss Counter
          writeMissCntr     Write Miss Counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMARM_RMCNT, MSMC_SMAWM_WMCNT
 *
 *   @b Example
 *   @verbatim

        // Clear the Read & Write Miss Counters
        CSL_MSMC_getMissCounter(&readMiss, &writeMiss);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setMissCounter (Uint32 readMiss, Uint32 writeMiss)
{
    CSL_FINS (hMsmc->SMARM, MSMC_SMARM_RMCNT, readMiss);
    CSL_FINS (hMsmc->SMAWM, MSMC_SMAWM_WMCNT, writeMiss);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getVictimCounter
 *
 *   @b Description
 *   @n This function gets the victim counter
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	Victim Counter
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMAV_VCNT
 *
 *   @b Example
 *   @verbatim
        Uint32  victimCounter;

        // Get the Victim Counter
        victimCounter = CSL_MSMC_getVictimCounter();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getVictimCounter (void)
{
    return CSL_FEXT (hMsmc->SMAV, MSMC_SMAV_VCNT);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setVictimCounter
 *
 *   @b Description
 *   @n This function sets the victim counter
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	Victim Counter
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAV_VCNT
 *
 *   @b Example
 *   @verbatim

        // Reset the Victim Counter
        CSL_MSMC_setVictimCounter(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setVictimCounter (Uint32 victimCounter)
{
    CSL_FINS (hMsmc->SMAV, MSMC_SMAV_VCNT, victimCounter);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getMemoryArbiterCounters
 *
 *   @b Description
 *   @n This function gets the performance counters for the memory arbiters 
 *      (all banks).
 *
 *   @b Arguments
     @verbatim
          priorityElevationCounter  Priority Elevation Counter
          accCyclesCounter          Accumulated Cycles counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMAMP_MPECNT, MSMC_SMAMP_ACWM
 *
 *   @b Example
 *   @verbatim
        Uint8   priorityElevationCounter;
        Uint32  accCyclesCounter;

        // Get the Memory Arbiter Counters
        CSL_MSMC_getMemoryArbiterCounters(&priorityElevationCounter, &accCyclesCounter);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getMemoryArbiterCounters 
(
    Uint8*  priorityElevationCounter, 
    Uint32* accCyclesCounter
)
{
    *priorityElevationCounter   = CSL_FEXT (hMsmc->SMAMP, MSMC_SMAMP_MPECNT);
    *accCyclesCounter           = CSL_FEXT (hMsmc->SMAMP, MSMC_SMAMP_ACWM);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setMemoryArbiterCounters
 *
 *   @b Description
 *   @n This function sets the performance counters for the memory arbiters 
 *      (all banks).
 *
 *   @b Arguments
     @verbatim
          accCyclesCounter          Accumulated Cycles counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAMP_ACWM
 *
 *   @b Example
 *   @verbatim

        // Clear the Memory Arbiter Counters
        CSL_MSMC_setMemoryArbiterCounters(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setMemoryArbiterCounters(Uint32 accCyclesCounter)
{
    CSL_FINS (hMsmc->SMAMP, MSMC_SMAMP_ACWM, accCyclesCounter);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getPortArbiterCounters
 *
 *   @b Description
 *   @n This function gets the performance counters for the port arbiters 
 *
 *   @b Arguments
     @verbatim
          priorityElevationCounter  Priority Elevation Counter
          accCyclesCounter          Accumulated Cycles counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMAEP_EPECNT, MSMC_SMAEP_ACWE
 *
 *   @b Example
 *   @verbatim
        Uint8   priorityElevationCounter;
        Uint32  accCyclesCounter;

        // Get the Port Arbiter Counters
        CSL_MSMC_getPortArbiterCounters(&priorityElevationCounter, &accCyclesCounter);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getPortArbiterCounters 
(
    Uint8*  priorityElevationCounter, 
    Uint32* accCyclesCounter
)
{
    *priorityElevationCounter   = CSL_FEXT (hMsmc->SMAEP, MSMC_SMAEP_EPECNT);
    *accCyclesCounter           = CSL_FEXT (hMsmc->SMAEP, MSMC_SMAEP_ACWE);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setPortArbiterCounters
 *
 *   @b Description
 *   @n This function sets the performance counters for the port arbiters 
 *
 *   @b Arguments
     @verbatim
          accCyclesCounter          Accumulated Cycles counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAEP_ACWE
 *
 *   @b Example
 *   @verbatim

        // Reset the Port Arbiter Counters
        CSL_MSMC_setPortArbiterCounters(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setPortArbiterCounters(Uint32 accCyclesCounter)
{
    CSL_FINS (hMsmc->SMAEP, MSMC_SMAEP_ACWE, accCyclesCounter);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getPerfFilters
 *
 *   @b Description
 *   @n This function gets the performance counters filters
 *
 *   @b Arguments
     @verbatim
          ccf           CPU filter for counters
          pidf          System request PrivID filter for counters
          hm            Selects if SMAH:HCNT counts only cache hits or SRAM accesses
          pth           Priority threshold filter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n MSMC_SMAC_CCF, MSMC_SMAC_PIDF, MSMC_SMAC_HM, MSMC_SMAC_PTH
 *
 *   @b Example
 *   @verbatim
        Uint8   ccf;
        Uint16  pidf;
        Uint8   hm;
        Uint8   pth;

        // Get the Performance Filters
        CSL_MSMC_getPerfFilters(&ccf, &pidf, &hm, &pth);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getPerfFilters 
(
    Uint8*  ccf, 
    Uint16* pidf,
    Uint8*  hm, 
    Uint8*  pth    
)
{
    Uint32 value = hMsmc->SMAC;

    *ccf    = CSL_FEXT (value, MSMC_SMAC_CCF);
    *pidf   = CSL_FEXT (value, MSMC_SMAC_PIDF);
    *hm     = CSL_FEXT (value, MSMC_SMAC_HM);
    *pth    = CSL_FEXT (value, MSMC_SMAC_PTH);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setPerfFilters
 *
 *   @b Description
 *   @n This function sets the performance counters filters
 *
 *   @b Arguments
     @verbatim
          ccf           CPU filter for counters
          pidf          System request PrivID filter for counters
          hm            Selects if SMAH:HCNT counts only cache hits or SRAM accesses
          pth           Priority threshold filter
     @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAC_CCF, MSMC_SMAC_PIDF, MSMC_SMAC_HM, MSMC_SMAC_PTH
 *
 *   @b Example
 *   @verbatim
        Uint8   ccf;
        Uint16  pidf;
        Uint8   hm;
        Uint8   pth;

        // Get the Performance Filters
        CSL_MSMC_getPerfFilters(&ccf, &pidf, &hm, &pth);
        ...
        // Make sure we counter SRAM access also.
        CSL_MSMC_setPerfFilters(ccf, pidf, 1, pth);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setPerfFilters 
(
    Uint8   ccf, 
    Uint16  pidf,
    Uint8   hm, 
    Uint8   pth    
)
{
    hMsmc->SMAC = CSL_FMK(MSMC_SMAC_CCF,  ccf)   |
                  CSL_FMK(MSMC_SMAC_PIDF, pidf)  |
                  CSL_FMK(MSMC_SMAC_HM,   hm)    |
                  CSL_FMK(MSMC_SMAC_PTH,  pth);
}

/** ============================================================================
 *   @n@b CSL_MSMC_enableAnalysisCounter
 *
 *   @b Description
 *   @n This function enables the analysis counters
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAC_AEN=1
 *
 *   @b Example
 *   @verbatim
        
        // Enables the performance analysis counter
        CSL_MSMC_enableAnalysisCounter();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_enableAnalysisCounter (void)
{
    CSL_FINS (hMsmc->SMAC, MSMC_SMAC_AEN, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_disableAnalysisCounter
 *
 *   @b Description
 *   @n This function disables the analysis counters
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n MSMC_SMAC_AEN=0
 *
 *   @b Example
 *   @verbatim
        
        // Disables the performance analysis counter
        CSL_MSMC_disableAnalysisCounter();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_disableAnalysisCounter (void)
{
    CSL_FINS (hMsmc->SMAC, MSMC_SMAC_AEN, 0);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getPriorityLevel
 *
 *   @b Description
 *   @n This function gets the priority level used for software initiated
 *      global invalidate
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMCFG_INVP
 *
 *   @b Example
 *   @verbatim
        Uint8   priorityLevel;

        // Get the priority Level. 
        priorityLevel = CSL_MSMC_getPriorityLevel ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_MSMC_getPriorityLevel (void)
{
	return CSL_FEXT (hMsmc->SMCFG, MSMC_SMCFG_INVP);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setPriorityLevel
 *
 *   @b Description
 *   @n This function sets the priority level used for software initiated
 *      global invalidate
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMCFG_INVP
 *
 *   @b Example
 *   @verbatim

        // Set the priority Level. 
        CSL_MSMC_setPriorityLevel (0x1);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setPriorityLevel (Uint8 priorityLevel)
{
	CSL_FINS (hMsmc->SMCFG, MSMC_SMCFG_INVP, priorityLevel);
}

/** ============================================================================
 *   @n@b CSL_MSMC_freezeCache
 *
 *   @b Description
 *   @n This function freezes the MSMC cache
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMCFG_EFZ=1
 *
 *   @b Example
 *   @verbatim

        // Freeze the MSMC Cache
        CSL_MSMC_freezeCache ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_freezeCache (void)
{
	CSL_FINS (hMsmc->SMCFG, MSMC_SMCFG_EFZ, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_unFreezeCache
 *
 *   @b Description
 *   @n This function unfreezes the MSMC cache
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Writes
 *   @n MSMC_SMCFG_EFZ=0
 *
 *   @b Example
 *   @verbatim

        // Unfreeze the MSMC Cache
        CSL_MSMC_unFreezeCache ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_unFreezeCache (void)
{
	CSL_FINS (hMsmc->SMCFG, MSMC_SMCFG_EFZ, 0);
}

/** ============================================================================
 *   @n@b CSL_MSMC_isCacheFrozen
 *
 *   @b Description
 *   @n This function checks if the MSMC cache is frozen or not?
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE   - Cache is frozen
 *	 @n	 FALSE - Cache is not frozen
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n MSMC_SMCFG_EFZ
 *
 *   @b Example
 *   @verbatim

        // Check the status of the cache
        if (CSL_MSMC_isCacheFrozen () == TRUE)
        {
            // Cache is frozen.
        }
        else
        {
            // Cache is NOT frozen.
        }

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_MSMC_isCacheFrozen (void)
{
	if (CSL_FEXT (hMsmc->SMCFG, MSMC_SMCFG_EFZ) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_MSMC_invalidateCache
 *
 *   @b Description
 *   @n This function does a global invalidation of all the lines in the cache.
 *
 *   @b Arguments
 *	 @n	 None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	All Cache lines are invalidated.
 *
 *   @b Writes
 *   @n MSMC_SMCFG_GIW=1
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_invalidateCache ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_invalidateCache (void)
{
	CSL_FINS (hMsmc->SMCFG, MSMC_SMCFG_GIW, 1);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getInvalidateCacheStatus
 *
 *   @b Description
 *   @n This function returns the value of invalidate bit in SMCFG register.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMCFG_GIW
 *
 *   @b Example
 *   @verbatim
        Uint32 invStatus;

        invStatus = CSL_MSMC_getInvalidateCacheStatus ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getInvalidateCacheStatus (void)
{
	return CSL_FEXT (hMsmc->SMCFG, MSMC_SMCFG_GIW);
}

/** ============================================================================
 *   @n@b CSL_MSMC_setCacheSize
 *
 *   @b Description
 *   @n This function sets the amount of memory configured as cache.
 *
 *   @b Arguments
     @verbatim
          cmode      size of cache
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	MSMC Cache Memory Size is configured.
 *
 *   @b Writes
 *   @n MSMC_SMCFG_CMODE
 *
 *   @b Example
 *   @verbatim
        CSL_MSMC_setCacheSize (2);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_setCacheSize (Uint32 cmode)
{
	CSL_FINS (hMsmc->SMCFG, MSMC_SMCFG_CMODE, cmode);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getCacheSize
 *
 *   @b Description
 *   @n This function returns the amount of memory configured as cache.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>
 *   @n Cache Size
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMCFG_CMODE
 *
 *   @b Example
 *   @verbatim
        Uint32  cacheSize;

        cacheSize = CSL_MSMC_getCacheSize ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getCacheSize (void)
{
	return CSL_FEXT (hMsmc->SMCFG, MSMC_SMCFG_CMODE);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSharedMemoryErrorAddress
 *
 *   @b Description
 *   @n This function returns the lower 32 bits of the 36 bit address used in the
 *   accessing the corrupted location.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *   @n Shared Memory Error Address
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMNCERRAR
 *
 *   @b Example
 *   @verbatim
        Uint32  memErrAddress;

        memErrAddress = CSL_MSMC_getSharedMemoryErrorAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getSharedMemoryErrorAddress (void)
{
	return hMsmc->SMNCERRAR;
}

/** ============================================================================
 *   @n@b CSL_MSMC_getSharedMemoryErrorAddressExtendedInfo
 *
 *   @b Description
 *   @n This function returns extended information i.e. upper 4 bits, priv id 
 *      which caused the error which accessing the corrupted location.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *   @n Shared Memory Extended Information
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMNCERRXR_SEEADDR, MSMC_SMNCERRXR_SEPID, MSMC_SMNCERRXR_SER
 *
 *   @b Example
 *   @verbatim
        Uint8  seeAddr;
        Uint8  sepId;
        Uint8  ser;

        CSL_MSMC_getSharedMemoryErrorAddressExtendedInfo (&seeAddr, &sepId, &ser);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MSMC_getSharedMemoryErrorAddressExtendedInfo 
(
    Uint8* seeAddr, 
    Uint8* sepId, 
    Uint8* ser
)
{
	Uint32 value = hMsmc->SMNCERRXR;

    *seeAddr = CSL_FEXT (value, MSMC_SMNCERRXR_SEEADDR);
    *sepId   = CSL_FEXT (value, MSMC_SMNCERRXR_SEPID);
    *ser     = CSL_FEXT (value, MSMC_SMNCERRXR_SER);
}

/** ============================================================================
 *   @n@b CSL_MSMC_getScrubbingEngineErrorAddress
 *
 *   @b Description
 *   @n This function returns the scrubbing engine address after a 2-bit non
 *   correctable address is detected.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  
 *   @n Address which caused the error
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n MSMC_SMNCEA_SENCA
 *
 *   @b Example
 *   @verbatim

        Uint32 errAddress;

        errAddress = CSL_MSMC_getScrubbingEngineErrorAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MSMC_getScrubbingEngineErrorAddress()
{
    return CSL_FEXT (hMsmc->SMNCEA, MSMC_SMNCEA_SENCA);
}

/* @} */

#ifdef __cplusplus
}
#endif

#endif /* _CSL_MSMCAUX_H_ */


