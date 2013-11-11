/* ============================================================================
 * Copyright (c) Texas Instruments Incorporated 2008, 2009
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
 * @file csl_xmcAux.h
 *
 * @brief 
 *  API Auxilary header file for XMC CSL. It gives the definitions of the 
 *  status query & control functions.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par  
 */

#ifndef _CSL_XMCAUX_H_
#define _CSL_XMCAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_xmc.h>

/** @addtogroup CSL_XMC_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_XMC_setXMPAXH
 *
 *   @b Description
 *   @n This function sets the contents of XMPAXH register indicated by the
 * 		index here.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 16 registers (0-15)
        mpaxh       CSL_XMC_XMPAXH structure that needs to be set into the
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
 *	 @n	 Corresponding XMPAXH register configured with the value passed.
 *
 *   @b Writes
 * 	 @n XMC_XMPAXH_SEGSZ, XMC_XMPAXH_BADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_XMC_XMPAXH mpaxh;

        mpaxh.segSize = 4;
        mpaxh.baseAddress = 0x1000;

        CSL_XMC_setXMPAXH (index, &mpaxh);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_setXMPAXH 
(
	Uint32          index,
	CSL_XMC_XMPAXH* mpaxh
)
{
    hXmc->XMPAX[index].XMPAXH = CSL_FMK(XMC_XMPAXH_SEGSZ, mpaxh->segSize) |
                                CSL_FMK(XMC_XMPAXH_BADDR, mpaxh->bAddr);
}

/** ============================================================================
 *   @n@b CSL_XMC_setXMPAXL
 *
 *   @b Description
 *   @n This function sets the contents of XMPAXL register corresponding to the
 * 		index specified. 
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 16 registers (0-15)
        mpaxl       CSL_XMC_XMPAXL structure that needs to be set into the
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
 *	 @n	Corresponding XMPAXL register configured with the value passed.
 *
 *   @b Writes
 *   @n XMC_XMPAXL_RADDR, 
 *      XMC_XMPAXL_UX, 
 *      XMC_XMPAXL_UW,
 *      XMC_XMPAXL_UR, 
 *      XMC_XMPAXL_SX, 
 *      XMC_XMPAXL_SW,
 *      XMC_XMPAXL_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_XMC_XMPAXL mpaxl;

        mpaxl.ux = 1;
        mpaxl.uw = 1;
        mpaxl.ur = 1;
        mpaxl.sx = 1;
        mpaxl.sw = 1;
        mpaxl.sr = 1;
        mpaxl.rAddr = 0x4000 ;

        CSL_XMC_setXMPAXL (index, &mpaxl);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_setXMPAXL 
(
	Uint32           index,
	CSL_XMC_XMPAXL*  mpaxl
)
{
    Uint32 value = 0;

	/* Configure the XMPAXL register specified by the index. */	
	CSL_FINS (value, XMC_XMPAXL_UX, mpaxl->ux);
	CSL_FINS (value, XMC_XMPAXL_UW, mpaxl->uw);
	CSL_FINS (value, XMC_XMPAXL_UR, mpaxl->ur);
	CSL_FINS (value, XMC_XMPAXL_SX, mpaxl->sx);
	CSL_FINS (value, XMC_XMPAXL_SW, mpaxl->sw);
	CSL_FINS (value, XMC_XMPAXL_SR, mpaxl->sr);
	CSL_FINS (value, XMC_XMPAXL_RADDR, mpaxl->rAddr);

	hXmc->XMPAX[index].XMPAXL = value;
}

/** ============================================================================
 *   @n@b CSL_XMC_getXMPAXH
 *
 *   @b Description
 *   @n This function gets the contents of XMPAXH register.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 16 XMPAXH registers
        mpaxh       CSL_XMC_XMPAXH structure that needs to be populated with
                    XMPAXH register contents.
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
 *   @n XMC_XMPAXH_SEGSZ, XMC_XMPAXH_BADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_XMC_XMPAXH mpaxh;

        CSL_XMC_getXMPAXH (index, &mpaxh);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_getXMPAXH 
(
	Uint32           index,
	CSL_XMC_XMPAXH*  mpaxh
)
{
    Uint32 value = hXmc->XMPAX[index].XMPAXH;

	/* Retrieve contents of XMPAXH register specified by the index */		
	mpaxh->segSize = CSL_FEXT (value, XMC_XMPAXH_SEGSZ);
	mpaxh->bAddr   = CSL_FEXT (value, XMC_XMPAXH_BADDR);
}

/** ============================================================================
 *   @n@b CSL_XMC_getXMPAXL
 *
 *   @b Description
 *   @n This function gets the contents of XMPAXL register.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 16 registers
        mpaxl       CSL_XMC_XMPAXL structure that needs to be populated with
                    XMPAXL register contents.
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
 *   @n XMC_XMPAXL_RADDR, 
 *      XMC_XMPAXL_UX, 
 *      XMC_XMPAXL_UW,
 *      XMC_XMPAXL_UR, 
 *      XMC_XMPAXL_SX, 
 *      XMC_XMPAXL_SW,
 *      XMC_XMPAXL_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_XMC_XMPAXL mpaxl;

        CSL_XMC_getXMPAXL (index, &mpaxl);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_getXMPAXL (
	Uint32 index,
	CSL_XMC_XMPAXL * mpaxl
)
{
    Uint32 value = hXmc->XMPAX[index].XMPAXL;

	/* Retrieve contents of XMPAXL register using the index specified */		
	mpaxl->rAddr = CSL_FEXT (value, XMC_XMPAXL_RADDR);
	mpaxl->sr = CSL_FEXT (value, XMC_XMPAXL_SR);
	mpaxl->sw = CSL_FEXT (value, XMC_XMPAXL_SW);
	mpaxl->sx = CSL_FEXT (value, XMC_XMPAXL_SX);
	mpaxl->ur = CSL_FEXT (value, XMC_XMPAXL_UR);
	mpaxl->uw = CSL_FEXT (value, XMC_XMPAXL_UW);
	mpaxl->ux = CSL_FEXT (value, XMC_XMPAXL_UX);
}

/** ============================================================================
 *   @n@b CSL_XMC_getFaultAddress
 *
 *   @b Description
 *   @n This function gets the access address causing the fault.
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
 *   @n XMC_XMPFAR_FADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 faultAddr;

        faultAddr = CSL_XMC_getFaultAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_XMC_getFaultAddress (void)
{
	return CSL_FEXT (hXmc->XMPFAR, XMC_XMPFAR_FADDR);
}

/** ============================================================================
 *   @n@b CSL_XMC_clearFault
 *
 *   @b Description
 *   @n This function clears the fault information.
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
 *	 @n	XMPFCR register configured with the value passed. Fault 
 *	    address and status registers are cleared.
 *
 *   @b Writes
 *   @n XMC_XMPFCR_MPFCLR=1
 *
 *   @b Affects
 *   @n XMC_XMPFAR_FADDR=0,
 *      XMC_XMPFSR_LOCAL=0, 
 * 		XMC_XMPFSR_SR=0, 
 * 		XMC_XMPFSR_SX=0,
 * 		XMC_XMPFSR_UR=0, 
 * 		XMC_XMPFSR_UW=0, 
 * 		XMC_XMPFSR_UW=0
 *
 *   @b Example
 *   @verbatim
        CSL_XMC_clearFault ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_clearFault (void)
{
	hXmc->XMPFCR = CSL_FMK(XMC_XMPFCR_MPFCLR, 1);
}

/** ============================================================================
 *   @n@b CSL_XMC_getFaultStatus
 *
 *   @b Description
 *   @n This function gets the contents of Fault Status Register XMPFSR.
 *
 *   @b Arguments
     @verbatim
          xmpfsr    CSL_XMC_MPFSR structure that needs to be filled in from
                    XMPFSR register
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
 *   @n XMC_XMPFSR_LOCAL, 
 * 		XMC_XMPFSR_SR, 
 * 		XMC_XMPFSR_SX,
 * 		XMC_XMPFSR_UR, 
 * 		XMC_XMPFSR_UW, 
 * 		XMC_XMPFSR_UW
 *
 *   @b Example
 *   @verbatim
        CSL_XMC_MPFSR xmpfsr;

        CSL_XMC_getFaultStatus (&xmpfsr);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_getFaultStatus (CSL_XMC_MPFSR * xmpfsr)
{
    Uint32 value = hXmc->XMPFSR;

	xmpfsr->local = CSL_FEXT (value, XMC_XMPFSR_LOCAL);
	xmpfsr->sr = CSL_FEXT (value, XMC_XMPFSR_SR);
	xmpfsr->sw = CSL_FEXT (value, XMC_XMPFSR_SW);
	xmpfsr->sx = CSL_FEXT (value, XMC_XMPFSR_SX);
	xmpfsr->ur = CSL_FEXT (value, XMC_XMPFSR_UR);
	xmpfsr->uw = CSL_FEXT (value, XMC_XMPFSR_UW);
	xmpfsr->ux = CSL_FEXT (value, XMC_XMPFSR_UX);
}

/** ============================================================================
 *   @n@b CSL_XMC_invalidatePrefetchBuffer
 *
 *   @b Description
 *   @n This function when called marks all slots in the data prefetch buffer and program
 * 		prefetch buffer invalid by writing 1 in the XPFCMD register's INV bit.
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
 *	 @n	CSL_XMC_XPFCMD_INV bit set to 1 and data and program prefetch buffers are 
 * 		invalidated.
 *
 *   @b Writes
 *   @n XMC_XPFCMD_INV=1
 *
 *   @b Affects
 *   @n XMC_XPFADDR_DVH=0,
 *      XMC_XPFADDR_DVL=0
 *
 *
 *   @b Example
 *   @verbatim
 * 
        CSL_XMC_invalidatePrefetchBuffer ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_invalidatePrefetchBuffer (void)
{
	hXmc->XPFCMD = CSL_FMK(XMC_XPFCMD_INV, 1);
}

/** ============================================================================
 *   @n@b CSL_XMC_loadAnalysisCounterEnable
 *
 *   @b Description
 *   @n This function when called sets the 'ACENL' bit of the XPFCMD register, thus
 *      triggering a copy/load of ACEN bits (Analysis counter enable mode) into 
 *      ACEN bits of XPFACS register.
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
 *	 @n	CSL_XMC_XPFCMD_ACENL bit set to 1 and ACEN bits value is loaded into the
 *	    ACEN bits of XPFACS register.
 *
 *   @b Writes
 *   @n XMC_XPFCMD_ACENL=1
 *
 *   @b Affects
 *   @n XMC_XPFACS_ACEN
 *
 *   @b Example
 *   @verbatim
 * 
        CSL_XMC_loadAnalysisCounterEnable ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_loadAnalysisCounterEnable (void)
{
	hXmc->XPFCMD = CSL_FMK (XMC_XPFCMD_ACENL, 1);
}

/** ============================================================================
 *   @n@b CSL_XMC_setAnalysisCounterEnableMode
 *
 *   @b Description
 *   @n This function when called sets up the Event counting mode by writing to
 *      the 'ACEN' bits of the XPFCMD register.
 *
 *   @b Arguments
     @verbatim
          acenMode  Analyis counter mode.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	CSL_XMC_XPFCMD_ACEN bits written with the Analysis counter mode specified.
 *
 *   @b Writes
 *   @n XMC_XPFCMD_ACEN
 *
 *   @b Example
 *   @verbatim
 
        // Enable both program and data events
        CSL_XMC_setAnalysisCounterEnableMode (CSL_XMC_ACEN_MODE_COUNT_ALL);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_setAnalysisCounterEnableMode (CSL_XMC_ACEN_MODE acenMode)
{
	hXmc->XPFCMD = CSL_FMK(XMC_XPFCMD_ACEN, acenMode);
}

/** ============================================================================
 *   @n@b CSL_XMC_resetAnalysisCounters
 *
 *   @b Description
 *   @n This function when called sets the 'ACRST' bit of the XPFCMD register, thus
 *      triggering a clear/reset operation on the prefetch analysis counter registers.
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
 *	 @n	CSL_XMC_XPFCMD_ACRST bit set to 1 and prefetch analysis counter registers
 *	    are cleared.
 *
 *   @b Writes
 *   @n XMC_XPFCMD_ACRST=1
 *
 *   @b Affects
 *   @n XMC_XPFAC0_SENT=0,
 *      XMC_XPFAC1_CANCELED=0,
 *      XMC_XPFAC2_HIT=0,
 *      XMC_XPFAC3_MISS=0
 *
 *
 *   @b Example
 *   @verbatim
 * 
        CSL_XMC_resetAnalysisCounters ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_resetAnalysisCounters (void)
{
	hXmc->XPFCMD = CSL_FMK(XMC_XPFCMD_ACRST, 1);
}

/** ============================================================================
 *   @n@b CSL_XMC_setPrefetchCommand
 *
 *   @b Description
 *   @n This function sets up the contents of the XPFCMD register based on the
 *      inputs specified.
 *
 *   @b Arguments
     @verbatim
        inv         Invalidate Prefetch Buffer command flag. When set to 1,
                    data and program prefetch buffers are invalidated.
        acEnLoad    Analysis Counter Enable Load flag. When set to 1, the
                    Analysis Counter Enable mode is loaded into the XPFACS status
                    register.
        acEnMode    Analyis counter enable mode. Specifies the event counting mode
                    for the prefetcher.
        acRst       Analysis counter Reset flag. When set to 1, resets all
                    the prefetch event counters.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	CSL_XMC_XPFCMD written with the settings specified.
 *
 *   @b Writes
 *   @n XMC_XPFCMD_INV,
 *      XMC_XPFCMD_ACENL,
 *      XMC_XPFCMD_ACEN,
 *      XMC_XPFCMD_ACRST
 *
 *   @b Example
 *   @verbatim
        // Issue the following prefetch commands:
        //  - Dont invalidate prefetch buffers
        //  - Enable Analysis Counter Enable load
        //  - Enable both program and data event counting
        //  - Dont reset the prefetch counters
        CSL_XMC_setPrefetchCommand (0, 1, CSL_XMC_ACEN_MODE_COUNT_ALL, 0);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_setPrefetchCommand 
(
    Uint8               inv,
    Uint8               acEnLoad,
    CSL_XMC_ACEN_MODE   acenMode,
    Uint8               acRst
)
{
    hXmc->XPFCMD    =   CSL_FMK (XMC_XPFCMD_INV, inv)        |
                        CSL_FMK (XMC_XPFCMD_ACENL, acEnLoad) |
                        CSL_FMK (XMC_XPFCMD_ACEN, acenMode)  |
                        CSL_FMK (XMC_XPFCMD_ACRST, acRst);
}

/** ============================================================================
 *   @n@b CSL_XMC_getAnalysisCounterEnableStatus
 *
 *   @b Description
 *   @n This function returns the Analysis counter enable mode by reading the
 *      contents of 'ACEN' bits from the XPFACS register.
 *
 *   @b Arguments
 *	 @n	None
 *
 *   <b> Return Value </b>  CSL_XMC_ACEN_MODE   - Event counting mode
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	CSL_XMC_XPFACS_ACEN bits read and returned
 *
 *   @b Reads
 *   @n XMC_XPFACS_ACEN
 *
 *   @b Example
 *   @verbatim
        CSL_XMC_ACEN_MODE   acenMode;

        // Read ACEN Mode Status
        acenMode = CSL_XMC_getAnalysisCounterEnableStatus ();
        if (acenMode == CSL_XMC_ACEN_MODE_COUNT_DISABLE)
        {
            ...
        }

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE CSL_XMC_ACEN_MODE CSL_XMC_getAnalysisCounterEnableStatus (void)
{
	return (CSL_XMC_ACEN_MODE) CSL_FEXT (hXmc->XPFACS, XMC_XPFACS_ACEN);
}

/** ============================================================================
 *   @n@b CSL_XMC_getNumPrefetchSent
 *
 *   @b Description
 *   @n This function returns the contents of XPFAC0 register, i.e., the number
 *      of prefetch requests sent into the system.
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
 *	 @n	None
 *
 *   @b Reads
 *   @n XMC_XPFAC0_SENT
 *
 *   @b Example
 *   @verbatim
        Uint32      numPrefetchSent;

        // Read Prefetch request sent count
        numPrefetchSent = CSL_XMC_getNumPrefetchSent ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_XMC_getNumPrefetchSent (void)
{
	return CSL_FEXT (hXmc->XPFAC0, XMC_XPFAC0_SENT);
}

/** ============================================================================
 *   @n@b CSL_XMC_getNumPrefetchCanceled
 *
 *   @b Description
 *   @n This function returns the contents of XPFAC1 register, i.e., the number
 *      of prefetch requests canceled (prefetch requests that returned a non-zero 
 *      rstatus or other error).
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
 *	 @n	None
 *
 *   @b Reads
 *   @n XMC_XPFAC1_CANCELED
 *
 *   @b Example
 *   @verbatim
        Uint32      numPrefetchCanceled;

        // Read Prefetch request canceled
        numPrefetchCanceled = CSL_XMC_getNumPrefetchCanceled ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_XMC_getNumPrefetchCanceled (void)
{
	return CSL_FEXT (hXmc->XPFAC1, XMC_XPFAC1_CANCELED);
}

/** ============================================================================
 *   @n@b CSL_XMC_getNumPrefetchHits
 *
 *   @b Description
 *   @n This function returns the contents of XPFAC2 register, i.e., the number
 *      of demand prefetch requests that resulted in a successful prefetch.
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
 *	 @n	None
 *
 *   @b Reads
 *   @n XMC_XPFAC2_HIT
 *
 *   @b Example
 *   @verbatim
        Uint32      numPrefetchHits;

        // Read Number of Prefetch request hits
        numPrefetchHits = CSL_XMC_getNumPrefetchHits ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_XMC_getNumPrefetchHits (void)
{
	return CSL_FEXT (hXmc->XPFAC2, XMC_XPFAC2_HIT);
}

/** ============================================================================
 *   @n@b CSL_XMC_getNumPrefetchMisses
 *
 *   @b Description
 *   @n This function returns the contents of XPFAC3 register, i.e., the number
 *      of prefetch requests that resulted in a miss and had to be submitted as
 *      a demand prefetch request into the system.
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
 *	 @n	None
 *
 *   @b Reads
 *   @n XMC_XPFAC3_MISS
 *
 *   @b Example
 *   @verbatim
        Uint32      numPrefetchMisses;

        // Read Number of Prefetch request misses
        numPrefetchMisses = CSL_XMC_getNumPrefetchMisses ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_XMC_getNumPrefetchMisses (void)
{
	return CSL_FEXT (hXmc->XPFAC3, XMC_XPFAC3_MISS);
}

/** ============================================================================
 *   @n@b CSL_XMC_getPrefetchAddress
 *
 *   @b Description
 *   @n This function gets the contents of XPFADDR register.
 *
 *   @b Arguments
     @verbatim
          index     Index into the set of 8 registers
          xpfaddr   CSL_XMC_XPFADDR structure that needs to be filled from
                    XPFADDR register
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
 *   @n XMC_XPFADDR_ADDR, 
 * 		XMC_XPFADDR_DIR, 
 * 		XMC_XPFADDR_DPH,
 * 		XMC_XPFADDR_DVH, 
 * 		XMC_XPFADDR_AVH, 
 * 		XMC_XPFADDR_DPL,
 * 		XMC_XPFADDR_DVL, 
 * 		XMC_XPFADDR_AVL
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_XMC_XPFADDR xpfaddr;

        CSL_XMC_getPrefetchAddress (index, &xpfaddr);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_getPrefetchAddress 
(
    Uint32              index,
	CSL_XMC_XPFADDR*    xpfaddr
)
{
    Uint32 value = hXmc->XPFADDR[index];

	xpfaddr->addr = CSL_FEXT (value, XMC_XPFADDR_ADDR);
	xpfaddr->dir  = CSL_FEXT (value, XMC_XPFADDR_DIR);
	xpfaddr->dph  = CSL_FEXT (value, XMC_XPFADDR_DPH);
	xpfaddr->dvh  = CSL_FEXT (value, XMC_XPFADDR_DVH);
	xpfaddr->avh  = CSL_FEXT (value, XMC_XPFADDR_AVH);
	xpfaddr->dpl  = CSL_FEXT (value, XMC_XPFADDR_DPL);
	xpfaddr->dvl  = CSL_FEXT (value, XMC_XPFADDR_DVL);
	xpfaddr->avl  = CSL_FEXT (value, XMC_XPFADDR_AVL);
}

/** ============================================================================
 *   @n@b CSL_XMC_setMDMAPriority
 *
 *   @b Description
 *   @n This function configures the priority of transaction submissions to 
 *    	Master DMA (MDMA).
 *
 *   @b Arguments
     @verbatim
        priority        Priority value to set for MDMA transactions.
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
 *   @n XMC_MDMAARBX_PRI
 *
 *   @b Example
 *   @verbatim
        Uint32 priority = 4;

        CSL_XMC_setMDMAPriority (priority);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_XMC_setMDMAPriority (Uint32 priority)
{
	CSL_FINS (hXmc->MDMAARBX, XMC_MDMAARBX_PRI, priority);
}

/** ============================================================================
 *   @n@b CSL_XMC_getMDMAPriority
 *
 *   @b Description
 *   @n This function retrieves the priority of transaction submissions to 
 *    	Master DMA (MDMA).
 *
 *   @b Arguments
 *	 @n	 None
 *
 *  <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XMC_MDMAARBX_PRI
 *
 *   @b Example
 *   @verbatim
        Uint32 priority;

        priority = CSL_XMC_getMDMAPriority ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_XMC_getMDMAPriority (void)
{
	return CSL_FEXT (hXmc->MDMAARBX, XMC_MDMAARBX_PRI);
}


#ifdef __cplusplus
}
#endif

/* @} */
#endif /* _CSL_XMCAUX_H_ */

