/* ============================================================================
 * Copyright (c) Texas Instruments Incorporated 2009                 
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
 * @file  csl_memprotAux.h
 *
 * @brief  
 *  API Auxilary header file for MEMPROT CSL. Contains the different control 
 *  command and status query functions definations
 *   
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par   
 */

#ifndef _CSL_MEMPROTAUX_H
#define _CSL_MEMPROTAUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_memprot.h>

/** @addtogroup CSL_MEMPROT_FUNCTION
@{ */

/** ============================================================================
 *   @n@b CSL_MEMPROT_getL2FaultAddress
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
 *   @n CGEM_L2MPFAR_ADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 faultAddr;

        faultAddr = CSL_MEMPROT_getL2FaultAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MEMPROT_getL2FaultAddress (
	void
)
{

	return CSL_FEXT (hCgemRegs->L2MPFAR, CGEM_L2MPFAR_ADDR);
}



/** ============================================================================
 *   @n@b CSL_MEMPROT_clearL2Fault
 *
 *   @b Description
 *   @n This function clears the L2 fault information.
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
 *	 @n	CSL_CGEM_L2MPFCR_MPFCLR register bit field set to one. Previous
 * 		Fault information is cleared from the Fault Status and Address registers.
 *
 *   @b Writes
 *   @n CGEM_L2MPFCR_MPFCLR=1 
 *
 *   @b Affects
 *   @n CGEM_L2MPFAR_ADDR=0,
 *      CGEM_L2MPFSR_FID=0, 
 * 		CGEM_L2MPFSR_LOCAL=0, 
 * 		CGEM_L2MPFSR_NS=0,
 * 		CGEM_L2MPFSR_SR=0, 
 * 		CGEM_L2MPFSR_SW=0, 
 * 		CGEM_L2MPFSR_SX=0,
 * 		CGEM_L2MPFSR_UR=0, 
 * 		CGEM_L2MPFSR_UW=0, 
 * 		CGEM_L2MPFSR_UX=0   
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_clearL2Fault ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_clearL2Fault (
	void
)
{

	hCgemRegs->L2MPFCR  =   CSL_FMK (CGEM_L2MPFCR_MPFCLR, 1);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_getL2FaultStatus
 *
 *   @b Description
 *   @n This function gets the contents of L2 Fault Status Register L2MPFSR.
 *
 *   @b Arguments
     @verbatim
        xmpfsr      CSL_MEMPROT_MPFSR structure that needs to be filled in from
                    L2MPFSR register
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
 *   @n CGEM_L2MPFSR_FID, 
 * 		CGEM_L2MPFSR_LOCAL, 
 * 		CGEM_L2MPFSR_NS,
 * 		CGEM_L2MPFSR_SR, 
 * 		CGEM_L2MPFSR_SW, 
 * 		CGEM_L2MPFSR_SX,
 * 		CGEM_L2MPFSR_UR, 
 * 		CGEM_L2MPFSR_UW, 
 * 		CGEM_L2MPFSR_UX
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_MPFSR mpfsr;

        CSL_MEMPROT_getL2FaultStatus (&mpfsr);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL2FaultStatus (
	CSL_MEMPROT_MPFSR * mpfsr
)
{
    Uint32  tmp;

    tmp =   hCgemRegs->L2MPFSR;
	mpfsr->fid = CSL_FEXT (tmp, CGEM_L2MPFSR_FID);	
	mpfsr->local = CSL_FEXT (tmp, CGEM_L2MPFSR_LOCAL);
	mpfsr->ns = CSL_FEXT (tmp, CGEM_L2MPFSR_NS);
	mpfsr->sr = CSL_FEXT (tmp, CGEM_L2MPFSR_SR);
	mpfsr->sw = CSL_FEXT (tmp, CGEM_L2MPFSR_SW);
	mpfsr->sx = CSL_FEXT (tmp, CGEM_L2MPFSR_SX);
	mpfsr->ur = CSL_FEXT (tmp, CGEM_L2MPFSR_UR);
	mpfsr->uw = CSL_FEXT (tmp, CGEM_L2MPFSR_UW);
	mpfsr->ux = CSL_FEXT (tmp, CGEM_L2MPFSR_UX);

	return;
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_setL2Attributes
 *
 *   @b Description
 *   @n This function sets the contents of L2MPPA register corresponding to the
 * 		index specified. 
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 32 registers (0-31)
        mppa        CSL_MEMPROT_MPPA structure that needs to be set into the
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
 *	 @n	L2MPPA register corresponding to index specified is configured with 
 * 		the attributes passed in mppa argument.
 *
 *   @b Writes
 *   @n CGEM_L2MPPA_AID5, 
 * 		CGEM_L2MPPA_AID4, 
 * 		CGEM_L2MPPA_AID3, 
 * 		CGEM_L2MPPA_AID2, 
 * 		CGEM_L2MPPA_AID1, 
 * 		CGEM_L2MPPA_AID0,
 * 		CGEM_L2MPPA_AIDX, 
 * 		CGEM_L2MPPA_LOCAL, 
 * 		CGEM_L2MPPA_UX,
 * 		CGEM_L2MPPA_UW, 
 * 		CGEM_L2MPPA_UR, 
 * 		CGEM_L2MPPA_SX,
 * 		CGEM_L2MPPA_SW, 
 * 		CGEM_L2MPPA_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_MEMPROT_MPPA mppa;

        mppa->aid5 = 1;
        mppa->aid4 = 1;
        mppa->aid3 = 1;
        mppa->aid2 = 1;
        mppa->aid1 = 1;	
        mppa->aid0 = 1;
        mppa->aidx = 1;		
        mppa->local = 1;
        mppa->ux = 1;
        mppa->uw = 1;
        mppa->ur = 1;
        mppa->sx = 1;
        mppa->sw = 1;
        mppa->sr = 1;

        CSL_MEMPROT_setL2Attributes (index, &mppa);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_setL2Attributes (
	Uint32 index,
	CSL_MEMPROT_MPPA * mppa
)
{
	/* Configure the L2MPPA register specified by the index. */
	hCgemRegs->L2MPPA[index]    =   CSL_FMK (CGEM_L2MPPA_AID5, mppa->aid5) |
	                                CSL_FMK (CGEM_L2MPPA_AID4, mppa->aid4) |
	                                CSL_FMK (CGEM_L2MPPA_AID3, mppa->aid3) |
	                                CSL_FMK (CGEM_L2MPPA_AID2, mppa->aid2) |
	                                CSL_FMK (CGEM_L2MPPA_AID1, mppa->aid1) |
	                                CSL_FMK (CGEM_L2MPPA_AID0, mppa->aid0) |
	                                CSL_FMK (CGEM_L2MPPA_AIDX, mppa->aidx) |
	                                CSL_FMK (CGEM_L2MPPA_LOCAL, mppa->local) |
	                                CSL_FMK (CGEM_L2MPPA_UX, mppa->ux) |	
	                                CSL_FMK (CGEM_L2MPPA_UW, mppa->uw) |
                                    CSL_FMK (CGEM_L2MPPA_UR, mppa->ur) |
	                                CSL_FMK (CGEM_L2MPPA_SX, mppa->sx) |
                                    CSL_FMK (CGEM_L2MPPA_SW, mppa->sw) |
	                                CSL_FMK (CGEM_L2MPPA_SR, mppa->sr);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_getL2Attributes
 *
 *   @b Description
 *   @n This function gets the L2 memory protection attributes by reading the 
 * 		contents of L2MPPA register corresponding to the index specified.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 32 MPPA registers
        mpaxl       CSL_MEMPROT_MPPA structure that needs to be populated with
                    L2MPPA register contents.
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
 *   @n CGEM_L2MPPA_AID5, 
 *      CGEM_L2MPPA_AID4, 
 *      CGEM_L2MPPA_AID3, 
 *      CGEM_L2MPPA_AID2, 
 *      CGEM_L2MPPA_AID1, 
 *      CGEM_L2MPPA_AID0,
 *      CGEM_L2MPPA_AIDX, 
 *      CGEM_L2MPPA_LOCAL, 
 *      CGEM_L2MPPA_UX,
 *      CGEM_L2MPPA_UW, 
 *      CGEM_L2MPPA_UR, 
 *      CGEM_L2MPPA_SX,
 *      CGEM_L2MPPA_SW, 
 *      CGEM_L2MPPA_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_MEMPROT_MPPA mppa;

        CSL_MEMPROT_getL2Attributes (index, &mppa);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL2Attributes (
	Uint32 index,
	CSL_MEMPROT_MPPA * mppa
)
{
    Uint32 tmp;

	/* Retrieve contents of L2MPPA register using the index specified */	
    tmp =   hCgemRegs->L2MPPA[index];
	mppa->aid5 = CSL_FEXT (tmp, CGEM_L2MPPA_AID5);
	mppa->aid4 = CSL_FEXT (tmp, CGEM_L2MPPA_AID4);
	mppa->aid3 = CSL_FEXT (tmp, CGEM_L2MPPA_AID3);
	mppa->aid2 = CSL_FEXT (tmp, CGEM_L2MPPA_AID2);
	mppa->aid1 = CSL_FEXT (tmp, CGEM_L2MPPA_AID1);
	mppa->aid0 = CSL_FEXT (tmp, CGEM_L2MPPA_AID0);
	mppa->aidx = CSL_FEXT (tmp, CGEM_L2MPPA_AIDX);
	mppa->local = CSL_FEXT (tmp, CGEM_L2MPPA_LOCAL);						
	mppa->sr = CSL_FEXT (tmp, CGEM_L2MPPA_SR);
	mppa->sw = CSL_FEXT (tmp, CGEM_L2MPPA_SW);
	mppa->sx = CSL_FEXT (tmp, CGEM_L2MPPA_SX);
	mppa->ur = CSL_FEXT (tmp, CGEM_L2MPPA_UR);
	mppa->uw = CSL_FEXT (tmp, CGEM_L2MPPA_UW);
	mppa->ux = CSL_FEXT (tmp, CGEM_L2MPPA_UX);

	return;
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1PFaultAddress
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
 *   @n CGEM_L1PMPFAR_ADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 faultAddr;

        faultAddr = CSL_MEMPROT_getL1PFaultAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MEMPROT_getL1PFaultAddress (
	void
)
{

	return CSL_FEXT (hCgemRegs->L1PMPFAR, CGEM_L1PMPFAR_ADDR);
}



/** ============================================================================
 *   @n@b CSL_MEMPROT_clearL1PFault
 *
 *   @b Description
 *   @n This function clears the L1P fault information.
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
 *	 @n	CSL_CGEM_L1PMPFCR_MPFCLR register bit field set to one. Previous
 * 		Fault information is cleared from the Fault Status and Address registers.
 *
 *   @b Writes
 *   @n CGEM_L1PMPFCR_MPFCLR=1
 *
 *   @b Affects
 *   @n CGEM_L1PMPFAR_ADDR=0,
 *      CGEM_L1PMPFSR_FID=0, 
 * 		CGEM_L1PMPFSR_LOCAL=0, 
 * 		CGEM_L1PMPFSR_NS=0,
 * 		CGEM_L1PMPFSR_SR=0, 
 * 		CGEM_L1PMPFSR_SW=0, 
 * 		CGEM_L1PMPFSR_SX=0, 
 * 		CGEM_L1PMPFSR_UR=0, 
 * 		CGEM_L1PMPFSR_UW=0, 
 * 		CGEM_L1PMPFSR_UX=0
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_clearL1PFault ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_clearL1PFault (
	void
)
{

	hCgemRegs->L1PMPFCR =   CSL_FMK (CGEM_L1PMPFCR_MPFCLR, 1);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1PFaultStatus
 *
 *   @b Description
 *   @n This function gets the contents of L1P Fault Status Register L1PMPFSR.
 *
 *   @b Arguments
     @verbatim
        xmpfsr      CSL_MEMPROT_MPFSR structure that needs to be filled in from
                    L1PMPFSR register
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
 *   @n CGEM_L1PMPFSR_FID, 
 * 		CGEM_L1PMPFSR_LOCAL, 
 * 		CGEM_L1PMPFSR_NS,
 * 		CGEM_L1PMPFSR_SR, 
 * 		CGEM_L1PMPFSR_SW, 
 * 		CGEM_L1PMPFSR_SX, 
 * 		CGEM_L1PMPFSR_UR, 
 * 		CGEM_L1PMPFSR_UW, 
 * 		CGEM_L1PMPFSR_UX
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_MPFSR mpfsr;

        CSL_MEMPROT_getL1PFaultStatus (&mpfsr);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL1PFaultStatus (
	CSL_MEMPROT_MPFSR * mpfsr
)
{
    Uint32  tmp;

    tmp = hCgemRegs->L1PMPFSR;
	mpfsr->fid = CSL_FEXT (tmp, CGEM_L1PMPFSR_FID);	
	mpfsr->local = CSL_FEXT (tmp, CGEM_L1PMPFSR_LOCAL);
	mpfsr->ns = CSL_FEXT (tmp, CGEM_L1PMPFSR_NS);
	mpfsr->sr = CSL_FEXT (tmp, CGEM_L1PMPFSR_SR);
	mpfsr->sw = CSL_FEXT (tmp, CGEM_L1PMPFSR_SW);
	mpfsr->sx = CSL_FEXT (tmp, CGEM_L1PMPFSR_SX);
	mpfsr->ur = CSL_FEXT (tmp, CGEM_L1PMPFSR_UR);
	mpfsr->uw = CSL_FEXT (tmp, CGEM_L1PMPFSR_UW);
	mpfsr->ux = CSL_FEXT (tmp, CGEM_L1PMPFSR_UX);

	return;
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_setL1PAttributes
 *
 *   @b Description
 *   @n This function sets the L1P attributes, by configuring the contents of L1PMPPA 
 * 		register corresponding to the index specified. 
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 32 L1PMPPA registers (0-31)
        mppa        CSL_MEMPROT_MPPA structure that needs to be set into the
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
 *	 @n	L1PMPPA register configured with the value passed.
 *
 *   @b Writes
 *   @n CGEM_L1PMPPA_AID5, 
 * 		CGEM_L1PMPPA_AID4, 
 * 		CGEM_L1PMPPA_AID3, 
 * 		CGEM_L1PMPPA_AID2, 
 * 		CGEM_L1PMPPA_AID1, 
 * 		CGEM_L1PMPPA_AID0,
 * 		CGEM_L1PMPPA_AIDX, 
 * 		CGEM_L1PMPPA_LOCAL, 
 * 		CGEM_L1PMPPA_UX, 
 * 		CGEM_L1PMPPA_UW, 
 * 		CGEM_L1PMPPA_UR, 
 * 		CGEM_L1PMPPA_SX,
 * 		CGEM_L1PMPPA_SW, 
 * 		CGEM_L1PMPPA_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_MEMPROT_MPPA mppa;

        mppa->aid5 = 1;
        mppa->aid4 = 1;
        mppa->aid3 = 1;
        mppa->aid2 = 1;
        mppa->aid1 = 1;	
        mppa->aid0 = 1;
        mppa->aidx = 1;		
        mppa->local = 1;
        mppa->ux = 1;
        mppa->uw = 1;
        mppa->ur = 1;
        mppa->sx = 1;
        mppa->sw = 1;
        mppa->sr = 1;

        CSL_MEMPROT_setL1PAttributes (index, &mppa);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_setL1PAttributes (
    Uint32      index,
	CSL_MEMPROT_MPPA * mppa
)
{
	/* Configure the L1PMPPA register specified by the index. */
	hCgemRegs->L1PMPPA[index]   =   CSL_FMK (CGEM_L1PMPPA_AID5, mppa->aid5) |
	                                CSL_FMK (CGEM_L1PMPPA_AID4, mppa->aid4) |	
	                                CSL_FMK (CGEM_L1PMPPA_AID3, mppa->aid3) |
	                                CSL_FMK (CGEM_L1PMPPA_AID2, mppa->aid2) |
	                                CSL_FMK (CGEM_L1PMPPA_AID1, mppa->aid1) |
	                                CSL_FMK (CGEM_L1PMPPA_AID0, mppa->aid0) |	
	                                CSL_FMK (CGEM_L1PMPPA_AIDX, mppa->aidx) |
	                                CSL_FMK (CGEM_L1PMPPA_LOCAL, mppa->local) |
                                    CSL_FMK (CGEM_L1PMPPA_UX, mppa->ux) |	
	                                CSL_FMK (CGEM_L1PMPPA_UW, mppa->uw) |
	                                CSL_FMK (CGEM_L1PMPPA_UR, mppa->ur) |
	                                CSL_FMK (CGEM_L1PMPPA_SX, mppa->sx) |
	                                CSL_FMK (CGEM_L1PMPPA_SW, mppa->sw) |
	                                CSL_FMK (CGEM_L1PMPPA_SR, mppa->sr);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1PAttributes
 *
 *   @b Description
 *   @n This function gets the L1P attributes by reading the contents of L1PMPPA 
 * 		register corresponding to the index specified.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 32 L1PMPPA registers
        mpaxl       CSL_MEMPROT_MPPA structure that needs to be populated with
                    L1PMPPA register contents.
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
 *   @n CGEM_L1PMPPA_AID5, 
 *      CGEM_L1PMPPA_AID4, 
 *      CGEM_L1PMPPA_AID3, 
 *      CGEM_L1PMPPA_AID2, 
 *      CGEM_L1PMPPA_AID1, 
 *      CGEM_L1PMPPA_AID0,
 *      CGEM_L1PMPPA_AIDX, 
 *      CGEM_L1PMPPA_LOCAL, 
 *      CGEM_L1PMPPA_UX, 
 *      CGEM_L1PMPPA_UW, 
 *      CGEM_L1PMPPA_UR, 
 *      CGEM_L1PMPPA_SX,
 *      CGEM_L1PMPPA_SW, 
 *      CGEM_L1PMPPA_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_MEMPROT_MPPA mppa;

        CSL_MEMPROT_getL1PAttributes (index, &mppa);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL1PAttributes (
	Uint32 index,
	CSL_MEMPROT_MPPA * mppa
)
{
    Uint32 tmp;        

	/* Retrieve contents of L1PMPPA register using the index specified */	
    tmp =   hCgemRegs->L1PMPPA[index];
	mppa->aid5 = CSL_FEXT (tmp, CGEM_L1PMPPA_AID5);
	mppa->aid4 = CSL_FEXT (tmp, CGEM_L1PMPPA_AID4);
	mppa->aid3 = CSL_FEXT (tmp, CGEM_L1PMPPA_AID3);
	mppa->aid2 = CSL_FEXT (tmp, CGEM_L1PMPPA_AID2);
	mppa->aid1 = CSL_FEXT (tmp, CGEM_L1PMPPA_AID1);
	mppa->aid0 = CSL_FEXT (tmp, CGEM_L1PMPPA_AID0);
	mppa->aidx = CSL_FEXT (tmp, CGEM_L1PMPPA_AIDX);
	mppa->local = CSL_FEXT (tmp, CGEM_L1PMPPA_LOCAL);						
	mppa->sr = CSL_FEXT (tmp, CGEM_L1PMPPA_SR);
	mppa->sw = CSL_FEXT (tmp, CGEM_L1PMPPA_SW);
	mppa->sx = CSL_FEXT (tmp, CGEM_L1PMPPA_SX);
	mppa->ur = CSL_FEXT (tmp, CGEM_L1PMPPA_UR);
	mppa->uw = CSL_FEXT (tmp, CGEM_L1PMPPA_UW);
	mppa->ux = CSL_FEXT (tmp, CGEM_L1PMPPA_UX);

	return;
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1DFaultAddress
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
 *   @n CGEM_L1DMPFAR_ADDR
 *
 *   @b Example
 *   @verbatim
        Uint32 faultAddr;

        faultAddr = CSL_MEMPROT_getL1DFaultAddress ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_MEMPROT_getL1DFaultAddress (
	void
)
{

	return CSL_FEXT (hCgemRegs->L1DMPFAR, CGEM_L1DMPFAR_ADDR);
}



/** ============================================================================
 *   @n@b CSL_MEMPROT_clearL1DFault
 *
 *   @b Description
 *   @n This function clears the L1D fault information.
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
 *	 @n	CSL_CGEM_L1DMPFCR_MPFCLR register bit field set to one. Previous
 * 		Fault information is cleared from the Fault Status register.
 *
 *   @b Writes
 *   @n CGEM_L1DMPFCR_MPFCLR=1
 *
 *   @b Affects
 *   @n CGEM_L1DMPFAR_ADDR=0,
 *      CGEM_L1DMPFSR_FID=0, 
 * 		CGEM_L1DMPFSR_LOCAL=0, 
 * 		CGEM_L1DMPFSR_NS=0,
 * 		CGEM_L1DMPFSR_SR=0, 
 * 		CGEM_L1DMPFSR_SW=0, 
 * 		CGEM_L1DMPFSR_SX=0,
 * 		CGEM_L1DMPFSR_UR=0, 
 * 		CGEM_L1DMPFSR_UW=0, 
 * 		CGEM_L1DMPFSR_UX=0
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_clearL1DFault ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_clearL1DFault (
	void
)
{

	hCgemRegs->L1DMPFCR =   CSL_FMK(CGEM_L1DMPFCR_MPFCLR, 1);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1DFaultStatus
 *
 *   @b Description
 *   @n This function gets the contents of L1D Fault Status Register L1DMPFSR.
 *
 *   @b Arguments
     @verbatim
        xmpfsr      CSL_MEMPROT_MPFSR structure that needs to be filled in from
                    L1DMPFSR register
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
 *   @n CGEM_L1DMPFSR_FID, 
 * 		CGEM_L1DMPFSR_LOCAL, 
 * 		CGEM_L1DMPFSR_NS,
 * 		CGEM_L1DMPFSR_SR, 
 * 		CGEM_L1DMPFSR_SW, 
 * 		CGEM_L1DMPFSR_SX,
 * 		CGEM_L1DMPFSR_UR, 
 * 		CGEM_L1DMPFSR_UW, 
 * 		CGEM_L1DMPFSR_UX
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_MPFSR mpfsr;

        CSL_MEMPROT_getL1DFaultStatus (&mpfsr);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL1DFaultStatus (
	CSL_MEMPROT_MPFSR * mpfsr
)
{
    Uint32 tmp;

    tmp =   hCgemRegs->L1DMPFSR;
	mpfsr->fid = CSL_FEXT (tmp, CGEM_L1DMPFSR_FID);	
	mpfsr->local = CSL_FEXT (tmp, CGEM_L1DMPFSR_LOCAL);
	mpfsr->ns = CSL_FEXT (tmp, CGEM_L1DMPFSR_NS);
	mpfsr->sr = CSL_FEXT (tmp, CGEM_L1DMPFSR_SR);
	mpfsr->sw = CSL_FEXT (tmp, CGEM_L1DMPFSR_SW);
	mpfsr->sx = CSL_FEXT (tmp, CGEM_L1DMPFSR_SX);
	mpfsr->ur = CSL_FEXT (tmp, CGEM_L1DMPFSR_UR);
	mpfsr->uw = CSL_FEXT (tmp, CGEM_L1DMPFSR_UW);
	mpfsr->ux = CSL_FEXT (tmp, CGEM_L1DMPFSR_UX);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_setL1DKey
 *
 *   @b Description
 *   @n This function configures the MPLK register corresponding to the index
 * 		specified with the key value passed to this API.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 4 MPLK registers
        l1dkey      32 Lock bits to be put into the MPLK register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_MEMPROT_resetL1DLock() must be set to reset locks before calling this API.
 *
 *   <b> Post Condition </b>
 *	 @n	L1DMPLK register corresponding to index specified is configured 
 * 		with the key value passed in l1dkey argument.
 *
 *   @b Writes
 *   @n CGEM_MPLK_MPLK
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        Uint32 value = 0x32;

        CSL_MEMPROT_resetL1DLock ();		
        ...
        CSL_MEMPROT_setL1DKey (index, value);        
        ...
        CSL_MEMPROT_lockL1D ();
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_setL1DKey (Uint32 index, Uint32 l1dkey)
{
	hCgemRegs->MPLK[index]  =   CSL_FMK (CGEM_MPLK_MPLK, l1dkey);
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_resetL1DLock
 *
 *   @b Description
 *   @n This function resets the L1D Lock by issuing the KEYR command to 
 * 		MPLKCMD register.
 *
 *   @b Arguments	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n	KEYR bit in MPLKCMD register is set and lock reset sequence 
 * 		is set in motion.
 *
 *   @b Writes
 *   @n CGEM_MPLKCMD_KEYR=1
 *
 *   @b Example
 *   @verbatim

        CSL_MEMPROT_resetL1DLock ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_resetL1DLock (void)
{
	hCgemRegs->MPLKCMD  =   CSL_FMK (CGEM_MPLKCMD_KEYR, 1);
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_lockL1D
 *
 *   @b Description
 *   @n This function locks the L1D by issuing a LOCK command using the L1DMPLKCMD 
 * 		register.
 *
 *   @b Arguments	None
 *
 *   <b> Return Value </b>
 *	 @n None
 *
 *   <b> Pre Condition </b>
 *   @n @a CSL_MEMPROT_resetL1DLock() must be called to reset locks before calling 
 *      this API. Also, if a new key value needs to be configured it must be done 
 *      too using @a CSL_MEMPROT_setL1DKey() API before doing the lock operation.
 *
 *   <b> Post Condition </b>
 *   @n	LOCK bit set in the L1DMPLKCMD register.
 *
 *   @b Writes
 *   @n CGEM_MPLKCMD_LOCK=1
 *
 *   @b Affects
 *   @n CGEM_MPLKSTAT_LK=1
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        Uint32 value = 0x32;

        CSL_MEMPROT_resetL1DLock ();
        ...		
        CSL_MEMPROT_setL1DKey (index, value);        
        ...        
        CSL_MEMPROT_lockL1D ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_lockL1D (void)
{
	hCgemRegs->MPLKCMD  =   CSL_FMK (CGEM_MPLKCMD_LOCK, 1);
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_unlockL1D
 *
 *   @b Description
 *   @n This function unlocks L1D by issuing an UNLOCK command using the L1DMPLKCMD 
 * 		register.
 *
 *   @b Arguments	None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n @a CSL_MEMPROT_resetL1DLock() must be called to reset locks before calling 
 *      this API. Also, a valid key must be configured using @a CSL_MEMPROT_setL1DKey()
 *      API before calling this API.
 *
 *   <b> Post Condition </b>
 *   @n	UNLOCK bit set in the L1DMPLKCMD register.
 *
 *   @b Writes
 *   @n CGEM_MPLKCMD_UNLOCK=1
 *
 *   @b Affects
 *   @n CGEM_MPLKSTAT_LK=0
 *
 *   @b Example
 *   @verbatim
  
        CSL_MEMPROT_resetL1DLock ();
        ...
        CSL_MEMPROT_unlockL1D ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_unlockL1D (void)
{
	hCgemRegs->MPLKCMD  =   CSL_FMK (CGEM_MPLKCMD_UNLOCK, 1);
}

/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1DLockStatus
 *
 *   @b Description
 *   @n This function gets the L1D lock status from the contents of L1DMPLKSTAT register.
 *
 *   @b Arguments
     @verbatim
          mpfsr      	CSL_MEMPROT_MPLKSTAT structure that needs to be filled in from
          			  	L1DMPLKSTAT register
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
 *   @n CGEM_MPLKSTAT_NSL, CGEM_MPLKSTAT_LK
 *
 *   @b Example
 *   @verbatim
        CSL_MEMPROT_MPLKSTAT mplkstat;

        CSL_MEMPROT_getL1DLockStatus (&mplkstat);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL1DLockStatus (
	CSL_MEMPROT_MPLKSTAT * mplkstat
)
{
    Uint32 tmp;

    tmp = hCgemRegs->MPLKSTAT;
	mplkstat->nsl = CSL_FEXT (tmp, CGEM_MPLKSTAT_NSL);
	mplkstat->lk  = CSL_FEXT (tmp, CGEM_MPLKSTAT_LK);
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_setL1DAttributes
 *
 *   @b Description
 *   @n This function sets the contents of L1DMPPA register corresponding to the
 * 		index specified. 
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 32 registers (0-31)
        mppa        CSL_MEMPROT_MPPA structure that needs to be set into the
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
 *	 @n	L1DMPPA register corresponding to index specified is configured with 
 * 		the attributes passed.
 *
 *   @b Writes
 *   @n CGEM_L1DMPPA_AID5, 
 * 		CGEM_L1DMPPA_AID4, 
 * 		CGEM_L1DMPPA_AID3,
 *		CGEM_L1DMPPA_AID2, 
 * 		CGEM_L1DMPPA_AID1, 
 * 		CGEM_L1DMPPA_AID0,
 * 		CGEM_L1DMPPA_AIDX, 
 * 		CGEM_L1DMPPA_LOCAL, 
 * 		CGEM_L1DMPPA_UW, 
 * 		CGEM_L1DMPPA_UR, 
 * 		CGEM_L1DMPPA_SW, 
 * 		CGEM_L1DMPPA_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_MEMPROT_MPPA mppa;

        mppa->aid5 = 1;
        mppa->aid4 = 1;
        mppa->aid3 = 1;
        mppa->aid2 = 1;
        mppa->aid1 = 1;	
        mppa->aid0 = 1;
        mppa->aidx = 1;		
        mppa->local = 1;
        mppa->uw = 1;
        mppa->ur = 1;
        mppa->sw = 1;
        mppa->sr = 1;

        CSL_MEMPROT_setL1DAttributes (index, &mppa);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_setL1DAttributes (
	Uint32 index,
	CSL_MEMPROT_MPPA * mppa
)
{
	/* Configure the L1DMPPA register specified by the index. */
	hCgemRegs->L1DMPPA[index]   =   CSL_FMK (CGEM_L1DMPPA_AID5, mppa->aid5) |
	                                CSL_FMK (CGEM_L1DMPPA_AID4, mppa->aid4) |
	                                CSL_FMK (CGEM_L1DMPPA_AID3, mppa->aid3) |
	                                CSL_FMK (CGEM_L1DMPPA_AID2, mppa->aid2) |
	                                CSL_FMK (CGEM_L1DMPPA_AID1, mppa->aid1) |
	                                CSL_FMK (CGEM_L1DMPPA_AID0, mppa->aid0) |	
	                                CSL_FMK (CGEM_L1DMPPA_AIDX, mppa->aidx) |
	                                CSL_FMK (CGEM_L1DMPPA_LOCAL, mppa->local) |
	                                CSL_FMK (CGEM_L1DMPPA_UW, mppa->uw) |
	                                CSL_FMK (CGEM_L1DMPPA_UR, mppa->ur) |
	                                CSL_FMK (CGEM_L1DMPPA_SW, mppa->sw) |
	                                CSL_FMK (CGEM_L1DMPPA_SR, mppa->sr);

	return;
}


/** ============================================================================
 *   @n@b CSL_MEMPROT_getL1DAttributes
 *
 *   @b Description
 *   @n This function gets the L1D attributes from the contents of L1DMPPA register
 * 		corresponding to the index specified.
 *
 *   @b Arguments
     @verbatim
        index       Index into the set of 32 L1DMPPA registers
        mpaxl       CSL_MEMPROT_MPPA structure that needs to be populated with
                    L1DMPPA register contents.
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
 *   @n CGEM_L1DMPPA_AID5, 
 *      CGEM_L1DMPPA_AID4, 
 *      CGEM_L1DMPPA_AID3,
 *      CGEM_L1DMPPA_AID2, 
 *      CGEM_L1DMPPA_AID1, 
 *      CGEM_L1DMPPA_AID0,
 *      CGEM_L1DMPPA_AIDX, 
 *      CGEM_L1DMPPA_LOCAL, 
 *      CGEM_L1DMPPA_UW, 
 *      CGEM_L1DMPPA_UR, 
 *      CGEM_L1DMPPA_SW, 
 *      CGEM_L1DMPPA_SR
 *
 *   @b Example
 *   @verbatim
        Uint32 index = 0;
        CSL_MEMPROT_MPPA mppa;

        CSL_MEMPROT_getL1DAttributes (index, &mppa);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_MEMPROT_getL1DAttributes (
	Uint32 index,
	CSL_MEMPROT_MPPA * mppa
)
{
    Uint32  tmp;

	/* Retrieve contents of L1DMPPA register using the index specified */	
    tmp =   hCgemRegs->L1DMPPA[index];
	mppa->aid5 = CSL_FEXT (tmp, CGEM_L1DMPPA_AID5);
	mppa->aid4 = CSL_FEXT (tmp, CGEM_L1DMPPA_AID4);
	mppa->aid3 = CSL_FEXT (tmp, CGEM_L1DMPPA_AID3);
	mppa->aid2 = CSL_FEXT (tmp, CGEM_L1DMPPA_AID2);
	mppa->aid1 = CSL_FEXT (tmp, CGEM_L1DMPPA_AID1);
	mppa->aid0 = CSL_FEXT (tmp, CGEM_L1DMPPA_AID0);
	mppa->aidx = CSL_FEXT (tmp, CGEM_L1DMPPA_AIDX);
	mppa->local = CSL_FEXT (tmp, CGEM_L1DMPPA_LOCAL);						
	mppa->sr = CSL_FEXT (tmp, CGEM_L1DMPPA_SR);
	mppa->sw = CSL_FEXT (tmp, CGEM_L1DMPPA_SW);
	mppa->ur = CSL_FEXT (tmp, CGEM_L1DMPPA_UR);
	mppa->uw = CSL_FEXT (tmp, CGEM_L1DMPPA_UW);

	return;
}

#ifdef __cplusplus
}
#endif

#endif
/**
@}
*/

