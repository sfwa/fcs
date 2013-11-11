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
 * @file csl_bwmngmtAux.h
 *
 * @brief 
 *  API Auxilary header file for BWMNGMT CSL. It gives the definitions of the 
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
#ifndef CSL_BWMNGMTAUX_H_
#define CSL_BWMNGMTAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_bwmngmt.h>

/** @addtogroup CSL_BWMNGMT_FUNCTION
 @{ */
 
/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL1DCPUArb
 *
 *   @b Description
 *   @n This function sets the contents of CPUARBD, the CPU Arbitration control
 * 		register for L1D memory block.
 *
 *   @b Arguments
     @verbatim
          cpuArbSetup	CSL_BWMNGMT_CPUARB_SETUP structure that needs to be used to 
                        configure the L1D CPU arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 CPUARBD register configured with the values passed in the 
 * 		 CSL_BWMNGMT_CPUARB_SETUP structure.
 *
 *   @b Writes
 * 	 @n	CGEM_CPUARBD_PRI, 
 * 		CGEM_CPUARBD_MAXWAIT
 *   @n  \n The following registers and fields are programmed by this API \n
 *   	 CPU Arbitration Parameters 
 *       -   PRI field set in L1D \n
 *       -   MAXWAIT field set in L1D \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Sets CPU Priority to 1, CPU Maxwait to 8 for L1D block.

        CSL_BWMNGMT_CPUARB_SETUP 	cpuArbSetup;
        
        cpuArbSetup.priority 	= CSL_BWMNGMT_PRI_PRI1;
        cpuArbSetup.maxWait  	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL1DCPUArb (&cpuArbSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL1DCPUArb (
	CSL_BWMNGMT_CPUARB_SETUP * cpuArbSetup
)
{
	/* Configure the CPU Arbitration Priority and Max Wait values 
	 * for the L1D memory block.
	 */
	hCgem->CPUARBD  =   CSL_FMK(CGEM_CPUARBD_PRI, cpuArbSetup->priority) | 
	                    CSL_FMK(CGEM_CPUARBD_MAXWAIT, cpuArbSetup->maxWait);			

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL1DCPUArb
 *
 *   @b Description
 *   @n This function retrieves the contents of CPUARBD, the CPU Arbitration control
 * 		register for L1D.
 *
 *   @b Arguments
     @verbatim
          cpuArbSetup	CSL_BWMNGMT_CPUARB_SETUP structure that needs to be filled 
                        with contents of L1D CPU arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_CPUARBD_PRI, 
 * 		CGEM_CPUARBD_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        CSL_BWMNGMT_CPUARB_SETUP 	cpuArbSetup;
        
        CSL_BWMNGMT_getL1DCPUArb (&cpuArbSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_getL1DCPUArb (
	CSL_BWMNGMT_CPUARB_SETUP * cpuArbSetup
)
{
	/* Retrieve the CPU Arbitration Priority and Max Wait values 
	 * for L1D.
	 */
	cpuArbSetup->priority = CSL_FEXT(hCgem->CPUARBD, CGEM_CPUARBD_PRI);
	cpuArbSetup->maxWait = CSL_FEXT(hCgem->CPUARBD, CGEM_CPUARBD_MAXWAIT);			

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL2CPUArb
 *
 *   @b Description
 *   @n This function sets the contents of CPUARBU, the CPU Arbitration control
 * 		register for L2 memory.
 *
 *   @b Arguments
     @verbatim
          cpuArbSetup	CSL_BWMNGMT_CPUARB_SETUP structure that needs to be used to 
                        configure the L2 CPU arbitration control registers
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 CPUARBU register configured with the values passed in the 
 * 		 CSL_BWMNGMT_CPUARB_SETUP structure.
 *
 *   @b Writes
 * 	 @n	CGEM_CPUARBU_PRI, 
 * 		CGEM_CPUARBU_MAXWAIT
 *   @n  \n The following registers and fields are programmed by this API \n
 *   	 CPU Arbitration Parameters 
 *       -   PRI field set in L2 \n
 *       -   MAXWAIT field set in L2 \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Sets CPU Priority to 1, CPU Maxwait to 8 for L2.

        CSL_BWMNGMT_CPUARB_SETUP 	cpuArbSetup;
        
        cpuArbSetup.priority 	= CSL_BWMNGMT_PRI_PRI1;
        cpuArbSetup.maxWait  	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL2CPUArb (&cpuArbSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL2CPUArb (
	CSL_BWMNGMT_CPUARB_SETUP * cpuArbSetup
)
{
	/* Configure the CPU Arbitration Priority and Max Wait values 
	 * for the L2.
	 */
    hCgem->CPUARBU  =   CSL_FMK(CGEM_CPUARBU_PRI, cpuArbSetup->priority) |
	                    CSL_FMK(CGEM_CPUARBU_MAXWAIT, cpuArbSetup->maxWait);  		

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL2CPUArb
 *
 *   @b Description
 *   @n This function retrieves the contents of CPUARBU, the CPU Arbitration control
 * 		register for L2.
 *
 *   @b Arguments
     @verbatim
          cpuArbSetup	CSL_BWMNGMT_CPUARB_SETUP structure that needs to be filled 
                        with contents of L2 CPU arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_CPUARBU_PRI, 
 * 		CGEM_CPUARBU_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        CSL_BWMNGMT_CPUARB_SETUP 	cpuArbSetup;
        
        CSL_BWMNGMT_getL2CPUArb (&cpuArbSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_getL2CPUArb (
	CSL_BWMNGMT_CPUARB_SETUP * cpuArbSetup
)
{
	/* Retrieve the CPU Arbitration Priority and Max Wait values 
	 * for L2.
	 */
	cpuArbSetup->priority = CSL_FEXT(hCgem->CPUARBU, CGEM_CPUARBU_PRI);
	cpuArbSetup->maxWait = CSL_FEXT(hCgem->CPUARBU, CGEM_CPUARBU_MAXWAIT);			

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setExternalCPUArb
 *
 *   @b Description
 *   @n This function sets the contents of CPUARBE, the CPU Arbitration control
 * 		register for EXT/EMC memory block.
 *
 *   @b Arguments
     @verbatim
          cpuArbSetup	CSL_BWMNGMT_CPUARB_SETUP structure that needs to be used to 
                        configure the CPU arbitration control registers
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 CPUARBE register configured with the values passed in the 
 * 		 CSL_BWMNGMT_CPUARB_SETUP structure.
 *
 *   @b Writes
 * 	 @n	CGEM_CPUARBE_PRI, 
 * 		CGEM_CPUARBE_MAXWAIT
 *   @n  \n The following registers and fields are programmed by this API \n
 *   	 CPU Arbitration Parameters
 *       -   PRI field set in EXT/EMC \n
 *       -   MAXWAIT field set in EXT/EMC \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Sets CPU Priority to 1, CPU Maxwait to 8 for EMC.

        CSL_BWMNGMT_CPUARB_SETUP 	cpuArbSetup;
        
        cpuArbSetup.priority 	= CSL_BWMNGMT_PRI_PRI1;
        cpuArbSetup.maxWait  	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setExternalCPUArb (&cpuArbSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setExternalCPUArb (
	CSL_BWMNGMT_CPUARB_SETUP * cpuArbSetup
)
{
	/* Configure the CPU Arbitration Priority and Max Wait values 
	 * for the external (EMC) memory block.
	 */
    hCgem->CPUARBE = CSL_FMK(CGEM_CPUARBE_PRI, cpuArbSetup->priority) |
	                 CSL_FMK(CGEM_CPUARBE_MAXWAIT, cpuArbSetup->maxWait);  	

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getExternalCPUArb
 *
 *   @b Description
 *   @n This function retrieves the contents of CPUARBE, the CPU Arbitration control
 * 		register for external memory/EMC.
 *
 *   @b Arguments
     @verbatim
          cpuArbSetup	CSL_BWMNGMT_CPUARB_SETUP structure that needs to be filled
                        with contents of EXT CPU arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_CPUARBE_PRI, 
 * 		CGEM_CPUARBE_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        CSL_BWMNGMT_CPUARB_SETUP 	cpuArbSetup;
        
        CSL_BWMNGMT_getExternalCPUArb (&cpuArbSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_getExternalCPUArb (
	CSL_BWMNGMT_CPUARB_SETUP * cpuArbSetup
)
{
	/* Retrieve the CPU Arbitration Priority and Max Wait values for EMC. */
	cpuArbSetup->priority = CSL_FEXT(hCgem->CPUARBE, CGEM_CPUARBE_PRI);
	cpuArbSetup->maxWait = CSL_FEXT(hCgem->CPUARBE, CGEM_CPUARBE_MAXWAIT);			

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL1DUserCoherenceArb
 *
 *   @b Description
 *   @n This function sets the contents of UCARBD register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        the L1D User Coherence arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 UCARBD register configured
 *
 *   @b Writes
 * 	 @n	CGEM_UCARBD_MAXWAIT
 *   @n  \n The following registers and fields are programmed by this API \n
 *     	 UC Arbitration Parameter 
 *       -   MAXWAIT field set in L1D \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set UC Maxwait to 8 for L1D.

        Uint32		maxwaitVal;

        maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL1DUserCoherenceArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL1DUserCoherenceArb (
	Uint32					maxwaitVal
)
{
	/* Configure the L1D User Coherence Arbitration Max Wait value */
	CSL_FINS(hCgem->UCARBD, CGEM_UCARBD_MAXWAIT, maxwaitVal);		

	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL1DUserCoherenceArb
 *
 *   @b Description
 *   @n This function retrieves the contents of UCARBD, the User Coherence Arbitration 
 * 		control register for L1D.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_UCARBD_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getL1DUserCoherenceArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getL1DUserCoherenceArb (
	void
)
{
	/* Retrieve the User Coherence Arbitration Max Wait value for L1D. */
	return CSL_FEXT(hCgem->UCARBD, CGEM_UCARBD_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL2UserCoherenceArb
 *
 *   @b Description
 *   @n This function sets the contents of UCARBU register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        the L2 User Coherence arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 UCARBU register configured
 *
 *   @b Writes
 * 	 @n	CGEM_UCARBU_MAXWAIT
 *   @n \n The following registers and fields are programmed by this API \n
 *     	 UC Arbitration Parameter
 *       -   MAXWAIT field set in L2 \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set UC Maxwait to 8 for L2 block only.

        Uint32		maxwaitVal;

        maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL2UserCoherenceArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL2UserCoherenceArb (
	Uint32					maxwaitVal
)
{
	/* Configure the User Coherence Arbitration Max Wait value for L2. */
	CSL_FINS(hCgem->UCARBU, CGEM_UCARBU_MAXWAIT, maxwaitVal);       	

	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL2UserCoherenceArb
 *
 *   @b Description
 *   @n This function retrieves the contents of UCARBU, the User Coherence Arbitration 
 * 		control register for L2.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_UCARBU_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getL2UserCoherenceArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getL2UserCoherenceArb (
	void
)
{
	/* Retrieve the User Coherence Arbitration Max Wait value for L2. */
	return CSL_FEXT(hCgem->UCARBU, CGEM_UCARBU_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL1DIDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of IDMAARBD register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        the L1D Internal DMA (IDMA) arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 IDMAARBD
 *
 *   @b Writes
 * 	 @n	CGEM_IDMAARBD_MAXWAIT
 *   @n \n The following registers and fields are programmed by this API \n
 *   	 IDMA Arbitration Parameter
 *       -   MAXWAIT field set in L1D \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set IDMA Maxwait to 8 for L1D block only.

        Uint32		maxwaitVal;

        maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL1DIDMAArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL1DIDMAArb (
	Uint32					maxwaitVal
)
{
	/* Configure the L1D IDMA Arbitration Max Wait value */
	CSL_FINS(hCgem->IDMAARBD, CGEM_IDMAARBD_MAXWAIT, maxwaitVal);

	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL1DIDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of IDMAARBD, the L1D Internal Memory 
 * 		Arbitration control register.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_IDMAARBD_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getL1DIDMAArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getL1DIDMAArb (
	void
)
{
	/* Retrieve the IDMAARBD Max Wait value. */
	return CSL_FEXT(hCgem->IDMAARBD, CGEM_IDMAARBD_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL2IDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of IDMAARBU register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        the L2 Internal DMA (IDMA) arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 IDMAARBU register modified
 *
 *   @b Writes
 * 	 @n	CGEM_IDMAARBU_MAXWAIT
 *   @n \n The following registers and fields are programmed by this API \n
 *   	 IDMA Arbitration Parameter
 *       -   MAXWAIT field set in L2 \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set IDMA Maxwait to 8 for L2 block only.

        Uint32		maxwaitVal;

        maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL2IDMAArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL2IDMAArb (
	Uint32					maxwaitVal
)
{
	/* Configure the L2 IDMA Arbitration Max Wait value */
	CSL_FINS(hCgem->IDMAARBU, CGEM_IDMAARBU_MAXWAIT, maxwaitVal);

	return;
}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL2IDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of IDMAARBU, the L2 Internal Memory 
 * 		Arbitration control register.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_IDMAARBU_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getL2IDMAArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getL2IDMAArb(
	void
)
{
	/* Retrieve the IDMAARBU Max Wait value. */
	return CSL_FEXT(hCgem->IDMAARBU, CGEM_IDMAARBU_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setExternalIDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of IDMAARBE register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to the External 
                        Memory(EMC)'s Internal DMA (IDMA) arbitration control registers.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 IDMAARBE register configured
 *
 *   @b Writes
 * 	 @n	CGEM_IDMAARBE_MAXWAIT
 *   @n \n The following registers and fields are programmed by this API \n
 *   	 IDMA Arbitration Parameter
 *       -   MAXWAIT field set in EMC \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set IDMA Maxwait to 8 for EXT/EMC block only.

        Uint32		maxwaitVal;

        maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setExternalIDMAArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setExternalIDMAArb (
	Uint32					maxwaitVal
)
{
	/* Configure the EMC IDMA Arbitration Max Wait value */
	CSL_FINS(hCgem->IDMAARBE, CGEM_IDMAARBE_MAXWAIT, maxwaitVal);

	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getExternalIDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of IDMAARBE, the EMC Internal Memory 
 * 		Arbitration control register.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_IDMAARBE_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getExternalIDMAArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getExternalIDMAArb (
	void
)
{
	/* Retrieve the IDMAARBE Max Wait value. */
	return CSL_FEXT(hCgem->IDMAARBE, CGEM_IDMAARBE_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL1DSDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of SDMAARBD register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        the L1D Slave DMA (SDMA) arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 SDMAARBD register configured
 *
 *   @b Writes
 * 	 @n	CGEM_SDMAARBD_MAXWAIT
 *   @n \n The following register and fields are programmed by this API \n
 *   	 SDMA Arbitration Parameter 
 *       -   MAXWAIT field set in L1D \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set SDMA Maxwait to 8 for L1D block.

        Uint32		maxwaitVal;

       	maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL1DSDMAArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL1DSDMAArb (
	Uint32					maxwaitVal
)
{
	/* Configure the L1D Slave DMA(SDMA) Arbitration Max Wait value */
	CSL_FINS(hCgem->SDMAARBD, CGEM_SDMAARBD_MAXWAIT, maxwaitVal);

	return;
} 

/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL1DSDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of SDMAARBD, L1D's Slave DMA (SDMA)
 * 		Memory Arbitration control register.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_SDMAARBD_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getL1DSDMAArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getL1DSDMAArb (
	void
)
{
	/* Retrieve the SDMAARBD Max Wait value. */
	return CSL_FEXT(hCgem->SDMAARBD, CGEM_SDMAARBD_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL2SDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of SDMAARBU register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        the L2 Slave DMA (SDMA) arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 SDMAARBU register configured
 *
 *   @b Writes
 * 	 @n	CGEM_SDMAARBU_MAXWAIT
 *   @n \n The following register and fields are programmed by this API \n
 *   	 SDMA Arbitration Parameter 
 *       -   MAXWAIT field set in L2 \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set SDMA Maxwait to 8 for L2.

        Uint32		maxwaitVal;

       	maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setL2SDMAArb(maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL2SDMAArb (
	Uint32					maxwaitVal
)
{
	/* Configure the L2 Slave DMA(SDMA) Arbitration Max Wait value */
	CSL_FINS(hCgem->SDMAARBU, CGEM_SDMAARBU_MAXWAIT, maxwaitVal);

	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL2SDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of SDMAARBD, L2's Slave DMA (SDMA)
 * 		Memory Arbitration control register.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_SDMAARBU_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getL2SDMAArb();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getL2SDMAArb (
	void
)
{
	/* Retrieve the SDMAARBU Max Wait value. */
	return CSL_FEXT(hCgem->SDMAARBU, CGEM_SDMAARBU_MAXWAIT);			

}


/** ============================================================================
 *   @n@b CSL_BWMNGMT_setExternalSDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of SDMAARBE register.
 *
 *   @b Arguments
     @verbatim
          maxwaitVal	Maximum Wait Value that needs to be set to 
                        External Memory/EMC's Slave DMA (SDMA) arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 SDMAARBE register configured
 *
 *   @b Writes
 * 	 @n	CGEM_SDMAARBE_MAXWAIT
 *   @n \n The following register and fields are programmed by this API \n
 *   	 SDMA Arbitration Parameter 
 *       -   MAXWAIT field set in EMC \n
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set SDMA Maxwait to 8 for EMC.

        Uint32		maxwaitVal;

       	maxwaitVal	= CSL_BWMNGMT_MAXWAIT_MAXWAIT8;
        
        CSL_BWMNGMT_setExternalSDMAArb (maxwaitVal);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setExternalSDMAArb (
	Uint32					maxwaitVal
)
{
	/* Configure the EMC Slave DMA(SDMA) Arbitration Max Wait value */
	CSL_FINS(hCgem->SDMAARBE, CGEM_SDMAARBE_MAXWAIT, maxwaitVal);

	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getExternalSDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of SDMAARBE, EMC's Slave DMA (SDMA)
 * 		Memory Arbitration control register.
 *
 *   @b Arguments
 *	 @n	None
 * 
 *   <b> Return Value </b>
 *	 @n	 Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None.
 *
 *   @b Reads
 * 	 @n	CGEM_SDMAARBE_MAXWAIT
 *
 *   @b Example
 *   @verbatim
        Uint32		maxwaitVal;
        
        maxwaitVal = CSL_BWMNGMT_getExternalSDMAArb ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BWMNGMT_getExternalSDMAArb (
	void
)
{
	/* Retrieve the SDMAARBE Max Wait value. */
	return CSL_FEXT(hCgem->SDMAARBE, CGEM_SDMAARBE_MAXWAIT);			

}

/** ============================================================================
 *   @n@b CSL_BWMNGMT_setL2MDMAArb
 *
 *   @b Description
 *   @n This function sets the contents of MDMAARBU register with the priority
 * 		and urgent priority values passed to this API.
 *
 *   @b Arguments
     @verbatim
          mdmaPriSetup	CSL_BWMNGMT_MDMAPRI_SETUP that contains the priority values 
                        that need to be set to the L2/UMC Master DMA (MDMA) arbitration 
                        control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 MDMAARBU register configured
 *
 *   @b Writes
 * 	 @n	CGEM_MDMAARBU_PRI, 
 * 		CGEM_MDMAARBU_UPRI
 *
 *   @b Example
 *   @verbatim
    	Example 1: Set MDMA Priority and Effecitve Priority to 6.

        CSL_BWMNGMT_MDMAPRI_SETUP	mdmaPriSetup;
        
        mdmaPriSetup.priority	= CSL_BWMNGMT_PRI_PRI6;
        mdmaPriSetup.uPriority	= CSL_BWMNGMT_PRI_PRI6;        
        
        CSL_BWMNGMT_setL2MDMAArb(&mdmaPriSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_setL2MDMAArb (
	CSL_BWMNGMT_MDMAPRI_SETUP *	mdmaPriSetup
)
{
	/* Configure the Master DMA(MDMA) Priority and Urgent Priority values */
	hCgem->MDMAARBU =   CSL_FMK(CGEM_MDMAARBU_PRI,  mdmaPriSetup->priority) |
	                    CSL_FMK(CGEM_MDMAARBU_UPRI, mdmaPriSetup->uPriority);
	return;
} 


/** ============================================================================
 *   @n@b CSL_BWMNGMT_getL2MDMAArb
 *
 *   @b Description
 *   @n This function retrieves the contents of MDMAARBU register.
 *
 *   @b Arguments
     @verbatim
          mdmaPriSetup	CSL_BWMNGMT_MDMAPRI_SETUP that will be filled with contents of 
                        the Master DMA (MDMA) arbitration control register.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *	 @n	 None
 *
 *   @b Reads
 * 	 @n	CGEM_MDMAARBU_PRI, 
 * 		CGEM_MDMAARBU_UPRI
 *
 *   @b Example
 *   @verbatim
    	Example 1: Retrieve MDMA priority values.

        CSL_BWMNGMT_MDMAPRI_SETUP	mdmaPriSetup;
        
        CSL_BWMNGMT_getL2MDMAArb (&mdmaPriSetup);

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BWMNGMT_getL2MDMAArb (
	CSL_BWMNGMT_MDMAPRI_SETUP *	mdmaPriSetup
)
{
	/* Retrieve the Master DMA(MDMA) Priority and Urgent Priority values. */
	mdmaPriSetup->priority = CSL_FEXT(hCgem->MDMAARBU, CGEM_MDMAARBU_PRI);
	mdmaPriSetup->uPriority = CSL_FEXT(hCgem->MDMAARBU, CGEM_MDMAARBU_UPRI);
	return;
} 

 /**
@}*/

#endif /*CSL_BWMNGMTAUX_H_*/
