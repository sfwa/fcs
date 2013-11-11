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
 * @file csl_ipcAux.h
 *
 * @brief 
 *  API Auxilary header file for IPC CSL. It gives the definitions of the 
 *  query & control functions.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par 
 */

#ifndef CSL_IPCAUX_H_
#define CSL_IPCAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_ipc.h>

/** @addtogroup CSL_IPC_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_IPC_genNMIEvent
 *
 *   @b Description
 *   @n This function sets the NMIG bit of the NMI Generation Register (NMIGRx) 
 * 		to create an NMI pulse to the GEM corresponding to the index
 * 		specified here. 
 *
 *   @b Arguments
     @verbatim
        index       GEM number for which the NMI event is to be raised.
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None. 
 *
 *   <b> Post Condition </b>
 *	 @n	NMIG bit in the corresponding NMIGRx register configured.
 *
 *   @b Writes
 * 	 @n	IPC_NMIGR_NMIG=1 
 *
 *   @b Example
 *   @verbatim
        Example 1: Raise an NMI interrupt to Gem 2
        Uint32 index = 2;

        CSL_IPC_genNMIEvent (index);
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IPC_genNMIEvent (
	Uint32 index
)
{
	CSL_FINS (hIpc->NMIGR[index], IPC_NMIGR_NMIG, 1);

	return;
}

 
/** ============================================================================
 *   @n@b CSL_IPC_genGEMInterrupt
 *
 *   @b Description
 *   @n This function sets the IPCG bit of the IPC Generation Register (IPCGRx) 
 * 		to create an inter-DSP pulse to the	GEM corresponding to the index
 * 		specified here. This API also configures the source ID for this 
 * 		interrupt by setting the SRCSx bit of the IPCGRx register based on
 * 		the source ID specified.
 *
 *   @b Arguments
     @verbatim
        index       GEM number for which the interrupt is to be raised.
        srcId       Indicates which of the 0-27 SRCSx bits needs to be set 
                    in the IPCGRx registers corresponding to the index specified. 
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *	 @n	IPCG and SRCSx/SRCCx bits in the corresponding IPCGRx/IPCARx register 
 * 		configured.
 *
 *   @b Writes
 * 	 @n	IPC_IPCGR_IPCG=1, 
 * 	 	IPC_IPCGR_SRCS0=1;
 * 		IPC_IPCGR_SRCS1=1;
 * 		IPC_IPCGR_SRCS2=1;
 * 		IPC_IPCGR_SRCS3=1;
 * 		IPC_IPCGR_SRCS4=1;
 * 		IPC_IPCGR_SRCS5=1;
 * 		IPC_IPCGR_SRCS6=1;
 * 		IPC_IPCGR_SRCS7=1;
 * 		IPC_IPCGR_SRCS8=1;
 * 		IPC_IPCGR_SRCS9=1;
 * 		IPC_IPCGR_SRCS10=1;
 * 		IPC_IPCGR_SRCS11=1;
 * 		IPC_IPCGR_SRCS12=1;
 * 		IPC_IPCGR_SRCS13=1;
 * 		IPC_IPCGR_SRCS14=1;
 * 		IPC_IPCGR_SRCS15=1;
 * 		IPC_IPCGR_SRCS16=1;
 * 		IPC_IPCGR_SRCS17=1;
 * 		IPC_IPCGR_SRCS18=1;
 * 		IPC_IPCGR_SRCS19=1;
 * 		IPC_IPCGR_SRCS20=1;
 * 		IPC_IPCGR_SRCS21=1;
 * 		IPC_IPCGR_SRCS22=1;
 * 		IPC_IPCGR_SRCS23=1;
 * 		IPC_IPCGR_SRCS24=1;
 * 		IPC_IPCGR_SRCS25=1;
 * 		IPC_IPCGR_SRCS26=1;
 * 		IPC_IPCGR_SRCS27=1;
 *
 *   @b Affects
 *   @n IPC_IPCAR_SRCC0=1;
 * 		IPC_IPCAR_SRCC1=1;
 * 		IPC_IPCAR_SRCC2=1;
 * 		IPC_IPCAR_SRCC3=1;
 * 		IPC_IPCAR_SRCC4=1;
 * 		IPC_IPCAR_SRCC5=1;
 * 		IPC_IPCAR_SRCC6=1;
 * 		IPC_IPCAR_SRCC7=1;
 * 		IPC_IPCAR_SRCC8=1;
 * 		IPC_IPCAR_SRCC9=1;
 * 		IPC_IPCAR_SRCC10=1;
 * 		IPC_IPCAR_SRCC11=1;
 * 		IPC_IPCAR_SRCC12=1;
 * 		IPC_IPCAR_SRCC13=1;
 * 		IPC_IPCAR_SRCC14=1;
 * 		IPC_IPCAR_SRCC15=1;
 * 		IPC_IPCAR_SRCC16=1;
 * 		IPC_IPCAR_SRCC17=1;
 * 		IPC_IPCAR_SRCC18=1;
 * 		IPC_IPCAR_SRCC19=1;
 * 		IPC_IPCAR_SRCC20=1;
 * 		IPC_IPCAR_SRCC21=1;
 * 		IPC_IPCAR_SRCC22=1;
 * 		IPC_IPCAR_SRCC23=1;
 * 		IPC_IPCAR_SRCC24=1;
 * 		IPC_IPCAR_SRCC25=1;
 * 		IPC_IPCAR_SRCC26=1;
 * 		IPC_IPCAR_SRCC27=1
 * 
 *   @b Example
 *   @verbatim
        Example 1: An application running on GEM 2 is trying to raise
        an interrupt to Gem 1. The source Id for Gem 2 say is 2. 
        Uint32 	index = 1;
        Uint32	srcId = 2;

        CSL_IPC_genGEMInterrupt (index, srcId);
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IPC_genGEMInterrupt (
	Uint32				index,
	Uint32				srcId
)
{
    hIpc->IPCGR[index]  =   CSL_FMK (IPC_IPCGR_IPCG, 1) | 
                            CSL_FMKR (CSL_IPC_IPCGR_SRCS0_SHIFT + srcId, CSL_IPC_IPCGR_SRCS0_SHIFT + srcId, 1);
	
	return;
}


/** ============================================================================
 *   @n@b CSL_IPC_isGEMInterruptSourceSet
 *
 *   @b Description
 *   @n This function checks if the SRCSx bit of the IPCGRx register is set. 
 * 		It returns 1 if the SRCSx bit corresponding to the srcId is set in the 
 * 		IPCGRx register	corresponding to the index specified. Otherwise it returns
 * 		0. 
 *
 *   @b Arguments
     @verbatim
        index       GEM number for which the IPCGRx register needs to be checked.
        srcId       Indicates which of the 0-27 SRCSx bits needs to be read 
                    in the IPCGRx registers corresponding to the index specified. 
	 @endverbatim
 *
 *   <b> Return Value </b>
 *   @n 1   -   Indicates that the corresponding SRCSx bit is set and the 
 *              srcId specified is in fact the source for the IPC Gem interrupt. \n
 *      0   -   Indicates that corresponding SRCSx bit not set and the 
 *              srcId specified is not the IPC source.
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 *   @n IPC_IPCGR_SRCS0;
 *      IPC_IPCGR_SRCS1;
 *      IPC_IPCGR_SRCS2;
 *      IPC_IPCGR_SRCS3;
 *      IPC_IPCGR_SRCS4;
 *      IPC_IPCGR_SRCS5;
 *      IPC_IPCGR_SRCS6;
 *      IPC_IPCGR_SRCS7;
 *      IPC_IPCGR_SRCS8;
 *      IPC_IPCGR_SRCS9;
 *      IPC_IPCGR_SRCS10;
 *      IPC_IPCGR_SRCS11;
 *      IPC_IPCGR_SRCS12;
 *      IPC_IPCGR_SRCS13;
 *      IPC_IPCGR_SRCS14;
 *      IPC_IPCGR_SRCS15;
 *      IPC_IPCGR_SRCS16;
 *      IPC_IPCGR_SRCS17;
 *      IPC_IPCGR_SRCS18;
 *      IPC_IPCGR_SRCS19;
 *      IPC_IPCGR_SRCS20;
 *      IPC_IPCGR_SRCS21;
 *      IPC_IPCGR_SRCS22;
 *      IPC_IPCGR_SRCS23;
 *      IPC_IPCGR_SRCS24;
 *      IPC_IPCGR_SRCS25;
 *      IPC_IPCGR_SRCS26;
 *      IPC_IPCGR_SRCS27
 * 
 *   @b Example
 *   @verbatim
        Example 1: An application running on Gem 1 received an interrupt and
        wants to check if the interrupt was from the core it was waiting on, 
        i.e., the Gem 2.
   
        Uint32 	index = 1;
        Uint32	srcId = 2;
        Uint32	retVal;

        retVal = CSL_IPC_isGEMInterruptSourceSet (index, srcId);
        
        if (retVal == 0)
        {
        	...		// Maybe cotinue waiting for the IPC/message
       	}
       	else
       	{
       		...		// Do the needful processing.
       	}
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_IPC_isGEMInterruptSourceSet (
	Uint32				index,
	Uint32				srcId
)
{
	return CSL_FEXTR (hIpc->IPCGR[index], CSL_IPC_IPCGR_SRCS0_SHIFT + srcId, CSL_IPC_IPCGR_SRCS0_SHIFT + srcId);

}

/** ============================================================================
 *   @n@b CSL_IPC_isGEMInterruptAckSet
 *
 *   @b Description
 *   @n This function checks if the SRCCx bit of the IPCARx register is set. 
 *      It returns 1 if the SRCCx bit corresponding to the srcId is set in the 
 *      IPCARx register corresponding to the index specified. Otherwise it returns
 *      0. 
 *
 *   @b Arguments
     @verbatim
        index       GEM number for which the IPCARx register needs to be checked.
        srcId       Indicates which of the 0-27 SRCCx bits needs to be read 
                    in the IPCARx registers corresponding to the index specified. 
     @endverbatim
 *
 *   <b> Return Value </b>
 *  @n  1   -       Indicates that the corresponding SRCCx bit is set and the 
 *                  srcId specified is in fact the source for the IPC Gem interrupt. \n
 *      0   -       Indicates that corresponding SRCCx bit not set and the 
 *                  srcId specified is not the IPC source.
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n IPC_IPCAR_SRCC0;
 *      IPC_IPCAR_SRCC1;
 *      IPC_IPCAR_SRCC2;
 *      IPC_IPCAR_SRCC3;
 *      IPC_IPCAR_SRCC4;
 *      IPC_IPCAR_SRCC5;
 *      IPC_IPCAR_SRCC6;
 *      IPC_IPCAR_SRCC7;
 *      IPC_IPCAR_SRCC8;
 *      IPC_IPCAR_SRCC9;
 *      IPC_IPCAR_SRCC10;
 *      IPC_IPCAR_SRCC11;
 *      IPC_IPCAR_SRCC12;
 *      IPC_IPCAR_SRCC13;
 *      IPC_IPCAR_SRCC14;
 *      IPC_IPCAR_SRCC15;
 *      IPC_IPCAR_SRCC16;
 *      IPC_IPCAR_SRCC17;
 *      IPC_IPCAR_SRCC18;
 *      IPC_IPCAR_SRCC19;
 *      IPC_IPCAR_SRCC20;
 *      IPC_IPCAR_SRCC21;
 *      IPC_IPCAR_SRCC22;
 *      IPC_IPCAR_SRCC23;
 *      IPC_IPCAR_SRCC24;
 *      IPC_IPCAR_SRCC25;
 *      IPC_IPCAR_SRCC26;
 *      IPC_IPCAR_SRCC27
 * 
 *   @b Example
 *   @verbatim
        Uint32  index = 1;
        Uint32  srcId = 2;
        Uint32  retVal;

        retVal = CSL_IPC_isGEMInterruptAckSet (index, srcId);
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_IPC_isGEMInterruptAckSet (
    Uint32              index,
    Uint32              srcId
)
{
    return CSL_FEXTR (hIpc->IPCAR[index], CSL_IPC_IPCAR_SRCC0_SHIFT + srcId, CSL_IPC_IPCAR_SRCC0_SHIFT + srcId);

}

/** ============================================================================
 *   @n@b CSL_IPC_clearGEMInterruptSource
 *
 *   @b Description
 *   @n This function clears the interrupt source IDs by setting the SRCCx bit of 
 * 		the IPC	Acknowledgment Register (IPCARx) and SRCSx bit of IPC Generation 
 * 		Register (IPCGRx) corresponding to the GEM index and Source ID specified.  
 *
 *   @b Arguments
     @verbatim
        index       GEM number for which the interrupt surce is to be cleared.
        srcId       Indicates which of the 0-27 SRCSx/SRCCx bits needs to be cleared
                    in the IPCGRx/IPCARx registers corresponding to the index specified. 
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *	 @n	SRCSx/SRCCx bits in the corresponding IPCGRx/IPCARx register are cleared.
 *
 *   @b Writes
 *   @n IPC_IPCAR_SRCC0=0;
 *      IPC_IPCAR_SRCC1=0;
 *      IPC_IPCAR_SRCC2=0;
 *      IPC_IPCAR_SRCC3=0;
 *      IPC_IPCAR_SRCC4=0;
 *      IPC_IPCAR_SRCC5=0;
 *      IPC_IPCAR_SRCC6=0;
 *      IPC_IPCAR_SRCC7=0;
 *      IPC_IPCAR_SRCC8=0;
 *      IPC_IPCAR_SRCC9=0;
 *      IPC_IPCAR_SRCC10=0;
 *      IPC_IPCAR_SRCC11=0;
 *      IPC_IPCAR_SRCC12=0;
 *      IPC_IPCAR_SRCC13=0;
 *      IPC_IPCAR_SRCC14=0;
 *      IPC_IPCAR_SRCC15=0;
 *      IPC_IPCAR_SRCC16=0;
 *      IPC_IPCAR_SRCC17=0;
 *      IPC_IPCAR_SRCC18=0;
 *      IPC_IPCAR_SRCC19=0;
 *      IPC_IPCAR_SRCC20=0;
 *      IPC_IPCAR_SRCC21=0;
 *      IPC_IPCAR_SRCC22=0;
 *      IPC_IPCAR_SRCC23=0;
 *      IPC_IPCAR_SRCC24=0;
 *      IPC_IPCAR_SRCC25=0;
 *      IPC_IPCAR_SRCC26=0;
 *      IPC_IPCAR_SRCC27=0
 *
 *   @b Affects
 *   @n IPC_IPCGR_SRCS0=0,
 *      IPC_IPCGR_SRCS1=0;
 *      IPC_IPCGR_SRCS2=0;
 *      IPC_IPCGR_SRCS3=0;
 *      IPC_IPCGR_SRCS4=0;
 *      IPC_IPCGR_SRCS5=0;
 *      IPC_IPCGR_SRCS6=0;
 *      IPC_IPCGR_SRCS7=0;
 *      IPC_IPCGR_SRCS8=0;
 *      IPC_IPCGR_SRCS9=0;
 *      IPC_IPCGR_SRCS10=0;
 *      IPC_IPCGR_SRCS11=0;
 *      IPC_IPCGR_SRCS12=0;
 *      IPC_IPCGR_SRCS13=0;
 *      IPC_IPCGR_SRCS14=0;
 *      IPC_IPCGR_SRCS15=0;
 *      IPC_IPCGR_SRCS16=0;
 *      IPC_IPCGR_SRCS17=0;
 *      IPC_IPCGR_SRCS18=0;
 *      IPC_IPCGR_SRCS19=0;
 *      IPC_IPCGR_SRCS20=0;
 *      IPC_IPCGR_SRCS21=0;
 *      IPC_IPCGR_SRCS22=0;
 *      IPC_IPCGR_SRCS23=0;
 *      IPC_IPCGR_SRCS24=0;
 *      IPC_IPCGR_SRCS25=0;
 *      IPC_IPCGR_SRCS26=0;
 *      IPC_IPCGR_SRCS27=0
 * 
 *   @b Example
 *   @verbatim
        Example 1: Clear the interrupt for Gem 1 raised by Gem 2. 
        Uint32 	index = 1;
        Uint32	srcId = 2;

        CSL_IPC_clearGEMInterruptSource (index, srcId);
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IPC_clearGEMInterruptSource (
	Uint32				index,
	Uint32				srcId
)
{
    hIpc->IPCAR[index]  =   CSL_FMKR (CSL_IPC_IPCAR_SRCC0_SHIFT + srcId, CSL_IPC_IPCAR_SRCC0_SHIFT + srcId, 1);
	
	return;
}


/** ============================================================================
 *   @n@b CSL_IPC_genHostInterrupt
 *
 *   @b Description
 *   @n This function sets the IPCG bit of the Host IPC Generation Register (IPCGRH) 
 * 		to create an interrupt pulse on the device pin. This API also configures the 
 * 		source ID for this interrupt by setting the SRCSx bit of the IPCGRHx 
 * 		register based on the source ID specified.
 *
 *   @b Arguments
     @verbatim
        srcId       Indicates which of the 0-27 SRCSx bits needs to be set 
                    in the IPCGRH register. 
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *	 @n	IPCG and SRCSx/SRCCx bits in the IPCGRH/IPCARH register configured.
 *
 *   @b Writes
 * 	 @n	IPC_IPCGRH_IPCG=1, 
 * 	 	IPC_IPCGRH_SRCS0=1;
 * 		IPC_IPCGRH_SRCS1=1;
 * 		IPC_IPCGRH_SRCS2=1;
 * 		IPC_IPCGRH_SRCS3=1;
 * 		IPC_IPCGRH_SRCS4=1;
 * 		IPC_IPCGRH_SRCS5=1;
 * 		IPC_IPCGRH_SRCS6=1;
 * 		IPC_IPCGRH_SRCS7=1;
 * 		IPC_IPCGRH_SRCS8=1;
 * 		IPC_IPCGRH_SRCS9=1;
 * 		IPC_IPCGRH_SRCS10=1;
 * 		IPC_IPCGRH_SRCS11=1;
 * 		IPC_IPCGRH_SRCS12=1;
 * 		IPC_IPCGRH_SRCS13=1;
 * 		IPC_IPCGRH_SRCS14=1;
 * 		IPC_IPCGRH_SRCS15=1;
 * 		IPC_IPCGRH_SRCS16=1;
 * 		IPC_IPCGRH_SRCS17=1;
 * 		IPC_IPCGRH_SRCS18=1;
 * 		IPC_IPCGRH_SRCS19=1;
 * 		IPC_IPCGRH_SRCS20=1;
 * 		IPC_IPCGRH_SRCS21=1;
 * 		IPC_IPCGRH_SRCS22=1;
 * 		IPC_IPCGRH_SRCS23=1;
 * 		IPC_IPCGRH_SRCS24=1;
 * 		IPC_IPCGRH_SRCS25=1;
 * 		IPC_IPCGRH_SRCS26=1;
 * 		IPC_IPCGRH_SRCS27=1
 *
 *   @b Affects
 * 	 	IPC_IPCARH_SRCC0=1;
 * 		IPC_IPCARH_SRCC1=1;
 * 		IPC_IPCARH_SRCC2=1;
 * 		IPC_IPCARH_SRCC3=1;
 * 		IPC_IPCARH_SRCC4=1;
 * 		IPC_IPCARH_SRCC5=1;
 * 		IPC_IPCARH_SRCC6=1;
 * 		IPC_IPCARH_SRCC7=1;
 * 		IPC_IPCARH_SRCC8=1;
 * 		IPC_IPCARH_SRCC9=1;
 * 		IPC_IPCARH_SRCC10=1;
 * 		IPC_IPCARH_SRCC11=1;
 * 		IPC_IPCARH_SRCC12=1;
 * 		IPC_IPCARH_SRCC13=1;
 * 		IPC_IPCARH_SRCC14=1;
 * 		IPC_IPCARH_SRCC15=1;
 * 		IPC_IPCARH_SRCC16=1;
 * 		IPC_IPCARH_SRCC17=1;
 * 		IPC_IPCARH_SRCC18=1;
 * 		IPC_IPCARH_SRCC19=1;
 * 		IPC_IPCARH_SRCC20=1;
 * 		IPC_IPCARH_SRCC21=1;
 * 		IPC_IPCARH_SRCC22=1;
 * 		IPC_IPCARH_SRCC23=1;
 * 		IPC_IPCARH_SRCC24=1;
 * 		IPC_IPCARH_SRCC25=1;
 * 		IPC_IPCARH_SRCC26=1;
 * 		IPC_IPCARH_SRCC27=1
 * 
 *   @b Example
 *   @verbatim
        Example 1: An application running on GEM 2 is trying to raise
        a host interrupt. 
        Uint32	srcId = 2;

        CSL_IPC_genHostInterrupt (srcId);
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IPC_genHostInterrupt (
	Uint32				srcId
)
{
    hIpc->IPCGRH    =   CSL_FMK (IPC_IPCGRH_IPCG, 1) |
                        CSL_FMKR (CSL_IPC_IPCGRH_SRCS0_SHIFT + srcId, CSL_IPC_IPCGRH_SRCS0_SHIFT + srcId, 1);
	
	return;
}


/** ============================================================================
 *   @n@b CSL_IPC_isHostInterruptSourceSet
 *
 *   @b Description
 *   @n This function checks if the SRCSx bit of the IPCGRH register is set. 
 * 		It returns 1 if the SRCSx bit corresponding to the srcId is set in the 
 * 		IPCGRH register. Otherwise it returns 0. 
 *
 *   @b Arguments
     @verbatim
        srcId       Indicates which of the 0-27 SRCSx bits needs to be read 
                    in the IPCGRH register. 
	 @endverbatim
 *
 *   <b> Return Value </b>
 *   @n 1   -       Indicates that the corresponding SRCSx bit is set and the 
 *                  srcId specified is in fact the source for the IPC Host interrupt. \n
 *      0   -       Indicates that corresponding SRCSx bit not set and the 
 *                  srcId specified is not the IPC source.
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *	 @n	None
 *
 *   @b Reads
 * 	 @n	IPC_IPCGRH_SRCS0;
 * 		IPC_IPCGRH_SRCS1;
 * 		IPC_IPCGRH_SRCS2;
 * 		IPC_IPCGRH_SRCS3;
 * 		IPC_IPCGRH_SRCS4;
 * 		IPC_IPCGRH_SRCS5;
 * 		IPC_IPCGRH_SRCS6;
 * 		IPC_IPCGRH_SRCS7;
 * 		IPC_IPCGRH_SRCS8;
 * 		IPC_IPCGRH_SRCS9;
 * 		IPC_IPCGRH_SRCS10;
 * 		IPC_IPCGRH_SRCS11;
 * 		IPC_IPCGRH_SRCS12;
 * 		IPC_IPCGRH_SRCS13;
 * 		IPC_IPCGRH_SRCS14;
 * 		IPC_IPCGRH_SRCS15;
 * 		IPC_IPCGRH_SRCS16;
 * 		IPC_IPCGRH_SRCS17;
 * 		IPC_IPCGRH_SRCS18;
 * 		IPC_IPCGRH_SRCS19;
 * 		IPC_IPCGRH_SRCS20;
 * 		IPC_IPCGRH_SRCS21;
 * 		IPC_IPCGRH_SRCS22;
 * 		IPC_IPCGRH_SRCS23;
 * 		IPC_IPCGRH_SRCS24;
 * 		IPC_IPCGRH_SRCS25;
 * 		IPC_IPCGRH_SRCS26;
 * 		IPC_IPCGRH_SRCS27
 * 
 *   @b Example
 *   @verbatim
        Example 1: Check if the host interrupt's source was set correctly
        to 2, i.e., the Gem 2 the source of the host interrupt.
 
        Uint32	srcId = 2;
        Uint32	retVal;

        retVal = CSL_IPC_isHostInterruptSourceSet (srcId);
        
        if (retVal == 0)
        {
            ...		// Do something.
        }
        else
        {
            ...		// Do the needful processing.
        }
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_IPC_isHostInterruptSourceSet (
	Uint32				srcId
)
{
	return CSL_FEXTR (hIpc->IPCGRH, CSL_IPC_IPCGRH_SRCS0_SHIFT + srcId, CSL_IPC_IPCGRH_SRCS0_SHIFT + srcId);

}


/** ============================================================================
 *   @n@b CSL_IPC_isHostInterruptAckSet
 *
 *   @b Description
 *   @n This function checks if the SRCCx bit of the IPCARH register is set. 
 *      It returns 1 if the SRCCx bit corresponding to the srcId is set in the 
 *      IPCARH register. Otherwise it returns 0. 
 *
 *   @b Arguments
     @verbatim
        srcId       Indicates which of the 0-27 SRCCx bits needs to be read 
                    in the IPCARH register. 
     @endverbatim
 *
 *   <b> Return Value </b>
 *  @n  1   -       Indicates that the corresponding SRCCx bit is set and the 
 *                  srcId specified is in fact the source for the IPC Host interrupt. \n
 *      0   -       Indicates that corresponding SRCCx bit not set and the 
 *                  srcId specified is not the IPC source.
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n IPC_IPCARH_SRCC0;
 *      IPC_IPCARH_SRCC1;
 *      IPC_IPCARH_SRCC2;
 *      IPC_IPCARH_SRCC3;
 *      IPC_IPCARH_SRCC4;
 *      IPC_IPCARH_SRCC5;
 *      IPC_IPCARH_SRCC6;
 *      IPC_IPCARH_SRCC7;
 *      IPC_IPCARH_SRCC8;
 *      IPC_IPCARH_SRCC9;
 *      IPC_IPCARH_SRCC10;
 *      IPC_IPCARH_SRCC11;
 *      IPC_IPCARH_SRCC12;
 *      IPC_IPCARH_SRCC13;
 *      IPC_IPCARH_SRCC14;
 *      IPC_IPCARH_SRCC15;
 *      IPC_IPCARH_SRCC16;
 *      IPC_IPCARH_SRCC17;
 *      IPC_IPCARH_SRCC18;
 *      IPC_IPCARH_SRCC19;
 *      IPC_IPCARH_SRCC20;
 *      IPC_IPCARH_SRCC21;
 *      IPC_IPCARH_SRCC22;
 *      IPC_IPCARH_SRCC23;
 *      IPC_IPCARH_SRCC24;
 *      IPC_IPCARH_SRCC25;
 *      IPC_IPCARH_SRCC26;
 *      IPC_IPCARH_SRCC27
 * 
 *   @b Example
 *   @verbatim
        Uint32  srcId = 2;
        Uint32  retVal;

        retVal = CSL_IPC_isHostInterruptAckSet (srcId);
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_IPC_isHostInterruptAckSet (
    Uint32              srcId
)
{
    return CSL_FEXTR (hIpc->IPCARH, CSL_IPC_IPCARH_SRCC0_SHIFT + srcId, CSL_IPC_IPCARH_SRCC0_SHIFT + srcId);

}


/** ============================================================================
 *   @n@b CSL_IPC_clearHostInterruptSource
 *
 *   @b Description
 *   @n This function clears the interrupt source IDs by setting the SRCCx bit 
 * 		of the Host IPC Acknowledgment Register (IPCARH) and SRCSx bit of Host 
 * 		IPC Generation Register (IPCGRH) corresponding to the Source ID specified.  
 *
 *   @b Arguments
     @verbatim
        srcId       Indicates which of the 0-27 SRCSx/SRCCx bits needs to be cleared
                    in the IPCGRH/IPCARH registers. 
	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	None
 *
 *   <b> Pre Condition </b>
 *   @n None 
 *
 *   <b> Post Condition </b>
 *	 @n	SRCSx/SRCCx bits in the corresponding IPCGRH/IPCARH register are cleared.
 *
 *   @b Writes
 *   @n IPC_IPCARH_SRCC0=0;
 * 		IPC_IPCARH_SRCC1=0;
 * 		IPC_IPCARH_SRCC2=0;
 * 		IPC_IPCARH_SRCC3=0;
 * 		IPC_IPCARH_SRCC4=0;
 * 		IPC_IPCARH_SRCC5=0;
 * 		IPC_IPCARH_SRCC6=0;
 * 		IPC_IPCARH_SRCC7=0;
 * 		IPC_IPCARH_SRCC8=0;
 * 		IPC_IPCARH_SRCC9=0;
 * 		IPC_IPCARH_SRCC10=0;
 * 		IPC_IPCARH_SRCC11=0;
 * 		IPC_IPCARH_SRCC12=0;
 * 		IPC_IPCARH_SRCC13=0;
 * 		IPC_IPCARH_SRCC14=0;
 * 		IPC_IPCARH_SRCC15=0;
 * 		IPC_IPCARH_SRCC16=0;
 * 		IPC_IPCARH_SRCC17=0;
 * 		IPC_IPCARH_SRCC18=0;
 * 		IPC_IPCARH_SRCC19=0;
 * 		IPC_IPCARH_SRCC20=0;
 * 		IPC_IPCARH_SRCC21=0;
 * 		IPC_IPCARH_SRCC22=0;
 * 		IPC_IPCARH_SRCC23=0;
 * 		IPC_IPCARH_SRCC24=0;
 * 		IPC_IPCARH_SRCC25=0;
 * 		IPC_IPCARH_SRCC26=0;
 * 		IPC_IPCARH_SRCC27=0
 *
 *   @b Affects
 * 	 @n	IPC_IPCGRH_SRCS0=0;
 * 		IPC_IPCGRH_SRCS1=0;
 * 		IPC_IPCGRH_SRCS2=0;
 * 		IPC_IPCGRH_SRCS3=0;
 * 		IPC_IPCGRH_SRCS4=0;
 * 		IPC_IPCGRH_SRCS5=0;
 * 		IPC_IPCGRH_SRCS6=0;
 * 		IPC_IPCGRH_SRCS7=0;
 * 		IPC_IPCGRH_SRCS8=0;
 * 		IPC_IPCGRH_SRCS9=0;
 * 		IPC_IPCGRH_SRCS10=0;
 * 		IPC_IPCGRH_SRCS11=0;
 * 		IPC_IPCGRH_SRCS12=0;
 * 		IPC_IPCGRH_SRCS13=0;
 * 		IPC_IPCGRH_SRCS14=0;
 * 		IPC_IPCGRH_SRCS15=0;
 * 		IPC_IPCGRH_SRCS16=0;
 * 		IPC_IPCGRH_SRCS17=0;
 * 		IPC_IPCGRH_SRCS18=0;
 * 		IPC_IPCGRH_SRCS19=0;
 * 		IPC_IPCGRH_SRCS20=0;
 * 		IPC_IPCGRH_SRCS21=0;
 * 		IPC_IPCGRH_SRCS22=0;
 * 		IPC_IPCGRH_SRCS23=0;
 * 		IPC_IPCGRH_SRCS24=0;
 * 		IPC_IPCGRH_SRCS25=0;
 * 		IPC_IPCGRH_SRCS26=0;
 * 		IPC_IPCGRH_SRCS27=0;
 * 
 *   @b Example
 *   @verbatim
        Example 1: Clear the host interrupt raised by Gem 2. 
        Uint32	srcId = 2;

        CSL_IPC_clearHostInterruptSource (srcId);
	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IPC_clearHostInterruptSource (
	Uint32				srcId
)
{
    hIpc->IPCARH    =   CSL_FMKR (CSL_IPC_IPCGRH_SRCS0_SHIFT + srcId, CSL_IPC_IPCGRH_SRCS0_SHIFT + srcId, 1);
	
	return;
}

#ifdef __cplusplus
}
#endif

/* @} */

#endif /*CSL_IPCAUX_H_*/
