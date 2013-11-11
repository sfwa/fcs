/**
 *   @file  csl_srioAuxPhyLayer.h
 *
 *   @brief   
 *      This is the SRIO Auxilary Physical Header File which exposes the various
 *      CSL Functional Layer API's to configure the SRIO Switch.
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

#ifndef _CSL_SRIO_AUX_PHY_LAYER_H_
#define _CSL_SRIO_AUX_PHY_LAYER_H_

#include <ti/csl/soc.h>
#include <ti/csl/csl_srio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_SRIO_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_SRIO_GetDeviceInfo
 *
 *   @b Description
 *   @n The function reads the Device Identifier information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          deviceId          Device Identity populated by this API.
          deviceVendorId    Device Vendor Identity populated by this API  
          deviceRev         Device Revision populated by this API
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
 *   @n SRIO_RIO_DEV_ID_DEV_ID,SRIO_RIO_DEV_ID_DEV_VEN_ID,
 *   @n SRIO_RIO_DEV_INFO_DEVICE_REV
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          deviceId;
        Uint16          deviceVendorId;
        Uint32          deviceRev;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Read the Device Identity CAR
        CSL_SRIO_GetDeviceInfo (hSrio, &deviceId, &deviceVendorId, &deviceRev);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDeviceInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16*         deviceId,
    Uint16*         deviceVendorId,
    Uint32*         deviceRev
)
{
    Uint32 value = hSrio->RIO_DEV_ID;

    *deviceId       = CSL_FEXT(value, SRIO_RIO_DEV_ID_DEV_ID);
    *deviceVendorId = CSL_FEXT(value, SRIO_RIO_DEV_ID_DEV_VEN_ID);
    *deviceRev      = CSL_FEXT(hSrio->RIO_DEV_INFO, SRIO_RIO_DEV_INFO_DEVICE_REV);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDeviceInfo
 *
 *   @b Description
 *   @n The function sets up the Device Identifier information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          deviceId          Device Identity to configure
          deviceVendorId    Device Vendor Identity to configure
          deviceRev         Device Revision to configure
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
 *   @n SRIO_RIO_DEV_ID_DEV_ID,SRIO_RIO_DEV_ID_DEV_VEN_ID,
 *   @n SRIO_RIO_DEV_INFO_DEVICE_REV
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          deviceId;
        Uint16          deviceVendorId;
        Uint32          deviceRev;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Read the Device Identity CAR
        CSL_SRIO_GetDeviceInfo (hSrio, &deviceId, &deviceVendorId, &deviceRev);
        ...
        // Setup the device identity info
        CSL_SRIO_SetDeviceInfo (hSrio, deviceId, deviceVendorId, deviceRev);
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDeviceInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16          deviceId,
    Uint16          deviceVendorId,
    Uint32          deviceRev
)
{
    hSrio->RIO_DEV_ID   = CSL_FMK(SRIO_RIO_DEV_ID_DEV_ID,       deviceId)         |
                          CSL_FMK(SRIO_RIO_DEV_ID_DEV_VEN_ID,   deviceVendorId);
    hSrio->RIO_DEV_INFO = CSL_FMK(SRIO_RIO_DEV_INFO_DEVICE_REV, deviceRev);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAssemblyInfo
 *
 *   @b Description
 *   @n The function reads the Assembly Identifier information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          asblyId           Assembly Identity populated by this API.
          asblyVendorId     Assembly Vendor Identity populated by this API  
          asblyRev          Assembly Revision populated by this API
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
 *   @n SRIO_RIO_ASBLY_ID_ASBLY_ID,SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID,
 *   @n SRIO_RIO_ASBLY_INFO_ASBLY_REV,SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          asblyId;
        Uint16          asblyVendorId;
        Uint16          asblyRev;
        Uint16          extFeaturePtr;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Read the Assembly Identity CAR
        CSL_SRIO_GetAssemblyInfo (hSrio, &asblyId, &asblyVendorId, &asblyRev, &extFeaturePtr);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAssemblyInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16*         asblyId,
    Uint16*         asblyVendorId,
    Uint16*         asblyRev,
    Uint16*         extFeaturePtr
)
{
    Uint32 value;

    /* Read the assembly and vendor identifier */
    value = hSrio->RIO_ASBLY_ID;
    *asblyId       = CSL_FEXT(value, SRIO_RIO_ASBLY_ID_ASBLY_ID);
    *asblyVendorId = CSL_FEXT(value, SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID);

    /* Read the assembly information and external features */
    value = hSrio->RIO_ASBLY_INFO;
    *asblyRev      = CSL_FEXT(value, SRIO_RIO_ASBLY_INFO_ASBLY_REV);
    *extFeaturePtr = CSL_FEXT(value, SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetAssemblyInfo
 *
 *   @b Description
 *   @n The function sets up the Assembly Identifier information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          asblyId           Assembly Identity to configure.
          asblyVendorId     Assembly Vendor Identity to configure  
          asblyRev          Assembly Revision to configure
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
 *   @n SRIO_RIO_ASBLY_ID_ASBLY_ID,SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID,
 *   @n SRIO_RIO_ASBLY_INFO_ASBLY_REV,SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          asblyId;
        Uint16          asblyVendorId;
        Uint16          asblyRev;
        Uint16          extFeaturePtr;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Read the Assembly Identity CAR
        CSL_SRIO_GetAssemblyInfo (hSrio, &asblyId, &asblyVendorId, &asblyRev, &extFeaturePtr);
        ...
        // Sets up the Assembly Identity info
        CSL_SRIO_SetAssemblyInfo (hSrio, asblyId, asblyVendorId, asblyRev, extFeaturePtr);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetAssemblyInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16          asblyId,
    Uint16          asblyVendorId,
    Uint16          asblyRev,
    Uint16          extFeaturePtr
)
{
    hSrio->RIO_ASBLY_ID   = CSL_FMK(SRIO_RIO_ASBLY_ID_ASBLY_ID,      asblyId)         |
                            CSL_FMK(SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID,  asblyVendorId);

    hSrio->RIO_ASBLY_INFO = CSL_FMK(SRIO_RIO_ASBLY_INFO_ASBLY_REV,   asblyRev)         |
                            CSL_FMK(SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR,extFeaturePtr);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetProcessingElementFeatures
 *
 *   @b Description
 *   @n The function gets the processing element features.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          peFeatures        Processing Element Features to be populated by this API
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
 *   @n SRIO_RIO_PE_FEAT_BRDG,SRIO_RIO_PE_FEAT_MEM,SRIO_RIO_PE_FEAT_PROC,
 *   @n SRIO_RIO_PE_FEAT_SW,SRIO_RIO_PE_FEAT_MULT_P,SRIO_RIO_PE_FEAT_FLOW_ARB,
 *   @n SRIO_RIO_PE_FEAT_MC,SRIO_RIO_PE_FEAT_ERTC,SRIO_RIO_PE_FEAT_SRTC,
 *   @n SRIO_RIO_PE_FEAT_FLOW_CTRL, SRIO_RIO_PE_FEAT_CRF,
 *   @n SRIO_RIO_PE_FEAT_CTLS,SRIO_RIO_PE_FEAT_EXT_FEA,SRIO_RIO_PE_FEAT_EXT_AS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_PE_FEATURES    peFeatures;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the processing element features.
        CSL_SRIO_GetProcessingElementFeatures (hSrio, &peFeatures);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetProcessingElementFeatures
(
    CSL_SrioHandle      hSrio, 
    SRIO_PE_FEATURES*   ptrPEFeatures
)
{
    Uint32 value = hSrio->RIO_PE_FEAT;

    /* Populate the structure. */
    ptrPEFeatures->isBridge                       = CSL_FEXT (value, SRIO_RIO_PE_FEAT_BRDG);
    ptrPEFeatures->isEndpoint                     = CSL_FEXT (value, SRIO_RIO_PE_FEAT_MEM);
    ptrPEFeatures->isProcessor                    = CSL_FEXT (value, SRIO_RIO_PE_FEAT_PROC);
    ptrPEFeatures->isSwitch                       = CSL_FEXT (value, SRIO_RIO_PE_FEAT_SW);
    ptrPEFeatures->isMultiport                    = CSL_FEXT (value, SRIO_RIO_PE_FEAT_MULT_P);
    ptrPEFeatures->isFlowArbiterationSupported    = CSL_FEXT (value, SRIO_RIO_PE_FEAT_FLOW_ARB);
    ptrPEFeatures->isMulticastSupported           = CSL_FEXT (value, SRIO_RIO_PE_FEAT_MC);
    ptrPEFeatures->isExtendedRouteConfigSupported = CSL_FEXT (value, SRIO_RIO_PE_FEAT_ERTC);
    ptrPEFeatures->isStandardRouteConfigSupported = CSL_FEXT (value, SRIO_RIO_PE_FEAT_SRTC);
    ptrPEFeatures->isFlowControlSupported         = CSL_FEXT (value, SRIO_RIO_PE_FEAT_FLOW_CTRL);
    ptrPEFeatures->isCRFSupported                 = CSL_FEXT (value, SRIO_RIO_PE_FEAT_CRF);
    ptrPEFeatures->isCTLSSupported                = CSL_FEXT (value, SRIO_RIO_PE_FEAT_CTLS);
    ptrPEFeatures->isExtendedFeaturePtrValid      = CSL_FEXT (value, SRIO_RIO_PE_FEAT_EXT_FEA);
    ptrPEFeatures->numAddressBitSupported         = CSL_FEXT (value, SRIO_RIO_PE_FEAT_EXT_AS);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetProcessingElementFeatures
 *
 *   @b Description
 *   @n The function sets the processing element features.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          peFeatures        Processing Element Features to be configured
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
 *   @n SRIO_RIO_PE_FEAT_BRDG,SRIO_RIO_PE_FEAT_MEM,SRIO_RIO_PE_FEAT_PROC,
 *   @n SRIO_RIO_PE_FEAT_SW,SRIO_RIO_PE_FEAT_MULT_P,SRIO_RIO_PE_FEAT_FLOW_ARB,
 *   @n SRIO_RIO_PE_FEAT_MC,SRIO_RIO_PE_FEAT_ERTC,SRIO_RIO_PE_FEAT_SRTC,
 *   @n SRIO_RIO_PE_FEAT_FLOW_CTRL, SRIO_RIO_PE_FEAT_CRF,
 *   @n SRIO_RIO_PE_FEAT_CTLS,SRIO_RIO_PE_FEAT_EXT_FEA,SRIO_RIO_PE_FEAT_EXT_AS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_PE_FEATURES    peFeatures;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the processing element features.
        CSL_SRIO_GetProcessingElementFeatures (hSrio, &peFeatures);
        
        // Modify the processing element features and make this a BRIDGE
        peFeatures.isBridge = 1;
        CSL_SRIO_SetProcessingElementFeatures(hSrio, &peFeatures);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetProcessingElementFeatures
(
    CSL_SrioHandle      hSrio, 
    SRIO_PE_FEATURES*   ptrPEFeatures
)
{
    Uint32 value = 0;

    /* Initialize the various fields in the register */ 
    CSL_FINS (value, SRIO_RIO_PE_FEAT_BRDG,     ptrPEFeatures->isBridge);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_MEM,      ptrPEFeatures->isEndpoint);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_PROC,     ptrPEFeatures->isProcessor);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_SW,       ptrPEFeatures->isSwitch);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_MULT_P,   ptrPEFeatures->isMultiport);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_FLOW_ARB, ptrPEFeatures->isFlowArbiterationSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_MC,       ptrPEFeatures->isMulticastSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_ERTC,     ptrPEFeatures->isExtendedRouteConfigSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_SRTC,     ptrPEFeatures->isStandardRouteConfigSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_FLOW_CTRL,ptrPEFeatures->isFlowControlSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_CRF,      ptrPEFeatures->isCRFSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_CTLS,     ptrPEFeatures->isCTLSSupported);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_EXT_FEA,  ptrPEFeatures->isExtendedFeaturePtrValid);
    CSL_FINS (value, SRIO_RIO_PE_FEAT_EXT_AS,   ptrPEFeatures->numAddressBitSupported);

    hSrio->RIO_PE_FEAT = value;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetSwitchPortInfo
 *
 *   @b Description
 *   @n The function reads the Switch Port information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          numPorts          The total number of ports populated by this API
          portNum           The port number from which the Maintenance read operation
                            accessed this register.
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
 *   @n SRIO_RIO_SW_PORT_PORT_TOTAL,SRIO_RIO_SW_PORT_PORT_NUM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           numPorts;
        Uint8           portNum;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Switch port information
        CSL_SRIO_GetSwitchPortInfo (hSrio, &numPorts, &portNum);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetSwitchPortInfo
(
    CSL_SrioHandle  hSrio, 
    Uint8*          numPorts,
    Uint8*          portNum
)
{
    *numPorts   = CSL_FEXT(hSrio->RIO_SW_PORT,  SRIO_RIO_SW_PORT_PORT_TOTAL);
    *portNum    = CSL_FEXT(hSrio->RIO_SW_PORT,  SRIO_RIO_SW_PORT_PORT_NUM);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetSourceOperationCAR
 *
 *   @b Description
 *   @n The function reads the Switch Port Capabilities which are all the logical
 *      operations that can be issued by the application.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ptrSrcOp          Pointer to the Source CAR Structure populated by this 
                            API
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
 *   @n SRIO_RIO_SRC_OP_G_READ, SRIO_RIO_SRC_OP_G_IREAD, SRIO_RIO_SRC_OP_G_READ_OWN,
 *   @n SRIO_RIO_SRC_OP_G_DC_INVALIDATE, SRIO_RIO_SRC_OP_G_CASTOUT, SRIO_RIO_SRC_OP_G_DC_FLUSH,
 *   @n SRIO_RIO_SRC_OP_G_IO_READ, SRIO_RIO_SRC_OP_G_IC_INVALIDATE, SRIO_RIO_SRC_OP_G_TLB_INVALIDATE,
 *   @n SRIO_RIO_SRC_OP_G_TLB_SYNC, SRIO_RIO_SRC_OP_DS_TM, SRIO_RIO_SRC_OP_DS, 
 *   @n SRIO_RIO_SRC_OP_IMPLEMENT_DEF, SRIO_RIO_SRC_OP_READ, SRIO_RIO_SRC_OP_WRITE,
 *   @n SRIO_RIO_SRC_OP_STRM_WR, SRIO_RIO_SRC_OP_WR_RES, SRIO_RIO_SRC_OP_D_MSG, SRIO_RIO_SRC_OP_DBELL,
 *   @n SRIO_RIO_SRC_OP_ACSWAP, SRIO_RIO_SRC_OP_ATSWAP, SRIO_RIO_SRC_OP_A_INC, SRIO_RIO_SRC_OP_A_DEC,
 *   @n SRIO_RIO_SRC_OP_A_SET, SRIO_RIO_SRC_OP_A_CLEAR, SRIO_RIO_SRC_OP_A_SWAP, SRIO_RIO_SRC_OP_PORT_WR, 
 *   @n SRIO_RIO_SRC_OP_IMPLEMENT_DEF2
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_OP_CAR     srcOPCAR;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Source Operation CAR
        CSL_SRIO_GetSourceOperationCAR (hSrio, &srcOPCAR);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetSourceOperationCAR
(
    CSL_SrioHandle  hSrio, 
    SRIO_OP_CAR*    ptrSrcOp
)
{
    Uint32  value = hSrio->RIO_SRC_OP;

    /* Extract the fields from the Source OP */
    ptrSrcOp->gsmRead                   = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_READ);
    ptrSrcOp->gsmInstrnRead             = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_IREAD);
    ptrSrcOp->gsmReadOwn                = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_READ_OWN);
    ptrSrcOp->gsmDataCacheInvalidate    = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_DC_INVALIDATE);
    ptrSrcOp->gsmCastout                = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_CASTOUT);
    ptrSrcOp->gsmDataCacheFlush         = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_DC_FLUSH);
    ptrSrcOp->gsmIORead                 = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_IO_READ);
    ptrSrcOp->gsmInstrnCacheInvalidate  = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_IC_INVALIDATE);
    ptrSrcOp->gsmTLBInvalidate          = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_TLB_INVALIDATE);
    ptrSrcOp->gsmTLBSync                = CSL_FEXT (value, SRIO_RIO_SRC_OP_G_TLB_SYNC);
    ptrSrcOp->dataStreamingTM           = CSL_FEXT (value, SRIO_RIO_SRC_OP_DS_TM);
    ptrSrcOp->dataStreamingSupport      = CSL_FEXT (value, SRIO_RIO_SRC_OP_DS);
    ptrSrcOp->implnDefined              = CSL_FEXT (value, SRIO_RIO_SRC_OP_IMPLEMENT_DEF);
    ptrSrcOp->readSupport               = CSL_FEXT (value, SRIO_RIO_SRC_OP_READ);
    ptrSrcOp->writeSupport              = CSL_FEXT (value, SRIO_RIO_SRC_OP_WRITE);
    ptrSrcOp->streamWriteSupport        = CSL_FEXT (value, SRIO_RIO_SRC_OP_STRM_WR);
    ptrSrcOp->writeResponseSupport      = CSL_FEXT (value, SRIO_RIO_SRC_OP_WR_RES);
    ptrSrcOp->dataMessageSupport        = CSL_FEXT (value, SRIO_RIO_SRC_OP_D_MSG); 
    ptrSrcOp->doorbellSupport           = CSL_FEXT (value, SRIO_RIO_SRC_OP_DBELL); 
    ptrSrcOp->atomicCompareSwapSupport  = CSL_FEXT (value, SRIO_RIO_SRC_OP_ACSWAP);
    ptrSrcOp->atomicTestSwapSupport     = CSL_FEXT (value, SRIO_RIO_SRC_OP_ATSWAP);
    ptrSrcOp->atomicIncSupport          = CSL_FEXT (value, SRIO_RIO_SRC_OP_A_INC);
    ptrSrcOp->atomicDecSupport          = CSL_FEXT (value, SRIO_RIO_SRC_OP_A_DEC);
    ptrSrcOp->atomicSetSupport          = CSL_FEXT (value, SRIO_RIO_SRC_OP_A_SET);
    ptrSrcOp->atomicClearSupport        = CSL_FEXT (value, SRIO_RIO_SRC_OP_A_CLEAR);
    ptrSrcOp->atomicSwapSupport         = CSL_FEXT (value, SRIO_RIO_SRC_OP_A_SWAP);
    ptrSrcOp->portWriteOperationSupport = CSL_FEXT (value, SRIO_RIO_SRC_OP_PORT_WR);
    ptrSrcOp->implnDefined2             = CSL_FEXT (value, SRIO_RIO_SRC_OP_IMPLEMENT_DEF2);

    return;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetSourceOperationCAR
 *
 *   @b Description
 *   @n The function sets the Switch Port Capabilities which are all the logical
 *      operations that can be issued by the application.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ptrSrcOp          Pointer to the Source CAR Structure to be configured
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
 *   @n SRIO_RIO_SRC_OP_G_READ, SRIO_RIO_SRC_OP_G_IREAD, SRIO_RIO_SRC_OP_G_READ_OWN,
 *   @n SRIO_RIO_SRC_OP_G_DC_INVALIDATE, SRIO_RIO_SRC_OP_G_CASTOUT, SRIO_RIO_SRC_OP_G_DC_FLUSH,
 *   @n SRIO_RIO_SRC_OP_G_IO_READ, SRIO_RIO_SRC_OP_G_IC_INVALIDATE, SRIO_RIO_SRC_OP_G_TLB_INVALIDATE,
 *   @n SRIO_RIO_SRC_OP_G_TLB_SYNC, SRIO_RIO_SRC_OP_DS_TM, SRIO_RIO_SRC_OP_DS, 
 *   @n SRIO_RIO_SRC_OP_IMPLEMENT_DEF, SRIO_RIO_SRC_OP_READ, SRIO_RIO_SRC_OP_WRITE,
 *   @n SRIO_RIO_SRC_OP_STRM_WR, SRIO_RIO_SRC_OP_WR_RES, SRIO_RIO_SRC_OP_D_MSG, SRIO_RIO_SRC_OP_DBELL,
 *   @n SRIO_RIO_SRC_OP_ACSWAP, SRIO_RIO_SRC_OP_ATSWAP, SRIO_RIO_SRC_OP_A_INC, SRIO_RIO_SRC_OP_A_DEC,
 *   @n SRIO_RIO_SRC_OP_A_SET, SRIO_RIO_SRC_OP_A_CLEAR, SRIO_RIO_SRC_OP_A_SWAP, SRIO_RIO_SRC_OP_PORT_WR, 
 *   @n SRIO_RIO_SRC_OP_IMPLEMENT_DEF2
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_OP_CAR     srcOPCAR;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Source Operation CAR
        CSL_SRIO_GetSourceOperationCAR (hSrio, &srcOPCAR);

        // Allow Doorbell support.
        srcOPCAR.doorbellSupport = 1;

        // Set the Source Operation CAR
        CSL_SRIO_SetSourceOperationCAR (hSrio, &srcOPCAR);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetSourceOperationCAR
(
    CSL_SrioHandle  hSrio, 
    SRIO_OP_CAR*    ptrSrcOp
)
{
    Uint32  value = 0;

    /* Populate the fields into the Source OP */
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_READ,            ptrSrcOp->gsmRead);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_IREAD,           ptrSrcOp->gsmInstrnRead);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_READ_OWN,        ptrSrcOp->gsmReadOwn);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_DC_INVALIDATE,   ptrSrcOp->gsmDataCacheInvalidate);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_CASTOUT,         ptrSrcOp->gsmCastout);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_DC_FLUSH,        ptrSrcOp->gsmDataCacheFlush);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_IO_READ,         ptrSrcOp->gsmIORead);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_IC_INVALIDATE,   ptrSrcOp->gsmInstrnCacheInvalidate);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_TLB_INVALIDATE,  ptrSrcOp->gsmTLBInvalidate);
    CSL_FINS (value, SRIO_RIO_SRC_OP_G_TLB_SYNC,        ptrSrcOp->gsmTLBSync);
    CSL_FINS (value, SRIO_RIO_SRC_OP_DS_TM,             ptrSrcOp->dataStreamingTM);
    CSL_FINS (value, SRIO_RIO_SRC_OP_DS,                ptrSrcOp->dataStreamingSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_IMPLEMENT_DEF,     ptrSrcOp->implnDefined);
    CSL_FINS (value, SRIO_RIO_SRC_OP_READ,              ptrSrcOp->readSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_WRITE,             ptrSrcOp->writeSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_STRM_WR,           ptrSrcOp->streamWriteSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_WR_RES,            ptrSrcOp->writeResponseSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_D_MSG,             ptrSrcOp->dataMessageSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_DBELL,             ptrSrcOp->doorbellSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_ACSWAP,            ptrSrcOp->atomicCompareSwapSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_ATSWAP,            ptrSrcOp->atomicTestSwapSupport);   
    CSL_FINS (value, SRIO_RIO_SRC_OP_A_INC,             ptrSrcOp->atomicIncSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_A_DEC,             ptrSrcOp->atomicDecSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_A_SET,             ptrSrcOp->atomicSetSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_A_CLEAR,           ptrSrcOp->atomicClearSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_A_SWAP,            ptrSrcOp->atomicSwapSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_PORT_WR,           ptrSrcOp->portWriteOperationSupport);
    CSL_FINS (value, SRIO_RIO_SRC_OP_IMPLEMENT_DEF2,    ptrSrcOp->implnDefined2);    

    hSrio->RIO_SRC_OP = value;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDestOperationCAR
 *
 *   @b Description
 *   @n The function reads the Switch Port Capabilities which are all the logical
 *      operations that can be issued by the application.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ptrDstOp          Pointer to the Dest CAR Structure populated by this 
                            API
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
 *   @n SRIO_RIO_DEST_OP_G_READ, SRIO_RIO_DEST_OP_G_IREAD, SRIO_RIO_DEST_OP_G_READ_OWN,
 *   @n SRIO_RIO_DEST_OP_G_DC_INVALIDATE, SRIO_RIO_DEST_OP_G_CASTOUT, SRIO_RIO_DEST_OP_G_DC_FLUSH,
 *   @n SRIO_RIO_DEST_OP_G_IO_READ, SRIO_RIO_DEST_OP_G_IC_INVALIDATE, SRIO_RIO_DEST_OP_G_TLB_INVALIDATE,
 *   @n SRIO_RIO_DEST_OP_G_TLB_SYNC, SRIO_RIO_DEST_OP_DS_TM, SRIO_RIO_DEST_OP_DS,
 *   @n SRIO_RIO_DEST_OP_IMPLEMENT_DEF, SRIO_RIO_DEST_OP_READ, SRIO_RIO_DEST_OP_WRITE, 
 *   @n SRIO_RIO_DEST_OP_STRM_WR, SRIO_RIO_DEST_OP_WR_RES, SRIO_RIO_DEST_OP_D_MSG, SRIO_RIO_DEST_OP_DBELL,
 *   @n SRIO_RIO_DEST_OP_ACSWAP, SRIO_RIO_DEST_OP_ATSWAP, SRIO_RIO_DEST_OP_A_INC, SRIO_RIO_DEST_OP_A_DEC,
 *   @n SRIO_RIO_DEST_OP_A_SET, SRIO_RIO_DEST_OP_A_CLEAR, SRIO_RIO_DEST_OP_A_SWAP, SRIO_RIO_DEST_OP_PORT_WR,
 *   @n SRIO_RIO_DEST_OP_IMPLEMENT_DEF2
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_OP_CAR     dstOPCAR;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Destination Operation CAR
        CSL_SRIO_GetDestOperationCAR (hSrio, &dstOPCAR);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDestOperationCAR
(
    CSL_SrioHandle  hSrio, 
    SRIO_OP_CAR*    ptrDstOp
)
{
    Uint32  value = hSrio->RIO_DEST_OP;

    /* Extract the fields from the Destination OP */
    ptrDstOp->gsmRead                   = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_READ);
    ptrDstOp->gsmInstrnRead             = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_IREAD);
    ptrDstOp->gsmReadOwn                = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_READ_OWN);
    ptrDstOp->gsmDataCacheInvalidate    = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_DC_INVALIDATE);
    ptrDstOp->gsmCastout                = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_CASTOUT);
    ptrDstOp->gsmDataCacheFlush         = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_DC_FLUSH);
    ptrDstOp->gsmIORead                 = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_IO_READ);
    ptrDstOp->gsmInstrnCacheInvalidate  = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_IC_INVALIDATE);
    ptrDstOp->gsmTLBInvalidate          = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_TLB_INVALIDATE);
    ptrDstOp->gsmTLBSync                = CSL_FEXT (value, SRIO_RIO_DEST_OP_G_TLB_SYNC);
    ptrDstOp->dataStreamingTM           = CSL_FEXT (value, SRIO_RIO_DEST_OP_DS_TM);
    ptrDstOp->dataStreamingSupport      = CSL_FEXT (value, SRIO_RIO_DEST_OP_DS);
    ptrDstOp->implnDefined              = CSL_FEXT (value, SRIO_RIO_DEST_OP_IMPLEMENT_DEF);
    ptrDstOp->readSupport               = CSL_FEXT (value, SRIO_RIO_DEST_OP_READ);
    ptrDstOp->writeSupport              = CSL_FEXT (value, SRIO_RIO_DEST_OP_WRITE);
    ptrDstOp->streamWriteSupport        = CSL_FEXT (value, SRIO_RIO_DEST_OP_STRM_WR);
    ptrDstOp->writeResponseSupport      = CSL_FEXT (value, SRIO_RIO_DEST_OP_WR_RES);
    ptrDstOp->dataMessageSupport        = CSL_FEXT (value, SRIO_RIO_DEST_OP_D_MSG);
    ptrDstOp->doorbellSupport           = CSL_FEXT (value, SRIO_RIO_DEST_OP_DBELL);
    ptrDstOp->atomicCompareSwapSupport  = CSL_FEXT (value, SRIO_RIO_DEST_OP_ACSWAP);
    ptrDstOp->atomicTestSwapSupport     = CSL_FEXT (value, SRIO_RIO_DEST_OP_ATSWAP);
    ptrDstOp->atomicIncSupport          = CSL_FEXT (value, SRIO_RIO_DEST_OP_A_INC);
    ptrDstOp->atomicDecSupport          = CSL_FEXT (value, SRIO_RIO_DEST_OP_A_DEC);
    ptrDstOp->atomicSetSupport          = CSL_FEXT (value, SRIO_RIO_DEST_OP_A_SET);
    ptrDstOp->atomicClearSupport        = CSL_FEXT (value, SRIO_RIO_DEST_OP_A_CLEAR);
    ptrDstOp->atomicSwapSupport         = CSL_FEXT (value, SRIO_RIO_DEST_OP_A_SWAP);
    ptrDstOp->portWriteOperationSupport = CSL_FEXT (value, SRIO_RIO_DEST_OP_PORT_WR);
    ptrDstOp->implnDefined2             = CSL_FEXT (value, SRIO_RIO_DEST_OP_IMPLEMENT_DEF2);    
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDestOperationCAR
 *
 *   @b Description
 *   @n The function sets the Switch Port Capabilities which are all the logical
 *      operations that can be issued by the application.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ptrDstOp          Pointer to the Dest CAR Structure to be configured
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
 *   @n SRIO_RIO_DEST_OP_G_READ, SRIO_RIO_DEST_OP_G_IREAD, SRIO_RIO_DEST_OP_G_READ_OWN,
 *   @n SRIO_RIO_DEST_OP_G_DC_INVALIDATE, SRIO_RIO_DEST_OP_G_CASTOUT, SRIO_RIO_DEST_OP_G_DC_FLUSH,
 *   @n SRIO_RIO_DEST_OP_G_IO_READ, SRIO_RIO_DEST_OP_G_IC_INVALIDATE, SRIO_RIO_DEST_OP_G_TLB_INVALIDATE,
 *   @n SRIO_RIO_DEST_OP_G_TLB_SYNC, SRIO_RIO_DEST_OP_DS_TM, SRIO_RIO_DEST_OP_DS,
 *   @n SRIO_RIO_DEST_OP_IMPLEMENT_DEF, SRIO_RIO_DEST_OP_READ, SRIO_RIO_DEST_OP_WRITE, 
 *   @n SRIO_RIO_DEST_OP_STRM_WR, SRIO_RIO_DEST_OP_WR_RES, SRIO_RIO_DEST_OP_D_MSG, SRIO_RIO_DEST_OP_DBELL,
 *   @n SRIO_RIO_DEST_OP_ACSWAP, SRIO_RIO_DEST_OP_ATSWAP, SRIO_RIO_DEST_OP_A_INC, SRIO_RIO_DEST_OP_A_DEC,
 *   @n SRIO_RIO_DEST_OP_A_SET, SRIO_RIO_DEST_OP_A_CLEAR, SRIO_RIO_DEST_OP_A_SWAP, SRIO_RIO_DEST_OP_PORT_WR,
 *   @n SRIO_RIO_DEST_OP_IMPLEMENT_DEF2
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_OP_CAR     dstOPCAR;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Dest Operation CAR
        CSL_SRIO_GetDestOperationCAR (hSrio, &dstOPCAR);

        // Allow Doorbell support.
        dstOPCAR.doorbellSupport = 1;

        // Set the Dest Operation CAR
        CSL_SRIO_SetDestOperationCAR (hSrio, &dstOPCAR);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDestOperationCAR
(
    CSL_SrioHandle  hSrio, 
    SRIO_OP_CAR*    ptrDstOp
)
{
    Uint32  value = 0;

    /* Populate the fields into the Source OP */
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_READ,           ptrDstOp->gsmRead);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_IREAD,          ptrDstOp->gsmInstrnRead);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_READ_OWN,       ptrDstOp->gsmReadOwn);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_DC_INVALIDATE,  ptrDstOp->gsmDataCacheInvalidate);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_CASTOUT,        ptrDstOp->gsmCastout);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_DC_FLUSH,       ptrDstOp->gsmDataCacheFlush);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_IO_READ,        ptrDstOp->gsmIORead);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_IC_INVALIDATE,  ptrDstOp->gsmInstrnCacheInvalidate);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_TLB_INVALIDATE, ptrDstOp->gsmTLBInvalidate);
    CSL_FINS (value, SRIO_RIO_DEST_OP_G_TLB_SYNC,       ptrDstOp->gsmTLBSync);
    CSL_FINS (value, SRIO_RIO_DEST_OP_DS_TM,            ptrDstOp->dataStreamingTM);
    CSL_FINS (value, SRIO_RIO_DEST_OP_DS,               ptrDstOp->dataStreamingSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_IMPLEMENT_DEF,    ptrDstOp->implnDefined);
    CSL_FINS (value, SRIO_RIO_DEST_OP_READ,             ptrDstOp->readSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_WRITE,            ptrDstOp->writeSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_STRM_WR,          ptrDstOp->streamWriteSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_WR_RES,           ptrDstOp->writeResponseSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_D_MSG,            ptrDstOp->dataMessageSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_DBELL,            ptrDstOp->doorbellSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_ACSWAP,           ptrDstOp->atomicCompareSwapSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_ATSWAP,           ptrDstOp->atomicTestSwapSupport);   
    CSL_FINS (value, SRIO_RIO_DEST_OP_A_INC,            ptrDstOp->atomicIncSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_A_DEC,            ptrDstOp->atomicDecSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_A_SET,            ptrDstOp->atomicSetSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_A_CLEAR,          ptrDstOp->atomicClearSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_A_SWAP,           ptrDstOp->atomicSwapSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_PORT_WR,          ptrDstOp->portWriteOperationSupport);
    CSL_FINS (value, SRIO_RIO_DEST_OP_IMPLEMENT_DEF2,   ptrDstOp->implnDefined2);    

    hSrio->RIO_DEST_OP = value;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDataStreamingInfo
 *
 *   @b Description
 *   @n The function gets the data streaming information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          maxPDU            Maximum PDU 
          segSupport        Num of segmentation context
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
 *   @n SRIO_RIO_DS_INFO_MAX_PDU, SRIO_RIO_DS_INFO_SEG_SUPPORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          maxPDU;
        Uint16          segSupport;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Data Streaming Information
        CSL_SRIO_GetDataStreamingInfo (hSrio, &maxPDU, &segSupport);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDataStreamingInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16*         maxPDU,
    Uint16*         segSupport
)
{
    Uint32 value = hSrio->RIO_DS_INFO;

    *maxPDU     = CSL_FEXT (value, SRIO_RIO_DS_INFO_MAX_PDU);
    *segSupport = CSL_FEXT (value, SRIO_RIO_DS_INFO_SEG_SUPPORT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDataStreamingInfo
 *
 *   @b Description
 *   @n The function sets the data streaming information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          maxPDU            Maximum PDU 
          segSupport        Num of segmentation context
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
 *   @n SRIO_RIO_DS_INFO_MAX_PDU, SRIO_RIO_DS_INFO_SEG_SUPPORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          maxPDU;
        Uint16          segSupport;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Data Streaming Information: 64K bytes and 64K Segmentation Context
        CSL_SRIO_SetDataStreamingInfo (hSrio, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDataStreamingInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16          maxPDU,
    Uint16          segSupport
)
{
    hSrio->RIO_DS_INFO = CSL_FMK(SRIO_RIO_DS_INFO_MAX_PDU, maxPDU)             |
                         CSL_FMK(SRIO_RIO_DS_INFO_SEG_SUPPORT, segSupport);   
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDataStreamingType
 *
 *   @b Description
 *   @n The function gets the data streaming traffic management type 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          type              Data Streaming Type populated by this API
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
 *   @n SRIO_RIO_DS_LL_CTL_TM_TYPES
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           type;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Data Streaming Type
        CSL_SRIO_GetDataStreamingType (hSrio, &type);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDataStreamingType
(
    CSL_SrioHandle  hSrio, 
    Uint8*          type
)
{
    *type = CSL_FEXT (hSrio->RIO_DS_LL_CTL, SRIO_RIO_DS_LL_CTL_TM_TYPES);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDataStreamingMode
 *
 *   @b Description
 *   @n The function gets the data streaming traffic management mode of operation
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          mode              Traffic Management Mode populated by this API
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
 *   @n SRIO_RIO_DS_LL_CTL_TM_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Data Streaming TM Mode
        CSL_SRIO_GetDataStreamingMode (hSrio, &mode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDataStreamingMode
(
    CSL_SrioHandle  hSrio, 
    Uint8*          mode
)
{
    *mode = CSL_FEXT (hSrio->RIO_DS_LL_CTL, SRIO_RIO_DS_LL_CTL_TM_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDataStreamingMode
 *
 *   @b Description
 *   @n The function sets the data streaming traffic management mode of operation
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          mode              Traffic Management Mode to be configured
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
 *   @n SRIO_RIO_DS_LL_CTL_TM_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Data Streaming TM Mode
        CSL_SRIO_SetDataStreamingMode (hSrio, &mode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDataStreamingMode
(
    CSL_SrioHandle  hSrio, 
    Uint8           mode
)
{
    CSL_FINS (hSrio->RIO_DS_LL_CTL, SRIO_RIO_DS_LL_CTL_TM_MODE, mode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDataStreamingMTU
 *
 *   @b Description
 *   @n The function gets the data streaming MTU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          mtu               MTU populated by this API
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
 *   @n SRIO_RIO_DS_LL_CTL_MTU
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           mtu;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Data Streaming MTU
        CSL_SRIO_GetDataStreamingMTU (hSrio, &mtu);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDataStreamingMTU
(
    CSL_SrioHandle  hSrio, 
    Uint8*          mtu
)
{
    *mtu = CSL_FEXT (hSrio->RIO_DS_LL_CTL, SRIO_RIO_DS_LL_CTL_MTU);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDataStreamingMTU
 *
 *   @b Description
 *   @n The function sets the data streaming MTU
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          mtu               MTU to be configured
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
 *   @n SRIO_RIO_DS_LL_CTL_MTU
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Data Streaming MTU
        CSL_SRIO_SetDataStreamingMTU (hSrio, 64);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDataStreamingMTU
(
    CSL_SrioHandle  hSrio, 
    Uint8           mtu
)
{
    CSL_FINS (hSrio->RIO_DS_LL_CTL, SRIO_RIO_DS_LL_CTL_MTU, mtu);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPEExtendedAddressingControl
 *
 *   @b Description
 *   @n The function sets the processing element extended addressing control 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          controlInfo       Extended Addressing Control Information to be configured
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
 *   @n SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Extended Addressing Control Information to operate in 34 bit addresses
        CSL_SRIO_SetPEExtendedAddressingControl (hSrio, 0x1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPEExtendedAddressingControl
(
    CSL_SrioHandle  hSrio, 
    Uint8           controlInfo
)
{
    CSL_FINS (hSrio->RIO_PE_LL_CTL, SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL, controlInfo);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPEExtendedAddressingControl
 *
 *   @b Description
 *   @n The function gets the processing element extended addressing control 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          controlInfo       Extended Addressing Control Information populated
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
 *   @n SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           controlInfo;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Extended Addressing Control Information
        CSL_SRIO_GetPEExtendedAddressingControl (hSrio, &controlInfo);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPEExtendedAddressingControl
(
    CSL_SrioHandle  hSrio, 
    Uint8*          controlInfo
)
{
    *controlInfo = CSL_FEXT (hSrio->RIO_PE_LL_CTL, SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLocalConfig0SpaceInfo
 *
 *   @b Description
 *   @n The function sets the local configuration space base address0 Information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          baseInfo0         Local Configuration Base Address0 
          baseInfo1         Local Configuration Base Address1
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
 *   @n SRIO_RIO_LCL_CFG_HBAR_LCSBA0,SRIO_RIO_LCL_CFG_HBAR_LCSBA1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Local configuration information
        CSL_SRIO_SetLocalConfig0SpaceInfo (hSrio, 0x100, 0x200);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLocalConfig0SpaceInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16          baseInfo0,
    Uint16          baseInfo1
)
{
    hSrio->RIO_LCL_CFG_HBAR = CSL_FMK (SRIO_RIO_LCL_CFG_HBAR_LCSBA1, baseInfo1) |   
                              CSL_FMK (SRIO_RIO_LCL_CFG_HBAR_LCSBA0, baseInfo0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLocalConfig0SpaceInfo
 *
 *   @b Description
 *   @n The function gets the local configuration space base address0 information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          baseInfo0         Local Configuration Base Address0 
          baseInfo1         Local Configuration Base Address1
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
 *   @n SRIO_RIO_LCL_CFG_HBAR_LCSBA0,SRIO_RIO_LCL_CFG_HBAR_LCSBA1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          baseInfo0;
        Uint16          baseInfo1;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Local configuration information
        CSL_SRIO_GetLocalConfig0SpaceInfo (hSrio, &baseInfo0, &baseInfo1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLocalConfig0SpaceInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16*         baseInfo0,
    Uint16*         baseInfo1
)
{
    Uint32 value = hSrio->RIO_LCL_CFG_HBAR;
    *baseInfo0 = CSL_FEXT(value, SRIO_RIO_LCL_CFG_HBAR_LCSBA0);
    *baseInfo1 = CSL_FEXT(value, SRIO_RIO_LCL_CFG_HBAR_LCSBA1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLocalConfig1SpaceInfo
 *
 *   @b Description
 *   @n The function sets the local configuration space base address1 Information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          baseInfo0         Local Configuration Base Address0 
          baseInfo1         Local Configuration Base Address1
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
 *   @n SRIO_RIO_LCL_CFG_BAR_LCSBA0,SRIO_RIO_LCL_CFG_BAR_LCSBA1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Local configuration1 information
        CSL_SRIO_SetLocalConfig1SpaceInfo (hSrio, 0x100, 0x200);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLocalConfig1SpaceInfo
(
    CSL_SrioHandle  hSrio, 
    Uint8           baseInfo0,
    Uint32          baseInfo1
)
{
    hSrio->RIO_LCL_CFG_BAR = CSL_FMK (SRIO_RIO_LCL_CFG_BAR_LCSBA1, baseInfo1) |   
                             CSL_FMK (SRIO_RIO_LCL_CFG_BAR_LCSBA0, baseInfo0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLocalConfig1SpaceInfo
 *
 *   @b Description
 *   @n The function gets the local configuration space base address1 Information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          baseInfo0         Local Configuration Base Address0 
          baseInfo1         Local Configuration Base Address1
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
 *   @n SRIO_RIO_LCL_CFG_BAR_LCSBA0,SRIO_RIO_LCL_CFG_BAR_LCSBA1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           baseInfo0;
        Uint32          baseInfo1;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Local configuration1 information
        CSL_SRIO_GetLocalConfig1SpaceInfo (hSrio, &baseInfo0, &baseInfo1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLocalConfig1SpaceInfo
(
    CSL_SrioHandle  hSrio, 
    Uint8*          baseInfo0,
    Uint32*         baseInfo1
)
{
    Uint32 value = hSrio->RIO_LCL_CFG_BAR;
    *baseInfo0 = CSL_FEXT(value, SRIO_RIO_LCL_CFG_BAR_LCSBA0);
    *baseInfo1 = CSL_FEXT(value, SRIO_RIO_LCL_CFG_BAR_LCSBA1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetDeviceIDCSR
 *
 *   @b Description
 *   @n The function sets the 8bit and 16 device ID in the standard CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          baseID8Bit        8bit Device Identifier.
          baseID16Bit       16bit Device Identifier.
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
 *   @n SRIO_RIO_BASE_ID_BASE_ID,SRIO_RIO_BASE_ID_LARGE_BASE_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Device Identifier CSR
        CSL_SRIO_SetDeviceIDCSR (hSrio, 0x0, 0xDEAD);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetDeviceIDCSR
(
    CSL_SrioHandle  hSrio, 
    Uint8           baseID8Bit,
    Uint16          baseID16Bit
)
{
    /* Populate the fields into the Device ID CSR */
    hSrio->RIO_BASE_ID = CSL_FMK (SRIO_RIO_BASE_ID_BASE_ID, baseID8Bit)       | 
                         CSL_FMK (SRIO_RIO_BASE_ID_LARGE_BASE_ID, baseID16Bit);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDeviceIDCSR
 *
 *   @b Description
 *   @n The function gets the 8bit and 16 device ID in the standard CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          baseID8Bit        8bit Device Identifier populated by this API
          baseID16Bit       16bit Device Identifier populated by this API
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
 *   @n SRIO_RIO_BASE_ID_BASE_ID,SRIO_RIO_BASE_ID_LARGE_BASE_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           baseID8Bit;
        Uint16          baseID16Bit;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Device Identifier CSR
        CSL_SRIO_GetDeviceIDCSR (hSrio, &baseID8Bit, &baseID16Bit);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDeviceIDCSR
(
    CSL_SrioHandle  hSrio, 
    Uint8*          baseID8Bit,
    Uint16*         baseID16Bit
)
{
    Uint32 value = hSrio->RIO_BASE_ID;

    *baseID8Bit  = CSL_FEXT (value, SRIO_RIO_BASE_ID_BASE_ID);
    *baseID16Bit = CSL_FEXT (value, SRIO_RIO_BASE_ID_LARGE_BASE_ID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetHostDeviceID
 *
 *   @b Description
 *   @n The function sets the host device identifier which is responsible for 
 *      initializing the processing element.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          identifier        Device Identifier to be configured.
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
 *   @n SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Host Device Identifier.
        CSL_SRIO_SetHostDeviceID (hSrio, 0xBEEF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetHostDeviceID
(
    CSL_SrioHandle  hSrio, 
    Uint16          identifier
)
{
    /* Populate the fields into the Device ID CSR */
    hSrio->RIO_HOST_BASE_ID_LOCK = CSL_FMK (SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID, identifier);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetHostDeviceID
 *
 *   @b Description
 *   @n The function gets the host device identifier which is responsible for 
 *      initializing the processing element.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          identifier        Device Identifier to be populated by this API.
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
 *   @n SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          deviceID;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Host Device Identifier.
        CSL_SRIO_GetHostDeviceID (hSrio, &deviceID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetHostDeviceID
(
    CSL_SrioHandle  hSrio, 
    Uint16*         identifier
)
{
    *identifier = CSL_FEXT (hSrio->RIO_HOST_BASE_ID_LOCK, SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetCompTagCSR
 *
 *   @b Description
 *   @n The function sets the component tag CSR
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          ctag          Component Tag to be configured
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
 *   @n SRIO_RIO_COMP_TAG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Component Tag CSR
        CSL_SRIO_SetCompTagCSR (hSrio, 0xBEEF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetCompTagCSR
(
    CSL_SrioHandle  hSrio, 
    Uint32          ctag
)
{
    hSrio->RIO_COMP_TAG = ctag;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetCompTagCSR
 *
 *   @b Description
 *   @n The function gets the component tag CSR
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          ctag          Component Tag to be populated by this API
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
 *   @n SRIO_RIO_COMP_TAG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          tag;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the component TAG CSR
        CSL_SRIO_GetCompTagCSR (hSrio, &tag);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetCompTagCSR
(
    CSL_SrioHandle  hSrio, 
    Uint32*         ctag
)
{
    *ctag = hSrio->RIO_COMP_TAG;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetMaintBlockHeaderInfo
 *
 *   @b Description
 *   @n The function gets the maintainance block header information
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          extFeatPtr     Extended Feature Pointer populated by this API
          extFeatID      Extended Feature ID populated by this API
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
 *   @n SRIO_RIO_SP_MB_HEAD_EF_PTR,SRIO_RIO_SP_MB_HEAD_EF_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          extFeatPtr;
        Uint16          extFeatID;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the maintainance block header
        CSL_SRIO_GetMaintBlockHeaderInfo (hSrio, &extFeatPtr, &extFeatID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetMaintBlockHeaderInfo
(
    CSL_SrioHandle  hSrio, 
    Uint16*         extFeatPtr,
    Uint16*         extFeatID
)
{
    Uint32 value = hSrio->RIO_SP_MB_HEAD;
    *extFeatPtr = CSL_FEXT (value, SRIO_RIO_SP_MB_HEAD_EF_PTR);
    *extFeatID  = CSL_FEXT (value, SRIO_RIO_SP_MB_HEAD_EF_ID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortLinkTimeoutCSR
 *
 *   @b Description
 *   @n The functions gets the port link timeout. This timeout is for link events
 *      such as sending a packet to receiving the corresponding acknowledge, and 
 *      sending a link-request to receiving the corresponding link-response
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          timeout        Timeout value populated by this API
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
 *   @n SRIO_RIO_SP_LT_CTL_TVAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          timeout;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the port link timeout
        CSL_SRIO_GetPortLinkTimeoutCSR (hSrio, &timeout);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortLinkTimeoutCSR
(
    CSL_SrioHandle  hSrio, 
    Uint32*         timeout
)
{
    *timeout = CSL_FEXT (hSrio->RIO_SP_LT_CTL, SRIO_RIO_SP_LT_CTL_TVAL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortLinkTimeoutCSR
 *
 *   @b Description
 *   @n The functions sets the port link timeout.
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          timeout        Timeout value to be configured
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
 *   @n SRIO_RIO_SP_LT_CTL_TVAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          timeout;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the port link timeout
        CSL_SRIO_SetPortLinkTimeoutCSR (hSrio, 10);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortLinkTimeoutCSR
(
    CSL_SrioHandle  hSrio, 
    Uint32          timeout
)
{
    CSL_FINS (hSrio->RIO_SP_LT_CTL, SRIO_RIO_SP_LT_CTL_TVAL, timeout);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortResponseTimeoutCSR
 *
 *   @b Description
 *   @n The functions gets the port response timeout. This timeout is for 
 *      sending a packet to receiving the corresponding response packet
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          timeout        Timeout value populated by this API
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
 *   @n SRIO_RIO_SP_RT_CTL_TVAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          timeout;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the port response timeout
        CSL_SRIO_GetPortResponseTimeoutCSR (hSrio, &timeout);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortResponseTimeoutCSR
(
    CSL_SrioHandle  hSrio, 
    Uint32*         timeout
)
{
    *timeout = CSL_FEXT (hSrio->RIO_SP_RT_CTL, SRIO_RIO_SP_RT_CTL_TVAL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortResponseTimeoutCSR
 *
 *   @b Description
 *   @n The functions sets the port response timeout. This timeout is for 
 *      sending a packet to receiving the corresponding response packet
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          timeout        Timeout value to be configured
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
 *   @n SRIO_RIO_SP_RT_CTL_TVAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          timeout;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the port response timeout
        CSL_SRIO_SetPortResponseTimeoutCSR (hSrio, 0xFFFF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortResponseTimeoutCSR
(
    CSL_SrioHandle  hSrio, 
    Uint32          timeout
)
{
    CSL_FINS (hSrio->RIO_SP_RT_CTL, SRIO_RIO_SP_RT_CTL_TVAL, timeout);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortGeneralCSR
 *
 *   @b Description
 *   @n The functions gets the CSR information for all ports. 
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          hostDev        Host /Agent or Slave Device populated by this API
          masterEn       Master Enable populated by this API
          disc           Discovered flag populated by this API
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
 *   @n SRIO_RIO_SP_GEN_CTL_HOST,SRIO_RIO_SP_GEN_CTL_MAST_EN,SRIO_RIO_SP_GEN_CTL_DISC
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle hSrio;
        Uint8          hostDev;
        Uint8          masterEn;
        Uint8          disc;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the port information.
        CSL_SRIO_GetPortGeneralCSR (hSrio, &hostDev, &masterEn, &disc);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortGeneralCSR
(
    CSL_SrioHandle  hSrio, 
    Uint8*          hostDev,
    Uint8*          masterEn,
    Uint8*          disc
)
{
    Uint32 value = hSrio->RIO_SP_GEN_CTL;
    *hostDev  = CSL_FEXT (value, SRIO_RIO_SP_GEN_CTL_HOST);
    *masterEn = CSL_FEXT (value, SRIO_RIO_SP_GEN_CTL_MAST_EN);
    *disc     = CSL_FEXT (value, SRIO_RIO_SP_GEN_CTL_DISC);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortGeneralCSR
 *
 *   @b Description
 *   @n The functions sets the CSR information for all ports. 
 *
 *   @b Arguments
     @verbatim
          hSrio          Handle of the SRIO device
          hostDev        Host /Agent or Slave Device to be configured
          masterEn       Master Enable to be configured
          disc           Discovered flag to be configured
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
 *   @n SRIO_RIO_SP_GEN_CTL_HOST,SRIO_RIO_SP_GEN_CTL_MAST_EN,SRIO_RIO_SP_GEN_CTL_DISC
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the port information.
        CSL_SRIO_SetPortGeneralCSR (hSrio, 1, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortGeneralCSR
(
    CSL_SrioHandle  hSrio, 
    Uint8           hostDev,
    Uint8           masterEn,
    Uint8           disc
)
{
    hSrio->RIO_SP_GEN_CTL = CSL_FMK(SRIO_RIO_SP_GEN_CTL_HOST, hostDev)      |
                            CSL_FMK(SRIO_RIO_SP_GEN_CTL_MAST_EN, masterEn)  |
                            CSL_FMK(SRIO_RIO_SP_GEN_CTL_DISC, disc);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SendPortLinkMaintRequest
 *
 *   @b Description
 *   @n The function sends a port link request control symbol on the specified 
 *      SRIO port.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          cmd               Command to be sent in the link-request control symbol.
                            The following values hold good:-
                                - 0x3 i.e. Reset
                                - 0x4 i.e. Input Status
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
 *   @n SRIO_RIO_SP_LM_REQ_CMD
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Send a maintainance request on port 1 for reset
        CSL_SRIO_SendPortLinkMaintRequest (hSrio, 1, 3);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SendPortLinkMaintRequest
(
    CSL_SrioHandle  hSrio, 
    Uint8           portNum,
    Uint8           cmd
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_LM_REQ, SRIO_RIO_SP_LM_REQ_CMD, cmd);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsLinkResponseReceived
 *
 *   @b Description
 *   @n The function checks if a valid link response has been received to the 
 *      previously sent link request. 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_LM_RESP_RESP_VALID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the Link response received or not? 
        if (CSL_SRIO_IsLinkResponseReceived (hSrio, 1) == TRUE)
        {
            // Link Response was received.
        }
        else
        {
            // Link Response was NOT received 
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsLinkResponseReceived
(
    CSL_SrioHandle  hSrio, 
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_LM_RESP, SRIO_RIO_SP_LM_RESP_RESP_VALID) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLinkResponseStatusInfo
 *
 *   @b Description
 *   @n The function gets the status information about the link response.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          ackIdStatus       AckID status field populated by this API
          linkStatus        Link status field populated by this API
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
 *   @n SRIO_RIO_SP_LM_RESP_ACK_ID_STAT,SRIO_RIO_SP_LM_RESP_LINK_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           ackIdStatus;
        Uint8           linkStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the Link response received or not? 
        if (CSL_SRIO_IsLinkResponseReceived (hSrio, 1) == TRUE)
        {
            // Link Response was received.
            CSL_SRIO_GetLinkResponseStatusInfo (hSrio, 1, &ackIdStatus, &linkStatus);
        }
        else
        {
            // Link Response was NOT received.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLinkResponseStatusInfo
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          ackIdStatus,
    Uint8*          linkStatus
)
{
    Uint32 value = hSrio->RIO_SP[portNum].RIO_SP_LM_RESP;
    *ackIdStatus = CSL_FEXT (value, SRIO_RIO_SP_LM_RESP_ACK_ID_STAT);
    *linkStatus  = CSL_FEXT (value, SRIO_RIO_SP_LM_RESP_LINK_STAT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearOutstandingACK
 *
 *   @b Description
 *   @n The function is used to clear the outstanding acknowledgment ids and is 
 *      used for software assisted ackID realignment.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ACKID_STAT_CLR_OUTSTD_ACKID=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ACKID_STAT_OUTSTD_ACKID=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Clear outstanding ackID for SRIO Port 1
        CSL_SRIO_ClearOutstandingACK (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearOutstandingACK
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ACKID_STAT, SRIO_RIO_SP_ACKID_STAT_CLR_OUTSTD_ACKID, (Uint32)1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetACKStatus
 *
 *   @b Description
 *   @n The function is used to get the ACK status.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          inputAckID        Input port next expected ACK ID 
          outputUnAckID     Output port unacknowledged ackID status
          outputAckID       Output port next transmitted Acknowledge ID value
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
 *   @n SRIO_RIO_SP_ACKID_STAT_INB_ACKID,SRIO_RIO_SP_ACKID_STAT_OUTSTD_ACKID,SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           inputAckID;
        Uint8           outputUnAckID;
        Uint8           outputAckID;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the ACK Status for Port 1
        CSL_SRIO_GetACKStatus (hSrio, 1, &inputAckID, &outputUnAckID, &outputAckID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetACKStatus
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          inputAckID,
    Uint8*          outputUnAckID,
    Uint8*          outputAckID
)
{
    Uint32 value = hSrio->RIO_SP[portNum].RIO_SP_ACKID_STAT;

    *inputAckID     = CSL_FEXT (value, SRIO_RIO_SP_ACKID_STAT_INB_ACKID);
    *outputUnAckID  = CSL_FEXT (value, SRIO_RIO_SP_ACKID_STAT_OUTSTD_ACKID);
    *outputAckID    = CSL_FEXT (value, SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetACKStatus
 *
 *   @b Description
 *   @n The function is used to set the ACK status.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          inputAckID        Input port next expected ACK ID 
          outputUnAckID     Output port unacknowledged ackID status
          outputAckID       Output port next transmitted Acknowledge ID value
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
 *   @n SRIO_RIO_SP_ACKID_STAT_INB_ACKID, SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the ACK Status for Port 1
        CSL_SRIO_SetACKStatus (hSrio, 1, 100, 205);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetACKStatus
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8           inputAckID,
    Uint8           outputAckID
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ACKID_STAT, SRIO_RIO_SP_ACKID_STAT_INB_ACKID, inputAckID);
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ACKID_STAT, SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID, outputAckID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortBaudRate
 *
 *   @b Description
 *   @n The function is used to get the initialized baud rate of the port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          baudSel           Baud Rate Selection populated by this API
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
 *   @n SRIO_RIO_SP_CTL2_BAUD_SEL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           baudSel;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port 1 Baud Rate 
        CSL_SRIO_GetPortBaudRate (hSrio, 1, &baudSel);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortBaudRate
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          baudSel
)
{
    *baudSel = CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_BAUD_SEL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsAutomaticBaudRateDiscoverySupported
 *
 *   @b Description
 *   @n The function is used to check if automatic baud rate discovery is 
 *      supported or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_BAUD_DISC
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is Port1 Automatic Baud Rate Discovery supported
        if (CSL_SRIO_IsAutomaticBaudRateDiscoverySupported (hSrio, 1) == TRUE)
        {
            // Port1 Automatic Baud Rate Discovery supported
        }
        else
        {
            // Port1 Automatic Baud Rate Discovery NOT supported
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsAutomaticBaudRateDiscoverySupported
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_BAUD_DISC) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_Is1_25GBaudSupported
 *
 *   @b Description
 *   @n The function is used to check if 1.25 GBaud is supported or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - 1.25GBaud is supported
 *   @n FALSE - 1.25GBaud is not supported
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL2_GB_1P25
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if 1.25Gbaud is supported or not?
        if (CSL_SRIO_Is1_25GBaudSupported (hSrio, 1) == TRUE)
        {
            // 1.25Gbaud is supported
        }
        else
        {
            // 1.25Gbaud is not supported
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_Is1_25GBaudSupported
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_1P25) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_Enable1_25GBaud
 *
 *   @b Description
 *   @n The function is used to enable 1.25GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_1P25_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Port1 1.25G Baud
        CSL_SRIO_Enable1_25GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Enable1_25GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_1P25_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Disable1_25GBaud
 *
 *   @b Description
 *   @n The function is used to enable 1.25GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_1P25_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Port1 1.25G Baud
        CSL_SRIO_Disable1_25GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Disable1_25GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_1P25_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Is2_5GBaudSupported
 *
 *   @b Description
 *   @n The function is used to check if 2.5GBaud is supported or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - 2.5GBaud is supported
 *   @n FALSE - 2.5GBaud is not supported
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL2_GB_2P5
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if 2.5Gbaud is supported or not?
        if (CSL_SRIO_Is2_5GBaudSupported (hSrio, 1) == TRUE)
        {
            // 2.5Gbaud is supported
        }
        else
        {
            // 2.5Gbaud is not supported
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_Is2_5GBaudSupported
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_2P5) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_Enable2_5GBaud
 *
 *   @b Description
 *   @n The function is used to enable 2.5GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_2P5_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Port1 2.5GBaud
        CSL_SRIO_Enable2_5GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Enable2_5GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_2P5_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Disable2_5GBaud
 *
 *   @b Description
 *   @n The function is used to disable 2.5GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_2P5_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Port1 2.5GBaud
        CSL_SRIO_Disable2_5GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Disable2_5GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_2P5_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Is3_125GBaudSupported
 *
 *   @b Description
 *   @n The function is used to check if 3.125GBaud is supported or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - 3.125GBaud is supported
 *   @n FALSE - 3.125GBaud is not supported
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL2_GB_3P125
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if 3.125Gbaud is supported or not?
        if (CSL_SRIO_Is3_125GBaudSupported (hSrio, 1) == TRUE)
        {
            // 3.125Gbaud is supported
        }
        else
        {
            // 3.125Gbaud is not supported
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_Is3_125GBaudSupported
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_3P125) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_Enable3_125GBaud
 *
 *   @b Description
 *   @n The function is used to enable 3.125GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_3P125_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Port1 3.125GBaud
        CSL_SRIO_Enable3_125GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Enable3_125GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_3P125_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Disable3_125GBaud
 *
 *   @b Description
 *   @n The function is used to disable 3.125GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_3P125_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Port1 3.125GBaud
        CSL_SRIO_Disable3_125GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Disable3_125GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_3P125_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Is5GBaudSupported
 *
 *   @b Description
 *   @n The function is used to check if 5GBaud is supported or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - 5GBaud is supported
 *   @n FALSE - 5GBaud is not supported
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL2_GB_5P0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if 5Gbaud is supported or not?
        if (CSL_SRIO_Is5GBaudSupported (hSrio, 1) == TRUE)
        {
            // 5Gbaud is supported
        }
        else
        {
            // 5Gbaud is not supported
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_Is5GBaudSupported
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_5P0) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_Enable5GBaud
 *
 *   @b Description
 *   @n The function is used to enable 5GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_5P0_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Port1 5GBaud
        CSL_SRIO_Enable5GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Enable5GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_5P0_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Disable5GBaud
 *
 *   @b Description
 *   @n The function is used to disable 5GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_5P0_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Port1 5GBaud
        CSL_SRIO_Disable5GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Disable5GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_5P0_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Is6_25GBaudSupported
 *
 *   @b Description
 *   @n The function is used to check if 6_25GBaud is supported or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - 6_25GBaud is supported
 *   @n FALSE - 6_25GBaud is not supported
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL2_GB_6P25
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if 6_25Gbaud is supported or not?
        if (CSL_SRIO_Is6_25GBaudSupported (hSrio, 1) == TRUE)
        {
            // 6_25Gbaud is supported
        }
        else
        {
            // 6_25Gbaud is not supported
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_Is6_25GBaudSupported
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_6P25) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_Enable6_25GBaud
 *
 *   @b Description
 *   @n The function is used to enable 6.25GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_6P25_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Port1 6.25GBaud
        CSL_SRIO_Enable6_25GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Enable6_25GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_6P25_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_Disable6_25GBaud
 *
 *   @b Description
 *   @n The function is used to disable 6.25GBaud for the specified port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_GB_6P25_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Port1 6.25GBaud
        CSL_SRIO_Disable6_25GBaud (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_Disable6_25GBaud
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_GB_6P25_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableDataScrambling
 *
 *   @b Description
 *   @n The function is used to enable data scrambling for the specific port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_D_SCRM_DIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable Port1 Data Scrambling
        CSL_SRIO_EnableDataScrambling (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableDataScrambling
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_D_SCRM_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableDataScrambling
 *
 *   @b Description
 *   @n The function is used to disable data scrambling for the specific port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_D_SCRM_DIS=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable Port1 Data Scrambling
        CSL_SRIO_DisableDataScrambling (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableDataScrambling
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_D_SCRM_DIS, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsRemoteTxEmphasisEnabled
 *
 *   @b Description
 *   @n The function is used to check if the remote transmit emphasis is enabled
 *      or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL2_RTEC
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if remote Transmit Emphasis is enabled or not?
        if (CSL_SRIO_IsRemoteTxEmphasisEnabled (hSrio, 1) == TRUE)
        {
            // Remote Transmit Emphasis is enabled
        }
        else
        {
            // Remote Transmit Emphasis is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsRemoteTxEmphasisEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL2, SRIO_RIO_SP_CTL2_RTEC) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsOutputPktDropped
 *
 *   @b Description
 *   @n The function is used to check if the output port has discarded a packet
 *      or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has discarded a packet or not?
        if (CSL_SRIO_IsOutputPktDropped (hSrio, 1) == TRUE)
        {
            // Port1 has discarded the packet
        }
        else
        {
            // Port1 has NOT discarded the packet
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsOutputPktDropped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearOutputPktDropped
 *
 *   @b Description
 *   @n The function is used to clear the status of the output port after the 
 *      output port discarded condition has been detected
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has discarded a packet or not?
        if (CSL_SRIO_IsOutputPktDropped (hSrio, 1) == TRUE)
        {
            // Port1 has discarded the packet
            ...
            // Clear this error condition
            CSL_SRIO_ClearOutputPktDropped (hSrio, 1);
        }
        else
        {
            // Port1 has NOT discarded the packet
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearOutputPktDropped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsOutputFailed
 *
 *   @b Description
 *   @n The function is used to check if the output port has encountered a failed
 *      condition.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT failed condition?
        if (CSL_SRIO_IsOutputFailed (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT failed condition
        }
        else
        {
            // Port1 has NOT detected an OUTPUT failed condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsOutputFailed
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearOutputFailed
 *
 *   @b Description
 *   @n The function is used to clear the status of the output port after a
 *   failed condition was detected
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected a failed condition or not? 
        if (CSL_SRIO_IsOutputFailed (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT failed condition
            ...
            // Clear this error condition
            CSL_SRIO_ClearOutputFailed (hSrio, 1);
        }
        else
        {
            // Port1 has NOT detected an OUTPUT failed condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearOutputFailed
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsOutputDegraded
 *
 *   @b Description
 *   @n The function is used to check if the output port has encountered a degraded
 *      condition or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT degraded condition?
        if (CSL_SRIO_IsOutputDegraded (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT degraded condition
        }
        else
        {
            // Port1 has NOT detected an OUTPUT degraded condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsOutputDegraded
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearOutputDegrade
 *
 *   @b Description
 *   @n The function is used to clear the status of the output port after a
 *   failed condition was detected
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT degraded condition?
        if (CSL_SRIO_IsOutputDegraded (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT degraded condition
            ...
            // Clear this error condition
            CSL_SRIO_ClearOutputDegrade (hSrio, 1);
        }
        else
        {
            // Port1 has NOT detected an OUTPUT degraded condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearOutputDegrade
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsOutputRetryStopped
 *
 *   @b Description
 *   @n The function is used to check if the output port has received a packet
 *      retry control symbol.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_STOPPED
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT Retry condition?
        if (CSL_SRIO_IsOutputRetryStopped (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT Retry condition
        }
        else
        {
            // Port1 has NOT detected an OUTPUT Retry condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsOutputRetryStopped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_STOPPED) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearOutputRetry
 *
 *   @b Description
 *   @n The function is used to clear the status of the output port after the output
 *      retry condition was detected.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_STOPPED=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT Retry condition?
        if (CSL_SRIO_IsOutputRetryStopped (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT Retry condition
            ...
            // Clear this error condition
            CSL_SRIO_ClearOutputRetry (hSrio, 1);
        }
        else
        {
            // Port1 has NOT detected an OUTPUT Retry condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearOutputRetry
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsOutputErrorStopped
 *
 *   @b Description
 *   @n The function is used to check if the output port has detected an error
 *      and is in the output error stopped state
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_STOPPED
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT Error stopped condition?
        if (CSL_SRIO_IsOutputErrorStopped (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT Error stopped condition
        }
        else
        {
            // Port1 has NOT detected an OUTPUT Error stopped condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsOutputErrorStopped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_STOPPED) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearOutputErrorStopped
 *
 *   @b Description
 *   @n The function is used to clear the status of the output port after the output
 *      error stopped condition was detected.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_ENCTR=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_STOPPED=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an OUTPUT Error stopped condition?
        if (CSL_SRIO_IsOutputErrorStopped (hSrio, 1) == TRUE)
        {
            // Port1 has detected an OUTPUT Error stopped condition
            ...
            // Clear this error condition
            CSL_SRIO_ClearOutputErrorStopped (hSrio, 1);
        }
        else
        {
            // Port1 has NOT detected an OUTPUT Error stopped condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearOutputErrorStopped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsInputErrorStopped
 *
 *   @b Description
 *   @n The function is used to check if the port has detected an error
 *      and is in the input error stopped state
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_INPUT_ERR_STOPPED
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an INPUT Error stopped condition?
        if (CSL_SRIO_IsInputErrorStopped (hSrio, 1) == TRUE)
        {
            // Port1 has detected an INPUT Error stopped condition
        }
        else
        {
            // Port1 has NOT detected an INPUT Error stopped condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsInputErrorStopped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_INPUT_ERR_STOPPED) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearInputErrorStopped
 *
 *   @b Description
 *   @n The function is used to clear the status of the input port after the input
 *      error stopped condition was detected.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_INPUT_ERR_ENCTR=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_INPUT_ERR_STOPPED=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected an INPUT Error stopped condition?
        if (CSL_SRIO_IsInputErrorStopped (hSrio, 1) == TRUE)
        {
            // Port1 has detected an INPUT Error stopped condition
            ...
            // Clear this error condition
            CSL_SRIO_ClearInputErrorStopped (hSrio, 1);
        }
        else
        {
            // Port1 has NOT detected an INPUT Error stopped condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearInputErrorStopped
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_INPUT_ERR_ENCTR, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortWritePending
 *
 *   @b Description
 *   @n The function is used to check if the port has encountered a condition 
 *      which required it to an issue an I/O logical port-write maintainance 
 *      request
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected a port write pending condition?
        if (CSL_SRIO_IsPortWritePending (hSrio, 1) == TRUE)
        {
            // Port1 has detected a port write pending condition
        }
        else
        {
            // Port1 has NOT detected a port write pending condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortWritePending
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPortWritePending
 *
 *   @b Description
 *   @n The function is used to clear the port write pending condition
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND=1
 *
 *   @b Affects
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 has detected a port write pending condition?
        if (CSL_SRIO_IsPortWritePending (hSrio, 1) == TRUE)
        {
            // Port1 has detected a port write pending condition
            ...
            // Clear this error condition
            CSL_SRIO_ClearInputErrorStopped (hSrio, 1);
        }
        else
        {
            // Port1 has NOT detected a port write pending condition
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPortWritePending
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortAvailable
 *
 *   @b Description
 *   @n The function is used to check if the port is available?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_UNAVL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 is available or not?
        if (CSL_SRIO_IsPortAvailable (hSrio, 1) == TRUE)
        {
            // Port1 is available
        }
        else
        {
            // Port1 is not available
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortAvailable
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_PORT_UNAVL) == 0)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortFatalErrorDetected
 *
 *   @b Description
 *   @n The function is used to check if the port has detected an error from which 
 *      the hardware was unable to recover (a fatal error).
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Port Fatal Error detected
 *   @n FALSE - Port Fatal Error not detected
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_ERR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 fatal error was detected or not?
        if (CSL_SRIO_IsPortFatalErrorDetected (hSrio, 1) == TRUE)
        {
            // Port1 has FATAL Error 
        }
        else
        {
            // Port1 does not have a FATAL Error
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortFatalErrorDetected
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_PORT_ERR) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortOk
 *
 *   @b Description
 *   @n The function is used to check if the port is initialized and can be 
 *      used for exchanging error free control symbols.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Port is operational
 *   @n FALSE - Port is NOT operational
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_OK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 is ok and operational
        if (CSL_SRIO_IsPortOk (hSrio, 1) == TRUE)
        {
            // Port1 is OK and can be used
        }
        else
        {
            // Port1 is NOT OK and cannot be used
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortOk
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_PORT_OK) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortInitialized
 *
 *   @b Description
 *   @n The function is used to check if the port is initialized or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Port is initialized
 *   @n FALSE - Port is NOT initialized
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_ERR_STAT_PORT_UNINIT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if Port1 is initialized or not?
        if (CSL_SRIO_IsPortInitialized (hSrio, 1) == TRUE)
        {
            // Port1 is Initialized
        }
        else
        {
            // Port1 is NOT Initialized
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortInitialized
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_ERR_STAT, SRIO_RIO_SP_ERR_STAT_PORT_UNINIT) == 0)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetSupportedPortWidth
 *
 *   @b Description
 *   @n The function is used to get the supported port width for a specific port
 *      in addition to the 1x mode which is supported by all ports.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portWidth         Port Width populated by this API
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
 *   @n SRIO_RIO_SP_CTL_PORT_WIDTH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portWidth;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the supported port width for port1
        CSL_SRIO_GetSupportedPortWidth(hSrio, 1, &portWidth);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetSupportedPortWidth
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          portWidth
)
{
    *portWidth = CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_WIDTH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetInitializedPortWidth
 *
 *   @b Description
 *   @n The function is used to get the width of the port after initialization.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portWidth         Port Width populated by this API
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
 *   @n SRIO_RIO_SP_CTL_INIT_PWIDTH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portWidth;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the initialized port width for port1
        CSL_SRIO_GetInitializedPortWidth(hSrio, 1, &portWidth);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetInitializedPortWidth
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          portWidth
)
{
    *portWidth = CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_INIT_PWIDTH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetSoftwarePortWidth
 *
 *   @b Description
 *   @n The function is used to get the software port configuration to override
 *      the hardware sizes.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portWidth         Port Width populated by this API
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
 *   @n SRIO_RIO_SP_CTL_OVER_PWIDTH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portWidth;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the software port width override
        CSL_SRIO_GetSoftwarePortWidth(hSrio, 1, &portWidth);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetSoftwarePortWidth
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          portWidth
)
{
    *portWidth = CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_OVER_PWIDTH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetSoftwarePortWidth
 *
 *   @b Description
 *   @n The function is used to set the software port configuration to override
 *      the hardware sizes.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portWidth         Port Width to be configured
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
 *   @n SRIO_RIO_SP_CTL_OVER_PWIDTH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the software port width override to force 2x mode
        CSL_SRIO_SetSoftwarePortWidth(hSrio, 1, 5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetSoftwarePortWidth
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8           portWidth
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_OVER_PWIDTH, portWidth);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePort
 *
 *   @b Description
 *   @n The function is used to enable the specific port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_PORT_DIS=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable port 1
        CSL_SRIO_EnablePort(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePort
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_DIS, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePort
 *
 *   @b Description
 *   @n The function is used to disable the specific port
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_PORT_DIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable port 1
        CSL_SRIO_DisablePort(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePort
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortEnabled
 *
 *   @b Description
 *   @n The function is used to check if the port is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Port is enabled
 *   @n FALSE - Port is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_PORT_DIS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if port 1 is enabled or not?
        if (CSL_SRIO_IsPortEnabled(hSrio, 1) == TRUE)
        {
            // Port1 is enabled
        }
        else
        {
            // Port1 is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_DIS) == 0)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableOutputPort
 *
 *   @b Description
 *   @n The function is used to enable the output port to issue any packets
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_OTP_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable output port 1
        CSL_SRIO_EnableOutputPort(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableOutputPort
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_OTP_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableOutputPort
 *
 *   @b Description
 *   @n The function is used to disable the output port and is unable to issue
 *      any packets except to route or respond to MAINTENANCE packets.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_OTP_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable output port 1
        CSL_SRIO_DisableOutputPort(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableOutputPort
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_OTP_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsOutputPortEnabled
 *
 *   @b Description
 *   @n The function is used to check if the output port is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Output Port is enabled
 *   @n FALSE - Output Port is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_OTP_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if output port 1 is enabled or not?
        if (CSL_SRIO_IsOutputPortEnabled(hSrio, 1) == TRUE)
        {
            // Output Port1 is enabled
        }
        else
        {
            // Output Port1 is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsOutputPortEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_OTP_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableInputPort
 *
 *   @b Description
 *   @n The function is used to enable the input port to issue any packets
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_INP_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable input port 1
        CSL_SRIO_EnableInputPort(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableInputPort
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_INP_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableInputPort
 *
 *   @b Description
 *   @n The function is used to disable the input port to issue any packets
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_INP_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable input port 1
        CSL_SRIO_DisableInputPort(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableInputPort
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_INP_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsInputPortEnabled
 *
 *   @b Description
 *   @n The function is used to check if the input port is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Input Port is enabled
 *   @n FALSE - Input Port is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_INP_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if input port 1 is enabled or not?
        if (CSL_SRIO_IsInputPortEnabled(hSrio, 1) == TRUE)
        {
            // Input Port1 is enabled
        }
        else
        {
            // Input Port1 is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsInputPortEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_INP_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePortErrorChecking
 *
 *   @b Description
 *   @n The function is used to enable the error checking
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_ERR_DIS=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable error checking for port 1
        CSL_SRIO_EnablePortErrorChecking(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePortErrorChecking
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_ERR_DIS, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePortErrorChecking
 *
 *   @b Description
 *   @n The function is used to disable the error checking
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_ERR_DIS=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable error checking for port 1
        CSL_SRIO_DisablePortErrorChecking(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePortErrorChecking
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_ERR_DIS, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortErrorCheckingEnabled
 *
 *   @b Description
 *   @n The function is used to check if the error checking is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Error Checking is enabled
 *   @n FALSE - Error Checking is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_ERR_DIS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if error checking for port 1 is enabled or not?
        if (CSL_SRIO_IsPortErrorCheckingEnabled(hSrio, 1) == TRUE)
        {
            // Port1 error checking is enabled
        }
        else
        {
            // Port1 error checking is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortErrorCheckingEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_ERR_DIS) == 0)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetMulticastEventParticipant
 *
 *   @b Description
 *   @n The function is used to get the Multicast Event Participant status for the
 *      specific port.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          multicastEvent    Multicast Event Participant status populated by this API
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
 *   @n SRIO_RIO_SP_CTL_MULT_CS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           multicastEvent;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Multicast Event Participant status for Port 1
        CSL_SRIO_GetMulticastEventParticipant(hSrio, 1, &multicastEvent);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetMulticastEventParticipant
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          multicastEvent
)
{
    *multicastEvent = CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_MULT_CS);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetMulticastEventParticipant
 *
 *   @b Description
 *   @n The function is used to set the Multicast Event Participant status for the
 *      specific port.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          multicastEvent    Multicast Event Participant status to be configured
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
 *   @n SRIO_RIO_SP_CTL_MULT_CS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Multicast Event Participant status for Port 1
        CSL_SRIO_SetMulticastEventParticipant(hSrio, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetMulticastEventParticipant
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8           multicastEvent
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_MULT_CS, multicastEvent);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePortFlowControl
 *
 *   @b Description
 *   @n The function is used to enable the port flow control
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_FLOW_CTRL=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable flow control for port 1
        CSL_SRIO_EnablePortFlowControl(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePortFlowControl
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_FLOW_CTRL, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePortFlowControl
 *
 *   @b Description
 *   @n The function is used to disable the port flow control
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_FLOW_CTRL=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable flow control for port 1
        CSL_SRIO_DisablePortFlowControl(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePortFlowControl
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_FLOW_CTRL, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortFlowControlEnabled
 *
 *   @b Description
 *   @n The function is used to check if the flow control on the specific port 
 *      is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Flow Control is enabled
 *   @n FALSE - Flow Control is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_FLOW_CTRL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if flow control for port 1 is enabled or not?
        if (CSL_SRIO_IsPortFlowControlEnabled(hSrio, 1) == TRUE)
        {
            // Port1 flow control is enabled
        }
        else
        {
            // Port1 flow control is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortFlowControlEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_FLOW_CTRL) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePortFlowArbiteration
 *
 *   @b Description
 *   @n The function is used to enable the port flow arbiteration
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_FLOW_ARB=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable flow arbiteration for port 1
        CSL_SRIO_EnablePortFlowArbiteration(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePortFlowArbiteration
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_FLOW_ARB, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePortFlowArbiteration
 *
 *   @b Description
 *   @n The function is used to disable the port flow arbiteration
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_FLOW_ARB=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable flow arbiteration for port 1
        CSL_SRIO_DisablePortFlowArbiteration(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePortFlowArbiteration
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_FLOW_ARB, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortFlowArbiterationEnabled
 *
 *   @b Description
 *   @n The function is used to check if the flow arbiteration on the specific port 
 *      is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Flow Arbiteration is enabled
 *   @n FALSE - Flow Arbiteration is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_FLOW_ARB
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if flow arbiteration for port 1 is enabled or not?
        if (CSL_SRIO_IsPortFlowArbiterationEnabled(hSrio, 1) == TRUE)
        {
            // Port1 flow arbiteration is enabled
        }
        else
        {
            // Port1 flow arbiteration is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortFlowArbiterationEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_FLOW_ARB) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePortStopFail
 *
 *   @b Description
 *   @n The function is used to enable the port stop fail
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_STOP_FAIL_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable stop fail for port 1
        CSL_SRIO_EnablePortStopFail(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePortStopFail
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_STOP_FAIL_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePortStopFail
 *
 *   @b Description
 *   @n The function is used to disable the port stop fail
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_STOP_FAIL_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable stop fail for port 1
        CSL_SRIO_DisablePortStopFail(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePortStopFail
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_STOP_FAIL_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortStopFailEnabled
 *
 *   @b Description
 *   @n The function is used to check if the port stop failed is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Stop flow is enabled
 *   @n FALSE - Stop flow is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_STOP_FAIL_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if stop fail for port 1 is enabled or not?
        if (CSL_SRIO_IsPortStopFailEnabled(hSrio, 1) == TRUE)
        {
            // Port1 stop fail is enabled
        }
        else
        {
            // Port1 stop fail is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortStopFailEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_STOP_FAIL_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePortDrop
 *
 *   @b Description
 *   @n The function is used to enable the port drop
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_DROP_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable drop for port 1
        CSL_SRIO_EnablePortDrop(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePortDrop
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_DROP_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePortDrop
 *
 *   @b Description
 *   @n The function is used to disable the port drop
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_DROP_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable drop for port 1
        CSL_SRIO_DisablePortDrop(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePortDrop
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_DROP_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortDropEnabled
 *
 *   @b Description
 *   @n The function is used to check if the port drop is enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Port drop is enabled
 *   @n FALSE - Port drop is disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_DROP_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if stop fail for port 1 is enabled or not?
        if (CSL_SRIO_IsPortDropEnabled(hSrio, 1) == TRUE)
        {
            // Port1 drop is enabled
        }
        else
        {
            // Port1 drop is disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortDropEnabled
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_DROP_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePortLockout
 *
 *   @b Description
 *   @n The function is used to enable the port lockout
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_PORT_LOCKOUT=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable port lockout for port 1
        CSL_SRIO_EnablePortLockout(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePortLockout
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_LOCKOUT, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePortLockout
 *
 *   @b Description
 *   @n The function is used to disable the port lockout
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_SP_CTL_PORT_LOCKOUT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable port lockout for port 1
        CSL_SRIO_DisablePortLockout(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePortLockout
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_LOCKOUT, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPortLockedout
 *
 *   @b Description
 *   @n The function is used to check if the port is locked out or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Port is Locked out
 *   @n FALSE - Port is NOT Locked out
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_SP_CTL_PORT_LOCKOUT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if port 1 is locked out or not?
        if (CSL_SRIO_IsPortLockedout(hSrio, 1) == TRUE)
        {
            // Port1 is locked out
        }
        else
        {
            // Port1 is NOT locked out
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPortLockedout
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    if (CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PORT_LOCKOUT) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortType
 *
 *   @b Description
 *   @n The function is used to get the port type.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portType          Port Type populated by this API
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
 *   @n SRIO_RIO_SP_CTL_PTYP
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portType;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable port lockout for port 1
        CSL_SRIO_GetPortType(hSrio, 1, &portType);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortType
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          portType
)
{
    *portType = CSL_FEXT(hSrio->RIO_SP[portNum].RIO_SP_CTL, SRIO_RIO_SP_CTL_PTYP);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorReportBlockHeader
 *
 *   @b Description
 *   @n The function is used to get the error report block header information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          efPtr             Extended Feature Pointer populated by this API 
          efId              Extended Feature ID populated by this API 
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
 *   @n SRIO_RIO_ERR_RPT_BH_EF_PTR,SRIO_RIO_ERR_RPT_BH_EF_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint16          efPtr;
        Uint16          efId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Error reporting block header information
        CSL_SRIO_GetErrorReportBlockHeader(hSrio, &efPtr, &efId);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorReportBlockHeader
(
    CSL_SrioHandle  hSrio,
    Uint16*         efPtr,
    Uint16*         efId
)
{
    Uint32 value = hSrio->RIO_ERR_RPT_BH;
    *efPtr = CSL_FEXT(value, SRIO_RIO_ERR_RPT_BH_EF_PTR);
    *efId  = CSL_FEXT(value, SRIO_RIO_ERR_RPT_BH_EF_ID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorDetectCSR
 *
 *   @b Description
 *   @n The function is used to get the error detected CSR information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          errDetect         Error Detect Information populated by this API 
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
 *   @n SRIO_RIO_ERR_DET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errDetect;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the error detect information
        CSL_SRIO_GetErrorDetectCSR(hSrio, &errDetect);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorDetectCSR
(
    CSL_SrioHandle  hSrio,
    Uint32*         errDetect
)
{
    *errDetect = hSrio->RIO_ERR_DET;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetErrorDetectCSR
 *
 *   @b Description
 *   @n The function is used to set the error detected CSR information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          errDetect         Error Detect Information to be cleared
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
 *   @n SRIO_RIO_ERR_DET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errDetect;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the error detect information
        CSL_SRIO_GetErrorDetectCSR(hSrio, &errDetect);
        ...
        // Clear the error detect informatio
        CSL_SRIO_SetErrorDetectCSR(hSrio, errDetect);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetErrorDetectCSR
(
    CSL_SrioHandle  hSrio,
    Uint32          errDetect
)
{
    hSrio->RIO_ERR_DET = errDetect;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorEnable
 *
 *   @b Description
 *   @n The function is used to get the error enable status
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          errEnable         Error Enable Status populated by this API
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
 *   @n SRIO_RIO_ERR_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errEnable;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the error enable status information
        CSL_SRIO_GetErrorEnable(hSrio, &errEnable);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorEnable
(
    CSL_SrioHandle  hSrio,
    Uint32*         errEnable
)
{
    *errEnable = hSrio->RIO_ERR_EN;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetErrorEnable
 *
 *   @b Description
 *   @n The function is used to set the error enable status
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          errEnable         Error Enable Status populated by this API
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
 *   @n SRIO_RIO_ERR_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errEnable;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the error enable status information
        CSL_SRIO_GetErrorEnable(hSrio, &errEnable);
        
        // Ensure that IO Error response is enabled
        errEnable = errEnable | 0x80000000;
        CSL_SRIO_SetErrorEnable(hSrio, errEnable);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetErrorEnable
(
    CSL_SrioHandle  hSrio,
    Uint32          errEnable
)
{
    hSrio->RIO_ERR_EN = errEnable;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetHighAddressCapture
 *
 *   @b Description
 *   @n The function is used to get the MSB 32 bits of the address which
 *      caused the error
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          hiAddress         High Address populated by this API
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
 *   @n SRIO_RIO_H_ADDR_CAPT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          hiAddress;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the high address information
        CSL_SRIO_GetHighAddressCapture(hSrio, &hiAddress);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetHighAddressCapture
(
    CSL_SrioHandle  hSrio,
    Uint32*         hiAddress
)
{
    *hiAddress = hSrio->RIO_H_ADDR_CAPT;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetHighAddressCapture
 *
 *   @b Description
 *   @n The function is used to set the MSB 32 bits of the address which
 *      caused the error
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          hiAddress         High Address to be configured
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
 *   @n SRIO_RIO_H_ADDR_CAPT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Clear the High Address Error Capture 
        CSL_SRIO_SetHighAddressCapture(hSrio, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetHighAddressCapture
(
    CSL_SrioHandle  hSrio,
    Uint32          hiAddress
)
{
    hSrio->RIO_H_ADDR_CAPT = hiAddress;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetAddressCapture
 *
 *   @b Description
 *   @n The function is used to get the LSB 29 bits of the address which
 *      caused the error and also the extended address bits
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          address           Address populated by this API
          xamsbs            Extended Address Information
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
 *   @n SRIO_RIO_ADDR_CAPT_ADDR_LOW,SRIO_RIO_ADDR_CAPT_XAMSBS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          address;
        Uint8           xamsbs;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the address information which caused the error
        CSL_SRIO_GetAddressCapture(hSrio, &hiAddress, &xamsbs);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetAddressCapture
(
    CSL_SrioHandle  hSrio,
    Uint32*         address,
    Uint8*          xamsbs
)
{
    *address = CSL_FEXT(hSrio->RIO_ADDR_CAPT, SRIO_RIO_ADDR_CAPT_ADDR_LOW);
    *xamsbs  = CSL_FEXT(hSrio->RIO_ADDR_CAPT, SRIO_RIO_ADDR_CAPT_XAMSBS);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetAddressCapture
 *
 *   @b Description
 *   @n The function is used to set the LSB 29 bits of the address which
 *      caused the error and also the extended address bits
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          address           Address to be configured
          xamsbs            Extended Address Information
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
 *   @n SRIO_RIO_ADDR_CAPT_ADDR_LOW,SRIO_RIO_ADDR_CAPT_XAMSBS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Clear address information which caused the error
        CSL_SRIO_SetAddressCapture(hSrio, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetAddressCapture
(
    CSL_SrioHandle  hSrio,
    Uint32          address,
    Uint8           xamsbs
)
{
    hSrio->RIO_ADDR_CAPT = CSL_FMK(SRIO_RIO_ADDR_CAPT_ADDR_LOW, address)    |
                           CSL_FMK(SRIO_RIO_ADDR_CAPT_XAMSBS, xamsbs);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetIDCapture
 *
 *   @b Description
 *   @n The function is used to get the source and destination id associated
 *   with the error
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          msbdstId          MSB Destination ID
          lsbdstID          LSB Destination ID
          msbsrcId          MSB Source ID
          lsbsrcId          LSB Source ID          
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
 *   @n SRIO_RIO_ID_CAPT_MSB_DSTID,  SRIO_RIO_ID_CAPT_DSTID,
 *   @n SRIO_RIO_ID_CAPT_MSB_SRCTID, SRIO_RIO_ID_CAPT_SRCID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           msbdstId;
        Uint8           lsbdstID;
        Uint8           msbsrcId;
        Uint8           lsbsrcId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the address capture information which caused the error
        CSL_SRIO_GetIDCapture(hSrio, &msbdstId, &lsbdstID, &msbsrcId, &lsbsrcId);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetIDCapture
(
    CSL_SrioHandle  hSrio,
    Uint8*          msbdstId,
    Uint8*          lsbdstId,
    Uint8*          msbsrcId,
    Uint8*          lsbsrcId
)
{
    Uint32 value = hSrio->RIO_ID_CAPT;

    *msbdstId = CSL_FEXT (value, SRIO_RIO_ID_CAPT_MSB_DSTID);
    *lsbdstId = CSL_FEXT (value, SRIO_RIO_ID_CAPT_DSTID);
    *msbsrcId = CSL_FEXT (value, SRIO_RIO_ID_CAPT_MSB_SRCTID);
    *lsbsrcId = CSL_FEXT (value, SRIO_RIO_ID_CAPT_SRCID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetIDCapture
 *
 *   @b Description
 *   @n The function is used to set the source and destination id associated
 *   with the error
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          msbdstId          MSB Destination ID
          lsbdstID          LSB Destination ID
          msbsrcId          MSB Source ID
          lsbsrcId          LSB Source ID          
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
 *   @n SRIO_RIO_ID_CAPT_MSB_DSTID,  SRIO_RIO_ID_CAPT_DSTID,
 *   @n SRIO_RIO_ID_CAPT_MSB_SRCTID, SRIO_RIO_ID_CAPT_SRCID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           msbdstId;
        Uint8           lsbdstID;
        Uint8           msbsrcId;
        Uint8           lsbsrcId;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Clear the address capture information
        CSL_SRIO_SetIDCapture(hSrio, 0x0, 0x0, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetIDCapture
(
    CSL_SrioHandle  hSrio,
    Uint8           msbdstId,
    Uint8           lsbdstId,
    Uint8           msbsrcId,
    Uint8           lsbsrcId
)
{
    hSrio->RIO_ID_CAPT = CSL_FMK (SRIO_RIO_ID_CAPT_MSB_DSTID, msbdstId)    |
                         CSL_FMK (SRIO_RIO_ID_CAPT_DSTID,     lsbdstId)    |
                         CSL_FMK (SRIO_RIO_ID_CAPT_MSB_SRCTID,msbsrcId)    |
                         CSL_FMK (SRIO_RIO_ID_CAPT_SRCID,     lsbsrcId);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetControlCapture
 *
 *   @b Description
 *   @n The function is used to get the control information associated with 
 *   the error
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ftype             FTPYE associated with the error
          ttype             TTPYE associated with the error
          msgInfo           Message Information associated with the error
          implSpecific      Implementation Specific information
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
 *   @n SRIO_RIO_CTRL_CAPT_FTYPE,  SRIO_RIO_CTRL_CAPT_TTYPE,
 *   @n SRIO_RIO_CTRL_CAPT_MSG_INFO, SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           ftype;
        Uint8           ttype;
        Uint8           msgInfo;
        Uint16          implSpecific;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the control capture information which caused the error
        CSL_SRIO_GetControlCapture(hSrio, &ftype, &ttype, &msgInfo, &implSpecific);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetControlCapture
(
    CSL_SrioHandle  hSrio,
    Uint8*          ftype,
    Uint8*          ttype,
    Uint8*          msgInfo,
    Uint16*         implSpecific
)
{
    Uint32 value = hSrio->RIO_CTRL_CAPT;

    *ftype          = CSL_FEXT (value, SRIO_RIO_CTRL_CAPT_FTYPE);
    *ttype          = CSL_FEXT (value, SRIO_RIO_CTRL_CAPT_TTYPE);
    *msgInfo        = CSL_FEXT (value, SRIO_RIO_CTRL_CAPT_MSG_INFO);
    *implSpecific   = CSL_FEXT (value, SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetControlCapture
 *
 *   @b Description
 *   @n The function is used to set the control information associated with 
 *   the error
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ftype             FTPYE associated with the error
          ttype             TTPYE associated with the error
          msgInfo           Message Information associated with the error
          implSpecific      Implementation Specific information
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
 *   @n SRIO_RIO_CTRL_CAPT_FTYPE,  SRIO_RIO_CTRL_CAPT_TTYPE,
 *   @n SRIO_RIO_CTRL_CAPT_MSG_INFO, SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Clear the control capture information which caused the error
        CSL_SRIO_SetControlCapture(hSrio, 0x0, 0x0, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetControlCapture
(
    CSL_SrioHandle  hSrio,
    Uint8           ftype,
    Uint8           ttype,
    Uint8           msgInfo,
    Uint16          implSpecific
)
{
    hSrio->RIO_CTRL_CAPT = CSL_FMK (SRIO_RIO_CTRL_CAPT_FTYPE, ftype)            |
                           CSL_FMK (SRIO_RIO_CTRL_CAPT_TTYPE, ttype)            |
                           CSL_FMK (SRIO_RIO_CTRL_CAPT_MSG_INFO, msgInfo)       |
                           CSL_FMK (SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC, implSpecific);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortWriteDeviceId
 *
 *   @b Description
 *   @n The function is used to get the target device ID to be used when a 
 *      device generates a Maintenance Port-Write operation to report errors to 
 *      a system host.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          msbTargetId       Most significant byte of Port-Write Target device ID
          lsbTargetID       Least significant byte of Port-Write Target device ID
          lrgTrans          DeviceID size to use for a port-write
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
 *   @n SRIO_RIO_PW_TGT_ID_DEVICEID_MSB,SRIO_RIO_PW_TGT_ID_DEVICEID,
 *   @n SRIO_RIO_PW_TGT_ID_ID_LARGE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           msbTargetId;
        Uint8           lsbTargetID;
        Uint8           lrgTrans;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Target Device ID information.
        CSL_SRIO_GetPortWriteDeviceId(hSrio, &msbTargetId, &lsbTargetID, &lrgTrans);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortWriteDeviceId
(
    CSL_SrioHandle  hSrio,
    Uint8*          msbTargetId,
    Uint8*          lsbTargetID,
    Uint8*          lrgTrans
)
{
    Uint32 value = hSrio->RIO_PW_TGT_ID;
    *msbTargetId = CSL_FEXT(value, SRIO_RIO_PW_TGT_ID_DEVICEID_MSB);
    *lsbTargetID = CSL_FEXT(value, SRIO_RIO_PW_TGT_ID_DEVICEID);
    *lrgTrans    = CSL_FEXT(value, SRIO_RIO_PW_TGT_ID_ID_LARGE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortWriteDeviceId
 *
 *   @b Description
 *   @n The function is used to get the target device ID to be used when a 
 *      device generates a Maintenance Port-Write operation to report errors to 
 *      a system host.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          msbTargetId       Most significant byte of Port-Write Target device ID
          lsbTargetID       Least significant byte of Port-Write Target device ID
          lrgTrans          DeviceID size to use for a port-write
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
 *   @n SRIO_RIO_PW_TGT_ID_DEVICEID_MSB,SRIO_RIO_PW_TGT_ID_DEVICEID,
 *   @n SRIO_RIO_PW_TGT_ID_ID_LARGE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Port Write Target Device ID information.
        CSL_SRIO_SetPortWriteDeviceId (hSrio, 0x0, 0x80, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortWriteDeviceId
(
    CSL_SrioHandle  hSrio,
    Uint8           msbTargetId,
    Uint8           lsbTargetID,
    Uint8           lrgTrans
)
{
    hSrio->RIO_PW_TGT_ID = CSL_FMK (SRIO_RIO_PW_TGT_ID_DEVICEID_MSB, msbTargetId) |
                           CSL_FMK (SRIO_RIO_PW_TGT_ID_DEVICEID,     lsbTargetID) |
                           CSL_FMK (SRIO_RIO_PW_TGT_ID_ID_LARGE,     lrgTrans);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortError
 *
 *   @b Description
 *   @n The function is used to get the physical layer errors that have been 
 *      detected by the Port hardware
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
          errStatus         Error Status populated by this API
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
 *   @n SRIO_RIO_SP_ERR_DET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port 1 Error Status
        CSL_SRIO_GetPortError(hSrio, 1, &errStatus);
        if (errStatus != 0)
        {
            // Port Errors detected
        }
        else
        {
            // No Port Errors detected
        }        
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint32*         errStatus
)
{
    *errStatus = hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_DET;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPortError
 *
 *   @b Description
 *   @n The function is used to clear any port errors which were detected by 
 *      the port Physical layer.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_ERR_DET=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port 1 Error Status
        CSL_SRIO_GetPortError(hSrio, 1, &errStatus);
        if (errStatus != 0)
        {
            // Port Errors detected
            ...
            // Clear port errors.
            CSL_SRIO_ClearPortError (hSrio, 1);
        }
        else
        {
            // No port errors detected 
        }
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_DET = 0x0;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableImpSpecificPortError
 *
 *   @b Description
 *   @n The function is used to enable the ability for the SRIO port to rate count
 *      implementation specific errors.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the Implementation specific Port 1 Rate counting
        CSL_SRIO_EnableImpSpecificPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableImpSpecificPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN, (Uint32)1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableImpSpecificPortError
 *
 *   @b Description
 *   @n The function is used to disable the ability for the SRIO port to rate count
 *      implementation specific errors.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the Implementation specific Port 1 Rate counting
        CSL_SRIO_DisableImpSpecificPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableImpSpecificPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableControlSymBadCRCPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for Received 
 *      Control Symbol with a bad CRC
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_CRC_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the Control Symbol Bad CRC rate counting. 
        CSL_SRIO_EnableControlSymBadCRCPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableControlSymBadCRCPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_CRC_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableControlSymBadCRCPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for Received 
 *      Control Symbol with a bad CRC
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_CRC_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the Control Symbol Bad CRC rate counting. 
        CSL_SRIO_DisableControlSymBadCRCPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableControlSymBadCRCPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_CRC_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableUnexpectedAckIDPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for Received 
 *      Control Symbol with an unexpected ACK ID
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the unexpected ACK ID Port Error
        CSL_SRIO_EnableUnexpectedAckIDPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableUnexpectedAckIDPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableUnexpectedAckIDPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for Received 
 *      Control Symbol with an unexpected ACK ID
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the unexpected ACK ID Port Error
        CSL_SRIO_DisableUnexpectedAckIDPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableUnexpectedAckIDPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePacketNotAcceptedPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for packet not
 *      accepted control symbol.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the packet not accepted port error
        CSL_SRIO_EnablePacketNotAcceptedPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePacketNotAcceptedPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePacketNotAcceptedPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for packet not
 *      accepted control symbol.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the packet not accepted port error
        CSL_SRIO_DisablePacketNotAcceptedPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePacketNotAcceptedPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePacketUnexpectedAckIdError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for packets with
 *      unexpected ack id
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the packet with unexpected ack id rate counting
        CSL_SRIO_EnablePacketUnexpectedAckIdError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePacketUnexpectedAckIdError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePacketUnexpectedAckIdError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for packets with
 *      unexpected ack id
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the packet with unexpected ack id rate counting
        CSL_SRIO_DisablePacketUnexpectedAckIdError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePacketUnexpectedAckIdError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePacketBadCRCError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for packets with
 *      bad CRC
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the packet with bad CRC error rate counting
        CSL_SRIO_EnablePacketBadCRCError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePacketBadCRCError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePacketBadCRCError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for packets with
 *      bad CRC
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the packet with bad CRC error rate counting
        CSL_SRIO_DisablePacketBadCRCError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePacketBadCRCError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableIllegalSizePortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for packets with
 *      illegal size
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the rate counting of packets with illegal size error 
        CSL_SRIO_EnableIllegalSizePortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableIllegalSizePortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableIllegalSizePortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for packets with
 *      illegal size
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the rate counting of packets with illegal size error 
        CSL_SRIO_DisableIllegalSizePortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableIllegalSizePortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableDescramblingLOSPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting of the loss of 
 *      receiver de-scrambler synchronization when control symbol and packet data 
 *      is being scrambled before transmission.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the error rate counting for the LOS Descrambling
        CSL_SRIO_EnableDescramblingLOSPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableDescramblingLOSPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableDescramblingLOSPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting of the loss of 
 *      receiver de-scrambler synchronization when control symbol and packet data 
 *      is being scrambled before transmission.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the error rate counting for the LOS Descrambling
        CSL_SRIO_DisableDescramblingLOSPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableDescramblingLOSPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableIllegalAckIDPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for receiving an
 *      ACK ID that is not outstanding. 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the error rate counting for the illegal ACK ID.
        CSL_SRIO_EnableIllegalAckIDPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableIllegalAckIDPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableIllegalAckIDPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for receiving an
 *      ACK ID that is not outstanding. 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the error rate counting for the illegal ACK ID.
        CSL_SRIO_DisableIllegalAckIDPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableIllegalAckIDPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableProtocolPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for protocol 
 *      errors
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PROT_ERR_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the error rate counting for protocol errors.
        CSL_SRIO_EnableProtocolPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableProtocolPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PROT_ERR_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableProtocolPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for protocol 
 *      errors
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_PROT_ERR_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the error rate counting for protocol errors.
        CSL_SRIO_DisableProtocolPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableProtocolPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_PROT_ERR_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableDelineationPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for delineation 
 *      errors
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the error rate counting for delineation errors.
        CSL_SRIO_EnableDelineationPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableDelineationPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableDelineationPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for delineation 
 *      errors
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the error rate counting for delineation errors.
        CSL_SRIO_DisableDelineationPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableDelineationPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableIllegalCSPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for illegal control
 *      symbols.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the error rate counting for illegal control symbols.
        CSL_SRIO_EnableIllegalCSPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableIllegalCSPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableIllegalCSPortError
 *
 *   @b Description
 *   @n The function is used to disable the error rate counting for illegal control
 *      symbols.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the error rate counting for illegal control symbols.
        CSL_SRIO_DisableIllegalCSPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableIllegalCSPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableLinkTimeoutPortError
 *
 *   @b Description
 *   @n The function is used to enable the error rate counting for link responses
 *      not received within the specific timeout.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
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
 *   @n SRIO_RIO_SP_RATE_EN_LINK_TO_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the error rate counting for link timeout errors.
        CSL_SRIO_EnableLinkTimeoutPortError (hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableLinkTimeoutPortError
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum
)
{
    CSL_FINS (hSrio->RIO_SP_ERR[portNum].RIO_SP_RATE_EN, SRIO_RIO_SP_RATE_EN_LINK_TO_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorCaptureInfo
 *
 *   @b Description
 *   @n The function is used to get the error capture information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
          infoType          Error Information Type
          errType           Encoded Value of the the captured error bits
          implDep           Implementation Dependent Error Information
          valCapt           Capture Valid Information
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
 *   @n SRIO_RIO_SP_ERR_ATTR_CAPT_INFO_TYPE,SRIO_RIO_SP_ERR_ATTR_CAPT_ERR_TYPE,SRIO_RIO_SP_ERR_ATTR_CAPT_IMPL_DEP,SRIO_RIO_SP_ERR_ATTR_CAPT_VAL_CAPT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           infoType;
        Uint8           errType;
        Uint32          implDep;
        Uint8           valCapt;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the error capture information
        CSL_SRIO_GetErrorCaptureInfo (hSrio, 1, &infoType, &errType, &implDep, &valCapt);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorCaptureInfo
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          infoType,
    Uint8*          errType,
    Uint32*         implDep,
    Uint8*          valCapt
)
{
    *infoType = CSL_FEXT(hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_ATTR_CAPT, SRIO_RIO_SP_ERR_ATTR_CAPT_INFO_TYPE);
    *errType  = CSL_FEXT(hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_ATTR_CAPT, SRIO_RIO_SP_ERR_ATTR_CAPT_ERR_TYPE);
    *implDep  = CSL_FEXT(hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_ATTR_CAPT, SRIO_RIO_SP_ERR_ATTR_CAPT_IMPL_DEP);
    *valCapt  = CSL_FEXT(hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_ATTR_CAPT, SRIO_RIO_SP_ERR_ATTR_CAPT_VAL_CAPT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorCapture
 *
 *   @b Description
 *   @n The function is used to get the error capture information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
          errCap0           Error 0 Capture Information 
          errCap1           Error 1 Capture Information 
          errCap2           Error 2 Capture Information 
          errCap3           Error 3 Capture Information 
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
 *   @n SRIO_RIO_SP_ERR_CAPT_0,SRIO_RIO_SP_ERR_CAPT_1,
 *   @n SRIO_RIO_SP_ERR_CAPT_2,SRIO_RIO_SP_ERR_CAPT_3
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint32          errCap0;
        Uint32          errCap1;
        Uint32          errCap2;
        Uint32          errCap3;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the error capture information
        CSL_SRIO_GetErrorCapture (hSrio, 1, &errCap0, &errCap1, &errCap2, &errCap3);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorCapture
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint32*         errCap0,
    Uint32*         errCap1,
    Uint32*         errCap2,
    Uint32*         errCap3
)
{
    *errCap0 = hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_CAPT_0;
    *errCap1 = hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_CAPT_1;
    *errCap2 = hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_CAPT_2;
    *errCap3 = hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_CAPT_3;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorRateCSR
 *
 *   @b Description
 *   @n The function is used to get the error rate CSR which is used to monitor
 *      and control the reporting of the Port Physical layer errors.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
          ptrErrorRate      Error Rate configuration.
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
 *   @n SRIO_RIO_SP_ERR_RATE_ERR_RB,SRIO_RIO_SP_ERR_RATE_ERR_RR,
 *   @n SRIO_RIO_SP_ERR_RATE_ERR_PEAK,SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_ERR_RATE   errorRate;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Error Rate CSR Information.
        CSL_SRIO_GetErrorRateCSR (hSrio, 1, &errorRate);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorRateCSR
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    SRIO_ERR_RATE*  ptrErrorRate
)
{
    ptrErrorRate->errRateBias     = CSL_FEXT (hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_RATE,SRIO_RIO_SP_ERR_RATE_ERR_RB); 
    ptrErrorRate->errRateRecovery = CSL_FEXT (hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_RATE,SRIO_RIO_SP_ERR_RATE_ERR_RR); 
    ptrErrorRate->peak            = CSL_FEXT (hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_RATE,SRIO_RIO_SP_ERR_RATE_ERR_PEAK); 
    ptrErrorRate->errRateCnt      = CSL_FEXT (hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_RATE,SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetErrorRateCSR
 *
 *   @b Description
 *   @n The function is used to set the error rate CSR which is used to monitor
 *      and control the reporting of the Port Physical layer errors.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number 
          ptrErrorRate      Error Rate configuration.
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
 *   @n SRIO_RIO_SP_ERR_RATE_ERR_RB,SRIO_RIO_SP_ERR_RATE_ERR_RR,
 *   @n SRIO_RIO_SP_ERR_RATE_ERR_PEAK,SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        SRIO_ERR_RATE   errorRate;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Error Rate CSR Information.
        CSL_SRIO_GetErrorRateCSR (hSrio, 1, &errorRate);

        // Modify the configuration to not decrement the error rate counter
        errorRate.errRateBias = 0x0;
        CSL_SRIO_SetErrorRateCSR (hSrio, 1, &errorRate);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetErrorRateCSR
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    SRIO_ERR_RATE*  ptrErrorRate
)
{
    hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_RATE = 
            CSL_FMK (SRIO_RIO_SP_ERR_RATE_ERR_RB,  ptrErrorRate->errRateBias)      |
            CSL_FMK (SRIO_RIO_SP_ERR_RATE_ERR_RR,  ptrErrorRate->errRateRecovery)  |
            CSL_FMK (SRIO_RIO_SP_ERR_RATE_ERR_PEAK,ptrErrorRate->peak)             |
            CSL_FMK (SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT, ptrErrorRate->errRateCnt);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorRateThreshold
 *
 *   @b Description
 *   @n The function is used to get the error rate threshold.
 *
 *   @b Arguments
     @verbatim
          hSrio                     Handle of the SRIO device
          portNum                   SRIO Port Number 
          errRateFailedThreshold    Threshold value for reporting an error condition 
                                    due to possibly broken link
          errRateDegradeThreshold   Threshold value for reporting an error condition 
                                    due to a degrading link.
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
 *   @n SRIO_RIO_SP_ERR_THRESH_ERR_RFT, SRIO_RIO_SP_ERR_THRESH_ERR_RDT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           errRateFailedThreshold;
        Uint8           errRateDegradeThreshold;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Error Rate Threshold Information.
        CSL_SRIO_GetErrorRateThreshold (hSrio, 1, &errRateFailedThreshold, &errRateDegradeThreshold);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorRateThreshold
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8*          errRateFailedThreshold,
    Uint8*          errRateDegradeThreshold
)
{
    *errRateFailedThreshold  = CSL_FEXT (hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_THRESH, SRIO_RIO_SP_ERR_THRESH_ERR_RFT);
    *errRateDegradeThreshold = CSL_FEXT (hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_THRESH, SRIO_RIO_SP_ERR_THRESH_ERR_RDT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetErrorRateThreshold
 *
 *   @b Description
 *   @n The function is used to set the error rate threshold.
 *
 *   @b Arguments
     @verbatim
          hSrio                     Handle of the SRIO device
          portNum                   SRIO Port Number 
          errRateFailedThreshold    Threshold value for reporting an error condition 
                                    due to possibly broken link
          errRateDegradeThreshold   Threshold value for reporting an error condition 
                                    due to a degrading link.
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
 *   @n SRIO_RIO_SP_ERR_THRESH_ERR_RFT, SRIO_RIO_SP_ERR_THRESH_ERR_RDT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Error Rate Threshold Information disabling both failed & 
        // degraded triggers
        CSL_SRIO_SetErrorRateThreshold (hSrio, 1, 0x0, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetErrorRateThreshold
(
    CSL_SrioHandle  hSrio,
    Uint8           portNum,
    Uint8           errRateFailedThreshold,
    Uint8           errRateDegradeThreshold
)
{
    hSrio->RIO_SP_ERR[portNum].RIO_SP_ERR_THRESH = 
            CSL_FMK (SRIO_RIO_SP_ERR_THRESH_ERR_RFT, errRateFailedThreshold) | 
            CSL_FMK (SRIO_RIO_SP_ERR_THRESH_ERR_RDT, errRateDegradeThreshold);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLaneStatus
 *
 *   @b Description
 *   @n The function is used to get the lane status information.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
          ptrLaneStatus         Pointer to the Lane status
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
 *   @n SRIO_RIO_LANE_STAT0_PORT_NUM, SRIO_RIO_LANE_STAT0_LANE_NUM,
 *   @n SRIO_RIO_LANE_STAT0_TX_TYPE, SRIO_RIO_LANE_STAT0_TX_MODE,
 *   @n SRIO_RIO_LANE_STAT0_RX_TYPE, SRIO_RIO_LANE_STAT0_RX_INV,
 *   @n SRIO_RIO_LANE_STAT0_RX_TRN, SRIO_RIO_LANE_STAT0_RX_SYNC, 
 *   @n SRIO_RIO_LANE_STAT0_RX_RDY, SRIO_RIO_LANE_STAT0_ERR_CNT, 
 *   @n SRIO_RIO_LANE_STAT0_CHG_SYNC, SRIO_RIO_LANE_STAT0_CHG_TRN, 
 *   @n SRIO_RIO_LANE_STAT0_STAT1, SRIO_RIO_LANE_STAT0_STAT2_7,
 *   @n SRIO_RIO_LANE_STAT1_IDLE2, SRIO_RIO_LANE_STAT1_INFO_OK,
 *   @n SRIO_RIO_LANE_STAT1_CHG, SRIO_RIO_LANE_STAT1_IMPL_SPEC,
 *   @n SRIO_RIO_LANE_STAT1_LP_RX_TRN, SRIO_RIO_LANE_STAT1_LP_WIDTH,
 *   @n SRIO_RIO_LANE_STAT1_LP_LANE_NUM, SRIO_RIO_LANE_STAT1_LP_TAP_M1,
 *   @n SRIO_RIO_LANE_STAT1_LP_TAP_P1, SRIO_RIO_LANE_STAT1_LP_SCRM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        SRIO_LANE_STATUS    laneStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Lane Status Information.
        CSL_SRIO_GetLaneStatus (hSrio, 1, &laneStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLaneStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    SRIO_LANE_STATUS*   ptrLaneStatus
)
{
    Uint32  value = hSrio->RIO_LANE[portNum].RIO_LANE_STAT0;

    /* Populate the lane status */
    ptrLaneStatus->portNum = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_PORT_NUM);
    ptrLaneStatus->laneNum = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_LANE_NUM);
    ptrLaneStatus->txType  = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_TX_TYPE);
    ptrLaneStatus->txMode  = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_TX_MODE);
    ptrLaneStatus->rxType  = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_RX_TYPE);
    ptrLaneStatus->rxInv   = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_RX_INV);
    ptrLaneStatus->rxTrn   = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_RX_TRN);
    ptrLaneStatus->rxSync  = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_RX_SYNC);
    ptrLaneStatus->rxReady = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_RX_RDY);
    ptrLaneStatus->errCnt  = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_ERR_CNT);
    ptrLaneStatus->chgSync = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_CHG_SYNC);
    ptrLaneStatus->chgTrn  = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_CHG_TRN);
    ptrLaneStatus->stat1   = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_STAT1);
    ptrLaneStatus->stat2_7 = CSL_FEXT (value, SRIO_RIO_LANE_STAT0_STAT2_7);

    /* Get the lane status 1 */
    value = hSrio->RIO_LANE[portNum].RIO_LANE_STAT1;

    /* Populate the lane status */
    ptrLaneStatus->idle2        = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_IDLE2);
    ptrLaneStatus->infoOk       = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_INFO_OK);
    ptrLaneStatus->chg          = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_CHG);
    ptrLaneStatus->implSpecific = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_IMPL_SPEC);
    ptrLaneStatus->lpRxTrn      = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_LP_RX_TRN);
    ptrLaneStatus->lpWidth      = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_LP_WIDTH);
    ptrLaneStatus->lpLaneNum    = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_LP_LANE_NUM);
    ptrLaneStatus->lpTapM1      = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_LP_TAP_M1);
    ptrLaneStatus->lpTapP1      = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_LP_TAP_P1);
    ptrLaneStatus->lpScrm       = CSL_FEXT (value, SRIO_RIO_LANE_STAT1_LP_SCRM);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortPLMImplSpecificControl
 *
 *   @b Description
 *   @n The function is used to get the PLM Implementation Specific control
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
          ptrPLMControl         Pointer to the PLM Implementation specific control
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
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle          hSrio;
        SRIO_PLM_IMPL_CONTROL   implControl;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Implementation specific control
        CSL_SRIO_GetPortPLMImplSpecificControl (hSrio, 1, &implControl);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortPLMImplSpecificControl
(
    CSL_SrioHandle          hSrio,
    Uint8                   portNum,
    SRIO_PLM_IMPL_CONTROL*  ptrPLMControl
)
{
    Uint32  value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_IMP_SPEC_CTL;

    /* Populate the PLM implementation specific control */
    ptrPLMControl->payloadCapture   = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP);
    ptrPLMControl->useIdle2         = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2);
    ptrPLMControl->useIdle1         = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1);
    ptrPLMControl->dlbEn            = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN);
    ptrPLMControl->forceReinit      = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT);
    ptrPLMControl->softRstPort      = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT);
    ptrPLMControl->txBypass         = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS);
    ptrPLMControl->lineLoopbackMode = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN);
    ptrPLMControl->portSelfReset    = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST);
    ptrPLMControl->selfReset        = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST);
    ptrPLMControl->swapTx           = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX);
    ptrPLMControl->swapRx           = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX);
    ptrPLMControl->dltThresh        = CSL_FEXT (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortImplSpecificControl
 *
 *   @b Description
 *   @n The function is used to set the PLM Implementation Specific control
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
          ptrPLMControl         Pointer to the PLM Implementation specific control
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
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX, 
 *   @n SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle          hSrio;
        SRIO_PLM_IMPL_CONTROL   implControl;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Implementation specific control
        CSL_SRIO_GetPortPLMImplSpecificControl (hSrio, 1, &implControl);

        // Bypass the tx clock crossing the FIFO
        implControl.txBypass = 1;
        CSL_SRIO_SetPLMPortImplSpecificControl (hSrio, 1, &implControl);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortImplSpecificControl
(
    CSL_SrioHandle          hSrio,
    Uint8                   portNum,
    SRIO_PLM_IMPL_CONTROL*  ptrPLMControl
)
{
    Uint32  value = 0;

    /* Populate the PLM implementation specific control */
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP, ptrPLMControl->payloadCapture);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2, ptrPLMControl->useIdle2);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1, ptrPLMControl->useIdle1);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN, ptrPLMControl->dlbEn);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT, ptrPLMControl->forceReinit);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT, ptrPLMControl->softRstPort);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS, ptrPLMControl->txBypass);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN, ptrPLMControl->lineLoopbackMode);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST, ptrPLMControl->portSelfReset);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST, ptrPLMControl->selfReset);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX, ptrPLMControl->swapTx);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX, ptrPLMControl->swapRx);
    CSL_FINS (value, SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH, ptrPLMControl->dltThresh);

    hSrio->RIO_PLM[portNum].RIO_PLM_SP_IMP_SPEC_CTL = value;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortPowerDownControl
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Power Down status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
          powerDownStatus       Power Down Status  
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
 *   @n SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           powerDownStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Power Down Status
        CSL_SRIO_GetPLMPortPowerDownControl (hSrio, 1, &powerDownStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortPowerDownControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              powerDownStatus
)
{
    *powerDownStatus = CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_PWDN_CTL, SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortPowerDownControl
 *
 *   @b Description
 *   @n The function is used to set the PLM Port Power Down status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
          powerDownStatus       Power Down Status  
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
 *   @n SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PLM Port to be powered down
        CSL_SRIO_SetPLMPortPowerDownControl (hSrio, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortPowerDownControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               powerDownStatus
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_PWDN_CTL, SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT, powerDownStatus);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortMaxDenialErrorSet
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port Max. Denial error set.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Max. Denial Error is set
 *   @n FALSE   - Max. Denial Error is not set
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the MAX Denial Error set?
        if (CSL_SRIO_IsPLMPortMaxDenialErrorSet (hSrio, 1) == TRUE)
        {
            // MAX Denial error is set
        }
        else
        {
            // MAX Denial error is NOT set
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortMaxDenialErrorSet
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortMaxDenialErrorSet
 *
 *   @b Description
 *   @n The function is used to clear the PLM Port Max. Denial error
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the MAX Denial Error set?
        if (CSL_SRIO_IsPLMPortMaxDenialErrorSet (hSrio, 1) == TRUE)
        {
            // MAX Denial error is set
            ...
            // Clear the MAX Denial Error
            CSL_SRIO_ClearPLMPortMaxDenialErrorSet (hSrio, 1);
        }
        else
        {
            // MAX Denial error is NOT set
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortMaxDenialErrorSet
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL, (Uint32)1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortLinkInitialized
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port Link Initialized
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Port Link Initialized is set
 *   @n FALSE   - Port Link Initialized is not set
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_LINK_INIT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Link Initialized?
        if (CSL_SRIO_IsPLMPortLinkInitialized (hSrio, 1) == TRUE)
        {
            // PLM Port Link is initialized
        }
        else
        {
            // PLM Port Link is NOT initialized
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortLinkInitialized
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_LINK_INIT) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortLinkInitialized
 *
 *   @b Description
 *   @n The function is used to clear the PLM Port Link Initialization notification
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_LINK_INIT=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_LINK_INIT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Link Initialized?
        if (CSL_SRIO_IsPLMPortLinkInitialized (hSrio, 1) == TRUE)
        {
            // Link is initialized
            ...
            // Clear the Link initialization notification
            CSL_SRIO_ClearPLMPortMaxDenialErrorSet (hSrio, 1);
        }
        else
        {
            // Link is not initialized
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortLinkInitialized
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_LINK_INIT, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortDeadLink
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port has detected that the link
 *      partner has been removed
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Link partner has been removed
 *   @n FALSE   - Link partner has not been removed
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_DLT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Dead Link detected?
        if (CSL_SRIO_IsPLMPortDeadLink (hSrio, 1) == TRUE)
        {
            // PLM Port Dead Link detected
        }
        else
        {
            // PLM Port Dead Link NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortDeadLink
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_DLT) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortDeadLink
 *
 *   @b Description
 *   @n The function is used to clear the PLM Port Dead Link event
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_DLT=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_DLT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Dead Link detected?
        if (CSL_SRIO_IsPLMPortDeadLink (hSrio, 1) == TRUE)
        {
            // PLM Port Dead Link detected
            ...
            // Clear the Dead Link Event
            CSL_SRIO_ClearPLMPortDeadLink (hSrio, 1);
        }
        else
        {
            // PLM Port Dead Link NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortDeadLink
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_DLT, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortError
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port has detected an error
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Link error has been detected
 *   @n FALSE   - Link error has NOT been detected
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_PORT_ERR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port error detected?
        if (CSL_SRIO_IsPLMPortError (hSrio, 1) == TRUE)
        {
            // PLM Port error detected
        }
        else
        {
            // PLM Port error NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortError
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_PORT_ERR) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortError
 *
 *   @b Description
 *   @n The function is used to clear the PLM Port Error
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_PORT_ERR=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_PORT_ERR=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port error detected?
        if (CSL_SRIO_IsPLMPortError (hSrio, 1) == TRUE)
        {
            // PLM Port error detected
            ...
            // Clear the PLM Port Error.
            CSL_SRIO_ClearPLMPortError(hSrio, 1);
        }
        else
        {
            // PLM Port error NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortError
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_PORT_ERR, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortOutputFail
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port has detected an output fail
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Output fail has been detected
 *   @n FALSE   - Output fail has NOT been detected
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Output fail detected?
        if (CSL_SRIO_IsPLMPortOutputFail (hSrio, 1) == TRUE)
        {
            // PLM Port Output fail detected
        }
        else
        {
            // PLM Port Output fail NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortOutputFail
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortOutputFail
 *
 *   @b Description
 *   @n The function is used to clear the PLM Output Fail Port Error
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Output fail detected?
        if (CSL_SRIO_IsPLMPortOutputFail (hSrio, 1) == TRUE)
        {
            // PLM Port Output fail detected
            ...
            // Clear the PLM Outport error 
            CSL_SRIO_ClearPLMPortOutputFail (hSrio, 1);
        }
        else
        {
            // PLM Port Output fail NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortOutputFail
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortOutputDegrade
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port has detected an output 
 *      degrade
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Output degrade has been detected
 *   @n FALSE   - Output degrade has NOT been detected
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Output degrade detected?
        if (CSL_SRIO_IsPLMPortOutputDegrade (hSrio, 1) == TRUE)
        {
            // PLM Port Output degrade detected
        }
        else
        {
            // PLM Port Output degrade NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortOutputDegrade
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortOutputDegrade
 *
 *   @b Description
 *   @n The function is used to clear the PLM Output Degrade Port Error
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Output degrade detected?
        if (CSL_SRIO_IsPLMPortOutputDegrade (hSrio, 1) == TRUE)
        {
            // PLM Port Output degrade detected
            ...
            // Clear the PLM Output degrade condition
            CSL_SRIO_ClearPLMPortOutputDegrade (hSrio, 1);
        }
        else
        {
            // PLM Port Output degrade NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortOutputDegrade
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortMulticastCSDetected
 *
 *   @b Description
 *   @n The function is used to check if the PLM Port Multicast event control 
 *      symbol detected
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Multicast event control symbol has been detected
 *   @n FALSE   - Multicast event control symbol has NOT been detected
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_STATUS_MECS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Multicast Event Control Symbol detected?
        if (CSL_SRIO_IsPLMPortMulticastCSDetected (hSrio, 1) == TRUE)
        {
            // PLM Port Multicast Event Control Symbol detected
        }
        else
        {
            // PLM Port Multicast Event Control Symbol NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortMulticastCSDetected
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_MECS) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortMulticastCS
 *
 *   @b Description
 *   @n The function is used to clear the PLM Port Multicast Control Symbol
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number 
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
 *   @n SRIO_RIO_PLM_SP_STATUS_MECS=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_STATUS_MECS=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Is the PLM Port Multicast Event Control Symbol detected?
        if (CSL_SRIO_IsPLMPortMulticastCSDetected (hSrio, 1) == TRUE)
        {
            // PLM Port Multicast Event Control Symbol detected
            ...
            // Clear the PLM Port Multicast Event Control Symbol
            CSL_SRIO_ClearPLMPortMulticastCS (hSrio, 1);
        }
        else
        {
            // PLM Port Multicast Event Control Symbol NOT detected
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortMulticastCS
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_STATUS, SRIO_RIO_PLM_SP_STATUS_MECS, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortInterrupts
 *
 *   @b Description
 *   @n The function is used to get the current status of the PLM Port Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          portNum       SRIO Port Number
          intStatus     Current Interrupt status of the PLM port interrupts
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
 *   @n SRIO_RIO_PLM_SP_INT_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           intStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the current interrupt status
        CSL_SRIO_GetPLMPortInterrupts(hSrio, 1, &intStatus);

        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              intStatus
)
{
    *intStatus = hSrio->RIO_PLM[portNum].RIO_PLM_SP_INT_ENABLE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortInterrupts
 *
 *   @b Description
 *   @n The function is used to set the current status of the PLM Port Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          portNum       SRIO Port Number
          intStatus     Interrupt status to be configured
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
 *   @n SRIO_RIO_PLM_SP_INT_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           intStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the current interrupt status
        CSL_SRIO_GetPLMPortInterrupts(hSrio, 1, &intStatus);

        // Enable the Max Denial interrupt.
        intStatus = intStatus | 0x1;
        CSL_SRIO_SetPLMPortInterrupts(hSrio, 1, intStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               intStatus
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_INT_ENABLE = intStatus;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortWriteEnables
 *
 *   @b Description
 *   @n The function is used to get the current status of the PLM Port Write 
 *      enables. These events will enable a port write notification 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portWriteStatus   Current port write event status
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
 *   @n SRIO_RIO_PLM_SP_PW_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portWriteStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the current interrupt status
        CSL_SRIO_GetPLMPortWriteEnables(hSrio, 1, &portWriteStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortWriteEnables
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              portWriteStatus
)
{
    *portWriteStatus = hSrio->RIO_PLM[portNum].RIO_PLM_SP_PW_ENABLE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortWriteEnables
 *
 *   @b Description
 *   @n The function is used to set the current status of the PLM Port Write 
 *      enables. These events will enable a port write notification 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          portWriteStatus   Port write status to be configured
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
 *   @n SRIO_RIO_PLM_SP_PW_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portWriteStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the current interrupt status
        CSL_SRIO_GetPLMPortWriteEnables(hSrio, 1, &portWriteStatus);

        // Ensure that Max Denial port write notification is set.
        portWriteStatus = portWriteStatus | 0x1;
        CSL_SRIO_SetPLMPortWriteEnables (hSrio, 1, portWriteStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortWriteEnables
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               portWriteStatus
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_PW_ENABLE = portWriteStatus;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortEventGenerate
 *
 *   @b Description
 *   @n The function is used to get the current status of the PLM events.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          eventStatus       Current PLM event status
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
 *   @n SRIO_RIO_PLM_SP_EVENT_GEN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           eventStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the current PLM event generate 
        CSL_SRIO_GetPLMPortEventGenerate(hSrio, 1, &eventStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              eventStatus
)
{
    *eventStatus = hSrio->RIO_PLM[portNum].RIO_PLM_SP_EVENT_GEN;
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortEventGenerate
 *
 *   @b Description
 *   @n The function is used to set the current status of the PLM events.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          eventStatus       PLM event status to be configured
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
 *   @n SRIO_RIO_PLM_SP_EVENT_GEN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           eventStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the current PLM event generate 
        CSL_SRIO_GetPLMPortEventGenerate(hSrio, 1, &eventStatus);

        // Ensure MAX Denial event is set
        eventStatus = eventStatus | 0x1;
        CSL_SRIO_SetPLMPortEventGenerate (hSrio, 1, eventStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               eventStatus
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_EVENT_GEN = eventStatus;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePLMPortAllInterrupts
 *
 *   @b Description
 *   @n The function is used to enable all interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable all PLM Interrupts
        CSL_SRIO_EnablePLMPortAllInterrupts(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePLMPortAllInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_ALL_INT_EN, SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePLMPortAllInterrupts
 *
 *   @b Description
 *   @n The function is used to disable all interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable all PLM Interrupts
        CSL_SRIO_DisablePLMPortAllInterrupts(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePLMPortAllInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_ALL_INT_EN, SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortAllInterruptsEnabled
 *
 *   @b Description
 *   @n The function is used to check if the PLM port all interrupts are enabled
 *      or not?
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - PLM Port all interrupts are enabled
 *   @n FALSE - PLM Port all interrupts are disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if PLM Interrupts are enabled or not?
        if (CSL_SRIO_IsPLMPortAllInterruptsEnabled(hSrio, 1) == TRUE)
        {
            // PLM Interrupts are enabled.
        }
        else
        {
            // PLM Interrupts are disabled.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortAllInterruptsEnabled
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT(hSrio->RIO_PLM[portNum].RIO_PLM_SP_ALL_INT_EN, SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnablePLMPortAllPortWrites
 *
 *   @b Description
 *   @n The function is used to enable all port writes
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable all PLM Port Writes
        CSL_SRIO_EnablePLMPortAllPortWrites(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnablePLMPortAllPortWrites
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_ALL_PW_EN, SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisablePLMPortAllPortWrites
 *
 *   @b Description
 *   @n The function is used to disable all port writes
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
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
 *   @n SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable all PLM Port Writes
        CSL_SRIO_DisablePLMPortAllPortWrites(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisablePLMPortAllPortWrites
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_ALL_PW_EN, SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsPLMPortAllPortWritesEnabled
 *
 *   @b Description
 *   @n The function is used to disable all port writes
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE - PLM Port all Port Writes are enabled
 *   @n TRUE - PLM Port all Port Writes are disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if all PLM Port Writes are enabled or not?
        if (CSL_SRIO_IsPLMPortAllPortWritesEnabled(hSrio, 1) == TRUE)
        {
            // PLM Port All Port Writes are enabled
        }
        else
        {
            // PLM Port All Port Writes are disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsPLMPortAllPortWritesEnabled
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum
)
{
    if (CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_ALL_PW_EN, SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortPathControl
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Path control
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          pathID            Identifies the Path in which this port resides.
          pathConfig        Indicates the Path’s configuration
          pathMode          The mode of the path
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
 *   @n SRIO_RIO_PLM_SP_PATH_CTL_PATH_ID,SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION,
 *   @n SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           pathID;
        Uint8           pathConfig;
        Uint8           pathMode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port1 Path Control
        CSL_SRIO_GetPLMPortPathControl(hSrio, 1, &pathID, &pathConfig, &pathMode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortPathControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              pathID,
    Uint8*              pathConfig,
    Uint8*              pathMode
)
{
    Uint32 value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_PATH_CTL;
    *pathID     = CSL_FEXT (value, SRIO_RIO_PLM_SP_PATH_CTL_PATH_ID);
    *pathConfig = CSL_FEXT (value, SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION);
    *pathMode   = CSL_FEXT (value, SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortPathControlMode
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Path control mode
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          portNum           SRIO Port Number
          pathMode          The mode of the path
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
 *   @n SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the path mode for PLM Port 1 as 0
        CSL_SRIO_SetPLMPortPathControlMode(hSrio, 1, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortPathControlMode
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               pathMode
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_PATH_CTL, SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE, pathMode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortDiscoveryTimer
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Discovery timer
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          portDiscoveryTimer    PLM Port Discovery Timer
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
 *   @n SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portDiscoveryTimer;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Discovery Timer
        CSL_SRIO_GetPLMPortDiscoveryTimer(hSrio, 1, &portDiscoveryTimer);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortDiscoveryTimer
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              portDiscoveryTimer
)
{
    *portDiscoveryTimer = CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_DISCOVERY_TIMER,
                                    SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortDiscoveryTimer
 *
 *   @b Description
 *   @n The function is used to set the PLM Port Discovery timer
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          portDiscoveryTimer    PLM Port Discovery Timer
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
 *   @n SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PLM Discovery Timer
        CSL_SRIO_SetPLMPortDiscoveryTimer(hSrio, 1, 0x5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortDiscoveryTimer
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               portDiscoveryTimer
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_DISCOVERY_TIMER, SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER, 
              portDiscoveryTimer);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortSilenceTimer
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Silence timer
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          portSilenceTimer      PLM Port Silence Timer
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
 *   @n SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portSilenceTimer;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Silence Timer
        CSL_SRIO_GetPLMPortSilenceTimer(hSrio, 1, &portSilenceTimer);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortSilenceTimer
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              portSilenceTimer
)
{
    *portSilenceTimer = CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_SILENCE_TIMER,
                                    SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortSilenceTimer
 *
 *   @b Description
 *   @n The function is used to set up the PLM Port Silence timer value.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          portSilenceTimer      PLM Port Silence Timer value to configure
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
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           portSilenceTimer;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Silence Timer
        CSL_SRIO_GetPLMPortSilenceTimer(hSrio, 1, &portSilenceTimer);
        ...
        // Set the PLM Silence Timer
        CSL_SRIO_SetPLMPortSilenceTimer(hSrio, 1, &portSilenceTimer);
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortSilenceTimer
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               portSilenceTimer
)
{
   CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_SILENCE_TIMER,
            SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER, portSilenceTimer);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortVMinExponentConfig
 *
 *   @b Description
 *   @n The function is used to get the PLM Port VMin Export configuration
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrVminExponent       PLM Port VMin Exponent configuration
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
 *   @n SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP, SRIO_RIO_PLM_SP_VMIN_EXP_IMAX,
 *   @n SRIO_RIO_PLM_SP_VMIN_EXP_MMAX
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle          hSrio;
        SRIO_PLM_VMIN_EXPONENT  vminExponentConfig;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port VMin Exponent Configuration.
        CSL_SRIO_GetPLMPortVMinExponentConfig(hSrio, 1, &vminExponentConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortVMinExponentConfig
(
    CSL_SrioHandle          hSrio,
    Uint8                   portNum,
    SRIO_PLM_VMIN_EXPONENT* ptrVminExponent
)
{
    Uint32 value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_VMIN_EXP;

    ptrVminExponent->vminExp = CSL_FEXT (value, SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP);
    ptrVminExponent->imax    = CSL_FEXT (value, SRIO_RIO_PLM_SP_VMIN_EXP_IMAX); 
    ptrVminExponent->mmax    = CSL_FEXT (value, SRIO_RIO_PLM_SP_VMIN_EXP_MMAX);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortVMinExponentConfig
 *
 *   @b Description
 *   @n The function is used to set the PLM Port VMin Export configuration
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrVminExponent       PLM Port VMin Exponent configuration
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
 *   @n SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP, SRIO_RIO_PLM_SP_VMIN_EXP_IMAX,
 *   @n SRIO_RIO_PLM_SP_VMIN_EXP_MMAX
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle          hSrio;
        SRIO_PLM_VMIN_EXPONENT  vminExponentConfig;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port VMin Exponent Configuration.
        CSL_SRIO_GetPLMPortVMinExponentConfig(hSrio, 1, &vminExponentConfig);

        vminExponentConfig.imax = 2;
        CSL_SRIO_SetPLMPortVMinExponentConfig(hSrio, 1,  &vminExponentConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortVMinExponentConfig
(
    CSL_SrioHandle          hSrio,
    Uint8                   portNum,
    SRIO_PLM_VMIN_EXPONENT* ptrVminExponent
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_VMIN_EXP = 
            CSL_FMK (SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP, ptrVminExponent->vminExp)   |
            CSL_FMK (SRIO_RIO_PLM_SP_VMIN_EXP_IMAX, ptrVminExponent->imax)          |
            CSL_FMK (SRIO_RIO_PLM_SP_VMIN_EXP_MMAX, ptrVminExponent->mmax);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortPolarityControl
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Polarity Control
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrPolarityConfig     PLM Port Polarity configuration
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
 *   @n SRIO_RIO_PLM_SP_POL_CTL_TX3_POL, SRIO_RIO_PLM_SP_POL_CTL_TX2_POL,
 *   @n SRIO_RIO_PLM_SP_POL_CTL_TX1_POL, SRIO_RIO_PLM_SP_POL_CTL_TX0_POL,
 *   @n SRIO_RIO_PLM_SP_POL_CTL_RX3_POL, SRIO_RIO_PLM_SP_POL_CTL_RX2_POL,
 *   @n SRIO_RIO_PLM_SP_POL_CTL_RX1_POL, SRIO_RIO_PLM_SP_POL_CTL_RX0_POL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle              hSrio;
        SRIO_PLM_POLARITY_CONTROL   polarityControlConfig;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port Polarity Control Configuration.
        CSL_SRIO_GetPLMPortPolarityControl(hSrio, 1, &polarityControlConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortPolarityControl
(
    CSL_SrioHandle              hSrio,
    Uint8                       portNum,
    SRIO_PLM_POLARITY_CONTROL*  ptrPolarityConfig
)
{
    Uint32 value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_POL_CTL;

    ptrPolarityConfig->tx3Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_TX3_POL);
    ptrPolarityConfig->tx2Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_TX2_POL);
    ptrPolarityConfig->tx1Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_TX1_POL);
    ptrPolarityConfig->tx0Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_TX0_POL);
    ptrPolarityConfig->rx3Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_RX3_POL);
    ptrPolarityConfig->rx2Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_RX2_POL);
    ptrPolarityConfig->rx1Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_RX1_POL);
    ptrPolarityConfig->rx0Pol = CSL_FEXT (value, SRIO_RIO_PLM_SP_POL_CTL_RX0_POL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortPolarityControl
 *
 *   @b Description
 *   @n The function is used to set the PLM Port Polarity Control
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrPolarityConfig     PLM Port Polarity configuration
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
 *   @n SRIO_RIO_PLM_SP_POL_CTL_TX3_POL, SRIO_RIO_PLM_SP_POL_CTL_TX2_POL,
 *   @n SRIO_RIO_PLM_SP_POL_CTL_TX1_POL, SRIO_RIO_PLM_SP_POL_CTL_TX0_POL,
 *   @n SRIO_RIO_PLM_SP_POL_CTL_RX3_POL, SRIO_RIO_PLM_SP_POL_CTL_RX2_POL,
 *   @n SRIO_RIO_PLM_SP_POL_CTL_RX1_POL, SRIO_RIO_PLM_SP_POL_CTL_RX0_POL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle              hSrio;
        SRIO_PLM_POLARITY_CONTROL   polarityControlConfig;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port Polarity Control Configuration.
        CSL_SRIO_GetPLMPortPolarityControl(hSrio, 1, &polarityControlConfig);

        // Make sure that the TX3 Polarity is reversed
        polarityControlConfig.tx3Pol = 1;
        CSL_SRIO_SetPLMPortPolarityControl(hSrio, 1, &polarityControlConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortPolarityControl
(
    CSL_SrioHandle              hSrio,
    Uint8                       portNum,
    SRIO_PLM_POLARITY_CONTROL*  ptrPolarityConfig
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_POL_CTL = 
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_TX3_POL, ptrPolarityConfig->tx3Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_TX2_POL, ptrPolarityConfig->tx2Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_TX1_POL, ptrPolarityConfig->tx1Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_TX0_POL, ptrPolarityConfig->tx0Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_RX3_POL, ptrPolarityConfig->rx3Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_RX2_POL, ptrPolarityConfig->rx2Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_RX1_POL, ptrPolarityConfig->rx1Pol) |
        CSL_FMK (SRIO_RIO_PLM_SP_POL_CTL_RX0_POL, ptrPolarityConfig->rx0Pol);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortPacketDenialControl
 *
 *   @b Description
 *   @n The function is used to get the PLM Port Packet Denial.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          cntPacketNotAccepted  Controls whether Packet-Not-Accepted control 
                                symbols count toward the packet denial threshold value
          cntRetry              Controls whether Retry control symbols count toward 
                                the packet denial threshold value
          denialThreshold       Sets the threshold for reporting too many consecutive retries
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
 *   @n SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA, SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY,
 *   @n SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           cntPacketNotAccepted;
        Uint8           cntRetry;
        Uint8           denialThreshold;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port Packet Denial
        CSL_SRIO_GetPLMPortPacketDenialControl(hSrio, 1, &cntPacketNotAccepted, &cntRetry, &denialThreshold);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortPacketDenialControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              cntPacketNotAccepted,
    Uint8*              cntRetry,
    Uint8*              denialThreshold
)
{
    Uint32 value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_DENIAL_CTL;

    *cntPacketNotAccepted = CSL_FEXT (value, SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA);
    *cntRetry             = CSL_FEXT (value, SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY);
    *denialThreshold      = CSL_FEXT (value, SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortPacketDenialControl
 *
 *   @b Description
 *   @n The function is used to set the PLM Port Packet Denial.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          cntPacketNotAccepted  Controls whether Packet-Not-Accepted control 
                                symbols count toward the packet denial threshold value
          cntRetry              Controls whether Retry control symbols count toward 
                                the packet denial threshold value
          denialThreshold       Sets the threshold for reporting too many consecutive retries
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
 *   @n SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA, SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY,
 *   @n SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PLM Port Packet Denial
        CSL_SRIO_SetPLMPortPacketDenialControl(hSrio, 1, 1, 1, 2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortPacketDenialControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               cntPacketNotAccepted,
    Uint8               cntRetry,
    Uint8               denialThreshold
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_DENIAL_CTL = 
            CSL_FMK (SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA,       cntPacketNotAccepted)    |
            CSL_FMK (SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY,       cntRetry)                |
            CSL_FMK (SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH, denialThreshold);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortMECSStatus
 *
 *   @b Description
 *   @n The function is used to get the PLM Port MECS status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          cmd                   Indicates which MECS commands have been received.
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
 *   @n SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           cmd;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port MECS Status
        CSL_SRIO_GetPLMPortMECSStatus(hSrio, 1, &cmd);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortMECSStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              cmd
)
{
    *cmd = CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_RCVD_MECS, SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPLMPortMECSStatus
 *
 *   @b Description
 *   @n The function is used to clear the PLM Port MECS status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          cmd                   Indicates which MECS commands need to be cleared.
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
 *   @n SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT
 *
 *   @b Affects
 *   @n SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           cmd;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port MECS Status
        CSL_SRIO_GetPLMPortMECSStatus(hSrio, 1, &cmd);
        ...
        // Clear the PLM Port MECS Status.
        CSL_SRIO_ClearPLMPortMECSStatus(hSrio, 1, cmd);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPLMPortMECSStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               cmd
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_RCVD_MECS, SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT, cmd);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortMECSForwarding
 *
 *   @b Description
 *   @n The function is used to get the PLM Port MECS Forwarding 
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          subscription          Specifies which MECS cmd values the port 
                                should forward to its link
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
 *   @n SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;
        Uint8           subscription;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port MECS Forwarding
        CSL_SRIO_GetPLMPortMECSForwarding(hSrio, 1, &subscription);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortMECSForwarding
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              subscription
)
{
    *subscription = CSL_FEXT (hSrio->RIO_PLM[portNum].RIO_PLM_SP_MECS_FWD,
                              SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortMECSForwarding
 *
 *   @b Description
 *   @n The function is used to set the PLM Port MECS Forwarding 
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          subscription          Specifies which MECS cmd values the port 
                                should forward to its link
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
 *   @n SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle  hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PLM Port MECS Forwarding
        CSL_SRIO_SetPLMPortMECSForwarding(hSrio, 1, 0x1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortMECSForwarding
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               subscription
)
{
    CSL_FINS (hSrio->RIO_PLM[portNum].RIO_PLM_SP_MECS_FWD, SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION, subscription);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPLMPortCSTransmit
 *
 *   @b Description
 *   @n The function is used to get the Control Symbol Transmit1
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrCSTransmit         Control Symbol Transmit Configuration
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
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0, SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0,
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1,   SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB,
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1, SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD,
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2, SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle              hSrio;
        SRIO_PLM_CONTROL_SYMBOL     cntlSymbol;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PLM Port Control Symbol Transmit Configuration
        CSL_SRIO_GetPLMPortCSTransmit(hSrio, 1, &cntlSymbol);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPLMPortCSTransmit
(
    CSL_SrioHandle              hSrio,
    Uint8                       portNum,
    SRIO_PLM_CONTROL_SYMBOL*    ptrCSTransmit
)
{
    Uint32 value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_LONG_CS_TX1;

    ptrCSTransmit->stype0   = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0); 
    ptrCSTransmit->par0     = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0); 
    ptrCSTransmit->par1     = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1); 
    ptrCSTransmit->csEmb    = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB); 
    ptrCSTransmit->stype1   = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1); 
    ptrCSTransmit->cmd      = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD); 

    /* Get the value from the Control Symbol Transmit2 register */
    value = hSrio->RIO_PLM[portNum].RIO_PLM_SP_LONG_CS_TX2;
    ptrCSTransmit->stype2   = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2);
    ptrCSTransmit->parm     = CSL_FEXT (value,SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPLMPortCSTransmit
 *
 *   @b Description
 *   @n The function is used to set the Control Symbol Transmit1
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrCSTransmit         Control Symbol Transmit Configuration
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
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0, SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0,
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1,   SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB,
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1, SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD,
 *   @n SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2, SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle              hSrio;
        SRIO_PLM_CONTROL_SYMBOL     cntlSymbol;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PLM Port Control Symbol Transmit Configuration
        CSL_SRIO_SetPLMPortCSTransmit(hSrio, 1, &cntlSymbol);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPLMPortCSTransmit
(
    CSL_SrioHandle              hSrio,
    Uint8                       portNum,
    SRIO_PLM_CONTROL_SYMBOL*    ptrCSTransmit
)
{
    hSrio->RIO_PLM[portNum].RIO_PLM_SP_LONG_CS_TX1 = 
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0, ptrCSTransmit->stype0)    |
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0,   ptrCSTransmit->par0)      |
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1,   ptrCSTransmit->par1)      |
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB,  ptrCSTransmit->csEmb)     |
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1, ptrCSTransmit->stype1)    |
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD,     ptrCSTransmit->cmd);

    hSrio->RIO_PLM[portNum].RIO_PLM_SP_LONG_CS_TX2 =
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2, ptrCSTransmit->stype2)    |
            CSL_FMK (SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM,    ptrCSTransmit->parm);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTLMPortControl
 *
 *   @b Description
 *   @n The function is used to get the TLM port control
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrTLMControl         TLM Port Control
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
 *   @n SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT, SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT,
 *   @n SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS, SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS,
 *   @n SRIO_RIO_TLM_SP_CONTROL_LENGTH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle          hSrio;
        SRIO_TLM_CONTROL        tlmCntlSymbol;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Control Configuration
        CSL_SRIO_GetTLMPortControl(hSrio, 1, &tlmCntlSymbol);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTLMPortControl
(
    CSL_SrioHandle              hSrio,
    Uint8                       portNum,
    SRIO_TLM_CONTROL*           ptrTLMControl
)
{
    Uint32 value = hSrio->RIO_TLM[portNum].RIO_TLM_SP_CONTROL;

    ptrTLMControl->portGroupSelect = CSL_FEXT (value,SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT);
    ptrTLMControl->voqSelect       = CSL_FEXT (value,SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT);
    ptrTLMControl->tgtIDDis        = CSL_FEXT (value,SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS);
    ptrTLMControl->mtcTgtIDDis     = CSL_FEXT (value,SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS);
    ptrTLMControl->length          = CSL_FEXT (value,SRIO_RIO_TLM_SP_CONTROL_LENGTH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTLMPortControl
 *
 *   @b Description
 *   @n The function is used to set the TLM port control
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          ptrTLMControl         TLM Port Control
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
 *   @n SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT, SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT,
 *   @n SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS, SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle          hSrio;
        SRIO_TLM_CONTROL        tlmCntlSymbol;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Control Configuration
        CSL_SRIO_GetTLMPortControl(hSrio, 1, &tlmCntlSymbol);

        // Set Promiscuous mode.
        tlmCntlSymbol.tgtIDDis    = 1;
        tlmCntlSymbol.mtcTgtIDDis = 1;
        CSL_SRIO_SetTLMPortControl (hSrio, 1, &tlmCntlSymbol);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTLMPortControl
(
    CSL_SrioHandle              hSrio,
    Uint8                       portNum,
    SRIO_TLM_CONTROL*           ptrTLMControl
)
{
    hSrio->RIO_TLM[portNum].RIO_TLM_SP_CONTROL = 
            CSL_FMK (SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT, ptrTLMControl->portGroupSelect)  |
            CSL_FMK (SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT,       ptrTLMControl->voqSelect)        |
            CSL_FMK (SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS,       ptrTLMControl->tgtIDDis)         |
            CSL_FMK (SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS,   ptrTLMControl->mtcTgtIDDis);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTLMPortStatus
 *
 *   @b Description
 *   @n The function is used to get the TLM port status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC, SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igBadVC;
        Uint8               igBrrFilter;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Status
        CSL_SRIO_GetTLMPortStatus(hSrio, 1, &igBadVC, &igBrrFilter);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTLMPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              igBadVC,
    Uint8*              igBrrFilter
)
{
    Uint32 value = hSrio->RIO_TLM[portNum].RIO_TLM_SP_STATUS;

    *igBadVC     = CSL_FEXT (value, SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC);
    *igBrrFilter = CSL_FEXT (value, SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearTLMPortStatus
 *
 *   @b Description
 *   @n The function is used to clear the TLM port status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC=1, SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER=1
 *
 *   @b Affects
 *   @n SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC=0, SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igBadVC;
        Uint8               igBrrFilter;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Status
        CSL_SRIO_GetTLMPortStatus(hSrio, 1, &igBadVC, &igBrrFilter);
        ...
        // Clear the TLM Port Status
        CSL_SRIO_ClearTLMPortStatus (hSrio, 1, igBadVC, igBrrFilter);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearTLMPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               igBadVC,
    Uint8               igBrrFilter
)
{
    hSrio->RIO_TLM[portNum].RIO_TLM_SP_STATUS = 
            CSL_FMK (SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC,      igBadVC)        |
            CSL_FMK (SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER,  igBrrFilter);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTLMPortInterrupts
 *
 *   @b Description
 *   @n The function is used to get the TLM port interrupt status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC, 
 *   @n SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igBadVC;
        Uint8               igBrrFilter;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Interrupt Status
        CSL_SRIO_GetTLMPortInterrupts(hSrio, 1, &igBadVC, &igBrrFilter);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTLMPortInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              igBadVC,
    Uint8*              igBrrFilter
)
{
    Uint32 value = hSrio->RIO_TLM[portNum].RIO_TLM_SP_INT_ENABLE;

    *igBadVC     = CSL_FEXT (value, SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC);
    *igBrrFilter = CSL_FEXT (value, SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTLMPortInterrupts
 *
 *   @b Description
 *   @n The function is used to set the TLM Port Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC, 
 *   @n SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igBadVC;
        Uint8               igBrrFilter;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable BadVC Interrupt & Disable BRR Filter Interrupts 
        CSL_SRIO_SetTLMPortInterrupts(hSrio, 1, 1, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTLMPortInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               igBadVC,
    Uint8               igBrrFilter
)
{
    hSrio->RIO_TLM[portNum].RIO_TLM_SP_INT_ENABLE = 
            CSL_FMK (SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC,      igBadVC)        |
            CSL_FMK (SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER,  igBrrFilter);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTLMPortWriteEnable
 *
 *   @b Description
 *   @n The function is used to get the TLM port write enable status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC, 
 *   @n SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igBadVC;
        Uint8               igBrrFilter;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Write Enable status
        CSL_SRIO_GetTLMPortWriteEnable(hSrio, 1, &igBadVC, &igBrrFilter);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTLMPortWriteEnable
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              igBadVC,
    Uint8*              igBrrFilter
)
{
    Uint32 value = hSrio->RIO_TLM[portNum].RIO_TLM_SP_PW_ENABLE;

    *igBadVC     = CSL_FEXT (value, SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC);
    *igBrrFilter = CSL_FEXT (value, SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTLMPortWriteEnable
 *
 *   @b Description
 *   @n The function is used to set the TLM port write enable status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC, 
 *   @n SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the TLM Port Write Enable status for BAD VC and NOT for BRR Filter
        CSL_SRIO_SetTLMPortWriteEnable(hSrio, 1, 1, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTLMPortWriteEnable
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               igBadVC,
    Uint8               igBrrFilter
)
{
    hSrio->RIO_TLM[portNum].RIO_TLM_SP_PW_ENABLE = 
            CSL_FMK (SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC,     igBadVC)          | 
            CSL_FMK (SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER, igBrrFilter); 
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTLMPortEventGenerate
 *
 *   @b Description
 *   @n The function is used to set the TLM port event generates
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igBadVC               Detected an inbound packet with the VC bit set
          igBrrFilter           Discarded an inbound transaction based on the BRR
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
 *   @n SRIO_RIO_TLM_SP_EVENT_GEN_IG_BAD_VC, 
 *   @n SRIO_RIO_TLM_SP_EVENT_GEN_IG_BRR_FILTER
 *
 *   @b Affects
 *   @n SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC=1, SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igBadVC;
        Uint8               igBrrFilter;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the TLM Port Event Generate Status
        CSL_SRIO_SetTLMPortEventGenerate(hSrio, 1, 1, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTLMPortEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               igBadVC,
    Uint8               igBrrFilter
)
{
    hSrio->RIO_TLM[portNum].RIO_TLM_SP_EVENT_GEN = 
        CSL_FMK (SRIO_RIO_TLM_SP_EVENT_GEN_IG_BAD_VC,     igBadVC)      |
        CSL_FMK (SRIO_RIO_TLM_SP_EVENT_GEN_IG_BRR_FILTER, igBrrFilter);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTLMPortBaseRoutingInfo
 *
 *   @b Description
 *   @n The function is used to get the TLM Base Routing Information
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          brrNum                BRR Number for which this configuration is applied.
          enableStatus          Status of the BRR for accepting & routing packets
          maintRouting          Maintainance Packet Routing Status
          privateStatus         Status of the BRR to be used by its port or by all
                                ports
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
 *   @n SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE;
 *   @n SRIO_RIO_TLM_SP_BRR_1_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_1_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_1_CTL_PRIVATE;
 *   @n SRIO_RIO_TLM_SP_BRR_2_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_2_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_2_CTL_PRIVATE;
 *   @n SRIO_RIO_TLM_SP_BRR_3_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_3_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_3_CTL_PRIVATE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               enableStatus;
        Uint8               maintRouting;
        Uint8               privateStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Base routing information.
        CSL_SRIO_GetTLMPortBaseRoutingInfo(hSrio, 1, 0, &enableStatus, &maintRouting, &privateStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTLMPortBaseRoutingInfo
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               brrNum,
    Uint8*              enableStatus,
    Uint8*              maintRouting,
    Uint8*              privateStatus
)
{
    Uint32 value = hSrio->RIO_TLM[portNum].brr[brrNum].RIO_TLM_SP_BRR_CTL;

    *enableStatus   = CSL_FEXT (value, SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE);
    *maintRouting   = CSL_FEXT (value, SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM);
    *privateStatus  = CSL_FEXT (value, SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTLMPortBaseRoutingInfo
 *
 *   @b Description
 *   @n The function is used to set the TLM Base Routing Information
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          brrNum                BRR Number for which this configuration is applied.
          enableStatus          Status of the BRR for accepting & routing packets
          maintRouting          Maintainance Packet Routing Status
          privateStatus         Status of the BRR to be used by its port or by all
                                ports
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
 *   @n SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE;
 *   @n SRIO_RIO_TLM_SP_BRR_1_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_1_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_1_CTL_PRIVATE;
 *   @n SRIO_RIO_TLM_SP_BRR_2_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_2_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_2_CTL_PRIVATE;
 *   @n SRIO_RIO_TLM_SP_BRR_3_CTL_ENABLE, SRIO_RIO_TLM_SP_BRR_3_CTL_ROUTE_MR_TO_LLM,
 *   @n SRIO_RIO_TLM_SP_BRR_3_CTL_PRIVATE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the TLM Port Base routing information.
        CSL_SRIO_SetTLMPortBaseRoutingInfo(hSrio, 1, 0, 1, 0, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTLMPortBaseRoutingInfo
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               brrNum,
    Uint8               enableStatus,
    Uint8               maintRouting,
    Uint8               privateStatus
)
{
    hSrio->RIO_TLM[portNum].brr[brrNum].RIO_TLM_SP_BRR_CTL = 
              CSL_FMK (SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE,            enableStatus)   |
              CSL_FMK (SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM,   maintRouting)   |
              CSL_FMK (SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE,           privateStatus);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetTLMPortBaseRoutingPatternMatch
 *
 *   @b Description
 *   @n The function is used to get the TLM Base Routing Pattern Match information
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          brrNum                BRR Number for which this configuration is applied.
          pattern               16 bit ID used for comparison.
          match                 Indicates which of the 16bits should be used
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
 *   @n SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH;
 *   @n SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_MATCH;
 *   @n SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_MATCH;
 *   @n SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_MATCH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint16              pattern;
        Uint16              match;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the TLM Port Base routing pattern match information
        CSL_SRIO_GetTLMPortBaseRoutingPatternMatch(hSrio, 1, &pattern, &match);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetTLMPortBaseRoutingPatternMatch
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               brrNum,
    Uint16*             pattern,
    Uint16*             match
)
{
    Uint32 value = hSrio->RIO_TLM[portNum].brr[brrNum].RIO_TLM_SP_BRR_PATTERN_MATCH;

    *pattern = CSL_FEXT (value, SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN);
    *match   = CSL_FEXT (value, SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetTLMPortBaseRoutingPatternMatch
 *
 *   @b Description
 *   @n The function is used to set the TLM Base Routing Pattern Match information
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          brrNum                BRR Number for which this configuration is applied.
          pattern               16 bit ID used for comparison.
          match                 Indicates which of the 16bits should be used
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
 *   @n SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH;
 *   @n SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_MATCH;
 *   @n SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_MATCH;
 *   @n SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_PATTERN, SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_MATCH
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint16              pattern;
        Uint16              match;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the TLM Port Base routing pattern match information
        CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 1, &pattern, &match);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetTLMPortBaseRoutingPatternMatch
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               brrNum,
    Uint16              pattern,
    Uint16              match
)
{
    hSrio->RIO_TLM[portNum].brr[brrNum].RIO_TLM_SP_BRR_PATTERN_MATCH = 
            CSL_FMK (SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN, pattern)      |
            CSL_FMK (SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH,   match);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortControl
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Control information
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egReorderMode         Reorder Mode
          egReorderStick        Number of repeat times the CRQ is reordered
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
 *   @n SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE, SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               egReorderMode;
        Uint8               egReorderStick;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Control Information.
        CSL_SRIO_GetPBMPortControl(hSrio, 1, &egReorderMode, &egReorderStick);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              egReorderMode,
    Uint8*              egReorderStick
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_CONTROL;

    *egReorderMode  = CSL_FEXT (value, SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE);
    *egReorderStick = CSL_FEXT (value, SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPBMPortControl
 *
 *   @b Description
 *   @n The function is used to set the PBM Port Control information
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egReorderMode         Reorder Mode
          egReorderStick        Number of repeat times the CRQ is reordered
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
 *   @n SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE, SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PBM Port Control Information.
        CSL_SRIO_GetPBMPortControl(hSrio, 1, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPBMPortControl
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               egReorderMode,
    Uint8               egReorderStick
)
{
    hSrio->RIO_PBM[portNum].RIO_PBM_SP_CONTROL = 
            CSL_FMK (SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE,  egReorderMode)    |
            CSL_FMK (SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK, egReorderStick);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortStatus
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          igEmpty               PBMi Queue Status
          egEmpty               PBMe Queue Status
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_STATUS_IG_EMPTY, SRIO_RIO_PBM_SP_STATUS_EG_EMPTY,
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW, SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL, SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igEmpty;
        Uint8               egEmpty;
        Uint8               egDataOverflow;
        Uint8               egCRQOverflow;
        Uint8               egBadChannel;
        Uint8               egBabblePacket;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Status
        CSL_SRIO_GetPBMPortStatus(hSrio, 1, &igEmpty, &egEmpty, 
                                  &egDataOverflow, &egCRQOverflow, &egBadChannel, &egBabblePacket);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              igEmpty,
    Uint8*              egEmpty,
    Uint8*              egDataOverflow,
    Uint8*              egCRQOverflow,
    Uint8*              egBadChannel,
    Uint8*              egBabblePacket
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_STATUS;

    *igEmpty        = CSL_FEXT (value, SRIO_RIO_PBM_SP_STATUS_IG_EMPTY);
    *egEmpty        = CSL_FEXT (value, SRIO_RIO_PBM_SP_STATUS_EG_EMPTY);
    *egDataOverflow = CSL_FEXT (value, SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW);
    *egCRQOverflow  = CSL_FEXT (value, SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW);
    *egBadChannel   = CSL_FEXT (value, SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL);
    *egBabblePacket = CSL_FEXT (value, SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearPBMPortStatus
 *
 *   @b Description
 *   @n The function is used to clear the PBM Port Status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW=1, SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW=1,
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL=1, SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET=1
 *
 *   @b Affects
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW=0, SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW=0,
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL=0, SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igEmpty;
        Uint8               egEmpty;
        Uint8               egDataOverflow;
        Uint8               egCRQOverflow;
        Uint8               egBadChannel;
        Uint8               egBabblePacket;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Status
        CSL_SRIO_GetPBMPortStatus(hSrio, 1, &igEmpty, &egEmpty, 
                                  &egDataOverflow, &egCRQOverflow, &egBadChannel, &egBabblePacket);
        ...
        // Clear the PBM Port Status
        CSL_SRIO_ClearPBMPortStatus(hSrio, 1, egDataOverflow, egCRQOverflow, egBadChannel, egBabblePacket);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearPBMPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               egDataOverflow,
    Uint8               egCRQOverflow,
    Uint8               egBadChannel,
    Uint8               egBabblePacket
)
{
    hSrio->RIO_PBM[portNum].RIO_PBM_SP_STATUS =
            CSL_FMK (SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW,   egDataOverflow)     |
            CSL_FMK (SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW,    egCRQOverflow)      |
            CSL_FMK (SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL,     egBadChannel)       |
            CSL_FMK (SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET,   egBabblePacket);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortInterrupts
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Interrupt status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW, 
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL,
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               egDataOverflow;
        Uint8               egCRQOverflow;
        Uint8               egBadChannel;
        Uint8               egBabblePacket;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Interrupts
        CSL_SRIO_GetPBMPortInterrupts(hSrio, 1, &egDataOverflow, &egCRQOverflow, 
                                      &egBadChannel, &egBabblePacket);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              egDataOverflow,
    Uint8*              egCRQOverflow,
    Uint8*              egBadChannel,
    Uint8*              egBabblePacket
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_INT_ENABLE;

    *egDataOverflow = CSL_FEXT (value, SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW);
    *egCRQOverflow  = CSL_FEXT (value, SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW);
    *egBadChannel   = CSL_FEXT (value, SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL);
    *egBabblePacket = CSL_FEXT (value, SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPBMPortInterrupts
 *
 *   @b Description
 *   @n The function is used to set the PBM Port Interrupt status
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW, 
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL,
 *   @n SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PBM Port Interrupts for Data Overflow only
        CSL_SRIO_SetPBMPortInterrupts(hSrio, 1, 1, 0, 0, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPBMPortInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               egDataOverflow,
    Uint8               egCRQOverflow,
    Uint8               egBadChannel,
    Uint8               egBabblePacket
)
{
    hSrio->RIO_PBM[portNum].RIO_PBM_SP_INT_ENABLE = 
        CSL_FMK (SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW,   egDataOverflow)     |
        CSL_FMK (SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW,    egCRQOverflow)      |
        CSL_FMK (SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL,     egBadChannel)       |
        CSL_FMK (SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET,   egBabblePacket);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortWriteEnables
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Write Enables
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW, 
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL,
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               egDataOverflow;
        Uint8               egCRQOverflow;
        Uint8               egBadChannel;
        Uint8               egBabblePacket;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Write Enables
        CSL_SRIO_GetPBMPortWriteEnables(hSrio, 1, &egDataOverflow, &egCRQOverflow, 
                                      &egBadChannel, &egBabblePacket);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortWriteEnables
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              egDataOverflow,
    Uint8*              egCRQOverflow,
    Uint8*              egBadChannel,
    Uint8*              egBabblePacket
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_PW_ENABLE;

    *egDataOverflow = CSL_FEXT (value, SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW);
    *egCRQOverflow  = CSL_FEXT (value, SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW);
    *egBadChannel   = CSL_FEXT (value, SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL);
    *egBabblePacket = CSL_FEXT (value, SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPBMPortWriteEnables
 *
 *   @b Description
 *   @n The function is used to set the PBM Port Write Enables
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW, 
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL,
 *   @n SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PBM Port Write Enables for Data Overflow only
        CSL_SRIO_SetPBMPortWriteEnables(hSrio, 1, 1, 0, 0, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPBMPortWriteEnables
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               egDataOverflow,
    Uint8               egCRQOverflow,
    Uint8               egBadChannel,
    Uint8               egBabblePacket
)
{
    hSrio->RIO_PBM[portNum].RIO_PBM_SP_PW_ENABLE = 
            CSL_FMK (SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW, egDataOverflow)    |
            CSL_FMK (SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW,  egCRQOverflow)     |
            CSL_FMK (SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL,   egBadChannel)      |
            CSL_FMK (SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET, egBabblePacket);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortEventGenerate
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Event generates
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW, 
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL,
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               egDataOverflow;
        Uint8               egCRQOverflow;
        Uint8               egBadChannel;
        Uint8               egBabblePacket;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Event Generates
        CSL_SRIO_GetPBMPortEventGenerate(hSrio, 1, &egDataOverflow, &egCRQOverflow, 
                                      &egBadChannel, &egBabblePacket);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8*              egDataOverflow,
    Uint8*              egCRQOverflow,
    Uint8*              egBadChannel,
    Uint8*              egBabblePacket
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_EVENT_GEN;

    *egDataOverflow = CSL_FEXT (value, SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW);
    *egCRQOverflow  = CSL_FEXT (value, SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW);
    *egBadChannel   = CSL_FEXT (value, SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL);
    *egBabblePacket = CSL_FEXT (value, SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPBMPortEventGenerate
 *
 *   @b Description
 *   @n The function is used to set the PBM Port Event generates
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          egDataOverflow        PBMe received a request to enqueue a packet 
                                for which it did not have enough data storage.
          egCRQOverflow         PBMe received a request to enqueue a packet 
                                for which it did not have CRQ Entry.
          egBadChannel          PBMe received a request to enqueue a packet 
                                on a channel enqueue interface which should be 
                                unused for the path’s Mode.
          egBabblePacket        PBMe detected a packet that exceeded 276 bytes 
                                on its enqueue interface
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
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW, 
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW,
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL,
 *   @n SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET
 *
 *   @b Affects
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW=1, SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW=1,
 *   @n SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL=1, SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PBM Port Event Generates for Data overflow only
        CSL_SRIO_SetPBMPortEventGenerate(hSrio, 1, 1, 0, 0, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPBMPortEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               egDataOverflow,
    Uint8               egCRQOverflow,
    Uint8               egBadChannel,
    Uint8               egBabblePacket
)
{
    hSrio->RIO_PBM[portNum].RIO_PBM_SP_EVENT_GEN = 
            CSL_FMK (SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW,    egDataOverflow)     |
            CSL_FMK (SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW,     egCRQOverflow)      |
            CSL_FMK (SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL,      egBadChannel)       |
            CSL_FMK (SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET,    egBabblePacket);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortIngressResources
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Ingress resources
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          dataNodes             Indicates the number of Data Nodes 
                                implemented in the PBMi for packet storage
          tags                  Indicates the number of Tags implemented in 
                                the PBMi for packet storage for the port
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
 *   @n SRIO_RIO_PBM_SP_IG_RESOURCES_DATANODES, 
 *   @n SRIO_RIO_PBM_SP_IG_RESOURCES_TAGS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint16              dataNodes;
        Uint16              tags;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Ingress Resources
        CSL_SRIO_GetPBMPortIngressResources(hSrio, 1, &dataNodes, &tags);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortIngressResources
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint16*             dataNodes,
    Uint16*             tags
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_RESOURCES;

    *dataNodes = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_RESOURCES_DATANODES);
    *tags      = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_RESOURCES_TAGS);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortEgressResources
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Egress resources
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          dataNodes             Indicates the number of Data Nodes 
                                implemented in the PBMe for packet storage
          crqEntries            Indicates the number of CRQ Entries implemented in 
                                the PBMe for packet storage for the port
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
 *   @n SRIO_RIO_PBM_SP_EG_RESOURCES_DATANODES, 
 *   @n SRIO_RIO_PBM_SP_EG_RESOURCES_CRQ_ENTRIES
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint16              dataNodes;
        Uint8               crqEntries;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Egress Resources
        CSL_SRIO_GetPBMPortEgressResources(hSrio, 1, &dataNodes, &crqEntries);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortEgressResources
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint16*             dataNodes,
    Uint8*              crqEntries
)
{
    Uint32 value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_EG_RESOURCES;

    *dataNodes  = CSL_FEXT (value, SRIO_RIO_PBM_SP_EG_RESOURCES_DATANODES);
    *crqEntries = CSL_FEXT (value, SRIO_RIO_PBM_SP_EG_RESOURCES_CRQ_ENTRIES);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPBMPortIngressPriorityWatermark
 *
 *   @b Description
 *   @n The function is used to get the PBM Port Ingress Priority Watermark
 *      configuration
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          priority              Priority 0,1,2 or 3
          crfWatermark          CRF Watermark
          priWatermark          Priority Watermark
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
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM;
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM;
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM;
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint16              crfWatermark;
        Uint16              priWatermark;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the PBM Port Ingress 0 Watermarks
        CSL_SRIO_GetPBMPortIngressPriorityWatermark(hSrio, 1, 0, &crfWatermark, &priWatermark);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPBMPortIngressPriorityWatermark
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               priority,
    Uint16*             crfWatermark,
    Uint16*             priWatermark
)
{
    Uint32 value;

    switch (priority)
    {
        case 0:
        {
            value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK0;

            *crfWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM);
            *priWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM);                
            break;
        }
        case 1:
        {
            value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK1;

            *crfWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM);
            *priWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM);
            break;
        }
        case 2:
        {
            value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK2;

            *crfWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM);
            *priWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM);
            break;
        }
        case 3:
        {
            value = hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK3;

            *crfWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM);
            *priWatermark  = CSL_FEXT (value, SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM);
            break;
        }
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPBMPortIngressPriorityWatermark
 *
 *   @b Description
 *   @n The function is used to set the PBM Port Ingress Priority Watermark
 *      configuration
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          portNum               SRIO Port Number
          priority              Priority 0,1,2 or 3
          crfWatermark          CRF Watermark
          priWatermark          Priority Watermark
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
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM;
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM;
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM;
 *   @n SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM, SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the PBM Port Ingress 0 Watermarks
        CSL_SRIO_SetPBMPortIngressPriorityWatermark(hSrio, 1, 0, 10, 12);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPBMPortIngressPriorityWatermark
(
    CSL_SrioHandle      hSrio,
    Uint8               portNum,
    Uint8               priority,
    Uint16              crfWatermark,
    Uint16              priWatermark
)
{
    switch (priority)
    {
        case 0:
        {
            hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK0 = 
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM, crfWatermark)   |
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM,    priWatermark);                
            break;
        }
        case 1:
        {
            hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK1 = 
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM, crfWatermark)   |
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM,    priWatermark);
            break;
        }
        case 2:
        {
            hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK2 = 
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM, crfWatermark)   |
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM,    priWatermark);
            break;
        }
        case 3:
        {
            hSrio->RIO_PBM[portNum].RIO_PBM_SP_IG_WATERMARK3 = 
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM, crfWatermark)   |
                CSL_FMK (SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM,    priWatermark);
            break;
        }
    }
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtInterruptStatus
 *
 *   @b Description
 *   @n The function is used to get the Event Management Interrupt Status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          intStatus     Event Management Interrupt Status
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
 *   @n SRIO_RIO_EM_INT_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              intStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Interrupt Status
        CSL_SRIO_GetEventMgmtInterruptStatus(hSrio, &intStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtInterruptStatus
(
    CSL_SrioHandle      hSrio,
    Uint32*             intStatus
)
{
    *intStatus = hSrio->RIO_EM_INT_STAT;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtInterrupts
 *
 *   @b Description
 *   @n The function is used to get the Event Management Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          log           When set, this field enables a logical layer event 
                        detected in the User Core to cause an interrupt
          mecs          When set, this field enables an interrupt to be raised 
                        upon reception of an MECS with a command value that is 
                        enabled
          pwRx          When set, this field enables the reception of a port-write 
                        to cause an interrupt
          localLog      When set, this field enables an implementation-specific
                        Logical/Transport Layer Error to cause an interrupt
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
 *   @n SRIO_RIO_EM_INT_ENABLE_LOG,   SRIO_RIO_EM_INT_ENABLE_MECS, 
 *   @n SRIO_RIO_EM_INT_ENABLE_PW_RX, SRIO_RIO_EM_INT_ENABLE_LOCALOG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               log;
        Uint8               mecs;
        Uint8               pwRx;
        Uint8               localLog;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Interrupts 
        CSL_SRIO_GetEventMgmtInterrupts(hSrio, &log, &mecs, &pwRx, &localLog);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8*              log,
    Uint8*              mecs,
    Uint8*              pwRx,
    Uint8*              localLog
)
{
    Uint32  value = hSrio->RIO_EM_INT_ENABLE;

    *log        = CSL_FEXT (value, SRIO_RIO_EM_INT_ENABLE_LOG);
    *mecs       = CSL_FEXT (value, SRIO_RIO_EM_INT_ENABLE_MECS);
    *pwRx       = CSL_FEXT (value, SRIO_RIO_EM_INT_ENABLE_PW_RX);
    *localLog   = CSL_FEXT (value, SRIO_RIO_EM_INT_ENABLE_LOCALOG);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtInterrupts
 *
 *   @b Description
 *   @n The function is used to set the Event Management Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          log           When set, this field enables a logical layer event 
                        detected in the User Core to cause an interrupt
          mecs          When set, this field enables an interrupt to be raised 
                        upon reception of an MECS with a command value that is 
                        enabled
          pwRx          When set, this field enables the reception of a port-write 
                        to cause an interrupt
          localLog      When set, this field enables an implementation-specific
                        Logical/Transport Layer Error to cause an interrupt
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
 *   @n SRIO_RIO_EM_INT_ENABLE_LOG,   SRIO_RIO_EM_INT_ENABLE_MECS, 
 *   @n SRIO_RIO_EM_INT_ENABLE_PW_RX, SRIO_RIO_EM_INT_ENABLE_LOCALOG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management Interrupts for logical layer events only
        CSL_SRIO_SetEventMgmtInterrupts(hSrio, 1, 0, 0, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtInterrupts
(
    CSL_SrioHandle      hSrio,
    Uint8               log,
    Uint8               mecs,
    Uint8               pwRx,
    Uint8               localLog
)
{
    hSrio->RIO_EM_INT_ENABLE = 
        CSL_FMK (SRIO_RIO_EM_INT_ENABLE_LOG, log)           |
        CSL_FMK (SRIO_RIO_EM_INT_ENABLE_MECS, mecs)         |
        CSL_FMK (SRIO_RIO_EM_INT_ENABLE_PW_RX, pwRx)        |
        CSL_FMK (SRIO_RIO_EM_INT_ENABLE_LOCALOG, localLog);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtInterruptPortStatus
 *
 *   @b Description
 *   @n The function is used to get the Event Management Interrupt port status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          irqPending    Per-port interrupt pending status
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
 *   @n SRIO_RIO_EM_INT_PORT_STAT_IRQ_PENDING
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               irqPending;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Port Interrupt Status
        CSL_SRIO_GetEventMgmtInterruptPortStatus(hSrio, &irqPending);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtInterruptPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              irqPending
)
{
    *irqPending = CSL_FEXT (hSrio->RIO_EM_INT_PORT_STAT, SRIO_RIO_EM_INT_PORT_STAT_IRQ_PENDING);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtPortWriteStatus
 *
 *   @b Description
 *   @n The function is used to get the Event Management Port Write Status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          port          When set, a port-specific event has been detected
          log           An enabled logical layer event has been detected
          rcs           A reset request has been detected by a port
          multiportErr  Indicates that multiple ports have detected errors 
                        which use port-write notification
          localLog      Implementation-specific local Logical/Transport Layer Error
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
 *   @n SRIO_RIO_EM_PW_STAT_PORT,   SRIO_RIO_EM_PW_STAT_LOG, 
 *   @n SRIO_RIO_EM_PW_STAT_RCS,    SRIO_RIO_EM_PW_STAT_MULTIPORT_ERR,
 *   @n SRIO_RIO_EM_PW_STAT_LOCALOG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               port;
        Uint8               log;
        Uint8               rcs;
        Uint8               multiportErr;
        Uint8               localLog;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Port Status 
        CSL_SRIO_GetEventMgmtPortWriteStatus(hSrio, &port, &log, &rcs, &multiportErr, &localLog);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtPortWriteStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              port,
    Uint8*              log,
    Uint8*              rcs,
    Uint8*              multiportErr,
    Uint8*              localLog
)
{
    Uint32  value = hSrio->RIO_EM_PW_STAT;

    *port           = CSL_FEXT (value, SRIO_RIO_EM_PW_STAT_PORT);
    *log            = CSL_FEXT (value, SRIO_RIO_EM_PW_STAT_LOG);
    *rcs            = CSL_FEXT (value, SRIO_RIO_EM_PW_STAT_RCS);
    *multiportErr   = CSL_FEXT (value, SRIO_RIO_EM_PW_STAT_MULTIPORT_ERR);
    *localLog       = CSL_FEXT (value, SRIO_RIO_EM_PW_STAT_LOCALOG);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtPortWriteEnable
 *
 *   @b Description
 *   @n The function is used to get the Event Management Port Write Enable
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          log           When set, this field enables a logical layer event 
                        to cause a port-write to be sent.
          localLog      When set, this field enables an implementation-specific
                        Logical/Transport Layer Error to cause a port-write 
                        to be sent
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
 *   @n SRIO_RIO_EM_PW_EN_LOG, SRIO_RIO_EM_PW_EN_LOCALOG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               log;
        Uint8               localLog;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Port Enable 
        CSL_SRIO_GetEventMgmtPortWriteEnable(hSrio, &log, &localLog);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtPortWriteEnable
(
    CSL_SrioHandle      hSrio,
    Uint8*              log,
    Uint8*              localLog
)
{
    Uint32  value = hSrio->RIO_EM_PW_EN;

    *log            = CSL_FEXT (value, SRIO_RIO_EM_PW_EN_LOG);
    *localLog       = CSL_FEXT (value, SRIO_RIO_EM_PW_EN_LOCALOG);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtPortWriteEnable
 *
 *   @b Description
 *   @n The function is used to set the Event Management Port Write Enable
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          log           When set, this field enables a logical layer event 
                        to cause a port-write to be sent.
          localLog      When set, this field enables an implementation-specific
                        Logical/Transport Layer Error to cause a port-write 
                        to be sent
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
 *   @n SRIO_RIO_EM_PW_EN_LOG, SRIO_RIO_EM_PW_EN_LOCALOG
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management Port Enable for Log only events
        CSL_SRIO_SetEventMgmtPortWriteEnable(hSrio, 1, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtPortWriteEnable
(
    CSL_SrioHandle      hSrio,
    Uint8               log,
    Uint8               localLog
)
{
    hSrio->RIO_EM_PW_EN = 
            CSL_FMK (SRIO_RIO_EM_PW_EN_LOG,     log)    |
            CSL_FMK (SRIO_RIO_EM_PW_EN_LOCALOG, localLog);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtPortWritePendingStatus
 *
 *   @b Description
 *   @n The function is used to get the Event Management Port Write Status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          pwPending     Per-port Port Write pending status
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
 *   @n SRIO_RIO_EM_PW_PORT_STAT_PW_PENDING
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               pwPending;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Port Write status
        CSL_SRIO_GetEventMgmtPortWritePendingStatus(hSrio, &pwPending);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtPortWritePendingStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              pwPending
)
{
    *pwPending = CSL_FEXT (hSrio->RIO_EM_PW_PORT_STAT, SRIO_RIO_EM_PW_PORT_STAT_PW_PENDING);
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableEventMgmtInterrupts
 *
 *   @b Description
 *   @n The function is used to enable the Event Management Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
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
 *   @n SRIO_RIO_EM_DEV_INT_EN_INT_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the Event Management Interrupts
        CSL_SRIO_EnableEventMgmtInterrupts(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableEventMgmtInterrupts
(
    CSL_SrioHandle      hSrio
)
{
    CSL_FINS (hSrio->RIO_EM_DEV_INT_EN, SRIO_RIO_EM_DEV_INT_EN_INT_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableEventMgmtInterrupts
 *
 *   @b Description
 *   @n The function is used to disable the Event Management Interrupts
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
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
 *   @n SRIO_RIO_EM_DEV_INT_EN_INT_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the Event Management Interrupts
        CSL_SRIO_DisableEventMgmtInterrupts(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableEventMgmtInterrupts
(
    CSL_SrioHandle      hSrio
)
{
    CSL_FINS (hSrio->RIO_EM_DEV_INT_EN, SRIO_RIO_EM_DEV_INT_EN_INT_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsEventMgmtInterruptsEnabled
 *
 *   @b Description
 *   @n The function is used to check if the Event Management Interrupts are 
 *      enabled or disabled?
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    - Event Management Interrupts are enabled
 *   @n FALSE   - Event Management Interrupts are disabled
 *
 *   <b> Pre Condition </b>
 *   @n  CSL_SRIO_Open() must be called 
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n SRIO_RIO_EM_DEV_INT_EN_INT_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the Event Management Interrupts
        if (CSL_SRIO_IsEventMgmtInterruptsEnabled(hSrio) == TRUE)
        {
            // Event Management Interrupts Enabled
        }
        else
        {
            // Event Management Interrupts Disabled        
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsEventMgmtInterruptsEnabled
(
    CSL_SrioHandle      hSrio
)
{
    if (CSL_FEXT(hSrio->RIO_EM_DEV_INT_EN, SRIO_RIO_EM_DEV_INT_EN_INT_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_EnableEventMgmtPortWrite
 *
 *   @b Description
 *   @n The function is used to enable the Event Management Port Write
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
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
 *   @n SRIO_RIO_EM_DEV_PW_EN_PW_EN=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Enable the Event Management Port Writes
        CSL_SRIO_EnableEventMgmtPortWrite(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_EnableEventMgmtPortWrite
(
    CSL_SrioHandle      hSrio
)
{
    CSL_FINS (hSrio->RIO_EM_DEV_PW_EN, SRIO_RIO_EM_DEV_PW_EN_PW_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_SRIO_DisableEventMgmtPortWrite
 *
 *   @b Description
 *   @n The function is used to disable the Event Management Port Write
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
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
 *   @n SRIO_RIO_EM_DEV_PW_EN_PW_EN=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Disable the Event Management Port Writes
        CSL_SRIO_DisableEventMgmtPortWrite(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_DisableEventMgmtPortWrite
(
    CSL_SrioHandle      hSrio
)
{
    CSL_FINS (hSrio->RIO_EM_DEV_PW_EN, SRIO_RIO_EM_DEV_PW_EN_PW_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_SRIO_IsEventMgmtPortWriteEnabled
 *
 *   @b Description
 *   @n The function is used to check if the Event Management Port Write is 
 *      enabled or not?
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
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
 *   @n SRIO_RIO_EM_DEV_PW_EN_PW_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Check if the Event Management Port Writes are enabled or not?
        if (CSL_SRIO_IsEventMgmtPortWriteEnabled(hSrio) == TRUE)
        {
            // Event Management Port Writes are enabled
        }
        else
        {
            // Event Management Port Writes are disabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_SRIO_IsEventMgmtPortWriteEnabled
(
    CSL_SrioHandle      hSrio
)
{
    if (CSL_FEXT(hSrio->RIO_EM_DEV_PW_EN, SRIO_RIO_EM_DEV_PW_EN_PW_EN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtMECSStatus
 *
 *   @b Description
 *   @n The function is used to get the event management MECS status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          mecsStatus    MECS Command received by any SRIO port
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
 *   @n SRIO_RIO_EM_MECS_STAT_CMD_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               mecsStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Status
        CSL_SRIO_GetEventMgmtMECSStatus(hSrio, &mecsStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtMECSStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              mecsStatus
)
{
    *mecsStatus = CSL_FEXT (hSrio->RIO_EM_MECS_STAT, SRIO_RIO_EM_MECS_STAT_CMD_STAT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearEventMgmtMECSStatus
 *
 *   @b Description
 *   @n The function is used to clear the event management MECS status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          mecsStatus    MECS Command received by any SRIO port
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
 *   @n SRIO_RIO_EM_MECS_STAT_CMD_STAT
 *
 *   @b Affects
 *   @n SRIO_RIO_EM_MECS_STAT_CMD_STAT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               mecsStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Status
        CSL_SRIO_GetEventMgmtMECSStatus(hSrio, &mecsStatus);
        if (mecsStatus != 0)
        {
            // MECS with a command has been received.
            ...
            // Clear the MECS Event Management Status
            CSL_SRIO_ClearEventMgmtMECSStatus (hSrio, mecsStatus);
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearEventMgmtMECSStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               mecsStatus
)
{
    CSL_FINS(hSrio->RIO_EM_MECS_STAT, SRIO_RIO_EM_MECS_STAT_CMD_STAT, mecsStatus);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtMECSInterrupt
 *
 *   @b Description
 *   @n The function is used to get the event management MECS interrupt enable
 *      status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          mecsStatus    Enables MECS with set cmd to raise an interrupt request
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
 *   @n SRIO_RIO_EM_MECS_INT_EN_CMD_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               mecsStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Interrupt status
        CSL_SRIO_GetEventMgmtMECSInterrupt(hSrio, &mecsStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtMECSInterrupt
(
    CSL_SrioHandle      hSrio,
    Uint8*              mecsStatus
)
{
    *mecsStatus = CSL_FEXT (hSrio->RIO_EM_MECS_INT_EN, SRIO_RIO_EM_MECS_INT_EN_CMD_EN);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtMECSInterrupt
 *
 *   @b Description
 *   @n The function is used to set the event management MECS interrupt enable
 *      status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          mecsStatus    Enables MECS with set cmd to raise an interrupt request
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
 *   @n SRIO_RIO_EM_MECS_INT_EN_CMD_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management MECS Interrupt status for all cmds
        CSL_SRIO_SetEventMgmtMECSInterrupt(hSrio, 0xFF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtMECSInterrupt
(
    CSL_SrioHandle      hSrio,
    Uint8               mecsStatus
)
{
    CSL_FINS (hSrio->RIO_EM_MECS_INT_EN, SRIO_RIO_EM_MECS_INT_EN_CMD_EN, mecsStatus);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtMECSCaptureOut
 *
 *   @b Description
 *   @n The function is used to get the event management MECS capture out 
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cmdEn         Enables the associated bit to toggle
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
 *   @n SRIO_RIO_EM_MECS_CAP_EN_CMD_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               cmdEn;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Capture out
        CSL_SRIO_GetEventMgmtMECSCaptureOut(hSrio, &cmdEn);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtMECSCaptureOut
(
    CSL_SrioHandle      hSrio,
    Uint8*              cmdEn
)
{
    *cmdEn = CSL_FEXT (hSrio->RIO_EM_MECS_CAP_EN, SRIO_RIO_EM_MECS_CAP_EN_CMD_EN);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtMECSCaptureOut
 *
 *   @b Description
 *   @n The function is used to set the event management MECS capture out 
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cmdEn         Enables the associated bit to toggle
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
 *   @n SRIO_RIO_EM_MECS_CAP_EN_CMD_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management MECS Capture out for all commands
        CSL_SRIO_SetEventMgmtMECSCaptureOut(hSrio, 0xFF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtMECSCaptureOut
(
    CSL_SrioHandle      hSrio,
    Uint8               cmdEn
)
{
    CSL_FINS (hSrio->RIO_EM_MECS_CAP_EN, SRIO_RIO_EM_MECS_CAP_EN_CMD_EN, cmdEn);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtMECSTriggerIn
 *
 *   @b Description
 *   @n The function is used to get the event management MECS Trigger In
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cmdStat       Bit is set whenever an edge is detected on corresponding
          cmdEn         Enables the associated bit to trigger a MECS command
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
 *   @n SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT, SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               cmdEn;
        Uint8               cmdStat;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Trigger In
        CSL_SRIO_GetEventMgmtMECSTriggerIn(hSrio, &cmdStat, &cmdEn);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtMECSTriggerIn
(
    CSL_SrioHandle      hSrio,
    Uint8*              cmdStat,
    Uint8*              cmdEn
)
{
    Uint32 value = hSrio->RIO_EM_MECS_TRIG_EN;
    *cmdStat = CSL_FEXT (value, SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT);
    *cmdEn   = CSL_FEXT (value, SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtMECSTriggerIn
 *
 *   @b Description
 *   @n The function is used to set the event management MECS Trigger In
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cmdStat       Bit is set whenever an edge is detected on corresponding
          cmdEn         Enables the associated bit to trigger a MECS command
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
 *   @n SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT, SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               cmdEn;
        Uint8               cmdStat;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Trigger In
        CSL_SRIO_GetEventMgmtMECSTriggerIn(hSrio, &cmdStat, &cmdEn);
        ...
        // Set the Event Management MECS Trigger In to enable all MECS cmds
        CSL_SRIO_SetEventMgmtMECSTriggerIn(hSrio, cmdStat, 0xFF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtMECSTriggerIn
(
    CSL_SrioHandle      hSrio,
    Uint8               cmdStat,
    Uint8               cmdEn
)
{
    hSrio->RIO_EM_MECS_TRIG_EN = 
        CSL_FMK (SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT, cmdStat)    |
        CSL_FMK (SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN,   cmdEn);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtMECSRequest
 *
 *   @b Description
 *   @n The function is used to get the event management MECS Request
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          sendStatus    Send Status 
          cmd           MECS command to be sent
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
 *   @n SRIO_RIO_EM_MECS_REQ_SEND, SRIO_RIO_EM_MECS_REQ_CMD
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               sendStatus;
        Uint8               cmd;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Trigger In
        CSL_SRIO_GetEventMgmtMECSRequest(hSrio, &cmdStat, &cmdEn);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtMECSRequest
(
    CSL_SrioHandle      hSrio,
    Uint8*              sendStatus,
    Uint8*              cmd
)
{
    Uint32 value = hSrio->RIO_EM_MECS_REQ;
    *sendStatus = CSL_FEXT (value, SRIO_RIO_EM_MECS_REQ_SEND);
    *cmd        = CSL_FEXT (value, SRIO_RIO_EM_MECS_REQ_CMD);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtMECSRequest
 *
 *   @b Description
 *   @n The function is used to set the event management MECS Request
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          sendStatus    Send Status 
          cmd           MECS command to be sent
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
 *   @n SRIO_RIO_EM_MECS_REQ_SEND, SRIO_RIO_EM_MECS_REQ_CMD
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               sendStatus;
        Uint8               cmd;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Send the MECS Command 2.
        CSL_SRIO_SetEventMgmtMECSRequest(hSrio, 0x1, 0x2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtMECSRequest
(
    CSL_SrioHandle      hSrio,
    Uint8               sendStatus,
    Uint8               cmd
)
{
    hSrio->RIO_EM_MECS_REQ = 
            CSL_FMK (SRIO_RIO_EM_MECS_REQ_SEND, sendStatus)     |
            CSL_FMK (SRIO_RIO_EM_MECS_REQ_CMD, cmd);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtMECSPortStatus
 *
 *   @b Description
 *   @n The function is used to get the event management MECS Port Status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          port          MECS Received Status
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
 *   @n SRIO_RIO_EM_MECS_PORT_STAT_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               port;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Port Status
        CSL_SRIO_GetEventMgmtMECSPortStatus(hSrio, &port);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtMECSPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              port
)
{
    *port = CSL_FEXT (hSrio->RIO_EM_MECS_PORT_STAT, SRIO_RIO_EM_MECS_PORT_STAT_PORT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtMECSPortStatus
 *
 *   @b Description
 *   @n The function is used to set the event management MECS Port Status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          port          MECS Received Status
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
 *   @n SRIO_RIO_EM_MECS_PORT_STAT_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               port;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Port Status
        CSL_SRIO_GetEventMgmtMECSPortStatus(hSrio, &port);
        ...
        // Clear the Event Management MECS Port Status
        CSL_SRIO_SetEventMgmtMECSPortStatus (hSrio, port);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtMECSPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               port
)
{
    CSL_FINS(hSrio->RIO_EM_MECS_PORT_STAT, SRIO_RIO_EM_MECS_PORT_STAT_PORT, port);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtCmdStatus
 *
 *   @b Description
 *   @n The function is used to get the event management command status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cmdStat       MECS Command which were received 
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
 *   @n SRIO_RIO_EM_RST_PORT_STAT_RST_REQ
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               cmdStat;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Command Status
        CSL_SRIO_GetEventMgmtCmdStatus(hSrio, &cmdStat);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtCmdStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              cmdStat
)
{
    *cmdStat = CSL_FEXT (hSrio->RIO_EM_MECS_STAT, SRIO_RIO_EM_MECS_STAT_CMD_STAT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtCmdStatus
 *
 *   @b Description
 *   @n The function is used to set the event management command status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cmdStat       MECS Command to be configured
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
 *   @n SRIO_RIO_EM_MECS_STAT_CMD_STAT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               cmdStat;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Command Status
        CSL_SRIO_GetEventMgmtCmdStatus(hSrio, &cmdStat);
        ...
        // Clear the Event Management Command Status
        CSL_SRIO_SetEventMgmtCmdStatus(hSrio, cmdStat);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtCmdStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               cmdStat
)
{
    CSL_FINS (hSrio->RIO_EM_MECS_STAT, SRIO_RIO_EM_MECS_STAT_CMD_STAT, cmdStat);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtResetRequest
 *
 *   @b Description
 *   @n The function is used to get the event management reset request
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          rstReq        Per-port Reset Control Symbol event received status
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
 *   @n SRIO_RIO_EM_RST_PORT_STAT_RST_REQ
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               rstReq;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management Reset Request Status
        CSL_SRIO_GetEventMgmtResetRequest(hSrio, &rstReq);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtResetRequest
(
    CSL_SrioHandle      hSrio,
    Uint8*              rstReq
)
{
    *rstReq = CSL_FEXT (hSrio->RIO_EM_RST_PORT_STAT, SRIO_RIO_EM_RST_PORT_STAT_RST_REQ);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtResetRequest
 *
 *   @b Description
 *   @n The function is used to set the event management reset request
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          rstReq        Per-port Reset Control Symbol event received status
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
 *   @n SRIO_RIO_EM_RST_PORT_STAT_RST_REQ
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               rstReq;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management Reset Request Status 
        // and clear all the reset events
        CSL_SRIO_SetEventMgmtResetRequest(hSrio, 0xFF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtResetRequest
(
    CSL_SrioHandle      hSrio,
    Uint8               rstReq
)
{
    CSL_FINS (hSrio->RIO_EM_RST_PORT_STAT, SRIO_RIO_EM_RST_PORT_STAT_RST_REQ, rstReq);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtResetRequestInterrupt
 *
 *   @b Description
 *   @n The function is used to get the event management reset request interrupt
 *      enable status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          rstIntEn      Per-port Reset Request event interrupt enable
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
 *   @n SRIO_RIO_EM_RST_INT_EN_RST_INT_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               rstReq;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Reset Request Status
        CSL_SRIO_GetEventMgmtResetRequestInterrupt(hSrio, &rstReq);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtResetRequestInterrupt
(
    CSL_SrioHandle      hSrio,
    Uint8*              rstIntEn
)
{
    *rstIntEn = CSL_FEXT (hSrio->RIO_EM_RST_INT_EN, SRIO_RIO_EM_RST_INT_EN_RST_INT_EN);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtResetRequestInterrupt
 *
 *   @b Description
 *   @n The function is used to set the event management reset request interrupt
 *      enable status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          rstIntEn      Per-port Reset Request event interrupt enable
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
 *   @n SRIO_RIO_EM_RST_INT_EN_RST_INT_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management Reset Request Interrupt Enable for all ports
        CSL_SRIO_SetEventMgmtResetRequestInterrupt(hSrio, 0xF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtResetRequestInterrupt
(
    CSL_SrioHandle      hSrio,
    Uint8               rstIntEn
)
{
    CSL_FINS (hSrio->RIO_EM_RST_INT_EN, SRIO_RIO_EM_RST_INT_EN_RST_INT_EN, rstIntEn);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetEventMgmtResetRequestPortWrite
 *
 *   @b Description
 *   @n The function is used to get the event management reset request port write
 *      enable status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          rstPwEn       Per-port Reset Request port-write enable
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
 *   @n SRIO_RIO_EM_RST_PW_EN_RST_PW_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               rstReq;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Event Management MECS Reset Request Port Write Status
        CSL_SRIO_GetEventMgmtResetRequestPortWrite(hSrio, &rstReq);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetEventMgmtResetRequestPortWrite
(
    CSL_SrioHandle      hSrio,
    Uint8*              rstPwEn
)
{
    *rstPwEn = CSL_FEXT (hSrio->RIO_EM_RST_PW_EN, SRIO_RIO_EM_RST_PW_EN_RST_PW_EN);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetEventMgmtResetRequestPortWrite
 *
 *   @b Description
 *   @n The function is used to set the event management reset request port write
 *      enable status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          rstPwEn       Per-port Reset Request port-write enable
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
 *   @n SRIO_RIO_EM_RST_PW_EN_RST_PW_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               rstReq;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Event Management MECS Reset Request Port Write Status to
        // enable sending port write for reset request on all ports
        CSL_SRIO_SetEventMgmtResetRequestPortWrite(hSrio, 0xF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetEventMgmtResetRequestPortWrite
(
    CSL_SrioHandle      hSrio,
    Uint8               rstPwEn
)
{
    CSL_FINS (hSrio->RIO_EM_RST_PW_EN, SRIO_RIO_EM_RST_PW_EN_RST_PW_EN, rstPwEn);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortWriteControl
 *
 *   @b Description
 *   @n The function is used to get the port write control information
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          pwTimer       Port Write Timer
          pwcMode       Port Write Capture Mode
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
 *   @n SRIO_RIO_PW_CTL_PW_TIMER, SRIO_RIO_PW_CTL_PWC_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               pwTimer;
        Uint8               pwcMode;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Control
        CSL_SRIO_GetPortWriteControl(hSrio, &pwTimer, &pwcMode);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortWriteControl
(
    CSL_SrioHandle      hSrio,
    Uint8*              pwTimer,
    Uint8*              pwcMode
)
{
    *pwTimer = CSL_FEXT (hSrio->RIO_PW_CTL, SRIO_RIO_PW_CTL_PW_TIMER);
    *pwcMode = CSL_FEXT (hSrio->RIO_PW_CTL, SRIO_RIO_PW_CTL_PWC_MODE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortWriteControl
 *
 *   @b Description
 *   @n The function is used to set the port write control information
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          pwTimer       Port Write Timer
          pwcMode       Port Write Capture Mode
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
 *   @n SRIO_RIO_PW_CTL_PW_TIMER, SRIO_RIO_PW_CTL_PWC_MODE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Port Write Control to disable port write timer and to enable
        // continuous port capture 
        CSL_SRIO_SetPortWriteControl(hSrio, 0, 0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortWriteControl
(
    CSL_SrioHandle      hSrio,
    Uint8               pwTimer,
    Uint8               pwcMode
)
{
    hSrio->RIO_PW_CTL = 
            CSL_FMK (SRIO_RIO_PW_CTL_PW_TIMER, pwTimer)     |
            CSL_FMK (SRIO_RIO_PW_CTL_PWC_MODE, pwcMode);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortWriteRouting
 *
 *   @b Description
 *   @n The function is used to get the port write routing
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          port          Indicates if a port-write should be sent to this port.
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
 *   @n SRIO_RIO_PW_ROUTE_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               port;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Control
        CSL_SRIO_GetPortWriteRouting(hSrio, &port);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortWriteRouting
(
    CSL_SrioHandle      hSrio,
    Uint8*              port
)
{
    *port = CSL_FEXT (hSrio->RIO_PW_ROUTE, SRIO_RIO_PW_ROUTE_PORT);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortWriteRouting
 *
 *   @b Description
 *   @n The function is used to set the port write routing
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          port          Indicates if a port-write should be sent to this port.
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
 *   @n SRIO_RIO_PW_ROUTE_PORT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               port;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Port Write Routing to ensure port writes are sent on all ports
        CSL_SRIO_SetPortWriteRouting(hSrio, 0xF);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortWriteRouting
(
    CSL_SrioHandle      hSrio,
    Uint8               port
)
{
    CSL_FINS (hSrio->RIO_PW_ROUTE, SRIO_RIO_PW_ROUTE_PORT, port);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortWriteReceptionStatus
 *
 *   @b Description
 *   @n The function is used to get the port write reception status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          wrSize        WRSIZE, in combination with WDPTR, are used to 
                        determine the maximum size of the port-write which has 
                        been received
          wdPtr         WDPTR, in combination with WRSIZE, are used to 
                        determine the maximum size of the port-write which has 
                        been received
          pwDisc        Port-write was discarded
          pwVal         Port-write data registers contain valid data
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
 *   @n SRIO_RIO_PW_RX_STAT_WR_SIZE,  SRIO_RIO_PW_RX_STAT_WDPTR,
 *   @n SRIO_RIO_PW_RX_STAT_PW_DISC, SRIO_RIO_PW_RX_STAT_PW_VAL,
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               wrSize;
        Uint8               wdPtr;
        Uint8               pwDisc;
        Uint8               pwVal;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Reception Status
        CSL_SRIO_GetPortWriteReceptionStatus(hSrio, &wrSize, &wdPtr, &pwDisc, &pwVal);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortWriteReceptionStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              wrSize,
    Uint8*              wdPtr,
    Uint8*              pwDisc,
    Uint8*              pwVal
)
{
    Uint32 value = hSrio->RIO_PW_RX_STAT;
    *wrSize = CSL_FEXT (value, SRIO_RIO_PW_RX_STAT_WR_SIZE);
    *wdPtr  = CSL_FEXT (value, SRIO_RIO_PW_RX_STAT_WDPTR);
    *pwDisc = CSL_FEXT (value, SRIO_RIO_PW_RX_STAT_PW_DISC);
    *pwVal  = CSL_FEXT (value, SRIO_RIO_PW_RX_STAT_PW_VAL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortWriteEventGenerate
 *
 *   @b Description
 *   @n The function is used to get the port write reception status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          pwDisc        Port-write discard event generate status
          pwVal         Port-write data event generate status
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
 *   @n SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC, SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               pwDisc;
        Uint8               pwVal;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Event Generate Status
        CSL_SRIO_GetPortWriteEventGenerate(hSrio, &pwDisc, &pwVal);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortWriteEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8*              pwDisc,
    Uint8*              pwVal
)
{
    Uint32 value = hSrio->RIO_PW_RX_EVENT_GEN;
    *pwDisc = CSL_FEXT (value, SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC);
    *pwVal  = CSL_FEXT (value, SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortWriteEventGenerate
 *
 *   @b Description
 *   @n The function is used to set the port write reception status
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          pwDisc        Port-write discard event generate status
          pwVal         Port-write data event generate status
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
 *   @n SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC, SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL
 *   
 *   @b Affects
 *   @n SRIO_RIO_PW_RX_STAT_PW_DISC=1, SRIO_RIO_PW_RX_STAT_PW_VAL=1
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Port Write Event Generate Status
        CSL_SRIO_SetPortWriteEventGenerate(hSrio, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortWriteEventGenerate
(
    CSL_SrioHandle      hSrio,
    Uint8               pwDisc,
    Uint8               pwVal
)
{
    hSrio->RIO_PW_RX_EVENT_GEN = CSL_FMK(SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC, pwDisc) |
                                 CSL_FMK(SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL, pwVal);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortWriteReceptionCapture
 *
 *   @b Description
 *   @n The function is used to get the port write reception capture
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          cap0          Reception Capture 0
          cap1          Reception Capture 1
          cap2          Reception Capture 2
          cap3          Reception Capture 3
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
 *   @n SRIO_RIO_PW_RX_CAPT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              cap0;
        Uint32              cap1;
        Uint32              cap2;
        Uint32              cap3;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Reception Capture
        CSL_SRIO_GetPortWriteReceptionCapture(hSrio, &cap0, &cap1, &cap2, &cap3);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortWriteReceptionCapture
(
    CSL_SrioHandle      hSrio,
    Uint32*             cap0,
    Uint32*             cap1,
    Uint32*             cap2,
    Uint32*             cap3
)
{
    *cap0 = hSrio->RIO_PW_RX_CAPT[0];
    *cap1 = hSrio->RIO_PW_RX_CAPT[1];
    *cap2 = hSrio->RIO_PW_RX_CAPT[2];
    *cap3 = hSrio->RIO_PW_RX_CAPT[3];
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetPortWriteReceptionCapture
 *
 *   @b Description
 *   @n The function is used to set the port write reception capture
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          port          Port number for which the write reception capture needs
                        to be configured.
          cap           Reception Capture value
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
 *   @b Affects
 *   @n SRIO_RIO_PW_RX_CAPT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              cap0;
        Uint32              cap1;
        Uint32              cap2;
        Uint32              cap3;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Write Reception Capture
        CSL_SRIO_GetPortWriteReceptionCapture(hSrio, &cap0, &cap1, &cap2, &cap3);
        ...
        // Set the Port Write Reception Capture for port 0
        CSL_SRIO_SetPortWriteReceptionCapture(hSrio, 0, cap0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetPortWriteReceptionCapture
(
    CSL_SrioHandle      hSrio,
    Uint8               port,
    Uint32              cap
)
{
    CSL_FINS (hSrio->RIO_PW_RX_CAPT[port], SRIO_RIO_PW_RX_CAPT_PW_CAPT, cap);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetPortNumberInfo
 *
 *   @b Description
 *   @n The function is used to get the port number information
 *
 *   @b Arguments
     @verbatim
          hSrio         Handle of the SRIO device
          portNum       Port Number
          totalPort     Total number of ports
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
 *   @n SRIO_RIO_PORT_NUMBER_PORT_NUM,SRIO_RIO_PORT_NUMBER_PORT_TOTAL
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               portNum;
        Uint8               totalPort;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Port Information
        CSL_SRIO_GetPortNumberInfo(hSrio, &portNum, &totalPort);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetPortNumberInfo
(
    CSL_SrioHandle      hSrio,
    Uint8*              portNum,
    Uint8*              totalPort
)
{
    Uint32 value = hSrio->RIO_PORT_NUMBER;
    *portNum   = CSL_FEXT(value, SRIO_RIO_PORT_NUMBER_PORT_NUM);
    *totalPort = CSL_FEXT(value, SRIO_RIO_PORT_NUMBER_PORT_TOTAL);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLLMPortIPPrescalar
 *
 *   @b Description
 *   @n The function is used to get the LLM Port IP Prescalar
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          prescalarClock    Prescalar clock
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
 *   @n SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               prescalarClock;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the LLM Port IP Prescalar Clock.
        CSL_SRIO_GetLLMPortIPPrescalar(hSrio, &prescalarClock);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLLMPortIPPrescalar
(
    CSL_SrioHandle      hSrio,
    Uint8*              prescalarClock
)
{
    *prescalarClock = CSL_FEXT (hSrio->RIO_PRESCALAR_SRV_CLK, SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLLMPortIPPrescalar
 *
 *   @b Description
 *   @n The function is used to set the LLM Port IP Prescalar
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          prescalarClock    Prescalar clock
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
 *   @n SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               prescalarClock;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the LLM Port IP Prescalar Clock to the default value
        CSL_SRIO_SetLLMPortIPPrescalar(hSrio, 31);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLLMPortIPPrescalar
(
    CSL_SrioHandle      hSrio,
    Uint8               prescalarClock
)
{
    CSL_FINS (hSrio->RIO_PRESCALAR_SRV_CLK, SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK, prescalarClock);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLLMResetControl
 *
 *   @b Description
 *   @n The function is used to get the LLM Reset Control CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          clearSticky       Allows the SELF_RST and PWDN_PORT resets to 
                            clear sticky register bits in addition to the 
                            normal configuration registers
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
 *   @n SRIO_RIO_REG_RST_CTL_CLEAR_STICKY
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               clearSticky;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the LLM Port Reset Control CSR
        CSL_SRIO_GetLLMResetControl(hSrio, &clearSticky);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLLMResetControl
(
    CSL_SrioHandle      hSrio,
    Uint8*              clearSticky
)
{
    *clearSticky = CSL_FEXT (hSrio->RIO_REG_RST_CTL, SRIO_RIO_REG_RST_CTL_CLEAR_STICKY);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetLLMResetControl
 *
 *   @b Description
 *   @n The function is used to set the LLM Reset Control CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          clearSticky       Allows the SELF_RST and PWDN_PORT resets to 
                            clear sticky register bits in addition to the 
                            normal configuration registers
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
 *   @n SRIO_RIO_REG_RST_CTL_CLEAR_STICKY
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the LLM Port Reset Control CSR
        CSL_SRIO_SetLLMResetControl(hSrio, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetLLMResetControl
(
    CSL_SrioHandle      hSrio,
    Uint8               clearSticky
)
{
    CSL_FINS (hSrio->RIO_REG_RST_CTL, SRIO_RIO_REG_RST_CTL_CLEAR_STICKY, clearSticky);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorDetect
 *
 *   @b Description
 *   @n The function is used to get the error detect CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          illID             Illegal transaction target error
          illType           Unsupported Transaction
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
 *   @n SRIO_RIO_LOCAL_ERR_DET_ILL_ID, SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               illID;
        Uint8               illType;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Error Detect CSR Information
        CSL_SRIO_GetErrorDetect(hSrio, &illID, &illType);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorDetect
(
    CSL_SrioHandle      hSrio,
    Uint8*              illID,
    Uint8*              illType
)
{
    Uint32 value = hSrio->RIO_LOCAL_ERR_DET;

    *illID   = CSL_FEXT (value, SRIO_RIO_LOCAL_ERR_DET_ILL_ID);
    *illType = CSL_FEXT (value, SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetErrorDetect
 *
 *   @b Description
 *   @n The function is used to set the error detect CSR. 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          illID             Illegal transaction target error
          illType           Unsupported Transaction
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
 *   @n SRIO_RIO_LOCAL_ERR_DET_ILL_ID, SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
 
        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Error Detect CSR Information for illegal type
        CSL_SRIO_SetErrorDetect(hSrio, 0, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetErrorDetect
(
    CSL_SrioHandle      hSrio,
    Uint8               illID,
    Uint8               illType
)
{
    hSrio->RIO_LOCAL_ERR_DET = 
            CSL_FMK (SRIO_RIO_LOCAL_ERR_DET_ILL_ID,     illID)      |
            CSL_FMK (SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE,   illType);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetErrorDetectEnable
 *
 *   @b Description
 *   @n The function is used to get the error detect enable CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          illID             Illegal transaction target error
          illType           Unsupported Transaction
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
 *   @n SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN, SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               illID;
        Uint8               illType;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Error Detect Enable CSR Information
        CSL_SRIO_GetErrorDetectEnable(hSrio, &illID, &illType);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetErrorDetectEnable
(
    CSL_SrioHandle      hSrio,
    Uint8*              illID,
    Uint8*              illType
)
{
    Uint32 value = hSrio->RIO_LOCAL_ERR_EN;

    *illID   = CSL_FEXT (value, SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN);
    *illType = CSL_FEXT (value, SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN);
}

/** ============================================================================
 *   @n@b CSL_SRIO_SetErrorDetectEnable
 *
 *   @b Description
 *   @n The function is used to set the error detect enable CSR
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          illID             Illegal transaction target error
          illType           Unsupported Transaction
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
 *   @n SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN, SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Set the Error Detect Enable CSR Information for both illegal id and type
        CSL_SRIO_SetErrorDetectEnable(hSrio, 1, 1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_SetErrorDetectEnable
(
    CSL_SrioHandle      hSrio,
    Uint8               illID,
    Uint8               illType
)
{
    hSrio->RIO_LOCAL_ERR_EN = 
            CSL_FMK (SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN,   illID)    |
            CSL_FMK (SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN, illType);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetHiAddressCapture
 *
 *   @b Description
 *   @n The function is used to get the High Address capture CSR information. 
 *      The contents of this register are only valid when the device is using 
 *      50- or 66-bit addressing
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          address           Address Capture information
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
 *   @n SRIO_RIO_LOCAL_H_ADDR_CAPT
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              address;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Address Capture Information
        CSL_SRIO_GetHiAddressCapture(hSrio, &address);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetHiAddressCapture
(
    CSL_SrioHandle      hSrio,
    Uint32*             address
)
{
    *address   = hSrio->RIO_LOCAL_H_ADDR_CAPT;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLoAddressCapture
 *
 *   @b Description
 *   @n The function is used to get the Low Address capture CSR information. 
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          address           Address Capture information
          xambs             Extended address bits of the address associated 
                            with the error
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
 *   @n SRIO_RIO_LOCAL_ADDR_CAPT_ADDR, SRIO_RIO_LOCAL_ADDR_CAPT_XAMSBS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              address;
        Uint8               xambs;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Address Capture Information
        CSL_SRIO_GetLoAddressCapture(hSrio, &address, &xambs);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLoAddressCapture
(
    CSL_SrioHandle      hSrio,
    Uint32*             address,
    Uint8*              xambs
)
{
    Uint32 value = hSrio->RIO_LOCAL_ADDR_CAPT;
    *address = CSL_FEXT (value, SRIO_RIO_LOCAL_ADDR_CAPT_ADDR);
    *xambs   = CSL_FEXT (value, SRIO_RIO_LOCAL_ADDR_CAPT_XAMSBS);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearLoAddressCapture
 *
 *   @b Description
 *   @n The function is used to clear the Low Address capture CSR information. 
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
 *   @n SRIO_RIO_LOCAL_ADDR_CAPT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint32              address;
        Uint8               xambs;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Address Capture Information
        CSL_SRIO_GetLoAddressCapture(hSrio, &address, &xambs);
        ...
        // Clear the Address capture information
        CSL_SRIO_ClearLoAddressCapture(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearLoAddressCapture
(
    CSL_SrioHandle      hSrio
)
{
    hSrio->RIO_LOCAL_ADDR_CAPT = 0;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetDeviceIDCapture
 *
 *   @b Description
 *   @n The function is used to get the Device ID capture CSR information.
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          msbDstID          MSB Destination ID
          lsbDstID          LSB Destination ID
          msbSrcID          MSB Source ID
          lsbSrcID          LSB Source ID
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
 *   @n SRIO_RIO_LOCAL_ID_CAPT_MSB_DEST_ID, SRIO_RIO_LOCAL_ID_CAPT_DEST_ID,SRIO_RIO_LOCAL_ID_CAPT_MSB_SRC_ID, SRIO_RIO_LOCAL_ID_CAPT_SRC_ID
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               msbDstID;
        Uint8               lsbDstID;
        Uint8               msbSrcID;
        Uint8               lsbSrcID;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Device ID Capture information
        CSL_SRIO_GetDeviceIDCapture(hSrio, &msbDstID, &lsbDstID, &msbSrcID, &lsbSrcID);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetDeviceIDCapture
(
    CSL_SrioHandle      hSrio,
    Uint8*              msbDstID,
    Uint8*              lsbDstID,
    Uint8*              msbSrcID,
    Uint8*              lsbSrcID
)
{
    Uint32 value = hSrio->RIO_LOCAL_ID_CAPT;

    *msbDstID  = CSL_FEXT (value, SRIO_RIO_LOCAL_ID_CAPT_MSB_DEST_ID);
    *lsbDstID  = CSL_FEXT (value, SRIO_RIO_LOCAL_ID_CAPT_DEST_ID);
    *msbSrcID  = CSL_FEXT (value, SRIO_RIO_LOCAL_ID_CAPT_MSB_SRC_ID);
    *lsbSrcID  = CSL_FEXT (value, SRIO_RIO_LOCAL_ID_CAPT_SRC_ID);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearDeviceIDCapture
 *
 *   @b Description
 *   @n The function is used to clear the Device ID capture CSR information
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
 *   @n SRIO_RIO_LOCAL_ID_CAPT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               msbDstID;
        Uint8               lsbDstID;
        Uint8               msbSrcID;
        Uint8               lsbSrcID;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Device ID Capture information
        CSL_SRIO_GetDeviceIDCapture(hSrio, &msbDstID, &lsbDstID, &msbSrcID, &lsbSrcID);
        ...
        // Clear the Address capture information
        CSL_SRIO_ClearDeviceIDCapture(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearDeviceIDCapture
(
    CSL_SrioHandle      hSrio
)
{
    hSrio->RIO_LOCAL_ID_CAPT = 0;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetLocalControlCapture
 *
 *   @b Description
 *   @n The function is used to get the Local Control capture CSR information
 *
 *   @b Arguments
     @verbatim
          hSrio             Handle of the SRIO device
          ftype             FTYPE associated with the error.
          ttype             TTYPE associated with the error.
          msgInfo           Messages are not generated by SRIO-TEV2 and message
                            information is not logged
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
 *   @n SRIO_RIO_LOCAL_CTRL_CAPT_FTYPE,SRIO_RIO_LOCAL_CTRL_CAPT_TTYPE,
 *   @n SRIO_RIO_LOCAL_CTRL_CAPT_MESSAGE_INFO
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               ftype;
        Uint8               ttype;
        Uint8               msgInfo;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Device ID Capture information
        CSL_SRIO_GetLocalControlCapture(hSrio, &ftype, &ttype, &msgInfo);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetLocalControlCapture
(
    CSL_SrioHandle      hSrio,
    Uint8*              ftype,
    Uint8*              ttype,
    Uint8*              msgInfo
)
{
    Uint32 value = hSrio->RIO_LOCAL_CTRL_CAPT;

    *ftype   = CSL_FEXT (value, SRIO_RIO_LOCAL_CTRL_CAPT_FTYPE);
    *ttype   = CSL_FEXT (value, SRIO_RIO_LOCAL_CTRL_CAPT_TTYPE);
    *msgInfo = CSL_FEXT (value, SRIO_RIO_LOCAL_CTRL_CAPT_MESSAGE_INFO);
}

/** ============================================================================
 *   @n@b CSL_SRIO_ClearLocalControlCapture
 *
 *   @b Description
 *   @n The function is used to clear the Local Control capture CSR information
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
 *   @n SRIO_RIO_LOCAL_CTRL_CAPT=0
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               ftype;
        Uint8               ttype;
        Uint8               msgInfo;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Local control Capture information
        CSL_SRIO_GetLocalControlCapture(hSrio, &ftype, &ttype, &msgInfo);
        ...
        // Clear the local control capture
        CSL_SRIO_ClearLocalControlCapture(hSrio);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_ClearLocalControlCapture
(
    CSL_SrioHandle      hSrio
)
{
    hSrio->RIO_LOCAL_CTRL_CAPT = 0;
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetFabricControlStatus
 *
 *   @b Description
 *   @n The function is used to get the Fabric control status information.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          igLLMBackPressure     Ingress LLM backpressure indication
          igUCBackPressure      Ingress User Core backpressure indication
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
 *   @n SRIO_RIO_FABRIC_CSR_IG_LLM_BACKPRESSURE,SRIO_RIO_FABRIC_CSR_IG_UC_BACKPRESSURE
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igLLMBackPressure;
        Uint8               igUCBackPressure;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Fabric Control Status
        CSL_SRIO_GetFabricControlStatus(hSrio, &igLLMBackPressure, &igUCBackPressure);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetFabricControlStatus
(
    CSL_SrioHandle      hSrio,
    Uint8*              igLLMBackPressure,
    Uint8*              igUCBackPressure
)
{
    Uint32 value = hSrio->RIO_FABRIC_CSR;

    *igLLMBackPressure  = CSL_FEXT (value, SRIO_RIO_FABRIC_CSR_IG_LLM_BACKPRESSURE);
    *igUCBackPressure   = CSL_FEXT (value, SRIO_RIO_FABRIC_CSR_IG_UC_BACKPRESSURE);
}

/** ============================================================================
 *   @n@b CSL_SRIO_GetFabricControlPortStatus
 *
 *   @b Description
 *   @n The function is used to get the Fabric control port status information.
 *
 *   @b Arguments
     @verbatim
          hSrio                 Handle of the SRIO device
          port                  SRIO port number
          igPktEnableStatus     PBMi Packet Enable Indication
          egPktEnableStatus     PBMe Packet Enable Indication
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
 *   @n SRIO_RIO_SP_FABRIC_STATUS_IG_PKT_ENABLE_STATUS,
 *   @n SRIO_RIO_SP_FABRIC_STATUS_EG_PKT_ENABLE_STATUS
 *
 *   @b Example
 *   @verbatim
        CSL_SrioHandle      hSrio;
        Uint8               igPktEnableStatus;
        Uint8               egPktEnableStatus;

        // Open the CSL SRIO Module 0
        hSrio = CSL_SRIO_Open (0);

        // Get the Fabric Control Port Status
        CSL_SRIO_GetFabricControlPortStatus(hSrio, 1, &igPktEnableStatus, &egPktEnableStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_SRIO_GetFabricControlPortStatus
(
    CSL_SrioHandle      hSrio,
    Uint8               port,
    Uint8*              igPktEnableStatus,
    Uint8*              egPktEnableStatus
)
{
    Uint32 value = hSrio->RIO_SP_FABRIC_STATUS[port];

    *igPktEnableStatus  = CSL_FEXT (value, SRIO_RIO_SP_FABRIC_STATUS_IG_PKT_ENABLE_STATUS);
    *egPktEnableStatus  = CSL_FEXT (value, SRIO_RIO_SP_FABRIC_STATUS_EG_PKT_ENABLE_STATUS);
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _CSL_SRIO_AUX_PHY_LAYER_H_ */


