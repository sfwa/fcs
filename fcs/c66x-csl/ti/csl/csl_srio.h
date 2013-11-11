/**
 *   @file  csl_srio.h
 *
 *   @brief
 *      This is the header file for the SRIO CSL Functional Layer.
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

/** @defgroup CSL_SRIO_API SRIO
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *  RapidIOTM is a non-proprietary high-bandwidth system level interconnect. It is 
 *  intended to offer giga-byte per second performance levels for chip-to-chip and 
 *  board-to-board communication. Its layered architecture, allows a highly 
 *  scalable interconnect capable of future enhancements.
 *
 * @subsection References
 *   -# Rapid IO Module Specification
 *
 */
 
#ifndef _CSL_SRIO_H_
#define _CSL_SRIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_srio.h>

/**
@defgroup CSL_SRIO_SYMBOL  SRIO Symbols Defined
@ingroup CSL_SRIO_API
*/
/**
@defgroup CSL_SRIO_DATASTRUCT  SRIO Data Structures
@ingroup CSL_SRIO_API
*/
/**
@defgroup CSL_SRIO_FUNCTION  SRIO Functions
@ingroup CSL_SRIO_API
*/

/** @addtogroup CSL_SRIO_DATASTRUCT
 @{ */
        
/** @brief This is the handle to the SRIO instance */
typedef volatile CSL_SrioRegs*   CSL_SrioHandle;

/** @brief SRIO Message Description.
 *
 * This structure is used to define a SRIO message and is used by the
 * SRIO module to compare all received RIO packets to determine the 
 * receive queue where the packet is pushed to.
 */
typedef struct SRIO_MESSAGE
{
    Uint16  srcId;
    Uint8   mbx;
    Uint8   ltr;
    Uint8   mbxMask;
    Uint8   ltrMask;
    Uint8   segMap;
    Uint8   srcProm;
    Uint8   tt;
    Uint8   dstProm;
    Uint16  dstId;
    Uint8   flowId;
}SRIO_MESSAGE;

/** @brief SRIO Type 9 Message Description.
 *
 * This structure is used to define the TYPE9 Message. This is then
 * used to MAP Type9 messages to a specific destination queue.
 */
typedef struct SRIO_TYPE9_MESSAGE
{
    Uint16  srcId;
    Uint8   cos;
    Uint8   cosMask;
    Uint8   srcProm;
    Uint8   tt;
    Uint8   dstProm;
    Uint16  dstId;
    Uint16  streamId;
    Uint16  streamMask;
    Uint8   flowId;
}SRIO_TYPE9_MESSAGE;

/** @brief SRIO AMU Window
 *
 * This structure is used to define the SRIO AMU Window
 * properties.
 */
typedef struct SRIO_AMU_WINDOW
{
    Uint8   xambs;
    Uint8   paneCount;
    Uint16  paneSize;
    Uint8   winSize;
    Uint32  rapidIOMsb;
    Uint32  rapidIOLsb;
}SRIO_AMU_WINDOW;

/** @brief SRIO AMU Pane
 *
 * This structure is used to define the SRIO AMU Pane
 * properties.
 */
typedef struct SRIO_AMU_PANE
{
    Uint8   cmdEnc;
    Uint8   portId;
    Uint8   idSize;
    Uint16  dstID;
}SRIO_AMU_PANE;

/** @brief SRIO LSU Transfer
 *
 * This structure is used to configure LSU module for Transfer
 */
typedef struct SRIO_LSU_TRANSFER 
{
    Uint32  rapidIOMSB;
    Uint32  rapidIOLSB;
    Uint32  dspAddress;
    Uint32  bytecount;
    Uint8   doorbellValid;
    Uint8   intrRequest;
    Uint8   supInt;
    Uint8   xambs;
    Uint8   priority;
    Uint8   outPortID;
    Uint8   idSize;
    Uint8   srcIDMap;
    Uint16  dstID;
    Uint8   ttype;
    Uint8   ftype;
    Uint8   hopCount;
    Uint16  doorbellInfo;
}SRIO_LSU_TRANSFER;

/** @brief SRIO Processing Element Features
 *
 * This structure is used to configure the Tundra 
 * Processing Element Features.
 */
typedef struct SRIO_PE_FEATURES 
{
    Uint8   isBridge;
    Uint8   isEndpoint;
    Uint8   isProcessor;
    Uint8   isSwitch;
    Uint8   isMultiport;
    Uint8   isFlowArbiterationSupported;
    Uint8   isMulticastSupported;
    Uint8   isExtendedRouteConfigSupported;
    Uint8   isStandardRouteConfigSupported;
    Uint8   isFlowControlSupported;
    Uint8   isCRFSupported;
    Uint8   isCTLSSupported;
    Uint8   isExtendedFeaturePtrValid;
    Uint8   numAddressBitSupported;
}SRIO_PE_FEATURES;

/** @brief SRIO Operation Capability Register
 *
 * This structure is used to store the operation capability
 * configuration. 
 */
typedef struct SRIO_OP_CAR
{
    Uint8   gsmRead;
    Uint8   gsmInstrnRead;
    Uint8   gsmReadOwn;
    Uint8   gsmDataCacheInvalidate;
    Uint8   gsmCastout;
    Uint8   gsmDataCacheFlush;
    Uint8   gsmIORead;
    Uint8   gsmInstrnCacheInvalidate;
    Uint8   gsmTLBInvalidate;
    Uint8   gsmTLBSync;
    Uint8   dataStreamingTM;
    Uint8   dataStreamingSupport;
    Uint8   implnDefined;
    Uint8   readSupport;
    Uint8   writeSupport;
    Uint8   streamWriteSupport;
    Uint8   writeResponseSupport;
    Uint8   dataMessageSupport;
    Uint8   doorbellSupport;
    Uint8   atomicCompareSwapSupport;
    Uint8   atomicTestSwapSupport;
    Uint8   atomicIncSupport;
    Uint8   atomicDecSupport;
    Uint8   atomicSetSupport;
    Uint8   atomicClearSupport;
    Uint8   atomicSwapSupport;
    Uint8   portWriteOperationSupport;
    Uint8   implnDefined2;
}SRIO_OP_CAR;

/** @brief SRIO Error Rate 
 *
 * This structure is used to store the error rate CSR configuration
 * which is used to monitor and control the port physical layer
 * errors. 
 */
typedef struct SRIO_ERR_RATE
{
    Uint8   errRateBias;
    Uint8   errRateRecovery;
    Uint8   peak;
    Uint8   errRateCnt;
}SRIO_ERR_RATE;

/** @brief SRIO Lane Status
 *
 * This structure is used to store the lane status information.
 */
typedef struct SRIO_LANE_STATUS
{
    Uint8   portNum;
    Uint8   laneNum;
    Uint8   txType;
    Uint8   txMode;
    Uint8   rxType;
    Uint8   rxInv;
    Uint8   rxTrn;
    Uint8   rxSync;
    Uint8   rxReady;
    Uint8   errCnt;
    Uint8   chgSync;
    Uint8   chgTrn;
    Uint8   stat1;
    Uint8   stat2_7;
    Uint8   idle2;
    Uint8   infoOk;
    Uint8   chg;
    Uint8   implSpecific;
    Uint8   lpRxTrn;
    Uint8   lpWidth;
    Uint8   lpLaneNum;
    Uint8   lpTapM1;
    Uint8   lpTapP1;
    Uint8   lpScrm;
}SRIO_LANE_STATUS;

/** @brief SRIO Lane Status
 *
 * This structure is used to store the PLM Implementation specific
 * control register
 */
typedef struct SRIO_PLM_IMPL_CONTROL
{
    Uint8   payloadCapture;
    Uint8   useIdle2;
    Uint8   useIdle1;
    Uint8   dlbEn;
    Uint8   forceReinit;
    Uint8   softRstPort;
    Uint8   txBypass;
    Uint8   lineLoopbackMode;
    Uint8   portSelfReset;
    Uint8   selfReset;
    Uint8   swapTx;
    Uint8   swapRx;
    Uint8   dltThresh;
}SRIO_PLM_IMPL_CONTROL;

/** @brief SRIO PLM VMin Exponent
 *
 * This structure is used to store the PLM VMin Exponent configuration
 */
typedef struct SRIO_PLM_VMIN_EXPONENT
{
    Uint8   vminExp;
    Uint8   imax;
    Uint8   mmax;
}SRIO_PLM_VMIN_EXPONENT;

/** @brief SRIO PLM Polarity Control
 *
 * This structure is used to store the PLM Polarity Control
 */
typedef struct SRIO_PLM_POLARITY_CONTROL
{
    Uint8   tx3Pol;
    Uint8   tx2Pol;
    Uint8   tx1Pol;
    Uint8   tx0Pol;
    Uint8   rx3Pol;
    Uint8   rx2Pol;
    Uint8   rx1Pol;
    Uint8   rx0Pol;
}SRIO_PLM_POLARITY_CONTROL;

/** @brief SRIO PLM Control Symbol Configuration
 *
 * This structure is used to store the PLM Control Symbol configuration
 */
typedef struct SRIO_PLM_CONTROL_SYMBOL
{
    Uint8   stype0;
    Uint8   par0;
    Uint8   par1;
    Uint8   csEmb;
    Uint8   stype1;
    Uint8   cmd;
    Uint8   stype2;
    Uint8   parm;
}SRIO_PLM_CONTROL_SYMBOL;

/** @brief SRIO TLM Control Configuration
 *
 * This structure is used to store the SRIO TLM Control configuration
 */
typedef struct SRIO_TLM_CONTROL
{
    Uint8   portGroupSelect;
    Uint8   voqSelect;
    Uint8   tgtIDDis;
    Uint8   mtcTgtIDDis;
    Uint8   length;
}SRIO_TLM_CONTROL;

/**
@}
*/

/* Device specific API which opens the SRIO instance and returns a handle used in all subsequent calls */
extern CSL_SrioHandle CSL_SRIO_Open (Int32 instNum);

#ifdef __cplusplus
}
#endif

#endif /* _CSL_SRIO_H_ */


