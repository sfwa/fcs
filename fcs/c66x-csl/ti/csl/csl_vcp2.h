/* ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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
 *   @file  csl_vcp2.h
 *
 *   @brief  
 *      Header file with data structure and API declarations for VCP2 
 *      CSL module.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par
 */
 
/** @defgroup CSL_VCP2_API VCP2
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * Channel decoding of voice and low bit-rate data channels found in cellular standards such as 2.5G, 3G,
 * and WiMAX requires the decoding of convolutional encoded data. The Viterbi-decoder coprocessor 2
 * (VCP2) provided in the TCI648x devices performs Viterbi decoding for IS2000 and 3GPP wireless
 * standards. The VCP2 coprocessor also performs forward-error correction for 2G and 3G wireless systems.
 * The VCP2 coprocessor offers a very cost effective and synergistic solution when combined with Texas
 * Instruments (TI) DSPs. The VCP2 supports 1941 12.2 Kbps class A 3G voice channels running at
 * 333 MHz.
 *
 * @subsection References
 *    -# VCP2 User's Guide SPRUE09A.pdf (May 2006 - Revised April 2008)
 *
 */
#ifndef _CSL_VCP2_H_
#define _CSL_VCP2_H_

#include <ti/csl/csl.h>
#include <ti/csl/cslr_vcp2.h>
#include <ti/csl/soc.h>

/**
@defgroup CSL_VCP2_SYMBOL  VCP2 Symbols Defined
@ingroup CSL_VCP2_API
*/
/**
@defgroup CSL_VCP2_DATASTRUCT  VCP2 Data Structures
@ingroup CSL_VCP2_API
*/
/**
@defgroup CSL_VCP2_FUNCTION  VCP2 Functions
@ingroup CSL_VCP2_API
*/

#ifdef __cplusplus
extern "C"
{
#endif

/**
@addtogroup CSL_VCP2_SYMBOL
@{
*/

/******************************************************************************
 *  VCP Macros
 *****************************************************************************/
/** Code rate = 2 */
#define VCP2_RATE_1_2                2
/** Code rate = 3 */
#define VCP2_RATE_1_3                3
/** Code rate = 4 */
#define VCP2_RATE_1_4                4

/** Speed critical */
#define VCP2_SPEED_CRITICAL          0
/** Speed most critical */
#define VCP2_SPEED_MOST_CRITICAL     1
/** Performance critical */
#define VCP2_PERF_CRITICAL           2
/** Performance most critical */
#define VCP2_PERF_MOST_CRITICAL      3
/** Default value */
#define VCP2_PERF_DEFAULT            VCP2_SPEED_CRITICAL

/** Out order of VCP output for decoded data : 0 to 31 */
#define VCP2_OUTORDER_0_31           CSL_VCP2_VCPIC3_OUT_ORDER_LSB
/** Out order of VCP output for decoded data : 31 to 0 */
#define VCP2_OUTORDER_31_0           CSL_VCP2_VCPIC3_OUT_ORDER_MSB

/** Output decision type : Hard decisions */
#define VCP2_DECISION_HARD           CSL_VCP2_VCPIC5_SDHD_HARD
/** Output decision type : Soft decisions */
#define VCP2_DECISION_SOFT           CSL_VCP2_VCPIC5_SDHD_SOFT

/** Output parameters read flag : VCP read event is not generated */
#define VCP2_OUTF_NO                 CSL_VCP2_VCPIC5_OUTF_NO
/** Output parameters read flag : VCP read event is generated */
#define VCP2_OUTF_YES                CSL_VCP2_VCPIC5_OUTF_YES

/** No trace back allowed */
#define VCP2_TRACEBACK_NONE          CSL_VCP2_VCPIC5_TB_NO
/** Traceback mode : Tailed */
#define VCP2_TRACEBACK_TAILED        CSL_VCP2_VCPIC5_TB_TAIL
/** Traceback mode : Convergent */
#define VCP2_TRACEBACK_CONVERGENT    CSL_VCP2_VCPIC5_TB_CONV
/** Traceback mode : Mixed */
#define VCP2_TRACEBACK_MIXED         CSL_VCP2_VCPIC5_TB_MIX

/**
 * VCP unpause type : VCP restarts and processes one sliding window before
 * pausing again
 */
#define VCP2_UNPAUSE_ONESW           CSL_VCP2_VCPEXE_COMMAND_RESTART_PAUSE
/** VCP unpause type : VCP restarts */
#define VCP2_UNPAUSE_NORMAL          CSL_VCP2_VCPEXE_COMMAND_RESTART

/** Soft decisions memory format : 32-bit word packed */
#define VCP2_END_PACKED32            CSL_VCP2_VCPEND_SD_32BIT
/** Soft decisions memory format : Native (8 bits) */
#define VCP2_END_NATIVE              CSL_VCP2_VCPEND_SD_NATIVE

/** EMU mode : VCP halts at the end of completion of the current window of
 *  state metric processing or at the end of a frame */
#define VCP2_EMUHALT_DEFAULT         CSL_VCP2_VCPEMU_SOFT_HALT_DEFAULT
/**
 * EMU mode : VCP halts at the end of completion of the processing of the
 * frame
 */
#define VCP2_EMUHALT_FRAMEEND        CSL_VCP2_VCPEMU_SOFT_HALT_FRAMEEND

/***************************************************************************
 *  Following are Polynomials used in GSM/Edge/GPRS
 **************************************************************************/

/**
 *  GSM/Edge/GPRS generator polynomial 0
 */
#define VCP2_GEN_POLY_0    0x30

/**
 *  GSM/Edge/GPRS generator polynomial 1
 */
#define VCP2_GEN_POLY_1    0xB0

/**
 *  GSM/Edge/GPRS generator polynomial 2
 */
#define VCP2_GEN_POLY_2    0x50

/**
 *  GSM/Edge/GPRS generator polynomial 3
 */
#define VCP2_GEN_POLY_3    0xF0

/**
 *  GSM/Edge/GPRS generator polynomial 4
 */
#define VCP2_GEN_POLY_4    0x6C

/**
 *  GSM/Edge/GPRS generator polynomial 5
 */
#define VCP2_GEN_POLY_5    0x94

/**
 *  GSM/Edge/GPRS generator polynomial 6
 */
#define VCP2_GEN_POLY_6    0xF4

/**
 *  GSM/Edge/GPRS generator polynomial 7
 */
#define VCP2_GEN_POLY_7    0xE4

/**
 *  NULL generator polynomial for GSM/Edge/GPRS
 */
#define VCP2_GEN_POLY_GNULL 0x00

/**
@}
*/

/** @addtogroup CSL_VCP2_DATASTRUCT
 @{ */

/** VCP Peripheral ID structure */
typedef struct
{
    /** Peripheral type */
    Uint8 type;
    /** Peripheral class */
    Uint8 pid_class;
    /** Peripheral revision */
    Uint8 rev;
} VCP2_PID;

/**
 *  VCP code rate type
 */
typedef Uint32 VCP2_Rate;

/**
 *  VCP input configuration structure that holds all of the configuration
 *  values that are to be transferred to the VCP via the EDMA.
 */
typedef struct
{
    /** Value of VCP input configuration register 0 */
    Uint32 ic0;
    /** Value of VCP input configuration register 1 */
    Uint32 ic1;
    /** Value of VCP input configuration register 2 */
    Uint32 ic2;
    /** Value of VCP input configuration register 3 */
    Uint32 ic3;
    /** Value of VCP input configuration register 4 */
    Uint32 ic4;
    /** Value of VCP input configuration register 5 */
    Uint32 ic5;
} VCP2_ConfigIc;

/**
 *  VCP channel parameters structure that holds all of the information
 *  concerning the user channel. These values are used to generate the
 *  appropriate input configuration values for the VCP and to program
 *  the EDMA.
 */
typedef struct
{
    /** Code rate */
    VCP2_Rate rate;
    /** Constraint length */
    Uint8 constLen;
    /** Polynomial 0 */
    Uint8 poly0;
    /** Polynomial 1 */
    Uint8 poly1;
    /** Polynomial 2 */
    Uint8 poly2;
    /** Polynomial 3 */
    Uint8 poly3;
    /** Yamamoto threshold value*/
    Uint16 yamTh;
    /** Frame length i.e. number of symbols in a frame*/
    Uint16 frameLen;
    /** Reliability length */
    Uint16 relLen;
    /** Convergence distance */
    Uint16 convDist;
    /** Traceback state index */
    Uint16 traceBackIndex;
    /** Traceback state index enable/disable */
    Bool traceBackEn;
    /** Hard decision output ordering */
    Uint16 outOrder;
    /** Maximum initial state metric */
    Int16 maxSm;
    /** Minimum initial state metric */
    Int16 minSm;
    /** State index set to the maximum initial state metric*/
    Uint8 stateNum;
    /** Branch metrics buffer length in input FIFO */
    Uint8 bmBuffLen;
    /** Decisions buffer length in output FIFO */
    Uint8 decBuffLen;
    /** Traceback mode */
    Uint8 traceBack;
    /** Output parameters read flag */
    Uint8 readFlag;
    /** Decision selection: hard or soft */
    Uint8 decision;
    /** Number of branch metric frames*/
    Uint16 numBmFrames;
    /** Number of decision frames */
    Uint16 numDecFrames;
} VCP2_Params;

/**
 *  VCP base parameter structure that is used to configure the VCP parameters
 *  structure with the given values using VCP2_genParams() function.
 */
typedef struct
{
    /** Code rate */
    VCP2_Rate rate;
    /** Constraint length */
    Uint8 constLen;
    /** Frame length */
    Uint16 frameLen;
    /** Yamamoto threshold value */
    Uint16 yamTh;
    /** Maximum initial state metric value */
    Uint8 stateNum;
    /** Traceback convergement mode */
    Bool tbConvrgMode;
    /** Output decision type */
    Uint8 decision;
    /** Output parameters read flag */
    Uint8 readFlag;
    /** Enable/Disable tail biting */
    Bool tailBitEnable;
    /** Tailbiting traceback index mode */
    Uint16 traceBackIndex;
    /** Hard decision output ordering */
    Uint8 outOrder;
    /** Performance and speed */
    Uint8 perf;
} VCP2_BaseParams;

/** VCP Error structure */
typedef struct
{
    /** Traceback mode error */
    Bool tbnaErr;
    /** Frame length error */
    Bool ftlErr;
    /** Reliability + convergence distance error */
    Bool fctlErr;
    /** Max-Min error */
    Bool maxminErr;
    /** SYMX error */
    Bool symxErr;
    /** SYMR error */
    Bool symrErr;
} VCP2_Errors;

/** VCP generator polynomials structure */
typedef struct
{
    /** Generator polynomial 0 */
    Uint8 poly0;
    /** Generator polynomial 1 */
    Uint8 poly1;
    /** Generator polynomial 2 */
    Uint8 poly2;
    /** Generator polynomial 3 */
    Uint8 poly3;
} VCP2_Poly;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_Vcp2EdmaRegs           *CSL_Vcp2RegsOvly;
typedef volatile CSL_Vcp2ConfigRegs         *CSL_Vcp2CfgRegsOvly;


/**
 * This will have the base-address information for the peripheral instance
 */
typedef struct {
    /** Base-address of the registers of VCP2  */
    CSL_Vcp2RegsOvly    regs;
    /** Base-address of the Configuration registers of VCP2  */
    CSL_Vcp2CfgRegsOvly cfgregs;
} VCP2BaseAddress;

/**
 * This structure/object holds the context of the instance of VCP2
 * opened using VCP2_init() function.
 *
 * Pointer to this object is passed as VCP2 Handle to all VCP2 CSL APIs.
 * VCP2_init() function intializes this structure based on the parameters
 * passed
 */
typedef struct VCP2Obj {
    /** Pointer to the register overlay structure of the Tcp2 */
    CSL_Vcp2RegsOvly    regs;
    /** Pointer to the Configuration registers overlay structure of the Tcp2 */
    CSL_Vcp2CfgRegsOvly cfgregs;
    /** Instance of TCP2 being referred by this object  */
    CSL_InstNum         perNum;
} VCP2Obj;

/** This is a pointer to Vcp2Obj and is passed as the first
 *  parameter to all VCP2 CSL APIs
 */
typedef struct VCP2Obj *VCP2Handle;

/* @} */

/** @addtogroup CSL_VCP2_FUNCTION
 @{ */

/******************************************************************************
 * VCP2 global function declarations
 *****************************************************************************/

extern void VCP2_genParams (
    VCP2_BaseParams * pConfigBase,
    VCP2_Params     * pConfigParams
);

extern void VCP2_genIc (
    VCP2_Params     * pConfigParams,
    VCP2_ConfigIc   * pConfigIc
);

extern VCP2Handle VCP2_init(
    VCP2Obj         * pVcp2Obj,
    int             instNum, 
    int             * pStatus
);

extern CSL_Status  VCP2_Close (
    VCP2Handle      hVcp2
);

/* @} */
#ifdef __cplusplus
}
#endif

#endif
