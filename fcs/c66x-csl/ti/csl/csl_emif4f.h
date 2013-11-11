/**
 *   @file  csl_emif4.h
 *
 *   @brief   
 *      This is the main header file for the EMIF4F Module which defines
 *      all the data structures and exported API.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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

/** @defgroup CSL_EMIF4F_API EMIF4F
 *
 * @section Introduction
 *  The External Memory Interface (EMIF) is a TI developed re-usable IP component 
 *  targeted for SOC designs. The EMIF is a VBUSM slave peripheral providing an 
 *  interface to a wide variety of DDR SDRAM. This memory controller is a soft 
 *  macro and must be used with the DDR PHY hard macros to interface to the 
 *  DDR SDRAM. 
 *
 * @subsection References
 *   -# EMIF4F Functional Specification
 */

#ifndef _CSL_EMIF4F_H_
#define _CSL_EMIF4F_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_emif4f.h>

/**
@defgroup CSL_EMIF4F_SYMBOL  EMIF4F Symbols Defined
@ingroup CSL_EMIF4F_API
*/
/**
@defgroup CSL_EMIF4F_DATASTRUCT  EMIF4F Data Structures
@ingroup CSL_EMIF4F_API
*/
/**
@defgroup CSL_EMIF4F_FUNCTION  EMIF4F Functions
@ingroup CSL_EMIF4F_API
*/

/**
@addtogroup CSL_EMIF4F_SYMBOL
@{
*/

/**
 *  Handle to access EMIF4F registers accessible through config bus.
 */
#define hEmif     ((CSL_Emif4fRegs*)CSL_DDR3_EMIF_CONFIG_REGS)

/**
@}
*/

/** @addtogroup CSL_EMIF4F_DATASTRUCT
 @{ */

/** @brief EMIF4F SDRAM Configuration
 *
 * This structure is used to define the EMIF4F SDRAM 
 * Configuration
 */
typedef struct EMIF4F_SDRAM_CONFIG
{
    Uint8   type;
    Uint8   iBankPos;
    Uint8   ddrTerm;
    Uint8   ddrDDQS;
    Uint8   dynODT;
    Uint8   disableDLL;
    Uint8   SDRAMDrive;
    Uint8   CASWriteLat;
    Uint8   narrowMode;
    Uint8   CASLatency;
    Uint8   rowSize;
    Uint8   iBank;
    Uint8   eBank;
    Uint8   pageSize;

    Uint8   NVMEnable;
    Uint8   eBankPos;
    Uint8   rdbNum;
    Uint8   rdbSize;
}EMIF4F_SDRAM_CONFIG;

/** @brief EMIF4F Timing1 Configuration
 *
 * This structure is used to define the EMIF4F Timing1
 * Configuration
 */
typedef struct EMIF4F_TIMING1_CONFIG
{
    Uint8   t_rp;
    Uint8   t_rcd;
    Uint8   t_wr;
    Uint8   t_ras;
    Uint8   t_rc;
    Uint8   t_rrd;
    Uint8   t_wtr;
}EMIF4F_TIMING1_CONFIG;

/** @brief EMIF4F Timing2 Configuration
 *
 * This structure is used to define the EMIF4F Timing2
 * Configuration
 */
typedef struct EMIF4F_TIMING2_CONFIG
{
    Uint8   t_xp;
    Uint8   t_odt;
    Uint16  t_xsnr;
    Uint16  t_xsrd;
    Uint8   t_rtp;
    Uint8   t_cke;
}EMIF4F_TIMING2_CONFIG;

/** @brief EMIF4F Timing3 Configuration
 *
 * This structure is used to define the EMIF4F Timing3
 * Configuration
 */
typedef struct EMIF4F_TIMING3_CONFIG
{
    Uint8   t_pdll_ul;
    Uint8   t_csta;
    Uint8   t_ckesr;
    Uint8   zq_zqcs;
    Uint8   t_tdqsckmax;
    Uint16  t_rfc;
    Uint8   t_rasMax;
}EMIF4F_TIMING3_CONFIG;

/** @brief LPDDR2-NVM Timing Configuration
 *
 * This structure is used to define the LPDDR2-NVM Timing
 * Configuration
 */
typedef struct EMIF4F_LPDDR2NVM_TIMING_CONFIG
{
    Uint8   t_xp;
    Uint8   t_wtr;
    Uint8   t_rp;
    Uint8   t_wra;
    Uint8   t_rrd;
    Uint8   t_rcdmin;
}EMIF4F_LPDDR2NVM_TIMING_CONFIG;

/** @brief Power Management Configuration
 *
 * This structure is used to define the Power Management Configuration
 */
typedef struct EMIF4F_PWR_MGMT_CONFIG
{
    Uint8   pdTime;
    Uint8   dpdEnable;
    Uint8   lpMode;
    Uint8   srTime;
    Uint8   csTime;
}EMIF4F_PWR_MGMT_CONFIG;

/** @brief VBUS Configuration Values
 *
 * This structure is used to define the VBUS Configuration Values
 */
typedef struct EMIF4F_VBUS_CONFIG_VALUE
{
    Uint8   sysBusWidth;
    Uint8   statFIFODepth;
    Uint8   wrFIFODepth;
    Uint8   cmdFIFODepth;
    Uint8   rregFIFODepth;
    Uint8   rsdFIFODepth;
    Uint8   rcmdFIFODepth;
}EMIF4F_VBUS_CONFIG_VALUE;

/** @brief IODFT Control Values
 *
 * This structure is used to define the IODFT Test Logic Global
 * Control Values.
 */
typedef struct EMIF4F_IODFT_CONTROL
{
    Uint16  tlec;
    Uint8   mt;
    Uint8   actCapEn;
    Uint8   opgld;
    Uint8   mms;
    Uint8   mc;
    Uint8   pc;
    Uint8   tm;
}EMIF4F_IODFT_CONTROL;

/** @brief Performance Counter Configuration
 *
 * This structure is used to define the performance counter 
 * configuration
 */
typedef struct EMIF4F_PERF_CONFIG
{
    Uint8  mstIDEn;
    Uint8  regEn;
    Uint8  cntCfg;
}EMIF4F_PERF_CONFIG;

/** @brief SDRAM Output Impedance Calibration Configuation
 *
 * This structure is used to define the SDRAM Output Impedance 
 * Calibration Configuration.
 */
typedef struct EMIF4F_OUTPUT_IMP_CONFIG
{
    Uint8   zqCS1En;
    Uint8   zqCS0En;
    Uint8   zqDualCSEn;
    Uint8   zqSFEXITEn;
    Uint8   zqZQCLInterval;
    Uint8   zqZQCLMult;
    Uint16  zqRefInterval;
}EMIF4F_OUTPUT_IMP_CONFIG;

/** @brief Temperature Alert Configuration
 *
 * This structure is used to define the Temperature Alert Configuration.
 */
typedef struct EMIF4F_TEMP_ALERT_CONFIG
{
    Uint8   taCS1En;
    Uint8   taCS0En;
    Uint8   taSFEXITEn;
    Uint8   taDevWdth;
    Uint8   taDevCnt;
    Uint32  taRefInterval;
}EMIF4F_TEMP_ALERT_CONFIG;

/** @brief Priority to COS Mapping
 *
 * This structure is used to define the Priority to COS Mapping.
 */
typedef struct EMIF4_PRI_COS_MAPPING
{
    Uint8   enable;
    Uint8   pri7cos;
    Uint8   pri6cos;
    Uint8   pri5cos;
    Uint8   pri4cos;
    Uint8   pri3cos;
    Uint8   pri2cos;
    Uint8   pri1cos;
    Uint8   pri0cos;
}EMIF4_PRI_COS_MAPPING;

/** @brief Master ID to COS Mapping
 *
 * This structure is used to define the Master ID to COS Mapping.
 */
typedef struct EMIF4_MSTID_COS_MAPPING
{
    Uint8   enable;
    Uint8   mst1;
    Uint8   mstMask1;
    Uint8   mst2;
    Uint8   mstMask2;
    Uint8   mst3;
    Uint8   mstMask3;
}EMIF4_MSTID_COS_MAPPING;

/** @brief ECC Control 
 *
 * This structure is used to define the ECC Control Configuration.
 */
typedef struct EMIF4_ECC_CONTROL
{
    Uint8   enable;
    Uint8   addrRngProt;
    Uint8   addrRng2En;
    Uint8   addrRng1En;
}EMIF4_ECC_CONTROL;

/* @} */

/** @addtogroup CSL_EMIF4F_FUNCTION
 @{ */

/* @} */


#ifdef __cplusplus
}
#endif
#endif /* _CSL_EMIF4F_H_ */
