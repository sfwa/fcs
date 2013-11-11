/* =============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2002, 2003, 2004, 2005, 2006
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
/** ============================================================================
 *   @file  cslr_i2c.h
 *
 *   @path  $(CSLPATH)
 *
 *   @desc  This file contains the Register Desciptions for I2C
 *  ============================================================================
 */
#ifndef _CSLR_I2C_H_
#define _CSLR_I2C_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>


/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 ICOAR;
    volatile Uint32 ICIMR;
    volatile Uint32 ICSTR;
    volatile Uint32 ICCLKL;
    volatile Uint32 ICCLKH;
    volatile Uint32 ICCNT;
    volatile Uint32 ICDRR;
    volatile Uint32 ICSAR;
    volatile Uint32 ICDXR;
    volatile Uint32 ICMDR;
    volatile Uint32 ICIVR;
    volatile Uint32 ICEMDR;
    volatile Uint32 ICPSC;
    volatile Uint32 ICPID1;
    volatile Uint32 ICPID2;
} CSL_I2cRegs;

typedef volatile CSL_I2cRegs             *CSL_I2cRegsOvly;


/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* ICOAR */

#define CSL_I2C_ICOAR_OADDR_MASK         (0x000003FFu)
#define CSL_I2C_ICOAR_OADDR_SHIFT        (0x00000000u)
#define CSL_I2C_ICOAR_OADDR_RESETVAL     (0x00000000u)

#define CSL_I2C_ICOAR_RESETVAL           (0x00000000u)

/* ICIMR */

#define CSL_I2C_ICIMR_AAS_MASK           (0x00000040u)
#define CSL_I2C_ICIMR_AAS_SHIFT          (0x00000006u)
#define CSL_I2C_ICIMR_AAS_RESETVAL       (0x00000000u)

/*----AAS Tokens----*/
#define CSL_I2C_ICIMR_AAS_DISABLE        (0x00000000u)
#define CSL_I2C_ICIMR_AAS_ENABLE         (0x00000001u)

#define CSL_I2C_ICIMR_SCD_MASK           (0x00000020u)
#define CSL_I2C_ICIMR_SCD_SHIFT          (0x00000005u)
#define CSL_I2C_ICIMR_SCD_RESETVAL       (0x00000000u)

/*----SCD Tokens----*/
#define CSL_I2C_ICIMR_SCD_DISABLE        (0x00000000u)
#define CSL_I2C_ICIMR_SCD_ENABLE         (0x00000001u)

#define CSL_I2C_ICIMR_ICXRDY_MASK        (0x00000010u)
#define CSL_I2C_ICIMR_ICXRDY_SHIFT       (0x00000004u)
#define CSL_I2C_ICIMR_ICXRDY_RESETVAL    (0x00000000u)

/*----ICXRDY Tokens----*/
#define CSL_I2C_ICIMR_ICXRDY_DISABLE     (0x00000000u)
#define CSL_I2C_ICIMR_ICXRDY_ENABLE      (0x00000001u)

#define CSL_I2C_ICIMR_ICRRDY_MASK        (0x00000008u)
#define CSL_I2C_ICIMR_ICRRDY_SHIFT       (0x00000003u)
#define CSL_I2C_ICIMR_ICRRDY_RESETVAL    (0x00000000u)

/*----ICRRDY Tokens----*/
#define CSL_I2C_ICIMR_ICRRDY_DISABLE     (0x00000000u)
#define CSL_I2C_ICIMR_ICRRDY_ENABLE      (0x00000001u)

#define CSL_I2C_ICIMR_ARDY_MASK          (0x00000004u)
#define CSL_I2C_ICIMR_ARDY_SHIFT         (0x00000002u)
#define CSL_I2C_ICIMR_ARDY_RESETVAL      (0x00000000u)

/*----ARDY Tokens----*/
#define CSL_I2C_ICIMR_ARDY_DISABLE       (0x00000000u)
#define CSL_I2C_ICIMR_ARDY_ENABLE        (0x00000001u)

#define CSL_I2C_ICIMR_NACK_MASK          (0x00000002u)
#define CSL_I2C_ICIMR_NACK_SHIFT         (0x00000001u)
#define CSL_I2C_ICIMR_NACK_RESETVAL      (0x00000000u)

/*----NACK Tokens----*/
#define CSL_I2C_ICIMR_NACK_DISABLE       (0x00000000u)
#define CSL_I2C_ICIMR_NACK_ENABLE        (0x00000001u)

#define CSL_I2C_ICIMR_AL_MASK            (0x00000001u)
#define CSL_I2C_ICIMR_AL_SHIFT           (0x00000000u)
#define CSL_I2C_ICIMR_AL_RESETVAL        (0x00000000u)

/*----AL Tokens----*/
#define CSL_I2C_ICIMR_AL_DISABLE         (0x00000000u)
#define CSL_I2C_ICIMR_AL_ENABLE          (0x00000001u)

#define CSL_I2C_ICIMR_RESETVAL           (0x00000000u)

/* ICSTR */

#define CSL_I2C_ICSTR_SDIR_MASK          (0x00004000u)
#define CSL_I2C_ICSTR_SDIR_SHIFT         (0x0000000Eu)
#define CSL_I2C_ICSTR_SDIR_RESETVAL      (0x00000000u)

#define CSL_I2C_ICSTR_NACKSNT_MASK       (0x00002000u)
#define CSL_I2C_ICSTR_NACKSNT_SHIFT      (0x0000000Du)
#define CSL_I2C_ICSTR_NACKSNT_RESETVAL   (0x00000000u)

#define CSL_I2C_ICSTR_BB_MASK            (0x00001000u)
#define CSL_I2C_ICSTR_BB_SHIFT           (0x0000000Cu)
#define CSL_I2C_ICSTR_BB_RESETVAL        (0x00000000u)

#define CSL_I2C_ICSTR_RSFULL_MASK        (0x00000800u)
#define CSL_I2C_ICSTR_RSFULL_SHIFT       (0x0000000Bu)
#define CSL_I2C_ICSTR_RSFULL_RESETVAL    (0x00000000u)

#define CSL_I2C_ICSTR_XSMT_MASK          (0x00000400u)
#define CSL_I2C_ICSTR_XSMT_SHIFT         (0x0000000Au)
#define CSL_I2C_ICSTR_XSMT_RESETVAL      (0x00000001u)

#define CSL_I2C_ICSTR_AAS_MASK           (0x00000200u)
#define CSL_I2C_ICSTR_AAS_SHIFT          (0x00000009u)
#define CSL_I2C_ICSTR_AAS_RESETVAL       (0x00000000u)

#define CSL_I2C_ICSTR_AD0_MASK           (0x00000100u)
#define CSL_I2C_ICSTR_AD0_SHIFT          (0x00000008u)
#define CSL_I2C_ICSTR_AD0_RESETVAL       (0x00000000u)

#define CSL_I2C_ICSTR_SCD_MASK           (0x00000020u)
#define CSL_I2C_ICSTR_SCD_SHIFT          (0x00000005u)
#define CSL_I2C_ICSTR_SCD_RESETVAL       (0x00000000u)

#define CSL_I2C_ICSTR_ICXRDY_MASK        (0x00000010u)
#define CSL_I2C_ICSTR_ICXRDY_SHIFT       (0x00000004u)
#define CSL_I2C_ICSTR_ICXRDY_RESETVAL    (0x00000001u)

#define CSL_I2C_ICSTR_ICRRDY_MASK        (0x00000008u)
#define CSL_I2C_ICSTR_ICRRDY_SHIFT       (0x00000003u)
#define CSL_I2C_ICSTR_ICRRDY_RESETVAL    (0x00000000u)

#define CSL_I2C_ICSTR_ARDY_MASK          (0x00000004u)
#define CSL_I2C_ICSTR_ARDY_SHIFT         (0x00000002u)
#define CSL_I2C_ICSTR_ARDY_RESETVAL      (0x00000000u)

#define CSL_I2C_ICSTR_NACK_MASK          (0x00000002u)
#define CSL_I2C_ICSTR_NACK_SHIFT         (0x00000001u)
#define CSL_I2C_ICSTR_NACK_RESETVAL      (0x00000000u)

#define CSL_I2C_ICSTR_AL_MASK            (0x00000001u)
#define CSL_I2C_ICSTR_AL_SHIFT           (0x00000000u)
#define CSL_I2C_ICSTR_AL_RESETVAL        (0x00000000u)

#define CSL_I2C_ICSTR_RESETVAL           (0x00000410u)

/* ICCLKL */

#define CSL_I2C_ICCLKL_ICCL_MASK         (0x0000FFFFu)
#define CSL_I2C_ICCLKL_ICCL_SHIFT        (0x00000000u)
#define CSL_I2C_ICCLKL_ICCL_RESETVAL     (0x00000000u)

#define CSL_I2C_ICCLKL_RESETVAL          (0x00000000u)

/* ICCLKH */

#define CSL_I2C_ICCLKH_ICCH_MASK         (0x0000FFFFu)
#define CSL_I2C_ICCLKH_ICCH_SHIFT        (0x00000000u)
#define CSL_I2C_ICCLKH_ICCH_RESETVAL     (0x00000000u)

#define CSL_I2C_ICCLKH_RESETVAL          (0x00000000u)

/* ICCNT */

#define CSL_I2C_ICCNT_ICDC_MASK          (0x0000FFFFu)
#define CSL_I2C_ICCNT_ICDC_SHIFT         (0x00000000u)
#define CSL_I2C_ICCNT_ICDC_RESETVAL      (0x00000000u)

#define CSL_I2C_ICCNT_RESETVAL           (0x00000000u)

/* ICDRR */

#define CSL_I2C_ICDRR_D_MASK             (0x000000FFu)
#define CSL_I2C_ICDRR_D_SHIFT            (0x00000000u)
#define CSL_I2C_ICDRR_D_RESETVAL         (0x00000000u)

#define CSL_I2C_ICDRR_RESETVAL           (0x00000000u)

/* ICSAR */

#define CSL_I2C_ICSAR_SADDR_MASK         (0x000003FFu)
#define CSL_I2C_ICSAR_SADDR_SHIFT        (0x00000000u)
#define CSL_I2C_ICSAR_SADDR_RESETVAL     (0x000003FFu)

#define CSL_I2C_ICSAR_RESETVAL           (0x000003FFu)

/* ICDXR */

#define CSL_I2C_ICDXR_D_MASK             (0x000000FFu)
#define CSL_I2C_ICDXR_D_SHIFT            (0x00000000u)
#define CSL_I2C_ICDXR_D_RESETVAL         (0x00000000u)

#define CSL_I2C_ICDXR_RESETVAL           (0x00000000u)

/* ICMDR */

#define CSL_I2C_ICMDR_NACKMOD_MASK       (0x00008000u)
#define CSL_I2C_ICMDR_NACKMOD_SHIFT      (0x0000000Fu)
#define CSL_I2C_ICMDR_NACKMOD_RESETVAL   (0x00000000u)

#define CSL_I2C_ICMDR_FREE_MASK          (0x00004000u)
#define CSL_I2C_ICMDR_FREE_SHIFT         (0x0000000Eu)
#define CSL_I2C_ICMDR_FREE_RESETVAL      (0x00000000u)

#define CSL_I2C_ICMDR_STT_MASK           (0x00002000u)
#define CSL_I2C_ICMDR_STT_SHIFT          (0x0000000Du)
#define CSL_I2C_ICMDR_STT_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_STP_MASK           (0x00000800u)
#define CSL_I2C_ICMDR_STP_SHIFT          (0x0000000Bu)
#define CSL_I2C_ICMDR_STP_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_MST_MASK           (0x00000400u)
#define CSL_I2C_ICMDR_MST_SHIFT          (0x0000000Au)
#define CSL_I2C_ICMDR_MST_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_TRX_MASK           (0x00000200u)
#define CSL_I2C_ICMDR_TRX_SHIFT          (0x00000009u)
#define CSL_I2C_ICMDR_TRX_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_XA_MASK            (0x00000100u)
#define CSL_I2C_ICMDR_XA_SHIFT           (0x00000008u)
#define CSL_I2C_ICMDR_XA_RESETVAL        (0x00000000u)

#define CSL_I2C_ICMDR_RM_MASK            (0x00000080u)
#define CSL_I2C_ICMDR_RM_SHIFT           (0x00000007u)
#define CSL_I2C_ICMDR_RM_RESETVAL        (0x00000000u)

#define CSL_I2C_ICMDR_DLB_MASK           (0x00000040u)
#define CSL_I2C_ICMDR_DLB_SHIFT          (0x00000006u)
#define CSL_I2C_ICMDR_DLB_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_IRS_MASK           (0x00000020u)
#define CSL_I2C_ICMDR_IRS_SHIFT          (0x00000005u)
#define CSL_I2C_ICMDR_IRS_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_STB_MASK           (0x00000010u)
#define CSL_I2C_ICMDR_STB_SHIFT          (0x00000004u)
#define CSL_I2C_ICMDR_STB_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_FDF_MASK           (0x00000008u)
#define CSL_I2C_ICMDR_FDF_SHIFT          (0x00000003u)
#define CSL_I2C_ICMDR_FDF_RESETVAL       (0x00000000u)

#define CSL_I2C_ICMDR_BC_MASK            (0x00000007u)
#define CSL_I2C_ICMDR_BC_SHIFT           (0x00000000u)
#define CSL_I2C_ICMDR_BC_RESETVAL        (0x00000000u)

#define CSL_I2C_ICMDR_RESETVAL           (0x00000000u)

/* ICIVR */

#define CSL_I2C_ICIVR_TESTMD_MASK        (0x00000F00u)
#define CSL_I2C_ICIVR_TESTMD_SHIFT       (0x00000008u)
#define CSL_I2C_ICIVR_TESTMD_RESETVAL    (0x00000000u)

#define CSL_I2C_ICIVR_INTCODE_MASK       (0x00000007u)
#define CSL_I2C_ICIVR_INTCODE_SHIFT      (0x00000000u)
#define CSL_I2C_ICIVR_INTCODE_RESETVAL   (0x00000000u)

/*----INTCODE Tokens----*/
#define CSL_I2C_ICIVR_INTCODE_NONE       (0x00000000u)
#define CSL_I2C_ICIVR_INTCODE_AL         (0x00000001u)
#define CSL_I2C_ICIVR_INTCODE_NACK       (0x00000002u)
#define CSL_I2C_ICIVR_INTCODE_RAR        (0x00000003u)
#define CSL_I2C_ICIVR_INTCODE_RDR        (0x00000004u)
#define CSL_I2C_ICIVR_INTCODE_TDR        (0x00000005u)
#define CSL_I2C_ICIVR_INTCODE_SCD        (0x00000006u)
#define CSL_I2C_ICIVR_INTCODE_AAS        (0x00000007u)

#define CSL_I2C_ICIVR_RESETVAL           (0x00000000u)

/* ICEMDR */

#define CSL_I2C_ICEMDR_BCM_MASK          (0x00000001u)
#define CSL_I2C_ICEMDR_BCM_SHIFT         (0x00000000u)
#define CSL_I2C_ICEMDR_BCM_RESETVAL      (0x00000001u)

#define CSL_I2C_ICEMDR_RESETVAL          (0x00000001u)

/* ICPSC */

#define CSL_I2C_ICPSC_IPSC_MASK          (0x000000FFu)
#define CSL_I2C_ICPSC_IPSC_SHIFT         (0x00000000u)
#define CSL_I2C_ICPSC_IPSC_RESETVAL      (0x00000000u)

#define CSL_I2C_ICPSC_RESETVAL           (0x00000000u)

/* ICPID1 */

#define CSL_I2C_ICPID1_CLASS_MASK        (0x0000FF00u)
#define CSL_I2C_ICPID1_CLASS_SHIFT       (0x00000008u)
#define CSL_I2C_ICPID1_CLASS_RESETVAL    (0x00000001u)

#define CSL_I2C_ICPID1_REVISION_MASK     (0x000000FFu)
#define CSL_I2C_ICPID1_REVISION_SHIFT    (0x00000000u)
#define CSL_I2C_ICPID1_REVISION_RESETVAL (0x00000025u)

#define CSL_I2C_ICPID1_RESETVAL          (0x00000105u)

/* ICPID2 */

#define CSL_I2C_ICPID2_TYPE_MASK         (0x000000FFu)
#define CSL_I2C_ICPID2_TYPE_SHIFT        (0x00000000u)
#define CSL_I2C_ICPID2_TYPE_RESETVAL     (0x00000005u)

#define CSL_I2C_ICPID2_RESETVAL          (0x00000005u)

#endif
