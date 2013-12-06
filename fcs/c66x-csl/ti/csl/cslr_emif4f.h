/********************************************************************
* Copyright (C) 2003-2008 Texas Instruments Incorporated.
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
#ifndef _CSLR_EMIF4F_H_
#define _CSLR_EMIF4F_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */
#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>


/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 EMIF_MOD_ID_REV;
    volatile Uint32 STATUS;
    volatile Uint32 SDRAM_CONFIG;
    volatile Uint32 SDRAM_CONFIG_2;
    volatile Uint32 SDRAM_REF_CTRL;
    volatile Uint32 SDRAM_REF_CTRL_SHDW;
    volatile Uint32 SDRAM_TIM_1;
    volatile Uint32 SDRAM_TIM_1_SHDW;
    volatile Uint32 SDRAM_TIM_2;
    volatile Uint32 SDRAM_TIM_2_SHDW;
    volatile Uint32 SDRAM_TIM_3;
    volatile Uint32 SDRAM_TIM_3_SHDW;
    volatile Uint32 LPDDR2_NVM_TIM;
    volatile Uint32 LPDDR2_NVM_TIM_SHDW;
    volatile Uint32 PWR_MGMT_CTRL;
    volatile Uint32 PWR_MGMT_CTRL_SHDW;
    volatile Uint32 LPDDR2_MODE_REG_DATA;
    volatile Uint8 RSVD0[12];
    volatile Uint32 LPDDR2_MODE_REG_CFG;
    volatile Uint32 VBUSM_CONFIG;
    volatile Uint32 VBUSM_CFG_VAL_1;
    volatile Uint32 VBUSM_CFG_VAL_2;
    volatile Uint32 IODFT_TLGC;
    volatile Uint32 IODFT_CTRL_MISR_RSLT;
    volatile Uint32 IODFT_ADDR_MISR_RSLT;
    volatile Uint32 IODFT_DATA_MISR_RSLT_1;
    volatile Uint32 IODFT_DATA_MISR_RSLT_2;
    volatile Uint32 IODFT_DATA_MISR_RSLT_3;
    volatile Uint32 IODFT_DATA_MISR_RSLT_4;
    volatile Uint32 IODFT_DATA_MISR_RSLT_5;
    volatile Uint32 PERF_CNT_1;
    volatile Uint32 PERF_CNT_2;
    volatile Uint32 PERF_CNT_CFG;
    volatile Uint32 PERF_CNT_SEL;
    volatile Uint32 PERF_CNT_TIM;
    volatile Uint8 RSVD1[4];
    volatile Uint32 READ_IDLE_CTRL;
    volatile Uint32 READ_IDLE_CTRL_SHDW;
    volatile Uint8 RSVD2[4];
    volatile Uint32 IRQSTATUS_RAW_SYS;
    volatile Uint8 RSVD3[4];
    volatile Uint32 IRQSTATUS_SYS;
    volatile Uint8 RSVD4[4];
    volatile Uint32 IRQENABLE_SET_SYS;
    volatile Uint8 RSVD5[4];
    volatile Uint32 IRQENABLE_CLR_SYS;
    volatile Uint8 RSVD6[8];
    volatile Uint32 ZQ_CONFIG;
    volatile Uint32 TEMP_ALERT_CONFIG;
    volatile Uint32 VBUSM_ERR_LOG;
    volatile Uint32 RDWR_LVL_RMP_WIN;
    volatile Uint32 RDWR_LVL_RMP_CTRL;
    volatile Uint32 RDWR_LVL_CTRL;
    volatile Uint8 RSVD7[4];
    volatile Uint32 DDR_PHY_CTRL_1;
    volatile Uint32 DDR_PHY_CTRL_1_SHDW;
    volatile Uint32 DDR_PHY_CTRL_2;
    volatile Uint8 RSVD8[16];
    volatile Uint32 PRI_COS_MAP;
    volatile Uint32 MSTID_COS_1_MAP;
    volatile Uint32 MSTID_COS_2_MAP;
    volatile Uint8 RSVD9[4];
    volatile Uint32 ECC_CTRL;
    volatile Uint32 ECC_ADDR_RNG_1;
    volatile Uint32 ECC_ADDR_RNG_2;
    volatile Uint8 RSVD10[4];
    volatile Uint32 RD_WR_EXEC_THRSH;
} CSL_Emif4fRegs;

#endif
