/********************************************************************
* Copyright (C) 2003-2010 Texas Instruments Incorporated.
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
#ifndef _CSLR_SRIO_H_
#define _CSLR_SRIO_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a)  RIO_TLM_SP_BRR_0_CTL,RIO_TLM_SP_BRR_0_PATTERN_MATCH to
 *          RIO_TLM_SP_BRR_3_CTL,RIO_TLM_SP_BRR_3_PATTERN_MATCH 
 *          was made into an array of structures.
 *      b)  Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for block_enable_status
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_BLK_EN;
    volatile Uint32 RIO_BLK_EN_STAT;
} CSL_SrioBlock_enable_statusRegs;

/**************************************************************************\
* Register Overlay Structure for pf_cntl
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_PF_16B_CNTL;
    volatile Uint32 RIO_PF_8B_CNTL;
} CSL_SrioPf_cntlRegs;

/**************************************************************************\
* Register Overlay Structure for doorbell_icsr_iccr
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_DOORBELL_ICSR;
    volatile Uint8 RSVD0[4];
    volatile Uint32 RIO_DOORBELL_ICCR;
    volatile Uint8 RSVD3[4];
} CSL_SrioDoorbell_icsr_iccrRegs;

/**************************************************************************\
* Register Overlay Structure for lsu_icsr_iccr
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_LSU_ICSR;
    volatile Uint8 RSVD0[4];
    volatile Uint32 RIO_LSU_ICCR;
    volatile Uint8 RSVD5[4];
} CSL_SrioLsu_icsr_iccrRegs;

/**************************************************************************\
* Register Overlay Structure for doorbell_icrr
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_DOORBELL_ICRR1;
    volatile Uint32 RIO_DOORBELL_ICRR2;
    volatile Uint8 RSVD9[4];
} CSL_SrioDoorbell_icrrRegs;

/**************************************************************************\
* Register Overlay Structure for rxu_map
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_RXU_MAP_L;
    volatile Uint32 RIO_RXU_MAP_H;
    volatile Uint32 RIO_RXU_MAP_QID;
} CSL_SrioRxu_mapRegs;

/**************************************************************************\
* Register Overlay Structure for rxu_type9_map
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_RXU_TYPE9_MAP0;
    volatile Uint32 RIO_RXU_TYPE9_MAP1;
    volatile Uint32 RIO_RXU_TYPE9_MAP2;
} CSL_SrioRxu_type9_mapRegs;

/**************************************************************************\
* Register Overlay Structure for amu_window
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_AMU_WINDOW_REG0;
    volatile Uint32 RIO_AMU_WINDOW_REG1;
    volatile Uint32 RIO_AMU_WINDOW_REG2;
} CSL_SrioAmu_windowRegs;

/**************************************************************************\
* Register Overlay Structure for lsu_cmd
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_LSU_REG0;
    volatile Uint32 RIO_LSU_REG1;
    volatile Uint32 RIO_LSU_REG2;
    volatile Uint32 RIO_LSU_REG3;
    volatile Uint32 RIO_LSU_REG4;
    volatile Uint32 RIO_LSU_REG5;
    volatile Uint32 RIO_LSU_REG6;
} CSL_SrioLsu_cmdRegs;

/**************************************************************************\
* Register Overlay Structure for tx_channel_global_config
\**************************************************************************/
typedef struct  {
    volatile Uint32 TX_CHANNEL_GLOBAL_CONFIG_REG_A;
    volatile Uint32 TX_CHANNEL_GLOBAL_CONFIG_REG_B;
    volatile Uint8 RSVD19[24];
} CSL_SrioTx_channel_global_configRegs;

/**************************************************************************\
* Register Overlay Structure for rx_channel_global_config
\**************************************************************************/
typedef struct  {
    volatile Uint32 RX_CHANNEL_GLOBAL_CONFIG_REG;
    volatile Uint8 RSVD21[28];
} CSL_SrioRx_channel_global_configRegs;

/**************************************************************************\
* Register Overlay Structure for rx_flow_config
\**************************************************************************/
typedef struct  {
    volatile Uint32 RX_FLOW_CONFIG_REG_A;
    volatile Uint32 RX_FLOW_CONFIG_REG_B;
    volatile Uint32 RX_FLOW_CONFIG_REG_C;
    volatile Uint32 RX_FLOW_CONFIG_REG_D;
    volatile Uint32 RX_FLOW_CONFIG_REG_E;
    volatile Uint32 RX_FLOW_CONFIG_REG_F;
    volatile Uint32 RX_FLOW_CONFIG_REG_G;
    volatile Uint32 RX_FLOW_CONFIG_REG_H;
} CSL_SrioRx_flow_configRegs;

/**************************************************************************\
* Register Overlay Structure for rio_sp
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_SP_LM_REQ;
    volatile Uint32 RIO_SP_LM_RESP;
    volatile Uint32 RIO_SP_ACKID_STAT;
    volatile Uint8 RSVD0[8];
    volatile Uint32 RIO_SP_CTL2;
    volatile Uint32 RIO_SP_ERR_STAT;
    volatile Uint32 RIO_SP_CTL;
} CSL_SrioRio_spRegs;

/**************************************************************************\
* Register Overlay Structure for rio_sp_err
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_SP_ERR_DET;
    volatile Uint32 RIO_SP_RATE_EN;
    volatile Uint32 RIO_SP_ERR_ATTR_CAPT;
    volatile Uint32 RIO_SP_ERR_CAPT_0;
    volatile Uint32 RIO_SP_ERR_CAPT_1;
    volatile Uint32 RIO_SP_ERR_CAPT_2;
    volatile Uint32 RIO_SP_ERR_CAPT_3;
    volatile Uint8 RSVD0[12];
    volatile Uint32 RIO_SP_ERR_RATE;
    volatile Uint32 RIO_SP_ERR_THRESH;
    volatile Uint8 RSVD36[16];
} CSL_SrioRio_sp_errRegs;

/**************************************************************************\
* Register Overlay Structure for rio_lane
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_LANE_STAT0;
    volatile Uint32 RIO_LANE_STAT1;
    volatile Uint8 RSVD39[24];
} CSL_SrioRio_laneRegs;

/**************************************************************************\
* Register Overlay Structure for rio_plm
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_PLM_SP_IMP_SPEC_CTL;
    volatile Uint32 RIO_PLM_SP_PWDN_CTL;
    volatile Uint8 RSVD0[8];
    volatile Uint32 RIO_PLM_SP_STATUS;
    volatile Uint32 RIO_PLM_SP_INT_ENABLE;
    volatile Uint32 RIO_PLM_SP_PW_ENABLE;
    volatile Uint32 RIO_PLM_SP_EVENT_GEN;
    volatile Uint32 RIO_PLM_SP_ALL_INT_EN;
    volatile Uint32 RIO_PLM_SP_ALL_PW_EN;
    volatile Uint8 RSVD1[8];
    volatile Uint32 RIO_PLM_SP_PATH_CTL;
    volatile Uint32 RIO_PLM_SP_DISCOVERY_TIMER;
    volatile Uint32 RIO_PLM_SP_SILENCE_TIMER;
    volatile Uint32 RIO_PLM_SP_VMIN_EXP;
    volatile Uint32 RIO_PLM_SP_POL_CTL;
    volatile Uint8 RSVD2[4];
    volatile Uint32 RIO_PLM_SP_DENIAL_CTL;
    volatile Uint8 RSVD3[4];
    volatile Uint32 RIO_PLM_SP_RCVD_MECS;
    volatile Uint8 RSVD4[4];
    volatile Uint32 RIO_PLM_SP_MECS_FWD;
    volatile Uint8 RSVD5[4];
    volatile Uint32 RIO_PLM_SP_LONG_CS_TX1;
    volatile Uint32 RIO_PLM_SP_LONG_CS_TX2;
    volatile Uint8 RSVD42[24];
} CSL_SrioRio_plmRegs;

#ifndef CSL_MODIFICATION

typedef struct  {
    volatile Uint32 RIO_TLM_SP_BRR_CTL;
    volatile Uint32 RIO_TLM_SP_BRR_PATTERN_MATCH;
    volatile Uint8 RSVD1[8];
}CSL_SrioRio_BRRConfig;

#endif

/**************************************************************************\
* Register Overlay Structure for rio_tlm
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_TLM_SP_CONTROL;
    volatile Uint8 RSVD0[12];
    volatile Uint32 RIO_TLM_SP_STATUS;
    volatile Uint32 RIO_TLM_SP_INT_ENABLE;
    volatile Uint32 RIO_TLM_SP_PW_ENABLE;
    volatile Uint32 RIO_TLM_SP_EVENT_GEN;
#ifdef CSL_MODIFICATION    
    volatile Uint32 RIO_TLM_SP_BRR_0_CTL;
    volatile Uint32 RIO_TLM_SP_BRR_0_PATTERN_MATCH;
    volatile Uint8 RSVD1[8];
    volatile Uint32 RIO_TLM_SP_BRR_1_CTL;
    volatile Uint32 RIO_TLM_SP_BRR_1_PATTERN_MATCH;
    volatile Uint8 RSVD2[8];
    volatile Uint32 RIO_TLM_SP_BRR_2_CTL;
    volatile Uint32 RIO_TLM_SP_BRR_2_PATTERN_MATCH;
    volatile Uint8 RSVD3[8];
    volatile Uint32 RIO_TLM_SP_BRR_3_CTL;
    volatile Uint32 RIO_TLM_SP_BRR_3_PATTERN_MATCH;
    volatile Uint8 RSVD45[40];
#else
    CSL_SrioRio_BRRConfig   brr[4];
    volatile Uint8          RSVD45[32];
#endif
} CSL_SrioRio_tlmRegs;

/**************************************************************************\
* Register Overlay Structure for rio_pbm
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_PBM_SP_CONTROL;
    volatile Uint8 RSVD0[12];
    volatile Uint32 RIO_PBM_SP_STATUS;
    volatile Uint32 RIO_PBM_SP_INT_ENABLE;
    volatile Uint32 RIO_PBM_SP_PW_ENABLE;
    volatile Uint32 RIO_PBM_SP_EVENT_GEN;
    volatile Uint32 RIO_PBM_SP_IG_RESOURCES;
    volatile Uint32 RIO_PBM_SP_EG_RESOURCES;
    volatile Uint8 RSVD1[8];
    volatile Uint32 RIO_PBM_SP_IG_WATERMARK0;
    volatile Uint32 RIO_PBM_SP_IG_WATERMARK1;
    volatile Uint32 RIO_PBM_SP_IG_WATERMARK2;
    volatile Uint32 RIO_PBM_SP_IG_WATERMARK3;
    volatile Uint8 RSVD48[64];
} CSL_SrioRio_pbmRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 RIO_PID;
    volatile Uint32 RIO_PCR;
    volatile Uint8 RSVD0[12];
    volatile Uint32 RIO_PER_SET_CNTL;
    volatile Uint32 RIO_PER_SET_CNTL1;
    volatile Uint8 RSVD1[8];
    volatile Uint32 RIO_GBL_EN;
    volatile Uint32 RIO_GBL_EN_STAT;
    CSL_SrioBlock_enable_statusRegs BLOCK_ENABLE_STATUS[10];
    volatile Uint8 RSVD2[68];
    volatile Uint32 RIO_MULTIID_REG[8];
    CSL_SrioPf_cntlRegs PF_CNTL[8];
    volatile Uint8 RSVD4[96];
    CSL_SrioDoorbell_icsr_iccrRegs DOORBELL_ICSR_ICCR[4];
    CSL_SrioLsu_icsr_iccrRegs LSU_ICSR_ICCR[2];
    volatile Uint32 RIO_ERR_RST_EVNT_ICSR;
    volatile Uint8 RSVD6[4];
    volatile Uint32 RIO_ERR_RST_EVNT_ICCR;
    volatile Uint8 RSVD7[4];
    volatile Uint32 RIO_AMU_ICSR;
    volatile Uint8 RSVD8[4];
    volatile Uint32 RIO_AMU_ICCR;
    volatile Uint8 RSVD10[4];
    CSL_SrioDoorbell_icrrRegs DOORBELL_ICRR[4];
    volatile Uint32 RIO_LSU0_MODULE_ICRR[4];
    volatile Uint32 RIO_LSU1_MODULE_ICRR;
    volatile Uint8 RSVD11[12];
    volatile Uint32 RIO_ERR_RST_EVNT_ICRR;
    volatile Uint32 RIO_ERR_RST_EVNT_ICRR2;
    volatile Uint32 RIO_ERR_RST_EVNT_ICRR3;
    volatile Uint32 RIO_AMU_ICRR1;
    volatile Uint32 RIO_AMU_ICRR2;
    volatile Uint32 RIO_INTERRUPT_CTL;
    volatile Uint8 RSVD12[8];
    volatile Uint32 RIO_INTDST_DECODE[24];
    volatile Uint32 RIO_INTDST_RATE_CNT[16];
    volatile Uint32 RIO_INTDST_RATE_DIS;
    volatile Uint8 RSVD13[236];
    CSL_SrioRxu_mapRegs RXU_MAP[64];
    CSL_SrioRxu_type9_mapRegs RXU_TYPE9_MAP[64];
    volatile Uint32 RIO_AMU_SRCID_MAP[4];
    volatile Uint8 RSVD14[4];
    CSL_SrioAmu_windowRegs AMU_WINDOW[16];
    volatile Uint32 RIO_AMU_PRIORITY_MAP;
    volatile Uint32 RIO_AMU_CAPT0_MAP;
    volatile Uint32 RIO_AMU_CAPT1_MAP;
    volatile Uint32 RIO_AMU_WINDOW_PANE[128];
    volatile Uint32 RIO_AMU_FLOW_MASKS0;
    volatile Uint8 RSVD15[28];
    CSL_SrioLsu_cmdRegs LSU_CMD[8];
    volatile Uint32 RIO_LSU_SETUP_REG0;
    volatile Uint32 RIO_LSU_SETUP_REG1;
    volatile Uint32 LSU_STAT_REG[6];
    volatile Uint32 RIO_LSU_FLOW_MASKS[4];
    volatile Uint8 RSVD16[60];
    volatile Uint32 RIO_SUPERVISOR_ID;
    volatile Uint32 RIO_FLOW_CNTL[16];
    volatile Uint8 RSVD17[32];
    volatile Uint32 RIO_TX_CPPI_FLOW_MASKS[8];
    volatile Uint32 RIO_TX_QUEUE_SCH_INFO[4];
    volatile Uint32 RIO_GARBAGE_COLL_QID0;
    volatile Uint32 RIO_GARBAGE_COLL_QID1;
    volatile Uint32 RIO_GARBAGE_COLL_QID2;
    volatile Uint8 RSVD18[276];
    volatile Uint32 REVISION_REG;
    volatile Uint32 PERF_CONTROL_REG;
    volatile Uint32 EMULATION_CONTROL_REG;
    volatile Uint32 PRIORITY_CONTROL_REG;
    volatile Uint32 QM_BASE_ADDRESS_REG[4];
    volatile Uint8 RSVD20[992];
    CSL_SrioTx_channel_global_configRegs TX_CHANNEL_GLOBAL_CONFIG[16];
    volatile Uint8 RSVD22[512];
    CSL_SrioRx_channel_global_configRegs RX_CHANNEL_GLOBAL_CONFIG[16];
    volatile Uint8 RSVD23[512];
    volatile Uint32 TX_CHANNEL_SCHEDULER_CONFIG_REG[16];
    volatile Uint8 RSVD24[960];
    CSL_SrioRx_flow_configRegs RX_FLOW_CONFIG[20];
    volatile Uint8 RSVD25[36224];
    volatile Uint32 RIO_DEV_ID;
    volatile Uint32 RIO_DEV_INFO;
    volatile Uint32 RIO_ASBLY_ID;
    volatile Uint32 RIO_ASBLY_INFO;
    volatile Uint32 RIO_PE_FEAT;
    volatile Uint32 RIO_SW_PORT;
    volatile Uint32 RIO_SRC_OP;
    volatile Uint32 RIO_DEST_OP;
    volatile Uint8 RSVD26[28];
    volatile Uint32 RIO_DS_INFO;
    volatile Uint8 RSVD27[8];
    volatile Uint32 RIO_DS_LL_CTL;
    volatile Uint32 RIO_PE_LL_CTL;
    volatile Uint8 RSVD28[8];
    volatile Uint32 RIO_LCL_CFG_HBAR;
    volatile Uint32 RIO_LCL_CFG_BAR;
    volatile Uint32 RIO_BASE_ID;
    volatile Uint8 RSVD29[4];
    volatile Uint32 RIO_HOST_BASE_ID_LOCK;
    volatile Uint32 RIO_COMP_TAG;
    volatile Uint8 RSVD30[144];
    volatile Uint32 RIO_SP_MB_HEAD;
    volatile Uint8 RSVD31[28];
    volatile Uint32 RIO_SP_LT_CTL;
    volatile Uint32 RIO_SP_RT_CTL;
    volatile Uint8 RSVD32[20];
    volatile Uint32 RIO_SP_GEN_CTL;
    CSL_SrioRio_spRegs RIO_SP[4];
    volatile Uint8 RSVD33[3648];
    volatile Uint32 RIO_ERR_RPT_BH;
    volatile Uint8 RSVD34[4];
    volatile Uint32 RIO_ERR_DET;
    volatile Uint32 RIO_ERR_EN;
    volatile Uint32 RIO_H_ADDR_CAPT;
    volatile Uint32 RIO_ADDR_CAPT;
    volatile Uint32 RIO_ID_CAPT;
    volatile Uint32 RIO_CTRL_CAPT;
    volatile Uint8 RSVD35[8];
    volatile Uint32 RIO_PW_TGT_ID;
    volatile Uint8 RSVD37[20];
    CSL_SrioRio_sp_errRegs RIO_SP_ERR[4];
    volatile Uint8 RSVD38[7872];
    volatile Uint32 RIO_PER_LANE_BH;
    volatile Uint8 RSVD40[12];
    CSL_SrioRio_laneRegs RIO_LANE[4];
    volatile Uint8 RSVD41[53104];
    volatile Uint32 RIO_PLM_BH;
    volatile Uint8 RSVD43[124];
    CSL_SrioRio_plmRegs RIO_PLM[4];
    volatile Uint8 RSVD44[128];
    volatile Uint32 RIO_TLM_BH;
    volatile Uint8 RSVD46[124];
    CSL_SrioRio_tlmRegs RIO_TLM[4];
    volatile Uint8 RSVD47[128];
    volatile Uint32 RIO_PBM_BH;
    volatile Uint8 RSVD49[124];
    CSL_SrioRio_pbmRegs RIO_PBM[4];
    volatile Uint8 RSVD50[128];
    volatile Uint32 RIO_EM_BH;
    volatile Uint8 RSVD51[12];
    volatile Uint32 RIO_EM_INT_STAT;
    volatile Uint32 RIO_EM_INT_ENABLE;
    volatile Uint32 RIO_EM_INT_PORT_STAT;
    volatile Uint8 RSVD52[4];
    volatile Uint32 RIO_EM_PW_STAT;
    volatile Uint32 RIO_EM_PW_EN;
    volatile Uint32 RIO_EM_PW_PORT_STAT;
    volatile Uint8 RSVD53[4];
    volatile Uint32 RIO_EM_DEV_INT_EN;
    volatile Uint32 RIO_EM_DEV_PW_EN;
    volatile Uint8 RSVD54[4];
    volatile Uint32 RIO_EM_MECS_STAT;
    volatile Uint32 RIO_EM_MECS_INT_EN;
    volatile Uint32 RIO_EM_MECS_CAP_EN;
    volatile Uint32 RIO_EM_MECS_TRIG_EN;
    volatile Uint32 RIO_EM_MECS_REQ;
    volatile Uint32 RIO_EM_MECS_PORT_STAT;
    volatile Uint8 RSVD55[8];
    volatile Uint32 RIO_EM_MECS_EVENT_GEN;
    volatile Uint32 RIO_EM_RST_PORT_STAT;
    volatile Uint8 RSVD56[4];
    volatile Uint32 RIO_EM_RST_INT_EN;
    volatile Uint8 RSVD57[4];
    volatile Uint32 RIO_EM_RST_PW_EN;
    volatile Uint8 RSVD58[140];
    volatile Uint32 RIO_PW_BH;
    volatile Uint32 RIO_PW_CTL;
    volatile Uint32 RIO_PW_ROUTE;
    volatile Uint8 RSVD59[4];
    volatile Uint32 RIO_PW_RX_STAT;
    volatile Uint32 RIO_PW_RX_EVENT_GEN;
    volatile Uint8 RSVD60[8];
    volatile Uint32 RIO_PW_RX_CAPT[4];
    volatile Uint8 RSVD61[720];
    volatile Uint32 RIO_LLM_BH;
    volatile Uint8 RSVD62[32];
    volatile Uint32 RIO_WHITEBOARD;
    volatile Uint32 RIO_PORT_NUMBER;
    volatile Uint8 RSVD63[4];
    volatile Uint32 RIO_PRESCALAR_SRV_CLK;
    volatile Uint32 RIO_REG_RST_CTL;
    volatile Uint8 RSVD64[16];
    volatile Uint32 RIO_LOCAL_ERR_DET;
    volatile Uint32 RIO_LOCAL_ERR_EN;
    volatile Uint32 RIO_LOCAL_H_ADDR_CAPT;
    volatile Uint32 RIO_LOCAL_ADDR_CAPT;
    volatile Uint32 RIO_LOCAL_ID_CAPT;
    volatile Uint32 RIO_LOCAL_CTRL_CAPT;
    volatile Uint8 RSVD65[160];
    volatile Uint32 RIO_FABRIC_BH;
    volatile Uint8 RSVD66[12];
    volatile Uint32 RIO_FABRIC_CSR;
    volatile Uint8 RSVD67[44];
    volatile Uint32 RIO_SP_FABRIC_STATUS[4];
} CSL_SrioRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* rio_blk_en */

#define CSL_SRIO_RIO_BLK_EN_EN_MASK      (0x00000001u)
#define CSL_SRIO_RIO_BLK_EN_EN_SHIFT     (0x00000000u)
#define CSL_SRIO_RIO_BLK_EN_EN_RESETVAL  (0x00000001u)

#define CSL_SRIO_RIO_BLK_EN_RESETVAL     (0x00000001u)

/* rio_blk_en_stat */

#define CSL_SRIO_RIO_BLK_EN_STAT_EN_STATUS_MASK (0x00000001u)
#define CSL_SRIO_RIO_BLK_EN_STAT_EN_STATUS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_BLK_EN_STAT_EN_STATUS_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_BLK_EN_STAT_RESETVAL (0x00000001u)

/* rio_pf_16b_cntl */

#define CSL_SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PF_16B_CNTL_DEVID_16B_LO_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PF_16B_CNTL_DEVID_16B_UP_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_PF_16B_CNTL_RESETVAL (0xFFFFFFFFu)

/* rio_pf_8b_cntl */

#define CSL_SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO_MASK (0x000000FFu)
#define CSL_SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PF_8B_CNTL_DEVID_8B_LO_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PF_8B_CNTL_DEVID_8B_UP_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_PF_8B_CNTL_OUT_PORT_MASK (0x00030000u)
#define CSL_SRIO_RIO_PF_8B_CNTL_OUT_PORT_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PF_8B_CNTL_OUT_PORT_RESETVAL (0x00000003u)

#define CSL_SRIO_RIO_PF_8B_CNTL_RESETVAL (0x0003FFFFu)

/* rio_doorbell_icsr */

#define CSL_SRIO_RIO_DOORBELL_ICSR_RIO_DOORBELL_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_DOORBELL_ICSR_RIO_DOORBELL_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DOORBELL_ICSR_RIO_DOORBELL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICSR_RESETVAL (0x00000000u)

/* rio_doorbell_iccr */

#define CSL_SRIO_RIO_DOORBELL_ICCR_RIO_DOORBELL_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_DOORBELL_ICCR_RIO_DOORBELL_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DOORBELL_ICCR_RIO_DOORBELL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICCR_RESETVAL (0x00000000u)

/* rio_lsu_icsr */

#define CSL_SRIO_RIO_LSU_ICSR_RIO_LSU_ICSR_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LSU_ICSR_RIO_LSU_ICSR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_ICSR_RIO_LSU_ICSR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_ICSR_RESETVAL   (0x00000000u)

/* rio_lsu_iccr */

#define CSL_SRIO_RIO_LSU_ICCR_RIO_LSU_ICCR_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LSU_ICCR_RIO_LSU_ICCR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_ICCR_RIO_LSU_ICCR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_ICCR_RESETVAL   (0x00000000u)

/* rio_doorbell_icrr1 */

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR0_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR1_MASK (0x000000F0u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR2_MASK (0x00000F00u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR2_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR3_MASK (0x0000F000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR3_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR3_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR4_MASK (0x000F0000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR4_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR4_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR5_MASK (0x00F00000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR5_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR5_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR6_MASK (0x0F000000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR6_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR6_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR7_MASK (0xF0000000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR7_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_DOORBELL_ICRR1_ICR7_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR1_RESETVAL (0x00000000u)

/* rio_doorbell_icrr2 */

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR8_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR8_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR8_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR9_MASK (0x000000F0u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR9_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR9_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR10_MASK (0x00000F00u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR10_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR10_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR11_MASK (0x0000F000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR11_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR11_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR12_MASK (0x000F0000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR12_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR12_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR13_MASK (0x00F00000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR13_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR13_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR14_MASK (0x0F000000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR14_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR14_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR15_MASK (0xF0000000u)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR15_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_DOORBELL_ICRR2_ICR15_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DOORBELL_ICRR2_RESETVAL (0x00000000u)

/* rio_rxu_map_l */

#define CSL_SRIO_RIO_RXU_MAP_L_SRCID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_RXU_MAP_L_SRCID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_RXU_MAP_L_SRCID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_L_MBX_MASK  (0x003F0000u)
#define CSL_SRIO_RIO_RXU_MAP_L_MBX_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_RXU_MAP_L_MBX_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_L_LTR_MASK  (0x00C00000u)
#define CSL_SRIO_RIO_RXU_MAP_L_LTR_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_RXU_MAP_L_LTR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_L_MBX_MASK_MASK (0x3F000000u)
#define CSL_SRIO_RIO_RXU_MAP_L_MBX_MASK_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_RXU_MAP_L_MBX_MASK_RESETVAL (0x0000003Fu)

#define CSL_SRIO_RIO_RXU_MAP_L_LTR_MASK_MASK (0xC0000000u)
#define CSL_SRIO_RIO_RXU_MAP_L_LTR_MASK_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_RXU_MAP_L_LTR_MASK_RESETVAL (0x00000003u)

#define CSL_SRIO_RIO_RXU_MAP_L_RESETVAL  (0xFF000000u)

/* rio_rxu_map_h */

#define CSL_SRIO_RIO_RXU_MAP_H_SEG_MAP_MASK (0x00000001u)
#define CSL_SRIO_RIO_RXU_MAP_H_SEG_MAP_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_RXU_MAP_H_SEG_MAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_H_SRC_PROM_MASK (0x00000002u)
#define CSL_SRIO_RIO_RXU_MAP_H_SRC_PROM_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_RXU_MAP_H_SRC_PROM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_H_TT_MASK   (0x00006000u)
#define CSL_SRIO_RIO_RXU_MAP_H_TT_SHIFT  (0x0000000Du)
#define CSL_SRIO_RIO_RXU_MAP_H_TT_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_RXU_MAP_H_DEST_PROM_MASK (0x00008000u)
#define CSL_SRIO_RIO_RXU_MAP_H_DEST_PROM_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_RXU_MAP_H_DEST_PROM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_H_DEST_ID_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_RXU_MAP_H_DEST_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_RXU_MAP_H_DEST_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_H_RESETVAL  (0x00002000u)

/* rio_rxu_map_qid */

#define CSL_SRIO_RIO_RXU_MAP_QID_DEST_QID_MASK (0x00003FFFu)
#define CSL_SRIO_RIO_RXU_MAP_QID_DEST_QID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_RXU_MAP_QID_DEST_QID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_QID_FLOWID_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_RXU_MAP_QID_FLOWID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_RXU_MAP_QID_FLOWID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_MAP_QID_RESETVAL (0x00000000u)

/* rio_rxu_type9_map0 */

#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_SRCID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_SRCID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_SRCID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_COS_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_COS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK_MASK (0xFF000000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_COS_MASK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP0_RESETVAL (0x00000000u)

/* rio_rxu_type9_map1 */

#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM_MASK (0x00000002u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_SRC_PROM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_TT_MASK (0x00006000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_TT_SHIFT (0x0000000Du)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_TT_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM_MASK (0x00008000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_DEST_PROM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_DEST_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP1_RESETVAL (0x00002000u)

/* rio_rxu_type9_map2 */

#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_STRM_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_STRM_MASK_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_RXU_TYPE9_MAP2_RESETVAL (0xFFFF0000u)

/* rio_amu_window_reg0 */

#define CSL_SRIO_RIO_AMU_WINDOW_REG0_XAMBS_MASK (0x00000003u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_XAMBS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_XAMBS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT_MASK (0x00000C00u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_PANE_COUNT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE_MASK (0x00FFF000u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_PANE_SIZE_RESETVAL (0x00000800u)

#define CSL_SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE_MASK (0xFF000000u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG0_WINDOW_SIZE_RESETVAL (0x00000010u)

#define CSL_SRIO_RIO_AMU_WINDOW_REG0_RESETVAL (0x10800000u)

/* rio_amu_window_reg1 */

#define CSL_SRIO_RIO_AMU_WINDOW_REG1_RIO_ADDRESS_MSB_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_AMU_WINDOW_REG1_RIO_ADDRESS_MSB_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG1_RIO_ADDRESS_MSB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_REG1_RESETVAL (0x00000000u)

/* rio_amu_window_reg2 */

#define CSL_SRIO_RIO_AMU_WINDOW_REG2_RIO_ADDRESS_LSB_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_AMU_WINDOW_REG2_RIO_ADDRESS_LSB_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_WINDOW_REG2_RIO_ADDRESS_LSB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_REG2_RESETVAL (0x00000000u)

/* rio_lsu_reg0 */

#define CSL_SRIO_RIO_LSU_REG0_RIO_ADDRESS_MSB_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LSU_REG0_RIO_ADDRESS_MSB_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG0_RIO_ADDRESS_MSB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG0_RESETVAL   (0x00000000u)

/* rio_lsu_reg1 */

#define CSL_SRIO_RIO_LSU_REG1_RIO_ADDRESS_LSB_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LSU_REG1_RIO_ADDRESS_LSB_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG1_RIO_ADDRESS_LSB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG1_RESETVAL   (0x00000000u)

/* rio_lsu_reg2 */

#define CSL_SRIO_RIO_LSU_REG2_DSP_ADDRESS_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LSU_REG2_DSP_ADDRESS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG2_DSP_ADDRESS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG2_RESETVAL   (0x00000000u)

/* rio_lsu_reg3 */

#define CSL_SRIO_RIO_LSU_REG3_BYTE_COUNT_MASK (0x000FFFFFu)
#define CSL_SRIO_RIO_LSU_REG3_BYTE_COUNT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG3_BYTE_COUNT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG3_DRBLL_VALUE_MASK (0x80000000u)
#define CSL_SRIO_RIO_LSU_REG3_DRBLL_VALUE_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_LSU_REG3_DRBLL_VALUE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG3_RESETVAL   (0x00000000u)

/* rio_lsu_reg4 */

#define CSL_SRIO_RIO_LSU_REG4_INT_REQ_MASK (0x00000001u)
#define CSL_SRIO_RIO_LSU_REG4_INT_REQ_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG4_INT_REQ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_SUP_GINT_MASK (0x00000002u)
#define CSL_SRIO_RIO_LSU_REG4_SUP_GINT_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_LSU_REG4_SUP_GINT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_XAMBS_MASK (0x0000000Cu)
#define CSL_SRIO_RIO_LSU_REG4_XAMBS_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_LSU_REG4_XAMBS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_PRIORITY_MASK (0x000000F0u)
#define CSL_SRIO_RIO_LSU_REG4_PRIORITY_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_LSU_REG4_PRIORITY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_OUTPORTID_MASK (0x00000300u)
#define CSL_SRIO_RIO_LSU_REG4_OUTPORTID_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LSU_REG4_OUTPORTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_ID_SIZE_MASK (0x00000C00u)
#define CSL_SRIO_RIO_LSU_REG4_ID_SIZE_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_LSU_REG4_ID_SIZE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_SRCID_MAP_MASK (0x0000F000u)
#define CSL_SRIO_RIO_LSU_REG4_SRCID_MAP_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_LSU_REG4_SRCID_MAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_DESTID_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_LSU_REG4_DESTID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LSU_REG4_DESTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG4_RESETVAL   (0x00000000u)

/* rio_lsu_reg5 */

#define CSL_SRIO_RIO_LSU_REG5_TTYPE_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_LSU_REG5_TTYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG5_TTYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG5_FTYPE_MASK (0x000000F0u)
#define CSL_SRIO_RIO_LSU_REG5_FTYPE_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_LSU_REG5_FTYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG5_HOP_COUNT_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_LSU_REG5_HOP_COUNT_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LSU_REG5_HOP_COUNT_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_LSU_REG5_DRBLL_INFO_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_LSU_REG5_DRBLL_INFO_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LSU_REG5_DRBLL_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG5_RESETVAL   (0x0000FF00u)

/* rio_lsu_reg6 */

#define CSL_SRIO_RIO_LSU_REG6_LTID_MASK  (0x0000000Fu)
#define CSL_SRIO_RIO_LSU_REG6_LTID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG6_LTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_LCB_MASK   (0x00000010u)
#define CSL_SRIO_RIO_LSU_REG6_LCB_SHIFT  (0x00000004u)
#define CSL_SRIO_RIO_LSU_REG6_LCB_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_LSU_REG6_FULL_MASK  (0x40000000u)
#define CSL_SRIO_RIO_LSU_REG6_FULL_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_LSU_REG6_FULL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_BUSY_MASK  (0x80000000u)
#define CSL_SRIO_RIO_LSU_REG6_BUSY_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_LSU_REG6_BUSY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_FLUSH_MASK (0x00000001u)
#define CSL_SRIO_RIO_LSU_REG6_FLUSH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_REG6_FLUSH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_RESTART_MASK (0x00000002u)
#define CSL_SRIO_RIO_LSU_REG6_RESTART_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_LSU_REG6_RESTART_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_SCRID_MAP_MASK (0x0000003Cu)
#define CSL_SRIO_RIO_LSU_REG6_SCRID_MAP_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_LSU_REG6_SCRID_MAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_CBUSY_MASK (0x08000000u)
#define CSL_SRIO_RIO_LSU_REG6_CBUSY_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_LSU_REG6_CBUSY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_PRIVID_MASK (0xF0000000u)
#define CSL_SRIO_RIO_LSU_REG6_PRIVID_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_LSU_REG6_PRIVID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_REG6_RESETVAL   (0x00000010u)

/* tx_channel_global_config_reg_a */

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_PAUSE_MASK (0x20000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_PAUSE_SHIFT (0x0000001Du)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_PAUSE_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_TEARDOWN_MASK (0x40000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_TEARDOWN_SHIFT (0x0000001Eu)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_TEARDOWN_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE_MASK (0x80000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE_SHIFT (0x0000001Fu)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_A_RESETVAL (0x00000000u)

/* tx_channel_global_config_reg_b */

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_DEFAULT_QNUM_MASK (0x00000FFFu)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_DEFAULT_QNUM_SHIFT (0x00000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_DEFAULT_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_DEFAULT_QMGR_MASK (0x00003000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_DEFAULT_QMGR_SHIFT (0x0000000Cu)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_DEFAULT_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE_MASK (0x01000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE_SHIFT (0x00000018u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_PSWORDS_MASK (0x20000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_PSWORDS_SHIFT (0x0000001Du)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_PSWORDS_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_EINFO_MASK (0x40000000u)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_EINFO_SHIFT (0x0000001Eu)
#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_EINFO_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_GLOBAL_CONFIG_REG_B_RESETVAL (0x00000000u)

/* rx_channel_global_config_reg */

#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_PAUSE_MASK (0x20000000u)
#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_PAUSE_SHIFT (0x0000001Du)
#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_PAUSE_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_TEARDOWN_MASK (0x40000000u)
#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_TEARDOWN_SHIFT (0x0000001Eu)
#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_TEARDOWN_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE_MASK (0x80000000u)
#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE_SHIFT (0x0000001Fu)
#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_CHANNEL_GLOBAL_CONFIG_REG_RESETVAL (0x00000000u)

/* rx_flow_config_reg_a */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DEST_QNUM_MASK (0x00000FFFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DEST_QNUM_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DEST_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DEST_QMGR_MASK (0x00003000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DEST_QMGR_SHIFT (0x0000000Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DEST_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_SOP_OFFSET_MASK (0x01FF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_SOP_OFFSET_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_SOP_OFFSET_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_PS_LOCATION_MASK (0x02000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_PS_LOCATION_SHIFT (0x00000019u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_PS_LOCATION_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DESC_TYPE_MASK (0x0C000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DESC_TYPE_SHIFT (0x0000001Au)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_DESC_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_ERROR_HANDLING_MASK (0x10000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_ERROR_HANDLING_SHIFT (0x0000001Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_ERROR_HANDLING_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_PSINFO_PRESENT_MASK (0x20000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_PSINFO_PRESENT_SHIFT (0x0000001Du)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_PSINFO_PRESENT_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_EINFO_PRESENT_MASK (0x40000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_EINFO_PRESENT_SHIFT (0x0000001Eu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RX_EINFO_PRESENT_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_A_RESETVAL (0x00000000u)

/* rx_flow_config_reg_b */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_LO_MASK (0x000000FFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_LO_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_LO_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_HI_MASK (0x0000FF00u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_HI_SHIFT (0x00000008u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_HI_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_LO_MASK (0x00FF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_LO_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_LO_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_HI_MASK (0xFF000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_HI_SHIFT (0x00000018u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_HI_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_B_RESETVAL (0x00000000u)

/* rx_flow_config_reg_c */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SIZE_THRESH_EN_MASK (0x0000000Fu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SIZE_THRESH_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SIZE_THRESH_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_LO_SEL_MASK (0x00070000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_LO_SEL_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_LO_SEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_HI_SEL_MASK (0x00700000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_HI_SEL_SHIFT (0x00000014u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_HI_SEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_LO_SEL_MASK (0x07000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_LO_SEL_SHIFT (0x00000018u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_LO_SEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_HI_SEL_MASK (0x70000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_HI_SEL_SHIFT (0x0000001Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_HI_SEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_C_RESETVAL (0x00000000u)

/* rx_flow_config_reg_d */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QNUM_MASK (0x00000FFFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QNUM_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QMGR_MASK (0x00003000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QMGR_SHIFT (0x0000000Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QNUM_MASK (0x0FFF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QNUM_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QMGR_MASK (0x30000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QMGR_SHIFT (0x0000001Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_D_RESETVAL (0x00000000u)

/* rx_flow_config_reg_e */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QNUM_MASK (0x00000FFFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QNUM_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QMGR_MASK (0x00003000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QMGR_SHIFT (0x0000000Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QNUM_MASK (0x0FFF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QNUM_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QMGR_MASK (0x30000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QMGR_SHIFT (0x0000001Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_E_RESETVAL (0x00000000u)

/* rx_flow_config_reg_f */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH1_MASK (0x0000FFFFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH1_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH1_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH0_MASK (0xFFFF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH0_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH0_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_F_RESETVAL (0x00000000u)

/* rx_flow_config_reg_g */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QNUM_MASK (0x00000FFFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QNUM_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QMGR_MASK (0x00003000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QMGR_SHIFT (0x0000000Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_SIZE_THRESH2_MASK (0xFFFF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_SIZE_THRESH2_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RX_SIZE_THRESH2_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_G_RESETVAL (0x00000000u)

/* rx_flow_config_reg_h */

#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QNUM_MASK (0x00000FFFu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QNUM_SHIFT (0x00000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QMGR_MASK (0x00003000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QMGR_SHIFT (0x0000000Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QNUM_MASK (0x0FFF0000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QNUM_SHIFT (0x00000010u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QNUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QMGR_MASK (0x30000000u)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QMGR_SHIFT (0x0000001Cu)
#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QMGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RX_FLOW_CONFIG_REG_H_RESETVAL (0x00000000u)

/* rio_sp_lm_req */

#define CSL_SRIO_RIO_SP_LM_REQ_CMD_MASK  (0x00000007u)
#define CSL_SRIO_RIO_SP_LM_REQ_CMD_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_LM_REQ_CMD_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_LM_REQ_RESETVAL  (0x00000000u)

/* rio_sp_lm_resp */

#define CSL_SRIO_RIO_SP_LM_RESP_RESP_VALID_MASK (0x80000000u)
#define CSL_SRIO_RIO_SP_LM_RESP_RESP_VALID_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SP_LM_RESP_RESP_VALID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_LM_RESP_ACK_ID_STAT_MASK (0x000007E0u)
#define CSL_SRIO_RIO_SP_LM_RESP_ACK_ID_STAT_SHIFT (0x00000005u)
#define CSL_SRIO_RIO_SP_LM_RESP_ACK_ID_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_LM_RESP_LINK_STAT_MASK (0x0000001Fu)
#define CSL_SRIO_RIO_SP_LM_RESP_LINK_STAT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_LM_RESP_LINK_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_LM_RESP_RESETVAL (0x00000000u)

/* rio_sp_ackid_stat */

#define CSL_SRIO_RIO_SP_ACKID_STAT_CLR_OUTSTD_ACKID_MASK (0x80000000u)
#define CSL_SRIO_RIO_SP_ACKID_STAT_CLR_OUTSTD_ACKID_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SP_ACKID_STAT_CLR_OUTSTD_ACKID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ACKID_STAT_INB_ACKID_MASK (0x3F000000u)
#define CSL_SRIO_RIO_SP_ACKID_STAT_INB_ACKID_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_ACKID_STAT_INB_ACKID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ACKID_STAT_OUTSTD_ACKID_MASK (0x00001F00u)
#define CSL_SRIO_RIO_SP_ACKID_STAT_OUTSTD_ACKID_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SP_ACKID_STAT_OUTSTD_ACKID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID_MASK (0x0000003Fu)
#define CSL_SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ACKID_STAT_OUTB_ACKID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ACKID_STAT_RESETVAL (0x00000000u)

/* rio_sp_ctl2 */

#define CSL_SRIO_RIO_SP_CTL2_BAUD_SEL_MASK (0xF0000000u)
#define CSL_SRIO_RIO_SP_CTL2_BAUD_SEL_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_SP_CTL2_BAUD_SEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_BAUD_DISC_MASK (0x08000000u)
#define CSL_SRIO_RIO_SP_CTL2_BAUD_DISC_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_SP_CTL2_BAUD_DISC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_GB_1P25_MASK (0x02000000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_1P25_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_SP_CTL2_GB_1P25_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_CTL2_GB_1P25_EN_MASK (0x01000000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_1P25_EN_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_CTL2_GB_1P25_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_GB_2P5_MASK (0x00800000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_2P5_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_SP_CTL2_GB_2P5_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_CTL2_GB_2P5_EN_MASK (0x00400000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_2P5_EN_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_SP_CTL2_GB_2P5_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_GB_3P125_MASK (0x00200000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_3P125_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_SP_CTL2_GB_3P125_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_CTL2_GB_3P125_EN_MASK (0x00100000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_3P125_EN_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SP_CTL2_GB_3P125_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_GB_5P0_MASK (0x00080000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_5P0_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_SP_CTL2_GB_5P0_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_CTL2_GB_5P0_EN_MASK (0x00040000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_5P0_EN_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_SP_CTL2_GB_5P0_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_GB_6P25_MASK (0x00020000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_6P25_SHIFT (0x00000011u)
#define CSL_SRIO_RIO_SP_CTL2_GB_6P25_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_CTL2_GB_6P25_EN_MASK (0x00010000u)
#define CSL_SRIO_RIO_SP_CTL2_GB_6P25_EN_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_CTL2_GB_6P25_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_INACT_EN_MASK (0x00000008u)
#define CSL_SRIO_RIO_SP_CTL2_INACT_EN_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_SP_CTL2_INACT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_D_SCRM_DIS_MASK (0x00000004u)
#define CSL_SRIO_RIO_SP_CTL2_D_SCRM_DIS_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_SP_CTL2_D_SCRM_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_RTEC_MASK   (0x00000002u)
#define CSL_SRIO_RIO_SP_CTL2_RTEC_SHIFT  (0x00000001u)
#define CSL_SRIO_RIO_SP_CTL2_RTEC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_RTEC_EN_MASK (0x00000001u)
#define CSL_SRIO_RIO_SP_CTL2_RTEC_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_CTL2_RTEC_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL2_RESETVAL    (0x02AA0000u)

/* rio_sp_err_stat */

#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_MASK (0x80000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_EN_MASK (0x40000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_EN_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_SEQ_MASK (0x20000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_SEQ_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_SP_ERR_STAT_IDLE2_SEQ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_TXFC_MASK (0x08000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_TXFC_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_SP_ERR_STAT_TXFC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP_MASK (0x04000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_DROP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL_MASK (0x02000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_FAIL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR_MASK (0x01000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_DEGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_MASK (0x00100000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRIED_MASK (0x00080000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRIED_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRIED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_STOPPED_MASK (0x00040000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_STOPPED_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_RETRY_STOPPED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_MASK (0x00020000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_SHIFT (0x00000011u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_STOPPED_MASK (0x00010000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_STOPPED_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_ERR_STAT_OUTPUT_ERR_STOPPED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_RETRY_STOPPED_MASK (0x00000400u)
#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_RETRY_STOPPED_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_RETRY_STOPPED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_ERR_ENCTR_MASK (0x00000200u)
#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_ERR_ENCTR_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_ERR_ENCTR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_ERR_STOPPED_MASK (0x00000100u)
#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_ERR_STOPPED_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SP_ERR_STAT_INPUT_ERR_STOPPED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND_MASK (0x00000010u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_WRITE_PEND_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_UNAVL_MASK (0x00000008u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_UNAVL_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_UNAVL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_ERR_MASK (0x00000004u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_ERR_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_OK_MASK (0x00000002u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_OK_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_OK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_UNINIT_MASK (0x00000001u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_UNINIT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_STAT_PORT_UNINIT_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_ERR_STAT_RESETVAL (0x00000001u)

/* rio_sp_ctl */

#define CSL_SRIO_RIO_SP_CTL_PORT_WIDTH_MASK (0xC0000000u)
#define CSL_SRIO_RIO_SP_CTL_PORT_WIDTH_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_SP_CTL_PORT_WIDTH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_INIT_PWIDTH_MASK (0x38000000u)
#define CSL_SRIO_RIO_SP_CTL_INIT_PWIDTH_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_SP_CTL_INIT_PWIDTH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_OVER_PWIDTH_MASK (0x07000000u)
#define CSL_SRIO_RIO_SP_CTL_OVER_PWIDTH_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_CTL_OVER_PWIDTH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_PORT_DIS_MASK (0x00800000u)
#define CSL_SRIO_RIO_SP_CTL_PORT_DIS_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_SP_CTL_PORT_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_OTP_EN_MASK  (0x00400000u)
#define CSL_SRIO_RIO_SP_CTL_OTP_EN_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_SP_CTL_OTP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_INP_EN_MASK  (0x00200000u)
#define CSL_SRIO_RIO_SP_CTL_INP_EN_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_SP_CTL_INP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_ERR_DIS_MASK (0x00100000u)
#define CSL_SRIO_RIO_SP_CTL_ERR_DIS_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SP_CTL_ERR_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_MULT_CS_MASK (0x00080000u)
#define CSL_SRIO_RIO_SP_CTL_MULT_CS_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_SP_CTL_MULT_CS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_FLOW_CTRL_MASK (0x00040000u)
#define CSL_SRIO_RIO_SP_CTL_FLOW_CTRL_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_SP_CTL_FLOW_CTRL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_ENUM_B_MASK  (0x00020000u)
#define CSL_SRIO_RIO_SP_CTL_ENUM_B_SHIFT (0x00000011u)
#define CSL_SRIO_RIO_SP_CTL_ENUM_B_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_FLOW_ARB_MASK (0x00010000u)
#define CSL_SRIO_RIO_SP_CTL_FLOW_ARB_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_CTL_FLOW_ARB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_OVER_PWIDTH2_MASK (0x0000C000u)
#define CSL_SRIO_RIO_SP_CTL_OVER_PWIDTH2_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_SP_CTL_OVER_PWIDTH2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_PORT_WIDTH2_MASK (0x00003000u)
#define CSL_SRIO_RIO_SP_CTL_PORT_WIDTH2_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_SP_CTL_PORT_WIDTH2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_STOP_FAIL_EN_MASK (0x00000008u)
#define CSL_SRIO_RIO_SP_CTL_STOP_FAIL_EN_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_SP_CTL_STOP_FAIL_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_DROP_EN_MASK (0x00000004u)
#define CSL_SRIO_RIO_SP_CTL_DROP_EN_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_SP_CTL_DROP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_PORT_LOCKOUT_MASK (0x00000002u)
#define CSL_SRIO_RIO_SP_CTL_PORT_LOCKOUT_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_SP_CTL_PORT_LOCKOUT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_CTL_PTYP_MASK    (0x00000001u)
#define CSL_SRIO_RIO_SP_CTL_PTYP_SHIFT   (0x00000000u)
#define CSL_SRIO_RIO_SP_CTL_PTYP_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_SP_CTL_RESETVAL     (0x00000001u)

/* rio_sp_err_det */

#define CSL_SRIO_RIO_SP_ERR_DET_IMP_SPEC_MASK (0x80000000u)
#define CSL_SRIO_RIO_SP_ERR_DET_IMP_SPEC_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SP_ERR_DET_IMP_SPEC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_CS_CRC_ERR_MASK (0x00400000u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_CRC_ERR_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_CRC_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_CS_ILL_ID_MASK (0x00200000u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_ILL_ID_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_ILL_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_CS_NOT_ACC_MASK (0x00100000u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_NOT_ACC_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_NOT_ACC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_PKT_ILL_ACKID_MASK (0x00080000u)
#define CSL_SRIO_RIO_SP_ERR_DET_PKT_ILL_ACKID_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_SP_ERR_DET_PKT_ILL_ACKID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_PKT_CRC_ERR_MASK (0x00040000u)
#define CSL_SRIO_RIO_SP_ERR_DET_PKT_CRC_ERR_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_SP_ERR_DET_PKT_CRC_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_PKT_ILL_SIZE_MASK (0x00020000u)
#define CSL_SRIO_RIO_SP_ERR_DET_PKT_ILL_SIZE_SHIFT (0x00000011u)
#define CSL_SRIO_RIO_SP_ERR_DET_PKT_ILL_SIZE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_DSCRAM_LOS_MASK (0x00004000u)
#define CSL_SRIO_RIO_SP_ERR_DET_DSCRAM_LOS_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_SP_ERR_DET_DSCRAM_LOS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_LR_ACKID_ILL_MASK (0x00000020u)
#define CSL_SRIO_RIO_SP_ERR_DET_LR_ACKID_ILL_SHIFT (0x00000005u)
#define CSL_SRIO_RIO_SP_ERR_DET_LR_ACKID_ILL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_PROT_ERR_MASK (0x00000010u)
#define CSL_SRIO_RIO_SP_ERR_DET_PROT_ERR_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_SP_ERR_DET_PROT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_DELIN_ERR_MASK (0x00000004u)
#define CSL_SRIO_RIO_SP_ERR_DET_DELIN_ERR_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_SP_ERR_DET_DELIN_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_CS_ACK_ILL_MASK (0x00000002u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_ACK_ILL_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_SP_ERR_DET_CS_ACK_ILL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_LINK_TO_MASK (0x00000001u)
#define CSL_SRIO_RIO_SP_ERR_DET_LINK_TO_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_DET_LINK_TO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_DET_RESETVAL (0x00000000u)

/* rio_sp_rate_en */

#define CSL_SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN_MASK (0x80000000u)
#define CSL_SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SP_RATE_EN_IMP_SPEC_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_CS_CRC_EN_MASK (0x00400000u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_CRC_EN_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_CRC_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN_MASK (0x00200000u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_ILL_ID_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN_MASK (0x00100000u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_NOT_ACC_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN_MASK (0x00080000u)
#define CSL_SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_SP_RATE_EN_PKT_ILL_ACKID_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN_MASK (0x00040000u)
#define CSL_SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_SP_RATE_EN_PKT_CRC_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN_MASK (0x00020000u)
#define CSL_SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN_SHIFT (0x00000011u)
#define CSL_SRIO_RIO_SP_RATE_EN_PKT_ILL_SIZE_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN_MASK (0x00004000u)
#define CSL_SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_SP_RATE_EN_DSCRAM_LOS_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN_MASK (0x00000020u)
#define CSL_SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN_SHIFT (0x00000005u)
#define CSL_SRIO_RIO_SP_RATE_EN_LR_ACKID_ILL_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_PROT_ERR_EN_MASK (0x00000010u)
#define CSL_SRIO_RIO_SP_RATE_EN_PROT_ERR_EN_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_SP_RATE_EN_PROT_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN_MASK (0x00000004u)
#define CSL_SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_SP_RATE_EN_DELIN_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN_MASK (0x00000002u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_SP_RATE_EN_CS_ACK_ILL_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_LINK_TO_EN_MASK (0x00000001u)
#define CSL_SRIO_RIO_SP_RATE_EN_LINK_TO_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_RATE_EN_LINK_TO_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_RATE_EN_RESETVAL (0x00000000u)

/* rio_sp_err_attr_capt */

#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_INFO_TYPE_MASK (0xE0000000u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_INFO_TYPE_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_INFO_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_ERR_TYPE_MASK (0x1F000000u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_ERR_TYPE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_ERR_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_IMPL_DEP_MASK (0x00FFFFF0u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_IMPL_DEP_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_IMPL_DEP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_VAL_CAPT_MASK (0x00000001u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_VAL_CAPT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_VAL_CAPT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_ATTR_CAPT_RESETVAL (0x00000000u)

/* rio_sp_err_capt_0 */

#define CSL_SRIO_RIO_SP_ERR_CAPT_0_CAPT_0_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_SP_ERR_CAPT_0_CAPT_0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_CAPT_0_CAPT_0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_CAPT_0_RESETVAL (0x00000000u)

/* rio_sp_err_capt_1 */

#define CSL_SRIO_RIO_SP_ERR_CAPT_1_CAPT_1_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_SP_ERR_CAPT_1_CAPT_1_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_CAPT_1_CAPT_1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_CAPT_1_RESETVAL (0x00000000u)

/* rio_sp_err_capt_2 */

#define CSL_SRIO_RIO_SP_ERR_CAPT_2_CAPT_2_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_SP_ERR_CAPT_2_CAPT_2_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_CAPT_2_CAPT_2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_CAPT_2_RESETVAL (0x00000000u)

/* rio_sp_err_capt_3 */

#define CSL_SRIO_RIO_SP_ERR_CAPT_3_CAPT_3_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_SP_ERR_CAPT_3_CAPT_3_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_CAPT_3_CAPT_3_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_CAPT_3_RESETVAL (0x00000000u)

/* rio_sp_err_rate */

#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RB_MASK (0xFF000000u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RB_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RB_RESETVAL (0x00000080u)

#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RR_MASK (0x00030000u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_PEAK_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_PEAK_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_PEAK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT_MASK (0x000000FFu)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_ERR_RATE_ERR_RATE_CNT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_ERR_RATE_RESETVAL (0x80000000u)

/* rio_sp_err_thresh */

#define CSL_SRIO_RIO_SP_ERR_THRESH_ERR_RFT_MASK (0xFF000000u)
#define CSL_SRIO_RIO_SP_ERR_THRESH_ERR_RFT_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SP_ERR_THRESH_ERR_RFT_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_SP_ERR_THRESH_ERR_RDT_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_SP_ERR_THRESH_ERR_RDT_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_ERR_THRESH_ERR_RDT_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_SP_ERR_THRESH_RESETVAL (0xFFFF0000u)

/* rio_lane_stat0 */

#define CSL_SRIO_RIO_LANE_STAT0_PORT_NUM_MASK (0xFF000000u)
#define CSL_SRIO_RIO_LANE_STAT0_PORT_NUM_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LANE_STAT0_PORT_NUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_LANE_NUM_MASK (0x00F00000u)
#define CSL_SRIO_RIO_LANE_STAT0_LANE_NUM_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_LANE_STAT0_LANE_NUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_TX_TYPE_MASK (0x00080000u)
#define CSL_SRIO_RIO_LANE_STAT0_TX_TYPE_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_LANE_STAT0_TX_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_TX_MODE_MASK (0x00040000u)
#define CSL_SRIO_RIO_LANE_STAT0_TX_MODE_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_LANE_STAT0_TX_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_RX_TYPE_MASK (0x00030000u)
#define CSL_SRIO_RIO_LANE_STAT0_RX_TYPE_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LANE_STAT0_RX_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_RX_INV_MASK (0x00008000u)
#define CSL_SRIO_RIO_LANE_STAT0_RX_INV_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_LANE_STAT0_RX_INV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_RX_TRN_MASK (0x00004000u)
#define CSL_SRIO_RIO_LANE_STAT0_RX_TRN_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_LANE_STAT0_RX_TRN_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_LANE_STAT0_RX_SYNC_MASK (0x00002000u)
#define CSL_SRIO_RIO_LANE_STAT0_RX_SYNC_SHIFT (0x0000000Du)
#define CSL_SRIO_RIO_LANE_STAT0_RX_SYNC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_RX_RDY_MASK (0x00001000u)
#define CSL_SRIO_RIO_LANE_STAT0_RX_RDY_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_LANE_STAT0_RX_RDY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_ERR_CNT_MASK (0x00000F00u)
#define CSL_SRIO_RIO_LANE_STAT0_ERR_CNT_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LANE_STAT0_ERR_CNT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_CHG_SYNC_MASK (0x00000080u)
#define CSL_SRIO_RIO_LANE_STAT0_CHG_SYNC_SHIFT (0x00000007u)
#define CSL_SRIO_RIO_LANE_STAT0_CHG_SYNC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_CHG_TRN_MASK (0x00000040u)
#define CSL_SRIO_RIO_LANE_STAT0_CHG_TRN_SHIFT (0x00000006u)
#define CSL_SRIO_RIO_LANE_STAT0_CHG_TRN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_STAT1_MASK (0x00000008u)
#define CSL_SRIO_RIO_LANE_STAT0_STAT1_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_LANE_STAT0_STAT1_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_LANE_STAT0_STAT2_7_MASK (0x00000007u)
#define CSL_SRIO_RIO_LANE_STAT0_STAT2_7_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LANE_STAT0_STAT2_7_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT0_RESETVAL (0x00004008u)

/* rio_lane_stat1 */

#define CSL_SRIO_RIO_LANE_STAT1_IDLE2_MASK (0x80000000u)
#define CSL_SRIO_RIO_LANE_STAT1_IDLE2_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_LANE_STAT1_IDLE2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_INFO_OK_MASK (0x40000000u)
#define CSL_SRIO_RIO_LANE_STAT1_INFO_OK_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_LANE_STAT1_INFO_OK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_CHG_MASK (0x20000000u)
#define CSL_SRIO_RIO_LANE_STAT1_CHG_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_LANE_STAT1_CHG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_IMPL_SPEC_MASK (0x10000000u)
#define CSL_SRIO_RIO_LANE_STAT1_IMPL_SPEC_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_LANE_STAT1_IMPL_SPEC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_LP_RX_TRN_MASK (0x08000000u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_RX_TRN_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_LANE_STAT1_LP_RX_TRN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_LP_WIDTH_MASK (0x07000000u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_WIDTH_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_WIDTH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_LP_LANE_NUM_MASK (0x00F00000u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_LANE_NUM_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_LANE_NUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_LP_TAP_M1_MASK (0x000C0000u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_TAP_M1_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_TAP_M1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_LP_TAP_P1_MASK (0x00030000u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_TAP_P1_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_TAP_P1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_LP_SCRM_MASK (0x00008000u)
#define CSL_SRIO_RIO_LANE_STAT1_LP_SCRM_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_LANE_STAT1_LP_SCRM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LANE_STAT1_RESETVAL (0x00000000u)

/* rio_plm_sp_imp_spec_ctl */

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP_MASK (0x80000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PAYL_CAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2_MASK (0x40000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1_MASK (0x20000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_USE_IDLE1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN_MASK (0x10000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLB_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT_MASK (0x04000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_FORCE_REINIT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT_MASK (0x02000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SOFT_RST_PORT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS_MASK (0x01000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_TX_BYPASS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN_MASK (0x00800000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_LLB_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST_MASK (0x00200000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_PORT_SELF_RST_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST_MASK (0x00100000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SELF_RST_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX_MASK (0x000C0000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_TX_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX_MASK (0x00030000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_SWAP_RX_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_DLT_THRESH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_IMP_SPEC_CTL_RESETVAL (0x00000000u)

/* rio_plm_sp_pwdn_ctl */

#define CSL_SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT_MASK (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_PWDN_CTL_PWDN_PORT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PWDN_CTL_RESETVAL (0x00000000u)

/* rio_plm_sp_status */

#define CSL_SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL_MASK (0x80000000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_MAX_DENIAL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_LINK_INIT_MASK (0x10000000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_LINK_INIT_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_LINK_INIT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_DLT_MASK (0x08000000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_DLT_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_DLT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_PORT_ERR_MASK (0x04000000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_PORT_ERR_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_PLM_SP_STATUS_PORT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL_MASK (0x02000000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_OUTPUT_FAIL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR_MASK (0x01000000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_OUTPUT_DEGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_RST_REQ_MASK (0x00010000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_RST_REQ_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_RST_REQ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_PBM_PW_MASK (0x00008000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_PBM_PW_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_PBM_PW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_TLM_PW_MASK (0x00004000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_TLM_PW_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_TLM_PW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_MECS_MASK (0x00001000u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_MECS_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_MECS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_PBM_INT_MASK (0x00000800u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_PBM_INT_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_PLM_SP_STATUS_PBM_INT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_TLM_INT_MASK (0x00000400u)
#define CSL_SRIO_RIO_PLM_SP_STATUS_TLM_INT_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_PLM_SP_STATUS_TLM_INT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_STATUS_RESETVAL (0x00000000u)

/* rio_plm_sp_int_enable */

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_MAX_DENIAL_MASK (0x80000000u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_MAX_DENIAL_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_MAX_DENIAL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_LINK_INIT_MASK (0x10000000u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_LINK_INIT_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_LINK_INIT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_DLT_MASK (0x08000000u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_DLT_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_DLT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_PORT_ERR_MASK (0x04000000u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_PORT_ERR_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_PORT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_OUTPUT_FAIL_MASK (0x02000000u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_OUTPUT_FAIL_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_OUTPUT_FAIL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_OUTPUT_DEGR_MASK (0x01000000u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_OUTPUT_DEGR_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_OUTPUT_DEGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_INT_ENABLE_RESETVAL (0x00000000u)

/* rio_plm_sp_pw_enable */

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_MAX_DENIAL_MASK (0x80000000u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_MAX_DENIAL_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_MAX_DENIAL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_LINK_INIT_MASK (0x10000000u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_LINK_INIT_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_LINK_INIT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_DLT_MASK (0x08000000u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_DLT_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_DLT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_PORT_ERR_MASK (0x04000000u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_PORT_ERR_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_PORT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_OUTPUT_FAIL_MASK (0x02000000u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_OUTPUT_FAIL_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_OUTPUT_FAIL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_OUTPUT_DEGR_MASK (0x01000000u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_OUTPUT_DEGR_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_OUTPUT_DEGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PW_ENABLE_RESETVAL (0x00000000u)

/* rio_plm_sp_event_gen */

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_MAX_DENIAL_MASK (0x80000000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_MAX_DENIAL_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_MAX_DENIAL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_LINK_INIT_MASK (0x10000000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_LINK_INIT_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_LINK_INIT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_DLT_MASK (0x08000000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_DLT_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_DLT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_PORT_ERR_MASK (0x04000000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_PORT_ERR_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_PORT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_OUTPUT_FAIL_MASK (0x02000000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_OUTPUT_FAIL_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_OUTPUT_FAIL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_OUTPUT_DEGR_MASK (0x01000000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_OUTPUT_DEGR_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_OUTPUT_DEGR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_RST_REQ_MASK (0x00010000u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_RST_REQ_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_RST_REQ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_EVENT_GEN_RESETVAL (0x00000000u)

/* rio_plm_sp_all_int_en */

#define CSL_SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN_MASK (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_ALL_INT_EN_IRQ_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_ALL_INT_EN_RESETVAL (0x00000000u)

/* rio_plm_sp_all_pw_en */

#define CSL_SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN_MASK (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_ALL_PW_EN_PW_EN_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PLM_SP_ALL_PW_EN_RESETVAL (0x00000001u)

/* rio_plm_sp_path_ctl */

#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_ID_MASK (0x001F0000u)
#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION_MASK (0x00000700u)
#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE_MASK (0x00000007u)
#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_PATH_CTL_RESETVAL (0x00000000u)

/* rio_plm_sp_discovery_timer */

#define CSL_SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER_MASK (0xF0000000u)
#define CSL_SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_DISCOVERY_TIMER_DISCOVERY_TIMER_RESETVAL (0x00000007u)

#define CSL_SRIO_RIO_PLM_SP_DISCOVERY_TIMER_RESETVAL (0x70000000u)

/* rio_plm_sp_silence_timer */

#define CSL_SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER_MASK (0xF0000000u)
#define CSL_SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_SILENCE_TIMER_SILENCE_TIMER_RESETVAL (0x00000009u)

#define CSL_SRIO_RIO_PLM_SP_SILENCE_TIMER_RESETVAL (0x90000000u)

/* rio_plm_sp_vmin_exp */

#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP_MASK (0x1F000000u)
#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_VMIN_EXP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_IMAX_MASK (0x000F0000u)
#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_IMAX_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_IMAX_RESETVAL (0x00000003u)

#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_MMAX_MASK (0x00000F00u)
#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_MMAX_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_MMAX_RESETVAL (0x00000003u)

#define CSL_SRIO_RIO_PLM_SP_VMIN_EXP_RESETVAL (0x00030300u)

/* rio_plm_sp_pol_ctl */

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX3_POL_MASK (0x00080000u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX3_POL_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX3_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX2_POL_MASK (0x00040000u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX2_POL_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX2_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX1_POL_MASK (0x00020000u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX1_POL_SHIFT (0x00000011u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX1_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX0_POL_MASK (0x00010000u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX0_POL_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_TX0_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX3_POL_MASK (0x00000008u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX3_POL_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX3_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX2_POL_MASK (0x00000004u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX2_POL_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX2_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX1_POL_MASK (0x00000002u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX1_POL_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX1_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX0_POL_MASK (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX0_POL_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RX0_POL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_POL_CTL_RESETVAL (0x00000000u)

/* rio_plm_sp_denial_ctl */

#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA_MASK (0x20000000u)
#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_PNA_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY_MASK (0x10000000u)
#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_CNT_RTY_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH_MASK (0x000000FFu)
#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_DENIAL_THRESH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_DENIAL_CTL_RESETVAL (0x30000000u)

/* rio_plm_sp_rcvd_mecs */

#define CSL_SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT_MASK (0x000000FFu)
#define CSL_SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_RCVD_MECS_CMD_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_RCVD_MECS_RESETVAL (0x00000000u)

/* rio_plm_sp_mecs_fwd */

#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION_MASK (0x000000FEu)
#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_SUBSCRIPTION_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_MULT_CS_MASK (0x00000001u)
#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_MULT_CS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_MULT_CS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_MECS_FWD_RESETVAL (0x00000000u)

/* rio_plm_sp_long_cs_tx1 */

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0_MASK (0x70000000u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0_MASK (0x03F00000u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1_MASK (0x0003F000u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_PAR_1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB_MASK (0x00000100u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_CS_EMB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1_MASK (0x00000070u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_STYPE_1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD_MASK (0x00000007u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_CMD_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX1_RESETVAL (0x00000000u)

/* rio_plm_sp_long_cs_tx2 */

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2_MASK (0x70000000u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_STYPE_2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM_MASK (0x07FF0000u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_PARM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_SP_LONG_CS_TX2_RESETVAL (0x00000000u)

/* rio_tlm_sp_control */

#define CSL_SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT_MASK (0x40000000u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_PORTGROUP_SELECT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT_MASK (0x30000000u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_VOQ_SELECT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS_MASK (0x00200000u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_TGT_ID_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS_MASK (0x00100000u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_MTC_TGT_ID_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_CONTROL_LENGTH_MASK (0x0000F000u)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_LENGTH_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_TLM_SP_CONTROL_LENGTH_RESETVAL (0x00000009u)

#define CSL_SRIO_RIO_TLM_SP_CONTROL_RESETVAL (0x00009000u)

/* rio_tlm_sp_status */

#define CSL_SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_STATUS_IG_BAD_VC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER_MASK (0x00100000u)
#define CSL_SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_TLM_SP_STATUS_IG_BRR_FILTER_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_STATUS_RESETVAL (0x00000000u)

/* rio_tlm_sp_int_enable */

#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_IG_BAD_VC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER_MASK (0x00100000u)
#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_IG_BRR_FILTER_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_INT_ENABLE_RESETVAL (0x00000000u)

/* rio_tlm_sp_pw_enable */

#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_IG_BAD_VC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER_MASK (0x00100000u)
#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_IG_BRR_FILTER_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_PW_ENABLE_RESETVAL (0x00000000u)

/* rio_tlm_sp_event_gen */

#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_IG_BAD_VC_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_IG_BAD_VC_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_IG_BAD_VC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_IG_BRR_FILTER_MASK (0x00100000u)
#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_IG_BRR_FILTER_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_IG_BRR_FILTER_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_EVENT_GEN_RESETVAL (0x00000000u)

/* rio_tlm_sp_brr_0_ctl */

#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_ENABLE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM_MASK (0x04000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_ROUTE_MR_TO_LLM_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE_MASK (0x01000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_PRIVATE_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_0_CTL_RESETVAL (0x05000000u)

/* rio_tlm_sp_brr_0_pattern_match */

#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_PATTERN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_MATCH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_0_PATTERN_MATCH_RESETVAL (0x00000000u)

/* rio_tlm_sp_brr_1_ctl */

#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_ENABLE_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_ENABLE_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_ENABLE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_ROUTE_MR_TO_LLM_MASK (0x04000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_ROUTE_MR_TO_LLM_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_ROUTE_MR_TO_LLM_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_PRIVATE_MASK (0x01000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_PRIVATE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_PRIVATE_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_1_CTL_RESETVAL (0x05000000u)

/* rio_tlm_sp_brr_1_pattern_match */

#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_PATTERN_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_PATTERN_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_PATTERN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_MATCH_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_MATCH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_MATCH_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_TLM_SP_BRR_1_PATTERN_MATCH_RESETVAL (0x0000FFFFu)

/* rio_tlm_sp_brr_2_ctl */

#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_ENABLE_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_ENABLE_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_ENABLE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_ROUTE_MR_TO_LLM_MASK (0x04000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_ROUTE_MR_TO_LLM_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_ROUTE_MR_TO_LLM_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_PRIVATE_MASK (0x01000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_PRIVATE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_PRIVATE_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_2_CTL_RESETVAL (0x05000000u)

/* rio_tlm_sp_brr_2_pattern_match */

#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_PATTERN_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_PATTERN_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_PATTERN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_MATCH_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_MATCH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_MATCH_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_TLM_SP_BRR_2_PATTERN_MATCH_RESETVAL (0x0000FFFFu)

/* rio_tlm_sp_brr_3_ctl */

#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_ENABLE_MASK (0x80000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_ENABLE_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_ENABLE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_ROUTE_MR_TO_LLM_MASK (0x04000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_ROUTE_MR_TO_LLM_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_ROUTE_MR_TO_LLM_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_PRIVATE_MASK (0x01000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_PRIVATE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_PRIVATE_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_TLM_SP_BRR_3_CTL_RESETVAL (0x05000000u)

/* rio_tlm_sp_brr_3_pattern_match */

#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_PATTERN_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_PATTERN_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_PATTERN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_MATCH_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_MATCH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_MATCH_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_TLM_SP_BRR_3_PATTERN_MATCH_RESETVAL (0x0000FFFFu)

/* rio_pbm_sp_control */

#define CSL_SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE_MASK (0x00000030u)
#define CSL_SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK_MASK (0x00000007u)
#define CSL_SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_CONTROL_EG_REORDER_STICK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_CONTROL_RESETVAL (0x00000000u)

/* rio_pbm_sp_status */

#define CSL_SRIO_RIO_PBM_SP_STATUS_IG_EMPTY_MASK (0x00010000u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_IG_EMPTY_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_IG_EMPTY_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_EMPTY_MASK (0x00008000u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_EMPTY_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_EMPTY_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW_MASK (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_DATA_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW_MASK (0x00000008u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_CRQ_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL_MASK (0x00000002u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_BAD_CHANNEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET_MASK (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_STATUS_EG_BABBLE_PACKET_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_STATUS_RESETVAL (0x00018000u)

/* rio_pbm_sp_int_enable */

#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW_MASK (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_DATA_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW_MASK (0x00000008u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_CRQ_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL_MASK (0x00000002u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_BAD_CHANNEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET_MASK (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_EG_BABBLE_PACKET_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_INT_ENABLE_RESETVAL (0x00000000u)

/* rio_pbm_sp_pw_enable */

#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW_MASK (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_DATA_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW_MASK (0x00000008u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_CRQ_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL_MASK (0x00000002u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_BAD_CHANNEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET_MASK (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_EG_BABBLE_PACKET_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_PW_ENABLE_RESETVAL (0x00000000u)

/* rio_pbm_sp_event_gen */

#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW_MASK (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_DATA_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW_MASK (0x00000008u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_CRQ_OVERFLOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL_MASK (0x00000002u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_BAD_CHANNEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET_MASK (0x00000001u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_EG_BABBLE_PACKET_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_EVENT_GEN_RESETVAL (0x00000000u)

/* rio_pbm_sp_ig_resources */

#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_DATANODES_MASK (0x03FF0000u)
#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_DATANODES_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_DATANODES_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_TAGS_MASK (0x000003FFu)
#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_TAGS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_TAGS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_SP_IG_RESOURCES_RESETVAL (0x00000000u)

/* rio_pbm_sp_eg_resources */

#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_DATANODES_MASK (0x03FF0000u)
#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_DATANODES_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_DATANODES_RESETVAL (0x0000004Au)

#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_CRQ_ENTRIES_MASK (0x0000007Fu)
#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_CRQ_ENTRIES_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_CRQ_ENTRIES_RESETVAL (0x00000021u)

#define CSL_SRIO_RIO_PBM_SP_EG_RESOURCES_RESETVAL (0x004A0021u)

/* rio_pbm_sp_ig_watermark0 */

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM_MASK (0x03FF0000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0CRF_WM_RESETVAL (0x0000003Fu)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM_MASK (0x000003FFu)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_PRIO0_WM_RESETVAL (0x00000048u)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK0_RESETVAL (0x003F0048u)

/* rio_pbm_sp_ig_watermark1 */

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM_MASK (0x03FF0000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1CRF_WM_RESETVAL (0x0000002Du)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM_MASK (0x000003FFu)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_PRIO1_WM_RESETVAL (0x00000036u)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK1_RESETVAL (0x002D0036u)

/* rio_pbm_sp_ig_watermark2 */

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM_MASK (0x03FF0000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2CRF_WM_RESETVAL (0x0000001Bu)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM_MASK (0x000003FFu)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_PRIO2_WM_RESETVAL (0x00000024u)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK2_RESETVAL (0x001B0024u)

/* rio_pbm_sp_ig_watermark3 */

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM_MASK (0x03FF0000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3CRF_WM_RESETVAL (0x00000009u)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM_MASK (0x000003FFu)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_PRIO3_WM_RESETVAL (0x00000012u)

#define CSL_SRIO_RIO_PBM_SP_IG_WATERMARK3_RESETVAL (0x00090012u)

/* rio_pid */

#define CSL_SRIO_RIO_PID_MINOR_MASK      (0x0000003Fu)
#define CSL_SRIO_RIO_PID_MINOR_SHIFT     (0x00000000u)
#define CSL_SRIO_RIO_PID_MINOR_RESETVAL  (0x00000000u)

#define CSL_SRIO_RIO_PID_CUSTOM_MASK     (0x000000C0u)
#define CSL_SRIO_RIO_PID_CUSTOM_SHIFT    (0x00000006u)
#define CSL_SRIO_RIO_PID_CUSTOM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PID_MAJOR_MASK      (0x00000700u)
#define CSL_SRIO_RIO_PID_MAJOR_SHIFT     (0x00000008u)
#define CSL_SRIO_RIO_PID_MAJOR_RESETVAL  (0x00000001u)

#define CSL_SRIO_RIO_PID_RTL_MASK        (0x0000F800u)
#define CSL_SRIO_RIO_PID_RTL_SHIFT       (0x0000000Bu)
#define CSL_SRIO_RIO_PID_RTL_RESETVAL    (0x00000003u)

#define CSL_SRIO_RIO_PID_FUNC_MASK       (0x0FFF0000u)
#define CSL_SRIO_RIO_PID_FUNC_SHIFT      (0x00000010u)
#define CSL_SRIO_RIO_PID_FUNC_RESETVAL   (0x000004ABu)

#define CSL_SRIO_RIO_PID_BU_MASK         (0x30000000u)
#define CSL_SRIO_RIO_PID_BU_SHIFT        (0x0000001Cu)
#define CSL_SRIO_RIO_PID_BU_RESETVAL     (0x00000000u)

#define CSL_SRIO_RIO_PID_SCHEME_MASK     (0xC0000000u)
#define CSL_SRIO_RIO_PID_SCHEME_SHIFT    (0x0000001Eu)
#define CSL_SRIO_RIO_PID_SCHEME_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PID_RESETVAL        (0x44AB1900u)

/* rio_pcr */

#define CSL_SRIO_RIO_PCR_FREE_MASK       (0x00000001u)
#define CSL_SRIO_RIO_PCR_FREE_SHIFT      (0x00000000u)
#define CSL_SRIO_RIO_PCR_FREE_RESETVAL   (0x00000001u)

#define CSL_SRIO_RIO_PCR_SOFT_MASK       (0x00000002u)
#define CSL_SRIO_RIO_PCR_SOFT_SHIFT      (0x00000001u)
#define CSL_SRIO_RIO_PCR_SOFT_RESETVAL   (0x00000000u)

#define CSL_SRIO_RIO_PCR_PEREN_MASK      (0x00000004u)
#define CSL_SRIO_RIO_PCR_PEREN_SHIFT     (0x00000002u)
#define CSL_SRIO_RIO_PCR_PEREN_RESETVAL  (0x00000000u)

#define CSL_SRIO_RIO_PCR_LOCAL_DIS_MASK  (0x00000008u)
#define CSL_SRIO_RIO_PCR_LOCAL_DIS_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PCR_LOCAL_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PCR_RESTORE_MASK    (0x00000010u)
#define CSL_SRIO_RIO_PCR_RESTORE_SHIFT   (0x00000004u)
#define CSL_SRIO_RIO_PCR_RESTORE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PCR_RESETVAL        (0x00000001u)

/* rio_per_set_cntl */

#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES0_PRBS_OVR_MASK (0x00000001u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES0_PRBS_OVR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES0_PRBS_OVR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES1_PRBS_OVR_MASK (0x00000002u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES1_PRBS_OVR_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES1_PRBS_OVR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES2_PRBS_OVR_MASK (0x00000004u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES2_PRBS_OVR_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES2_PRBS_OVR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES3_PRBS_OVR_MASK (0x00000008u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES3_PRBS_OVR_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PER_SET_CNTL_SERDES3_PRBS_OVR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT_MASK (0x000000F0u)
#define CSL_SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PER_SET_CNTL_PRESCALER_SELECT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI_MASK (0x00000E00u)
#define CSL_SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_PER_SET_CNTL_CBA_TRANS_PRI_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM_MASK (0x00007000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI0_WM_RESETVAL (0x00000003u)

#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM_MASK (0x00038000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI1_WM_RESETVAL (0x00000002u)

#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM_MASK (0x001C0000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM_SHIFT (0x00000012u)
#define CSL_SRIO_RIO_PER_SET_CNTL_TX_PRI2_WM_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS_MASK (0x00200000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_PER_SET_CNTL_PROMOTE_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE_MASK (0x00C00000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_PER_SET_CNTL_TXU_RXU_LEND_SWAP_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE_MASK (0x01000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PER_SET_CNTL_BOOT_COMPLETE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE_MASK (0x06000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_PER_SET_CNTL_AMU_LEND_SWAP_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS_MASK (0x08000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_PER_SET_CNTL_LOG_TGT_ID_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE_MASK (0x30000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PER_SET_CNTL_LSU_LEND_SWAP_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE_MASK (0xC0000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_PER_SET_CNTL_MAU_LEND_SWAP_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL_RESETVAL (0x00053800u)

/* rio_per_set_cntl1 */

#define CSL_SRIO_RIO_PER_SET_CNTL1_CRF_MASK (0x00000001u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_CRF_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_CRF_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK_MASK (0x00000002u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_RXU_WATERMARK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL_MASK (0x0000000Cu)
#define CSL_SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_SYS_CLK_SEL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_MASK (0x000000F0u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_LOOPBACK_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL1_COS_EN_MASK (0x00000100u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_COS_EN_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PER_SET_CNTL1_COS_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_SET_CNTL1_RESETVAL (0x00000000u)

/* rio_gbl_en */

#define CSL_SRIO_RIO_GBL_EN_EN_MASK      (0x00000001u)
#define CSL_SRIO_RIO_GBL_EN_EN_SHIFT     (0x00000000u)
#define CSL_SRIO_RIO_GBL_EN_EN_RESETVAL  (0x00000001u)

#define CSL_SRIO_RIO_GBL_EN_RESETVAL     (0x00000001u)

/* rio_gbl_en_stat */

#define CSL_SRIO_RIO_GBL_EN_STAT_GBL_EN_STAT_MASK (0x00000001u)
#define CSL_SRIO_RIO_GBL_EN_STAT_GBL_EN_STAT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_GBL_EN_STAT_GBL_EN_STAT_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_GBL_EN_STAT_BLKX_EN_STAT_MASK (0x000007FEu)
#define CSL_SRIO_RIO_GBL_EN_STAT_BLKX_EN_STAT_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_GBL_EN_STAT_BLKX_EN_STAT_RESETVAL (0x000003FFu)

#define CSL_SRIO_RIO_GBL_EN_STAT_RESETVAL (0x000007FFu)

/* rio_multiid_reg */

#define CSL_SRIO_RIO_MULTIID_REG_16B_NODEID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_MULTIID_REG_16B_NODEID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_MULTIID_REG_16B_NODEID_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_MULTIID_REG_8B_NODEID_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_MULTIID_REG_8B_NODEID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_MULTIID_REG_8B_NODEID_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_MULTIID_REG_RESETVAL (0x00FFFFFFu)

/* rio_err_rst_evnt_icsr */

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_MCAST_INT_RECEIVED_MASK (0x00000001u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_MCAST_INT_RECEIVED_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_MCAST_INT_RECEIVED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT_WRITE_IN_RECEIVED_MASK (0x00000002u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT_WRITE_IN_RECEIVED_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT_WRITE_IN_RECEIVED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_LLERR_CAPTURE_MASK (0x00000004u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_LLERR_CAPTURE_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_LLERR_CAPTURE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT0_ERR_MASK (0x00000100u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT0_ERR_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT0_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT1_ERR_MASK (0x00000200u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT1_ERR_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT1_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT2_ERR_MASK (0x00000400u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT2_ERR_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT2_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT3_ERR_MASK (0x00000800u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT3_ERR_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_PORT3_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_DEVICE_RST_INT_MASK (0x00010000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_DEVICE_RST_INT_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_DEVICE_RST_INT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICSR_RESETVAL (0x00000000u)

/* rio_err_rst_evnt_iccr */

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_MCAST_INT_RECEIVED_MASK (0x00000001u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_MCAST_INT_RECEIVED_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_MCAST_INT_RECEIVED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT_WRITE_IN_RECEIVED_MASK (0x00000002u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT_WRITE_IN_RECEIVED_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT_WRITE_IN_RECEIVED_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_LLERR_CAPTURE_MASK (0x00000004u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_LLERR_CAPTURE_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_LLERR_CAPTURE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT0_ERR_MASK (0x00000100u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT0_ERR_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT0_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT1_ERR_MASK (0x00000200u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT1_ERR_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT1_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT2_ERR_MASK (0x00000400u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT2_ERR_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT2_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT3_ERR_MASK (0x00000800u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT3_ERR_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_PORT3_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_DEVICE_RST_INT_MASK (0x00010000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_DEVICE_RST_INT_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_DEVICE_RST_INT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICCR_RESETVAL (0x00000000u)

/* rio_amu_icsr */

#define CSL_SRIO_RIO_AMU_ICSR_CPRIVID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_AMU_ICSR_CPRIVID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_ICSR_CPRIVID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICSR_RESETVAL   (0x00000000u)

/* rio_amu_iccr */

#define CSL_SRIO_RIO_AMU_ICCR_CPRIVID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_AMU_ICCR_CPRIVID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_ICCR_CPRIVID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICCR_RESETVAL   (0x00000000u)

/* rio_lsu0_module_icrr */

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR0_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR1_MASK (0x000000F0u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR2_MASK (0x00000F00u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR2_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR3_MASK (0x0000F000u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR3_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR3_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR4_MASK (0x000F0000u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR4_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR4_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR5_MASK (0x00F00000u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR5_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR5_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR6_MASK (0x0F000000u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR6_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR6_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR7_MASK (0xF0000000u)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR7_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_ICR7_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU0_MODULE_ICRR_RESETVAL (0x00000000u)

/* rio_lsu1_module_icrr */

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR0_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR1_MASK (0x000000F0u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR2_MASK (0x00000F00u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR2_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR3_MASK (0x0000F000u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR3_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR3_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR4_MASK (0x000F0000u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR4_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR4_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR5_MASK (0x00F00000u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR5_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR5_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR6_MASK (0x0F000000u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR6_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR6_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR7_MASK (0xF0000000u)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR7_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_ICR7_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU1_MODULE_ICRR_RESETVAL (0x00000000u)

/* rio_err_rst_evnt_icrr */

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR0_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR1_MASK (0x000000F0u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR2_MASK (0x00000F00u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR2_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_ICR2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR_RESETVAL (0x00000000u)

/* rio_err_rst_evnt_icrr2 */

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR8_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR8_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR8_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR9_MASK (0x000000F0u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR9_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR9_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR10_MASK (0x00000F00u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR10_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR10_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR11_MASK (0x0000F000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR11_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_ICR11_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR2_RESETVAL (0x00000000u)

/* rio_err_rst_evnt_icrr3 */

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR3_ICR16_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR3_ICR16_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR3_ICR16_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_RST_EVNT_ICRR3_RESETVAL (0x00000000u)

/* rio_amu_icrr1 */

#define CSL_SRIO_RIO_AMU_ICRR1_ICR0_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR1_MASK (0x000000F0u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR2_MASK (0x00000F00u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR2_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR3_MASK (0x0000F000u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR3_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR3_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR4_MASK (0x000F0000u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR4_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR4_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR5_MASK (0x00F00000u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR5_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR5_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR6_MASK (0x0F000000u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR6_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR6_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_ICR7_MASK (0xF0000000u)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR7_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_AMU_ICRR1_ICR7_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR1_RESETVAL  (0x00000000u)

/* rio_amu_icrr2 */

#define CSL_SRIO_RIO_AMU_ICRR2_ICR8_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR8_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR8_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR9_MASK (0x000000F0u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR9_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR9_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR10_MASK (0x00000F00u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR10_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR10_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR11_MASK (0x0000F000u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR11_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR11_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR12_MASK (0x000F0000u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR12_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR12_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR13_MASK (0x00F00000u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR13_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR13_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR14_MASK (0x0F000000u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR14_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR14_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_ICR15_MASK (0xF0000000u)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR15_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_AMU_ICRR2_ICR15_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_ICRR2_RESETVAL  (0x00000000u)

/* rio_interrupt_ctl */

#define CSL_SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE_MASK (0x00000001u)
#define CSL_SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_INTERRUPT_CTL_DBLL_ROUTE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_INTERRUPT_CTL_RESETVAL (0x00000000u)

/* rio_intdst_decode */

#define CSL_SRIO_RIO_INTDST_DECODE_ISDR_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_INTDST_DECODE_ISDR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_INTDST_DECODE_ISDR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_INTDST_DECODE_RESETVAL (0x00000000u)

/* rio_intdst_rate_cnt */

#define CSL_SRIO_RIO_INTDST_RATE_CNT_COUNT_DOWN_VALUE_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_INTDST_RATE_CNT_COUNT_DOWN_VALUE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_INTDST_RATE_CNT_COUNT_DOWN_VALUE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_INTDST_RATE_CNT_RESETVAL (0x00000000u)

/* rio_intdst_rate_dis */

#define CSL_SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_INTDST_RATE_DIS_RATEN_DIS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_INTDST_RATE_DIS_RESETVAL (0x00000000u)

/* rio_amu_srcid_map */

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID0_8_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID0_8_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID0_8_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID1_9_MASK (0x000000F0u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID1_9_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID1_9_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID2_10_MASK (0x00000F00u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID2_10_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID2_10_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID3_11_MASK (0x0000F000u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID3_11_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID3_11_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID4_12_MASK (0x000F0000u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID4_12_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID4_12_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID5_13_MASK (0x00F00000u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID5_13_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID5_13_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID6_14_MASK (0x0F000000u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID6_14_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID6_14_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID7_15_MASK (0xF0000000u)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID7_15_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_AMU_SRCID_MAP_CPRIVID7_15_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_SRCID_MAP_RESETVAL (0x00000000u)

/* rio_amu_priority_map */

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI0_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI1_MASK (0x000000F0u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI1_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI2_MASK (0x00000F00u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI2_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI3_MASK (0x0000F000u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI3_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI3_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI4_MASK (0x000F0000u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI4_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI4_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI5_MASK (0x00F00000u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI5_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI5_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI6_MASK (0x0F000000u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI6_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI6_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI7_MASK (0xF0000000u)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI7_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_DMA_PRI7_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_PRIORITY_MAP_RESETVAL (0x00000000u)

/* rio_amu_capt0_map */

#define CSL_SRIO_RIO_AMU_CAPT0_MAP_TRANS_ADDRESS_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_AMU_CAPT0_MAP_TRANS_ADDRESS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_CAPT0_MAP_TRANS_ADDRESS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_CAPT0_MAP_RESETVAL (0x00000000u)

/* rio_amu_capt1_map */

#define CSL_SRIO_RIO_AMU_CAPT1_MAP_CMSTID_MASK (0x000000FFu)
#define CSL_SRIO_RIO_AMU_CAPT1_MAP_CMSTID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_CAPT1_MAP_CMSTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_CAPT1_MAP_CPRIVID_MASK (0x00000F00u)
#define CSL_SRIO_RIO_AMU_CAPT1_MAP_CPRIVID_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_AMU_CAPT1_MAP_CPRIVID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_CAPT1_MAP_DOORBELL_INFO_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_AMU_CAPT1_MAP_DOORBELL_INFO_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_CAPT1_MAP_DOORBELL_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_CAPT1_MAP_RESETVAL (0x00000000u)

/* rio_amu_window_pane */

#define CSL_SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC_MASK (0x00000003u)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_CMD_ENC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_PANE_PORT_ID_MASK (0x0000000Cu)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_PORT_ID_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_PORT_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE_MASK (0x0000C000u)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_ID_SIZE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_PANE_DESTID_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_DESTID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_WINDOW_PANE_DESTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_AMU_WINDOW_PANE_RESETVAL (0x00000000u)

/* rio_amu_flow_masks0 */

#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_AMU_FLOW_MASK_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_RESERVED_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_RESERVED_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_RESERVED_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_AMU_FLOW_MASKS0_RESETVAL (0xFFFFFFFFu)

/* rio_lsu_setup_reg0 */

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU0_CNT_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU0_CNT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU0_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU1_CNT_MASK (0x000000F0u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU1_CNT_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU1_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU2_CNT_MASK (0x00000F00u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU2_CNT_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU2_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU3_CNT_MASK (0x0000F000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU3_CNT_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU3_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU4_CNT_MASK (0x000F0000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU4_CNT_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU4_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU5_CNT_MASK (0x00F00000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU5_CNT_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU5_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU6_CNT_MASK (0x0F000000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU6_CNT_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU6_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU7_CNT_MASK (0xF0000000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU7_CNT_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_LSU7_CNT_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP0_MASK (0x0000001Fu)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP1_MASK (0x001F0000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP1_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LSU_SETUP_REG0_SHADOW_GRP1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_SETUP_REG0_RESETVAL (0x44444444u)

/* rio_lsu_setup_reg1 */

#define CSL_SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA_MASK (0x000000FFu)
#define CSL_SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_SETUP_REG1_LSU_EDMA_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT_MASK (0x00000300u)
#define CSL_SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LSU_SETUP_REG1_TIMEOUT_CNT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LSU_SETUP_REG1_RESETVAL (0x00000000u)

/* lsu_stat_reg */

#define CSL_SRIO_LSU_STAT_REG_LSUX_STAT_MASK (0xFFFFFFFFu)
#define CSL_SRIO_LSU_STAT_REG_LSUX_STAT_SHIFT (0x00000000u)
#define CSL_SRIO_LSU_STAT_REG_LSUX_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_LSU_STAT_REG_RESETVAL   (0x00000000u)

/* rio_lsu_flow_masks */

#define CSL_SRIO_RIO_LSU_FLOW_MASKS_LSU_FLOW_MASK_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LSU_FLOW_MASKS_LSU_FLOW_MASK_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LSU_FLOW_MASKS_LSU_FLOW_MASK_RESETVAL (0xFFFFFFFFu)

#define CSL_SRIO_RIO_LSU_FLOW_MASKS_RESETVAL (0xFFFFFFFFu)

/* rio_supervisor_id */

#define CSL_SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SUPERVISOR_ID_16B_SUPRVSR_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SUPERVISOR_ID_8B_SUPRVSR_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SUPERVISOR_ID_RESETVAL (0x00000000u)

/* rio_flow_cntl */

#define CSL_SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_FLOW_CNTL_FLOW_CNTL_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_FLOW_CNTL_TT_MASK   (0x00030000u)
#define CSL_SRIO_RIO_FLOW_CNTL_TT_SHIFT  (0x00000010u)
#define CSL_SRIO_RIO_FLOW_CNTL_TT_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_FLOW_CNTL_RESETVAL  (0x00010000u)

/* rio_tx_cppi_flow_masks */

#define CSL_SRIO_RIO_TX_CPPI_FLOW_MASKS_TX_QUEUE_FLOW_MASK_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_TX_CPPI_FLOW_MASKS_TX_QUEUE_FLOW_MASK_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TX_CPPI_FLOW_MASKS_TX_QUEUE_FLOW_MASK_RESETVAL (0xFFFFFFFFu)

#define CSL_SRIO_RIO_TX_CPPI_FLOW_MASKS_RESETVAL (0xFFFFFFFFu)

/* rio_tx_queue_sch_info */

#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO_MASK (0x000000FFu)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE0_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE1_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE2_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE3_INFO_MASK (0xFF000000u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE3_INFO_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_QUEUE3_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TX_QUEUE_SCH_INFO_RESETVAL (0x00000000u)

/* rio_garbage_coll_qid0 */

#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT_MASK (0x00003FFFu)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_TOUT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN_MASK (0x3FFF0000u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_GARBAGE_QID_LEN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_GARBAGE_COLL_QID0_RESETVAL (0x00000000u)

/* rio_garbage_coll_qid1 */

#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR_MASK (0x00003FFFu)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_TRANS_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY_MASK (0x3FFF0000u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_GARBAGE_QID_RETRY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_GARBAGE_COLL_QID1_RESETVAL (0x00000000u)

/* rio_garbage_coll_qid2 */

#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE_MASK (0x00003FFFu)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_SSIZE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG_MASK (0x3FFF0000u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_GARBAGE_QID_PROG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_GARBAGE_COLL_QID2_RESETVAL (0x00000000u)

/* revision_reg */

#define CSL_SRIO_REVISION_REG_REVMIN_MASK (0x0000003Fu)
#define CSL_SRIO_REVISION_REG_REVMIN_SHIFT (0x00000000u)
#define CSL_SRIO_REVISION_REG_REVMIN_RESETVAL (0x00000000u)

#define CSL_SRIO_REVISION_REG_CUSTOM_MASK (0x000000C0u)
#define CSL_SRIO_REVISION_REG_CUSTOM_SHIFT (0x00000006u)
#define CSL_SRIO_REVISION_REG_CUSTOM_RESETVAL (0x00000000u)

#define CSL_SRIO_REVISION_REG_REVMAJ_MASK (0x00000700u)
#define CSL_SRIO_REVISION_REG_REVMAJ_SHIFT (0x00000008u)
#define CSL_SRIO_REVISION_REG_REVMAJ_RESETVAL (0x00000001u)

#define CSL_SRIO_REVISION_REG_REVRTL_MASK (0x0000F800u)
#define CSL_SRIO_REVISION_REG_REVRTL_SHIFT (0x0000000Bu)
#define CSL_SRIO_REVISION_REG_REVRTL_RESETVAL (0x00000016u)

#define CSL_SRIO_REVISION_REG_FUNCTION_ID_MASK (0x0FFF0000u)
#define CSL_SRIO_REVISION_REG_FUNCTION_ID_SHIFT (0x00000010u)
#define CSL_SRIO_REVISION_REG_FUNCTION_ID_RESETVAL (0x00000E5Au)

#define CSL_SRIO_REVISION_REG_BU_MASK    (0x30000000u)
#define CSL_SRIO_REVISION_REG_BU_SHIFT   (0x0000001Cu)
#define CSL_SRIO_REVISION_REG_BU_RESETVAL (0x00000000u)

#define CSL_SRIO_REVISION_REG_RESERVED_MASK (0xC0000000u)
#define CSL_SRIO_REVISION_REG_RESERVED_SHIFT (0x0000001Eu)
#define CSL_SRIO_REVISION_REG_RESERVED_RESETVAL (0x00000001u)

#define CSL_SRIO_REVISION_REG_RESETVAL   (0x4E5AB100u)

/* perf_control_reg */

#define CSL_SRIO_PERF_CONTROL_REG_TIMEOUT_CNT_MASK (0x0000FFFFu)
#define CSL_SRIO_PERF_CONTROL_REG_TIMEOUT_CNT_SHIFT (0x00000000u)
#define CSL_SRIO_PERF_CONTROL_REG_TIMEOUT_CNT_RESETVAL (0x00000000u)

#define CSL_SRIO_PERF_CONTROL_REG_WARB_FIFO_DEPTH_MASK (0x003F0000u)
#define CSL_SRIO_PERF_CONTROL_REG_WARB_FIFO_DEPTH_SHIFT (0x00000010u)
#define CSL_SRIO_PERF_CONTROL_REG_WARB_FIFO_DEPTH_RESETVAL (0x00000020u)

#define CSL_SRIO_PERF_CONTROL_REG_RESETVAL (0x00200000u)

/* emulation_control_reg */

#define CSL_SRIO_EMULATION_CONTROL_REG_FREE_MASK (0x00000001u)
#define CSL_SRIO_EMULATION_CONTROL_REG_FREE_SHIFT (0x00000000u)
#define CSL_SRIO_EMULATION_CONTROL_REG_FREE_RESETVAL (0x00000000u)

#define CSL_SRIO_EMULATION_CONTROL_REG_SOFT_MASK (0x00000002u)
#define CSL_SRIO_EMULATION_CONTROL_REG_SOFT_SHIFT (0x00000001u)
#define CSL_SRIO_EMULATION_CONTROL_REG_SOFT_RESETVAL (0x00000000u)

#define CSL_SRIO_EMULATION_CONTROL_REG_LOOPBACK_EN_MASK (0x80000000u)
#define CSL_SRIO_EMULATION_CONTROL_REG_LOOPBACK_EN_SHIFT (0x0000001Fu)
#define CSL_SRIO_EMULATION_CONTROL_REG_LOOPBACK_EN_RESETVAL (0x00000001u)

#define CSL_SRIO_EMULATION_CONTROL_REG_RESETVAL (0x80000000u)

/* priority_control_reg */

#define CSL_SRIO_PRIORITY_CONTROL_REG_TX_PRIORITY_MASK (0x00000007u)
#define CSL_SRIO_PRIORITY_CONTROL_REG_TX_PRIORITY_SHIFT (0x00000000u)
#define CSL_SRIO_PRIORITY_CONTROL_REG_TX_PRIORITY_RESETVAL (0x00000000u)

#define CSL_SRIO_PRIORITY_CONTROL_REG_RX_PRIORITY_MASK (0x00070000u)
#define CSL_SRIO_PRIORITY_CONTROL_REG_RX_PRIORITY_SHIFT (0x00000010u)
#define CSL_SRIO_PRIORITY_CONTROL_REG_RX_PRIORITY_RESETVAL (0x00000000u)

#define CSL_SRIO_PRIORITY_CONTROL_REG_RESETVAL (0x00000000u)

/* qm_base_address_reg */

#define CSL_SRIO_QM_BASE_ADDRESS_REG_QM_BASE_MASK (0xFFFFFFFFu)
#define CSL_SRIO_QM_BASE_ADDRESS_REG_QM_BASE_SHIFT (0x00000000u)
#define CSL_SRIO_QM_BASE_ADDRESS_REG_QM_BASE_RESETVAL (0x00000000u)

#define CSL_SRIO_QM_BASE_ADDRESS_REG_RESETVAL (0x00000000u)

/* tx_channel_scheduler_config_reg */

#define CSL_SRIO_TX_CHANNEL_SCHEDULER_CONFIG_REG_PRIORITY_MASK (0x00000003u)
#define CSL_SRIO_TX_CHANNEL_SCHEDULER_CONFIG_REG_PRIORITY_SHIFT (0x00000000u)
#define CSL_SRIO_TX_CHANNEL_SCHEDULER_CONFIG_REG_PRIORITY_RESETVAL (0x00000000u)

#define CSL_SRIO_TX_CHANNEL_SCHEDULER_CONFIG_REG_RESETVAL (0x00000000u)

/* rio_dev_id */

#define CSL_SRIO_RIO_DEV_ID_DEV_ID_MASK  (0xFFFF0000u)
#define CSL_SRIO_RIO_DEV_ID_DEV_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_DEV_ID_DEV_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEV_ID_DEV_VEN_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_DEV_ID_DEV_VEN_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DEV_ID_DEV_VEN_ID_RESETVAL (0x00000030u)

#define CSL_SRIO_RIO_DEV_ID_RESETVAL     (0x00000030u)

/* rio_dev_info */

#define CSL_SRIO_RIO_DEV_INFO_DEVICE_REV_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_DEV_INFO_DEVICE_REV_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DEV_INFO_DEVICE_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEV_INFO_RESETVAL   (0x00000000u)

/* rio_asbly_id */

#define CSL_SRIO_RIO_ASBLY_ID_ASBLY_ID_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_ASBLY_ID_ASBLY_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_ASBLY_ID_ASBLY_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ASBLY_ID_ASBLY_VEN_ID_RESETVAL (0x00000030u)

#define CSL_SRIO_RIO_ASBLY_ID_RESETVAL   (0x00000030u)

/* rio_asbly_info */

#define CSL_SRIO_RIO_ASBLY_INFO_ASBLY_REV_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_ASBLY_INFO_ASBLY_REV_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_ASBLY_INFO_ASBLY_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ASBLY_INFO_EXT_FEAT_PTR_RESETVAL (0x00000100u)

#define CSL_SRIO_RIO_ASBLY_INFO_RESETVAL (0x00000100u)

/* rio_pe_feat */

#define CSL_SRIO_RIO_PE_FEAT_BRDG_MASK   (0x80000000u)
#define CSL_SRIO_RIO_PE_FEAT_BRDG_SHIFT  (0x0000001Fu)
#define CSL_SRIO_RIO_PE_FEAT_BRDG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_MEM_MASK    (0x40000000u)
#define CSL_SRIO_RIO_PE_FEAT_MEM_SHIFT   (0x0000001Eu)
#define CSL_SRIO_RIO_PE_FEAT_MEM_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PE_FEAT_PROC_MASK   (0x20000000u)
#define CSL_SRIO_RIO_PE_FEAT_PROC_SHIFT  (0x0000001Du)
#define CSL_SRIO_RIO_PE_FEAT_PROC_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PE_FEAT_SW_MASK     (0x10000000u)
#define CSL_SRIO_RIO_PE_FEAT_SW_SHIFT    (0x0000001Cu)
#define CSL_SRIO_RIO_PE_FEAT_SW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_MULT_P_MASK (0x08000000u)
#define CSL_SRIO_RIO_PE_FEAT_MULT_P_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_PE_FEAT_MULT_P_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_FLOW_ARB_MASK (0x00000800u)
#define CSL_SRIO_RIO_PE_FEAT_FLOW_ARB_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_PE_FEAT_FLOW_ARB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_MC_MASK     (0x00000400u)
#define CSL_SRIO_RIO_PE_FEAT_MC_SHIFT    (0x0000000Au)
#define CSL_SRIO_RIO_PE_FEAT_MC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_ERTC_MASK   (0x00000200u)
#define CSL_SRIO_RIO_PE_FEAT_ERTC_SHIFT  (0x00000009u)
#define CSL_SRIO_RIO_PE_FEAT_ERTC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_SRTC_MASK   (0x00000100u)
#define CSL_SRIO_RIO_PE_FEAT_SRTC_SHIFT  (0x00000008u)
#define CSL_SRIO_RIO_PE_FEAT_SRTC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_FLOW_CTRL_MASK (0x00000080u)
#define CSL_SRIO_RIO_PE_FEAT_FLOW_CTRL_SHIFT (0x00000007u)
#define CSL_SRIO_RIO_PE_FEAT_FLOW_CTRL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_CRF_MASK    (0x00000020u)
#define CSL_SRIO_RIO_PE_FEAT_CRF_SHIFT   (0x00000005u)
#define CSL_SRIO_RIO_PE_FEAT_CRF_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_CTLS_MASK   (0x00000010u)
#define CSL_SRIO_RIO_PE_FEAT_CTLS_SHIFT  (0x00000004u)
#define CSL_SRIO_RIO_PE_FEAT_CTLS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_EXT_FEA_MASK (0x00000008u)
#define CSL_SRIO_RIO_PE_FEAT_EXT_FEA_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PE_FEAT_EXT_FEA_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_EXT_AS_MASK (0x00000007u)
#define CSL_SRIO_RIO_PE_FEAT_EXT_AS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PE_FEAT_EXT_AS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PE_FEAT_RESETVAL    (0x60000000u)

/* rio_sw_port */

#define CSL_SRIO_RIO_SW_PORT_PORT_TOTAL_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_SW_PORT_PORT_TOTAL_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SW_PORT_PORT_TOTAL_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_SW_PORT_PORT_NUM_MASK (0x000000FFu)
#define CSL_SRIO_RIO_SW_PORT_PORT_NUM_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SW_PORT_PORT_NUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SW_PORT_RESETVAL    (0x00000400u)

/* rio_src_op */

#define CSL_SRIO_RIO_SRC_OP_G_READ_MASK  (0x80000000u)
#define CSL_SRIO_RIO_SRC_OP_G_READ_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SRC_OP_G_READ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_IREAD_MASK (0x40000000u)
#define CSL_SRIO_RIO_SRC_OP_G_IREAD_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_SRC_OP_G_IREAD_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_READ_OWN_MASK (0x20000000u)
#define CSL_SRIO_RIO_SRC_OP_G_READ_OWN_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_SRC_OP_G_READ_OWN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_DC_INVALIDATE_MASK (0x10000000u)
#define CSL_SRIO_RIO_SRC_OP_G_DC_INVALIDATE_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_SRC_OP_G_DC_INVALIDATE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_CASTOUT_MASK (0x08000000u)
#define CSL_SRIO_RIO_SRC_OP_G_CASTOUT_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_SRC_OP_G_CASTOUT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_DC_FLUSH_MASK (0x04000000u)
#define CSL_SRIO_RIO_SRC_OP_G_DC_FLUSH_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_SRC_OP_G_DC_FLUSH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_IO_READ_MASK (0x02000000u)
#define CSL_SRIO_RIO_SRC_OP_G_IO_READ_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_SRC_OP_G_IO_READ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_IC_INVALIDATE_MASK (0x01000000u)
#define CSL_SRIO_RIO_SRC_OP_G_IC_INVALIDATE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_SRC_OP_G_IC_INVALIDATE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_TLB_INVALIDATE_MASK (0x00800000u)
#define CSL_SRIO_RIO_SRC_OP_G_TLB_INVALIDATE_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_SRC_OP_G_TLB_INVALIDATE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_TLB_SYNC_MASK (0x00400000u)
#define CSL_SRIO_RIO_SRC_OP_G_TLB_SYNC_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_SRC_OP_G_TLB_SYNC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_RIO_RSVD_10_MASK (0x00200000u)
#define CSL_SRIO_RIO_SRC_OP_G_RIO_RSVD_10_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_SRC_OP_G_RIO_RSVD_10_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_G_RIO_RSVD_11_MASK (0x00100000u)
#define CSL_SRIO_RIO_SRC_OP_G_RIO_RSVD_11_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SRC_OP_G_RIO_RSVD_11_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_DS_TM_MASK   (0x00080000u)
#define CSL_SRIO_RIO_SRC_OP_DS_TM_SHIFT  (0x00000013u)
#define CSL_SRIO_RIO_SRC_OP_DS_TM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_DS_MASK      (0x00040000u)
#define CSL_SRIO_RIO_SRC_OP_DS_SHIFT     (0x00000012u)
#define CSL_SRIO_RIO_SRC_OP_DS_RESETVAL  (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_IMPLEMENT_DEF_MASK (0x00030000u)
#define CSL_SRIO_RIO_SRC_OP_IMPLEMENT_DEF_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SRC_OP_IMPLEMENT_DEF_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_READ_MASK    (0x00008000u)
#define CSL_SRIO_RIO_SRC_OP_READ_SHIFT   (0x0000000Fu)
#define CSL_SRIO_RIO_SRC_OP_READ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_WRITE_MASK   (0x00004000u)
#define CSL_SRIO_RIO_SRC_OP_WRITE_SHIFT  (0x0000000Eu)
#define CSL_SRIO_RIO_SRC_OP_WRITE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_STRM_WR_MASK (0x00002000u)
#define CSL_SRIO_RIO_SRC_OP_STRM_WR_SHIFT (0x0000000Du)
#define CSL_SRIO_RIO_SRC_OP_STRM_WR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_WR_RES_MASK  (0x00001000u)
#define CSL_SRIO_RIO_SRC_OP_WR_RES_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_SRC_OP_WR_RES_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_D_MSG_MASK   (0x00000800u)
#define CSL_SRIO_RIO_SRC_OP_D_MSG_SHIFT  (0x0000000Bu)
#define CSL_SRIO_RIO_SRC_OP_D_MSG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_DBELL_MASK   (0x00000400u)
#define CSL_SRIO_RIO_SRC_OP_DBELL_SHIFT  (0x0000000Au)
#define CSL_SRIO_RIO_SRC_OP_DBELL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_ACSWAP_MASK  (0x00000200u)
#define CSL_SRIO_RIO_SRC_OP_ACSWAP_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_SRC_OP_ACSWAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_ATSWAP_MASK  (0x00000100u)
#define CSL_SRIO_RIO_SRC_OP_ATSWAP_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SRC_OP_ATSWAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_A_INC_MASK   (0x00000080u)
#define CSL_SRIO_RIO_SRC_OP_A_INC_SHIFT  (0x00000007u)
#define CSL_SRIO_RIO_SRC_OP_A_INC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_A_DEC_MASK   (0x00000040u)
#define CSL_SRIO_RIO_SRC_OP_A_DEC_SHIFT  (0x00000006u)
#define CSL_SRIO_RIO_SRC_OP_A_DEC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_A_SET_MASK   (0x00000020u)
#define CSL_SRIO_RIO_SRC_OP_A_SET_SHIFT  (0x00000005u)
#define CSL_SRIO_RIO_SRC_OP_A_SET_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_A_CLEAR_MASK (0x00000010u)
#define CSL_SRIO_RIO_SRC_OP_A_CLEAR_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_SRC_OP_A_CLEAR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_A_SWAP_MASK  (0x00000008u)
#define CSL_SRIO_RIO_SRC_OP_A_SWAP_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_SRC_OP_A_SWAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_PORT_WR_MASK (0x00000004u)
#define CSL_SRIO_RIO_SRC_OP_PORT_WR_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_SRC_OP_PORT_WR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_IMPLEMENT_DEF2_MASK (0x00000003u)
#define CSL_SRIO_RIO_SRC_OP_IMPLEMENT_DEF2_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SRC_OP_IMPLEMENT_DEF2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SRC_OP_RESETVAL     (0x00000000u)

/* rio_dest_op */

#define CSL_SRIO_RIO_DEST_OP_G_READ_MASK (0x80000000u)
#define CSL_SRIO_RIO_DEST_OP_G_READ_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_DEST_OP_G_READ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_IREAD_MASK (0x40000000u)
#define CSL_SRIO_RIO_DEST_OP_G_IREAD_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_DEST_OP_G_IREAD_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_READ_OWN_MASK (0x20000000u)
#define CSL_SRIO_RIO_DEST_OP_G_READ_OWN_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_DEST_OP_G_READ_OWN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_DC_INVALIDATE_MASK (0x10000000u)
#define CSL_SRIO_RIO_DEST_OP_G_DC_INVALIDATE_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_DEST_OP_G_DC_INVALIDATE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_CASTOUT_MASK (0x08000000u)
#define CSL_SRIO_RIO_DEST_OP_G_CASTOUT_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_DEST_OP_G_CASTOUT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_DC_FLUSH_MASK (0x04000000u)
#define CSL_SRIO_RIO_DEST_OP_G_DC_FLUSH_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_DEST_OP_G_DC_FLUSH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_IO_READ_MASK (0x02000000u)
#define CSL_SRIO_RIO_DEST_OP_G_IO_READ_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_DEST_OP_G_IO_READ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_IC_INVALIDATE_MASK (0x01000000u)
#define CSL_SRIO_RIO_DEST_OP_G_IC_INVALIDATE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_DEST_OP_G_IC_INVALIDATE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_TLB_INVALIDATE_MASK (0x00800000u)
#define CSL_SRIO_RIO_DEST_OP_G_TLB_INVALIDATE_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_DEST_OP_G_TLB_INVALIDATE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_TLB_SYNC_MASK (0x00400000u)
#define CSL_SRIO_RIO_DEST_OP_G_TLB_SYNC_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_DEST_OP_G_TLB_SYNC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_RIO_RSVD_10_MASK (0x00200000u)
#define CSL_SRIO_RIO_DEST_OP_G_RIO_RSVD_10_SHIFT (0x00000015u)
#define CSL_SRIO_RIO_DEST_OP_G_RIO_RSVD_10_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_G_RIO_RSVD_11_MASK (0x00100000u)
#define CSL_SRIO_RIO_DEST_OP_G_RIO_RSVD_11_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_DEST_OP_G_RIO_RSVD_11_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_DS_TM_MASK  (0x00080000u)
#define CSL_SRIO_RIO_DEST_OP_DS_TM_SHIFT (0x00000013u)
#define CSL_SRIO_RIO_DEST_OP_DS_TM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_DS_MASK     (0x00040000u)
#define CSL_SRIO_RIO_DEST_OP_DS_SHIFT    (0x00000012u)
#define CSL_SRIO_RIO_DEST_OP_DS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_IMPLEMENT_DEF_MASK (0x00030000u)
#define CSL_SRIO_RIO_DEST_OP_IMPLEMENT_DEF_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_DEST_OP_IMPLEMENT_DEF_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_READ_MASK   (0x00008000u)
#define CSL_SRIO_RIO_DEST_OP_READ_SHIFT  (0x0000000Fu)
#define CSL_SRIO_RIO_DEST_OP_READ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_WRITE_MASK  (0x00004000u)
#define CSL_SRIO_RIO_DEST_OP_WRITE_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_DEST_OP_WRITE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_STRM_WR_MASK (0x00002000u)
#define CSL_SRIO_RIO_DEST_OP_STRM_WR_SHIFT (0x0000000Du)
#define CSL_SRIO_RIO_DEST_OP_STRM_WR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_WR_RES_MASK (0x00001000u)
#define CSL_SRIO_RIO_DEST_OP_WR_RES_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_DEST_OP_WR_RES_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_D_MSG_MASK  (0x00000800u)
#define CSL_SRIO_RIO_DEST_OP_D_MSG_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_DEST_OP_D_MSG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_DBELL_MASK  (0x00000400u)
#define CSL_SRIO_RIO_DEST_OP_DBELL_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_DEST_OP_DBELL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_ACSWAP_MASK (0x00000200u)
#define CSL_SRIO_RIO_DEST_OP_ACSWAP_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_DEST_OP_ACSWAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_ATSWAP_MASK (0x00000100u)
#define CSL_SRIO_RIO_DEST_OP_ATSWAP_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_DEST_OP_ATSWAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_A_INC_MASK  (0x00000080u)
#define CSL_SRIO_RIO_DEST_OP_A_INC_SHIFT (0x00000007u)
#define CSL_SRIO_RIO_DEST_OP_A_INC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_A_DEC_MASK  (0x00000040u)
#define CSL_SRIO_RIO_DEST_OP_A_DEC_SHIFT (0x00000006u)
#define CSL_SRIO_RIO_DEST_OP_A_DEC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_A_SET_MASK  (0x00000020u)
#define CSL_SRIO_RIO_DEST_OP_A_SET_SHIFT (0x00000005u)
#define CSL_SRIO_RIO_DEST_OP_A_SET_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_A_CLEAR_MASK (0x00000010u)
#define CSL_SRIO_RIO_DEST_OP_A_CLEAR_SHIFT (0x00000004u)
#define CSL_SRIO_RIO_DEST_OP_A_CLEAR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_A_SWAP_MASK (0x00000008u)
#define CSL_SRIO_RIO_DEST_OP_A_SWAP_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_DEST_OP_A_SWAP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_PORT_WR_MASK (0x00000004u)
#define CSL_SRIO_RIO_DEST_OP_PORT_WR_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_DEST_OP_PORT_WR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_IMPLEMENT_DEF2_MASK (0x00000003u)
#define CSL_SRIO_RIO_DEST_OP_IMPLEMENT_DEF2_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DEST_OP_IMPLEMENT_DEF2_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DEST_OP_RESETVAL    (0x00000000u)

/* rio_ds_info */

#define CSL_SRIO_RIO_DS_INFO_MAX_PDU_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_DS_INFO_MAX_PDU_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_DS_INFO_MAX_PDU_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DS_INFO_SEG_SUPPORT_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_DS_INFO_SEG_SUPPORT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DS_INFO_SEG_SUPPORT_RESETVAL (0x00000010u)

#define CSL_SRIO_RIO_DS_INFO_RESETVAL    (0x00000010u)

/* rio_ds_ll_ctl */

#define CSL_SRIO_RIO_DS_LL_CTL_TM_TYPES_MASK (0xF0000000u)
#define CSL_SRIO_RIO_DS_LL_CTL_TM_TYPES_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_DS_LL_CTL_TM_TYPES_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DS_LL_CTL_TM_MODE_MASK (0x0F000000u)
#define CSL_SRIO_RIO_DS_LL_CTL_TM_MODE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_DS_LL_CTL_TM_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_DS_LL_CTL_MTU_MASK  (0x000000FFu)
#define CSL_SRIO_RIO_DS_LL_CTL_MTU_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_DS_LL_CTL_MTU_RESETVAL (0x00000040u)

#define CSL_SRIO_RIO_DS_LL_CTL_RESETVAL  (0x00000040u)

/* rio_pe_ll_ctl */

#define CSL_SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL_MASK (0x00000007u)
#define CSL_SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PE_LL_CTL_EXT_ADDR_CTL_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PE_LL_CTL_RESETVAL  (0x00000001u)

/* rio_lcl_cfg_hbar */

#define CSL_SRIO_RIO_LCL_CFG_HBAR_LCSBA1_MASK (0x7FFF8000u)
#define CSL_SRIO_RIO_LCL_CFG_HBAR_LCSBA1_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_LCL_CFG_HBAR_LCSBA1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LCL_CFG_HBAR_LCSBA0_MASK (0x00007FFFu)
#define CSL_SRIO_RIO_LCL_CFG_HBAR_LCSBA0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LCL_CFG_HBAR_LCSBA0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LCL_CFG_HBAR_RESETVAL (0x00000000u)

/* rio_lcl_cfg_bar */

#define CSL_SRIO_RIO_LCL_CFG_BAR_LCSBA1_MASK (0x80000000u)
#define CSL_SRIO_RIO_LCL_CFG_BAR_LCSBA1_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_LCL_CFG_BAR_LCSBA1_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LCL_CFG_BAR_LCSBA0_MASK (0x7FFFFFFFu)
#define CSL_SRIO_RIO_LCL_CFG_BAR_LCSBA0_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LCL_CFG_BAR_LCSBA0_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LCL_CFG_BAR_RESETVAL (0x00000000u)

/* rio_base_id */

#define CSL_SRIO_RIO_BASE_ID_BASE_ID_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_BASE_ID_BASE_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_BASE_ID_BASE_ID_RESETVAL (0x000000FFu)

#define CSL_SRIO_RIO_BASE_ID_LARGE_BASE_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_BASE_ID_LARGE_BASE_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_BASE_ID_LARGE_BASE_ID_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_BASE_ID_RESETVAL    (0x00FFFFFFu)

/* rio_host_base_id_lock */

#define CSL_SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_HOST_BASE_ID_LOCK_HOST_BASE_ID_RESETVAL (0x0000FFFFu)

#define CSL_SRIO_RIO_HOST_BASE_ID_LOCK_RESETVAL (0x0000FFFFu)

/* rio_comp_tag */

#define CSL_SRIO_RIO_COMP_TAG_CTAG_MASK  (0xFFFFFFFFu)
#define CSL_SRIO_RIO_COMP_TAG_CTAG_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_COMP_TAG_CTAG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_COMP_TAG_RESETVAL   (0x00000000u)

/* rio_sp_mb_head */

#define CSL_SRIO_RIO_SP_MB_HEAD_EF_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_SP_MB_HEAD_EF_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_MB_HEAD_EF_PTR_RESETVAL (0x00001000u)

#define CSL_SRIO_RIO_SP_MB_HEAD_EF_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_SP_MB_HEAD_EF_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_SP_MB_HEAD_EF_ID_RESETVAL (0x00000002u)

#define CSL_SRIO_RIO_SP_MB_HEAD_RESETVAL (0x10000002u)

/* rio_sp_lt_ctl */

#define CSL_SRIO_RIO_SP_LT_CTL_TVAL_MASK (0xFFFFFF00u)
#define CSL_SRIO_RIO_SP_LT_CTL_TVAL_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SP_LT_CTL_TVAL_RESETVAL (0x00FFFFFFu)

#define CSL_SRIO_RIO_SP_LT_CTL_RESETVAL  (0xFFFFFF00u)

/* rio_sp_rt_ctl */

#define CSL_SRIO_RIO_SP_RT_CTL_TVAL_MASK (0xFFFFFF00u)
#define CSL_SRIO_RIO_SP_RT_CTL_TVAL_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_SP_RT_CTL_TVAL_RESETVAL (0x00FFFFFFu)

#define CSL_SRIO_RIO_SP_RT_CTL_RESETVAL  (0xFFFFFF00u)

/* rio_sp_gen_ctl */

#define CSL_SRIO_RIO_SP_GEN_CTL_HOST_MASK (0x80000000u)
#define CSL_SRIO_RIO_SP_GEN_CTL_HOST_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_SP_GEN_CTL_HOST_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_GEN_CTL_MAST_EN_MASK (0x40000000u)
#define CSL_SRIO_RIO_SP_GEN_CTL_MAST_EN_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_SP_GEN_CTL_MAST_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_GEN_CTL_DISC_MASK (0x20000000u)
#define CSL_SRIO_RIO_SP_GEN_CTL_DISC_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_SP_GEN_CTL_DISC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_GEN_CTL_RESETVAL (0x00000000u)

/* rio_err_rpt_bh */

#define CSL_SRIO_RIO_ERR_RPT_BH_EF_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_ERR_RPT_BH_EF_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_ERR_RPT_BH_EF_PTR_RESETVAL (0x00003000u)

#define CSL_SRIO_RIO_ERR_RPT_BH_EF_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_ERR_RPT_BH_EF_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ERR_RPT_BH_EF_ID_RESETVAL (0x00000007u)

#define CSL_SRIO_RIO_ERR_RPT_BH_RESETVAL (0x30000007u)

/* rio_err_det */

#define CSL_SRIO_RIO_ERR_DET_IO_ERR_RESP_MASK (0x80000000u)
#define CSL_SRIO_RIO_ERR_DET_IO_ERR_RESP_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_ERR_DET_IO_ERR_RESP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_MSG_ERR_RESP_MASK (0x40000000u)
#define CSL_SRIO_RIO_ERR_DET_MSG_ERR_RESP_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_ERR_DET_MSG_ERR_RESP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_GSM_ERR_RESP_MASK (0x20000000u)
#define CSL_SRIO_RIO_ERR_DET_GSM_ERR_RESP_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_ERR_DET_GSM_ERR_RESP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_MSG_FMT_ERR_MASK (0x10000000u)
#define CSL_SRIO_RIO_ERR_DET_MSG_FMT_ERR_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_ERR_DET_MSG_FMT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_ILL_TRANS_DECODE_MASK (0x08000000u)
#define CSL_SRIO_RIO_ERR_DET_ILL_TRANS_DECODE_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_ERR_DET_ILL_TRANS_DECODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_ILL_TRANS_TGT_ERR_MASK (0x04000000u)
#define CSL_SRIO_RIO_ERR_DET_ILL_TRANS_TGT_ERR_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_ERR_DET_ILL_TRANS_TGT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_MSG_REQ_TIMEOUT_MASK (0x02000000u)
#define CSL_SRIO_RIO_ERR_DET_MSG_REQ_TIMEOUT_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_ERR_DET_MSG_REQ_TIMEOUT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_PKT_RESP_TIMEOUT_MASK (0x01000000u)
#define CSL_SRIO_RIO_ERR_DET_PKT_RESP_TIMEOUT_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_ERR_DET_PKT_RESP_TIMEOUT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_UNSOLICITED_RESP_MASK (0x00800000u)
#define CSL_SRIO_RIO_ERR_DET_UNSOLICITED_RESP_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_ERR_DET_UNSOLICITED_RESP_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_UNSUPPORTED_TRANS_MASK (0x00400000u)
#define CSL_SRIO_RIO_ERR_DET_UNSUPPORTED_TRANS_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_ERR_DET_UNSUPPORTED_TRANS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_PDU_LEN_ERR_MASK (0x00004000u)
#define CSL_SRIO_RIO_ERR_DET_PDU_LEN_ERR_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_ERR_DET_PDU_LEN_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_SHORT_STREAM_SEG_MASK (0x00002000u)
#define CSL_SRIO_RIO_ERR_DET_SHORT_STREAM_SEG_SHIFT (0x0000000Du)
#define CSL_SRIO_RIO_ERR_DET_SHORT_STREAM_SEG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_LONG_STREAM_SEG_MASK (0x00001000u)
#define CSL_SRIO_RIO_ERR_DET_LONG_STREAM_SEG_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_ERR_DET_LONG_STREAM_SEG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_OPEN_STREAM_CONTEXT_MASK (0x00000800u)
#define CSL_SRIO_RIO_ERR_DET_OPEN_STREAM_CONTEXT_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_ERR_DET_OPEN_STREAM_CONTEXT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_MISSING_STREAM_CONTEXT_MASK (0x00000400u)
#define CSL_SRIO_RIO_ERR_DET_MISSING_STREAM_CONTEXT_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_ERR_DET_MISSING_STREAM_CONTEXT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_CPPI_SECURITY_VIOLATION_MASK (0x00000080u)
#define CSL_SRIO_RIO_ERR_DET_CPPI_SECURITY_VIOLATION_SHIFT (0x00000007u)
#define CSL_SRIO_RIO_ERR_DET_CPPI_SECURITY_VIOLATION_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_RX_DMA_ERR_MASK (0x00000040u)
#define CSL_SRIO_RIO_ERR_DET_RX_DMA_ERR_SHIFT (0x00000006u)
#define CSL_SRIO_RIO_ERR_DET_RX_DMA_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_DET_RESETVAL    (0x00000000u)

/* rio_err_en */

#define CSL_SRIO_RIO_ERR_EN_IO_ERR_RESP_EN_MASK (0x80000000u)
#define CSL_SRIO_RIO_ERR_EN_IO_ERR_RESP_EN_SHIFT (0x0000001Fu)
#define CSL_SRIO_RIO_ERR_EN_IO_ERR_RESP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_MSG_ERR_RESP_EN_MASK (0x40000000u)
#define CSL_SRIO_RIO_ERR_EN_MSG_ERR_RESP_EN_SHIFT (0x0000001Eu)
#define CSL_SRIO_RIO_ERR_EN_MSG_ERR_RESP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_GSM_ERR_RESP_EN_MASK (0x20000000u)
#define CSL_SRIO_RIO_ERR_EN_GSM_ERR_RESP_EN_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_ERR_EN_GSM_ERR_RESP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_MSG_FMT_ERR_EN_MASK (0x10000000u)
#define CSL_SRIO_RIO_ERR_EN_MSG_FMT_ERR_EN_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_ERR_EN_MSG_FMT_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_ILL_TRANS_DECODE_EN_MASK (0x08000000u)
#define CSL_SRIO_RIO_ERR_EN_ILL_TRANS_DECODE_EN_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_ERR_EN_ILL_TRANS_DECODE_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_ILL_TRANS_TGT_ERR_EN_MASK (0x04000000u)
#define CSL_SRIO_RIO_ERR_EN_ILL_TRANS_TGT_ERR_EN_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_ERR_EN_ILL_TRANS_TGT_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_MSG_REQ_TIMEOUT_EN_MASK (0x02000000u)
#define CSL_SRIO_RIO_ERR_EN_MSG_REQ_TIMEOUT_EN_SHIFT (0x00000019u)
#define CSL_SRIO_RIO_ERR_EN_MSG_REQ_TIMEOUT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_PKT_RESP_TIMEOUT_EN_MASK (0x01000000u)
#define CSL_SRIO_RIO_ERR_EN_PKT_RESP_TIMEOUT_EN_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_ERR_EN_PKT_RESP_TIMEOUT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_UNSOLICITED_RESP_EN_MASK (0x00800000u)
#define CSL_SRIO_RIO_ERR_EN_UNSOLICITED_RESP_EN_SHIFT (0x00000017u)
#define CSL_SRIO_RIO_ERR_EN_UNSOLICITED_RESP_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_UNSUPPORTED_TRANS_EN_MASK (0x00400000u)
#define CSL_SRIO_RIO_ERR_EN_UNSUPPORTED_TRANS_EN_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_ERR_EN_UNSUPPORTED_TRANS_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_PDU_LEN_ERR_EN_MASK (0x00004000u)
#define CSL_SRIO_RIO_ERR_EN_PDU_LEN_ERR_EN_SHIFT (0x0000000Eu)
#define CSL_SRIO_RIO_ERR_EN_PDU_LEN_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_SHORT_STREAM_SEG_EN_MASK (0x00002000u)
#define CSL_SRIO_RIO_ERR_EN_SHORT_STREAM_SEG_EN_SHIFT (0x0000000Du)
#define CSL_SRIO_RIO_ERR_EN_SHORT_STREAM_SEG_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_LONG_STREAM_SEG_EN_MASK (0x00001000u)
#define CSL_SRIO_RIO_ERR_EN_LONG_STREAM_SEG_EN_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_ERR_EN_LONG_STREAM_SEG_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_OPEN_STREAM_CONTEXT_EN_MASK (0x00000800u)
#define CSL_SRIO_RIO_ERR_EN_OPEN_STREAM_CONTEXT_EN_SHIFT (0x0000000Bu)
#define CSL_SRIO_RIO_ERR_EN_OPEN_STREAM_CONTEXT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_MISSING_STREAM_CONTEXT_EN_MASK (0x00000400u)
#define CSL_SRIO_RIO_ERR_EN_MISSING_STREAM_CONTEXT_EN_SHIFT (0x0000000Au)
#define CSL_SRIO_RIO_ERR_EN_MISSING_STREAM_CONTEXT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_CPPI_SECURITY_VIOLATION_EN_MASK (0x00000080u)
#define CSL_SRIO_RIO_ERR_EN_CPPI_SECURITY_VIOLATION_EN_SHIFT (0x00000007u)
#define CSL_SRIO_RIO_ERR_EN_CPPI_SECURITY_VIOLATION_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_RX_DMA_ERR_EN_MASK (0x00000040u)
#define CSL_SRIO_RIO_ERR_EN_RX_DMA_ERR_EN_SHIFT (0x00000006u)
#define CSL_SRIO_RIO_ERR_EN_RX_DMA_ERR_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ERR_EN_RESETVAL     (0x00000000u)

/* rio_h_addr_capt */

#define CSL_SRIO_RIO_H_ADDR_CAPT_ADDR_HIGH_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_H_ADDR_CAPT_ADDR_HIGH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_H_ADDR_CAPT_ADDR_HIGH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_H_ADDR_CAPT_RESETVAL (0x00000000u)

/* rio_addr_capt */

#define CSL_SRIO_RIO_ADDR_CAPT_ADDR_LOW_MASK (0xFFFFFFF8u)
#define CSL_SRIO_RIO_ADDR_CAPT_ADDR_LOW_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_ADDR_CAPT_ADDR_LOW_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ADDR_CAPT_XAMSBS_MASK (0x00000003u)
#define CSL_SRIO_RIO_ADDR_CAPT_XAMSBS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ADDR_CAPT_XAMSBS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ADDR_CAPT_RESETVAL  (0x00000000u)

/* rio_id_capt */

#define CSL_SRIO_RIO_ID_CAPT_MSB_DSTID_MASK (0xFF000000u)
#define CSL_SRIO_RIO_ID_CAPT_MSB_DSTID_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_ID_CAPT_MSB_DSTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ID_CAPT_DSTID_MASK  (0x00FF0000u)
#define CSL_SRIO_RIO_ID_CAPT_DSTID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_ID_CAPT_DSTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ID_CAPT_MSB_SRCTID_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_ID_CAPT_MSB_SRCTID_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_ID_CAPT_MSB_SRCTID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ID_CAPT_SRCID_MASK  (0x000000FFu)
#define CSL_SRIO_RIO_ID_CAPT_SRCID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_ID_CAPT_SRCID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_ID_CAPT_RESETVAL    (0x00000000u)

/* rio_ctrl_capt */

#define CSL_SRIO_RIO_CTRL_CAPT_FTYPE_MASK (0xF0000000u)
#define CSL_SRIO_RIO_CTRL_CAPT_FTYPE_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_CTRL_CAPT_FTYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_CTRL_CAPT_TTYPE_MASK (0x0F000000u)
#define CSL_SRIO_RIO_CTRL_CAPT_TTYPE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_CTRL_CAPT_TTYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_CTRL_CAPT_MSG_INFO_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_CTRL_CAPT_MSG_INFO_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_CTRL_CAPT_MSG_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_CTRL_CAPT_IMP_SPECIFIC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_CTRL_CAPT_RESETVAL  (0x00000000u)

/* rio_pw_tgt_id */

#define CSL_SRIO_RIO_PW_TGT_ID_DEVICEID_MSB_MASK (0xFF000000u)
#define CSL_SRIO_RIO_PW_TGT_ID_DEVICEID_MSB_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PW_TGT_ID_DEVICEID_MSB_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_TGT_ID_DEVICEID_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_PW_TGT_ID_DEVICEID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PW_TGT_ID_DEVICEID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_TGT_ID_ID_LARGE_MASK (0x00008000u)
#define CSL_SRIO_RIO_PW_TGT_ID_ID_LARGE_SHIFT (0x0000000Fu)
#define CSL_SRIO_RIO_PW_TGT_ID_ID_LARGE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_TGT_ID_RESETVAL  (0x00000000u)

/* rio_per_lane_bh */

#define CSL_SRIO_RIO_PER_LANE_BH_EF_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_PER_LANE_BH_EF_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PER_LANE_BH_EF_PTR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PER_LANE_BH_EF_ID_MASK (0x0000FFFFu)
#define CSL_SRIO_RIO_PER_LANE_BH_EF_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PER_LANE_BH_EF_ID_RESETVAL (0x0000000Du)

#define CSL_SRIO_RIO_PER_LANE_BH_RESETVAL (0x0000000Du)

/* rio_plm_bh */

#define CSL_SRIO_RIO_PLM_BH_NEXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_PLM_BH_NEXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PLM_BH_NEXT_BLK_PTR_RESETVAL (0x00000103u)

#define CSL_SRIO_RIO_PLM_BH_BLK_REV_MASK (0x0000F000u)
#define CSL_SRIO_RIO_PLM_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PLM_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_PLM_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PLM_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PLM_BH_RESETVAL     (0x01030000u)

/* rio_tlm_bh */

#define CSL_SRIO_RIO_TLM_BH_NXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_TLM_BH_NXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_TLM_BH_NXT_BLK_PTR_RESETVAL (0x00000106u)

#define CSL_SRIO_RIO_TLM_BH_BLK_REV_MASK (0x0000F000u)
#define CSL_SRIO_RIO_TLM_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_TLM_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_TLM_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_TLM_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_TLM_BH_RESETVAL     (0x01060000u)

/* rio_pbm_bh */

#define CSL_SRIO_RIO_PBM_BH_NEXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_PBM_BH_NEXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PBM_BH_NEXT_BLK_PTR_RESETVAL (0x00000109u)

#define CSL_SRIO_RIO_PBM_BH_BLK_REV_MASK (0x0000F000u)
#define CSL_SRIO_RIO_PBM_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PBM_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_PBM_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PBM_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PBM_BH_RESETVAL     (0x01090000u)

/* rio_em_bh */

#define CSL_SRIO_RIO_EM_BH_NEXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_EM_BH_NEXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_EM_BH_NEXT_BLK_PTR_RESETVAL (0x0000010Au)

#define CSL_SRIO_RIO_EM_BH_BLK_REV_MASK  (0x0000F000u)
#define CSL_SRIO_RIO_EM_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_EM_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_EM_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_BH_RESETVAL      (0x010A0000u)

/* rio_em_int_stat */

#define CSL_SRIO_RIO_EM_INT_STAT_PORT_MASK (0x20000000u)
#define CSL_SRIO_RIO_EM_INT_STAT_PORT_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_EM_INT_STAT_PORT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_STAT_LOG_MASK (0x10000000u)
#define CSL_SRIO_RIO_EM_INT_STAT_LOG_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_EM_INT_STAT_LOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_STAT_RCS_MASK (0x08000000u)
#define CSL_SRIO_RIO_EM_INT_STAT_RCS_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_EM_INT_STAT_RCS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_STAT_MECS_MASK (0x04000000u)
#define CSL_SRIO_RIO_EM_INT_STAT_MECS_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_EM_INT_STAT_MECS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_STAT_PW_RX_MASK (0x00010000u)
#define CSL_SRIO_RIO_EM_INT_STAT_PW_RX_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_EM_INT_STAT_PW_RX_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_STAT_LOCALOG_MASK (0x00000100u)
#define CSL_SRIO_RIO_EM_INT_STAT_LOCALOG_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_EM_INT_STAT_LOCALOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_STAT_RESETVAL (0x00000000u)

/* rio_em_int_enable */

#define CSL_SRIO_RIO_EM_INT_ENABLE_LOG_MASK (0x10000000u)
#define CSL_SRIO_RIO_EM_INT_ENABLE_LOG_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_EM_INT_ENABLE_LOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_ENABLE_MECS_MASK (0x04000000u)
#define CSL_SRIO_RIO_EM_INT_ENABLE_MECS_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_EM_INT_ENABLE_MECS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_ENABLE_PW_RX_MASK (0x00010000u)
#define CSL_SRIO_RIO_EM_INT_ENABLE_PW_RX_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_EM_INT_ENABLE_PW_RX_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_ENABLE_LOCALOG_MASK (0x00000100u)
#define CSL_SRIO_RIO_EM_INT_ENABLE_LOCALOG_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_EM_INT_ENABLE_LOCALOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_ENABLE_RESETVAL (0x00000000u)

/* rio_em_int_port_stat */

#define CSL_SRIO_RIO_EM_INT_PORT_STAT_IRQ_PENDING_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_EM_INT_PORT_STAT_IRQ_PENDING_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_INT_PORT_STAT_IRQ_PENDING_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_INT_PORT_STAT_RESETVAL (0x00000000u)

/* rio_em_pw_stat */

#define CSL_SRIO_RIO_EM_PW_STAT_PORT_MASK (0x20000000u)
#define CSL_SRIO_RIO_EM_PW_STAT_PORT_SHIFT (0x0000001Du)
#define CSL_SRIO_RIO_EM_PW_STAT_PORT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_STAT_LOG_MASK (0x10000000u)
#define CSL_SRIO_RIO_EM_PW_STAT_LOG_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_EM_PW_STAT_LOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_STAT_RCS_MASK (0x08000000u)
#define CSL_SRIO_RIO_EM_PW_STAT_RCS_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_EM_PW_STAT_RCS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_STAT_MULTIPORT_ERR_MASK (0x00000200u)
#define CSL_SRIO_RIO_EM_PW_STAT_MULTIPORT_ERR_SHIFT (0x00000009u)
#define CSL_SRIO_RIO_EM_PW_STAT_MULTIPORT_ERR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_STAT_LOCALOG_MASK (0x00000100u)
#define CSL_SRIO_RIO_EM_PW_STAT_LOCALOG_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_EM_PW_STAT_LOCALOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_STAT_RESETVAL (0x00000000u)

/* rio_em_pw_en */

#define CSL_SRIO_RIO_EM_PW_EN_LOG_MASK   (0x10000000u)
#define CSL_SRIO_RIO_EM_PW_EN_LOG_SHIFT  (0x0000001Cu)
#define CSL_SRIO_RIO_EM_PW_EN_LOG_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_EM_PW_EN_LOCALOG_MASK (0x00000100u)
#define CSL_SRIO_RIO_EM_PW_EN_LOCALOG_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_EM_PW_EN_LOCALOG_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_EN_RESETVAL   (0x10000000u)

/* rio_em_pw_port_stat */

#define CSL_SRIO_RIO_EM_PW_PORT_STAT_PW_PENDING_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_EM_PW_PORT_STAT_PW_PENDING_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_PW_PORT_STAT_PW_PENDING_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_PW_PORT_STAT_RESETVAL (0x00000000u)

/* rio_em_dev_int_en */

#define CSL_SRIO_RIO_EM_DEV_INT_EN_INT_EN_MASK (0x00000001u)
#define CSL_SRIO_RIO_EM_DEV_INT_EN_INT_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_DEV_INT_EN_INT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_DEV_INT_EN_RESETVAL (0x00000000u)

/* rio_em_dev_pw_en */

#define CSL_SRIO_RIO_EM_DEV_PW_EN_PW_EN_MASK (0x00000001u)
#define CSL_SRIO_RIO_EM_DEV_PW_EN_PW_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_DEV_PW_EN_PW_EN_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_EM_DEV_PW_EN_RESETVAL (0x00000001u)

/* rio_em_mecs_stat */

#define CSL_SRIO_RIO_EM_MECS_STAT_CMD_STAT_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_MECS_STAT_CMD_STAT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_STAT_CMD_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_STAT_RESETVAL (0x00000000u)

/* rio_em_mecs_int_en */

#define CSL_SRIO_RIO_EM_MECS_INT_EN_CMD_EN_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_MECS_INT_EN_CMD_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_INT_EN_CMD_EN_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_EM_MECS_INT_EN_RESETVAL (0x00000001u)

/* rio_em_mecs_cap_en */

#define CSL_SRIO_RIO_EM_MECS_CAP_EN_CMD_EN_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_MECS_CAP_EN_CMD_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_CAP_EN_CMD_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_CAP_EN_RESETVAL (0x00000000u)

/* rio_em_mecs_trig_en */

#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_CMD_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_CMD_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_TRIG_EN_RESETVAL (0x00000000u)

/* rio_em_mecs_req */

#define CSL_SRIO_RIO_EM_MECS_REQ_SEND_MASK (0x00000100u)
#define CSL_SRIO_RIO_EM_MECS_REQ_SEND_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_EM_MECS_REQ_SEND_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_REQ_CMD_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_MECS_REQ_CMD_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_REQ_CMD_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_REQ_RESETVAL (0x00000000u)

/* rio_em_mecs_port_stat */

#define CSL_SRIO_RIO_EM_MECS_PORT_STAT_PORT_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_EM_MECS_PORT_STAT_PORT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_PORT_STAT_PORT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_PORT_STAT_RESETVAL (0x00000000u)

/* rio_em_mecs_event_gen */

#define CSL_SRIO_RIO_EM_MECS_EVENT_GEN_CMD_STAT_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_MECS_EVENT_GEN_CMD_STAT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_MECS_EVENT_GEN_CMD_STAT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_MECS_EVENT_GEN_RESETVAL (0x00000000u)

/* rio_em_rst_port_stat */

#define CSL_SRIO_RIO_EM_RST_PORT_STAT_RST_REQ_MASK (0x000000FFu)
#define CSL_SRIO_RIO_EM_RST_PORT_STAT_RST_REQ_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_RST_PORT_STAT_RST_REQ_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_RST_PORT_STAT_RESETVAL (0x00000000u)

/* rio_em_rst_int_en */

#define CSL_SRIO_RIO_EM_RST_INT_EN_RST_INT_EN_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_EM_RST_INT_EN_RST_INT_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_RST_INT_EN_RST_INT_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_RST_INT_EN_RESETVAL (0x00000000u)

/* rio_em_rst_pw_en */

#define CSL_SRIO_RIO_EM_RST_PW_EN_RST_PW_EN_MASK (0x0000000Fu)
#define CSL_SRIO_RIO_EM_RST_PW_EN_RST_PW_EN_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_EM_RST_PW_EN_RST_PW_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_EM_RST_PW_EN_RESETVAL (0x00000000u)

/* rio_pw_bh */

#define CSL_SRIO_RIO_PW_BH_NEXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_PW_BH_NEXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_PW_BH_NEXT_BLK_PTR_RESETVAL (0x0000010Du)

#define CSL_SRIO_RIO_PW_BH_BLK_REV_MASK  (0x0000F000u)
#define CSL_SRIO_RIO_PW_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PW_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_PW_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PW_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_BH_RESETVAL      (0x010D0000u)

/* rio_pw_ctl */

#define CSL_SRIO_RIO_PW_CTL_PW_TIMER_MASK (0xF0000000u)
#define CSL_SRIO_RIO_PW_CTL_PW_TIMER_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_PW_CTL_PW_TIMER_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_CTL_PWC_MODE_MASK (0x01000000u)
#define CSL_SRIO_RIO_PW_CTL_PWC_MODE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_PW_CTL_PWC_MODE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_CTL_RESETVAL     (0x00000000u)

/* rio_pw_route */

#define CSL_SRIO_RIO_PW_ROUTE_PORT_MASK  (0x0000000Fu)
#define CSL_SRIO_RIO_PW_ROUTE_PORT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PW_ROUTE_PORT_RESETVAL (0x00000001u)

#define CSL_SRIO_RIO_PW_ROUTE_RESETVAL   (0x00000001u)

/* rio_pw_rx_stat */

#define CSL_SRIO_RIO_PW_RX_STAT_WR_SIZE_MASK (0x0000F000u)
#define CSL_SRIO_RIO_PW_RX_STAT_WR_SIZE_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_PW_RX_STAT_WR_SIZE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_STAT_WDPTR_MASK (0x00000100u)
#define CSL_SRIO_RIO_PW_RX_STAT_WDPTR_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PW_RX_STAT_WDPTR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_STAT_PW_SHORT_MASK (0x00000008u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_SHORT_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_SHORT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_STAT_PW_TRUNC_MASK (0x00000004u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_TRUNC_SHIFT (0x00000002u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_TRUNC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_STAT_PW_DISC_MASK (0x00000002u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_DISC_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_DISC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_STAT_PW_VAL_MASK (0x00000001u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_VAL_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PW_RX_STAT_PW_VAL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_STAT_RESETVAL (0x00000000u)

/* rio_pw_rx_event_gen */

#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC_MASK (0x00000002u)
#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC_SHIFT (0x00000001u)
#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_PW_DISC_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL_MASK (0x00000001u)
#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_PW_VAL_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_EVENT_GEN_RESETVAL (0x00000000u)

/* rio_pw_rx_capt */

#define CSL_SRIO_RIO_PW_RX_CAPT_PW_CAPT_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_PW_RX_CAPT_PW_CAPT_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PW_RX_CAPT_PW_CAPT_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PW_RX_CAPT_RESETVAL (0x00000000u)

/* rio_llm_bh */

#define CSL_SRIO_RIO_LLM_BH_NEXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_LLM_BH_NEXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LLM_BH_NEXT_BLK_PTR_RESETVAL (0x0000010Eu)

#define CSL_SRIO_RIO_LLM_BH_BLK_REV_MASK (0x0000F000u)
#define CSL_SRIO_RIO_LLM_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_LLM_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LLM_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_LLM_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LLM_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LLM_BH_RESETVAL     (0x010E0000u)

/* rio_whiteboard */

#define CSL_SRIO_RIO_WHITEBOARD_SCRATCH_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_WHITEBOARD_SCRATCH_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_WHITEBOARD_SCRATCH_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_WHITEBOARD_RESETVAL (0x00000000u)

/* rio_port_number */

#define CSL_SRIO_RIO_PORT_NUMBER_PORT_TOTAL_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_PORT_NUMBER_PORT_TOTAL_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_PORT_NUMBER_PORT_TOTAL_RESETVAL (0x00000004u)

#define CSL_SRIO_RIO_PORT_NUMBER_PORT_NUM_MASK (0x000000FFu)
#define CSL_SRIO_RIO_PORT_NUMBER_PORT_NUM_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PORT_NUMBER_PORT_NUM_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_PORT_NUMBER_RESETVAL (0x00000400u)

/* rio_prescalar_srv_clk */

#define CSL_SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK_MASK (0x000000FFu)
#define CSL_SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_PRESCALAR_SRV_CLK_PRESCALAR_SRV_CLK_RESETVAL (0x0000001Fu)

#define CSL_SRIO_RIO_PRESCALAR_SRV_CLK_RESETVAL (0x0000001Fu)

/* rio_reg_rst_ctl */

#define CSL_SRIO_RIO_REG_RST_CTL_CLEAR_STICKY_MASK (0x00000001u)
#define CSL_SRIO_RIO_REG_RST_CTL_CLEAR_STICKY_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_REG_RST_CTL_CLEAR_STICKY_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_REG_RST_CTL_RESETVAL (0x00000000u)

/* rio_local_err_det */

#define CSL_SRIO_RIO_LOCAL_ERR_DET_ILL_ID_MASK (0x04000000u)
#define CSL_SRIO_RIO_LOCAL_ERR_DET_ILL_ID_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_LOCAL_ERR_DET_ILL_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE_MASK (0x00400000u)
#define CSL_SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_LOCAL_ERR_DET_ILL_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ERR_DET_RESETVAL (0x00000000u)

/* rio_local_err_en */

#define CSL_SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN_MASK (0x04000000u)
#define CSL_SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_LOCAL_ERR_EN_ILL_ID_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN_MASK (0x00400000u)
#define CSL_SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN_SHIFT (0x00000016u)
#define CSL_SRIO_RIO_LOCAL_ERR_EN_ILL_TYPE_EN_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ERR_EN_RESETVAL (0x00000000u)

/* rio_local_h_addr_capt */

#define CSL_SRIO_RIO_LOCAL_H_ADDR_CAPT_ADDR_MASK (0xFFFFFFFFu)
#define CSL_SRIO_RIO_LOCAL_H_ADDR_CAPT_ADDR_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LOCAL_H_ADDR_CAPT_ADDR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_H_ADDR_CAPT_RESETVAL (0x00000000u)

/* rio_local_addr_capt */

#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_ADDR_MASK (0xFFFFFFF8u)
#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_ADDR_SHIFT (0x00000003u)
#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_ADDR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_XAMSBS_MASK (0x00000003u)
#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_XAMSBS_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_XAMSBS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ADDR_CAPT_RESETVAL (0x00000000u)

/* rio_local_id_capt */

#define CSL_SRIO_RIO_LOCAL_ID_CAPT_MSB_DEST_ID_MASK (0xFF000000u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_MSB_DEST_ID_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_MSB_DEST_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ID_CAPT_DEST_ID_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_DEST_ID_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_DEST_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ID_CAPT_MSB_SRC_ID_MASK (0x0000FF00u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_MSB_SRC_ID_SHIFT (0x00000008u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_MSB_SRC_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ID_CAPT_SRC_ID_MASK (0x000000FFu)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_SRC_ID_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_LOCAL_ID_CAPT_SRC_ID_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_ID_CAPT_RESETVAL (0x00000000u)

/* rio_local_ctrl_capt */

#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_FTYPE_MASK (0xF0000000u)
#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_FTYPE_SHIFT (0x0000001Cu)
#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_FTYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_TTYPE_MASK (0x0F000000u)
#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_TTYPE_SHIFT (0x00000018u)
#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_TTYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_MESSAGE_INFO_MASK (0x00FF0000u)
#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_MESSAGE_INFO_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_MESSAGE_INFO_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_LOCAL_CTRL_CAPT_RESETVAL (0x00000000u)

/* rio_fabric_bh */

#define CSL_SRIO_RIO_FABRIC_BH_NEXT_BLK_PTR_MASK (0xFFFF0000u)
#define CSL_SRIO_RIO_FABRIC_BH_NEXT_BLK_PTR_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_FABRIC_BH_NEXT_BLK_PTR_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_FABRIC_BH_BLK_REV_MASK (0x0000F000u)
#define CSL_SRIO_RIO_FABRIC_BH_BLK_REV_SHIFT (0x0000000Cu)
#define CSL_SRIO_RIO_FABRIC_BH_BLK_REV_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_FABRIC_BH_BLK_TYPE_MASK (0x00000FFFu)
#define CSL_SRIO_RIO_FABRIC_BH_BLK_TYPE_SHIFT (0x00000000u)
#define CSL_SRIO_RIO_FABRIC_BH_BLK_TYPE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_FABRIC_BH_RESETVAL  (0x00000000u)

/* rio_fabric_csr */

#define CSL_SRIO_RIO_FABRIC_CSR_IG_LLM_BACKPRESSURE_MASK (0x08000000u)
#define CSL_SRIO_RIO_FABRIC_CSR_IG_LLM_BACKPRESSURE_SHIFT (0x0000001Bu)
#define CSL_SRIO_RIO_FABRIC_CSR_IG_LLM_BACKPRESSURE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_FABRIC_CSR_IG_UC_BACKPRESSURE_MASK (0x04000000u)
#define CSL_SRIO_RIO_FABRIC_CSR_IG_UC_BACKPRESSURE_SHIFT (0x0000001Au)
#define CSL_SRIO_RIO_FABRIC_CSR_IG_UC_BACKPRESSURE_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_FABRIC_CSR_RESETVAL (0x00000000u)

/* rio_sp_fabric_status */

#define CSL_SRIO_RIO_SP_FABRIC_STATUS_IG_PKT_ENABLE_STATUS_MASK (0x00F00000u)
#define CSL_SRIO_RIO_SP_FABRIC_STATUS_IG_PKT_ENABLE_STATUS_SHIFT (0x00000014u)
#define CSL_SRIO_RIO_SP_FABRIC_STATUS_IG_PKT_ENABLE_STATUS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_FABRIC_STATUS_EG_PKT_ENABLE_STATUS_MASK (0x000F0000u)
#define CSL_SRIO_RIO_SP_FABRIC_STATUS_EG_PKT_ENABLE_STATUS_SHIFT (0x00000010u)
#define CSL_SRIO_RIO_SP_FABRIC_STATUS_EG_PKT_ENABLE_STATUS_RESETVAL (0x00000000u)

#define CSL_SRIO_RIO_SP_FABRIC_STATUS_RESETVAL (0x00000000u)

#endif
