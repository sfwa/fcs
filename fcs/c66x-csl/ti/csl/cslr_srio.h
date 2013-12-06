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

#endif
