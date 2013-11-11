/*
 * cslr_uart.h
 *
 * This file contains the macros for Register Chip Support Library (CSL) which 
 * can be used for operations on the respective underlying hardware/peripheral
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
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
 *   @file  cslr_uart.h
 *
 *   @path  $(CSLPATH)
 *
 *   @desc  This file contains the Register Descriptions for UART
 *
 *  ============================================================================
 */
#ifndef _CSLR_UART_H_
#define _CSLR_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>




/* Minimum unit = 4 bytes */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 RBR;
    volatile Uint32 IER;
    volatile Uint32 IIR;
    volatile Uint32 LCR;
    volatile Uint32 MCR;
    volatile Uint32 LSR;
    volatile Uint32 MSR;
    volatile Uint32 SCR;
    volatile Uint32 DLL;
    volatile Uint32 DLH;
    volatile Uint32 REVID1;
    volatile Uint32 REVID2;
    volatile Uint32 PWREMU_MGMT;
    volatile Uint32 MDR;
} CSL_UartRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_UartRegs             *CSL_UartRegsOvly;

/* Following 2 lines are added due to CSL3.x tools limitations */
#define THR RBR   /* RBR & THR have same offset */
#define FCR IIR   /* IIR & FCR have same offset */

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* RBR */


#define CSL_UART_RBR_DATA_MASK           (0x000000FFu)
#define CSL_UART_RBR_DATA_SHIFT          (0x00000000u)
#define CSL_UART_RBR_DATA_RESETVAL       (0x00000000u)

#define CSL_UART_RBR_RESETVAL            (0x00000000u)

/* THR */


#define CSL_UART_THR_DATA_MASK           (0x000000FFu)
#define CSL_UART_THR_DATA_SHIFT          (0x00000000u)
#define CSL_UART_THR_DATA_RESETVAL       (0x00000000u)

#define CSL_UART_THR_RESETVAL            (0x00000000u)

/* IER */


#define CSL_UART_IER_EDSSI_MASK          (0x00000008u)
#define CSL_UART_IER_EDSSI_SHIFT         (0x00000003u)
#define CSL_UART_IER_EDSSI_RESETVAL      (0x00000000u)
/*----EDSSI Tokens----*/
#define CSL_UART_IER_EDSSI_DISABLE       (0x00000000u)
#define CSL_UART_IER_EDSSI_ENABLE        (0x00000001u)

#define CSL_UART_IER_ELSI_MASK           (0x00000004u)
#define CSL_UART_IER_ELSI_SHIFT          (0x00000002u)
#define CSL_UART_IER_ELSI_RESETVAL       (0x00000000u)
/*----ELSI Tokens----*/
#define CSL_UART_IER_ELSI_DISABLE        (0x00000000u)
#define CSL_UART_IER_ELSI_ENABLE         (0x00000001u)

#define CSL_UART_IER_ETBEI_MASK          (0x00000002u)
#define CSL_UART_IER_ETBEI_SHIFT         (0x00000001u)
#define CSL_UART_IER_ETBEI_RESETVAL      (0x00000000u)
/*----ETBEI Tokens----*/
#define CSL_UART_IER_ETBEI_DISABLE       (0x00000000u)
#define CSL_UART_IER_ETBEI_ENABLE        (0x00000001u)

#define CSL_UART_IER_ERBI_MASK           (0x00000001u)
#define CSL_UART_IER_ERBI_SHIFT          (0x00000000u)
#define CSL_UART_IER_ERBI_RESETVAL       (0x00000000u)
/*----ERBI Tokens----*/
#define CSL_UART_IER_ERBI_DISABLE        (0x00000000u)
#define CSL_UART_IER_ERBI_ENABLE         (0x00000001u)

#define CSL_UART_IER_RESETVAL            (0x00000000u)

/* IIR */


#define CSL_UART_IIR_FIFOEN_MASK         (0x000000C0u)
#define CSL_UART_IIR_FIFOEN_SHIFT        (0x00000006u)
#define CSL_UART_IIR_FIFOEN_RESETVAL     (0x00000000u)
/*----FIFOEN Tokens----*/
#define CSL_UART_IIR_FIFOEN_DISABLE      (0x00000000u)
#define CSL_UART_IIR_FIFOEN_ENABLE       (0x00000003u)


#define CSL_UART_IIR_INTID_MASK          (0x0000000Eu)
#define CSL_UART_IIR_INTID_SHIFT         (0x00000001u)
#define CSL_UART_IIR_INTID_RESETVAL      (0x00000000u)
/*----INTID Tokens----*/
#define CSL_UART_IIR_INTID_MODSTAT       (0x00000000u)
#define CSL_UART_IIR_INTID_THRE          (0x00000001u)
#define CSL_UART_IIR_INTID_RDA           (0x00000002u)
#define CSL_UART_IIR_INTID_RLS           (0x00000003u)
#define CSL_UART_IIR_INTID_CTI           (0x00000006u)

#define CSL_UART_IIR_IPEND_MASK          (0x00000001u)
#define CSL_UART_IIR_IPEND_SHIFT         (0x00000000u)
#define CSL_UART_IIR_IPEND_RESETVAL      (0x00000001u)
/*----IPEND Tokens----*/
#define CSL_UART_IIR_IPEND_PEND          (0x00000000u)
#define CSL_UART_IIR_IPEND_NO_PEND       (0x00000001u)

#define CSL_UART_IIR_RESETVAL            (0x00000001u)

/* FCR */


#define CSL_UART_FCR_RXFIFTL_MASK        (0x000000C0u)
#define CSL_UART_FCR_RXFIFTL_SHIFT       (0x00000006u)
#define CSL_UART_FCR_RXFIFTL_RESETVAL    (0x00000000u)
/*----RXFIFTL Tokens----*/
#define CSL_UART_FCR_RXFIFTL_CHAR1       (0x00000000u)
#define CSL_UART_FCR_RXFIFTL_CHAR4       (0x00000001u)
#define CSL_UART_FCR_RXFIFTL_CHAR8       (0x00000002u)
#define CSL_UART_FCR_RXFIFTL_CHAR14      (0x00000003u)


#define CSL_UART_FCR_DMAMODE1_MASK       (0x00000008u)
#define CSL_UART_FCR_DMAMODE1_SHIFT      (0x00000003u)
#define CSL_UART_FCR_DMAMODE1_RESETVAL   (0x00000000u)
/*----DMAMODE1 Tokens----*/
#define CSL_UART_FCR_DMAMODE1_DISABLE    (0x00000000u)
#define CSL_UART_FCR_DMAMODE1_ENABLE     (0x00000001u)

#define CSL_UART_FCR_TXCLR_MASK          (0x00000004u)
#define CSL_UART_FCR_TXCLR_SHIFT         (0x00000002u)
#define CSL_UART_FCR_TXCLR_RESETVAL      (0x00000000u)
/*----TXCLR Tokens----*/
#define CSL_UART_FCR_TXCLR_CLR           (0x00000001u)

#define CSL_UART_FCR_RXCLR_MASK          (0x00000002u)
#define CSL_UART_FCR_RXCLR_SHIFT         (0x00000001u)
#define CSL_UART_FCR_RXCLR_RESETVAL      (0x00000000u)
/*----RXCLR Tokens----*/
#define CSL_UART_FCR_RXCLR_CLR           (0x00000001u)

#define CSL_UART_FCR_FIFOEN_MASK         (0x00000001u)
#define CSL_UART_FCR_FIFOEN_SHIFT        (0x00000000u)
#define CSL_UART_FCR_FIFOEN_RESETVAL     (0x00000000u)
/*----FIFOEN Tokens----*/
#define CSL_UART_FCR_FIFOEN_DISABLE      (0x00000000u)
#define CSL_UART_FCR_FIFOEN_ENABLE       (0x00000001u)

#define CSL_UART_FCR_RESETVAL            (0x00000000u)

/* LCR */


#define CSL_UART_LCR_DLAB_MASK           (0x00000080u)
#define CSL_UART_LCR_DLAB_SHIFT          (0x00000007u)
#define CSL_UART_LCR_DLAB_RESETVAL       (0x00000000u)
/*----DLAB Tokens----*/
#define CSL_UART_LCR_DLAB_DISABLE        (0x00000000u)
#define CSL_UART_LCR_DLAB_ENABLE         (0x00000001u)

#define CSL_UART_LCR_BC_MASK             (0x00000040u)
#define CSL_UART_LCR_BC_SHIFT            (0x00000006u)
#define CSL_UART_LCR_BC_RESETVAL         (0x00000000u)
/*----BC Tokens----*/
#define CSL_UART_LCR_BC_DISABLE          (0x00000000u)
#define CSL_UART_LCR_BC_ENABLE           (0x00000001u)

#define CSL_UART_LCR_SP_MASK             (0x00000020u)
#define CSL_UART_LCR_SP_SHIFT            (0x00000005u)
#define CSL_UART_LCR_SP_RESETVAL         (0x00000000u)
/*----SP Tokens----*/
#define CSL_UART_LCR_SP_DISABLE          (0x00000000u)
#define CSL_UART_LCR_SP_ENABLE           (0x00000001u)

#define CSL_UART_LCR_EPS_MASK            (0x00000010u)
#define CSL_UART_LCR_EPS_SHIFT           (0x00000004u)
#define CSL_UART_LCR_EPS_RESETVAL        (0x00000000u)
/*----EPS Tokens----*/
#define CSL_UART_LCR_EPS_ODD             (0x00000000u)
#define CSL_UART_LCR_EPS_EVEN            (0x00000001u)

#define CSL_UART_LCR_PEN_MASK            (0x00000008u)
#define CSL_UART_LCR_PEN_SHIFT           (0x00000003u)
#define CSL_UART_LCR_PEN_RESETVAL        (0x00000000u)
/*----PEN Tokens----*/
#define CSL_UART_LCR_PEN_DISABLE         (0x00000000u)
#define CSL_UART_LCR_PEN_ENABLE          (0x00000001u)

#define CSL_UART_LCR_STB_MASK            (0x00000004u)
#define CSL_UART_LCR_STB_SHIFT           (0x00000002u)
#define CSL_UART_LCR_STB_RESETVAL        (0x00000000u)
/*----STB Tokens----*/
#define CSL_UART_LCR_STB_1BIT            (0x00000000u)
#define CSL_UART_LCR_STB_WLS             (0x00000001u)

#define CSL_UART_LCR_WLS_MASK            (0x00000003u)
#define CSL_UART_LCR_WLS_SHIFT           (0x00000000u)
#define CSL_UART_LCR_WLS_RESETVAL        (0x00000000u)
/*----WLS Tokens----*/
#define CSL_UART_LCR_WLS_5BITS           (0x00000000u)
#define CSL_UART_LCR_WLS_6BITS           (0x00000001u)
#define CSL_UART_LCR_WLS_7BITS           (0x00000002u)
#define CSL_UART_LCR_WLS_8BITS           (0x00000003u)

#define CSL_UART_LCR_RESETVAL            (0x00000000u)

/* MCR */


#define CSL_UART_MCR_AFE_MASK            (0x00000020u)
#define CSL_UART_MCR_AFE_SHIFT           (0x00000005u)
#define CSL_UART_MCR_AFE_RESETVAL        (0x00000000u)
/*----AFE Tokens----*/
#define CSL_UART_MCR_AFE_DISABLE         (0x00000000u)
#define CSL_UART_MCR_AFE_ENABLE          (0x00000001u)

#define CSL_UART_MCR_LOOP_MASK           (0x00000010u)
#define CSL_UART_MCR_LOOP_SHIFT          (0x00000004u)
#define CSL_UART_MCR_LOOP_RESETVAL       (0x00000000u)
/*----LOOP Tokens----*/
#define CSL_UART_MCR_LOOP_DISABLE        (0x00000000u)
#define CSL_UART_MCR_LOOP_ENABLE         (0x00000001u)

#define CSL_UART_MCR_OUT2_MASK           (0x00000008u)
#define CSL_UART_MCR_OUT2_SHIFT          (0x00000003u)
#define CSL_UART_MCR_OUT2_RESETVAL       (0x00000000u)
/*----OUT2 Tokens----*/
#define CSL_UART_MCR_OUT2_DISABLE        (0x00000000u)
#define CSL_UART_MCR_OUT2_ENABLE         (0x00000001u)

#define CSL_UART_MCR_OUT1_MASK           (0x00000004u)
#define CSL_UART_MCR_OUT1_SHIFT          (0x00000002u)
#define CSL_UART_MCR_OUT1_RESETVAL       (0x00000000u)
/*----OUT1 Tokens----*/
#define CSL_UART_MCR_OUT1_DISABLE        (0x00000000u)
#define CSL_UART_MCR_OUT1_ENABLE         (0x00000001u)

#define CSL_UART_MCR_RTS_MASK            (0x00000002u)
#define CSL_UART_MCR_RTS_SHIFT           (0x00000001u)
#define CSL_UART_MCR_RTS_RESETVAL        (0x00000000u)
/*----RTS Tokens----*/
#define CSL_UART_MCR_RTS_HIGH            (0x00000000u)
#define CSL_UART_MCR_RTS_DISABLE         (0x00000000u)
#define CSL_UART_MCR_RTS_ENABLE          (0x00000001u)
#define CSL_UART_MCR_RTS_LOW             (0x00000001u)

#define CSL_UART_MCR_DTR_MASK            (0x00000001u)
#define CSL_UART_MCR_DTR_SHIFT           (0x00000000u)
#define CSL_UART_MCR_DTR_RESETVAL        (0x00000000u)
/*----DTR Tokens----*/
#define CSL_UART_MCR_DTR_DISABLE         (0x00000000u)
#define CSL_UART_MCR_DTR_ENABLE          (0x00000001u)

#define CSL_UART_MCR_RESETVAL            (0x00000000u)

/* LSR */


#define CSL_UART_LSR_RXFIFOE_MASK        (0x00000080u)
#define CSL_UART_LSR_RXFIFOE_SHIFT       (0x00000007u)
#define CSL_UART_LSR_RXFIFOE_RESETVAL    (0x00000000u)
/*----RXFIFOE Tokens----*/
#define CSL_UART_LSR_RXFIFOE_NOERROR     (0x00000000u)
#define CSL_UART_LSR_RXFIFOE_ERROR       (0x00000001u)

#define CSL_UART_LSR_TEMT_MASK           (0x00000040u)
#define CSL_UART_LSR_TEMT_SHIFT          (0x00000006u)
#define CSL_UART_LSR_TEMT_RESETVAL       (0x00000001u)
/*----TEMT Tokens----*/
#define CSL_UART_LSR_TEMT_FULL           (0x00000000u)
#define CSL_UART_LSR_TEMT_EMPTY          (0x00000001u)

#define CSL_UART_LSR_THRE_MASK           (0x00000020u)
#define CSL_UART_LSR_THRE_SHIFT          (0x00000005u)
#define CSL_UART_LSR_THRE_RESETVAL       (0x00000001u)
/*----THRE Tokens----*/
#define CSL_UART_LSR_THRE_FULL           (0x00000000u)
#define CSL_UART_LSR_THRE_EMPTY          (0x00000001u)

#define CSL_UART_LSR_BI_MASK             (0x00000010u)
#define CSL_UART_LSR_BI_SHIFT            (0x00000004u)
#define CSL_UART_LSR_BI_RESETVAL         (0x00000000u)
/*----BI Tokens----*/
#define CSL_UART_LSR_BI_NOBREAK          (0x00000000u)
#define CSL_UART_LSR_BI_BREAK            (0x00000001u)

#define CSL_UART_LSR_FE_MASK             (0x00000008u)
#define CSL_UART_LSR_FE_SHIFT            (0x00000003u)
#define CSL_UART_LSR_FE_RESETVAL         (0x00000000u)
/*----FE Tokens----*/
#define CSL_UART_LSR_FE_NOERROR          (0x00000000u)
#define CSL_UART_LSR_FE_ERROR            (0x00000001u)

#define CSL_UART_LSR_PE_MASK             (0x00000004u)
#define CSL_UART_LSR_PE_SHIFT            (0x00000002u)
#define CSL_UART_LSR_PE_RESETVAL         (0x00000000u)
/*----PE Tokens----*/
#define CSL_UART_LSR_PE_NOERROR          (0x00000000u)
#define CSL_UART_LSR_PE_ERROR            (0x00000001u)

#define CSL_UART_LSR_OE_MASK             (0x00000002u)
#define CSL_UART_LSR_OE_SHIFT            (0x00000001u)
#define CSL_UART_LSR_OE_RESETVAL         (0x00000000u)
/*----OE Tokens----*/
#define CSL_UART_LSR_OE_NOERROR          (0x00000000u)
#define CSL_UART_LSR_OE_ERROR            (0x00000001u)

#define CSL_UART_LSR_DR_MASK             (0x00000001u)
#define CSL_UART_LSR_DR_SHIFT            (0x00000000u)
#define CSL_UART_LSR_DR_RESETVAL         (0x00000000u)
/*----DR Tokens----*/
#define CSL_UART_LSR_DR_NOTREADY         (0x00000000u)
#define CSL_UART_LSR_DR_READY            (0x00000001u)

#define CSL_UART_LSR_RESETVAL            (0x00000060u)

/* MSR */


#define CSL_UART_MSR_CD_MASK             (0x00000080u)
#define CSL_UART_MSR_CD_SHIFT            (0x00000007u)
#define CSL_UART_MSR_CD_RESETVAL         (0x00000000u)
/*----CD Tokens----*/
#define CSL_UART_MSR_CD_NOCD             (0x00000000u)
#define CSL_UART_MSR_CD_CD               (0x00000001u)

#define CSL_UART_MSR_RI_MASK             (0x00000040u)
#define CSL_UART_MSR_RI_SHIFT            (0x00000006u)
#define CSL_UART_MSR_RI_RESETVAL         (0x00000000u)
/*----RI Tokens----*/
#define CSL_UART_MSR_RI_NORI             (0x00000000u)
#define CSL_UART_MSR_RI_RI               (0x00000001u)

#define CSL_UART_MSR_DSR_MASK            (0x00000020u)
#define CSL_UART_MSR_DSR_SHIFT           (0x00000005u)
#define CSL_UART_MSR_DSR_RESETVAL        (0x00000000u)
/*----DSR Tokens----*/
#define CSL_UART_MSR_DSR_NODSR           (0x00000000u)
#define CSL_UART_MSR_DSR_DSR             (0x00000001u)

#define CSL_UART_MSR_CTS_MASK            (0x00000010u)
#define CSL_UART_MSR_CTS_SHIFT           (0x00000004u)
#define CSL_UART_MSR_CTS_RESETVAL        (0x00000000u)
/*----CTS Tokens----*/
#define CSL_UART_MSR_CTS_NOCTS           (0x00000000u)
#define CSL_UART_MSR_CTS_CTS             (0x00000001u)

#define CSL_UART_MSR_DCD_MASK            (0x00000008u)
#define CSL_UART_MSR_DCD_SHIFT           (0x00000003u)
#define CSL_UART_MSR_DCD_RESETVAL        (0x00000000u)
/*----DCD Tokens----*/
#define CSL_UART_MSR_DCD_NODCD           (0x00000000u)
#define CSL_UART_MSR_DCD_DCD             (0x00000001u)

#define CSL_UART_MSR_TERI_MASK           (0x00000004u)
#define CSL_UART_MSR_TERI_SHIFT          (0x00000002u)
#define CSL_UART_MSR_TERI_RESETVAL       (0x00000000u)
/*----TERI Tokens----*/
#define CSL_UART_MSR_TERI_NOTERI         (0x00000000u)
#define CSL_UART_MSR_TERI_TERI           (0x00000001u)

#define CSL_UART_MSR_DDSR_MASK           (0x00000002u)
#define CSL_UART_MSR_DDSR_SHIFT          (0x00000001u)
#define CSL_UART_MSR_DDSR_RESETVAL       (0x00000000u)
/*----DDSR Tokens----*/
#define CSL_UART_MSR_DDSR_NODDSR         (0x00000000u)
#define CSL_UART_MSR_DDSR_DDSR           (0x00000001u)

#define CSL_UART_MSR_DCTS_MASK           (0x00000001u)
#define CSL_UART_MSR_DCTS_SHIFT          (0x00000000u)
#define CSL_UART_MSR_DCTS_RESETVAL       (0x00000000u)
/*----DCTS Tokens----*/
#define CSL_UART_MSR_DCTS_NODCTS         (0x00000000u)
#define CSL_UART_MSR_DCTS_DCTS           (0x00000001u)

#define CSL_UART_MSR_RESETVAL            (0x00000000u)

/* SCR */


#define CSL_UART_SCR_SCR_MASK            (0x000000FFu)
#define CSL_UART_SCR_SCR_SHIFT           (0x00000000u)
#define CSL_UART_SCR_SCR_RESETVAL        (0x00000000u)

#define CSL_UART_SCR_RESETVAL            (0x00000000u)

/* DLL */


#define CSL_UART_DLL_DLL_MASK            (0x000000FFu)
#define CSL_UART_DLL_DLL_SHIFT           (0x00000000u)
#define CSL_UART_DLL_DLL_RESETVAL        (0x00000000u)

#define CSL_UART_DLL_RESETVAL            (0x00000000u)

/* DLH */


#define CSL_UART_DLH_DLH_MASK            (0x000000FFu)
#define CSL_UART_DLH_DLH_SHIFT           (0x00000000u)
#define CSL_UART_DLH_DLH_RESETVAL        (0x00000000u)

#define CSL_UART_DLH_RESETVAL            (0x00000000u)

/* PID1 */

#define CSL_UART_REVID1_REV_MASK           (0xFFFFFFFFu)
#define CSL_UART_REVID1_REV_SHIFT          (0x00000000u)
#define CSL_UART_REVID1_REV_RESETVAL       (0x11020002u)

#define CSL_UART_REVID1_RESETVAL           (0x44141102u)

/* PID2 */


#define CSL_UART_REVID2_REV_MASK           (0xFFFFFFFFu)
#define CSL_UART_REVID2_REV_SHIFT          (0x00000000u)
#define CSL_UART_REVID2_REV_RESETVAL       (0x00000000u)

#define CSL_UART_REVID2_RESETVAL           (0x00000000u)

/* PWREMU_MGMT */



#define CSL_UART_PWREMU_MGMT_UTRST_MASK  (0x00004000u)
#define CSL_UART_PWREMU_MGMT_UTRST_SHIFT (0x0000000Eu)
#define CSL_UART_PWREMU_MGMT_UTRST_RESETVAL (0x00000000u)
/*----UTRST Tokens----*/
#define CSL_UART_PWREMU_MGMT_UTRST_RESET (0x00000000u)
#define CSL_UART_PWREMU_MGMT_UTRST_ENABLE (0x00000001u)

#define CSL_UART_PWREMU_MGMT_URRST_MASK  (0x00002000u)
#define CSL_UART_PWREMU_MGMT_URRST_SHIFT (0x0000000Du)
#define CSL_UART_PWREMU_MGMT_URRST_RESETVAL (0x00000000u)
/*----URRST Tokens----*/
#define CSL_UART_PWREMU_MGMT_URRST_RESET (0x00000000u)
#define CSL_UART_PWREMU_MGMT_URRST_ENABLE (0x00000001u)



#define CSL_UART_PWREMU_MGMT_FREE_MASK   (0x00000001u)
#define CSL_UART_PWREMU_MGMT_FREE_SHIFT  (0x00000000u)
#define CSL_UART_PWREMU_MGMT_FREE_RESETVAL (0x00000000u)
/*----FREE Tokens----*/
#define CSL_UART_PWREMU_MGMT_FREE_STOP   (0x00000000u)
#define CSL_UART_PWREMU_MGMT_FREE_RUN    (0x00000001u)

#define CSL_UART_PWREMU_MGMT_RESETVAL    (0x00000002u)

/* MDR */


#define CSL_UART_MDR_OSM_SEL_MASK        (0x00000001u)
#define CSL_UART_MDR_OSM_SEL_SHIFT       (0x00000000u)
#define CSL_UART_MDR_OSM_SEL_RESETVAL    (0x00000000u)
/*----OSM_SEL Tokens----*/
#define CSL_UART_MDR_OSM_SEL_16XOVERSAMPLING (0x00000000u)
#define CSL_UART_MDR_OSM_SEL_13XOVERSAMPLING (0x00000001u)

#define CSL_UART_MDR_RESETVAL            (0x00000000u)



#ifdef __cplusplus
}
#endif

#endif

