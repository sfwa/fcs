/********************************************************************
* Copyright (C) 2010 Texas Instruments Incorporated.
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
#ifndef _CSLR_PLLC_H_
#define _CSLR_PLLC_H_

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint8 RSVD0[228];
    volatile Uint32 RSTYPE;
    volatile Uint32 RSTCTRL;
    volatile Uint32 RSTCFG;
    volatile Uint32 RSISO;
    volatile Uint8 RSVD1[12];
    volatile Uint32 PLLCTL;
    volatile Uint8 RSVD2[4];
    volatile Uint32 SECCTL;
    volatile Uint8 RSVD3[4];
    volatile Uint32 PLLM;
    volatile Uint32 PREDIV;
    volatile Uint32 PLLDIV1_3[3];
    volatile Uint8 RSVD4[4];
    volatile Uint32 POSTDIV;
    volatile Uint8 RSVD5[12];
    volatile Uint32 PLLCMD;
    volatile Uint32 PLLSTAT;
    volatile Uint32 ALNCTL;
    volatile Uint32 DCHANGE;
    volatile Uint32 CKEN;
    volatile Uint32 CKSTAT;
    volatile Uint32 SYSTAT;
    volatile Uint8 RSVD6[12];
    volatile Uint32 PLLDIV4_16[13];
} CSL_PllcRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_PllcRegs               *CSL_PllcRegsOvly;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* RSTYPE */

#define CSL_PLLC_RSTYPE_EMUSRST_MASK		(0x10000000u)
#define CSL_PLLC_RSTYPE_EMUSRST_SHIFT		(0x0000001Cu)
#define CSL_PLLC_RSTYPE_EMUSRST_RESETVAL	(0x00000000u)

/*----EMUSRST Tokens----*/
#define CSL_PLLC_RSTYPE_EMUSRST_NO			(0x00000000u)
#define CSL_PLLC_RSTYPE_EMUSRST_YES			(0x00000001u)

#define CSL_PLLC_RSTYPE_WDRSTN_MASK		    (0x0000FF00u)
#define CSL_PLLC_RSTYPE_WDRSTN_SHIFT		(0x00000008u)
#define CSL_PLLC_RSTYPE_WDRSTN_RESETVAL	    (0x00000000u)

#define CSL_PLLC_RSTYPE_PLLCTLRST_MASK		(0x00000004u)
#define CSL_PLLC_RSTYPE_PLLCTLRST_SHIFT		(0x00000002u)
#define CSL_PLLC_RSTYPE_PLLCTLRST_RESETVAL	(0x00000000u)

/*----PLLCTLRST Tokens----*/
#define CSL_PLLC_RSTYPE_PLLCTLRST_NO	    (0x00000000u)
#define CSL_PLLC_RSTYPE_PLLCTLRST_YES	    (0x00000001u)

#define CSL_PLLC_RSTYPE_RESET_MASK          (0x00000002u)
#define CSL_PLLC_RSTYPE_RESET_SHIFT         (0x00000001u)
#define CSL_PLLC_RSTYPE_RESET_RESETVAL      (0x00000000u)

/*----RESET Tokens----*/
#define CSL_PLLC_RSTYPE_RESET_NO            (0x00000000u)
#define CSL_PLLC_RSTYPE_RESET_YES           (0x00000001u)

#define CSL_PLLC_RSTYPE_POR_MASK            (0x00000001u)
#define CSL_PLLC_RSTYPE_POR_SHIFT           (0x00000000u)
#define CSL_PLLC_RSTYPE_POR_RESETVAL        (0x00000000u)

/*----POR Tokens----*/
#define CSL_PLLC_RSTYPE_POR_NO              (0x00000000u)
#define CSL_PLLC_RSTYPE_POR_YES             (0x00000001u)

#define CSL_PLLC_RSTYPE_RESETVAL            (0x00000000u)

/* RSTCTRL */

#define CSL_PLLC_RSTCTRL_SWRST_MASK		    (0x00010000u)
#define CSL_PLLC_RSTCTRL_SWRST_SHIFT	    (0x00000010u)
#define CSL_PLLC_RSTCTRL_SWRST_RESETVAL     (0x00000001u)

/*----SWRST Tokens----*/
#define CSL_PLLC_RSTCTRL_SWRST_NO		    (0x00000001u)
#define CSL_PLLC_RSTCTRL_SWRST_YES		    (0x00000000u)

#define CSL_PLLC_RSTCTRL_KEY_MASK		    (0x0000FFFFu)
#define CSL_PLLC_RSTCTRL_KEY_SHIFT		    (0x00000000u)
#define CSL_PLLC_RSTCTRL_KEY_RESETVAL	    (0x00000003u)

#define CSL_PLLC_RSTCTRL_RESETVAL		    (0x00010003u)

/* RSTCFG */

#define CSL_PLLC_RSTCFG_PLLCTLRSTTYPE_MASK      (0x00002000u)
#define CSL_PLLC_RSTCFG_PLLCTLRSTTYPE_SHIFT		(0x0000000Du)
#define CSL_PLLC_RSTCFG_PLLCTLRSTTYPE_RESETVAL	(0x00000000u)

/*----PLLCTLRSTTYPE Tokens----*/
#define CSL_PLLC_RSTCFG_PLLCTLRSTTYPE_HARD	    (0x00000000u)
#define CSL_PLLC_RSTCFG_PLLCTLRSTTYPE_SOFT	    (0x00000001u)

#define CSL_PLLC_RSTCFG_RESETTYPE_MASK	        (0x00001000u)
#define CSL_PLLC_RSTCFG_RESETTYPE_SHIFT	        (0x0000000Cu)
#define CSL_PLLC_RSTCFG_RESETTYPE_RESETVAL      (0x00000000u)

/*----RESETTYPE Tokens----*/
#define CSL_PLLC_RSTCFG_RESETTYPE_HARD	        (0x00000000u)
#define CSL_PLLC_RSTCFG_RESETTYPE_SOFT	        (0x00000001u)

#define CSL_PLLC_RSTCFG_WDTYPEN_MASK	        (0x000000FFu)
#define CSL_PLLC_RSTCFG_WDTYPEN_SHIFT	        (0x00000000u)
#define CSL_PLLC_RSTCFG_WDTYPEN_RESETVAL        (0x00000000u)

/*----WDTYPEN Tokens----*/
#define CSL_PLLC_RSTCFG_WDTYPEN_HARD	        (0x00000000u)
#define CSL_PLLC_RSTCFG_WDTYPEN_SOFT	        (0x00000001u)

#define CSL_PLLC_RSTCFG_RESETVAL		        (0x00000000u)

/* RSISO */

#define CSL_PLLC_RSISO_SRIOISO_MASK	        (0x00000200u)
#define CSL_PLLC_RSISO_SRIOISO_SHIFT	    (0x00000009u)
#define CSL_PLLC_RSISO_SRIOISO_RESETVAL     (0x00000000u)

/*----SRIOISO Tokens----*/
#define CSL_PLLC_RSISO_SRIOISO_NO	        (0x00000000u)
#define CSL_PLLC_RSISO_SRIOISO_YES	        (0x00000001u)

#define CSL_PLLC_RSISO_SRISO_MASK	        (0x00000100u)
#define CSL_PLLC_RSISO_SRISO_SHIFT	        (0x00000008u)
#define CSL_PLLC_RSISO_SRISO_RESETVAL       (0x00000000u)

/*----SRISO Tokens----*/
#define CSL_PLLC_RSISO_SRISO_NO	            (0x00000000u)
#define CSL_PLLC_RSISO_SRISO_YES	        (0x00000001u)

#define CSL_PLLC_RSISO_AIF2ISO_MASK	        (0x00000008u)
#define CSL_PLLC_RSISO_AIF2ISO_SHIFT	    (0x00000003u)
#define CSL_PLLC_RSISO_AIF2ISO_RESETVAL     (0x00000000u)

/*----AIF2ISO Tokens----*/
#define CSL_PLLC_RSISO_AIF2ISO_NO	        (0x00000000u)
#define CSL_PLLC_RSISO_AIF2ISO_YES	        (0x00000001u)

#define CSL_PLLC_RSISO_RESETVAL		        (0x00000000u)

/* PLLCTL */

#define CSL_PLLC_PLLCTL_PLLENSRC_MASK       (0x00000020u)
#define CSL_PLLC_PLLCTL_PLLENSRC_SHIFT      (0x00000005u)
#define CSL_PLLC_PLLCTL_PLLENSRC_RESETVAL   (0x00000001u)

/*----PLLENSRC Tokens----*/
#define CSL_PLLC_PLLCTL_PLLENSRC_NO         (0x00000000u)
#define CSL_PLLC_PLLCTL_PLLENSRC_YES        (0x00000001u)

#define CSL_PLLC_PLLCTL_PLLRST_MASK         (0x00000008u)
#define CSL_PLLC_PLLCTL_PLLRST_SHIFT        (0x00000003u)
#define CSL_PLLC_PLLCTL_PLLRST_RESETVAL     (0x00000001u)

/*----PLLRST Tokens----*/
#define CSL_PLLC_PLLCTL_PLLRST_NO           (0x00000000u)
#define CSL_PLLC_PLLCTL_PLLRST_YES          (0x00000001u)

#define CSL_PLLC_PLLCTL_PLLPWRDN_MASK       (0x00000002u)
#define CSL_PLLC_PLLCTL_PLLPWRDN_SHIFT      (0x00000001u)
#define CSL_PLLC_PLLCTL_PLLPWRDN_RESETVAL   (0x00000000u)

/*----PLLPWRDN Tokens----*/
#define CSL_PLLC_PLLCTL_PLLPWRDN_NO         (0x00000000u)
#define CSL_PLLC_PLLCTL_PLLPWRDN_YES        (0x00000001u)

#define CSL_PLLC_PLLCTL_PLLEN_MASK          (0x00000001u)
#define CSL_PLLC_PLLCTL_PLLEN_SHIFT         (0x00000000u)
#define CSL_PLLC_PLLCTL_PLLEN_RESETVAL      (0x00000000u)

/*----PLLEN Tokens----*/
#define CSL_PLLC_PLLCTL_PLLEN_BYPASS        (0x00000000u)
#define CSL_PLLC_PLLCTL_PLLEN_PLL           (0x00000001u)

#define CSL_PLLC_PLLCTL_RESETVAL            (0x00000048u)

/* SECCTL */

#define CSL_PLLC_SECCTL_PLLSECCTL_MASK      (0x00FF0000u)
#define CSL_PLLC_SECCTL_PLLSECCTL_SHIFT     (0x00000010u)
#define CSL_PLLC_SECCTL_PLLSECCTL_RESETVAL  (0x00000000u)

#define CSL_PLLC_SECCTL_RESETVAL            (0x00000000u)

/* PLLM */

#define CSL_PLLC_PLLM_PLLM_MASK			        (0x0000003Fu)
#define CSL_PLLC_PLLM_PLLM_SHIFT            (0x00000000u)
#define CSL_PLLC_PLLM_PLLM_RESETVAL         (0x00000000u)

#define CSL_PLLC_PLLM_RESETVAL              (0x00000000u)

/* PREDIV */

#define CSL_PLLC_PREDIV_PREDEN_MASK         (0x00008000u)
#define CSL_PLLC_PREDIV_PREDEN_SHIFT        (0x0000000Fu)
#define CSL_PLLC_PREDIV_PREDEN_RESETVAL     (0x00000001u)

/*----PREDEN Tokens----*/
#define CSL_PLLC_PREDIV_PREDEN_DISABLE      (0x00000000u)
#define CSL_PLLC_PREDIV_PREDEN_ENABLE       (0x00000001u)

#define CSL_PLLC_PREDIV_RATIO_MASK          (0x0000001Fu)
#define CSL_PLLC_PREDIV_RATIO_SHIFT         (0x00000000u)
#define CSL_PLLC_PREDIV_RATIO_RESETVAL      (0x00000002u)

#define CSL_PLLC_PREDIV_RESETVAL            (0x00008002u)

/* PLLDIV1_3 */

#define CSL_PLLC_PLLDIV1_3_DNEN_MASK		(0x00008000u)
#define CSL_PLLC_PLLDIV1_3_DNEN_SHIFT		(0x0000000Fu)
#define CSL_PLLC_PLLDIV1_3_DNEN_RESETVAL	(0x00000001u)

/*----DnEN Tokens----*/
#define CSL_PLLC_PLLDIV1_3_DNEN_DISABLE	    (0x00000000u)
#define CSL_PLLC_PLLDIV1_3_DNEN_ENABLE	    (0x00000001u)

#define CSL_PLLC_PLLDIV1_3_RATIO_MASK       (0x0000001Fu)
#define CSL_PLLC_PLLDIV1_3_RATIO_SHIFT	    (0x00000000u)
#define CSL_PLLC_PLLDIV1_3_RATIO_RESETVAL   (0x00000002u)

#define CSL_PLLC_PLLDIV1_3_RESETVAL		    (0x00008002u)

/* POSTDIV */

#define CSL_PLLC_POSTDIV_POSTDEN_MASK       (0x00008000u)
#define CSL_PLLC_POSTDIV_POSTDEN_SHIFT      (0x0000000Fu)
#define CSL_PLLC_POSTDIV_POSTDEN_RESETVAL   (0x00000001u)

/*----POSTDEN Tokens----*/
#define CSL_PLLC_POSTDIV_POSTDEN_DISABLE    (0x00000000u)
#define CSL_PLLC_POSTDIV_POSTDEN_ENABLE     (0x00000001u)

#define CSL_PLLC_POSTDIV_RATIO_MASK         (0x0000001Fu)
#define CSL_PLLC_POSTDIV_RATIO_SHIFT        (0x00000000u)
#define CSL_PLLC_POSTDIV_RATIO_RESETVAL     (0x00000002u)

#define CSL_PLLC_POSTDIV_RESETVAL           (0x00008002u)

/* PLLCMD */

#define CSL_PLLC_PLLCMD_GOSET_MASK          (0x00000001u)
#define CSL_PLLC_PLLCMD_GOSET_SHIFT         (0x00000000u)
#define CSL_PLLC_PLLCMD_GOSET_RESETVAL      (0x00000000u)

/*----GOSET Tokens----*/
#define CSL_PLLC_PLLCMD_GOSET_CLRBIT        (0x00000000u)
#define CSL_PLLC_PLLCMD_GOSET_SET           (0x00000001u)

#define CSL_PLLC_PLLCMD_RESETVAL            (0x00000000u)

/* PLLSTAT */

#define CSL_PLLC_PLLSTAT_GOSTAT_MASK        (0x00000001u)
#define CSL_PLLC_PLLSTAT_GOSTAT_SHIFT       (0x00000000u)
#define CSL_PLLC_PLLSTAT_GOSTAT_RESETVAL    (0x00000000u)

/*----GOSTAT Tokens----*/
#define CSL_PLLC_PLLSTAT_GOSTAT_NONE        (0x00000000u)
#define CSL_PLLC_PLLSTAT_GOSTAT_INPROG      (0x00000001u)

#define CSL_PLLC_PLLSTAT_RESETVAL           (0x00000000u)

/* ALNCTL */

#define CSL_PLLC_ALNCTL_ALN1_16_MASK        (0x0000FFFFu)
#define CSL_PLLC_ALNCTL_ALN1_16_SHIFT       (0x00000000u)
#define CSL_PLLC_ALNCTL_ALN1_16_RESETVAL    (0x0000FFFFu)

#define CSL_PLLC_ALNCTL_RESETVAL            (0x0000FFFFu)

/* DCHANGE */

#define CSL_PLLC_DCHANGE_SYS1_16_MASK       (0x0000FFFFu)
#define CSL_PLLC_DCHANGE_SYS1_16_SHIFT      (0x00000000u)
#define CSL_PLLC_DCHANGE_SYS1_16_RESETVAL   (0x00000000u)

#define CSL_PLLC_DCHANGE_RESETVAL           (0x00000000u)

/* CKEN */

#define CSL_PLLC_CKEN_AUXEN_MASK            (0x00000001u)
#define CSL_PLLC_CKEN_AUXEN_SHIFT           (0x00000000u)
#define CSL_PLLC_CKEN_AUXEN_RESETVAL        (0x00000000u)

/*----AUXEN Tokens----*/
#define CSL_PLLC_CKEN_AUXEN_DISABLE         (0x00000000u)
#define CSL_PLLC_CKEN_AUXEN_ENABLE          (0x00000001u)

#define CSL_PLLC_CKEN_RESETVAL              (0x00000000u)

/* CKSTAT */

#define CSL_PLLC_CKSTAT_AUXON_MASK          (0x00000001u)
#define CSL_PLLC_CKSTAT_AUXON_SHIFT         (0x00000000u)
#define CSL_PLLC_CKSTAT_AUXON_RESETVAL      (0x00000000u)

/*----AUXON Tokens----*/
#define CSL_PLLC_CKSTAT_AUXON_DISABLE       (0x00000000u)
#define CSL_PLLC_CKSTAT_AUXON_ENABLE        (0x00000001u)

#define CSL_PLLC_CKSTAT_RESETVAL            (0x00000000u)

/* SYSTAT */

#define CSL_PLLC_SYSTAT_SYS1_16ON_MASK      (0x0000FFFFu)
#define CSL_PLLC_SYSTAT_SYS1_16ON_SHIFT     (0x00000000u)
#define CSL_PLLC_SYSTAT_SYS1_16ON_RESETVAL  (0x0000FFFFu)

#define CSL_PLLC_SYSTAT_RESETVAL		    (0x0000FFFFu)

/* PLLDIV4_16 */

#define CSL_PLLC_PLLDIV4_16_DNEN_MASK		(0x00008000u)
#define CSL_PLLC_PLLDIV4_16_DNEN_SHIFT		(0x0000000Fu)
#define CSL_PLLC_PLLDIV4_16_DNEN_RESETVAL	(0x00000001u)

/*----DnEN Tokens----*/
#define CSL_PLLC_PLLDIV4_16_DNEN_DISABLE	(0x00000000u)
#define CSL_PLLC_PLLDIV4_16_DNEN_ENABLE	    (0x00000001u)

#define CSL_PLLC_PLLDIV4_16_RATIO_MASK      (0x000000FFu)
#define CSL_PLLC_PLLDIV4_16_RATIO_SHIFT	    (0x00000000u)
#define CSL_PLLC_PLLDIV4_16_RATIO_RESETVAL  (0x00000002u)

#define CSL_PLLC_PLLDIV4_16_RESETVAL		(0x00008002u)

#endif
