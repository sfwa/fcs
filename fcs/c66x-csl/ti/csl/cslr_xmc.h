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
#ifndef _CSLR_XMC_H_
#define _CSLR_XMC_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) 	Modified the 'CSL_XmcRegs' register overlay structure. Merged 
 * 			the XMPAXL0, XMPAXH0, XMPAXL1, XMPAXH1 into the XMPAX array
 * 			and made XMPAX[] array an array of 16 such register pairs. This
 * 			was done because all the XMPAXL/H register pairs 0-15 had the 
 * 			exact same bit fields and masks and only diferred in the
 * 			reset values (which we do not use programatically). This 
 * 			simplifies the code and use of the APIs lot more simpler and
 * 			more consistent.
 * 	    b) Modified the header file includes to be RTSC compliant 
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for XMPAX
\**************************************************************************/
typedef struct  {
    volatile Uint32 XMPAXL;
    volatile Uint32 XMPAXH;
} CSL_XmcXmpaxRegs;


/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    CSL_XmcXmpaxRegs XMPAX[16];
    volatile Uint8 RSVD0[384];
    volatile Uint32 XMPFAR;
    volatile Uint32 XMPFSR;
    volatile Uint32 XMPFCR;
    volatile Uint8 RSVD1[116];
    volatile Uint32 MDMAARBX;
    volatile Uint8 RSVD2[124];
    volatile Uint32 XPFCMD;
    volatile Uint32 XPFACS;
    volatile Uint8 RSVD3[8];
    volatile Uint32 XPFAC0;
    volatile Uint32 XPFAC1;
    volatile Uint32 XPFAC2;
    volatile Uint32 XPFAC3;
    volatile Uint8 RSVD4[224];
    volatile Uint32 XPFADDR[8];
} CSL_XmcRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* XMPAXL */

#define CSL_XMC_XMPAXL_RADDR_MASK        (0xFFFFFF00u)
#define CSL_XMC_XMPAXL_RADDR_SHIFT       (0x00000008u)
#define CSL_XMC_XMPAXL_RADDR_RESETVAL    (0x00000000u)



#define CSL_XMC_XMPAXL_SR_MASK           (0x00000020u)
#define CSL_XMC_XMPAXL_SR_SHIFT          (0x00000005u)
#define CSL_XMC_XMPAXL_SR_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPAXL_SW_MASK           (0x00000010u)
#define CSL_XMC_XMPAXL_SW_SHIFT          (0x00000004u)
#define CSL_XMC_XMPAXL_SW_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPAXL_SX_MASK           (0x00000008u)
#define CSL_XMC_XMPAXL_SX_SHIFT          (0x00000003u)
#define CSL_XMC_XMPAXL_SX_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPAXL_UR_MASK           (0x00000004u)
#define CSL_XMC_XMPAXL_UR_SHIFT          (0x00000002u)
#define CSL_XMC_XMPAXL_UR_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPAXL_UW_MASK           (0x00000002u)
#define CSL_XMC_XMPAXL_UW_SHIFT          (0x00000001u)
#define CSL_XMC_XMPAXL_UW_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPAXL_UX_MASK           (0x00000001u)
#define CSL_XMC_XMPAXL_UX_SHIFT          (0x00000000u)
#define CSL_XMC_XMPAXL_UX_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPAXL_RESETVAL          (0x00000000u)

/* XMPAXH */

#define CSL_XMC_XMPAXH_BADDR_MASK        (0xFFFFF000u)
#define CSL_XMC_XMPAXH_BADDR_SHIFT       (0x0000000Cu)
#define CSL_XMC_XMPAXH_BADDR_RESETVAL    (0x00000000u)


#define CSL_XMC_XMPAXH_SEGSZ_MASK        (0x0000001Fu)
#define CSL_XMC_XMPAXH_SEGSZ_SHIFT       (0x00000000u)
#define CSL_XMC_XMPAXH_SEGSZ_RESETVAL    (0x00000000u)

#define CSL_XMC_XMPAXH_RESETVAL          (0x00000000u)

/* XMPAXL0 */

#define CSL_XMC_XMPAXL0_RADDR_MASK       (0xFFFFFF00u)
#define CSL_XMC_XMPAXL0_RADDR_SHIFT      (0x00000008u)
#define CSL_XMC_XMPAXL0_RADDR_RESETVAL   (0x00000000u)



#define CSL_XMC_XMPAXL0_SR_MASK          (0x00000020u)
#define CSL_XMC_XMPAXL0_SR_SHIFT         (0x00000005u)
#define CSL_XMC_XMPAXL0_SR_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL0_SW_MASK          (0x00000010u)
#define CSL_XMC_XMPAXL0_SW_SHIFT         (0x00000004u)
#define CSL_XMC_XMPAXL0_SW_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL0_SX_MASK          (0x00000008u)
#define CSL_XMC_XMPAXL0_SX_SHIFT         (0x00000003u)
#define CSL_XMC_XMPAXL0_SX_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL0_UR_MASK          (0x00000004u)
#define CSL_XMC_XMPAXL0_UR_SHIFT         (0x00000002u)
#define CSL_XMC_XMPAXL0_UR_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL0_UW_MASK          (0x00000002u)
#define CSL_XMC_XMPAXL0_UW_SHIFT         (0x00000001u)
#define CSL_XMC_XMPAXL0_UW_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL0_UX_MASK          (0x00000001u)
#define CSL_XMC_XMPAXL0_UX_SHIFT         (0x00000000u)
#define CSL_XMC_XMPAXL0_UX_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL0_RESETVAL         (0x000000FFu)

/* XMPAXH0 */

#define CSL_XMC_XMPAXH0_BADDR_MASK       (0xFFFFF000u)
#define CSL_XMC_XMPAXH0_BADDR_SHIFT      (0x0000000Cu)
#define CSL_XMC_XMPAXH0_BADDR_RESETVAL   (0x00000000u)


#define CSL_XMC_XMPAXH0_SEGSZ_MASK       (0x0000001Fu)
#define CSL_XMC_XMPAXH0_SEGSZ_SHIFT      (0x00000000u)
#define CSL_XMC_XMPAXH0_SEGSZ_RESETVAL   (0x0000001Eu)

#define CSL_XMC_XMPAXH0_RESETVAL         (0x0000001Eu)

/* XMPAXL1 */

#define CSL_XMC_XMPAXL1_RADDR_MASK       (0xFFFFFF00u)
#define CSL_XMC_XMPAXL1_RADDR_SHIFT      (0x00000008u)
#define CSL_XMC_XMPAXL1_RADDR_RESETVAL   (0x00800000u)



#define CSL_XMC_XMPAXL1_SR_MASK          (0x00000020u)
#define CSL_XMC_XMPAXL1_SR_SHIFT         (0x00000005u)
#define CSL_XMC_XMPAXL1_SR_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL1_SW_MASK          (0x00000010u)
#define CSL_XMC_XMPAXL1_SW_SHIFT         (0x00000004u)
#define CSL_XMC_XMPAXL1_SW_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL1_SX_MASK          (0x00000008u)
#define CSL_XMC_XMPAXL1_SX_SHIFT         (0x00000003u)
#define CSL_XMC_XMPAXL1_SX_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL1_UR_MASK          (0x00000004u)
#define CSL_XMC_XMPAXL1_UR_SHIFT         (0x00000002u)
#define CSL_XMC_XMPAXL1_UR_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL1_UW_MASK          (0x00000002u)
#define CSL_XMC_XMPAXL1_UW_SHIFT         (0x00000001u)
#define CSL_XMC_XMPAXL1_UW_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL1_UX_MASK          (0x00000001u)
#define CSL_XMC_XMPAXL1_UX_SHIFT         (0x00000000u)
#define CSL_XMC_XMPAXL1_UX_RESETVAL      (0x00000001u)

#define CSL_XMC_XMPAXL1_RESETVAL         (0x800000FFu)

/* XMPAXH1 */

#define CSL_XMC_XMPAXH1_BADDR_MASK       (0xFFFFF000u)
#define CSL_XMC_XMPAXH1_BADDR_SHIFT      (0x0000000Cu)
#define CSL_XMC_XMPAXH1_BADDR_RESETVAL   (0x00080000u)


#define CSL_XMC_XMPAXH1_SEGSZ_MASK       (0x0000001Fu)
#define CSL_XMC_XMPAXH1_SEGSZ_SHIFT      (0x00000000u)
#define CSL_XMC_XMPAXH1_SEGSZ_RESETVAL   (0x0000001Eu)

#define CSL_XMC_XMPAXH1_RESETVAL         (0x8000001Eu)

/* XMPFAR */

#define CSL_XMC_XMPFAR_FADDR_MASK        (0xFFFFFFFFu)
#define CSL_XMC_XMPFAR_FADDR_SHIFT       (0x00000000u)
#define CSL_XMC_XMPFAR_FADDR_RESETVAL    (0x00000000u)

#define CSL_XMC_XMPFAR_RESETVAL          (0x00000000u)

/* XMPFSR */


#define CSL_XMC_XMPFSR_LOCAL_MASK        (0x00000100u)
#define CSL_XMC_XMPFSR_LOCAL_SHIFT       (0x00000008u)
#define CSL_XMC_XMPFSR_LOCAL_RESETVAL    (0x00000000u)



#define CSL_XMC_XMPFSR_SR_MASK           (0x00000020u)
#define CSL_XMC_XMPFSR_SR_SHIFT          (0x00000005u)
#define CSL_XMC_XMPFSR_SR_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPFSR_SW_MASK           (0x00000010u)
#define CSL_XMC_XMPFSR_SW_SHIFT          (0x00000004u)
#define CSL_XMC_XMPFSR_SW_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPFSR_SX_MASK           (0x00000008u)
#define CSL_XMC_XMPFSR_SX_SHIFT          (0x00000003u)
#define CSL_XMC_XMPFSR_SX_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPFSR_UR_MASK           (0x00000004u)
#define CSL_XMC_XMPFSR_UR_SHIFT          (0x00000002u)
#define CSL_XMC_XMPFSR_UR_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPFSR_UW_MASK           (0x00000002u)
#define CSL_XMC_XMPFSR_UW_SHIFT          (0x00000001u)
#define CSL_XMC_XMPFSR_UW_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPFSR_UX_MASK           (0x00000001u)
#define CSL_XMC_XMPFSR_UX_SHIFT          (0x00000000u)
#define CSL_XMC_XMPFSR_UX_RESETVAL       (0x00000000u)

#define CSL_XMC_XMPFSR_RESETVAL          (0x00000000u)

/* XMPFCR */


#define CSL_XMC_XMPFCR_MPFCLR_MASK       (0x00000001u)
#define CSL_XMC_XMPFCR_MPFCLR_SHIFT      (0x00000000u)
#define CSL_XMC_XMPFCR_MPFCLR_RESETVAL   (0x00000000u)

#define CSL_XMC_XMPFCR_RESETVAL          (0x00000000u)

/* MDMAARBX */


#define CSL_XMC_MDMAARBX_PRI_MASK        (0x00070000u)
#define CSL_XMC_MDMAARBX_PRI_SHIFT       (0x00000010u)
#define CSL_XMC_MDMAARBX_PRI_RESETVAL    (0x00000007u)


#define CSL_XMC_MDMAARBX_RESETVAL        (0x00070000u)

/* XPFCMD */


#define CSL_XMC_XPFCMD_ACRST_MASK        (0x00000010u)
#define CSL_XMC_XPFCMD_ACRST_SHIFT       (0x00000004u)
#define CSL_XMC_XPFCMD_ACRST_RESETVAL    (0x00000000u)

#define CSL_XMC_XPFCMD_ACEN_MASK         (0x0000000Cu)
#define CSL_XMC_XPFCMD_ACEN_SHIFT        (0x00000002u)
#define CSL_XMC_XPFCMD_ACEN_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFCMD_ACENL_MASK        (0x00000002u)
#define CSL_XMC_XPFCMD_ACENL_SHIFT       (0x00000001u)
#define CSL_XMC_XPFCMD_ACENL_RESETVAL    (0x00000000u)

#define CSL_XMC_XPFCMD_INV_MASK          (0x00000001u)
#define CSL_XMC_XPFCMD_INV_SHIFT         (0x00000000u)
#define CSL_XMC_XPFCMD_INV_RESETVAL      (0x00000000u)

#define CSL_XMC_XPFCMD_RESETVAL          (0x00000000u)

/* XPFACS */


#define CSL_XMC_XPFACS_ACEN_MASK         (0x0000000Cu)
#define CSL_XMC_XPFACS_ACEN_SHIFT        (0x00000002u)
#define CSL_XMC_XPFACS_ACEN_RESETVAL     (0x00000000u)


#define CSL_XMC_XPFACS_RESETVAL          (0x00000000u)

/* XPFAC0 */


#define CSL_XMC_XPFAC0_SENT_MASK         (0x000FFFFFu)
#define CSL_XMC_XPFAC0_SENT_SHIFT        (0x00000000u)
#define CSL_XMC_XPFAC0_SENT_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFAC0_RESETVAL          (0x00000000u)

/* XPFAC1 */


#define CSL_XMC_XPFAC1_CANCELED_MASK     (0x000FFFFFu)
#define CSL_XMC_XPFAC1_CANCELED_SHIFT    (0x00000000u)
#define CSL_XMC_XPFAC1_CANCELED_RESETVAL (0x00000000u)

#define CSL_XMC_XPFAC1_RESETVAL          (0x00000000u)

/* XPFAC2 */


#define CSL_XMC_XPFAC2_HIT_MASK          (0x000FFFFFu)
#define CSL_XMC_XPFAC2_HIT_SHIFT         (0x00000000u)
#define CSL_XMC_XPFAC2_HIT_RESETVAL      (0x00000000u)

#define CSL_XMC_XPFAC2_RESETVAL          (0x00000000u)

/* XPFAC3 */


#define CSL_XMC_XPFAC3_MISS_MASK         (0x000FFFFFu)
#define CSL_XMC_XPFAC3_MISS_SHIFT        (0x00000000u)
#define CSL_XMC_XPFAC3_MISS_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFAC3_RESETVAL          (0x00000000u)

/* XPFADDR */

#define CSL_XMC_XPFADDR_ADDR_MASK        (0xFFFFFF80u)
#define CSL_XMC_XPFADDR_ADDR_SHIFT       (0x00000007u)
#define CSL_XMC_XPFADDR_ADDR_RESETVAL    (0x00000000u)

#define CSL_XMC_XPFADDR_DIR_MASK         (0x00000040u)
#define CSL_XMC_XPFADDR_DIR_SHIFT        (0x00000006u)
#define CSL_XMC_XPFADDR_DIR_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_DPH_MASK         (0x00000020u)
#define CSL_XMC_XPFADDR_DPH_SHIFT        (0x00000005u)
#define CSL_XMC_XPFADDR_DPH_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_DVH_MASK         (0x00000010u)
#define CSL_XMC_XPFADDR_DVH_SHIFT        (0x00000004u)
#define CSL_XMC_XPFADDR_DVH_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_AVH_MASK         (0x00000008u)
#define CSL_XMC_XPFADDR_AVH_SHIFT        (0x00000003u)
#define CSL_XMC_XPFADDR_AVH_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_DPL_MASK         (0x00000004u)
#define CSL_XMC_XPFADDR_DPL_SHIFT        (0x00000002u)
#define CSL_XMC_XPFADDR_DPL_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_DVL_MASK         (0x00000002u)
#define CSL_XMC_XPFADDR_DVL_SHIFT        (0x00000001u)
#define CSL_XMC_XPFADDR_DVL_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_AVL_MASK         (0x00000001u)
#define CSL_XMC_XPFADDR_AVL_SHIFT        (0x00000000u)
#define CSL_XMC_XPFADDR_AVL_RESETVAL     (0x00000000u)

#define CSL_XMC_XPFADDR_RESETVAL         (0x00000000u)

#endif
