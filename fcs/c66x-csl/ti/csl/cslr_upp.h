/*
 * cslr_upp.h
 *
 * This file contains the macros for Register Chip Support Library (CSL) which 
 * can be used for operations on the respective underlying hardware/peripheral
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#ifndef _CSLR_UPP_H_
#define _CSLR_UPP_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a)  Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 UPPID;
    volatile Uint32 UPPCR;
    volatile Uint32 UPDLB;
    volatile Uint8 RSVD0[4];
    volatile Uint32 UPCTL;
    volatile Uint32 UPICR;
    volatile Uint32 UPIVR;
    volatile Uint32 UPTCR;
    volatile Uint32 UPISR;
    volatile Uint32 UPIER;
    volatile Uint32 UPIES;
    volatile Uint32 UPIEC;
    volatile Uint32 UPEOI;
    volatile Uint8 RSVD1[12];
    volatile Uint32 UPID0;
    volatile Uint32 UPID1;
    volatile Uint32 UPID2;
    volatile Uint8 RSVD2[4];
    volatile Uint32 UPIS0;
    volatile Uint32 UPIS1;
    volatile Uint32 UPIS2;
    volatile Uint8 RSVD3[4];
    volatile Uint32 UPQD0;
    volatile Uint32 UPQD1;
    volatile Uint32 UPQD2;
    volatile Uint8 RSVD4[4];
    volatile Uint32 UPQS0;
    volatile Uint32 UPQS1;
    volatile Uint32 UPQS2;
} CSL_UppRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_UppRegs         *CSL_UppRegsOvly;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* UPPID */

#define CSL_UPP_UPPID_REVID_MASK         (0xFFFFFFFFu)
#define CSL_UPP_UPPID_REVID_SHIFT        (0x00000000u)
#define CSL_UPP_UPPID_REVID_RESETVAL     (0x44230100u)

#define CSL_UPP_UPPID_RESETVAL           (0x44230100u)

/* UPPCR */


#define CSL_UPP_UPPCR_DB_MASK            (0x00000080u)
#define CSL_UPP_UPPCR_DB_SHIFT           (0x00000007u)
#define CSL_UPP_UPPCR_DB_RESETVAL        (0x00000000u)
/*----DB Tokens----*/
#define CSL_UPP_UPPCR_DB_IDLE            (0x00000000u)
#define CSL_UPP_UPPCR_DB_ACTIVE          (0x00000001u)


#define CSL_UPP_UPPCR_SWRST_MASK         (0x00000010u)
#define CSL_UPP_UPPCR_SWRST_SHIFT        (0x00000004u)
#define CSL_UPP_UPPCR_SWRST_RESETVAL     (0x00000000u)
/*----SWRST Tokens----*/
#define CSL_UPP_UPPCR_SWRST_RUNNING      (0x00000000u)
#define CSL_UPP_UPPCR_SWRST_RESET        (0x00000001u)

#define CSL_UPP_UPPCR_EN_MASK            (0x00000008u)
#define CSL_UPP_UPPCR_EN_SHIFT           (0x00000003u)
#define CSL_UPP_UPPCR_EN_RESETVAL        (0x00000000u)
/*----EN Tokens----*/
#define CSL_UPP_UPPCR_EN_DISABLE         (0x00000000u)
#define CSL_UPP_UPPCR_EN_ENABLE          (0x00000001u)

#define CSL_UPP_UPPCR_RTEMU_MASK         (0x00000004u)
#define CSL_UPP_UPPCR_RTEMU_SHIFT        (0x00000002u)
#define CSL_UPP_UPPCR_RTEMU_RESETVAL     (0x00000000u)
/*----RTEMU Tokens----*/
#define CSL_UPP_UPPCR_RTEMU_DISABLE      (0x00000000u)
#define CSL_UPP_UPPCR_RTEMU_ENABLE       (0x00000001u)

#define CSL_UPP_UPPCR_SOFT_MASK          (0x00000002u)
#define CSL_UPP_UPPCR_SOFT_SHIFT         (0x00000001u)
#define CSL_UPP_UPPCR_SOFT_RESETVAL      (0x00000001u)
/*----SOFT Tokens----*/
#define CSL_UPP_UPPCR_SOFT_DISABLE       (0x00000000u)
#define CSL_UPP_UPPCR_SOFT_ENABLE        (0x00000001u)

#define CSL_UPP_UPPCR_FREE_MASK          (0x00000001u)
#define CSL_UPP_UPPCR_FREE_SHIFT         (0x00000000u)
#define CSL_UPP_UPPCR_FREE_RESETVAL      (0x00000001u)
/*----FREE Tokens----*/
#define CSL_UPP_UPPCR_FREE_DISABLE       (0x00000000u)
#define CSL_UPP_UPPCR_FREE_ENABLE        (0x00000001u)

#define CSL_UPP_UPPCR_RESETVAL           (0x00000003u)

/* UPDLB */


#define CSL_UPP_UPDLB_BA_MASK            (0x00002000u)
#define CSL_UPP_UPDLB_BA_SHIFT           (0x0000000Du)
#define CSL_UPP_UPDLB_BA_RESETVAL        (0x00000000u)
/*----BA Tokens----*/
#define CSL_UPP_UPDLB_BA_DISABLE         (0x00000000u)
#define CSL_UPP_UPDLB_BA_ENABLE          (0x00000001u)

#define CSL_UPP_UPDLB_AB_MASK            (0x00001000u)
#define CSL_UPP_UPDLB_AB_SHIFT           (0x0000000Cu)
#define CSL_UPP_UPDLB_AB_RESETVAL        (0x00000000u)
/*----AB Tokens----*/
#define CSL_UPP_UPDLB_AB_DISABLE         (0x00000000u)
#define CSL_UPP_UPDLB_AB_ENABLE          (0x00000001u)


#define CSL_UPP_UPDLB_RESETVAL           (0x00000000u)

/* UPCTL */


#define CSL_UPP_UPCTL_DPFB_MASK          (0x60000000u)
#define CSL_UPP_UPCTL_DPFB_SHIFT         (0x0000001Du)
#define CSL_UPP_UPCTL_DPFB_RESETVAL      (0x00000000u)
/*----DPFB Tokens----*/
#define CSL_UPP_UPCTL_DPFB_RJZE          (0x00000000u)
#define CSL_UPP_UPCTL_DPFB_RJSE          (0x00000001u)
#define CSL_UPP_UPCTL_DPFB_LJZF          (0x00000002u)
#define CSL_UPP_UPCTL_DPFB_RESERVED      (0x00000003u)

#define CSL_UPP_UPCTL_DPWB_MASK          (0x1C000000u)
#define CSL_UPP_UPCTL_DPWB_SHIFT         (0x0000001Au)
#define CSL_UPP_UPCTL_DPWB_RESETVAL      (0x00000000u)
/*----DPWB Tokens----*/
#define CSL_UPP_UPCTL_DPWB_FULL          (0x00000000u)
#define CSL_UPP_UPCTL_DPWB_9BIT          (0x00000001u)
#define CSL_UPP_UPCTL_DPWB_10BIT         (0x00000002u)
#define CSL_UPP_UPCTL_DPWB_11BIT         (0x00000003u)
#define CSL_UPP_UPCTL_DPWB_12BIT         (0x00000004u)
#define CSL_UPP_UPCTL_DPWB_13BIT         (0x00000005u)
#define CSL_UPP_UPCTL_DPWB_14BIT         (0x00000006u)
#define CSL_UPP_UPCTL_DPWB_15BIT         (0x00000007u)

#define CSL_UPP_UPCTL_IWB_MASK           (0x02000000u)
#define CSL_UPP_UPCTL_IWB_SHIFT          (0x00000019u)
#define CSL_UPP_UPCTL_IWB_RESETVAL       (0x00000000u)
/*----IWB Tokens----*/
#define CSL_UPP_UPCTL_IWB_8BIT           (0x00000000u)
#define CSL_UPP_UPCTL_IWB_16BIT          (0x00000001u)

#define CSL_UPP_UPCTL_DRB_MASK           (0x01000000u)
#define CSL_UPP_UPCTL_DRB_SHIFT          (0x00000018u)
#define CSL_UPP_UPCTL_DRB_RESETVAL       (0x00000000u)
/*----DRB Tokens----*/
#define CSL_UPP_UPCTL_DRB_SINGLE         (0x00000000u)
#define CSL_UPP_UPCTL_DRB_DOUBLE         (0x00000001u)


#define CSL_UPP_UPCTL_DPFA_MASK          (0x00600000u)
#define CSL_UPP_UPCTL_DPFA_SHIFT         (0x00000015u)
#define CSL_UPP_UPCTL_DPFA_RESETVAL      (0x00000000u)
/*----DPFA Tokens----*/
#define CSL_UPP_UPCTL_DPFA_RJZE          (0x00000000u)
#define CSL_UPP_UPCTL_DPFA_RJSE          (0x00000001u)
#define CSL_UPP_UPCTL_DPFA_LJZF          (0x00000002u)
#define CSL_UPP_UPCTL_DPFA_RESERVED      (0x00000003u)

#define CSL_UPP_UPCTL_DPWA_MASK          (0x001C0000u)
#define CSL_UPP_UPCTL_DPWA_SHIFT         (0x00000012u)
#define CSL_UPP_UPCTL_DPWA_RESETVAL      (0x00000000u)
/*----DPWA Tokens----*/
#define CSL_UPP_UPCTL_DPWA_FULL          (0x00000000u)
#define CSL_UPP_UPCTL_DPWA_9BIT          (0x00000001u)
#define CSL_UPP_UPCTL_DPWA_10BIT         (0x00000002u)
#define CSL_UPP_UPCTL_DPWA_11BIT         (0x00000003u)
#define CSL_UPP_UPCTL_DPWA_12BIT         (0x00000004u)
#define CSL_UPP_UPCTL_DPWA_13BIT         (0x00000005u)
#define CSL_UPP_UPCTL_DPWA_14BIT         (0x00000006u)
#define CSL_UPP_UPCTL_DPWA_15BIT         (0x00000007u)

#define CSL_UPP_UPCTL_IWA_MASK           (0x00020000u)
#define CSL_UPP_UPCTL_IWA_SHIFT          (0x00000011u)
#define CSL_UPP_UPCTL_IWA_RESETVAL       (0x00000000u)
/*----IWA Tokens----*/
#define CSL_UPP_UPCTL_IWA_8BIT           (0x00000000u)
#define CSL_UPP_UPCTL_IWA_16BIT          (0x00000001u)

#define CSL_UPP_UPCTL_DRA_MASK           (0x00010000u)
#define CSL_UPP_UPCTL_DRA_SHIFT          (0x00000010u)
#define CSL_UPP_UPCTL_DRA_RESETVAL       (0x00000000u)
/*----DRA Tokens----*/
#define CSL_UPP_UPCTL_DRA_SINGLE         (0x00000000u)
#define CSL_UPP_UPCTL_DRA_DOUBLE         (0x00000001u)


#define CSL_UPP_UPCTL_DDRDEMUX_MASK      (0x00000010u)
#define CSL_UPP_UPCTL_DDRDEMUX_SHIFT     (0x00000004u)
#define CSL_UPP_UPCTL_DDRDEMUX_RESETVAL  (0x00000000u)
/*----DDRDEMUX Tokens----*/
#define CSL_UPP_UPCTL_DDRDEMUX_DISABLE   (0x00000000u)
#define CSL_UPP_UPCTL_DDRDEMUX_ENABLE    (0x00000001u)

#define CSL_UPP_UPCTL_SDRTXIL_MASK       (0x00000008u)
#define CSL_UPP_UPCTL_SDRTXIL_SHIFT      (0x00000003u)
#define CSL_UPP_UPCTL_SDRTXIL_RESETVAL   (0x00000000u)
/*----SDRTXIL Tokens----*/
#define CSL_UPP_UPCTL_SDRTXIL_DISABLE    (0x00000000u)
#define CSL_UPP_UPCTL_SDRTXIL_ENABLE     (0x00000001u)

#define CSL_UPP_UPCTL_CHN_MASK           (0x00000004u)
#define CSL_UPP_UPCTL_CHN_SHIFT          (0x00000002u)
#define CSL_UPP_UPCTL_CHN_RESETVAL       (0x00000000u)
/*----CHN Tokens----*/
#define CSL_UPP_UPCTL_CHN_ONE            (0x00000000u)
#define CSL_UPP_UPCTL_CHN_TWO            (0x00000001u)

#define CSL_UPP_UPCTL_MODE_MASK          (0x00000003u)
#define CSL_UPP_UPCTL_MODE_SHIFT         (0x00000000u)
#define CSL_UPP_UPCTL_MODE_RESETVAL      (0x00000000u)
/*----MODE Tokens----*/
#define CSL_UPP_UPCTL_MODE_RECEIVE       (0x00000000u)
#define CSL_UPP_UPCTL_MODE_TRANSMIT      (0x00000001u)
#define CSL_UPP_UPCTL_MODE_DUPLEX0       (0x00000002u)
#define CSL_UPP_UPCTL_MODE_DUPLEX1       (0x00000003u)

#define CSL_UPP_UPCTL_RESETVAL           (0x00000000u)

/* UPICR */


#define CSL_UPP_UPICR_TRISB_MASK         (0x20000000u)
#define CSL_UPP_UPICR_TRISB_SHIFT        (0x0000001Du)
#define CSL_UPP_UPICR_TRISB_RESETVAL     (0x00000000u)
/*----TRISB Tokens----*/
#define CSL_UPP_UPICR_TRISB_DISABLE      (0x00000000u)
#define CSL_UPP_UPICR_TRISB_ENABLE       (0x00000001u)

#define CSL_UPP_UPICR_CLKINVB_MASK       (0x10000000u)
#define CSL_UPP_UPICR_CLKINVB_SHIFT      (0x0000001Cu)
#define CSL_UPP_UPICR_CLKINVB_RESETVAL   (0x00000000u)
/*----CLKINVB Tokens----*/
#define CSL_UPP_UPICR_CLKINVB_NORMAL     (0x00000000u)
#define CSL_UPP_UPICR_CLKINVB_INVERT     (0x00000001u)

#define CSL_UPP_UPICR_CLKDIVB_MASK       (0x0F000000u)
#define CSL_UPP_UPICR_CLKDIVB_SHIFT      (0x00000018u)
#define CSL_UPP_UPICR_CLKDIVB_RESETVAL   (0x00000000u)


#define CSL_UPP_UPICR_WAITB_MASK         (0x00200000u)
#define CSL_UPP_UPICR_WAITB_SHIFT        (0x00000015u)
#define CSL_UPP_UPICR_WAITB_RESETVAL     (0x00000000u)
/*----WAITB Tokens----*/
#define CSL_UPP_UPICR_WAITB_DISABLE      (0x00000000u)
#define CSL_UPP_UPICR_WAITB_ENABLE       (0x00000001u)

#define CSL_UPP_UPICR_ENAB_MASK          (0x00100000u)
#define CSL_UPP_UPICR_ENAB_SHIFT         (0x00000014u)
#define CSL_UPP_UPICR_ENAB_RESETVAL      (0x00000000u)
/*----ENAB Tokens----*/
#define CSL_UPP_UPICR_ENAB_DISABLE       (0x00000000u)
#define CSL_UPP_UPICR_ENAB_ENABLE        (0x00000001u)

#define CSL_UPP_UPICR_STARTB_MASK        (0x00080000u)
#define CSL_UPP_UPICR_STARTB_SHIFT       (0x00000013u)
#define CSL_UPP_UPICR_STARTB_RESETVAL    (0x00000000u)
/*----STARTB Tokens----*/
#define CSL_UPP_UPICR_STARTB_DISABLE     (0x00000000u)
#define CSL_UPP_UPICR_STARTB_ENABLE      (0x00000001u)

#define CSL_UPP_UPICR_WAITPOLB_MASK      (0x00040000u)
#define CSL_UPP_UPICR_WAITPOLB_SHIFT     (0x00000012u)
#define CSL_UPP_UPICR_WAITPOLB_RESETVAL  (0x00000000u)
/*----WAITPOLB Tokens----*/
#define CSL_UPP_UPICR_WAITPOLB_NORMAL    (0x00000000u)
#define CSL_UPP_UPICR_WAITPOLB_INVERT    (0x00000001u)

#define CSL_UPP_UPICR_ENAPOLB_MASK       (0x00020000u)
#define CSL_UPP_UPICR_ENAPOLB_SHIFT      (0x00000011u)
#define CSL_UPP_UPICR_ENAPOLB_RESETVAL   (0x00000000u)
/*----ENAPOLB Tokens----*/
#define CSL_UPP_UPICR_ENAPOLB_NORMAL     (0x00000000u)
#define CSL_UPP_UPICR_ENAPOLB_INVERT     (0x00000001u)

#define CSL_UPP_UPICR_STARTPOLB_MASK     (0x00010000u)
#define CSL_UPP_UPICR_STARTPOLB_SHIFT    (0x00000010u)
#define CSL_UPP_UPICR_STARTPOLB_RESETVAL (0x00000000u)
/*----STARTPOLB Tokens----*/
#define CSL_UPP_UPICR_STARTPOLB_NORMAL   (0x00000000u)
#define CSL_UPP_UPICR_STARTPOLB_INVERT   (0x00000001u)


#define CSL_UPP_UPICR_TRISA_MASK         (0x00002000u)
#define CSL_UPP_UPICR_TRISA_SHIFT        (0x0000000Du)
#define CSL_UPP_UPICR_TRISA_RESETVAL     (0x00000000u)
/*----TRISA Tokens----*/
#define CSL_UPP_UPICR_TRISA_DISABLE      (0x00000000u)
#define CSL_UPP_UPICR_TRISA_ENABLE       (0x00000001u)

#define CSL_UPP_UPICR_CLKINVA_MASK       (0x00001000u)
#define CSL_UPP_UPICR_CLKINVA_SHIFT      (0x0000000Cu)
#define CSL_UPP_UPICR_CLKINVA_RESETVAL   (0x00000000u)
/*----CLKINVA Tokens----*/
#define CSL_UPP_UPICR_CLKINVA_NORMAL     (0x00000000u)
#define CSL_UPP_UPICR_CLKINVA_INVERT     (0x00000001u)

#define CSL_UPP_UPICR_CLKDIVA_MASK       (0x00000F00u)
#define CSL_UPP_UPICR_CLKDIVA_SHIFT      (0x00000008u)
#define CSL_UPP_UPICR_CLKDIVA_RESETVAL   (0x00000000u)


#define CSL_UPP_UPICR_WAITA_MASK         (0x00000020u)
#define CSL_UPP_UPICR_WAITA_SHIFT        (0x00000005u)
#define CSL_UPP_UPICR_WAITA_RESETVAL     (0x00000000u)
/*----WAITA Tokens----*/
#define CSL_UPP_UPICR_WAITA_DISABLE      (0x00000000u)
#define CSL_UPP_UPICR_WAITA_ENABLE       (0x00000001u)

#define CSL_UPP_UPICR_ENAA_MASK          (0x00000010u)
#define CSL_UPP_UPICR_ENAA_SHIFT         (0x00000004u)
#define CSL_UPP_UPICR_ENAA_RESETVAL      (0x00000000u)
/*----ENAA Tokens----*/
#define CSL_UPP_UPICR_ENAA_DISABLE       (0x00000000u)
#define CSL_UPP_UPICR_ENAA_ENABLE        (0x00000001u)

#define CSL_UPP_UPICR_STARTA_MASK        (0x00000008u)
#define CSL_UPP_UPICR_STARTA_SHIFT       (0x00000003u)
#define CSL_UPP_UPICR_STARTA_RESETVAL    (0x00000000u)
/*----STARTA Tokens----*/
#define CSL_UPP_UPICR_STARTA_DISABLE     (0x00000000u)
#define CSL_UPP_UPICR_STARTA_ENABLE      (0x00000001u)

#define CSL_UPP_UPICR_WAITPOLA_MASK      (0x00000004u)
#define CSL_UPP_UPICR_WAITPOLA_SHIFT     (0x00000002u)
#define CSL_UPP_UPICR_WAITPOLA_RESETVAL  (0x00000000u)
/*----WAITPOLA Tokens----*/
#define CSL_UPP_UPICR_WAITPOLA_NORMAL    (0x00000000u)
#define CSL_UPP_UPICR_WAITPOLA_INVERT    (0x00000001u)

#define CSL_UPP_UPICR_ENAPOLA_MASK       (0x00000002u)
#define CSL_UPP_UPICR_ENAPOLA_SHIFT      (0x00000001u)
#define CSL_UPP_UPICR_ENAPOLA_RESETVAL   (0x00000000u)
/*----ENAPOLA Tokens----*/
#define CSL_UPP_UPICR_ENAPOLA_NORMAL     (0x00000000u)
#define CSL_UPP_UPICR_ENAPOLA_INVERT     (0x00000001u)

#define CSL_UPP_UPICR_STARTPOLA_MASK     (0x00000001u)
#define CSL_UPP_UPICR_STARTPOLA_SHIFT    (0x00000000u)
#define CSL_UPP_UPICR_STARTPOLA_RESETVAL (0x00000000u)
/*----STARTPOLA Tokens----*/
#define CSL_UPP_UPICR_STARTPOLA_NORMAL   (0x00000000u)
#define CSL_UPP_UPICR_STARTPOLA_INVERT   (0x00000001u)

#define CSL_UPP_UPICR_RESETVAL           (0x00000000u)

/* UPIVR */

#define CSL_UPP_UPIVR_VALB_MASK          (0xFFFF0000u)
#define CSL_UPP_UPIVR_VALB_SHIFT         (0x00000010u)
#define CSL_UPP_UPIVR_VALB_RESETVAL      (0x00000000u)

#define CSL_UPP_UPIVR_VALA_MASK          (0x0000FFFFu)
#define CSL_UPP_UPIVR_VALA_SHIFT         (0x00000000u)
#define CSL_UPP_UPIVR_VALA_RESETVAL      (0x00000000u)

#define CSL_UPP_UPIVR_RESETVAL           (0x00000000u)

/* UPTCR */


#define CSL_UPP_UPTCR_TXSIZEB_MASK       (0x03000000u)
#define CSL_UPP_UPTCR_TXSIZEB_SHIFT      (0x00000018u)
#define CSL_UPP_UPTCR_TXSIZEB_RESETVAL   (0x00000000u)
/*----TXSIZEB Tokens----*/
#define CSL_UPP_UPTCR_TXSIZEB_64B        (0x00000000u)
#define CSL_UPP_UPTCR_TXSIZEB_128B       (0x00000001u)
#define CSL_UPP_UPTCR_TXSIZEB_256B       (0x00000003u)


#define CSL_UPP_UPTCR_TXSIZEA_MASK       (0x00030000u)
#define CSL_UPP_UPTCR_TXSIZEA_SHIFT      (0x00000010u)
#define CSL_UPP_UPTCR_TXSIZEA_RESETVAL   (0x00000000u)
/*----TXSIZEA Tokens----*/
#define CSL_UPP_UPTCR_TXSIZEA_64B        (0x00000000u)
#define CSL_UPP_UPTCR_TXSIZEA_128B       (0x00000001u)
#define CSL_UPP_UPTCR_TXSIZEA_256B       (0x00000003u)


#define CSL_UPP_UPTCR_RDSIZEQ_MASK       (0x00000300u)
#define CSL_UPP_UPTCR_RDSIZEQ_SHIFT      (0x00000008u)
#define CSL_UPP_UPTCR_RDSIZEQ_RESETVAL   (0x00000000u)
/*----RDSIZEQ Tokens----*/
#define CSL_UPP_UPTCR_RDSIZEQ_64B        (0x00000000u)
#define CSL_UPP_UPTCR_RDSIZEQ_128B       (0x00000001u)
#define CSL_UPP_UPTCR_RDSIZEQ_256B       (0x00000003u)


#define CSL_UPP_UPTCR_RDSIZEI_MASK       (0x00000003u)
#define CSL_UPP_UPTCR_RDSIZEI_SHIFT      (0x00000000u)
#define CSL_UPP_UPTCR_RDSIZEI_RESETVAL   (0x00000000u)
/*----RDSIZEI Tokens----*/
#define CSL_UPP_UPTCR_RDSIZEI_64B        (0x00000000u)
#define CSL_UPP_UPTCR_RDSIZEI_128B       (0x00000001u)
#define CSL_UPP_UPTCR_RDSIZEI_256B       (0x00000003u)

#define CSL_UPP_UPTCR_RESETVAL           (0x00000000u)

/* UPISR */


#define CSL_UPP_UPISR_EOLQ_MASK          (0x00001000u)
#define CSL_UPP_UPISR_EOLQ_SHIFT         (0x0000000Cu)
#define CSL_UPP_UPISR_EOLQ_RESETVAL      (0x00000000u)
/*----EOLQ Tokens----*/
#define CSL_UPP_UPISR_EOLQ_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_EOLQ_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_EOWQ_MASK          (0x00000800u)
#define CSL_UPP_UPISR_EOWQ_SHIFT         (0x0000000Bu)
#define CSL_UPP_UPISR_EOWQ_RESETVAL      (0x00000000u)
/*----EOWQ Tokens----*/
#define CSL_UPP_UPISR_EOWQ_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_EOWQ_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_ERRQ_MASK          (0x00000400u)
#define CSL_UPP_UPISR_ERRQ_SHIFT         (0x0000000Au)
#define CSL_UPP_UPISR_ERRQ_RESETVAL      (0x00000000u)
/*----ERRQ Tokens----*/
#define CSL_UPP_UPISR_ERRQ_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_ERRQ_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_UORQ_MASK          (0x00000200u)
#define CSL_UPP_UPISR_UORQ_SHIFT         (0x00000009u)
#define CSL_UPP_UPISR_UORQ_RESETVAL      (0x00000000u)
/*----UORQ Tokens----*/
#define CSL_UPP_UPISR_UORQ_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_UORQ_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_DPEQ_MASK          (0x00000100u)
#define CSL_UPP_UPISR_DPEQ_SHIFT         (0x00000008u)
#define CSL_UPP_UPISR_DPEQ_RESETVAL      (0x00000000u)
/*----DPEQ Tokens----*/
#define CSL_UPP_UPISR_DPEQ_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_DPEQ_TRUE          (0x00000001u)


#define CSL_UPP_UPISR_EOLI_MASK          (0x00000010u)
#define CSL_UPP_UPISR_EOLI_SHIFT         (0x00000004u)
#define CSL_UPP_UPISR_EOLI_RESETVAL      (0x00000000u)
/*----EOLI Tokens----*/
#define CSL_UPP_UPISR_EOLI_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_EOLI_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_EOWI_MASK          (0x00000008u)
#define CSL_UPP_UPISR_EOWI_SHIFT         (0x00000003u)
#define CSL_UPP_UPISR_EOWI_RESETVAL      (0x00000000u)
/*----EOWI Tokens----*/
#define CSL_UPP_UPISR_EOWI_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_EOWI_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_ERRI_MASK          (0x00000004u)
#define CSL_UPP_UPISR_ERRI_SHIFT         (0x00000002u)
#define CSL_UPP_UPISR_ERRI_RESETVAL      (0x00000000u)
/*----ERRI Tokens----*/
#define CSL_UPP_UPISR_ERRI_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_ERRI_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_UORI_MASK          (0x00000002u)
#define CSL_UPP_UPISR_UORI_SHIFT         (0x00000001u)
#define CSL_UPP_UPISR_UORI_RESETVAL      (0x00000000u)
/*----UORI Tokens----*/
#define CSL_UPP_UPISR_UORI_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_UORI_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_DPEI_MASK          (0x00000001u)
#define CSL_UPP_UPISR_DPEI_SHIFT         (0x00000000u)
#define CSL_UPP_UPISR_DPEI_RESETVAL      (0x00000000u)
/*----DPEI Tokens----*/
#define CSL_UPP_UPISR_DPEI_FALSE         (0x00000000u)
#define CSL_UPP_UPISR_DPEI_TRUE          (0x00000001u)

#define CSL_UPP_UPISR_RESETVAL           (0x00000000u)

/* UPIER */


#define CSL_UPP_UPIER_EOLQ_MASK          (0x00001000u)
#define CSL_UPP_UPIER_EOLQ_SHIFT         (0x0000000Cu)
#define CSL_UPP_UPIER_EOLQ_RESETVAL      (0x00000000u)
/*----EOLQ Tokens----*/
#define CSL_UPP_UPIER_EOLQ_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_EOLQ_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_EOWQ_MASK          (0x00000800u)
#define CSL_UPP_UPIER_EOWQ_SHIFT         (0x0000000Bu)
#define CSL_UPP_UPIER_EOWQ_RESETVAL      (0x00000000u)
/*----EOWQ Tokens----*/
#define CSL_UPP_UPIER_EOWQ_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_EOWQ_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_ERRQ_MASK          (0x00000400u)
#define CSL_UPP_UPIER_ERRQ_SHIFT         (0x0000000Au)
#define CSL_UPP_UPIER_ERRQ_RESETVAL      (0x00000000u)
/*----ERRQ Tokens----*/
#define CSL_UPP_UPIER_ERRQ_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_ERRQ_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_UORQ_MASK          (0x00000200u)
#define CSL_UPP_UPIER_UORQ_SHIFT         (0x00000009u)
#define CSL_UPP_UPIER_UORQ_RESETVAL      (0x00000000u)
/*----UORQ Tokens----*/
#define CSL_UPP_UPIER_UORQ_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_UORQ_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_DPEQ_MASK          (0x00000100u)
#define CSL_UPP_UPIER_DPEQ_SHIFT         (0x00000008u)
#define CSL_UPP_UPIER_DPEQ_RESETVAL      (0x00000000u)
/*----DPEQ Tokens----*/
#define CSL_UPP_UPIER_DPEQ_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_DPEQ_TRUE          (0x00000001u)


#define CSL_UPP_UPIER_EOLI_MASK          (0x00000010u)
#define CSL_UPP_UPIER_EOLI_SHIFT         (0x00000004u)
#define CSL_UPP_UPIER_EOLI_RESETVAL      (0x00000000u)
/*----EOLI Tokens----*/
#define CSL_UPP_UPIER_EOLI_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_EOLI_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_EOWI_MASK          (0x00000008u)
#define CSL_UPP_UPIER_EOWI_SHIFT         (0x00000003u)
#define CSL_UPP_UPIER_EOWI_RESETVAL      (0x00000000u)
/*----EOWI Tokens----*/
#define CSL_UPP_UPIER_EOWI_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_EOWI_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_ERRI_MASK          (0x00000004u)
#define CSL_UPP_UPIER_ERRI_SHIFT         (0x00000002u)
#define CSL_UPP_UPIER_ERRI_RESETVAL      (0x00000000u)
/*----ERRI Tokens----*/
#define CSL_UPP_UPIER_ERRI_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_ERRI_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_UORI_MASK          (0x00000002u)
#define CSL_UPP_UPIER_UORI_SHIFT         (0x00000001u)
#define CSL_UPP_UPIER_UORI_RESETVAL      (0x00000000u)
/*----UORI Tokens----*/
#define CSL_UPP_UPIER_UORI_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_UORI_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_DPEI_MASK          (0x00000001u)
#define CSL_UPP_UPIER_DPEI_SHIFT         (0x00000000u)
#define CSL_UPP_UPIER_DPEI_RESETVAL      (0x00000000u)
/*----DPEI Tokens----*/
#define CSL_UPP_UPIER_DPEI_FALSE         (0x00000000u)
#define CSL_UPP_UPIER_DPEI_TRUE          (0x00000001u)

#define CSL_UPP_UPIER_RESETVAL           (0x00000000u)

/* UPIES */


#define CSL_UPP_UPIES_EOLQ_MASK          (0x00001000u)
#define CSL_UPP_UPIES_EOLQ_SHIFT         (0x0000000Cu)
#define CSL_UPP_UPIES_EOLQ_RESETVAL      (0x00000000u)
/*----EOLQ Tokens----*/
#define CSL_UPP_UPIES_EOLQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_EOLQ_SET           (0x00000001u)

#define CSL_UPP_UPIES_EOWQ_MASK          (0x00000800u)
#define CSL_UPP_UPIES_EOWQ_SHIFT         (0x0000000Bu)
#define CSL_UPP_UPIES_EOWQ_RESETVAL      (0x00000000u)
/*----EOWQ Tokens----*/
#define CSL_UPP_UPIES_EOWQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_EOWQ_SET           (0x00000001u)

#define CSL_UPP_UPIES_ERRQ_MASK          (0x00000400u)
#define CSL_UPP_UPIES_ERRQ_SHIFT         (0x0000000Au)
#define CSL_UPP_UPIES_ERRQ_RESETVAL      (0x00000000u)
/*----ERRQ Tokens----*/
#define CSL_UPP_UPIES_ERRQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_ERRQ_SET           (0x00000001u)

#define CSL_UPP_UPIES_UORQ_MASK          (0x00000200u)
#define CSL_UPP_UPIES_UORQ_SHIFT         (0x00000009u)
#define CSL_UPP_UPIES_UORQ_RESETVAL      (0x00000000u)
/*----UORQ Tokens----*/
#define CSL_UPP_UPIES_UORQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_UORQ_SET           (0x00000001u)

#define CSL_UPP_UPIES_DPEQ_MASK          (0x00000100u)
#define CSL_UPP_UPIES_DPEQ_SHIFT         (0x00000008u)
#define CSL_UPP_UPIES_DPEQ_RESETVAL      (0x00000000u)
/*----DPEQ Tokens----*/
#define CSL_UPP_UPIES_DPEQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_DPEQ_SET           (0x00000001u)


#define CSL_UPP_UPIES_EOLI_MASK          (0x00000010u)
#define CSL_UPP_UPIES_EOLI_SHIFT         (0x00000004u)
#define CSL_UPP_UPIES_EOLI_RESETVAL      (0x00000000u)
/*----EOLI Tokens----*/
#define CSL_UPP_UPIES_EOLI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_EOLI_SET           (0x00000001u)

#define CSL_UPP_UPIES_EOWI_MASK          (0x00000008u)
#define CSL_UPP_UPIES_EOWI_SHIFT         (0x00000003u)
#define CSL_UPP_UPIES_EOWI_RESETVAL      (0x00000000u)
/*----EOWI Tokens----*/
#define CSL_UPP_UPIES_EOWI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_EOWI_SET           (0x00000001u)

#define CSL_UPP_UPIES_ERRI_MASK          (0x00000004u)
#define CSL_UPP_UPIES_ERRI_SHIFT         (0x00000002u)
#define CSL_UPP_UPIES_ERRI_RESETVAL      (0x00000000u)
/*----ERRI Tokens----*/
#define CSL_UPP_UPIES_ERRI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_ERRI_SET           (0x00000001u)

#define CSL_UPP_UPIES_UORI_MASK          (0x00000002u)
#define CSL_UPP_UPIES_UORI_SHIFT         (0x00000001u)
#define CSL_UPP_UPIES_UORI_RESETVAL      (0x00000000u)
/*----UORI Tokens----*/
#define CSL_UPP_UPIES_UORI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_UORI_SET           (0x00000001u)

#define CSL_UPP_UPIES_DPEI_MASK          (0x00000001u)
#define CSL_UPP_UPIES_DPEI_SHIFT         (0x00000000u)
#define CSL_UPP_UPIES_DPEI_RESETVAL      (0x00000000u)
/*----DPEI Tokens----*/
#define CSL_UPP_UPIES_DPEI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIES_DPEI_SET           (0x00000001u)

#define CSL_UPP_UPIES_RESETVAL           (0x00000000u)

/* UPIEC */


#define CSL_UPP_UPIEC_EOLQ_MASK          (0x00001000u)
#define CSL_UPP_UPIEC_EOLQ_SHIFT         (0x0000000Cu)
#define CSL_UPP_UPIEC_EOLQ_RESETVAL      (0x00000000u)
/*----EOLQ Tokens----*/
#define CSL_UPP_UPIEC_EOLQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_EOLQ_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_EOWQ_MASK          (0x00000800u)
#define CSL_UPP_UPIEC_EOWQ_SHIFT         (0x0000000Bu)
#define CSL_UPP_UPIEC_EOWQ_RESETVAL      (0x00000000u)
/*----EOWQ Tokens----*/
#define CSL_UPP_UPIEC_EOWQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_EOWQ_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_ERRQ_MASK          (0x00000400u)
#define CSL_UPP_UPIEC_ERRQ_SHIFT         (0x0000000Au)
#define CSL_UPP_UPIEC_ERRQ_RESETVAL      (0x00000000u)
/*----ERRQ Tokens----*/
#define CSL_UPP_UPIEC_ERRQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_ERRQ_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_UORQ_MASK          (0x00000200u)
#define CSL_UPP_UPIEC_UORQ_SHIFT         (0x00000009u)
#define CSL_UPP_UPIEC_UORQ_RESETVAL      (0x00000000u)
/*----UORQ Tokens----*/
#define CSL_UPP_UPIEC_UORQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_UORQ_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_DPEQ_MASK          (0x00000100u)
#define CSL_UPP_UPIEC_DPEQ_SHIFT         (0x00000008u)
#define CSL_UPP_UPIEC_DPEQ_RESETVAL      (0x00000000u)
/*----DPEQ Tokens----*/
#define CSL_UPP_UPIEC_DPEQ_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_DPEQ_CLEAR         (0x00000001u)


#define CSL_UPP_UPIEC_EOLI_MASK          (0x00000010u)
#define CSL_UPP_UPIEC_EOLI_SHIFT         (0x00000004u)
#define CSL_UPP_UPIEC_EOLI_RESETVAL      (0x00000000u)
/*----EOLI Tokens----*/
#define CSL_UPP_UPIEC_EOLI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_EOLI_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_EOWI_MASK          (0x00000008u)
#define CSL_UPP_UPIEC_EOWI_SHIFT         (0x00000003u)
#define CSL_UPP_UPIEC_EOWI_RESETVAL      (0x00000000u)
/*----EOWI Tokens----*/
#define CSL_UPP_UPIEC_EOWI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_EOWI_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_ERRI_MASK          (0x00000004u)
#define CSL_UPP_UPIEC_ERRI_SHIFT         (0x00000002u)
#define CSL_UPP_UPIEC_ERRI_RESETVAL      (0x00000000u)
/*----ERRI Tokens----*/
#define CSL_UPP_UPIEC_ERRI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_ERRI_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_UORI_MASK          (0x00000002u)
#define CSL_UPP_UPIEC_UORI_SHIFT         (0x00000001u)
#define CSL_UPP_UPIEC_UORI_RESETVAL      (0x00000000u)
/*----UORI Tokens----*/
#define CSL_UPP_UPIEC_UORI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_UORI_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_DPEI_MASK          (0x00000001u)
#define CSL_UPP_UPIEC_DPEI_SHIFT         (0x00000000u)
#define CSL_UPP_UPIEC_DPEI_RESETVAL      (0x00000000u)
/*----DPEI Tokens----*/
#define CSL_UPP_UPIEC_DPEI_DISABLED      (0x00000000u)
#define CSL_UPP_UPIEC_DPEI_CLEAR         (0x00000001u)

#define CSL_UPP_UPIEC_RESETVAL           (0x00000000u)

/* UPEOI */


#define CSL_UPP_UPEOI_EOI_MASK           (0x000000FFu)
#define CSL_UPP_UPEOI_EOI_SHIFT          (0x00000000u)
#define CSL_UPP_UPEOI_EOI_RESETVAL       (0x00000000u)
/*----EOI Tokens----*/
#define CSL_UPP_UPEOI_EOI_DONE           (0x00000000u)

#define CSL_UPP_UPEOI_RESETVAL           (0x00000000u)

/* UPID0 */

#define CSL_UPP_UPID0_ADDRH_MASK         (0xFFFFFFF8u)
#define CSL_UPP_UPID0_ADDRH_SHIFT        (0x00000003u)
#define CSL_UPP_UPID0_ADDRH_RESETVAL     (0x00000000u)

#define CSL_UPP_UPID0_ADDR_MASK          (0x00000007u)
#define CSL_UPP_UPID0_ADDR_SHIFT         (0x00000000u)
#define CSL_UPP_UPID0_ADDR_RESETVAL      (0x00000000u)

#define CSL_UPP_UPID0_RESETVAL           (0x00000000u)

/* UPID1 */

#define CSL_UPP_UPID1_LNCNT_MASK         (0xFFFF0000u)
#define CSL_UPP_UPID1_LNCNT_SHIFT        (0x00000010u)
#define CSL_UPP_UPID1_LNCNT_RESETVAL     (0x00000000u)

#define CSL_UPP_UPID1_BCNTH_MASK         (0x0000FFFEu)
#define CSL_UPP_UPID1_BCNTH_SHIFT        (0x00000001u)
#define CSL_UPP_UPID1_BCNTH_RESETVAL     (0x00000000u)

#define CSL_UPP_UPID1_BCNT_MASK          (0x00000001u)
#define CSL_UPP_UPID1_BCNT_SHIFT         (0x00000000u)
#define CSL_UPP_UPID1_BCNT_RESETVAL      (0x00000000u)

#define CSL_UPP_UPID1_RESETVAL           (0x00000000u)

/* UPID2 */


#define CSL_UPP_UPID2_LNOFFSETH_MASK     (0x0000FFF8u)
#define CSL_UPP_UPID2_LNOFFSETH_SHIFT    (0x00000003u)
#define CSL_UPP_UPID2_LNOFFSETH_RESETVAL (0x00000000u)

#define CSL_UPP_UPID2_LNOFFSET_MASK      (0x00000007u)
#define CSL_UPP_UPID2_LNOFFSET_SHIFT     (0x00000000u)
#define CSL_UPP_UPID2_LNOFFSET_RESETVAL  (0x00000000u)

#define CSL_UPP_UPID2_RESETVAL           (0x00000000u)

/* UPIS0 */

#define CSL_UPP_UPIS0_ADDR_MASK          (0xFFFFFFFFu)
#define CSL_UPP_UPIS0_ADDR_SHIFT         (0x00000000u)
#define CSL_UPP_UPIS0_ADDR_RESETVAL      (0x00000000u)

#define CSL_UPP_UPIS0_RESETVAL           (0x00000000u)

/* UPIS1 */

#define CSL_UPP_UPIS1_LNCNT_MASK         (0xFFFF0000u)
#define CSL_UPP_UPIS1_LNCNT_SHIFT        (0x00000010u)
#define CSL_UPP_UPIS1_LNCNT_RESETVAL     (0x00000000u)

#define CSL_UPP_UPIS1_BCNT_MASK          (0x0000FFFFu)
#define CSL_UPP_UPIS1_BCNT_SHIFT         (0x00000000u)
#define CSL_UPP_UPIS1_BCNT_RESETVAL      (0x00000000u)

#define CSL_UPP_UPIS1_RESETVAL           (0x00000000u)

/* UPIS2 */


#define CSL_UPP_UPIS2_WM_MASK            (0x000000F0u)
#define CSL_UPP_UPIS2_WM_SHIFT           (0x00000004u)
#define CSL_UPP_UPIS2_WM_RESETVAL        (0x00000000u)


#define CSL_UPP_UPIS2_PEND_MASK          (0x00000002u)
#define CSL_UPP_UPIS2_PEND_SHIFT         (0x00000001u)
#define CSL_UPP_UPIS2_PEND_RESETVAL      (0x00000000u)
/*----PEND Tokens----*/
#define CSL_UPP_UPIS2_PEND_FALSE         (0x00000000u)
#define CSL_UPP_UPIS2_PEND_TRUE          (0x00000001u)

#define CSL_UPP_UPIS2_ACT_MASK           (0x00000001u)
#define CSL_UPP_UPIS2_ACT_SHIFT          (0x00000000u)
#define CSL_UPP_UPIS2_ACT_RESETVAL       (0x00000000u)
/*----ACT Tokens----*/
#define CSL_UPP_UPIS2_ACT_FALSE          (0x00000000u)
#define CSL_UPP_UPIS2_ACT_TRUE           (0x00000001u)

#define CSL_UPP_UPIS2_RESETVAL           (0x00000000u)

/* UPQD0 */

#define CSL_UPP_UPQD0_ADDRH_MASK         (0xFFFFFFF8u)
#define CSL_UPP_UPQD0_ADDRH_SHIFT        (0x00000003u)
#define CSL_UPP_UPQD0_ADDRH_RESETVAL     (0x00000000u)

#define CSL_UPP_UPQD0_ADDR_MASK          (0x00000007u)
#define CSL_UPP_UPQD0_ADDR_SHIFT         (0x00000000u)
#define CSL_UPP_UPQD0_ADDR_RESETVAL      (0x00000000u)

#define CSL_UPP_UPQD0_RESETVAL           (0x00000000u)

/* UPQD1 */

#define CSL_UPP_UPQD1_LNCNT_MASK         (0xFFFF0000u)
#define CSL_UPP_UPQD1_LNCNT_SHIFT        (0x00000010u)
#define CSL_UPP_UPQD1_LNCNT_RESETVAL     (0x00000000u)

#define CSL_UPP_UPQD1_BCNTH_MASK         (0x0000FFFEu)
#define CSL_UPP_UPQD1_BCNTH_SHIFT        (0x00000001u)
#define CSL_UPP_UPQD1_BCNTH_RESETVAL     (0x00000000u)

#define CSL_UPP_UPQD1_BCNT_MASK          (0x00000001u)
#define CSL_UPP_UPQD1_BCNT_SHIFT         (0x00000000u)
#define CSL_UPP_UPQD1_BCNT_RESETVAL      (0x00000000u)

#define CSL_UPP_UPQD1_RESETVAL           (0x00000000u)

/* UPQD2 */


#define CSL_UPP_UPQD2_LNOFFSETH_MASK     (0x0000FFF8u)
#define CSL_UPP_UPQD2_LNOFFSETH_SHIFT    (0x00000003u)
#define CSL_UPP_UPQD2_LNOFFSETH_RESETVAL (0x00000000u)

#define CSL_UPP_UPQD2_LNOFFSET_MASK      (0x00000007u)
#define CSL_UPP_UPQD2_LNOFFSET_SHIFT     (0x00000000u)
#define CSL_UPP_UPQD2_LNOFFSET_RESETVAL  (0x00000000u)

#define CSL_UPP_UPQD2_RESETVAL           (0x00000000u)

/* UPQS0 */

#define CSL_UPP_UPQS0_ADDR_MASK          (0xFFFFFFFFu)
#define CSL_UPP_UPQS0_ADDR_SHIFT         (0x00000000u)
#define CSL_UPP_UPQS0_ADDR_RESETVAL      (0x00000000u)

#define CSL_UPP_UPQS0_RESETVAL           (0x00000000u)

/* UPQS1 */

#define CSL_UPP_UPQS1_LNCNT_MASK         (0xFFFF0000u)
#define CSL_UPP_UPQS1_LNCNT_SHIFT        (0x00000010u)
#define CSL_UPP_UPQS1_LNCNT_RESETVAL     (0x00000000u)

#define CSL_UPP_UPQS1_BCNT_MASK          (0x0000FFFFu)
#define CSL_UPP_UPQS1_BCNT_SHIFT         (0x00000000u)
#define CSL_UPP_UPQS1_BCNT_RESETVAL      (0x00000000u)

#define CSL_UPP_UPQS1_RESETVAL           (0x00000000u)

/* UPQS2 */


#define CSL_UPP_UPQS2_WM_MASK            (0x000000F0u)
#define CSL_UPP_UPQS2_WM_SHIFT           (0x00000004u)
#define CSL_UPP_UPQS2_WM_RESETVAL        (0x00000000u)


#define CSL_UPP_UPQS2_PEND_MASK          (0x00000002u)
#define CSL_UPP_UPQS2_PEND_SHIFT         (0x00000001u)
#define CSL_UPP_UPQS2_PEND_RESETVAL      (0x00000000u)
/*----PEND Tokens----*/
#define CSL_UPP_UPQS2_PEND_FALSE         (0x00000000u)
#define CSL_UPP_UPQS2_PEND_TRUE          (0x00000001u)

#define CSL_UPP_UPQS2_ACT_MASK           (0x00000001u)
#define CSL_UPP_UPQS2_ACT_SHIFT          (0x00000000u)
#define CSL_UPP_UPQS2_ACT_RESETVAL       (0x00000000u)
/*----ACT Tokens----*/
#define CSL_UPP_UPQS2_ACT_FALSE          (0x00000000u)
#define CSL_UPP_UPQS2_ACT_TRUE           (0x00000001u)

#define CSL_UPP_UPQS2_RESETVAL           (0x00000000u)

#endif
