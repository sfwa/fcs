/*  ===========================================================================
 *  Copyright (c) Texas Instruments Inc 2002, 2003, 2004, 2005, 2006
 *
 *  Use of this software is controlled by the terms and conditions found
 *  in the license agreement under which this software has been supplied
 *  provided
 *
 *  ===========================================================================
 */

/** ============================================================================
 *   @file  cslr_ectl.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  This file contains the Register Desciptions for TIMER
 *
 */

#ifndef _CSLR_ECTL_H_
#define _CSLR_ECTL_H_

#include <cslr.h>
#include <tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 IDVER;
    volatile Uint32 SOFT_RESET;
    volatile Uint32 EM_CONTROL;
    volatile Uint32 INT_CONTROL;
    volatile Uint32 C0_RX_THRESH_EN;
    volatile Uint32 C0_RX_EN;
    volatile Uint32 C0_TX_EN;
    volatile Uint32 C0_MISC_EN;
    volatile Uint32 C1_RX_THRESH_EN;
    volatile Uint32 C1_RX_EN;
    volatile Uint32 C1_TX_EN;
    volatile Uint32 C1_MISC_EN;
    volatile Uint32 C2_RX_THRESH_EN;
    volatile Uint32 C2_RX_EN;
    volatile Uint32 C2_TX_EN;
    volatile Uint32 C2_MISC_EN;
    volatile Uint32 C3_RX_THRESH_EN;
    volatile Uint32 C3_RX_EN;
    volatile Uint32 C3_TX_EN;
    volatile Uint32 C3_MISC_EN;
    volatile Uint32 C4_RX_THRESH_EN;
    volatile Uint32 C4_RX_EN;
    volatile Uint32 C4_TX_EN;
    volatile Uint32 C4_MISC_EN;
    volatile Uint32 C5_RX_THRESH_EN;
    volatile Uint32 C5_RX_EN;
    volatile Uint32 C5_TX_EN;
    volatile Uint32 C5_MISC_EN;
    volatile Uint32 C6_RX_THRESH_EN;
    volatile Uint32 C6_RX_EN;
    volatile Uint32 C6_TX_EN;
    volatile Uint32 C6_MISC_EN;
    volatile Uint32 C7_RX_THRESH_EN;
    volatile Uint32 C7_RX_EN;
    volatile Uint32 C7_TX_EN;
    volatile Uint32 C7_MISC_EN;
    volatile Uint32 C0_RX_THRESH_STAT;
    volatile Uint32 C0_RX_STAT;
    volatile Uint32 C0_TX_STAT;
    volatile Uint32 C0_MISC_STAT;
    volatile Uint32 C1_RX_THRESH_STAT;
    volatile Uint32 C1_RX_STAT;
    volatile Uint32 C1_TX_STAT;
    volatile Uint32 C1_MISC_STAT;
    volatile Uint32 C2_RX_THRESH_STAT;
    volatile Uint32 C2_RX_STAT;
    volatile Uint32 C2_TX_STAT;
    volatile Uint32 C2_MISC_STAT;
    volatile Uint32 C3_RX_THRESH_STAT;
    volatile Uint32 C3_RX_STAT;
    volatile Uint32 C3_TX_STAT;
    volatile Uint32 C3_MISC_STAT;
    volatile Uint32 C4_RX_THRESH_STAT;
    volatile Uint32 C4_RX_STAT;
    volatile Uint32 C4_TX_STAT;
    volatile Uint32 C4_MISC_STAT;
    volatile Uint32 C5_RX_THRESH_STAT;
    volatile Uint32 C5_RX_STAT;
    volatile Uint32 C5_TX_STAT;
    volatile Uint32 C5_MISC_STAT;
    volatile Uint32 C6_RX_THRESH_STAT;
    volatile Uint32 C6_RX_STAT;
    volatile Uint32 C6_TX_STAT;
    volatile Uint32 C6_MISC_STAT;
    volatile Uint32 C7_RX_THRESH_STAT;
    volatile Uint32 C7_RX_STAT;
    volatile Uint32 C7_TX_STAT;
    volatile Uint32 C7_MISC_STAT;
    volatile Uint32 C0_RX_IMAX;
    volatile Uint32 C0_TX_IMAX;
    volatile Uint32 C1_RX_IMAX;
    volatile Uint32 C1_TX_IMAX;
    volatile Uint32 C2_RX_IMAX;
    volatile Uint32 C2_TX_IMAX;
    volatile Uint32 C3_RX_IMAX;
    volatile Uint32 C3_TX_IMAX;
    volatile Uint32 C4_RX_IMAX;
    volatile Uint32 C4_TX_IMAX;
    volatile Uint32 C5_RX_IMAX;
    volatile Uint32 C5_TX_IMAX;
    volatile Uint32 C6_RX_IMAX;
    volatile Uint32 C6_TX_IMAX;
    volatile Uint32 C7_RX_IMAX;
    volatile Uint32 C7_TX_IMAX;
} CSL_EctlRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_EctlRegs             *CSL_EctlRegsOvly;


/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* IDVER */

#define CSL_ECTL_IDVER_CPGMACSS_S_IDENT_MASK (0xFFFF0000u)
#define CSL_ECTL_IDVER_CPGMACSS_S_IDENT_SHIFT (0x00000010u)
#define CSL_ECTL_IDVER_CPGMACSS_S_IDENT_RESETVAL (0x0000002Du)

#define CSL_ECTL_IDVER_CPGMACSS_S_RTL_VER_MASK (0x0000F800u)
#define CSL_ECTL_IDVER_CPGMACSS_S_RTL_VER_SHIFT (0x0000000Bu)
#define CSL_ECTL_IDVER_CPGMACSS_S_RTL_VER_RESETVAL (0x00000000u)

#define CSL_ECTL_IDVER_CPGMACSS_S_MAJ_VER_MASK (0x00000700u)
#define CSL_ECTL_IDVER_CPGMACSS_S_MAJ_VER_SHIFT (0x00000008u)
#define CSL_ECTL_IDVER_CPGMACSS_S_MAJ_VER_RESETVAL (0x00000001u)

#define CSL_ECTL_IDVER_CPGMACSS_S_MINOR_VER_MASK (0x000000FFu)
#define CSL_ECTL_IDVER_CPGMACSS_S_MINOR_VER_SHIFT (0x00000000u)
#define CSL_ECTL_IDVER_CPGMACSS_S_MINOR_VER_RESETVAL (0x00000002u)

#define CSL_ECTL_IDVER_RESETVAL (0x002D0102u)

/* SOFT_RESET */


#define CSL_ECTL_SOFT_RESET_SOFT_RESET_MASK (0x00000001u)
#define CSL_ECTL_SOFT_RESET_SOFT_RESET_SHIFT (0x00000000u)
#define CSL_ECTL_SOFT_RESET_SOFT_RESET_RESETVAL (0x00000000u)
/*----SOFT_RESET Tokens----*/
#define CSL_ECTL_SOFT_RESET_SOFT_RESET_YES (0x00000001u)
#define CSL_ECTL_SOFT_RESET_SOFT_RESET_NO (0x00000000u)

#define CSL_ECTL_SOFT_RESET_RESETVAL (0x00000000u)

/* EM_CONTROL */


#define CSL_ECTL_EM_CONTROL_SOFT_MASK (0x00000002u)
#define CSL_ECTL_EM_CONTROL_SOFT_SHIFT (0x00000001u)
#define CSL_ECTL_EM_CONTROL_SOFT_RESETVAL (0x00000000u)

#define CSL_ECTL_EM_CONTROL_FREE_MASK (0x00000001u)
#define CSL_ECTL_EM_CONTROL_FREE_SHIFT (0x00000000u)
#define CSL_ECTL_EM_CONTROL_FREE_RESETVAL (0x00000000u)

#define CSL_ECTL_EM_CONTROL_RESETVAL (0x00000000u)

/* INT_CONTROL */


#define CSL_ECTL_INT_CONTROL_INT_PACE_EN_MASK (0x003F0000u)
#define CSL_ECTL_INT_CONTROL_INT_PACE_EN_SHIFT (0x00000010u)
#define CSL_ECTL_INT_CONTROL_INT_PACE_EN_RESETVAL (0x00000000u)
/*----INT_PACE_EN Tokens----*/
#define CSL_ECTL_INT_CONTROL_INT_PACE_EN_ENABLE_RX (0x00000000u)
#define CSL_ECTL_INT_CONTROL_INT_PACE_EN_ENABLE_TX (0x00000001u)


#define CSL_ECTL_INT_CONTROL_INT_PRESCALE_MASK (0x00000FFFu)
#define CSL_ECTL_INT_CONTROL_INT_PRESCALE_SHIFT (0x00000000u)
#define CSL_ECTL_INT_CONTROL_INT_PRESCALE_RESETVAL (0x00000000u)

#define CSL_ECTL_INT_CONTROL_RESETVAL (0x00000000u)

/* RX_THRESH_EN */


#define CSL_ECTL_RX_THRESH_EN_RX_THRESH_EN_MASK (0x000000FFu)
#define CSL_ECTL_RX_THRESH_EN_RX_THRESH_EN_SHIFT (0x00000000u)
#define CSL_ECTL_RX_THRESH_EN_RX_THRESH_EN_RESETVAL (0x00000000u)

#define CSL_ECTL_RX_THRESH_EN_RESETVAL (0x00000000u)

/* RX_EN */


#define CSL_ECTL_RX_EN_RX_EN_MASK (0x000000FFu)
#define CSL_ECTL_RX_EN_RX_EN_SHIFT (0x00000000u)
#define CSL_ECTL_RX_EN_RX_EN_RESETVAL (0x00000000u)

#define CSL_ECTL_RX_EN_RESETVAL (0x00000000u)

/* TX_EN */


#define CSL_ECTL_TX_EN_TX_EN_MASK (0x000000FFu)
#define CSL_ECTL_TX_EN_TX_EN_SHIFT (0x00000000u)
#define CSL_ECTL_TX_EN_TX_EN_RESETVAL (0x00000000u)

#define CSL_ECTL_TX_EN_RESETVAL (0x00000000u)

/* MISC_EN */


#define CSL_ECTL_MISC_EN_MISC_EN_MASK (0x0000000Fu)
#define CSL_ECTL_MISC_EN_MISC_EN_SHIFT (0x00000000u)
#define CSL_ECTL_MISC_EN_MISC_EN_RESETVAL (0x00000000u)

#define CSL_ECTL_MISC_EN_RESETVAL (0x00000000u)

/* RX_THRESH_STAT */


#define CSL_ECTL_RX_THRESH_STAT_RX_THRESH_STAT_MASK (0x000000FFu)
#define CSL_ECTL_RX_THRESH_STAT_RX_THRESH_STAT_SHIFT (0x00000000u)
#define CSL_ECTL_RX_THRESH_STAT_RX_THRESH_STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_RX_THRESH_STAT_RESETVAL (0x00000000u)

/* RX_STAT */


#define CSL_ECTL_RX_STAT_RX_STAT_MASK (0x000000FFu)
#define CSL_ECTL_RX_STAT_RX_STAT_SHIFT (0x00000000u)
#define CSL_ECTL_RX_STAT_RX_STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_RX_STAT_RESETVAL (0x00000000u)

/* TX_STAT */


#define CSL_ECTL_TX_STAT_TX_STAT_MASK (0x000000FFu)
#define CSL_ECTL_TX_STAT_TX_STAT_SHIFT (0x00000000u)
#define CSL_ECTL_TX_STAT_TX_STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_TX_STAT_RESETVAL (0x00000000u)

/* MISC_STAT */


#define CSL_ECTL_MISC_STAT_MISC_STAT_MASK (0x0000000Fu)
#define CSL_ECTL_MISC_STAT_MISC_STAT_SHIFT (0x00000000u)
#define CSL_ECTL_MISC_STAT_MISC_STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_MISC_STAT_RESETVAL (0x00000000u)

/* RX_IMAX */


#define CSL_ECTL_RX_IMAX_RX_IMAX_MASK (0x0000003Fu)
#define CSL_ECTL_RX_IMAX_RX_IMAX_SHIFT (0x00000000u)
#define CSL_ECTL_RX_IMAX_RX_IMAX_RESETVAL (0x00000000u)

#define CSL_ECTL_RX_IMAX_RESETVAL (0x00000000u)

/* TX_IMAX */


#define CSL_ECTL_TX_IMAX_TX_IMAX_MASK (0x0000003Fu)
#define CSL_ECTL_TX_IMAX_TX_IMAX_SHIFT (0x00000000u)
#define CSL_ECTL_TX_IMAX_TX_IMAX_RESETVAL (0x00000000u)

#define CSL_ECTL_TX_IMAX_RESETVAL (0x00000000u)

#endif
