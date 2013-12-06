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

#endif
