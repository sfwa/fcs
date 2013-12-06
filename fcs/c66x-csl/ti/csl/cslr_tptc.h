/********************************************************************
* Copyright (C) 2003-2008 Texas Instruments Incorporated.
* All Rights Reserved
*********************************************************************
* file: cslr_tptc.h
*
* Brief: This file contains the Register Description for tptc
*
*********************************************************************/
#ifndef _CSLR_TPTC_H_
#define _CSLR_TPTC_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified header file includes as per the RTSC specification
 *      b) Added the DST FIFO Regs and modified RL
 */
#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for CSL_TptcDstFifoRegs
\**************************************************************************/
typedef struct  {
    volatile Uint32 TPTC_DFOPT;
    volatile Uint8 RSVD0[4];
    volatile Uint32 TPTC_DFCNT;
    volatile Uint32 TPTC_DFDST;
    volatile Uint32 TPTC_DFBIDX;
    volatile Uint32 TPTC_DFMPPRXY;
    volatile Uint8 RSVD1[40];
} CSL_TptcDstFifoRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 TPCC_PID;
    volatile Uint32 TPTC_TCCFG;
    volatile Uint8 RSVD0[244];
    volatile Uint32 TPTC_CLKGDIS;
    volatile Uint32 TPTC_STAT;
    volatile Uint32 TPTC_INTSTAT;
    volatile Uint32 TPTC_INTEN;
    volatile Uint32 TPTC_INTCLR;
    volatile Uint32 TPTC_INTCMD;
    volatile Uint8 RSVD1[12];
    volatile Uint32 TPTC_ERRSTAT;
    volatile Uint32 TPTC_ERREN;
    volatile Uint32 TPTC_ERRCLR;
    volatile Uint32 TPTC_ERRDET;
    volatile Uint32 TPTC_ERRCMD;
    volatile Uint8 RSVD2[12];
    volatile Uint32 TPTC_RDRATE;
    volatile Uint8 RSVD3[188];
    volatile Uint32 TPTC_POPT;
    volatile Uint32 TPTC_PSRC;
    volatile Uint32 TPTC_PCNT;
    volatile Uint32 TPTC_PDST;
    volatile Uint32 TPTC_PBIDX;
    volatile Uint32 TPTC_PMPPRXY;
    volatile Uint8 RSVD4[40];
    volatile Uint32 TPTC_SAOPT;
    volatile Uint32 TPTC_SASRC;
    volatile Uint32 TPTC_SACNT;
    volatile Uint8 RSVD5[4];
    volatile Uint32 TPTC_SABIDX;
    volatile Uint32 TPTC_SAMPPRXY;
    volatile Uint32 TPTC_SACNTRLD;
    volatile Uint32 TPTC_SASRCBREF;
    volatile Uint8 RSVD6[32];
    volatile Uint32 TPTC_DFCNTRLD;
    volatile Uint32 TPTC_DFSRCBREF;
    volatile Uint32 TPTC_DFDSTBREF;
    volatile Uint8 RSVD7[116];
    CSL_TptcDstFifoRegs TPTC_DSTFIFO[4];
} CSL_TptcRegs;

#endif
