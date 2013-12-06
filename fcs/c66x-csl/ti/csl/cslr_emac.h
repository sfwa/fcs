/*
 * Copyright (C) 2002-2012 Texas Instruments Incorporated - http://www.ti.com/
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

/** ============================================================================
 *   @file  cslr_emac.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  This file contains the Register Desciptions for EMAC
 *
 */
#ifndef _CSLR_EMAC_H_
#define _CSLR_EMAC_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a)  Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 TXIDVER;
    volatile Uint32 TXCONTROL;
    volatile Uint32 TXTEARDOWN;
    volatile Uint8 RSVD0[4];
    volatile Uint32 RXIDVER;
    volatile Uint32 RXCONTROL;
    volatile Uint32 RXTEARDOWN;
    volatile Uint8 RSVD1[100];
    volatile Uint32 TXINTSTATRAW;
    volatile Uint32 TXINTSTATMASKED;
    volatile Uint32 TXINTMASKSET;
    volatile Uint32 TXINTMASKCLEAR;
    volatile Uint32 MACINVECTOR;
    volatile Uint32 MACEOIVECTOR;
    volatile Uint8 RSVD2[8];
    volatile Uint32 RXINTSTATRAW;
    volatile Uint32 RXINTSTATMASKED;
    volatile Uint32 RXINTMASKSET;
    volatile Uint32 RXINTMASKCLEAR;
    volatile Uint32 MACINTSTATRAW;
    volatile Uint32 MACINTSTATMASKED;
    volatile Uint32 MACINTMASKSET;
    volatile Uint32 MACINTMASKCLEAR;
    volatile Uint8 RSVD3[64];
    volatile Uint32 RXMBPENABLE;
    volatile Uint32 RXUNICASTSET;
    volatile Uint32 RXUNICASTCLEAR;
    volatile Uint32 RXMAXLEN;
    volatile Uint32 RXBUFFEROFFSET;
    volatile Uint32 RXFILTERLOWTHRESH;
    volatile Uint8 RSVD4[8];
    volatile Uint32 RX0FLOWTHRESH;
    volatile Uint32 RX1FLOWTHRESH;
    volatile Uint32 RX2FLOWTHRESH;
    volatile Uint32 RX3FLOWTHRESH;
    volatile Uint32 RX4FLOWTHRESH;
    volatile Uint32 RX5FLOWTHRESH;
    volatile Uint32 RX6FLOWTHRESH;
    volatile Uint32 RX7FLOWTHRESH;
    volatile Uint32 RX0FREEBUFFER;
    volatile Uint32 RX1FREEBUFFER;
    volatile Uint32 RX2FREEBUFFER;
    volatile Uint32 RX3FREEBUFFER;
    volatile Uint32 RX4FREEBUFFER;
    volatile Uint32 RX5FREEBUFFER;
    volatile Uint32 RX6FREEBUFFER;
    volatile Uint32 RX7FREEBUFFER;
    volatile Uint32 MACCONTROL;
    volatile Uint32 MACSTATUS;
    volatile Uint32 EMCONTROL;
    volatile Uint32 FIFOCONTROL;
    volatile Uint32 MACCONFIG;
    volatile Uint32 SOFTRESET;
    volatile Uint8 RSVD5[88];
    volatile Uint32 MACSRCADDRLO;
    volatile Uint32 MACSRCADDRHI;
    volatile Uint32 MACHASH1;
    volatile Uint32 MACHASH2;
    volatile Uint32 BOFFTEST;
    volatile Uint32 TPACETEST;
    volatile Uint32 RXPAUSE;
    volatile Uint32 TXPAUSE;
    volatile Uint8 RSVD6[16];
    volatile Uint32 RXGOODFRAMES;
    volatile Uint32 RXBCASTFRAMES;
    volatile Uint32 RXMCASTFRAMES;
    volatile Uint32 RXPAUSEFRAMES;
    volatile Uint32 RXCRCERRORS;
    volatile Uint32 RXALIGNCODEERRORS;
    volatile Uint32 RXOVERSIZED;
    volatile Uint32 RXJABBER;
    volatile Uint32 RXUNDERSIZED;
    volatile Uint32 RXFRAGMENTS;
    volatile Uint32 RXFILTERED;
    volatile Uint32 RXQOSFILTERED;
    volatile Uint32 RXOCTETS;
    volatile Uint32 TXGOODFRAMES;
    volatile Uint32 TXBCASTFRAMES;
    volatile Uint32 TXMCASTFRAMES;
    volatile Uint32 TXPAUSEFRAMES;
    volatile Uint32 TXDEFERRED;
    volatile Uint32 TXCOLLISION;
    volatile Uint32 TXSINGLECOLL;
    volatile Uint32 TXMULTICOLL;
    volatile Uint32 TXEXCESSIVECOLL;
    volatile Uint32 TXLATECOLL;
    volatile Uint32 TXUNDERRUN;
    volatile Uint32 TXCARRIERSENSE;
    volatile Uint32 TXOCTETS;
    volatile Uint32 FRAME64;
    volatile Uint32 FRAME65T127;
    volatile Uint32 FRAME128T255;
    volatile Uint32 FRAME256T511;
    volatile Uint32 FRAME512T1023;
    volatile Uint32 FRAME1024TUP;
    volatile Uint32 NETOCTETS;
    volatile Uint32 RXSOFOVERRUNS;
    volatile Uint32 RXMOFOVERRUNS;
    volatile Uint32 RXDMAOVERRUNS;
    volatile Uint8 RSVD7[624];
    volatile Uint32 MACADDRLO;
    volatile Uint32 MACADDRHI;
    volatile Uint32 MACINDEX;
    volatile Uint8 RSVD8[244];
    volatile Uint32 TX0HDP;
    volatile Uint32 TX1HDP;
    volatile Uint32 TX2HDP;
    volatile Uint32 TX3HDP;
    volatile Uint32 TX4HDP;
    volatile Uint32 TX5HDP;
    volatile Uint32 TX6HDP;
    volatile Uint32 TX7HDP;
    volatile Uint32 RX0HDP;
    volatile Uint32 RX1HDP;
    volatile Uint32 RX2HDP;
    volatile Uint32 RX3HDP;
    volatile Uint32 RX4HDP;
    volatile Uint32 RX5HDP;
    volatile Uint32 RX6HDP;
    volatile Uint32 RX7HDP;
    volatile Uint32 TX0CP;
    volatile Uint32 TX1CP;
    volatile Uint32 TX2CP;
    volatile Uint32 TX3CP;
    volatile Uint32 TX4CP;
    volatile Uint32 TX5CP;
    volatile Uint32 TX6CP;
    volatile Uint32 TX7CP;
    volatile Uint32 RX0CP;
    volatile Uint32 RX1CP;
    volatile Uint32 RX2CP;
    volatile Uint32 RX3CP;
    volatile Uint32 RX4CP;
    volatile Uint32 RX5CP;
    volatile Uint32 RX6CP;
    volatile Uint32 RX7CP;
} CSL_EmacRegs;


/***********************************************************************
 * EMAC Descriptor
 *
 * The following is the format of a single buffer descriptor
 * on the EMAC.
 ***********************************************************************/
typedef struct _EMAC_Desc {
  struct _EMAC_Desc *pNext;     /* Pointer to next descriptor in chain */
  Uint8             *pBuffer;   /* Pointer to data buffer              */
  Uint32            BufOffLen;  /* Buffer Offset(MSW) and Length(LSW)  */
  Uint32            PktFlgLen;  /* Packet Flags(MSW) and Length(LSW)   */
} EMAC_Desc;


/* ------------------------ */
/* DESCRIPTOR ACCESS MACROS */
/* ------------------------ */

/* Packet Flags */
#define EMAC_DSC_FLAG_SOP                       0x80000000u
#define EMAC_DSC_FLAG_EOP                       0x40000000u
#define EMAC_DSC_FLAG_OWNER                     0x20000000u
#define EMAC_DSC_FLAG_EOQ                       0x10000000u
#define EMAC_DSC_FLAG_TDOWNCMPLT                0x08000000u
#define EMAC_DSC_FLAG_PASSCRC                   0x04000000u

/* The following flags are RX only */
#define EMAC_DSC_FLAG_JABBER                    0x02000000u
#define EMAC_DSC_FLAG_OVERSIZE                  0x01000000u
#define EMAC_DSC_FLAG_FRAGMENT                  0x00800000u
#define EMAC_DSC_FLAG_UNDERSIZED                0x00400000u
#define EMAC_DSC_FLAG_CONTROL                   0x00200000u
#define EMAC_DSC_FLAG_OVERRUN                   0x00100000u
#define EMAC_DSC_FLAG_CODEERROR                 0x00080000u
#define EMAC_DSC_FLAG_ALIGNERROR                0x00040000u
#define EMAC_DSC_FLAG_CRCERROR                  0x00020000u
#define EMAC_DSC_FLAG_NOMATCH                   0x00010000u

#endif
