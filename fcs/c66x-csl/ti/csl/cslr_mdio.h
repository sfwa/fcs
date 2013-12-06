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

/** ============================================================================
 *   @file  cslr_mdio.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  This file contains the Register Desciptions for MDIO
 *
 */

#ifndef _CSLR_MDIO_H_
#define _CSLR_MDIO_H_

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
    volatile Uint32 VERSION;
    volatile Uint32 CONTROL;
    volatile Uint32 ALIVE;
    volatile Uint32 LINK;
    volatile Uint32 LINKINTRAW;
    volatile Uint32 LINKINTMASKED;
    volatile Uint8 RSVD0[8];
    volatile Uint32 USERINTRAW;
    volatile Uint32 USERINTMASKED;
    volatile Uint32 USERINTMASKSET;
    volatile Uint32 USERINTMASKCLEAR;
    volatile Uint8 RSVD1[80];
    volatile Uint32 USERACCESS0;
    volatile Uint32 USERPHYSEL0;
    volatile Uint32 USERACCESS1;
    volatile Uint32 USERPHYSEL1;
} CSL_MdioRegs;

#endif
