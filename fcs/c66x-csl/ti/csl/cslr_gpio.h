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
#ifndef _CSLR_GPIO_H_
#define _CSLR_GPIO_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for Bank_Registers
\**************************************************************************/
typedef struct  {
    volatile Uint32 DIR;
    volatile Uint32 OUT_DATA;
    volatile Uint32 SET_DATA;
    volatile Uint32 CLR_DATA;
    volatile Uint32 IN_DATA;
    volatile Uint32 SET_RIS_TRIG;
    volatile Uint32 CLR_RIS_TRIG;
    volatile Uint32 SET_FAL_TRIG;
    volatile Uint32 CLR_FAL_TRIG;
    volatile Uint32 INTSTAT;
} CSL_GpioBank_registersRegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 PID;
    volatile Uint32 PCR;
    volatile Uint32 BINTEN;
    volatile Uint8 RSVD0[4];
    CSL_GpioBank_registersRegs BANK_REGISTERS[4];
} CSL_GpioRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* DIR */

#define CSL_GPIO_DIR_DIR_MASK            (0x0000FFFFu)
#define CSL_GPIO_DIR_DIR_SHIFT           (0x00000000u)
#define CSL_GPIO_DIR_DIR_RESETVAL        (0x0000FFFFu)


#define CSL_GPIO_DIR_RESETVAL            (0x0000FFFFu)

/* OUT_DATA */

#define CSL_GPIO_OUT_DATA_OUT_MASK       (0x0000FFFFu)
#define CSL_GPIO_OUT_DATA_OUT_SHIFT      (0x00000000u)
#define CSL_GPIO_OUT_DATA_OUT_RESETVAL   (0x00000000u)


#define CSL_GPIO_OUT_DATA_RESETVAL       (0x00000000u)

/* SET_DATA */

#define CSL_GPIO_SET_DATA_SET_MASK       (0x0000FFFFu)
#define CSL_GPIO_SET_DATA_SET_SHIFT      (0x00000000u)
#define CSL_GPIO_SET_DATA_SET_RESETVAL   (0x00000000u)


#define CSL_GPIO_SET_DATA_RESETVAL       (0x00000000u)

/* CLR_DATA */

#define CSL_GPIO_CLR_DATA_CLR_MASK       (0x0000FFFFu)
#define CSL_GPIO_CLR_DATA_CLR_SHIFT      (0x00000000u)
#define CSL_GPIO_CLR_DATA_CLR_RESETVAL   (0x00000000u)


#define CSL_GPIO_CLR_DATA_RESETVAL       (0x00000000u)

/* IN_DATA */

#define CSL_GPIO_IN_DATA_IN_MASK         (0x0000FFFFu)
#define CSL_GPIO_IN_DATA_IN_SHIFT        (0x00000000u)
#define CSL_GPIO_IN_DATA_IN_RESETVAL     (0x00000000u)


#define CSL_GPIO_IN_DATA_RESETVAL        (0x00000000u)

/* SET_RIS_TRIG */

#define CSL_GPIO_SET_RIS_TRIG_SETRIS_MASK (0x0000FFFFu)
#define CSL_GPIO_SET_RIS_TRIG_SETRIS_SHIFT (0x00000000u)
#define CSL_GPIO_SET_RIS_TRIG_SETRIS_RESETVAL (0x00000000u)


#define CSL_GPIO_SET_RIS_TRIG_RESETVAL   (0x00000000u)

/* CLR_RIS_TRIG */

#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_MASK (0x0000FFFFu)
#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_SHIFT (0x00000000u)
#define CSL_GPIO_CLR_RIS_TRIG_CLRRIS_RESETVAL (0x00000000u)


#define CSL_GPIO_CLR_RIS_TRIG_RESETVAL   (0x00000000u)

/* SET_FAL_TRIG */

#define CSL_GPIO_SET_FAL_TRIG_SETFAL_MASK (0x0000FFFFu)
#define CSL_GPIO_SET_FAL_TRIG_SETFAL_SHIFT (0x00000000u)
#define CSL_GPIO_SET_FAL_TRIG_SETFAL_RESETVAL (0x00000000u)


#define CSL_GPIO_SET_FAL_TRIG_RESETVAL   (0x00000000u)

/* CLR_FAL_TRIG */

#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_MASK (0x0000FFFFu)
#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_SHIFT (0x00000000u)
#define CSL_GPIO_CLR_FAL_TRIG_CLRFAL_RESETVAL (0x00000000u)


#define CSL_GPIO_CLR_FAL_TRIG_RESETVAL   (0x00000000u)

/* INTSTAT */

#define CSL_GPIO_INTSTAT_STAT_MASK       (0x0000FFFFu)
#define CSL_GPIO_INTSTAT_STAT_SHIFT      (0x00000000u)
#define CSL_GPIO_INTSTAT_STAT_RESETVAL   (0x00000000u)


#define CSL_GPIO_INTSTAT_RESETVAL        (0x00000000u)

/* PID */

#define CSL_GPIO_PID_MINOR_MASK          (0x0000003Fu)
#define CSL_GPIO_PID_MINOR_SHIFT         (0x00000000u)
#define CSL_GPIO_PID_MINOR_RESETVAL      (0x00000005u)

#define CSL_GPIO_PID_CUSTOM_MASK         (0x000000C0u)
#define CSL_GPIO_PID_CUSTOM_SHIFT        (0x00000006u)
#define CSL_GPIO_PID_CUSTOM_RESETVAL     (0x00000000u)

#define CSL_GPIO_PID_MAJOR_MASK          (0x00000700u)
#define CSL_GPIO_PID_MAJOR_SHIFT         (0x00000008u)
#define CSL_GPIO_PID_MAJOR_RESETVAL      (0x00000001u)

#define CSL_GPIO_PID_RTL_MASK            (0x0000F800u)
#define CSL_GPIO_PID_RTL_SHIFT           (0x0000000Bu)
#define CSL_GPIO_PID_RTL_RESETVAL        (0x00000002u)

#define CSL_GPIO_PID_FUNC_MASK           (0x0FFF0000u)
#define CSL_GPIO_PID_FUNC_SHIFT          (0x00000010u)
#define CSL_GPIO_PID_FUNC_RESETVAL       (0x00000483u)


#define CSL_GPIO_PID_SCHEME_MASK         (0xC0000000u)
#define CSL_GPIO_PID_SCHEME_SHIFT        (0x0000001Eu)
#define CSL_GPIO_PID_SCHEME_RESETVAL     (0x00000001u)

#define CSL_GPIO_PID_RESETVAL            (0x44831105u)

/* PCR */

#define CSL_GPIO_PCR_FREE_MASK           (0x00000001u)
#define CSL_GPIO_PCR_FREE_SHIFT          (0x00000000u)
#define CSL_GPIO_PCR_FREE_RESETVAL       (0x00000001u)

#define CSL_GPIO_PCR_SOFT_MASK           (0x00000002u)
#define CSL_GPIO_PCR_SOFT_SHIFT          (0x00000001u)
#define CSL_GPIO_PCR_SOFT_RESETVAL       (0x00000000u)


#define CSL_GPIO_PCR_RESETVAL            (0x00000001u)

/* BINTEN */

#define CSL_GPIO_BINTEN_EN_MASK          (0x000000FFu)
#define CSL_GPIO_BINTEN_EN_SHIFT         (0x00000000u)
#define CSL_GPIO_BINTEN_EN_RESETVAL      (0x00000000u)


#define CSL_GPIO_BINTEN_RESETVAL         (0x00000000u)

#endif

