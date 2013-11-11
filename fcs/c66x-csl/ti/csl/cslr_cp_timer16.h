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
#ifndef _CSLR_CP_TIMER16_H_
#define _CSLR_CP_TIMER16_H_

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 TIMER_CNTRL_REG;
    volatile Uint32 TIMER_LOAD_REG;
    volatile Uint32 TIMER_VALUE_REG;
    volatile Uint32 TIMER_IRQ_REG;
} CSL_Cp_timer16Regs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* TIMER_CNTRL_REG */

#define CSL_CP_TIMER16_TIMER_CNTRL_REG_GO_MASK (0x00000001u)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_GO_SHIFT (0x00000000u)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_GO_RESETVAL (0x00000000u)

#define CSL_CP_TIMER16_TIMER_CNTRL_REG_MODE_MASK (0x00000002u)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_MODE_SHIFT (0x00000001u)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_MODE_RESETVAL (0x00000000u)

#define CSL_CP_TIMER16_TIMER_CNTRL_REG_PRESCALE_MASK (0x0000003Cu)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_PRESCALE_SHIFT (0x00000002u)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_PRESCALE_RESETVAL (0x00000000u)

#define CSL_CP_TIMER16_TIMER_CNTRL_REG_PSE_MASK (0x00008000u)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_PSE_SHIFT (0x0000000Fu)
#define CSL_CP_TIMER16_TIMER_CNTRL_REG_PSE_RESETVAL (0x00000000u)

#define CSL_CP_TIMER16_TIMER_CNTRL_REG_RESETVAL (0x00000000u)

/* TIMER_LOAD_REG */

#define CSL_CP_TIMER16_TIMER_LOAD_REG_COUNT_MASK (0x0000FFFFu)
#define CSL_CP_TIMER16_TIMER_LOAD_REG_COUNT_SHIFT (0x00000000u)
#define CSL_CP_TIMER16_TIMER_LOAD_REG_COUNT_RESETVAL (0x0000FFFFu)

#define CSL_CP_TIMER16_TIMER_LOAD_REG_RESETVAL (0x0000FFFFu)

/* TIMER_VALUE_REG */

#define CSL_CP_TIMER16_TIMER_VALUE_REG_CURR_COUNT_MASK (0x0000FFFFu)
#define CSL_CP_TIMER16_TIMER_VALUE_REG_CURR_COUNT_SHIFT (0x00000000u)
#define CSL_CP_TIMER16_TIMER_VALUE_REG_CURR_COUNT_RESETVAL (0x0000FFFFu)

#define CSL_CP_TIMER16_TIMER_VALUE_REG_RESETVAL (0x0000FFFFu)

/* TIMER_IRQ_REG */

#define CSL_CP_TIMER16_TIMER_IRQ_REG_INTERRUPT_MASK (0x00000001u)
#define CSL_CP_TIMER16_TIMER_IRQ_REG_INTERRUPT_SHIFT (0x00000000u)
#define CSL_CP_TIMER16_TIMER_IRQ_REG_INTERRUPT_RESETVAL (0x00000000u)

#define CSL_CP_TIMER16_TIMER_IRQ_REG_RESETVAL (0x00000000u)

#endif
