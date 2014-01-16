/*  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2002, 2003, 2004, 2005, 2006, 2008
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
 *   @file  cslr.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  This file contains the macro definations for Register layer
 *
 */

/* Register layer central -- contains field-manipulation macro definitions */

#ifndef _CSLR_H_
#define _CSLR_H_

/* the "expression" macros */

/* the Field MaKe macro */
#define CSL_FMK(PER_REG_FIELD, val)                                         \
    (((val) << CSL_##PER_REG_FIELD##_SHIFT) & CSL_##PER_REG_FIELD##_MASK)

/* the Field EXTract macro */
#define CSL_FEXT(reg, PER_REG_FIELD)                                        \
    (((reg) & CSL_##PER_REG_FIELD##_MASK) >> CSL_##PER_REG_FIELD##_SHIFT)

/* the Field INSert macro */
#define CSL_FINS(reg, PER_REG_FIELD, val)                                   \
    ((reg) = ((reg) & ~CSL_##PER_REG_FIELD##_MASK)                          \
    | CSL_FMK(PER_REG_FIELD, val))


/* the "token" macros */

/* the Field MaKe (Token) macro */
#define CSL_FMKT(PER_REG_FIELD, TOKEN)                                      \
    CSL_FMK(PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)

/* the Field INSert (Token) macro */
#define CSL_FINST(reg, PER_REG_FIELD, TOKEN)                                \
    CSL_FINS((reg), PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)


/* the "raw" macros */

/* the Field MaKe (Raw) macro */
#define CSL_FMKR(msb, lsb, val)                                             \
    (((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))

/* the Field EXTract (Raw) macro */
#define CSL_FEXTR(reg, msb, lsb)                                            \
    (((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))

/* the Field INSert (Raw) macro */
#define CSL_FINSR(reg, msb, lsb, val)                                       \
    ((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))         \
    | CSL_FMKR(msb, lsb, val))

#endif /* _CSLR_H_ */

