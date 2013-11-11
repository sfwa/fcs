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
#ifndef _CSLR_TCP3D_DMA_OFFSETS_H_
#define _CSLR_TCP3D_DMA_OFFSETS_H_

/* Minimum unit = 1 byte */

/* TCP3D DMA MEMORY OFFSETS */

/* Input Config Register Memory offsets */
#define CSL_TCP3D_DMA_TCP3D_IC_CFG0_P0_OFFSET       (0x000000u)
#define CSL_TCP3D_DMA_TCP3D_OUT_STS0_P0_OFFSET      (0x000100u)

#define CSL_TCP3D_DMA_TCP3D_IC_CFG0_P1_OFFSET       (0x000200u)
#define CSL_TCP3D_DMA_TCP3D_OUT_STS0_P1_OFFSET      (0x000300u)

/* Input Memory offsets */
#define CSL_TCP3D_DMA_TCP3D_SYS_P0_OFFSET           (0x010000u)
#define CSL_TCP3D_DMA_TCP3D_PAR0_P0_OFFSET          (0x012000u)
#define CSL_TCP3D_DMA_TCP3D_PAR1_P0_OFFSET          (0x014000u)
#define CSL_TCP3D_DMA_TCP3D_INTER_P0_OFFSET         (0x020000u)

#define CSL_TCP3D_DMA_TCP3D_SYS_P1_OFFSET           (0x016000u)
#define CSL_TCP3D_DMA_TCP3D_PAR0_P1_OFFSET          (0x018000u)
#define CSL_TCP3D_DMA_TCP3D_PAR1_P1_OFFSET          (0x01A000u)
#define CSL_TCP3D_DMA_TCP3D_INTER_P1_OFFSET         (0x024000u)

/* Output Memory offsets */
#define CSL_TCP3D_DMA_TCP3D_OUT_HD0_OFFSET          (0x030000u)
#define CSL_TCP3D_DMA_TCP3D_OUT_HD1_OFFSET          (0x031000u)

#define CSL_TCP3D_DMA_TCP3D_OUT_SO0_OFFSET          (0x080000u)
#define CSL_TCP3D_DMA_TCP3D_OUT_SO1_OFFSET          (0x082000u)
#define CSL_TCP3D_DMA_TCP3D_OUT_SO2_OFFSET          (0x084000u)

#endif
