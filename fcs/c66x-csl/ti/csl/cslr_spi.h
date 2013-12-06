/*
 * cslr_spi.h
 *
 * This file contains the macros for Register Chip Support Library (CSL) which
 * can be used for operations on the respective underlying hardware/peripheral
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
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
 *   @file  cslr_spi.h
 *
 *   @path  $(CSLPATH)
 *
 *   @desc  This file contains the Register Descriptions for SPI
 *
 *  ============================================================================
 */
#ifndef _CSLR_SPI_H_
#define _CSLR_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/cslr.h>
#include <ti/csl/tistdtypes.h>


/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 SPIGCR0;
    volatile Uint32 SPIGCR1;
    volatile Uint32 SPIINT0;
    volatile Uint32 SPILVL;
    volatile Uint32 SPIFLG;
    volatile Uint32 SPIPC0;
    volatile Uint32 SPIPC1;
    volatile Uint32 SPIPC2;
    volatile Uint32 SPIPC3;
    volatile Uint32 SPIPC4;
    volatile Uint32 SPIPC5;
    volatile Uint8 RSVD0[12];
    volatile Uint32 SPIDAT0;
    volatile Uint32 SPIDAT1;
    volatile Uint32 SPIBUF;
    volatile Uint32 SPIEMU;
    volatile Uint32 SPIDELAY;
    volatile Uint32 SPIDEF;
    volatile Uint32 SPIFMT[4];
    volatile Uint32 INTVEC[2];
} CSL_SpiRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_SpiRegs         *CSL_SpiRegsOvly;

#ifdef __cplusplus
}
#endif

#endif
