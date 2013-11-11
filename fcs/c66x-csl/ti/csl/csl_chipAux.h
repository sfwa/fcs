/* ============================================================================
 * Copyright (c) Texas Instruments Incorporated 2008, 2009
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

/** 
 * @file csl_chipAux.h
 *
 * @brief 
 *  API Auxilary header file for CHIP CSL to read/write from the C64x+ CPU
 *  Control Register File.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par  
 */

#ifndef _CSL_CHIPAUX_H_
#define _CSL_CHIPAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_chip.h>

/** @addtogroup CSL_CHIP_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_chipReadAMR
 *
 *   @b Description
 *   @n This API reads the Addressing Mode (AMR) control register and returns its
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n AMR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 amrVal;
        amrVal = CSL_chipReadAMR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadAMR()
{
    /** Addressing Mode Register */
    return AMR;            
}

/** ============================================================================
 *   @n@b CSL_chipReadCSR
 *
 *   @b Description
 *   @n This API reads the Control Status (CSR) register and returns its
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CSR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 csrVal;
        csrVal = CSL_chipReadCSR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadCSR()
{
    /** Control Status Register */                
    return CSR;    
}

/** ============================================================================
 *   @n@b CSL_chipReadIFR
 *
 *   @b Description
 *   @n This API reads the Interrrupt Flag register (IFR) and returns its
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>   Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n IFR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 ifrVal;
        ifrVal = CSL_chipReadIFR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadIFR()
{
    /** Interrupt Flag Register */
    return IFR;      
}

/** ============================================================================
 *   @n@b CSL_chipReadIER
 *
 *   @b Description
 *   @n This API reads the Interrrupt Enable register (IER) and returns its
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n IER
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 ierVal;
        ierVal = CSL_chipReadIER ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadIER()
{
    /** Interrupt Enable Register */
    return IER;              
}

/** ============================================================================
 *   @n@b CSL_chipReadISTP
 *
 *   @b Description
 *   @n This API reads the Interrrupt Service Table Pointer register (ISTP) and 
 *      returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value   </b> Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ISTP
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 istpVal;
        istpVal = CSL_chipReadISTP ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadISTP()
{
    /** Interrupt Service Table Pointer Register */
    return ISTP;
}

/** ============================================================================
 *   @n@b CSL_chipReadIRP
 *
 *   @b Description
 *   @n This API reads the Interrrupt Return Pointer register (IRP) and 
 *      returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n IRP
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 irpVal;
        irpVal = CSL_chipReadIRP ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadIRP()
{
    /** Interrupt Return Pointer Register */                
    return IRP;           
}

/** ============================================================================
 *   @n@b CSL_chipReadNRP
 *
 *   @b Description
 *   @n This API reads the Nonmaskable Interrupt Return Pointer register (NRP) 
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n NRP
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 nrpVal;
        nrpVal = CSL_chipReadNRP ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadNRP()
{
    /** Nonmaskable Interrupt (NMI) Registers */                
    return NRP;               
}

/** ============================================================================
 *   @n@b CSL_chipReadERP
 *
 *   @b Description
 *   @n This API reads the Exception Return Pointer register (ERP) and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n NRP
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 erpVal;
        erpVal = CSL_chipReadERP ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadERP()
{
    /** Exception Return Pointer Registers */                
    return NRP;              
}

/** ============================================================================
 *   @n@b CSL_chipReadTSCL
 *
 *   @b Description
 *   @n This API reads the Time Stamp Counter Lower Order 32-bits (TSCL) register
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_chipWriteTSCL() must be called with any arbitrary value to
 *      enable the TSC first before this API is called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n TSCL
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 tsclVal;
        tsclVal = CSL_chipReadTSCL ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadTSCL()
{
    /** Time Stamp Counter Register - Low */
    return TSCL;
}

/** ============================================================================
 *   @n@b CSL_chipReadTSCH
 *
 *   @b Description
 *   @n This API reads the Time Stamp Counter Higher Order 32-bits (TSCH) register
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value   </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n @a CSL_chipWriteTSCL() must be called with any arbitrary value to
 *      enable the TSC first before this API is called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n TSCH
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 tschVal;
        tschVal = CSL_chipReadTSCH ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadTSCH()
{
    /** Time Stamp Counter Registers - High */
    return TSCH;
}

/** ============================================================================
 *   @n@b CSL_chipReadARP
 *
 *   @b Description
 *   @n This API reads the Analysis Return Pointer (ARP) register
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ARP
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 arpVal;
        arpVal = CSL_chipReadARP ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadARP()
{
    /** Analysis Return Pointer */
    return ARP;
}

/** ============================================================================
 *   @n@b CSL_chipReadILC
 *
 *   @b Description
 *   @n This API reads the Inner Loop SPL Buffer Count (ILC) register
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ILC
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 ilcVal;
        ilcVal = CSL_chipReadILC ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadILC()
{
    /** SPLOOP Inner Loop Count Register */
    return ILC;               
}


/** ============================================================================
 *   @n@b CSL_chipReadRILC
 *
 *   @b Description
 *   @n This API reads the Reload Inner Loop SPL Buffer Count (RILC) register
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n RILC
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 rilcVal;
        rilcVal = CSL_chipReadRILC ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadRILC()
{
    /** SPLOOP Reload Inner Loop Count Register */                
    return RILC;             
}


/** ============================================================================
 *   @n@b CSL_chipReadREP
 *
 *   @b Description
 *   @n This API reads the Restricted Entry Point Address (REP) register
 *      and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n REP
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 repVal;
        repVal = CSL_chipReadREP ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadREP()
{
    /** Restricted Entry Point Address Register */
    return REP;             
}

/** ============================================================================
 *   @n@b CSL_chipReadPCE1
 *
 *   @b Description
 *   @n This API reads the Program Counter, E1 Phase (PCE1) register and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PCE1
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 pce1Val;
        pce1Val = CSL_chipReadPCE1 ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadPCE1()
{
    /** E1 Phase Program Counter */
    return PCE1;                
}


/** ============================================================================
 *   @n@b CSL_chipReadDNUM
 *
 *   @b Description
 *   @n This API reads the DSP Core Number (DSPNUM) register and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n DNUM
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 dnumVal;
        dnumVal = CSL_chipReadDNUM ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadDNUM()
{
    /** DSP Core Number Register */
    return DNUM;              
}

/** ============================================================================
 *   @n@b CSL_chipReadSSR
 *
 *   @b Description
 *   @n This API reads the Saturation Status Register (SSR) and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n SSR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 ssrVal;
        ssrVal = CSL_chipReadSSR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadSSR()
{
    /** Saturation Status Register */
    return SSR;               
}

/** ============================================================================
 *   @n@b CSL_chipReadGPLYA
 *
 *   @b Description
 *   @n This API reads the GMPY A-side polynomial Register (GPLYA) and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n GPLYA
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 gplyaVal;
        gplyaVal = CSL_chipReadGPLYA ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadGPLYA()
{
    /** GMPY Polynomial.A Side Register */
    return GPLYA;              
}

/** ============================================================================
 *   @n@b CSL_chipReadGPLYB
 *
 *   @b Description
 *   @n This API reads the GMPY B-side polynomial Register (GPLYB) and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n GPLYB
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 gplybVal;
        gplybVal = CSL_chipReadGPLYB ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadGPLYB()
{
    /** GMPY Polynomial.B Side Register */
    return GPLYB;                
}


/** ============================================================================
 *   @n@b CSL_chipReadGFPGFR
 *
 *   @b Description
 *   @n This API reads the Galios Field Multiply Control Register (GFPGFR) and 
 *      returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n GFPGFR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 gfpgfrVal;
        gfpgfrVal = CSL_chipReadGFPGFR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadGFPGFR()
{
    /** Galois Field Polynomial Generator Function Register */
    return GFPGFR;             
}


/** ============================================================================
 *   @n@b CSL_chipReadDIER
 *
 *   @b Description
 *   @n This API reads the Debug Interrupt Enable Register (DIER) and returns its 
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value   </b> Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n DIER
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 dierVal;
        dierVal = CSL_chipReadDIER ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadDIER()
{
    /** Debug Interrupt Enable Register */
    return DIER;               
}


/** ============================================================================
 *   @n@b CSL_chipReadTSR
 *
 *   @b Description
 *   @n This API reads the Task State Register (TSR) and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value   </b> Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n TSR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 tsrVal;
        tsrVal = CSL_chipReadTSR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadTSR()
{
    /** Task State Register */
    return TSR;               
}


/** ============================================================================
 *   @n@b CSL_chipReadITSR
 *
 *   @b Description
 *   @n This API reads the Interrupt Task State Register (ITSR) and returns its 
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b> Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ITSR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 itsrVal;
        itsrVal = CSL_chipReadITSR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadITSR()
{
    /** Interrupt Task State Register */
    return ITSR;                
}

/** ============================================================================
 *   @n@b CSL_chipReadNTSR
 *
 *   @b Description
 *   @n This API reads the NMI/Exception Task State Register (NTSR) and returns its 
 *      contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n NTSR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 ntsrVal;
        ntsrVal = CSL_chipReadNTSR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadNTSR()
{
    /** NMI/Exception Task State Register */
    return NTSR;              
}


/** ============================================================================
 *   @n@b CSL_chipReadEFR
 *
 *   @b Description
 *   @n This API reads the Exception Flag Register (EFR) and returns its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b> Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n EFR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 efrVal;
        efrVal = CSL_chipReadEFR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadEFR()
{
    /** Exception Flag Register */
    return EFR;                
}


/** ============================================================================
 *   @n@b CSL_chipReadIERR
 *
 *   @b Description
 *   @n This API reads the Internal Exception Report Register (IERR) and returns 
 *      its contents.
 *
 *   @b Arguments   None
 *
 *   <b> Return Value  </b>  Uint32
 *   @li            The control register value read
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n IERR
 *
 *  <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 ierrVal;
        ierrVal = CSL_chipReadIERR ();

    @endverbatim
 * ===========================================================================
 */        
CSL_IDEF_INLINE Uint32  CSL_chipReadIERR()
{
    /** Internal Exception Report Register */
    return IERR;
}


/** ============================================================================
 *   @n@b CSL_chipWriteAMR
 *
 *   @b Description
 *   @n This API configures the Addressing Mode Register (AMR) with the specified 
 *      new value 'val'. This API returns the old AMR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *
 *   <b> Return Value   </b>  Uint32
 *      @li         New programmed value
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The AMR control register is written with the value passed
 *
 *   @b Writes
 *   @n AMR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldamr;
        oldamr = CSL_chipWriteAMR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteAMR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Addressing Mode Register */
    oldVal  = AMR;
    AMR     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteCSR
 *
 *   @b Description
 *   @n This API configures the Control Status Register (CSR) with the specified 
 *      new value 'val'. This API returns the old CSR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b> Uint32
 *      @li         New programmed value
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The CSR control register is written with the value passed
 *
 *   @b Writes
 *   @n CSR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldcsr;
        oldcsr = CSL_chipWriteCSR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteCSR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Control Status Register */                
    oldVal  = CSR;
    CSR     = val;

    return oldVal;            
}

/** ============================================================================
 *   @n@b CSL_chipWriteISR
 *
 *   @b Description
 *   @n This API configures the Interrupt Set Register (ISR) with the specified 
 *      new value 'val'. This API returns the old ISR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b> Uint32
 *      @li         New programmed value
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ISR control register is written with the value passed
 *
 *   @b Writes
 *   @n ISR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldISR;
        oldISR = CSL_chipWriteISR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteISR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Read Interrupt Flag Register and write to the Interrupt Set Register */
    oldVal  = IFR;
    ISR     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteICR
 *
 *   @b Description
 *   @n This API writes to the Interrupt Clear Register (ICR) with the specified 
 *      new value 'val' to clear the maskable interrupts configured in the IFR
 *      register. This API returns the old interrupt state from IFR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from IFR
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ICR control register is written with the value passed
 *
 *   @b Writes
 *   @n ICR
 *
 *   @b Affects
 *   @n IFR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldIFR;
        oldIFR = CSL_chipWriteICR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteICR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /* Reads the old interrupt set status from the Interrupt Flag Register and 
     * clears the interrupts specified by writing to Interrupt Clear Register 
     */
    oldVal  = IFR;
    ICR     = val;

    return oldVal;
}

/** ============================================================================
 *   @n@b CSL_chipWriteIER
 *
 *   @b Description
 *   @n This API writes to the Interrupt Enable Register (IER) with the specified 
 *      new value 'val' to enable/disable individual interrupts in the system. 
 *      This API returns the old interrupt enable state from IER value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from IER
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The IER control register is written with the value passed
 *
 *   @b Writes
 *   @n IER
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldIER;
        oldIER = CSL_chipWriteIER (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteIER(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Interrupt Enable Register */
    oldVal  = IER;
    IER     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteISTP
 *
 *   @b Description
 *   @n This API writes to the Interrupt Service Table Pointer Register (ISTP) 
 *      with the specified value 'val' to setup a ISR base address for a given 
 *      interrupt in the ISTB. This API returns the old ISTP value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b> Uint32
 *      @li         New programmed value from ISTP
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ISTP control register is written with the value passed
 *
 *   @b Writes
 *   @n ISTP
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldISTP;
        oldISTP = CSL_chipWriteISTP (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteISTP(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Interrupt Service Table Pointer Register */
    oldVal  = ISTP;
    ISTP    = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteIRP
 *
 *   @b Description
 *   @n This API writes to the Interrupt Return Pointer Register (IRP) 
 *      with the specified value 'val' to setup a return point in the program to
 *      continue execution after a maskable interrupt is executed. This API 
 *      returns the old IRP value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b> Uint32
 *      @li         New programmed value from IRP
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The IRP control register is written with the value passed
 *
 *   @b Writes
 *   @n IRP
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldIRP;
        oldIRP = CSL_chipWriteIRP (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteIRP(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Interrupt Return Pointer Register */                
    oldVal  = IRP;
    IRP     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteNRP
 *
 *   @b Description
 *   @n This API writes to the NMI Return Pointer Register (NRP) with the specified 
 *      value 'val' to setup a return point in the program to continue execution after 
 *      an NMI has occured. This API returns the old NRP value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from NRP
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The NRP control register is written with the value passed
 *
 *   @b Writes
 *   @n NRP
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldNRP;
        oldNRP = CSL_chipWriteNRP (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteNRP(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Nonmaskable Interrupt (NMI) Registers */                
    oldVal  = NRP;
    NRP     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteERP
 *
 *   @b Description
 *   @n This API writes to the Exception Return Pointer Register (ERP) with the specified 
 *      value 'val' to setup a return point in the program to continue execution after 
 *      an exception has occured. This API returns the old ERP value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b> Uint32
 *      @li         New programmed value from NRP register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The NRP control register is written with the value passed
 *
 *   @b Writes
 *   @n NRP
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldERP;
        oldERP = CSL_chipWriteERP (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteERP(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Exception Return Point (ERP) Registers */                
    oldVal  = NRP;
    NRP     = val;              

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteTSCL
 *
 *   @b Description
 *   @n This API writes to the Time-Stamp Counter (low order 32 bits) Register (TSCL) 
 *      with the specified value 'val' to enable the TSC. This API returns the 
 *      old TSCL value on return. The actual value written "val" to TSCL has no effect on
 *      the Time stamp counter. The value is simply ignored by the hardware, and a write to
 *      TSCL is used only to enable the Time Stamp Counter.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from TSCL register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The TSCL control register is written with the value passed
 *
 *   @b Writes
 *   @n TSCL
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldTSCL;
        oldTSCL = CSL_chipWriteTSCL (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteTSCL(CSL_Reg32  val)
{
    Uint32  oldVal;
    
    /** Time Stamp Counter Register - Low */
    oldVal  = TSCL;
    TSCL    = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteARP
 *
 *   @b Description
 *   @n This API writes to the Analysis Return Pointer (ARP) with the 
 *      specified value 'val'. This API returns the old ARP value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from ARP register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ARP control register is written with the value passed
 *
 *   @b Writes
 *   @n ARP
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldARP;
        oldARP = CSL_chipWriteARP (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteARP(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Analysis Return Point Register */
    oldVal  = ARP;
    ARP     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteILC
 *
 *   @b Description
 *   @n This API writes to the SPLOOP Inner Loop Count Register (ILC) with the 
 *      specified value 'val'. This API returns the old ILC value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from ILC register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ILC control register is written with the value passed
 *
 *   @b Writes
 *   @n ILC
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldILC;
        oldILC = CSL_chipWriteILC (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteILC(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** SPLOOP Inner Loop Count Register */
    oldVal  = ILC;
    ILC     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteRILC
 *
 *   @b Description
 *   @n This API writes to the SPLOOP Reload Inner Loop Count Register (RILC) with 
 *      the specified value 'val'. This API returns the old RILC value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from RILC register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The RILC control register is written with the value passed
 *
 *   @b Writes
 *   @n RILC
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldRILC;
        oldRILC = CSL_chipWriteRILC (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteRILC(CSL_Reg32  val)
{
    Uint32  oldVal;
    
    /** SPLOOP Reload Inner Loop Count Register */                
    oldVal  = RILC;
    RILC    = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteREP
 *
 *   @b Description
 *   @n This API writes to the Restricted Entry-Point Register (REP) with 
 *      the specified value 'val'. This API returns the old REP value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from REP register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The REP control register is written with the value passed
 *
 *   @b Writes
 *   @n REP
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldREP;
        oldREP = CSL_chipWriteREP (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteREP(CSL_Reg32  val)
{
    Uint32  oldVal;
    
    /** Restricted Entry Point Address Register */
    oldVal  = REP;
    REP     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteSSR
 *
 *   @b Description
 *   @n This API writes to the Saturation Status Register (SSR) with 
 *      the specified value 'val'. This API returns the old SSR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from SSR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The SSR control register is written with the value passed
 *
 *   @b Writes
 *   @n SSR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldSSR;
        oldSSR = CSL_chipWriteSSR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteSSR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Saturation Status Register */
    oldVal  = SSR;
    SSR     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteGPLYA
 *
 *   @b Description
 *   @n This API writes to the GMPY Polynomial-A side Register (GPLYA) with 
 *      the specified value 'val'. This API returns the old GPLYA value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from GPLYA register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The GPLYA control register is written with the value passed
 *
 *   @b Writes
 *   @n GPLYA
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldGPLYA;
        oldGPLYA = CSL_chipWriteGPLYA (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteGPLYA(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** GMPY Polynomial.A Side Register */
    oldVal  = GPLYA;
    GPLYA   = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteGPLYB
 *
 *   @b Description
 *   @n This API writes to the GMPY Polynomial-B side Register (GPLYB) with 
 *      the specified value 'val'. This API returns the old GPLYB value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from GPLYB register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The GPLYB control register is written with the value passed
 *
 *   @b Writes
 *   @n GPLYB
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldGPLYB;
        oldGPLYB = CSL_chipWriteGPLYB (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteGPLYB(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** GMPY Polynomial.B Side Register */
    oldVal  = GPLYB;
    GPLYB   = val;

    return oldVal;
}

/** ============================================================================
 *   @n@b CSL_chipWriteGFPGFR
 *
 *   @b Description
 *   @n This API writes to the Galios Field Polynomial Generator Function Register 
 *      (GFPGFR) with the specified value 'val'. This API returns the old GFPGFR value 
 *      on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from GFPGFR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The GFPGFR control register is written with the value passed
 *
 *   @b Writes
 *   @n GFPGFR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldGFPGFR;
        oldGFPGFR = CSL_chipWriteGFPGFR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteGFPGFR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Galois Field Polynomial Generator Function Register */
    oldVal  = GFPGFR;
    GFPGFR  = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteDIER
 *
 *   @b Description
 *   @n This API writes to the Debug Interrupt Enable Register (DIER) with the 
 *      specified value 'val'. This API returns the old DIER value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from DIER register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The DIER control register is written with the value passed
 *
 *   @b Writes
 *   @n DIER
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldDIER;
        oldDIER = CSL_chipWriteDIER (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteDIER(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Debug Interrupt Enable Register */
    oldVal  = DIER;
    DIER     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteTSR
 *
 *   @b Description
 *   @n This API writes to the Task State Register (TSR) with the specified value 
 *      'val'. This API returns the old TSR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from TSR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The TSR control register is written with the value passed
 *
 *   @b Writes
 *   @n TSR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldTSR;
        oldTSR = CSL_chipWriteTSR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteTSR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Task State Register */
    oldVal  = TSR;
    TSR     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteITSR
 *
 *   @b Description
 *   @n This API writes to the Interrupt Task State Register (ITSR) with the 
 *      specified value 'val'. This API returns the old ITSR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from ITSR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ITSR control register is written with the value passed
 *
 *   @b Writes
 *   @n ITSR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldITSR;
        oldITSR = CSL_chipWriteITSR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteITSR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Interrupt Task State Register */
    oldVal  = ITSR;
    ITSR    = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteNTSR
 *
 *   @b Description
 *   @n This API writes to the NMI/Exception Task State Register (NTSR) with the 
 *      specified value 'val'. This API returns the old NTSR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from NTSR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The NTSR control register is written with the value passed
 *
 *   @b Writes
 *   @n NTSR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldNTSR;
        oldNTSR = CSL_chipWriteNTSR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteNTSR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** NMI/Exception Task State Register */
    oldVal  = NTSR;
    NTSR    = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteECR
 *
 *   @b Description
 *   @n This API writes to the Exception Clear Register (ECR) with the specified 
 *      value 'val' to clear the corresponding bits in the EFR. This API returns 
 *      the old EFR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b> Uint32
 *      @li         New programmed value from EFR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The ECR control register is written with the value passed and the 
 *       corresponding exception bits in the EFR register are cleared.
 *
 *   @b Writes
 *   @n EFR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldEFR;
        oldEFR = CSL_chipWriteECR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteECR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Return the old value from Exception Flag Register
     * and set the Clear Register 
     */
    oldVal  = EFR;
    ECR     = val;

    return oldVal;
}


/** ============================================================================
 *   @n@b CSL_chipWriteIERR
 *
 *   @b Description
 *   @n This API writes to the Internal Exception Report Register (IERR) with the 
 *      specified value 'val'. This API returns the old IERR value on return.
 *
 *   @b Arguments
 *   @verbatim      
        val         Value to be written.
     @endverbatim
 *
 *   <b> Return Value  </b>  Uint32
 *      @li         New programmed value from IERR register
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The IERR control register is written with the value passed.
 *
 *   @b Writes
 *   @n IERR
 *
 * <b> Usage Constraints: </b>
 *  @n Please refer to the C64x+ user guide for constraints while accessing 
 *     registers in different privilege levels.
 *
 *   @b Example
 *   @verbatim
 
        Uint32 oldIERR;
        oldIERR = CSL_chipWriteIERR (56);

    @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Uint32  CSL_chipWriteIERR(CSL_Reg32  val)
{
    Uint32  oldVal;

    /** Internal Exception Report Register */
    oldVal  = IERR;
    IERR    = val;

    return oldVal;
}


#ifdef __cplusplus
}
#endif

/* @} */
#endif /* _CSL_CHIPAUX_H_ */

