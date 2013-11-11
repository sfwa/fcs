/* ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 20009
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

/** ===========================================================================
 *   @file  csl_vcp2Aux.h
 *
 *   @brief  
 *      Auxiliary API header for VCP2 CSL-FL
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par 
 */

#ifndef _CSL_VCP2AUX_H_
#define _CSL_VCP2AUX_H_

#include <ti/csl/csl_vcp2.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_VCP2_FUNCTION
 @{ */

/**
 * ============================================================================
 *  @n@b VCP2_ceil
 *
 *  @b Description
 *  @n This function calculates the ceiling for a given value and a power of 2.
 *     The arguments follow the formula: ceilVal * 2^pwr2 = ceiling+
 *     (val, pwr2).
 *
 *  @b Arguments
    @verbatim
        val         Value to be augmented.
        pwr2        The power of two by which val must be divisible.
    @endverbatim
 *
 *  <b>Return Value </b>    Uint32
 *  @n  Value       The smallest number which when multiplied by 2^pwr2 is
 *                  greater than val.
 *
 *  <b>Pre Condition </b>
 *  @n None.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n None.
 *
 *  @b Example
 *  @verbatim
        Uint32  numSysPar;
        numSysPar = VCP2_ceil ((frameLen * rate), 4);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_ceil (
    Uint32      val,
    Uint32      pwr2
)
{
    Uint32 value;

    value = ( ((val) - ( ((val) >> (pwr2)) << (pwr2)) ) == 0) ?   \
            ((val) >> (pwr2)) : (((val) >> (pwr2)) + 1);

    return value;
}

/**
 * ============================================================================
 *  @n@b VCP2_normalCeil
 *
 *  @b Description
 *  @n Returns the value rounded to the nearest integer greater than or
 *      equal to (val1/val2)
 *
 *  @b Arguments
    @verbatim
        val1        Value to be augmented.
        val2        Value by which val1 must be divisible.
    @endverbatim
 *
 *  <b>Return Value </b>    Uint32
 *
 *  <b>Pre Condition </b>
 *  @n None.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n None.
 *
 *  @b Example
 *  @verbatim
        Uint32 framelen = 51200;
        Uint32  numSubFrame;

        ...

        // to calculate the number of sub frames for SP mode
        numSubFrame = VCP2_normalCeil (framelen, VCP2_SUB_FRAME_SIZE_MAX);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_normalCeil (
    Uint32      val1,
    Uint32      val2
)
{
    Uint32 value;

    value = ( ((val1) % (val2)) != 0  ) ? ( ((val1) / (val2)) + 1 ) : \
               ((val1) / (val2));

    return value;
}

/**
 * ============================================================================
 *  @n@b VCP2_getPeripheralID
 *
 *  @b Description
 *  @n This function returns the value programmed into the VCPPID register.  
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
        hPid            Handle to the PID structure that needs to
                        be populated with PID info retrieved. 
    @endverbatim
 *
 *  <b>Return Value </b> None
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>  None
 *
 *  @b Reads
 *  @n  VCP2_VCPPID_TYPE,
 *      VCP2_VCPPID_CLASS,
 *      VCP2_VCPPID_REV
 *
 *  @b Example
    @verbatim
        VCP2Handle      hVcp2;
        VCP2Obj         vcp2Obj;
        VCP2_PID        vcp2Pid;
                   
        ...
        
        hVcp2 = VCP2_init(&vcp2Obj, 0, &status);    
        
        ...
        
        VCP2_getPeripheralID (hVcp2, &vcp2Pid);            
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_getPeripheralID (
    VCP2Handle          hVcp2,
    VCP2_PID*           hPid  
    
)
{
    hPid->pid_class = CSL_FEXT (hVcp2->cfgregs->VCPPID, VCP2_VCPPID_CLASS);
    hPid->type  = CSL_FEXT (hVcp2->cfgregs->VCPPID, VCP2_VCPPID_TYPE);
    hPid->rev   = CSL_FEXT (hVcp2->cfgregs->VCPPID, VCP2_VCPPID_REV);
    
    return;        
}

/**
 * ============================================================================
 *  @n@b VCP2_getBmEndian
 *
 *  @b Description
 *  @n This function returns the value programmed into the VCPEND register for
 *     the branch metrics data for Big Endian mode indicating whether the data
 *     is in its native 8-bit format ('1') or 32-bit word packed ('0').
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Uint32
 *  @n  Value           Branch metric memory format.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   32-bit word packed.
 *  @li    1   -   Native (8 bits).
 *
 *  @b Reads
 *  @n VCP2_VCPEND_BM
 *
 *  @b Example
    @verbatim
        If (VCP2_getBmEndian (hVcp2)) 
        {
            ...
        } // end if //
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_getBmEndian (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_BM);
}

/**
 * ============================================================================
 *  @n@b VCP2_getIcConfig
 *
 *  @b Description
 *  @n This function gets the current VCPIC register values and puts them in a
 *     structure of type VCP2_ConfigIc.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
        pConfigIc       Pointer to the structure of type VCP2_ConfigIc to hold
                        the values of VCPIC registers.
    @endverbatim
 *
 *  <b>Return Value </b> 
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The structure of type VCP2_ConfigIc passed as arguement contains the
 *     values of the VCP configuration registers.
 *
 *  @b Reads
 *  @n  VCP2_VCPIC0,
 *      VCP2_VCPIC1,
 *      VCP2_VCPIC2,
 *      VCP2_VCPIC3,
 *      VCP2_VCPIC4,
 *      VCP2_VCPIC5
 *
 *  @b Example
 *  @verbatim
            VCP2_ConfigIc    configIc;

            ...
            VCP2_getIcConfig (hVcp2, &configIc);

    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_getIcConfig (
	VCP2Handle       hVcp2,
    VCP2_ConfigIc    *configIc
)
{
    register int x0, x1, x2, x3, x4, x5;

    x0 = hVcp2->regs->VCPIC0;
    x1 = hVcp2->regs->VCPIC1;
    x2 = hVcp2->regs->VCPIC2;
    x3 = hVcp2->regs->VCPIC3;
    x4 = hVcp2->regs->VCPIC4;
    x5 = hVcp2->regs->VCPIC5;

    configIc->ic0 = x0;
    configIc->ic1 = x1;
    configIc->ic2 = x2;
    configIc->ic3 = x3;
    configIc->ic4 = x4;
    configIc->ic5 = x5;

}

/**
 * ============================================================================
 *  @n@b VCP2_getNumInFifo
 *
 *  @b Description
 *  @n This function returns the count, number of symbols currently in the
 *     input FIFO.
 *
 *  @b Arguments
    @verbatim
        hVcp2          Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Number of symbols in the branch metric input FIFO
 *                      buffer.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT1_NSYMIF
 *
 *  @b Example
 *  @verbatim
        Uint32 numSym;
        numSym = VCP2_getNumInFifo (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_getNumInFifo (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT1, VCP2_VCPSTAT1_NSYMIF);
}

/**
 * ============================================================================
 *  @n@b VCP2_getNumOutFifo
 *
 *  @b Description
 *  @n This function returns the count, number of symbols currently in the
 *     output FIFO.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Number of symbols present in the output FIFO buffer.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT1_NSYMOF
 *
 *  @b Example
 *  @verbatim
        Uint32  numSym;
        numSym = VCP2_getNumOutFifo (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_getNumOutFifo (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT1, VCP2_VCPSTAT1_NSYMOF);
}

/**
 * ============================================================================
 *  @n@b VCP2_getSdEndian
 *
 *  @b Description
 *  @n This function returns the value programmed into the VCPEND register for
 *     the soft decision data for Big Endian mode indicating whether the data
 *     is in its native 8-bit format ('1') or 32-bit word packed ('0').
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Soft decisions memory format.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   32-bit word packed.
 *  @li    1   -   Native (8 bits).
 *
 *  @b Reads
 *  @n VCP2_VCPEND_SD
 *
 *  @b Example
 *  @verbatim
        If (VCP2_getSdEndian (hVcp2)) 
        {
            ...
        } // end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_getSdEndian (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_SD);
}

/**
 * ============================================================================
 *  @n@b VCP2_getStateIndex
 *
 *  @b Description
 *  @n This function returns an index for the final maximum state metric.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Final maximum state metric index.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Reads
 *  @n VCP2_VCPOUT1_FMAXI
 *
 *  @b Example
 *  @verbatim
        Uint8   index

        index = VCP2_getStateIndex(hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint8 VCP2_getStateIndex (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->regs->VCPOUT1, VCP2_VCPOUT1_FMAXI);
}

/**
 * ============================================================================
 *  @n@b VCP2_getYamBit
 *
 *  @b Description
 *  @n This function returns the value of the Yamamoto bit after the VCP
 *     decoding.
 *
 *  @b Arguments
    @verbatim
        hVcp2       Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Yamamoto bit result. This bit is a quality indicator
 *                      bit and is only used if the Yamamoto logic is enabled.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The return bit value indicates,
 *  @li     0 - at least one trellis stage had an absolute difference less than
 *              the Yamamoto threshold and the decoded frame has poor quality.
 *  @li     1 - no trellis stage had an absolute difference less than the
 *              Yamamoto threshold and the frame has good quality.
 *
 *  @b Reads
 *  @n VCP2_VCPOUT1_YAM
 *
 *  @b Example
 *  @verbatim
        Uint8   yamBit

        yamBit = VCP2_getYamBit(hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint8 VCP2_getYamBit (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->regs->VCPOUT1, VCP2_VCPOUT1_YAM);
}

/**
 * ============================================================================
 *  @n@b VCP2_getMaxSm
 *
 *  @b Description
 *  @n This function returns the final maximum state metric after the VCP has
 *     completed its decoding.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Maximum state metric value for the final trellis stage.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Reads
 *  @n VCP2_VCPOUT0_FMAXS
 *
 *  @b Example
 *  @verbatim
        Int16   maxSm

        maxSm = VCP2_getMaxSm(hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Int16 VCP2_getMaxSm (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->regs->VCPOUT0, VCP2_VCPOUT0_FMAXS);
}

/**
 * ============================================================================
 *  @n@b VCP2_getMinSm
 *
 *  @b Description
 *  @n This function returns the final minimum state metric after the VCP has
 *     completed its decoding.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Minimum state metric value for the final trellis stage.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Reads
 *  @n VCP2_VCPOUT0_FMINS
 *
 *  @b Example
 *  @verbatim
        Int16   minSm

        minSm = VCP2_getMinSm(hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Int16 VCP2_getMinSm (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->regs->VCPOUT0, VCP2_VCPOUT0_FMINS);
}

/**
 * ============================================================================
 *  @n@b VCP2_icConfig
 *
 *  @b Description
 *  @n This function programs the VCP input configuration registers with the
 *     values provided through the VCP2_ConfigIc structure.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
        pVcpConfigIc    Pointer to VCP2_ConfigIc structure instance
                        containing the input configuration register values.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n VCP input configuration registers are configured appropriately.
 *
 *  @b Writes
 *  @n  VCP2_VCPIC0,
 *      VCP2_VCPIC1,
 *      VCP2_VCPIC2,
 *      VCP2_VCPIC3,
 *      VCP2_VCPIC4,
 *      VCP2_VCPIC5
 * 
 *  @b Example
 *  @verbatim
        VCP2_ConfigIc    configIc;
        configIc.ic0  =  0xf0b07050;
        configIc.ic1  =  0x10320000;
        configIc.ic2  =  0x000007fa;
        configIc.ic3  =  0x00000054;
        configIc.ic4  =  0x00800800;
        configIc.ic5  =  0x51f3000c;
        ...
        VCP2_icConfig (hVcp2, &configIc);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_icConfig (
	VCP2Handle       hVcp2,
    VCP2_ConfigIc    *vcpConfigIc
)
{
    register int x0, x1, x2, x3, x4, x5;

    x0 = vcpConfigIc->ic0;
    x1 = vcpConfigIc->ic1;
    x2 = vcpConfigIc->ic2;
    x3 = vcpConfigIc->ic3;
    x4 = vcpConfigIc->ic4;
    x5 = vcpConfigIc->ic5;

    hVcp2->regs->VCPIC0 = x0;
    hVcp2->regs->VCPIC1 = x1;
    hVcp2->regs->VCPIC2 = x2;
    hVcp2->regs->VCPIC3 = x3;
    hVcp2->regs->VCPIC4 = x4;
    hVcp2->regs->VCPIC5 = x5;
}

/**
 * ============================================================================
 *  @n@b VCP2_icConfigArgs
 *
 *  @b Description
 *  @n This function programs the VCP input configuration registers with the
 *     given values.
 *
 *  @b Arguments
    @verbatim
        hVcp2       Handle to VCP2 instance.
        ic0         Value to program input configuration register 0.
        ic1         Value to program input configuration register 1.
        ic2         Value to program input configuration register 2.
        ic3         Value to program input configuration register 3.
        ic4         Value to program input configuration register 4.
        ic5         Value to program input configuration register 5.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n  VCP2_VCPIC0,
 *      VCP2_VCPIC1,
 *      VCP2_VCPIC2,
 *      VCP2_VCPIC3,
 *      VCP2_VCPIC4,
 *      VCP2_VCPIC5
 *
 *  @b Example
 *  @verbatim
        Uint32 ic0, ic1, ic2, ic3, ic4, ic5;
        ...
        ic0  =  0xf0b07050;
        ic1  =  0x10320000;
        ic2  =  0x000007fa;
        ic3  =  0x00000054;
        ic4  =  0x00800800;
        ic5  =  0x51f3000c;
        ...
        VCP2_icConfigArgs (hVcp2, ic0, ic1, ic2, ic3, ic4, ic5);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_icConfigArgs (
	VCP2Handle  hVcp2,
    Uint32      ic0,
    Uint32      ic1,
    Uint32      ic2,
    Uint32      ic3,
    Uint32      ic4,
    Uint32      ic5
)
{
    hVcp2->regs->VCPIC0 = ic0;
    hVcp2->regs->VCPIC1 = ic1;
    hVcp2->regs->VCPIC2 = ic2;
    hVcp2->regs->VCPIC3 = ic3;
    hVcp2->regs->VCPIC4 = ic4;
    hVcp2->regs->VCPIC5 = ic5;
}

/**
 * ============================================================================
 *  @n@b VCP2_setBmEndian
 *
 *  @b Description
 *  @n This function programs the VCP to view the format of the branch metrics
 *     data as either native 8-bit format ('1') or values packed into 32-bit
 *     words in little endian format ('0').
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
        bmEnd           '1' for native 8-bit format and '0' for 32-bit word packed
                        format
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n VCP2_VCPEND_BM
 *
 *  @b Example
 *  @verbatim
        Uint32 bmEnd = VCP2_END_NATIVE;
        VCP2_setBmEndian (hVcp2, bmEnd);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_setBmEndian (
	VCP2Handle         hVcp2,
    Uint32             bmEnd
)
{
    CSL_FINS (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_BM, bmEnd);
}

/**
 * ============================================================================
 *  @n@b VCP2_setNativeEndian
 *
 *  @b Description
 *  @n This function programs the VCP to view the format of all data as native
 *     8-bit format.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n  VCP2_VCPEND_BM=1, 
 *      VCP2_VCPEND_SD=1
 *
 *  @b Example
 *  @verbatim
        VCP2_setNativeEndian (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_setNativeEndian (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_BM, NATIVE);
    CSL_FINST (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_SD, NATIVE);
}

/**
 * ============================================================================
 *  @n@b VCP2_setPacked32Endian
 *
 *  @b Description
 *  @n This function programs the VCP to view the format of all data as packed
 *     data in 32-bit words.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n  VCP2_VCPEND_BM=0,
 *      VCP2_VCPEND_SD=0
 *
 *  @b Example
 *  @verbatim
        VCP2_setPacked32Endian (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_setPacked32Endian (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_BM, 32BIT);
    CSL_FINST (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_SD, 32BIT);
}

/**
 * ============================================================================
 *  @n@b VCP2_setSdEndian
 *
 *  @b Description
 *  @n This function programs the VCP to view the format of the soft decision
 *     data as either native 8-bit format ('1') or values packed into 32-bit
 *     words in little endian format ('0').
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
        sdEnd           '1' for native 8-bit format and '0' for 32-bit word packed
                        format
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Writes
 *  @n VCP2_VCPEND_SD
 *
 *  @b Example
 *  @verbatim
        Uint32 sdEnd = VCP2_END_NATIVE;
        VCP2_setSdEndian (hVcp2, sdEnd);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_setSdEndian (
	VCP2Handle         hVcp2,
    Uint32             sdEnd
)
{
    CSL_FINS (hVcp2->cfgregs->VCPEND, VCP2_VCPEND_SD, sdEnd);
}

/**
 * ============================================================================
 *  @n@b VCP2_addPoly
 *
 *  @b Description
 *  @n This function is used to add either predefined or user defined
 *     Polynomials to the generated VCP2_Params.
 *
 *  @b Arguments
    @verbatim
        pPoly           Pointer to the structure of type VCP2_Poly containing
                        the values of generator polynomials.
        pParams         Pointer to the structure of type VCP2_Params containing
                        the generated values for input configuration registers.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n None.
 *
 *  <b>Post Condition </b>
 *  @n Writes the pParams structure passed as input with the result.
 *
 *  @b Writes
 *  @n None
 *
 *  @b Example
 *  @verbatim
        VCP2_Poly        poly = {VCP2_GEN_POLY_3, VCP2_GEN_POLY_1,
                                    VCP2_GEN_POLY_2, VCP2_GEN_POLY_3};
        VCP2_Params      params;
        VCP2_Baseparams  baseParams;
        
        ...

        VCP2_genParams (&baseParams, &params);
        VCP2_addPoly (&poly, &params);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_addPoly (
    VCP2_Poly        *poly,
    VCP2_Params      *params
)
{
    Uint8 x0, x1, x2, x3;

    x0 = poly->poly0;
    x1 = poly->poly1;
    x2 = poly->poly2;
    x3 = poly->poly3;

    params->poly0 = x0;
    params->poly1 = x1;
    params->poly2 = x2;
    params->poly3 = x3;

}

/**
 * ============================================================================
 *  @n@b VCP2_statError
 *
 *  @b Description
 *  @n This function returns a Boolean value indicating whether any VCP error
 *     has occurred.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Boolean value
 *  @li     bitStatus   ERR bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    FALSE   -   No error.
 *  @li    TRUE    -   VCP paused due to error.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_ERR
 *
 *  @b Example
 *  @verbatim
        VCP2_Error error;
        
        // check whether an error has occurred
        if (VCP2_statError (hVcp2)) 
        {
            VCP2_getErrors (hVcp2, &error);
        } // end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Bool VCP2_statError (
    VCP2Handle         hVcp2
)
{
    return (Bool) CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_ERR);
}

/**
 * ============================================================================
 *  @n@b VCP2_statInFifo
 *
 *  @b Description
 *  @n This function returns the input FIFO's empty status flag. A '1'
 *     indicates that the input FIFO is empty and a '0' indicates it is not
 *     empty.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> bitStatus
 *  @li     bitStatus   IFEMP bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   Input FIFO is not empty.
 *  @li    1   -   Input FIFO is empty.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_IFEMP
 *
 *  @b Example
 *  @verbatim
        If (VCP2_statInFifo (hVcp2)) 
        {
            ...
        } // end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statInFifo (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_IFEMP);
}

/**
 * ============================================================================
 *  @n@b VCP2_statOutFifo
 *
 *  @b Description
 *  @n This function returns the output FIFO's full status flag. A '1'
 *     indicates that the output FIFO is full and a '0' indicates it is not.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> bitStatus
 *  @li     bitStatus   OFFUL bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   Output FIFO is not full.
 *  @li    1   -   Output FIFO is full.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_OFFUL
 *
 *  @b Example
 *  @verbatim
        If (VCP2_statOutFifo (hVcp2)) 
        {
            ...
        } // end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statOutFifo (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_OFFUL);
}

/**
 * ============================================================================
 *  @n@b VCP2_statPause
 *
 *  @b Description
 *  @n This function returns the PAUSE bit status indicating whether the VCP is
 *     paused or not.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> bitStatus
 *  @li     bitStatus   PAUSE bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   VCP is not paused.
 *  @li    1   -   VCP is paused.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_PAUSE
 *
 *  @b Example
 *  @verbatim
        // Pause the VCP.
        VCP2_pause (hVcp2);
        
        // Wait for pause to take place
        while (! VCP2_statPause (hVcp2));
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statPause (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_PAUSE);
}

/**
 * ============================================================================
 *  @n@b VCP2_statRun
 *
 *  @b Description
 *  @n This function returns the RUN bit status indicating whether the VCP is
 *     running or not.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> bitStatus
 *  @li     bitStatus   RUN bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   VCP is not running.
 *  @li    1   -   VCP is running.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_RUN
 *
 *  @b Example
 *  @verbatim
        // start the VCP
        VCP2_start (hVcp2);
        
        // check that the VCP is running
        while (! VCP2_statRun (hVcp2));
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statRun (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_RUN);
}

/**
 * ============================================================================
 *  @n@b VCP2_statSymProc
 *
 *  @b Description
 *  @n This function returns the number of symbols processed, NSYMPROC bitfield
 *     of VCP.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> Value
 *  @li     Value       Number of symbols processed.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n None.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_NSYMPROC
 *
 *  @b Example
 *  @verbatim
        Uint32 numSym;
        
        ...

        numSym = VCP2_statSymProc (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statSymProc (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_NSYMPROC);
}

/**
 * ============================================================================
 *  @n@b VCP2_statWaitIc
 *
 *  @b Description
 *  @n This function returns the WIC bit status indicating whether the VCP is
 *     waiting to receive new input configuration values.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> bitStatus
 *  @li     bitStatus   WIC bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   VCP is not waiting for input configuration words.
 *  @li    1   -   VCP is waiting for input configuration words.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_WIC
 *
 *  @b Example
 *  @verbatim
        If (VCP2_statWaitIc (hVcp2)) 
        {
            ...
        } // end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statWaitIc (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_WIC);
}

/**
 * ============================================================================
 *  @n@b VCP2_start
 *
 *  @b Description
 *  @n This function starts the VCP by writing a start command to the VCPEXE
 *     register.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n VCP is started.
 *
 *  @b Writes
 *  @n VCP2_VCPEXE_COMMAND=1
 *
 *  @b Affects
 *  @n VCP2_VCPSTAT0_RUN=1
 *
 *  @b Example
 *  @verbatim
        VCP2_start (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_start (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEXE, VCP2_VCPEXE_COMMAND, START);
}

/**
 * ============================================================================
 *  @n@b VCP2_pause
 *
 *  @b Description
 *  @n This function pauses the VCP by writing a pause command to the VCPEXE
 *     register.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.The VCP should be operating in 
 *      debug/emulation mode.
 *
 *  <b>Post Condition </b>
 *  @n VCP is paused.
 *
 *  @b Writes
 *  @n VCP2_VCPEXE_COMMAND=2
 *
 *  @b Affects
 *  @n VCP2_VCPSTAT0_PAUSE=1
 *
 *  @b Example
 *  @verbatim
        VCP2_pause (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_pause (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEXE, VCP2_VCPEXE_COMMAND, PAUSE);
}

/**
 * ============================================================================
 *  @n@b VCP2_unpause
 *
 *  @b Description
 *  @n This function un-pauses the VCP, previously paused by VCP2_pause()
 *     function, by writing the un-pause command to the VCPEXE register.
 *          This function restarts the VCP at the beginning of current
 *     traceback, and VCP will run to normal completion.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.The VCP should be operating in 
 *      debug/emulation mode.
 *
 *  <b>Post Condition </b>
 *  @n VCP is restarted.
 *
 *  @b Writes
 *  @n VCP2_VCPEXE_COMMAND=4
 *
 *  @b Affects
 *  @n VCP2_VCPSTAT0_PAUSE=0
 *
 *  @b Example
 *  @verbatim
        VCP2_unpause (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_unpause (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEXE, VCP2_VCPEXE_COMMAND, RESTART);
}

/**
 * ============================================================================
 *  @n@b VCP2_stepTraceback
 *
 *  @b Description
 *  @n This function un-pauses the VCP, previously paused by VCP2_pause()
 *     function, by writing the un-pause command to the VCPEXE register.
 *     This function restarts the VCP at the beginning of current
 *     traceback and halts at the next traceback (i.e Step Single Traceback).
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it. The VCP should be operating in 
 *      debug/emulation mode.
 *
 *  <b>Post Condition </b>
 *  @n VCP is restarted.
 *
 *  @b Writes
 *  @n VCP2_VCPEXE_COMMAND=3
 *
 *  @b Example
 *  @verbatim
        VCP2_stepTraceback (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_stepTraceback (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEXE, VCP2_VCPEXE_COMMAND, RESTART_PAUSE);
}

/**
 * ============================================================================
 *  @n@b VCP2_reset
 *
 *  @b Description
 *  @n This function sets all the VCP control registers to their default
 *     values.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n All registers in the VCP are reset except for the execution register,
 *  endian register, emulation register and other internal registers.
 *
 *  @b Writes
 *  @n VCP2_VCPEXE_COMMAND=5
 *
 *  @b Example
 *  @verbatim
           VCP2_reset (hVcp2);

    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_reset (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEXE, VCP2_VCPEXE_COMMAND, STOP);
}

/**
 * ============================================================================
 *  @n@b VCP2_getErrors
 *
 *  @b Description
 *  @n This function will acquire the VCPERR register values and fill in the
 *     fields of VCP2_Error structure and pass it back as the results.
 *
 *  @b Arguments
    @verbatim
    		hVcp2       Handle to VCP2 instance.
            pVcpErr     Pointer to the VCP2_Errors structure instance.

    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The fields of the VCP2_Errors structure indicate the respective errors if
 *     occurred. Writes VCPSTAT0 register, as a side effect. Clears ERR bit.
 *
 *  @b Reads
 *  @n  VCP2_VCPERR_TBNAERR,
 *      VCP2_VCPERR_FTLERR,
 *      VCP2_VCPERR_FCTLERR,
 *      VCP2_VCPERR_MAXMINERR,
 *      VCP2_VCPERR_E_SYMX,
 *      VCP2_VCPERR_E_SYMR
 * 
 *  @b Affects
 *  @n  VCP2_VCPSTAT0_ERR=0
 *
 *  @b Example
    @verbatim
        VCP2_Errors error;
        
        // check whether an error has occurred
        if (VCP2_statError (hVcp2)) 
        {
            VCP2_getErrors (hVcp2, &error);
        } // end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_getErrors (
    VCP2Handle          hVcp2,
    VCP2_Errors         *pVcpErr
)
{
    pVcpErr->tbnaErr = (Bool) CSL_FEXT (hVcp2->cfgregs->VCPERR, VCP2_VCPERR_TBNAERR);
    pVcpErr->ftlErr = (Bool) CSL_FEXT (hVcp2->cfgregs->VCPERR, VCP2_VCPERR_FTLERR);
    pVcpErr->fctlErr = (Bool) CSL_FEXT (hVcp2->cfgregs->VCPERR, VCP2_VCPERR_FCTLERR);
    pVcpErr->maxminErr = (Bool) CSL_FEXT (hVcp2->cfgregs->VCPERR, VCP2_VCPERR_MAXMINERR);
    pVcpErr->symxErr = (Bool) CSL_FEXT (hVcp2->cfgregs->VCPERR, VCP2_VCPERR_E_SYMX);
    pVcpErr->symrErr = (Bool) CSL_FEXT (hVcp2->cfgregs->VCPERR, VCP2_VCPERR_E_SYMR);
}

/**
 * ============================================================================
 *  @n@b VCP2_statEmuHalt
 *
 *  @b Description
 *  @n This function returns the EMUHALT bit status indicating whether the VCP
 *     halt is due to emulation or not.
 *
 *  @b Arguments
    @verbatim
        hVcp2           Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b> bitStatus
 *  @li     bitStatus   Emuhalt bit field value of VCP status register 0.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n The returned value indicates
 *  @li    0   -   Not halt due to emulation.
 *  @li    1   -   Halt due to emulation.
 *
 *  @b Reads
 *  @n VCP2_VCPSTAT0_EMUHALT
 *
 *  @b Example
    @verbatim
        If (VCP2_statEmuHalt (hVcp2)) 
        {
            ...
        }// end if
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
Uint32 VCP2_statEmuHalt (
    VCP2Handle         hVcp2
)
{
    return CSL_FEXT (hVcp2->cfgregs->VCPSTAT0, VCP2_VCPSTAT0_EMUHALT);
}

/**
 * ============================================================================
 *  @n@b VCP2_emuDisable
 *
 *  @b Description
 *  @n This function disables the emulation/debug mode of VCP.
 *
 *  @b Arguments
    @verbatim
        hVcp2       Handle to VCP2 instance.
    @endverbatim
 *
 *  <b>Return Value </b>
 *  @n None.
 *
 *  <b>Pre Condition </b>
 *  @n @a VCP2_init() must be called to initialize the VCP2 peripheral
 *      before doing any operations on it.
 *
 *  <b>Post Condition </b>
 *  @n VCP emulation control register is modified.
 *
 *  @b Writes
 *  @n VCP2_VCPEMU_FREE=1
 *
 *  @b Example
    @verbatim
        VCP2_emuDisable (hVcp2);
    @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE
void VCP2_emuDisable (
    VCP2Handle         hVcp2
)
{
    CSL_FINST (hVcp2->cfgregs->VCPEMU, VCP2_VCPEMU_FREE, FREE);
}

/* @} */

#ifdef __cplusplus
}
#endif

#endif



