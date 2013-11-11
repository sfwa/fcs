/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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
 *   @file  csl_chip.h
 *
 *   @brief  
 *      API header file for CHIP module CSL
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par
 */
 
/** @defgroup CSL_CHIP_API CHIP 
 *
 *  @section Introduction 
 *  
 * @subsection xxx Overview
 *  This module deals with all System On Chip (SOC) configurations. It consists
 *  of Configuration Registers specific for the chip. Following are the Registers
 *  associated with the CHIP module:   \n 
 *
 *      (1) Addressing Mode Register - This register specifies the addressing 
 *          mode for the registers which can perform linear or circular addressing, 
 *          also contain sizes for circular addressing  \n
 *          
 *      (2) Control Status Register - This register contains the control and status
 *          bits. This register is used to control the mode of cache. This is also 
 *          used to enable or disable all the interrupts except reset and 
 *          non maskable interrupt. \n
 *
 *      (3) Interrupt Flag Register - This register contains the status of 
 *          INT4-INT15 and NMI interrupt. Each corresponding bit in the IFR is set 
 *          to 1 when that interrupt occurs; otherwise, the bits are cleared to 0. \n
 *
 *      (4) Interrupt Set Register - This register allows user to manually set the
 *          maskable interrupts (INT4-INT15) in the interrupt flag register (IFR).
 *          Writing a 1 to any of the bits in ISR causes the corresponding interrupt 
 *          flag to be set in IFR. \n
 *
 *      (5) Interrupt Clear Register - This register allows user to manually clear
 *          the maskable interrupts (INT15-INT4) in the interrupt flag register 
 *          (IFR). Writing a 1 to any of the bits in ICR causes the corresponding 
 *          interrupt flag to be cleared in IFR. \n
 *
 *      (6) Interrupt Enable Register - This register enables and disables individual
 *          interrupts and this not accessible in User mode. \n
 *
 *      (7) Interrupt Service Table Pointer Register - This register is used to
 *          locate the interrupt service routine (ISR). \n
 *
 *      (8) Interrupt Return Pointer Register - This register contains the return
 *          pointer that directs the CPU to the proper location to continue program
 *          execution after processing a maskable interrupt. \n
 *
 *      (9) Nonmaskable Interrupt (NMI) Return Pointer Register - This register
 *          contains the return pointer that directs the CPU to the proper location 
 *          to continue program execution after processing of a non-maskable 
 *          interrupt (NMI). \n
 *
 *      (10)Exception Return Pointer Register - This register contains the return
 *          pointer that directs the CPU to the proper location to continue program
 *          execution after processing of a exception. \n
 *
 *      (11)Time Stamp Counter Registers - The CPU contains a free running 64-bit
 *          counter that advances each CPU clock after counting is enabled. The counter
 *          is accessed using two 32-bit read-only control registers, Time Stamp Counter
 *          Registers - Low (TSCL) and Time Stamp Counter Registers - High (TSCH). The
 *          counter is enabled by writing to TSCL. The value written is ignored. Once
 *          enabled, counting cannot be disabled under program control. Counting is
 *          disabled in the following cases:
 *                  a.  After exiting the reset state.
 *                  b.  When the CPU is fully powered down. \n
 *
 *      (12)Analysis Return Pointer \n
 *
 *      (13)SPLOOP Inner Loop Count Register - The SPLOOP or SPLOOPD instructions
 *          use the SPLOOP inner loop count register (ILC), as the count of the number
 *          of iterations left to perform. The ILC content is decremented at each stage
 *          boundary until the ILC content reaches 0. \n
 *
 *      (14)SPLOOP Reload Inner Loop Count Register - Predicated SPLOOP or 
 *          SPLOOPD instructions used in conjunction with a SPMASKR or SPKERNELR 
 *          instruction use the SPLOOP reload inner loop count register (RILC), as 
 *          the iteration count value to be written to the SPLOOP inner loop count 
 *          register (ILC) in the cycle before the reload operation begins. \n
 *
 *      (15)Restricted Entry Point Address Register - This register is used by the
 *          SWENR instruction as the target of the change of control when an SWENR
 *          instruction is issued. The contents of REP should be preinitialized by the
 *          processor in Supervisor mode before any SWENR instruction is issued.  \n
 *
 *      (16)E1 Phase Program Counter - This register contains the 32-bit address of the 
 *          fetch packet in the E1 pipeline phase. \n
 *
 *      (17)DSP Core Number Register - This register provides an identifier to shared 
 *          resources in the system which identifies which CPU is accessing those
 *          resources. The contents of this register are set to a specific value at reset. \n
 *
 *      (18)Saturation Status Register - This register provides saturation flags for
 *          each functional unit, making it possible for the program to distinguish
 *          between saturations caused by different instructions in the same execute 
 *          packet. \n
 *
 *      (19)GMPY Polynomial.A Side Register - The GMPY instruction uses the 32-bit
 *          polynomial in the GMPY polynomial-A side register (GPLYA), when the
 *          instruction is executed on the M1 unit. \n
 *
 *      (20)GMPY Polynomial.B Side Register - The GMPY instruction uses the 32-bit
 *          polynomial in the GMPY polynomial-B side register (GPLYB), when the 
 *          instruction is executed on the M2 unit. \n
 *
 *      (21)Galois Field Polynomial Generator Function Register - This register
 *          controls the field size and the Galois field polynomial generator of the
 *          Galois field multiply hardware. \n
 *
 *      (22)Debug Interrupt Enable Register - This register is used to designate 
 *          which interrupts and exceptions are treated as high priority interrupts
 *          when operating in real-time emulation mode. \n
 *
 *      (23)Task State Register - This register contains all of the status bits that
 *          determine or indicate the current execution environment. TSR is saved in the
 *          event of an interrupt or exception to the ITSR or NTSR, respectively. \n
 *
 *      (24)Interrupt Task State Register - This register is used to store the
 *          contents of the task state register (TSR) in the event of an interrupt. \n
 *
 *      (25)NMI/Exception Task State Register - This register is used to store
 *          the contents of the task state register (TSR) and the conditions under 
 *          which an exception occurred in the event of a nonmaskable interrupt (NMI) or 
 *          an exception. \n
 *
 *      (26)Exception Flag Register - This register contains bits that indicate
 *          which exceptions have been detected. Clearing the EFR bits is done by
 *          writing a 1 to the corresponding bit position in the exception clear
 *          register (ECR). \n
 *
 *      (27)Exception Clear Register - This register is used to clear individual
 *          bits in the exception flag register (EFR). Writing a 1 to any bit in ECR
 *          clears the corresponding bit in EFR. \n
 *
 *      (28)Internal Exception Report Register - This register contains flags that
 *          indicate the cause of the internal exception. \n
 */
 
#ifndef _CSL_CHIP_H_
#define _CSL_CHIP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/csl/cslr_chip.h>

#if defined(_TMS320C6X)
#include <c6x.h>
#endif

#include <ti/csl/soc.h>        

/**
@defgroup CSL_CHIP_ENUM  CHIP Enumerated Data Types
@ingroup CSL_CHIP_API
*/

/**
@defgroup CSL_CHIP_FUNCTION  CHIP Functions
@ingroup CSL_CHIP_API
*/

/* CHIP global macro declarations */

/* CHIP global typedef declarations */

/** @addtogroup CSL_CHIP_ENUM
 @{ */
/** Enum for the CHIP registers */
typedef enum  {
    /** Addressing Mode Register */
    CSL_CHIP_AMR = 0,
    /** Control Status Register */
    CSL_CHIP_CSR = 1,
    /** Interrupt Flag Register */
    CSL_CHIP_IFR = 2,
    /** Interrupt Set Register */
    CSL_CHIP_ISR = 2,
    /** Interrupt Clear Register */
    CSL_CHIP_ICR = 3,
    /** Interrupt Enable Register */
    CSL_CHIP_IER = 4,
    /** Interrupt Service Table Pointer Register */
    CSL_CHIP_ISTP= 5,
    /** Interrupt Return Pointer Register */
    CSL_CHIP_IRP= 6,
    /** Nonmaskable Interrupt (NMI) Return Pointer Register */
    CSL_CHIP_NRP= 7,
    /** Exception Return Pointer Register */
    CSL_CHIP_ERP= 7,
    /** Time Stamp Counter Register - Low */
    CSL_CHIP_TSCL= 10,
    /** Time Stamp Counter Registers - High */
    CSL_CHIP_TSCH= 11,
    /** Analysis Return Pointer */
    CSL_CHIP_ARP= 12,    
    /** SPLOOP Inner Loop Count Register */
    CSL_CHIP_ILC= 13,
    /** SPLOOP Reload Inner Loop Count Register */
    CSL_CHIP_RILC= 14,
    /** Restricted Entry Point Address Register */
    CSL_CHIP_REP= 15,
    /** E1 Phase Program Counter */
    CSL_CHIP_PCE1= 16,
    /** DSP Core Number Register */
    CSL_CHIP_DNUM= 17,
    /** Saturation Status Register */
    CSL_CHIP_SSR= 21,
    /** GMPY Polynomial.A Side Register */
    CSL_CHIP_GPLYA= 22,
    /** GMPY Polynomial.B Side Register */
    CSL_CHIP_GPLYB= 23,
    /** Galois Field Polynomial Generator Function Register */
    CSL_CHIP_GFPGFR= 24,
    /** Debug interrupt enable register */
    CSL_CHIP_DIER=25,
    /** Task State Register */
    CSL_CHIP_TSR= 26,
    /** Interrupt Task State Register */
    CSL_CHIP_ITSR= 27,
    /** NMI/Exception Task State Register */
    CSL_CHIP_NTSR= 28,
    /** Exception Flag Register */
    CSL_CHIP_EFR= 29,
    /** Exception Clear Register */
    CSL_CHIP_ECR= 29,
    /** Internal Exception Report Register */
    CSL_CHIP_IERR= 31
} CSL_ChipReg;

/**
@}
*/

/* CHIP global function declarations   */

extern Uint32  CSL_chipWriteReg (
    CSL_ChipReg    reg,
    CSL_Reg32      val
);

extern Uint32  CSL_chipReadReg(
    CSL_ChipReg    reg
);

#ifdef __cplusplus
}
#endif

#endif
