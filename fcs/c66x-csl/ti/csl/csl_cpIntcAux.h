/**
 *   @file  csl_cpintcAux.h
 *
 *   @brief   
 *      This is the CPINTC Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the CPINTC Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
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
 
#ifndef _CSL_CPINTC_AUX_H_
#define _CSL_CPINTC_AUX_H_

#include <ti/csl/csl_cpIntc.h>

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup CSL_CPINTC_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_CPINTC_setNestingMode
 *
 *   @b Description
 *   @n This API configures the Interrupt Controller nesting mode.
 *      
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
        nestMode    Nesting Mode to be configured
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The interrupt controller is configured to use the specified nesting mode.
 *
 *   @b  Writes
 *   @n  CPINTC_CONTROL_REG_NEST_MODE
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCNestingMode nestMode;
        CSL_CPINTC_HANDLE     hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_setNestingMode (hnd, nestMode);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_setNestingMode
(
    CSL_CPINTC_Handle       hnd, 
    CSL_CPINTCNestingMode   nestMode
)
{
    /* Write the CONTROL_REG with the specified nesting mode. */        
    ((CSL_CPINTC_RegsOvly)hnd)->CONTROL_REG = CSL_FMK(CPINTC_CONTROL_REG_NEST_MODE, nestMode);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_getNestingMode
 *
 *   @b Description
 *   @n This API gets the current nesting mode which is configured in the
 *      interrupt controller.
 * 
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
     @endverbatim
 * 
 *      
 *   <b> Return Value </b> 
 *       CSL_CPINTCNestingMode - Current Nesting Mode.
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_CONTROL_REG_NEST_MODE
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTC_HANDLE     hnd;
        CSL_CPINTCNestingMode nestMode;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        nestMode = CSL_CPINTC_getNestingMode();
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE CSL_CPINTCNestingMode CSL_CPINTC_getNestingMode (CSL_CPINTC_Handle hnd)
{
    return (CSL_CPINTCNestingMode)CSL_FEXT(((CSL_CPINTC_RegsOvly)hnd)->CONTROL_REG, 
                                            CPINTC_CONTROL_REG_NEST_MODE);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_setNestingLevel
 *
 *   @b Description
 *   @n This API overrides the Interrupt Controller global nesting level that
        is set by automatic nesting	mode
 *      
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
        nestLevel   Nesting Level to be configured
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The interrupt controller is configured to use the specified nesting level.
 *
 *   @b  Writes
 *   @n  CPINTC_GLB_NEST_LEVEL_REG_GLB_NEST_LEVEL
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCNestingLevel nestLevel;
        CSL_CPINTC_HANDLE      hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_setNestingLevel (hnd, nestLevel);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_setNestingLevel
(
    CSL_CPINTC_Handle       hnd, 
    CSL_CPINTCNestingLevel  nestLevel
)
{
	Uint32 writeVal;

	writeVal = nestLevel & 0x1ff;
	writeVal |= 0x80000000;		/* set bit 31 auto_override */
    /* Write the GLB_NEST_LEVEL_REG with the specified nesting mode. */        
    ((CSL_CPINTC_RegsOvly)hnd)->GLB_NEST_LEVEL_REG = writeVal;
}

/** ============================================================================
 *   @n@b CSL_CPINTC_getNestingLevel
 *
 *   @b Description
 *   @n This API gets the current nesting level which is configured in the
 *      interrupt controller.
 * 
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
     @endverbatim
 * 
 *      
 *   <b> Return Value </b> 
 *       CSL_CPINTCNestingLevel - Current Nesting Level.
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_GLB_NEST_LEVEL_REG_GLB_NEST_LEVEL
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTC_HANDLE      hnd;
        CSL_CPINTCNestingLevel nestLevel;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        nestLevel = CSL_CPINTC_getNestingLevel();
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE CSL_CPINTCNestingLevel CSL_CPINTC_getNestingLevel (CSL_CPINTC_Handle hnd)
{
    return (CSL_CPINTCNestingLevel)CSL_FEXT(((CSL_CPINTC_RegsOvly)hnd)->GLB_NEST_LEVEL_REG, 
                                            CPINTC_GLB_NEST_LEVEL_REG_GLB_NEST_LEVEL);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_enableAllHostInterrupt
 *
 *   @b Description
 *   @n This API enables all the Host Interrupts in the system.
 * 
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
     @endverbatim
 *       
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  All host interrupts are enabled in the system. Individual Host
 *       interrupts are still enabled/disabled from their individual 
 *       registers
 *
 *   @b  Writes
 *   @n  CPINTC_GLOBAL_ENABLE_HINT_REG_ENABLE_HINT_ANY=1
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTC_HANDLE     hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_enableAllHostInterrupt (hnd);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_enableAllHostInterrupt (CSL_CPINTC_Handle hnd)
{
    /* Enable all host interrupts by writing 1 to the register. */
    ((CSL_CPINTC_RegsOvly)hnd)->GLOBAL_ENABLE_HINT_REG = 
            CSL_FMK(CPINTC_GLOBAL_ENABLE_HINT_REG_ENABLE_HINT_ANY, 1);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_disableAllHostInterrupt
 *
 *   @b Description
 *   @n This API disables all the Host Interrupts in the system.
 * 
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
     @endverbatim
 *       
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  All host interrupts are disabled in the system. 
 *
 *   @b  Writes
 *   @n  CPINTC_GLOBAL_ENABLE_HINT_REG_ENABLE_HINT_ANY=0
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTC_HANDLE     hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_disableAllHostInterrupt (hnd);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_disableAllHostInterrupt (CSL_CPINTC_Handle hnd)
{
    /* Disable all host interrupts by writing 0 to the register. */
    ((CSL_CPINTC_RegsOvly)hnd)->GLOBAL_ENABLE_HINT_REG = 
            CSL_FMK(CPINTC_GLOBAL_ENABLE_HINT_REG_ENABLE_HINT_ANY, 0);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_enableSysInterrupt
 *
 *   @b Description
 *   @n This API enables the system interrupt in the interrupt controller.
 *      
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        sysIntr    System Interrupt which is to be enabled.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The specific System Interrupt is enabled in the controller and
 *       can now generate host interrupts.
 *
 *   @b  Writes
 *   @n  CPINTC_ENABLE_SET_INDEX_REG_ENABLE_SET_INDEX
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCSystemInterrupt     sysIntr;
        CSL_CPINTC_HANDLE             hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_enableSysInterrupt (hnd, sysIntr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_enableSysInterrupt 
(
    CSL_CPINTC_Handle           hnd,
    CSL_CPINTCSystemInterrupt   sysIntr
)
{
    /* Write the sysIntr to the ENABLE_SET_INDEX_REG to enable the interrupt. */
    ((CSL_CPINTC_RegsOvly)hnd)->ENABLE_SET_INDEX_REG = 
            CSL_FMK(CPINTC_ENABLE_SET_INDEX_REG_ENABLE_SET_INDEX, sysIntr);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_disableSysInterrupt
 *
 *   @b Description
 *   @n This API disables the system interrupt in the interrupt controller.
 *      
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        sysIntr    System Interrupt which is to be disabled.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The specific System Interrupt is disabled in the controller and
 *       cannot generate host interrupts.
 *
 *   @b  Writes
 *   @n  CPINTC_ENABLE_CLR_INDEX_REG_ENABLE_CLR_INDEX
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCSystemInterrupt     sysIntr;
        CSL_CPINTC_HANDLE             hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_disableSysInterrupt (hnd, sysIntr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_disableSysInterrupt 
(
    CSL_CPINTC_Handle           hnd,
    CSL_CPINTCSystemInterrupt   sysIntr
)
{
    /* Write the sysIntr to the ENABLE_CLR_INDEX_REG to disable the interrupt. */
    ((CSL_CPINTC_RegsOvly)hnd)->ENABLE_CLR_INDEX_REG = 
            CSL_FMK(CPINTC_ENABLE_CLR_INDEX_REG_ENABLE_CLR_INDEX, sysIntr);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_clearSysInterrupt
 *
 *   @b Description
 *   @n This API clear the system interrupt in the interrupt controller.
 *      
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        sysIntr    System Interrupt which is to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The specific system interrupt has been serviced and is cleared. 
 *
 *   @b  Writes
 *   @n  CPINTC_STATUS_CLR_INDEX_REG_STATUS_CLR_INDEX
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCSystemInterrupt     sysIntr;
        CSL_CPINTC_HANDLE             hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0       
        ...
        CSL_CPINTC_clearSysInterrupt (hnd, sysIntr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_clearSysInterrupt 
(
    CSL_CPINTC_Handle           hnd,
    CSL_CPINTCSystemInterrupt   sysIntr
)
{
    /* Write the sysIntr to the STATUS_CLR_INDEX_REG to clear the interrupt. */
    ((CSL_CPINTC_RegsOvly)hnd)->STATUS_CLR_INDEX_REG = 
            CSL_FMK(CPINTC_STATUS_CLR_INDEX_REG_STATUS_CLR_INDEX, sysIntr);
}

/** ============================================================================
 *   @n@b CSL_CPINTC_enableHostInterrupt
 *
 *   @b Description
 *   @n This API enables the host interrupt in the interrupt controller.
 *      
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        sysIntr    Host Interrupt which is to be enabled.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The specific Host Interrupt is enabled in the controller and
 *       can now generate host interrupts.
 *
 *   @b  Writes
 *   @n  CPINTC_HINT_ENABLE_SET_INDEX_REG_HINT_ENABLE_SET_INDEX
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCHostInterrupt     hostIntr;
        CSL_CPINTC_HANDLE           hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0       
        ...
        CSL_CPINTC_enableHostInterrupt (hnd, hostIntr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_enableHostInterrupt 
(
    CSL_CPINTC_Handle           hnd,
    CSL_CPINTCHostInterrupt     hostIntr
)
{
    /* Write the hostIntr to the HINT_ENABLE_SET_INDEX_REG to enable the interrupt. */
    ((CSL_CPINTC_RegsOvly)hnd)->HINT_ENABLE_SET_INDEX_REG = 
            CSL_FMK(CPINTC_HINT_ENABLE_SET_INDEX_REG_HINT_ENABLE_SET_INDEX, hostIntr);
}

/** ============================================================================
 *   @n@b CSL_CPINTCdisableHostInterrupt
 *
 *   @b Description
 *   @n This API disables the host interrupt in the interrupt controller.
 *      
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        sysIntr    Host Interrupt which is to be disabled.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The specific Host Interrupt is enabled in the controller and
 *       can now generate host interrupts.
 *
 *   @b  Writes
 *   @n  CPINTC_HINT_ENABLE_CLR_INDEX_REG_HINT_ENABLE_CLR_INDEX
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCHostInterrupt     hostIntr;
        CSL_CPINTC_HANDLE           hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTCdisableHostInterrupt (hnd, hostIntr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_disableHostInterrupt
(
    CSL_CPINTC_Handle           hnd,
    CSL_CPINTCHostInterrupt     hostIntr
)
{
    /* Write the hostIntr to the HINT_ENABLE_CLR_INDEX_REG to disable the interrupt. */
    ((CSL_CPINTC_RegsOvly)hnd)->HINT_ENABLE_CLR_INDEX_REG = 
            CSL_FMK(CPINTC_HINT_ENABLE_CLR_INDEX_REG_HINT_ENABLE_CLR_INDEX, hostIntr);        
}

/** ============================================================================
 *   @n@b CSL_CPINTC_isInterruptPending
 *
 *   @b Description
 *   @n This API is used to check if there are any pending interrupts active
 *      in the system or not.
 * 
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       TRUE  - Interrupts are pending.
 *       FALSE - No interrupts are pending.
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_GLB_PRI_INTR_REG_GLB_NONE
 *
 *   @b  Example
 *   @verbatim
        Bool                pendingStatus;
        CSL_CPINTC_HANDLE   hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        pendingStatus = CSL_CPINTCisInterruptPending(hnd);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Bool CSL_CPINTC_isInterruptPending (CSL_CPINTC_Handle hnd)
{
    Int32   pendStatus;
    
    /* Read the 'GLB_PRI_INTR_REG' to determine if there is a pending interrupt or not? */
    pendStatus = CSL_FEXT(((CSL_CPINTC_RegsOvly)hnd)->GLB_PRI_INTR_REG, 
                          CPINTC_GLB_PRI_INTR_REG_GLB_NONE);
    return (pendStatus == 1) ? FALSE : TRUE; 
}

/** ============================================================================
 *   @n@b CSL_CPINTC_getPendingInterrupt
 *
 *   @b Description
 *   @n This API gets the current highest priority pending interrupt. This API
 *      should only be called after checking if there are pending interrupts 
 *      using the 'CSL_CPINTCisInterruptPending'.
 * 
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       CSL_CPINTCHostInterrupt - Highest Priority System Interrupt.
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_GLB_PRI_INTR_REG_GLB_PRI_INTR
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCHostInterrupt intr;
        Bool                    pendingStatus;
        CSL_CPINTC_HANDLE       hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        pendingStatus = CSL_CPINTC_isInterruptPending(hnd);
        if (pendingStatus == TRUE)
        {
            intr = CSL_CPINTC_getPendingInterrupt(hnd);
        }
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE CSL_CPINTCSystemInterrupt CSL_CPINTC_getPendingInterrupt (CSL_CPINTC_Handle hnd)
{
    return (CSL_CPINTCHostInterrupt)(CSL_FEXT(((CSL_CPINTC_RegsOvly)hnd)->GLB_PRI_INTR_REG, 
                      CPINTC_GLB_PRI_INTR_REG_GLB_PRI_INTR));
}

/** ============================================================================
 *   @n@b CSL_CPINTC_isHostInterruptPending
 *
 *   @b Description
 *   @n This API is used to check if there are any pending interrupts active
 *      for a particular host.
 * 
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        hostIntr   This is the specific host interrupt output.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       TRUE  - Interrupts are pending.
 *       FALSE - No interrupts are pending.
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_PRI_HINT_REG0_NONE_HINT_0..CPINTC_PRI_HINT_REG255_NONE_HINT_255
 *
 *   @b  Example
 *   @verbatim
        Bool                pendingStatus;
        CSL_CPINTC_HANDLE   hnd;
        CSL_CPINTCHostInterrupt host;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        pendingStatus = CSL_CPINTC_isHostInterruptPending(hnd, host);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE Bool CSL_CPINTC_isHostInterruptPending 
(
	CSL_CPINTC_Handle hnd, 
    CSL_CPINTCHostInterrupt     hostIntr
)
{
    Int32   pendStatus;
    
    /* Read the 'PRI_HINT_REG' to determine if there is a pending interrupt for this host */
    pendStatus = CSL_FEXT(((CSL_CPINTC_RegsOvly)hnd)->PRI_HINT_REG[hostIntr], 
                          CPINTC_PRI_HINT_REG0_NONE_HINT_0);
    return (pendStatus == 1) ? FALSE : TRUE; 
}

/** ============================================================================
 *   @n@b CSL_CPINTC_getPendingHostInterrupt
 *
 *   @b Description
 *   @n This API gets the current highest priority pending interrupt for a
 *      given host. This API should only be called after checking if there
 *      are pending interrupts using the 'CSL_CPINTCisHostInterruptPending'.
 * 
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        hostIntr   This is the specific host interrupt output.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       CSL_CPINTCSystemInterrupt - Highest Priority System Interrupt.
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_PRI_HINT_REG0_PRI_HINT_0..CPINTC_PRI_HINT_REG255_PRI_HINT_255
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCSystemInterrupt intr;
        Bool                      pendingStatus;
        CSL_CPINTC_HANDLE         hnd;
        CSL_CPINTCHostInterrupt   host;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        pendingStatus = CSL_CPINTC_isHostInterruptPending(hnd, host);
        if (pendingStatus == TRUE)
        {
            intr = CSL_CPINTC_getPendingHostInterrupt(hnd, host);
        }
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE CSL_CPINTCSystemInterrupt CSL_CPINTC_getPendingHostInterrupt 
(
	CSL_CPINTC_Handle hnd,
    CSL_CPINTCHostInterrupt     hostIntr
)
{
    return (CSL_CPINTCSystemInterrupt)(CSL_FEXT(((CSL_CPINTC_RegsOvly)hnd)->PRI_HINT_REG[hostIntr], 
                      CPINTC_PRI_HINT_REG0_PRI_HINT_0));
}

/** ============================================================================
 *   @n@b CSL_CPINTC_getRawInterruptStatus
 *
 *   @b Description
 *   @n The function is used to get the contents of the RAW Interrupt status
 *      register. 
 * 
 *   @b Arguments
 *   @verbatim
        hnd         This is the handle to the CPINTC instance.
        index       RAW Status Index register which is to be read
        rawStatus   RAW Status which indicates which interrupts are pending.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  CPINTC_RAW_STATUS_REG
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTC_HANDLE   hnd;
        Uint32              rawStatus;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        // Check if there are any interrupts between 0-31 pending.
        CSL_CPINTC_getRawInterruptStatus(hnd, 0, &rawStatus);
        if (rawStatus != 0)
        {
            // An Interrupt from 0-31 is pending.
        }
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_getRawInterruptStatus 
(
    CSL_CPINTC_Handle   hnd,
    Uint8               index,
    Uint32*             rawStatus
)
{
    *rawStatus = ((CSL_CPINTC_RegsOvly)hnd)->RAW_STATUS_REG[index];
}

/** ============================================================================
 *   @n@b CSL_CPINTC_mapSystemIntrToChannel
 *
 *   @b Description
 *   @n This API is used to map a system interrupt to a specific channel.
 *
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        sysIntr    System Interrupt which is to be mapped
        channel    Channel Number to which the interrupt is to be mapped.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Writes
 *   @n  CPINTC_CH_MAP_REG0_CH_MAP_0..CPINTC_CH_MAP_REG49_CH_MAP_196
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCSystemInterrupt sysIntr;
        CSL_CPINTCChannel         channel;
        CSL_CPINTC_HANDLE         hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_mapSystemIntrToChannel(hnd, sysIntr, channel);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_mapSystemIntrToChannel 
(
    CSL_CPINTC_Handle           hnd,
    CSL_CPINTCSystemInterrupt   sysIntr,
    CSL_CPINTCChannel           channel
)
{

#ifdef _LITTLE_ENDIAN
    ((CSL_CPINTC_RegsOvly)hnd)->CH_MAP[sysIntr] = channel;
#else
   ((CSL_CPINTC_RegsOvly)hnd)->CH_MAP[(sysIntr&~3) + (3-(sysIntr&3))] = channel;
#endif
   return;
}

/** ============================================================================
 *   @n@b CSL_CPINTC_mapChannelToHostInterrupt
 *
 *   @b Description
 *   @n This API is used to map the channel to the host interrupt.
 *
 *   @b Arguments
 *   @verbatim
        hnd        This is the handle to the CPINTC instance.
        channel    Channel Number to be mapped
        sysIntr    Host Interrupt to which the channel is mapped to.
     @endverbatim
 *
 *   <b> Return Value </b> 
 *       None 
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_CPINTC_open() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Writes
 *   @n  CPINTC_HINT_MAP_REG0_HINT_MAP_0..CPINTC_HINT_MAP_REG49_HINT_MAP_196
 *
 *   @b  Example
 *   @verbatim
        CSL_CPINTCHostInterrupt hostIntr;
        CSL_CPINTCChannel       channel;
        CSL_CPINTC_HANDLE       hnd;
        ...
        hnd = CSL_CPINTC_open (0);    // Opens CPINTC Instance 0
        ...
        CSL_CPINTC_mapChannelToHostInterrupt(hnd, channel, hostIntr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_CPINTC_mapChannelToHostInterrupt 
(
    CSL_CPINTC_Handle         hnd,
    CSL_CPINTCChannel         channel,
    CSL_CPINTCHostInterrupt   hostIntr
)
{

#ifdef _LITTLE_ENDIAN
    ((CSL_CPINTC_RegsOvly)hnd)->HINT_MAP[hostIntr] = channel;
#else
   ((CSL_CPINTC_RegsOvly)hnd)->HINT_MAP[(hostIntr&~3) + (3-(hostIntr&3))] = channel;
#endif
    return;
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _CSL_CPINTC_AUX_H_ */

