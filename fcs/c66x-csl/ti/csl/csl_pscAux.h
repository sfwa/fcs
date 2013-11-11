/**
 *   @file  csl_pscAux.h
 *
 *   @brief API Auxilary header file for PSC CSL
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2002-2010 Texas Instruments, Inc.
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
#ifndef _CSL_PSCAUX_H_
#define _CSL_PSCAUX_H_

#include <ti/csl/csl_psc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_PSC_FUNCTION
 @{ */


/** ============================================================================
 *   @n@b CSL_PSC_getVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the PSC peripheral identification register
 *      contents.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  Uint32  - version value
 *	 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PSC_PID
 *
 *   @b Example
 *   @verbatim
        Uint32      versionInfo;

        versionInfo =   CSL_PSC_getVersionInfo ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_PSC_getVersionInfo (void)
{

    return hPscRegs->PID;
}


/** ============================================================================
 *   @n@b CSL_PSC_getVoltageControl
 *
 *   @b Description
 *   @n This function retrieves the Smart reflex bits from the voltage
 *      control identification register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  Uint8
 *	 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PSC_VCNTLID_VCNTL
 *
 *   @b Example
 *   @verbatim
        Uint32      vcntlInfo;

        vcntlInfo =   CSL_PSC_getVoltageControl ();

	 @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_PSC_getVoltageControl (void)
{

    return CSL_FEXT (hPscRegs->VCNTLID, PSC_VCNTLID_VCNTL);
}
        

/** ============================================================================
 *   @n@b CSL_PSC_setModuleNextState
 *
 *   @b Description
 *   @n This function sets up the "Next" state the module should be 
 *      transitioned for a given module.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       LPSC Module for which the next state must be configured.
        state           Next state to which the module must be transitioned.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None 
 *
 *   <b> Post Condition </b>
 *   @n  Module is moved to configured next state after transition is triggered
 *       using @a CSL_PSC_startStateTransition () API.
 *
 *   @b  Writes
 *   @n  PSC_MDCTL_NEXT
 *
 *   @b  Example
 *   @verbatim
        ...
        // Enable Module 1's clock.
        CSL_PSC_setModuleNextState (1, PSC_MODSTATE_ENABLE);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_setModuleNextState (
    Uint32                  moduleNum,
    CSL_PSC_MODSTATE        state
)
{
    CSL_FINS (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_NEXT, state);
    
    return;
}

/** ===========================================================================
 *   @n@b CSL_PSC_getModuleNextState
 *
 *   @b Description
 *   @n This function returns the next state configured for a given module.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the state must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  CSL_PSC_MODSTATE 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDCTL_NEXT
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's next state configured
        if (CSL_PSC_getModuleNextState (2) == PSC_MODSTATE_ENABLE)
        {
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_PSC_MODSTATE CSL_PSC_getModuleNextState  (
    Uint32                  moduleNum
)
{
    return (CSL_PSC_MODSTATE) CSL_FEXT (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_NEXT);
}

/** ===========================================================================
 *   @n@b CSL_PSC_setModuleLocalReset
 *
 *   @b Description
 *   @n This function configures the Module local reset control. 
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
        resetState      Assert/Deassert module local reset.
     @endverbatim
 *
 *   <b> Return Value </b>  None 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n PSC_MDCTL_LRST
 *
 *   @b Example
 *   @verbatim
        ...
        // Assert Module 2's local reset 
        CSL_PSC_setModuleLocalReset (2, PSC_MDLRST_ASSERTED);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_setModuleLocalReset  (
    Uint32                  moduleNum,
    CSL_PSC_MDLRST          resetState
)
{
    CSL_FINS (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_LRST, resetState);

    return;
}

/** ===========================================================================
 *   @n@b CSL_PSC_getModuleLocalReset
 *
 *   @b Description
 *   @n This function reads the Module local reset control configured.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  CSL_PSC_MDLRST 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDCTL_LRST
 *
 *   @b Example
 *   @verbatim
 *      Uint32  resetState;
 
        ...
        // Check Module 2's local reset bit
        resetState = CSL_PSC_getModuleLocalReset (2);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_PSC_MDLRST CSL_PSC_getModuleLocalReset  (
    Uint32                  moduleNum
)
{
    return (CSL_PSC_MDLRST) CSL_FEXT (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_LRST);
}


/** ===========================================================================
 *   @n@b CSL_PSC_enableModuleResetIsolation
 *
 *   @b Description
 *   @n This function enables the Module reset isolation control.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the configuration must be done.
     @endverbatim
 *
 *   <b> Return Value </b>  None 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n PSC_MDCTL_RSTISO=1
 *
 *   @b Example
 *   @verbatim
        ...
        // Enable Module 2's reset isolation
        CSL_PSC_enableModuleResetIsolation (2);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_enableModuleResetIsolation  (
    Uint32                  moduleNum
)
{
    CSL_FINST (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_RSTISO, ENABLE);

    return;
}


/** ===========================================================================
 *   @n@b CSL_PSC_disableModuleResetIsolation
 *
 *   @b Description
 *   @n This function disables the Module reset isolation control.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the configuration must be done.
     @endverbatim
 *
 *   <b> Return Value </b>  None 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Writes
 *   @n PSC_MDCTL_RSTISO=0
 *
 *   @b Example
 *   @verbatim
        ...
        // Disable Module 2's reset isolation
        CSL_PSC_disableModuleResetIsolation (2);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_disableModuleResetIsolation  (
    Uint32                  moduleNum
)
{
    CSL_FINST (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_RSTISO, DISABLE);

    return;
}


/** ===========================================================================
 *   @n@b CSL_PSC_isModuleResetIsolationEnabled
 *
 *   @b Description
 *   @n This function reads the Module reset isolation control bit.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  Bool 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDCTL_RSTISO
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's reset isolation configuration
        if (CSL_PSC_isModuleResetIsolationEnabled (2) == TRUE)
        {
            // Module 2 reset isolation enabled
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE Bool CSL_PSC_isModuleResetIsolationEnabled  (
    Uint32                  moduleNum
)
{
    return CSL_FEXT (hPscRegs->MDCTL[moduleNum], PSC_MDCTL_RSTISO);
}


/** ===========================================================================
 *   @n@b CSL_PSC_getModuleState
 *
 *   @b Description
 *   @n This function returns the current state of a given module.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the state must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  CSL_PSC_MODSTATE 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n Power Domain status is returned
 *
 *   @b Reads
 *   @n PSC_MDSTAT_STATE
 *
 *   @b Example
 *   @verbatim
        ...
        // Check if Module 2's clock is enabled.
        if (CSL_PSC_getModuleState (2) == PSC_MODSTATE_ENABLE)
        {
            // Module 2's clock is enabled.
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_PSC_MODSTATE CSL_PSC_getModuleState  (
    Uint32                  moduleNum
)
{
    return (CSL_PSC_MODSTATE) CSL_FEXT(hPscRegs->MDSTAT[moduleNum], PSC_MDSTAT_STATE);
}


/** ===========================================================================
 *   @n@b CSL_PSC_getModuleLocalResetStatus
 *
 *   @b Description
 *   @n This function returns the Module local reset actual status. 
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  CSL_PSC_MDLRST 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDSTAT_LRST
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's local reset status
        if (CSL_PSC_getModuleLocalResetStatus (2) == PSC_MDLRST_ASSERTED)
        {
            // Module 2's local reset asserted.
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_PSC_MDLRST CSL_PSC_getModuleLocalResetStatus  (
    Uint32                  moduleNum
)
{
    return (CSL_PSC_MDLRST) CSL_FEXT(hPscRegs->MDSTAT[moduleNum], PSC_MDSTAT_LRST);
}


/** ===========================================================================
 *   @n@b CSL_PSC_isModuleLocalResetDone
 *
 *   @b Description
 *   @n This function returns the Module local reset initialization done status.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  Bool 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDSTAT_LRSTDONE
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's local reset initialization done status
        if (CSL_PSC_isModuleLocalResetDone (2))
        {
            // Module 2's local reset init done.
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE Bool CSL_PSC_isModuleLocalResetDone  (
    Uint32                  moduleNum
)
{
    return (Bool) CSL_FEXT(hPscRegs->MDSTAT[moduleNum], PSC_MDSTAT_LRSTDONE);
}


/** ===========================================================================
 *   @n@b CSL_PSC_getModuleResetStatus
 *
 *   @b Description
 *   @n This function returns the Module reset actual status. 
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  CSL_PSC_MDRST 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDSTAT_MRST
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's reset status
        if (CSL_PSC_getModuleResetStatus (2) == PSC_MDRST_ASSERTED)
        {
            // Module 2's reset asserted.
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_PSC_MDRST CSL_PSC_getModuleResetStatus  (
    Uint32                  moduleNum
)
{
    return (CSL_PSC_MDRST) CSL_FEXT(hPscRegs->MDSTAT[moduleNum], PSC_MDSTAT_MRST);
}


/** ===========================================================================
 *   @n@b CSL_PSC_isModuleResetDone
 *
 *   @b Description
 *   @n This function returns the Module reset initialization done status.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  Bool 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDSTAT_MRSTDONE
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's reset initialization done status
        if (CSL_PSC_isModuleResetDone (2))
        {
            // Module 2's reset init done.
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE Bool CSL_PSC_isModuleResetDone  (
    Uint32                  moduleNum
)
{
    return (Bool) CSL_FEXT(hPscRegs->MDSTAT[moduleNum], PSC_MDSTAT_MRSTDONE);
}


/** ===========================================================================
 *   @n@b CSL_PSC_isModuleClockOn
 *
 *   @b Description
 *   @n This function returns the actual modclk output to module.
 *
 *   @b Arguments
 *   @verbatim
        moduleNum       Module number for which the clock status must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  Bool 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n PSC_MDSTAT_MCKOUT
 *
 *   @b Example
 *   @verbatim
        ...
        // Check Module 2's modclk status
        if (CSL_PSC_isModuleClockOn (2))
        {
            // Module 2's modclk on.
            ...
        }
        else
        {
            // Module 2's modclk gated.
            ...
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE Bool CSL_PSC_isModuleClockOn  (
    Uint32                  moduleNum
)
{
    return (Bool) CSL_FEXT(hPscRegs->MDSTAT[moduleNum], PSC_MDSTAT_MCKOUT);
}


/** ============================================================================
 *   @n@b CSL_PSC_enablePowerDomain
 *
 *   @b Description
 *   @n This function enables the specified power domain.
 *
 *   @b Arguments
 *   @verbatim
        pwrDmnNum       Power domain number that needs to be enabled.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None 
 *
 *   <b> Post Condition </b>
 *   @n  Power domain is enabled.
 *
 *   @b  Writes
 *   @n  PSC_PDCTL_NEXT=1
 *
 *   @b  Example
 *   @verbatim
        ...
        // On the power domain 2
        CSL_PSC_enablePowerDomain (2);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_enablePowerDomain (
    Uint32                  pwrDmnNum
)
{
    CSL_FINST (hPscRegs->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, ON);
    
    return;
}


/** ============================================================================
 *   @n@b CSL_PSC_disablePowerDomain
 *
 *   @b Description
 *   @n This function turns off the specified power domain.
 *
 *   @b Arguments
 *   @verbatim
        pwrDmnNum       Power domain number that needs to be disabled.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None 
 *
 *   <b> Post Condition </b>
 *   @n  Power domain is disabled.
 *
 *   @b  Writes
 *   @n  PSC_PDCTL_NEXT=0
 *
 *   @b  Example
 *   @verbatim
        ...
        // Off the power domain 2
        CSL_PSC_disablePowerDomain (2);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_disablePowerDomain (
    Uint32                  pwrDmnNum
)
{
    CSL_FINST (hPscRegs->PDCTL[pwrDmnNum], PSC_PDCTL_NEXT, OFF);
    
    return;
}



/** ===========================================================================
 *   @n@b CSL_PSC_getPowerDomainState
 *
 *   @b Description
 *   @n This function returns the current state of a given power domain.
 *
 *   @b Arguments
 *   @verbatim
        pwrDmnNum       Power domain number for which the state must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  CSL_PSC_PDSTATE 
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n Power Domain status is returned
 *
 *   @b Reads
 *   @n PSC_PDSTAT_STATE
 *
 *   @b Example
 *   @verbatim
        ...
        // Check if Power domain is On.
        if (CSL_PSC_getPowerDomainState(2) == PSC_PDSTATE_ON)
        {
            // Power domain 2 is on
            ...
        }
        else
        {
            // Power domain 2 is off
        }
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_PSC_PDSTATE CSL_PSC_getPowerDomainState  (
    Uint32                  pwrDmnNum
)
{
    return (CSL_PSC_PDSTATE) CSL_FEXT(hPscRegs->PDSTAT[pwrDmnNum], PSC_PDSTAT_STATE);
}



/** ============================================================================
 *   @n@b CSL_PSC_startStateTransition
 *
 *   @b Description
 *   @n This function sets the 'GO' bit in the PSC Command register. All state
 *      transitions marked for a specified power domain and all its modules are
 *      initiated by the hardware.
 *
 *      This function starts a given Power domain and all its modules state 
 *      transition.
 *
 *   @b Arguments       
 *   @verbatim
        pwrDmnNum       Power domain number for which the state transition 
                        must be initiated.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None 
 *
 *   <b> Post Condition </b>
 *   @n  Power domain and modules are moved to a new "Next" state as marked
 *       earlier using APIs: @a CSL_PSC_setModuleNextState  (),
 *       @a CSL_PSC_enablePowerDomain (), @a CSL_PSC_disablePowerDomain ().
 *
 *   @b  Writes
 *   @n  PSC_PTCMD
 *
 *   @b  Example
 *   @verbatim
        ...
        // To Enable Power domain 2 and modules 3, 4
        CSL_PSC_enablePowerDomain (2);
        CSL_PSC_setModuleNextState  (3, PSC_MODSTATE_ENABLE);
        CSL_PSC_setModuleNextState (4, PSC_MODSTATE_ENABLE);
        CSL_PSC_startStateTransition (2);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_PSC_startStateTransition (
    Uint32                  pwrDmnNum
)
{
    hPscRegs->PTCMD =   (1 << pwrDmnNum);
    
    return;
}

/** ===========================================================================
 *   @n@b CSL_PSC_isStateTransitionDone
 *
 *   @b Description
 *   @n This function gets the transition status of the power domain. This function
 *      returns 0 to indicate that the state transition initiated earlier using
 *      @a CSL_PSC_startStateTransition () API for the specified power domain has not
 *      yet been completed, and is in progress still. This function returns 1
 *      to indicate when this transition is completed in the hardware.
 *
 *   @b Arguments
 *   @verbatim
        pwrDmnNum       Power domain number for which the state transition status 
                        must be retrieved.
     @endverbatim
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *    @n Power domain transition status value is read
 *
 *   @b Reads
 *   @n PSC_PTSTAT
 *
 *   @b Example
 *   @verbatim
        ...
        // Ensure no transition in progress for Power domain 2 
        // before we start a new one.
        while (!CSL_PSC_isStateTransitionDone (2));

        // To Enable Power domain 2 and modules 3, 4
        CSL_PSC_enablePowerDomain (2);
        CSL_PSC_setModuleNextState (3, PSC_MODSTATE_ENABLE);
        CSL_PSC_setModuleNextState (4, PSC_MODSTATE_ENABLE);
        CSL_PSC_startStateTransition (2);

        // Wait until the transition process is completed.
        while (!CSL_PSC_isStateTransitionDone (2));
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_PSC_isStateTransitionDone (
    Uint32                  pwrDmnNum
)
{
    Uint32  pdTransStatus;

    pdTransStatus =   CSL_FEXTR (hPscRegs->PTSTAT, pwrDmnNum, pwrDmnNum);

    if (pdTransStatus)
    {
        /* Power domain transition is in progress. Return 0 to indicate not yet done. */            
        return 0;
    }
    else
    {
        /* Power domain transition is done. */            
        return 1;
    }
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_PSCAUX_H_ */

