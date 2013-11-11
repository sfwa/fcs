/**
 *   @file  csl_tmrAux.h
 *
 *   @brief   
 *      This is the TIMER Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the TIMER Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2002, 2003, 2004, 2005, 2008, 2009, Texas Instruments, Inc.
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

#ifndef _CSL_TMRAUX_H_
#define _CSL_TMRAUX_H_

#include <ti/csl/csl_tmr.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_TIMER_FUNCTION
 @{ */
        
/** ============================================================================
 *   @n@b CSL_tmrGetTimHiCount
 *
 *   @b Description
 *   @n This function gets the value of the CNTHI counter
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Pointer to the object that holds reference to the
                     instance of TIMER requested after the call
         countHi     Output parameter to get the CNTHI value
     @endverbatim
 *
 *   <b> Return Value </b>  
 *   @n The argument countHi is populated with the contents of the CNTHI register
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b  Reads
 *   @n  TMR_CNTHI_CNT
 *
 *   @b  Example
 *   @verbatim
        CSL_TmrHandle   hTmr;
        Unit32*         countHi;
        ...
        CSL_tmrGetTimHiCount(hTmr, &countHi);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_tmrGetTimHiCount (CSL_TmrHandle hTmr,Uint32 *countHi)
{
    *countHi = CSL_FEXT (hTmr->regs->CNTHI, TMR_CNTHI_CNT);
}

/** ===========================================================================
 *   @n@b CSL_tmrGetTimLoCount
 *
 *   @b Description
 *   @n This function gets the value of the CNTLO counter
 *
 *   @b Arguments
 *   @verbatim
         hTmr       Pointer to the object that holds reference to the
                    instance of TIMER requested after the call
         countLo    Output parameter to get the CNTLO value
     @endverbatim
 *
 *   <b> Return Value </b>  
 *      None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n The argument countLo is populated with the contents of the CNTLO register
 *
 *   @b Reads
 *   @n TMR_CNTLO_CNT
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        Unit32        *countLo;
        ...
        CSL_tmrGetTimLoCount(hTmr, &countLo);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE void CSL_tmrGetTimLoCount (CSL_TmrHandle hTmr,Uint32* countLo)
{
    *countLo = CSL_FEXT (hTmr->regs->CNTLO, TMR_CNTLO_CNT);
}

/** ===========================================================================
 *   @n@b CSL_tmrGetTstatLoStatus
 *
 *   @b Description
 *   @n This function gets the status of the TINTLO
 *
 *   @b Arguments
 *   @verbatim
         hTmr   Pointer to the object that holds reference to the
                instance of TIMER requested after the call
     @endverbatim
 *
 *   <b> Return Value </b>  
 *   @n CSL_TmrTstatLo - TINTLO status value
 *   @li CSL_TMR_TSTAT_LO_LOW  - Data low
 *   @li CSL_TMR_TSTAT_LO_HIGH - Data High
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n TMR_TCR_TSTAT_LO
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle  hTmr;
        CSL_TmrTstatLo statusLo;
        ...
        statusLo = CSL_tmrGetTstatLoStatus(hTmr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE CSL_TmrTstat CSL_tmrGetTstatLoStatus (CSL_TmrHandle hTmr)
{        
    return (CSL_TmrTstat)CSL_FEXT(hTmr->regs->TCR, TMR_TCR_TSTAT_LO);
}

/** ===========================================================================
 *   @n@b CSL_tmrGetTstatHiStatus
 *
 *   @b Description
 *   @n This function gets the status of the TINTHI
 *
 *   @b Arguments
 *   @verbatim
         hTmr   Pointer to the object that holds reference to the
                instance of TIMER requested after the call
     @endverbatim
 *
 *   <b> Return Value </b>  
 *   @n CSL_TmrTstatHi - TINTHI status value
 *   @li CSL_TMR_TSTATHI_LOW  - Data low
 *   @li CSL_TMR_TSTATHI_HIGH - Data High
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n TMR_TCR_TSTAT_HI
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle  hTmr;
        CSL_TmrTstatHi statusHi;
        ...
        statusHi = CSL_tmrGetTstatHiStatus(hTmr);
        ...
     @endverbatim
 * ============================================================================
 */
CSL_IDEF_INLINE CSL_TmrTstat CSL_tmrGetTstatHiStatus (CSL_TmrHandle hTmr)
{
    return (CSL_TmrTstat)CSL_FEXT(hTmr->regs->TCR, TMR_TCR_TSTAT_HI);
}

/** ===========================================================================
 *   @n@b CSL_tmrGetWdflagStatus
 *
 *   @b Description
 *   @n This function gets the status of the WDFLAG
 *
 *   @b Arguments
 *   @verbatim
         hTmr   Pointer to the object that holds reference to the
                instance of TIMER requested after the call
     @endverbatim
 *
 *   <b> Return Value </b>  
 *   @n CSL_TmrFlagBitStatus - WDFLAG status value
 *   @li  CSL_TMR_WDFLAG_NOTIMEOUT  - no watchdog timeout occurred
 *   @li  CSL_TMR_WDFLAG_TIMEOUT    - watchdog timeout occurred
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called and the
 *       Timer should be set in WATCHDOG mode
 *
 *   <b> Post Condition </b>
 *    @n None
 *
 *   @b Reads
 *   @n TMR_WDTCR_WDFLAG
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle          hTmr;
        CSL_TmrWdflagBitStatus status;
        ...
        status = CSL_tmrGetWdflagStatus(hTmr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE CSL_TmrWdflagBitStatus CSL_tmrGetWdflagStatus (CSL_TmrHandle hTmr)
{
    return (CSL_TmrWdflagBitStatus)CSL_FEXT(hTmr->regs->WDTCR, TMR_WDTCR_WDFLAG);
}

/** ============================================================================
 *   @n@b CSL_TmrLoadPrdLo
 *
 *   @b Description
 *      Loads the General purpose timer LOW period register.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Gptimer instance
         loadVal     Value to be loaded to the Gptimer period register Low
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n Period register is loaded with the given value.
 *
 *   @b Writes
 *   @n TMR_PRDLO_PRDLO
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        Uint32       loadVal;

        loadVal = 10;
        CSL_TmrLoadPrdLo(hWdt, &loadVal);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrLoadPrdLo (CSL_TmrHandle hTmr,Uint32* loadVal)
{
    CSL_FINS (hTmr->regs->PRDLO, TMR_PRDLO_PRDLO, *loadVal); 
}

/** ============================================================================
 *   @n@b CSL_TmrLoadPrdHi
 *
 *   @b Description
 *      Loads the General purpose timer HIGH period register 
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the GPtimer instance
         loadVal     Value to be loaded to the Gptimer period register High
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  Period register is loaded with the given value.
 *
 *   @b Writes
 *   @n TMR_PRDHI_PRDHI
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle   hTmr;
        Uint32          loadVal;
        ...
        loadVal = 10;
        CSL_TmrLoadPrdHi(hWdt, &loadVal);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrLoadPrdHi (CSL_TmrHandle hTmr,Uint32* loadVal)
{
    CSL_FINS (hTmr->regs->PRDHI, TMR_PRDHI_PRDHI, *loadVal); 
}

/** ============================================================================
 *   @n@b CSL_TmrLoadPrescalarHi
 *
 *   @b Description
 *      Configures the prescalar values for the HIGH Timer.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer Instance
         loadVal     Value to be loaded to the PRESCALAR bits in the TGCR register
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  Gptimer pre scalar register is loaded with the given value.
 *
 *   @b Writes
 *   @n TMR_TGCR_PSCHI
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        Uint8         *loadVal;
        ...
        CSL_TmrLoadPrescalarHi(hWdt, &loadVal);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrLoadPrescalarHi (CSL_TmrHandle hTmr, Uint8* loadVal)
{
    CSL_FINS(hTmr->regs->TGCR, TMR_TGCR_PSCHI, *loadVal);
}

/** ============================================================================
 *   @n@b CSL_TmrStartHi
 *
 *   @b Description
 *      The functions starts the HIGH timer.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer instance
         countMode   Specifies the timer counting mode (OFF/CONTINIOUS/SINGLE Shot)
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The HIGH timer is brought out of reset and is activated in the 
 *       provided counting mode.
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMHIRS=1,TMR_TCR_ENAMODE_HI
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle  hTmr;
        ...
        CSL_TmrStartHi(hTmr, CSL_TMR_ENAMODE_ENABLE);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStartHi (CSL_TmrHandle hTmr, CSL_TmrEnamode countMode)
{
    /* Bring the HIGH Timer out of reset. */
    CSL_FINST(hTmr->regs->TGCR, TMR_TGCR_TIMHIRS, RESET_OFF);

    /* Configure the HIGH TIMER with the appropriate counting mode. */
    CSL_FINS(hTmr->regs->TCR, TMR_TCR_ENAMODE_HI, countMode);
    return;
}

/** ============================================================================
 *   @n@b CSL_TmrStartLo
 *
 *   @b Description
 *      The function starts the LOW Timer.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Gptimer instance
         countMode   Specifies the timer counting mode (OFF/CONTINIOUS/SINGLE Shot)
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The LOW timer is brought out of reset and is activated in the 
 *       provided counting mode.
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMLORS=1,TMR_TCR_ENAMODE_LO
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle  hTmr;
        ...
        CSL_TmrStartLo(hTmr, CSL_TMR_ENAMODE_ENABLE);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStartLo (CSL_TmrHandle hTmr, CSL_TmrEnamode countMode)
{
    /* Bring the LOW Timer out of reset. */
    CSL_FINST(hTmr->regs->TGCR, TMR_TGCR_TIMLORS, RESET_OFF);

    /* Configure the LOW TIMER with the appropriate counting mode. */
    CSL_FINS(hTmr->regs->TCR, TMR_TCR_ENAMODE_LO, countMode);
}

/** ===========================================================================
 *   @n@b CSL_TmrStart64
 *
 *   @b Description
 *      The function starts the 64bit Timer by bringing both the LOW and HIGH
 *      timer out of reset.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Gptimer instance
         countMode   Specifies the timer counting mode (OFF/CONTINIOUS/SINGLE Shot)
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n   Set the GP OR chained timer counting mode and
 *		  timer Low and High reset register.
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMLORS=1,TMR_TGCR_TIMHIRS=1,
 *   @n TMR_TCR_ENAMODE_LO,TMR_TCR_ENAMODE_HI
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle  hTmr;
        ...
        CSL_TmrStart64(hTmr, CSL_TMR_ENAMODE_ENABLE);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStart64 (CSL_TmrHandle  hTmr, CSL_TmrEnamode countMode)
{
    Uint32 tmpReg;

    /* Bring the LOW and HIGH Timer outs of reset. */
    tmpReg = hTmr->regs->TGCR;
    CSL_FINST(tmpReg, TMR_TGCR_TIMLORS, RESET_OFF);
    CSL_FINST(tmpReg, TMR_TGCR_TIMHIRS, RESET_OFF);
    hTmr->regs->TGCR = tmpReg; 

    /* Configure the counting mode for both the LOW and HIGH Timers. */
    tmpReg = hTmr->regs->TCR;
    CSL_FINS(tmpReg, TMR_TCR_ENAMODE_LO, countMode);
    CSL_FINS(tmpReg, TMR_TCR_ENAMODE_HI, countMode);
    hTmr->regs->TCR = tmpReg;
}

/** ===========================================================================
 *   @n@b CSL_TmrStopLo
 *
 *   @b Description
 *      The function stops the LOW Timer
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  Low Timer is disabled.
 *
 *   @b Writes
 *   @n TMR_TCR_ENAMODE_LO=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        ...
        CSL_TmrStopLo(hTmr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStopLo (CSL_TmrHandle hTmr)
{
    CSL_FINST(hTmr->regs->TCR, TMR_TCR_ENAMODE_LO, DISABLE);
}

/** ===========================================================================
 *   @n@b CSL_TmrStopHi
 *
 *   @b Description
 *      The function stops the HIGH Timer
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  High Timer is disabled.
 *
 *   @b Writes
 *   @n TMR_TCR_ENAMODE_HI=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        ...
        CSL_TmrStopHi(hTmr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStopHi (CSL_TmrHandle hTmr)
{
    CSL_FINST(hTmr->regs->TCR, TMR_TCR_ENAMODE_HI, DISABLE);
}

/** ===========================================================================
 *   @n@b CSL_TmrStop64
 *
 *   @b Description
 *      The function stops the 64bit timer.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The 64bit timer is stopped 
 *
 *   @b Writes
 *   @n TMR_TCR_ENAMODE_LO=0,TMR_TCR_ENAMODE_HI=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        ...
        CSL_TmrStop64(hWdt);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStop64 (CSL_TmrHandle hTmr)
{
    Uint32 tmpReg;

    /* Disable the LOW and HIGH Timers. */
    tmpReg = hTmr->regs->TCR;
    CSL_FINST(tmpReg, TMR_TCR_ENAMODE_LO, DISABLE);
    CSL_FINST(tmpReg, TMR_TCR_ENAMODE_HI, DISABLE);
    hTmr->regs->TCR = tmpReg;
}

/** ===========================================================================
 *   @n@b CSL_TmrResetHi
 *
 *   @b Description
 *      The HIGH timer is moved to RESET state.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the HIGH timer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n High timer is reset. 
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMHIRS=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        ...
        CSL_TmrResetHi(hTmr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrResetHi (CSL_TmrHandle hTmr)
{
    CSL_FINST(hTmr->regs->TGCR, TMR_TGCR_TIMHIRS, RESET_ON);
}

/** ===========================================================================
 *   @n@b CSL_TmrResetLo
 *
 *   @b Description
 *      The LOW timer is moved to RESET state.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Gptimer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n Low Timer is Reset 
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMLORS=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        ...
        CSL_TmrResetLo(hWdt);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrResetLo (CSL_TmrHandle hTmr)
{
    CSL_FINST(hTmr->regs->TGCR, TMR_TGCR_TIMLORS, RESET_ON);
}

/** ===========================================================================
 *   @n@b CSL_TmrReset64
 *
 *   @b Description
 *      The function resets the 64bit timer.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  64bit Timer is reset.
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMLORS=0,TMR_TGCR_TIMHIRS=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hTmr;
        ...
        CSL_TmrReset64(hTmr);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrReset64 (CSL_TmrHandle hTmr)
{
    Uint32 tmpReg = hTmr->regs->TGCR;

    /* Move both the Low and High Timers into reset. */
    CSL_FINST(tmpReg, TMR_TGCR_TIMLORS, RESET_ON);
    CSL_FINST(tmpReg, TMR_TGCR_TIMHIRS, RESET_ON);
    hTmr->regs->TGCR = tmpReg;
}

/** ===========================================================================
 *   @n@b CSL_TmrStartWdt
 *
 *   @b Description
 *      The function starts the timer in Watch Dog Mode.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Timer instance
         countMode   Specifies the timer counting mode (OFF/CONTINIOUS/SINGLE Shot)
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n  Timer has been started in WATCH Dog Mode.
 *
 *   @b Writes
 *   @n TMR_TGCR_TIMLORS=1,TMR_TGCR_TIMHIRS=1,TMR_WDTCR_WDEN=1,TMR_TCR_ENAMODE_LO,TMR_TCR_ENAMODE_HI;
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle  hWdt;
        ...
        CSL_TmrStartWdt(hWdt, CSL_TMR_ENAMODE_CONT);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrStartWdt (CSL_TmrHandle  hTmr,CSL_TmrEnamode countMode)
{
    Uint32 tmpReg = hTmr->regs->TGCR;

    /* Bring the LOW and HIGH Timers out of reset. */
    CSL_FINST(tmpReg, TMR_TGCR_TIMLORS, RESET_OFF);
    CSL_FINST(tmpReg, TMR_TGCR_TIMHIRS, RESET_OFF);
    hTmr->regs->TGCR = tmpReg;

    /* Enable the Watch Dog Timer */
    CSL_FINST(hTmr->regs->WDTCR, TMR_WDTCR_WDEN, ENABLE);

    /* Enable the LOW and HIGH Timers to operate */
    tmpReg = hTmr->regs->TCR;
    CSL_FINS(tmpReg, TMR_TCR_ENAMODE_LO, countMode);
    CSL_FINS(tmpReg, TMR_TCR_ENAMODE_HI, countMode);
    hTmr->regs->TCR = tmpReg;
}

/** ===========================================================================
 *   @n@b CSL_TmrLoadtWdkey
 *
 *   @b Description
 *      The function loads the watch dog timer with the appropriate key.
 *
 *   @b Arguments
 *   @verbatim
         hTmr        Handle to the Gptimer instance
         cmd         Specifies the WATCHDOG Key. This can only be either of 
                     the following:-
                        - CSL_TMR_WDTCR_WDKEY_CMD1
                        - CSL_TMR_WDTCR_WDKEY_CMD2
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n   Loads the  WDKEY in WDTCR register.
 *
 *   @b Writes
 *   @n TMR_WDTCR_WDKEY
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hWdt;
        ...
        CSL_TmrLoadWdkey(hWdt, CSL_TMR_WDTCR_WDKEY_CMD1);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrLoadWdkey (CSL_TmrHandle hTmr, Uint16 cmd)
{
    CSL_FINS(hTmr->regs->WDTCR, TMR_WDTCR_WDKEY, cmd);
}

/** ===========================================================================
 *   @n@b CSL_TmrDisableWdt
 *
 *   @b Description
 *      The function disables the Watch Dog Timer.
 *
 *   @b Arguments
 *   @verbatim
        hTmr         Handle to the Timer instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *       None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a CSL_tmrInit() and @a CSL_tmrOpen() must be called.
 *
 *   <b> Post Condition </b>
 *   @n   The Watch Dog timer is disabled.
 *
 *   @b Writes
 *   @n TMR_WDTCR_WDEN=0
 *
 *   @b Example
 *   @verbatim
        CSL_TmrHandle hWdt;
        ...
        CSL_TmrDisableWdt(hWdt);
        ...
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void CSL_TmrDisableWdt (CSL_TmrHandle hTmr)
{
    CSL_FINST(hTmr->regs->WDTCR, TMR_WDTCR_WDEN, DISABLE);
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_TMRAUX_H_ */

