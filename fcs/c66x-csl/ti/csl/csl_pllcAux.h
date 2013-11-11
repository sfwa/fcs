/**
 *   @file  csl_pllcAux.h
 *
 *   @brief API Auxilary header file for PLLC CSL
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010 Texas Instruments, Inc.
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
#ifndef _CSL_PLLCAUX_H_
#define _CSL_PLLCAUX_H_

#include <ti/csl/csl_pllc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_PLLC_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_PLLC_getResetStatus
 *
 *   @b Description
 *   @n Gets the Reset Type Status of the PLLC.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n RSTYPE
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint32          response;
        ...

        response = CSL_pllcGetResetStatus (hPllc);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_PLLC_getResetStatus (CSL_PllcHandle   hPllc)
{
   	return hPllc->RSTYPE;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setResetCtrlReg
 *
 *   @b Description
 *   @n Sets up the Key and Software Reset bit in Reset control register contents.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            key             Key value to setup
            swRstEnable     Enable/disable software reset
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_RSTCTRL_KEY,
 *      PLLC_RSTCTRL_SWRST
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setResetCtrlReg (hPllc, CSL_PLLC_RSTCTRL_VALID_KEY, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setResetCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint16              key,
    Uint8               swRstEnable
)
{
   	hPllc->RSTCTRL  =   CSL_FMK (PLLC_RSTCTRL_KEY, key) |
                        CSL_FMK (PLLC_RSTCTRL_SWRST, swRstEnable);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getResetCtrlReg
 *
 *   @b Description
 *   @n Retrieves the contents of the Reset control register
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pKey            Key value read
            pSwRstEnable    Software reset enable bit read
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_RSTCTRL_KEY,
 *      PLLC_RSTCTRL_SWRST
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint16          key;
        Uint8           swRstEnable
        ...

        CSL_PLLC_getResetCtrlReg (hPllc, &key, &swRstEnable);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getResetCtrlReg 
(
    CSL_PllcHandle      hPllc, 
    Uint16*             pKey,
    Uint8*              pSwRstEnable
)
{
    Uint32              regVal;        

    regVal  =   hPllc->RSTCTRL;

    *pKey           =   CSL_FEXT (regVal, PLLC_RSTCTRL_KEY);
    *pSwRstEnable   =   CSL_FEXT (regVal, PLLC_RSTCTRL_SWRST);

   	return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setResetCfgReg
 *
 *   @b Description
 *   @n Sets up the contents of Reset configuration register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            wdType          Reset type initiated by Watchdog timers. Set 0 for hard
                            reset and 1 for soft reset
            resetType       Reset type initiated by RESET. Set 0 for hard
                            reset and 1 for soft reset
            pllCtrlRstType  Reset type initiated by PLL controller. Set 0 for hard
                            reset and 1 for soft reset
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API. Setup a 
 *       valid key using @a CSL_PLLC_setResetCtrlReg () API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_RSTCFG_WDTYPEN,
 *      PLLC_RSTCFG_RESETTYPE,
 *      PLLC_RSTCFG_PLLCTLRSTTYPE
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setResetCfgReg (hPllc, 0, 0, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setResetCfgReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               wdType,
    Uint8               resetType,
    Uint8               pllCtrlRstType
)
{
   	hPllc->RSTCFG   =   CSL_FMK (PLLC_RSTCFG_WDTYPEN, wdType) |
                        CSL_FMK (PLLC_RSTCFG_RESETTYPE, resetType) |
                        CSL_FMK (PLLC_RSTCFG_PLLCTLRSTTYPE, pllCtrlRstType);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getResetCfgReg
 *
 *   @b Description
 *   @n Retrieves the contents of Reset configuration register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pWdType         Reset type initiated by Watchdog timers. Returns 0 for hard
                            reset and 1 for soft reset
            pResetType      Reset type initiated by RESET. Returns 0 for hard
                            reset and 1 for soft reset
            pPllCtrlRstType Reset type initiated by PLL controller. Returns 0 for hard
                            reset and 1 for soft reset
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_RSTCFG_WDTYPEN,
 *      PLLC_RSTCFG_RESETTYPE,
 *      PLLC_RSTCFG_PLLCTLRSTTYPE
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           wdType, resetType, pllCtrlRstType;

        ...

        CSL_PLLC_getResetCfgReg (hPllc, &wdType, &resetType, &pllCtrlRstType);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getResetCfgReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pWdType,
    Uint8*              pResetType,
    Uint8*              pPllCtrlRstType
)
{
    Uint32              regVal;

    regVal  =   hPllc->RSTCFG;
    
    *pWdType            =   CSL_FEXT (regVal, PLLC_RSTCFG_WDTYPEN);
    *pResetType         =   CSL_FEXT (regVal, PLLC_RSTCFG_RESETTYPE);
    *pPllCtrlRstType    =   CSL_FEXT (regVal, PLLC_RSTCFG_PLLCTLRSTTYPE);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setResetIsoReg
 *
 *   @b Description
 *   @n Sets up the contents of Reset Islolation register. Can be used to setup
 *      the module clocks that need to be work without pausing through non Power-on
 *      reset.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            aif2Iso         Isolate AIF2 module? Set to 1 to isolate it
            srIso           Isolate Smart Reflex module? Set to 1 to isolate it
            srioIso         Isolate SRIO module? Set to 1 to isolate it
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_RSISO_AIF2ISO,
 *      PLLC_RSISO_SRISO,
 *      PLLC_RSISO_SRIOISO
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setResetIsoReg (hPllc, 0, 0, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setResetIsoReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               aif2Iso,
    Uint8               srIso,
    Uint8               srioIso
)
{
   	hPllc->RSISO    =   CSL_FMK (PLLC_RSISO_AIF2ISO, aif2Iso) |
                        CSL_FMK (PLLC_RSISO_SRISO, srIso) |
                        CSL_FMK (PLLC_RSISO_SRIOISO, srioIso);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getResetIsoReg
 *
 *   @b Description
 *   @n Retrieves the contents of Reset Islolation register. Indicates 
 *      the module clocks that are setup to work without pausing through non Power-on
 *      reset.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pAif2Iso        Indicates if AIF2 module is reset isolated
            pSrIso          Indicates if Smart Reflex module is reset isolated
            pSrioIso        Indicates if SRIO module is reset isolated
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_RSISO_AIF2ISO,
 *      PLLC_RSISO_SRISO,
 *      PLLC_RSISO_SRIOISO
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           aif2Iso, srIso, srioIso;

        ...

        CSL_PLLC_getResetIsoReg (hPllc, &aif2Iso, &srIso, &srioIso);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getResetIsoReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pAif2Iso,
    Uint8*              pSrIso,
    Uint8*              pSrioIso
)
{
    Uint32              regVal;

    regVal  =   hPllc->RSISO;
    
    *pAif2Iso   =   CSL_FEXT (regVal, PLLC_RSISO_AIF2ISO);
    *pSrIso     =   CSL_FEXT (regVal, PLLC_RSISO_SRISO);
    *pSrioIso   =   CSL_FEXT (regVal, PLLC_RSISO_SRIOISO);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllCtrlPllEnSrc
 *
 *   @b Description
 *   @n Sets up the PLLENSRC bit of PLL Control Register. Can be used to enable/disable
 *      configuration of PLLEN bit of PLLCTL register. Writes to PLLEN bit take 
 *      effect on PLLC only when PLLENSRC bit is set to 0.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            value           0/1 value to configure in PLLENSRC bit
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLCTL_PLLENSRC
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        // Enable PLLEN bit configuration
        CSL_PLLC_setPllCtrlPllEnSrc (hPllc, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllCtrlPllEnSrc 
(
    CSL_PllcHandle      hPllc,
    Uint8               value  
)
{
    CSL_FINS (hPllc->PLLCTL, PLLC_PLLCTL_PLLENSRC, value);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllCtrlPllEn
 *
 *   @b Description
 *   @n Sets the PLLEN bit of PLL Control Register. This bit must be set to 0
 *      to put PLLC in Bypass mode and to 1 to put it in PLL operational mode.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            value           0/1 value to configure in PLL Enable bit. Set to 0 
                            to put PLLC in Bypass mode and to 1 to put in PLL mode.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *       Enable configuration of PLLEN bit first using @a CSL_PLLC_setPllCtrlPllEnSrc () API
 *       by passing it a value 0.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLCTL_PLLEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        // Enable PLLEN bit configuration
        CSL_PLLC_setPllCtrlPllEnSrc (hPllc, 0);

        // Put PLLC in Bypass mode 
        CSL_PLLC_setPllCtrlPllEn (hPllc, 0);

        // Configure PLLM/Pre-Divider
        ...

        // Put PLLC back in PLL mode
        CSL_PLLC_setPllCtrlPllEn (hPllc, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllCtrlPllEn 
(
    CSL_PllcHandle      hPllc,
    Uint8               value
)
{
    CSL_FINS (hPllc->PLLCTL, PLLC_PLLCTL_PLLEN, value);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllCtrlPllReset
 *
 *   @b Description
 *   @n Sets up the PLLRST bit of PLL Control Register. Must be set to 1 to
 *      put PLLC in reset mode and to 0 to bring it out of reset.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            value           Value to configure in PLL Reset bit. Set to 1 to assert 
                            reset and 0 to release it from reset.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *       PLLC must be put in Bypass mode by passing a value 0 to @a CSL_PLLC_setPllCtrlPllEn () 
 *       API before asserting/de-asserting reset using this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLCTL_PLLRST
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        // Enable PLLEN bit configuration
        CSL_PLLC_setPllCtrlPllEnSrc (hPllc, 0);

        // Put PLLC in Bypass mode 
        CSL_PLLC_setPllCtrlPllEn (hPllc, 0);

        // Put PLLC in reset
        CSL_PLLC_setPllCtrlPllReset (hPllc, 1);

        // Do PLLC configuration 
        ...

        // Bring PLLC out of reset
        CSL_PLLC_setPllCtrlPllReset (hPllc, 0);

        // Put PLLC back in PLL mode
        CSL_PLLC_setPllCtrlPllEn (hPllc, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllCtrlPllReset 
(
    CSL_PllcHandle      hPllc,
    Uint8               value
)
{
    CSL_FINS (hPllc->PLLCTL, PLLC_PLLCTL_PLLRST, value);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllCtrlPllPowerDown
 *
 *   @b Description
 *   @n Sets up the PLLPWRDWN bit of PLL Control Register. Must be set to 1 to
 *      power down PLL and to 0 to wake up the PLL.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            value           Value to configure in PLL Power-down mode select bit. 
                            Set to 1 to place PLLC in power-down mode and to 0 to
                            wake it up.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *       PLLC must be put in Bypass mode by passing a value 0 to @a CSL_PLLC_setPllCtrlPllEn () 
 *       API before powering up/down using this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLCTL_PLLPWRDN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        // Enable PLLEN bit configuration
        CSL_PLLC_setPllCtrlPllEnSrc (hPllc, 0);

        // Put PLLC in Bypass mode 
        CSL_PLLC_setPllCtrlPllEn (hPllc, 0);

        // Power down PLL
        CSL_PLLC_setPllCtrlPllPowerDown (hPllc, 1);

        ...

        // Wake up PLL
        CSL_PLLC_setPllCtrlPllPowerDown (hPllc, 0);

        // Put PLLC back in PLL mode
        CSL_PLLC_setPllCtrlPllEn (hPllc, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllCtrlPllPowerDown 
(
    CSL_PllcHandle      hPllc,
    Uint8               value
)
{
    CSL_FINS (hPllc->PLLCTL, PLLC_PLLCTL_PLLPWRDN, value);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllCtrlReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Control Register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pllEn           PLL Enable bit. Set to 0 to put PLLC in Bypass mode and 
                            to 1 to put in PLL mode.
            pllPwrDwn       PLL Power-down mode select bit. Set to 1 to place PLLC
                            in power-down mode.
            pllRst          PLL Reset bit. Set to 1 to assert reset and 0 to release 
                            it from reset.
            pllEnSrc        PLL Enable Source bit. Set to 1 to enable writes to PLLEN
                            bit.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLCTL_PLLEN,
 *      PLLC_PLLCTL_PLLPWRDN,
 *      PLLC_PLLCTL_PLLRST,
 *      PLLC_PLLCTL_PLLENSRC
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setPllCtrlReg (hPllc, 0, 0, 1, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               pllEn,
    Uint8               pllPwrDwn,
    Uint8               pllRst,
    Uint8               pllEnSrc  
)
{
    /* Enable PLLEN bit if requested */                
    CSL_FINS (hPllc->PLLCTL, PLLC_PLLCTL_PLLENSRC, pllEnSrc);

    /* Configure rest of the bits in the register now */
   	hPllc->PLLCTL   =   CSL_FMK (PLLC_PLLCTL_PLLEN, pllEn) |
                        CSL_FMK (PLLC_PLLCTL_PLLPWRDN, pllPwrDwn) |
                        CSL_FMK (PLLC_PLLCTL_PLLRST, pllRst);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllCtrlReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Control Register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pPllEn          PLL Enable bit. 0 indicates Bypass mode and 1 indicates 
                            PLL mode.
            pPllPwrDwn      PLL Power-down mode select bit. 1 indicates PLLC is
                            powered down and 0 indicates it is operational.
            pPllRst         PLL Reset bit. 1 indicates reset is asserted and 0 indicates
                            reset is cleared.
            pPllEnSrc       PLL Enable Source bit. Indicates if PLLEN bit is enabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_PLLCTL_PLLEN,
 *      PLLC_PLLCTL_PLLPWRDN,
 *      PLLC_PLLCTL_PLLRST,
 *      PLLC_PLLCTL_PLLENSRC
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           pllen, pllPwrDwn, pllRst, pllSrcEn;

        ...

        CSL_PLLC_getPllCtrlReg (hPllc, &pllen, &pllPwrDwn, &pllRst, &pllSrcEn);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pPllEn,
    Uint8*              pPllPwrDwn,
    Uint8*              pPllRst,
    Uint8*              pPllEnSrc  
)
{
    Uint32              regVal;

    regVal  =   hPllc->PLLCTL;
    
    *pPllEn     =   CSL_FEXT (regVal, PLLC_PLLCTL_PLLEN);
    *pPllPwrDwn =   CSL_FEXT (regVal, PLLC_PLLCTL_PLLPWRDN);
    *pPllRst    =   CSL_FEXT (regVal, PLLC_PLLCTL_PLLRST);
    *pPllEnSrc  =   CSL_FEXT (regVal, PLLC_PLLCTL_PLLENSRC);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllSecCtrlReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Secondary Control Register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pllSecCtrlVal   Value to configure in the PLL Secondary control
                            register.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_SECCTL_PLLSECCTL
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setPllSecCtrlReg (hPllc, 0x10);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllSecCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               pllSecCtrlVal
)
{
    CSL_FINS (hPllc->SECCTL, PLLC_SECCTL_PLLSECCTL, pllSecCtrlVal);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllSecCtrlReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Secondary Control Register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Uint8
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_SECCTL_PLLSECCTL
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           pllSecCtrlVal;

        ...

        pllSecCtrlVal   =   CSL_PLLC_getPllSecCtrlReg (hPllc);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_PLLC_getPllSecCtrlReg 
(
    CSL_PllcHandle      hPllc
)
{
    return CSL_FEXT (hPllc->SECCTL, PLLC_SECCTL_PLLSECCTL);
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllMultiplierCtrlReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Multiplier Control Register. 
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pllmVal         multiplier value to configure.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *       PLLC must be put in Bypass mode by passing a value 0 to @a CSL_PLLC_setPllCtrlPllEn () 
 *       API before setting up the multiplier using this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLM_PLLM
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        // Enable PLLEN bit configuration
        CSL_PLLC_setPllCtrlPllEnSrc (hPllc, 0);

        // Put PLLC in Bypass mode 
        CSL_PLLC_setPllCtrlPllEn (hPllc, 0);

        // Configure PLLM/Pre-Divider
        // Setup x1 multiplier rate
        CSL_PLLC_setPllMultiplierCtrlReg (hPllc, 0);

        ...

        // Put PLLC back in PLL mode
        CSL_PLLC_setPllCtrlPllEn (hPllc, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllMultiplierCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               pllmVal
)
{
    CSL_FINS (hPllc->PLLM, PLLC_PLLM_PLLM, pllmVal);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllMultiplierCtrlReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Multiplier Control Register. 
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Uint8
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_PLLM_PLLM
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           pllmVal;

        ...

        pllmVal = CSL_PLLC_getPllMultiplierCtrlReg (hPllc);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_PLLC_getPllMultiplierCtrlReg 
(
    CSL_PllcHandle      hPllc
)
{
    return CSL_FEXT (hPllc->PLLM, PLLC_PLLM_PLLM);
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllPreDivReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Pre-Divider Register. 
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            preDivEnable    Enable/disable the pre-divider
            preDivRatio     Divider ratio bits       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *       PLLC must be put in Bypass mode by passing a value 0 to @a CSL_PLLC_setPllCtrlPllEn () 
 *       API before setting up the pre-divider using this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PREDIV_RATIO,
 *      PLLC_PREDIV_PREDEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        // Enable PLLEN bit configuration
        CSL_PLLC_setPllCtrlPllEnSrc (hPllc, 0);

        // Put PLLC in Bypass mode 
        CSL_PLLC_setPllCtrlPllEn (hPllc, 0);

        // Configure Pre-Divider
        // Setup /1 divider rate and enable divider
        CSL_PLLC_setPllPreDivReg (hPllc, 1, 0);

        ...

        // Put PLLC back in PLL mode
        CSL_PLLC_setPllCtrlPllEn (hPllc, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllPreDivReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               preDivEnable,
    Uint8               preDivRatio
)
{
    hPllc->PREDIV   =   CSL_FMK (PLLC_PREDIV_PREDEN, preDivEnable) |
                        CSL_FMK (PLLC_PREDIV_RATIO, preDivRatio);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllPreDivReg
 *
 *   @b Description
 *   @n Returns the contents of PLL Pre-Divider Register. 
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pPreDivEnable   Indicates if pre-divider is enabled
            pPreDivRatio    Divider ratio bits       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_PREDIV_RATIO,
 *      PLLC_PREDIV_PREDEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           preDivEnable, preDivRatio;

        ...

        CSL_PLLC_getPllPreDivReg (hPllc, &preDivEnable, &preDivRatio);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllPreDivReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pPreDivEnable,
    Uint8*              pPreDivRatio
)
{
    Uint32              regVal;

    regVal  =   hPllc->PREDIV;
    
    *pPreDivEnable  =   CSL_FEXT (regVal, PLLC_PREDIV_PREDEN);
    *pPreDivRatio   =   CSL_FEXT (regVal, PLLC_PREDIV_RATIO);

    return;
}


/** ============================================================================
 *   @n@b CSL_PLLC_setPllDivReg
 *
 *   @b Description
 *   @n Sets up the contents of any given PLL Divider Register corresponding to 
 *      the divider number specified.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            divNum          PLLC Divider register number. The divider number is 
                            1 based.
            divEnable       Enable/disable the divider
            divRatio        Divider ratio bits       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *       Ensure @a CSL_PLLC_getPllStatusReg () returns 0 for GOSTAT bit status before
 *       configuring the Dividers.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLDIV1_3_DNEN,
 *      PLLC_PLLDIV1_3_RATIO;
 *      PLLC_PLLDIV4_16_DNEN, 
 *      PLLC_PLLDIV4_16_RATIO
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           goStatus;

        // Ensure no GO operation in progress already
        CSL_PLLC_getPllStatusReg (hPllc, &goStatus);
        while (goStatus != 0)
        {
            // wait some time and recheck GOSTAT status
            ...

            CSL_PLLC_getPllStatusReg (hPllc, &goStatus);
        }

        // Setup /1 divider rate and enable divider 1
        CSL_PLLC_setPllDivReg (hPllc, 1, 0);

        // Set the respective ALNn bit in ALNCTL register
        CSL_PLLC_setPllAlignCtrlReg (hPllc, 1);

        // Start GO operation
        CSL_PLLC_setPllCmdReg (hPllc, 1);

        // Ensure GO operation completes
        CSL_PLLC_getPllStatusReg (hPllc, &goStatus);
        while (goStatus != 0)
        {
            // wait some time and recheck GOSTAT status
            ...

            CSL_PLLC_getPllStatusReg (hPllc, &goStatus);
        }
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllDivReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               divNum,
    Uint8               divEnable,
    Uint8               divRatio
)
{
    if (divNum <= 3)
    {
        hPllc->PLLDIV1_3 [divNum - 1]   =   CSL_FMK (PLLC_PLLDIV1_3_DNEN, divEnable) |
                                            CSL_FMK (PLLC_PLLDIV1_3_RATIO, divRatio);
    }
    else
    {
        hPllc->PLLDIV4_16 [divNum - 3 - 1]  =   CSL_FMK (PLLC_PLLDIV4_16_DNEN, divEnable) |
                                                CSL_FMK (PLLC_PLLDIV4_16_RATIO, divRatio);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllDivReg
 *
 *   @b Description
 *   @n Returns the contents of PLL Divider Register corresponding to a given
 *      divider number.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            divNum          PLLC Divider register number. The divider number is 
                            1 based.
            pDivEnable      Indicates if divider is enabled
            pDivRatio       Divider ratio bits       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_PLLDIV1_3_DNEN,
 *      PLLC_PLLDIV1_3_RATIO;
 *      PLLC_PLLDIV4_16_DNEN, 
 *      PLLC_PLLDIV4_16_RATIO
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           divEnable, divRatio;

        ...

        // Get div 3 settings
        CSL_PLLC_getPllDivReg (hPllc, 3, &divEnable, &divRatio);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllDivReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               divNum,
    Uint8*              pDivEnable,
    Uint8*              pDivRatio
)
{
    Uint32              regVal;

    if (divNum <= 3)
    {
        regVal  =   hPllc->PLLDIV1_3 [divNum - 1];
    
        *pDivEnable =   CSL_FEXT (regVal, PLLC_PLLDIV1_3_DNEN);
        *pDivRatio  =   CSL_FEXT (regVal, PLLC_PLLDIV1_3_RATIO);
    }
    else
    {
        regVal  =   hPllc->PLLDIV4_16 [divNum - 3 - 1];
    
        *pDivEnable =   CSL_FEXT (regVal, PLLC_PLLDIV4_16_DNEN);
        *pDivRatio  =   CSL_FEXT (regVal, PLLC_PLLDIV4_16_RATIO);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllPostDivReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Post-Divider Register. 
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            postDivEnable   Enable/disable the post-divider
            postDivRatio    Divider ratio bits       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_POSTDIV_RATIO,
 *      PLLC_POSTDIV_POSTDEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        // Setup /1 divider rate and enable post divider
        CSL_PLLC_setPllPostDivReg (hPllc, 1, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllPostDivReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               postDivEnable,
    Uint8               postDivRatio
)
{
    hPllc->POSTDIV  =   CSL_FMK (PLLC_POSTDIV_POSTDEN, postDivEnable) |
                        CSL_FMK (PLLC_POSTDIV_RATIO, postDivRatio);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllPostDivReg
 *
 *   @b Description
 *   @n Returns the contents of PLL Post-Divider Register. 
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pPostDivEnable  Indicates if post-divider is enabled
            pPostDivRatio   Divider ratio bits       
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_POSTDIV_RATIO,
 *      PLLC_POSTDIV_POSTDEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           postDivEnable, postDivRatio;

        ...

        CSL_PLLC_getPllPostDivReg (hPllc, &postDivEnable, &postDivRatio);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllPostDivReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pPostDivEnable,
    Uint8*              pPostDivRatio
)
{
    Uint32              regVal;

    regVal  =   hPllc->POSTDIV;
    
    *pPostDivEnable =   CSL_FEXT (regVal, PLLC_POSTDIV_POSTDEN);
    *pPostDivRatio  =   CSL_FEXT (regVal, PLLC_POSTDIV_RATIO);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllCmdReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Command register. Can be used to enable the 
 *      GOSET bit in PLLC Command register to initiate the GO operation or to
 *      clear it.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            goSetEnable     Set to 1 to enable GOSET bit and start GO operation 
                            and 0 to clear the bit.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_PLLCMD_GOSET
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        // Enable GO operation
        CSL_PLLC_setPllCmdReg (hPllc, 1);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllCmdReg 
(
    CSL_PllcHandle      hPllc,
    Uint8               goSetEnable
)
{
    CSL_FINS (hPllc->PLLCMD, PLLC_PLLCMD_GOSET, goSetEnable);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllCmdReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Command register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pGoSetEnable    GOSET bit status.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_PLLCMD_GOSET
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           goSetEnable;
        ...

        CSL_PLLC_getPllCmdReg (hPllc, &goSetEnable);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllCmdReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pGoSetEnable
)
{
    *pGoSetEnable   =   CSL_FEXT (hPllc->PLLCMD, PLLC_PLLCMD_GOSET);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllStatusReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Status register. Indicates the status 
 *      of the GO operation.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pGoStatus       GO operation status. 1 indicates GO operation is
                            in progress. 0 indicates GO operation is not in progress.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_PLLSTAT_GOSTAT
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           goStatus;
        ...

        CSL_PLLC_getPllStatusReg (hPllc, &goStatus);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllStatusReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pGoStatus
)
{
    *pGoStatus   =   CSL_FEXT (hPllc->PLLSTAT, PLLC_PLLSTAT_GOSTAT);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllAlignCtrlReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Clock Align control register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            alnCtlVal       Value to write to register.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_ALNCTL_ALN1_16
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setPllAlignCtrlReg (hPllc, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllAlignCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint32              alnCtlVal
)
{
    CSL_FINS (hPllc->ALNCTL, PLLC_ALNCTL_ALN1_16, alnCtlVal);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllAlignCtrlReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Clock Align control register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_ALNCTL_ALN1_16
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint32          alnCtlVal;
        ...

        alnCtlVal   =   CSL_PLLC_getPllAlignCtrlReg (hPllc);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_PLLC_getPllAlignCtrlReg 
(
    CSL_PllcHandle      hPllc
)
{
    return CSL_FEXT (hPllc->ALNCTL, PLLC_ALNCTL_ALN1_16);
}

/** ============================================================================
 *   @n@b CSL_PLLC_setPllDChangeReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Divider Ratio Change Status register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            dChangeVal      Value to write to register.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_DCHANGE_SYS1_16
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setPllDChangeReg (hPllc, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllDChangeReg 
(
    CSL_PllcHandle      hPllc,
    Uint32              dChangeVal
)
{
    CSL_FINS (hPllc->DCHANGE, PLLC_DCHANGE_SYS1_16, dChangeVal);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllDChangeReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Divider Ratio Change Status register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_DCHANGE_SYS1_16
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint32          dChangeVal;
        ...

        dChangeVal   =   CSL_PLLC_getPllDChangeReg (hPllc);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_PLLC_getPllDChangeReg 
(
    CSL_PllcHandle      hPllc
)
{
    return CSL_FEXT (hPllc->DCHANGE, PLLC_DCHANGE_SYS1_16);
}


/** ============================================================================
 *   @n@b CSL_PLLC_setPllClkEnableCtrlReg
 *
 *   @b Description
 *   @n Sets up the contents of PLL Clock Enable Control register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            auxClkEnable    Enable/disable PLLC output clock, AUXCLK
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n PLLC_CKEN_AUXEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;

        ...

        CSL_PLLC_setPllClkEnableCtrlReg (hPllc, 0);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_setPllClkEnableCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint32              auxClkEnable
)
{
    CSL_FINS (hPllc->CKEN, PLLC_CKEN_AUXEN, auxClkEnable);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllClkEnableCtrlReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Clock Enable Control register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pAuxClkEnable   Indicates if AUXCLK is disabled/enabled
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_CKEN_AUXEN
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           auxClkEnable;
        ...

        CSL_PLLC_getPllClkEnableCtrlReg (hPllc, &auxClkEnable);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllClkEnableCtrlReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pAuxClkEnable
)
{
    *pAuxClkEnable  =   CSL_FEXT (hPllc->CKEN, PLLC_CKEN_AUXEN);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllClkStatusReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL Clock Status register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
            pAuxOn          Indicates if AUXCLK is ON
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  void
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_CKSTAT_AUXON
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint8           auxOn;
        ...

        CSL_PLLC_getPllClkStatusReg (hPllc, &auxOn);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE void CSL_PLLC_getPllClkStatusReg 
(
    CSL_PllcHandle      hPllc,
    Uint8*              pAuxOn
)
{
    *pAuxOn =   CSL_FEXT (hPllc->CKSTAT, PLLC_CKSTAT_AUXON);

    return;
}

/** ============================================================================
 *   @n@b CSL_PLLC_getPllSysClkStatusReg
 *
 *   @b Description
 *   @n Retrieves the contents of PLL SYSCLK Status register.
 *
 *   @b Arguments
 *   @verbatim
            hPllc           Handle to the PLLC instance
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  Must call @a CSL_PLLC_open () before calling any PLLC CSL API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n PLLC_SYSTAT_SYS1_16ON
 *
 *   @b Example
 *   @verbatim
        CSL_PllcHandle  hPllc;
        Uint32          sysClkStatus;
        ...

        sysClkStatus    =   CSL_PLLC_getPllSysClkStatusReg (hPllc);
     @endverbatim
 *  ============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_PLLC_getPllSysClkStatusReg 
(
    CSL_PllcHandle      hPllc
)
{
    return CSL_FEXT (hPllc->SYSTAT, PLLC_SYSTAT_SYS1_16ON);
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* CSL_PLLCAUX_H_ */

