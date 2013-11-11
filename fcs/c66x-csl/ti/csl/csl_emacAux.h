/*
 * Copyright (C) 2002-2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *   @file  csl_emacAux.h
 *
 *   @path  $(CSLPATH)\inc
 *
 *   @desc  Auxiliary API header file for EMAC CSL
 *
 */

/* =============================================================================
 *  Revision History
 *  ===============
 *  29-May-2006  PSK File Created.
 *  16-July-2008 Update for Doxygen
 * =============================================================================
 */

#ifndef _CSL_EMACAUX_H_
#define _CSL_EMACAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <csl_emac.h>

/** @addtogroup CSL_EMAC_FUNCTION
 @{ */
/** ============================================================================
 *   @n@b EMAC_txChannelTeardown
 *
 *   @b Description
 *      Tear down selective transmit channel/channels
 *
 *   @b Arguments
     @verbatim
        hEMAC      handle to the opened EMAC device
            val        mask of selective tx channels to be torn down.
     @endverbatim
 *
 *   <b> Return Value </b>  EMAC_ERROR_INVALID - Invalid instance number
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  Tear down specific tx channels
 *
 *   @b Modifies
 *   @n EMAC registers
 *
 *   @b Example
 *   @verbatim
        Uint32             val = EMAC_TEARDOWN_CHANNEL(0) | EMAC_TEARDOWN_CHANNEL(1);
        EMAC_Device        EMACObj;
        EMAC_Config        ecfg;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj);

        ...


        EMAC_txChannelTeardown (&EMACObj, val);
     @endverbatim
 * =============================================================================
 */
static inline int EMAC_txChannelTeardown (
    Handle hEMAC,
    Uint32 val
)
{
    EMAC_Device     *pd = (EMAC_Device *)hEMAC;
    CSL_EmacRegs    *emacRegs;
    int temp;

    /* Validate our handle */
    if (!pd || pd->DevMagic != EMAC_DEVMAGIC)
        return (EMAC_ERROR_INVALID);

    emacRegs = pd->emacRegs;

    for (temp = 0; val != 0; val >>= 1, temp += 1) {
        if (val & 0x1) {
            emacRegs->TXTEARDOWN = temp;
        }
    }
    return 0;    
}


/** ============================================================================
 *   @n@b EMAC_rxChannelTeardown
 *
 *   @b Description
 *      Tear down selective receive channel/channels
 *
 *   @b Arguments
     @verbatim
            val        mask of selective rx channels to be torn down.
        hEMAC      handle to the opened EMAC device
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  Tear down specific rx channels
 *
 *   @b Modifies
 *   @n EMAC registers
 *
 *   @b Example
 *   @verbatim
        Uint32             val = EMAC_TEARDOWN_CHANNEL(0) | EMAC_TEARDOWN_CHANNEL(1);
        EMAC_Device        EMACObj;
        EMAC_Config        ecfg;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj);

        ...


        EMAC_rxChannelTeardown (&EMACObj, val);
     @endverbatim
 * =============================================================================
 */
static inline int EMAC_rxChannelTeardown (
    Handle hEMAC,
    Uint32 val
)
{
    EMAC_Device     *pd = (EMAC_Device *)hEMAC;
    CSL_EmacRegs    *emacRegs;
    int temp;

    /* Validate our handle */
    if (!pd || pd->DevMagic != EMAC_DEVMAGIC)
        return (EMAC_ERROR_INVALID);

    emacRegs = pd->emacRegs;

    for (temp = 0; val != 0; val >>= 1, temp += 1) {
        if (val & 0x1) {
            emacRegs->RXTEARDOWN = temp;
        }
    }
    return 0;    
}
/* @} */


#ifdef __cplusplus
}
#endif

#endif /*_CSL_EMACAUX_H_ */

