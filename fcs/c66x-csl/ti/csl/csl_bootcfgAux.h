/**
 *   @file  csl_bootcfgAux.h
 *
 *   @brief   
 *      This is the Boot Configuration Auxilary Header File which exposes 
 *      the various CSL Functional Layer API's to configure the BOOTCFG Module.
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

#ifndef _CSL_BOOTCFG_AUX_H_
#define _CSL_BOOTCFG_AUX_H_

#include <ti/csl/csl_bootcfg.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup CSL_BOOTCFG_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_BootCfgGetRevisionInfo
 *
 *   @b Description
 *   @n The function is used to get the revision information for the boot 
 *      configuration module.
 *
 *   @b Arguments
     @verbatim
        info    Revision information populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_REVISION_REG
 *
 *   @b Example
 *   @verbatim
        Uint32  info;

        // Get the boot configuration revision information.
        CSL_BootCfgGetRevisionInfo (&info);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetRevisionInfo (Uint32* info)
{
    *info = hBootCfg->REVISION_REG;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetDeviceId
 *
 *   @b Description
 *   @n The function is used to get the device identifier.
 *
 *   @b Arguments
     @verbatim
        id    Device Identifier populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_DEVICE_ID_REG0
 *
 *   @b Example
 *   @verbatim
        Uint32  id;

        // Get the device identifier.
        CSL_BootCfgGetDeviceId (&id);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetDeviceId (Uint32* id)
{
    *id = hBootCfg->DEVICE_ID_REG0;
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsLittleEndian
 *
 *   @b Description
 *   @n The function checks if the device is running in little endian mode or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Device is in Little Endian Mode
 *   @n FALSE - Device is in Big Endian Mode
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOT_REG0_BIG_ENDIAN
 *
 *   @b Example
 *   @verbatim

        // Check if device is in little endian mode or not?
        if (CSL_BootCfgIsLittleEndian () == TRUE)
        {
            // Little Endian
        }
        else
        {
            // Big Endian
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsLittleEndian (void)
{
    if (CSL_FEXT(hBootCfg->BOOT_REG0, BOOTCFG_BOOT_REG0_BIG_ENDIAN) == 1)
        return TRUE;
    return FALSE;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetBootMode
 *
 *   @b Description
 *   @n The function returns the boot mode which is latched at POR or RESETFULL.
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Boot mode 
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOT_REG0_BOOT_MODE
 *
 *   @b Example
 *   @verbatim

        Uint16 bootMode;

        // Get the boot mode.
        bootMode = CSL_BootCfgGetBootMode();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint16 CSL_BootCfgGetBootMode (void)
{
    return CSL_FEXT(hBootCfg->BOOT_REG0, BOOTCFG_BOOT_REG0_BOOT_MODE);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetBootMode
 *
 *   @b Description
 *   @n The function sets the boot mode 
 *
 *   @b Arguments
     @verbatim
        bootMode    - Boot Mode to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None 
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_BOOT_REG0_BOOT_MODE
 *
 *   @b Example
 *   @verbatim

        // Set the boot mode.
        CSL_BootCfgSetBootMode(0x10);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetBootMode (Uint16 bootMode)
{
    CSL_FINS(hBootCfg->BOOT_REG0, BOOTCFG_BOOT_REG0_BOOT_MODE, bootMode);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetPCIEMode
 *
 *   @b Description
 *   @n The function returns the PCIE mode which is latched at POR or RESETFULL.
 *      The PCIE can operate in three modes, End-point mode, Legacy End-point mode 
 *      or Root complex mode.
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n 0x0 - PCIE in End-point mode
 *   @n 0x1 - PCIE in Legacy End-point mode
 *   @n 0x2 - PCIE in Root complex mode
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOT_REG0_PCIESS_MODE
 *
 *   @b Example
 *   @verbatim

        Uint8 pcieMode;

        // Get the PCIE-SS mode.
        pcieMode = CSL_BootCfgGetPCIEMode();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_BootCfgGetPCIEMode (void)
{
    return CSL_FEXT(hBootCfg->BOOT_REG0, BOOTCFG_BOOT_REG0_PCIESS_MODE);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetPCIEMode
 *
 *   @b Description
 *   @n The function sets the PCIESS Mode.
 *
 *   @b Arguments
     @verbatim
        pcieMode    - PCIE Mode to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_BOOT_REG0_PCIESS_MODE
 *
 *   @b Example
 *   @verbatim

        // Set the PCIE-SS mode.
        CSL_BootCfgSetPCIEMode(1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetPCIEMode (Uint8 pcieMode)
{
    CSL_FINS(hBootCfg->BOOT_REG0, BOOTCFG_BOOT_REG0_PCIESS_MODE, pcieMode);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetPCIEPLLLock
 *
 *   @b Description
 *   @n The function is to used get the PCIE PLL Lock status
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n lock - Lock status
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_STS_PCIE_STS_LOCK
 *
 *   @b Example
 *   @verbatim

        Uint16 lock;

        // Get the PCIE PLL lock status
        CSL_BootCfgGetPCIEPLLLock(&lock);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetPCIEPLLLock (Uint16* lock)
{
    *lock = CSL_FEXT(hBootCfg->STS_PCIE, BOOTCFG_STS_PCIE_STS_LOCK);
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsPCIEEnabled
 *
 *   @b Description
 *   @n The function checks if the PCIE mode is enabled or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - PCIE is enabled
 *   @n FALSE - PCIE is disabled
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOT_REG0_PCIESSEN
 *
 *   @b Example
 *   @verbatim

        // Is the PCIE-SS Enabled or not?
        if (CSL_BootCfgIsPCIEEnabled() == TRUE)
        {
            // PCIESS is enabled
        }
        else
        {
            // PCIESS is disabled
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsPCIEEnabled (void)
{
    return (Bool)CSL_FEXT(hBootCfg->BOOT_REG0, BOOTCFG_BOOT_REG0_PCIESSEN);
}

/** ============================================================================
 *   @n@b CSL_BootCfgUnlockKicker
 *
 *   @b Description
 *   @n The function is used to unlock the kicker mechanism which will allow the 
 *      Boot configuration MMR registers to be come writeable. 
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_KICK_REG0=0x83e70b13, BOOTCFG_KICK_REG1=0x95a4f1e0 
 *
 *   @b Example
 *   @verbatim

        // Unlock the kicker to ensure Boot configuration MMR is writeable
        CSL_BootCfgUnlockKicker();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgUnlockKicker (void)
{
    hBootCfg->KICK_REG0 = 0x83e70b13;
    hBootCfg->KICK_REG1 = 0x95a4f1e0;

    return;
}

/** ============================================================================
 *   @n@b CSL_BootCfgLockKicker
 *
 *   @b Description
 *   @n The function is used to lock the kicker mechanism which will ensure
 *      that all subequent writes to the Boot configuration MMR are now blocked
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_KICK_REG0=0x1, BOOTCFG_KICK_REG1=0x1 
 *
 *   @b Example
 *   @verbatim

        // Unlock the kicker to ensure Boot configuration MMR is writeable
        CSL_BootCfgUnlockKicker();
        ...
        // Lock the kicker mechanism
        CSL_BootCfgLockKicker();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgLockKicker (void)
{
    hBootCfg->KICK_REG0 = 0x1;
    hBootCfg->KICK_REG1 = 0x1;

    return;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetDSPBootAddress
 *
 *   @b Description
 *   @n The function is used to get the initial fetch address of each GEM core
 *
 *   @b Arguments
     @verbatim
        coreIdx -   Index of the core for which the fecth address is required
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOTADDR_GEM0_REG_BOOTADDR_GEM0
 *
 *   @b Example
 *   @verbatim
        Uint32  bootAddress;

        // Get the Boot configuration DSP Boot Address for Core 1
        bootAddress = CSL_BootCfgGetDSPBootAddress(1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BootCfgGetDSPBootAddress (Uint8 coreIdx)
{
    return CSL_FEXT(hBootCfg->BOOTADDR_GEM_REG[coreIdx], BOOTCFG_BOOTADDR_GEM0_REG_BOOTADDR_GEM0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetDSPBootAddress
 *
 *   @b Description
 *   @n The function is used to set the initial fetch address of each GEM core
 *      This can be done only for non-secure devices.
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Index of the core for which the fetch address is configured
        bootAddress - Boot Address to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_BOOTADDR_GEM0_REG_BOOTADDR_GEM0
 *
 *   @b Example
 *   @verbatim
        Uint32  bootAddress;

        // Set the Boot configuration DSP Boot Address for Core 1
        CSL_BootCfgSetDSPBootAddress(1, 0x20B00000);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetDSPBootAddress (Uint8 coreIdx, Uint32 bootAddress)
{
    CSL_FINS(hBootCfg->BOOTADDR_GEM_REG[coreIdx], BOOTCFG_BOOTADDR_GEM0_REG_BOOTADDR_GEM0, bootAddress);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetMacIdentifier
 *
 *   @b Description
 *   @n The function is used to get the MAC address for this device derived from
 *      the EFUSE.
 *
 *   @b Arguments
     @verbatim
        macId1  -   Contents of the MAC ID1 register
        macId2  -   Contents of the MAC ID2 register
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_EMAC_ID1,BOOTCFG_EMAC_ID2
 *
 *   @b Example
 *   @verbatim
        Uint32  macId1;
        Uint32  macId2;

        // Get the MAC Identifier
        CSL_BootCfgGetMacIdentifier(&macId1, &macId2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetMacIdentifier (Uint32* macId1, Uint32* macId2)
{
    *macId1 = hBootCfg->EMAC_ID1;
    *macId2 = hBootCfg->EMAC_ID2;
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsBootComplete
 *
 *   @b Description
 *   @n The function is used to check if the booting is complete for a specific
 *      core or not?
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core index which is to be checked
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOT_COMPLETE_GEM0_COMPLETE;BOOTCFG_BOOT_COMPLETE_GEM1_COMPLETE;
 *
 *   @b Example
 *   @verbatim

        // Check if the booting is complete or not for core 1?
        if (CSL_BootCfgIsBootComplete(1) == TRUE)
        {
            // Booting is done
        }
        else
        {
            // Booting is not done.        
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsBootComplete (Uint8 coreIdx)
{
    if (coreIdx == 0)
        return (Bool)CSL_FEXT (hBootCfg->BOOT_COMPLETE,BOOTCFG_BOOT_COMPLETE_GEM0_COMPLETE);
    else
        return (Bool)CSL_FEXT (hBootCfg->BOOT_COMPLETE,BOOTCFG_BOOT_COMPLETE_GEM1_COMPLETE);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetBootComplete
 *
 *   @b Description
 *   @n The function is used to set the boot complete. This can be done only once
 *      by the software.
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core index which for which boot complete is to be set
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_BOOT_COMPLETE_GEM0_COMPLETE;BOOTCFG_BOOT_COMPLETE_GEM1_COMPLETE;
 *
 *   @b Example
 *   @verbatim

        // Booting is done for core 0
        CSL_BootCfgSetBootComplete (0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetBootComplete (Uint8 coreIdx)
{
    if (coreIdx == 0)
        CSL_FINS (hBootCfg->BOOT_COMPLETE,BOOTCFG_BOOT_COMPLETE_GEM0_COMPLETE, 1);
    if (coreIdx == 1)
        CSL_FINS (hBootCfg->BOOT_COMPLETE,BOOTCFG_BOOT_COMPLETE_GEM1_COMPLETE, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetBootProgress
 *
 *   @b Description
 *   @n The function is used to get the boot progress. 
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Boot progress contents.
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_BOOT_PROGRESS
 *
 *   @b Example
 *   @verbatim
        Uint32  bootProgress;

        // Get the boot progress
        bootProgress = CSL_BootCfgGetBootProgress ();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_BootCfgGetBootProgress (void)
{
    return hBootCfg->BOOT_PROGRESS;
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetBootProgress
 *
 *   @b Description
 *   @n The function is used to set the boot progress. 
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Boot progress contents.
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_BOOT_PROGRESS
 *
 *   @b Example
 *   @verbatim

        // Set the boot progress
        CSL_BootCfgSetBootProgress (0xDEAD);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetBootProgress (Uint32 bootProgress)
{
    hBootCfg->BOOT_PROGRESS = bootProgress;
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsCoreLocalReset
 *
 *   @b Description
 *   @n The function is used to check if the core is locally reset or not?
 *
 *   @b Arguments
     @verbatim
        coreIdx - Core Index for which the reset status is needed
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Core is locally reset.
 *   @n FALSE - Core is not locally reset.
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_RESET_STAT_LRST_STAT0;BOOTCFG_RESET_STAT_LRST_STAT1;
 *
 *   @b Example
 *   @verbatim

        // Is Core 0 locally reset?
        if (CSL_BootCfgIsCoreLocalReset (0) == TRUE)
        {
            // Core 0 is locally reset.
        }
        else
        {
            // Core 0 is not locally reset.
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsCoreLocalReset (Uint8 coreIdx)
{
    if (coreIdx == 0)
        return (Bool)CSL_FEXT (hBootCfg->RESET_STAT,BOOTCFG_BOOT_COMPLETE_GEM0_COMPLETE);
    else
        return (Bool)CSL_FEXT (hBootCfg->RESET_STAT,BOOTCFG_BOOT_COMPLETE_GEM1_COMPLETE);
}

/** ============================================================================
 *   @n@b CSL_BootCfgClearLocalReset
 *
 *   @b Description
 *   @n The function is used to clear the local reset state of the specified core
 *
 *   @b Arguments
     @verbatim
        coreIdx - Core Index for which the local reset state is to be cleared
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_RESET_STAT_CLR_LRST_STAT_0_CLR;BOOTCFG_RESET_STAT_CLR_LRST_STAT_1_CLR;
 *
 *   @b  Affects
 *   @n  BOOTCFG_RESET_STAT_LRST_STAT0=0;BOOTCFG_RESET_STAT_LRST_STAT1=0;
 *
 *   @b Example
 *   @verbatim

        // Is Core 0 locally reset?
        if (CSL_BootCfgIsCoreLocalReset (0) == TRUE)
        {
            // Core 0 is locally reset.
            ...
            // Clear Core 0 local reset.
            CSL_BootCfgClearLocalReset (0);
        }
        else
        {
            // Core 0 is not locally reset.
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgClearLocalReset (Uint8 coreIdx)
{
    if (coreIdx == 0)
        CSL_FINS (hBootCfg->RESET_STAT_CLR,BOOTCFG_RESET_STAT_CLR_LRST_STAT_0_CLR, 1);
    if (coreIdx == 1)
        CSL_FINS (hBootCfg->RESET_STAT_CLR,BOOTCFG_RESET_STAT_CLR_LRST_STAT_1_CLR, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsGlobalReset
 *
 *   @b Description
 *   @n The function is used to check if the device is globally reset or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Device is global reset.
 *   @n FALSE - Device is not globally reset.
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_RESET_STAT_GRST_STAT
 *
 *   @b Example
 *   @verbatim

        // Is device globally reset?
        if (CSL_BootCfgIsGlobalReset () == TRUE)
        {
            // Device is global reset
        }
        else
        {
            // Device is NOT global reset
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsGlobalReset (void)
{
    return (Bool)CSL_FEXT (hBootCfg->RESET_STAT,BOOTCFG_RESET_STAT_GRST_STAT);
}

/** ============================================================================
 *   @n@b CSL_BootCfgClearGlobalReset
 *
 *   @b Description
 *   @n The function is used to clear the global reset state of the specified core
 *
 *   @b Arguments
     @verbatim
        coreIdx - Core Index for which the global reset state is to be cleared
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_RESET_STAT_GRST_STAT
 *
 *   @b Example
 *   @verbatim

        // Is device globally reset?
        if (CSL_BootCfgIsGlobalReset () == TRUE)
        {
            // Device is global reset
            ...
            // Clear the global reset flag
            CSL_BootCfgClearGlobalReset();
        }
        else
        {
            // Device is NOT global reset
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgClearGlobalReset (void)
{
    CSL_FINS (hBootCfg->RESET_STAT,BOOTCFG_RESET_STAT_GRST_STAT, (Uint32) 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetNMILocalResetStatus
 *
 *   @b Description
 *   @n The function is used to get the NMI & Local reset status for a specific
 *      core.
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index for which the status is required
        nmiStatus   - NMI Status populated by this API
        localStatus - Local Reset Status populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Reads
 *   @n  BOOTCFG_LRSTNMISTAT_NMI_STAT0,BOOTCFG_LRSTNMISTAT_LRESET_STAT0;
 *   @n  BOOTCFG_LRSTNMISTAT_NMI_STAT1,BOOTCFG_LRSTNMISTAT_LRESET_STAT1;
 *
 *   @b Example
 *   @verbatim

        Uint8   nmiStatus;
        Uint8   localStatus;

        // Get the NMI and Local Reset Status for core 1
        CSL_BootCfgGetNMILocalResetStatus (1, &nmiStatus, &localStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetNMILocalResetStatus (Uint8 coreIdx, Uint8* nmiStatus, Uint8* localStatus)
{
    Uint32 value = hBootCfg->LRSTNMISTAT;

    if (coreIdx == 0)
    {
        *nmiStatus   = CSL_FEXT (value, BOOTCFG_LRSTNMISTAT_NMI_STAT0);
        *localStatus = CSL_FEXT (value, BOOTCFG_LRSTNMISTAT_LRESET_STAT0);
    }
    if (coreIdx == 1)
    {
        *nmiStatus   = CSL_FEXT (value, BOOTCFG_LRSTNMISTAT_NMI_STAT1);
        *localStatus = CSL_FEXT (value, BOOTCFG_LRSTNMISTAT_LRESET_STAT1);
    } 
}

/** ============================================================================
 *   @n@b CSL_BootCfgClearNMILocalResetStatus
 *
 *   @b Description
 *   @n The function is used to clear the NMI & Local reset status for a specific
 *      core.
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index for which the status is required
        nmiStatus   - NMI Status to be cleared
        localStatus - Local Reset Status to be cleared
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b  Writes
 *   @n  BOOTCFG_LRSTNMISTAT_CLR_NMI_STAT_0_CLR,BOOTCFG_LRSTNMISTAT_CLR_LRESET_STAT_0_CLR;
 *   @n  BOOTCFG_LRSTNMISTAT_CLR_NMI_STAT_1_CLR,BOOTCFG_LRSTNMISTAT_CLR_LRESET_STAT_1_CLR;
 *
 *   @b  Affects
 *   @n  BOOTCFG_LRSTNMISTAT_NMI_STAT0=0,BOOTCFG_LRSTNMISTAT_LRESET_STAT0=0;
 *   @n  BOOTCFG_LRSTNMISTAT_NMI_STAT1=0,BOOTCFG_LRSTNMISTAT_LRESET_STAT1=0;
 *
 *   @b Example
 *   @verbatim

        Uint8   nmiStatus;
        Uint8   localStatus;

        // Get the NMI and Local Reset Status for core 1
        CSL_BootCfgGetNMILocalResetStatus (1, &nmiStatus, &localStatus);
        ...
        // Clear the status
        CSL_BootCfgClearNMILocalResetStatus (1, nmiStatus, localStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgClearNMILocalResetStatus (Uint8 coreIdx, Uint8 nmiStatus, Uint8 localStatus)
{
    Uint32 value = 0;

    if (coreIdx == 0)
    {
        CSL_FINS (value, BOOTCFG_LRSTNMISTAT_CLR_NMI_STAT_0_CLR,    nmiStatus);
        CSL_FINS (value, BOOTCFG_LRSTNMISTAT_CLR_LRESET_STAT_0_CLR, localStatus);
    }
    if (coreIdx == 1)
    {
        CSL_FINS (value, BOOTCFG_LRSTNMISTAT_CLR_NMI_STAT_1_CLR,    nmiStatus);
        CSL_FINS (value, BOOTCFG_LRSTNMISTAT_CLR_LRESET_STAT_1_CLR, localStatus);
    } 
    /* Write to the register to clear the NMI Status. */
    hBootCfg->LRSTNMISTAT_CLR = value;
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsSysClockEnabled
 *
 *   @b Description
 *   @n The function checks if the system clock is enabled or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Clock output enabled
 *   @n FALSE - No Clock Output
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_DEVCFG_SYSCLKOUTEN
 *
 *   @b Example
 *   @verbatim

        // Is System clock is enabled or not?
        if (CSL_BootCfgIsSysClockEnabled () == TRUE)
        {
            // Clock output is Enabled
        }
        else
        {
            // No Clock Output
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsSysClockEnabled (void)
{
    return (Bool)CSL_FEXT(hBootCfg->DEVCFG, BOOTCFG_DEVCFG_SYSCLKOUTEN);
}

/** ============================================================================
 *   @n@b CSL_BootCfgEnableSysClock
 *
 *   @b Description
 *   @n The function is used to enable the sys clock 
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_DEVCFG_SYSCLKOUTEN=1
 *
 *   @b Example
 *   @verbatim

        CSL_BootCfgEnableSysClock();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgEnableSysClock (void)
{
    CSL_FINS(hBootCfg->DEVCFG, BOOTCFG_DEVCFG_SYSCLKOUTEN, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgDisableSysClock
 *
 *   @b Description
 *   @n The function is used to disable the sys clock 
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_DEVCFG_SYSCLKOUTEN=0
 *
 *   @b Example
 *   @verbatim

        CSL_BootCfgDisableSysClock();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgDisableSysClock (void)
{
    CSL_FINS(hBootCfg->DEVCFG, BOOTCFG_DEVCFG_SYSCLKOUTEN, 0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetPowerStatus
 *
 *   @b Description
 *   @n The function gets the Power status
 *
 *   @b Arguments
     @verbatim
        standbyStatus       - Indicates whether in Standby mode or not
        hibernationStatus   - Indicates whether in Hibernation mode or not
        hibernationMode     - Indicates whether in Hibernation mode1 or mode2
        generalPurpose      - General Purpose value in the register
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_PWR_STAT_STANDBY,BOOTCFG_PWR_STAT_HIBERNATION,
 *   @n BOOTCFG_PWR_STAT_HIBERNATION_MODE,BOOTCFG_PWR_STAT_PWR_STAT_GENERAL
 *
 *   @b Example
 *   @verbatim

        Uint8  standby;
        Uint8  hibernation;
        Uint8  hibernationMode;
        Uint32 generalPurpose;

        // Get the power status.
        CSL_BootCfgGetPowerStatus(&standby, &hibernation, &hibernationMode, &generalPurpose);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetPowerStatus 
(
    Uint8*  standby, 
    Uint8*  hibernation, 
    Uint8*  hibernationMode,
    Uint32* generalPurpose
)
{
    Uint32 value = hBootCfg->PWR_STAT;

    *standby        = CSL_FEXT (value, BOOTCFG_PWR_STAT_STANDBY);
    *hibernation    = CSL_FEXT (value, BOOTCFG_PWR_STAT_HIBERNATION);
    *hibernationMode= CSL_FEXT (value, BOOTCFG_PWR_STAT_HIBERNATION_MODE);
    *generalPurpose = CSL_FEXT (value, BOOTCFG_PWR_STAT_PWR_STAT_GENERAL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetPowerStatus
 *
 *   @b Description
 *   @n The function sets the Power status
 *
 *   @b Arguments
     @verbatim
        standbyStatus       - Status of the standby to be configured
        hibernationStatus   - Hibernation Status to be configured
        hibernationMode     - Hibernation mode to be configured
        generalPurpose      - General Purpose value to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_PWR_STAT_STANDBY,BOOTCFG_PWR_STAT_HIBERNATION,
 *   @n BOOTCFG_PWR_STAT_HIBERNATION_MODE,BOOTCFG_PWR_STAT_PWR_STAT_GENERAL
 *
 *   @b Example
 *   @verbatim

        Uint8  standby;
        Uint8  hibernation;
        Uint8  hibernationMode;
        Uint32 generalPurpose;

        // Get the power status.
        CSL_BootCfgGetPowerStatus(&standby, &hibernation, &hibernationMode, &generalPurpose);
        ...
        // Set the power status to be in standby mode
        CSL_BootCfgSetPowerStatus (1, hibernation, hibernationMode, generalPurpose);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetPowerStatus 
(
    Uint8  standby, 
    Uint8  hibernation, 
    Uint8  hibernationMode,
    Uint32 generalPurpose
)
{
    hBootCfg->PWR_STAT = CSL_FMK (BOOTCFG_PWR_STAT_STANDBY,         standby)            |
                         CSL_FMK (BOOTCFG_PWR_STAT_HIBERNATION,     hibernation)        |
                         CSL_FMK (BOOTCFG_PWR_STAT_HIBERNATION_MODE,hibernationMode)    |
                         CSL_FMK (BOOTCFG_PWR_STAT_PWR_STAT_GENERAL,generalPurpose); 
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSRIOSERDESStatus
 *
 *   @b Description
 *   @n The function gets the SRIO SERDES Status.
 *
 *   @b Arguments
     @verbatim
        srioSERDESStatus    - Status populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_STS_SRIO_STS_SRIO
 *
 *   @b Example
 *   @verbatim

        Uint32 srioSERDESStatus;

        // Get the SRIO SERDES status.
        CSL_BootCfgGetSRIOSERDESStatus(&srioSERDESStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSRIOSERDESStatus 
(
    Uint32*  srioSERDESStatus 
)
{
    *srioSERDESStatus = CSL_FEXT (hBootCfg->STS_SRIO, BOOTCFG_STS_SRIO_STS_SRIO);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSGMIISERDESStatus
 *
 *   @b Description
 *   @n The function gets the SGMII SERDES Status.
 *
 *   @b Arguments
     @verbatim
        sgmiiSERDESStatus    - Status populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_STS_SGMII_STS_SGMII
 *
 *   @b Example
 *   @verbatim

        Uint32 sgmiiSERDESStatus;

        // Get the SGMII SERDES status.
        CSL_BootCfgGetSGMIISERDESStatus(&sgmiiSERDESStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSGMIISERDESStatus 
(
    Uint32*  sgmiiSERDESStatus
)
{
    *sgmiiSERDESStatus = CSL_FEXT (hBootCfg->STS_SGMII, BOOTCFG_STS_SGMII_STS_SGMII);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetPCIESERDESStatus
 *
 *   @b Description
 *   @n The function gets the PCIE SERDES Status.
 *
 *   @b Arguments
     @verbatim
        pcieSERDESStatus    - Status populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_STS_PCIE_STS_PCIE
 *
 *   @b Example
 *   @verbatim

        Uint32 pcieSERDESStatus;

        // Get the PCIE SERDES status.
        CSL_BootCfgGetPCIESERDESStatus(&pcieSERDESStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetPCIESERDESStatus 
(
    Uint32*  pcieSERDESStatus 
)
{
    *pcieSERDESStatus = CSL_FEXT (hBootCfg->STS_PCIE, BOOTCFG_STS_PCIE_STS_PCIE);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetVUSRSERDESStatus
 *
 *   @b Description
 *   @n The function gets the VUSR SERDES Status.
 *
 *   @b Arguments
     @verbatim
        vusrSERDESStatus    - Status populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_STS_VUSR_STS_VUSR
 *
 *   @b Example
 *   @verbatim

        Uint32 vusrSERDESStatus;

        // Get the VUSR SERDES status.
        CSL_BootCfgGetVUSRSERDESStatus(&vusrSERDESStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetVUSRSERDESStatus 
(
    Uint32*  vusrSERDESStatus 
)
{
    *vusrSERDESStatus = CSL_FEXT (hBootCfg->STS_VUSR, BOOTCFG_STS_VUSR_STS_VUSR);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGenerateNMIEvent
 *
 *   @b Description
 *   @n The function is used to generate a NMI event to a specific core.
 *
 *   @b Arguments
     @verbatim
        coreNum    - Core Number on which the NMI event is to be generated.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_NMIGR_0_NMIGR_0_REG;BOOTCFG_NMIGR_1_NMIGR_1_REG;
 *
 *   @b Example
 *   @verbatim

        // Generate NMI Event on Core 0
        CSL_BootCfgGenerateNMIEvent(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGenerateNMIEvent 
(
    Uint8  coreNum
)
{
    CSL_FINS (hBootCfg->NMIGR[coreNum], BOOTCFG_NMIGR_0_NMIGR_0_REG, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGenerateInterDSPInterrupt
 *
 *   @b Description
 *   @n The function is used to generate an inter-DSP interrupt.
 *
 *   @b Arguments
     @verbatim
        coreNum  - Core Number to which the interrupt is directed.
        sourceId - Source Id which identifies the entity generating the interrupt
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_IPCGR0_IPCGR0_SRC, BOOTCFG_IPCGR0_IPCGR0_REG=1;
 *   @n BOOTCFG_IPCGR1_IPCGR1_SRC, BOOTCFG_IPCGR1_IPCGR1_REG=1;
 *
 *   @b Example
 *   @verbatim

        // Generate Inter-DSP Interrupt to Core 1 with the Source Id as 4
        CSL_BootCfgGenerateInterDSPInterrupt(1, 4);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGenerateInterDSPInterrupt 
(
    Uint8   coreNum,
    Uint32  sourceId 
)
{
    hBootCfg->IPCGR[coreNum] = CSL_FMK(BOOTCFG_IPCGR0_IPCGR0_SRC, sourceId) | 
                               CSL_FMK(BOOTCFG_IPCGR0_IPCGR0_REG, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGenerateExternalHostInterrupt
 *
 *   @b Description
 *   @n The function is used to generate an external host interrupt.
 *
 *   @b Arguments
     @verbatim
        sourceId - Source Id which identifies the entity generating the interrupt
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_IPCGRH_IPCGRH_SRC, BOOTCFG_IPCGRH_IPCGRH_REG=1;
 *
 *   @b Example
 *   @verbatim

        // Generate External Host Interrupt with the Source Id as 2
        CSL_BootCfgGenerateExternalHostInterrupt(2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGenerateExternalHostInterrupt 
(
    Uint32  sourceId 
)
{
    hBootCfg->IPCGRH = CSL_FMK(BOOTCFG_IPCGRH_IPCGRH_SRC, sourceId) |
                       CSL_FMK(BOOTCFG_IPCGRH_IPCGRH_REG, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetInterDSPInterruptStatus
 *
 *   @b Description
 *   @n The function is used to get the status of the Inter-DSP Interrupt 
 *      register. 
 *
 *   @b Arguments
     @verbatim
        coreNum   - Core Number for which the interrupt status is read.
        intStatus - Interrupt Status(Source ID) populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_IPCAR0_IPCGR0_SRC_CLR; BOOTCFG_IPCAR1_IPCGR1_SRC_CLR;
 *
 *   @b Example
 *   @verbatim

        Uint32  srcId;

        // Get the Inter-DSP Interrupt Status for Core 2
        CSL_BootCfgGetInterDSPInterruptStatus(2, &srcId);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetInterDSPInterruptStatus 
(
    Uint8   coreNum,
    Uint32* intStatus 
)
{
    *intStatus = CSL_FEXT(hBootCfg->IPCAR[coreNum], BOOTCFG_IPCAR0_IPCGR0_SRC_CLR);
}

/** ============================================================================
 *   @n@b CSL_BootCfgClearInterDSPInterruptStatus
 *
 *   @b Description
 *   @n The function is used to clear the status of the Inter-DSP Interrupt 
 *      register. 
 *
 *   @b Arguments
     @verbatim
        coreNum   - Core Number for which the interrupt status is read.
        intStatus - Interrupt Status(Source ID) to be cleared.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_IPCAR0_IPCGR0_SRC_CLR; BOOTCFG_IPCAR1_IPCGR1_SRC_CLR;
 *
 *   @b Example
 *   @verbatim

        Uint32  srcId;

        // Get the Inter-DSP Interrupt Status for Core 2
        CSL_BootCfgGetInterDSPInterruptStatus(2, &srcId);
        ...
        // Clear the interrupts.
        CSL_BootCfgClearInterDSPInterruptStatus(2, srcId);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgClearInterDSPInterruptStatus 
(
    Uint8   coreNum,
    Uint32  intStatus 
)
{
    CSL_FINS(hBootCfg->IPCAR[coreNum], BOOTCFG_IPCAR0_IPCGR0_SRC_CLR, intStatus);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetExternalHostInterruptStatus
 *
 *   @b Description
 *   @n The function is used to get the status of the External Host Interrupt 
 *      register. 
 *
 *   @b Arguments
     @verbatim
        intStatus - Interrupt Status(Source ID) populated by this API.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_IPCARH_IPCGRH_SRC_CLR
 *
 *   @b Example
 *   @verbatim

        Uint32  srcId;

        // Get the External Host Interrupt Status
        CSL_BootCfgGetExternalHostInterruptStatus(&srcId);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetExternalHostInterruptStatus 
(
    Uint32* intStatus 
)
{
    *intStatus = CSL_FEXT(hBootCfg->IPCARH, BOOTCFG_IPCARH_IPCGRH_SRC_CLR);
}

/** ============================================================================
 *   @n@b CSL_BootCfgClearExternalHostInterruptStatus
 *
 *   @b Description
 *   @n The function is used to clear the status of the External Host Interrupt 
 *      register. 
 *
 *   @b Arguments
     @verbatim
        intStatus - Interrupt Status(Source ID) to be cleared
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_IPCARH_IPCGRH_SRC_CLR
 *
 *   @b Example
 *   @verbatim

        Uint32  srcId;

        // Get the External Host Interrupt Status
        CSL_BootCfgGetExternalHostInterruptStatus(&srcId);
        ...
        // Clear the Interrupt Status
        CSL_BootCfgClearExternalHostInterruptStatus(srcId);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgClearExternalHostInterruptStatus 
(
    Uint32 intStatus 
)
{
    CSL_FINS(hBootCfg->IPCARH, BOOTCFG_IPCARH_IPCGRH_SRC_CLR, intStatus);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetTimerInputSelection
 *
 *   @b Description
 *   @n The function gets the timer input selection
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Timer Input Selection
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_TINPSEL_TINPSEL
 *
 *   @b Example
 *   @verbatim

        Uint16 timerInput;

        // Get the timer input selection 
        timerInput = CSL_BootCfgGetTimerInputSelection();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint16 CSL_BootCfgGetTimerInputSelection (void)
{
    return CSL_FEXT (hBootCfg->TINPSEL,BOOTCFG_TINPSEL_TINPSEL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetTimerInputSelection
 *
 *   @b Description
 *   @n The function sets the timer input selection
 *
 *   @b Arguments
     @verbatim
        timerInput  - Timer Input selection to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Timer Input Selection
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_TINPSEL_TINPSEL
 *
 *   @b Example
 *   @verbatim

        // Set the timer input selection 
        CSL_BootCfgSetTimerInputSelection(0x1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetTimerInputSelection (Uint16 timerInput)
{
    CSL_FINS (hBootCfg->TINPSEL,BOOTCFG_TINPSEL_TINPSEL, timerInput);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetTimerOutputSelection
 *
 *   @b Description
 *   @n The function gets the timer output selection
 *
 *   @b Arguments
     @verbatim
        timerOutputSel0 -   Timer output selection 0
        timerOutputSel1 -   Timer output selection 1
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_TOUTSEL_TOUTSEL0,BOOTCFG_TOUTSEL_TOUTSEL1
 *
 *   @b Example
 *   @verbatim

        Uint8 timerOutputSel0;
        Uint8 timerOutputSel1;

        // Get the timer output selections
        CSL_BootCfgGetTimerOutputSelection(&timerOutputSel0, &timerOutputSel1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetTimerOutputSelection (Uint8* timerOutputSel0, Uint8* timerOutputSel1)
{
    Uint32 value = hBootCfg->TOUTSEL;

    *timerOutputSel0 = CSL_FEXT (value, BOOTCFG_TOUTSEL_TOUTSEL0);
    *timerOutputSel1 = CSL_FEXT (value, BOOTCFG_TOUTSEL_TOUTSEL1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetTimerOutputSelection
 *
 *   @b Description
 *   @n The function sets the timer output selection
 *
 *   @b Arguments
     @verbatim
        timerOutputSel0 -   Timer output selection 0
        timerOutputSel1 -   Timer output selection 1
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_TOUTSEL_TOUTSEL0,BOOTCFG_TOUTSEL_TOUTSEL1
 *
 *   @b Example
 *   @verbatim

        // Set the timer output selections
        CSL_BootCfgGetTimerOutputSelection(0x0, 0x1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetTimerOutputSelection (Uint8 timerOutputSel0, Uint8 timerOutputSel1)
{
    hBootCfg->TOUTSEL = CSL_FMK (BOOTCFG_TOUTSEL_TOUTSEL0, timerOutputSel0) |
                        CSL_FMK (BOOTCFG_TOUTSEL_TOUTSEL1, timerOutputSel1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetResetMuxLockStatus
 *
 *   @b Description
 *   @n The function gets the reset mux block lock status
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
        lockStatus  - Status of the register fields lock 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_RSTMUX0_RSTMUX_LOCK0
 *
 *   @b Example
 *   @verbatim

        Uint8   lockStatus;

        // Get the lock status for core 0
        CSL_BootCfgGetResetMuxLockStatus (0, &lockStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetResetMuxLockStatus (Uint8 coreIdx, Uint8* lockStatus)
{
    *lockStatus = CSL_FEXT (hBootCfg->RSTMUX[coreIdx],BOOTCFG_RSTMUX0_RSTMUX_LOCK0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetResetMuxLockStatus
 *
 *   @b Description
 *   @n The function sets the reset mux block lock status
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
        lockStatus  - Status of the register fields lock 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_RSTMUX0_RSTMUX_LOCK0
 *
 *   @b Example
 *   @verbatim

        // Lock registers for Core 0
        CSL_BootCfgSetResetMuxLockStatus (0, 1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetResetMuxLockStatus (Uint8 coreIdx, Uint8 lockStatus)
{
    CSL_FINS (hBootCfg->RSTMUX[coreIdx],BOOTCFG_RSTMUX0_RSTMUX_LOCK0, lockStatus);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetResetMuxOutputMode
 *
 *   @b Description
 *   @n The function gets the reset mux block output mode
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
        outputMode  - Output Mode populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_RSTMUX0_RSTMUX_OMODE0
 *
 *   @b Example
 *   @verbatim

        Uint8   outputMode;

        // Get the output mode for core 0
        CSL_BootCfgGetResetMuxOutputMode (0, &outputMode);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetResetMuxOutputMode (Uint8 coreIdx, Uint8* outputMode)
{
    *outputMode = CSL_FEXT (hBootCfg->RSTMUX[coreIdx], BOOTCFG_RSTMUX0_RSTMUX_OMODE0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetResetMuxOutputMode
 *
 *   @b Description
 *   @n The function sets the reset mux block output mode
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
        outputMode  - Output Mode to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_RSTMUX0_RSTMUX_OMODE0
 *
 *   @b Example
 *   @verbatim

        // Set the output mode for core 0 to ensure that the WD Timer generates a local reset
        CSL_BootCfgSetResetMuxOutputMode (0, 2);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetResetMuxOutputMode (Uint8 coreIdx, Uint8 outputMode)
{
    CSL_FINS (hBootCfg->RSTMUX[coreIdx], BOOTCFG_RSTMUX0_RSTMUX_OMODE0, outputMode);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetResetMuxDelay
 *
 *   @b Description
 *   @n The function gets the reset mux delay between NMI & IReset
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
        delay       - Delay populated by this API
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_RSTMUX0_RSTMUX_DELAY0
 *
 *   @b Example
 *   @verbatim

        Uint8   delay;

        // Get the delay for core 0
        CSL_BootCfgGetResetMuxOutputMode (0, &delay);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetResetMuxDelay (Uint8 coreIdx, Uint8* delay)
{
    *delay = CSL_FEXT (hBootCfg->RSTMUX[coreIdx], BOOTCFG_RSTMUX0_RSTMUX_DELAY0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetResetMuxDelay
 *
 *   @b Description
 *   @n The function sets the reset mux delay between NMI & IReset
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
        delay       - Delay to be configured
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_RSTMUX0_RSTMUX_DELAY0
 *
 *   @b Example
 *   @verbatim

        // Set the delay for core 0 to be 2048 CPU/6 cycles delay between NMI & lreset
        CSL_BootCfgSetResetMuxDelay (0, 0x3);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetResetMuxDelay (Uint8 coreIdx, Uint8 delay)
{
    CSL_FINS (hBootCfg->RSTMUX[coreIdx], BOOTCFG_RSTMUX0_RSTMUX_DELAY0, delay);
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsResetMuxEventDetected
 *
 *   @b Description
 *   @n The function checks if an event is detected by the Result Mux block 
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - Event has been detected
 *   @n FALSE - Event has not been detected
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_RSTMUX0_RSTMUX_EVSTAT0
 *
 *   @b Example
 *   @verbatim

        // Check if there is an event detected or not for core 1
        if (CSL_BootCfgIsResetMuxEventDetected (1) == TRUE)
        {
            // Reset Event has been detected
        }
        else
        {
            // Reset Event has NOT been detected
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsResetMuxEventDetected (Uint8 coreIdx)
{
    return (Bool)CSL_FEXT (hBootCfg->RSTMUX[coreIdx], BOOTCFG_RSTMUX0_RSTMUX_EVSTAT0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgClearResetMuxEvent
 *
 *   @b Description
 *   @n The function is to used clear the detected event in the reset mux block
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Core Index
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_RSTMUX0_RSTMUX_EVSTAT_CLR0
 *
 *   @b Example
 *   @verbatim

        // Check if there is an event detected or not for core 1
        if (CSL_BootCfgIsResetMuxEventDetected (1) == TRUE)
        {
            // Reset Event has been detected
            ...
            // Clear the reset event
            CSL_BootCfgClearResetMuxEvent(1);
        }
        else
        {
            // Reset Event has NOT been detected
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgClearResetMuxEvent (Uint8 coreIdx)
{
    CSL_FINS (hBootCfg->RSTMUX[coreIdx], BOOTCFG_RSTMUX0_RSTMUX_EVSTAT_CLR0, 1);
}




/** ============================================================================
 *   @n@b CSL_BootCfgGetCOREPLLConfiguration
 *
 *   @b Description
 *   @n The function is to used get the CORE PLL configuration.
 *
 *   @b Arguments
     @verbatim
        corePLLConfig0    - CORE PLL Configuration0.
        corePLLConfig1    - CORE PLL Configuration1.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_CORE_PLL_CTL0, BOOTCFG_CORE_PLL_CTL1
 *
 *   @b Example
 *   @verbatim

        Uint32  corePLLConfig0;
        Uint32  corePLLConfig1;

        // Get the CORE PLL Configuration
        CSL_BootCfgGetCOREPLLConfiguration (&corePLLConfig0, &corePLLConfig1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetCOREPLLConfiguration (Uint32* corePLLConfig0, Uint32* corePLLConfig1)
{
    *corePLLConfig0 = hBootCfg->CORE_PLL_CTL0;
    *corePLLConfig1 = hBootCfg->CORE_PLL_CTL1;
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetCOREPLLConfiguration
 *
 *   @b Description
 *   @n The function is to used set the CORE PLL configuration.
 *
 *   @b Arguments
     @verbatim
        corePLLConfig0    - CORE PLL Configuration0.
        corePLLConfig1    - CORE PLL Configuration1.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CORE_PLL_CTL0, BOOTCFG_CORE_PLL_CTL1
 *
 *   @b Example
 *   @verbatim

        Uint32  corePLLConfig0;
        Uint32  corePLLConfig1;

        // Get the CORE PLL Configuration
        CSL_BootCfgGetCOREPLLConfiguration (&corePLLConfig0, &corePLLConfig1);
        ...
        // Set the CORE PLL Configuration
        CSL_BootCfgSetCOREPLLConfiguration (corePLLConfig0, corePLLConfig1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetCOREPLLConfiguration (Uint32 corePLLConfig0, Uint32 corePLLConfig1)
{
    hBootCfg->CORE_PLL_CTL0 = corePLLConfig0;
    hBootCfg->CORE_PLL_CTL1 = corePLLConfig1;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetDDR3PLLConfiguration
 *
 *   @b Description
 *   @n The function is to used get the DDR3 PLL configuration.
 *
 *   @b Arguments
     @verbatim
        ddr3PLLConfig0    - DDR3 PLL Configuration0.
        ddr3PLLConfig1    - DDR3 PLL Configuration1.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_DDR3_PLL_CTL0, BOOTCFG_DDR3_PLL_CTL1
 *
 *   @b Example
 *   @verbatim

        Uint32  ddr3PLLConfig0;
        Uint32  ddr3PLLConfig1;

        // Get the DDR3 PLL Configuration
        CSL_BootCfgGetDDR3PLLConfiguration (&ddr3PLLConfig0, &ddr3PLLConfig1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetDDR3PLLConfiguration (Uint32* ddr3PLLConfig0, Uint32* ddr3PLLConfig1)
{
    *ddr3PLLConfig0 = hBootCfg->DDR3_PLL_CTL0;
    *ddr3PLLConfig1 = hBootCfg->DDR3_PLL_CTL1;
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetDDR3PLLConfiguration
 *
 *   @b Description
 *   @n The function is to used set the DDR3 PLL configuration.
 *
 *   @b Arguments
     @verbatim
        ddr3PLLConfig0    - DDR3 PLL Configuration0.
        ddr3PLLConfig1    - DDR3 PLL Configuration1.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_DDR3_PLL_CTL0, BOOTCFG_DDR3_PLL_CTL1
 *
 *   @b Example
 *   @verbatim

        Uint32  ddr3PLLConfig0;
        Uint32  ddr3PLLConfig1;

        // Get the DDR3 PLL Configuration
        CSL_BootCfgGetDDR3PLLConfiguration (&ddr3PLLConfig0, &ddr3PLLConfig1);
        ...
        // Set the DDR3 PLL Configuration
        CSL_BootCfgSetDDR3PLLConfiguration (ddr3PLLConfig0, ddr3PLLConfig1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetDDR3PLLConfiguration (Uint32 ddr3PLLConfig0, Uint32 ddr3PLLConfig1)
{
    hBootCfg->DDR3_PLL_CTL0 = ddr3PLLConfig0;
    hBootCfg->DDR3_PLL_CTL1 = ddr3PLLConfig1;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSGMIIConfigPLL
 *
 *   @b Description
 *   @n The function is to used get the SGMII configuration PLL.
 *
 *   @b Arguments
     @verbatim
        configPLL   - SGMII PLL Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SGMII_CFGPLL_SGMII_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16  configPLL;

        // Get the SGMII Configuration PLL
        CSL_BootCfgGetSGMIIConfigPLL (&configPLL);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSGMIIConfigPLL (Uint16* configPLL)
{
    *configPLL = CSL_FEXT(hBootCfg->SGMII_CFGPLL, BOOTCFG_SGMII_CFGPLL_SGMII_CFGPLL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSGMIIConfigPLL
 *
 *   @b Description
 *   @n The function is to used set the SGMII configuration PLL.
 *
 *   @b Arguments
     @verbatim
        configPLL   - SGMII PLL Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SGMII_CFGPLL_SGMII_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16  configPLL;

        // Get the SGMII Configuration PLL
        CSL_BootCfgGetSGMIIConfigPLL (&configPLL);

        // Enable the PLL and configure it.
        configPLL = configPLL | 0x1;
        CSL_BootCfgSetSGMIIConfigPLL (configPLL);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSGMIIConfigPLL (Uint16 configPLL)
{
    CSL_FINS(hBootCfg->SGMII_CFGPLL, BOOTCFG_SGMII_CFGPLL_SGMII_CFGPLL, configPLL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSGMIIRxConfig
 *
 *   @b Description
 *   @n The function is to used get the SGMII receive configuration.
 *
 *   @b Arguments
     @verbatim
        index      - Index of the SGMII Receive Configuration (0)
        rxConfig   - SGMII Receive configuration.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SGMII_CFGRX0_SGMII_RXCFG0
 *
 *   @b Example
 *   @verbatim

        Uint32  rxConfig;

        // Get the SGMII Receive Configuration
        CSL_BootCfgGetSGMIIRxConfig (0, &rxConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSGMIIRxConfig (Uint8 index, Uint32* rxConfig)
{
    if (index == 0)
        *rxConfig = CSL_FEXT(hBootCfg->SGMII_CFGRX0, BOOTCFG_SGMII_CFGRX0_SGMII_RXCFG0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSGMIIRxConfig
 *
 *   @b Description
 *   @n The function is to used set the SGMII receive configuration.
 *
 *   @b Arguments
     @verbatim
        index      - Index of the SGMII Receive Configuration (0)
        rxConfig   - SGMII Receive configuration.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SGMII_CFGRX0_SGMII_RXCFG0
 *
 *   @b Example
 *   @verbatim

        Uint32  rxConfig;

        // Get the SGMII Receive Configuration
        CSL_BootCfgGetSGMIIRxConfig (0, &rxConfig);
        ...
        // Enable the SGMII Receive
        rxConfig = rxConfig | 0x1;

        // Set the SGMII Receive configuration.
        CSL_BootCfgSetSGMIIRxConfig (0, rxConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSGMIIRxConfig (Uint8 index, Uint32 rxConfig)
{
    if (index == 0)
        CSL_FINS(hBootCfg->SGMII_CFGRX0, BOOTCFG_SGMII_CFGRX0_SGMII_RXCFG0, rxConfig);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSGMIITxConfig
 *
 *   @b Description
 *   @n The function is to used get the SGMII transmit configuration.
 *
 *   @b Arguments
     @verbatim
        index      - Index of the SGMII Transmit Configuration (0)
        txConfig   - SGMII Transmit configuration.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SGMII_CFGTX0_SGMII_TXCFG0
 *
 *   @b Example
 *   @verbatim

        Uint32  txConfig;

        // Get the SGMII Transmit Configuration
        CSL_BootCfgGetSGMIITxConfig (0, &txConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSGMIITxConfig (Uint8 index, Uint32* txConfig)
{
    if (index == 0)
        *txConfig = CSL_FEXT(hBootCfg->SGMII_CFGTX0, BOOTCFG_SGMII_CFGTX0_SGMII_TXCFG0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSGMIITxConfig
 *
 *   @b Description
 *   @n The function is to used set the SGMII transmit configuration.
 *
 *   @b Arguments
     @verbatim
        index      - Index of the SGMII Transmit Configuration (0)
        txConfig   - SGMII Transmit configuration.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SGMII_CFGTX0_SGMII_TXCFG0
 *
 *   @b Example
 *   @verbatim

        Uint32  txConfig;

        // Get the SGMII Transmit Configuration
        CSL_BootCfgGetSGMIITxConfig (0, &txConfig);

        // Enable the SGMII Transmit 
        txConfig = txConfig | 0x1;
        CSL_BootCfgSetSGMIITxConfig (0, txConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSGMIITxConfig (Uint8 index, Uint32 txConfig)
{
    if (index == 0)
        CSL_FINS(hBootCfg->SGMII_CFGTX0, BOOTCFG_SGMII_CFGTX0_SGMII_TXCFG0, txConfig);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSGMIIReserved
 *
 *   @b Description
 *   @n The function is to used get the SGMII reserved information
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - SGMII Reserved Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SGMII_RSVD_SGMII_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32  rsvdInfo;

        // Get the SGMII Reserved information
        CSL_BootCfgGetSGMIIReserved (&rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSGMIIReserved (Uint32* rsvdInfo)
{
    *rsvdInfo = CSL_FEXT(hBootCfg->SGMII_RSVD, BOOTCFG_SGMII_RSVD_SGMII_RSVD);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSGMIIReserved
 *
 *   @b Description
 *   @n The function is to used set the SGMII reserved information
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - SGMII Reserved Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SGMII_RSVD_SGMII_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32  rsvdInfo;

        // Get the SGMII Reserved information
        CSL_BootCfgGetSGMIIReserved (&rsvdInfo);
        ...
        // Set the SGMII Reserved information
        CSL_BootCfgSetSGMIIReserved (rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSGMIIReserved (Uint32 rsvdInfo)
{
    CSL_FINS (hBootCfg->SGMII_RSVD, BOOTCFG_SGMII_RSVD_SGMII_RSVD, rsvdInfo);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetPCIEConfigPLL
 *
 *   @b Description
 *   @n The function is to used get the PCIE configuration PLL
 *
 *   @b Arguments
     @verbatim
        config   - PCIE Configuration PLL
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_PCIE_CFGPLL_PCIE_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16 config;

        // Get the PCIE Configuration PLL
        CSL_BootCfgGetPCIEConfigPLL (&config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetPCIEConfigPLL (Uint16* config)
{
    *config = CSL_FEXT(hBootCfg->PCIE_CFGPLL, BOOTCFG_PCIE_CFGPLL_PCIE_CFGPLL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetPCIEConfigPLL
 *
 *   @b Description
 *   @n The function is to used set the PCIE configuration PLL
 *
 *   @b Arguments
     @verbatim
        config   - PCIE Configuration PLL
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_PCIE_CFGPLL_PCIE_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16 config;

        // Get the PCIE Configuration PLL
        CSL_BootCfgGetPCIEConfigPLL (&config);

        // Enable the PLL
        config = config | 0x1;
        CSL_BootCfgSetPCIEConfigPLL (config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetPCIEConfigPLL (Uint16 config)
{
    CSL_FINS(hBootCfg->PCIE_CFGPLL, BOOTCFG_PCIE_CFGPLL_PCIE_CFGPLL, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSRIOSERDESConfigPLL
 *
 *   @b Description
 *   @n The function is to used get the SRIO SERDES configuration PLL
 *
 *   @b Arguments
     @verbatim
        config   - SRIO SERDES Configuration PLL
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SRIO_SERDES_CFGPLL_SRIO_SERDES_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16 config;

        // Get the SRIO SERDES Configuration PLL
        CSL_BootCfgGetSRIOSERDESConfigPLL (&config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSRIOSERDESConfigPLL (Uint16* config)
{
    *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGPLL, BOOTCFG_SRIO_SERDES_CFGPLL_SRIO_SERDES_CFGPLL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSRIOSERDESConfigPLL
 *
 *   @b Description
 *   @n The function is to used set the SRIO SERDES configuration PLL
 *
 *   @b Arguments
     @verbatim
        config   - SRIO SERDES Configuration PLL
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SRIO_SERDES_CFGPLL_SRIO_SERDES_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16 config;

        // Get the SRIO SERDES Configuration PLL
        CSL_BootCfgGetSRIOSERDESConfigPLL (&config);

        // Enable the SRIO SERDES PLL Configuration.
        config = config | 0x1;
        CSL_BootCfgSetSRIOSERDESConfigPLL (config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSRIOSERDESConfigPLL (Uint16 config)
{
    CSL_FINS(hBootCfg->SRIO_SERDES_CFGPLL, BOOTCFG_SRIO_SERDES_CFGPLL_SRIO_SERDES_CFGPLL, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetPCIEReserved
 *
 *   @b Description
 *   @n The function is to used get the PCIE reserved information
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - PCIE Reserved Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_PCIE_SERDES_RSVD_PCIE_SERDES_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32  rsvdInfo;

        // Get the PCIE Reserved information
        CSL_BootCfgGetPCIEReserved (&rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetPCIEReserved (Uint8* rsvdInfo)
{
    *rsvdInfo = CSL_FEXT(hBootCfg->PCIE_SERDES_RSVD, BOOTCFG_PCIE_SERDES_RSVD_PCIE_SERDES_RSVD);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetPCIEReserved
 *
 *   @b Description
 *   @n The function is to used set the PCIE reserved information
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - PCIE Reserved Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_PCIE_SERDES_RSVD_PCIE_SERDES_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32  rsvdInfo;

        // Get the PCIE Reserved information
        CSL_BootCfgGetPCIEReserved (&rsvdInfo);
        ...
        // Set the PCIE Reserved information.
        CSL_BootCfgSetPCIEReserved (rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetPCIEReserved (Uint8 rsvdInfo)
{
    CSL_FINS(hBootCfg->PCIE_SERDES_RSVD, BOOTCFG_PCIE_SERDES_RSVD_PCIE_SERDES_RSVD, rsvdInfo);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSRIOSERDESRxConfig
 *
 *   @b Description
 *   @n The function is to used get the SRIO SERDES receive configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the SRIO SERDES to be configured (0-3)
        config   - SRIO SERDES Receive Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SRIO_SERDES_CFGRX0_SRIO_SERDES_CFGRX0;
 *   @n BOOTCFG_SRIO_SERDES_CFGRX1_SRIO_SERDES_CFGRX1;
 *   @n BOOTCFG_SRIO_SERDES_CFGRX2_SRIO_SERDES_CFGRX2;
 *   @n BOOTCFG_SRIO_SERDES_CFGRX3_SRIO_SERDES_CFGRX3
 *
 *   @b Example
 *   @verbatim

        Uint32 config;

        // Get the SRIO SERDES Receive Configuration
        CSL_BootCfgGetSRIOSERDESRxConfig (0, &config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSRIOSERDESRxConfig (Uint8 index, Uint32* config)
{
    if (index == 0)
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGRX0, BOOTCFG_SRIO_SERDES_CFGRX0_SRIO_SERDES_CFGRX0);
    else if (index == 1)
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGRX1, BOOTCFG_SRIO_SERDES_CFGRX1_SRIO_SERDES_CFGRX1);
    else if (index == 2)
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGRX2, BOOTCFG_SRIO_SERDES_CFGRX2_SRIO_SERDES_CFGRX2);
    else
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGRX3, BOOTCFG_SRIO_SERDES_CFGRX3_SRIO_SERDES_CFGRX3);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSRIOSERDESRxConfig
 *
 *   @b Description
 *   @n The function is to used set the SRIO SERDES receive configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the SRIO SERDES to be configured (0-3)
        config   - SRIO SERDES Receive Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SRIO_SERDES_CFGRX0_SRIO_SERDES_CFGRX0;
 *   @n BOOTCFG_SRIO_SERDES_CFGRX1_SRIO_SERDES_CFGRX1;
 *   @n BOOTCFG_SRIO_SERDES_CFGRX2_SRIO_SERDES_CFGRX2;
 *   @n BOOTCFG_SRIO_SERDES_CFGRX3_SRIO_SERDES_CFGRX3
 *
 *   @b Example
 *   @verbatim

        Uint32 config;

        // Get the SRIO SERDES Receive Configuration
        CSL_BootCfgGetSRIOSERDESRxConfig (1, &config);

        // Enable the SRIO SERDES Configuration
        config = config | 0x1;
        CSL_BootCfgSetSRIOSERDESRxConfig (1, config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSRIOSERDESRxConfig (Uint8 index, Uint32 config)
{
    if (index == 0)
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGRX0, BOOTCFG_SRIO_SERDES_CFGRX0_SRIO_SERDES_CFGRX0, config);
    else if (index == 1)
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGRX1, BOOTCFG_SRIO_SERDES_CFGRX1_SRIO_SERDES_CFGRX1, config);
    else if (index == 2)
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGRX2, BOOTCFG_SRIO_SERDES_CFGRX2_SRIO_SERDES_CFGRX2, config);
    else
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGRX3, BOOTCFG_SRIO_SERDES_CFGRX3_SRIO_SERDES_CFGRX3, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSRIOSERDESTxConfig
 *
 *   @b Description
 *   @n The function is to used get the SRIO SERDES transmit configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the SRIO SERDES to be configured (0-3)
        config   - SRIO SERDES Transmit Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SRIO_SERDES_CFGTX0_SRIO_SERDES_CFGTX0;
 *   @n BOOTCFG_SRIO_SERDES_CFGTX1_SRIO_SERDES_CFGTX1;
 *   @n BOOTCFG_SRIO_SERDES_CFGTX2_SRIO_SERDES_CFGTX2;
 *   @n BOOTCFG_SRIO_SERDES_CFGTX3_SRIO_SERDES_CFGTX3
 *
 *   @b Example
 *   @verbatim

        Uint32 config;

        // Get the SRIO SERDES Transmit Configuration
        CSL_BootCfgGetSRIOSERDESTxConfig (0, &config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSRIOSERDESTxConfig (Uint8 index, Uint32* config)
{
    if (index == 0)
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGTX0, BOOTCFG_SRIO_SERDES_CFGTX0_SRIO_SERDES_CFGTX0);
    else if (index == 1)
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGTX1, BOOTCFG_SRIO_SERDES_CFGTX1_SRIO_SERDES_CFGTX1);
    else if (index == 2)
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGTX2, BOOTCFG_SRIO_SERDES_CFGTX2_SRIO_SERDES_CFGTX2);
    else
        *config = CSL_FEXT(hBootCfg->SRIO_SERDES_CFGTX3, BOOTCFG_SRIO_SERDES_CFGTX3_SRIO_SERDES_CFGTX3);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSRIOSERDESTxConfig
 *
 *   @b Description
 *   @n The function is to used set the SRIO SERDES transmit configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the SRIO SERDES to be configured (0-3)
        config   - SRIO SERDES Transmit Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SRIO_SERDES_CFGTX0_SRIO_SERDES_CFGTX0;
 *   @n BOOTCFG_SRIO_SERDES_CFGTX1_SRIO_SERDES_CFGTX1;
 *   @n BOOTCFG_SRIO_SERDES_CFGTX2_SRIO_SERDES_CFGTX2;
 *   @n BOOTCFG_SRIO_SERDES_CFGTX3_SRIO_SERDES_CFGTX3
 *
 *   @b Example
 *   @verbatim

        Uint32 config;

        // Set the SRIO SERDES Transmit Configuration
        CSL_BootCfgSetSRIOSERDESTxConfig (0, &config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSRIOSERDESTxConfig (Uint8 index, Uint32 config)
{
    if (index == 0)
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGTX0, BOOTCFG_SRIO_SERDES_CFGTX0_SRIO_SERDES_CFGTX0, config);
    else if (index == 1)
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGTX1, BOOTCFG_SRIO_SERDES_CFGTX1_SRIO_SERDES_CFGTX1, config);
    else if (index == 2)
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGTX2, BOOTCFG_SRIO_SERDES_CFGTX2_SRIO_SERDES_CFGTX2, config);
    else
        CSL_FINS (hBootCfg->SRIO_SERDES_CFGTX3, BOOTCFG_SRIO_SERDES_CFGTX3_SRIO_SERDES_CFGTX3, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetSRIOSERDESReserved
 *
 *   @b Description
 *   @n The function is to used get the SRIO SERDES reserved information. All the 
 *   reserved bits of cfgrxi[] and cfgtxi[] port of the SerDes interface 
 *   are mapped to the chip-level SERDES_RSVD registers. Only one 
 *   SERDES_RSVD register is allocated per peripheral irrespective of the number 
 *   of lanes. In case of peripheral with multiplelanes, the SERDES_RSVD register 
 *   will be broadcast to all lanes.
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - SRIO SERDES Reserved information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SRIO_SERDES_RSVD_SRIO_SERDES_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32 rsvdInfo;

        // Get the SRIO SERDES Reserved Information.
        CSL_BootCfgGetSRIOSERDESReserved (&rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetSRIOSERDESReserved (Uint32* rsvdInfo)
{
    *rsvdInfo = CSL_FEXT(hBootCfg->SRIO_SERDES_RSVD, BOOTCFG_SRIO_SERDES_RSVD_SRIO_SERDES_RSVD);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetSRIOSERDESReserved
 *
 *   @b Description
 *   @n The function is to used set the SRIO SERDES reserved information.
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - SRIO SERDES Reserved information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SRIO_SERDES_RSVD_SRIO_SERDES_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32 rsvdInfo;

        // Get the SRIO SERDES Reserved Information.
        CSL_BootCfgGetSRIOSERDESReserved (&rsvdInfo);
        ...
        // Set the SRIO SERDES Reserved Information.
        CSL_BootCfgSetSRIOSERDESReserved(rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetSRIOSERDESReserved (Uint32 rsvdInfo)
{
    CSL_FINS(hBootCfg->SRIO_SERDES_RSVD, BOOTCFG_SRIO_SERDES_RSVD_SRIO_SERDES_RSVD, rsvdInfo);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetLEDPassDoneStatus
 *
 *   @b Description
 *   @n The function is used to get the LED pass done status for a specific core
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Index of the core for which the status is required.
        pass        - Pass Status Flag
        done        - Done Status Flag
        idle        - Idle Status Flag
        waitLoop    - Function Wait Loop Flag
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_LED_GEM_PASSDONE_P0,BOOTCFG_LED_GEM_PASSDONE_D0,
 *   @n BOOTCFG_LED_GEM_PASSDONE_I0,BOOTCFG_LED_GEM_PASSDONE_L0;
 *   @n BOOTCFG_LED_GEM_PASSDONE_P1,BOOTCFG_LED_GEM_PASSDONE_D1,
 *   @n BOOTCFG_LED_GEM_PASSDONE_I1,BOOTCFG_LED_GEM_PASSDONE_L1;
 *
 *   @b Example
 *   @verbatim

        Uint8  pass, 
        Uint8  done, 
        Uint8  idle, 
        Uint8  waitLoop

        // Get the LED Pass Done Status for Core 1
        CSL_BootCfgGetLEDPassDoneStatus (1, &pass, &done, &idle, &waitLoop);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetLEDPassDoneStatus 
(
    Uint8   coreIdx, 
    Uint8*  pass, 
    Uint8*  done, 
    Uint8*  idle, 
    Uint8*  waitLoop
)
{
    Uint32 value = hBootCfg->LED_GEM_PASSDONE; 

    if (coreIdx == 0)
    {
        *pass     = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_P0);
        *done     = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_D0);
        *idle     = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_I0);
        *waitLoop = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_L0);
    }else
    {
        *pass     = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_P1);
        *done     = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_D1);
        *idle     = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_I1);
        *waitLoop = CSL_FEXT (value, BOOTCFG_LED_GEM_PASSDONE_L1);
    }
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetLEDPassDoneStatus
 *
 *   @b Description
 *   @n The function is used to set the LED pass done status for a specific core
 *
 *   @b Arguments
     @verbatim
        coreIdx     - Index of the core for which the status is required.
        pass        - Pass Status Flag
        done        - Done Status Flag
        idle        - Idle Status Flag
        waitLoop    - Function Wait Loop Flag
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_LED_GEM_PASSDONE_P0,BOOTCFG_LED_GEM_PASSDONE_D0,
 *   @n BOOTCFG_LED_GEM_PASSDONE_I0,BOOTCFG_LED_GEM_PASSDONE_L0;
 *   @n BOOTCFG_LED_GEM_PASSDONE_P1,BOOTCFG_LED_GEM_PASSDONE_D1,
 *   @n BOOTCFG_LED_GEM_PASSDONE_I1,BOOTCFG_LED_GEM_PASSDONE_L1;
 *
 *   @b Example
 *   @verbatim

        Uint8  pass, 
        Uint8  done, 
        Uint8  idle, 
        Uint8  waitLoop

        // Get the LED Pass Done Status for Core 0
        CSL_BootCfgGetLEDPassDoneStatus (0, &pass, &done, &idle, &waitLoop);
        ...
        // Set the PASS Flag for core 0
        CSL_BootCfgSetLEDPassDoneStatus(0, 1, done, idle, waitLoop);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetLEDPassDoneStatus 
(
    Uint8  coreIdx, 
    Uint8  pass, 
    Uint8  done, 
    Uint8  idle, 
    Uint8  waitLoop
)
{
    if (coreIdx == 0)
    {
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_P0, pass);
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_D0, done);
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_I0, idle);
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_L0, waitLoop);
    }else
    {
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_P1, pass);
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_D1, done);
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_I1, idle);
        CSL_FINS (hBootCfg->LED_GEM_PASSDONE, BOOTCFG_LED_GEM_PASSDONE_L1, waitLoop);
    }
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsLEDPLLLocked
 *
 *   @b Description
 *   @n The function is to used check if the LED PLL is locked or not?
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE  - LED PLL Locked
 *   @n FALSE - LED PLL is not Locked
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_LED_PLLLOCK_PLLLOCK
 *
 *   @b Example
 *   @verbatim

        // Is the LED PLL Locked or not?
        if (CSL_BootCfgIsLEDPLLLocked () == TRUE)
        {
            // LED PLL is locked
        }
        else
        {
            // LED PLL is not locked.
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsLEDPLLLocked (void)
{
    return (Bool)CSL_FEXT(hBootCfg->LED_PLLLOCK, BOOTCFG_LED_PLLLOCK_PLLLOCK);
}

/** ============================================================================
 *   @n@b CSL_BootCfgLockLEDPLL
 *
 *   @b Description
 *   @n The function is to used lock the LED PLL
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_LED_PLLLOCK_STICKY_EN_LOCK=1
 *
 *   @b Example
 *   @verbatim

        // Lock the LED PLL 
        CSL_BootCfgLockLEDPLL();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgLockLEDPLL (void)
{
    CSL_FINS(hBootCfg->LED_PLLLOCK, BOOTCFG_LED_PLLLOCK_STICKY_EN_LOCK, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgUnlockLEDPLL
 *
 *   @b Description
 *   @n The function is to used unlock the LED PLL
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_LED_PLLLOCK_STICKY_EN_LOCK=0
 *
 *   @b Example
 *   @verbatim

        // Lock the LED PLL 
        CSL_BootCfgLockLEDPLL();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgUnlockLEDPLL (void)
{
    CSL_FINS(hBootCfg->LED_PLLLOCK, BOOTCFG_LED_PLLLOCK_STICKY_EN_LOCK, 0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetLEDChipPassDoneStatus
 *
 *   @b Description
 *   @n The function is to used to get the LED Chip Pass Done Status
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_LED_CHIP_PASSDONE_CHIP_PASS,BOOTCFG_LED_CHIP_PASSDONE_CHIP_DONE
 *
 *   @b Example
 *   @verbatim

        Uint8   pass;
        Uint8   done;

        // Get the LED Chip Pass Done 
        CSL_BootCfgGetLEDChipPassDoneStatus(&pass, &done);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetLEDChipPassDoneStatus (Uint8* pass, Uint8* done)
{
    Uint32 value = hBootCfg->LED_CHIP_PASSDONE;

    *pass = CSL_FEXT(value, BOOTCFG_LED_CHIP_PASSDONE_CHIP_PASS);
    *done = CSL_FEXT(value, BOOTCFG_LED_CHIP_PASSDONE_CHIP_DONE);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetLEDChipPassDoneStatus
 *
 *   @b Description
 *   @n The function is to used to set the LED Chip Pass Done Status
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_LED_CHIP_PASSDONE_CHIP_PASS,BOOTCFG_LED_CHIP_PASSDONE_CHIP_DONE
 *
 *   @b Example
 *   @verbatim

        // Set the LED Chip Pass Done flags 
        CSL_BootCfgSetLEDChipPassDoneStatus(1, 1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetLEDChipPassDoneStatus (Uint8 pass, Uint8 done)
{
    hBootCfg->LED_CHIP_PASSDONE = CSL_FMK (BOOTCFG_LED_CHIP_PASSDONE_CHIP_PASS, pass) | 
                                  CSL_FMK (BOOTCFG_LED_CHIP_PASSDONE_CHIP_DONE, done);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetDeviceSecureControl
 *
 *   @b Description
 *   @n The function is to used to get the device secure control
 *
 *   @b Arguments
     @verbatim
        vusrSecEnStatus     - VUSR Secure Enable Status
        pbistSecEnStatus    - PBIST Secure Enable Status
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_SECURE_CONTROL_VUSR_SEC_EN,BOOTCFG_SECURE_CONTROL_PBIST_SECURE_EN
 *
 *   @b Example
 *   @verbatim

        Uint8   vusrSecEnStatus;
        Uint8   pbistSecEnStatus;

        // Get the Secure Control
        CSL_BootCfgGetDeviceSecureControl(&vusrSecEnStatus, &pbistSecEnStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetDeviceSecureControl (Uint8* vusrSecEnStatus, Uint8* pbistSecEnStatus)
{
    Uint32 value = hBootCfg->SECURE_CONTROL;

    *vusrSecEnStatus = CSL_FEXT(value, BOOTCFG_SECURE_CONTROL_VUSR_SEC_EN);
    *pbistSecEnStatus= CSL_FEXT(value, BOOTCFG_SECURE_CONTROL_PBIST_SECURE_EN);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetDeviceSecureControl
 *
 *   @b Description
 *   @n The function is to used to set the device secure control. This API can
 *      only be called from a secure master
 *
 *   @b Arguments
     @verbatim
        vusrSecEnStatus     - VUSR Secure Enable Status
        pbistSecEnStatus    - PBIST Secure Enable Status
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_SECURE_CONTROL_VUSR_SEC_EN,BOOTCFG_SECURE_CONTROL_PBIST_SECURE_EN
 *
 *   @b Example
 *   @verbatim

        Uint8   vusrSecEnStatus;
        Uint8   pbistSecEnStatus;

        // Get the Secure Control
        CSL_BootCfgGetDeviceSecureControl(&vusrSecEnStatus, &pbistSecEnStatus);

        // Set the secure control.
        CSL_BootCfgSetDeviceSecureControl(1, pbistSecEnStatus);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetDeviceSecureControl (Uint8 vusrSecEnStatus, Uint8 pbistSecEnStatus)
{
    hBootCfg->SECURE_CONTROL = CSL_FMK(BOOTCFG_SECURE_CONTROL_VUSR_SEC_EN, vusrSecEnStatus)     |
                               CSL_FMK(BOOTCFG_SECURE_CONTROL_PBIST_SECURE_EN, pbistSecEnStatus);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetROMChecksum
 *
 *   @b Description
 *   @n The function is to used to get the ROM checksum
 *
 *   @b Arguments
     @verbatim
        romLSBChecksum     - LSB of 64-bit ROM Checksum
        romMSBChecksum     - MSB of 64-bit ROM Checksum
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_EFUSE_SECROM_CHKSUM0,BOOTCFG_EFUSE_SECROM_CHKSUM1
 *
 *   @b Example
 *   @verbatim

        Uint32   romLSBChecksum;
        Uint32   romMSBChecksum;

        // Get the ROM Checksum
        CSL_BootCfgGetROMChecksum(&romLSBChecksum, &romMSBChecksum);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetROMChecksum (Uint32* romLSBChecksum, Uint32* romMSBChecksum)
{
    *romLSBChecksum = hBootCfg->EFUSE_SECROM_CHKSUM0;
    *romMSBChecksum = hBootCfg->EFUSE_SECROM_CHKSUM1;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetCorePLLObservationClockControl
 *
 *   @b Description
 *   @n The function is to used to get the core PLL observation clock control
 *
 *   @b Arguments
     @verbatim
        selection - Controls which clock is output
        enable    - Status of the Core PLL IO Clock buffer.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_SEL,BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_EN
 *
 *   @b Example
 *   @verbatim

        Uint8   selection;
        Uint8   enable;

        // Get the Core PLL Observation Clock Control 
        CSL_BootCfgGetCorePLLObservationClockControl(&selection, &enable);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetCorePLLObservationClockControl 
(
    Uint8* selection, 
    Uint8* enable
)
{
    Uint32 value = hBootCfg->OBSCLK_CTL;

    *selection = CSL_FEXT (value, BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_SEL);
    *enable    = CSL_FEXT (value, BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_EN);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetCorePLLObservationClockControl
 *
 *   @b Description
 *   @n The function is to used to set the core PLL observation clock control
 *
 *   @b Arguments
     @verbatim
        selection - Controls which clock is output
        enable    - Status of the Core PLL IO Clock buffer.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_SEL,BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_EN
 *
 *   @b Example
 *   @verbatim

        Uint8   selection;
        Uint8   enable;

        // Get the Core PLL Observation Clock Control 
        CSL_BootCfgGetCorePLLObservationClockControl(&selection, &enable);
        
        // IO Clock buffer is enabled.
        CSL_BootCfgSetCorePLLObservationClockControl(selection, 1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetCorePLLObservationClockControl 
(
    Uint8 selection, 
    Uint8 enable
)
{
    CSL_FINS (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_SEL, selection);
    CSL_FINS (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_CORE_PLL_OBSCLK_EN, enable);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetDDR3PLLObservationClockControl
 *
 *   @b Description
 *   @n The function is to used to get the DDR3 PLL observation clock control
 *
 *   @b Arguments
     @verbatim
        selection - Controls which clock is output
        enable    - Status of the DDR3 PLL IO Clock buffer.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_SEL,BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_EN
 *
 *   @b Example
 *   @verbatim

        Uint8   selection;
        Uint8   enable;

        // Get the DDR3 PLL Observation Clock Control 
        CSL_BootCfgGetDDR3PLLObservationClockControl(&selection, &enable);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetDDR3PLLObservationClockControl 
(
    Uint8* selection, 
    Uint8* enable
)
{
    Uint32 value = hBootCfg->OBSCLK_CTL;

    *selection = CSL_FEXT (value, BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_SEL);
    *enable    = CSL_FEXT (value, BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_EN);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetDDR3PLLObservationClockControl
 *
 *   @b Description
 *   @n The function is to used to set the DDR3 PLL observation clock control
 *
 *   @b Arguments
     @verbatim
        selection - Controls which clock is output
        enable    - Status of the DDR3 PLL IO Clock buffer.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_SEL,BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_EN
 *
 *   @b Example
 *   @verbatim

        Uint8   selection;
        Uint8   enable;

        // Get the DDR3 PLL Observation Clock Control 
        CSL_BootCfgGetDDR3PLLObservationClockControl(&selection, &enable);

        // DDR3 IO Clock Buffer is disabled.
        CSL_BootCfgSetDDR3PLLObservationClockControl(selection, 0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetDDR3PLLObservationClockControl 
(
    Uint8 selection, 
    Uint8 enable
)
{
    CSL_FINS (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_SEL, selection);
    CSL_FINS (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_DDR3_PLL_OBSCLK_EN, enable);
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsPSCEMIF4FResetEnabled
 *
 *   @b Description
 *   @n The function is to used to get the status of PSC EMIF4F Reset Enable bit
 *      in the Observation Clock Control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_OBSCLK_CTL_PSC_EMIF4F_RST_EN
 *
 *   @b Example
 *   @verbatim

        // Get the PSC EMIF4F Reset enable bit status
        if (CSL_BootCfgIsPSCEMIF4FResetEnabled() == TRUE)
        {
            // EMIF4F/DDR3 can be reset separately from rest of the chip.
        }
        else
        {
            // EMIF4F Reset disabled.
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsPSCEMIF4FResetEnabled (void)
{
    return CSL_FEXT (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_PSC_EMIF4F_RST_EN);
}

/** ============================================================================
 *   @n@b CSL_BootCfgEnablePSCEMIF4FReset
 *
 *   @b Description
 *   @n The function enables the PSC EMIF4F Reset Enable bit
 *      in the Observation Clock Control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_OBSCLK_CTL_PSC_EMIF4F_RST_EN=1
 *
 *   @b Example
 *   @verbatim

        // Enable PSC EMIF4 Reset Enable bit
        CSL_BootCfgEnablePSCEMIF4FReset();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgEnablePSCEMIF4FReset (void)
{
    CSL_FINS (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_PSC_EMIF4F_RST_EN, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgDisablePSCEMIF4FReset
 *
 *   @b Description
 *   @n The function disables the PSC EMIF4F Reset Enable bit
 *      in the Observation Clock Control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_OBSCLK_CTL_PSC_EMIF4F_RST_EN=0
 *
 *   @b Example
 *   @verbatim

        // Disable PSC EMIF4 Reset Enable bit
        CSL_BootCfgEnablePSCEMIF4FReset();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgDisablePSCEMIF4FReset (void)
{
    CSL_FINS (hBootCfg->OBSCLK_CTL, BOOTCFG_OBSCLK_CTL_PSC_EMIF4F_RST_EN, 0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetVUSRConfigPLL
 *
 *   @b Description
 *   @n The function is to used to get the VUSR Configuration PLL 
 *
 *   @b Arguments
     @verbatim
        config   - VUSR Configuration PLL
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_VUSR_CFGPLL_VUSR_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16   config;

        // Get the VUSR Configuration PLL
        CSL_BootCfgGetVUSRConfigPLL(&config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetVUSRConfigPLL (Uint16* config)
{
    *config = CSL_FEXT(hBootCfg->VUSR_CFGPLL, BOOTCFG_VUSR_CFGPLL_VUSR_CFGPLL);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetVUSRConfigPLL
 *
 *   @b Description
 *   @n The function is to used to set the VUSR Configuration PLL 
 *
 *   @b Arguments
     @verbatim
        config   - VUSR Configuration PLL
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_VUSR_CFGPLL_VUSR_CFGPLL
 *
 *   @b Example
 *   @verbatim

        Uint16   config;

        // Get the VUSR Configuration PLL
        CSL_BootCfgGetVUSRConfigPLL(&config);
        ...
        // Enable the PLL
        config = config | 0x1;
        CSL_BootCfgSetVUSRConfigPLL (config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetVUSRConfigPLL (Uint16 config)
{
    CSL_FINS (hBootCfg->VUSR_CFGPLL, BOOTCFG_VUSR_CFGPLL_VUSR_CFGPLL, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetVUSRRxConfig
 *
 *   @b Description
 *   @n The function is to used to get the VUSR Receive Configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the VUSR Receive Configuration (0 - 3)
        config   - VUSR Receive Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_VUSR_CFGRX0_VUSR_CFGRX0;BOOTCFG_VUSR_CFGRX1_VUSR_CFGRX1;
 *   @n BOOTCFG_VUSR_CFGRX2_VUSR_CFGRX2;BOOTCFG_VUSR_CFGRX1_VUSR_CFGRX3
 *
 *   @b Example
 *   @verbatim

        Uint32   rxConfig;

        // Get the VUSR 0 Receive Configuration PLL
        CSL_BootCfgGetVUSRRxConfig(0, &rxConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetVUSRRxConfig (Uint8 index, Uint32* config)
{
    if (index == 0)
        *config = CSL_FEXT(hBootCfg->VUSR_CFGRX0, BOOTCFG_VUSR_CFGRX0_VUSR_CFGRX0);
    else if (index == 1)
        *config = CSL_FEXT(hBootCfg->VUSR_CFGRX1, BOOTCFG_VUSR_CFGRX1_VUSR_CFGRX1);
    else if (index == 2)
        *config = CSL_FEXT(hBootCfg->VUSR_CFGRX2, BOOTCFG_VUSR_CFGRX2_VUSR_CFGRX2);
    else 
        *config = CSL_FEXT(hBootCfg->VUSR_CFGRX3, BOOTCFG_VUSR_CFGRX3_VUSR_CFGRX3);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetVUSRRxConfig
 *
 *   @b Description
 *   @n The function is to used to set the VUSR Receive Configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the VUSR Receive Configuration (0 - 3)
        config   - VUSR Receive Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_VUSR_CFGRX0_VUSR_CFGRX0;BOOTCFG_VUSR_CFGRX1_VUSR_CFGRX1;
 *   @n BOOTCFG_VUSR_CFGRX2_VUSR_CFGRX2;BOOTCFG_VUSR_CFGRX1_VUSR_CFGRX3
 *
 *   @b Example
 *   @verbatim

        Uint32   rxConfig;

        // Get the VUSR 0 Receive Configuration PLL
        CSL_BootCfgGetVUSRRxConfig(0, &rxConfig);
        ...
        // Enable the Receive
        rxConfig = rxConfig | 0x1;
        CSL_BootCfgSetVUSRRxConfig (0, rxConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetVUSRRxConfig (Uint8 index, Uint32 config)
{
    if (index == 0)
        CSL_FINS (hBootCfg->VUSR_CFGRX0, BOOTCFG_VUSR_CFGRX0_VUSR_CFGRX0, config);
    else if (index == 1)
        CSL_FINS (hBootCfg->VUSR_CFGRX1, BOOTCFG_VUSR_CFGRX1_VUSR_CFGRX1, config);
    else if (index == 2)
        CSL_FINS (hBootCfg->VUSR_CFGRX2, BOOTCFG_VUSR_CFGRX2_VUSR_CFGRX2, config);
    else 
        CSL_FINS (hBootCfg->VUSR_CFGRX3, BOOTCFG_VUSR_CFGRX3_VUSR_CFGRX3, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetVUSRTxConfig
 *
 *   @b Description
 *   @n The function is to used to get the VUSR Transmit Configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the VUSR Transmit Configuration (0 - 3)
        config   - VUSR Transmit Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_VUSR_CFGTX0_VUSR_CFGTX0;BOOTCFG_VUSR_CFGTX1_VUSR_CFGTX1;
 *   @n BOOTCFG_VUSR_CFGTX2_VUSR_CFGTX2;BOOTCFG_VUSR_CFGTX3_VUSR_CFGTX3
 *
 *   @b Example
 *   @verbatim

        Uint32   txConfig;

        // Get the VUSR 0 Transmit Configuration PLL
        CSL_BootCfgGetVUSRTxConfig(0, &txConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetVUSRTxConfig (Uint8 index, Uint32* config)
{
    if (index == 0)
        *config = CSL_FEXT(hBootCfg->VUSR_CFGTX0, BOOTCFG_VUSR_CFGTX0_VUSR_CFGTX0);
    else if (index == 1)
        *config = CSL_FEXT(hBootCfg->VUSR_CFGTX1, BOOTCFG_VUSR_CFGTX1_VUSR_CFGTX1);
    else if (index == 2)
        *config = CSL_FEXT(hBootCfg->VUSR_CFGTX2, BOOTCFG_VUSR_CFGTX2_VUSR_CFGTX2);
    else 
        *config = CSL_FEXT(hBootCfg->VUSR_CFGTX3, BOOTCFG_VUSR_CFGTX3_VUSR_CFGTX3);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetVUSRTxConfig
 *
 *   @b Description
 *   @n The function is to used to set the VUSR Transmit Configuration
 *
 *   @b Arguments
     @verbatim
        index    - Index of the VUSR Transmit Configuration (0 - 3)
        config   - VUSR Transmit Configuration
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_VUSR_CFGTX0_VUSR_CFGTX0;BOOTCFG_VUSR_CFGTX1_VUSR_CFGTX1;
 *   @n BOOTCFG_VUSR_CFGTX2_VUSR_CFGTX2;BOOTCFG_VUSR_CFGTX3_VUSR_CFGTX3
 *
 *   @b Example
 *   @verbatim

        Uint32   txConfig;

        // Get the VUSR 0 Transmit Configuration PLL
        CSL_BootCfgGetVUSRTxConfig(0, &txConfig);

        // Enable the Transmit 
        txConfig = txConfig | 0x1;
        CSL_BootCfgSetVUSRTxConfig(0, txConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetVUSRTxConfig (Uint8 index, Uint32 config)
{
    if (index == 0)
        CSL_FINS (hBootCfg->VUSR_CFGTX0, BOOTCFG_VUSR_CFGTX0_VUSR_CFGTX0, config);
    else if (index == 1)
        CSL_FINS (hBootCfg->VUSR_CFGTX1, BOOTCFG_VUSR_CFGTX1_VUSR_CFGTX1, config);
    else if (index == 2)
        CSL_FINS (hBootCfg->VUSR_CFGTX2, BOOTCFG_VUSR_CFGTX2_VUSR_CFGTX2, config);
    else 
        CSL_FINS (hBootCfg->VUSR_CFGTX3, BOOTCFG_VUSR_CFGTX3_VUSR_CFGTX3, config);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetVUSRReserved
 *
 *   @b Description
 *   @n The function is to used to get the VUSR Reserved information
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - VUSR Reserved Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_VUSR_SERDES_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32   rsvdInfo;

        // Get the VUSR Reserved Information
        CSL_BootCfgGetVUSRReserved(&rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetVUSRReserved (Uint32* rsvdInfo)
{
    *rsvdInfo = hBootCfg->VUSR_SERDES_RSVD;
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetVUSRReserved
 *
 *   @b Description
 *   @n The function is to used to set the VUSR Reserved information
 *
 *   @b Arguments
     @verbatim
        rsvdInfo   - VUSR Reserved Information
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_VUSR_SERDES_RSVD
 *
 *   @b Example
 *   @verbatim

        Uint32   rsvdInfo;

        // Get the VUSR Reserved Information
        CSL_BootCfgGetVUSRReserved(&rsvdInfo);
        ...
        // Set the VUSR reserved informatio
        CSL_BootCfgSetVUSRReserved(rsvdInfo);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetVUSRReserved (Uint32 rsvdInfo)
{
    hBootCfg->VUSR_SERDES_RSVD = rsvdInfo;
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetQMPriority
 *
 *   @b Description
 *   @n The function is to used to get QM Master port transactions priority.
 *
 *   @b Arguments
     @verbatim
        qmprio  -   QM Master port transaction priority read.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_QM_PRIOR
 *
 *   @b Example
 *   @verbatim

        Uint8   qmprio;

        // Get the QM Priority from Chip Misc Control register
        CSL_BootCfgGetQMPriority(&qmprio);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetQMPriority 
(
    Uint8* qmprio
)
{
    *qmprio = CSL_FEXT (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_QM_PRIOR);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetQMPriority
 *
 *   @b Description
 *   @n The function is to used to set up QM Master port transactions priority.
 *
 *   @b Arguments
     @verbatim
        qmprio  -   QM Master port transaction priority to configure.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_QM_PRIOR
 *
 *   @b Example
 *   @verbatim

        Uint8   qmprio = 1;

        // Set the QM Priority in Chip Misc Control register
        CSL_BootCfgSetQMPriority(qmprio);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetQMPriority(Uint8 qmprio)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_QM_PRIOR, qmprio);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetGEML2ReadLatency
 *
 *   @b Description
 *   @n The function is to used to get GEM L2's memory read latency.
 *
 *   @b Arguments
     @verbatim
        latency  -   GEM L2 memory read latency
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2RDLAT
 *
 *   @b Example
 *   @verbatim

        Uint8   latency;

        // Get the GEM L2 Read latency from Chip Misc Control register
        CSL_BootCfgGetGEML2ReadLatency(&latency);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetGEML2ReadLatency 
(
    Uint8* latency
)
{
    *latency = CSL_FEXT (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2RDLAT);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetGEML2ReadLatency
 *
 *   @b Description
 *   @n The function is to used to set up GEM L2 memory read latency.
 *
 *   @b Arguments
     @verbatim
        latency  -   GEM L2 memory read latency to configure.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2RDLAT
 *
 *   @b Example
 *   @verbatim

        Uint8   latency = 1;

        // Set the GEM L2 Read latency in Chip Misc Control register
        CSL_BootCfgSetGEML2ReadLatency(latency);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetGEML2ReadLatency 
(
    Uint8   latency
)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2RDLAT, latency);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetGEML2AccessLatency
 *
 *   @b Description
 *   @n The function is to used to get GEM L2's memory access latency.
 *
 *   @b Arguments
     @verbatim
        latency  -   GEM L2 memory access latency
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2ACS_LAT
 *
 *   @b Example
 *   @verbatim

        Uint8   latency;

        // Get the GEM L2 Access latency from Chip Misc Control register
        CSL_BootCfgGetGEML2AccessLatency(&latency);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetGEML2AccessLatency 
(
    Uint8* latency
)
{
    *latency = CSL_FEXT (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2ACS_LAT);
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetGEML2AccessLatency
 *
 *   @b Description
 *   @n The function is to used to set up GEM L2 memory access latency.
 *
 *   @b Arguments
     @verbatim
        latency  -   GEM L2 memory access latency to configure.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2ACS_LAT
 *
 *   @b Example
 *   @verbatim

        Uint8   latency = 1;

        // Set the GEM L2 Access latency in Chip Misc Control register
        CSL_BootCfgSetGEML2AccessLatency(latency);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetGEML2AccessLatency 
(
    Uint8   latency
)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_L2ACS_LAT, latency);
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsDDR3PSCLockEnabled
 *
 *   @b Description
 *   @n The function is to used to get the DDR3 PSC Lock status from the 
 *      Chip Miscellaneous Control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_EMIF4F_PSC_LOCK_N
 *
 *   @b Example
 *   @verbatim

        // Get the DDR3 PSC lock status from Chip Misc Control register
        if (CSL_BootCfgIsDDR3PSCLockEnabled() == TRUE)
        {
            // DDR3 PSC lock enabled.
        }
        else
        {
            // DDR3 PSC lock disabled.
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsDDR3PSCLockEnabled (void)
{
    return CSL_FEXT (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_EMIF4F_PSC_LOCK_N);
}

/** ============================================================================
 *   @n@b CSL_BootCfgEnableDDR3PSCLock
 *
 *   @b Description
 *   @n The function enables the DDR3 PSC Lock, thus enabling DDR3 EMIF to be 
 *      reset independently of the rest of the chip.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_EMIF4F_PSC_LOCK_N=1
 *
 *   @b Example
 *   @verbatim

        // Lock DDR3 PSC from reset with chip
        CSL_BootCfgEnableDDR3PSCLock();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgEnableDDR3PSCLock (void)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_EMIF4F_PSC_LOCK_N, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgDisableDDR3PSCLock
 *
 *   @b Description
 *   @n The function disables the DDR3 PSC Lock.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_EMIF4F_PSC_LOCK_N=0
 *
 *   @b Example
 *   @verbatim

        // Unlock DDR3 PSC.
        CSL_BootCfgDisableDDR3PSCLock();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgDisableDDR3PSCLock (void)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_GEM_EMIF4F_PSC_LOCK_N, 0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgIsMSMCParityResetBlocked
 *
 *   @b Description
 *   @n The function is to used to get the MSMC parity RAM reset block status 
 *      from the Chip Miscellaneous Control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_MSMC_BLOCK_PARITY_RST
 *
 *   @b Example
 *   @verbatim

        // Get the MSMC parity block status from Chip Misc Control register
        if (CSL_BootCfgIsMSMCParityResetBlocked() == TRUE)
        {
            // MSMC parity RAM reset is blocked.
        }
        else
        {
            // MSMC parity RAM reset is un-blocked.
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_BootCfgIsMSMCParityResetBlocked (void)
{
    return CSL_FEXT (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_MSMC_BLOCK_PARITY_RST);
}

/** ============================================================================
 *   @n@b CSL_BootCfgBlockMSMCParityReset
 *
 *   @b Description
 *   @n The function enables the MSMC Parity RAM block bit, thus preventing the
 *      MSMC parity RAM from being reset.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_MSMC_BLOCK_PARITY_RST=1
 *
 *   @b Example
 *   @verbatim

        // Block MSMC parity RAM from reset
        CSL_BootCfgBlockMSMCParityReset();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgBlockMSMCParityReset (void)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_MSMC_BLOCK_PARITY_RST, 1);
}

/** ============================================================================
 *   @n@b CSL_BootCfgUnblockMSMCParityReset
 *
 *   @b Description
 *   @n The function disables the MSMC Parity RAM block bit, thus enabling the
 *      MSMC parity RAM reset control.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_CHIP_MISC_CHIP_MISC_MSMC_BLOCK_PARITY_RST=0
 *
 *   @b Example
 *   @verbatim

        // Un-Block MSMC parity RAM from reset
        CSL_BootCfgUnblockMSMCParityReset();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgUnblockMSMCParityReset (void)
{
    CSL_FINS (hBootCfg->CHIP_MISC, BOOTCFG_CHIP_MISC_CHIP_MISC_MSMC_BLOCK_PARITY_RST, 0);
}

/** ============================================================================
 *   @n@b CSL_BootCfgGetDDRConfig
 *
 *   @b Description
 *   @n The function is to used to get the DDR3 Configuration.
 *
 *   @b Arguments
     @verbatim
        regNum  - DDR Configuration Register to be read.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n BOOTCFG_DDR3_CONFIG_REG_0;BOOTCFG_DDR3_CONFIG_REG_1;BOOTCFG_DDR3_CONFIG_REG_2;
 *   @n BOOTCFG_DDR3_CONFIG_REG_3;BOOTCFG_DDR3_CONFIG_REG_4;BOOTCFG_DDR3_CONFIG_REG_5;
 *   @n BOOTCFG_DDR3_CONFIG_REG_6;BOOTCFG_DDR3_CONFIG_REG_7;BOOTCFG_DDR3_CONFIG_REG_8;
 *   @n BOOTCFG_DDR3_CONFIG_REG_9;BOOTCFG_DDR3_CONFIG_REG_10;BOOTCFG_DDR3_CONFIG_REG_11;
 *   @n BOOTCFG_DDR3_CONFIG_REG_12;BOOTCFG_DDR3_CONFIG_REG_13;BOOTCFG_DDR3_CONFIG_REG_14;
 *   @n BOOTCFG_DDR3_CONFIG_REG_15;BOOTCFG_DDR3_CONFIG_REG_16;BOOTCFG_DDR3_CONFIG_REG_17;
 *   @n BOOTCFG_DDR3_CONFIG_REG_18;BOOTCFG_DDR3_CONFIG_REG_19;BOOTCFG_DDR3_CONFIG_REG_20;
 *   @n BOOTCFG_DDR3_CONFIG_REG_21;BOOTCFG_DDR3_CONFIG_REG_22;BOOTCFG_DDR3_CONFIG_REG_23;
 *   @n BOOTCFG_DDR3_CONFIG_REG_24;BOOTCFG_DDR3_CONFIG_REG_25
 *
 *   @b Example
 *   @verbatim

        Uint32   ddr3Config;

        // Get the DDR3 Register 1 Configuration.
        CSL_BootCfgGetDDRConfig(1, &ddr3Config);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgGetDDRConfig 
(
    Uint8   regNum,
    Uint32* ddr3Config
)
{
    *ddr3Config = hBootCfg->DDR3_CONFIG_REG[regNum];
}

/** ============================================================================
 *   @n@b CSL_BootCfgSetDDRConfig
 *
 *   @b Description
 *   @n The function is to used to set the DDR3 Configuration.
 *
 *   @b Arguments
     @verbatim
        regNum  - DDR Configuration Register to be configured.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n BOOTCFG_DDR3_CONFIG_REG_0;BOOTCFG_DDR3_CONFIG_REG_1;BOOTCFG_DDR3_CONFIG_REG_2;
 *   @n BOOTCFG_DDR3_CONFIG_REG_3;BOOTCFG_DDR3_CONFIG_REG_4;BOOTCFG_DDR3_CONFIG_REG_5;
 *   @n BOOTCFG_DDR3_CONFIG_REG_6;BOOTCFG_DDR3_CONFIG_REG_7;BOOTCFG_DDR3_CONFIG_REG_8;
 *   @n BOOTCFG_DDR3_CONFIG_REG_9;BOOTCFG_DDR3_CONFIG_REG_10;BOOTCFG_DDR3_CONFIG_REG_11;
 *   @n BOOTCFG_DDR3_CONFIG_REG_12;BOOTCFG_DDR3_CONFIG_REG_13;BOOTCFG_DDR3_CONFIG_REG_14;
 *   @n BOOTCFG_DDR3_CONFIG_REG_15;BOOTCFG_DDR3_CONFIG_REG_16;BOOTCFG_DDR3_CONFIG_REG_17;
 *   @n BOOTCFG_DDR3_CONFIG_REG_18;BOOTCFG_DDR3_CONFIG_REG_19;BOOTCFG_DDR3_CONFIG_REG_20;
 *   @n BOOTCFG_DDR3_CONFIG_REG_21;BOOTCFG_DDR3_CONFIG_REG_22;BOOTCFG_DDR3_CONFIG_REG_23;
 *   @n BOOTCFG_DDR3_CONFIG_REG_24;BOOTCFG_DDR3_CONFIG_REG_25
 *
 *   @b Example
 *   @verbatim

        // Set the DDR3 Register 2 Configuration.
        CSL_BootCfgSetDDRConfig(2, 0x123565);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_BootCfgSetDDRConfig 
(
    Uint8   regNum,
    Uint32  ddr3Config
)
{
    hBootCfg->DDR3_CONFIG_REG[regNum] = ddr3Config;
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* _CSL_BOOTCFG_AUX_H_ */

