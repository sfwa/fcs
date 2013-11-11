/**
 *   @file  csl_emif4fAux.h
 *
 *   @brief   
 *      This is the EMIF4F Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the EMIF4F Module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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

#ifndef _CSL_EMIF4FAUX_H_
#define _CSL_EMIF4FAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_emif4f.h>

/** @addtogroup CSL_EMIF4F_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetModuleInfo
 *
 *   @b Description
 *   @n This function reads the peripheral ID register which identifies the 
 *      module id, RTL version information etc.
 *
 *   @b Arguments
     @verbatim
          moduleID     Module Id Information populated by this API
          rtlInfo      RTL Version Information populated by this API
          majRev       Major Revision Information populated by this API
          minRev       Minor Revision Information populated by this API
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
 *   @n EMIF4F_EMIF_MOD_ID_REV_REG_MODULE_ID,EMIF4F_EMIF_MOD_ID_REV_REG_RTL_VERSION,
 *      EMIF4F_EMIF_MOD_ID_REV_REG_MAJOR_REVISION,EMIF4F_EMIF_MOD_ID_REV_REG_MINOR_REVISION
 *
 *   @b Example
 *   @verbatim
        Uint8   moduleID;
        Uint8   rtlInfo;
        Uint8   majRev;
        Uint8   minRev;

        // Get the EMIF4F Module Information.
        CSL_EMIF4F_GetModuleInfo (&moduleID, &rtlInfo, &majRev, &minRev);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetModuleInfo
(
    Uint8*  moduleID,    
    Uint8*  rtlInfo, 
    Uint8*  majRev, 
    Uint8*  minRev
)
{
    Uint32 value = hEmif->EMIF_MOD_ID_REV;

    *moduleID = CSL_FEXT (value, EMIF4F_EMIF_MOD_ID_REV_REG_MODULE_ID);
    *rtlInfo  = CSL_FEXT (value, EMIF4F_EMIF_MOD_ID_REV_REG_RTL_VERSION);
    *majRev   = CSL_FEXT (value, EMIF4F_EMIF_MOD_ID_REV_REG_MAJOR_REVISION);
    *minRev   = CSL_FEXT (value, EMIF4F_EMIF_MOD_ID_REV_REG_MINOR_REVISION);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsBigEndian
 *
 *   @b Description
 *   @n The function gets the endianess mode in which the EMIF4F is operating. 
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   Big Endian Mode
 *   @n FALSE   -   Little Endian Mode.
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_BE
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F Endianness mode.
        if (CSL_EMIF4F_IsBigEndian () == TRUE)
        {
            // Big Endian Mode.
        }
        else
        {
            // Little Endian Mode.
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsBigEndian()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_BE);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsAsynchronous
 *
 *   @b Description
 *   @n The function gets the dual clock mode on which the EMIF4F is operating.
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   Asynchronous (V_CLK and M_CLK can have any freq ratio)
 *   @n FALSE   -   Synchronous  (V_CLK is the same as M_CLK)
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_DUAL_CLK_MODE
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F Clock
        if (CSL_EMIF4F_IsAsynchronous () == TRUE)
        {
            // Asynchronous Clock
        }
        else
        {
            // Synchronous Clock
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsAsynchronous()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_DUAL_CLK_MODE);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsFastInit
 *
 *   @b Description
 *   @n The function gets the status of the EMIF fast initialization mode 
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   Fast Initialization Mode has been enabled.
 *   @n FALSE   -   Fast Initialization Mode has been disabled.
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_FAST_INIT
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F Fast Initialization status
        if (CSL_EMIF4F_IsFastInit () == TRUE)
        {
            // Fast Initialization is disabled
        }
        else
        {
            // Fast Initialization is enabled
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsFastInit()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_FAST_INIT);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsReadDQSGateTrainingTimeout
 *
 *   @b Description
 *   @n The function gets the status of the DQS Gate Training Time out.
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   DQS Gate Training Timed out
 *   @n FALSE   -   DQS Gate Training has not Timed out
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_RDLVLGATETO
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F DQS gate training timeout status
        if (CSL_EMIF4F_IsReadDQSGateTrainingTimeout () == TRUE)
        {
            // DQS gate training has timed out
        }
        else
        {
            // DQS gate training has not timed out
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsReadDQSGateTrainingTimeout()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_RDLVLGATETO);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsReadDataEyeTrainingTimeout
 *
 *   @b Description
 *   @n The function gets the status of the Data eye training timeout.
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   Data Eye Training Timed out
 *   @n FALSE   -   Data Eye Training has not Timed out
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_RDLVLTO
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F Data Eye training timeout status
        if (CSL_EMIF4F_IsReadDataEyeTrainingTimeout () == TRUE)
        {
            // Data Eye training has timed out
        }
        else
        {
            // Data Eye training has not timed out
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsReadDataEyeTrainingTimeout()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_RDLVLTO);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsWriteLevelingTimeout
 *
 *   @b Description
 *   @n The function gets the status of the Write Leveling Timeout
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   Write Leveling Timed out
 *   @n FALSE   -   Write Leveling has not Timed out
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_WRLVLTO
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F Write Leveling timeout status
        if (CSL_EMIF4F_IsWriteLevelingTimeout () == TRUE)
        {
            // Write Leveling has timed out
        }
        else
        {
            // Write Leveling has not timed out
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsWriteLevelingTimeout()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_WRLVLTO);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_IsDDRPhyRead
 *
 *   @b Description
 *   @n The function gets the status of the DDR PHY
 *
 *   @b Arguments
     @verbatim
         None 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n TRUE    -   DDR PHY is ready
 *   @n FALSE   -   DDR PHY is not ready
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_STATUS_REG_PHY_DLL_READY
 *
 *   @b Example
 *   @verbatim

        // Get the EMIF4F DDR PHY Ready Status
        if (CSL_EMIF4F_IsDDRPhyRead () == TRUE)
        {
            // DDR PHY is ready
        }
        else
        {
            // DDR PHY is not ready
        }
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Bool CSL_EMIF4F_IsDDRPhyRead()
{
    return (Bool)CSL_FEXT(hEmif->STATUS, EMIF4F_STATUS_REG_PHY_DLL_READY);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetSDRAMConfig
 *
 *   @b Description
 *   @n The function gets the current SDRAM Configuration.
 *
 *   @b Arguments
     @verbatim
         ptrConfig  SDRAM Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_CONFIG_REG_SDRAM_TYPE,EMIF4F_SDRAM_CONFIG_REG_IBANK_POS,
 *      EMIF4F_SDRAM_CONFIG_REG_DDR_TERM,EMIF4F_SDRAM_CONFIG_REG_DDR2_DDQS,
 *      EMIF4F_SDRAM_CONFIG_REG_DYN_ODT,EMIF4F_SDRAM_CONFIG_REG_DDR_DISABLE_DLL,
 *      EMIF4F_SDRAM_CONFIG_REG_SDRAM_DRIVE,EMIF4F_SDRAM_CONFIG_REG_CWL,
 *      EMIF4F_SDRAM_CONFIG_REG_NARROW_MODE,EMIF4F_SDRAM_CONFIG_REG_CL,
 *      EMIF4F_SDRAM_CONFIG_REG_ROWSIZE,EMIF4F_SDRAM_CONFIG_REG_IBANK,
 *      EMIF4F_SDRAM_CONFIG_REG_EBANK,EMIF4F_SDRAM_CONFIG_REG_PAGESIZE,
 *   @n EMIF4F_SDRAM_CONFIG_2_REG_CS1NVMEN,EMIF4F_SDRAM_CONFIG_2_REG_EBANK_POS,
 *      EMIF4F_SDRAM_CONFIG_2_REG_RDBNUM,EMIF4F_SDRAM_CONFIG_2_REG_RDBSIZE
 *
 *   @b Example
 *   @verbatim
        EMIF4F_SDRAM_CONFIG  sdramConfig;
        ...
        // Get the EMIF4F SDRAM Configuration.
        CSL_EMIF4F_GetSDRAMConfig (&sdramConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetSDRAMConfig(EMIF4F_SDRAM_CONFIG* ptrConfig)
{
    Uint32  sdramConfig = hEmif->SDRAM_CONFIG;

    /* Extract all the fields from the SDRAM Configuration. */
    ptrConfig->type         = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_SDRAM_TYPE);
    ptrConfig->iBankPos     = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_IBANK_POS);
    ptrConfig->ddrTerm      = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DDR_TERM);
    ptrConfig->ddrDDQS      = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DDR2_DDQS);
    ptrConfig->dynODT       = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DYN_ODT);
    ptrConfig->disableDLL   = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DDR_DISABLE_DLL);
    ptrConfig->SDRAMDrive   = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_SDRAM_DRIVE);
    ptrConfig->CASWriteLat  = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_CWL);
    ptrConfig->narrowMode   = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_NARROW_MODE);
    ptrConfig->CASLatency   = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_CL);
    ptrConfig->rowSize      = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_ROWSIZE);
    ptrConfig->iBank        = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_IBANK);
    ptrConfig->eBank        = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_EBANK);
    ptrConfig->pageSize     = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_PAGESIZE);

    /* Get the SDRAM Configuration 2 Register */
    sdramConfig = hEmif->SDRAM_CONFIG_2;

    ptrConfig->NVMEnable   = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_CS1NVMEN);
    ptrConfig->eBankPos    = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_EBANK_POS);
    ptrConfig->rdbNum      = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_RDBNUM);
    ptrConfig->rdbSize     = CSL_FEXT(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_RDBSIZE); 
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetSDRAMConfig
 *
 *   @b Description
 *   @n The function sets the SDRAM Configuration as specified.
 *
 *   @b Arguments
     @verbatim
         ptrConfig  SDRAM Configuration to be configured
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
 *   @n EMIF4F_SDRAM_CONFIG_REG_SDRAM_TYPE,EMIF4F_SDRAM_CONFIG_REG_IBANK_POS,
 *      EMIF4F_SDRAM_CONFIG_REG_DDR_TERM,EMIF4F_SDRAM_CONFIG_REG_DDR2_DDQS,
 *      EMIF4F_SDRAM_CONFIG_REG_DYN_ODT,EMIF4F_SDRAM_CONFIG_REG_DDR_DISABLE_DLL,
 *      EMIF4F_SDRAM_CONFIG_REG_SDRAM_DRIVE,EMIF4F_SDRAM_CONFIG_REG_CWL,
 *      EMIF4F_SDRAM_CONFIG_REG_NARROW_MODE,EMIF4F_SDRAM_CONFIG_REG_CL,
 *      EMIF4F_SDRAM_CONFIG_REG_ROWSIZE,EMIF4F_SDRAM_CONFIG_REG_IBANK,
 *      EMIF4F_SDRAM_CONFIG_REG_EBANK,EMIF4F_SDRAM_CONFIG_REG_PAGESIZE
 *   @n EMIF4F_SDRAM_CONFIG_2_REG_CS1NVMEN,EMIF4F_SDRAM_CONFIG_2_REG_EBANK_POS,
 *      EMIF4F_SDRAM_CONFIG_2_REG_RDBNUM,EMIF4F_SDRAM_CONFIG_2_REG_RDBSIZE
 *
 *   @b Example
 *   @verbatim
        EMIF4F_SDRAM_CONFIG  sdramConfig;
        ...
        // Get the EMIF4F SDRAM Configuration.
        CSL_EMIF4F_GetSDRAMConfig (&sdramConfig);
        ...
        // Set the SDRAM Data Bus Width to be 32 bits
        sdramConfig.narrowMode = 0;

        // Set the EMIF4F SDRAM Configuration.
        CSL_EMIF4F_SetSDRAMConfig (&sdramConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetSDRAMConfig(EMIF4F_SDRAM_CONFIG* ptrConfig)
{
    Uint32  sdramConfig = 0;

    /* Initialize the SDRAM Configuration. */
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_SDRAM_TYPE, ptrConfig->type);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_IBANK_POS, ptrConfig->iBankPos);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DDR_TERM, ptrConfig->ddrTerm);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DDR2_DDQS, ptrConfig->ddrDDQS);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DYN_ODT, ptrConfig->dynODT);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_DDR_DISABLE_DLL, ptrConfig->disableDLL);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_SDRAM_DRIVE, ptrConfig->SDRAMDrive);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_CWL, ptrConfig->CASWriteLat);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_NARROW_MODE, ptrConfig->narrowMode);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_CL, ptrConfig->CASLatency);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_ROWSIZE, ptrConfig->rowSize);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_IBANK, ptrConfig->iBank);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_EBANK, ptrConfig->eBank);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_REG_PAGESIZE, ptrConfig->pageSize);

    /* Write to the register */
    hEmif->SDRAM_CONFIG = sdramConfig;
    
    /* Initialize the SDRAM2 Configuration. */
    sdramConfig = 0;    
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_CS1NVMEN, ptrConfig->NVMEnable);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_EBANK_POS, ptrConfig->eBankPos);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_RDBNUM, ptrConfig->rdbNum);
    CSL_FINS(sdramConfig, EMIF4F_SDRAM_CONFIG_2_REG_RDBSIZE, ptrConfig->rdbSize);

    /* Write to the register. */
    hEmif->SDRAM_CONFIG_2 = sdramConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_DisableInitRefresh
 *
 *   @b Description
 *   @n The function is used to disable the initialization and refresh.
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_INITREF_DIS=1
 *
 *   @b Example
 *   @verbatim

        // Disable Initialization & Refresh
        CSL_EMIF4F_DisableInitRefresh();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_DisableInitRefresh(void)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_INITREF_DIS, (Uint32)1);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_EnableInitRefresh
 *
 *   @b Description
 *   @n The function is used to enable the initialization and refresh.
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_INITREF_DIS=0
 *
 *   @b Example
 *   @verbatim

        // Enable Initialization & Refresh
        CSL_EMIF4F_EnableInitRefresh();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_EnableInitRefresh(void)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_INITREF_DIS, 0);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetSelfRefreshTemp
 *
 *   @b Description
 *   @n The function is used to get the DDR3 Self Refresh Temperature.
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n 0 - Normal Operating Temperature
 *   @n 1 - Extended Operating Temperature
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_SRT
 *
 *   @b Example
 *   @verbatim

        // Get the current self refresh temperature configuration.
        if (CSL_EMIF4F_GetSelfRefreshTemp() == 0)
        {
            // Normal Operating Temperature
        }
        else
        {
            // Extended Operating Temperature
        }

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_EMIF4F_GetSelfRefreshTemp(void)
{
    return CSL_FEXT(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_SRT);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetSelfRefreshTemp
 *
 *   @b Description
 *   @n The function is used to get the DDR3 Self Refresh Temperature.
 *
 *   @b Arguments
     @verbatim
         temperature    - Operating Temperature to be configured.
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_SRT
 *
 *   @b Example
 *   @verbatim

        // Set Self Referesh Temperature for normal operation.
        CSL_EMIF4F_SetSelfRefreshTemp(0);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetSelfRefreshTemp(Uint8 temperature)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_SRT, temperature);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_EnableAutoRefresh
 *
 *   @b Description
 *   @n The function is used to enable auto self refresh.
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_ASR=1
 *
 *   @b Example
 *   @verbatim

        // Enable Auto Self Refresh
        CSL_EMIF4F_EnableAutoRefresh();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_EnableAutoRefresh(void)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_ASR, 1);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_DisableAutoRefresh
 *
 *   @b Description
 *   @n The function is used to disable auto self refresh.
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_ASR=0
 *
 *   @b Example
 *   @verbatim

        // Disable Auto Self Refresh
        CSL_EMIF4F_DisableAutoRefresh();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_DisableAutoRefresh(void)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_ASR, 0);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPartialArraySelfRefresh
 *
 *   @b Description
 *   @n The function is used to get the partial array self refresh.
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_PASR
 *
 *   @b Example
 *   @verbatim
        Uint8 pasr;

        // Get the Partial Array Refresh Rate Configuration
        pasr = CSL_EMIF4F_GetPartialArraySelfRefresh();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_EMIF4F_GetPartialArraySelfRefresh(void)
{
    return CSL_FEXT(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_PASR);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPartialArraySelfRefresh
 *
 *   @b Description
 *   @n The function is used to set the partial array self refresh.
 *
 *   @b Arguments
     @verbatim
         pasr   Partial Array Self Refresh configuration
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_PASR
 *
 *   @b Example
 *   @verbatim

        // Set the Partial Array Refresh Rate Configuration to be for full array
        CSL_EMIF4F_SetPartialArraySelfRefresh(0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPartialArraySelfRefresh(Uint8 pasr)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_PASR, pasr);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetRefreshRate
 *
 *   @b Description
 *   @n The function is used to get the refresh rate
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Current Refresh Rate configuration
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_REFRESH_RATE
 *
 *   @b Example
 *   @verbatim
        Uint8   refreshRate;

        // Get the Refresh Rate 
        refreshRate = CSL_EMIF4F_GetRefreshRate();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint16 CSL_EMIF4F_GetRefreshRate(void)
{
    return CSL_FEXT(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_REFRESH_RATE);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetRefreshRate
 *
 *   @b Description
 *   @n The function is used to set the refresh rate
 *
 *   @b Arguments
     @verbatim
         refreshRate   Refresh Rate to be configured
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
 *   @n EMIF4F_SDRAM_REF_CTRL_REG_REFRESH_RATE
 *
 *   @b Example
 *   @verbatim

        // Enable Initialization & Refresh
        CSL_EMIF4F_EnableInitRefresh();

        // Set the Refresh Rate to be 15.7 us 
        CSL_EMIF4F_SetRefreshRate(0x7AA);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetRefreshRate(Uint16 refreshRate)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL, EMIF4F_SDRAM_REF_CTRL_REG_REFRESH_RATE, refreshRate);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowRefreshRate
 *
 *   @b Description
 *   @n The function is used to get the refresh rate from the shadow register
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Current Shadow register Refresh Rate configuration
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_SDRAM_REF_CTRL_SHDW_REG_REFRESH_RATE_SHDW
 *
 *   @b Example
 *   @verbatim
        Uint8   refreshRate;

        // Get the Shadow Register Refresh Rate 
        refreshRate = CSL_EMIF4F_GetShadowRefreshRate();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint16 CSL_EMIF4F_GetShadowRefreshRate(void)
{
    return CSL_FEXT(hEmif->SDRAM_REF_CTRL_SHDW, EMIF4F_SDRAM_REF_CTRL_SHDW_REG_REFRESH_RATE_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowRefreshRate
 *
 *   @b Description
 *   @n The function is used to set the refresh rate
 *
 *   @b Arguments
     @verbatim
         refreshRate   Refresh Rate to be configured
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
 *   @n EMIF4F_SDRAM_REF_CTRL_SHDW_REG_REFRESH_RATE_SHDW
 *
 *   @b Example
 *   @verbatim

        // Set the Shadow Register Refresh Rate to be 15.7 us 
        CSL_EMIF4F_SetShadowRefreshRate(0x7AA);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowRefreshRate(Uint16 refreshRate)
{
    CSL_FINS(hEmif->SDRAM_REF_CTRL_SHDW, EMIF4F_SDRAM_REF_CTRL_SHDW_REG_REFRESH_RATE_SHDW, refreshRate);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetTiming1Config
 *
 *   @b Description
 *   @n The function is used to get the SDRAM Timing1 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_TIM_1_REG_T_RP, EMIF4F_SDRAM_TIM_1_REG_T_RCD,
 *      EMIF4F_SDRAM_TIM_1_REG_T_WR, EMIF4F_SDRAM_TIM_1_REG_T_RAS,
 *      EMIF4F_SDRAM_TIM_1_REG_T_RC, EMIF4F_SDRAM_TIM_1_REG_T_RRD,
 *      EMIF4F_SDRAM_TIM_1_REG_T_WTR
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING1_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetTiming1Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetTiming1Config(EMIF4F_TIMING1_CONFIG* ptrTimingConfig)
{   
    Uint32 timingConfig = hEmif->SDRAM_TIM_1;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_rp  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RP);
    ptrTimingConfig->t_rcd = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RCD);
    ptrTimingConfig->t_wr  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_WR);
    ptrTimingConfig->t_ras = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RAS);
    ptrTimingConfig->t_rc  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RC);
    ptrTimingConfig->t_rrd = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RRD);
    ptrTimingConfig->t_wtr = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_WTR);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetTiming1Config
 *
 *   @b Description
 *   @n The function is used to set the SDRAM Timing1 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_SDRAM_TIM_1_REG_T_RP, EMIF4F_SDRAM_TIM_1_REG_T_RCD,
 *      EMIF4F_SDRAM_TIM_1_REG_T_WR, EMIF4F_SDRAM_TIM_1_REG_T_RAS,
 *      EMIF4F_SDRAM_TIM_1_REG_T_RC, EMIF4F_SDRAM_TIM_1_REG_T_RRD,
 *      EMIF4F_SDRAM_TIM_1_REG_T_WTR
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING1_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetTiming1Config(&timingConfig);

        // Set the min number of m_clk cycles from precharge to activate
        timingConfig.t_rp = 1;

        // Set the timing configuration.
        CSL_EMIF4F_SetTiming1Config (&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetTiming1Config(EMIF4F_TIMING1_CONFIG* ptrTimingConfig)
{   
    Uint32 timingConfig = 0;

    /* Insert all the fields from the Timing Register */
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RP,  ptrTimingConfig->t_rp);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RCD, ptrTimingConfig->t_rcd);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_WR,  ptrTimingConfig->t_wr);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RAS, ptrTimingConfig->t_ras);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RC,  ptrTimingConfig->t_rc);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_RRD, ptrTimingConfig->t_rrd);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_REG_T_WTR, ptrTimingConfig->t_wtr);

    /* Initialize the timing configuration register. */
    hEmif->SDRAM_TIM_1 = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowTiming1Config
 *
 *   @b Description
 *   @n The function is used to get the Shadow SDRAM Timing1 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RP_SHDW, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RCD_SHDW,
 *      EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WR_SHDW, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RAS_SHDW,
 *      EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RC_SHDW, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RRD_SHDW,
 *      EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WTR_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING1_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetShadowTiming1Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetShadowTiming1Config(EMIF4F_TIMING1_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = hEmif->SDRAM_TIM_1_SHDW;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_rp  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RP_SHDW);
    ptrTimingConfig->t_rcd = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RCD_SHDW);
    ptrTimingConfig->t_wr  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WR_SHDW);
    ptrTimingConfig->t_ras = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RAS_SHDW);
    ptrTimingConfig->t_rc  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RC_SHDW);
    ptrTimingConfig->t_rrd = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RRD_SHDW);
    ptrTimingConfig->t_wtr = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WTR_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowTiming1Config
 *
 *   @b Description
 *   @n The function is used to set the Shadow SDRAM Timing1 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RP_SHDW, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RCD_SHDW,
 *      EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WR_SHDW, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RAS_SHDW,
 *      EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RC_SHDW, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RRD_SHDW,
 *      EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WTR_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING1_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetShadowTiming1Config(&timingConfig);
        ...
        // Set the min number of m_clk cycles from precharge to activate
        timingConfig.t_rp = 1;

        // Configure the Timing Shadow         
        CSL_EMIF4F_SetShadowTiming1Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowTiming1Config(EMIF4F_TIMING1_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = 0;

    /* Insert all the fields from the Timing Register */
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RP_SHDW,  ptrTimingConfig->t_rp);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RCD_SHDW, ptrTimingConfig->t_rcd);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WR_SHDW,  ptrTimingConfig->t_wr);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RAS_SHDW, ptrTimingConfig->t_ras);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RC_SHDW,  ptrTimingConfig->t_rc);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_RRD_SHDW, ptrTimingConfig->t_rrd);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_1_SHDW_REG_T_WTR_SHDW, ptrTimingConfig->t_wtr);

    /* Initialize the register */
    hEmif->SDRAM_TIM_1_SHDW = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetTiming2Config
 *
 *   @b Description
 *   @n The function is used to get the SDRAM Timing2 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_TIM_2_REG_T_XP,  EMIF4F_SDRAM_TIM_2_REG_T_ODT,
 *      EMIF4F_SDRAM_TIM_2_REG_T_XSNR,EMIF4F_SDRAM_TIM_2_REG_T_XSRD,
 *      EMIF4F_SDRAM_TIM_2_REG_T_RTP, EMIF4F_SDRAM_TIM_2_REG_T_CKE
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING2_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetTiming2Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetTiming2Config(EMIF4F_TIMING2_CONFIG* ptrTimingConfig)
{   
    Uint32 timingConfig = hEmif->SDRAM_TIM_2;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_xp   = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_XP);
    ptrTimingConfig->t_odt  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_ODT);
    ptrTimingConfig->t_xsnr = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_XSNR);
    ptrTimingConfig->t_xsrd = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_XSRD);
    ptrTimingConfig->t_rtp  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_RTP);
    ptrTimingConfig->t_cke  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_CKE);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetTiming2Config
 *
 *   @b Description
 *   @n The function is used to set the SDRAM Timing2 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_SDRAM_TIM_2_REG_T_XP,  EMIF4F_SDRAM_TIM_2_REG_T_ODT,
 *      EMIF4F_SDRAM_TIM_2_REG_T_XSNR,EMIF4F_SDRAM_TIM_2_REG_T_XSRD,
 *      EMIF4F_SDRAM_TIM_2_REG_T_RTP, EMIF4F_SDRAM_TIM_2_REG_T_CKE
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING2_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetTiming2Config(&timingConfig);

        // Set the number of m_clk cycles from Power Down to any command
        timingConfig.t_xp = 0x2;

        // Set the new timing configuration.
        CSL_EMIF4F_SetTiming2Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetTiming2Config(EMIF4F_TIMING2_CONFIG* ptrTimingConfig)
{   
    Uint32 timingConfig = 0;

    /* Initialize all the fields for the timing configuration */
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_XP,  ptrTimingConfig->t_xp);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_ODT, ptrTimingConfig->t_odt);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_XSNR,ptrTimingConfig->t_xsnr);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_XSRD,ptrTimingConfig->t_xsrd);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_RTP, ptrTimingConfig->t_rtp);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_REG_T_CKE, ptrTimingConfig->t_cke);

    /* Initialize the register */
    hEmif->SDRAM_TIM_2 = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowTiming2Config
 *
 *   @b Description
 *   @n The function is used to get the Shadow SDRAM Timing2 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XP_SHDW,  EMIF4F_SDRAM_TIM_2_SHDW_REG_T_ODT_SHDW,
 *      EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSNR_SHDW,EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSRD_SHDW,
 *      EMIF4F_SDRAM_TIM_2_SHDW_REG_T_RTP_SHDW, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_CKE_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING2_CONFIG timingConfig;

        // Get the current shadow timing configuration.
        CSL_EMIF4F_GetShadowTiming2Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetShadowTiming2Config(EMIF4F_TIMING2_CONFIG* ptrTimingConfig)
{   
    Uint32 timingConfig = hEmif->SDRAM_TIM_2_SHDW;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_xp   = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XP_SHDW);
    ptrTimingConfig->t_odt  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_ODT_SHDW);
    ptrTimingConfig->t_xsnr = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSNR_SHDW);
    ptrTimingConfig->t_xsrd = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSRD_SHDW);
    ptrTimingConfig->t_rtp  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_RTP_SHDW);
    ptrTimingConfig->t_cke  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_CKE_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowTiming2Config
 *
 *   @b Description
 *   @n The function is used to set the Shadow SDRAM Timing2 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XP_SHDW,  EMIF4F_SDRAM_TIM_2_SHDW_REG_T_ODT_SHDW,
 *      EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSNR_SHDW,EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSRD_SHDW,
 *      EMIF4F_SDRAM_TIM_2_SHDW_REG_T_RTP_SHDW, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_CKE_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING2_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetShadowTiming2Config(&timingConfig);

        // Set the number of m_clk cycles from Power Down to any command
        timingConfig.t_xp = 0x2;

        // Set the new timing configuration.
        CSL_EMIF4F_SetShadowTiming2Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowTiming2Config(EMIF4F_TIMING2_CONFIG* ptrTimingConfig)
{   
    Uint32 timingConfig = 0;

    /* Initialize all the fields for the timing configuration */
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XP_SHDW,  ptrTimingConfig->t_xp);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_ODT_SHDW, ptrTimingConfig->t_odt);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSNR_SHDW,ptrTimingConfig->t_xsnr);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_XSRD_SHDW,ptrTimingConfig->t_xsrd);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_RTP_SHDW, ptrTimingConfig->t_rtp);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_2_SHDW_REG_T_CKE_SHDW, ptrTimingConfig->t_cke);

    /* Initialize the register */
    hEmif->SDRAM_TIM_2_SHDW = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetTiming3Config
 *
 *   @b Description
 *   @n The function is used to get the SDRAM Timing3 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_TIM_3_REG_T_PDLL_UL,  EMIF4F_SDRAM_TIM_3_REG_T_CSTA,
 *      EMIF4F_SDRAM_TIM_3_REG_T_CKESR,  EMIF4F_SDRAM_TIM_3_REG_ZQ_ZQCS,
 *      EMIF4F_SDRAM_TIM_3_REG_T_TDQSCKMAX,EMIF4F_SDRAM_TIM_3_REG_T_RFC,
 *      EMIF4F_SDRAM_TIM_3_REG_T_RAS_MAX
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING3_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetTiming3Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetTiming3Config(EMIF4F_TIMING3_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = hEmif->SDRAM_TIM_3;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_pdll_ul  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_PDLL_UL);
    ptrTimingConfig->t_csta     = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_CSTA);
    ptrTimingConfig->t_ckesr    = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_CKESR);
    ptrTimingConfig->zq_zqcs    = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_ZQ_ZQCS);
    ptrTimingConfig->t_tdqsckmax= CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_TDQSCKMAX);
    ptrTimingConfig->t_rfc      = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_RFC);
    ptrTimingConfig->t_rasMax   = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_RAS_MAX);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetTiming3Config
 *
 *   @b Description
 *   @n The function is used to set the SDRAM Timing3 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_SDRAM_TIM_3_REG_T_PDLL_UL,  EMIF4F_SDRAM_TIM_3_REG_T_CSTA,
 *      EMIF4F_SDRAM_TIM_3_REG_T_CKESR,  EMIF4F_SDRAM_TIM_3_REG_ZQ_ZQCS,
 *      EMIF4F_SDRAM_TIM_3_REG_T_TDQSCKMAX,EMIF4F_SDRAM_TIM_3_REG_T_RFC,
 *      EMIF4F_SDRAM_TIM_3_REG_T_RAS_MAX
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING3_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetTiming3Config(&timingConfig);

        // Set the Maximum number of reg_refresh_rate intervals from Activate to Precharge command. 
        timingConfig.t_rasMax = 0x3;

        // Set the timing configuration 
        CSL_EMIF4F_SetTiming3Config (&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetTiming3Config(EMIF4F_TIMING3_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = 0;

    /* Initialize all the fields */
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_PDLL_UL,    ptrTimingConfig->t_pdll_ul);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_CSTA,    ptrTimingConfig->t_csta);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_CKESR,    ptrTimingConfig->t_ckesr);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_ZQ_ZQCS,    ptrTimingConfig->zq_zqcs);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_TDQSCKMAX,ptrTimingConfig->t_tdqsckmax);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_RFC,      ptrTimingConfig->t_rfc);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_REG_T_RAS_MAX,  ptrTimingConfig->t_rasMax);

    /* Configure the register */
    hEmif->SDRAM_TIM_3 = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowTiming3Config
 *
 *   @b Description
 *   @n The function is used to get the Shadow SDRAM Timing3 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_SDRAM_TIM_3_SHDW_REG_T_PDLL_UL_SHDW,   EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CSTA_SHDW,
 *      EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CKESR_SHDW,     EMIF4F_SDRAM_TIM_3_SHDW_REG_ZQ_ZQCS_SHDW,
 *      EMIF4F_SDRAM_TIM_3_SHDW_REG_T_TDQSCKMAX_SHDW, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RFC_SHDW,
 *      EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RAS_MAX_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING3_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetShadowTiming3Config(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetShadowTiming3Config(EMIF4F_TIMING3_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = hEmif->SDRAM_TIM_3_SHDW;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_pdll_ul  = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_PDLL_UL_SHDW);
    ptrTimingConfig->t_csta     = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CSTA_SHDW);
    ptrTimingConfig->t_ckesr    = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CKESR_SHDW);
    ptrTimingConfig->zq_zqcs    = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_ZQ_ZQCS_SHDW);
    ptrTimingConfig->t_tdqsckmax= CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_TDQSCKMAX_SHDW);
    ptrTimingConfig->t_rfc      = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RFC_SHDW);
    ptrTimingConfig->t_rasMax   = CSL_FEXT(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RAS_MAX_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowTiming3Config
 *
 *   @b Description
 *   @n The function is used to set the Shadow SDRAM Timing3 Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_SDRAM_TIM_3_SHDW_REG_T_PDLL_UL_SHDW,   EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CSTA_SHDW,
 *      EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CKESR_SHDW,     EMIF4F_SDRAM_TIM_3_SHDW_REG_ZQ_ZQCS_SHDW,
 *      EMIF4F_SDRAM_TIM_3_SHDW_REG_T_TDQSCKMAX_SHDW, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RFC_SHDW,
 *      EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RAS_MAX_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TIMING3_CONFIG timingConfig;

        // Get the current timing configuration.
        CSL_EMIF4F_GetShadowTiming3Config(&timingConfig);

        // Set the Maximum number of reg_refresh_rate intervals from Activate to Precharge command. 
        timingConfig.t_rasMax = 0x3;

        // Set the timing configuration 
        CSL_EMIF4F_SetShadowTiming3Config (&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowTiming3Config(EMIF4F_TIMING3_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = 0;

    /* Initialize all the fields */
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_PDLL_UL_SHDW,    ptrTimingConfig->t_pdll_ul);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CSTA_SHDW,    ptrTimingConfig->t_csta);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_CKESR_SHDW,    ptrTimingConfig->t_ckesr);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_ZQ_ZQCS_SHDW,    ptrTimingConfig->zq_zqcs);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_TDQSCKMAX_SHDW,ptrTimingConfig->t_tdqsckmax);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RFC_SHDW,      ptrTimingConfig->t_rfc);
    CSL_FINS(timingConfig, EMIF4F_SDRAM_TIM_3_SHDW_REG_T_RAS_MAX_SHDW,  ptrTimingConfig->t_rasMax);

    /* Configure the register */
    hEmif->SDRAM_TIM_3_SHDW = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLPDDR2NVMTimingConfig
 *
 *   @b Description
 *   @n The function is used to get the LPDDR2 NVM Timing Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_XP,  EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WTR,
 *      EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RP,  EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WRA,
 *      EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RRD, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RCDMIN
 *
 *   @b Example
 *   @verbatim
        EMIF4F_LPDDR2NVM_TIMING_CONFIG timingConfig;

        // Get the current LPDDR2 NVM configuration.
        CSL_EMIF4F_GetLPDDR2NVMTimingConfig(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetLPDDR2NVMTimingConfig(EMIF4F_LPDDR2NVM_TIMING_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = hEmif->LPDDR2_NVM_TIM;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_xp       = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_XP);
    ptrTimingConfig->t_wtr      = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WTR);
    ptrTimingConfig->t_rp       = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RP);
    ptrTimingConfig->t_wra      = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WRA);
    ptrTimingConfig->t_rrd      = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RRD);
    ptrTimingConfig->t_rcdmin   = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RCDMIN);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLPDDR2NVMTimingConfig
 *
 *   @b Description
 *   @n The function is used to set the LPDDR2 NVM Timing Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_XP,  EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WTR,
 *      EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RP,  EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WRA,
 *      EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RRD, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RCDMIN
 *
 *   @b Example
 *   @verbatim
        EMIF4F_LPDDR2NVM_TIMING_CONFIG timingConfig;

        // Get the current LPDDR2 NVM configuration.
        CSL_EMIF4F_GetLPDDR2NVMTimingConfig(&timingConfig);

        // Set the Minimum number of m_clk cycles from Power-Down exit to any command
        timingConfig.t_xp = 1;

        // Set the LPDDR2 NVM configuration.
        CSL_EMIF4F_SetLPDDR2NVMTimingConfig(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLPDDR2NVMTimingConfig(EMIF4F_LPDDR2NVM_TIMING_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = 0;

    /* Extract all the fields from the Timing Register */
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_XP,    ptrTimingConfig->t_xp);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WTR,   ptrTimingConfig->t_wtr);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RP,    ptrTimingConfig->t_rp);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_WRA,   ptrTimingConfig->t_wra);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RRD,   ptrTimingConfig->t_rrd);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_REG_NVM_T_RCDMIN,ptrTimingConfig->t_rcdmin);

    /* Initialize the register. */
    hEmif->LPDDR2_NVM_TIM = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowLPDDR2NVMTimingConfig
 *
 *   @b Description
 *   @n The function is used to get the Shadow LPDDR2 NVM Timing Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration populated by this API
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
 *   @n EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_XP_SHDW,  EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WTR_SHDW,
 *      EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RP_SHDW,  EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WRA_SHDW,
 *      EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RRD_SHDW, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RCDMIN_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_LPDDR2NVM_TIMING_CONFIG timingConfig;

        // Get the current Shadow LPDDR2 NVM configuration.
        CSL_EMIF4F_GetShadowLPDDR2NVMTimingConfig(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetShadowLPDDR2NVMTimingConfig(EMIF4F_LPDDR2NVM_TIMING_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = hEmif->LPDDR2_NVM_TIM_SHDW;

    /* Extract all the fields from the Timing Register */
    ptrTimingConfig->t_xp       = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_XP_SHDW);
    ptrTimingConfig->t_wtr      = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WTR_SHDW);
    ptrTimingConfig->t_rp       = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RP_SHDW);
    ptrTimingConfig->t_wra      = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WRA_SHDW);
    ptrTimingConfig->t_rrd      = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RRD_SHDW);
    ptrTimingConfig->t_rcdmin   = CSL_FEXT(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RCDMIN_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowLPDDR2NVMTimingConfig
 *
 *   @b Description
 *   @n The function is used to set the Shadow LPDDR2 NVM Timing Configuration
 *
 *   @b Arguments
     @verbatim
         ptrTimingConfig       Timing Configuration to be configured
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
 *   @n EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_XP_SHDW,  EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WTR_SHDW,
 *      EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RP_SHDW,  EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WRA_SHDW,
 *      EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RRD_SHDW, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RCDMIN_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_LPDDR2NVM_TIMING_CONFIG timingConfig;

        // Get the current Shadow LPDDR2 NVM configuration.
        CSL_EMIF4F_GetShadowLPDDR2NVMTimingConfig(&timingConfig);

        // Set the Minimum number of m_clk cycles from Power-Down exit to any command
        timingConfig.t_xp = 1;

        // Set the Shadow LPDDR2 NVM configuration.
        CSL_EMIF4F_SetShadowLPDDR2NVMTimingConfig(&timingConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowLPDDR2NVMTimingConfig(EMIF4F_LPDDR2NVM_TIMING_CONFIG* ptrTimingConfig)
{
    Uint32 timingConfig = 0;

    /* Extract all the fields from the Timing Register */
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_XP_SHDW,    ptrTimingConfig->t_xp);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WTR_SHDW,   ptrTimingConfig->t_wtr);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RP_SHDW,    ptrTimingConfig->t_rp);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_WRA_SHDW,   ptrTimingConfig->t_wra);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RRD_SHDW,   ptrTimingConfig->t_rrd);
    CSL_FINS(timingConfig, EMIF4F_LPDDR2_NVM_TIM_SHDW_REG_NVM_T_RCDMIN_SHDW,ptrTimingConfig->t_rcdmin);

    /* Initialize the register. */
    hEmif->LPDDR2_NVM_TIM_SHDW = timingConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPowerMgmtConfig
 *
 *   @b Description
 *   @n The function is used to get the Power Management Configuration.
 *
 *   @b Arguments
     @verbatim
         ptrPwrMgmtConfig       Power Management Configuration populated by this API
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
 *   @n EMIF4F_PWR_MGMT_CTRL_REG_PD_TIM,  EMIF4F_PWR_MGMT_CTRL_REG_DPD_EN,
 *      EMIF4F_PWR_MGMT_CTRL_REG_LP_MODE, EMIF4F_PWR_MGMT_CTRL_REG_SR_TIM,
 *      EMIF4F_PWR_MGMT_CTRL_REG_CS_TIM
 *
 *   @b Example
 *   @verbatim
        EMIF4F_PWR_MGMT_CONFIG pwrConfig;

        // Get the power management configuration.
        CSL_EMIF4F_GetPowerMgmtConfig(&pwrConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPowerMgmtConfig(EMIF4F_PWR_MGMT_CONFIG* ptrPwrMgmtConfig)
{
    Uint32 powerMgmtConfig = hEmif->PWR_MGMT_CTRL;

    /* Extract all the fields from the Power Management Register */
    ptrPwrMgmtConfig->pdTime     = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_PD_TIM);
    ptrPwrMgmtConfig->dpdEnable  = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_DPD_EN);
    ptrPwrMgmtConfig->lpMode     = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_LP_MODE);
    ptrPwrMgmtConfig->srTime     = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_SR_TIM);
    ptrPwrMgmtConfig->csTime     = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_CS_TIM);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPowerMgmtConfig
 *
 *   @b Description
 *   @n The function is used to set the Power Management Configuration.
 *
 *   @b Arguments
     @verbatim
         ptrPwrMgmtConfig       Power Management Configuration to be configured
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
 *   @n EMIF4F_PWR_MGMT_CTRL_REG_PD_TIM,  EMIF4F_PWR_MGMT_CTRL_REG_DPD_EN,
 *      EMIF4F_PWR_MGMT_CTRL_REG_LP_MODE, EMIF4F_PWR_MGMT_CTRL_REG_SR_TIM,
 *      EMIF4F_PWR_MGMT_CTRL_REG_CS_TIM
 *
 *   @b Example
 *   @verbatim
        EMIF4F_PWR_MGMT_CONFIG pwrConfig;

        // Get the power management configuration.
        CSL_EMIF4F_GetPowerMgmtConfig(&pwrConfig);

        // Enable Deep Power Down Mode
        pwrConfig.dpdEnable = 1;

        // Set the power management configuration
        CSL_EMIF4F_SetPowerMgmtConfig(&pwrConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPowerMgmtConfig(EMIF4F_PWR_MGMT_CONFIG* ptrPwrMgmtConfig)
{
    Uint32 powerMgmtConfig = 0;

    /* Initialzie all the fields for the Power Management Register */
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_PD_TIM,  ptrPwrMgmtConfig->pdTime);
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_DPD_EN,  ptrPwrMgmtConfig->dpdEnable);
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_LP_MODE, ptrPwrMgmtConfig->lpMode);
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_SR_TIM,  ptrPwrMgmtConfig->srTime);
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_REG_CS_TIM,  ptrPwrMgmtConfig->csTime);

    /* Initialize the register */
    hEmif->PWR_MGMT_CTRL = powerMgmtConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowPowerMgmtConfig
 *
 *   @b Description
 *   @n The function is used to get the Shadow Power Management Configuration.
 *
 *   @b Arguments
     @verbatim
         ptrPwrMgmtConfig       Power Management Configuration populated by this API
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
 *   @n EMIF4F_PWR_MGMT_CTRL_SHDW_REG_PD_TIM_SHDW, 
 *      EMIF4F_PWR_MGMT_CTRL_SHDW_REG_SR_TIM_SHDW,
 *      EMIF4F_PWR_MGMT_CTRL_SHDW_REG_CS_TIM_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_PWR_MGMT_CONFIG pwrConfig;

        // Get the shadow power management configuration.
        CSL_EMIF4F_GetShadowPowerMgmtConfig(&pwrConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetShadowPowerMgmtConfig(EMIF4F_PWR_MGMT_CONFIG* ptrPwrMgmtConfig)
{
    Uint32 powerMgmtConfig = hEmif->PWR_MGMT_CTRL_SHDW;

    /* Extract all the fields from the Power Management Register */
    ptrPwrMgmtConfig->pdTime = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_SHDW_REG_PD_TIM_SHDW);
    ptrPwrMgmtConfig->srTime = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_SHDW_REG_SR_TIM_SHDW);
    ptrPwrMgmtConfig->csTime = CSL_FEXT(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_SHDW_REG_CS_TIM_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowPowerMgmtConfig
 *
 *   @b Description
 *   @n The function is used to set the Shadow Power Management Configuration.
 *
 *   @b Arguments
     @verbatim
         ptrPwrMgmtConfig       Power Management Configuration to be configured
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
 *   @n EMIF4F_PWR_MGMT_CTRL_SHDW_REG_PD_TIM_SHDW, 
 *      EMIF4F_PWR_MGMT_CTRL_SHDW_REG_SR_TIM_SHDW,
 *      EMIF4F_PWR_MGMT_CTRL_SHDW_REG_CS_TIM_SHDW
 *
 *   @b Example
 *   @verbatim
        EMIF4F_PWR_MGMT_CONFIG pwrConfig;

        // Get the power management configuration.
        CSL_EMIF4F_GetShadowPowerMgmtConfig(&pwrConfig);

        // Set the power management timer for clock stop to immediately enter clock stop
        pwrConfig.csTime = 0;

        // Set the power management configuration
        CSL_EMIF4F_SetShadowPowerMgmtConfig(&pwrConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowPowerMgmtConfig(EMIF4F_PWR_MGMT_CONFIG* ptrPwrMgmtConfig)
{
    Uint32 powerMgmtConfig = 0;

    /* Initialzie all the fields for the Power Management Register */
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_SHDW_REG_PD_TIM_SHDW,  ptrPwrMgmtConfig->pdTime);
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_SHDW_REG_SR_TIM_SHDW,  ptrPwrMgmtConfig->srTime);
    CSL_FINS(powerMgmtConfig, EMIF4F_PWR_MGMT_CTRL_SHDW_REG_CS_TIM_SHDW,  ptrPwrMgmtConfig->csTime);

    /* Initialize the register */
    hEmif->PWR_MGMT_CTRL_SHDW = powerMgmtConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLPDDR2Mode
 *
 *   @b Description
 *   @n The function is used to get the LPDDR2 Mode
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n LPDDR2 Mode 
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_LPDDR2_MODE_REG_DATA_REG_VALUE_0 
 *
 *   @b Example
 *   @verbatim
        Uint8 mode;

        // Get the LPDDR2 Mode
        mode = CSL_EMIF4F_GetLPDDR2Mode();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_EMIF4F_GetLPDDR2Mode(void)
{
    return CSL_FEXT(hEmif->LPDDR2_MODE_REG_DATA, EMIF4F_LPDDR2_MODE_REG_DATA_REG_VALUE_0);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLPDDR2Mode
 *
 *   @b Description
 *   @n The function is used to get the LPDDR2 Mode
 *
 *   @b Arguments
     @verbatim
         mode   LPDDR2 Mode to be configured
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
 *   @n EMIF4F_LPDDR2_MODE_REG_DATA_REG_VALUE_0 
 *
 *   @b Example
 *   @verbatim

        // Set the LPDDR2 Mode
        CSL_EMIF4F_SetLPDDR2Mode(1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLPDDR2Mode(Uint8 mode)
{
    CSL_FINS(hEmif->LPDDR2_MODE_REG_DATA, EMIF4F_LPDDR2_MODE_REG_DATA_REG_VALUE_0, mode);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLPDDR2ConfigCS
 *
 *   @b Description
 *   @n The function is used to get the LPDDR2 Configuration Chip Select
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n LPDDR2 Configuration Chip Select 
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_LPDDR2_MODE_REG_CFG_REG_CS 
 *
 *   @b Example
 *   @verbatim
        Uint8 cs;

        // Get the LPDDR2 Chip Select
        cs = CSL_EMIF4F_GetLPDDR2ConfigCS();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_EMIF4F_GetLPDDR2ConfigCS(void)
{
    return CSL_FEXT(hEmif->LPDDR2_MODE_REG_CFG, EMIF4F_LPDDR2_MODE_REG_CFG_REG_CS);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLPDDR2ConfigCS
 *
 *   @b Description
 *   @n The function is used to set the LPDDR2 Configuration Chip Select
 *
 *   @b Arguments
     @verbatim
         chipSelect     LPDDR2 Configuration Chip Select 
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
 *   @n EMIF4F_LPDDR2_MODE_REG_CFG_REG_CS 
 *
 *   @b Example
 *   @verbatim

        // Set the LPDDR2 Chip Select to use CS0
        CSL_EMIF4F_SetLPDDR2ConfigCS(0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLPDDR2ConfigCS(Uint8 chipSelect)
{
    CSL_FINS(hEmif->LPDDR2_MODE_REG_CFG, EMIF4F_LPDDR2_MODE_REG_CFG_REG_CS, chipSelect);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLPDDR2ConfigRefreshEnable
 *
 *   @b Description
 *   @n The function is used to get the LPDDR2 Configuration Refresh Enable
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n LPDDR2 Configuration Refresh Enable
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_LPDDR2_MODE_REG_CFG_REG_REFRESH_EN 
 *
 *   @b Example
 *   @verbatim
        Uint8 refreshEnable;

        // Get the LPDDR2 Config Refresh Enable
        refreshEnable = CSL_EMIF4F_GetLPDDR2ConfigRefreshEnable();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_EMIF4F_GetLPDDR2ConfigRefreshEnable(void)
{
    return CSL_FEXT(hEmif->LPDDR2_MODE_REG_CFG, EMIF4F_LPDDR2_MODE_REG_CFG_REG_REFRESH_EN);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLPDDR2ConfigRefreshEnable
 *
 *   @b Description
 *   @n The function is used to set the LPDDR2 Configuration Refresh Enable
 *
 *   @b Arguments
     @verbatim
         refreshEnable  LPDDR2 Configuration Refresh Enable State
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
 *   @n EMIF4F_LPDDR2_MODE_REG_CFG_REG_REFRESH_EN 
 *
 *   @b Example
 *   @verbatim

        // Enable refresh enable after MRW
        CSL_EMIF4F_SetLPDDR2ConfigRefreshEnable(1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLPDDR2ConfigRefreshEnable(Uint8 refreshEnable)
{
    CSL_FINS(hEmif->LPDDR2_MODE_REG_CFG, EMIF4F_LPDDR2_MODE_REG_CFG_REG_REFRESH_EN, refreshEnable);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLPDDR2ConfigModeRegAddress
 *
 *   @b Description
 *   @n The function is used to get the LPDDR2 Configuration Mode Register Address
 *
 *   @b Arguments
     @verbatim
         None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n LPDDR2 Configuration Mode Register Address
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_LPDDR2_MODE_REG_CFG_REG_ADDRESS 
 *
 *   @b Example
 *   @verbatim
        Uint8 modeRegAddress;

        // Get the LPDDR2 Config Mode Register Address
        modeRegAddress = CSL_EMIF4F_GetLPDDR2ConfigModeRegAddress();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint8 CSL_EMIF4F_GetLPDDR2ConfigModeRegAddress(void)
{
    return CSL_FEXT(hEmif->LPDDR2_MODE_REG_CFG, EMIF4F_LPDDR2_MODE_REG_CFG_REG_ADDRESS);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLPDDR2ConfigModeRegAddress
 *
 *   @b Description
 *   @n The function is used to set the LPDDR2 Configuration Mode Register Address
 *
 *   @b Arguments
     @verbatim
         modeRegAddr    LPDDR2 Configuration Mode Register Address
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
 *   @n EMIF4F_LPDDR2_MODE_REG_CFG_REG_ADDRESS 
 *
 *   @b Example
 *   @verbatim

        // Set the LPDDR2 Config Mode Register Address
        CSL_EMIF4F_SetLPDDR2ConfigModeRegAddress(0x10);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLPDDR2ConfigModeRegAddress(Uint8 modeRegAddr)
{
    CSL_FINS(hEmif->LPDDR2_MODE_REG_CFG, EMIF4F_LPDDR2_MODE_REG_CFG_REG_ADDRESS, modeRegAddr);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetVBUSConfig
 *
 *   @b Description
 *   @n The function is used to get the VBUSM Configuration
 *
 *   @b Arguments
     @verbatim
        cosCount1   Priority Raise Counter for Class of Service 1
        cosCount2   Priority Raise Counter for Class of Service 2
        prOldCount  Priority Raise Old Counter
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Priority Raise Old Counter
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_1, EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_2,
 *   @n EMIF4F_VBUSM_CONFIG_REG_PR_OLD_COUNT
 *
 *   @b Example
 *   @verbatim
        Uint8 cosCount1;
        Uint8 cosCount2;
        Uint8 prOldCount;

        // Get the VBUS Configuration
        CSL_EMIF4F_GetVBUSConfig(&cosCount1, &cosCount2, &prOldCount);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetVBUSConfig(Uint8* cosCount1, Uint8* cosCount2, Uint8* prOldCount)
{
    *cosCount1  = CSL_FEXT(hEmif->VBUSM_CONFIG, EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_1);
    *cosCount2  = CSL_FEXT(hEmif->VBUSM_CONFIG, EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_2);
    *prOldCount = CSL_FEXT(hEmif->VBUSM_CONFIG, EMIF4F_VBUSM_CONFIG_REG_PR_OLD_COUNT);
    return;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetVBUSConfig
 *
 *   @b Description
 *   @n The function is used to set the VBUSM Configuration
 *
 *   @b Arguments
     @verbatim
        cosCount1   Priority Raise Counter for Class of Service 1
        cosCount2   Priority Raise Counter for Class of Service 2
        prOldCount  Priority Raise Old Counter
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
 *   @n EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_1, EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_2,
 *   @n EMIF4F_VBUSM_CONFIG_REG_PR_OLD_COUNT
 *
 *   @b Example
 *   @verbatim
        Uint8 cosCount1;
        Uint8 cosCount2;
        Uint8 prOldCount;

        // Get the VBUS Configuration
        CSL_EMIF4F_GetVBUSConfig(&cosCount1, &cosCount2, &prOldCount);

        // Set the VBUS Configuration
        CSL_EMIF4F_SetVBUSConfig(cosCount1, cosCount2, 0x1);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetVBUSConfig(Uint8 cosCount1, Uint8 cosCount2, Uint8 prOldCount)
{
    hEmif->VBUSM_CONFIG = CSL_FMK (EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_1, cosCount1) |
                          CSL_FMK (EMIF4F_VBUSM_CONFIG_REG_COS_COUNT_2, cosCount2) |
                          CSL_FMK (EMIF4F_VBUSM_CONFIG_REG_PR_OLD_COUNT, prOldCount);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetVBUSConfigValues
 *
 *   @b Description
 *   @n The function is used to get the VBUSM Configuration values
 *
 *   @b Arguments
     @verbatim
        ptrVBUSConfigValue      VBUS Configuration populated by this API
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
 *   @n EMIF4F_VBUSM_CFG_VAL_1_REG_SYS_BUS_WIDTH, EMIF4F_VBUSM_CFG_VAL_1_REG_STAT_FIFO_DEPTH,
 *      EMIF4F_VBUSM_CFG_VAL_1_REG_WR_FIFO_DEPTH, EMIF4F_VBUSM_CFG_VAL_1_REG_CMD_FIFO_DEPTH,
 *   @n EMIF4F_VBUSM_CFG_VAL_2_REG_RREG_FIFO_DEPTH, EMIF4F_VBUSM_CFG_VAL_2_REG_RSD_FIFO_DEPTH,
 *      EMIF4F_VBUSM_CFG_VAL_2_REG_RCMD_FIFO_DEPTH
 *
 *   @b Example
 *   @verbatim
        EMIF4F_VBUS_CONFIG_VALUE     VBUSconfigValue;

        // Get the VBUS Configuration values
        CSL_EMIF4F_GetVBUSConfigValues(&VBUSconfigValue);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetVBUSConfigValues(EMIF4F_VBUS_CONFIG_VALUE* ptrVBUSConfigValue)
{
    Uint32  vbusConfigValue = hEmif->VBUSM_CFG_VAL_1;

    /* Extract all the VBUS Configuration Values */
    ptrVBUSConfigValue->sysBusWidth  = CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_1_REG_SYS_BUS_WIDTH);
    ptrVBUSConfigValue->statFIFODepth= CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_1_REG_STAT_FIFO_DEPTH);
    ptrVBUSConfigValue->wrFIFODepth  = CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_1_REG_WR_FIFO_DEPTH);
    ptrVBUSConfigValue->cmdFIFODepth = CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_1_REG_CMD_FIFO_DEPTH);

    /* Get the configuration from the second register */
    vbusConfigValue = hEmif->VBUSM_CFG_VAL_2;

    /* Extract all the VBUS Configuration Values */
    ptrVBUSConfigValue->rregFIFODepth = CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_2_REG_RREG_FIFO_DEPTH);
    ptrVBUSConfigValue->rsdFIFODepth  = CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_2_REG_RSD_FIFO_DEPTH);
    ptrVBUSConfigValue->rcmdFIFODepth = CSL_FEXT(vbusConfigValue, EMIF4F_VBUSM_CFG_VAL_2_REG_RCMD_FIFO_DEPTH);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetIODFTControl
 *
 *   @b Description
 *   @n The function is used to get the IODFT Control configuration
 *
 *   @b Arguments
     @verbatim
        ptrIODFTControl      IODFT Control Configuration populated by this API
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
 *   @n EMIF4F_IODFT_TLGC_REG_TLEC,      EMIF4F_IODFT_TLGC_REG_MT,
 *      EMIF4F_IODFT_TLGC_REG_ACT_CAP_EN,EMIF4F_IODFT_TLGC_REG_OPG_LD,
 *      EMIF4F_IODFT_TLGC_REG_MMS,       EMIF4F_IODFT_TLGC_REG_MC,        
 *      EMIF4F_IODFT_TLGC_REG_PC,        EMIF4F_IODFT_TLGC_REG_TM
 *
 *   @b Example
 *   @verbatim
        EMIF4F_IODFT_CONTROL     IODFTcontrol;

        // Get the IODFT Configuration values
        CSL_EMIF4F_GetIODFTControl(&IODFTcontrol);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetIODFTControl(EMIF4F_IODFT_CONTROL* ptrIODFTControl)
{
    Uint32  iodftControl = hEmif->IODFT_TLGC;

    /* Extract all the IODFT Control Values */
    ptrIODFTControl->tlec    = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_TLEC);
    ptrIODFTControl->mt      = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_MT);
    ptrIODFTControl->actCapEn= CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_ACT_CAP_EN);
    ptrIODFTControl->opgld   = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_OPG_LD);
    ptrIODFTControl->mms     = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_MMS);
    ptrIODFTControl->mc      = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_MC);
    ptrIODFTControl->pc      = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_PC);
    ptrIODFTControl->tm      = CSL_FEXT(iodftControl, EMIF4F_IODFT_TLGC_REG_TM);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetIODFTControl
 *
 *   @b Description
 *   @n The function is used to set the IODFT Control configuration
 *
 *   @b Arguments
     @verbatim
        ptrIODFTControl      IODFT Control Configuration to be configured
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
 *   @n EMIF4F_IODFT_TLGC_REG_TLEC,      EMIF4F_IODFT_TLGC_REG_MT,
 *      EMIF4F_IODFT_TLGC_REG_ACT_CAP_EN,EMIF4F_IODFT_TLGC_REG_OPG_LD,
 *      EMIF4F_IODFT_TLGC_REG_MMS,       EMIF4F_IODFT_TLGC_REG_MC,        
 *      EMIF4F_IODFT_TLGC_REG_PC,        EMIF4F_IODFT_TLGC_REG_TM
 *
 *   @b Example
 *   @verbatim
        EMIF4F_IODFT_CONTROL     IODFTcontrol;

        // Get the IODFT Configuration values
        CSL_EMIF4F_GetIODFTControl(&IODFTcontrol);

        // Set the new IODFT configuration 
        CSL_EMIF4F_SetIODFTControl (&IODFTcontrol);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetIODFTControl(EMIF4F_IODFT_CONTROL* ptrIODFTControl)
{
    Uint32  iodftControl = 0;

    /* Initialize all the IODFT Control Values */
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_TLEC,      ptrIODFTControl->tlec);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_MT,        ptrIODFTControl->mt);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_ACT_CAP_EN,ptrIODFTControl->actCapEn);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_OPG_LD,    ptrIODFTControl->opgld);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_MMS,       ptrIODFTControl->mms);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_MC,        ptrIODFTControl->mc);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_PC,        ptrIODFTControl->pc);
    CSL_FINS(iodftControl, EMIF4F_IODFT_TLGC_REG_TM,        ptrIODFTControl->tm);

    /* Initialize the register */
    hEmif->IODFT_TLGC = iodftControl;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetIODFTControlResults
 *
 *   @b Description
 *   @n The function is used to get the IODFT Control results.
 *
 *   @b Arguments
     @verbatim
        dqmTLMRResults      DQM TLMR Results populated by this API
        ctlTLMRResults      CTL TLMR Results populated by this API
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
 *   @n EMIF4F_IODFT_CTRL_MISR_RSLT_REG_DQM_TLMR, EMIF4F_IODFT_CTRL_MISR_RSLT_REG_CTL_TLMR
 *
 *   @b Example
 *   @verbatim
        Uint16     dqmTLMRResults;
        Uint16     ctlTLMRResults;

        // Get the IODFT Control Results
        CSL_EMIF4F_GetIODFTControlResults(&dqmTLMRResults, &ctlTLMRResults);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetIODFTControlResults(Uint16* dqmTLMRResults, Uint16* ctlTLMRResults)
{
    *dqmTLMRResults = CSL_FEXT (hEmif->IODFT_CTRL_MISR_RSLT, EMIF4F_IODFT_CTRL_MISR_RSLT_REG_DQM_TLMR);
    *ctlTLMRResults = CSL_FEXT (hEmif->IODFT_CTRL_MISR_RSLT, EMIF4F_IODFT_CTRL_MISR_RSLT_REG_CTL_TLMR);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetIODFTAddrResults
 *
 *   @b Description
 *   @n The function is used to get the IODFT Address results.
 *
 *   @b Arguments
     @verbatim
        addrTLMRResults      Addr TLMR Results populated by this API
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
 *   @n EMIF4F_IODFT_ADDR_MISR_RSLT_REG_ADDR_TLMR
 *
 *   @b Example
 *   @verbatim
        Uint32     addrTLMRResults;

        // Get the IODFT Address results
        CSL_EMIF4F_GetIODFTAddrResults(&addrTLMRResults);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetIODFTAddrResults(Uint32* addrTLMRResults)
{
    *addrTLMRResults = CSL_FEXT (hEmif->IODFT_ADDR_MISR_RSLT, EMIF4F_IODFT_ADDR_MISR_RSLT_REG_ADDR_TLMR);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetIODFTDataResults
 *
 *   @b Description
 *   @n The function is used to get the IODFT Data results.
 *
 *   @b Arguments
     @verbatim
        dataTLMRResults1      Data TLMR1 Results populated by this API
        dataTLMRResults2      Data TLMR2 Results populated by this API
        dataTLMRResults3      Data TLMR3 Results populated by this API
        dataTLMRResults4      Data TLMR4 Results populated by this API
        dataTLMRResults5      Data TLMR5 Results populated by this API
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
 *   @n EMIF4F_IODFT_DATA_MISR_RSLT_1,EMIF4F_IODFT_DATA_MISR_RSLT_2,
 *      EMIF4F_IODFT_DATA_MISR_RSLT_3,EMIF4F_IODFT_DATA_MISR_RSLT_4,
 *      EMIF4F_IODFT_DATA_MISR_RSLT_5
 *
 *   @b Example
 *   @verbatim
        Uint32     dataTLMRResults1;
        Uint32     dataTLMRResults2;
        Uint32     dataTLMRResults3;
        Uint32     dataTLMRResults4;
        Uint32     dataTLMRResults5;

        // Get the IODFT Data results
        CSL_EMIF4F_GetIODFTDataResults(&dataTLMRResults1, &dataTLMRResults2, 
                                      &dataTLMRResults3, &dataTLMRResults4, 
                                      &dataTLMRResults5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetIODFTDataResults
(
    Uint32* dataTLMRResults1,
    Uint32* dataTLMRResults2,
    Uint32* dataTLMRResults3,
    Uint32* dataTLMRResults4,
    Uint32* dataTLMRResults5
)
{
    *dataTLMRResults1 = hEmif->IODFT_DATA_MISR_RSLT_1;
    *dataTLMRResults2 = hEmif->IODFT_DATA_MISR_RSLT_2;
    *dataTLMRResults3 = hEmif->IODFT_DATA_MISR_RSLT_3;
    *dataTLMRResults4 = hEmif->IODFT_DATA_MISR_RSLT_4;
    *dataTLMRResults5 = hEmif->IODFT_DATA_MISR_RSLT_5;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPerfCounters
 *
 *   @b Description
 *   @n The function is used to get the performance counters
 *
 *   @b Arguments
     @verbatim
        perfCounter1        Performance Counter1 populated by this API
        perfCounter2        Performance Counter2 populated by this API
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
 *   @n EMIF4F_PERF_CNT_1,EMIF4F_PERF_CNT_2
 *
 *   @b Example
 *   @verbatim
        Uint32     perfCounter1;
        Uint32     perfCounter2;

        // Get the EMIF4F Performance counters
        CSL_EMIF4F_GetPerfCounters(&perfCounter1, &perfCounter2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPerfCounters
(
    Uint32* perfCounter1,
    Uint32* perfCounter2
)
{
    *perfCounter1 = hEmif->PERF_CNT_1;
    *perfCounter2 = hEmif->PERF_CNT_2;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPerfCounterConfig
 *
 *   @b Description
 *   @n The function is used to get the configuration for performance counters
 *
 *   @b Arguments
     @verbatim
        counter                 Perf Counter for which configuration is required (1 or 2)
        ptrPerfCounterConfig    Performance Counter Configuration populated by this API
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
 *   @n EMIF4F_PERF_CNT_CFG_REG_CNTR2_MSTID_EN, EMIF4F_PERF_CNT_CFG_REG_CNTR2_REGION_EN,
 *      EMIF4F_PERF_CNT_CFG_REG_CNTR2_CFG;
 *      EMIF4F_PERF_CNT_CFG_REG_CNTR1_MSTID_EN,
 *      EMIF4F_PERF_CNT_CFG_REG_CNTR1_REGION_EN, EMIF4F_PERF_CNT_CFG_REG_CNTR1_CFG
 *
 *   @b Example
 *   @verbatim
        EMIF4F_PERF_CONFIG     perfCounterConfig;

        // Get the EMIF4F Performance Counter Configuration for counter 1
        CSL_EMIF4F_GetPerfCounterConfig(1, &perfCounterConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPerfCounterConfig
(
    Uint8              counter,
    EMIF4F_PERF_CONFIG* ptrPerfCounterConfig
)
{
    Uint32  perfConfig = hEmif->PERF_CNT_CFG;

    if (counter == 2)
    {
        ptrPerfCounterConfig->mstIDEn = CSL_FEXT(perfConfig, EMIF4F_PERF_CNT_CFG_REG_CNTR2_MSTID_EN);
        ptrPerfCounterConfig->regEn   = CSL_FEXT(perfConfig, EMIF4F_PERF_CNT_CFG_REG_CNTR2_REGION_EN);
        ptrPerfCounterConfig->cntCfg  = CSL_FEXT(perfConfig, EMIF4F_PERF_CNT_CFG_REG_CNTR2_CFG);
    } 
    else if (counter == 1)
    {
        ptrPerfCounterConfig->mstIDEn = CSL_FEXT(perfConfig, EMIF4F_PERF_CNT_CFG_REG_CNTR1_MSTID_EN);
        ptrPerfCounterConfig->regEn   = CSL_FEXT(perfConfig, EMIF4F_PERF_CNT_CFG_REG_CNTR1_REGION_EN);
        ptrPerfCounterConfig->cntCfg  = CSL_FEXT(perfConfig, EMIF4F_PERF_CNT_CFG_REG_CNTR1_CFG);
    }
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPerfCounterConfig
 *
 *   @b Description
 *   @n The function is used to set the configuration for performance counters
 *
 *   @b Arguments
     @verbatim
        counter                 Perf Counter for which configuration is to be set (1 or 2)
        ptrPerfCounterConfig    Performance Counter Configuration to be configured
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
 *   @n EMIF4F_PERF_CNT_CFG_REG_CNTR2_MCONNID_EN, EMIF4F_PERF_CNT_CFG_REG_CNTR2_REGION_EN,
 *      EMIF4F_PERF_CNT_CFG_REG_CNTR2_CFG;
 *      EMIF4F_PERF_CNT_CFG_REG_CNTR1_MCONNID_EN,
 *      EMIF4F_PERF_CNT_CFG_REG_CNTR1_REGION_EN,  EMIF4F_PERF_CNT_CFG_REG_CNTR1_CFG
 *
 *   @b Example
 *   @verbatim
        EMIF4F_PERF_CONFIG     perfCounterConfig;

        // Get the EMIF4F Performance Counter Configuration for counter 2
        CSL_EMIF4F_GetPerfCounterConfig(2, &perfCounterConfig);

        // Enable Master ID Filter Enable 
        perfCounterConfig.mstIDEn = 1;
        
        // Set the EMIF4F Performance Counter Configuration.
        CSL_EMIF4F_SetPerfCounterConfig(2, &perfCounterConfig);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPerfCounterConfig
(
    Uint8               counter, 
    EMIF4F_PERF_CONFIG*  ptrPerfCounterConfig
)
{
    /* Configure the performance counter configuration.*/
    if (counter == 1)
    {
        CSL_FINS(hEmif->PERF_CNT_CFG, EMIF4F_PERF_CNT_CFG_REG_CNTR1_MSTID_EN,  ptrPerfCounterConfig->mstIDEn);
        CSL_FINS(hEmif->PERF_CNT_CFG, EMIF4F_PERF_CNT_CFG_REG_CNTR1_REGION_EN, ptrPerfCounterConfig->regEn);
        CSL_FINS(hEmif->PERF_CNT_CFG, EMIF4F_PERF_CNT_CFG_REG_CNTR1_CFG,       ptrPerfCounterConfig->cntCfg);
    }
    else if (counter == 2)
    {
        CSL_FINS(hEmif->PERF_CNT_CFG, EMIF4F_PERF_CNT_CFG_REG_CNTR2_MSTID_EN,  ptrPerfCounterConfig->mstIDEn);
        CSL_FINS(hEmif->PERF_CNT_CFG, EMIF4F_PERF_CNT_CFG_REG_CNTR2_REGION_EN, ptrPerfCounterConfig->regEn);
        CSL_FINS(hEmif->PERF_CNT_CFG, EMIF4F_PERF_CNT_CFG_REG_CNTR2_CFG,       ptrPerfCounterConfig->cntCfg);
    }
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPerfCounterMasterRegion
 *
 *   @b Description
 *   @n The function is used to get the master region selection for the specific
 *      performance counters
 *
 *   @b Arguments
     @verbatim
        counter          Perf Counter for which master region selection is configured
        mstID            Master ID selection for the specified performance counter
        regionSel        Region Selection for the specified performance counter
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
 *   @n EMIF4F_PERF_CNT_SEL_REG_MCONNID1, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL1;
 *      EMIF4F_PERF_CNT_SEL_REG_MCONNID2, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL2
 *
 *   @b Example
 *   @verbatim
        Uint8   mstID;
        Uint8   regionSel;

        // Get the Master Region Selection for Counter 1
        CSL_EMIF4F_GetPerfCounterMasterRegion (1, &mstID, &regionSel);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPerfCounterMasterRegion
(
    Uint8   counter,
    Uint8*  mstID,
    Uint8*  regionSel
)
{
    if (counter == 1)
    {
        *mstID     = CSL_FEXT (hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_MSTID1);
        *regionSel = CSL_FEXT (hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL1);
    }
    else if (counter == 2)
    {
        *mstID     = CSL_FEXT (hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_MSTID2);
        *regionSel = CSL_FEXT (hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL2);
    }
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPerfCounterMasterRegion
 *
 *   @b Description
 *   @n The function is used to set the master region selection for the specific
 *      performance counters
 *
 *   @b Arguments
     @verbatim
        counter          Perf Counter for which master region selection is configured
        mstID            Master ID selection for the specified performance counter
        regionSel        Region Selection for the specified performance counter
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
 *   @n EMIF4F_PERF_CNT_SEL_REG_MCONNID1, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL1;
 *      EMIF4F_PERF_CNT_SEL_REG_MCONNID2, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL2
 *
 *   @b Example
 *   @verbatim
        Uint8   mstID;
        Uint8   regionSel;

        // Get the Master Region Selection for Counter 1
        CSL_EMIF4F_GetPerfCounterMasterRegion (1, &mstID, &regionSel);

        // Change Master ID to be 5
        CSL_EMIF4F_SetPerfCounterMasterRegion(1, 5, regionSel);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPerfCounterMasterRegion
(
    Uint8   counter,
    Uint8   mstID,
    Uint8   regionSel
)
{
    if (counter == 1)
    {
        CSL_FINS(hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_MSTID1,      mstID);
        CSL_FINS(hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL1, regionSel);
    }
    else if (counter == 2)
    {
        CSL_FINS(hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_MSTID2,      mstID);
        CSL_FINS(hEmif->PERF_CNT_SEL, EMIF4F_PERF_CNT_SEL_REG_REGION_SEL2, regionSel);
    }
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPerfCounterTime
 *
 *   @b Description
 *   @n The function is used to get the performance counter timer register
 *
 *   @b Arguments
     @verbatim
        None
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n Performance counter Time Register
 *
 *   <b> Pre Condition </b>
 *   @n None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n EMIF4F_PERF_CNT_TIM
 *
 *   @b Example
 *   @verbatim
        Uint32  perfCount;

        // Get the Performance Counter Time Register
        perfCount = CSL_EMIF4F_GetPerfCounterTime ();
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CSL_EMIF4F_GetPerfCounterTime(void)
{
    return hEmif->PERF_CNT_TIM;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetIdleControl
 *
 *   @b Description
 *   @n The function is used to get the idle control information
 *
 *   @b Arguments
     @verbatim
        idleLen         Idle Length populated by this API
        idleInterval    Idle Interval populated by this API
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
 *   @n EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_LEN, EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_INTERVAL
 *
 *   @b Example
 *   @verbatim
        Uint8   idleLen;
        Uint16  idleInterval;

        // Get the Idle Control Configuration
        CSL_EMIF4F_GetIdleControl (&idleLen, &idleInterval);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetIdleControl(Uint8* idleLen, Uint16* idleInterval)
{
    *idleLen      = CSL_FEXT(hEmif->READ_IDLE_CTRL, EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_LEN);
    *idleInterval = CSL_FEXT(hEmif->READ_IDLE_CTRL, EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_INTERVAL);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetIdleControl
 *
 *   @b Description
 *   @n The function is used to set the idle control information
 *
 *   @b Arguments
     @verbatim
        idleLen         Idle Length to be configured
        idleInterval    Idle Interval to be configured
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
 *   @n EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_LEN, 
 *      EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_INTERVAL
 *
 *   @b Example
 *   @verbatim
        ...
        // Set the Idle Control Configuration
        CSL_EMIF4F_SetIdleControl (5, 10);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetIdleControl(Uint8 idleLen, Uint16 idleInterval)
{
    CSL_FINS(hEmif->READ_IDLE_CTRL, EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_LEN, idleLen);
    CSL_FINS(hEmif->READ_IDLE_CTRL, EMIF4F_READ_IDLE_CTRL_REG_READ_IDLE_INTERVAL, idleInterval);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetIdleControlShadow
 *
 *   @b Description
 *   @n The function is used to get the idle control information from the Shadow
 *      register.
 *
 *   @b Arguments
     @verbatim
        idleLen         Idle Length populated by this API
        idleInterval    Idle Interval populated by this API
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
 *   @n EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_LEN_SHDW, 
 *      EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_INTERVAL_SHDW
 *
 *   @b Example
 *   @verbatim
        // Get the Idle Control Configuration from the Shadow  register
        CSL_EMIF4F_GetIdleControlShadow (&idleLen, &idleInterval);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetIdleControlShadow(Uint8* idleLen, Uint16* idleInterval)
{
    *idleLen      = CSL_FEXT(hEmif->READ_IDLE_CTRL_SHDW, EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_LEN_SHDW);
    *idleInterval = CSL_FEXT(hEmif->READ_IDLE_CTRL_SHDW, EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_INTERVAL_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetIdleControlShadow
 *
 *   @b Description
 *   @n The function is used to set the idle control information in the Shadow
 *      register
 *
 *   @b Arguments
     @verbatim
        idleLen         Idle Length to be configured
        idleInterval    Idle Interval to be configured
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
 *   @n EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_LEN_SHDW, 
 *      EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_INTERVAL_SHDW
 *
 *   @b Example
 *   @verbatim
        ...
        // Set the Idle Control Configuration in the Shadow register
        CSL_EMIF4F_SetIdleControlShadow (5, 10);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetIdleControlShadow(Uint8 idleLen, Uint16 idleInterval)
{
    CSL_FINS(hEmif->READ_IDLE_CTRL_SHDW, EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_LEN_SHDW, idleLen);
    CSL_FINS(hEmif->READ_IDLE_CTRL_SHDW, EMIF4F_READ_IDLE_CTRL_SHDW_REG_READ_IDLE_INTERVAL_SHDW, idleInterval);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetInterruptRawStatus
 *
 *   @b Description
 *   @n The function is used to get the interrupt RAW status 
 *
 *   @b Arguments
     @verbatim
        rawStatus       Raw Interrupt Status populated by this API
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
 *     EMIF4F_IRQSTATUS_RAW_SYS
 *
 *   @b Example
 *   @verbatim
        Uint8   rawStatus;

        // Get the Interrupt RAW Status
        CSL_EMIF4F_GetInterruptRawStatus (&rawStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetInterruptRawStatus(Uint8* rawStatus)
{
    *rawStatus = hEmif->IRQSTATUS_RAW_SYS;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetInterruptRawStatus
 *
 *   @b Description
 *   @n The function is used to set the interrupt RAW status 
 *
 *   @b Arguments
     @verbatim
        rawStatus       Raw Interrupt Status to be configured
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
 *     EMIF4F_IRQSTATUS_RAW_SYS
 *
 *   @b Example
 *   @verbatim
        
        // Set the Interrupt RAW Status for LPDDR2 NVM data not valid
        CSL_EMIF4F_SetInterruptRawStatus (0x4);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetInterruptRawStatus(Uint8 rawStatus)
{
    hEmif->IRQSTATUS_RAW_SYS = rawStatus;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetInterruptStatus
 *
 *   @b Description
 *   @n The function is used to get the interrupt status 
 *
 *   @b Arguments
     @verbatim
        intrStatus       Interrupt Status populated by this API
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
 *     EMIF4F_IRQSTATUS_SYS
 *
 *   @b Example
 *   @verbatim
        Uint8   intrStatus;

        // Get the Interrupt Status
        CSL_EMIF4F_GetInterruptStatus (&intrStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetInterruptStatus(Uint8* intrStatus)
{
    *intrStatus = hEmif->IRQSTATUS_SYS;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_ClearInterruptStatus
 *
 *   @b Description
 *   @n The function is used to clear the interrupt status 
 *
 *   @b Arguments
     @verbatim
        intrStatus       Interrupt Status populated by this API
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
 *     EMIF4F_IRQSTATUS_SYS
 *
 *   @b Example
 *   @verbatim
        Uint8   intrStatus;

        // Get the Interrupt Status
        CSL_EMIF4F_GetInterruptRawStatus (&intrStatus);
        ...
        // Process the interrupts
        ...
        // Clear the Interrupts
        CSL_EMIF4F_ClearInterruptStatus (intrStatus);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_ClearInterruptStatus(Uint8 intrStatus)
{
    hEmif->IRQSTATUS_SYS = intrStatus;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_EnableInterrupts
 *
 *   @b Description
 *   @n The function is used to enable interrupts
 *
 *   @b Arguments
     @verbatim
        intrFlag       Interrupt flag for interrupts to be enabled
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
 *     EMIF4F_IRQENABLE_SET_SYS
 *
 *   @b Example
 *   @verbatim

        // Enable address error interrupt 
        CSL_EMIF4F_EnableInterrupts (0x1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_EnableInterrupts(Uint8 intrStatus)
{
    hEmif->IRQENABLE_SET_SYS = intrStatus;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_DisableInterrupts
 *
 *   @b Description
 *   @n The function is used to disable interrupts
 *
 *   @b Arguments
     @verbatim
        intrFlag       Interrupt flag for interrupts to be disabled
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
 *     EMIF4F_IRQENABLE_CLR_SYS
 *
 *   @b Example
 *   @verbatim

        // Disable address error interrupt 
        CSL_EMIF4F_DisableInterrupts (0x1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_DisableInterrupts(Uint8 intrStatus)
{
    hEmif->IRQENABLE_CLR_SYS = intrStatus;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetOutputImpedanceConfig
 *
 *   @b Description
 *   @n The function is used to get the Output Impedance Calibrartion configuration
 *
 *   @b Arguments
     @verbatim
        ptrOutputImpedanceConfig    Output Impedance Calibrartion configuration 
                                    populated by this API
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
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_CS1EN,      EMIF4F_ZQ_CONFIG_REG_ZQ_CS0EN, 
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_DUALCALEN,  EMIF4F_ZQ_CONFIG_REG_ZQ_SFEXITEN,
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_ZQINIT_MULT,EMIF4F_ZQ_CONFIG_REG_ZQ_ZQCL_MULT,
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_REFINTERVAL
 *
 *   @b Example
 *   @verbatim
        EMIF4F_OUTPUT_IMP_CONFIG outputImpConfig; 

        // Get the output impedance configuration
        CSL_EMIF4F_GetOutputImpedanceConfig (&outputImpConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetOutputImpedanceConfig(EMIF4F_OUTPUT_IMP_CONFIG* ptrOutputImpedanceConfig)
{
    Uint32 zqConfig = hEmif->ZQ_CONFIG;

    /* Extract all the fields and populate the structure */
    ptrOutputImpedanceConfig->zqCS1En       = CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_CS1EN);
    ptrOutputImpedanceConfig->zqCS0En       = CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_CS0EN);
    ptrOutputImpedanceConfig->zqDualCSEn    = CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_DUALCALEN);
    ptrOutputImpedanceConfig->zqSFEXITEn    = CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_SFEXITEN);
    ptrOutputImpedanceConfig->zqZQCLInterval= CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_ZQINIT_MULT);
    ptrOutputImpedanceConfig->zqZQCLMult    = CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_ZQCL_MULT);
    ptrOutputImpedanceConfig->zqRefInterval = CSL_FEXT (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_REFINTERVAL);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetOutputImpedanceConfig
 *
 *   @b Description
 *   @n The function is used to set the Output Impedance Calibrartion configuration
 *
 *   @b Arguments
     @verbatim
        ptrOutputImpedanceConfig    Output Impedance Calibrartion configuration to be
                                    configured
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
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_CS1EN,      EMIF4F_ZQ_CONFIG_REG_ZQ_CS0EN, 
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_DUALCALEN,  EMIF4F_ZQ_CONFIG_REG_ZQ_SFEXITEN,
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_ZQINIT_MULT,EMIF4F_ZQ_CONFIG_REG_ZQ_ZQCL_MULT,
 *     EMIF4F_ZQ_CONFIG_REG_ZQ_REFINTERVAL
 *
 *   @b Example
 *   @verbatim
        EMIF4F_OUTPUT_IMP_CONFIG outputImpConfig; 

        // Get the output impedance configuration
        CSL_EMIF4F_GetOutputImpedanceConfig (&outputImpConfig);

        // Enable ZQ Calibration for CS1
        outputImpConfig.zqCS1En = 1;

        // Set the new output impedance configuration.
        CSL_EMIF4F_SetOutputImpedanceConfig(&outputImpConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetOutputImpedanceConfig(EMIF4F_OUTPUT_IMP_CONFIG* ptrOutputImpedanceConfig)
{
    Uint32 zqConfig = 0;

    /* Initialize the fields */
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_CS1EN, ptrOutputImpedanceConfig->zqCS1En);
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_CS0EN, ptrOutputImpedanceConfig->zqCS0En);
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_DUALCALEN, ptrOutputImpedanceConfig->zqDualCSEn);
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_SFEXITEN, ptrOutputImpedanceConfig->zqSFEXITEn);
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_ZQINIT_MULT, ptrOutputImpedanceConfig->zqZQCLInterval);
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_ZQCL_MULT, ptrOutputImpedanceConfig->zqZQCLMult);
    CSL_FINS (zqConfig, EMIF4F_ZQ_CONFIG_REG_ZQ_REFINTERVAL, ptrOutputImpedanceConfig->zqRefInterval);

    /* Initialize the register */
    hEmif->ZQ_CONFIG = zqConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetTempAlertConfig
 *
 *   @b Description
 *   @n The function is used to get the Temperature Alert configuration
 *
 *   @b Arguments
     @verbatim
        ptrTempAlertConfig    Temperature Alert Configuration populated by this API
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
 *     EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS1EN,    EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS0EN, 
 *     EMIF4F_TEMP_ALERT_CONFIG_REG_TA_SFEXITEN, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVWDT,
 *     EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVCNT,   EMIF4F_TEMP_ALERT_CONFIG_REG_TA_REFINTERVAL
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TEMP_ALERT_CONFIG tempAlertConfig; 

        // Get the temperature alert configuration.
        CSL_EMIF4F_GetTempAlertConfig (&tempAlertConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetTempAlertConfig(EMIF4F_TEMP_ALERT_CONFIG* ptrTempAlertConfig)
{
    Uint32 tempAlertConfig = hEmif->TEMP_ALERT_CONFIG;

    /* Extract all the fields and populate the structure */
    ptrTempAlertConfig->taCS1En       = CSL_FEXT (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS1EN);
    ptrTempAlertConfig->taCS0En       = CSL_FEXT (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS0EN);
    ptrTempAlertConfig->taSFEXITEn    = CSL_FEXT (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_SFEXITEN);
    ptrTempAlertConfig->taDevWdth     = CSL_FEXT (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVWDT);
    ptrTempAlertConfig->taDevCnt      = CSL_FEXT (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVCNT);
    ptrTempAlertConfig->taRefInterval = CSL_FEXT (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_REFINTERVAL);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetTempAlertConfig
 *
 *   @b Description
 *   @n The function is used to set the Temperature Alert configuration
 *
 *   @b Arguments
     @verbatim
        ptrTempAlertConfig    Temperature Alert Configuration to be configured
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
 *     EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS1EN,    EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS0EN, 
 *     EMIF4F_TEMP_ALERT_CONFIG_REG_TA_SFEXITEN, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVWDT,
 *     EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVCNT,   EMIF4F_TEMP_ALERT_CONFIG_REG_TA_REFINTERVAL
 *
 *   @b Example
 *   @verbatim
        EMIF4F_TEMP_ALERT_CONFIG tempAlertConfig; 

        // Get the temperature alert configuration.
        CSL_EMIF4F_GetTempAlertConfig (&tempAlertConfig);

        // Enable Temperature Alert Polling for CS1
        tempAlertConfig.taCS1En = 1;

        // Set the new temperature alert configuration.
        CSL_EMIF4F_SetTempAlertConfig(&tempAlertConfig);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetTempAlertConfig(EMIF4F_TEMP_ALERT_CONFIG* ptrTempAlertConfig)
{
    Uint32 tempAlertConfig = 0;

    /* Extract all the fields and populate the structure */
    CSL_FINS (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS1EN,       ptrTempAlertConfig->taCS1En);
    CSL_FINS (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_CS0EN,       ptrTempAlertConfig->taCS0En);
    CSL_FINS (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_SFEXITEN,    ptrTempAlertConfig->taSFEXITEn);
    CSL_FINS (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVWDT,      ptrTempAlertConfig->taDevWdth);
    CSL_FINS (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_DEVCNT,      ptrTempAlertConfig->taDevCnt);
    CSL_FINS (tempAlertConfig, EMIF4F_TEMP_ALERT_CONFIG_REG_TA_REFINTERVAL, ptrTempAlertConfig->taRefInterval);

    /* Initialize the register */
    hEmif->TEMP_ALERT_CONFIG = tempAlertConfig;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetErrorLog
 *
 *   @b Description
 *   @n The function is used to get the error log
 *
 *   @b Arguments
     @verbatim
        errCRSel    CRSel for the first errored transaction populated by this API
        errAddMode  Addressing Mode populated by this API
        errCmdType  Command Type of the first errored transaction populated by this API
        errMstID    Master ID of the first errored transaction populated by this API
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
 *     EMIF4F_VBUSM_ERR_LOG_REG_CRSEL,EMIF4F_VBUSM_ERR_LOG_REG_CAMODE, 
 *     EMIF4F_VBUSM_ERR_LOG_REG_CDIR, EMIF4F_VBUSM_ERR_LOG_REG_CMSTID
 *
 *   @b Example
 *   @verbatim
        Uint8  errCRSel;
        Uint8  errAddMode;
        Uint8  errCmdType;
        Uint8  errMstID;

        // Get the Error Log 
        CSL_EMIF4F_GetErrorLog (&errCRSel, &errAddMode, &errCmdType, &errMstID);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetErrorLog
(
    Uint8*  errCRSel,
    Uint8*  errAddMode,
    Uint8*  errCmdType,
    Uint8*  errMstID
)
{
    Uint32 errLog = hEmif->VBUSM_ERR_LOG;

    /* Extract all the fields from the error log register */
    *errCRSel   = CSL_FEXT (errLog, EMIF4F_VBUSM_ERR_LOG_REG_CRSEL);
    *errAddMode = CSL_FEXT (errLog, EMIF4F_VBUSM_ERR_LOG_REG_CAMODE);
    *errCmdType = CSL_FEXT (errLog, EMIF4F_VBUSM_ERR_LOG_REG_CDIR);
    *errMstID   = CSL_FEXT (errLog, EMIF4F_VBUSM_ERR_LOG_REG_CMSTID);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLevelingRampWindow
 *
 *   @b Description
 *   @n The function is used to get the incremental leveling ramp window.
 *
 *   @b Arguments
     @verbatim
        lvlRampWindow   - Incremental leveling ramp window in number of 
                          refresh periods
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
 *      EMIF4F_RDWR_LVL_RMP_WIN_REG_RDWRLVLINC_RMP_WIN
 *
 *   @b Example
 *   @verbatim
        Uint16  lvlRampWindow;

        // Get the Leveling Ramp Window
        CSL_EMIF4F_GetLevelingRampWindow (&lvlRampWindow);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetLevelingRampWindow(Uint16* lvlRampWindow)
{
    *lvlRampWindow = CSL_FEXT(hEmif->RDWR_LVL_RMP_WIN, EMIF4F_RDWR_LVL_RMP_WIN_REG_RDWRLVLINC_RMP_WIN);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLevelingRampWindow
 *
 *   @b Description
 *   @n The function is used to set the incremental leveling ramp window.
 *
 *   @b Arguments
     @verbatim
        lvlRampWindow   - Incremental leveling ramp window in number of 
                          refresh periods
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
 *      EMIF4F_RDWR_LVL_RMP_WIN_REG_RDWRLVLINC_RMP_WIN
 *
 *   @b Example
 *   @verbatim
        
        // Set the Leveling Ramp Window
        CSL_EMIF4F_SetLevelingRampWindow (1000);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLevelingRampWindow(Uint16 lvlRampWindow)
{
    CSL_FINS(hEmif->RDWR_LVL_RMP_WIN, EMIF4F_RDWR_LVL_RMP_WIN_REG_RDWRLVLINC_RMP_WIN, lvlRampWindow);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLevelingRampControlInfo
 *
 *   @b Description
 *   @n The function is used to get the incremental leveling ramp control information
 *
 *   @b Arguments
     @verbatim
        preScalar   - Incremental leveling pre-scalar in number of refresh periods
        readDataEye - Incremental read data eye training interval during ramp window
        readDQS     - Incremental read DQS gate training interval during ramp window
        writeInt    - Incremental write leveling interval during ramp window
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
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVL_EN,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVLINC_RMP_PRE,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLINC_RMP_INT,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLGATEINC_RMP_INT,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_WRLVLINC_RMP_INT
 *
 *   @b Example
 *   @verbatim
        Uint8  preScalar;
        Uint8  readDataEye;
        Uint8  readDQS;
        Uint8  writeInt;
        Bool   enable;

        // Get the Leveling Ramp Control Information
        CSL_EMIF4F_GetLevelingRampControlInfo (&enable, &preScalar, &readDataEye, &readDQS, &writeInt);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetLevelingRampControlInfo
(
    Bool*   enable,
    Uint8*  preScalar,
    Uint8*  readDataEye,
    Uint8*  readDQS,
    Uint8*  writeInt
)
{
    Uint32  value = hEmif->RDWR_LVL_RMP_CTRL;

    *enable      = CSL_FEXT(value, EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVL_EN);
    *preScalar   = CSL_FEXT(value, EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVLINC_RMP_PRE);
    *readDataEye = CSL_FEXT(value, EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLINC_RMP_INT);
    *readDQS     = CSL_FEXT(value, EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLGATEINC_RMP_INT);
    *writeInt    = CSL_FEXT(value, EMIF4F_RDWR_LVL_RMP_CTRL_REG_WRLVLINC_RMP_INT);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLevelingRampControlInfo
 *
 *   @b Description
 *   @n The function is used to set the incremental leveling ramp control information
 *
 *   @b Arguments
     @verbatim
        enable      - Read-write leveling enable.
        preScalar   - Incremental leveling pre-scalar in number of refresh periods
        readDataEye - Incremental read data eye training interval during ramp window
        readDQS     - Incremental read DQS gate training interval during ramp window
        writeInt    - Incremental write leveling interval during ramp window
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
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVL_EN,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVLINC_RMP_PRE,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLINC_RMP_INT,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLGATEINC_RMP_INT,
 *      EMIF4F_RDWR_LVL_RMP_CTRL_REG_WRLVLINC_RMP_INT
 *
 *   @b Example
 *   @verbatim
        Uint8  preScalar;
        Uint8  readDataEye;
        Uint8  readDQS;
        Uint8  writeInt;
        Bool   enable;

        // Get the Leveling Ramp Control Information
        CSL_EMIF4F_GetLevelingRampControlInfo (&enable, &preScalar, &readDataEye, &readDQS, &writeInt);
        ...
        // Set the Leveling Ramp control information.
        CSL_EMIF4F_SetLevelingRampControlInfo(enable, preScalar, readDataEye, readDQS, 0x1);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLevelingRampControlInfo
(
    Bool   enable,
    Uint8  preScalar,
    Uint8  readDataEye,
    Uint8  readDQS,
    Uint8  writeInt
)
{
    hEmif->RDWR_LVL_RMP_CTRL = CSL_FMK(EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVL_EN, enable) |
                               CSL_FMK(EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDWRLVLINC_RMP_PRE, preScalar) |
                               CSL_FMK(EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLINC_RMP_INT, readDataEye) |
                               CSL_FMK(EMIF4F_RDWR_LVL_RMP_CTRL_REG_RDLVLGATEINC_RMP_INT, readDQS) |
                               CSL_FMK(EMIF4F_RDWR_LVL_RMP_CTRL_REG_WRLVLINC_RMP_INT, writeInt);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetLevelingControlInfo
 *
 *   @b Description
 *   @n The function is used to get the incremental leveling control information
 *
 *   @b Arguments
     @verbatim
        trigger     - Full leveling trigger
        preScalar   - Incremental leveling pre-scalar in number of refresh periods
        readDataEye - Incremental read data eye training interval
        readDQS     - Incremental read DQS gate training interval
        writeInt    - Incremental write leveling interval
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
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLFULL_START,
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLINC_PRE,
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDLVLINC_INT,
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDLVLGATEINC_INT,
 *      EMIF4F_RDWR_LVL_CTRL_REG_WRLVLINC_INT
 *
 *   @b Example
 *   @verbatim
        Uint8  trigger;
        Uint8  preScalar;
        Uint8  readDataEye;
        Uint8  readDQS;
        Uint8  writeInt;

        // Get the Leveling Control Information
        CSL_EMIF4F_GetLevelingControlInfo (&trigger, &preScalar, &readDataEye, &readDQS, &writeInt);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetLevelingControlInfo
(
    Uint8*  trigger,
    Uint8*  preScalar,
    Uint8*  readDataEye,
    Uint8*  readDQS,
    Uint8*  writeInt
)
{
    Uint32  value = hEmif->RDWR_LVL_CTRL;

    *trigger     = CSL_FEXT(value, EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLFULL_START);
    *preScalar   = CSL_FEXT(value, EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLINC_PRE);
    *readDataEye = CSL_FEXT(value, EMIF4F_RDWR_LVL_CTRL_REG_RDLVLINC_INT);
    *readDQS     = CSL_FEXT(value, EMIF4F_RDWR_LVL_CTRL_REG_RDLVLGATEINC_INT);
    *writeInt    = CSL_FEXT(value, EMIF4F_RDWR_LVL_CTRL_REG_WRLVLINC_INT);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetLevelingControlInfo
 *
 *   @b Description
 *   @n The function is used to set the incremental leveling control information
 *
 *   @b Arguments
     @verbatim
        trigger     - Full leveling trigger
        preScalar   - Incremental leveling pre-scalar in number of refresh periods
        readDataEye - Incremental read data eye training interval
        readDQS     - Incremental read DQS gate training interval
        writeInt    - Incremental write leveling interval
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
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLFULL_START,
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLINC_PRE,
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDLVLINC_INT,
 *      EMIF4F_RDWR_LVL_CTRL_REG_RDLVLGATEINC_INT,
 *      EMIF4F_RDWR_LVL_CTRL_REG_WRLVLINC_INT
 *
 *   @b Example
 *   @verbatim
        Uint8  trigger;
        Uint8  preScalar;
        Uint8  readDataEye;
        Uint8  readDQS;
        Uint8  writeInt;

        // Get the Leveling Control Information
        CSL_EMIF4F_GetLevelingControlInfo (&trigger, &preScalar, &readDataEye, &readDQS, &writeInt);
        ...
        // Set the Leveling Control Information
        CSL_EMIF4F_SetLevelingControlInfo(1, preScalar, readDataEye, readDQS, writeInt);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetLevelingControlInfo
(
    Uint8  trigger,
    Uint8  preScalar,
    Uint8  readDataEye,
    Uint8  readDQS,
    Uint8  writeInt
)
{
    hEmif->RDWR_LVL_CTRL = CSL_FMK(EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLFULL_START, trigger)    |
                           CSL_FMK(EMIF4F_RDWR_LVL_CTRL_REG_RDWRLVLINC_PRE, preScalar)     |
                           CSL_FMK(EMIF4F_RDWR_LVL_CTRL_REG_RDLVLINC_INT, readDataEye)     |
                           CSL_FMK(EMIF4F_RDWR_LVL_CTRL_REG_RDLVLGATEINC_INT, readDQS)     |
                           CSL_FMK(EMIF4F_RDWR_LVL_CTRL_REG_WRLVLINC_INT, writeInt);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPhyControl
 *
 *   @b Description
 *   @n The function is used to get the DDR PHY Control
 *
 *   @b Arguments
     @verbatim
        ddrPHYControl       DDR PHY Control specific info populated by this API
        ddrPHYReadLatency   DDR PHY Read Latency populated by this API
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
 *      EMIF4F_DDR_PHY_CTRL_1_REG_DDR_PHY_CTRL_1, EMIF4F_DDR_PHY_CTRL_1_REG_READ_LATENCY
 *
 *   @b Example
 *   @verbatim
        Uint32 ddrPHYControl;
        Uint8  ddrPHYReadLatency;

        // Get the DDR PHY Control
        CSL_EMIF4F_GetPhyControl (&ddrPHYControl, &ddrPHYReadLatency);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPhyControl
(
    Uint32* ddrPHYControl,
    Uint8*  ddrPHYReadLatency
)
{
    Uint32 ddrControl = hEmif->DDR_PHY_CTRL_1;

    /* Extract all the fields from the PHY Control register */
    *ddrPHYControl     = CSL_FEXT (ddrControl, EMIF4F_DDR_PHY_CTRL_1_REG_DDR_PHY_CTRL_1);
    *ddrPHYReadLatency = CSL_FEXT (ddrControl, EMIF4F_DDR_PHY_CTRL_1_REG_READ_LATENCY);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPhyControl
 *
 *   @b Description
 *   @n The function is used to set the DDR PHY Control
 *
 *   @b Arguments
     @verbatim
        ddrPHYControl       DDR PHY Control specific to be configured
        ddrPHYReadLatency   DDR PHY Read Latency to be configured
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
 *      EMIF4F_DDR_PHY_CTRL_1_REG_DDR_PHY_CTRL_1, EMIF4F_DDR_PHY_CTRL_1_REG_READ_LATENCY
 *
 *   @b Example
 *   @verbatim
        Uint32 ddrPHYControl;
        Uint8  ddrPHYReadLatency;

        // Get the DDR PHY Control
        CSL_EMIF4F_GetPhyControl (&ddrPHYControl, &ddrPHYReadLatency);

        // Set Read Latency as 5 and reconfigure the PHY
        CSL_EMIF4F_SetPhyControl(ddrPHYControl, 5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPhyControl
(
    Uint32 ddrPHYControl,
    Uint8  ddrPHYReadLatency
)
{
    Uint32 ddrControl = 0x0;

    /* Initialize the fields in the DDR PHY Control */
    CSL_FINS (ddrControl, EMIF4F_DDR_PHY_CTRL_1_REG_DDR_PHY_CTRL_1, ddrPHYControl);
    CSL_FINS (ddrControl, EMIF4F_DDR_PHY_CTRL_1_REG_READ_LATENCY, ddrPHYReadLatency);

    /* Initialize the register */
    hEmif->DDR_PHY_CTRL_1 = ddrControl;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetShadowPhyControl
 *
 *   @b Description
 *   @n The function is used to get the Shadow DDR PHY Control
 *
 *   @b Arguments
     @verbatim
        ddrPHYControl       DDR PHY Control specific info populated by this API
        ddrPHYReadLatency   DDR PHY Read Latency populated by this API
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
 *      EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_DDR_PHY_CTRL_1_SHDW, 
 *      EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_READ_LATENCY_SHDW
 *
 *   @b Example
 *   @verbatim
        Uint32 ddrPHYControl;
        Uint8  ddrPHYReadLatency;

        // Get the DDR PHY Control
        CSL_EMIF4F_GetShadowPhyControl (&ddrPHYControl, &ddrPHYReadLatency);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetShadowPhyControl
(
    Uint32* ddrPHYControl,
    Uint8*  ddrPHYReadLatency
)
{
    Uint32 ddrControl = hEmif->DDR_PHY_CTRL_1_SHDW;

    /* Extract all the fields from the PHY Control register */
    *ddrPHYControl     = CSL_FEXT (ddrControl, EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_DDR_PHY_CTRL_1_SHDW);
    *ddrPHYReadLatency = CSL_FEXT (ddrControl, EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_READ_LATENCY_SHDW);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetShadowPhyControl
 *
 *   @b Description
 *   @n The function is used to set the Shadow DDR PHY Control
 *
 *   @b Arguments
     @verbatim
        ddrPHYControl       DDR PHY Control specific to be configured
        ddrPHYReadLatency   DDR PHY Read Latency to be configured
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
 *      EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_DDR_PHY_CTRL_1_SHDW, 
 *      EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_READ_LATENCY_SHDW
 *
 *   @b Example
 *   @verbatim
        Uint32 ddrPHYControl;
        Uint8  ddrPHYReadLatency;

        // Get the DDR PHY Control
        CSL_EMIF4F_GetShadowPhyControl (&ddrPHYControl, &ddrPHYReadLatency);

        // Set Read Latency as 5 and reconfigure the PHY
        CSL_EMIF4F_SetShadowPhyControl(ddrPHYControl, 5);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetShadowPhyControl
(
    Uint32 ddrPHYControl,
    Uint8  ddrPHYReadLatency
)
{
    Uint32 ddrControl = 0x0;

    /* Initialize the fields in the DDR PHY Control */
    CSL_FINS (ddrControl, EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_DDR_PHY_CTRL_1_SHDW, ddrPHYControl);
    CSL_FINS (ddrControl, EMIF4F_DDR_PHY_CTRL_1_SHDW_REG_READ_LATENCY_SHDW, ddrPHYReadLatency);

    /* Initialize the register */
    hEmif->DDR_PHY_CTRL_1_SHDW = ddrControl;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPhyControl2
 *
 *   @b Description
 *   @n The function is used to read the DDR PHY Control2 register. This is
 *      PHY specific
 *
 *   @b Arguments
     @verbatim
        ddrPHYControl       DDR PHY Control configuration populated by this API
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
 *      EMIF4F_DDR_PHY_CTRL_2
 *
 *   @b Example
 *   @verbatim
        Uint32 ddrPHYControl;

        // Get the DDR PHY Control
        CSL_EMIF4F_GetPhyControl2 (&ddrPHYControl);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPhyControl2(Uint32* ddrPHYControl)
{
    *ddrPHYControl = hEmif->DDR_PHY_CTRL_2;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPhyControl2
 *
 *   @b Description
 *   @n The function is used to write the DDR PHY Control2 register. This is
 *      PHY specific
 *
 *   @b Arguments
     @verbatim
        ddrPHYControl       DDR PHY Control configuration to be configured
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
 *      EMIF4F_DDR_PHY_CTRL_2
 *
 *   @b Example
 *   @verbatim
 
        // Set the DDR PHY Control2 configuration
        CSL_EMIF4F_SetPhyControl2 (0x10);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPhyControl2(Uint32 ddrPHYControl)
{
    hEmif->DDR_PHY_CTRL_2 = ddrPHYControl;
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetPriorityToCOSMapping
 *
 *   @b Description
 *   @n The function is used to get the priority to class of service mapping.
 *
 *   @b Arguments
     @verbatim
        ptrPriCosMapper       Priority to COS Mapping configuration
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
 *      EMIF4F_PRI_COS_MAP_REG_PRI_COS_MAP_EN, EMIF4F_PRI_COS_MAP_REG_PRI_7_COS,
 *      EMIF4F_PRI_COS_MAP_REG_PRI_5_COS, EMIF4F_PRI_COS_MAP_REG_PRI_4_COS,
 *      EMIF4F_PRI_COS_MAP_REG_PRI_3_COS, EMIF4F_PRI_COS_MAP_REG_PRI_2_COS,
 *      EMIF4F_PRI_COS_MAP_REG_PRI_1_COS, EMIF4F_PRI_COS_MAP_REG_PRI_0_COS     
 *
 *   @b Example
 *   @verbatim
        EMIF4_PRI_COS_MAPPING priCosMapper;

        // Get the Priority to COS Mapper information
        CSL_EMIF4F_GetPriorityToCOSMapping (&priCosMapper);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetPriorityToCOSMapping(EMIF4_PRI_COS_MAPPING* ptrPriCosMapper)
{
    Uint32 value = hEmif->PRI_COS_MAP;

    ptrPriCosMapper->enable  = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_COS_MAP_EN);
    ptrPriCosMapper->pri7cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_7_COS);
    ptrPriCosMapper->pri6cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_6_COS);
    ptrPriCosMapper->pri5cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_5_COS);
    ptrPriCosMapper->pri4cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_4_COS);
    ptrPriCosMapper->pri3cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_3_COS);
    ptrPriCosMapper->pri2cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_2_COS);
    ptrPriCosMapper->pri1cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_1_COS);
    ptrPriCosMapper->pri0cos = CSL_FEXT (value, EMIF4F_PRI_COS_MAP_REG_PRI_0_COS);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetPriorityToCOSMapping
 *
 *   @b Description
 *   @n The function is used to set the priority to class of service mapping.
 *
 *   @b Arguments
     @verbatim
        ptrPriCosMapper       Priority to COS Mapping configuration
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
 *      EMIF4F_PRI_COS_MAP_REG_PRI_COS_MAP_EN, EMIF4F_PRI_COS_MAP_REG_PRI_7_COS,
 *      EMIF4F_PRI_COS_MAP_REG_PRI_5_COS, EMIF4F_PRI_COS_MAP_REG_PRI_4_COS,
 *      EMIF4F_PRI_COS_MAP_REG_PRI_3_COS, EMIF4F_PRI_COS_MAP_REG_PRI_2_COS,
 *      EMIF4F_PRI_COS_MAP_REG_PRI_1_COS, EMIF4F_PRI_COS_MAP_REG_PRI_0_COS     
 *
 *   @b Example
 *   @verbatim
        EMIF4_PRI_COS_MAPPING priCosMapper;

        // Get the Priority to COS Mapper information
        CSL_EMIF4F_GetPriorityToCOSMapping (&priCosMapper);
        ...
        // Configure the mapper to allow priority to COS mapping 
        priCosMapper.enable = 1;
        CSL_EMIF4F_SetPriorityToCOSMapping (&priCosMapper);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetPriorityToCOSMapping(EMIF4_PRI_COS_MAPPING* ptrPriCosMapper)
{
    hEmif->PRI_COS_MAP = 
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_COS_MAP_EN, ptrPriCosMapper->enable)  |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_7_COS, ptrPriCosMapper->pri7cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_6_COS, ptrPriCosMapper->pri6cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_5_COS, ptrPriCosMapper->pri5cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_4_COS, ptrPriCosMapper->pri4cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_3_COS, ptrPriCosMapper->pri3cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_2_COS, ptrPriCosMapper->pri2cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_1_COS, ptrPriCosMapper->pri1cos)      |
        CSL_FMK (EMIF4F_PRI_COS_MAP_REG_PRI_0_COS, ptrPriCosMapper->pri0cos);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetMstIDToCos1Mapping
 *
 *   @b Description
 *   @n The function is used to get the master id to Class of Service 1 mapping
 *
 *   @b Arguments
     @verbatim
        ptrMstIDCosMapper       Master ID to COS Mapping configuration
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
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSTID_COS_1_MAP_EN, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_1_COS_1,
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSK_1_COS_1, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_2_COS_1,
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSK_2_COS_1, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_3_COS_1,
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSK_3_COS_1
 *
 *   @b Example
 *   @verbatim
        EMIF4_MSTID_COS_MAPPING mstIDCosMapper;

        // Get the Master ID to COS1 Mapper information
        CSL_EMIF4F_GetMstIDToCos1Mapping (&mstIDCosMapper);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetMstIDToCos1Mapping(EMIF4_MSTID_COS_MAPPING* ptrMstIDCosMapper)
{
    Uint32 value = hEmif->MSTID_COS_1_MAP;

    ptrMstIDCosMapper->enable   = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_COS_1_MAP_EN);
    ptrMstIDCosMapper->mst1     = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_1_COS_1);
    ptrMstIDCosMapper->mstMask1 = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSK_1_COS_1);
    ptrMstIDCosMapper->mst2     = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_2_COS_1);
    ptrMstIDCosMapper->mstMask2 = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSK_2_COS_1);
    ptrMstIDCosMapper->mst3     = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_3_COS_1);
    ptrMstIDCosMapper->mstMask3 = CSL_FEXT (value, EMIF4F_MSTID_COS_1_MAP_REG_MSK_3_COS_1);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetMstIDToCos1Mapping
 *
 *   @b Description
 *   @n The function is used to set the master id to Class of Service 1 mapping
 *
 *   @b Arguments
     @verbatim
        ptrMstIDCosMapper       Master ID to COS Mapping configuration
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
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSTID_COS_1_MAP_EN, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_1_COS_1,
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSK_1_COS_1, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_2_COS_1,
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSK_2_COS_1, EMIF4F_MSTID_COS_1_MAP_REG_MSTID_3_COS_1,
 *      EMIF4F_MSTID_COS_1_MAP_REG_MSK_3_COS_1
 *
 *   @b Example
 *   @verbatim
        EMIF4_MSTID_COS_MAPPING mstIDCosMapper;

        // Get the Master ID to COS1 Mapper information
        CSL_EMIF4F_GetMstIDToCos1Mapping (&mstIDCosMapper);
        ...
        // Enable the Master ID to COS1 Mapping
        mstIDCosMapper.enable = 1;
        CSL_EMIF4F_SetMstIDToCos1Mapping (&mstIDCosMapper);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetMstIDToCos1Mapping(EMIF4_MSTID_COS_MAPPING* ptrMstIDCosMapper)
{
    hEmif->MSTID_COS_1_MAP = 
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSTID_COS_1_MAP_EN, ptrMstIDCosMapper->enable)  |
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSTID_1_COS_1,      ptrMstIDCosMapper->mst1)    |
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSK_1_COS_1,        ptrMstIDCosMapper->mstMask1)|
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSTID_2_COS_1,      ptrMstIDCosMapper->mst2)    |
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSK_2_COS_1,        ptrMstIDCosMapper->mstMask2)|
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSTID_3_COS_1,      ptrMstIDCosMapper->mst3)    |
            CSL_FMK (EMIF4F_MSTID_COS_1_MAP_REG_MSK_3_COS_1,        ptrMstIDCosMapper->mstMask3);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetMstIDToCos2Mapping
 *
 *   @b Description
 *   @n The function is used to get the master id to Class of Service 2 mapping
 *
 *   @b Arguments
     @verbatim
        ptrMstIDCosMapper       Master ID to COS Mapping configuration
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
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSTID_COS_2_MAP_EN, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_1_COS_2,
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSK_1_COS_2, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_2_COS_2,
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSK_2_COS_2, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_3_COS_2,
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSK_3_COS_2
 *
 *   @b Example
 *   @verbatim
        EMIF4_MSTID_COS_MAPPING mstIDCosMapper;

        // Get the Master ID to COS2 Mapper information
        CSL_EMIF4F_GetMstIDToCos2Mapping (&mstIDCosMapper);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetMstIDToCos2Mapping(EMIF4_MSTID_COS_MAPPING* ptrMstIDCosMapper)
{
    Uint32 value = hEmif->MSTID_COS_2_MAP;

    ptrMstIDCosMapper->enable   = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_COS_2_MAP_EN);
    ptrMstIDCosMapper->mst1     = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_1_COS_2);
    ptrMstIDCosMapper->mstMask1 = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSK_1_COS_2);
    ptrMstIDCosMapper->mst2     = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_2_COS_2);
    ptrMstIDCosMapper->mstMask2 = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSK_2_COS_2);
    ptrMstIDCosMapper->mst3     = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_3_COS_2);
    ptrMstIDCosMapper->mstMask3 = CSL_FEXT (value, EMIF4F_MSTID_COS_2_MAP_REG_MSK_3_COS_2);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetMstIDToCos2Mapping
 *
 *   @b Description
 *   @n The function is used to set the master id to Class of Service 2 mapping
 *
 *   @b Arguments
     @verbatim
        ptrMstIDCosMapper       Master ID to COS Mapping configuration
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
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSTID_COS_2_MAP_EN, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_1_COS_2,
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSK_1_COS_2, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_2_COS_2,
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSK_2_COS_2, EMIF4F_MSTID_COS_2_MAP_REG_MSTID_3_COS_2,
 *      EMIF4F_MSTID_COS_2_MAP_REG_MSK_3_COS_2
 *
 *   @b Example
 *   @verbatim
        EMIF4_MSTID_COS_MAPPING mstIDCosMapper;

        // Get the Master ID to COS2 Mapper information
        CSL_EMIF4F_GetMstIDToCos2Mapping (&mstIDCosMapper);
        ...
        // Enable the Master ID to COS2 Mapping
        mstIDCosMapper.enable = 1;
        CSL_EMIF4F_SetMstIDToCos2Mapping (&mstIDCosMapper);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetMstIDToCos2Mapping(EMIF4_MSTID_COS_MAPPING* ptrMstIDCosMapper)
{
    hEmif->MSTID_COS_2_MAP = 
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSTID_COS_2_MAP_EN, ptrMstIDCosMapper->enable)  |
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSTID_1_COS_2,      ptrMstIDCosMapper->mst1)    |
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSK_1_COS_2,        ptrMstIDCosMapper->mstMask1)|
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSTID_2_COS_2,      ptrMstIDCosMapper->mst2)    |
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSK_2_COS_2,        ptrMstIDCosMapper->mstMask2)|
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSTID_3_COS_2,      ptrMstIDCosMapper->mst3)    |
            CSL_FMK (EMIF4F_MSTID_COS_2_MAP_REG_MSK_3_COS_2,        ptrMstIDCosMapper->mstMask3);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetECCControl
 *
 *   @b Description
 *   @n The function is used to get the ECC Control Information
 *
 *   @b Arguments
     @verbatim
        ptrECCControl       ECC Control Configuration
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
 *      EMIF4F_ECC_CTRL_REG_ECC_EN, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_PROT,
 *      EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_2_EN, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_1_EN
 *
 *   @b Example
 *   @verbatim
        EMIF4_ECC_CONTROL eccControl;

        // Get the ECC Control Configuration
        CSL_EMIF4F_GetECCControl (&eccControl);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetECCControl(EMIF4_ECC_CONTROL* ptrECCControl)
{
    Uint32 value = hEmif->ECC_CTRL;

    ptrECCControl->enable       = CSL_FEXT (value, EMIF4F_ECC_CTRL_REG_ECC_EN);
    ptrECCControl->addrRngProt  = CSL_FEXT (value, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_PROT);
    ptrECCControl->addrRng2En   = CSL_FEXT (value, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_2_EN);
    ptrECCControl->addrRng1En   = CSL_FEXT (value, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_1_EN);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetECCControl
 *
 *   @b Description
 *   @n The function is used to set the ECC Control Information
 *
 *   @b Arguments
     @verbatim
        ptrECCControl       ECC Control Configuration
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
 *      EMIF4F_ECC_CTRL_REG_ECC_EN, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_PROT,
 *      EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_2_EN, EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_1_EN
 *
 *   @b Example
 *   @verbatim
        EMIF4_ECC_CONTROL eccControl;

        // Get the ECC Control Configuration
        CSL_EMIF4F_GetECCControl (&eccControl);
        ..
        // Enable ECC 
        eccControl.enable = 1;
        CSL_EMIF4F_SetECCControl (&eccControl);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetECCControl(EMIF4_ECC_CONTROL* ptrECCControl)
{
    hEmif->ECC_CTRL = 
        CSL_FMK (EMIF4F_ECC_CTRL_REG_ECC_EN,            ptrECCControl->enable)      |
        CSL_FMK (EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_PROT, ptrECCControl->addrRngProt) |
        CSL_FMK (EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_2_EN, ptrECCControl->addrRng2En)  |
        CSL_FMK (EMIF4F_ECC_CTRL_REG_ECC_ADDR_RNG_1_EN, ptrECCControl->addrRng1En);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetECCAddressRange
 *
 *   @b Description
 *   @n The function is used to get the ECC Address Range
 *
 *   @b Arguments
     @verbatim
        range           Address Range for which start and end address are required
        endAddr         End Address
        startAddr       Start Address
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
 *      EMIF4F_ECC_ADDR_RNG_1_REG_ECC_END_ADDR_1, EMIF4F_ECC_ADDR_RNG_1_REG_ECC_STRT_ADDR_1;
 *      EMIF4F_ECC_ADDR_RNG_2_REG_ECC_END_ADDR_2, EMIF4F_ECC_ADDR_RNG_2_REG_ECC_STRT_ADDR_2
 *
 *   @b Example
 *   @verbatim
        Uint16  endAddr;
        Uint16  startAddr;

        // Get the ECC Address Range1
        CSL_EMIF4F_GetECCAddressRange (1, &endAddr, &startAddr);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetECCAddressRange(Uint8 range, Uint16* endAddr, Uint16* startAddr)
{
    Uint32 value;

    if (range == 1)
    {
        value = hEmif->ECC_ADDR_RNG_1;
        *endAddr    = CSL_FEXT(value, EMIF4F_ECC_ADDR_RNG_1_REG_ECC_END_ADDR_1);
        *startAddr  = CSL_FEXT(value, EMIF4F_ECC_ADDR_RNG_1_REG_ECC_STRT_ADDR_1);
    }
    else if (range == 2)
    {
        value = hEmif->ECC_ADDR_RNG_2;
        *endAddr    = CSL_FEXT(value, EMIF4F_ECC_ADDR_RNG_2_REG_ECC_END_ADDR_2);
        *startAddr  = CSL_FEXT(value, EMIF4F_ECC_ADDR_RNG_2_REG_ECC_STRT_ADDR_2);
    }
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetECCAddressRange
 *
 *   @b Description
 *   @n The function is used to set the ECC Address Range
 *
 *   @b Arguments
     @verbatim
        range           Address Range for which start and end address are required
        endAddr         End Address
        startAddr       Start Address
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
 *      EMIF4F_ECC_ADDR_RNG_1_REG_ECC_END_ADDR_1, EMIF4F_ECC_ADDR_RNG_1_REG_ECC_STRT_ADDR_1;
 *      EMIF4F_ECC_ADDR_RNG_2_REG_ECC_END_ADDR_2, EMIF4F_ECC_ADDR_RNG_2_REG_ECC_STRT_ADDR_2
 *
 *   @b Example
 *   @verbatim
        Uint16  endAddr;
        Uint16  startAddr;

        // Set the ECC Address Range1
        CSL_EMIF4F_SetECCAddressRange (1, 0x100, 0x0);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetECCAddressRange(Uint8 range, Uint16 endAddr, Uint16 startAddr)
{
    if (range == 1)
    {
        hEmif->ECC_ADDR_RNG_1 = 
            CSL_FMK (EMIF4F_ECC_ADDR_RNG_1_REG_ECC_END_ADDR_1,  endAddr) |
            CSL_FMK (EMIF4F_ECC_ADDR_RNG_1_REG_ECC_STRT_ADDR_1, startAddr);
    }
    else if (range == 2)
    {
        hEmif->ECC_ADDR_RNG_2 = 
            CSL_FMK (EMIF4F_ECC_ADDR_RNG_2_REG_ECC_END_ADDR_2,  endAddr) |
            CSL_FMK (EMIF4F_ECC_ADDR_RNG_2_REG_ECC_STRT_ADDR_2, startAddr);
    }
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_GetReadWriteThreshold
 *
 *   @b Description
 *   @n The function is used to get the Read, Write Execution Thresholds
 *
 *   @b Arguments
     @verbatim
        wrThreshold       Write Threshold
        rdThreshold       Read  Threshold
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
 *      EMIF4F_RD_WR_EXEC_THRSH_REG_WR_THRSH, EMIF4F_RD_WR_EXEC_THRSH_REG_RD_THRSH
 *
 *   @b Example
 *   @verbatim
        Uint8   wrThreshold;
        Uint8   rdThreshold;

        // Get the Read, Write Execution Thresholds.
        CSL_EMIF4F_GetReadWriteThreshold (&wrThreshold, &rdThreshold);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_GetReadWriteThreshold(Uint8* wrThreshold, Uint8* rdThreshold)
{
    Uint32 value = hEmif->RD_WR_EXEC_THRSH;

    *wrThreshold = CSL_FEXT (value, EMIF4F_RD_WR_EXEC_THRSH_REG_WR_THRSH);
    *rdThreshold = CSL_FEXT (value, EMIF4F_RD_WR_EXEC_THRSH_REG_RD_THRSH);
}

/** ============================================================================
 *   @n@b CSL_EMIF4F_SetReadWriteThreshold
 *
 *   @b Description
 *   @n The function is used to set the Read, Write Execution Thresholds
 *
 *   @b Arguments
     @verbatim
        wrThreshold       Write Threshold
        rdThreshold       Read  Threshold
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
 *      EMIF4F_RD_WR_EXEC_THRSH_REG_WR_THRSH, EMIF4F_RD_WR_EXEC_THRSH_REG_RD_THRSH
 *
 *   @b Example
 *   @verbatim

        // Get the Read, Write Execution Thresholds.
        CSL_EMIF4F_SetReadWriteThreshold (1, 2);
        ...
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_EMIF4F_SetReadWriteThreshold(Uint8 wrThreshold, Uint8 rdThreshold)
{
    hEmif->RD_WR_EXEC_THRSH = 
        CSL_FMK (EMIF4F_RD_WR_EXEC_THRSH_REG_WR_THRSH, wrThreshold) |
        CSL_FMK (EMIF4F_RD_WR_EXEC_THRSH_REG_RD_THRSH, rdThreshold);
}




/* @} */

#ifdef __cplusplus
}
#endif

#endif /* _CSL_EMIF4FAUX_H_ */




