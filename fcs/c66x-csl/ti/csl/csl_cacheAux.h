/**
 *   @file  csl_cacheAux.h
 *
 *   @brief   
 *      This is the CACHE Auxilary Header File which exposes the various
 *      CSL Functional Layer API's to configure the CACHE Module.
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

#ifndef _CSL_CACHEAUX_H_
#define _CSL_CACHEAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_cache.h>

/** @addtogroup CSL_CACHE_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CACHE_enableCaching
 *
 *   @b Description
 *   @n This function enables caching for a specific memory region. 
 *
 *   @b Arguments
     @verbatim
          mar      Memory region for which cache is to be enabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n Memory region is now cacheable.
 *
 *   @b Writes
 *   @n CGEM_MAR0_PC=1
 *
 *   @b Example
 *   @verbatim
        CACHE_enableCaching (20);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_enableCaching  (Uint8 mar)
{
    CSL_FINS(hCache->MAR[mar], CGEM_MAR0_PC, 1);
}

/** ============================================================================
 *   @n@b CACHE_disableCaching
 *
 *   @b Description
 *   @n This function disables caching for a specific memory region. 
 *
 *   @b Arguments
     @verbatim
          mar      Memory region for which cache is to be disabled.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n Memory region is now *not* cacheable.
 *
 *   @b Writes
 *   @n CGEM_MAR0_PC=0
 *
 *   @b Example
 *   @verbatim
        CACHE_disableCaching (20);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_disableCaching (Uint8 mar)
{
    CSL_FINS(hCache->MAR[mar], CGEM_MAR0_PC, 0);
}

/** ============================================================================
 *   @n@b CACHE_getMemRegionInfo
 *
 *   @b Description
 *   @n This function is used to get memory region information.
 *
 *   @b Arguments
     @verbatim
          mar      Memory region for which the information is required.
          pcx      Is address cacheable in external cache (MSMC)
          pfx      Is address prefetchable 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n CGEM_MAR0_PCX, CGEM_MAR0_PFX
 *
 *   @b Example
 *   @verbatim
        
        Uint8 pcx;
        Uint8 pfx;

        // Get the memory region information for 20
        CACHE_getMemRegionInfo (20, &pcx, &pfx);

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_getMemRegionInfo (Uint8 mar, Uint8* pcx, Uint8* pfx)
{
    Uint32 value = hCache->MAR[mar];

    *pcx = CSL_FEXT (value, CGEM_MAR0_PCX);
    *pfx = CSL_FEXT (value, CGEM_MAR0_PFX);
}

/** ============================================================================
 *   @n@b CACHE_setMemRegionInfo
 *
 *   @b Description
 *   @n This function is used to set memory region information.
 *
 *   @b Arguments
     @verbatim
          mar      Memory region for which the information is required.
          pcx      Is address cacheable in external cache (MSMC)
          pfx      Is address prefetchable 
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n CGEM_MAR0_PCX, CGEM_MAR0_PFX
 *
 *   @b Example
 *   @verbatim
        
        Uint8 pcx;
        Uint8 pfx;

        // Get the memory region information for 20
        CACHE_getMemRegionInfo (20, &pcx, &pfx);
        ...
        // Ensure Memory Region 20 is not prefetchable.
        CACHE_setMemRegionInfo(20, pcx, 0);
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_setMemRegionInfo (Uint8 mar, Uint8 pcx, Uint8 pfx)
{
    CSL_FINS (hCache->MAR[mar], CGEM_MAR0_PCX, pcx);
    CSL_FINS (hCache->MAR[mar], CGEM_MAR0_PFX, pfx);
}

/** ============================================================================
 *   @n@b CACHE_setL1DSize
 *
 *   @b Description
 *   @n This function is used to set the L1 Data Cache Size.  
 *
 *   @b Arguments
      @verbatim
          newSize      Cache Size to be configured.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CGEM_L1DCFG_L1DMODE
 *
 *   @b Example
 *   @verbatim
        
        CACHE_setL1DSize(1); // Configure 4K Cache Size

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_setL1DSize (CACHE_L1Size newSize)
{
    CSL_FINS (hCache->L1DCFG, CGEM_L1DCFG_L1DMODE, newSize);
}

/** ============================================================================
 *   @n@b CACHE_getL1DSize
 *
 *   @b Description
 *   @n This function is used to get the L1 Data Cache Size.  
 *
 *   @b Arguments
 *   @n None
 * 
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CGEM_L1DCFG_L1DMODE
 *
 *   @b Example
 *   @verbatim
        CACHE_L1Size cacheSize;
        
        cacheSize = CACHE_getL1DSize();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE CACHE_L1Size CACHE_getL1DSize (void)
{
    return (CACHE_L1Size)CSL_FEXT (hCache->L1DCFG, CGEM_L1DCFG_L1DMODE);
}

/** ============================================================================
 *   @n@b CACHE_freezeL1D
 *
 *   @b Description
 *   @n This function is used to freeze the L1D cache.  
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CGEM_L1DCC_OPER=1
 *
 *   @b Example
 *   @verbatim
        
        CACHE_freezeL1D();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_freezeL1D(void)
{
    /* Set the Freeze Mode Enabled bit. */
    CSL_FINS (hCache->L1DCC, CGEM_L1DCC_OPER, 1);
}

/** ============================================================================
 *   @n@b CACHE_unfreezeL1D
 *
 *   @b Description
 *   @n This function is used to unfreeze the L1D cache.  
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CGEM_L1DCC_OPER=0
 *
 *   @b Example
 *   @verbatim
        
        CACHE_unfreezeL1D();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_unfreezeL1D(void)
{
    /* Reset the Freeze Mode Enabled bit. */
    CSL_FINS (hCache->L1DCC, CGEM_L1DCC_OPER, 0);
}

/** ============================================================================
 *   @n@b CACHE_getPrevL1DMode
 *
 *   @b Description
 *   @n This function is used get the previous operating state of the L1D cache 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CGEM_L1DCC_POPER
 *
 *   @b Example
 *   @verbatim
        Uint32 prev;
        
        prev = CACHE_getPrevL1DMode();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CACHE_getPrevL1DMode(void)
{
    return CSL_FEXT (hCache->L1DCC, CGEM_L1DCC_POPER);    
}

/** ============================================================================
 *   @n@b CACHE_invAllL1dWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D global invalidate operation
 *      to complete. This API should be used only if the CACHE_invAllL1d was called
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_invAllL1d(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CGEM_L1DINV_I=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_invAllL1d(CACHE_NOWAIT); // Invalidate the L1D cache
        ...        
        CACHE_invAllL1dWait();        // Wait for the invalidate operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invAllL1dWait (void)
{
    /* Wait for the Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L1DINV, CGEM_L1DINV_I) == 1);
}

/** ============================================================================
 *   @n@b CACHE_invAllL1d
 *
 *   @b Description
 *   @n This function is used to globally invalidate the L1D cache.  
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The L1D Cache is being invalidated. 
 *
 *   @b Writes
 *   @n CGEM_L1DINV_I=1
 *
 *   @b Example
 *   @verbatim
        
        CACHE_invAllL1d(CACHE_WAIT); // Invalidate the L1D cache 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invAllL1d (CACHE_Wait wait)
{
    /* Invalidate the Cache Line. */
    CSL_FINS (hCache->L1DINV, CGEM_L1DINV_I, 1); 
    
    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_invAllL1dWait();
}

/** ============================================================================
 *   @n@b CACHE_wbAllL1dWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D writeback operation
 *      to complete. This API should be used only if the CACHE_wbAllL1d was called
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbAllL1d(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The L1D Dirty lines are written back
 *
 *   @b Reads
 *   @n CGEM_L1DWB_C=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_wbAllL1d(CACHE_NOWAIT); // Writeback the L1D cache
        ...        
        CACHE_wbAllL1dWait();        // Wait for the writeback operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbAllL1dWait (void)
{
    /* Wait for the Writeback operation to complete. */
    while (CSL_FEXT(hCache->L1DWB, CGEM_L1DWB_C) == 1);
}

/** ============================================================================
 *   @n@b CACHE_wbAllL1d
 *
 *   @b Description
 *   @n This function is used to writeback the dirty lines of the L1D Cache  
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The dirty lines of the L1D Cache are being written back 
 *
 *   @b Writes
 *   @n CGEM_L1DWB_C=1
 *
 *   @b Example
 *   @verbatim
        
        CACHE_wbAllL1d(CACHE_WAIT); // Writeback the Dirty Lines of the L1D cache 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbAllL1d (CACHE_Wait wait)
{
    /* Writeback the Cache Line. */
    CSL_FINS (hCache->L1DWB, CGEM_L1DWB_C, 1); 
    
    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_wbAllL1dWait();
}

/** ============================================================================
 *   @n@b CACHE_wbInvAllL1dWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D writeback invalidate operation
 *      to complete. This API should be used only if the CACHE_wbInvAllL1d was called
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbInvAllL1d(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The L1D Dirty lines are written back
 *
 *   @b Reads
 *   @n CGEM_L1DWBINV_C=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_wbInvAllL1d(CACHE_NOWAIT); // Invalidate/Writeback the L1D cache
        ...        
        CACHE_wbInvAllL1dWait();        // Wait for the Invalidate/Writeback operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvAllL1dWait (void)
{
    /* Wait for the Invalidate Writeback operation to complete. */
    while (CSL_FEXT(hCache->L1DWBINV, CGEM_L1DWBINV_C) == 1);
}

/** ============================================================================
 *   @n@b CACHE_wbInvAllL1d
 *
 *   @b Description
 *   @n This function is used to invalidate and writeback the dirty lines of the 
 *      L1D Cache  
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  Invalidates and Writebacks the dirty lines of the L1D Cache 
 *
 *   @b Writes
 *   @n CGEM_L1DWBINV_C=1
 *
 *   @b Example
 *   @verbatim
        
        CACHE_wbInvAllL1d(CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void  CACHE_wbInvAllL1d (CACHE_Wait wait)
{
    /* Invalidate and writeback the cache line. */
    CSL_FINS (hCache->L1DWBINV, CGEM_L1DWBINV_C, 1); 
    
    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_wbInvAllL1dWait();     
}

/** ============================================================================
 *   @n@b CACHE_invL1dWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D invalidate block operation to 
 *      complete. This API should be used only if the CACHE_invL1d was called 
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_invL1d(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The L1D Block Cache is invalidated. 
 *
 *   @b Reads
 *   @n CGEM_L1DIWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_invL1d((void *)ptr_buffer, 128, CACHE_NOWAIT);
        ...        
        CACHE_invL1dWait();        // Wait for the Invalidate/Writeback operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invL1dWait (void)
{
    /* Wait for the Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L1DIWC, CGEM_L1DIWC_WC) != 0);
}

/** ============================================================================
 *   @n@b CACHE_invL1d
 *
 *   @b Description
 *   @n This function is used to invalidate a block in the L1D Cache. Although
 *      the block size can be specified in the number of bytes, the cache
 *      controller operates on whole cache lines. To prevent unintended behavior
 *      "blockPtr" should be aligned on the cache line size and "byteCnt" should
 *      be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be invalidated
          byteCnt       Size of the block to be invalidated.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the blockPtr are being invalidated  
 *
 *   @b Writes
 *   @n CGEM_L1DIBAR_ADDR,CGEM_L1DIWC_WC
 *
 *   @b Example
 *   @verbatim
        
        Uint8* ptr_buffer;
                
        // Invalidate 128 bytes of the buffer.
        CACHE_invL1d((void *)ptr_buffer, 128, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invL1d 
(
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length */
    hCache->L1DIBAR = CSL_FMK(CGEM_L1DIBAR_ADDR, (Uint32)blockPtr);
    hCache->L1DIWC  = CSL_FMK(CGEM_L1DIWC_WC,    (Uint32)((byteCnt+3)>>2));

    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_invL1dWait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_wbL1dWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D writeback block operation to 
 *      complete. This API should be used only if the CACHE_wbL1d was called 
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbL1d(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The dirty lines of the L1D Block Cache have been written back. 
 *
 *   @b Reads
 *   @n CGEM_L1DWWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_wbL1d((void *)ptr_buffer, 128, CACHE_NOWAIT);
        ...        
        CACHE_wbL1dWait();        // Wait for the writeback operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbL1dWait (void)
{
    /* Wait for the Writeback operation to complete. */
    while (CSL_FEXT(hCache->L1DWWC, CGEM_L1DWWC_WC) != 0);    
}

/** ============================================================================
 *   @n@b CACHE_wbL1d
 *
 *   @b Description
 *   @n This function is used to writeback the dirty lines of the block address.
 *      Although the block size can be specified in the number of bytes, the cache
 *      controller operates on whole cache lines. To prevent unintended behavior
 *      "blockPtr" should be aligned on the cache line size and "byteCnt" should
 *      be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be written back
          byteCnt       Size of the block to be written back.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the blockPtr are being written back  
 *
 *   @b Writes
 *   @n CGEM_L1DWBAR_ADDR,CGEM_L1DWWC_WC
 *
 *   @b Example
 *   @verbatim
        
        Uint8* ptr_buffer;
                
        // Writeback 128 bytes of the buffer.
        CACHE_wbL1d((void *)ptr_buffer, 128, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbL1d 
(
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length */
    hCache->L1DWBAR = CSL_FMK (CGEM_L1DWBAR_ADDR, (Uint32)blockPtr);
    hCache->L1DWWC  = CSL_FMK (CGEM_L1DWWC_WC,    (Uint32)((byteCnt+3)>>2));

    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_wbL1dWait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_wbInvL1dWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D invalidate/writeback block 
 *      operation to complete. This API should be used only if the CACHE_wbInvL1d 
 *      was called with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbInvL1d(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The dirty lines of the L1D Block Cache have been written back and the cache
 *       contents pointed to by the block address are also invalidated.
 *
 *   @b Reads
 *   @n CGEM_L1DWIWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_wbInvL1d((void *)ptr_buffer, 128, CACHE_NOWAIT);
        ...        
        CACHE_wbInvL1dWait();        // Wait for the operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvL1dWait (void)
{
    /* Wait for the Block Writeback/Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L1DWIWC, CGEM_L1DWIWC_WC) != 0);
}

/** ============================================================================
 *   @n@b CACHE_wbInvL1d
 *
 *   @b Description
 *   @n This function is used to invalidate and writeback the dirty lines 
 *      of the block address.  Although the block size can be specified in 
 *      the number of bytes, the cache controller operates on whole cache lines. 
 *      To prevent unintended behavior "blockPtr" should be aligned on the 
 *      cache line size and "byteCnt" should be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be invalidated/written back
          byteCnt       Size of the block to be invalidated/written back.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the blockPtr are being invalidated and the dirty lines are
 *       written back  
 *
 *   @b Writes
 *   @n CGEM_L1DWIBAR_ADDR,CGEM_L1DWIWC_WC
 *
 *   @b Example
 *   @verbatim
        
        Uint8* ptr_buffer;
                
        // Writeback/Invalidate 128 bytes of the buffer.
        CACHE_wbInvL1d((void *)ptr_buffer, 128, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvL1d 
(
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length */
    hCache->L1DWIBAR = CSL_FMK(CGEM_L1DWIBAR_ADDR, (Uint32)blockPtr);
    hCache->L1DWIWC  = CSL_FMK(CGEM_L1DWIWC_WC,    (Uint32)((byteCnt+3)>>2));
 
    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_wbInvL1dWait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_setL1PSize
 *
 *   @b Description
 *   @n This function is used to set the L1P Cache Size.  
 *
 *   @b Arguments
      @verbatim
          newSize      Cache Size to be configured.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CGEM_L1PCFG_L1PMODE
 *
 *   @b Example
 *   @verbatim
        
        CACHE_setL1PSize(1); // Configure 4K Cache Size

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_setL1PSize (CACHE_L1Size newSize)
{
    CSL_FINS (hCache->L1PCFG, CGEM_L1PCFG_L1PMODE, newSize);
}

/** ============================================================================
 *   @n@b CACHE_getL1PSize
 *
 *   @b Description
 *   @n This function is used to get the L1P Cache Size.  
 *
 *   @b Arguments
 *   @n None
 * 
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CGEM_L1PCFG_L1PMODE
 *
 *   @b Example
 *   @verbatim
        CACHE_L1Size cacheSize;
        
        cacheSize = CACHE_getL1PSize();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE CACHE_L1Size CACHE_getL1PSize (void)
{
    return (CACHE_L1Size)CSL_FEXT (hCache->L1PCFG, CGEM_L1PCFG_L1PMODE);
}

/** ============================================================================
 *   @n@b CACHE_freezeL1P
 *
 *   @b Description
 *   @n This function is used to freeze the L1P cache.  
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CGEM_L1PCC_OPER=1
 *
 *   @b Example
 *   @verbatim
        
        CACHE_freezeL1P();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_freezeL1P(void)
{
    /* Set the Freeze Mode Enabled bit. */
    CSL_FINS (hCache->L1PCC, CGEM_L1PCC_OPER, 1);
}

/** ============================================================================
 *   @n@b CACHE_unfreezeL1P
 *
 *   @b Description
 *   @n This function is used to unfreeze the L1D cache.  
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CGEM_L1PCC_OPER=0
 *
 *   @b Example
 *   @verbatim
        
        CACHE_unfreezeL1D();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_unfreezeL1P(void)
{
    /* Reset the Freeze Mode Enabled bit. */
    CSL_FINS (hCache->L1PCC, CGEM_L1PCC_OPER, 0);
}

/** ============================================================================
 *   @n@b CACHE_getPrevL1PMode
 *
 *   @b Description
 *   @n This function is used get the previous operating state of the L1P cache 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CGEM_L1PCC_POPER
 *
 *   @b Example
 *   @verbatim
        Uint32  prev;
        
        prev = CACHE_getPrevL1PMode();

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE Uint32 CACHE_getPrevL1PMode(void)
{
    return CSL_FEXT (hCache->L1PCC, CGEM_L1PCC_POPER);    
}

/** ============================================================================
 *   @n@b CACHE_invL1pWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1D invalidate block operation to 
 *      complete. This API should be used only if the CACHE_invL1p was called 
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_invL1p(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The L1D Block Cache is invalidated. 
 *
 *   @b Reads
 *   @n CGEM_L1PIWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_invL1p((void *)&foo, 128, CACHE_NOWAIT);
        ...        
        CACHE_invL1pWait();        // Wait for the Invalidate operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invL1pWait (void)
{
    /* Wait for the Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L1PIWC, CGEM_L1PIWC_WC) != 0);    
}

/** ============================================================================
 *   @n@b CACHE_invL1p
 *
 *   @b Description
 *   @n This function is used to invalidate the L1P Cache pointed by the block 
 *      address. Although the block size can be specified in the number of bytes, 
 *      the cache controller operates on whole cache lines. To prevent unintended 
 *      behavior "blockPtr" should be aligned on the cache line size and "byteCnt" 
 *      should be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be invalidated
          byteCnt       Size of the block to be invalidated.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the blockPtr are being invalidated
 *
 *   @b Writes
 *   @n CGEM_L1PIBAR_ADDR,CGEM_L1PIWC_WC
 *
 *   @b Example
 *   @verbatim
 
        // Invalidate the 128 bytes of the function 'foo'
        CACHE_invL1p((void *)&foo, 128, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invL1p 
(
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length which is to be invalidated */
    hCache->L1PIBAR = CSL_FMK(CGEM_L1PIBAR_ADDR, (Uint32)blockPtr);
    hCache->L1PIWC  = CSL_FMK(CGEM_L1PIWC_WC,    (Uint32)((byteCnt+3)>>2));

    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_invL1pWait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_invAllL1pWait
 *
 *   @b Description
 *   @n This function is used to wait for the L1P invalidate operation to complete. 
 *      This API should be used only if the CACHE_invAllL1p was called with the 
 *      CACHE_NOWAIT argument.  
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_invAllL1p(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The L1P Cache is invalidated. 
 *
 *   @b Reads
 *   @n CGEM_L1PINV_I=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_invAllL1p(CACHE_NOWAIT);
        ...        
        CACHE_invAllL1pWait();        // Wait for the Invalidate operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invAllL1pWait (void)
{
    /* Wait for the Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L1PINV, CGEM_L1PINV_I) == 1);    
}

/** ============================================================================
 *   @n@b CACHE_invAllL1p
 *
 *   @b Description
 *   @n This function is used to invalidate the entire L1P Cache 
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The entire L1P cache is being invalidated.  
 *
 *   @b Writes
 *   @n CGEM_L1PINV_I=1
 *
 *   @b Example
 *   @verbatim
         
        CACHE_invAllL1p(CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invAllL1p (CACHE_Wait wait)
{
    /* Invalidate the L1P Cache. */
    CSL_FINS (hCache->L1PINV, CGEM_L1PINV_I, 1);    

    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_invAllL1pWait();
}

/** ============================================================================
 *   @n@b CACHE_setL2Size
 *
 *   @b Description
 *   @n This function is used to set the new size of the L2 Cache.  
 *
 *   @b Arguments
      @verbatim
          newSize   New Size of the L2 Cache to be set.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The L2 Cache is configured to use the new size.  
 *
 *   @b Writes
 *   @n CGEM_L2CFG_L2MODE
 *
 *   @b Example
 *   @verbatim
         
        CACHE_setL2Size(CACHE_32KCACHE); // Use 32K L2 Cache. 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_setL2Size (CACHE_L2Size newSize)
{
    /* Set the new L2 cache size. */
    CSL_FINS (hCache->L2CFG, CGEM_L2CFG_L2MODE, newSize);
}

/** ============================================================================
 *   @n@b CACHE_getL2Size
 *
 *   @b Description
 *   @n This function is used to get the L2 cache size.  
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  CACHE_L2Size
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None  
 *
 *   @b Reads
 *   @n CGEM_L2CFG_L2MODE
 *
 *   @b Example
 *   @verbatim
        CACHE_L2Size size;
        
        size = CACHE_getL2Size(); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE CACHE_L2Size CACHE_getL2Size (void)
{    
    return (CACHE_L2Size) CSL_FEXT (hCache->L2CFG, CGEM_L2CFG_L2MODE);
}

/** ============================================================================
 *   @n@b CACHE_freezeL2
 *
 *   @b Description
 *   @n This function is used to freeze the L2 Cache  
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The L2 Cache is frozen.  
 *
 *   @b Example
 *   @verbatim
         
        CACHE_freezeL2(); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_freezeL2 (void)
{
    /* The RL File does not define the L2CC bit so we used the RAW macro to 
     * configure the corresponding bit. */
    CSL_FINSR(hCache->L2CFG, 3, 3, 1);
}

/** ============================================================================
 *   @n@b CACHE_unfreezeL2
 *
 *   @b Description
 *   @n This function is used to unfreeze the L2 Cache  
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The L2 Cache is unfrozen  
 *
 *   @b Example
 *   @verbatim
         
        CACHE_unfreezeL2(); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_unfreezeL2 (void)
{
    /* The RL File does not define the L2CC bit so we used the RAW macro to 
     * configure the corresponding bit. */
    CSL_FINSR(hCache->L2CFG, 3, 3, 0);
}

/** ============================================================================
 *   @n@b CACHE_wbL2Wait
 *
 *   @b Description
 *   @n This function is used to wait for the L2 writeback block operation to 
 *      complete. This API should be used only if the CACHE_wbL2 was called 
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbL2(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The dirty lines of the L1D Block Cache have been written back. 
 *
 *   @b Reads
 *   @n CGEM_L2WWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_wbL2((void *)ptr_buffer, 128, CACHE_NOWAIT);
        ...        
        CACHE_wbL2Wait();        // Wait for the writeback operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbL2Wait (void)
{
    /* Wait for the Writeback operation to complete. */
    while (CSL_FEXT(hCache->L2WWC, CGEM_L2WWC_WC) != 0);        
}

/** ============================================================================
 *   @n@b CACHE_wbL2
 *
 *   @b Description
 *   @n This function is used to writeback the contents of the L2 Cache. Although 
 *      the block size can be specified in the number of bytes, the cache 
 *      controller operates on whole cache lines. To prevent unintended behavior 
 *      "blockPtr" should be aligned on the  cache line size and "byteCnt" 
 *      should be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be written back
          byteCnt       Size of the block to be written block.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The dirty lines of the L2 Cache are being written back.   
 *
 *   @b Writes
 *   @n CGEM_L2WBAR_ADDR,CGEM_L2WWC_WC
 *
 *   @b Example
 *   @verbatim
        Uint8* ptr_buffer;
        
        // Writeback the contents of the buffer. 
        CACHE_wbL2(ptr_buffer, 100, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbL2 
(
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length */
    hCache->L2WBAR = CSL_FMK (CGEM_L2WBAR_ADDR, (Uint32)blockPtr);    
    hCache->L2WWC  = CSL_FMK (CGEM_L2WWC_WC,    (Uint32)((byteCnt+3)>>2));

    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_wbL2Wait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_invL2Wait
 *
 *   @b Description
 *   @n This function is used to wait for the L2 invalidate block operation to 
 *      complete. This API should be used only if the CACHE_invL2 was called 
 *      with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_invL2(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  Invalidate the contents of the L2 Cache. 
 *
 *   @b Reads
 *   @n CGEM_L2IWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_invL2((void *)ptr_buffer, 128, CACHE_NOWAIT);
        ...        
        CACHE_invL2Wait();        // Wait for the Invalidate operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invL2Wait (void)
{
    /* Wait for the Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L2IWC, CGEM_L2IWC_WC) != 0);        
}

/** ============================================================================
 *   @n@b CACHE_invL2
 *
 *   @b Description
 *   @n This function is used to invalidate the contents of the L2 Cache. 
 *      Although the block size can be specified in  the number of bytes, 
 *      the cache controller operates on whole cache lines. To prevent unintended 
 *      behavior "blockPtr" should be aligned on the cache line size and "byteCnt" 
 *      should be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be invalidated
          byteCnt       Size of the block to be invalidated.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache are being invalidated.   
 *
 *   @b Writes
 *   @n CGEM_L2IBAR_ADDR,CGEM_L2IWC_WC
 *
 *   @b Example
 *   @verbatim
        Uint8* ptr_buffer;
        
        // Invalidate the contents of the buffer. 
        CACHE_invL2(ptr_buffer, 100, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invL2 
(
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length */
    hCache->L2IBAR = CSL_FMK (CGEM_L2IBAR_ADDR, (Uint32)blockPtr);    
    hCache->L2IWC  = CSL_FMK (CGEM_L2IWC_WC,    (Uint32)((byteCnt+3)>>2));

    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_invL2Wait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_wbInvL2Wait
 *
 *   @b Description
 *   @n This function is used to wait for the L2 Writeback & invalidate block 
 *      operation to complete. This API should be used only if the CACHE_wbInvL2 
 *      was called with the CACHE_NOWAIT argument. 
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbInvL2(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  Invalidate the contents of the L2 Cache. 
 *
 *   @b Reads
 *   @n CGEM_L2WIWC_WC=0
 *
 *   @b Example
 *   @verbatim
                
        CACHE_wbInvL2((void *)ptr_buffer, 128, CACHE_NOWAIT);
        ...        
        CACHE_wbInvL2Wait();  // Wait for the Writeback-Invalidate operation to complete.

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvL2Wait (void)
{
    /* Wait for the Writeback & Invalidate operation to complete. */
    while (CSL_FEXT(hCache->L2WIWC, CGEM_L2WIWC_WC) != 0);        
}

/** ============================================================================
 *   @n@b CACHE_wbInvL2
 *
 *   @b Description
 *   @n This function is used to write back and invalidate the contents of the L2 Cache.
 *      Although the block size can be specified in the number of bytes, 
 *      the cache controller operates on whole cache lines. To prevent unintended 
 *      behavior "blockPtr" should be aligned on the cache line size and "byteCnt" 
 *      should be a multiple of the cache line size.
 *
 *   @b Arguments
      @verbatim
          blockPtr      Address of the block which is to be written back & invalidated
          byteCnt       Size of the block to be written back & invalidated.
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache are being written back & invalidated.   
 *
 *   @b Writes
 *   @n CGEM_L2WIBAR_ADDR,CGEM_L2WIWC_WC
 *
 *   @b Example
 *   @verbatim
        Uint8* ptr_buffer;
        
        // Invalidate the contents of the buffer. 
        CACHE_wbInvL2(ptr_buffer, 100, CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvL2 (
    void*       blockPtr,
    Uint32      byteCnt,
    CACHE_Wait  wait
)
{
    /* Setup the block address and length */
    hCache->L2WIBAR = CSL_FMK(CGEM_L2WIBAR_ADDR, (Uint32)blockPtr);
    hCache->L2WIWC  = CSL_FMK(CGEM_L2WIWC_WC,    (Uint32)((byteCnt+3)>>2));
 
    /* Determine if we need to wait for the operation to complete. */
    if (wait == CACHE_WAIT)
        CACHE_wbInvL2Wait();
    else if (wait == CACHE_FENCE_WAIT)
        _mfence();
}

/** ============================================================================
 *   @n@b CACHE_wbAllL2Wait
 *
 *   @b Description
 *   @n This function is used to wait for the L2 Writeback & invalidate operation 
 *      to complete. This API should be used only if the CACHE_wbAllL2 was called 
 *      with the CACHE_NOWAIT argument.   
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbAllL2(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache have been written back
 *
 *   @b Reads
 *   @n CGEM_L2WB_C=0
 *
 *   @b Example
 *   @verbatim
        
        // Writeback the contents of the L2 Cache.
        CACHE_wbAllL2(CACHE_NOWAIT);
        
        // Wait for the operation to complete. 
        CACHE_wbAllL2Wait(); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbAllL2Wait (void)
{
    /* Wait for the writeback operation to complete. */
    while (CSL_FEXT(hCache->L2WB, CGEM_L2WB_C) == 1);    
}

/** ============================================================================
 *   @n@b CACHE_wbAllL2
 *
 *   @b Description
 *   @n This function is used to write back all the contents of the L2 Cache.  
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache are being written back.   
 *
 *   @b Writes
 *   @n CGEM_L2WB_C=1
 *
 *   @b Example
 *   @verbatim
        
        // Writeback the contents of the L2 Cache. 
        CACHE_wbAllL2(CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbAllL2 (CACHE_Wait wait)
{
    CSL_FINS (hCache->L2WB, CGEM_L2WB_C, 1);
    
    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_wbAllL2Wait();
}

/** ============================================================================
 *   @n@b CACHE_invAllL2Wait
 *
 *   @b Description
 *   @n This function is used to wait for the L2 Invalidate operation to complete. 
 *      This API should be used only if the CACHE_invAllL2 was called with the 
 *      CACHE_NOWAIT argument.   
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_invAllL2(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache have been invalidated   
 *
 *   @b Reads
 *   @n CGEM_L2INV_I=0
 *
 *   @b Example
 *   @verbatim
        
        // Invalidate the contents of the L2 Cache.
        CACHE_invAllL2(CACHE_NOWAIT);
        
        // Wait for the operation to complete. 
        CACHE_invAllL2Wait(); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invAllL2Wait (void)
{
    /* Wait for the invalidate operation to complete. */
    while (CSL_FEXT(hCache->L2INV, CGEM_L2INV_I) == 1);    
}

/** ============================================================================
 *   @n@b CACHE_invAllL2
 *
 *   @b Description
 *   @n This function is used to invalidate all the contents of the L2 Cache.  
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache are being invalidated.   
 *
 *   @b Writes
 *   @n CGEM_L2INV_I=1 
 *
 *   @b Example
 *   @verbatim
        
        // Invalidate the contents of the L2 Cache. 
        CACHE_invAllL2(CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_invAllL2 (CACHE_Wait wait)
{
    CSL_FINS (hCache->L2INV, CGEM_L2INV_I, 1);
    
    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_invAllL2Wait();    
}

/** ============================================================================
 *   @n@b CACHE_wbInvAllL2Wait
 *
 *   @b Description
 *   @n This function is used to wait for the L2 Writeback and Invalidate 
 *      operation to complete. This API should be used only if the CACHE_wbInvAllL2 was 
 *      called with the CACHE_NOWAIT argument.   
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CACHE_wbInvAllL2(wait=CACHE_NOWAIT) must be called.
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache have been invalidated and written back
 *
 *   @b Reads
 *   @n CGEM_L2WBINV_C=0
 *
 *   @b Example
 *   @verbatim
        
        // Writeback & Invalidate the contents of the L2 Cache.
        CACHE_wbInvAllL2(CACHE_NOWAIT);
        
        // Wait for the operation to complete. 
        CACHE_wbInvAllL2Wait(); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvAllL2Wait (void)
{
    /* Wait for the writeback-invalidate operation to complete. */
    while (CSL_FEXT(hCache->L2WBINV, CGEM_L2WBINV_C) == 1);    
}

/** ============================================================================
 *   @n@b CACHE_wbInvAllL2
 *
 *   @b Description
 *   @n This function is used to writeback and invalidate all the contents of the L2 Cache.  
 *
 *   @b Arguments
      @verbatim
          wait          Indicates if the call should block or not.
     @endverbatim 
 *
 *   <b> Return Value </b>  
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  The contents of the L2 Cache are being written back & invalidated.   
 *
 *   @b Writes
 *   @n CGEM_L2WBINV_C=1
 *
 *   @b Example
 *   @verbatim
        
        // Invalidate the contents of the L2 Cache. 
        CACHE_wbInvAllL2(CACHE_WAIT); 

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CACHE_wbInvAllL2 (CACHE_Wait wait)
{
    CSL_FINS (hCache->L2WBINV, CGEM_L2WBINV_C, 1);
    
    /* Determine if we need to wait for the operation to complete. */
    if (wait)
        CACHE_wbInvAllL2Wait();    
}

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /*CSL_CACHEAUX_H_*/
