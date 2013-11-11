/* ============================================================================
 * Copyright (c) Texas Instruments Incorporated 2008, 2009
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
 * @file csl_idmaAux.h
 *
 * @brief 
 *  API Auxilary header file for IDMA CSL. It gives the definitions of the 
 *  query, status & control functions.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
 *  @n   Use of this software is controlled by the terms and conditions found 
 *  @n   in the license agreement under which this software has been supplied.
 *  ===========================================================================
 *  \par  
 */

#ifndef CSL_IDMAAUX_H_
#define CSL_IDMAAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl_idma.h>

/** @addtogroup CSL_IDMA_FUNCTION
 @{ */

/** ============================================================================
 *   @n@b CSL_IDMA_chan0TransferData
 *
 *   @b Description
 *   @n This function configures IMDA Channel 0 to perform a transfer between 
 *      Internal Memory (L1P, L1D, L2) and Configuration Space(CFG) based on the 
 *      inputs to the function. This API initiates a transfer and if needed waits 
 *      till the transfer completes based on the "bWaitToCompletion" flag value passed. 
 * 
 *   @b Arguments
     @verbatim
        idmaChan0Config     CSL_IDMA_IDMA0CONFIG structure pointer that holds
                            information relevant to perform a IDMA Channel 0 configuration
                            space transfer.
        bWaitToCompletion   Boolean flag that indicates if this function should 
                            wait till the transfer is completed or should
                            just return after initiating one. When set to 1, this
                            API waits and returns after requested transfer is
                            complete. Otherwise, returns immediately after configuring
                            appropriate registers.                             
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None. 
 *
 *   <b> Post Condition </b>
 *   @n The appropriate IDMA Channel 0 registers are configured to initiate the
 *      CFG space transfer operation. Furthermore, if bWaitToCompletion argument is set
 *      to 1, the API polls on the IDMA0 Status register to wait until the
 *      transfer completes. 
 *      
 *   @b Writes
 *   @n CGEM_IDMA0_MASK_MASK,
 *      CGEM_IDMA0_SOURCE_SOURCEADDR,
 *      CGEM_IDMA0_DEST_DESTADDR,
 *      CGEM_IDMA0_COUNT_INT, 
 *      CGEM_IDMA0_COUNT_COUNT
 *
 *   @b Example
 *   @verbatim

        #define MASK    0xFFFFFF0F

        // Align various arrays to a word address, in internal L2
        #pragma DATA_SECTION(src, "ISRAM");
        #pragma DATA_ALIGN(src,      32);
        #pragma DATA_ALIGN(dst2,     32);

        //  Static 128 byte array of data in "src" with known test pattern.   
        unsigned int src[32] = {
            0xDEADBEEF, 0xFADE0003, 0x5AA51C3A, 0x03036BA3,
            0x0000ABCD, 0x00001234, 0x00005670, 0x00005678,
            0x000003BE, 0x0000760F, 0x9675A800, 0xABCDEF12,
            0xEEEECDEA, 0x01234567, 0x00000000, 0xFEEDFADE,
            0x0, 0x0,   0x0, 0x0,   0x0, 0x0,   0x0, 0x0,
            0x0, 0x0,   0x0, 0x0,   0x0, 0x0,   0x0, 0x0        
        } ;

        // EDMA MMR Space Address
        unsigned int * dst2 = (unsigned int *)0x02704000;

        CSL_IDMA_IDMA0CONFIG    idmaChan0Config;
        Uint32                  bWaitEnable = 1;
    
        mask = MASK;
        idmaChan0Config.mask        =   mask;
        idmaChan0Config.source      =   src;
        idmaChan0Config.destn       =   dst2;
        idmaChan0Config.count       =   0;  // transfer one 32-byte window 
        idmaChan0Config.intEnable   =   1;  // Enable CPU interrupt on completion.
        CSL_IDMA_chan0TransferData (&idmaChan0Config, bWaitEnable);  
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan0TransferData (
    CSL_IDMA_IDMA0CONFIG*   idmaChan0Config,
    Uint32                  bWaitToCompletion
)
{
    volatile Uint32  pend;
    
    if( bWaitToCompletion )
    {
        /* Make sure that there are no pending transfers before using 
         * this channel. This is done by reading bit "1" of the status 
         * register.                                                    
         */
        while( (pend = ( CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_PEND) |
                         CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_ACTV)
                       )
                )        
             ); 
    }
    
    /*  Poke in "mask", "src", "dst" and "count" with the correct          
     *  count and interrupt flag on.                       
     */
    CSL_FINS (hIdma->IDMA0_MASK, CGEM_IDMA0_MASK_MASK, idmaChan0Config->mask);     
    hIdma->IDMA0_SOURCE	=	(Uint32)idmaChan0Config->source;
    hIdma->IDMA0_DEST	=	(Uint32)idmaChan0Config->destn;
    hIdma->IDMA0_COUNT 	=   CSL_FMK (CGEM_IDMA0_COUNT_INT, idmaChan0Config->intEnable) |
                            CSL_FMK (CGEM_IDMA0_COUNT_COUNT, idmaChan0Config->count);        

    /* Wait till the transfer is complete. */
    if( bWaitToCompletion )
    {
        while( (pend	=	( CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_PEND) |
                         	  CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_ACTV)
                       		)
                )        
             ); 
    
    }
    
    return;
}

/** ============================================================================
 *   @n@b CSL_IDMA_chan0Wait
 *
 *   @b Description
 *   @n This function waits until all previous transfers for IDMA Channel
 *      0 have been completed by making sure that both active and pend  
 *      bits are zero. These are the two least significant bits of the  
 *      status register for the channel.                                
 *
 *      Waits until previous transfers have completed for IDMA channel 0
 *      before returning.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n @a CSL_IDMA_chan0TransferData(bWaitToCompletion=0) must be called
 *      to setup an IDMA Channel 0 data transfer before this API is called.
 *
 *   <b> Post Condition </b>
 *   @n None
 *      
 *   @b Reads
 *   @n CGEM_IDMA0_STAT_PEND, 
 *      CGEM_IDMA0_STAT_ACTV
 *
 *   @b Affects
 *   @n CGEM_IDMA0_STAT_PEND=0, 
 *      CGEM_IDMA0_STAT_ACTV=0
 *
 *   @b Example
 *   @verbatim

        #define MASK    0xFFFFFF0F

        // Align various arrays to a word address, in internal L2
        #pragma DATA_SECTION(src, "ISRAM");
        #pragma DATA_ALIGN(src,      32);
        #pragma DATA_ALIGN(dst2,     32);

        //  Static 128 byte array of data in "src" with known test pattern.   
        unsigned int src[32] = {
            0xDEADBEEF, 0xFADE0003, 0x5AA51C3A, 0x03036BA3,
            0x0000ABCD, 0x00001234, 0x00005670, 0x00005678,
            0x000003BE, 0x0000760F, 0x9675A800, 0xABCDEF12,
            0xEEEECDEA, 0x01234567, 0x00000000, 0xFEEDFADE,
            0x0, 0x0,   0x0, 0x0,   0x0, 0x0,   0x0, 0x0,
            0x0, 0x0,   0x0, 0x0,   0x0, 0x0,   0x0, 0x0        
        } ;

        // EDMA MMR Space Address
        unsigned int * dst2 = (unsigned int *)0x02704000;

        CSL_IDMA_IDMA0CONFIG    idmaChan0Config;
        Uint32                  bWaitEnable = 1;
    
        mask = MASK;
        idmaChan0Config.mask        =   mask;
        idmaChan0Config.source      =   src;
        idmaChan0Config.destn       =   dst2;
        idmaChan0Config.count       =   0;  // transfer one 32-byte window 
        idmaChan0Config.intEnable   =   1;  // Enable CPU interrupt on completion.
        CSL_IDMA_chan0TransferData (&idmaChan0Config, bWaitEnable);        
        
        CSL_IDMA_chan0Wait ();
        
        ...
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan0Wait (
        void
)
{
    volatile Uint32  pend;
    
    while( (pend = ( CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_PEND) |
                     CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_ACTV)
                   )
           )        
         ); 
    
    return;
}


/** ============================================================================
 *   @n@b CSL_IDMA_chan0GetStatus
 *
 *   @b Description
 *   @n This function waits until all previous transfers for IDMA Channel
 *      0 have been completed by making sure that both active and pend  
 *      bits are zero. These are the two least significant bits of the  
 *      status register for the channel.                                
 *
 *      Waits until previous transfers have completed for IDMA channel 1
 *      before returning.
 *
 *   @b Arguments
     @verbatim
        idmaChanStatus      CSL_IDMA_STATUS structure pointer that needs to
                            be populated with iDMA Channel 0 transfer status.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None. 
 *
 *   <b> Post Condition </b>
 *   @n None
 *      
 *   @b Reads
 *   @n CGEM_IDMA0_STAT_PEND, 
 *      CGEM_IDMA0_STAT_ACTV
 *
 *   @b Example
 *   @verbatim
        
        CSL_IDMA_STATUS     idma1Status;
    
        CSL_IDMA_chan0GetStatus (&idma1Status);
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan0GetStatus (
        CSL_IDMA_STATUS*    idmaChanStatus
)
{
    idmaChanStatus->isPending	= 	CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_PEND);
    idmaChanStatus->isActive  	=	CSL_FEXT(hIdma->IDMA0_STAT, CGEM_IDMA0_STAT_ACTV);    
    
    return;
}


/** ============================================================================
 *   @n@b CSL_IDMA_chan1TransferData
 *
 *   @b Description
 *   @n This function initiates a GEM local memory "block transfer" operation 
 *      using the IDMA Channel 1. This API assumes that the source and destination 
 *      addresses passed in the "idmaChan1Config" argument are both in GEM's internal
 *      memory, i.e., L1P, L1D, L2 or CFG memories. Transfers from addresses
 *      that are not in the internal memory will raise an exception. This API 
 *      initiates a block transfer and if needed waits till the transfer completes
 *      based on the "bWaitToCompletion" flag value passed. 
 *
 *   @b Arguments
     @verbatim
        idmaChan1Config     CSL_IDMA_IDMA1CONFIG structure pointer that holds
                            information relevant to perform a IDMA Channel 1 block 
                            transfer.
        bWaitToCompletion   Boolean flag that indicates if this function should 
                            wait till the block transfer is completed or should
                            just return after initiating one. When set to 1, this
                            API waits and returns after requested block transfer is
                            complete. Otherwise, returns immediately after configuring
                            appropriate registers.                             
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None. 
 *
 *   <b> Post Condition </b>
 *   @n The appropriate IDMA Channel 1 registers are configured to initiate the
 *      data transfer operation. Furthermore, if bWaitToCompletion argument is set
 *      to 1, the API polls on the IDMA1 Status register to wait until the
 *      transfer completes. 
 *      
 *   @b Writes
 *   @n CGEM_IDMA1_SOURCE_SOURCEADDR,
 *      CGEM_IDMA1_DEST_DESTADDR,
 *      CGEM_IDMA1_COUNT_PRI, 
 *      CGEM_IDMA1_COUNT_INT, 
 *      CGEM_IDMA1_COUNT_FILL, 
 *      CGEM_IDMA1_COUNT_COUNT
 *
 *   @b Example
 *   @verbatim

        #pragma DATA_ALIGN(src1,      8);
        #pragma DATA_ALIGN(dst1,     8);

        // Static 80 byte array of data in "src1" with known test pattern
        Uint32 src1[20] =
        {
            0xDEADBEEF, 0xFADEBABE, 0x5AA51C3A, 0xD4536BA3,
            0x5E69BA23, 0x4884A01F, 0x9265ACDA, 0xFFFF0123,
            0xBEADDABE, 0x234A76B2, 0x9675ABCD, 0xABCDEF12,
            0xEEEECDEA, 0x01234567, 0x00000000, 0xFEEDFADE,
            0x0A1B2C3D, 0x4E5F6B7C, 0x5AA5ECCE, 0xFABEFACE
        }; 
        Uint32                  dst1[20];               
        CSL_IDMA_IDMA1CONFIG    idmaChan1Config;
        Uint32                  bWaitEnable = 1;        
        
        idmaChan1Config.source = src1;
        idmaChan1Config.destn = dst1;
        idmaChan1Config.intEnable = 1;
        idmaChan1Config.priority = IDMA_PRI_7;
        idmaChan1Config.count = DATA_COUNT_BYTES;
        
        CSL_IDMA_chan1TransferData (&idmaChan1Config, bWaitEnable); 
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan1TransferData (
    CSL_IDMA_IDMA1CONFIG*   idmaChan1Config,
    Uint32                  bWaitToCompletion
)
{
    volatile Uint32 pend;
    
    if( bWaitToCompletion )
    {
        /* Make sure that there are no pending transfers before using 
         * this channel. This is done by reading bit "1" of the status 
         * register.                                                    
         */
        while( (pend =  ( CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_PEND) |
                          CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_ACTV)
                        )
                )        
             ); 
    }
    
    /*  Poke in "src", "dst" and "count" with the correct          
     *  priority and interrupt flags on. Also disable the "fill" bit
     *  since this is a block transfer.                       
     */
   	hIdma->IDMA1_SOURCE	=	(Uint32)idmaChan1Config->source;
   	hIdma->IDMA1_DEST	=	(Uint32)idmaChan1Config->destn;
    hIdma->IDMA1_COUNT 	=   CSL_FMK(CGEM_IDMA1_COUNT_PRI, idmaChan1Config->priority) |
                            CSL_FMK(CGEM_IDMA1_COUNT_INT, idmaChan1Config->intEnable) |
                            CSL_FMK(CGEM_IDMA1_COUNT_FILL, ZERO) |
                            CSL_FMK(CGEM_IDMA1_COUNT_COUNT, idmaChan1Config->count);
    
    /* Wait till the transfer is complete. */
    if( bWaitToCompletion )
    {
        while( (pend 	=	( CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_PEND) |
                          	  CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_ACTV)
                        	)
               )                 
             );   
    }
    
    return;
}

/** ============================================================================
 *   @n@b CSL_IDMA_chan1FillData
 *
 *   @b Description
 *   @n This function initiates a GEM local memory "block fill" operation 
 *      using the IDMA Channel 1. Given a fill value in Channel configuration structure's
 *      source address, a valid destination address and the number of bytes to fill, this
 *      API fills the destination with the fill value specified. This API initiates a 
 *      block fill and if needed waits till the transfer completes based on the 
 *      "bWaitToCompletion" flag value passed. 
 *
 *   @b Arguments
     @verbatim
        idmaChan1Config     CSL_IDMA_IDMA1CONFIG structure pointer that holds
                            information relevant to perform a IDMA Channel 1 block 
                            fill operation.
        bWaitToCompletion   Boolean flag that indicates if this function should 
                            wait till the block fill is completed or should
                            just return after initiating one. When set to 1, this
                            API waits and returns after requested block transfer is
                            complete. Otherwise, returns immediately after configuring
                            appropriate registers.                             
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None. 
 *
 *   <b> Post Condition </b>
 *   @n The appropriate IDMA Channel 1 registers are configured to initiate the
 *      block fill operation. Furthermore, if bWaitToCompletion argument is set
 *      to 1, the API polls on the IDMA1 Status register to wait until the
 *      fill operation completes. 
 *      
 *   @b Writes
 *   @n CGEM_IDMA1_SOURCE_SOURCEADDR,
 *      CGEM_IDMA1_DEST_DESTADDR,
 *      CGEM_IDMA1_COUNT_PRI, 
 *      CGEM_IDMA1_COUNT_INT, 
 *      CGEM_IDMA1_COUNT_FILL, 
 *      CGEM_IDMA1_COUNT_COUNT
 *
 *   @b Example
 *   @verbatim
        
        #pragma DATA_ALIGN(dst1,     8);
        Uint32                  dst1[20];               
        CSL_IDMA_IDMA1CONFIG    idmaChan1Config;
        Uint32                  bWaitEnable = 1;        
       
        // Fill the destination with a value of "0xAAAABABA"
        idmaChan1Config.source = (Uint32 *) (0xAAAABABA);
        idmaChan1Config.destn = dst1;
        idmaChan1Config.intEnable = 1;
        idmaChan1Config.priority = IDMA_PRI_7;
        idmaChan1Config.count = 20;
        
        CSL_IDMA_chan1FillData (&idmaChan1Config, bWaitEnable);
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan1FillData (
    CSL_IDMA_IDMA1CONFIG*   idmaChan1Config,
    Uint32                  bWaitToCompletion
)
{
    volatile Uint32  pend;
    
    if( bWaitToCompletion )
    {
        /* Make sure that there are no pending transfers before using 
         * this channel. This is done by reading bit "1" of the status 
         * register.                                                    
         */
        while( (pend =  ( CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_PEND) |
                          CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_ACTV)
                        )
               )                 
             ); 
    }
    
    /*  Poke in "src", "dst" and "count" with the correct          
     *  priority and interrupt flags on. Also enable the "fill" bit
     *  since this is a block fill operation.                       
     */
    hIdma->IDMA1_SOURCE	=	(Uint32)idmaChan1Config->source;
    hIdma->IDMA1_DEST	=	(Uint32)idmaChan1Config->destn;
    hIdma->IDMA1_COUNT 	=   CSL_FMK(CGEM_IDMA1_COUNT_PRI, idmaChan1Config->priority) |
                            CSL_FMK(CGEM_IDMA1_COUNT_INT, idmaChan1Config->intEnable) |
                            CSL_FMK(CGEM_IDMA1_COUNT_FILL, ONE) |
                            CSL_FMK(CGEM_IDMA1_COUNT_COUNT, idmaChan1Config->count);        

    /* Wait till the transfer is complete. */
    if( bWaitToCompletion )
    {
        while( (pend =  ( CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_PEND) |
                          CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_ACTV)
                        )
               )                 
             );  
    }
    
    return;
}

/** ============================================================================
 *   @n@b CSL_IDMA_chan1Wait
 *
 *   @b Description
 *   @n This function waits until all previous transfers for IDMA Channel
 *      1 have been completed by making sure that both active and pend  
 *      bits are zero. These are the two least significant bits of the  
 *      status register for the channel.                                
 *
 *      Waits until previous transfers have completed for IDMA channel 1
 *      before returning.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n @a CSL_IDMA_chan1TransferData(bWaitToCompletion = 0) or 
 *      @a CSL_IDMA_chan1FillData(bWaitToCompletion = 0) must be called to
 *      setup IDMA Channel 1 data transfer before this API is called.
 *
 *   <b> Post Condition </b>
 *   @n None
 *      
 *   @b Reads
 *   @n CGEM_IDMA1_STAT_PEND, 
 *      CGEM_IDMA1_STAT_ACTV
 *
 *   @b Affects
 *   @n CGEM_IDMA1_STAT_PEND=0, 
 *      CGEM_IDMA1_STAT_ACTV=0 
 *
 *   @b Example
 *   @verbatim

        #pragma DATA_ALIGN(src1,      8);
        #pragma DATA_ALIGN(dst1,     8);

        // Static 80 byte array of data in "src1" with known test pattern
        Uint32 src1[20] =
        {
            0xDEADBEEF, 0xFADEBABE, 0x5AA51C3A, 0xD4536BA3,
            0x5E69BA23, 0x4884A01F, 0x9265ACDA, 0xFFFF0123,
            0xBEADDABE, 0x234A76B2, 0x9675ABCD, 0xABCDEF12,
            0xEEEECDEA, 0x01234567, 0x00000000, 0xFEEDFADE,
            0x0A1B2C3D, 0x4E5F6B7C, 0x5AA5ECCE, 0xFABEFACE
        }; 
        Uint32                  dst1[20];               
        CSL_IDMA_IDMA1CONFIG    idmaChan1Config;
        Uint32                  bWaitEnable = 0;        
        
        idmaChan1Config.source = src1;
        idmaChan1Config.destn = dst1;
        idmaChan1Config.intEnable = 1;
        idmaChan1Config.priority = IDMA_PRI_7;
        idmaChan1Config.count = DATA_COUNT_BYTES;
        
        CSL_IDMA_chan1TransferData (&idmaChan1Config, bWaitEnable); 

        CSL_IDMA_chan1Wait();
        
        ...

     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan1Wait (
        void
)
{
    volatile Uint32  pend;
    
    while( (pend =  ( CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_PEND) |
                      CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_ACTV)
                    )
            )                 
         );  
    
    return;
}


/** ============================================================================
 *   @n@b CSL_IDMA_chan1GetStatus
 *
 *   @b Description
 *   @n This function waits until all previous transfers for IDMA Channel
 *      1 have been completed by making sure that both active and pend  
 *      bits are zero. These are the two least significant bits of the  
 *      status register for the channel.                                
 *
 *      Waits until previous transfers have completed for IDMA channel 1
 *      before returning.
 *
 *   @b Arguments
     @verbatim
        idmaChanStatus      CSL_IDMA_STATUS structure pointer that needs to
                            be populated with iDMA Channel 1 transfer status.
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n None. 
 *
 *   <b> Post Condition </b>
 *   @n None
 *      
 *   @b Reads
 *   @n CGEM_IDMA1_STAT_PEND, 
 *      CGEM_IDMA1_STAT_ACTV
 *
 *   @b Example
 *   @verbatim
        CSL_IDMA_STATUS     idma1Status;
    
        CSL_IDMA_chan1GetStatus (&idma1Status);
        
     @endverbatim
 * =============================================================================
 */
CSL_IDEF_INLINE void CSL_IDMA_chan1GetStatus (
        CSL_IDMA_STATUS*    idmaChanStatus
)
{
    idmaChanStatus->isPending = CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_PEND);
    idmaChanStatus->isActive  = CSL_FEXT(hIdma->IDMA1_STAT, CGEM_IDMA1_STAT_ACTV);    
    
    return;
}


#ifdef __cplusplus
}
#endif

/** @} */

#endif /*CSL_IDMAAUX_H_*/
