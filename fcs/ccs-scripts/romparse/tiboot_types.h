/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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



#ifndef _TYPES_H
#define _TYPES_H
/*******************************************************************************
 * FILE PURPOSE:  DSP specific C type definitions.
 *******************************************************************************
 * FILE NAME: swpform.h  
 *
 * DESCRIPTION: Defines general use types for DSP.  
 *
 *  @file   types.h
 *
 *  @brief
 *      This file provides architecture specific typedefs
 *
 ******************************************************************************/


#include <stdlib.h>  /* Defines NULL */

/* a signed 16-bit integer */
typedef short int16;
typedef unsigned short uint16;

typedef int int32;
typedef unsigned int uint32;

typedef char char8;
typedef unsigned char uchar8;

typedef char int8;
typedef unsigned char uint8;


typedef unsigned char word;

typedef short BOOL;
typedef short bool;
typedef short Bool;
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE  1
#endif


/* tistdtypes.h types used from the emac driver */
typedef unsigned char Uint8;
typedef char Int8;
typedef int Int32;
typedef unsigned short Uint16;
typedef unsigned int Uint32;

/* TI boot types */
typedef unsigned char  UINT8;
typedef unsigned short UINT16;
typedef short          SINT16;
typedef unsigned int   UINT32;
typedef int            SINT32;


/* Types from evm driver */
typedef volatile unsigned int  VUint32;
typedef volatile unsigned char VUint8;

/* Types from the ethernet driver */
typedef unsigned int  IPN;

#endif /* types.h */
