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



#ifndef _ROMPARSE_H
#define _ROMPARSE_H
/*************************************************************************************
 * FILE PURPOSE: Define boot data structures
 *************************************************************************************
 * FILE NAME: romparse.h
 *
 * DESCRIPTION: Defines the structures used to create the boot parameter
 *				table
 *
 *************************************************************************************/
 
/* tiboot.h defines the boot parameters structure that will be created */
#if defined(c6472)
 #include "tiboot_c6472.h"
#elif defined(c6474) || defined(c6474l)
 #include "tiboot_c6474.h"
#elif defined(c6455)
 #include "tiboot_c6455.h"
#elif defined(c6457)
 #include "tiboot_c6457.h"
#elif defined(c66x)
 #include "tiboot_c66x.h"
#elif defined(c665x)
 #include "tiboot_c665x.h"
#else
 #error invalid or missing device specification
#endif

/* Define the number of boot parameter tables that will be put on the rom */
#define NUM_BOOT_PARAM_TABLES   8

/* Define the size reserved for the PCI configuration table */
#define PCI_EEAI_PARAM_SIZE    0x20

/* Define a structure mapping the boot parameter table number to a program file
 * to an eeprom byte address */
#define MAX_FNAME_LEN        132
#define MAX_DATA_LEN_32bit   32768
typedef struct {
  char fname[MAX_FNAME_LEN];
  int  sizeBytes;
  unsigned int  addressBytes;
  unsigned int data[MAX_DATA_LEN_32bit];
  int  tag[NUM_BOOT_PARAM_TABLES];          /* identifies boot parameter tables which use this file */
  int  align;                               /* alignment requirements for the file */
} progFile_t;

/* Define the PCI parameter structure */
#define PCI_DATA_LEN_32bit  8
typedef struct {
  char fname[MAX_FNAME_LEN];
  int sizeBytes;
  int addressBytes;
  unsigned int data[PCI_DATA_LEN_32bit];
} pciFile_t;


/* Distinguish between a layout and pad */
#define PLT_PAD     10
#define PLT_FILE    11
typedef struct {
  int type;
  int index;
} plt_t;

/* Define a layout table. A layout table is a block of data which contains the addresses
 * of data files. Each address is 32 bits, with the upper 16 bits specifying the i2c 
 * id, the lower address the byte address of the 1st block in the table */
#define MAX_LAYOUTS         2
#define MAX_LAYOUT_FILES    8
typedef struct  {
  int nPlt;                      /* Number of elements in the plt array */
  plt_t plt[MAX_LAYOUT_FILES];   /* Index of each file/pad in order */
  
  unsigned int address;         /* I2c data address of the table */
  unsigned int dev_addr;        /* I2c device address of the table */
  int align;
} layout_t;


/* Pad section. The pad section creates a gap in the i2c memory map */
#define MAX_PADS        8
typedef struct  {
  int          id;
  unsigned int address;     /* I2C data address */
  unsigned int dev_addr;    /* I2C device address */
  unsigned int len;
} pad_t;


/* Layout/pad interleave. The rom specification must be in order, so this structure tracks
 * the arrangement of layouts and pads */
typedef struct
{
  int type;   /* Either PAD or LAYOUT */
  int index;  /* The array index for the pad/layout */
  
} padLayoutOrder_t;




#endif /* ROMPARSE_H */
