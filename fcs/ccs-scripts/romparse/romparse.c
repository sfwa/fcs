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



/*************************************************************************************
 * FILE PURPOSE: Create an I2C rom with multiple boot parameter sections and
 *               programs
 *************************************************************************************
 * FILE NAME: romparse.c
 *
 * DESCRIPTION: Creates a ccs hex file which contains the i2c eprom boot parameter 
 *              tables as well as any code.
 *
 *************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include "rparse.tab.h"
#include "romparse.h"

/*************************************************************************************
 * Definition: fixed i2c map locations
 *************************************************************************************/
#define PCI_PARAM_BASE  (NUM_BOOT_PARAM_TABLES * 0x80)
#define DATA_BASE       (PCI_PARAM_BASE + PCI_EEAI_PARAM_SIZE)


/*************************************************************************************
 * Declaration: The base address of the i2c ROM being created. This is just
 *              the I2C bus address. The default is 0x50
 *************************************************************************************/
int i2cRomBase = IBL_CFG_I2C_MAP_TABLE_DATA_BUS_ADDR;

/*************************************************************************************
 * Declaration: The flex input file is assigned based on the command line
 *************************************************************************************/
extern FILE *yyin;

/*************************************************************************************
 * Declaration: Keep track of lines in the parse
 *************************************************************************************/
int line = 1;

/*************************************************************************************
 * Declaration: currentType identifies the current parse mode, either SECTION
 *              or LAYOUT.
 *************************************************************************************/
int currentType;

/*************************************************************************************
 * Declaration: The boot parameter tables. The current table is copied into position
 *              when the section parse is complete.
 *************************************************************************************/
BOOT_PARAMS_T boot_params[NUM_BOOT_PARAM_TABLES];
BOOT_PARAMS_T current_table;
int           current_file;       /* Identifies the program file in the current table */
int           ctable_index = -1;  /* Destination of current table */
int           max_index    =  0;  /* maximum table index, used for compacting output */

/************************************************************************************
 * Declaration: Layout tables. 
 ************************************************************************************/
layout_t  layouts[MAX_LAYOUTS];   /* Array of layout structures */                
int       currentLayout;          /* Currently active layout    */

/************************************************************************************
 * Declaration: Pads
 ************************************************************************************/
pad_t pads[MAX_PADS];             /* Array of pad structures */
int   currentPad;                 /* Currently active pad    */

/************************************************************************************
 * Declaration: The structure storing the program data files, and the number of
 *              programs used.
 ************************************************************************************/
progFile_t progFile[NUM_BOOT_PARAM_TABLES];
int        nProgFiles = 0;

/************************************************************************************
 * Declaration: The PCI parameter structure
 ************************************************************************************/
pciFile_t pciFile;
int       pciSet = 0;

/*************************************************************************************
 * Declaration: The array that tracks the ordering of pad and layout structures
 *************************************************************************************/
padLayoutOrder_t padLayoutOrder[MAX_PADS+MAX_LAYOUTS];
int currentPL = 0;

/*************************************************************************************
 * Declaration: The next free address in the ROM for general data usage. For the
 *              start address there is room for the initial boot parameter tables, 
 *              plus room for the PCI eeai config.
 *************************************************************************************/
int romBase = DATA_BASE;


/*************************************************************************************
 * Declaration: Args passed in from the command line
 *************************************************************************************/
char *inputFile;
int   compact = 0;

/*************************************************************************************
 * Declaration: The value used to fill gaps in the file. For some devices this
 *              value must be set to 0xff so subsequent writing to these gaps will work
 *************************************************************************************/
unsigned char fillVal = 0;


/*************************************************************************************
 * FUNCTION PURPOSE: flex/bison required support functions.
 *************************************************************************************
 * DESCRIPTION: yyerror and yywrap are required to support flex and/or bison parsing
 *              of the parameter description file. yyerror directly reports the
 *              error, yywrap is unused.
 *************************************************************************************/
void yyerror (char *s)
{
  fprintf (stderr, "flex/bison error is %s at line %d\n", s, line);
} /* yyerror */

void yywrap (void)
{
} /* yywrap */

/*************************************************************************************
 * FUNCTION PURPOSE: Initialize a boot table
 *************************************************************************************
 * DESCRIPTION: Sets a boot parameter table to 0s
 *************************************************************************************/
void initTable (BOOT_PARAMS_T *current_table)
{
  memset (current_table, 0, sizeof(BOOT_PARAMS_T));
}

/*************************************************************************************
 * FUNCTION PURPOSE: Initialize the program data file table
 *************************************************************************************
 * DESCRIPTION: The size and tags are all setup
 *************************************************************************************/
void initProgFile (void)
{
  int i, j;

  for (i = 0; i < NUM_BOOT_PARAM_TABLES; i++)  {
    progFile[i].sizeBytes = 0;
    progFile[i].align     = 0;

    for (j = 0; j < NUM_BOOT_PARAM_TABLES; j++)
      progFile[i].tag[j] = -1;

  }

}


/*************************************************************************************
 * FUNCTION PURPOSE: Set the currently active parse type
 *************************************************************************************
 * DESCRIPTION: Indicates if the subsequent parameters belong to a section or
 *              a layout
 *************************************************************************************/
void rBegin (int blockType)
{
    currentType = blockType;
}

/*************************************************************************************
 * FUNCTION PURPOSE: Initialize a layout structure
 *************************************************************************************
 * DESCRIPTION: The layout is set to the initial state
 *************************************************************************************/
void initLayout (layout_t *cl)
{

  cl->nPlt     = 0;
  cl->dev_addr = i2cRomBase;
  cl->address  = 0;
  cl->align    = 0;

}

/*************************************************************************************
 * FUNCTION PURPOSE: Complete a layout
 *************************************************************************************
 * DESCRIPTION: The parser has found a complete layout specification. Complete
 *              a layout structure
 *************************************************************************************/
void setLayout (void)
{
  int i;
  int currentAlign;
  int newAlign;

  for (i = 0; i < layouts[currentLayout].nPlt; i++)  {

    if (layouts[currentLayout].plt[i].type == PLT_FILE)  {

      currentAlign = progFile[layouts[currentLayout].plt[i].index].align;
      newAlign     = layouts[currentLayout].align;

      if (newAlign > currentAlign)
        progFile[layouts[currentLayout].plt[i].index].align = newAlign;
    }

  }


  padLayoutOrder[currentPL].type  = LAYOUT;
  padLayoutOrder[currentPL].index = currentLayout;
  currentPL += 1;
    
  currentLayout += 1;      /* Advance to the next layout */

  if (currentLayout < MAX_LAYOUTS)
    initLayout (&layouts[currentLayout]);

}    

/*************************************************************************************
 * FUNCTION PURPOSE: Initialize a pad structure
 *************************************************************************************
 * DESCRIPTION: A pad structure is set to the default state
 *************************************************************************************/
void initPad (pad_t *p)
{
  p->id       = -1;
  p->address  = 0;
  p->dev_addr = i2cRomBase;
  p->len      = 0;
}


/**************************************************************************************
 * FUNCTION PURPOSE: Complete a pad
 **************************************************************************************
 * DESCRIPTION: The parser has found a complete pad specification. Complete the pad
 *              structure
 **************************************************************************************/
void setPad (void)
{

  padLayoutOrder[currentPL].type  = PAD;
  padLayoutOrder[currentPL].index = currentPad;
  currentPL += 1;

  currentPad += 1;

  if (currentPad < MAX_PADS)
    initPad (&pads[currentPad]);

}


/*************************************************************************************
 * FUNCTION PURPOSE: Complete a section
 *************************************************************************************
 * DESCRIPTION: The parser has detected a complete section. Copy the section into
 *              it's correct table location.
 *************************************************************************************/
void section (void)
{
  int i;

  /* It's an error if no section value has been declared */
  if (ctable_index == -1)  {
    fprintf (stderr, "romparse: the section did not have a boot paramter index specified\n");
    exit (-1);
  }

  /* Make sure the table is in range */
  if (ctable_index >= NUM_BOOT_PARAM_TABLES)  {
    fprintf (stderr, "romparse: The section index is too large (max %d)\n", NUM_BOOT_PARAM_TABLES-1);
    exit (-1);
  }

  /* The length must be set. Currently this program only supports I2C mode, so the
   * length is fixed */

  current_table.common.length   = 30;

  #if (defined(c66x) || defined(c665x))
    if (current_table.common.boot_mode == BOOT_MODE_SPI)
        current_table.common.length   = sizeof(BOOT_PARAMS_SPI_T);
    else
        current_table.common.length   = sizeof(BOOT_PARAMS_I2C_T);
  #endif
       
  current_table.common.checksum = 0;

  /* Copy the table */
  memcpy (&boot_params[ctable_index], &current_table, sizeof (BOOT_PARAMS_T));
  initTable (&current_table);

  /* Track the maximum table index */
  if (ctable_index > max_index)
    max_index = ctable_index;

  /* If the section referenced a data file, link the data file back to this section */
  if (current_file >= 0)  {
    for (i = 0; i < NUM_BOOT_PARAM_TABLES; i++)  {
      if (progFile[current_file].tag[i] < 0) {
        progFile[current_file].tag[i] = ctable_index;
        break;
	  }
    }
  }

  ctable_index = -1;
  current_file = -1;

} /* section */

/***************************************************************************************
 * FUNCTION PURPOSE: Open a ccs hex file and read in the data.
 ***************************************************************************************
 * DESCRIPTION: Reads a ccs hex format data file, loads the file into the
 *              next program file structure. Returns the index of the just loaded
 *              table.
 ***************************************************************************************/
int openProgFile (char *fname)
{
  FILE *str;
  int a, b, c, d, e;
  int i;
  char iline[132];

  /* Store the file name */
  strcpy (progFile[nProgFiles].fname, fname);

  /* Open the data file */
  str = fopen (fname, "r");
  if (str == NULL)  {
    fprintf (stderr, "romparse: Could not open file %s\n", fname);
    exit (-1);
  }

  /* Read the one line ccs header. The length field in terms of lines */
  fgets (iline, 132, str);
  sscanf (iline, "%x %x %x %x %x", &a, &b, &c, &d, &e);
  progFile[nProgFiles].sizeBytes = e * 4; /* Length was in 4 byte words */

  /* Read in the data */
  for (i = 0; i < e; i++)  {
    fgets (iline, 132, str);
    sscanf (&(iline[2]), "%x", &((progFile[nProgFiles]).data[i]));
  }

  fclose (str);

  i = nProgFiles;
  nProgFiles += 1;

  return (i);

} /* openProgFile */

/***************************************************************************************
 * FUNCTION PURPOSE: Load the PCI paramter section
 ***************************************************************************************
 * DESCRIPTION: Loads the PCI parameter section and stores it in the rom. */
int setPciParams (char *fname)
{
  FILE *str;
  int a, b, c, d, e;
  int i;
  char iline[132];
  char *z;

  if (pciSet)  {
    fprintf (stderr, "romparse: PCI parameters specified more then once\n");
    exit (-1);
  }

  /* the input string still contains the quotes. Remove them here */
  z = &fname[1];
  *strchr (z, '"') = '\0';

  /* Store the file name */
  strcpy (pciFile.fname, z);

  /* Open the data file */
  str = fopen (z, "r");
  if (str == NULL)  {
    fprintf (stderr, "romparse: Could not open pci file %s\n", fname);
    exit (-1);
  }

  /* The address of the pci params is currently fixed */
  pciFile.addressBytes = PCI_PARAM_BASE;

  /* Read the one line ccs header. The length field is in terms of lines */
  fgets (iline, 132, str);
  sscanf (iline, "%x %x %x %x %x", &a, &b, &c, &d, &e);
  pciFile.sizeBytes = e * 4;  /* Convert length to bytes */

  /* Read in the data */
  for (i = 0; i < e; i++)  {
    fgets (iline, 132, str);
    sscanf (&(iline[2]), "%x", &(pciFile.data[i]));
  }

  pciSet = 1;

  return (0);

} /* setPciParams */

 

/***************************************************************************************
 * FUNCTION PURPOSE: Store an assignment
 ***************************************************************************************
 * DESCRIPTION: Stores an assigned value into the current boot parameter table
 ***************************************************************************************/
void assignKeyVal (int field, int value)
{

  switch (currentType)   {


    case SECTION:


      switch (field)  {

        case BOOT_MODE:        current_table.common.boot_mode = value;
                               break;

        case PARAM_INDEX:      ctable_index = value;
                               break;

        case OPTIONS:          current_table.i2c.options = value;
                               break;

        case MULTI_I2C_ID:     current_table.i2c.multi_i2c_id = value;
                               break;

        case MY_I2C_ID:        current_table.i2c.my_i2c_id = value;
                               break;

        case CORE_FREQ_MHZ:    
                               #if (defined(c66x) || defined(c665x))
                                   if (current_table.common.boot_mode == BOOT_MODE_SPI)  {
                                        current_table.spi.cpuFreqMhz = value;
                                        break;
                                   }
                               #endif
        
                               current_table.i2c.core_freq_mhz = value;
                               break;

        case I2C_CLK_FREQ_KHZ: current_table.i2c.i2c_clk_freq_khz = value;
                               break;

        case NEXT_DEV_ADDR:    current_table.i2c.next_dev_addr = value;
                               break;
                               

        case NEXT_DEV_ADDR_EXT: current_table.i2c.next_dev_addr_ext = value;
                                break;

        case ADDRESS_DELAY:    current_table.i2c.address_delay = value;
                               break;

#if (!defined(c6455) && !defined(c66x) && !defined(c665x))
        case SWPLL:            current_table.i2c.swPll = value;
                               break;
#endif

#if (defined(c66x) || defined(c665x))
        case SWPLL_PREDIV:    current_table.common.swPllCfg_lsw &= 0x00ff;
                              current_table.common.swPllCfg_lsw |= ((value & 0xff) << 16);
                              break;

        case SWPLL_MULT:      current_table.common.swPllCfg_msw &= 0xc000;
                              current_table.common.swPllCfg_msw |= (value & 0x3fff);
                              break;

        case SWPLL_POSTDIV:   current_table.common.swPllCfg_lsw &= 0xff00;
                              current_table.common.swPllCfg_lsw |= (value & 0xff);
                              break;

        case SWPLL_FLAGS:     current_table.common.swPllCfg_msw &= 0x3fff;
                              current_table.common.swPllCfg_msw |= ((value & 0x3) << 14);
                              break;

#endif

        case DEV_ADDR_EXT:     current_table.i2c.dev_addr_ext = value;
                               break;

        case DEV_ADDR:         current_table.i2c.dev_addr = value;
                               break;


#if (defined(c66x) || defined(c665x))
        case N_PINS:           current_table.spi.nPins = value;
                               break;

        case MODE:             current_table.spi.mode = value;
                               break;

        case C2T_DELAY:        current_table.spi.c2tdelay = value;
                               break;

        case BUS_FREQ_MHZ:     current_table.spi.busFreqMhz = value;
                               break;

        case BUS_FREQ_KHZ:     current_table.spi.busFreqKhz = value;
                               break;

        case ADDR_WIDTH:       current_table.spi.addrWidth = value;
                               break;

        case CSEL:             current_table.spi.csel = value;
                               break;

#endif

        default:
            fprintf (stderr, "romparse: Invalid assignment in section specification (line %d)\n", line);
            break;

      }

      break;


    case LAYOUT:

      if (currentLayout >= MAX_LAYOUTS)  { 
        fprintf (stderr, "romparse: Too many layout sections (max = %d)\n", MAX_LAYOUTS);
        exit (-1);
      }


      switch (field)  {

        case DEV_ADDR_EXT: layouts[currentLayout].dev_addr = value;
                           break;

        case DEV_ADDR:     layouts[currentLayout].address = value;
                           break;

        case ALIGN:        layouts[currentLayout].align = value;
                           break;

        case PAD_FILE_ID:   if (layouts[currentLayout].nPlt >= MAX_LAYOUT_FILES)  {
                              fprintf (stderr, "romparse: line %d: number of layout entries exceeds maximum of %d\n", line, MAX_LAYOUT_FILES);
                              exit (-1);
                            }
                            layouts[currentLayout].plt[layouts[currentLayout].nPlt].type  = PLT_PAD;
                            layouts[currentLayout].plt[layouts[currentLayout].nPlt].index = value;
                            layouts[currentLayout].nPlt += 1;
                            break;


        default:
            fprintf (stderr, "romparase: Invalid assignment in layout specification (line %d)\n", line);
            break;

      }
      break;


    case PAD:

      if (currentPad >= MAX_PADS)  {
        fprintf (stderr, "romparse: Too many pad sections (max = %d)\n", MAX_PADS);
        exit (-1);
      }

      switch (field)  {

        case DEV_ADDR_EXT: pads[currentPad].dev_addr = value;
                           break;

        case DEV_ADDR:  pads[currentPad].address = value;
                        break;

        case LENGTH:    pads[currentPad].len = value;
                        break;

        case PAD_FILE_ID: pads[currentPad].id = value;
                          break;

        default:
          fprintf (stderr, "romparse: Invalid assignment in pad specificaiton (line %d)\n", line);
          break;

      }
      break;

   }


} /* assignKeyVal */


/*******************************************************************************
 * FUNCTION PURPOSE: Parse a string input.
 *******************************************************************************
 * DESCRIPTION: Takes a string input. Currently only the i2c exe file name can be 
 *              assigned a string. 
 *******************************************************************************/
void assignKeyStr (int value, char *y)
{
  int i;
  char *z;

  /* The special case of a 0 (plus the quotes) length string means an empty entry for a layout */
  if (strlen(y) == 2)  {

    if (currentType == LAYOUT)  {
      if (layouts[currentLayout].nPlt <= MAX_LAYOUT_FILES)  {
        layouts[currentLayout].plt[layouts[currentLayout].nPlt].type  = PLT_FILE;
        layouts[currentLayout].plt[layouts[currentLayout].nPlt].index = -1;
        layouts[currentLayout].nPlt += 1;

      }  else  {
        fprintf (stderr, "romparse: line %d: Max number (%d) of layout specification exceeded\n", line, MAX_LAYOUT_FILES);
      }
    }  else
         fprintf (stderr, "romparse: Number of layout sections exceeded (max = %d)\n", MAX_LAYOUTS);

    return;
  }


  /* the input string still contains the quotes. Remove them here */
  z = &y[1];
  *strchr (z, '"') = '\0';

  /* Check if the file name is already open */
  for (i = 0; i < nProgFiles; i++)  {

    if (!strcmp (z, progFile[i].fname))  {

      /* Found a match */

      if (currentType == SECTION)  {

        current_file = i;

        if (current_table.i2c.dev_addr_ext == 0)
          current_table.i2c.dev_addr_ext = i2cRomBase;  /* hard coded to i2c rom slave address */

      }  else  {   /* LAYOUT */

        if (currentLayout < MAX_LAYOUTS)  {
          if (layouts[currentLayout].nPlt <= MAX_LAYOUT_FILES)  {
            layouts[currentLayout].plt[layouts[currentLayout].nPlt].type  = PLT_FILE;
            layouts[currentLayout].plt[layouts[currentLayout].nPlt].index = i;
            layouts[currentLayout].nPlt += 1;

          }  else  {
            fprintf (stderr, "romparse: line %d: Max number (%d) of layout specification exceeded\n", line, MAX_LAYOUT_FILES);
          }
        }  else
          fprintf (stderr, "romparse: Number of layout sections exceeded (max = %d)\n", MAX_LAYOUTS);

      }
        

      return;
    }

  }

  /* Open and read the ccs file, set the ROM address */
  i = openProgFile (z);
  if (i >= 0) {

    if (currentType == SECTION)  {

      current_file = i;
      if (current_table.i2c.dev_addr_ext == 0)
          current_table.i2c.dev_addr_ext = i2cRomBase;

    }  else  {  /* LAYOUT */
        
        if (currentLayout < MAX_LAYOUTS)  {
          if (layouts[currentLayout].nPlt <= MAX_LAYOUT_FILES)  {
            layouts[currentLayout].plt[layouts[currentLayout].nPlt].type  = PLT_FILE;
            layouts[currentLayout].plt[layouts[currentLayout].nPlt].index = i;
            layouts[currentLayout].nPlt += 1;

          }  else  {
            fprintf (stderr, "romparse: line %d: Max number (%d) of layout specification exceeded\n", line, MAX_LAYOUT_FILES);
          }
        }  else
          fprintf (stderr, "romparse: Number of layout sections exceeded (max = %d)\n", MAX_LAYOUTS);

    }
      
        
  }

} /* assignKeyStr */

/************************************************************************************
 * FUNCTION PURPOSE: Put a 32 bit value into the i2c image memory
 ************************************************************************************
 * DESCRIPTION: The 32 bit value is placed in memory in big endian format. The
 *              new offset is returned (4 bytes more then the input offset)
 ************************************************************************************/
unsigned int imageWord (unsigned int base, unsigned int start, unsigned char *image, unsigned int value)
{
    image[base-start+0] = (value >> 24) & 0xff;
    image[base-start+1] = (value >> 16) & 0xff;
    image[base-start+2] = (value >>  8) & 0xff;
    image[base-start+3] = (value >>  0) & 0xff;

    return (base + 4);

}

/************************************************************************************
 * FUNCTION PURPOSE: Create a 32 bit value from the image array
 ************************************************************************************
 * DESCRIPTION: A 32 bit word in big endian format is created
 ************************************************************************************/
unsigned int formWord (unsigned int p, unsigned char *image)
{
  unsigned int v;

  v = (image[p+0] << 24) |
      (image[p+1] << 16) |
      (image[p+2] <<  8) |
      (image[p+3] <<  0) ;

  return (v);

}

/************************************************************************************
 * FUNCTION PURPOSE: Pad the image array
 ************************************************************************************
 * DESCRIPTION: Byte (value 0) are added to the image to reach the desired address
 *              The desired address is returned.
 ************************************************************************************/
unsigned int imagePad (unsigned int base, unsigned int start, unsigned char *image, unsigned int desired)
{
  int i;

  if (desired < base)  {
    fprintf (stderr, "romparse: Padd to %d requested, but current base (%d) is already past this point\n",
             desired, base);
    exit (-1);
  }

  for (i = base; i < desired; i++)
    image[i-start] = fillVal;

  return (desired);

}

/************************************************************************************
 * FUNCTION PURPOSE: Opens and writes the output file
 ************************************************************************************
 * DESCRIPTION: Creates the output file in ccs format.
 ************************************************************************************/
void createOutput (void)
{
  FILE *str;
  int   totalLenBytes;
  int   i, j, k;
  int   nTables, len;
  int   i2cRomStart;
  unsigned int value, v1, v2;
  unsigned int base;
  unsigned char *image;

  str = fopen ("i2crom.ccs", "w");
  if (str == NULL)  {
    fprintf (stderr, "romparse: Could not open output file i2crom.ccs for writing\n");
    exit (-1);
  }

  /* Compact the i2c eeprom to use the minimum memory possible */
  base    = (i2cRomBase << 16) + PCI_PARAM_BASE;
  nTables = NUM_BOOT_PARAM_TABLES; 

  if ((compact != 0) && (pciSet == 0))  {
    nTables = max_index + 1;
    base    = (i2cRomBase << 16) + (nTables * 0x80);  /* The number of parameter tables * size of a parameter table */
  }

  if (pciSet)
    base = base + PCI_EEAI_PARAM_SIZE;


  /* Change the layout index value for pad mapping to a true array index value.
   * Also reflect the device address from the layout into the pad */
  for (i = 0; i < currentLayout; i++)  {
    
    for (j = 0; j < layouts[i].nPlt; j++)  {

      if (layouts[i].plt[j].type == PLT_PAD)  {

        for (k = 0; k < currentPad; k++)  {

          if (layouts[i].plt[j].index == pads[k].id)  {
            layouts[i].plt[j].index = k;
            pads[k].dev_addr = layouts[i].dev_addr;
          }
        }
      }
    }
  }

  /* Pad, layout tables */
  for (i = 0; i < currentPL; i++)  {

    j = padLayoutOrder[i].index;

    if (padLayoutOrder[i].type == LAYOUT)  {

      /* Determine the size of the table. Four bytes for each file, plus the 4 byte header */ 
      v1 = (layouts[j].nPlt * 4) + 4;

      v2 = (layouts[j].dev_addr << 16) + layouts[j].address;

      if (v2 == 0)
        base = base + v1;

      else  {

        if (base > v2)  {
          fprintf (stderr, "romparse: fatal error - layout block %d specified a start address of 0x%04x\n", j, (layouts[j].dev_addr << 16) + layouts[j].address);
          fprintf (stderr, "          but this conflicts with the base mapping (ends at 0x%04x)\n", base);
          exit (-1);
        }

        base = v2 + v1;  /* new base is the base plus the size */


      }  
    }  else  {   /* Otherwise this is a pad */

      if (base > ((pads[j].dev_addr << 16) + pads[j].address))  {
        fprintf (stderr, "romparse: fatal error - pad block %d specified a start address of 0x%04x\n", j, (pads[j].dev_addr << 16) + pads[j].address);
        fprintf (stderr, "          but this conflicts with the base mapping (ends at 0x%04x)\n", base);
        exit (-1);
      }

      base = (pads[j].dev_addr << 16) + pads[j].address + pads[j].len;

    }
  }

  for (i = 0; i < NUM_BOOT_PARAM_TABLES; i++)  {
    if (progFile[i].align > 0)  
      base = ((base + progFile[i].align - 1) / progFile[i].align) * progFile[i].align;
    progFile[i].addressBytes = base;
    base = base + progFile[i].sizeBytes;
  }

  /* Setup the base program file addresses. If a parameter set has
   * been tagged it means that this is an i2c program load */
  for (i = 0; i < NUM_BOOT_PARAM_TABLES; i++)  {
    for (j = 0; j < NUM_BOOT_PARAM_TABLES; j++)  {
      if (progFile[i].tag[j] >= 0)  {
        
        #if (defined(c66x) || defined(c665x))
          if (boot_params[progFile[i].tag[j]].common.boot_mode == BOOT_MODE_SPI)  {
            boot_params[progFile[i].tag[j]].spi.read_addr_lsw = (progFile[i].addressBytes & 0xffff);
            boot_params[progFile[i].tag[j]].spi.read_addr_msw = (progFile[i].addressBytes  >> 16) & 0xffff;
            continue;
          }
        #endif

        boot_params[progFile[i].tag[j]].i2c.dev_addr = (progFile[i].addressBytes & 0xffff);
      }
    }
  }

  /* Round up the size to a multiple of 4 bytes to fit into a ccs data file */
  base = (base + 3) & ~3;

  i2cRomStart = (i2cRomBase << 16);
      
  /* The total length of the i2c eeprom is now stored in base */
  /* Write out the ccs header */
  fprintf (str, "1651 1 10000 1 %x\n", (base - i2cRomStart) >> 2);

  /* Create the image in memory */
  image = malloc ((base - i2cRomStart) * sizeof (unsigned char));
  if (image == NULL)  {
    fprintf (stderr, "romparse: malloc failed creating the output image\n");
    exit (-1);
  }

  memset (image, fillVal, (base - i2cRomStart));

  /* Write out the boot parameter tables. 0x80 bytes will be written out.
   * There are 16 bits in every parameter field, which is why the index
   * is from 0 to 0x40 */
  base = i2cRomBase << 16;
  for (i = 0; i < nTables; i++)  {
    for (j = 0; j < (0x80 >> 1); j += 2)  {
      v1 = boot_params[i].parameter[j];
      v2 = boot_params[i].parameter[j+1];
      value = (v1 << 16) | v2;
      base = imageWord (base, i2cRomStart, image, value);
    }
  }

  /* Write out the PCI parameter base. If none was included then zeros will be
   * written out */
  if (pciSet)  {
    for (i = 0; i < PCI_DATA_LEN_32bit; i++)  {
      base = imageWord (base, i2cRomStart, image, pciFile.data[i]);
    }
  }


  /* Layout sections */
  for (i = 0; i < currentLayout; i++)  {

    v1 = (layouts[i].dev_addr << 16) + layouts[i].address;

    /* subtract out device address bits */
    if (v1 > 0)
      base  = imagePad (base, i2cRomStart, image, v1);

    len   = (layouts[i].nPlt * 4) + 4;

    /* Write out the block size and checksum */
    base = imageWord(base, i2cRomStart, image, len << 16);

    for (j = 0; j < layouts[i].nPlt; j++)  {
        
        if (layouts[i].plt[j].type == PLT_FILE)  {
          if (layouts[i].plt[j].index == -1)  {
            base = imageWord (base, i2cRomStart, image, 0xffffffff);
          } else {
            base = imageWord (base, i2cRomStart, image, progFile[layouts[i].plt[j].index].addressBytes);
          } 
        }  else  {
          v1 = pads[layouts[i].plt[j].index].dev_addr;
          v2 = pads[layouts[i].plt[j].index].address;
          base = imageWord (base, i2cRomStart, image, (v1 << 16) + v2);
        }

    }

  }
                                

  /* Write out each of the program files */
  for (i = 0; i < nProgFiles; i++)  {

    v1 = progFile[i].addressBytes;
    base = imagePad (base, i2cRomStart, image, v1);

    for (j = 0; j < progFile[i].sizeBytes >> 2; j++)
      base = imageWord (base, i2cRomStart, image, (progFile[i]).data[j]);
  }

  /* Write out the data file */
  for (i = 0; i < base - i2cRomStart; i += 4) 
    fprintf (str, "0x%08x\n", formWord (i, image));

  free (image);

  /* Close the output file */
  fclose (str);

} /* createOutput  */

/************************************************************************************
 * FUNCTION PURPOSE: Initialize the pci paramter table
 ************************************************************************************
 * DESCRIPTION: Zeros the pci parameters
 ************************************************************************************/
void initPciParams (void)
{
  memset (&pciFile, 0, sizeof(pciFile_t));
} /* initPciParams */


/************************************************************************************
 * FUNCTION PURPOSE: Read an integer value from a string
 ************************************************************************************
 * DESCRIPTION: A decimal or hex value is scanned
 ************************************************************************************/
int readVal (char *s)
{
  int ret;

  if ((s[0] == '0') && (s[1] == 'x'))
    sscanf (&s[2], "%x", &ret);
  else
    sscanf (s, "%d", &ret);

  return (ret);

}
  

/************************************************************************************
 * FUNCTION PURPOSE: Parse the input arguments.
 ************************************************************************************
 * DESCRIPTION: Returns -1 on invalid args
 ************************************************************************************/
int parseIt (int argc, char *argv[])
{
  int i;

  if (argc < 2)  {
     fprintf (stderr, "usage: %s [-compact] [-rom_base x] [-fill <fillval>] inputfile\n", argv[0]);
     return (-1);
  }

  inputFile = NULL;  

  for (i = 1; i < argc;  )  {

    if (!strcmp (argv[i], "-compact"))  {
      compact = 1;
      i += 1;

    } else if (!strcmp (argv[i], "-rom_base"))  {
      i2cRomBase = readVal (argv[i+1]);
      i += 2;

    } else if (!strcmp (argv[i], "-fill"))  {
      fillVal = readVal (argv[i+1]);
      i += 2;

    } else  {

      if (inputFile != NULL)  {
        fprintf (stderr, "usage: %s [-compact] [-rom_base x] [-fill <fillval>] inputfile\n", argv[0]);
        return (-1);
      }

      inputFile = argv[i];
      i += 1;
    }
  }

  return (0);

}



/************************************************************************************
 * FUNCTION PURPOSE: main function
 ************************************************************************************
 * DESCRIPTION: Performs the processing sequence.
 ************************************************************************************/
int main (int argc, char *argv[])
{
  int i;

  /* Initialize the tables */
  for (i = 0; i < NUM_BOOT_PARAM_TABLES; i++)
    initTable(&boot_params[i]);

  initTable (&current_table);
  current_file = -1;

  /* Initialize the program file structure */
  initProgFile ();

  /* Initialize the PCI param table */
  initPciParams ();

  /* Initialize the layout structures */
  currentLayout = 0;
  initLayout (&layouts[currentLayout]);

  /* Parse the input parameters */
  if (parseIt (argc, argv))
    return (-1);

  
  yyin = fopen (inputFile, "r");
  if (yyin == NULL)  {
    fprintf (stderr, "%s: could not open file %s\n", argv[0], inputFile);
    return (-1);
  }

  /* Parse the input description file */
  yyparse();


  /* Create the output file */
  createOutput ();

  return (0);
}



