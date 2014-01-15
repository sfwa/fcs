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



/************************************************************************************
 * FILE PURPOSE: Convert a hex6x boot table file into the format required
 *               by the c6x chip boot loader. 
 ************************************************************************************
 * FILE NAME: bconvert.c
 *
 * DESCRIPTION: Converts a boot table. The boot table format is:
 *
 *    /--------------------------------------------------------\
 *    |              32 bit entry point address                |
 *    +--------------------------------------------------------+
 *    |              32 bit section byte count                 | \
 *    +--------------------------------------------------------+  \
 *    |         32 bit section byte start address              |   \
 *    +-------------+-------------+-------------+--------------+    repeat each section
 *    |  Data byte  |  Data byte  |  Data byte  |  Data byte   |   /
 *    +-------------+-------------+-------------+--------------+  /
 *    |  Data byte  |  Data byte  |  Data byte  |  Data byte   | /
 *    +-------------+-------------+-------------+--------------+ 
 *    |        32 bit zero byte count (end of boot table)      | 
 *    \--------------------------------------------------------/
 *
 *
 *  The C6x boot loader expects all 32 bit values to be in big endian
 *  (most significant byte arrives first). Data bytes are also arranged to be 32 bit
 *  big endian represented.
 *
 *  This program handles conversion of data values that are in sections that are
 *  not multiples of 4 bytes. For example, a program compiled in big endian format
 *  with a 1 byte section will have the following table entry (address 0x22222222), value
 *  0x33
 *
 *  00 00 00 01   22 22 22 22   33 00 00 00
 *
 *  The same file compiled with little endian mode will have the entry
 *
 *  00 00 00 01   22 22 22 22   00 00 00 33
 *
 *
 *  Since the boot loader has no idea what endianness the code was compiled for, this
 *  program performs an endian conversion on any section that does not have a length
 *  that is a multiple of 4 bytes, if the little endian option is specified. Nothing
 *  is changed for the big endian mode.
 *
 *  Invokation:
 *
 *  bconvert -be|-le [input_file] [output_file]
 *
 ***************************************************************************************/

#include "stdio.h"
#include "malloc.h"
#include "string.h"

/* Global variable storing the invokation name */
char *invok;

/* Error values */
enum {
  ERR_PARSE_TOO_MANY_ARGS = 1000,
  ERR_PARSE_NO_ENDIAN,
  ERR_PARSE_INPUT_OPEN_FAIL,
  ERR_PARSE_OUTPUT_OPEN_FAIL,
  ERR_READ_BFILE_INITIAL_MALLOC_FAIL,
  ERR_READ_BFILE_REALLOC_FAIL,
  ERR_VALUE32_SIZE_ERR,
  ERR_VALUE16_SIZE_ERR,
  ERR_DATA32_SIZE_ERR,
  ERR_REG32_PARSE_ERROR,
  ERR_DATA32_REMAIN_ERR 
};

enum {
  LITTLE,
  BIG
};

/************************************************************************************
 * FUNTION PURPOSE: Send an error string
 ************************************************************************************
 * DESCRIPTION: Prints an error to stderr
 ************************************************************************************/
void showErr (int errflag, int line)
{
  char *s;

  switch (errflag)  {
    case ERR_PARSE_TOO_MANY_ARGS:
       s = "Parse error: too many args specified";
       break;

    case ERR_PARSE_NO_ENDIAN:
       s = "Parse error: no endian mode specified";
       break;

    case ERR_PARSE_INPUT_OPEN_FAIL:
       s = "File error: failed to open specified input file";
       break;

    case ERR_PARSE_OUTPUT_OPEN_FAIL:
       s = "File error: Failed to open specified output file";
       break;

    case ERR_READ_BFILE_INITIAL_MALLOC_FAIL:
       s = "Memory error: Initial malloc call failed";
       break;

    case ERR_READ_BFILE_REALLOC_FAIL:
       s = "Memory error: Subsequent realloc call failed";
       break;

    case ERR_VALUE32_SIZE_ERR:
       s = "Data format error: End of data on 32 bit value read";
       break;

    case ERR_VALUE16_SIZE_ERR:
       s = "Data format error: End of data on 16 bit value read";
       break;

    case ERR_REG32_PARSE_ERROR:
       s = "Parse error: error parsing after reg32 arg";
       break;

    case ERR_DATA32_REMAIN_ERR:
       s = "Parse error: A remainder size greater then four was found";
       break;

    default:
       s = "Unspecified error";
       break;

  }

  fprintf (stderr, "%s: %s, line %d\n", invok, s, line);

} /* showErr */


/*************************************************************************************
 * FUNCTION PURPOSE: Check if a string is prefixed with "0x".
 *************************************************************************************
 * DESCRIPTION: Returns non-zero if the string begins with "0x"
 *************************************************************************************/
int isChex (char *s)
{
  if ((s[0] == '0') && (s[1] == 'x'))
    return (1);

  return (0);

} /* isChex */



/*************************************************************************************
 * FUNCTION PURPOSE: Parse the input parameters
 *************************************************************************************
 * DESCRIPTION: Checks for required args, opens source and destination streams.
 *************************************************************************************/
int parseit (int argc, char *argv[], FILE **fin, FILE **fout, int *endian)
{
  int inspec  = 0;
  int outspec = 0;
  int espec   = 0;
  int c       = 1;

  char *iname;
  char *oname;

  *endian = -1;

  /* Store the invokation name */
  invok = argv[0];

  while (c < argc)  {

    /* -be | -le */
    if (!espec)  {
      if (!strcmp (argv[c], "-be"))  {
        *endian = BIG;
        espec = 1;
        c += 1;
        continue;
      } else if (!strcmp (argv[c], "-le"))  {
        *endian = LITTLE;
        espec = 1;
        c += 1;
        continue;
      }
    }


    /* input file */
    if (!inspec)  {
      inspec = 1;
      iname = argv[c];
      c += 1;
      continue;
    }

    /* output file */
    if (!outspec)  {
      outspec = 1;
      oname = argv[c];
      c += 1;
      continue;
    }

    /* Don't know what to do with the arg */
    return (ERR_PARSE_TOO_MANY_ARGS);

  }


  /* Make sure endian is known */
  if (!espec) 
    return (ERR_PARSE_NO_ENDIAN);

  /* Open input file if not stdin */
  if (inspec)  {
    *fin = fopen (iname, "r");
    if (*fin == NULL)
      return (ERR_PARSE_INPUT_OPEN_FAIL);
  }

  /* Open output file if not stdin */
  if (outspec)  {
    *fout = fopen (oname, "w");
    if (*fout == NULL)
      return (ERR_PARSE_OUTPUT_OPEN_FAIL);
  }
 
  return (0);

} /* parseit */



/***************************************************************************************
 * FUNCTION PURPOSE: Check if data is ascii
 ***************************************************************************************
 * DESCRIPTION: Returns 1 if a byte is 0-9, a-f, 0 otherwise
 ***************************************************************************************/
int asciiByte (unsigned char c)
{
  if ((c >= '0') && (c <= '9'))
    return (1);

  if ((c >= 'A') && (c <= 'F'))
    return (1);

  return (0);
} /* asciiByte */

/**************************************************************************************
 * FUNCTION PURPOSE: Returns the binary equivalent of an ascii byte
 **************************************************************************************
 * DESCRIPTION: Conversion from ascii to binary
 **************************************************************************************/
int toNum (unsigned char c)
{
  if ((c >= '0') && (c <= '9'))
    return (c - '0');

  return (c - 'A' + 10);

} /* toNum */


/**********************************************************************************
 * FUNCTION PURPOSE: Read a line from a file, toss it
 **********************************************************************************
 * DESCRIPTION: Reads a line, including the newline character, and throws it away.
 **********************************************************************************/
void  stripLine (FILE *s)
{
  char iline[132];

  fgets (iline, 131, s);

} /* stripLine */

/************************************************************************************
 * FILE PURPOSE: Read the hex55 data file
 ************************************************************************************
 * DESCRIPTION: Reads the input data file. Strips the first two lines, reads
 *              the byte stream.
 ************************************************************************************/
#define MALLOC_BLOCK_SIZE   512000
unsigned char *readBFile (FILE *fin, unsigned *n, int *errcode)
{
  unsigned char *d;
  unsigned allocSize;
  unsigned m;
  unsigned char x, y;

  /* Create a block of data */
  allocSize = MALLOC_BLOCK_SIZE;
  d = malloc (allocSize * sizeof (unsigned char));
  if (d == NULL)  {
    *errcode = ERR_READ_BFILE_INITIAL_MALLOC_FAIL;
    if (fin != stdin)
      fclose (fin);
    return (NULL);
  }

  /* Strip the 1st two lines */
  stripLine (fin);
  stripLine (fin);

  *errcode = 0;
  m = 0;

  for (;;)  {

    /* Read the 1st ascii char */
    do  {
      x = fgetc (fin);
      if (x == (unsigned char)EOF)  {
        *errcode = 0;
        *n = m;
        if (fin != stdin)
          fclose (fin);
        return (d);
      }
    } while (!asciiByte(x));

    /* Read the next ascii char */
    y = fgetc(fin);
    if (y == (unsigned char)EOF)  {
      *errcode = 0;
      *n = m;
      if (fin != stdin)
        fclose (fin);
      return (d);
    }

    /* Convert the two characters into a byte */
    if (asciiByte(y))
      d[m++] = (toNum(x) << 4) | toNum(y);

    /* Verify memory bounds */
    if (m >= allocSize)  {
      allocSize += MALLOC_BLOCK_SIZE;
      d= realloc (d, allocSize);
      if (d== NULL)  {
        *errcode = ERR_READ_BFILE_REALLOC_FAIL;
        if (fin != stdin)
          fclose (fin);
        return (NULL);
      }
    }

  } /* end for */

} /* readBFile */

/**************************************************************************************
 * FUNCTION PURPOSE: converts four bytes into an unsigned value
 **************************************************************************************
 * DESCRIPTION: Converts bytes to a value
 **************************************************************************************/
unsigned value32bitAdd (int endian, unsigned char *data, unsigned n, unsigned *p, int *errflag, unsigned add)
{
  unsigned v;
  unsigned w;
  unsigned q;

  /* Verify that there are 4 values still in the character array */
  if ((*p + 4) > n)  {
    *errflag = ERR_VALUE32_SIZE_ERR;
    return (0);
  }

  /* Store the original pointer */
  q = w = *p;

  v = (unsigned)data[w+0] << 24 |
      (unsigned)data[w+1] << 16 |
      (unsigned)data[w+2] <<  8 |
      (unsigned)data[w+3] <<  0 ; 

  *errflag = 0;

  /* Add any additional value */
  v = v + add;

  /* Write the data back in big endian format */
  data[q+0] = (v >> 24) & 0xff;
  data[q+1] = (v >> 16) & 0xff;
  data[q+2] = (v >>  8) & 0xff;
  data[q+3] = (v >>  0) & 0xff;

  *p = q+4;
  return (v);

} /* value32bitAdd */

/**************************************************************************************
 * FUNCTION PURPOSE: converts four bytes into an unsigned value
 **************************************************************************************
 * DESCRIPTION: Converts bytes to a value, depending on the endian configuration.
 **************************************************************************************/
unsigned value32bit (int endian, unsigned char *data, unsigned n, unsigned *p, int *errflag)
{
  return (value32bitAdd (endian, data, n, p, errflag, 0));

} /* value32bit */




/*********************************************************************************
 * FUNCTION PURPOSE: Convert up to four bytes to big endian 
 *********************************************************************************
 * DESCRIPTION: Data bytes are converted. 
 *********************************************************************************/
#define SWAP(x,y,z)  (z)=(x);(x)=(y);(y)=(z)
void data32bit (int endian, unsigned char *data, unsigned n, unsigned *p, unsigned m, int *errflag)
{

  /* Calculate the number of bytes to convert, limited to four bytes */
  if (m > 4)
    m = 4;

  /* return an error if there are not enough values in the array */
  if ((*p + m) > n)  {
    *errflag = ERR_DATA32_SIZE_ERR;
    return;
  }

  /* Clear the error flag */
  *errflag = 0;


  /* The data is always already in big endian, there is nothing to do but advance the pointer */
  *p += m;

} /* data32bit */

void remain32bit (int endian, unsigned char *data, unsigned n, unsigned *p, unsigned m, int *errflag)
{
  unsigned w;
  unsigned char h;

  /* return an error if the size is greater then or equal to 4 */
  if (m >= 4)  {
    *errflag = ERR_DATA32_REMAIN_ERR;
    return;
  }

  /* return an error if there are not enough values in the array */
  if ((*p + m) > n)  {
    *errflag = ERR_DATA32_SIZE_ERR;
    return;
  }

  w = *p;

  /* Only swap if endianness is little */
  if (endian == LITTLE)  {
    /* Swap the next four bytes */
    SWAP(data[w+0], data[w+3], h);
    SWAP(data[w+1], data[w+2], h);
  }

  /* Update the full four elements */
  *p = *p + 4;


} /* remain32bit */


/*********************************************************************************
 * FUNCTION PURPOSE: Convert 2 bytes into an unsigned value
 *********************************************************************************
 * DESCRIPTION: Converts the next two bytes into an unsigned value based on
 *              the endian configuration.
 *********************************************************************************/
unsigned value16bit (int endian, unsigned char *data, unsigned n, unsigned *p, int *errflag)
{
  unsigned v;
  unsigned q;

  /* Verify that there are 4 values still in the character array */
  if ((*p + 2) > n)  {
    *errflag = ERR_VALUE16_SIZE_ERR;
    return (0);
  }

  /* Store the original pointer */
  q = *p;

  /* convert based on endianness. For little endain the 16 bit words are actually
   * big endian, but the bytes in those words are not */
  if (endian == BIG)  {
    v = data[(*p)++] <<  8 |
        data[(*p)++] <<  0 ;

  }  else  {
    v = data[(*p)++] <<  0 |
        data[(*p)++] <<  8 ;
  }

  *errflag = 0;

  /* Write the data back in big endian format */
  data[q++] = (v >>  8) & 0xff;
  data[q++] = (v >>  0) & 0xff;

  return (v);

} /* value16bit */

/**************************************************************************************
 * FUNCTION PURPOSE: Writes a 16 bit value into the array
 **************************************************************************************
 * DESCRIPTION: Writes a big endian 16 bit value, increments the array pointer.
 **************************************************************************************/
void write16bit (unsigned value, unsigned char *data, unsigned *p)
{
  data[(*p)++] = (value >> 8) & 0xff;
  data[(*p)++] = (value >> 0) & 0xff;

} /* write16bit */


/*************************************************************************************
 * FUNCTION PURPOSE: Write the output file
 *************************************************************************************
 * DESCRIPTION: Writes the resulting output.
 *************************************************************************************/
void writeBFile (FILE *fout, unsigned char *data, unsigned n)
{
  unsigned i;

  /* Write the two line header */
  fprintf (fout, "%c\n$A000000\n", (unsigned char)2);

  for (i = 0; i < n; i++)  {
    if ( ((i+1)%24)  )
      fprintf (fout, "%02X ", data[i]);
    else
      fprintf (fout, "%02X\n", data[i]);
  }

  /* Write the close character */
  fprintf (fout, "\n%c", (unsigned char)3);

  if (fout != stdout)
    fclose (fout);

} /* writeBFile */
  

/**************************************************************************************
 * FUNCTION PURPOSE: Main 
 **************************************************************************************
 * DESCRIPTION: Provides the top level program flow.
 **************************************************************************************/
int main (int argc, char *argv[])
{
  FILE *fin;            /* input stream  */
  FILE *fout;           /* output stream */

  unsigned char *data;  /* The data set  */
  unsigned n;           /* Data set size */
  unsigned p;           /* Data index    */
  unsigned v;           /* Data value    */
  unsigned n32;         /* Number of bytes that form complete 32 bit values */
  unsigned r32;         /* Number of bytes remaining (0-3) */

  int endian;           /* Endian          */  
  int errflag;          /* error indicator */
  int i, j;             /* loop var        */
  int origRegs;         /* original reg count */
  int shift;            /* data shift amount  */


  /* Parse the input */
  if (errflag = parseit (argc, argv, &fin, &fout, &endian))  {
    showErr (errflag, __LINE__);
    return (-1);
  }

  /* Read the raw data file */
  data = readBFile (fin, &n, &errflag);
  if (data == NULL)  {
    showErr (errflag, __LINE__);
    return(-1);
  }

  /* Parse the sections */
  p = 0;

  /* The entry point */
  v = value32bit (endian, data, n, &p, &errflag);
  if (errflag)  {
    showErr (errflag, __LINE__);
    return(-1);
  }

  /* The sections */
  do  {
  
    /* Get the section byte count */   
    v = value32bit (endian, data, n, &p, &errflag);
    if (errflag)  {
      showErr (errflag, __LINE__);
      return(-1);
    }

    if (v)  {
      /* Convert the start address (adjusts the array index) */
      value32bit (endian, data, n, &p, &errflag);
      if (errflag)  {
        showErr (errflag, __LINE__);
        return(-1);
      }
    }

    /* Determine how many bytes form complete 32 bit fields, and how many are left over */
    n32 = v & 0xfffffffc;
    r32 = v & 0x00000003;

    /* Convert the data to big endian format  */
    for (i = 0; i < n32; i += 4)  {
      data32bit (endian, data, n, &p, v-i, &errflag);
      if (errflag)  {
        showErr (errflag, __LINE__);
        return (-1);
      }
    }

    /* Convert any remaining bytes. */
    if (r32)  {
      remain32bit (endian, data, n, &p, r32, &errflag);
      if (errflag)  {
        showErr (errflag, __LINE__);
        return (-1);
      }
    }

  } while (v);

  /*  Write out the data file */
  writeBFile (fout, data, n);

  /* Return resources */
  free (data);

  return (0); 

}
    






 





