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



/* Create an ascii hex i2c data file */

#include <stdio.h>
#include <malloc.h>

unsigned onesComplementAdd (unsigned value1, unsigned value2)
{
  unsigned result;

  result = (unsigned)value1 + (unsigned)value2;

  result = (result >> 16) + (result & 0xFFFF); /* add in carry   */
  result += (result >> 16);                    /* maybe one more */
  result = (result & 0xffff);
  return (unsigned)result;

} /* end of beth_ones_complement_add() */


int asciiByte (unsigned char c)
{
  if ((c >= '0') && (c <= '9'))
    return (1);

  if ((c >= 'A') && (c <= 'F'))
    return (1);

  return (0);
}

int toNum (unsigned char c)
{
  if ((c >= '0') && (c <= '9'))
    return (c - '0');

  return (c - 'A' + 10);

}


void  stripLine (FILE *s)
{
  char iline[132];

  fgets (iline, 131, s);

}

/* Read a .b file. */
int readBFile (FILE *s, unsigned char *data, unsigned maxSize)
{
  unsigned char x, y;
  int byteCount = 0;

  /* Strip the 1st two lines */
  stripLine (s);
  stripLine (s);

  for (;;) {

    /* read the 1st ascii char */
    do  {
      x = fgetc (s);
      if (x == (unsigned char)EOF)
        return (byteCount);

    } while (!asciiByte(x));

    /* Read the next ascii char */
    y = fgetc (s);
    if (y == (unsigned char)EOF)
      return (byteCount);
    if (asciiByte(y))
      data[byteCount++] = (toNum(x) << 4) | toNum (y);

    if (byteCount >= maxSize)  {
      fprintf (stderr, "Max input array size exceeded\n");
      return (-1);
    }

  }


}


int copyBlock (unsigned char *source, int idx, int maxSize, unsigned char *dest, int count)
{
  int i;

  for (i = 0; i < count; i++)  {
    if (idx >= maxSize)
      break;
    dest[i] = source[idx++];
  }

  return (i);

}

void blockCheckSum (unsigned char *block, int blockSize)
{
  unsigned checksum = 0;
  unsigned value;
  int i;

  if (blockSize & 0x0001)  {
    fprintf (stderr, "program requires an even blocksize\n");
    exit (-1);
  }

  for (i = 0; i < blockSize; i += 2) {
    value = (block[i] << 8) | block[i+1];
    checksum = onesComplementAdd (checksum, value);
  }

  /* Put the checksum into the block starting at byte 2. Use big endian */
  checksum = ~checksum;
  block[3] = checksum & 0xff;
  block[2] = (checksum >> 8) & 0xff;

}

#define SIZE	0x20000   /* max array size */

int main (int argc, char *argv[])
{
  FILE *strin;
  FILE *strout;

  unsigned char *dataSet1;
  unsigned char *dataSet2;

  unsigned char block[128];
  unsigned blockSize;

  unsigned pIn;
  unsigned pOut;

  int inSize;
  int i;

  /* Arg check */
  if (argc != 3)  {
    fprintf (stderr, "usage: %s infile outfile\n", argv[0]);
    return (-1);
  }

  /* Open the input file */
  strin = fopen (argv[1], "r");
  if (strin == NULL)  {
    fprintf (stderr, "%s: Could not open file %s for reading\n", argv[0], argv[1]);
    return (-1);
  }

  /* Allocate the two data set memories */
  dataSet1 = malloc (SIZE * sizeof (unsigned char));
  dataSet2 = malloc (SIZE * sizeof (unsigned char));
  if ((dataSet1 == NULL) || (dataSet2 == NULL))  {
    fprintf (stderr, "%s: Malloc failure\n", argv[0]);
    return (-1);
  }

  /* Read the data into the byte stream */
  if ((inSize = readBFile (strin, dataSet1, SIZE)) < 0)
    return (inSize);
  fclose (strin);

  /* Perform the i2c block formatting. The block size will be fixed at 128 bytes,
   * 2 bytes of length, 2 bytes checksum, 124 bytes of data. */
  pIn = 0;
  pOut = 0;

  do  {

    /* Copy the block, leave 4 bytes at the top */
    blockSize = copyBlock (dataSet1, pIn, inSize, &block[4], 124);
    pIn += blockSize; /* advance to next data in source */

    if (blockSize)  {
      blockSize += 4;   /* Add room for the header - big endian */
      block[1] = blockSize & 0xff;
      block[0] = (blockSize >> 8) & 0xff;
      block[2] = block[3] = 0;

      /* Checksum the block */
      blockCheckSum (block, blockSize);

      /* Copy the results to the destination block */
      if ((pOut + blockSize) >= SIZE)  {
        fprintf (stderr, "%s: destination array size exceeded\n", argv[0]);
        return (-1);
      }
      for (i = 0; i < blockSize; i++)
        dataSet2[pOut++] = block[i];
    }

  } while (blockSize == 128);


  /* Copy the resulting data set into the output file in ccs format */
  strout = fopen (argv[2], "w");
  if (strout == NULL)  {
    fprintf (stderr, "%s: Could not open %s for writing\n", argv[0], argv[2]);
    return (-1);
  }


  /* Write the two line header */
  fprintf (strout, "%c\n$A000000\n", (unsigned char)2);

  /* Write out the data */
  for (i = 0; i < pOut; i++)  {
    if ( ((i+1)%24) )
      fprintf (strout, "%02X ", dataSet2[i]);
    else
      fprintf (strout, "%02X\n", dataSet2[i]);
  }

  /* Write the close character */
  fprintf (strout, "\n%c", (unsigned char)3);

  fclose (strout);

  return (0);

}



    




