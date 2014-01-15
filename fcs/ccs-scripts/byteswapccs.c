/********************************************************************************************
 * FILE PURPOSE: Byte swap a CCS data file
 ********************************************************************************************
 * FILE NAME: byteswapccs.c
 *
 * DESCRIPTION: A CCS file is read in, the data is byte swapped, and a CCS file is written out
 *
 *  usage: byteswapccs infile outfile
 *
 ********************************************************************************************/
#include <stdio.h>
#include <stdlib.h>


int main (int argc, char *argv[])
{
    FILE *fin, *fout;
    unsigned int v, b0, b1, b2, b3;
    int a, b, c, d, n;
    int i;
    char iline[132];


    if (argc != 3)  {
        fprintf (stderr, "usage: %s infile outfile\n", argv[0]);
        return (-1);
    }


    fin = fopen (argv[1], "r");
    if (fin == NULL)  {
        fprintf (stderr, "%s: Could not open input file %s\n", argv[0], argv[1]);
        return (-1);
    }

    fout = fopen (argv[2], "w");
    if (fout == NULL)  {
        fprintf (stderr, "%s: Could not open output file %s\n", argv[0], argv[2]);
        fclose (fin);
        return (-1);
    }


    /* Read the CCS data file header, write it out unchanged */
    fgets (iline, 132, fin);
    sscanf (iline, "%x %x %x %x %x", &a, &b, &c, &d, &n);
    fputs (iline, fout);

    for (i = 0; i < n; i++)  {
        fgets (iline, 132, fin);
        sscanf (&iline[2], "%x", &v);

        b0 = (v >> 24) & 0xff;
        b1 = (v >> 16) & 0xff;
        b2 = (v >>  8) & 0xff;
        b3 = (v >>  0) & 0xff;

        v = (b3 << 24) | (b2 << 16) | (b1 <<8) | b0;
        fprintf (fout, "0x%08x\n", v);
    }

    fclose (fout);
    fclose (fin);

    return (0);

}
