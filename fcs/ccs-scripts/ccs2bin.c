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



/* Convert a ccs file to a raw binary file
 *
 *  usage: ccs2bin [-swap] ccsfile binfile
 */

#include <stdio.h>
#include <string.h>

unsigned int swap(unsigned int v)
{
    unsigned int w;

    w = (((v >> 24) & 0xff) <<  0)  |
        (((v >> 16) & 0xff) <<  8)  |
        (((v >>  8) & 0xff) << 16)  |
        (((v >>  0) & 0xff) << 24);

    return (w);

}
FILE *fin  = NULL;
FILE *fout = NULL;
int doswap = 0;

#define USAGE  "usage: %s [-swap] ccsfile binfile"

int parseit (int argc, char *argv[])
{
    int i;

    if ((argc != 3) && (argc != 4))  {
       fprintf (stderr, USAGE, argv[0]);
       return (-1);
    }

    for (i = 1; i < argc; i++)  {

        if (!strcmp (argv[i], "-swap"))
            doswap = 1;

        else if (fin == NULL)  {
            fin = fopen (argv[i], "r");
            if (fin == NULL)  {
                fprintf (stderr, "%s: Could not open file %s\n", argv[0], argv[i]);
                return (-1);
            }

        }  else if (fout == NULL)  {
            fout = fopen (argv[i], "wb");
            if (fout == NULL)  {
                fprintf (stderr, "%s: Could not open file %s\n", argv[0], argv[i]);
                fclose (fin);
                return (-1);
            }

        } else  {

            fprintf (stderr, USAGE, argv[0]);
            fclose (fout);
            fclose (fin);
            return (-1);
        }
    }

    return (0);

}





int main (int argc, char *argv[])
{
    unsigned int n;
    unsigned int v;
    unsigned int i;

    int a, b, c, d;


    char iline[132];

    if (parseit (argc, argv))
        return (-1);

    fgets (iline, 131, fin);
    sscanf (iline, "%x %x %x %x %x", &a, &b, &c, &d, &n);


    for (i = 0; i < n; i++)  {
        fgets (iline, 131, fin);
        sscanf (&iline[2], "%x", &v);
        if (doswap)
            v = swap(v);
        fwrite (&v, sizeof(unsigned int), 1, fout);
    }

    fclose (fout);
    fclose (fin);

    return (0);

}
