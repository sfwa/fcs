#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "tiboot.h"

#undef  BOOT_PARAMS_SPI_OPTIONS_SHIFT
#define BOOT_PARAMS_SPI_OPTIONS_SHIFT (BOOT_PARAMS_SPI_OPTIONS_MASK - 1)

#undef  BOOT_PARAMS_SPI_SET_BOOTTBL_MODE
#define BOOT_PARAMS_SPI_SET_BOOTTBL_MODE(options, mode)              \
        (options) = ((options) & BOOT_PARAMS_SPI_OPTIONS_MASK) |     \
                    (((mode)   & BOOT_PARAMS_SPI_OPTIONS_MASK) <<    \
                                 BOOT_PARAMS_SPI_OPTIONS_SHIFT)

/*
- This simple application "prepends" a "Boot Parameter Table" and concatenates this BPT with
a .ccs file creating the resultant output file i2crom.ccs.
- The specification for the BPT is provided via the .map file specified on the command line.
*/

BOOT_PARAMS_SPI_T     spiBootParamTable;
char                  szExeFile[1024];

/*************************************************************************************
 * Declaration: The base address of the i2c ROM being created. This is just
 *              the I2C bus address. The default is 0x50
 *************************************************************************************/
int i2cRomBase = 0x00000050; //IBL_CFG_I2C_MAP_TABLE_DATA_BUS_ADDR;

FILE *yyin;
char line[1024];

/*************************************************************************************
 * Declaration: The next free address in the ROM for general data usage. For the
 *              start address there is room for the initial boot parameter tables,
 *              plus room for the PCI eeai config.
 *************************************************************************************/
int romBase = 0x03000000; //DATA_BASE;

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

int isFieldAndValue(char *szIn, char *szField, int *val)
{
    char *pch;
    int offset = 0;
    int result = 0;

    pch = strstr(szIn,szField);
    if (NULL != pch)
    {
        offset = (int)pch - (int)szIn + strlen(szField);
        pch = strrchr(&szIn[offset],'=');
        if (NULL != pch)
        {
            offset=1;
            while (('\0' != pch[offset]) && (('\t' == pch[offset]) || (' ' == pch[offset]))) offset++;
            if ('\0' != &pch[offset])
            {
                *val = readVal(&pch[offset]);
                result = 1;
            } /* if */
        } /* if */
    } /* if */

    return (result);

} /* isFieldAndValue() */

int isFieldAndString(char *szIn, char *szField, char **szString)
{
    char *pch;
    int offset = 0;
    int result = 0;

    pch = strstr(szIn,szField);
    if (NULL != pch)
    {
        offset = (int)pch - (int)szIn + strlen(szField);
        pch = strrchr(&szIn[offset],'=');
        if (NULL != pch)
        {
            offset=1;
            while (('\0' != pch[offset]) && (('\t' == pch[offset]) || (' ' == pch[offset]))) offset++;
            if ('\0' != &pch[offset])
            {
                *szString = &pch[offset];
//printf("string @ offset=%d found:%s\n",offset,*szString);
                result = 1;
            } /* if */
        } /* if */
    } /* if */

    return (result);

} /* isFieldAndString() */

/******************************************************************************
 *
 * Function:
 *  ReadLine
 *
 * Description:
 *  Reads a line of characters from a file.
 *
 * Parameters:
 *  fp - Handle to an open file in the "rt" mode.
 *  line - location to store character read from fp.
 *
 * Returns:
 *  Negative - Error occured
 *  Positive - Number of characters read including '\0'
 *
 ******************************************************************************/
int ReadLine(FILE *fp,char *line)
{
    int ch;
    int count = 0;

    ch = fgetc(fp);

    if (EOF == ch)
    {
        count = -1;
    } /* if */
    else
    {
        while(('\n' != ch) && (EOF != ch))
        {
            *line++ = ch;
            ch = fgetc(fp);
            count++;
        } /* while */
        *line++ = '\0';
        count++;
    } /* else */

    return (count);

} /* ReadLine() */

int yyparse(void)
{
    int count = 0;
    int index = 0;
    int val = 0;
    int sectionStart = -1;
    int sectionOpen  = -1;
    int sectionEnd   = -1;
    char *pch;
    char *szStr;

    memset(&spiBootParamTable,0,sizeof(spiBootParamTable));

    /* Parse "section" */
    while (((count-index) >= 0) && (-1 == sectionEnd))
    {
        count = ReadLine(yyin,line);
        index = 0;
//printf("TOP ... count=%d, index=%d, line:%s\n",count,index,&line[index]);
        if ((count-index) > 0)
        {
            if (-1 == sectionStart)
            {
                if ((count-index) > 0)
                {
                    pch = strstr(&line[index],"section");
                    if (NULL != pch)
                    {
                        sectionStart = 1;
                        index  = 7 + ((int)pch - (int)line);
//printf("SectionStart ... count=%d, index=%d, line:%s\n",count-index,index,&line[index]);
                    } /* if */
                } /* if */
            } /* if */

            if (-1 == sectionOpen)
            {
                if ((count-index) > 0)
                {
                    pch = strrchr(&line[index],'{');
                    if (NULL != pch)
                    {
                        sectionOpen = 1;
                        index  = 1 + ((int)pch - (int)line);
//printf("SectionOpen ... count=%d, index=%d, line:%s\n",count-index,index,&line[index]);
                    } /* if */
                } /* if */
            } /* if */

            else
            {
                if ((count-index) > 0)
                {
//printf("Fields ... count=%d, index=%d, line:%s\n",count-index,index,&line[index]);
                    if (isFieldAndValue(&line[index],"boot_mode",&val))          {spiBootParamTable.boot_mode           = val;}
                    if (isFieldAndValue(&line[index],"param_index",&val))        {spiBootParamTable.portNum             = val;}
                    if (isFieldAndValue(&line[index],"sw_pll_flags",&val))       {spiBootParamTable.swPllCfg_msw       |= (UINT16)(0x0000C000 & (val<<14));}
                    if (isFieldAndValue(&line[index],"sw_pll_mult",&val))        {spiBootParamTable.swPllCfg_msw       |= (UINT16)(0x00003FFF & (val<<0));}
                    if (isFieldAndValue(&line[index],"sw_pll_prediv",&val))      {spiBootParamTable.swPllCfg_lsw       |= (UINT16)(0x0000FF00 & (val<<8));}
                    if (isFieldAndValue(&line[index],"sw_pll_postdiv",&val))     {spiBootParamTable.swPllCfg_lsw       |= (UINT16)(0x000000FF & (val<<0));}
                    if (isFieldAndValue(&line[index],"options",&val))            {spiBootParamTable.options             = val;}
                    if (isFieldAndValue(&line[index],"addr_width",&val))         {spiBootParamTable.addrWidth           = val;}
                    if (isFieldAndValue(&line[index],"n_pins",&val))             {spiBootParamTable.nPins               = val;}
                    if (isFieldAndValue(&line[index],"csel",&val))               {spiBootParamTable.csel                = val;}
                    if (isFieldAndValue(&line[index],"mode",&val))               {spiBootParamTable.mode                = val; /*BOOT_PARAMS_SPI_SET_BOOTTBL_MODE(spiBootParamTable.options,val);*/}
                    if (isFieldAndValue(&line[index],"c2t_delay",&val))          {spiBootParamTable.c2tdelay            = val;}
                    if (isFieldAndValue(&line[index],"core_freq_mhz",&val))      {spiBootParamTable.cpuFreqMhz          = val;}
                    if (isFieldAndValue(&line[index],"bus_freq_mhz",&val))       {spiBootParamTable.busFreqMhz          = val;}
                    if (isFieldAndValue(&line[index],"bus_freq_khz",&val))       {spiBootParamTable.busFreqKhz          = val;}
                    if (isFieldAndValue(&line[index],"read_addr_msw",&val))      {spiBootParamTable.read_addr_msw       = (UINT16)(0x0000FFFF & (val<<0));}
                    if (isFieldAndValue(&line[index],"read_addr_lsw",&val))      {spiBootParamTable.read_addr_lsw       = (UINT16)(0x0000FFFF & (val<<0));}
                    if (isFieldAndValue(&line[index],"next_csel",&val))          {spiBootParamTable.next_csel           = val;}
                    if (isFieldAndValue(&line[index],"next_read_addr_msw",&val)) {spiBootParamTable.next_read_addr_msw  = val;}
                    if (isFieldAndValue(&line[index],"next_read_addr_lsw",&val)) {spiBootParamTable.next_read_addr_lsw  = val;}
                    if (isFieldAndString(&line[index],"exe_file",&szStr))        {if('"'==szStr[0]) strncpy(szExeFile,&szStr[1],strlen(szStr)-2); else strcpy(szExeFile,szStr);}

                    /* Are we done? */
                    pch = strrchr(&line[index],'}');
                    if (NULL != pch)
                    {
                        sectionEnd = 1;
                        index = 1 + ((int)pch - (int)line);
//printf("SectionEnd ... count=%d, index=%d, line:%s\n",count-index,index,&line[index]);
                    } /* if */
                } /* if */
            } /* else */
        } /* if */
    } /* while */

#if 1
    printf("length             =   %d\n",  spiBootParamTable.length);
    printf("checksum           =   %d\n",  spiBootParamTable.checksum);
    printf("boot_mode          =   %d\n",  spiBootParamTable.boot_mode);
    printf("param_index        =   %d\n",  spiBootParamTable.portNum);
    printf("sw_pll_flags       = 0x%04X\n",spiBootParamTable.swPllCfg_msw);
    printf("sw_pll_mult        = 0x%04X\n",spiBootParamTable.swPllCfg_msw);
    printf("sw_pll_prediv      = 0x%04X\n",spiBootParamTable.swPllCfg_lsw);
    printf("sw_pll_postdiv     = 0x%04X\n",spiBootParamTable.swPllCfg_lsw);
    printf("options            = 0x%04X\n",spiBootParamTable.options);
    printf("addr_width         =   %d\n",  spiBootParamTable.addrWidth);
    printf("n_pins             =   %d\n",  spiBootParamTable.nPins);
    printf("csel               =   %d\n",  spiBootParamTable.csel);
    printf("mode               =   %d\n",  spiBootParamTable.mode);
    printf("c2t_delay          =   %d\n",  spiBootParamTable.c2tdelay);
    printf("core_freq_mhz      =   %d\n",  spiBootParamTable.cpuFreqMhz);
    printf("bus_freq_mhz       =   %d\n",  spiBootParamTable.busFreqMhz);
    printf("bus_freq_khz       =   %d\n",  spiBootParamTable.busFreqKhz);
    printf("read_addr_msw      = 0x%04X\n",spiBootParamTable.read_addr_msw);
    printf("read_addr_lsw      = 0x%04X\n",spiBootParamTable.read_addr_lsw);
    printf("next_csel          =   %d\n",  spiBootParamTable.next_csel);
    printf("next_read_addr_msw = 0x%04X\n",spiBootParamTable.next_read_addr_msw);
    printf("next_read_addr_lsw = 0x%04X\n",spiBootParamTable.next_read_addr_lsw);
    printf("exe_file           =   %s\n",  szExeFile);
#endif

    return (0);

} /* yyparse() */

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
 * FUNCTION PURPOSE: Create a 32 bit value from the image array
 ************************************************************************************
 * DESCRIPTION: A 32 bit word in big endian format is created
 ************************************************************************************/
unsigned int formWord (unsigned int p, unsigned char *image)
{
  unsigned int v;

/*
  v = (image[p+0] << 24) |
      (image[p+1] << 16) |
      (image[p+2] <<  8) |
      (image[p+3] <<  0) ;
*/

  v = (image[p+1] << 24) |
      (image[p+0] << 16) |
      (image[p+3] <<  8) |
      (image[p+2] <<  0) ;

  return (v);

}

int createOutput(void)
{
    FILE *fpIn;
    FILE *fpOut;
    int i, count, a, b, c, d, num32BitWords, value;

    fpIn = fopen (szExeFile, "r");
    if (NULL == fpIn)
    {
        fprintf (stderr, "could not open file %s\n",szExeFile);
        return (-1);
    } /* if */

    fpOut = fopen ("i2crom.ccs", "w");
    if (NULL == fpOut)
    {
        fprintf (stderr, "could not open file %s\n","i2crom.ccs");
        return (-1);
    } /* if */

    /* Obtain record from first line of .ccs */
    count = ReadLine(fpIn,line);
    sscanf(line,"%d %d %d %d %x",&a,&b,&c,&d,&num32BitWords);
    printf("from %s ... %d %d %d %d %d\n",szExeFile,a,b,c,d,num32BitWords);

    /* Update the boot table */
    spiBootParamTable.length         = 0x50; /* 80 bytes */
    spiBootParamTable.checksum       = 0;
    spiBootParamTable.read_addr_lsw  = 0x400; /* Allow 1KB at the start */

    /* Output header record for .ccs */
    fprintf(fpOut,"%d %d %d %d %x\n",a,b,c,d,num32BitWords + 256);

    /* Output the spiBootParamTable  */
    value = formWord(0, (unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(4, (unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(8, (unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(12,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(16,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(20,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(24,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(28,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(32,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);
    value = formWord(36,(unsigned char *)&spiBootParamTable); fprintf(fpOut,"0x%08x\n",value);

    /* Output the remainder of the padding */
    for (i = 40; i < 1024; i += 4) {
        fprintf(fpOut, "0x00000000\n");
    }

    /* Append the .ccs file to the output "i2crom.ccs" */
    for (i=0; i<num32BitWords; i++)
    {
        count = ReadLine(fpIn,line);
        //printf("%s\n",line);
        value = readVal(line);
        //printf("0x%08x\n",value);
        fprintf(fpOut, "0x%08x\n", value);
        //printf("from %s ... entry=%d ... 0x%08X\n",szExeFile,i,value);
    } /* for */

    fclose (fpIn);
    fclose (fpOut);

    return (0);
}

int main (int argc, char *argv[])
{


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

}
