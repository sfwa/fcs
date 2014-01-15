/******************************************************************************
 * Copyright (c) 2011 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/

/**************************************************************************************
 * FILE PURPOSE: NOR writer utility
 **************************************************************************************
 * FILE NAME: norwriter.c
 *
 * DESCRIPTION: A simple nor writer using platform lib APIs to program the NOR flash
 *              with an image that the ibl can read.
 *
 ***************************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "types.h"

/* NOR writer utility version */
char version[] = "01.00.00.03";

/* The input file name is hard coded */
char *input_file = "nor_writer_input.txt";

uint32_t swap_byte = 0;

/* Parameters defined in the input_file */
#define FILE_NAME      "file_name"
#define START_ADDR     "start_addr"

/* Memory address to store the write data -- use MSMC SRAM instead of DDR3 */
#define WRITE_DATA_ADDRESS    0x0C000000 /* 0x80000000 */

/******************************************************************************
 * Structure:   NOR_WRITER_INFO_T
 *
 *              NOR writer control data. This structure should be filled in
 *              by the user before running
 ******************************************************************************/
#define MAX_LINE_LENGTH 40
typedef struct NOR_WRITER_INFO_tag
{
    char        file_name[MAX_LINE_LENGTH]; /* CCS format data file name */
    uint32_t    blockSizeBytes;             /* The size of each sector */
    uint32_t    deviceTotalBytes;           /* Total number of bytes available in the device */
    uint32_t    startAddr;                  /* Start address to write */
    uint32_t    writeBytes;                 /* Number of bytes to be written into the device */
    uint8_t     *writeData;                 /* Address to store the write data */
    uint8_t     *readData;                  /* Address to store the read data */

} NOR_WRITER_INFO_T;

NOR_WRITER_INFO_T norWriterInfo;

/* OSAL functions for Platform Library */
uint8_t *Osal_platformMalloc (uint32_t num_bytes, uint32_t alignment)
{
	return malloc(num_bytes);
}

void Osal_platformFree (uint8_t *dataPtr, uint32_t num_bytes)
{
    /* Free up the memory */
    if (dataPtr)
    {
        free(dataPtr);
    }
}

void Osal_platformSpiCsEnter(void)
{
    return;
}

void Osal_platformSpiCsExit (void)
{
    return;
}

/******************************************************************************
 * Function:    print_platform_errno
 ******************************************************************************/
void
print_platform_errno
(
    void
)
{
    printf ("Returned platform error number is %d\n", platform_errno);
}

/******************************************************************************
 * Function:    form_block
 *
 *      Form a block of data to write to the NOR. The block is
 *      created as a byte stream from the 4 byte stream in which
 *      the MSB is always sent first.
 ******************************************************************************/
void
formBlock
(
    uint32_t      *data,
    uint32_t      blockSize,
    uint8_t       *scratch
)
{
    uint32_t i, j;

    /* Convert the data to a byte stream */
    for (i = j = 0; j < blockSize; i++, j+=4)
    {
        scratch[j+0] = (data[i] >> 24) & 0xff;
        scratch[j+1] = (data[i] >> 16) & 0xff;
        scratch[j+2] = (data[i] >>  8) & 0xff;
        scratch[j+3] = (data[i] >>  0) & 0xff;
    }
}

/******************************************************************************
 * Function:    flash_nor
 *
 *              Write the image to flash.
 *              Returns TRUE if the image is written successfully
 *                      FALSE if the image write fails
 ******************************************************************************/
Bool
flash_nor
(
    PLATFORM_DEVICE_info    *p_device
)
{
    uint32_t      wPos, wLen;
    uint32_t      block, start_block;
    uint8_t       *scrach_block;

    if (swap_byte)
    {
        scrach_block = malloc(norWriterInfo.blockSizeBytes);
        if (scrach_block == NULL)
        {
            printf ("Can not allocate scratch block memory!\n");
            return (FALSE);
        }
    }

    start_block = norWriterInfo.startAddr / norWriterInfo.blockSizeBytes;

    /* Program the NOR */
    for (block = start_block, wPos = 0; wPos < norWriterInfo.writeBytes; block++, wPos += norWriterInfo.blockSizeBytes)
    {
        printf ("Flashing sector %d (%d bytes of %d)\n", block, wPos, norWriterInfo.writeBytes);

        wLen = norWriterInfo.blockSizeBytes;
        if (norWriterInfo.writeBytes - wPos < norWriterInfo.blockSizeBytes)
        {
            wLen = norWriterInfo.writeBytes - wPos;
        }

        if (swap_byte)
        {
            formBlock((uint32_t *)(&norWriterInfo.writeData[wPos]), norWriterInfo.blockSizeBytes, scrach_block);
        }
        else
        {
            scrach_block = &norWriterInfo.writeData[wPos];
        }
        if (platform_device_write(p_device->handle,
                                  block*norWriterInfo.blockSizeBytes,
                                  scrach_block,
                                  wLen) != Platform_EOK)
        {
            printf ("platform_nor_write sector # %d failed!\n", block);
            print_platform_errno();
            if (swap_byte) free (scrach_block);
            return (FALSE);
        }
    }

    if (swap_byte) free (scrach_block);
    return (TRUE);
}

/******************************************************************************
 * Function:    flash_verify
 *
 *              Read back the data file that was just flashed. On errors mark the block as bad.
 *              Returns TRUE if the image verified correctly.
 *                      FALSE if the image verification failed
 ******************************************************************************/
Bool
flash_verify
(
    PLATFORM_DEVICE_info    *p_device
)
{
    uint32_t      rPos, rLen;
    uint32_t      i, j;
    uint32_t      block, start_block;
    uint8_t       *scrach_block;
    uint32_t      *read_data_w;

    if (swap_byte)
    {
        scrach_block = malloc(norWriterInfo.blockSizeBytes);
        if (scrach_block == NULL)
        {
            printf ("Can not allocate scratch block memory!\n");
            return (FALSE);
        }
    }

    start_block = norWriterInfo.startAddr / norWriterInfo.blockSizeBytes;

    for (block = start_block, rPos = 0; rPos < norWriterInfo.writeBytes; block++, rPos += norWriterInfo.blockSizeBytes)
    {
        printf ("Reading and verifying sector %d (%d bytes of %d)\n", block, rPos, norWriterInfo.writeBytes);

        if (!swap_byte)
        {
            scrach_block = &norWriterInfo.readData[rPos];

        }
        /* Read a sector of data */
        if(platform_device_read(p_device->handle,
                                block*norWriterInfo.blockSizeBytes,
                                scrach_block,
                                norWriterInfo.blockSizeBytes) != Platform_EOK)
        {
            printf ("Failure in sector %d\n", block);
            print_platform_errno();
            if (swap_byte) free (scrach_block);
            return (FALSE);
        }

        /* Convert the packed data */
        if (swap_byte)
        {
            read_data_w = (uint32_t *)(&norWriterInfo.readData[rPos]);
            for  (i = 0, j = 0; i < norWriterInfo.blockSizeBytes; i += 4)
                read_data_w[j++] = (scrach_block[i+0] << 24) | (scrach_block[i+1] << 16) | (scrach_block[i+2] << 8) | scrach_block[i+3];
        }

        /* Read the data from the file */
        rLen = norWriterInfo.blockSizeBytes;
        if (norWriterInfo.writeBytes - rPos < norWriterInfo.blockSizeBytes)
        {
            rLen = norWriterInfo.writeBytes - rPos;
        }

        for (i = rPos; i < rLen; i++)
        {
            if (norWriterInfo.readData[i] != norWriterInfo.writeData[i])
            {
                printf ("Failure in sector %d, at byte %d, (at byte %d in the data file) expected 0x%08x, read 0x%08x\n",
                        block, i, rPos, norWriterInfo.writeData[i], norWriterInfo.readData[i]);
                if (swap_byte) free (scrach_block);
                return (FALSE);
            }
        }

    }

    if (swap_byte) free (scrach_block);
    return (TRUE);
}

/******************************************************************************
 * Function:    parse_input_file
 ******************************************************************************/
static Bool
parse_input_file
(
    FILE*               fp
)
{
    char line[MAX_LINE_LENGTH];
    char tokens[] = " :=;\n\r";
    char *key, *data;

    memset(line, 0, MAX_LINE_LENGTH);

    fgets(line, MAX_LINE_LENGTH, fp);
    key  = (char *)strtok(line, tokens);
    data = (char *)strtok(NULL, tokens);

    if(strlen(data) == 0)
    {
       return FALSE;
    }

    if(strcmp(key, FILE_NAME) != 0)
    {
        return FALSE;
    }

    strcpy (norWriterInfo.file_name, data);

    fgets(line, MAX_LINE_LENGTH, fp);
    key  = (char *)strtok(line, tokens);
    data = (char *)strtok(NULL, tokens);

    if(strlen(data) == 0)
    {
       return FALSE;
    }

    if(strcmp(key, START_ADDR) != 0)
    {
        return FALSE;
    }

    norWriterInfo.startAddr = (uint32_t)atoi(data);

    return TRUE;
}

/******************************************************************************
 * Function:    find_file_length
 ******************************************************************************/
static Bool
find_file_length
(
    FILE*               fp
)
{
    char        line[MAX_LINE_LENGTH];
    char        *pEnd;
    char        *ext;
    uint32_t    data_len, write_addr;

    memset(line, 0, MAX_LINE_LENGTH);

    ext = strrchr(norWriterInfo.file_name, '.');


    if (ext && (strcmp(ext, ".dat") == 0))
    {
    fgets(line, MAX_LINE_LENGTH, fp);

    /* Read the write address from the CCS header */
    strtoul (line,&pEnd,16);
    strtoul (pEnd,&pEnd,16);
    write_addr = strtoul (pEnd,&pEnd,16);
    strtoul (pEnd,&pEnd,16);

    /* Read the data length */
    data_len = (strtoul (pEnd,NULL,16)) * 4;
    }
    else
    {
        /* find the data length by seeking to the end and getting position */
        fseek(fp, 0, SEEK_END);
        data_len = ftell(fp);
        fseek(fp, 0, SEEK_SET);
    }

    if (data_len > (norWriterInfo.deviceTotalBytes - norWriterInfo.startAddr))
    {
        printf ("The data file is too big to fit into the device.\n");
        return FALSE;
    }

    norWriterInfo.writeBytes = data_len;
    if (write_addr != WRITE_DATA_ADDRESS)
        write_addr = WRITE_DATA_ADDRESS;
    norWriterInfo.writeData  = (uint8_t *)write_addr;
    norWriterInfo.readData   = (uint8_t *)(write_addr + norWriterInfo.deviceTotalBytes);

    return TRUE;
}

/******************************************************************************
 * Function:    main
 ******************************************************************************/
void main ()
{
    FILE                    *fp;
    platform_init_flags     init_flags;
    platform_init_config    init_config;
    PLATFORM_DEVICE_info    *p_device;
    Bool                    ret;

    printf("NOR Writer Utility Version %s\n\n", version);

    fp = fopen(input_file, "r");
    if (fp == NULL)
    {
        printf("Error in opening %s input file\n", input_file);
        return;
    }

    ret = parse_input_file(fp);
    fclose (fp);

    if (ret == FALSE)
    {
        printf("Error in parsing %s input file\n", input_file);
        return;
    }

    /* Initialize main Platform lib */
    memset(&init_config, 0, sizeof(platform_init_config));
    memset(&init_flags, 1, sizeof(platform_init_flags));
    init_flags.pll = 0;
    init_flags.ddr = 0;
    if (platform_init(&init_flags, &init_config) != Platform_EOK)
    {
        printf ("Platform init failed!\n");
        print_platform_errno();
        return;
    }
#if !(defined(_EVMC6657L_))
    p_device = platform_device_open(PLATFORM_DEVID_NORN25Q128, 0);
#else
    p_device = platform_device_open(PLATFORM_DEVID_NORN25Q032A, 0);
#endif
    if (p_device == NULL)
    {
        printf ("NOR device open failed!\n");
        print_platform_errno();
        return;
    }
    norWriterInfo.deviceTotalBytes  = p_device->block_count * p_device->page_count * p_device->page_size;
    norWriterInfo.blockSizeBytes    = p_device->page_count * p_device->page_size;

    if ((norWriterInfo.startAddr % norWriterInfo.blockSizeBytes) != 0)
    {
        printf ("The start programming address 0x%8x set in %s is not at the beginning of a block, block size = 0x%4x\n",
                norWriterInfo.startAddr,
                norWriterInfo.file_name,
                norWriterInfo.blockSizeBytes);
        return;
    }

    /* Open and find the length of the data file */
    fp = fopen (norWriterInfo.file_name, "rb");
    if (fp == NULL)
    {
      printf ("Failed to open file %s\n", norWriterInfo.file_name);
      platform_device_close(p_device->handle);
      return;
    }

    /* Parse the CCS format file */
    ret = find_file_length(fp);
    fclose (fp);
    if (ret == FALSE)
    {
        printf("Error in parsing CCS file %s\n", norWriterInfo.file_name);
        platform_device_close(p_device->handle);
        return;
    }

    /* Write the flash */
    if (flash_nor (p_device) == FALSE)
    {
        printf ("NOR write failed\n");
        platform_device_close(p_device->handle);
        return;
    }

    /* verify the flash */
    if(flash_verify (p_device) == FALSE)
    {
        printf ("NOR read verify failed\n");
        platform_device_close(p_device->handle);
        return;
    }


    printf ("NOR programming completed successfully\n");

    platform_device_close(p_device->handle);

    return;
}




