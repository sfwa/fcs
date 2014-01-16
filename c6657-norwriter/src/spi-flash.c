/*****************************************************************************
 * Copyright (c) 2011-2012 Texas Instruments Incorporated - http://www.ti.com
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
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

#include "cslr_device.h"
#include "cslr_spi.h"

#include "spi-flash.h"

/* TODO: replace these with CSL values */
#define SPI_BASE                CSL_SPI_REGS
#define SPI_SPIGCR0             *( volatile Uint32* )( SPI_BASE + 0x0 )
#define SPI_SPIGCR1             *( volatile Uint32* )( SPI_BASE + 0x4 )
#define SPI_SPIINT0             *( volatile Uint32* )( SPI_BASE + 0x8 )
#define SPI_SPILVL              *( volatile Uint32* )( SPI_BASE + 0xc )
#define SPI_SPIFLG              *( volatile Uint32* )( SPI_BASE + 0x10 )
#define SPI_SPIPC0              *( volatile Uint32* )( SPI_BASE + 0x14 )
#define SPI_SPIDAT0             *( volatile Uint32* )( SPI_BASE + 0x38 )
#define SPI_SPIDAT1             *( volatile Uint32* )( SPI_BASE + 0x3c )
#define SPI_SPIBUF              *( volatile Uint32* )( SPI_BASE + 0x40 )
#define SPI_SPIEMU              *( volatile Uint32* )( SPI_BASE + 0x44 )
#define SPI_SPIDELAY            *( volatile Uint32* )( SPI_BASE + 0x48 )
#define SPI_SPIDEF              *( volatile Uint32* )( SPI_BASE + 0x4c )
#define SPI_SPIFMT0             *( volatile Uint32* )( SPI_BASE + 0x50 )
#define SPI_SPIFMT1             *( volatile Uint32* )( SPI_BASE + 0x54 )
#define SPI_SPIFMT2             *( volatile Uint32* )( SPI_BASE + 0x58 )
#define SPI_SPIFMT3             *( volatile Uint32* )( SPI_BASE + 0x5c )
#define SPI_INTVEC0             *( volatile Uint32* )( SPI_BASE + 0x60 )
#define SPI_INTVEC1             *( volatile Uint32* )( SPI_BASE + 0x64 )

#define SPI_NOR_CS              0         /* SPI Chip Select number for NOR */
#define SPI_MODULE_CLK          166666666 /* SYSCLK7  = CPU_Clk/6 in HZ */
#define SPI_MAX_FREQ            25000000  /* SPI Max frequency in Hz */
#define SPI_NOR_CHAR_LENTH      8         /* Number of bits per SPI trasfered
                                             data element for NOR flash */


typedef enum {
    SPI_EOK,
    SPI_EFAIL
} SPI_STATUS;

static uint32_t data1_reg_val;

static void spi_delay(uint32_t delay) {
    volatile uint32_t i;
    for (i = 0; i < delay; i++);
}

/*****************************************************************************
 *
 * Function:    spi_claim
 *
 * Description: This function claims the SPI bus in the SPI controller
 *
 * Parameters:  Uint32 cs       - Chip Select number for the slave SPI device
 *              Uint32 freq     - SPI clock frequency
 *
 * Return Value: error status
 *
 ****************************************************************************/
SPI_STATUS spi_claim(uint32_t cs, uint32_t freq) {
    uint32_t scalar;

    /* Enable the SPI hardware */
    SPI_SPIGCR0 = CSL_SPI_SPIGCR0_RESET_IN_RESET;
    spi_delay(2000);
    SPI_SPIGCR0 = CSL_SPI_SPIGCR0_RESET_OUT_OF_RESET;

    /* Set master mode, powered up and not activated */
    SPI_SPIGCR1 =
        (CSL_SPI_SPIGCR1_MASTER_MASTER << CSL_SPI_SPIGCR1_MASTER_SHIFT) |
        (CSL_SPI_SPIGCR1_CLKMOD_INTERNAL << CSL_SPI_SPIGCR1_CLKMOD_SHIFT);


    /* CS0, CS1, CLK, Slave in and Slave out are functional pins */
    if (cs == 0) {
        SPI_SPIPC0 =
            (CSL_SPI_SPIPC0_SCS0FUN0_SPI << CSL_SPI_SPIPC0_SCS0FUN0_SHIFT) |
            (CSL_SPI_SPIPC0_CLKFUN_SPI << CSL_SPI_SPIPC0_CLKFUN_SHIFT) |
            (CSL_SPI_SPIPC0_SIMOFUN_SPI << CSL_SPI_SPIPC0_SIMOFUN_SHIFT) |
            (CSL_SPI_SPIPC0_SOMIFUN_SPI << CSL_SPI_SPIPC0_SOMIFUN_SHIFT);
    } else if (cs == 1) {
        SPI_SPIPC0 = 0xFFFF &
            ((CSL_SPI_SPIPC0_SCS0FUN1_SPI << CSL_SPI_SPIPC0_SCS0FUN1_SHIFT) |
             (CSL_SPI_SPIPC0_CLKFUN_SPI << CSL_SPI_SPIPC0_CLKFUN_SHIFT) |
             (CSL_SPI_SPIPC0_SIMOFUN_SPI << CSL_SPI_SPIPC0_SIMOFUN_SHIFT) |
             (CSL_SPI_SPIPC0_SOMIFUN_SPI << CSL_SPI_SPIPC0_SOMIFUN_SHIFT));
    }

    /* setup format */
    scalar = ((SPI_MODULE_CLK / freq) - 1) & 0xFF;

    if (cs == 0) {
        SPI_SPIFMT0 =
            (8 << CSL_SPI_SPIFMT_CHARLEN_SHIFT) |
            (scalar << CSL_SPI_SPIFMT_PRESCALE_SHIFT) |
            (CSL_SPI_SPIFMT_PHASE_DELAY << CSL_SPI_SPIFMT_PHASE_SHIFT) |
            (CSL_SPI_SPIFMT_POLARITY_LOW << CSL_SPI_SPIFMT_POLARITY_SHIFT) |
            (CSL_SPI_SPIFMT_SHIFTDIR_MSB << CSL_SPI_SPIFMT_SHIFTDIR_SHIFT);
    } else if (cs == 1) {
        SPI_SPIFMT0 =
            (16 << CSL_SPI_SPIFMT_CHARLEN_SHIFT) |
            (scalar << CSL_SPI_SPIFMT_PRESCALE_SHIFT) |
            (CSL_SPI_SPIFMT_PHASE_NO_DELAY << CSL_SPI_SPIFMT_PHASE_SHIFT) |
            (CSL_SPI_SPIFMT_POLARITY_LOW << CSL_SPI_SPIFMT_POLARITY_SHIFT) |
            (CSL_SPI_SPIFMT_SHIFTDIR_MSB << CSL_SPI_SPIFMT_SHIFTDIR_SHIFT);
    }

    /* hold cs active at end of transfer until explicitly de-asserted */
    data1_reg_val =
        (CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT) |
        (0x02 << CSL_SPI_SPIDAT1_CSNR_SHIFT);
     if (cs == 0) {
         SPI_SPIDAT1 =
            (CSL_SPI_SPIDAT1_CSHOLD_ENABLE << CSL_SPI_SPIDAT1_CSHOLD_SHIFT) |
            (0x02 << CSL_SPI_SPIDAT1_CSNR_SHIFT);
     }

    /* including a minor delay. No science here. Should be good even with
    * no delay
    */
    if (cs == 0) {
        SPI_SPIDELAY = (8 << CSL_SPI_SPIDELAY_C2TDELAY_SHIFT) |
                       (8 << CSL_SPI_SPIDELAY_T2CDELAY_SHIFT);
        /* default chip select register */
        SPI_SPIDEF  = CSL_SPI_SPIDEF_RESETVAL;
    } else if (cs == 1) {
        SPI_SPIDELAY = (6 << CSL_SPI_SPIDELAY_C2TDELAY_SHIFT) |
                       (3 << CSL_SPI_SPIDELAY_T2CDELAY_SHIFT);
    }

    /* no interrupts */
    SPI_SPIINT0 = CSL_SPI_SPIINT0_RESETVAL;
    SPI_SPILVL = CSL_SPI_SPILVL_RESETVAL;

    /* enable SPI */
    SPI_SPIGCR1 |=
        (CSL_SPI_SPIGCR1_ENABLE_ENABLE << CSL_SPI_SPIGCR1_ENABLE_SHIFT);

    if (cs == 1) {
        SPI_SPIDAT0 = 1 << 15;
        spi_delay(10000);
        /* Read SPIFLG, wait untill the RX full interrupt */
        if (SPI_SPIFLG & (CSL_SPI_SPIFLG_RXINTFLG_FULL <<
                          CSL_SPI_SPIFLG_RXINTFLG_SHIFT)) {
            /* Read one byte data */
            scalar = SPI_SPIBUF & 0xFF;
            /* Clear the Data */
            SPI_SPIBUF = 0;
        } else {
            /* Read one byte data */
            scalar = SPI_SPIBUF & 0xFF;
            return SPI_EFAIL;
        }
    }
    return SPI_EOK;
}

/*****************************************************************************
 *
 * Function:    spi_release
 *
 * Description: This function releases the bus in SPI controller
 *
 * Parameters:  None
 *
 * Return Value: None
 *
 ****************************************************************************/
void spi_release(void) {
    /* Disable the SPI hardware */
    SPI_SPIGCR1 = CSL_SPI_SPIGCR1_RESETVAL;
}

/*****************************************************************************
 *
 * Function:    spi_xfer
 *
 * Description: This function sends and receives 8-bit data serially
 *
 * Parameters:  uint32_t nbytes   - Number of bytes of the TX data
 *              uint8_t* data_out - Pointer to the TX data
 *              uint8_t* data_in  - Pointer to the RX data
 *              Bool terminate  - TRUE: terminate the transfer, release the CS
 *                                FALSE: hold the CS
 *
 * Return Value: error status
 *
 ****************************************************************************/
SPI_STATUS spi_xfer(uint32_t nbytes, uint8_t* data_out, uint8_t* data_in,
bool terminate) {
    uint32_t i, buf_reg;
    uint8_t* tx_ptr = data_out;
    uint8_t* rx_ptr = data_in;

    /* Clear out any pending read data */
    buf_reg = SPI_SPIBUF;

    for (i = 0; i < nbytes; i++) {
        /* Wait untill TX buffer is not full */
        while (SPI_SPIBUF & CSL_SPI_SPIBUF_TXFULL_MASK);

        /* Set the TX data to SPIDAT1 */
        data1_reg_val &= ~0xFFFF;
        if (tx_ptr) {
            data1_reg_val |= *tx_ptr;
            tx_ptr++;
        }

        /* Write to SPIDAT1 */
        if (i == nbytes - 1 && terminate) {
            /*
            Release the CS at the end of the transfer when terminate flag is
            TRUE
            */
            SPI_SPIDAT1 = data1_reg_val & ~(CSL_SPI_SPIDAT1_CSHOLD_ENABLE <<
                                            CSL_SPI_SPIDAT1_CSHOLD_SHIFT);
        } else {
            SPI_SPIDAT1 = data1_reg_val;
        }


        /* Read SPIBUF, wait untill the RX buffer is not empty */
        while (SPI_SPIBUF & CSL_SPI_SPIBUF_RXEMPTY_MASK);

        /* Read one byte data */
        buf_reg = SPI_SPIBUF;
        if (rx_ptr) {
            *rx_ptr = buf_reg & 0xFF;
            rx_ptr++;
        }
    }
    return SPI_EOK;
}


/*****************************************************************************
 *
 * Function:    spi_cmd
 *
 * Description: This function sends a single byte command and receives
 * response data
 *
 * Parameters:  uint8_t  cmd      - Command sent to the NOR flash
 *              uint8_t* response - Pointer to the RX response data
 *              uint32_t len      - Lenght of the response in bytes
 *
 * Return Value: error status
 *
 ****************************************************************************/
SPI_STATUS spi_cmd(uint8_t cmd, uint8_t* response, uint32_t len) {
    bool flags = false;
    uint32_t ret;

    if (len == 0) {
        flags = true;
    }

    /* Send the command byte */
    ret = spi_xfer(1, &cmd, NULL, flags);
    assert(ret == SPI_EOK);

    /* Receive the response */
    if (len) {
        ret = spi_xfer(len, NULL, response, true);
        assert(ret == SPI_EOK);
    }
    return ret;
}

/*****************************************************************************
 *
 * Function:    spi_cmd_read
 *
 * Description: This function sends a read command and reads data from the
 * flash
 *
 * Parameters:  uint8_t  cmd      - Command sent to the NOR flash
 *              uint32_t cmd_len  - Length of the command in bytes
 *              uint8_t* dat      - Pointer to the data read
 *              uint32_t data_len - Lenght of the data read in bytes
 *
 * Return Value: error status
 *
 ****************************************************************************/
SPI_STATUS spi_cmd_read(uint8_t* cmd, uint32_t cmd_len, uint8_t* data,
uint32_t data_len) {
    bool flags = false;
    uint32_t ret;

    if (data_len == 0) {
        /* Send write command */
        ret = spi_xfer(cmd_len, cmd, NULL, true);
        assert(ret == SPI_EOK);
    } else {
        /* Send write command */
        ret = spi_xfer(cmd_len, cmd, NULL, false);
        assert(ret == SPI_EOK);

        /* Write data */
        ret = spi_xfer(data_len, NULL, data, true);
        assert(ret == SPI_EOK);
    }

    return ret;
}

/*****************************************************************************
 *
 * Function:    spi_cmd_write
 *
 * Description: This function sends a write command and writes data to the
 * flash
 *
 * Parameters:  uint8_t  cmd      - Command sent to the NOR flash
 *              uint32_t cmd_len  - Length of the command in bytes
 *              uint8_t* dat      - Pointer to the data to be written
 *              uint32_t data_len - Lenght of the data in bytes
 *
 * Return Value: error status
 *
 ****************************************************************************/
SPI_STATUS spi_cmd_write(uint8_t* cmd, uint32_t cmd_len, uint8_t* data,
uint32_t data_len) {
    bool flags = false;
    uint32_t ret;

    if (data_len == 0) {
        /* Send write command */
        ret = spi_xfer(cmd_len, cmd, NULL, true);
        assert(ret == SPI_EOK);
    } else {
        /* Send write command */
        ret = spi_xfer(cmd_len, cmd, NULL, false);
        assert(ret == SPI_EOK);

        /* Write data */
        ret = spi_xfer(data_len, data, NULL, true);
        assert(ret == SPI_EOK);
    }

    return ret;
}

/* Internal delay function to wait for NOR to become ready */
static NOR_STATUS nor_wait_ready(uint32_t timeout) {
    NOR_STATUS  ret;
    uint8_t status;
    uint8_t cmd = SPI_NOR_CMD_RDSR;

    do {
        /* Send Read Status command */
        ret = spi_xfer(1, &cmd, NULL, false);
        if (ret) {
            return ret;
        }

        /* Read status value */
        ret = spi_xfer(1, NULL, &status, true);
        if (ret) {
            return ret;
        }

        if ((status & SPI_NOR_SR_WIP) == 0) {
            break;
        }

        timeout--;
        if (!timeout) {
            break;
        }

    } while (1);

    if ((status & SPI_NOR_SR_WIP) == 0) {
        return NOR_EOK;
    }

    /* Timed out */
    return NOR_EFAIL;
}

/*****************************************************************************
 *
 * Function:    nor_init
 *
 * Description: This function configures the SPI controller communication
 *              between the DSP and the NOR flash
 *
 * Parameters:  None
 *
 * Return Value: error status
 *
 ****************************************************************************/
NOR_STATUS nor_init(void) {
    NOR_STATUS ret;
    uint8_t idcode[3]; /* Initialize the SPI interface */

    /* Claim the SPI controller */
    spi_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Read the ID codes */
    ret = spi_cmd(SPI_NOR_CMD_RDID, idcode, sizeof(idcode));
    if (ret) {
        spi_release();
        return NOR_EFAIL;
    }

    if ((idcode[0] != SPI_NOR_MANID_MICRON) &&
        (idcode[0] != SPI_NOR_MANID_MACRONIX)) {
        /* Expected Manufacturer ID does not match */
        spi_release();
        return NOR_EFAIL;
    }

    spi_release();
    return NOR_EOK;
}

/*****************************************************************************
 *
 * Function:    nor_read
 *
 * Description: This function reads data from the NOR flash
 *
 * Parameters:  uint32_t addr     - Byte address of the NOR flash
 *              uint32_t len      - Lenth in bytes to read
 *              uint8_t* buf      - Pointer of the buffer to store the bytes
 *                                  read
 *
 * Return Value: error status
 *
 ****************************************************************************/
NOR_STATUS nor_read(uint32_t addr, uint32_t len, uint8_t* buf) {
    uint8_t cmd[4];
    NOR_STATUS  ret_val;

    /* Claim the SPI controller */
    spi_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Validate address input */
    if (addr + len > SPI_NOR_MAX_FLASH_SIZE) {
        spi_release();
        return NOR_EFAIL;
    }

    /* Initialize the command to be sent serially */
    cmd[0] = SPI_NOR_CMD_READ;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    ret_val = spi_cmd_read(cmd, 4, buf, len);

    spi_release();
    return (ret_val);
}

/*****************************************************************************
 *
 * Function:    nor_write
 *
 * Description: This function writes data to the NOR flash
 *
 * Parameters:  uint32_t addr     - Byte address of the NOR flash
 *              uint32_t len      - Lenth in bytes to write
 *              uint8_t* buf      - Pointer of the buffer to store the write
 *                                  data
 *
 * Return Value: error status
 *
 ****************************************************************************/
NOR_STATUS nor_write(uint32_t addr, uint32_t len, uint8_t* buf) {
    uint32_t page_addr;
    uint32_t byte_addr;
    uint32_t page_size;
    uint32_t loopCount;

    uint32_t chunk_len;
    uint32_t actual;
    uint32_t ret;
    uint8_t cmd[4];

    /* Claim the SPI controller */
    spi_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /* Validate address input */
    if (addr + len > SPI_NOR_MAX_FLASH_SIZE) {
        spi_release();
        return NOR_EFAIL;
    }

    page_size = SPI_NOR_PAGE_SIZE;
    page_addr = addr / page_size;
    byte_addr = addr & (SPI_NOR_PAGE_SIZE - 1); /* % page_size; */

    ret = NOR_EOK;
    for (actual = 0; actual < len; actual += chunk_len) {
        /* Send Write Enable command */
        ret = spi_cmd(SPI_NOR_CMD_WREN, NULL, 0);
        if (ret) {
            spi_release();
            return NOR_EFAIL;
        }

        /* Send Page Program command */
        chunk_len = ((len - actual) < (page_size - byte_addr) ?
            (len - actual) : (page_size - byte_addr));

        cmd[0] = SPI_NOR_CMD_PP;
        cmd[1] = (uint8_t)(addr>>16);
        cmd[2] = (uint8_t)(addr>>8);
        cmd[3] = (uint8_t)addr;

        ret = spi_cmd_write(cmd, 4, buf + actual, chunk_len);
        if (ret) {
            spi_release();
            return NOR_EFAIL;
        }

        ret = nor_wait_ready(SPI_NOR_PROG_TIMEOUT);
        if (ret) {
            spi_release();
            return NOR_EFAIL;
        }

        page_addr++;
        addr += chunk_len;
        byte_addr = 0;

        loopCount = 4000;
        while (loopCount--) {
            asm("   NOP");
        }
    }

    spi_release();
    return (NOR_STATUS)ret;
}

/*****************************************************************************
 *
 * Function:    nor_erase
 *
 * Description: This function writes data to the NOR flash
 *
 * Parameters:  uint32_t  sector_number - Sector number to erase,
 *                                      if sector_number = -1, do bulk erase
 *
 * Return Value: error status
 *
 ****************************************************************************/
NOR_STATUS nor_erase(uint32_t  sector_number) {
    NOR_STATUS ret;
    uint8_t cmd[4];
    uint32_t cmd_len;
    uint32_t address;

    /* Claim the SPI controller */
    spi_claim(SPI_NOR_CS, SPI_MAX_FREQ);

    /*
    * This function currently uses sector erase only.
    * probably speed things up by using bulk erase
    * when possible.
    */

    if (sector_number == SPI_NOR_BE_SECTOR_NUM) {
        cmd[0]  = SPI_NOR_CMD_BE;
        cmd_len = 1;
    } else if (sector_number >= SPI_NOR_SECTOR_COUNT) {
        return NOR_EFAIL;
    } else {
        address = sector_number * SPI_NOR_SECTOR_SIZE;
        cmd[0]  = SPI_NOR_CMD_SE;
        cmd[1] = (address >> 16) & 0xff;
        cmd[2] = (address >>  8) & 0xff;
        cmd[3] = (address >>  0) & 0xff;

        cmd_len = 4;
    }

    /* Send Write Enable command */
    ret = spi_cmd(SPI_NOR_CMD_WREN, NULL, 0);
    if (ret) {
        spi_release();
        return NOR_EFAIL;
    }

    ret = spi_cmd_write(cmd, cmd_len, NULL, 0);
    if (ret) {
        spi_release();
        return NOR_EFAIL;
    }

    ret = nor_wait_ready(SPI_NOR_SECTOR_ERASE_TIMEOUT);
    if (ret) {
        spi_release();
        return NOR_EFAIL;
    }
    spi_release();
    return ret;
}
