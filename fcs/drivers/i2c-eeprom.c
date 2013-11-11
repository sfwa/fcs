/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

#include "i2c-eeprom.h"

/*
Tw -- maximum write cycle duration in milliseconds. It's possible to poll to
get faster writes, but this isn't significant in our application.
*/
#define FCS_I2C_EEPROM_TW 5

/*
I2C delay in cycles. Just a general delay amount used for various things
in the sample code; not sure if it's actually required.

TODO: check if this is necessary.
*/
#define FCS_I2C_DELAY_CYCLES 100

/*
7-bit I2C addresses -- packed as the high-order 7 bits of the address byte,
where the LSB is the R/_W flag
*/
#define FCS_I2C_EEPROM_MEM_ADDR 0x50
#define FCS_I2C_EEPROM_CODE_ADDR 0x51

/*
400kHz I2C clock -- prescaler to 12MHz and 50% duty cycle on the clock

FIXME: These values are for the 1000MHz EVM clock, not our 1250MHz clock.
*/
#define FCS_I2C_PRESCALER 13
#define FCS_I2C_CLK_LOW 9
#define FCS_I2C_CLK_HIGH 9

/*
I2C structure base address
*/
#define FCS_I2C ((CSL_I2cRegs*)CSL_I2C_DATA_CONTROL_REGS)

/*
M24256 protocol uses 16-bit addresses; the MSB is sent first, then the LSB.

To write, send the device address byte with R/_W low, then the two-byte
address, then one or more data bytes and STOP.

To read sequentially, send the device address with R/_W low, then the two-byte
address, then the device address with R/_W high, then continue reading. Last
byte requires NO ACK + STOP.

Endurance is applied to 4-byte groups (the device writes 4 bytes at a time)
but for simplicity we only deal in pages.
*/

static enum {
    NO_CMD,
    READ_CMD,
    WRITE_CMD
} current_cmd_type;
static uint16_t current_cmd_addr;
static enum fcs_eeprom_cmd_result_t current_cmd_status;
static uint8_t *current_cmd_buf;
static uint64_t current_cmd_start;

static void _i2c_delay(void) {
    uint32_t start_t = TSCL;
    while (TSCL - start_t < FCS_I2C_DELAY_CYCLES);
}

void fcs_eeprom_init(void) {
    current_cmd_type = NO_CMD;
    current_cmd_addr = 0xffffu;
    current_cmd_status = FCS_EEPROM_CMD_IDLE;
    current_cmd_buf = NULL;

    /* Set reset state on I2C */
    FCS_I2C->ICMDR &= ~CSL_I2C_ICMDR_IRS_MASK;

    _i2c_delay();

    /* Clock configuration */
    FCS_I2C->ICPSC  = FCS_I2C_PRESCALER;
    FCS_I2C->ICCLKL = FCS_I2C_CLK_LOW;
    FCS_I2C->ICCLKH = FCS_I2C_CLK_HIGH;

    /*
    Set transmit (TRX) and master (MST) mode, with I2C transactions run even
    while in breakpoints (FREE)
    */

    FCS_I2C->ICMDR = CSL_I2C_ICMDR_MST_MASK |
                     CSL_I2C_ICMDR_TRX_MASK |
                     CSL_I2C_ICMDR_FREE_MASK;


    /* Clear reset state */
    FCS_I2C->ICMDR |= CSL_I2C_ICMDR_IRS_MASK;

    _i2c_delay();
}

enum fcs_eeprom_cmd_result_t fcs_eeprom_read_page(uint8_t *restrict dst,
uint16_t src_addr) {
    assert(src_addr < FCS_EEPROM_PAGE_SIZE * FCS_EEPROM_PAGE_COUNT);
    assert(src_addr & 0x3fu == 0x00);
    assert(current_cmd_status != FCS_EEPROM_CMD_PENDING);

    /* Set current command info */
    current_cmd_status = FCS_EEPROM_CMD_PENDING;
    current_cmd_addr = src_addr;
    current_cmd_type = READ_CMD;
    current_cmd_buf = dst;

    /* TODO: Initiate read */
}

enum fcs_eeprom_cmd_result_t fcs_eeprom_write_page(uint16_t dst_addr,
uint8_t *restrict src) {
    assert(dst_addr < FCS_EEPROM_PAGE_SIZE * FCS_EEPROM_PAGE_COUNT);
    assert(dst_addr & 0x3fu == 0x00);
    assert(current_cmd_status != FCS_EEPROM_CMD_PENDING);

    /* Set current command info */
    current_cmd_status = FCS_EEPROM_CMD_PENDING;
    current_cmd_addr = dst_addr;
    current_cmd_type = WRITE_CMD;
    current_cmd_buf = src;

    /* TODO: Initiate write */
}

enum fcs_eeprom_cmd_result_t fcs_eeprom_cmd_status(void) {
    assert(current_cmd_type != NO_CMD);

    return current_cmd_status;
}

void fcs_eeprom_tick(void) {
    /* TODO: handle pending commands */
}
