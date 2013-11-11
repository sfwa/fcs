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

#ifndef _FCS_I2C_EEPROM_H
#define _FCS_I2C_EEPROM_H

/*
I2C DMA driver for the M24256 EEPROM.
*/

/* EEPROM page size in bytes */
#define FCS_EEPROM_PAGE_SIZE 64

/* Number of pages in EEPROM */
#define FCS_EEPROM_PAGE_COUNT 512

enum fcs_eeprom_cmd_result_t {
    FCS_EEPROM_CMD_OK = 0,
    FCS_EEPROM_CMD_PENDING,
    FCS_EEPROM_CMD_ERR_NO_ACK,
    FCS_EEPROM_CMD_ERR_OTHER,
    FCS_EEPROM_CMD_IDLE
};

/* Initialize the EEPROM state */
void fcs_eeprom_init(void);

/*
Read a 64-byte page from the EEPROM. dst must point to a buffer at least
64 bytes long. src_addr bits 15:6 specify the page to read; 5:0 MUST BE 0.

Returns FCS_EEPROM_CMD_PENDING. Will assert if there's a command already in
progress.
*/
enum fcs_eeprom_cmd_result_t fcs_eeprom_read_page(uint8_t *restrict dst,
uint16_t src_addr);

/*
Write a 64-byte page to the EEPROM. src must point to a buffer at least
64 bytes long. dst_addr bits 15:6 specify the page to read; 5:0 MUST BE 0.

The contents of src are NOT copied, and MUST NOT change until
fcs_eeprom_cmd_status returns FCS_EEPROM_CMD_OK.

Returns FCS_EEPROM_CMD_PENDING. Will assert if there's a command already in
progress.
*/
enum fcs_eeprom_cmd_result_t fcs_eeprom_write_page(uint16_t dst_addr,
uint8_t *restrict src);

/*
Returns the status of the last-executed command, or FCS_EEPROM_CMD_PENDING if
a command is still in progress. Will assert if no command has been issued.
*/
enum fcs_eeprom_cmd_result_t fcs_eeprom_cmd_status(void);

/*
Called by fcs_config tick every millisecond. Handles multi-stage I2C commands
and updates command completion status.
*/
void fcs_eeprom_tick(void);

#endif
