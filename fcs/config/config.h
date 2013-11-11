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

#ifndef _FCS_CONFIG_H
#define _FCS_CONFIG_H

#define FCS_CORE_AHRS 0
#define FCS_CORE_NMPC 1
#define FCS_CORE_UTIL 0
#define FCS_CORE_COMMS 0
#define FCS_CORE_CONFIG 0
#define FCS_CORE_PIKSI 0

#define FCS_CLOCK_HZ 1250000000u
#define FCS_CORE0_TICK_HZ 1000u
#define FCS_CORE1_TICK_HZ 50u

#define FCS_DEADLINE_EXCEEDED_CYCLES (FCS_CLOCK_HZ << 2)

/*
Called by CORE0 on boot. Initializes I2C and starts reading from the EEPROM.
*/
void fcs_config_init(void);

/*
Called each tick of CORE0 (1kHz). Manages ongoing tasks.
*/
void fcs_config_tick(void);

/*
Configuration API

fcs_config_get_current returns an opaque pointer to the currently-active
configuration, and each module needs to check that against the configuration
it's currently using. If there's a mismatch, the module needs to assert that
the configuration is valid (SHOULD have already been done, but just to be
sure), and then update its own internal state with the new configuration
(restarting if necessary).

Modules access their configuration by calling fcs_config_get_param_block with
a parameter key and version number. The function returns OK if a matching
parameter was found, and otherwise indicates if the parameter was missing or
had a version mismatch. Modules can then look for previous versions of the
block if they understand them, or error if no comprehensible versions of a
required parameter exist.

Internally there are two config buffers in a ping-pong configuration; at any
given time, one buffer stores the live config and the other is used for the
staging config. When fcs_config_activate_staging is called, the config in the
staging buffer is promoted to live, and the live buffer is re-used for
staging.

The configuration is parsed from a netstring
(http://en.wikipedia.org/wiki/Netstring) with the following structure:
CONFIG_LENGTH:
    CONFIG_NAME_LENGTH:CONFIG_NAME,
    CONFIG_DATA_LENGTH:
        CONFIG_ENTRY0_LENGTH:CONFIG_ENTRY0,
        ...
        CONFIG_ENTRYn_LENGTH:CONFIG_ENTRYn
    ,
    CONFIG_HASH_LENGTH:CONFIG_HASH,
,
*/

/*
Opaque reference to a configuration set. Each system checks to make sure its
current configuration is still active, and reconfigures if not.
*/

#define FCS_CONFIG_KEY_LEN 16
#define FCS_CONFIG_NUM_PARAMS 32
#define FCS_CONFIG_PARAM_SIZE 1024
#define FCS_CONFIG_MAX_NAME_LEN 64
#define FCS_CONFIG_MAX_HASH_LEN 256

typedef void* fcs_config_t;

/* Result codes for fcs_config_get_param_block */
enum fcs_config_param_result_t {
    FCS_CONFIG_PARAM_OK,
    FCS_CONFIG_PARAM_MISSING,
    FCS_CONFIG_PARAM_TOO_LONG
};

/* Result codes for config validation functions for other modules */
enum fcs_config_validation_result_t {
    FCS_CONFIG_OK,
    FCS_CONFIG_ERROR
};

/* Get a reference to the current configuration */
fcs_config_t fcs_config_get_current(void);

/* Get a reference to the staging configuration */
fcs_config_t fcs_config_get_staging(void);

/*
Parse a partial config data block and save it to the config staging area,
returning a parse result.
*/
enum fcs_config_validation_result_t fcs_config_parse_incremental(
uint8_t *restrict in, size_t len);

/*
Switch staging config to live config
*/
void fcs_config_activate_staging();

/*
Check that a configuration is valid -- calls each of the module configuration
validation routines.
*/
enum fcs_config_validation_result_t fcs_config_validate_staging(void);

/*
Copy configuration parameter data identified by a six-character key and a
16-bit version ID into the dst buffer. Returns FCS_CONFIG_PARAM_TOO_LONG if
the parameter length exceeds dst_size bytes.

Returns FCS_CONFIG_PARAM_MISSING if the parameter key is not found,
FCS_CONFIG_PARAM_TOO_NEW if the parameter version is greater than
param_version, and FCS_CONFIG_PARAM_TOO_OLD if the parameter version is less
than param_version.
*/
enum fcs_config_param_result_t fcs_config_get_param_block(fcs_config_t config,
uint16_t dst_size, uint8_t *dst, const uint8_t *restrict param_key,
uint16_t param_version);

/*
Test whether or not a parameter block with the specified key and version
exists.
*/
enum fcs_config_param_result_t fcs_config_check_param_block(
fcs_config_t config, uint16_t buf_size, const uint8_t *restrict param_key,
uint16_t param_version);

#endif
