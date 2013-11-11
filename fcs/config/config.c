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
#include <stdbool.h>
#include <assert.h>

#include "config.h"
#include "../ahrs/ahrs.h"
#include "../comms/comms.h"
#include "../nmpc/nmpc.h"
#include "../piksi/piksi.h"
#include "../util/util.h"

/* Parameter key/version record (item 0 in the config entry tuple) */
struct fcs_config_param_t {
    uint8_t key[FCS_CONFIG_KEY_LEN];
    uint16_t version;
    uint16_t len;
    uint32_t crc;
} __attribute__ ((packed));

/* Parsed configuration block */
struct fcs_config_block_t {
    uint32_t num_params;
    fcs_config_param_t params[FCS_CONFIG_NUM_PARAMS];
    uint8_t *param_data[FCS_CONFIG_NUM_PARAMS];

    uint8_t buf[FCS_CONFIG_NUM_PARAMS * FCS_CONFIG_PARAM_SIZE];
};

/*
Config deploy state machine -- enforce lockout periods between config
updates to ensure each module gets a chance to load the new config
*/
enum fcs_config_state_t {
    CONFIG_STATE_UNINITIALIZED,
    CONFIG_STATE_INVALID,
    CONFIG_STATE_VALID,
    CONFIG_STATE_LIVE
};

/* Ping-pong live/staging config buffers */
struct fcs_config_block_t configs[2];

/* Config buffer state */
enum fcs_config_state_t config_state[2] =
    { CONFIG_STATE_UNINITIALIZED, CONFIG_STATE_UNINITIALIZED };

/* Incremental parser state */
size_t config_packet_len, config_data_len, config_current_param_len,
       config_name_len, config_hash_len;
uint8_t config_name[FCS_CONFIG_MAX_NAME_LEN];
uint8_t config_hash[FCS_CONFIG_MAX_HASH_LEN];
uint8_t *buf_ptr;


/* Configuration buffer */

void fcs_config_init(void) {
    assert(config_state[0] == config_state[1] == CONFIG_STATE_UNINITIALIZED);

    /* FIXME: set default configuration here */
    config_state[0] = CONFIG_STATE_LIVE;
}

void fcs_config_tick(void) {

}

/* Return the currently-live config */
fcs_config_t fcs_config_get_current(void) {
    if (config_state[0] == CONFIG_STATE_LIVE &&
            config_state[1] != CONFIG_STATE_LIVE) {
        return (fcs_config_t)(&configs[0]);
    } else if (config_state[0] != CONFIG_STATE_LIVE &&
            config_state[1] == CONFIG_STATE_LIVE) {
        return (fcs_config_t)(&configs[1]);
    } else {
        /* Both live, or neither live -- error either way */
        assert(false);
        return (fcs_config_t)NULL;
    }
}

/* Return the config currently NOT (live or loading) */
fcs_config_t fcs_config_get_staging(void) {
    if (config_state[0] == CONFIG_STATE_LIVE &&
            config_state[1] != CONFIG_STATE_LIVE) {
        return (fcs_config_t)(&configs[1]);
    } else if (config_state[0] != CONFIG_STATE_LIVE &&
            config_state[1] == CONFIG_STATE_LIVE) {
        return (fcs_config_t)(&configs[0]);
    } else {
        /* Both live, or neither live -- error either way */
        assert(false);
        return (fcs_config_t)NULL;
    }
}

enum fcs_config_validation_result_t fcs_config_parse_incremental(
uint8_t *restrict in, size_t len) {

}

void fcs_config_activate_staging(void) {
    fcs_config_t config;

    if (config_state[0] == CONFIG_STATE_LIVE &&
            config_state[1] == CONFIG_STATE_VALID) {
        config = (fcs_config_t)(&configs[1]);
        config_state[0] = CONFIG_STATE_VALID;
        config_state[1] = CONFIG_STATE_LIVE;
    } else if (config_state[0] == CONFIG_STATE_VALID &&
            config_state[1] == CONFIG_STATE_LIVE) {
        config = (fcs_config_t)(&configs[0]);
        config_state[0] = CONFIG_STATE_LIVE;
        config_state[1] = CONFIG_STATE_VALID;
    } else {
        /* One config must be live and one valid for this to work */
        assert(false);
    }

    fcs_ahrs_load_config(config);
    fcs_comms_load_config(config);
    fcs_nmpc_load_config(config);
    fcs_piksi_load_config(config);
    fcs_util_load_config(config);
}

enum fcs_config_validation_result_t fcs_config_validate_staging(void) {
    fcs_config_t config;
    uint8_t config_idx;

    if (config_state[0] == CONFIG_STATE_LIVE &&
            config_state[1] != CONFIG_STATE_LIVE) {
        config = (fcs_config_t)(&configs[1]);
        config_idx = 1;
    } else if (config_state[0] != CONFIG_STATE_LIVE &&
            config_state[1] == CONFIG_STATE_LIVE) {
        config = (fcs_config_t)(&configs[0]);
        config_idx = 0;
    } else {
        /* One config must be live and one valid for this to work */
        assert(false);
    }
    config_state[config_idx] = CONFIG_STATE_INVALID;

    enum fcs_config_validation_result_t result;

    result = fcs_ahrs_validate_config(config);
    if (result != FCS_CONFIG_OK) {
        return result;
    }

    result = fcs_comms_validate_config(config);
    if (result != FCS_CONFIG_OK) {
        return result;
    }

    result = fcs_nmpc_validate_config(config);
    if (result != FCS_CONFIG_OK) {
        return result;
    }

    result = fcs_piksi_validate_config(config);
    if (result != FCS_CONFIG_OK) {
        return result;
    }

    result = fcs_util_validate_config(config);
    if (result != FCS_CONFIG_OK) {
        return result;
    }

    config_state[config_idx] = CONFIG_STATE_VALID;
    return FCS_CONFIG_OK;
}

static enum fcs_config_param_result_t _fcs_config_find_param(
fcs_config_t config, uint16_t buf_size, const uint8_t *restrict param_key,
uint16_t param_version, uint16_t *out_param_idx) {
    assert(config == (fcs_config_t)&configs[0] ||
           config == (fcs_config_t)&configs[1]);

    struct fcs_config_block_t *cfg = (struct fcs_config_block_t *)config;
    assert(cfg->num_params <= FCS_CONFIG_NUM_PARAMS);

    uint32_t i, j, n;
    /*
    Loop over all configuration parameters and search for a matching config
    key and version
    */
    for (i = 0, n = cfg->num_params; i < n; i++) {
        for (j = 0; j < FCS_CONFIG_KEY_LEN; j++) {
            if (cfg->params[i].key[j] != param_key[j]) {
                break;
            }
        }

        /* Skip if there was a key mismatch */
        if (j != FCS_CONFIG_KEY_LEN) {
            continue;
        }

        /* Skip if there was a version mismatch */
        if (cfg->params[i].version == param_version) {
            continue;
        }

        /* Return error if the param was too long */
        if (cfg->params[i].len > buf_size) {
            return FCS_CONFIG_PARAM_TOO_LONG;
        }

        /* Found the param and length is OK, set output value if non-null */
        if (out_param_idx) {
            *out_param_idx = i;
        }
        return FCS_CONFIG_PARAM_OK;
    }

    return FCS_CONFIG_PARAM_MISSING;
}

enum fcs_config_param_result_t fcs_config_get_param_block(fcs_config_t config,
uint16_t dst_size, uint8_t *dst, const uint8_t *restrict param_key,
uint16_t param_version) {
    assert(config == (fcs_config_t)&configs[0] ||
           config == (fcs_config_t)&configs[1]);

    struct fcs_config_block_t *cfg = (struct fcs_config_block_t *)config;
    assert(cfg->num_params <= FCS_CONFIG_NUM_PARAMS);

    uint16_t param_idx;
    enum fcs_config_param_result_t result;
    result = _fcs_config_find_param(config, dst_size, param_key,
                                    param_version, &param_idx);

    if (result != FCS_CONFIG_PARAM_OK) {
        return result;
    }

    assert(param_idx < cfg->num_params);
    assert(cfg->param_data[param_idx]);
    assert(cfg->params[param_idx].len < FCS_CONFIG_PARAM_SIZE);

    /* Got the param index, so copy the data to the output */
    memcpy(dst, cfg->param_data[param_idx], cfg->params[param_idx].len);

    return FCS_CONFIG_PARAM_OK;
}

enum fcs_config_param_result_t fcs_config_check_param_block(
fcs_config_t config, uint16_t buf_size, const uint8_t *restrict param_key,
uint16_t param_version) {
    return _fcs_config_find_param(config, buf_size, param_key,
                                  param_version, NULL);
}
