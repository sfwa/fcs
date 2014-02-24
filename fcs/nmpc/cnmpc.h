/*
Copyright (C) 2013 Daniel Dyer

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

#ifndef CNMPC_H
#define CNMPC_H

#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

struct nmpc_state_t {
    real_t position[3];
    real_t velocity[3];
    real_t attitude[4]; /* x, y, z, W */
    real_t angular_velocity[3];
};

enum nmpc_result_t {
    NMPC_OK,
    NMPC_INFEASIBLE,
    NMPC_ERROR
};

/*
If state_position_delta is true, the position component of each state vector
in the reference trajectory is assumed to be relative to the previous state
vector.
*/
void nmpc_init(bool state_position_delta);
void nmpc_preparation_step(void);
void nmpc_feedback_step(float measurement[NMPC_STATE_DIM]);
enum nmpc_result_t nmpc_get_controls(float controls[NMPC_CONTROL_DIM]);
void nmpc_update_horizon(float new_reference[NMPC_REFERENCE_DIM]);

/* Functions for setting weights and bounds for the OCP solver. */
void nmpc_set_state_weights(float coeffs[NMPC_DELTA_DIM]);
void nmpc_set_control_weights(float coeffs[NMPC_CONTROL_DIM]);
void nmpc_set_terminal_weights(float coeffs[NMPC_DELTA_DIM]);
void nmpc_set_lower_control_bound(float coeffs[NMPC_CONTROL_DIM]);
void nmpc_set_upper_control_bound(float coeffs[NMPC_CONTROL_DIM]);
void nmpc_set_reference_point(float coeffs[NMPC_REFERENCE_DIM],
uint32_t i);

/* Function to set the wind estimate for the dynamics model. */
void nmpc_set_wind_velocity(float x, float y, float z);

/*
Functions to access the compiled configuration
*/
enum nmpc_precision_t {
    NMPC_PRECISION_FLOAT = 0,
    NMPC_PRECISION_DOUBLE = 1
};

uint32_t nmpc_config_get_state_dim(void);
uint32_t nmpc_config_get_control_dim(void);
uint32_t nmpc_config_get_horizon_length(void);
float nmpc_config_get_step_length(void);
enum nmpc_precision_t nmpc_config_get_precision(void);

#ifdef __cplusplus
}
#endif

#endif
