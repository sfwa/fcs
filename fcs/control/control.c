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
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "control.h"

#include "../nmpc/config.h"
#include "../nmpc/cnmpc.h"

struct fcs_control_state_t fcs_global_control_state;

void fcs_control_init(void) {
    float state_weights[NMPC_DELTA_DIM] = {
        1e-1, 1e-1, 1e1, 1e0, 1e0, 1e0, 1e-1, 1e0, 1e0, 7e0, 7e-1, 1e-2
    };
    float terminal_weights[NMPC_DELTA_DIM] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float control_weights[NMPC_CONTROL_DIM] = { 1e-7, 1e-3, 1e-3 };
    float lower_control_bound[NMPC_CONTROL_DIM];
    float upper_control_bound[NMPC_CONTROL_DIM];
    float reference[NMPC_REFERENCE_DIM];
    size_t i;

    /* Clear GPIO outs */
    fcs_global_control_state.gpio_state = 0;

    /*
    Configure throttle control: 0-12000 RPM, cruise at 9000, rate change at
    9000 RPM/sec
    */
    fcs_global_control_state.controls[0].setpoint = 9000.0f;
    fcs_global_control_state.controls[0].min = 0.0f;
    fcs_global_control_state.controls[0].max = 12000.0f;
    fcs_global_control_state.controls[0].rate = 9000.0f;

    /*
    Configure left elevon: neutral setpoint, +/- 45 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[1].setpoint = 0.0f;
    fcs_global_control_state.controls[1].min = -M_PI * 0.25f;
    fcs_global_control_state.controls[1].max = M_PI * 0.25f;
    fcs_global_control_state.controls[1].rate = M_PI * 0.3333333f;

    /*
    Configure right elevon: neutral setpoint, +/- 45 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[2].setpoint = 0.0f;
    fcs_global_control_state.controls[2].min = -M_PI * 0.25f;
    fcs_global_control_state.controls[2].max = M_PI * 0.25f;
    fcs_global_control_state.controls[2].rate = M_PI * 0.3333333f;

    /* Set final (unused) control channel */
    fcs_global_control_state.controls[3].setpoint = 0.0f;
    fcs_global_control_state.controls[3].min = 0.0f;
    fcs_global_control_state.controls[3].max = 1.0f;
    fcs_global_control_state.controls[3].rate = 1.0f;

    /*
    Set up NMPC parameters -- state and control weights as well as control
    bounds
    */
    for (i = 0; i < NMPC_CONTROL_DIM; i++) {
        lower_control_bound[i] = fcs_global_control_state.controls[i].min;
        upper_control_bound[i] = fcs_global_control_state.controls[i].max;
    }

    nmpc_set_state_weights(state_weights);
    nmpc_set_control_weights(control_weights);
    nmpc_set_terminal_weights(terminal_weights);
    nmpc_set_lower_control_bound(lower_control_bound);
    nmpc_set_upper_control_bound(upper_control_bound);

    /* Initialise the NMPC system */
    nmpc_init();

    /* TODO: Fill the horizon based on the current path */
    for (i = 0; i < OCP_HORIZON_LENGTH; i++) {
        nmpc_set_reference_point(reference, i);
    }
}

void fcs_control_tick(void) {
    enum nmpc_result_t result;
    float controls[NMPC_CONTROL_DIM], state[NMPC_STATE_DIM],
          reference[NMPC_REFERENCE_DIM], wind[3];
    size_t i;

    /*
    Run preparation -- this does nothing in the current implementation but
    might in future.
    */
    nmpc_preparation_step();

    /*
    Get the latest data from the AHRS. Since we don't lock anything here, it's
    possible for the UKF to start updating the data under us, but the
    worst-case scenario is that we get some data from 1ms later, which
    shouldn't be an issue.

    TODO: Work out what to do with the lat/lon/alt to NED conversion -- base
    everything on a static reference point, or treat vehicle coordinates as
    0, 0, 0 and shift the reference trajectory each time?
    */
    state[0] = 0.0f;
    state[1] = 0.0f;
    state[2] = 0.0f;
    state[3] = fcs_global_ahrs_state.velocity[0];
    state[4] = fcs_global_ahrs_state.velocity[1];
    state[5] = fcs_global_ahrs_state.velocity[2];
    state[6] = fcs_global_ahrs_state.attitude[0];
    state[7] = fcs_global_ahrs_state.attitude[1];
    state[8] = fcs_global_ahrs_state.attitude[2];
    state[9] = fcs_global_ahrs_state.attitude[3];
    state[10] = fcs_global_ahrs_state.angular_velocity[0];
    state[11] = fcs_global_ahrs_state.angular_velocity[1];
    state[12] = fcs_global_ahrs_state.angular_velocity[2];

    wind[0] = fcs_global_ahrs_state.wind_velocity[0];
    wind[1] = fcs_global_ahrs_state.wind_velocity[1];
    wind[2] = fcs_global_ahrs_state.wind_velocity[2];

    /*
    Feedback with the latest state data. This solves the QP and generates the
    control values. Set the wind based on the latest UKF estimate as well.
    */
    nmpc_set_wind_velocity(wind[0], wind[1], wind[2]);
    nmpc_feedback_step(state);

    /* Get the control values and update the global state. */
    result = nmpc_get_controls(controls);
    for (i = 0; i < NMPC_CONTROL_DIM; i++) {
        fcs_global_control_state.controls[i].setpoint = controls[i];
    }

    /*
    Update the horizon with the next reference trajectory step. The first
    NMPC_STATE_DIM values are the reference state (position, velocity,
    attitude, angular velocity), and the next NMPC_CONTROL_DIM values are the
    reference control values.
    */
    reference[NMPC_STATE_DIM + 0] = 9000.0f;
    reference[NMPC_STATE_DIM + 1u] = 0.0f;
    reference[NMPC_STATE_DIM + 2u] = 0.0f;
    nmpc_update_horizon(reference);
}

void _start_plan(struct fcs_plan_t *plan) {
    /*
    TODO:
    Path definition -- waypoint IDs and connecting line type.

    Interpolation state -- track current segment and t parameter along segment
    based on position (10m waypoint crossing and 10m cross-track error).

    If > 10m from the path, add a curve segment from the current
    position/heading to the next unvisited point.

    Interpolation function -- return a sequence of points separated by N
    metres; entire plan conducted at a set speed.

    Line interpolation -- linear interpolation between points; reference
    heading and speed derived from the deltas.

    Dubins curve interpolation type -- see
    https://github.com/AndrewWalker/Dubins-Curves/blob/master/src/dubins.c
    */
}
