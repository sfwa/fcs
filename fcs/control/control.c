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
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <float.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../nmpc/cnmpc.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "control.h"
#include "../exports/exports.h"
#include "trajectory.h"


#ifdef __TI_COMPILER_VERSION__
#include <c6x.h>
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_gpio.h"
inline uint32_t cycle_count(void) {
    return TSCL;
}
#else
inline uint32_t cycle_count(void) {
    uint64_t result;
    __asm__ __volatile__ ("rdtsc" : "=A" (result));
    return (uint32_t)(result & UINT32_MAX);
}
#endif

inline static bool is_path_valid() {
    return fcs_global_nav_state.reference_path_id[0] !=
                FCS_CONTROL_INVALID_PATH_ID;
}

inline static bool is_navigating() {
    return is_path_valid() &&
           fcs_global_control_state.mode == FCS_CONTROL_MODE_AUTO;
}

inline static bool is_stabilising() {
    return fcs_global_nav_state.reference_path_id[0] ==
                FCS_CONTROL_STABILISE_PATH_ID;
}

inline static bool is_position_error_ok(float err) {
    return is_path_valid() &&
           (is_stabilising() || absval(err) < FCS_CONTROL_POSITION_TOLERANCE);
}


struct fcs_control_state_t fcs_global_control_state;
struct fcs_nav_state_t fcs_global_nav_state;

static uint32_t control_infeasibility_timer;
static uint32_t control_tick;


void fcs_control_init(void) {
    float state_weights[NMPC_DELTA_DIM] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float terminal_weights[NMPC_DELTA_DIM] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float control_weights[NMPC_CONTROL_DIM] = { 3e2, 5e2, 5e2 };
    float lower_control_bound[NMPC_CONTROL_DIM] = { 0.0f, 0.25f, 0.25f };
    float upper_control_bound[NMPC_CONTROL_DIM] = { 1.0f, 0.75f, 0.75f };

    /* Clear GPIO outs */
    fcs_global_control_state.gpio_state = 0;

    /*
    Configure throttle control: 0-12000 RPM, cruise at 9000, rate change at
    9000 RPM/sec
    */
    fcs_global_control_state.controls[0].setpoint = 0.6f;
    fcs_global_control_state.controls[0].rate = 0.5f;

    /*
    Configure left elevon: neutral setpoint, +/- 36 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[1].setpoint = 0.5f;
    fcs_global_control_state.controls[1].rate = 2.0f;

    /*
    Configure right elevon: neutral setpoint, +/- 36 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[2].setpoint = 0.5f;
    fcs_global_control_state.controls[2].rate = 2.0f;

    /* Set final (unused) control channel */
    fcs_global_control_state.controls[3].setpoint = 0.0f;
    fcs_global_control_state.controls[3].rate = 1.0f;

    /*
    Set up NMPC parameters -- state and control weights as well as control
    bounds
    */
    nmpc_set_state_weights(state_weights);
    nmpc_set_control_weights(control_weights);
    nmpc_set_terminal_weights(terminal_weights);
    nmpc_set_lower_control_bound(lower_control_bound);
    nmpc_set_upper_control_bound(upper_control_bound);

    /* Initialise the NMPC system */
    nmpc_init(true);

    /* Fill the horizon with zeros -- it'll be re-calculated next tick. */
    memset(fcs_global_nav_state.reference_trajectory, 0,
           sizeof(fcs_global_nav_state.reference_trajectory));

    /*
    Set the reference path to invalid, so it gets recalculated once the AHRS
    is ready.
    */
    memset(fcs_global_nav_state.reference_path_id, 0xFFu,
           sizeof(fcs_global_nav_state.reference_path_id));

    /* Configure the holding path */
    fcs_global_nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].start_waypoint_id =
        FCS_CONTROL_HOLD_WAYPOINT_ID;
    fcs_global_nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].end_waypoint_id =
        FCS_CONTROL_HOLD_WAYPOINT_ID;
    fcs_global_nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].type =
        FCS_PATH_FIGURE_EIGHT;
    fcs_global_nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].flags = 0;
    fcs_global_nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].next_path_id =
        FCS_CONTROL_HOLD_PATH_ID;

    control_infeasibility_timer = 0;
    control_tick = 0;

    fcs_global_control_state.mode = FCS_CONTROL_MODE_MANUAL;
}

void fcs_control_tick(void) {
    enum nmpc_result_t result;
    struct fcs_state_estimate_t state_estimate;
    struct fcs_nav_state_t *nav = &fcs_global_nav_state;
    float controls[NMPC_CONTROL_DIM], alt_diff;
    size_t i;
    uint32_t start_t = cycle_count();
    bool control_timeout = false;

    assert(fcs_global_control_state.mode != FCS_CONTROL_MODE_STARTUP_VALUE);

#ifdef __TI_COMPILER_VERSION__
    /* FIXME -- move this to the board definition file instead */
    volatile CSL_GpioRegs *const gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;
    if (gpio->BANK_REGISTERS[0].IN_DATA & 0x40u) {
        fcs_global_control_state.mode = FCS_CONTROL_MODE_AUTO;
    } else {
        fcs_global_control_state.mode = FCS_CONTROL_MODE_MANUAL;
    }
#else
    fcs_global_control_state.mode = FCS_CONTROL_MODE_AUTO;
#endif

    /*
    Check for multiple infeasible results in a row, and reset the trajectory
    if necessary.
    */
    if (control_tick - control_infeasibility_timer
            > FCS_CONTROL_INFEASIBILITY_TIMEOUT) {
        control_timeout = true;
        control_infeasibility_timer = control_tick;
        fcs_global_counters.nmpc_resets++;
    } else {
        control_tick++;
    }

    /*
    Read the relevant parts of the AHRS output and convert them to a format
    usable by the control system.

    Don't access fcs_global_ahrs_state directly, since the cache coherence of
    that structure is not guaranteed.
    */
    fcs_exports_recv_state(&state_estimate);

    alt_diff = absval(state_estimate.alt - nav->reference_trajectory[0].alt);

    /*
    Four options here:
    1. We're in manual mode -- just recalculate the trajectory each time as
       though we were going through recovery to a holding pattern.
    2. The current position is fairly near the expected position, so we're
       following the path OK.
    3. The current position is more than N metres from the expected position,
       so we need to add a new path to get back on track.
    4. There's no expected position, because the path hasn't been initialized.

    If option 2, we get the next point from the planner and update the last
    point in the horizon. This is the common case.

    If option 3, we effectively need to add a path between the current point
    and the first point in the reference trajectory. Once that's done we
    re-calculate the entire reference trajectory in one go.

    If option 4, we switch to the hold path around the current position.

    The offset between current and expected position is given by the first
    three elements of `state`, since all positions are given in NED relative
    to the first point in the reference trajectory.
    */
    if (fcs_global_control_state.mode == FCS_CONTROL_MODE_MANUAL) {
        /*
        While in manual mode, continuously try to enter a holding pattern
        */
        fcs_trajectory_start_hold(nav, &state_estimate);
        fcs_trajectory_recalculate(nav, &state_estimate);
        fcs_trajectory_timestep(nav, &state_estimate);
    } else if (!control_timeout && is_navigating() &&
               is_position_error_ok(alt_diff)) {
        fcs_trajectory_timestep(nav, &state_estimate);
    } else if (is_path_valid()) {
        /*
        If we're not already stabilising, construct a path sequence that gets
        the vehicle back to the next point in the reference trajectory.
        */
        fcs_trajectory_start_recover(nav, &state_estimate);
        fcs_trajectory_recalculate(nav, &state_estimate);
        fcs_trajectory_timestep(nav, &state_estimate);
    } else {
        /*
        Path uninitialized; enter a holding pattern.
        */
        fcs_trajectory_start_hold(nav, &state_estimate);
        fcs_trajectory_recalculate(nav, &state_estimate);
        fcs_trajectory_timestep(nav, &state_estimate);
    }

    /* Get the control values and update the global state. */
    result = nmpc_get_controls(controls);
    if (result == NMPC_OK) {
        control_infeasibility_timer = control_tick;
    } else {
    	fcs_global_counters.nmpc_errors++;
    }

    for (i = 0; i < NMPC_CONTROL_DIM; i++) {
        fcs_global_control_state.controls[i].setpoint = controls[i];
    }

    fcs_exports_send_control();

    fcs_global_counters.nmpc_last_cycle_count = cycle_count() - start_t;
    fcs_global_counters.nmpc_objective_value = nmpc_get_objective_value();
}
