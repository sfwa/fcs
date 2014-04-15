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

#include "../util/util.h"
#include "../util/3dmath.h"
#include "../nmpc/cnmpc.h"
#include "../stats/stats.h"
#include "control.h"
#include "../exports/exports.h"
#include "../exports/parameter.h"
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


static struct fcs_control_state_t control_state;
static struct fcs_nav_state_t nav_state;
static uint32_t control_infeasibility_timer;
static uint32_t control_tick;


inline static bool is_path_valid() {
    return nav_state.reference_path_id[0] != FCS_CONTROL_INVALID_PATH_ID;
}

inline static bool is_navigating() {
    return is_path_valid() && control_state.mode == FCS_CONTROL_MODE_AUTO;
}

inline static bool is_stabilising() {
    return nav_state.reference_path_id[0] == FCS_CONTROL_STABILISE_PATH_ID;
}

inline static bool is_position_error_ok(float err) {
    return is_path_valid() &&
           (is_stabilising() || absval(err) < FCS_CONTROL_POSITION_TOLERANCE);
}

static void _read_estimate_log(struct fcs_state_estimate_t *estimate);


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
    control_state.gpio_state = 0;

    /*
    Configure throttle control: 0-12000 RPM, cruise at 9000, rate change at
    9000 RPM/sec
    */
    control_state.controls[0].setpoint = 0.6f;
    control_state.controls[0].rate = 0.5f;

    /*
    Configure left elevon: neutral setpoint, +/- 36 deg travel, 60 deg/s rate
    */
    control_state.controls[1].setpoint = 0.5f;
    control_state.controls[1].rate = 2.0f;

    /*
    Configure right elevon: neutral setpoint, +/- 36 deg travel, 60 deg/s rate
    */
    control_state.controls[2].setpoint = 0.5f;
    control_state.controls[2].rate = 2.0f;

    /* Set final (unused) control channel */
    control_state.controls[3].setpoint = 0.0f;
    control_state.controls[3].rate = 1.0f;

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
    memset(nav_state.reference_trajectory, 0,
           sizeof(nav_state.reference_trajectory));

    /*
    Set the reference path to invalid, so it gets recalculated once the AHRS
    is ready.
    */
    memset(nav_state.reference_path_id, 0xFFu,
           sizeof(nav_state.reference_path_id));

    /* Configure the holding path */
    nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].start_waypoint_id =
        FCS_CONTROL_HOLD_WAYPOINT_ID;
    nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].end_waypoint_id =
        FCS_CONTROL_HOLD_WAYPOINT_ID;
    nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].type = FCS_PATH_FIGURE_EIGHT;
    nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].flags = 0;
    nav_state.paths[FCS_CONTROL_HOLD_PATH_ID].next_path_id =
        FCS_CONTROL_HOLD_PATH_ID;

    control_infeasibility_timer = 0;
    control_tick = 0;

    control_state.mode = FCS_CONTROL_MODE_MANUAL;
}

void fcs_control_tick(void) {
    enum nmpc_result_t result;
    struct fcs_state_estimate_t state_estimate;
    struct fcs_log_t *control_log;
    struct fcs_parameter_t param;
    float controls[NMPC_CONTROL_DIM], alt_diff;
    uint32_t start_t = cycle_count();
    bool control_timeout = false;

    assert(control_state.mode != FCS_CONTROL_MODE_STARTUP_VALUE);

#ifdef __TI_COMPILER_VERSION__
    /* FIXME -- move this to the board definition file instead */
    volatile CSL_GpioRegs *const gpio = (CSL_GpioRegs*)CSL_GPIO_REGS;
    if (gpio->BANK_REGISTERS[0].IN_DATA & 0x40u) {
        control_state.mode = FCS_CONTROL_MODE_AUTO;
    } else {
        control_state.mode = FCS_CONTROL_MODE_MANUAL;
    }
#else
    control_state.mode = FCS_CONTROL_MODE_AUTO;
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
    TODO: Handle path and waypoint updates
    fcs_exports_recv_waypoint_update(&waypoint_update);
    if (waypoint_update.nav_state_version > nav_state.version) {
        assert(waypoint_update.waypoint_id < FCS_CONTROL_MAX_WAYPOINTS);

        nav_state.waypoints[waypoint_update.waypoint_id] =
            waypoint_update.waypoint;
        nav_state.version = waypoint_update.nav_state_version;
    }

    fcs_exports_recv_path_update(&path_update);
    if (path_update.nav_state_version > nav_state.version) {
        assert(path_update.path_id < FCS_CONTROL_MAX_PATHS);

        nav_state.paths[path_update.path_id] = path_update.path;
        nav_state.version = path_update.nav_state_version;
    }
    */

    /*
    Read the relevant parts of the AHRS output and convert them to a format
    usable by the control system.
    */
    _read_estimate_log(&state_estimate);

    alt_diff = absval(state_estimate.alt -
                      nav_state.reference_trajectory[0].alt);

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
    if (control_state.mode == FCS_CONTROL_MODE_MANUAL) {
        /*
        While in manual mode, continuously try to enter a holding pattern
        */
        fcs_trajectory_start_hold(&nav_state, &state_estimate);
        fcs_trajectory_recalculate(&nav_state, &state_estimate);
        fcs_trajectory_timestep(&nav_state, &state_estimate);
    } else if (!control_timeout && is_navigating() &&
               is_position_error_ok(alt_diff)) {
        fcs_trajectory_timestep(&nav_state, &state_estimate);
    } else if (is_path_valid()) {
        /*
        If we're not already stabilising, construct a path sequence that gets
        the vehicle back to the next point in the reference trajectory.
        */
        fcs_trajectory_start_recover(&nav_state, &state_estimate);
        fcs_trajectory_recalculate(&nav_state, &state_estimate);
        fcs_trajectory_timestep(&nav_state, &state_estimate);
    } else {
        /*
        Path uninitialized; enter a holding pattern.
        */
        fcs_trajectory_start_hold(&nav_state, &state_estimate);
        fcs_trajectory_recalculate(&nav_state, &state_estimate);
        fcs_trajectory_timestep(&nav_state, &state_estimate);
    }

    /* Get the control values and update the global state. */
    result = nmpc_get_controls(controls);
    if (result == NMPC_OK) {
        control_infeasibility_timer = control_tick;
    } else {
    	fcs_global_counters.nmpc_errors++;
    }

    /* Write the output to the control log */
    control_log = fcs_exports_log_open(FCS_LOG_TYPE_CONTROL, FCS_MODE_WRITE);
    assert(control_log);

    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, 3u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_SETPOINT);
    fcs_parameter_set_device_id(&param, 0);

    param.data.u16[0] = (uint16_t)(controls[0] * (float)UINT16_MAX);
    param.data.u16[1] = (uint16_t)(controls[1] * (float)UINT16_MAX);
    param.data.u16[2] = (uint16_t)(controls[2] * (float)UINT16_MAX);

    fcs_log_add_parameter(control_log, &param);

    control_log = fcs_exports_log_close(control_log);
    assert(!control_log);

    fcs_global_counters.nmpc_last_cycle_count = cycle_count() - start_t;
    fcs_global_counters.nmpc_objective_value = nmpc_get_objective_value();
}

/*
Copy estimate values from the estimate log to the state estimate structure.
See _populate_estimate_log at ahrs/ahrs.c:329.
*/
static void _read_estimate_log(struct fcs_state_estimate_t *estimate) {
    assert(estimate);

    struct fcs_log_t *estimate_log;
    struct fcs_parameter_t param;

    estimate_log = fcs_exports_log_open(FCS_LOG_TYPE_ESTIMATE, FCS_MODE_READ);
    assert(estimate_log);

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_POSITION_LLA, 0, &param)) {
        estimate->lat = (double)param.data.i32[0] *
                        (M_PI / (double)INT32_MAX);
        estimate->lon = (double)param.data.i32[1] *
                        (M_PI / (double)INT32_MAX);
        estimate->alt = (float)param.data.i32[2] * 1e2f;
    } else {
        /* FIXME */
        estimate->lat = 0.0;
        estimate->lon = 0.0;
        estimate->alt = 0.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_VELOCITY_NED, 0, &param)) {
        estimate->velocity[0] = (float)param.data.i32[0] * 1e2f;
        estimate->velocity[1] = (float)param.data.i32[1] * 1e2f;
        estimate->velocity[2] = (float)param.data.i32[2] * 1e2f;
    } else {
        /* FIXME */
        estimate->velocity[0] = estimate->velocity[1] = estimate->velocity[2]
            = 0.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_ATTITUDE_Q, 0, &param)) {
        estimate->attitude[X] = (float)param.data.i32[X] *
                                (1.0f / (float)INT16_MAX);
        estimate->attitude[Y] = (float)param.data.i32[Y] *
                                (1.0f / (float)INT16_MAX);
        estimate->attitude[Z] = (float)param.data.i32[Z] *
                                (1.0f / (float)INT16_MAX);
        estimate->attitude[W] = (float)param.data.i32[W] *
                                (1.0f / (float)INT16_MAX);
    } else {
        /* FIXME */
        estimate->attitude[X] = estimate->attitude[Y] = estimate->attitude[Z]
            = 0.0f;
        estimate->attitude[W] = 1.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ, 0,
            &param)) {
        estimate->angular_velocity[X] = (float)param.data.i32[X] * 2.0f *
                                        ((float)M_PI / (float)INT16_MAX);
        estimate->angular_velocity[Y] = (float)param.data.i32[Y] * 2.0f *
                                        ((float)M_PI / (float)INT16_MAX);
        estimate->angular_velocity[Z] = (float)param.data.i32[Z] * 2.0f *
                                        ((float)M_PI / (float)INT16_MAX);
    } else {
        /* FIXME */
        estimate->angular_velocity[0] = estimate->angular_velocity[1] =
            estimate->angular_velocity[2] = 0.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED, 0,
            &param)) {
        estimate->wind_velocity[0] = (float)param.data.i32[0] * 1e2f;
        estimate->wind_velocity[1] = (float)param.data.i32[1] * 1e2f;
        estimate->wind_velocity[2] = (float)param.data.i32[2] * 1e2f;
    } else {
        /* FIXME */
        estimate->wind_velocity[0] = estimate->wind_velocity[1] =
            estimate->wind_velocity[2] = 0.0f;
    }

    /* TODO: set mode mode */

    estimate_log = fcs_exports_log_close(estimate_log);
    assert(!estimate_log);
}
