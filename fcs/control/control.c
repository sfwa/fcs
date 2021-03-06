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
inline static uint32_t cycle_count(void) {
    return TSCL;
}
#else
inline static uint32_t cycle_count(void) {
    uint64_t result;
    __asm__ __volatile__ ("rdtsc" : "=A" (result));
    return (uint32_t)(result & UINT32_MAX);
}
#endif

/* TODO: make static after debugging sitl.py */
extern struct fcs_control_state_t control_state;
extern struct fcs_nav_state_t nav_state;

struct fcs_control_state_t control_state;
struct fcs_nav_state_t nav_state;
static uint32_t control_infeasibility_timer;
static uint32_t control_tick;
static uint32_t control_hold_timer;
static uint32_t control_manual_timer;
static uint32_t control_auto_timer;
static float control_last_throttle;


static void _read_estimate_log(struct fcs_state_estimate_t *estimate,
int32_t *time_since_last_gps, int32_t *time_since_last_data);
static bool is_latlng_in_poly(double pt_lat, double pt_lng, float pt_alt,
size_t num_point_ids, uint16_t point_ids[], struct fcs_waypoint_t points[]);
static bool is_path_valid();
static bool is_stabilising();
static bool is_position_error_ok(float err);


static bool is_path_valid() {
    struct fcs_path_t *path;
    struct fcs_waypoint_t *start, *end;

    if (nav_state.reference_path_id[0] != FCS_CONTROL_INVALID_PATH_ID) {
        path = &nav_state.paths[nav_state.reference_path_id[0]];
        start = &nav_state.waypoints[path->start_waypoint_id];
        end = &nav_state.waypoints[path->end_waypoint_id];

        /* Make sure the path isn't too long */
        if (absval(start->lat - end->lat) * WGS84_A <
                    FCS_CONTROL_MAX_PATH_LENGTH &&
                absval(start->lon - end->lon) * WGS84_A <
                    FCS_CONTROL_MAX_PATH_LENGTH &&
                start->alt && end->alt &&
                !isnan(start->alt) && !isnan(end->alt)) {
            return true;
        }
    }

    return false;
}

static bool is_stabilising() {
    return nav_state.reference_path_id[0] == FCS_CONTROL_STABILISE_PATH_ID;
}

static bool is_position_error_ok(float err) {
    return is_path_valid() &&
           (is_stabilising() || absval(err) < FCS_CONTROL_POSITION_TOLERANCE);
}

#ifndef __TI_COMPILER_VERSION__
#include <stdio.h>
#endif

static void fcs_abort(const char *reason) {
#ifdef __TI_COMPILER_VERSION__
    fcs_assert(0);
#else
    printf("ABORT: %s", reason);
#endif
}

void fcs_control_init(void) {
    float state_weights[NMPC_DELTA_DIM] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float terminal_weights[NMPC_DELTA_DIM] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float control_weights[NMPC_CONTROL_DIM] = { 2e2, 3e1, 3e1 };
    float lower_control_bound[NMPC_CONTROL_DIM] = { 0.25f, 0.25f, 0.25f };
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
    control_manual_timer = 0;
    control_tick = 0;

    control_state.mode = FCS_CONTROL_MODE_MANUAL;
    control_state.intent = FCS_CONTROL_INTENT_NAVIGATING;
}

void fcs_control_tick(void) {
    uint16_t error_type_bits;
    enum nmpc_result_t result;
    struct fcs_state_estimate_t state_estimate;
    struct fcs_log_t *control_log, *measurement_log;
    struct fcs_path_t *path;
    struct fcs_waypoint_t *release_point;
    struct fcs_waypoint_t waypoint;
    struct fcs_parameter_t param, param2;
    float controls[NMPC_CONTROL_DIM], alt_diff, yaw, pitch, roll,
          release_pt_dist, next_release_pt_dist, ned[3];
    uint32_t start_t = cycle_count();
    int32_t ms_since_last_gps, ms_since_last_data;
    uint16_t manual_setpoint[NMPC_CONTROL_DIM];
    uint8_t param_key[4], i;
    bool control_timeout = false, needs_path_reset = false, do_release;

    fcs_assert(control_state.mode != FCS_CONTROL_MODE_STARTUP_VALUE);


    measurement_log = fcs_exports_log_open(FCS_LOG_TYPE_MEASUREMENT,
                                           FCS_MODE_READ);
    fcs_assert(measurement_log);

    /*
    Find the control mode -- switch to the mode identified in the packet if
    it's present, otherwise default to auto (safer).
    */
    if (fcs_parameter_find_by_type_and_device(
            measurement_log, FCS_PARAMETER_CONTROL_MODE, 1u, &param) &&
            param.data.u8[0] == 0u) {
        control_auto_timer = 0;
        control_manual_timer++;
        if (control_manual_timer > FCS_CONTROL_MANUAL_TRANSITION_TIMEOUT) {
            control_state.mode = FCS_CONTROL_MODE_MANUAL;
        }
    } else {
        control_manual_timer = 0;
        control_auto_timer++;
        if (control_auto_timer > FCS_CONTROL_AUTO_TRANSITION_TIMEOUT) {
            control_state.mode = FCS_CONTROL_MODE_AUTO;
        }
    }

    /*
    If we're in manual mode, extract the input control values and use those as
    the setpoint.
    */
    if (control_state.mode == FCS_CONTROL_MODE_MANUAL &&
            fcs_parameter_find_by_type_and_device(
                measurement_log, FCS_PARAMETER_CONTROL_POS, 1u, &param)) {
        memcpy(manual_setpoint, param.data.u16, sizeof(manual_setpoint));
    }

    /* Handle navigation state updates */
    needs_path_reset = false;
    for (i = 0; i < 2u; i++) {
        if (fcs_parameter_find_by_type_and_device(
                measurement_log, FCS_PARAMETER_NAV_VERSION, i, &param) &&
                param.data.u32[0] == nav_state.version + 1u) {
            /* Look for path and waypoint updates */
            if (fcs_parameter_find_by_type_and_device(
                        measurement_log, FCS_PARAMETER_NAV_PATH_ID, i, &param) &&
                    fcs_parameter_find_by_key_and_device(
                        measurement_log, FCS_PARAMETER_KEY_PATH, i, &param2) &&
                    param.data.u16[0] < FCS_CONTROL_MAX_PATHS) {
                /* Update path */
                (void)fcs_parameter_get_key_value(
                    param_key, (uint8_t*)&nav_state.paths[param.data.u16[0]],
                    sizeof(struct fcs_path_t), &param2);
                nav_state.version++;
            } else if (fcs_parameter_find_by_type_and_device(
                        measurement_log, FCS_PARAMETER_NAV_WAYPOINT_ID, i,
                        &param) &&
                    fcs_parameter_find_by_key_and_device(
                        measurement_log, FCS_PARAMETER_KEY_WAYPOINT, i,
                        &param2) &&
                    param.data.u16[0] < FCS_CONTROL_MAX_WAYPOINTS) {
                /* Update waypoint */
                (void)fcs_parameter_get_key_value(
                    param_key, (uint8_t*)&nav_state.waypoints[param.data.u16[0]],
                    sizeof(struct fcs_waypoint_t), &param2);
                nav_state.version++;
            } else if (fcs_parameter_find_by_type_and_device(
                        measurement_log, FCS_PARAMETER_NAV_PATH_ID, i, &param) &&
                    fcs_parameter_find_by_key_and_device(
                        measurement_log, FCS_PARAMETER_KEY_REROUTE, i,
                        &param2) &&
                    param.data.u16[0] < FCS_CONTROL_MAX_PATHS &&
                    nav_state.reference_path_id[0] != param.data.u16[0]) {
                /* Re-route to the new path */
                nav_state.reference_path_id[0] = param.data.u16[0];
                nav_state.version++;
                needs_path_reset = true;
                control_state.intent = FCS_CONTROL_INTENT_NAVIGATING;
            } else if (fcs_parameter_find_by_key_and_device(
                        measurement_log, FCS_PARAMETER_KEY_NAV_BOUNDARY, i,
                        &param)) {
                /* Update mission boundary */
                (void)fcs_parameter_get_key_value(
                    param_key, (uint8_t*)&nav_state.boundary,
                    sizeof(struct fcs_boundary_t), &param);
                nav_state.version++;
            } else if (fcs_parameter_find_by_key_and_device(
                        measurement_log, FCS_PARAMETER_KEY_ABORT, i,
                        &param)) {
                /* Abort */
                fcs_abort("Manual abort");
            }
        }
    }

    measurement_log = fcs_exports_log_close(measurement_log);
    fcs_assert(!measurement_log);

    /*
    Read the relevant parts of the AHRS output and convert them to a format
    usable by the control system.
    */
    _read_estimate_log(&state_estimate, &ms_since_last_gps,
                       &ms_since_last_data);

    alt_diff = absval(state_estimate.alt -
                      nav_state.reference_trajectory[0].alt);

    /* Check we're still inside the mission boundary */
    if (!is_latlng_in_poly(state_estimate.lat, state_estimate.lon,
                           state_estimate.alt,
                           nav_state.boundary.num_waypoint_ids,
                           nav_state.boundary.waypoint_ids,
                           nav_state.waypoints)) {
        /* Lock up so the AHRS core signals I/O boards to abort flight */
        fcs_abort("Mission boundary crossed");
    }

    if (control_state.mode == FCS_CONTROL_MODE_AUTO) {
        /* Handle loss of data link and loss of GPS when in autonomous mode */
        if (ms_since_last_gps > 1000 && ms_since_last_data > 10000) {
            /* Lock up */
            fcs_abort("Lost GPS and lost data link");
        }

        if (ms_since_last_gps > 30000) {
            /* Lock up */
            fcs_abort("Lost GPS > 30 sec");
        } else if (ms_since_last_gps > 1000) {
            /* Enter a holding pattern if we're not already in one */
            if (nav_state.reference_path_id[0] != FCS_CONTROL_HOLD_PATH_ID &&
                    nav_state.reference_path_id[0] !=
                    FCS_CONTROL_STABILISE_PATH_ID) {
                fcs_trajectory_start_hold(&nav_state, &state_estimate);
                fcs_trajectory_recalculate(&nav_state, &state_estimate);
            }
        }

        if (control_state.intent == FCS_CONTROL_INTENT_RETURNING_HOME &&
                control_hold_timer > 120000) {
            /* We've been holding at the HOME waypoint for 2 min -- abort */
            fcs_abort("Lost data link");
        } else if (control_state.intent == FCS_CONTROL_INTENT_RALLYING &&
                   control_hold_timer > 120000) {
            /*
            We've been holding at the RALLY waypoint for 2 min -- return
            home.
            */
            memcpy(&nav_state.waypoints[FCS_CONTROL_STABILISE_WAYPOINT_ID],
                   &nav_state.reference_trajectory[0],
                   sizeof(struct fcs_waypoint_t));
            memcpy(&nav_state.waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID],
                   &nav_state.waypoints[FCS_CONTROL_HOME_WAYPOINT_ID],
                   sizeof(struct fcs_waypoint_t));

            path = &nav_state.paths[FCS_CONTROL_RETURN_HOME_PATH_ID];
            path->start_waypoint_id = FCS_CONTROL_STABILISE_WAYPOINT_ID;
            path->end_waypoint_id = FCS_CONTROL_HOME_WAYPOINT_ID;
            path->next_path_id = FCS_CONTROL_HOLD_PATH_ID;
            path->type = FCS_PATH_DUBINS_CURVE;

            nav_state.reference_path_id[0] = FCS_CONTROL_RETURN_HOME_PATH_ID;
            fcs_trajectory_recalculate(&nav_state, &state_estimate);

            control_state.intent = FCS_CONTROL_INTENT_RETURNING_HOME;
        } else if (control_state.intent == FCS_CONTROL_INTENT_NAVIGATING &&
                   ms_since_last_data > 10000) {
            /* Lost the data link -- go to the RALLY waypoint */
            memcpy(&nav_state.waypoints[FCS_CONTROL_STABILISE_WAYPOINT_ID],
                   &nav_state.reference_trajectory[0],
                   sizeof(struct fcs_waypoint_t));
            memcpy(&nav_state.waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID],
                   &nav_state.waypoints[FCS_CONTROL_RALLY_WAYPOINT_ID],
                   sizeof(struct fcs_waypoint_t));

            path = &nav_state.paths[FCS_CONTROL_RALLY_PATH_ID];
            path->start_waypoint_id = FCS_CONTROL_STABILISE_WAYPOINT_ID;
            path->end_waypoint_id = FCS_CONTROL_RALLY_WAYPOINT_ID;
            path->next_path_id = FCS_CONTROL_HOLD_PATH_ID;
            path->type = FCS_PATH_DUBINS_CURVE;

            nav_state.reference_path_id[0] = FCS_CONTROL_RALLY_PATH_ID;
            fcs_trajectory_recalculate(&nav_state, &state_estimate);

            control_state.intent = FCS_CONTROL_INTENT_RALLYING;
        }
    }

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
        control_timeout = false;
    }

    /* FIXME: for debug logging */
    error_type_bits = (control_state.mode == FCS_CONTROL_MODE_MANUAL ? 0x1u : 0x0) +
                      (control_timeout ? 0x2u : 0x0) +
                      (needs_path_reset ? 0x4u : 0x0) +
                      (!is_path_valid() ? 0x8u : 0x0) +
                      (!is_position_error_ok(alt_diff) ? 0x10u : 0x0);

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
    if (control_state.mode == FCS_CONTROL_MODE_MANUAL || !is_path_valid()) {
        /*
        If in manual mode or the path is uninitialized, initialize the
        reference trajectory with the current state and enter a holding
        pattern.
        */
        yaw_pitch_roll_from_quaternion_f(&yaw, &pitch, &roll,
                                         state_estimate.attitude);

        nav_state.reference_trajectory[0].lat = state_estimate.lat;
        nav_state.reference_trajectory[0].lon = state_estimate.lon;
        nav_state.reference_trajectory[0].alt = state_estimate.alt;
        nav_state.reference_trajectory[0].yaw = yaw;
        nav_state.reference_trajectory[0].pitch = pitch;
        nav_state.reference_trajectory[0].roll = roll;
        nav_state.reference_trajectory[0].airspeed =
            FCS_CONTROL_DEFAULT_AIRSPEED;

        fcs_trajectory_start_hold(&nav_state, &state_estimate);
        fcs_trajectory_recalculate(&nav_state, &state_estimate);
    } else if (control_timeout || needs_path_reset ||
               !is_position_error_ok(alt_diff)) {
        /*
        If we're not already stabilising, construct a path sequence that gets
        the vehicle back to the next point in the reference trajectory.
        */
        fcs_trajectory_start_recover(&nav_state, &state_estimate,
                                     needs_path_reset);
        fcs_trajectory_recalculate(&nav_state, &state_estimate);
    }

    fcs_trajectory_timestep(&nav_state, &state_estimate);

    /* If we're holding, increment the hold timer; otherwise reset it */
    if (nav_state.reference_path_id[0] == FCS_CONTROL_HOLD_PATH_ID) {
        control_hold_timer += 20;
    } else {
        control_hold_timer = 0;
    }

    /* Get the control values and update the global state. */
    result = nmpc_get_controls(controls);
    if (result == NMPC_OK) {
        control_infeasibility_timer = control_tick;
    } else {
    	fcs_global_counters.nmpc_errors++;

        /*
        During infeasibility, scale the control deflection to 1/2 its usual
        value in order to avoid sudden movement.
        */
        controls[1] = (controls[1] - 0.5f) * 0.5f + 0.5f;
        controls[2] = (controls[2] - 0.5f) * 0.5f + 0.5f;
    }

    /*
    Check the throttle cut flag -- if it's set, cut the throttle immediately
    */
    if (nav_state.reference_trajectory[0].flags &
            FCS_WAYPOINT_FLAG_THROTTLE_CUT_MASK) {
        controls[0] = 0.0f;
        control_last_throttle = 0.0f;
    }

    control_log = fcs_exports_log_open(FCS_LOG_TYPE_CONTROL, FCS_MODE_WRITE);
    fcs_assert(control_log);

    /* Write the setpoints to the control log */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, 3u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_SETPOINT);
    fcs_parameter_set_device_id(&param, 0);
    if (control_state.mode == FCS_CONTROL_MODE_AUTO) {
        /* Use auto setpoints */
        controls[0] = control_last_throttle + (controls[0] - control_last_throttle) * 0.1f;
        control_last_throttle = controls[0];
        param.data.u16[0] = (uint16_t)(controls[0] * (float)UINT16_MAX);
        param.data.u16[1] = (uint16_t)(controls[1] * (float)UINT16_MAX);
        param.data.u16[2] = (uint16_t)(controls[2] * (float)UINT16_MAX);
    } else {
        /* Use manual setpoints */
        param.data.u16[0] = manual_setpoint[0];
        param.data.u16[1] = manual_setpoint[1];
        param.data.u16[2] = manual_setpoint[2];
    }
    fcs_log_add_parameter(control_log, &param);

    /*
    Add GPIO to the control log if everything's stable and a payload release
    is requested -- if the first or last point is a release path, then the
    release point must be somewhere along the trajectory.

    If:
    1. there is a release point in the current horizon, and
    2. the current position plus current velocity would be further from the
       release point than the current position is,
    release the payload.
    */
    if (result == NMPC_OK &&
        ((nav_state.reference_path_id[0] < FCS_CONTROL_MAX_PATHS &&
            nav_state.paths[nav_state.reference_path_id[0]].type ==
            FCS_PATH_RELEASE) ||
         (nav_state.reference_path_id[OCP_HORIZON_LENGTH] < FCS_CONTROL_MAX_PATHS &&
            nav_state.paths[nav_state.reference_path_id[OCP_HORIZON_LENGTH]].type ==
            FCS_PATH_RELEASE))) {
        path = &nav_state.paths[nav_state.reference_path_id[0]];
        if (path->type != FCS_PATH_RELEASE) {
            /* Release path must have been at the end of the trajectory */
            path = &nav_state.paths[nav_state.reference_path_id[OCP_HORIZON_LENGTH]];
        }

        release_point = &nav_state.waypoints[path->start_waypoint_id];
        waypoint.lat = state_estimate.lat;
        waypoint.lon = state_estimate.lon;
        waypoint.alt = state_estimate.alt;

        _ned_from_point_diff(ned, release_point, &waypoint);
        ned[Z] = 0.0f;
        release_pt_dist = vector3_norm_f(ned);

        ned[X] += state_estimate.velocity[X] * (1.0f / (float)OCP_HORIZON_LENGTH);
        ned[Y] += state_estimate.velocity[Y] * (1.0f / (float)OCP_HORIZON_LENGTH);
        next_release_pt_dist = vector3_norm_f(ned);

        if (next_release_pt_dist >= release_pt_dist) {
            do_release = true;
        } else {
            do_release = false;
        }
    } else {
        do_release = false;
    }

    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 8u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_GP_OUT);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u8[0] = do_release ? 0xFu : 0x0u;
    fcs_log_add_parameter(control_log, &param);

    /*
    Calculate cycle count and objective value, then add them to the control
    log as float32s
    */
    fcs_global_counters.nmpc_last_cycle_count = cycle_count() - start_t;
    fcs_global_counters.nmpc_objective_value = nmpc_get_objective_value();

    fcs_parameter_set_header(&param, FCS_VALUE_FLOAT, 32u, 4u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_CONTROL_STATUS);
    fcs_parameter_set_device_id(&param, 0);
    param.data.f32[0] = fcs_global_counters.nmpc_last_cycle_count;
    param.data.f32[1] = fcs_global_counters.nmpc_objective_value;
    param.data.f32[2] = fcs_global_counters.nmpc_errors;
    param.data.f32[3] = fcs_global_counters.nmpc_resets;
    fcs_log_add_parameter(control_log, &param);

    /*
    Add navigation status to the control log -- path ID, nav state version,
    and a reference waypoint packet
    */
    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 16u, 2u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_NAV_PATH_ID);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u16[0] = nav_state.reference_path_id[0];
    param.data.u16[1] = error_type_bits;
    fcs_log_add_parameter(control_log, &param);

    fcs_parameter_set_header(&param, FCS_VALUE_UNSIGNED, 32u, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_NAV_VERSION);
    fcs_parameter_set_device_id(&param, 0);
    param.data.u32[0] = nav_state.version;
    fcs_log_add_parameter(control_log, &param);

    fcs_parameter_set_key_value(
        &param, FCS_PARAMETER_KEY_REFERENCE_POINT,
        (uint8_t*)&nav_state.reference_trajectory[0],
        sizeof(nav_state.reference_trajectory[0]));
    fcs_parameter_set_device_id(&param, 0);
    fcs_log_add_parameter(control_log, &param);

    control_log = fcs_exports_log_close(control_log);
    fcs_assert(!control_log);
}

void fcs_control_reset(void) {
    struct fcs_state_estimate_t state_estimate;
    int32_t tmp1, tmp2;
    _read_estimate_log(&state_estimate, &tmp1, &tmp2);
    fcs_trajectory_start_recover(&nav_state, &state_estimate, true);
    fcs_trajectory_recalculate(&nav_state, &state_estimate);
    control_state.intent = FCS_CONTROL_INTENT_NAVIGATING;
}

/*
Copy estimate values from the estimate log to the state estimate structure.
See _populate_estimate_log at ahrs/ahrs.c:329.
*/
static void _read_estimate_log(struct fcs_state_estimate_t *estimate,
int32_t *time_since_last_gps, int32_t *time_since_last_data) {
    fcs_assert(estimate);

    struct fcs_log_t *estimate_log;
    struct fcs_parameter_t param;

    estimate_log = fcs_exports_log_open(FCS_LOG_TYPE_ESTIMATE, FCS_MODE_READ);
    fcs_assert(estimate_log);

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_AHRS_STATUS, 0, &param)) {
        *time_since_last_gps = param.data.i16[0];
        *time_since_last_data = param.data.i16[1];
    } else {
        *time_since_last_gps = 0;
        *time_since_last_data = 0;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_POSITION_LLA, 0, &param)) {
        estimate->lat = (double)param.data.i32[0] *
                        (M_PI / (double)INT32_MAX);
        estimate->lon = (double)param.data.i32[1] *
                        (M_PI / (double)INT32_MAX);
        estimate->alt = (float)param.data.i32[2] * 1e-2f;
    } else {
        /* FIXME */
        estimate->lat = 0.0;
        estimate->lon = 0.0;
        estimate->alt = 0.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_VELOCITY_NED, 0, &param)) {
        estimate->velocity[0] = (float)param.data.i16[0] * 1e-2f;
        estimate->velocity[1] = (float)param.data.i16[1] * 1e-2f;
        estimate->velocity[2] = (float)param.data.i16[2] * 1e-2f;
    } else {
        /* FIXME */
        estimate->velocity[0] = estimate->velocity[1] = estimate->velocity[2]
            = 0.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_ATTITUDE_Q, 0, &param)) {
        estimate->attitude[X] = (float)param.data.i16[X] *
                                (1.0f / (float)INT16_MAX);
        estimate->attitude[Y] = (float)param.data.i16[Y] *
                                (1.0f / (float)INT16_MAX);
        estimate->attitude[Z] = (float)param.data.i16[Z] *
                                (1.0f / (float)INT16_MAX);
        estimate->attitude[W] = (float)param.data.i16[W] *
                                (1.0f / (float)INT16_MAX);

        quaternion_normalize_f(estimate->attitude, estimate->attitude, true);
    } else {
        /* FIXME */
        estimate->attitude[X] = estimate->attitude[Y] = estimate->attitude[Z]
            = 0.0f;
        estimate->attitude[W] = 1.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ, 0,
            &param)) {
        estimate->angular_velocity[X] = (float)param.data.i16[X] * 4.0f *
                                        ((float)M_PI / (float)INT16_MAX);
        estimate->angular_velocity[Y] = (float)param.data.i16[Y] * 4.0f *
                                        ((float)M_PI / (float)INT16_MAX);
        estimate->angular_velocity[Z] = (float)param.data.i16[Z] * 4.0f *
                                        ((float)M_PI / (float)INT16_MAX);
    } else {
        /* FIXME */
        estimate->angular_velocity[0] = estimate->angular_velocity[1] =
            estimate->angular_velocity[2] = 0.0f;
    }

    if (fcs_parameter_find_by_type_and_device(
            estimate_log, FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED, 0,
            &param)) {
        estimate->wind_velocity[0] = (float)param.data.i16[0] * 1e-2f;
        estimate->wind_velocity[1] = (float)param.data.i16[1] * 1e-2f;
        estimate->wind_velocity[2] = (float)param.data.i16[2] * 1e-2f;
    } else {
        /* FIXME */
        estimate->wind_velocity[0] = estimate->wind_velocity[1] =
            estimate->wind_velocity[2] = 0.0f;
    }

    /* TODO: set mode */

    estimate_log = fcs_exports_log_close(estimate_log);
    fcs_assert(!estimate_log);

/*
    printf("estimate: %13.9f %13.9f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f"
           " %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\n",
           estimate->lat, estimate->lon, estimate->alt, estimate->velocity[0],
           estimate->velocity[1], estimate->velocity[2],
           estimate->attitude[0], estimate->attitude[1],
           estimate->attitude[2], estimate->attitude[3],
           estimate->angular_velocity[0], estimate->angular_velocity[1],
           estimate->angular_velocity[2], estimate->wind_velocity[0],
           estimate->wind_velocity[1], estimate->wind_velocity[2]);
*/
}

static bool is_latlng_in_poly(double pt_lat, double pt_lng, float pt_alt,
size_t num_point_ids, uint16_t point_ids[], struct fcs_waypoint_t points[]) {
    /*
    Check lat/long is within boundary.
    See http://msdn.microsoft.com/en-us/library/cc451895.aspx

    Scaling of all lat and lng points is 1/10,000,000 of a degree.

    Does not work at the poles, and probably also doesn't work across the
    -180/180 split. Also doesn't do any correction for spherical surfaces so
    won't work for very large polygons (more than a few degrees across).
    */
    fcs_assert(!num_point_ids || (points && point_ids));
    fcs_assert(num_point_ids <= FCS_CONTROL_BOUNDARY_MAX_WAYPOINTS);

    uint8_t in_bounds = 0;
    size_t i, j;
    double i_lat, i_lng, j_lat, j_lng, ij_grad;

    if (num_point_ids == 0) {
        return true;
    }

    for (i = 0, j = num_point_ids - 1; i < num_point_ids; i++) {
        fcs_assert(point_ids[i] < FCS_CONTROL_MAX_WAYPOINTS);
        fcs_assert(point_ids[j] < FCS_CONTROL_MAX_WAYPOINTS);

        /* Altitude check */
        if (points[point_ids[i]].alt < pt_alt) {
            return false;
        }

        /*
        Only validate i points because each point in the polygon is used as
        both j and i
        */
        i_lat = points[point_ids[i]].lat;
        i_lng = points[point_ids[i]].lon;
        j_lat = points[point_ids[j]].lat;
        j_lng = points[point_ids[j]].lon;

        /*
        Check if line segment points are either side of the POI,
        longitudinally
        */
        if ((i_lng < pt_lng && j_lng >= pt_lng) ||
                (j_lng < pt_lng && i_lng >= pt_lng)) {
            /* See which side of the line segment the point is on */
            ij_grad = (pt_lng - i_lng) * (j_lat - i_lat) / (j_lng - i_lng);

            if (i_lat + ij_grad < pt_lat) {
                in_bounds = ~in_bounds;
            }
        }
        j = i;
    }

    return in_bounds != 0;
}
