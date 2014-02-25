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

#include "../nmpc/config.h"
#include "../nmpc/cnmpc.h"


#define WGS84_A 6378137.0

#pragma DATA_SECTION(fcs_global_control_state, ".shared")
struct fcs_control_state_t fcs_global_control_state;

#pragma DATA_SECTION(fcs_global_nav_state, ".shared")
struct fcs_nav_state_t fcs_global_nav_state;


float _interpolate_linear(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

float _interpolate_figure_eight(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

bool _plan_dubins_rsr(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b);

bool _plan_dubins_lsl(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b);

bool _plan_dubins_rsl(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b);

bool _plan_dubins_lsr(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b);

void _calculate_dubins_r(float *out, const float *reference, float t);

void _calculate_dubins_l(float *out, const float *reference, float t);

void _calculate_dubins_s(float *out, const float *reference, float t);

float _interpolate_dubins(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

float _next_point_from_path(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
enum fcs_path_type_t type, float t);

void _ned_from_point_diff(float *restrict ned,
const struct fcs_waypoint_t *restrict ref,
const struct fcs_waypoint_t *restrict point);

void _get_ahrs_state(float *restrict state, float *restrict wind,
volatile struct fcs_ahrs_state_t *ahrs_state,
const struct fcs_waypoint_t *restrict reference);

void _make_reference(float *restrict reference,
const struct fcs_waypoint_t *current_point,
const struct fcs_waypoint_t *last_point, const struct fcs_waypoint_t *start,
const float *restrict wind);

void _next_point(struct fcs_waypoint_t *restrict new_point,
uint16_t *restrict new_point_path_id,
const struct fcs_waypoint_t *restrict last_point,
const uint16_t *restrict last_point_path_id, const float *restrict wind,
struct fcs_nav_state_t *nav);

void _shift_horizon(struct fcs_nav_state_t *nav, const float *restrict wind);

void _recalculate_horizon(struct fcs_nav_state_t *nav,
const float *restrict wind);


void fcs_control_init(void) {
    float state_weights[NMPC_DELTA_DIM] = {
        1e1, 1e1, 1e2, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float terminal_weights[NMPC_DELTA_DIM] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float control_weights[NMPC_CONTROL_DIM] = { 1e1, 5e2, 5e2 };
    float lower_control_bound[NMPC_CONTROL_DIM];
    float upper_control_bound[NMPC_CONTROL_DIM];
    size_t i;

    /* Clear GPIO outs */
    fcs_global_control_state.gpio_state = 0;

    /*
    Configure throttle control: 0-12000 RPM, cruise at 9000, rate change at
    9000 RPM/sec
    */
    fcs_global_control_state.controls[0].setpoint = 0.6f;
    fcs_global_control_state.controls[0].min = 0.0f;
    fcs_global_control_state.controls[0].max = 1.0f;
    fcs_global_control_state.controls[0].rate = 0.5f;

    /*
    Configure left elevon: neutral setpoint, +/- 36 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[1].setpoint = 0.5f;
    fcs_global_control_state.controls[1].min = 0.0f;
    fcs_global_control_state.controls[1].max = 1.0f;
    fcs_global_control_state.controls[1].rate = 3.0f;

    /*
    Configure right elevon: neutral setpoint, +/- 36 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[2].setpoint = 0.5f;
    fcs_global_control_state.controls[2].min = 0.0f;
    fcs_global_control_state.controls[2].max = 1.0f;
    fcs_global_control_state.controls[2].rate = 3.0f;

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
}

void fcs_control_tick(void) {
    /*
    Early return -- if we're not in armed/active/holding there's nothing much
    to do.
    */
    if (fcs_global_ahrs_state.mode != FCS_MODE_ARMED &&
        fcs_global_ahrs_state.mode != FCS_MODE_ACTIVE &&
        fcs_global_ahrs_state.mode != FCS_MODE_HOLDING) {
        return;
    }

    enum nmpc_result_t result;
    struct fcs_nav_state_t *nav = &fcs_global_nav_state;
    float controls[NMPC_CONTROL_DIM], state[NMPC_STATE_DIM], wind[3];
    uint16_t original_path_id;
    struct fcs_waypoint_t *waypoint;
    size_t i;

    /*
    Run preparation -- this does nothing in the current implementation but
    might in future.
    */
    nmpc_preparation_step();

    /*
    Read the relevant parts of the AHRS output and convert them to a format
    usable by the control system.
    */
    _get_ahrs_state(state, wind, &fcs_global_ahrs_state,
                    nav->reference_trajectory);

    /*
    Three options here:
    1. The current position is fairly near the expected position, so we're
       following the path OK.
    2. The current position is more than N metres from the expected position,
       so we need to add a new path to get back on track.
    3. There's no expected position, because the path hasn't been initialized.

    If option 1, we get the next point from the planner and update the last
    point in the horizon. This is the common case.

    If option 2, we effectively need to add a path between the current point
    and the first point in the reference trajectory. Once that's done we
    re-calculate the entire reference trajectory in one go.

    If option 3, we switch to the hold path around the current position.

    The offset between current and expected position is given by the first
    three elements of `state`, since all positions are given in NED relative
    to the first point in the reference trajectory.
    */
    if (vector3_norm_f(state) < FCS_CONTROL_POSITION_TOLERANCE) {
        /*
        Feedback with the latest state data. This solves the QP and generates
        the control values.
        */
        nmpc_set_wind_velocity(wind[0], wind[1], wind[2]);
        nmpc_feedback_step(state);

        /* Move on to the next reference point in the horizon */
        _shift_horizon(nav, wind);
    } else if (nav->reference_path_id[0] != FCS_CONTROL_INVALID_PATH_ID) {
        /*
        Construct a path sequence that gets the vehicle back to the next point
        in the reference trajectory. This is done in two parts: a resume path,
        which represents the un-flown portion of the current path (from the
        first point in the reference trajectory to the end of that path), and
        an interpolation path, which represents the path to get from the
        current vehicle state to the first point in the reference trajectory.

        If we're currently attempting to interpolate back to the resume path
        (from a previous recovery event), we can skip the resume path updates
        and just re-calculate the interpolation path.

        Otherwise, create a path in FCS_CONTROL_RESUME_PATH_ID with the
        current path type, navigating between FCS_CONTROL_RESUME_WAYPOINT_ID
        and the end_waypoint_id and next_path_id of the current path.
        */
        if (nav->reference_path_id[0] != FCS_CONTROL_INTERPOLATE_PATH_ID) {
            original_path_id = nav->reference_path_id[0];
            memcpy(&nav->paths[FCS_CONTROL_RESUME_PATH_ID],
                   &nav->paths[original_path_id], sizeof(struct fcs_path_t));

            /*
            The resume waypoint is set to the next point in the reference
            trajectory.
            */
            memcpy(
                &nav->waypoints[FCS_CONTROL_RESUME_WAYPOINT_ID],
                &nav->waypoints[
                    nav->paths[original_path_id].start_waypoint_id],
                sizeof(struct fcs_waypoint_t));

            /*
            Set the starting point of the resume path to the resume waypoint.
            */
            nav->paths[FCS_CONTROL_RESUME_PATH_ID].start_waypoint_id =
                FCS_CONTROL_RESUME_WAYPOINT_ID;
        }

        /*
        Then, we create a path in FCS_CONTROL_INTERPOLATE_PATH_ID with the
        current state as the starting point, and the end waypoint as
        FCS_CONTROL_RESUME_WAYPOINT_ID. The path type is
        FCS_PATH_DUBINS_CURVE, and the next_path_id is
        FCS_CONTROL_RESUME_PATH_ID.
        */
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].start_waypoint_id =
            FCS_CONTROL_INTERPOLATE_WAYPOINT_ID;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].end_waypoint_id =
            FCS_CONTROL_RESUME_WAYPOINT_ID;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].type =
            FCS_PATH_DUBINS_CURVE;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].flags = 0;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].next_path_id =
            FCS_CONTROL_RESUME_PATH_ID;

        /*
        Set the interpolate waypoint position and heading to the current AHRS
        state; the pitch and roll are cleared, because we don't want anything
        too violent, and the airspeed is pulled from the resume waypoint,
        since that will presumably be reasonable.
        */
        waypoint = &nav->waypoints[FCS_CONTROL_INTERPOLATE_WAYPOINT_ID];
        waypoint->lat = fcs_global_ahrs_state.lat;
        waypoint->lat = fcs_global_ahrs_state.lon;
        waypoint->alt = (float)fcs_global_ahrs_state.alt;
        waypoint->airspeed =
            nav->waypoints[FCS_CONTROL_RESUME_WAYPOINT_ID].airspeed;

        double qx, qy, qz, qw;
        qx = -fcs_global_ahrs_state.attitude[X];
        qy = -fcs_global_ahrs_state.attitude[Y];
        qz = -fcs_global_ahrs_state.attitude[Z];
        qw = fcs_global_ahrs_state.attitude[W];

        waypoint->yaw = (float)atan2(2.0f * (qx * qy + qw * qz),
                                     qw * qw - qz * qz - qy * qy + qx * qx);
        waypoint->pitch = 0.0;
        waypoint->roll = 0.0;

        /* Change the first reference path ID to the recovery path ID */
        //nav->reference_path_id[0] = FCS_CONTROL_INTERPOLATE_PATH_ID;

        /*
        We need to recalculate the horizon from scratch before the control
        input will mean anything.
        */
        _recalculate_horizon(nav, wind);

        /* Now that's done, grab the latest state data and run the feedback */
        _get_ahrs_state(state, wind, &fcs_global_ahrs_state,
                        nav->reference_trajectory);
        nmpc_set_wind_velocity(wind[0], wind[1], wind[2]);
        nmpc_feedback_step(state);
    } else {
        /*
        Path uninitialized; enter a holding pattern.

        Set up the initial waypoint (current position and yaw, standard
        airspeed, and arbitrary pitch/roll).
        */
        waypoint = &nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID];
        waypoint->lat = fcs_global_ahrs_state.lat;
        waypoint->lon = fcs_global_ahrs_state.lon;
        waypoint->alt = (float)fcs_global_ahrs_state.alt;
        waypoint->airspeed = FCS_CONTROL_DEFAULT_AIRSPEED;

        double qx, qy, qz, qw;
        qx = -fcs_global_ahrs_state.attitude[X];
        qy = -fcs_global_ahrs_state.attitude[Y];
        qz = -fcs_global_ahrs_state.attitude[Z];
        qw = fcs_global_ahrs_state.attitude[W];

        waypoint->yaw = (float)atan2(2.0f * (qx * qy + qw * qz),
                                     qw * qw - qz * qz - qy * qy + qx * qx);

        waypoint->pitch = 0.0;
        waypoint->roll = 0.0;

        /* Initialize the first path ID in the reference trajectory. */
        nav->reference_path_id[0] = FCS_CONTROL_HOLD_PATH_ID;

        /* Re-calculate the trajectory, then run the feedback step. */
        _recalculate_horizon(nav, wind);
        _get_ahrs_state(state, wind, &fcs_global_ahrs_state,
                        nav->reference_trajectory);
        nmpc_set_wind_velocity(wind[0], wind[1], wind[2]);
        nmpc_feedback_step(state);
    }

    /* Get the control values and update the global state. */
    result = nmpc_get_controls(controls);
    for (i = 0; i < NMPC_CONTROL_DIM; i++) {
        fcs_global_control_state.controls[i].setpoint = controls[i];
    }
}

float _interpolate_linear(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t) {
    float delta_n, delta_e, x, end_ned[3], distance, target_roll,
          target_airspeed, wind_n, wind_e, wind_dot;

    /*
    Linear interpolation of lat, lon, alt, airspeed and roll; pitch is set
    based on climb rate, and yaw is set based on heading between start
    and end points.

    The interpolation parameter used is based on the position of
    last_point on the line between start and end.
    */
    if (absval(start->lat - end->lat) > absval(start->lon - end->lon)) {
        /*
        Use the latitude difference to determine interpolation parameter
        */
        x = (float)((last_point->lat - start->lat) / (end->lat - start->lat));
    } else {
        /*
        Use the longitude difference to determine interpolation
        parameter
        */
        x = (float)((last_point->lon - start->lon) / (end->lon - start->lon));
    }

    if (isnan(x) || x > 1.0) {
        x = 1.0;
    } else if (x < 0.0) {
        x = 0.0;
    }

    /* Work out the airspeed -- make sure it's never less than the default */
    target_airspeed = start->airspeed + x * (end->airspeed - start->airspeed);
    if (target_airspeed < FCS_CONTROL_DEFAULT_AIRSPEED) {
        target_airspeed = FCS_CONTROL_DEFAULT_AIRSPEED;
    }

    /*
    Convert the end point to a NED offset from the last point. Move in the
    direction of the offset, based on the airspeed, heading and wind vector.

    If we move past the last point, clip to the last point and return the
    "unused" time.
    */
    _ned_from_point_diff(end_ned, last_point, end);

    distance = (float)sqrt(end_ned[0] * end_ned[0] + end_ned[1] * end_ned[1]);
    if (distance <= 1e-6) {
        /* Already at the end point */
        t = 0.0;
        delta_n = 0.0;
        delta_e = 0.0;
    } else {
        distance = 1.0f / distance;

        wind_dot = (end_ned[0] * distance * wind[0] +
                    end_ned[1] * distance * wind[1]);
        wind_n = end_ned[0] * distance * wind_dot;
        wind_e = end_ned[1] * distance * wind_dot;

        delta_n = (end_ned[0] * distance) * last_point->airspeed + wind_n;
        delta_e = (end_ned[1] * distance) * last_point->airspeed + wind_e;

        /*
        If we're past the last point, work out how much t we should use. Base
        the calculation on whichever delta is larger, to improve the accuracy
        of the result.
        */
        if (absval(end_ned[0]) > absval(end_ned[1]) &&
                absval(delta_n) * t > absval(end_ned[0])) {
            t = end_ned[0] / delta_n;
        } else if (absval(end_ned[0]) < absval(end_ned[1]) &&
                absval(delta_e) * t > absval(end_ned[1])) {
            t = end_ned[1] / delta_e;
        }
    }

    new_point->lat = last_point->lat + t * (1.0/WGS84_A) * delta_n;
    new_point->lon = last_point->lon +
                     t * (1.0/WGS84_A) * delta_e / cos(last_point->lat);

    /*
    Everything else is interpolated based on x, which is derived from the
    position. If the last point is at the midpoint between start and end, this
    x will be 0.5; this is not mathematically correct (e.g. since airspeed is
    interpolated, if there's an airspeed change over the path the roll rate
    will change).
    */
    new_point->alt = start->alt + x * (end->alt - start->alt);
    new_point->airspeed = target_airspeed;
    /* In theory we don't need to update this during the path, but whatever */
    new_point->yaw =
        mod_2pi_f((float)atan2((end->lon - start->lon) * cos(last_point->lat),
                               end->lat - start->lat));
    new_point->pitch = 0.0f;

    /* Interpolate roll in whichever direction is the shortest. */
    target_roll = end->roll - start->roll;
    if (target_roll > M_PI) {
        target_roll -= M_PI * 2.0f;
    } else if (target_roll < -M_PI) {
        target_roll += M_PI * 2.0f;
    }
    new_point->roll = start->roll + x * target_roll;
    if (new_point->roll > M_PI) {
        new_point->roll -= M_PI * 2.0f;
    } else if (new_point->roll < -M_PI) {
        new_point->roll += M_PI * 2.0f;
    }

    return t;
}

float _interpolate_figure_eight(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t) {
#pragma unused(end)
    /*
    Figure-eight path with the crossover point (middle of the 8) on the
    start waypoint. Altitude is set by the start waypoint; all other
    values are defined by the curve.

    We construct the figure-eight using two circles touching at the start/end
    point. The first circle involves a clockwise turn (banking to starboard),
    and the second a counter-clockwise turn (banking to port). The turn radius
    is FCS_CONTROL_TURN_RADIUS.

    The yaw value of the starting point determines the orientation of the
    pattern; the centres of the circles are FCS_CONTROL_TURN_RADIUS metres
    directly to the port and starboard of the start pose.

    Each circle takes
        2 * pi * FCS_CONTROL_TURN_RADIUS / airspeed
    seconds to complete, so the yaw rate is just
        FCS_CONTROL_TURN_RADIUS / airspeed.

    The sign of last point's roll value determines whether that value is added
    to or subtracted from last point's yaw value to get the new yaw value;
    every full circle, the roll value is negated.

    The new point's lat and lon are determined from the start lat/lon and the
    new yaw.
    */
    float yaw_rate, offset_n, offset_e, sd, cd, sy, cy, target_airspeed,
          target_yaw, target_roll, tangent_n,
          tangent_e, wind_dot, wind_yaw;

    /* Fly at the start speed. */
    target_airspeed = start->airspeed;

    /*
    Work out the wind correction for yaw rate -- project the wind vector onto
    the tangent of the path at the current point (which is the same direction
    as our velocity/heading) and then add that number of radians to the next
    yaw value.
    */
    tangent_n = (float)cos(last_point->yaw);
    tangent_e = (float)sin(last_point->yaw);
    wind_dot = tangent_n * wind[0] + tangent_e * wind[1];
    wind_yaw = t * wind_dot * (float)(1.0 / FCS_CONTROL_TURN_RADIUS);

    /*
    Roll angle is based on airspeed and turn radius (constant):
    roll_deg = 90 - atan(9.8 * r / v^2)
    */
    target_roll =
        (float)(M_PI * 0.5 - atan2(G_ACCEL * FCS_CONTROL_TURN_RADIUS,
                                   (target_airspeed * target_airspeed)));

    /* If delta yaw > start yaw - last yaw, it's time to change direction. */
    target_yaw = start->yaw - last_point->yaw;
    if (target_yaw > M_PI) {
        target_yaw -= M_PI * 2.0;
    } else if (target_yaw < -M_PI) {
        target_yaw += M_PI * 2.0;
    }

    /*
    Determine yaw rate based on airspeed; whether it's left or right depends
    on the current bank angle.
    */
    yaw_rate = target_airspeed * (float)(1.0 / FCS_CONTROL_TURN_RADIUS);

    if (last_point->roll > 0.0 && target_yaw > 0.0 &&
            target_yaw < yaw_rate * t + wind_yaw) {
        target_roll = -target_roll;
        yaw_rate = -target_airspeed * (float)(1.0 / FCS_CONTROL_TURN_RADIUS);
        wind_yaw = -wind_yaw;
    } else if (last_point->roll < 0.0 && target_yaw < 0.0 &&
               target_yaw > -yaw_rate * t - wind_yaw) {
        yaw_rate = target_airspeed * (float)(1.0 / FCS_CONTROL_TURN_RADIUS);
    } else if (last_point->roll < 0.0) {
        target_roll = -target_roll;
        yaw_rate = -yaw_rate;
        wind_yaw = -wind_yaw;
    }

    new_point->roll = target_roll;

    /* Scale roll angle to resolve discontinuity during direction change */
    if (absval(target_yaw) < 1.0) {
        new_point->roll *= max(0.01, absval(target_yaw));
    }

    /* Work out next yaw value, constraining to 0..2*pi. */
    new_point->yaw = mod_2pi_f(last_point->yaw + yaw_rate * t + wind_yaw);

    sy = (float)sin(start->yaw);
    cy = (float)cos(start->yaw);
    sd = (float)sin(new_point->yaw - start->yaw);
    cd = (float)cos(new_point->yaw - start->yaw);

    if (last_point->roll < 0.0) {
        /* Bank to the left, so the circle origin is to port. */
        offset_n = sd * -cy + (cd - 1.0f) * -sy;
        offset_e = sd * -sy - (cd - 1.0f) * -cy;
    } else {
        offset_n = sd * cy + (cd - 1.0f) * sy;
        offset_e = sd * sy - (cd - 1.0f) * cy;
    }

    offset_n *= FCS_CONTROL_TURN_RADIUS;
    offset_e *= FCS_CONTROL_TURN_RADIUS;

    /*
    Don't need to multiply offsets by t here, since it's done in the yaw
    calculation, from which these offsets are derived.
    */
    new_point->lat = start->lat + (1.0/WGS84_A) * offset_n;
    new_point->lon = start->lon + (1.0/WGS84_A) * offset_e / cos(start->lat);

    new_point->alt = start->alt;
    new_point->airspeed = target_airspeed;
    new_point->pitch = 0.0f;

    /* Always returning t means we never advance to the next path. */
    return t;
}

bool _plan_dubins_rsr(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b) {
    float tmp0, tmp1, tmp2;

    /* Compute RSR distance + initial turn required */
    tmp0 = d + sa - sb;
    tmp2 = 2.0f + d * d - 2.0f * ca_b + 2.0f * d * (sa - sb);

    if (tmp2 >= 0.0 && tmp0 >= 0.0) {
        tmp1 = (float)atan2(cb - ca, tmp0);
        *t = (float)sqrt(tmp2);
        *p = mod_2pi_f(-alpha + tmp1);
        *q = mod_2pi_f(beta - tmp1);

        return true;
    } else {
        return false;
    }
}

bool _plan_dubins_lsl(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b) {
    float tmp0, tmp1, tmp2;

    /* Compute LSL distance + initial turn required */
    tmp0 = d - sa + sb;
    tmp2 = 2.0f + d * d - 2.0f * ca_b + 2.0f * d * (sb - sa);

    if (tmp2 > 0.0 && tmp0 > 0.0) {
        tmp1 = (float)atan2(ca - cb,  tmp0);
        *t = (float)sqrt(tmp2);
        *p = mod_2pi_f(alpha - tmp1);
        *q = mod_2pi_f(-beta + tmp1);

        return true;
    } else {
        return false;
    }
}

bool _plan_dubins_rsl(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b) {
    float tmp1, tmp2;

    /* Compute RSL distance + initial turn required */
    tmp1 = -2.0f + d * d + 2.0f * ca_b + 2.0f * d * (sa + sb);
    if (tmp1 >= 0.0) {
        *t = (float)sqrt(tmp1);
        tmp2 = (float)(atan2(-ca - cb, d + sa + sb) - atan2(-2.0, *t));
        *p = mod_2pi_f(-alpha + tmp2);
        *q = mod_2pi_f(-mod_2pi_f(beta) + tmp2);

        return true;
    } else {
        return false;
    }
}

bool _plan_dubins_lsr(float *t, float *p, float *q, float alpha, float beta,
float d, float sa, float sb, float ca, float cb, float ca_b) {
    float tmp1, tmp2;

    /* Compute LSR distance + initial turn required */
    tmp1 = d * d - 2.0f + 2.0f * ca_b - 2.0f * d * (sa + sb);
    if (tmp1 >= 0.0) {
        *t = (float)sqrt(tmp1);
        tmp2 = (float)(atan2(ca + cb, d - sa - sb) - atan2(2.0, *t));
        *p = mod_2pi_f(alpha - tmp2);
        *q = mod_2pi_f(beta - tmp2);

        return true;
    } else {
        return false;
    }
}

void _calculate_dubins_r(float *out, const float *ref, float t) {
    out[0] = (float)(ref[0] + sin(ref[2] + t) - sin(ref[2]));
    out[1] = (float)(ref[1] - cos(ref[2] + t) + cos(ref[2]));
    out[2] = ref[2] + t;
}

void _calculate_dubins_l(float *out, const float *ref, float t) {
    out[0] = (float)(ref[0] - sin(ref[2] - t) + sin(ref[2]));
    out[1] = (float)(ref[1] + cos(ref[2] - t) - cos(ref[2]));
    out[2] = ref[2] - t;
}

void _calculate_dubins_s(float *out, const float *ref, float t) {
    out[0] = (float)(ref[0] + cos(ref[2]) * t);
    out[1] = (float)(ref[1] + sin(ref[2]) * t);
    out[2] = ref[2];
}

float _interpolate_dubins(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t) {
    /*
    Dubins path -- picks the shortest of LSL, LSR, RSL, RSR.

    In each of these curves, altitude is interpolated linearly; pitch is
    based on the climb rate, while airspeed, yaw and roll are set by the
    curve.

    A Dubins vehicle has one of three control states -- turning left at the
    maximum rate, turning right at the maximum rate, or travelling straight
    ahead. These states are identified by the combination of `path_type` and
    `segment`, which are stored in the generated point's flags.

    These calculations are all done with turn radius normalized to 1, and in
    a 2D coordinate system where yaw = 0 is along the +X-axis, and
    yaw = M_PI / 2 is along the +Y-axis. "Left" is defined as a turn in the
    direction of increasing yaw, i.e. counter-clockwise. This is the opposite
    sense to everything else, so to resolve the incosistency the path names
    and turn commands are inverted compared with other Dubins implementations.
    */
    float end_ned[3], target_airspeed, theta, a, b, sa, sb, ca, cb, ca_b, d,
          min_d, target_roll, tangent_n,
          tangent_e, wind_dot, straight_d, start_turn_d, end_turn_d, path_t,
          interpolation_rate, ref[3], out[3], p1[3], p2[3];
    uint8_t segment, last_segment; /* 0, 1 or 2 (curve, straight, curve) */
    int8_t first_action, last_action; /* -1 = left, 0 = straight, 1 = right */
    uint8_t path_type = 0xFFu; /* FCS_WAYPOINT_FLAG_DUBINS_LSL &c */

    _ned_from_point_diff(end_ned, start, end);

    theta = mod_2pi_f((float)atan2(end_ned[1], end_ned[0]));
    a = mod_2pi_f(start->yaw - theta);
    b = mod_2pi_f(end->yaw - theta);

    d = (float)sqrt(end_ned[0] * end_ned[0] + end_ned[1] * end_ned[1]) *
        (float)(1.0 / FCS_CONTROL_TURN_RADIUS);
    sa = (float)sin(a);
    sb = (float)sin(b);
    ca = (float)cos(a);
    cb = (float)cos(b);
    ca_b = (float)cos(a - b);

    first_action = last_action = 0;

    if (last_point == start) {
        /*
        We're evaluating the first point in the path; work out which path to
        take and then save the result in the point flags.
        */
        min_d = FLT_MAX;
        if (_plan_dubins_rsr(&straight_d, &start_turn_d, &end_turn_d, a, b, d,
                             sa, sb, ca, cb, ca_b)) {
            min_d = start_turn_d + straight_d + end_turn_d;
            path_type = FCS_WAYPOINT_FLAG_DUBINS_RSR;
            first_action = last_action = 1;
        }
        if (_plan_dubins_lsl(&straight_d, &start_turn_d, &end_turn_d, a, b, d,
                             sa, sb, ca, cb, ca_b)) {
            if (start_turn_d + straight_d + end_turn_d < min_d) {
                min_d = start_turn_d + straight_d + end_turn_d;
                path_type = FCS_WAYPOINT_FLAG_DUBINS_LSL;
                first_action = last_action = -1;
            }
        }
        if (_plan_dubins_rsl(&straight_d, &start_turn_d, &end_turn_d, a, b, d,
                             sa, sb, ca, cb, ca_b)) {
            if (start_turn_d + straight_d + end_turn_d < min_d) {
                min_d = start_turn_d + straight_d + end_turn_d;
                path_type = FCS_WAYPOINT_FLAG_DUBINS_RSL;
                first_action = 1;
                last_action = -1;
            }
        }
        if (_plan_dubins_lsr(&straight_d, &start_turn_d, &end_turn_d, a, b, d,
                             sa, sb, ca, cb, ca_b)) {
            if (start_turn_d + straight_d + end_turn_d < min_d) {
                min_d = start_turn_d + straight_d + end_turn_d;
                path_type = FCS_WAYPOINT_FLAG_DUBINS_LSR;
                first_action = -1;
                last_action = 1;
            }
        }

        last_segment = 0;

        assert(path_type != 0xFFu && "No valid Dubins path found.");
    } else {
        /* Already started the path, use the existing trajectory. */
        path_type =
            (last_point->flags & FCS_WAYPOINT_FLAG_PARAM_MASK) >>
            FCS_WAYPOINT_FLAG_PARAM_OFFSET;
        last_segment =
            (last_point->flags & FCS_WAYPOINT_FLAG_SEGMENT_MASK) >>
            FCS_WAYPOINT_FLAG_SEGMENT_OFFSET;

        /* Get the relevant path parameters. */
        switch (path_type) {
            case FCS_WAYPOINT_FLAG_DUBINS_RSR:
                _plan_dubins_rsr(&straight_d, &start_turn_d, &end_turn_d,
                                 a, b, d, sa, sb, ca, cb, ca_b);
                first_action = last_action = 1;
                break;
            case FCS_WAYPOINT_FLAG_DUBINS_LSL:
                _plan_dubins_lsl(&straight_d, &start_turn_d, &end_turn_d,
                                 a, b, d, sa, sb, ca, cb, ca_b);
                first_action = last_action = -1;
                break;
            case FCS_WAYPOINT_FLAG_DUBINS_RSL:
                _plan_dubins_rsl(&straight_d, &start_turn_d, &end_turn_d,
                                 a, b, d, sa, sb, ca, cb, ca_b);
                first_action = 1;
                last_action = -1;
                break;
            case FCS_WAYPOINT_FLAG_DUBINS_LSR:
                _plan_dubins_lsr(&straight_d, &start_turn_d, &end_turn_d,
                                 a, b, d, sa, sb, ca, cb, ca_b);
                first_action = -1;
                last_action = 1;
                break;
            default:
                assert(false && "Invalid Dubins path type");
                break;
        }

        /* Calculate total path length. */
        min_d = start_turn_d + straight_d + end_turn_d;
    }

    /* Both of these should have been set to one of the curve types. */
    assert(first_action && last_action);

    /* Get the position at the end of the first curve. */
    ref[0] = 0.0;
    ref[1] = 0.0;
    ref[2] = start->yaw;
    if (first_action == -1) {
        _calculate_dubins_l(p1, ref, start_turn_d);
    } else if (first_action == 1) {
        _calculate_dubins_r(p1, ref, start_turn_d);
    } else {
        assert(false && "Invalid Dubins first action");
    }

    /* Find the path interpolation parameter as at the last point. */
    if (last_segment == 0) {
        /*
        First curve -- move on to the next segment if the target angle is
        below the error threshold.
        */
        path_t = mod_2pi_f((last_point->yaw - start->yaw) * first_action);
    } else if (last_segment == 1u) {
        /*
        Straight section -- based on the first curve end point p1, work out
        how far along we are
        */
        _ned_from_point_diff(end_ned, start, last_point);

        end_ned[0] *= (float)(1.0 / FCS_CONTROL_TURN_RADIUS);
        end_ned[1] *= (float)(1.0 / FCS_CONTROL_TURN_RADIUS);

        end_ned[0] -= p1[0];
        end_ned[1] -= p1[1];

        path_t = start_turn_d +
            (float)sqrt(end_ned[0] * end_ned[0] + end_ned[1] * end_ned[1]);
    } else if (last_segment == 2u) {
        path_t = min_d -
                 mod_2pi_f((end->yaw - last_point->yaw) * last_action);
    } else {
        assert(false && "Invalid Dubins segment ID");
    }

    /*
    Work out the wind correction for path interpolation rate -- project the
    wind vector onto the tangent of the path at the current point (which is
    the same direction as our velocity/heading) and then add that distance to
    the parameter.
    */
    tangent_n = (float)cos(last_point->yaw);
    tangent_e = (float)sin(last_point->yaw);
    wind_dot = tangent_n * wind[0] + tangent_e * wind[1];

    target_airspeed = start->airspeed;
    target_roll =
        (float)(M_PI * 0.5 - atan2(G_ACCEL * FCS_CONTROL_TURN_RADIUS,
                                   (target_airspeed * target_airspeed)));

    interpolation_rate =
        (target_airspeed + wind_dot) * (float)(1.0 / FCS_CONTROL_TURN_RADIUS);

    path_t += t * interpolation_rate;
    if (path_t >= min_d) {
        /* Run out of path; work out how much unused */
        t -= (path_t - min_d) / interpolation_rate;
        /* FIXME? */
    }

    /* Got the current path parameter; work out the position of the point. */
    if (path_t <= start_turn_d) {
        if (first_action == -1) {
            _calculate_dubins_l(out, ref, path_t);
            target_roll = -target_roll;
        } else if (first_action == 1) {
            _calculate_dubins_r(out, ref, path_t);
        } else {
            assert(false && "Invalid Dubins first action");
        }

        segment = 0;
    } else if (path_t <= start_turn_d + straight_d) {
        _calculate_dubins_s(out, p1, path_t - start_turn_d);
        target_roll = 0.0;

        segment = 1u;
    } else {
        /* Get the position at the start of the last curve. */
        _calculate_dubins_s(p2, p1, straight_d);

        /* Process the last curve. */
        if (last_action == -1) {
            _calculate_dubins_l(
                out, p2, path_t - start_turn_d - straight_d);
            target_roll = -target_roll;
        } else if (last_action == 1) {
            _calculate_dubins_r(
                out, p2, path_t - start_turn_d - straight_d);
        } else {
            assert(false && "Invalid Dubins last action");
        }

        segment = 2u;
    }

    out[0] *= FCS_CONTROL_TURN_RADIUS;
    out[1] *= FCS_CONTROL_TURN_RADIUS;
    out[2] = mod_2pi_f(out[2]);

    /* Save the interpolation state in the next point's flags. */
    new_point->flags =
        ((uint32_t)path_type << FCS_WAYPOINT_FLAG_PARAM_OFFSET) |
        ((uint32_t)segment << FCS_WAYPOINT_FLAG_SEGMENT_OFFSET);

    /*
    Now that the required control state has been identified, update yaw and
    position based on that.
    */
    new_point->airspeed = target_airspeed;
    new_point->yaw = out[2];
    new_point->pitch = 0.0;
    new_point->roll = target_roll;
    new_point->lat = start->lat + (1.0/WGS84_A) * out[0];
    new_point->lon = start->lon +
                     (1.0/WGS84_A) * out[1] / cos(last_point->lat);
    new_point->alt = start->alt + (end->alt - start->alt) * (path_t / min_d);

    return t;
}

float _next_point_from_path(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
enum fcs_path_type_t type, float t) {
    if (type == FCS_PATH_LINE) {
        return _interpolate_linear(new_point, last_point, wind, start, end,
                                   t);
    } else if (type == FCS_PATH_DUBINS_CURVE) {
        return _interpolate_dubins(new_point, last_point, wind, start, end,
                                   t);
    } else if (type == FCS_PATH_FIGURE_EIGHT) {
        return _interpolate_figure_eight(new_point, last_point, wind, start,
                                         end, t);
    } else {
        assert(false && "Invalid path type.");
    }

    return t;
}

void _ned_from_point_diff(float *restrict ned,
const struct fcs_waypoint_t *restrict ref,
const struct fcs_waypoint_t *restrict point) {
    assert(ned && ref && point);
    _nassert((size_t)ned % 4u == 0);
    _nassert((size_t)ref % 8u == 0);
    _nassert((size_t)point % 8u == 0);

    /*
    Convert lat/lon to N, E by linearizing around current position -- this
    will break around the poles, and isn't accurate over medium distances
    (a few kilometres), but is adequate for reference trajectory calculation.
    */
    ned[0] = (float)((point->lat - ref->lat) * WGS84_A);
    ned[1] = (float)((point->lon - ref->lon) * WGS84_A * cos(ref->lat));

    /* D offset is just the difference in altitudes */
    ned[2] = ref->alt - point->alt;
}

void _get_ahrs_state(float *restrict state, float *restrict wind,
volatile struct fcs_ahrs_state_t *ahrs_state,
const struct fcs_waypoint_t *restrict reference) {
    /*
    Get the latest data from the AHRS. Since we don't lock anything here, it's
    possible for the UKF to start updating the data under us, but the
    worst-case scenario is that we get some data from 1ms later, which
    shouldn't be an issue.

    Since the NMPC code only looks at deltas between states, we can set the
    current position to the NED offset between the lat/lon/alt of the vehicle
    and the lat/lon/alt of the first point in the reference trajectory.
    */
    struct fcs_waypoint_t current_point;

    current_point.lat = ahrs_state->lat;
    current_point.lon = ahrs_state->lon;
    current_point.alt = (float)ahrs_state->alt;
    _ned_from_point_diff(state, reference, &current_point);

    /* state[2:0] has been set by the call above */
    state[3] = (float)ahrs_state->velocity[0];
    state[4] = (float)ahrs_state->velocity[1];
    state[5] = (float)ahrs_state->velocity[2];
    state[6] = (float)ahrs_state->attitude[0];
    state[7] = (float)ahrs_state->attitude[1];
    state[8] = (float)ahrs_state->attitude[2];
    state[9] = (float)ahrs_state->attitude[3];
    state[10] = (float)ahrs_state->angular_velocity[0];
    state[11] = (float)ahrs_state->angular_velocity[1];
    state[12] = (float)ahrs_state->angular_velocity[2];

    /* Set the wind based on the latest UKF estimate as well. */
    wind[0] = (float)ahrs_state->wind_velocity[0];
    wind[1] = (float)ahrs_state->wind_velocity[1];
    wind[2] = (float)ahrs_state->wind_velocity[2];
}

void _make_reference(float *restrict reference,
const struct fcs_waypoint_t *current_point,
const struct fcs_waypoint_t *last_point, const struct fcs_waypoint_t *start,
const float *restrict wind) {
    assert(reference);
    assert(current_point);
    assert(start);
    _nassert((size_t)reference % 4u == 0);
    _nassert((size_t)current_point % 8u == 0);

    /*
    last_point is only used for reference climb rate determination from
    altitude differences -- if it's not present, just use the current point so
    the climb rate will be 0.
    */
    if (!last_point) {
        last_point = current_point;
    }

    /*
    Determine reference velocity based on airspeed, yaw and current wind;
    determine reference attitude based on waypoint yaw, pitch and roll.
    */
    float next_reference_velocity[3], next_reference_attitude[4];

    next_reference_velocity[0] =
        current_point->airspeed * (float)cos(current_point->yaw) +
        wind[0];
    next_reference_velocity[1] =
        current_point->airspeed * (float)sin(current_point->yaw) +
        wind[1];
    next_reference_velocity[2] =
        OCP_STEP_LENGTH * (current_point->alt - last_point->alt);

    quaternion_f_from_yaw_pitch_roll(
        next_reference_attitude, current_point->yaw, current_point->pitch,
        current_point->roll);

    /*
    Update the horizon with the next reference trajectory step. The first
    NMPC_STATE_DIM values are the reference state (position, velocity,
    attitude, angular velocity), and the next NMPC_CONTROL_DIM values are the
    reference control values.
    */
    _ned_from_point_diff(reference, last_point, current_point);
    reference[3] = next_reference_velocity[0];
    reference[4] = next_reference_velocity[1];
    reference[5] = next_reference_velocity[2];
    reference[6] = next_reference_attitude[0];
    reference[7] = next_reference_attitude[1];
    reference[8] = next_reference_attitude[2];
    reference[9] = next_reference_attitude[3];
    reference[10] = OCP_STEP_LENGTH * (current_point->roll - last_point->roll);
    reference[11] = 0.0;
    reference[12] = 0.0;
    /* FIXME: reference points should be specified in the control config. */
    reference[NMPC_STATE_DIM + 0] = 0.5f;
    reference[NMPC_STATE_DIM + 1u] = 0.5f;
    reference[NMPC_STATE_DIM + 2u] = 0.5f;
}

/*
Determine the next point and path in the trajectory following `last_point`,
along the path given by `last_point_path_id`. This consumes up to 10 paths in
order to return a point OCP_STEP_LENGTH seconds ahead of `last_point`.
*/
void _next_point(struct fcs_waypoint_t *restrict new_point,
uint16_t *restrict new_point_path_id,
const struct fcs_waypoint_t *restrict last_point,
const uint16_t *restrict last_point_path_id, const float *restrict wind,
struct fcs_nav_state_t *nav) {
    float t;
    struct fcs_path_t *path;
    size_t i;

    assert(*last_point_path_id != FCS_CONTROL_INVALID_PATH_ID);

    /*
    Since we might have hit the end of a path, and it's theoretically possible
    to skip certain degenerate paths in a single timestep (waypoints closer
    than speed * OCP_STEP_LENGTH) we do this iteratively (but never more than
    an arbitrarily-chosen 10 iterations).

    Each time _next_point_from_path returns a value smaller than the
    timestep requested, we advance to the next path.
    */
    t = OCP_STEP_LENGTH;
    path = &nav->paths[*last_point_path_id];
    *new_point_path_id = *last_point_path_id;
    t -= _next_point_from_path(
        new_point, last_point, wind, &nav->waypoints[path->start_waypoint_id],
        &nav->waypoints[path->end_waypoint_id], path->type, t);

    i = 0;
    while (t > 0.0 && i < 10u) {
        if (path->next_path_id == FCS_CONTROL_INVALID_PATH_ID) {
            *new_point_path_id = FCS_CONTROL_HOLD_PATH_ID;
        } else {
            *new_point_path_id = path->next_path_id;
        }

        /*
        Sanity checks on consistency of path navigation -- the start waypoint
        of path N+1 must be the same as the end waypoint of path N, and the
        types can't both be FCS_PATH_LINE (since the headings would always
        mismatch).

        HOLD paths are exempt from that because they always result in a
        waypoint switch (to FCS_CONTROL_HOLD_WAYPOINT_ID).
        */
        assert(nav->paths[*new_point_path_id].start_waypoint_id ==
               path->end_waypoint_id ||
               *new_point_path_id == FCS_CONTROL_HOLD_PATH_ID);
        assert(nav->paths[*new_point_path_id].type != FCS_PATH_LINE ||
               path->type != FCS_PATH_LINE);

        path = &nav->paths[*new_point_path_id];
        t -= _next_point_from_path(
            new_point, &nav->waypoints[path->start_waypoint_id], wind,
            &nav->waypoints[path->start_waypoint_id],
            &nav->waypoints[path->end_waypoint_id], path->type, t);

        i++;
    }
}

/*
Shift the horizon by one timestep, effectively advancing the current position
along the reference trajectory. The last step in the reference trajectory is
calculated based on the current nav path state.

This is called every timestep provided the reference trajectory is being
followed correctly. If the vehicle state is "too far" from the reference
trajectory, such that a recovery path is needed, _recalculate_horizon
should be called instead.
*/
void _shift_horizon(struct fcs_nav_state_t *nav, const float *restrict wind) {
    float reference[NMPC_REFERENCE_DIM];
    struct fcs_waypoint_t *ref, *new_point, *last_point;
    uint16_t *ref_path_id, *new_point_path_id, *last_point_path_id;

    /*
    Shift the reference trajectory waypoints, as well as the corresponding
    path IDs.
    */
    ref = nav->reference_trajectory;
    new_point = &ref[OCP_HORIZON_LENGTH];
    last_point = &ref[OCP_HORIZON_LENGTH - 1u];

    ref_path_id = nav->reference_path_id;
    new_point_path_id = &ref_path_id[OCP_HORIZON_LENGTH];
    last_point_path_id = &ref_path_id[OCP_HORIZON_LENGTH - 1u];

    memmove(ref, &ref[1], sizeof(struct fcs_waypoint_t) * OCP_HORIZON_LENGTH);
    memmove(ref_path_id, &ref_path_id[1],
            sizeof(uint16_t) * OCP_HORIZON_LENGTH);

    _next_point(new_point, new_point_path_id, last_point, last_point_path_id,
                wind, nav);

    /*
    If we're not currently in a holding pattern, shift the hold waypoint to
    the end of the reference trajectory.
    */
    if (*last_point_path_id != FCS_CONTROL_HOLD_PATH_ID) {
        memcpy(&nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID],
           new_point, sizeof(struct fcs_waypoint_t));
    }

    _make_reference(reference, new_point, last_point,
                    nav->reference_trajectory, wind);
    nmpc_update_horizon(reference);
}

/*
Re-calculates the entire reference trajectory based on the current nav path
state. This is called when the nav path changes (e.g. loss of data link mode
entered), and also when the vehicle state diverges too far from the reference
trajectory.
*/
void _recalculate_horizon(struct fcs_nav_state_t *nav,
const float *restrict wind) {
    float reference[NMPC_REFERENCE_DIM];
    struct fcs_waypoint_t *ref, *new_point, *last_point;
    uint16_t *ref_path_id, *new_point_path_id, *last_point_path_id;
    struct fcs_path_t *path;
    size_t i;

    ref = nav->reference_trajectory;
    ref_path_id = nav->reference_path_id;

    /*
    First point; the reference path ID must already have been set, so
    generate a t=0 point for that path.
    */
    new_point = NULL;
    last_point = NULL;
    last_point_path_id = NULL;

    path = &nav->paths[ref_path_id[0]];
    _next_point_from_path(
        ref, &nav->waypoints[path->start_waypoint_id], wind,
        &nav->waypoints[path->start_waypoint_id],
        &nav->waypoints[path->end_waypoint_id], path->type, 0.0);
    _make_reference(reference, ref, NULL, ref, wind);
    nmpc_set_reference_point(reference, 0);

    for (i = 1u; i <= OCP_HORIZON_LENGTH; i++) {
        new_point = &ref[i];
        new_point_path_id = &ref_path_id[i];

        last_point = &ref[i - 1u];
        last_point_path_id = &ref_path_id[i - 1u];

        _next_point(new_point, new_point_path_id, last_point,
                    last_point_path_id, wind, nav);
        _make_reference(reference, new_point, last_point, ref, wind);

        assert(i < UINT32_MAX);
        nmpc_set_reference_point(reference, (uint32_t)i);
    }

    /*
    If we're not currently in a holding pattern, set the hold waypoint to
    the end of the reference trajectory.
    */
    if (*last_point_path_id != FCS_CONTROL_HOLD_PATH_ID) {
        assert(new_point);
        memcpy(&nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID],
           new_point, sizeof(struct fcs_waypoint_t));
    }
}
