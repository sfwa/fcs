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
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#include "../util/util.h"
#include "../util/3dmath.h"
#include "../nmpc/cnmpc.h"
#include "control.h"
#include "../exports/exports.h"
#include "trajectory.h"


static void _shift_horizon(struct fcs_nav_state_t *nav,
const float *restrict wind);

static void _make_reference(float *restrict reference,
const struct fcs_waypoint_t *current_point,
const struct fcs_waypoint_t *last_point, const struct fcs_waypoint_t *start,
const float *restrict wind);

static void _next_point(struct fcs_waypoint_t *restrict new_point,
uint16_t *restrict new_point_path_id,
const struct fcs_waypoint_t *restrict last_point,
const uint16_t *restrict last_point_path_id, const float *restrict wind,
struct fcs_nav_state_t *nav);

static float _next_point_from_path(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
enum fcs_path_type_t type, float t);

static void _stabilise_path_to_waypoint(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate,
uint16_t out_waypoint_id, uint16_t out_path_id);

static void _get_ahrs_state(float *restrict state,
const struct fcs_state_estimate_t *restrict state_estimate,
const struct fcs_waypoint_t *restrict reference);

void _get_next_reference_point(float *restrict state);


static float stabilise_state_weights[NMPC_DELTA_DIM] = {
    1e-4f, 1e-4f, 1e-2f, /* position */
    1e0f, 1e0f, 1e0f, /* velocity */
    1e0f, 1e0f, 2e-1f, /* attitude */
    1e0f, 1e0f, 1e0f /* angular velocity */
};
static float normal_state_weights[NMPC_DELTA_DIM] = {
    2e-2f, 2e-2f, 2e0f,  /* position */
    1e0f, 1e0f, 1e0f,  /* velocity */
    4e0f, 1e0f, 2e-1f,  /* attitude */
    2e0f, 1e0f, 1e0f /* angular velocity */
};


/*
Re-calculates the entire reference trajectory based on the current nav path
state. This is called when the nav path changes (e.g. loss of data link mode
entered), and also when the vehicle state diverges too far from the reference
trajectory.
*/
void fcs_trajectory_recalculate(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate) {
    fcs_assert(nav);
    fcs_assert(state_estimate);

    float reference[NMPC_REFERENCE_DIM], weights[NMPC_DELTA_DIM];
    struct fcs_waypoint_t *ref, *new_point, *last_point;
    uint16_t *ref_path_id, *new_point_path_id, *last_point_path_id;
    struct fcs_path_t *path;
    size_t i, j;

    ref = nav->reference_trajectory;
    ref_path_id = nav->reference_path_id;

    /*
    First point; the reference path ID must already have been set, so
    generate a t=0 point for that path.
    */
    new_point = NULL;
    last_point = NULL;
    last_point_path_id = NULL;

    nmpc_init(true);

    /* Calculate the first point and save it to the reference trajectory */
    path = &nav->paths[ref_path_id[0]];
    _next_point_from_path(
        ref, &nav->waypoints[path->start_waypoint_id],
        state_estimate->wind_velocity,
        &nav->waypoints[path->start_waypoint_id],
        &nav->waypoints[path->end_waypoint_id], path->type, 0.0);
    _make_reference(reference, ref, NULL, ref, state_estimate->wind_velocity);

    if (ref_path_id[0] == FCS_CONTROL_STABILISE_PATH_ID) {
        nmpc_set_state_weights(stabilise_state_weights);
    } else {
        nmpc_set_state_weights(normal_state_weights);
    }
    nmpc_set_reference_point(reference, 0);

    /* Do the same for each subsequent point */
    for (i = 1u; i <= OCP_HORIZON_LENGTH; i++) {
        new_point = &ref[i];
        new_point_path_id = &ref_path_id[i];

        last_point = &ref[i - 1u];
        last_point_path_id = &ref_path_id[i - 1u];

        _next_point(new_point, new_point_path_id, last_point,
                    last_point_path_id, state_estimate->wind_velocity, nav);
        _make_reference(reference, new_point, last_point, ref,
                        state_estimate->wind_velocity);

        fcs_assert(i < UINT32_MAX);
        if (ref_path_id[i] == FCS_CONTROL_STABILISE_PATH_ID) {
            for (j = 0; j < NMPC_DELTA_DIM; j++) {
                weights[j] = stabilise_state_weights[j] +
                    (normal_state_weights[j] - stabilise_state_weights[j]) *
                    ((float)i / (float)OCP_HORIZON_LENGTH) *
                    ((float)i / (float)OCP_HORIZON_LENGTH);
            }
            nmpc_set_state_weights(weights);
        } else {
            nmpc_set_state_weights(normal_state_weights);
        }
        nmpc_set_reference_point(reference, (uint32_t)i);
    }

    /*
    If we're not currently in a holding pattern, set the hold waypoint to
    the end of the reference trajectory.
    */
    if (*last_point_path_id != FCS_CONTROL_HOLD_PATH_ID &&
            *last_point_path_id != FCS_CONTROL_STABILISE_PATH_ID &&
            *last_point_path_id != FCS_CONTROL_INTERPOLATE_PATH_ID) {
        fcs_assert(new_point);
        memcpy(&nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID],
           new_point, sizeof(struct fcs_waypoint_t));
        nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID].flags =
            FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_ENTRY;
    }
}

void fcs_trajectory_timestep(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate) {
    fcs_assert(nav);
    fcs_assert(state_estimate);

    float state_vec[NMPC_STATE_DIM];

    _get_ahrs_state(state_vec, state_estimate, nav->reference_trajectory);

    nmpc_set_wind_velocity(
        state_estimate->wind_velocity[0], state_estimate->wind_velocity[1],
        state_estimate->wind_velocity[2]);
    nmpc_feedback_step(state_vec);
    _shift_horizon(nav, state_estimate->wind_velocity);
}

void fcs_trajectory_start_recover(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate, bool from_start) {
    fcs_assert(nav);
    fcs_assert(state_estimate);

    uint16_t original_path_id;

    /*
    Construct a path sequence that gets the vehicle back to the next point in
    the reference trajectory. This is done in three parts:
    1. a stabilise path, which is a 5-second straight path starting at the
       current reference position and moving in the reference heading;
    2. an interpolation path, which is a curve from the endpoint of the
       stabilisation path to the start of the un-flown portion of the current
       path (which is the first point in the reference trajectory at the start
       of this function);
    3. a resume path, which is the remaining portion of the original path
       from the first unflown point to the end.

    If we're currently attempting to stabilise or interpolate (from a previous
    recovery event), we can skip the resume path updates and just re-calculate
    the stabilise and interpolation paths.

    Otherwise, create a path in FCS_CONTROL_RESUME_PATH_ID with the
    current path type, navigating between FCS_CONTROL_RESUME_WAYPOINT_ID
    and the end_waypoint_id and next_path_id of the current path.
    */
    if (nav->reference_path_id[0] == FCS_CONTROL_HOLD_PATH_ID) {
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].next_path_id =
            FCS_CONTROL_HOLD_PATH_ID;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].end_waypoint_id =
            FCS_CONTROL_HOLD_WAYPOINT_ID;
        nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID].flags =
            FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_ENTRY;
    } else if (nav->reference_path_id[0] != FCS_CONTROL_INTERPOLATE_PATH_ID &&
               nav->reference_path_id[0] != FCS_CONTROL_STABILISE_PATH_ID) {
        original_path_id = nav->reference_path_id[0];
        memcpy(&nav->paths[FCS_CONTROL_RESUME_PATH_ID],
               &nav->paths[original_path_id], sizeof(struct fcs_path_t));

        if (from_start) {
            memcpy(&nav->waypoints[FCS_CONTROL_RESUME_WAYPOINT_ID],
                   &nav->waypoints[nav->paths[original_path_id].start_waypoint_id],
                   sizeof(struct fcs_waypoint_t));
        } else {
            /*
            The resume waypoint is set to the next point in the reference
            trajectory.
            */
            memcpy(&nav->waypoints[FCS_CONTROL_RESUME_WAYPOINT_ID],
                   &nav->reference_trajectory[0], sizeof(struct fcs_waypoint_t));
        }


        /*
        Set the starting point of the resume path to the resume waypoint.
        */
        nav->paths[FCS_CONTROL_RESUME_PATH_ID].start_waypoint_id =
            FCS_CONTROL_RESUME_WAYPOINT_ID;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].next_path_id =
            FCS_CONTROL_RESUME_PATH_ID;
        nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].end_waypoint_id =
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
    nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].type =
        FCS_PATH_DUBINS_CURVE;
    nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].flags = 0;

    /*
    Start a 5-second stabilisation path before following the interpolation
    path back to the original departure point. This sets the staring point of
    the interpolation path to the end of the stabilisation path.
    */
    _stabilise_path_to_waypoint(nav, state_estimate,
                                FCS_CONTROL_INTERPOLATE_WAYPOINT_ID,
                                FCS_CONTROL_INTERPOLATE_PATH_ID);
}

void fcs_trajectory_start_hold(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate) {
    fcs_assert(nav);
    fcs_assert(state_estimate);

    /*
    Start a 5-second stabilisation path and enter a holding pattern at the
    end of it.
    */
    _stabilise_path_to_waypoint(nav, state_estimate,
                                FCS_CONTROL_HOLD_WAYPOINT_ID,
                                FCS_CONTROL_HOLD_PATH_ID);

    nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].next_path_id =
        FCS_CONTROL_HOLD_PATH_ID;
    nav->paths[FCS_CONTROL_INTERPOLATE_PATH_ID].end_waypoint_id =
        FCS_CONTROL_HOLD_WAYPOINT_ID;

    nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID].flags =
        FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_ENTRY;
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
static void _shift_horizon(struct fcs_nav_state_t *nav,
const float *restrict wind) {
    fcs_assert(nav);
    fcs_assert(wind);

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
    if (*last_point_path_id != FCS_CONTROL_HOLD_PATH_ID &&
            *last_point_path_id != FCS_CONTROL_STABILISE_PATH_ID &&
            *last_point_path_id != FCS_CONTROL_INTERPOLATE_PATH_ID) {
        memcpy(&nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID],
           last_point, sizeof(struct fcs_waypoint_t));
        nav->waypoints[FCS_CONTROL_HOLD_WAYPOINT_ID].flags =
            FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_ENTRY;
    }

    _make_reference(reference, new_point, last_point,
                    nav->reference_trajectory, wind);

    nmpc_set_state_weights(normal_state_weights);
    nmpc_update_horizon(reference);
}


static float _next_point_from_path(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
enum fcs_path_type_t type, float t) {
    if (type == FCS_PATH_LINE) {
        return fcs_trajectory_interpolate_linear(new_point, last_point, wind,
                                                 start, end, t);
    } else if (type == FCS_PATH_DUBINS_CURVE) {
        return fcs_trajectory_interpolate_dubins(new_point, last_point, wind,
                                                 start, end, t);
    } else if (type == FCS_PATH_FIGURE_EIGHT) {
        return fcs_trajectory_interpolate_figure_eight(new_point, last_point,
                                                       wind, start, end, t);
    } else if (type == FCS_PATH_RELEASE) {
        return fcs_trajectory_interpolate_release(new_point, last_point, wind,
                                                  start, end, t);
    } else {
        fcs_assert(false && "Invalid path type.");
    }
}

static void _make_reference(float *restrict reference,
const struct fcs_waypoint_t *current_point,
const struct fcs_waypoint_t *last_point, const struct fcs_waypoint_t *start,
const float *restrict wind) {
    fcs_assert(reference);
    fcs_assert(current_point);
    fcs_assert(start);
    fcs_assert(wind);
    _nassert((size_t)reference % 4u == 0);
    _nassert((size_t)current_point % 8u == 0);
    _nassert((size_t)wind % 4u == 0);

    if (!last_point) {
        last_point = current_point;
    }

    /*
    Determine reference velocity based on airspeed, yaw and current wind;
    determine reference attitude based on waypoint yaw, pitch and roll.
    */
    float next_reference_velocity[3], next_reference_attitude[4],
          last_reference_attitude[4], tmp[4], tmp2[4], actual_yaw,
          tangent_n, tangent_e, wind_dot;

    tangent_n = (float)cos(current_point->yaw);
    tangent_e = (float)sin(current_point->yaw);
    wind_dot = tangent_n * wind[0] + tangent_e * wind[1];

    next_reference_velocity[0] =
        (wind_dot + current_point->airspeed) * tangent_n;
    next_reference_velocity[1] =
        (wind_dot + current_point->airspeed) * tangent_e;
    next_reference_velocity[2] = (1.0f / OCP_STEP_LENGTH) *
                                 (last_point->alt - current_point->alt);

    actual_yaw = (float)atan2(current_point->airspeed * tangent_e - wind[1],
                              current_point->airspeed * tangent_n - wind[0]);

    /* Calculate angular velocity based on the reference attitudes */
    quaternion_f_from_yaw_pitch_roll(next_reference_attitude,
                                     current_point->yaw, current_point->pitch,
                                     current_point->roll);

    quaternion_f_from_yaw_pitch_roll(last_reference_attitude, last_point->yaw,
                                     last_point->pitch, last_point->roll);

    tmp[0] = (next_reference_attitude[0] - last_reference_attitude[0]) *
             2.0f * (1.0f / OCP_STEP_LENGTH);
    tmp[1] = (next_reference_attitude[1] - last_reference_attitude[1]) *
             2.0f * (1.0f / OCP_STEP_LENGTH);
    tmp[2] = (next_reference_attitude[2] - last_reference_attitude[2]) *
             2.0f * (1.0f / OCP_STEP_LENGTH);
    tmp[3] = (next_reference_attitude[3] - last_reference_attitude[3]) *
             2.0f * (1.0f / OCP_STEP_LENGTH);

    /* Take the conjugate of last_reference_attitude */
    last_reference_attitude[3] *= -1.0;

    quaternion_multiply_f(tmp2, tmp, last_reference_attitude);

    /* Work out the attitude again with wind-compensated yaw */
    quaternion_f_from_yaw_pitch_roll(next_reference_attitude,
                                     actual_yaw, current_point->pitch,
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
    reference[10] = tmp2[0];
    reference[11] = tmp2[1];
    reference[12] = tmp2[2];
    /* FIXME: reference points should be specified in the control config. */
    reference[NMPC_STATE_DIM + 0] = 0.47f;
    reference[NMPC_STATE_DIM + 1u] = 0.46f;
    reference[NMPC_STATE_DIM + 2u] = 0.46f;
}

/*
Determine the next point and path in the trajectory following `last_point`,
along the path given by `last_point_path_id`. This consumes up to 10 paths in
order to return a point OCP_STEP_LENGTH seconds ahead of `last_point`.
*/
static void _next_point(struct fcs_waypoint_t *restrict new_point,
uint16_t *restrict new_point_path_id,
const struct fcs_waypoint_t *restrict last_point,
const uint16_t *restrict last_point_path_id, const float *restrict wind,
struct fcs_nav_state_t *nav) {
    fcs_assert(new_point);
    fcs_assert(new_point_path_id);
    fcs_assert(last_point);
    fcs_assert(last_point_path_id);
    fcs_assert(wind);
    fcs_assert(nav);
    fcs_assert(*last_point_path_id != FCS_CONTROL_INVALID_PATH_ID);

    float t;
    struct fcs_path_t *path;
    size_t i;

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
        of path N+1 must be the same as the end waypoint of path N.

        HOLD paths are exempt from that because they always result in a
        waypoint switch (to FCS_CONTROL_HOLD_WAYPOINT_ID).
        */
        fcs_assert(nav->paths[*new_point_path_id].start_waypoint_id ==
                   path->end_waypoint_id ||
                   *new_point_path_id == FCS_CONTROL_HOLD_PATH_ID);

        path = &nav->paths[*new_point_path_id];
        t -= _next_point_from_path(
            new_point, &nav->waypoints[path->start_waypoint_id], wind,
            &nav->waypoints[path->start_waypoint_id],
            &nav->waypoints[path->end_waypoint_id], path->type, t);

        i++;
    }
}

static void _stabilise_path_to_waypoint(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate,
uint16_t out_waypoint_id, uint16_t out_path_id) {
    fcs_assert(nav);
    fcs_assert(state_estimate);
    fcs_assert(out_path_id != FCS_CONTROL_INVALID_PATH_ID);
    fcs_assert(out_waypoint_id != FCS_CONTROL_INVALID_WAYPOINT_ID);

    struct fcs_waypoint_t *waypoint, *out_waypoint;
    float stabilise_delta[2], alt;

    out_waypoint = &nav->waypoints[out_waypoint_id];

    /* Set up the stabilisation path waypoint */
    waypoint = &nav->waypoints[FCS_CONTROL_STABILISE_WAYPOINT_ID];
    waypoint->lat = state_estimate->lat;
    waypoint->lon = state_estimate->lon;
    waypoint->alt = state_estimate->alt;
    waypoint->airspeed = nav->reference_trajectory[0].airspeed >
                            FCS_CONTROL_DEFAULT_AIRSPEED ?
                         nav->reference_trajectory[0].airspeed :
                         FCS_CONTROL_DEFAULT_AIRSPEED;
    waypoint->yaw = nav->reference_trajectory[0].yaw;
    waypoint->pitch = 0.0f;
    waypoint->roll = nav->reference_trajectory[0].roll;

    /*
    Set up the holding pattern waypoint (current position and yaw,
    standard airspeed, and arbitrary pitch/roll).
    */
    stabilise_delta[0] = (float)(cos(waypoint->yaw) * waypoint->airspeed);
    stabilise_delta[1] = (float)(sin(waypoint->yaw) * waypoint->airspeed);
    alt = waypoint->alt < state_estimate->alt ?
          waypoint->alt : state_estimate->alt;

    out_waypoint->lat = waypoint->lat +
                        (1.0/WGS84_A) * stabilise_delta[0] * 3.0f;
    out_waypoint->lon = waypoint->lon +
                        (1.0/WGS84_A) * stabilise_delta[1] * 3.0f /
                        cos(waypoint->lat);
    out_waypoint->alt = alt;
    out_waypoint->airspeed = FCS_CONTROL_DEFAULT_AIRSPEED;
    out_waypoint->yaw = waypoint->yaw;
    out_waypoint->pitch = 0.0f;
    out_waypoint->roll = 0.0f;

    nav->paths[FCS_CONTROL_STABILISE_PATH_ID].start_waypoint_id =
        FCS_CONTROL_STABILISE_WAYPOINT_ID;
    nav->paths[FCS_CONTROL_STABILISE_PATH_ID].end_waypoint_id =
        out_waypoint_id;
    nav->paths[FCS_CONTROL_STABILISE_PATH_ID].next_path_id =
        out_path_id;
    nav->paths[FCS_CONTROL_STABILISE_PATH_ID].type = FCS_PATH_LINE;

    /* Initialize the first path ID in the reference trajectory. */
    nav->reference_path_id[0] = FCS_CONTROL_STABILISE_PATH_ID;
}

void _ned_from_point_diff(float *restrict ned,
const struct fcs_waypoint_t *restrict ref,
const struct fcs_waypoint_t *restrict point) {
    fcs_assert(ned && ref && point);
    _nassert((size_t)ned % 4u == 0);
    _nassert((size_t)ref % 8u == 0);
    _nassert((size_t)point % 8u == 0);

    /*
    Convert lat/lon to N, E by linearizing around current position -- this
    will break around the poles, and isn't accurate over medium distances
    (several kilometres), but is adequate for reference trajectory
    calculation.
    */
    ned[0] = (float)((point->lat - ref->lat) * WGS84_A);
    ned[1] = (float)((point->lon - ref->lon) * WGS84_A * cos(ref->lat));

    /* D offset is just the difference in altitudes */
    ned[2] = ref->alt - point->alt;
}

static void _get_ahrs_state(float *restrict state,
const struct fcs_state_estimate_t *restrict state_estimate,
const struct fcs_waypoint_t *restrict reference) {
    fcs_assert(state);
    fcs_assert(state_estimate);
    fcs_assert(reference);
    _nassert((size_t)state % 4u == 0);

    /*
    Get the latest data from the AHRS. Since the NMPC code only looks at
    deltas between states, we can set the current position to the NED offset
    between the lat/lon/alt of the vehicle and the lat/lon/alt of the first
    point in the reference trajectory.
    */
    struct fcs_waypoint_t current_point;

    current_point.lat = state_estimate->lat;
    current_point.lon = state_estimate->lon;
    current_point.alt = state_estimate->alt;
    _ned_from_point_diff(state, reference, &current_point);

    /* state[2:0] has been set by the call above */
    state[3] = state_estimate->velocity[0];
    state[4] = state_estimate->velocity[1];
    state[5] = state_estimate->velocity[2];
    state[6] = state_estimate->attitude[0];
    state[7] = state_estimate->attitude[1];
    state[8] = state_estimate->attitude[2];
    state[9] = state_estimate->attitude[3];
    state[10] = state_estimate->angular_velocity[0];
    state[11] = state_estimate->angular_velocity[1];
    state[12] = state_estimate->angular_velocity[2];
}

void _get_next_reference_point(float *restrict state) {
    fcs_assert(state);

    nmpc_get_reference_point(state, 0);
}
