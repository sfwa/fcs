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
#include <stdbool.h>
#include <math.h>
#include <float.h>

#include "../util/util.h"
#include "../util/3dmath.h"
#include "../nmpc/cnmpc.h"
#include "control.h"
#include "../exports/exports.h"
#include "trajectory.h"


static bool _plan_dubins_rsr(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b);

static bool _plan_dubins_lsl(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b);

static bool _plan_dubins_rsl(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b);

static bool _plan_dubins_lsr(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b);

static void _calculate_dubins_r(float *out, const float *reference, float t);

static void _calculate_dubins_l(float *out, const float *reference, float t);

static void _calculate_dubins_s(float *out, const float *reference, float t);


static bool _plan_dubins_rsr(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b) {
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

static bool _plan_dubins_lsl(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b) {
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

static bool _plan_dubins_rsl(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b) {
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

static bool _plan_dubins_lsr(float *t, float *p, float *q, float alpha,
float beta, float d, float sa, float sb, float ca, float cb, float ca_b) {
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

static void _calculate_dubins_r(float *out, const float *ref, float t) {
    out[0] = (float)(ref[0] + sin(ref[2] + t) - sin(ref[2]));
    out[1] = (float)(ref[1] - cos(ref[2] + t) + cos(ref[2]));
    out[2] = ref[2] + t;
}

static void _calculate_dubins_l(float *out, const float *ref, float t) {
    out[0] = (float)(ref[0] - sin(ref[2] - t) + sin(ref[2]));
    out[1] = (float)(ref[1] + cos(ref[2] - t) - cos(ref[2]));
    out[2] = ref[2] - t;
}

static void _calculate_dubins_s(float *out, const float *ref, float t) {
    out[0] = (float)(ref[0] + cos(ref[2]) * t);
    out[1] = (float)(ref[1] + sin(ref[2]) * t);
    out[2] = ref[2];
}

float fcs_trajectory_interpolate_dubins(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t) {
    /*
    Dubins path -- picks the shortest of LSL, LSR, RSL, RSR (LRL/RLR are not
    supported as they are not required for feasibility, only for optimality).

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
          min_d, target_roll, target_yaw, tangent_n, tangent_e, wind_dot,
          straight_d, start_turn_d, end_turn_d, path_t, tangent_ground_speed,
          interpolation_rate, ref[3], out[3], p1[3], p2[3];
    uint8_t segment, last_segment; /* 0, 1 or 2 (curve, straight, curve) */
    int8_t first_action, last_action; /* -1 = left, 0 = straight, 1 = right */
    uint8_t path_type = 0xFFu; /* FCS_WAYPOINT_FLAG_DUBINS_LSL &c */
    bool needs_roll_change;

    /* Handle zero-length paths */
    if (start == end) {
        return 0.0;
    }

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

        fcs_assert(path_type != 0xFFu && "No valid Dubins path found.");
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
                fcs_assert(false && "Invalid Dubins path type");
        }

        /* Calculate total path length. */
        min_d = start_turn_d + straight_d + end_turn_d;
    }

    /* Both of these should have been set to one of the curve types. */
    fcs_assert(first_action && last_action);

    /* Get the position at the end of the first curve. */
    ref[0] = 0.0;
    ref[1] = 0.0;
    ref[2] = start->yaw;
    if (first_action == -1) {
        _calculate_dubins_l(p1, ref, start_turn_d);
    } else if (first_action == 1) {
        _calculate_dubins_r(p1, ref, start_turn_d);
    } else {
        fcs_assert(false && "Invalid Dubins first action");
    }

    /*
    Work out whether the roll angle needs to change during the straight
    segment -- only if there's a turn direction change or the straight segment
    is long enough.
    */
    if (last_action != first_action || straight_d > 0.5) {
        needs_roll_change = true;
    } else {
        needs_roll_change = false;
    }

    /* Find the path interpolation parameter as at the last point. */
    if (last_segment == 0) {
        /*
        First curve -- move on to the next segment if the target angle is
        below the error threshold.
        */
        path_t = mod_2pi_f((last_point->yaw - start->yaw) * first_action);
        /*
        Start at 0 roll, reach a maximum half-way through the turn and end at
        0 again.
        */
        target_yaw = start_turn_d * 0.5f -
                     absval(start_turn_d * 0.5f - path_t);
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
        target_yaw = 0.0;
    } else if (last_segment == 2u) {
        path_t = mod_2pi_f((end->yaw - last_point->yaw) * last_action);
        target_yaw = end_turn_d * 0.5f -
                     absval(end_turn_d * 0.5f - path_t);
        path_t = min_d - path_t;
    } else {
        fcs_assert(false && "Invalid Dubins segment ID");
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

    tangent_ground_speed = target_airspeed ; //+ wind_dot * 0.33f;

    target_roll = (float)(M_PI * 0.5 - atan2(
        G_ACCEL * FCS_CONTROL_TURN_RADIUS,
        (tangent_ground_speed * tangent_ground_speed)));

    /*
    Smooth roll changes by interpolating from 0 to maximum over the course of
    1/2 of a radian in yaw.
    */
    if (absval(target_yaw) < 0.333333f &&
            (needs_roll_change ||
             (last_segment == 0 && path_t < start_turn_d * 0.5f) ||
             (last_segment == 2u && path_t < min_d - end_turn_d * 0.5f))) {
        target_roll *= max(0.01f, absval(target_yaw) * 3.0f);
    }

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
            fcs_assert(false && "Invalid Dubins first action");
        }

        segment = 0;
    } else if (path_t <= start_turn_d + straight_d) {
        _calculate_dubins_s(out, p1, path_t - start_turn_d);
        if (needs_roll_change) {
            target_roll = 0.0;
        }

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
            fcs_assert(false && "Invalid Dubins last action");
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
    new_point->pitch = 3.0f * ((float)M_PI / 180.0f);
    new_point->roll = target_roll;
    new_point->lat = start->lat + (1.0/WGS84_A) * out[0];
    new_point->lon = start->lon +
                     (1.0/WGS84_A) * out[1] / cos(last_point->lat);
    new_point->alt = start->alt + (end->alt - start->alt) * (path_t / min_d);

    return t;
}
