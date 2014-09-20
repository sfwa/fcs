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


static inline float _angle_diff(float from, float to) {
    float m, n, o;

    to = mod_2pi_f(to);
    from = mod_2pi_f(from);

    m = to - from;
    n = to - from + 2.0f * (float)M_PI;
    o = to - from - 2.0f * (float)M_PI;

    if (absval(m) < absval(n) && absval(m) < absval(o)) {
        return m;
    } else if (absval(n) < absval(o)) {
        return n;
    } else {
        return o;
    }
}


float fcs_trajectory_interpolate_figure_eight(
struct fcs_waypoint_t *new_point, const struct fcs_waypoint_t *last_point,
const float *restrict wind, const struct fcs_waypoint_t *start,
const struct fcs_waypoint_t *end, float t) {
#ifndef __TI_COMPILER_VERSION__
#pragma unused(end)
#endif
    /*
    Figure-eight path with the crossover point (middle of the 8) on the
    start waypoint. Altitude is set by the start waypoint; all other
    values are defined by the curve.

    The figure-eight is made from a right entry segment (a straight line of
    length FCS_CONTROL_TURN_RADIUS), a right curve (270 degrees), a right
    exit (another straight line of length FCS_CONTROL_TURN_RADIUS,
    perpendicular to the first), a left entry segment (same direction, same
    length), a left curve (another 270 degrees) and a left exit segment (same
    direction as the right entry segment, leading back to the origin).

    This is arranged as follows:

                                            111111111111
                                        1111            1111
                                      11                    11
                                    11                        11
                                    11                        11
                                  11                            11
                                  11                            11
                                  11                            11
                                  00                            11
                                  00                            11
                                  00                            11
                                  00                          11
                                  00                          11
                                  00                        11
                                  00                    1111
              44444433333333333333**22222222222222111111
          4444                    55
        44                        55
      44                          55
      44                          55
    44                            55
    44                            55
    44                            55
    44                            44
    44                            44
    44                            44
      44                        44
      44                        44
        44                    44
          4444            4444
              444444444444

    The yaw value of the starting point determines the orientation of the
    pattern; all calculations are done in UV space which is scaled and rotated
    such that the initial (right entry) segment is (0, 0) to (1, 0) and the
    arc radius is 1.

    Each circle takes
        2 * pi * FCS_CONTROL_TURN_RADIUS / airspeed
    seconds to complete, so the yaw rate is just
        +/- FCS_CONTROL_TURN_RADIUS / airspeed.
    */
    float offset_n, offset_e, offset_u, offset_v, sd, cd, sy, cy,
          target_airspeed, target_yaw, target_roll, tangent_n, tangent_e,
          wind_dot, tangent_ground_speed, last_ned[3], delta_pos, delta_yaw,
          wind_perp2, wind_comp;
    uint8_t last_segment, new_segment;

    sy = (float)sin(start->yaw);
    cy = (float)cos(start->yaw);

    /* Fly at the start speed. */
    target_airspeed = start->airspeed;
    target_yaw = mod_2pi_f(_angle_diff(start->yaw, last_point->yaw));

    /*
    Work out the wind correction for position -- project the wind vector onto
    our current heading.
    */
    tangent_n = (float)cos(last_point->yaw);
    tangent_e = (float)sin(last_point->yaw);
    wind_dot = tangent_n * wind[0] + tangent_e * wind[1];
    wind_perp2 = (wind[0] * wind[0] + wind[1] * wind[1] -
                  wind_dot * wind_dot);
    wind_comp = (float)sqrt(max(0.0,
                                start->airspeed * start->airspeed -
                                wind_perp2));
    tangent_ground_speed = wind_dot + wind_comp;

    delta_pos = tangent_ground_speed * t *
                (float)(1.0 / FCS_CONTROL_TURN_RADIUS);

    last_segment = new_segment =
        (uint8_t)(last_point->flags & FCS_WAYPOINT_FLAG_PARAM_MASK);

    if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_CURVE) {
        target_yaw += delta_pos;

        if (target_yaw >= (float)M_PI * 1.5f) {
            target_yaw = (float)M_PI * 1.5f;
            new_segment = FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_EXIT;
        }

        sd = (float)sin(target_yaw);
        cd = (float)cos(target_yaw);

        offset_u = sd + 1.0f;
        offset_v = -cd + 1.0f;
    } else if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_LEFT_CURVE) {
        target_yaw -= delta_pos;

        if (target_yaw <= 0.0f) {
            target_yaw = 0.0f;
            new_segment = FCS_WAYPOINT_FLAG_FIGURE8_LEFT_EXIT;
        }

        sd = (float)sin(target_yaw);
        cd = (float)cos(target_yaw);

        /* Bank to the left, so the circle origin is to port. */
        offset_u = -sd - 1.0f;
        offset_v = cd - 1.0f;
    } else {
        /*
        Straight segment -- move from the most recent point in the appropriate
        direction
        */
        _ned_from_point_diff(last_ned, start, last_point);

        last_ned[0] *= (float)(1.0 / FCS_CONTROL_TURN_RADIUS);
        last_ned[1] *= (float)(1.0 / FCS_CONTROL_TURN_RADIUS);

        /* Transform NE back into UV through rotation by -start_yaw */
        offset_u = last_ned[0] * cy - last_ned[1] * -sy;
        offset_v = last_ned[0] * -sy + last_ned[1] * cy;

        if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_ENTRY) {
            offset_u += delta_pos;
            offset_v = 0.0f;

            if (offset_u >= 1.0f) {
                new_segment = FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_CURVE;
            }
        } else if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_EXIT) {
            offset_u = 0.0f;
            offset_v -= delta_pos;

            if (offset_v <= 0.0f) {
                new_segment = FCS_WAYPOINT_FLAG_FIGURE8_LEFT_ENTRY;
            }
        } else if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_LEFT_ENTRY) {
            offset_u = 0.0f;
            offset_v -= delta_pos;

            if (offset_v <= -1.0f) {
                new_segment = FCS_WAYPOINT_FLAG_FIGURE8_LEFT_CURVE;
            }
        } else if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_LEFT_EXIT) {
            offset_u += delta_pos;
            offset_v = 0.0f;

            if (offset_u >= 0.0f) {
                new_segment = FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_ENTRY;
            }
        } else {
            fcs_assert(false && "Invalid figure-eight segment ID");
        }
    }

    if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_LEFT_CURVE ||
            last_segment == FCS_WAYPOINT_FLAG_FIGURE8_RIGHT_CURVE) {
        /*
        Roll angle is based on airspeed and turn radius (constant):
        roll_deg = 90 - atan(9.8 * r / v^2)
        */
        tangent_ground_speed = target_airspeed;
        target_roll = (float)(M_PI * 0.5 - atan2(
            G_ACCEL * FCS_CONTROL_TURN_RADIUS,
            (tangent_ground_speed * tangent_ground_speed)));
        if (last_segment == FCS_WAYPOINT_FLAG_FIGURE8_LEFT_CURVE) {
            target_roll = -target_roll;
        }

        delta_yaw = min(
            absval(_angle_diff(target_yaw, 0.0f)),
            absval(_angle_diff(target_yaw, (float)M_PI * 1.5f)));
        if (delta_yaw < 0.333333f) {
            /*
            Scale roll angle to avoid discontinuity at start and end of
            curved segments
            */
            target_roll *= delta_yaw * 3.0f;
        }
    } else {
        target_roll = 0.0f;
    }

    /* Transform UV into NE through rotation by start->yaw */
    offset_n = offset_u * cy - offset_v * sy;
    offset_e = offset_u * sy + offset_v * cy;

    offset_n *= FCS_CONTROL_TURN_RADIUS;
    offset_e *= FCS_CONTROL_TURN_RADIUS;

    /*
    Don't need to multiply offsets by t here, since it's done in the yaw
    calculation, from which these offsets are derived.
    */
    new_point->lat = start->lat + (1.0/WGS84_A) * offset_n;
    new_point->lon = start->lon + (1.0/WGS84_A) * offset_e / cos(start->lat);

    new_point->yaw = mod_2pi_f(start->yaw + target_yaw);
    new_point->pitch = 3.0f * ((float)M_PI / 180.0f);
    new_point->roll = target_roll;
    new_point->alt = start->alt;
    new_point->airspeed = target_airspeed;
    new_point->flags = new_segment;

    /* Always returning t means we never advance to the next path. */
    return t;
}
