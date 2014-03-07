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
#include <assert.h>
#include <float.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../nmpc/cnmpc.h"
#include "../exports/exports.h"
#include "control.h"
#include "trajectory.h"


float fcs_trajectory_interpolate_linear(struct fcs_waypoint_t *new_point,
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
    new_point->pitch = start->pitch + x * (end->pitch - start->pitch);

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
