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

#ifndef TRAJECTORY_H_

#define WGS84_A 6378137.0

struct fcs_state_estimate_t {
    double lat;
    double lon;
    float alt;
    float velocity[3];
    float attitude[4];
    float angular_velocity[3];
    float wind_velocity[3];
    uint8_t mode;
};

float fcs_trajectory_interpolate_linear(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

float fcs_trajectory_interpolate_figure_eight(
struct fcs_waypoint_t *new_point, const struct fcs_waypoint_t *last_point,
const float *restrict wind, const struct fcs_waypoint_t *start,
const struct fcs_waypoint_t *end, float t);

float fcs_trajectory_interpolate_dubins(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

void fcs_trajectory_recalculate(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate);

void fcs_trajectory_timestep(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate);

void fcs_trajectory_start_recover(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate);

void fcs_trajectory_start_hold(struct fcs_nav_state_t *nav,
const struct fcs_state_estimate_t *restrict state_estimate);

/* FIXME: rename/relocate */
void _ned_from_point_diff(float *restrict ned,
const struct fcs_waypoint_t *restrict ref,
const struct fcs_waypoint_t *restrict point);

#endif
