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

#ifndef _FCS_CONTROL_H
#define _FCS_CONTROL_H

#define FCS_CONTROL_CHANNELS 4u
#define FCS_CONTROL_MAX_WAYPOINTS 1000u
#define FCS_CONTROL_MAX_PATHS 500u
#define FCS_CONTROL_BOUNDARY_MAX_WAYPOINTS 64u

/*
If the vehicle is more than 10m from where it should be, switch to a recovery
trajectory to get back on track.
*/
#define FCS_CONTROL_POSITION_TOLERANCE 10.0

/* Airspeed used for holding patterns etc, in m/s */
#define FCS_CONTROL_DEFAULT_AIRSPEED 20.0

/* Turn radius in metres */
#define FCS_CONTROL_TURN_RADIUS 20.0

enum fcs_path_type_t {
    FCS_PATH_LINE,
    FCS_PATH_CURVE_SHORTEST,
    FCS_PATH_CURVE_LSL,
    FCS_PATH_CURVE_LSR,
    FCS_PATH_CURVE_RSL,
    FCS_PATH_CURVE_RSR,
    FCS_PATH_CURVE_RLR,
    FCS_PATH_CURVE_LRL,
    FCS_PATH_FIGURE_EIGHT,

    /* Sentinel */
    FCS_PATH_INVALID
};

struct fcs_control_channel_t {
    float setpoint;
    float min;
    float max;
    float rate;
};

struct fcs_waypoint_t {
    double lat, lon;
    float alt;
    float airspeed;
    float yaw, pitch, roll;
    uint32_t flags;
};

struct fcs_path_t {
    uint16_t start_waypoint_id;
    uint16_t end_waypoint_id;
    enum fcs_path_type_t type;

    uint16_t flags;
    uint16_t next_path_id;
};

/* Invalid/uninitialized path ID */
#define FCS_CONTROL_INVALID_PATH_ID 0xFFFFu

/* Marker for 'hold at HOLD_WAYPOINT_ID' path */
#define FCS_CONTROL_HOLD_PATH_ID (FCS_CONTROL_MAX_PATHS - 1u)

/* Marker for 'interpolate from an arbitrary state to the next path' path */
#define FCS_CONTROL_INTERPOLATE_PATH_ID (FCS_CONTROL_MAX_PATHS - 2u)

/* Marker for 'resume following a partially-flown path' path */
#define FCS_CONTROL_RESUME_PATH_ID (FCS_CONTROL_MAX_PATHS - 3u)

/* Marker for 'last plan position' waypoint ID */
#define FCS_CONTROL_HOLD_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 1u)

/*
Marker for 'interpolate from an arbitrary state to the next path' waypoint ID
*/
#define FCS_CONTROL_INTERPOLATE_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 2u)

/* Marker for 'partially-flown path resume position' waypoint ID */
#define FCS_CONTROL_RESUME_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 3u)

struct fcs_boundary_t {
    uint16_t num_waypoint_ids;
    uint16_t waypoint_ids[FCS_CONTROL_BOUNDARY_MAX_WAYPOINTS];
    uint8_t flags;
};

struct fcs_control_state_t {
    struct fcs_control_channel_t controls[FCS_CONTROL_CHANNELS];
    uint8_t gpio_state;

    enum fcs_control_status_t status;
};

struct fcs_global_nav_state_t {
    struct fcs_path_t paths[FCS_CONTROL_MAX_PATHS];
    struct fcs_waypoint_t waypoints[FCS_CONTROL_MAX_WAYPOINTS];
    struct fcs_boundary_t boundary;

    struct fcs_waypoint_t reference_trajectory[OCP_HORIZON_LENGTH];
    uint16_t reference_path_id[OCP_HORIZON_LENGTH];
};

extern struct fcs_control_state_t fcs_global_control_state;
extern struct fcs_nav_state_t fcs_global_nav_state;

void fcs_control_init(void);
void fcs_control_tick(void);

#endif
