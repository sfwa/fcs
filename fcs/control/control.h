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

enum fcs_control_mode_t {
    FCS_CONTROL_MODE_STARTUP_VALUE = 0,
    FCS_CONTROL_MODE_MANUAL = 'R',
    FCS_CONTROL_MODE_AUTO = 'F'
};

/*
If the vehicle is more than 10m from where it should be, switch to a recovery
trajectory to get back on track.
*/
#define FCS_CONTROL_POSITION_TOLERANCE 10.0

/* Airspeed used for holding patterns etc, in m/s */
#define FCS_CONTROL_DEFAULT_AIRSPEED 20.0

/* Turn radius in metres */
#define FCS_CONTROL_TURN_RADIUS 60.0

/*
The number of control ticks (@ 50Hz) after which the path should be recomputed
*/
#define FCS_CONTROL_INFEASIBILITY_TIMEOUT 1u

enum fcs_path_type_t {
    FCS_PATH_LINE = 'L',
    FCS_PATH_DUBINS_CURVE = 'D',
    FCS_PATH_FIGURE_EIGHT = '8',
    FCS_PATH_INVALID = 0
};

struct fcs_control_channel_t {
    float setpoint;
    float rate;
};

struct fcs_waypoint_t {
    double lat, lon;
    float alt;
    float airspeed;
    float yaw, pitch, roll;
    uint32_t flags;
};

/*
These flags are used to identify point types in the reference trajectory
generation system.
*/
#define FCS_WAYPOINT_FLAG_PARAM_MASK 0x00000003u
#define FCS_WAYPOINT_FLAG_PARAM_OFFSET 0

#define FCS_WAYPOINT_FLAG_DUBINS_RSR 0x0u
#define FCS_WAYPOINT_FLAG_DUBINS_LSL 0x1u
#define FCS_WAYPOINT_FLAG_DUBINS_RSL 0x2u
#define FCS_WAYPOINT_FLAG_DUBINS_LSR 0x3u

#define FCS_WAYPOINT_FLAG_SEGMENT_MASK 0x0000000Cu
#define FCS_WAYPOINT_FLAG_SEGMENT_OFFSET 2u


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

/* Marker for 'stabilise after loss of control' path */
#define FCS_CONTROL_STABILISE_PATH_ID (FCS_CONTROL_MAX_PATHS - 4u)

/* Marker for 'last plan position' waypoint ID */
#define FCS_CONTROL_HOLD_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 1u)

/*
Marker for 'interpolate from an arbitrary state to the next path' waypoint ID
*/
#define FCS_CONTROL_INTERPOLATE_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 2u)

/* Marker for 'partially-flown path resume position' waypoint ID */
#define FCS_CONTROL_RESUME_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 3u)

/* Marker for 'stabilise path commence' waypoint ID */
#define FCS_CONTROL_STABILISE_WAYPOINT_ID (FCS_CONTROL_MAX_WAYPOINTS - 4u)

struct fcs_state_estimate_t {
    double lat;
    double lon;
    float alt;
    float velocity[3];
    float attitude[4];
    float angular_velocity[3];
    float wind_velocity[3];
    /* Pad so the structure fills two whole L1 cache lines (128 bytes) */
    uint8_t reserved[56];
};

struct fcs_boundary_t {
    uint16_t num_waypoint_ids;
    uint16_t waypoint_ids[FCS_CONTROL_BOUNDARY_MAX_WAYPOINTS];
    uint8_t flags;
};

struct fcs_control_state_t {
    struct fcs_control_channel_t controls[FCS_CONTROL_CHANNELS];
    enum fcs_control_mode_t mode;
    uint8_t gpio_state;
};

struct fcs_nav_state_t {
    struct fcs_path_t paths[FCS_CONTROL_MAX_PATHS];
    struct fcs_waypoint_t waypoints[FCS_CONTROL_MAX_WAYPOINTS];
    struct fcs_boundary_t boundary;

    struct fcs_waypoint_t reference_trajectory[OCP_HORIZON_LENGTH + 1u];
    uint16_t reference_path_id[OCP_HORIZON_LENGTH + 1u];
};

extern volatile struct fcs_control_state_t fcs_global_control_state;
extern volatile struct fcs_nav_state_t fcs_global_nav_state;

void fcs_control_init(void);
void fcs_control_tick(void);

#endif
