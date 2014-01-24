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
#define FCS_CONTROL_MAX_PLANS 16u
#define FCS_CONTROL_BOUNDARY_MAX_WAYPOINTS 64u
#define FCS_CONTROL_PLAN_MAX_WAYPOINTS 512u

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
    uint8_t flags;
};

struct fcs_plan_t {
    uint16_t waypoint_count;
    uint16_t waypoint_ids[FCS_CONTROL_PLAN_MAX_WAYPOINTS];
    uint16_t next_waypoint_id_index;
    uint16_t chained_plan_id;
    uint8_t flags;
};

/* Marker for empty chained plan ID */
#define FCS_CONTROL_PLAN_ID_INVALID 0xFFFFu

struct fcs_boundary_t {
    uint16_t num_waypoint_ids;
    uint16_t waypoint_ids[FCS_CONTROL_BOUNDARY_MAX_WAYPOINTS];
    uint8_t flags;
};

struct fcs_control_state_t {
    struct fcs_control_channel_t controls[FCS_CONTROL_CHANNELS];
    struct fcs_plan_t plans[FCS_CONTROL_MAX_PLANS];
    struct fcs_waypoint_t waypoints[FCS_CONTROL_MAX_WAYPOINTS];
    struct fcs_boundary_t boundary;

    /*
    The plan at the top of the stack (plan_stack_ids[plan_stack_count-1])
    is currently running. Once it concludes, chained_plan_id will replace it
    on the stack unless it's FCS_CONTROL_PLAN_ID_INVALID.

    If the plan at the top of the stack concludes without a chained plan being
    set, it is removed from the stack and execution of the next plan down
    resumes.
    */
    uint16_t plan_stack_count;
    uint16_t plan_stack_ids[FCS_CONTROL_MAX_PLANS];
};

extern struct fcs_control_state_t fcs_global_control_state;

void fcs_control_init(void);
void fcs_control_tick(void);

#endif
