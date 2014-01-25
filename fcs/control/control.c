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
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "control.h"

struct fcs_control_state_t fcs_global_control_state;

void fcs_control_init(void) {
    /*
    TODO
    - Configure NMPC data structures
    - Initialize default plans and waypoints lists
    */

    /* Clear GPIO outs */
    fcs_global_control_state.gpio_state = 0;

    /*
    Configure throttle control: 0-18000 RPM, cruise at 15000, rate change at
    9000 RPM/sec
    */
    fcs_global_control_state.controls[0].setpoint = 15000.0f;
    fcs_global_control_state.controls[0].min = 0.0f;
    fcs_global_control_state.controls[0].max = 18000.0f;
    fcs_global_control_state.controls[0].rate = 9000.0f;

    /*
    Configure left elevon: neutral setpoint, +/- 45 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[1].setpoint = 0.0f;
    fcs_global_control_state.controls[1].min = -M_PI * 0.25f;
    fcs_global_control_state.controls[1].max = M_PI * 0.25f;
    fcs_global_control_state.controls[1].rate = M_PI * 0.3333333f;

    /*
    Configure right elevon: neutral setpoint, +/- 45 deg travel, 60 deg/s rate
    */
    fcs_global_control_state.controls[2].setpoint = 0.0f;
    fcs_global_control_state.controls[2].min = -M_PI * 0.25f;
    fcs_global_control_state.controls[2].max = M_PI * 0.25f;
    fcs_global_control_state.controls[2].rate = M_PI * 0.3333333f;

    /* Set final (unused) control channel */
    fcs_global_control_state.controls[3].setpoint = 0.0f;
    fcs_global_control_state.controls[3].min = 0.0f;
    fcs_global_control_state.controls[3].max = 1.0f;
    fcs_global_control_state.controls[3].rate = 1.0f;
}

void fcs_control_tick(void) {
    /*
    TODO
    - Iterate over waypoints in current trajectory (including the current
      state as the first waypoint, and the projection of the current position
      onto the path between the previous waypoint and the current waypoint
      as the second waypoint)
      - The reference heading of each waypoint is the midpoint of the entry
        track (from previous waypoint) and the exit track (to next waypoint)
      - If the difference between entry and exit tracks is more than X,
        generate a Dubins path from the previous waypoint to the current
        waypoint, and the current waypoint to the next waypoint
      - Interpolate linearly between all arc endpoints
      - Terminate iteration once the NMPC horizon has been filled

    - Pass the interpolated waypoint list to the NMPC code as a reference
      trajectory
    - Evaluate the control problem based on the latest state
    - Output the control values for the next frame
    - Check mission boundary
    - Check error conditions
    */
}
