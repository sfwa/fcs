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

#ifndef _FCS_AHRS_H
#define _FCS_AHRS_H

enum fcs_mode_t {
    /* Placeholder */
    FCS_MODE_STARTUP_VALUE = 0,
    /*
    State: initializing (I)
    Previous states: none
    Entry condition: startup
    Control output: none
    */
    FCS_MODE_INITIALIZING = 'I',
    /*
    State: simulating (M)
    Previous states: initializing, calibrating, safe
    Entry condition: packet on HITL port
    Control output: from NMPC
    */
    FCS_MODE_SIMULATING = 'M',
    /*
    State: calibrating (C)
    Previous states: initializing, safe, simulating
    Entry condition: received GPS packet && (mode == initializing ||
        calibration command from GCS || no HITL packet received for 5s)
    Control output: none
    */
    FCS_MODE_CALIBRATING = 'C',
    /*
    State: safe (S)
    Previous states: calibrating, armed, active
    Entry condition: time since last GPS packet received < 1s &&
        (time since entered calibration > 30s ||
        safe command from GCS)
    Control output: none
    */
    FCS_MODE_SAFE = 'S',
    /*
    State: armed (R)
    Previous states: safe
    Entry condition: arm command from GCS && all sensor health == OK &&
        mission boundary defined && home waypoint defined &&
        recovery waypoint defined
    Control output: take-off, failsafe device arm asserted
    */
    FCS_MODE_ARMED = 'R',
    /*
    State: active (A)
    Previous states: armed, holding
    Entry condition: freefall detected (take-off) || activate command from GCS
    Control output: from NMPC
    Waypoint mode: normal
    */
    FCS_MODE_ACTIVE = 'A',
    /*
    State: holding (H)
    Previous states: active
    Entry condition: lost GPS || hold command from GCS
    Control output: from NMPC
    Waypoint mode: hold at current position
    */
    FCS_MODE_HOLDING = 'H',
    /*
    State: abort (F)
    Previous states: armed, active, holding
    Entry condition: mission boundary crossed ||
        (lost data link && lost gps) ||
        termination requested || sensor health != OK
    Control output: failsafe (all min), failsafe device abort asserted
    */
    FCS_MODE_ABORT = 'F'
};

void fcs_ahrs_init(void);
void fcs_ahrs_tick(void);

/*
Change the AHRS mode. Returns false if the mode change is rejected, or true
if successful.
*/
bool fcs_ahrs_set_mode(enum fcs_mode_t mode);

#endif
