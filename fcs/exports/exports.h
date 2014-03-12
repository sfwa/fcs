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

#ifndef EXPORTS_H_
#define EXPORTS_H_

struct fcs_state_estimate_t {
    double lat;
    double lon;
    float alt;
    float velocity[3];
    float attitude[4];
    float angular_velocity[3];
    float wind_velocity[3];
    uint8_t mode;
    /* Pad so the structure fills two whole L1 cache lines (128 bytes) */
    uint8_t reserved[55];
};

struct fcs_control_output_t {
    uint64_t nmpc_errors;
    uint64_t nmpc_resets;

    float values[4];
    float rates[4];

    float objective_val;
    uint32_t cycles;

    double reference_lat;
    double reference_lon;
    float reference_alt;
    float reference_airspeed;
    float reference_yaw;
    float reference_pitch;
    float reference_roll;

    uint32_t nav_state_version;
    uint16_t path_id;

    uint8_t gpio;
    uint8_t mode;

    /* Pad so the structure fills two whole L1 cache lines (128 bytes) */
    uint8_t reserved[32];
};

void fcs_exports_init(void);

void fcs_exports_send_state(void);
void fcs_exports_send_control(void);

void fcs_exports_recv_state(struct fcs_state_estimate_t *state);
void fcs_exports_recv_control(struct fcs_control_output_t *control);

#endif
