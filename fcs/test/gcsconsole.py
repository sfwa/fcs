#!/usr/bin/env python
# coding=utf-8

#Copyright (C) 2013 Ben Dyer
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import sys
import plog
import time
import math
import serial
import binascii
import ConfigParser

def tick(t, data):
    global LAST_CONTROL_DATA

    state_pos = None
    state_velocity = (-1, -1, -1)
    state_wind = (0, 0, 0)
    state_attitude = (0, 0, 0)
    state_angular_velocity = (0, 0, 0)
    control_refp = None
    control_path = -1
    control_cycles = -1
    control_obj_val = -1
    control_errors = -1
    control_resets = -1
    control_values = (-1, -1, -1)
    control_mode = [-1, -1]
    control_nav_version = -1
    control_pos = (-1, -1, -1)
    control_error_type = 0
    last_gps = 0
    last_data = 0

    for param in data:
        pt = param.parameter_type
        if isinstance(param, plog.DataParameter):
            pv = param.values
        else:
            pv = param.value

        if pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA:
            state_pos = (
                pv[0] * math.pi / 2**31,
                pv[1] * math.pi / 2**31,
                pv[2] * 1e-2
            )
        elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_VELOCITY_NED:
            state_velocity = map(lambda x: float(x) * 1e-2, pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q:
            att_q = map(lambda x: float(x) / 32767.0, pv)
            state_attitude = plog.q_to_euler(att_q)
        elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED:
            state_wind = map(lambda x: float(x) * 1e-2, pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_AHRS_MODE:
            mode = chr(pv[0])
        elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_MODE:
            control_mode[param.device_id] = pv[0]
        elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_SETPOINT:
            control_values = map(lambda x: float(x) / 65535.0, pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_NAV_PATH_ID:
            control_path = pv[0]
            control_error_type = pv[1]
        elif pt == plog.ParameterType.FCS_PARAMETER_KEY_VALUE:
            control_refp = plog.extract_waypoint(pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_STATUS:
            control_cycles = pv[0]
            control_obj_val = pv[1]
            control_errors = pv[2]
            control_resets = pv[3]
        elif pt == plog.ParameterType.FCS_PARAMETER_AHRS_STATUS:
            last_gps = pv[0]
            last_data = pv[1]
        elif pt == plog.ParameterType.FCS_PARAMETER_NAV_VERSION:
            control_nav_version = pv[0]


    control_data = (control_cycles, control_obj_val, control_errors,
                    control_resets)
    if not state_pos or not control_refp:
        return ""

    LAST_CONTROL_DATA = control_data

    state_airspeed = math.sqrt((state_velocity[0] - state_wind[0])**2 +
                               (state_velocity[1] - state_wind[1])**2 +
                               (state_velocity[2] - state_wind[2])**2)

    return (
        "t=%8d, " +
        "alt=%3.3f, alt_ref=%3.3f, " +
        "n=%6.2f, e=%6.2f, d=%6.2f, " +
        "tas=%5.2f, tas_ref=%5.2f, " +
        "yaw=%3.0f, yaw_ref=%3.0f, " +
        "pitch=%4.0f, pitch_ref=%4.0f, " +
        "roll=%4.0f, roll_ref=%4.0f, " +
        "t=%.3f, l=%.3f, r=%.3f, " +
        "objval=%10.1f, cycles=%9d, errors=%9d, resets=%9d, " +
        "path=%4d, last_gps=%4d, mode1=%d, nav_state=%d"
    ) % (
        (t, ) +
        (state_pos[2], control_refp.get("alt", 0)) +
        plog.lla_to_ned(
            (control_refp.get("lat", 0), control_refp.get("lon", 0),
                control_refp.get("alt", 0)),
            state_pos) +
        (state_airspeed, control_refp.get("airspeed", 0)) +
        (state_attitude[0], math.degrees(control_refp.get("yaw", 0))) +
        (state_attitude[1], math.degrees(control_refp.get("pitch", 0))) +
        (state_attitude[2], math.degrees(control_refp.get("roll", 0))) +
        (control_values[0], control_values[1], control_values[2]) +
        (control_obj_val, control_cycles, control_errors, control_resets) +
        (control_path, last_gps, control_mode[1], control_nav_version)
    )


if __name__ == "__main__":
    if not sys.argv or len(sys.argv) < 2:
        print "Usage: serialdump /PATH/TO/DEVICE"
        sys.exit(1)

    start_t = time.time()

    conn = serial.Serial(sys.argv[1], 57600, timeout=None)
    for logl in plog.iterlogs(conn):
        try:
            result = tick(time.time() - start_t, logl)
            if result:
                print result
        except Exception:
            raise

        control_config = ConfigParser.RawConfigParser()
        control_config.read('dynamics.conf')
        control_params = [
            control_config.getint('default', 'roll_due_to_control'),
            control_config.getint('default', 'roll_due_to_beta'),
            control_config.getint('default', 'roll_due_to_roll_rate'),
            control_config.getint('default', 'pitch_due_to_control'),
            control_config.getint('default', 'yaw_due_to_control'),
            control_config.getint('default', 'yaw_due_to_beta'),
            control_config.getint('default', 'yaw_due_to_yaw_rate')
        ]
        control_param_log = plog.ParameterLog()
        control_param_log.append(plog.KeyValueParameter(
            device_id=0, key='parm',
            value="".join(map(lambda x: chr(x), control_params))))

        conn.write(control_param_log.serialize())
        print "Sent: %s" % binascii.b2a_hex(control_param_log.serialize())
