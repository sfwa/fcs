# Split a combined measurement log into two I/O board logs with alternating
# packets

import os
import fcs
import sys
import plog
import math
import time
import vectors
import binascii


LAST_CONTROL_DATA = None


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
        elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ:
            state_angular_velocity = map(
                lambda x: math.degrees(float(x) / (32767.0 / math.pi * 0.25)),
                pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED:
            state_wind = map(lambda x: float(x) * 1e-2, pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_AHRS_MODE:
            mode = chr(pv[0])
        elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_MODE:
            control_mode[param.device_id] = pv[0]
        elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_SETPOINT:
            control_values = map(lambda x: float(x) / 65535.0, pv)
        elif pt == plog.ParameterType.FCS_PARAMETER_CONTROL_POS and param.device_id == 1:
            control_pos = map(lambda x: float(x) / 65535.0, pv)
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
    if not state_pos or not control_refp or LAST_CONTROL_DATA == control_data:
        return ""
    if control_mode[1] != 1:
        return ""

    LAST_CONTROL_DATA = control_data

    state_airspeed = math.sqrt((state_velocity[0] - state_wind[0])**2 +
                               (state_velocity[1] - state_wind[1])**2 +
                               (state_velocity[2] - state_wind[2])**2)

    ned_off = plog.lla_to_ned(
        state_pos,
        (control_refp["lat"], control_refp["lon"], control_refp["alt"]))
    ref_v = (
        control_refp["airspeed"] * math.cos(control_refp["yaw"]) + state_wind[0],
        control_refp["airspeed"] * math.sin(control_refp["yaw"]) + state_wind[1]
    )
    ate = abs((ned_off[0] * ref_v[0] + ned_off[1] * ref_v[1]) / \
              math.sqrt(ref_v[0] ** 2 + ref_v[1] ** 2))
    xte = math.sqrt((ned_off[0] ** 2 + ned_off[1] ** 2) - ate ** 2)

    return (
        "t=%8d, " +
        "alt=%3.3f, alt_ref=%3.3f, " +
        "n=%6.2f, e=%6.2f, d=%6.2f, xte=%3.2f, ate=%3.2f, " +
        "tas=%5.2f, tas_ref=%5.2f, " +
        "heading=%3.0f, heading_ref=%3.0f, " +
        "yaw=%3.0f, yaw_ref=%3.0f, " +
        "pitch=%4.0f, pitch_ref=%4.0f, " +
        "roll=%4.0f, roll_ref=%4.0f, " +
        "vyaw=%4.0f, vpitch=%4.0f, vroll=%4.0f, " +
        "t=%.3f, l=%.3f, r=%.3f, " +
        "tp=%.3f, lp=%.3f, rp=%.3f, " +
        "objval=%10.1f, cycles=%9d, errors=%9d, resets=%9d, " +
        "path=%4d, last_gps=%4d, mode1=%d, flags=%08x, err=%04x\n"
    ) % (
        (t, ) +
        (state_pos[2], control_refp["alt"]) +
        ned_off + (xte, ate) +
        (state_airspeed, control_refp["airspeed"]) +
        (math.degrees(math.atan2(state_velocity[1], state_velocity[0])),
            math.degrees(math.atan2(ref_v[1], ref_v[0]))) +
        (state_attitude[0], math.degrees(control_refp["yaw"])) +
        (state_attitude[1], math.degrees(control_refp["pitch"])) +
        (state_attitude[2], math.degrees(control_refp["roll"])) +
        (state_angular_velocity[2], state_angular_velocity[1],
            state_angular_velocity[0]) +
        (control_values[0], control_values[1], control_values[2]) +
        (control_pos[0], control_pos[1], control_pos[2]) +
        (control_obj_val, control_cycles, control_errors, control_resets) +
        (control_path, last_gps, control_mode[1], control_refp["flags"],
            control_error_type)
    )


if __name__ == "__main__":
    fcs.init(sys.argv[1])

    infile = open(sys.argv[2], 'rb') if len(sys.argv) > 2 else sys.stdin
    outfile = open(sys.argv[3], 'wb') if len(sys.argv) > 3 else sys.stdout

    i = 0
    for logf in plog.iterlogs(infile):
        try:
            outfile.write(tick(i, logf))
            outfile.flush()
        except Exception:
            raise
        i += 1

    infile.close()
    outfile.close()
