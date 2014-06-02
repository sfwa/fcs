# Copyright (C) 2013 Ben Dyer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

import os
import fcs
import sys
import plog
import copy
import math
import time
import socket
import vectors
import datetime
import binascii
import collections
from ctypes import *


"""
Driver for SITL testing of the FCS navigation and control algorithms in an
X-Plane simulation.

Does not currently simulate FCS comms or AHRS functionality.
"""

ahrs_tick = 0


START_LAT = -26.584781
START_LON = 151.842325
START_ALT = 460


sim_state = {
    "lat": None,
    "lon": None,
    "alt": 0.0,
    "velocity": [0.0, 0.0, 0.0],
    "attitude": [0.0, 0.0, 0.0, 1.0],
    "angular_velocity": [0.0, 0.0, 0.0],
    "wind_velocity": [0.0, 0.0, 0.0]
}
sim_state_delay = collections.deque()


sim_ref = {
    "wind_n": None,
    "wind_e": 0.0,
    "wind_d": 0.0,
    "attitude_yaw": 0.0,
    "attitude_pitch": 0.0,
    "attitude_roll": 0.0,
    "airspeed": 0.0
}


def socket_readlines(socket):
    buf = socket.recv(4096)
    done = False
    while 1:
        if "\n" in buf:
            (line, buf) = buf.split("\n", 1)
            yield line + "\n"
        else:
            break


def tick(lat=None, lon=None, alt=None, velocity=None, attitude=None,
         angular_velocity=None, wind_velocity=None):
    """
    Runs the FCS control and comms tasks with the state data provided as
    though it came from the AHRS, and returns the control output.
    """
    if not fcs._fcs:
        raise RuntimeError("Please call fcs.init()")

    estimate_log = plog.ParameterLog(
        log_type=plog.LogType.FCS_LOG_TYPE_ESTIMATE)

    estimate_log.append(
        plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA,
            value_type=plog.ValueType.FCS_VALUE_SIGNED,
            value_precision=32,
            values=[
                int(lat * (2**31 - 1) / math.pi),
                int(lon * (2**31 - 1) / math.pi),
                int(alt * 1e2)
            ]
        )
    )

    estimate_log.append(
        plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_VELOCITY_NED,
            value_type=plog.ValueType.FCS_VALUE_SIGNED,
            value_precision=16,
            values=map(lambda x: int(x * 1e2), velocity)
        )
    )

    estimate_log.append(
        plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q,
            value_type=plog.ValueType.FCS_VALUE_SIGNED,
            value_precision=16,
            values=map(lambda x: int(x * (2**15 - 1)), attitude)
        )
    )

    estimate_log.append(
        plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ,
            value_type=plog.ValueType.FCS_VALUE_SIGNED,
            value_precision=16,
            values=map(lambda x: int(x * (2**15 - 1) / math.pi * 0.25), angular_velocity)
        )
    )

    estimate_log.append(
        plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED,
            value_type=plog.ValueType.FCS_VALUE_SIGNED,
            value_precision=16,
            values=map(lambda x: int(x * 1e2), wind_velocity)
        )
    )

    fcs.write(3, estimate_log.serialize())
    #print binascii.b2a_hex(estimate_log.serialize())

    fcs._fcs.fcs_board_tick()
    fcs._fcs.fcs_ahrs_tick()
    fcs._fcs.fcs_control_tick()

    # Read out ignored streams
    fcs.read(0, 1023)
    fcs.read(1, 1023)
    fcs.read(2, 1023)
    fcs.read(4, 1023)

    try:
        control_log = plog.ParameterLog.deserialize(fcs.read(3, 1023))

        control_param = control_log.find_by(device_id=0, parameter_type=plog.ParameterType.FCS_PARAMETER_CONTROL_SETPOINT)
        return map(lambda x: float(x) / float(2**16), control_param.values)
    except Exception:
        return [0.0, 0.5, 0.5]


def connect_to_xplane():
    # Disable X-Plane simulation and set up initial position.
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('127.0.0.1', 51000))
    sock.sendall("sub sim/operation/override/override_planepath\n")
    sock.sendall("set sim/operation/override/override_planepath [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n")
    sock.sendall("sub sim/operation/override/override_control_surfaces\n")
    sock.sendall("set sim/operation/override/override_control_surfaces 1\n")
    sock.sendall("sub sim/operation/override/override_throttles\n")
    sock.sendall("set sim/operation/override/override_throttles 1\n")
    sock.sendall("sub sim/flightmodel/position/q\n")
    sock.sendall("sub sim/flightmodel/position/psi 0\n")  # yaw
    sock.sendall("sub sim/flightmodel/position/theta 0\n")  # pitch
    sock.sendall("sub sim/flightmodel/position/phi 0\n")  # roll
    sock.sendall("sub sim/flightmodel/position/P 0\n")
    sock.sendall("sub sim/flightmodel/position/Q 0\n")
    sock.sendall("sub sim/flightmodel/position/R 0\n")
    sock.sendall("sub sim/flightmodel/position/latitude 0\n")
    sock.sendall("sub sim/flightmodel/position/longitude 0\n")
    sock.sendall("sub sim/flightmodel/position/elevation 0\n")
    sock.sendall("sub sim/flightmodel/position/local_x 0\n")
    sock.sendall("sub sim/flightmodel/position/local_y 0\n")
    sock.sendall("sub sim/flightmodel/position/local_z 0\n")
    sock.sendall("sub sim/flightmodel/position/local_vx 0\n")
    sock.sendall("sub sim/flightmodel/position/local_vy 0\n")
    sock.sendall("sub sim/flightmodel/position/local_vz 0\n")
    sock.sendall("sub sim/flightmodel/engine/ENGN_thro_use\n")
    sock.sendall("sub sim/flightmodel/controls/wing1l_ail1def\n")
    sock.sendall("sub sim/flightmodel/controls/wing1r_ail1def\n")
    sock.sendall("sub sim/weather/wind_now_x_msc\n")
    sock.sendall("sub sim/weather/wind_now_y_msc\n")
    sock.sendall("sub sim/weather/wind_now_z_msc\n")
    sock.sendall("sub sim/flightmodel/position/true_airspeed 0\n")
    sock.sendall("extplane-set update_interval 0.02\n")

    # Wait for some data
    sock.recv(1024)

    return sock


def reset_xplane_state(s, yaw=18.0, pitch=0.0, roll=0.0, velocity=None):
    disable_xplane_sim(s)

    s.sendall("world-set %f %f %f\n" % (START_LAT, START_LON, START_ALT))
    s.sendall("sub sim/operation/failures/rel_engfir0")
    time.sleep(1.0)

    # Clear the engine fire -- this happens whenever the X8 is on the ground
    s.sendall("set sim/operation/failures/rel_engfir0 0")
    s.sendall("unsub sim/operation/failures/rel_engfir0")

    if not velocity:
        velocity = [-20.0, 0.0, 0.0]

    # Set the simulation's initial state
    update = ""

    xplane_q = [0, 0, 0, 1]
    psi = yaw / 2.0
    theta = pitch / 2.0
    phi = roll / 2.0
    xplane_q[0] = math.cos(psi) * math.cos(theta) * math.cos(phi) + math.sin(psi) * math.sin(theta) * math.sin(phi)
    xplane_q[1] = math.cos(psi) * math.cos(theta) * math.sin(phi) - math.sin(psi) * math.sin(theta) * math.cos(phi)
    xplane_q[2] = math.cos(psi) * math.sin(theta) * math.cos(phi) + math.sin(psi) * math.cos(theta) * math.sin(phi)
    xplane_q[3] = -math.cos(psi) * math.sin(theta) * math.sin(phi) + math.sin(psi) * math.cos(theta) * math.cos(phi)

    update += "set sim/flightmodel/position/q [%.6f,%.6f,%.6f,%.6f]\n" % (xplane_q[0], xplane_q[1], xplane_q[2], xplane_q[3])
    update += "set sim/flightmodel/position/psi %.6f\n" % math.degrees(yaw)
    update += "set sim/flightmodel/position/theta %.6f\n" % math.degrees(pitch)
    update += "set sim/flightmodel/position/phi %.6f\n" % math.degrees(roll)

    update += "set sim/flightmodel/position/local_vx %.6f\n" % velocity[1]
    update += "set sim/flightmodel/position/local_vy %.6f\n" % -velocity[2]
    update += "set sim/flightmodel/position/local_vz %.6f\n" % -velocity[0]

    update += "set sim/flightmodel/position/P 0\n"
    update += "set sim/flightmodel/position/Q 0\n"
    update += "set sim/flightmodel/position/R 0\n"

    # Zero controls.
    update += "set sim/flightmodel/engine/ENGN_thro [0,0,0,0,0,0,0,0]\n"
    update += "set sim/flightmodel/controls/wing1l_ail1def 0\n"
    update += "set sim/flightmodel/controls/wing1r_ail1def 0\n"

    s.sendall(update)


def enable_xplane_sim(s):
    s.sendall("set sim/operation/override/override_planepath [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n")
    s.setblocking(0)

    update = ""

    yaw = math.radians(180)
    pitch = math.radians(0)
    roll = math.radians(0)
    velocity = [-20.0, 0.0, 0.0]

    xplane_q = [0, 0, 0, 1]
    psi = yaw / 2.0
    theta = pitch / 2.0
    phi = roll / 2.0
    xplane_q[0] = math.cos(psi) * math.cos(theta) * math.cos(phi) + math.sin(psi) * math.sin(theta) * math.sin(phi)
    xplane_q[1] = math.cos(psi) * math.cos(theta) * math.sin(phi) - math.sin(psi) * math.sin(theta) * math.cos(phi)
    xplane_q[2] = math.cos(psi) * math.sin(theta) * math.cos(phi) + math.sin(psi) * math.cos(theta) * math.sin(phi)
    xplane_q[3] = -math.cos(psi) * math.sin(theta) * math.sin(phi) + math.sin(psi) * math.cos(theta) * math.cos(phi)

    update += "set sim/flightmodel/position/q [%.6f,%.6f,%.6f,%.6f]\n" % (xplane_q[0], xplane_q[1], xplane_q[2], xplane_q[3])
    update += "set sim/flightmodel/position/psi %.6f\n" % math.degrees(yaw)
    update += "set sim/flightmodel/position/theta %.6f\n" % math.degrees(pitch)
    update += "set sim/flightmodel/position/phi %.6f\n" % math.degrees(roll)

    update += "set sim/flightmodel/position/local_vx %.6f\n" % velocity[1]
    update += "set sim/flightmodel/position/local_vy %.6f\n" % -velocity[2]
    update += "set sim/flightmodel/position/local_vz %.6f\n" % -velocity[0]

    update += "set sim/flightmodel/position/P %.6f\n" % math.radians(14.4)
    update += "set sim/flightmodel/position/Q %.6f\n" % math.radians(-5.0)
    update += "set sim/flightmodel/position/R %.6f\n" % math.radians(-6.7)
    s.sendall(update)


def disable_xplane_sim(s):
    s.setblocking(1)
    sock.sendall("set sim/operation/override/override_planepath [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n")


def recv_state_from_xplane(s):
    global sim_state, sim_ref

    # Get latest "measured" data.
    try:
        for line in socket_readlines(s):
            fields = line.split(" ")
            if len(fields) != 3:
                continue
            if fields[1] == "sim/flightmodel/position/latitude":
                sim_state["lat"] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/longitude":
                sim_state["lon"] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/elevation":
                sim_state["alt"] = float(fields[2])
            elif fields[1] == "sim/flightmodel/position/local_vx":
                sim_state["velocity"][1] = float(fields[2])
            elif fields[1] == "sim/flightmodel/position/local_vy":
                sim_state["velocity"][2] = -float(fields[2])
            elif fields[1] == "sim/flightmodel/position/local_vz":
                sim_state["velocity"][0] = -float(fields[2])
            elif fields[1] == "sim/flightmodel/position/psi":
                sim_ref["attitude_yaw"] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/theta":
                sim_ref["attitude_pitch"] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/phi":
                sim_ref["attitude_roll"] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/P":
                sim_state["angular_velocity"][0] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/Q":
                sim_state["angular_velocity"][1] = math.radians(float(fields[2]))
            elif fields[1] == "sim/flightmodel/position/R":
                sim_state["angular_velocity"][2] = math.radians(float(fields[2]))
            elif fields[1] == "sim/weather/wind_now_x_msc":
                sim_ref["wind_e"] = float(fields[2])
            elif fields[1] == "sim/weather/wind_now_y_msc":
                sim_ref["wind_d"] = -float(fields[2])
            elif fields[1] == "sim/weather/wind_now_z_msc":
                sim_ref["wind_n"] = -float(fields[2])
            elif fields[1] == "sim/flightmodel/position/true_airspeed":
                sim_ref["airspeed"] = float(fields[2])
    except socket.error:
        pass

    # Average out changes in wind over a period of time
    sim_state["wind_velocity"][0] += (sim_ref["wind_n"] - sim_state["wind_velocity"][0]) * 0.01
    sim_state["wind_velocity"][1] += (sim_ref["wind_e"] - sim_state["wind_velocity"][1]) * 0.01
    sim_state["wind_velocity"][2] += (sim_ref["wind_d"] - sim_state["wind_velocity"][2]) * 0.01

    # Recalculate quaternion in case euler angles have been updated.
    sim_state["attitude"] = plog.euler_to_q(
        sim_ref["attitude_yaw"], sim_ref["attitude_pitch"],
        sim_ref["attitude_roll"]
    )

    if sim_state["attitude"][3] < 0:
        sim_state["attitude"] = map(lambda x: -x, sim_state["attitude"])


def send_state_to_xplane(s):
    state = (c_float * 13)()
    fcs._fcs._get_next_reference_point(state, 0)

    velocity = state[3:6]
    attitude = state[6:10]
    angular_velocity = state[10:13]

    s.sendall("world-set %f %f %f\n" % (
        math.degrees(fcs.nav_state.reference_trajectory[0].lat),
        math.degrees(fcs.nav_state.reference_trajectory[0].lon),
        fcs.nav_state.reference_trajectory[0].alt))

    q = (attitude[3], -attitude[0], -attitude[1], -attitude[2])
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2.0 + q[3] ** 2.0))
    pitch = math.asin(2.0 * (q[0] * q[2] - q[3] * q[1]))
    roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] ** 2.0 + q[2] ** 2.0))

    update = "set sim/flightmodel/position/psi %.6f\n" % math.degrees(yaw)
    update += "set sim/flightmodel/position/theta %.6f\n" % math.degrees(pitch)
    update += "set sim/flightmodel/position/phi %.6f\n" % math.degrees(roll)

    s.sendall(update)


def send_control_to_xplane(s, controls):
    update = "set sim/flightmodel/engine/ENGN_thro_use [%.6f,0,0,0,0,0,0,0]\n" % (controls[0] - 0.15)
    update += "set sim/flightmodel/controls/wing1l_ail1def %.6f\n" % (math.degrees(controls[1] - 0.5) * 1.5)
    update += "set sim/flightmodel/controls/wing1r_ail1def %.6f\n" % (math.degrees(controls[2] - 0.5) * 1.5)
    s.sendall(update)


TURN_RADIUS = 60.0
WGS84_A = 6378137.0


def ll_pt_ned_offset(a, origin):
    return (
        (a[0] - origin[0]) * WGS84_A,
        (a[1] - origin[1]) * WGS84_A * math.cos(origin[0]),
        0.0
    )


def ned_pt_ll(a, origin):
    return (
        origin[0] + a[0] / WGS84_A,
        origin[1] + a[1] / (WGS84_A * math.cos(origin[0])),
        0.0
    )


def ll_pt_pt_distance(a, b):
    x = ll_pt_ned_offset(a, b)
    return math.sqrt(x[0]**2 + x[1]**2)


def ll_pt_line_distance(a, line):
    # Convert a and line vector to NED based on line[0] being the origin
    a_ned = ll_pt_ned_offset(a, line[0])
    l_ned = ll_pt_ned_offset(line[1], line[0])
    return abs(l_ned[1] * a_ned[0] - l_ned[0] * a_ned[1]) / math.sqrt(l_ned[0]**2 + l_ned[1]**2)


def ll_pt_line_direction(a, line):
    a_ned = ll_pt_ned_offset(a, line[0])
    l_ned = ll_pt_ned_offset(line[1], line[0])
    direction = l_ned[1] * a_ned[0] - l_ned[0] * a_ned[1]
    return 1.0 if direction > 0.0 else -1.0


PATTERN_WIDTH = 240.0 # 130.0 # 150.0


def generate_search_pattern(area):
    # Find the length and point indices of the longest side by checking each
    # in turn
    longest_side = None
    longest_side_length = 0.0
    for i in range(len(area)):
        side_length = ll_pt_pt_distance(area[i], area[(i + 1) % len(area)])
        if side_length > longest_side_length:
            longest_side = [i, (i + 1) % len(area)]
            longest_side_length = side_length

    # Find the distance of the furthest point from the longest side, which
    # will become the baseline
    furthest_point = None
    furthest_point_distance = 0.0
    for i in range(len(area)):
        point_distance = ll_pt_line_distance(area[i], (area[longest_side[0]],
                                                       area[longest_side[1]]))
        if point_distance > furthest_point_distance:
            furthest_point = i
            furthest_point_distance = point_distance

    # Convert area to NED relative to first point
    ned_area = [ll_pt_ned_offset(a, area[0]) for a in area]

    # Find the orientation of the search pattern (heading of initial path)
    heading = math.atan2(
        ned_area[longest_side[0]][1] - ned_area[longest_side[1]][1],
        ned_area[longest_side[0]][0] - ned_area[longest_side[1]][0]
    )

    # Determine the number of passes to be made (2 passes per cycle, so the
    # number must be even).
    pass_distance = PATTERN_WIDTH
    pass_direction = ll_pt_line_direction(
        area[furthest_point], (area[longest_side[0]], area[longest_side[1]]))

    n_passes = int(math.ceil(furthest_point_distance / pass_distance))

    pass_offset = (
        math.cos(heading + math.pi * 0.5 * pass_direction) * pass_distance,
        math.sin(heading + math.pi * 0.5 * pass_direction) * pass_distance
    )

    pattern_points = []

    # Generate points/paths for the first sweep, at 120m intervals
    for i in range(n_passes):
        if i % 2 == 1:  # odd, 0-1
            pattern_points += [
                (ned_area[longest_side[0]][0] + pass_offset[0] * (i),
                 ned_area[longest_side[0]][1] + pass_offset[1] * (i)),
                (ned_area[longest_side[1]][0] + pass_offset[0] * (i),
                 ned_area[longest_side[1]][1] + pass_offset[1] * (i))
            ]
        else:  # event, 1-0
            pattern_points += [
                (ned_area[longest_side[1]][0] + pass_offset[0] * (i),
                 ned_area[longest_side[1]][1] + pass_offset[1] * (i)),
                (ned_area[longest_side[0]][0] + pass_offset[0] * (i),
                 ned_area[longest_side[0]][1] + pass_offset[1] * (i))
            ]

    # Generate points/paths for the second sweep, at 120m intervals with a 60m offset
    for i in range(n_passes - 1, -1, -1):
        if i % 2 == 0:  # even, 0-1
            pattern_points += [
                (ned_area[longest_side[0]][0] + pass_offset[0] * (i + 0.5),
                 ned_area[longest_side[0]][1] + pass_offset[1] * (i + 0.5)),
                (ned_area[longest_side[1]][0] + pass_offset[0] * (i + 0.5),
                 ned_area[longest_side[1]][1] + pass_offset[1] * (i + 0.5))
            ]
        else:  # odd, 1-0
            pattern_points += [
                (ned_area[longest_side[1]][0] + pass_offset[0] * (i + 0.5),
                 ned_area[longest_side[1]][1] + pass_offset[1] * (i + 0.5)),
                (ned_area[longest_side[0]][0] + pass_offset[0] * (i + 0.5),
                 ned_area[longest_side[0]][1] + pass_offset[1] * (i + 0.5))
            ]

    pattern_points = [ned_pt_ll(a, area[0]) for a in pattern_points]

    return pattern_points, heading


MB = [
    (math.radians(-26.569564), math.radians(151.837373), 933.0),
    (math.radians(-26.569956), math.radians(151.839405), 933.0),
    (math.radians(-26.576823), math.radians(151.841142), 933.0),
    (math.radians(-26.576990), math.radians(151.840045), 933.0),
    (math.radians(-26.581227), math.radians(151.841062), 933.0),
    (math.radians(-26.582060), math.radians(151.842503), 933.0),
    (math.radians(-26.579179), math.radians(151.849329), 933.0),
    (math.radians(-26.590009), math.radians(151.850974), 933.0),
    (math.radians(-26.609574), math.radians(151.863100), 933.0),
    (math.radians(-26.638958), math.radians(151.858839), 933.0),
    (math.radians(-26.644972), math.radians(151.847224), 933.0),
    (math.radians(-26.643572), math.radians(151.830350), 933.0)
]
SA = [
    (math.radians(-26.617445), math.radians(151.843184), 573.0),
    (math.radians(-26.619171), math.radians(151.855573), 573.0),
    (math.radians(-26.637843), math.radians(151.851597), 573.0),
    (math.radians(-26.638828), math.radians(151.849121), 573.0),
    (math.radians(-26.637393), math.radians(151.839386), 573.0)
]
EL = [
    (math.radians(-26.607212), math.radians(151.845389), 573.0),
    (math.radians(-26.617498), math.radians(151.843567), 573.0),
    (math.radians(-26.618365), math.radians(151.849788), 573.0),
    (math.radians(-26.608092), math.radians(151.851646), 573.0)
]
HOME = (math.radians(-26.584781), math.radians(151.842325), 460.0)
HOLD = (math.radians(-26.607212), math.radians(151.845389))


if __name__ == "__main__":
    fcs.init(sys.argv[1])
    if len(sys.argv) > 2:
        max_t = int(sys.argv[2])
    else:
        max_t = -1

    # Curve from HOME to EL01
    fcs.nav_state.waypoints[0].lat = HOME[0]
    fcs.nav_state.waypoints[0].lon = HOME[1]
    fcs.nav_state.waypoints[0].alt = HOME[2]
    fcs.nav_state.waypoints[0].airspeed = 22.0
    fcs.nav_state.waypoints[0].yaw = math.radians(180.0)
    fcs.nav_state.waypoints[0].pitch = 0.0
    fcs.nav_state.waypoints[0].roll = 0.0

    fcs.nav_state.waypoints[1].lat = EL[0][0]
    fcs.nav_state.waypoints[1].lon = EL[0][1]
    fcs.nav_state.waypoints[1].alt = EL[0][2]
    fcs.nav_state.waypoints[1].airspeed = 22.0
    fcs.nav_state.waypoints[1].yaw = math.radians(180.0)
    fcs.nav_state.waypoints[1].pitch = 0.0
    fcs.nav_state.waypoints[1].roll = 0.0

    fcs.nav_state.paths[0].start_waypoint_id = 0
    fcs.nav_state.paths[0].end_waypoint_id = 1
    fcs.nav_state.paths[0].type = FCS_PATH_LINE
    fcs.nav_state.paths[0].next_path_id = 1

    # Another curve to EL02
    fcs.nav_state.waypoints[2].lat = EL[1][0]
    fcs.nav_state.waypoints[2].lon = EL[1][1]
    fcs.nav_state.waypoints[2].alt = EL[1][2]
    fcs.nav_state.waypoints[2].airspeed = 22.0
    fcs.nav_state.waypoints[2].yaw = math.radians(180.0)
    fcs.nav_state.waypoints[2].pitch = 0.0
    fcs.nav_state.waypoints[2].roll = 0.0

    fcs.nav_state.paths[1].start_waypoint_id = 1
    fcs.nav_state.paths[1].end_waypoint_id = 2
    fcs.nav_state.paths[1].type = FCS_PATH_DUBINS_CURVE
    fcs.nav_state.paths[1].next_path_id = 2

    fcs.nav_state.paths[2].start_waypoint_id = 2
    fcs.nav_state.paths[2].end_waypoint_id = 3
    fcs.nav_state.paths[2].type = FCS_PATH_DUBINS_CURVE
    fcs.nav_state.paths[2].next_path_id = 3

    pattern_points, heading = generate_search_pattern(SA)
    for i in range(len(pattern_points)):
        fcs.nav_state.waypoints[3 + i].lat = pattern_points[i][0]
        fcs.nav_state.waypoints[3 + i].lon = pattern_points[i][1]
        fcs.nav_state.waypoints[3 + i].alt = SA[0][2]
        fcs.nav_state.waypoints[3 + i].airspeed = 22.0
        if int(i / 2) % 2 == 0:
            fcs.nav_state.waypoints[3 + i].yaw = heading
        else:
            fcs.nav_state.waypoints[3 + i].yaw = heading + math.pi
        fcs.nav_state.waypoints[3 + i].pitch = 0.0
        fcs.nav_state.waypoints[3 + i].roll = 0.0

        #print "[%.9f, %.9f]," % (math.degrees(nav_state.waypoints[3 + i].lon), math.degrees(nav_state.waypoints[3 + i].lat))

        if i % 2 == 0:
            fcs.nav_state.paths[3 + i].start_waypoint_id = 3 + i
            fcs.nav_state.paths[3 + i].end_waypoint_id = 3 + i + 1
            fcs.nav_state.paths[3 + i].type = FCS_PATH_LINE
            fcs.nav_state.paths[3 + i].next_path_id = 3 + i + 1
        else:
            fcs.nav_state.paths[3 + i].start_waypoint_id = 3 + i
            fcs.nav_state.paths[3 + i].end_waypoint_id = 3 + i + 1
            fcs.nav_state.paths[3 + i].type = FCS_PATH_DUBINS_CURVE
            fcs.nav_state.paths[3 + i].next_path_id = 3 + i + 1

    # Exit pattern to EL03
    fcs.nav_state.waypoints[3 + len(pattern_points)].lat = EL[2][0]
    fcs.nav_state.waypoints[3 + len(pattern_points)].lon = EL[2][1]
    fcs.nav_state.waypoints[3 + len(pattern_points)].alt = EL[2][2]
    fcs.nav_state.waypoints[3 + len(pattern_points)].airspeed = 22.0
    fcs.nav_state.waypoints[3 + len(pattern_points)].yaw = 0.0
    fcs.nav_state.waypoints[3 + len(pattern_points)].pitch = 0.0
    fcs.nav_state.waypoints[3 + len(pattern_points)].roll = 0.0

    # Curve from EL03 to EL04
    fcs.nav_state.waypoints[4 + len(pattern_points)].lat = EL[3][0]
    fcs.nav_state.waypoints[4 + len(pattern_points)].lon = EL[3][1]
    fcs.nav_state.waypoints[4 + len(pattern_points)].alt = EL[3][2]
    fcs.nav_state.waypoints[4 + len(pattern_points)].airspeed = 22.0
    fcs.nav_state.waypoints[4 + len(pattern_points)].yaw = 0.0
    fcs.nav_state.waypoints[4 + len(pattern_points)].pitch = 0.0
    fcs.nav_state.waypoints[4 + len(pattern_points)].roll = 0.0

    fcs.nav_state.paths[3 + len(pattern_points)].start_waypoint_id = 3 + len(pattern_points)
    fcs.nav_state.paths[3 + len(pattern_points)].end_waypoint_id = 4 + len(pattern_points)
    fcs.nav_state.paths[3 + len(pattern_points)].type = FCS_PATH_DUBINS_CURVE
    fcs.nav_state.paths[3 + len(pattern_points)].next_path_id = 4 + len(pattern_points)

    # Curve to home and hold
    fcs.nav_state.paths[4 + len(pattern_points)].start_waypoint_id = 4 + len(pattern_points)
    fcs.nav_state.paths[4 + len(pattern_points)].end_waypoint_id = 0
    fcs.nav_state.paths[4 + len(pattern_points)].type = FCS_PATH_DUBINS_CURVE
    fcs.nav_state.paths[4 + len(pattern_points)].next_path_id = 0xFFFF

    # Register the path with the FCS
    fcs.nav_state.reference_path_id[0] = 0
    fcs._fcs.fcs_control_reset()

    sock = connect_to_xplane()
    reset_xplane_state(sock)

    time.sleep(1.0)

    enable_xplane_sim(sock)

    time.sleep(0.1)

    print "t,target_lat,target_lon,target_alt,target_airspeed,target_yaw,actual_lat,actual_lon,actual_alt,actual_airspeed,actual_yaw,wind_n,wind_e,wind_d,ctl_t,ctl_l,ctl_r"

    controls = [0.0, 0.5, 0.5]
    t = 0
    dist_air = dist_gnd = 0.0
    try:
        while True:
            iter_start = time.time()

            recv_state_from_xplane(sock)

            sim_state_delay.append(sim_state)

            # Skip the rest until we have a full set of data
            if len(sim_state_delay) < 10:
                time.sleep(0.02)
                continue

            sim_state_delay.popleft()

            controls = tick(
                lat=sim_state_delay[0]["lat"], lon=sim_state_delay[0]["lon"],
                alt=sim_state_delay[-1]["alt"], velocity=sim_state_delay[0]["velocity"],
                attitude=sim_state_delay[-2]["attitude"],
                angular_velocity=sim_state_delay[-1]["angular_velocity"],
                wind_velocity=sim_state_delay[0]["wind_velocity"])

            print "%.2f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % (
                t * 0.02,
                math.degrees(fcs.nav_state.reference_trajectory[0].lat),
                math.degrees(fcs.nav_state.reference_trajectory[0].lon),
                fcs.nav_state.reference_trajectory[0].alt,
                fcs.nav_state.reference_trajectory[0].airspeed,
                math.degrees(fcs.nav_state.reference_trajectory[0].yaw),
                math.degrees(fcs.nav_state.reference_trajectory[0].pitch),
                math.degrees(fcs.nav_state.reference_trajectory[0].roll),
                math.degrees(sim_state["lat"]),
                math.degrees(sim_state["lon"]),
                sim_state["alt"],
                sim_ref["airspeed"],
                math.degrees(sim_ref["attitude_yaw"]),
                math.degrees(sim_ref["attitude_pitch"]),
                math.degrees(sim_ref["attitude_roll"]),
                sim_ref["wind_n"],
                sim_ref["wind_e"],
                sim_ref["wind_d"],
                controls[0],
                controls[1],
                controls[2]
            )

            send_control_to_xplane(sock, controls)

            t += 1
            dist_air += sim_ref["airspeed"] * 0.02
            dist_gnd += math.sqrt(
                sim_state["velocity"][0]**2 + sim_state["velocity"][1]**2 +
                sim_state["velocity"][2]**2) * 0.02

            if fcs.nav_state.reference_path_id[0] == 499:
                print "COMPLETED"
                print "Time taken: %.0f min %.0f sec" % (
                    int(t * 0.02) / 60, (t * 0.02) % 60.0)
                print "Air distance: %.1f km" % (dist_air * 0.001)
                print "Ground distance: %.1f km" % (dist_gnd * 0.001)
                raise StopIteration()

            #if t % 500 == 499:
            #    nav_state.reference_path_id[0] = 0xFFFF

            if sim_state["alt"] < 50.0:
                print "LOST CONTROL"
                print "Reference trajectory was:"
                print "\n".join(("    " + repr(w)) for w in list(fcs.nav_state.reference_trajectory))
                raise StopIteration()

            slack_time = 0.02 - (time.time() - iter_start)
            if slack_time > 0:
                time.sleep(slack_time)
            else:
                print "DEADLINE MISSED: %.3fs late" % -slack_time
    finally:
        disable_xplane_sim(sock)
