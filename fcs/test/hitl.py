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
import sys
import copy
import math
import time
import plog
import crcmod
import socket
import serial
import struct
import vectors
import collections
import datetime
import binascii
import traceback
import random
from cobs import cobsr


"""
Driver for HITL testing of the FCS navigation and control algorithms in an
X-Plane simulation, with the DSP connected via a USB-serial adapter
"""

ahrs_tick = 0


START_LAT = -37.81358378
START_LON = 144.9
START_ALT = 200


WGS84_A = 6378137.0


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


def euler_to_q(yaw, pitch, roll):
    return (vectors.Q.rotate("X", -roll) *
            vectors.Q.rotate("Y", -pitch) *
            vectors.Q.rotate("Z", -yaw))


def lla_to_ned(a, origin):
    return (
        (a[0] - origin[0]) * WGS84_A,
        (a[1] - origin[1]) * WGS84_A * math.cos(origin[0]),
        a[2] - origin[2]
    )


def write_state(conn, lat=None, lon=None, alt=None, velocity=None,
                attitude=None, angular_velocity=None, wind_velocity=None):
    """
    Runs the FCS control and comms tasks with the state data provided as
    though it came from the AHRS, and returns the control output.
    """
    # Write the state out to the DSP and wait for the result
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

    state_out = estimate_log.serialize()
    conn.write(state_out)
    #print binascii.b2a_hex(state_out)

PACKET = ""
PACKETS = []
LAST_CONTROL = None

def read_control(conn):
    global PACKET, PACKETS, LAST_CONTROL

    data = conn.read()
    got_message = False
    while data:
        for ch in data:
            if ch == "\x00" or PACKET:
                if ch == "\x00" and PACKET:
                    if PACKET[-1] == "\x00":
                        PACKET = PACKET[:-1]
                    else:
                        got_message = True
                PACKET = PACKET + ch
                if len(PACKET) > 1024:
                    PACKET = ""
            if got_message:
                PACKETS.append(PACKET)
                PACKET = ""
                got_message = False

        data = conn.read()

    last_packet = PACKETS[-1] if PACKETS else None
    PACKETS = []

    try:
        control_log = plog.ParameterLog.deserialize(last_packet)

        # print control_log

        control_param = control_log.find_by(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_CONTROL_SETPOINT)
        path, err_type = control_log.find_by(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_NAV_PATH_ID).values
        refp = control_log.find_by(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_KEY_VALUE)
        cycles, obj_val, errors, resets = control_log.find_by(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_CONTROL_STATUS).values

        LAST_CONTROL = (
            map(lambda x: float(x) / float(2**16), control_param.values),
            plog.extract_waypoint(refp.value),
            obj_val, cycles, errors, resets, control_log.tick, path, err_type
        )

        return LAST_CONTROL
    except Exception:
        return LAST_CONTROL or ([0.0, 0.5, 0.5], {}, 0, 0, 0, 0, 0, 0xFFFF, 0x0)


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


def reset_xplane_state(s, yaw=0.0, pitch=0.0, roll=0.0, velocity=None):
    disable_xplane_sim(s)

    s.sendall("world-set %f %f %f\n" % (-37.8136, START_LON, START_ALT))
    time.sleep(1.0)

    # Clear the engine fire -- this happens whenever the X8 is on the ground
    s.sendall("sub sim/operation/failures/rel_engfir0")
    s.sendall("set sim/operation/failures/rel_engfir0 0")
    s.sendall("unsub sim/operation/failures/rel_engfir0")

    if not velocity:
        velocity = [15.52, 0.51, 0.74]

    # Set the simulation's initial state
    update = ""

    yaw = math.radians(0.0)
    pitch = math.radians(13.4)
    roll = math.radians(1.1)

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

    yaw = math.radians(0.0)
    pitch = math.radians(13.4)
    roll = math.radians(1.1)
    velocity = [15.52, 0.51, 0.74]

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

    update += "set sim/flightmodel/position/P %.6f\n" % math.radians(-6.7)
    update += "set sim/flightmodel/position/Q %.6f\n" % math.radians(-5.0)
    update += "set sim/flightmodel/position/R %.6f\n" % math.radians(14.4)
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
                sim_state["alt"] = float(fields[2]) + random.uniform(-1.0, 1.0)
            elif fields[1] == "sim/flightmodel/position/local_vx":
                sim_state["velocity"][1] = float(fields[2]) + random.uniform(-2.0, 2.0)
            elif fields[1] == "sim/flightmodel/position/local_vy":
                sim_state["velocity"][2] = -float(fields[2]) + random.uniform(-2.0, 2.0)
            elif fields[1] == "sim/flightmodel/position/local_vz":
                sim_state["velocity"][0] = -float(fields[2]) + random.uniform(-2.0, 2.0)
            elif fields[1] == "sim/flightmodel/position/psi":
                sim_ref["attitude_yaw"] = math.radians(float(fields[2])) + random.uniform(-0.1, 0.1)
            elif fields[1] == "sim/flightmodel/position/theta":
                sim_ref["attitude_pitch"] = math.radians(float(fields[2])) + random.uniform(-0.1, 0.1)
            elif fields[1] == "sim/flightmodel/position/phi":
                sim_ref["attitude_roll"] = math.radians(float(fields[2])) + random.uniform(-0.1, 0.1)
            elif fields[1] == "sim/flightmodel/position/P":
                sim_state["angular_velocity"][0] = math.radians(float(fields[2])) + random.uniform(-0.2, 0.2)
            elif fields[1] == "sim/flightmodel/position/Q":
                sim_state["angular_velocity"][1] = math.radians(float(fields[2])) + random.uniform(-0.2, 0.2)
            elif fields[1] == "sim/flightmodel/position/R":
                sim_state["angular_velocity"][2] = math.radians(float(fields[2])) + random.uniform(-0.2, 0.2)
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
    sim_state["attitude"] = euler_to_q(
        sim_ref["attitude_yaw"], sim_ref["attitude_pitch"],
        sim_ref["attitude_roll"]
    )

    if sim_state["attitude"][3] < 0:
        sim_state["attitude"] = map(lambda x: -x, sim_state["attitude"])


def send_control_to_xplane(s, controls):
    update = "set sim/flightmodel/engine/ENGN_thro_use [%.6f,0,0,0,0,0,0,0]\n" % controls[0]
    update += "set sim/flightmodel/controls/wing1l_ail1def %.6f\n" % (math.degrees(controls[1] - 0.5) * 1.5)
    update += "set sim/flightmodel/controls/wing1r_ail1def %.6f\n" % (math.degrees(controls[2] - 0.5) * 1.5)
    s.sendall(update)


if __name__ == "__main__":
    conn = serial.Serial("/dev/tty.usbserial-FTXP1CI7", 921600, timeout=0)
    conn.read(1024)  # Clear out the read buffer

    sock = connect_to_xplane()
    reset_xplane_state(sock)

    time.sleep(1.0)

    enable_xplane_sim(sock)

    time.sleep(0.1)

    controls = [0.0, 0.5, 0.5]
    t = time.time()
    last_print_tick = 0
    try:
        while True:
            recv_state_from_xplane(sock)

            sim_state_delay.append(sim_state)

            # Skip the rest until we have a full set of data
            if len(sim_state_delay) < 10:
                time.sleep(0.02)
                continue

            sim_state_delay.popleft()

            write_state(
                conn,
                # lat=sim_state_delay[0]["lat"],
                # lon=sim_state_delay[0]["lon"],
                # alt=sim_state_delay[-1]["alt"],
                # velocity=sim_state_delay[0]["velocity"],
                # attitude=sim_state_delay[-2]["attitude"],
                # angular_velocity=sim_state_delay[-1]["angular_velocity"],
                # wind_velocity=sim_state_delay[0]["wind_velocity"])
                lat=sim_state_delay[-1]["lat"],
                lon=sim_state_delay[-1]["lon"],
                alt=sim_state_delay[-1]["alt"],
                velocity=sim_state_delay[-1]["velocity"],
                attitude=sim_state_delay[-1]["attitude"],
                angular_velocity=sim_state_delay[-1]["angular_velocity"],
                wind_velocity=sim_state_delay[-1]["wind_velocity"]
            )

            controls, ref_point, obj_val, cycles, nmpc_errors, nmpc_resets, \
                tick, path, control_error_type = read_control(conn)

            send_control_to_xplane(sock, controls)

            #print "%.2f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % (
            #    t * 0.02,
            #    math.degrees(ref_point.get("lat", 0)),
            #    math.degrees(ref_point.get("lon", 0)),
            #    ref_point.get("alt", 0),
            #    ref_point.get("airspeed", 0),
            #    math.degrees(ref_point.get("yaw", 0)),
            #    math.degrees(ref_point.get("pitch", 0)),
            #    math.degrees(ref_point.get("roll", 0)),
            #    math.degrees(sim_state["lat"]),
            #    math.degrees(sim_state["lon"]),
            #    sim_state["alt"],
            #    sim_ref["airspeed"],
            #    #sim_state["velocity"][0],
            #    #sim_state["velocity"][1],
            #    #sim_state["velocity"][2],
            #    math.degrees(sim_ref["attitude_yaw"]),
            #    math.degrees(sim_ref["attitude_pitch"]),
            #    math.degrees(sim_ref["attitude_roll"]),
            #    sim_ref["wind_n"],
            #    sim_ref["wind_e"],
            #    sim_ref["wind_d"],
            #    controls[0],
            #    controls[1],
            #    controls[2]
            #)

            if tick != last_print_tick:
                print (
                    "t=%6.2f, " +
                    "n=%6.2f, e=%6.2f, d=%6.2f, " +
                    "tas=%5.2f, tas_ref=%5.2f, " +
                    "yaw=%3.0f, yaw_ref=%3.0f, " +
                    "pitch=%4.0f, pitch_ref=%4.0f, " +
                    "roll=%4.0f, roll_ref=%4.0f, " +
                    "vyaw=%4.0f, vpitch=%4.0f, vroll=%4.0f, " +
                    "t=%.3f, l=%.3f, r=%.3f, " +
                    "objval=%10.1f, cycles=%9d, errors=%9d, resets=%9d, " +
                    "path=%4d, err=%04x"
                ) % (
                    (time.time() - t, ) +
                    lla_to_ned((ref_point.get("lat", 0), ref_point.get("lon", 0),
                               ref_point.get("alt", 0)), (sim_state["lat"],
                               sim_state["lon"], sim_state["alt"])) +
                    (sim_ref["airspeed"], ref_point.get("airspeed", 0)) +
                    (math.degrees(sim_ref["attitude_yaw"]),
                        math.degrees(ref_point.get("yaw", 0))) +
                    (math.degrees(sim_ref["attitude_pitch"]),
                        math.degrees(ref_point.get("pitch", 0))) +
                    (math.degrees(sim_ref["attitude_roll"]),
                        math.degrees(ref_point.get("roll", 0))) +
                    (math.degrees(sim_state["angular_velocity"][2]),
                        math.degrees(sim_state["angular_velocity"][1]),
                        math.degrees(sim_state["angular_velocity"][0])) +
                    (controls[0], controls[1], controls[2]) +
                    (obj_val, cycles, nmpc_errors, nmpc_resets) +
                    (path, control_error_type)
                )
                last_print_tick = tick

            if sim_state["alt"] < 50.0:
                print "LOST CONTROL"
                raise StopIteration()
    finally:
        disable_xplane_sim(sock)
