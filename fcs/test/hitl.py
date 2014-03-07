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

import os
import sys
import copy
import math
import time
import crcmod
import socket
import serial
import struct
import vectors
import datetime
import binascii
import traceback
from cobs import cobsr


"""
Driver for HITL testing of the FCS navigation and control algorithms in an
X-Plane simulation, with the DSP connected via a USB-serial adapter
"""

FCS_MODE_INITIALIZING = ord("I")
FCS_MODE_CALIBRATING = ord("C")
FCS_MODE_SAFE = ord("S")
FCS_MODE_ARMED = ord("R")
FCS_MODE_ACTIVE = ord("A")
FCS_MODE_HOLDING = ord("H")
FCS_MODE_ABORT = ord("F")


FCS_PATH_LINE = ord("L")
FCS_PATH_DUBINS_CURVE = ord("D")
FCS_PATH_FIGURE_EIGHT = ord("8")
FCS_PATH_INVALID = 0


FCS_CONTROL_INVALID_PATH_ID = 0xFFFF
FCS_CONTROL_HOLD_PATH_ID = 500 - 1
FCS_CONTROL_INTERPOLATE_PATH_ID = 500 - 2
FCS_CONTROL_RESUME_PATH_ID = 500 - 3
FCS_CONTROL_HOLD_WAYPOINT_ID = 1000 - 1
FCS_CONTROL_INTERPOLATE_WAYPOINT_ID = 1000 - 2
FCS_CONTROL_RESUME_WAYPOINT_ID = 1000 - 3


ahrs_state = None
control_state = None
nav_state = None
ahrs_tick = 0


START_LAT = -37.81358378
START_LON = 144.9
START_ALT = 200


sim_state = {
    "lat": None,
    "lon": None,
    "alt": 0.0,
    "velocity": [0.0, 0.0, 0.0],
    "attitude": [0.0, 0.0, 0.0, 1.0],
    "angular_velocity": [0.0, 0.0, 0.0],
    "wind_velocity": [0.0, 0.0, 0.0]
}

sim_ref = {
    "wind_n": None,
    "wind_e": 0.0,
    "wind_d": 0.0,
    "attitude_yaw": 0.0,
    "attitude_pitch": 0.0,
    "attitude_roll": 0.0,
    "airspeed": 0.0
}


CRC8_FUNC = crcmod.mkCrcFun(0x12F, 0, False, 0)

def crc8(data):
    return chr(CRC8_FUNC(data, crc=0))


def serialize_state(lat=None, lon=None, alt=None, velocity=None,
                    attitude=None, angular_velocity=None, wind_velocity=None):
    tick = 0
    result = struct.pack("<H3d3d4d3d3d", tick, lat, lon, alt, velocity[0],
                         velocity[1], velocity[2], attitude[0], attitude[1],
                         attitude[2], attitude[3], angular_velocity[0],
                         angular_velocity[1], angular_velocity[2],
                         wind_velocity[0], wind_velocity[1], wind_velocity[2])
    return "\x00" + cobsr.encode(crc8(result) + result) + "\x00"


def deserialize_control(s):
    s = cobsr.decode(s.strip("\x00"))
    s_crc = crc8(s[1:])
    if s[0] != s_crc:
        print "CRC8 failure (%x, expected %x): %s" % (
            s[0], s_crc, binascii.b2a_hex(s))
        return None
    else:
        ctl = struct.unpack("<H4HfL2dff3f", s[1:])
        return dict(zip(["tick", "pwm0", "pwm1", "pwm2", "pwm3",
                         "objective_val", "cycles", "reference_lat",
                         "reference_lon", "reference_alt",
                         "reference_airspeed", "reference_yaw",
                         "reference_pitch", "reference_roll"], ctl))


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


def send_state_to_dsp(conn, **kwargs):
    """
    Runs the FCS control and comms tasks with the state data provided as
    though it came from the AHRS, and returns the control output.
    """
    # Write the state out to the DSP and wait for the result
    out_packet = serialize_state(**kwargs)
    conn.write(out_packet)


def recv_control_from_dsp(conn):
    in_packet = ""
    while not in_packet:
        in_packet = conn.read(60)

    return deserialize_control(in_packet)


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
        velocity = [20.0, 0.0, 0.0]

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

    yaw = math.radians(0.0)
    pitch = math.radians(0.0)
    roll = math.radians(0.0)
    velocity = [20.0, 0.0, 0.0]

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
    sim_state["attitude"] = euler_to_q(
        sim_ref["attitude_yaw"], sim_ref["attitude_pitch"],
        sim_ref["attitude_roll"]
    )

    if sim_state["attitude"][3] < 0:
        sim_state["attitude"] = map(lambda x: -x, sim_state["attitude"])


def send_control_to_xplane(s, controls):
    update = "set sim/flightmodel/engine/ENGN_thro_use [%.6f,0,0,0,0,0,0,0]\n" % controls[0]
    update += "set sim/flightmodel/controls/wing1l_ail1def %.6f\n" % math.degrees(controls[1] - 0.5)
    update += "set sim/flightmodel/controls/wing1r_ail1def %.6f\n" % math.degrees(controls[2] - 0.5)
    s.sendall(update)


if __name__ == "__main__":
    conn = serial.Serial("/dev/tty.usbserial", 921600, timeout=0.01)
    conn.read(1024)  # Clear out the read buffer

    sock = connect_to_xplane()
    reset_xplane_state(sock)

    time.sleep(1.0)

    enable_xplane_sim(sock)

    time.sleep(0.1)

    print "t,target_lat,target_lon,target_alt,target_airspeed,target_yaw,target_pitch,target_roll,actual_lat,actual_lon,actual_alt,actual_airspeed,actual_yaw,actual_pitch,actual_roll,wind_n,wind_e,wind_d,ctl_t,ctl_l,ctl_r"

    t = 0
    try:
        while True:
            iter_start = time.time()

            if t > 0:
                result = recv_control_from_dsp(conn)
                thr = result["pwm0"] / 65535.0
                le = result["pwm1"] / 65535.0
                re = result["pwm2"] / 65535.0
                send_control_to_xplane(
                    sock, [thr, le, re])

                print "Objective %.6f, cycles %d" % (result["objective_val"], result["cycles"])

                print "%.2f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % (
                    t * 0.02,
                    math.degrees(result["reference_lat"]),
                    math.degrees(result["reference_lon"]),
                    result["reference_alt"],
                    result["reference_airspeed"],
                    math.degrees(result["reference_yaw"]),
                    math.degrees(result["reference_pitch"]),
                    math.degrees(result["reference_roll"]),
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
                    result["pwm0"],
                    result["pwm1"],
                    result["pwm2"]
                )

            recv_state_from_xplane(sock)

            # Skip the rest until we have a full set of data
            if sim_state["lat"] is None or sim_state["lon"] is None or \
                    sim_ref["wind_n"] is None:
                time.sleep(0.02)
                continue

            send_state_to_dsp(conn, **sim_state)

            t += 1

            if abs(sim_state["alt"]) < 50.0:
                print "LOST CONTROL"
                raise StopIteration()

            slack_time = 0.02 - (time.time() - iter_start)
            if slack_time > 0:
                print "Time %.3fs" % (time.time() - iter_start)
                time.sleep(slack_time)
            else:
                print "DEADLINE MISSED: %.3fs late" % -slack_time
    finally:
        disable_xplane_sim(sock)
