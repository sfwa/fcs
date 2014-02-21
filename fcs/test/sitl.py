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
import socket
import datetime
from ctypes import *


"""
Driver for SITL testing of the FCS navigation and control algorithms in an
X-Plane simulation.

Does not currently simulate FCS comms or AHRS functionality.
"""


_fcs = None


FCS_STREAM_UART_INT0 = 0
FCS_STREAM_UART_INT1 = 1
FCS_STREAM_UART_EXT0 = 2
FCS_STREAM_UART_EXT1 = 3
FCS_STREAM_USB = 4
FCS_STREAM_NUM_DEVICES = 5


FCS_STREAM_OK = 0
FCS_STREAM_ERROR = 1


FCS_MODE_INITIALIZING = ord("I")
FCS_MODE_CALIBRATING = ord("C")
FCS_MODE_SAFE = ord("S")
FCS_MODE_ARMED = ord("R")
FCS_MODE_ACTIVE = ord("A")
FCS_MODE_HOLDING = ord("H")
FCS_MODE_ABORT = ord("F")


FCS_PATH_LINE = 0
FCS_PATH_DUBINS_CURVE = 1
FCS_PATH_FIGURE_EIGHT = 2
FCS_PATH_INVALID = 3


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


sim_state = {
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0.0,
    "velocity": [0.0, 0.0, 0.0],
    "attitude": [0.0, 0.0, 0.0, 1.0],
    "angular_velocity": [0.0, 0.0, 0.0],
    "wind_velocity": [0.0, 0.0, 0.0]
}

sim_ref = {
    "wind_n": 0.0,
    "wind_e": 0.0,
    "wind_d": 0.0,
    "attitude_yaw": 0.0,
    "attitude_pitch": 0.0,
    "attitude_roll": 0.0
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
    q = [0, 0, 0, 0]

    sz = math.sin(yaw * 0.5)
    sy = math.sin(pitch * 0.5)
    sx = math.sin(roll * 0.5)
    cz = math.cos(yaw * 0.5)
    cy = math.cos(pitch * 0.5)
    dx = math.cos(roll * 0.5)

    q[3] = cx * cy * cz + sx * sy * sz
    q[0] = sx * cy * cz - cx * sy * sz
    q[1] = cx * sy * cz + sx * cy * sz
    q[2] = cx * cy * sz - sx * sy * cz

    return q


class TRICALInstance(Structure):
    _fields_ = [
        ("field_norm", c_float),
        ("measurement_noise", c_float),
        ("state", c_float * 12),
        ("state_covariance", c_float * 12 * 12),
        ("measurement_count", c_uint)
    ]


class AHRSCalibrationEntry(Structure):
    _fields_ = [
        ("header", c_ubyte),
        ("sensor", c_ubyte),
        ("type", c_ubyte),
        ("reserved", c_ubyte),
        ("error", c_float),
        ("params", c_float * 12),
        ("orientation", c_float * 4),
        ("offset", c_float * 3),
        ("scale_factor", c_float)
    ]


class AHRSCalibrationMap(Structure):
    _fields_ = [
        ("sensor_calibration", AHRSCalibrationEntry * 128)
    ]


class AHRSMeasurementLog(Structure):
    _fields_ = [
        ("data", c_ubyte * 256),
        ("length", c_uint)
    ]


class AHRSState(Structure):
    _fields_ = [
        ("solution_time", c_ulonglong),
        ("measurements", AHRSMeasurementLog),

        # UKF state + control input
        ("lat", c_double),
        ("lon", c_double),
        ("alt", c_double),
        ("velocity", c_double * 3),
        ("acceleration", c_double * 3),
        ("attitude", c_double * 4),
        ("angular_velocity", c_double * 3),
        ("angular_acceleration", c_double * 3),
        ("wind_velocity", c_double * 3),
        ("gyro_bias", c_double * 3),
        ("control_pos", c_double * 4),

        # UKF error estimate
        ("lat_error", c_double),
        ("lon_error", c_double),
        ("alt_error", c_double),
        ("velocity_error", c_double * 3),
        ("acceleration_error", c_double * 3),
        ("attitude_error", c_double * 3),
        ("angular_velocity_error", c_double * 3),
        ("angular_acceleration_error", c_double * 3),
        ("wind_velocity_error", c_double * 3),
        ("gyro_bias_error", c_double * 3),

        # Configuration
        ("wmm_field_dir", c_double * 3),
        ("wmm_field_norm", c_double),
        ("ukf_process_noise", c_double * 24),
        ("ukf_dynamics_model", c_uint),
        ("calibration", AHRSCalibrationMap),
        ("trical_instances", TRICALInstance * 4),
        ("trical_update_attitude", c_double * 4 * 4),

        # Aerodynamics
        ("reference_alt", c_double),
        ("reference_pressure", c_double),
        ("aero_static_pressure", c_double),
        ("aero_static_temp", c_double),
        ("aero_dynamic_pressure", c_double),

        # Sensor health
        ("last_accelerometer_time", c_ulonglong),
        ("last_gyroscope_time", c_ulonglong),
        ("last_magnetometer_time", c_ulonglong),
        ("last_barometer_time", c_ulonglong),
        ("last_pitot_time", c_ulonglong),
        ("last_gps_time", c_ulonglong),
        ("gps_num_svs", c_uint * 2),
        ("gps_pdop", c_double * 2),

        # Mode
        ("mode_start_time", c_ulonglong),
        ("mode", c_uint),

        # Payload
        ("payload_present", c_ubyte)
    ]


class ControlChannel(Structure):
    _fields_ = [
        ("setpoint", c_float),
        ("min", c_float),
        ("max", c_float),
        ("rate", c_float)
    ]


class ControlState(Structure):
    _fields_ = [
        ("controls", ControlChannel * 4),
        ("gpio_state", c_ubyte)
    ]


class NavPath(Structure):
    _fields_ = [
        ("start_waypoint_id", c_ushort),
        ("end_waypoint_id", c_ushort),
        ("type", c_uint),
        ("flags", c_ushort),
        ("next_path_id", c_ushort)
    ]


class NavWaypoint(Structure):
    _fields_ = [
        ("lat", c_double),
        ("lon", c_double),
        ("alt", c_float),
        ("airspeed", c_float),
        ("yaw", c_float),
        ("pitch", c_float),
        ("roll", c_float),
        ("flags", c_uint)
    ]

    def __repr__(self):
        return ("Waypoint(lat=%12.9f, lon=%12.9f, alt=%6.2f, airspeed=%4.1f, "
                + "yaw=%5.2f, pitch=%5.2f, roll=%5.2f, flags=%x)") % (
                self.lat, self.lon, self.alt, self.airspeed,
                math.degrees(self.yaw), math.degrees(self.pitch),
                math.degrees(self.roll), self.flags)


class NavBoundary(Structure):
    _fields_ = [
        ("num_waypoint_ids", c_ushort),
        ("waypoint_ids", c_ushort * 64),
        ("flags", c_ubyte)
    ]


class NavState(Structure):
    _fields_ = [
        ("paths", NavPath * 500),
        ("waypoints", NavWaypoint * 1000),
        ("boundary", NavBoundary),
        ("reference_trajectory", NavWaypoint * 101),
        ("reference_path_id", c_ushort * 101)
    ]


def reset():
    """
    (Re-)initializes all FCS modules.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")

    _fcs.fcs_board_init_platform()
    _fcs.fcs_util_init()
    _fcs.fcs_comms_init()
    _fcs.fcs_ahrs_init()
    _fcs.fcs_control_init()

    ahrs_state.mode = FCS_MODE_ACTIVE


def tick(lat=None, lon=None, alt=None, velocity=None, attitude=None,
         angular_velocity=None, wind_velocity=None):
    """
    Runs the FCS control and comms tasks with the state data provided as
    though it came from the AHRS, and returns the control output.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")

    global ahrs_tick, ahrs_state, nav_state, control_state
    _fcs.fcs_measurement_log_init(ahrs_state.measurements, ahrs_tick)
    ahrs_tick += 50

    ahrs_state.lat = lat
    ahrs_state.lon = lon
    ahrs_state.alt = alt
    ahrs_state.velocity = velocity
    ahrs_state.attitude = attitude
    ahrs_state.angular_velocity = angular_velocity
    ahrs_state.wind_velocity = wind_velocity

    _fcs.fcs_control_tick()

    return [
        control_state.controls[0].setpoint,
        control_state.controls[1].setpoint,
        control_state.controls[2].setpoint
    ]


def write(stream_id, value):
    """
    Writes the character array `value` (up to 255 bytes) to the stream
    identified by `stream_id`. Returns the number of bytes written.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")
    if stream_id < 0 or stream_id > FCS_STREAM_NUM_DEVICES:
        raise ValueError("Invalid stream ID")
    if len(value) >= 256:
        raise ValueError(
            "Input value is too long (got %d bytes, max is 255)" % len(value))

    bytes_written = _fcs._fcs_stream_write_to_rx_buffer(
        stream_id, value, len(value))

    return bytes_written


def read(stream_id, max_len):
    """
    Reads up to `max_len` bytes (which must be equal to or smaller than 255)
    from the stream identified by `stream_id`.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")
    if stream_id < 0 or stream_id > FCS_STREAM_NUM_DEVICES:
        raise ValueError("Invalid stream ID")
    if max_len >= 256:
        raise ValueError(
            "Too many bytes requested (got %d, max is 255)" % max_len)
    elif max_len < 1:
        raise ValueError(
            "Can't request fewer than 1 bytes (got %d)" % max_len)

    buf = create_string_buffer(256)
    bytes_read = _fcs._fcs_stream_read_from_tx_buffer(
        stream_id, buf, max_len)

    return buf[0:bytes_read]


def init(dll_path):
    """
    Loads the FCS dynamic library at `dll_path` and sets up the ctypes
    interface. Must be called before any other functions from this module.
    """
    global _fcs, ahrs_state, nav_state, control_state
    # Load the library
    _fcs = cdll.LoadLibrary(dll_path)

    # Get a reference to the required globals
    ahrs_state = AHRSState.in_dll(_fcs, "fcs_global_ahrs_state")
    nav_state = NavState.in_dll(_fcs, "fcs_global_nav_state")
    control_state = ControlState.in_dll(_fcs, "fcs_global_control_state")

    # From ahrs/ahrs.h
    _fcs.fcs_ahrs_init.argtypes = []
    _fcs.fcs_ahrs_init.restype = None

    _fcs.fcs_ahrs_tick.argtypes = []
    _fcs.fcs_ahrs_tick.restype = None

    # From ahrs/measurement.h
    _fcs.fcs_measurement_log_init.argtypes = [POINTER(AHRSMeasurementLog),
                                              c_ushort]
    _fcs.fcs_measurement_log_init.restype = None

    # From comms/comms.h
    _fcs.fcs_comms_init.argtypes = []
    _fcs.fcs_comms_init.restype = None

    _fcs.fcs_comms_tick.argtypes = []
    _fcs.fcs_comms_tick.restype = None

    # From drivers/stream.c
    _fcs._fcs_stream_write_to_rx_buffer.argtypes = [c_ubyte, c_char_p,
                                                    c_ulong]
    _fcs._fcs_stream_write_to_rx_buffer.restype = c_ulong

    _fcs._fcs_stream_read_from_tx_buffer.argtypes = [c_ubyte, c_char_p,
                                                     c_ulong]
    _fcs._fcs_stream_read_from_tx_buffer.restype = c_ulong

    # From hardware/platform/cpuv1-ioboardv1.c
    _fcs.fcs_board_init_platform.argtypes = []
    _fcs.fcs_board_init_platform.restype = None

    _fcs.fcs_board_tick.argtypes = []
    _fcs.fcs_board_tick.restype = None

    # From control/control.h
    _fcs.fcs_control_init.argtypes = []
    _fcs.fcs_control_init.restype = None

    _fcs.fcs_control_tick.argtypes = []
    _fcs.fcs_control_tick.restype = None

    # From util/util.h
    _fcs.fcs_util_init.argtypes = []
    _fcs.fcs_util_init.restype = None

    reset()


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
    sock.sendall("extplane-set update_interval 0.02\n")

    # Wait for some data
    sock.recv(1024)

    return sock


def reset_xplane_state(s, yaw=0.0, pitch=0.0, roll=0.0, velocity=None):
    s.sendall("world-set -37.8136 144.9 100\n")
    time.sleep(1.0)

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
    update = "set sim/operation/override/override_planepath [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n"
    s.sendall(update)

    s.setblocking(0)


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
    except socket.error:
        pass

    # Average out changes in wind over a period of time
    sim_state["wind"][0] += (sim_ref["wind_n"] - sim_state["wind"][0]) * 0.01
    sim_state["wind"][1] += (sim_ref["wind_e"] - sim_state["wind"][1]) * 0.01
    sim_state["wind"][2] += (sim_ref["wind_d"] - sim_state["wind"][2]) * 0.01

    # Recalculate quaternion in case euler angles have been updated.
    sim_state["attitude"] = euler_to_q(
        sim_ref["attitude_yaw"], sim_ref["attitude_pitch"],
        sim_ref["attitude_roll"]
    )

    if sim_state["attitude"][3] < 0:
        sim_state["attitude"] = map(sim_state["attitude"], lambda x: -x)


def send_control_to_xplane(s, controls):
    update = "set sim/flightmodel/engine/ENGN_thro_use [%.6f,0,0,0,0,0,0,0]\n" % controls[0]
    update += "set sim/flightmodel/controls/wing1l_ail1def %.6f\n" % math.degrees(controls[1] - 0.5)
    update += "set sim/flightmodel/controls/wing1r_ail1def %.6f\n" % math.degrees(controls[2] - 0.5)
    s.sendall(update)


if __name__ == "__main__":
    init(sys.argv[1])
    if len(sys.argv) > 2:
        max_t = int(sys.argv[2])
    else:
        max_t = -1

    sock = connect_to_xplane()
    reset_xplane_state(sock)

    time.sleep(1.0)

    enable_xplane_sim(sock)

    while True:
        iter_start = time.time()

        recv_state_from_xplane(sock)
        controls = tick()
        print "T: %08.6f   L: %08.6f   R: %08.6f" % tuple(controls[0:3])
        send_control_to_xplane(sock, controls)

        #print "\n".join(repr(w) for w in list(nav_state.reference_trajectory))

        slack_time = 0.02 - (time.time() - iter_start)
        if slack_time > 0:
            time.sleep(slack_time)
        else:
            print "DEADLINE MISSED: %.3fs late" % -slack_time
