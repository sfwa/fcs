import os
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


FCS_CONTROL_MODE_AUTO = ord("F")
FCS_CONTROL_MODE_MANUAL = ord("R")


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
        ("rate", c_float)
    ]


class ControlState(Structure):
    _fields_ = [
        ("controls", ControlChannel * 4),
        ("mode", c_uint),
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
        ("version", c_uint),
        ("paths", NavPath * 500),
        ("waypoints", NavWaypoint * 1000),
        ("boundary", NavBoundary),
        ("reference_trajectory", NavWaypoint * 101),
        ("reference_path_id", c_ushort * 101)
    ]


class AHRSStateEstimate(Structure):
    _fields_ = [
        ("lat", c_double),
        ("lon", c_double),
        ("alt", c_float),
        ("velocity", c_float * 3),
        ("attitude", c_float * 4),
        ("angular_velocity", c_float * 3),
        ("wind_velocity", c_float * 3),
        ("mode", c_ubyte),
        ("reserved", c_ubyte * 55)
    ]


class SensorHealth(Structure):
    _fields_ = [
        ("accel_value", c_double * 3),
        ("accel_covariance", c_double * 3),
        ("accel_innovation", c_double * 3),
        ("gyro_value", c_double * 3),
        ("gyro_covariance", c_double * 3),
        ("gyro_innovation", c_double * 3),
        ("mag_value", c_double * 3),
        ("mag_covariance", c_double * 3),
        ("mag_innovation", c_double * 3),
        ("gps_position_value", c_double * 2),
        ("gps_position_covariance", c_double * 2),
        ("gps_position_innovation", c_double * 2),
        ("gps_velocity_value", c_double * 3),
        ("gps_velocity_covariance", c_double * 3),
        ("gps_velocity_innovation", c_double * 3),
        ("pitot_value", c_double),
        ("pitot_covariance", c_double),
        ("pitot_innovation", c_double),
        ("barometer_value", c_double),
        ("barometer_covariance", c_double),
        ("barometer_innovation", c_double),
        ("accel_gyro_present", c_ubyte),
        ("mag_present", c_ubyte),
        ("gps_present", c_ubyte),
        ("pitot_present", c_ubyte),
        ("barometer_present", c_ubyte),
    ]


def reset():
    """
    (Re-)initializes all FCS modules.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")

    _fcs.fcs_board_init_platform()
    _fcs.fcs_exports_init()
    _fcs.fcs_ahrs_init()
    _fcs.fcs_control_init()


def write(stream_id, value):
    """
    Writes the character array `value` (up to 255 bytes) to the stream
    identified by `stream_id`. Returns the number of bytes written.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")
    if stream_id < 0 or stream_id > FCS_STREAM_NUM_DEVICES:
        raise ValueError("Invalid stream ID")
    if len(value) >= 1024:
        raise ValueError(
            "Input value is too long (got %d bytes, max is 1023)" % len(value))

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
    if max_len >= 1024:
        raise ValueError(
            "Too many bytes requested (got %d, max is 1023)" % max_len)
    elif max_len < 1:
        raise ValueError(
            "Can't request fewer than 1 bytes (got %d)" % max_len)

    buf = create_string_buffer(1024)
    bytes_read = _fcs._fcs_stream_read_from_tx_buffer(
        stream_id, buf, max_len)

    return buf[0:bytes_read]


def get_sensor_health():
    result = SensorHealth()
    _fcs.ukf_get_sensor_health(result)
    return result


def init(dll_path):
    """
    Loads the FCS dynamic library at `dll_path` and sets up the ctypes
    interface. Must be called before any other functions from this module.
    """
    global _fcs, nav_state, control_state
    # Load the library
    _fcs = cdll.LoadLibrary(dll_path)

    # Get a reference to the required globals
    nav_state = NavState.in_dll(_fcs, "nav_state")
    control_state = ControlState.in_dll(_fcs, "control_state")

    # From ahrs/ahrs.h
    _fcs.fcs_ahrs_init.argtypes = []
    _fcs.fcs_ahrs_init.restype = None

    _fcs.fcs_ahrs_tick.argtypes = []
    _fcs.fcs_ahrs_tick.restype = None

    _fcs.ukf_get_sensor_health.argtypes = [POINTER(SensorHealth)]
    _fcs.ukf_get_sensor_health.restype = None

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

    _fcs.fcs_control_reset.argtypes = []
    _fcs.fcs_control_reset.restype = None

    # From exports/exports.h
    _fcs.fcs_exports_init.argtypes = []
    _fcs.fcs_exports_init.restype = None

    # From control/trajectory.h
    _fcs._get_next_reference_point.argtypes = [POINTER(c_float * 13), c_ulong]
    _fcs._get_next_reference_point.restype = None

    reset()
