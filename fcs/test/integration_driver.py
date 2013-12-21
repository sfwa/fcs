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
from ctypes import *


"""
Interface to a main.c-less library version of fcs. On startup calls each
module's init function (single-process/single-core), then enters the main loop
and calls the tick functions each timestep.

All I/O goes through stubbed versions of the serial drivers, so each of the
five streams has its own independent input buffer and output buffer. Packets
are read from the input file and written to the output file each timestep.

Input and output file formats are multiplexed and timestamped. Each line
starts with an integer identifying the timestep at which the input on that
line should be processed (or the timestamp at which the output on that line
was generated). The next character is a comma, followed by comma-separated
values for each stream. Timestamps must be in ascending order, but do not have
to be contiguous. Each stream value is a quoted Python string.

Input is read from stdin, and output is written to stdout.

If no file is sent to stdin, press Ctrl-D to send an EOF and allow processing
to continue.
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


ahrs_state = None
ahrs_tick = 0


class TRICALInstance(Structure):
    _fields_ = [
        ("field_norm", c_float),
        ("measurement_noise", c_float),
        ("state", c_float * 9),
        ("state_covariance", c_float * 9 * 9),
        ("measurement_count", c_uint)
    ]


class AHRSCalibrationEntry(Structure):
    _fields_ = [
        ("header", c_ubyte),
        ("sensor", c_ubyte),
        ("type", c_ubyte),
        ("reserved", c_ubyte),
        ("error", c_float),
        ("params", c_float * 9),
        ("orientation", c_float * 4),
        ("offset", c_float * 3)
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

        # UKF covariance
        ("lat_covariance", c_double),
        ("lon_covariance", c_double),
        ("alt_covariance", c_double),
        ("velocity_covariance", c_double * 3),
        ("acceleration_covariance", c_double * 3),
        ("attitude_covariance", c_double * 3),
        ("angular_velocity_covariance", c_double * 3),
        ("angular_acceleration_covariance", c_double * 3),
        ("wind_velocity_covariance", c_double * 3),
        ("gyro_bias_covariance", c_double * 3),

        # Configuration
        ("wmm_field", c_double * 3),
        ("ukf_process_noise", c_double * 24),
        ("ukf_dynamics_model", c_uint),
        ("calibration", AHRSCalibrationMap),
        ("trical_instances", TRICALInstance * 2)
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
    _fcs.fcs_piksi_init()
    _fcs.fcs_ahrs_init()
    _fcs.fcs_nmpc_init()


def tick():
    """
    Runs all periodic FCS tasks.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")

    global ahrs_tick, ahrs_state
    _fcs.fcs_measurement_log_init(ahrs_state.measurements, ahrs_tick)
    ahrs_tick += 1

    _fcs.fcs_board_tick()
    _fcs.fcs_piksi_tick()
    _fcs.fcs_ahrs_tick()
    _fcs.fcs_nmpc_tick()
    _fcs.fcs_comms_tick()


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
    global _fcs, ahrs_state
    # Load the library
    _fcs = cdll.LoadLibrary(dll_path)

    # Get a reference to the state
    ahrs_state = AHRSState.in_dll(_fcs, "fcs_global_ahrs_state")

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

    _fcs.fcs_stream_bytes_available.argtypes = [c_ubyte]
    _fcs.fcs_stream_bytes_available.restype = c_ulong

    # From hardware/platform/cpuv1-ioboardv1.c
    _fcs.fcs_board_init_platform.argtypes = []
    _fcs.fcs_board_init_platform.restype = None

    _fcs.fcs_board_tick.argtypes = []
    _fcs.fcs_board_tick.restype = None

    # From nmpc/nmpc.h
    _fcs.fcs_nmpc_init.argtypes = []
    _fcs.fcs_nmpc_init.restype = None

    _fcs.fcs_nmpc_tick.argtypes = []
    _fcs.fcs_nmpc_tick.restype = None

    # From piksi/piksi.h
    _fcs.fcs_piksi_init.argtypes = []
    _fcs.fcs_piksi_init.restype = None

    _fcs.fcs_piksi_tick.argtypes = []
    _fcs.fcs_piksi_tick.restype = None

    # From util/util.h
    _fcs.fcs_util_init.argtypes = []
    _fcs.fcs_util_init.restype = None

    reset()


if __name__ == "__main__":
    init(sys.argv[1])
    if len(sys.argv) > 2:
        max_t = int(sys.argv[2])
    else:
        max_t = -1

    t = 0
    input_line = "-1,"
    while t != max_t:
        # Handle input for this timestep
        input_t, _, input_data = input_line.partition(",")
        # Wait for the current time to match the input timestep
        if input_t and int(input_t) <= t:
            # Load the next line (to be processed later), if available
            try:
                input_line = sys.stdin.next() or ","
            except StopIteration:
                input_line = ","
            # Process the data for the current timestep
            stream_data = eval("(" + input_data + ")")
            for i, value in enumerate(stream_data):
                bytes_written = write(i, value)
                assert bytes_written == len(value)

        tick()

        # Write output for this timestep
        stream_data = [read(i, 255) for i in xrange(FCS_STREAM_NUM_DEVICES)]
        if any(stream_data):
            sys.stdout.write(
                ("%9d," % t) + ",".join(("%r" % s) for s in stream_data) +
                "\n"
            )
            sys.stdout.flush()

        t += 1
