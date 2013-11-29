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


def reset():
    """
    (Re-)initializes all FCS modules.
    """
    if not _fcs:
        raise RuntimeError("Please call init()")

    _fcs.fcs_config_init()
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

    _fcs.fcs_comms_tick()
    _fcs.fcs_config_tick()
    _fcs.fcs_piksi_tick()
    _fcs.fcs_ahrs_tick()
    _fcs.fcs_nmpc_tick()


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
    global _fcs
    # Load the library
    _fcs = cdll.LoadLibrary(dll_path)

    # From ahrs/ahrs.h
    _fcs.fcs_ahrs_init.argtypes = []
    _fcs.fcs_ahrs_init.restype = None

    _fcs.fcs_ahrs_tick.argtypes = []
    _fcs.fcs_ahrs_tick.restype = None

    # From comms/comms.h
    _fcs.fcs_comms_init.argtypes = []
    _fcs.fcs_comms_init.restype = None

    _fcs.fcs_comms_tick.argtypes = []
    _fcs.fcs_comms_tick.restype = None

    # From config/config.h
    _fcs.fcs_config_init.argtypes = []
    _fcs.fcs_config_init.restype = None

    _fcs.fcs_config_tick.argtypes = []
    _fcs.fcs_config_tick.restype = None

    # From drivers/stream.c
    _fcs._fcs_stream_write_to_rx_buffer.argtypes = [c_ubyte, c_char_p,
                                                    c_ulong]
    _fcs._fcs_stream_write_to_rx_buffer.restype = c_ulong

    _fcs._fcs_stream_read_from_tx_buffer.argtypes = [c_ubyte, c_char_p,
                                                     c_ulong]
    _fcs._fcs_stream_read_from_tx_buffer.restype = c_ulong

    _fcs.fcs_stream_bytes_available.argtypes = [c_ubyte]
    _fcs.fcs_stream_bytes_available.restype = c_ulong

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
