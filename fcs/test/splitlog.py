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


def tick(log_frame):
    if not fcs._fcs:
        raise RuntimeError("Please call fcs.init()")

    io0_log = plog.ParameterLog(
        log_type=plog.LogType.FCS_LOG_TYPE_MEASUREMENT)
    io1_log = plog.ParameterLog(
        log_type=plog.LogType.FCS_LOG_TYPE_MEASUREMENT)

    for param in log_frame:
        if param.parameter_type.value > plog.ParameterType.FCS_PARAMETER_IO_STATUS.value:
            pass
        elif param.device_id == 0:
            io0_log.append(param)
        elif param.device_id == 1:
            io1_log.append(param)

    return io0_log.serialize() + io1_log.serialize()


if __name__ == "__main__":
    fcs.init(sys.argv[1])

    infile = open(sys.argv[2], 'rb') if len(sys.argv) > 2 else sys.stdin
    outfile = open(sys.argv[3], 'wb') if len(sys.argv) > 3 else sys.stdout

    for logf in plog.iterlogs(infile):
        outfile.write(tick(logf))

    infile.close()
    outfile.close()
