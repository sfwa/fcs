# TODO:
# - Read split measurement log and write measurement 0 and 1 log components to
#   FCS streams 0 and 1
# - Read AHRS estimate log and output those values, along with the input
#   estimates
# - Compare with horizon detection output if video file provided as input
# - Optionally play input estimate, replay estimate and detected horizon as
#   video overlays
# - Optionally play replay estimate via X-Plane

import os
import fcs
import sys
import plog
import math
import time
import vectors
import binascii
import itertools


def grouped(iterable, n):
    "s -> (s0,s1,s2,...sn-1), (sn,sn+1,sn+2,...s2n-1), (s2n,s2n+1,s2n+2,...s3n-1), ..."
    return itertools.izip(*[iter(iterable)]*n)


def tick(io0, io1):
    if not fcs._fcs:
        raise RuntimeError("Please call fcs.init()")

    fcs.write(0, io0)
    fcs.write(1, io1)

    fcs._fcs.fcs_board_tick()
    fcs._fcs.fcs_ahrs_tick()

    fcs.read(1, 1023)
    fcs.read(2, 1023)
    fcs.read(3, 1023)
    fcs.read(4, 1023)

    try:
        return plog.ParameterLog.deserialize(fcs.read(0, 1023))
    except Exception:
        return None


if __name__ == "__main__":
    fcs.init(sys.argv[1])

    infile = open(sys.argv[2], 'rb') if len(sys.argv) > 2 else sys.stdin
    outfile = open(sys.argv[3], 'wb') if len(sys.argv) > 3 else sys.stdout

    outfile.write("t,lat,lon,alt,vn,ve,vd,q0,q1,q2,q3,yaw,pitch,roll,vroll,vpitch,vyaw,wn,we,wd,mode\n")

    estimate_t = 0.0
    indata = infile.read().split('\x00\x00')
    for io0, io1 in grouped(indata, 2):
        data = tick('\x00' + io0.strip('\x00') + '\x00',
                    '\x00' + io1.strip('\x00') + '\x00')
        if not data:
            continue

        estimate_t += 0.001

        for param in data:
            pt = param.parameter_type
            pv = param.values
            if pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA:
                pos = [
                    math.degrees(pv[0] * math.pi / 2**31),
                    math.degrees(pv[1] * math.pi / 2**31),
                    pv[2] * 1e-2
                ]
            elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_VELOCITY_NED:
                v = map(lambda x: float(x) * 1e-2, pv)
            elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q:
                att_q = map(lambda x: float(x) / 32767.0, pv)
                att_ypr = list(plog.q_to_euler(att_q))
            elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ:
                angular_v = map(
                    lambda x: math.degrees(float(x) / (32767.0 / math.pi * 0.25)),
                    pv)
            elif pt == plog.ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED:
                wind_v = map(lambda x: float(x) * 1e-2, pv)
            elif pt == plog.ParameterType.FCS_PARAMETER_AHRS_MODE:
                mode = chr(pv[0])

        outfile.write(
            (
                "%.3f," +
                "%.8f,%.8f,%.2f," +
                "%.2f,%.2f,%.2f," +
                "%.5f,%.5f,%.5f,%.5f," +
                "%.1f,%.1f,%.1f," +
                "%.1f,%.1f,%.1f," +
                "%.2f,%.2f,%.2f," +
                "%s\n"
            ) % tuple(
                [estimate_t] +
                pos +
                v +
                att_q +
                att_ypr +
                angular_v +
                wind_v +
                [mode]
            )
        )

    infile.close()
    outfile.close()
