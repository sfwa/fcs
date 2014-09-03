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

    sensor_health = fcs.get_sensor_health()

    try:
        return plog.ParameterLog.deserialize(fcs.read(0, 1023)), sensor_health
    except Exception:
        return None, None


if __name__ == "__main__":
    fcs.init(sys.argv[1])

    infile = open(sys.argv[2], 'rb') if len(sys.argv) > 2 else sys.stdin
    outfile = open(sys.argv[3], 'wb') if len(sys.argv) > 3 else sys.stdout

    outfile.write("t,lat,lon,alt,v,vn,ve,vd,q0,q1,q2,q3,yaw,pitch,roll,vroll,vpitch,vyaw,wn,we,wd,mode,")
    outfile.write("accel_x,accel_y,accel_z,accel_i_x,accel_i_y,accel_i_z,gyro_x,gyro_y,gyro_z,gyro_i_x,gyro_i_y,gyro_i_z,mag_x,mag_y,mag_z,mag_i_x,mag_i_y,mag_i_z,gps_pos_n,gps_pos_e,gps_pos_i_n,gps_pos_i_e,gps_vel_n,gps_vel_e,gps_vel_d,gps_vel_i_n,gps_vel_i_e,gps_vel_i_d,pitot,pitot_i,baro,baro_i,last\n")

    estimate_t = 0.0
    indata = infile.read().split('\x00\x00')
    for io0, io1 in grouped(indata, 2):
        data, health = tick('\x00' + io0.strip('\x00') + '\x00',
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

        vv = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

        outfile.write(
            (
                "%.3f," +
                "%.8f,%.8f,%.2f," +
                "%.2f,%.2f,%.2f,%.2f," +
                "%.5f,%.5f,%.5f,%.5f," +
                "%.1f,%.1f,%.1f," +
                "%.1f,%.1f,%.1f," +
                "%.2f,%.2f,%.2f," +
                "%s,"
            ) % tuple(
                [estimate_t] +
                pos +
                [vv] + v +
                att_q +
                att_ypr +
                angular_v +
                wind_v +
                [mode]
            )
        )

        if health.accel_gyro_present:
            outfile.write("%f,%f,%f,%f,%f,%f," %
                (tuple(health.accel_value) + tuple(health.accel_innovation)))
            outfile.write("%f,%f,%f,%f,%f,%f," %
                (tuple(health.gyro_value) + tuple(health.gyro_innovation)))
        else:
            outfile.write(",,,,,,")
            outfile.write(",,,,,,")

        if health.mag_present:
            outfile.write("%f,%f,%f,%f,%f,%f," %
                (tuple(health.mag_value) + tuple(health.mag_innovation)))
        else:
            outfile.write(",,,,,,")

        if health.gps_present:
            outfile.write("%f,%f,%f,%f," %
                (tuple(health.gps_position_value) + tuple(health.gps_position_innovation)))
            outfile.write("%f,%f,%f,%f,%f,%f," %
                (tuple(health.gps_velocity_value) + tuple(health.gps_velocity_innovation)))
        else:
            outfile.write(",,,,")
            outfile.write(",,,,,,")

        if health.pitot_present:
            outfile.write("%f,%f," % (health.pitot_value, health.pitot_innovation))
        else:
            outfile.write(",,")

        if health.barometer_present:
            outfile.write("%f,%f," % (health.barometer_value, health.barometer_innovation))
        else:
            outfile.write(",,")

        outfile.write("\n")

    infile.close()
    outfile.close()
