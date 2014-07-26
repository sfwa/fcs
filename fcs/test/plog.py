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


# Dependencies: enum34, cobs


import sys
import math
import json
import struct
import vectors
import binascii
import traceback
from enum import Enum
from cobs import cobsr


WGS84_A = 6378137.0


FCS_LOG_MIN_LENGTH = 5
FCS_LOG_MAX_LENGTH = 1013
FCS_LOG_SERIALIZED_LENGTH = 1024


class ParameterType(Enum):
    (FCS_PARAMETER_ACCELEROMETER_XYZ,
    FCS_PARAMETER_GYROSCOPE_XYZ,
    FCS_PARAMETER_MAGNETOMETER_XYZ,
    FCS_PARAMETER_PITOT,
    FCS_PARAMETER_PRESSURE_TEMP,
    FCS_PARAMETER_IV,
    FCS_PARAMETER_GPS_POSITION_LLA,
    FCS_PARAMETER_GPS_VELOCITY_NED,
    FCS_PARAMETER_GPS_INFO,
    FCS_PARAMETER_CONTROL_POS,
    FCS_PARAMETER_CONTROL_MODE,
    FCS_PARAMETER_GP_IN,
    FCS_PARAMETER_RADIO,
    FCS_PARAMETER_IO_STATUS,
    FCS_PARAMETER_HAL_ACCELEROMETER_XYZ,
    FCS_PARAMETER_HAL_ACCELEROMETER_VARIANCE,
    FCS_PARAMETER_HAL_ACCELEROMETER_OFFSET_XYZ,
    FCS_PARAMETER_HAL_GYROSCOPE_XYZ,
    FCS_PARAMETER_HAL_GYROSCOPE_VARIANCE,
    FCS_PARAMETER_HAL_MAGNETOMETER_XYZ,
    FCS_PARAMETER_HAL_MAGNETOMETER_VARIANCE,
    FCS_PARAMETER_HAL_AIRSPEED,
    FCS_PARAMETER_HAL_AIRSPEED_VARIANCE,
    FCS_PARAMETER_HAL_PRESSURE_ALTITUDE,
    FCS_PARAMETER_HAL_PRESSURE_ALTITUDE_VARIANCE,
    FCS_PARAMETER_HAL_POSITION_LAT_LON,
    FCS_PARAMETER_HAL_POSITION_LAT_LON_VARIANCE,
    FCS_PARAMETER_HAL_POSITION_ALT,
    FCS_PARAMETER_HAL_POSITION_ALT_VARIANCE,
    FCS_PARAMETER_HAL_VELOCITY_NED,
    FCS_PARAMETER_HAL_VELOCITY_NED_VARIANCE,
    FCS_PARAMETER_DERIVED_REFERENCE_PRESSURE,
    FCS_PARAMETER_DERIVED_REFERENCE_ALT,
    FCS_PARAMETER_ESTIMATED_POSITION_LLA,
    FCS_PARAMETER_ESTIMATED_VELOCITY_NED,
    FCS_PARAMETER_ESTIMATED_ATTITUDE_Q,
    FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ,
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED,
    FCS_PARAMETER_ESTIMATED_GYRO_BIAS_XYZ,
    FCS_PARAMETER_ESTIMATED_POSITION_SD,
    FCS_PARAMETER_ESTIMATED_VELOCITY_SD,
    FCS_PARAMETER_ESTIMATED_ATTITUDE_SD,
    FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_SD,
    FCS_PARAMETER_ESTIMATED_STATIC_PRESSURE,
    FCS_PARAMETER_ESTIMATED_STATIC_TEMP,
    FCS_PARAMETER_ESTIMATED_DYNAMIC_PRESSURE,
    FCS_PARAMETER_ESTIMATED_WMM_FIELD,
    FCS_PARAMETER_CONTROL_SETPOINT,
    FCS_PARAMETER_AHRS_MODE,
    FCS_PARAMETER_GP_OUT,
    FCS_PARAMETER_NAV_VERSION,
    FCS_PARAMETER_NAV_PATH_ID,
    FCS_PARAMETER_NAV_WAYPOINT_ID,
    FCS_PARAMETER_AHRS_STATUS,
    FCS_PARAMETER_CONTROL_STATUS,
    FCS_PARAMETER_KEY_VALUE) = range(1, 57)


class ValueType(Enum):
    (FCS_VALUE_UNSIGNED,
    FCS_VALUE_SIGNED,
    FCS_VALUE_FLOAT,
    FCS_VALUE_RESERVED) = range(4)


FCS_PARAMETER_KEY_PATH = "PATH"
FCS_PARAMETER_KEY_WAYPOINT = "WAYP"
FCS_PARAMETER_KEY_REFERENCE_POINT = "REFP"


class Parameter(object):
    device_id = None
    parameter_type = None

    def __init__(self, device_id=0, parameter_type=None):
        self.device_id = device_id
        self.parameter_type = parameter_type

    def serialize(self):
        pass

    @classmethod
    def deserialize(cls, data):
        if ord(data[0]) & 0x80:
            return KeyValueParameter.deserialize(data)
        else:
            return DataParameter.deserialize(data)


class DataParameter(Parameter):
    VALUE_FORMAT_CHAR = {
        (ValueType.FCS_VALUE_UNSIGNED, 8): 'B',
        (ValueType.FCS_VALUE_UNSIGNED, 16): 'H',
        (ValueType.FCS_VALUE_UNSIGNED, 32): 'L',
        (ValueType.FCS_VALUE_UNSIGNED, 64): 'Q',
        (ValueType.FCS_VALUE_SIGNED, 8): 'b',
        (ValueType.FCS_VALUE_SIGNED, 16): 'h',
        (ValueType.FCS_VALUE_SIGNED, 32): 'l',
        (ValueType.FCS_VALUE_SIGNED, 64): 'q',
        (ValueType.FCS_VALUE_FLOAT, 8): 'B', # shouldn't be needed
        (ValueType.FCS_VALUE_FLOAT, 16): 'H', # shouldn't be needed
        (ValueType.FCS_VALUE_FLOAT, 32): 'f',
        (ValueType.FCS_VALUE_FLOAT, 64): 'd',
        # Shouldn't be needed!
        (ValueType.FCS_VALUE_RESERVED, 8): 'B',
        (ValueType.FCS_VALUE_RESERVED, 16): 'H',
        (ValueType.FCS_VALUE_RESERVED, 32): 'L',
        (ValueType.FCS_VALUE_RESERVED, 64): 'Q'
    }

    value_type = None
    value_precision = None
    values = None

    def __init__(self, device_id=None, parameter_type=None, value_type=None,
                 value_precision=None, values=None):
        super(DataParameter, self).__init__(device_id=device_id,
                                            parameter_type=parameter_type)

        self.value_type = value_type
        self.value_precision = value_precision
        self.values = values or []

    def __repr__(self):
        return json.dumps({
            "class": "DataParameter",
            "deviceId": self.device_id,
            "parameterType": self.parameter_type.name,
            "valueType": self.value_type.name,
            "valuePrecision": self.value_precision,
            "values": list(self.values)
        })

    def serialize(self):
        # Convert bits to log(bytes).
        precision = int(round(math.log(self.value_precision / 8, 2)))
        # Pack the header byte with value type, precision and count
        header = (((self.value_type.value << 5) & 0x60) |
                  ((precision << 3) & 0x18) | (len(self.values) - 1))

        # Format the parameter header
        result = struct.pack("<BBB", header, self.device_id,
                             self.parameter_type.value)

        # Find the correct format character given the value type and
        # precision.
        value_fmt = DataParameter.VALUE_FORMAT_CHAR[
            self.value_type, self.value_precision]

        if not value_fmt:
            raise ValueError("Can't serialize format %s with precision %d" %
                             (self.value_type, self.value_precision))

        # Serialize the value data
        try:
            result += struct.pack("<%d%s" % (len(self.values), value_fmt),
                                  *self.values)
        except Exception:
            print "Couldn't serialize %s: %s" % (repr(self.parameter_type),
                                                 repr(self.values))
            raise

        return result

    @classmethod
    def deserialize(cls, data):
        header, device_id, parameter_type = struct.unpack("<BBB", data[0:3])
        data = data[3:]

        value_type = ValueType((header & 0x60) >> 5)
        value_precision = int(1 << (((header & 0x18) >> 3))) * 8
        value_count = (header & 0x07) + 1

        value_fmt = cls.VALUE_FORMAT_CHAR[value_type, value_precision]
        value_len = value_count * int(value_precision / 8)

        try:
            values = struct.unpack("<%d%s" % (value_count, value_fmt),
                                   data[0:value_len])
        except:
            pass
            #print "Couldn't unpack parameter of type %d from count %d, precision %d: %s" % (
            #            parameter_type, value_count, value_precision, binascii.b2a_hex(data[0:value_len]))
        data = data[value_len:]

        try:
            param = cls(device_id=device_id,
                        parameter_type=ParameterType(parameter_type),
                        value_type=value_type, value_precision=value_precision,
                        values=list(values))
        except Exception:
            #traceback.print_exc()
            param = None

        return param, data


class KeyValueParameter(Parameter):
    key = None
    value = None

    def __init__(self, device_id=None,
                 parameter_type=ParameterType.FCS_PARAMETER_KEY_VALUE,
                 key=None, value=None):
        super(KeyValueParameter, self).__init__(device_id=device_id,
                                                parameter_type=parameter_type)

        self.key = key or ""
        self.value = value or ""

    def __repr__(self):
        return json.dumps({
            "class": "KeyValueParameter",
            "deviceId": self.device_id,
            "parameterType": self.parameter_type.name,
            "key": binascii.b2a_hex(self.key),
            "value": binascii.b2a_hex(self.value)
        })

    def serialize(self):
        # Pack the header byte -- high bit to indicate it's a key/value param,
        # and then the remaining 7 bits are the length in bytes
        header = (0x80 | (len(self.value) + 4))
        # Serialize the header and data in one go
        return struct.pack("<BBB4s%ds" % len(self.value), header,
                           self.device_id, self.parameter_type.value,
                           self.key, self.value)

    @classmethod
    def deserialize(cls, data):
        header, device_id, parameter_type = struct.unpack("<BBB", data[0:3])
        value_len = int(header & 0x7f)
        data = data[3:]
        key = data[0:4]
        data = data[4:]

        try:
            param = cls(device_id=device_id,
                        parameter_type=ParameterType(parameter_type),
                        key=key, value=data[0:value_len - 4])
        except Exception:
            #traceback.print_exc()
            param = None

        return param, data[value_len - 4:]


class LogType(Enum):
    (FCS_LOG_TYPE_MEASUREMENT,
    FCS_LOG_TYPE_SENSOR_HAL,
    FCS_LOG_TYPE_ESTIMATE,
    FCS_LOG_TYPE_CONTROL,
    FCS_LOG_TYPE_COMBINED) = range(1, 6)


class ParameterLog(list):
    log_type = None
    tick = 0

    def __init__(self, log_type=None, tick=0, *args):
        super(ParameterLog, self).__init__(args[0] if len(args) else [])

        self.log_type = log_type or LogType.FCS_LOG_TYPE_COMBINED
        self.tick = tick

    def __repr__(self):
        return json.dumps({
            "logType": self.log_type.name,
            "tick": self.tick,
            "parameters": [json.loads(repr(p)) for p in self]
        })

    @classmethod
    def deserialize(cls, data):
        data = cobsr.decode(data.strip('\x00'))

        data_crc = struct.pack("<L" , binascii.crc32(data[:-4]) & 0xFFFFFFFF)
        if data[-4:] != data_crc:
            raise ValueError("CRC mismatch: provided %s, calculated %s" %
                             (binascii.b2a_hex(data[-4:]),
                              binascii.b2a_hex(data_crc)))

        log_type, _, tick = struct.unpack("<BHH", data[0:5])
        data = data[5:]

        result = cls(log_type=LogType(log_type), tick=tick)

        while len(data) > 4:
            param, tail = Parameter.deserialize(data)
            if param:
                result.append(param)
            if data == tail:
                break
            data = tail

        return result

    def serialize(self):
        header = struct.pack("<BHH", self.log_type.value, 0, self.tick)
        data = "".join([p.serialize() for p in self])
        if len(data) + len(header) > FCS_LOG_MAX_LENGTH:
            raise ValueError("Log too long to serialize (%d bytes)" %
                             (len(data) + len(header)))

        crc = struct.pack("<L" , binascii.crc32(header + data) & 0xFFFFFFFF)

        return "\x00" + cobsr.encode(header + data + crc) + "\x00"

    def merge(self, src):
        self.extend(src)

        if self.log_type != src.log_type:
            self.log_type = LogType.FCS_LOG_TYPE_COMBINED

    def find_by(self, device_id=None, parameter_type=None):
        for param in self:
            if param.device_id == device_id and \
                    param.parameter_type == parameter_type:
                return param

        return None

    def print_c_serialization(self):
        print ''.join(('\\x%02x' % ord(c)) for c in self.serialize())

    def print_c_data(self):
        print ''.join(('\\x%02x' % ord(c)) for c in
                      cobsr.decode(self.serialize())[:-4])


def extract_waypoint(data):
    if len(data) < 40:
        data = data + "\x00" * (40 - len(data))
    elif len(data) > 40:
        data = data[0:40]
    return dict(zip(
        ("lat", "lon", "alt", "airspeed", "yaw", "pitch", "roll", "flags"),
        struct.unpack("<ddfffffL", data)))


def pack_waypoint(waypoint):
    return struct.pack("<ddfffffL", waypoint["lat"], waypoint["lon"],
                       waypoint["alt"], waypoint["yaw"], waypoint["pitch"],
                       waypoint["roll"], waypoint["flags"])


def unpack_path(data):
    return dict(zip(
        ("start_waypoint_id", "end_waypoint_id", "type", "flags",
         "next_path_id"),
        struct.unpack("<HHLHH", data)))


def pack_path(path):
    return struct.pack("<HHLHH", path["start_waypoint_id"],
                       path["end_waypoint_id"], path["type"], path["flags"],
                       path["next_path_id"])


def unpack_boundary(data):
    result = struct.unpack("<H16HB", data)
    return result[1:result[0] + 1]


def pack_boundary(boundary):
    # boundary is an array of waypoint IDs
    return struct.pack("<H16HB", len(boundary), *(boundary + [0]))


def q_to_euler(q):
    qx = -q[0]
    qy = -q[1]
    qz = -q[2]
    qw = q[3]

    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:
        return 0, 0, 0

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    yaw = math.atan2(2.0 * (qx * qy + qw * qz),
                     qw * qw - qz * qz - qy * qy + qx * qx)
    pitch = math.asin(-2.0 * (qx * qz - qy * qw))
    roll = math.atan2(2.0 * (qy * qz + qx * qw),
                      qw * qw + qz * qz - qy * qy - qx * qx)

    yaw = math.degrees(yaw)
    pitch = math.degrees(pitch)
    roll = math.degrees(roll)

    if yaw < 0.0:
        yaw += 360.0

    return yaw, pitch, roll


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


def iterlogs(stream):
    in_packet = False
    got_data = False
    data = ''
    while True:
        c = stream.read(1)
        if not c:
            break
        elif c == '\x00' and not in_packet:
            in_packet = True
            got_data = False
        elif c == '\x00' and not got_data:
            in_packet = True
            got_data = False
        elif c == '\x00' and got_data and in_packet:
            try:
                logf = ParameterLog.deserialize(data)
            except Exception:
                if len(data) != 13:
                    sys.stderr.write("Invalid packet: %s\n" % binascii.b2a_hex(data))
            else:
                yield logf
            data = ''
            in_packet = False
            got_data = False
        else:
            data += c
            got_data = True


def iterlogs_raw(stream):
    in_packet = False
    got_data = False
    data = ''
    while True:
        c = stream.read(1)
        if not c:
            break
        elif c == '\x00' and not in_packet:
            in_packet = True
            got_data = False
        elif c == '\x00' and not got_data:
            in_packet = True
            got_data = False
        elif c == '\x00' and got_data and in_packet:
            yield data
            data = ''
            in_packet = False
            got_data = False
        else:
            data += c
            got_data = True


def print_estimate_log(data):
    pos = [0, 0, 0]
    v = [0, 0, 0]
    att_q = [0, 0, 0, 0]
    att_ypr = [0, 0, 0]
    angular_v = [0, 0, 0]
    wind_v = [0, 0, 0]
    mode = 'X'
    control_mode = 0
    status = [-1, 0, 0, 0]
    path = 0

    for param in data:
        pt = param.parameter_type
        if isinstance(param, DataParameter):
            pv = param.values
        else:
            pv = param.value

        if pt == ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA:
            pos = [
                math.degrees(pv[0] * math.pi / 2**31),
                math.degrees(pv[1] * math.pi / 2**31),
                pv[2] * 1e-2
            ]
        elif pt == ParameterType.FCS_PARAMETER_ESTIMATED_VELOCITY_NED:
            v = map(lambda x: float(x) * 1e-2, pv[0:3])
        elif pt == ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q:
            att_q = map(lambda x: float(x) / 32767.0, pv[0:4])
            att_ypr = list(q_to_euler(att_q))
        elif pt == ParameterType.FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ:
            angular_v = map(
                lambda x: math.degrees(float(x) / (32767.0 / math.pi * 0.25)),
                pv[0:3])
        elif pt == ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED:
            wind_v = map(lambda x: float(x) * 1e-2, pv[0:3])
        elif pt == ParameterType.FCS_PARAMETER_AHRS_MODE:
            mode = chr(pv[0])
        elif pt == ParameterType.FCS_PARAMETER_CONTROL_MODE:
            control_mode = pv[0]
        elif pt == ParameterType.FCS_PARAMETER_CONTROL_STATUS:
            status = pv[0:4]
        elif pt == ParameterType.FCS_PARAMETER_NAV_PATH_ID:
            path = pv[0]

    print (
            "%.8f,%.8f,%.2f," +
            "%.2f,%.2f,%.2f," +
            "%.5f,%.5f,%.5f,%.5f," +
            "%.1f,%.1f,%.1f," +
            "%.1f,%.1f,%.1f," +
            "%.2f,%.2f,%.2f," +
            "%s,%d,%d,%.0f,%.0f,%.0f"
        ) % tuple(
            pos +
            v +
            att_q +
            att_ypr +
            angular_v +
            wind_v +
            ['x', 0, 0] +
            #[mode] +
            #[control_mode] +
            #[path] +
            [status[0], status[2], status[3]]
        )

    return control_mode, math.radians(pos[0]), math.radians(pos[1]), pos[2]


def print_measurement_log(i, data):
    #tmp = data.find_by(device_id=0, parameter_type=ParameterType.FCS_PARAMETER_AHRS_STATUS)
    #if tmp and tmp.values[0] < 10:
    #    return

    #tmp = data.find_by(device_id=0, parameter_type=ParameterType.FCS_PARAMETER_PRESSURE_TEMP)
    #if not tmp:
    #    return

    #print "%d (%d): " % (i, tmp.values[0])

    for device_id in range(2):
        try:
            tmp = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_GPS_INFO).values
            gps_info= "%d,%d,%d" % (tmp[0], tmp[1], tmp[2])
        except Exception:
            gps_info = ",,"

        try:
            tmp = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_GPS_POSITION_LLA).values
            gps_pos = "%.8f,%.8f,%.2f" % (tmp[0] * 1e-7, tmp[1] * 1e-7, tmp[2] * 1e-3)
        except Exception:
            gps_pos = ",,"

        try:
            tmp = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_GPS_VELOCITY_NED).values
            gps_v = "%.2f,%.2f,%.2f" % (tmp[0] * 1e-3, tmp[1] * 1e-3, tmp[2] * 1e-3)
        except Exception:
            gps_v = ",,"

        try:
            accel = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_ACCELEROMETER_XYZ).values
            accel = "%d,%d,%d" % tuple(accel)
        except Exception:
            accel = ",,"

        try:
            gyro = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_GYROSCOPE_XYZ).values
            gyro = "%d,%d,%d" % tuple(gyro)
        except Exception:
            gyro = ",,"

        try:
            mag = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_ACCELEROMETER_XYZ).values
            mag = "%d,%d,%d" % tuple(mag)
        except Exception:
            mag = ",,"

        try:
            pitot = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_PITOT).values
            pitot = "%d" % pitot[0]
        except Exception:
            pitot = ""

        try:
            baro = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_PRESSURE_TEMP).values
            baro = "%d" % baro[0]
        except Exception:
            baro = ""

        try:
            iv = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_IV).values
            iv = "%d,%d" % tuple(iv)
        except Exception:
            iv = ","

        try:
            control_pos = data.find_by(
                device_id=device_id,
                parameter_type=ParameterType.FCS_PARAMETER_CONTROL_POS).values
            control_pos = "%d,%d,%d" % tuple(control_pos)
        except Exception:
            control_pos = ",,"

        sys.stdout.write(",".join([gps_info, gps_pos, gps_v, accel, gyro, mag, pitot, baro, iv, control_pos]) + ("," if device_id == 0 else "\n"))


def print_control_log(data):
    status = [-1, -1, -1, -1]
    path = -1
    setpoint = [-1, -1, -1]

    for param in data:
        pt = param.parameter_type
        if isinstance(param, DataParameter):
            pv = param.values
        else:
            pv = param.value

        if pt == ParameterType.FCS_PARAMETER_CONTROL_STATUS:
            status = pv
        elif pt == ParameterType.FCS_PARAMETER_NAV_PATH_ID:
            path = pv[0]
        elif pt == ParameterType.FCS_PARAMETER_CONTROL_SETPOINT:
            setpoint = map(lambda x: x / 65535.0, pv)

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
        "path=%4d"
    ) % (
        (0.0, ) +
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
        (controls[0], controls[1], controls[2]) +
        (obj_val, cycles, nmpc_errors, nmpc_resets) +
        (path, )
    )

    print "%.0f,%.1f,%.0f,%.0f,%d,%.6f,%.6f,%.6f" % tuple(status + [path] + setpoint)


if __name__ == "__main__":
    print "lat,lon,alt,vn,ve,vd,q0,q1,q2,q3,yaw,pitch,roll,vroll,vpitch,vyaw,wn,we,wd,mode,control_mode,path,objval,errors,resets"

    #print "gps_mode_1,gps_pdop_1,gps_numsv_1,gps_lat_1,gps_lon_1,gps_alt_1,gps_n_1,gps_e_1,gps_d_1,accel_x_1," + \
    #      "accel_y_1,accel_z_1,gyro_x_1,gyro_y_1,gyro_z_1,mag_x_1,mag_y_1," + \
    #      "mag_z_1,pitot_1,baro_1,i_1,v_1,control_thr_1,control_lail_1," + \
    #      "control_rail_1,gps_mode_2,gps_pdop_2,gps_numsv_2,gps_lat_2,gps_lon_2,gps_alt_2,gps_n_2,gps_e_2," + \
    #      "gps_d_2,accel_x_2,accel_y_2,accel_z_2,gyro_x_2,gyro_y_2,gyro_z_2," + \
    #      "mag_x_2,mag_y_2,mag_z_2,pitot_2,baro_2,i_2,v_2,control_thr_2," + \
    #      "control_lail_2,control_rail_2"

    n = 0
    for logf in iterlogs(sys.stdin):
        #waypoint = None
        #result = logf.find_by(device_id=0, parameter_type=ParameterType.FCS_PARAMETER_KEY_VALUE)
        #if result:
        #    waypoint = extract_waypoint(result.value)

        try:
            print_estimate_log(logf)
            #print_measurement_log(n, logf)
            n += 1
        except Exception:
            raise
            #print repr(logf)
            #raise
            #print "Incomplete packet"

