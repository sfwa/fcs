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
import binascii
import traceback
from enum import Enum
from cobs import cobsr


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
        (ValueType.FCS_VALUE_FLOAT, 8): None,
        (ValueType.FCS_VALUE_FLOAT, 16): None,
        (ValueType.FCS_VALUE_FLOAT, 32): 'f',
        (ValueType.FCS_VALUE_FLOAT, 64): 'd',
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

        values = struct.unpack("<%d%s" % (value_count, value_fmt),
                               data[0:value_len])
        data = data[value_len:]

        try:
            param = cls(device_id=device_id,
                        parameter_type=ParameterType(parameter_type),
                        value_type=value_type, value_precision=value_precision,
                        values=list(values))
        except Exception:
            traceback.print_exc()
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
            "key": self.key,
            "value": binascii.b2a_hex(self.value)
        })

    def serialize(self):
        # Pack the header byte -- high bit to indicate it's a key/value param,
        # and then the remaining 7 bits are the length in bytes
        header = (0x80 | len(self.value))
        # Serialize the header and data in one go
        return struct.pack("<BBB4s%ds" % len(self.value), header,
                           self.device_id, self.parameter_type.value,
                           self.key, self.value)

    @classmethod
    def deserialize(cls, data):
        header, device_id, parameter_type = struct.unpack("<BBB", data[0:3])
        value_len = header & 0x7f

        try:
            param = cls(device_id=device_id,
                        parameter_type=ParameterType(parameter_type),
                        key=data[3:7], value=data[7:value_len + 7])
        except Exception:
            traceback.print_exc()
            param = None

        return param, data[value_len + 7:]


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
    return dict(zip(
        ("lat", "lon", "alt", "airspeed", "yaw", "pitch", "roll", "flags"),
        struct.unpack("<ddfffffL", data)))


def q_to_euler(q):
    qx = -q[0]
    qy = -q[1]
    qz = -q[2]
    qw = q[3]

    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
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


def print_estimate_log(data):
    tmp = data.find_by(
        device_id=0,
        parameter_type=ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA).values
    pos = [math.degrees(tmp[0] * math.pi / 2**31), math.degrees(tmp[1] * math.pi / 2**31), tmp[2] * 1e-2]

    tmp = data.find_by(
        device_id=0,
        parameter_type=ParameterType.FCS_PARAMETER_ESTIMATED_VELOCITY_NED).values
    v = map(lambda x: float(x) * 1e-2, tmp)

    att = data.find_by(
        device_id=0,
        parameter_type=ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q).values
    att_ypr = list(q_to_euler(map(lambda x: float(x) / 32767.0, att)))

    tmp = data.find_by(
        device_id=0,
        parameter_type=ParameterType.FCS_PARAMETER_ESTIMATED_ANGULAR_VELOCITY_XYZ).values
    angular_v = map(lambda x: math.degrees(float(x) / (32767.0 / math.pi * 0.25)), tmp)

    tmp = data.find_by(
        device_id=0,
        parameter_type=ParameterType.FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED).values
    wind_v = map(lambda x: float(x) * 1e-2, tmp)

    mode = chr(data.find_by(
        device_id=0,
        parameter_type=ParameterType.FCS_PARAMETER_AHRS_MODE).values[0])

    print ("lat=%12.8f, lon=%12.8f, alt=%6.2f, " + \
           "vn=%5.2f, ve=%5.2f, vd=%5.2f, " + \
           "yaw=%5.1f, pitch=%5.1f, roll=%6.1f, " + \
           "yaw_rate=%5.1f, pitch_rate=%5.1f, roll_rate=%5.1f, " + \
           "wind_vn=%5.2f, wind_ve=%5.2f, wind_vd=%5.2f, mode=%s") % \
          tuple(pos + v + att_ypr + angular_v + wind_v + [mode])


if __name__ == "__main__":
    n = 0
    in_packet = False
    got_data = False
    data = ''
    while True:
        c = sys.stdin.read(1)
        if not c:
            break
        elif c == '\x00' and not in_packet:
            in_packet = True
            got_data = False
        elif c == '\x00' and not got_data:
            in_packet = True
            got_data = False
        elif c == '\x00' and got_data and in_packet:
            n += 1
            if n % 20 == 0:
                try:
                    logf = ParameterLog.deserialize(data)
                    #print repr(logf)
                except Exception:
                    print "Invalid packet: %s" % binascii.b2a_hex(data)
                else:
                    try:
                        print_estimate_log(logf)
                    except Exception:
                        print "Incomplete packet"
            data = ''
            in_packet = False
            got_data = False
        else:
            data += c
            got_data = True

