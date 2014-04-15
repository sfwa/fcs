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


import math
import struct
import binascii
from enum import Enum
from cobs import cobsr


FCS_LOG_MIN_LENGTH = 5
FCS_LOG_MAX_LENGTH = 1016
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
    FCS_PARAMETER_RADIO,
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
    FCS_PARAMETER_NAV_PATH,
    FCS_PARAMETER_AHRS_STATUS,
    FCS_PARAMETER_CONTROL_STATUS,
    FCS_PARAMETER_KEY_VALUE) = range(1, 54)


class ValueType(Enum):
    (FCS_VALUE_UNSIGNED,
    FCS_VALUE_SIGNED,
    FCS_VALUE_FLOAT,
    FCS_VALUE_RESERVED) = range(4)


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
        pass


class DataParameter(Parameter):
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

    def serialize(self):
        # Convert bits to log(bytes).
        precision = int(round(math.log(self.value_precision / 8, 2)))
        # Pack the header byte with value type, precision and count
        header = (((self.value_type.value << 5) & 0x60) |
                  ((precision << 3) & 0x18) | len(self.values))

        # Format the parameter header
        result = struct.pack("<BBB", header, self.device_id,
                             self.parameter_type.value)

        # Find the correct format character given the value type and
        # precision.
        value_fmt = {
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
        }[self.value_type, self.value_precision]

        if not value_fmt:
            raise ValueError("Can't serialize format %s with precision %d" %
                             (self.value_type, self.value_precision))

        # Serialize the value data
        result += struct.pack("<%d%s" % (len(self.values) - 1, value_fmt),
                              *self.values)

        return result

    @classmethod
    def deserialize(cls, data):
        pass


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
        pass


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

    @classmethod
    def deserialize(cls, data):
        pass

    def serialize(self):
        header = struct.pack("<BHH", self.log_type.value, 0, self.tick)
        data = "".join([p.serialize() for p in self])
        if len(data) + len(header) > FCS_LOG_MAX_LENGTH:
            raise ValueError("Log too long to serialize (%d bytes)" %
                             (len(data) + len(header)))

        crc = struct.pack("<L" , binascii.crc32(header + data) & 0xFFFFFFFF)

        return cobsr.encode(header + data + crc)

    def merge(self, src):
        self.extend(src)

        if self.log_type != src.log_type:
            self.log_type = LogType.FCS_LOG_TYPE_COMBINED

    def find_by(self, device_id=None, parameter_type=None):
        pass

    def print_c_serialization(self):
        print ''.join(('\\x%02x' % ord(c)) for c in self.serialize())

    def print_c_data(self):
        print ''.join(('\\x%02x' % ord(c)) for c in
                      cobsr.decode(self.serialize())[:-4])
