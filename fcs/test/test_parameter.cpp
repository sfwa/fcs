/*
Copyright (C) 2014 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "test.h"

extern "C" {
#include "exports/parameter.h"
}

#include <cstdlib>
#include <cstring>

/* Parameter read access on null pointer, triggering an assertion */
TEST(Parameter, GettingNullPtr) {
    EXPECT_DEATH(
        { fcs_parameter_get_type(NULL); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_get_device_id(NULL); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_get_num_values(NULL); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_get_precision_bits(NULL); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_get_length(NULL); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_get_value_type(NULL); },
        "Assertion.*failed");
}

/* Parameter read access on invalid parameters, triggering an assertion */
TEST(Parameter, GettingInvalidType) {
    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0;
            p.type = FCS_PARAMETER_INVALID;
            fcs_parameter_get_type(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0;
            p.type = FCS_PARAMETER_INVALID;
            fcs_parameter_get_device_id(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0;
            p.type = FCS_PARAMETER_INVALID;
            fcs_parameter_get_num_values(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0;
            p.type = FCS_PARAMETER_INVALID;
            fcs_parameter_get_precision_bits(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0;
            p.type = FCS_PARAMETER_INVALID;
            fcs_parameter_get_length(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0;
            p.type = FCS_PARAMETER_INVALID;
            fcs_parameter_get_value_type(&p);
        },
        "Assertion.*failed"
    );
}

TEST(Parameter, GettingInvalidLength) {
    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x1Fu;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_type(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x1Fu;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_device_id(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x1Fu;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_num_values(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x1Fu;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_precision_bits(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x1Fu;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_length(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x1Fu;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_value_type(&p);
        },
        "Assertion.*failed"
    );
}

TEST(Parameter, GettingInvalidValueType) {
    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x60u;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_type(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x60u;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_device_id(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x60u;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_num_values(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x60u;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_precision_bits(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x60u;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_length(&p);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            p.header = 0x60u;
            p.type = FCS_PARAMETER_ACCELEROMETER_XYZ;
            fcs_parameter_get_value_type(&p);
        },
        "Assertion.*failed"
    );
}

/* Valid parameter read access */
TEST(Parameter, GettingValid) {
    /*
    >>> import plog
    >>> p = plog.ParameterLog()
    >>> p.append(plog.DataParameter(
    ...     device_id=1,
    ...     parameter_type=plog.ParameterType.FCS_PARAMETER_MAGNETOMETER_XYZ,
    ...     value_type=plog.ValueType.FCS_VALUE_SIGNED, value_precision=16,
    ...     values=[0x0024, 0x2400, 0x0421]))
    >>> p.print_c_data()
    */
    uint8_t param_data[] = "\x2A\x01\x03\x24\x00\x00\x24\x21\x04";
    struct fcs_parameter_t param;

    memcpy(&param, param_data, sizeof(param_data) - 1u);

    EXPECT_EQ(1u, fcs_parameter_get_device_id(&param));
    EXPECT_EQ(FCS_PARAMETER_MAGNETOMETER_XYZ, fcs_parameter_get_type(&param));
    EXPECT_EQ(FCS_VALUE_SIGNED, fcs_parameter_get_value_type(&param));
    EXPECT_EQ(16u, fcs_parameter_get_precision_bits(&param));
    EXPECT_EQ(9u, fcs_parameter_get_length(&param));
    EXPECT_EQ(3u, fcs_parameter_get_num_values(&param));
}

/* Parameter write access on null pointer, triggering an assertion */
TEST(Parameter, SettingNullPtr) {
    EXPECT_DEATH(
        { fcs_parameter_set_type(NULL, FCS_PARAMETER_ACCELEROMETER_XYZ); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_set_device_id(NULL, 0); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { fcs_parameter_set_header(NULL, FCS_VALUE_UNSIGNED, 8u, 1u); },
        "Assertion.*failed");
}

/* Parameter write access with invalid data, triggering an assertion */
TEST(Parameter, SettingInvalidValues) {
    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_type(&p, FCS_PARAMETER_INVALID);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_type(&p, FCS_PARAMETER_LAST);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_header(&p, FCS_VALUE_RESERVED, 8u, 1u);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_header(&p, FCS_VALUE_SIGNED, 7u, 1u);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_header(&p, FCS_VALUE_SIGNED, 8u, 0);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_header(&p, FCS_VALUE_SIGNED, 8u, 9u);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_header(&p, FCS_VALUE_SIGNED, 65u, 1u);
        },
        "Assertion.*failed"
    );

    EXPECT_DEATH(
        {
            struct fcs_parameter_t p;
            fcs_parameter_set_header(&p, FCS_VALUE_SIGNED, 64u, 8u);
        },
        "Assertion.*failed"
    );
}

/* Valid parameter write access */
TEST(Parameter, SettingValid) {
    /*
    >>> import plog
    >>> p = plog.ParameterLog()
    >>> p.append(plog.DataParameter(
    ...     device_id=1,
    ...     parameter_type=plog.ParameterType.FCS_PARAMETER_MAGNETOMETER_XYZ,
    ...     value_type=plog.ValueType.FCS_VALUE_SIGNED, value_precision=16,
    ...     values=[0x0024, 0x2400, 0x0421]))
    >>> p.print_c_data()
    */
    uint8_t param_data[] = "\x2A\x01\x03\x24\x00\x00\x24\x21\x04";
    uint8_t buf[1024];
    struct fcs_parameter_t param;

    fcs_parameter_set_header(&param, FCS_VALUE_SIGNED, 16u, 3u);
    fcs_parameter_set_device_id(&param, 1u);
    fcs_parameter_set_type(&param, FCS_PARAMETER_MAGNETOMETER_XYZ);

    param.data.i16[0] = 0x0024;
    param.data.i16[1] = 0x2400;
    param.data.i16[2] = 0x0421;

    memcpy(buf, &param, sizeof(param));

    EXPECT_EQ(0x2A, param.header);
    EXPECT_BUFFER_EQ(param_data, buf, sizeof(param_data) - 1u);
}

/* Key/value read access with invalid arguments, triggering an assertion */
TEST(Parameter, GettingKeyValueNullKey) {

}

TEST(Parameter, GettingKeyValueNullValue) {

}

TEST(Parameter, GettingKeyValueNullParameter) {

}

TEST(Parameter, GettingKeyValueZeroLengthBuffer) {

}

TEST(Parameter, GettingKeyValueTooLong) {

}

/* Key/value read access with valid data */
TEST(Parameter, GettingKeyValueZeroLengthParameter) {

}

TEST(Parameter, GettingKeyValueNormal) {

}

/* Key/value write access with invalid arguments, triggering an assertion */
TEST(Parameter, SettingKeyValueNullParameter) {

}

TEST(Parameter, SettingKeyValueNullKey) {

}

TEST(Parameter, SettingKeyValueNullValue) {

}

TEST(Parameter, SettingKeyValueTooLong) {

}

/* Key/value write access with valid data */
TEST(Parameter, SettingKeyValueZeroLengthParameter) {

}

TEST(Parameter, SettingKeyValueNormal) {

}
