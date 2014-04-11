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
#include "exports/log.h"
}

#include <cstdlib>
#include <cstring>

/* Log initialization assert conditions */
TEST(Log, InitializationNullPtr) {
    EXPECT_DEATH(
        { fcs_log_init(NULL, FCS_LOG_TYPE_COMBINED, 0); },
        "Assertion.*failed");
}

TEST(Log, InitializationInvalidType) {
    EXPECT_DEATH(
        { struct fcs_log_t l; fcs_log_init(&l, FCS_LOG_TYPE_INVALID, 0); },
        "Assertion.*failed");

    EXPECT_DEATH(
        { struct fcs_log_t l; fcs_log_init(&l, FCS_LOG_TYPE_LAST, 0); },
        "Assertion.*failed");
}

/* Correct log initialization */
TEST(Log, InitializationValid) {
    struct fcs_log_t l;
    uint8_t s[] = "\x01\x00\x00\xe5\xf0";

    fcs_log_init(&l, FCS_LOG_TYPE_MEASUREMENT, 0xF0E5u);

    EXPECT_BUFFER_EQ(s, l.data, sizeof(s) - 1u);
    EXPECT_EQ(sizeof(s) - 1u, l.length);
}

/* Log serialization assert conditions */
TEST(Log, SerializationNullBuf) {
    EXPECT_DEATH(
        {
            struct fcs_log_t l;
            l.length = 10u;
            l.data[0] = FCS_LOG_TYPE_MEASUREMENT;
            fcs_log_serialize(NULL, 5u, &l);
        },
        "Assertion.*failed"
    );
}

TEST(Log, SerializationZeroLengthBuf) {
    EXPECT_DEATH(
        {
            struct fcs_log_t l;
            uint8_t buf[20];
            l.length = 10u;
            l.data[0] = FCS_LOG_TYPE_MEASUREMENT;
            fcs_log_serialize(buf, 0, &l);
        },
        "Assertion.*failed"
    );
}

TEST(Log, SerializationNullLog) {
    EXPECT_DEATH(
        {
            uint8_t buf[20];
            fcs_log_serialize(buf, 20u, NULL);
        },
        "Assertion.*failed"
    );
}

TEST(Log, SerializationBufTooShort) {
    EXPECT_DEATH(
        {
            struct fcs_log_t l;
            uint8_t buf[10];
            l.length = 10u;
            l.data[0] = FCS_LOG_TYPE_MEASUREMENT;
            fcs_log_serialize(buf, 10u, &l);
        },
        "Assertion.*failed"
    );
}

/* Correct log serialization */
TEST(Log, SerializationBufJustFits) {
    struct fcs_log_t l;
    uint8_t buf[17];
    size_t len;
    l.length = 10u;
    l.data[0] = FCS_LOG_TYPE_MEASUREMENT;
    len = fcs_log_serialize(buf, 17u, &l);

    EXPECT_EQ(16u, len);
}

TEST(Log, SerializationMinLength) {
    struct fcs_log_t l;
    uint8_t buf[32];
    uint8_t r[] = "\x00\x02\x01\x01\x65\xe5\xf0\x18\x2b\xf6\x00";
    size_t len;

    fcs_log_init(&l, FCS_LOG_TYPE_MEASUREMENT, 0xF0E5u);

    len = fcs_log_serialize(buf, 32u, &l);

    EXPECT_EQ(11u, len);
    EXPECT_BUFFER_EQ(r, buf, sizeof(r) - 1u);
}

TEST(Log, SerializationMaxLength) {
    struct fcs_log_t l;
    uint8_t buf[1024];
    size_t len;

    memset(&l, 0x5Eu, sizeof(l));
    memset(buf, 0xFFu, sizeof(buf));

    fcs_log_init(&l, FCS_LOG_TYPE_MEASUREMENT, 0xF0E5u);
    l.length = FCS_LOG_MAX_LENGTH;

    len = fcs_log_serialize(buf, 1024u, &l);

    EXPECT_EQ(1023u, len);
    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[1022]);
    EXPECT_EQ(0xFFu, buf[1023]);
}

/* Log deserialization assert conditions */
TEST(Log, DeserializationNullBuf) {
    EXPECT_DEATH(
        { struct fcs_log_t l; fcs_log_deserialize(&l, NULL, 5u); },
        "Assertion.*failed");
}

TEST(Log, DeserializationNullLog) {
    EXPECT_DEATH(
        { uint8_t b[50]; fcs_log_deserialize(NULL, b, 50u); },
        "Assertion.*failed");
}

TEST(Log, DeserializationZeroLengthBuf) {
    EXPECT_DEATH(
        { uint8_t b; struct fcs_log_t l; fcs_log_deserialize(&l, &b, 0); },
        "Assertion.*failed");
}

/*
Log deserialization error conditions (returns false rather than asserting)
*/
TEST(Log, DeserializationTooShort) {
    uint8_t buf[1024];
    struct fcs_log_t l;
    bool result;

    memset(buf, 0xFFu, sizeof(buf));
    buf[0] = 0;
    buf[10] = 0;

    result = fcs_log_deserialize(&l, buf, 1u);

    EXPECT_EQ(false, result);
}

TEST(Log, DeserializationTooLong) {
    uint8_t buf[1024];
    struct fcs_log_t l;
    bool result;

    memset(buf, 0xFFu, sizeof(buf));
    buf[0] = 0;
    buf[1023] = 0;

    result = fcs_log_deserialize(&l, buf, 1024u);

    EXPECT_EQ(false, result);
}

TEST(Log, DeserializationMissingFraming) {
    uint8_t buf[] = "\x02\x05\x01\x01\x01\x02\x0b\x03\x2e\x01\x02\x02\x02\x03"
                    "\x05\x5b\x83\xb7\x02",
            buf2[] = "\x00\x02\x05\x01\x01\x01\x02\x0b\x03\x2e\x01\x02\x02"
                     "\x02\x03\x05\x5b\x83\xb7\x02",
            buf3[] = "\x02\x05\x01\x01\x01\x02\x0b\x03\x2e\x01\x02\x02\x02"
                     "\x03\x05\x5b\x83\xb7\x02\x00";
    struct fcs_log_t l;
    bool result;

    result = fcs_log_deserialize(&l, buf, sizeof(buf) - 1u);

    EXPECT_EQ(false, result);

    result = fcs_log_deserialize(&l, buf2, sizeof(buf2) - 1u);

    EXPECT_EQ(false, result);

    result = fcs_log_deserialize(&l, buf3, sizeof(buf3) - 1u);

    EXPECT_EQ(false, result);
}

TEST(Log, DeserializationUnexpectedZero) {
    uint8_t buf[] = "\x00\x02\x05\x01\x01\x01\x02\x0b\x03\x2e\x00\x01\x02\x02"
                    "\x02\x03\x05\x5b\x83\xb7\x02\x00";
    struct fcs_log_t l;
    bool result;

    result = fcs_log_deserialize(&l, buf, sizeof(buf) - 1u);

    EXPECT_EQ(false, result);
}

TEST(Log, DeserializationPayloadTooShort) {
    uint8_t buf[] = "\x00\x02\x05\x01\x00\x01\x01\x02\x0b\x03\x2e\x00\x01\x02"
                    "\x02\x02\x03\x05\x5b\x83\xb7\x02\x00";
    struct fcs_log_t l;
    bool result;

    result = fcs_log_deserialize(&l, buf, sizeof(buf) - 1u);

    EXPECT_EQ(false, result);
}

TEST(Log, DeserializationPayloadTooLong) {
    /* TODO: encoded packet short enough, but payload length exceeds 1013B */
}

TEST(Log, DeserializationInvalidCRC) {
    uint8_t buf[] = "\x00\x02\x05\x01\x01\x01\x02\x0b\x03\x2e\x01\x02\x02\x02"
                    "\x03\x05\x5b\x83\xb6\x02";
    struct fcs_log_t l;
    bool result;

    result = fcs_log_deserialize(&l, buf, sizeof(buf) - 1u);

    EXPECT_EQ(false, result);
}

/* Log deserialization success conditions */
TEST(Log, DeserializationValid) {
    uint8_t buf[] = "\x00\x02\x01\x01\x04\xe5\xf0\x0b\x03\x2e\x01\x02\x02\x02"
                    "\x03\x5d\x3c\x8f\x35\x00";
    uint8_t r[] = "\x01\x00\x00\xe5\xf0\x0b\x00\x2e\x01\x00\x02\x00\x03\x00";
    struct fcs_log_t l;
    bool result;

    result = fcs_log_deserialize(&l, buf, sizeof(buf) - 1u);

    EXPECT_EQ(true, result);
    EXPECT_EQ(sizeof(r) - 1u, l.length);
    EXPECT_BUFFER_EQ(r, l.data, sizeof(r) - 1u);
}

/* Log serialization round-trip */
TEST(Log, ShortSerDesValid) {
    uint8_t buf[32u];
    struct fcs_log_t l, r;
    bool result;
    size_t len;

    fcs_log_init(&l, FCS_LOG_TYPE_MEASUREMENT, 0xF0E5u);

    len = fcs_log_serialize(buf, 32u, &l);

    ASSERT_EQ(11u, len);

    result = fcs_log_deserialize(&r, buf, len);

    EXPECT_EQ(l.length, r.length);
    EXPECT_BUFFER_EQ(l.data, r.data, r.length);
}

TEST(Log, LongSerDesValid) {
    uint8_t buf[1024u];
    struct fcs_log_t l, r;
    bool result;
    size_t len;

    memset(l.data, 0xFFu, sizeof(l.data));

    fcs_log_init(&l, FCS_LOG_TYPE_MEASUREMENT, 0xF0E5u);
    l.length = FCS_LOG_MAX_LENGTH;

    len = fcs_log_serialize(buf, 1024u, &l);

    ASSERT_EQ(FCS_LOG_SERIALIZED_LENGTH - 1u, len);

    result = fcs_log_deserialize(&r, buf, len);

    EXPECT_EQ(l.length, r.length);
    EXPECT_BUFFER_EQ(l.data, r.data, r.length);
}
