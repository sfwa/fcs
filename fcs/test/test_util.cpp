#include "test.h"

extern "C" {
#include "config/config.h"
#include "util/util.h"
}

TEST(CRC8, NotInitialised) {
    EXPECT_DEATH({ fcs_crc8((uint8_t*)"message", 7, 0); }, "^Assertion");
}

TEST(CRC8, Initialisation) {
    fcs_crc8_init(0x2Fu);
}

TEST(CRC8, OneByte) {
    uint8_t result;
    uint8_t data = 'x';

    result = fcs_crc8(&data, 1u, 0x00);
    EXPECT_EQ(0x6Cu, result);
}

TEST(CRC8, AllZeros) {
    uint8_t result;
    uint8_t data[] = "\x00\x00\x00\x00";

    result = fcs_crc8(data, 4, 0x00);
    EXPECT_EQ(0x00, result);
}

TEST(CRC8, Message) {
    uint8_t result;
    uint8_t data[] = "message";

    result = fcs_crc8(data, 7, 0x00);
    EXPECT_EQ(0x38u, result);
}

TEST(CRC8, InitialValue) {
    uint8_t result;
    uint8_t data[] = "message";

    result = fcs_crc8(data, 7, 0x12u);
    EXPECT_EQ(0x25u, result);
}

TEST(CRC8, NoBuffer) {
    EXPECT_DEATH( { fcs_crc8(NULL, 5, 0); }, "^Assertion" );
}

TEST(CRC8, TooLong) {
    EXPECT_DEATH({ uint8_t x[512]; fcs_crc8(x, 512, 0); }, "^Assertion");
}

TEST(CRC8, TooShort) {
    EXPECT_DEATH({ uint8_t x[512]; fcs_crc8(x, 0, 0); }, "^Assertion");
}

TEST(CRC32, OneByte) {
    uint32_t result;
    uint8_t data = 'x';

    result = fcs_crc32(&data, 1u, 0xFFFFFFFFu);
    EXPECT_EQ(0x8CDC1683u, result);
}

TEST(CRC32, AllZeros) {
    uint32_t result;
    uint8_t data[] = "\x00\x00\x00\x00";

    result = fcs_crc32(data, 1u, 0xFFFFFFFFu);
    EXPECT_EQ(0xD202EF8Du, result);
}

TEST(CRC32, Message) {
    uint32_t result;
    uint8_t data[] = "message";

    result = fcs_crc32(data, 7u, 0xFFFFFFFFu);
    EXPECT_EQ(0xB6BD307Fu, result);
}

TEST(CRC32, NoBuffer) {
    EXPECT_DEATH( { fcs_crc32(NULL, 5, 0); }, "^Assertion" );
}

TEST(CRC32, TooShort) {
    EXPECT_DEATH( { uint8_t x[512]; fcs_crc32(x, 0, 0); }, "^Assertion" );
}

TEST(TextChecksum, Message) {
    uint8_t result;
    uint8_t data[] = "GPGLL,5300.97914,N,00259.98174,E,125926,A";

    result = fcs_text_checksum(data, sizeof(data) - 1u);
    EXPECT_EQ(0x28, result);
}

TEST(TextChecksum, NoBuffer) {
    EXPECT_DEATH( { fcs_text_checksum(NULL, 5); }, "^Assertion" );
}

TEST(TextChecksum, TooLong) {
    EXPECT_DEATH(
        { uint8_t x[512]; fcs_text_checksum(x, 512); }, "^Assertion" );
}

TEST(TextChecksum, TooShort) {
    EXPECT_DEATH(
        { uint8_t x[512]; fcs_text_checksum(x, 0); }, "^Assertion" );
}

TEST(ASCIIFixedFromDouble, Zero) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_fixed_from_double(result, 0.0, 0, 7u);
    ASSERT_EQ(8u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0000000", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.0, 0, 7u);
    ASSERT_EQ(8u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0000000", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.0, 0, 1u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.0, 0, 1u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.0, 1u, 0);
    ASSERT_EQ(1u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.0, 1u, 0);
    ASSERT_EQ(1u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.0, 1u, 1u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.0, 1u, 1u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0", (char*)result);
}

TEST(ASCIIFixedFromDouble, FractionalOnly) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_fixed_from_double(result, 0.12345678, 0, 7u);
    ASSERT_EQ(8u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".1234568", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.12345678, 0, 7u);
    ASSERT_EQ(9u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.1234568", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 1.12345678, 0, 7u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("OF", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -1.12345678, 0, 7u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-OF", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.12345678, 0, 1u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".1", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.12345678, 0, 1u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.1", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.123, 0, 7u);
    ASSERT_EQ(8u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".1230000", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.123, 0, 7u);
    ASSERT_EQ(9u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.1230000", (char*)result);
}

TEST(ASCIIFixedFromDouble, IntegralOnly) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_fixed_from_double(result, 0.12345678, 1u, 0);
    ASSERT_EQ(1u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.12345678, 1u, 0);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 9.9, 1u, 0);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("OF", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -9.9, 1u, 0);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-OF", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.99, 1u, 0u);
    ASSERT_EQ(1u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("1", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.99, 1u, 0u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-1", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 123456.7890123, 6u,
                                                0u);
    ASSERT_EQ(6u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("123457", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -123456.7890123, 6u,
                                                0u);
    ASSERT_EQ(7u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-123457", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 1234567890123, 6u,
                                                0u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("OF", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -1234567890123, 6u,
                                                0u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-OF", (char*)result);
}

TEST(ASCIIFixedFromDouble, Mixed) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_fixed_from_double(result, 0.12345678, 1u, 1u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".1", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.12345678, 1u, 1u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.1", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 9.9, 1u, 1u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("9.9", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -9.9, 1u, 1u);
    ASSERT_EQ(4u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-9.9", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.99, 1u, 1u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("1.0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.99, 1u, 1u);
    ASSERT_EQ(4u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-1.0", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 123456.7890123, 6u,
                                                7u);
    ASSERT_EQ(14u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("123456.7890123", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -123456.7890123, 6u,
                                                7u);
    ASSERT_EQ(15u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-123456.7890123", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 1234567890123, 6u,
                                                7u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("OF", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -1234567890123, 6u,
                                                7u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-OF", (char*)result);
}

TEST(ASCIIFixedFromDouble, Rounding) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_fixed_from_double(result, 0.12345, 1u, 4u);
    ASSERT_EQ(5u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".1235", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.12345, 1u, 4u);
    ASSERT_EQ(6u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.1235", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.0001, 1u, 3u);
    ASSERT_EQ(4u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".000", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.0001, 1u, 3u);
    ASSERT_EQ(5u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.000", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, 0.0001, 1u, 4u);
    ASSERT_EQ(5u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ(".0001", (char*)result);

    result_length = fcs_ascii_fixed_from_double(result, -0.0001, 1u, 4u);
    ASSERT_EQ(6u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-.0001", (char*)result);
}

TEST(ASCIIFromInt32, Valid) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_from_int32(result, 1, 4u);
    ASSERT_EQ(1u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("1", (char*)result);

    result_length = fcs_ascii_from_int32(result, -1, 4u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-1", (char*)result);

    result_length = fcs_ascii_from_int32(result, 1092903, 10u);
    ASSERT_EQ(7u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("1092903", (char*)result);

    result_length = fcs_ascii_from_int32(result, -1092903, 10u);
    ASSERT_EQ(8u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-1092903", (char*)result);

    result_length = fcs_ascii_from_int32(result, INT32_MAX, 10u);
    ASSERT_EQ(10u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("2147483647", (char*)result);

    result_length = fcs_ascii_from_int32(result, INT32_MIN, 10u);
    ASSERT_EQ(11u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-2147483648", (char*)result);
}

TEST(ASCIIFromInt32, Overflow) {
    size_t result_length;
    uint8_t result[32];

    result_length = fcs_ascii_from_int32(result, 1092903, 6u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("OF", (char*)result);

    result_length = fcs_ascii_from_int32(result, -1092903, 6u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-OF", (char*)result);

    result_length = fcs_ascii_from_int32(result, INT32_MAX, 9u);
    ASSERT_EQ(2u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("OF", (char*)result);

    result_length = fcs_ascii_from_int32(result, INT32_MIN, 9u);
    ASSERT_EQ(3u, result_length);
    result[result_length] = 0;
    EXPECT_STREQ("-OF", (char*)result);
}
