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
