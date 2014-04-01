/*
Copyright (C) 2013 Ben Dyer

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
#include "util/util.h"
}

#include <cstdlib>

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
    EXPECT_DEATH({ fcs_crc32(NULL, 5, 0); }, "Assertion.*failed");
}

TEST(CRC32, TooShort) {
    EXPECT_DEATH(
        { uint8_t x[512]; fcs_crc32(x, 0, 0); }, "Assertion.*failed");
}
