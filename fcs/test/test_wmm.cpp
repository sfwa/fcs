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

#include <cstdlib>
#include <cmath>

extern "C" {
#include "util/util.h"
#include "util/3dmath.h"
#include "ahrs/wmm.h"
}

TEST(WMM, Initialisation) {
    fcs_wmm_init();
}

/*
The below test points are from:
http://www.ngdc.noaa.gov/geomag/WMM/data/WMM2010/WMM2010testvalues.pdf
*/

TEST(WMM, TestPoint0) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 80.0,
        (M_PI/180.0) * 0.0,
        0.0,
        2010.0,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(6649.5, field[0], 1e-1);
    EXPECT_NEAR(-714.6, field[1], 1e-1);
    EXPECT_NEAR(54346.2, field[2], 1e-1);
}

TEST(WMM, TestPoint1) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 0.0,
        (M_PI/180.0) * 120.0,
        0.0,
        2010.0,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(39428.8, field[0], 1e-1);
    EXPECT_NEAR(664.9, field[1], 1e-1);
    EXPECT_NEAR(-11683.8, field[2], 1e-1);
}

TEST(WMM, TestPoint2) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * -80.0,
        (M_PI/180.0) * 240.0,
        0.0,
        2010.0,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(5657.7, field[0], 1e-1);
    EXPECT_NEAR(15727.3, field[1], 1e-1);
    EXPECT_NEAR(-53407.5, field[2], 1e-1);
}

TEST(WMM, TestPoint3) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 80.0,
        (M_PI/180.0) * 0.0,
        100000.0,
        2010.0,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(6332.2, field[0], 1e-1);
    EXPECT_NEAR(-729.1, field[1], 1e-1);
    EXPECT_NEAR(52194.9, field[2], 1e-1);
}

TEST(WMM, TestPoint4) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 0.0,
        (M_PI/180.0) * 120.0,
        100000.0,
        2010.0,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(37452.0, field[0], 1e-1);
    EXPECT_NEAR(611.9, field[1], 1e-1);
    EXPECT_NEAR(-11180.8, field[2], 1e-1);
}

TEST(WMM, TestPoint5) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * -80.0,
        (M_PI/180.0) * 240.0,
        100000.0,
        2010.0,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(5484.3, field[0], 1e-1);
    EXPECT_NEAR(14762.8, field[1], 1e-1);
    EXPECT_NEAR(-50834.8, field[2], 1e-1);
}

TEST(WMM, TestPoint6) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 80.0,
        (M_PI/180.0) * 0.0,
        0.0,
        2012.5,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(6658.0, field[0], 1e-1);
    EXPECT_NEAR(-606.7, field[1], 1e-1);
    EXPECT_NEAR(54420.4, field[2], 1e-1);
}

TEST(WMM, TestPoint7) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 0.0,
        (M_PI/180.0) * 120.0,
        0.0,
        2012.5,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(39423.9, field[0], 1e-1);
    EXPECT_NEAR(608.1, field[1], 1e-1);
    EXPECT_NEAR(-11540.5, field[2], 1e-1);
}

TEST(WMM, TestPoint8) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * -80.0,
        (M_PI/180.0) * 240.0,
        0.0,
        2012.5,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(5713.6, field[0], 1e-1);
    EXPECT_NEAR(15731.8, field[1], 1e-1);
    EXPECT_NEAR(-53184.3, field[2], 1e-1);
}

TEST(WMM, TestPoint9) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 80.0,
        (M_PI/180.0) * 0.0,
        100000.0,
        2012.5,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(6340.9, field[0], 1e-1);
    EXPECT_NEAR(-625.1, field[1], 1e-1);
    EXPECT_NEAR(52261.9, field[2], 1e-1);
}

TEST(WMM, TestPoint10) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * 0.0,
        (M_PI/180.0) * 120.0,
        100000.0,
        2012.5,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(37448.1, field[0], 1e-1);
    EXPECT_NEAR(559.7, field[1], 1e-1);
    EXPECT_NEAR(-11044.2, field[2], 1e-1);
}

TEST(WMM, TestPoint11) {
    bool result;
    double field[3];
    result = fcs_wmm_calculate_field(
        (M_PI/180.0) * -80.0,
        (M_PI/180.0) * 240.0,
        100000.0,
        2012.5,
        field);
    ASSERT_EQ(true, result);

    EXPECT_NEAR(5535.5, field[0], 1e-1);
    EXPECT_NEAR(14765.4, field[1], 1e-1);
    EXPECT_NEAR(-50625.9, field[2], 1e-1);
}
