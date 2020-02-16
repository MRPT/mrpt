/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/bits_math.h>

TEST(bits_math, sign)
{
	EXPECT_EQ(mrpt::sign(-8), -1);
	EXPECT_EQ(mrpt::sign(8), 1);
	EXPECT_EQ(mrpt::sign(0), 1);
	EXPECT_EQ(mrpt::sign(8.0), 1);
	EXPECT_EQ(mrpt::sign(-8.0), -1);
	EXPECT_EQ(mrpt::sign(0.0), 1);
}

TEST(bits_math, keep_min)
{
	int min = 40;
	mrpt::keep_min(min, 30);
	EXPECT_EQ(min, 30);
	mrpt::keep_min(min, 80);
	EXPECT_EQ(min, 30);
	mrpt::keep_min(min, -80);
	EXPECT_EQ(min, -80);
}
TEST(bits_math, keep_max)
{
	int max = 40;
	mrpt::keep_max(max, 30);
	EXPECT_EQ(max, 40);
	mrpt::keep_max(max, 80);
	EXPECT_EQ(max, 80);
	mrpt::keep_max(max, -80);
	EXPECT_EQ(max, 80);
}

TEST(bits_math, round2up)
{
	EXPECT_EQ(mrpt::round2up(4), 4);
	EXPECT_EQ(mrpt::round2up(5), 8);
	EXPECT_EQ(mrpt::round2up(200), 256);
}

TEST(bits_math, hypot_fast)
{
	EXPECT_NEAR(mrpt::hypot_fast(3.0, 4.0), 5.0, 1e-6);
	EXPECT_NEAR(mrpt::hypot_fast(3.0f, 4.0f), 5.0f, 1e-6f);
}

TEST(bits_math, deg_rad)
{
	EXPECT_NEAR(mrpt::DEG2RAD(180.0L), static_cast<long double>(M_PI), 1e-6L);
	EXPECT_NEAR(mrpt::DEG2RAD(180.0), M_PI, 1e-6);
	EXPECT_NEAR(mrpt::DEG2RAD(180), M_PI, 1e-6);
	EXPECT_NEAR(mrpt::DEG2RAD(180.0f), d2f(M_PI), 1e-4f);
	{
		using namespace mrpt;  // _deg
		EXPECT_NEAR(180.0_deg, M_PI, 1e-6);
	}

	EXPECT_NEAR(
		mrpt::RAD2DEG(static_cast<long double>(M_PI)), 180.0L,
		static_cast<long double>(1e-6));
	EXPECT_NEAR(mrpt::RAD2DEG(M_PI), 180.0, 1e-6);
	EXPECT_NEAR(mrpt::RAD2DEG(d2f(M_PI)), 180.0f, 1e-4f);
}

TEST(bits_math, signWithZero)
{
	EXPECT_EQ(mrpt::signWithZero(89), 1);
	EXPECT_EQ(mrpt::signWithZero(89.0), 1);
	EXPECT_EQ(mrpt::signWithZero(0), 0);
	EXPECT_EQ(mrpt::signWithZero(0.0), 0);
	EXPECT_EQ(mrpt::signWithZero(-89), -1);
	EXPECT_EQ(mrpt::signWithZero(-89.0), -1);
}

TEST(bits_math, abs_diff)
{
	EXPECT_EQ(mrpt::abs_diff(10, 20), 10);
	EXPECT_EQ(mrpt::abs_diff(20, 10), 10);
	EXPECT_DOUBLE_EQ(mrpt::abs_diff(20.0, 10.0), 10.0);
	EXPECT_DOUBLE_EQ(mrpt::abs_diff(10.0, 20.0), 10.0);
}

TEST(bits_math, min3)
{
	EXPECT_EQ(mrpt::min3(1, 2, 3), 1);
	EXPECT_EQ(mrpt::min3(-1, -2, -3), -3);

	EXPECT_DOUBLE_EQ(mrpt::min3(1., 2., 3.), 1.);
	EXPECT_DOUBLE_EQ(mrpt::min3(-1., -2., -3.), -3.);
}

TEST(bits_math, max3)
{
	EXPECT_EQ(mrpt::max3(1, 2, 3), 3);
	EXPECT_EQ(mrpt::max3(-1, -2, -3), -1);

	EXPECT_DOUBLE_EQ(mrpt::max3(1., 2., 3.), 3.);
	EXPECT_DOUBLE_EQ(mrpt::max3(-1., -2., -3.), -1.);
}

TEST(bits_math, fix)
{
	EXPECT_EQ(mrpt::fix(1.2), 1);
	EXPECT_EQ(mrpt::fix(2.9), 2);
	EXPECT_EQ(mrpt::fix(-1.2), -1);
	EXPECT_EQ(mrpt::fix(-2.9), -2);
}

TEST(bits_math, saturate_val)
{
	EXPECT_DOUBLE_EQ(mrpt::saturate_val(11.2, -10.0, 10.0), 10.0);
	EXPECT_DOUBLE_EQ(mrpt::saturate_val(1.2, -10.0, 10.0), 1.2);
	EXPECT_DOUBLE_EQ(mrpt::saturate_val(-11.2, -10.0, 10.0), -10.0);
	EXPECT_DOUBLE_EQ(mrpt::saturate_val(-1.2, -10.0, 10.0), -1.2);
}

TEST(bits_math, saturate)
{
	double v = 11.2;
	mrpt::saturate(v, -10.0, 10.0);
	EXPECT_DOUBLE_EQ(v, 10.0);
	mrpt::saturate(v, -20.0, 20.0);
	EXPECT_DOUBLE_EQ(v, 10.0);
	mrpt::saturate(v, -1.0, 1.0);
	EXPECT_DOUBLE_EQ(v, 1.0);
}

TEST(bits_math, lowestPositive)
{
	EXPECT_EQ(mrpt::lowestPositive(1, 2), 1);
	EXPECT_EQ(mrpt::lowestPositive(2, 1), 1);
	EXPECT_EQ(mrpt::lowestPositive(-10, 3), 3);
	EXPECT_EQ(mrpt::lowestPositive(3, -1), 3);
}
