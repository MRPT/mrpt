/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>
#include <gtest/gtest.h>

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

TEST(bits_math, round2up)
{
	EXPECT_EQ(mrpt::round2up(4), 4);
	EXPECT_EQ(mrpt::round2up(5), 8);
	EXPECT_EQ(mrpt::round2up(200), 256);
}
