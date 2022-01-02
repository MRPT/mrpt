/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt;
using namespace std;

TEST(Wrap2PI_tests, angDistance)
{
	using mrpt::math::angDistance;

	EXPECT_NEAR(angDistance(0.0, 1.0), 1.0, 1e-10);
	EXPECT_NEAR(angDistance(1.0, 1.0), 0.0, 1e-10);
	EXPECT_NEAR(angDistance(1.0, 0.0), -1.0, 1e-10);

	EXPECT_NEAR(angDistance(-(M_PI - 0.1), (M_PI - 0.1)), -0.2, 1e-6);
	EXPECT_NEAR(angDistance((M_PI - 0.1), -(M_PI - 0.1)), +0.2, 1e-6);
}
