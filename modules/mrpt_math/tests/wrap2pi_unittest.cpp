/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
