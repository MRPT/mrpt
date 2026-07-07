/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/poses/Lie/SO.h>

#include <cmath>

using namespace mrpt::poses::Lie;
using namespace mrpt::math;

TEST(LieSO3, ExpLogRoundTrip)
{
  CVectorFixedDouble<3> x;
  x[0] = 0.1;
  x[1] = -0.2;
  x[2] = 0.05;

  const auto R = SO<3>::exp(x);
  const auto x2 = SO<3>::log(R);

  EXPECT_NEAR(x[0], x2[0], 1e-6);
  EXPECT_NEAR(x[1], x2[1], 1e-6);
  EXPECT_NEAR(x[2], x2[2], 1e-6);
}

TEST(LieSO3, LogNearIdentity)
{
  // n < 1e-7 branch:
  CVectorFixedDouble<3> zero;
  zero[0] = zero[1] = zero[2] = 0;
  const auto R = SO<3>::exp(zero);
  const auto x = SO<3>::log(R);
  EXPECT_NEAR(x[0], 0.0, 1e-9);
  EXPECT_NEAR(x[1], 0.0, 1e-9);
  EXPECT_NEAR(x[2], 0.0, 1e-9);
}

TEST(LieSO3, JacobDexpeDe)
{
  CVectorFixedDouble<3> x;
  x[0] = 0.1;
  x[1] = 0.2;
  x[2] = 0.3;
  const auto J = SO<3>::jacob_dexpe_de(x);
  EXPECT_EQ(J.rows(), 9);
  EXPECT_EQ(J.cols(), 3);
}

TEST(LieSO3, JacobDlogvDvNearIdentityAndGeneral)
{
  // d > 0.99999 branch (near-identity rotation):
  CVectorFixedDouble<3> tiny;
  tiny[0] = 1e-6;
  tiny[1] = 0;
  tiny[2] = 0;
  const auto Rtiny = SO<3>::exp(tiny);
  const auto Jtiny = SO<3>::jacob_dlogv_dv(Rtiny);
  EXPECT_EQ(Jtiny.rows(), 3);
  EXPECT_EQ(Jtiny.cols(), 9);

  // General branch:
  CVectorFixedDouble<3> x;
  x[0] = 0.3;
  x[1] = 0.4;
  x[2] = 0.5;
  const auto R = SO<3>::exp(x);
  const auto J = SO<3>::jacob_dlogv_dv(R);
  EXPECT_EQ(J.rows(), 3);
  EXPECT_EQ(J.cols(), 9);
}

TEST(LieSO3, VeeRmRtAndFromYPR)
{
  CVectorFixedDouble<3> x;
  x[0] = 0.1;
  x[1] = 0.2;
  x[2] = 0.3;
  const auto R = SO<3>::exp(x);
  const auto v = SO<3>::vee_RmRt(R);
  EXPECT_EQ(v.size(), 3u);

  const auto Rypr = SO<3>::fromYPR(0.1, 0.2, 0.3);
  EXPECT_NEAR(Rypr(0, 0), std::cos(0.1) * std::cos(0.2), 1e-9);
}

TEST(LieSO2, ExpLogAndJacobians)
{
  CVectorFixedDouble<1> x;
  x[0] = 0.5;
  const double R = SO<2>::exp(x);
  const auto x2 = SO<2>::log(R);
  EXPECT_NEAR(x[0], x2[0], 1e-9);

  const auto Jexp = SO<2>::jacob_dexpe_de(x);
  EXPECT_NEAR(Jexp(0, 0), 1.0, 1e-9);

  const auto Jlog = SO<2>::jacob_dlogv_dv(R);
  EXPECT_NEAR(Jlog(0, 0), 1.0, 1e-9);
}
