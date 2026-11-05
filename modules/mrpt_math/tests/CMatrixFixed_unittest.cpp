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
#include <mrpt/math/CMatrixFixed.h>

#include <Eigen/Dense>
#include <cmath>

TEST(CMatrixFixed, CtorUninit)
{
  mrpt::math::CMatrixFixed<double, 2, 2> M(mrpt::math::UNINITIALIZED_MATRIX);
  // do nothing, just test that the ctor above compiles
  (void)M(0, 0);
}

TEST(CMatrixFixed, CtorAllZeros)
{
  mrpt::math::CMatrixFixed<double, 2, 2> M;
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) EXPECT_EQ(M(i, j), .0);
}

TEST(CMatrixFixed, Identity)
{
  mrpt::math::CMatrixFixed<double, 3, 3> M;
  M.setIdentity();
  for (int i = 0; i < 3; i++)
  {
    EXPECT_EQ(M(i, i), 1.0);
    // Also test access via data():
    EXPECT_EQ(M(i, i), M.data()[i + i * 3]);
  }

  // Check that access via data() is what we expect:
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) EXPECT_EQ(&M(r, c), &M.data()[c + r * 3]);
}

TEST(CMatrixFixed, asString)
{
  mrpt::math::CMatrixFixed<double, 2, 2> M;
  M.setIdentity();
  EXPECT_EQ(std::string("1 0\n0 1"), M.asString());
}

TEST(CMatrixFixed, GetSetEigen)
{
  {
    mrpt::math::CMatrixFixed<double, 3, 3> M;
    auto em = M.asEigen();
    em.setIdentity();
    for (int i = 0; i < 3; i++) EXPECT_EQ(M(i, i), 1.0);
  }
  {
    mrpt::math::CMatrixFixed<double, 3, 3> M;
    auto em = M.asEigen();
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
      {
        const auto n = ((i + 1) * 3) + (j * 1001);
        em(i, j) = n;
        EXPECT_NEAR(M(i, j), em(i, j), 1e-9) << "(i,j)=(" << i << "," << j << ")\n";
      }
  }
}

// Assigning an expression that references the destination matrix itself
TEST(CMatrixFixed, AssignAliasedExpression)
{
  mrpt::math::CMatrixDouble33 R;
  const double c = std::cos(0.7);
  const double s = std::sin(0.7);
  R(0, 0) = c;
  R(0, 1) = -s;
  R(1, 0) = s;
  R(1, 1) = c;
  R(2, 2) = 1;
  mrpt::math::CMatrixDouble33 M;
  M(0, 0) = 4;
  M(1, 1) = 2;
  M(2, 2) = 1;
  M(0, 1) = M(1, 0) = 0.5;
  M(1, 2) = M(2, 1) = -0.3;
  const Eigen::Matrix3d expected = R.asEigen() * M.asEigen() * R.asEigen().transpose();
  M = R.asEigen() * M.asEigen() * R.asEigen().transpose();
  EXPECT_NEAR((M.asEigen() - expected).cwiseAbs().maxCoeff(), 0, 1e-14);

  mrpt::math::CMatrixDouble33 T = R;
  T = T.asEigen().transpose();
  EXPECT_NEAR((T.asEigen() - R.asEigen().transpose()).cwiseAbs().maxCoeff(), 0, 0);
}
