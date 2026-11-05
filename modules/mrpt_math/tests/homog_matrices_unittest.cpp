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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/homog_matrices.h>

#include <Eigen/Dense>

using mrpt::math::CMatrixDouble;
using mrpt::math::CMatrixDouble33;
using mrpt::math::CMatrixDouble44;
using mrpt::math::TPose3D;

namespace
{
CMatrixDouble44 samplePoseMatrix()
{
  return TPose3D(1.0, -2.0, 0.5, 0.4, -0.3, 0.9).getHomogeneousMatrix();
}

void expectIsInverse(const CMatrixDouble44& M, const CMatrixDouble44& Minv)
{
  const Eigen::Matrix4d P = M.asEigen() * Minv.asEigen();
  EXPECT_LT((P - Eigen::Matrix4d::Identity()).array().abs().maxCoeff(), 1e-12);
  EXPECT_LT((Minv.asEigen() - M.asEigen().inverse()).array().abs().maxCoeff(), 1e-12);
}
}  // namespace

TEST(homog_matrices, inverseTwoArguments)
{
  const CMatrixDouble44 M = samplePoseMatrix();
  CMatrixDouble44 Minv;
  mrpt::math::homogeneousMatrixInverse(M, Minv);
  expectIsInverse(M, Minv);

  // Dynamic-size output:
  CMatrixDouble Mdyn(M);
  CMatrixDouble MinvDyn;
  mrpt::math::homogeneousMatrixInverse(Mdyn, MinvDyn);
  EXPECT_EQ(MinvDyn.rows(), 4);
  EXPECT_EQ(MinvDyn.cols(), 4);
  CMatrixDouble44 MinvFixed;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      MinvFixed(i, j) = MinvDyn(i, j);
    }
  }
  expectIsInverse(M, MinvFixed);
}

TEST(homog_matrices, inverseInPlace)
{
  const CMatrixDouble44 M = samplePoseMatrix();
  CMatrixDouble44 Minv = M;
  mrpt::math::homogeneousMatrixInverse(Minv);
  expectIsInverse(M, Minv);
}

TEST(homog_matrices, inverseRotationAndTranslation)
{
  const CMatrixDouble44 M = samplePoseMatrix();
  const CMatrixDouble33 R = M.blockCopy<3, 3>(0, 0);
  mrpt::math::CVectorFixedDouble<3> t;
  for (int i = 0; i < 3; i++)
  {
    t[i] = M(i, 3);
  }
  CMatrixDouble33 Rinv;
  mrpt::math::CVectorFixedDouble<3> tinv;
  mrpt::math::homogeneousMatrixInverse(R, t, Rinv, tinv);
  CMatrixDouble44 Minv = CMatrixDouble44::Identity();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Minv(i, j) = Rinv(i, j);
    }
    Minv(i, 3) = tinv[i];
  }
  expectIsInverse(M, Minv);
}
