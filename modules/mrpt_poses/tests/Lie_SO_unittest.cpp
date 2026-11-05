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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <algorithm>
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

namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

// Closed-form inverse right Jacobian of SO(3).
Eigen::Matrix3d rightJacobianInverse(const Eigen::Vector3d& w)
{
  const double th = w.norm();
  const Eigen::Matrix3d W = skew(w);
  return Eigen::Matrix3d::Identity() + 0.5 * W +
         (1.0 / (th * th) - (1 + std::cos(th)) / (2 * th * std::sin(th))) * W * W;
}

CVectorFixedDouble<3> vec3(const Eigen::Vector3d& v)
{
  CVectorFixedDouble<3> r;
  for (int i = 0; i < 3; i++)
  {
    r[i] = v[i];
  }
  return r;
}

Eigen::Vector3d randomUnitVector()
{
  auto& rng = mrpt::random::getRandomGenerator();
  Eigen::Vector3d n(
      rng.drawGaussian1D_normalized(), rng.drawGaussian1D_normalized(),
      rng.drawGaussian1D_normalized());
  return n.normalized();
}
}  // namespace

TEST(LieSO3, ExpMatchesAngleAxis)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (double th : {0.0, 1e-9, 1e-7, 5e-7, 1e-3, 0.5, 2.0, 3.0, M_PI, 4.0, 6.0})
  {
    const Eigen::Vector3d n = randomUnitVector();
    const auto R = SO<3>::exp(vec3(th * n));
    const Eigen::Matrix3d Rref = Eigen::AngleAxisd(th, n).toRotationMatrix();
    EXPECT_LT((R.asEigen() - Rref).array().abs().maxCoeff(), 1e-12) << "th=" << th;
  }
}

TEST(LieSO3, LogIsPrincipalAndInvertsExp)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (double th : {1e-10, 1e-6, 1e-3, 0.5, 2.0, 3.0, M_PI - 1e-6})
  {
    for (int k = 0; k < 20; k++)
    {
      const Eigen::Vector3d n = randomUnitVector();
      const Eigen::Matrix3d R = Eigen::AngleAxisd(th, n).toRotationMatrix();
      const auto w = SO<3>::log(CMatrixDouble33(R));
      const Eigen::Vector3d we(w[0], w[1], w[2]);
      EXPECT_LT((we - th * n).norm(), 1e-9 * std::max(1.0, th)) << "th=" << th;
      EXPECT_LT((SO<3>::exp(w).asEigen() - R).array().abs().maxCoeff(), 1e-12);
    }
  }
  // Rotation vectors longer than pi map to the principal value:
  const Eigen::Vector3d n = Eigen::Vector3d(1, 2, -1).normalized();
  const auto w = SO<3>::log(SO<3>::exp(vec3(4.0 * n)));
  const Eigen::Vector3d we(w[0], w[1], w[2]);
  EXPECT_NEAR((we - (4.0 - 2 * M_PI) * n).norm(), 0, 1e-12);
}

TEST(LieSO3, LogNearPiKeepsTheAngle)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (double d : {1e-6, 1e-7, 5e-8, 1e-8, 1e-9, 1e-11})
  {
    for (int k = 0; k < 10; k++)
    {
      const Eigen::Vector3d n = randomUnitVector();
      const double th = M_PI - d;
      const Eigen::Matrix3d R = Eigen::AngleAxisd(th, n).toRotationMatrix();
      const auto w = SO<3>::log(CMatrixDouble33(R));
      const Eigen::Vector3d we(w[0], w[1], w[2]);
      EXPECT_NEAR(we.norm(), th, 1e-12) << "d=" << d;
      EXPECT_LT((we - th * n).norm(), 1e-9) << "d=" << d;
    }
  }
}

TEST(LieSO3, LogNearGimbalLockInvertsExp)
{
  using mrpt::poses::CPose3D;
  const CPose3D A = CPose3D::FromYawPitchRoll(0.7, -0.4, 1.1);
  const CPose3D B = (-A) + CPose3D::FromYawPitchRoll(0.3, M_PI / 2 - 1e-13, -0.8);
  const CMatrixDouble33 R = (A + B).getRotationMatrix();
  const auto Re = SO<3>::exp(SO<3>::log(R));
  EXPECT_LT((Re.asEigen() - R.asEigen()).array().abs().maxCoeff(), 1e-12);
}

TEST(LieSO3, JacobDexpeDeAtZeroVsFiniteDifferences)
{
  const auto J = SO<3>::jacob_dexpe_de(CVectorFixedDouble<3>::Zero());
  const double h = 1e-7;
  for (int k = 0; k < 3; k++)
  {
    CVectorFixedDouble<3> e = CVectorFixedDouble<3>::Zero();
    e[k] = h;
    const auto Rp = SO<3>::exp(e);
    e[k] = -h;
    const auto Rm = SO<3>::exp(e);
    for (int c = 0; c < 3; c++)
    {
      for (int r = 0; r < 3; r++)
      {
        EXPECT_NEAR(J(3 * c + r, k), (Rp(r, c) - Rm(r, c)) / (2 * h), 1e-8);
      }
    }
  }
}

// Chain rule on the manifold: J_log(R) * d vec(R*exp(e))/de == Jr^{-1}(log R),
// including the small-angle branch.
TEST(LieSO3, JacobDlogvDvOnManifold)
{
  using mrpt::poses::CPose3D;
  using mrpt::poses::Lie::SE;
  mrpt::random::getRandomGenerator().randomize(1234);
  for (double th : {1e-5, 1e-3, 4e-3, 0.01, 0.1, 0.5, 1.0, 2.0, 3.0, 3.1})
  {
    const Eigen::Vector3d w = th * randomUnitVector();
    const auto R = SO<3>::exp(vec3(w));
    const auto Jlog = SO<3>::jacob_dlogv_dv(R);
    const auto Jde = SE<3>::jacob_dDexpe_de(CPose3D(R, CVectorFixedDouble<3>()));
    const Eigen::Matrix3d Jchain = Jlog.asEigen() * Jde.asEigen().block<9, 3>(0, 3);
    EXPECT_LT((Jchain - rightJacobianInverse(w)).array().abs().maxCoeff(), 1e-9) << "th=" << th;
  }
}
