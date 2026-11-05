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

// Checks for transform_gaussian_{unscented,montecarlo,linear}() against
// closed-form propagation through linear and quadratic maps, and for the
// angle-wrapping support of the unscented transform.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;

namespace
{
using Vec3 = CVectorFixedDouble<3>;
using Vec2 = CVectorFixedDouble<2>;

// y = A*x + b
void linearMap(const Vec3& x, const double& /*unused*/, Vec2& y)
{
  y[0] = 1.0 * x[0] - 2.0 * x[1] + 0.5 * x[2] + 3.0;
  y[1] = 0.3 * x[0] + 1.5 * x[1] - 1.0 * x[2] - 1.0;
}
Eigen::Matrix<double, 2, 3> linearMapA()
{
  Eigen::Matrix<double, 2, 3> A;
  A << 1.0, -2.0, 0.5, 0.3, 1.5, -1.0;
  return A;
}

// y = [x0*x1, x0^2]
void quadraticMap(const Vec2& x, const double& /*unused*/, Vec2& y)
{
  y[0] = x[0] * x[1];
  y[1] = x[0] * x[0];
}

void identity2(const Vec2& x, const double& /*unused*/, Vec2& y) { y = x; }

CMatrixDouble33 testCov3()
{
  CMatrixDouble33 C;
  C(0, 0) = 0.4;
  C(1, 1) = 0.2;
  C(2, 2) = 0.3;
  C(0, 1) = C(1, 0) = 0.05;
  C(0, 2) = C(2, 0) = -0.1;
  C(1, 2) = C(2, 1) = 0.02;
  return C;
}

}  // namespace

// All three estimators are exact (MC: up to sampling noise) for linear maps.
TEST(transform_gaussian, LinearMap_AllMethods)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  Vec3 xm;
  xm[0] = 1.0;
  xm[1] = -2.0;
  xm[2] = 0.5;
  const CMatrixDouble33 C = testCov3();
  const Eigen::Matrix<double, 2, 3> A = linearMapA();
  const Eigen::Vector2d ym_exp = A * xm.asEigen() + Eigen::Vector2d(3.0, -1.0);
  const Eigen::Matrix2d yc_exp = A * C.asEigen() * A.transpose();
  const double dummy = 0;

  Vec2 ym;
  CMatrixDouble22 yc;
  transform_gaussian_unscented(xm, C, linearMap, dummy, ym, yc);
  EXPECT_NEAR((ym.asEigen() - ym_exp).norm(), 0.0, 1e-6);
  EXPECT_NEAR((yc.asEigen() - yc_exp).cwiseAbs().maxCoeff(), 0.0, 1e-6);

  // Non-default SUT parameters must give the same result for a linear map:
  transform_gaussian_unscented(xm, C, linearMap, dummy, ym, yc, nullptr, 1.0, 0.0, 2.0);
  EXPECT_NEAR((ym.asEigen() - ym_exp).norm(), 0.0, 1e-9);
  EXPECT_NEAR((yc.asEigen() - yc_exp).cwiseAbs().maxCoeff(), 0.0, 1e-9);

  Vec3 incr;
  incr.fill(1e-5);
  transform_gaussian_linear(xm, C, linearMap, dummy, ym, yc, incr);
  EXPECT_NEAR((ym.asEigen() - ym_exp).norm(), 0.0, 1e-9);
  EXPECT_NEAR((yc.asEigen() - yc_exp).cwiseAbs().maxCoeff(), 0.0, 1e-8);

  transform_gaussian_montecarlo(xm, C, linearMap, dummy, ym, yc, 200000);
  EXPECT_NEAR((ym.asEigen() - ym_exp).norm(), 0.0, 0.01);
  EXPECT_NEAR(
      (yc.asEigen() - yc_exp).cwiseAbs().maxCoeff(), 0.0, 0.02 * yc_exp.cwiseAbs().maxCoeff());
}

// The unscented transform captures the mean of a quadratic map exactly:
// E[x0*x1] = m0*m1 + C01, E[x0^2] = m0^2 + C00.
TEST(transform_gaussian, Unscented_QuadraticMapMean)
{
  Vec2 xm;
  xm[0] = 1.5;
  xm[1] = -0.5;
  CMatrixDouble22 C;
  C(0, 0) = 0.3;
  C(1, 1) = 0.2;
  C(0, 1) = C(1, 0) = 0.1;
  const double dummy = 0;
  Vec2 ym;
  CMatrixDouble22 yc;
  transform_gaussian_unscented(xm, C, quadraticMap, dummy, ym, yc, nullptr, 1.0, 0.0, 2.0);
  EXPECT_NEAR(ym[0], 1.5 * -0.5 + 0.1, 1e-9);
  EXPECT_NEAR(ym[1], 1.5 * 1.5 + 0.3, 1e-9);
  // Var(x0^2) = 4 m0^2 C00 + 2 C00^2 for a Gaussian (exact with beta=2, K=0
  // only for 1D; here just check it is close to the true value):
  EXPECT_NEAR(yc(1, 1), 4 * 1.5 * 1.5 * 0.3 + 2 * 0.3 * 0.3, 0.2);
}

// Identity map on [x, angle] with the angle flagged for wrapping: the output
// must equal the input Gaussian.
TEST(transform_gaussian, Unscented_WrapFlagsDoNotAffectLinearCoordinates)
{
  Vec2 xm;
  xm[0] = 0.0;
  xm[1] = 0.2;
  CMatrixDouble22 C;
  C(0, 0) = 9.0;
  C(1, 1) = 0.01;
  C(0, 1) = C(1, 0) = 0.15;
  const bool wrap[2] = {false, true};
  const double dummy = 0;
  Vec2 ym;
  CMatrixDouble22 yc;
  transform_gaussian_unscented(xm, C, identity2, dummy, ym, yc, wrap, 1.0, 0.0, 2.0);
  EXPECT_NEAR(ym[0], 0.0, 1e-9);
  EXPECT_NEAR(ym[1], 0.2, 1e-9);
  EXPECT_NEAR((yc.asEigen() - C.asEigen()).cwiseAbs().maxCoeff(), 0.0, 1e-9) << yc.asEigen();
}

// Angle-wrapped mean with the default SUT parameters (alpha=1e-3, which gives
// a central weight of about -1e6) when the sigma points straddle +-pi/2.
TEST(transform_gaussian, Unscented_WrappedMeanNearHalfPi)
{
  Vec2 xm;
  xm[0] = M_PI / 2 + 5e-5;
  xm[1] = 0.0;
  CMatrixDouble22 C;
  C(0, 0) = 0.01;
  C(1, 1) = 0.01;
  const bool wrap[2] = {true, false};
  const double dummy = 0;
  Vec2 ym;
  CMatrixDouble22 yc;
  transform_gaussian_unscented(xm, C, identity2, dummy, ym, yc, wrap);
  EXPECT_NEAR(ym[0], xm[0], 1e-6);
  EXPECT_NEAR(yc(0, 0), 0.01, 1e-5);
}
