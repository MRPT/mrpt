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
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Dense>
#include <mrpt/math/interp_fit.hpp>

using namespace mrpt::math;

TEST(InterpFit, InterpolateWithinRange)
{
  std::vector<double> ys{0.0, 10.0, 20.0, 30.0};
  // 4 samples spanning [0,4): sample width Ax=1
  EXPECT_NEAR(mrpt::math::interpolate(0.5, ys, 0.0, 4.0), 5.0, 1e-6);
  EXPECT_NEAR(mrpt::math::interpolate(1.5, ys, 0.0, 4.0), 15.0, 1e-6);
}

TEST(InterpFit, InterpolateClampsBelowAndAbove)
{
  std::vector<double> ys{0.0, 10.0, 20.0, 30.0};
  EXPECT_NEAR(mrpt::math::interpolate(-5.0, ys, 0.0, 4.0), 0.0, 1e-6);
  EXPECT_NEAR(mrpt::math::interpolate(100.0, ys, 0.0, 4.0), 30.0, 1e-6);
}

TEST(InterpFit, SplineSegmentBeforeMidpoints)
{
  // x-samples must be non-decreasing; y = x^2 (smooth data)
  std::vector<double> xs{0, 1, 2, 3};
  std::vector<double> ys{0, 1, 4, 9};

  // t in [x0, x1): first branch of the piecewise cubic
  const double r = mrpt::math::spline(0.5, xs, ys, false);
  EXPECT_NEAR(r, 0.25, 0.3);
}

TEST(InterpFit, SplineSegmentMiddle)
{
  std::vector<double> xs{0, 1, 2, 3};
  std::vector<double> ys{0, 1, 4, 9};

  // t in [x1, x2): second branch
  const double r = mrpt::math::spline(1.5, xs, ys, false);
  EXPECT_NEAR(r, 2.25, 0.3);
}

TEST(InterpFit, SplineSegmentLast)
{
  std::vector<double> xs{0, 1, 2, 3};
  std::vector<double> ys{0, 1, 4, 9};

  // t in [x2, x3): third branch
  const double r = mrpt::math::spline(2.5, xs, ys, false);
  EXPECT_NEAR(r, 6.25, 0.5);
}

TEST(InterpFit, SplineWrap2PiHandlesJumps)
{
  // Angles that jump across the +/-pi boundary must be unwrapped internally
  // before fitting the spline, and the result wrapped back to ]-pi,pi].
  std::vector<double> xs{0, 1, 2, 3};
  std::vector<double> ys{3.0, -3.1, -3.0, 3.1};  // wraps around +/-pi

  const double r = mrpt::math::spline(1.5, xs, ys, true);
  EXPECT_GE(r, -M_PI);
  EXPECT_LE(r, M_PI);
}

TEST(InterpFit, LeastSquareLinearFitSinglePoint)
{
  CVectorDouble x(4);
  x[0] = 0;
  x[1] = 1;
  x[2] = 2;
  x[3] = 3;
  CVectorDouble y(4);
  y[0] = 1;
  y[1] = 3;
  y[2] = 5;
  y[3] = 7;  // y = 2x + 1

  const double r = mrpt::math::leastSquareLinearFit<double, CVectorDouble, 4>(1.5, x, y, false);
  EXPECT_NEAR(r, 4.0, 1e-6);
}

TEST(InterpFit, LeastSquareLinearFitVector)
{
  CVectorDouble x(4);
  x[0] = 0;
  x[1] = 1;
  x[2] = 2;
  x[3] = 3;
  CVectorDouble y(4);
  y[0] = 1;
  y[1] = 3;
  y[2] = 5;
  y[3] = 7;  // y = 2x + 1

  CVectorDouble ts(2);
  ts[0] = 0.5;
  ts[1] = 2.5;
  CVectorDouble outs;

  mrpt::math::leastSquareLinearFit<CVectorDouble, CVectorDouble, CVectorDouble, 4>(
      ts, outs, x, y, false);

  ASSERT_EQ(outs.size(), 2);
  EXPECT_NEAR(outs[0], 2.0, 1e-6);
  EXPECT_NEAR(outs[1], 6.0, 1e-6);
}

TEST(InterpFit, LeastSquareLinearFitVectorWrap2Pi)
{
  CVectorDouble x(3);
  x[0] = 0;
  x[1] = 1;
  x[2] = 2;
  CVectorDouble y(3);
  y[0] = 0;
  y[1] = 0.1;
  y[2] = 0.2;

  CVectorDouble ts(1);
  ts[0] = 1.0;
  CVectorDouble outs;

  mrpt::math::leastSquareLinearFit<CVectorDouble, CVectorDouble, CVectorDouble, 3>(
      ts, outs, x, y, true);

  ASSERT_EQ(outs.size(), 1);
  EXPECT_GE(outs[0], -M_PI);
  EXPECT_LE(outs[0], M_PI);
}

TEST(InterpFit, splineMatchesNaturalCubicSpline)
{
  CVectorDouble xs(4);
  CVectorDouble ys(4);
  xs[0] = 0.0;
  xs[1] = 1.0;
  xs[2] = 2.5;
  xs[3] = 3.0;
  ys[0] = 1.0;
  ys[1] = -1.0;
  ys[2] = 2.0;
  ys[3] = 0.5;
  const double h[3] = {1.0, 1.5, 0.5};
  // Natural spline: second derivatives z0=z3=0, solve for z1,z2
  Eigen::Matrix2d A;
  A << 2 * (h[0] + h[1]), h[1], h[1], 2 * (h[1] + h[2]);
  Eigen::Vector2d b;
  b << 6 * ((ys[2] - ys[1]) / h[1] - (ys[1] - ys[0]) / h[0]),
      6 * ((ys[3] - ys[2]) / h[2] - (ys[2] - ys[1]) / h[1]);
  const Eigen::Vector2d zi = A.inverse() * b;
  const double z[4] = {0.0, zi[0], zi[1], 0.0};
  const auto ref = [&](double t)
  {
    int i = 0;
    if (t >= xs[1])
    {
      i = 1;
    }
    if (t >= xs[2])
    {
      i = 2;
    }
    const double hi = h[i];
    return z[i + 1] * std::pow(t - xs[i], 3) / (6 * hi) +
           z[i] * std::pow(xs[i + 1] - t, 3) / (6 * hi) +
           (ys[i + 1] / hi - hi * z[i + 1] / 6) * (t - xs[i]) +
           (ys[i] / hi - hi * z[i] / 6) * (xs[i + 1] - t);
  };
  for (double t = 0.05; t < 3.0; t += 0.1)
  {
    EXPECT_NEAR(mrpt::math::spline(t, xs, ys), ref(t), 1e-12) << "t=" << t;
  }
  // A natural spline reproduces linear data exactly
  for (int i = 0; i < 4; i++)
  {
    ys[i] = 2.0 * xs[i] - 1.0;
  }
  EXPECT_NEAR(mrpt::math::spline(1.9, xs, ys), 2.8, 1e-12);
  EXPECT_NEAR(mrpt::math::spline(0.3, xs, ys), -0.4, 1e-12);
}

TEST(InterpFit, leastSquareLinearFitMatchesClosedForm)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  const int n = 20;
  CVectorDouble xs(n);
  CVectorDouble ys(n);
  for (int i = 0; i < n; i++)
  {
    xs[i] = 100.0 + i * 0.5;
    ys[i] = -3.0 + 0.7 * xs[i] + rng.drawGaussian1D(0.0, 0.1);
  }
  // Closed form simple linear regression
  const double mx = xs.asEigen().mean();
  const double my = ys.asEigen().mean();
  double sxy = 0;
  double sxx = 0;
  for (int i = 0; i < n; i++)
  {
    sxy += (xs[i] - mx) * (ys[i] - my);
    sxx += (xs[i] - mx) * (xs[i] - mx);
  }
  const double slope = sxy / sxx;
  const double icpt = my - slope * mx;
  for (const double t : {90.0, 105.0, 120.0})
  {
    EXPECT_NEAR(mrpt::math::leastSquareLinearFit(t, xs, ys), icpt + slope * t, 1e-9);
  }
  CVectorDouble ts(2);
  ts[0] = 95.0;
  ts[1] = 111.0;
  CVectorDouble outs;
  mrpt::math::leastSquareLinearFit(ts, outs, xs, ys);
  ASSERT_EQ(outs.size(), 2);
  EXPECT_NEAR(outs[0], icpt + slope * 95.0, 1e-9);
  EXPECT_NEAR(outs[1], icpt + slope * 111.0, 1e-9);
}

TEST(InterpFit, interpolateUsesCellConvention)
{
  // N samples over [x0,x1): sample i at x0 + i*(x1-x0)/N
  const std::vector<double> ys = {0.0, 10.0, 20.0, 30.0, 40.0};
  for (double x = 0.0; x < 4.0; x += 0.25)
  {
    EXPECT_NEAR(mrpt::math::interpolate(x, ys, 0.0, 5.0), 10.0 * x, 1e-12);
  }
  EXPECT_EQ(mrpt::math::interpolate(4.5, ys, 0.0, 5.0), 40.0);
  EXPECT_NEAR(mrpt::math::interpolate2points(0.25, 0.0, 1.0, 1.0, 3.0), 1.5, 1e-15);
  EXPECT_NEAR(mrpt::math::interpolate2points(2.0, 0.0, 1.0, 1.0, 3.0), 5.0, 1e-15);
}

// Regression: with wrap2pi, interpolate2points() and leastSquareLinearFit()
// only wrapped their output, so wrapped input angles across +-pi were
// interpolated along the long arc. spline() already unwrapped them.
TEST(InterpFit, Wrap2PiInputsAcrossPi)
{
  const auto theta = [](double t) { return M_PI - 0.3 + 0.2 * t; };
  mrpt::math::CVectorFixedDouble<4> xs;
  mrpt::math::CVectorFixedDouble<4> ys;
  for (int i = 0; i < 4; i++)
  {
    xs[i] = i;
    ys[i] = mrpt::math::wrapToPi(theta(i));
  }
  for (const double t : {1.2, 1.5, 1.9})
  {
    const double lin = mrpt::math::interpolate2points(t, xs[1], ys[1], xs[2], ys[2], true);
    EXPECT_NEAR(mrpt::math::wrapToPi(lin - theta(t)), 0.0, 1e-12) << "t=" << t;
    const double fit = mrpt::math::leastSquareLinearFit<double, decltype(xs), 4>(t, xs, ys, true);
    EXPECT_NEAR(mrpt::math::wrapToPi(fit - theta(t)), 0.0, 1e-12) << "t=" << t;
    const double spl = mrpt::math::spline(t, xs, ys, true);
    EXPECT_NEAR(mrpt::math::wrapToPi(spl - theta(t)), 0.0, 1e-12) << "t=" << t;
    for (const double v : {lin, fit, spl})
    {
      EXPECT_GE(v, -M_PI);
      EXPECT_LT(v, M_PI);
    }
  }
  // Vector version of the fit:
  mrpt::math::CVectorDouble ts(2);
  ts[0] = 1.5;
  ts[1] = 2.5;
  mrpt::math::CVectorDouble outs;
  mrpt::math::CVectorDouble xd(4);
  mrpt::math::CVectorDouble yd(4);
  for (int i = 0; i < 4; i++)
  {
    xd[i] = xs[i];
    yd[i] = ys[i];
  }
  mrpt::math::leastSquareLinearFit(ts, outs, xd, yd, true);
  ASSERT_EQ(outs.size(), 2);
  EXPECT_NEAR(mrpt::math::wrapToPi(outs[0] - theta(1.5)), 0.0, 1e-12);
  EXPECT_NEAR(mrpt::math::wrapToPi(outs[1] - theta(2.5)), 0.0, 1e-12);
}
