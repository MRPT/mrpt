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
