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
#include <mrpt/math/CLevenbergMarquardt.h>
#include <mrpt/math/CQuaternion.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

// Reuse code from example:
#include "../../../mrpt_examples_cpp/math_optimize_lm_example/LevMarqTest_impl.cpp"  // NOLINT

TEST(LevMarq, optimizeTest)
{
  TestLevMarq();
  EXPECT_LT(levmarq_final_error, 1e-2);
}

namespace
{
// Samples of y = 2.5 * exp(-1.3 t) + 0.5 (exact, no noise)
void expModelError(const CVectorDouble& x, const CVectorDouble& ts, CVectorDouble& err)
{
  err.resize(ts.size());
  for (int i = 0; i < ts.size(); i++)
  {
    const double t = ts[i];
    const double y = 2.5 * std::exp(-1.3 * t) + 0.5;
    err[i] = x[0] * std::exp(x[1] * t) + x[2] - y;
  }
}

CVectorDouble sampleTimes()
{
  CVectorDouble ts(25);
  for (int i = 0; i < ts.size(); i++)
  {
    ts[i] = 0.2 * i;
  }
  return ts;
}
}  // namespace

TEST(LevenbergMarquardt, convergesToExactParametersOfExponentialModel)
{
  CLevenbergMarquardt lm;
  CVectorDouble x0(3);
  x0[0] = 1.0;
  x0[1] = -0.5;
  x0[2] = 0.0;
  CVectorDouble incr(3);
  incr.fill(1e-7);
  CLevenbergMarquardt::TResultInfo info;
  CVectorDouble x;
  lm.execute(
      x, x0, expModelError, incr, sampleTimes(), info, mrpt::system::LVL_ERROR, 500, 1e-3, 1e-12,
      1e-12);
  EXPECT_NEAR(x[0], 2.5, 1e-6);
  EXPECT_NEAR(x[1], -1.3, 1e-6);
  EXPECT_NEAR(x[2], 0.5, 1e-6);
  EXPECT_LT(info.final_sqr_err, 1e-12);
  EXPECT_LT(info.final_sqr_err, info.initial_sqr_err);
  EXPECT_LT(info.iterations_executed, 500U);
}

TEST(LevenbergMarquardt, linearProblemMatchesNormalEquations)
{
  // err = A x - b
  Eigen::Matrix<double, 6, 2> A;
  A << 1, 0.5, 2, -1, 0, 3, 1, 1, -2, 0.25, 0.5, 0.5;
  Eigen::Matrix<double, 6, 1> b;
  b << 1, 2, -1, 0.5, 3, 0;
  const Eigen::Vector2d xls = (A.transpose() * A).ldlt().solve(A.transpose() * b);

  CVectorDouble dummy(1);
  const auto f = [&](const CVectorDouble& x, const CVectorDouble&, CVectorDouble& err)
  {
    err.resize(6);
    const Eigen::Matrix<double, 6, 1> e = A * Eigen::Vector2d(x[0], x[1]) - b;
    for (int i = 0; i < 6; i++)
    {
      err[i] = e[i];
    }
  };
  CLevenbergMarquardt lm;
  CVectorDouble x0(2);
  x0.fill(0.0);
  CVectorDouble incr(2);
  incr.fill(1e-6);
  CLevenbergMarquardt::TResultInfo info;
  CVectorDouble x;
  lm.execute(x, x0, f, incr, dummy, info, mrpt::system::LVL_ERROR);
  EXPECT_NEAR(x[0], xls[0], 1e-6);
  EXPECT_NEAR(x[1], xls[1], 1e-6);
  // H is the Gauss-Newton Hessian approximation J^T J
  const Eigen::Matrix2d JtJ = A.transpose() * A;
  ASSERT_EQ(info.H.rows(), 2);
  EXPECT_LT((info.H.asEigen() - JtJ).cwiseAbs().maxCoeff(), 1e-5);
  EXPECT_NEAR(info.final_sqr_err, (A * xls - b).squaredNorm(), 1e-9);
}

// The returned path starts at x0 and ends at the optimum
TEST(LevenbergMarquardt, pathStartsAtX0AndEndsAtOptimum)
{
  CLevenbergMarquardt lm;
  CVectorDouble x0(3);
  x0[0] = 1.0;
  x0[1] = -0.5;
  x0[2] = 0.0;
  CVectorDouble incr(3);
  incr.fill(1e-7);
  CLevenbergMarquardt::TResultInfo info;
  CVectorDouble x;
  lm.execute(x, x0, expModelError, incr, sampleTimes(), info, mrpt::system::LVL_ERROR);
  ASSERT_GT(info.path.rows(), 1);
  ASSERT_EQ(info.path.cols(), 4);
  EXPECT_EQ(info.path(0, 0), x0[0]);
  EXPECT_EQ(info.path(0, 1), x0[1]);
  EXPECT_EQ(info.path(0, 2), x0[2]);
  EXPECT_NEAR(info.path(0, 3), info.initial_sqr_err, 1e-12 * info.initial_sqr_err);
  const auto last = info.path.rows() - 1;
  EXPECT_EQ(info.path(last, 0), x[0]);
  EXPECT_EQ(info.path(last, 1), x[1]);
  EXPECT_EQ(info.path(last, 2), x[2]);
  EXPECT_EQ(info.path(last, 3), info.final_sqr_err);
}

// Regression: with x_increment_adder, the Jacobian was estimated by
// perturbing x additively, inconsistently with the steps actually applied,
// and the optimization stalled far from the optimum. Here x is a rotation
// vector, updated on the manifold as x <- log(exp(x) * exp(h)).
TEST(LevenbergMarquardt, onManifoldIncrementAdderConverges)
{
  using mrpt::math::CQuaternionDouble;
  using mrpt::math::CVectorDouble;
  using Vec3 = mrpt::math::CVectorFixedDouble<3>;

  const auto rotOf = [](const CVectorDouble& v)
  {
    Vec3 w;
    for (int i = 0; i < 3; i++)
    {
      w[i] = v[i];
    }
    const auto q = CQuaternionDouble::exp<Vec3>(w);
    Eigen::Matrix3d R;
    q.rotationMatrixNoResize(R);
    return R;
  };
  CVectorDouble xTrue(3);
  xTrue[0] = -1.2;
  xTrue[1] = -1.9;
  xTrue[2] = -1.8;
  const Eigen::Matrix3d Rgt = rotOf(xTrue);
  const std::vector<Eigen::Vector3d> pts = {
      { 1, 0,   0},
      { 0, 2,   0},
      { 0, 0,   3},
      { 1, 1,   1},
      {-1, 2, 0.5}
  };

  const auto f = [&](const CVectorDouble& x, const CVectorDouble&, CVectorDouble& err)
  {
    const Eigen::Matrix3d R = rotOf(x);
    err.resize(static_cast<int>(3 * pts.size()));
    for (size_t k = 0; k < pts.size(); k++)
    {
      const Eigen::Vector3d e = (R - Rgt) * pts[k];
      for (int i = 0; i < 3; i++)
      {
        err[static_cast<int>(3 * k) + i] = e[i];
      }
    }
  };
  const auto adder =
      [](CVectorDouble& xn, const CVectorDouble& xo, const CVectorDouble& h, const CVectorDouble&)
  {
    Vec3 wo;
    Vec3 wh;
    for (int i = 0; i < 3; i++)
    {
      wo[i] = xo[i];
      wh[i] = h[i];
    }
    CQuaternionDouble q;
    q.crossProduct(CQuaternionDouble::exp<Vec3>(wo), CQuaternionDouble::exp<Vec3>(wh));
    const auto w = q.ln<Vec3>();
    xn.resize(3);
    for (int i = 0; i < 3; i++)
    {
      xn[i] = w[i];
    }
  };

  CVectorDouble x0(3);
  x0.fill(0.1);
  CVectorDouble incr(3);
  incr.fill(1e-6);
  CVectorDouble params;
  CVectorDouble xOpt;
  mrpt::math::CLevenbergMarquardt lm;
  mrpt::math::CLevenbergMarquardt::TResultInfo info;
  lm.execute(
      xOpt, x0, f, incr, params, info, mrpt::system::LVL_ERROR, 100, 1e-3, 1e-12, 1e-12, false,
      adder);
  EXPECT_LT(info.final_sqr_err, 1e-16);
  EXPECT_LT((rotOf(xOpt) - Rgt).cwiseAbs().maxCoeff(), 1e-8);
  EXPECT_LT(info.iterations_executed, 30u);
}
