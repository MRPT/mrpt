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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/SO_SE_average.h>

#include <Eigen/Dense>
#include <cmath>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

void run_test_so2_avrg(const double* angs, const size_t N, const double ang_correct_avr)
{
  SO_average<2> so_avr;
  for (size_t i = 0; i < N; i++) so_avr.append(angs[i]);
  const double calc_avr = so_avr.get_average();
  EXPECT_NEAR(mrpt::math::wrapToPi(ang_correct_avr - calc_avr), .0, 1e-6);
}

TEST(SE2_SE3_avrg, SO2_average)
{
  // Simple tests:
  {
    const double angs[] = {.1};
    const double ang_correct_avr = .1;
    run_test_so2_avrg(angs, sizeof(angs) / sizeof(angs[0]), ang_correct_avr);
  }
  {
    // Undetermined average: documented to return 0
    const double angs[] = {.0, M_PI};
    const double ang_correct_avr = .0;
    run_test_so2_avrg(angs, sizeof(angs) / sizeof(angs[0]), ang_correct_avr);
  }
  {
    const double angs[] = {-0.75 * M_PI, 0.75 * M_PI};
    const double ang_correct_avr = 1.0 * M_PI;
    run_test_so2_avrg(angs, sizeof(angs) / sizeof(angs[0]), ang_correct_avr);
  }
  {
    const double angs[] = {-0.75 * M_PI, 0.75 * M_PI, 0.3 * M_PI};
    // const double angs_w[] = {1.0, 1.0, 0.1 };
    const double ang_correct_avr = 2.3668403111754515;
    run_test_so2_avrg(angs, sizeof(angs) / sizeof(angs[0]), ang_correct_avr);
  }
  // Test launching an exception when there is no data:
  {
    const double dummy[] = {0.};
    try
    {
      run_test_so2_avrg(dummy, 0, 0);
      GTEST_FAIL() << "An exception should have been raised before this point!!";
    }
    catch (std::exception&)
    {
      // This error is expected, it's OK.
    }
  }
}

void run_test_so3_avrg(
    const double* angs, const size_t N, const mrpt::math::CMatrixDouble33& correct_avr)
{
  SO_average<3> so_avr;
  for (size_t i = 0; i < N; i++)
  {
    mrpt::poses::CPose3D rot(0, 0, 0, angs[3 * i + 0], angs[3 * i + 1], angs[3 * i + 2]);
    so_avr.append(rot.getRotationMatrix());
  }
  const auto calc_avr = so_avr.get_average();
  EXPECT_NEAR((correct_avr - calc_avr).array().abs().sum(), .0, 1e-5);
}

TEST(SE2_SE3_avrg, SO3_average)
{
  // Simple tests:
  {
    const double angs[] = {.0, .0, .0};
    const auto correct_avr = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0).getRotationMatrix();
    run_test_so3_avrg(angs, sizeof(angs) / (3 * sizeof(angs[0])), correct_avr);
  }
  {
    const double angs[] = {-.75 * M_PI, .0, .0, .75 * M_PI, .0, .0};
    const auto correct_avr = mrpt::poses::CPose3D(0, 0, 0, M_PI, 0, 0).getRotationMatrix();
    run_test_so3_avrg(angs, sizeof(angs) / (3 * sizeof(angs[0])), correct_avr);
  }
  {
    const double angs[] = {.0, -0.2, .0, .0, 0.2, .0};
    const auto correct_avr = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0).getRotationMatrix();
    run_test_so3_avrg(angs, sizeof(angs) / (3 * sizeof(angs[0])), correct_avr);
  }
  {
    const double angs[] = {.0, .0, .3, .0, .0, -.3};
    const auto correct_avr = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0).getRotationMatrix();
    run_test_so3_avrg(angs, sizeof(angs) / (3 * sizeof(angs[0])), correct_avr);
  }
}

TEST(SE2_SE3_avrg, SO2_undeterminedThrowsIfEnabled)
{
  SO_average<2> a;
  a.enable_exception_on_undeterminate = true;
  a.append(0.0);
  a.append(M_PI);
  EXPECT_ANY_THROW(a.get_average());
  a.enable_exception_on_undeterminate = false;
  EXPECT_NEAR(a.get_average(), 0.0, 1e-12);
}

TEST(SE2_SE3_avrg, SO2_weightedAverage)
{
  SO_average<2> a;
  a.append(M_PI - 0.1, 1.0);
  a.append(-M_PI + 0.1, 3.0);
  // Weighted circular mean, computed independently:
  const double x = std::cos(M_PI - 0.1) + 3 * std::cos(-M_PI + 0.1);
  const double y = std::sin(M_PI - 0.1) + 3 * std::sin(-M_PI + 0.1);
  EXPECT_NEAR(mrpt::math::wrapToPi(a.get_average() - std::atan2(y, x)), 0.0, 1e-12);
}

TEST(SE2_SE3_avrg, SO3_undetermined)
{
  SO_average<3> a;
  a.append(CPose3D::FromYawPitchRoll(0, 0, 0).getRotationMatrix());
  a.append(CPose3D::FromYawPitchRoll(M_PI, 0, 0).getRotationMatrix());
  // Documented to return the identity:
  const Eigen::Matrix3d R = a.get_average().asEigen();
  EXPECT_LT((R - Eigen::Matrix3d::Identity()).array().abs().maxCoeff(), 1e-12);
  a.enable_exception_on_undeterminate = true;
  EXPECT_ANY_THROW(a.get_average());
}

TEST(SE2_SE3_avrg, SO3_rankTwoInputHasUniqueAverage)
{
  // Rx(+90deg) and Rx(-90deg) sum to diag(2,0,0) (undetermined average);
  // adding Ry(+-90deg) gives diag(2,2,0), whose closest rotation is the
  // identity.
  SO_average<3> a;
  a.enable_exception_on_undeterminate = true;
  a.append(CPose3D::FromYawPitchRoll(0, 0, M_PI / 2).getRotationMatrix());
  a.append(CPose3D::FromYawPitchRoll(0, 0, -M_PI / 2).getRotationMatrix());
  EXPECT_ANY_THROW(a.get_average());
  a.append(CPose3D::FromYawPitchRoll(0, M_PI / 2, 0).getRotationMatrix());
  a.append(CPose3D::FromYawPitchRoll(0, -M_PI / 2, 0).getRotationMatrix());
  const Eigen::Matrix3d R = a.get_average().asEigen();
  EXPECT_LT((R - Eigen::Matrix3d::Identity()).array().abs().maxCoeff(), 1e-12);
}

TEST(SE2_SE3_avrg, SE3_averageOfSymmetricSet)
{
  // Poses symmetric (to first order) about a mean pose average back to it:
  const CPose3D mean(1.0, -2.0, 0.5, 0.7, -0.3, 1.2);
  SE_average<3> avg;
  for (int k = 0; k < 3; k++)
  {
    for (double s : {-1.0, 1.0})
    {
      double v[6] = {0, 0, 0, 0, 0, 0};
      v[k] = 0.3 * s;
      v[3 + k] = 0.25 * s;
      avg.append(mean + CPose3D(v[0], v[1], v[2], v[3], v[4], v[5]), 1.0);
    }
  }
  CPose3D out;
  avg.get_average(out);
  const Eigen::Matrix3d Ro = out.getRotationMatrix().asEigen();
  EXPECT_LT((Ro.transpose() * Ro - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  const Eigen::AngleAxisd d(mean.getRotationMatrix().asEigen().transpose() * Ro);
  EXPECT_LT(std::abs(d.angle()), 1e-2);
  // Arithmetic mean of the translations:
  double sx = 0;
  for (int k = 0; k < 3; k++)
  {
    for (double s : {-1.0, 1.0})
    {
      double v[6] = {0, 0, 0, 0, 0, 0};
      v[k] = 0.3 * s;
      v[3 + k] = 0.25 * s;
      sx += (mean + CPose3D(v[0], v[1], v[2], v[3], v[4], v[5])).x();
    }
  }
  EXPECT_NEAR(out.x(), sx / 6, 1e-12);
}
