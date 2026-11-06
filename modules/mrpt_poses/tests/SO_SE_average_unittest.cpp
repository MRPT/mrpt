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
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/SO_SE_average.h>

#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <vector>

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

// Weighted SE(2) averages: weighted mean of the translations and weighted
// circular mean of the headings (across +-pi).
TEST(SE2_SE3_avrg, SE2_weightedAverage)
{
  const std::vector<std::pair<CPose2D, double>> data = {
      { CPose2D(1.0,  2.0,  M_PI - 0.1), 0.5},
      { CPose2D(2.0, -1.0, -M_PI + 0.2), 2.0},
      {CPose2D(-1.0,  0.5,  M_PI - 0.3), 1.5}
  };
  SE_average<2> avr;
  double sw = 0;
  double sx = 0;
  double sy = 0;
  double sc = 0;
  double ss = 0;
  for (const auto& [p, w] : data)
  {
    avr.append(p, w);
    sw += w;
    sx += w * p.x();
    sy += w * p.y();
    sc += w * std::cos(p.phi());
    ss += w * std::sin(p.phi());
  }
  CPose2D m;
  avr.get_average(m);
  EXPECT_NEAR(m.x(), sx / sw, 1e-12);
  EXPECT_NEAR(m.y(), sy / sw, 1e-12);
  EXPECT_NEAR(mrpt::math::wrapToPi(m.phi() - std::atan2(ss, sc)), 0.0, 1e-12);

  // An integer weight is equivalent to repeating the sample:
  SE_average<2> rep;
  rep.append(data[0].first);
  rep.append(data[1].first);
  rep.append(data[1].first);
  SE_average<2> wgt;
  wgt.append(data[0].first, 1.0);
  wgt.append(data[1].first, 2.0);
  CPose2D a;
  CPose2D b;
  rep.get_average(a);
  wgt.get_average(b);
  EXPECT_NEAR((a.asVectorVal() - b.asVectorVal()).asEigen().cwiseAbs().maxCoeff(), 0.0, 1e-12);
}

// Weighted SE(3) averages: weighted mean translation, and the rotation
// closest (Frobenius norm) to the weighted sum of rotation matrices.
TEST(SE2_SE3_avrg, SE3_weightedAverage)
{
  const std::vector<std::pair<CPose3D, double>> data = {
      { CPose3D(1.0,  2.0, 3.0, 0.3, -0.2,  0.1), 0.5},
      { CPose3D(2.0, -1.0, 0.5, 0.5,  0.1, -0.2), 2.0},
      {CPose3D(-1.0,  0.5, 1.0, 0.2,  0.0,  0.3), 1.5}
  };
  SE_average<3> avr;
  double sw = 0;
  Eigen::Vector3d st = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sR = Eigen::Matrix3d::Zero();
  for (const auto& [p, w] : data)
  {
    avr.append(p, w);
    sw += w;
    st += w * Eigen::Vector3d(p.x(), p.y(), p.z());
    sR += w * p.getRotationMatrix().asEigen();
  }
  CPose3D m;
  avr.get_average(m);
  EXPECT_NEAR((Eigen::Vector3d(m.x(), m.y(), m.z()) - st / sw).norm(), 0.0, 1e-12);

  // Independent projection onto SO(3): R = argmax tr(R^T * sR) (polar
  // decomposition), checked through its optimality: sR^T * R is symmetric
  // and positive definite.
  const Eigen::Matrix3d R = m.getRotationMatrix().asEigen();
  EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
  const Eigen::Matrix3d S = R.transpose() * sR;
  EXPECT_NEAR((S - S.transpose()).norm(), 0.0, 1e-12);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(0.5 * (S + S.transpose()));
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);

  // For nearby rotations, it approximates the weighted geodesic mean:
  // log(R) ~ sum_i w_i log(R_i) / sum_i w_i (same rotation axis here).
  SE_average<3> axis;
  axis.append(CPose3D(0, 0, 0, 0.10, 0, 0), 1.0);
  axis.append(CPose3D(0, 0, 0, 0.20, 0, 0), 3.0);
  CPose3D ma;
  axis.get_average(ma);
  EXPECT_NEAR(ma.yaw(), 0.175, 2e-4);
  EXPECT_NEAR(ma.pitch(), 0.0, 1e-12);
  EXPECT_NEAR(ma.roll(), 0.0, 1e-12);
}
