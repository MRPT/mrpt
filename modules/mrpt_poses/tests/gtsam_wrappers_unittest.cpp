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

// Only built if GTSAM is found.

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace
{
using M6 = Eigen::Matrix<double, 6, 6>;
using V6 = Eigen::Matrix<double, 6, 1>;

V6 ypr6(const mrpt::poses::CPose3D& p)
{
  V6 v;
  v << p.x(), p.y(), p.z(), p.yaw(), p.pitch(), p.roll();
  return v;
}

// d (x,y,z,yaw,pitch,roll)(mean * Expmap(xi)) / d xi
M6 numJacobianYprWrtLocal(const gtsam::Pose3& mean)
{
  M6 J;
  const double h = 1e-6;
  for (int k = 0; k < 6; k++)
  {
    gtsam::Vector6 xi = gtsam::Vector6::Zero();
    xi[k] = h;
    const mrpt::poses::CPose3D pp(mrpt::gtsam_wrappers::toTPose3D(mean.retract(xi)));
    const mrpt::poses::CPose3D pm(mrpt::gtsam_wrappers::toTPose3D(mean.retract(-xi)));
    V6 d = ypr6(pp) - ypr6(pm);
    for (int i = 3; i < 6; i++)
    {
      d[i] = mrpt::math::wrapToPi(d[i]);
    }
    J.col(k) = d / (2 * h);
  }
  return J;
}

M6 testCovariance()
{
  M6 A;
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      A(i, j) = std::sin(1.0 + 7.0 * i + 3.0 * j);
    }
  }
  return 0.01 * (A * A.transpose() + M6::Identity());
}

const std::vector<mrpt::poses::CPose3D> testPoses = {
    mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0),
    mrpt::poses::CPose3D(1, 2, 3, mrpt::DEG2RAD(90.0), 0, 0),
    mrpt::poses::CPose3D(1, 2, 3, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(5.0), mrpt::DEG2RAD(-3.0)),
    mrpt::poses::CPose3D(-5, 2, 1, mrpt::DEG2RAD(170.0), mrpt::DEG2RAD(40.0), mrpt::DEG2RAD(60.0)),
    mrpt::poses::CPose3D(
        -5, 2, 1, mrpt::DEG2RAD(-120.0), mrpt::DEG2RAD(80.0), mrpt::DEG2RAD(10.0))};

double relErr(const M6& a, const M6& ref) { return (a - ref).norm() / ref.norm(); }
}  // namespace

// The covariance of a GTSAM Pose3 (e.g. a marginal) is that of the body-frame
// increment xi of mean * Expmap(xi): check against finite differences.
TEST(gtsam_wrappers, to_mrpt_se3_pdf_vs_numerical)
{
  const M6 S = testCovariance();
  for (const auto& p : testPoses)
  {
    const gtsam::Pose3 P = mrpt::gtsam_wrappers::toPose3(p);
    const M6 J = numJacobianYprWrtLocal(P);
    const M6 expected = J * S * J.transpose();

    const auto pdf = mrpt::gtsam_wrappers::to_mrpt_se3_pdf(P, S);
    EXPECT_LT(relErr(pdf.cov.asEigen(), expected), 1e-8) << p;
    EXPECT_NEAR((pdf.mean.asVectorVal() - p.asVectorVal()).norm(), 0, 1e-12) << p;

    // The plain reordering is only right at the identity:
    if (p == mrpt::poses::CPose3D::Identity())
    {
      EXPECT_LT(relErr(mrpt::gtsam_wrappers::to_mrpt_se3_cov6(S).asEigen(), expected), 1e-8);
    }
  }
}

TEST(gtsam_wrappers, to_gtsam_se3_cov6_vs_numerical)
{
  const M6 C = testCovariance();
  for (const auto& p : testPoses)
  {
    mrpt::poses::CPose3DPDFGaussian pdf;
    pdf.mean = p;
    pdf.cov.asEigen() = C;

    gtsam::Pose3 P;
    gtsam::Matrix6 S;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(pdf, P, S);

    // d Logmap(mean^-1 * pose(ypr + d)) / d d
    M6 J;
    const double h = 1e-6;
    for (int k = 0; k < 6; k++)
    {
      V6 vp = ypr6(p);
      V6 vm = ypr6(p);
      vp[k] += h;
      vm[k] -= h;
      const auto Pp = mrpt::gtsam_wrappers::toPose3(
          mrpt::poses::CPose3D(vp[0], vp[1], vp[2], vp[3], vp[4], vp[5]));
      const auto Pm = mrpt::gtsam_wrappers::toPose3(
          mrpt::poses::CPose3D(vm[0], vm[1], vm[2], vm[3], vm[4], vm[5]));
      J.col(k) = (P.localCoordinates(Pp) - P.localCoordinates(Pm)) / (2 * h);
    }
    EXPECT_LT(relErr(S, J * C * J.transpose()), 1e-8) << p;

    // Round trip:
    const auto back = mrpt::gtsam_wrappers::to_mrpt_se3_pdf(P, S);
    EXPECT_LT(relErr(back.cov.asEigen(), C), 1e-12) << p;
  }
}
