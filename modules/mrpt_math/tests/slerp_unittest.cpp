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
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/slerp.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

// Some
TEST(SLERP_tests, correctShortestPath)
{
  CMatrixDouble44 HM, HMe;
  {  // Both poses at (0,0,0) angles:
    const TPose3D pose_a(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
    {
      TPose3D pose_interp;
      mrpt::math::slerp(pose_a, pose_b, 0, pose_interp);
      const TPose3D expected(0, 0, 0, 0, 0, 0);
      pose_interp.getHomogeneousMatrix(HM);
      expected.getHomogeneousMatrix(HMe);
      EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
          << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
    }
    {
      TPose3D pose_interp;
      mrpt::math::slerp(pose_a, pose_b, 1, pose_interp);
      const TPose3D expected(0, 0, 0, 0, 0, 0);
      pose_interp.getHomogeneousMatrix(HM);
      expected.getHomogeneousMatrix(HMe);
      EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
          << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
    }
    {
      TPose3D pose_interp;
      mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
      const TPose3D expected(0, 0, 0, 0, 0, 0);
      pose_interp.getHomogeneousMatrix(HM);
      expected.getHomogeneousMatrix(HMe);
      EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
          << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
    }
  }

  {  // Poses at yaw=+-179deg
    const TPose3D pose_a(0, 0, 0, 179.0_deg, 0.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, -179.0_deg, 0.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, -180.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }
  {  // Poses at yaw=-+179deg
    const TPose3D pose_a(0, 0, 0, -179.0_deg, 0.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, 179.0_deg, 0.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, -180.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }
  {  // Poses at yaw=-+179deg
    const TPose3D pose_a(0, 0, 0, -179.0_deg, 0.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, 179.0_deg, 0.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp_ypr(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, -180.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a.asString() << "\npose_b: " << pose_b.asString()
        << "\ninterp: " << pose_interp.asString() << "\n";
  }

  {  // Poses at yaw=+-40
    const TPose3D pose_a(0, 0, 0, 40.0_deg, 0.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, -40.0_deg, 0.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }
  {  // Poses at yaw=-+40
    const TPose3D pose_a(0, 0, 0, -40.0_deg, 0.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, 40.0_deg, 0.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }

  {  // Poses at pitch=+-40
    const TPose3D pose_a(0, 0, 0, 0.0_deg, 40.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, 0.0_deg, -40.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }
  {  // Poses at pitch=-+40
    const TPose3D pose_a(0, 0, 0, 0.0_deg, -40.0_deg, 0.0_deg);
    const TPose3D pose_b(0, 0, 0, 0.0_deg, 40.0_deg, 0.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }

  {  // Poses at roll=-+40
    const TPose3D pose_a(0, 0, 0, 0.0_deg, 0.0_deg, -40.0_deg);
    const TPose3D pose_b(0, 0, 0, 0.0_deg, 0.0_deg, 40.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }
  {  // Poses at roll=+-40
    const TPose3D pose_a(0, 0, 0, 0.0_deg, 0.0_deg, 40.0_deg);
    const TPose3D pose_b(0, 0, 0, 0.0_deg, 0.0_deg, -40.0_deg);
    TPose3D pose_interp;
    mrpt::math::slerp(pose_a, pose_b, 0.5, pose_interp);
    const TPose3D expected(0, 0, 0, 0.0_deg, 0, 0);
    pose_interp.getHomogeneousMatrix(HM);
    expected.getHomogeneousMatrix(HMe);
    EXPECT_NEAR(0, (HM - HMe).sum_abs(), 1e-4)
        << "pose_a: " << pose_a << "\npose_b: " << pose_b << "\ninterp: " << pose_interp << "\n";
  }
}

namespace
{
CQuaternionDouble quatFromYPR(double yaw, double pitch, double roll)
{
  CQuaternionDouble q;
  TPose3D(0, 0, 0, yaw, pitch, roll).getAsQuaternion(q);
  return q;
}

Eigen::Matrix3d rotOf(const CQuaternionDouble& q)
{
  CMatrixDouble33 R;
  q.rotationMatrixNoResize(R);
  return R.asEigen();
}
}  // namespace

TEST(SLERP_tests, followsGeodesicAndShortestPath)
{
  const auto q0 = quatFromYPR(0.2, -0.3, 0.5);
  const auto q1 = quatFromYPR(2.5, 0.4, -1.0);
  const Eigen::Matrix3d R0 = rotOf(q0);
  const Eigen::Matrix3d R1 = rotOf(q1);
  const Eigen::AngleAxisd d(R0.transpose() * R1);
  CQuaternionDouble q1neg = q1;
  for (int i = 0; i < 4; i++)
  {
    q1neg[i] = -q1[i];
  }
  for (double t : {0.0, 0.25, 0.5, 0.9, 1.0})
  {
    const Eigen::Matrix3d Rt = R0 * Eigen::AngleAxisd(t * d.angle(), d.axis()).toRotationMatrix();
    CQuaternionDouble q;
    mrpt::math::slerp(q0, q1, t, q);
    EXPECT_LT((rotOf(q) - Rt).array().abs().maxCoeff(), 1e-12) << "t=" << t;
    CQuaternionDouble qn;
    mrpt::math::slerp(q0, q1neg, t, qn);
    EXPECT_LT((rotOf(qn) - Rt).array().abs().maxCoeff(), 1e-12) << "t=" << t;
  }
}

TEST(SLERP_tests, outputIsUnitForNearbyInputs)
{
  const auto q0 = quatFromYPR(0.0, 0.0, 0.0);
  const auto q1 = quatFromYPR(0.0015, 0.0, 0.0);
  CQuaternionDouble q;
  mrpt::math::slerp(q0, q1, 0.5, q);
  EXPECT_NEAR(q.normSqr(), 1.0, 1e-12);
  EXPECT_LT((rotOf(q) - rotOf(quatFromYPR(0.00075, 0.0, 0.0))).array().abs().maxCoeff(), 1e-9);
}

TEST(SLERP_tests, slerpYPRNearGimbalLock)
{
  // Interpolated orientations close to pitch=90deg keep the exact rotation:
  const TPose3D a(0, 0, 0, 0.3, mrpt::DEG2RAD(89.6), 0.2);
  const TPose3D b(0, 0, 0, 0.3, mrpt::DEG2RAD(89.8), 0.2);
  TPose3D p;
  mrpt::math::slerp_ypr(a, b, 0.5, p);
  const TPose3D ref(0, 0, 0, 0.3, mrpt::DEG2RAD(89.7), 0.2);
  EXPECT_LT(
      (p.getRotationMatrix().asEigen() - ref.getRotationMatrix().asEigen())
          .array()
          .abs()
          .maxCoeff(),
      1e-12);
  TPose3D p2;
  mrpt::math::slerp(a, b, 0.5, p2);
  EXPECT_LT(
      (p2.getRotationMatrix().asEigen() - ref.getRotationMatrix().asEigen())
          .array()
          .abs()
          .maxCoeff(),
      1e-12);
}
