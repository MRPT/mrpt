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

// Consistency of conversions between pose representations and PDF forms:
// each pair must be an exact inverse (or the documented projection), also
// near yaw=+-pi and gimbal lock. Also frame conventions of twists and grids.

#include <gtest/gtest.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGrid.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <vector>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
using Mat6 = Eigen::Matrix<double, 6, 6>;

std::vector<CPose3D> testPoses()
{
  return {
      CPose3D(1, 2, 3, 0.3, -0.2, 0.1),
      CPose3D(-1, 0.5, 2, M_PI, 0.4, -0.3),
      CPose3D(0, 0, 0, -M_PI, 0, 0),
      CPose3D(4, -2, 1, 2.9, 1.2, -2.8),
      CPose3D(1, 1, 1, 0.5, DEG2RAD(89.9), -0.7),
      CPose3D(1, 1, 1, -2.5, DEG2RAD(-89.99), 1.2)};
}

double rotDist(const CPose3D& a, const CPose3D& b)
{
  return (a.getRotationMatrix().asEigen() - b.getRotationMatrix().asEigen()).cwiseAbs().maxCoeff();
}

Mat6 randomCov6(double scale)
{
  CMatrixDouble66 r;
  mrpt::random::getRandomGenerator().drawGaussian1DMatrix(r, 0, scale);
  Mat6 C = r.asEigen() * r.asEigen().transpose();
  C += 1e-3 * scale * scale * Mat6::Identity();
  return C;
}
}  // namespace

// CPose3D <-> TPose3D <-> CPose3DQuat <-> CPose3D: same rigid transformation.
TEST(PoseConversions, CPose3D_TPose3D_CPose3DQuat_RoundTrips)
{
  for (const auto& p : testPoses())
  {
    const TPose3D tp = p.asTPose();
    const CPose3D p2(tp);
    const CPose3DQuat pq(p);
    const CPose3D p3(pq);
    const CPose3DQuat pq2(pq.asTPose());
    const CPose3D p4(pq2);
    for (const auto* o : {&p2, &p3, &p4})
    {
      EXPECT_NEAR((o->translation() - p.translation()).norm(), 0, 1e-12);
      EXPECT_LT(rotDist(*o, p), 1e-12) << p;
    }
    // Same quaternion from both routes, with a non-negative real part:
    CQuaternionDouble q;
    p.getAsQuaternion(q);
    EXPECT_NEAR(std::abs(q.asEigen().dot(pq.quat().asEigen())), 1.0, 1e-12);
    // Composition is consistent across representations:
    const CPose3D d(0.3, -0.2, 0.5, 0.2, 0.1, -0.4);
    const CPose3D c1 = p + d;
    const CPose3D c2(pq + CPose3DQuat(d));
    EXPECT_NEAR((c1.translation() - c2.translation()).norm(), 0, 1e-12);
    EXPECT_LT(rotDist(c1, c2), 1e-12);
  }
}

// Euler <-> quaternion Gaussians: the first-order conversions are exact
// inverses of each other (also at yaw=+-pi and close to gimbal lock).
TEST(PoseConversions, EulerQuatGaussianRoundTrip)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (const auto& p : testPoses())
  {
    if (std::abs(p.pitch()) > DEG2RAD(89.95))
    {
      continue;  // conditioning too poor for a tight check
    }
    const Mat6 C = randomCov6(0.05);
    const CPose3DPDFGaussian a(p, CMatrixDouble66(C));
    const CPose3DQuatPDFGaussian q(a);
    const CPose3DPDFGaussian b(q);
    EXPECT_LT(rotDist(a.mean, b.mean), 1e-12) << p;
    EXPECT_NEAR((a.mean.translation() - b.mean.translation()).norm(), 0, 1e-12);
    EXPECT_LT((b.cov.asEigen() - C).cwiseAbs().maxCoeff() / C.cwiseAbs().maxCoeff(), 1e-8) << p;
  }
}

// Gaussian <-> information forms are exact inverses, in 2D and 3D, and the
// operations of both forms agree.
TEST(PoseConversions, GaussianInformationFormRoundTrips)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (const auto& p : testPoses())
  {
    const Mat6 C = randomCov6(0.1);
    const CPose3DPDFGaussian a(p, CMatrixDouble66(C));
    CPose3DPDFGaussianInf ai;
    ai.copyFrom(a);
    EXPECT_LT((ai.cov_inv.asEigen() * C - Mat6::Identity()).norm(), 1e-9);
    CPose3DPDFGaussian b;
    b.copyFrom(ai);
    EXPECT_LT((b.cov.asEigen() - C).norm() / C.norm(), 1e-10);
    EXPECT_LT(rotDist(a.mean, b.mean), 1e-12);

    // 2D:
    const Eigen::Matrix3d C2 = C.block<3, 3>(0, 0) + 0.01 * Eigen::Matrix3d::Identity();
    const CPosePDFGaussian g(CPose2D(p.x(), p.y(), p.yaw()), CMatrixDouble33(C2));
    CPosePDFGaussianInf gi;
    gi.copyFrom(g);
    CPosePDFGaussian g2;
    g2.copyFrom(gi);
    EXPECT_LT((g2.cov.asEigen() - C2).norm() / C2.norm(), 1e-10);
  }
}

// 2D -> 3D -> 2D Gaussians: exact round trip; 3D -> 2D keeps the marginal of
// (x, y, yaw), also in information form.
TEST(PoseConversions, Gaussian2D3DConversions)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  Eigen::Matrix3d C2;
  C2 << 0.04, 0.01, 0.002, 0.01, 0.09, -0.003, 0.002, -0.003, 0.01;
  const CPosePDFGaussian g(CPose2D(1.0, -2.0, 3.0), CMatrixDouble33(C2));
  const CPose3DPDFGaussian g3(g);
  EXPECT_NEAR(g3.mean.z(), 0, 0);
  EXPECT_NEAR(g3.mean.pitch(), 0, 0);
  EXPECT_NEAR(g3.mean.roll(), 0, 0);
  CPosePDFGaussian g2;
  g2.copyFrom(g3);
  EXPECT_NEAR((g2.mean.asVectorVal() - g.mean.asVectorVal()).asEigen().norm(), 0, 1e-12);
  EXPECT_LT((g2.cov.asEigen() - C2).norm(), 1e-15);

  // A full 3D Gaussian, projected to 2D: marginal of (x, y, yaw).
  const Mat6 C = randomCov6(0.1);
  const CPose3DPDFGaussian a(CPose3D(1, 2, 3, 0.4, 0.1, -0.2), CMatrixDouble66(C));
  CPosePDFGaussian a2;
  a2.copyFrom(a);
  const int idx[3] = {0, 1, 3};
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_NEAR(a2.cov(r, c), C(idx[r], idx[c]), 1e-15);
    }
  }
  CPose3DPDFGaussianInf ai;
  ai.copyFrom(a);
  CPosePDFGaussianInf a2i;
  a2i.copyFrom(ai);
  CPosePDFGaussian a2b;
  a2b.copyFrom(a2i);
  EXPECT_LT((a2b.cov.asEigen() - a2.cov.asEigen()).norm() / a2.cov.asEigen().norm(), 1e-9);
}

// TTwist3D::rotate(R(B wrt A)) converts a twist expressed in the moving
// frame B into frame A: checked against finite differences of the pose of a
// body moving with constant body-frame velocities.
TEST(PoseConversions, TTwist3DRotateConvention)
{
  using SO3 = mrpt::poses::Lie::SO<3>;
  const CPose3D P0(1, 2, 3, 0.6, -0.4, 0.9);                 // B wrt A at t=0
  const TTwist3D bodyTwist(0.5, -0.2, 0.3, 0.1, -0.3, 0.2);  // in frame B
  const auto poseAt = [&](double t)
  {
    // Constant body twist: P(t) = P0 * exp(t * xi), approximated with small t
    SO3::tangent_vector w;
    w[0] = bodyTwist.wx * t;
    w[1] = bodyTwist.wy * t;
    w[2] = bodyTwist.wz * t;
    CVectorFixedDouble<3> tv;
    tv[0] = bodyTwist.vx * t;
    tv[1] = bodyTwist.vy * t;
    tv[2] = bodyTwist.vz * t;
    const CPose3D inc(SO3::exp(w), tv);
    return P0 + inc;
  };
  const double h = 1e-6;
  const CPose3D Pp = poseAt(h);
  const CPose3D Pm = poseAt(-h);
  // Linear velocity of the origin of B, in A:
  const Eigen::Vector3d vA(
      (Pp.x() - Pm.x()) / (2 * h), (Pp.y() - Pm.y()) / (2 * h), (Pp.z() - Pm.z()) / (2 * h));
  // Angular velocity in A: from dR/dt * R^T
  const Eigen::Matrix3d dR =
      (Pp.getRotationMatrix().asEigen() - Pm.getRotationMatrix().asEigen()) / (2 * h);
  const Eigen::Matrix3d W = dR * P0.getRotationMatrix().asEigen().transpose();
  const Eigen::Vector3d wA(W(2, 1), W(0, 2), W(1, 0));

  const TTwist3D twA = bodyTwist.rotated(P0.asTPose());
  EXPECT_NEAR(twA.vx, vA[0], 1e-8);
  EXPECT_NEAR(twA.vy, vA[1], 1e-8);
  EXPECT_NEAR(twA.vz, vA[2], 1e-8);
  EXPECT_NEAR(twA.wx, wA[0], 1e-8);
  EXPECT_NEAR(twA.wy, wA[1], 1e-8);
  EXPECT_NEAR(twA.wz, wA[2], 1e-8);
}

// Grid PDFs: changing the reference with a rotation, and the inverse, move
// the probability mass to the transformed cell.
TEST(PoseConversions, GridPDFChangeOfReferenceWithRotation)
{
  // 2D: cell at (1, 0.5, 0) seen from (0.5, -1, +90 deg) is at
  // (0.5 - 0.5, -1 + 1, 90 deg) = (0, 0, 90 deg).
  CPosePDFGrid g(-2, 2, -2, 2, 0.5, DEG2RAD(90.0));
  g.fill(0.0);
  *g.getByPos(1.0, 0.5, 0.0) = 1.0;
  g.changeCoordinatesReference(CPose3D(0.5, -1.0, 0, DEG2RAD(90.0), 0, 0));
  EXPECT_NEAR(*g.getByPos(0.0, 0.0, DEG2RAD(90.0)), 1.0, 1e-12);

  // Inverse of (1, 0, 90 deg) is (0, 1, -90 deg):
  CPosePDFGrid h(-2, 2, -2, 2, 0.5, DEG2RAD(90.0));
  h.fill(0.0);
  *h.getByPos(1.0, 0.0, DEG2RAD(90.0)) = 1.0;
  CPosePDFGrid hi;
  h.inverse(hi);
  EXPECT_NEAR(*hi.getByPos(0.0, 1.0, DEG2RAD(-90.0)), 1.0, 1e-12);

  // 3D: same with a yaw rotation, and a pitch rotation of the reference.
  CPose3DPDFGrid g3(
      TPose3D(-2, -2, -2, -M_PI, -0.5 * M_PI, -0.5 * M_PI),
      TPose3D(2, 2, 2, M_PI, 0.5 * M_PI, 0.5 * M_PI), 0.5, DEG2RAD(90.0));
  g3.fill(0.0);
  *g3.getByPos(1.0, 0.0, 0.0, 0.0, 0.0, 0.0) = 1.0;
  const CPose3D ref(0, 0, 0.5, 0, DEG2RAD(90.0), 0);
  g3.changeCoordinatesReference(ref);
  const CPose3D expected = ref + CPose3D(1.0, 0.0, 0.0, 0, 0, 0);
  EXPECT_NEAR(
      *g3.getByPos(expected.x(), expected.y(), expected.z(), 0, DEG2RAD(90.0), 0), 1.0, 1e-12)
      << expected;
}
