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

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <array>
#include <vector>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

template class mrpt::CTraitsTest<mrpt::poses::CPose3DQuat>;
template class mrpt::CTraitsTest<mrpt::poses::CPose3D>;
template class mrpt::CTraitsTest<mrpt::math::CQuaternionDouble>;

class QuaternionTests : public ::testing::Test
{
 protected:
  void SetUp() override {}
  void TearDown() override {}
  void test_gimbalLock(double YAW, double PITCH, double ROLL)
  {
    CQuaternionDouble q1, q1r;
    CPose3D p1(0, 0, 0, YAW, PITCH, ROLL);
    p1.getAsQuaternion(q1);
    CPose3D(q1, 0, 0, 0).getAsQuaternion(q1r);

    EXPECT_NEAR(
        0,
        std::abs((CPose3D(q1, 0, 0, 0).asVectorVal() - CPose3D(q1r, 0, 0, 0).asVectorVal()).sum()),
        1e-6);
  }

  void test_toYPRAndBack(double YAW, double PITCH, double ROLL)
  {
    CPose3D p1(0, 0, 0, YAW, PITCH, ROLL);
    CPose3DQuat q1(p1);
    CPose3D p2 = CPose3D(q1);

    EXPECT_NEAR(0, (p1.getRotationMatrix() - p2.getRotationMatrix()).array().abs().sum(), 1e-4)
        << "ypr->quat->ypr failed with:" << endl
        << "  p1:" << p1 << endl
        << "  q1:" << q1 << endl
        << "  p2:" << p2 << "\n";

    CPose3D p3(q1.quat(), q1[0], q1[1], q1[2]);
    EXPECT_NEAR(0, (p1.getRotationMatrix() - p3.getRotationMatrix()).array().abs().sum(), 1e-4)
        << "pose constructor from quat failed with:" << endl
        << "  p1:" << p1 << endl
        << "  q1:" << q1 << endl
        << "  p3:" << p3 << "\n";
  }

  void test_lnAndExpMatches(double yaw1, double pitch1, double roll1)
  {
    const CPose3D pp(0, 0, 0, yaw1, pitch1, roll1);
    CQuaternionDouble q1;
    pp.getAsQuaternion(q1);

    auto q1_ln = q1.ln<mrpt::math::CVectorDouble>();
    const CQuaternionDouble q2 = CQuaternionDouble::exp(q1_ln);

    // q2 should be == q1
    EXPECT_NEAR(0, (q1 - q2).array().abs().sum(), 1e-10) << "q1:\n"
                                                         << q1 << endl
                                                         << "q2:\n"
                                                         << q2 << endl
                                                         << "Error:\n"
                                                         << (q1 - q2) << "\n";
  }

  void test_ExpAndLnMatches(double v0, double v1, double v2)
  {
    CVectorFixedDouble<3> v;
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;

    const CQuaternionDouble q1 = CQuaternionDouble::exp(v);
    auto q1_ln = q1.ln<CVectorFixedDouble<3>>();

    // q1_ln should be == v
    EXPECT_NEAR(0, (q1_ln - v).array().abs().sum(), 1e-10) << "v:\n"
                                                           << v << endl
                                                           << "q1_ln:\n"
                                                           << q1_ln << endl
                                                           << "Error:\n"
                                                           << (q1_ln - v) << "\n";
  }
};

TEST_F(QuaternionTests, crossProduct)
{
  CQuaternionDouble q1, q2, q3;

  // q1 = CQuaternionDouble(1,2,3,4); q1.normalize();
  CPose3D p1(0, 0, 0, 10.0_deg, 30.0_deg, -20.0_deg);
  p1.getAsQuaternion(q1);

  CPose3D p2(0, 0, 0, 30.0_deg, -20.0_deg, 10.0_deg);
  p2.getAsQuaternion(q2);

  // q3 = q1 x q2
  q3.crossProduct(q1, q2);

  const CPose3D p3 = p1 + p2;

  EXPECT_NEAR(0, std::abs((p3.asVectorVal() - CPose3D(q3, 0, 0, 0).asVectorVal()).sum()), 1e-6)
      << "q1 = " << q1 << endl
      << "q1 as CPose3D = " << CPose3D(q1, 0, 0, 0) << endl
      << "q2 = " << q2 << endl
      << "q2 as CPose3D = " << CPose3D(q2, 0, 0, 0) << endl
      << "q3 = q1 * q2 = " << q3 << endl
      << "q3 as CPose3D = " << CPose3D(q3, 0, 0, 0) << endl
      << "Should be equal to p3 = p1 (+) p2 = " << p3 << "\n";
}

// Use special cases: gimbal lock:
TEST_F(QuaternionTests, gimbalLock)
{
  test_gimbalLock(20.0_deg, 90.0_deg, 0.0_deg);
  test_gimbalLock(20.0_deg, -90.0_deg, 0.0_deg);
}

TEST_F(QuaternionTests, ToYPRAndBack)
{
  test_toYPRAndBack(20.0_deg, 30.0_deg, 40.0_deg);
  test_toYPRAndBack(20.0_deg, 30.0_deg, 40.0_deg);
  test_toYPRAndBack(30.0_deg, 90.0_deg, 0.0_deg);
  test_toYPRAndBack(-30.0_deg, 90.0_deg, 0.0_deg);
  test_toYPRAndBack(-30.0_deg, 88.0_deg, 60.0_deg);
  test_toYPRAndBack(-30.0_deg, 10.0_deg, 60.0_deg);
}

TEST_F(QuaternionTests, LnAndExpMatches)
{
  const double list_test_YPR_angles_degrees[][3] = {
      {  0,   0,   0},
      { -1,   0,   0},
      {  1,   0,   0},
      {  0,  -1,   0},
      {  0,   1,   0},
      {  0,   0,  -1},
      {  0,   0,   1},
      { 40,   0,   0},
      {  0,  40,   0},
      {  0,   0,  40},
      {-40,   0,   0},
      {  0, -40,   0},
      {  0,   0, -40},
      {-30,  20,  50},
      {-30,  20, -50},
      { 30, -20, -50}
  };

  for (const auto& list_test_YPR_angles_degree : list_test_YPR_angles_degrees)
    test_lnAndExpMatches(
        DEG2RAD(list_test_YPR_angles_degree[0]), DEG2RAD(list_test_YPR_angles_degree[1]),
        DEG2RAD(list_test_YPR_angles_degree[2]));
}

TEST_F(QuaternionTests, ExpAndLnMatches)
{
  const double list_test_XYZ[][3] = {
      {    0,     0,     0},
      {   -1,     0,     0},
      {    1,     0,     0},
      {    0,    -1,     0},
      {    0,     1,     0},
      {    0,     0,    -1},
      {    0,     0,     1},
      { 1e-3,     0,     0},
      {    0,  1e-3,     0},
      {    0,     0,  1e-3},
      {-1e-3,     0,     0},
      {    0, -1e-3,     0},
      {    0,     0, -1e-3},
      { -0.1,   0.2,   0.3},
      { -0.1,  -0.2,   0.3},
      { -0.1,  -0.2,  -0.3},
      { -0.1,   0.2,  -0.3},
      {  0.1,   0.2,  -0.3},
      {  0.1,   0.2,   0.3}
  };

  for (const auto& i : list_test_XYZ) test_ExpAndLnMatches(i[0], i[1], i[2]);
}

TEST_F(QuaternionTests, ThrowOnNotNormalized)
{
  EXPECT_NO_THROW(mrpt::math::CQuaternion(1.0, 0.0, 0.0, 0.0));
  EXPECT_ANY_THROW(mrpt::math::CQuaternion(0.9, 0.0, 0.0, 0.0));
  EXPECT_ANY_THROW(mrpt::math::CQuaternion(1.1, 0.0, 0.0, 0.0));
  EXPECT_ANY_THROW(mrpt::math::CQuaternion(1.0, 0.1, 0.0, 0.0));
  EXPECT_ANY_THROW(mrpt::math::CQuaternion(1.0, 0.0, -0.1, 0.0));
  EXPECT_ANY_THROW(mrpt::math::CQuaternion(1.0, 0.0, 0.0, -0.1));
}

TEST_F(QuaternionTests, ensurePositiveRealPart)
{
  {
    auto q = mrpt::math::CQuaternion(1.0, 0.0, 0.0, 0.0);
    EXPECT_GE(q.r(), 0.0);
  }
  {
    auto q = mrpt::math::CQuaternion(-1.0, 0.0, 0.0, 0.0);
    EXPECT_GE(q.r(), 0.0);
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

Eigen::Vector4d hamilton(const CQuaternionDouble& a, const CQuaternionDouble& b)
{
  Eigen::Vector4d o;
  o[0] = a.r() * b.r() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
  o[1] = a.r() * b.x() + b.r() * a.x() + a.y() * b.z() - b.y() * a.z();
  o[2] = a.r() * b.y() + b.r() * a.y() + a.z() * b.x() - b.z() * a.x();
  o[3] = a.r() * b.z() + b.r() * a.z() + a.x() * b.y() - b.x() * a.y();
  return o;
}

Eigen::Matrix3d rotOf(const CQuaternionDouble& q)
{
  CMatrixDouble33 R;
  q.rotationMatrixNoResize(R);
  return R.asEigen();
}
}  // namespace

TEST(CQuaternion, fromRodriguesVectorSmallAngle)
{
  for (double th : {1e-7, 3e-7, 9e-7})
  {
    CQuaternionDouble q;
    q.fromRodriguesVector(std::vector<double>{0.0, th, 0.0});
    EXPECT_NEAR(q.r(), std::cos(th / 2), 1e-16) << "th=" << th;
    EXPECT_NEAR(q.normSqr(), 1.0, 1e-15) << "th=" << th;
  }
}

TEST(CQuaternion, lnOfExpSmallAngles)
{
  for (double th : {1e-8, 1e-7, 3e-7, 9e-7, 1e-6, 1e-4, 0.3})
  {
    const auto q = CQuaternionDouble::exp(std::vector<double>{th, 0.0, 0.0});
    const auto l = q.ln<std::vector<double>>();
    EXPECT_NEAR(l[0] / th, 1.0, 1e-9) << "th=" << th;
  }
}

TEST(CQuaternion, lnNegativeRealPartSmallVector)
{
  const double th = 2 * M_PI - 1e-8;
  const auto q = CQuaternionDouble::exp(std::vector<double>{th, 0.0, 0.0});
  const auto l = q.ln<std::vector<double>>();
  const auto q2 = CQuaternionDouble::exp(l);
  EXPECT_LT((rotOf(q) - rotOf(q2)).array().abs().maxCoeff(), 1e-12);
}

TEST(CQuaternion, rotationMatrixAndRotatePointAgree)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  auto& rng = mrpt::random::getRandomGenerator();
  for (int k = 0; k < 30; k++)
  {
    const auto q = quatFromYPR(
        rng.drawUniform(-M_PI, M_PI), rng.drawUniform(-1.5, 1.5), rng.drawUniform(-M_PI, M_PI));
    const Eigen::Matrix3d R = rotOf(q);
    EXPECT_LT((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 1e-14);
    EXPECT_NEAR(R.determinant(), 1.0, 1e-14);
    const Eigen::Vector3d l(0.3, -1.2, 2.5);
    Eigen::Vector3d g;
    q.rotatePoint(l[0], l[1], l[2], g[0], g[1], g[2]);
    EXPECT_LT((g - R * l).norm(), 1e-14);
    Eigen::Vector3d li;
    q.inverseRotatePoint(g[0], g[1], g[2], li[0], li[1], li[2]);
    EXPECT_LT((li - l).norm(), 1e-14);
    // Quaternion product matches matrix product:
    const auto q2 = quatFromYPR(0.4, -0.3, 1.2);
    CQuaternionDouble q12;
    q12.crossProduct(q, q2);
    EXPECT_LT((rotOf(q12) - R * rotOf(q2)).array().abs().maxCoeff(), 1e-14);
    EXPECT_GE(q12.r(), 0.0);
  }
}

// rotationJacobian() returns d(q_this x r)/dr (the left-multiplication matrix of q_this).
TEST(CQuaternion, rotationJacobianIsLeftMultiplicationMatrix)
{
  const auto q = quatFromYPR(0.3, 0.2, -0.4);
  const auto r = quatFromYPR(-0.5, 0.1, 0.7);
  CMatrixDouble44 J;
  q.rotationJacobian(J);
  EXPECT_LT((J.asEigen() * Eigen::Vector4d(r[0], r[1], r[2], r[3]) - hamilton(q, r)).norm(), 1e-15);
}

TEST(CQuaternion, normalizationJacobianVsFiniteDifferences)
{
  CQuaternionDouble q(mrpt::math::UNINITIALIZED_QUATERNION);
  q[0] = 0.9;
  q[1] = -0.3;
  q[2] = 0.25;
  q[3] = 0.4;
  CMatrixDouble44 J;
  q.normalizationJacobian(J);
  const double h = 1e-7;
  for (int k = 0; k < 4; k++)
  {
    Eigen::Vector4d a(q[0], q[1], q[2], q[3]);
    Eigen::Vector4d b = a;
    a[k] += h;
    b[k] -= h;
    const Eigen::Vector4d d = (a / a.norm() - b / b.norm()) / (2 * h);
    for (int r = 0; r < 4; r++)
    {
      EXPECT_NEAR(J(r, k), d[r], 1e-8);
    }
  }
}

TEST(CQuaternion, rpyAndJacobianVsFiniteDifferences)
{
  for (const auto& ypr : std::vector<std::array<double, 3>>{
           {     0.3,  0.4, -0.7},
           {    -2.9,  1.2,  2.5},
           {     1.0, -1.3, -3.0},
           {M_PI / 2,  0.0,  0.0}
  })
  {
    const auto q = quatFromYPR(ypr[0], ypr[1], ypr[2]);
    CMatrixDouble J;
    double r0 = 0;
    double p0 = 0;
    double y0 = 0;
    q.rpy_and_jacobian(r0, p0, y0, &J);
    EXPECT_NEAR(y0, ypr[0], 1e-12);
    EXPECT_NEAR(p0, ypr[1], 1e-12);
    EXPECT_NEAR(r0, ypr[2], 1e-12);
    const double h = 1e-7;
    for (int k = 0; k < 4; k++)
    {
      CQuaternionDouble qa = q;
      CQuaternionDouble qb = q;
      qa[k] += h;
      qb[k] -= h;
      double ra = 0;
      double pa = 0;
      double ya = 0;
      double rb = 0;
      double pb = 0;
      double yb = 0;
      qa.rpy(ra, pa, ya);
      qb.rpy(rb, pb, yb);
      // Row order of the Jacobian: yaw, pitch, roll.
      EXPECT_NEAR(J(0, k), (ya - yb) / (2 * h), 1e-6);
      EXPECT_NEAR(J(1, k), (pa - pb) / (2 * h), 1e-6);
      EXPECT_NEAR(J(2, k), (ra - rb) / (2 * h), 1e-6);
    }
  }
}

TEST(CQuaternion, yprQuaternionJacobianVsFiniteDifferences)
{
  const TPose3D p(0, 0, 0, 0.3, 0.4, -0.7);
  CQuaternionDouble q;
  CMatrixDouble43 J;
  p.getAsQuaternion(q, J);
  const double h = 1e-7;
  for (int k = 0; k < 3; k++)
  {
    TPose3D a = p;
    TPose3D b = p;
    a[3 + k] += h;
    b[3 + k] -= h;
    CQuaternionDouble qa;
    CQuaternionDouble qb;
    a.getAsQuaternion(qa);
    b.getAsQuaternion(qb);
    for (int r = 0; r < 4; r++)
    {
      EXPECT_NEAR(J(r, k), (qa[r] - qb[r]) / (2 * h), 1e-8);
    }
  }
  // Consistency with the rotation matrix:
  EXPECT_LT((rotOf(q) - p.getRotationMatrix().asEigen()).array().abs().maxCoeff(), 1e-15);
}

TEST(CQuaternion, jacobianRodriguesFromQuatVsFiniteDifferences)
{
  const auto q = quatFromYPR(0.3, 0.4, -0.7);
  ASSERT_GT(q.r(), 0.0);
  const auto J = q.jacobian_rodrigues_from_quat();
  // omega(q) = 2*acos(qr)/|qv| * qv, with all 4 components independent:
  auto omega = [](const Eigen::Vector4d& v)
  {
    const Eigen::Vector3d qv = v.tail<3>();
    return Eigen::Vector3d(2 * std::acos(v[0]) / qv.norm() * qv);
  };
  const double h = 1e-7;
  for (int k = 0; k < 4; k++)
  {
    Eigen::Vector4d a(q[0], q[1], q[2], q[3]);
    Eigen::Vector4d b = a;
    a[k] += h;
    b[k] -= h;
    const Eigen::Vector3d d = (omega(a) - omega(b)) / (2 * h);
    for (int r = 0; r < 3; r++)
    {
      EXPECT_NEAR(J(r, k), d[r], 1e-6);
    }
  }
}

namespace
{
// Exact yaw/pitch/roll of a (possibly non-unit) quaternion, via its rotation matrix:
Eigen::Vector3d exactYPR(const CQuaternionDouble& q0)
{
  CQuaternionDouble q = q0;
  q.normalize();
  CMatrixDouble33 R;
  q.rotationMatrixNoResize(R);
  double y = 0;
  double p = 0;
  double r = 0;
  TPose3D::SO3_to_yaw_pitch_roll(R, y, p, r);
  return {y, p, r};
}
}  // namespace

// Regression: rpy() snapped pitch to +-90 deg within ~0.36 deg of gimbal lock.
TEST(CQuaternion, rpyNearGimbalLockReproducesRotation)
{
  for (const double sgn : {1.0, -1.0})
  {
    for (const double offDeg : {0.5, 0.3, 0.1, 1e-2, 1e-4, 1e-6, 0.0})
    {
      const CPose3D P = CPose3D::FromYawPitchRoll(0.3, sgn * DEG2RAD(90.0 - offDeg), -0.5);
      CQuaternionDouble q;
      P.getAsQuaternion(q);
      double r = 0;
      double p = 0;
      double y = 0;
      q.rpy(r, p, y);
      const auto R = CPose3D::FromYawPitchRoll(y, p, r).getRotationMatrix();
      EXPECT_LT((R.asEigen() - P.getRotationMatrix().asEigen()).array().abs().maxCoeff(), 1e-12)
          << "pitch offset (deg): " << offDeg;
      // Non-unit quaternions give the same rotation:
      CQuaternionDouble q2 = q;
      q2 *= 1.01;
      double r2 = 0;
      double p2 = 0;
      double y2 = 0;
      q2.rpy(r2, p2, y2);
      const auto R2 = CPose3D::FromYawPitchRoll(y2, p2, r2).getRotationMatrix();
      EXPECT_LT((R2.asEigen() - P.getRotationMatrix().asEigen()).array().abs().maxCoeff(), 1e-12);
    }
  }
}

// Regression: within ~0.36 deg of gimbal lock, the Jacobian only had a yaw row.
TEST(CQuaternion, rpyJacobianNearGimbalLockVsFiniteDifferences)
{
  for (const double pitchDeg : {10.0, 80.0, 89.5, 89.7, 89.9, 89.99, -89.8})
  {
    const CPose3D P = CPose3D::FromYawPitchRoll(0.3, DEG2RAD(pitchDeg), -0.5);
    CQuaternionDouble q;
    P.getAsQuaternion(q);
    double r = 0;
    double p = 0;
    double y = 0;
    CMatrixFixed<double, 3, 4> J;
    q.rpy_and_jacobian(r, p, y, &J, false);
    CMatrixDouble44 dnorm;
    q.normalizationJacobian(dnorm);
    const Eigen::Matrix<double, 3, 4> Ja = J.asEigen() * dnorm.asEigen();
    // Central differences of the exact angles of the normalized quaternion:
    Eigen::Matrix<double, 3, 4> Jn;
    const double h = 1e-8;
    for (int k = 0; k < 4; k++)
    {
      CQuaternionDouble qp = q;
      CQuaternionDouble qm = q;
      qp[k] += h;
      qm[k] -= h;
      Eigen::Vector3d d = exactYPR(qp) - exactYPR(qm);
      for (int i = 0; i < 3; i++)
      {
        d[i] = wrapToPi(d[i]);
      }
      Jn.col(k) = d / (2 * h);
    }
    EXPECT_LT((Ja - Jn).norm() / Jn.norm(), 1e-6) << "pitch (deg): " << pitchDeg;
  }
}
