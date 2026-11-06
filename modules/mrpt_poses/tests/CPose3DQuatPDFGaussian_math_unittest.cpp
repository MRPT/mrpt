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

// Math checks for CPose3DQuatPDF / CPose3DQuatPDFGaussian against
// finite-difference Jacobians of the actual pose operations.

#include <gtest/gtest.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <functional>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat7 = Eigen::Matrix<double, 7, 7>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

Vec7 toVec(const CPose3DQuat& p)
{
  Vec7 v;
  for (int i = 0; i < 7; i++)
  {
    v[i] = p[i];
  }
  return v;
}

// Builds a pose from a 7-vector, normalizing the quaternion by its norm only
// (without forcing a positive real part).
CPose3DQuat toPose(const Vec7& v)
{
  const double n = v.tail<4>().norm();
  CQuaternionDouble q(UNINITIALIZED_QUATERNION);
  for (int i = 0; i < 4; i++)
  {
    q[i] = v[3 + i] / n;
  }
  CPose3DQuat p(UNINITIALIZED_QUATERNION);
  p.x(v[0]);
  p.y(v[1]);
  p.z(v[2]);
  p.quat() = q;
  return p;
}

Mat7 numJacobian(const std::function<Vec7(const Vec7&)>& f, const Vec7& x0, double h = 1e-7)
{
  Mat7 J;
  for (int j = 0; j < 7; j++)
  {
    Vec7 xp = x0;
    Vec7 xm = x0;
    xp[j] += h;
    xm[j] -= h;
    J.col(j) = (f(xp) - f(xm)) / (2 * h);
  }
  return J;
}

double maxAbsDiff(const Mat7& a, const Mat7& b) { return (a - b).cwiseAbs().maxCoeff(); }

void checkQuatCompositionJacobians(const CPose3D& x6, const CPose3D& u6)
{
  const CPose3DQuat x(x6);
  const CPose3DQuat u(u6);
  CMatrixDouble77 df_dx;
  CMatrixDouble77 df_du;
  CPose3DQuat xu(UNINITIALIZED_QUATERNION);
  CPose3DQuatPDF::jacobiansPoseComposition(x, u, df_dx, df_du, &xu);
  EXPECT_NEAR((toVec(xu) - toVec(x + u)).norm(), 0.0, 1e-12);

  const auto fx = [&](const Vec7& v) -> Vec7 { return toVec(toPose(v) + u); };
  const auto fu = [&](const Vec7& v) -> Vec7 { return toVec(x + toPose(v)); };
  const Mat7 Jx = numJacobian(fx, toVec(x));
  const Mat7 Ju = numJacobian(fu, toVec(u));
  EXPECT_NEAR(maxAbsDiff(df_dx.asEigen(), Jx), 0.0, 1e-6) << "x=" << x6 << " u=" << u6 << "\n"
                                                          << df_dx.asEigen() << "\n\n"
                                                          << Jx;
  EXPECT_NEAR(maxAbsDiff(df_du.asEigen(), Ju), 0.0, 1e-6) << "x=" << x6 << " u=" << u6 << "\n"
                                                          << df_du.asEigen() << "\n\n"
                                                          << Ju;
}

CPose3DQuatPDFGaussian randomQuatPDF(const CPose3D& mean, double scale)
{
  CMatrixDouble66 r;
  mrpt::random::getRandomGenerator().drawGaussian1DMatrix(r, 0, scale);
  Mat6 C = r.asEigen() * r.asEigen().transpose();
  C += 1e-3 * scale * scale * Mat6::Identity();
  return CPose3DQuatPDFGaussian(CPose3DPDFGaussian(mean, CMatrixDouble66(C)));
}

}  // namespace

// 7x7 composition Jacobians for moderate rotations.
TEST(CPose3DQuatPDFMath, JacobiansPoseComposition_FiniteDifferences)
{
  checkQuatCompositionJacobians(
      CPose3D(1, 2, 3, 0.3, 0.2, -0.1), CPose3D(-0.5, 1.5, 0.2, 0.6, -0.3, 0.4));
  checkQuatCompositionJacobians(
      CPose3D(0, 1, 0, 1.2, -0.4, 0.3), CPose3D(4.0, -1.0, 2.0, 0.5, 0.7, -0.6));
}

// Large rotations: crossProduct() returns the product with a non-negative
// real part, and the Jacobians must account for that sign flip.
TEST(CPose3DQuatPDFMath, JacobiansPoseComposition_LargeRotations)
{
  checkQuatCompositionJacobians(CPose3D(0, 0, 0, 2.0, 0, 0), CPose3D(1, 0, 0, 2.0, 0, 0));
  checkQuatCompositionJacobians(
      CPose3D(1, 2, 3, 2.9, 0.2, -0.1), CPose3D(-0.5, 1.5, 0.2, 0.6, -0.3, 0.4));
}

// inverse(): covariance vs numeric Jacobian of the inverse.
TEST(CPose3DQuatPDFGaussianMath, Inverse_FiniteDifferences)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (const auto& m : {CPose3D(1, 2, 3, 0.3, 0.2, -0.1), CPose3D(-4, 1, 0.5, 2.5, -0.9, 2.0)})
  {
    const CPose3DQuatPDFGaussian p = randomQuatPDF(m, 0.05);
    const CPose3DQuatPDFGaussian pInv = -p;

    const auto f = [](const Vec7& v) -> Vec7
    {
      const CPose3DQuat q = toPose(v);
      CPose3DQuat r(UNINITIALIZED_QUATERNION);
      double lx = 0;
      double ly = 0;
      double lz = 0;
      q.inverseComposePoint(0, 0, 0, lx, ly, lz);
      r.x(lx);
      r.y(ly);
      r.z(lz);
      q.quat().conj(r.quat());
      return toVec(r);
    };
    const Mat7 J = numJacobian(f, toVec(p.mean));
    EXPECT_NEAR(maxAbsDiff(pInv.cov.asEigen(), J * p.cov.asEigen() * J.transpose()), 0.0, 1e-8);
    EXPECT_NEAR(maxAbsDiff(p.inverseJacobian().asEigen(), J), 0.0, 1e-6);
  }
}

// ref (+) p and p (+) u vs numeric Jacobians (moderate rotations).
TEST(CPose3DQuatPDFGaussianMath, CompositionAndChangeOfReference_FiniteDifferences)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3DQuatPDFGaussian a = randomQuatPDF(CPose3D(1, 2, 3, 0.3, 0.2, -0.1), 0.05);
  const CPose3DQuatPDFGaussian b = randomQuatPDF(CPose3D(-0.5, 1.5, 0.2, 0.6, -0.3, 0.4), 0.05);

  const auto fa = [&](const Vec7& v) -> Vec7 { return toVec(toPose(v) + b.mean); };
  const auto fb = [&](const Vec7& v) -> Vec7 { return toVec(a.mean + toPose(v)); };
  const Mat7 Ja = numJacobian(fa, toVec(a.mean));
  const Mat7 Jb = numJacobian(fb, toVec(b.mean));

  const CPose3DQuatPDFGaussian r = a + b;
  const Mat7 expected =
      Ja * a.cov.asEigen() * Ja.transpose() + Jb * b.cov.asEigen() * Jb.transpose();
  EXPECT_NEAR(maxAbsDiff(r.cov.asEigen(), expected), 0.0, 1e-8);

  CPose3DQuatPDFGaussian rb = b;
  rb.changeCoordinatesReference(a.mean);
  EXPECT_NEAR(maxAbsDiff(rb.cov.asEigen(), Jb * b.cov.asEigen() * Jb.transpose()), 0.0, 1e-8);
}

// If pose_to = pose_from (+) d, with d independent of pose_from, then
// pose_from.inverseCompositionCrossCorrelation(pose_to) must recover the
// covariance of d (to first order), while the naive "-" overestimates it.
TEST(CPose3DQuatPDFGaussianMath, InverseCompositionCrossCorrelation_RecoversDisplacement)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3DQuatPDFGaussian from = randomQuatPDF(CPose3D(1, 2, 3, 0.3, 0.2, -0.1), 0.05);
  const CPose3DQuatPDFGaussian d = randomQuatPDF(CPose3D(2, -1, 0.5, 0.4, -0.1, 0.2), 0.02);
  const CPose3DQuatPDFGaussian to = from + d;

  const CPose3DQuatPDFGaussian dEst = from.inverseCompositionCrossCorrelation(to);
  EXPECT_NEAR((toVec(dEst.mean) - toVec(d.mean)).norm(), 0.0, 1e-9);
  EXPECT_NEAR(maxAbsDiff(dEst.cov.asEigen(), d.cov.asEigen()), 0.0, 1e-8)
      << dEst.cov.asEigen() << "\n\n"
      << d.cov.asEigen();

  const CPose3DQuatPDFGaussian dNaive = to - from;
  EXPECT_GT(dNaive.cov(0, 0), d.cov(0, 0));
}

// evaluatePDF() on the quaternion Gaussian obtained from an Euler Gaussian.
// That 7x7 covariance is rank 6 (unit-norm constraint).
TEST(CPose3DQuatPDFGaussianMath, EvaluatePDF_RankDeficientCovariance)
{
  const Mat6 C = Mat6::Identity() * 0.01;
  const CPose3DPDFGaussian a(CPose3D(1, 2, 3, 0.3, 0.2, 0.1), CMatrixDouble66(C));
  const CPose3DQuatPDFGaussian qa(a);
  const CPose3DQuat q2(CPose3D(1.1, 2, 3, 0.35, 0.2, 0.1));

  const double vMean = qa.evaluatePDF(qa.mean);
  const double vNorm = qa.evaluateNormalizedPDF(q2);
  EXPECT_FALSE(std::isnan(vMean));
  EXPECT_FALSE(std::isnan(vNorm));
  EXPECT_GT(vNorm, 0.0);
  EXPECT_LE(vNorm, 1.0);
}

// The unscented quaternion-to-Euler conversion must give a sensible mean and
// covariance when yaw is close to pi/2 (wrapped mean of the sigma points).
TEST(CPose3DQuatPDFMath, UnscentedQuatToEulerNearHalfPiYaw)
{
  const bool orig = mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION();
  const Mat6 C6 = Mat6::Identity() * 1e-4;
  const CPose3D mean(1, 2, 3, M_PI / 2 + 1e-5, 0.1, 0.2);
  const CPose3DQuatPDFGaussian q(CPose3DPDFGaussian(mean, CMatrixDouble66(C6)));

  mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION(true);
  const CPose3DPDFGaussian back(q);
  mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION(orig);

  EXPECT_NEAR(mrpt::math::wrapToPi(back.mean.yaw() - mean.yaw()), 0.0, 1e-4)
      << "yaw=" << back.mean.yaw();
  EXPECT_NEAR(back.cov(3, 3), 1e-4, 1e-5);
}

// Regression: the unscented Euler-to-quaternion conversion wrapped the yaw of
// the sigma points, so around yaw=+-pi they got quaternions of opposite signs
// and the result was meaningless (e.g. a mean quaternion of norm ~1e5).
TEST(CPose3DQuatPDFMath, UnscentedEulerToQuatNearPiYaw)
{
  const bool orig = mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION();
  const Mat6 C6 = Mat6::Identity() * 1e-2;
  for (const double yaw : {M_PI, -M_PI, M_PI - 1e-5, 3.0})
  {
    const CPose3DPDFGaussian p(CPose3D(1, 2, 3, yaw, 0.2, 0.1), CMatrixDouble66(C6));
    mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(false);
    const CPose3DQuatPDFGaussian lin(p);
    mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(true);
    const CPose3DQuatPDFGaussian sut(p);
    mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(orig);

    EXPECT_NEAR(sut.mean.quat().norm(), 1.0, 1e-12) << "yaw=" << yaw;
    EXPECT_NEAR((toVec(sut.mean) - toVec(lin.mean)).cwiseAbs().maxCoeff(), 0.0, 1e-3)
        << "yaw=" << yaw;
    EXPECT_NEAR((sut.cov.asEigen() - lin.cov.asEigen()).cwiseAbs().maxCoeff(), 0.0, 1e-4)
        << "yaw=" << yaw;
  }
}

// Regression: samples had non-unit quaternions (|q|-1 up to ~2.5e-2 for 0.1
// rad of angular uncertainty), i.e. they were not rigid transformations.
TEST(CPose3DQuatPDFGaussianMath, DrawSamplesHaveUnitQuaternions)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const Mat6 C6 = Mat6::Identity() * 1e-2;
  const CPose3D mean(1, 2, 3, 0.5, 0.2, 0.1);
  const CPose3DQuatPDFGaussian q(CPose3DPDFGaussian(mean, CMatrixDouble66(C6)));
  const size_t N = 20000;
  Eigen::Matrix<double, 6, 1> sum = Eigen::Matrix<double, 6, 1>::Zero();
  Mat6 sum2 = Mat6::Zero();
  for (size_t i = 0; i < N; i++)
  {
    CPose3DQuat s;
    q.drawSingleSample(s);
    ASSERT_NEAR(s.quat().norm(), 1.0, 1e-12);
    // Statistics of the samples, as Euler angles:
    const CPose3D ps(s);
    Eigen::Matrix<double, 6, 1> v;
    v << ps.x() - mean.x(), ps.y() - mean.y(), ps.z() - mean.z(),
        mrpt::math::wrapToPi(ps.yaw() - mean.yaw()), ps.pitch() - mean.pitch(),
        mrpt::math::wrapToPi(ps.roll() - mean.roll());
    sum += v;
    sum2 += v * v.transpose();
  }
  std::vector<CVectorDouble> many;
  q.drawManySamples(100, many);
  for (const auto& s : many)
  {
    EXPECT_NEAR(std::sqrt(s[3] * s[3] + s[4] * s[4] + s[5] * s[5] + s[6] * s[6]), 1.0, 1e-12);
  }
  // Same covariance as the original Euler Gaussian, up to second order terms:
  const Eigen::Matrix<double, 6, 1> m = sum / static_cast<double>(N);
  const Mat6 S = sum2 / static_cast<double>(N) - m * m.transpose();
  EXPECT_LT(m.cwiseAbs().maxCoeff(), 5e-3);
  EXPECT_LT((S - C6).cwiseAbs().maxCoeff(), 1.5e-3) << S;
}

// Regression: the Mahalanobis distance between two Gaussians of the same
// pose, with quaternions of opposite signs, was ~1e10 instead of 0.
TEST(CPose3DQuatPDFGaussianMath, MahalanobisDistance_QuaternionSign)
{
  const Mat6 C6 = Mat6::Identity() * 1e-2;
  for (const double dx : {0.0, 0.05})
  {
    CPose3DQuatPDFGaussian a(
        CPose3DPDFGaussian(CPose3D(1, 2, 3, 0.5, 0.2, 0.1), CMatrixDouble66(C6)));
    const CPose3DQuatPDFGaussian b(
        CPose3DPDFGaussian(CPose3D(1 + dx, 2, 3, 0.5, 0.2, 0.1), CMatrixDouble66(C6)));
    // The same PDF as b, with -q: the quaternion cross-covariances flip.
    CPose3DQuatPDFGaussian bm = b;
    Mat7 J = Mat7::Identity();
    for (int i = 3; i < 7; i++)
    {
      bm.mean[i] = -b.mean[i];
      J(i, i) = -1;
    }
    bm.cov.asEigen() = J * b.cov.asEigen() * J.transpose();
    const double d = a.mahalanobisDistanceTo(b);
    EXPECT_NEAR(d, dx / std::sqrt(2 * 1e-2), 1e-6);
    EXPECT_NEAR(a.mahalanobisDistanceTo(bm), d, 1e-6) << "dx=" << dx;
  }
}
