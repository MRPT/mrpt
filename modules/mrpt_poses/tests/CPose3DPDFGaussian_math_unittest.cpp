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

// Math checks for CPose3DPDFGaussian, CPose3DPDFGaussianInf and the
// Euler <-> quaternion Gaussian conversions, against finite-difference
// Jacobians, closed-form expressions and Monte Carlo.

#include <gtest/gtest.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <functional>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat7 = Eigen::Matrix<double, 7, 7>;

Vec6 toVec(const CPose3D& p)
{
  Vec6 v;
  v << p.x(), p.y(), p.z(), p.yaw(), p.pitch(), p.roll();
  return v;
}
CPose3D toPose(const Vec6& v) { return {v[0], v[1], v[2], v[3], v[4], v[5]}; }

template <int M, int N>
Eigen::Matrix<double, M, N> numJacobian(
    const std::function<Eigen::Matrix<double, M, 1>(const Eigen::Matrix<double, N, 1>&)>& f,
    const Eigen::Matrix<double, N, 1>& x0,
    const std::array<bool, M>& wrapOut,
    double h = 1e-6)
{
  Eigen::Matrix<double, M, N> J;
  for (int j = 0; j < N; j++)
  {
    Eigen::Matrix<double, N, 1> xp = x0;
    Eigen::Matrix<double, N, 1> xm = x0;
    xp[j] += h;
    xm[j] -= h;
    Eigen::Matrix<double, M, 1> d = f(xp) - f(xm);
    for (int i = 0; i < M; i++)
    {
      if (wrapOut[i])
      {
        d[i] = mrpt::math::wrapToPi(d[i]);
      }
    }
    J.col(j) = d / (2 * h);
  }
  return J;
}

const std::array<bool, 6> kWrap6 = {false, false, false, true, true, true};
const std::array<bool, 7> kNoWrap7 = {false, false, false, false, false, false, false};

Mat6 randomCov6(double scale)
{
  CMatrixDouble66 r;
  mrpt::random::getRandomGenerator().drawGaussian1DMatrix(r, 0, scale);
  Mat6 C = r.asEigen() * r.asEigen().transpose();
  C += 1e-3 * scale * scale * Mat6::Identity();
  return C;
}

double maxAbsDiff(const Mat6& a, const Mat6& b) { return (a - b).cwiseAbs().maxCoeff(); }
double maxAbs(const Mat6& a) { return a.cwiseAbs().maxCoeff(); }

}  // namespace

namespace
{
void checkJacobiansPoseComposition(const CPose3D& x, const CPose3D& u)
{
  CMatrixDouble66 df_dx;
  CMatrixDouble66 df_du;
  CPose3DPDF::jacobiansPoseComposition(x, u, df_dx, df_du);

  const auto fx = [&](const Vec6& v) -> Vec6 { return toVec(toPose(v) + u); };
  const auto fu = [&](const Vec6& v) -> Vec6 { return toVec(x + toPose(v)); };
  const Mat6 Jx = numJacobian<6, 6>(fx, toVec(x), kWrap6);
  const Mat6 Ju = numJacobian<6, 6>(fu, toVec(u), kWrap6);
  EXPECT_NEAR(maxAbsDiff(df_dx.asEigen(), Jx), 0.0, 1e-6) << "x=" << x << " u=" << u << "\n"
                                                          << df_dx.asEigen() << "\n\n"
                                                          << Jx;
  EXPECT_NEAR(maxAbsDiff(df_du.asEigen(), Ju), 0.0, 1e-6) << "x=" << x << " u=" << u << "\n"
                                                          << df_du.asEigen() << "\n\n"
                                                          << Ju;
}
}  // namespace

// The 6x6 YPR composition Jacobians vs finite differences (moderate angles).
TEST(CPose3DPDFMath, JacobiansPoseComposition_FiniteDifferences)
{
  checkJacobiansPoseComposition(
      CPose3D(1, 2, 3, 0.3, 0.2, -0.1), CPose3D(-0.5, 1.5, 0.2, 0.6, -0.3, 0.4));
  checkJacobiansPoseComposition(
      CPose3D(5, -3, 1, -1.0, 1.0, 1.0), CPose3D(0.1, 0.2, 0.3, 0.0, 0.0, 0.0));
  checkJacobiansPoseComposition(
      CPose3D(0, 1, 0, 1.2, -0.4, 0.3), CPose3D(4.0, -1.0, 2.0, 0.5, 0.7, -0.6));
}

// Large rotations, where the raw quaternion product of x and u has a negative
// real part and CQuaternion::crossProduct() flips its sign.
TEST(CPose3DPDFMath, JacobiansPoseComposition_LargeRotations)
{
  // Two 2 rad yaw rotations: total 4 rad, raw product real part cos(2)<0.
  checkJacobiansPoseComposition(CPose3D(0, 0, 0, 2.0, 0, 0), CPose3D(1, 0, 0, 2.0, 0, 0));
  checkJacobiansPoseComposition(
      CPose3D(0, 0, 0, 3.0, -1.2, 2.9), CPose3D(4.0, -1.0, 2.0, -2.8, 0.7, -3.0));

  // The simplest observable consequence: yaw-only uncertainty composed with
  // a deterministic 1 m forward motion. Cov(x_out, yaw_out) = -sin(2)*var.
  Mat6 C = Mat6::Zero();
  C(3, 3) = 0.01;
  CPose3DPDFGaussian p(CPose3D(0, 0, 0, 2.0, 0, 0), CMatrixDouble66(C));
  p += CPose3D(1, 0, 0, 2.0, 0, 0);
  EXPECT_NEAR(p.cov(0, 3), -std::sin(2.0) * 0.01, 1e-9);
  EXPECT_NEAR(p.cov(1, 3), std::cos(2.0) * 0.01, 1e-9);

  // Same thing through the quaternion PDF and the uncertain-uncertain path:
  CPose3DPDFGaussian p2(CPose3D(0, 0, 0, 2.0, 0, 0), CMatrixDouble66(C));
  p2 = p2 + CPose3DPDFGaussian(CPose3D(1, 0, 0, 2.0, 0, 0), CMatrixDouble66());
  EXPECT_NEAR(p2.cov(0, 3), -std::sin(2.0) * 0.01, 1e-9);

  // inverse() is computed through the same quaternion composition, and is
  // affected when the mean quaternion has a negative real part:
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3D m(1, 2, 3, 3.0, -1.2, 2.9);
  const Mat6 Cm = randomCov6(0.05);
  const CPose3DPDFGaussian q(m, CMatrixDouble66(Cm));
  const auto f = [&](const Vec6& v) -> Vec6 { return toVec(-toPose(v)); };
  const Mat6 J = numJacobian<6, 6>(f, toVec(m), kWrap6);
  EXPECT_NEAR(maxAbsDiff((-q).cov.asEigen(), J * Cm * J.transpose()), 0.0, 1e-6);
}

// Composition, inverse composition, inverse and change of reference.
TEST(CPose3DPDFGaussianMath, Operators_FiniteDifferences)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3D xm(1, 2, 3, 1.2, 0.2, -0.1);
  const CPose3D um(-0.5, 1.5, 0.2, 0.6, -0.3, 0.4);
  const Mat6 Cx = randomCov6(0.05);
  const Mat6 Cu = randomCov6(0.05);
  const CPose3DPDFGaussian X(xm, CMatrixDouble66(Cx));
  const CPose3DPDFGaussian U(um, CMatrixDouble66(Cu));

  // x (+) u
  {
    const auto fx = [&](const Vec6& v) -> Vec6 { return toVec(toPose(v) + um); };
    const auto fu = [&](const Vec6& v) -> Vec6 { return toVec(xm + toPose(v)); };
    const Mat6 Jx = numJacobian<6, 6>(fx, toVec(xm), kWrap6);
    const Mat6 Ju = numJacobian<6, 6>(fu, toVec(um), kWrap6);
    const CPose3DPDFGaussian R = X + U;
    EXPECT_NEAR(
        maxAbsDiff(R.cov.asEigen(), Jx * Cx * Jx.transpose() + Ju * Cu * Ju.transpose()), 0.0,
        1e-6);

    CPose3DPDFGaussian R2 = X;
    R2 += um;
    EXPECT_NEAR(maxAbsDiff(R2.cov.asEigen(), Jx * Cx * Jx.transpose()), 0.0, 1e-6);
  }
  // x (-) u
  {
    const auto fx = [&](const Vec6& v) -> Vec6 { return toVec(toPose(v) - um); };
    const auto fu = [&](const Vec6& v) -> Vec6 { return toVec(xm - toPose(v)); };
    const Mat6 Jx = numJacobian<6, 6>(fx, toVec(xm), kWrap6);
    const Mat6 Ju = numJacobian<6, 6>(fu, toVec(um), kWrap6);
    const CPose3DPDFGaussian R = X - U;
    EXPECT_NEAR(
        maxAbsDiff(R.cov.asEigen(), Jx * Cx * Jx.transpose() + Ju * Cu * Ju.transpose()), 0.0,
        1e-6);
    EXPECT_NEAR((toVec(R.mean) - toVec(xm - um)).norm(), 0.0, 1e-9);
  }
  // inverse
  {
    const auto f = [&](const Vec6& v) -> Vec6 { return toVec(-toPose(v)); };
    const Mat6 J = numJacobian<6, 6>(f, toVec(xm), kWrap6);
    const CPose3DPDFGaussian R = -X;
    EXPECT_NEAR(maxAbsDiff(R.cov.asEigen(), J * Cx * J.transpose()), 0.0, 1e-6);
  }
  // ref (+) x
  {
    const CPose3D ref(-2, 0.5, 1, -2.0, 0.4, 0.9);
    const auto f = [&](const Vec6& v) -> Vec6 { return toVec(ref + toPose(v)); };
    const Mat6 J = numJacobian<6, 6>(f, toVec(xm), kWrap6);
    CPose3DPDFGaussian R = X;
    R.changeCoordinatesReference(ref);
    EXPECT_NEAR(maxAbsDiff(R.cov.asEigen(), J * Cx * J.transpose()), 0.0, 1e-6);
  }
}

// The information form must give the same results as the covariance form.
TEST(CPose3DPDFGaussianInfMath, OperatorsMatchCovarianceForm)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3D xm(1, 2, 3, 2.9, 0.2, -0.1);
  const CPose3D um(-0.5, 1.5, 0.2, 0.6, -0.3, 0.4);
  const Mat6 Cx = randomCov6(0.05);
  const Mat6 Cu = randomCov6(0.05);
  const CPose3DPDFGaussian X(xm, CMatrixDouble66(Cx));
  const CPose3DPDFGaussian U(um, CMatrixDouble66(Cu));
  const CPose3DPDFGaussianInf Xi(xm, CMatrixDouble66(Mat6(Cx.inverse())));
  const CPose3DPDFGaussianInf Ui(um, CMatrixDouble66(Mat6(Cu.inverse())));

  const auto covOf = [](const CPose3DPDFGaussianInf& p) -> Mat6
  { return p.cov_inv.asEigen().inverse(); };

  EXPECT_NEAR(maxAbsDiff(covOf(Xi + Ui), (X + U).cov.asEigen()), 0.0, 1e-8);
  EXPECT_NEAR(maxAbsDiff(covOf(Xi - Ui), (X - U).cov.asEigen()), 0.0, 1e-8);

  CPose3DPDFGaussian Xr = X;
  CPose3DPDFGaussianInf Xir = Xi;
  const CPose3D ref(-2, 0.5, 1, -2.0, 0.4, 0.9);
  Xr.changeCoordinatesReference(ref);
  Xir.changeCoordinatesReference(ref);
  EXPECT_NEAR(maxAbsDiff(covOf(Xir), Xr.cov.asEigen()), 0.0, 1e-8);

  CPose3DPDFGaussian F;
  F.bayesianFusion(
      X, CPose3DPDFGaussian(CPose3D(1.1, 2.1, 2.9, 2.8, 0.1, 0.0), CMatrixDouble66(Cu)));
  CPose3DPDFGaussianInf Fi;
  Fi.bayesianFusion(
      Xi, CPose3DPDFGaussianInf(
              CPose3D(1.1, 2.1, 2.9, 2.8, 0.1, 0.0), CMatrixDouble66(Mat6(Cu.inverse()))));
  EXPECT_NEAR(maxAbsDiff(covOf(Fi), F.cov.asEigen()), 0.0, 1e-8);
  EXPECT_NEAR((toVec(Fi.mean) - toVec(F.mean)).norm(), 0.0, 1e-8);
}

// operator+=(CPose3D) in information form: Inf' = J^-T*Inf*J^-1.
TEST(CPose3DPDFGaussianInfMath, OperatorPlusEqualsPose3D_MatchesCovarianceForm)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3D xm(1, 2, 3, 0.5, 0.2, -0.1);
  const CPose3D Ap(2.0, -1.0, 0.5, 0.3, 0.1, 0.2);
  const Mat6 C = randomCov6(0.05);

  CPose3DPDFGaussian X(xm, CMatrixDouble66(C));
  CPose3DPDFGaussianInf Xi(xm, CMatrixDouble66(Mat6(C.inverse())));
  X += Ap;
  Xi += Ap;
  const Mat6 got = Xi.cov_inv.asEigen().inverse();
  EXPECT_NEAR(maxAbsDiff(got, X.cov.asEigen()), 0.0, 1e-8) << "Inf form:\n"
                                                           << got << "\nCov form:\n"
                                                           << X.cov.asEigen();
}

// inverse() in information form must match the covariance form.
TEST(CPose3DPDFGaussianInfMath, Inverse_MatchesCovarianceForm)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose3D xm(1, 2, 3, 0.5, 0.2, -0.1);
  const Mat6 C = randomCov6(0.05);
  const CPose3DPDFGaussian X(xm, CMatrixDouble66(C));
  const CPose3DPDFGaussianInf Xi(xm, CMatrixDouble66(Mat6(C.inverse())));

  const CPose3DPDFGaussian R = -X;
  Mat6 got = Mat6::Constant(std::numeric_limits<double>::quiet_NaN());
  try
  {
    const CPose3DPDFGaussianInf Ri = -Xi;
    got = Ri.cov_inv.asEigen().inverse();
    EXPECT_NEAR((toVec(Ri.mean) - toVec(R.mean)).norm(), 0.0, 1e-9);
  }
  catch (const std::exception& e)
  {
    ADD_FAILURE() << "inverse() threw: " << e.what();
  }
  EXPECT_NEAR(maxAbsDiff(got, R.cov.asEigen()), 0.0, 1e-8) << got;
}

// The Mahalanobis distance must wrap angle differences: poses with
// yaw = +-(pi - eps) are close to each other.
TEST(CPose3DPDFGaussianMath, MahalanobisDistance_AngleWrap)
{
  Mat6 C = Mat6::Identity() * 0.01;
  CPose3DPDFGaussian a(CPose3D(1, 2, 3, M_PI - 0.01, 0.1, 0.2), CMatrixDouble66(C));
  CPose3DPDFGaussian b(CPose3D(1, 2, 3, -M_PI + 0.01, 0.1, 0.2), CMatrixDouble66(C));
  // Expected: yaw difference of 0.02 rad, combined variance 0.02.
  const double expected = 0.02 / std::sqrt(0.02);
  EXPECT_NEAR(a.mahalanobisDistanceTo(b), expected, 1e-6);

  CPose3DPDFGaussianInf ai(a.mean, CMatrixDouble66(Mat6(C.inverse())));
  CPose3DPDFGaussianInf bi(b.mean, CMatrixDouble66(Mat6(C.inverse())));
  EXPECT_NEAR(ai.mahalanobisDistanceTo(bi), expected, 1e-6);

  // Without wrap issues, it must match the closed form:
  CPose3DPDFGaussian c(CPose3D(1.1, 2.0, 2.9, 0.3, 0.1, 0.2), CMatrixDouble66(C));
  CPose3DPDFGaussian d(CPose3D(1.0, 2.2, 3.0, 0.1, 0.0, 0.2), CMatrixDouble66(C));
  Vec6 dv = toVec(d.mean) - toVec(c.mean);
  EXPECT_NEAR(c.mahalanobisDistanceTo(d), std::sqrt(dv.dot((2 * C).inverse() * dv)), 1e-9);
}

// bayesianFusion must not average raw Euler angles across +-pi.
TEST(CPose3DPDFGaussianMath, BayesianFusion_AngleWrap)
{
  Mat6 C = Mat6::Identity() * 0.01;
  const CPose3DPDFGaussian a(CPose3D(1, 2, 3, M_PI - 0.05, 0.1, 0.2), CMatrixDouble66(C));
  const CPose3DPDFGaussian b(CPose3D(1, 2, 3, -M_PI + 0.05, 0.1, 0.2), CMatrixDouble66(C));
  CPose3DPDFGaussian r;
  r.bayesianFusion(a, b);
  EXPECT_NEAR(std::abs(mrpt::math::wrapToPi(r.mean.yaw() - M_PI)), 0.0, 1e-6)
      << "fused yaw=" << r.mean.yaw();
}

// evaluatePDF() vs the closed form, for a query whose yaw lies on the other
// side of +-pi.
TEST(CPose3DPDFGaussianMath, EvaluatePDF_AngleWrap)
{
  Mat6 C = Mat6::Zero();
  C.diagonal() << 0.04, 0.09, 0.01, 0.01, 0.02, 0.03;
  const CPose3D mean(1, 2, 3, M_PI - 0.02, 0.1, 0.2);
  const CPose3DPDFGaussian p(mean, CMatrixDouble66(C));
  const CPose3DPDFGaussianInf pi(mean, CMatrixDouble66(Mat6(C.inverse())));

  const CPose3D q(1.1, 2.1, 3.0, -M_PI + 0.02, 0.15, 0.2);
  Vec6 d;
  d << 0.1, 0.1, 0.0, 0.04, 0.05, 0.0;
  const double expected =
      std::exp(-0.5 * d.dot(C.inverse() * d)) / std::sqrt(std::pow(2 * M_PI, 6) * C.determinant());
  EXPECT_NEAR(p.evaluatePDF(q), expected, 1e-6 * expected);
  EXPECT_NEAR(pi.evaluatePDF(q), expected, 1e-6 * expected);
  const double expectedNorm = std::exp(-0.5 * d.dot(C.inverse() * d));
  EXPECT_NEAR(p.evaluateNormalizedPDF(q), expectedNorm, 1e-9);
  EXPECT_NEAR(pi.evaluateNormalizedPDF(q), expectedNorm, 1e-9);
}

// drawManySamples() sample statistics.
TEST(CPose3DPDFGaussianMath, DrawManySamples_MonteCarlo)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const Mat6 C = randomCov6(0.05);
  const CPose3D mean(1, 2, 3, 0.3, -0.2, 0.1);
  const CPose3DPDFGaussian p(mean, CMatrixDouble66(C));
  const CPose3DPDFGaussianInf pi(mean, CMatrixDouble66(Mat6(C.inverse())));

  const size_t N = 100000;
  for (int method = 0; method < 2; method++)
  {
    std::vector<CVectorDouble> v;
    if (method == 0)
    {
      p.drawManySamples(N, v);
    }
    else
    {
      pi.drawManySamples(N, v);
    }
    ASSERT_EQ(v.size(), N);
    Mat6 S = Mat6::Zero();
    Vec6 m = Vec6::Zero();
    for (const auto& s : v)
    {
      Vec6 d;
      for (int k = 0; k < 6; k++)
      {
        d[k] = s[k] - toVec(mean)[k];
      }
      m += d / N;
      S += d * d.transpose() / N;
    }
    S -= m * m.transpose();
    EXPECT_NEAR(m.cwiseAbs().maxCoeff(), 0.0, 3e-3);
    EXPECT_NEAR(maxAbsDiff(S, C), 0.0, 0.03 * maxAbs(C)) << S << "\n\n" << C;
  }
}

// Euler -> quaternion -> Euler Gaussian conversions vs numeric Jacobians.
TEST(CPose3DQuatPDFGaussianMath, EulerQuatConversion_FiniteDifferences)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const Mat6 C = randomCov6(0.05);
  const CPose3D mean(1, 2, 3, 2.5, -0.7, 0.4);
  const CPose3DPDFGaussian p(mean, CMatrixDouble66(C));
  const CPose3DQuatPDFGaussian q(p);

  const auto f = [](const Vec6& v) -> Vec7
  {
    const CPose3DQuat pq(toPose(v));
    Vec7 r;
    for (int i = 0; i < 7; i++)
    {
      r[i] = pq[i];
    }
    return r;
  };
  const Eigen::Matrix<double, 7, 6> J = numJacobian<7, 6>(f, toVec(mean), kNoWrap7);
  const Mat7 expectedQ = J * C * J.transpose();
  EXPECT_NEAR((q.cov.asEigen() - expectedQ).cwiseAbs().maxCoeff(), 0.0, 1e-7);

  // Back to Euler: must recover the original covariance.
  const CPose3DPDFGaussian p2(q);
  EXPECT_NEAR(maxAbsDiff(p2.cov.asEigen(), C), 0.0, 1e-7);
  EXPECT_NEAR((toVec(p2.mean) - toVec(mean)).norm(), 0.0, 1e-9);

  // Quaternion -> Euler for a quaternion Gaussian with a full-rank
  // covariance (i.e. including the normalization direction).
  Mat7 Cq = Mat7::Identity() * 1e-3;
  Cq(3, 4) = Cq(4, 3) = 2e-4;
  const CPose3DQuatPDFGaussian q2(q.mean, CMatrixDouble77(Cq));
  const CPose3DPDFGaussian p3(q2);
  const auto g = [](const Vec7& v) -> Vec6
  {
    CQuaternionDouble qq(v[3], v[4], v[5], v[6]);
    qq.normalize();
    const CPose3D pp(CPose3DQuat(v[0], v[1], v[2], qq));
    return toVec(pp);
  };
  Vec7 q0;
  for (int i = 0; i < 7; i++)
  {
    q0[i] = q.mean[i];
  }
  const Eigen::Matrix<double, 6, 7> G = numJacobian<6, 7>(g, q0, kWrap6);
  EXPECT_NEAR(maxAbsDiff(p3.cov.asEigen(), G * Cq * G.transpose()), 0.0, 1e-7);
}

// Differential entropy of a Gaussian: 0.5*log((2*pi*e)^n * det(C)).
// Realistic small 6D covariances have det(C) far below machine epsilon.
TEST(CPose3DPDFGaussianMath, CovarianceEntropy)
{
  Mat6 C = Mat6::Zero();
  C.diagonal() << 1.0, 2.0, 0.5, 0.1, 0.2, 0.3;
  const CPose3DPDFGaussian big(CPose3D(0, 0, 0), CMatrixDouble66(C));
  const auto H = [](const Mat6& c)
  { return 0.5 * (6.0 * std::log(2 * M_PI * std::exp(1.0)) + std::log(c.determinant())); };
  EXPECT_NEAR(big.getCovarianceEntropy(), H(C), 1e-9);

  // 1 cm, 0.5 deg standard deviations:
  Mat6 Cs = Mat6::Zero();
  Cs.diagonal() << 1e-4, 1e-4, 1e-4, 7.6e-5, 7.6e-5, 7.6e-5;
  const CPose3DPDFGaussian small(CPose3D(0, 0, 0), CMatrixDouble66(Cs));
  EXPECT_NEAR(small.getCovarianceEntropy(), H(Cs), 1e-6)
      << "det=" << Cs.determinant() << " eps=" << std::numeric_limits<double>::epsilon();
}

// Regression: near gimbal lock (|pitch| within ~0.36 deg of 90 deg) the
// composition Jacobians had zero pitch and roll rows, so composed PDFs got a
// zero pitch variance. The Jacobians grow as 1/cos(pitch): relative test.
TEST(CPose3DPDFMath, JacobiansPoseComposition_NearGimbalLock)
{
  const CPose3D x(1, 2, 3, 0.3, 1.2, -0.2);
  for (const double pitchDeg : {89.0, 89.7, 89.9, -89.8})
  {
    const CPose3D target(2, -1, 0.5, 0.4, DEG2RAD(pitchDeg), 0.6);
    const CPose3D u = (-x) + target;
    CMatrixDouble66 df_dx;
    CMatrixDouble66 df_du;
    CPose3DPDF::jacobiansPoseComposition(x, u, df_dx, df_du);
    const auto fx = [&](const Vec6& v) -> Vec6 { return toVec(toPose(v) + u); };
    const auto fu = [&](const Vec6& v) -> Vec6 { return toVec(x + toPose(v)); };
    const Mat6 Jx = numJacobian<6, 6>(fx, toVec(x), kWrap6, 1e-7);
    const Mat6 Ju = numJacobian<6, 6>(fu, toVec(u), kWrap6, 1e-7);
    EXPECT_LT((df_dx.asEigen() - Jx).norm() / Jx.norm(), 1e-5) << "pitch: " << pitchDeg;
    EXPECT_LT((df_du.asEigen() - Ju).norm() / Ju.norm(), 1e-5) << "pitch: " << pitchDeg;

    // The composed pitch is uncertain if the input pitch is:
    Mat6 C = Mat6::Zero();
    C(4, 4) = 1e-4;
    CPose3DPDFGaussian p(x, CMatrixDouble66(C));
    p += u;
    EXPECT_GT(p.cov(4, 4), 1e-6) << "pitch: " << pitchDeg;
  }
}

// Regression: quaternion to Euler Gaussian conversion near gimbal lock gave a
// rotation off by up to ~0.4 deg and a wrong covariance.
TEST(CPose3DQuatPDFGaussianMath, QuatToEulerNearGimbalLock)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (const double pitchDeg : {89.7, 89.9, -89.8})
  {
    const CPose3D mean(1, 2, 3, 0.4, DEG2RAD(pitchDeg), -0.7);
    const CPose3DQuat qmean(mean);
    // Covariance of the quaternion: tangent to the unit sphere at the mean.
    Eigen::Matrix<double, 7, 6> T = Eigen::Matrix<double, 7, 6>::Zero();
    T.topLeftCorner<3, 3>().setIdentity();
    {
      CQuaternionDouble q = qmean.quat();
      // Small rotation increments: q * (1, w/2)
      CMatrixDouble44 Lq;
      q.rotationJacobian(Lq);
      T.block<4, 3>(3, 3) = 0.5 * Lq.asEigen().rightCols<3>();
    }
    const Mat6 C6 = randomCov6(0.01);
    const Mat7 Cq = T * C6 * T.transpose();
    const CPose3DQuatPDFGaussian pq(qmean, CMatrixDouble77(Cq));
    const CPose3DPDFGaussian pe(pq);

    EXPECT_LT(
        (pe.mean.getRotationMatrix().asEigen() - mean.getRotationMatrix().asEigen())
            .cwiseAbs()
            .maxCoeff(),
        1e-12);

    const auto g = [](const Vec7& v) -> Vec6
    {
      CQuaternionDouble qq(v[3], v[4], v[5], v[6]);
      qq.normalize();
      CMatrixDouble33 R;
      qq.rotationMatrixNoResize(R);
      double yaw = 0;
      double pitch = 0;
      double roll = 0;
      TPose3D::SO3_to_yaw_pitch_roll(R, yaw, pitch, roll);
      Vec6 r;
      r << v[0], v[1], v[2], yaw, pitch, roll;
      return r;
    };
    Vec7 q0;
    for (int i = 0; i < 7; i++)
    {
      q0[i] = qmean[i];
    }
    const Eigen::Matrix<double, 6, 7> G = numJacobian<6, 7>(g, q0, kWrap6, 1e-8);
    const Mat6 expected = G * Cq * G.transpose();
    EXPECT_LT(maxAbsDiff(pe.cov.asEigen(), expected) / maxAbs(expected), 1e-5)
        << "pitch: " << pitchDeg;
  }
}
