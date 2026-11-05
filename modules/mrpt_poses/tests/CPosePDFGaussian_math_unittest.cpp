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

// Math checks for CPosePDFGaussian and CPosePDFGaussianInf against
// finite-difference Jacobians, analytic formulas and Monte Carlo.

#include <gtest/gtest.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <functional>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

Vec3 toVec(const CPose2D& p) { return {p.x(), p.y(), p.phi()}; }
CPose2D toPose(const Vec3& v) { return {v[0], v[1], v[2]}; }

// Central-difference Jacobian of a R^n -> R^m map. Components flagged in
// `wrapOut` are angles: their differences are wrapped to [-pi,pi].
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

const std::array<bool, 3> kWrap3 = {false, false, true};

Mat3 randomCov(double scale)
{
  CMatrixDouble33 r;
  mrpt::random::getRandomGenerator().drawGaussian1DMatrix(r, 0, scale);
  Mat3 C = r.asEigen() * r.asEigen().transpose();
  C += 1e-4 * scale * scale * Mat3::Identity();
  return C;
}

double maxAbsDiff(const Mat3& a, const Mat3& b) { return (a - b).cwiseAbs().maxCoeff(); }

}  // namespace

// p (+) Ap, with Ap deterministic: cov must be J*C*J^T, J = d(x+Ap)/dx.
TEST(CPosePDFGaussianMath, OperatorPlusEqualsPose2D_Covariance)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D mean(1.0, 2.0, 0.5);
  const CPose2D Ap(3.0, -1.0, 0.7);
  const Mat3 C = randomCov(0.1);

  CPosePDFGaussian p(mean, CMatrixDouble33(C));
  p += Ap;

  const auto f = [&](const Vec3& x) -> Vec3 { return toVec(toPose(x) + Ap); };
  const Mat3 J = numJacobian<3, 3>(f, toVec(mean), kWrap3);
  const Mat3 expected = J * C * J.transpose();

  EXPECT_NEAR(maxAbsDiff(p.cov.asEigen(), expected), 0.0, 1e-6)
      << "got:\n"
      << p.cov.asEigen() << "\nexpected:\n"
      << expected;
  EXPECT_NEAR((toVec(p.mean) - toVec(mean + Ap)).norm(), 0.0, 1e-12);
}

// Same check for the information form.
TEST(CPosePDFGaussianInfMath, OperatorPlusEqualsPose2D_Covariance)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D mean(1.0, 2.0, 0.5);
  const CPose2D Ap(3.0, -1.0, 0.7);
  const Mat3 C = randomCov(0.1);

  CPosePDFGaussianInf p(mean, CMatrixDouble33(Mat3(C.inverse())));
  p += Ap;

  const auto f = [&](const Vec3& x) -> Vec3 { return toVec(toPose(x) + Ap); };
  const Mat3 J = numJacobian<3, 3>(f, toVec(mean), kWrap3);
  const Mat3 expected = J * C * J.transpose();
  const Mat3 got = p.cov_inv.asEigen().inverse();

  EXPECT_NEAR(maxAbsDiff(got, expected), 0.0, 1e-6) << "got:\n"
                                                    << got << "\nexpected:\n"
                                                    << expected;
}

// Uncertain pose (+) uncertain pose: compare against numeric Jacobians.
TEST(CPosePDFGaussianMath, OperatorPlusEqualsGaussian_Covariance)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D xm(1.0, 2.0, 2.9);
  const CPose2D um(-0.5, 1.5, 0.6);
  const Mat3 Cx = randomCov(0.1);
  const Mat3 Cu = randomCov(0.1);

  const CPosePDFGaussian a(xm, CMatrixDouble33(Cx));
  const CPosePDFGaussian b(um, CMatrixDouble33(Cu));
  const CPosePDFGaussian r = a + b;

  const auto fx = [&](const Vec3& x) -> Vec3 { return toVec(toPose(x) + um); };
  const auto fu = [&](const Vec3& u) -> Vec3 { return toVec(xm + toPose(u)); };
  const Mat3 Jx = numJacobian<3, 3>(fx, toVec(xm), kWrap3);
  const Mat3 Ju = numJacobian<3, 3>(fu, toVec(um), kWrap3);
  const Mat3 expected = Jx * Cx * Jx.transpose() + Ju * Cu * Ju.transpose();

  EXPECT_NEAR(maxAbsDiff(r.cov.asEigen(), expected), 0.0, 1e-6);
  EXPECT_NEAR((toVec(r.mean) - toVec(xm + um)).norm(), 0.0, 1e-12);

  // The information form must give the same result:
  const CPosePDFGaussianInf ai(xm, CMatrixDouble33(Mat3(Cx.inverse())));
  const CPosePDFGaussianInf bi(um, CMatrixDouble33(Mat3(Cu.inverse())));
  const CPosePDFGaussianInf ri = ai + bi;
  EXPECT_NEAR(maxAbsDiff(ri.cov_inv.asEigen().inverse(), expected), 0.0, 1e-6);
}

// ref (+) p, with ref deterministic.
TEST(CPosePDFGaussianMath, ChangeCoordinatesReference_Covariance)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D mean(1.0, -2.0, -2.5);
  const CPose2D ref(4.0, 1.0, 1.2);
  const Mat3 C = randomCov(0.2);

  CPosePDFGaussian p(mean, CMatrixDouble33(C));
  p.changeCoordinatesReference(ref);

  const auto f = [&](const Vec3& x) -> Vec3 { return toVec(ref + toPose(x)); };
  const Mat3 J = numJacobian<3, 3>(f, toVec(mean), kWrap3);
  EXPECT_NEAR(maxAbsDiff(p.cov.asEigen(), J * C * J.transpose()), 0.0, 1e-6);

  // operator+(CPose2D, CPosePDFGaussian) is the same operation:
  const CPosePDFGaussian p2 = ref + CPosePDFGaussian(mean, CMatrixDouble33(C));
  EXPECT_NEAR(maxAbsDiff(p2.cov.asEigen(), p.cov.asEigen()), 0.0, 1e-12);
}

// inverse(): compare to numeric Jacobian, for both forms (the information
// matrix transforms with the inverse Jacobian).
TEST(CPosePDFGaussianInfMath, Inverse_MatchesCovarianceForm)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D mean(3.0, -1.0, 0.8);
  const Mat3 C = randomCov(0.1);

  CPosePDFGaussian g(mean, CMatrixDouble33(C));
  CPosePDFGaussian gInv;
  g.inverse(gInv);

  const auto f = [&](const Vec3& x) -> Vec3 { return toVec(CPose2D() - toPose(x)); };
  const Mat3 J = numJacobian<3, 3>(f, toVec(mean), kWrap3);
  const Mat3 expected = J * C * J.transpose();
  EXPECT_NEAR(maxAbsDiff(gInv.cov.asEigen(), expected), 0.0, 1e-6);

  CPosePDFGaussianInf gi(mean, CMatrixDouble33(Mat3(C.inverse())));
  CPosePDFGaussianInf giInv;
  gi.inverse(giInv);
  const Mat3 gotCov = giInv.cov_inv.asEigen().inverse();
  EXPECT_NEAR(maxAbsDiff(gotCov, expected), 0.0, 1e-6) << "got:\n"
                                                       << gotCov << "\nexpected:\n"
                                                       << expected;
}

// x1 (-) x0 for independent variables.
TEST(CPosePDFGaussianMath, InverseComposition_Independent)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D m1(2.0, 1.0, -2.8);
  const CPose2D m0(-1.0, 3.0, 2.7);
  const Mat3 C1 = randomCov(0.1);
  const Mat3 C0 = randomCov(0.1);

  const CPosePDFGaussian x1(m1, CMatrixDouble33(C1));
  const CPosePDFGaussian x0(m0, CMatrixDouble33(C0));
  const CPosePDFGaussian d = x1 - x0;

  const auto f1 = [&](const Vec3& v) -> Vec3 { return toVec(toPose(v) - m0); };
  const auto f0 = [&](const Vec3& v) -> Vec3 { return toVec(m1 - toPose(v)); };
  const Mat3 J1 = numJacobian<3, 3>(f1, toVec(m1), kWrap3);
  const Mat3 J0 = numJacobian<3, 3>(f0, toVec(m0), kWrap3);
  const Mat3 expected = J1 * C1 * J1.transpose() + J0 * C0 * J0.transpose();

  EXPECT_NEAR(maxAbsDiff(d.cov.asEigen(), expected), 0.0, 1e-6);

  // Aliased in/out argument: x1 -= x0
  CPosePDFGaussian x1b = x1;
  x1b -= x0;
  EXPECT_NEAR(maxAbsDiff(x1b.cov.asEigen(), expected), 0.0, 1e-6);
  EXPECT_NEAR((toVec(x1b.mean) - toVec(d.mean)).norm(), 0.0, 1e-12);

  // Information form must agree:
  const CPosePDFGaussianInf x1i(m1, CMatrixDouble33(Mat3(C1.inverse())));
  const CPosePDFGaussianInf x0i(m0, CMatrixDouble33(Mat3(C0.inverse())));
  const CPosePDFGaussianInf di = x1i - x0i;
  EXPECT_NEAR(maxAbsDiff(di.cov_inv.asEigen().inverse(), expected), 0.0, 1e-6);
}

// x1 (-) x0 given their cross-covariance. A symmetric COV_01 is used so the
// expected value does not depend on whether it means Cov(x0,x1) or Cov(x1,x0).
TEST(CPosePDFGaussianMath, InverseComposition_WithCrossCovariance)
{
  const CPose2D m1(2.0, 1.0, 0.4);
  const CPose2D m0(-1.0, 3.0, -0.3);
  Mat3 C0 = Mat3::Zero();
  C0.diagonal() << 0.04, 0.09, 0.01;
  Mat3 C1 = Mat3::Zero();
  C1.diagonal() << 0.05, 0.02, 0.02;
  Mat3 C01 = Mat3::Zero();
  C01.diagonal() << 0.03, 0.03, 0.01;  // Joint 6x6 cov is still PD

  const CPosePDFGaussian x1(m1, CMatrixDouble33(C1));
  const CPosePDFGaussian x0(m0, CMatrixDouble33(C0));
  CPosePDFGaussian d;
  d.inverseComposition(x1, x0, CMatrixDouble33(C01));

  const auto f1 = [&](const Vec3& v) -> Vec3 { return toVec(toPose(v) - m0); };
  const auto f0 = [&](const Vec3& v) -> Vec3 { return toVec(m1 - toPose(v)); };
  const Mat3 J1 = numJacobian<3, 3>(f1, toVec(m1), kWrap3);
  const Mat3 J0 = numJacobian<3, 3>(f0, toVec(m0), kWrap3);
  const Mat3 cross = J0 * C01 * J1.transpose();
  const Mat3 expected =
      J1 * C1 * J1.transpose() + J0 * C0 * J0.transpose() + cross + cross.transpose();

  EXPECT_NEAR(maxAbsDiff(d.cov.asEigen(), expected), 0.0, 1e-6)
      << "got:\n"
      << d.cov.asEigen() << "\nexpected:\n"
      << expected;

  // Fully correlated case x1 == x0: the difference must be exactly zero.
  CPosePDFGaussian dSame;
  dSame.inverseComposition(x0, x0, CMatrixDouble33(C0));
  EXPECT_NEAR(dSame.cov.asEigen().cwiseAbs().maxCoeff(), 0.0, 1e-9) << dSame.cov.asEigen();
}

// Distribution of the 2D point q (+) l.
TEST(CPosePDFGaussianMath, ComposePoint_Covariance)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CPose2D mean(1.0, 2.0, 0.5);
  const Mat3 C = randomCov(0.1);
  const CPosePDFGaussian q(mean, CMatrixDouble33(C));
  const mrpt::math::TPoint2D l(3.0, -1.0);

  CPoint2DPDFGaussian g;
  q.composePoint(l, g);

  const auto f = [&](const Vec3& v) -> Eigen::Vector2d
  {
    double gx = 0;
    double gy = 0;
    toPose(v).composePoint(l.x, l.y, gx, gy);
    return {gx, gy};
  };
  const Eigen::Matrix<double, 2, 3> J = numJacobian<2, 3>(f, toVec(mean), {false, false});
  const Eigen::Matrix2d expected = J * C * J.transpose();

  EXPECT_NEAR((g.cov.asEigen() - expected).cwiseAbs().maxCoeff(), 0.0, 1e-6)
      << "got:\n"
      << g.cov.asEigen() << "\nexpected:\n"
      << expected;
}

// Fusion of N(x1,C1) and N(x2,C2) vs the closed form.
TEST(CPosePDFGaussianMath, BayesianFusion_ClosedForm)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const Mat3 C1 = randomCov(0.2);
  const Mat3 C2 = randomCov(0.2);
  const Vec3 x1(1.0, 2.0, 0.3);
  const Vec3 x2(1.2, 1.7, 0.1);

  const CPosePDFGaussian p1(toPose(x1), CMatrixDouble33(C1));
  const CPosePDFGaussian p2(toPose(x2), CMatrixDouble33(C2));
  CPosePDFGaussian r;
  r.bayesianFusion(p1, p2);

  const Mat3 S = (C1.inverse() + C2.inverse()).inverse();
  const Vec3 x = S * (C1.inverse() * x1 + C2.inverse() * x2);
  EXPECT_NEAR(maxAbsDiff(r.cov.asEigen(), S), 0.0, 1e-9);
  EXPECT_NEAR((toVec(r.mean) - x).norm(), 0.0, 1e-9);

  const CPosePDFGaussianInf p1i(toPose(x1), CMatrixDouble33(Mat3(C1.inverse())));
  const CPosePDFGaussianInf p2i(toPose(x2), CMatrixDouble33(Mat3(C2.inverse())));
  CPosePDFGaussianInf ri;
  ri.bayesianFusion(p1i, p2i);
  EXPECT_NEAR(maxAbsDiff(ri.cov_inv.asEigen().inverse(), S), 0.0, 1e-9);
  EXPECT_NEAR((toVec(ri.mean) - x).norm(), 0.0, 1e-9);
}

// Two estimates of a heading near +-pi must fuse into a heading near pi.
TEST(CPosePDFGaussianMath, BayesianFusion_AngleWrap)
{
  Mat3 C = Mat3::Identity() * 0.01;
  const CPosePDFGaussian p1(CPose2D(1.0, 0.0, M_PI - 0.05), CMatrixDouble33(C));
  const CPosePDFGaussian p2(CPose2D(1.0, 0.0, -M_PI + 0.05), CMatrixDouble33(C));
  CPosePDFGaussian r;
  r.bayesianFusion(p1, p2);
  EXPECT_NEAR(std::abs(mrpt::math::wrapToPi(r.mean.phi() - M_PI)), 0.0, 1e-9)
      << "fused phi=" << r.mean.phi();

  const CPosePDFGaussianInf p1i(p1.mean, CMatrixDouble33(Mat3(C.inverse())));
  const CPosePDFGaussianInf p2i(p2.mean, CMatrixDouble33(Mat3(C.inverse())));
  CPosePDFGaussianInf ri;
  ri.bayesianFusion(p1i, p2i);
  EXPECT_NEAR(std::abs(mrpt::math::wrapToPi(ri.mean.phi() - M_PI)), 0.0, 1e-9)
      << "fused phi=" << ri.mean.phi();
}

// evaluatePDF() and evaluateNormalizedPDF() vs the closed form, including a
// query whose heading lies on the other side of +-pi.
TEST(CPosePDFGaussianMath, EvaluatePDF_ClosedFormAndAngleWrap)
{
  Mat3 C = Mat3::Zero();
  C.diagonal() << 0.04, 0.09, 0.01;
  const CPose2D mean(1.0, 2.0, M_PI - 0.02);
  const CPosePDFGaussian p(mean, CMatrixDouble33(C));
  const CPosePDFGaussianInf pi(mean, CMatrixDouble33(Mat3(C.inverse())));

  const auto analytic = [&](const Vec3& d)
  {
    return std::exp(-0.5 * d.dot(C.inverse() * d)) /
           std::sqrt(std::pow(2 * M_PI, 3) * C.determinant());
  };

  // Same side of pi:
  const CPose2D q1(1.1, 2.1, M_PI - 0.05);
  const Vec3 d1(0.1, 0.1, -0.03);
  EXPECT_NEAR(p.evaluatePDF(q1), analytic(d1), 1e-9);
  EXPECT_NEAR(pi.evaluatePDF(q1), analytic(d1), 1e-9);
  EXPECT_NEAR(p.evaluateNormalizedPDF(q1), analytic(d1) / analytic(Vec3::Zero()), 1e-9);

  // Across +-pi: the true heading difference is +0.04 rad.
  const CPose2D q2(1.1, 2.1, -M_PI + 0.02);
  const Vec3 d2(0.1, 0.1, 0.04);
  EXPECT_NEAR(p.evaluatePDF(q2), analytic(d2), 1e-6);
  EXPECT_NEAR(pi.evaluatePDF(q2), analytic(d2), 1e-6);
  EXPECT_NEAR(p.evaluateNormalizedPDF(q2), analytic(d2) / analytic(Vec3::Zero()), 1e-6);
}

// Mahalanobis distance vs closed form, including heading wrap.
TEST(CPosePDFGaussianMath, MahalanobisDistance)
{
  Mat3 C1 = Mat3::Zero();
  C1.diagonal() << 0.04, 0.09, 0.01;
  Mat3 C2 = Mat3::Zero();
  C2.diagonal() << 0.01, 0.01, 0.03;
  CPosePDFGaussian a(CPose2D(1.0, 2.0, M_PI - 0.1), CMatrixDouble33(C1));
  CPosePDFGaussian b(CPose2D(1.3, 1.8, -M_PI + 0.1), CMatrixDouble33(C2));
  const Vec3 d(-0.3, 0.2, -0.2);
  const double expected = std::sqrt(d.dot((C1 + C2).inverse() * d));
  EXPECT_NEAR(a.mahalanobisDistanceTo(b), expected, 1e-9);

  CPosePDFGaussianInf ai(a.mean, CMatrixDouble33(Mat3(C1.inverse())));
  CPosePDFGaussianInf bi(b.mean, CMatrixDouble33(Mat3(C2.inverse())));
  EXPECT_NEAR(ai.mahalanobisDistanceTo(bi), expected, 1e-9);
}

// Sample mean and covariance of drawManySamples() / drawSingleSample().
TEST(CPosePDFGaussianMath, DrawSamples_MonteCarlo)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  Mat3 C;
  C << 0.04, 0.01, 0.002, 0.01, 0.09, -0.003, 0.002, -0.003, 0.01;
  const CPose2D mean(1.0, 2.0, M_PI - 0.05);
  const CPosePDFGaussian p(mean, CMatrixDouble33(C));
  const CPosePDFGaussianInf pi(mean, CMatrixDouble33(Mat3(C.inverse())));

  const size_t N = 100000;
  for (int method = 0; method < 3; method++)
  {
    std::vector<Vec3> s(N);
    if (method == 0)
    {
      std::vector<CVectorDouble> v;
      p.drawManySamples(N, v);
      for (size_t i = 0; i < N; i++)
      {
        s[i] = Vec3(v[i][0], v[i][1], v[i][2]);
      }
    }
    else if (method == 1)
    {
      for (size_t i = 0; i < N; i++)
      {
        CPose2D q;
        p.drawSingleSample(q);
        s[i] = toVec(q);
      }
    }
    else
    {
      std::vector<CVectorDouble> v;
      pi.drawManySamples(N, v);
      for (size_t i = 0; i < N; i++)
      {
        s[i] = Vec3(v[i][0], v[i][1], v[i][2]);
      }
    }
    Mat3 S = Mat3::Zero();
    Vec3 m = Vec3::Zero();
    for (const auto& si : s)
    {
      Vec3 d = si - toVec(mean);
      d[2] = mrpt::math::wrapToPi(d[2]);
      EXPECT_LE(std::abs(si[2]), M_PI + 1e-12);
      m += d / N;
      S += d * d.transpose() / N;
    }
    S -= m * m.transpose();
    EXPECT_NEAR(m.norm(), 0.0, 3e-3) << "method=" << method;
    EXPECT_NEAR(maxAbsDiff(S, C), 0.0, 2e-3) << "method=" << method << "\n" << S;
  }
}

// 3D information form -> 2D information form must marginalize z, pitch, roll
// (i.e. invert the 3x3 sub-block of the covariance, not take the 3x3
// sub-block of the information matrix).
TEST(CPosePDFGaussianInfMath, CopyFrom3DInf_Marginalizes)
{
  Eigen::Matrix<double, 6, 6> C6;
  C6.setIdentity();
  C6 *= 0.05;
  C6(0, 2) = C6(2, 0) = 0.03;  // x-z correlation
  C6(3, 4) = C6(4, 3) = 0.02;  // yaw-pitch correlation
  const CPose3D mean(1, 2, 3, 0.3, 0.1, -0.2);

  const CPose3DPDFGaussian g6(mean, CMatrixDouble66(C6));
  const CPose3DPDFGaussianInf g6i(mean, CMatrixDouble66(Eigen::Matrix<double, 6, 6>(C6.inverse())));

  CPosePDFGaussian g3;
  g3.copyFrom(g6);
  CPosePDFGaussianInf g3i;
  g3i.copyFrom(g6i);

  EXPECT_NEAR(g3.cov(0, 0), 0.05, 1e-12);
  EXPECT_NEAR(maxAbsDiff(g3i.cov_inv.asEigen().inverse(), g3.cov.asEigen()), 0.0, 1e-9)
      << "from Inf:\n"
      << g3i.cov_inv.asEigen().inverse() << "\nfrom Gaussian:\n"
      << g3.cov.asEigen();
}

// The information form must transform like the covariance form
TEST(CPosePDFGaussianInfMath, ChangeCoordinatesReference_MatchesCovarianceForm)
{
  mrpt::random::getRandomGenerator().randomize(4321);
  const CPose2D mean(1.0, -2.0, -2.5);
  const Mat3 C = randomCov(0.3);
  for (const double phi : {0.0, 0.4, 1.2, -2.9})
  {
    const CPose2D ref(4.0, 1.0, phi);
    CPosePDFGaussian p(mean, CMatrixDouble33(C));
    p.changeCoordinatesReference(ref);
    CPosePDFGaussianInf pi(mean, CMatrixDouble33(Mat3(C.inverse())));
    pi.changeCoordinatesReference(ref);
    EXPECT_NEAR(maxAbsDiff(pi.cov_inv.asEigen(), p.cov.asEigen().inverse()), 0.0, 1e-6)
        << "phi=" << phi;
    EXPECT_NEAR(maxAbsDiff(pi.cov_inv.asEigen(), pi.cov_inv.asEigen().transpose()), 0.0, 1e-12);
  }
}

// Point PDFs: the covariance rotates with the new reference frame
TEST(CPointPDFGaussianMath, ChangeCoordinatesReference_Covariance)
{
  mrpt::random::getRandomGenerator().randomize(99);
  const CPose3D ref(1, 2, 3, 0.7, -0.4, 1.9);
  const Mat3 C = randomCov(0.5);
  CPointPDFGaussian p(CPoint3D(1, -1, 2), CMatrixDouble33(C));
  p.changeCoordinatesReference(ref);
  const Mat3 R = ref.getRotationMatrix().asEigen();
  EXPECT_NEAR(maxAbsDiff(p.cov.asEigen(), R * C * R.transpose()), 0.0, 1e-12);

  const Eigen::Matrix2d C2 = C.block<2, 2>(0, 0);
  CPoint2DPDFGaussian p2(CPoint2D(1, -1), CMatrixDouble22(C2));
  const CPose3D ref2(1, 2, 0, 0.9, 0, 0);
  p2.changeCoordinatesReference(ref2);
  const Eigen::Matrix2d R2 = ref2.getRotationMatrix().asEigen().block<2, 2>(0, 0);
  EXPECT_NEAR((p2.cov.asEigen() - R2 * C2 * R2.transpose()).cwiseAbs().maxCoeff(), 0.0, 1e-12);
}
