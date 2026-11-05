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

// Math checks for the sum-of-Gaussians PDFs (CPosePDFSOG, CPose3DPDFSOG,
// CPointPDFSOG): mixture moments, products of mixtures, inverse, merging.

#include <gtest/gtest.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

Vec3 toVec(const CPose2D& p) { return {p.x(), p.y(), p.phi()}; }

// Gaussian density N(x; mu, S) in closed form.
template <int N>
double gaussPdf(
    const Eigen::Matrix<double, N, 1>& x,
    const Eigen::Matrix<double, N, 1>& mu,
    const Eigen::Matrix<double, N, N>& S)
{
  const Eigen::Matrix<double, N, 1> d = x - mu;
  return std::exp(-0.5 * d.dot(S.inverse() * d)) /
         std::sqrt(std::pow(2 * M_PI, N) * S.determinant());
}

// Linear weights normalized to sum 1.
template <class SOG>
std::vector<double> linearWeights(const SOG& sog)
{
  std::vector<double> w;
  double sum = 0;
  for (const auto& m : sog)
  {
    w.push_back(std::exp(m.log_w));
    sum += w.back();
  }
  for (auto& wi : w)
  {
    wi /= sum;
  }
  return w;
}

Mat3 diag3(double a, double b, double c)
{
  Mat3 M = Mat3::Zero();
  M.diagonal() << a, b, c;
  return M;
}

}  // namespace

// Mixture covariance: sum_i w_i (C_i + (m_i - m)(m_i - m)^T).
TEST(CPosePDFSOGMath, CovarianceAndMean_ClosedForm)
{
  CPosePDFSOG sog(2);
  sog[0].mean = CPose2D(0, 0, 0.1);
  sog[0].cov = CMatrixDouble33(diag3(0.01, 0.02, 0.001));
  sog[0].log_w = std::log(0.25);
  sog[1].mean = CPose2D(2, 1, 0.3);
  sog[1].cov = CMatrixDouble33(diag3(0.03, 0.01, 0.002));
  sog[1].log_w = std::log(0.75);

  const auto [C, m] = sog.getCovarianceAndMean();
  const Vec3 m0 = toVec(sog[0].mean);
  const Vec3 m1 = toVec(sog[1].mean);
  const Vec3 mu = toVec(m);
  const Mat3 expected = 0.25 * (sog[0].cov.asEigen() + (m0 - mu) * (m0 - mu).transpose()) +
                        0.75 * (sog[1].cov.asEigen() + (m1 - mu) * (m1 - mu).transpose());
  EXPECT_NEAR((C.asEigen() - expected).cwiseAbs().maxCoeff(), 0.0, 1e-9);
}

// Mixture covariance for two modes at +-(pi - 0.1): heading deviations must
// be wrapped.
TEST(CPosePDFSOGMath, CovarianceAndMean_AngleWrap)
{
  CPosePDFSOG sog(2);
  sog[0].mean = CPose2D(0, 0, M_PI - 0.1);
  sog[0].cov = CMatrixDouble33(diag3(0.01, 0.01, 0.001));
  sog[1].mean = CPose2D(0, 0, -M_PI + 0.1);
  sog[1].cov = CMatrixDouble33(diag3(0.01, 0.01, 0.001));

  const auto [C, m] = sog.getCovarianceAndMean();
  EXPECT_NEAR(std::abs(mrpt::math::wrapToPi(m.phi() - M_PI)), 0.0, 1e-6);
  EXPECT_NEAR(C(2, 2), 0.001 + 0.01, 1e-6) << C.asEigen();
}

// Product of a SOG and a Gaussian: modes and weights in closed form,
// w'_i ~ w_i * N(m_i; m, C_i + C).
// Modes have different covariances on purpose.
TEST(CPosePDFSOGMath, BayesianFusion_WeightsAndModes)
{
  CPosePDFSOG sog(2);
  sog[0].mean = CPose2D(0.0, 0.0, 0.1);
  sog[0].cov = CMatrixDouble33(diag3(0.04, 0.04, 0.01));
  sog[0].log_w = std::log(0.5);
  sog[1].mean = CPose2D(1.0, 0.5, 0.2);
  sog[1].cov = CMatrixDouble33(diag3(0.5, 0.3, 0.2));
  sog[1].log_w = std::log(0.5);

  const Mat3 C = diag3(0.1, 0.1, 0.05);
  const CPosePDFGaussian g(CPose2D(0.5, 0.2, 0.15), CMatrixDouble33(C));

  CPosePDFSOG r;
  r.bayesianFusion(sog, g);
  ASSERT_EQ(r.size(), 2u);

  std::vector<double> expectedW(2);
  double sum = 0;
  for (size_t i = 0; i < 2; i++)
  {
    const Mat3 Ci = sog[i].cov.asEigen();
    expectedW[i] = 0.5 * gaussPdf<3>(toVec(sog[i].mean), toVec(g.mean), Mat3(Ci + C));
    sum += expectedW[i];

    // Mode i must be the product of Gaussians:
    const Mat3 S = (Ci.inverse() + C.inverse()).inverse();
    const Vec3 mu = S * (Ci.inverse() * toVec(sog[i].mean) + C.inverse() * toVec(g.mean));
    EXPECT_NEAR((r[i].cov.asEigen() - S).cwiseAbs().maxCoeff(), 0.0, 1e-9);
    EXPECT_NEAR((toVec(r[i].mean) - mu).norm(), 0.0, 1e-9);
  }
  const auto w = linearWeights(r);
  for (size_t i = 0; i < 2; i++)
  {
    EXPECT_NEAR(w[i], expectedW[i] / sum, 1e-6) << "mode " << i;
  }
}

// inverse() of each mode, keeping the weights.
TEST(CPosePDFSOGMath, Inverse_MatchesGaussianInverse)
{
  CPosePDFSOG sog(2);
  sog[0].mean = CPose2D(1.0, 2.0, 0.5);
  sog[0].cov = CMatrixDouble33(diag3(0.04, 0.02, 0.01));
  sog[0].log_w = std::log(0.2);
  sog[1].mean = CPose2D(-1.0, 0.5, -1.0);
  sog[1].cov = CMatrixDouble33(diag3(0.01, 0.03, 0.02));
  sog[1].log_w = std::log(0.8);

  CPosePDFSOG inv;
  sog.inverse(inv);
  ASSERT_EQ(inv.size(), 2u);
  for (size_t i = 0; i < 2; i++)
  {
    const CPosePDFGaussian g(sog[i].mean, sog[i].cov);
    CPosePDFGaussian gInv;
    g.inverse(gInv);
    EXPECT_NEAR((toVec(inv[i].mean) - toVec(gInv.mean)).norm(), 0.0, 1e-9) << "mode " << i;
    EXPECT_NEAR((inv[i].cov.asEigen() - gInv.cov.asEigen()).cwiseAbs().maxCoeff(), 0.0, 1e-9)
        << "mode " << i;
    EXPECT_NEAR(inv[i].log_w, sog[i].log_w, 1e-12) << "mode " << i;
  }
}

// operator+=(CPose2D) must apply the composition Jacobian to each mode.
TEST(CPosePDFSOGMath, OperatorPlusEqualsPose2D_MatchesGaussian)
{
  CPosePDFSOG sog(1);
  sog[0].mean = CPose2D(1.0, 2.0, 0.5);
  sog[0].cov = CMatrixDouble33(diag3(0.04, 0.02, 0.01));
  const CPose2D Ap(3.0, -1.0, 0.7);

  // Expected: Jacobian of x (+) Ap w.r.t. x
  CMatrixDouble33 J;
  CMatrixDouble33 dummy;
  CPosePDF::jacobiansPoseComposition(sog[0].mean, Ap, J, dummy, true, false);
  const Mat3 expected = J.asEigen() * sog[0].cov.asEigen() * J.asEigen().transpose();

  sog += Ap;
  EXPECT_NEAR((sog[0].cov.asEigen() - expected).cwiseAbs().maxCoeff(), 0.0, 1e-9);
}

// mergeModes() must preserve the first two moments of the merged pair.
TEST(CPosePDFSOGMath, MergeModes_MomentPreserving)
{
  CPosePDFSOG sog(2);
  sog[0].mean = CPose2D(0.0, 0.0, 0.1);
  sog[0].cov = CMatrixDouble33(diag3(0.04, 0.04, 0.01));
  sog[0].log_w = std::log(0.3);
  sog[1].mean = CPose2D(0.1, -0.05, 0.12);
  sog[1].cov = CMatrixDouble33(diag3(0.05, 0.03, 0.01));
  sog[1].log_w = std::log(0.7);
  const Vec3 m0 = toVec(sog[0].mean);
  const Vec3 m1 = toVec(sog[1].mean);
  const Vec3 mu = 0.3 * m0 + 0.7 * m1;
  const Mat3 expectedCov = 0.3 * (sog[0].cov.asEigen() + (m0 - mu) * (m0 - mu).transpose()) +
                           0.7 * (sog[1].cov.asEigen() + (m1 - mu) * (m1 - mu).transpose());

  sog.mergeModes(1e9);
  ASSERT_EQ(sog.size(), 1u);
  EXPECT_NEAR((toVec(sog[0].mean) - mu).norm(), 0.0, 1e-9);
  EXPECT_NEAR((sog[0].cov.asEigen() - expectedCov).cwiseAbs().maxCoeff(), 0.0, 1e-9);
}

// Merging two modes at +-(pi - 0.01) must yield a heading of pi.
TEST(CPosePDFSOGMath, MergeModes_AngleWrap)
{
  CPosePDFSOG sog(2);
  for (size_t i = 0; i < 2; i++)
  {
    sog[i].cov = CMatrixDouble33(diag3(0.04, 0.04, 0.01));
    sog[i].log_w = 0;
  }
  sog[0].mean = CPose2D(0.0, 0.0, M_PI - 0.01);
  sog[1].mean = CPose2D(0.0, 0.0, -M_PI + 0.01);

  sog.mergeModes(0.5);
  ASSERT_EQ(sog.size(), 1u);
  EXPECT_NEAR(std::abs(mrpt::math::wrapToPi(sog[0].mean.phi() - M_PI)), 0.0, 1e-6)
      << "merged phi=" << sog[0].mean.phi();
}

// Regression: the merge criterion was twice the upper bound B(i,j) of the KL
// divergence of eq. (21) in Runnalls 2007. Here, B(i,j) is computed as
// wi*KL(Ni||Nij) + wj*KL(Nj||Nij), which equals it for moment-matched merges.
TEST(CPosePDFSOGMath, MergeModes_ThresholdIsKLBound)
{
  const double wi = 0.4;
  const double wj = 0.6;
  const Vec3 mi(0, 0, 0);
  const Vec3 mj(0.3, -0.1, 0.05);
  const Mat3 Pi = diag3(0.04, 0.09, 0.01);
  const Mat3 Pj = diag3(0.09, 0.04, 0.02);
  const Vec3 mij = wi * mi + wj * mj;
  const Mat3 Pij = wi * Pi + wj * Pj + wi * wj * (mi - mj) * (mi - mj).transpose();
  const auto kl = [](const Vec3& m0, const Mat3& S0, const Vec3& m1, const Mat3& S1)
  {
    const Mat3 S1i = S1.inverse();
    const Vec3 d = m1 - m0;
    return 0.5 * ((S1i * S0).trace() + d.dot(S1i * d) - 3 +
                  std::log(S1.determinant() / S0.determinant()));
  };
  const double B = wi * kl(mi, Pi, mij, Pij) + wj * kl(mj, Pj, mij, Pij);

  for (const double factor : {0.95, 1.05})
  {
    CPosePDFSOG sog(2);
    sog[0].mean = CPose2D(mi[0], mi[1], mi[2]);
    sog[0].cov = CMatrixDouble33(Pi);
    sog[0].log_w = std::log(wi);
    sog[1].mean = CPose2D(mj[0], mj[1], mj[2]);
    sog[1].cov = CMatrixDouble33(Pj);
    sog[1].log_w = std::log(wj);
    sog.mergeModes(factor * B);
    EXPECT_EQ(sog.size(), factor < 1 ? 2u : 1u) << "B=" << B;
  }
}

// CPose3DPDFSOG mixture covariance: the mode means are centered on the
// mixture mean.
TEST(CPose3DPDFSOGMath, CovarianceAndMean_ClosedForm)
{
  CPose3DPDFSOG sog(1);
  Mat6 C = Mat6::Identity() * 0.01;
  sog.begin()->val = CPose3DPDFGaussian(CPose3D(10, 0, 0, 0, 0, 0), CMatrixDouble66(C));
  sog.begin()->log_w = 0;

  const auto [Cout, m] = sog.getCovarianceAndMean();
  EXPECT_NEAR(m.x(), 10.0, 1e-9);
  EXPECT_NEAR((Cout.asEigen() - C).cwiseAbs().maxCoeff(), 0.0, 1e-9) << Cout.asEigen();
}

// CPose3DPDFSOG::bayesianFusion() weights include the overlap factor
// N(m1_i; m2_j, C1_i + C2_j): a mode far away from the other density gets a
// negligible weight.
TEST(CPose3DPDFSOGMath, BayesianFusion_Weights)
{
  const Mat6 C = Mat6::Identity() * 0.01;
  CPose3DPDFSOG p1(2);
  auto it = p1.begin();
  it->val = CPose3DPDFGaussian(CPose3D(0, 0, 0, 0, 0, 0), CMatrixDouble66(C));
  it->log_w = 0;
  ++it;
  it->val = CPose3DPDFGaussian(CPose3D(1.5, 0, 0, 0, 0, 0), CMatrixDouble66(C));
  it->log_w = 0;

  CPose3DPDFSOG p2(1);
  p2.begin()->val = CPose3DPDFGaussian(CPose3D(0.05, 0, 0, 0, 0, 0), CMatrixDouble66(C));
  p2.begin()->log_w = 0;

  CPose3DPDFSOG r;
  r.bayesianFusion(p1, p2);
  ASSERT_EQ(r.size(), 2u);

  Vec6 a = Vec6::Zero();
  Vec6 b = Vec6::Zero();
  Vec6 c = Vec6::Zero();
  b[0] = 1.5;
  c[0] = 0.05;
  const double w0 = gaussPdf<6>(a, c, Mat6(2 * C));
  const double w1 = gaussPdf<6>(b, c, Mat6(2 * C));
  const auto w = linearWeights(r);
  EXPECT_NEAR(w[0], w0 / (w0 + w1), 1e-6);
  EXPECT_NEAR(w[1], w1 / (w0 + w1), 1e-6);
}

// Product of two point SOGs: weights w1_i*w2_j*N(m1_i; m2_j, C1_i + C2_j).
TEST(CPointPDFSOGMath, BayesianFusion_Weights)
{
  CPointPDFSOG p1(2);
  p1[0].val = CPointPDFGaussian(CPoint3D(0, 0, 0), CMatrixDouble33(diag3(0.04, 0.04, 0.04)));
  p1[0].log_w = 0;
  p1[1].val = CPointPDFGaussian(CPoint3D(1, 0.5, 0), CMatrixDouble33(diag3(0.5, 0.3, 0.2)));
  p1[1].log_w = 0;
  CPointPDFSOG p2(1);
  p2[0].val = CPointPDFGaussian(CPoint3D(0.5, 0.2, 0.1), CMatrixDouble33(diag3(0.1, 0.1, 0.1)));
  p2[0].log_w = 0;

  CPointPDFSOG r;
  r.bayesianFusion(p1, p2);
  ASSERT_EQ(r.size(), 2u);

  std::vector<double> expectedW(2);
  double sum = 0;
  for (size_t i = 0; i < 2; i++)
  {
    const auto& m1 = p1[i].val;
    const Vec3 mu1(m1.mean.x(), m1.mean.y(), m1.mean.z());
    const Vec3 mu2(p2[0].val.mean.x(), p2[0].val.mean.y(), p2[0].val.mean.z());
    expectedW[i] = gaussPdf<3>(mu1, mu2, Mat3(m1.cov.asEigen() + p2[0].val.cov.asEigen()));
    sum += expectedW[i];
  }
  const auto w = linearWeights(r);
  for (size_t i = 0; i < 2; i++)
  {
    EXPECT_NEAR(w[i], expectedW[i] / sum, 1e-6) << "mode " << i;
  }
}

// Mode selection frequencies of drawSingleSample() follow the weights.
TEST(CPosePDFSOGMath, DrawSingleSample_ModeFrequencies)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CPosePDFSOG sog(2);
  sog[0].mean = CPose2D(-10, 0, 0);
  sog[0].cov = CMatrixDouble33(diag3(0.01, 0.01, 0.001));
  sog[0].log_w = std::log(0.2) + 5.0;  // unnormalized on purpose
  sog[1].mean = CPose2D(10, 0, 0);
  sog[1].cov = CMatrixDouble33(diag3(0.01, 0.01, 0.001));
  sog[1].log_w = std::log(0.8) + 5.0;

  const size_t N = 20000;
  size_t n1 = 0;
  for (size_t i = 0; i < N; i++)
  {
    CPose2D p;
    sog.drawSingleSample(p);
    if (p.x() > 0)
    {
      n1++;
    }
  }
  EXPECT_NEAR(static_cast<double>(n1) / N, 0.8, 0.015);
}

// The RTTI parent of CPointPDFSOG must be CPointPDF.
TEST(CPointPDFSOGMath, RuntimeClassHierarchy)
{
  const CPointPDFSOG sog;
  EXPECT_TRUE(IS_DERIVED(sog, CPointPDF));
  EXPECT_FALSE(IS_DERIVED(sog, CPosePDF));
}

// Product of a planar (z variance = 0) point SOG with a 3D point SOG: the
// class explicitly tries to support this case.
TEST(CPointPDFSOGMath, BayesianFusion_PlanarModes)
{
  CPointPDFSOG p1(1);
  p1[0].val = CPointPDFGaussian(CPoint3D(0, 0, 0), CMatrixDouble33(diag3(0.04, 0.04, 0.0)));
  p1[0].log_w = 0;
  CPointPDFSOG p2(1);
  p2[0].val = CPointPDFGaussian(CPoint3D(0.1, 0.1, 0), CMatrixDouble33(diag3(0.04, 0.04, 0.0)));
  p2[0].log_w = 0;
  CPointPDFSOG r;
  bool ok = true;
  try
  {
    r.bayesianFusion(p1, p2);
  }
  catch (const std::exception&)
  {
    ok = false;
  }
  ASSERT_TRUE(ok);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_NEAR(r[0].val.mean.x(), 0.05, 1e-9);
  EXPECT_NEAR(r[0].val.cov(0, 0), 0.02, 1e-9);
}

// A planar mode must not make the other fused modes planar too.
TEST(CPointPDFSOGMath, BayesianFusion_MixedPlanarAnd3DModes)
{
  CPointPDFSOG p1(2);
  p1[0].val = CPointPDFGaussian(CPoint3D(0, 0, 0), CMatrixDouble33(diag3(0.04, 0.04, 0.0)));
  p1[0].log_w = 0;
  p1[1].val = CPointPDFGaussian(CPoint3D(0, 0, 0), CMatrixDouble33(diag3(0.04, 0.04, 0.04)));
  p1[1].log_w = 0;
  CPointPDFSOG p2(1);
  p2[0].val = CPointPDFGaussian(CPoint3D(0.1, 0.1, 0), CMatrixDouble33(diag3(0.04, 0.04, 0.04)));
  p2[0].log_w = 0;
  CPointPDFSOG r;
  r.bayesianFusion(p1, p2);
  ASSERT_EQ(r.size(), 2u);
  EXPECT_NEAR(r[0].val.cov(2, 2), 0.0, 1e-12);
  EXPECT_NEAR(r[1].val.cov(2, 2), 0.02, 1e-9);
}

// Mode selection frequencies of CPointPDFSOG::drawSingleSample().
TEST(CPointPDFSOGMath, DrawSingleSample_ModeFrequencies)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CPointPDFSOG sog(3);
  const double w[3] = {0.5, 0.5, 2.0};  // unnormalized on purpose
  for (size_t i = 0; i < 3; i++)
  {
    sog[i].val = CPointPDFGaussian(
        CPoint3D(10.0 * static_cast<double>(i), 0, 0), CMatrixDouble33(diag3(0.01, 0.01, 0.01)));
    sog[i].log_w = std::log(w[i]) - 3.0;
  }
  const size_t N = 20000;
  size_t n[3] = {0, 0, 0};
  for (size_t k = 0; k < N; k++)
  {
    CPoint3D p;
    sog.drawSingleSample(p);
    const auto i = static_cast<size_t>(std::lround(p.x() / 10.0));
    ASSERT_LT(i, 3u);
    n[i]++;
  }
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_NEAR(static_cast<double>(n[i]) / N, w[i] / 3.0, 0.015) << "mode " << i;
  }
}
