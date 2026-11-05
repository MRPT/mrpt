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

// Checks mrpt::tfest against independent closed-form solutions.

#include <gtest/gtest.h>
#include <mrpt/core/cpu.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/tfest/se2.h>
#include <mrpt/tfest/se3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>

using mrpt::math::TPoint3D;
using mrpt::math::TPose2D;
using mrpt::tfest::TMatchingPair;
using mrpt::tfest::TMatchingPairList;

namespace
{
mrpt::random::CRandomGenerator& rng() { return mrpt::random::getRandomGenerator(); }

// Correspondences global = q (+) local (+ optional noise on both sets).
TMatchingPairList makeSE2List(
    const TPose2D& q, size_t n, double spread, double offset, double noiseStd)
{
  TMatchingPairList L;
  const double c = std::cos(q.phi);
  const double s = std::sin(q.phi);
  for (size_t i = 0; i < n; i++)
  {
    const double lx = rng().drawUniform(-spread, spread);
    const double ly = rng().drawUniform(-spread, spread);
    const double gx = q.x + c * lx - s * ly;
    const double gy = q.y + s * lx + c * ly;
    TMatchingPair p;
    p.globalIdx = static_cast<uint32_t>(i);
    p.localIdx = static_cast<uint32_t>(i);
    p.global = {
        static_cast<float>(offset + gx + noiseStd * rng().drawGaussian1D_normalized()),
        static_cast<float>(offset + gy + noiseStd * rng().drawGaussian1D_normalized()), 0.0f};
    p.local = {
        static_cast<float>(offset + lx + noiseStd * rng().drawGaussian1D_normalized()),
        static_cast<float>(offset + ly + noiseStd * rng().drawGaussian1D_normalized()), 0.0f};
    L.push_back(p);
  }
  return L;
}

// Independent double-precision, centered (two-pass) 2D LS solution.
TPose2D refSE2(const TMatchingPairList& L)
{
  double mgx = 0;
  double mgy = 0;
  double mlx = 0;
  double mly = 0;
  for (const auto& p : L)
  {
    mgx += p.global.x;
    mgy += p.global.y;
    mlx += p.local.x;
    mly += p.local.y;
  }
  const double n = static_cast<double>(L.size());
  mgx /= n;
  mgy /= n;
  mlx /= n;
  mly /= n;
  double dotSum = 0;
  double crossSum = 0;
  for (const auto& p : L)
  {
    const double ax = p.global.x - mgx;
    const double ay = p.global.y - mgy;
    const double bx = p.local.x - mlx;
    const double by = p.local.y - mly;
    dotSum += ax * bx + ay * by;
    crossSum += bx * ay - by * ax;
  }
  TPose2D q;
  q.phi = std::atan2(crossSum, dotSum);
  q.x = mgx - (std::cos(q.phi) * mlx - std::sin(q.phi) * mly);
  q.y = mgy - (std::sin(q.phi) * mlx + std::cos(q.phi) * mly);
  return q;
}

bool runSE2(const TMatchingPairList& L, TPose2D& q, mrpt::math::CMatrixDouble33* cov, bool useSSE2)
{
  const bool hadSSE2 = mrpt::cpu::supports(mrpt::cpu::feature::SSE2);
  mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, useSSE2 && hadSSE2);
  const bool ok = mrpt::tfest::se2_l2(L, q, cov);
  mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, hadSSE2);
  return ok;
}

Eigen::Matrix3Xd toEigen(const std::vector<TPoint3D>& pts)
{
  Eigen::Matrix3Xd M(3, pts.size());
  for (size_t i = 0; i < pts.size(); i++)
  {
    M.col(i) << pts[i].x, pts[i].y, pts[i].z;
  }
  return M;
}

void makeSE3Points(
    size_t n,
    double scale,
    double noiseStd,
    std::vector<TPoint3D>& pThis,
    std::vector<TPoint3D>& pOther,
    mrpt::poses::CPose3D& gt)
{
  gt = mrpt::poses::CPose3D(1.0, -2.0, 0.5, 0.7, -0.3, 0.2);
  pThis.clear();
  pOther.clear();
  for (size_t i = 0; i < n; i++)
  {
    const TPoint3D b(rng().drawUniform(-4, 4), rng().drawUniform(-4, 4), rng().drawUniform(-4, 4));
    // this = t + s * R * other
    TPoint3D r = gt.rotateVector(b * scale);
    r += gt.translation();
    r.x += noiseStd * rng().drawGaussian1D_normalized();
    r.y += noiseStd * rng().drawGaussian1D_normalized();
    r.z += noiseStd * rng().drawGaussian1D_normalized();
    pThis.push_back(r);
    pOther.push_back(b);
  }
}

double maxResidual(
    const std::vector<TPoint3D>& pThis,
    const std::vector<TPoint3D>& pOther,
    const mrpt::poses::CPose3DQuat& q,
    double s)
{
  double m = 0;
  for (size_t i = 0; i < pThis.size(); i++)
  {
    TPoint3D r = mrpt::poses::CPose3D(q).rotateVector(pOther[i] * s);
    r += TPoint3D(q.x(), q.y(), q.z());
    m = std::max(m, (r - pThis[i]).norm());
  }
  return m;
}

}  // namespace

TEST(TfestReference, se2_l2_MatchesCenteredDoubleSolution)
{
  rng().randomize(1234);
  for (int trial = 0; trial < 20; trial++)
  {
    const TPose2D gt(
        rng().drawUniform(-3, 3), rng().drawUniform(-3, 3), rng().drawUniform(-M_PI, M_PI));
    const auto L = makeSE2List(gt, 40, 5.0, 0.0, 0.05);
    const TPose2D ref = refSE2(L);
    for (const bool sse : {true, false})
    {
      TPose2D q;
      ASSERT_TRUE(runSE2(L, q, nullptr, sse));
      EXPECT_NEAR(q.x, ref.x, 1e-4) << "sse=" << sse;
      EXPECT_NEAR(q.y, ref.y, 1e-4) << "sse=" << sse;
      EXPECT_NEAR(mrpt::math::angDistance(q.phi, ref.phi), 0.0, 1e-5) << "sse=" << sse;
    }
  }
}

TEST(TfestReference, se2_l2_SSE2AndScalarPathsAgree)
{
  if (!mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
  {
    GTEST_SKIP() << "No SSE2";
  }
  rng().randomize(1234);
  for (const size_t n : {2u, 3u, 17u, 1000u})
  {
    const auto L = makeSE2List(TPose2D(0.3, -0.7, 2.1), n, 3.0, 2.0, 0.02);
    TPose2D q1;
    TPose2D q2;
    mrpt::math::CMatrixDouble33 c1;
    mrpt::math::CMatrixDouble33 c2;
    ASSERT_TRUE(runSE2(L, q1, &c1, true));
    ASSERT_TRUE(runSE2(L, q2, &c2, false));
    // Same float operations in the same order: results should be identical
    // up to compiler contraction differences.
    EXPECT_NEAR(q1.x, q2.x, 1e-5) << n;
    EXPECT_NEAR(q1.y, q2.y, 1e-5) << n;
    EXPECT_NEAR(q1.phi, q2.phi, 1e-6) << n;
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        EXPECT_NEAR(c1(r, c), c2(r, c), 1e-6 * (1.0 + std::abs(c2(r, c)))) << n;
      }
    }
  }
}

// Point clouds far from the origin must not lose accuracy.
TEST(TfestReference, se2_l2_AccurateFarFromOrigin)
{
  rng().randomize(1234);
  const TPose2D gt(2.0, -1.0, 0.3);
  for (const double offset : {100.0, 1000.0, 5000.0})
  {
    const auto L = makeSE2List(gt, 200, 5.0, offset, 0.0);
    const TPose2D ref = refSE2(L);
    // The float inputs themselves allow ~1e-5 rad accuracy here.
    EXPECT_NEAR(mrpt::math::angDistance(ref.phi, gt.phi), 0.0, 1e-4) << offset;
    TPose2D q;
    ASSERT_TRUE(mrpt::tfest::se2_l2(L, q));
    EXPECT_NEAR(mrpt::math::angDistance(q.phi, gt.phi), 0.0, 1e-4) << "offset=" << offset;
  }
}

// With all local points coincident the rotation is undefined: no NaN/Inf
// covariance may be reported as a success.
TEST(TfestReference, se2_l2_DegenerateInputCovarianceIsFiniteOrFails)
{
  TMatchingPairList L;
  L.emplace_back(0, 0, 1.0f, 2.0f, 0.0f, 0.5f, 0.5f, 0.0f);
  L.emplace_back(1, 1, 3.0f, 1.0f, 0.0f, 0.5f, 0.5f, 0.0f);
  TPose2D q;
  mrpt::math::CMatrixDouble33 cov;
  const bool ok = mrpt::tfest::se2_l2(L, q, &cov);
  if (ok)
  {
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        EXPECT_TRUE(std::isfinite(cov(r, c))) << "(" << r << "," << c << ")";
      }
    }
  }
}

TEST(TfestReference, se2_l2_CovarianceMatchesMonteCarlo)
{
  rng().randomize(1234);
  const TPose2D gt(1.0, 2.0, 0.4);
  const double sigma = 0.02;
  // Fixed geometry, off-center so all cross terms are non-zero:
  rng().randomize(1);
  const auto L0 = makeSE2List(gt, 12, 3.0, 0.0, 0.0);
  std::vector<std::array<double, 3>> samples;
  mrpt::math::CMatrixDouble33 covN;
  rng().randomize(1234);
  const size_t nMC = 6000;
  for (size_t k = 0; k < nMC; k++)
  {
    TMatchingPairList L = L0;
    for (auto& p : L)
    {
      p.global.x += static_cast<float>(sigma * rng().drawGaussian1D_normalized());
      p.global.y += static_cast<float>(sigma * rng().drawGaussian1D_normalized());
      p.local.x += static_cast<float>(sigma * rng().drawGaussian1D_normalized());
      p.local.y += static_cast<float>(sigma * rng().drawGaussian1D_normalized());
    }
    TPose2D q;
    ASSERT_TRUE(mrpt::tfest::se2_l2(L, q, k == 0 ? &covN : nullptr));
    samples.push_back({q.x, q.y, mrpt::math::wrapToPi(q.phi - gt.phi)});
  }
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  Eigen::Vector3d m = Eigen::Vector3d::Zero();
  for (const auto& s : samples)
  {
    m += Eigen::Vector3d(s[0], s[1], s[2]);
  }
  m /= static_cast<double>(nMC);
  for (const auto& s : samples)
  {
    const Eigen::Vector3d d = Eigen::Vector3d(s[0], s[1], s[2]) - m;
    C += d * d.transpose();
  }
  C /= static_cast<double>(nMC - 1);
  // Analytic: normalized covariance times the per-coordinate variance.
  const Eigen::Matrix3d A = covN.asEigen() * (sigma * sigma);
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      const double scaleRC = std::sqrt(A(r, r) * A(c, c));
      EXPECT_NEAR(C(r, c) / scaleRC, A(r, c) / scaleRC, 0.08) << "(" << r << "," << c << ")\nMC:\n"
                                                              << C << "\nanalytic:\n"
                                                              << A;
    }
  }
}

TEST(TfestReference, se3_l2_RigidMatchesUmeyama)
{
  rng().randomize(1234);
  std::vector<TPoint3D> a;
  std::vector<TPoint3D> b;
  mrpt::poses::CPose3D gt;
  makeSE3Points(30, 1.0, 0.05, a, b, gt);
  mrpt::poses::CPose3DQuat q;
  double s = 0;
  ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q, s, true));
  EXPECT_EQ(s, 1.0);
  const Eigen::Matrix4d T = Eigen::umeyama(toEigen(b), toEigen(a), false);
  const auto R = mrpt::poses::CPose3D(q).getRotationMatrix();
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_NEAR(R(r, c), T(r, c), 1e-9);
    }
  }
  EXPECT_NEAR(q.x(), T(0, 3), 1e-9);
  EXPECT_NEAR(q.y(), T(1, 3), 1e-9);
  EXPECT_NEAR(q.z(), T(2, 3), 1e-9);
}

// The estimated scale must be the one mapping "other" into "this".
TEST(TfestReference, se3_l2_RecoversScale)
{
  rng().randomize(1234);
  for (const double scale : {0.5, 2.0, 3.7})
  {
    std::vector<TPoint3D> a;
    std::vector<TPoint3D> b;
    mrpt::poses::CPose3D gt;
    makeSE3Points(20, scale, 0.0, a, b, gt);
    mrpt::poses::CPose3DQuat q;
    double s = 0;
    ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q, s, false));
    EXPECT_NEAR(s, scale, 1e-9) << "true scale=" << scale;
    EXPECT_LT(maxResidual(a, b, q, s), 1e-9) << "true scale=" << scale;
  }
}

TEST(TfestReference, se3_l2_RotationIndependentOfScaleOption)
{
  rng().randomize(1234);
  std::vector<TPoint3D> a;
  std::vector<TPoint3D> b;
  mrpt::poses::CPose3D gt;
  makeSE3Points(25, 1.6, 0.03, a, b, gt);
  mrpt::poses::CPose3DQuat q1;
  mrpt::poses::CPose3DQuat q2;
  double s1 = 0;
  double s2 = 0;
  ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q1, s1, true));
  ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q2, s2, false));
  const Eigen::Matrix4d T = Eigen::umeyama(toEigen(b), toEigen(a), true);
  const auto R1 = mrpt::poses::CPose3D(q1).getRotationMatrix();
  const auto R2 = mrpt::poses::CPose3D(q2).getRotationMatrix();
  const double sU = T.block<3, 3>(0, 0).col(0).norm();
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_NEAR(R1(r, c), R2(r, c), 1e-12);
      EXPECT_NEAR(R1(r, c), T(r, c) / sU, 1e-9);
    }
  }
}

TEST(TfestReference, se3_l2_ThreeNonCollinearPointsExact)
{
  rng().randomize(1234);
  std::vector<TPoint3D> a;
  std::vector<TPoint3D> b;
  mrpt::poses::CPose3D gt;
  makeSE3Points(3, 1.0, 0.0, a, b, gt);
  mrpt::poses::CPose3DQuat q;
  double s = 0;
  ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q, s, true));
  EXPECT_LT((mrpt::poses::CPose3D(q).asVectorVal() - gt.asVectorVal()).norm(), 1e-9);
}

TEST(TfestReference, se3_l2_CollinearPointsStillAlign)
{
  // Rotation about the common line is unobservable, but any returned
  // solution must still map every point onto its correspondence.
  const mrpt::poses::CPose3D gt(1.0, 2.0, 3.0, 0.4, 0.1, -0.2);
  std::vector<TPoint3D> a;
  std::vector<TPoint3D> b;
  for (int i = 0; i < 6; i++)
  {
    const TPoint3D p(0.5 * i, -0.25 * i, 1.0 * i);
    TPoint3D r;
    gt.composePoint(p, r);
    a.push_back(r);
    b.push_back(p);
  }
  mrpt::poses::CPose3DQuat q;
  double s = 0;
  ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q, s, true));
  EXPECT_LT(maxResidual(a, b, q, 1.0), 1e-9);
}

TEST(TfestReference, se3_l2_MatchingListOverloadsAgree)
{
  rng().randomize(1234);
  std::vector<TPoint3D> a;
  std::vector<TPoint3D> b;
  mrpt::poses::CPose3D gt;
  makeSE3Points(15, 1.0, 0.01, a, b, gt);
  mrpt::tfest::TMatchingPairList_d Ld;
  for (size_t i = 0; i < a.size(); i++)
  {
    Ld.emplace_back(
        static_cast<uint32_t>(i), static_cast<uint32_t>(i), a[i].x, a[i].y, a[i].z, b[i].x, b[i].y,
        b[i].z);
  }
  mrpt::poses::CPose3DQuat q1;
  mrpt::poses::CPose3DQuat q2;
  double s1 = 0;
  double s2 = 0;
  ASSERT_TRUE(mrpt::tfest::se3_l2(a, b, q1, s1, true));
  ASSERT_TRUE(mrpt::tfest::se3_l2(Ld, q2, s2, true));
  for (int i = 0; i < 7; i++)
  {
    EXPECT_NEAR(q1[i], q2[i], 1e-12);
  }
}

// The saved MATLAB script must draw each correspondence as a 3D segment.
TEST(TfestReference, SaveAsMATLABScriptDrawsSegments)
{
  TMatchingPairList L;
  L.emplace_back(0, 0, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);
  const std::string fil = mrpt::system::getTempFileName();
  L.saveAsMATLABScript(fil);
  std::ifstream f(fil);
  ASSERT_TRUE(f.is_open());
  std::string line;
  std::string lineCmd;
  while (std::getline(f, line))
  {
    if (line.rfind("line(", 0) == 0)
    {
      lineCmd = line;
      break;
    }
  }
  ASSERT_FALSE(lineCmd.empty());
  // Extract all bracketed arrays: expect X=[gx lx], Y=[gy ly], Z=[gz lz].
  std::vector<std::vector<double>> arrays;
  const std::regex arr(R"(\[([^\]]*)\])");
  for (auto it = std::sregex_iterator(lineCmd.begin(), lineCmd.end(), arr);
       it != std::sregex_iterator(); ++it)
  {
    std::istringstream ss((*it)[1].str());
    std::vector<double> v;
    double d = 0;
    while (ss >> d)
    {
      v.push_back(d);
    }
    arrays.push_back(v);
  }
  // Skip a possible trailing 'Color' array:
  ASSERT_GE(arrays.size(), 3u) << lineCmd;
  const std::vector<std::vector<double>> expected = {
      {1.0, 4.0},
      {2.0, 5.0},
      {3.0, 6.0}
  };
  for (size_t k = 0; k < 3; k++)
  {
    EXPECT_EQ(arrays[k], expected[k]) << lineCmd;
  }
}

// The RANSAC early exit compares the RMSE against max_rmse_to_end.
TEST(TfestReference, se2_l2_robust_EarlyExitUsesRMSE)
{
  rng().randomize(1234);
  const auto L = makeSE2List(TPose2D(1.0, 0.5, 0.2), 30, 10.0, 0.0, 0.1);
  // Actual RMSE of the full set (about 0.2, i.e. MSE about 0.04):
  const TPose2D ref = refSE2(L);
  double sse = 0;
  for (const auto& p : L)
  {
    const double gx = ref.x + std::cos(ref.phi) * p.local.x - std::sin(ref.phi) * p.local.y;
    const double gy = ref.y + std::sin(ref.phi) * p.local.x + std::cos(ref.phi) * p.local.y;
    sse += (gx - p.global.x) * (gx - p.global.x) + (gy - p.global.y) * (gy - p.global.y);
  }
  const double rmse = std::sqrt(sse / static_cast<double>(L.size()));
  ASSERT_GT(rmse, 0.15);
  ASSERT_LT(rmse * rmse, 0.1);

  mrpt::tfest::TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = 30;
  params.ransac_nSimulations = 10;
  params.ransac_mahalanobisDistanceThreshold = 1e6;
  params.max_rmse_to_end = 0.1;  // below the actual RMSE: must not stop early
  params.verbose = true;
  mrpt::tfest::TSE2RobustResult res;
  testing::internal::CaptureStdout();
  const bool ok = mrpt::tfest::se2_l2_robust(L, 0.1, params, res);
  const std::string out = testing::internal::GetCapturedStdout();
  ASSERT_TRUE(ok);
  EXPECT_NE(out.find("Finished after 10 iterations"), std::string::npos) << out;
  EXPECT_EQ(res.ransac_iters, 10u);

  // A threshold above the actual RMSE must stop after the first iteration:
  params.max_rmse_to_end = 0.3;
  params.verbose = false;
  ASSERT_TRUE(mrpt::tfest::se2_l2_robust(L, 0.1, params, res));
  EXPECT_EQ(res.ransac_iters, 1u);
}

// Regression: with ransac_fuseByCorrsMatch, a consensus set found again in a
// later iteration (in a different, random order) was added as a new mode
// instead of increasing the weight of the existing one.
TEST(TfestReference, se2_l2_robust_FusesRepeatedConsensusSets)
{
  rng().randomize(1234);
  // Two groups of correspondences, consistent with two different poses:
  const TPose2D qA(1.0, 0.5, 0.2);
  const TPose2D qB(-2.0, 1.0, -0.8);
  TMatchingPairList L = makeSE2List(qA, 15, 10.0, 0.0, 0.01);
  const TMatchingPairList LB = makeSE2List(qB, 10, 10.0, 0.0, 0.01);
  for (auto p : LB)
  {
    p.globalIdx += 100;
    p.localIdx += 100;
    L.push_back(p);
  }

  mrpt::tfest::TSE2RobustParams params;
  params.ransac_minSetSize = 5;
  params.ransac_maxSetSize = 30;
  params.ransac_nSimulations = 100;
  params.max_rmse_to_end = 1e-9;  // run all the iterations
  params.ransac_fuseByCorrsMatch = true;
  mrpt::tfest::TSE2RobustResult res;
  ASSERT_TRUE(mrpt::tfest::se2_l2_robust(L, 0.01, params, res));
  ASSERT_GE(res.transformation.size(), 2u);

  // No two modes from the same consensus set:
  for (size_t i = 0; i < res.transformation.size(); i++)
  {
    for (size_t j = i + 1; j < res.transformation.size(); j++)
    {
      const auto& a = res.transformation.get(i).mean;
      const auto& b = res.transformation.get(j).mean;
      EXPECT_GT(
          std::abs(a.x() - b.x()) + std::abs(a.y() - b.y()) + std::abs(a.phi() - b.phi()), 1e-6)
          << "modes " << i << " and " << j << " are duplicates: " << a;
    }
  }

  // Both hypotheses are found, with weights proportional to the number of
  // times their consensus sets were drawn:
  double wA = 0;
  double wB = 0;
  double wSum = 0;
  for (const auto& m : res.transformation)
  {
    const double w = std::exp(m.log_w);
    wSum += w;
    if (std::abs(m.mean.x() - qA.x) < 0.05 && std::abs(m.mean.y() - qA.y) < 0.05)
    {
      wA += w;
    }
    if (std::abs(m.mean.x() - qB.x) < 0.05 && std::abs(m.mean.y() - qB.y) < 0.05)
    {
      wB += w;
    }
  }
  EXPECT_GT(wA / wSum, 0.3);
  EXPECT_GT(wB / wSum, 0.1);
  EXPECT_GT(wA, wB);
}

// Inliers must be unique, also when the user callback rejects candidates.
TEST(TfestReference, se3_l2_robust_InliersAreUnique)
{
  rng().randomize(1234);
  std::vector<TPoint3D> a;
  std::vector<TPoint3D> b;
  mrpt::poses::CPose3D gt;
  makeSE3Points(20, 1.0, 0.0, a, b, gt);
  TMatchingPairList L;
  for (size_t i = 0; i < a.size(); i++)
  {
    L.emplace_back(
        static_cast<uint32_t>(i), static_cast<uint32_t>(i), static_cast<float>(a[i].x),
        static_cast<float>(a[i].y), static_cast<float>(a[i].z), static_cast<float>(b[i].x),
        static_cast<float>(b[i].y), static_cast<float>(b[i].z));
  }
  mrpt::tfest::TSE3RobustParams params;
  params.ransac_minSetSize = 5;
  params.ransac_maxSetSizePct = 0.5;
  params.ransac_nmaxSimulations = 20;
  params.ransac_threshold_lin = 0.05;
  params.user_individual_compat_callback = [](const mrpt::tfest::TPotentialMatch& pm)
  { return (pm.idx_this % 2) == 0; };
  mrpt::tfest::TSE3RobustResult res;
  ASSERT_TRUE(mrpt::tfest::se3_l2_robust(L, params, res));
  const std::set<uint32_t> uniq(res.inliers_idx.begin(), res.inliers_idx.end());
  EXPECT_EQ(uniq.size(), res.inliers_idx.size());
  EXPECT_EQ(res.inliers_idx.size(), 10u);
  for (const auto i : res.inliers_idx)
  {
    EXPECT_EQ(i % 2, 0u);
  }
}
