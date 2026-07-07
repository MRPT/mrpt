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
#include <mrpt/core/cpu.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/tfest/se2.h>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;

namespace
{
// Builds a list of correspondences consistent with the given SE(2)
// transform `q`, i.e. global = q (+) local, for a small fixed set of
// local points.
TMatchingPairList buildConsistentList(const TPose2D& q, size_t nExtra = 0)
{
  TMatchingPairList list;
  const std::vector<TPoint2D> localPts = {
      { 0.0,  0.0},
      { 1.0,  0.0},
      { 0.0,  1.0},
      { 2.0,  1.5},
      {-1.0,  0.5},
      { 1.5, -1.0}
  };

  uint32_t idx = 0;
  for (const auto& lp : localPts)
  {
    const TPoint2D gp = q.composePoint(lp);
    TMatchingPair p;
    p.globalIdx = idx;
    p.localIdx = idx;
    p.global.x = static_cast<float>(gp.x);
    p.global.y = static_cast<float>(gp.y);
    p.global.z = 0;
    p.local.x = static_cast<float>(lp.x);
    p.local.y = static_cast<float>(lp.y);
    p.local.z = 0;
    list.push_back(p);
    idx++;
  }
  for (size_t i = 0; i < nExtra; i++)
  {
    // Deliberately inconsistent / outlier correspondences:
    TMatchingPair p;
    p.globalIdx = idx;
    p.localIdx = idx;
    p.global.x = static_cast<float>(10.0 + static_cast<double>(i));
    p.global.y = static_cast<float>(-10.0 - static_cast<double>(i));
    p.global.z = 0;
    p.local.x = static_cast<float>(-i);
    p.local.y = static_cast<float>(i * 2);
    p.local.z = 0;
    list.push_back(p);
    idx++;
  }
  return list;
}
}  // namespace

// ------------------------------------------------------
// se2_l2()
// ------------------------------------------------------

TEST(tfest_se2, se2_l2_LessThanTwoPointsFails)
{
  TMatchingPairList empty;
  TPose2D out;
  EXPECT_FALSE(tfest::se2_l2(empty, out));

  TMatchingPairList oneElem;
  oneElem.push_back(TMatchingPair(0, 0, 1, 2, 0, 1, 2, 0));
  EXPECT_FALSE(tfest::se2_l2(oneElem, out));
}

TEST(tfest_se2, se2_l2_RecoversKnownTransform)
{
  const TPose2D q(1.0, 2.0, 0.4);
  const auto list = buildConsistentList(q);

  TPose2D out;
  ASSERT_TRUE(tfest::se2_l2(list, out));
  EXPECT_NEAR(out.x, q.x, 1e-3);
  EXPECT_NEAR(out.y, q.y, 1e-3);
  EXPECT_NEAR(out.phi, q.phi, 1e-3);
}

TEST(tfest_se2, se2_l2_Covariance)
{
  const TPose2D q(0.5, -0.3, 0.1);
  const auto list = buildConsistentList(q);

  TPose2D out;
  CMatrixDouble33 cov;
  ASSERT_TRUE(tfest::se2_l2(list, out, &cov));
  // Diagonal must be finite:
  EXPECT_TRUE(std::isfinite(cov(0, 0)));
  EXPECT_TRUE(std::isfinite(cov(1, 1)));
  EXPECT_TRUE(std::isfinite(cov(2, 2)));
  // Off-diagonal symmetry:
  EXPECT_NEAR(cov(0, 1), cov(1, 0), 1e-9);
  EXPECT_NEAR(cov(0, 2), cov(2, 0), 1e-9);
  EXPECT_NEAR(cov(1, 2), cov(2, 1), 1e-9);
}

TEST(tfest_se2, se2_l2_GaussianWrapper)
{
  const TPose2D q(0.2, 0.7, -0.15);
  const auto list = buildConsistentList(q);

  CPosePDFGaussian out;
  ASSERT_TRUE(tfest::se2_l2(list, out));
  EXPECT_NEAR(out.mean.x(), q.x, 1e-3);
  EXPECT_NEAR(out.mean.y(), q.y, 1e-3);
  EXPECT_NEAR(out.mean.phi(), q.phi, 1e-3);
}

TEST(tfest_se2, se2_l2_NonSSE2Fallback)
{
  // Force the non-vectorized code path, regardless of the actual CPU
  // capabilities, to also cover the scalar reference implementation:
  const bool hadSSE2 = mrpt::cpu::supports(mrpt::cpu::feature::SSE2);
  mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, false);

  const TPose2D q(0.6, -0.1, 0.33);
  const auto list = buildConsistentList(q);

  TPose2D out;
  const bool ok = tfest::se2_l2(list, out);

  mrpt::cpu::overrideDetectedFeature(mrpt::cpu::feature::SSE2, hadSSE2);

  ASSERT_TRUE(ok);
  EXPECT_NEAR(out.x, q.x, 1e-3);
  EXPECT_NEAR(out.y, q.y, 1e-3);
  EXPECT_NEAR(out.phi, q.phi, 1e-3);
}

TEST(tfest_se2, se2_l2_DegeneratePhiZero)
{
  // All correspondences at the same point => Ax=Ay=0 => phi defaults to 0
  TMatchingPairList list;
  for (int i = 0; i < 3; i++)
  {
    TMatchingPair p(i, i, 0, 0, 0, 0, 0, 0);
    list.push_back(p);
  }
  TPose2D out;
  ASSERT_TRUE(tfest::se2_l2(list, out));
  EXPECT_NEAR(out.phi, 0.0, 1e-9);
}

// ------------------------------------------------------
// se2_l2_robust()
// ------------------------------------------------------

TEST(tfest_se2_robust, BasicConsensusNoOutliers)
{
  getRandomGenerator().randomize(1234);

  const TPose2D q(0.3, -0.2, 0.25);
  const auto list = buildConsistentList(q);

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = static_cast<unsigned int>(list.size());
  params.ransac_nSimulations = 20;

  TSE2RobustResult result;
  ASSERT_TRUE(tfest::se2_l2_robust(list, 0.02, params, result));
  EXPECT_GE(result.largestSubSet.size(), 3u);
  ASSERT_GT(result.transformation.size(), 0u);
}

TEST(tfest_se2_robust, WithOutliersDynamicIterations)
{
  getRandomGenerator().randomize(42);

  const TPose2D q(1.0, 0.5, 0.35);
  const auto list = buildConsistentList(q, /*nExtra=*/2);

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = static_cast<unsigned int>(list.size());
  params.ransac_nSimulations = 0;  // dynamic # of iterations
  params.verbose = true;

  TSE2RobustResult result;
  ASSERT_TRUE(tfest::se2_l2_robust(list, 0.02, params, result));
  EXPECT_GE(result.largestSubSet.size(), 3u);
}

TEST(tfest_se2_robust, PointsAlgorithmInsteadOfLandmarks)
{
  getRandomGenerator().randomize(7);

  const TPose2D q(0.1, 0.1, 0.05);
  const auto list = buildConsistentList(q, /*nExtra=*/1);

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = static_cast<unsigned int>(list.size());
  params.ransac_nSimulations = 30;
  params.ransac_algorithmForLandmarks = false;

  TSE2RobustResult result;
  ASSERT_TRUE(tfest::se2_l2_robust(list, 0.02, params, result));
  EXPECT_GE(result.largestSubSet.size(), 3u);
}

TEST(tfest_se2_robust, FuseByApproxDistanceInsteadOfExactMatch)
{
  getRandomGenerator().randomize(99);

  const TPose2D q(0.4, -0.4, 0.2);
  const auto list = buildConsistentList(q, /*nExtra=*/1);

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = static_cast<unsigned int>(list.size());
  params.ransac_nSimulations = 60;  // enough repeats to exercise fuse-by-distance matching
  params.ransac_fuseByCorrsMatch = false;
  // Prevent the early "good enough" break, so several consensus sets get a
  // chance to be found and fused together (covers the "found match" paths):
  params.max_rmse_to_end = 1e-20;

  TSE2RobustResult result;
  ASSERT_TRUE(tfest::se2_l2_robust(list, 0.02, params, result));
  EXPECT_GE(result.largestSubSet.size(), 3u);
  EXPECT_GT(result.transformation.size(), 0u);
}

TEST(tfest_se2_robust, FuseByExactCorrsMatchRepeatedModes)
{
  getRandomGenerator().randomize(123);

  const TPose2D q(-0.2, 0.6, -0.15);
  const auto list = buildConsistentList(q);

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = static_cast<unsigned int>(list.size());
  params.ransac_nSimulations = 100;
  params.ransac_fuseByCorrsMatch = true;  // default, exercised here explicitly
  // Prevent the early "good enough" break, so the very same consensus subset
  // gets picked more than once, exercising the "already added mode" path:
  params.max_rmse_to_end = 1e-20;

  TSE2RobustResult result;
  ASSERT_TRUE(tfest::se2_l2_robust(list, 0.02, params, result));
  EXPECT_GE(result.largestSubSet.size(), 3u);
  EXPECT_GT(result.transformation.size(), 0u);
}

TEST(tfest_se2_robust, UserIndividualCompatibilityCallback)
{
  getRandomGenerator().randomize(55);

  const TPose2D q(0.2, 0.3, 0.1);
  const auto list = buildConsistentList(q);

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;
  params.ransac_maxSetSize = static_cast<unsigned int>(list.size());
  params.ransac_nSimulations = 20;
  // Reject correspondence idx 0, forcing the RANSAC loop to skip it:
  params.user_individual_compat_callback = [](const TPotentialMatch& pm) -> bool
  { return pm.idx_this != 0; };

  TSE2RobustResult result;
  ASSERT_TRUE(tfest::se2_l2_robust(list, 0.02, params, result));
  for (const auto& c : result.largestSubSet)
  {
    EXPECT_NE(c.globalIdx, 0u);
  }
}

TEST(tfest_se2_robust, NotEnoughCorrespondencesFails)
{
  TMatchingPairList list;
  list.push_back(TMatchingPair(0, 0, 0, 0, 0, 0, 0, 0));
  list.push_back(TMatchingPair(1, 1, 1, 1, 0, 1, 1, 0));

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;  // more than available correspondences

  TSE2RobustResult result;
  EXPECT_FALSE(tfest::se2_l2_robust(list, 0.02, params, result));
  EXPECT_EQ(result.largestSubSet.size(), 0u);
}

TEST(tfest_se2_robust, NotEnoughUniqueMatchesFails)
{
  TMatchingPairList list;
  // All correspondences refer to the very same (globalIdx, localIdx) pair,
  // so the number of *different* matches is just 1.
  for (int i = 0; i < 5; i++)
  {
    TMatchingPair p(0, 0, static_cast<float>(i), 0, 0, static_cast<float>(i), 0, 0);
    list.push_back(p);
  }

  TSE2RobustParams params;
  params.ransac_minSetSize = 3;

  TSE2RobustResult result;
  EXPECT_FALSE(tfest::se2_l2_robust(list, 0.02, params, result));
}
