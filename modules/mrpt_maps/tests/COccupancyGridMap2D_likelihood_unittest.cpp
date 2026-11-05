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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/poses/CPose2D.h>

#include <cmath>
#include <limits>
#include <sstream>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;

namespace
{
// A map (resolution 0.1 m) with a finite vertical wall of occupied cells at
// x=5.0, y in [-2, 2], free everywhere else. The map extends well past the
// wall and past `buildMatchingScan()`'s maxRange in every direction, so that
// re-evaluating a scan from a perturbed pose never projects a point outside
// the grid bounds.
COccupancyGridMap2D buildGridWithWall()
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-15.0f, 16.0f, -15.0f, 15.0f, res);
  grid.fill(1.0f);  // all free

  const int cx = grid.x2idx(5.0f);
  const int cyMin = grid.y2idx(-2.0f);
  const int cyMax = grid.y2idx(2.0f);
  for (int cy = cyMin; cy <= cyMax; cy++) grid.setCell(cx, cy, 0.0f);

  return grid;
}

// A noise-free scan, simulated from `truePose`, that perfectly matches the
// occupied wall in `grid`.
CObservation2DRangeScan buildMatchingScan(const COccupancyGridMap2D& grid, const CPose2D& truePose)
{
  CObservation2DRangeScan scan;
  scan.aperture = mrpt::DEG2RAD(90.0f);
  scan.maxRange = 9.0f;
  scan.sensorPose = CPose3D();

  grid.laserScanSimulator(scan, truePose, 0.6f /*threshold*/, 91 /*N*/, 0.0f /*noiseStd*/);
  return scan;
}
}  // namespace

class LikelihoodMethodTest : public ::testing::TestWithParam<COccupancyGridMap2D::TLikelihoodMethod>
{
};

// For every likelihood method, the likelihood of a noise-free scan evaluated
// at the pose it was simulated from must be at least as large as when
// evaluated at a clearly wrong (perturbed) pose.
TEST_P(LikelihoodMethodTest, TruePoseAtLeastAsLikelyAsPerturbed)
{
  auto grid = buildGridWithWall();

  const CPose2D truePose(0.0, 0.0, 0.0);
  // Shifted along x, so the recorded ranges no longer match the distance to
  // the wall.
  const CPose2D perturbedPose(3.0, 0.0, 0.0);

  const auto scan = buildMatchingScan(grid, truePose);

  grid.likelihoodOptions.likelihoodMethod = GetParam();
  // lmConsensusOWA requires at least as many scan points as OWA_weights
  // entries (default: 100); keep it small and safely below the scan's ray
  // count. Harmless for the other methods, which never read OWA_weights.
  grid.likelihoodOptions.OWA_weights.assign(20, 1.0f / 20.0f);

  const double likTrue = grid.computeObservationLikelihood(scan, CPose3D(truePose));
  const double likPerturbed = grid.computeObservationLikelihood(scan, CPose3D(perturbedPose));

  EXPECT_GE(likTrue, likPerturbed);
}

INSTANTIATE_TEST_SUITE_P(
    AllMethods,
    LikelihoodMethodTest,
    ::testing::Values(
        COccupancyGridMap2D::lmMeanInformation,
        COccupancyGridMap2D::lmRayTracing,
        COccupancyGridMap2D::lmConsensus,
        COccupancyGridMap2D::lmCellsDifference,
        COccupancyGridMap2D::lmLikelihoodField_Thrun,
        COccupancyGridMap2D::lmLikelihoodField_II,
        COccupancyGridMap2D::lmConsensusOWA));

TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodFieldThrunMonotonicity)
{
  const auto grid = buildGridWithWall();

  CSimplePointsMap pmTrue;
  grid.getAsPointCloud(pmTrue, 0.5f);
  ASSERT_GT(pmTrue.size(), 0u);

  // A point cloud placed in free space, far from the wall, so none of its
  // points align with occupied cells.
  CSimplePointsMap pmPerturbed;
  for (size_t i = 0; i < pmTrue.size(); i++) pmPerturbed.insertPoint(-4.0f, -4.0f, 0.0f);

  const double likTrue = grid.computeLikelihoodField_Thrun(pmTrue, std::nullopt);
  const double likPerturbed = grid.computeLikelihoodField_Thrun(pmPerturbed, std::nullopt);

  EXPECT_GT(likTrue, likPerturbed);
}

TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodFieldIIMonotonicity)
{
  const auto grid = buildGridWithWall();

  CSimplePointsMap pm;
  grid.getAsPointCloud(pm, 0.5f);
  ASSERT_GT(pm.size(), 0u);

  const double likTrue = grid.computeLikelihoodField_II(pm, std::nullopt);
  const double likPerturbed = grid.computeLikelihoodField_II(pm, CPose2D(3.0, 0.0, 0.0));

  EXPECT_GT(likTrue, likPerturbed);
}

// The lmConsensusOWA branch keeps its own list of the top-M individual
// per-point likelihoods (see OWA_weights), so it needs at least as many
// points as weights; use a small weight vector to keep this test cheap.
TEST(COccupancyGridMap2DLikelihoodTests, ConsensusOWAMonotonicity)
{
  auto grid = buildGridWithWall();
  grid.likelihoodOptions.likelihoodMethod = COccupancyGridMap2D::lmConsensusOWA;
  grid.likelihoodOptions.OWA_weights.assign(5, 1.0f / 5.0f);

  const CPose2D truePose(0.0, 0.0, 0.0);
  const CPose2D perturbedPose(3.0, 0.0, 0.0);

  const auto scan = buildMatchingScan(grid, truePose);

  const double likTrue = grid.computeObservationLikelihood(scan, CPose3D(truePose));
  const double likPerturbed = grid.computeObservationLikelihood(scan, CPose3D(perturbedPose));

  EXPECT_TRUE(std::isfinite(likTrue));
  EXPECT_TRUE(std::isfinite(likPerturbed));
  EXPECT_GT(likTrue, likPerturbed);
}

TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodOptionsLoadFromConfigFileAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("lik", "LF_stdHit", 0.5f);
  cfg.write("lik", "LF_zHit", 0.8f);
  cfg.write("lik", "LF_decimation", 3);

  COccupancyGridMap2D::TLikelihoodOptions opts;
  opts.loadFromConfigFile(cfg, "lik");

  EXPECT_FLOAT_EQ(opts.LF_stdHit, 0.5f);
  EXPECT_FLOAT_EQ(opts.LF_zHit, 0.8f);
  EXPECT_EQ(opts.LF_decimation, 3u);

  std::ostringstream ss;
  opts.dumpToTextStream(ss);
  const std::string text = ss.str();

  EXPECT_NE(text.find("likelihoodMethod"), std::string::npos);
  EXPECT_NE(text.find("LF_stdHit"), std::string::npos);
  EXPECT_NE(text.find("OWA_weights"), std::string::npos);
}

TEST(COccupancyGridMap2DLikelihoodTests, CanComputeObservationLikelihoodTrueForPlanarScan)
{
  const auto grid = buildGridWithWall();

  const CPose2D truePose(0.0, 0.0, 0.0);
  const auto scan = buildMatchingScan(grid, truePose);

  EXPECT_TRUE(grid.canComputeObservationLikelihood(scan));
}

TEST(COccupancyGridMap2DLikelihoodTests, CanComputeObservationLikelihoodFalseForNonScanObservation)
{
  const auto grid = buildGridWithWall();

  CObservationRange obs;  // Not a CObservation2DRangeScan
  EXPECT_FALSE(grid.canComputeObservationLikelihood(obs));
}

TEST(COccupancyGridMap2DLikelihoodTests, CanComputeObservationLikelihoodFalseForAltitudeMismatch)
{
  auto grid = buildGridWithWall();
  grid.insertionOptions.useMapAltitude = true;
  grid.insertionOptions.mapAltitude = 5.0f;

  const CPose2D truePose(0.0, 0.0, 0.0);
  const auto scan = buildMatchingScan(grid, truePose);  // sensorPose.z() == 0

  EXPECT_FALSE(grid.canComputeObservationLikelihood(scan));
}

namespace
{
// A two-ray scan. Evaluated from a robot at yaw=+90 deg, ray #0 points to +x
// and ray #1 points to -x.
CObservation2DRangeScan buildTwoRayScan(float r0, float r1)
{
  CObservation2DRangeScan s;
  s.aperture = static_cast<float>(M_PI);
  s.rightToLeft = true;
  s.maxRange = 80.0f;
  s.resizeScan(2);
  s.setScanRange(0, r0);
  s.setScanRange(1, r1);
  s.setScanRangeValidity(0, true);
  s.setScanRangeValidity(1, true);
  return s;
}

const CPose3D robotLookingAtY(0, 0, 0, M_PI / 2, 0, 0);
}  // namespace

// Regression: out-of-map points must count as unknown cells (no out of bounds
// reads).
TEST(COccupancyGridMap2DLikelihoodTests, ConsensusPointOutOfMapIsUnknown)
{
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.1f);
  grid.fill(1.0f);  // all free
  grid.likelihoodOptions.likelihoodMethod = COccupancyGridMap2D::lmConsensus;
  grid.likelihoodOptions.consensus_pow = 1;

  // (2.5, 0) is out of the map, (-1, 0) lies on a free cell:
  const auto scan = buildTwoRayScan(2.5f, 1.0f);
  const double lik = grid.computeObservationLikelihood(scan, robotLookingAtY);

  // P(occupied) is 0.5 (unknown) and ~0 (free), averaged:
  const double pOccFree = 1.0 - grid.getCell(0, 0);
  EXPECT_NEAR(lik, std::log(0.5 * (0.5 + pOccFree)), 1e-3);
}

// Regression: ConsensusOWA threw for points out of the map along one axis.
TEST(COccupancyGridMap2DLikelihoodTests, ConsensusOWAPointOutOfMap)
{
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.1f);
  grid.fill(1.0f);
  grid.likelihoodOptions.likelihoodMethod = COccupancyGridMap2D::lmConsensusOWA;
  grid.likelihoodOptions.OWA_weights = {1.0f};

  const auto scan = buildTwoRayScan(2.5f, 1.0f);
  double lik = 0;
  EXPECT_NO_THROW(lik = grid.computeObservationLikelihood(scan, robotLookingAtY));
  // The best point is the out-of-map one, all its cells unknown:
  EXPECT_NEAR(lik, std::log(0.5), 1e-6);
}

// Regression: OWA must fuse the highest per-point likelihoods, not the lowest.
TEST(COccupancyGridMap2DLikelihoodTests, ConsensusOWAUsesHighestLikelihoods)
{
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.1f);
  grid.fill(0.99f);  // free
  // Occupied block around (1, 0):
  for (int cx = grid.x2idx(0.5f); cx <= grid.x2idx(1.5f); cx++)
  {
    for (int cy = grid.y2idx(-0.5f); cy <= grid.y2idx(0.5f); cy++)
    {
      grid.setCell(cx, cy, 0.01f);
    }
  }
  grid.likelihoodOptions.likelihoodMethod = COccupancyGridMap2D::lmConsensusOWA;
  grid.likelihoodOptions.OWA_weights = {1.0f};

  // (1, 0) on the occupied block, (-1, 0) on free space:
  const auto scan = buildTwoRayScan(1.0f, 1.0f);
  const double lik = grid.computeObservationLikelihood(scan, robotLookingAtY);
  const double pOccOfOccupied = 1.0 - grid.getCell(grid.x2idx(1.0f), grid.y2idx(0.0f));
  EXPECT_NEAR(lik, std::log(pOccOfOccupied), 1e-4);
}

// Regression: likelihood field II gave NaN for points out of the map.
TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodFieldIIPointOutOfMapFinite)
{
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.1f);
  grid.fill(1.0f);
  grid.likelihoodOptions.likelihoodMethod = COccupancyGridMap2D::lmLikelihoodField_II;
  grid.likelihoodOptions.LF_decimation = 1;

  const auto scan = buildTwoRayScan(2.5f, 1.0f);
  for (const bool alternate : {false, true})
  {
    grid.likelihoodOptions.LF_alternateAverageMethod = alternate;
    const double lik = grid.computeObservationLikelihood(scan, robotLookingAtY);
    EXPECT_TRUE(std::isfinite(lik)) << "alternate=" << alternate;
  }
}

// The per-point likelihood of LF-II is the window average: a point well inside
// a uniform region gives the same value as the analytic window average.
TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodFieldIIUniformRegion)
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, res);
  grid.fill(1.0f);
  const auto& lo = grid.likelihoodOptions;
  grid.likelihoodOptions.LF_decimation = 1;

  CSimplePointsMap pts;
  pts.insertPoint(grid.idx2x(20), grid.idx2y(20), 0);

  const float pFree = grid.getCell(20, 20);
  const int K = static_cast<int>(std::ceil(lo.LF_maxCorrsDistance / res));
  double expected = 0;
  for (int dx = -K; dx <= K; dx++)
  {
    for (int dy = -K; dy <= K; dy++)
    {
      const double d2 =
          mrpt::square(static_cast<double>(dx) * res) + mrpt::square(static_cast<double>(dy) * res);
      expected +=
          pFree / lo.LF_maxRange + (1 - pFree) * std::exp(-0.5 * d2 / mrpt::square(lo.LF_stdHit));
    }
  }
  expected /= mrpt::square(2 * K + 1);

  EXPECT_NEAR(grid.computeLikelihoodField_II(pts), std::log(expected), 1e-5);
}

// Regression: points on the last row/column of the grid were taken as out of
// the map by the Thrun likelihood field.
TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodFieldThrunLastColumn)
{
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.1f);
  grid.fill(0.99f);
  const int last = static_cast<int>(grid.getSizeX()) - 1;
  grid.setCell(last, 20, 0.01f);
  grid.setCell(last - 1, 25, 0.01f);
  grid.likelihoodOptions.LF_decimation = 1;

  CSimplePointsMap pLast;
  pLast.insertPoint(grid.idx2x(last), grid.idx2y(20), 0);
  CSimplePointsMap pInner;
  pInner.insertPoint(grid.idx2x(last - 1), grid.idx2y(25), 0);

  EXPECT_NEAR(
      grid.computeLikelihoodField_Thrun(pLast), grid.computeLikelihoodField_Thrun(pInner), 1e-9);
}

// The Thrun likelihood field uses exp(-0.5*d^2/sigma^2) of the distance to the
// closest occupied cell (with LF_useSquareDist=false).
TEST(COccupancyGridMap2DLikelihoodTests, LikelihoodFieldThrunGaussianOfDistance)
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, res);
  grid.fill(0.99f);
  grid.setCell(20, 20, 0.01f);
  const auto& lo = grid.likelihoodOptions;

  for (const int k : {0, 1, 2})
  {
    CSimplePointsMap pts;
    pts.insertPoint(grid.idx2x(20 + k), grid.idx2y(20), 0);
    const double d = static_cast<double>(k) * res;

    grid.likelihoodOptions.LF_useSquareDist = false;
    const double expected = lo.LF_zRandom / lo.LF_maxRange +
                            lo.LF_zHit * std::exp(-0.5 * d * d / mrpt::square(lo.LF_stdHit));
    EXPECT_NEAR(grid.computeLikelihoodField_Thrun(pts), std::log(expected), 1e-4) << "k=" << k;

    grid.likelihoodOptions.LF_useSquareDist = true;
    grid.likelihoodOptions.enableLikelihoodCache = false;
    const double expectedSq =
        lo.LF_zRandom / lo.LF_maxRange +
        lo.LF_zHit * std::exp(-0.5 * std::pow(d, 4) / mrpt::square(lo.LF_stdHit));
    EXPECT_NEAR(grid.computeLikelihoodField_Thrun(pts), std::log(expectedSq), 1e-4) << "k=" << k;
    grid.likelihoodOptions.enableLikelihoodCache = true;
  }
}
