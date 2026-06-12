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
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;

namespace
{
// A map (resolution 0.1 m) with a finite vertical wall of occupied cells at
// x=5.0, y in [-2, 2], free everywhere else. The map extends past the wall so
// the wall cells are not on the outermost border row.
COccupancyGridMap2D buildGridWithWall()
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-5.0f, 6.0f, -5.0f, 5.0f, res);
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
