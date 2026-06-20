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
#include <mrpt/poses/CPose2D.h>

using namespace mrpt::maps;
using namespace mrpt::poses;

namespace
{
// A 4x4 m map (resolution 0.1 m), all free except for a single occupied cell
// at (1.0, 1.0).
COccupancyGridMap2D buildGridWithOneObstacle()
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, res);
  grid.fill(1.0f);  // all free
  grid.setCell(grid.x2idx(1.0f), grid.y2idx(1.0f), 0.0f);
  return grid;
}
}  // namespace

TEST(COccupancyGridMap2DMatchingTests, DetermineMatching2DExactCorrespondence)
{
  const auto grid = buildGridWithOneObstacle();

  // Use the exact center of the occupied cell so the correspondence has zero
  // residual distance.
  const float obstacleX = grid.idx2x(grid.x2idx(1.0f));
  const float obstacleY = grid.idx2y(grid.y2idx(1.0f));

  CSimplePointsMap pm;
  pm.insertPoint(obstacleX, obstacleY, 0.0f);

  mrpt::tfest::TMatchingPairList correspondences;
  TMatchingParams params;
  params.maxDistForCorrespondence = 0.2f;
  TMatchingExtraResults extraResults;

  grid.determineMatching2D(&pm, CPose2D(0.0, 0.0, 0.0), correspondences, params, extraResults);

  EXPECT_EQ(correspondences.size(), 1u);
  EXPECT_FLOAT_EQ(extraResults.correspondencesRatio, 1.0f);
  EXPECT_NEAR(extraResults.sumSqrDist, 0.0f, 1e-3f);
}

TEST(COccupancyGridMap2DMatchingTests, DetermineMatching2DNoCorrespondenceWhenFarAway)
{
  const auto grid = buildGridWithOneObstacle();

  CSimplePointsMap pm;
  pm.insertPoint(1.0f, 1.0f, 0.0f);

  mrpt::tfest::TMatchingPairList correspondences;
  TMatchingParams params;
  params.maxDistForCorrespondence = 0.2f;
  TMatchingExtraResults extraResults;

  // Move the comparison point far away from the only obstacle in the grid.
  grid.determineMatching2D(&pm, CPose2D(10.0, 10.0, 0.0), correspondences, params, extraResults);

  EXPECT_EQ(correspondences.size(), 0u);
  EXPECT_FLOAT_EQ(extraResults.correspondencesRatio, 0.0f);
}
