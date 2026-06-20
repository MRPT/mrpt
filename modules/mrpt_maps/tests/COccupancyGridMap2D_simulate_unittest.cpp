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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;

namespace
{
// Builds a 10x2 m map (resolution 0.1 m) with a vertical wall of occupied
// cells at x=5.0, free everywhere else. The robot is meant to sit at the
// origin (0,0), looking towards +x, so the wall is exactly 5.0 m ahead.
COccupancyGridMap2D buildGridWithWallAhead()
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-1.0f, 9.0f, -1.0f, 1.0f, res);
  grid.fill(1.0f);  // all free

  const int cx = grid.x2idx(5.0f);
  for (unsigned int cy = 0; cy < grid.getSizeY(); cy++) grid.setCell(cx, cy, 0.0f);

  return grid;
}
}  // namespace

TEST(COccupancyGridMap2DSimulateTests, simulateScanRayHitsWall)
{
  const auto grid = buildGridWithWallAhead();

  float out_range;
  bool out_valid;
  grid.simulateScanRay(
      0.0, 0.0, 0.0 /*angle: +x direction*/, out_range, out_valid, 10.0 /*maxRange*/,
      0.4f /*threshold_free*/);

  EXPECT_TRUE(out_valid);
  EXPECT_NEAR(out_range, 5.0f, 0.1f);
}

TEST(COccupancyGridMap2DSimulateTests, laserScanSimulatorWallDistance)
{
  const auto grid = buildGridWithWallAhead();

  CObservation2DRangeScan scan;
  scan.aperture = 0.02f;  // very narrow FOV, all rays point ~straight ahead
  scan.maxRange = 10.0f;
  scan.sensorPose = CPose3D();  // sensor at the robot's origin, no rotation

  const CPose2D robotPose(0.0, 0.0, 0.0);  // looking towards +x
  grid.laserScanSimulator(scan, robotPose, 0.6f /*threshold*/, 3 /*N*/, 0.0f /*noiseStd*/);

  ASSERT_EQ(scan.getScanSize(), 3u);
  for (size_t i = 0; i < scan.getScanSize(); i++)
  {
    EXPECT_TRUE(scan.getScanRangeValidity(i));
    EXPECT_NEAR(scan.getScanRange(i), 5.0f, 0.1f);
  }
}

TEST(COccupancyGridMap2DSimulateTests, sonarSimulatorBasic)
{
  const auto grid = buildGridWithWallAhead();

  CObservationRange obs;
  obs.maxSensorDistance = 10.0f;
  obs.sensorConeAperture = mrpt::DEG2RAD(1.0f);  // narrow cone, ~straight ahead

  CObservationRange::TMeasurement m;
  m.sensorPose = mrpt::poses::CPose3D().asTPose();  // sensor at robot origin, no rotation
  obs.sensedData.push_back(m);

  const CPose2D robotPose(0.0, 0.0, 0.0);  // looking towards +x
  grid.sonarSimulator(obs, robotPose, 0.6f /*threshold*/);

  ASSERT_EQ(obs.sensedData.size(), 1u);
  EXPECT_NEAR(obs.sensedData.front().sensedDistance, 5.0f, 0.1f);
}
