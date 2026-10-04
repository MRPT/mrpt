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
#include <mrpt/random.h>

#include <algorithm>
#include <cmath>

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

TEST(COccupancyGridMap2DSimulateTests, simulateScanRayMissesOutOfGrid)
{
  const auto grid = buildGridWithWallAhead();

  float out_range;
  bool out_valid;
  // Ray pointing away from the wall (towards -x), so it leaves the grid
  // (whose x range starts at -1.0) well before reaching max range.
  grid.simulateScanRay(
      0.0, 0.0, M_PI /*angle: -x direction*/, out_range, out_valid, 10.0 /*maxRange*/,
      0.4f /*threshold_free*/);

  EXPECT_FALSE(out_valid);
  EXPECT_FLOAT_EQ(out_range, 10.0f);
}

TEST(COccupancyGridMap2DSimulateTests, simulateScanRayMissesShortMaxRange)
{
  auto grid = buildGridWithWallAhead();
  // Use a step size of exactly one cell so the resulting range at the
  // end of ray-tracing matches max_range_meters exactly.
  grid.insertionOptions.raytraceStepSizeInCellUnits = 1.0;

  float out_range;
  bool out_valid;
  // Ray pointing straight at the wall (5.0 m away), but with a max range
  // shorter than that distance, so the ray never reaches the wall.
  grid.simulateScanRay(
      0.0, 0.0, 0.0 /*angle: +x direction*/, out_range, out_valid, 2.0 /*maxRange*/,
      0.4f /*threshold_free*/);

  EXPECT_FALSE(out_valid);
  EXPECT_NEAR(out_range, 2.0f, 1e-3f);
}

TEST(COccupancyGridMap2DSimulateTests, simulateScanRayWithNoise)
{
  const auto grid = buildGridWithWallAhead();

  mrpt::random::getRandomGenerator().randomize(1234 /*fixed seed*/);

  float out_range;
  bool out_valid;
  grid.simulateScanRay(
      0.0, 0.0, 0.0 /*angle: +x direction*/, out_range, out_valid, 10.0 /*maxRange*/,
      0.4f /*threshold_free*/, 0.05 /*noiseStd*/, mrpt::DEG2RAD(0.5) /*angleNoiseStd*/);

  EXPECT_TRUE(out_valid);
  // The noise is modest, so the returned range must stay close to the true
  // 5.0 m distance to the wall.
  EXPECT_NEAR(out_range, 5.0f, 1.0f);
}

TEST(COccupancyGridMap2DSimulateTests, laserScanSimulatorDecimation)
{
  const auto grid = buildGridWithWallAhead();

  CObservation2DRangeScan scan;
  scan.aperture = 0.02f;  // very narrow FOV, all rays point ~straight ahead
  scan.maxRange = 10.0f;
  scan.sensorPose = CPose3D();

  const CPose2D robotPose(0.0, 0.0, 0.0);
  const unsigned int decimation = 2;
  grid.laserScanSimulator(
      scan, robotPose, 0.6f /*threshold*/, 5 /*N*/, 0.0f /*noiseStd*/, decimation);

  ASSERT_EQ(scan.getScanSize(), 5u);

  // Indices 0, 2, 4 were actually simulated (0 and multiples of decimation).
  for (size_t i = 0; i < scan.getScanSize(); i += decimation)
  {
    EXPECT_TRUE(scan.getScanRangeValidity(i));
    EXPECT_NEAR(scan.getScanRange(i), 5.0f, 0.1f);
  }

  // Indices 1 and 3 were never touched, so they keep resizeScan()'s default
  // (zero-initialized) values.
  EXPECT_FALSE(scan.getScanRangeValidity(1));
  EXPECT_FLOAT_EQ(scan.getScanRange(1), 0.0f);
  EXPECT_FALSE(scan.getScanRangeValidity(3));
  EXPECT_FLOAT_EQ(scan.getScanRange(3), 0.0f);
}

TEST(COccupancyGridMap2DSimulateTests, laserScanSimulatorWithUncertaintyUnscented)
{
  const auto grid = buildGridWithWallAhead();

  COccupancyGridMap2D::TLaserSimulUncertaintyParams params;
  params.method = COccupancyGridMap2D::sumUnscented;
  params.nRays = 7;
  params.aperture = mrpt::DEG2RAD(2.0f);  // narrow, all rays point ~straight ahead
  params.robotPose.mean = CPose2D(0.0, 0.0, 0.0);
  params.robotPose.cov.setIdentity();
  params.robotPose.cov *= 0.0001;
  params.sensorPose = CPose3D();
  params.maxRange = 10.0f;
  params.threshold = 0.6f;

  COccupancyGridMap2D::TLaserSimulUncertaintyResult results;
  grid.laserScanSimulatorWithUncertainty(params, results);

  ASSERT_EQ(results.scanWithUncert.rangeScan.getScanSize(), params.nRays);
  ASSERT_EQ(static_cast<size_t>(results.scanWithUncert.rangesMean.size()), params.nRays);

  const double minCovarFloor = 0.5 * grid.getResolution() * grid.getResolution();
  for (size_t i = 0; i < params.nRays; i++)
  {
    EXPECT_NEAR(results.scanWithUncert.rangesMean[i], 5.0, 0.5);
    EXPECT_GE(results.scanWithUncert.rangesCovar(i, i), minCovarFloor - 1e-9);
  }
}

TEST(COccupancyGridMap2DSimulateTests, laserScanSimulatorWithUncertaintyMonteCarlo)
{
  const auto grid = buildGridWithWallAhead();

  COccupancyGridMap2D::TLaserSimulUncertaintyParams params;
  params.method = COccupancyGridMap2D::sumMonteCarlo;
  params.MC_samples = 30;
  params.nRays = 5;
  params.aperture = mrpt::DEG2RAD(2.0f);
  params.robotPose.mean = CPose2D(0.0, 0.0, 0.0);
  params.robotPose.cov.setIdentity();
  params.robotPose.cov *= 0.0001;
  params.sensorPose = CPose3D();
  params.maxRange = 10.0f;
  params.threshold = 0.6f;

  COccupancyGridMap2D::TLaserSimulUncertaintyResult results;
  grid.laserScanSimulatorWithUncertainty(params, results);

  ASSERT_EQ(results.scanWithUncert.rangeScan.getScanSize(), params.nRays);

  const double minCovarFloor = 0.5 * grid.getResolution() * grid.getResolution();
  for (size_t i = 0; i < params.nRays; i++)
  {
    EXPECT_NEAR(results.scanWithUncert.rangesMean[i], 5.0, 0.5);
    EXPECT_GE(results.scanWithUncert.rangesCovar(i, i), minCovarFloor - 1e-9);
  }
}

TEST(COccupancyGridMap2DSimulateTests, laserScanSimulatorWithUncertaintyUnknownMethodThrows)
{
  const auto grid = buildGridWithWallAhead();

  COccupancyGridMap2D::TLaserSimulUncertaintyParams params;
  params.method = static_cast<COccupancyGridMap2D::TLaserSimulUncertaintyMethod>(99);
  params.nRays = 5;

  COccupancyGridMap2D::TLaserSimulUncertaintyResult results;
  EXPECT_THROW(grid.laserScanSimulatorWithUncertainty(params, results), std::runtime_error);
}

namespace
{
// Distance from (ox,oy) along direction (dx,dy) (unit vector) at which the
// ray enters the axis-aligned box [x0,x1]x[y0,y1] (slab method), or -1.
double rayBoxEntry(
    double ox, double oy, double dx, double dy, double x0, double x1, double y0, double y1)
{
  double tmin = 0;
  double tmax = 1e9;
  for (int axis = 0; axis < 2; axis++)
  {
    const double o = axis == 0 ? ox : oy;
    const double d = axis == 0 ? dx : dy;
    const double lo = axis == 0 ? x0 : y0;
    const double hi = axis == 0 ? x1 : y1;
    if (std::abs(d) < 1e-12)
    {
      if (o < lo || o > hi) return -1;
      continue;
    }
    double t1 = (lo - o) / d;
    double t2 = (hi - o) / d;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
  }
  return tmin <= tmax ? tmin : -1;
}
}  // namespace

// The range must be the distance at which the ray enters the first occupied
// cell (its closest point), also for oblique rays.
TEST(COccupancyGridMap2DSimulateTests, simulateScanRayExactWallEntry)
{
  const auto grid = buildGridWithWallAhead();
  // Near boundary of the wall cells:
  const double wallX = grid.idx2x(grid.x2idx(5.0f)) - 0.5 * grid.getResolution();
  for (const double angDeg : {0.0, 5.0, 8.0, -7.0})
  {
    const double ang = angDeg * M_PI / 180.0;
    bool out_valid = false;
    float out_range = 0;
    grid.simulateScanRay(0.0, 0.0, ang, out_range, out_valid, 10.0, 0.5f, 0, 0);
    ASSERT_TRUE(out_valid) << "angle=" << angDeg;
    EXPECT_NEAR(out_range, wallX / std::cos(ang), 1e-3) << "angle=" << angDeg;
  }
}

// A ray clipping just the corner of an occupied cell must detect it (a
// fixed-step ray marching may jump over it).
TEST(COccupancyGridMap2DSimulateTests, simulateScanRayHitsCellCorner)
{
  const float res = 0.1f;
  COccupancyGridMap2D grid(-1.0f, 9.0f, -1.0f, 1.0f, res);
  grid.fill(1.0f);  // all free
  const int cx = grid.x2idx(3.05f);
  const int cy = grid.y2idx(0.15f);
  grid.setCell(cx, cy, 0.0f);
  const double x0 = grid.idx2x(cx) - 0.5 * res;
  const double y0 = grid.idx2y(cy) - 0.5 * res;

  // Aim just inside the lower-right corner of the cell:
  const double tx = x0 + res - 0.004;
  const double ty = y0 + 0.004;
  const double ang = std::atan2(ty, tx);

  bool out_valid = false;
  float out_range = 0;
  grid.simulateScanRay(0.0, 0.0, ang, out_range, out_valid, 10.0, 0.5f, 0, 0);
  ASSERT_TRUE(out_valid);
  const double expected =
      rayBoxEntry(0, 0, std::cos(ang), std::sin(ang), x0, x0 + res, y0, y0 + res);
  ASSERT_GT(expected, 0);
  EXPECT_NEAR(out_range, expected, 1e-3);
}
