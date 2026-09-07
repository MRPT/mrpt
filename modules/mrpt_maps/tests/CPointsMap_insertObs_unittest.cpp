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

// Covers the observation types accepted by CPointsMap::insertObservation() that
// the main CPointsMap test file does not reach, plus the remaining
// point-accessor and nearest-neighbor overloads.

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <sstream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;

namespace
{
CSimplePointsMap makeGrid(int n = 5, float step = 1.0f)
{
  CSimplePointsMap m;
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) m.insertPoint(i * step, j * step, 0.0f);
  return m;
}

CObservation2DRangeScan::Ptr makeScan2D()
{
  auto scan = CObservation2DRangeScan::Create();
  scan->timestamp = mrpt::Clock::now();
  scan->aperture = M_PIf;
  scan->rightToLeft = true;
  scan->maxRange = 20.0f;
  scan->resizeScan(60);
  for (size_t i = 0; i < 60; i++)
  {
    scan->setScanRange(i, 5.0f);
    scan->setScanRangeValidity(i, true);
  }
  return scan;
}
}  // namespace

TEST(CPointsMapInsertObs, insertObservationRange)
{
  // A sonar-like observation: each cone is inserted as a single point at the
  // measured distance along the sensor's X axis.
  auto obs = CObservationRange::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->minSensorDistance = 0.1f;
  obs->maxSensorDistance = 10.0f;
  obs->sensorConeAperture = 0.2f;

  CObservationRange::TMeasurement m;
  m.sensorID = 0;
  m.sensorPose = mrpt::math::TPose3D(0, 0, 0, 0, 0, 0);
  m.sensedDistance = 3.0f;
  obs->sensedData.push_back(m);

  m.sensorPose = mrpt::math::TPose3D(0, 0, 0, M_PI / 2, 0, 0);
  m.sensedDistance = 4.0f;
  obs->sensedData.push_back(m);

  CSimplePointsMap map;
  EXPECT_TRUE(map.insertObservation(*obs));
  // Each cone is discretized into a patch of points at the sensed distance:
  ASSERT_GT(map.size(), 2U);

  size_t nAt3m = 0, nAt4m = 0;
  for (size_t i = 0; i < map.size(); i++)
  {
    mrpt::math::TPoint3D p;
    map.getPoint(i, p);
    const double r = p.norm();
    if (std::abs(r - 3.0) < 1e-4) nAt3m++;
    if (std::abs(r - 4.0) < 1e-4) nAt4m++;
  }
  EXPECT_GT(nAt3m, 0U);
  EXPECT_GT(nAt4m, 0U);
  EXPECT_EQ(nAt3m + nAt4m, map.size());

  // Out-of-range readings are dropped:
  CSimplePointsMap map2;
  obs->sensedData[0].sensedDistance = 100.0f;
  obs->sensedData[1].sensedDistance = 0.01f;
  EXPECT_TRUE(map2.insertObservation(*obs));
  EXPECT_EQ(map2.size(), 0U);
}

TEST(CPointsMapInsertObs, insertObservationPointCloud)
{
  auto obs = CObservationPointCloud::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->sensorPose = mrpt::poses::CPose3D(10, 0, 0, 0, 0, 0);
  obs->pointcloud = CSimplePointsMap::Create();
  obs->pointcloud->insertPoint(1.0f, 2.0f, 3.0f);
  obs->pointcloud->insertPoint(-1.0f, 0.0f, 0.0f);

  CSimplePointsMap map;
  EXPECT_TRUE(map.insertObservation(*obs));
  ASSERT_EQ(map.size(), 2U);

  // Points come back in the map frame, i.e. shifted by the sensor pose:
  mrpt::math::TPoint3D p;
  map.getPoint(0, p);
  EXPECT_NEAR(p.x, 11.0, 1e-4);

  // An observation without a point cloud is rejected:
  auto emptyObs = CObservationPointCloud::Create();
  emptyObs->timestamp = mrpt::Clock::now();
  CSimplePointsMap map2;
  EXPECT_THROW(map2.insertObservation(*emptyObs), std::exception);
}

TEST(CPointsMapInsertObs, insertObservationRotatingScan)
{
  auto obs = CObservationRotatingScan::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->rowCount = 2;
  obs->columnCount = 8;
  obs->rangeResolution = 0.01;
  obs->sensorPose = mrpt::poses::CPose3D(0, 0, 0.5, 0, 0, 0);
  obs->rangeImage.setZero(obs->rowCount, obs->columnCount);
  obs->intensityImage.setZero(obs->rowCount, obs->columnCount);
  obs->azimuthSpan = 2 * M_PI;
  obs->minRange = 0.1;
  obs->maxRange = 50.0;
  for (size_t r = 0; r < obs->rowCount; r++)
    for (size_t c = 0; c < obs->columnCount; c++) obs->rangeImage(r, c) = 500;  // 5 m

  CSimplePointsMap map;
  EXPECT_TRUE(map.insertObservation(*obs));
  EXPECT_GT(map.size(), 0U);

  // The likelihood of the same observation against the map it built must be
  // finite (and better than against an empty map):
  const double lik = map.computeObservationLikelihood(*obs, mrpt::poses::CPose3D());
  EXPECT_TRUE(std::isfinite(lik));

  CSimplePointsMap emptyMap;
  EXPECT_NEAR(emptyMap.computeObservationLikelihood(*obs, mrpt::poses::CPose3D()), -100.0, 1e-9);
}

TEST(CPointsMapInsertObs, insertObservationVelodyneScan)
{
  // A minimal, already-decoded Velodyne observation:
  auto obs = CObservationVelodyneScan::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->sensorPose = mrpt::poses::CPose3D(0, 0, 1.0, 0, 0, 0);
  for (int i = 0; i < 10; i++)
  {
    obs->point_cloud.x.push_back(static_cast<float>(i));
    obs->point_cloud.y.push_back(0.0f);
    obs->point_cloud.z.push_back(0.0f);
    obs->point_cloud.intensity.push_back(100);
    obs->point_cloud.laser_id.push_back(0);
    obs->point_cloud.azimuth.push_back(0.0f);
  }

  CSimplePointsMap map;
  EXPECT_TRUE(map.insertObservation(*obs));
  ASSERT_EQ(map.size(), 10U);

  mrpt::math::TPoint3D p;
  map.getPoint(3, p);
  EXPECT_NEAR(p.z, 1.0, 1e-4);

  const double lik = map.computeObservationLikelihood(*obs, mrpt::poses::CPose3D());
  EXPECT_TRUE(std::isfinite(lik));
}

TEST(CPointsMapInsertObs, insertObservationPointCloudLikelihood)
{
  CSimplePointsMap map = makeGrid();

  auto obs = CObservationPointCloud::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->pointcloud = CSimplePointsMap::Create();
  obs->pointcloud->insertPoint(1.0f, 1.0f, 0.0f);
  obs->pointcloud->insertPoint(2.0f, 2.0f, 0.0f);

  const double lik = map.computeObservationLikelihood(*obs, mrpt::poses::CPose3D());
  EXPECT_TRUE(std::isfinite(lik));

  // A point cloud far away must be less likely:
  auto farObs = CObservationPointCloud::Create();
  farObs->timestamp = mrpt::Clock::now();
  farObs->pointcloud = CSimplePointsMap::Create();
  farObs->pointcloud->insertPoint(100.0f, 100.0f, 0.0f);
  EXPECT_LT(map.computeObservationLikelihood(*farObs, mrpt::poses::CPose3D()), lik);
}

TEST(CPointsMapInsertObs, insertPlanarMapRejectsNonHorizontalScans)
{
  auto scan = makeScan2D();
  // A sensor looking downwards is not a planar (horizontal) scan:
  scan->setSensorPose(mrpt::poses::CPose3D(0, 0, 1.0, 0, M_PI / 4, 0));

  CSimplePointsMap map;
  map.insertionOptions.isPlanarMap = true;
  map.insertionOptions.horizontalTolerance = mrpt::DEG2RAD(1.0);
  EXPECT_FALSE(map.insertObservation(*scan));
  EXPECT_EQ(map.size(), 0U);

  // A horizontal one is accepted:
  scan->setSensorPose(mrpt::poses::CPose3D(0, 0, 1.0, 0, 0, 0));
  EXPECT_TRUE(map.insertObservation(*scan));
  EXPECT_GT(map.size(), 0U);

  // 3D range scans are never inserted into a planar map:
  auto obs3D = CObservation3DRangeScan::Create();
  obs3D->timestamp = mrpt::Clock::now();
  obs3D->hasPoints3D = true;
  obs3D->resizePoints3DVectors(1);
  obs3D->points3D_x[0] = 1.0f;
  obs3D->points3D_y[0] = 0.0f;
  obs3D->points3D_z[0] = 0.0f;
  const size_t before = map.size();
  EXPECT_FALSE(map.insertObservation(*obs3D));
  EXPECT_EQ(map.size(), before);
}

TEST(CPointsMapInsertObs, insertWithFuseWithExisting)
{
  auto scan = makeScan2D();

  CSimplePointsMap map;
  map.insertObservation(*scan);
  const size_t nAfterFirst = map.size();
  ASSERT_GT(nAfterFirst, 0U);

  // Re-inserting the same scan with fusion enabled must not double the cloud:
  map.insertionOptions.fuseWithExisting = true;
  map.insertionOptions.minDistBetweenLaserPoints = 0.05f;
  EXPECT_TRUE(map.insertObservation(*scan));
  EXPECT_LE(map.size(), 2 * nAfterFirst);

  // The same for a 3D range scan:
  auto obs3D = CObservation3DRangeScan::Create();
  obs3D->timestamp = mrpt::Clock::now();
  obs3D->hasPoints3D = true;
  obs3D->resizePoints3DVectors(3);
  for (int i = 0; i < 3; i++)
  {
    obs3D->points3D_x[i] = static_cast<float>(i);
    obs3D->points3D_y[i] = 0.0f;
    obs3D->points3D_z[i] = 0.0f;
  }
  CSimplePointsMap map3d;
  map3d.insertObservation(*obs3D);
  map3d.insertionOptions.fuseWithExisting = true;
  EXPECT_TRUE(map3d.insertObservation(*obs3D));
  EXPECT_LE(map3d.size(), 6U);
}

TEST(CPointsMapInsertObs, getAllPointsWithDecimation)
{
  CSimplePointsMap map;
  for (int i = 0; i < 20; i++) map.insertPoint(i, 2 * i, 3 * i);

  std::vector<float> xs, ys;
  map.getAllPoints(xs, ys);
  EXPECT_EQ(xs.size(), 20U);
  EXPECT_EQ(ys.size(), 20U);
  EXPECT_NEAR(ys[3], 6.0f, 1e-5f);

  map.getAllPoints(xs, ys, 4);
  EXPECT_EQ(xs.size(), 5U);
  EXPECT_NEAR(xs[1], 4.0f, 1e-5f);

  std::vector<mrpt::math::TPoint2D> pts;
  map.getAllPoints(pts, 5);
  EXPECT_EQ(pts.size(), 4U);

  std::vector<float> zs;
  map.getAllPoints(xs, ys, zs, 2);
  EXPECT_EQ(zs.size(), 10U);
  EXPECT_NEAR(zs[1], 6.0f, 1e-5f);

  EXPECT_THROW(map.getAllPoints(xs, ys, 0), std::exception);
}

TEST(CPointsMapInsertObs, setAllPoints)
{
  CSimplePointsMap map;

  const std::vector<float> X{1.0f, 2.0f, 3.0f};
  const std::vector<float> Y{4.0f, 5.0f, 6.0f};
  const std::vector<float> Z{7.0f, 8.0f, 9.0f};

  map.setAllPoints(X, Y, Z);
  ASSERT_EQ(map.size(), 3U);
  mrpt::math::TPoint3D p;
  map.getPoint(2, p);
  EXPECT_NEAR(p.z, 9.0, 1e-5);

  // The 2D overload zeroes z:
  map.setAllPoints(X, Y);
  ASSERT_EQ(map.size(), 3U);
  map.getPoint(2, p);
  EXPECT_NEAR(p.z, 0.0, 1e-5);

  const std::vector<float> shorter{1.0f};
  EXPECT_THROW(map.setAllPoints(X, shorter), std::exception);
  EXPECT_THROW(map.setAllPoints(X, Y, shorter), std::exception);
}

TEST(CPointsMapInsertObs, save2DToTextStream)
{
  CSimplePointsMap map;
  map.insertPoint(1.0f, 2.0f, 3.0f);
  map.insertPoint(4.0f, 5.0f, 6.0f);

  std::ostringstream ss;
  EXPECT_TRUE(map.save2D_to_text_stream(ss));

  // Two lines, each with the x,y pair only:
  const std::string s = ss.str();
  EXPECT_NE(s.find("1.000000 2.000000"), std::string::npos);
  EXPECT_NE(s.find("4.000000 5.000000"), std::string::npos);
  EXPECT_EQ(s.find("3.000000"), std::string::npos);
}

TEST(CPointsMapInsertObs, nnSearchOverloads)
{
  const CSimplePointsMap map = makeGrid();

  // 2D multiple search:
  {
    std::vector<mrpt::math::TPoint2Df> results;
    std::vector<float> dists;
    std::vector<uint64_t> ids;
    map.nn_multiple_search({0.1f, 0.1f}, 3, results, dists, ids);
    EXPECT_EQ(results.size(), 3U);
    EXPECT_EQ(ids.size(), 3U);
  }
  // 3D multiple search:
  {
    std::vector<mrpt::math::TPoint3Df> results;
    std::vector<float> dists;
    std::vector<uint64_t> ids;
    map.nn_multiple_search({0.1f, 0.1f, 0.0f}, 4, results, dists, ids);
    EXPECT_EQ(results.size(), 4U);
  }
  // 2D radius search, both the unbounded and the maxPoints variants:
  {
    std::vector<mrpt::math::TPoint2Df> results;
    std::vector<float> dists;
    std::vector<uint64_t> ids;
    map.nn_radius_search({0.0f, 0.0f}, 1.1f * 1.1f, results, dists, ids, 0);
    EXPECT_EQ(results.size(), 3U);  // itself + the two neighbors at 1 m

    map.nn_radius_search({0.0f, 0.0f}, 1.1f * 1.1f, results, dists, ids, 2);
    EXPECT_EQ(results.size(), 2U);
    EXPECT_EQ(dists.size(), 2U);
  }
  // 3D radius search with a maximum number of points:
  {
    std::vector<mrpt::math::TPoint3Df> results;
    std::vector<float> dists;
    std::vector<uint64_t> ids;
    map.nn_radius_search({0.0f, 0.0f, 0.0f}, 1.1f * 1.1f, results, dists, ids, 2);
    EXPECT_EQ(results.size(), 2U);

    map.nn_radius_search({0.0f, 0.0f, 0.0f}, 1.1f * 1.1f, results, dists, ids, 0);
    EXPECT_EQ(results.size(), 3U);
  }
}

TEST(CPointsMapInsertObs, getVisualizationIntoColorFromZ)
{
  CSimplePointsMap map;
  for (int i = 0; i < 10; i++) map.insertPoint(i, 0, i * 0.1f);

  // Flat color:
  {
    map.renderOptions.colormap = mrpt::img::cmNONE;
    mrpt::viz::CSetOfObjects o;
    map.getVisualizationInto(o);
    EXPECT_EQ(o.size(), 1U);
  }
  // Colorized by height:
  {
    map.renderOptions.colormap = mrpt::img::cmJET;
    mrpt::viz::CSetOfObjects o;
    map.getVisualizationInto(o);
    EXPECT_EQ(o.size(), 1U);
  }
}
