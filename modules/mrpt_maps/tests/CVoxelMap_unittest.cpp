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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>

#include <sstream>

using mrpt::maps::CVoxelMap;
using mrpt::maps::CVoxelMapRGB;

namespace
{
// See the note in CColouredOctoMap_unittest.cpp: CVoxelMapRGB's 3D-scan path
// unprojects via a range image + camera intrinsics, not raw points3D_*.
mrpt::obs::CObservation3DRangeScan makeRangeImageScan()
{
  mrpt::obs::CObservation3DRangeScan o;
  o.hasRangeImage = true;
  o.range_is_depth = true;
  o.rangeUnits = 0.001f;
  o.cameraParams.ncols = 8;
  o.cameraParams.nrows = 8;
  o.cameraParams.setIntrinsicParamsFromValues(100, 100, 4, 4);
  o.rangeImage.setSize(8, 8);
  o.rangeImage.fill(2000);  // 2000 * rangeUnits = 2.0 m depth
  o.sensorPose = mrpt::poses::CPose3D::Identity();
  return o;
}
}  // namespace

// =========================================================================
//  CVoxelMap: basic construction and isEmpty
// =========================================================================

TEST(CVoxelMap, EmptyOnConstruction)
{
  CVoxelMap m(0.1);
  EXPECT_TRUE(m.isEmpty());
}

// =========================================================================
//  updateVoxel and getPointOccupancy
// =========================================================================

TEST(CVoxelMap, UpdateAndQueryOccupancy)
{
  CVoxelMap m(0.1);

  // Mark a voxel as occupied several times to push log-odds high
  for (int i = 0; i < 10; ++i) m.updateVoxel(0.0, 0.0, 0.0, /*occupied=*/true);

  EXPECT_FALSE(m.isEmpty());

  double prob = 0.0;
  bool found = m.getPointOccupancy(0.0, 0.0, 0.0, prob);
  EXPECT_TRUE(found);
  EXPECT_GT(prob, 0.5);  // must be classified as occupied
}

TEST(CVoxelMap, MarkFreeVoxel)
{
  CVoxelMap m(0.1);

  // Mark a voxel as free several times
  for (int i = 0; i < 10; ++i) m.updateVoxel(1.0, 1.0, 1.0, /*occupied=*/false);

  double prob = 0.0;
  bool found = m.getPointOccupancy(1.0, 1.0, 1.0, prob);
  EXPECT_TRUE(found);
  EXPECT_LT(prob, 0.5);  // must be classified as free
}

TEST(CVoxelMap, QueryUnknownVoxel)
{
  CVoxelMap m(0.1);

  double prob = 0.0;
  bool found = m.getPointOccupancy(999.0, 999.0, 999.0, prob);
  EXPECT_FALSE(found);
}

// =========================================================================
//  getOccupiedVoxels
// =========================================================================

TEST(CVoxelMap, GetOccupiedVoxels)
{
  CVoxelMap m(0.1);
  m.insertionOptions.prob_hit = 0.9;
  m.insertionOptions.clamp_max = 0.99;

  // Insert three well-separated occupied voxels
  for (int i = 0; i < 15; ++i)
  {
    m.updateVoxel(0.0, 0.0, 0.0, true);
    m.updateVoxel(1.0, 0.0, 0.0, true);
    m.updateVoxel(0.0, 1.0, 0.0, true);
  }

  auto pts = m.getOccupiedVoxels();
  ASSERT_NE(pts, nullptr);
  EXPECT_EQ(pts->size(), 3u);
}

// =========================================================================
//  clear
// =========================================================================

TEST(CVoxelMap, Clear)
{
  CVoxelMap m(0.1);
  m.updateVoxel(0.0, 0.0, 0.0, true);
  EXPECT_FALSE(m.isEmpty());

  m.clear();
  EXPECT_TRUE(m.isEmpty());
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CVoxelMap, SerializeRoundTrip)
{
  CVoxelMap src(0.1);
  for (int i = 0; i < 5; ++i) src.updateVoxel(0.0, 0.0, 0.0, true);
  for (int i = 0; i < 5; ++i) src.updateVoxel(1.0, 0.0, 0.0, false);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  // CVoxelMap copy assignment is not implemented (Bonxai limitation).
  // Deserialize into a Ptr and access via pointer.
  CVoxelMap::Ptr dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    dst = std::dynamic_pointer_cast<CVoxelMap>(obj);
    ASSERT_NE(dst, nullptr);
  }

  EXPECT_FALSE(dst->isEmpty());
  double prob = 0;
  EXPECT_TRUE(dst->getPointOccupancy(0.0, 0.0, 0.0, prob));
  EXPECT_GT(prob, 0.5);
}

// =========================================================================
//  insertPointCloudAsEndPoints
// =========================================================================

TEST(CVoxelMap, InsertPointCloudAsEndPoints)
{
  CVoxelMap m(0.1);

  mrpt::math::TPoint3D sensorOrg{0, 0, 0};
  mrpt::maps::CSimplePointsMap pts;
  for (int i = 1; i <= 10; ++i) pts.insertPoint(float(i) * 0.5f, 0.0f, 0.0f);

  m.insertPointCloudAsEndPoints(pts, sensorOrg);
  EXPECT_FALSE(m.isEmpty());
}

// =========================================================================
//  CVoxelMapRGB: smoke tests
// =========================================================================

TEST(CVoxelMapRGB, EmptyOnConstruction)
{
  CVoxelMapRGB m(0.1);
  EXPECT_TRUE(m.isEmpty());
}

TEST(CVoxelMapRGB, UpdateAndClear)
{
  CVoxelMapRGB m(0.1);
  for (int i = 0; i < 5; ++i) m.updateVoxel(0.0, 0.0, 0.0, true);
  EXPECT_FALSE(m.isEmpty());
  m.clear();
  EXPECT_TRUE(m.isEmpty());
}

// =========================================================================
//  CVoxelMap: internal_insertObservation() via generic / point-cloud
//  observations, insertPointCloudAsRays, remove_voxels_farther_than,
//  internal_computeObservationLikelihood
// =========================================================================

TEST(CVoxelMap, InsertGeneric2DScanObservation)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  CVoxelMap m(0.1);
  EXPECT_TRUE(m.insertObservation(scan1));
  EXPECT_FALSE(m.isEmpty());
}

TEST(CVoxelMap, InsertUnhandledObservationTypeReturnsFalse)
{
  CVoxelMap m(0.1);
  mrpt::obs::CObservation3DRangeScan noPoints;
  noPoints.hasPoints3D = false;
  noPoints.hasRangeImage = false;
  EXPECT_FALSE(m.insertObservation(noPoints));
  EXPECT_TRUE(m.isEmpty());
}

TEST(CVoxelMap, InsertObservationPointCloudAsRays)
{
  CVoxelMap m(0.1);
  m.insertionOptions.ray_trace_free_space = true;

  auto pc = mrpt::maps::CSimplePointsMap::Create();
  for (int i = 1; i <= 5; ++i) pc->insertPoint(float(i) * 0.5f, 0.0f, 0.0f);

  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = pc;
  obs.sensorPose = mrpt::poses::CPose3D::Identity();

  EXPECT_TRUE(m.insertObservation(obs));
  EXPECT_FALSE(m.isEmpty());
}

TEST(CVoxelMap, InsertObservationPointCloudEmptyReturnsFalse)
{
  CVoxelMap m(0.1);
  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = mrpt::maps::CSimplePointsMap::Create();  // empty
  EXPECT_FALSE(m.insertObservation(obs));
}

TEST(CVoxelMap, RemoveVoxelsFartherThan)
{
  CVoxelMap m(0.1);
  m.insertionOptions.remove_voxels_farther_than = 0.5;

  // Insert a nearby and a far voxel:
  m.updateVoxel(0.0, 0.0, 0.0, true);
  m.updateVoxel(5.0, 0.0, 0.0, true);

  auto pc = mrpt::maps::CSimplePointsMap::Create();
  pc->insertPoint(0.05f, 0.0f, 0.0f);  // Close to the sensor origin.

  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = pc;
  obs.sensorPose = mrpt::poses::CPose3D::Identity();

  // Triggers the "remove voxels farther than" pruning pass before insertion:
  EXPECT_TRUE(m.insertObservation(obs));
}

TEST(CVoxelMap, ComputeObservationLikelihoodFinite)
{
  CVoxelMap m(0.1);
  for (int i = 0; i < 5; ++i) m.updateVoxel(0.0, 0.0, 0.0, true);

  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  const double lik = m.computeObservationLikelihood(scan1, mrpt::poses::CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(lik));
}

TEST(CVoxelMap, ComputeObservationLikelihoodEmptyPointsReturnsZero)
{
  CVoxelMap m(0.1);
  mrpt::obs::CObservation3DRangeScan noPoints;
  noPoints.hasPoints3D = false;
  noPoints.hasRangeImage = false;
  EXPECT_NEAR(
      m.computeObservationLikelihood(noPoints, mrpt::poses::CPose3D::Identity()), 0.0, 1e-9);
}

TEST(CVoxelMap, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_voxel_00_creationOpts]
resolution=0.2

[map_voxel_00_insertOpts]
max_range=10.0
ray_trace_free_space=true
)"""");

  CVoxelMap::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_voxel_00");
  EXPECT_NEAR(def.resolution, 0.2, 1e-6);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  auto map = CVoxelMap::CreateFromMapDefinition(def);
  ASSERT_NE(map, nullptr);
  EXPECT_NEAR(map->insertionOptions.max_range, 10.0, 1e-6);
  EXPECT_TRUE(map->insertionOptions.ray_trace_free_space);
}

// =========================================================================
//  TVoxelMap_InsertionOptions / TVoxelMap_LikelihoodOptions /
//  TVoxelMap_RenderingOptions: load/save/serialize round-trips
// =========================================================================

TEST(CVoxelMap, InsertionOptionsConfigFileRoundTrip)
{
  mrpt::maps::TVoxelMap_InsertionOptions opts;
  opts.max_range = 12.5;
  opts.prob_hit = 0.8;
  opts.decimation = 3;
  opts.remove_voxels_farther_than = 5.0;

  mrpt::config::CConfigFileMemory cfg;
  opts.saveToConfigFile(cfg, "ins");

  mrpt::maps::TVoxelMap_InsertionOptions loaded;
  loaded.loadFromConfigFile(cfg, "ins");
  EXPECT_NEAR(loaded.max_range, 12.5, 1e-6);
  EXPECT_NEAR(loaded.prob_hit, 0.8, 1e-6);
  EXPECT_EQ(loaded.decimation, 3u);
  EXPECT_NEAR(loaded.remove_voxels_farther_than, 5.0, 1e-6);
}

TEST(CVoxelMap, InsertionOptionsSerializeRoundTrip)
{
  mrpt::maps::TVoxelMap_InsertionOptions opts;
  opts.max_range = 7.5;
  opts.remove_voxels_farther_than = 3.0;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    opts.writeToStream(ar);
  }
  buf.Seek(0);

  mrpt::maps::TVoxelMap_InsertionOptions dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    dst.readFromStream(ar);
  }
  EXPECT_NEAR(dst.max_range, 7.5, 1e-6);
  EXPECT_NEAR(dst.remove_voxels_farther_than, 3.0, 1e-6);
}

TEST(CVoxelMap, RenderingOptionsSerializeRoundTrip)
{
  mrpt::maps::TVoxelMap_RenderingOptions opts;
  opts.generateOccupiedVoxels = false;
  opts.occupiedThreshold = 0.7f;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    opts.writeToStream(ar);
  }
  buf.Seek(0);

  mrpt::maps::TVoxelMap_RenderingOptions dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    dst.readFromStream(ar);
  }
  EXPECT_FALSE(dst.generateOccupiedVoxels);
  EXPECT_NEAR(dst.occupiedThreshold, 0.7f, 1e-3f);
}

TEST(CVoxelMap, LikelihoodOptionsConfigFileAndSerializeRoundTrip)
{
  mrpt::maps::TVoxelMap_LikelihoodOptions opts;
  opts.decimate_up_to = 42;
  opts.occupiedThreshold = 0.65;

  mrpt::config::CConfigFileMemory cfg;
  opts.saveToConfigFile(cfg, "lik");

  mrpt::maps::TVoxelMap_LikelihoodOptions loaded;
  loaded.loadFromConfigFile(cfg, "lik");
  EXPECT_EQ(loaded.decimate_up_to, 42);
  EXPECT_NEAR(loaded.occupiedThreshold, 0.65, 1e-6);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    opts.writeToStream(ar);
  }
  buf.Seek(0);
  mrpt::maps::TVoxelMap_LikelihoodOptions dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    dst.readFromStream(ar);
  }
  EXPECT_EQ(dst.decimate_up_to, 42);
  EXPECT_NEAR(dst.occupiedThreshold, 0.65, 1e-6);
}

// =========================================================================
//  CVoxelMapRGB: 3D-scan-with-color path, generic path, likelihood,
//  TMapDefinition
// =========================================================================

TEST(CVoxelMapRGB, Insert3DScanWithColour)
{
  CVoxelMapRGB m(0.1);
  EXPECT_TRUE(m.insertObservation(makeRangeImageScan()));
  EXPECT_FALSE(m.isEmpty());
}

TEST(CVoxelMapRGB, InsertGeneric2DScanObservation)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  CVoxelMapRGB m(0.1);
  EXPECT_TRUE(m.insertObservation(scan1));
  EXPECT_FALSE(m.isEmpty());
}

TEST(CVoxelMapRGB, ComputeObservationLikelihoodFinite)
{
  CVoxelMapRGB m(0.1);
  for (int i = 0; i < 5; ++i) m.updateVoxel(0.0, 0.0, 0.0, true);

  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  const double lik = m.computeObservationLikelihood(scan1, mrpt::poses::CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(lik));
}

TEST(CVoxelMapRGB, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_voxelrgb_00_creationOpts]
resolution=0.15

[map_voxelrgb_00_insertOpts]
prob_hit=0.85
)"""");

  CVoxelMapRGB::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_voxelrgb_00");
  EXPECT_NEAR(def.resolution, 0.15, 1e-6);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  auto map = CVoxelMapRGB::CreateFromMapDefinition(def);
  ASSERT_NE(map, nullptr);
  EXPECT_NEAR(map->insertionOptions.prob_hit, 0.85, 1e-6);
}
