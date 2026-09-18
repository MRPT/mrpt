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
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>

#include <sstream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

namespace
{
// CColouredOctoMap::internal_insertObservation() unprojects 3D scans via
// unprojectInto(), which requires a valid range image + camera intrinsics
// (hasPoints3D/points3D_x_y_z alone are NOT enough here, unlike COctoMap's
// generic 3D-scan path).
CObservation3DRangeScan makeSimple3DScan()
{
  CObservation3DRangeScan o;
  o.hasRangeImage = true;
  o.range_is_depth = true;
  o.rangeUnits = 0.001f;
  o.cameraParams.ncols = 8;
  o.cameraParams.nrows = 8;
  o.cameraParams.setIntrinsicParamsFromValues(100, 100, 4, 4);
  o.rangeImage.setSize(8, 8);
  o.rangeImage.fill(2000);  // 2000 * rangeUnits = 2.0 m depth
  o.sensorPose = CPose3D::Identity();
  return o;
}
}  // namespace

TEST(CColouredOctoMap, Insert2DScan)
{
  CObservation2DRangeScan scan1;
  stock_observations::example2DRangeScan(scan1);

  CColouredOctoMap map(0.1);
  EXPECT_TRUE(map.insertObservation(scan1));
  EXPECT_GT(map.size(), 0u);
}

TEST(CColouredOctoMap, Insert3DScanUpdatesColour)
{
  CColouredOctoMap map(0.1);
  EXPECT_TRUE(map.insertObservation(makeSimple3DScan()));
  EXPECT_GT(map.size(), 0u);
}

TEST(CColouredOctoMap, InsertUnsupportedObservationReturnsFalse)
{
  CColouredOctoMap map(0.1);
  // internal_insertObservation() only special-cases CObservation2DRangeScan
  // and CObservation3DRangeScan; anything else falls through to `return
  // false`:
  mrpt::obs::CObservationComment unrelated;
  EXPECT_FALSE(map.insertObservation(unrelated));
}

TEST(CColouredOctoMap, GetPointColourUnmappedCellReturnsFalse)
{
  CColouredOctoMap map(0.1);
  uint8_t r, g, b;
  EXPECT_FALSE(map.getPointColour(100.0f, 100.0f, 100.0f, r, g, b));
}

TEST(CColouredOctoMap, UpdateVoxelColourAllMethods)
{
  CColouredOctoMap map(0.1);
  map.updateVoxel(0.0, 0.0, 0.0, true);

  uint8_t r, g, b;

  map.setVoxelColourMethod(CColouredOctoMap::SET);
  map.updateVoxelColour(0.0, 0.0, 0.0, 200, 10, 10);
  ASSERT_TRUE(map.getPointColour(0.0f, 0.0f, 0.0f, r, g, b));
  EXPECT_EQ(r, 200);

  map.setVoxelColourMethod(CColouredOctoMap::AVERAGE);
  map.updateVoxelColour(0.0, 0.0, 0.0, 0, 200, 10);
  EXPECT_TRUE(map.getPointColour(0.0f, 0.0f, 0.0f, r, g, b));

  map.setVoxelColourMethod(CColouredOctoMap::INTEGRATE);
  map.updateVoxelColour(0.0, 0.0, 0.0, 10, 10, 200);
  EXPECT_TRUE(map.getPointColour(0.0f, 0.0f, 0.0f, r, g, b));
}

TEST(CColouredOctoMap, InsertRayAndUpdateVoxel)
{
  CColouredOctoMap map(0.1);
  map.insertRay(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  map.updateVoxel(2.0, 0.0, 0.0, true);
  EXPECT_GT(map.size(), 0u);
}

TEST(CColouredOctoMap, IsPointWithinOctoMap)
{
  CColouredOctoMap map(0.1);
  EXPECT_NO_THROW(map.isPointWithinOctoMap(0.0f, 0.0f, 0.0f));
}

TEST(CColouredOctoMap, GettersAndSettersRoundTrip)
{
  CColouredOctoMap map(0.2);
  EXPECT_NEAR(map.getResolution(), 0.2, 1e-6);

  map.setOccupancyThres(0.6);
  EXPECT_NEAR(map.getOccupancyThres(), 0.6, 1e-3);
  map.setProbHit(0.8);
  EXPECT_NEAR(map.getProbHit(), 0.8, 1e-3);
  map.setProbMiss(0.3);
  EXPECT_NEAR(map.getProbMiss(), 0.3, 1e-3);
  map.setClampingThresMin(0.05);
  EXPECT_NEAR(map.getClampingThresMin(), 0.05, 1e-3);
  map.setClampingThresMax(0.95);
  EXPECT_NEAR(map.getClampingThresMax(), 0.95, 1e-3);

  EXPECT_TRUE(std::isfinite(map.getOccupancyThresLog()));
  EXPECT_TRUE(std::isfinite(map.getProbHitLog()));
  EXPECT_TRUE(std::isfinite(map.getProbMissLog()));
  EXPECT_TRUE(std::isfinite(map.getClampingThresMinLog()));
  EXPECT_TRUE(std::isfinite(map.getClampingThresMaxLog()));

  map.insertObservation(makeSimple3DScan());
  EXPECT_GT(map.getTreeDepth(), 0u);
  EXPECT_GT(map.memoryUsage(), 0u);
  EXPECT_GT(map.memoryUsageNode(), 0u);
  EXPECT_GE(map.memoryFullGrid(), 0u);
  EXPECT_GT(map.volume(), 0.0);
  EXPECT_GT(map.calcNumNodes(), 0u);
  EXPECT_GT(map.getNumLeafNodes(), 0u);

  double x, y, z;
  EXPECT_NO_THROW(map.getMetricSize(x, y, z));
  EXPECT_NO_THROW(map.getMetricMin(x, y, z));
  EXPECT_NO_THROW(map.getMetricMax(x, y, z));
}

TEST(CColouredOctoMap, GetAsOctoMapVoxels)
{
  CColouredOctoMap map(0.1);
  map.insertObservation(makeSimple3DScan());

  mrpt::viz::COctoMapVoxels voxels;
  EXPECT_NO_THROW(map.getAsOctoMapVoxels(voxels));
}

TEST(CColouredOctoMap, SerializeRoundTrip)
{
  CColouredOctoMap src(0.1);
  src.insertObservation(makeSimple3DScan());

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  mrpt::serialization::CSerializable::Ptr obj;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> obj;
  }
  auto* dst = dynamic_cast<CColouredOctoMap*>(obj.get());
  ASSERT_NE(dst, nullptr);
  EXPECT_EQ(dst->size(), src.size());
}

TEST(CColouredOctoMap, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_colorocto_00_creationOpts]
resolution=0.3

[map_colorocto_00_insertOpts]
maxrange=10.0
)"""");

  CColouredOctoMap::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_colorocto_00");
  EXPECT_NEAR(def.resolution, 0.3, 1e-6);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  auto map = CColouredOctoMap::CreateFromMapDefinition(def);
  ASSERT_NE(map, nullptr);
  EXPECT_NEAR(map->insertionOptions.maxrange, 10.0, 1e-6);
}
