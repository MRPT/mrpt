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
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>

#include <filesystem>
#include <sstream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

TEST(COctoMapTests, updateVoxels)
{
  // Copied from the example program in the "octomap" C++ library.

  COctoMap map(0.1);

  map.updateVoxel(1, 1, 1, true);  // integrate 'occupied' measurement

  map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
  map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement
  map.updateVoxel(1.5, 1, 1, true);  // integrate 'occupied' measurement

  map.updateVoxel(-1, -1, 1, false);  // integrate 'occupied' measurement

  double occup;
  bool is_mapped;
  mrpt::math::TPoint3D pt;

  pt = mrpt::math::TPoint3D(1, 1, 1);
  is_mapped = map.getPointOccupancy(
      static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z), occup);
  EXPECT_GT(occup, 0.5);
  EXPECT_TRUE(is_mapped);

  pt = mrpt::math::TPoint3D(-1, -1, 1);
  is_mapped = map.getPointOccupancy(
      static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z), occup);
  EXPECT_LT(occup, 0.5);
  EXPECT_TRUE(is_mapped);
}

TEST(COctoMapTests, insert2DScan)
{
  // Load scans:
  mrpt::obs::CObservation2DRangeScan scan1;
  stock_observations::example2DRangeScan(scan1);

  // Insert the scan in the map and check expected values:
  {
    COctoMap map(0.1);
    map.insertObservation(scan1);
  }
}

namespace
{
CObservation3DRangeScan makeSimple3DScan()
{
  CObservation3DRangeScan o;
  o.hasPoints3D = true;
  o.points3D_x = {1.0f, 1.0f, -1.0f};
  o.points3D_y = {0.0f, 0.5f, -1.0f};
  o.points3D_z = {0.0f, 0.5f, 1.0f};
  o.sensorPose = CPose3D::Identity();
  return o;
}
}  // namespace

TEST(COctoMapTests, Insert3DScan)
{
  COctoMap map(0.1);
  EXPECT_TRUE(map.insertObservation(makeSimple3DScan()));
  EXPECT_GT(map.size(), 0u);
}

TEST(COctoMapTests, InsertEmpty3DScanReturnsFalse)
{
  COctoMap map(0.1);
  CObservation3DRangeScan o;
  o.hasPoints3D = false;
  EXPECT_FALSE(map.insertObservation(o));
}

TEST(COctoMapTests, GetPointOccupancyUnmappedCell)
{
  COctoMap map(0.1);
  double occ;
  EXPECT_FALSE(map.getPointOccupancy(100.0f, 100.0f, 100.0f, occ));

  const auto opt = map.getPointOccupancy(100.0f, 100.0f, 100.0f);
  EXPECT_FALSE(opt.has_value());
}

TEST(COctoMapTests, ComputeObservationLikelihoodFinite)
{
  COctoMap map(0.1);
  map.insertObservation(makeSimple3DScan());

  const double lik = map.computeObservationLikelihood(makeSimple3DScan(), CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(lik));
}

TEST(COctoMapTests, InsertPointCloudWithSensorOrigin)
{
  COctoMap map(0.1);
  CSimplePointsMap pts;
  pts.insertPoint(1.0f, 0.0f, 0.0f);
  pts.insertPoint(-1.0f, 0.0f, 0.0f);

  map.insertPointCloud(pts, 0.0f, 0.0f, 0.0f);
  EXPECT_GT(map.size(), 0u);
}

TEST(COctoMapTests, CastRayHitsObstacle)
{
  COctoMap map(0.1);
  map.updateVoxel(2.0, 0.0, 0.0, true);

  // The cells between the origin and the obstacle are unmapped (unknown),
  // not free, since only the single occupied voxel above was ever touched;
  // ignoreUnknownCells=true is required for the ray to pass through them
  // instead of aborting on the first unknown cell.
  TPoint3D end;
  const bool hit = map.castRay(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), end, true, -1.0);
  EXPECT_TRUE(hit);
}

TEST(COctoMapTests, IsPointWithinOctoMap)
{
  COctoMap map(0.1);
  // A finite octomap always reports coordinates within its representable
  // range as "within" (it grows dynamically), so just check it doesn't
  // throw for a normal coordinate:
  EXPECT_NO_THROW(map.isPointWithinOctoMap(0.0f, 0.0f, 0.0f));
}

TEST(COctoMapTests, GettersAndSettersRoundTrip)
{
  COctoMap map(0.2);
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

TEST(COctoMapTests, GetAsOctoMapVoxels)
{
  COctoMap map(0.1);
  map.insertObservation(makeSimple3DScan());

  mrpt::viz::COctoMapVoxels voxels;
  EXPECT_NO_THROW(map.getAsOctoMapVoxels(voxels));
}

TEST(COctoMapTests, SaveMetricMapRepresentationToFile)
{
  COctoMap map(0.1);
  map.insertObservation(makeSimple3DScan());

  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path();
  const std::string prefix =
      (dir / ("mrpt_COctoMap_unittest_" + std::to_string(static_cast<long>(getpid())) + "_" +
              std::to_string(counter++)))
          .string();

  EXPECT_NO_THROW(map.saveMetricMapRepresentationToFile(prefix));
  EXPECT_TRUE(std::filesystem::exists(prefix + "_3D.3Dscene"));
  EXPECT_TRUE(std::filesystem::exists(prefix + "_binary.bt"));

  std::filesystem::remove(prefix + "_3D.3Dscene");
  std::filesystem::remove(prefix + "_binary.bt");
}

TEST(COctoMapTests, SerializeRoundTrip)
{
  COctoMap src(0.1);
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
  auto* dst = dynamic_cast<COctoMap*>(obj.get());
  ASSERT_NE(dst, nullptr);
  EXPECT_EQ(dst->size(), src.size());
}

TEST(COctoMapTests, TInsertionOptionsLoadFromConfigFileAndDump)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("ins", "maxrange", 20.0);
  cfg.write("ins", "pruning", true);
  cfg.write("ins", "probHit", 0.75);

  COctoMap map(0.1);
  map.insertionOptions.loadFromConfigFile(cfg, "ins");
  EXPECT_NEAR(map.insertionOptions.maxrange, 20.0, 1e-6);
  EXPECT_TRUE(map.insertionOptions.pruning);
  EXPECT_NEAR(map.getProbHit(), 0.75, 1e-3);

  std::stringstream ss;
  map.insertionOptions.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(COctoMapTests, TLikelihoodOptionsSerializeRoundTripAndDump)
{
  COctoMap::TLikelihoodOptions opts;
  opts.decimation = 5;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    opts.writeToStream(ar);
  }
  buf.Seek(0);

  COctoMap::TLikelihoodOptions dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    dst.readFromStream(ar);
  }
  EXPECT_EQ(dst.decimation, 5);

  std::stringstream ss;
  dst.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(COctoMapTests, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_octo_00_creationOpts]
resolution=0.25

[map_octo_00_insertOpts]
maxrange=15.0
)"""");

  COctoMap::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_octo_00");
  EXPECT_NEAR(def.resolution, 0.25, 1e-6);

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  auto map = COctoMap::CreateFromMapDefinition(def);
  ASSERT_NE(map, nullptr);
  EXPECT_NEAR(map->insertionOptions.maxrange, 15.0, 1e-6);
}
