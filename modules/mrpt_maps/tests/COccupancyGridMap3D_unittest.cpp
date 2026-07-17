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
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <test_mrpt_common.h>

#include <filesystem>
#include <sstream>

TEST(COccupancyGridMap3DTests, insert2DScan)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  // Insert the scan in the grid map and check expected values:
  {
    mrpt::maps::COccupancyGridMap3D grid;
    grid.insertObservation(scan1);

    // A cell in front of the laser should have a high "freeness"
    EXPECT_GT(grid.getFreenessByPos(0.5, 0, 0), 0.53f);
  }
}

TEST(COccupancyGridMap3DTests, NearestNeighborsCapable)
{
  // low freeness=occupied
  const float occupied = 0.2f, resolution = 0.10f;
  const std::vector<mrpt::math::TPoint3Df> occupiedPoints = {
      {1.0f, 2.0f, 1.0f},
      {1.1f, 2.0f, 1.0f},
      {1.2f, 2.0f, 1.0f},
      {2.0f, 3.0f, 2.0f}
  };

  mrpt::maps::COccupancyGridMap3D grid({-10.0f, -10.0f, -5.0f}, {10.0f, 10.0f, 50.0f}, resolution);
  for (const auto& pt : occupiedPoints) grid.setFreenessByPos(pt.x, pt.y, pt.z, occupied);

  mrpt::maps::NearestNeighborsCapable& nn =
      dynamic_cast<mrpt::maps::NearestNeighborsCapable&>(grid);

  {
    mrpt::math::TPoint3Df result;
    uint64_t resultIndex;
    float out_dist_sqr = 0;
    bool found = nn.nn_single_search({0.90f, 1.95f, 1.0f}, result, out_dist_sqr, resultIndex);
    EXPECT_TRUE(found);
    EXPECT_NEAR(result.x, 1.0f, resolution);
    EXPECT_NEAR(result.y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dist_sqr), 0.15f, resolution);
  }
  {
    mrpt::math::TPoint3Df result;
    uint64_t resultIndex;
    float out_dist_sqr = 0;
    bool found = nn.nn_single_search({-4.0f, 2.1f, 1.0f}, result, out_dist_sqr, resultIndex);
    EXPECT_TRUE(found);
    EXPECT_NEAR(result.x, 1.0f, resolution);
    EXPECT_NEAR(result.y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dist_sqr), 5.0f, resolution);
  }

  {
    std::vector<mrpt::math::TPoint3Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_multiple_search({-2.0f, 5.0f, 1.0f}, 2, results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), 2UL);
    EXPECT_EQ(out_dists_sqr.size(), results.size());
    EXPECT_EQ(resultIndices.size(), results.size());

    EXPECT_NEAR(results.at(0).x, 1.0f, resolution);
    EXPECT_NEAR(results.at(0).y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dists_sqr.at(0)), std::hypot(-2.0f - 1.0f, 5.0f - 2.0f), resolution);

    EXPECT_NEAR(results.at(1).x, 1.1f, resolution);
    EXPECT_NEAR(results.at(1).y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dists_sqr.at(1)), std::hypot(-2.0f - 1.1f, 5.0f - 2.0f), resolution);
  }
  {
    std::vector<mrpt::math::TPoint3Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_radius_search(
        {-2.0f, 5.0f, 1.0f}, mrpt::square(10.0f), results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), occupiedPoints.size());
    EXPECT_EQ(out_dists_sqr.size(), results.size());

    EXPECT_NEAR(results.at(0).x, 1.0f, resolution);
    EXPECT_NEAR(results.at(0).y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dists_sqr.at(0)), std::hypot(-2.0f - 1.0f, 5.0f - 2.0f), resolution);
  }
  {
    std::vector<mrpt::math::TPoint3Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_radius_search(
        {0.9f, 1.9f, 1.0f}, mrpt::square(1.0f), results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), 3UL);
    EXPECT_EQ(out_dists_sqr.size(), results.size());
  }
  {
    std::vector<mrpt::math::TPoint3Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_radius_search(
        {0.5f, 1.5f, 1.0f}, mrpt::square(0.5f), results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), 0UL);
    EXPECT_EQ(out_dists_sqr.size(), results.size());
  }
}

TEST(COccupancyGridMap3DTests, insertScan3D)
{
  using namespace std::string_literals;

  const auto fil = mrpt::UNITTEST_BASEDIR() + "/tests/test-3d-obs-ground.rawlog"s;
  if (!mrpt::system::fileExists(fil))
  {
    GTEST_FAIL() << "ERROR: test due to missing file: " << fil << "\n";
    return;
  }

  // Load sample 3D scan from file:
  mrpt::obs::CSensoryFrame sf;
  mrpt::io::CCompressedInputStream f(fil);
  mrpt::serialization::archiveFrom(f) >> sf;

  auto obs = sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>();
  ASSERT_(obs);

  {
    mrpt::maps::COccupancyGridMap3D grid;
    grid.insertObservation(*obs);

    // A cell in front of the laser should have a high "freeness"
    EXPECT_GT(grid.getFreenessByPos(0.2f, 0.2f, 0.1f), 0.53f);
  }
}

// =========================================================================
//  Likelihood: canCompute (true for 2D scans, false otherwise) and
//  computeObservationLikelihood (unimplemented -- throws, see the
//  "Implement me!" in COccupancyGridMap3D_likelihood.cpp)
// =========================================================================

TEST(COccupancyGridMap3DTests, CanComputeObservationLikelihood2DScanTrue)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  EXPECT_TRUE(grid.canComputeObservationLikelihood(scan1));
}

TEST(COccupancyGridMap3DTests, CanComputeObservationLikelihoodOtherTypeFalse)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation3DRangeScan scan3d;
  EXPECT_FALSE(grid.canComputeObservationLikelihood(scan3d));
}

TEST(COccupancyGridMap3DTests, ComputeObservationLikelihoodThrows)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  EXPECT_THROW(
      grid.computeObservationLikelihood(scan1, mrpt::poses::CPose3D::Identity()), std::exception);
}

// =========================================================================
//  updateCell / fill / setSize / resizeGrid / isEmpty
// =========================================================================

TEST(COccupancyGridMap3DTests, IsEmptyBeforeAndAfterInsertion)
{
  mrpt::maps::COccupancyGridMap3D grid;
  EXPECT_TRUE(grid.isEmpty());

  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  grid.insertObservation(scan1);
  EXPECT_FALSE(grid.isEmpty());
}

TEST(COccupancyGridMap3DTests, UpdateCellAndFill)
{
  mrpt::maps::COccupancyGridMap3D grid({-2.0f, -2.0f, -2.0f}, {2.0f, 2.0f, 2.0f}, 0.5f);
  // fill()/updateCell() are low-level cell manipulators that don't go
  // through OnPostSuccesfulInsertObs(), so isEmpty() (which only reflects
  // whether insertObservation() has ever succeeded) stays true; just check
  // neither call throws.
  EXPECT_NO_THROW(grid.fill(0.5f));
  EXPECT_NO_THROW(grid.updateCell(0, 0, 0, 0.9f));
}

TEST(COccupancyGridMap3DTests, SetSizeAndResizeGrid)
{
  mrpt::maps::COccupancyGridMap3D grid({-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}, 0.5f);
  EXPECT_NO_THROW(grid.setSize({-2.0f, -2.0f, -2.0f}, {2.0f, 2.0f, 2.0f}, 0.5f));
  EXPECT_NO_THROW(grid.resizeGrid({-3.0f, -3.0f, -3.0f}, {3.0f, 3.0f, 3.0f}));
}

// =========================================================================
//  determineMatching2D / compute3DMatchingRatio
// =========================================================================

TEST(COccupancyGridMap3DTests, DetermineMatching2DNotImplementedThrows)
{
  // determineMatching2D() and compute3DMatchingRatio() are stubs (see
  // "Implement me!" in COccupancyGridMap3D.cpp) -- verify the documented
  // (if unfinished) behavior rather than a working match:
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  grid.insertObservation(scan1);

  auto otherMap = mrpt::maps::CSimplePointsMap::Create();
  otherMap->insertPoint(0.5f, 0.0f, 0.0f);

  mrpt::maps::TMatchingParams params;
  params.maxDistForCorrespondence = 1.0f;
  mrpt::maps::TMatchingExtraResults extraResults;
  mrpt::tfest::TMatchingPairList correspondences;

  EXPECT_THROW(
      grid.determineMatching2D(
          otherMap.get(), mrpt::poses::CPose2D(0, 0, 0), correspondences, params, extraResults),
      std::exception);
}

TEST(COccupancyGridMap3DTests, Compute3DMatchingRatioNotImplementedThrows)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  grid.insertObservation(scan1);

  mrpt::maps::CSimplePointsMap otherMap;
  otherMap.insertPoint(0.5f, 0.0f, 0.0f);

  mrpt::maps::TMatchingRatioParams params;
  EXPECT_THROW(
      grid.compute3DMatchingRatio(&otherMap, mrpt::poses::CPose3D::Identity(), params),
      std::exception);
}

// =========================================================================
//  getAsOctoMapVoxels / getVisualizationInto / saveMetricMapRepresentationToFile
// =========================================================================

TEST(COccupancyGridMap3DTests, GetAsOctoMapVoxels)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  grid.insertObservation(scan1);

  mrpt::viz::COctoMapVoxels voxels;
  EXPECT_NO_THROW(grid.getAsOctoMapVoxels(voxels));
}

TEST(COccupancyGridMap3DTests, GetVisualizationInto)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  grid.insertObservation(scan1);

  auto scene = mrpt::viz::CSetOfObjects::Create();
  EXPECT_NO_THROW(grid.getVisualizationInto(*scene));
}

TEST(COccupancyGridMap3DTests, SaveMetricMapRepresentationToFile)
{
  mrpt::maps::COccupancyGridMap3D grid;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  grid.insertObservation(scan1);

  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path();
  const std::string prefix =
      (dir / ("mrpt_COccupancyGridMap3D_unittest_" + std::to_string(static_cast<long>(getpid())) +
              "_" + std::to_string(counter++)))
          .string();

  EXPECT_NO_THROW(grid.saveMetricMapRepresentationToFile(prefix));
  std::filesystem::remove(prefix + "_binary.bt");
  std::filesystem::remove(prefix + "_3D.3Dscene");
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(COccupancyGridMap3DTests, SerializeRoundTrip)
{
  mrpt::maps::COccupancyGridMap3D src;
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  src.insertObservation(scan1);

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
  auto* dst = dynamic_cast<mrpt::maps::COccupancyGridMap3D*>(obj.get());
  ASSERT_NE(dst, nullptr);
  EXPECT_FALSE(dst->isEmpty());
}

// =========================================================================
//  TInsertionOptions / TRenderingOptions: load/save/serialize
// =========================================================================

TEST(COccupancyGridMap3DTests, InsertionOptionsConfigFileRoundTrip)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("ins", "maxDistanceInsertion", 20.0f);
  cfg.write("ins", "maxOccupancyUpdateCertainty", 0.7f);
  cfg.write("ins", "decimation", 2);

  mrpt::maps::COccupancyGridMap3D::TInsertionOptions opts;
  opts.loadFromConfigFile(cfg, "ins");
  EXPECT_NEAR(opts.maxDistanceInsertion, 20.0f, 1e-3f);
  EXPECT_NEAR(opts.maxOccupancyUpdateCertainty, 0.7f, 1e-3f);
  EXPECT_EQ(opts.decimation, 2u);

  mrpt::config::CConfigFileMemory cfg2;
  EXPECT_NO_THROW(opts.saveToConfigFile(cfg2, "ins"));
}

TEST(COccupancyGridMap3DTests, RenderingOptionsSerializeRoundTrip)
{
  mrpt::maps::COccupancyGridMap3D::TRenderingOptions opts;
  opts.generateGridLines = true;
  opts.generateOccupiedVoxels = false;

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    opts.writeToStream(ar);
  }
  buf.Seek(0);

  mrpt::maps::COccupancyGridMap3D::TRenderingOptions dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    dst.readFromStream(ar);
  }
  EXPECT_TRUE(dst.generateGridLines);
  EXPECT_FALSE(dst.generateOccupiedVoxels);
}

// =========================================================================
//  TMapDefinition / optionsByName
// =========================================================================

TEST(COccupancyGridMap3DTests, MapDefinitionLoadAndCreate)
{
  const mrpt::config::CConfigFileMemory cfg(R""""(
[map_occ3d_00_creationOpts]
resolution=0.25

[map_occ3d_00_insertOpts]
maxDistanceInsertion=12.0
)"""");

  mrpt::maps::COccupancyGridMap3D::TMapDefinition def;
  def.loadFromConfigFile(cfg, "map_occ3d_00");

  std::stringstream ss;
  def.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());

  auto map = mrpt::maps::COccupancyGridMap3D::CreateFromMapDefinition(def);
  ASSERT_NE(map, nullptr);
  EXPECT_NEAR(map->insertionOptions.maxDistanceInsertion, 12.0f, 1e-3f);
}

TEST(COccupancyGridMap3DTests, OptionsByName)
{
  mrpt::maps::COccupancyGridMap3D grid;
  const auto opts = grid.optionsByName();
  EXPECT_FALSE(opts.empty());
}
