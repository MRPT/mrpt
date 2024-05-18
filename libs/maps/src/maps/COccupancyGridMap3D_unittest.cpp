/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

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

// We need OPENCV to read the image internal to CObservation3DRangeScan,
// so skip this test if built without opencv.
#if MRPT_HAS_OPENCV

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
  mrpt::io::CFileGZInputStream f(fil);
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

#endif
