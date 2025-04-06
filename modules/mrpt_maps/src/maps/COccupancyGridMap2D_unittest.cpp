/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
//
#include <mrpt/config.h>
#include <test_mrpt_common.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

TEST(COccupancyGridMap2DTests, insert2DScan)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  stock_observations::example2DRangeScan(scan1);

  // Insert the scan in the grid map and check expected values:
  {
    COccupancyGridMap2D grid(-50.0f, 50.0f, -50.0f, 50.0f, 0.10f);
    grid.insertObservation(scan1);

    EXPECT_GT(grid.getPos(0.5, 0), 0.51f);  // A cell in front of the laser
                                            // should have a high "freeness"
  }
}

TEST(COccupancyGridMap2DTests, NearestNeighborsCapable)
{
  // low freeness=occupied
  const float occupied = 0.2f, resolution = 0.10f;
  const std::vector<mrpt::math::TPoint2Df> occupiedPoints = {
      {1.0f, 2.0f},
      {1.1f, 2.0f},
      {1.2f, 2.0f},
      {2.0f, 3.0f}
  };

  COccupancyGridMap2D grid(-10.0f, 10.0f, -10.0f, 10.0f, resolution);
  for (const auto& pt : occupiedPoints) grid.setCell(grid.x2idx(pt.x), grid.x2idx(pt.y), occupied);

  mrpt::maps::NearestNeighborsCapable& nn =
      dynamic_cast<mrpt::maps::NearestNeighborsCapable&>(grid);

  {
    mrpt::math::TPoint2Df result;
    uint64_t resultIndex;
    float out_dist_sqr = 0;
    bool found = nn.nn_single_search({0.90f, 1.95f}, result, out_dist_sqr, resultIndex);
    EXPECT_TRUE(found);
    EXPECT_NEAR(result.x, 1.0f, resolution);
    EXPECT_NEAR(result.y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dist_sqr), 0.15f, resolution);
  }
  {
    mrpt::math::TPoint2Df result;
    float out_dist_sqr = 0;
    uint64_t resultIndex;
    bool found = nn.nn_single_search({-4.0f, 2.1f}, result, out_dist_sqr, resultIndex);
    EXPECT_TRUE(found);
    EXPECT_NEAR(result.x, 1.0f, resolution);
    EXPECT_NEAR(result.y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dist_sqr), 5.0f, resolution);
  }

  {
    std::vector<mrpt::math::TPoint2Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_multiple_search({-2.0f, 5.0f}, 2, results, out_dists_sqr, resultIndices);

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
    std::vector<mrpt::math::TPoint2Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_radius_search({-2.0f, 5.0f}, mrpt::square(10.0f), results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), occupiedPoints.size());
    EXPECT_EQ(out_dists_sqr.size(), results.size());

    EXPECT_NEAR(results.at(0).x, 1.0f, resolution);
    EXPECT_NEAR(results.at(0).y, 2.0f, resolution);
    EXPECT_NEAR(std::sqrt(out_dists_sqr.at(0)), std::hypot(-2.0f - 1.0f, 5.0f - 2.0f), resolution);
  }
  {
    std::vector<mrpt::math::TPoint2Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_radius_search({0.9f, 1.9f}, mrpt::square(1.0f), results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), 3UL);
    EXPECT_EQ(out_dists_sqr.size(), results.size());
  }
  {
    std::vector<mrpt::math::TPoint2Df> results;
    std::vector<float> out_dists_sqr;
    std::vector<uint64_t> resultIndices;
    nn.nn_radius_search({0.5f, 1.5f}, mrpt::square(0.5f), results, out_dists_sqr, resultIndices);

    EXPECT_EQ(results.size(), 0UL);
    EXPECT_EQ(out_dists_sqr.size(), results.size());
  }
}

// We need OPENCV to read the image.
#if MRPT_HAS_OPENCV && MRPT_HAS_FYAML

TEST(COccupancyGridMap2DTests, loadFromROSMapServerYAML)
{
  using namespace std::string_literals;
  const auto fil = mrpt::UNITTEST_BASEDIR() + "/tests/yaml_32.yaml"s;

  auto grid = mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML(fil);

  ASSERT_EQUAL_(grid.getResolution(), 0.15f);
  ASSERT_NEAR_(grid.getXMin(), -4.5f, 0.01f);
  ASSERT_NEAR_(grid.getXMax(), 0.f, 0.01f);
  ASSERT_NEAR_(grid.getYMin(), 0.f, 0.01f);
  ASSERT_NEAR_(grid.getYMax(), 9.9f, 0.01f);

  ASSERT_EQUAL_(grid.getPos(2.0, 2.0), 0.5f);
}

#endif
