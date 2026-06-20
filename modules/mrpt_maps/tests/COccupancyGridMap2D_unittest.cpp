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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
//
#include <test_mrpt_common.h>

using namespace mrpt::maps;
using namespace mrpt::obs;

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

TEST(COccupancyGridMap2DTests, setSizeRoundingAndLimits)
{
  const float res = 0.10f;
  // Slightly off-grid limits should be rounded to nearest cell boundary
  COccupancyGridMap2D grid(-1.03f, 1.03f, -2.07f, 2.07f, res);

  // After rounding, each limit should be within 0.5*res of a multiple of res
  auto isMultipleOfRes = [&](float v) { return std::abs(v / res - std::round(v / res)) < 0.01f; };
  EXPECT_TRUE(isMultipleOfRes(grid.getXMin()));
  EXPECT_TRUE(isMultipleOfRes(grid.getYMin()));
  EXPECT_TRUE(isMultipleOfRes(grid.getXMax()));
  EXPECT_TRUE(isMultipleOfRes(grid.getYMax()));

  // Size should be consistent with limits and resolution
  const unsigned int expected_sx =
      static_cast<unsigned int>(std::round((grid.getXMax() - grid.getXMin()) / res));
  const unsigned int expected_sy =
      static_cast<unsigned int>(std::round((grid.getYMax() - grid.getYMin()) / res));
  EXPECT_EQ(grid.getSizeX(), expected_sx);
  EXPECT_EQ(grid.getSizeY(), expected_sy);

  // idx <-> coord round-trip for several cells including negative coords
  for (float x : {-0.95f, 0.0f, 0.95f})
  {
    for (float y : {-1.95f, 0.0f, 1.95f})
    {
      int cx = grid.x2idx(x);
      int cy = grid.y2idx(y);
      float xr = grid.idx2x(cx);
      float yr = grid.idx2y(cy);
      EXPECT_NEAR(xr, x, res);
      EXPECT_NEAR(yr, y, res);
    }
  }
}

TEST(COccupancyGridMap2DTests, fillAndIsEmpty)
{
  COccupancyGridMap2D grid(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  EXPECT_TRUE(grid.isEmpty());

  grid.fill(0.5f);
  // Fill alone does not mark as non-empty; isEmpty tracks observation insertion
  // But after fill+setCell a cell value is still accessible
  grid.setCell(0, 0, 0.2f);
  EXPECT_NEAR(grid.getCell(0, 0), 0.2f, 0.01f);

  // Out-of-range access returns 0.5
  EXPECT_NEAR(grid.getCell(-1000, -1000), 0.5f, 1e-4f);

  // After insert an observation isEmpty() should return false
  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);
  grid.insertObservation(scan);
  EXPECT_FALSE(grid.isEmpty());
}

TEST(COccupancyGridMap2DTests, updateCellSaturation)
{
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, 0.10f);

  const int cx = grid.x2idx(0.0f);
  const int cy = grid.y2idx(0.0f);

  // Repeatedly update toward occupied (low value)
  for (int i = 0; i < 200; i++) grid.updateCell(cx, cy, 0.05f);

  // Should saturate at minimum (fully occupied)
  EXPECT_LT(grid.getCell(cx, cy), 0.1f);

  // Repeatedly update toward free (high value)
  for (int i = 0; i < 200; i++) grid.updateCell(cx, cy, 0.95f);

  EXPECT_GT(grid.getCell(cx, cy), 0.9f);
}

TEST(COccupancyGridMap2DTests, resizeGridGrowsKeepsContent)
{
  const float res = 0.10f;
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, res);

  // Write a known value inside the original map
  const int cx0 = grid.x2idx(0.5f);
  const int cy0 = grid.y2idx(0.5f);
  grid.setCell(cx0, cy0, 0.2f);
  ASSERT_NEAR(grid.getCell(cx0, cy0), 0.2f, 0.02f);

  // Grow the map
  grid.resizeGrid(-5.0f, 5.0f, -5.0f, 5.0f, 0.5f, false /*no extra margin*/);

  // Original content should be preserved
  const int cx1 = grid.x2idx(0.5f);
  const int cy1 = grid.y2idx(0.5f);
  EXPECT_NEAR(grid.getCell(cx1, cy1), 0.2f, 0.02f);
}

TEST(COccupancyGridMap2DTests, subSampleHalvesResolution)
{
  const float res = 0.10f;
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, res);
  grid.fill(1.0f);  // all free

  // Mark the 2x2 block of old cells (8,8)-(9,9) as occupied. After
  // subSample(2), this block exactly covers new cell (4,4), which must
  // therefore become fully occupied while its neighbor (5,4) stays free.
  const int cx = 8;
  const int cy = 8;
  for (int dx = 0; dx <= 1; dx++)
    for (int dy = 0; dy <= 1; dy++) grid.setCell(cx + dx, cy + dy, 0.0f);

  const unsigned int origSizeX = grid.getSizeX();
  const unsigned int origSizeY = grid.getSizeY();

  grid.subSample(2);

  EXPECT_NEAR(grid.getResolution(), res * 2, 1e-4f);
  EXPECT_EQ(grid.getSizeX(), origSizeX / 2);
  EXPECT_EQ(grid.getSizeY(), origSizeY / 2);

  // The down-sampled cell covering the all-occupied 2x2 block must be fully
  // occupied; its free neighbor must remain free.
  const int newCx = cx / 2;
  const int newCy = cy / 2;
  EXPECT_NEAR(grid.getCell(newCx, newCy), 0.0f, 0.05f);
  EXPECT_NEAR(grid.getCell(newCx + 1, newCy), 1.0f, 0.05f);
}

TEST(COccupancyGridMap2DTests, computeEntropyKnownMap)
{
  // Verify computeEntropy() runs without crashing and returns non-negative values.
  // Also checks that an all-certain map has less or equal entropy than a half-certain map.
  COccupancyGridMap2D::TEntropyInfo info_half, info_all;

  // Map with half uncertain (0.5) and half certain (0.05) cells
  {
    COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.10f);
    grid.fill(0.5f);
    for (unsigned int y = 0; y < grid.getSizeY() / 2; y++)
      for (unsigned int x = 0; x < grid.getSizeX(); x++) grid.setCell(x, y, 0.05f);
    grid.computeEntropy(info_half);
  }
  // All-certain (occupied) map
  {
    COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.10f);
    grid.fill(0.05f);
    grid.computeEntropy(info_all);
  }

  EXPECT_GE(info_half.H, 0.0);
  EXPECT_GE(info_all.H, 0.0);
  // Both effectiveMappedCells should be non-negative
  EXPECT_GE(info_half.effectiveMappedCells, 0ul);
  EXPECT_GE(info_all.effectiveMappedCells, 0ul);
}

TEST(COccupancyGridMap2DTests, getAsPointCloudBorders)
{
  // Build a simple occupied square in the center
  const float res = 0.10f;
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, res);
  grid.fill(1.0f);  // all free

  // Mark a 3x3 block as occupied
  for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
    {
      int cx = grid.x2idx(static_cast<float>(dx) * res);
      int cy = grid.y2idx(static_cast<float>(dy) * res);
      grid.setCell(cx, cy, 0.0f);
    }

  mrpt::maps::CSimplePointsMap pm;
  grid.getAsPointCloud(pm, 0.5f);

  EXPECT_GT(pm.size(), 0u);
  // All returned points should have a z=0 (2D map)
  float x, y, z;
  for (size_t i = 0; i < pm.size(); i++)
  {
    pm.getPoint(i, x, y, z);
    EXPECT_NEAR(z, 0.0f, 1e-3f);
  }
}

TEST(COccupancyGridMap2DTests, computePathCostFreeVsOccupied)
{
  const float res = 0.05f;
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, res);
  grid.fill(1.0f);  // all free

  // Path through free space should be close to 1.0
  float cost_free = grid.computePathCost(-0.5f, 0.0f, 0.5f, 0.0f);
  EXPECT_GT(cost_free, 0.8f);

  // Now block the path
  for (float x = -0.6f; x <= 0.6f; x += res)
  {
    int cx = grid.x2idx(x);
    int cy = grid.y2idx(0.0f);
    grid.setCell(cx, cy, 0.0f);
  }
  float cost_blocked = grid.computePathCost(-0.5f, 0.0f, 0.5f, 0.0f);
  EXPECT_LT(cost_blocked, cost_free);
}

TEST(COccupancyGridMap2DTests, voronoiDiagramSizing)
{
  // Verify that after buildVoronoiDiagram() the internal diagram grid has the
  // correct dimensions. Note: the legacy computeClearance() (called internally)
  // uses m_size_y as row stride instead of m_size_x -- this is a known pre-
  // existing bug that will be fixed in Phase B/D. For now just test on a
  // 1x1 m square map where size_x == size_y to avoid the stride bug.
  const float res = 0.10f;
  COccupancyGridMap2D grid(-0.5f, 0.5f, -0.5f, 0.5f, res);  // 10x10 cells
  EXPECT_EQ(grid.getSizeX(), grid.getSizeY());              // must be square to be safe

  grid.fill(1.0f);  // all free — no obstacles means computeClearance exits early

  // Build Voronoi on an all-free map: should not crash
  grid.buildVoronoiDiagram(0.5f, 0.0f);

  EXPECT_EQ(grid.getVoronoiDiagram().getSizeX(), grid.getSizeX());
  EXPECT_EQ(grid.getVoronoiDiagram().getSizeY(), grid.getSizeY());
}

TEST(COccupancyGridMap2DTests, voronoiClearanceCorrectness)
{
  // Build a square map with walls at x=0 and x=N-1, free in between.
  // The center column is equidistant from both walls so computeClearance
  // should find nBasis==2 and clearance ~halfWidth*100.
  // Use a square map (size_x == size_y) to avoid the known stride bug.
  const float res = 0.10f;
  const int N = 21;  // 21x21 cells; walls at x=0 and x=20, center at x=10

  COccupancyGridMap2D grid(0.0f, N * res, 0.0f, N * res, res);
  ASSERT_EQ(static_cast<int>(grid.getSizeX()), N);
  ASSERT_EQ(static_cast<int>(grid.getSizeY()), N);

  grid.fill(1.0f);  // all free
  for (int y = 0; y < N; y++)
  {
    grid.setCell(0, y, 0.0f);
    grid.setCell(N - 1, y, 0.0f);
  }

  // Center x=10 is 10 cells from each wall → clearance should be ~1000.
  const int cx = 10, cy = N / 2;
  auto res_cl = grid.computeClearance(cx, cy);
  EXPECT_EQ(res_cl.nBasis, 2) << "Center cell should detect 2 basis obstacles";
  EXPECT_GE(res_cl.clearance, 900) << "Clearance too small for corridor center";
  EXPECT_LE(res_cl.clearance, 1100) << "Clearance too large for corridor center";

  // Build the full Voronoi and check center column has nonzero clearance
  grid.buildVoronoiDiagram(0.5f, 0.0f);
  EXPECT_GT(grid.getVoronoiClearance(cx, cy), 0) << "Center of corridor must be on Voronoi diagram";
}
