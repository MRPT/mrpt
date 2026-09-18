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
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest/TMatchingPair.h>
//
#include <test_mrpt_common.h>

#include <limits>

using namespace mrpt::maps;
using namespace std::string_literals;

namespace
{
std::string testDataDir() { return mrpt::UNITTEST_BASEDIR() + "/tests"s; }

// A small grid with a couple of distinct cell probabilities, used to test
// the bitmap save/load round-trip.
COccupancyGridMap2D makeSmallGrid()
{
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, 0.2f);
  grid.fill(0.5f);
  grid.setCell(grid.x2idx(-0.5f), grid.y2idx(-0.5f), 0.05f);  // occupied
  grid.setCell(grid.x2idx(0.5f), grid.y2idx(0.5f), 0.95f);    // free
  return grid;
}
}  // namespace

// =========================================================================
//  saveAsBitmapFile() / loadFromBitmapFile()
// =========================================================================

TEST(COccupancyGridMap2D_IO, SaveAndLoadBitmapFileRoundTrip)
{
  const auto src = makeSmallGrid();

  const std::string file = mrpt::system::getTempFileName() + ".png"s;
  ASSERT_TRUE(src.saveAsBitmapFile(file));
  ASSERT_TRUE(mrpt::system::fileExists(file));

  // The bitmap file only stores cell values, not the world-coordinate
  // origin, so it must be re-centered on load with the same sentinel `src`
  // itself is centered around (0,0), matching src's own placement.
  const mrpt::math::TPoint2D middleSentinel(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  COccupancyGridMap2D dst;
  ASSERT_TRUE(dst.loadFromBitmapFile(file, src.getResolution(), middleSentinel));

  EXPECT_EQ(dst.getSizeX(), src.getSizeX());
  EXPECT_EQ(dst.getSizeY(), src.getSizeY());
  EXPECT_NEAR(
      dst.getCell(dst.x2idx(-0.5f), dst.y2idx(-0.5f)),
      src.getCell(src.x2idx(-0.5f), src.y2idx(-0.5f)), 0.05f);
  EXPECT_NEAR(
      dst.getCell(dst.x2idx(0.5f), dst.y2idx(0.5f)), src.getCell(src.x2idx(0.5f), src.y2idx(0.5f)),
      0.05f);
}

TEST(COccupancyGridMap2D_IO, LoadFromBitmapFileMissingFileReturnsFalse)
{
  COccupancyGridMap2D dst;
  EXPECT_FALSE(dst.loadFromBitmapFile(
      testDataDir() + "/this_file_does_not_exist.png"s, 0.1f, mrpt::math::TPoint2D(0, 0)));
}

// =========================================================================
//  loadFromBitmap(): explicit origin vs. the "middle of bitmap" sentinel
// =========================================================================

TEST(COccupancyGridMap2D_IO, LoadFromBitmapDefaultOriginCentersImage)
{
  mrpt::img::CImage img(10, 10, mrpt::img::CH_GRAY);
  img.filledRectangle({0, 0}, {9, 9}, mrpt::img::TColor(128, 128, 128));

  const float res = 0.1f;
  const mrpt::math::TPoint2D middleSentinel(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

  COccupancyGridMap2D grid;
  ASSERT_TRUE(grid.loadFromBitmap(img, res, middleSentinel));

  // Origin at the middle of a 10x10 image means the grid extends 5 cells
  // (0.5 m) in each direction from (0,0).
  EXPECT_NEAR(grid.getXMin(), -0.5f, 1e-3f);
  EXPECT_NEAR(grid.getXMax(), 0.5f, 1e-3f);
  EXPECT_NEAR(grid.getYMin(), -0.5f, 1e-3f);
  EXPECT_NEAR(grid.getYMax(), 0.5f, 1e-3f);
}

TEST(COccupancyGridMap2D_IO, LoadFromBitmapExplicitOrigin)
{
  mrpt::img::CImage img(10, 10, mrpt::img::CH_GRAY);
  img.filledRectangle({0, 0}, {9, 9}, mrpt::img::TColor(128, 128, 128));

  const float res = 0.1f;
  // Origin at the image's top-left corner: the grid extends purely to
  // positive x/y.
  COccupancyGridMap2D grid;
  ASSERT_TRUE(grid.loadFromBitmap(img, res, mrpt::math::TPoint2D(0, 0)));

  EXPECT_NEAR(grid.getXMin(), 0.0f, 1e-3f);
  EXPECT_NEAR(grid.getXMax(), 1.0f, 1e-3f);
  EXPECT_NEAR(grid.getYMin(), 0.0f, 1e-3f);
  EXPECT_NEAR(grid.getYMax(), 1.0f, 1e-3f);
}

// =========================================================================
//  saveAsBitmapTwoMapsWithCorrespondences() / saveMetricMapRepresentationToFile()
// =========================================================================

TEST(COccupancyGridMap2D_IO, SaveAsBitmapTwoMapsWithCorrespondences)
{
  const auto m1 = makeSmallGrid();
  const auto m2 = makeSmallGrid();

  mrpt::tfest::TMatchingPairList corrs;
  corrs.push_back(mrpt::tfest::TMatchingPair(0, 0, -0.5f, -0.5f, 0.0f, -0.5f, -0.5f, 0.0f));

  const std::string file = mrpt::system::getTempFileName() + ".png"s;
  EXPECT_TRUE(COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(file, m1, m2, corrs));
  EXPECT_TRUE(mrpt::system::fileExists(file));
}

TEST(COccupancyGridMap2D_IO, SaveMetricMapRepresentationToFileCreatesPngAndLimits)
{
  const auto grid = makeSmallGrid();

  const std::string prefix = mrpt::system::getTempFileName();
  grid.saveMetricMapRepresentationToFile(prefix);

  EXPECT_TRUE(mrpt::system::fileExists(prefix + ".png"s));
  EXPECT_TRUE(mrpt::system::fileExists(prefix + "_limits.txt"s));
}

// =========================================================================
//  loadFromROSMapServerYAML(): branches not already covered by the
//  pre-existing test in COccupancyGridMap2D_unittest.cpp (which only checks
//  the default "trinary" mode with negate=0)
// =========================================================================

TEST(COccupancyGridMap2D_IO, LoadFromROSMapServerYAMLScaleMode)
{
  const auto fil = testDataDir() + "/yaml_32_scale.yaml"s;

  auto grid = COccupancyGridMap2D::FromROSMapServerYAML(fil);

  EXPECT_NEAR(grid.getResolution(), 0.15f, 1e-4f);
  EXPECT_NEAR(grid.getXMin(), -4.5f, 0.01f);
}

TEST(COccupancyGridMap2D_IO, LoadFromROSMapServerYAMLNegate)
{
  const auto filPlain = testDataDir() + "/yaml_32.yaml"s;
  const auto filNegate = testDataDir() + "/yaml_32_negate.yaml"s;

  auto gridPlain = COccupancyGridMap2D::FromROSMapServerYAML(filPlain);
  auto gridNegate = COccupancyGridMap2D::FromROSMapServerYAML(filNegate);

  // Negating the image inverts occupied/free, so a cell that reads as
  // (mostly) free in the plain map should read as (mostly) occupied in the
  // negated one, and vice versa, at any position away from mid-gray.
  const float pPlain = gridPlain.getPos(-2.0, 2.0);
  const float pNegate = gridNegate.getPos(-2.0, 2.0);
  EXPECT_NE(pPlain, pNegate);
}

TEST(COccupancyGridMap2D_IO, LoadFromROSMapServerYAMLMissingImageReturnsFalse)
{
  COccupancyGridMap2D grid;
  EXPECT_FALSE(grid.loadFromROSMapServerYAML(testDataDir() + "/yaml_32_missing_image.yaml"s));
}

TEST(COccupancyGridMap2D_IO, LoadFromROSMapServerYAMLInvalidModeReturnsFalse)
{
  // The unsupported 'mode' value makes the implementation throw internally,
  // but the whole function body is wrapped in a try/catch, so the public
  // API must still just report failure via a `false` return.
  COccupancyGridMap2D grid;
  EXPECT_FALSE(grid.loadFromROSMapServerYAML(testDataDir() + "/yaml_32_badmode.yaml"s));
}
