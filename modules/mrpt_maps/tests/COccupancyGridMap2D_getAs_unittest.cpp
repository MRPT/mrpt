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
#include <mrpt/img/CImage.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CTexturedPlane.h>

using namespace mrpt::maps;

namespace
{
// A single row with three cells: a strongly occupied one, an unknown one,
// and a strongly free one. Useful to exercise the gray/RGB/tricolor
// branches of getAsImage().
COccupancyGridMap2D makeThreeCellRowGrid()
{
  COccupancyGridMap2D grid(0.0f, 3.0f, 0.0f, 1.0f, 1.0f);
  EXPECT_EQ(grid.getSizeX(), 3u);
  EXPECT_EQ(grid.getSizeY(), 1u);
  grid.setCell(0, 0, 0.05f);  // occupied
  grid.setCell(1, 0, 0.50f);  // unknown
  grid.setCell(2, 0, 0.95f);  // free
  return grid;
}

// A single column with two cells (occupied, then free), used to check that
// verticalFlip reorders the output image rows.
COccupancyGridMap2D makeTwoCellColumnGrid()
{
  COccupancyGridMap2D grid(0.0f, 1.0f, 0.0f, 2.0f, 1.0f);
  EXPECT_EQ(grid.getSizeX(), 1u);
  EXPECT_EQ(grid.getSizeY(), 2u);
  grid.setCell(0, 0, 0.05f);  // occupied
  grid.setCell(0, 1, 0.95f);  // free
  return grid;
}
}  // namespace

// =========================================================================
//  getAsImage(): all combinations of verticalFlip / forceRGB / tricolor
// =========================================================================

TEST(COccupancyGridMap2D_GetAs, ImageAllParamCombinationsProduceValidImage)
{
  const auto grid = makeThreeCellRowGrid();

  for (const bool verticalFlip : {false, true})
  {
    for (const bool forceRGB : {false, true})
    {
      for (const bool tricolor : {false, true})
      {
        COccupancyGridMap2D::TGetAsImageParams params;
        params.verticalFlip = verticalFlip;
        params.forceRGB = forceRGB;
        params.tricolor = tricolor;

        mrpt::img::CImage img;
        grid.getAsImage(img, params);

        EXPECT_EQ(static_cast<unsigned int>(img.getWidth()), grid.getSizeX());
        EXPECT_EQ(static_cast<unsigned int>(img.getHeight()), grid.getSizeY());
        EXPECT_EQ(img.isColor(), forceRGB);
      }
    }
  }
}

TEST(COccupancyGridMap2D_GetAs, ImageGrayNoTricolorNoFlipValues)
{
  const auto grid = makeThreeCellRowGrid();

  mrpt::img::CImage img;
  grid.getAsImage(img);  // default params: gray, no flip, no tricolor

  EXPECT_FALSE(img.isColor());

  const uint8_t occupiedPixel = img.at<uint8_t>(0, 0);
  const uint8_t unknownPixel = img.at<uint8_t>(1, 0);
  const uint8_t freePixel = img.at<uint8_t>(2, 0);

  EXPECT_LT(occupiedPixel, 60);
  EXPECT_NEAR(unknownPixel, 127, 10);
  EXPECT_GT(freePixel, 200);
}

TEST(COccupancyGridMap2D_GetAs, ImageRGBNoTricolorChannelsMatchGray)
{
  const auto grid = makeThreeCellRowGrid();

  COccupancyGridMap2D::TGetAsImageParams params;
  params.forceRGB = true;

  mrpt::img::CImage img;
  grid.getAsImage(img, params);

  ASSERT_TRUE(img.isColor());
  for (int col = 0; col < 3; col++)
  {
    const uint8_t r = img.at<uint8_t>(col, 0, 0);
    const uint8_t g = img.at<uint8_t>(col, 0, 1);
    const uint8_t b = img.at<uint8_t>(col, 0, 2);
    EXPECT_EQ(r, g);
    EXPECT_EQ(g, b);
  }
  EXPECT_LT(img.at<uint8_t>(0, 0, 0), 60);
  EXPECT_GT(img.at<uint8_t>(2, 0, 0), 200);
}

TEST(COccupancyGridMap2D_GetAs, ImageGrayTricolorQuantizesToThreeLevels)
{
  const auto grid = makeThreeCellRowGrid();

  COccupancyGridMap2D::TGetAsImageParams params;
  params.tricolor = true;

  mrpt::img::CImage img;
  grid.getAsImage(img, params);

  // The tricolor quantization clamps to exactly 0 (occupied), 127 (unknown)
  // or 255 (free), regardless of the exact underlying probability.
  EXPECT_EQ(img.at<uint8_t>(0, 0), 0);
  EXPECT_EQ(img.at<uint8_t>(1, 0), 127);
  EXPECT_EQ(img.at<uint8_t>(2, 0), 255);
}

TEST(COccupancyGridMap2D_GetAs, ImageRGBTricolorQuantizesToThreeLevels)
{
  const auto grid = makeThreeCellRowGrid();

  COccupancyGridMap2D::TGetAsImageParams params;
  params.forceRGB = true;
  params.tricolor = true;

  mrpt::img::CImage img;
  grid.getAsImage(img, params);

  ASSERT_TRUE(img.isColor());
  EXPECT_EQ(img.at<uint8_t>(0, 0, 0), 0);
  EXPECT_EQ(img.at<uint8_t>(1, 0, 0), 127);
  EXPECT_EQ(img.at<uint8_t>(2, 0, 0), 255);
}

TEST(COccupancyGridMap2D_GetAs, ImageVerticalFlipReversesRowOrder)
{
  const auto grid = makeTwoCellColumnGrid();

  mrpt::img::CImage imgNoFlip;
  grid.getAsImage(imgNoFlip);  // verticalFlip=false by default

  mrpt::img::CImage imgFlip;
  COccupancyGridMap2D::TGetAsImageParams params;
  params.verticalFlip = true;
  grid.getAsImage(imgFlip, params);

  // Without flip, the free cell (grid row 1) is drawn on the top image row.
  EXPECT_GT(imgNoFlip.at<uint8_t>(0, 0), 200);
  EXPECT_LT(imgNoFlip.at<uint8_t>(0, 1), 60);

  // With flip, the order is reversed: occupied cell first, free cell last.
  EXPECT_LT(imgFlip.at<uint8_t>(0, 0), 60);
  EXPECT_GT(imgFlip.at<uint8_t>(0, 1), 200);
}

// =========================================================================
//  getAsImageFiltered()
// =========================================================================

TEST(COccupancyGridMap2D_GetAs, ImageFilteredNoOpWhenFilterSizesAreZero)
{
  COccupancyGridMap2D grid(0.0f, 5.0f, 0.0f, 5.0f, 1.0f);
  grid.fill(1.0f);           // all free
  grid.setCell(2, 2, 0.0f);  // single occupied cell in the middle
  grid.insertionOptions.CFD_features_gaussian_size = 0;
  grid.insertionOptions.CFD_features_median_size = 0;

  mrpt::img::CImage imgPlain;
  grid.getAsImage(imgPlain);

  mrpt::img::CImage imgFiltered;
  grid.getAsImageFiltered(imgFiltered);

  ASSERT_EQ(imgFiltered.getWidth(), imgPlain.getWidth());
  ASSERT_EQ(imgFiltered.getHeight(), imgPlain.getHeight());
  for (int y = 0; y < imgPlain.getHeight(); y++)
  {
    for (int x = 0; x < imgPlain.getWidth(); x++)
    {
      EXPECT_EQ(imgFiltered.at<uint8_t>(x, y), imgPlain.at<uint8_t>(x, y));
    }
  }
}

TEST(COccupancyGridMap2D_GetAs, ImageFilteredChangesPixelsWhenFilterSizesAreNonzero)
{
  COccupancyGridMap2D grid(0.0f, 5.0f, 0.0f, 5.0f, 1.0f);
  grid.fill(1.0f);           // all free
  grid.setCell(2, 2, 0.0f);  // single occupied cell surrounded by free space

  mrpt::img::CImage imgPlain;
  grid.getAsImage(imgPlain);

  grid.insertionOptions.CFD_features_gaussian_size = 3;
  grid.insertionOptions.CFD_features_median_size = 3;

  mrpt::img::CImage imgFiltered;
  grid.getAsImageFiltered(imgFiltered);

  ASSERT_EQ(imgFiltered.getWidth(), imgPlain.getWidth());
  ASSERT_EQ(imgFiltered.getHeight(), imgPlain.getHeight());

  // Smoothing a lone dark pixel amid an otherwise uniform bright area must
  // change at least some pixel values with respect to the unfiltered image.
  bool anyPixelChanged = false;
  for (int y = 0; y < imgPlain.getHeight() && !anyPixelChanged; y++)
  {
    for (int x = 0; x < imgPlain.getWidth() && !anyPixelChanged; x++)
    {
      if (imgFiltered.at<uint8_t>(x, y) != imgPlain.at<uint8_t>(x, y))
      {
        anyPixelChanged = true;
      }
    }
  }
  EXPECT_TRUE(anyPixelChanged);
}

TEST(COccupancyGridMap2D_GetAs, ImageFilteredWithDefaultInsertionOptionsRuns)
{
  // Default insertionOptions has non-zero gaussian/median filter sizes, so
  // this exercises the same branches without explicitly configuring them.
  const auto grid = makeThreeCellRowGrid();

  mrpt::img::CImage img;
  EXPECT_NO_THROW(grid.getAsImageFiltered(img));
  EXPECT_EQ(static_cast<unsigned int>(img.getWidth()), grid.getSizeX());
  EXPECT_EQ(static_cast<unsigned int>(img.getHeight()), grid.getSizeY());
}

// =========================================================================
//  getVisualizationInto()
// =========================================================================

TEST(COccupancyGridMap2D_GetAs, VisualizationInsertsOneTexturedPlaneByDefault)
{
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, 0.5f);
  grid.fill(0.5f);

  ASSERT_TRUE(grid.genericMapParams.enableSaveAs3DObject);

  mrpt::viz::CSetOfObjects o;
  EXPECT_EQ(o.size(), 0u);
  grid.getVisualizationInto(o);

  ASSERT_EQ(o.size(), 1u);
  const auto plane = o.getByClass<mrpt::viz::CTexturedPlane>();
  EXPECT_TRUE(plane != nullptr);
}

TEST(COccupancyGridMap2D_GetAs, VisualizationDisabledLeavesSetUnchanged)
{
  COccupancyGridMap2D grid(-1.0f, 1.0f, -1.0f, 1.0f, 0.5f);
  grid.fill(0.5f);
  grid.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects o;
  grid.getVisualizationInto(o);

  EXPECT_EQ(o.size(), 0u);
}

// =========================================================================
//  getAsPointCloud()
// =========================================================================

TEST(COccupancyGridMap2D_GetAs, PointCloudAllFreeGridIsEmpty)
{
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, 0.10f);
  grid.fill(1.0f);  // all free, no occupied cells at all

  mrpt::maps::CSimplePointsMap pm;
  grid.getAsPointCloud(pm, 0.5f);

  EXPECT_EQ(pm.size(), 0u);
}

TEST(COccupancyGridMap2D_GetAs, PointCloudIsolatedOccupiedCellAppearsAsPoint)
{
  const float res = 0.10f;
  COccupancyGridMap2D grid(-2.0f, 2.0f, -2.0f, 2.0f, res);
  grid.fill(1.0f);  // all free

  // A single occupied cell, away from the grid border, has all-free
  // neighbors, so it is a "border occupied cell" (adjacent to free space)
  // and must be reported as a point.
  const int cx = grid.x2idx(0.0f);
  const int cy = grid.y2idx(0.0f);
  grid.setCell(cx, cy, 0.0f);

  mrpt::maps::CSimplePointsMap pm;
  grid.getAsPointCloud(pm, 0.5f);

  ASSERT_EQ(pm.size(), 1u);
  float x, y, z;
  pm.getPoint(0, x, y, z);
  EXPECT_NEAR(x, grid.idx2x(cx), res);
  EXPECT_NEAR(y, grid.idx2y(cy), res);
}
