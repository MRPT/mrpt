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
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
//
#include <test_mrpt_common.h>

using namespace mrpt::maps;

// Helper: serializes a grid to a memory buffer and deserializes it back
static COccupancyGridMap2D roundTrip(const COccupancyGridMap2D& src)
{
  mrpt::io::CMemoryStream buf;
  auto arch_out = mrpt::serialization::archiveFrom(buf);
  arch_out << src;

  buf.Seek(0);
  auto arch_in = mrpt::serialization::archiveFrom(buf);
  COccupancyGridMap2D dst;
  arch_in >> dst;
  return dst;
}

TEST(COccupancyGridMap2DSerializationTests, RoundTripEmpty)
{
  COccupancyGridMap2D src(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  auto dst = roundTrip(src);

  EXPECT_NEAR(dst.getXMin(), src.getXMin(), 1e-4f);
  EXPECT_NEAR(dst.getXMax(), src.getXMax(), 1e-4f);
  EXPECT_NEAR(dst.getYMin(), src.getYMin(), 1e-4f);
  EXPECT_NEAR(dst.getYMax(), src.getYMax(), 1e-4f);
  EXPECT_NEAR(dst.getResolution(), src.getResolution(), 1e-5f);
  EXPECT_EQ(dst.getSizeX(), src.getSizeX());
  EXPECT_EQ(dst.getSizeY(), src.getSizeY());
}

TEST(COccupancyGridMap2DSerializationTests, RoundTripCells)
{
  COccupancyGridMap2D src(-2.0f, 2.0f, -2.0f, 2.0f, 0.10f);
  src.fill(0.5f);

  // Set a few known cells
  src.setCell(src.x2idx(-1.0f), src.y2idx(-1.0f), 0.1f);
  src.setCell(src.x2idx(0.0f), src.y2idx(0.0f), 0.9f);
  src.setCell(src.x2idx(1.0f), src.y2idx(1.0f), 0.3f);

  auto dst = roundTrip(src);

  EXPECT_NEAR(dst.getCell(dst.x2idx(-1.0f), dst.y2idx(-1.0f)), 0.1f, 0.02f);
  EXPECT_NEAR(dst.getCell(dst.x2idx(0.0f), dst.y2idx(0.0f)), 0.9f, 0.02f);
  EXPECT_NEAR(dst.getCell(dst.x2idx(1.0f), dst.y2idx(1.0f)), 0.3f, 0.02f);
  // Unknown cells remain 0.5
  EXPECT_NEAR(dst.getCell(dst.x2idx(0.5f), dst.y2idx(-0.5f)), 0.5f, 0.02f);
}

TEST(COccupancyGridMap2DSerializationTests, RoundTripInsertionOptions)
{
  COccupancyGridMap2D src(-3.0f, 3.0f, -3.0f, 3.0f, 0.05f);
  src.insertionOptions.maxDistanceInsertion = 12.5f;
  src.insertionOptions.maxOccupancyUpdateCertainty = 0.75f;
  src.insertionOptions.wideningBeamsWithDistance = true;

  auto dst = roundTrip(src);

  EXPECT_NEAR(dst.insertionOptions.maxDistanceInsertion, 12.5f, 1e-4f);
  EXPECT_NEAR(dst.insertionOptions.maxOccupancyUpdateCertainty, 0.75f, 1e-4f);
  EXPECT_TRUE(dst.insertionOptions.wideningBeamsWithDistance);
}

// ---------------------------------------------------------------
// Golden blob test: load a pre-generated v6 stream and verify
// ---------------------------------------------------------------

static std::string goldenBlobPath()
{
  return mrpt::mrpt_data_dir() + "/tests/occupancy_grid_v6_golden.bin";
}

TEST(COccupancyGridMap2DSerializationTests, LegacyV6GoldenBlob)
{
  const auto blobFile = goldenBlobPath();

  // If the blob file does not exist the test is skipped with an informative
  // message (it must be generated once and committed to the repo).
  if (!mrpt::system::fileExists(blobFile))
  {
    GTEST_SKIP() << "Golden blob not found at: " << blobFile
                 << " -- run the GenerateV6GoldenBlob test once to create it.";
    return;
  }

  mrpt::io::CFileInputStream f(blobFile);
  auto arch = mrpt::serialization::archiveFrom(f);
  COccupancyGridMap2D grid;
  arch >> grid;

  // These values match what GenerateV6GoldenBlob wrote
  EXPECT_NEAR(grid.getResolution(), 0.10f, 1e-4f);
  EXPECT_NEAR(grid.getXMin(), -5.0f, 0.01f);
  EXPECT_NEAR(grid.getXMax(), 5.0f, 0.01f);
  EXPECT_NEAR(grid.getYMin(), -5.0f, 0.01f);
  EXPECT_NEAR(grid.getYMax(), 5.0f, 0.01f);
  // Center cell was set to occupied (0.1)
  EXPECT_LT(grid.getCell(grid.x2idx(0.0f), grid.y2idx(0.0f)), 0.2f);
  // A free cell near (1,1)
  EXPECT_GT(grid.getCell(grid.x2idx(1.0f), grid.y2idx(1.0f)), 0.8f);
}

// Run this test manually (--gtest_filter=*GenerateV6GoldenBlob) ONCE before
// any refactoring to produce the committed golden blob.
TEST(COccupancyGridMap2DSerializationTests, GenerateV6GoldenBlob)
{
  const auto blobFile = goldenBlobPath();

  COccupancyGridMap2D grid(-5.0f, 5.0f, -5.0f, 5.0f, 0.10f);
  grid.fill(0.5f);
  grid.setCell(grid.x2idx(0.0f), grid.y2idx(0.0f), 0.1f);  // occupied center
  grid.setCell(grid.x2idx(1.0f), grid.y2idx(1.0f), 0.9f);  // free cell

  mrpt::io::CFileOutputStream f(blobFile);
  auto arch = mrpt::serialization::archiveFrom(f);
  arch << grid;

  std::cout << "[GenerateV6GoldenBlob] Written to: " << blobFile << "\n";
  SUCCEED();
}
