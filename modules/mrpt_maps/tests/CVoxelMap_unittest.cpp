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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/serialization/CArchive.h>

using mrpt::maps::CVoxelMap;
using mrpt::maps::CVoxelMapRGB;

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
