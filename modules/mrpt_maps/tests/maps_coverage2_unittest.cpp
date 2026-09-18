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

// Assorted gaps left by the per-class test files: the generic point-field
// accessors of CPointsMap, the voxel-map visualization coloring schemes and
// the textual description of a CObservationPointCloud.

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/viz/COctoMapVoxels.h>

#include <limits>
#include <sstream>

using namespace mrpt::maps;
using mrpt::obs::CObservationPointCloud;

TEST(CPointsMapFields, genericFieldAccessors)
{
  CSimplePointsMap m;
  m.insertPoint(1.0f, 2.0f, 3.0f);
  m.insertPoint(4.0f, 5.0f, 6.0f);

  // The three built-in float fields:
  EXPECT_NEAR(m.getPointField_float(0, "x"), 1.0f, 1e-6f);
  EXPECT_NEAR(m.getPointField_float(0, "y"), 2.0f, 1e-6f);
  EXPECT_NEAR(m.getPointField_float(0, "z"), 3.0f, 1e-6f);
  // Any other field name yields zero rather than throwing:
  EXPECT_NEAR(m.getPointField_float(0, "intensity"), 0.0f, 1e-6f);

  // A plain points map has no non-float fields at all:
  EXPECT_NEAR(m.getPointField_double(0, "anything"), 0.0, 1e-12);
  EXPECT_EQ(m.getPointField_uint16(0, "anything"), 0);
  EXPECT_EQ(m.getPointField_uint8(0, "anything"), 0);
  EXPECT_EQ(m.getPointField_uint32(0, "anything"), 0u);

  // Setters for the built-in fields:
  m.setPointField_float(1, "x", 40.0f);
  m.setPointField_float(1, "y", 50.0f);
  m.setPointField_float(1, "z", 60.0f);
  EXPECT_NEAR(m.getPointField_float(1, "z"), 60.0f, 1e-6f);
  // Unknown names are silently ignored:
  EXPECT_NO_THROW(m.setPointField_float(1, "intensity", 1.0f));
  EXPECT_NO_THROW(m.setPointField_double(1, "anything", 1.0));
  EXPECT_NO_THROW(m.setPointField_uint16(1, "anything", 1));
  EXPECT_NO_THROW(m.setPointField_uint8(1, "anything", 1));
  EXPECT_NO_THROW(m.setPointField_uint32(1, "anything", 1));

  // Out-of-bounds indices are rejected:
  EXPECT_THROW(m.getPointField_float(10, "x"), std::exception);
  EXPECT_THROW(m.setPointField_float(10, "x", 0.0f), std::exception);
}

TEST(CPointsMapFields, bufferRefsByFieldName)
{
  CSimplePointsMap m;
  m.insertPoint(1.0f, 2.0f, 3.0f);

  const CSimplePointsMap& cm = m;

  ASSERT_NE(cm.getPointsBufferRef_float_field("x"), nullptr);
  ASSERT_NE(cm.getPointsBufferRef_float_field("y"), nullptr);
  ASSERT_NE(cm.getPointsBufferRef_float_field("z"), nullptr);
  EXPECT_EQ(cm.getPointsBufferRef_float_field("intensity"), nullptr);
  EXPECT_EQ(cm.getPointsBufferRef_double_field("anything"), nullptr);
  EXPECT_EQ(cm.getPointsBufferRef_uint16_field("anything"), nullptr);
  EXPECT_EQ(cm.getPointsBufferRef_uint8_field("anything"), nullptr);
  EXPECT_EQ(cm.getPointsBufferRef_uint32_field("anything"), nullptr);

  // Non-const overloads allow modifying the buffer in place:
  auto* xs = m.getPointsBufferRef_float_field("x");
  ASSERT_NE(xs, nullptr);
  (*xs)[0] = 11.0f;
  EXPECT_NEAR(m.getPointField_float(0, "x"), 11.0f, 1e-6f);
  EXPECT_NE(m.getPointsBufferRef_float_field("y"), nullptr);
  EXPECT_NE(m.getPointsBufferRef_float_field("z"), nullptr);
  EXPECT_EQ(m.getPointsBufferRef_float_field("intensity"), nullptr);
  EXPECT_EQ(m.getPointsBufferRef_double_field("anything"), nullptr);
  EXPECT_EQ(m.getPointsBufferRef_uint16_field("anything"), nullptr);
  EXPECT_EQ(m.getPointsBufferRef_uint8_field("anything"), nullptr);
  EXPECT_EQ(m.getPointsBufferRef_uint32_field("anything"), nullptr);
}

namespace
{
CVoxelMap makeVoxelMap()
{
  CVoxelMap m(0.25);
  // A few occupied voxels and a few free ones:
  // Repeat the updates so the log-odds get well past both thresholds:
  for (int rep = 0; rep < 10; rep++)
  {
    for (int i = 0; i < 3; i++) m.updateVoxel(i * 0.25, 0, i * 0.25, true);
    for (int i = 3; i < 6; i++) m.updateVoxel(i * 0.25, 0, i * 0.25, false);
  }
  return m;
}
}  // namespace

TEST(CVoxelMapViz, getAsOctoMapVoxelsAllColoringModes)
{
  auto m = makeVoxelMap();

  using mrpt::viz::COctoMapVoxels;
  const COctoMapVoxels::visualization_mode_t modes[] = {
      COctoMapVoxels::FIXED, COctoMapVoxels::COLOR_FROM_HEIGHT,
      COctoMapVoxels::COLOR_FROM_OCCUPANCY, COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY,
      COctoMapVoxels::TRANS_AND_COLOR_FROM_OCCUPANCY};

  for (const auto mode : modes)
  {
    COctoMapVoxels vx;
    vx.setVisualizationMode(mode);
    m.getAsOctoMapVoxels(vx);
    EXPECT_GT(vx.getVoxelCount(mrpt::viz::VOXEL_SET_OCCUPIED), 0U)
        << "mode " << static_cast<int>(mode);
  }

  // Unsupported schemes for this map class:
  {
    COctoMapVoxels vx;
    vx.setVisualizationMode(COctoMapVoxels::MIXED);
    EXPECT_THROW(m.getAsOctoMapVoxels(vx), std::exception);
  }
  {
    // A voxel type without colour cannot be colored from RGB data:
    COctoMapVoxels vx;
    vx.setVisualizationMode(COctoMapVoxels::COLOR_FROM_RGB_DATA);
    EXPECT_THROW(m.getAsOctoMapVoxels(vx), std::exception);
  }
}

TEST(CVoxelMapViz, renderingOptionsFilterVoxelSets)
{
  auto m = makeVoxelMap();

  using mrpt::viz::COctoMapVoxels;

  // Only free voxels:
  {
    COctoMapVoxels vx;
    vx.setVisualizationMode(COctoMapVoxels::FIXED);
    m.renderingOptions.generateOccupiedVoxels = false;
    m.renderingOptions.generateFreeVoxels = true;
    m.getAsOctoMapVoxels(vx);
    EXPECT_EQ(vx.getVoxelCount(mrpt::viz::VOXEL_SET_OCCUPIED), 0U);
    EXPECT_GT(vx.getVoxelCount(mrpt::viz::VOXEL_SET_FREESPACE), 0U);
  }
  // Only occupied voxels:
  {
    COctoMapVoxels vx;
    vx.setVisualizationMode(COctoMapVoxels::FIXED);
    m.renderingOptions.generateOccupiedVoxels = true;
    m.renderingOptions.generateFreeVoxels = false;
    m.getAsOctoMapVoxels(vx);
    EXPECT_GT(vx.getVoxelCount(mrpt::viz::VOXEL_SET_OCCUPIED), 0U);
    EXPECT_EQ(vx.getVoxelCount(mrpt::viz::VOXEL_SET_FREESPACE), 0U);
  }
}

TEST(CObservationPointCloudDescription, getDescriptionAsText)
{
  auto obs = CObservationPointCloud::Create();
  obs->timestamp = mrpt::Clock::now();
  obs->sensorLabel = "lidar";
  obs->sensorPose = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
  obs->pointcloud = CSimplePointsMap::Create();
  for (int i = 0; i < 10; i++)
  {
    const auto fi = static_cast<float>(i);
    obs->pointcloud->insertPoint(fi * 1.0f, fi * 2.0f, fi * 0.5f);
  }

  std::ostringstream ss;
  obs->getDescriptionAsText(ss);
  const std::string s = ss.str();
  EXPECT_NE(s.find("lidar"), std::string::npos);
  EXPECT_FALSE(s.empty());

  // The same, on an observation with no point cloud at all:
  auto empty = CObservationPointCloud::Create();
  empty->timestamp = mrpt::Clock::now();
  std::ostringstream ss2;
  EXPECT_NO_THROW(empty->getDescriptionAsText(ss2));
  EXPECT_FALSE(ss2.str().empty());

  // ...and on one whose coordinates include NaNs, which the min/max helper
  // must skip:
  auto withNaN = CObservationPointCloud::Create();
  withNaN->timestamp = mrpt::Clock::now();
  withNaN->pointcloud = CSimplePointsMap::Create();
  withNaN->pointcloud->insertPoint(1.0f, 1.0f, 1.0f);
  withNaN->pointcloud->insertPoint(std::numeric_limits<float>::quiet_NaN(), 2.0f, 2.0f);
  std::ostringstream ss3;
  EXPECT_NO_THROW(withNaN->getDescriptionAsText(ss3));
  EXPECT_FALSE(ss3.str().empty());
}
