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
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>

#include <fstream>

using namespace mrpt::viz;

namespace
{
std::string tempPlyFile(const std::string& suffix)
{
  return mrpt::system::getTempFileName() + suffix + ".ply";
}
}  // namespace

TEST(PLY_import_export, PointCloudRoundTripAscii)
{
  CPointCloud pc;
  for (int i = 0; i < 20; i++)
  {
    pc.insertPoint(
        static_cast<float>(i), static_cast<float>(i) * 0.5f, static_cast<float>(i) * -0.25f);
  }

  const std::string file = tempPlyFile("_pc_ascii");
  const bool saveOk = pc.saveToPlyFile(file, false /*ascii*/);
  ASSERT_TRUE(saveOk) << pc.getSavePLYErrorString();

  CPointCloud pc2;
  const bool loadOk = pc2.loadFromPlyFile(file);
  ASSERT_TRUE(loadOk) << pc2.getLoadPLYErrorString();

  ASSERT_EQ(pc2.size(), pc.size());
  for (size_t i = 0; i < pc.size(); i++)
  {
    const auto& p1 = pc.getPoint3Df(i);
    const auto& p2 = pc2.getPoint3Df(i);
    EXPECT_NEAR(p1.x, p2.x, 1e-4f);
    EXPECT_NEAR(p1.y, p2.y, 1e-4f);
    EXPECT_NEAR(p1.z, p2.z, 1e-4f);
  }

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, PointCloudRoundTripBinary)
{
  CPointCloud pc;
  for (int i = 0; i < 20; i++)
  {
    pc.insertPoint(
        static_cast<float>(i) * 0.1f, static_cast<float>(-i), static_cast<float>(i) * 2.0f);
  }

  const std::string file = tempPlyFile("_pc_bin");
  const bool saveOk = pc.saveToPlyFile(file, true /*binary*/);
  ASSERT_TRUE(saveOk) << pc.getSavePLYErrorString();

  CPointCloud pc2;
  const bool loadOk = pc2.loadFromPlyFile(file);
  ASSERT_TRUE(loadOk) << pc2.getLoadPLYErrorString();

  ASSERT_EQ(pc2.size(), pc.size());
  for (size_t i = 0; i < pc.size(); i++)
  {
    const auto& p1 = pc.getPoint3Df(i);
    const auto& p2 = pc2.getPoint3Df(i);
    EXPECT_NEAR(p1.x, p2.x, 1e-4f);
    EXPECT_NEAR(p1.y, p2.y, 1e-4f);
    EXPECT_NEAR(p1.z, p2.z, 1e-4f);
  }

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ColouredPointCloudRoundTrip)
{
  CPointCloudColoured pc;
  for (int i = 0; i < 15; i++)
  {
    mrpt::math::TPointXYZfRGBAu8 p(
        static_cast<float>(i), static_cast<float>(i) * -1.0f, 0.5f,
        static_cast<uint8_t>(i * 10 % 256), static_cast<uint8_t>(255 - i * 5), 128, 255);
    pc.insertPoint(p);
  }

  const std::string file = tempPlyFile("_pcc_ascii");
  const bool saveOk = pc.saveToPlyFile(file, false /*ascii*/);
  ASSERT_TRUE(saveOk) << pc.getSavePLYErrorString();

  // Saving must not disturb the cloud being saved:
  ASSERT_EQ(pc.size(), 15u);
  EXPECT_NEAR(pc.getPoint3Df(3).x, 3.0f, 1e-6f);

  CPointCloudColoured pc2;
  const bool loadOk = pc2.loadFromPlyFile(file);
  ASSERT_TRUE(loadOk) << pc2.getLoadPLYErrorString();

  ASSERT_EQ(pc2.size(), pc.size());
  for (size_t i = 0; i < pc.size(); i++)
  {
    const auto& p1 = pc.getPoint3Df(i);
    const auto& p2 = pc2.getPoint3Df(i);
    EXPECT_NEAR(p1.x, p2.x, 1e-4f);
    EXPECT_NEAR(p1.y, p2.y, 1e-4f);
    EXPECT_NEAR(p1.z, p2.z, 1e-4f);

    // ...and the per-channel color must survive the round trip too:
    const auto c1 = pc.getPointColor(i);
    const auto c2 = pc2.getPointColor(i);
    EXPECT_NEAR(c1.R, c2.R, 1);
    EXPECT_NEAR(c1.G, c2.G, 1);
    EXPECT_NEAR(c1.B, c2.B, 1);
  }

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ColouredPointCloudRoundTripBinary)
{
  CPointCloudColoured pc;
  pc.push_back(1.0f, 2.0f, 3.0f, 1.0f, 0.0f, 0.0f);
  pc.push_back(-1.0f, 0.5f, 0.0f, 0.0f, 1.0f, 0.5f);

  const std::string file = tempPlyFile("_pcc_bin");
  ASSERT_TRUE(pc.saveToPlyFile(file, true /*binary*/)) << pc.getSavePLYErrorString();

  CPointCloudColoured pc2;
  ASSERT_TRUE(pc2.loadFromPlyFile(file)) << pc2.getLoadPLYErrorString();

  ASSERT_EQ(pc2.size(), 2u);
  EXPECT_NEAR(pc2.getPoint3Df(0).x, 1.0f, 1e-4f);
  const auto c = pc2.getPointColor(0);
  EXPECT_EQ(c.R, 255);
  EXPECT_EQ(c.G, 0);
  EXPECT_EQ(c.B, 0);

  mrpt::system::deleteFile(file);
}

namespace
{
void writeTextFile(const std::string& file, const std::string& contents)
{
  std::ofstream f(file);
  f << contents;
}
}  // namespace

TEST(PLY_import_export, ReadExternalUcharColors)
{
  // The layout written by most other tools: uchar red/green/blue, no intensity.
  const std::string file = tempPlyFile("_ext_uchar");
  writeTextFile(
      file,
      "ply\n"
      "format ascii 1.0\n"
      "comment made elsewhere\n"
      "element vertex 2\n"
      "property float x\n"
      "property float y\n"
      "property float z\n"
      "property uchar red\n"
      "property uchar green\n"
      "property uchar blue\n"
      "end_header\n"
      "0 0 0 255 0 0\n"
      "1 2 3 0 128 255\n");

  CPointCloudColoured pc;
  std::vector<std::string> comments;
  ASSERT_TRUE(pc.loadFromPlyFile(file, &comments)) << pc.getLoadPLYErrorString();
  ASSERT_EQ(pc.size(), 2u);
  EXPECT_NEAR(pc.getPoint3Df(1).z, 3.0f, 1e-5f);

  const auto c0 = pc.getPointColor(0);
  EXPECT_EQ(c0.R, 255);
  EXPECT_EQ(c0.G, 0);
  const auto c1 = pc.getPointColor(1);
  EXPECT_NEAR(c1.G, 128, 1);
  EXPECT_EQ(c1.B, 255);

  ASSERT_EQ(comments.size(), 1u);
  EXPECT_EQ(comments[0], "made elsewhere");

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ReadExternalFloatColorsAndDoubleCoords)
{
  // Float color channels are already in [0,1] and must not be rescaled;
  // "double" coordinates exercise a different type-conversion path.
  const std::string file = tempPlyFile("_ext_float");
  writeTextFile(
      file,
      "ply\n"
      "format ascii 1.0\n"
      "element vertex 1\n"
      "property double x\n"
      "property double y\n"
      "property double z\n"
      "property float red\n"
      "property float green\n"
      "property float blue\n"
      "end_header\n"
      "1.5 -2.5 0.25 1.0 0.5 0.0\n");

  CPointCloudColoured pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  ASSERT_EQ(pc.size(), 1u);
  EXPECT_NEAR(pc.getPoint3Df(0).x, 1.5f, 1e-5f);
  EXPECT_NEAR(pc.getPoint3Df(0).y, -2.5f, 1e-5f);

  const auto c = pc.getPointColor(0);
  EXPECT_EQ(c.R, 255);
  EXPECT_NEAR(c.G, 127, 2);
  EXPECT_EQ(c.B, 0);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ReadFileWithFacesIgnoresThem)
{
  // A mesh file: the point-cloud importers keep the vertices and skip faces.
  const std::string file = tempPlyFile("_faces");
  writeTextFile(
      file,
      "ply\n"
      "format ascii 1.0\n"
      "element vertex 3\n"
      "property float x\n"
      "property float y\n"
      "property float z\n"
      "element face 1\n"
      "property list uchar int vertex_indices\n"
      "end_header\n"
      "0 0 0\n"
      "1 0 0\n"
      "0 1 0\n"
      "3 0 1 2\n");

  CPointCloud pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  EXPECT_EQ(pc.size(), 3u);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, MalformedHeaderFails)
{
  const std::string file = tempPlyFile("_malformed");
  writeTextFile(file, "not a ply file at all\n");

  CPointCloud pc;
  EXPECT_FALSE(pc.loadFromPlyFile(file));
  EXPECT_FALSE(pc.getLoadPLYErrorString().empty());

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, SaveWithCommentsAndObjInfo)
{
  CPointCloud pc;
  pc.insertPoint(1.0f, 2.0f, 3.0f);

  const std::string file = tempPlyFile("_comments");
  std::vector<std::string> comments{"generated by an mrpt_viz unit test"};
  std::vector<std::string> objInfo{"unit test object"};
  const bool saveOk = pc.saveToPlyFile(file, false, comments, objInfo);
  ASSERT_TRUE(saveOk);

  CPointCloud pc2;
  std::vector<std::string> readComments;
  std::vector<std::string> readObjInfo;
  const bool loadOk = pc2.loadFromPlyFile(file, &readComments, &readObjInfo);
  ASSERT_TRUE(loadOk);
  EXPECT_EQ(pc2.size(), 1u);

  ASSERT_EQ(readComments.size(), 1u);
  EXPECT_EQ(readComments[0], comments[0]);
  ASSERT_EQ(readObjInfo.size(), 1u);
  EXPECT_EQ(readObjInfo[0], objInfo[0]);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, LoadNonExistentFileFails)
{
  CPointCloud pc;
  const bool loadOk = pc.loadFromPlyFile("/nonexistent/path/does_not_exist.ply");
  EXPECT_FALSE(loadOk);
  EXPECT_FALSE(pc.getLoadPLYErrorString().empty());
}

TEST(PLY_import_export, EmptyCloudRoundTrip)
{
  CPointCloud pc;
  const std::string file = tempPlyFile("_empty");
  const bool saveOk = pc.saveToPlyFile(file, false);
  ASSERT_TRUE(saveOk);

  CPointCloud pc2;
  const bool loadOk = pc2.loadFromPlyFile(file);
  ASSERT_TRUE(loadOk);
  EXPECT_EQ(pc2.size(), 0u);

  mrpt::system::deleteFile(file);
}
