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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

#ifndef _WIN32
#include <sys/resource.h>
#endif

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

TEST(PLY_import_export, ReadMixedTypeColorChannels)
{
  // Nothing stops a file from declaring one channel as uchar [0,255] and
  // another as float [0,1]; each needs its own scale.
  const std::string file = tempPlyFile("_ext_mixed");
  writeTextFile(
      file,
      "ply\n"
      "format ascii 1.0\n"
      "element vertex 1\n"
      "property float x\n"
      "property float y\n"
      "property float z\n"
      "property uchar red\n"
      "property float green\n"
      "property uchar blue\n"
      "end_header\n"
      "0 0 0 255 1.0 0\n");

  CPointCloudColoured pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  ASSERT_EQ(pc.size(), 1u);

  const auto c = pc.getPointColor(0);
  EXPECT_EQ(c.R, 255);
  EXPECT_EQ(c.G, 255);
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

namespace
{
/** Little helper to build the raw contents of a binary PLY file. */
class BinaryBuilder
{
 public:
  explicit BinaryBuilder(bool bigEndian) : m_bigEndian(bigEndian) {}

  template <typename T>
  BinaryBuilder& put(T v)
  {
    char buf[sizeof(T)];
    std::memcpy(buf, &v, sizeof(T));
    // The bytes are in the host order: reverse them if the file has the other
    const uint16_t one = 1;
    uint8_t firstByte;
    std::memcpy(&firstByte, &one, 1);
    const bool hostIsLittleEndian = firstByte == 1;
    if (m_bigEndian == hostIsLittleEndian)
    {
      std::reverse(buf, buf + sizeof(T));
    }
    m_data.append(buf, sizeof(T));
    return *this;
  }
  const std::string& str() const { return m_data; }

 private:
  bool m_bigEndian;
  std::string m_data;
};

void writeBinaryFile(const std::string& file, const std::string& header, const std::string& data)
{
  std::ofstream f(file, std::ios::binary);
  f << header;
  f.write(data.data(), static_cast<std::streamsize>(data.size()));
}
}  // namespace

TEST(PLY_import_export, ReadBinaryBigEndian)
{
  const std::string file = tempPlyFile("_be");
  BinaryBuilder b(true);
  b.put<float>(1.5f).put<float>(-2.0f).put<float>(3.25f).put<float>(0.5f);
  b.put<float>(4.0f).put<float>(5.0f).put<float>(6.0f).put<float>(1.0f);
  writeBinaryFile(
      file,
      "ply\nformat binary_big_endian 1.0\nelement vertex 2\n"
      "property float x\nproperty float y\nproperty float z\n"
      "property float intensity\nend_header\n",
      b.str());

  CPointCloudColoured pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  ASSERT_EQ(pc.size(), 2u);
  EXPECT_NEAR(pc.getPoint3Df(0).x, 1.5f, 1e-6f);
  EXPECT_NEAR(pc.getPoint3Df(0).y, -2.0f, 1e-6f);
  EXPECT_NEAR(pc.getPoint3Df(1).z, 6.0f, 1e-6f);
  // Grayscale from the intensity channel:
  EXPECT_NEAR(pc.getPointColor(0).R, 128, 1);
  EXPECT_EQ(pc.getPointColor(1).B, 255);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ReadBinaryLittleEndianScalarTypesAndExtraProps)
{
  // Each coordinate uses a different on-disk type, and there are properties
  // the importer does not know about, which must be skipped in place.
  const std::string file = tempPlyFile("_le_types");
  BinaryBuilder b(false);
  for (int i = 0; i < 2; i++)
  {
    b.put<int16_t>(static_cast<int16_t>(-3 + i));       // x: short
    b.put<uint16_t>(static_cast<uint16_t>(40000 + i));  // y: ushort
    b.put<int32_t>(-100000 + i);                        // z: int
    b.put<uint32_t>(3000000000u);                       // extra: uint
    b.put<int8_t>(-5);                                  // extra: char
    b.put<uint8_t>(200);                                // extra: uchar
    b.put<double>(0.25);                                // extra: double
    b.put<double>(1234.5 + i);                          // timestamp: double
  }
  writeBinaryFile(
      file,
      "ply\nformat binary_little_endian 1.0\nelement vertex 2\n"
      "property short x\nproperty ushort y\nproperty int z\n"
      "property uint extra_uint\nproperty char extra_char\n"
      "property uchar extra_uchar\nproperty double extra_double\n"
      "property double timestamp\nend_header\n",
      b.str());

  CPointCloud pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  ASSERT_EQ(pc.size(), 2u);
  EXPECT_NEAR(pc.getPoint3Df(0).x, -3.0f, 1e-6f);
  EXPECT_NEAR(pc.getPoint3Df(1).x, -2.0f, 1e-6f);
  EXPECT_NEAR(pc.getPoint3Df(0).y, 40000.0f, 1e-3f);
  EXPECT_NEAR(pc.getPoint3Df(0).z, -100000.0f, 1e-1f);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ReadBinaryWithFaceListElement)
{
  // A binary mesh: the vertex element is read, the face list (variable
  // length records) after it is never reached but must not break the header.
  const std::string file = tempPlyFile("_bin_faces");
  BinaryBuilder b(false);
  b.put<float>(0.f).put<float>(0.f).put<float>(0.f);
  b.put<float>(1.f).put<float>(0.f).put<float>(0.f);
  b.put<float>(0.f).put<float>(1.f).put<float>(0.f);
  b.put<uint8_t>(3).put<int32_t>(0).put<int32_t>(1).put<int32_t>(2);
  writeBinaryFile(
      file,
      "ply\nformat binary_little_endian 1.0\nelement vertex 3\n"
      "property float x\nproperty float y\nproperty float z\n"
      "element face 1\nproperty list uchar int vertex_indices\nend_header\n",
      b.str());

  CPointCloud pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  EXPECT_EQ(pc.size(), 3u);
  EXPECT_NEAR(pc.getPoint3Df(2).y, 1.0f, 1e-6f);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ReadListPropertyInsideVertexElement)
{
  // A list-valued property in the vertex element itself has to be skipped,
  // both in ASCII and in binary files.
  {
    const std::string file = tempPlyFile("_list_ascii");
    writeTextFile(
        file,
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property list uchar float extra\nend_header\n"
        "1 2 3 2 0.5 0.25\n"
        "4 5 6 0\n");
    CPointCloud pc;
    ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
    ASSERT_EQ(pc.size(), 2u);
    EXPECT_NEAR(pc.getPoint3Df(1).z, 6.0f, 1e-6f);
    mrpt::system::deleteFile(file);
  }
  {
    const std::string file = tempPlyFile("_list_bin");
    BinaryBuilder b(false);
    b.put<float>(1.f).put<float>(2.f).put<float>(3.f);
    b.put<uint8_t>(2).put<float>(0.5f).put<float>(0.25f);
    b.put<float>(4.f).put<float>(5.f).put<float>(6.f);
    b.put<uint8_t>(0);
    writeBinaryFile(
        file,
        "ply\nformat binary_little_endian 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property list uchar float extra\nend_header\n",
        b.str());
    CPointCloud pc;
    ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
    ASSERT_EQ(pc.size(), 2u);
    EXPECT_NEAR(pc.getPoint3Df(1).z, 6.0f, 1e-6f);
    mrpt::system::deleteFile(file);
  }
}

TEST(PLY_import_export, ReadTruncatedBinaryFails)
{
  const std::string file = tempPlyFile("_truncated");
  BinaryBuilder b(false);
  b.put<float>(1.f).put<float>(2.f).put<float>(3.f);  // only 1 of 5 vertices
  writeBinaryFile(
      file,
      "ply\nformat binary_little_endian 1.0\nelement vertex 5\n"
      "property float x\nproperty float y\nproperty float z\nend_header\n",
      b.str());

  CPointCloud pc;
  EXPECT_FALSE(pc.loadFromPlyFile(file));
  EXPECT_FALSE(pc.getLoadPLYErrorString().empty());

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, ReadHeaderErrors)
{
  // Malformed headers must be reported as errors, never crash.
  const std::vector<std::pair<std::string, std::string>> cases = {
      {         "unknown_format","ply\nformat martian 1.0\nelement vertex 0\nend_header\n"                                 },
      {           "unknown_type",
       "ply\nformat ascii 1.0\nelement vertex 1\nproperty quaternion x\nend_header\n1\n"   },
      {"property_before_element",   "ply\nformat ascii 1.0\nproperty float x\nend_header\n"},
      {         "short_property",
       "ply\nformat ascii 1.0\nelement vertex 1\nproperty float\nend_header\n1\n"          },
      {    "short_list_property",
       "ply\nformat ascii 1.0\nelement vertex 1\nproperty list uchar\nend_header\n1\n"     },
      {          "short_element",     "ply\nformat ascii 1.0\nelement vertex\nend_header\n"},
      {           "short_format",       "ply\nformat ascii\nelement vertex 0\nend_header\n"},
  };
  for (const auto& [name, contents] : cases)
  {
    const std::string file = tempPlyFile("_hdr_" + name);
    writeTextFile(file, contents);
    CPointCloud pc;
    EXPECT_FALSE(pc.loadFromPlyFile(file)) << name;
    EXPECT_FALSE(pc.getLoadPLYErrorString().empty()) << name;
    mrpt::system::deleteFile(file);
  }
}

TEST(PLY_import_export, NonFiniteAndHugeFloatValuesAreLoaded)
{
  // NaN, infinity and values beyond the integer range must not be a problem
  // (converting them to integers is undefined behavior):
  const std::string file = tempPlyFile("_nonfinite");
  BinaryBuilder b(false);
  b.put<float>(std::numeric_limits<float>::quiet_NaN());
  b.put<float>(std::numeric_limits<float>::infinity());
  b.put<float>(1e30f);
  b.put<double>(-1e300);  // an extra double property, which is skipped
  b.put<float>(1.0f).put<float>(2.0f).put<float>(3.0f);
  b.put<double>(std::numeric_limits<double>::quiet_NaN());
  writeBinaryFile(
      file,
      "ply\nformat binary_little_endian 1.0\nelement vertex 2\n"
      "property float x\nproperty float y\nproperty float z\n"
      "property double extra\nend_header\n",
      b.str());

  CPointCloud pc;
  ASSERT_TRUE(pc.loadFromPlyFile(file)) << pc.getLoadPLYErrorString();
  ASSERT_EQ(pc.size(), 2u);
  EXPECT_TRUE(std::isnan(pc.getPoint3Df(0).x));
  EXPECT_TRUE(std::isinf(pc.getPoint3Df(0).y));
  EXPECT_NEAR(pc.getPoint3Df(1).z, 3.0f, 1e-6f);

  mrpt::system::deleteFile(file);
}

TEST(PLY_import_export, FailedLoadsDoNotLeakFileDescriptors)
{
  const std::string bad1 = tempPlyFile("_leak_badheader");
  const std::string bad2 = tempPlyFile("_leak_truncated");
  const std::string good = tempPlyFile("_leak_good");
  writeTextFile(bad1, "ply\nformat ascii 1.0\nelement vertex 1\nproperty float\nend_header\n1\n");
  {
    BinaryBuilder b(false);
    b.put<float>(1.f);
    writeBinaryFile(
        bad2,
        "ply\nformat binary_little_endian 1.0\nelement vertex 5\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n",
        b.str());
  }
  writeTextFile(
      good,
      "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\n"
      "property float z\nend_header\n1 2 3\n");

  // More failures than descriptors available: if each one leaked a file, the
  // last loads would fail.
#ifndef _WIN32
  rlimit oldLimit{};
  ASSERT_EQ(getrlimit(RLIMIT_NOFILE, &oldLimit), 0);
  rlimit newLimit = oldLimit;
  newLimit.rlim_cur = std::min<rlim_t>(oldLimit.rlim_cur, 256);
  ASSERT_EQ(setrlimit(RLIMIT_NOFILE, &newLimit), 0);
#endif
  for (int i = 0; i < 1000; i++)
  {
    CPointCloud pc;
    EXPECT_FALSE(pc.loadFromPlyFile(i % 2 ? bad1 : bad2));
  }
  CPointCloud pc;
  EXPECT_TRUE(pc.loadFromPlyFile(good)) << pc.getLoadPLYErrorString();
#ifndef _WIN32
  setrlimit(RLIMIT_NOFILE, &oldLimit);
#endif

  mrpt::system::deleteFile(bad1);
  mrpt::system::deleteFile(bad2);
  mrpt::system::deleteFile(good);
}
