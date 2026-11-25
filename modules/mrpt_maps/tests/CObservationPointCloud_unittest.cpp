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
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/serialization/CArchive.h>

#include <filesystem>
#include <limits>
#include <sstream>

using mrpt::obs::CObservationPointCloud;

namespace
{
mrpt::maps::CSimplePointsMap::Ptr makeSamplePoints()
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  pts->insertPoint(1.0f, 2.0f, 3.0f);
  pts->insertPoint(-1.0f, -2.0f, -3.0f);
  return pts;
}

std::string makeTempFilePath(const std::string& suffix)
{
  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path();
  return (dir /
          ("mrpt_CObservationPointCloud_unittest_" + std::to_string(static_cast<long>(getpid())) +
           "_" + std::to_string(counter++) + suffix))
      .string();
}
}  // namespace

TEST(CObservationPointCloud, ConstructFrom3DRangeScan)
{
  mrpt::obs::CObservation3DRangeScan scan3d;
  // An empty range scan should still yield a valid (possibly empty)
  // pointcloud, not a crash:
  CObservationPointCloud obs(scan3d);
  EXPECT_NE(obs.pointcloud, nullptr);
}

TEST(CObservationPointCloud, SensorPoseAccessors)
{
  CObservationPointCloud obs;
  const mrpt::poses::CPose3D p(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  obs.setSensorPose(p);
  EXPECT_NEAR(obs.getSensorPose().distanceTo(p), 0.0, 1e-9);
}

TEST(CObservationPointCloud, GetDescriptionAsTextNullPointcloud)
{
  CObservationPointCloud obs;
  obs.pointcloud.reset();
  std::stringstream ss;
  EXPECT_NO_THROW(obs.getDescriptionAsText(ss));
  EXPECT_NE(ss.str().find("nullptr"), std::string::npos);
}

TEST(CObservationPointCloud, GetDescriptionAsTextWithPoints)
{
  CObservationPointCloud obs;
  obs.pointcloud = makeSamplePoints();
  std::stringstream ss;
  EXPECT_NO_THROW(obs.getDescriptionAsText(ss));
  EXPECT_FALSE(ss.str().empty());
}

// The ranges of every field type are listed; a field without valid (non-NaN)
// values must not be dereferenced:
TEST(CObservationPointCloud, GetDescriptionAsTextOfAllFieldTypes)
{
  auto pts = mrpt::maps::CGenericPointsMap::Create();
  pts->registerField_float("intensity");
  pts->registerField_float("all_nan");
  pts->registerField_double("t");
  pts->registerField_uint16("ring");
  pts->registerField_uint8("label");
  pts->registerField_uint32("id");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  constexpr int N = 8;  // a power of two, so the field buffers are full
  for (int i = 0; i < N; i++)
  {
    pts->insertPoint(static_cast<float>(i), 0, 0);
    pts->insertPointField_float("intensity", i == 1 ? nan : static_cast<float>(i) * 0.5f);
    pts->insertPointField_float("all_nan", nan);
    pts->insertPointField_double("t", 10.0 + i);
    pts->insertPointField_uint16("ring", static_cast<uint16_t>(7 + i));
    pts->insertPointField_uint8("label", static_cast<uint8_t>(i));
    pts->insertPointField_uint32("id", static_cast<uint32_t>(100 + i));
  }

  CObservationPointCloud obs;
  obs.pointcloud = pts;
  std::stringstream ss;
  obs.getDescriptionAsText(ss);
  const std::string s = ss.str();
  EXPECT_NE(s.find("Number of points: 8"), std::string::npos) << s;
  EXPECT_NE(s.find("range: [0.00, 3.50]"), std::string::npos) << s;  // intensity, NaN ignored
  EXPECT_NE(s.find("range: [10.00, 17.00]"), std::string::npos) << s;
  EXPECT_NE(s.find("range: [7, 14]"), std::string::npos) << s;
  EXPECT_NE(s.find("range: [0, 7]"), std::string::npos) << s;
  EXPECT_NE(s.find("range: [100, 107]"), std::string::npos) << s;
  EXPECT_NE(s.find("all NaN"), std::string::npos) << s;
}

TEST(CObservationPointCloud, SerializeRoundTrip)
{
  CObservationPointCloud src;
  src.pointcloud = makeSamplePoints();
  src.sensorPose = mrpt::poses::CPose3D(1.0, 2.0, 3.0, 0, 0, 0);
  src.sensorLabel = "TEST_PC";

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  mrpt::serialization::CSerializable::Ptr obj;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> obj;
  }
  ASSERT_NE(obj, nullptr);
  auto* dst = dynamic_cast<CObservationPointCloud*>(obj.get());
  ASSERT_NE(dst, nullptr);

  ASSERT_NE(dst->pointcloud, nullptr);
  EXPECT_EQ(dst->pointcloud->size(), src.pointcloud->size());
  EXPECT_EQ(dst->sensorLabel, "TEST_PC");
  EXPECT_NEAR(dst->sensorPose.distanceTo(src.sensorPose), 0.0, 1e-9);
}

TEST(CObservationPointCloud, ExternalStoragePlainTextRoundTrip)
{
  const std::string file = makeTempFilePath(".txt");

  CObservationPointCloud obs;
  obs.pointcloud = makeSamplePoints();
  EXPECT_FALSE(obs.isExternallyStored());

  obs.setAsExternalStorage(file, CObservationPointCloud::ExternalStorageFormat::PlainTextFile);
  EXPECT_TRUE(obs.isExternallyStored());
  EXPECT_EQ(obs.getExternalStorageFile(), file);

  // unload() must write the in-memory pointcloud to the external file and
  // free the in-memory copy:
  obs.unload();
  EXPECT_TRUE(std::filesystem::exists(file));

  // load_impl() (invoked indirectly via any accessor that needs the data)
  // must bring the points back:
  obs.load_impl();
  ASSERT_NE(obs.pointcloud, nullptr);
  EXPECT_EQ(obs.pointcloud->size(), 2u);

  std::filesystem::remove(file);
}

TEST(CObservationPointCloud, ExternalStorageMrptSerializationRoundTrip)
{
  const std::string file = makeTempFilePath(".bin.gz");

  CObservationPointCloud obs;
  obs.pointcloud = makeSamplePoints();
  obs.setAsExternalStorage(file, CObservationPointCloud::ExternalStorageFormat::MRPT_Serialization);

  obs.unload();
  EXPECT_TRUE(std::filesystem::exists(file));

  obs.load_impl();
  ASSERT_NE(obs.pointcloud, nullptr);
  EXPECT_EQ(obs.pointcloud->size(), 2u);

  std::filesystem::remove(file);
}

TEST(CObservationPointCloud, ExternalStorageKittiUnloadThrows)
{
  const std::string file = makeTempFilePath("_nonexistent.bin");

  CObservationPointCloud obs;
  obs.pointcloud = makeSamplePoints();
  obs.setAsExternalStorage(file, CObservationPointCloud::ExternalStorageFormat::KittiBinFile);

  // Saving to Kitti format is documented as unsupported: unload() must
  // throw rather than silently drop the data, since the file doesn't
  // already exist on disk.
  EXPECT_THROW(obs.unload(), std::exception);
}

TEST(CObservationPointCloud, SetAsExternalStorageTwiceThrows)
{
  CObservationPointCloud obs;
  obs.setAsExternalStorage("a.bin", CObservationPointCloud::ExternalStorageFormat::PlainTextFile);
  EXPECT_THROW(
      obs.setAsExternalStorage(
          "b.bin", CObservationPointCloud::ExternalStorageFormat::PlainTextFile),
      std::exception);
}

TEST(CObservationPointCloud, UnloadNoOpWhenNotExternallyStored)
{
  CObservationPointCloud obs;
  obs.pointcloud = makeSamplePoints();
  // Not externally stored: unload() must be a no-op, keeping the points.
  EXPECT_NO_THROW(obs.unload());
  ASSERT_NE(obs.pointcloud, nullptr);
  EXPECT_EQ(obs.pointcloud->size(), 2u);
}

TEST(CObservationPointCloud, LoadImplNoOpWhenNotExternallyStored)
{
  CObservationPointCloud obs;
  obs.pointcloud = makeSamplePoints();
  EXPECT_NO_THROW(obs.load_impl());
  ASSERT_NE(obs.pointcloud, nullptr);
  EXPECT_EQ(obs.pointcloud->size(), 2u);
}
