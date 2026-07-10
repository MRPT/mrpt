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
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/VelodyneCalibration.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <cstring>
#include <fstream>
#include <sstream>

using namespace mrpt::obs;

namespace
{
// Mirrors the exact packed binary layout of
// CObservationVelodyneScan::TVelodyneRawPacket, but with public members, so
// tests can build synthetic packets and memcpy() them into place (the real
// struct only exposes accessor getters, matching how this class is meant to
// be filled in from a raw byte buffer coming from the network/a pcap file).
#pragma pack(push, 1)
struct RawLaserReturn
{
  uint16_t distance{0};
  uint8_t intensity{0};
};
struct RawBlock
{
  uint16_t header{0};
  uint16_t rotation{0};
  RawLaserReturn returns[32];
};
struct RawPacket
{
  RawBlock blocks[12];
  uint32_t gps_timestamp{0};
  uint8_t laser_return_mode{0};
  uint8_t velodyne_model_ID{0};
};
#pragma pack(pop)

static_assert(sizeof(RawPacket) == sizeof(CObservationVelodyneScan::TVelodyneRawPacket));
static_assert(sizeof(RawPacket) == CObservationVelodyneScan::PACKET_SIZE);

// Thin wrapper to avoid GCC's overly-eager -Wdangling-reference false
// positive when binding a `const auto&` directly to the string-literal-taking
// overload of LoadDefaultCalibration().
const VelodyneCalibration& loadDefaultCal(const std::string& model)
{
  return VelodyneCalibration::LoadDefaultCalibration(model);
}

// Builds a minimal but valid VLP-16 (16-laser) scan with `nPackets` packets,
// in single ("strongest") return mode, each containing a flat, constant-range
// synthetic "wall" so that generatePointCloud()/fromVelodyne() have real data
// to project.
CObservationVelodyneScan::Ptr buildSyntheticVLP16Scan(
    size_t nPackets = 2, uint8_t returnMode = CObservationVelodyneScan::RETMODE_STRONGEST)
{
  auto obs = CObservationVelodyneScan::Create();
  obs->calibration = loadDefaultCal("VLP16");
  obs->minRange = 0.5;
  obs->maxRange = 100.0;
  obs->sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  obs->timestamp = mrpt::Clock::now();

  obs->scan_packets.resize(nPackets);
  for (size_t p = 0; p < nPackets; p++)
  {
    RawPacket raw{};
    raw.gps_timestamp = static_cast<uint32_t>(1000 + p * 553);  // one packet ~553us apart
    raw.laser_return_mode = returnMode;
    raw.velodyne_model_ID = 0x22;  // VLP-16

    for (int b = 0; b < 12; b++)
    {
      raw.blocks[b].header = CObservationVelodyneScan::UPPER_BANK;
      // Rotation advances both across blocks and packets:
      raw.blocks[b].rotation = static_cast<uint16_t>(
          (static_cast<int>(p) * 12 * 30 + b * 30) % CObservationVelodyneScan::ROTATION_MAX_UNITS);
      for (int k = 0; k < 32; k++)
      {
        // Constant-range synthetic wall (5.0 m, in 2mm units):
        raw.blocks[b].returns[k].distance = 2500;
        raw.blocks[b].returns[k].intensity = static_cast<uint8_t>(100 + k);
      }
    }
    std::memcpy(
        static_cast<void*>(&obs->scan_packets[p]), static_cast<const void*>(&raw), sizeof(raw));
  }
  return obs;
}
}  // namespace

// ------------------- VelodyneCalibration -------------------

TEST(VelodyneCalibration, LoadDefaultVLP16)
{
  const auto& cal = loadDefaultCal("VLP16");
  EXPECT_FALSE(cal.empty());
  EXPECT_EQ(cal.laser_corrections.size(), 16u);
}

TEST(VelodyneCalibration, LoadDefaultHDL32)
{
  const auto& cal = loadDefaultCal("HDL32");
  EXPECT_FALSE(cal.empty());
  EXPECT_EQ(cal.laser_corrections.size(), 32u);
}

TEST(VelodyneCalibration, LoadDefaultHDL64)
{
  const auto& cal = loadDefaultCal("HDL64");
  EXPECT_FALSE(cal.empty());
  EXPECT_EQ(cal.laser_corrections.size(), 64u);
}

TEST(VelodyneCalibration, LoadDefaultUnknownModel)
{
  const auto& cal = loadDefaultCal("NOT_A_REAL_MODEL");
  EXPECT_TRUE(cal.empty());
}

TEST(VelodyneCalibration, LoadDefaultIsCached)
{
  const auto& cal1 = loadDefaultCal("VLP16");
  const auto& cal2 = loadDefaultCal("VLP16");
  EXPECT_EQ(&cal1, &cal2);
}

TEST(VelodyneCalibration, EmptyAndClear)
{
  VelodyneCalibration cal;
  EXPECT_TRUE(cal.empty());
  cal = loadDefaultCal("VLP16");
  EXPECT_FALSE(cal.empty());
  cal.clear();
  EXPECT_TRUE(cal.empty());
}

TEST(VelodyneCalibration, LoadFromXMLTextMalformed)
{
  VelodyneCalibration cal;
  EXPECT_FALSE(cal.loadFromXMLText("<not><valid"));
}

TEST(VelodyneCalibration, LoadFromXMLFileMissing)
{
  VelodyneCalibration cal;
  EXPECT_FALSE(cal.loadFromXMLFile("/nonexistent/path/to/calib.xml"));
}

TEST(VelodyneCalibration, LoadFromYAMLTextMalformed)
{
  VelodyneCalibration cal;
  EXPECT_FALSE(cal.loadFromYAMLText("num_lasers: 0"));
}

TEST(VelodyneCalibration, LoadFromYAMLFileMissing)
{
  VelodyneCalibration cal;
  EXPECT_FALSE(cal.loadFromYAMLFile("/nonexistent/path/to/calib.yaml"));
}

TEST(VelodyneCalibration, RoundtripThroughXMLFile)
{
  // Re-derive an XML file from the default VLP-16 XML text, then reload it
  // from disk to exercise loadFromXMLFile()'s success path.
  const auto& original = loadDefaultCal("VLP16");
  ASSERT_FALSE(original.empty());

  // We don't have direct access to the embedded XML string here, so instead
  // verify loadFromXMLFile() succeeds when pointed at a copy of a minimal
  // XML document with the expected structure for a single laser:
  const std::string xml =
      "<boost_serialization><DB>"
      "<enabled_ class_id=\"0\" tracking_level=\"0\" version=\"0\">"
      "<count>1</count><item_version>0</item_version><item>1</item></enabled_>"
      "<points_ class_id=\"1\" tracking_level=\"0\" version=\"0\">"
      "<count>1</count><item_version>0</item_version>"
      "<item><px><id_>0</id_><rotCorrection_>1.5</rotCorrection_>"
      "<vertCorrection_>2.5</vertCorrection_><distCorrection_>10</distCorrection_>"
      "<distCorrectionX_>0</distCorrectionX_><distCorrectionY_>0</distCorrectionY_>"
      "<vertOffsetCorrection_>5</vertOffsetCorrection_>"
      "<horizOffsetCorrection_>3</horizOffsetCorrection_>"
      "<focalDistance_>0</focalDistance_><focalSlope_>0</focalSlope_>"
      "<closeSlope_>0</closeSlope_></px><off1_>0</off1_><off2_>0</off2_><off3_>0</off3_>"
      "</item></points_>"
      "</DB></boost_serialization>";

  const std::string tmpFile = mrpt::system::getTempFileName() + ".xml";
  {
    std::ofstream f(tmpFile);
    f << xml;
  }

  VelodyneCalibration cal;
  const bool ok = cal.loadFromXMLFile(tmpFile);
  EXPECT_TRUE(ok);
  ASSERT_EQ(cal.laser_corrections.size(), 1u);
  EXPECT_NEAR(cal.laser_corrections[0].azimuthCorrection, 1.5, 1e-6);
  EXPECT_NEAR(cal.laser_corrections[0].verticalCorrection, 2.5, 1e-6);

  mrpt::system::deleteFile(tmpFile);
}

TEST(VelodyneCalibration, RoundtripThroughYAMLFile)
{
  const std::string yaml =
      "num_lasers: 1\n"
      "lasers:\n"
      " - {laser_id: 0, rot_correction: 0.1, vert_correction: 0.2, "
      "dist_correction: 0.01, vert_offset_correction: 0.02, "
      "horiz_offset_correction: 0.03}\n";

  const std::string tmpFile = mrpt::system::getTempFileName() + ".yaml";
  {
    std::ofstream f(tmpFile);
    f << yaml;
  }

  VelodyneCalibration cal;
  const bool ok = cal.loadFromYAMLFile(tmpFile);
  EXPECT_TRUE(ok);
  ASSERT_EQ(cal.laser_corrections.size(), 1u);
  EXPECT_NEAR(cal.laser_corrections[0].azimuthCorrection, 0.1, 1e-6);

  mrpt::system::deleteFile(tmpFile);
}

// ------------------- CObservationVelodyneScan -------------------

TEST(CObservationVelodyneScan, GeneratePointCloudDefault)
{
  auto obs = buildSyntheticVLP16Scan();
  obs->generatePointCloud();
  EXPECT_GT(obs->point_cloud.size(), 0u);
  EXPECT_EQ(obs->point_cloud.x.size(), obs->point_cloud.intensity.size());
}

TEST(CObservationVelodyneScan, GeneratePointCloudWithFilters)
{
  auto obs = buildSyntheticVLP16Scan();

  CObservationVelodyneScan::TGeneratePointCloudParameters params;
  params.minAzimuth_deg = 10.0;
  params.maxAzimuth_deg = 350.0;
  params.minDistance = 0.5f;
  params.maxDistance = 50.0f;
  params.filterByROI = true;
  params.ROI_x_min = -100;
  params.ROI_x_max = 100;
  params.ROI_y_min = -100;
  params.ROI_y_max = 100;
  params.ROI_z_min = -100;
  params.ROI_z_max = 100;
  params.filterOutIsolatedPoints = true;
  params.isolatedPointsFilterDistance = 5.0;
  params.generatePerPointTimestamp = true;
  params.generatePerPointAzimuth = true;
  params.generatePointsForLaserID = true;

  obs->generatePointCloud(params);
  EXPECT_EQ(obs->point_cloud.timestamp.size(), obs->point_cloud.size());
  EXPECT_EQ(obs->point_cloud.azimuth.size(), obs->point_cloud.size());
  EXPECT_EQ(obs->point_cloud.laser_id.size(), obs->point_cloud.size());
  EXPECT_EQ(obs->point_cloud.pointsForLaserID.size(), 16u);
}

TEST(CObservationVelodyneScan, GeneratePointCloudNegativeROIFilter)
{
  auto obs = buildSyntheticVLP16Scan();
  CObservationVelodyneScan::TGeneratePointCloudParameters params;
  params.filterBynROI = true;
  params.nROI_x_min = -1000;
  params.nROI_x_max = 1000;
  params.nROI_y_min = -1000;
  params.nROI_y_max = 1000;
  params.nROI_z_min = -1000;
  params.nROI_z_max = 1000;
  obs->generatePointCloud(params);
  // Everything falls within the exclusion region, so no points should pass:
  EXPECT_EQ(obs->point_cloud.size(), 0u);
}

TEST(CObservationVelodyneScan, GeneratePointCloudDualReturnMode)
{
  auto obs = buildSyntheticVLP16Scan(2, CObservationVelodyneScan::RETMODE_DUAL);
  // Make the "2nd" (odd) blocks differ in range from the preceding block so
  // they are not treated as duplicated points:
  for (auto& pkt : obs->scan_packets)
  {
    RawPacket raw{};
    std::memcpy(static_cast<void*>(&raw), static_cast<const void*>(&pkt), sizeof(raw));
    for (int b = 1; b < 12; b += 2)
    {
      for (int k = 0; k < 32; k++) raw.blocks[b].returns[k].distance = 3000;
    }
    std::memcpy(static_cast<void*>(&pkt), static_cast<const void*>(&raw), sizeof(raw));
  }

  CObservationVelodyneScan::TGeneratePointCloudParameters params;
  params.dualKeepStrongest = true;
  params.dualKeepLast = true;
  obs->generatePointCloud(params);
  EXPECT_GT(obs->point_cloud.size(), 0u);
}

TEST(CObservationVelodyneScan, TPointCloudReserveClear)
{
  CObservationVelodyneScan::TPointCloud pc;
  pc.reserve(100);
  pc.x = {1, 2, 3};
  pc.y = {1, 2, 3};
  pc.z = {1, 2, 3};
  pc.intensity = {1, 2, 3};
  EXPECT_EQ(pc.size(), 3u);
  pc.clear();
  EXPECT_EQ(pc.size(), 0u);
  pc.x = {1, 2, 3};
  pc.clear_deep();
  EXPECT_EQ(pc.size(), 0u);
}

TEST(CObservationVelodyneScan, GetDescriptionAsTextAndUnload)
{
  auto obs = buildSyntheticVLP16Scan();
  obs->generatePointCloud();
  std::stringstream ss;
  obs->getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  obs->unload();
  EXPECT_EQ(obs->point_cloud.size(), 0u);
}

TEST(CObservationVelodyneScan, SerializationRoundtrip)
{
  auto obs1 = buildSyntheticVLP16Scan();
  obs1->sensorLabel = "velodyne_test";

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << *obs1;
  buf.Seek(0);

  auto obs2 = CObservationVelodyneScan::Create();
  arch >> *obs2;

  EXPECT_EQ(obs2->sensorLabel, obs1->sensorLabel);
  EXPECT_EQ(obs2->scan_packets.size(), obs1->scan_packets.size());
  EXPECT_EQ(obs2->calibration.laser_corrections.size(), obs1->calibration.laser_corrections.size());
}

// ------------------- CObservationRotatingScan -------------------

TEST(CObservationRotatingScan, FromVelodyne)
{
  auto velo = buildSyntheticVLP16Scan(3);
  velo->sensorLabel = "velo";

  CObservationRotatingScan rot;
  rot.fromVelodyne(*velo);

  EXPECT_EQ(rot.rowCount, 16);
  EXPECT_GT(rot.columnCount, 0);
  EXPECT_EQ(rot.sensorLabel, "velo");
  EXPECT_FALSE(rot.lidarModel.empty());
  EXPECT_EQ(rot.organizedPoints.rows(), rot.rowCount);
  EXPECT_EQ(rot.organizedPoints.cols(), rot.columnCount);
}

TEST(CObservationRotatingScan, FromGenericVelodyne)
{
  auto velo = buildSyntheticVLP16Scan(3);
  CObservationRotatingScan rot;
  EXPECT_TRUE(rot.fromGeneric(*velo));
  EXPECT_GT(rot.rowCount, 0);
}

TEST(CObservationRotatingScan, FromScan2D)
{
  CObservation2DRangeScan scan2d;
  scan2d.aperture = static_cast<float>(M_PI);
  scan2d.rightToLeft = true;
  scan2d.maxRange = 20.0f;
  scan2d.sensorLabel = "laser2d";
  scan2d.resizeScan(8);
  for (size_t i = 0; i < 8; i++)
  {
    scan2d.setScanRange(i, 5.0f + static_cast<float>(i));
    scan2d.setScanRangeValidity(i, true);
  }

  CObservationRotatingScan rot;
  rot.fromScan2D(scan2d);

  EXPECT_EQ(rot.rowCount, 1);
  EXPECT_EQ(rot.columnCount, 8);
  EXPECT_NE(rot.lidarModel.find("2D_SCAN_"), std::string::npos);
}

TEST(CObservationRotatingScan, FromGenericScan2D)
{
  CObservation2DRangeScan scan2d;
  scan2d.resizeScan(4);
  CObservationRotatingScan rot;
  EXPECT_TRUE(rot.fromGeneric(scan2d));
}

TEST(CObservationRotatingScan, FromGenericUnsupported)
{
  CObservationIMU imu;
  CObservationRotatingScan rot;
  EXPECT_FALSE(rot.fromGeneric(imu));
}

TEST(CObservationRotatingScan, GetDescriptionAsText)
{
  auto velo = buildSyntheticVLP16Scan();
  CObservationRotatingScan rot;
  rot.fromVelodyne(*velo);
  std::stringstream ss;
  rot.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CObservationRotatingScan, SerializationRoundtrip)
{
  auto velo = buildSyntheticVLP16Scan();
  CObservationRotatingScan rot1;
  rot1.fromVelodyne(*velo);
  rot1.sensorLabel = "rot_test";

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << rot1;
  buf.Seek(0);

  CObservationRotatingScan rot2;
  arch >> rot2;

  EXPECT_EQ(rot2.sensorLabel, rot1.sensorLabel);
  EXPECT_EQ(rot2.rowCount, rot1.rowCount);
  EXPECT_EQ(rot2.columnCount, rot1.columnCount);
  EXPECT_EQ(rot2.rangeImage.rows(), rot1.rangeImage.rows());
}

TEST(CObservationRotatingScan, SaveLoadTextFileRoundtrip)
{
  auto velo = buildSyntheticVLP16Scan();
  CObservationRotatingScan rot1;
  rot1.fromVelodyne(*velo);

  const std::string tmpFile = mrpt::system::getTempFileName() + ".txt";
  ASSERT_TRUE(rot1.saveToTextFile(tmpFile));

  CObservationRotatingScan rot2;
  rot2.rowCount = rot1.rowCount;
  rot2.columnCount = rot1.columnCount;
  ASSERT_TRUE(rot2.loadFromTextFile(tmpFile));

  EXPECT_EQ(rot2.rangeImage.rows(), rot1.rangeImage.rows());
  EXPECT_EQ(rot2.rangeImage.cols(), rot1.rangeImage.cols());

  mrpt::system::deleteFile(tmpFile);
}

TEST(CObservationRotatingScan, ExternalStorageMRPTSerializationRoundtrip)
{
  auto velo = buildSyntheticVLP16Scan();
  CObservationRotatingScan rot1;
  rot1.fromVelodyne(*velo);

  const std::string tmpFile = mrpt::system::getTempFileName() + "_rot.bin";
  EXPECT_FALSE(rot1.isExternallyStored());
  rot1.setAsExternalStorage(
      tmpFile, CObservationRotatingScan::ExternalStorageFormat::MRPT_Serialization);
  EXPECT_TRUE(rot1.isExternallyStored());
  EXPECT_EQ(rot1.getExternalStorageFile(), tmpFile);

  rot1.unload();  // will save to disk since the file doesn't exist yet
  EXPECT_TRUE(mrpt::system::fileExists(mrpt::io::lazy_load_absolute_path(tmpFile)));

  rot1.load();
  EXPECT_EQ(rot1.rangeImage.rows(), 16);

  mrpt::system::deleteFile(mrpt::io::lazy_load_absolute_path(tmpFile));
}
