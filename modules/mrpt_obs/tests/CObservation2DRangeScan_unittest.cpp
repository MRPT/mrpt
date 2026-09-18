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
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/serialization/CArchive.h>

#include <cstring>
#include <sstream>

using namespace mrpt::obs;

namespace
{
void fillScan(CObservation2DRangeScan& o, size_t N = 8)
{
  o.aperture = static_cast<float>(M_PI);
  o.rightToLeft = true;
  o.maxRange = 20.0f;
  o.resizeScan(N);
  for (size_t i = 0; i < N; i++)
  {
    o.setScanRange(i, 5.0f + static_cast<float>(i));
    o.setScanRangeValidity(i, true);
  }
}
}  // namespace

TEST(CObservation2DRangeScan, ResizeAndBasicAccessors)
{
  CObservation2DRangeScan o;
  fillScan(o);
  EXPECT_EQ(o.getScanSize(), 8u);
  EXPECT_NEAR(o.getScanRange(0), 5.0f, 1e-4f);
  o.setScanRange(1, 9.5f);
  EXPECT_NEAR(o.getScanRange(1), 9.5f, 1e-4f);

  EXPECT_TRUE(o.getScanRangeValidity(0));
  o.setScanRangeValidity(0, false);
  EXPECT_FALSE(o.getScanRangeValidity(0));

  EXPECT_FALSE(o.hasIntensity());
  o.setScanHasIntensity(true);
  EXPECT_TRUE(o.hasIntensity());
  o.setScanIntensity(2, 42);
  EXPECT_EQ(o.getScanIntensity(2), 42);
}

TEST(CObservation2DRangeScan, ResizeScanAndAssign)
{
  CObservation2DRangeScan o;
  o.resizeScanAndAssign(5, 3.0f, true, 7);
  ASSERT_EQ(o.getScanSize(), 5u);
  for (size_t i = 0; i < 5; i++)
  {
    EXPECT_NEAR(o.getScanRange(i), 3.0f, 1e-6f);
    EXPECT_TRUE(o.getScanRangeValidity(i));
    EXPECT_EQ(o.getScanIntensity(i), 7);
  }
}

TEST(CObservation2DRangeScan, LoadFromVectors)
{
  CObservation2DRangeScan o;
  std::vector<float> ranges = {1.0f, 2.0f, 3.0f};
  std::vector<char> valid = {1, 0, 1};
  o.loadFromVectors(3, ranges.data(), valid.data());
  ASSERT_EQ(o.getScanSize(), 3u);
  EXPECT_TRUE(o.getScanRangeValidity(0));
  EXPECT_FALSE(o.getScanRangeValidity(1));
}

TEST(CObservation2DRangeScan, GetScanAngleAndRelativeTimestamp)
{
  CObservation2DRangeScan o;
  fillScan(o);
  o.sweepDuration = 1.0f;

  EXPECT_NEAR(o.getScanAngle(0), -0.5 * M_PI, 1e-4);
  EXPECT_NEAR(o.getScanAngle(o.getScanSize() - 1), 0.5 * M_PI, 1e-4);

  EXPECT_NEAR(o.getScanRelativeTimestamp(0), 0.0f, 1e-6f);
  EXPECT_GT(o.getScanRelativeTimestamp(o.getScanSize() - 1), 0.0f);

  o.rightToLeft = false;
  EXPECT_NEAR(o.getScanAngle(0), 0.5 * M_PI, 1e-4);

  // sweepDuration<=0 -> always zero:
  o.sweepDuration = 0.0f;
  EXPECT_NEAR(o.getScanRelativeTimestamp(1), 0.0f, 1e-6f);
}

TEST(CObservation2DRangeScan, IsPlanarScan)
{
  CObservation2DRangeScan o;
  EXPECT_TRUE(o.isPlanarScan());
  o.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0.5, 0);
  EXPECT_FALSE(o.isPlanarScan());
}

TEST(CObservation2DRangeScan, GetScanProperties)
{
  CObservation2DRangeScan o;
  fillScan(o, 10);
  T2DScanProperties p;
  o.getScanProperties(p);
  EXPECT_EQ(p.nRays, 10u);
  EXPECT_NEAR(p.aperture, static_cast<float>(M_PI), 1e-6f);
  EXPECT_TRUE(p.rightToLeft);
}

TEST(CObservation2DRangeScan, T2DScanPropertiesOperatorLess)
{
  T2DScanProperties a, b;
  a.nRays = 5;
  b.nRays = 10;
  EXPECT_TRUE(a < b);
  EXPECT_FALSE(b < a);

  a.nRays = b.nRays = 5;
  a.aperture = 1.0f;
  b.aperture = 2.0f;
  EXPECT_TRUE(a < b);

  a.aperture = b.aperture;
  a.rightToLeft = true;
  b.rightToLeft = false;
  EXPECT_TRUE(a < b);
  EXPECT_FALSE(b < a);
}

TEST(CObservation2DRangeScan, TruncateByDistanceAndAngleNoHeightFilter)
{
  CObservation2DRangeScan o;
  fillScan(o, 5);
  o.truncateByDistanceAndAngle(6.0f, static_cast<float>(M_PI));
  // Ray 0 has range 5.0 < 6.0 -> should become invalid:
  EXPECT_FALSE(o.getScanRangeValidity(0));
}

TEST(CObservation2DRangeScan, TruncateByDistanceAndAngleWithHeightFilter)
{
  CObservation2DRangeScan o;
  fillScan(o, 5);
  o.truncateByDistanceAndAngle(0.0f, static_cast<float>(M_PI), -1.0f, 1.0f, 0.0f);
  // Just exercise the height-filter branch without throwing:
  SUCCEED();
}

TEST(CObservation2DRangeScan, FilterByExclusionAreasPolygonOnly)
{
  CObservation2DRangeScan o;
  fillScan(o, 4);
  o.sensorPose = mrpt::poses::CPose3D();

  mrpt::math::CPolygon poly;
  poly.push_back(mrpt::math::TPoint2D(-100, -100));
  poly.push_back(mrpt::math::TPoint2D(100, -100));
  poly.push_back(mrpt::math::TPoint2D(100, 100));
  poly.push_back(mrpt::math::TPoint2D(-100, 100));

  std::vector<mrpt::math::CPolygon> areas = {poly};
  o.filterByExclusionAreas(areas);
  // The whole huge polygon covers the origin, so all points at z=0 within
  // range get excluded:
  for (size_t i = 0; i < o.getScanSize(); i++)
  {
    EXPECT_FALSE(o.getScanRangeValidity(i));
  }
}

TEST(CObservation2DRangeScan, FilterByExclusionAreasEmptyIsNoOp)
{
  CObservation2DRangeScan o;
  fillScan(o, 4);
  o.filterByExclusionAreas(std::vector<mrpt::math::CPolygon>());
  for (size_t i = 0; i < o.getScanSize(); i++) EXPECT_TRUE(o.getScanRangeValidity(i));
}

TEST(CObservation2DRangeScan, FilterByExclusionAreasWithZRange)
{
  CObservation2DRangeScan o;
  fillScan(o, 4);

  mrpt::math::CPolygon poly;
  poly.push_back(mrpt::math::TPoint2D(-100, -100));
  poly.push_back(mrpt::math::TPoint2D(100, -100));
  poly.push_back(mrpt::math::TPoint2D(100, 100));
  poly.push_back(mrpt::math::TPoint2D(-100, 100));

  CObservation2DRangeScan::TListExclusionAreasWithRanges areas;
  areas.emplace_back(poly, std::make_pair(10.0, 20.0));  // z range excludes z=0
  o.filterByExclusionAreas(areas);
  // z=0 points are outside [10,20], so nothing should be excluded:
  for (size_t i = 0; i < o.getScanSize(); i++) EXPECT_TRUE(o.getScanRangeValidity(i));
}

TEST(CObservation2DRangeScan, FilterByExclusionAreasEmptyScan)
{
  CObservation2DRangeScan o;  // no scan data
  mrpt::math::CPolygon poly;
  poly.push_back(mrpt::math::TPoint2D(-1, -1));
  poly.push_back(mrpt::math::TPoint2D(1, -1));
  poly.push_back(mrpt::math::TPoint2D(1, 1));
  std::vector<mrpt::math::CPolygon> areas = {poly};
  o.filterByExclusionAreas(areas);  // should just return, no crash
  SUCCEED();
}

TEST(CObservation2DRangeScan, FilterByExclusionAnglesForwardRange)
{
  CObservation2DRangeScan o;
  fillScan(o, 8);
  std::vector<std::pair<double, double>> angles = {
      {-0.1, 0.1}
  };
  o.filterByExclusionAngles(angles);
  // At least the central ray(s) should now be invalid:
  bool anyInvalid = false;
  for (size_t i = 0; i < o.getScanSize(); i++)
    if (!o.getScanRangeValidity(i)) anyInvalid = true;
  EXPECT_TRUE(anyInvalid);
}

TEST(CObservation2DRangeScan, FilterByExclusionAnglesWrappedRange)
{
  CObservation2DRangeScan o;
  fillScan(o, 8);
  // A "wrapped" range where end < start in index space:
  std::vector<std::pair<double, double>> angles = {
      {2.0, -2.0}
  };
  o.filterByExclusionAngles(angles);
  SUCCEED();
}

TEST(CObservation2DRangeScan, FilterByExclusionAnglesEmptyIsNoOp)
{
  CObservation2DRangeScan o;
  fillScan(o, 4);
  o.filterByExclusionAngles(std::vector<std::pair<double, double>>());
  for (size_t i = 0; i < o.getScanSize(); i++) EXPECT_TRUE(o.getScanRangeValidity(i));
}

TEST(CObservation2DRangeScan, BuildAuxPointsMapThrowsWithoutMrptMaps)
{
  CObservation2DRangeScan o;
  fillScan(o, 4);
  EXPECT_THROW(o.buildAuxPointsMap<mrpt::maps::CMetricMap>(), std::exception);
  EXPECT_EQ(o.getAuxPointsMap<mrpt::maps::CMetricMap>(), nullptr);
}

TEST(CObservation2DRangeScan, GetDescriptionAsTextWithIntensity)
{
  CObservation2DRangeScan o;
  fillScan(o, 4);
  o.setScanHasIntensity(true);
  o.setScanIntensity(0, 5);
  std::stringstream ss;
  o.getDescriptionAsText(ss);
  EXPECT_NE(ss.str().find("Raw intensity values"), std::string::npos);
}

TEST(CObservation2DRangeScan, ExportTxt)
{
  CObservation2DRangeScan o;
  fillScan(o, 3);
  EXPECT_TRUE(o.exportTxtSupported());
  EXPECT_NE(o.exportTxtHeader().find("RANGES"), std::string::npos);
  EXPECT_FALSE(o.exportTxtDataRow().empty());

  o.setScanHasIntensity(true);
  EXPECT_NE(o.exportTxtHeader().find("INTENSITY"), std::string::npos);
  EXPECT_FALSE(o.exportTxtDataRow().empty());
}

TEST(CObservation2DRangeScan, SerializationRoundtripCurrentVersion)
{
  CObservation2DRangeScan o1;
  fillScan(o1, 6);
  o1.setScanHasIntensity(true);
  for (size_t i = 0; i < o1.getScanSize(); i++) o1.setScanIntensity(i, static_cast<int>(i * 3));
  o1.sweepDuration = 0.5f;
  o1.deltaPitch = 0.1;
  o1.sensorLabel = "laser1";

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservation2DRangeScan o2;
  arch >> o2;

  ASSERT_EQ(o2.getScanSize(), o1.getScanSize());
  EXPECT_NEAR(o2.getScanRange(2), o1.getScanRange(2), 1e-5f);
  EXPECT_TRUE(o2.hasIntensity());
  EXPECT_EQ(o2.getScanIntensity(3), o1.getScanIntensity(3));
  EXPECT_NEAR(o2.sweepDuration, o1.sweepDuration, 1e-6f);
  EXPECT_NEAR(o2.deltaPitch, o1.deltaPitch, 1e-9);
  EXPECT_EQ(o2.sensorLabel, o1.sensorLabel);
}

namespace
{
void writeObjectHeader(mrpt::serialization::CArchive& arch, const char* className, uint8_t version)
{
  const auto len = static_cast<int8_t>(strlen(className) | 0x80);
  arch << len;
  arch.WriteBuffer(className, strlen(className));
  arch << version;
}
void writeObjectFooter(mrpt::serialization::CArchive& arch) { arch << static_cast<uint8_t>(0x88); }
}  // namespace

TEST(CObservation2DRangeScan, DeserializeLegacyVersion0)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CObservation2DRangeScan", 0);

  arch << 3.14159f;                // aperture
  arch << true;                    // rightToLeft
  arch << 30.0f;                   // maxRange
  arch << mrpt::poses::CPose3D();  // sensorPose
  mrpt::math::CMatrixF covSensorPose(6, 6);
  arch << covSensorPose;

  const uint32_t N = 3;
  arch << N;
  const float ranges[3] = {1.0f, 40.0f, 3.0f};  // [1] should end up invalid (>= maxRange)
  arch.WriteBufferFixEndianness(ranges, N);
  // version 0: no validRange, no stdError, no timestamp
  writeObjectFooter(arch);
  buf.Seek(0);

  CObservation2DRangeScan o;
  arch >> o;

  ASSERT_EQ(o.getScanSize(), 3u);
  EXPECT_TRUE(o.getScanRangeValidity(0));
  EXPECT_FALSE(o.getScanRangeValidity(1));  // range(40) >= maxRange(30)
  EXPECT_NEAR(o.stdError, 0.01f, 1e-6f);
}

TEST(CObservation2DRangeScan, DeserializeLegacyVersion4NoLabelNoDeltaPitch)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CObservation2DRangeScan", 4);

  arch << 3.14159f;
  arch << true;
  arch << 30.0f;
  arch << mrpt::poses::CPose3D();
  mrpt::math::CMatrixF covSensorPose(6, 6);
  arch << covSensorPose;  // version<6: still present

  const uint32_t N = 2;
  arch << N;
  const float ranges[2] = {2.0f, 3.0f};
  arch.WriteBufferFixEndianness(ranges, N);
  const char valid[2] = {1, 1};
  arch.WriteBuffer(valid, sizeof(valid));
  arch << 0.02f;  // stdError
  mrpt::system::TTimeStamp ts = mrpt::Clock::now();
  arch.WriteBufferFixEndianness(&ts, 1);
  arch << 0.1f;  // beamAperture
  // version<5: no sensorLabel, no deltaPitch, no intensity (v<7), no
  // sweepDuration (v<8)
  writeObjectFooter(arch);
  buf.Seek(0);

  CObservation2DRangeScan o;
  arch >> o;
  ASSERT_EQ(o.getScanSize(), 2u);
  EXPECT_EQ(o.sensorLabel, "");
  EXPECT_NEAR(o.deltaPitch, 0.0, 1e-9);
  EXPECT_NEAR(o.sweepDuration, 0.0f, 1e-9f);
}
