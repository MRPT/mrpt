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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>

#include <cstring>

using namespace mrpt::obs;

TEST(CSensoryFrame, InsertPushBackOperatorPlusEqual)
{
  CSensoryFrame sf;
  EXPECT_TRUE(sf.empty());

  auto obs1 = CObservationOdometry::Create();
  obs1->sensorLabel = "odo";
  sf.push_back(obs1);
  EXPECT_EQ(sf.size(), 1u);

  auto obs2 = CObservationOdometry::Create();
  obs2->sensorLabel = "odo2";
  sf += obs2;
  EXPECT_EQ(sf.size(), 2u);

  CSensoryFrame sf2;
  auto obs3 = CObservationOdometry::Create();
  sf2.insert(obs3);
  sf += sf2;
  EXPECT_EQ(sf.size(), 3u);

  sf.clear();
  EXPECT_TRUE(sf.empty());
}

TEST(CSensoryFrame, GetObservationByClass)
{
  CSensoryFrame sf;
  sf.insert(CObservationOdometry::Create());
  auto scan1 = CObservation2DRangeScan::Create();
  scan1->sensorLabel = "laser1";
  sf.insert(scan1);
  auto scan2 = CObservation2DRangeScan::Create();
  scan2->sensorLabel = "laser2";
  sf.insert(scan2);

  auto found0 = sf.getObservationByClass<CObservation2DRangeScan>(0);
  ASSERT_TRUE(found0);
  EXPECT_EQ(found0->sensorLabel, "laser1");

  auto found1 = sf.getObservationByClass<CObservation2DRangeScan>(1);
  ASSERT_TRUE(found1);
  EXPECT_EQ(found1->sensorLabel, "laser2");

  auto notFound = sf.getObservationByClass<CObservation2DRangeScan>(2);
  EXPECT_FALSE(notFound);

  // Non-const overload:
  auto foundMutable = sf.getObservationByClass<CObservation2DRangeScan>(0);
  ASSERT_TRUE(foundMutable);
  foundMutable->sensorLabel = "changed";
  EXPECT_EQ(scan1->sensorLabel, "changed");
}

TEST(CSensoryFrame, GetObservationByIndex)
{
  CSensoryFrame sf;
  auto obs = CObservationOdometry::Create();
  sf.insert(obs);

  auto got = sf.getObservationByIndex(0);
  ASSERT_TRUE(got);
  EXPECT_THROW(sf.getObservationByIndex(5), std::exception);

  const CSensoryFrame& csf = sf;
  auto gotConst = csf.getObservationByIndex(0);
  ASSERT_TRUE(gotConst);

  auto asOdo = sf.getObservationByIndexAs<CObservationOdometry::Ptr>(0);
  ASSERT_TRUE(asOdo);
  auto asOdoConst = csf.getObservationByIndexAs<CObservationOdometry::Ptr>(0);
  ASSERT_TRUE(asOdoConst);
}

TEST(CSensoryFrame, GetObservationBySensorLabel)
{
  CSensoryFrame sf;
  auto obs1 = CObservationOdometry::Create();
  obs1->sensorLabel = "ODO";
  sf.insert(obs1);
  auto obs2 = CObservationOdometry::Create();
  obs2->sensorLabel = "ODO";
  sf.insert(obs2);

  auto found0 = sf.getObservationBySensorLabel("odo", 0);  // case-insensitive
  EXPECT_TRUE(found0);
  auto found1 = sf.getObservationBySensorLabel("ODO", 1);
  EXPECT_TRUE(found1);
  auto notFound = sf.getObservationBySensorLabel("nope");
  EXPECT_FALSE(notFound);

  const CSensoryFrame& csf = sf;
  auto foundConst = csf.getObservationBySensorLabel("ODO");
  EXPECT_TRUE(foundConst);

  auto asOdo = sf.getObservationBySensorLabelAs<CObservationOdometry::Ptr>("ODO");
  ASSERT_TRUE(asOdo);
  auto asOdoConst = csf.getObservationBySensorLabelAs<CObservationOdometry::Ptr>("ODO");
  ASSERT_TRUE(asOdoConst);
}

TEST(CSensoryFrame, EraseByIndexEraseIteratorEraseByLabel)
{
  CSensoryFrame sf;
  for (int i = 0; i < 3; i++)
  {
    auto o = CObservationOdometry::Create();
    o->sensorLabel = (i == 1) ? "target" : "other";
    sf.insert(o);
  }
  ASSERT_EQ(sf.size(), 3u);

  sf.eraseByIndex(0);
  EXPECT_EQ(sf.size(), 2u);
  EXPECT_THROW(sf.eraseByIndex(100), std::exception);

  auto it = sf.begin();
  it = sf.erase(it);
  EXPECT_EQ(sf.size(), 1u);
  EXPECT_THROW(sf.erase(sf.end()), std::exception);

  CSensoryFrame sf2;
  auto a = CObservationOdometry::Create();
  a->sensorLabel = "A";
  sf2.insert(a);
  auto b = CObservationOdometry::Create();
  b->sensorLabel = "B";
  sf2.insert(b);
  auto a2 = CObservationOdometry::Create();
  a2->sensorLabel = "A";
  sf2.insert(a2);

  sf2.eraseByLabel("A");
  EXPECT_EQ(sf2.size(), 1u);
  EXPECT_EQ(sf2.getObservationByIndex(0)->sensorLabel, "B");
}

TEST(CSensoryFrame, Iterators)
{
  CSensoryFrame sf;
  for (int i = 0; i < 3; i++) sf.insert(CObservationOdometry::Create());

  size_t count = 0;
  for (auto it = sf.begin(); it != sf.end(); ++it) count++;
  EXPECT_EQ(count, 3u);

  const CSensoryFrame& csf = sf;
  count = 0;
  for (auto it = csf.begin(); it != csf.end(); ++it) count++;
  EXPECT_EQ(count, 3u);
}

TEST(CSensoryFrame, Swap)
{
  CSensoryFrame a, b;
  a.insert(CObservationOdometry::Create());
  EXPECT_EQ(a.size(), 1u);
  EXPECT_EQ(b.size(), 0u);
  a.swap(b);
  EXPECT_EQ(a.size(), 0u);
  EXPECT_EQ(b.size(), 1u);
}

TEST(CSensoryFrame, BuildAuxPointsMapThrowsWithoutMrptMaps)
{
  CSensoryFrame sf;
  auto scan = CObservation2DRangeScan::Create();
  scan->resizeScan(4);
  sf.insert(scan);
  EXPECT_THROW(sf.buildAuxPointsMap<mrpt::maps::CMetricMap>(), std::exception);
  EXPECT_EQ(sf.getAuxPointsMap<mrpt::maps::CMetricMap>(), nullptr);
}

TEST(CSensoryFrame, SerializationRoundtripCurrentVersion)
{
  CSensoryFrame sf1;
  auto obs1 = CObservationOdometry::Create();
  obs1->sensorLabel = "odo";
  sf1.insert(obs1);
  auto scan = CObservation2DRangeScan::Create();
  scan->resizeScan(3);
  scan->setScanRange(0, 1.5f);
  sf1.insert(scan);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << sf1;
  buf.Seek(0);

  CSensoryFrame sf2;
  arch >> sf2;
  ASSERT_EQ(sf2.size(), 2u);
  EXPECT_EQ(sf2.getObservationByIndex(0)->sensorLabel, "odo");
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

TEST(CSensoryFrame, DeserializeLegacyVersion0WithID)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CSensoryFrame", 0);

  arch << static_cast<uint32_t>(123);  // legacy "ID" field (removed in v2)
  mrpt::system::TTimeStamp ts = mrpt::Clock::now();
  arch.WriteBufferFixEndianness(&ts, 1);  // version==0: extra timestamp

  const uint32_t n = 1;
  arch << n;
  auto obs = CObservationOdometry::Create();
  arch << *obs;  // ObjectReadFromStream reads a full nested object

  writeObjectFooter(arch);
  buf.Seek(0);

  CSensoryFrame sf;
  arch >> sf;
  ASSERT_EQ(sf.size(), 1u);
  // version==0 forces all observations' timestamp to the legacy value:
  EXPECT_EQ(sf.getObservationByIndex(0)->timestamp, ts);
}

TEST(CSensoryFrame, DeserializeLegacyVersion1NoTimestampOverride)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CSensoryFrame", 1);

  arch << static_cast<uint32_t>(42);  // legacy "ID" (removed in v2)
  const uint32_t n = 1;
  arch << n;
  auto obs = CObservationOdometry::Create();
  obs->timestamp = mrpt::Clock::now();
  arch << *obs;

  writeObjectFooter(arch);
  buf.Seek(0);

  CSensoryFrame sf;
  arch >> sf;
  ASSERT_EQ(sf.size(), 1u);
}
