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
#include <mrpt/obs/gnss_messages.h>
#include <mrpt/serialization/CArchive.h>

#include <sstream>

using namespace mrpt::obs::gnss;
using namespace mrpt::serialization;

namespace
{
// Generic roundtrip test: write a message via the polymorphic base-class
// interface, read it back into a fresh Factory-built instance, and check the
// message type is preserved. Also exercises the human-readable dump methods.
void checkRoundtrip(gnss_message& msg)
{
  mrpt::io::CMemoryStream buf;
  auto arch = archiveFrom(buf);
  msg.writeToStream(arch);
  buf.Seek(0);

  std::unique_ptr<gnss_message> recons(gnss_message::Factory(msg.message_type));
  ASSERT_TRUE(recons);
  recons->readFromStream(arch);
  EXPECT_EQ(recons->message_type, msg.message_type);

  std::stringstream ss;
  msg.dumpToStream(ss);
  msg.dumpToConsole(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_FALSE(msg.getMessageTypeAsString().empty());
}
}  // namespace

TEST(gnss_messages, FactoryAndTypeString)
{
  for (const auto type :
       {NMEA_GGA,
        NMEA_GSA,
        NMEA_RMC,
        NMEA_ZDA,
        NMEA_VTG,
        NMEA_GLL,
        TOPCON_PZS,
        TOPCON_SATS,
        NV_OEM6_GENERIC_FRAME,
        NV_OEM6_BESTPOS,
        NV_OEM6_GENERIC_SHORT_FRAME,
        NV_OEM6_INSPVAS,
        NV_OEM6_RANGECMP,
        NV_OEM6_RXSTATUS,
        NV_OEM6_RAWEPHEM,
        NV_OEM6_VERSION,
        NV_OEM6_RAWIMUS,
        NV_OEM6_MARKPOS,
        NV_OEM6_MARKTIME,
        NV_OEM6_MARK2TIME,
        NV_OEM6_IONUTC})
  {
    EXPECT_TRUE(gnss_message::FactoryKnowsMsgType(type));
    std::unique_ptr<gnss_message> msg(gnss_message::Factory(type));
    ASSERT_TRUE(msg) << "type=" << type;
    EXPECT_EQ(msg->message_type, type);
    EXPECT_FALSE(msg->getMessageTypeAsString().empty());
  }
}

TEST(gnss_messages, FactoryUnknownType)
{
  const auto unknownType = static_cast<gnss_message_type_t>(7);
  EXPECT_FALSE(gnss_message::FactoryKnowsMsgType(unknownType));
  EXPECT_EQ(gnss_message::Factory(unknownType), nullptr);
}

TEST(gnss_messages, NmeaGGA)
{
  Message_NMEA_GGA m;
  m.fields.latitude_degrees = 36.5;
  m.fields.longitude_degrees = -4.5;
  m.fields.altitude_meters = 123.4;
  m.fields.fix_quality = 1;
  m.fields.satellitesUsed = 7;
  m.fields.thereis_HDOP = true;
  m.fields.HDOP = 1.2f;
  m.fields.UTCTime.hour = 10;
  m.fields.UTCTime.minute = 20;
  m.fields.UTCTime.sec = 30.5;
  checkRoundtrip(m);

  std::stringstream desc;
  std::stringstream vals;
  EXPECT_TRUE(m.getAllFieldDescriptions(desc));
  EXPECT_TRUE(m.getAllFieldValues(vals));
  EXPECT_FALSE(desc.str().empty());
  EXPECT_FALSE(vals.str().empty());

  // Also exercise the "unknown fix mode" and "no HDOP" branches:
  Message_NMEA_GGA m2;
  m2.fields.fix_quality = 200;
  m2.fields.thereis_HDOP = false;
  std::stringstream ss2;
  m2.dumpToStream(ss2);
  EXPECT_NE(ss2.str().find("UNKNOWN"), std::string::npos);
}

TEST(gnss_messages, NmeaGLL)
{
  Message_NMEA_GLL m;
  m.fields.latitude_degrees = 10.0;
  m.fields.longitude_degrees = 20.0;
  m.fields.validity_char = 'A';
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NmeaRMC)
{
  Message_NMEA_RMC m;
  m.fields.latitude_degrees = 1.0;
  m.fields.longitude_degrees = 2.0;
  m.fields.speed_knots = 5.0;
  m.fields.direction_degrees = 90.0;
  m.fields.date_day = 15;
  m.fields.date_month = 6;
  m.fields.date_year = 24;
  m.fields.positioning_mode = 'A';
  checkRoundtrip(m);

  const auto ts = m.getDateAsTimestamp();
  mrpt::system::TTimeParts parts;
  mrpt::system::timestampToParts(ts, parts);
  EXPECT_EQ(parts.day, 15);
  EXPECT_EQ(parts.month, 6);

  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NmeaVTG)
{
  Message_NMEA_VTG m;
  m.fields.true_track = 10.0;
  m.fields.magnetic_track = 12.0;
  m.fields.ground_speed_knots = 5.0;
  m.fields.ground_speed_kmh = 9.0;
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NmeaGSA)
{
  Message_NMEA_GSA m;
  m.fields.PDOP = 1.1;
  m.fields.HDOP = 2.2;
  m.fields.VDOP = 3.3;
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NmeaZDA)
{
  Message_NMEA_ZDA m;
  m.fields.date_day = 1;
  m.fields.date_month = 2;
  m.fields.date_year = 2024;
  m.fields.UTCTime.hour = 3;
  m.fields.UTCTime.minute = 4;
  m.fields.UTCTime.sec = 5.0;
  checkRoundtrip(m);

  const auto tsDate = m.getDateAsTimestamp();
  const auto tsDateTime = m.getDateTimeAsTimestamp();
  EXPECT_NE(tsDate, tsDateTime);

  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, TopconPZS)
{
  Message_TOPCON_PZS m;
  m.latitude_degrees = 40.0;
  m.longitude_degrees = -3.0;
  m.height_meters = 650.0;
  m.hasCartesianPosVel = true;
  m.cartesian_x = 1.0;
  m.cartesian_y = 2.0;
  m.cartesian_z = 3.0;
  m.hasPosCov = true;
  m.hasVelCov = true;
  m.hasStats = true;
  m.stats_GPS_sats_used = 8;
  m.stats_GLONASS_sats_used = 4;
  checkRoundtrip(m);

  // Also test the "no optional data" dump branches:
  Message_TOPCON_PZS m2;
  std::stringstream ss2;
  m2.dumpToStream(ss2);
  EXPECT_FALSE(ss2.str().empty());
}

TEST(gnss_messages, TopconSATS)
{
  Message_TOPCON_SATS m;
  m.USIs = {1, 2, 3};
  m.ELs = {10, 20, 30};
  m.AZs = {100, 200, 300};
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelBestpos)
{
  Message_NV_OEM6_BESTPOS m;
  m.fields.lat = 36.7;
  m.fields.lon = -4.4;
  m.fields.hgt = 50.0;
  m.fields.solution_stat = nv_oem6_solution_status::SOL_COMPUTED;
  m.fields.position_type = nv_oem6_position_type::SINGLE;
  m.fields.num_sats_tracked = 9;
  m.fields.num_sats_sol = 8;
  checkRoundtrip(m);

  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NovatelInspvas)
{
  Message_NV_OEM6_INSPVAS m;
  m.fields.lat = 1.0;
  m.fields.lon = 2.0;
  m.fields.hgt = 3.0;
  m.fields.ins_status = nv_oem6_ins_status_type::INS_SOLUTION_GOOD;
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NovatelInscovs)
{
  Message_NV_OEM6_INSCOVS m;
  for (int i = 0; i < 9; i++)
  {
    m.fields.pos_cov[i] = i;
    m.fields.att_cov[i] = i * 2;
    m.fields.vel_cov[i] = i * 3;
  }
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NovatelRxstatus)
{
  Message_NV_OEM6_RXSTATUS m;
  m.fields.error = 0x1234;
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelRawephem)
{
  Message_NV_OEM6_RAWEPHEM m;
  m.fields.sat_prn = 5;
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelRawimus)
{
  Message_NV_OEM6_RAWIMUS m;
  m.fields.accel_x = 100;
  m.fields.accel_y_neg = 200;
  m.fields.accel_z = 300;
  m.fields.gyro_x = 10;
  m.fields.gyro_y_neg = 20;
  m.fields.gyro_z = 30;
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NovatelMarkpos)
{
  Message_NV_OEM6_MARKPOS m;
  m.fields.lat = 1.0;
  m.fields.lon = 2.0;
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelMarktime)
{
  Message_NV_OEM6_MARKTIME m;
  m.fields.week = 100;
  m.fields.week_seconds = 12.5;
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NovatelMark2time)
{
  Message_NV_OEM6_MARK2TIME m;
  m.fields.week = 200;
  m.fields.week_seconds = 5.5;
  checkRoundtrip(m);
  std::stringstream ss;
  EXPECT_TRUE(m.getAllFieldDescriptions(ss));
  EXPECT_TRUE(m.getAllFieldValues(ss));
}

TEST(gnss_messages, NovatelIonutc)
{
  Message_NV_OEM6_IONUTC m;
  m.fields.a0 = 1.0;
  m.fields.utc_wn = 42;
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelGenericFrame)
{
  Message_NV_OEM6_GENERIC_FRAME m;
  m.header.msg_id = 55;
  m.msg_body = {1, 2, 3, 4, 5};
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelGenericFrameEmptyBody)
{
  Message_NV_OEM6_GENERIC_FRAME m;
  m.msg_body.clear();
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelGenericShortFrame)
{
  Message_NV_OEM6_GENERIC_SHORT_FRAME m;
  m.header.msg_id = 77;
  m.msg_body = {9, 8, 7};
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelVersion)
{
  Message_NV_OEM6_VERSION m;
  Message_NV_OEM6_VERSION::TComponentVersion comp{};
  comp.type = 1;
  m.components.push_back(comp);
  m.num_comps = static_cast<uint32_t>(m.components.size());
  checkRoundtrip(m);
}

TEST(gnss_messages, NovatelRangecmp)
{
  Message_NV_OEM6_RANGECMP m;
  Message_NV_OEM6_RANGECMP::TCompressedRangeLog log{};
  m.obs_data.push_back(log);
  m.num_obs = static_cast<uint32_t>(m.obs_data.size());
  checkRoundtrip(m);
}

TEST(gnss_messages, EnumToStrKnownAndUnknown)
{
  EXPECT_EQ(
      nv_oem6_solution_status::enum2str(nv_oem6_solution_status::SOL_COMPUTED), "SOL_COMPUTED");
  EXPECT_EQ(nv_oem6_solution_status::enum2str(-1), "???");

  EXPECT_EQ(nv_oem6_position_type::enum2str(nv_oem6_position_type::SINGLE), "SINGLE");
  EXPECT_EQ(nv_oem6_position_type::enum2str(-1), "???");

  EXPECT_EQ(
      nv_oem6_ins_status_type::enum2str(nv_oem6_ins_status_type::INS_ALIGNING), "INS_ALIGNING");
  EXPECT_EQ(nv_oem6_ins_status_type::enum2str(-1), "???");
}

TEST(gnss_messages, UTCTimeEqualityAndTimestamp)
{
  UTC_time a;
  a.hour = 10;
  a.minute = 20;
  a.sec = 30.0;
  UTC_time b = a;
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  b.sec = 31.0;
  EXPECT_TRUE(a != b);
  EXPECT_FALSE(a == b);

  const auto ts = a.getAsTimestamp(mrpt::Clock::now());
  mrpt::system::TTimeParts parts;
  mrpt::system::timestampToParts(ts, parts, false);
  EXPECT_EQ(parts.hour, 10);
  EXPECT_EQ(parts.minute, 20);
}

TEST(gnss_messages, ReadAndBuildFromStreamSequence)
{
  mrpt::io::CMemoryStream buf;
  auto arch = archiveFrom(buf);

  Message_NMEA_GGA gga;
  gga.fields.latitude_degrees = 5.0;
  Message_NV_OEM6_BESTPOS bp;
  bp.fields.lat = 6.0;

  gga.writeToStream(arch);
  bp.writeToStream(arch);
  buf.Seek(0);

  std::unique_ptr<gnss_message> m1(gnss_message::readAndBuildFromStream(arch));
  std::unique_ptr<gnss_message> m2(gnss_message::readAndBuildFromStream(arch));
  ASSERT_TRUE(m1);
  ASSERT_TRUE(m2);
  EXPECT_TRUE(m1->isOfType(NMEA_GGA));
  EXPECT_TRUE(m1->isOfClass<Message_NMEA_GGA>());
  EXPECT_TRUE(m2->isOfType(NV_OEM6_BESTPOS));
}

TEST(gnss_messages, ReadAndBuildFromStreamUnknownTypeThrows)
{
  mrpt::io::CMemoryStream buf;
  auto arch = archiveFrom(buf);
  arch.WriteAs<int32_t>(-12345);
  buf.Seek(0);
  EXPECT_THROW(gnss_message::readAndBuildFromStream(arch), std::exception);
}

TEST(gnss_messages, ReadFromStreamWrongTypeThrows)
{
  mrpt::io::CMemoryStream buf;
  auto arch = archiveFrom(buf);
  Message_NMEA_GGA gga;
  gga.writeToStream(arch);
  buf.Seek(0);

  Message_NV_OEM6_BESTPOS bp;
  EXPECT_THROW(bp.readFromStream(arch), std::exception);
}
