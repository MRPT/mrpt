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
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/serialization/CArchive.h>

#include <cstring>
#include <sstream>

using namespace mrpt::obs;
using namespace mrpt::obs::gnss;

TEST(CObservationGPS, SetGetMsgByClassAndType)
{
  CObservationGPS obs;
  EXPECT_FALSE(obs.hasMsgType(NMEA_GGA));
  EXPECT_FALSE((obs.hasMsgClass<Message_NMEA_GGA>()));

  Message_NMEA_GGA gga;
  gga.fields.latitude_degrees = 36.5;
  obs.setMsg(gga);

  EXPECT_TRUE(obs.hasMsgType(NMEA_GGA));
  EXPECT_TRUE((obs.hasMsgClass<Message_NMEA_GGA>()));
  EXPECT_TRUE(obs.has_GGA_datum());
  EXPECT_FALSE(obs.has_RMC_datum());

  auto& gotRef = obs.getMsgByClass<Message_NMEA_GGA>();
  EXPECT_NEAR(gotRef.fields.latitude_degrees, 36.5, 1e-9);

  auto* gotPtr = obs.getMsgByClassPtr<Message_NMEA_GGA>();
  ASSERT_TRUE(gotPtr);
  EXPECT_NEAR(gotPtr->fields.latitude_degrees, 36.5, 1e-9);

  auto* notPresent = obs.getMsgByClassPtr<Message_NMEA_RMC>();
  EXPECT_FALSE(notPresent);

  auto* byType = obs.getMsgByType(NMEA_GGA);
  ASSERT_TRUE(byType);
  EXPECT_THROW(obs.getMsgByType(NMEA_RMC), std::exception);
  EXPECT_THROW((obs.getMsgByClass<Message_NMEA_RMC>()), std::exception);

  const CObservationGPS& cobs = obs;
  EXPECT_TRUE(cobs.getMsgByType(NMEA_GGA));
  EXPECT_THROW((cobs.getMsgByClass<Message_NMEA_RMC>()), std::exception);
  EXPECT_TRUE((cobs.getMsgByClassPtr<Message_NMEA_GGA>()));
}

TEST(CObservationGPS, ClearAndCovarianceEnu)
{
  CObservationGPS obs;
  Message_NMEA_GGA gga;
  obs.setMsg(gga);
  obs.covariance_enu = mrpt::math::CMatrixDouble33::Identity();
  EXPECT_TRUE(obs.hasMsgType(NMEA_GGA));

  obs.clear();
  EXPECT_FALSE(obs.hasMsgType(NMEA_GGA));
  EXPECT_FALSE(obs.covariance_enu.has_value());
}

TEST(CObservationGPS, InferFixTypeFromGgaQuality)
{
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(0), GnssFixType::NO_FIX);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(1), GnssFixType::AUTONOMOUS);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(2), GnssFixType::DGPS);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(3), GnssFixType::AUTONOMOUS);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(4), GnssFixType::RTK_FIXED);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(5), GnssFixType::RTK_FLOAT);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(6), GnssFixType::DEAD_RECKONING);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(8), GnssFixType::SIMULATION);
  EXPECT_EQ(CObservationGPS::inferFixTypeFromGgaQuality(200), GnssFixType::UNKNOWN);
}

TEST(CObservationGPS, GetBestFixType)
{
  CObservationGPS obs;
  EXPECT_EQ(obs.getBestFixType(), GnssFixType::UNKNOWN);

  Message_NMEA_GGA gga;
  gga.fields.fix_quality = 4;
  obs.setMsg(gga);
  EXPECT_EQ(obs.getBestFixType(), GnssFixType::RTK_FIXED);

  obs.fix_type = GnssFixType::PPP;
  EXPECT_EQ(obs.getBestFixType(), GnssFixType::PPP);
}

TEST(CObservationGPS, GnssServiceBitmask)
{
  const auto mask = GnssService::GPS | GnssService::GALILEO;
  EXPECT_TRUE(hasService(mask, GnssService::GPS));
  EXPECT_TRUE(hasService(mask, GnssService::GALILEO));
  EXPECT_FALSE(hasService(mask, GnssService::GLONASS));
  EXPECT_FALSE(hasService(mask, GnssService::BEIDOU));
}

TEST(CObservationGPS, DumpToStreamAllBranches)
{
  CObservationGPS obs;
  obs.fix_type = GnssFixType::RTK_FLOAT;
  obs.gnss_service_mask =
      GnssService::GPS | GnssService::GLONASS | GnssService::BEIDOU | GnssService::GALILEO;
  obs.covariance_enu = mrpt::math::CMatrixDouble33();
  obs.covariance_enu->setDiagonal(3, 0.01);
  Message_NMEA_GGA gga;
  obs.setMsg(gga);

  std::stringstream ss;
  obs.dumpToStream(ss);
  const std::string txt = ss.str();
  EXPECT_NE(txt.find("RTK_FLOAT"), std::string::npos);
  EXPECT_NE(txt.find("GPS"), std::string::npos);
  EXPECT_NE(txt.find("GLONASS"), std::string::npos);
  EXPECT_NE(txt.find("BeiDou"), std::string::npos);
  EXPECT_NE(txt.find("Galileo"), std::string::npos);
  EXPECT_NE(txt.find("ENU covariance"), std::string::npos);

  std::stringstream ss2;
  obs.dumpToConsole(ss2);
  EXPECT_FALSE(ss2.str().empty());
}

TEST(CObservationGPS, DumpToStreamNoCovarianceNoServiceUnknownFixType)
{
  CObservationGPS obs;
  obs.fix_type = static_cast<GnssFixType>(250);  // out-of-range -> "?"
  std::stringstream ss;
  obs.dumpToStream(ss);
  const std::string txt = ss.str();
  EXPECT_NE(txt.find("?"), std::string::npos);
  EXPECT_NE(txt.find("(none)"), std::string::npos);
}

TEST(CObservationGPS, GetDescriptionAsText)
{
  CObservationGPS obs;
  obs.sensorPose = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
  obs.originalReceivedTimestamp = mrpt::Clock::now();
  std::stringstream ss;
  obs.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CObservationGPS, SerializationRoundtripCurrentVersion)
{
  CObservationGPS obs1;
  obs1.sensorPose = mrpt::poses::CPose3D(0.1, 0.2, 0.3, 0.01, 0.02, 0.03);
  obs1.has_satellite_timestamp = true;
  obs1.covariance_enu = mrpt::math::CMatrixDouble33();
  obs1.covariance_enu->setDiagonal(3, 0.5);
  obs1.fix_type = GnssFixType::RTK_FIXED;
  obs1.gnss_service_mask = GnssService::GPS;
  Message_NMEA_GGA gga;
  gga.fields.latitude_degrees = 10.0;
  obs1.setMsg(gga);
  Message_NMEA_RMC rmc;
  obs1.setMsg(rmc);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << obs1;
  buf.Seek(0);

  CObservationGPS obs2;
  arch >> obs2;

  EXPECT_EQ(obs2.has_satellite_timestamp, obs1.has_satellite_timestamp);
  ASSERT_TRUE(obs2.covariance_enu.has_value());
  EXPECT_EQ(obs2.fix_type, obs1.fix_type);
  EXPECT_EQ(obs2.gnss_service_mask, obs1.gnss_service_mask);
  EXPECT_TRUE(obs2.hasMsgType(NMEA_GGA));
  EXPECT_TRUE(obs2.hasMsgType(NMEA_RMC));
  EXPECT_NEAR(obs2.getMsgByClass<Message_NMEA_GGA>().fields.latitude_degrees, 10.0, 1e-9);
}

namespace
{
// Manually emits the same "class header" that CArchive::WriteObject() writes
// before delegating to serializeTo(): class name + explicit version byte.
// This lets us test old serializeFrom() version branches without needing an
// archived sample file for each of them.
void writeGpsObjectHeader(mrpt::serialization::CArchive& arch, uint8_t version)
{
  const char* className = "CObservationGPS";
  const auto len = static_cast<int8_t>(strlen(className) | 0x80);
  arch << len;
  arch.WriteBuffer(className, strlen(className));
  arch << version;
}

void writeGpsObjectFooter(mrpt::serialization::CArchive& arch)
{
  arch << static_cast<uint8_t>(0x88);  // SERIALIZATION_END_FLAG
}
}  // namespace

TEST(CObservationGPS, DeserializeLegacyVersion0)
{
  // Version 0 is the oldest format: just two optional raw NMEA GGA/RMC blocks.
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);

  Message_NMEA_GGA gga;
  gga.fields.latitude_degrees = 5.5;
  gga.fields.longitude_degrees = -3.3;

  Message_NMEA_RMC rmc;
  rmc.fields.latitude_degrees = 1.0;

  writeGpsObjectHeader(arch, 0);
  arch << true;  // has_GGA_datum_
  arch.WriteBuffer(&gga.fields, sizeof(gga.fields));
  arch << true;  // has_RMC_datum_
  arch.WriteBuffer(&rmc.fields, sizeof(rmc.fields));
  writeGpsObjectFooter(arch);
  buf.Seek(0);

  CObservationGPS obs;
  arch >> obs;

  ASSERT_TRUE(obs.has_GGA_datum());
  ASSERT_TRUE(obs.has_RMC_datum());
  EXPECT_NEAR(obs.getMsgByClass<Message_NMEA_GGA>().fields.latitude_degrees, 5.5, 1e-9);
  EXPECT_NEAR(obs.getMsgByClass<Message_NMEA_RMC>().fields.latitude_degrees, 1.0, 1e-9);
}

TEST(CObservationGPS, DeserializeLegacyVersion0NoData)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeGpsObjectHeader(arch, 0);
  arch << false;  // has_GGA_datum_
  arch << false;  // has_RMC_datum_
  writeGpsObjectFooter(arch);
  buf.Seek(0);

  CObservationGPS obs;
  arch >> obs;
  EXPECT_FALSE(obs.has_GGA_datum());
  EXPECT_FALSE(obs.has_RMC_datum());
}

TEST(CObservationGPS, DeserializeLegacyVersion1Minimal)
{
  // Version 1: no timestamp (added in v3), no sensorLabel (added when >1), no
  // custom sensorPose (added in v4), no PZS (v5+), no SATS entry (v7+).
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeGpsObjectHeader(arch, 1);
  arch << false;  // has_GGA_datum_
  arch << false;  // has_RMC_datum_
  writeGpsObjectFooter(arch);
  buf.Seek(0);

  CObservationGPS obs;
  arch >> obs;
  EXPECT_FALSE(obs.has_GGA_datum());
  EXPECT_FALSE(obs.has_RMC_datum());
  EXPECT_EQ(obs.sensorLabel, "");
  EXPECT_FALSE(obs.hasMsgType(TOPCON_SATS));
}

TEST(CObservationGPS, DeserializeLegacyVersion9Full)
{
  // Version 9: exercises the full GGA/RMC/PZS(+cartesian/cov/stats)/SATS
  // (empty) parsing paths, the most feature-complete of the legacy formats.
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeGpsObjectHeader(arch, 9);

  arch << mrpt::Clock::now();  // timestamp (version>=3)

  // GGA:
  arch << true;
  arch << static_cast<uint8_t>(10) << static_cast<uint8_t>(20) << 30.5;  // UTC h,m,s
  arch << 36.5 << -4.5;                                                  // lat, lon
  arch << static_cast<uint8_t>(1);                                       // fix_quality
  arch << 100.0;                                                         // altitude
  arch << 1.0 << 2.0 << 3.0;         // geoidal, orthometric, corrected (v>=9)
  arch << static_cast<uint32_t>(7);  // satellitesUsed
  arch << true;                      // thereis_HDOP
  arch << 1.5f;                      // HDOP

  // RMC:
  arch << true;
  arch << static_cast<uint8_t>(11) << static_cast<uint8_t>(22) << 33.0;  // UTC
  arch << static_cast<int8_t>('A');                                      // validity_char
  arch << 1.0 << 2.0;                                                    // lat, lon
  arch << 5.0 << 90.0;                                                   // speed, direction

  arch << std::string("gps_label");                // sensorLabel (version>1)
  arch << mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);  // sensorPose (version>=4)

  // PZS (version>=5):
  arch << true;                          // has_PZS_datum_
  arch << 10.0 << 20.0 << 30.0 << 31.0;  // lat,lon,height,RTK_height
  arch << 0.5f;                          // PSigma
  arch << 45.0;                          // angle_transmitter
  arch << static_cast<uint8_t>(1) << static_cast<uint8_t>(2) << static_cast<uint8_t>(90)
       << static_cast<uint8_t>(80) << static_cast<uint8_t>(0);  // nId,Fix,TX,RX,error
  // version>=6:
  arch << true;               // hasCartesianPosVel
  arch << 1.0 << 2.0 << 3.0;  // cartesian x,y,z
  arch << 0.1 << 0.2 << 0.3;  // cartesian vx,vy,vz
  arch << true;               // hasPosCov
  {
    mrpt::math::CMatrixFloat44 m;
    m.setIdentity();
    arch.WriteBufferFixEndianness(&m(0, 0), m.size());
  }
  arch << true;  // hasVelCov
  {
    mrpt::math::CMatrixFloat44 m;
    m.setIdentity();
    arch.WriteBufferFixEndianness(&m(0, 0), m.size());
  }
  arch << true;                                                // hasStats
  arch << static_cast<uint8_t>(8) << static_cast<uint8_t>(4);  // GPS/GLONASS sats used
  arch << static_cast<uint8_t>(50);                            // stats_rtk_fix_progress (v>=8)

  // SATS (version>=7): present but empty to avoid the "broken" read path:
  arch << false;  // has_SATS_datum_

  writeGpsObjectFooter(arch);
  buf.Seek(0);

  CObservationGPS obs;
  arch >> obs;

  ASSERT_TRUE(obs.has_GGA_datum());
  ASSERT_TRUE(obs.has_RMC_datum());
  EXPECT_NEAR(obs.getMsgByClass<Message_NMEA_GGA>().fields.latitude_degrees, 36.5, 1e-9);
  EXPECT_NEAR(obs.getMsgByClass<Message_NMEA_GGA>().fields.geoidal_distance, 1.0, 1e-6);
  EXPECT_EQ(obs.sensorLabel, "gps_label");
  ASSERT_TRUE(obs.hasMsgType(TOPCON_PZS));
  EXPECT_NEAR(obs.getMsgByClass<Message_TOPCON_PZS>().latitude_degrees, 10.0, 1e-9);
  EXPECT_TRUE(obs.hasMsgType(TOPCON_SATS));
}

TEST(CObservationGPS, DeserializeLegacyVersion9SatsThrows)
{
  // The TOPCON_SATS reader in the legacy path is intentionally broken and
  // throws if a SATS datum is actually present in the stream.
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeGpsObjectHeader(arch, 9);

  arch << mrpt::Clock::now();
  arch << false;  // has_GGA_datum_
  arch << false;  // has_RMC_datum_
  arch << std::string("");
  arch << mrpt::poses::CPose3D();
  arch << false;  // has_PZS_datum_
  arch << true;   // has_SATS_datum_ -> triggers the broken/throwing path

  buf.Seek(0);

  CObservationGPS obs;
  EXPECT_THROW(arch >> obs, std::exception);
}
