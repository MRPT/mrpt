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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "mock_stream.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::MockStream;

namespace
{
// SCIP2.0 replies are: <command echo><status0><status1><sum><LF>, optionally
// followed by data lines "<payload><sum><LF>", and closed by a final LF.
// The driver strips the per-line sum char, so its value is irrelevant here.
std::string scipReply(const std::string& cmdEcho, const std::string& status = "00")
{
  return cmdEcho + status + "P" + "\x0A" + "\x0A";
}

std::string scipReplyWithData(
    const std::string& cmdEcho, const std::string& payload, const std::string& status = "00")
{
  return cmdEcho + status + "P" + "\x0A" + payload + "P" + "\x0A" + "\x0A";
}

/** A URG-04LX-like answer to the "PP" (sensor parameters) command, reporting
 *  the scan window [first,last] out of 1024 steps per 360 deg. */
std::string sensorParamsPayload(unsigned int first, unsigned int last)
{
  char buf[256];
  ::snprintf(
      buf, sizeof(buf),
      "MODL:URG-04LX;DMIN:20;DMAX:5600;ARES:1024;AMIN:%u;AMAX:%u;AFRT:384;SCAN:600;", first, last);
  return buf;
}

/** The continuous-scan command the driver issues for a given window. */
std::string measCommandFor(unsigned int first, unsigned int last, bool intensity = false)
{
  char cmd[64];
  ::snprintf(cmd, sizeof(cmd), "M%c%04u%04u01000\x0A", intensity ? 'E' : 'D', first, last);
  return cmd;
}

/** Scripts a mock stream with the whole turnOn() handshake. */
void scriptTurnOnSession(MockStream& s, unsigned int first, unsigned int last)
{
  s.replyTo("SCIP2.0\x0A", scipReply("SCIP2.0\x0A"));
  s.replyTo("BM\x0A", scipReply("BM\x0A"));
  s.replyTo("QT\x0A", scipReply("QT\x0A"));
  s.replyTo("HS0\x0A", scipReply("HS0\x0A"));
  s.replyTo("HS1\x0A", scipReply("HS1\x0A"));
  s.replyTo("PP\x0A", scipReplyWithData("PP\x0A", sensorParamsPayload(first, last)));
  s.replyTo("VV\x0A", scipReplyWithData("VV\x0A", "VEND:Hokuyo;PROD:URG-04LX;FIRM:3.3.00;"));

  for (bool intensity : {false, true})
  {
    const std::string cmd = measCommandFor(first, last, intensity);
    s.replyTo(cmd, scipReply(cmd));
  }
}

/** Builds one continuous-mode MD scan reply: a 4-char timestamp followed by
 *  `nRanges` 3-char encoded ranges, all reading `range_mm`. */
std::string makeScanReply(
    const std::string& mdCmd, int nRanges, int range_mm, unsigned int timestamp = 0)
{
  std::string payload;
  payload += static_cast<char>(0x30 + ((timestamp >> 18) & 0x3F));
  payload += static_cast<char>(0x30 + ((timestamp >> 12) & 0x3F));
  payload += static_cast<char>(0x30 + ((timestamp >> 6) & 0x3F));
  payload += static_cast<char>(0x30 + (timestamp & 0x3F));

  for (int i = 0; i < nRanges; i++)
  {
    payload += static_cast<char>(0x30 + ((range_mm >> 12) & 0x3F));
    payload += static_cast<char>(0x30 + ((range_mm >> 6) & 0x3F));
    payload += static_cast<char>(0x30 + (range_mm & 0x3F));
  }
  return scipReplyWithData(mdCmd, payload, "99");
}

// Scan window kept small so the expected byte counts stay easy to follow.
constexpr unsigned int FIRST_STEP = 44;
constexpr unsigned int LAST_STEP = 143;
constexpr int N_RANGES = static_cast<int>(LAST_STEP - FIRST_STEP + 1);
}  // namespace

TEST(CHokuyoURG, turnOnRunsTheWholeSCIP20Handshake)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, 44, 725);

  CHokuyoURG laser;
  laser.bindIO(s);

  EXPECT_TRUE(laser.turnOn());

  const std::string& sent = s->tx();
  EXPECT_NE(sent.find("SCIP2.0\x0A"), std::string::npos);
  EXPECT_NE(sent.find("BM\x0A"), std::string::npos);
  EXPECT_NE(sent.find("PP\x0A"), std::string::npos);
  EXPECT_NE(sent.find("VV\x0A"), std::string::npos);
  // The scan window must be the one reported by the "PP" reply (AMIN/AMAX):
  EXPECT_NE(sent.find("MD00440725"), std::string::npos);
}

TEST(CHokuyoURG, decodeScanOverMockedStream)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, FIRST_STEP, LAST_STEP);

  CHokuyoURG laser;
  laser.bindIO(s);
  ASSERT_TRUE(laser.turnOn());

  s->pushRx(makeScanReply(measCommandFor(FIRST_STEP, LAST_STEP), N_RANGES, 1234));

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(hwError);
  ASSERT_TRUE(thereIs);
  ASSERT_EQ(obs.getScanSize(), static_cast<size_t>(N_RANGES));
  EXPECT_TRUE(obs.rightToLeft);
  // maxRange comes from the DMAX:5600 field of the "PP" reply:
  EXPECT_NEAR(obs.maxRange, 5.6f, 1e-4f);
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    EXPECT_NEAR(obs.getScanRange(i), 1.234f, 1e-4f) << "at i=" << i;
    EXPECT_TRUE(obs.getScanRangeValidity(i)) << "at i=" << i;
  }
  // aperture = nRanges * 2*pi / ARES
  EXPECT_NEAR(obs.aperture, N_RANGES * 2 * M_PI / 1024.0, 1e-4);
}

TEST(CHokuyoURG, truncatedScanIsReportedAsHardwareError)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, FIRST_STEP, LAST_STEP);

  CHokuyoURG laser;
  laser.bindIO(s);
  ASSERT_TRUE(laser.turnOn());

  // One range short of what the driver asked for:
  s->pushRx(makeScanReply(measCommandFor(FIRST_STEP, LAST_STEP), N_RANGES - 1, 1000));

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(thereIs);
  EXPECT_TRUE(hwError);
}

TEST(CHokuyoURG, readingsBelowMinimumRangeAreMarkedInvalid)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, FIRST_STEP, LAST_STEP);

  CHokuyoURG laser;
  laser.bindIO(s);
  ASSERT_TRUE(laser.turnOn());

  // 10 mm is below the 20 mm the driver treats as the valid minimum:
  s->pushRx(makeScanReply(measCommandFor(FIRST_STEP, LAST_STEP), N_RANGES, 10));

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.doProcessSimple(thereIs, obs, hwError);

  ASSERT_TRUE(thereIs);
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    EXPECT_FALSE(obs.getScanRangeValidity(i)) << "at i=" << i;
  }
}

TEST(CHokuyoURG, deviceErrorStatusAbortsTurnOn)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, 44, 725);
  // Status "11" is an error reply to the "switch laser on" command:
  s->replyTo("BM\x0A", scipReply("BM\x0A", "11"));

  CHokuyoURG laser;
  laser.bindIO(s);

  EXPECT_FALSE(laser.turnOn());
}

TEST(CHokuyoURG, motorSpeedOutOfRangeAbortsTurnOn)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string sec = "HOKUYO";
  cfg.write(sec, "IP_DIR", "192.168.0.10");
  cfg.write(sec, "PORT_DIR", 10940);
  // Valid motor speeds are 540..600 rpm:
  cfg.write(sec, "HOKUYO_motorSpeed_rpm", 300);

  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, 44, 725);

  CHokuyoURG laser;
  laser.loadConfig(cfg, sec);
  laser.bindIO(s);

  EXPECT_FALSE(laser.turnOn());
}

TEST(CHokuyoURG, motorSpeedInRangeIsSentToTheDevice)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string sec = "HOKUYO";
  cfg.write(sec, "IP_DIR", "192.168.0.10");
  cfg.write(sec, "PORT_DIR", 10940);
  cfg.write(sec, "HOKUYO_motorSpeed_rpm", 540);

  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, 44, 725);
  // (600-540)/6 = 10
  s->replyTo("CR10\x0A", scipReply("CR10\x0A"));

  CHokuyoURG laser;
  laser.loadConfig(cfg, sec);
  laser.bindIO(s);

  EXPECT_TRUE(laser.turnOn());
  EXPECT_NE(s->tx().find("CR10\x0A"), std::string::npos);
}

TEST(CHokuyoURG, intensityModeAsksForMEScans)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, 44, 725);

  CHokuyoURG laser;
  laser.bindIO(s);
  EXPECT_TRUE(laser.setIntensityMode(true));

  EXPECT_TRUE(laser.turnOn());
  // Intensity mode maps to the "ME" command instead of "MD":
  EXPECT_NE(s->tx().find("ME00440725"), std::string::npos);
}

TEST(CHokuyoURG, silentDeviceYieldsNoScanAndNoCrash)
{
  auto s = std::make_shared<MockStream>();
  scriptTurnOnSession(*s, FIRST_STEP, LAST_STEP);

  CHokuyoURG laser;
  laser.bindIO(s);
  ASSERT_TRUE(laser.turnOn());

  // The device goes quiet:
  s->setReadsReturnNothing(true);

  bool thereIs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser.doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(thereIs);
}

TEST(CHokuyoURG, unconfiguredDriverThrows)
{
  // Neither bindIO() nor a COM port / IP was given: that is a configuration
  // error, and is reported as such rather than as a hardware failure.
  CHokuyoURG laser;

  bool thereIs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  EXPECT_THROW(laser.doProcessSimple(thereIs, obs, hwError), std::exception);
}

TEST(CHokuyoURG, loadConfigRejectsAmbiguousConnectionSettings)
{
  {
    // Neither serial port nor IP given:
    mrpt::config::CConfigFileMemory cfg;
    cfg.write("HOKUYO", "sensorLabel", "laser1");
    CHokuyoURG laser;
    EXPECT_THROW(laser.loadConfig(cfg, "HOKUYO"), std::exception);
  }
  {
    // Both given at once:
    mrpt::config::CConfigFileMemory cfg;
    cfg.write("HOKUYO", "COM_port_LIN", "/dev/ttyACM0");
    cfg.write("HOKUYO", "COM_port_WIN", "COM3");
    cfg.write("HOKUYO", "IP_DIR", "192.168.0.10");
    cfg.write("HOKUYO", "PORT_DIR", 10940);
    CHokuyoURG laser;
    EXPECT_THROW(laser.loadConfig(cfg, "HOKUYO"), std::exception);
  }
  {
    // Ethernet without a port number:
    mrpt::config::CConfigFileMemory cfg;
    cfg.write("HOKUYO", "IP_DIR", "192.168.0.10");
    cfg.write("HOKUYO", "PORT_DIR", 0);
    CHokuyoURG laser;
    EXPECT_THROW(laser.loadConfig(cfg, "HOKUYO"), std::exception);
  }
}

TEST(CHokuyoURG, loadConfigReadsSensorSettings)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string sec = "HOKUYO";
#ifdef _WIN32
  cfg.write(sec, "COM_port_WIN", "COM3");
#else
  cfg.write(sec, "COM_port_LIN", "/dev/ttyACM0");
#endif
  cfg.write(sec, "HOKUYO_motorSpeed_rpm", 600);
  cfg.write(sec, "pose_x", 0.5);
  cfg.write(sec, "pose_y", -0.25);
  cfg.write(sec, "pose_z", 0.1);
  cfg.write(sec, "pose_yaw", 90.0);
  cfg.write(sec, "scan_interval", 3);
  cfg.write(sec, "intensity", true);

  CHokuyoURG laser;
  laser.loadConfig(cfg, sec);

  EXPECT_EQ(laser.getScanInterval(), 3U);
#ifdef _WIN32
  EXPECT_EQ(laser.getSerialPort(), "COM3");
#else
  EXPECT_EQ(laser.getSerialPort(), "/dev/ttyACM0");
#endif
}
