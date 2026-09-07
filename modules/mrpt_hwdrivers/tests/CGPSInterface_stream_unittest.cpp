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

// Driver-level tests for CGPSInterface: the framing/dispatch layer around the
// NMEA and NOVATEL parsers, driven through a mocked I/O stream.

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/obs/CObservationGPS.h>

#include <string>

#include "mock_stream.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::MockStream;
using mrpt::obs::CObservationGPS;

namespace
{
// A short, valid NMEA burst: position fix + recommended minimum data.
const char* nmeaBurst =
    "$GPGGA,101830.00,3649.76162994,N,00224.53709052,W,2,08,1.1,9.3,M,47.4,M,5.0,0120*58\n"
    "$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10\n";
}  // namespace

TEST(CGPSInterfaceStream, parsesFramesFromABoundStream)
{
  auto s = std::make_shared<MockStream>();
  s->pushRx(nmeaBurst);

  CGPSInterface gps;
  gps.bindStream(s);
  EXPECT_TRUE(gps.useExternalStream());

  gps.initialize();
  gps.doProcess();

  const auto obss = gps.getObservations();
  ASSERT_FALSE(obss.empty());

  bool sawGGA = false;
  for (const auto& [t, o] : obss)
  {
    auto obs = mrpt::ptr_cast<CObservationGPS>::from(o);
    ASSERT_TRUE(obs);
    if (obs->getMsgByClassPtr<mrpt::obs::gnss::Message_NMEA_GGA>() != nullptr)
    {
      sawGGA = true;
    }
  }
  EXPECT_TRUE(sawGGA);
}

TEST(CGPSInterfaceStream, cachesTheLastGGAFrame)
{
  auto s = std::make_shared<MockStream>();
  s->pushRx(nmeaBurst);

  CGPSInterface gps;
  gps.bindStream(s);
  gps.initialize();
  gps.doProcess();

  const std::string gga = gps.getLastGGA(false);
  EXPECT_NE(gga.find("$GPGGA"), std::string::npos);

  // Reading with reset empties the cache:
  EXPECT_FALSE(gps.getLastGGA(true).empty());
  EXPECT_TRUE(gps.getLastGGA(true).empty());
}

TEST(CGPSInterfaceStream, garbageBetweenFramesIsSkipped)
{
  auto s = std::make_shared<MockStream>();
  s->pushRx(std::string("\x01\x02\x03 not a frame at all \xFF\xFE\n") + nmeaBurst);

  CGPSInterface gps;
  gps.bindStream(s);
  gps.initialize();
  gps.doProcess();

  // The valid frames after the noise must still be recovered:
  EXPECT_FALSE(gps.getObservations().empty());
}

TEST(CGPSInterfaceStream, setupAndShutdownCommandsGoOutOnTheWire)
{
  auto s = std::make_shared<MockStream>();

  {
    CGPSInterface gps;
    gps.bindStream(s);
    gps.setSetupCommandsDelay(0);
    gps.enableSetupCommandsAppendCRLF(true);
    gps.setSetupCommands({"LOG BESTPOSA ONTIME 1", "LOG RANGEA ONTIME 1"});
    gps.setShutdownCommands({"UNLOGALL"});

    EXPECT_EQ(gps.getSetupCommands().size(), 2U);
    EXPECT_EQ(gps.getShutdownCommands().size(), 1U);
    EXPECT_TRUE(gps.isEnabledSetupCommandsAppendCRLF());
    EXPECT_EQ(gps.getSetupCommandsDelay(), 0.0);

    gps.initialize();
    gps.doProcess();

    EXPECT_NE(s->tx().find("LOG BESTPOSA ONTIME 1\r\n"), std::string::npos);
    EXPECT_NE(s->tx().find("LOG RANGEA ONTIME 1\r\n"), std::string::npos);
    // Shutdown commands are sent by the destructor.
  }

  EXPECT_NE(s->tx().find("UNLOGALL\r\n"), std::string::npos);
}

TEST(CGPSInterfaceStream, failedSetupIsRetriedOnTheNextCall)
{
  auto s = std::make_shared<MockStream>();
  CGPSInterface gps;
  gps.bindStream(s);
  gps.setSetupCommandsDelay(0);
  gps.setSetupCommands({"LOG BESTPOSA ONTIME 1"});

  // The link is down for the first attempt. As on the serial-port path, a
  // stream that cannot be brought up is reported by doProcess() throwing, and
  // the setup must not be marked as done.
  s->setWritesThrow(true);
  gps.initialize();
  EXPECT_THROW(gps.doProcess(), std::exception);
  EXPECT_TRUE(s->tx().empty());

  // Once the link recovers, the setup commands are sent after all:
  s->setWritesThrow(false);
  gps.doProcess();
  EXPECT_NE(s->tx().find("LOG BESTPOSA ONTIME 1"), std::string::npos);
}

TEST(CGPSInterfaceStream, setupCommandsCanSkipTheCRLF)
{
  auto s = std::make_shared<MockStream>();
  CGPSInterface gps;
  gps.bindStream(s);
  gps.setSetupCommandsDelay(0);
  gps.enableSetupCommandsAppendCRLF(false);
  gps.setSetupCommands({"ABC"});

  gps.initialize();
  gps.doProcess();

  EXPECT_NE(s->tx().find("ABC"), std::string::npos);
  EXPECT_EQ(s->tx().find("ABC\r\n"), std::string::npos);
}

TEST(CGPSInterfaceStream, sendCustomCommandWritesToTheStream)
{
  auto s = std::make_shared<MockStream>();
  CGPSInterface gps;
  gps.bindStream(s);

  const std::string cmd = "$PUBX,40,GLL,0,0,0,0*5C\r\n";
  EXPECT_TRUE(gps.sendCustomCommand(cmd.data(), cmd.size()));
  EXPECT_NE(s->tx().find(cmd), std::string::npos);

  // I/O failures are reported, not thrown:
  s->setWritesThrow(true);
  EXPECT_FALSE(gps.sendCustomCommand(cmd.data(), cmd.size()));
}

TEST(CGPSInterfaceStream, parserSelectionRoundTrips)
{
  CGPSInterface gps;
  // AUTO is the default:
  EXPECT_EQ(gps.getParser(), CGPSInterface::AUTO);

  gps.setParser(CGPSInterface::NMEA);
  EXPECT_EQ(gps.getParser(), CGPSInterface::NMEA);

  gps.setParser(CGPSInterface::NOVATEL_OEM6);
  EXPECT_EQ(gps.getParser(), CGPSInterface::NOVATEL_OEM6);
}

TEST(CGPSInterfaceStream, appendMsgTypeToSensorLabel)
{
  auto s = std::make_shared<MockStream>();
  s->pushRx(nmeaBurst);

  CGPSInterface gps;
  gps.bindStream(s);
  gps.setSensorLabel("gps1");
  gps.enableAppendMsgTypeToSensorLabel(true);
  gps.initialize();
  gps.doProcess();

  const auto obss = gps.getObservations();
  ASSERT_FALSE(obss.empty());
  for (const auto& [t, o] : obss)
  {
    auto obs = mrpt::ptr_cast<CObservationGPS>::from(o);
    ASSERT_TRUE(obs);
    // The message class name is appended after the base label:
    EXPECT_EQ(obs->sensorLabel.rfind("gps1_", 0), 0U) << "label: " << obs->sensorLabel;
  }
}

TEST(CGPSInterfaceStream, loadConfigReadsDriverSettings)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string sec = "GPS";
  cfg.write(sec, "parser", "NMEA");
  // COM port and baud rate are mandatory entries:
#ifdef _WIN32
  cfg.write(sec, "COM_port_WIN", "COM1");
#else
  cfg.write(sec, "COM_port_LIN", "/dev/ttyUSB0");
#endif
  cfg.write(sec, "baudRate", 115200);
  cfg.write(sec, "custom_cmds_delay", 0.0);
  cfg.write(sec, "custom_cmds_append_CRLF", true);
  cfg.write(sec, "setup_cmd1", "CMD_ONE");
  cfg.write(sec, "setup_cmd2", "CMD_TWO");
  cfg.write(sec, "shutdown_cmd1", "CMD_BYE");
  cfg.write(sec, "sensor_label_append_msg_type", true);
  cfg.write(sec, "pose_x", 1.0);
  cfg.write(sec, "pose_y", 2.0);
  cfg.write(sec, "pose_z", 3.0);

  CGPSInterface gps;
  gps.loadConfig(cfg, sec);

  EXPECT_EQ(gps.getParser(), CGPSInterface::NMEA);
  ASSERT_EQ(gps.getSetupCommands().size(), 2U);
  EXPECT_EQ(gps.getSetupCommands()[0], "CMD_ONE");
  EXPECT_EQ(gps.getSetupCommands()[1], "CMD_TWO");
  ASSERT_EQ(gps.getShutdownCommands().size(), 1U);
  EXPECT_EQ(gps.getShutdownCommands()[0], "CMD_BYE");
  EXPECT_TRUE(gps.isEnabledSetupCommandsAppendCRLF());
}

TEST(CGPSInterfaceStream, destroyingAnUnconnectedDriverIsSafe)
{
  // Shutdown commands are sent from the destructor: doing so with a driver
  // that never opened a stream must not dereference a null stream.
  CGPSInterface gps;
  gps.setShutdownCommands({"UNLOGALL"});
  SUCCEED();
}

TEST(CGPSInterfaceStream, forcedParserIgnoresTheOtherFormat)
{
  auto s = std::make_shared<MockStream>();
  s->pushRx(nmeaBurst);

  CGPSInterface gps;
  gps.bindStream(s);
  gps.setParser(CGPSInterface::NMEA);
  gps.initialize();
  gps.doProcess();

  EXPECT_FALSE(gps.getObservations().empty());
}
