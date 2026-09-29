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
#include <mrpt/hwdrivers/CCANBusReader.h>
#include <mrpt/obs/CObservationCANBusJ1939.h>

#include <memory>

#include "pty_device.h"

#ifdef MRPT_HWDRIVERS_TESTS_HAVE_PTY

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::PtyDevice;

namespace
{
mrpt::config::CConfigFileMemory canConfig(
    const std::string& port, const std::string& canSpeed = "250000")
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("CAN", "COM_port_LIN", port);
  cfg.write("CAN", "COM_baudRate", "57600");
  cfg.write("CAN", "CANBusSpeed", canSpeed);
  cfg.write("CAN", "nTries_command", "2");
  return cfg;
}

/** Makes `dev` behave as a CAN converter: it acknowledges the setup commands
 * and, once the channel has been opened ("O"), it sends `frame`. */
void playConverter(PtyDevice& dev, const std::string& frame)
{
  // A read may end in the middle of a command: keep what is left for the next
  auto pending = std::make_shared<std::string>();
  dev.startResponder(
      [frame, pending](const std::string& req) -> std::string
      {
        *pending += req;
        std::string reply;
        size_t end;
        while ((end = pending->find('\r')) != std::string::npos)
        {
          const std::string cmd = pending->substr(0, end);
          pending->erase(0, end + 1);
          if (cmd == "V")
          {
            reply += "V1010\r";
          }
          else if (cmd.size() == 2 && cmd[0] == 'S')
          {
            reply += "\r";
          }
          else if (cmd == "O")
          {
            reply += "\r" + frame;
          }
        }
        return reply;
      });
}

// A J1939 frame: 'T', 8 hex digits of identifier (priority, PDU format, PDU
// specific, source address), the data length, the data in hex, and a CR.
const std::string GOOD_FRAME =
    "T18FEF1008"
    "1122334455667788"
    "\r";
}  // namespace

TEST(CCANBusReader, ReadsAJ1939Frame)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  playConverter(dev, GOOD_FRAME);

  CCANBusReader sensor;
  const auto cfg = canConfig(dev.slaveName());
  sensor.loadConfig(cfg, "CAN");

  bool thereIsObs = false;
  bool hwError = true;
  mrpt::obs::CObservationCANBusJ1939 obs;
  sensor.doProcessSimple(thereIsObs, obs, hwError);
  ASSERT_TRUE(thereIsObs);
  EXPECT_FALSE(hwError);

  EXPECT_EQ(obs.m_priority, 6);
  EXPECT_EQ(obs.m_pdu_format, 0xFE);
  EXPECT_EQ(obs.m_pdu_spec, 0xF1);
  EXPECT_EQ(obs.m_src_address, 0x00);
  EXPECT_EQ(obs.m_pgn, 0xFEF1);
  EXPECT_EQ(obs.m_data_length, 8);
  ASSERT_EQ(obs.m_data.size(), 8u);
  EXPECT_EQ(obs.m_data[0], 0x11);
  EXPECT_EQ(obs.m_data[7], 0x88);
  EXPECT_EQ(obs.sensorLabel, "CANBusReader");
  EXPECT_EQ(obs.m_raw_frame.size(), GOOD_FRAME.size());
}

TEST(CCANBusReader, InitializeAndDoProcess)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  // A frame with a shorter payload:
  playConverter(
      dev,
      "T0CF00400"
      "2"
      "ABCD"
      "\r");

  CCANBusReader sensor;
  const auto cfg = canConfig(dev.slaveName(), "500000");
  sensor.loadConfig(cfg, "CAN");
  ASSERT_NO_THROW(sensor.initialize());

  sensor.doProcess();
  const auto lst = sensor.getObservations();
  ASSERT_EQ(lst.size(), 1u);
  const auto obs =
      std::dynamic_pointer_cast<mrpt::obs::CObservationCANBusJ1939>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->m_data_length, 2);
  ASSERT_EQ(obs->m_data.size(), 2u);
  EXPECT_EQ(obs->m_data[0], 0xAB);
  EXPECT_EQ(obs->m_data[1], 0xCD);
  EXPECT_EQ(obs->m_pgn, 0xF004u);
}

TEST(CCANBusReader, SetupErrors)
{
  // An unsupported CAN bus speed cannot be configured:
  {
    PtyDevice dev;
    if (!dev.ok())
    {
      GTEST_SKIP() << "No pseudo-terminal available";
    }
    playConverter(dev, GOOD_FRAME);
    CCANBusReader sensor;
    const auto cfg = canConfig(dev.slaveName(), "12345");
    sensor.loadConfig(cfg, "CAN");
    bool thereIsObs = true;
    bool hwError = false;
    mrpt::obs::CObservationCANBusJ1939 obs;
    sensor.doProcessSimple(thereIsObs, obs, hwError);
    EXPECT_FALSE(thereIsObs);
    EXPECT_TRUE(hwError);

    // A failed setup is not remembered as a working connection: with a valid
    // speed, the next call sets everything up again and reads the frame.
    sensor.setCANReaderSpeed(250000);
    thereIsObs = false;
    hwError = true;
    sensor.doProcessSimple(thereIsObs, obs, hwError);
    EXPECT_TRUE(thereIsObs);
    EXPECT_FALSE(hwError);
  }
  // A port that cannot be opened:
  {
    CCANBusReader sensor;
    const auto cfg = canConfig("/dev/mrpt-no-such-serial-port");
    sensor.loadConfig(cfg, "CAN");
    EXPECT_ANY_THROW(sensor.initialize());

    bool thereIsObs = true;
    bool hwError = false;
    mrpt::obs::CObservationCANBusJ1939 obs;
    sensor.doProcessSimple(thereIsObs, obs, hwError);
    EXPECT_TRUE(hwError);
  }
  // No port configured at all:
  {
    CCANBusReader sensor;
    EXPECT_ANY_THROW(sensor.initialize());
  }
}

TEST(CCANBusReader, Accessors)
{
  CCANBusReader sensor;
  sensor.setSerialPort("ttyS7");
  EXPECT_EQ(sensor.getSerialPort(), "ttyS7");
  sensor.setBaudRate(38400);
  EXPECT_EQ(sensor.getBaudRate(), 38400);
  sensor.setCANReaderSpeed(125000);
  EXPECT_EQ(sensor.getCANReaderSpeed(), 125000u);
  sensor.setCANReaderTimeStamping(true);
  EXPECT_TRUE(sensor.getCANReaderTimeStamping());
  EXPECT_EQ(sensor.getCurrentConnectTry(), 0u);
}

#endif  // MRPT_HWDRIVERS_TESTS_HAVE_PTY
