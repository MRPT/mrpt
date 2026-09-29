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
#include <mrpt/hwdrivers/CRaePID.h>
#include <mrpt/obs/CObservationGasSensors.h>

#include <map>
#include <memory>
#include <mutex>

#include "pty_device.h"

#ifdef MRPT_HWDRIVERS_TESTS_HAVE_PTY

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::PtyDevice;

namespace
{
/** Answers the one-letter PID commands with a canned text. */
struct FakePID
{
  PtyDevice dev;
  std::map<char, std::string> replies;
  std::mutex mtx;

  FakePID()
  {
    dev.startResponder(
        [this](const std::string& req) -> std::string
        {
          std::lock_guard<std::mutex> lck(mtx);
          const auto it = replies.find(req.empty() ? '\0' : req[0]);
          return it == replies.end() ? std::string() : it->second;
        });
  }
  void set(char cmd, const std::string& reply)
  {
    std::lock_guard<std::mutex> lck(mtx);
    replies[cmd] = reply;
  }
};

mrpt::config::CConfigFileMemory pidConfig(const std::string& port)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("PID", "COM_port_PID", port);
  cfg.write("PID", "pose_x", "0");
  cfg.write("PID", "pose_y", "0");
  cfg.write("PID", "pose_z", "0");
  cfg.write("PID", "pose_roll", "0");
  cfg.write("PID", "pose_pitch", "0");
  cfg.write("PID", "pose_yaw", "0");
  return cfg;
}
}  // namespace

TEST(CRaePID, MeasurementAndInformationCommands)
{
  FakePID pid;
  if (!pid.dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  pid.set('R', "1500.0\r\n");
  pid.set('F', "V2.5\r\n");
  pid.set('M', "MiniRAE 3000\r\n");
  pid.set('S', "SN-12345\r\n");
  pid.set('N', "MyPID\r\n");
  pid.set('P', "Sleep...\r\n");
  pid.set('C', "1000 2000 3000\r\n");
  pid.set('L', "100 5\r\n");

  CRaePID sensor;
  const auto cfg = pidConfig(pid.dev.slaveName());
  sensor.loadConfig(cfg, "PID");

  // A measurement (the reading is scaled by 1/1000):
  sensor.doProcess();
  const auto lst = sensor.getObservations();
  ASSERT_EQ(lst.size(), 1u);
  const auto obs =
      std::dynamic_pointer_cast<mrpt::obs::CObservationGasSensors>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->sensorLabel, "RAE_PID");
  ASSERT_EQ(obs->m_readings.size(), 1u);
  ASSERT_EQ(obs->m_readings[0].readingsVoltage.size(), 1u);
  EXPECT_NEAR(obs->m_readings[0].readingsVoltage[0], 1.5, 1e-6);

  // The port is open now: the information commands work.
  EXPECT_EQ(sensor.getFirmware(), "V2.5");
  EXPECT_EQ(sensor.getModel(), "MiniRAE 3000");
  EXPECT_EQ(sensor.getSerialNumber(), "SN-12345");
  EXPECT_EQ(sensor.getName(), "MyPID");
  EXPECT_TRUE(sensor.switchPower());

  const auto full = sensor.getFullInfo();
  ASSERT_EQ(full.m_readings.size(), 3u);
  EXPECT_NEAR(full.m_readings[2].readingsVoltage.back(), 3.0, 1e-6);

  float minLimit = 0;
  float maxLimit = 0;
  sensor.getLimits(minLimit, maxLimit);
  EXPECT_NEAR(maxLimit, 100.0f, 1e-6f);
  EXPECT_NEAR(minLimit, 5.0f, 1e-6f);
}

TEST(CRaePID, ErrorStatusAndMalformedReplies)
{
  FakePID pid;
  if (!pid.dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  pid.set('R', "10.0\r\n");
  CRaePID sensor;
  const auto cfg = pidConfig(pid.dev.slaveName());
  sensor.loadConfig(cfg, "PID");
  sensor.doProcess();  // opens the port

  std::string err;
  pid.set('E', "0 0\r\n");
  EXPECT_FALSE(sensor.errorStatus(err));

  pid.set('E', "1 0\r\n");
  EXPECT_TRUE(sensor.errorStatus(err));
  EXPECT_EQ(err, "1 0");

  // A reply that does not have two fields must not crash:
  pid.set('E', "7\r\n");
  EXPECT_TRUE(sensor.errorStatus(err));
  pid.set('E', "\r\n");
  EXPECT_TRUE(sensor.errorStatus(err));

  float mn = 0;
  float mx = 0;
  pid.set('L', "12\r\n");
  EXPECT_ANY_THROW(sensor.getLimits(mn, mx));

  // A power switch that does not confirm:
  pid.set('P', "Wake up\r\n");
  EXPECT_FALSE(sensor.switchPower());
}

TEST(CRaePID, CannotOpenThePort)
{
  CRaePID sensor;
  const auto cfg = pidConfig("/dev/mrpt-no-such-serial-port");
  sensor.loadConfig(cfg, "PID");
  EXPECT_ANY_THROW(sensor.doProcess());
}

#endif  // MRPT_HWDRIVERS_TESTS_HAVE_PTY
