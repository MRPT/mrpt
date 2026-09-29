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
#include <mrpt/core/bits_math.h>
#include <mrpt/hwdrivers/CGillAnemometer.h>
#include <mrpt/obs/CObservationWindSensor.h>

#include "pty_device.h"

#ifdef MRPT_HWDRIVERS_TESTS_HAVE_PTY

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::PtyDevice;

namespace
{
mrpt::config::CConfigFileMemory anemometerConfig(const std::string& port)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("WIND", "COM_port_LIN", port);
  cfg.write("WIND", "pose_x", "1.0");
  cfg.write("WIND", "pose_y", "2.0");
  cfg.write("WIND", "pose_z", "3.0");
  cfg.write("WIND", "pose_roll", "0");
  cfg.write("WIND", "pose_pitch", "0");
  cfg.write("WIND", "pose_yaw", "90");
  return cfg;
}

/** Makes a fake anemometer stream `line` and returns the observation the
 * driver produces from it (or null). The driver opens (and flushes) the port
 * inside doProcess(), so the device has to keep transmitting. */
mrpt::obs::CObservationWindSensor::Ptr observationFor(const std::string& line)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    return nullptr;
  }
  dev.startStreaming(line, 10);

  CGillAnemometer sensor;
  const auto cfg = anemometerConfig(dev.slaveName());
  sensor.loadConfig(cfg, "WIND");
  sensor.doProcess();
  const auto lst = sensor.getObservations();
  if (lst.empty())
  {
    return nullptr;
  }
  return std::dynamic_pointer_cast<mrpt::obs::CObservationWindSensor>(lst.begin()->second);
}

bool havePty()
{
  PtyDevice dev;
  return dev.ok();
}
}  // namespace

TEST(CGillAnemometer, DecodesTheReadings)
{
  if (!havePty())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }

  // <STX>Q,dir,speed,units,status,<ETX>
  {
    const auto obs = observationFor("\x02Q,090,002.50,M,00,\x03\r\n");
    ASSERT_TRUE(obs);
    EXPECT_NEAR(obs->speed, 2.5, 1e-9);
    EXPECT_NEAR(obs->direction, 90.0, 1e-9);
    EXPECT_EQ(obs->sensorLabel, "WINDSONIC");
    EXPECT_NEAR(obs->sensorPoseOnRobot.x(), 1.0, 1e-9);
    EXPECT_NEAR(obs->sensorPoseOnRobot.z(), 3.0, 1e-9);
    // The yaw is given in degrees in the configuration file:
    EXPECT_NEAR(obs->sensorPoseOnRobot.yaw(), mrpt::DEG2RAD(90.0), 1e-6);
  }
  // km/h are converted to m/s:
  {
    const auto obs = observationFor("\x02Q,180,036.00,K,00,\x03\r\n");
    ASSERT_TRUE(obs);
    EXPECT_NEAR(obs->speed, 10.0, 1e-9);
  }
  // Without wind there is no direction field:
  {
    const auto obs = observationFor("\x02Q,000.00,M,00,\x03\r\n");
    ASSERT_TRUE(obs);
    EXPECT_NEAR(obs->speed, 0.0, 1e-9);
    EXPECT_NEAR(obs->direction, 0.0, 1e-9);
  }
  {
    const auto obs = observationFor("\x02Q,018.00,K,00,\x03\r\n");
    ASSERT_TRUE(obs);
    EXPECT_NEAR(obs->speed, 5.0, 1e-9);
  }
}

TEST(CGillAnemometer, BadReadingsAreIgnored)
{
  if (!havePty())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }

  // Error status, in both layouts:
  EXPECT_FALSE(observationFor("\x02Q,090,002.50,M,04,\x03\r\n"));
  EXPECT_FALSE(observationFor("\x02Q,002.50,M,02,\x03\r\n"));
  // Wrong number of fields:
  EXPECT_FALSE(observationFor("garbage,line\r\n"));
  // Unsupported units still produce an observation, with zero speed:
  const auto obs = observationFor("\x02Q,090,002.50,X,00,\x03\r\n");
  ASSERT_TRUE(obs);
  EXPECT_NEAR(obs->speed, 0.0, 1e-9);
  const auto obs2 = observationFor("\x02Q,002.50,X,00,\x03\r\n");
  ASSERT_TRUE(obs2);
  EXPECT_NEAR(obs2->speed, 0.0, 1e-9);
}

TEST(CGillAnemometer, CannotOpenThePort)
{
  CGillAnemometer sensor;
  const auto cfg = anemometerConfig("/dev/mrpt-no-such-serial-port");
  sensor.loadConfig(cfg, "WIND");
  EXPECT_ANY_THROW(sensor.doProcess());
}

#endif  // MRPT_HWDRIVERS_TESTS_HAVE_PTY
