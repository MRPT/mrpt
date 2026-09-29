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
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>
#include <mrpt/obs/CObservationIMU.h>

#include "pty_device.h"

#ifdef MRPT_HWDRIVERS_TESTS_HAVE_PTY

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::PtyDevice;

namespace
{
mrpt::config::CConfigFileMemory gyroConfig(const std::string& port, const std::string& mode)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("GYRO", "COM_port_LIN", port);
  cfg.write("GYRO", "operatingMode", mode);
  cfg.write("GYRO", "pose_x", "0.5");
  cfg.write("GYRO", "pose_yaw", "90");
  return cfg;
}
}  // namespace

TEST(CGyroKVHDSP3000, RateMode)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  dev.startStreaming("  10.5 1\n", 10);

  CGyroKVHDSP3000 gyro;
  const auto cfg = gyroConfig(dev.slaveName(), "rate");
  gyro.loadConfig(cfg, "GYRO");
  gyro.initialize();

  // The very first reading is discarded:
  gyro.doProcess();
  EXPECT_TRUE(gyro.getObservations().empty());
  gyro.doProcess();
  const auto lst = gyro.getObservations();
  ASSERT_EQ(lst.size(), 1u);
  const auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->sensorLabel, "KVH_DSP3000");
  EXPECT_TRUE(obs->dataIsPresent[mrpt::obs::IMU_YAW_VEL]);
  EXPECT_FALSE(obs->dataIsPresent[mrpt::obs::IMU_YAW]);
  EXPECT_NEAR(obs->rawMeasurements[mrpt::obs::IMU_YAW_VEL], mrpt::DEG2RAD(10.5), 1e-9);
  EXPECT_NEAR(obs->sensorPose.x(), 0.5, 1e-9);
  EXPECT_NEAR(obs->sensorPose.yaw(), mrpt::DEG2RAD(90.0), 1e-6);

  // The driver selected the rate mode on the device:
  const std::string sent = dev.receive(16, 500);
  ASSERT_GE(sent.size(), 2u);
  EXPECT_EQ(sent[0], 'R');
}

TEST(CGyroKVHDSP3000, AngleModes)
{
  for (const std::string mode : {"integral", "incremental"})
  {
    PtyDevice dev;
    if (!dev.ok())
    {
      GTEST_SKIP() << "No pseudo-terminal available";
    }
    dev.startStreaming("45.0 1\n", 10);

    CGyroKVHDSP3000 gyro;
    const auto cfg = gyroConfig(dev.slaveName(), mode);
    gyro.loadConfig(cfg, "GYRO");
    gyro.initialize();
    gyro.doProcess();
    gyro.doProcess();
    const auto lst = gyro.getObservations();
    ASSERT_EQ(lst.size(), 1u) << mode;
    const auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(lst.begin()->second);
    ASSERT_TRUE(obs);
    EXPECT_TRUE(obs->dataIsPresent[mrpt::obs::IMU_YAW]) << mode;
    EXPECT_NEAR(obs->rawMeasurements[mrpt::obs::IMU_YAW], mrpt::DEG2RAD(45.0), 1e-9);

    // Mode selection ('P' or 'A') followed by the angle reset ('Z'):
    const std::string sent = dev.receive(16, 500);
    ASSERT_GE(sent.size(), 4u) << mode;
    EXPECT_EQ(sent[0], mode == "integral" ? 'P' : 'A');
    EXPECT_NE(sent.find('Z'), std::string::npos);
  }
}

TEST(CGyroKVHDSP3000, InvalidReadingsAreSkipped)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  // The second field is the validity flag: 0 means "not valid". A line with
  // fewer than two fields is not a reading either.
  dev.startStreaming("5.0 0\nnonsense\n", 10);

  CGyroKVHDSP3000 gyro;
  const auto cfg = gyroConfig(dev.slaveName(), "rate");
  gyro.loadConfig(cfg, "GYRO");
  gyro.initialize();
  for (int i = 0; i < 4; i++)
  {
    gyro.doProcess();
  }
  EXPECT_TRUE(gyro.getObservations().empty());
}

TEST(CGyroKVHDSP3000, InitializeFailsWithoutPort)
{
  CGyroKVHDSP3000 gyro;
  const auto cfg = gyroConfig("/dev/mrpt-no-such-serial-port", "rate");
  gyro.loadConfig(cfg, "GYRO");
  EXPECT_ANY_THROW(gyro.initialize());
}

#endif  // MRPT_HWDRIVERS_TESTS_HAVE_PTY
