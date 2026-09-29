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
#include <mrpt/core/bit_cast.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/obs/CObservationIMU.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "pty_device.h"

#ifdef MRPT_HWDRIVERS_TESTS_HAVE_PTY

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::PtyDevice;

namespace
{
mrpt::config::CConfigFileMemory imuConfig(const std::string& port, const std::string& model)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("IMU", "serialPort", port);
  cfg.write("IMU", "sensorModel", model);
  cfg.write("IMU", "pose_x", "0.25");
  cfg.write("IMU", "pose_yaw", "90");
  return cfg;
}

/** Calls doProcess() until the sensor has produced observations (or ~3 s). */
CGenericSensor::TListObservations pollObservations(CTaoboticsIMU& imu)
{
  for (int i = 0; i < 150; i++)
  {
    imu.doProcess();
    auto lst = imu.getObservations();
    if (!lst.empty())
    {
      return lst;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return {};
}

// --- hfi-b6 frames: 0x55 TYPE D0..D7 CHECKSUM (11 bytes) ---
std::string b6Frame(uint8_t type)
{
  std::string f(11, '\0');
  f[0] = 0x55;
  f[1] = static_cast<char>(type);
  return f;
}

// --- hfi-a9 frames ---
void putFloat(std::string& f, size_t pos, float v)
{
  const auto u = mrpt::bit_cast<uint32_t>(v);
  for (int b = 0; b < 4; b++)
  {
    f[pos + static_cast<size_t>(b)] = static_cast<char>((u >> (8 * b)) & 0xFF);
  }
}

std::string a9ImuFrame(const std::array<float, 9>& d)
{
  std::string f(49, '\0');
  f[0] = static_cast<char>(0xAA);
  f[1] = 0x55;
  f[2] = 0x2c;
  for (size_t i = 0; i < 9; i++)
  {
    putFloat(f, 11 + 4 * i, d[i]);
  }
  return f;
}

std::string a9AngleFrame(float roll, float pitch, float yaw)
{
  std::string f(25, '\0');
  f[0] = static_cast<char>(0xAA);
  f[1] = 0x55;
  f[2] = 0x14;
  putFloat(f, 11, roll);
  putFloat(f, 15, pitch);
  putFloat(f, 19, yaw);
  return f;
}
}  // namespace

TEST(CTaoboticsIMU, HfiA9FramesBecomeObservations)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  // Some noise, then one data frame plus one attitude frame, repeated:
  const std::string block = std::string("\x01\x02\x03", 3) +
                            a9ImuFrame({0.1f, 0.2f, 0.3f, 0.0f, 0.0f, 1.0f, 10.f, 20.f, 30.f}) +
                            a9AngleFrame(10.0f, 20.0f, 30.0f);
  dev.startStreaming(block + block + block, 20);

  CTaoboticsIMU imu;
  const auto cfg = imuConfig(dev.slaveName(), "hfi-a9");
  imu.loadConfig(cfg, "IMU");
  imu.initialize();

  const auto lst = pollObservations(imu);
  ASSERT_GE(lst.size(), 1u);
  const auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->sensorLabel, "IMU");
  EXPECT_NEAR(obs->sensorPose.x(), 0.25, 1e-9);
  EXPECT_NEAR(obs->get(mrpt::obs::IMU_Z_ACC), -9.8, 1e-5);
  EXPECT_NEAR(obs->get(mrpt::obs::IMU_WY), 0.2, 1e-6);
  EXPECT_NEAR(obs->get(mrpt::obs::IMU_MAG_Z), 30.0, 1e-4);

  // The attitude is a unit quaternion:
  const double w = obs->get(mrpt::obs::IMU_ORI_QUAT_W);
  const double x = obs->get(mrpt::obs::IMU_ORI_QUAT_X);
  const double y = obs->get(mrpt::obs::IMU_ORI_QUAT_Y);
  const double z = obs->get(mrpt::obs::IMU_ORI_QUAT_Z);
  EXPECT_NEAR(w * w + x * x + y * y + z * z, 1.0, 1e-6);
}

TEST(CTaoboticsIMU, HfiB6FramesAndSynchronization)
{
  PtyDevice dev;
  if (!dev.ok())
  {
    GTEST_SKIP() << "No pseudo-terminal available";
  }
  // Junk first; the parser waits for an acceleration frame (0x51) to
  // synchronize, and an unknown frame type must not break it:
  std::string block = "\x10\x20";
  block += b6Frame(0x51);
  block += b6Frame(0x52);
  block += b6Frame(0x54);  // unknown type
  block += b6Frame(0x53);  // closes one observation
  dev.startStreaming(block + block, 20);

  CTaoboticsIMU imu;
  const auto cfg = imuConfig(dev.slaveName(), "hfi-b6");
  imu.loadConfig(cfg, "IMU");
  imu.initialize();

  const auto lst = pollObservations(imu);
  ASSERT_GE(lst.size(), 1u);
  const auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->sensorLabel, "IMU");
}

TEST(CTaoboticsIMU, ConfigurationErrors)
{
  // Unknown model:
  {
    CTaoboticsIMU imu;
    const auto cfg = imuConfig("/dev/null", "no-such-model");
    imu.loadConfig(cfg, "IMU");
    EXPECT_ANY_THROW(imu.initialize());
  }
  // A port that cannot be opened leaves the sensor in the error state, and
  // doProcess() keeps trying to recover it without crashing:
  {
    CTaoboticsIMU imu;
    const auto cfg = imuConfig("/dev/mrpt-no-such-serial-port", "hfi-a9");
    imu.loadConfig(cfg, "IMU");
    EXPECT_NO_THROW(imu.initialize());
    EXPECT_EQ(imu.getState(), CGenericSensor::ssError);
    EXPECT_NO_THROW(imu.doProcess());
  }
  // Port settings can only change before initialize():
  {
    CTaoboticsIMU imu;
    imu.setSerialPort("/dev/mrpt-no-such-serial-port");
    imu.setSerialBaudRate(115200);
    const auto cfg = imuConfig("/dev/mrpt-no-such-serial-port", "hfi-a9");
    imu.loadConfig(cfg, "IMU");
    imu.initialize();
  }
  {
    PtyDevice dev;
    if (dev.ok())
    {
      CTaoboticsIMU imu;
      const auto cfg = imuConfig(dev.slaveName(), "hfi-a9");
      imu.loadConfig(cfg, "IMU");
      imu.initialize();
      EXPECT_ANY_THROW(imu.setSerialPort("/dev/other"));
      EXPECT_ANY_THROW(imu.setSerialBaudRate(9600));
    }
  }
  // doProcess() before initialize() is a programming error:
  {
    CTaoboticsIMU imu;
    EXPECT_ANY_THROW(imu.doProcess());
  }
}

#endif  // MRPT_HWDRIVERS_TESTS_HAVE_PTY
