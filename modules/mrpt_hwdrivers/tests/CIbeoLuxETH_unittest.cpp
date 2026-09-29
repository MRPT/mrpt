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
#include <mrpt/hwdrivers/CIbeoLuxETH.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <chrono>
#include <thread>

#include "fake_tcp_device.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::FakeTcpDevice;

namespace
{
struct Point
{
  unsigned layer;
  int hangleTicks;
  unsigned distanceCm;
};

/** A binary Ibeo LUX message: magic word, 20 byte header, then the payload. */
std::string ibeoFrame(uint16_t dataType, const std::string& payload)
{
  std::string s = "\xAF\xFE\xC0\xC2";
  std::string header(20, '\0');
  header[10] = static_cast<char>(dataType >> 8);
  header[11] = static_cast<char>(dataType & 0xFF);
  return s + header + payload;
}

/** A "scan data" (0x2202) message with the given points. */
std::string ibeoScanFrame(const std::vector<Point>& pts, unsigned angleTicks = 5760)
{
  std::string scanHeader(44, '\0');
  scanHeader[22] = static_cast<char>(angleTicks & 0xFF);
  scanHeader[23] = static_cast<char>(angleTicks >> 8);
  scanHeader[28] = static_cast<char>(pts.size() & 0xFF);
  scanHeader[29] = static_cast<char>(pts.size() >> 8);
  std::string payload = scanHeader;
  for (const auto& p : pts)
  {
    std::string pt(10, '\0');
    pt[0] = static_cast<char>(p.layer);
    pt[2] = static_cast<char>(p.hangleTicks & 0xFF);
    pt[3] = static_cast<char>((p.hangleTicks >> 8) & 0xFF);
    pt[4] = static_cast<char>(p.distanceCm & 0xFF);
    pt[5] = static_cast<char>(p.distanceCm >> 8);
    payload += pt;
  }
  return ibeoFrame(0x2202, payload);
}

/** Waits until the sensor has delivered observations (or a timeout). */
CGenericSensor::TListObservations waitForObservations(CIbeoLuxETH& sensor)
{
  for (int i = 0; i < 300; i++)
  {
    auto lst = sensor.getObservations();
    if (!lst.empty())
    {
      return lst;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return {};
}
}  // namespace

TEST(CIbeoLuxETH, CommandsAreWellFormed)
{
  CIbeoLuxETH sensor;
  unsigned char buf[32] = {};
  sensor.makeCommandHeader(buf);
  EXPECT_EQ(buf[0], 0xAF);
  EXPECT_EQ(buf[3], 0xC2);
  EXPECT_EQ(buf[14], 0x20);
  EXPECT_EQ(buf[15], 0x10);

  sensor.makeStartCommand(buf);
  EXPECT_EQ(buf[11], 0x04);
  EXPECT_EQ(buf[24], 0x20);
  sensor.makeStopCommand(buf);
  EXPECT_EQ(buf[24], 0x21);
  sensor.makeTypeCommand(buf);
  EXPECT_EQ(buf[11], 0x08);
  EXPECT_EQ(buf[25], 0x05);
}

TEST(CIbeoLuxETH, DecodesScanMessages)
{
  // Garbage, messages the driver ignores, an unknown one, and finally a scan
  // with valid and invalid points:
  std::string stream = "\x01\x02\xAF\xAF\xFE\x00junk";
  stream += ibeoFrame(0x2030, std::string(4, '\0'));
  stream += ibeoFrame(0x2221, std::string(4, '\0'));
  stream += ibeoFrame(0x2805, std::string(4, '\0'));
  stream += ibeoFrame(0x2020, std::string(4, '\0'));
  stream += ibeoFrame(0x1234, std::string(4, '\0'));  // unknown type
  stream += ibeoScanFrame({
      {2,    0,  1000}, // straight ahead, 10 m: valid
      {0,  100,  2000}, // valid
      {1, -100,   500}, // valid
      {3,    0,    10}, // too close
      {2,    0, 30000}, // too far
      {9,    0,  1000}, // invalid layer
      {2, 4000,  1000}, // outside the horizontal field of view
  });

  FakeTcpDevice dev([](const std::string&) { return std::string(); }, stream);
  ASSERT_TRUE(dev.isReady());

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LUX", "pose_x", "1.5");
  CIbeoLuxETH sensor("127.0.0.1", dev.port());
  sensor.loadConfig(cfg, "LUX");
  sensor.initialize();
  sensor.doProcess();  // does nothing: data comes from the thread

  const auto lst = waitForObservations(sensor);
  ASSERT_GE(lst.size(), 1u);
  const auto obs =
      std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_TRUE(obs->hasPoints3D);
  ASSERT_EQ(obs->points3D_x.size(), 3u);
  ASSERT_EQ(obs->points3D_y.size(), 3u);
  ASSERT_EQ(obs->points3D_z.size(), 3u);

  // First point: 10 m ahead in the scanner frame, which the driver maps to
  // -X (theta = vrad + pi):
  EXPECT_NEAR(obs->points3D_x[0], -10.0f, 0.05f);
  EXPECT_NEAR(obs->points3D_y[0], 0.0f, 0.1f);
  EXPECT_NEAR(obs->points3D_z[0], 0.0f, 1e-3f);

  // The driver sent the filter and start commands to the device:
  std::string sent;
  for (int i = 0; i < 100 && sent.size() < 32u + 28u; i++)
  {
    sent.clear();
    for (const auto& r : dev.requests())
    {
      sent += r;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GE(sent.size(), 32u + 28u);
  EXPECT_EQ(static_cast<unsigned char>(sent[0]), 0xAF);
}

TEST(CIbeoLuxETH, UnreachableDeviceDoesNotCrash)
{
  // The connection is made from the data thread: failing to connect must not
  // take the whole process down.
  CIbeoLuxETH sensor("127.0.0.1", 18799);
  sensor.initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(sensor.getObservations().empty());
}

TEST(CIbeoLuxETH, DestructorWithoutInitialize)
{
  // Destroying a sensor whose thread never started must be harmless.
  {
    CIbeoLuxETH sensor;
  }
  SUCCEED();
}
