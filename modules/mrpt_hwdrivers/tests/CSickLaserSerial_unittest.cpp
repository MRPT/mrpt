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
#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/system/crc.h>

#include <string>
#include <vector>

#include "mock_stream.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::MockStream;

namespace
{
// Same generator polynomial the driver uses for its frames.
constexpr uint16_t SICK_CRC16_GEN_POL = 0x8005;

/** Builds a continuous-mode measurement frame (command 0xB0), as the LMS
 *  sends it:
 *  | STX | ADDR | L1 | L2 | COM | INF1 | INF2 | DATA | STA | CRC1 | CRC2 |
 */
std::string makeScanFrame(
    const std::vector<int>& ranges,
    bool mmMode,
    uint8_t status = 0,
    bool corruptCRC = false,
    uint8_t commandByte = 0xB0)
{
  const int nPoints = static_cast<int>(ranges.size());
  // INF1/INF2: low 9 bits are the point count, top 2 bits select mm mode.
  const uint16_t info = static_cast<uint16_t>((nPoints & 0x01FF) | (mmMode ? 0x4000 : 0x0000));

  std::vector<uint8_t> f;
  f.push_back(0x02);         // STX
  f.push_back(0x80);         // ADDR
  f.push_back(0);            // L1 (filled in below)
  f.push_back(0);            // L2
  f.push_back(commandByte);  // COM
  f.push_back(static_cast<uint8_t>(info & 0xFF));
  f.push_back(static_cast<uint8_t>(info >> 8));
  for (int r : ranges)
  {
    f.push_back(static_cast<uint8_t>(r & 0xFF));
    f.push_back(static_cast<uint8_t>((r >> 8) & 0xFF));
  }
  f.push_back(status);

  // The length field counts everything after the 4-byte header, excluding the
  // 2 CRC bytes: the driver reads a total frame of (6 + L) bytes.
  const uint16_t len = static_cast<uint16_t>(f.size() + 2 - 6);
  f[2] = static_cast<uint8_t>(len & 0xFF);
  f[3] = static_cast<uint8_t>(len >> 8);

  uint16_t crc = mrpt::system::compute_CRC16(f.data(), f.size(), SICK_CRC16_GEN_POL);
  if (corruptCRC)
  {
    crc = static_cast<uint16_t>(crc + 1);
  }
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>(crc >> 8));

  return {f.begin(), f.end()};
}

/** A driver already bound to `s` and set to skip the serial-line setup
 *  handshake, which is what a real port would need but a mock does not. */
std::shared_ptr<CSickLaserSerial> makeBoundLaser(const std::shared_ptr<MockStream>& s)
{
  auto laser = std::make_shared<CSickLaserSerial>();
  mrpt::config::CConfigFileMemory cfg;
  // A COM port name is a mandatory config entry even when a stream is bound:
#ifdef _WIN32
  cfg.write("SICK", "COM_port_WIN", "COM1");
#else
  cfg.write("SICK", "COM_port_LIN", "/dev/null");
#endif
  cfg.write("SICK", "skip_laser_config", true);
  laser->loadConfig(cfg, "SICK");
  laser->bindIO(s);
  return laser;
}
}  // namespace

TEST(CSickLaserSerial, decodeCentimeterModeScan)
{
  auto s = std::make_shared<MockStream>();
  // 361 points at 250 cm = 2.5 m, the classic LMS200 180 deg / 0.5 deg scan:
  const std::vector<int> ranges(361, 250);
  s->pushRx(makeScanFrame(ranges, false));

  auto laser = makeBoundLaser(s);

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(hwError);
  ASSERT_TRUE(thereIs);
  ASSERT_EQ(obs.getScanSize(), ranges.size());
  EXPECT_TRUE(obs.rightToLeft);
  EXPECT_NEAR(obs.aperture, M_PI, 1e-4);
  // Centimeter mode tops out at 81 m:
  EXPECT_NEAR(obs.maxRange, 81.0f, 1e-4f);
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    EXPECT_NEAR(obs.getScanRange(i), 2.50f, 1e-4f) << "at i=" << i;
    EXPECT_TRUE(obs.getScanRangeValidity(i)) << "at i=" << i;
  }
}

TEST(CSickLaserSerial, decodeMillimeterModeScan)
{
  auto s = std::make_shared<MockStream>();
  // 181 points at 1234 mm:
  const std::vector<int> ranges(181, 1234);
  s->pushRx(makeScanFrame(ranges, true));

  auto laser = makeBoundLaser(s);

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  ASSERT_TRUE(thereIs);
  ASSERT_EQ(obs.getScanSize(), ranges.size());
  // Millimeter mode tops out at 32.7 m:
  EXPECT_NEAR(obs.maxRange, 32.7f, 1e-4f);
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    EXPECT_NEAR(obs.getScanRange(i), 1.234f, 1e-4f) << "at i=" << i;
  }
}

TEST(CSickLaserSerial, outOfRangeReadingsAreMarkedInvalid)
{
  auto s = std::make_shared<MockStream>();
  // 0x1FFF cm = 81.91 m, above the 81 m centimeter-mode maximum:
  const std::vector<int> ranges(181, 0x1FFF);
  s->pushRx(makeScanFrame(ranges, false));

  auto laser = makeBoundLaser(s);

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  ASSERT_TRUE(thereIs);
  for (size_t i = 0; i < obs.getScanSize(); i++)
  {
    EXPECT_FALSE(obs.getScanRangeValidity(i)) << "at i=" << i;
  }
}

TEST(CSickLaserSerial, badCRCIsRejected)
{
  auto s = std::make_shared<MockStream>();
  const std::vector<int> ranges(181, 500);
  s->pushRx(makeScanFrame(ranges, false, 0, /*corruptCRC=*/true));

  auto laser = makeBoundLaser(s);

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(thereIs);
}

TEST(CSickLaserSerial, resynchronizesAfterLeadingGarbage)
{
  auto s = std::make_shared<MockStream>();
  const std::vector<int> ranges(181, 300);
  // Bytes that are not a valid STX/ADDR header must be skipped over:
  s->pushRx(std::string("\x11\x22\x33\x44", 4) + makeScanFrame(ranges, false));

  auto laser = makeBoundLaser(s);

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  ASSERT_TRUE(thereIs);
  EXPECT_EQ(obs.getScanSize(), ranges.size());
  EXPECT_NEAR(obs.getScanRange(0), 3.0f, 1e-4f);
}

TEST(CSickLaserSerial, silentDeviceYieldsNoScan)
{
  auto s = std::make_shared<MockStream>();
  auto laser = makeBoundLaser(s);

  bool thereIs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(thereIs);
}

TEST(CSickLaserSerial, nonMeasurementFramesAreIgnored)
{
  auto s = std::make_shared<MockStream>();
  // A well-formed frame, CRC included, whose command byte is not the 0xB0 of a
  // measurement reply. The command byte has to be set before the CRC is
  // computed, or the driver rejects the frame at the CRC check instead and
  // never reaches the dispatch this test is about.
  s->pushRx(makeScanFrame(
      std::vector<int>(10, 100), false, /*status=*/0, /*corruptCRC=*/false,
      /*commandByte=*/0xA0));

  auto laser = makeBoundLaser(s);

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  laser->doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(thereIs);
}

TEST(CSickLaserSerial, unboundDriverWithoutAPortThrows)
{
  CSickLaserSerial laser;

  bool thereIs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  // No bindIO() and no setSerialPort(): reported as a hardware error.
  laser.doProcessSimple(thereIs, obs, hwError);

  EXPECT_FALSE(thereIs);
  EXPECT_TRUE(hwError);
}

TEST(CSickLaserSerial, loadConfigReadsSensorSettings)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string sec = "SICK";
#ifdef _WIN32
  cfg.write(sec, "COM_port_WIN", "COM5");
#else
  cfg.write(sec, "COM_port_LIN", "/dev/ttyS0");
#endif
  cfg.write(sec, "COM_baudRate", 38400);
  cfg.write(sec, "mm_mode", true);
  cfg.write(sec, "FOV", 180);
  cfg.write(sec, "resolution", 50);
  cfg.write(sec, "pose_x", 0.1);
  cfg.write(sec, "pose_y", 0.2);
  cfg.write(sec, "pose_z", 0.3);

  CSickLaserSerial laser;
  EXPECT_NO_THROW(laser.loadConfig(cfg, sec));

  // The COM port name is mandatory:
  mrpt::config::CConfigFileMemory bad;
  bad.write(sec, "COM_baudRate", 38400);
  CSickLaserSerial laser2;
  EXPECT_THROW(laser2.loadConfig(bad, sec), std::exception);
}
