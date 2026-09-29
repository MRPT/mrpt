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
#include <mrpt/hwdrivers/CLMS100eth.h>

#include "fake_tcp_device.h"
#include "sick_telegrams.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::ETX;
using mrpt::hwdrivers::testing::FakeTcpDevice;
using mrpt::hwdrivers::testing::pollObservations;
using mrpt::hwdrivers::testing::pollScan;
using mrpt::hwdrivers::testing::scanTelegram;
using mrpt::hwdrivers::testing::STX;

namespace
{
/** The handler of a well-behaved LMS100. */
FakeTcpDevice::Handler lmsHandler(const std::string& scan, int notReadyPolls = 0)
{
  auto polls = std::make_shared<int>(notReadyPolls);
  return [scan, polls](const std::string& req) -> std::string
  {
    if (req.find("LMDscandata") != std::string::npos && req.find("sRN") != std::string::npos)
    {
      return scan;
    }
    if (req.find("STlms") != std::string::npos)
    {
      const bool ready = (*polls)-- <= 0;
      return std::string(1, STX) + "sRA STlms " + (ready ? "7" : "1") + " 0" + ETX;
    }
    // Every other command is simply acknowledged:
    return std::string(1, STX) + "sAN ok 1" + ETX;
  };
}
}  // namespace

TEST(CLMS100Eth, ScanAcquisition)
{
  FakeTcpDevice dev(lmsHandler(scanTelegram({1000, 2000, 30000, 4500})));
  ASSERT_TRUE(dev.isReady());

  CLMS100Eth sensor("127.0.0.1", dev.port());
  sensor.setSensorPose(mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
  ASSERT_TRUE(sensor.turnOn());

  bool thereIsObs = false;
  bool hwError = true;
  mrpt::obs::CObservation2DRangeScan obs;
  thereIsObs = false;
  pollScan(sensor, thereIsObs, obs, hwError);
  ASSERT_TRUE(thereIsObs);
  EXPECT_FALSE(hwError);

  ASSERT_EQ(obs.getScanSize(), 4u);
  EXPECT_NEAR(obs.getScanRange(0), 1.0f, 1e-5f);
  EXPECT_NEAR(obs.getScanRange(2), 30.0f, 1e-4f);
  // Farther than the maximum range: not valid
  EXPECT_TRUE(obs.getScanRangeValidity(1));
  EXPECT_FALSE(obs.getScanRangeValidity(2));
  EXPECT_NEAR(obs.sensorPose.x(), 1.0, 1e-9);

  // The initialization sequence was sent to the device, in order:
  const auto reqs = dev.requests();
  ASSERT_GE(reqs.size(), 5u);
  EXPECT_NE(reqs[0].find("SetAccessMode"), std::string::npos);
  EXPECT_NE(reqs[1].find("mLMPsetscancfg"), std::string::npos);
  EXPECT_NE(reqs[2].find("LMDscandatacfg"), std::string::npos);
  EXPECT_NE(reqs[3].find("LMCstartmeas"), std::string::npos);

  EXPECT_TRUE(sensor.turnOff());
  // After turning it off no scans are produced:
  sensor.doProcessSimple(thereIsObs, obs, hwError);
  EXPECT_FALSE(thereIsObs);
  EXPECT_TRUE(hwError);
}

TEST(CLMS100Eth, InitializeFromConfigAndDoProcess)
{
  FakeTcpDevice dev(lmsHandler(scanTelegram({500, 600, 700})));
  ASSERT_TRUE(dev.isReady());

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("LMS", "ip_address", "127.0.0.1");
  cfg.write("LMS", "TCP_port", std::to_string(dev.port()));
  cfg.write("LMS", "sensorLabel", "MY_SICK");
  cfg.write("LMS", "pose_x", "0.5");
  cfg.write("LMS", "pose_yaw", "90");
  cfg.write("LMS", "process_rate", "20");

  CLMS100Eth sensor;
  sensor.loadConfig(cfg, "LMS");
  ASSERT_NO_THROW(sensor.initialize());

  const auto lst = pollObservations(sensor);
  ASSERT_EQ(lst.size(), 1u);
  const auto obs =
      std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->sensorLabel, "MY_SICK");
  EXPECT_NEAR(obs->sensorPose.x(), 0.5, 1e-9);
  EXPECT_NEAR(obs->sensorPose.yaw(), mrpt::DEG2RAD(90.0), 1e-6);
  EXPECT_EQ(obs->getScanSize(), 3u);
}

TEST(CLMS100Eth, WaitsUntilTheDeviceIsReady)
{
  // First status poll says "not ready":
  FakeTcpDevice dev(lmsHandler(scanTelegram({1000}), 1 /*not ready polls*/));
  ASSERT_TRUE(dev.isReady());
  CLMS100Eth sensor("127.0.0.1", dev.port());
  EXPECT_TRUE(sensor.turnOn());
}

TEST(CLMS100Eth, InitializeFailsWithoutDevice)
{
  // Nothing listens on this port:
  CLMS100Eth sensor("127.0.0.1", 18799);
  EXPECT_FALSE(sensor.turnOn());
  EXPECT_ANY_THROW(sensor.initialize());
}

TEST(CLMS100Eth, SilentDeviceMakesTurnOnFail)
{
  FakeTcpDevice dev([](const std::string&) { return std::string(); });
  ASSERT_TRUE(dev.isReady());
  CLMS100Eth sensor("127.0.0.1", dev.port());
  EXPECT_FALSE(sensor.turnOn());
}

TEST(CLMS100Eth, MalformedScansAreReportedAsHardwareErrors)
{
  // A device that says "ready" but answers scan requests with junk:
  const std::vector<std::string> junkTelegrams = {
      std::string(1, STX) + "sFA 5" + ETX, std::string(1, STX) + "sRA WrongName 1 2" + ETX};
  for (const auto& junk : junkTelegrams)
  {
    FakeTcpDevice dev(lmsHandler(junk));
    ASSERT_TRUE(dev.isReady());
    CLMS100Eth sensor("127.0.0.1", dev.port());
    ASSERT_TRUE(sensor.turnOn());
    bool thereIsObs = true;
    bool hwError = false;
    mrpt::obs::CObservation2DRangeScan obs;
    sensor.doProcessSimple(thereIsObs, obs, hwError);
    EXPECT_FALSE(thereIsObs);
    EXPECT_TRUE(hwError);
  }
}

TEST(CLMS100Eth, ScanWithWrongChannelThrows)
{
  FakeTcpDevice dev(lmsHandler(scanTelegram({100}, "0", "RSSI1")));
  ASSERT_TRUE(dev.isReady());
  CLMS100Eth sensor("127.0.0.1", dev.port());
  ASSERT_TRUE(sensor.turnOn());
  bool thereIsObs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  EXPECT_ANY_THROW(sensor.doProcessSimple(thereIsObs, obs, hwError));
}

TEST(CLMS100Eth, ScanWithStatusErrorsIsStillDecoded)
{
  for (const std::string& status : std::vector<std::string>{"1", "4"})
  {
    FakeTcpDevice dev(lmsHandler(scanTelegram({1000, 2000}, status)));
    ASSERT_TRUE(dev.isReady());
    CLMS100Eth sensor("127.0.0.1", dev.port());
    ASSERT_TRUE(sensor.turnOn());
    bool thereIsObs = false;
    bool hwError = false;
    mrpt::obs::CObservation2DRangeScan obs;
    thereIsObs = false;
    pollScan(sensor, thereIsObs, obs, hwError);
    EXPECT_TRUE(thereIsObs) << status;
    EXPECT_EQ(obs.getScanSize(), 2u);
  }
}

TEST(CLMS100Eth, TruncatedScanIsAnError)
{
  // The header announces 5 ranges but only 2 arrive:
  std::string s = scanTelegram({1000, 2000});
  const auto pos = s.find(" 2 ");
  ASSERT_NE(pos, std::string::npos);
  s.replace(pos, 3, " 5 ");
  FakeTcpDevice dev(lmsHandler(s));
  ASSERT_TRUE(dev.isReady());
  CLMS100Eth sensor("127.0.0.1", dev.port());
  ASSERT_TRUE(sensor.turnOn());
  bool thereIsObs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  sensor.doProcessSimple(thereIsObs, obs, hwError);
  EXPECT_FALSE(thereIsObs);
  EXPECT_TRUE(hwError);
}

TEST(CLMS100Eth, NoDataWhenDeviceIsSilentDuringAcquisition)
{
  // Ready, but stops answering scan requests:
  auto answerScans = std::make_shared<std::atomic<bool>>(true);
  auto base = lmsHandler(scanTelegram({1000}));
  FakeTcpDevice dev(
      [base, answerScans](const std::string& req) -> std::string
      {
        if (req.find("sRN LMDscandata") != std::string::npos && !*answerScans)
        {
          return {};
        }
        return base(req);
      });
  ASSERT_TRUE(dev.isReady());
  CLMS100Eth sensor("127.0.0.1", dev.port());
  ASSERT_TRUE(sensor.turnOn());
  *answerScans = false;

  bool thereIsObs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  sensor.doProcessSimple(thereIsObs, obs, hwError);
  EXPECT_FALSE(thereIsObs);
  EXPECT_TRUE(hwError);
}
