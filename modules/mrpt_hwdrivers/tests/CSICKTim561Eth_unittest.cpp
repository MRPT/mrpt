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
#include <mrpt/hwdrivers/CSICKTim561Eth_2050101.h>

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
/** The handler of a well-behaved TiM561: it acknowledges the SOPAS setup
 * commands and answers scan requests with the given telegram. */
FakeTcpDevice::Handler timHandler(const std::string& scan)
{
  return [scan](const std::string& req) -> std::string
  {
    if (req.find("sRN LMDscandata") != std::string::npos)
    {
      return scan;
    }
    return std::string(1, STX) + "sRA ack 1" + ETX;
  };
}
}  // namespace

TEST(CSICKTim561Eth, ScanAcquisition)
{
  FakeTcpDevice dev(timHandler(scanTelegram({1000, 2000, 15000, 4500})));
  ASSERT_TRUE(dev.isReady());

  CSICKTim561Eth sensor("127.0.0.1", dev.port());
  sensor.setSensorPose(mrpt::poses::CPose3D(0.1, 0.2, 0.3, 0, 0, 0));
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
  EXPECT_NEAR(obs.getScanRange(2), 15.0f, 1e-4f);
  EXPECT_TRUE(obs.getScanRangeValidity(1));
  EXPECT_FALSE(obs.getScanRangeValidity(2));  // beyond the 10 m maximum range
  EXPECT_NEAR(obs.sensorPose.z(), 0.3, 1e-9);

  // Initialization sequence:
  const auto reqs = dev.requests();
  ASSERT_GE(reqs.size(), 5u);
  EXPECT_NE(reqs[0].find("sRIO"), std::string::npos);
  EXPECT_NE(reqs[1].find("SerialNumber"), std::string::npos);
  EXPECT_NE(reqs[2].find("FirmwareVersion"), std::string::npos);
  EXPECT_NE(reqs[3].find("SCdevicestate"), std::string::npos);
  EXPECT_NE(reqs[4].find("sEN LMDscandata"), std::string::npos);

  EXPECT_TRUE(sensor.turnOff());
  sensor.doProcessSimple(thereIsObs, obs, hwError);
  EXPECT_FALSE(thereIsObs);
  EXPECT_TRUE(hwError);
}

TEST(CSICKTim561Eth, InitializeFromConfigAndDoProcess)
{
  FakeTcpDevice dev(timHandler(scanTelegram({500, 600})));
  ASSERT_TRUE(dev.isReady());

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("TIM", "ip_address", "127.0.0.1");
  cfg.write("TIM", "TCP_port", std::to_string(dev.port()));
  cfg.write("TIM", "sensorLabel", "MY_TIM");
  cfg.write("TIM", "pose_z", "0.7");

  CSICKTim561Eth sensor;
  sensor.loadConfig(cfg, "TIM");
  ASSERT_NO_THROW(sensor.initialize());

  const auto lst = pollObservations(sensor);
  ASSERT_EQ(lst.size(), 1u);
  const auto obs =
      std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(lst.begin()->second);
  ASSERT_TRUE(obs);
  EXPECT_EQ(obs->sensorLabel, "MY_TIM");
  EXPECT_NEAR(obs->sensorPose.z(), 0.7, 1e-6);
  EXPECT_EQ(obs->getScanSize(), 2u);
}

TEST(CSICKTim561Eth, ConnectionFailures)
{
  CSICKTim561Eth sensor("127.0.0.1", 18799);  // nothing listens here
  EXPECT_FALSE(sensor.turnOn());
  EXPECT_ANY_THROW(sensor.initialize());

  // A device that connects but never answers:
  FakeTcpDevice silent([](const std::string&) { return std::string(); });
  ASSERT_TRUE(silent.isReady());
  CSICKTim561Eth sensor2("127.0.0.1", silent.port());
  EXPECT_FALSE(sensor2.turnOn());
  EXPECT_FALSE(sensor2.rebootDev());
}

TEST(CSICKTim561Eth, RebootDevice)
{
  FakeTcpDevice dev(timHandler(scanTelegram({100})));
  ASSERT_TRUE(dev.isReady());
  CSICKTim561Eth sensor("127.0.0.1", dev.port());
  ASSERT_TRUE(sensor.turnOn());
  EXPECT_TRUE(sensor.rebootDev());

  const auto reqs = dev.requests();
  ASSERT_GE(reqs.size(), 2u);
  EXPECT_NE(reqs[reqs.size() - 2].find("SetAccessMode"), std::string::npos);
  EXPECT_NE(reqs.back().find("mSCreboot"), std::string::npos);
}

TEST(CSICKTim561Eth, BadScansAreHardwareErrors)
{
  const std::vector<std::string> junkTelegrams = {
      std::string(1, STX) + "sFA 5" + ETX,
      std::string(1, STX) + "sRA WrongName 1 2" + ETX,
  };
  for (const auto& junk : junkTelegrams)
  {
    FakeTcpDevice dev(timHandler(junk));
    ASSERT_TRUE(dev.isReady());
    CSICKTim561Eth sensor("127.0.0.1", dev.port());
    ASSERT_TRUE(sensor.turnOn());
    bool thereIsObs = true;
    bool hwError = false;
    mrpt::obs::CObservation2DRangeScan obs;
    sensor.doProcessSimple(thereIsObs, obs, hwError);
    EXPECT_FALSE(thereIsObs);
    EXPECT_TRUE(hwError);
  }

  // A telegram with distances in another channel is a configuration error:
  FakeTcpDevice dev(timHandler(scanTelegram({100}, "0", "RSSI1")));
  ASSERT_TRUE(dev.isReady());
  CSICKTim561Eth sensor("127.0.0.1", dev.port());
  ASSERT_TRUE(sensor.turnOn());
  bool thereIsObs = false;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  EXPECT_ANY_THROW(sensor.doProcessSimple(thereIsObs, obs, hwError));
}

TEST(CSICKTim561Eth, DeviceStatusValuesAreAllAccepted)
{
  for (const std::string& status : std::vector<std::string>{"0", "1", "2"})
  {
    FakeTcpDevice dev(timHandler(scanTelegram({1000, 2000}, status)));
    ASSERT_TRUE(dev.isReady());
    CSICKTim561Eth sensor("127.0.0.1", dev.port());
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

TEST(CSICKTim561Eth, NoDataWhenTheDeviceStopsAnswering)
{
  auto answerScans = std::make_shared<std::atomic<bool>>(true);
  auto base = timHandler(scanTelegram({1000}));
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
  CSICKTim561Eth sensor("127.0.0.1", dev.port());
  ASSERT_TRUE(sensor.turnOn());
  *answerScans = false;

  bool thereIsObs = true;
  bool hwError = false;
  mrpt::obs::CObservation2DRangeScan obs;
  sensor.doProcessSimple(thereIsObs, obs, hwError);
  EXPECT_FALSE(thereIsObs);
  EXPECT_TRUE(hwError);
}
