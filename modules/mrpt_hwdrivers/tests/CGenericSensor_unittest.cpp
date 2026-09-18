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
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/registerAllClasses.h>

#include <memory>
#include <string>

using namespace mrpt::hwdrivers;

TEST(CGenericSensor, factoryKnowsTheRegisteredDrivers)
{
  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();

  // A few drivers that are always built, whatever the optional deps:
  for (const char* name : {"CHokuyoURG", "CGPSInterface", "CSickLaserSerial", "CVelodyneScanner"})
  {
    std::unique_ptr<CGenericSensor> s(CGenericSensor::createSensor(name));
    ASSERT_TRUE(s) << "createSensor failed for: " << name;
    EXPECT_EQ(std::string(s->GetRuntimeClass()->className), std::string(name));
  }
}

TEST(CGenericSensor, factoryReturnsNullForUnknownClasses)
{
  EXPECT_EQ(CGenericSensor::createSensor("NoSuchSensorClass"), nullptr);
  EXPECT_EQ(CGenericSensor::createSensor(""), nullptr);
}

TEST(CGenericSensor, everyRegisteredClassIsConstructible)
{
  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();

  // Every driver registered by registerAllClasses_mrpt_hwdrivers(). They are
  // all registered whether or not their optional SDK is available, so a few
  // of them refuse to be built at all in a stripped-down configuration; the
  // rest must construct *and* destruct cleanly without initialize() having
  // been called, which is what rawlog-grabber does before configuring them.
  const char* allClasses[] = {"CSickLaserUSB",    "CIbeoLuxETH",
                              "CHokuyoURG",       "CRoboPeakLidar",
                              "CGPSInterface",    "CIMUXSens_MT4",
                              "CCameraSensor",    "CWirelessPower",
                              "CRaePID",          "CImpinjRFID",
                              "CSickLaserSerial", "CEnoseModular",
                              "CGillAnemometer",  "CNTRIPEmitter",
                              "CLMS100Eth",       "CPhidgetInterfaceKitProximitySensors",
                              "CGyroKVHDSP3000",  "CKinect",
                              "COpenNI2Sensor",   "COpenNI2_RGBD360",
                              "CCANBusReader",    "CNationalInstrumentsDAQ",
                              "CGPS_NTRIP",       "CVelodyneScanner",
                              "CSICKTim561Eth",   "CTaoboticsIMU"};

  size_t nBuilt = 0;
  for (const char* name : allClasses)
  {
    std::unique_ptr<CGenericSensor> s;
    try
    {
      s.reset(CGenericSensor::createSensor(name));
    }
    catch (const std::exception&)
    {
      // Built without the SDK this driver needs: it says so and refuses.
      continue;
    }
    ASSERT_TRUE(s) << "createSensor failed for registered class: " << name;
    nBuilt++;
    EXPECT_EQ(std::string(s->GetRuntimeClass()->className), std::string(name));
    // A freshly built sensor is idle and holds nothing:
    EXPECT_TRUE(s->getObservations().empty()) << name;
    EXPECT_EQ(s->getState(), CGenericSensor::ssInitializing) << name;
  }
  EXPECT_GT(nBuilt, 15U);
}

TEST(CGenericSensor, loadConfigReadsTheCommonParameters)
{
  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();
  std::unique_ptr<CGenericSensor> s(CGenericSensor::createSensor("CGPSInterface"));
  ASSERT_TRUE(s);

  mrpt::config::CConfigFileMemory cfg;
  const std::string sec = "SENSOR";
  cfg.write(sec, "process_rate", 25.0);
  cfg.write(sec, "max_queue_len", 500);
  cfg.write(sec, "grab_decimation", 3);
  cfg.write(sec, "sensorLabel", "my_sensor");
  // Mandatory for this particular driver:
#ifdef _WIN32
  cfg.write(sec, "COM_port_WIN", "COM1");
#else
  cfg.write(sec, "COM_port_LIN", "/dev/ttyUSB0");
#endif
  cfg.write(sec, "baudRate", 4800);

  s->loadConfig(cfg, sec);

  EXPECT_NEAR(s->getProcessRate(), 25.0, 1e-9);
  EXPECT_EQ(s->getSensorLabel(), "my_sensor");
}

TEST(CGenericSensor, sensorLabelRoundTrips)
{
  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();
  std::unique_ptr<CGenericSensor> s(CGenericSensor::createSensor("CHokuyoURG"));
  ASSERT_TRUE(s);

  s->setSensorLabel("laser_front");
  EXPECT_EQ(s->getSensorLabel(), "laser_front");
  EXPECT_EQ(s->getState(), CGenericSensor::ssInitializing);
}

TEST(CGenericSensor, missingMandatoryEntryThrows)
{
  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();
  std::unique_ptr<CGenericSensor> s(CGenericSensor::createSensor("CGPSInterface"));
  ASSERT_TRUE(s);

  // No COM port given, which this driver requires:
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("SENSOR", "process_rate", 10.0);
  EXPECT_THROW(s->loadConfig(cfg, "SENSOR"), std::exception);
}
