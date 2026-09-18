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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <cstring>
#include <sstream>

using namespace mrpt::obs;

namespace
{
CObservationGasSensors::TObservationENose makeENose()
{
  CObservationGasSensors::TObservationENose n;
  n.eNosePoseOnTheRobot = mrpt::math::TPose3D(0.1, 0.2, 0.3, 0, 0, 0);
  n.readingsVoltage = {1.0f, 2.0f};
  n.sensorTypes = {0x2600, 0x2602};
  n.hasTemperature = true;
  n.temperature = 25.5f;
  n.isActive = true;
  return n;
}
}  // namespace

TEST(CObservationGasSensors, GetSetSensorPose)
{
  CObservationGasSensors o;
  // No readings -> default pose at origin:
  EXPECT_NEAR(o.getSensorPose().x(), 0.0, 1e-9);

  o.m_readings.push_back(makeENose());
  EXPECT_NEAR(o.getSensorPose().x(), 0.1, 1e-6);

  o.setSensorPose(mrpt::poses::CPose3D(5, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o.getSensorPose().x(), 5.0, 1e-6);
}

TEST(CObservationGasSensors, GetDescriptionAsTextWithAndWithoutTemperature)
{
  CObservationGasSensors o;
  auto n1 = makeENose();
  o.m_readings.push_back(n1);
  auto n2 = makeENose();
  n2.hasTemperature = false;
  o.m_readings.push_back(n2);

  std::stringstream ss;
  o.getDescriptionAsText(ss);
  const std::string txt = ss.str();
  EXPECT_NE(txt.find("degC"), std::string::npos);
  EXPECT_NE(txt.find("NOT AVAILABLE"), std::string::npos);
}

TEST(CObservationGasSensors, SerializationRoundtripCurrentVersion)
{
  CObservationGasSensors o1;
  o1.m_readings.push_back(makeENose());
  auto n2 = makeENose();
  n2.hasTemperature = false;
  o1.m_readings.push_back(n2);
  o1.sensorLabel = "enose";
  o1.timestamp = mrpt::Clock::now();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservationGasSensors o2;
  arch >> o2;

  ASSERT_EQ(o2.m_readings.size(), 2u);
  EXPECT_NEAR(o2.m_readings[0].readingsVoltage[0], 1.0f, 1e-6f);
  EXPECT_TRUE(o2.m_readings[0].hasTemperature);
  EXPECT_FALSE(o2.m_readings[1].hasTemperature);
  EXPECT_EQ(o2.sensorLabel, "enose");
}

namespace
{
void writeObjectHeader(mrpt::serialization::CArchive& arch, const char* className, uint8_t version)
{
  const auto len = static_cast<int8_t>(strlen(className) | 0x80);
  arch << len;
  arch.WriteBuffer(className, strlen(className));
  arch << version;
}
void writeObjectFooter(mrpt::serialization::CArchive& arch) { arch << static_cast<uint8_t>(0x88); }
}  // namespace

TEST(CObservationGasSensors, DeserializeLegacyVersion0FixedSanchoFormat)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CObservationGasSensors", 0);

  mrpt::math::CVectorFloat readings;
  readings.resize(16);
  for (int i = 0; i < 16; i++) readings[i] = static_cast<float>(i);
  arch << readings;

  writeObjectFooter(arch);
  buf.Seek(0);

  CObservationGasSensors o;
  arch >> o;

  ASSERT_EQ(o.m_readings.size(), 2u);
  EXPECT_NEAR(o.m_readings[0].readingsVoltage[0], readings[2], 1e-6f);
  EXPECT_NEAR(o.m_readings[1].readingsVoltage[0], readings[8], 1e-6f);
}

TEST(CObservationGasSensors, DeserializeLegacyVersion2NoLabelNoTimestamp)
{
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  writeObjectHeader(arch, "CObservationGasSensors", 2);

  const uint32_t n = 1;
  arch << n;
  arch << mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
  std::vector<float> voltages = {0.5f, 0.6f};
  arch << voltages;
  std::vector<int> types = {1, 2};
  arch << types;
  // version < 3: no hasTemperature field

  writeObjectFooter(arch);
  buf.Seek(0);

  CObservationGasSensors o;
  arch >> o;

  ASSERT_EQ(o.m_readings.size(), 1u);
  EXPECT_FALSE(o.m_readings[0].hasTemperature);
  EXPECT_EQ(o.sensorLabel, "");
}

// ------------------- CMOSmodel -------------------

TEST(CObservationGasSensorsCMOSmodel, GetGasDistributionEstimationDecimatesAndEstimates)
{
  CObservationGasSensors::CMOSmodel model;
  model.decimate_value = 2;
  model.winNoise_size = 4;
  model.a_rise = 1.0f;
  model.b_rise = 0.5f;
  model.a_decay = 1.0f;
  model.b_decay = 0.5f;

  const auto t0 = mrpt::Clock::now();
  bool gotEstimation = false;
  float lastReading = 0;
  mrpt::system::TTimeStamp lastTs = t0;

  for (int i = 0; i < 12; i++)
  {
    float reading = 1.0f + 0.1f * static_cast<float>(i);
    mrpt::system::TTimeStamp ts = t0 + std::chrono::milliseconds(100 * i);
    if (model.get_GasDistribution_estimation(reading, ts))
    {
      gotEstimation = true;
      lastReading = reading;
      lastTs = ts;
    }
  }
  EXPECT_TRUE(gotEstimation);
  // These are simply "used" to avoid unused-variable warnings and to confirm
  // valid finite results were produced:
  EXPECT_TRUE(std::isfinite(lastReading));
  EXPECT_NE(lastTs, mrpt::system::TTimeStamp());
}

TEST(CObservationGasSensorsCMOSmodel, DecayBranch)
{
  CObservationGasSensors::CMOSmodel model;
  model.decimate_value = 1;
  model.winNoise_size = 2;

  const auto t0 = mrpt::Clock::now();
  float r1 = 5.0f;
  mrpt::system::TTimeStamp ts1 = t0;
  model.get_GasDistribution_estimation(r1, ts1);

  // A decreasing reading should exercise the "decay" branch (tau=a_decay*...):
  float r2 = 1.0f;
  mrpt::system::TTimeStamp ts2 = t0 + std::chrono::milliseconds(100);
  const bool ok = model.get_GasDistribution_estimation(r2, ts2);
  EXPECT_TRUE(ok);
}

TEST(CObservationGasSensorsCMOSmodel, SaveMaplogCreatesFile)
{
  CObservationGasSensors::CMOSmodel model;
  model.decimate_value = 1;
  model.winNoise_size = 2;
  model.save_maplog = true;

  const auto t0 = mrpt::Clock::now();
  float r = 1.0f;
  mrpt::system::TTimeStamp ts = t0;
  model.get_GasDistribution_estimation(r, ts);
  r = 2.0f;
  ts = t0 + std::chrono::milliseconds(50);
  model.get_GasDistribution_estimation(r, ts);

  const std::string logFile = "./log_MOSmodel_GasDistribution.txt";
  EXPECT_TRUE(mrpt::system::fileExists(logFile));
  mrpt::system::deleteFile(logFile);
}
