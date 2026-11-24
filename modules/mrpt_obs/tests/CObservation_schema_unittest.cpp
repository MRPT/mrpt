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

// Schema (JSON) serialization of observation classes (MRPT issue #820).

#include <gtest/gtest.h>
#include <mrpt/containers/config.h>  // MRPT_HAS_FYAML
#include <mrpt/obs/CObservationBatteryState.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/serialization/CSchemeArchive.h>

#include <array>
#include <sstream>
#include <string>

#if MRPT_HAS_FYAML

namespace
{
using mrpt::obs::CObservationIMU;
using mrpt::obs::CObservationOdometry;

// A timestamp with sub-microsecond ticks, to check the exact round-trip:
const auto kStamp = mrpt::Clock::fromDouble(1700000000.0) + mrpt::Clock::duration(1234567);

CObservationIMU makeIMU()
{
  CObservationIMU o;
  o.sensorLabel = "imu_front";
  o.timestamp = kStamp;
  o.sensorPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.1, -0.2, 0.3, 0.5, -0.1, 0.05);
  o.set(mrpt::obs::IMU_X_ACC, 0.25);
  o.set(mrpt::obs::IMU_Y_ACC, -0.5);
  o.set(mrpt::obs::IMU_Z_ACC, 9.81);
  o.set(mrpt::obs::IMU_WZ, 0.0125);
  o.set(mrpt::obs::IMU_ORI_QUAT_W, 1.0);
  o.set(mrpt::obs::IMU_Z_ACC_GLOBAL, -9.80665);  // last enum entry
  return o;
}

void expectSamePose(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b)
{
  EXPECT_NEAR(a.x(), b.x(), 1e-12);
  EXPECT_NEAR(a.y(), b.y(), 1e-12);
  EXPECT_NEAR(a.z(), b.z(), 1e-12);
  EXPECT_NEAR(a.yaw(), b.yaw(), 1e-9);
  EXPECT_NEAR(a.pitch(), b.pitch(), 1e-9);
  EXPECT_NEAR(a.roll(), b.roll(), 1e-9);
}

void expectSameIMU(const CObservationIMU& a, const CObservationIMU& b)
{
  EXPECT_EQ(a.sensorLabel, b.sensorLabel);
  EXPECT_EQ(a.timestamp, b.timestamp);
  expectSamePose(a.sensorPose, b.sensorPose);
  for (size_t i = 0; i < mrpt::obs::COUNT_IMU_DATA_FIELDS; i++)
  {
    const auto idx = static_cast<mrpt::obs::TIMUDataIndex>(i);
    EXPECT_EQ(a.has(idx), b.has(idx)) << "field index " << i;
    if (a.has(idx) && b.has(idx))
    {
      EXPECT_EQ(a.get(idx), b.get(idx)) << "field index " << i;
    }
  }
}

std::string toJSONText(const mrpt::serialization::CSerializable& obj)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch = obj;
  std::stringstream ss;
  ss << arch;
  return ss.str();
}

template <typename T>
T fromJSONText(const std::string& txt)
{
  auto arch = mrpt::serialization::archiveJSON();
  std::stringstream ss(txt);
  ss >> arch;
  T out;
  arch.readTo(out);
  return out;
}
}  // namespace

TEST(CObservationIMU, SchemaRoundTrip)
{
  const auto src = makeIMU();

  auto arch = mrpt::serialization::archiveJSON();
  arch = src;
  CObservationIMU dst;
  arch.readTo(dst);

  expectSameIMU(src, dst);
}

TEST(CObservationIMU, SchemaRoundTripThroughText)
{
  const auto src = makeIMU();
  const std::string txt = toJSONText(src);

  // Fields are written by name, and only if present:
  EXPECT_NE(txt.find("\"IMU_X_ACC\""), std::string::npos) << txt;
  EXPECT_NE(txt.find("\"IMU_YAW_VEL\""), std::string::npos) << txt;
  EXPECT_NE(txt.find("\"IMU_Z_ACC_GLOBAL\""), std::string::npos) << txt;
  EXPECT_EQ(txt.find("\"IMU_X_VEL\""), std::string::npos) << txt;
  EXPECT_NE(txt.find("\"imu_front\""), std::string::npos) << txt;
  EXPECT_EQ(txt.find("_count"), std::string::npos) << txt;

  expectSameIMU(src, fromJSONText<CObservationIMU>(txt));
}

TEST(CObservationIMU, SchemaNoMeasurements)
{
  CObservationIMU src;
  src.sensorLabel = "empty";
  src.timestamp = kStamp;

  // Start from a non-empty object to check that stale data gets cleared:
  auto dst = makeIMU();
  auto arch = mrpt::serialization::archiveJSON();
  arch = src;
  arch.readTo(dst);

  expectSameIMU(src, dst);
}

TEST(CObservationIMU, SchemaUnknownFieldThrows)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch = makeIMU();
  arch["measurements"][static_cast<size_t>(0)]["field"] = std::string("IMU_NOT_A_FIELD");

  CObservationIMU dst;
  EXPECT_ANY_THROW(arch.readTo(dst));
}

TEST(CObservationOdometry, SchemaRoundTripFull)
{
  CObservationOdometry src;
  src.sensorLabel = "odom";
  src.timestamp = kStamp;
  src.odometry = mrpt::poses::CPose2D(1.5, -2.25, 0.75);
  src.hasEncodersInfo = true;
  src.encoderLeftTicks = -1200;
  src.encoderRightTicks = 3400;
  src.hasVelocities = true;
  src.velocityLocal = mrpt::math::TTwist2D(0.5, -0.05, 0.2);

  const auto dst = fromJSONText<CObservationOdometry>(toJSONText(src));

  EXPECT_EQ(dst.sensorLabel, src.sensorLabel);
  EXPECT_EQ(dst.timestamp, src.timestamp);
  EXPECT_NEAR(dst.odometry.x(), 1.5, 1e-12);
  EXPECT_NEAR(dst.odometry.y(), -2.25, 1e-12);
  EXPECT_NEAR(dst.odometry.phi(), 0.75, 1e-12);
  EXPECT_TRUE(dst.hasEncodersInfo);
  EXPECT_EQ(dst.encoderLeftTicks, -1200);
  EXPECT_EQ(dst.encoderRightTicks, 3400);
  EXPECT_TRUE(dst.hasVelocities);
  EXPECT_EQ(dst.velocityLocal.vx, 0.5);
  EXPECT_EQ(dst.velocityLocal.vy, -0.05);
  EXPECT_EQ(dst.velocityLocal.omega, 0.2);
}

TEST(CObservationOdometry, SchemaRoundTripPoseOnly)
{
  CObservationOdometry src;
  src.timestamp = kStamp;
  src.odometry = mrpt::poses::CPose2D(-3.0, 4.0, -1.0);

  // Pre-filled destination: optional data must be reset, not kept.
  CObservationOdometry dst;
  dst.hasEncodersInfo = true;
  dst.encoderLeftTicks = 7;
  dst.hasVelocities = true;
  dst.velocityLocal = mrpt::math::TTwist2D(1.0, 1.0, 1.0);

  auto arch = mrpt::serialization::archiveJSON();
  arch = src;
  arch.readTo(dst);

  EXPECT_EQ(dst.timestamp, src.timestamp);
  EXPECT_NEAR(dst.odometry.x(), -3.0, 1e-12);
  EXPECT_NEAR(dst.odometry.phi(), -1.0, 1e-12);
  EXPECT_FALSE(dst.hasEncodersInfo);
  EXPECT_EQ(dst.encoderLeftTicks, 0);
  EXPECT_FALSE(dst.hasVelocities);
  EXPECT_EQ(dst.velocityLocal.vx, 0.0);
  EXPECT_EQ(dst.velocityLocal.omega, 0.0);
}

TEST(CObservationOdometry, SchemaReadingIMUDocumentThrows)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch = makeIMU();

  CObservationOdometry dst;
  EXPECT_ANY_THROW(arch.readTo(dst));
}

namespace
{
const auto kPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.4, -0.1, 0.25, 1.2, 0.05, -0.3);

void expectSameCommon(const mrpt::obs::CObservation& a, const mrpt::obs::CObservation& b)
{
  EXPECT_EQ(a.sensorLabel, b.sensorLabel);
  EXPECT_EQ(a.timestamp, b.timestamp);
}

/** Round trip into a destination object that already holds other data, to
 * check that list contents are replaced, not appended to. */
template <typename T>
T roundTripInto(const T& src, T dst)
{
  auto arch = mrpt::serialization::archiveJSON();
  std::stringstream ss(toJSONText(src));
  ss >> arch;
  arch.readTo(dst);
  return dst;
}
}  // namespace

TEST(CObservationRange, SchemaRoundTrip)
{
  mrpt::obs::CObservationRange src;
  src.sensorLabel = "sonars";
  src.timestamp = kStamp;
  src.minSensorDistance = 0.05f;
  src.maxSensorDistance = 7.5f;
  src.sensorConeAperture = 0.3f;
  for (const uint16_t id : std::array<uint16_t, 2>{3, 65535})
  {
    mrpt::obs::CObservationRange::TMeasurement m;
    m.sensorID = id;
    m.sensorPose = mrpt::math::TPose3D(0.1 * id, -0.2, 0.05, 0.3, -0.1, 0.2);
    m.sensedDistance = 1.25f + id;
    m.sensorNoiseStdDeviation = 0.02f;
    src.sensedData.push_back(m);
  }

  mrpt::obs::CObservationRange stale;
  stale.sensedData.resize(5);
  const auto dst = roundTripInto(src, stale);

  expectSameCommon(src, dst);
  EXPECT_EQ(dst.minSensorDistance, src.minSensorDistance);
  EXPECT_EQ(dst.maxSensorDistance, src.maxSensorDistance);
  EXPECT_EQ(dst.sensorConeAperture, src.sensorConeAperture);
  ASSERT_EQ(dst.sensedData.size(), 2U);
  for (size_t i = 0; i < 2; i++)
  {
    const auto& a = src.sensedData[i];
    const auto& b = dst.sensedData[i];
    EXPECT_EQ(a.sensorID, b.sensorID);
    expectSamePose(mrpt::poses::CPose3D(a.sensorPose), mrpt::poses::CPose3D(b.sensorPose));
    EXPECT_EQ(a.sensedDistance, b.sensedDistance);
    EXPECT_EQ(a.sensorNoiseStdDeviation, b.sensorNoiseStdDeviation);
  }

  // Empty measurement list, still written as such:
  src.sensedData.clear();
  const std::string txt = toJSONText(src);
  EXPECT_NE(txt.find("\"sensedData\": []"), std::string::npos) << txt;
  EXPECT_TRUE(roundTripInto(src, stale).sensedData.empty());
}

TEST(CObservationRange, SchemaSensorIDOutOfRangeThrows)
{
  mrpt::obs::CObservationRange src;
  src.sensedData.resize(1);
  auto arch = mrpt::serialization::archiveJSON();
  arch = src;
  arch["sensedData"][static_cast<size_t>(0)]["sensorID"] = static_cast<uint32_t>(65536);

  mrpt::obs::CObservationRange dst;
  EXPECT_ANY_THROW(arch.readTo(dst));
}

TEST(CObservationBatteryState, SchemaRoundTrip)
{
  mrpt::obs::CObservationBatteryState src;
  src.sensorLabel = "battery";
  src.timestamp = kStamp;
  src.voltageMainRobotBattery = 22.2;
  src.voltageMainRobotComputer = 12.05;
  src.voltageMainRobotBatteryIsValid = true;
  src.voltageMainRobotComputerIsValid = false;
  src.voltageOtherBatteries.resize(3);
  src.voltageOtherBatteries[0] = 3.7;
  src.voltageOtherBatteries[1] = 3.65;
  src.voltageOtherBatteries[2] = 0.0;
  src.voltageOtherBatteriesValid = {true, true, false};

  mrpt::obs::CObservationBatteryState stale;
  stale.voltageOtherBatteries.resize(8);
  stale.voltageOtherBatteriesValid.assign(8, true);
  const auto dst = roundTripInto(src, stale);

  expectSameCommon(src, dst);
  EXPECT_EQ(dst.voltageMainRobotBattery, src.voltageMainRobotBattery);
  EXPECT_EQ(dst.voltageMainRobotComputer, src.voltageMainRobotComputer);
  EXPECT_TRUE(dst.voltageMainRobotBatteryIsValid);
  EXPECT_FALSE(dst.voltageMainRobotComputerIsValid);
  ASSERT_EQ(dst.voltageOtherBatteries.size(), 3);
  for (int i = 0; i < 3; i++)
  {
    EXPECT_EQ(dst.voltageOtherBatteries[i], src.voltageOtherBatteries[i]);
  }
  EXPECT_EQ(dst.voltageOtherBatteriesValid, src.voltageOtherBatteriesValid);
}

TEST(CObservationComment, SchemaRoundTrip)
{
  mrpt::obs::CObservationComment src;
  src.sensorLabel = "notes";
  src.timestamp = kStamp;
  src.text = "Line 1: \"quoted\", tab\tand unicode \xC3\xB1\nLine 2";

  const auto dst = roundTripInto(src, mrpt::obs::CObservationComment());
  expectSameCommon(src, dst);
  EXPECT_EQ(dst.text, src.text);
}

TEST(CObservationWindSensor, SchemaRoundTrip)
{
  mrpt::obs::CObservationWindSensor src;
  src.sensorLabel = "anemometer";
  src.timestamp = kStamp;
  src.speed = 6.75;
  src.direction = 247.5;
  src.sensorPoseOnRobot = kPose;

  const auto dst = roundTripInto(src, mrpt::obs::CObservationWindSensor());
  expectSameCommon(src, dst);
  EXPECT_EQ(dst.speed, src.speed);
  EXPECT_EQ(dst.direction, src.direction);
  expectSamePose(dst.sensorPoseOnRobot, src.sensorPoseOnRobot);
}

TEST(CObservationWirelessPower, SchemaRoundTrip)
{
  mrpt::obs::CObservationWirelessPower src;
  src.sensorLabel = "wifi";
  src.timestamp = kStamp;
  src.power = -67.5;
  src.sensorPoseOnRobot = kPose;

  const auto dst = roundTripInto(src, mrpt::obs::CObservationWirelessPower());
  expectSameCommon(src, dst);
  EXPECT_EQ(dst.power, src.power);
  expectSamePose(dst.sensorPoseOnRobot, src.sensorPoseOnRobot);
}

TEST(CObservationRFID, SchemaRoundTrip)
{
  mrpt::obs::CObservationRFID src;
  src.sensorLabel = "rfid";
  src.timestamp = kStamp;
  src.sensorPoseOnRobot = kPose;
  src.tag_readings.resize(2);
  src.tag_readings[0].power = -45.25;
  src.tag_readings[0].epc = "3000E2003412DC03011956118401";
  src.tag_readings[0].antennaPort = "1";
  src.tag_readings[1].power = -1000;
  src.tag_readings[1].epc = "";
  src.tag_readings[1].antennaPort = "ant-2";

  mrpt::obs::CObservationRFID stale;
  stale.tag_readings.resize(4);
  const auto dst = roundTripInto(src, stale);

  expectSameCommon(src, dst);
  expectSamePose(dst.sensorPoseOnRobot, src.sensorPoseOnRobot);
  ASSERT_EQ(dst.tag_readings.size(), 2U);
  for (size_t i = 0; i < 2; i++)
  {
    EXPECT_EQ(dst.tag_readings[i].power, src.tag_readings[i].power);
    EXPECT_EQ(dst.tag_readings[i].epc, src.tag_readings[i].epc);
    EXPECT_EQ(dst.tag_readings[i].antennaPort, src.tag_readings[i].antennaPort);
  }
}

TEST(CObservationBeaconRanges, SchemaRoundTrip)
{
  mrpt::obs::CObservationBeaconRanges src;
  src.sensorLabel = "uwb";
  src.timestamp = kStamp;
  src.minSensorDistance = 0.2f;
  src.maxSensorDistance = 50.0f;
  src.stdError = 0.05f;
  for (int32_t id : {7, -1})
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement m;
    m.sensorLocationOnRobot = mrpt::poses::CPoint3D(0.1, 0.2 * id, 0.3);
    m.sensedDistance = 3.5f + static_cast<float>(id);
    m.beaconID = id;
    src.sensedData.push_back(m);
  }
  src.auxEstimatePose = mrpt::poses::CPose2D(1.0, -2.0, 0.5);

  mrpt::obs::CObservationBeaconRanges stale;
  stale.sensedData.resize(3);
  const auto dst = roundTripInto(src, stale);

  expectSameCommon(src, dst);
  EXPECT_EQ(dst.minSensorDistance, src.minSensorDistance);
  EXPECT_EQ(dst.maxSensorDistance, src.maxSensorDistance);
  EXPECT_EQ(dst.stdError, src.stdError);
  ASSERT_EQ(dst.sensedData.size(), 2U);
  for (size_t i = 0; i < 2; i++)
  {
    const auto& a = src.sensedData[i];
    const auto& b = dst.sensedData[i];
    EXPECT_NEAR(a.sensorLocationOnRobot.x(), b.sensorLocationOnRobot.x(), 1e-12);
    EXPECT_NEAR(a.sensorLocationOnRobot.y(), b.sensorLocationOnRobot.y(), 1e-12);
    EXPECT_NEAR(a.sensorLocationOnRobot.z(), b.sensorLocationOnRobot.z(), 1e-12);
    EXPECT_EQ(a.sensedDistance, b.sensedDistance);
    EXPECT_EQ(a.beaconID, b.beaconID);
  }
  EXPECT_NEAR(dst.auxEstimatePose.x(), 1.0, 1e-12);
  EXPECT_NEAR(dst.auxEstimatePose.y(), -2.0, 1e-12);
  EXPECT_NEAR(dst.auxEstimatePose.phi(), 0.5, 1e-12);
}

#endif  // MRPT_HAS_FYAML
