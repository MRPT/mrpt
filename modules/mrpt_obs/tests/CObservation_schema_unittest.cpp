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
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/serialization/config.h>

#include <sstream>
#include <string>

#if MRPT_HAS_JSONCPP

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

#endif  // MRPT_HAS_JSONCPP
