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

// Exercises the backwards-compatibility branches of serializeFrom() for the
// observation/action classes that still support older streaming versions.

#include "legacy_serialization.h"

#include <gtest/gtest.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPosePDFGaussian.h>

using namespace mrpt::obs;
using namespace mrpt::serialization;
using mrpt_test::writeLegacyObjectFrame;

namespace
{
template <class T>
std::shared_ptr<T> readBackAs(mrpt::io::CMemoryStream& buf)
{
  auto arch = mrpt::serialization::archiveFrom(buf);
  return std::dynamic_pointer_cast<T>(arch.ReadObject());
}

// Common tail of the version 2..3 CActionRobotMovement2D payload
void writeVelocitiesAndEncodersV2V3(CArchive& a)
{
  a << bool(true);    // hasVelocities
  a << float(1.5f);   // linear velocity
  a << float(0.25f);  // angular velocity
  a << bool(true);    // hasEncodersInfo
  a << int32_t(100);  // left ticks
  a << int32_t(200);  // right ticks
}

void writeOdometryModelConfigV4Plus(CArchive& a, uint8_t version)
{
  a << mrpt::poses::CPose2D(1.0, 0.5, 0.1);  // rawOdometryIncrementReading
  a << int32_t(0);                           // modelSelection == mmGaussian
  a << float(0.02f) << float(0.03f) << float(0.04f) << float(0.05f);  // a1..a4
  a << float(0.01f) << float(0.02f);                                  // minStdXY, minStdPHI
  a << int32_t(50);                                                   // nParticlesCount
  a << float(0.1f) << float(0.2f) << float(0.3f) << float(0.4f);      // alfa1..alfa4
  if (version >= 5) a << float(0.001f) << float(0.002f);              // additional_std_XY / _phi
}
}  // namespace

TEST(LegacySerialization, CActionRobotMovement2D_v7_odometry)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "CActionRobotMovement2D", 7,
      [](CArchive& a)
      {
        a << int32_t(CActionRobotMovement2D::emOdometry);
        writeOdometryModelConfigV4Plus(a, 7);
        a << bool(true);
        a << mrpt::math::TTwist2D(1.0, 0.0, 0.5);
        a << bool(true);
        a << int32_t(10) << int32_t(20);
        a << mrpt::Clock::now();
      });

  const auto act = readBackAs<CActionRobotMovement2D>(buf);
  ASSERT_TRUE(act);
  EXPECT_EQ(act->estimationMethod, CActionRobotMovement2D::emOdometry);
  EXPECT_NEAR(act->rawOdometryIncrementReading.x(), 1.0, 1e-6);
  EXPECT_TRUE(act->hasVelocities);
  EXPECT_NEAR(act->velocityLocal.vx, 1.0, 1e-6);
  EXPECT_TRUE(act->hasEncodersInfo);
  EXPECT_EQ(act->encoderLeftTicks, 10);
  EXPECT_EQ(act->motionModelConfiguration.thrunModel.nParticlesCount, 50U);
}

TEST(LegacySerialization, CActionRobotMovement2D_v7_noVelocitiesNoEncoders)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "CActionRobotMovement2D", 7,
      [](CArchive& a)
      {
        a << int32_t(CActionRobotMovement2D::emScan2DMatching);
        // The PDF is streamed directly for any method other than odometry:
        const mrpt::poses::CPosePDFGaussian pdf(mrpt::poses::CPose2D(2.0, 3.0, 0.0));
        a << static_cast<const mrpt::serialization::CSerializable&>(pdf);
        a << bool(false);  // hasVelocities
        a << bool(false);  // hasEncodersInfo
        a << mrpt::Clock::now();
      });

  const auto act = readBackAs<CActionRobotMovement2D>(buf);
  ASSERT_TRUE(act);
  EXPECT_EQ(act->estimationMethod, CActionRobotMovement2D::emScan2DMatching);
  EXPECT_FALSE(act->hasVelocities);
  EXPECT_FALSE(act->hasEncodersInfo);
  EXPECT_EQ(act->encoderLeftTicks, 0);
  mrpt::poses::CPose2D mean;
  act->poseChange->getMean(mean);
  EXPECT_NEAR(mean.x(), 2.0, 1e-6);
}

TEST(LegacySerialization, CActionRobotMovement2D_v4_v5_v6)
{
  for (const uint8_t version : {uint8_t(4), uint8_t(5), uint8_t(6)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CActionRobotMovement2D", version,
        [version](CArchive& a)
        {
          a << int32_t(CActionRobotMovement2D::emOdometry);
          writeOdometryModelConfigV4Plus(a, version);
          a << bool(true);
          a << float(1.0f) << float(0.5f);  // pre-v7: separate linear/angular
          a << bool(true);
          a << int32_t(3) << int32_t(4);
          if (version >= 6) a << mrpt::Clock::now();
        });

    const auto act = readBackAs<CActionRobotMovement2D>(buf);
    ASSERT_TRUE(act) << "version " << int(version);
    EXPECT_NEAR(act->velocityLocal.vx, 1.0, 1e-6);
    EXPECT_NEAR(act->velocityLocal.vy, 0.0, 1e-6);
    EXPECT_NEAR(act->velocityLocal.omega, 0.5, 1e-6);
    EXPECT_EQ(act->encoderRightTicks, 4);
    if (version < 6)
    {
      EXPECT_EQ(act->timestamp, INVALID_TIMESTAMP);
    }
    // The additional Thrun std devs default to zero before version 5:
    if (version < 5)
    {
      EXPECT_NEAR(act->motionModelConfiguration.thrunModel.additional_std_XY, 0.0f, 1e-9f);
    }
  }
}

TEST(LegacySerialization, CActionRobotMovement2D_v3)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "CActionRobotMovement2D", 3,
      [](CArchive& a)
      {
        a << int32_t(CActionRobotMovement2D::emOdometry);
        a << mrpt::poses::CPose2D(1.0, 0.0, 0.0);
        a << int32_t(0);  // modelSelection
        // 3 discarded floats + minStdXY + minStdPHI (as doubles)
        a << float(0) << float(0) << float(0);
        a << double(0.01) << double(0.02);
        a << int32_t(70);  // nParticlesCount
        a << float(0.1f) << float(0.2f) << float(0.3f) << float(0.4f);
        writeVelocitiesAndEncodersV2V3(a);
      });

  const auto act = readBackAs<CActionRobotMovement2D>(buf);
  ASSERT_TRUE(act);
  EXPECT_EQ(act->motionModelConfiguration.thrunModel.nParticlesCount, 70U);
  EXPECT_NEAR(act->velocityLocal.vx, 1.5, 1e-6);
  EXPECT_EQ(act->encoderRightTicks, 200);
}

TEST(LegacySerialization, CActionRobotMovement2D_v2)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "CActionRobotMovement2D", 2,
      [](CArchive& a)
      {
        a << int32_t(CActionRobotMovement2D::emOdometry);
        a << mrpt::poses::CPose2D(0.5, 0.25, 0.0);
        // A 44-byte blob of obsolete options, read and discarded:
        const uint8_t dummy[44] = {0};
        a.WriteBuffer(dummy, sizeof(dummy));
        writeVelocitiesAndEncodersV2V3(a);
      });

  const auto act = readBackAs<CActionRobotMovement2D>(buf);
  ASSERT_TRUE(act);
  EXPECT_NEAR(act->rawOdometryIncrementReading.x(), 0.5, 1e-6);
  // The options are reset to their defaults:
  EXPECT_EQ(act->motionModelConfiguration.thrunModel.nParticlesCount, 300U);
}

TEST(LegacySerialization, CActionRobotMovement2D_v0_v1)
{
  for (const uint8_t version : {uint8_t(0), uint8_t(1)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CActionRobotMovement2D", version,
        [version](CArchive& a)
        {
          const mrpt::poses::CPosePDFGaussian pdf(mrpt::poses::CPose2D(1.0, 2.0, 0.0));
          a << static_cast<const mrpt::serialization::CSerializable&>(pdf);
          a << int32_t(CActionRobotMovement2D::emOdometry);
          if (version >= 1) writeVelocitiesAndEncodersV2V3(a);
        });

    const auto act = readBackAs<CActionRobotMovement2D>(buf);
    ASSERT_TRUE(act) << "version " << int(version);
    // The raw odometry is reconstructed as the mean of the stored PDF:
    EXPECT_NEAR(act->rawOdometryIncrementReading.x(), 1.0, 1e-6);
    if (version == 0)
    {
      EXPECT_FALSE(act->hasVelocities);
      EXPECT_FALSE(act->hasEncodersInfo);
      EXPECT_EQ(act->encoderLeftTicks, 0);
    }
    else
    {
      EXPECT_TRUE(act->hasEncodersInfo);
      EXPECT_EQ(act->encoderLeftTicks, 100);
    }
  }
}

TEST(LegacySerialization, CActionRobotMovement2D_unknownVersion)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(buf, "CActionRobotMovement2D", 200, [](CArchive&) {});

  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(), std::exception);
}

TEST(LegacySerialization, CObservationStereoImages_v5)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "CObservationStereoImages", 5,
      [](CArchive& a)
      {
        a << mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
        mrpt::img::TCamera cam;
        a << cam;                                          // leftCamera
        a << cam;                                          // rightCamera
        a << mrpt::img::CImage(8, 8, mrpt::img::CH_GRAY);  // imageLeft
        a << mrpt::img::CImage(8, 8, mrpt::img::CH_GRAY);  // imageRight
        a << mrpt::Clock::now();
        a << mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(0.12, 0, 0, 0, 0, 0));
        a << std::string("stereo_v5");
      });

  const auto obs = readBackAs<CObservationStereoImages>(buf);
  ASSERT_TRUE(obs);
  EXPECT_TRUE(obs->hasImageRight);
  EXPECT_FALSE(obs->hasImageDisparity);
  EXPECT_EQ(obs->sensorLabel, "stereo_v5");
  EXPECT_NEAR(obs->rightCameraPose.x(), 0.12, 1e-6);
}

TEST(LegacySerialization, CObservationStereoImages_v0_to_v4)
{
  for (const uint8_t version : {uint8_t(0), uint8_t(1), uint8_t(2), uint8_t(3), uint8_t(4)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservationStereoImages", version,
        [version](CArchive& a)
        {
          a << mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);  // cameraPose (as CPose3D)
          mrpt::math::CMatrixF intParams(3, 3);
          intParams.setIdentity();
          a << intParams;
          a << mrpt::img::CImage(8, 8, mrpt::img::CH_GRAY);
          a << mrpt::img::CImage(8, 8, mrpt::img::CH_GRAY);
          if (version >= 1) a << mrpt::Clock::now();
          if (version >= 2) a << mrpt::poses::CPose3D(0.2, 0, 0, 0, 0, 0);
          if (version >= 3) a << double(0.004);  // focal length in meters
          if (version >= 4) a << std::string("stereo_old");
        });

    const auto obs = readBackAs<CObservationStereoImages>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    EXPECT_TRUE(obs->hasImageRight);
    EXPECT_NEAR(obs->cameraPose.x(), 1.0, 1e-6);

    if (version < 1)
    {
      EXPECT_EQ(obs->timestamp, INVALID_TIMESTAMP);
    }
    if (version < 2)
    {
      EXPECT_NEAR(obs->rightCameraPose.x(), 0.10, 1e-5);
    }
    if (version >= 2)
    {
      EXPECT_NEAR(obs->rightCameraPose.x(), 0.2, 1e-5);
    }
    if (version >= 3)
    {
      EXPECT_NEAR(obs->leftCamera.focalLengthMeters, 0.004, 1e-9);
    }
    if (version < 3)
    {
      EXPECT_NEAR(obs->leftCamera.focalLengthMeters, 0.002, 1e-9);
    }
    EXPECT_EQ(obs->sensorLabel, version >= 4 ? "stereo_old" : "");
  }
}

TEST(LegacySerialization, CObservationStereoImages_unknownVersion)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(buf, "CObservationStereoImages", 200, [](CArchive&) {});

  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(), std::exception);
}

TEST(LegacySerialization, CObservationImage_v0_to_v3)
{
  for (const uint8_t version : {uint8_t(0), uint8_t(1), uint8_t(2), uint8_t(3)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservationImage", version,
        [version](CArchive& a)
        {
          a << mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
          // Before v4, the distortion and intrinsic matrices were streamed:
          mrpt::math::CMatrixF dist(1, 5);
          dist.fill(0.1f);
          mrpt::math::CMatrixF intr(3, 3);
          intr.setIdentity();
          a << dist << intr;
          a << mrpt::img::CImage(4, 4, mrpt::img::CH_GRAY);
          if (version >= 1) a << mrpt::Clock::now();
          if (version >= 2) a << double(0.005);
          if (version >= 3) a << std::string("cam_old");
        });

    const auto obs = readBackAs<CObservationImage>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    EXPECT_NEAR(obs->cameraPose.x(), 1.0, 1e-6);
    EXPECT_NEAR(obs->cameraParams.dist[0], 0.1, 1e-5);
    EXPECT_NEAR(obs->cameraParams.intrinsicParams(1, 1), 1.0, 1e-9);
    EXPECT_NEAR(obs->cameraParams.focalLengthMeters, version >= 2 ? 0.005 : 0.002, 1e-9);
    EXPECT_EQ(obs->sensorLabel, version >= 3 ? "cam_old" : "");
  }
}

TEST(LegacySerialization, CObservationImage_v0_wrongSizedDistortion)
{
  // A distortion matrix that is not 1x5 leaves the parameters at zero:
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(
      buf, "CObservationImage", 0,
      [](CArchive& a)
      {
        a << mrpt::poses::CPose3D();
        mrpt::math::CMatrixF dist(1, 4);
        dist.fill(0.5f);
        mrpt::math::CMatrixF intr(3, 3);
        intr.setIdentity();
        a << dist << intr;
        a << mrpt::img::CImage(4, 4, mrpt::img::CH_GRAY);
      });

  const auto obs = readBackAs<CObservationImage>(buf);
  ASSERT_TRUE(obs);
  for (int i = 0; i < 5; i++) EXPECT_NEAR(obs->cameraParams.dist[i], 0.0, 1e-12);
}

TEST(LegacySerialization, CObservationImage_unknownVersion)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(buf, "CObservationImage", 200, [](CArchive&) {});
  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(), std::exception);
}

TEST(LegacySerialization, CObservationBeaconRanges_v0_to_v2)
{
  for (const uint8_t version : {uint8_t(0), uint8_t(1), uint8_t(2)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservationBeaconRanges", version,
        [version](CArchive& a)
        {
          a << float(0.1f) << float(30.0f) << float(0.05f);  // min/max/stdError
          a << uint32_t(2);                                  // number of readings
          for (uint32_t i = 0; i < 2; i++)
          {
            a << mrpt::poses::CPoint3D(0.1 * i, 0, 0);
            a << float(5.0f + static_cast<float>(i));
            a << uint32_t(100 + i);
          }
          if (version >= 1) a << mrpt::poses::CPose2D(1, 2, 0);
          if (version >= 2) a << std::string("beacons_old");
        });

    const auto obs = readBackAs<CObservationBeaconRanges>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    ASSERT_EQ(obs->sensedData.size(), 2U);
    EXPECT_EQ(obs->sensedData[1].beaconID, 101U);
    EXPECT_NEAR(obs->sensedData[1].sensedDistance, 6.0f, 1e-5f);
    if (version >= 1)
    {
      EXPECT_NEAR(obs->auxEstimatePose.y(), 2.0, 1e-6);
    }
    EXPECT_EQ(obs->sensorLabel, version >= 2 ? "beacons_old" : "");
    EXPECT_EQ(obs->timestamp, INVALID_TIMESTAMP);
  }
}

TEST(LegacySerialization, CObservationBeaconRanges_unknownVersion)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(buf, "CObservationBeaconRanges", 200, [](CArchive&) {});
  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(), std::exception);
}

TEST(LegacySerialization, CObservationIMU_v0_to_v3)
{
  for (const uint8_t version : {uint8_t(0), uint8_t(1), uint8_t(2), uint8_t(3)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservationIMU", version,
        [version](CArchive& a)
        {
          a << mrpt::poses::CPose3D(0, 0, 1, 0, 0, 0);
          const std::vector<bool> present(mrpt::obs::COUNT_IMU_DATA_FIELDS, true);
          a << present;
          a << mrpt::Clock::now();
          if (version < 1)
          {
            mrpt::math::CVectorFloat v(mrpt::obs::COUNT_IMU_DATA_FIELDS);
            for (int i = 0; i < v.size(); i++) v[i] = static_cast<float>(i);
            a << v;
          }
          else
          {
            std::vector<double> v(mrpt::obs::COUNT_IMU_DATA_FIELDS);
            for (size_t i = 0; i < v.size(); i++) v[i] = static_cast<double>(i);
            a << v;
          }
          a << std::string("imu_old");
        });

    const auto obs = readBackAs<CObservationIMU>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    EXPECT_EQ(obs->sensorLabel, "imu_old");
    EXPECT_NEAR(obs->sensorPose.z(), 1.0, 1e-6);
    EXPECT_TRUE(obs->dataIsPresent[mrpt::obs::IMU_X_ACC]);
    EXPECT_NEAR(obs->rawMeasurements[mrpt::obs::IMU_X_ACC], 0.0, 1e-9);

    // Before v2 the yaw and roll rates were stored swapped and are fixed up:
    if (version < 2)
    {
      EXPECT_NEAR(
          obs->rawMeasurements[mrpt::obs::IMU_YAW_VEL],
          static_cast<double>(mrpt::obs::IMU_ROLL_VEL), 1e-9);
    }
    else
    {
      EXPECT_NEAR(
          obs->rawMeasurements[mrpt::obs::IMU_YAW_VEL], static_cast<double>(mrpt::obs::IMU_YAW_VEL),
          1e-9);
    }
  }
}

TEST(LegacySerialization, CObservationGasSensors_v2_to_v4)
{
  for (const uint8_t version : {uint8_t(2), uint8_t(3), uint8_t(4)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservationGasSensors", version,
        [version](CArchive& a)
        {
          a << uint32_t(1);  // one e-nose
          a << mrpt::poses::CPose3D(0.2, -0.15, 0.1, 0, 0, 0);
          mrpt::math::CVectorFloat volts(2);
          volts[0] = 1.0f;
          volts[1] = 2.0f;
          a << volts;
          a << std::vector<int>{10, 20};
          if (version >= 3)
          {
            a << bool(true);
            a << float(25.5f);
          }
          if (version >= 4) a << std::string("enose_old");
        });

    const auto obs = readBackAs<CObservationGasSensors>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    ASSERT_EQ(obs->m_readings.size(), 1U);
    EXPECT_EQ(obs->m_readings[0].readingsVoltage.size(), 2U);
    EXPECT_EQ(obs->m_readings[0].sensorTypes[1], 20);
    if (version >= 3)
    {
      EXPECT_TRUE(obs->m_readings[0].hasTemperature);
      EXPECT_NEAR(obs->m_readings[0].temperature, 25.5f, 1e-5f);
    }
    else
    {
      EXPECT_FALSE(obs->m_readings[0].hasTemperature);
    }
    EXPECT_EQ(obs->sensorLabel, version >= 4 ? "enose_old" : "");
    EXPECT_EQ(obs->timestamp, INVALID_TIMESTAMP);
  }
}

TEST(LegacySerialization, CObservationGasSensors_v0_v1)
{
  for (const uint8_t version : {uint8_t(0), uint8_t(1)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservationGasSensors", version,
        [](CArchive& a)
        {
          // The oldest format: a single block of 16 raw values
          mrpt::math::CVectorFloat readings(16);
          for (int i = 0; i < readings.size(); i++) readings[i] = static_cast<float>(i);
          a << readings;
        });

    const auto obs = readBackAs<CObservationGasSensors>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    // The 16 raw values are split into the two e-noses of that setup:
    EXPECT_EQ(obs->m_readings.size(), 2U);
  }
}

namespace
{
// Payload of the pre-v9 CObservation3DRangeScan format (range image as a
// float matrix, no extra layers).
void write3DScanLegacyPayload(CArchive& a, uint8_t version)
{
  a << float(10.0f);                            // maxRange
  a << mrpt::poses::CPose3D(0, 0, 1, 0, 0, 0);  // sensorPose

  if (version > 0) a << bool(true);  // hasPoints3D
  const uint32_t N = 3;
  a << N;
  const float xs[N] = {1.0f, 2.0f, 3.0f};
  const float ys[N] = {0.0f, 0.0f, 0.0f};
  const float zs[N] = {0.5f, 0.5f, 0.5f};
  a.WriteBufferFixEndianness(xs, N);
  a.WriteBufferFixEndianness(ys, N);
  a.WriteBufferFixEndianness(zs, N);
  if (version == 0)
  {
    const char validRange[N] = {1, 1, 1};
    a.WriteBuffer(validRange, sizeof(validRange));
  }
  if (version >= 8)
  {
    const uint16_t idxs[N] = {0, 1, 2};
    a.WriteBufferFixEndianness(idxs, N);
    a.WriteBufferFixEndianness(idxs, N);
  }

  if (version >= 1)
  {
    a << bool(true);  // hasRangeImage
    // Pre-v9: a plain float matrix of ranges
    mrpt::math::CMatrixF ri(2, 2);
    ri(0, 0) = 1.0f;
    ri(0, 1) = 2.0f;
    ri(1, 0) = 3.0f;
    ri(1, 1) = 4.0f;
    a << ri;

    a << bool(false);  // hasIntensityImage
    a << bool(false);  // hasConfidenceImage

    if (version >= 2)
    {
      mrpt::img::TCamera cam;
      a << cam;
      if (version >= 4)
      {
        a << cam;
        a << mrpt::poses::CPose3D();
      }
    }
  }

  a << float(0.01f);          // stdError
  a << mrpt::Clock::now();    // timestamp
  a << std::string("depth");  // sensorLabel

  if (version >= 3)
  {
    a << bool(false) << std::string();  // points3D external storage
    a << bool(false) << std::string();  // rangeImage external storage
  }
  if (version >= 5) a << bool(true);   // range_is_depth
  if (version >= 6) a << int8_t(0);    // intensityImageChannel
  if (version >= 7) a << bool(false);  // no pixel labels
}
}  // namespace

TEST(LegacySerialization, CObservation3DRangeScan_v0_to_v8)
{
  for (const uint8_t version :
       {uint8_t(0), uint8_t(1), uint8_t(2), uint8_t(3), uint8_t(4), uint8_t(5), uint8_t(6),
        uint8_t(7), uint8_t(8)})
  {
    mrpt::io::CMemoryStream buf;
    writeLegacyObjectFrame(
        buf, "CObservation3DRangeScan", version,
        [version](CArchive& a) { write3DScanLegacyPayload(a, version); });

    const auto obs = readBackAs<mrpt::obs::CObservation3DRangeScan>(buf);
    ASSERT_TRUE(obs) << "version " << int(version);
    EXPECT_TRUE(obs->hasPoints3D);
    EXPECT_EQ(obs->points3D_x.size(), 3U);
    EXPECT_NEAR(obs->points3D_x[2], 3.0f, 1e-5f);

    if (version >= 1)
    {
      EXPECT_TRUE(obs->hasRangeImage);
      EXPECT_EQ(obs->rangeImage.rows(), 2);
      EXPECT_EQ(obs->rangeImage.cols(), 2);
      // Pre-v9 files stored ranges in meters, converted to the fixed-point
      // representation with the default 1mm units:
      EXPECT_NEAR(obs->rangeImage(0, 1) * obs->rangeUnits, 2.0f, 1e-3f);
      // The camera resolution is auto-fixed to match the range image:
      EXPECT_EQ(obs->cameraParams.ncols, 2U);
      EXPECT_EQ(obs->cameraParams.nrows, 2U);
    }
    EXPECT_EQ(obs->sensorLabel, "depth");
    if (version < 5)
    {
      EXPECT_TRUE(obs->range_is_depth);
    }
  }
}

TEST(LegacySerialization, CObservation3DRangeScan_unknownVersion)
{
  mrpt::io::CMemoryStream buf;
  writeLegacyObjectFrame(buf, "CObservation3DRangeScan", 200, [](CArchive&) {});
  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(), std::exception);
}
