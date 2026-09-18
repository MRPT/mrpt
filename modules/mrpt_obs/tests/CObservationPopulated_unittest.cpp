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
#include <mrpt/obs/CObservation3DScene.h>
#include <mrpt/obs/CObservation6DFeatures.h>
#include <mrpt/obs/CObservationBatteryState.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/Scene.h>

#include <sstream>

using namespace mrpt::obs;

// ------------------- CObservationBearingRange -------------------

TEST(CObservationBearingRange, PopulateDescribeAndRoundtrip)
{
  CObservationBearingRange o1;
  o1.minSensorDistance = 0.5f;
  o1.maxSensorDistance = 10.0f;
  o1.sensorLocationOnRobot = mrpt::poses::CPose3D(0.1, 0, 0, 0, 0, 0);
  o1.validCovariances = true;

  CObservationBearingRange::TMeasurement m;
  m.range = 3.0f;
  m.yaw = 0.1f;
  m.pitch = 0.0f;
  m.landmarkID = 7;
  m.covariance.setIdentity();
  o1.sensedData.push_back(m);
  o1.sensor_std_range = 0.01f;

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());
  o1.debugPrintOut();

  EXPECT_TRUE(o1.exportTxtSupported());
  EXPECT_FALSE(o1.exportTxtHeader().empty());
  EXPECT_FALSE(o1.exportTxtDataRow().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 0.1, 1e-6);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationBearingRange o2;
  arch >> o2;
  ASSERT_EQ(o2.sensedData.size(), 1u);
  EXPECT_EQ(o2.sensedData[0].landmarkID, 7);
  EXPECT_TRUE(o2.validCovariances);
}

// ------------------- CObservationBeaconRanges -------------------

TEST(CObservationBeaconRanges, PopulateDescribeAndRoundtrip)
{
  CObservationBeaconRanges o1;
  o1.minSensorDistance = 0.1f;
  o1.maxSensorDistance = 20.0f;
  o1.stdError = 0.05f;

  CObservationBeaconRanges::TMeasurement m;
  m.sensorLocationOnRobot = mrpt::poses::CPoint3D(0.2, 0, 0);
  m.sensedDistance = 5.0f;
  m.beaconID = 3;
  o1.sensedData.push_back(m);

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_NEAR(o1.getSensedRangeByBeaconID(3), 5.0f, 1e-4f);
  EXPECT_NEAR(o1.getSensedRangeByBeaconID(999), 0.0f, 1e-6f);

  EXPECT_TRUE(o1.exportTxtSupported());
  EXPECT_FALSE(o1.exportTxtHeader().empty());
  EXPECT_FALSE(o1.exportTxtDataRow().empty());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationBeaconRanges o2;
  arch >> o2;
  ASSERT_EQ(o2.sensedData.size(), 1u);
  EXPECT_EQ(o2.sensedData[0].beaconID, 3);
}

// ------------------- CObservationRange -------------------

TEST(CObservationRangeTest, PopulateDescribeAndRoundtrip)
{
  CObservationRange o1;
  o1.minSensorDistance = 0.1f;
  o1.maxSensorDistance = 4.0f;
  o1.sensorConeAperture = 0.3f;

  CObservationRange::TMeasurement m;
  m.sensorID = 2;
  m.sensorPose = mrpt::math::TPose3D(1, 0, 0, 0, 0, 0);
  m.sensedDistance = 2.5f;
  m.sensorNoiseStdDeviation = 0.02f;
  o1.sensedData.push_back(m);

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 1.0, 1e-6);
  o1.setSensorPose(mrpt::poses::CPose3D(5, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o1.getSensorPose().x(), 5.0, 1e-6);

  size_t count = 0;
  for (auto it = o1.begin(); it != o1.end(); ++it) count++;
  EXPECT_EQ(count, 1u);
  const CObservationRange& co1 = o1;
  count = 0;
  for (auto it = co1.begin(); it != co1.end(); ++it) count++;
  EXPECT_EQ(count, 1u);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationRange o2;
  arch >> o2;
  ASSERT_EQ(o2.sensedData.size(), 1u);
  EXPECT_EQ(o2.sensedData[0].sensorID, 2);
}

// ------------------- CObservation6DFeatures -------------------

TEST(CObservation6DFeatures, PopulateDescribeAndRoundtrip)
{
  CObservation6DFeatures o1;
  o1.minSensorDistance = 0.5f;
  o1.maxSensorDistance = 15.0f;

  CObservation6DFeatures::TMeasurement m;
  m.id = 9;
  m.pose = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
  o1.sensedFeatures.push_back(m);

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservation6DFeatures o2;
  arch >> o2;
  ASSERT_EQ(o2.sensedFeatures.size(), 1u);
  EXPECT_EQ(o2.sensedFeatures[0].id, 9);
}

// ------------------- CObservationOdometry -------------------

TEST(CObservationOdometryTest, PopulateDescribeAndRoundtrip)
{
  CObservationOdometry o1;
  o1.odometry = mrpt::poses::CPose2D(1, 2, 0.1);
  o1.hasEncodersInfo = true;
  o1.encoderLeftTicks = 100;
  o1.encoderRightTicks = 105;
  o1.hasVelocities = true;
  o1.velocityLocal.vx = 0.5;
  o1.velocityLocal.omega = 0.1;

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_TRUE(o1.exportTxtSupported());
  EXPECT_FALSE(o1.exportTxtHeader().empty());
  EXPECT_FALSE(o1.exportTxtDataRow().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 0.0, 1e-9);
  o1.setSensorPose(mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0));  // no-op, just for coverage

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationOdometry o2;
  arch >> o2;
  EXPECT_NEAR(o2.odometry.x(), 1.0, 1e-6);
  EXPECT_EQ(o2.encoderLeftTicks, 100);
  EXPECT_TRUE(o2.hasVelocities);
}

// ------------------- CObservationBatteryState -------------------

TEST(CObservationBatteryState, PopulateDescribeAndRoundtrip)
{
  CObservationBatteryState o1;
  o1.voltageMainRobotBattery = 12.5;
  o1.voltageMainRobotComputer = 5.0;
  o1.voltageMainRobotBatteryIsValid = true;
  o1.voltageMainRobotComputerIsValid = false;
  o1.voltageOtherBatteries.resize(2);
  o1.voltageOtherBatteries[0] = 3.3;
  o1.voltageOtherBatteries[1] = 1.5;
  o1.voltageOtherBatteriesValid = {true, false};

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_TRUE(o1.exportTxtSupported());
  EXPECT_FALSE(o1.exportTxtHeader().empty());
  EXPECT_FALSE(o1.exportTxtDataRow().empty());

  o1.setSensorPose(mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
  const auto pose = o1.getSensorPose();
  (void)pose;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationBatteryState o2;
  arch >> o2;
  EXPECT_NEAR(o2.voltageMainRobotBattery, 12.5, 1e-6);
  EXPECT_TRUE(o2.voltageMainRobotBatteryIsValid);
  ASSERT_EQ(o2.voltageOtherBatteries.size(), 2u);
}

// ------------------- CObservationRFID -------------------

TEST(CObservationRFIDTest, PopulateDescribeAndRoundtrip)
{
  CObservationRFID o1;
  o1.sensorPoseOnRobot = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  CObservationRFID::TTagReading tag;
  tag.power = -50.0;
  tag.epc = "EPC12345";
  tag.antennaPort = "ant1";
  o1.tag_readings.push_back(tag);

  EXPECT_EQ(o1.getNtags(), 1u);

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 1.0, 1e-6);
  o1.setSensorPose(mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o1.getSensorPose().x(), 2.0, 1e-6);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationRFID o2;
  arch >> o2;
  ASSERT_EQ(o2.tag_readings.size(), 1u);
  EXPECT_EQ(o2.tag_readings[0].epc, "EPC12345");
}

// ------------------- CObservationWirelessPower -------------------

TEST(CObservationWirelessPowerTest, PopulateDescribeAndRoundtrip)
{
  CObservationWirelessPower o1;
  o1.power = 42.0;
  o1.sensorPoseOnRobot = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_TRUE(o1.exportTxtSupported());
  EXPECT_FALSE(o1.exportTxtHeader().empty());
  EXPECT_FALSE(o1.exportTxtDataRow().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 1.0, 1e-6);
  o1.setSensorPose(mrpt::poses::CPose3D(3, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o1.getSensorPose().x(), 3.0, 1e-6);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationWirelessPower o2;
  arch >> o2;
  EXPECT_NEAR(o2.power, 42.0, 1e-6);
}

// ------------------- CObservationReflectivity -------------------

TEST(CObservationReflectivityTest, PopulateDescribeAndRoundtrip)
{
  CObservationReflectivity o1;
  o1.reflectivityLevel = 0.7f;
  o1.channel = 3;
  o1.sensorPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  o1.sensorStdNoise = 0.1f;

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_TRUE(o1.exportTxtSupported());
  EXPECT_FALSE(o1.exportTxtHeader().empty());
  EXPECT_FALSE(o1.exportTxtDataRow().empty());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationReflectivity o2;
  arch >> o2;
  EXPECT_NEAR(o2.reflectivityLevel, 0.7f, 1e-6f);
  EXPECT_EQ(o2.channel, 3);
}

// ------------------- CObservationImage -------------------

TEST(CObservationImageTest, PopulateDescribeAndRoundtrip)
{
  CObservationImage o1;
  o1.cameraPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  mrpt::obs::stock_observations::exampleImage(o1.image);
  o1.cameraParams.ncols = o1.image.getWidth();
  o1.cameraParams.nrows = o1.image.getHeight();

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  mrpt::img::CImage undistorted;
  o1.getUndistortedImage(undistorted);
  EXPECT_GT(undistorted.getWidth(), 0u);

  o1.load();
  o1.unload();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationImage o2;
  arch >> o2;
  EXPECT_EQ(o2.image.getWidth(), o1.image.getWidth());
}

// ------------------- CObservationStereoImages -------------------

TEST(CObservationStereoImagesTest, PopulateDescribeSwapAndRoundtrip)
{
  CObservationStereoImages o1;
  mrpt::obs::stock_observations::exampleImage(o1.imageLeft, 0);
  mrpt::obs::stock_observations::exampleImage(o1.imageRight, 1);
  o1.hasImageRight = true;
  o1.cameraPose = mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0));
  o1.rightCameraPose = mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(0.1, 0, 0, 0, 0, 0));
  o1.leftCamera.ncols = o1.imageLeft.getWidth();
  o1.leftCamera.nrows = o1.imageLeft.getHeight();
  o1.rightCamera = o1.leftCamera;

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  mrpt::img::TStereoCamera sc;
  o1.getStereoCameraParams(sc);
  const auto sc2 = o1.getStereoCameraParams();
  EXPECT_EQ(sc.leftCamera.ncols, sc2.leftCamera.ncols);

  mrpt::img::TStereoCamera newParams = sc;
  o1.setStereoCameraParams(newParams);

  // Default (zero) distortion coefficients mean "already rectified":
  EXPECT_TRUE(o1.areImagesRectified());
  o1.leftCamera.dist[0] = 0.1;
  EXPECT_FALSE(o1.areImagesRectified());

  EXPECT_NEAR(o1.getSensorPose().x(), 1.0, 1e-6);
  o1.setSensorPose(mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o1.getSensorPose().x(), 2.0, 1e-6);

  CObservationStereoImages o1b;
  o1.swap(o1b);
  EXPECT_TRUE(o1b.hasImageRight);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1b;
  buf.Seek(0);
  CObservationStereoImages o2;
  arch >> o2;
  EXPECT_TRUE(o2.hasImageRight);
  EXPECT_EQ(o2.imageLeft.getWidth(), o1b.imageLeft.getWidth());
}

// ------------------- CObservation3DScene -------------------

TEST(CObservation3DSceneTest, PopulateDescribeAndVisualize)
{
  CObservation3DScene o1;
  o1.scene = mrpt::viz::Scene::Create();
  o1.sensorPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 1.0, 1e-6);
  o1.setSensorPose(mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o1.getSensorPose().x(), 2.0, 1e-6);

  auto container = mrpt::viz::CSetOfObjects::Create();
  o1.getVisualizationInto(*container);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservation3DScene o2;
  arch >> o2;
  ASSERT_TRUE(o2.scene);
}

// ------------------- CObservationRGBD360 -------------------

TEST(CObservationRGBD360Test, PopulateDescribeAndRoundtrip)
{
  CObservationRGBD360 o1;
  o1.sensorPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  o1.maxRange = 8.0f;
  o1.hasRangeImage = true;
  o1.rangeImage_setSize(4, 4, 0);
  o1.rangeImage_setSize(4, 4, 1);
  o1.hasIntensityImage = true;
  for (unsigned i = 0; i < CObservationRGBD360::NUM_SENSORS; i++)
  {
    o1.timestamps[i] = mrpt::Clock::now();
    o1.intensityImages[i] = mrpt::img::CImage(4, 4, mrpt::img::CH_RGB);
  }

  std::stringstream ss;
  o1.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  EXPECT_NEAR(o1.getSensorPose().x(), 1.0, 1e-6);
  o1.setSensorPose(mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o1.getSensorPose().x(), 2.0, 1e-6);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);
  CObservationRGBD360 o2;
  arch >> o2;
  EXPECT_TRUE(o2.hasRangeImage);
  EXPECT_EQ(o2.rangeImages[0].rows(), 4);
}
