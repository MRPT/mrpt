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
#include <mrpt/obs/CObservation2DRangeScanWithUncertainty.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationSkeleton.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationStereoImagesFeatures.h>
#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/obs/format_externals_filename.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <sstream>

using namespace mrpt::obs;

// ------------------- CObservationSkeleton -------------------

TEST(CObservationSkeleton, SetFieldsAndGetDescription)
{
  CObservationSkeleton o;
  o.head.x = 1;
  o.head.y = 2;
  o.head.z = 3;
  o.head.conf = 0.9;
  o.sensorPose = mrpt::poses::CPose3D(0.1, 0.2, 0.3, 0, 0, 0);

  EXPECT_NEAR(o.getSensorPose().x(), 0.1, 1e-9);
  o.setSensorPose(mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
  EXPECT_NEAR(o.getSensorPose().x(), 1.0, 1e-9);

  std::stringstream ss;
  o.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CObservationSkeleton, SerializationRoundtrip)
{
  CObservationSkeleton o1;
  o1.head.x = 1.5;
  o1.left_hand.conf = 0.5;
  o1.sensorPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservationSkeleton o2;
  arch >> o2;
  EXPECT_NEAR(o2.head.x, o1.head.x, 1e-9);
  EXPECT_NEAR(o2.left_hand.conf, o1.left_hand.conf, 1e-9);
  EXPECT_NEAR(o2.sensorPose.x(), o1.sensorPose.x(), 1e-9);
}

// ------------------- CObservationStereoImagesFeatures -------------------

TEST(CObservationStereoImagesFeatures, ConstructorAndFields)
{
  mrpt::img::TCamera camLeft;
  mrpt::img::TCamera camRight;
  mrpt::poses::CPose3DQuat rightPose;
  mrpt::poses::CPose3DQuat onRobot;

  CObservationStereoImagesFeatures o(camLeft, camRight, rightPose, onRobot);
  EXPECT_NEAR(o.getSensorPose().x(), 0.0, 1e-9);

  o.setSensorPose(mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0));
  EXPECT_NEAR(o.getSensorPose().x(), 1.0, 1e-9);

  o.setSensorPose(mrpt::poses::CPose3DQuat(mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0)));
  EXPECT_NEAR(o.getSensorPose().x(), 2.0, 1e-9);

  mrpt::poses::CPose3DQuat outPose;
  o.getSensorPose(outPose);
  EXPECT_NEAR(outPose.x(), 2.0, 1e-9);

  TStereoImageFeatures f;
  f.ID = 5;
  f.pixels.first = mrpt::img::TPixelCoordf(1.0f, 2.0f);
  f.pixels.second = mrpt::img::TPixelCoordf(3.0f, 4.0f);
  o.theFeatures.push_back(f);

  std::stringstream ss;
  o.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  const std::string tmpFile = mrpt::system::getTempFileName() + ".txt";
  o.saveFeaturesToTextFile(tmpFile);
  EXPECT_TRUE(mrpt::system::fileExists(tmpFile));
  mrpt::system::deleteFile(tmpFile);
}

TEST(CObservationStereoImagesFeatures, SerializationRoundtrip)
{
  CObservationStereoImagesFeatures o1;
  TStereoImageFeatures f;
  f.ID = 7;
  f.pixels.first = mrpt::img::TPixelCoordf(1, 2);
  f.pixels.second = mrpt::img::TPixelCoordf(3, 4);
  o1.theFeatures.push_back(f);
  o1.cameraLeft.ncols = 640;
  o1.cameraRight.ncols = 640;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservationStereoImagesFeatures o2;
  arch >> o2;
  ASSERT_EQ(o2.theFeatures.size(), 1u);
  EXPECT_EQ(o2.theFeatures[0].ID, 7u);
  EXPECT_EQ(o2.cameraLeft.ncols, 640u);
}

// ------------------- CObservationRobotPose -------------------

TEST(CObservationRobotPose, SetFieldsAndDescription)
{
  CObservationRobotPose o;
  o.sensorPose = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);
  o.pose.mean = mrpt::poses::CPose3D(0.5, 0.5, 0, 0, 0, 0);
  o.pose.cov.setIdentity();

  EXPECT_NEAR(o.getSensorPose().x(), 1.0, 1e-9);
  o.setSensorPose(mrpt::poses::CPose3D(5, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o.getSensorPose().x(), 5.0, 1e-9);

  std::stringstream ss;
  o.getDescriptionAsText(ss);
  EXPECT_FALSE(ss.str().empty());

  ASSERT_TRUE(o.exportTxtSupported());
  const std::string header = o.exportTxtHeader();
  const std::string row = o.exportTxtDataRow();
  EXPECT_FALSE(header.empty());
  EXPECT_FALSE(row.empty());
}

TEST(CObservationRobotPose, SerializationRoundtrip)
{
  CObservationRobotPose o1;
  o1.sensorPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  o1.pose.mean = mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0);
  o1.pose.cov.setIdentity();

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservationRobotPose o2;
  arch >> o2;
  EXPECT_NEAR(o2.sensorPose.x(), o1.sensorPose.x(), 1e-9);
  EXPECT_NEAR(o2.pose.mean.x(), o1.pose.mean.x(), 1e-9);
}

// ------------------- CObservationWindSensor -------------------

TEST(CObservationWindSensor, SetFieldsAndDescription)
{
  CObservationWindSensor o;
  o.speed = 5.5;
  o.direction = 90.0;
  o.sensorPoseOnRobot = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);

  EXPECT_NEAR(o.getSensorPose().x(), 1.0, 1e-9);
  o.setSensorPose(mrpt::poses::CPose3D(2, 0, 0, 0, 0, 0));
  EXPECT_NEAR(o.getSensorPose().x(), 2.0, 1e-9);

  std::stringstream ss;
  o.getDescriptionAsText(ss);

  ASSERT_TRUE(o.exportTxtSupported());
  EXPECT_FALSE(o.exportTxtHeader().empty());
  EXPECT_FALSE(o.exportTxtDataRow().empty());
}

TEST(CObservationWindSensor, SerializationRoundtrip)
{
  CObservationWindSensor o1;
  o1.speed = 3.3;
  o1.direction = 45.0;
  o1.sensorPoseOnRobot = mrpt::poses::CPose3D(1, 2, 3, 0, 0, 0);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservationWindSensor o2;
  arch >> o2;
  EXPECT_DOUBLE_EQ(o2.speed, o1.speed);
  EXPECT_DOUBLE_EQ(o2.direction, o1.direction);
  EXPECT_NEAR(o2.sensorPoseOnRobot.x(), o1.sensorPoseOnRobot.x(), 1e-9);
}

// ------------------- CObservation2DRangeScanWithUncertainty -------------------

TEST(CObservation2DRangeScanWithUncertainty, EvaluateScanLikelihood)
{
  CObservation2DRangeScanWithUncertainty unc;
  const size_t N = 5;
  unc.rangeScan.resizeScan(N);
  unc.rangesMean.resize(N);
  unc.rangesCovar.setZero(N, N);
  for (size_t i = 0; i < N; i++)
  {
    unc.rangeScan.setScanRange(i, 5.0f);
    unc.rangeScan.setScanRangeValidity(i, true);
    unc.rangesMean[i] = 5.0;
    unc.rangesCovar(i, i) = 0.01;
  }

  CObservation2DRangeScan other;
  other.resizeScan(N);
  other.maxRange = 20.0f;
  other.stdError = 0.01f;
  for (size_t i = 0; i < N; i++)
  {
    other.setScanRange(i, 5.0f);
    other.setScanRangeValidity(i, true);
  }

  CObservation2DRangeScanWithUncertainty::TEvalParams params;
  const double lik = unc.evaluateScanLikelihood(other, params);
  EXPECT_GT(lik, 0.0);
  EXPECT_LE(lik, 1.0001);
}

TEST(CObservation2DRangeScanWithUncertainty, EvaluateScanLikelihoodWithOutlierAndInvalidRay)
{
  CObservation2DRangeScanWithUncertainty unc;
  const size_t N = 3;
  unc.rangeScan.resizeScan(N);
  unc.rangesMean.resize(N);
  unc.rangesCovar.setZero(N, N);
  for (size_t i = 0; i < N; i++)
  {
    unc.rangesMean[i] = 5.0;
    unc.rangesCovar(i, i) = 0.01;
  }

  CObservation2DRangeScan other;
  other.resizeScan(N);
  other.maxRange = 20.0f;
  other.stdError = 0.01f;
  // One "shorter" range (outlier branch), one invalid ray (lost-ray branch),
  // one matching:
  other.setScanRange(0, 1.0f);
  other.setScanRangeValidity(0, true);
  other.setScanRange(1, 20.0f);
  other.setScanRangeValidity(1, false);
  other.setScanRange(2, 5.0f);
  other.setScanRangeValidity(2, true);

  CObservation2DRangeScanWithUncertainty::TEvalParams params;
  const double lik = unc.evaluateScanLikelihood(other, params);
  EXPECT_GE(lik, 0.0);
}

// ------------------- format_externals_filename -------------------

TEST(format_externals_filename, TypeAndLabelSubstitution)
{
  CObservationImage img;
  img.sensorLabel = "cam1";
  img.timestamp = mrpt::Clock::now();

  const std::string result = mrpt::obs::format_externals_filename(img, "${type}_${label}");
  EXPECT_EQ(result, "img_cam1");
}

TEST(format_externals_filename, StereoAnd3DTypes)
{
  CObservationStereoImages stereo;
  EXPECT_EQ(mrpt::obs::format_externals_filename(stereo, "${type}"), "stereo");

  CObservation3DRangeScan rgbd;
  EXPECT_EQ(mrpt::obs::format_externals_filename(rgbd, "${type}"), "3dcam");

  CObservationWindSensor other;
  EXPECT_EQ(mrpt::obs::format_externals_filename(other, "${type}"), "other");
}
