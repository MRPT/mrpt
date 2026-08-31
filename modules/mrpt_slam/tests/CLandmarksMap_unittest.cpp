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

/** Unit tests for mrpt::maps::CLandmarksMap: its container accessors, the
 *  minimal CMetricMap interface it implements, serialization, and the
 *  range-bearing sensor simulator.
 */

#include <gtest/gtest.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/archiveFrom_std_streams.h>

#include <cmath>
#include <sstream>
#include <tuple>

using mrpt::maps::CLandmark;
using mrpt::maps::CLandmarksMap;

namespace
{
CLandmarksMap makeMap(const std::vector<mrpt::math::TPoint3D>& pts)
{
  CLandmarksMap m;
  int64_t id = 100;
  for (const auto& p : pts)
  {
    CLandmark lm;
    lm.ID = id++;
    lm.pose_mean = p;
    lm.seenTimesCount = 3;
    m.landmarks.push_back(lm);
  }
  return m;
}

/** A bearing-range observation with a wide-open sensor (everything visible). */
mrpt::obs::CObservationBearingRange makeWideOpenObs()
{
  mrpt::obs::CObservationBearingRange o;
  o.minSensorDistance = 0;
  o.maxSensorDistance = 1e6f;
  o.fieldOfView_yaw = 2 * M_PIf;
  o.fieldOfView_pitch = 2 * M_PIf;
  return o;
}
}  // namespace

TEST(CLandmarksMap, containerAccessors)
{
  auto m = makeMap({
      {1, 0, 0},
      {0, 2, 0},
      {0, 0, 3}
  });

  EXPECT_EQ(m.size(), 3U);
  EXPECT_FALSE(m.isEmpty());
  EXPECT_EQ(m.asString(), "CLandmarksMap");

  EXPECT_EQ(m.landmarks.get(1)->ID, 101);
  const auto& cm = m;
  EXPECT_EQ(cm.landmarks.get(1)->ID, 101);

  EXPECT_THROW(std::ignore = m.landmarks.get(3), std::exception);
  EXPECT_THROW(std::ignore = cm.landmarks.get(3), std::exception);

  ASSERT_TRUE(cm.landmarks.getByBeaconID(102) != nullptr);
  EXPECT_NEAR(cm.landmarks.getByBeaconID(102)->pose_mean.z, 3.0, 1e-9);
  EXPECT_EQ(cm.landmarks.getByBeaconID(999), nullptr);

  // Range-for over both const and non-const iterators:
  size_t n = 0;
  for (auto& lm : m.landmarks)
  {
    lm.seenTimesCount++;
    n++;
  }
  for (const auto& lm : cm.landmarks)
  {
    EXPECT_EQ(lm.seenTimesCount, 4U);
  }
  EXPECT_EQ(n, 3U);

  m.clear();  // CMetricMap::clear() -> internal_clear()
  EXPECT_EQ(m.size(), 0U);
  EXPECT_TRUE(m.isEmpty());
}

TEST(CLandmarksMap, metricMapInterfaceIsInert)
{
  auto m = makeMap({
      {1, 0, 0}
  });
  mrpt::obs::CObservationOdometry obs;

  EXPECT_FALSE(m.insertObservation(obs));
  EXPECT_FALSE(m.canComputeObservationLikelihood(obs));
  EXPECT_EQ(m.computeObservationLikelihood(obs, mrpt::poses::CPose3D()), 0.0);

  mrpt::viz::CSetOfObjects objs;
  m.getVisualizationInto(objs);
  EXPECT_EQ(objs.size(), 0U);

  // A no-op, but it must not throw:
  EXPECT_NO_THROW(m.saveMetricMapRepresentationToFile("unused"));
}

TEST(CLandmarksMap, serializationRoundTrip)
{
  auto m = makeMap({
      { 1, 2,  3},
      {-4, 5, -6}
  });
  m.landmarks.get(0)->pose_cov_11 = 0.5;
  m.landmarks.get(0)->pose_cov_22 = 0.25;
  m.landmarks.get(0)->pose_cov_33 = 0.125;
  m.landmarks.get(0)->pose_cov_12 = 0.1;
  m.landmarks.get(0)->pose_cov_13 = 0.2;
  m.landmarks.get(0)->pose_cov_23 = 0.3;

  std::stringstream ss;
  auto arch = mrpt::serialization::archiveFrom<std::iostream>(ss);
  arch << m;

  CLandmarksMap m2;
  // Not empty beforehand, to check that serializeFrom() clears it:
  m2.landmarks.push_back(CLandmark());
  arch >> m2;

  ASSERT_EQ(m2.size(), m.size());
  for (size_t i = 0; i < m.size(); i++)
  {
    const auto& a = *m.landmarks.get(i);
    const auto& b = *m2.landmarks.get(i);
    EXPECT_EQ(a.ID, b.ID);
    EXPECT_NEAR(a.pose_mean.x, b.pose_mean.x, 1e-9);
    EXPECT_NEAR(a.pose_mean.y, b.pose_mean.y, 1e-9);
    EXPECT_NEAR(a.pose_mean.z, b.pose_mean.z, 1e-9);
    EXPECT_NEAR(a.pose_cov_12, b.pose_cov_12, 1e-9);
    EXPECT_NEAR(a.pose_cov_13, b.pose_cov_13, 1e-9);
    EXPECT_NEAR(a.pose_cov_23, b.pose_cov_23, 1e-9);
    EXPECT_EQ(a.seenTimesCount, b.seenTimesCount);
  }
}

TEST(CLandmarksMap, simulateNoiselessGeometry)
{
  // One landmark 5 m ahead of the robot, 1 m to its left:
  const auto m = makeMap({
      {5.0, 1.0, 0.0}
  });

  auto obs = makeWideOpenObs();
  std::vector<size_t> assoc;
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs,
      true /*ids*/, 0, 0, 0, &assoc);

  ASSERT_EQ(obs.sensedData.size(), 1U);
  EXPECT_EQ(obs.sensedData[0].landmarkID, 100);
  EXPECT_NEAR(obs.sensedData[0].range, std::sqrt(26.0), 1e-4);
  EXPECT_NEAR(obs.sensedData[0].yaw, std::atan2(1.0, 5.0), 1e-5);
  EXPECT_NEAR(obs.sensedData[0].pitch, 0.0, 1e-5);
  EXPECT_FALSE(obs.validCovariances);
  ASSERT_EQ(assoc.size(), 1U);
  EXPECT_EQ(assoc[0], 0U);

  // The very same landmark seen from a robot already at it: zero range.
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(5.0, 1.0, 0.0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs);
  ASSERT_EQ(obs.sensedData.size(), 1U);
  EXPECT_NEAR(obs.sensedData[0].range, 0.0, 1e-5);
}

TEST(CLandmarksMap, simulateHonorsSensorPoseOnRobot)
{
  const auto m = makeMap({
      {5.0, 0.0, 0.0}
  });

  auto obs = makeWideOpenObs();
  // Sensor 1 m ahead of the robot origin => 1 m closer to the landmark:
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(1.0, 0, 0, 0, 0, 0), obs);

  ASSERT_EQ(obs.sensedData.size(), 1U);
  EXPECT_NEAR(obs.sensedData[0].range, 4.0, 1e-4);
  EXPECT_NEAR(obs.sensorLocationOnRobot.x(), 1.0, 1e-9);
}

TEST(CLandmarksMap, simulateAnonymousLandmarks)
{
  const auto m = makeMap({
      {5.0, 0.0, 0.0}
  });

  auto obs = makeWideOpenObs();
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs,
      false /* sensorDetectsIDs */);

  ASSERT_EQ(obs.sensedData.size(), 1U);
  EXPECT_EQ(obs.sensedData[0].landmarkID, mrpt::obs::INVALID_LANDMARK_ID);
}

TEST(CLandmarksMap, simulateFiltersOutOfRangeAndOutOfFOV)
{
  const auto m = makeMap({
      {  1.0, 0.0, 0.0}, // too close
      {  5.0, 0.0, 0.0}, // visible
      {100.0, 0.0, 0.0}, // too far
      { -5.0, 0.0, 0.0}, // behind the sensor: out of the yaw FOV
      {  0.0, 0.0, 5.0}, // straight up: out of the pitch FOV
  });

  mrpt::obs::CObservationBearingRange obs;
  obs.minSensorDistance = 2.0f;
  obs.maxSensorDistance = 50.0f;
  obs.fieldOfView_yaw = mrpt::DEG2RAD(90.0f);
  obs.fieldOfView_pitch = mrpt::DEG2RAD(20.0f);

  std::vector<size_t> assoc;
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs, true, 0,
      0, 0, &assoc);

  ASSERT_EQ(obs.sensedData.size(), 1U);
  EXPECT_EQ(obs.sensedData[0].landmarkID, 101);
  ASSERT_EQ(assoc.size(), 1U);
  EXPECT_EQ(assoc[0], 1U);
}

TEST(CLandmarksMap, simulateNoiseIsAppliedAndRangeStaysNonNegative)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  const auto m = makeMap({
      {0.001, 0.0, 0.0}
  });
  auto obs = makeWideOpenObs();

  bool anyDifferent = false;
  for (int i = 0; i < 50; i++)
  {
    m.simulateRangeBearingReadings(
        mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs, true,
        1.0 /*sigmaRange*/, 0.05 /*sigmaYaw*/, 0.05 /*sigmaPitch*/);
    if (obs.sensedData.empty()) continue;
    EXPECT_GE(obs.sensedData[0].range, 0.0f);
    if (std::abs(obs.sensedData[0].range - 0.001f) > 1e-4f) anyDifferent = true;
    EXPECT_LE(std::abs(obs.sensedData[0].yaw), M_PIf);
  }
  EXPECT_TRUE(anyDifferent);

  EXPECT_NEAR(obs.sensor_std_range, 1.0f, 1e-6);
  EXPECT_NEAR(obs.sensor_std_yaw, 0.05f, 1e-6);
  EXPECT_NEAR(obs.sensor_std_pitch, 0.05f, 1e-6);
}

TEST(CLandmarksMap, simulateSpuriousDetections)
{
  mrpt::random::getRandomGenerator().randomize(4321);

  const auto m = makeMap({
      {5.0, 0.0, 0.0}
  });
  auto obs = makeWideOpenObs();
  obs.minSensorDistance = 1.0f;
  obs.maxSensorDistance = 20.0f;
  obs.fieldOfView_yaw = mrpt::DEG2RAD(180.0f);
  obs.fieldOfView_pitch = mrpt::DEG2RAD(90.0f);

  std::vector<size_t> assoc;
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs, true,
      0.01, 0.01, 0.01, &assoc, 4.0 /*spuriousMean*/, 0.5 /*spuriousStd*/);

  ASSERT_EQ(assoc.size(), obs.sensedData.size());
  ASSERT_GT(obs.sensedData.size(), 1U);

  size_t nSpurious = 0;
  for (size_t i = 0; i < assoc.size(); i++)
  {
    if (assoc[i] != std::string::npos) continue;
    nSpurious++;
    EXPECT_EQ(obs.sensedData[i].landmarkID, mrpt::obs::INVALID_LANDMARK_ID);
    EXPECT_GE(obs.sensedData[i].range, obs.minSensorDistance);
    EXPECT_LE(obs.sensedData[i].range, obs.maxSensorDistance);
    EXPECT_LE(std::abs(obs.sensedData[i].yaw), 0.5f * obs.fieldOfView_yaw);
    EXPECT_LE(std::abs(obs.sensedData[i].pitch), 0.5f * obs.fieldOfView_pitch);
  }
  EXPECT_GT(nSpurious, 0U);
}

// With zero angular noise the spurious detections are placed at the sensor
// axis (yaw=pitch=0), the convention for range-only sensors.
TEST(CLandmarksMap, spuriousOfRangeOnlySensorsAreOnAxis)
{
  mrpt::random::getRandomGenerator().randomize(777);

  const CLandmarksMap m;  // no real landmarks at all
  auto obs = makeWideOpenObs();
  obs.minSensorDistance = 1.0f;
  obs.maxSensorDistance = 20.0f;

  std::vector<size_t> assoc;
  m.simulateRangeBearingReadings(
      mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0), obs, true, 0,
      0, 0, &assoc, 3.0, 0.0);

  ASSERT_GT(obs.sensedData.size(), 0U);
  for (size_t i = 0; i < obs.sensedData.size(); i++)
  {
    EXPECT_EQ(assoc[i], std::string::npos);
    EXPECT_EQ(obs.sensedData[i].yaw, 0.0f);
    EXPECT_EQ(obs.sensedData[i].pitch, 0.0f);
  }
}
