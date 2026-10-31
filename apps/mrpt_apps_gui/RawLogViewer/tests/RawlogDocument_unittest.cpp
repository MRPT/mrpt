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

#include "RawlogDocument.h"

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>

#include <chrono>
#include <fstream>
#include <sstream>

using namespace mrpt::obs;

namespace
{
const auto t0 = mrpt::Clock::fromDouble(1000.0);

mrpt::Clock::time_point at(double s)
{
  return t0 + std::chrono::duration_cast<mrpt::Clock::duration>(std::chrono::duration<double>(s));
}

CObservation::Ptr makeIMU(const std::string& label, double t)
{
  auto o = CObservationIMU::Create();
  o->sensorLabel = label;
  o->timestamp = at(t);
  o->set(IMU_WZ, t);
  return o;
}

/** Observations-only: "imu" at 10 Hz, "odo" at 2 Hz, over 2 s. */
RawlogDocument makeObsOnlyDoc()
{
  RawlogDocument d;
  for (int i = 0; i < 20; i++)
  {
    d.rawlog.insert(makeIMU("imu", 0.1 * i));
    if (i % 5 == 0)
    {
      auto odo = CObservationOdometry::Create();
      odo->sensorLabel = "odo";
      odo->timestamp = at(0.1 * i + 0.01);
      odo->odometry = mrpt::poses::CPose2D(0.1 * i, 0, 0);
      d.rawlog.insert(odo);
    }
  }
  d.rebuildIndex();
  return d;
}

/** SF-based, without timestamps: action + SF with a "scan". */
RawlogDocument makeSFDoc(size_t N)
{
  RawlogDocument d;
  for (size_t i = 0; i < N; i++)
  {
    CActionCollection acts;
    CActionRobotMovement2D mov;
    CActionRobotMovement2D::TMotionModelOptions opts;
    mov.computeFromOdometry(mrpt::poses::CPose2D(1.0, 0, 0), opts);
    mov.timestamp = INVALID_TIMESTAMP;
    acts.insert(mov);
    d.rawlog.insert(acts);

    CSensoryFrame sf;
    auto scan = CObservation2DRangeScan::Create();
    scan->timestamp = INVALID_TIMESTAMP;  // like old datasets
    scan->sensorLabel = "scan";
    scan->resizeScan(3);
    sf.insert(scan);
    d.rawlog.insert(sf);
  }
  d.rebuildIndex();
  return d;
}
}  // namespace

TEST(RawlogDocument, IndexWithTimestamps)
{
  const auto d = makeObsOnlyDoc();
  EXPECT_TRUE(d.usesTimestamps());
  ASSERT_EQ(d.sensors().size(), 2u);
  EXPECT_EQ(d.sensors().at("imu").count, 20u);
  EXPECT_EQ(d.sensors().at("odo").count, 4u);
  EXPECT_NEAR(d.sensors().at("imu").rate(), 10.0, 1e-6);
  EXPECT_NEAR(d.entryTime(0), 0.0, 1e-9);

  // Timeline lookups:
  const auto e = d.entryAtTime(0.55);
  ASSERT_TRUE(e.has_value());
  EXPECT_NEAR(d.entryTime(*e), 0.51, 1e-6);  // the odometry at 0.51

  auto imu = std::dynamic_pointer_cast<CObservationIMU>(d.latestObservation("imu", 0.55));
  ASSERT_TRUE(imu);
  EXPECT_NEAR(imu->get(IMU_WZ), 0.5, 1e-9);
  EXPECT_FALSE(d.latestObservation("odo", -1.0));
  EXPECT_FALSE(d.latestObservation("nonexistent", 1.0));
  EXPECT_EQ(d.classCounts().at("mrpt::obs::CObservationIMU"), 20u);
}

TEST(RawlogDocument, IndexWithoutTimestamps)
{
  const auto d = makeSFDoc(5);
  EXPECT_FALSE(d.usesTimestamps());
  EXPECT_EQ(d.entryTime(3), 3.0);
  EXPECT_EQ(d.sensors().at("scan").count, 5u);
  // The scan of the SF at entry 3 is the latest at t=3.5:
  EXPECT_TRUE(d.latestObservation("scan", 3.5));
  EXPECT_NE(d.summaryText().find("No valid timestamps"), std::string::npos);
}

TEST(RawlogDocument, SaveAndLoad)
{
  auto d = makeObsOnlyDoc();
  d.rawlog.setCommentText("hello");
  const auto file = mrpt::system::getTempFileName() + ".rawlog";
  ASSERT_TRUE(d.save(file, {}));
  EXPECT_FALSE(d.modified);

  std::vector<std::string> warnings;
  auto loaded = RawlogDocument::load(file, {}, {}, warnings);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_TRUE(warnings.empty());
  EXPECT_EQ(loaded->rawlog.size(), d.rawlog.size());
  EXPECT_EQ(loaded->rawlog.getCommentText(), "hello");
  EXPECT_EQ(loaded->sensors().size(), 2u);

  // A part only:
  RawlogDocument::LoadOptions opts;
  opts.firstEntry = 2;
  opts.maxEntries = 5;
  auto part = RawlogDocument::load(file, opts, {}, warnings);
  ASSERT_TRUE(part.has_value());
  EXPECT_EQ(part->rawlog.size(), 5u);

  // Cancelled:
  auto cancelled = RawlogDocument::load(
      file, {}, [](double) { return false; }, warnings);
  EXPECT_FALSE(cancelled.has_value());

  // A cancelled save does not touch the existing file:
  const auto sizeBefore = mrpt::system::getFileSize(file);
  d.rawlog.insert(makeIMU("x", 5));
  EXPECT_FALSE(d.save(file, [](double) { return false; }));
  EXPECT_EQ(mrpt::system::getFileSize(file), sizeBefore);
  mrpt::system::deleteFile(file);
}

TEST(RawlogDocument, LoadMissingFile)
{
  std::vector<std::string> warnings;
  EXPECT_THROW(RawlogDocument::load("/nonexistent/file.rawlog", {}, {}, warnings), std::exception);
}

TEST(RawlogDocument, EditOperations)
{
  auto d = makeObsOnlyDoc();
  const size_t N = d.rawlog.size();  // 24

  EXPECT_EQ(d.renameSensor("odo", "odometry"), 4u);
  EXPECT_EQ(d.sensors().count("odometry"), 1u);
  EXPECT_TRUE(d.modified);

  EXPECT_EQ(
      d.setSensorPose("imu", mrpt::poses::CPose3D(1, 2, 3, 0.1, 0, 0), true /*only XYZ*/), 20u);
  auto imu = d.latestObservation("imu", 10.0);
  EXPECT_NEAR(imu->getSensorPose().x(), 1.0, 1e-9);
  EXPECT_NEAR(imu->getSensorPose().yaw(), 0.0, 1e-9);

  EXPECT_EQ(
      d.removeObservations([](const CObservation& o) { return o.sensorLabel == "odometry"; }), 4u);
  EXPECT_EQ(d.rawlog.size(), N - 4);

  EXPECT_EQ(d.keepRange(0, 9, false), N - 4 - 10);
  EXPECT_EQ(d.rawlog.size(), 10u);
  EXPECT_EQ(d.keepRange(0, 4, true), 5u);
  EXPECT_EQ(d.rawlog.size(), 5u);
}

TEST(RawlogDocument, EditSensoryFrames)
{
  auto d = makeSFDoc(4);
  // Observations within SFs are removed, the SFs stay:
  EXPECT_EQ(
      d.removeObservations([](const CObservation& o) { return o.sensorLabel == "scan"; }), 4u);
  EXPECT_EQ(d.rawlog.size(), 8u);
  EXPECT_EQ(d.sensors().size(), 0u);

  auto d2 = makeSFDoc(4);
  CActionRobotMovement2D::TMotionModelOptions opts;
  opts.gaussianModel.minStdXY = 0.5;
  EXPECT_EQ(d2.setMotionModel(opts, 0, 3), 2u);  // entries 0 and 2
  ASSERT_TRUE(d2.firstMotionModel().has_value());
  EXPECT_NEAR(d2.firstMotionModel()->gaussianModel.minStdXY, 0.5, 1e-9);
}

TEST(RawlogDocument, BatchSensorPoses)
{
  auto d = makeSFDoc(2);
  const std::string cfg =
      "[s1]\n"
      "sensorLabel = scan\n"
      "pose_x = 0.5\n"
      "pose_yaw = 90\n";
  EXPECT_EQ(d.applySensorPosesFromConfig(cfg), 2u);
  const auto o = d.latestObservation("scan", 10);
  EXPECT_NEAR(o->getSensorPose().x(), 0.5, 1e-9);
  EXPECT_NEAR(o->getSensorPose().yaw(), M_PI / 2, 1e-9);
  EXPECT_THROW(d.applySensorPosesFromConfig("[a]\nfoo=1\n"), std::exception);
}

TEST(RawlogDocument, ExportToText)
{
  const auto d = makeObsOnlyDoc();
  const auto file = mrpt::system::getTempFileName() + ".txt";
  EXPECT_EQ(d.exportToText("mrpt::obs::CObservationOdometry", "", file), 4u);
  std::ifstream f(file);
  std::stringstream ss;
  ss << f.rdbuf();
  const auto s = ss.str();
  EXPECT_NE(s.find("% TIMESTAMP X Y PHI_RAD"), std::string::npos);
  EXPECT_NE(s.find("1001.510000 1.500000"), std::string::npos);
  EXPECT_EQ(d.exportToText("mrpt::obs::CObservationIMU", "nope", file), 0u);
  mrpt::system::deleteFile(file);
}

TEST(RawlogDocument, BuildRawMap)
{
  auto d = makeSFDoc(5);  // 1 m forward per step, a 3-ray scan each
  // Give the scans valid ranges:
  d.forEachObservation(
      [](size_t, const CObservation::Ptr& o)
      {
        auto scan = std::dynamic_pointer_cast<CObservation2DRangeScan>(o);
        scan->aperture = 0.2f;
        for (size_t k = 0; k < 3; k++)
        {
          scan->setScanRange(k, 2.0f);
          scan->setScanRangeValidity(k, true);
        }
      });
  const std::string cfg =
      "[map]\n"
      "pointsMap_count=1\n"
      "[map_pointsMap_00_insertOpts]\n"
      "minDistBetweenLaserPoints=0\n";
  std::shared_ptr<mrpt::maps::CMultiMetricMap> map;
  std::vector<mrpt::poses::CPose3D> path;
  ASSERT_TRUE(d.buildRawMap(cfg, 0, 100, 1, {}, map, path));
  ASSERT_EQ(path.size(), 5u);
  EXPECT_NEAR(path.back().x(), 5.0, 1e-6);
  auto pts = map->mapByClass<mrpt::maps::CSimplePointsMap>();
  ASSERT_TRUE(pts);
  EXPECT_EQ(pts->size(), 15u);
  // The last scan points are 2 m ahead of the last pose:
  EXPECT_NEAR(pts->getPointsBufferRef_x()[14], 7.0, 0.1);

  // Decimation:
  ASSERT_TRUE(d.buildRawMap(cfg, 0, 100, 2, {}, map, path));
  EXPECT_EQ(map->mapByClass<mrpt::maps::CSimplePointsMap>()->size(), 9u);

  EXPECT_THROW(d.buildRawMap("[map]\n", 0, 100, 1, {}, map, path), std::exception);
}
