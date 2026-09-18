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

/** Unit tests for the ICP-based map builder and the CMetricMapBuilder base
 *  class, driven by 2D laser scans simulated from a synthetic room.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/random.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/system/filesystem.h>

#include <sstream>

#include "slam_synthetic_room.h"

using mrpt::slam::CMetricMapBuilderICP;
using mrpt::test::makeOdometryAction;
using mrpt::test::simulateSF;

namespace
{
void setDefaultMaps(CMetricMapBuilderICP& b, bool withGrid = true)
{
  b.setVerbosityLevel(mrpt::system::LVL_ERROR);
  b.ICP_options.mapInitializers.clear();
  if (withGrid)
  {
    mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
    def.resolution = 0.10f;
    def.insertionOpts.maxDistanceInsertion = 15.0f;
    b.ICP_options.mapInitializers.push_back(def);
  }
  {
    mrpt::maps::CSimplePointsMap::TMapDefinition def;
    def.insertionOpts.minDistBetweenLaserPoints = 0.05f;
    b.ICP_options.mapInitializers.push_back(def);
  }
  b.ICP_options.insertionLinDistance = 0.3;
  b.ICP_options.insertionAngDistance = mrpt::DEG2RAD(20.0);
  b.ICP_options.localizationLinDistance = 0.1;
  b.ICP_options.localizationAngDistance = mrpt::DEG2RAD(5.0);
  b.initialize();
}

/** Drives the builder along a straight path, feeding odometry + scans. */
mrpt::poses::CPose2D runShortSession(CMetricMapBuilderICP& b, size_t nSteps = 5)
{
  mrpt::poses::CPose2D gtPose(-2.0, 0.0, 0.0);

  {
    const auto t = mrpt::test::nextTimestamp();
    auto acts0 = makeOdometryAction(mrpt::poses::CPose2D(0, 0, 0), t);
    auto sf0 = simulateSF(gtPose, t);
    b.processActionObservation(*acts0, *sf0);
  }

  for (size_t i = 0; i < nSteps; i++)
  {
    const mrpt::poses::CPose2D incr(0.4, 0, 0);
    gtPose = gtPose + incr;

    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(incr, t);
    auto sf = simulateSF(gtPose, t);
    b.processActionObservation(*acts, *sf);
  }
  return gtPose;
}
}  // namespace

TEST(CMetricMapBuilderICP, buildsAMapMatchingAgainstPoints)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);
  b.ICP_options.matchAgainstTheGrid = false;

  EXPECT_EQ(b.getCurrentlyBuiltMapSize(), 0U);
  runShortSession(b);

  EXPECT_GT(b.getCurrentlyBuiltMapSize(), 1U);

  const auto pose = b.getCurrentPoseEstimation();
  ASSERT_TRUE(pose);
  // 2.0 m travelled from the starting point:
  EXPECT_NEAR(pose->getMeanVal().x(), 2.0, 0.4);

  mrpt::maps::CSimpleMap sm;
  b.getCurrentlyBuiltMap(sm);
  EXPECT_EQ(sm.size(), b.getCurrentlyBuiltMapSize());

  EXPECT_FALSE(b.getCurrentlyBuiltMetricMap().isEmpty());

  std::vector<float> xs, ys;
  b.getCurrentMapPoints(xs, ys);
  EXPECT_GT(xs.size(), 0U);
  EXPECT_EQ(xs.size(), ys.size());
}

TEST(CMetricMapBuilderICP, buildsAMapMatchingAgainstTheGrid)
{
  mrpt::random::getRandomGenerator().randomize(4321);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);
  b.ICP_options.matchAgainstTheGrid = true;

  runShortSession(b);
  EXPECT_GT(b.getCurrentlyBuiltMapSize(), 1U);
}

TEST(CMetricMapBuilderICP, usesGridAltitudeFilter)
{
  mrpt::random::getRandomGenerator().randomize(555);

  CMetricMapBuilderICP b;
  b.setVerbosityLevel(mrpt::system::LVL_ERROR);
  {
    mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
    def.resolution = 0.10f;
    def.insertionOpts.useMapAltitude = true;
    def.insertionOpts.mapAltitude = 0;
    b.ICP_options.mapInitializers.push_back(def);
  }
  {
    mrpt::maps::CSimplePointsMap::TMapDefinition def;
    b.ICP_options.mapInitializers.push_back(def);
  }
  b.ICP_options.matchAgainstTheGrid = true;
  b.initialize();

  // A scan at the grid altitude is used...
  runShortSession(b, 2);
  const size_t nAtAltitude = b.getCurrentlyBuiltMapSize();
  EXPECT_GT(nAtAltitude, 0U);

  // ...while one at a different altitude cannot feed ICP:
  const auto t = mrpt::test::nextTimestamp();
  auto sf = simulateSF(mrpt::poses::CPose2D(0, 0, 0), t);
  auto scan =
      std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(sf->getObservationByIndex(0));
  ASSERT_TRUE(scan);
  scan->sensorPose = mrpt::poses::CPose3D(0, 0, 5.0, 0, 0, 0);
  auto acts = makeOdometryAction(mrpt::poses::CPose2D(1.0, 0, 0), t);
  EXPECT_NO_THROW(b.processActionObservation(*acts, *sf));
}

TEST(CMetricMapBuilderICP, refusesToRunWithoutMaps)
{
  CMetricMapBuilderICP b;
  b.setVerbosityLevel(mrpt::system::LVL_ERROR);
  b.ICP_options.mapInitializers.clear();
  b.initialize();

  auto sf = simulateSF(mrpt::poses::CPose2D(0, 0, 0), mrpt::test::nextTimestamp());
  EXPECT_THROW(b.processObservation(sf->getObservationByIndex(0)), std::exception);
}

TEST(CMetricMapBuilderICP, mapUpdatingCanBeDisabled)
{
  mrpt::random::getRandomGenerator().randomize(7);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);
  b.enableMapUpdating(false);

  runShortSession(b, 3);
  EXPECT_EQ(b.getCurrentlyBuiltMapSize(), 0U);
  EXPECT_TRUE(b.getCurrentlyBuiltMetricMap().isEmpty());
}

TEST(CMetricMapBuilderICP, alwaysInsertByClass)
{
  mrpt::random::getRandomGenerator().randomize(8);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);
  // Insertion distances so large that nothing would ever be inserted...
  b.ICP_options.insertionLinDistance = 1e6;
  b.ICP_options.insertionAngDistance = 1e6;
  // ...unless the observation class is force-inserted:
  b.options.alwaysInsertByClass.insert(CLASS_ID(mrpt::obs::CObservation2DRangeScan));

  runShortSession(b, 3);
  EXPECT_GT(b.getCurrentlyBuiltMapSize(), 3U);
}

TEST(CMetricMapBuilderICP, observationsWithoutTimestamp)
{
  mrpt::random::getRandomGenerator().randomize(9);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);

  auto sf = simulateSF(mrpt::poses::CPose2D(0, 0, 0), mrpt::test::nextTimestamp());
  auto obs = sf->getObservationByIndex(0);
  obs->timestamp = INVALID_TIMESTAMP;
  EXPECT_NO_THROW(b.processObservation(obs));
  EXPECT_GT(b.getCurrentlyBuiltMapSize(), 0U);
}

TEST(CMetricMapBuilderICP, initializeFromAPreviousMap)
{
  mrpt::random::getRandomGenerator().randomize(10);

  CMetricMapBuilderICP first;
  setDefaultMaps(first);
  runShortSession(first, 3);

  mrpt::maps::CSimpleMap prevMap;
  first.getCurrentlyBuiltMap(prevMap);
  ASSERT_GT(prevMap.size(), 1U);

  CMetricMapBuilderICP second;
  second.setVerbosityLevel(mrpt::system::LVL_ERROR);
  second.ICP_options = first.ICP_options;

  mrpt::poses::CPosePDFGaussian x0;
  x0.mean = mrpt::poses::CPose2D(1.0, 0.0, 0.0);
  second.initialize(prevMap, &x0);

  EXPECT_EQ(second.getCurrentlyBuiltMapSize(), prevMap.size());
  EXPECT_FALSE(second.getCurrentlyBuiltMetricMap().isEmpty());
  EXPECT_NEAR(second.getCurrentPoseEstimation()->getMeanVal().x(), 1.0, 1e-6);

  // clear() resets everything back to an empty map at the origin:
  second.clear();
  EXPECT_EQ(second.getCurrentlyBuiltMapSize(), 0U);
  EXPECT_NEAR(second.getCurrentPoseEstimation()->getMeanVal().x(), 0.0, 1e-9);
}

TEST(CMetricMapBuilderICP, saveAndLoadCurrentMapFile)
{
  mrpt::random::getRandomGenerator().randomize(12);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);
  runShortSession(b, 3);
  const size_t n = b.getCurrentlyBuiltMapSize();
  ASSERT_GT(n, 1U);

  const std::string f = mrpt::system::getTempFileName() + std::string(".simplemap.gz");
  b.saveCurrentMapToFile(f, true /*gz*/);
  EXPECT_TRUE(mrpt::system::fileExists(f));

  CMetricMapBuilderICP b2;
  b2.setVerbosityLevel(mrpt::system::LVL_ERROR);
  b2.ICP_options = b.ICP_options;
  b2.initialize();
  b2.loadCurrentMapFromFile(f);
  EXPECT_EQ(b2.getCurrentlyBuiltMapSize(), n);
  mrpt::system::deleteFile(f);

  // Uncompressed variant:
  const std::string f2 = mrpt::system::getTempFileName() + std::string(".simplemap");
  b.saveCurrentMapToFile(f2, false);
  EXPECT_TRUE(mrpt::system::fileExists(f2));
  mrpt::system::deleteFile(f2);

  // A non-existing file leaves the builder with an empty map:
  CMetricMapBuilderICP b3;
  b3.setVerbosityLevel(mrpt::system::LVL_ERROR);
  b3.ICP_options = b.ICP_options;
  b3.initialize();
  b3.loadCurrentMapFromFile("/tmp/this_file_does_not_exist.simplemap");
  EXPECT_EQ(b3.getCurrentlyBuiltMapSize(), 0U);
}

TEST(CMetricMapBuilderICP, setCurrentMapFile)
{
  mrpt::random::getRandomGenerator().randomize(13);

  const std::string f = mrpt::system::getTempFileName() + std::string(".simplemap.gz");

  {
    CMetricMapBuilderICP b;
    setDefaultMaps(b);
    // Starts a "new" (non-existing) map file...
    b.setCurrentMapFile(f.c_str());
    runShortSession(b, 3);
    // ...which is flushed to disk when detaching from it:
    b.setCurrentMapFile("");
  }
  EXPECT_TRUE(mrpt::system::fileExists(f));
  mrpt::system::deleteFile(f);
}

TEST(CMetricMapBuilderICP, saveCurrentEstimationToImage)
{
  mrpt::random::getRandomGenerator().randomize(14);

  CMetricMapBuilderICP b;
  setDefaultMaps(b);
  runShortSession(b, 3);

  const std::string f = mrpt::system::getTempFileName() + std::string(".png");
  b.saveCurrentEstimationToImage(f);
  EXPECT_TRUE(mrpt::system::fileExists(f));
  mrpt::system::deleteFile(f);

  // Without a gridmap there is nothing to render: a clear error, not a crash.
  CMetricMapBuilderICP bNoGrid;
  setDefaultMaps(bNoGrid, false /*withGrid*/);
  runShortSession(bNoGrid, 2);
  EXPECT_THROW(bNoGrid.saveCurrentEstimationToImage(f), std::exception);
}

TEST(CMetricMapBuilderICP, configParamsLoadDumpAndAssign)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("ICP", "matchAgainstTheGrid", true);
  cfg.write("ICP", "insertionLinDistance", 2.5);
  cfg.write("ICP", "insertionAngDistance", 45.0);
  cfg.write("ICP", "localizationLinDistance", 0.75);
  cfg.write("ICP", "localizationAngDistance", 15.0);
  cfg.write("ICP", "minICPgoodnessToAccept", 0.55);
  cfg.write("ICP", "verbosity_level", "LVL_ERROR");
  cfg.write("ICP", "mrpt::maps::CSimplePointsMap_count", 1);

  CMetricMapBuilderICP b;
  b.ICP_options.loadFromConfigFile(cfg, "ICP");

  EXPECT_TRUE(b.ICP_options.matchAgainstTheGrid);
  EXPECT_NEAR(b.ICP_options.insertionLinDistance, 2.5, 1e-9);
  EXPECT_NEAR(b.ICP_options.insertionAngDistance, mrpt::DEG2RAD(45.0), 1e-9);
  EXPECT_NEAR(b.ICP_options.localizationLinDistance, 0.75, 1e-9);
  EXPECT_NEAR(b.ICP_options.localizationAngDistance, mrpt::DEG2RAD(15.0), 1e-9);
  EXPECT_NEAR(b.ICP_options.minICPgoodnessToAccept, 0.55, 1e-9);
  EXPECT_EQ(b.ICP_options.mapInitializers.size(), 1U);

  std::stringstream ss;
  b.ICP_options.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("insertionLinDistance"), std::string::npos);

  // operator=() copies everything but the verbosity reference:
  CMetricMapBuilderICP b2;
  b2.ICP_options = b.ICP_options;
  EXPECT_NEAR(b2.ICP_options.insertionLinDistance, 2.5, 1e-9);
  EXPECT_EQ(b2.ICP_options.mapInitializers.size(), 1U);
}
