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

/** Unit tests for the RBPF map builder and its underlying
 *  mrpt::maps::CMultiMetricMapPDF, driven by 2D laser scans simulated from a
 *  synthetic room (no dataset files needed).
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/archiveFrom_std_streams.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/system/filesystem.h>

#include <sstream>

#include "slam_synthetic_room.h"

using mrpt::maps::CMultiMetricMapPDF;
using mrpt::slam::CMetricMapBuilderRBPF;

namespace
{
using mrpt::test::makeOdometryAction;
using mrpt::test::simulateSF;

CMetricMapBuilderRBPF::TConstructionOptions defaultRBPFOptions(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm algo =
        mrpt::bayes::CParticleFilter::pfStandardProposal)
{
  CMetricMapBuilderRBPF::TConstructionOptions o;
  o.verbosity_level = mrpt::system::LVL_ERROR;
  o.insertionLinDistance = 0.5;
  o.insertionAngDistance = mrpt::DEG2RAD(20.0);
  o.localizeLinDistance = 0.1;
  o.localizeAngDistance = mrpt::DEG2RAD(5.0);

  o.PF_options.PF_algorithm = algo;
  o.PF_options.adaptiveSampleSize = false;
  o.PF_options.sampleSize = 5;
  o.PF_options.resamplingMethod = mrpt::bayes::CParticleFilter::prSystematic;

  {
    mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
    def.resolution = 0.10f;
    def.insertionOpts.maxDistanceInsertion = 15.0f;
    o.mapsInitializers.push_back(def);
  }
  {
    mrpt::maps::CSimplePointsMap::TMapDefinition def;
    o.mapsInitializers.push_back(def);
  }
  return o;
}

/** Drives the map builder along a straight path of `nSteps` steps of 0.4 m. */
void runShortSession(CMetricMapBuilderRBPF& b, size_t nSteps = 5)
{
  mrpt::poses::CPose2D gtPose(-2.0, 0.0, 0.0);

  // First observation, with no movement:
  {
    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(mrpt::poses::CPose2D(0, 0, 0), t);
    auto sf = simulateSF(gtPose, t);
    b.processActionObservation(*acts, *sf);
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
}
}  // namespace

TEST(CMetricMapBuilderRBPF, buildsAMapWithStandardProposal)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  CMetricMapBuilderRBPF builder(defaultRBPFOptions());
  EXPECT_EQ(builder.getCurrentlyBuiltMapSize(), 0U);

  runShortSession(builder);

  EXPECT_GT(builder.getCurrentlyBuiltMapSize(), 1U);

  // The gridmap of the most likely particle must contain the room walls:
  const auto& mmap = builder.getCurrentlyBuiltMetricMap();
  auto grid = mmap.mapByClass<mrpt::maps::COccupancyGridMap2D>();
  ASSERT_TRUE(grid);
  EXPECT_FALSE(grid->isEmpty());

  mrpt::maps::CSimpleMap sm;
  builder.getCurrentlyBuiltMap(sm);
  EXPECT_EQ(sm.size(), builder.getCurrentlyBuiltMapSize());

  // The estimated displacement must be close to the true 2.0 m travelled:
  const auto poseEst = builder.getCurrentPoseEstimation();
  ASSERT_TRUE(poseEst);
  EXPECT_NEAR(poseEst->getMeanVal().x(), 2.0, 0.5);

  std::deque<mrpt::math::TPose3D> path;
  builder.getCurrentMostLikelyPath(path);
  EXPECT_GT(path.size(), 1U);

  EXPECT_GT(builder.mapPDF.getNumberOfObservationsInSimplemap(), 0U);
}

TEST(CMetricMapBuilderRBPF, alternativePFAlgorithms)
{
  for (const auto algo :
       {mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard,
        mrpt::bayes::CParticleFilter::pfOptimalProposal,
        mrpt::bayes::CParticleFilter::pfAuxiliaryPFOptimal})
  {
    mrpt::random::getRandomGenerator().randomize(1000 + static_cast<int>(algo));

    auto opts = defaultRBPFOptions(algo);
    // Use the gridmap-based approximation to the optimal proposal:
    opts.predictionOptions.pfOptimalProposal_mapSelection = 0;

    CMetricMapBuilderRBPF builder(opts);
    runShortSession(builder, 3);

    EXPECT_GT(builder.getCurrentlyBuiltMapSize(), 1U) << "PF algorithm #" << static_cast<int>(algo);
  }
}

TEST(CMetricMapBuilderRBPF, optimalProposalWithPointsMap)
{
  mrpt::random::getRandomGenerator().randomize(99);

  auto opts = defaultRBPFOptions(mrpt::bayes::CParticleFilter::pfOptimalProposal);
  opts.predictionOptions.pfOptimalProposal_mapSelection = 3;  // points map

  CMetricMapBuilderRBPF builder(opts);
  runShortSession(builder, 3);

  EXPECT_GT(builder.getCurrentlyBuiltMapSize(), 1U);
}

TEST(CMetricMapBuilderRBPF, adaptiveSampleSize)
{
  mrpt::random::getRandomGenerator().randomize(77);

  auto opts = defaultRBPFOptions();
  opts.PF_options.adaptiveSampleSize = true;
  // A dynamic number of particles requires multinomial resampling:
  opts.PF_options.resamplingMethod = mrpt::bayes::CParticleFilter::prMultinomial;
  opts.predictionOptions.KLD_params.KLD_minSampleSize = 3;
  opts.predictionOptions.KLD_params.KLD_maxSampleSize = 20;

  CMetricMapBuilderRBPF builder(opts);
  runShortSession(builder, 3);

  EXPECT_GE(builder.mapPDF.particlesCount(), 3U);
}

TEST(CMetricMapBuilderRBPF, constructionOptionsLoadFromConfigFile)
{
  mrpt::config::CConfigFileMemory cfg;
  // PF options (several of them have no default and must be present):
  cfg.write("RBPF", "adaptiveSampleSize", false);
  cfg.write("RBPF", "BETA", 0.5);
  cfg.write("RBPF", "sampleSize", 7);
  cfg.write("RBPF", "PF_algorithm", "pfStandardProposal");
  cfg.write("RBPF", "resamplingMethod", "prSystematic");
  // ...and this class' own options:
  cfg.write("RBPF", "insertionLinDistance", 1.25);
  cfg.write("RBPF", "insertionAngDistance_deg", 22.0);
  cfg.write("RBPF", "localizeLinDistance", 0.55);
  cfg.write("RBPF", "localizeAngDistance_deg", 11.0);
  cfg.write("RBPF", "verbosity_level", "LVL_ERROR");
  cfg.write("RBPF", "mrpt::maps::CSimplePointsMap_count", 1);
  cfg.write("RBPF", "pfOptimalProposal_mapSelection", 0);

  CMetricMapBuilderRBPF::TConstructionOptions o2;
  o2.loadFromConfigFile(cfg, "RBPF");

  EXPECT_EQ(o2.PF_options.sampleSize, 7U);
  EXPECT_NEAR(o2.insertionLinDistance, 1.25, 1e-9);
  EXPECT_NEAR(o2.insertionAngDistance, mrpt::DEG2RAD(22.0), 1e-9);
  EXPECT_NEAR(o2.localizeLinDistance, 0.55, 1e-9);
  EXPECT_NEAR(o2.localizeAngDistance, mrpt::DEG2RAD(11.0), 1e-9);
  EXPECT_EQ(o2.verbosity_level, mrpt::system::LVL_ERROR);
  EXPECT_EQ(o2.mapsInitializers.size(), 1U);

  // A missing mandatory entry is an error:
  mrpt::config::CConfigFileMemory empty;
  CMetricMapBuilderRBPF::TConstructionOptions o3;
  EXPECT_THROW(o3.loadFromConfigFile(empty, "RBPF"), std::exception);

  std::stringstream ss;
  defaultRBPFOptions().dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CMetricMapBuilderRBPF, copyAssignmentAndDefaultCtor)
{
  mrpt::random::getRandomGenerator().randomize(5);

  CMetricMapBuilderRBPF src(defaultRBPFOptions());
  runShortSession(src, 2);

  CMetricMapBuilderRBPF dst;  // "empty" constructor
  dst = src;
  EXPECT_EQ(dst.getCurrentlyBuiltMapSize(), src.getCurrentlyBuiltMapSize());

  // Self-assignment must be a no-op:
  CMetricMapBuilderRBPF* pSrc = &src;
  src = *pSrc;
  EXPECT_EQ(dst.getCurrentlyBuiltMapSize(), src.getCurrentlyBuiltMapSize());
}

TEST(CMetricMapBuilderRBPF, initializeFromPreviousMap)
{
  mrpt::random::getRandomGenerator().randomize(11);

  CMetricMapBuilderRBPF first(defaultRBPFOptions());
  runShortSession(first, 3);

  mrpt::maps::CSimpleMap prevMap;
  first.getCurrentlyBuiltMap(prevMap);
  ASSERT_GT(prevMap.size(), 1U);

  CMetricMapBuilderRBPF second(defaultRBPFOptions());
  second.initialize(prevMap);
  EXPECT_EQ(second.getCurrentlyBuiltMapSize(), prevMap.size());

  // Each particle path holds one node per previous keyframe:
  std::deque<mrpt::math::TPose3D> path;
  second.mapPDF.getPath(0, path);
  EXPECT_EQ(path.size(), prevMap.size());

  // Also with an explicit initial pose:
  mrpt::poses::CPosePDFGaussian x0;
  x0.mean = mrpt::poses::CPose2D(1.0, 2.0, 0.0);
  CMetricMapBuilderRBPF third(defaultRBPFOptions());
  third.initialize(mrpt::maps::CSimpleMap(), &x0);
  bool valid = false;
  const auto p0 = third.mapPDF.getLastPose(0, valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(p0.x, 1.0, 1e-9);
  EXPECT_NEAR(p0.y, 2.0, 1e-9);
}

TEST(CMetricMapBuilderRBPF, debugOutputs)
{
  mrpt::random::getRandomGenerator().randomize(21);

  CMetricMapBuilderRBPF builder(defaultRBPFOptions());
  runShortSession(builder, 2);

  const std::string pathFile = mrpt::system::getTempFileName();
  builder.saveCurrentPathEstimationToTextFile(pathFile);
  EXPECT_TRUE(mrpt::system::fileExists(pathFile));
  EXPECT_GT(mrpt::system::getFileSize(pathFile), 0U);
  mrpt::system::deleteFile(pathFile);

  const std::string imgFile = mrpt::system::getTempFileName() + std::string(".png");
  builder.saveCurrentEstimationToImage(imgFile);
  EXPECT_TRUE(mrpt::system::fileExists(imgFile));
  mrpt::system::deleteFile(imgFile);

  mrpt::img::CImage img(200, 200);
  EXPECT_NO_THROW(builder.drawCurrentEstimationToImage(&img));

  EXPECT_TRUE(std::isfinite(builder.getCurrentJointEntropy()));
}

TEST(CMultiMetricMapPDF, clearVariants)
{
  auto opts = defaultRBPFOptions();
  CMultiMetricMapPDF pdf(opts.PF_options, opts.mapsInitializers, opts.predictionOptions);

  ASSERT_EQ(pdf.particlesCount(), 5U);

  pdf.clear(mrpt::poses::CPose2D(1.0, 2.0, 0.5));
  bool valid = false;
  auto p = pdf.getLastPose(0, valid);
  EXPECT_TRUE(valid);
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.yaw, 0.5, 1e-9);

  pdf.clear(mrpt::poses::CPose3D(3.0, 0, 0, 0, 0, 0));
  p = pdf.getLastPose(0, valid);
  EXPECT_NEAR(p.x, 3.0, 1e-9);
  EXPECT_EQ(pdf.getNumberOfObservationsInSimplemap(), 0U);

  // clear() with an empty previous map falls back to the pose-only version:
  pdf.clear(mrpt::maps::CSimpleMap(), mrpt::poses::CPose3D(4.0, 0, 0, 0, 0, 0));
  p = pdf.getLastPose(0, valid);
  EXPECT_NEAR(p.x, 4.0, 1e-9);

  EXPECT_THROW(std::ignore = pdf.getLastPose(99, valid), std::exception);
  std::deque<mrpt::math::TPose3D> path;
  EXPECT_THROW(pdf.getPath(99, path), std::exception);
}

TEST(CMultiMetricMapPDF, pathsEntropyAndAveragedMap)
{
  mrpt::random::getRandomGenerator().randomize(31);

  CMetricMapBuilderRBPF builder(defaultRBPFOptions());
  runShortSession(builder, 3);

  auto& pdf = builder.mapPDF;

  EXPECT_GT(pdf.getCurrentEntropyOfPaths(), -1e10);

  // The averaged map is cached until the next insertion:
  const auto* avg1 = pdf.getAveragedMetricMapEstimation();
  ASSERT_TRUE(avg1);
  const auto* avg2 = pdf.getAveragedMetricMapEstimation();
  EXPECT_EQ(avg1, avg2);
  EXPECT_TRUE(avg1->mapByClass<mrpt::maps::COccupancyGridMap2D>());

  const auto* best = pdf.getCurrentMostLikelyMetricMap();
  ASSERT_TRUE(best);
  EXPECT_TRUE(best->mapByClass<mrpt::maps::COccupancyGridMap2D>());

  // Updating the SF poses from the current paths must not change their count:
  const size_t n = pdf.getNumberOfObservationsInSimplemap();
  pdf.updateSensoryFrameSequence();
  EXPECT_EQ(pdf.getNumberOfObservationsInSimplemap(), n);

  mrpt::poses::CPose3DPDFParticles est;
  pdf.getEstimatedPosePDFAtTime(0, est);
  EXPECT_EQ(est.size(), pdf.particlesCount());
}

TEST(CMultiMetricMapPDF, serializationRoundTrip)
{
  mrpt::random::getRandomGenerator().randomize(41);

  CMetricMapBuilderRBPF builder(defaultRBPFOptions());
  runShortSession(builder, 2);

  std::stringstream ss;
  auto arch = mrpt::serialization::archiveFrom<std::iostream>(ss);
  arch << builder.mapPDF;

  CMultiMetricMapPDF pdf2;
  arch >> pdf2;

  ASSERT_EQ(pdf2.particlesCount(), builder.mapPDF.particlesCount());
  EXPECT_EQ(
      pdf2.getNumberOfObservationsInSimplemap(),
      builder.mapPDF.getNumberOfObservationsInSimplemap());

  std::deque<mrpt::math::TPose3D> p1, p2;
  builder.mapPDF.getPath(0, p1);
  pdf2.getPath(0, p2);
  ASSERT_EQ(p1.size(), p2.size());
  for (size_t i = 0; i < p1.size(); i++) EXPECT_NEAR(p1[i].x, p2[i].x, 1e-9);
}

TEST(CMultiMetricMapPDF, predictionParamsSaveLoad)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("PDF", "pfOptimalProposal_mapSelection", 3);
  cfg.write("PDF", "ICPGlobalAlign_MinQuality", 0.5);
  cfg.write("PDF", "KLD_minSampleSize", 30);

  CMultiMetricMapPDF::TPredictionParams p;
  p.loadFromConfigFile(cfg, "PDF");

  EXPECT_EQ(p.pfOptimalProposal_mapSelection, 3);
  EXPECT_NEAR(p.ICPGlobalAlign_MinQuality, 0.5f, 1e-6);
  EXPECT_EQ(p.KLD_params.KLD_minSampleSize, 30U);

  std::stringstream ss;
  p.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("pfOptimalProposal_mapSelection"), std::string::npos);
}

namespace
{
/** Three beacons around the robot path, for the range-only SLAM tests. */
const std::vector<mrpt::math::TPoint3D>& beaconPositions()
{
  static const std::vector<mrpt::math::TPoint3D> pts = {
      { 0.0,  3.0, 0.0},
      { 4.0, -2.0, 0.0},
      {-3.0, -2.0, 0.0}
  };
  return pts;
}

mrpt::obs::CSensoryFrame::Ptr simulateBeaconRanges(
    const mrpt::poses::CPose2D& robotPose, const mrpt::system::TTimeStamp t)
{
  auto obs = mrpt::obs::CObservationBeaconRanges::Create();
  obs->timestamp = t;
  obs->stdError = 0.05f;

  int64_t id = 0;
  for (const auto& p : beaconPositions())
  {
    mrpt::obs::CObservationBeaconRanges::TMeasurement m;
    m.beaconID = id++;
    m.sensorLocationOnRobot = mrpt::poses::CPoint3D(0, 0, 0);
    m.sensedDistance = static_cast<float>(mrpt::poses::CPoint3D(p).distanceTo(
        mrpt::poses::CPoint3D(robotPose.x(), robotPose.y(), 0)));
    obs->sensedData.push_back(m);
  }

  auto sf = mrpt::obs::CSensoryFrame::Create();
  sf->insert(obs);
  return sf;
}

CMetricMapBuilderRBPF::TConstructionOptions rangeOnlySLAMOptions(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm algo)
{
  CMetricMapBuilderRBPF::TConstructionOptions o;
  o.verbosity_level = mrpt::system::LVL_ERROR;
  o.insertionLinDistance = 0;  // insert every step
  o.insertionAngDistance = 0;
  o.localizeLinDistance = 0;
  o.localizeAngDistance = 0;

  o.PF_options.PF_algorithm = algo;
  o.PF_options.adaptiveSampleSize = false;
  o.PF_options.sampleSize = 5;
  o.PF_options.resamplingMethod = mrpt::bayes::CParticleFilter::prSystematic;

  mrpt::maps::CBeaconMap::TMapDefinition def;
  // The optimal-proposal path only handles Gaussian/SOG beacon PDFs:
  def.insertionOpts.insertAsMonteCarlo = false;
  o.mapsInitializers.push_back(def);

  o.predictionOptions.pfOptimalProposal_mapSelection = 2;  // beacon map
  return o;
}

/** Drives a range-only SLAM session along a short straight path. */
void runRangeOnlySession(CMetricMapBuilderRBPF& b, size_t nSteps = 4)
{
  mrpt::poses::CPose2D gtPose(0, 0, 0);
  {
    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(mrpt::poses::CPose2D(0, 0, 0), t);
    auto sf = simulateBeaconRanges(gtPose, t);
    b.processActionObservation(*acts, *sf);
  }
  for (size_t i = 0; i < nSteps; i++)
  {
    const mrpt::poses::CPose2D incr(0.3, 0, 0);
    gtPose = gtPose + incr;
    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(incr, t);
    auto sf = simulateBeaconRanges(gtPose, t);
    b.processActionObservation(*acts, *sf);
  }
}
}  // namespace

TEST(CMetricMapBuilderRBPF, rangeOnlySLAMWithOptimalProposal)
{
  mrpt::random::getRandomGenerator().randomize(2026);

  CMetricMapBuilderRBPF builder(
      rangeOnlySLAMOptions(mrpt::bayes::CParticleFilter::pfOptimalProposal));
  runRangeOnlySession(builder);

  const auto& mmap = builder.getCurrentlyBuiltMetricMap();
  auto beacons = mmap.mapByClass<mrpt::maps::CBeaconMap>();
  ASSERT_TRUE(beacons);
  EXPECT_EQ(beacons->size(), beaconPositions().size());
  EXPECT_GT(builder.getCurrentlyBuiltMapSize(), 1U);
}

TEST(CMetricMapBuilderRBPF, rangeOnlySLAMWithAuxiliaryOptimal)
{
  mrpt::random::getRandomGenerator().randomize(2027);

  CMetricMapBuilderRBPF builder(
      rangeOnlySLAMOptions(mrpt::bayes::CParticleFilter::pfAuxiliaryPFOptimal));
  runRangeOnlySession(builder, 3);

  auto beacons = builder.getCurrentlyBuiltMetricMap().mapByClass<mrpt::maps::CBeaconMap>();
  ASSERT_TRUE(beacons);
  EXPECT_EQ(beacons->size(), beaconPositions().size());
}
