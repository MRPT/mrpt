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

/** Unit tests for the Monte Carlo localization PDFs (SE(2) and SE(3)) over a
 *  synthetic room, exercising the four particle filter algorithms without
 *  needing any dataset file.
 *  \sa CMonteCarloLocalization2D_unittest.cpp for the dataset-based test.
 */

#include <gtest/gtest.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/random.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>

#include "slam_synthetic_room.h"

using mrpt::slam::CMonteCarloLocalization2D;
using mrpt::slam::CMonteCarloLocalization3D;
using mrpt::test::makeOdometryAction;
using mrpt::test::simulateSF;

namespace
{
mrpt::maps::COccupancyGridMap2D::Ptr referenceMap()
{
  auto m = mrpt::maps::COccupancyGridMap2D::Create();
  *m = mrpt::test::groundTruthRoom();
  return m;
}

mrpt::bayes::CParticleFilter::TParticleFilterOptions pfOptions(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm algo, bool adaptive = false)
{
  mrpt::bayes::CParticleFilter::TParticleFilterOptions o;
  o.PF_algorithm = algo;
  o.adaptiveSampleSize = adaptive;
  o.sampleSize = 50;
  o.resamplingMethod = adaptive ? mrpt::bayes::CParticleFilter::prMultinomial
                                : mrpt::bayes::CParticleFilter::prSystematic;
  o.BETA = 0.5;
  return o;
}

/** Runs a short localization session from a known starting area, returning the
 *  final mean estimate. */
template <class PDF>
mrpt::poses::CPose3D runLocalization(
    PDF& pdf, const mrpt::bayes::CParticleFilter::TParticleFilterOptions& opts, size_t nSteps = 4)
{
  mrpt::bayes::CParticleFilter pf;
  pf.m_options = opts;

  mrpt::poses::CPose2D gtPose(-2.0, 0.0, 0.0);
  {
    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(mrpt::poses::CPose2D(0, 0, 0), t);
    auto sf = simulateSF(gtPose, t);
    pf.executeOn(pdf, acts.get(), sf.get());
  }
  for (size_t i = 0; i < nSteps; i++)
  {
    const mrpt::poses::CPose2D incr(0.3, 0, 0);
    gtPose = gtPose + incr;
    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(incr, t);
    auto sf = simulateSF(gtPose, t);
    pf.executeOn(pdf, acts.get(), sf.get());
  }
  return mrpt::poses::CPose3D(gtPose);
}
}  // namespace

TEST(CMonteCarloLocalization2DSynthetic, resetUniformFreeSpace)
{
  mrpt::random::getRandomGenerator().randomize(1);

  auto map = referenceMap();

  CMonteCarloLocalization2D pdf(1);
  pdf.resetUniformFreeSpace(map.get(), 0.7, 40, -1.0, 1.0, -1.0, 1.0, -M_PI, M_PI);

  EXPECT_EQ(pdf.particlesCount(), 40U);
  for (size_t i = 0; i < pdf.particlesCount(); i++)
  {
    const auto p = pdf.getParticlePose(i);
    EXPECT_LT(std::abs(p.x), 1.5);
    EXPECT_LT(std::abs(p.y), 1.5);
  }

  // Without area limits, the whole free space of the map is used:
  pdf.resetUniformFreeSpace(map.get(), 0.7, 20);
  EXPECT_EQ(pdf.particlesCount(), 20U);

  // A fully occupied map has no free cell to place particles at:
  auto occupied = mrpt::maps::COccupancyGridMap2D::Create();
  *occupied = mrpt::test::groundTruthRoom();
  occupied->fill(0.0f);
  EXPECT_THROW(pdf.resetUniformFreeSpace(occupied.get()), std::exception);
}

TEST(CMonteCarloLocalization2DSynthetic, convergesWithAllPFAlgorithms)
{
  const std::vector<mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm> algos = {
      mrpt::bayes::CParticleFilter::pfStandardProposal,
      mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard,
      mrpt::bayes::CParticleFilter::pfAuxiliaryPFOptimal};

  auto map = referenceMap();

  for (const auto algo : algos)
  {
    mrpt::random::getRandomGenerator().randomize(100 + static_cast<int>(algo));

    CMonteCarloLocalization2D pdf(50);
    pdf.options.metricMap = map;
    pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

    const auto gt = runLocalization(pdf, pfOptions(algo));

    const auto est = pdf.getMeanVal();
    EXPECT_NEAR(est.x(), gt.x(), 0.6) << "PF algorithm #" << static_cast<int>(algo);
    EXPECT_NEAR(est.y(), gt.y(), 0.6) << "PF algorithm #" << static_cast<int>(algo);
  }
}

// The exact optimal proposal is not implemented for plain localization:
TEST(CMonteCarloLocalization2DSynthetic, optimalProposalIsNotImplemented)
{
  mrpt::random::getRandomGenerator().randomize(2);

  auto map = referenceMap();

  CMonteCarloLocalization2D pdf(10);
  pdf.options.metricMap = map;
  pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

  EXPECT_THROW(
      runLocalization(pdf, pfOptions(mrpt::bayes::CParticleFilter::pfOptimalProposal), 1),
      std::exception);
}

TEST(CMonteCarloLocalization2DSynthetic, adaptiveSampleSizeKLD)
{
  mrpt::random::getRandomGenerator().randomize(3);

  auto map = referenceMap();

  CMonteCarloLocalization2D pdf(50);
  pdf.options.metricMap = map;
  pdf.options.KLD_params.KLD_minSampleSize = 20;
  pdf.options.KLD_params.KLD_maxSampleSize = 200;
  pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

  runLocalization(pdf, pfOptions(mrpt::bayes::CParticleFilter::pfStandardProposal, true), 3);

  EXPECT_GE(pdf.particlesCount(), 20U);
  EXPECT_LE(pdf.particlesCount(), 200U);
}

TEST(CMonteCarloLocalization2DSynthetic, oneMapPerParticle)
{
  mrpt::random::getRandomGenerator().randomize(4);

  auto map = referenceMap();

  CMonteCarloLocalization2D pdf(10);
  pdf.options.metricMaps.assign(10, map);
  pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

  const auto gt =
      runLocalization(pdf, pfOptions(mrpt::bayes::CParticleFilter::pfStandardProposal), 2);
  EXPECT_NEAR(pdf.getMeanVal().x(), gt.x(), 1.0);
}

TEST(CMonteCarloLocalization3DSynthetic, convergesWithStandardProposal)
{
  mrpt::random::getRandomGenerator().randomize(5);

  auto map = referenceMap();

  CMonteCarloLocalization3D pdf(50);
  pdf.options.metricMap = map;
  pdf.resetUniform(
      mrpt::math::TPose3D(-2.5, -0.5, 0, -mrpt::DEG2RAD(20.0), 0, 0),
      mrpt::math::TPose3D(-1.5, 0.5, 0, mrpt::DEG2RAD(20.0), 0, 0));

  const auto gt = runLocalization(pdf, pfOptions(mrpt::bayes::CParticleFilter::pfStandardProposal));

  const auto est = pdf.getMeanVal();
  EXPECT_NEAR(est.x(), gt.x(), 0.7);
  EXPECT_NEAR(est.y(), gt.y(), 0.7);

  bool valid = false;
  const auto p0 = pdf.getLastPose(0, valid);
  EXPECT_TRUE(valid);
  EXPECT_TRUE(std::isfinite(p0.x));
  EXPECT_THROW(std::ignore = pdf.getLastPose(pdf.particlesCount(), valid), std::exception);
}

TEST(CMonteCarloLocalization3DSynthetic, auxiliaryPFStandard)
{
  mrpt::random::getRandomGenerator().randomize(6);

  auto map = referenceMap();

  CMonteCarloLocalization3D pdf(40);
  pdf.options.metricMap = map;
  pdf.resetUniform(
      mrpt::math::TPose3D(-2.5, -0.5, 0, -mrpt::DEG2RAD(20.0), 0, 0),
      mrpt::math::TPose3D(-1.5, 0.5, 0, mrpt::DEG2RAD(20.0), 0, 0));

  const auto gt =
      runLocalization(pdf, pfOptions(mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard), 3);
  EXPECT_NEAR(pdf.getMeanVal().x(), gt.x(), 1.0);
}

TEST(CMonteCarloLocalization2DSynthetic, adaptiveSampleSizeWithAuxiliaryPFs)
{
  // The KLD adaptive-sampling code path of the auxiliary particle filters is
  // a separate implementation from the fixed-sample-size one:
  const std::vector<mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm> algos = {
      mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard,
      mrpt::bayes::CParticleFilter::pfAuxiliaryPFOptimal};

  auto map = referenceMap();

  for (const auto algo : algos)
  {
    mrpt::random::getRandomGenerator().randomize(200 + static_cast<int>(algo));

    CMonteCarloLocalization2D pdf(50);
    pdf.options.metricMap = map;
    pdf.options.KLD_params.KLD_minSampleSize = 20;
    pdf.options.KLD_params.KLD_maxSampleSize = 200;
    pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

    runLocalization(pdf, pfOptions(algo, true), 3);

    EXPECT_GE(pdf.particlesCount(), 20U) << "PF algorithm #" << static_cast<int>(algo);
    EXPECT_LE(pdf.particlesCount(), 200U) << "PF algorithm #" << static_cast<int>(algo);
  }
}

TEST(CMonteCarloLocalization2DSynthetic, accumulates3DOdometryActions)
{
  // Both the standard proposal (which reads the action directly) and the
  // auxiliary variants (which accumulate actions across steps) accept a
  // CActionRobotMovement3D instead of the usual 2D one.
  for (const auto algo :
       {mrpt::bayes::CParticleFilter::pfStandardProposal,
        mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard})
  {
    mrpt::random::getRandomGenerator().randomize(6);

    auto map = referenceMap();

    CMonteCarloLocalization2D pdf(20);
    pdf.options.metricMap = map;
    pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

    mrpt::bayes::CParticleFilter pf;
    pf.m_options = pfOptions(algo);

    mrpt::poses::CPose2D gtPose(-2.0, 0.0, 0.0);
    for (int i = 0; i < 3; i++)
    {
      const mrpt::poses::CPose2D incr(0.3, 0, 0);
      gtPose = gtPose + incr;
      const auto t = mrpt::test::nextTimestamp();

      // A 3D action instead of the usual 2D one:
      auto acts = mrpt::obs::CActionCollection::Create();
      mrpt::obs::CActionRobotMovement3D act3D;
      act3D.timestamp = t;
      act3D.poseChange.mean = mrpt::poses::CPose3D(incr);
      act3D.poseChange.cov.setDiagonal(1e-4);
      acts->insert(act3D);

      auto sf = simulateSF(gtPose, t);
      pf.executeOn(pdf, acts.get(), sf.get());
    }

    EXPECT_EQ(pdf.particlesCount(), 20U);
    EXPECT_TRUE(std::isfinite(pdf.getMeanVal().x()));
  }
}

TEST(CMonteCarloLocalization2DSynthetic, mixing2DAnd3DActionsThrows)
{
  mrpt::random::getRandomGenerator().randomize(7);

  auto map = referenceMap();

  CMonteCarloLocalization2D pdf(10);
  pdf.options.metricMap = map;
  pdf.resetUniform(-2.5, -1.5, -0.5, 0.5, -mrpt::DEG2RAD(20.0), mrpt::DEG2RAD(20.0));

  // Only the auxiliary variants accumulate actions across steps, and hence
  // are the ones that can end up mixing a 2D and a 3D one:
  mrpt::bayes::CParticleFilter pf;
  pf.m_options = pfOptions(mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard);

  const mrpt::poses::CPose2D gtPose(-2.0, 0.0, 0.0);

  // A first step with a 3D action and no observations, so the movement is
  // only accumulated and not consumed:
  {
    auto acts = mrpt::obs::CActionCollection::Create();
    mrpt::obs::CActionRobotMovement3D act3D;
    act3D.timestamp = mrpt::test::nextTimestamp();
    act3D.poseChange.mean = mrpt::poses::CPose3D(mrpt::poses::CPose2D(0.3, 0, 0));
    act3D.poseChange.cov.setDiagonal(1e-4);
    acts->insert(act3D);

    // With no observations at all the movement is only accumulated:
    pf.executeOn(pdf, acts.get(), nullptr);
  }

  // ...and now a 2D one, which cannot be mixed with the accumulated 3D:
  {
    const auto t = mrpt::test::nextTimestamp();
    auto acts = makeOdometryAction(mrpt::poses::CPose2D(0.3, 0, 0), t);
    auto sf = simulateSF(gtPose, t);
    EXPECT_THROW(pf.executeOn(pdf, acts.get(), sf.get()), std::exception);
  }
}

TEST(CMonteCarloLocalization3DSynthetic, auxiliaryPFOptimalAndAdaptiveSampling)
{
  auto map = referenceMap();

  for (const auto algo :
       {mrpt::bayes::CParticleFilter::pfAuxiliaryPFOptimal,
        mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard})
  {
    mrpt::random::getRandomGenerator().randomize(300 + static_cast<int>(algo));

    CMonteCarloLocalization3D pdf(40);
    pdf.options.metricMap = map;
    // The 3D KLD binning uses its own TPoseBin3D specialization:
    pdf.options.KLD_params.KLD_minSampleSize = 20;
    pdf.options.KLD_params.KLD_maxSampleSize = 150;
    pdf.resetUniform(
        mrpt::math::TPose3D(-2.5, -0.5, 0, -mrpt::DEG2RAD(20.0), 0, 0),
        mrpt::math::TPose3D(-1.5, 0.5, 0, mrpt::DEG2RAD(20.0), 0, 0));

    runLocalization(pdf, pfOptions(algo, true), 3);

    EXPECT_GE(pdf.particlesCount(), 20U) << "PF algorithm #" << static_cast<int>(algo);
    EXPECT_LE(pdf.particlesCount(), 150U) << "PF algorithm #" << static_cast<int>(algo);
    EXPECT_TRUE(std::isfinite(pdf.getMeanVal().x()));
  }
}
