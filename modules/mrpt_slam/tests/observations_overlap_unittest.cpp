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

/** Unit tests for mrpt::slam::observationsOverlap(): the per-observation
 *  overlap ratio, its sensory-frame average, and the effect of the optional
 *  relative pose between both observations.
 */

#include <gtest/gtest.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/observations_overlap.h>

using mrpt::slam::observationsOverlap;

namespace
{
mrpt::obs::CObservation2DRangeScan::Ptr makeScan(int index)
{
  auto scan = mrpt::obs::CObservation2DRangeScan::Create();
  mrpt::obs::stock_observations::example2DRangeScan(*scan, index);
  return scan;
}
}  // namespace

TEST(observationsOverlap, identicalScansFullOverlap)
{
  const auto s = makeScan(0);
  EXPECT_NEAR(observationsOverlap(s, s), 1.0, 1e-6);
}

TEST(observationsOverlap, displacedScansDoNotOverlap)
{
  const auto s = makeScan(0);

  // Move the second scan 10 m away: no point can be matched anymore.
  const mrpt::poses::CPose3D farAway(10.0, 0, 0, 0, 0, 0);
  EXPECT_NEAR(observationsOverlap(s, s, &farAway), 0.0, 1e-6);
}

TEST(observationsOverlap, differentScansPartialOverlap)
{
  const auto s0 = makeScan(0);
  const auto s1 = makeScan(1);

  const double ov = observationsOverlap(s0, s1);
  EXPECT_GT(ov, 0.0);
  EXPECT_LT(ov, 1.0);
}

TEST(observationsOverlap, nonLaserObservationsGiveZero)
{
  auto odo = mrpt::obs::CObservationOdometry::Create();
  const auto s = makeScan(0);

  EXPECT_EQ(observationsOverlap(odo, odo), 0.0);
  EXPECT_EQ(observationsOverlap(odo, s), 0.0);
  EXPECT_EQ(observationsOverlap(s, odo), 0.0);
}

TEST(observationsOverlap, sensoryFrameAveragesPairs)
{
  const auto s0 = makeScan(0);
  const auto s1 = makeScan(1);

  auto sf1 = mrpt::obs::CSensoryFrame::Create();
  sf1->insert(s0);

  auto sf2 = mrpt::obs::CSensoryFrame::Create();
  sf2->insert(s0);
  sf2->insert(s1);

  // Average of the 1x2 pairs:
  const double expected = 0.5 * (observationsOverlap(s0, s0) + observationsOverlap(s0, s1));
  EXPECT_NEAR(observationsOverlap(sf1, sf2), expected, 1e-9);
  EXPECT_NEAR(observationsOverlap(*sf1, *sf2), expected, 1e-9);
}

TEST(observationsOverlap, emptySensoryFramesGiveZero)
{
  auto empty = mrpt::obs::CSensoryFrame::Create();
  auto sf = mrpt::obs::CSensoryFrame::Create();
  sf->insert(makeScan(0));

  EXPECT_EQ(observationsOverlap(empty, empty), 0.0);
  EXPECT_EQ(observationsOverlap(empty, sf), 0.0);
  EXPECT_EQ(observationsOverlap(sf, empty), 0.0);
}

// The relative pose argument must be honored by the sensory-frame overload
// too, otherwise callers silently compare observations as if co-located.
TEST(observationsOverlap, sensoryFrameHonorsRelativePose)
{
  const auto s = makeScan(0);

  auto sf1 = mrpt::obs::CSensoryFrame::Create();
  sf1->insert(s);
  auto sf2 = mrpt::obs::CSensoryFrame::Create();
  sf2->insert(s);

  const mrpt::poses::CPose3D farAway(10.0, 0, 0, 0, 0, 0);

  EXPECT_NEAR(observationsOverlap(sf1, sf2), 1.0, 1e-6);
  EXPECT_NEAR(observationsOverlap(sf1, sf2, &farAway), 0.0, 1e-6);
  EXPECT_NEAR(observationsOverlap(*sf1, *sf2, &farAway), 0.0, 1e-6);
}
