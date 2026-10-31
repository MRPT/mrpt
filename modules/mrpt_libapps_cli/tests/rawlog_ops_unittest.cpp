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
#include <mrpt/apps_cli/rawlog_ops.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <chrono>

using namespace mrpt::obs;
using namespace mrpt::apps;

namespace
{
const auto t0 = mrpt::Clock::fromDouble(1000.0);

mrpt::Clock::time_point at(double s)
{
  return t0 + std::chrono::duration_cast<mrpt::Clock::duration>(std::chrono::duration<double>(s));
}

CObservation::Ptr makeObs(const std::string& label, double t)
{
  auto o = CObservationIMU::Create();
  o->sensorLabel = label;
  o->timestamp = at(t);
  return o;
}

/** N steps of: action (1 m forward), SF with "a" and "b" observations. */
CRawlog makeSFRawlog(size_t N)
{
  CRawlog r;
  r.setCommentText("test");
  for (size_t i = 0; i < N; i++)
  {
    CActionCollection acts;
    CActionRobotMovement2D mov;
    CActionRobotMovement2D::TMotionModelOptions opts;
    mov.computeFromOdometry(mrpt::poses::CPose2D(1.0, 0, 0), opts);
    mov.timestamp = at(static_cast<double>(i));
    acts.insert(mov);
    r.insert(acts);

    CSensoryFrame sf;
    sf.insert(makeObs("a", static_cast<double>(i) + 0.1));
    sf.insert(makeObs("b", static_cast<double>(i) + 0.2));
    r.insert(sf);
  }
  return r;
}

size_t countObservations(const CRawlog& r)
{
  size_t n = 0;
  for (size_t i = 0; i < r.size(); i++)
  {
    if (r.getType(i) == CRawlog::TEntryType::etSensoryFrame)
    {
      n += std::dynamic_pointer_cast<const CSensoryFrame>(r.getAsGeneric(i))->size();
    }
    else if (r.getType(i) == CRawlog::TEntryType::etObservation)
    {
      n++;
    }
  }
  return n;
}

/** True if entries alternate action, SF, action, SF... */
bool isActionSFAlternating(const CRawlog& r)
{
  for (size_t i = 0; i < r.size(); i++)
  {
    const auto expected = (i % 2 == 0) ? CRawlog::TEntryType::etActionCollection
                                       : CRawlog::TEntryType::etSensoryFrame;
    if (r.getType(i) != expected)
    {
      return false;
    }
  }
  return r.size() % 2 == 0;
}

double totalOdometryX(const CRawlog& r)
{
  double x = 0;
  for (size_t i = 0; i < r.size(); i++)
  {
    if (r.getType(i) != CRawlog::TEntryType::etActionCollection)
    {
      continue;
    }
    auto acts = std::dynamic_pointer_cast<const CActionCollection>(r.getAsGeneric(i));
    if (auto m = acts->getBestMovementEstimation(); m)
    {
      x += m->poseChange->getMeanVal().x();
    }
  }
  return x;
}
}  // namespace

TEST(rawlog_ops, decimate)
{
  auto r = makeSFRawlog(10);
  const auto kept = rawlog_ops::decimate(r, 3);
  EXPECT_EQ(kept, 3u);
  EXPECT_TRUE(isActionSFAlternating(r));
  EXPECT_EQ(countObservations(r), 3u * 2u);  // one SF out of each 3
  EXPECT_EQ(r.getCommentText(), "test");
  // The odometry of the 9 steps that ended in a kept SF is preserved:
  EXPECT_NEAR(totalOdometryX(r), 9.0, 1e-6);
}

TEST(rawlog_ops, losslessDecimate)
{
  auto r = makeSFRawlog(10);
  const auto kept = rawlog_ops::losslessDecimate(r, 3);
  EXPECT_EQ(kept, 4u);                   // 3 full groups + the remainder
  EXPECT_EQ(countObservations(r), 20u);  // nothing lost
  EXPECT_TRUE(isActionSFAlternating(r));
  EXPECT_NEAR(totalOdometryX(r), 10.0, 1e-6);
}

TEST(rawlog_ops, decimateRejectsObservationsOnly)
{
  CRawlog r;
  r.insert(makeObs("a", 0));
  EXPECT_THROW(rawlog_ops::decimate(r, 2), std::exception);
  EXPECT_TRUE(rawlog_ops::isObservationsOnly(r));
}

TEST(rawlog_ops, compact)
{
  CRawlog r;
  for (int i = 0; i < 3; i++)
  {
    CActionCollection acts;
    CActionRobotMovement2D mov;
    CActionRobotMovement2D::TMotionModelOptions opts;
    mov.computeFromOdometry(mrpt::poses::CPose2D(1.0, 0, 0), opts);
    acts.insert(mov);
    r.insert(acts);  // three consecutive actions
  }
  {
    CSensoryFrame empty;
    r.insert(empty);
    CSensoryFrame sf1;
    sf1.insert(makeObs("a", 0.1));
    r.insert(sf1);
    CSensoryFrame sf2;
    sf2.insert(makeObs("a", 0.2));
    sf2.insert(makeObs("b", 0.3));
    r.insert(sf2);
  }
  auto r2 = r;  // shallow copy: shares the entries with "r"
  const auto st = rawlog_ops::compact(r, false);
  EXPECT_EQ(r2.size(), 6u);
  EXPECT_EQ(countObservations(r2), 3u);  // the input entries are not modified
  EXPECT_NEAR(totalOdometryX(r2), 3.0, 1e-6);
  EXPECT_EQ(st.actionsRemoved, 2u);
  EXPECT_EQ(st.sensoryFramesRemoved, 2u);  // the empty one and the merged one
  EXPECT_EQ(r.size(), 2u);
  EXPECT_EQ(countObservations(r), 3u);
  EXPECT_NEAR(totalOdometryX(r), 3.0, 1e-6);

  rawlog_ops::compact(r2, true /*one per label*/);
  EXPECT_EQ(countObservations(r2), 2u);
}

TEST(rawlog_ops, toObservationsOnlyAndBack)
{
  auto r = makeSFRawlog(5);
  rawlog_ops::toObservationsOnly(r);
  EXPECT_TRUE(rawlog_ops::isObservationsOnly(r));
  EXPECT_EQ(r.size(), 5u * 3u);  // a, b + odometry per step

  // Sorted by timestamp, and odometry accumulated:
  mrpt::Clock::time_point last = INVALID_TIMESTAMP;
  double lastOdoX = 0;
  for (size_t i = 0; i < r.size(); i++)
  {
    auto o = r.getAsObservation(i);
    if (last != INVALID_TIMESTAMP)
    {
      EXPECT_GE(o->timestamp, last);
    }
    last = o->timestamp;
    if (auto odo = std::dynamic_pointer_cast<CObservationOdometry>(o); odo)
    {
      EXPECT_NEAR(odo->odometry.x(), lastOdoX + 1.0, 1e-6);
      lastOdoX = odo->odometry.x();
    }
  }

  rawlog_ops::toSensoryFrames(r, 0.5, false);
  EXPECT_FALSE(rawlog_ops::isObservationsOnly(r));
  EXPECT_TRUE(isActionSFAlternating(r));
  EXPECT_EQ(countObservations(r), 15u);
  // The odometry increments between the first and the last SF survive:
  EXPECT_NEAR(totalOdometryX(r), 4.0, 1e-6);
}

TEST(rawlog_ops, sortShiftAndRegenerate)
{
  CRawlog r;
  r.insert(makeObs("a", 2.0));
  r.insert(makeObs("b", 1.0));
  r.insert(makeObs("a", 3.0));
  rawlog_ops::sortByTimestamp(r);
  EXPECT_EQ(r.getAsObservation(0)->sensorLabel, "b");

  EXPECT_EQ(rawlog_ops::shiftTimestamps(r, {"b"}, 5.0), 1u);
  rawlog_ops::sortByTimestamp(r);
  EXPECT_EQ(r.getAsObservation(2)->sensorLabel, "b");
  EXPECT_NEAR(mrpt::Clock::toDouble(r.getAsObservation(2)->timestamp), 1006.0, 1e-6);

  // Regenerate timestamps within sensory frames:
  CRawlog r2;
  CSensoryFrame sf;
  auto bad = makeObs("x", 0);
  bad->timestamp = INVALID_TIMESTAMP;
  sf.insert(bad);
  sf.insert(makeObs("y", 7.0));
  r2.insert(sf);
  EXPECT_EQ(rawlog_ops::regenerateTimestampsInSensoryFrames(r2), 1u);
  EXPECT_EQ(bad->timestamp, at(7.0));
}

TEST(rawlog_ops, progressCanAbort)
{
  auto r = makeSFRawlog(200);
  const auto before = r.size();
  rawlog_ops::decimate(r, 2, [](double) { return false; });
  EXPECT_EQ(r.size(), before);  // unchanged
}
