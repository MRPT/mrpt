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

#include <mrpt/apps_cli/rawlog_ops.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <set>

using namespace mrpt::obs;
using namespace mrpt::poses;

namespace mrpt::apps::rawlog_ops
{
namespace
{
/** Calls the progress callback every few entries: false means "abort". */
bool reportProgress(const progress_callback_t& progress, size_t i, size_t n)
{
  if (!progress || n == 0 || (i % 64) != 0)
  {
    return true;
  }
  return progress(static_cast<double>(i) / static_cast<double>(n));
}

void requireSensoryFrames(const CRawlog& rawlog)
{
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    if (rawlog.getType(i) == CRawlog::TEntryType::etObservation)
    {
      THROW_EXCEPTION("This operation is for SensoryFrame-based rawlogs only.");
    }
  }
}

void requireObservationsOnly(const CRawlog& rawlog)
{
  if (!isObservationsOnly(rawlog))
  {
    THROW_EXCEPTION("This operation is for observations-only rawlogs.");
  }
}
}  // namespace

bool isObservationsOnly(const CRawlog& rawlog)
{
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    const auto t = rawlog.getType(i);
    if (t == CRawlog::TEntryType::etSensoryFrame || t == CRawlog::TEntryType::etActionCollection)
    {
      return false;
    }
  }
  return true;
}

size_t decimate(CRawlog& rawlog, size_t ratio, const progress_callback_t& progress)
{
  ASSERT_GE_(ratio, 1U);
  requireSensoryFrames(rawlog);

  CRawlog out;
  out.setCommentText(rawlog.getCommentText());

  CActionRobotMovement2D::TMotionModelOptions odoOptions;
  bool movInit = false;
  CPose2D accumMovement(0, 0, 0);
  size_t sfCounter = 0;
  size_t kept = 0;

  const size_t N = rawlog.size();
  for (size_t i = 0; i < N; i++)
  {
    if (!reportProgress(progress, i, N))
    {
      return 0;
    }
    auto obj = rawlog.getAsGeneric(i);
    if (rawlog.getType(i) == CRawlog::TEntryType::etActionCollection)
    {
      auto acts = std::dynamic_pointer_cast<CActionCollection>(obj);
      if (auto mov = acts->getBestMovementEstimation(); mov)
      {
        accumMovement = accumMovement + mov->poseChange->getMeanVal();
        if (!movInit)
        {
          odoOptions = mov->motionModelConfiguration;
          movInit = true;
        }
      }
    }
    else if (rawlog.getType(i) == CRawlog::TEntryType::etSensoryFrame)
    {
      if (++sfCounter < ratio)
      {
        continue;
      }
      // Keep the last SF of each group, preceded by the motion since the
      // previously kept one:
      sfCounter = 0;
      CActionCollection acts;
      if (movInit)
      {
        CActionRobotMovement2D mov;
        mov.computeFromOdometry(accumMovement, odoOptions);
        acts.insert(mov);
        accumMovement = CPose2D(0, 0, 0);
      }
      out.insert(acts);
      out.insert(obj);
      kept++;
    }
  }
  rawlog = std::move(out);
  return kept;
}

size_t losslessDecimate(CRawlog& rawlog, size_t ratio, const progress_callback_t& progress)
{
  ASSERT_GE_(ratio, 1U);
  requireSensoryFrames(rawlog);

  CRawlog out;
  out.setCommentText(rawlog.getCommentText());

  CActionRobotMovement2D::TMotionModelOptions odoOptions;
  bool movInit = false;
  CPose2D accumMovement(0, 0, 0);
  CSensoryFrame::Ptr accumSF;
  size_t sfCounter = 0;
  size_t kept = 0;

  // Writes the motion since the last group, then the merged group, which
  // is referred to the pose of its last SF:
  const auto flushGroup = [&]()
  {
    CActionCollection acts;
    if (movInit)
    {
      CActionRobotMovement2D mov;
      mov.computeFromOdometry(accumMovement, odoOptions);
      acts.insert(mov);
      accumMovement = CPose2D(0, 0, 0);
    }
    out.insert(acts);
    out.insert(accumSF);
    accumSF.reset();
    kept++;
  };

  const size_t N = rawlog.size();
  for (size_t i = 0; i < N; i++)
  {
    if (!reportProgress(progress, i, N))
    {
      return 0;
    }
    auto obj = rawlog.getAsGeneric(i);
    if (rawlog.getType(i) == CRawlog::TEntryType::etActionCollection)
    {
      auto acts = std::dynamic_pointer_cast<CActionCollection>(obj);
      if (auto mov = acts->getBestMovementEstimation(); mov)
      {
        const CPose2D incr = mov->poseChange->getMeanVal();
        // Observations already accumulated were taken from a pose that is
        // now "behind" the robot: refer their sensor poses to the new pose.
        if (accumSF)
        {
          const CPose3D invIncr = CPose3D(0, 0, 0) - CPose3D(incr);
          for (auto& o : *accumSF)
          {
            o->setSensorPose(invIncr + o->getSensorPose());
          }
        }
        accumMovement = accumMovement + incr;
        if (!movInit)
        {
          odoOptions = mov->motionModelConfiguration;
          movInit = true;
        }
      }
    }
    else if (rawlog.getType(i) == CRawlog::TEntryType::etSensoryFrame)
    {
      if (!accumSF)
      {
        accumSF = CSensoryFrame::Create();
      }
      // Copies, since their sensor poses may be modified:
      for (const auto& o : *std::dynamic_pointer_cast<CSensoryFrame>(obj))
      {
        accumSF->insert(std::dynamic_pointer_cast<CObservation>(o->duplicateGetSmartPtr()));
      }
      if (++sfCounter >= ratio)
      {
        sfCounter = 0;
        flushGroup();
      }
    }
  }
  if (accumSF && !accumSF->empty())
  {
    flushGroup();
  }
  rawlog = std::move(out);
  return kept;
}

CompactStats compact(CRawlog& rawlog, bool onlyOnePerLabel, const progress_callback_t& progress)
{
  requireSensoryFrames(rawlog);
  CompactStats stats;

  CRawlog out;
  out.setCommentText(rawlog.getCommentText());

  CActionRobotMovement2D::Ptr lastAct;
  CActionCollection::Ptr lastActs;
  CSensoryFrame::Ptr lastSF;

  const auto finishSF = [&]()
  {
    if (!lastSF || !onlyOnePerLabel)
    {
      return;
    }
    auto newSF = CSensoryFrame::Create();
    std::set<std::string> labels;
    for (const auto& o : *lastSF)
    {
      if (labels.insert(o->sensorLabel).second)
      {
        newSF->insert(o);
      }
    }
    *lastSF = std::move(*newSF);
  };

  const size_t N = rawlog.size();
  for (size_t i = 0; i < N; i++)
  {
    if (!reportProgress(progress, i, N))
    {
      return {};
    }
    auto obj = rawlog.getAsGeneric(i);
    if (rawlog.getType(i) == CRawlog::TEntryType::etActionCollection)
    {
      auto acts = std::dynamic_pointer_cast<CActionCollection>(obj);
      auto mov = acts->getMovementEstimationByType(CActionRobotMovement2D::emOdometry);
      if (lastAct && mov)
      {
        // Merge with the previous action:
        lastAct->computeFromOdometry(
            lastAct->rawOdometryIncrementReading + mov->rawOdometryIncrementReading,
            lastAct->motionModelConfiguration);
        stats.actionsRemoved++;
        continue;
      }
      finishSF();
      lastSF.reset();
      // A deep copy, since it may be modified by later merges:
      lastActs = std::dynamic_pointer_cast<CActionCollection>(acts->duplicateGetSmartPtr());
      lastAct = lastActs->getMovementEstimationByType(CActionRobotMovement2D::emOdometry);
      out.insert(lastActs);
    }
    else if (rawlog.getType(i) == CRawlog::TEntryType::etSensoryFrame)
    {
      auto sf = std::dynamic_pointer_cast<CSensoryFrame>(obj);
      if (sf->empty())
      {
        stats.sensoryFramesRemoved++;
        continue;
      }
      if (lastSF)
      {
        // Merge with the previous sensory frame:
        for (const auto& o : *sf)
        {
          lastSF->insert(o);
        }
        stats.sensoryFramesRemoved++;
        continue;
      }
      // A copy of the container (observations are shared), since more
      // observations may be merged into it:
      lastSF = std::dynamic_pointer_cast<CSensoryFrame>(sf->duplicateGetSmartPtr());
      lastAct.reset();
      out.insert(lastSF);
    }
  }
  finishSF();
  rawlog = std::move(out);
  return stats;
}

void toObservationsOnly(CRawlog& rawlog, const progress_callback_t& progress)
{
  std::multimap<mrpt::Clock::time_point, CObservation::Ptr> sorted;
  CPose2D odoAccum;
  mrpt::Clock::time_point lastValidObsTime = INVALID_TIMESTAMP;

  const size_t N = rawlog.size();
  for (size_t i = 0; i < N; i++)
  {
    if (!reportProgress(progress, i, N))
    {
      return;
    }
    auto obj = rawlog.getAsGeneric(i);
    switch (rawlog.getType(i))
    {
      case CRawlog::TEntryType::etSensoryFrame:
        for (const auto& o : *std::dynamic_pointer_cast<CSensoryFrame>(obj))
        {
          sorted.emplace(o->timestamp, o);
          lastValidObsTime = o->timestamp;
        }
        break;
      case CRawlog::TEntryType::etActionCollection:
      {
        auto acts = std::dynamic_pointer_cast<CActionCollection>(obj);
        if (auto mov = acts->getBestMovementEstimation(); mov)
        {
          odoAccum = odoAccum + mov->poseChange->getMeanVal();
          auto odo = CObservationOdometry::Create();
          odo->sensorLabel = "odometry";
          odo->timestamp = mov->timestamp != INVALID_TIMESTAMP ? mov->timestamp : lastValidObsTime;
          odo->odometry = odoAccum;
          sorted.emplace(odo->timestamp, odo);
        }
        break;
      }
      case CRawlog::TEntryType::etObservation:
      {
        auto o = std::dynamic_pointer_cast<CObservation>(obj);
        sorted.emplace(o->timestamp, o);
        break;
      }
      default:
        break;
    }
  }
  CRawlog out;
  out.setCommentText(rawlog.getCommentText());
  for (const auto& [t, o] : sorted)
  {
    out.insert(o);
  }
  rawlog = std::move(out);
}

void toSensoryFrames(
    CRawlog& rawlog,
    double maxLengthSeconds,
    bool onlyOnePerLabel,
    const progress_callback_t& progress)
{
  requireObservationsOnly(rawlog);

  CRawlog out;
  out.setCommentText(rawlog.getCommentText());

  CSensoryFrame sf;
  std::set<std::string> sfLabels;
  mrpt::Clock::time_point sfFirstTime = INVALID_TIMESTAMP;
  CObservationOdometry::Ptr lastSFOdo;
  CObservationOdometry::Ptr curSFOdo;

  // Writes the odometry increment since the previous SF, then the SF:
  const auto flushSF = [&]()
  {
    CActionCollection acts;
    if (lastSFOdo && curSFOdo)
    {
      CActionRobotMovement2D act;
      act.timestamp = curSFOdo->timestamp;
      CActionRobotMovement2D::TMotionModelOptions opts;
      act.computeFromOdometry(curSFOdo->odometry - lastSFOdo->odometry, opts);
      acts.insert(act);
    }
    out.insert(acts);
    out.insert(sf);
    if (curSFOdo)
    {
      lastSFOdo = curSFOdo;
    }
    curSFOdo.reset();
    sf.clear();
    sfLabels.clear();
    sfFirstTime = INVALID_TIMESTAMP;
  };

  const size_t N = rawlog.size();
  for (size_t i = 0; i < N; i++)
  {
    if (!reportProgress(progress, i, N))
    {
      return;
    }
    auto o = rawlog.getAsObservation(i);
    const double len = sfFirstTime == INVALID_TIMESTAMP
                           ? 0
                           : mrpt::system::timeDifference(sfFirstTime, o->timestamp);
    if (len > maxLengthSeconds && sf.size() != 0)
    {
      flushSF();
    }
    if (!onlyOnePerLabel || sfLabels.count(o->sensorLabel) == 0)
    {
      sf.insert(o);
      sfLabels.insert(o->sensorLabel);
    }
    if (sfFirstTime == INVALID_TIMESTAMP)
    {
      sfFirstTime = o->timestamp;
    }
    if (auto odo = std::dynamic_pointer_cast<CObservationOdometry>(o); odo)
    {
      curSFOdo = odo;
    }
  }
  if (sf.size() != 0)
  {
    flushSF();
  }
  rawlog = std::move(out);
}

void sortByTimestamp(CRawlog& rawlog, bool useSensorTimestamp)
{
  requireObservationsOnly(rawlog);
  std::multimap<mrpt::Clock::time_point, size_t> sorted;
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    if (rawlog.getType(i) != CRawlog::TEntryType::etObservation)
    {
      continue;
    }
    const auto o = rawlog.getAsObservation(i);
    const auto t = useSensorTimestamp ? o->getTimeStamp() : o->getOriginalReceivedTimeStamp();
    if (t == INVALID_TIMESTAMP)
    {
      THROW_EXCEPTION_FMT("Entry %zu does not have a valid timestamp.", i);
    }
    sorted.emplace(t, i);
  }
  CRawlog out;
  out.setCommentText(rawlog.getCommentText());
  for (const auto& [t, idx] : sorted)
  {
    out.insert(rawlog.getAsObservation(idx));
  }
  rawlog = std::move(out);
}

size_t shiftTimestamps(CRawlog& rawlog, const std::vector<std::string>& labels, double seconds)
{
  const auto dt =
      std::chrono::duration_cast<mrpt::Clock::duration>(std::chrono::duration<double>(seconds));
  // Invalid timestamps stay invalid:
  const auto selected = [&](const CObservation& o)
  {
    return o.timestamp != INVALID_TIMESTAMP &&
           (labels.empty() ||
            std::find(labels.begin(), labels.end(), o.sensorLabel) != labels.end());
  };

  size_t n = 0;
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    switch (rawlog.getType(i))
    {
      case CRawlog::TEntryType::etSensoryFrame:
        for (auto& o : *rawlog.getAsObservations(i))
        {
          if (selected(*o))
          {
            o->timestamp += dt;
            n++;
          }
        }
        break;
      case CRawlog::TEntryType::etObservation:
      {
        auto o = rawlog.getAsObservation(i);
        if (selected(*o))
        {
          o->timestamp += dt;
          n++;
        }
        break;
      }
      default:
        break;
    }
  }
  return n;
}

size_t regenerateTimestampsInSensoryFrames(CRawlog& rawlog)
{
  size_t n = 0;
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    if (rawlog.getType(i) != CRawlog::TEntryType::etSensoryFrame)
    {
      continue;
    }
    auto sf = rawlog.getAsObservations(i);
    mrpt::Clock::time_point t = INVALID_TIMESTAMP;
    for (const auto& o : *sf)
    {
      if (o->timestamp != INVALID_TIMESTAMP)
      {
        t = o->timestamp;
        break;
      }
    }
    if (t == INVALID_TIMESTAMP)
    {
      continue;
    }
    for (auto& o : *sf)
    {
      if (o->timestamp == INVALID_TIMESTAMP)
      {
        o->timestamp = t;
        n++;
      }
    }
  }
  return n;
}

}  // namespace mrpt::apps::rawlog_ops
