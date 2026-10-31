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
#pragma once

#include <mrpt/obs/CRawlog.h>

#include <functional>
#include <string>
#include <vector>

/** In-memory transformations of rawlogs, shared by rawlog-edit and the GUI
 * tools. All of them work on a whole mrpt::obs::CRawlog.
 *
 * Long operations take an optional progress callback, called with a fraction
 * in [0,1]; if it returns false, the operation is aborted and the rawlog is
 * left unchanged.
 */
namespace mrpt::apps::rawlog_ops
{
using progress_callback_t = std::function<bool(double fraction)>;

/** Keeps one out of each `ratio` sensory frames of a SensoryFrame-based
 * rawlog, accumulating the odometry of the discarded steps into the kept
 * actions. \return The number of sensory frames kept. */
size_t decimate(mrpt::obs::CRawlog& rawlog, size_t ratio, const progress_callback_t& progress = {});

/** Like decimate(), but no observation is lost: each group of `ratio`
 * sensory frames is merged into one, with the sensor poses of the earlier
 * observations corrected by the odometry since then. SensoryFrame-based
 * rawlogs only. \return The number of sensory frames in the result. */
size_t losslessDecimate(
    mrpt::obs::CRawlog& rawlog, size_t ratio, const progress_callback_t& progress = {});

struct CompactStats
{
  size_t actionsRemoved = 0;
  size_t sensoryFramesRemoved = 0;
};

/** Compacts a SensoryFrame-based rawlog: consecutive actions are merged into
 * one (their odometry added up), consecutive sensory frames into one, and
 * empty sensory frames are removed. If `onlyOnePerLabel`, only the first
 * observation of each sensor label is kept in each sensory frame. */
CompactStats compact(
    mrpt::obs::CRawlog& rawlog, bool onlyOnePerLabel, const progress_callback_t& progress = {});

/** Converts a SensoryFrame-based rawlog into an observations-only one,
 * sorted by timestamp. Odometry actions become CObservationOdometry
 * observations (label "odometry") with the accumulated odometry. */
void toObservationsOnly(mrpt::obs::CRawlog& rawlog, const progress_callback_t& progress = {});

/** Converts an observations-only rawlog into a SensoryFrame-based one,
 * grouping observations into sensory frames up to `maxLengthSeconds` long,
 * with odometry actions between them computed from the CObservationOdometry
 * observations, if any. If `onlyOnePerLabel`, only the first observation of
 * each label is kept in each sensory frame. */
void toSensoryFrames(
    mrpt::obs::CRawlog& rawlog,
    double maxLengthSeconds,
    bool onlyOnePerLabel,
    const progress_callback_t& progress = {});

/** Sorts an observations-only rawlog by timestamp: the sensor timestamp, or
 * the computer-based one (CObservation::getOriginalReceivedTimeStamp()).
 * Throws if an observation has no valid timestamp. */
void sortByTimestamp(mrpt::obs::CRawlog& rawlog, bool useSensorTimestamp = true);

/** Shifts the timestamps of the observations with the given sensor labels
 * (all of them, if empty). \return The number of observations changed. */
size_t shiftTimestamps(
    mrpt::obs::CRawlog& rawlog, const std::vector<std::string>& labels, double seconds);

/** Within each sensory frame, gives the observations without a valid
 * timestamp the first valid one found in the same frame.
 * \return The number of observations changed. */
size_t regenerateTimestampsInSensoryFrames(mrpt::obs::CRawlog& rawlog);

/** True if the rawlog has no actions or sensory frames. */
[[nodiscard]] bool isObservationsOnly(const mrpt::obs::CRawlog& rawlog);

}  // namespace mrpt::apps::rawlog_ops
