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

#include <mrpt/apps_cli/rawlog_ops.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose3D.h>

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

/** Returns false to abort a long operation. Its argument is the progress, in [0,1]. */
using progress_callback_t = mrpt::apps::rawlog_ops::progress_callback_t;

/** Statistics of the observations of one sensor label. */
struct SensorStats
{
  std::string label;
  std::set<std::string> classNames;
  size_t count = 0;
  /** Timeline times of its observations (sorted). */
  std::vector<double> times;
  mrpt::Clock::time_point firstTimestamp = INVALID_TIMESTAMP;
  mrpt::Clock::time_point lastTimestamp = INVALID_TIMESTAMP;

  /** Average rate in Hz, or 0 if unknown. */
  [[nodiscard]] double rate() const;
};

/** A rawlog loaded in memory, with an index for the timeline and the sensor
 * panels, and the in-memory edit operations of the viewer. No GUI code. */
class RawlogDocument
{
 public:
  mrpt::obs::CRawlog rawlog;
  std::string fileName;
  /** Unsaved changes. */
  bool modified = false;

  struct LoadOptions
  {
    /** Entries before this one are skipped. */
    size_t firstEntry = 0;
    /** Maximum number of entries to load (0 = all). */
    size_t maxEntries = 0;
  };

  /** Loads a rawlog. Returns nullopt if `progress` aborted it. Throws on
   * errors (a truncated file is not an error: what was read is kept, and a
   * warning added). */
  static std::optional<RawlogDocument> load(
      const std::string& file,
      const LoadOptions& options,
      const progress_callback_t& progress,
      std::vector<std::string>& warnings);

  /** Saves the rawlog (compressed). Returns false if `progress` aborted it. */
  bool save(const std::string& file, const progress_callback_t& progress);

  /** Must be called after modifying `rawlog`. */
  void rebuildIndex();

  /** @name Index
   * @{ */

  /** True if the timeline uses timestamps (in seconds since the first
   * one); false if it uses entry indices (rawlogs without timestamps). */
  [[nodiscard]] bool usesTimestamps() const { return m_usesTimestamps; }

  [[nodiscard]] mrpt::Clock::time_point firstTimestamp() const { return m_firstTimestamp; }
  [[nodiscard]] mrpt::Clock::time_point lastTimestamp() const { return m_lastTimestamp; }

  /** Timeline time of each entry. */
  [[nodiscard]] double entryTime(size_t index) const { return m_entryTimes.at(index); }

  /** Timeline times of all entries, sorted. */
  [[nodiscard]] const std::vector<double>& sortedEntryTimes() const { return m_sortedTimes; }

  /** Entry at position `k` of sortedEntryTimes(). */
  [[nodiscard]] size_t sortedEntry(size_t k) const { return m_sortedEntries.at(k); }

  /** Position of an entry in sortedEntryTimes(). */
  [[nodiscard]] size_t sortedPosition(size_t index) const { return m_sortedPosition.at(index); }

  /** The entry at the given timeline time (the last one not after it). */
  [[nodiscard]] std::optional<size_t> entryAtTime(double t) const;

  /** Statistics of each sensor label. */
  [[nodiscard]] const std::map<std::string, SensorStats>& sensors() const { return m_sensors; }

  /** Counts of entries by class name (top level and within sensory frames). */
  [[nodiscard]] const std::map<std::string, size_t>& classCounts() const { return m_classCounts; }

  /** The latest observation of a sensor at or before timeline time `t`. */
  [[nodiscard]] mrpt::obs::CObservation::Ptr latestObservation(
      const std::string& label, double t) const;

  /** Timeline time of a timestamp. */
  [[nodiscard]] double toTimelineTime(mrpt::Clock::time_point t) const;

  /** Multi-line description of the rawlog: format, entries, duration... */
  [[nodiscard]] std::string summaryText() const;

  /** @} */

  /** @name Edit operations
   * All return the number of modified (or removed) objects.
   * @{ */

  /** Keeps (or removes, if `remove`) the entries in [first, last]. */
  size_t keepRange(size_t first, size_t last, bool remove);

  /** Removes observations (top level, or within sensory frames) for which
   * `pred` returns true. Empty sensory frames are kept. */
  size_t removeObservations(const std::function<bool(const mrpt::obs::CObservation&)>& pred);

  /** Removes 2D scans with an absolute pitch or roll above the threshold. */
  size_t keepHorizontalScansOnly(double maxPitchDeg);

  /** Swaps the red and blue channels of all images. */
  size_t swapRedBlue();

  size_t renameSensor(const std::string& oldLabel, const std::string& newLabel);

  /** Sets the sensor pose of all observations with the given label. If
   * `onlyXYZ`, the orientation is kept. */
  size_t setSensorPose(const std::string& label, const mrpt::poses::CPose3D& pose, bool onlyXYZ);

  /** Sets the camera parameters of all monocular images with the given label. */
  size_t setCameraParams(const std::string& label, const mrpt::img::TCamera& params);

  /** Applies sensor poses (and optionally camera parameters) from a config
   * text with one section per sensor (keys: sensorLabel, pose_x, pose_y,
   * pose_z, pose_yaw, pose_pitch, pose_roll in degrees, and optionally
   * calib_params=[fx fy cx cy] and distort_params=[k1 k2 p1 p2]). */
  size_t applySensorPosesFromConfig(const std::string& configText);

  /** Recomputes the odometry actions in [first, last] with a new motion
   * model. */
  size_t setMotionModel(
      const mrpt::obs::CActionRobotMovement2D::TMotionModelOptions& opts,
      size_t first,
      size_t last);

  /** The motion model of the first odometry action, if any. */
  [[nodiscard]] std::optional<mrpt::obs::CActionRobotMovement2D::TMotionModelOptions>
  firstMotionModel() const;

  /** Recovers the camera parameters of the 3D range scans with the given
   * label (all, if empty) from the first one, and applies them to all.
   * Returns the number of changed observations, and the reprojection error. */
  std::pair<size_t, double> recoverCameraParams3D(const std::string& label);

  /** @} */

  /** Builds a map by inserting the observations of entries [first, last]
   * (one out of each `decimation`) at the robot poses from odometry. The
   * maps are defined by a config text, as in
   * mrpt::maps::TSetOfMetricMapInitializers. `path` gets the robot pose at
   * each observation. Returns false if `progress` aborted it. */
  bool buildRawMap(
      const std::string& configText,
      size_t first,
      size_t last,
      size_t decimation,
      const progress_callback_t& progress,
      std::shared_ptr<mrpt::maps::CMultiMetricMap>& map,
      std::vector<mrpt::poses::CPose3D>& path);

  /** Writes one line per observation of the class (and label, if not
   * empty) to a text file, with class-specific columns. Returns the number
   * of lines. */
  size_t exportToText(
      const std::string& className, const std::string& label, const std::string& file) const;

  /** Classes with columns known to exportToText(). */
  [[nodiscard]] static std::vector<std::string> exportableClasses();

  /** Calls `f` for each observation (top level and within sensory
   * frames), with the entry index. */
  void forEachObservation(
      const std::function<void(size_t entry, const mrpt::obs::CObservation::Ptr&)>& f) const;

 private:
  bool m_usesTimestamps = false;
  mrpt::Clock::time_point m_firstTimestamp = INVALID_TIMESTAMP;
  mrpt::Clock::time_point m_lastTimestamp = INVALID_TIMESTAMP;
  std::vector<double> m_entryTimes;
  std::vector<double> m_sortedTimes;
  std::vector<size_t> m_sortedEntries;
  std::vector<size_t> m_sortedPosition;
  std::map<std::string, SensorStats> m_sensors;
  std::map<std::string, size_t> m_classCounts;

  struct ObsRef
  {
    double time;
    size_t entry;
    int indexInSF;  // -1 for top-level observations
  };
  std::map<std::string, std::vector<ObsRef>> m_obsByLabel;

  [[nodiscard]] mrpt::obs::CObservation::Ptr resolve(const ObsRef& r) const;
};
