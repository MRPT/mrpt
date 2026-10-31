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

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

using namespace mrpt::obs;

namespace
{
/** The timestamp of an entry: its own, the earliest one of a sensory
 * frame, or the one of the first action of a collection. */
mrpt::Clock::time_point entryTimestamp(const CRawlog& r, size_t i)
{
  switch (r.getType(i))
  {
    case CRawlog::TEntryType::etObservation:
      return r.getAsObservation(i)->timestamp;
    case CRawlog::TEntryType::etSensoryFrame:
    {
      mrpt::Clock::time_point t = INVALID_TIMESTAMP;
      for (const auto& o : *r.getAsObservations(i))
      {
        if (o && o->timestamp != INVALID_TIMESTAMP && (t == INVALID_TIMESTAMP || o->timestamp < t))
        {
          t = o->timestamp;
        }
      }
      return t;
    }
    case CRawlog::TEntryType::etActionCollection:
    {
      const auto acts = r.getAsAction(i);
      if (acts->size() != 0)
      {
        return acts->get(0)->timestamp;
      }
      return INVALID_TIMESTAMP;
    }
    default:
      return INVALID_TIMESTAMP;
  }
}

std::string fmtTime(mrpt::Clock::time_point t)
{
  return mrpt::format("%.06f", mrpt::Clock::toDouble(t));
}
}  // namespace

double SensorStats::rate() const
{
  if (count < 2 || firstTimestamp == INVALID_TIMESTAMP || lastTimestamp == INVALID_TIMESTAMP)
  {
    return 0;
  }
  const double dt = mrpt::system::timeDifference(firstTimestamp, lastTimestamp);
  return dt > 0 ? static_cast<double>(count - 1) / dt : 0;
}

// ---------------------------------------------------------------------------
// Load / save
// ---------------------------------------------------------------------------
std::optional<RawlogDocument> RawlogDocument::load(
    const std::string& file,
    const LoadOptions& options,
    const progress_callback_t& progress,
    std::vector<std::string>& warnings)
{
  mrpt::io::CCompressedInputStream f;
  if (!f.open(file))
  {
    THROW_EXCEPTION_FMT("Cannot open file: '%s'", file.c_str());
  }
  auto arch = mrpt::serialization::archiveFrom(f);
  const double totalBytes = static_cast<double>(std::max<uint64_t>(1, f.getTotalBytesCount()));

  RawlogDocument doc;
  doc.fileName = file;
  for (size_t idx = 0;; idx++)
  {
    if (progress && (idx % 64) == 0 &&
        !progress(std::min(1.0, static_cast<double>(f.getPosition()) / totalBytes)))
    {
      return std::nullopt;
    }
    mrpt::serialization::CSerializable::Ptr obj;
    try
    {
      obj = arch.ReadObject();
    }
    catch (const mrpt::serialization::CExceptionEOF&)
    {
      break;
    }
    catch (const std::exception& e)
    {
      warnings.push_back(mrpt::format(
          "Stopped reading after %zu entries (truncated or corrupted file?):\n%s", idx,
          mrpt::exception_to_str(e).c_str()));
      break;
    }
    if (!obj)
    {
      continue;
    }
    // Very old rawlogs are one serialized CRawlog object:
    if (auto r = std::dynamic_pointer_cast<CRawlog>(obj); r)
    {
      doc.rawlog = *r;
      continue;
    }
    // The comments are not an entry:
    if (std::dynamic_pointer_cast<CObservationComment>(obj))
    {
      doc.rawlog.insert(obj);
      continue;
    }
    if (idx < options.firstEntry)
    {
      continue;
    }
    doc.rawlog.insert(obj);
    if (options.maxEntries != 0 && doc.rawlog.size() >= options.maxEntries)
    {
      break;
    }
  }
  doc.rebuildIndex();
  return doc;
}

bool RawlogDocument::save(const std::string& file, const progress_callback_t& progress)
{
  // Write to a temporary file first, so an aborted save does not destroy
  // the original file:
  const std::string tmpFile = file + ".tmp";
  {
    mrpt::io::CCompressedOutputStream f;
    if (!f.open(tmpFile))
    {
      THROW_EXCEPTION_FMT("Cannot create file: '%s'", tmpFile.c_str());
    }
    auto arch = mrpt::serialization::archiveFrom(f);
    if (const auto comments = rawlog.getCommentText(); !comments.empty())
    {
      CObservationComment c;
      c.text = comments;
      arch << c;
    }
    const size_t N = rawlog.size();
    for (size_t i = 0; i < N; i++)
    {
      if (progress && (i % 64) == 0 && !progress(static_cast<double>(i) / static_cast<double>(N)))
      {
        f.close();
        mrpt::system::deleteFile(tmpFile);
        return false;
      }
      arch << *rawlog.getAsGeneric(i);
    }
  }
  if (mrpt::system::fileExists(file))
  {
    mrpt::system::deleteFile(file);
  }
  std::string err;
  if (!mrpt::system::renameFile(tmpFile, file, &err))
  {
    THROW_EXCEPTION_FMT(
        "Cannot rename '%s' to '%s': %s", tmpFile.c_str(), file.c_str(), err.c_str());
  }
  fileName = file;
  modified = false;
  return true;
}

// ---------------------------------------------------------------------------
// Index
// ---------------------------------------------------------------------------
void RawlogDocument::forEachObservation(
    const std::function<void(size_t entry, const CObservation::Ptr&)>& f) const
{
  // Callers may modify the observations, which belong to this document:
  auto& r = const_cast<CRawlog&>(rawlog);
  for (size_t i = 0; i < r.size(); i++)
  {
    switch (r.getType(i))
    {
      case CRawlog::TEntryType::etObservation:
        f(i, r.getAsObservation(i));
        break;
      case CRawlog::TEntryType::etSensoryFrame:
        for (auto& o : *r.getAsObservations(i))
        {
          if (o)
          {
            f(i, o);
          }
        }
        break;
      default:
        break;
    }
  }
}

double RawlogDocument::toTimelineTime(mrpt::Clock::time_point t) const
{
  if (t == INVALID_TIMESTAMP || m_firstTimestamp == INVALID_TIMESTAMP)
  {
    return 0;
  }
  return mrpt::system::timeDifference(m_firstTimestamp, t);
}

void RawlogDocument::rebuildIndex()
{
  const size_t N = rawlog.size();
  std::vector<mrpt::Clock::time_point> stamps(N, INVALID_TIMESTAMP);
  size_t nValid = 0;
  m_firstTimestamp = INVALID_TIMESTAMP;
  m_lastTimestamp = INVALID_TIMESTAMP;
  for (size_t i = 0; i < N; i++)
  {
    stamps[i] = entryTimestamp(rawlog, i);
    if (stamps[i] == INVALID_TIMESTAMP)
    {
      continue;
    }
    nValid++;
    if (m_firstTimestamp == INVALID_TIMESTAMP || stamps[i] < m_firstTimestamp)
    {
      m_firstTimestamp = stamps[i];
    }
    if (m_lastTimestamp == INVALID_TIMESTAMP || stamps[i] > m_lastTimestamp)
    {
      m_lastTimestamp = stamps[i];
    }
  }
  // In SF-based rawlogs, actions usually have no timestamp: half the
  // entries with timestamps is enough.
  m_usesTimestamps = nValid > 0 && nValid * 2 >= N;

  m_entryTimes.assign(N, 0.0);
  double prev = 0;
  for (size_t i = 0; i < N; i++)
  {
    if (!m_usesTimestamps)
    {
      m_entryTimes[i] = static_cast<double>(i);
      continue;
    }
    // Entries without a timestamp go with the previous one:
    m_entryTimes[i] = stamps[i] != INVALID_TIMESTAMP ? toTimelineTime(stamps[i]) : prev;
    prev = m_entryTimes[i];
  }

  m_sortedEntries.resize(N);
  std::iota(m_sortedEntries.begin(), m_sortedEntries.end(), 0);
  std::stable_sort(
      m_sortedEntries.begin(), m_sortedEntries.end(),
      [this](size_t a, size_t b) { return m_entryTimes[a] < m_entryTimes[b]; });
  m_sortedTimes.resize(N);
  m_sortedPosition.resize(N);
  for (size_t k = 0; k < N; k++)
  {
    m_sortedTimes[k] = m_entryTimes[m_sortedEntries[k]];
    m_sortedPosition[m_sortedEntries[k]] = k;
  }

  // Sensors and classes:
  m_sensors.clear();
  m_obsByLabel.clear();
  m_classCounts.clear();
  for (size_t i = 0; i < N; i++)
  {
    m_classCounts[rawlog.getAsGeneric(i)->GetRuntimeClass()->className]++;
    if (rawlog.getType(i) == CRawlog::TEntryType::etSensoryFrame)
    {
      const auto sf = rawlog.getAsObservations(i);
      for (size_t k = 0; k < sf->size(); k++)
      {
        const auto o = sf->getObservationByIndex(k);
        if (!o)
        {
          continue;
        }
        m_classCounts[o->GetRuntimeClass()->className]++;
        const double t = (m_usesTimestamps && o->timestamp != INVALID_TIMESTAMP)
                             ? toTimelineTime(o->timestamp)
                             : m_entryTimes[i];
        m_obsByLabel[o->sensorLabel].push_back({t, i, static_cast<int>(k)});
      }
    }
    else if (rawlog.getType(i) == CRawlog::TEntryType::etObservation)
    {
      m_obsByLabel[rawlog.getAsObservation(i)->sensorLabel].push_back({m_entryTimes[i], i, -1});
    }
  }
  for (auto& [label, refs] : m_obsByLabel)
  {
    std::stable_sort(
        refs.begin(), refs.end(), [](const ObsRef& a, const ObsRef& b) { return a.time < b.time; });
    auto& s = m_sensors[label];
    s.label = label;
    s.count = refs.size();
    s.times.reserve(refs.size());
    for (const auto& ref : refs)
    {
      s.times.push_back(ref.time);
      const auto o = resolve(ref);
      s.classNames.insert(o->GetRuntimeClass()->className);
      if (o->timestamp == INVALID_TIMESTAMP)
      {
        continue;
      }
      if (s.firstTimestamp == INVALID_TIMESTAMP || o->timestamp < s.firstTimestamp)
      {
        s.firstTimestamp = o->timestamp;
      }
      if (s.lastTimestamp == INVALID_TIMESTAMP || o->timestamp > s.lastTimestamp)
      {
        s.lastTimestamp = o->timestamp;
      }
    }
  }
}

std::optional<size_t> RawlogDocument::entryAtTime(double t) const
{
  if (m_sortedTimes.empty())
  {
    return std::nullopt;
  }
  auto it = std::upper_bound(m_sortedTimes.begin(), m_sortedTimes.end(), t);
  const size_t k = (it == m_sortedTimes.begin())
                       ? 0
                       : static_cast<size_t>(std::distance(m_sortedTimes.begin(), it)) - 1;
  return m_sortedEntries[k];
}

CObservation::Ptr RawlogDocument::resolve(const ObsRef& r) const
{
  auto& rl = const_cast<CRawlog&>(rawlog);
  if (r.indexInSF < 0)
  {
    return rl.getAsObservation(r.entry);
  }
  return rl.getAsObservations(r.entry)->getObservationByIndex(static_cast<size_t>(r.indexInSF));
}

CObservation::Ptr RawlogDocument::latestObservation(const std::string& label, double t) const
{
  const auto it = m_obsByLabel.find(label);
  if (it == m_obsByLabel.end() || it->second.empty())
  {
    return {};
  }
  const auto& refs = it->second;
  auto ub = std::upper_bound(
      refs.begin(), refs.end(), t, [](double v, const ObsRef& r) { return v < r.time; });
  if (ub == refs.begin())
  {
    return {};
  }
  return resolve(*std::prev(ub));
}

std::string RawlogDocument::summaryText() const
{
  std::stringstream ss;
  size_t nSF = 0;
  size_t nActs = 0;
  size_t nObs = 0;
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    switch (rawlog.getType(i))
    {
      case CRawlog::TEntryType::etSensoryFrame:
        nSF++;
        break;
      case CRawlog::TEntryType::etActionCollection:
        nActs++;
        break;
      case CRawlog::TEntryType::etObservation:
        nObs++;
        break;
      default:
        break;
    }
  }
  ss << "Entries: " << rawlog.size() << " (" << nActs << " action collections, " << nSF
     << " sensory frames, " << nObs << " observations)\n";
  ss << "Format: "
     << (nObs == rawlog.size() ? "observations only"
         : nObs == 0           ? "sensory frames and actions"
                               : "mixed")
     << "\n";
  if (m_firstTimestamp != INVALID_TIMESTAMP)
  {
    ss << "First timestamp: " << mrpt::system::dateTimeToString(m_firstTimestamp) << " UTC\n";
    ss << "Last timestamp:  " << mrpt::system::dateTimeToString(m_lastTimestamp) << " UTC\n";
    ss << mrpt::format(
        "Duration: %.03f s\n", mrpt::system::timeDifference(m_firstTimestamp, m_lastTimestamp));
  }
  else
  {
    ss << "No valid timestamps: the timeline shows entry indices.\n";
  }
  return ss.str();
}

// ---------------------------------------------------------------------------
// Edit operations
// ---------------------------------------------------------------------------
size_t RawlogDocument::keepRange(size_t first, size_t last, bool remove)
{
  const size_t N = rawlog.size();
  if (N == 0 || first > last || first >= N)
  {
    return 0;
  }
  last = std::min(last, N - 1);
  CRawlog out;
  out.setCommentText(rawlog.getCommentText());
  size_t removed = 0;
  for (size_t i = 0; i < N; i++)
  {
    const bool inRange = i >= first && i <= last;
    if (inRange != remove)
    {
      out.insert(rawlog.getAsGeneric(i));
    }
    else
    {
      removed++;
    }
  }
  rawlog = std::move(out);
  modified = modified || removed != 0;
  rebuildIndex();
  return removed;
}

size_t RawlogDocument::removeObservations(const std::function<bool(const CObservation&)>& pred)
{
  CRawlog out;
  out.setCommentText(rawlog.getCommentText());
  size_t removed = 0;
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    switch (rawlog.getType(i))
    {
      case CRawlog::TEntryType::etObservation:
        if (pred(*rawlog.getAsObservation(i)))
        {
          removed++;
          continue;
        }
        break;
      case CRawlog::TEntryType::etSensoryFrame:
      {
        auto sf = rawlog.getAsObservations(i);
        for (auto it = sf->begin(); it != sf->end();)
        {
          if (*it && pred(**it))
          {
            it = sf->erase(it);
            removed++;
          }
          else
          {
            ++it;
          }
        }
        break;
      }
      default:
        break;
    }
    out.insert(rawlog.getAsGeneric(i));
  }
  rawlog = std::move(out);
  modified = modified || removed != 0;
  rebuildIndex();
  return removed;
}

size_t RawlogDocument::keepHorizontalScansOnly(double maxPitchDeg)
{
  const double maxAng = mrpt::DEG2RAD(maxPitchDeg);
  return removeObservations(
      [maxAng](const CObservation& o)
      {
        const auto* scan = dynamic_cast<const CObservation2DRangeScan*>(&o);
        return scan != nullptr && (std::abs(scan->sensorPose.pitch()) > maxAng ||
                                   std::abs(scan->sensorPose.roll()) > maxAng);
      });
}

size_t RawlogDocument::swapRedBlue()
{
  size_t n = 0;
  const auto swapImg = [&n](mrpt::img::CImage& img)
  {
    // Externally-stored images live in their own files: not modified.
    if (img.isExternallyStored() || img.isEmpty() || !img.isColor())
    {
      return;
    }
    img.swapRB();
    n++;
  };
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        if (auto im = std::dynamic_pointer_cast<CObservationImage>(o); im)
        {
          swapImg(im->image);
        }
        else if (auto st = std::dynamic_pointer_cast<CObservationStereoImages>(o); st)
        {
          swapImg(st->imageLeft);
          if (st->hasImageRight)
          {
            swapImg(st->imageRight);
          }
        }
      });
  modified = modified || n != 0;
  return n;
}

size_t RawlogDocument::renameSensor(const std::string& oldLabel, const std::string& newLabel)
{
  size_t n = 0;
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        if (o->sensorLabel == oldLabel)
        {
          o->sensorLabel = newLabel;
          n++;
        }
      });
  modified = modified || n != 0;
  rebuildIndex();
  return n;
}

size_t RawlogDocument::setSensorPose(
    const std::string& label, const mrpt::poses::CPose3D& pose, bool onlyXYZ)
{
  size_t n = 0;
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        if (o->sensorLabel != label)
        {
          return;
        }
        auto p = pose;
        if (onlyXYZ)
        {
          const auto cur = o->getSensorPose();
          p = mrpt::poses::CPose3D(
              pose.x(), pose.y(), pose.z(), cur.yaw(), cur.pitch(), cur.roll());
        }
        o->setSensorPose(p);
        n++;
      });
  modified = modified || n != 0;
  return n;
}

size_t RawlogDocument::setCameraParams(const std::string& label, const mrpt::img::TCamera& params)
{
  size_t n = 0;
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        if (o->sensorLabel != label)
        {
          return;
        }
        if (auto im = std::dynamic_pointer_cast<CObservationImage>(o); im)
        {
          im->cameraParams = params;
          n++;
        }
      });
  modified = modified || n != 0;
  return n;
}

size_t RawlogDocument::applySensorPosesFromConfig(const std::string& configText)
{
  mrpt::config::CConfigFileMemory cfg(configText);
  std::map<std::string, mrpt::poses::CPose3D> poses;
  std::map<std::string, mrpt::img::TCamera> cams;
  for (const auto& section : cfg.sections())
  {
    const auto label = cfg.read_string(section, "sensorLabel", "");
    if (label.empty())
    {
      continue;
    }
    poses[label] = mrpt::poses::CPose3D(
        cfg.read_double(section, "pose_x", 0), cfg.read_double(section, "pose_y", 0),
        cfg.read_double(section, "pose_z", 0),
        mrpt::DEG2RAD(cfg.read_double(section, "pose_yaw", 0)),
        mrpt::DEG2RAD(cfg.read_double(section, "pose_pitch", 0)),
        mrpt::DEG2RAD(cfg.read_double(section, "pose_roll", 0)));

    mrpt::math::CVectorDouble calib;
    mrpt::math::CVectorDouble distort;
    cfg.read_vector(section, "calib_params", mrpt::math::CVectorDouble(), calib);
    cfg.read_vector(section, "distort_params", mrpt::math::CVectorDouble(), distort);
    if (calib.size() == 4 && distort.size() == 4)
    {
      mrpt::img::TCamera cam;
      cam.fx(calib[0]);
      cam.fy(calib[1]);
      cam.cx(calib[2]);
      cam.cy(calib[3]);
      cam.k1(distort[0]);
      cam.k2(distort[1]);
      cam.p1(distort[2]);
      cam.p2(distort[3]);
      cams[label] = cam;
    }
  }
  if (poses.empty())
  {
    THROW_EXCEPTION("No section with a 'sensorLabel' entry was found.");
  }
  size_t n = 0;
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        if (auto it = poses.find(o->sensorLabel); it != poses.end())
        {
          o->setSensorPose(it->second);
          n++;
        }
        if (auto it = cams.find(o->sensorLabel); it != cams.end())
        {
          if (auto im = std::dynamic_pointer_cast<CObservationImage>(o); im)
          {
            im->cameraParams = it->second;
          }
        }
      });
  modified = modified || n != 0;
  return n;
}

size_t RawlogDocument::setMotionModel(
    const CActionRobotMovement2D::TMotionModelOptions& opts, size_t first, size_t last)
{
  size_t n = 0;
  for (size_t i = first; i < rawlog.size() && i <= last; i++)
  {
    if (rawlog.getType(i) != CRawlog::TEntryType::etActionCollection)
    {
      continue;
    }
    auto acts = rawlog.getAsAction(i);
    for (auto& a : *acts)
    {
      auto mov = std::dynamic_pointer_cast<CActionRobotMovement2D>(a.get_ptr());
      if (!mov || mov->estimationMethod != CActionRobotMovement2D::emOdometry)
      {
        continue;
      }
      mov->computeFromOdometry(mov->rawOdometryIncrementReading, opts);
      n++;
    }
  }
  modified = modified || n != 0;
  return n;
}

std::optional<CActionRobotMovement2D::TMotionModelOptions> RawlogDocument::firstMotionModel() const
{
  for (size_t i = 0; i < rawlog.size(); i++)
  {
    if (rawlog.getType(i) != CRawlog::TEntryType::etActionCollection)
    {
      continue;
    }
    if (auto mov =
            rawlog.getAsAction(i)->getMovementEstimationByType(CActionRobotMovement2D::emOdometry);
        mov)
    {
      return mov->motionModelConfiguration;
    }
  }
  return std::nullopt;
}

std::pair<size_t, double> RawlogDocument::recoverCameraParams3D(const std::string& label)
{
  std::optional<mrpt::img::TCamera> params;
  double err = 0;
  size_t n = 0;
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        auto o3D = std::dynamic_pointer_cast<CObservation3DRangeScan>(o);
        if (!o3D || (!label.empty() && o3D->sensorLabel != label))
        {
          return;
        }
        if (!params)
        {
          o3D->load();
          mrpt::img::TCamera cam;
          err = CObservation3DRangeScan::recoverCameraCalibrationParameters(*o3D, cam);
          params = cam;
        }
        o3D->cameraParams = *params;
        n++;
      });
  modified = modified || n != 0;
  return {n, err};
}

// ---------------------------------------------------------------------------
// Export
// ---------------------------------------------------------------------------
std::vector<std::string> RawlogDocument::exportableClasses()
{
  return {"mrpt::obs::CObservation2DRangeScan",  "mrpt::obs::CObservationBeaconRanges",
          "mrpt::obs::CObservationBearingRange", "mrpt::obs::CObservationGPS",
          "mrpt::obs::CObservationGasSensors",   "mrpt::obs::CObservationIMU",
          "mrpt::obs::CObservationOdometry",     "mrpt::obs::CObservationRange",
          "mrpt::obs::CObservationWirelessPower"};
}

size_t RawlogDocument::exportToText(
    const std::string& className, const std::string& label, const std::string& file) const
{
  mrpt::io::CFileOutputStream f;
  if (!f.open(file))
  {
    THROW_EXCEPTION_FMT("Cannot create file: '%s'", file.c_str());
  }
  std::stringstream header;
  header << "% Exported from '" << mrpt::system::extractFileName(fileName) << "', class "
         << className << (label.empty() ? std::string() : ", label " + label) << "\n";

  const std::string nan = "nan";
  bool headerDone = false;
  size_t n = 0;
  forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        if (o->GetRuntimeClass()->className != className ||
            (!label.empty() && o->sensorLabel != label))
        {
          return;
        }
        std::stringstream ss;
        std::string columns = "% TIMESTAMP";
        ss << fmtTime(o->timestamp);
        if (auto gps = std::dynamic_pointer_cast<CObservationGPS>(o); gps)
        {
          columns += " LAT_DEG LON_DEG ALT_M FIX_QUALITY SATELLITES HDOP";
          const auto* gga = gps->getMsgByClassPtr<gnss::Message_NMEA_GGA>();
          if (!gga)
          {
            return;
          }
          ss << mrpt::format(
              " %.10f %.10f %.4f %u %u %.3f", gga->fields.latitude_degrees,
              gga->fields.longitude_degrees, gga->fields.altitude_meters,
              static_cast<unsigned>(gga->fields.fix_quality),
              static_cast<unsigned>(gga->fields.satellitesUsed), gga->fields.HDOP);
        }
        else if (auto imu = std::dynamic_pointer_cast<CObservationIMU>(o); imu)
        {
          columns += " (one column per TIMUDataIndex value, nan if missing)";
          for (size_t k = 0; k < COUNT_IMU_DATA_FIELDS; k++)
          {
            if (imu->dataIsPresent.at(k))
            {
              ss << mrpt::format(" %.8f", imu->rawMeasurements.at(k));
            }
            else
            {
              ss << " " << nan;
            }
          }
        }
        else if (auto odo = std::dynamic_pointer_cast<CObservationOdometry>(o); odo)
        {
          columns += " X Y PHI_RAD ENC_LEFT ENC_RIGHT VX VY OMEGA";
          ss << mrpt::format(
              " %.6f %.6f %.6f %i %i %.6f %.6f %.6f", odo->odometry.x(), odo->odometry.y(),
              odo->odometry.phi(), static_cast<int>(odo->encoderLeftTicks),
              static_cast<int>(odo->encoderRightTicks), odo->velocityLocal.vx,
              odo->velocityLocal.vy, odo->velocityLocal.omega);
        }
        else if (auto scan = std::dynamic_pointer_cast<CObservation2DRangeScan>(o); scan)
        {
          columns += " APERTURE_RAD N RANGE_0 ... RANGE_N-1 (0 = invalid)";
          ss << mrpt::format(" %.6f %zu", scan->aperture, scan->getScanSize());
          for (size_t k = 0; k < scan->getScanSize(); k++)
          {
            ss << mrpt::format(
                " %.4f", scan->getScanRangeValidity(k) ? scan->getScanRange(k) : 0.0f);
          }
        }
        else if (auto br = std::dynamic_pointer_cast<CObservationBeaconRanges>(o); br)
        {
          columns += " (BEACON_ID RANGE) pairs";
          for (const auto& m : br->sensedData)
          {
            ss << mrpt::format(" %i %.4f", static_cast<int>(m.beaconID), m.sensedDistance);
          }
        }
        else if (auto rb = std::dynamic_pointer_cast<CObservationBearingRange>(o); rb)
        {
          columns += " (ID RANGE YAW_RAD PITCH_RAD) tuples";
          for (const auto& m : rb->sensedData)
          {
            ss << mrpt::format(
                " %i %.4f %.6f %.6f", static_cast<int>(m.landmarkID), m.range, m.yaw, m.pitch);
          }
        }
        else if (auto rg = std::dynamic_pointer_cast<CObservationRange>(o); rg)
        {
          columns += " (SENSOR_ID RANGE) pairs";
          for (const auto& m : rg->sensedData)
          {
            ss << mrpt::format(" %u %.4f", static_cast<unsigned>(m.sensorID), m.sensedDistance);
          }
        }
        else if (auto gas = std::dynamic_pointer_cast<CObservationGasSensors>(o); gas)
        {
          columns += " READINGS_V of each e-nose, in order";
          for (const auto& e : gas->m_readings)
          {
            for (float v : e.readingsVoltage)
            {
              ss << mrpt::format(" %.5f", v);
            }
          }
        }
        else if (auto wp = std::dynamic_pointer_cast<CObservationWirelessPower>(o); wp)
        {
          columns += " POWER";
          ss << mrpt::format(" %.4f", wp->power);
        }
        else
        {
          return;
        }
        if (!headerDone)
        {
          headerDone = true;
          const auto h = header.str() + columns + "\n";
          f.Write(h.data(), h.size());
        }
        const auto line = ss.str() + "\n";
        f.Write(line.data(), line.size());
        n++;
      });
  return n;
}

// ---------------------------------------------------------------------------
// Raw map
// ---------------------------------------------------------------------------
bool RawlogDocument::buildRawMap(
    const std::string& configText,
    size_t first,
    size_t last,
    size_t decimation,
    const progress_callback_t& progress,
    std::shared_ptr<mrpt::maps::CMultiMetricMap>& map,
    std::vector<mrpt::poses::CPose3D>& path)
{
  mrpt::config::CConfigFileMemory cfg(configText);
  mrpt::maps::TSetOfMetricMapInitializers inits;
  inits.loadFromConfigFile(cfg, "map");
  if (inits.size() == 0)
  {
    THROW_EXCEPTION("The configuration defines no map: see its [map] section.");
  }
  map = std::make_shared<mrpt::maps::CMultiMetricMap>();
  map->setListOfMaps(inits);
  path.clear();
  if (rawlog.empty())
  {
    return true;
  }
  last = std::min(last, rawlog.size() - 1);
  decimation = std::max<size_t>(1, decimation);

  mrpt::poses::CPose2D pose;
  std::optional<mrpt::poses::CPose2D> odoRef;
  const auto useOdo = [&](const CObservation& o)
  {
    if (const auto* odo = dynamic_cast<const CObservationOdometry*>(&o); odo)
    {
      if (!odoRef)
      {
        odoRef = odo->odometry;
      }
      pose = odo->odometry - *odoRef;
    }
  };
  size_t nObsEntries = 0;
  for (size_t i = first; i <= last; i++)
  {
    if (progress && (i % 32) == 0 &&
        !progress(static_cast<double>(i - first) / static_cast<double>(last - first + 1)))
    {
      return false;
    }
    switch (rawlog.getType(i))
    {
      case CRawlog::TEntryType::etActionCollection:
        if (auto mov = rawlog.getAsAction(i)->getBestMovementEstimation(); mov)
        {
          pose = pose + mov->poseChange->getMeanVal();
        }
        break;
      case CRawlog::TEntryType::etSensoryFrame:
      {
        auto sf = rawlog.getAsObservations(i);
        for (const auto& o : *sf)
        {
          useOdo(*o);
        }
        if ((nObsEntries++ % decimation) == 0)
        {
          sf->insertObservationsInto(*map, mrpt::poses::CPose3D(pose));
        }
        path.emplace_back(pose);
        break;
      }
      case CRawlog::TEntryType::etObservation:
      {
        auto o = rawlog.getAsObservation(i);
        useOdo(*o);
        if ((nObsEntries++ % decimation) == 0)
        {
          o->insertObservationInto(*map, mrpt::poses::CPose3D(pose));
        }
        path.emplace_back(pose);
        break;
      }
      default:
        break;
    }
  }
  return true;
}
