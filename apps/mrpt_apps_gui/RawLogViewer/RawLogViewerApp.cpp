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

#include "RawLogViewerApp.h"

#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API
#include <imgui_stdlib.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/imgui/config_paths.h>
#include <mrpt/imgui/style.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/stock_objects.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>

#include "RawLogViewerApp_common.h"

using mrpt::imgui::CImGuiFileDialog;
using mrpt::imgui::tooltip;
using namespace mrpt::obs;
using namespace rlv;

namespace
{
/** Adapts an AsyncTaskContext to the progress callbacks of the operations. */
progress_callback_t progressOf(mrpt::imgui::AsyncTaskContext& ctx)
{
  return [&ctx](double f)
  {
    ctx.setProgress(f);
    return !ctx.cancelRequested();
  };
}
}  // namespace

RawLogViewerApp::RawLogViewerApp() :
    m_doc(std::make_shared<RawlogDocument>()),
    m_batchPoseConfig(DEFAULT_BATCH_POSE_CONFIG),
    m_rawMapConfig(DEFAULT_RAWMAP_CONFIG)
{
  m_about.appName = APP_NAME;
  m_about.description =
      "Viewer and editor of MRPT datasets (rawlog files): browse their entries, play them back "
      "along a timeline with one panel per sensor, and edit them.";
  m_about.extraTabs.emplace_back("Keyboard and mouse", KEYBOARD_HELP);
  m_about.extraTabs.emplace_back("Tips", TIPS);
}

RawLogViewerApp::~RawLogViewerApp()
{
  // The worker may be using the document:
  m_task.cancelAndWait();
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------
void RawLogViewerApp::loadSettings()
{
  const auto dir = mrpt::imgui::userConfigDirectory(APP_NAME);
  m_cfg = std::make_unique<mrpt::config::CConfigFile>(dir.empty() ? "" : dir + "config.ini");
  auto& c = *m_cfg;
  m_showTree = c.read_bool(CFG_SECT, "showTree", m_showTree);
  m_showSelected = c.read_bool(CFG_SECT, "showSelected", m_showSelected);
  m_showInfo = c.read_bool(CFG_SECT, "showInfo", m_showInfo);
  m_showConsole = c.read_bool(CFG_SECT, "showConsole", m_showConsole);
  m_showView = c.read_bool(CFG_SECT, "showView", m_showView);
  m_showOdoPath = c.read_bool(CFG_SECT, "showOdoPath", m_showOdoPath);
  m_showGpsPath = c.read_bool(CFG_SECT, "showGpsPath", m_showGpsPath);
  m_showCurrentObs = c.read_bool(CFG_SECT, "showCurrentObs", m_showCurrentObs);
  m_followRobot = c.read_bool(CFG_SECT, "followRobot", m_followRobot);
  m_timeline.speed = c.read_double(CFG_SECT, "playbackSpeed", m_timeline.speed);
  m_panelCtx.vizParams.load_from_ini_file(c, "ViewOptions");
  mrpt::system::tokenize(c.read_string(CFG_SECT, "recentFiles", ""), "|", m_recentFiles);
}

void RawLogViewerApp::saveSettings()
{
  if (!m_cfg)
  {
    return;
  }
  auto& c = *m_cfg;
  c.write(CFG_SECT, "showTree", m_showTree);
  c.write(CFG_SECT, "showSelected", m_showSelected);
  c.write(CFG_SECT, "showInfo", m_showInfo);
  c.write(CFG_SECT, "showConsole", m_showConsole);
  c.write(CFG_SECT, "showView", m_showView);
  c.write(CFG_SECT, "showOdoPath", m_showOdoPath);
  c.write(CFG_SECT, "showGpsPath", m_showGpsPath);
  c.write(CFG_SECT, "showCurrentObs", m_showCurrentObs);
  c.write(CFG_SECT, "followRobot", m_followRobot);
  c.write(CFG_SECT, "playbackSpeed", m_timeline.speed);
  m_panelCtx.vizParams.save_to_ini_file(c, "ViewOptions");

  std::string recent;
  for (const auto& f : m_recentFiles)
  {
    recent += (recent.empty() ? "" : "|") + f;
  }
  c.write(CFG_SECT, "recentFiles", recent);

  // The sensor panels open, to reopen them with the next rawlog:
  std::string labels;
  for (const auto& p : m_sensorPanels)
  {
    if (p.open)
    {
      labels += (labels.empty() ? "" : "|") + p.label;
    }
  }
  c.write(CFG_SECT, "sensorPanels", labels);
}

std::string RawLogViewerApp::lastDir() const
{
  if (!m_fileDlg.lastDirectory().empty())
  {
    return m_fileDlg.lastDirectory();
  }
  return m_cfg ? m_cfg->read_string(CFG_SECT, "LastDir", "") : std::string();
}

void RawLogViewerApp::addRecentFile(const std::string& file)
{
  m_recentFiles.erase(
      std::remove(m_recentFiles.begin(), m_recentFiles.end(), file), m_recentFiles.end());
  m_recentFiles.insert(m_recentFiles.begin(), file);
  if (m_recentFiles.size() > MAX_RECENT_FILES)
  {
    m_recentFiles.resize(MAX_RECENT_FILES);
  }
  m_cfg->write(CFG_SECT, "LastDir", mrpt::system::extractFileDirectory(file));
}

// ---------------------------------------------------------------------------
// Document and background tasks
// ---------------------------------------------------------------------------
void RawLogViewerApp::newDocument()
{
  m_doc = std::make_shared<RawlogDocument>();
  m_rawMap.reset();
  onDocumentChanged(false);
}

void RawLogViewerApp::confirmDiscardChanges(std::function<void()> then)
{
  if (!m_doc->modified)
  {
    then();
    return;
  }
  m_afterUnsavedDecision = std::move(then);
  m_openUnsavedPopup = true;
}

void RawLogViewerApp::startDocTask(
    const std::string& title,
    std::function<std::string(RawlogDocument&, mrpt::imgui::AsyncTaskContext&)> op)
{
  if (busy())
  {
    m_msgs.warning("Busy", "Another operation is still running.");
    return;
  }
  m_taskTitle = title;
  // The app outlives the task (the destructor waits for it), so the result
  // closure can use `this`:
  m_task.start(
      [this, doc = m_doc, op = std::move(op),
       title](mrpt::imgui::AsyncTaskContext& ctx) -> std::function<void()>
      {
        try
        {
          const auto msg = op(*doc, ctx);
          return [this, msg, title]()
          {
            if (!msg.empty())
            {
              m_console.push(mrpt::system::LVL_INFO, title, msg);
              m_statusMsg = title + ": " + msg;
            }
            onDocumentChanged(true);
          };
        }
        catch (const std::exception& e)
        {
          const std::string err = mrpt::exception_to_str(e);
          return [this, err, title]()
          {
            m_msgs.error(title, err);
            m_console.push(mrpt::system::LVL_ERROR, title, err);
            onDocumentChanged(true);
          };
        }
      });
}

void RawLogViewerApp::startLoad(const std::string& file, const RawlogDocument::LoadOptions& opts)
{
  if (busy())
  {
    m_msgs.warning("Busy", "Another operation is still running.");
    return;
  }
  m_taskTitle = "Loading " + mrpt::system::extractFileName(file);
  m_task.start(
      [this, file, opts](mrpt::imgui::AsyncTaskContext& ctx) -> std::function<void()>
      {
        try
        {
          const mrpt::system::CTicTac tic;
          std::vector<std::string> warnings;
          auto doc = RawlogDocument::load(file, opts, progressOf(ctx), warnings);
          if (!doc)
          {
            return [this]() { m_statusMsg = "Loading cancelled."; };
          }
          auto newDoc = std::make_shared<RawlogDocument>(std::move(*doc));
          const double secs = tic.Tac();
          return [this, newDoc, warnings, file, secs]()
          {
            m_doc = newDoc;
            m_rawMap.reset();
            for (const auto& w : warnings)
            {
              m_msgs.warning("Loading the rawlog", w);
              m_console.push(mrpt::system::LVL_WARN, "load", w);
            }
            addRecentFile(file);
            setImagesDirectory(CRawlog::detectImagesDirectory(file));
            const auto msg = mrpt::format(
                "Loaded %zu entries from '%s' in %.02f s.", m_doc->rawlog.size(), file.c_str(),
                secs);
            m_console.push(mrpt::system::LVL_INFO, "load", msg);
            m_statusMsg = msg;
            onDocumentChanged(false);
            if (initialEntry)
            {
              selectEntry(*initialEntry, true);
              initialEntry.reset();
            }
          };
        }
        catch (const std::exception& e)
        {
          const std::string err = mrpt::exception_to_str(e);
          return [this, err]()
          {
            m_msgs.error("Error loading the rawlog", err);
            m_console.push(mrpt::system::LVL_ERROR, "load", err);
          };
        }
      });
}

void RawLogViewerApp::startSave(const std::string& file)
{
  startDocTask(
      "Saving " + mrpt::system::extractFileName(file),
      [this, file](RawlogDocument& doc, mrpt::imgui::AsyncTaskContext& ctx)
      {
        if (!doc.save(file, progressOf(ctx)))
        {
          return std::string("Cancelled.");
        }
        return mrpt::format("Saved %zu entries to '%s'.", doc.rawlog.size(), file.c_str());
      });
  addRecentFile(file);
}

void RawLogViewerApp::pollTask()
{
  if (auto r = m_task.poll(); r && *r)
  {
    (*r)();
  }
}

void RawLogViewerApp::setImagesDirectory(const std::string& dir)
{
  m_imagesDirOverride = dir;
  mrpt::img::CImage::setImagesPathBase(dir);
  m_console.push(mrpt::system::LVL_INFO, "images", "Directory of external images: " + dir);
  // Panels must reload their images:
  m_selectedPanel.reset();
  for (auto& p : m_sensorPanels)
  {
    p.panel.reset();
  }
  updateCurrentInScene();
}

void RawLogViewerApp::onDocumentChanged(bool keepPosition)
{
  const auto& doc = *m_doc;
  const size_t N = doc.rawlog.size();
  m_treeDirty = true;
  if (!keepPosition)
  {
    m_expanded.clear();
  }

  // Timeline:
  m_timeline.setKeyTimes(doc.sortedEntryTimes());
  if (N != 0)
  {
    m_timeline.setRange(doc.sortedEntryTimes().front(), doc.sortedEntryTimes().back());
  }
  else
  {
    m_timeline.setRange(0, 0);
  }
  m_timeline.clearTracks();
  size_t k = 0;
  const size_t nSensors = doc.sensors().size();
  for (const auto& [label, s] : doc.sensors())
  {
    mrpt::imgui::CImGuiTimeline::Track t;
    t.name = label.empty() ? std::string("(no label)") : label;
    const auto c = mrpt::img::colormap(
        mrpt::img::cmJET,
        nSensors > 1 ? static_cast<float>(k) / static_cast<float>(nSensors - 1) : 0.5f);
    t.color = mrpt::img::TColor(
        static_cast<uint8_t>(c.R * 255), static_cast<uint8_t>(c.G * 255),
        static_cast<uint8_t>(c.B * 255));
    t.times = s.times;
    m_timeline.addTrack(std::move(t));
    k++;
  }
  if (doc.usesTimestamps())
  {
    m_timeline.timeFormatter = [](double t) { return mrpt::format("t = %.03f s", t); };
    m_timeline.stepSize = 0.1;
  }
  else
  {
    m_timeline.timeFormatter = [](double t) { return mrpt::format("entry #%.0f", t); };
    m_timeline.stepSize = 1.0;
  }
  m_panelCtx.referenceTime = doc.firstTimestamp();

  // Ranges of the edit panels:
  const int last = N == 0 ? 0 : static_cast<int>(N - 1);
  for (int* r : {m_editRange, m_rawMapRange, m_motionRange})
  {
    r[0] = keepPosition ? std::min(r[0], last) : 0;
    r[1] = keepPosition ? std::min(r[1], last) : last;
  }

  if (!keepPosition)
  {
    // Reopen the sensor panels of the last session, if the sensors exist:
    std::vector<std::string> labels;
    mrpt::system::tokenize(m_cfg->read_string(CFG_SECT, "sensorPanels", ""), "|", labels);
    for (const auto& l : labels)
    {
      if (doc.sensors().count(l) != 0)
      {
        openSensorPanel(l);
      }
    }
  }

  rebuildPaths();

  // Selection:
  m_selectedPanel.reset();
  if (N == 0)
  {
    m_curEntry.reset();
    m_selected.reset();
    updateCurrentInScene();
  }
  else
  {
    selectEntry(keepPosition && m_curEntry ? std::min(*m_curEntry, N - 1) : 0, true);
  }
}

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------
void RawLogViewerApp::selectEntry(size_t index, bool moveTimeline)
{
  if (index >= m_doc->rawlog.size())
  {
    return;
  }
  m_curEntry = index;
  m_selected = m_doc->rawlog.getAsGeneric(index);
  m_selectedIndexInSF = -1;
  m_scrollTreeToSelection = true;
  if (moveTimeline && !m_inSelect)
  {
    m_inSelect = true;
    m_timeline.seekToKey(m_doc->sortedPosition(index));
    m_inSelect = false;
  }
  updateCurrentInScene();
}

void RawLogViewerApp::selectObject(size_t entry, int indexInSF)
{
  selectEntry(entry, true);
  if (indexInSF < 0 || m_doc->rawlog.getType(entry) != CRawlog::TEntryType::etSensoryFrame)
  {
    return;
  }
  auto sf = m_doc->rawlog.getAsObservations(entry);
  if (static_cast<size_t>(indexInSF) < sf->size())
  {
    m_selected = sf->getObservationByIndex(static_cast<size_t>(indexInSF));
    m_selectedIndexInSF = indexInSF;
  }
}

void RawLogViewerApp::onTimelineChanged(double t)
{
  if (m_inSelect || busy())
  {
    return;
  }
  m_inSelect = true;
  // The timeline keys are the entry times, sorted:
  const auto key = m_timeline.currentKeyIndex();
  const auto e = key ? std::optional<size_t>(m_doc->sortedEntry(*key)) : m_doc->entryAtTime(t);
  if (e && e != m_curEntry)
  {
    selectEntry(*e, false);
  }
  m_inSelect = false;
}

void RawLogViewerApp::openSensorPanel(const std::string& label)
{
  m_focusSensorPanel = label;
  for (auto& p : m_sensorPanels)
  {
    if (p.label == label)
    {
      p.open = true;
      return;
    }
  }
  SensorPanel p;
  p.label = label;
  m_sensorPanels.emplace_back(std::move(p));
}

// ---------------------------------------------------------------------------
// 3D view
// ---------------------------------------------------------------------------
void RawLogViewerApp::rebuildPaths()
{
  const auto& r = m_doc->rawlog;

  // Odometry path: one pose per entry.
  m_odoPoses.clear();
  m_glOdoPath->clear();
  mrpt::poses::CPose2D pose;
  std::optional<mrpt::poses::CPose2D> odoRef;
  bool any = false;
  const auto useOdoObs = [&](const CObservation& o)
  {
    const auto* odo = dynamic_cast<const CObservationOdometry*>(&o);
    if (odo == nullptr)
    {
      return;
    }
    if (!odoRef)
    {
      odoRef = odo->odometry;
    }
    pose = odo->odometry - *odoRef;
    any = true;
  };
  std::vector<mrpt::poses::CPose3D> poses;
  poses.reserve(r.size());
  for (size_t i = 0; i < r.size(); i++)
  {
    switch (r.getType(i))
    {
      case CRawlog::TEntryType::etActionCollection:
        if (auto mov = r.getAsAction(i)->getBestMovementEstimation(); mov)
        {
          pose = pose + mov->poseChange->getMeanVal();
          any = true;
        }
        break;
      case CRawlog::TEntryType::etSensoryFrame:
        for (const auto& o : *r.getAsObservations(i))
        {
          useOdoObs(*o);
        }
        break;
      case CRawlog::TEntryType::etObservation:
        useOdoObs(*r.getAsObservation(i));
        break;
      default:
        break;
    }
    poses.emplace_back(pose);
  }
  if (any)
  {
    m_odoPoses = std::move(poses);
    for (size_t i = 1; i < m_odoPoses.size(); i++)
    {
      const auto& a = m_odoPoses[i - 1];
      const auto& b = m_odoPoses[i];
      if (a.translation() != b.translation())
      {
        m_glOdoPath->appendLine(a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
      }
    }
  }

  // GPS paths, one per sensor, in ENU coordinates relative to the first fix:
  m_glGpsPath->clear();
  std::optional<mrpt::topography::TGeodeticCoords> ref;
  std::map<std::string, mrpt::math::TPoint3D> prevByLabel;
  m_doc->forEachObservation(
      [&](size_t, const CObservation::Ptr& o)
      {
        auto gps = std::dynamic_pointer_cast<CObservationGPS>(o);
        if (!gps)
        {
          return;
        }
        const auto* gga = gps->getMsgByClassPtr<gnss::Message_NMEA_GGA>();
        if (gga == nullptr || gga->fields.fix_quality == 0)
        {
          return;
        }
        const mrpt::topography::TGeodeticCoords c(
            gga->fields.latitude_degrees, gga->fields.longitude_degrees,
            gga->fields.altitude_meters);
        if (!ref)
        {
          ref = c;
        }
        mrpt::math::TPoint3D p;
        mrpt::topography::geodeticToENU_WGS84(c, p, *ref);
        if (auto it = prevByLabel.find(gps->sensorLabel); it != prevByLabel.end())
        {
          const auto& prev = it->second;
          m_glGpsPath->appendLine(prev.x, prev.y, prev.z, p.x, p.y, p.z);
        }
        prevByLabel[gps->sensorLabel] = p;
      });
}

void RawLogViewerApp::updateCurrentInScene()
{
  if (!m_glCurrent)
  {
    return;
  }
  m_lastVizVersion = m_panelCtx.vizParamsVersion;
  m_glCurrent->clear();
  m_glOdoPath->setVisibility(m_showOdoPath);
  m_glGpsPath->setVisibility(m_showGpsPath);
  m_glRawMap->setVisibility(m_showRawMapInScene);
  if (!m_curEntry || *m_curEntry >= m_doc->rawlog.size())
  {
    m_glRobot->setVisibility(false);
    return;
  }
  const size_t i = *m_curEntry;
  if (m_showCurrentObs)
  {
    try
    {
      auto& r = m_doc->rawlog;
      if (r.getType(i) == CRawlog::TEntryType::etSensoryFrame)
      {
        obs_to_viz(*r.getAsObservations(i), m_panelCtx.vizParams, *m_glCurrent);
      }
      else if (r.getType(i) == CRawlog::TEntryType::etObservation)
      {
        obs_to_viz(r.getAsObservation(i), m_panelCtx.vizParams, *m_glCurrent);
      }
    }
    catch (const std::exception& e)
    {
      m_statusMsg = "Error showing the entry in 3D: " + mrpt::exception_to_str(e);
    }
  }
  const auto pose = i < m_odoPoses.size() ? m_odoPoses[i] : mrpt::poses::CPose3D();
  m_glCurrent->setPose(pose);
  m_glRobot->setPose(pose);
  m_glRobot->setVisibility(true);
  if (m_followRobot)
  {
    m_view.view.cameraController.setCameraPointing(
        static_cast<float>(pose.x()), static_cast<float>(pose.y()), static_cast<float>(pose.z()));
  }
}

// ---------------------------------------------------------------------------
// File dialogs
// ---------------------------------------------------------------------------
void RawLogViewerApp::handleFileDialogResult(const std::vector<std::string>& files)
{
  if (files.empty())
  {
    return;  // cancelled
  }
  const auto& f = files.front();
  const auto& tag = m_fileDlg.tag;
  if (tag == TAG_OPEN)
  {
    startLoad(f);
  }
  else if (tag == TAG_LOAD_PART)
  {
    m_pendingLoadFile = f;
    m_openLoadPartPopup = true;
  }
  else if (tag == TAG_SAVE)
  {
    startSave(f);
  }
  else if (tag == TAG_IMAGES_DIR)
  {
    setImagesDirectory(f);
  }
  else if (tag == TAG_EXPORT_TXT)
  {
    const auto cls = m_exportClass;
    const auto label = m_exportLabel;
    startDocTask(
        "Export to text",
        [cls, label, f](RawlogDocument& doc, mrpt::imgui::AsyncTaskContext&)
        {
          const auto n = doc.exportToText(cls, label, f);
          return mrpt::format("Wrote %zu lines to '%s'.", n, f.c_str());
        });
  }
  else if (tag == TAG_RAWMAP_TXT && m_rawMap)
  {
    // The map class decides the extension of each file it writes:
    const auto dot = f.find_last_of('.');
    const auto slash = f.find_last_of("/\\");
    const bool hasExt = dot != std::string::npos && (slash == std::string::npos || dot > slash);
    m_rawMap->saveMetricMapRepresentationToFile(hasExt ? f.substr(0, dot) : f);
    m_console.push(mrpt::system::LVL_INFO, "raw map", "Map saved with prefix: " + f);
  }
  else if (tag == TAG_RAWMAP_3D && m_rawMap)
  {
    mrpt::viz::Scene scene;
    scene.insert(mrpt::viz::CGridPlaneXY::Create());
    scene.insert(m_rawMap->getVisualization());
    scene.saveToFile(f);
    m_console.push(mrpt::system::LVL_INFO, "raw map", "3D scene saved to: " + f);
  }
  else if (tag == TAG_RAWMAP_PATH)
  {
    std::ofstream out(f);
    out << "% ENTRY_INDEX X Y Z YAW PITCH ROLL\n";
    for (size_t i = 0; i < m_rawMapPath.size(); i++)
    {
      const auto& p = m_rawMapPath[i];
      out << mrpt::format(
          "%zu %.6f %.6f %.6f %.6f %.6f %.6f\n", i, p.x(), p.y(), p.z(), p.yaw(), p.pitch(),
          p.roll());
    }
    m_console.push(mrpt::system::LVL_INFO, "raw map", "Path saved to: " + f);
  }
  else if (tag == TAG_BATCH_LOAD)
  {
    std::ifstream in(f);
    m_batchPoseConfig.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  else if (tag == TAG_BATCH_SAVE)
  {
    std::ofstream out(f);
    out << m_batchPoseConfig;
  }
}

// ---------------------------------------------------------------------------
// GUI: menus, toolbar, status bar
// ---------------------------------------------------------------------------
void RawLogViewerApp::askOpParam(
    const std::string& title,
    const std::string& valueLabel,
    double initialValue,
    bool needsLabel,
    std::function<void(double, const std::string&)> apply)
{
  m_opParamTitle = title;
  m_opParamLabel = valueLabel;
  m_opParamValue = initialValue;
  m_opParamNeedsLabel = needsLabel;
  if (needsLabel && !m_doc->sensors().empty() && m_doc->sensors().count(m_opParamSensor) == 0)
  {
    m_opParamSensor = m_doc->sensors().begin()->first;
  }
  m_opParamApply = std::move(apply);
  m_openOpParamPopup = true;
}

void RawLogViewerApp::renderMainMenu()
{
  using mrpt::apps::rawlog_ops::progress_callback_t;
  const bool hasDoc = !m_doc->rawlog.empty();
  const bool idle = !busy();

  if (ImGui::BeginMenu("File"))
  {
    if (ImGui::MenuItem(ICON_MS_ADD " New (empty)", nullptr, false, idle))
    {
      confirmDiscardChanges([this]() { newDocument(); });
    }
    if (ImGui::MenuItem(ICON_MS_FOLDER_OPEN " Open...", "Ctrl+O", false, idle))
    {
      confirmDiscardChanges(
          [this]()
          {
            m_fileDlg.tag = TAG_OPEN;
            m_fileDlg.open(CImGuiFileDialog::Mode::Open, "Open rawlog", RAWLOG_FILTERS, lastDir());
          });
    }
    if (ImGui::BeginMenu(ICON_MS_HISTORY " Open recent", idle && !m_recentFiles.empty()))
    {
      for (const auto& f : m_recentFiles)
      {
        if (ImGui::MenuItem(f.c_str()))
        {
          confirmDiscardChanges([this, f]() { startLoad(f); });
        }
      }
      ImGui::EndMenu();
    }
    if (ImGui::MenuItem(ICON_MS_FILTER_LIST " Load a part only...", nullptr, false, idle))
    {
      confirmDiscardChanges(
          [this]()
          {
            m_fileDlg.tag = TAG_LOAD_PART;
            m_fileDlg.open(
                CImGuiFileDialog::Mode::Open, "Load a part of a rawlog", RAWLOG_FILTERS, lastDir());
          });
    }
    if (ImGui::MenuItem(
            ICON_MS_RESTART_ALT " Revert", nullptr, false, idle && !m_doc->fileName.empty()))
    {
      confirmDiscardChanges([this]() { startLoad(m_doc->fileName); });
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_SAVE_AS " Save as...", "Ctrl+S", false, idle && hasDoc))
    {
      m_fileDlg.tag = TAG_SAVE;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Save rawlog as", RAWLOG_FILTERS,
          m_doc->fileName.empty() ? lastDir() : m_doc->fileName);
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_CLOSE " Quit"))
    {
      confirmDiscardChanges(
          [this]()
          {
            m_doc->modified = false;
            m_app.requestClose();
          });
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Edit", idle && hasDoc))
  {
    ImGui::MenuItem(ICON_MS_EDIT " Edit rawlog...", nullptr, &m_showEdit);
    if (ImGui::MenuItem(ICON_MS_COMMENT " Comments..."))
    {
      m_commentsEdit = m_doc->rawlog.getCommentText();
      m_openCommentsPopup = true;
    }
    if (ImGui::MenuItem(ICON_MS_LABEL " Rename a sensor..."))
    {
      m_renameFrom = m_doc->sensors().empty() ? "" : m_doc->sensors().begin()->first;
      m_renameTo = m_renameFrom;
      m_openRenamePopup = true;
    }
    ImGui::MenuItem(
        ICON_MS_SENSORS " Sensor poses and camera parameters...", nullptr, &m_showSensorParams);
    ImGui::Separator();
    // Decimation and format conversions, shared with rawlog-edit:
    namespace ops = mrpt::apps::rawlog_ops;
    const auto runOp = [this](const std::string& title, auto fn)
    {
      startDocTask(
          title,
          [fn](RawlogDocument& doc, mrpt::imgui::AsyncTaskContext& ctx)
          {
            auto msg = fn(doc.rawlog, progressOf(ctx));
            if (ctx.cancelRequested())
            {
              return std::string("Cancelled.");
            }
            doc.modified = true;
            doc.rebuildIndex();
            return msg;
          });
    };
    if (ImGui::MenuItem("Decimate..."))
    {
      askOpParam(
          "Decimate", "Keep one sensory frame out of N", 2, false,
          [runOp](double v, const std::string&)
          {
            const auto n = static_cast<size_t>(std::max(1.0, v));
            runOp(
                "Decimate", [n](CRawlog& r, const progress_callback_t& p)
                { return mrpt::format("Sensory frames kept: %zu", ops::decimate(r, n, p)); });
          });
    }
    if (ImGui::MenuItem("Loss-less decimation..."))
    {
      askOpParam(
          "Loss-less decimation", "Merge each N sensory frames", 2, false,
          [runOp](double v, const std::string&)
          {
            const auto n = static_cast<size_t>(std::max(1.0, v));
            runOp(
                "Loss-less decimation",
                [n](CRawlog& r, const progress_callback_t& p) {
                  return mrpt::format(
                      "Sensory frames in the output: %zu", ops::losslessDecimate(r, n, p));
                });
          });
    }
    for (const bool onePerLabel : {false, true})
    {
      if (ImGui::MenuItem(onePerLabel ? "Compact (one observation per sensor)" : "Compact"))
      {
        runOp(
            "Compact",
            [onePerLabel](CRawlog& r, const progress_callback_t& p)
            {
              const auto st = ops::compact(r, onePerLabel, p);
              return mrpt::format(
                  "Actions removed: %zu, sensory frames removed: %zu", st.actionsRemoved,
                  st.sensoryFramesRemoved);
            });
      }
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Convert to observations-only"))
    {
      runOp(
          "Convert to observations-only",
          [](CRawlog& r, const progress_callback_t& p)
          {
            ops::toObservationsOnly(r, p);
            return std::string("Done.");
          });
    }
    if (ImGui::MenuItem("Convert to sensory frames..."))
    {
      askOpParam(
          "Convert to sensory frames", "Maximum length of each frame [s]", 0.1, false,
          [runOp](double v, const std::string&)
          {
            runOp(
                "Convert to sensory frames",
                [v](CRawlog& r, const progress_callback_t& p)
                {
                  ops::toSensoryFrames(r, v, false, p);
                  return std::string("Done.");
                });
          });
    }
    if (ImGui::MenuItem("Sort by timestamp"))
    {
      runOp(
          "Sort by timestamp",
          [](CRawlog& r, const progress_callback_t&)
          {
            ops::sortByTimestamp(r);
            return std::string("Done.");
          });
    }
    if (ImGui::MenuItem("Shift timestamps of a sensor..."))
    {
      askOpParam(
          "Shift timestamps", "Time shift [s]", 0.0, true,
          [runOp](double v, const std::string& label)
          {
            runOp(
                "Shift timestamps",
                [v, label](CRawlog& r, const progress_callback_t&) {
                  return mrpt::format(
                      "Observations modified: %zu", ops::shiftTimestamps(r, {label}, v));
                });
          });
    }
    if (ImGui::MenuItem("Regenerate timestamps in sensory frames"))
    {
      runOp(
          "Regenerate timestamps",
          [](CRawlog& r, const progress_callback_t&)
          {
            return mrpt::format(
                "Timestamps regenerated: %zu", ops::regenerateTimestampsInSensoryFrames(r));
          });
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Sensors", idle && hasDoc))
  {
    if (ImGui::BeginMenu(ICON_MS_ADD_BOX " Add sensor panel"))
    {
      for (const auto& [label, s] : m_doc->sensors())
      {
        std::string classes;
        for (const auto& c : s.classNames)
        {
          classes += (classes.empty() ? "" : ", ") + c;
        }
        if (ImGui::MenuItem(
                (label.empty() ? std::string("(no label)") : label).c_str(), classes.c_str()))
        {
          openSensorPanel(label);
        }
      }
      ImGui::EndMenu();
    }
    ImGui::Separator();
    ImGui::MenuItem(ICON_MS_NAVIGATION " Odometry motion model...", nullptr, &m_showMotionModel);
    if (ImGui::MenuItem(ICON_MS_CAMERA " Recover the camera parameters of 3D cameras"))
    {
      startDocTask(
          "Recover camera parameters",
          [](RawlogDocument& doc, mrpt::imgui::AsyncTaskContext&)
          {
            const auto [n, err] = doc.recoverCameraParams3D("");
            if (n == 0)
            {
              return std::string("No 3D range scans found.");
            }
            return mrpt::format(
                "Calibrated with the first 3D observation (average reprojection error: "
                "%.04f px) and applied to %zu observations.",
                err, n);
          });
    }
    if (ImGui::MenuItem(ICON_MS_SATELLITE_ALT " Show the GPS path in 3D", nullptr, &m_showGpsPath))
    {
      updateCurrentInScene();
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Tools", idle && hasDoc))
  {
    ImGui::MenuItem(ICON_MS_MAP " Raw map from odometry...", nullptr, &m_showRawMapPanel);
    ImGui::MenuItem(ICON_MS_FILE_EXPORT " Export observations to text...", nullptr, &m_showExport);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("View"))
  {
    ImGui::MenuItem(PANEL_TREE, nullptr, &m_showTree);
    ImGui::MenuItem(PANEL_SELECTED, nullptr, &m_showSelected);
    ImGui::MenuItem(PANEL_INFO, nullptr, &m_showInfo);
    ImGui::MenuItem(PANEL_VIEW, nullptr, &m_showView);
    ImGui::MenuItem(PANEL_CONSOLE, nullptr, &m_showConsole);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Help"))
  {
    if (ImGui::MenuItem(ICON_MS_TIPS_AND_UPDATES " Tips..."))
    {
      m_openTipsPopup = true;
    }
    if (ImGui::MenuItem(ICON_MS_INFO " About", "F1"))
    {
      m_about.open();
    }
    ImGui::EndMenu();
  }
}

void RawLogViewerApp::renderToolbar()
{
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar;
  const float h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2;
  if (ImGui::BeginViewportSideBar("##toolbar", ImGui::GetMainViewport(), ImGuiDir_Up, h, flags))
  {
    const bool idle = !busy();
    const bool hasDoc = !m_doc->rawlog.empty();
    const auto button = [](const char* label, const char* tip, bool enabled = true)
    {
      ImGui::BeginDisabled(!enabled);
      const bool r = ImGui::Button(label);
      ImGui::EndDisabled();
      tooltip(tip);
      ImGui::SameLine();
      return r;
    };
    const auto toggle = [](const char* label, bool* v, const char* tip)
    {
      const bool active = *v;
      if (active)
      {
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
      }
      const bool clicked = ImGui::Button(label);
      if (active)
      {
        ImGui::PopStyleColor();
      }
      if (clicked)
      {
        *v = !*v;
      }
      tooltip(tip);
      ImGui::SameLine();
      return clicked;
    };
    const auto sep = []()
    {
      ImGui::TextDisabled("|");
      ImGui::SameLine();
    };

    if (button(ICON_MS_FOLDER_OPEN " Open", "Open a rawlog (Ctrl+O)", idle))
    {
      confirmDiscardChanges(
          [this]()
          {
            m_fileDlg.tag = TAG_OPEN;
            m_fileDlg.open(CImGuiFileDialog::Mode::Open, "Open rawlog", RAWLOG_FILTERS, lastDir());
          });
    }
    if (button(ICON_MS_SAVE_AS " Save as", "Save the rawlog as... (Ctrl+S)", idle && hasDoc))
    {
      m_fileDlg.tag = TAG_SAVE;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Save rawlog as", RAWLOG_FILTERS,
          m_doc->fileName.empty() ? lastDir() : m_doc->fileName);
    }
    sep();
    toggle(ICON_MS_EDIT " Edit", &m_showEdit, "Edit the rawlog: ranges, sensors, classes...");
    toggle(
        ICON_MS_MAP " Raw map", &m_showRawMapPanel, "Build a map from odometry and observations");
    toggle(ICON_MS_NAVIGATION " Motion model", &m_showMotionModel, "Odometry motion model");
    sep();
    if (toggle(ICON_MS_ROUTE " Odometry", &m_showOdoPath, "Show the odometry path in 3D"))
    {
      updateCurrentInScene();
    }
    if (toggle(ICON_MS_MY_LOCATION " Follow", &m_followRobot, "The camera follows the robot"))
    {
      updateCurrentInScene();
    }
    sep();
    if (button(ICON_MS_INFO, "About (F1)"))
    {
      m_about.open();
    }
  }
  ImGui::End();
}

void RawLogViewerApp::renderStatusBar()
{
  if (mrpt::imgui::beginStatusBar())
  {
    if (busy())
    {
      ImGui::Text(ICON_MS_SCHEDULE " %s... %.0f%%", m_taskTitle.c_str(), 100.0 * m_task.progress());
    }
    else
    {
      ImGui::TextUnformatted(m_statusMsg.c_str());
    }
    if (m_curEntry && !m_doc->rawlog.empty())
    {
      ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.65f);
      ImGui::Text(
          "Entry %zu / %zu%s", *m_curEntry, m_doc->rawlog.size(),
          m_doc->modified ? "  | " ICON_MS_EDIT " modified" : "");
    }
  }
  mrpt::imgui::endStatusBar();
}

void RawLogViewerApp::renderTaskProgress()
{
  constexpr const char* id = "Working...###TaskProgress";
  if (busy() && !ImGui::IsPopupOpen(id))
  {
    ImGui::OpenPopup(id);
  }
  ImGui::SetNextWindowSize(ImVec2(440, 0));
  if (ImGui::BeginPopupModal(
          id, nullptr, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize))
  {
    if (!busy())
    {
      ImGui::CloseCurrentPopup();
    }
    else
    {
      ImGui::TextUnformatted(m_taskTitle.c_str());
      ImGui::ProgressBar(static_cast<float>(m_task.progress()), ImVec2(-1, 0));
      if (const auto msg = m_task.progressMessage(); !msg.empty())
      {
        ImGui::TextUnformatted(msg.c_str());
      }
      if (ImGui::Button(ICON_MS_CLOSE " Cancel"))
      {
        m_task.cancelAndWait();
        m_statusMsg = m_taskTitle + ": cancelled.";
        m_console.push(mrpt::system::LVL_WARN, "task", m_statusMsg);
        onDocumentChanged(true);
        ImGui::CloseCurrentPopup();
      }
    }
    ImGui::EndPopup();
  }
}

void RawLogViewerApp::renderPopups()
{
  // --- Load a part ---
  if (m_openLoadPartPopup)
  {
    ImGui::OpenPopup("Load a part of the rawlog");
    m_openLoadPartPopup = false;
  }
  if (ImGui::BeginPopupModal(
          "Load a part of the rawlog", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted(m_pendingLoadFile.c_str());
    ImGui::InputInt("First entry", &m_loadPartFirst);
    ImGui::InputInt("Number of entries", &m_loadPartCount);
    m_loadPartFirst = std::max(0, m_loadPartFirst);
    m_loadPartCount = std::max(1, m_loadPartCount);
    if (ImGui::Button("Load"))
    {
      RawlogDocument::LoadOptions opts;
      opts.firstEntry = static_cast<size_t>(m_loadPartFirst);
      opts.maxEntries = static_cast<size_t>(m_loadPartCount);
      startLoad(m_pendingLoadFile, opts);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Comments ---
  if (m_openCommentsPopup)
  {
    ImGui::OpenPopup("Rawlog comments");
    m_openCommentsPopup = false;
  }
  ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);
  if (ImGui::BeginPopupModal("Rawlog comments"))
  {
    ImGui::InputTextMultiline(
        "##comments", &m_commentsEdit, ImVec2(-1, -ImGui::GetFrameHeightWithSpacing() * 1.2f));
    if (ImGui::Button("Apply"))
    {
      if (m_commentsEdit != m_doc->rawlog.getCommentText())
      {
        m_doc->rawlog.setCommentText(m_commentsEdit);
        m_doc->modified = true;
      }
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Rename sensor ---
  if (m_openRenamePopup)
  {
    ImGui::OpenPopup("Rename a sensor");
    m_openRenamePopup = false;
  }
  if (ImGui::BeginPopupModal("Rename a sensor", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    if (ImGui::BeginCombo("Sensor", m_renameFrom.c_str()))
    {
      for (const auto& [label, s] : m_doc->sensors())
      {
        if (ImGui::Selectable(label.c_str(), label == m_renameFrom))
        {
          m_renameFrom = label;
          m_renameTo = label;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::InputText("New label", &m_renameTo);
    ImGui::BeginDisabled(m_renameTo == m_renameFrom);
    if (ImGui::Button("Rename"))
    {
      const auto from = m_renameFrom;
      const auto to = m_renameTo;
      for (auto& p : m_sensorPanels)
      {
        if (p.label == from)
        {
          p.label = to;
        }
      }
      startDocTask(
          "Rename sensor",
          [from, to](RawlogDocument& doc, mrpt::imgui::AsyncTaskContext&)
          {
            return mrpt::format(
                "'%s' -> '%s': %zu observations", from.c_str(), to.c_str(),
                doc.renameSensor(from, to));
          });
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Operation parameter ---
  if (m_openOpParamPopup)
  {
    ImGui::OpenPopup("Parameters###OpParam");
    m_openOpParamPopup = false;
  }
  if (ImGui::BeginPopupModal("Parameters###OpParam", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted(m_opParamTitle.c_str());
    ImGui::Separator();
    if (m_opParamNeedsLabel && ImGui::BeginCombo("Sensor", m_opParamSensor.c_str()))
    {
      for (const auto& [label, s] : m_doc->sensors())
      {
        if (ImGui::Selectable(label.c_str(), label == m_opParamSensor))
        {
          m_opParamSensor = label;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::InputDouble(m_opParamLabel.c_str(), &m_opParamValue);
    if (ImGui::Button("OK"))
    {
      if (m_opParamApply)
      {
        m_opParamApply(m_opParamValue, m_opParamSensor);
      }
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Unsaved changes ---
  if (m_openUnsavedPopup)
  {
    ImGui::OpenPopup("Unsaved changes");
    m_openUnsavedPopup = false;
  }
  if (ImGui::BeginPopupModal("Unsaved changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted(
        "The rawlog has unsaved changes, which will be lost.\nUse File > Save as... to keep "
        "them.");
    if (ImGui::Button("Discard the changes"))
    {
      ImGui::CloseCurrentPopup();
      if (m_afterUnsavedDecision)
      {
        auto f = std::move(m_afterUnsavedDecision);
        m_afterUnsavedDecision = nullptr;
        f();
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      m_afterUnsavedDecision = nullptr;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Tips ---
  if (m_openTipsPopup)
  {
    ImGui::OpenPopup(ICON_MS_TIPS_AND_UPDATES " Tips");
    m_openTipsPopup = false;
  }
  ImGui::SetNextWindowSize(ImVec2(560, 0), ImGuiCond_Appearing);
  if (ImGui::BeginPopupModal(ICON_MS_TIPS_AND_UPDATES " Tips", nullptr))
  {
    ImGui::TextWrapped("%s", TIPS);
    if (ImGui::Button("Close"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void RawLogViewerApp::handleKeyboard()
{
  const auto& io = ImGui::GetIO();
  if (io.WantTextInput)
  {
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F1))
  {
    m_about.open();
  }
  if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O) && !busy())
  {
    confirmDiscardChanges(
        [this]()
        {
          m_fileDlg.tag = TAG_OPEN;
          m_fileDlg.open(CImGuiFileDialog::Mode::Open, "Open rawlog", RAWLOG_FILTERS, lastDir());
        });
  }
  if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S) && !busy() && !m_doc->rawlog.empty())
  {
    m_fileDlg.tag = TAG_SAVE;
    m_fileDlg.open(
        CImGuiFileDialog::Mode::Save, "Save rawlog as", RAWLOG_FILTERS,
        m_doc->fileName.empty() ? lastDir() : m_doc->fileName);
  }
}

void RawLogViewerApp::setupDefaultLayout(unsigned int dockspaceId)
{
  ImGuiID left = 0;
  ImGuiID right = 0;
  ImGuiID bottom = 0;
  ImGuiID center = dockspaceId;
  left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.24f, nullptr, &center);
  right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.30f, nullptr, &center);
  bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.35f, nullptr, &center);
  const ImGuiID leftBottom =
      ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.40f, nullptr, &left);

  ImGui::DockBuilderDockWindow("Rawlog", left);
  ImGui::DockBuilderDockWindow("Info", leftBottom);
  ImGui::DockBuilderDockWindow("Selected", right);
  for (const char* w : {"View", "Edit", "SensorParams", "MotionModel", "RawMap", "Export"})
  {
    ImGui::DockBuilderDockWindow(w, right);
  }
  ImGui::DockBuilderDockWindow("Console", bottom);
  m_bottomDockId = bottom;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void RawLogViewerApp::renderFrame()
{
  pollTask();
  handleKeyboard();

  if (auto r = m_fileDlg.poll(); r)
  {
    handleFileDialogResult(*r);
  }

  if (!busy() && m_panelCtx.vizParamsVersion != m_lastVizVersion)
  {
    updateCurrentInScene();
  }

  // Window title:
  {
    static std::string lastTitle;
    std::string title = APP_NAME;
    if (!m_doc->fileName.empty())
    {
      title = mrpt::system::extractFileName(m_doc->fileName) + (m_doc->modified ? " *" : "") +
              " - " + title;
    }
    if (title != lastTitle)
    {
      lastTitle = title;
      m_app.setWindowTitle(title);
    }
  }

  // Docked panels:
  renderTreePanel();
  renderSelectedPanel();
  renderInfoPanel();
  renderViewPanel();
  renderEditPanel();
  renderSensorParamsPanel();
  renderMotionModelPanel();
  renderRawMapPanel();
  renderExportPanel();
  // The console first: new sensor panels are docked next to it.
  if (m_showConsole)
  {
    if (ImGui::Begin(PANEL_CONSOLE, &m_showConsole))
    {
      m_console.render();
    }
    ImGui::End();
  }
  if (m_bottomDockId == 0)
  {
    // Layout loaded from file:
    if (auto* w = ImGui::FindWindowByName(PANEL_CONSOLE); w != nullptr && w->DockId != 0)
    {
      m_bottomDockId = w->DockId;
    }
  }
  renderSensorPanels();

  m_view.render(m_app);
  renderPopups();
  renderTaskProgress();
  m_msgs.render();
  m_about.render();
}

int RawLogViewerApp::run(const std::string& fileToOpen)
{
  mrpt::imgui::CImGuiAppShell::Options opts;
  opts.width = 1440;
  opts.height = 900;
  if (!m_app.init(APP_NAME, opts))
  {
    std::cerr << "Could not create the application window.\n";
    return 1;
  }
  loadSettings();

  // 3D scene:
  m_scene = mrpt::viz::Scene::Create();
  m_scene->insert(mrpt::viz::CGridPlaneXY::Create(-50, 50, -50, 50, 0, 5));
  m_glOdoPath = mrpt::viz::CSetOfLines::Create();
  m_glOdoPath->setColor_u8(0xff, 0x40, 0x40);
  m_glOdoPath->setLineWidth(2);
  m_scene->insert(m_glOdoPath);
  m_glGpsPath = mrpt::viz::CSetOfLines::Create();
  m_glGpsPath->setColor_u8(0x40, 0xff, 0x40);
  m_glGpsPath->setLineWidth(2);
  m_scene->insert(m_glGpsPath);
  m_glRawMap = mrpt::viz::CSetOfObjects::Create();
  m_scene->insert(m_glRawMap);
  m_glCurrent = mrpt::viz::CSetOfObjects::Create();
  m_scene->insert(m_glCurrent);
  m_glRobot = mrpt::viz::stock_objects::RobotPioneer();
  m_scene->insert(m_glRobot);
  m_view.view.setScene(m_scene);
  m_view.view.cameraController.setZoomDistance(30.0f);
  m_view.view.cameraController.setElevationDegrees(45.0f);

  m_timeline.onTimeChanged = [this](double t) { onTimelineChanged(t); };

  m_app.mainMenuBar = [this]() { renderMainMenu(); };
  m_app.beforeDockspace = [this]()
  {
    renderToolbar();
    renderStatusBar();
    m_timeline.renderAsBottomBar();
  };
  m_app.setupDefaultLayout = [this](unsigned int id) { setupDefaultLayout(id); };
  m_app.onFilesDropped = [this](const std::vector<std::string>& files)
  {
    if (!files.empty() && !busy())
    {
      const auto& f = files.front();
      confirmDiscardChanges([this, f]() { startLoad(f); });
    }
  };
  m_app.onCloseRequest = [this]()
  {
    if (!m_doc->modified)
    {
      return true;
    }
    confirmDiscardChanges(
        [this]()
        {
          m_doc->modified = false;
          m_app.requestClose();
        });
    return false;
  };

  newDocument();
  if (!fileToOpen.empty())
  {
    startLoad(fileToOpen);
  }

  m_app.run([this]() { renderFrame(); });

  m_task.cancelAndWait();
  saveSettings();
  return 0;
}
