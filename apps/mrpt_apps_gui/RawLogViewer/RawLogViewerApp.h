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

#include <mrpt/apps_imgui/ObservationPanels.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/imgui/AsyncTask.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/CImGuiLogConsole.h>
#include <mrpt/imgui/CImGuiTimeline.h>
#include <mrpt/imgui/widgets.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "RawlogDocument.h"

class RawLogViewerApp
{
 public:
  RawLogViewerApp();
  ~RawLogViewerApp();

  RawLogViewerApp(const RawLogViewerApp&) = delete;
  RawLogViewerApp& operator=(const RawLogViewerApp&) = delete;

  /** Runs the application. `fileToOpen` may be empty. */
  int run(const std::string& fileToOpen);

  /** Entry to select once the rawlog given to run() is loaded. */
  std::optional<size_t> initialEntry;

 private:
  mrpt::imgui::CImGuiAppShell m_app;
  mrpt::imgui::CImGuiBackgroundScene m_view;
  mrpt::imgui::CImGuiTimeline m_timeline;
  mrpt::imgui::CImGuiFileDialog m_fileDlg;
  mrpt::imgui::CImGuiMessageQueue m_msgs;
  mrpt::imgui::CImGuiAboutDialog m_about;
  mrpt::imgui::CImGuiLogConsole m_console;
  std::unique_ptr<mrpt::config::CConfigFile> m_cfg;

  /** The loaded rawlog (never null). */
  std::shared_ptr<RawlogDocument> m_doc;

  /** Long operations on the document. They return a closure that applies
   * their result in the GUI thread. While running, nothing else touches
   * the document. */
  mrpt::imgui::AsyncTask<std::function<void()>> m_task;
  std::string m_taskTitle;

  // --- Selection ---
  /** Selected top-level entry. */
  std::optional<size_t> m_curEntry;
  /** Selected object: the entry, or an observation within it. */
  mrpt::serialization::CSerializable::Ptr m_selected;
  int m_selectedIndexInSF = -1;

  // --- Panels ---
  bool m_showTree = true;
  bool m_showSelected = true;
  bool m_showInfo = true;
  bool m_showConsole = true;
  bool m_showView = false;
  bool m_showEdit = false;
  bool m_showSensorParams = false;
  bool m_showMotionModel = false;
  bool m_showRawMapPanel = false;
  bool m_showExport = false;

  mrpt::apps::PanelContext m_panelCtx;
  std::unique_ptr<mrpt::apps::ObservationPanel> m_selectedPanel;
  std::unique_ptr<mrpt::apps::ObservationPanel> m_selectedTextPanel;
  const mrpt::rtti::TRuntimeClassId* m_selectedPanelClass = nullptr;

  /** A user-added panel showing the latest observation of a sensor. */
  struct SensorPanel
  {
    std::string label;
    std::unique_ptr<mrpt::apps::ObservationPanel> panel;
    const mrpt::rtti::TRuntimeClassId* panelClass = nullptr;
    bool open = true;
  };
  std::vector<SensorPanel> m_sensorPanels;
  /** Label of the sensor panel to bring to the front in the next frame. */
  std::optional<std::string> m_focusSensorPanel;

  // --- Tree ---
  struct TreeRow
  {
    size_t entry;
    int indexInSF;  // -1: the entry itself
  };
  std::vector<TreeRow> m_treeRows;
  bool m_treeDirty = true;
  std::set<size_t> m_expanded;
  std::string m_treeFilter;
  bool m_scrollTreeToSelection = false;

  // --- 3D view ---
  mrpt::viz::Scene::Ptr m_scene;
  mrpt::viz::CSetOfObjects::Ptr m_glCurrent;
  mrpt::viz::CSetOfLines::Ptr m_glOdoPath;
  mrpt::viz::CSetOfLines::Ptr m_glGpsPath;
  mrpt::viz::CSetOfObjects::Ptr m_glRawMap;
  mrpt::viz::CSetOfObjects::Ptr m_glRobot;
  /** Robot pose at each entry, from odometry (empty if none). */
  std::vector<mrpt::poses::CPose3D> m_odoPoses;
  bool m_showOdoPath = true;
  bool m_showGpsPath = false;
  bool m_showRawMapInScene = true;
  bool m_showCurrentObs = true;
  bool m_followRobot = false;
  int m_lastVizVersion = -1;

  // --- Edit panel ---
  int m_editRange[2] = {0, 0};
  std::set<std::string> m_editClasses;
  std::set<std::string> m_editLabels;
  float m_maxPitchDeg = 2.0f;

  // --- Sensor parameters panel ---
  std::string m_spLabel;
  float m_spPose[6] = {0, 0, 0, 0, 0, 0};  // x y z [m], yaw pitch roll [deg]
  bool m_spOnlyXYZ = false;
  float m_spCam[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // fx fy cx cy k1 k2 p1 p2 focal_m
  std::string m_batchPoseConfig;

  // --- Motion model panel ---
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions m_motionModel;
  float m_motionTestIncr[3] = {1.0f, 0.0f, 0.0f};  // x y [m], phi [deg]
  bool m_motionAll = true;
  int m_motionRange[2] = {0, 0};
  std::vector<double> m_motionSamples[3];

  // --- Raw map panel ---
  std::string m_rawMapConfig;
  int m_rawMapRange[2] = {0, 0};
  int m_rawMapDecimation = 1;
  std::shared_ptr<mrpt::maps::CMultiMetricMap> m_rawMap;
  std::vector<mrpt::poses::CPose3D> m_rawMapPath;

  // --- Export panel ---
  std::string m_exportClass;
  std::string m_exportLabel;

  // --- Popups ---
  bool m_openLoadPartPopup = false;
  std::string m_pendingLoadFile;
  int m_loadPartFirst = 0;
  int m_loadPartCount = 1000;
  bool m_openCommentsPopup = false;
  std::string m_commentsEdit;
  bool m_openRenamePopup = false;
  std::string m_renameFrom;
  std::string m_renameTo;
  bool m_openOpParamPopup = false;
  std::string m_opParamTitle;
  std::string m_opParamLabel;
  double m_opParamValue = 0;
  bool m_opParamNeedsLabel = false;
  std::string m_opParamSensor;
  std::function<void(double, const std::string&)> m_opParamApply;
  bool m_openTipsPopup = false;
  bool m_openUnsavedPopup = false;
  std::function<void()> m_afterUnsavedDecision;
  bool m_closeAfterSave = false;

  unsigned int m_bottomDockId = 0;
  bool m_inSelect = false;
  std::string m_imagesDirOverride;
  std::string m_statusMsg;
  std::vector<std::string> m_recentFiles;

  // Methods: files and tasks
  void loadSettings();
  void saveSettings();
  [[nodiscard]] std::string lastDir() const;
  void addRecentFile(const std::string& file);
  void newDocument();
  void startLoad(const std::string& file, const RawlogDocument::LoadOptions& opts = {});
  void startSave(const std::string& file);
  /** Runs `op` on the document in the background, then `done` with its
   * result message in the GUI thread. */
  void startDocTask(
      const std::string& title,
      std::function<std::string(RawlogDocument&, mrpt::imgui::AsyncTaskContext&)> op);
  void pollTask();
  void onDocumentChanged(bool keepPosition);
  void setImagesDirectory(const std::string& dir);
  void confirmDiscardChanges(std::function<void()> then);
  void handleFileDialogResult(const std::vector<std::string>& files);

  // Selection and timeline
  void selectEntry(size_t index, bool moveTimeline);
  void selectObject(size_t entry, int indexInSF);
  void onTimelineChanged(double t);
  void openSensorPanel(const std::string& label);

  // 3D view
  void rebuildPaths();
  void updateCurrentInScene();

  // GUI
  void renderFrame();
  void renderMainMenu();
  void renderToolbar();
  void renderStatusBar();
  void renderTreePanel();
  void renderSelectedPanel();
  void renderInfoPanel();
  void renderViewPanel();
  void renderEditPanel();
  void renderSensorParamsPanel();
  void renderMotionModelPanel();
  void renderRawMapPanel();
  void renderExportPanel();
  void renderSensorPanels();
  void renderPopups();
  void renderTaskProgress();
  void handleKeyboard();
  void setupDefaultLayout(unsigned int dockspaceId);
  /** Asks for a number (and a sensor label, if `needsLabel`) in a popup,
   * then calls `apply`. */
  void askOpParam(
      const std::string& title,
      const std::string& valueLabel,
      double initialValue,
      bool needsLabel,
      std::function<void(double, const std::string&)> apply);
  void rebuildTreeRows();
  [[nodiscard]] bool busy() const { return m_task.isRunning(); }
};
