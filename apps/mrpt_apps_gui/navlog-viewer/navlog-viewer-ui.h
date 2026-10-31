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

#include <mrpt/img/TColor.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/CImGuiPlotPanel.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/imgui/CImGuiTimeline.h>
#include <mrpt/imgui/widgets.h>
#include <mrpt/nav.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class NavlogViewerApp
{
 public:
  NavlogViewerApp();
  ~NavlogViewerApp() = default;

  /** Runs the GUI; `fileToOpen` may be empty. */
  int run(const std::string& fileToOpen);

  void loadLogfile(const std::string& filName);

 private:
  // =========== UI ==============
  mrpt::imgui::CImGuiAppShell m_app;
  mrpt::imgui::CImGuiBackgroundScene m_bg;
  mrpt::imgui::CImGuiTimeline m_timeline;
  mrpt::imgui::CImGuiFileDialog m_fileDlg;
  mrpt::imgui::CImGuiMessageQueue m_msgs;
  mrpt::imgui::CImGuiAboutDialog m_about;

  mrpt::viz::Scene::Ptr m_scene;

  // Panels:
  bool m_showLogInfo = true;
  bool m_showViewOptions = true;
  bool m_showDetails = true;
  bool m_showManualPick = false;
  bool m_showPTGViews = true;
  bool m_showVelPlot = true;

  /** Robot velocities over the whole log, vs. time. */
  mrpt::imgui::CImGuiPlotPanel m_velPlot;

  // View options:
  bool m_useOdometryCoords = false;
  bool m_globalFrame = true;
  bool m_showDelays = true;
  bool m_drawShape = true;
  bool m_showAllDebugFields = false;
  bool m_showCursorXY = true;
  bool m_clearanceOverPath = false;
  bool m_ortho2DView = true;
  int m_numPastPoses = 10;
  float m_shapeMinDist = 1.0f;
  /** 0: TP-Obstacles only, 1: + final scores, 2: + preliminary scores */
  int m_perPTGPlots = 2;

  // Manual pick:
  bool m_manualPickMode = false;
  int m_manualPickPTGSlider = 0;
  int m_manualPickTrajSlider = 0;
  int m_manualPickPTGMax = 0;
  int m_manualPickTrajMax = 0;
  std::string m_manualPickLabel = "Selection: None";

  // Texts shown in the panels:
  std::string m_txtSelectedPTG;
  std::string m_txtTimeIndex;
  std::string m_txtLogDuration = "???";
  std::string m_mousePosText;
  struct InfoLine
  {
    std::string text;
    mrpt::img::TColorf color;
  };
  std::vector<InfoLine> m_infoLines;

  // ============ app data ===============
  std::string m_openedFileName = "UNNAMED";

  std::vector<mrpt::serialization::CSerializable::Ptr> m_logdata;
  // Retrieved from the first entry in m_logdata when loading
  std::vector<mrpt::nav::CParameterizedTrajectoryGenerator::Ptr> m_logdata_ptg_paths;

  /** The TP-space view of one PTG. */
  struct VizPTG
  {
    mrpt::viz::Scene::Ptr scene;
    std::unique_ptr<mrpt::imgui::CImGuiSceneView> view;
    std::string title;
    bool open = true;
  };

  std::map<unsigned int, VizPTG> m_mywins3D;

  mrpt::system::TTimeStamp m_log_first_tim = INVALID_TIMESTAMP;
  mrpt::system::TTimeStamp m_log_last_tim = INVALID_TIMESTAMP;

  std::optional<size_t> m_lastShownIdx;

  void updateInfoFromLoadedLog();

  /** The log entry at the current timeline position. */
  [[nodiscard]] int currentLogIndex() const;

  void updateVisualization();
  void onMatlabPlots(const std::string& fileName);
  void onSavePTGParams(const std::string& fileName);
  void onSaveScoreMatrix(const std::string& fileName);
  void onOdometryVsLocalizationReport(const std::string& fileName);
  void updateMouseXY();
  void onMatlabExportPaths(const std::string& fileName);
  void onExportSelected(const std::string& fileName);
  void onSaveCurrentObstacles(const std::string& fileName);

  void onManualPickAppendYaml();
  void updateManualPickControls();

  void renderFrame();
  void renderMainMenu();
  void renderPanels();
  void renderPTGViews();
  void handleFileDialogResult(const std::vector<std::string>& files);
  void setupDefaultLayout(unsigned int dockspaceId);
  void addInfoLine(const std::string& s, const mrpt::img::TColorf& c = {1, 1, 1});

  std::vector<mrpt::math::TPose2D> m_pastPoses;

  int m_manualPickPTGIdx = -1;
  int m_manualPickTrajectoryIdx = -1;
  double m_manualPickTimestamp = .0;
};
