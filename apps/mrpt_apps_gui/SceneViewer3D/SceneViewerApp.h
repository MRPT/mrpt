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

#include <mrpt/config/CConfigFile.h>
#include <mrpt/imgui/AsyncTask.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/CImGuiLogConsole.h>
#include <mrpt/imgui/widgets.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>

#include <ctime>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/** One keyframe of a camera path ("travelling"). */
struct CameraKeyframe
{
  double t = 0;               //!< seconds since the path start
  float x = 0, y = 0, z = 0;  //!< point the camera looks at
  float zoom = 10;
  float azimuthDeg = 0;
  float elevationDeg = 30;
};

/** Interpolates a camera path at time t (keyframes sorted by time).
 * Azimuth is interpolated along the shortest angle. */
CameraKeyframe interpolateCameraPath(const std::vector<CameraKeyframe>& path, double t);

/** Re-times a camera path so the camera target moves at a constant speed. */
void retimeCameraPathConstantSpeed(std::vector<CameraKeyframe>& path, double speed);

class SceneViewerApp
{
 public:
  SceneViewerApp();

  /** Runs the application. `fileToOpen` may be empty. */
  int run(const std::string& fileToOpen);

 private:
  mrpt::imgui::CImGuiAppShell m_app;
  mrpt::imgui::CImGuiBackgroundScene m_view;
  mrpt::imgui::CImGuiFileDialog m_fileDlg;
  mrpt::imgui::CImGuiMessageQueue m_msgs;
  mrpt::imgui::CImGuiAboutDialog m_about;
  mrpt::imgui::CImGuiLogConsole m_console;
  std::unique_ptr<mrpt::config::CConfigFile> m_cfg;

  mrpt::viz::Scene::Ptr m_scene;
  std::string m_loadedFileName;
  std::string m_statusMsg;

  // --- Options (persisted) ---

  // --- Panels visibility ---

  // --- Autoplay / capture ---
  std::string m_captureDir;

  // --- Camera travelling ---
  enum class Travelling
  {
    None,
    Circular,
    Path
  };
  Travelling m_travelling = Travelling::None;
  // Circular:
  // Path:
  std::vector<CameraKeyframe> m_path;

  // --- Selection ---
  std::vector<mrpt::viz::CVisualObject::Ptr> m_selected;

  // --- Async loading ---
  struct LoadResult
  {
    mrpt::viz::Scene::Ptr scene;           //!< a whole new scene (for .3Dscene / PLY)
    mrpt::viz::CVisualObject::Ptr object;  //!< or an object to insert
    std::string fileName;
    std::string info;
    double seconds = 0;
    bool isInASequence = false;
    bool keepCamera = false;
  };
  mrpt::imgui::AsyncTask<LoadResult> m_loader;

  // --- Popup state ---
  std::string m_pendingPlyFile;

  std::string m_pendingHighResFile;

  std::vector<std::string> m_vizClassNames;
  std::vector<bool> m_vizClassChecked;

  /** Actions to run at the start of the next frame, outside of any ImGui
   * window (e.g. off-screen renders). */
  std::vector<std::function<void()>> m_deferred;

  // --- Frame stats ---

  // Scalars and flags, grouped to avoid padding:
  std::time_t m_loadedFileMTime = 0;
  double m_lastFileWatchCheck = 0;
  double m_lastAutoplayStep = 0;
  double m_travellingT = 0;
  double m_fps = 0;
  int m_autoplayDelayMs = 5;
  int m_captureCount = 0;
  float m_circTarget[3] = {0, 0, 0};
  float m_circZoom = 25;
  float m_circElevation = 15;
  float m_circAzimuth = 90;
  float m_circMinOffset = 90;
  float m_circMaxOffset = 90;
  float m_circSpeedDegPerSec = 5;
  float m_pathSpeed = 0.2f;
  int m_plyClass = 0;      // 0: CPointCloud, 1: CPointCloudColoured
  int m_plyColorFrom = 0;  // 0: none, 1:X, 2:Y, 3:Z
  float m_plyPointSize = 1.0f;
  float m_plyYPR[3] = {0, 0, 90};
  int m_highResSize[2] = {1920, 1080};
  float m_scaleFactor = 1.0f;
  bool m_showFileNameInViewport = false;
  bool m_freeCameraAlways = false;
  bool m_freeCameraAlwaysNoAzimuth = false;
  bool m_shadows = true;
  bool m_ssao = false;
  bool m_ortho = false;
  bool m_watchFile = true;
  bool m_showViewPanel = true;
  bool m_showTravellingPanel = true;
  bool m_showConsole = false;
  bool m_autoplay = false;
  bool m_capturing = false;
  bool m_pathConstantSpeed = false;
  bool m_pathLoop = false;
  bool m_openPlyPopup = false;
  bool m_plyGrid = true;
  bool m_plyCorner = true;
  bool m_openHighResPopup = false;
  bool m_openSelectByClassPopup = false;
  bool m_openScalePopup = false;

  // Methods:
  void loadSettings();
  void saveSettings();

  void newScene();
  void startLoadSceneFile(const std::string& file, bool isInASequence = false);
  void startImportModel(const std::string& file);
  void startImportPly(const std::string& file);
  void applyLoadResult(LoadResult r);
  void reload();
  void saveScene(const std::string& file);
  void importImage(const std::string& file);
  void exportPointCloudsToPly(const std::string& file);
  void highResRender(const std::string& file, int w, int h);
  void loadNeighborFile(int direction);
  void saveCroppedSceneImage(const mrpt::img::CImage& frame, const std::string& file);

  void applyViewportOptions();
  void showStats();

  void renderFrame();
  void renderMainMenu();
  void renderToolbar();
  void renderStatusBar();
  void renderViewPanel();
  void renderTravellingPanel();
  void renderPopups();
  void handleFileDialogResult(const std::vector<std::string>& files);
  void handleKeyboard();
  void updateTravelling(double dt);
  void checkFileWatch();
  void updateAutoplayAndCapture();
  void setupDefaultLayout(unsigned int dockspaceId);

  [[nodiscard]] std::string lastDir() const;
  void setLastDir(const std::string& file);
};
