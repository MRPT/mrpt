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

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API
#include <imgui_stdlib.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/imgui/widgets.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPlanarLaserScan.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/stock_objects.h>

#include <CLI/CLI.hpp>
#include <cstdio>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>

namespace
{
#include "virtual_map1.xpm"
}  // namespace

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace mrpt::serialization;
using mrpt::imgui::CImGuiFileDialog;
using mrpt::imgui::tooltip;

namespace
{
constexpr const char* APP_NAME = "GridmapNavSimul";

/** Simulation time step [s] (the period of the original app timer). */
constexpr double SIM_STEP = 0.01;

/** A map center coordinate from the UI: negative means "image center". */
double centerCoord(float v)
{
  return v < 0 ? std::numeric_limits<double>::max() : static_cast<double>(v);
}

constexpr const char* TAG_LOAD_MAP = "load_map";
constexpr const char* TAG_OUT_RAWLOG = "out_rawlog";
constexpr const char* TAG_RESIM_GT = "resim_gt";
constexpr const char* TAG_RESIM_RAWLOG = "resim_rawlog";
constexpr const char* TAG_RESIM_OUT = "resim_out";

/** Decodes a XPM image (one char per pixel, "c #RRGGBB" or "c None" colors)
 * into a grayscale image. Transparent pixels become gray (unknown space). */
CImage xpmToGray(const char* const* xpm)
{
  int w = 0;
  int h = 0;
  int nColors = 0;
  int cpp = 0;
  if (std::sscanf(xpm[0], "%d %d %d %d", &w, &h, &nColors, &cpp) != 4 || cpp != 1)
  {
    THROW_EXCEPTION("Unsupported XPM image");
  }
  std::map<char, uint8_t> palette;
  for (int i = 0; i < nColors; i++)
  {
    const std::string line = xpm[1 + i];
    const auto hash = line.find('#');
    uint8_t v = 0x80;
    if (hash != std::string::npos && line.size() >= hash + 7)
    {
      const auto rgb = std::stoul(line.substr(hash + 1, 6), nullptr, 16);
      const auto r = (rgb >> 16) & 0xff;
      const auto g = (rgb >> 8) & 0xff;
      const auto b = rgb & 0xff;
      v = static_cast<uint8_t>((r + g + b) / 3);
    }
    palette[line[0]] = v;
  }
  CImage img(w, h, CH_GRAY);
  for (int y = 0; y < h; y++)
  {
    const char* row = xpm[1 + nColors + y];
    auto* dst = img.ptrLine<uint8_t>(y);
    for (int x = 0; x < w; x++)
    {
      const auto it = palette.find(row[x]);
      dst[x] = it != palette.end() ? it->second : 0x80;
    }
  }
  return img;
}

class GridmapNavSimulApp
{
 public:
  GridmapNavSimulApp();
  int run();

 private:
  mrpt::imgui::CImGuiAppShell m_app;
  mrpt::imgui::CImGuiBackgroundScene m_world;
  mrpt::imgui::CImGuiFileDialog m_fileDlg;
  mrpt::imgui::CImGuiMessageQueue m_msgs;
  mrpt::imgui::CImGuiAboutDialog m_about;

  mrpt::viz::Scene::Ptr m_scene = mrpt::viz::Scene::Create();
  mrpt::viz::CSetOfObjects::Ptr gl_grid = mrpt::viz::CSetOfObjects::Create();
  mrpt::viz::CSetOfObjects::Ptr gl_robot;
  mrpt::viz::CPlanarLaserScan::Ptr gl_scan;
  mrpt::viz::CPointCloud::Ptr gl_path_GT;
  mrpt::viz::CPointCloud::Ptr gl_path_ODO;

  // Simulation:
  mrpt::kinematics::CVehicleSimul_DiffDriven m_robot;
  /** Simulated time pending to run, in seconds. */
  double m_simTimeToRun = 0;
  COccupancyGridMap2D m_grid;
  CPose2D m_lastOdo;
  CPose2D m_poseStart;
  std::vector<CPose2D> m_pathGT;
  std::vector<CPose2D> m_pathODO;
  bool m_followRobot = true;

  // Sensor:
  int m_laserNRanges = 361;
  float m_laserSpanDeg = 180;
  float m_laserStdRange = 0.01f;
  float m_laserStdBearingDeg = 0.05f;

  // Odometry error model (relative to each motion increment):
  float m_Axb = 0.01f;
  float m_Axs = 0.01f;
  float m_Ayb = 0.01f;
  float m_Ays = 0.10f;
  float m_Apb = static_cast<float>(DEG2RAD(0.01));
  float m_Aps = static_cast<float>(DEG2RAD(0.01));

  // Recording:
  bool m_recording = false;
  std::string m_outRawlog = "./simul.rawlog";
  std::string m_outGT = "./simul.rawlog.GT.txt";
  bool m_sfFormat = true;  // Actions/SensoryFrames rawlog format
  int m_decimation = 1;
  long m_decimationCount = 0;
  CCompressedOutputStream m_outs;
  CFileOutputStream m_outGTFile;
  size_t m_recordedSteps = 0;

  // Teleoperation:
  bool m_useJoystick = false;

  // Re-simulation:
  std::string m_resimGT;
  std::string m_resimRawlog;

  // Popup to load a map from an image:
  bool m_openMapImagePopup = false;
  std::string m_pendingMapImage;
  float m_mapCellSize = 0.05f;
  float m_mapCenter[2] = {-1, -1};

  bool m_showSimulation = true;
  bool m_showSensor = true;
  bool m_showOdometry = true;

  void updateGridMap3D()
  {
    gl_grid->clear();
    gl_grid->insert(m_grid.getVisualization());
  }
  void startRecording();
  void stopRecording();
  void simulateStep(double dt);
  void handleTeleop(double dt);
  CObservation2DRangeScan simulateScan(const CPose2D& p);
  void resimulate(const std::string& outRawlog);
  void handleFileDialogResult(const std::vector<std::string>& files);

  void renderMainMenu();
  void renderPanels();
  void renderFrame();
};

GridmapNavSimulApp::GridmapNavSimulApp()
{
  m_about.appName = APP_NAME;
  m_about.description =
      "Simulator of a differential-driven robot with a 2D laser scanner in an "
      "occupancy grid map, to record synthetic datasets (rawlogs) with ground truth.";
  m_about.extraTabs.emplace_back(
      "Driving",
      "Arrow keys (over the 3D view): increase/decrease the linear and angular speed.\n"
      "Space: brake.\n"
      "With 'Use joystick' enabled, the first joystick axes drive the robot, its\n"
      "first button brakes and the next two zoom in and out.\n");

  // Default map:
  m_grid.loadFromBitmap(xpmToGray(virtual_map1_xpm), 0.03f);

  m_scene->insert(mrpt::viz::CGridPlaneXY::Create(-100, 100, -100, 100, 0, 5));
  updateGridMap3D();
  m_scene->insert(gl_grid);

  gl_path_GT = mrpt::viz::CPointCloud::Create();
  gl_path_GT->setColor(0, 0, 0, 0.7f);
  gl_path_GT->setLocation(0, 0, 0.01);
  gl_path_GT->setPointSize(3);
  m_scene->insert(gl_path_GT);

  gl_path_ODO = mrpt::viz::CPointCloud::Create();
  gl_path_ODO->setColor(0, 1, 0, 0.7f);
  gl_path_ODO->setLocation(0, 0, 0.01);
  gl_path_ODO->setPointSize(2);
  m_scene->insert(gl_path_ODO);

  gl_robot = mrpt::viz::stock_objects::RobotPioneer();
  m_scene->insert(gl_robot);
  gl_scan = mrpt::viz::CPlanarLaserScan::Create();
  gl_robot->insert(gl_scan);

  m_world.view.setScene(m_scene);
  auto& cam = m_world.view.cameraController;
  cam.setCameraPointing(0.0f, 0.0f, 0.0f);
  cam.setElevationDegrees(60.0f);
  cam.setZoomDistance(25.0f);
}

CObservation2DRangeScan GridmapNavSimulApp::simulateScan(const CPose2D& p)
{
  CObservation2DRangeScan scan;
  scan.sensorLabel = "LASER_SIM";
  scan.sensorPose.setFromValues(0.20, 0, 0.10);
  scan.maxRange = 80;
  scan.aperture = static_cast<float>(DEG2RAD(m_laserSpanDeg));
  scan.stdError = m_laserStdRange;
  m_grid.laserScanSimulator(
      scan, p, 0.6f, static_cast<size_t>(std::max(2, m_laserNRanges)), m_laserStdRange, 1,
      static_cast<float>(DEG2RAD(m_laserStdBearingDeg)));
  return scan;
}

void GridmapNavSimulApp::startRecording()
{
  m_pathGT.clear();
  m_pathODO.clear();
  gl_path_GT->clear();
  gl_path_ODO->clear();

  m_poseStart = CPose2D(m_robot.getCurrentGTPose());
  m_robot.setCurrentOdometricPose(TPose2D(0, 0, 0));
  m_lastOdo = CPose2D(0, 0, 0);
  m_robot.setOdometryErrors(true, m_Axb, m_Axs, m_Ayb, m_Ays, m_Apb, m_Aps);

  if (!m_outs.open(m_outRawlog))
  {
    THROW_EXCEPTION_FMT("Cannot open output rawlog file: '%s'", m_outRawlog.c_str());
  }
  if (!m_outGTFile.open(m_outGT, mrpt::io::OpenMode::TRUNCATE))
  {
    m_outs.close();
    THROW_EXCEPTION_FMT("Cannot open output GT file: '%s'", m_outGT.c_str());
  }
  // Save the reference gridmap too:
  {
    const std::string gridFile = m_outRawlog + "_grid.gridmap.gz";
    CCompressedOutputStream outGrid;
    if (!outGrid.open(gridFile))
    {
      m_outs.close();
      m_outGTFile.close();
      THROW_EXCEPTION_FMT("Cannot open output file for the gridmap: '%s'", gridFile.c_str());
    }
    archiveFrom(outGrid) << m_grid;
  }
  m_decimationCount = 0;
  m_recordedSteps = 0;
  m_recording = true;
}

void GridmapNavSimulApp::stopRecording()
{
  m_recording = false;
  if (m_outs.fileOpenCorrectly())
  {
    m_outs.close();
  }
  if (m_outGTFile.fileOpenCorrectly())
  {
    m_outGTFile.close();
  }
}

void GridmapNavSimulApp::handleTeleop(double dt)
{
  const float V_MAX_ACC = 1;
  const float W_MAX_ACC = DEG2RAD(45.0f);

  const TTwist2D velLocal = m_robot.getCurrentGTVelLocal();
  double v = velLocal.vx;
  double w = velLocal.omega;
  bool changed = false;

  // Joystick (GLFW):
  if (m_useJoystick && (glfwJoystickPresent(GLFW_JOYSTICK_1) != 0))
  {
    int nAxes = 0;
    int nBtns = 0;
    const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &nAxes);
    const unsigned char* btns = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &nBtns);
    if (axes != nullptr && nAxes >= 2)
    {
      v -= axes[1] * V_MAX_ACC * dt;
      w -= axes[0] * W_MAX_ACC * dt;
      changed = true;
    }
    if (btns != nullptr && nBtns >= 1 && btns[0] == GLFW_PRESS)
    {
      v *= 0.95;
      w *= 0.95;
    }
    auto& cam = m_world.view.cameraController;
    if (btns != nullptr && nBtns >= 2 && btns[1] == GLFW_PRESS)
    {
      cam.setZoomDistance(cam.getZoomDistance() * 0.99f);
    }
    if (btns != nullptr && nBtns >= 3 && btns[2] == GLFW_PRESS)
    {
      cam.setZoomDistance(cam.getZoomDistance() * 1.01f);
    }
  }

  // Keyboard (unless typing in a panel):
  const ImGuiIO& io = ImGui::GetIO();
  if (!io.WantTextInput && (!io.WantCaptureKeyboard || m_world.isHovered()))
  {
    float x = 0;
    float y = 0;
    if (ImGui::IsKeyDown(ImGuiKey_UpArrow))
    {
      y = -1;
    }
    if (ImGui::IsKeyDown(ImGuiKey_DownArrow))
    {
      y = 1;
    }
    if (ImGui::IsKeyDown(ImGuiKey_LeftArrow))
    {
      x = -1;
    }
    if (ImGui::IsKeyDown(ImGuiKey_RightArrow))
    {
      x = 1;
    }
    if (x != 0 || y != 0)
    {
      v -= y * V_MAX_ACC * dt;
      w -= x * W_MAX_ACC * dt;
      changed = true;
    }
    else if (ImGui::IsKeyDown(ImGuiKey_Space))
    {
      v *= 0.95;
      w *= 0.95;
      changed = true;
    }
  }
  if (changed)
  {
    m_robot.movementCommand(v, w);
  }
}

void GridmapNavSimulApp::simulateStep(double dt)
{
  m_robot.simulateOneTimeStep(dt);
  handleTeleop(dt);

  const TPose2D p = m_robot.getCurrentGTPose();
  const auto scan = simulateScan(CPose2D(p));

  if (m_recording && m_outs.fileOpenCorrectly() &&
      0 == (m_decimationCount++ % std::max(1, m_decimation)))
  {
    const auto timNow = mrpt::Clock::now();
    const CPose2D odoNow = CPose2D(m_robot.getCurrentOdometricPose());

    if (m_sfFormat)
    {
      CActionCollection acts;
      CActionRobotMovement2D act;
      CActionRobotMovement2D::TMotionModelOptions opts;
      opts.modelSelection = CActionRobotMovement2D::mmGaussian;
      const CPose2D Aodom = odoNow - m_lastOdo;
      m_lastOdo = odoNow;
      act.computeFromOdometry(Aodom, opts);
      act.timestamp = timNow;
      acts.insert(act);
      archiveFrom(m_outs) << acts;

      CSensoryFrame sf;
      auto obs = CObservation2DRangeScan::Create(scan);
      obs->timestamp = timNow;
      sf.insert(obs);
      archiveFrom(m_outs) << sf;
    }
    else
    {
      CObservationOdometry odo;
      odo.timestamp = timNow;
      odo.sensorLabel = "odometry";
      odo.odometry = odoNow;
      odo.hasVelocities = true;
      odo.velocityLocal = m_robot.getCurrentGTVelLocal();
      auto obs = scan;
      obs.timestamp = timNow;
      archiveFrom(m_outs) << odo << obs;
    }

    // Ground truth pose, relative to the start:
    const CPose2D rel = CPose2D(p) - m_poseStart;
    m_outGTFile.printf(
        "%f %.03f %.03f %.03f\n", mrpt::Clock::toDouble(timNow), rel.x(), rel.y(), rel.phi());

    m_pathGT.emplace_back(p);
    m_pathODO.push_back(odoNow);
    gl_path_GT->insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), 0.0f);
    const CPose2D odoGlobal = m_poseStart + odoNow;
    gl_path_ODO->insertPoint(
        static_cast<float>(odoGlobal.x()), static_cast<float>(odoGlobal.y()), 0.0f);
    m_recordedSteps++;
  }

  gl_scan->setScan(scan);
  gl_robot->setPose(CPose3D(p));

  if (m_followRobot)
  {
    auto& cam = m_world.view.cameraController;
    cam.setCameraPointing(
        static_cast<float>(p.x), static_cast<float>(p.y), cam.getCameraPointingZ());
  }
}

void GridmapNavSimulApp::resimulate(const std::string& outRawlog)
{
  // GT file rows are: time x y phi
  CMatrixDouble GT;
  GT.loadFromTextFile(m_resimGT);

  CRawlog rawlog;
  if (!rawlog.loadFromRawLogFile(m_resimRawlog))
  {
    THROW_EXCEPTION_FMT("Cannot load rawlog: '%s'", m_resimRawlog.c_str());
  }
  ASSERT_(rawlog.size() > 0);
  ASSERT_(GT.cols() >= 4);
  ASSERTMSG_(
      rawlog.getType(0) == CRawlog::TEntryType::etActionCollection,
      "Only rawlogs in the Actions/SensoryFrames format can be re-simulated");
  ASSERT_EQUAL_(rawlog.size() / 2, static_cast<size_t>(GT.rows()));

  for (size_t i = 1; i < rawlog.size(); i += 2)
  {
    ASSERT_(rawlog.getType(i) == CRawlog::TEntryType::etSensoryFrame);
    auto sf = rawlog.getAsObservations(i);
    const auto r = static_cast<int>(i / 2);
    const CPose2D gtPose(GT(r, 1), GT(r, 2), GT(r, 3));
    auto scan = sf->getObservationByClass<CObservation2DRangeScan>();
    ASSERT_(scan);
    m_grid.laserScanSimulator(
        *scan, gtPose, 0.5f, static_cast<size_t>(std::max(2, m_laserNRanges)), m_laserStdRange, 1,
        static_cast<float>(DEG2RAD(m_laserStdBearingDeg)));
  }

  CCompressedOutputStream f(outRawlog);
  archiveFrom(f) << rawlog;
  // The GT file is the same:
  GT.saveToTextFile(outRawlog + ".GT.txt", MATRIX_FORMAT_FIXED);
  m_msgs.info(
      "Re-simulation", "Saved: " + outRawlog + "\nand its ground truth: " + outRawlog + ".GT.txt");
}

void GridmapNavSimulApp::handleFileDialogResult(const std::vector<std::string>& files)
{
  if (files.empty())
  {
    return;
  }
  const std::string& f = files.front();
  const std::string& tag = m_fileDlg.tag;
  if (tag == TAG_LOAD_MAP)
  {
    if (mrpt::system::lowerCase(mrpt::system::extractFileExtension(f, true)) == "gridmap")
    {
      CCompressedInputStream in(f);
      archiveFrom(in) >> m_grid;
      updateGridMap3D();
    }
    else
    {
      m_pendingMapImage = f;
      m_openMapImagePopup = true;
    }
  }
  else if (tag == TAG_OUT_RAWLOG)
  {
    m_outRawlog = f;
    m_outGT = f + ".GT.txt";
  }
  else if (tag == TAG_RESIM_GT)
  {
    m_resimGT = f;
    m_fileDlg.tag = TAG_RESIM_RAWLOG;
    m_fileDlg.open(
        CImGuiFileDialog::Mode::Open, "Corresponding rawlog...",
        {
            {"Rawlogs", {"rawlog"}}
    },
        mrpt::system::extractFileDirectory(f));
  }
  else if (tag == TAG_RESIM_RAWLOG)
  {
    m_resimRawlog = f;
    m_fileDlg.tag = TAG_RESIM_OUT;
    m_fileDlg.open(
        CImGuiFileDialog::Mode::Save, "New rawlog to create...",
        {
            {"Rawlogs", {"rawlog"}}
    },
        mrpt::system::extractFileDirectory(f) + "/simul_new.rawlog");
  }
  else if (tag == TAG_RESIM_OUT)
  {
    resimulate(f);
  }
}

void GridmapNavSimulApp::renderMainMenu()
{
  if (ImGui::BeginMenu("File"))
  {
    if (ImGui::MenuItem(ICON_MS_FOLDER_OPEN " Load gridmap...", nullptr, false, !m_recording))
    {
      m_fileDlg.tag = TAG_LOAD_MAP;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Open, "Select grid map to load",
          {
              {         "Images", {"png", "jpg", "gif", "bmp"}},
              {"Binary gridmaps",    {"gridmap", "gridmap.gz"}}
      });
    }
    if (ImGui::MenuItem(ICON_MS_SAVE " Set output rawlog file...", nullptr, false, !m_recording))
    {
      m_fileDlg.tag = TAG_OUT_RAWLOG;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Rawlog to create",
          {
              {"Rawlogs", {"rawlog"}}
      },
          m_outRawlog);
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_CLOSE " Quit"))
    {
      m_app.requestClose();
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View"))
  {
    ImGui::MenuItem("Camera follows the robot", nullptr, &m_followRobot);
    if (ImGui::MenuItem("Clear robot paths"))
    {
      gl_path_GT->clear();
      gl_path_ODO->clear();
    }
    ImGui::Separator();
    ImGui::MenuItem("Simulation", nullptr, &m_showSimulation);
    ImGui::MenuItem("Sensor", nullptr, &m_showSensor);
    ImGui::MenuItem("Odometry error model", nullptr, &m_showOdometry);
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Help"))
  {
    if (ImGui::MenuItem(ICON_MS_INFO " About", "F1"))
    {
      m_about.open();
    }
    ImGui::EndMenu();
  }
}

void GridmapNavSimulApp::renderPanels()
{
  if (m_showSimulation)
  {
    if (ImGui::Begin(ICON_MS_VIDEOCAM " Simulation###Simulation", &m_showSimulation))
    {
      if (!m_recording)
      {
        if (ImGui::Button(ICON_MS_FIBER_MANUAL_RECORD " START recording"))
        {
          try
          {
            startRecording();
          }
          catch (const std::exception& e)
          {
            m_msgs.error("Error", mrpt::exception_to_str(e));
          }
        }
      }
      else
      {
        if (ImGui::Button(ICON_MS_STOP " STOP recording"))
        {
          stopRecording();
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%zu steps recorded", m_recordedSteps);
      }
      ImGui::BeginDisabled(m_recording);
      ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 6);
      ImGui::InputText("Output rawlog", &m_outRawlog);
      ImGui::SameLine();
      if (ImGui::Button(ICON_MS_FOLDER_OPEN "##outraw"))
      {
        m_fileDlg.tag = TAG_OUT_RAWLOG;
        m_fileDlg.open(
            CImGuiFileDialog::Mode::Save, "Rawlog to create",
            {
                {"Rawlogs", {"rawlog"}}
        },
            m_outRawlog);
      }
      ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 6);
      ImGui::InputText("Output GT", &m_outGT);
      tooltip("Ground truth poses, one per line: time x y phi (relative to the start)");
      ImGui::Checkbox("Actions/SensoryFrames format", &m_sfFormat);
      tooltip("If unchecked, the rawlog holds observations only (odometry + scans)");
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
      ImGui::InputInt("Decimation", &m_decimation);
      m_decimation = std::max(1, m_decimation);
      ImGui::EndDisabled();
      ImGui::Separator();
      ImGui::Checkbox("Use joystick", &m_useJoystick);
      if (m_useJoystick && (glfwJoystickPresent(GLFW_JOYSTICK_1) == 0))
      {
        ImGui::SameLine();
        ImGui::TextDisabled("(none detected)");
      }
      ImGui::TextDisabled("Drive with the arrow keys over the 3D view; Space brakes.");
      ImGui::Separator();
      ImGui::BeginDisabled(m_recording);
      if (ImGui::Button(ICON_MS_REPLAY " Re-simulate a rawlog..."))
      {
        m_fileDlg.tag = TAG_RESIM_GT;
        m_fileDlg.open(
            CImGuiFileDialog::Mode::Open, "Existing GT file...",
            {
                {"Text files", {"txt"}}
        });
      }
      tooltip(
          "Regenerate the laser scans of a recorded rawlog (and its GT file), with the "
          "current sensor parameters.");
      ImGui::EndDisabled();
    }
    ImGui::End();
  }

  if (m_showSensor)
  {
    if (ImGui::Begin(ICON_MS_SENSORS " Sensor###Sensor", &m_showSensor))
    {
      ImGui::PushItemWidth(ImGui::GetFontSize() * 7);
      ImGui::InputInt("Range count", &m_laserNRanges);
      m_laserNRanges = std::clamp(m_laserNRanges, 2, 100000);
      ImGui::InputFloat("Span (deg)", &m_laserSpanDeg, 1.0f, 10.0f, "%.1f");
      m_laserSpanDeg = std::clamp(m_laserSpanDeg, 1.0f, 360.0f);
      ImGui::InputFloat("Range noise std. (m)", &m_laserStdRange, 0.001f, 0.01f, "%.4f");
      ImGui::InputFloat("Bearing noise std. (deg)", &m_laserStdBearingDeg, 0.01f, 0.1f, "%.3f");
      ImGui::PopItemWidth();
    }
    ImGui::End();
  }

  if (m_showOdometry)
  {
    if (ImGui::Begin(ICON_MS_ROUTE " Odometry error model###Odometry", &m_showOdometry))
    {
      ImGui::TextWrapped(
          "Odometry increments are scaled by (1 + bias + std * N(0,1)) in each "
          "component: values are fractions of each motion increment (e.g. 0.01 = 1%%). "
          "Applied when recording starts.");
      ImGui::BeginDisabled(m_recording);
      ImGui::PushItemWidth(ImGui::GetFontSize() * 7);
      ImGui::InputFloat("x: bias", &m_Axb, 0, 0, "%.5f");
      ImGui::InputFloat("x: std", &m_Axs, 0, 0, "%.5f");
      ImGui::InputFloat("y: bias", &m_Ayb, 0, 0, "%.5f");
      ImGui::InputFloat("y: std", &m_Ays, 0, 0, "%.5f");
      ImGui::InputFloat("heading: bias", &m_Apb, 0, 0, "%.6f");
      ImGui::InputFloat("heading: std", &m_Aps, 0, 0, "%.6f");
      ImGui::PopItemWidth();
      ImGui::EndDisabled();
    }
    ImGui::End();
  }

  if (m_openMapImagePopup)
  {
    ImGui::OpenPopup("Grid map from image");
    m_openMapImagePopup = false;
  }
  if (ImGui::BeginPopupModal("Grid map from image", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted(mrpt::system::extractFileName(m_pendingMapImage).c_str());
    ImGui::InputFloat("Cell size (m/pixel)", &m_mapCellSize, 0.01f, 0.1f, "%.4f");
    ImGui::InputFloat2("Central pixel (x, y); -1: image center", m_mapCenter);
    if (ImGui::Button("Load"))
    {
      CImage img;
      if (!img.loadFromFile(m_pendingMapImage, CH_GRAY))
      {
        m_msgs.error("Error", "Can't load the image file (check its format).");
      }
      else if (!m_grid.loadFromBitmap(
                   img, std::max(1e-4f, m_mapCellSize),
                   {centerCoord(m_mapCenter[0]), centerCoord(m_mapCenter[1])}))
      {
        m_msgs.error("Error", "Can't load the image file into the gridmap.");
      }
      else
      {
        updateGridMap3D();
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
}

void GridmapNavSimulApp::renderFrame()
{
  if (auto files = m_fileDlg.poll(); files)
  {
    try
    {
      handleFileDialogResult(*files);
    }
    catch (const std::exception& e)
    {
      m_msgs.error("Error", mrpt::exception_to_str(e));
    }
  }
  if (!ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_F1, false))
  {
    m_about.open();
  }

  try
  {
    // Fixed time steps, so the simulation and the recording rate do not
    // depend on the display frame rate:
    m_simTimeToRun += std::min(0.25, static_cast<double>(ImGui::GetIO().DeltaTime));
    while (m_simTimeToRun >= SIM_STEP)
    {
      simulateStep(SIM_STEP);
      m_simTimeToRun -= SIM_STEP;
    }
  }
  catch (const std::exception& e)
  {
    stopRecording();
    m_msgs.error("Exception", mrpt::exception_to_str(e));
  }

  m_world.render(m_app);
  renderPanels();
  m_about.render();
  m_msgs.render();
}

int GridmapNavSimulApp::run()
{
  if (!m_app.init(APP_NAME))
  {
    std::cerr << "Could not create the application window.\n";
    return 1;
  }
  m_app.mainMenuBar = [this]() { renderMainMenu(); };
  m_app.beforeDockspace = [this]()
  {
    if (mrpt::imgui::beginStatusBar())
    {
      const auto p = m_robot.getCurrentGTPose();
      const auto v = m_robot.getCurrentGTVelLocal();
      ImGui::Text(
          "Pose: (%.03f, %.03f, %.01f deg)   Twist: v=%.03f m/s  w=%.02f deg/s", p.x, p.y,
          RAD2DEG(p.phi), v.vx, RAD2DEG(v.omega));
      if (m_recording)
      {
        ImGui::SameLine(0, ImGui::GetFontSize() * 2);
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), ICON_MS_FIBER_MANUAL_RECORD " Recording");
      }
    }
    mrpt::imgui::endStatusBar();
  };
  m_app.setupDefaultLayout = [](unsigned int dockspaceId)
  {
    ImGuiID right = 0;
    ImGuiID rest = 0;
    ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Right, 0.28f, &right, &rest);
    ImGuiID rightBottom = 0;
    ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.45f, &rightBottom, &right);
    ImGui::DockBuilderDockWindow("Simulation", right);
    ImGui::DockBuilderDockWindow("Sensor", rightBottom);
    ImGui::DockBuilderDockWindow("Odometry", rightBottom);
  };
  m_app.onCloseRequest = [this]()
  {
    stopRecording();
    return true;
  };

  m_app.run([this]() { renderFrame(); });
  stopRecording();
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli("GridmapNavSimul: record simulated rawlogs of a robot in a gridmap");
    CLI11_PARSE(cli, argc, argv);
    GridmapNavSimulApp app;
    return app.run();
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
