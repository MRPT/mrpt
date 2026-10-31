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

#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API
#include <imgui_stdlib.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/imgui/widgets.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/nav.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPlanarLaserScan.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/stock_objects.h>

#include <CLI/CLI.hpp>
#include <cmath>
#include <iostream>
#include <memory>

#include "../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h"

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::nav;
using namespace mrpt::poses;
using mrpt::imgui::CImGuiFileDialog;
using mrpt::imgui::tooltip;

namespace
{
constexpr const char* APP_NAME = "holonomic-navigator-demo";

/** Simulation parameters, in the "SIMULATOR" section of the parameters. */
struct TSimulOptions : public mrpt::config::CLoadableOptions
{
  double ROBOT_MAX_SPEED{4.0};
  double MAX_SENSOR_RADIUS{5.0};
  uint64_t SENSOR_NUM_RANGES{181};
  double SENSOR_RANGE_NOISE_STD{0.02};

  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& source, const std::string& section) override
  {
    MRPT_LOAD_CONFIG_VAR(ROBOT_MAX_SPEED, double, source, section);
    MRPT_LOAD_CONFIG_VAR(MAX_SENSOR_RADIUS, double, source, section);
    MRPT_LOAD_CONFIG_VAR(SENSOR_NUM_RANGES, uint64_t, source, section);
    MRPT_LOAD_CONFIG_VAR(SENSOR_RANGE_NOISE_STD, double, source, section);
  }
  void saveToConfigFile(
      mrpt::config::CConfigFileBase& cfg, const std::string& section) const override
  {
    const int WN = 40;
    const int WV = 20;
    cfg.write(
        section, "ROBOT_MAX_SPEED", ROBOT_MAX_SPEED, WN, WV, "Maximum speed for the robot (m/s)");
    cfg.write(
        section, "MAX_SENSOR_RADIUS", MAX_SENSOR_RADIUS, WN, WV,
        "Maximum range of the 360deg sensor (meters)");
    cfg.write(
        section, "SENSOR_NUM_RANGES", SENSOR_NUM_RANGES, WN, WV,
        "Number of ranges in the 360deg sensor FOV");
    cfg.write(
        section, "SENSOR_RANGE_NOISE_STD", SENSOR_RANGE_NOISE_STD, WN, WV,
        "Sensor noise (one sigma, in meters)");
  }
};

/** Four arrows pointing to a point: the navigation target marker. */
mrpt::viz::CSetOfObjects::Ptr targetMarker()
{
  auto m = mrpt::viz::CSetOfObjects::Create();
  const TPoint3Df dirs[4] = {
      { 1,  0, 0},
      {-1,  0, 0},
      { 0,  1, 0},
      { 0, -1, 0}
  };
  for (const auto& d : dirs)
  {
    auto obj = mrpt::viz::CArrow::Create(d, d * 0.2f, 0.4f, 0.05f, 0.15f);
    obj->setColor_u8(TColor(0, 0, 255));
    m->insert(obj);
  }
  return m;
}

class HolonomicNavDemoApp
{
 public:
  HolonomicNavDemoApp();
  int run(bool startNow);

 private:
  mrpt::imgui::CImGuiAppShell m_app;
  mrpt::imgui::CImGuiBackgroundScene m_world;
  mrpt::imgui::CImGuiSceneView m_localView;
  mrpt::imgui::CImGuiFileDialog m_fileDlg;
  mrpt::imgui::CImGuiMessageQueue m_msgs;
  mrpt::imgui::CImGuiAboutDialog m_about;

  mrpt::viz::Scene::Ptr m_worldScene = mrpt::viz::Scene::Create();
  mrpt::viz::Scene::Ptr m_localScene = mrpt::viz::Scene::Create();

  // Simulation:
  TSimulOptions m_simulOptions;
  std::unique_ptr<CAbstractHolonomicReactiveMethod> m_holonomicMethod;
  COccupancyGridMap2D m_gridMap;
  TPose2D m_targetPose{5, 3, 0};
  TPose2D m_robotPose{0, 0, 0};
  bool m_running = false;
  mrpt::system::CTicTac m_runtime;

  // GUI state:
  enum class PickMode
  {
    None,
    Target,
    Robot
  };
  PickMode m_pickMode = PickMode::None;
  std::optional<TPoint2D> m_cursorPos;
  int m_holoMethod = 1;  // 0: VFF, 1: ND
  std::string m_params;
  std::string m_localInfo;
  double m_lastSpeed = 0;
  double m_lastDirDeg = 0;
  bool m_showMaxRange = true;
  bool m_showRobotPath = true;
  bool m_showConfig = true;
  bool m_showLocalView = true;
  bool m_showStats = true;

  // Popup to load a map from an image:
  bool m_openMapImagePopup = false;
  std::string m_pendingMapImage;
  float m_mapCellSize = 0.05f;
  float m_mapCenter[2] = {-1, -1};

  // Scene objects:
  mrpt::viz::CSetOfObjects::Ptr gl_grid;
  mrpt::viz::CSetOfObjects::Ptr gl_robot;
  mrpt::viz::CSetOfObjects::Ptr gl_target;
  mrpt::viz::CSetOfObjects::Ptr gl_placing_target;
  mrpt::viz::CSetOfObjects::Ptr gl_placing_robot;
  mrpt::viz::CDisk::Ptr gl_robot_sensor_range;
  mrpt::viz::CSetOfLines::Ptr gl_robot_path;
  mrpt::viz::CPlanarLaserScan::Ptr gl_scan3D;
  mrpt::viz::CPlanarLaserScan::Ptr gl_scan2D;
  mrpt::viz::CSimpleLine::Ptr gl_line_direction;
  mrpt::viz::CPointCloud::Ptr gl_rel_target;
  mrpt::viz::CSetOfLines::Ptr gl_nd_gaps;

  void buildScenes();
  void defaultParams();
  void reinitSimulator();
  void simulateOneStep(double dt);
  void updateDynamicObjects();
  void updateCursor();
  void onWorldClick();
  void loadMapFile(const std::string& fil);

  void renderMainMenu();
  void renderToolbar();
  void renderPanels();
  void renderFrame();
};

HolonomicNavDemoApp::HolonomicNavDemoApp()
{
  m_about.appName = APP_NAME;
  m_about.description =
      "Interactive simulator of holonomic reactive navigation methods (VFF, ND) "
      "in a 2D occupancy grid map.";
  m_about.extraTabs.emplace_back(
      "Usage",
      "1. Place the robot and the target with the toolbar buttons (or the R and T\n"
      "   keys), then left-click on the map.\n"
      "2. Choose the holonomic method and edit its parameters in the\n"
      "   Configuration panel.\n"
      "3. Press START (Space toggles start/stop).\n"
      "\n"
      "The Local view shows the normalized sensor readings, the target and the\n"
      "direction chosen by the method (and the gaps found by ND).\n");

  // Default map:
  mrpt::io::CMemoryStream s(DEFAULT_GRIDMAP_DATA, sizeof(DEFAULT_GRIDMAP_DATA));
  mrpt::serialization::archiveFrom(s) >> m_gridMap;

  buildScenes();
  defaultParams();
}

void HolonomicNavDemoApp::defaultParams()
{
  mrpt::config::CConfigFileMemory cfg;
  m_simulOptions.saveToConfigFile(cfg, "SIMULATOR");
  CHolonomicVFF const holoVFF;
  holoVFF.options.saveToConfigFile(cfg, holoVFF.getConfigFileSectionName());
  CHolonomicND const holoND;
  holoND.options.saveToConfigFile(cfg, holoND.getConfigFileSectionName());
  m_params = cfg.getContent();
}

void HolonomicNavDemoApp::buildScenes()
{
  using namespace mrpt::viz;
  auto& scene = *m_worldScene;
  {
    auto obj = CGridPlaneXY::Create(-50, 50, -50, 50, 0, 1);
    obj->setColor_u8(TColor(30, 30, 30, 50));
    scene.insert(obj);
  }
  gl_grid = CSetOfObjects::Create();
  scene.insert(gl_grid);
  m_gridMap.getVisualizationInto(*gl_grid);

  gl_robot = CSetOfObjects::Create();
  {
    auto obj = CCylinder::Create(0.2f, 0.1f, 0.9f);
    obj->setColor_u8(TColor::red());
    gl_robot->insert(obj);
  }
  scene.insert(gl_robot);

  gl_scan3D = CPlanarLaserScan::Create();
  gl_scan3D->enableLine(false);
  gl_scan3D->setPointSize(2.0);
  gl_scan3D->setLocation(0, 0, 0.01);  // avoid z-fighting
  gl_robot->insert(gl_scan3D);

  gl_robot_sensor_range = CDisk::Create(0, 0);
  gl_robot_sensor_range->setColor_u8(TColor(0, 0, 255, 90));
  gl_robot_sensor_range->setLocation(0, 0, 0.01);
  gl_robot->insert(gl_robot_sensor_range);

  gl_robot_path = CSetOfLines::Create();
  gl_robot_path->setLineWidth(2);
  gl_robot_path->setColor_u8(TColor(40, 40, 40, 200));
  scene.insert(gl_robot_path);

  gl_target = targetMarker();
  scene.insert(gl_target);

  gl_placing_target = targetMarker();
  gl_placing_target->setVisibility(false);
  scene.insert(gl_placing_target);

  gl_placing_robot = CSetOfObjects::Create();
  {
    auto obj = CCylinder::Create(0.2f, 0.1f, 0.9f);
    obj->setColor_u8(TColor(255, 0, 0, 120));
    gl_placing_robot->insert(obj);
  }
  gl_placing_robot->setVisibility(false);
  scene.insert(gl_placing_robot);

  scene.insert(stock_objects::CornerXYZ(1));

  m_world.view.setScene(m_worldScene);
  auto& cam = m_world.view.cameraController;
  cam.setCameraPointing(0.0f, 0.0f, 0.0f);
  cam.setZoomDistance(40.0f);
  cam.setElevationDegrees(70.0f);
  cam.setAzimuthDegrees(-100.0f);

  // Local (normalized, robot-centric) view:
  auto& ls = *m_localScene;
  {
    auto obj = CGridPlaneXY::Create(-1, 1.001f, -1, 1.001f, 0, 1);
    obj->setColor_u8(TColor(200, 200, 200, 80));
    ls.insert(obj);
  }
  gl_scan2D = CPlanarLaserScan::Create();
  gl_scan2D->enableLine(false);
  gl_scan2D->enableSurface(false);
  gl_scan2D->setPointSize(3.0);
  ls.insert(gl_scan2D);

  gl_line_direction = CSimpleLine::Create();
  gl_line_direction->setLineWidth(4);
  gl_line_direction->setColor_u8(TColor(255, 255, 0));
  ls.insert(gl_line_direction);

  gl_rel_target = CPointCloud::Create();
  gl_rel_target->setPointSize(9);
  gl_rel_target->setColor_u8(TColor(0, 0, 255));
  gl_rel_target->insertPoint(0, 0, 0);
  ls.insert(gl_rel_target);

  ls.insert(stock_objects::CornerXYSimple(0.1f, 2));

  gl_nd_gaps = CSetOfLines::Create();
  gl_nd_gaps->setLineWidth(2);
  gl_nd_gaps->setColor_u8(TColor(204, 102, 51));
  ls.insert(gl_nd_gaps);

  m_localView.setScene(m_localScene);
  auto& lc = m_localView.cameraController;
  lc.setCameraPointing(0.0f, 0.0f, 0.0f);
  lc.setZoomDistance(2.2f);
  lc.setElevationDegrees(90.0f);
  lc.setAzimuthDegrees(-90.0f);
  lc.setProjectiveModel(false);

  updateDynamicObjects();
}

void HolonomicNavDemoApp::reinitSimulator()
{
  m_holonomicMethod.reset();
  if (m_holoMethod == 0)
  {
    m_holonomicMethod = std::make_unique<CHolonomicVFF>();
  }
  else
  {
    m_holonomicMethod = std::make_unique<CHolonomicND>();
  }
  mrpt::config::CConfigFileMemory cfg;
  cfg.setContent(m_params);
  m_holonomicMethod->initialize(cfg);
  m_simulOptions.loadFromConfigFile(cfg, "SIMULATOR");

  gl_robot_sensor_range->setDiskRadius(
      static_cast<float>(m_simulOptions.MAX_SENSOR_RADIUS * 1.01),
      static_cast<float>(m_simulOptions.MAX_SENSOR_RADIUS * 0.99));
}

void HolonomicNavDemoApp::simulateOneStep(double dt)
{
  // Simulate a 360deg range scan:
  CObservation2DRangeScan scan;
  scan.aperture = M_2PIf;
  scan.rightToLeft = true;
  scan.maxRange = static_cast<float>(m_simulOptions.MAX_SENSOR_RADIUS);
  scan.sensorPose = CPose3D();

  m_gridMap.laserScanSimulator(
      scan, CPose2D(m_robotPose), 0.5f, m_simulOptions.SENSOR_NUM_RANGES,
      static_cast<float>(m_simulOptions.SENSOR_RANGE_NOISE_STD));

  gl_scan3D->setScan(scan);  // real scan, in the world view

  // Normalize:
  for (size_t j = 0; j < scan.getScanSize(); j++)
  {
    scan.setScanRange(j, scan.getScanRange(j) / scan.maxRange);
  }
  gl_scan2D->setScan(scan);  // scaled scan, in the local view

  // Navigate:
  TPose2D relTargetPose = m_targetPose - m_robotPose;
  relTargetPose.x *= 1.0 / scan.maxRange;
  relTargetPose.y *= 1.0 / scan.maxRange;

  CAbstractHolonomicReactiveMethod::NavInput ni;
  ni.targets.push_back(relTargetPose);
  ni.obstacles.resize(scan.getScanSize());
  for (unsigned int i = 0; i < ni.obstacles.size(); i++)
  {
    ni.obstacles[i] = scan.getScanRange(i);
  }
  ni.maxRobotSpeed = m_simulOptions.ROBOT_MAX_SPEED;
  ni.maxObstacleDist = m_simulOptions.MAX_SENSOR_RADIUS;

  const auto no = m_holonomicMethod->navigate(ni);
  m_lastSpeed = no.desiredSpeed;
  m_lastDirDeg = RAD2DEG(no.desiredDirection);

  // Move the robot:
  m_robotPose.x += std::cos(no.desiredDirection) * no.desiredSpeed * dt;
  m_robotPose.y += std::sin(no.desiredDirection) * no.desiredSpeed * dt;

  // Path:
  const TPoint3D cur_pt(m_robotPose.x, m_robotPose.y, 0.01);
  if (gl_robot_path->empty())
  {
    gl_robot_path->appendLine(cur_pt, cur_pt);
  }
  else
  {
    gl_robot_path->appendLineStrip(cur_pt);
  }

  gl_rel_target->setLocation(relTargetPose.x, relTargetPose.y, 0);

  m_localInfo.clear();
  gl_nd_gaps->clear();
  if (auto log = std::dynamic_pointer_cast<CLogFileRecord_ND>(no.logRecord); log)
  {
    const size_t nGaps = log->gaps_ini.size();
    m_localInfo = mrpt::format(
        "ND situation: %s\nGap count: %zu",
        mrpt::typemeta::TEnumType<CHolonomicND::TSituations>::value2name(log->situation).c_str(),
        nGaps);

    gl_nd_gaps->appendLine(0, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < nGaps; i++)
    {
      const size_t N_STEPS = 20;
      for (size_t j = 0; j < N_STEPS; j++)
      {
        const double sec = log->gaps_ini[i] + static_cast<double>(j) *
                                                  (log->gaps_end[i] - log->gaps_ini[i]) /
                                                  static_cast<double>(N_STEPS - 1);
        const double ang = M_PI * (-1 + 2 * sec / static_cast<double>(scan.getScanSize()));
        const double d = scan.getScanRange(static_cast<size_t>(sec)) - 0.05;
        gl_nd_gaps->appendLineStrip(
            static_cast<float>(d * std::cos(ang)), static_cast<float>(d * std::sin(ang)), 0.0f);
      }
      gl_nd_gaps->appendLineStrip(0, 0, 0);
    }
  }

  // Movement direction:
  const double d = no.desiredSpeed / m_simulOptions.ROBOT_MAX_SPEED;
  gl_line_direction->setLineCoords(
      0, 0, 0, static_cast<float>(std::cos(no.desiredDirection) * d),
      static_cast<float>(std::sin(no.desiredDirection) * d), 0);
}

void HolonomicNavDemoApp::updateDynamicObjects()
{
  gl_robot->setLocation(m_robotPose.x, m_robotPose.y, 0);

  // Animate the target:
  {
    const double MIN = 0.7;
    const double MAX = 1;
    const double t = std::fmod(m_runtime.Tac(), 1.0);
    const double s = 4 * t * (MAX - MIN) * (1 - t) + MIN;  // parabolic bounce
    gl_target->setLocation(TPoint3D(m_targetPose.translation()));
    gl_target->setScale(static_cast<float>(s));
  }
  gl_robot_sensor_range->setVisibility(m_showMaxRange);
  gl_robot_path->setVisibility(m_showRobotPath);
}

void HolonomicNavDemoApp::updateCursor()
{
  m_cursorPos.reset();
  if (!m_world.isHovered())
  {
    gl_placing_target->setVisibility(false);
    gl_placing_robot->setVisibility(false);
    return;
  }
  const auto ray = m_world.view.mouseRay();
  if (!ray)
  {
    return;
  }
  const TPlane ground(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
  TObject3D inters;
  intersect(*ray, ground, inters);
  TPoint3D p;
  if (!inters.getPoint(p))
  {
    return;
  }
  m_cursorPos = TPoint2D(p.x, p.y);
  gl_placing_target->setVisibility(m_pickMode == PickMode::Target);
  gl_placing_robot->setVisibility(m_pickMode == PickMode::Robot);
  gl_placing_target->setLocation(p.x, p.y, 0.05);
  gl_placing_robot->setLocation(p.x, p.y, 0.05);
}

void HolonomicNavDemoApp::onWorldClick()
{
  if (!m_cursorPos || m_pickMode == PickMode::None)
  {
    return;
  }
  if (m_pickMode == PickMode::Target)
  {
    m_targetPose = {m_cursorPos->x, m_cursorPos->y, .0};
  }
  else
  {
    m_robotPose.x = m_cursorPos->x;
    m_robotPose.y = m_cursorPos->y;
  }
  m_pickMode = PickMode::None;
  gl_placing_target->setVisibility(false);
  gl_placing_robot->setVisibility(false);
}

void HolonomicNavDemoApp::loadMapFile(const std::string& fil)
{
  const std::string ext = mrpt::system::lowerCase(mrpt::system::extractFileExtension(fil, true));
  if (ext == "gridmap")
  {
    mrpt::io::CCompressedInputStream f(fil);
    mrpt::serialization::archiveFrom(f) >> m_gridMap;
    m_gridMap.getVisualizationInto(*gl_grid);
  }
  else
  {
    // An image: ask for its resolution and origin first.
    m_pendingMapImage = fil;
    m_openMapImagePopup = true;
  }
}

void HolonomicNavDemoApp::renderMainMenu()
{
  if (ImGui::BeginMenu("File"))
  {
    if (ImGui::MenuItem(ICON_MS_FOLDER_OPEN " Load map..."))
    {
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Open, "Select grid map to load",
          {
              {         "Images", {"png", "jpg", "gif", "bmp"}},
              {"Binary gridmaps",    {"gridmap", "gridmap.gz"}}
      });
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
    ImGui::MenuItem("Maximum sensor range", nullptr, &m_showMaxRange);
    ImGui::MenuItem("Robot path", nullptr, &m_showRobotPath);
    if (ImGui::MenuItem("Clear robot path"))
    {
      gl_robot_path->clear();
    }
    ImGui::Separator();
    ImGui::MenuItem("Configuration", nullptr, &m_showConfig);
    ImGui::MenuItem("Local view", nullptr, &m_showLocalView);
    ImGui::MenuItem("Stats", nullptr, &m_showStats);
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

void HolonomicNavDemoApp::renderToolbar()
{
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar;
  const float h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2;
  if (ImGui::BeginViewportSideBar("##toolbar", ImGui::GetMainViewport(), ImGuiDir_Up, h, flags))
  {
    const auto modeButton = [this](const char* label, PickMode mode, const char* tip)
    {
      const bool active = m_pickMode == mode;
      if (active)
      {
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
      }
      if (ImGui::Button(label))
      {
        m_pickMode = active ? PickMode::None : mode;
      }
      if (active)
      {
        ImGui::PopStyleColor();
      }
      tooltip(tip);
      ImGui::SameLine();
    };
    modeButton(ICON_MS_MY_LOCATION " Place robot", PickMode::Robot, "Then click on the map (R)");
    modeButton(ICON_MS_FLAG " Set target", PickMode::Target, "Then click on the map (T)");
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    if (!m_running)
    {
      if (ImGui::Button(ICON_MS_PLAY_ARROW " START"))
      {
        try
        {
          reinitSimulator();
          m_running = true;
        }
        catch (const std::exception& e)
        {
          m_msgs.error("Error in the parameters", mrpt::exception_to_str(e));
        }
      }
    }
    else if (ImGui::Button(ICON_MS_STOP " STOP"))
    {
      m_running = false;
    }
    tooltip("Start / stop the navigation (Space)");
    if (m_pickMode != PickMode::None)
    {
      ImGui::SameLine();
      ImGui::TextColored(
          ImVec4(1, 0.8f, 0.3f, 1), m_pickMode == PickMode::Robot
                                        ? "Click on the map to place the robot (Esc cancels)"
                                        : "Click on the map to set the target (Esc cancels)");
    }
  }
  ImGui::End();
}

void HolonomicNavDemoApp::renderPanels()
{
  if (m_showConfig)
  {
    if (ImGui::Begin(ICON_MS_TUNE " Configuration###Configuration", &m_showConfig))
    {
      ImGui::BeginDisabled(m_running);
      ImGui::TextUnformatted("Holonomic method:");
      ImGui::RadioButton("VFF (Virtual Force Field)", &m_holoMethod, 0);
      ImGui::RadioButton("ND (Nearness Diagram)", &m_holoMethod, 1);
      ImGui::Separator();
      ImGui::TextUnformatted("Parameters (simulator and methods):");
      ImGui::SameLine();
      if (ImGui::SmallButton("Defaults"))
      {
        defaultParams();
      }
      ImGui::InputTextMultiline(
          "##params", &m_params, ImVec2(-1, -1), ImGuiInputTextFlags_AllowTabInput);
      ImGui::EndDisabled();
    }
    ImGui::End();
  }
  if (m_showLocalView)
  {
    if (ImGui::Begin(ICON_MS_RADAR " Local view###Local view", &m_showLocalView))
    {
      m_localView.render();
    }
    ImGui::End();
  }
  if (m_showStats)
  {
    if (ImGui::Begin(ICON_MS_QUERY_STATS " Stats###Stats", &m_showStats))
    {
      mrpt::imgui::keyValueTable(
          "##stats",
          {
              {             "Robot",mrpt::format("(%.03f, %.03f)",                                                    m_robotPose.x,                                 m_robotPose.y)                                    },
              {            "Target",                               mrpt::format("(%.03f, %.03f)",                                                   m_targetPose.x,                                m_targetPose.y)},
              {"Distance to target",
               mrpt::format(
               "%.03f m", (m_targetPose.translation() - m_robotPose.translation()).norm())                                               },
              {             "Speed",                                    mrpt::format("%.03f m/s",                                                     m_lastSpeed)                                               },
              {         "Direction",                                    mrpt::format("%.01f deg",                                                    m_lastDirDeg)                                               },
              {            "Cursor", m_cursorPos
 ? mrpt::format("(%.03f, %.03f)",                                                   m_cursorPos->x, m_cursorPos->y)
 : std::string("-")                                                                       },
      });
      if (!m_localInfo.empty())
      {
        ImGui::Separator();
        ImGui::TextUnformatted(m_localInfo.c_str());
      }
    }
    ImGui::End();
  }

  // Popup: grid parameters of a map image
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
      else if (!m_gridMap.loadFromBitmap(
                   img, std::max(1e-4f, m_mapCellSize),
                   {static_cast<double>(m_mapCenter[0]), static_cast<double>(m_mapCenter[1])}))
      {
        m_msgs.error("Error", "Can't load the image file into the gridmap.");
      }
      else
      {
        m_gridMap.getVisualizationInto(*gl_grid);
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

void HolonomicNavDemoApp::renderFrame()
{
  if (auto files = m_fileDlg.poll(); files && !files->empty())
  {
    try
    {
      loadMapFile(files->front());
    }
    catch (const std::exception& e)
    {
      m_msgs.error("Error loading the map", mrpt::exception_to_str(e));
    }
  }

  const ImGuiIO& io = ImGui::GetIO();
  if (!io.WantTextInput && !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId))
  {
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
      m_about.open();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_R, false))
    {
      m_pickMode = PickMode::Robot;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_T, false))
    {
      m_pickMode = PickMode::Target;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
    {
      m_pickMode = PickMode::None;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
      if (m_running)
      {
        m_running = false;
      }
      else
      {
        try
        {
          reinitSimulator();
          m_running = true;
        }
        catch (const std::exception& e)
        {
          m_msgs.error("Error in the parameters", mrpt::exception_to_str(e));
        }
      }
    }
  }

  if (m_running)
  {
    try
    {
      // Fixed time step, as the original 10 ms timer, but capped:
      simulateOneStep(std::min(0.05, static_cast<double>(io.DeltaTime)));
    }
    catch (const std::exception& e)
    {
      m_running = false;
      m_msgs.error("Exception", mrpt::exception_to_str(e));
    }
  }
  updateDynamicObjects();

  m_world.render(m_app);
  updateCursor();
  renderPanels();
  m_about.render();
  m_msgs.render();
}

int HolonomicNavDemoApp::run(bool startNow)
{
  if (!m_app.init(APP_NAME))
  {
    std::cerr << "Could not create the application window.\n";
    return 1;
  }
  m_app.mainMenuBar = [this]() { renderMainMenu(); };
  m_app.beforeDockspace = [this]()
  {
    renderToolbar();
    if (mrpt::imgui::beginStatusBar())
    {
      ImGui::Text(
          "Robot: (%.03f, %.03f)   Target: (%.03f, %.03f)", m_robotPose.x, m_robotPose.y,
          m_targetPose.x, m_targetPose.y);
      if (m_cursorPos)
      {
        ImGui::SameLine(0, ImGui::GetFontSize() * 2);
        ImGui::TextDisabled("X=%.03f Y=%.03f Z=0", m_cursorPos->x, m_cursorPos->y);
      }
    }
    mrpt::imgui::endStatusBar();
  };
  m_app.setupDefaultLayout = [](unsigned int dockspaceId)
  {
    ImGuiID right = 0;
    ImGuiID rest = 0;
    ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Right, 0.30f, &right, &rest);
    ImGuiID rightBottom = 0;
    ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.55f, &rightBottom, &right);
    ImGui::DockBuilderDockWindow("Local view", right);
    ImGui::DockBuilderDockWindow("Configuration", rightBottom);
    ImGui::DockBuilderDockWindow("Stats", rightBottom);
  };
  m_world.view.onLeftClick = [this](float, float) { onWorldClick(); };

  if (startNow)
  {
    reinitSimulator();
    m_running = true;
  }

  m_app.run([this]() { renderFrame(); });
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli("holonomic-navigator-demo: simulator of holonomic reactive navigation methods");
    bool start = false;
    cli.add_flag("-s,--start", start, "Start navigating at once, with the default settings");
    CLI11_PARSE(cli, argc, argv);
    HolonomicNavDemoApp app;
    return app.run(start);
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
