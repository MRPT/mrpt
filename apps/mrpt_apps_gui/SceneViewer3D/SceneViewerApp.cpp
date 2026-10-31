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

#include "SceneViewerApp.h"

#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API
#include <imgui_stdlib.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/imgui/config_paths.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/stock_objects.h>
#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using mrpt::imgui::CImGuiFileDialog;
using mrpt::imgui::tooltip;

namespace
{
constexpr const char* APP_NAME = "SceneViewer3D";
constexpr const char* CFG_SECT = "SceneViewer3D";
constexpr const char* PANEL_VIEW = ICON_MS_TUNE " View###View";
constexpr const char* PANEL_TRAVELLING = ICON_MS_VIDEOCAM " Travelling###Travelling";
constexpr const char* PANEL_CONSOLE = ICON_MS_TERMINAL " Console###Console";

// Tags to know what a file dialog result is for:
constexpr const char* TAG_OPEN = "open";
constexpr const char* TAG_SAVE = "save";
constexpr const char* TAG_IMPORT_MODEL = "import_model";
constexpr const char* TAG_IMPORT_PLY = "import_ply";
constexpr const char* TAG_IMPORT_IMAGE = "import_image";
constexpr const char* TAG_EXPORT_PLY = "export_ply";
constexpr const char* TAG_SNAPSHOT = "snapshot";
constexpr const char* TAG_HIGHRES = "highres";
constexpr const char* TAG_CAPTURE_DIR = "capture_dir";
constexpr const char* TAG_PATH_LOAD = "path_load";
constexpr const char* TAG_PATH_SAVE = "path_save";

const std::vector<mrpt::imgui::FileDialogFilter> SCENE_FILTERS = {
    {"3D scenes", {"3Dscene", "3Dscene.gz"}},
};

const std::vector<mrpt::imgui::FileDialogFilter> MODEL_FILTERS = {
    {"3D models (Assimp)",
     {"dae", "blend", "3ds",     "ase",  "obj", "ifc", "xgl",  "zgl", "ply", "dxf",
     "lwo", "lws",   "lxo",     "stl",  "x",   "ac",  "ms3d", "cob", "scn", "bvh",
     "csm", "xml",   "irrmesh", "irr",  "mdl", "md2", "md3",  "pk3", "mdc", "md5mesh",
     "smd", "vta",   "m3",      "3d",   "b3d", "q3d", "q3s",  "nff", "off", "raw",
     "ter", "hmp",   "ndo",     "gltf", "glb", "fbx"}},
};

const std::vector<mrpt::imgui::FileDialogFilter> IMAGE_FILTERS = {
    {"Images", {"png", "jpg", "jpeg", "bmp", "tif", "tiff"}},
};

const char* KEYBOARD_HELP =
    "Keyboard:\n"
    "  Left / Right, F7 / F8 : Previous / next file in the same directory\n"
    "  F5                    : Reload the file\n"
    "  F2                    : Save a snapshot of the 3D view\n"
    "  F1                    : About\n"
    "  Ctrl+N / Ctrl+O / Ctrl+S : New / open / save scene\n"
    "\n"
    "Mouse (over the 3D view):\n"
    "  Left button + drag          : Orbit the camera\n"
    "  Right button + drag         : Pan\n"
    "  Middle button + drag        : Pan\n"
    "  Wheel                       : Zoom in / out\n"
    "  Shift + Left button + drag  : Zoom (vertical) / move along Z\n"
    "  Ctrl + Left button + drag   : Look around\n";

std::time_t fileMTime(const std::string& f)
{
  struct stat st
  {
  };
  return ::stat(f.c_str(), &st) == 0 ? st.st_mtime : 0;
}

/** Shortest signed angle difference b - a, in degrees, in (-180,180]. */
float angleDiffDeg(float a, float b)
{
  float d = std::fmod(b - a, 360.0f);
  if (d > 180.0f)
  {
    d -= 360.0f;
  }
  if (d <= -180.0f)
  {
    d += 360.0f;
  }
  return d;
}

/** Loads a .3Dscene file into a new scene. */
mrpt::viz::Scene::Ptr loadSceneFile(const std::string& file)
{
  mrpt::io::CCompressedInputStream f(file);
  auto scene = mrpt::viz::Scene::Create();
  mrpt::serialization::archiveFrom(f) >> *scene;
  return scene;
}
}  // namespace

// ---------------------------------------------------------------------------
// Camera path helpers
// ---------------------------------------------------------------------------
CameraKeyframe interpolateCameraPath(const std::vector<CameraKeyframe>& path, double t)
{
  if (path.empty())
  {
    return {};
  }
  if (t <= path.front().t)
  {
    return path.front();
  }
  if (t >= path.back().t)
  {
    return path.back();
  }
  const auto it = std::upper_bound(
      path.begin(), path.end(), t, [](double tt, const CameraKeyframe& k) { return tt < k.t; });
  const auto& b = *it;
  const auto& a = *(it - 1);
  const double span = b.t - a.t;
  const auto u = static_cast<float>(span > 0 ? (t - a.t) / span : 0.0);
  const auto lerp = [u](float p, float q) { return p + (q - p) * u; };
  CameraKeyframe k;
  k.t = t;
  k.x = lerp(a.x, b.x);
  k.y = lerp(a.y, b.y);
  k.z = lerp(a.z, b.z);
  k.zoom = lerp(a.zoom, b.zoom);
  k.elevationDeg = lerp(a.elevationDeg, b.elevationDeg);
  k.azimuthDeg = a.azimuthDeg + angleDiffDeg(a.azimuthDeg, b.azimuthDeg) * u;
  return k;
}

void retimeCameraPathConstantSpeed(std::vector<CameraKeyframe>& path, double speed)
{
  if (path.empty() || speed <= 0)
  {
    return;
  }
  path.front().t = 0;
  for (size_t i = 1; i < path.size(); i++)
  {
    const auto& a = path[i - 1];
    auto& b = path[i];
    const double d = std::sqrt(
        (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
    // Keyframes at the same place (only rotating) still take one second:
    b.t = a.t + (d > 1e-9 ? d / speed : 1.0);
  }
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------
SceneViewerApp::SceneViewerApp()
{
  const auto dir = mrpt::imgui::userConfigDirectory(APP_NAME);
  m_cfg = std::make_unique<mrpt::config::CConfigFile>(dir.empty() ? "" : dir + "config.ini");
  m_about.appName = APP_NAME;
  m_about.description = "Viewer of 3D scenes (.3Dscene files), 3D models and point clouds.";
  m_about.extraTabs.emplace_back("Keyboard and mouse", KEYBOARD_HELP);
  m_view.view.cameraController.setZoomLimits(0.1f, 32000.0f);
}

void SceneViewerApp::loadSettings()
{
  auto& c = *m_cfg;
  m_showFileNameInViewport = c.read_bool(CFG_SECT, "showFileNameInViewport", false);
  m_freeCameraAlways = c.read_bool(CFG_SECT, "freeCameraAlways", false);
  m_freeCameraAlwaysNoAzimuth = c.read_bool(CFG_SECT, "freeCameraAlwaysNoAzimuth", false);
  m_autoplayDelayMs = c.read_int(CFG_SECT, "autoplayDelayMs", 5);
  m_shadows = c.read_bool(CFG_SECT, "shadows", true);
  m_ssao = c.read_bool(CFG_SECT, "ssao", false);
  m_watchFile = c.read_bool(CFG_SECT, "watchFile", true);
  m_showViewPanel = c.read_bool(CFG_SECT, "showViewPanel", true);
  m_showTravellingPanel = c.read_bool(CFG_SECT, "showTravellingPanel", true);
  m_showConsole = c.read_bool(CFG_SECT, "showConsole", false);
  m_circSpeedDegPerSec = c.read_float(CFG_SECT, "travellingSpeed", 5.0f);
}

void SceneViewerApp::saveSettings()
{
  auto& c = *m_cfg;
  c.write(CFG_SECT, "showFileNameInViewport", m_showFileNameInViewport);
  c.write(CFG_SECT, "freeCameraAlways", m_freeCameraAlways);
  c.write(CFG_SECT, "freeCameraAlwaysNoAzimuth", m_freeCameraAlwaysNoAzimuth);
  c.write(CFG_SECT, "autoplayDelayMs", m_autoplayDelayMs);
  c.write(CFG_SECT, "shadows", m_shadows);
  c.write(CFG_SECT, "ssao", m_ssao);
  c.write(CFG_SECT, "watchFile", m_watchFile);
  c.write(CFG_SECT, "showViewPanel", m_showViewPanel);
  c.write(CFG_SECT, "showTravellingPanel", m_showTravellingPanel);
  c.write(CFG_SECT, "showConsole", m_showConsole);
  c.write(CFG_SECT, "travellingSpeed", m_circSpeedDegPerSec);
}

std::string SceneViewerApp::lastDir() const { return m_cfg->read_string(CFG_SECT, "LastDir", ""); }

void SceneViewerApp::setLastDir(const std::string& file)
{
  m_cfg->write(CFG_SECT, "LastDir", mrpt::system::extractFileDirectory(file));
}

// ---------------------------------------------------------------------------
// Scene management
// ---------------------------------------------------------------------------
void SceneViewerApp::applyViewportOptions()
{
  if (!m_scene)
  {
    return;
  }
  if (auto vp = m_scene->getViewport(); vp)
  {
    vp->enableShadowCasting(m_shadows);
    vp->lightParameters().ssao_enabled = m_ssao;
    vp->clearTextMessages();
    if (m_showFileNameInViewport && !m_loadedFileName.empty())
    {
      vp->addTextMessage(20, 20, mrpt::system::extractFileName(m_loadedFileName));
    }
  }
  m_view.view.cameraController.setProjectiveModel(!m_ortho);
}

void SceneViewerApp::newScene()
{
  using namespace mrpt::viz;
  using mrpt::math::TPoint3D;

  m_scene = Scene::Create();
  m_selected.clear();

  {
    auto obj = CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
    obj->setColor(0.4f, 0.4f, 0.4f);
    m_scene->insert(obj);
  }
  {
    // Solid ground slab, so shadows are visible on it:
    auto ground = CBox::Create(TPoint3D(-20, -20, -0.05), TPoint3D(20, 20, 0.0));
    ground->setColor(0.6f, 0.6f, 0.6f);
    m_scene->insert(ground);
  }
  m_scene->insert(stock_objects::CornerXYZ());
  {
    auto box = CBox::Create(TPoint3D(-0.5, -0.5, 0.0), TPoint3D(0.5, 0.5, 1.0));
    box->setPose(mrpt::poses::CPose3D::FromTranslation(2.0, 2.0, 0.0));
    box->setColor(0.8f, 0.5f, 0.2f);
    m_scene->insert(box);
  }
  {
    // A clone viewport in a corner, showing the same scene and camera:
    auto vi = m_scene->createViewport("clone");
    vi->setViewportPosition(0.05, 0.05, 0.2, 0.2);
    vi->setCloneView("main");
    vi->setCloneCamera(true);
    vi->setBorderSize(1);
  }

  auto& cam = m_view.view.cameraController;
  cam.setCameraPointing(0.0f, 0.0f, 0.0f);
  cam.setZoomDistance(20.0f);
  cam.setElevationDegrees(45.0f);
  cam.setAzimuthDegrees(45.0f);

  m_loadedFileName = "no-name.3Dscene";
  m_loadedFileMTime = 0;
  m_view.view.setScene(m_scene);
  applyViewportOptions();
}

void SceneViewerApp::startLoadSceneFile(const std::string& file, bool isInASequence)
{
  setLastDir(file);
  m_statusMsg = "Loading " + mrpt::system::extractFileName(file) + "...";
  m_loader.start(
      [file, isInASequence]()
      {
        LoadResult r;
        mrpt::system::CTicTac const tic;
        r.scene = loadSceneFile(file);
        r.seconds = tic.Tac();
        r.fileName = file;
        r.isInASequence = isInASequence;
        return r;
      });
}

void SceneViewerApp::startImportModel(const std::string& file)
{
  setLastDir(file);
  m_statusMsg = "Importing " + mrpt::system::extractFileName(file) + "...";
  m_loader.start(
      [file]()
      {
        LoadResult r;
        mrpt::system::CTicTac const tic;
        auto obj = mrpt::viz::CAssimpModel::Create();
        obj->loadScene(file);
        r.object = obj;
        r.seconds = tic.Tac();
        r.fileName = file;
        r.keepCamera = true;
        return r;
      });
}

void SceneViewerApp::startImportPly(const std::string& file)
{
  setLastDir(file);
  m_statusMsg = "Importing " + mrpt::system::extractFileName(file) + "...";
  const int cls = m_plyClass;
  const int colorFrom = m_plyColorFrom;
  const float ptSize = m_plyPointSize;
  const bool grid = m_plyGrid;
  const bool corner = m_plyCorner;
  const mrpt::math::TPose3D pose(
      0, 0, 0, mrpt::DEG2RAD(m_plyYPR[0]), mrpt::DEG2RAD(m_plyYPR[1]), mrpt::DEG2RAD(m_plyYPR[2]));

  m_loader.start(
      [=]()
      {
        using namespace mrpt::viz;
        LoadResult r;
        mrpt::system::CTicTac const tic;

        CPointCloud::Ptr pts;
        CPointCloudColoured::Ptr ptsCol;
        PLY_Importer* ply = nullptr;
        if (cls == 0)
        {
          pts = CPointCloud::Create();
          ply = pts.get();
        }
        else
        {
          ptsCol = CPointCloudColoured::Create();
          ply = ptsCol.get();
        }
        std::vector<std::string> comments;
        std::vector<std::string> info;
        if (!ply->loadFromPlyFile(file, &comments, &info))
        {
          THROW_EXCEPTION_FMT(
              "Error loading or parsing the PLY file:\n%s", ply->getLoadPLYErrorString().c_str());
        }

        auto scene = Scene::Create();
        if (grid)
        {
          auto obj = CGridPlaneXY::Create(-50, 50, -50, 50, 0, 1);
          obj->setColor(0.3f, 0.3f, 0.3f);
          scene->insert(obj);
        }
        if (corner)
        {
          scene->insert(stock_objects::CornerXYZ());
        }
        if (pts)
        {
          pts->setPointSize(ptSize);
          if (colorFrom == 1)
          {
            pts->enableColorFromX();
          }
          else if (colorFrom == 2)
          {
            pts->enableColorFromY();
          }
          else if (colorFrom == 3)
          {
            pts->enableColorFromZ();
          }
          pts->setPose(mrpt::poses::CPose3D(pose));
          scene->insert(pts);
        }
        if (ptsCol)
        {
          ptsCol->setPointSize(ptSize);
          ptsCol->setPose(mrpt::poses::CPose3D(pose));
          scene->insert(ptsCol);
        }

        std::string sC;
        std::string sI;
        mrpt::system::stringListAsString(comments, sC);
        mrpt::system::stringListAsString(info, sI);
        r.info = "Comments:\n" + sC + "\nObject info:\n" + sI;
        r.scene = scene;
        r.fileName = "Imported_" + mrpt::system::extractFileName(file) + ".3Dscene";
        r.seconds = tic.Tac();
        return r;
      });
}

void SceneViewerApp::applyLoadResult(LoadResult r)
{
  auto& cam = m_view.view.cameraController;

  if (r.object)
  {
    if (!m_scene)
    {
      newScene();
    }
    m_scene->insert(r.object);
    m_statusMsg = mrpt::format(
        "Imported %s in %.03f s", mrpt::system::extractFileName(r.fileName).c_str(), r.seconds);
    m_console.push(mrpt::system::LVL_INFO, APP_NAME, m_statusMsg);
    return;
  }

  ASSERT_(r.scene);
  auto& scene = r.scene;

  const bool isSceneFile = mrpt::system::fileExists(r.fileName);
  if (isSceneFile && (scene->viewportsCount() == 0 || !scene->getViewport("main") ||
                      scene->getViewport("main")->size() == 0))
  {
    m_msgs.warning("Warning", "File is empty or format unrecognized:\n" + r.fileName);
    m_autoplay = false;
  }

  // Guess the clip distances from the scene size:
  if (auto vp = scene->getViewport(); vp)
  {
    const auto bbox = scene->getBoundingBox();
    const double maxScale = std::max<double>(1000.0, (bbox.max - bbox.min).norm());
    vp->setViewportClipDistances(0.1f, static_cast<float>(5.0 * maxScale));
  }

  if (scene->followCamera())
  {
    auto view = scene->getViewport("main");
    ASSERTMSG_(view, "There is no 'main' viewport in the 3D scene!");

    mrpt::viz::CCamera::Ptr sceneCam = scene->getByClass<mrpt::viz::CCamera>();
    const bool camIsObject = sceneCam != nullptr;
    if (!camIsObject)
    {
      sceneCam = mrpt::viz::CCamera::Create(view->getCamera());
    }

    cam.setCameraPointing(
        sceneCam->getPointingAtX(), sceneCam->getPointingAtY(), sceneCam->getPointingAtZ());

    // Take the camera of the file, unless moving through a sequence with
    // "keep the camera" (all of it, or only zoom and elevation):
    const bool keepCamera = r.isInASequence && m_freeCameraAlways;
    if (!keepCamera)
    {
      cam.setZoomDistance(sceneCam->getZoomDistance());
      cam.setElevationDegrees(sceneCam->getElevationDegrees());
    }
    if (!keepCamera || m_freeCameraAlwaysNoAzimuth)
    {
      cam.setAzimuthDegrees(sceneCam->getAzimuthDegrees());
    }
    if (camIsObject)
    {
      scene->removeObject(sceneCam);
    }
  }
  else if (!isSceneFile)
  {
    // Imported point cloud: frame it.
    cam.setCameraPointing(0.0f, 0.0f, 0.0f);
    cam.setZoomDistance(4.0f);
    cam.setAzimuthDegrees(45.0f);
    cam.setElevationDegrees(30.0f);
  }

  m_scene = scene;
  m_selected.clear();
  m_loadedFileName = r.fileName;
  m_loadedFileMTime = isSceneFile ? fileMTime(r.fileName) : 0;
  m_view.view.setScene(m_scene);
  applyViewportOptions();

  m_statusMsg = mrpt::format(
      "%s loaded in %.03f s", mrpt::system::extractFileName(r.fileName).c_str(), r.seconds);
  m_console.push(mrpt::system::LVL_INFO, APP_NAME, m_statusMsg);
  if (!r.info.empty())
  {
    m_msgs.info("File info", r.info);
  }
}

void SceneViewerApp::reload()
{
  if (mrpt::system::fileExists(m_loadedFileName))
  {
    startLoadSceneFile(m_loadedFileName, true);
  }
}

void SceneViewerApp::saveScene(const std::string& file)
{
  setLastDir(file);
  mrpt::io::CCompressedOutputStream fo(file);
  mrpt::serialization::archiveFrom(fo) << *m_scene;
  m_statusMsg = "Saved: " + file;
}

void SceneViewerApp::importImage(const std::string& file)
{
  setLastDir(file);
  mrpt::img::CImage im;
  if (!im.loadFromFile(file))
  {
    THROW_EXCEPTION_FMT("Error loading image file: '%s'", file.c_str());
  }
  m_scene = mrpt::viz::Scene::Create();
  m_scene->getViewport()->setImageView(im);
  m_view.view.setScene(m_scene);
  m_loadedFileName = file;
  m_loadedFileMTime = 0;
}

void SceneViewerApp::exportPointCloudsToPly(const std::string& file)
{
  using namespace mrpt::viz;
  setLastDir(file);
  const std::string base =
      mrpt::system::extractFileExtension(file) == "ply" ? file.substr(0, file.size() - 4) : file;
  unsigned int count = 0;
  m_scene->visitAllObjects(
      [&](const CVisualObject::Ptr& obj)
      {
        if (auto pc = std::dynamic_pointer_cast<CPointCloud>(obj); pc)
        {
          pc->saveToPlyFile(mrpt::format("%s_%03u.ply", base.c_str(), ++count));
        }
        else if (auto pcc = std::dynamic_pointer_cast<CPointCloudColoured>(obj); pcc)
        {
          pcc->saveToPlyFile(mrpt::format("%s_%03u.ply", base.c_str(), ++count));
        }
      });
  m_msgs.info(
      "Export to PLY",
      mrpt::format(
          "%u point cloud(s) exported to PLY files named:\n%s_NNN.ply", count, base.c_str()));
}

void SceneViewerApp::highResRender(const std::string& file, int w, int h)
{
  setLastDir(file);
  mrpt::opengl::CFBORender::Parameters params;
  params.width = static_cast<unsigned int>(w);
  params.height = static_cast<unsigned int>(h);
  params.create_EGL_context = false;  // reuse the window GL context

  mrpt::opengl::CFBORender render(params);
  mrpt::viz::CCamera cam;
  m_view.view.cameraController.applyTo(cam);
  render.setCamera(cam);
  mrpt::img::CImage frame(w, h, mrpt::img::CH_RGB);
  render.render_RGB(*m_scene, frame);
  if (!frame.saveToFile(file))
  {
    THROW_EXCEPTION_FMT("Could not save image to: '%s'", file.c_str());
  }
  m_statusMsg = mrpt::format("Rendered %dx%d image to: %s", w, h, file.c_str());
}

void SceneViewerApp::saveCroppedSceneImage(const mrpt::img::CImage& frame, const std::string& file)
{
  // Crop the window image to the visible 3D view (the dockspace central
  // node), so panels do not appear in the image:
  const auto r = m_app.centralNodeRect();
  const auto s = m_app.framebufferScale();
  const auto clampTo = [](float v, float hi) { return static_cast<int>(std::clamp(v, 0.0f, hi)); };
  const auto W = static_cast<float>(frame.getWidth());
  const auto H = static_cast<float>(frame.getHeight());
  const ImVec2 vpPos = ImGui::GetMainViewport()->Pos;
  const int x0 = clampTo((r[0] - vpPos.x) * s[0], W);
  const int y0 = clampTo((r[1] - vpPos.y) * s[1], H);
  const int x1 = clampTo((r[0] - vpPos.x + r[2]) * s[0], W);
  const int y1 = clampTo((r[1] - vpPos.y + r[3]) * s[1], H);
  mrpt::img::CImage out;
  if (x1 - x0 >= 8 && y1 - y0 >= 8)
  {
    frame.extract_patch(out, {x0, y0}, {x1 - x0, y1 - y0});
  }
  else
  {
    out = frame.makeShallowCopy();
  }
  if (!out.saveToFile(file))
  {
    m_console.push(mrpt::system::LVL_ERROR, APP_NAME, "Could not save image: " + file);
  }
}

void SceneViewerApp::loadNeighborFile(int direction)
{
  namespace fs = std::filesystem;
  if (m_loadedFileName.empty())
  {
    return;
  }
  std::error_code ec;
  const fs::path cur = fs::absolute(m_loadedFileName, ec);
  const fs::path dir = cur.parent_path();
  if (!fs::is_directory(dir, ec))
  {
    return;
  }
  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(dir, ec))
  {
    if (e.is_regular_file(ec) &&
        mrpt::system::strCmpI(
            mrpt::system::extractFileExtension(e.path().string(), true /*ignore .gz*/), "3Dscene"))
    {
      files.push_back(e.path());
    }
  }
  std::sort(files.begin(), files.end());
  const auto it = std::find(files.begin(), files.end(), cur);
  if (it == files.end())
  {
    return;
  }
  const auto idx = std::distance(files.begin(), it) + direction;
  if (idx < 0 || idx >= static_cast<std::ptrdiff_t>(files.size()))
  {
    m_autoplay = false;  // end of the sequence
    return;
  }
  startLoadSceneFile(files[static_cast<size_t>(idx)].string(), true /*sequence*/);
}

void SceneViewerApp::showStats()
{
  size_t nObjects = 0;
  size_t nPoints = 0;
  std::map<std::string, size_t> perClass;
  m_scene->visitAllObjects(
      [&](const mrpt::viz::CVisualObject::Ptr& o)
      {
        nObjects++;
        perClass[o->GetRuntimeClass()->className]++;
        if (auto pc = std::dynamic_pointer_cast<mrpt::viz::CPointCloud>(o); pc)
        {
          nPoints += pc->size();
        }
        else if (auto pcc = std::dynamic_pointer_cast<mrpt::viz::CPointCloudColoured>(o); pcc)
        {
          nPoints += pcc->size();
        }
      });
  std::ostringstream ss;
  ss << "Viewports: " << m_scene->viewportsCount() << "\n"
     << "Objects: " << nObjects << "\n"
     << "Points (in point clouds): " << nPoints << "\n\n"
     << "Objects per class:\n";
  for (const auto& [cls, n] : perClass)
  {
    ss << "  " << cls << ": " << n << "\n";
  }
  m_msgs.info("Scene statistics", ss.str());
}

// ---------------------------------------------------------------------------
// Per-frame logic
// ---------------------------------------------------------------------------
void SceneViewerApp::checkFileWatch()
{
  if (!m_watchFile || m_loadedFileMTime == 0 || m_loader.isRunning())
  {
    return;
  }
  const double now = ImGui::GetTime();
  if (now - m_lastFileWatchCheck < 1.0)
  {
    return;
  }
  m_lastFileWatchCheck = now;
  const auto mt = fileMTime(m_loadedFileName);
  if (mt != 0 && mt != m_loadedFileMTime)
  {
    m_loadedFileMTime = mt;
    m_console.push(mrpt::system::LVL_INFO, APP_NAME, "File changed on disk, reloading.");
    startLoadSceneFile(m_loadedFileName, true /*keep camera*/);
  }
}

void SceneViewerApp::updateAutoplayAndCapture()
{
  if (m_autoplay && !m_loader.isRunning())
  {
    const double now = ImGui::GetTime();
    if (now - m_lastAutoplayStep >= m_autoplayDelayMs * 1e-3)
    {
      m_lastAutoplayStep = now;
      loadNeighborFile(+1);
    }
  }
  if (m_capturing)
  {
    const std::string file =
        mrpt::format("%s/screenshot_%07i.png", m_captureDir.c_str(), m_captureCount++);
    m_app.requestFrameGrab([this, file](const mrpt::img::CImage& im)
                           { saveCroppedSceneImage(im, file); });
  }
}

void SceneViewerApp::updateTravelling(double dt)
{
  auto& cam = m_view.view.cameraController;
  switch (m_travelling)
  {
    case Travelling::Circular:
    {
      const float az = cam.getAzimuthDegrees() + static_cast<float>(m_circSpeedDegPerSec * dt);
      if (az < m_circAzimuth + m_circMaxOffset)
      {
        cam.setAzimuthDegrees(az);
      }
      else
      {
        m_travelling = Travelling::None;
      }
      break;
    }
    case Travelling::Path:
    {
      if (m_path.empty())
      {
        m_travelling = Travelling::None;
        break;
      }
      m_travellingT += dt;
      if (m_travellingT > m_path.back().t)
      {
        if (m_pathLoop)
        {
          m_travellingT = m_path.front().t;
        }
        else
        {
          m_travelling = Travelling::None;
        }
      }
      const auto k = interpolateCameraPath(m_path, m_travellingT);
      cam.setCameraPointing(k.x, k.y, k.z);
      cam.setZoomDistance(k.zoom);
      cam.setAzimuthDegrees(k.azimuthDeg);
      cam.setElevationDegrees(k.elevationDeg);
      break;
    }
    case Travelling::None:
    default:
      break;
  }
}

void SceneViewerApp::handleKeyboard()
{
  const ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId))
  {
    return;
  }
  const bool ctrl = io.KeyCtrl;
  if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
  {
    m_about.open();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F2, false))
  {
    m_fileDlg.tag = TAG_SNAPSHOT;
    m_fileDlg.open(
        CImGuiFileDialog::Mode::Save, "Save snapshot", IMAGE_FILTERS, lastDir() + "/snapshot.png");
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F5, false))
  {
    reload();
  }
  // Left/right navigate files, unless a panel with keyboard focus wants them:
  const bool navKeysFree = !io.WantCaptureKeyboard || m_view.isHovered();
  if (ImGui::IsKeyPressed(ImGuiKey_F7, false) ||
      (navKeysFree && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, false)))
  {
    loadNeighborFile(-1);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F8, false) ||
      (navKeysFree && ImGui::IsKeyPressed(ImGuiKey_RightArrow, false)))
  {
    loadNeighborFile(+1);
  }
  if (ctrl && ImGui::IsKeyPressed(ImGuiKey_N, false))
  {
    newScene();
  }
  if (ctrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
  {
    m_fileDlg.tag = TAG_OPEN;
    m_fileDlg.open(CImGuiFileDialog::Mode::Open, "Open 3D scene", SCENE_FILTERS, lastDir());
  }
  if (ctrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
  {
    m_fileDlg.tag = TAG_SAVE;
    m_fileDlg.open(CImGuiFileDialog::Mode::Save, "Save 3D scene", SCENE_FILTERS, lastDir());
  }
}

void SceneViewerApp::handleFileDialogResult(const std::vector<std::string>& files)
{
  if (files.empty())
  {
    return;  // cancelled
  }
  const std::string& f = files.front();
  const std::string& tag = m_fileDlg.tag;
  if (tag == TAG_OPEN)
  {
    startLoadSceneFile(f);
  }
  else if (tag == TAG_SAVE)
  {
    std::string out = f;
    if (mrpt::system::extractFileExtension(out, true) != "3Dscene")
    {
      out += ".3Dscene";
    }
    saveScene(out);
  }
  else if (tag == TAG_IMPORT_MODEL)
  {
    startImportModel(f);
  }
  else if (tag == TAG_IMPORT_PLY)
  {
    m_pendingPlyFile = f;
    m_openPlyPopup = true;
  }
  else if (tag == TAG_IMPORT_IMAGE)
  {
    importImage(f);
  }
  else if (tag == TAG_EXPORT_PLY)
  {
    exportPointCloudsToPly(f);
  }
  else if (tag == TAG_SNAPSHOT)
  {
    setLastDir(f);
    m_app.requestFrameGrab([this, f](const mrpt::img::CImage& im)
                           { saveCroppedSceneImage(im, f); });
  }
  else if (tag == TAG_HIGHRES)
  {
    m_pendingHighResFile = f;
    m_openHighResPopup = true;
  }
  else if (tag == TAG_CAPTURE_DIR)
  {
    m_captureDir = f;
    m_captureCount = 0;
    m_capturing = true;
  }
  else if (tag == TAG_PATH_SAVE)
  {
    std::ofstream out(f);
    out << "% t x y z zoom azimuth_deg elevation_deg\n";
    for (const auto& k : m_path)
    {
      out << k.t << " " << k.x << " " << k.y << " " << k.z << " " << k.zoom << " " << k.azimuthDeg
          << " " << k.elevationDeg << "\n";
    }
  }
  else if (tag == TAG_PATH_LOAD)
  {
    std::ifstream in(f);
    std::string line;
    std::vector<CameraKeyframe> path;
    while (std::getline(in, line))
    {
      if (line.empty() || line[0] == '%' || line[0] == '#')
      {
        continue;
      }
      std::istringstream ss(line);
      CameraKeyframe k;
      if (ss >> k.t >> k.x >> k.y >> k.z >> k.zoom >> k.azimuthDeg >> k.elevationDeg)
      {
        path.push_back(k);
      }
    }
    std::sort(
        path.begin(), path.end(),
        [](const CameraKeyframe& a, const CameraKeyframe& b) { return a.t < b.t; });
    m_path = std::move(path);
  }
}

// ---------------------------------------------------------------------------
// GUI
// ---------------------------------------------------------------------------
void SceneViewerApp::renderMainMenu()
{
  const auto openDlg = [this](
                           const char* tag, CImGuiFileDialog::Mode mode, const char* title,
                           const std::vector<mrpt::imgui::FileDialogFilter>& filters)
  {
    m_fileDlg.tag = tag;
    m_fileDlg.open(mode, title, filters, lastDir());
  };

  if (ImGui::BeginMenu("File"))
  {
    if (ImGui::MenuItem(ICON_MS_ADD " New scene", "Ctrl+N"))
    {
      newScene();
    }
    if (ImGui::MenuItem(ICON_MS_FOLDER_OPEN " Open...", "Ctrl+O"))
    {
      openDlg(TAG_OPEN, CImGuiFileDialog::Mode::Open, "Open 3D scene", SCENE_FILTERS);
    }
    if (ImGui::MenuItem(ICON_MS_REFRESH " Reload", "F5"))
    {
      reload();
    }
    if (ImGui::MenuItem(ICON_MS_SAVE " Save...", "Ctrl+S"))
    {
      openDlg(TAG_SAVE, CImGuiFileDialog::Mode::Save, "Save 3D scene", SCENE_FILTERS);
    }
    ImGui::Separator();
    if (ImGui::BeginMenu(ICON_MS_UPLOAD_FILE " Import"))
    {
      if (ImGui::MenuItem("3D model (any Assimp format)..."))
      {
        openDlg(TAG_IMPORT_MODEL, CImGuiFileDialog::Mode::Open, "Import 3D model", MODEL_FILTERS);
      }
      if (ImGui::MenuItem("PLY point cloud..."))
      {
        openDlg(
            TAG_IMPORT_PLY, CImGuiFileDialog::Mode::Open, "Import PLY point cloud",
            {
                {"PLY files", {"ply"}}
        });
      }
      if (ImGui::MenuItem("Image..."))
      {
        openDlg(TAG_IMPORT_IMAGE, CImGuiFileDialog::Mode::Open, "Import image", IMAGE_FILTERS);
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(ICON_MS_FILE_EXPORT " Export"))
    {
      if (ImGui::MenuItem("Point clouds to PLY files..."))
      {
        openDlg(
            TAG_EXPORT_PLY, CImGuiFileDialog::Mode::Save, "Export point clouds (one PLY file each)",
            {
                {"PLY files", {"ply"}}
        });
      }
      ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_ARROW_BACK " Previous file", "F7"))
    {
      loadNeighborFile(-1);
    }
    if (ImGui::MenuItem(ICON_MS_ARROW_FORWARD " Next file", "F8"))
    {
      loadNeighborFile(+1);
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_PHOTO_CAMERA " Take snapshot...", "F2"))
    {
      m_fileDlg.tag = TAG_SNAPSHOT;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Save snapshot", IMAGE_FILTERS,
          lastDir() + "/snapshot.png");
    }
    if (ImGui::MenuItem(ICON_MS_SCREENSHOT_MONITOR " High-resolution render..."))
    {
      m_fileDlg.tag = TAG_HIGHRES;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "High-resolution render to file", IMAGE_FILTERS,
          lastDir() + "/render.png");
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_QUERY_STATS " Scene statistics"))
    {
      showStats();
    }
    if (ImGui::MenuItem(ICON_MS_TERMINAL " Print scene as text to console"))
    {
      std::ostringstream ss;
      m_scene->asYAML().printAsYAML(ss);
      std::cout << ss.str();
      m_console.push(mrpt::system::LVL_INFO, APP_NAME, ss.str());
      m_showConsole = true;
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_CLOSE " Quit", "Alt+F4"))
    {
      m_app.requestClose();
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Edit"))
  {
    ImGui::TextDisabled("%zu objects selected", m_selected.size());
    if (ImGui::MenuItem("Select none"))
    {
      m_selected.clear();
    }
    if (ImGui::MenuItem("Select by class..."))
    {
      m_openSelectByClassPopup = true;
    }
    if (ImGui::MenuItem("Re-scale selected...", nullptr, false, !m_selected.empty()))
    {
      m_openScalePopup = true;
    }
    if (ImGui::MenuItem(ICON_MS_DELETE " Delete selected", nullptr, false, !m_selected.empty()))
    {
      for (const auto& o : m_selected)
      {
        m_scene->removeObject(o);
      }
      m_selected.clear();
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Tools"))
  {
    if (ImGui::MenuItem(ICON_MS_DELETE " Delete all objects"))
    {
      m_scene->clear();
      m_selected.clear();
    }
    if (ImGui::BeginMenu("Insert stock object"))
    {
      if (ImGui::MenuItem("Simple robot model"))
      {
        m_scene->insert(mrpt::viz::stock_objects::RobotPioneer());
      }
      if (ImGui::MenuItem("XYZ corner"))
      {
        m_scene->insert(mrpt::viz::stock_objects::CornerXYZ());
      }
      ImGui::EndMenu();
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("View"))
  {
    ImGui::MenuItem(PANEL_VIEW, nullptr, &m_showViewPanel);
    ImGui::MenuItem(PANEL_TRAVELLING, nullptr, &m_showTravellingPanel);
    ImGui::MenuItem(PANEL_CONSOLE, nullptr, &m_showConsole);
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

void SceneViewerApp::renderToolbar()
{
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar;
  const float h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2;
  if (ImGui::BeginViewportSideBar("##toolbar", ImGui::GetMainViewport(), ImGuiDir_Up, h, flags))
  {
    const auto button = [](const char* label, const char* tip)
    {
      const bool r = ImGui::Button(label);
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

    if (button(ICON_MS_ADD " New", "New scene (Ctrl+N)"))
    {
      newScene();
    }
    if (button(ICON_MS_FOLDER_OPEN " Open", "Open a .3Dscene file (Ctrl+O)"))
    {
      m_fileDlg.tag = TAG_OPEN;
      m_fileDlg.open(CImGuiFileDialog::Mode::Open, "Open 3D scene", SCENE_FILTERS, lastDir());
    }
    if (button(ICON_MS_ARROW_BACK, "Previous file in the directory (Left, F7)"))
    {
      loadNeighborFile(-1);
    }
    if (button(ICON_MS_ARROW_FORWARD, "Next file in the directory (Right, F8)"))
    {
      loadNeighborFile(+1);
    }
    if (button(ICON_MS_REFRESH, "Reload (F5)"))
    {
      reload();
    }
    sep();
    if (toggle("Ortho", &m_ortho, "Orthographic projection"))
    {
      applyViewportOptions();
    }
    if (toggle("Shadows", &m_shadows, "Shadow casting"))
    {
      applyViewportOptions();
    }
    if (toggle("SSAO", &m_ssao, "Screen-space ambient occlusion"))
    {
      applyViewportOptions();
    }
    sep();
    if (toggle(ICON_MS_PLAY_ARROW " Autoplay", &m_autoplay, "Load the next file periodically"))
    {
      m_lastAutoplayStep = ImGui::GetTime();
    }
    {
      bool capturing = m_capturing;
      if (toggle(
              ICON_MS_FIBER_MANUAL_RECORD " Capture", &capturing,
              "Save every frame of the 3D view as an image, to a directory"))
      {
        if (capturing)
        {
          m_fileDlg.tag = TAG_CAPTURE_DIR;
          m_fileDlg.open(
              CImGuiFileDialog::Mode::SelectFolder,
              "Choose the directory where to save the screenshots", {}, lastDir());
        }
        else
        {
          m_capturing = false;
        }
      }
    }
    if (button(ICON_MS_PHOTO_CAMERA, "Take snapshot of the 3D view (F2)"))
    {
      m_fileDlg.tag = TAG_SNAPSHOT;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Save snapshot", IMAGE_FILTERS,
          lastDir() + "/snapshot.png");
    }
    sep();
    if (button(ICON_MS_INFO, "About (F1)"))
    {
      m_about.open();
    }
    if (m_loader.isRunning())
    {
      ImGui::TextUnformatted(ICON_MS_SCHEDULE " Loading...");
    }
  }
  ImGui::End();
}

void SceneViewerApp::renderStatusBar()
{
  if (mrpt::imgui::beginStatusBar())
  {
    const auto& cam = m_view.view.cameraController;
    ImGui::TextUnformatted(m_statusMsg.c_str());
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.35f);
    ImGui::Text(
        "Center=(%.02f,%.02f,%.02f) Zoom:%.02f AZ=%.02f EL=%.02f deg", cam.getCameraPointingX(),
        cam.getCameraPointingY(), cam.getCameraPointingZ(), cam.getZoomDistance(),
        cam.getAzimuthDegrees(), cam.getElevationDegrees());
    ImGui::SameLine();
    ImGui::TextDisabled(
        "| %.01f FPS | %zu viewports", m_fps, m_scene ? m_scene->viewportsCount() : size_t(0));
    if (m_capturing)
    {
      ImGui::SameLine();
      ImGui::TextColored(
          ImVec4(1, 0.4f, 0.4f, 1), ICON_MS_STOP_CIRCLE " Capturing (%d)", m_captureCount);
    }
  }
  mrpt::imgui::endStatusBar();
}

void SceneViewerApp::renderViewPanel()
{
  if (!m_showViewPanel)
  {
    return;
  }
  if (!ImGui::Begin(PANEL_VIEW, &m_showViewPanel))
  {
    ImGui::End();
    return;
  }
  ImGui::PushItemWidth(-ImGui::GetFontSize() * 9);
  auto& cam = m_view.view.cameraController;

  if (ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen))
  {
    bool changed = false;
    changed |= ImGui::Checkbox("Orthographic projection", &m_ortho);
    changed |= ImGui::Checkbox("Shadows", &m_shadows);
    changed |= ImGui::Checkbox("Ambient occlusion (SSAO)", &m_ssao);
    changed |= ImGui::Checkbox("Show file name in the view", &m_showFileNameInViewport);
    if (changed)
    {
      applyViewportOptions();
    }
    if (m_scene)
    {
      if (auto vp = m_scene->getViewport(); vp)
      {
        const auto c = vp->getCustomBackgroundColor();
        float col[3] = {c.R, c.G, c.B};
        if (ImGui::ColorEdit3("Background", col))
        {
          vp->setCustomBackgroundColor({col[0], col[1], col[2]});
        }
      }
    }
  }

  if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen))
  {
    float pt[3] = {cam.getCameraPointingX(), cam.getCameraPointingY(), cam.getCameraPointingZ()};
    if (ImGui::DragFloat3("Looking at", pt, 0.05f))
    {
      cam.setCameraPointing(pt[0], pt[1], pt[2]);
    }
    float zoom = cam.getZoomDistance();
    if (ImGui::DragFloat("Distance", &zoom, 0.05f, 0.1f, 32000.0f, "%.2f"))
    {
      cam.setZoomDistance(zoom);
    }
    float az = cam.getAzimuthDegrees();
    if (ImGui::DragFloat("Azimuth (deg)", &az, 0.5f))
    {
      cam.setAzimuthDegrees(az);
    }
    float el = cam.getElevationDegrees();
    if (ImGui::DragFloat("Elevation (deg)", &el, 0.5f, -90.0f, 90.0f))
    {
      cam.setElevationDegrees(el);
    }
    float fov = cam.getFOVdeg();
    if (ImGui::SliderFloat("FOV (deg)", &fov, 5.0f, 150.0f))
    {
      cam.setFOVdeg(fov);
    }
  }

  if (ImGui::CollapsingHeader("Files and sequences", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Checkbox("Reload the file when it changes on disk", &m_watchFile);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
    ImGui::InputInt("Autoplay delay (ms)", &m_autoplayDelayMs, 10, 100);
    m_autoplayDelayMs = std::max(0, m_autoplayDelayMs);
    ImGui::Checkbox("Keep the camera between files", &m_freeCameraAlways);
    mrpt::imgui::tooltip(
        "When moving through a sequence of files, keep the current camera "
        "zoom, elevation and azimuth instead of the ones saved in the files.");
    ImGui::BeginDisabled(!m_freeCameraAlways);
    ImGui::Checkbox("Keep zoom and elevation only", &m_freeCameraAlwaysNoAzimuth);
    mrpt::imgui::tooltip("Keep the zoom and elevation, but take the azimuth from each file.");
    ImGui::EndDisabled();
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

void SceneViewerApp::renderTravellingPanel()
{
  if (!m_showTravellingPanel)
  {
    return;
  }
  if (!ImGui::Begin(PANEL_TRAVELLING, &m_showTravellingPanel))
  {
    ImGui::End();
    return;
  }
  ImGui::PushItemWidth(-ImGui::GetFontSize() * 9);
  auto& cam = m_view.view.cameraController;

  if (m_travelling != Travelling::None)
  {
    if (ImGui::Button(ICON_MS_STOP " Stop travelling"))
    {
      m_travelling = Travelling::None;
    }
    ImGui::Separator();
  }

  if (ImGui::CollapsingHeader("Circular", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::PushID("circ");
    ImGui::DragFloat3("Center", m_circTarget, 0.05f);
    ImGui::DragFloat("Distance", &m_circZoom, 0.1f, 0.1f, 32000.0f);
    ImGui::DragFloat("Elevation (deg)", &m_circElevation, 0.5f, -90.0f, 90.0f);
    ImGui::DragFloat("Azimuth (deg)", &m_circAzimuth, 0.5f);
    ImGui::DragFloat("Start offset (deg)", &m_circMinOffset, 0.5f, 0.0f, 3600.0f);
    ImGui::DragFloat("End offset (deg)", &m_circMaxOffset, 0.5f, 0.0f, 3600.0f);
    ImGui::DragFloat("Speed (deg/s)", &m_circSpeedDegPerSec, 0.1f, 0.1f, 360.0f);
    if (ImGui::Button("Grab current camera"))
    {
      m_circTarget[0] = cam.getCameraPointingX();
      m_circTarget[1] = cam.getCameraPointingY();
      m_circTarget[2] = cam.getCameraPointingZ();
      m_circZoom = cam.getZoomDistance();
      m_circElevation = cam.getElevationDegrees();
      m_circAzimuth = cam.getAzimuthDegrees();
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_PLAY_ARROW " Start##circ"))
    {
      cam.setCameraPointing(m_circTarget[0], m_circTarget[1], m_circTarget[2]);
      cam.setZoomDistance(m_circZoom);
      cam.setElevationDegrees(m_circElevation);
      cam.setAzimuthDegrees(m_circAzimuth - m_circMinOffset);
      m_travelling = Travelling::Circular;
    }
    ImGui::PopID();
  }

  if (ImGui::CollapsingHeader("Camera path", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::PushID("path");
    if (ImGui::Button(ICON_MS_ADD " Add current camera"))
    {
      CameraKeyframe k;
      k.t = m_path.empty() ? 0.0 : m_path.back().t + 2.0;
      k.x = cam.getCameraPointingX();
      k.y = cam.getCameraPointingY();
      k.z = cam.getCameraPointingZ();
      k.zoom = cam.getZoomDistance();
      k.azimuthDeg = cam.getAzimuthDegrees();
      k.elevationDeg = cam.getElevationDegrees();
      m_path.push_back(k);
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_FOLDER_OPEN " Load..."))
    {
      m_fileDlg.tag = TAG_PATH_LOAD;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Open, "Load camera path",
          {
              {"Text files", {"txt"}}
      },
          lastDir());
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_SAVE " Save..."))
    {
      m_fileDlg.tag = TAG_PATH_SAVE;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Save camera path",
          {
              {"Text files", {"txt"}}
      },
          lastDir());
    }
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_DELETE " Clear"))
    {
      m_path.clear();
    }

    ImGui::Checkbox("Constant speed (m/s):", &m_pathConstantSpeed);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    ImGui::InputFloat("##speed", &m_pathSpeed, 0, 0, "%.3f");
    m_pathSpeed = std::max(1e-3f, m_pathSpeed);
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &m_pathLoop);

    const ImGuiTableFlags tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                   ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX |
                                   ImGuiTableFlags_SizingFixedFit;
    const float tableH = std::min(
        ImGui::GetFrameHeightWithSpacing() * 10,
        ImGui::GetFrameHeightWithSpacing() * (2.5f + static_cast<float>(m_path.size())));
    std::optional<size_t> toDelete;
    if (ImGui::BeginTable("##keyframes", 8, tflags, ImVec2(0, tableH)))
    {
      ImGui::TableSetupScrollFreeze(0, 1);
      const float colW = ImGui::GetFontSize() * 4.0f;
      for (const char* h : {"t (s)", "x", "y", "z", "dist", "az", "el"})
      {
        ImGui::TableSetupColumn(h, ImGuiTableColumnFlags_WidthFixed, colW);
      }
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
      ImGui::TableHeadersRow();
      for (size_t i = 0; i < m_path.size(); i++)
      {
        auto& k = m_path[i];
        ImGui::PushID(static_cast<int>(i));
        ImGui::TableNextRow();
        const auto cell = [](const char* id, float* v)
        {
          ImGui::TableNextColumn();
          ImGui::SetNextItemWidth(-1);
          ImGui::DragFloat(id, v, 0.05f, 0, 0, "%.2f");
        };
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-1);
        double t = k.t;
        if (ImGui::InputDouble("##t", &t, 0, 0, "%.2f"))
        {
          k.t = t;
        }
        cell("##x", &k.x);
        cell("##y", &k.y);
        cell("##z", &k.z);
        cell("##zoom", &k.zoom);
        cell("##az", &k.azimuthDeg);
        cell("##el", &k.elevationDeg);
        ImGui::TableNextColumn();
        if (ImGui::SmallButton(ICON_MS_DELETE))
        {
          toDelete = i;
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    if (toDelete)
    {
      m_path.erase(m_path.begin() + static_cast<std::ptrdiff_t>(*toDelete));
    }

    if (ImGui::Button(ICON_MS_PLAY_ARROW " Start##path") && !m_path.empty())
    {
      std::sort(
          m_path.begin(), m_path.end(),
          [](const CameraKeyframe& a, const CameraKeyframe& b) { return a.t < b.t; });
      if (m_pathConstantSpeed)
      {
        retimeCameraPathConstantSpeed(m_path, m_pathSpeed);
      }
      m_travellingT = m_path.front().t;
      m_travelling = Travelling::Path;
    }
    // The path may have been emptied in this frame:
    if (m_travelling == Travelling::Path && !m_path.empty())
    {
      ImGui::SameLine();
      const double span = m_path.back().t - m_path.front().t;
      ImGui::ProgressBar(
          static_cast<float>(span > 0 ? (m_travellingT - m_path.front().t) / span : 0.0),
          ImVec2(-1, 0));
    }
    ImGui::PopID();
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

void SceneViewerApp::renderPopups()
{
  // --- PLY import options (modal: it gates the import) ---
  if (m_openPlyPopup)
  {
    ImGui::OpenPopup("PLY import options");
    m_openPlyPopup = false;
  }
  if (ImGui::BeginPopupModal("PLY import options", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted(mrpt::system::extractFileName(m_pendingPlyFile).c_str());
    ImGui::Separator();
    ImGui::TextUnformatted("Import as class:");
    ImGui::RadioButton("mrpt::viz::CPointCloud", &m_plyClass, 0);
    ImGui::RadioButton("mrpt::viz::CPointCloudColoured", &m_plyClass, 1);
    if (m_plyClass == 0)
    {
      ImGui::TextUnformatted("Intensity from coordinate:");
      ImGui::RadioButton("None", &m_plyColorFrom, 0);
      ImGui::SameLine();
      ImGui::RadioButton("X", &m_plyColorFrom, 1);
      ImGui::SameLine();
      ImGui::RadioButton("Y", &m_plyColorFrom, 2);
      ImGui::SameLine();
      ImGui::RadioButton("Z", &m_plyColorFrom, 3);
    }
    ImGui::SliderFloat("Point size", &m_plyPointSize, 1.0f, 10.0f, "%.1f");
    ImGui::DragFloat3("Yaw, pitch, roll (deg)", m_plyYPR, 1.0f, -360.0f, 360.0f);
    ImGui::Checkbox("Add XY ground grid", &m_plyGrid);
    ImGui::Checkbox("Add XYZ corner at origin", &m_plyCorner);
    ImGui::Separator();
    if (ImGui::Button("Import"))
    {
      startImportPly(m_pendingPlyFile);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- High resolution render size ---
  if (m_openHighResPopup)
  {
    ImGui::OpenPopup("High-resolution render");
    m_openHighResPopup = false;
  }
  if (ImGui::BeginPopupModal("High-resolution render", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted(m_pendingHighResFile.c_str());
    ImGui::InputInt2("Width x height (pixels)", m_highResSize);
    m_highResSize[0] = std::clamp(m_highResSize[0], 16, 16384);
    m_highResSize[1] = std::clamp(m_highResSize[1], 16, 16384);
    if (ImGui::Button("Render"))
    {
      const std::string f = m_pendingHighResFile;
      const int w = m_highResSize[0];
      const int h = m_highResSize[1];
      m_deferred.emplace_back([this, f, w, h]() { highResRender(f, w, h); });
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Select by class ---
  if (m_openSelectByClassPopup)
  {
    if (m_vizClassNames.empty())
    {
      for (const auto* c : mrpt::rtti::getAllRegisteredClasses())
      {
        if (c->derivedFrom(CLASS_ID(mrpt::viz::CVisualObject)))
        {
          m_vizClassNames.emplace_back(c->className);
        }
      }
      std::sort(m_vizClassNames.begin(), m_vizClassNames.end());
      m_vizClassChecked.assign(m_vizClassNames.size(), false);
    }
    ImGui::OpenPopup("Select by class");
    m_openSelectByClassPopup = false;
  }
  ImGui::SetNextWindowSize(
      ImVec2(ImGui::GetFontSize() * 25, ImGui::GetFontSize() * 25), ImGuiCond_Appearing);
  if (ImGui::BeginPopupModal("Select by class"))
  {
    ImGui::BeginChild("##classes", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()));
    for (size_t i = 0; i < m_vizClassNames.size(); i++)
    {
      bool v = m_vizClassChecked[i];
      if (ImGui::Checkbox(m_vizClassNames[i].c_str(), &v))
      {
        m_vizClassChecked[i] = v;
      }
    }
    ImGui::EndChild();
    if (ImGui::Button("Select"))
    {
      std::set<std::string> wanted;
      for (size_t i = 0; i < m_vizClassNames.size(); i++)
      {
        if (m_vizClassChecked[i])
        {
          wanted.insert(m_vizClassNames[i]);
        }
      }
      m_selected.clear();
      m_scene->visitAllObjects(
          [&](const mrpt::viz::CVisualObject::Ptr& o)
          {
            if (wanted.count(o->GetRuntimeClass()->className) != 0)
            {
              m_selected.push_back(o);
            }
          });
      m_statusMsg = mrpt::format("%zu objects selected", m_selected.size());
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // --- Scale selection ---
  if (m_openScalePopup)
  {
    ImGui::OpenPopup("Re-scale selected objects");
    m_openScalePopup = false;
  }
  if (ImGui::BeginPopupModal(
          "Re-scale selected objects", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::InputFloat("Scale factor", &m_scaleFactor, 0.1f, 1.0f, "%.4f");
    if (ImGui::Button("Apply"))
    {
      for (const auto& o : m_selected)
      {
        o->setScale(
            m_scaleFactor * o->getScaleX(), m_scaleFactor * o->getScaleY(),
            m_scaleFactor * o->getScaleZ());
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

  // --- Long load in progress ---
  if (m_loader.isRunning() && ImGui::GetTime() > 0)
  {
    const ImVec2 c = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(c, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowBgAlpha(0.85f);
    if (ImGui::Begin(
            "##loading", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                ImGuiWindowFlags_NoNav))
    {
      ImGui::TextUnformatted(m_statusMsg.c_str());
    }
    ImGui::End();
  }

  m_about.render();
  m_msgs.render();
}

void SceneViewerApp::setupDefaultLayout(unsigned int dockspaceId)
{
  ImGuiID right = 0;
  ImGuiID rest = 0;
  ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Right, 0.30f, &right, &rest);
  ImGui::DockBuilderDockWindow("View", right);
  ImGui::DockBuilderDockWindow("Travelling", right);
  ImGuiID bottom = 0;
  ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, 0.22f, &bottom, &rest);
  ImGui::DockBuilderDockWindow("Console", bottom);
}

void SceneViewerApp::renderFrame()
{
  // Deferred actions (e.g. off-screen renders), outside of any window:
  for (auto& f : std::exchange(m_deferred, {}))
  {
    try
    {
      f();
    }
    catch (const std::exception& e)
    {
      m_msgs.error("Error", mrpt::exception_to_str(e));
    }
  }

  const double dt = ImGui::GetIO().DeltaTime;
  m_fps = 0.9 * m_fps + 0.1 * (dt > 0 ? 1.0 / dt : 0.0);

  try
  {
    if (auto r = m_loader.poll(); r)
    {
      applyLoadResult(std::move(*r));
    }
  }
  catch (const std::exception& e)
  {
    m_autoplay = false;
    m_statusMsg = "Error loading file.";
    m_msgs.error("Error loading file", mrpt::exception_to_str(e));
    m_console.push(mrpt::system::LVL_ERROR, APP_NAME, mrpt::exception_to_str(e));
  }

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

  updateTravelling(dt);
  checkFileWatch();
  updateAutoplayAndCapture();
  handleKeyboard();

  m_view.render(m_app);
  renderViewPanel();
  renderTravellingPanel();
  if (m_showConsole)
  {
    if (ImGui::Begin(PANEL_CONSOLE, &m_showConsole))
    {
      m_console.render();
    }
    ImGui::End();
  }
  renderPopups();
}

int SceneViewerApp::run(const std::string& fileToOpen)
{
  if (!m_app.init(APP_NAME))
  {
    std::cerr << "Could not create the application window.\n";
    return 1;
  }
  loadSettings();

  m_app.mainMenuBar = [this]() { renderMainMenu(); };
  m_app.beforeDockspace = [this]()
  {
    renderToolbar();
    renderStatusBar();
  };
  m_app.setupDefaultLayout = [this](unsigned int id) { setupDefaultLayout(id); };
  m_app.onFilesDropped = [this](const std::vector<std::string>& files)
  {
    if (files.empty())
    {
      return;
    }
    const auto& f = files.front();
    if (mrpt::system::strCmpI(mrpt::system::extractFileExtension(f, true), "3Dscene"))
    {
      startLoadSceneFile(f);
    }
    else
    {
      startImportModel(f);
    }
  };

  newScene();

  if (!fileToOpen.empty())
  {
    if (mrpt::system::strCmpI(
            "3Dscene", mrpt::system::extractFileExtension(fileToOpen, true /*ignore .gz*/)))
    {
      startLoadSceneFile(fileToOpen);
    }
    else
    {
      std::cout << "Filename extension does not match `3Dscene`, "
                   "importing as an Assimp model...\n";
      startImportModel(fileToOpen);
    }
  }

  m_app.run([this]() { renderFrame(); });

  saveSettings();
  return 0;
}
