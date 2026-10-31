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

#include "navlog-viewer-ui.h"

#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API
#include <implot.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/containers/printf_vector.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>  // intersect()
#include <mrpt/math/utils.h>
#include <mrpt/rtti/CListOfClasses.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/stock_objects.h>

#include <algorithm>  // replace()
#include <fstream>
#include <iostream>

// Configure look:
static const mrpt::viz::TFontParams& getFontParams()
{
  static mrpt::viz::TFontParams fp;
  static bool init = false;
  if (!init)
  {
    fp.vfont_style = mrpt::viz::FILL;
    fp.vfont_name = "mono";
    fp.vfont_scale = 10.0f;  // pixels
    fp.draw_shadow = true;
  }
  return fp;
}

// Text lines of the "Entry details" panel:
#define ADD_WIN_TEXTMSG_COL(__MSG, __COL) addInfoLine(__MSG, __COL)
#define ADD_WIN_TEXTMSG(__MSG)            addInfoLine(__MSG)

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::serialization;
using namespace mrpt::config;
using namespace mrpt::poses;
using namespace mrpt::nav;

NavlogViewerApp::NavlogViewerApp()
{
  m_about.appName = "navlog-viewer";
  m_about.description = "Viewer of reactive navigation logs (*.reactivenavlog).";
  m_about.extraTabs.emplace_back(
      "Keyboard and mouse",
      "Space            : Play / pause\n"
      "Left / Right     : Previous / next log entry\n"
      "PageUp / PageDown: 10 entries backward / forward\n"
      "Home / End       : First / last entry\n"
      "Ctrl+O           : Open a log file\n"
      "F1               : About\n"
      "\n"
      "Mouse over the 3D view: left drag rotates (if not in 2D mode), right\n"
      "drag pans, wheel zooms.\n");

  // Background scene:
  // -----------------------------
  {
    mrpt::viz::Scene::Ptr scene = mrpt::viz::Scene::Create();

    auto& cam = m_bg.view.cameraController;
    cam.setAzimuthDegrees(-90.0f);
    cam.setElevationDegrees(90.0f);
    cam.setZoomDistance(25.0f);

    {
      auto vi = scene->createViewport("xyz");
      vi->setViewportPosition(-0.1, 0.0, 0.1, 0.1);
      vi->setTransparent(true);

      auto glCorners = mrpt::viz::stock_objects::CornerXYZSimple(1.0f, 2.0f);
      scene->insert(glCorners, "xyz");
    }

    // XY ground plane:
    mrpt::viz::CGridPlaneXY::Ptr gl_grid =
        mrpt::viz::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1, 0.75f);
    gl_grid->setColor_u8(mrpt::img::TColor(0xa0a0a0, 0x90));
    scene->insert(gl_grid);

    // XYZ corner at origin:
    scene->insert(mrpt::viz::stock_objects::CornerXYZSimple(1.0, 2.0));

    m_scene = std::move(scene);
    m_bg.view.setScene(m_scene);
  }

  m_timeline.onTimeChanged = [this](double)
  {
    const auto idx = m_timeline.currentKeyIndex();
    if (idx != m_lastShownIdx)
    {
      m_lastShownIdx = idx;
      updateVisualization();
    }
  };
}

void NavlogViewerApp::loadLogfile(const std::string& fileName)
{
  using namespace std::string_literals;

  try
  {
    m_openedFileName = fileName;
    m_app.setWindowTitle("navlog-viewer - "s + mrpt::system::extractFileName(fileName));

    mrpt::io::CCompressedInputStream f(fileName);

    m_logdata.clear();
    m_logdata_ptg_paths.clear();
    m_pastPoses.clear();

    mrpt::rtti::CListOfClasses validClasses;
    validClasses.insert(CLASS_ID(mrpt::nav::CLogFileRecord));

    m_log_first_tim = INVALID_TIMESTAMP;
    m_log_last_tim = INVALID_TIMESTAMP;

    auto arch = archiveFrom(f);
    for (;;)
    {
      try
      {
        CSerializable::Ptr obj = arch.ReadObject();
        if (!validClasses.contains(obj->GetRuntimeClass()))
        {
          m_msgs.warning(
              "Error loading navlog file",
              mrpt::format("Unexpected class found: %s", obj->GetRuntimeClass()->className));
          break;
        }
        m_logdata.push_back(obj);

        // generate time stats:
        if (IS_CLASS(*obj, CLogFileRecord))
        {
          const CLogFileRecord::Ptr logptr = std::dynamic_pointer_cast<CLogFileRecord>(obj);
          const auto it = logptr->timestamps.find("tim_start_iteration");
          if (it != logptr->timestamps.end()) m_log_last_tim = it->second;

          if (!logptr->infoPerPTG.empty())
          {
            size_t nPTGs = logptr->infoPerPTG.size();
            if (nPTGs > m_logdata_ptg_paths.size())
            {
              m_logdata_ptg_paths.resize(nPTGs);
              for (size_t i = 0; i < nPTGs; i++)
              {
                if (logptr->infoPerPTG[i].ptg) m_logdata_ptg_paths[i] = logptr->infoPerPTG[i].ptg;
              }
            }
            else
            {
              for (size_t i = 0; i < nPTGs; i++) logptr->infoPerPTG[i].ptg = m_logdata_ptg_paths[i];
            }
          }
        }

        if (m_log_first_tim == INVALID_TIMESTAMP && m_log_last_tim != INVALID_TIMESTAMP)
          m_log_first_tim = m_log_last_tim;
      }
      catch (CExceptionEOF&)
      {
        break;
      }
      catch (const std::exception& e)
      {
        // EOF in the middle of an object... It may be usual if the logger
        // is shut down not cleanly.
        // Show the error message (catch below)
        throw;
      }
    }
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }

  // Update stats, etc...
  updateInfoFromLoadedLog();
}

void NavlogViewerApp::updateInfoFromLoadedLog()
{
  const size_t N = m_logdata.size();

  // Timeline: real time if all entries have timestamps, else 10 entries/s.
  std::vector<double> keys;
  keys.reserve(N);
  bool allStamped = N > 0;
  for (size_t i = 0; i < N && allStamped; i++)
  {
    const auto log = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
    if (!log)
    {
      allStamped = false;
      break;
    }
    const auto it = log->timestamps.find("tim_start_iteration");
    if (it == log->timestamps.end())
    {
      allStamped = false;
      break;
    }
    keys.push_back(mrpt::Clock::toDouble(it->second));
  }
  if (!allStamped || !std::is_sorted(keys.begin(), keys.end()))
  {
    keys.clear();
    for (size_t i = 0; i < N; i++)
    {
      keys.push_back(0.1 * static_cast<double>(i));
    }
  }

  m_timeline.setRange(0, 0);
  m_timeline.setKeyTimes(keys);
  if (!keys.empty())
  {
    m_timeline.setRange(keys.front(), keys.back());
  }
  m_timeline.clearTracks();
  m_timeline.addTrack({
      "Log entries", {0x80, 0xc0, 0xff, 0xff},
       keys, true
  });

  // Velocities over time (x: the same units as the timeline):
  {
    std::vector<double> t;
    std::vector<double> vx;
    std::vector<double> w;
    for (size_t i = 0; i < N && i < keys.size(); i++)
    {
      const auto log = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
      if (!log)
      {
        continue;
      }
      t.push_back(keys[i] - keys.front());
      vx.push_back(log->cur_vel_local.vx);
      w.push_back(mrpt::RAD2DEG(log->cur_vel_local.omega));
    }
    m_velPlot.clear();
    m_velPlot.xLabel = allStamped ? "Time (s)" : "Entry index / 10";
    m_velPlot.setSeries("vx (m/s)", t, vx);
    m_velPlot.setSeries("omega (deg/s)", t, w);
  }

  m_pastPoses.clear();
  m_lastShownIdx.reset();
  m_timeline.jumpToStart();
  if (N > 0)
  {
    m_lastShownIdx = 0;
    updateVisualization();
  }

  m_txtLogDuration = "???";
  if (m_log_first_tim != INVALID_TIMESTAMP && m_log_last_tim != INVALID_TIMESTAMP)
  {
    m_txtLogDuration =
        mrpt::system::intervalFormat(mrpt::system::timeDifference(m_log_first_tim, m_log_last_tim));
  }
}

// ---------------------------------------------
// 				DRAW ONE LOG RECORD
// ---------------------------------------------
void NavlogViewerApp::updateVisualization()
{
  try
  {
    using namespace std::string_literals;

    updateManualPickControls();

    const int log_idx = currentLogIndex();
    if (log_idx < 0 || log_idx >= int(m_logdata.size()))
    {
      return;
    }
    m_infoLines.clear();
    // In the future, we could handle more log classes. For now, only
    // "CLogFileRecord::Ptr":
    auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
    const CLogFileRecord& log = *logptr;

    m_txtSelectedPTG =
        std::to_string(log.nSelectedPTG) + " from [0-"s + std::to_string(log.nPTGs - 1) + "]"s;

    {
      std::string timeIndexStr = "Time index: "s + std::to_string(log_idx) + " / "s +
                                 std::to_string(std::max<size_t>(1U, m_logdata.size()) - 1);

      if (auto itTim = log.timestamps.find("tim_start_iteration"); itTim != log.timestamps.end())
      {
        timeIndexStr += " (tim_start_iteration="s +
                        mrpt::system::dateTimeLocalToString(itTim->second) + " [local time])"s;
      }
      m_txtTimeIndex = timeIndexStr;
    }

    const bool is_NOP_cmd = log.ptg_index_NOP >= 0;
    const int sel_ptg_idx = !is_NOP_cmd ? log.nSelectedPTG : log.ptg_index_NOP;

    // Draw WS-obstacles
    // --------------------------------
    // Update 3D view:
    {
      mrpt::viz::Scene::Ptr& scene = m_scene;

      const CVectorFloat shap_x = log.robotShape_x, shap_y = log.robotShape_y;

      // Robot frame of reference:
      mrpt::viz::CSetOfObjects::Ptr gl_robot_frame;
      {
        mrpt::viz::CVisualObject::Ptr gl_rbframe_r =
            scene->getByName("robot_frame");  // Get or create if new
        if (!gl_rbframe_r)
        {
          gl_robot_frame = mrpt::viz::CSetOfObjects::Create();
          gl_robot_frame->setName("robot_frame");
          scene->insert(gl_robot_frame);
        }
        else
        {
          gl_robot_frame = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(gl_rbframe_r);
        }

        // Global or local coordinates?
        if (m_useOdometryCoords)
        {
          // Use odometry pose increment wrt initial instant,
          // taking the localization-based "good" pose of the initial
          // instant as a reference, such that the obtained
          // coordinates closely match those expected in the "map"
          // frame:
          auto log0ptr = mrpt::ptr_cast<CLogFileRecord>::from(m_logdata[0]);
          ASSERT_(log0ptr.get());
          const auto curPose =
              log0ptr->robotPoseLocalization + (log.robotPoseOdometry - log0ptr->robotPoseOdometry);

          gl_robot_frame->setPose(curPose);
          m_pastPoses.push_back(curPose);
        }
        else if (m_globalFrame)
        {
          gl_robot_frame->setPose(
              mrpt::poses::CPose3D(mrpt::poses::CPose2D(log.robotPoseLocalization)));

          m_pastPoses.push_back(log.robotPoseLocalization);

          // Move the window focus:
          auto& cam = m_bg.view.cameraController;
          const float px = cam.getCameraPointingX();
          const float py = cam.getCameraPointingY();
          const float cam_zoom = cam.getZoomDistance();
          if ((mrpt::math::TPoint2D(log.robotPoseLocalization) - mrpt::math::TPoint2D(px, py))
                  .norm() > .3 * cam_zoom)
            cam.setCameraPointing(
                static_cast<float>(log.robotPoseLocalization.x),
                static_cast<float>(log.robotPoseLocalization.y), 0.0f);
        }
        else
        {
          gl_robot_frame->setPose(mrpt::poses::CPose3D());
          m_pastPoses.clear();
        }
      }

      // custom visuals:
      {
        mrpt::viz::CSetOfObjects::Ptr gl_custom_vizs;
        {
          // Get or create if new
          auto gl_r = gl_robot_frame->getByName("custom_viz");
          if (!gl_r)
          {
            gl_custom_vizs = mrpt::viz::CSetOfObjects::Create();
            gl_custom_vizs->setName("custom_viz");
            gl_robot_frame->insert(gl_custom_vizs);
          }
          else
          {
            gl_custom_vizs = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(gl_r);
          }

          gl_custom_vizs->clear();
          for (const auto& viz : log.visuals) gl_custom_vizs->insert(viz);
        }
      }

      // Extrapolated poses from delay models:
      {
        mrpt::viz::CSetOfObjects::Ptr gl_relposes;
        mrpt::viz::CVisualObject::Ptr gl_relposes_r =
            gl_robot_frame->getByName("relposes");  // Get or create if new
        if (!gl_relposes_r)
        {
          gl_relposes = mrpt::viz::CSetOfObjects::Create();
          gl_relposes->setName("relposes");
          gl_robot_frame->insert(gl_relposes);
        }
        else
        {
          gl_relposes = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(gl_relposes_r);
        }

        gl_relposes->clear();
        if (m_showDelays)
        {
          {
            mrpt::viz::CSetOfObjects::Ptr gl_relpose_sense =
                mrpt::viz::stock_objects::CornerXYSimple(0.3f, 1);
            gl_relpose_sense->setName("sense");
            gl_relpose_sense->enableShowName(true);
            gl_relpose_sense->setPose(log.relPoseSense);
            gl_relposes->insert(gl_relpose_sense);
          }
          {
            mrpt::viz::CSetOfObjects::Ptr gl_relpose_cmdvel =
                mrpt::viz::stock_objects::CornerXYSimple(0.3f, 1);
            gl_relpose_cmdvel->setName("cmdVel");
            gl_relpose_cmdvel->enableShowName(true);
            gl_relpose_cmdvel->setPose(log.relPoseVelCmd);
            gl_relposes->insert(gl_relpose_cmdvel);
          }
        }
      }

      // track of past poses:
      if (!m_pastPoses.empty())
      {
        mrpt::viz::CSetOfObjects::Ptr gl_poses;
        mrpt::viz::CVisualObject::Ptr gl_track =
            scene->getByName("past_poses");  // Get or create if new
        if (!gl_track)
        {
          gl_poses = mrpt::viz::CSetOfObjects::Create();
          gl_poses->setName("past_poses");
          scene->insert(gl_poses);
        }
        else
        {
          gl_poses = dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(gl_track);
        }
        ASSERT_(gl_poses);
        gl_poses->clear();
        for (const auto& p : m_pastPoses)
        {
          auto glCorner = mrpt::viz::stock_objects::CornerXYSimple(0.3f, 1.0f);
          glCorner->setPose(p);
          gl_poses->insert(glCorner);
        }
      }

      const size_t maxPastPoses = static_cast<size_t>(std::max(0, m_numPastPoses));
      while (!m_pastPoses.empty() && m_pastPoses.size() > maxPastPoses)
      {
        m_pastPoses.erase(m_pastPoses.begin());
      }

      {
        // Obstacles: original
        mrpt::viz::CPointCloud::Ptr gl_obs;
        mrpt::viz::CVisualObject::Ptr gl_obs_r =
            gl_robot_frame->getByName("obs-raw");  // Get or create if new
        if (!gl_obs_r)
        {
          gl_obs = mrpt::viz::CPointCloud::Create();
          gl_obs->setName("obs-raw");
          gl_obs->setPointSize(3);
          gl_obs->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0x00));
          gl_robot_frame->insert(gl_obs);
        }
        else
        {
          gl_obs = dynamic_pointer_cast<mrpt::viz::CPointCloud>(gl_obs_r);
        }
        gl_obs->loadFromPointsMap(&log.WS_Obstacles_original);
        if (m_showDelays)
          gl_obs->setPose(log.relPoseSense);
        else
          gl_obs->setPose(mrpt::poses::CPose3D());
      }
      {
        // Obstacles: after optional filtering
        mrpt::viz::CPointCloud::Ptr gl_obs;
        mrpt::viz::CVisualObject::Ptr gl_obs_r =
            gl_robot_frame->getByName("obs");  // Get or create if new
        if (!gl_obs_r)
        {
          gl_obs = mrpt::viz::CPointCloud::Create();
          gl_obs->setName("obs");
          gl_obs->setPointSize(1.5);
          gl_obs->setLocation(0, 0, 0.001);
          gl_obs->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff));
          gl_robot_frame->insert(gl_obs);
        }
        else
        {
          gl_obs = std::dynamic_pointer_cast<mrpt::viz::CPointCloud>(gl_obs_r);
        }
        gl_obs->loadFromPointsMap(&log.WS_Obstacles);
        if (m_showDelays)
          gl_obs->setPose(log.relPoseSense);
        else
          gl_obs->setPose(mrpt::poses::CPose3D());
      }

      {
        // Selected PTG path:
        mrpt::viz::CSetOfLines::Ptr gl_path;
        mrpt::viz::CVisualObject::Ptr gl_path_r =
            gl_robot_frame->getByName("path");  // Get or create if new
        if (!gl_path_r)
        {
          gl_path = mrpt::viz::CSetOfLines::Create();
          gl_path->setName("path");
          gl_path->setLineWidth(2.0);
          gl_path->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00));
          gl_robot_frame->insert(gl_path);
        }
        else
          gl_path = mrpt::ptr_cast<mrpt::viz::CSetOfLines>::from(gl_path_r);
        gl_path->clear();

        if (sel_ptg_idx < int(m_logdata_ptg_paths.size()) && sel_ptg_idx >= 0)
        {
          mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg = m_logdata_ptg_paths[sel_ptg_idx];
          if (ptg)
          {
            if (!ptg->isInitialized()) ptg->initialize();

            auto& ipp = log.infoPerPTG[sel_ptg_idx];

            // Set instantaneous dyn state:
            ptg->updateNavDynamicState(is_NOP_cmd ? ipp.lastDynState : ipp.dynState);

            // Draw path:
            const uint16_t selected_k = static_cast<uint16_t>(
                log.ptg_index_NOP < 0 ? ptg->alpha2index(ipp.desiredDirection)
                                      : log.ptg_last_k_NOP);
            double max_dist = ptg->getRefDistance();
            ptg->add_robotShape_to_setOfLines(*gl_path);

            ptg->renderPathAsSimpleLine(selected_k, *gl_path, 0.10, max_dist);

            // PTG origin:
            // enable delays model?
            mrpt::math::TPose2D ptg_origin =
                (m_showDelays) ? log.relPoseVelCmd : mrpt::math::TPose2D(0, 0, 0);

            // "NOP cmd" case:
            if (log.ptg_index_NOP >= 0)
            {
              ptg_origin = ptg_origin - log.rel_cur_pose_wrt_last_vel_cmd_NOP;
            }

            gl_path->setPose(ptg_origin);

            // Overlay a sequence of robot shapes:
            if (m_drawShape)
            {
              double min_shape_dists = std::max(0.01, static_cast<double>(m_shapeMinDist));

              for (double d = min_shape_dists; d < max_dist; d += min_shape_dists)
              {
                const auto step = ptg->getPathStepForDist(selected_k, d);
                if (!step) continue;
                const auto p = ptg->getPathPose(selected_k, *step);
                ptg->add_robotShape_to_setOfLines(*gl_path, mrpt::poses::CPose2D(p));
              }
            }
            {
              // Robot shape:
              mrpt::viz::CSetOfLines::Ptr gl_shape;
              mrpt::viz::CVisualObject::Ptr gl_shape_r =
                  gl_robot_frame->getByName("shape");  // Get or create if new
              if (!gl_shape_r)
              {
                gl_shape = std::make_shared<mrpt::viz::CSetOfLines>();
                gl_shape->setName("shape");
                gl_shape->setLineWidth(4.0);
                gl_shape->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00));
                gl_robot_frame->insert(gl_shape);
              }
              else
              {
                gl_shape = std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(gl_shape_r);
              }
              gl_shape->clear();
              ptg->add_robotShape_to_setOfLines(*gl_shape);
            }
            {
              mrpt::viz::CSetOfLines::Ptr gl_shape;
              mrpt::viz::CVisualObject::Ptr gl_shape_r =
                  gl_robot_frame->getByName("velocity");  // Get or create if new
              if (!gl_shape_r)
              {
                gl_shape = std::make_shared<mrpt::viz::CSetOfLines>();
                gl_shape->setName("velocity");
                gl_shape->setLineWidth(4.0);
                gl_shape->setColor_u8(mrpt::img::TColor(0x00, 0xff, 0xff));
                gl_robot_frame->insert(gl_shape);
              }
              else
              {
                gl_shape = std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(gl_shape_r);
              }
              gl_shape->clear();
              const mrpt::math::TTwist2D& velLocal = log.cur_vel_local;
              gl_shape->appendLine(0, 0, 0, velLocal.vx, velLocal.vy, 0);
            }
          }
        }
      }

      {
        // Manually-picked PTG path:
        mrpt::viz::CSetOfLines::Ptr gl_path;
        mrpt::viz::CVisualObject::Ptr gl_path_r =
            gl_robot_frame->getByName("manual-path");  // Get or create if new
        if (!gl_path_r)
        {
          gl_path = mrpt::viz::CSetOfLines::Create();
          gl_path->setName("manual-path");
          gl_path->setLineWidth(2.0);
          gl_path->setColor_u8(mrpt::img::TColor(0x00, 0xff, 0x00));
          gl_robot_frame->insert(gl_path);
        }
        else
          gl_path = mrpt::ptr_cast<mrpt::viz::CSetOfLines>::from(gl_path_r);
        gl_path->clear();

        if (m_manualPickPTGIdx >= 0 && m_manualPickPTGIdx < int(m_logdata_ptg_paths.size()) &&
            m_manualPickTrajectoryIdx >= 0)
        {
          mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
              m_logdata_ptg_paths[m_manualPickPTGIdx];
          if (ptg)
          {
            if (!ptg->isInitialized()) ptg->initialize();

            const bool this_NOP_cmd =
                m_manualPickPTGIdx == static_cast<int>(m_logdata_ptg_paths.size() - 1);

            auto& ipp = log.infoPerPTG[m_manualPickPTGIdx];

            // Set instantaneous dyn state:
            ptg->updateNavDynamicState(this_NOP_cmd ? ipp.lastDynState : ipp.dynState);

            // Draw path:
            const uint16_t selected_k = static_cast<uint16_t>(m_manualPickTrajectoryIdx);
            double max_dist = ptg->getRefDistance();
            ptg->add_robotShape_to_setOfLines(*gl_path);

            ptg->renderPathAsSimpleLine(selected_k, *gl_path, 0.10, max_dist);

            // PTG origin:
            // enable delays model?
            mrpt::math::TPose2D ptg_origin =
                (m_showDelays) ? log.relPoseVelCmd : mrpt::math::TPose2D(0, 0, 0);

            // "NOP cmd" case:
            if (log.ptg_index_NOP >= 0)
            {
              ptg_origin = ptg_origin - log.rel_cur_pose_wrt_last_vel_cmd_NOP;
            }

            gl_path->setPose(ptg_origin);

            // Overlay a sequence of robot shapes:
            if (m_drawShape)
            {
              double min_shape_dists = std::max(0.01, static_cast<double>(m_shapeMinDist));

              for (double d = min_shape_dists; d < max_dist; d += min_shape_dists)
              {
                const auto step = ptg->getPathStepForDist(selected_k, d);
                if (!step) continue;
                const auto p = ptg->getPathPose(selected_k, *step);
                ptg->add_robotShape_to_setOfLines(*gl_path, mrpt::poses::CPose2D(p));
              }
            }
            {
              // Robot shape:
              mrpt::viz::CSetOfLines::Ptr gl_shape;
              mrpt::viz::CVisualObject::Ptr gl_shape_r =
                  gl_robot_frame->getByName("shape");  // Get or create if new
              if (!gl_shape_r)
              {
                gl_shape = std::make_shared<mrpt::viz::CSetOfLines>();
                gl_shape->setName("shape");
                gl_shape->setLineWidth(4.0);
                gl_shape->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00));
                gl_robot_frame->insert(gl_shape);
              }
              else
              {
                gl_shape = std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(gl_shape_r);
              }
              gl_shape->clear();
              ptg->add_robotShape_to_setOfLines(*gl_shape);
            }
          }
        }
      }

      {
        // Target:
        mrpt::viz::CSetOfObjects::Ptr gl_trgCorners;
        mrpt::viz::CVisualObject::Ptr gl_trgC_r =
            gl_robot_frame->getByName("target_corners");  // Get or create if new

        mrpt::viz::CPointCloud::Ptr gl_trg;
        mrpt::viz::CVisualObject::Ptr gl_trg_r =
            gl_robot_frame->getByName("target");  // Get or create if new
        if (!gl_trg_r)
        {
          gl_trg = mrpt::viz::CPointCloud::Create();
          gl_trg->setName("target");
          gl_trg->enableShowName(true);
          gl_trg->setPointSize(9.0);
          gl_trg->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0x00));
          gl_robot_frame->insert(gl_trg);

          gl_trgCorners = mrpt::viz::CSetOfObjects::Create();
          gl_trgCorners->setName("target_corners");
          gl_robot_frame->insert(gl_trgCorners);
        }
        else
        {
          gl_trg = std::dynamic_pointer_cast<mrpt::viz::CPointCloud>(gl_trg_r);
          gl_trgCorners = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(gl_trgC_r);
          ASSERT_(gl_trg);
          ASSERT_(gl_trgCorners);
        }
        // Move the map & add a point at (0,0,0) so the name label
        // appears at the target:
        gl_trg->clear();
        gl_trgCorners->clear();
        if (!log.WS_targets_relative.empty())
        {
          const auto t0 = log.WS_targets_relative[0];
          const float tz = .05f;
          gl_trg->setLocation(t0.x, t0.y, tz);
          for (const auto& t : log.WS_targets_relative)
          {
            gl_trg->insertPoint(static_cast<float>(t.x - t0.x), static_cast<float>(t.y - t0.y), tz);
            auto glCorner = mrpt::viz::stock_objects::CornerXYZ(0.25f);
            glCorner->setPose(t);
            gl_trgCorners->insert(glCorner);
          }
        }
      }
    }

    // Show extra info as text msgs:
    // ---------------------------------

    if (m_showAllDebugFields)
    {
      for (const auto& e : log.timestamps)
        ADD_WIN_TEXTMSG(mrpt::format(
            "Timestamp %-20s=%s", e.first.c_str(),
            mrpt::system::dateTimeLocalToString(e.second).c_str()));
    }

    {
      const unsigned int nObsOrg = static_cast<unsigned int>(log.WS_Obstacles_original.size()),
                         nObs = static_cast<unsigned int>(log.WS_Obstacles.size());
      const unsigned int nObsFiltered = nObsOrg - nObs;

      if (nObsFiltered)
      {
        ADD_WIN_TEXTMSG(mrpt::format("Obstacle points=%u (%u filtered out)", nObs, nObsFiltered));
      }
      else
      {
        ADD_WIN_TEXTMSG(mrpt::format("Obstacle points=%u", nObs));
      }
    }

    ADD_WIN_TEXTMSG(mrpt::format(
        "cmd_vel=%s", log.cmd_vel ? log.cmd_vel->asString().c_str() : "NOP (Continue last PTG)"));

    ADD_WIN_TEXTMSG(mrpt::format(
        "cur_vel      =[%.02f m/s, %0.2f m/s, %.02f dps]", log.cur_vel.vx, log.cur_vel.vy,
        mrpt::RAD2DEG(log.cur_vel.omega)));
    ADD_WIN_TEXTMSG(mrpt::format(
        "cur_vel_local=[%.02f m/s, %0.2f m/s, %.02f dps]", log.cur_vel_local.vx,
        log.cur_vel_local.vy, mrpt::RAD2DEG(log.cur_vel_local.omega)));

    {
      static TPose2D formerPoseLoc = log.robotPoseLocalization;

      ADD_WIN_TEXTMSG(mrpt::format(
          "robot_pose    =%35s | Increment since last step=%s",
          log.robotPoseLocalization.asString().c_str(),
          (log.robotPoseLocalization - formerPoseLoc).asString().c_str()));

      formerPoseLoc = log.robotPoseLocalization;
    }

    {
      static TPose2D formerPoseOdo = log.robotPoseOdometry;

      ADD_WIN_TEXTMSG(mrpt::format(
          "robot_odometry=%35s | Increment since last step=%s",
          log.robotPoseOdometry.asString().c_str(),
          (log.robotPoseOdometry - formerPoseOdo).asString().c_str()));

      formerPoseOdo = log.robotPoseOdometry;
    }

    {
      for (unsigned int i = 0; i < log.WS_targets_relative.size(); i++)
      {
        ADD_WIN_TEXTMSG(
            mrpt::format("rel_target[%u]=%s", i, log.WS_targets_relative[i].asString().c_str()));
      }
    }

    if (log.cmd_vel_original)
    {
      std::stringstream ss;
      ss << "original cmd_vel: ";
      ss << log.cmd_vel_original->asString();
      ADD_WIN_TEXTMSG(ss.str());
    }

    {
      std::stringstream ss;
      ss << "Performance: ";
      for (size_t i = 0; i < log.infoPerPTG.size(); i++)
        ss << "PTG#" << i
           << mrpt::format(
                  " TPObs:%ss HoloNav:%ss |",
                  mrpt::system::unitsFormat(log.infoPerPTG[i].timeForTPObsTransformation).c_str(),
                  mrpt::system::unitsFormat(log.infoPerPTG[i].timeForHolonomicMethod).c_str());
      ADD_WIN_TEXTMSG(ss.str());
    }

    for (unsigned int nPTG = 0; nPTG < log.infoPerPTG.size(); nPTG++)
    {
      const CLogFileRecord::TInfoPerPTG& pI = log.infoPerPTG[nPTG];

      mrpt::img::TColorf col;
      if (static_cast<int>(nPTG) == log.nSelectedPTG)
        col = mrpt::img::TColorf(1, 1, 1);
      else
        col = mrpt::img::TColorf(.8f, .8f, .8f);

      std::string sFactors;
      for (const auto& kv : pI.evalFactors)
      {
        sFactors += kv.first;
        sFactors += ": ";
        sFactors += std::to_string(kv.second);
        sFactors += " ";
      }

      ADD_WIN_TEXTMSG_COL(
          mrpt::format(
              "PTG#%u: SelDir=%+7.01f deg SelSpeed=%.03f Eval=%5.03f. %s", nPTG,
              mrpt::RAD2DEG(pI.desiredDirection), pI.desiredSpeed, pI.evaluation, sFactors.c_str()),
          col);
    }

    ADD_WIN_TEXTMSG(mrpt::format(
        "relPoseSense: %s relPoseVelCmd:%s", log.relPoseSense.asString().c_str(),
        log.relPoseVelCmd.asString().c_str()));

    if (m_showAllDebugFields)
    {
      if (!log.infoPerPTG.empty())
      {
        ADD_WIN_TEXTMSG(mrpt::format(
                            "navDynState: curVelLocal=%s relTarget=%s "
                            "targetRelSpeed=%.02f",
                            log.infoPerPTG.front().dynState.curVelLocal.asString().c_str(),
                            log.infoPerPTG.front().dynState.relTarget.asString().c_str(),
                            log.infoPerPTG.front().dynState.targetRelSpeed)
                            .c_str());
      }

      for (const auto& e : log.values)
        ADD_WIN_TEXTMSG(mrpt::format(
            "%-30s=%s ", e.first.c_str(), mrpt::system::unitsFormat(e.second, 3, false).c_str()));

      for (const auto& e : log.additional_debug_msgs)
        ADD_WIN_TEXTMSG(mrpt::format("%-30s=%s ", e.first.c_str(), e.second.c_str()));
    }

    // Draw TP-obstacles
    // --------------------------------
    // log.infoPerPTG.size() may be != nPTGs in the last entry is used for
    // "NOP cmdvel"

    for (unsigned int nPTG = 0; nPTG < log.infoPerPTG.size(); nPTG++)
    {
      const bool is_selected_ptg = (int(nPTG) == log.nSelectedPTG);
      VizPTG& viz = m_mywins3D[nPTG];
      if (!viz.scene)
      {
        viz.title = mrpt::format("PTG #%u (TP-space)###ptg%u", nPTG, nPTG);
        viz.scene = mrpt::viz::Scene::Create();
        viz.view = std::make_unique<mrpt::imgui::CImGuiSceneView>();
        viz.view->setScene(viz.scene);
        auto& scene = viz.scene;

        scene->getViewport()->addTextMessage(
            4, 4, mrpt::format("[%u]:%s", nPTG, log.infoPerPTG[nPTG].PTG_desc.c_str()), 0 /*id*/,
            getFontParams());

        scene->insert(mrpt::viz::CGridPlaneXY::Create(-1.0f, 1.0f, -1.0f, 1.0f, .0f, 1.0f));
        scene->insert(mrpt::viz::stock_objects::CornerXYSimple(0.4f, 2.0f));

        auto& cam = viz.view->cameraController;
        cam.setAzimuthDegrees(-90.0f);
        cam.setElevationDegrees(90.0f);
        cam.setZoomDistance(4.6f);
        cam.setProjectiveModel(false);

        {
          auto gl_obj = mrpt::viz::CDisk::Create();
          gl_obj->setDiskRadius(1.01f, 1.0);
          gl_obj->setSlicesCount(30);
          gl_obj->setColor_u8(mrpt::img::TColor(0x30, 0x30, 0x30, 0xff));
          scene->insert(gl_obj);
        }
        {
          auto gl_obj = mrpt::viz::CSetOfLines::Create();
          gl_obj->setName("tp_obstacles");
          gl_obj->setLineWidth(1.0f);
          gl_obj->setVerticesPointSize(4.0f);
          gl_obj->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff, 0xff));
          scene->insert(gl_obj);
        }
        {
          auto gl_obj = mrpt::viz::CSetOfLines::Create();
          gl_obj->setName("score_phase1");
          gl_obj->setLineWidth(1.0f);
          gl_obj->setVerticesPointSize(2.0f);
          gl_obj->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0x00, 0xff));
          scene->insert(gl_obj);
        }
        {
          auto gl_obj = mrpt::viz::CSetOfLines::Create();
          gl_obj->setName("score_phase2");
          gl_obj->setLineWidth(1.0f);
          gl_obj->setVerticesPointSize(2.0f);
          gl_obj->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0xff, 0xff));
          scene->insert(gl_obj);
        }
        {
          auto gl_obj = mrpt::viz::CSetOfLines::Create();
          gl_obj->setName("tp_selected_dir");
          gl_obj->setLineWidth(3.0f);
          gl_obj->setColor_u8(mrpt::img::TColor(0x00, 0xff, 0x00, 0xd0));
          scene->insert(gl_obj);
        }
        {
          auto gl_obj = mrpt::viz::CSetOfObjects::Create();
          gl_obj->setName("tp_target");
          gl_obj->setLocation(0, 0, 0.02f);
          scene->insert(gl_obj);
        }
        {
          auto gl_obj = mrpt::viz::CPointCloud::Create();
          gl_obj->setName("tp_robot");
          gl_obj->setPointSize(4.0f);
          gl_obj->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00, 0xa0));
          gl_obj->setLocation(0, 0, 0.02f);
          scene->insert(gl_obj);
        }
#if 0  // should be re-done with CMesh3D?
			{
				auto gl_obj =
					mrpt::viz::CMesh::Create(true /*transparency*/);
				gl_obj->setName("tp_clearance");
				gl_obj->setScale(1.0f, 1.0f, 5.0f);
				scene->insert(gl_obj);
			}
#endif
      }

      mrpt::viz::Scene::Ptr& scene = viz.scene;
      auto view = scene->getViewport();

      // Draw dynamic stuff:
      const CLogFileRecord::TInfoPerPTG& pI = log.infoPerPTG[nPTG];
      vector<float> xs, ys;

      const size_t nAlphas = pI.TP_Obstacles.size();

      view->clearTextMessages();

      auto fp = getFontParams();
      fp.color = is_selected_ptg ? TColorf(1.0f, 1.0f, 0.f) : TColorf(1.0f, 1.0f, 1.0f);
      view->addTextMessage(
          4, 4, mrpt::format("[%u]:%s", nPTG, log.infoPerPTG[nPTG].PTG_desc.c_str()), 0 /*id*/, fp);

      // Chosen direction:
      {
        const double aDir = pI.desiredDirection;

        auto gl_obj =
            std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(scene->getByName("tp_selected_dir"));
        gl_obj->clear();
        gl_obj->appendLine(0, 0, 0, pI.desiredSpeed * cos(aDir), pI.desiredSpeed * sin(aDir), 0);
      }

      // obstacles:
      xs.clear();
      ys.clear();
      xs.reserve(nAlphas);
      ys.reserve(nAlphas);
      for (size_t i = 0; i < nAlphas; ++i)
      {
        const double a =
            -M_PI + (static_cast<double>(i) + 0.5) * 2 * M_PI / static_cast<double>(nAlphas);
        const double r = pI.TP_Obstacles[i];
        xs.push_back(static_cast<float>(r * cos(a)));
        ys.push_back(static_cast<float>(r * sin(a)));
      }
      {
        auto gl_obj =
            std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(scene->getByName("tp_obstacles"));
        gl_obj->clear();
        if (nAlphas > 2)
        {
          gl_obj->appendLine(xs[0], ys[0], 0, xs[1], ys[1], 0);
          for (size_t i = 2; i < nAlphas; i++) gl_obj->appendLineStrip(xs[i], ys[i], 0);
        }
      }

      // Target:
      {
        auto gl_obj =
            std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(scene->getByName("tp_target"));
        gl_obj->clear();
        auto glTargetPts = mrpt::viz::CPointCloud::Create();
        glTargetPts->setPointSize(5);
        glTargetPts->setColor_u8(0, 0, 0);
        gl_obj->insert(glTargetPts);
        for (const auto& p : pI.TP_Targets)
        {
          auto glCorner = mrpt::viz::stock_objects::CornerXYZ(0.15f);
          glCorner->setPose(p);
          gl_obj->insert(glCorner);
          glTargetPts->insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), 0.02f);
        }

        if (!pI.TP_Targets.empty())
        {
          double ang = ::atan2(pI.TP_Targets[0].y, pI.TP_Targets[0].x);
          int tp_target_k = 0;
          if (sel_ptg_idx < int(m_logdata_ptg_paths.size()) && sel_ptg_idx >= 0)
          {
            mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
                m_logdata_ptg_paths[sel_ptg_idx];
            if (ptg)
            {
              tp_target_k = ptg->alpha2index(ang);
            }
          }

          auto fp2 = getFontParams();
          fp2.draw_shadow = false;
          view->addTextMessage(
              4, -12,
              mrpt::format(
                  "TP_Target[0]=(%.02f,%.02f) k=%i ang=%.02f deg", pI.TP_Targets[0].x,
                  pI.TP_Targets[0].y, tp_target_k, mrpt::RAD2DEG(ang)),
              1 /*id*/, fp2);
        }
      }
      if (m_showAllDebugFields)
      {
        unsigned int ptg_unique_id = 2;
        int ptg_lineY = 1;
        for (const auto& e : pI.evalFactors)
        {
          view->addTextMessage(
              4,
              5.0 + static_cast<double>(ptg_lineY++) *
                        static_cast<double>(getFontParams().vfont_scale + 3),
              mrpt::format("%20s=%6.03f", e.first.c_str(), e.second), ptg_unique_id++,
              getFontParams());
        }
      }

      // Current robot pt (normally in pure reactive, at (0,0)):
      {
        auto gl_obj =
            std::dynamic_pointer_cast<mrpt::viz::CPointCloud>(scene->getByName("tp_robot"));
        gl_obj->clear();
        gl_obj->insertPoint(
            static_cast<float>(pI.TP_Robot.x), static_cast<float>(pI.TP_Robot.y), 0.0f);
      }

      // Clearance-diagram:
#if 0
		{
			auto gl_obj = std::dynamic_pointer_cast<mrpt::viz::CMesh>(
				scene->getByName("tp_clearance"));
			if (pI.clearance.empty())
				gl_obj->setVisibility(false);
			else
			{
				gl_obj->setVisibility(true);
				pI.clearance.renderAs3DObject(
					*gl_obj, -1.0, 1.0, -1.0, 1.0, 0.15,
					m_clearanceOverPath /*interp over path*/);
			}
		}
#endif
      // Scores
      {
        auto gl_obj1 =
            std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(scene->getByName("score_phase1"));
        auto gl_obj2 =
            std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(scene->getByName("score_phase2"));
        const bool visible1 = m_perPTGPlots >= 1;
        const bool visible2 = m_perPTGPlots >= 2;
        gl_obj1->clear();
        gl_obj2->clear();
        gl_obj1->setVisibility(visible1);
        gl_obj2->setVisibility(visible2);

        if ((visible1 || visible2) && pI.HLFR && nAlphas > 2)
        {
          const auto& dir_evals = pI.HLFR->dirs_eval;
          const bool has_scores = !dir_evals.empty() && dir_evals[0].size() == nAlphas;
          const size_t num_scores = dir_evals.size();

          for (size_t iScore = 0; has_scores && iScore < num_scores; iScore++)
          {
            vector<mrpt::math::TPoint2D> pts;
            pts.reserve(nAlphas);
            for (size_t i = 0; i < nAlphas; ++i)
            {
              const double a =
                  -M_PI + (static_cast<double>(i) + 0.5) * 2 * M_PI / static_cast<double>(nAlphas);
              const double r = dir_evals[iScore][i];
              pts.emplace_back(r * cos(a), r * sin(a));
            }

            mrpt::viz::CSetOfLines::Ptr& gl_obj = iScore == (num_scores - 1) ? gl_obj1 : gl_obj2;

            gl_obj->appendLine(pts[0].x, pts[0].y, 0, pts[1].x, pts[1].y, 0);
            for (size_t i = 2; i < nAlphas; i++)
              gl_obj->appendLineStrip(
                  static_cast<float>(pts[i].x), static_cast<float>(pts[i].y), 0.0f);
          }
        }
      }
    }  // end for each PTG
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

// ------------------------------------------------------------------------
//     Generate a MATLAB script that draws the overall navigation log
// ------------------------------------------------------------------------
void NavlogViewerApp::onMatlabPlots(const std::string& fileName)
{
  try
  {
    ofstream f(fileName);
    if (!f.is_open()) throw runtime_error("Error writing to file!");

    f << "% Script for drawing navigation log\n"
      << "% Generated automatically by navlog-viewer - MRPT " << mrpt::system::MRPT_getVersion()
      << "\n"
      << "%  From log: " << m_openedFileName << "\n"
      << "% "
         "-------------------------------------------------------------"
         "--------"
         "----\n\n";

    f << "%%\n"
      << "function [] = main()\n"
      << "figure;"
      << "title('Path for " << mrpt::system::extractFileName(fileName) << "');"
      << "% Robot shape: (x,y) in each line\n"
      << "rs = [-0.3 -0.3;0.6 -0.3;0.6 0.3;-0.3 0.3];\n"
      << "dec_shps = 15;"
      << "dec=0;";

    const int DECIMATE_POINTS = 10;
    int decim_point_cnt = 0;

    std::vector<float> X, Y;    // Obstacles
    std::vector<float> TX, TY;  // Target over time

    const size_t N = m_logdata.size();
    for (size_t i = 0; i < N; i++)
    {
      const CLogFileRecord::Ptr logsptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
      const CLogFileRecord* logptr = logsptr.get();

      const auto robotPose = logptr->robotPoseLocalization;
      CPose2D observationBasePose = CPose2D(robotPose);

      if (m_showDelays) observationBasePose = observationBasePose + CPose2D(logptr->relPoseSense);

      f << mrpt::format(
          "dec=dec+1; if (dec>=dec_shps); drawRobotShape(rs,[%f %f "
          "%f]); "
          "dec=0; end\n",
          robotPose.x, robotPose.y, robotPose.phi);

      if (++decim_point_cnt >= DECIMATE_POINTS)
      {
        CSimplePointsMap pts;
        pts.changeCoordinatesReference(logptr->WS_Obstacles, CPose3D(observationBasePose));

        const auto& pX = pts.getPointsBufferRef_x();
        const auto& pY = pts.getPointsBufferRef_y();

        X.insert(X.begin(), pX.begin(), pX.end());
        Y.insert(Y.begin(), pY.begin(), pY.end());
      }

      // Target:
      const mrpt::math::TPoint2D trg_glob =
          mrpt::math::TPoint2D(robotPose + logptr->WS_targets_relative[0]);
      if (TX.empty() || std::abs((*TX.rbegin()) - trg_glob.x) > 1e-3 ||
          std::abs((*TY.rbegin()) - trg_glob.y) > 1e-3)
      {
        TX.push_back(static_cast<float>(trg_glob.x));
        TY.push_back(static_cast<float>(trg_glob.y));
      }
    }

    f << "\n % Points: \n"
      << " Ps = [";
    for (size_t k = 0; k < X.size(); k++)
    {
      f << X[k] << " " << Y[k] << "\n";
    }

    f << "];\n"
      << "plot(Ps(:,1),Ps(:,2),'k.','MarkerSize',3);\n";

    f << "\n % Target point: \n"
      << " Ts = [";
    for (size_t k = 0; k < TX.size(); k++)
    {
      f << TX[k] << " " << TY[k] << "\n";
    }

    f << "];\n"
      << "plot(Ts(:,1),Ts(:,2),'rx','MarkerSize',10);\n";

    f << "axis equal;\n"
      << "\n";

    f << "%% drawRobotShape()\n"
      << "function [] = drawRobotShape(sh,pose)\n"
      << "nPts=size(sh,1);\n"
      << "Pts=[];\n"
      << "for i=1:(nPts+1),\n"
      << "    j=mod(i-1,nPts)+1;\n"
      << "    cc=cos(pose(3)); ss=sin(pose(3)); x=pose(1); y=pose(2);\n"
      << "    Pts=[Pts;x+cc*sh(j,1)-ss*sh(j,2) y+ss*sh(j,1)+cc*sh(j,2) "
         "];\n"
      << "end\n"
      << "plot(Pts(:,1),Pts(:,2)); hold on;\n";
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::onSavePTGParams(const std::string& fileName)
{
  try
  {
    const std::string sSection = "PTG_PARAMS";
    mrpt::config::CConfigFileMemory cfg;

    cfg.write(sSection, "PTG_COUNT", m_logdata_ptg_paths.size());

    CConfigFilePrefixer cfg_pre;
    cfg_pre.bind(cfg);

    for (size_t i = 0; i < m_logdata_ptg_paths.size(); i++)
    {
      mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg = m_logdata_ptg_paths[i];
      if (!ptg) continue;

      const std::string sKeyPrefix = mrpt::format("PTG%d_", static_cast<int>(i));
      cfg_pre.setPrefixes("", sKeyPrefix);

      ptg->saveToConfigFile(cfg_pre, sSection);
    }

    const std::string sCfgText = cfg.getContent();

    std::ofstream f(fileName);
    f << sCfgText;
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::onOdometryVsLocalizationReport(const std::string& fileName)
{
  try
  {
    std::ofstream f(fileName);
    ASSERT_(f.is_open());

    f << "% timestamp  localizationPoseIncr  odometryPoseIncr\n";

    std::optional<mrpt::math::TPose2D> lastOdo, lastLocPose;

    const size_t N = m_logdata.size();
    for (size_t i = 0; i < N; i++)
    {
      const CLogFileRecord::Ptr logsptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
      const CLogFileRecord* logptr = logsptr.get();
      if (!logptr) continue;
      const auto& log = *logptr;

      const auto& odo = log.robotPoseOdometry;
      const auto& pose = log.robotPoseLocalization;

      if (lastOdo && lastLocPose)
      {
        const auto odoIncr = odo - *lastOdo;
        const auto locIncr = pose - *lastLocPose;

        if (!log.timestamps.empty())
          f << mrpt::format("%.03f ", mrpt::Clock::toDouble(log.timestamps.begin()->second));
        else
          f << "0 ";

        for (int k = 0; k < 3; k++) f << locIncr[k] << " ";
        for (int k = 0; k < 3; k++) f << odoIncr[k] << " ";
        f << "\n";
      }

      lastOdo = odo;
      lastLocPose = pose;
    }
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::onSaveScoreMatrix(const std::string& fileName)
{
  try
  {
    const size_t N = m_logdata.size();
    for (size_t i = 0; i < N; i++)
    {
      const CLogFileRecord::Ptr logsptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
      const CLogFileRecord* logptr = logsptr.get();

      for (size_t iPTG = 0; iPTG < logptr->infoPerPTG.size(); iPTG++)
      {
        const CHolonomicLogFileRecord::Ptr& hlog = logptr->infoPerPTG[iPTG].HLFR;
        if (!hlog) continue;

        const mrpt::math::CMatrixD* dirs_scores = hlog->getDirectionScores();
        if (!dirs_scores || dirs_scores->rows() < 2) continue;

        const std::string sFil = mrpt::system::fileNameChangeExtension(
            fileName, mrpt::format(
                          "step%06u_ptg%02u.txt", static_cast<unsigned int>(i),
                          static_cast<unsigned int>(iPTG)));

        dirs_scores->saveToTextFile(sFil, mrpt::math::MATRIX_FORMAT_FIXED);
      }
    }
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::onSaveCurrentObstacles(const std::string& fileName)
{
  try
  {
    const int log_idx = currentLogIndex();
    if (log_idx >= int(m_logdata.size()))
    {
      return;
    }
    auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
    const CLogFileRecord& log = *logptr;

    (void)log.WS_Obstacles_original.save2D_to_text_file(fileName);
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::updateMouseXY()
{
  m_mousePosText.clear();
  if (!m_showCursorXY || !m_bg.isHovered())
  {
    return;
  }
  const auto mouse_ray_opt = m_bg.view.mouseRay();
  if (!mouse_ray_opt)
  {
    return;
  }
  // Intersection of the mouse ray with the Z=0 plane:
  const mrpt::math::TPlane ground_plane(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
  mrpt::math::TObject3D inters;
  mrpt::math::intersect(*mouse_ray_opt, ground_plane, inters);
  mrpt::math::TPoint3D inters_pt;
  if (inters.getPoint(inters_pt))
  {
    m_mousePosText = mrpt::format("Mouse: X=%.04f  Y=%.04f", inters_pt.x, inters_pt.y);
  }
}

void NavlogViewerApp::onExportSelected(const std::string& filename)
{
  try
  {
    const int log_idx = currentLogIndex();
    if (log_idx >= int(m_logdata.size()))
    {
      return;
    }
    auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
    const CLogFileRecord& log = *logptr;

    mrpt::io::CCompressedOutputStream f(filename);
    ASSERT_(f.is_open());
    auto a = mrpt::serialization::archiveFrom(f);

    a << log;
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::onMatlabExportPaths(const std::string& fileName)
{
  try
  {
    const size_t N = m_logdata.size();
    ASSERTMSG_(N > 0, "Log is empty! Load a valid log first...");

    ofstream f(fileName);
    if (!f.is_open()) throw runtime_error("Error writing to file!");

    f << "% Script for drawing navigation paths\n"
      << "% Generated automatically by navlog-viewer - MRPT " << mrpt::system::MRPT_getVersion()
      << "\n"
      << "%  From log: " << m_openedFileName << "\n"
      << "% "
         "-------------------------------------------------------------"
         "----"
         "----"
         "----\n\n";

    std::map<double, int> selected_PTG_over_time;  // time: tim_start_iteration
    std::map<double, double> iteration_duration;   // time: tim_start_iteration
    struct TRobotPoseVel
    {
      mrpt::math::TPose2D pose, poseOdom;
      mrpt::math::TTwist2D velGlobal, velLocal;
    };
    std::map<double, TRobotPoseVel> global_local_vel;  // time: curPoseAndVel
    std::map<double, double> vals_timoff_obstacles, vals_timoff_curPoseVelAge,
        vals_timoff_sendVelCmd;  // time: tim_start_iteration

    const int MAX_CMDVEL_COMPONENTS = 15;
    using cmdvel_vector_t = mrpt::math::CVectorDouble;
    // map: time -> tim_send_cmd_vel
    std::map<double, cmdvel_vector_t> cmdvels;

    double tim_start_iteration = .0;

    for (size_t i = 0; i < N; i++)
    {
      const CLogFileRecord::Ptr logsptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
      const CLogFileRecord* logptr = logsptr.get();

      {
        const auto it = logptr->timestamps.find("tim_start_iteration");
        if (it != logptr->timestamps.end())
        {
          tim_start_iteration = mrpt::Clock::toDouble(it->second);
        }
        else
        {
          tim_start_iteration++;  // just in case we don't have
                                  // valid
                                  // iteration timestamp (??)
        }
      }

      // selected PTG:
      selected_PTG_over_time[tim_start_iteration] = logptr->nSelectedPTG;

      // iter dur:
      {
        auto itExecT = logptr->values.find("executionTime");
        if (itExecT != logptr->values.end())
          iteration_duration[tim_start_iteration] = itExecT->second;
      }

      // timoff_obstacles;
      {
        const auto it = logptr->values.find("timoff_obstacles");
        if (it != logptr->values.end()) vals_timoff_obstacles[tim_start_iteration] = it->second;
      }
      // timoff_curPoseVelAge;
      {
        const auto it = logptr->values.find("timoff_curPoseVelAge");
        if (it != logptr->values.end()) vals_timoff_curPoseVelAge[tim_start_iteration] = it->second;
      }
      // timoff_sendVelCmd
      {
        const auto it = logptr->values.find("timoff_sendVelCmd");
        if (it != logptr->values.end()) vals_timoff_sendVelCmd[tim_start_iteration] = it->second;
      }

      // curPoseAndVel:
      {
        double tim_pose = tim_start_iteration;  // default

        const auto it = logptr->timestamps.find("curPoseAndVel");
        if (it != logptr->timestamps.end())
        {
          tim_pose = mrpt::Clock::toDouble(it->second);
        }

        auto& p = global_local_vel[tim_pose];
        p.pose = logptr->robotPoseLocalization;
        p.poseOdom = logptr->robotPoseOdometry;
        p.velGlobal = logptr->cur_vel;
        p.velLocal = logptr->cur_vel_local;
      }

      // send cmd vels:
      if (logptr->cmd_vel)
      {
        double tim_send_cmd_vel = tim_start_iteration;  // default
        const auto it = logptr->timestamps.find("tim_send_cmd_vel");
        if (it != logptr->timestamps.end())
        {
          tim_send_cmd_vel = mrpt::Clock::toDouble(it->second);
        }

        auto& p = cmdvels[tim_send_cmd_vel];
        p.resize(MAX_CMDVEL_COMPONENTS);
        p.setZero();
        for (size_t k = 0; k < logptr->cmd_vel->getVelCmdLength(); k++)
          p[k] = logptr->cmd_vel->getVelCmdElement(static_cast<int>(k));
      }

    }  // end for each timestep

    double t_ref = 0;
    if (!selected_PTG_over_time.empty())
    {
      t_ref = selected_PTG_over_time.begin()->first;
    }

    f << "clear; close all;\n";

    f << "% robot pose over time. Columns: [time curPoseAndVel, "
         "x,y,phi_rad,  "
         "odo_x,odo_y,odo_phi_rad]\n"
         "robot_pose = [";
    for (const auto& e : global_local_vel)
    {
      f << (e.first - t_ref) << "," << e.second.pose.x << "," << e.second.pose.y << ","
        << e.second.pose.phi << ", " << e.second.poseOdom.x << "," << e.second.poseOdom.y << ","
        << e.second.poseOdom.phi << " ; ";
    }
    f << "];\n"
         "robot_pose(:,4) = unwrap(robot_pose(:,4));\n"
         "figure(); subplot(2,1,1); \n"
         "plot(robot_pose(:, 1), robot_pose(:, 2 : 4), '.', "
         "robot_pose(:, "
         "1), "
         "robot_pose(:, 2 : 4), '-'); xlabel('Time'); legend('x', 'y', "
         "'phi "
         "(rad)'); title('robot pose'); \n"
         "xl=xlim; subplot(2,1,2);\n"
         "robot_pose_diff = diff(robot_pose(:, 2 : 4));\n"
         "idxs_rob_incr_z = find(robot_pose_diff(:, 1) == 0 & "
         "robot_pose_diff(:, 2) == 0 & robot_pose_diff(:, 3) == 0);\n"
         "plot(robot_pose(2:end, 1), diff(robot_pose(:, 2 : 4)), "
         "'.');\n"
         "hold on;\n"
         "plot(robot_pose(idxs_rob_incr_z, 1), "
         "robot_pose_diff(idxs_rob_incr_z, :), 'k', 'MarkerSize', 11, "
         "'Marker', 'square');\n"
         "xlabel('Time'); legend('x', 'y', 'phi (rad)'); title('Robot "
         "pose "
         "*increments*');\n"
         "xlim(xl);\n\n";

    f << "% Selected PTG over time. Columns: [tim_start_iteration, "
         "0-based "
         "index selected PTG]\n"
         "selected_PTG = [";
    for (const auto& e : selected_PTG_over_time)
    {
      f << (e.first - t_ref) << "," << e.second << " ; ";
    }
    f << "];\n"
         "figure(); plot(selected_PTG(:,1),selected_PTG(:,2), 'x'); "
         "xlabel('Time'); ylabel('Selected PTG');\n\n";

    if (!iteration_duration.empty())
    {
      f << "% Iteration duration. Columns: [tim_start_iteration, "
           "iter_duration_seconds]\n"
           "iteration_duration = [";
      for (const auto& e : iteration_duration)
      {
        f << (e.first - t_ref) << "," << e.second << " ; ";
      }
      f << "];\n"
           "figure(); "
           "plot(iteration_duration(:,1),iteration_duration(:,2), "
           "'x');\n"
           "hold on;\n"
           "plot(selected_PTG(1:(end-1),1),diff(selected_PTG(:,1)),'."
           "')"
           ";"
           "xlabel('Time'); legend('Iteration duration', 'Diff "
           "consecutive "
           "call time'); title('rnav_iter_call_time_duration');\n\n";
    }

    if (!vals_timoff_obstacles.empty())
    {
      f << "% vals_timoff_obstacles. Columns: [tim_start_iteration, "
           "value]\n"
           "timoff_obstacles = [";
      for (const auto& e : vals_timoff_obstacles)
      {
        f << (e.first - t_ref) << " , " << e.second << " ; ";
      }
      f << "];\n"
           "figure(); "
           "plot(timoff_obstacles(:,1),timoff_obstacles(:,2), "
           "'.',timoff_obstacles(:,1),timoff_obstacles(:,2), '-'); "
           "xlabel('Time'); title('timoff_obstacles');\n\n";
    }
    if (!vals_timoff_curPoseVelAge.empty())
    {
      f << "% vals_timoff_curPoseVelAge. Columns: "
           "[tim_start_iteration, "
           "value]\n"
           "timoff_curPoseVelAge = [";
      for (const auto& e : vals_timoff_curPoseVelAge)
      {
        f << (e.first - t_ref) << " , " << e.second << " ; ";
      }
      f << "];\n"
           "figure(); "
           "plot(timoff_curPoseVelAge(:,1),timoff_curPoseVelAge(:,2),"
           " "
           "'.',timoff_curPoseVelAge(:,1),timoff_curPoseVelAge(:,2), "
           "'-'); "
           "xlabel('Time'); title('timoff_curPoseVelAge');\n\n";
    }
    if (!vals_timoff_sendVelCmd.empty())
    {
      f << "% vals_timoff_sendVelCmd. Columns: [tim_start_iteration, "
           "value]\n"
           "timoff_sendVelCmd = [";
      for (const auto& e : vals_timoff_sendVelCmd)
      {
        f << (e.first - t_ref) << " , " << e.second << " ; ";
      }
      f << "];\n"
           "figure(); plot(timoff_sendVelCmd (:,1),timoff_sendVelCmd "
           "(:,2), "
           "'.',timoff_sendVelCmd (:,1),timoff_sendVelCmd (:,2), "
           "'-'); "
           "xlabel('Time'); title('timoff_sendVelCmd ');\n\n";
    }

    f << "% robot vel over time. Columns: [time curPoseAndVel, "
         "vx,vy,omega_rad_sec]\n"
         "robot_vel_global = [";
    for (const auto& e : global_local_vel)
    {
      f << (e.first - t_ref) << "," << e.second.velGlobal.vx << "," << e.second.velGlobal.vy << ","
        << e.second.velGlobal.omega << " ; ";
    }
    f << "];\n"
         "figure(); "
         "plot(robot_vel_global(:,1),robot_vel_global(:,2:4), "
         "'.', "
         "robot_vel_global(:,1),robot_vel_global(:,2:4), '-'); "
         "xlabel('Time'); "
         "title('Velocities (global)'); legend('vx','vy','omega');\n\n";

    f << "robot_vel_local = [";
    for (const auto& e : global_local_vel)
    {
      f << (e.first - t_ref) << "," << e.second.velLocal.vx << "," << e.second.velLocal.vy << ","
        << e.second.velLocal.omega << " ; ";
    }
    f << "];\n"
         "figure(); plot(robot_vel_local(:,1),robot_vel_local(:,2:4), "
         "'.', "
         "robot_vel_local(:,1),robot_vel_local(:,2:4), '-'); "
         "xlabel('Time'); "
         "title('Velocities (local)'); legend('vx','vy','omega');\n\n";

    // cmdvels:
    f << "% Movement commands sent to robot. Columns: [time "
         "curPoseAndVel, "
         "vx,vy,omega_rad_sec]\n"
         "cmdvels = [";
    for (const auto& e : cmdvels)
    {
      f << (e.first - t_ref) << " ";
      f << e.second << " ; ";
    }
    f << "];\n"
         "figure(); plot(cmdvels(:,1),cmdvels(:,2:"
      << (MAX_CMDVEL_COMPONENTS + 1)
      << "), '.', cmdvels(:,1),cmdvels(:,2:" << (MAX_CMDVEL_COMPONENTS + 1)
      << "), '-'); xlabel('Time'); title('Issued motion commands "
         "(meaning "
         "CVehicleVelCmd-dependend)');\n\n";
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::onManualPickAppendYaml()
{
  try
  {
    const std::string yamlFileName = "nav-selected.yaml";

    mrpt::containers::yaml d;
    if (mrpt::system::fileExists(yamlFileName))
    {
      // load for appending:
      d = mrpt::containers::yaml::FromFile(yamlFileName);
    }
    else
    {
      // Start from scratch:
      d = mrpt::containers::yaml::Sequence();
    }

    // Export individual navlog file:
    const std::string navLogSingleFile = mrpt::format("log_%.03f.navlog", m_manualPickTimestamp);

    onExportSelected(navLogSingleFile);

    // Append entry:
    mrpt::containers::yaml entry = mrpt::containers::yaml::Map();
    entry["file"] = navLogSingleFile;
    entry["entry_index"] = 0;  // since we are exporting a single-entry file
    entry["selected_ptg_index"] = m_manualPickPTGIdx;
    entry["selected_trajectory_index"] = m_manualPickTrajectoryIdx;

    d.push_back(entry);

    std::ofstream fOut(yamlFileName);
    d.printAsYAML(fOut);
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

void NavlogViewerApp::updateManualPickControls()
{
  try
  {
    m_manualPickPTGIdx = -1;
    m_manualPickTrajectoryIdx = -1;

    const int log_idx = currentLogIndex();
    if (log_idx < 0 || log_idx >= int(m_logdata.size()))
    {
      return;
    }
    auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
    const CLogFileRecord& log = *logptr;

    m_manualPickPTGMax = std::max(0, static_cast<int>(log.nPTGs) - 1);
    m_manualPickPTGSlider = std::clamp(m_manualPickPTGSlider, 0, m_manualPickPTGMax);
    const int manualPickedPTG = m_manualPickPTGSlider;

    if (manualPickedPTG >= static_cast<int>(m_logdata_ptg_paths.size()))
    {
      return;
    }
    const auto& ptg = m_logdata_ptg_paths.at(manualPickedPTG);
    if (!ptg)
    {
      m_manualPickLabel = mrpt::format("PTG#%i is not available in this log", manualPickedPTG);
      return;
    }

    m_manualPickTrajMax = std::max(0, static_cast<int>(ptg->getPathCount()) - 1);
    m_manualPickTrajSlider = std::clamp(m_manualPickTrajSlider, 0, m_manualPickTrajMax);
    const int manualPickedTraj = m_manualPickTrajSlider;

    m_manualPickLabel =
        mrpt::format("Selected PTG#%i trajectory #%i", manualPickedPTG, manualPickedTraj);

    if (m_manualPickMode)
    {
      if (!log.timestamps.empty())
      {
        m_manualPickTimestamp = mrpt::Clock::toDouble(log.timestamps.begin()->second);
      }
      m_manualPickPTGIdx = manualPickedPTG;
      m_manualPickTrajectoryIdx = manualPickedTraj;
    }
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error", mrpt::exception_to_str(e));
  }
}

// ------------------------------------------------------------------------
//     GUI
// ------------------------------------------------------------------------
namespace
{
constexpr const char* PANEL_LOG_INFO = ICON_MS_INFO " Log info###LogInfo";
constexpr const char* PANEL_VIEW = ICON_MS_TUNE " View options###ViewOptions";
constexpr const char* PANEL_DETAILS = ICON_MS_NOTES " Entry details###EntryDetails";
constexpr const char* PANEL_MANUAL_PICK = ICON_MS_ADS_CLICK " Manual pick###ManualPick";

constexpr const char* TAG_OPEN = "open";
constexpr const char* TAG_MATLAB_PLOTS = "matlab_plots";
constexpr const char* TAG_MATLAB_PATHS = "matlab_paths";
constexpr const char* TAG_PTG_PARAMS = "ptg_params";
constexpr const char* TAG_SCORES = "scores";
constexpr const char* TAG_ODOM_REPORT = "odom_report";
constexpr const char* TAG_OBSTACLES = "obstacles";
constexpr const char* TAG_EXPORT_ENTRY = "export_entry";

ImVec4 toImVec4(const mrpt::img::TColorf& c) { return {c.R, c.G, c.B, c.A}; }
}  // namespace

int NavlogViewerApp::currentLogIndex() const
{
  const auto idx = m_timeline.currentKeyIndex();
  return idx ? static_cast<int>(*idx) : -1;
}

void NavlogViewerApp::addInfoLine(const std::string& s, const mrpt::img::TColorf& c)
{
  m_infoLines.push_back({s, c});
}

void NavlogViewerApp::handleFileDialogResult(const std::vector<std::string>& files)
{
  if (files.empty())
  {
    return;
  }
  const std::string& f = files.front();
  const std::string& tag = m_fileDlg.tag;
  if (tag == TAG_OPEN)
  {
    loadLogfile(f);
  }
  else if (tag == TAG_MATLAB_PLOTS)
  {
    onMatlabPlots(f);
  }
  else if (tag == TAG_MATLAB_PATHS)
  {
    onMatlabExportPaths(f);
  }
  else if (tag == TAG_PTG_PARAMS)
  {
    onSavePTGParams(f);
  }
  else if (tag == TAG_SCORES)
  {
    onSaveScoreMatrix(f);
  }
  else if (tag == TAG_ODOM_REPORT)
  {
    onOdometryVsLocalizationReport(f);
  }
  else if (tag == TAG_OBSTACLES)
  {
    onSaveCurrentObstacles(f);
  }
  else if (tag == TAG_EXPORT_ENTRY)
  {
    onExportSelected(f);
  }
}

void NavlogViewerApp::renderMainMenu()
{
  using mrpt::imgui::CImGuiFileDialog;
  const bool hasLog = !m_logdata.empty();
  const auto saveDlg = [this](const char* tag, const char* title, const char* desc, const char* ext)
  {
    m_fileDlg.tag = tag;
    m_fileDlg.open(
        CImGuiFileDialog::Mode::Save, title,
        {
            {desc, {ext}}
    });
  };

  if (ImGui::BeginMenu("File"))
  {
    if (ImGui::MenuItem(ICON_MS_FOLDER_OPEN " Open log...", "Ctrl+O"))
    {
      m_fileDlg.tag = TAG_OPEN;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Open, "Open navigation log",
          {
              {"Navigation logs", {"reactivenavlog", "navlog"}}
      });
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_CLOSE " Quit"))
    {
      m_app.requestClose();
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Export", hasLog))
  {
    if (ImGui::MenuItem("PTG parameters (.ini)..."))
    {
      saveDlg(TAG_PTG_PARAMS, "Save PTG configuration", "INI files", "ini");
    }
    if (ImGui::MenuItem("Current obstacle map..."))
    {
      saveDlg(TAG_OBSTACLES, "Save current obstacles", "Plain text", "txt");
    }
    if (ImGui::MenuItem("Map plot to MATLAB..."))
    {
      saveDlg(TAG_MATLAB_PLOTS, "Export MATLAB/Octave script", "MATLAB/Octave script", "m");
    }
    if (ImGui::MenuItem("Paths info to MATLAB..."))
    {
      saveDlg(TAG_MATLAB_PATHS, "Export MATLAB/Octave script", "MATLAB/Octave script", "m");
    }
    if (ImGui::MenuItem("Score matrices..."))
    {
      saveDlg(TAG_SCORES, "Save score matrices", "Plain text", "txt");
    }
    if (ImGui::MenuItem("Current log entry..."))
    {
      saveDlg(TAG_EXPORT_ENTRY, "Export current log entry", "MRPT navlog file", "navlog");
    }
    if (ImGui::MenuItem("Localization vs odometry report..."))
    {
      saveDlg(TAG_ODOM_REPORT, "Save localization vs odometry report", "Plain text", "txt");
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View"))
  {
    ImGui::MenuItem(PANEL_LOG_INFO, nullptr, &m_showLogInfo);
    ImGui::MenuItem(PANEL_VIEW, nullptr, &m_showViewOptions);
    ImGui::MenuItem(PANEL_DETAILS, nullptr, &m_showDetails);
    ImGui::MenuItem(PANEL_MANUAL_PICK, nullptr, &m_showManualPick);
    ImGui::MenuItem(ICON_MS_RADAR " PTG views", nullptr, &m_showPTGViews);
    ImGui::MenuItem(ICON_MS_SHOW_CHART " Velocities", nullptr, &m_showVelPlot);
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

void NavlogViewerApp::renderPanels()
{
  bool changed = false;

  if (m_showLogInfo)
  {
    if (ImGui::Begin(PANEL_LOG_INFO, &m_showLogInfo))
    {
      mrpt::imgui::keyValueTable(
          "##loginfo", {
                           {        "File", mrpt::system::extractFileName(m_openedFileName)},
                           { "Log entries",                std::to_string(m_logdata.size())},
                           {"Log duration",                                m_txtLogDuration},
                           {"Selected PTG",                                m_txtSelectedPTG},
      });
      ImGui::TextWrapped("%s", m_txtTimeIndex.c_str());
    }
    ImGui::End();
  }

  if (m_showViewOptions)
  {
    if (ImGui::Begin(PANEL_VIEW, &m_showViewOptions))
    {
      changed |= ImGui::Checkbox("Use raw odometry", &m_useOdometryCoords);
      changed |= ImGui::Checkbox("In global coordinates", &m_globalFrame);
      changed |= ImGui::Checkbox("Show delays model-based poses", &m_showDelays);
      changed |= ImGui::Checkbox("Draw robot shape on trajectories", &m_drawShape);
      changed |= ImGui::Checkbox("Show all debug fields", &m_showAllDebugFields);
      ImGui::Checkbox("Show mouse coordinates", &m_showCursorXY);
      changed |= ImGui::Checkbox("Clearance: over path (vs pointwise)", &m_clearanceOverPath);
      ImGui::Checkbox("Orthographic 2D view", &m_ortho2DView);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
      changed |= ImGui::InputFloat("Min. distance between robot shapes", &m_shapeMinDist, 0.1f);
      m_shapeMinDist = std::max(0.01f, m_shapeMinDist);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
      changed |= ImGui::InputInt("Past poses shown", &m_numPastPoses);
      m_numPastPoses = std::max(0, m_numPastPoses);
      static const char* plots[] = {"TP-Obstacles only", "+ final scores", "+ preliminary scores"};
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
      changed |= ImGui::Combo("Shown for each PTG", &m_perPTGPlots, plots, 3);
    }
    ImGui::End();
  }

  if (m_showManualPick)
  {
    if (ImGui::Begin(PANEL_MANUAL_PICK, &m_showManualPick))
    {
      changed |= ImGui::Checkbox("Enable manual pick", &m_manualPickMode);
      ImGui::BeginDisabled(!m_manualPickMode);
      changed |= ImGui::SliderInt("PTG index", &m_manualPickPTGSlider, 0, m_manualPickPTGMax);
      changed |=
          ImGui::SliderInt("Trajectory index", &m_manualPickTrajSlider, 0, m_manualPickTrajMax);
      ImGui::TextUnformatted(m_manualPickLabel.c_str());
      if (ImGui::Button("Append to 'nav-selected.yaml'"))
      {
        onManualPickAppendYaml();
      }
      ImGui::EndDisabled();
    }
    ImGui::End();
  }

  if (changed)
  {
    updateVisualization();
  }

  if (m_showVelPlot)
  {
    if (ImGui::Begin(ICON_MS_SHOW_CHART " Velocities###Velocities", &m_showVelPlot))
    {
      // A vertical line at the current entry; clicking the plot seeks there.
      const double t0 = m_timeline.minTime();
      m_velPlot.onPlotOverlay = [this, t0]()
      {
        double x = m_timeline.time() - t0;
        if (ImPlot::DragLineX(0, &x, ImVec4(1, 1, 0, 1), 1.0f))
        {
          m_timeline.setTime(x + t0);
        }
        if (ImPlot::IsPlotHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            !ImGui::GetIO().KeyCtrl)
        {
          m_timeline.setTime(ImPlot::GetPlotMousePos().x + t0);
        }
      };
      m_velPlot.render("##vel");
    }
    ImGui::End();
  }

  if (m_showDetails)
  {
    if (ImGui::Begin(PANEL_DETAILS, &m_showDetails))
    {
      for (const auto& l : m_infoLines)
      {
        ImGui::PushStyleColor(ImGuiCol_Text, toImVec4(l.color));
        ImGui::TextUnformatted(l.text.c_str());
        ImGui::PopStyleColor();
      }
    }
    ImGui::End();
  }
}

void NavlogViewerApp::renderPTGViews()
{
  if (!m_showPTGViews)
  {
    return;
  }
  for (auto& [idx, viz] : m_mywins3D)
  {
    if (!viz.open || !viz.view)
    {
      continue;
    }
    ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(viz.title.c_str(), &viz.open))
    {
      viz.view->render();
    }
    ImGui::End();
  }
}

void NavlogViewerApp::setupDefaultLayout(unsigned int dockspaceId)
{
  ImGuiID left = 0;
  ImGuiID rest = 0;
  ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Left, 0.28f, &left, &rest);
  ImGuiID leftBottom = 0;
  ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.55f, &leftBottom, &left);
  ImGui::DockBuilderDockWindow("LogInfo", left);
  ImGui::DockBuilderDockWindow("ViewOptions", left);
  ImGui::DockBuilderDockWindow("ManualPick", left);
  ImGui::DockBuilderDockWindow("EntryDetails", leftBottom);
  ImGuiID right = 0;
  ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.30f, &right, &rest);
  ImGuiID bottom = 0;
  ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, 0.28f, &bottom, &rest);
  ImGui::DockBuilderDockWindow("Velocities", bottom);
  for (unsigned int i = 0; i < 8; i++)
  {
    ImGui::DockBuilderDockWindow(mrpt::format("ptg%u", i).c_str(), right);
  }
}

void NavlogViewerApp::renderFrame()
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

  // Keyboard shortcuts besides the timeline ones:
  const ImGuiIO& io = ImGui::GetIO();
  if (!io.WantTextInput)
  {
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
      m_about.open();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
      m_fileDlg.tag = TAG_OPEN;
      m_fileDlg.open(
          mrpt::imgui::CImGuiFileDialog::Mode::Open, "Open navigation log",
          {
              {"Navigation logs", {"reactivenavlog", "navlog"}}
      });
    }
  }

  // Restrict camera motion in 2D mode:
  auto& mainCam = m_bg.view.cameraController;
  if (m_ortho2DView)
  {
    mainCam.setElevationDegrees(90);
    mainCam.setAzimuthDegrees(-90);
  }
  mainCam.setProjectiveModel(!m_ortho2DView);

  // Copy the camera orientation into the small XYZ corner view:
  if (auto view = m_scene->getViewport("xyz"); view)
  {
    auto& xyzCam = view->getCamera();
    xyzCam.setAzimuthDegrees(mainCam.getAzimuthDegrees());
    xyzCam.setElevationDegrees(mainCam.getElevationDegrees());
    xyzCam.setOrthogonal(true);
    xyzCam.setZoomDistance(2.0);
  }

  m_bg.render(m_app);
  updateMouseXY();
  renderPanels();
  renderPTGViews();
  m_about.render();
  m_msgs.render();
}

int NavlogViewerApp::run(const std::string& fileToOpen)
{
  if (!m_app.init("navlog-viewer"))
  {
    std::cerr << "Could not create the application window.\n";
    return 1;
  }
  m_app.mainMenuBar = [this]() { renderMainMenu(); };
  m_app.beforeDockspace = [this]()
  {
    m_timeline.renderAsBottomBar();
    if (mrpt::imgui::beginStatusBar())
    {
      ImGui::TextUnformatted(m_txtTimeIndex.c_str());
      if (!m_mousePosText.empty())
      {
        ImGui::SameLine(0, ImGui::GetFontSize() * 2);
        ImGui::TextDisabled("%s", m_mousePosText.c_str());
      }
    }
    mrpt::imgui::endStatusBar();
  };
  m_app.setupDefaultLayout = [this](unsigned int id) { setupDefaultLayout(id); };
  m_app.onFilesDropped = [this](const std::vector<std::string>& files)
  {
    if (!files.empty())
    {
      loadLogfile(files.front());
    }
  };

  if (!fileToOpen.empty())
  {
    loadLogfile(fileToOpen);
  }

  m_app.run([this]() { renderFrame(); });
  return 0;
}
