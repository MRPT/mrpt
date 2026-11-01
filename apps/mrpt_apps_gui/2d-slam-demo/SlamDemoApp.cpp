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

#include "SlamDemoApp.h"

#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder API
#include <imgui_stdlib.h>
#include <implot.h>
#include <mrpt/core/format.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::maps;
using mrpt::imgui::CImGuiFileDialog;
using mrpt::imgui::tooltip;

namespace
{
constexpr const char* APP_NAME = "2d-slam-demo";

constexpr const char* TAG_SAVE_STATE = "save_state";
constexpr const char* TAG_SAVE_DA = "save_da";
constexpr const char* TAG_RAWLOG = "rawlog";
constexpr const char* TAG_LOAD_CFG = "load_cfg";
constexpr const char* TAG_SAVE_CFG = "save_cfg";
constexpr const char* TAG_MAP_FILE = "map_file";

constexpr double RAD2DEGSQ = (180.0 / M_PI) * (180.0 / M_PI);

const ImVec4 RED(1.0f, 0.25f, 0.25f, 1.0f);
const ImVec4 BLUE(0.35f, 0.55f, 1.0f, 1.0f);
const ImVec4 GREEN(0.3f, 0.9f, 0.3f, 1.0f);
const ImVec4 GREY(0.7f, 0.7f, 0.7f, 1.0f);

ImPlotSpec lineSpec(const ImVec4& col, float w = 1.5f)
{
  ImPlotSpec s;
  s.LineColor = col;
  s.LineWeight = w;
  return s;
}

/** Draws the "quantiles"-sigma ellipse of a 2D Gaussian (cov [a b; b c]). */
void plotCovEllipse(
    const char* label,
    double cx,
    double cy,
    double a,
    double b,
    double c,
    double quantiles,
    const ImVec4& col,
    float w = 1.5f)
{
  const double t1 = 0.5 * (a + c);
  const double t2 = std::sqrt(0.25 * (a - c) * (a - c) + b * b);
  const double l1 = std::max(0.0, t1 + t2);
  const double l2 = std::max(0.0, t1 - t2);
  const double ang = 0.5 * std::atan2(2 * b, a - c);
  const double r1 = quantiles * std::sqrt(l1);
  const double r2 = quantiles * std::sqrt(l2);
  constexpr int N = 40;
  double xs[N + 1];
  double ys[N + 1];
  for (int i = 0; i <= N; i++)
  {
    const double t = 2 * M_PI * i / N;
    const double ex = r1 * std::cos(t);
    const double ey = r2 * std::sin(t);
    xs[i] = cx + ex * std::cos(ang) - ey * std::sin(ang);
    ys[i] = cy + ex * std::sin(ang) + ey * std::cos(ang);
  }
  ImPlot::PlotLine(label, xs, ys, N + 1, lineSpec(col, w));
}

/** The robot as a triangle at a 2D pose. */
void plotRobot(const char* label, const CPose2D& p, const ImVec4& col)
{
  const double lx[4] = {0, 0, 0.2, 0};
  const double ly[4] = {-0.1, 0.1, 0, -0.1};
  double xs[4];
  double ys[4];
  for (int i = 0; i < 4; i++)
  {
    double gx = 0;
    double gy = 0;
    p.composePoint(lx[i], ly[i], gx, gy);
    xs[i] = gx;
    ys[i] = gy;
  }
  ImPlot::PlotLine(label, xs, ys, 4, lineSpec(col, 2.0f));
}

/** The area seen by the sensor, at a robot pose. */
void plotSensorArea(
    const char* label, const SlamDemoSimulator::TSimulationOptions& o, const CPose2D& robot)
{
  constexpr size_t N = 60;
  std::vector<double> xs;
  std::vector<double> ys;
  const CPose2D sensorPose = robot + o.sensorOnTheRobot;
  for (size_t i = 0; i < N; i++)
  {
    const double a = o.sensor_fov * (-0.5 + static_cast<double>(i) / (N - 1));
    double gx = 0;
    double gy = 0;
    sensorPose.composePoint(
        o.sensor_max_range * std::cos(a), o.sensor_max_range * std::sin(a), gx, gy);
    xs.push_back(gx);
    ys.push_back(gy);
  }
  xs.push_back(sensorPose.x());
  ys.push_back(sensorPose.y());
  xs.push_back(xs.front());
  ys.push_back(ys.front());
  ImPlot::PlotLine(label, xs.data(), ys.data(), static_cast<int>(xs.size()), lineSpec(GREY, 1.0f));
}

/** Sensor noise covariance of a range-bearing reading. */
CMatrixDouble22 sensorNoise(const CRangeBearingKFSLAM2D& slam)
{
  CMatrixDouble22 NOISE;
  NOISE(0, 0) = square(slam.options.std_sensor_range);
  NOISE(1, 1) = square(slam.options.std_sensor_yaw);
  return NOISE;
}
}  // namespace

SlamDemoApp::SlamDemoApp(SlamDemoSimulator& sim) : m_sim(sim)
{
  m_about.appName = APP_NAME;
  m_about.description =
      "Interactive simulator of range-bearing EKF-SLAM with data association.\n"
      "Run with --help to see the command line arguments (batch experiments).\n"
      "Inspired by a MATLAB version written by J. Neira and J. Tardos.";
  m_about.extraTabs.emplace_back(
      "Keyboard",
      "R     : Reset\n"
      "S     : One step\n"
      "Space : Run / stop\n"
      "B     : Batch run\n"
      "F1    : About\n");

  for (auto* p : {&m_plotErrX, &m_plotErrY, &m_plotErrPhi, &m_plotTimes, &m_plotDA})
  {
    p->xLabel = "Step";
    p->showToolbar = false;
  }
  m_plotErrX.yLabel = "X error (m)";
  m_plotErrY.yLabel = "Y error (m)";
  m_plotErrPhi.yLabel = "Phi error (deg)";
  m_plotTimes.yLabel = "Time (ms)";
  m_plotDA.yLabel = "Count";

  m_console.attach(m_sim.slam);
}

void SlamDemoApp::reset()
{
  m_running = false;
  try
  {
    m_sim.reset();
    m_statusMsg = mrpt::format("Reset: %zu landmarks.", m_sim.gtMap.landmarks.size());
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error resetting the simulation", mrpt::exception_to_str(e));
  }
  m_plottedSteps = SIZE_MAX;
}

void SlamDemoApp::startBatch()
{
  // The same experiment as the command line batch mode: from the start, with
  // the same number of steps.
  size_t N = 0;
  try
  {
    N = m_sim.defaultBatchSteps();
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Cannot run the batch experiment", mrpt::exception_to_str(e));
    return;
  }
  reset();
  m_batch.start(
      [this, N](mrpt::imgui::AsyncTaskContext& ctx)
      {
        mrpt::system::CTicTac const tic;
        for (size_t i = 0; i < N && !ctx.cancelRequested(); i++)
        {
          m_sim.step();
          ctx.setProgress(static_cast<double>(i + 1) / static_cast<double>(N));
        }
        return tic.Tac();
      });
}

void SlamDemoApp::stepOnce()
{
  try
  {
    mrpt::system::CTicTac const tic;
    m_sim.step();
    m_statusMsg = mrpt::format("Step %zu done in %.03f ms", m_sim.historic.size(), 1e3 * tic.Tac());
  }
  catch (const std::exception& e)
  {
    m_running = false;
    m_msgs.error("Error in the simulation step", mrpt::exception_to_str(e));
    std::cerr << mrpt::exception_to_str(e) << "\n";
  }
}

void SlamDemoApp::updatePlotsIfNeeded()
{
  const auto& H = m_sim.historic;
  if (m_plottedSteps == H.size())
  {
    return;
  }
  m_plottedSteps = H.size();

  const size_t N = H.size();
  std::vector<double> t(N);
  std::vector<double> ex(N);
  std::vector<double> ey(N);
  std::vector<double> ep(N);
  std::vector<double> bx(N);
  std::vector<double> by(N);
  std::vector<double> bp(N);
  std::vector<double> times(N);
  std::vector<double> tp(N);
  std::vector<double> tn(N);
  std::vector<double> fp(N);
  std::vector<double> fn(N);
  std::vector<double> jcbb(N);
  for (size_t i = 0; i < N; i++)
  {
    const auto& h = H[i];
    t[i] = static_cast<double>(i);
    ex[i] = h.GT_robot_pose.x() - h.estimate_robot_pose.mean.x();
    ey[i] = h.GT_robot_pose.y() - h.estimate_robot_pose.mean.y();
    ep[i] = RAD2DEG(wrapToPi(h.GT_robot_pose.phi() - h.estimate_robot_pose.mean.phi()));
    bx[i] = 3 * std::sqrt(h.estimate_robot_pose.cov(0, 0));
    by[i] = 3 * std::sqrt(h.estimate_robot_pose.cov(1, 1));
    bp[i] = RAD2DEG(3 * std::sqrt(h.estimate_robot_pose.cov(2, 2)));
    times[i] = 1e3 * h.run_time;
    tp[i] = h.da_true_pos;
    tn[i] = h.da_true_neg;
    fp[i] = h.da_false_pos;
    fn[i] = h.da_false_neg;
    jcbb[i] = static_cast<double>(h.jcbb_iters);
  }
  const auto neg = [](std::vector<double> v)
  {
    for (auto& x : v)
    {
      x = -x;
    }
    return v;
  };
  const auto errPlot = [&](mrpt::imgui::CImGuiPlotPanel& p, const std::vector<double>& err,
                           const std::vector<double>& bound)
  {
    p.clear();
    p.setBand("3-sigma bound", t, neg(bound), bound);
    p.seriesOrCreate("3-sigma bound").color = mrpt::img::TColor(0x40, 0xd0, 0x40);
    p.setSeries("error", t, err);
    p.seriesOrCreate("error").color = mrpt::img::TColor(0xff, 0xff, 0xff);
  };
  errPlot(m_plotErrX, ex, bx);
  errPlot(m_plotErrY, ey, by);
  errPlot(m_plotErrPhi, ep, bp);

  m_plotTimes.clear();
  m_plotTimes.setSeries("KF update time", t, times);

  m_plotDA.clear();
  m_plotDA.setSeries("True positives", t, tp);
  m_plotDA.setSeries("True negatives", t, tn);
  m_plotDA.setSeries("False positives", t, fp);
  m_plotDA.setSeries("False negatives", t, fn);
  m_plotDA.setSeries("JCBB iterations", t, jcbb);
  for (auto& s : {"True positives", "True negatives", "False positives", "False negatives"})
  {
    m_plotDA.seriesOrCreate(s).style = mrpt::imgui::CImGuiPlotPanel::Style::Stairs;
  }
}

// ---------------------------------------------------------------------------
// Plots
// ---------------------------------------------------------------------------
void SlamDemoApp::renderGroundTruth()
{
  if (!ImPlot::BeginPlot("Ground truth", ImVec2(-1, -1), ImPlotFlags_Equal))
  {
    return;
  }
  ImPlot::SetupAxes("x (m)", "y (m)");
  const auto& lms = m_sim.gtMap.landmarks;
  if (lms.size() > 0)
  {
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto& lm : lms)
    {
      xs.push_back(lm.pose_mean.x);
      ys.push_back(lm.pose_mean.y);
    }
    ImPlotSpec s;
    s.Marker = ImPlotMarker_Circle;
    s.MarkerSize = 3;
    s.MarkerFillColor = BLUE;
    s.MarkerLineColor = BLUE;
    ImPlot::PlotScatter(
        mrpt::format("Landmarks (%zu)", lms.size()).c_str(), xs.data(), ys.data(),
        static_cast<int>(xs.size()), s);
  }
  plotSensorArea("Sensor range", m_sim.options, m_sim.gtPose);
  plotRobot("Robot", m_sim.gtPose, RED);
  ImPlot::EndPlot();
}

void SlamDemoApp::renderObservation()
{
  if (!ImPlot::BeginPlot("Observation (robot frame)", ImVec2(-1, -1), ImPlotFlags_Equal))
  {
    return;
  }
  ImPlot::SetupAxes("x (m)", "y (m)");
  const auto& o = m_sim.options;
  plotSensorArea("Sensor range", o, CPose2D());
  plotRobot("Robot", CPose2D(), RED);

  const auto NOISE = sensorNoise(m_sim.slam);
  for (size_t k = 0; k < m_sim.lastObservation.sensedData.size(); k++)
  {
    const auto& d = m_sim.lastObservation.sensedData[k];
    const double hr = d.range;
    const double ha = d.yaw;
    const double cphi = std::cos(o.sensorOnTheRobot.phi() + ha);
    const double sphi = std::sin(o.sensorOnTheRobot.phi() + ha);
    const auto lm = o.sensorOnTheRobot + CPoint2D(hr * std::cos(ha), hr * std::sin(ha));
    // C = J * NOISE * J^T, with NOISE diagonal:
    const double j00 = cphi;
    const double j01 = -hr * sphi;
    const double j10 = sphi;
    const double j11 = hr * cphi;
    const double n0 = NOISE(0, 0);
    const double n1 = NOISE(1, 1);
    const double c00 = j00 * j00 * n0 + j01 * j01 * n1;
    const double c01 = j00 * j10 * n0 + j01 * j11 * n1;
    const double c11 = j10 * j10 * n0 + j11 * j11 * n1;
    plotCovEllipse("##obs", lm.x(), lm.y(), c00, c01, c11, 3, RED);
    const std::string lbl = d.landmarkID != mrpt::obs::INVALID_LANDMARK_ID
                                ? mrpt::format("#%u", static_cast<unsigned>(d.landmarkID))
                                : std::string("?");
    ImPlot::PlotText(lbl.c_str(), lm.x(), lm.y(), ImVec2(0, -12));
  }
  ImPlot::EndPlot();
}

void SlamDemoApp::renderEstimatedMap()
{
  CPosePDFGaussian estRobotPose;
  std::vector<TPoint2D> LMs;
  std::map<unsigned int, CLandmark::TLandmarkID> landmarkIDs;
  CVectorDouble Xkk;
  CMatrixDouble Pkk;
  m_sim.slam.getCurrentState(estRobotPose, LMs, landmarkIDs, Xkk, Pkk);

  const auto title = mrpt::format("Estimated map (%zu landmarks)###estmap", LMs.size());
  if (!ImPlot::BeginPlot(title.c_str(), ImVec2(-1, -1), ImPlotFlags_Equal))
  {
    return;
  }
  ImPlot::SetupAxes("x (m)", "y (m)");
  plotRobot("Robot", estRobotPose.mean, RED);
  plotCovEllipse(
      "Robot 3-sigma", estRobotPose.mean.x(), estRobotPose.mean.y(), estRobotPose.cov(0, 0),
      estRobotPose.cov(0, 1), estRobotPose.cov(1, 1), 3, RED);
  for (size_t i = 0; i < LMs.size(); i++)
  {
    const auto idx = static_cast<int>(3 + 2 * i);
    if (idx + 1 >= Pkk.rows())
    {
      break;
    }
    plotCovEllipse(
        "Landmarks 3-sigma", LMs[i].x, LMs[i].y, Pkk(idx, idx), Pkk(idx, idx + 1),
        Pkk(idx + 1, idx + 1), 3, BLUE);
    std::string lbl = mrpt::format("#%zu", i);
    if (m_sim.options.show_map_real_correspondences)
    {
      if (const auto it = m_sim.estimatedIDX2realIDX.find(i);
          it != m_sim.estimatedIDX2realIDX.end())
      {
        lbl += mrpt::format("->%zu", it->second);
      }
    }
    ImPlot::PlotText(lbl.c_str(), LMs[i].x, LMs[i].y, ImVec2(0, -12));
  }
  ImPlot::EndPlot();
}

void SlamDemoApp::renderIndividualCompatibility()
{
  if (!ImPlot::BeginPlot("Individual compatibility (range-bearing space)", ImVec2(-1, -1)))
  {
    return;
  }
  const auto& o = m_sim.options;
  const double halfFov = RAD2DEG(o.sensor_fov) * 0.5;
  ImPlot::SetupAxes("Bearing (deg)", "Range (m)");
  ImPlot::SetupAxesLimits(
      -halfFov - 10, halfFov + 10, -0.1, o.sensor_max_range + 0.5, ImPlotCond_Always);

  {
    const double xs[5] = {-halfFov, halfFov, halfFov, -halfFov, -halfFov};
    const double ys[5] = {0, 0, o.sensor_max_range, o.sensor_max_range, 0};
    ImPlot::PlotLine("Sensor range", xs, ys, 5, lineSpec(GREY, 1.0f));
  }

  const auto NOISE = sensorNoise(m_sim.slam);
  const auto& obs = m_sim.lastObservation.sensedData;
  const auto& da = m_sim.slam.getLastDataAssociation();
  const size_t obs_size = m_sim.slam.get_observation_size();

  for (size_t i = 0; i < obs.size(); i++)
  {
    const double hr = obs[i].range;
    const double ha = obs[i].yaw;
    plotCovEllipse(
        "Observations", RAD2DEG(ha), hr, RAD2DEGSQ * NOISE(1, 1), 0, NOISE(0, 0), 2, RED);
    ImPlot::PlotText(mrpt::format("O%zu", i).c_str(), RAD2DEG(ha), hr, ImVec2(0, -12));
  }
  for (size_t i = 0; i < da.predictions_IDs.size(); i++)
  {
    const auto row = static_cast<int>(i);
    const auto r0 = static_cast<int>(obs_size * i);
    const double hr = da.Y_pred_means(row, 0);
    const double ha = da.Y_pred_means(row, 1);
    double a = 0;
    double b = 0;
    double c = 0;
    if (static_cast<size_t>(da.Y_pred_covs.cols()) == obs_size)
    {
      a = RAD2DEGSQ * da.Y_pred_covs(r0 + 1, 1);
      b = RAD2DEG(da.Y_pred_covs(r0 + 1, 0));
      c = da.Y_pred_covs(r0, 0);
    }
    else
    {
      a = RAD2DEGSQ * da.Y_pred_covs(r0 + 1, r0 + 1);
      b = RAD2DEG(da.Y_pred_covs(r0 + 1, r0));
      c = da.Y_pred_covs(r0, r0);
    }
    plotCovEllipse("Predictions", RAD2DEG(ha), hr, a, b, c, 2, BLUE);
    ImPlot::PlotText(
        mrpt::format("P%u", static_cast<unsigned>(da.predictions_IDs[i])).c_str(), RAD2DEG(ha), hr,
        ImVec2(0, 12));
  }
  // Individually compatible pairs (thick: the final association):
  for (unsigned int ob = 0; ob < obs.size(); ob++)
  {
    for (unsigned int p = 0; p < da.predictions_IDs.size(); p++)
    {
      if (static_cast<int>(p) >= da.results.indiv_compatibility.rows() ||
          static_cast<int>(ob) >= da.results.indiv_compatibility.cols() ||
          !da.results.indiv_compatibility(p, ob))
      {
        continue;
      }
      const auto it = da.results.associations.find(ob);
      const bool isFinal =
          it != da.results.associations.end() && it->second == da.predictions_IDs[p];
      const double xs[2] = {RAD2DEG(da.Y_pred_means(p, 1)), RAD2DEG(obs[ob].yaw)};
      const double ys[2] = {da.Y_pred_means(p, 0), obs[ob].range};
      ImPlot::PlotLine(
          isFinal ? "Final association" : "Compatible", xs, ys, 2,
          lineSpec(isFinal ? GREEN : GREY, isFinal ? 3.0f : 1.0f));
    }
  }
  ImPlot::EndPlot();
}

void SlamDemoApp::renderDataAssociationTable()
{
  const auto& da = m_sim.slam.getLastDataAssociation();
  const auto& obs = m_sim.lastObservation.sensedData;
  const size_t nP = da.predictions_IDs.size();
  const size_t nO = obs.size();
  ImGui::TextDisabled(
      "Distances between predictions (rows) and observations (columns); "
      "the final associations are highlighted.");
  if (nP == 0 || nO == 0 || static_cast<size_t>(da.results.indiv_distances.rows()) < nP ||
      static_cast<size_t>(da.results.indiv_distances.cols()) < nO)
  {
    ImGui::TextUnformatted("(no data association in the last step)");
    return;
  }
  const ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollX |
                                ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;
  const int nCols = static_cast<int>(std::min<size_t>(nO, 300)) + 1;
  if (ImGui::BeginTable("##da", nCols, flags))
  {
    ImGui::TableSetupScrollFreeze(1, 1);
    ImGui::TableSetupColumn("");
    for (int o = 0; o + 1 < nCols; o++)
    {
      ImGui::TableSetupColumn(mrpt::format("O%d", o).c_str());
    }
    ImGui::TableHeadersRow();
    const ImU32 hl = ImGui::GetColorU32(ImVec4(0.7f, 0.6f, 0.0f, 0.6f));
    for (size_t p = 0; p < nP; p++)
    {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::Text("P%u", static_cast<unsigned>(da.predictions_IDs[p]));
      for (int o = 0; o + 1 < nCols; o++)
      {
        ImGui::TableNextColumn();
        const double v = da.results.indiv_distances(static_cast<int>(p), o);
        const auto it = da.results.associations.find(static_cast<size_t>(o));
        if (it != da.results.associations.end() && it->second == da.predictions_IDs[p])
        {
          ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, hl);
        }
        if (v > 500)
        {
          ImGui::TextUnformatted("inf");
        }
        else if (v < -950)
        {
          ImGui::TextUnformatted("-inf");
        }
        else
        {
          ImGui::Text("%.02f", v);
        }
      }
    }
    ImGui::EndTable();
  }
}

void SlamDemoApp::renderErrors()
{
  const float h = std::max(80.0f, (ImGui::GetContentRegionAvail().y - 8) / 3.0f);
  m_plotErrX.render("Error in X", h);
  m_plotErrY.render("Error in Y", h);
  m_plotErrPhi.render("Error in heading", h);
}

void SlamDemoApp::renderStats()
{
  unsigned int TP = 0;
  unsigned int TN = 0;
  unsigned int FP = 0;
  unsigned int FN = 0;
  for (const auto& h : m_sim.historic)
  {
    TP += h.da_true_pos;
    TN += h.da_true_neg;
    FP += h.da_false_pos;
    FN += h.da_false_neg;
  }
  mrpt::imgui::keyValueTable(
      "##totals", {
                      {          "Steps", std::to_string(m_sim.historic.size())},
                      { "True positives",                    std::to_string(TP)},
                      { "True negatives",                    std::to_string(TN)},
                      {"False positives",                    std::to_string(FP)},
                      {"False negatives",                    std::to_string(FN)},
  });
  const float h = std::max(80.0f, (ImGui::GetContentRegionAvail().y - 8) / 2.0f);
  m_plotDA.render("Data association per step", h);
  m_plotTimes.render("Computation time", h);
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
void SlamDemoApp::renderParams()
{
  auto& o = m_sim.options;
  auto& kf = m_sim.slam.KF_options;
  auto& so = m_sim.slam.options;
  ImGui::PushItemWidth(ImGui::GetFontSize() * 8);

  ImGui::TextDisabled("Changes apply from the next step; map changes on Reset.");

  if (ImGui::CollapsingHeader("Kalman filter", ImGuiTreeNodeFlags_DefaultOpen))
  {
    static const char* methods[] = {
        "EKF (naive)", "EKF one-scalar-at-a-time (Davison)", "IKF (full)"};
    int m = static_cast<int>(kf.method);
    if (ImGui::Combo("Method##kf", &m, methods, 3))
    {
      kf.method = static_cast<mrpt::bayes::TKFMethod>(m);
    }
    ImGui::BeginDisabled(kf.method != mrpt::bayes::kfIKFFull);
    ImGui::InputInt("IKF iterations", &kf.IKF_iterations);
    kf.IKF_iterations = std::max(1, kf.IKF_iterations);
    ImGui::EndDisabled();
    bool numTran = !kf.use_analytic_transition_jacobian;
    if (ImGui::Checkbox("Numeric Jacobian: transition model", &numTran))
    {
      kf.use_analytic_transition_jacobian = !numTran;
    }
    bool numObs = !kf.use_analytic_observation_jacobian;
    if (ImGui::Checkbox("Numeric Jacobian: observation model", &numObs))
    {
      kf.use_analytic_observation_jacobian = !numObs;
    }
  }

  if (ImGui::CollapsingHeader("Robot motion", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::InputDouble("Square path length (m)", &o.path_square_len, 0.5, 1.0, "%.2f");
    ImGui::InputDouble("Step length (m)", &o.robot_step_length, 0.01, 0.1, "%.3f");
    ImGui::InputDouble("Odometry sigma XY (m)", &o.odometry_noise_std_xy, 0.001, 0.01, "%.4f");
    double phiDeg = RAD2DEG(o.odometry_noise_std_phi);
    if (ImGui::InputDouble("Odometry sigma phi (deg)", &phiDeg, 0.01, 0.1, "%.3f"))
    {
      o.odometry_noise_std_phi = DEG2RAD(phiDeg);
    }
    ImGui::InputDouble(
        "Odometry uncertainty overestimation", &o.uncert_overestim_odom, 0.05, 0.1, "%.2f");
  }

  if (ImGui::CollapsingHeader("Sensor", ImGuiTreeNodeFlags_DefaultOpen))
  {
    float sr = so.std_sensor_range;
    if (ImGui::InputFloat("Range sigma (m)", &sr, 0.01f, 0.1f, "%.3f"))
    {
      so.std_sensor_range = std::max(1e-4f, sr);
    }
    float sy = static_cast<float>(RAD2DEG(so.std_sensor_yaw));
    if (ImGui::InputFloat("Bearing sigma (deg)", &sy, 0.1f, 1.0f, "%.3f"))
    {
      so.std_sensor_yaw = static_cast<float>(DEG2RAD(std::max(1e-4f, sy)));
    }
    ImGui::InputDouble(
        "Sensor uncertainty overestimation", &o.uncert_overestim_sensor, 0.05, 0.1, "%.2f");
    ImGui::InputDouble("Min. range (m)", &o.sensor_min_range, 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Max. range (m)", &o.sensor_max_range, 0.1, 1.0, "%.2f");
    double fovDeg = RAD2DEG(o.sensor_fov);
    if (ImGui::InputDouble("Field of view (deg)", &fovDeg, 1.0, 10.0, "%.1f"))
    {
      o.sensor_fov = DEG2RAD(std::clamp(fovDeg, 1.0, 360.0));
    }
    double sp[3] = {
        o.sensorOnTheRobot.x(), o.sensorOnTheRobot.y(), RAD2DEG(o.sensorOnTheRobot.phi())};
    if (ImGui::InputScalarN("Pose on robot (x,y,phi deg)", ImGuiDataType_Double, sp, 3))
    {
      o.sensorOnTheRobot = CPose2D(sp[0], sp[1], DEG2RAD(sp[2]));
    }
    ImGui::Checkbox("Sensor identifies the landmarks (no D.A.)", &o.sensorDistingishesLandmarks);
    ImGui::InputDouble("Spurious readings: mean", &o.spurious_count_mean, 0.1, 1.0, "%.2f");
    ImGui::InputDouble("Spurious readings: std", &o.spurious_count_std, 0.1, 1.0, "%.2f");
  }

  if (ImGui::CollapsingHeader("Data association", ImGuiTreeNodeFlags_DefaultOpen))
  {
    static const char* daMethods[] = {"Nearest neighbor (NN)", "JCBB"};
    static const char* metrics[] = {"Mahalanobis", "Matching likelihood"};
    int dm = static_cast<int>(so.data_assoc_method);
    if (ImGui::Combo("Method##da", &dm, daMethods, 2))
    {
      so.data_assoc_method = static_cast<TDataAssociationMethod>(dm);
    }
    int met = static_cast<int>(so.data_assoc_metric);
    if (ImGui::Combo("Distance", &met, metrics, 2))
    {
      so.data_assoc_metric = static_cast<TDataAssociationMetric>(met);
    }
    int icm = static_cast<int>(so.data_assoc_IC_metric);
    if (ImGui::Combo("Ind. compatibility", &icm, metrics, 2))
    {
      so.data_assoc_IC_metric = static_cast<TDataAssociationMetric>(icm);
    }
    ImGui::InputDouble("IC chi2 threshold", &so.data_assoc_IC_chi2_thres, 0.01, 0.1, "%.4f");
    ImGui::InputDouble("IC ML threshold", &so.data_assoc_IC_ml_threshold, 0.1, 1.0, "%.4f");
    ImGui::Checkbox("Show estimated->real landmark indices", &o.show_map_real_correspondences);
  }

  if (ImGui::CollapsingHeader("Ground truth map", ImGuiTreeNodeFlags_DefaultOpen))
  {
    int kind = o.map_generator == "1" ? 0 : (o.map_generator == "2" ? 1 : 2);
    bool changed = false;
    changed |= ImGui::RadioButton("Square corridor", &kind, 0);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Random", &kind, 1);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("From file", &kind, 2);
    if (kind == 1)
    {
      int n = static_cast<int>(o.randomMap_nLMs);
      if (ImGui::InputInt("Landmarks", &n))
      {
        o.randomMap_nLMs = static_cast<uint32_t>(std::max(1, n));
      }
    }
    if (kind == 2)
    {
      if (m_mapFile.empty() && o.map_generator != "1" && o.map_generator != "2")
      {
        m_mapFile = o.map_generator;
      }
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14);
      if (ImGui::InputTextWithHint("##mapfile", "Text file, one 'x y' per row", &m_mapFile))
      {
        changed = true;
      }
      ImGui::SameLine();
      if (ImGui::Button(ICON_MS_FOLDER_OPEN "##browsemap"))
      {
        m_fileDlg.tag = TAG_MAP_FILE;
        m_fileDlg.open(
            CImGuiFileDialog::Mode::Open, "Landmarks map file",
            {
                {"Text files", {"txt"}}
        });
      }
    }
    if (changed)
    {
      o.map_generator = kind == 0 ? "1" : (kind == 1 ? "2" : m_mapFile);
    }
    ImGui::InputInt("Random seed (-1: random)", &o.random_seed);
  }

  ImGui::Separator();
  ImGui::SliderInt("Step period when running (ms)", &m_stepPeriodMs, 0, 500);
  ImGui::PopItemWidth();
  if (ImGui::Button(ICON_MS_RESTART_ALT " Reset with these parameters"))
  {
    reset();
  }
}

void SlamDemoApp::renderProfiler()
{
  if (!m_showProfiler)
  {
    return;
  }
  ImGui::SetNextWindowSize(ImVec2(700, 400), ImGuiCond_FirstUseEver);
  if (ImGui::Begin(ICON_MS_TIMELINE " Profiler###Profiler", &m_showProfiler))
  {
    if (ImGui::Button(ICON_MS_DELETE " Reset statistics"))
    {
      m_sim.slam.getProfiler().clear();
    }
    ImGui::Separator();
    ImGui::BeginChild(
        "##prof", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::TextUnformatted(m_sim.slam.getProfiler().getStatsAsText().c_str());
    ImGui::EndChild();
  }
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Menus, toolbar, layout
// ---------------------------------------------------------------------------
void SlamDemoApp::renderMainMenu()
{
  const bool busy = m_batch.isRunning();
  if (ImGui::BeginMenu("Simulation"))
  {
    if (ImGui::MenuItem(ICON_MS_RESTART_ALT " Reset", "R", false, !busy))
    {
      reset();
    }
    if (ImGui::MenuItem(ICON_MS_STEP " One step", "S", false, !busy && !m_running))
    {
      stepOnce();
    }
    if (ImGui::MenuItem(ICON_MS_PLAY_ARROW " Run", "Space", false, !busy && !m_running))
    {
      m_running = true;
    }
    if (ImGui::MenuItem(ICON_MS_STOP " Stop", "Space", false, m_running))
    {
      m_running = false;
    }
    if (ImGui::MenuItem(ICON_MS_FAST_FORWARD " Run batch (non-interactive)", "B", false, !busy))
    {
      startBatch();
    }
    ImGui::Separator();
    if (ImGui::MenuItem(ICON_MS_CLOSE " Quit"))
    {
      m_app.requestClose();
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Tools", !busy))
  {
    if (ImGui::MenuItem(ICON_MS_FOLDER_OPEN " Load parameters..."))
    {
      m_fileDlg.tag = TAG_LOAD_CFG;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Open, "Load parameters",
          {
              {"Config files", {"ini", "cfg"}}
      });
    }
    if (ImGui::MenuItem(ICON_MS_SAVE " Save parameters..."))
    {
      m_fileDlg.tag = TAG_SAVE_CFG;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::Save, "Save parameters",
          {
              {"Config files", {"ini", "cfg"}}
      });
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Save filter state..."))
    {
      m_fileDlg.tag = TAG_SAVE_STATE;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::SelectFolder,
          "Directory for kf_mean.txt, kf_cov.txt and slam.3Dscene");
    }
    if (ImGui::MenuItem("Save last data association..."))
    {
      m_fileDlg.tag = TAG_SAVE_DA;
      m_fileDlg.open(
          CImGuiFileDialog::Mode::SelectFolder,
          "Directory for prediction_IDs.txt, prediction_means.txt and prediction_cov.txt");
    }
    bool saving = m_sim.isSavingRawlog();
    if (ImGui::MenuItem("Save simulated data to a rawlog", nullptr, &saving))
    {
      if (saving)
      {
        m_fileDlg.tag = TAG_RAWLOG;
        m_fileDlg.open(
            CImGuiFileDialog::Mode::Save, "Save rawlog",
            {
                {"Rawlogs", {"rawlog"}}
        });
      }
      else
      {
        m_sim.setRawlogOutput({});
      }
    }
    ImGui::Separator();
    ImGui::MenuItem(ICON_MS_TIMELINE " Profiler", nullptr, &m_showProfiler);
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View"))
  {
    ImGui::MenuItem("Maps and observation", nullptr, &m_showMaps);
    ImGui::MenuItem("Data association", nullptr, &m_showDA);
    ImGui::MenuItem("Errors", nullptr, &m_showErrors);
    ImGui::MenuItem("Statistics", nullptr, &m_showStats);
    ImGui::MenuItem("Parameters", nullptr, &m_showParams);
    ImGui::MenuItem("Log", nullptr, &m_showLog);
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

void SlamDemoApp::renderToolbar()
{
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar;
  const float h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2;
  if (ImGui::BeginViewportSideBar("##toolbar", ImGui::GetMainViewport(), ImGuiDir_Up, h, flags))
  {
    const bool busy = m_batch.isRunning();
    ImGui::BeginDisabled(busy);
    if (ImGui::Button(ICON_MS_RESTART_ALT " Reset"))
    {
      reset();
    }
    tooltip("Reset the simulation (R)");
    ImGui::SameLine();
    ImGui::BeginDisabled(m_running);
    if (ImGui::Button(ICON_MS_STEP " Step"))
    {
      stepOnce();
    }
    ImGui::EndDisabled();
    tooltip("Run one step (S)");
    ImGui::SameLine();
    if (ImGui::Button(m_running ? ICON_MS_STOP " Stop" : ICON_MS_PLAY_ARROW " Run"))
    {
      m_running = !m_running;
    }
    tooltip("Run / stop the interactive simulation (Space)");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MS_FAST_FORWARD " Batch"))
    {
      startBatch();
    }
    tooltip("Run a full non-interactive experiment (B)");
    ImGui::EndDisabled();
    if (busy)
    {
      ImGui::SameLine();
      ImGui::ProgressBar(
          static_cast<float>(m_batch.progress()), ImVec2(ImGui::GetFontSize() * 12, 0));
      ImGui::SameLine();
      if (ImGui::Button("Cancel"))
      {
        // Wait for the current step: the worker modifies the simulator.
        m_batch.cancelAndWait();
      }
    }
  }
  ImGui::End();
}

void SlamDemoApp::handleFileDialogResult(const std::vector<std::string>& files)
{
  if (files.empty())
  {
    return;
  }
  const std::string& f = files.front();
  const std::string& tag = m_fileDlg.tag;
  if (tag == TAG_SAVE_STATE)
  {
    m_sim.saveFilterState(f);
    m_statusMsg = "Filter state saved to: " + f;
  }
  else if (tag == TAG_SAVE_DA)
  {
    m_sim.saveLastDataAssociation(f);
    m_statusMsg = "Data association saved to: " + f;
  }
  else if (tag == TAG_RAWLOG)
  {
    m_sim.setRawlogOutput(f);
    m_statusMsg = "Saving simulated data to: " + f;
  }
  else if (tag == TAG_LOAD_CFG)
  {
    m_sim.loadConfig(f);
    reset();
  }
  else if (tag == TAG_SAVE_CFG)
  {
    m_sim.saveConfig(f);
  }
  else if (tag == TAG_MAP_FILE)
  {
    m_mapFile = f;
    m_sim.options.map_generator = f;
  }
}

void SlamDemoApp::setupDefaultLayout(unsigned int dockspaceId)
{
  // Left: parameters; center: 2x2 maps; right: DA/stats; bottom: errors.
  ImGuiID left = 0;
  ImGuiID rest = 0;
  ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Left, 0.22f, &left, &rest);
  ImGui::DockBuilderDockWindow("Parameters", left);
  ImGuiID right = 0;
  ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.30f, &right, &rest);
  ImGuiID rightBottom = 0;
  ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.45f, &rightBottom, &right);
  ImGui::DockBuilderDockWindow("Errors", right);
  ImGui::DockBuilderDockWindow("Statistics", rightBottom);
  ImGui::DockBuilderDockWindow("DA matrix", rightBottom);
  ImGui::DockBuilderDockWindow("Log", rightBottom);
  ImGuiID top = 0;
  ImGuiID bottom = 0;
  ImGui::DockBuilderSplitNode(rest, ImGuiDir_Up, 0.5f, &top, &bottom);
  ImGuiID tl = 0;
  ImGuiID tr = 0;
  ImGui::DockBuilderSplitNode(top, ImGuiDir_Left, 0.5f, &tl, &tr);
  ImGuiID bl = 0;
  ImGuiID br = 0;
  ImGui::DockBuilderSplitNode(bottom, ImGuiDir_Left, 0.5f, &bl, &br);
  ImGui::DockBuilderDockWindow("Ground truth", tl);
  ImGui::DockBuilderDockWindow("Estimated map", tr);
  ImGui::DockBuilderDockWindow("Observation", bl);
  ImGui::DockBuilderDockWindow("Ind. compatibility", br);
}

void SlamDemoApp::renderFrame()
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

  try
  {
    if (auto t = m_batch.poll(); t)
    {
      const auto st = m_sim.computeStats(*t);
      m_statusMsg = mrpt::format("%zu steps done in %.03f s", st.steps, *t);
    }
  }
  catch (const std::exception& e)
  {
    m_msgs.error("Error in the batch run", mrpt::exception_to_str(e));
  }

  const bool busy = m_batch.isRunning();

  // Keyboard shortcuts:
  const ImGuiIO& io = ImGui::GetIO();
  if (!io.WantTextInput && !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId))
  {
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
      m_about.open();
    }
    if (!busy && ImGui::IsKeyPressed(ImGuiKey_R, false))
    {
      reset();
    }
    if (!busy && !m_running && ImGui::IsKeyPressed(ImGuiKey_S))
    {
      stepOnce();
    }
    if (!busy && ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
      m_running = !m_running;
    }
    if (!busy && ImGui::IsKeyPressed(ImGuiKey_B, false))
    {
      startBatch();
    }
  }

  // Interactive run:
  if (m_running && !busy && ImGui::GetTime() - m_lastStepTime >= m_stepPeriodMs * 1e-3)
  {
    m_lastStepTime = ImGui::GetTime();
    stepOnce();
  }

  // A keyboard shortcut above may have just started a batch run:
  if (busy || m_batch.isRunning())
  {
    // The simulator is being updated by the batch run: do not read it now.
    ImGui::SetNextWindowPos(
        ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin(
            "##busy", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoSavedSettings))
    {
      ImGui::Text("Running batch experiment... %.0f%%", 100 * m_batch.progress());
    }
    ImGui::End();
    m_msgs.render();
    return;
  }

  updatePlotsIfNeeded();

  const auto panel = [](const char* title, bool* open, const auto& contents)
  {
    if (!*open)
    {
      return;
    }
    if (ImGui::Begin(title, open))
    {
      contents();
    }
    ImGui::End();
  };

  panel(ICON_MS_MAP " Ground truth###Ground truth", &m_showMaps, [&]() { renderGroundTruth(); });
  panel(ICON_MS_MAP " Estimated map###Estimated map", &m_showMaps, [&]() { renderEstimatedMap(); });
  panel(ICON_MS_RADAR " Observation###Observation", &m_showMaps, [&]() { renderObservation(); });
  panel(
      ICON_MS_SCATTER_PLOT " Individual compatibility###Ind. compatibility", &m_showDA,
      [&]() { renderIndividualCompatibility(); });
  panel(ICON_MS_SHOW_CHART " Errors###Errors", &m_showErrors, [&]() { renderErrors(); });
  panel(
      ICON_MS_TABLE_ROWS " DA matrix###DA matrix", &m_showDA,
      [&]() { renderDataAssociationTable(); });
  panel(ICON_MS_QUERY_STATS " Statistics###Statistics", &m_showStats, [&]() { renderStats(); });
  panel(ICON_MS_TUNE " Parameters###Parameters", &m_showParams, [&]() { renderParams(); });
  panel(ICON_MS_TERMINAL " Log###Log", &m_showLog, [&]() { m_console.render(); });
  renderProfiler();

  m_about.render();
  m_msgs.render();
}

int SlamDemoApp::run()
{
  mrpt::imgui::CImGuiAppShell::Options opts;
  opts.width = 1500;
  opts.height = 950;
  if (!m_app.init(APP_NAME, opts))
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
      ImGui::TextUnformatted(m_statusMsg.c_str());
    }
    mrpt::imgui::endStatusBar();
  };
  m_app.setupDefaultLayout = [this](unsigned int id) { setupDefaultLayout(id); };

  if (m_sim.historic.empty() && m_sim.gtMap.landmarks.size() == 0)
  {
    reset();
  }

  m_app.run([this]() { renderFrame(); });
  m_batch.cancelAndWait();  // it modifies the simulator
  return 0;
}
