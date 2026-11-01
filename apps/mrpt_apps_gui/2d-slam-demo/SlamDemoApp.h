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

#include <mrpt/imgui/AsyncTask.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/imgui/CImGuiLogConsole.h>
#include <mrpt/imgui/CImGuiPlotPanel.h>
#include <mrpt/imgui/widgets.h>

#include <string>

#include "SlamDemoSimulator.h"

/** The 2d-slam-demo GUI: plots of the ground truth, observations, estimated
 * map, data association and errors, over a SlamDemoSimulator. */
class SlamDemoApp
{
 public:
  explicit SlamDemoApp(SlamDemoSimulator& sim);

  /** Runs the GUI until the window is closed. */
  int run();

 private:
  SlamDemoSimulator& m_sim;

  mrpt::imgui::CImGuiAppShell m_app;
  mrpt::imgui::CImGuiFileDialog m_fileDlg;
  mrpt::imgui::CImGuiMessageQueue m_msgs;
  mrpt::imgui::CImGuiAboutDialog m_about;
  mrpt::imgui::CImGuiLogConsole m_console;

  mrpt::imgui::CImGuiPlotPanel m_plotErrX;
  mrpt::imgui::CImGuiPlotPanel m_plotErrY;
  mrpt::imgui::CImGuiPlotPanel m_plotErrPhi;
  mrpt::imgui::CImGuiPlotPanel m_plotTimes;
  mrpt::imgui::CImGuiPlotPanel m_plotDA;
  size_t m_plottedSteps = SIZE_MAX;

  /** Batch run in the background: returns its total time (s). */
  mrpt::imgui::AsyncTask<double> m_batch;

  bool m_running = false;
  double m_lastStepTime = 0;
  int m_stepPeriodMs = 20;
  std::string m_statusMsg;

  // Panels:
  bool m_showMaps = true;
  bool m_showDA = true;
  bool m_showErrors = true;
  bool m_showStats = true;
  bool m_showParams = true;
  bool m_showLog = false;
  bool m_showProfiler = false;

  // Edited map file name (when the map comes from a file):
  std::string m_mapFile;

  void reset();
  void stepOnce();
  void startBatch();

  void updatePlotsIfNeeded();
  void renderMainMenu();
  void renderToolbar();
  void renderGroundTruth();
  void renderObservation();
  void renderEstimatedMap();
  void renderIndividualCompatibility();
  void renderDataAssociationTable();
  void renderErrors();
  void renderStats();
  void renderParams();
  void renderProfiler();
  void renderFrame();
  void handleFileDialogResult(const std::vector<std::string>& files);
  void setupDefaultLayout(unsigned int dockspaceId);
};
