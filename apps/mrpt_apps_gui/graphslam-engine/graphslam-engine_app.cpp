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

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <mrpt/graphslam/apps_related/CGraphSlamHandler.h>
#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>
#include <mrpt/system/COutputLogger.h>

#include <CLI/CLI.hpp>
#include <string>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;
using namespace mrpt::graphslam::optimizers;
using namespace mrpt::graphslam::apps;
using namespace std;

template <class GRAPH_T>
void execGraphSlamEngine(
    mrpt::system::COutputLogger* logger,
    bool list_node_regs,
    bool list_edge_regs,
    bool list_all_regs,
    bool list_opts,
    bool disable_visuals_flag,
    const string& ini_fname,
    const string& rawlog_fname,
    const string& ground_truth_fname,
    const string& node_reg,
    const string& edge_reg,
    const string& optimizer)
{
  // Instance for managing the available graphslam deciders optimizers
  TUserOptionsChecker<GRAPH_T> options_checker;
  options_checker.createDeciderOptimizerMappings();
  options_checker.populateDeciderOptimizerProperties();

  // fetch the command line options
  // ////////////////////////////////////////////////////////////

  // decide whether to display the help messages for the deciders/optimizers
  {
    bool list_registrars = false;

    if (list_all_regs)
    {
      options_checker.dumpRegistrarsToConsole("all");
      list_registrars = true;
    }
    if (list_node_regs)
    {
      options_checker.dumpRegistrarsToConsole("node");
      list_registrars = true;
    }
    if (list_edge_regs)
    {
      options_checker.dumpRegistrarsToConsole("edge");
      list_registrars = true;
    }

    if (list_opts)
    {
      options_checker.dumpOptimizersToConsole();
    }

    if (list_registrars || list_opts)
    {
      logger->logFmt(LVL_INFO, "Exiting.. ");
      return;
    }
  }

  if (disable_visuals_flag)
  {  // enabling Visualization objects
    logger->logFmt(LVL_WARN, "Running on headless mode - Visuals disabled");
  }

  logger->logFmt(LVL_INFO, "Node registration decider: %s", node_reg.c_str());
  logger->logFmt(LVL_INFO, "Edge registration decider: %s", edge_reg.c_str());
  logger->logFmt(LVL_INFO, "graphSLAM Optimizer: %s", optimizer.c_str());

  // CGraphSlamHandler initialization
  CGraphSlamHandler<GRAPH_T> graphslam_handler(logger, &options_checker, !disable_visuals_flag);
  graphslam_handler.setFNames(ini_fname, rawlog_fname, ground_truth_fname);

  graphslam_handler.initEngine(node_reg, edge_reg, optimizer);
  graphslam_handler.printParams();
  graphslam_handler.execute();
}

// Main
// ////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // initializign the logger instance
  COutputLogger logger("graphslam-engine_app");
  logger.logging_enable_keep_record = true;

  try
  {
    CLI::App app(" graphslam-engine - Part of the MRPT");
    app.set_version_flag("--version", MRPT_getVersion());

    std::string ini_file;
    app.add_option("-i,--ini-file", ini_file, ".ini configuration file")->required();

    std::string rawlog_file;
    app.add_option("-r,--rawlog", rawlog_file, "Rawlog dataset file")->required();

    std::string ground_truth_file;
    app.add_option("-g,--ground-truth", ground_truth_file, "Ground-truth textfile");

    std::string node_reg = "CFixedIntervalsNRD";
    app.add_option("-n,--node-reg", node_reg, "Specify Node Registration Decider");

    std::string edge_reg = "CICPCriteriaERD";
    app.add_option("-e,--edge-reg", edge_reg, "Specify Edge Registration Decider");

    std::string optimizer = "CLevMarqGSO";
    app.add_option("-o,--optimizer", optimizer, "Specify GraphSlam Optimizer");

    bool list_node_regs = false;
    app.add_flag(
        "--list-node-regs", list_node_regs, "List available node registration decider classes");

    bool list_edge_regs = false;
    app.add_flag(
        "--list-edge-regs", list_edge_regs, "List available edge registration decider classes");

    bool list_all_regs = false;
    app.add_flag("--list-regs", list_all_regs, "List (all) available registration decider classes");

    bool list_opts = false;
    app.add_flag(
        "--list-optimizers", list_opts, "List (all) available graphslam optimizer classes");

    bool disable_visuals_flag = false;
    app.add_flag(
        "--disable-visuals", disable_visuals_flag,
        "Disable Visualization - Overrides related visualize* directives of the .ini file");

    bool dim_2d = false;
    app.add_flag("--2d", dim_2d, "Construct 2D graph of constraints (use CPosePDFGaussianInf)");

    bool dim_3d = false;
    app.add_flag("--3d", dim_3d, "Construct 3D graph of constraints (use CPose3DPDFGaussianInf)");

    // Input Validation
    CLI11_PARSE(app, argc, argv);

    // Ensure exactly one of --2d or --3d is provided
    if ((dim_2d && dim_3d) || (!dim_2d && !dim_3d))
    {
      std::cerr << "Error: Exactly one of --2d or --3d must be specified"
                << "\n";
      return 1;
    }

    // CGraphSlamEngine initialization
    if (dim_2d)
    {
      execGraphSlamEngine<CNetworkOfPoses2DInf>(
          &logger, list_node_regs, list_edge_regs, list_all_regs, list_opts, disable_visuals_flag,
          ini_file, rawlog_file, ground_truth_file, node_reg, edge_reg, optimizer);
    }
    else
    {
      execGraphSlamEngine<CNetworkOfPoses3DInf>(
          &logger, list_node_regs, list_edge_regs, list_all_regs, list_opts, disable_visuals_flag,
          ini_file, rawlog_file, ground_truth_file, node_reg, edge_reg, optimizer);
    }

    return 0;
  }
  catch (const std::exception& e)
  {
    const auto se = mrpt::exception_to_str(e);
    logger.logStr(LVL_ERROR, se);
    std::cerr << se << "\n";
    mrpt::system::pause();
    return -1;
  }

  return 0;
}
