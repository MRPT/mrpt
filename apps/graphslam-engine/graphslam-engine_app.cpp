/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/system/COutputLogger.h>

#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <mrpt/graphslam/apps_related/CGraphSlamHandler.h>
#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>

#include <mrpt/3rdparty/tclap/CmdLine.h>

#include <string>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;
using namespace mrpt::graphslam::optimizers;
using namespace mrpt::graphslam::apps;
using namespace std;

// command line arguments
// ////////////////////////////////////////////////////////////

TCLAP::CmdLine cmd_line(
	/*output message = */ " graphslam-engine - Part of the MRPT\n",
	/* delimeter = */ ' ', /* version = */ MRPT_getVersion().c_str());

TCLAP::ValueArg<string> arg_ini_file(
	/*flag = */ "i", /*name = */ "ini-file",
	/*desc = */ ".ini configuration file", /* required = */ true,
	/* default value = */ "", /*typeDesc = */ "config.ini",
	/*parser = */ cmd_line);

TCLAP::ValueArg<string> arg_rawlog_file(
	"r", "rawlog", "Rawlog dataset file", true, "", "contents.rawlog",
	/*parser = */ cmd_line);

TCLAP::ValueArg<string> arg_ground_truth_file(
	"g", "ground-truth", "Ground-truth textfile", false, "",
	"contents.rawlog.GT.txt", cmd_line);

// Specify the Registration Deciders to use
TCLAP::ValueArg<string> arg_node_reg(
	"n", "node-reg", "Specify Node Registration Decider", false,
	"CFixedIntervalsNRD", "CICPCriteriaNRD", cmd_line);
TCLAP::ValueArg<string> arg_edge_reg(
	"e", "edge-reg", "Specify Edge Registration Decider", false,
	"CICPCriteriaERD", "CICPCriteriaERD", cmd_line);
TCLAP::ValueArg<string> arg_optimizer(
	"o", "optimizer", "Specify GraphSlam Optimizer", false, "CLevMarqGSO",
	"CLevMarqGSO", cmd_line);

// list available deciders
TCLAP::SwitchArg list_node_registrars(
	"", "list-node-regs", "List available node registration decider classes",
	cmd_line, false);
TCLAP::SwitchArg list_edge_registrars(
	"", "list-edge-regs", "List available edge registration decider classes",
	cmd_line, false);
TCLAP::SwitchArg list_all_registrars(
	"", "list-regs", "List (all) available registration decider classes",
	cmd_line, false);
TCLAP::SwitchArg list_optimizers(
	"", "list-optimizers", "List (all) available graphslam optimizer classes",
	cmd_line, false);

// specify whether to run on headless mode - no visuals
// flag overrides all visualization related directives of the .ini file
// handy for usage when no visualization is needed or when running on
// real-robots in headless mode
TCLAP::SwitchArg disable_visuals(
	"", "disable-visuals",
	"Disable Visualization - Overrides related visualize* directives of the "
	".ini file",
	cmd_line, false);

TCLAP::SwitchArg dim_2d(
	"", "2d", "Construct 2D graph of constraints (use CPosePDFGaussianInf)");
TCLAP::SwitchArg dim_3d(
	"", "3d", "Construct 3D graph of constraints (use CPose3DPDFGaussianInf)");

template <class GRAPH_T>
void execGraphSlamEngine(mrpt::system::COutputLogger* logger)
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

		if (list_all_registrars.getValue())
		{
			options_checker.dumpRegistrarsToConsole("all");
			list_registrars = true;
		}
		if (list_node_registrars.getValue())
		{
			options_checker.dumpRegistrarsToConsole("node");
			list_registrars = true;
		}
		if (list_edge_registrars.getValue())
		{
			options_checker.dumpRegistrarsToConsole("edge");
			list_registrars = true;
		}

		if (list_optimizers.getValue())
		{
			options_checker.dumpOptimizersToConsole();
		}

		if (list_registrars || list_optimizers.getValue())
		{
			logger->logFmt(LVL_INFO, "Exiting.. ");
			return;
		}
	}

	// fetch the filenames
	// ini file
	string ini_fname = arg_ini_file.getValue();
	// rawlog file
	string rawlog_fname = arg_rawlog_file.getValue();

	// ground-truth file
	string ground_truth_fname;
	if (arg_ground_truth_file.isSet())
	{
		ground_truth_fname = arg_ground_truth_file.getValue();
	}

	if (disable_visuals.getValue())
	{  // enabling Visualization objects
		logger->logFmt(LVL_WARN, "Running on headless mode - Visuals disabled");
	}

	// fetch which registration deciders / optimizer to use
	string node_reg = arg_node_reg.getValue();
	string edge_reg = arg_edge_reg.getValue();
	string optimizer = arg_optimizer.getValue();
	logger->logFmt(LVL_INFO, "Node registration decider: %s", node_reg.c_str());
	logger->logFmt(LVL_INFO, "Edge registration decider: %s", edge_reg.c_str());
	logger->logFmt(LVL_INFO, "graphSLAM Optimizer: %s", optimizer.c_str());

	// CGraphSlamHandler initialization
	CGraphSlamHandler<GRAPH_T> graphslam_handler(
		logger, &options_checker, !disable_visuals.getValue());
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
		bool showHelp = argc > 1 && !os::_strcmp(argv[1], "--help");
		bool showVersion = argc > 1 && !os::_strcmp(argv[1], "--version");

		// Input Validation
		cmd_line.xorAdd(dim_2d, dim_3d);
		if (!cmd_line.parse(argc, argv) || showVersion || showHelp)
		{
			return 0;
		}

		// CGraphSlamEngine initialization
		if (dim_2d.getValue())
		{
			execGraphSlamEngine<CNetworkOfPoses2DInf>(&logger);
		}
		else
		{
			execGraphSlamEngine<CNetworkOfPoses3DInf>(&logger);
		}

		return 0;
	}
	catch (const std::exception& e)
	{
		const auto se = mrpt::exception_to_str(e);
		logger.logStr(LVL_ERROR, se);
		std::cerr << se << std::endl;
		mrpt::system::pause();
		return -1;
	}

	return 0;
}
