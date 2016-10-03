/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "supplementary_funs.h"
#include "TGraphSlamHandler.h"

#include <mrpt/obs/CRawlog.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/graphslam.h>

#include <mrpt/graphslam/CRawlogMP.h>
#include <mrpt/graphslam/CRosTopicMP.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <cerrno>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;
using namespace mrpt::graphslam::optimizers;
using namespace mrpt::graphslam::measurement_providers;

using namespace std;

// command line arguments
// ////////////////////////////////////////////////////////////

TCLAP::CmdLine cmd_line(/*output message = */ " graphslam-engine - Part of the MRPT\n",
		/* delimeter = */ ' ', /* version = */ MRPT_getVersion().c_str());

TCLAP::ValueArg<string> arg_ini_file(/*flag = */ "i", /*name = */ "ini-file",
		/*desc = */ ".ini configuration file", /* required = */ true,
		/* default value = */ "", /*typeDesc = */ "config.ini", /*parser = */ cmd_line);

// Mutually Exclusive Args - Either use a rawlog file or fetch the measurements
// from a TCP Socket (online graphSLAM execution)
TCLAP::ValueArg<string> arg_rawlog_file("r", "rawlog",
		"Rawlog dataset file",	false, "", "contents.rawlog");
TCLAP::SwitchArg arg_online_exec("", "online",
		"Execute online graphSLAM", /*default = */ false);


TCLAP::ValueArg<string> arg_ground_truth_file("g", "ground-truth",
		"Ground-truth textfile",	false, "", "contents.rawlog.GT.txt", cmd_line);

// Specify the Registration Deciders to use
TCLAP::ValueArg<string> arg_node_reg("n"  , "node-reg"  , "Specify Node Registration Decider" , false , "CFixedIntervalsNRD" , "CICPCriteriaNRD" , cmd_line);
TCLAP::ValueArg<string> arg_edge_reg("e"  , "edge-reg"  , "Specify Edge Registration Decider" , false , "CICPCriteriaERD"    , "CICPCriteriaERD" , cmd_line);
TCLAP::ValueArg<string> arg_optimizer("o" , "optimizer" , "Specify GraphSlam Optimizer"       , false , "CLevMarqGSO"        , "CLevMarqGSO"     , cmd_line);

// list available deciders
TCLAP::SwitchArg list_node_registrars("" , "list-node-regs"  , "List available node registration decider classes"  , cmd_line , false);
TCLAP::SwitchArg list_edge_registrars("" , "list-edge-regs"  , "List available edge registration decider classes"  , cmd_line , false);
TCLAP::SwitchArg list_all_registrars(""  , "list-regs"       , "List (all) available registration decider classes" , cmd_line , false);
TCLAP::SwitchArg list_optimizers(""      , "list-optimizers" , "List (all) available graphslam optimizer classes"  , cmd_line , false);

// specify whether to run on headless mode - no visuals
// flag overrides all visualization related directives of the .ini file
// handy for usage when no visualization is needed or when running on
// real-robots in headless mode
TCLAP::SwitchArg disable_visuals("","disable-visuals","Disable Visualization - Overrides related visualize* directives of the .ini file",cmd_line, false);




// Main
// ////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	try {
		// initializign the logger instance
		COutputLogger logger("graphslam-engine_app");
		logger.logging_enable_keep_record = true;

		bool showHelp		 = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");


		// vectors containing the prooperties of the available deciders/optimizers
		vector<TRegistrationDeciderProps*> registrars_vec;
		vector<TOptimizerProps*> optimizers_vec;

		// registering the available deciders
		{ // CFixedIntervalsNRD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CFixedIntervalsNRD";
			dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses odometry information for estimating the robot movement";
			dec->type = "Node";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CActionRobotMovement2D - Format #1");
			dec->observations_used.push_back("CObservationOdometry - Format #2");

			registrars_vec.push_back(dec);
		}
		{ // CICPCriteriaNRD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CICPCriteriaNRD";
			dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses 2D/3D RangeScans alignment for estimating the robot movement";
			dec->type = "Node";
			dec->rawlog_format = "#2 - Observation-only";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #2");
			dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

			registrars_vec.push_back(dec);
		}
		{ // CEmptyNRD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CEmptyNRD";
			dec->description = "Empty Decider - does nothing when its class methods are called";
			dec->type = "Node";
			dec->rawlog_format = "Both";

			registrars_vec.push_back(dec);
		}
		{ // CICPCriteriaERD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CICPCriteriaERD";
			dec->description = "Register a new edge by alligning the provided 2D/3D RangeScans of 2 nodes. Uses the goodness of the ICP Alignment as the criterium for adding a new edge";
			dec->type = "Edge";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
			dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

			registrars_vec.push_back(dec);
		}
		{ // CEmptyERD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CEmptyERD";
			dec->description = "Empty Decider - does nothing when its class methods are called";
			dec->type = "Edge";
			dec->rawlog_format = "Both";

			registrars_vec.push_back(dec);
		}
		{ // CLoopCloserERD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CLoopCloserERD";
			dec->description = "Partition the map and register *sets* of edges based on the Pairwise consistency matrix of each set.";
			dec->type = "Edge";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");

			registrars_vec.push_back(dec);
		}

		// registering the available optimizers
		{ // CLevMarqGSO
			TOptimizerProps* opt = new TOptimizerProps;
			opt->name = "CLevMarqGSO";
			opt->description = "Levenberg-Marqurdt non-linear graphSLAM solver";

			optimizers_vec.push_back(opt);
		}

		// state the mutually exclusive variables
		cmd_line.xorAdd(arg_rawlog_file, arg_online_exec);

		// Input Validation
		if (!cmd_line.parse( argc, argv ) ||  showVersion || showHelp) {
			return 0;
		}
		// fetch the command line options
		// ////////////////////////////////////////////////////////////

		// decide whether to display the help messages for the deciders/optimizers
		{
			bool list_registrars = false;

			if (list_all_registrars.getValue()) {
				dumpRegistrarsToConsole(registrars_vec, "all");
				list_registrars = true;
			}
			if (list_node_registrars.getValue()) {
				dumpRegistrarsToConsole(registrars_vec, "node");
				list_registrars = true;
			}
			if (list_edge_registrars.getValue()) {
				dumpRegistrarsToConsole(registrars_vec, "edge");
				list_registrars = true;
			}

			if (list_optimizers.getValue()) {
				dumpOptimizersToConsole(optimizers_vec);
			}

			if (list_registrars || list_optimizers.getValue()) {
				logger.logStr(LVL_INFO, "Exiting.. ");
				return 0;
			}
		}

		// fetch which registration deciders / optimizer to use
		string node_reg = arg_node_reg.getValue();
		string edge_reg = arg_edge_reg.getValue();
		string optimizer = arg_optimizer.getValue();
		ASSERTMSG_(checkRegistrationDeciderExists(registrars_vec, node_reg, "node"),
				format("\nNode Registration Decider %s is not available.\n",
					node_reg.c_str()) );
		ASSERTMSG_(checkRegistrationDeciderExists(registrars_vec, edge_reg, "edge"),
				format("\nEdge Registration Decider %s is not available.\n",
					edge_reg.c_str()) );
		ASSERTMSG_(checkOptimizerExists(optimizers_vec, optimizer),
				format("\nOptimizer %s is not available\n",
					optimizer.c_str()) );

		// fetch the filenames
		// ini file
		string ini_fname = arg_ini_file.getValue();
		// rawlog file - either use one or run online
		string rawlog_fname = arg_rawlog_file.isSet()? arg_rawlog_file.getValue(): "";
		ASSERT_(arg_rawlog_file.isSet() ^ arg_online_exec.isSet());

		// ground-truth file
		string ground_truth_fname;
		if ( arg_ground_truth_file.isSet() ) {
			ground_truth_fname = arg_ground_truth_file.getValue();
		}

		if (disable_visuals.getValue()) { // enabling Visualization objects
			logger.logStr(LVL_WARN, "Running on headless mode - Visuals disabled");
		}

		//
		// assemble maps of available deciders/optimizers
		//

		// node registration deciders
		typedef std::map<std::string, CNodeRegistrationDecider<CNetworkOfPoses2DInf>*(*)()> node_regs_t;

		node_regs_t node_regs_map;
		node_regs_map["CFixedIntervalsNRD"] =
			&createNodeRegistrationDecider<CFixedIntervalsNRD<CNetworkOfPoses2DInf> >;
		node_regs_map["CEmptyNRD"] =
			&createNodeRegistrationDecider<CEmptyNRD<CNetworkOfPoses2DInf> >;
		node_regs_map["CICPCriteriaNRD"] =
			&createNodeRegistrationDecider<CICPCriteriaNRD<CNetworkOfPoses2DInf> >;

		// edge registration deciders
		typedef std::map<std::string, CEdgeRegistrationDecider<CNetworkOfPoses2DInf>*(*)()> edge_regs_t;

		edge_regs_t edge_regs_map;
		edge_regs_map["CICPCriteriaERD"] =
			&createEdgeRegistrationDecider<CICPCriteriaERD<CNetworkOfPoses2DInf> >;
		edge_regs_map["CEmptyERD"] =
			&createEdgeRegistrationDecider<CEmptyERD<CNetworkOfPoses2DInf> >;
		edge_regs_map["CLoopCloserERD"] =
			&createEdgeRegistrationDecider<CLoopCloserERD<CNetworkOfPoses2DInf> >;

		// optimizers
		typedef std::map<std::string, CGraphSlamOptimizer<CNetworkOfPoses2DInf>*(*)()> optimizers_t;

		optimizers_t optimizers_map;
		optimizers_map["CLevMarqGSO"] =
			&createGraphSlamOptimizer<CLevMarqGSO<CNetworkOfPoses2DInf> >;

		logger.logFmt(LVL_INFO, "Node registration decider: %s", node_reg.c_str());
		logger.logFmt(LVL_INFO, "Edge registration decider: %s", edge_reg.c_str());
		logger.logFmt(LVL_INFO, "graphSLAM Optimizer: %s", optimizer.c_str());

		// Initialize class responsible for reading in the measurements
		mrpt::graphslam::measurement_providers::CMeasurementProvider*
			measurement_provider = NULL;

		// Decide where to read the measurements from
		if (rawlog_fname.empty()) { // run in online mode
			logger.logFmt(LVL_WARN, "Executing online graphSLAM");
			measurement_provider = new CRosTopicMP();
		}
		else {
			logger.logFmt(LVL_WARN, "Executing graphSLAM using rawlog files");
			measurement_provider = new CRawlogMP();
			dynamic_cast<CRawlogMP*>(measurement_provider)->setRawlogFname(rawlog_fname);
		}
		measurement_provider->loadParams(ini_fname);

		// Initialization of related objects
		// TGraphSlamHandler
		TGraphSlamHandler graphslam_handler;
		graphslam_handler.setOutputLoggerPtr(&logger);
		graphslam_handler.readConfigFname(ini_fname);

		// initialize visuals
		if (!disable_visuals.getValue()) {
			graphslam_handler.initVisualization();
		}


		// CGraphSlamEngine
		CGraphSlamEngine<CNetworkOfPoses2DInf> graphslam_engine(
				ini_fname,
				rawlog_fname,
				ground_truth_fname,
				graphslam_handler.win_manager,
				node_regs_map[node_reg](),
				edge_regs_map[edge_reg](),
				optimizers_map[optimizer]());

		// Variables initialization
		CActionCollectionPtr action;
		CSensoryFramePtr observations;
		CObservationPtr observation;
		size_t curr_rawlog_entry;

		// Read the dataset and pass the measurements to CGraphSlamEngine
		while(measurement_provider->getActionObservationPairOrObservation(
					action,
					observations,
					observation,
					curr_rawlog_entry)) {

			bool break_exec = graphslam_handler.queryObserverForEvents();
			if (break_exec) {
				break;
			}

			// actual call to the graphSLAM execution method
			graphslam_engine.execGraphSlamStep(
					action,
					observations,
					observation,
					curr_rawlog_entry);

		}
		logger.logFmt(LVL_WARN, "Finished graphslam execution.");

		logger.logFmt(LVL_INFO, "Generating overall report...");
		graphslam_engine.generateReportFiles();
		// save the 3DScene and the graph
		if (graphslam_handler.save_graph) {
			std::string save_graph_fname = 
				graphslam_handler.output_dir_fname +
				"/" +
				graphslam_handler.save_graph_fname;
			graphslam_engine.saveGraph(&save_graph_fname);
		}
		if (!disable_visuals.getValue() && graphslam_handler.save_3DScene) {
			std::string save_3DScene_fname = 
				graphslam_handler.output_dir_fname +
				"/" +
				graphslam_handler.save_3DScene_fname;

			graphslam_engine.save3DScene(&save_3DScene_fname);
		}


		////////////////////////////////////////////////////////////////////////

	}
	catch (exception& e) {
		setConsoleColor(CONCOL_RED, true);
		cerr << "Program finished with an exception!" << endl;
		setConsoleColor(CONCOL_NORMAL, true);

		cerr << e.what() << endl;

		mrpt::system::pause();
		return -1;
	}
	catch (...) {
		setConsoleColor(CONCOL_RED, true);
		cerr << "Program finished for an untyped exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL, true);

		mrpt::system::pause();
		return -1;
	}

	return 0;
}
