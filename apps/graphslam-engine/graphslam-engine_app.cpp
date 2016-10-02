/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "TGraphSlamHandler.h"
#include "supplementary_funs.h"

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



// Properties struct for the Registration Decider Classes
// ////////////////////////////////////////////////////////////
struct TRegistrationDeciderProps {
	TRegistrationDeciderProps() {}
	~TRegistrationDeciderProps() {}

  string name;
  string description;
  string type; // type of registration decider - node/edge?
  string rawlog_format; // rawlog formats that the decider can be used in
  vector<string> observations_used;
};
vector<TRegistrationDeciderProps*> deciders_vec;

// Properties struct for the Optimizer Classes
// ////////////////////////////////////////////////////////////
struct TOptimizerProps {
	TOptimizerProps() {}
	~TOptimizerProps() {}

	string name;
	string description;

};
vector<TOptimizerProps*> optimizers_vec;

// functions for initializing decider/optimizer instances based on the user
// command line choices
// http://stackoverflow.com/a/582456/2843583
// ////////////////////////////////////////////////////////////
template<typename T>
mrpt::graphslam::deciders::CNodeRegistrationDecider<CNetworkOfPoses2DInf>* createNodeRegistrationDecider() {
	return new T;
}
template<typename T>
mrpt::graphslam::deciders::CEdgeRegistrationDecider<CNetworkOfPoses2DInf>* createEdgeRegistrationDecider() {
	return new T;
}
template<typename T>
mrpt::graphslam::optimizers::CGraphSlamOptimizer<CNetworkOfPoses2DInf>* createGraphSlamOptimizer() {
	return new T;
}


// Misc. used functions
// ////////////////////////////////////////////////////////////
void dumpRegistrarsToConsole(string reg_type);
bool checkRegistrationDeciderExists(string node_reg, string reg_type);
void dumpOptimizersToConsole();
bool checkOptimizerExists(string opt_name);
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

		// registering the available deciders
		{ // CFixedIntervalsNRD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CFixedIntervalsNRD";
			dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses odometry information for estimating the robot movement";
			dec->type = "Node";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CActionRobotMovement2D - Format #1");
			dec->observations_used.push_back("CObservationOdometry - Format #2");

			deciders_vec.push_back(dec);
		}
		{ // CICPCriteriaNRD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CICPCriteriaNRD";
			dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses 2D/3D RangeScans alignment for estimating the robot movement";
			dec->type = "Node";
			dec->rawlog_format = "#2 - Observation-only";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #2");
			dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

			deciders_vec.push_back(dec);
		}
		{ // CEmptyNRD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CEmptyNRD";
			dec->description = "Empty Decider - does nothing when its class methods are called";
			dec->type = "Node";
			dec->rawlog_format = "Both";

			deciders_vec.push_back(dec);
		}
		{ // CICPCriteriaERD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CICPCriteriaERD";
			dec->description = "Register a new edge by alligning the provided 2D/3D RangeScans of 2 nodes. Uses the goodness of the ICP Alignment as the criterium for adding a new edge";
			dec->type = "Edge";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
			dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

			deciders_vec.push_back(dec);
		}
		{ // CEmptyERD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CEmptyERD";
			dec->description = "Empty Decider - does nothing when its class methods are called";
			dec->type = "Edge";
			dec->rawlog_format = "Both";

			deciders_vec.push_back(dec);
		}
		{ // CLoopCloserERD
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CLoopCloserERD";
			dec->description = "Partition the map and register *sets* of edges based on the Pairwise consistency matrix of each set.";
			dec->type = "Edge";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");

			deciders_vec.push_back(dec);
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
				dumpRegistrarsToConsole("all");
				list_registrars = true;
			}
			if (list_node_registrars.getValue()) {
				dumpRegistrarsToConsole("node");
				list_registrars = true;
			}
			if (list_edge_registrars.getValue()) {
				dumpRegistrarsToConsole("edge");
				list_registrars = true;
			}

			if (list_optimizers.getValue()) {
				dumpOptimizersToConsole();
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
		ASSERTMSG_(checkRegistrationDeciderExists(node_reg, "node"),
				format("\nNode Registration Decider %s is not available.\n",
					node_reg.c_str()) );
		checkRegistrationDeciderExists(edge_reg, "edge");
		ASSERTMSG_(checkRegistrationDeciderExists(edge_reg, "edge"),
				format("\nEdge Registration Decider %s is not available.\n",
					edge_reg.c_str()) );
		ASSERTMSG_(checkOptimizerExists(optimizer),
				format("\nOptimizer %s is not available\n",
					optimizer.c_str()) );

		// fetch the filenames
		// ini file
		string ini_fname = arg_ini_file.getValue();
		// rawlog file - either use one or run online
		// If we run online just pass an empty rawlog_fname string to the
		// execGraphSlam call
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
		node_regs_map["CFixedIntervalsNRD"] = &createNodeRegistrationDecider<CFixedIntervalsNRD<CNetworkOfPoses2DInf> >;
		node_regs_map["CEmptyNRD"] = &createNodeRegistrationDecider<CEmptyNRD<CNetworkOfPoses2DInf> >;
		node_regs_map["CICPCriteriaNRD"] = &createNodeRegistrationDecider<CICPCriteriaNRD<CNetworkOfPoses2DInf> >;

		// edge registration deciders
		typedef std::map<std::string, CEdgeRegistrationDecider<CNetworkOfPoses2DInf>*(*)()> edge_regs_t;

		edge_regs_t edge_regs_map;
		edge_regs_map["CICPCriteriaERD"] = &createEdgeRegistrationDecider<CICPCriteriaERD<CNetworkOfPoses2DInf> >;
		edge_regs_map["CEmptyERD"] = &createEdgeRegistrationDecider<CEmptyERD<CNetworkOfPoses2DInf> >;
		edge_regs_map["CLoopCloserERD"] = &createEdgeRegistrationDecider<CLoopCloserERD<CNetworkOfPoses2DInf> >;

		// optimizers
		typedef std::map<std::string, CGraphSlamOptimizer<CNetworkOfPoses2DInf>*(*)()> optimizers_t;

		optimizers_t optimizers_map;
		optimizers_map["CLevMarqGSO"] = &createGraphSlamOptimizer<CLevMarqGSO<CNetworkOfPoses2DInf> >;


		logger.logStr(LVL_INFO, format("Node registration decider: %s", node_reg.c_str()));
		logger.logStr(LVL_INFO, format("Edge registration decider: %s", edge_reg.c_str()));
		logger.logStr(LVL_INFO, format("graphSLAM Optimizer: %s", optimizer.c_str()));

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



		// CGraphSlamEngine
		CGraphSlamEngine<CNetworkOfPoses2DInf> graphslam_engine(
				ini_fname,
				rawlog_fname,
				ground_truth_fname,
				!disable_visuals.getValue(),
				node_regs_map[node_reg](),
				edge_regs_map[edge_reg](),
				optimizers_map[optimizer]());

		// Read the dataset and pass the measurements to CGraphSlamEngine

		// Variables initialization
		CActionCollectionPtr action;
		CSensoryFramePtr observations;
		CObservationPtr observation;
		size_t curr_rawlog_entry;

		while(measurement_provider->getActionObservationPairOrObservation(
					action,
					observations,
					observation,
					curr_rawlog_entry)) {

			// actual call to the graphSLAM execution method
			graphslam_engine.execGraphSlamStep(
					action,
					observations,
					observation,
					curr_rawlog_entry);

		}

		logger.logStr(LVL_WARN, "Finished graphslam execution.");

		logger.logStr(LVL_INFO, "Generating overall report...");
		graphslam_engine.generateReportFiles();
		if (graphslam_handler.save_graph) {
			graphslam_engine.saveGraph(&graphslam_handler.save_graph_fname);
		}
		if (graphslam_handler.save_3DScene) {
			graphslam_engine.save3DScene(&graphslam_handler.save_3DScene_fname);
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

string sep_header(40, '=');
string sep_subheader(20, '-');

// Print the properties of the available registars
void dumpRegistrarsToConsole(string reg_type="all") {
	MRPT_START;

	ASSERTMSG_((strCmpI(reg_type, "node") ||
			system::strCmpI(reg_type, "edge") ||
			system::strCmpI(reg_type, "all")),
			format("Registrar string '%s' does not match a known registrar name.\n"
				"Specify 'node' 'edge' or 'all'", reg_type.c_str()));

	if ( system::strCmpI(reg_type, "node") || system::strCmpI(reg_type, "edge") ) {

		cout << endl << "Available " << system::upperCase(reg_type) << " Registration Deciders: " << endl;
		cout << sep_header << endl;
		for (vector<TRegistrationDeciderProps*>::const_iterator dec_it = deciders_vec.begin();
				dec_it != deciders_vec.end(); ++dec_it) {
			TRegistrationDeciderProps* dec = *dec_it;
			if ( system::strCmpI(dec->type, reg_type) ) {
				cout << dec->name << endl;
				cout << sep_subheader << endl;
				cout << "\t- " << "Description: " <<  dec->description << endl;
				cout << "\t- " << "Rawlog Format: " <<  dec->rawlog_format << endl;
				cout << "\t- " << "Observations that can be used: " << endl;
				for (vector<string>::const_iterator obs_it = dec->observations_used.begin();
						obs_it != dec->observations_used.end(); ++obs_it) {
					cout << "\t\t+ " << *obs_it << endl;
				}
			}
		}
	}
	else { // print both
		dumpRegistrarsToConsole("node");
		dumpRegistrarsToConsole("edge");
	}

	MRPT_END;
}

// Print the properties of the available registars
void dumpOptimizersToConsole() {
	MRPT_START;

	cout << endl << "Available GraphSlam Optimizer classes: " << endl;
	cout << sep_header << endl;

	for (vector<TOptimizerProps*>::const_iterator opt_it = optimizers_vec.begin();
			opt_it != optimizers_vec.end(); ++opt_it) {
		TOptimizerProps* opt = *opt_it;
		cout << opt->name << endl;
		cout << sep_subheader << endl;
		cout << "\t- " << "Description: " <<  opt->description << endl;
	}

	MRPT_END;
}

// check if the given registration decider is implemented
bool checkRegistrationDeciderExists(string given_reg, string reg_type) {
	MRPT_START;

	ASSERTMSG_((strCmpI(reg_type, "node") ||
			system::strCmpI(reg_type, "edge")),
			format("Registrar string '%s' does not match a known registrar name.\n"
				"Specify 'node' or 'edge' ", reg_type.c_str()));
	bool found = false;

	for (vector<TRegistrationDeciderProps*>::const_iterator dec_it = deciders_vec.begin();
			dec_it != deciders_vec.end(); ++dec_it) {
		TRegistrationDeciderProps* dec = *dec_it;
		if ( system::strCmpI(dec->type, reg_type)) {
			if (system::strCmpI(dec->name, given_reg)) {
				found = true;
				return found;
			}
		}
	}

	return found;
	MRPT_END;
}

// check if the given optimizer is implemented
bool checkOptimizerExists(string given_opt) {
	MRPT_START;

	bool found = false;

	for (vector<TOptimizerProps*>::const_iterator opt_it = optimizers_vec.begin();
			opt_it != optimizers_vec.end(); ++opt_it) {
		TOptimizerProps* opt = *opt_it;
		if ( system::strCmpI(opt->name, given_opt) ) {
			found = true;
			return found;
		}
	}

	return found;
	MRPT_END;
}


