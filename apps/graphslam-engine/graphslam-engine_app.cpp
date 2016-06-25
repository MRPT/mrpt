/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/obs/CRawlog.h>
#include <mrpt/graphslam.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/system/string_utils.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <cerrno>

#include "CGraphSlamEngine.h"
#include "CWindowObserver.h"

// node/edge registration deciders
#include "CFixedIntervalsNRD.h"
#include "CICPGoodnessERD.h"
#include "CICPGoodnessNRD.h"
#include "CEmptyNRD.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::graphslam::deciders;

using namespace std;

#define VERBOSE_COUT	if (verbose) std::cout << "[graphslam_engine] "

// command line arguments
// ////////////////////////////////////////////////////////////

TCLAP::CmdLine cmd(/*output message = */ " graphslam-engine - Part of the MRPT\n",
		/* delimeter = */ ' ', /* version = */ MRPT_getVersion().c_str());

TCLAP::ValueArg<string> arg_ini_file(/*flag = */ "i", /*name = */ "ini_file",
		/*desc = */ ".ini configuration file", /* required = */ true,
		/* default value = */ "", /*typeDesc = */ "config.ini", /*parser = */ cmd);
TCLAP::ValueArg<string> arg_rawlog_file( "r", "rawlog",
		"Rawlog dataset file",	true, "", "contents.rawlog", cmd);

// OPTIONAL - If dataset was generated from GridMapNavSimul program and the
// visualize_ground_truth is set to true in the .ini file, the ground truth is
// automatically found
TCLAP::ValueArg<string> arg_ground_truth_file( "g", "ground-truth",
		"Ground-truth textfile",	false, "", "contents.rawlog.GT.txt", cmd);

// Specify the Registration Deciders to use
TCLAP::ValueArg<string> arg_node_reg("n", "node-reg",
		"Specify Node registration decider",	false, "CFixedIntervalsNRD", "CICPGoodnessNRD", cmd);
TCLAP::ValueArg<string> arg_edge_reg("e", "edge-reg",
		"Specify Edge registration decider",	false, "CICPGoodnessERD", "CICPGoodnessERD", cmd);

// list available graphs, deciders_vec
TCLAP::SwitchArg list_node_registrars("","list-node-regs","List available node registration decider classes",cmd, false);
TCLAP::SwitchArg list_edge_registrars("","list-edge-regs","List available edge registration decider classes",cmd, false);
TCLAP::SwitchArg list_all_registrars("","list-regs","List (all) available registration decider classes",cmd, false);

CWindowObserver  graph_win_observer;

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

// Used Functions
// ////////////////////////////////////////////////////////////
void dumpRegistrarsToConsole(string reg_type); 
bool checkRegistrationDeciderExists(string node_reg, string reg_type);
// Main
// ////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	try {

		bool showHelp		 = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

		{
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CFixedIntervalsNRD";
			dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses odometry information for estimating the robot movement";
			dec->type = "Node";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CActionRobotMovement2D - Format #1");
			dec->observations_used.push_back("CObservationOdometry - Format #2");

			deciders_vec.push_back(dec);
		}
		{
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CICPGoodnessNRD";
			dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses 2D/3D RangeScans alignment for estimating the robot movement";
			dec->type = "Node";
			dec->rawlog_format = "#2 - Observation-only";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #2");
			dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

			deciders_vec.push_back(dec);
		}
		{
			TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
			dec->name = "CICPGoodnessERD";
			dec->description = "Register a new edge by alligning the provided 2D/3D RangeScans of 2 nodes. Uses the goodness of the ICP Alignment as the criterium for adding a new edge";
			dec->type = "Edge";
			dec->rawlog_format = "Both";
			dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
			dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

			deciders_vec.push_back(dec);
		}


		// Input Validation
		if (!cmd.parse( argc, argv ) ||  showVersion || showHelp) {
			return 0;
		}
		// fetch the command line options
		// ////////////////////////////////////////////////////////////

		// list_*registrars switch args
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

			if (list_registrars) {
				VERBOSE_COUT << "Exiting.. " << endl;
				return 0;
			}
		}

		// fetch which registration deciders to use
		string node_reg = arg_node_reg.getValue();
		string edge_reg = arg_edge_reg.getValue();
		ASSERTMSG_(checkRegistrationDeciderExists(node_reg, "node"), 
				format("Node Registration Decider %s is not available. ", node_reg.c_str()) );
		checkRegistrationDeciderExists(edge_reg, "edge");
		ASSERTMSG_(checkRegistrationDeciderExists(edge_reg, "edge"), 
				format("Edge Registration Decider %s is not available. ", edge_reg.c_str()) );

		// fetch the filenames
		string ini_fname = arg_ini_file.getValue();
		string ground_truth_fname;
		string rawlog_fname = arg_rawlog_file.getValue();
		if ( arg_ground_truth_file.isSet() ) {
			ground_truth_fname = arg_ground_truth_file.getValue();
		}

		// Visualization
		CDisplayWindow3D	graph_win("Graphslam building procedure",800, 600);
		graph_win.setPos(800, 600);
		graph_win_observer.observeBegin(graph_win);
		{
			COpenGLScenePtr &scene = graph_win.get3DSceneAndLock();
			opengl::COpenGLViewportPtr main_view = scene->getViewport("main");
			graph_win_observer.observeBegin( *main_view );
			graph_win.unlockAccess3DScene();
		}
		VERBOSE_COUT << "Listening to graph_window events..." << endl;

		bool exit_normally;
		// take all the different combinations of node / edge registration deciders
		// one-by-one. 
		cout << "Node registration decider: " << node_reg << endl;
		cout << "Edge registration decider: " << edge_reg << endl;
		if (system::strCmpI(node_reg, "CFixedIntervalsNRD")) {
			if (system::strCmpI(edge_reg, "CICPGoodnessERD")) {
				// Initialize the CGraphSlamEngine_t class
				CGraphSlamEngine_t
					<
					CNetworkOfPoses2DInf,
					CFixedIntervalsNRD_t<CNetworkOfPoses2DInf>,
					CICPGoodnessERD_t<CNetworkOfPoses2DInf> 
					>
					graph_engine(
							ini_fname, 
							&graph_win,
							&graph_win_observer,
							rawlog_fname,
							ground_truth_fname);
				exit_normally = graph_engine.parseRawlogFile();
			}
		}
		else if (system::strCmpI(node_reg, "CICPGoodnessNRD")) {
			if (system::strCmpI(edge_reg, "CICPGoodnessERD")) {
				// Initialize the CGraphSlamEngine_t class
				CGraphSlamEngine_t
					<
					CNetworkOfPoses2DInf,
					CICPGoodnessNRD_t<CNetworkOfPoses2DInf>,
					CICPGoodnessERD_t<CNetworkOfPoses2DInf>
					>
					graph_engine(
							ini_fname, 
							&graph_win,
							&graph_win_observer,
							rawlog_fname,
							ground_truth_fname);
				exit_normally = graph_engine.parseRawlogFile();
			}
		}

		while (graph_win.isOpen() && exit_normally) {
			mrpt::system::sleep(100);
			graph_win.forceRepaint();
		}

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

// Print the properties of the available registars
void dumpRegistrarsToConsole(string reg_type="all") {
	MRPT_START;

	ASSERTMSG_((strCmpI(reg_type, "node") || 
			system::strCmpI(reg_type, "edge") || 
			system::strCmpI(reg_type, "all")),
			format("Registrar string '%s' does not match a known registrar name.\n"
				"Specify 'node' 'edge' or 'all'", reg_type.c_str()));

	string sep_header(40, '=');
	string sep_subheader(20, '-');
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
