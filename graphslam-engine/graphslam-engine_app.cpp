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

#include <mrpt/otherlibs/tclap/CmdLine.h>

#include <string>
#include <cerrno>

#include "CGraphSlamEngine.h"
#include "CWindowObserver.h"

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;

using namespace std;

#define VERBOSE_COUT	if (verbose) std::cout << "[graphslam_engine] "

/*
 * Command line options initialization
 */

//
TCLAP::CmdLine cmd(/*output message = */ " graphslam-engine - Part of the MRPT\n",
		/* delimeter = */ ' ', /* version = */ MRPT_getVersion().c_str());

TCLAP::ValueArg<string> arg_ini_file(/*flag = */ "i", /*name = */ "ini_file",
		/*desc = */ ".ini configuration file", /* required = */ false,
		/* default value = */ "", /*typeDesc = */ "config.ini", /*parser = */ cmd);
TCLAP::ValueArg<string> arg_rawlog_file( "r", "rawlog",
		"Rawlog dataset file",	false, "", "captured_observations.rawlog", cmd);
TCLAP::SwitchArg arg_do_demo(/*flag = */ "d", /*name = */ "demo",
		/*desc = */ "Default file for demonstration purposes",
		/* parser = */ cmd, /* default = */ false);


CWindowObserver  graph_win_observer;

/**
 * main
 */
int main(int argc, char **argv)
{
	bool showHelp		 = argc>1 && !os::_strcmp(argv[1],"--help");
	bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

	try {

		/**
		 * Command line arguments parsing
		 */
		// validation
		if (!cmd.parse( argc, argv ) ||  showVersion || showHelp) {
			return 0;
		}
		else if (argc == 1 || (!arg_do_demo.isSet() && !arg_ini_file.isSet())) {
			THROW_EXCEPTION("Neither .ini file or demo option was specified." << endl
					<< "Use -h [--help] flag for list of available options" << endl
					<< "Exiting..");
		}
		else if (arg_do_demo.isSet()) {
			if (arg_rawlog_file.isSet() || arg_ini_file.isSet()) {
				THROW_EXCEPTION("-d [--demo] flag cannot be specified alongside other flags." << endl
						<< "Use -h [--help] flag for list of available options" << endl
						<< "Exiting.." << endl);
			}
		}
		// fetching the .ini file
		string config_fname;
		if ( arg_do_demo.isSet() ) {
			config_fname = "../default_config.ini";
			VERBOSE_COUT << "Using the demo .ini file: " << config_fname << endl;
		}
		else {
			config_fname = arg_ini_file.getValue();
			//VERBOSE_COUT << "Rawlog file: " << config_fname << endl;
		}

		/**
		 * Objects initialization
		 */

		// Visualization
		CDisplayWindow3D	graph_win("Graphslam building procedure",800, 600);
		graph_win.setPos(0, 0);
		graph_win_observer.observeBegin(graph_win);
		{
			COpenGLScenePtr &scene = graph_win.get3DSceneAndLock();
			opengl::COpenGLViewportPtr main_view = scene->getViewport("main");
			graph_win_observer.observeBegin( *main_view );
			graph_win.unlockAccess3DScene();
		}
		VERBOSE_COUT << "Listening to graph_window events..." << endl;

		// Global square root erro plot

		// Initialize the CGraphSlamEngine_t class
		string rawlog_fname;
		if (arg_rawlog_file.isSet()) {
			rawlog_fname = arg_rawlog_file.getValue();
		}
		CGraphSlamEngine_t<CNetworkOfPoses2DInf> g_engine(config_fname, 
				&graph_win,
				&graph_win_observer,
				rawlog_fname);

		bool exit_normally = g_engine.parseRawlogFile();

		// saving the graph to external file
		g_engine.saveGraph();

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
