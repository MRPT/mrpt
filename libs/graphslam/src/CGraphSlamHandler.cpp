/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Implementation file for CGraphSlamHandler class

#include "graphslam-precomp.h"
#include <mrpt/graphslam/apps_related/CGraphSlamHandler.h>

CGraphSlamHandler::CGraphSlamHandler() {
	using namespace mrpt::system;

	this->logger = NULL;
	win_manager = NULL;
	win_observer = NULL;
	win = NULL;
}

//////////////////////////////////////////////////////////////////////////////

void CGraphSlamHandler::setOutputLoggerPtr(mrpt::utils::COutputLogger* logger) {
	MRPT_START;

	ASSERT_(logger);
	this->logger = logger;

	MRPT_END;
}

//////////////////////////////////////////////////////////////////////////////
CGraphSlamHandler::~CGraphSlamHandler() {
	// Removed MRPT_START/END since a dtor can't launch an exception
	using namespace mrpt::utils;

	logger->logFmt(LVL_WARN, "graphslam-engine has finished.");

	// keep the window open until user closes it.
	if (win) {
		logger->logFmt(LVL_WARN, "Application will exit when the display window is closed.");
		bool break_exec = false;
		while (win->isOpen() && break_exec == false) {
			break_exec = !this->queryObserverForEvents();
			mrpt::system::sleep(100);
			win->forceRepaint();
		}
	}

	if (win) {
		logger->logFmt(LVL_DEBUG, "Releasing CDisplayWindow3D instance...");
		delete win;
	}
	if (win_observer) {
		logger->logFmt(LVL_DEBUG, "Releasing CWindowObserver instance...");
		delete win_observer;
	}
	if (win_manager) {
		logger->logFmt(LVL_DEBUG, "Releasing CWindowManager instance...");
		delete win_manager;
	}


	logger->logFmt(LVL_INFO, "Exited.");
}

//////////////////////////////////////////////////////////////////////////////

void CGraphSlamHandler::readConfigFname(const std::string& fname) {
	MRPT_START;
	using namespace mrpt::utils;

	ASSERTMSG_(mrpt::system::fileExists(fname),
			mrpt::format("\nConfiguration file not found: \n%s\n", fname.c_str()));

	if (logger) {
		logger->logFmt(LVL_INFO, "Reading the .ini file... ");
	}

	CConfigFile cfg_file(fname);

	output_dir_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"output_dir_fname",
			"graphslam_engine_results", false);
	user_decides_about_output_dir = cfg_file.read_bool(
			"GeneralConfiguration",
			"user_decides_about_output_dir",
			false, false);

	save_graph = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_graph",
			true, false);
	save_3DScene = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_3DScene",
			true, false);
	save_gridmap = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_gridmap",
			true, false);
	save_graph_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_graph_fname",
			"output_graph.graph", false);
	save_3DScene_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_3DScene_fname",
			"scene.3DScene", false);
	save_gridmap_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_gridmap_fname",
			"output_gridmap", false);

	MRPT_END;
}
//////////////////////////////////////////////////////////////////////////////

void CGraphSlamHandler::printParams() const {
	std::cout << this->getParamsAsString() << std::endl;

}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamHandler::getParamsAsString(std::string* str) const {
	using namespace std;

	ASSERT_(str);

	stringstream ss_out("");

	ss_out << "\n------------[ graphslam-engine_app Parameters ]------------"
		<< std::endl;

	// general configuration parameters
	ss_out << "User decides about output dir?  = "
		<< (user_decides_about_output_dir ? "TRUE" : "FALSE")
		<< std::endl;
	ss_out << "Output directory                = "
		<< output_dir_fname
		<< std::endl;
	ss_out << "Generate .graph file?           = "
		<< ( save_graph? "TRUE" : "FALSE" )
		<< std::endl;
	ss_out << "Generate .3DScene file?         = "
		<< ( save_3DScene? "TRUE" : "FALSE" )
		<< std::endl;
	if (save_graph) {
		ss_out << "Generated .graph filename       = "
			<< save_graph_fname
			<< std::endl;
	}
	if (save_3DScene) {
		ss_out << "Generated .3DScene filename     = "
			<< save_3DScene_fname << std::endl;
	}
	ss_out << "Rawlog filename                 = "
		<< rawlog_fname
		<< std::endl;

	*str = ss_out.str();
}
//////////////////////////////////////////////////////////////////////////////
std::string CGraphSlamHandler::getParamsAsString() const {
	std::string str;
	this->getParamsAsString(&str);
	return str;
}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamHandler::setRawlogFname(std::string rawlog_fname) {
	using namespace mrpt::utils;

	logger->logFmt(LVL_DEBUG, "Successfully fetched rawlog filename");
	this->rawlog_fname = rawlog_fname;
}


//////////////////////////////////////////////////////////////////////////////
void CGraphSlamHandler::initVisualization() {
	MRPT_START;

	using namespace mrpt::opengl;
	using namespace mrpt::gui;
	using namespace mrpt::utils;
	using namespace mrpt::graphslam;

	win_observer = new CWindowObserver();
	win = new CDisplayWindow3D(
			"GraphSlam building procedure", 800, 600);
	win->setPos(400, 200);
	win_observer->observeBegin(*win);
	{
		COpenGLScenePtr &scene = win->get3DSceneAndLock();
		COpenGLViewportPtr main_view = scene->getViewport("main");
		win_observer->observeBegin( *main_view );
		win->unlockAccess3DScene();
	}

	logger->logFmt(LVL_DEBUG, "Initialized CDisplayWindow3D...");
	logger->logFmt(LVL_DEBUG, "Listening to CDisplayWindow3D events...");

	// pass the window and the observer pointers to the CWindowManager instance
	win_manager = new mrpt::graphslam::CWindowManager();
	win_manager->setCDisplayWindow3DPtr(win);
	win_manager->setWindowObserverPtr(win_observer);

	MRPT_END;
}

//////////////////////////////////////////////////////////////////////////////
bool CGraphSlamHandler::queryObserverForEvents() {
	MRPT_START;

	std::map<std::string, bool> events_occurred;
	win_observer->returnEventsStruct(
			&events_occurred,
			/* reset_keypresses = */ false);
	bool request_to_exit = events_occurred.find("Ctrl+c")->second;

	return !request_to_exit;
	MRPT_END;
}
//////////////////////////////////////////////////////////////////////////////
