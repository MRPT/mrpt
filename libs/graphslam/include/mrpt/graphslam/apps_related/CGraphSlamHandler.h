/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CGRAPHSLAMHANDLER_H
#define CGRAPHSLAMHANDLER_H

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/threads.h>

#include <mrpt/graphslam/link_pragmas.h>
#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/graphslam/misc/CWindowManager.h>

#include <string>
#include <sstream>
#include <iostream>

/**\brief Manage variables and methods related to applications executing
 * graphSLAM using the mrpt-graphslam API
 *
 * As a quick overview, class instances deal with the following:
 * - Manage user interaction with the visuals (e.g. CDisplayWindow instance)
 * - Manage general user options (e.g. user output directory preferences)
 */
class GRAPHSLAM_IMPEXP CGraphSlamHandler {
public:
	//
	// Ctor, Dtor
	//
	CGraphSlamHandler();
	~CGraphSlamHandler();

	//
	// Class methods
	//
	/**\brief Read configuration variables for the current graphSLAM execution
	 * from a .ini file
	 *
	 * \sa printParams
	 */
	void readConfigFname(const std::string& fname);
	/**\brief Print in a formatted manner the general configuraiton variables for
	 * the current graphSLAM execution
	 *
	 * \sa readConfigFname, getParamsAsString
	 */
	void printParams() const;
	/**\name Fetch the general configuraiton variables for
	 * the current graphSLAM execution
	 */
	/**\{*/
	void getParamsAsString(std::string* str) const;
	std::string getParamsAsString() const;
	/**\}*/
	/**\brief Set a logging instance */
	void setOutputLoggerPtr(mrpt::utils::COutputLogger* logger);
	/**\brief Initialize visualization (e.g. the CDisplayWindow instance that
	 * shows the overall graphSLAM execution)
	 */
	void initVisualization();
	/**\brief Set the rawlog filename */
	void setRawlogFname(std::string rawlog_fname);
	/**\brief Query the CWindowObserver instance for any pressed keys that might
	 * be of interest (e.g. <C-c>)
	 *
	 * \return True if graphslam execution is to be continued normally
	 */
	bool queryObserverForEvents();
	

	//
	// Class variable members 
	//
	std::string output_dir_fname;
	bool user_decides_about_output_dir;

	bool save_graph;
	bool save_3DScene;
	bool save_gridmap;

	std::string rawlog_fname;

	std::string save_graph_fname;
	std::string save_3DScene_fname;
	std::string save_gridmap_fname;

	mrpt::graphslam::CWindowManager* win_manager;
	mrpt::graphslam::CWindowObserver* win_observer;
	mrpt::gui::CDisplayWindow3D* win;

	mrpt::utils::COutputLogger* logger;
};


#endif /* end of include guard: CGRAPHSLAMHANDLER_H */
