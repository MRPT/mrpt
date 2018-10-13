/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/graphslam/misc/CWindowManager.h>
#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>
#include <mrpt/graphslam/CGraphSlamEngine.h>

#include <string>
#include <sstream>
#include <iostream>

/**\brief Manage variables and methods related to applications executing
 * graphSLAM using the mrpt-graphslam API.
 *
 * CGraphSlamHandler class instances deal with the following:
 * - Manage user interaction with the visuals (e.g. CDisplayWindow instance)
 * - Manage general user options (e.g. user output directory preferences)
 */
template <class GRAPH_T = mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamHandler
{
   public:
	CGraphSlamHandler(
		mrpt::system::COutputLogger* logger,
		mrpt::graphslam::apps::TUserOptionsChecker<GRAPH_T>* options_checker,
		const bool enable_visuals = true);
	~CGraphSlamHandler();
	/**\brief Set the relevant filenames for instantiating CGraphSlamEngine
	 * instance
	 */
	void setFNames(
		const std::string& ini_fname, const std::string& rawlog_fname,
		const std::string& ground_truth_fname = std::string());
	/**\brief Print in a formatted manner the general configuraiton variables
	 * for
	 * the current graphSLAM execution
	 *
	 * \sa readConfigFname, getParamsAsString
	 */
	void printParams() const;
	/** Fetch the general configuraiton variables for the current graphSLAM
	 * execution
	 */
	/**\{*/
	void getParamsAsString(std::string* str) const;
	std::string getParamsAsString() const;
	/**\}*/
	/**\brief Initialize visualization (e.g. the CDisplayWindow instance that
	 * shows the overall graphSLAM execution)
	 */
	void initVisualization();
	/**\brief Method to be called for parsing the rawlog file provided and for
	 * running graphSLAM using that information
	 */
	void execute();
	void initEngine(
		const std::string& node_reg_str, const std::string& edge_reg_str,
		const std::string& optimizer_str);
	/**\brief Override the results directory filename that was initially set in
	 * the .ini file.
	 */
	void setResultsDirName(const std::string& dirname);

   protected:
	/**\brief Initialize (clean up and create new files) the output directory.
	 *
	 * If directory already exists (most probably from previous runs), the user
	 * is given 3 options:
	 * - Remove the current directory contents
	 * - Rename (and keep) the current directory contents
	 * - Manually handle the conflict
	 *
	 * User can also set the .ini parameter user_decides_about_output_dir  flag
	 * to false, if he doesn't care about the previous results directory. In
	 * this case the 1st choice is picked.
	 *
	 * \param[in] Name of the output directory to be used
	 *
	 * \sa CGraphSlamEngine::initResultsFile
	 */
	void initOutputDir(
		const std::string& output_dir_fname = "graphslam_results");
	/**\brief Query the CWindowObserver instance for any pressed keys that might
	 * be of interest (e.g. <C-c>)
	 *
	 * \return True if graphslam execution is to be continued normally
	 */
	bool queryObserverForEvents();
	/**\brief Read configuration variables for the current graphSLAM execution
	 * from a .ini file
	 *
	 * \sa printParams
	 */
	void readConfigFname(const std::string& fname);
	void saveResults(const std::string& output_dir_fname);
	void saveMap(const std::string& fname);

	std::string m_output_dir_fname;
	bool m_user_decides_about_output_dir;

	bool m_save_graph;
	bool m_save_3DScene;
	bool m_save_map;

	std::string m_ini_fname;
	std::string m_rawlog_fname;
	std::string m_gt_fname;

	std::string m_save_graph_fname;
	std::string m_save_3DScene_fname;
	std::string m_save_map_fname;

	mrpt::graphslam::CGraphSlamEngine<GRAPH_T>* m_engine = nullptr;
	mrpt::graphslam::CWindowManager* m_win_manager = nullptr;
	mrpt::graphslam::CWindowObserver* m_win_observer = nullptr;
	mrpt::gui::CDisplayWindow3D* m_win = nullptr;

	mrpt::system::COutputLogger* m_logger;

	/**\brief TUserOptionsChecker instance whose task is to evaluate the
	 * Registration Decider, Optimizer instances that are given by the user.
	 */
	mrpt::graphslam::apps::TUserOptionsChecker<GRAPH_T>* m_options_checker;

	bool m_do_save_results = true;
	bool m_has_set_fnames = false;
	;
	bool m_enable_visuals;
};

#include "CGraphSlamHandler_impl.h"
