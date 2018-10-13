/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include "CGraphSlamHandler.h"
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/obs/CRawlog.h>

template <class GRAPH_T>
CGraphSlamHandler<GRAPH_T>::CGraphSlamHandler(
	mrpt::system::COutputLogger* logger,
	mrpt::graphslam::apps::TUserOptionsChecker<GRAPH_T>* options_checker,
	const bool enable_visuals)
	: m_logger(logger),
	  m_options_checker(options_checker),
	  m_enable_visuals(enable_visuals)
{
	using namespace mrpt::system;
	ASSERTDEB_(m_logger);
	ASSERTDEB_(m_options_checker);

	if (m_enable_visuals)
	{
		this->initVisualization();
	}
}

template <class GRAPH_T>
CGraphSlamHandler<GRAPH_T>::~CGraphSlamHandler()
{
	m_logger->logFmt(mrpt::system::LVL_WARN, "graphslam-engine has finished.");

	// keep the window open until user closes it.
	if (m_win)
	{
		m_logger->logFmt(
			mrpt::system::LVL_WARN,
			"Application will exit when the display window is closed.");
		bool break_exec = false;
		while (m_win->isOpen() && break_exec == false)
		{
			break_exec = !this->queryObserverForEvents();
			std::this_thread::sleep_for(100ms);
			m_win->forceRepaint();
		}
	}

	if (m_do_save_results)
	{
		this->initOutputDir(m_output_dir_fname);
		if (m_engine)
		{
			this->saveResults(m_output_dir_fname);
		}
	}

	if (m_engine)
	{
		delete m_engine;
	}

	if (m_enable_visuals)
	{
		if (m_win)
		{
			m_logger->logFmt(
				mrpt::system::LVL_DEBUG,
				"Releasing CDisplayWindow3D instance...");
			delete m_win;
		}

		if (m_win_observer)
		{
			m_logger->logFmt(
				mrpt::system::LVL_DEBUG,
				"Releasing CWindowObserver instance...");
			delete m_win_observer;
		}

		if (m_win_manager)
		{
			m_logger->logFmt(
				mrpt::system::LVL_DEBUG,
				"Releasing CWindowManager instance...");
			delete m_win_manager;
		}
	}
	m_logger->logFmt(mrpt::system::LVL_INFO, "Exited.");
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::initOutputDir(
	const std::string& output_dir_fname)
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::system;
	using namespace mrpt;

	m_logger->logFmt(
		mrpt::system::LVL_INFO, "Setting up output directory: %s",
		output_dir_fname.c_str());

	// current time vars - handy in the rest of the function.
	mrpt::system::TTimeStamp cur_date(getCurrentTime());
	string cur_date_str(timeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));

	// Determine what to do with existing results if previous output directory
	// exists
	if (directoryExists(output_dir_fname))
	{
		int answer_int;
		if (m_user_decides_about_output_dir)
		{
			stringstream question;
			string answer;

			question << "Directory exists. Choose between the "
					 << "following options" << std::endl;
			question << "\t 1: Rename current folder and start new "
					 << "output directory (default)" << std::endl;
			question << "\t 2: Remove existing contents and continue execution "
					 << std::endl;
			question << "\t 3: Handle potential conflict manually "
						"(Halts program execution)"
					 << std::endl;
			question << "\t [ 1 | 2 | 3 ] --> ";
			std::cout << question.str();

			getline(cin, answer);
			answer = mrpt::system::trim(answer);
			answer_int = atoi(&answer[0]);
		}
		else
		{
			answer_int = 2;
		}

		switch (answer_int)
		{
			case 2:
			{
				m_logger->logFmt(
					mrpt::system::LVL_INFO, "Deleting existing files...");
				// purge directory
				deleteFilesInDirectory(
					output_dir_fname,
					/*deleteDirectoryAsWell = */ false);
				break;
			}
			case 3:
			{
				// I don't need to exit gracefully here..
				exit(0);
			}
			case 1:
			default:
			{
				// rename the whole directory to DATE_TIME_${OUTPUT_DIR_NAME}
				string dst_fname = output_dir_fname + cur_date_validstr;
				m_logger->logFmt(
					mrpt::system::LVL_INFO, "Renaming directory to: %s",
					dst_fname.c_str());
				string error_msg;
#if _DEBUG
				bool did_rename =
#endif
					renameFile(output_dir_fname, dst_fname, &error_msg);
				ASSERTDEBMSG_(
					did_rename, format(
									"\nError while trying to rename the output "
									"directory: %s",
									error_msg.c_str()));
				break;
			}
		}  // end switch (answer_int)
	}  // end if directory exists..

	// Now rebuild the directory from scratch
	m_logger->logFmt(
		mrpt::system::LVL_INFO, "Creating the new directory structure...");
	string cur_fname;

	// debug_fname
	createDirectory(output_dir_fname);
	m_logger->logFmt(
		mrpt::system::LVL_INFO, "Finished initializing output directory.");

	MRPT_END;
}  // end of initOutputDir

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::setFNames(
	const std::string& ini_fname, const std::string& rawlog_fname,
	const std::string& ground_truth_fname)
{
	this->m_ini_fname = ini_fname;
	this->m_rawlog_fname = rawlog_fname;
	this->m_gt_fname = ground_truth_fname;

	this->readConfigFname(m_ini_fname);

	m_has_set_fnames = true;
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::readConfigFname(const std::string& fname)
{
	ASSERTDEBMSG_(
		mrpt::system::fileExists(fname),
		mrpt::format("\nConfiguration file not found: \n%s\n", fname.c_str()));

	m_logger->logFmt(mrpt::system::LVL_INFO, "Reading the .ini file... ");

	mrpt::config::CConfigFile cfg_file(fname);

	m_user_decides_about_output_dir = cfg_file.read_bool(
		"GeneralConfiguration", "user_decides_about_output_dir", false, false);
	m_output_dir_fname = cfg_file.read_string(
		"GeneralConfiguration", "output_dir_fname", "graphslam_results", false);
	m_save_graph =
		cfg_file.read_bool("GeneralConfiguration", "save_graph", true, false);
	m_save_3DScene =
		cfg_file.read_bool("GeneralConfiguration", "save_3DScene", true, false);
	m_save_map =
		cfg_file.read_bool("GeneralConfiguration", "save_map", true, false);
	m_save_graph_fname = cfg_file.read_string(
		"GeneralConfiguration", "save_graph_fname", "output_graph.graph",
		false);
	m_save_3DScene_fname = cfg_file.read_string(
		"GeneralConfiguration", "save_3DScene_fname", "scene.3DScene", false);
	m_save_map_fname = cfg_file.read_string(
		"GeneralConfiguration", "save_map_fname", "output_map", false);
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::initEngine(
	const std::string& node_reg_str, const std::string& edge_reg_str,
	const std::string& optimizer_str)
{
	using namespace mrpt;

	ASSERTDEB_(m_has_set_fnames);

	ASSERTDEBMSG_(
		m_options_checker->checkRegistrationDeciderExists(node_reg_str, "node"),
		format(
			"\nNode Registration Decider %s is not available.\n",
			node_reg_str.c_str()));
	ASSERTDEBMSG_(
		m_options_checker->checkRegistrationDeciderExists(edge_reg_str, "edge"),
		format(
			"\nEdge Registration Decider %s is not available.\n",
			edge_reg_str.c_str()));
	ASSERTDEBMSG_(
		m_options_checker->checkOptimizerExists(optimizer_str),
		format("\nOptimizer %s is not available\n", optimizer_str.c_str()));

	m_engine = new mrpt::graphslam::CGraphSlamEngine<GRAPH_T>(
		m_ini_fname, m_rawlog_fname, m_gt_fname, m_win_manager,
		m_options_checker->node_regs_map[node_reg_str](),
		m_options_checker->edge_regs_map[edge_reg_str](),
		m_options_checker->optimizers_map[optimizer_str]());
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::printParams() const
{
	std::cout << this->getParamsAsString() << std::endl;
	m_engine->printParams();
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::getParamsAsString(std::string* str) const
{
	using namespace std;

	ASSERTDEB_(str);

	stringstream ss_out("");

	ss_out << "\n------------[ graphslam-engine_app Parameters ]------------"
		   << std::endl;

	// general configuration parameters
	ss_out << "User decides about output dir?  = "
		   << (m_user_decides_about_output_dir ? "TRUE" : "FALSE") << std::endl;
	ss_out << "Output directory                = " << m_output_dir_fname
		   << std::endl;
	ss_out << "Generate .graph file?           = "
		   << (m_save_graph ? "TRUE" : "FALSE") << std::endl;
	ss_out << "Generate .3DScene file?         = "
		   << (m_save_3DScene ? "TRUE" : "FALSE") << std::endl;
	if (m_save_graph)
	{
		ss_out << "Generated .graph filename       = " << m_save_graph_fname
			   << std::endl;
	}
	if (m_save_3DScene)
	{
		ss_out << "Generated .3DScene filename     = " << m_save_3DScene_fname
			   << std::endl;
	}
	ss_out << "Rawlog filename                 = " << m_rawlog_fname
		   << std::endl;

	*str = ss_out.str();
}
template <class GRAPH_T>
std::string CGraphSlamHandler<GRAPH_T>::getParamsAsString() const
{
	std::string str;
	this->getParamsAsString(&str);
	return str;
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::setResultsDirName(const std::string& dirname)
{
	m_output_dir_fname = mrpt::system::fileNameStripInvalidChars(dirname);
	m_logger->logFmt(
		mrpt::system::LVL_WARN, "Overriding .ini Results directory -> %s...",
		m_output_dir_fname.c_str());
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::saveResults(
	const std::string& output_dir_fname)
{
	ASSERTDEB_(m_engine);

	m_logger->logFmt(mrpt::system::LVL_INFO, "Generating overall report...");
	m_engine->generateReportFiles(output_dir_fname);
	// save the graph and the 3DScene
	if (m_save_graph)
	{
		std::string save_graph_fname =
			output_dir_fname + "/" + m_save_graph_fname;
		m_engine->saveGraph(&save_graph_fname);
	}
	if (m_enable_visuals && m_save_3DScene)
	{
		std::string save_3DScene_fname =
			output_dir_fname + "/" + m_save_3DScene_fname;
		m_engine->save3DScene(&save_3DScene_fname);
	}

	// get the occupancy map that was built
	if (m_save_map)
	{
		this->saveMap(output_dir_fname + "/" + m_save_map_fname);
	}

	m_logger->logFmt(mrpt::system::LVL_INFO, "Generated report.");
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::saveMap(const std::string& fname)
{
	mrpt::maps::COccupancyGridMap2D::Ptr map =
		mrpt::make_aligned_shared<mrpt::maps::COccupancyGridMap2D>();
	m_engine->getMap(map);
	// map->saveAsBitmapFile(fname); // doesn't work.
	map->saveMetricMapRepresentationToFile(fname);
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::execute()
{
	using namespace mrpt::obs;
	ASSERTDEB_(m_engine);

	// Variables initialization
	mrpt::io::CFileGZInputStream rawlog_stream(m_rawlog_fname);
	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;
	CObservation::Ptr observation;
	size_t curr_rawlog_entry;
	auto arch = mrpt::serialization::archiveFrom(rawlog_stream);

	// Read the dataset and pass the measurements to CGraphSlamEngine
	bool cont_exec = true;
	while (CRawlog::getActionObservationPairOrObservation(
			   arch, action, observations, observation, curr_rawlog_entry) &&
		   cont_exec)
	{
		// actual call to the graphSLAM execution method
		// Exit if user pressed C-c
		cont_exec = m_engine->_execGraphSlamStep(
			action, observations, observation, curr_rawlog_entry);
	}
	m_logger->logFmt(mrpt::system::LVL_WARN, "Finished graphslam execution.");
}

template <class GRAPH_T>
void CGraphSlamHandler<GRAPH_T>::initVisualization()
{
	using namespace mrpt::opengl;
	using namespace mrpt::gui;
	using namespace mrpt::graphslam;

	m_win_observer = new CWindowObserver();
	m_win = new CDisplayWindow3D("GraphSlam building procedure", 800, 600);
	m_win->setPos(400, 200);
	m_win_observer->observeBegin(*m_win);
	{
		COpenGLScene::Ptr& scene = m_win->get3DSceneAndLock();
		COpenGLViewport::Ptr main_view = scene->getViewport("main");
		m_win_observer->observeBegin(*main_view);
		m_win->unlockAccess3DScene();
	}

	m_logger->logFmt(
		mrpt::system::LVL_DEBUG, "Initialized CDisplayWindow3D...");
	m_logger->logFmt(
		mrpt::system::LVL_DEBUG, "Listening to CDisplayWindow3D events...");

	// pass the window and the observer pointers to the CWindowManager instance
	m_win_manager = new mrpt::graphslam::CWindowManager();
	m_win_manager->setCDisplayWindow3DPtr(m_win);
	m_win_manager->setWindowObserverPtr(m_win_observer);
}

template <class GRAPH_T>
bool CGraphSlamHandler<GRAPH_T>::queryObserverForEvents()
{
	std::map<std::string, bool> events_occurred;
	m_win_observer->returnEventsStruct(
		&events_occurred,
		/* reset_keypresses = */ false);
	bool request_to_exit = events_occurred.find("Ctrl+c")->second;

	return !request_to_exit;
}
