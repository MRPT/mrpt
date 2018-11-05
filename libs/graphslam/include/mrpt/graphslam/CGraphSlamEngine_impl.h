/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

namespace mrpt::graphslam
{
template <class GRAPH_T>
const std::string CGraphSlamEngine<GRAPH_T>::header_sep = std::string(80, '-');
template <class GRAPH_T>
const std::string CGraphSlamEngine<GRAPH_T>::report_sep = std::string(2, '\n');

template <class GRAPH_T>
CGraphSlamEngine<GRAPH_T>::CGraphSlamEngine(
	const std::string& config_file, const std::string& rawlog_fname,
	const std::string& fname_GT, mrpt::graphslam::CWindowManager* win_manager,
	mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>* node_reg,
	mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>* edge_reg,
	mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>* optimizer)
	: m_node_reg(node_reg),
	  m_edge_reg(edge_reg),
	  m_optimizer(optimizer),
	  m_enable_visuals(win_manager != nullptr),
	  m_config_fname(config_file),
	  m_rawlog_fname(rawlog_fname),
	  m_fname_GT(fname_GT),
	  m_GT_poses_step(1),
	  m_win_manager(win_manager),
	  m_paused_message("Program is paused. Press \"p/P\" to resume."),
	  m_text_index_paused_message(345),  // just a large number.
	  m_odometry_color(0, 0, 255),
	  m_GT_color(0, 255, 0),
	  m_estimated_traj_color(255, 165, 0),
	  m_optimized_map_color(255, 0, 0),
	  m_current_constraint_type_color(255, 255, 255),
	  m_robot_model_size(1),
	  m_class_name("CGraphSlamEngine"),
	  m_is_first_time_node_reg(true)
{
	this->initClass();
}

template <class GRAPH_T>
CGraphSlamEngine<GRAPH_T>::~CGraphSlamEngine()
{
	// change back the CImage path
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum"))
	{
		MRPT_LOG_DEBUG_STREAM("Changing back the CImage PATH");
		mrpt::img::CImage::setImagesPathBase(m_img_prev_path_base);
	}
}

template <class GRAPH_T>
typename GRAPH_T::global_pose_t
	CGraphSlamEngine<GRAPH_T>::getCurrentRobotPosEstimation() const
{
	std::lock_guard<std::mutex> graph_lock(m_graph_section);
	return m_node_reg->getCurrentRobotPosEstimation();
}

template <class GRAPH_T>
typename GRAPH_T::global_poses_t
	CGraphSlamEngine<GRAPH_T>::getRobotEstimatedTrajectory() const
{
	std::lock_guard<std::mutex> graph_lock(m_graph_section);
	return m_graph.nodes;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::getNodeIDsOfEstimatedTrajectory(
	std::set<mrpt::graphs::TNodeID>* nodes_set) const
{
	ASSERTDEB_(nodes_set);
	m_graph.getAllNodes(*nodes_set);
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initClass()
{
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::opengl;
	using namespace std;

	// logger instance properties
	m_time_logger.setName(m_class_name);
	this->logging_enable_keep_record = true;
	this->setLoggerName(m_class_name);

	// Assert that the deciders/optimizer pointers are valid
	ASSERTDEB_(m_node_reg);
	ASSERTDEB_(m_edge_reg);
	ASSERTDEB_(m_optimizer);

	// Assert that the graph class used is supported.
	{
		MRPT_LOG_INFO_STREAM("Verifying support for given MRPT graph class...");

		// TODO - initialize vector in a smarter way.
		m_supported_constraint_types.push_back("CPosePDFGaussianInf");
		m_supported_constraint_types.push_back("CPose3DPDFGaussianInf");

		constraint_t c;
		const string c_str(c.GetRuntimeClass()->className);

		bool found =
			(std::find(
				 m_supported_constraint_types.begin(),
				 m_supported_constraint_types.end(),
				 c_str) != m_supported_constraint_types.end());

		if (found)
		{
			MRPT_LOG_INFO_STREAM("[OK] Class: " << c_str);
		}
		else
		{
			MRPT_LOG_WARN_STREAM(
				"Given graph class " << c_str
									 << " has not been tested consistently yet."
									 << "Proceed at your own risk.");
			mrpt::system::pause();
		}

		// store it in a string for future use.
		m_current_constraint_type = c_str;
		m_current_constraint_type = "Constraints: " + m_current_constraint_type;
	}

	// If a valid CWindowManager pointer is given then visuals are on.
	if (m_enable_visuals)
	{
		m_win = m_win_manager->win;
		m_win_observer = m_win_manager->observer;
	}
	else
	{
		m_win = nullptr;
		m_win_observer = nullptr;

		MRPT_LOG_WARN_STREAM("Visualization is off. Running on headless mode");
	}

	// set the CDisplayWindowPlots pointer to null for starters, we don't know
	// if
	// we are using it

	m_observation_only_dataset = false;
	m_request_to_exit = false;

	// max node number already in the graph
	m_nodeID_max = INVALID_NODEID;

	m_is_paused = false;
	m_GT_poses_index = 0;

	// pass the necessary variables/objects to the deciders/optimizes
	// pass a graph ptr after the instance initialization
	m_node_reg->setGraphPtr(&m_graph);
	m_edge_reg->setGraphPtr(&m_graph);
	m_optimizer->setGraphPtr(&m_graph);

	// pass the window manager pointer
	// note: m_win_manager contains a pointer to the CDisplayWindow3D instance
	if (m_enable_visuals)
	{
		m_node_reg->setWindowManagerPtr(m_win_manager);
		m_edge_reg->setWindowManagerPtr(m_win_manager);
		m_optimizer->setWindowManagerPtr(m_win_manager);
		m_edge_counter.setWindowManagerPtr(m_win_manager);
	}

	// pass a lock in case of multithreaded implementation
	m_node_reg->setCriticalSectionPtr(&m_graph_section);
	m_edge_reg->setCriticalSectionPtr(&m_graph_section);
	m_optimizer->setCriticalSectionPtr(&m_graph_section);

	// Load the parameters that each one of the self/deciders/optimizer classes
	// needs
	this->loadParams(m_config_fname);

	if (!m_enable_visuals)
	{
		MRPT_LOG_WARN_STREAM("Switching all visualization parameters off...");
		m_visualize_odometry_poses = 0;
		m_visualize_GT = 0;
		m_visualize_map = 0;
		m_visualize_estimated_trajectory = 0;
		m_visualize_SLAM_metric = 0;
		m_enable_curr_pos_viewport = 0;
		m_enable_range_viewport = 0;
		m_enable_intensity_viewport = 0;
	}

	m_use_GT = !m_fname_GT.empty();
	if (m_use_GT)
	{
		if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum"))
		{
			THROW_EXCEPTION("Not Implemented Yet.");
			this->alignOpticalWithMRPTFrame();
			// this->readGTFileRGBD_TUM(m_fname_GT, &m_GT_poses);
		}
		else if (mrpt::system::strCmpI(m_GT_file_format, "navsimul"))
		{
			this->readGTFile(m_fname_GT, &m_GT_poses);
		}
	}

	// plot the GT related visuals only if ground-truth file is given
	if (!m_use_GT)
	{
		MRPT_LOG_WARN_STREAM(
			"Ground truth file was not provided. Switching the related "
			"visualization parameters off...");
		m_visualize_GT = 0;
		m_visualize_SLAM_metric = 0;
	}

	// timestamp
	if (m_enable_visuals)
	{
		m_win_manager->assignTextMessageParameters(
			&m_offset_y_timestamp, &m_text_index_timestamp);
	}

	if (m_visualize_map)
	{
		this->initMapVisualization();
	}

	// Configuration of various trajectories visualization
	ASSERTDEB_(m_has_read_config);
	if (m_enable_visuals)
	{
		// odometry visualization
		if (m_visualize_odometry_poses)
		{
			this->initOdometryVisualization();
		}
		// GT Visualization
		if (m_visualize_GT)
		{
			this->initGTVisualization();
		}
		// estimated trajectory visualization
		this->initEstimatedTrajectoryVisualization();
		// current robot pose  viewport
		if (m_enable_curr_pos_viewport)
		{
			this->initCurrPosViewport();
		}
	}

	// change the CImage path in case of RGBD datasets
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum"))
	{
		// keep the last path - change back to it after rawlog parsing
		m_img_prev_path_base = mrpt::img::CImage::getImagesPathBase();

		std::string rawlog_fname_noext =
			system::extractFileName(m_rawlog_fname);
		std::string rawlog_dir = system::extractFileDirectory(m_rawlog_fname);
		m_img_external_storage_dir =
			rawlog_dir + rawlog_fname_noext + "_Images/";
		mrpt::img::CImage::setImagesPathBase(m_img_external_storage_dir);
	}

	// 3DRangeScans viewports initialization, in case of RGBD datasets
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum"))
	{
		if (m_enable_range_viewport)
		{
			this->initRangeImageViewport();
		}
		if (m_enable_intensity_viewport)
		{
			this->initIntensityImageViewport();
		}
	}
	// axis
	if (m_enable_visuals)
	{
		COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

		CAxis::Ptr obj = mrpt::make_aligned_shared<CAxis>();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
		obj->setName("axis");
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// Add additional keystrokes in the CDisplayWindow3D help textBox
	if (m_enable_visuals)
	{
		// keystrokes initialization
		m_keystroke_pause_exec = "p";
		m_keystroke_odometry = "o";
		m_keystroke_GT = "g";
		m_keystroke_estimated_trajectory = "t";
		m_keystroke_map = "m";

		// keystrokes registration
		m_win_observer->registerKeystroke(
			m_keystroke_pause_exec, "Pause/Resume program execution");
		m_win_observer->registerKeystroke(
			m_keystroke_odometry, "Toggle Odometry visualization");
		m_win_observer->registerKeystroke(
			m_keystroke_GT, "Toggle Ground-Truth visualization");
		m_win_observer->registerKeystroke(
			m_keystroke_estimated_trajectory,
			"Toggle Estimated trajectory visualization");
		m_win_observer->registerKeystroke(
			m_keystroke_map, "Toggle Map visualization");
	}

	// register the types of edges
	vector<string> vec_edge_types;
	vec_edge_types.emplace_back("Odometry");
	vec_edge_types.emplace_back("ICP2D");
	vec_edge_types.emplace_back("ICP3D");

	for (auto cit = vec_edge_types.begin(); cit != vec_edge_types.end(); ++cit)
	{
		m_edge_counter.addEdgeType(*cit);
	}

	// Visualize the edge statistics
	if (m_enable_visuals)
	{
		// total edges - loop closure edges
		double offset_y_total_edges, offset_y_loop_closures;
		int text_index_total_edges, text_index_loop_closures;

		m_win_manager->assignTextMessageParameters(
			&offset_y_total_edges, &text_index_total_edges);

		// build each one of these
		map<string, double> name_to_offset_y;
		map<string, int> name_to_text_index;
		for (auto it = vec_edge_types.begin(); it != vec_edge_types.end(); ++it)
		{
			m_win_manager->assignTextMessageParameters(
				&name_to_offset_y[*it], &name_to_text_index[*it]);
		}

		m_win_manager->assignTextMessageParameters(
			&offset_y_loop_closures, &text_index_loop_closures);

		// add all the parameters to the CEdgeCounter object
		m_edge_counter.setTextMessageParams(
			name_to_offset_y, name_to_text_index, offset_y_total_edges,
			text_index_total_edges, offset_y_loop_closures,
			text_index_loop_closures);
	}

	// Type of the generated graph
	if (m_enable_visuals)
	{
		m_win_manager->assignTextMessageParameters(
			&m_offset_y_current_constraint_type,
			&m_text_index_current_constraint_type);
		m_win_manager->addTextMessage(
			m_offset_x_left, -m_offset_y_current_constraint_type,
			m_current_constraint_type,
			mrpt::img::TColorf(m_current_constraint_type_color),
			m_text_index_current_constraint_type);
	}

	// query node/edge deciders for visual objects initialization
	if (m_enable_visuals)
	{
		std::lock_guard<std::mutex> graph_lock(m_graph_section);
		m_time_logger.enter("Visuals");
		m_node_reg->initializeVisuals();
		m_edge_reg->initializeVisuals();
		m_optimizer->initializeVisuals();
		m_time_logger.leave("Visuals");
	}

	m_init_timestamp = INVALID_TIMESTAMP;

	// COccupancyGridMap2D Initialization
	{
		mrpt::maps::COccupancyGridMap2D::Ptr gridmap =
			mrpt::make_aligned_shared<mrpt::maps::COccupancyGridMap2D>();

		gridmap->setSize(
			/* min_x = */ -20.0f,
			/* float max_x = */ 20.0f,
			/* float min_y = */ -20.0f,
			/* float max_y = */ 20.0f,
			/* float resolution = */ 0.05f);

		// TODO - Read these from the .ini file
		// observation insertion options
		gridmap->insertionOptions.maxOccupancyUpdateCertainty = 0.8f;
		gridmap->insertionOptions.maxDistanceInsertion = 5;
		gridmap->insertionOptions.wideningBeamsWithDistance = true;
		gridmap->insertionOptions.decimation = 2;

		m_gridmap_cached = gridmap;
		m_map_is_cached = false;
	}

	// COctoMap Initialization
	{
		mrpt::maps::COctoMap::Ptr octomap =
			mrpt::make_aligned_shared<mrpt::maps::COctoMap>();

		// TODO - adjust the insertionoptions...
		// TODO - Read these from the .ini file
		octomap->insertionOptions.setOccupancyThres(0.5);
		octomap->insertionOptions.setProbHit(0.7);
		octomap->insertionOptions.setProbMiss(0.4);
		octomap->insertionOptions.setClampingThresMin(0.1192);
		octomap->insertionOptions.setClampingThresMax(0.971);

		m_octomap_cached = octomap;
		m_map_is_cached = false;
	}

	// In case we are given an RGBD TUM Dataset - try and read the info file so
	// that we know how to play back the GT poses.
	try
	{
		m_info_params.setRawlogFile(m_rawlog_fname);
		m_info_params.parseFile();
		// set the rate at which we read from the GT poses vector
		int num_of_objects = std::atoi(
			m_info_params.fields["Overall number of objects"].c_str());
		m_GT_poses_step = m_GT_poses.size() / num_of_objects;

		MRPT_LOG_INFO_STREAM(
			"Overall number of objects in rawlog: " << num_of_objects);
		MRPT_LOG_INFO_STREAM(
			"Setting the Ground truth read step to: " << m_GT_poses_step);
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_INFO_STREAM("RGBD_TUM info file was not found: " << e.what());
	}

	// SLAM evaluation metric
	m_curr_deformation_energy = 0;
	if (m_visualize_SLAM_metric)
	{
		this->initSlamMetricVisualization();
	}

	// Message to be displayed on pause
	if (m_enable_visuals)
	{
		this->m_win->addTextMessage(
			0.5, 0.3, "", mrpt::img::TColorf(1.0, 0, 0),
			m_text_index_paused_message);
	}

	MRPT_END;
}  // end of initClass

template <class GRAPH_T>
bool CGraphSlamEngine<GRAPH_T>::execGraphSlamStep(
	mrpt::obs::CObservation::Ptr& observation, size_t& rawlog_entry)
{
	using namespace mrpt::obs;

	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;

	return this->_execGraphSlamStep(
		action, observations, observation, rawlog_entry);
}  // end of execGraphSlamStep

template <class GRAPH_T>
bool CGraphSlamEngine<GRAPH_T>::_execGraphSlamStep(
	mrpt::obs::CActionCollection::Ptr& action,
	mrpt::obs::CSensoryFrame::Ptr& observations,
	mrpt::obs::CObservation::Ptr& observation, size_t& rawlog_entry)
{
	MRPT_START;

	using namespace std;
	using namespace mrpt;
	using namespace mrpt::poses;
	using namespace mrpt::obs;
	using namespace mrpt::obs::utils;
	using namespace mrpt::opengl;
	using namespace mrpt::system;

	m_time_logger.enter("proc_time");

	ASSERTDEBMSG_(
		m_has_read_config,
		format("\nConfig file is not read yet.\nExiting... \n"));
	// good to go..

	// read first measurement independently if we haven't already
	if (m_init_timestamp == INVALID_TIMESTAMP)
	{
		m_init_timestamp = getTimeStamp(action, observations, observation);
		MRPT_LOG_DEBUG_STREAM("execGraphSlamStep: first run");

		if (observation)
		{
			MRPT_LOG_DEBUG_STREAM("Observation only dataset!");
			m_observation_only_dataset = true;  // false by default
		}
		else
		{
			MRPT_LOG_DEBUG_STREAM("Action-observation dataset!");
			ASSERTDEB_(action);
			m_observation_only_dataset = false;

			CPose3D increment_pose_3d;
			action->getFirstMovementEstimationMean(increment_pose_3d);
			pose_t increment_pose(increment_pose_3d);
			m_curr_odometry_only_pose += increment_pose;
		}

		// TODO enable this and test this.
		// return true;
	}

	// NRD
	bool registered_new_node;
	{
		std::lock_guard<std::mutex> graph_lock(m_graph_section);
		m_time_logger.enter("node_registrar");
		registered_new_node =
			m_node_reg->updateState(action, observations, observation);
		m_time_logger.leave("node_registrar");
	}

	{  // get the 2D laser scan, if there
		CObservation2DRangeScan::Ptr scan =
			getObservation<CObservation2DRangeScan>(observations, observation);
		if (scan)
		{
			m_last_laser_scan2D = scan;

			if (!m_first_laser_scan2D)
			{  // capture first laser scan seperately
				m_first_laser_scan2D = m_last_laser_scan2D;
			}
		}
	}

	if (registered_new_node)
	{
		// At the first node registration, must have registered exactly 2 nodes
		// (root + first)
		if (m_is_first_time_node_reg)
		{
			std::lock_guard<std::mutex> graph_lock(m_graph_section);
			m_nodeID_max = 0;
			if (m_graph.nodeCount() != 2)
			{
				MRPT_LOG_ERROR_STREAM(
					"Expected [2] new registered nodes"
					<< " but got [" << m_graph.nodeCount() << "]");
				THROW_EXCEPTION(format("Illegal node registration"));
			}

			m_nodes_to_laser_scans2D.insert(
				make_pair(m_nodeID_max, m_first_laser_scan2D));

			m_is_first_time_node_reg = false;
		}

		// going to be incremented in monitorNodeRegistration anyway.
		m_nodeID_max++;

		// either way add just one odometry edge
		m_edge_counter.addEdge("Odometry");
	}
	this->monitorNodeRegistration(
		registered_new_node, "NodeRegistrationDecider");

	// Edge registration procedure - Optimization
	// run this so that the ERD, GSO can be updated with the latest
	// observations even when no new nodes have been added to the graph
	{  // ERD
		std::lock_guard<std::mutex> graph_lock(m_graph_section);

		m_time_logger.enter("edge_registrar");
		m_edge_reg->updateState(action, observations, observation);
		m_time_logger.leave("edge_registrar");
	}
	this->monitorNodeRegistration(
		registered_new_node, "EdgeRegistrationDecider");

	{  // GSO
		std::lock_guard<std::mutex> graph_lock(m_graph_section);

		m_time_logger.enter("optimizer");
		m_optimizer->updateState(action, observations, observation);
		m_time_logger.leave("optimizer");
	}
	this->monitorNodeRegistration(registered_new_node, "GraphSlamOptimizer");

	// current timestamp - to be filled depending on the format
	m_curr_timestamp = getTimeStamp(action, observations, observation);

	if (observation)
	{
		// Read a single observation from the rawlog
		// (Format #2 rawlog file)

		// odometry
		if (IS_CLASS(observation, CObservationOdometry))
		{
			CObservationOdometry::Ptr obs_odometry =
				std::dynamic_pointer_cast<CObservationOdometry>(observation);

			m_curr_odometry_only_pose = pose_t(obs_odometry->odometry);
			m_odometry_poses.push_back(m_curr_odometry_only_pose);
		}
		else if (IS_CLASS(observation, CObservation3DRangeScan))
		{
			m_last_laser_scan3D =
				std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(
					observation);
		}
	}
	else
	{
		// action, observations should contain a pair of valid data
		// (Format #1 rawlog file)
		ASSERTDEB_(observations);
		ASSERTDEB_(action);

		CPose3D increment_pose_3d;
		action->getFirstMovementEstimationMean(increment_pose_3d);
		pose_t increment_pose(increment_pose_3d);
		m_curr_odometry_only_pose += increment_pose;
		m_odometry_poses.push_back(m_curr_odometry_only_pose);
	}  // ELSE FORMAT #1 - Action/Observations

	if (registered_new_node)
	{
		this->execDijkstraNodesEstimation();

		// keep track of the laser scans so that I can later visualize the map
		m_nodes_to_laser_scans2D[m_nodeID_max] = m_last_laser_scan2D;

		if (m_enable_visuals && m_visualize_map)
		{
			std::lock_guard<std::mutex> graph_lock(m_graph_section);
			// bool full_update = m_optimizer->justFullyOptimizedGraph();
			bool full_update = true;
			this->updateMapVisualization(m_nodes_to_laser_scans2D, full_update);
		}

		// query node/edge deciders for visual objects update
		if (m_enable_visuals)
		{
			this->updateAllVisuals();
		}

		// update the edge counter
		std::map<std::string, int> edge_types_to_nums;
		m_edge_reg->getEdgesStats(&edge_types_to_nums);
		if (edge_types_to_nums.size())
		{
			for (auto it = edge_types_to_nums.begin();
				 it != edge_types_to_nums.end(); ++it)
			{
				// loop closure
				if (mrpt::system::strCmpI(it->first, "lc"))
				{
					m_edge_counter.setLoopClosureEdgesManually(it->second);
				}
				else
				{
					m_edge_counter.setEdgesManually(it->first, it->second);
				}
			}
		}

		// update the graph visualization

		if (m_enable_curr_pos_viewport)
		{
			updateCurrPosViewport();
		}
		// update visualization of estimated trajectory
		if (m_enable_visuals)
		{
			bool full_update = true;  // don't care, do it anyway.
			m_time_logger.enter("Visuals");
			this->updateEstimatedTrajectoryVisualization(full_update);
			m_time_logger.leave("Visuals");
		}

		// refine the SLAM metric  and update its corresponding visualization
		// This is done only when the GT is available.
		if (m_use_GT)
		{
			m_time_logger.enter("SLAM_metric");
			this->computeSlamMetric(m_nodeID_max, m_GT_poses_index);
			m_time_logger.leave("SLAM_metric");
			if (m_visualize_SLAM_metric)
			{
				m_time_logger.enter("Visuals");
				this->updateSlamMetricVisualization();
				m_time_logger.leave("Visuals");
			}
		}

		// mark the map outdated
		m_map_is_cached = false;

	}  // IF REGISTERED_NEW_NODE

	//
	// Visualization related actions
	//
	m_time_logger.enter("Visuals");
	// Timestamp textMessage
	// Use the dataset timestamp otherwise fallback to
	// mrpt::system::getCurrentTime
	if (m_enable_visuals)
	{
		if (m_curr_timestamp != INVALID_TIMESTAMP)
		{
			m_win_manager->addTextMessage(
				m_offset_x_left, -m_offset_y_timestamp,
				format(
					"Simulated time: %s",
					timeToString(m_curr_timestamp).c_str()),
				mrpt::img::TColorf(1.0, 1.0, 1.0),
				/* unique_index = */ m_text_index_timestamp);
		}
		else
		{
			m_win_manager->addTextMessage(
				m_offset_x_left, -m_offset_y_timestamp,
				format(
					"Wall time: %s",
					timeToString(mrpt::system::getCurrentTime()).c_str()),
				mrpt::img::TColorf(1.0, 1.0, 1.0),
				/* unique_index = */ m_text_index_timestamp);
		}
	}

	// Odometry visualization
	if (m_visualize_odometry_poses && m_odometry_poses.size())
	{
		this->updateOdometryVisualization();
	}

	// ensure that the GT is visualized at the same rate as the SLAM procedure
	// handle RGBD-TUM datasets manually. Advance the GT index accordingly
	if (m_use_GT)
	{
		if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum"))
		{  // 1/loop
			if (m_enable_visuals)
			{
				this->updateGTVisualization();  // I have already taken care of
				// the step
			}
			m_GT_poses_index += m_GT_poses_step;
		}
		else if (mrpt::system::strCmpI(m_GT_file_format, "navsimul"))
		{
			if (m_observation_only_dataset)
			{  // 1/2loops
				if (rawlog_entry % 2 == 0)
				{
					if (m_enable_visuals)
					{
						this->updateGTVisualization();  // I have already taken
						// care of the step
					}
					m_GT_poses_index += m_GT_poses_step;
				}
			}
			else
			{  // 1/loop
				// get both action and observation at a single step - same rate
				// as GT
				if (m_enable_visuals)
				{
					this->updateGTVisualization();  // I have already taken care
					// of the step
				}
				m_GT_poses_index += m_GT_poses_step;
			}
		}
	}

	// 3DRangeScans viewports update
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum"))
	{
		if (m_enable_range_viewport && m_last_laser_scan3D)
		{
			this->updateRangeImageViewport();
		}

		if (m_enable_intensity_viewport && m_last_laser_scan3D)
		{
			this->updateIntensityImageViewport();
		}
	}

	// Query for events and take corresponding actions
	if (m_enable_visuals)
	{
		this->queryObserverForEvents();
	}
	m_time_logger.leave("Visuals");

	m_dataset_grab_time =
		mrpt::system::timeDifference(m_init_timestamp, m_curr_timestamp);
	m_time_logger.leave("proc_time");

	return !m_request_to_exit;
	MRPT_END;
}  // end of _execGraphSlamStep

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::execDijkstraNodesEstimation()
{
	{
		std::lock_guard<std::mutex> graph_lock(m_graph_section);
		m_time_logger.enter("dijkstra_nodes_estimation");
		m_graph.dijkstra_nodes_estimate();
		m_time_logger.leave("dijkstra_nodes_estimation");
	}
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::monitorNodeRegistration(
	bool registered /*=false*/, std::string class_name /*="Class"*/)
{
	MRPT_START;
	using namespace mrpt;

	std::lock_guard<std::mutex> graph_lock(m_graph_section);
	size_t listed_nodeCount =
		(m_nodeID_max == INVALID_NODEID ? 0 : m_nodeID_max + 1);

	if (!registered)
	{  // just check that it's the same.
		ASSERTDEBMSG_(
			listed_nodeCount == m_graph.nodeCount(),
			format(
				"listed_nodeCount [%lu] != nodeCount() [%lu]",
				static_cast<unsigned long>(listed_nodeCount),
				static_cast<unsigned long>(m_graph.nodeCount())));
	}
	else
	{
		if (listed_nodeCount != m_graph.nodeCount())
		{
			MRPT_LOG_ERROR_STREAM(
				class_name << " illegally added new nodes to the graph "
						   << ", wanted to see [" << listed_nodeCount
						   << "] but saw [" << m_graph.nodeCount() << "]");
			THROW_EXCEPTION(
				format("Illegal node registration by %s.", class_name.c_str()));
		}
	}
	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::getMap(
	mrpt::maps::COccupancyGridMap2D::Ptr map,
	mrpt::system::TTimeStamp* acquisition_time) const
{
	MRPT_START;

	if (!map)
	{
		map = mrpt::make_aligned_shared<mrpt::maps::COccupancyGridMap2D>();
	}
	ASSERTDEB_(map);

	if (!m_map_is_cached)
	{
		this->computeMap();
	}
	map->copyMapContentFrom(*m_gridmap_cached);

	// fill the timestamp if this is given
	if (acquisition_time)
	{
		*acquisition_time = m_map_acq_time;
	}
	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::getMap(
	mrpt::maps::COctoMap::Ptr map,
	mrpt::system::TTimeStamp* acquisition_time) const
{
	MRPT_START;
	THROW_EXCEPTION("Not Implemented Yet.");

	if (!m_map_is_cached)
	{
		this->computeMap();
	}
	//	map =
	// dynamic_cast<mrpt::maps::COctoMap::Ptr>(m_octomap_cached->clone());
	ASSERTDEB_(map);

	// fill the timestamp if this is given
	if (acquisition_time)
	{
		*acquisition_time = m_map_acq_time;
	}

	MRPT_END;
}

template <class GRAPH_T>
inline void CGraphSlamEngine<GRAPH_T>::computeMap() const
{
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::maps;
	using namespace mrpt::poses;

	std::lock_guard<std::mutex> graph_lock(m_graph_section);

	if (!constraint_t::is_3D())
	{  // 2D Poses
		// MRPT_LOG_DEBUG_STREAM("Computing the occupancy gridmap...");
		mrpt::maps::COccupancyGridMap2D::Ptr gridmap = m_gridmap_cached;
		gridmap->clear();

		// traverse all the nodes - add their laser scans at their corresponding
		// poses
		for (auto it = m_nodes_to_laser_scans2D.begin();
			 it != m_nodes_to_laser_scans2D.end(); ++it)
		{
			const mrpt::graphs::TNodeID& curr_node = it->first;

			// fetch LaserScan
			const mrpt::obs::CObservation2DRangeScan::Ptr& curr_laser_scan =
				it->second;
			ASSERTDEBMSG_(
				curr_laser_scan, format(
									 "LaserScan of nodeID: %lu is not present.",
									 static_cast<unsigned long>(curr_node)));

			// Fetch pose at which to display the LaserScan
			CPose3D scan_pose = getLSPoseForGridMapVisualization(curr_node);

			// Add all to gridmap
			gridmap->insertObservation(curr_laser_scan.get(), &scan_pose);
		}

		m_map_is_cached = true;
		m_map_acq_time = mrpt::system::now();
	}
	else
	{  // 3D Pose
		// MRPT_LOG_DEBUG_STREAM("Computing the Octomap...");
		THROW_EXCEPTION("Not Implemented Yet. Method is to compute a COctoMap");
		// MRPT_LOG_DEBUG_STREAM("Computed COctoMap successfully.");
	}

	MRPT_END;
}  // end of computeMap

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::loadParams(const std::string& fname)
{
	MRPT_START;

	ASSERTDEBMSG_(
		mrpt::system::fileExists(fname),
		mrpt::format("\nConfiguration file not found: \n%s\n", fname.c_str()));

	mrpt::config::CConfigFile cfg_file(fname);

	// Section: GeneralConfiguration

	m_user_decides_about_output_dir = cfg_file.read_bool(
		"GeneralConfiguration", "user_decides_about_output_dir", false, false);
	m_GT_file_format = cfg_file.read_string(
		"GeneralConfiguration", "ground_truth_file_format", "NavSimul", false);

	// Minimum verbosity level of the logger
	int min_verbosity_level =
		cfg_file.read_int("GeneralConfiguration", "class_verbosity", 1, false);
	this->setMinLoggingLevel(mrpt::system::VerbosityLevel(min_verbosity_level));

	// Section: VisualizationParameters

	// map visualization
	m_visualize_map = cfg_file.read_bool(
		"VisualizationParameters", "visualize_map", true, false);

	// odometry-only visualization
	m_visualize_odometry_poses = cfg_file.read_bool(
		"VisualizationParameters", "visualize_odometry_poses", true, false);
	m_visualize_estimated_trajectory = cfg_file.read_bool(
		"VisualizationParameters", "visualize_estimated_trajectory", true,
		false);

	// GT configuration visualization
	m_visualize_GT = cfg_file.read_bool(
		"VisualizationParameters", "visualize_ground_truth", true, false);
	// SLAM metric plot
	m_visualize_SLAM_metric = cfg_file.read_bool(
		"VisualizationParameters", "visualize_SLAM_metric", true, false);

	// Viewports flags
	m_enable_curr_pos_viewport = cfg_file.read_bool(
		"VisualizationParameters", "enable_curr_pos_viewport", true, false);
	m_enable_range_viewport = cfg_file.read_bool(
		"VisualizationParameters", "enable_range_viewport", false, false);
	m_enable_intensity_viewport = cfg_file.read_bool(
		"VisualizationParameters", "enable_intensity_viewport", false, false);

	m_node_reg->loadParams(fname);
	m_edge_reg->loadParams(fname);
	m_optimizer->loadParams(fname);

	m_has_read_config = true;
	MRPT_END;
}
template <class GRAPH_T>
std::string CGraphSlamEngine<GRAPH_T>::getParamsAsString() const
{
	MRPT_START;

	std::string str;
	this->getParamsAsString(&str);
	return str;

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::getParamsAsString(std::string* params_out) const
{
	MRPT_START;
	ASSERTDEB_(m_has_read_config);
	using namespace std;

	stringstream ss_out;

	ss_out
		<< "\n------------[ Graphslam_engine Problem Parameters ]------------"
		<< std::endl;
	ss_out << "Config filename                 = " << m_config_fname
		   << std::endl;

	ss_out << "Ground Truth File format        = " << m_GT_file_format
		   << std::endl;
	ss_out << "Ground Truth filename           = " << m_fname_GT << std::endl;

	ss_out << "Visualize odometry              = "
		   << (m_visualize_odometry_poses ? "TRUE" : "FALSE") << std::endl;
	ss_out << "Visualize estimated trajectory  = "
		   << (m_visualize_estimated_trajectory ? "TRUE" : "FALSE")
		   << std::endl;
	ss_out << "Visualize map                   = "
		   << (m_visualize_map ? "TRUE" : "FALSE") << std::endl;
	ss_out << "Visualize Ground Truth          = "
		   << (m_visualize_GT ? "TRUE" : "FALSE") << std::endl;

	ss_out << "Visualize SLAM metric plot      = "
		   << (m_visualize_SLAM_metric ? "TRUE" : "FALSE") << std::endl;

	ss_out << "Enable curr. position viewport  = "
		   << (m_enable_curr_pos_viewport ? "TRUE" : "FALSE") << endl;
	ss_out << "Enable range img viewport       = "
		   << (m_enable_range_viewport ? "TRUE" : "FALSE") << endl;
	ss_out << "Enable intensity img viewport   = "
		   << (m_enable_intensity_viewport ? "TRUE" : "FALSE") << endl;

	ss_out << "-----------------------------------------------------------"
		   << std::endl;
	ss_out << std::endl;

	// copy the stringstream contents to the passed in string
	*params_out = ss_out.str();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::printParams() const
{
	MRPT_START;
	std::cout << getParamsAsString();

	m_node_reg->printParams();
	m_edge_reg->printParams();
	m_optimizer->printParams();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initResultsFile(const std::string& fname)
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::system;

	MRPT_LOG_INFO_STREAM("Setting up file: " << fname);

	// current time vars
	mrpt::system::TTimeStamp cur_date(getCurrentTime());
	std::string time_spec = "UTC Time";
	string cur_date_str(dateTimeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));

	m_out_streams[fname].open(fname);
	ASSERTDEBMSG_(
		m_out_streams[fname].fileOpenCorrectly(),
		mrpt::format("\nError while trying to open %s\n", fname.c_str()));

	const std::string sep(80, '#');

	m_out_streams[fname].printf("# Mobile Robot Programming Toolkit (MRPT)\n");
	m_out_streams[fname].printf("# http::/www.mrpt.org\n");
	m_out_streams[fname].printf("# GraphSlamEngine Application\n");
	m_out_streams[fname].printf(
		"# Automatically generated file - %s: %s\n", time_spec.c_str(),
		cur_date_str.c_str());
	m_out_streams[fname].printf("%s\n\n", sep.c_str());

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initRangeImageViewport()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	COpenGLViewport::Ptr viewp_range;

	viewp_range = scene->createViewport("viewp_range");
	double x, y, h, w;
	m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp_range->setViewportPosition(x, y, h, w);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateAllVisuals()
{
	MRPT_START;
	std::lock_guard<std::mutex> graph_lock(m_graph_section);
	m_time_logger.enter("Visuals");

	m_node_reg->updateVisuals();
	m_edge_reg->updateVisuals();
	m_optimizer->updateVisuals();

	m_time_logger.leave("Visuals");
	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateRangeImageViewport()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::math;
	using namespace mrpt::opengl;

	if (m_last_laser_scan3D->hasRangeImage)
	{
		// TODO - make this a static class member - or at least a private member
		// of the class
		CMatrixFloat range2D;
		mrpt::img::CImage img;

		// load the image if not already loaded..
		m_last_laser_scan3D->load();
		range2D = m_last_laser_scan3D->rangeImage *
				  (1.0f / 5.0);  // TODO - without the magic number?
		img.setFromMatrix(range2D);

		COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
		COpenGLViewport::Ptr viewp_range = scene->getViewport("viewp_range");
		viewp_range->setImageView(img);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initIntensityImageViewport()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	COpenGLViewport::Ptr viewp_intensity;

	viewp_intensity = scene->createViewport("viewp_intensity");
	double x, y, w, h;
	m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp_intensity->setViewportPosition(x, y, w, h);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateIntensityImageViewport()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	if (m_last_laser_scan3D->hasIntensityImage)
	{
		mrpt::img::CImage img;

		// load the image if not already loaded..
		m_last_laser_scan3D->load();
		img = m_last_laser_scan3D->intensityImage;

		COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
		COpenGLViewport::Ptr viewp_intensity =
			scene->getViewport("viewp_intensity");
		viewp_intensity->setImageView(img);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}  // end of updateIntensityImageViewport

template <class GRAPH_T>
mrpt::opengl::CSetOfObjects::Ptr
	CGraphSlamEngine<GRAPH_T>::initRobotModelVisualization()
{
	pose_t p;
	return this->initRobotModelVisualizationInternal(p);
}  // end of initRobotModelVisualization

template <class GRAPH_T>
mrpt::opengl::CSetOfObjects::Ptr
	CGraphSlamEngine<GRAPH_T>::initRobotModelVisualizationInternal(
		const mrpt::poses::CPose2D& p_unused)
{
	return mrpt::opengl::stock_objects::RobotPioneer();

}  // end of initRobotModelVisualizationInternal - CPose2D

template <class GRAPH_T>
mrpt::opengl::CSetOfObjects::Ptr
	CGraphSlamEngine<GRAPH_T>::initRobotModelVisualizationInternal(
		const mrpt::poses::CPose3D& p_unused)
{
	return mrpt::opengl::stock_objects::CornerXYZ();
}  // end of initRobotModelVisualizationInternal - CPose3D

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initCurrPosViewport()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	COpenGLViewport::Ptr viewp =
		scene->createViewport("curr_robot_pose_viewport");
	// Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
	viewp->setCloneView("main");
	double x, y, h, w;
	m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp->setViewportPosition(x, y, h, w);
	viewp->setTransparent(false);
	viewp->getCamera().setAzimuthDegrees(90);
	viewp->getCamera().setElevationDegrees(90);
	viewp->setCustomBackgroundColor(
		mrpt::img::TColorf(205, 193, 197, /*alpha = */ 255));
	viewp->getCamera().setZoomDistance(30);
	viewp->getCamera().setOrthogonal();

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
inline void CGraphSlamEngine<GRAPH_T>::updateCurrPosViewport()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;
	using namespace mrpt::poses;

	ASSERTDEB_(m_enable_visuals);

	global_pose_t curr_robot_pose = this->getCurrentRobotPosEstimation();

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	COpenGLViewport::Ptr viewp = scene->getViewport("curr_robot_pose_viewport");
	viewp->getCamera().setPointingAt(CPose3D(curr_robot_pose));

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	MRPT_LOG_DEBUG_STREAM("Updated the \"current_pos\" viewport");

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::readGTFile(
	const std::string& fname_GT, std::vector<mrpt::poses::CPose2D>* gt_poses,
	std::vector<mrpt::system::TTimeStamp>* gt_timestamps)
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::system;

	// make sure file exists
	ASSERTDEBMSG_(
		fileExists(fname_GT),
		format(
			"\nGround-truth file %s was not found.\n"
			"Either specify a valid ground-truth filename or set set the "
			"m_visualize_GT flag to false\n",
			fname_GT.c_str()));

	mrpt::io::CFileInputStream file_GT(fname_GT);
	ASSERTDEBMSG_(file_GT.fileOpenCorrectly(), "\nCouldn't open GT file\n");
	ASSERTDEBMSG_(gt_poses, "\nNo valid std::vector<pose_t>* was given\n");

	string curr_line;

	// parse the file - get timestamp and pose and fill in the pose_t vector
	for (size_t line_num = 0; file_GT.readLine(curr_line); line_num++)
	{
		vector<string> curr_tokens;
		system::tokenize(curr_line, " ", curr_tokens);

		// check the current pose dimensions
		ASSERTDEBMSG_(
			curr_tokens.size() == constraint_t::state_length + 1,
			"\nGround Truth File doesn't seem to contain data as generated by "
			"the "
			"GridMapNavSimul application.\n Either specify the GT file format "
			"or set "
			"visualize_ground_truth to false in the .ini file\n");

		// timestamp
		if (gt_timestamps)
		{
			auto timestamp =
				mrpt::Clock::fromDouble(atof(curr_tokens[0].c_str()));
			gt_timestamps->push_back(timestamp);
		}

		// pose
		pose_t curr_pose(
			atof(curr_tokens[1].c_str()), atof(curr_tokens[2].c_str()),
			atof(curr_tokens[3].c_str()));
		gt_poses->push_back(curr_pose);
	}

	file_GT.close();

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::readGTFile(
	const std::string& fname_GT, std::vector<mrpt::poses::CPose3D>* gt_poses,
	std::vector<mrpt::system::TTimeStamp>* gt_timestamps)
{
	THROW_EXCEPTION("Not implemented.");
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::readGTFileRGBD_TUM(
	const std::string& fname_GT, std::vector<mrpt::poses::CPose2D>* gt_poses,
	std::vector<mrpt::system::TTimeStamp>* gt_timestamps)
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::system;

	// make sure file exists
	ASSERTDEBMSG_(
		fileExists(fname_GT),
		format(
			"\nGround-truth file %s was not found.\n"
			"Either specify a valid ground-truth filename or set set the "
			"m_visualize_GT flag to false\n",
			fname_GT.c_str()));

	mrpt::io::CFileInputStream file_GT(fname_GT);
	ASSERTDEBMSG_(
		file_GT.fileOpenCorrectly(),
		"\nreadGTFileRGBD_TUM: Couldn't openGT file\n");
	ASSERTDEBMSG_(gt_poses, "No valid std::vector<pose_t>* was given");

	string curr_line;

	// move to the first non-commented immediately - comments before this..
	for (size_t i = 0; file_GT.readLine(curr_line); i++)
	{
		if (curr_line.at(0) != '#')
		{
			break;
		}
	}

	// handle the first pose seperately
	// make sure that the ground-truth starts at 0.
	pose_t pose_diff;
	vector<string> curr_tokens;
	mrpt::system::tokenize(curr_line, " ", curr_tokens);

	// check the current pose dimensions
	ASSERTDEBMSG_(
		curr_tokens.size() == 8,
		"\nGround Truth File doesn't seem to contain data as specified in "
		"RGBD_TUM related "
		"datasets.\n Either specify correct the GT file format or set "
		"visualize_ground_truth to false in the .ini file\n");

	// quaternion
	CQuaternionDouble quat;
	quat.r(atof(curr_tokens[7].c_str()));
	quat.x(atof(curr_tokens[4].c_str()));
	quat.y(atof(curr_tokens[5].c_str()));
	quat.z(atof(curr_tokens[6].c_str()));
	double r, p, y;
	quat.rpy(r, p, y);

	CVectorDouble curr_coords(3);
	curr_coords[0] = atof(curr_tokens[1].c_str());
	curr_coords[1] = atof(curr_tokens[2].c_str());
	curr_coords[2] = atof(curr_tokens[3].c_str());

	// initial pose
	pose_t curr_pose(curr_coords[0], curr_coords[1], y);
	// pose_t curr_pose(0, 0, 0);

	pose_diff = curr_pose;

	// parse the file - get timestamp and pose and fill in the pose_t vector
	for (; file_GT.readLine(curr_line);)
	{
		system::tokenize(curr_line, " ", curr_tokens);
		ASSERTDEBMSG_(
			curr_tokens.size() == 8,
			"\nGround Truth File doesn't seem to contain data as specified in "
			"RGBD_TUM related "
			"datasets.\n Either specify correct the GT file format or set "
			"visualize_ground_truth to false in the .ini file\n");

		// timestamp
		if (gt_timestamps)
		{
			auto timestamp =
				mrpt::Clock::fromDouble(atof(curr_tokens[0].c_str()));
			gt_timestamps->push_back(timestamp);
		}

		// quaternion
		quat.r(atof(curr_tokens[7].c_str()));
		quat.x(atof(curr_tokens[4].c_str()));
		quat.y(atof(curr_tokens[5].c_str()));
		quat.z(atof(curr_tokens[6].c_str()));
		quat.rpy(r, p, y);

		// pose
		curr_coords[0] = atof(curr_tokens[1].c_str());
		curr_coords[1] = atof(curr_tokens[2].c_str());
		curr_coords[2] = atof(curr_tokens[3].c_str());

		// current ground-truth pose
		pose_t gt_pose(curr_coords[0], curr_coords[1], y);

		gt_pose.x() -= pose_diff.x();
		gt_pose.y() -= pose_diff.y();
		gt_pose.phi() -= pose_diff.phi();
		// curr_pose += -pose_diff;
		gt_poses->push_back(gt_pose);
	}

	file_GT.close();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::alignOpticalWithMRPTFrame()
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;

	// aligning GT (optical) frame with the MRPT frame
	// Set the rotation matrix from the corresponding RPY angles
	// MRPT Frame: X->forward; Y->Left; Z->Upward
	// Optical Frame: X->Right; Y->Downward; Z->Forward
	ASSERTDEB_(m_has_read_config);
	// rotz
	double anglez = DEG2RAD(0.0);
	const double tmpz[] = {
		cos(anglez), -sin(anglez), 0, sin(anglez), cos(anglez), 0, 0, 0, 1};
	CMatrixDouble rotz(3, 3, tmpz);

	// roty
	double angley = DEG2RAD(0.0);
	// double angley = DEG2RAD(90.0);
	const double tmpy[] = {cos(angley),  0, sin(angley), 0, 1, 0,
						   -sin(angley), 0, cos(angley)};
	CMatrixDouble roty(3, 3, tmpy);

	// rotx
	// double anglex = DEG2RAD(-90.0);
	double anglex = DEG2RAD(0.0);
	const double tmpx[] = {
		1, 0, 0, 0, cos(anglex), -sin(anglex), 0, sin(anglex), cos(anglex)};
	CMatrixDouble rotx(3, 3, tmpx);

	stringstream ss_out;
	ss_out << "\nConstructing the rotation matrix for the GroundTruth Data..."
		   << endl;
	m_rot_TUM_to_MRPT = rotz * roty * rotx;

	ss_out << "Rotation matrices for optical=>MRPT transformation" << endl;
	ss_out << "rotz: " << endl << rotz << endl;
	ss_out << "roty: " << endl << roty << endl;
	ss_out << "rotx: " << endl << rotx << endl;
	ss_out << "Full rotation matrix: " << endl << m_rot_TUM_to_MRPT << endl;

	MRPT_LOG_DEBUG_STREAM(ss_out);

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::queryObserverForEvents()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	ASSERTDEBMSG_(
		m_win_observer,
		"\nqueryObserverForEvents method was called even though no Observer "
		"object was provided\n");

	std::map<std::string, bool> events_occurred;
	m_win_observer->returnEventsStruct(&events_occurred);
	m_request_to_exit = events_occurred.find("Ctrl+c")->second;

	// odometry visualization
	if (events_occurred[m_keystroke_odometry])
	{
		this->toggleOdometryVisualization();
	}
	// GT visualization
	if (events_occurred[m_keystroke_GT])
	{
		this->toggleGTVisualization();
	}
	// Map visualization
	if (events_occurred[m_keystroke_map])
	{
		this->toggleMapVisualization();
	}
	// Estimated Trajectory Visualization
	if (events_occurred[m_keystroke_estimated_trajectory])
	{
		this->toggleEstimatedTrajectoryVisualization();
	}
	// pause/resume program execution
	if (events_occurred[m_keystroke_pause_exec])
	{
		this->togglePause();
	}

	// notify the deciders/optimizer of any events they may be interested in
	// MRPT_LOG_DEBUG_STREAM("Notifying deciders/optimizer for events");
	m_node_reg->notifyOfWindowEvents(events_occurred);
	m_edge_reg->notifyOfWindowEvents(events_occurred);
	m_optimizer->notifyOfWindowEvents(events_occurred);

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::toggleOdometryVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;
	MRPT_LOG_INFO_STREAM("Toggling Odometry visualization...");

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	if (m_visualize_odometry_poses)
	{
		CRenderizable::Ptr obj = scene->getByName("odometry_poses_cloud");
		obj->setVisibility(!obj->isVisible());

		obj = scene->getByName("robot_odometry_poses");
		obj->setVisibility(!obj->isVisible());
	}
	else
	{
		dumpVisibilityErrorMsg("visualize_odometry_poses");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::toggleGTVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;
	MRPT_LOG_INFO_STREAM("Toggling Ground Truth visualization");

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	if (m_visualize_GT)
	{
		CRenderizable::Ptr obj = scene->getByName("GT_cloud");
		obj->setVisibility(!obj->isVisible());

		obj = scene->getByName("robot_GT");
		obj->setVisibility(!obj->isVisible());
	}
	else
	{
		dumpVisibilityErrorMsg("visualize_ground_truth");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::toggleMapVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace std;
	using namespace mrpt::opengl;
	MRPT_LOG_INFO_STREAM("Toggling Map visualization... ");

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	// get total number of nodes
	int num_of_nodes;
	{
		std::lock_guard<std::mutex> graph_lock(m_graph_section);
		num_of_nodes = m_graph.nodeCount();
	}

	// name of gui object
	stringstream scan_name("");

	for (int node_cnt = 0; node_cnt != num_of_nodes; ++node_cnt)
	{
		// build the name of the potential corresponding object in the scene
		scan_name.str("");
		scan_name << "laser_scan_";
		scan_name << node_cnt;

		CRenderizable::Ptr obj = scene->getByName(scan_name.str());
		// current node may not have laserScans => may not have corresponding
		// obj
		if (obj)
		{
			obj->setVisibility(!obj->isVisible());
		}
	}
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::toggleEstimatedTrajectoryVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;
	MRPT_LOG_INFO_STREAM("Toggling Estimated Trajectory visualization... ");

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	if (m_visualize_estimated_trajectory)
	{
		CRenderizable::Ptr obj = scene->getByName("estimated_traj_setoflines");
		obj->setVisibility(!obj->isVisible());

		obj = scene->getByName("robot_estimated_traj");
		obj->setVisibility(!obj->isVisible());
	}
	else
	{
		dumpVisibilityErrorMsg("visualize_estimated_trajectory");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::dumpVisibilityErrorMsg(
	std::string viz_flag, int sleep_time)
{
	MRPT_START;

	MRPT_LOG_ERROR_STREAM(
		"Cannot toggle visibility of specified object.\n"
		<< "Make sure that the corresponding visualization flag (" << viz_flag
		<< ") is set to true in the .ini file.");
	std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

	MRPT_END;
}

template <class GRAPH_T>
mrpt::system::TTimeStamp CGraphSlamEngine<GRAPH_T>::getTimeStamp(
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observations,
	const mrpt::obs::CObservation::Ptr observation)
{
	MRPT_START;
	using namespace mrpt::obs;
	using namespace mrpt::system;

	// make sure that adequate data is given.
	ASSERTDEBMSG_(
		action || observation,
		"Neither action or observation contains valid data.");

	mrpt::system::TTimeStamp timestamp = INVALID_TIMESTAMP;
	if (observation)
	{
		timestamp = observation->timestamp;
	}
	else
	{
		// querry action part first
		timestamp = action->get(0)->timestamp;

		// if still not available query the observations in the CSensoryFrame
		if (timestamp == INVALID_TIMESTAMP)
		{
			for (auto sens_it = observations->begin();
				 sens_it != observations->end(); ++sens_it)
			{
				timestamp = (*sens_it)->timestamp;
				if (timestamp != INVALID_TIMESTAMP)
				{
					break;
				}
			}
		}
	}
	return timestamp;
	MRPT_END;
}

template <class GRAPH_T>
mrpt::poses::CPose3D
	CGraphSlamEngine<GRAPH_T>::getLSPoseForGridMapVisualization(
		const mrpt::graphs::TNodeID nodeID) const
{
	return mrpt::poses::CPose3D(m_graph.nodes.at(nodeID));
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initMapVisualization()
{
	using namespace mrpt::opengl;

	CSetOfObjects::Ptr map_obj = mrpt::make_aligned_shared<CSetOfObjects>();
	map_obj->setName("map");
	COpenGLScene::Ptr& scene = this->m_win->get3DSceneAndLock();
	scene->insert(map_obj);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateMapVisualization(
	const std::map<
		mrpt::graphs::TNodeID, mrpt::obs::CObservation2DRangeScan::Ptr>&
		nodes_to_laser_scans2D,
	bool full_update)
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace std;
	using namespace mrpt::poses;
	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	CSetOfObjects::Ptr map_obj;
	{
		CRenderizable::Ptr obj = scene->getByName("map");
		map_obj = std::dynamic_pointer_cast<CSetOfObjects>(obj);
		ASSERTDEB_(map_obj);
	}

	mrpt::system::CTicTac map_update_timer;
	map_update_timer.Tic();

	// get the set of nodes for which to run the update
	std::set<mrpt::graphs::TNodeID> nodes_set;
	{
		if (full_update)
		{
			// for all the nodes get the node position and the corresponding
			// laser scan
			// if they were recorded and visualize them
			m_graph.getAllNodes(nodes_set);
			MRPT_LOG_DEBUG_STREAM("Executing full update of the map visuals");
			map_obj->clear();

		}  // end if - full update
		else
		{  // add only current nodeID
			nodes_set.insert(m_nodeID_max);
		}  // end if - partial update
	}

	// for all the nodes in the previously populated set
	for (mrpt::graphs::TNodeID node_it : nodes_set)
	{
		// name of gui object
		stringstream scan_name("");
		scan_name << "laser_scan_";
		scan_name << node_it;

		// get the node laser scan
		CObservation2DRangeScan::Ptr scan_content;
		auto search = nodes_to_laser_scans2D.find(node_it);

		// make sure that the laser scan exists and is valid
		if (search != nodes_to_laser_scans2D.end() && search->second)
		{
			scan_content = search->second;

			CObservation2DRangeScan scan_decimated;
			this->decimateLaserScan(
				*scan_content, &scan_decimated,
				/*keep_every_n_entries = */ 5);

			// if the scan doesn't already exist, add it to the map object,
			// otherwise just
			// adjust its pose
			CRenderizable::Ptr obj = map_obj->getByName(scan_name.str());
			CSetOfObjects::Ptr scan_obj =
				std::dynamic_pointer_cast<CSetOfObjects>(obj);
			if (!scan_obj)
			{
				scan_obj = mrpt::make_aligned_shared<CSetOfObjects>();

				// creating and inserting the observation in the CSetOfObjects
				mrpt::maps::CSimplePointsMap m;
				m.insertObservation(&scan_decimated);
				m.getAs3DObject(scan_obj);

				scan_obj->setName(scan_name.str());
				this->setObjectPropsFromNodeID(node_it, scan_obj);

				// set the visibility of the object the same value as the
				// visibility of
				// the previous - Needed for proper toggling of the visibility
				// of the
				// whole map
				{
					stringstream prev_scan_name("");
					prev_scan_name << "laser_scan_" << node_it - 1;
					CRenderizable::Ptr prev_obj =
						map_obj->getByName(prev_scan_name.str());
					if (prev_obj)
					{
						scan_obj->setVisibility(prev_obj->isVisible());
					}
				}

				map_obj->insert(scan_obj);
			}
			else
			{
				scan_obj = std::dynamic_pointer_cast<CSetOfObjects>(scan_obj);
			}

			// finally set the pose correctly - as computed by graphSLAM
			const CPose3D& scan_pose = CPose3D(m_graph.nodes.at(node_it));
			scan_obj->setPose(scan_pose);
		}
		else
		{
			MRPT_LOG_DEBUG_STREAM(
				"Laser scans of NodeID " << node_it << "are  empty/invalid");
		}

	}  // end for set of nodes

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	double elapsed_time = map_update_timer.Tac();
	MRPT_LOG_DEBUG_STREAM(
		"updateMapVisualization took: " << elapsed_time << "s");
	MRPT_END;
}  // end of updateMapVisualization

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::setObjectPropsFromNodeID(
	const mrpt::graphs::TNodeID nodeID,
	mrpt::opengl::CSetOfObjects::Ptr& viz_object)
{
	MRPT_START;
	viz_object->setColor_u8(m_optimized_map_color);
	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::decimateLaserScan(
	mrpt::obs::CObservation2DRangeScan& laser_scan_in,
	mrpt::obs::CObservation2DRangeScan* laser_scan_out,
	const int keep_every_n_entries /*= 2*/)
{
	MRPT_START;

	size_t scan_size = laser_scan_in.scan.size();

	// assign the decimated scans, ranges
	std::vector<float> new_scan(
		scan_size);  // Was [], but can't use non-constant argument with arrays
	std::vector<char> new_validRange(scan_size);
	size_t new_scan_size = 0;
	for (size_t i = 0; i != scan_size; i++)
	{
		if (i % keep_every_n_entries == 0)
		{
			new_scan[new_scan_size] = laser_scan_in.scan[i];
			new_validRange[new_scan_size] = laser_scan_in.validRange[i];
			new_scan_size++;
		}
	}
	laser_scan_out->loadFromVectors(
		new_scan_size, &new_scan[0], &new_validRange[0]);

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initGTVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	// assertions
	ASSERTDEB_(m_has_read_config);
	ASSERTDEB_(
		m_win &&
		"Visualization of data was requested but no CDisplayWindow3D pointer "
		"was provided");

	// point cloud
	CPointCloud::Ptr GT_cloud = mrpt::make_aligned_shared<CPointCloud>();
	GT_cloud->setPointSize(1.0);
	GT_cloud->enablePointSmooth();
	GT_cloud->enableColorFromX(false);
	GT_cloud->enableColorFromY(false);
	GT_cloud->enableColorFromZ(false);
	GT_cloud->setColor_u8(m_GT_color);
	GT_cloud->setName("GT_cloud");

	// robot model
	CSetOfObjects::Ptr robot_model = this->setCurrentPositionModel(
		"robot_GT", m_GT_color, m_robot_model_size);

	// insert them to the scene
	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	scene->insert(GT_cloud);
	scene->insert(robot_model);
	m_win->unlockAccess3DScene();

	m_win_manager->assignTextMessageParameters(
		&m_offset_y_GT, &m_text_index_GT);
	m_win_manager->addTextMessage(
		m_offset_x_left, -m_offset_y_GT, mrpt::format("Ground truth path"),
		mrpt::img::TColorf(m_GT_color), m_text_index_GT);
	m_win->forceRepaint();

	MRPT_END;
}  // end of initGTVisualization

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateGTVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	// add to the GT PointCloud and visualize it
	// check that GT vector is not depleted
	if (m_visualize_GT && m_GT_poses_index < m_GT_poses.size())
	{
		ASSERTDEB_(
			m_win &&
			"Visualization of data was requested but no CDisplayWindow3D "
			"pointer was given");

		COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

		CRenderizable::Ptr obj = scene->getByName("GT_cloud");
		CPointCloud::Ptr GT_cloud = std::dynamic_pointer_cast<CPointCloud>(obj);

		// add the latest GT pose
		mrpt::poses::CPose3D p(m_GT_poses[m_GT_poses_index]);
		GT_cloud->insertPoint(p.x(), p.y(), p.z());

		// robot model of GT trajectory
		obj = scene->getByName("robot_GT");
		CSetOfObjects::Ptr robot_obj =
			std::dynamic_pointer_cast<CSetOfObjects>(obj);
		robot_obj->setPose(p);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}  // end of updateGTVisualization

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initOdometryVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_has_read_config);
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	// point cloud
	CPointCloud::Ptr odometry_poses_cloud =
		mrpt::make_aligned_shared<CPointCloud>();
	odometry_poses_cloud->setPointSize(1.0);
	odometry_poses_cloud->enablePointSmooth();
	odometry_poses_cloud->enableColorFromX(false);
	odometry_poses_cloud->enableColorFromY(false);
	odometry_poses_cloud->enableColorFromZ(false);
	odometry_poses_cloud->setColor_u8(m_odometry_color);
	odometry_poses_cloud->setName("odometry_poses_cloud");

	// robot model
	CSetOfObjects::Ptr robot_model = this->setCurrentPositionModel(
		"robot_odometry_poses", m_odometry_color, m_robot_model_size);

	// insert them to the scene
	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	scene->insert(odometry_poses_cloud);
	scene->insert(robot_model);
	m_win->unlockAccess3DScene();

	m_win_manager->assignTextMessageParameters(
		&m_offset_y_odometry, &m_text_index_odometry);
	m_win_manager->addTextMessage(
		m_offset_x_left, -m_offset_y_odometry, mrpt::format("Odometry path"),
		mrpt::img::TColorf(m_odometry_color), m_text_index_odometry);

	m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateOdometryVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	ASSERTDEBMSG_(
		m_win,
		"Visualization of data was requested but no CDisplayWindow3D pointer "
		"was given");
	using namespace mrpt::opengl;

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	// point cloud
	CRenderizable::Ptr obj = scene->getByName("odometry_poses_cloud");
	CPointCloud::Ptr odometry_poses_cloud =
		std::dynamic_pointer_cast<CPointCloud>(obj);
	mrpt::poses::CPose3D p(m_odometry_poses.back());

	odometry_poses_cloud->insertPoint(p.x(), p.y(), p.z());

	// robot model
	obj = scene->getByName("robot_odometry_poses");
	CSetOfObjects::Ptr robot_obj =
		std::dynamic_pointer_cast<CSetOfObjects>(obj);
	robot_obj->setPose(p);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initEstimatedTrajectoryVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	// SetOfLines
	CSetOfLines::Ptr estimated_traj_setoflines =
		mrpt::make_aligned_shared<CSetOfLines>();
	estimated_traj_setoflines->setColor_u8(m_estimated_traj_color);
	estimated_traj_setoflines->setLineWidth(1.5);
	estimated_traj_setoflines->setName("estimated_traj_setoflines");
	// append a dummy line so that you can later use append using
	// CSetOfLines::appendLienStrip method.
	estimated_traj_setoflines->appendLine(
		/* 1st */ 0, 0, 0,
		/* 2nd */ 0, 0, 0);

	// robot model
	// pose_t initial_pose;
	CSetOfObjects::Ptr robot_model = this->setCurrentPositionModel(
		"robot_estimated_traj", m_estimated_traj_color, m_robot_model_size);

	// insert objects in the graph
	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();
	if (m_visualize_estimated_trajectory)
	{
		scene->insert(estimated_traj_setoflines);
	}
	scene->insert(robot_model);
	m_win->unlockAccess3DScene();

	if (m_visualize_estimated_trajectory)
	{
		m_win_manager->assignTextMessageParameters(
			&m_offset_y_estimated_traj, &m_text_index_estimated_traj);
		m_win_manager->addTextMessage(
			m_offset_x_left, -m_offset_y_estimated_traj,
			mrpt::format("Estimated trajectory"),
			mrpt::img::TColorf(m_estimated_traj_color),
			m_text_index_estimated_traj);
	}

	m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateEstimatedTrajectoryVisualization(
	bool full_update)
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	using namespace mrpt::opengl;

	std::lock_guard<std::mutex> graph_lock(m_graph_section);
	ASSERTDEB_(m_graph.nodeCount() != 0);

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	CRenderizable::Ptr obj;
	if (m_visualize_estimated_trajectory)
	{
		// set of lines
		obj = scene->getByName("estimated_traj_setoflines");
		CSetOfLines::Ptr estimated_traj_setoflines =
			std::dynamic_pointer_cast<CSetOfLines>(obj);

		// gather set of nodes for which to append lines - all of the nodes in
		// the graph or just the last inserted..
		std::set<mrpt::graphs::TNodeID> nodes_set;
		{
			if (full_update)
			{
				this->getNodeIDsOfEstimatedTrajectory(&nodes_set);
				estimated_traj_setoflines->clear();
				// dummy way so that I can use appendLineStrip afterwards.
				estimated_traj_setoflines->appendLine(
					/* 1st */ 0, 0, 0,
					/* 2nd */ 0, 0, 0);
			}
			else
			{
				nodes_set.insert(m_graph.nodeCount() - 1);
			}
		}
		// append line for each node in the set
		for (unsigned long it : nodes_set)
		{
			mrpt::poses::CPose3D p(m_graph.nodes.at(it));

			estimated_traj_setoflines->appendLineStrip(p.x(), p.y(), p.z());
		}
	}

	// robot model
	// set the robot position to the last recorded pose in the graph
	obj = scene->getByName("robot_estimated_traj");
	CSetOfObjects::Ptr robot_obj =
		std::dynamic_pointer_cast<CSetOfObjects>(obj);
	pose_t curr_estimated_pos = m_graph.nodes[m_graph.nodeCount() - 1];
	robot_obj->setPose(curr_estimated_pos);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	MRPT_END;
}  // end of updateEstimatedTrajectoryVisualization

template <class GRAPH_T>
CGraphSlamEngine<GRAPH_T>::TRGBDInfoFileParams::TRGBDInfoFileParams(
	const std::string& rawlog_fname)
{
	this->setRawlogFile(rawlog_fname);
	this->initTRGBDInfoFileParams();
}
template <class GRAPH_T>
CGraphSlamEngine<GRAPH_T>::TRGBDInfoFileParams::TRGBDInfoFileParams()
{
	this->initTRGBDInfoFileParams();
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::TRGBDInfoFileParams::setRawlogFile(
	const std::string& rawlog_fname)
{
	// get the correct info filename from the rawlog_fname
	std::string dir = mrpt::system::extractFileDirectory(rawlog_fname);
	std::string rawlog_filename = mrpt::system::extractFileName(rawlog_fname);
	std::string name_prefix = "rawlog_";
	std::string name_suffix = "_info.txt";
	info_fname = dir + name_prefix + rawlog_filename + name_suffix;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::TRGBDInfoFileParams::initTRGBDInfoFileParams()
{
	// fields to use
	fields["Overall number of objects"] = "";
	fields["Observations format"] = "";
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::TRGBDInfoFileParams::parseFile()
{
	ASSERT_FILE_EXISTS_(info_fname);
	using namespace std;

	// open file
	mrpt::io::CFileInputStream info_file(info_fname);
	ASSERTDEBMSG_(
		info_file.fileOpenCorrectly(),
		"\nTRGBDInfoFileParams::parseFile: Couldn't open info file\n");

	string curr_line;
	size_t line_cnt = 0;

	// parse until you find an empty line.
	while (true)
	{
		info_file.readLine(curr_line);
		line_cnt++;
		if (curr_line.size() == 0) break;
	}

	// parse the meaningful data
	while (info_file.readLine(curr_line))
	{
		// split current line at ":"
		vector<string> curr_tokens;
		mrpt::system::tokenize(curr_line, ":", curr_tokens);

		ASSERTDEB_EQUAL_(curr_tokens.size(), 2);

		// evaluate the name. if name in info struct then fill the corresponding
		// info struct parameter with the value_part in the file.
		std::string literal_part = mrpt::system::trim(curr_tokens[0]);
		std::string value_part = mrpt::system::trim(curr_tokens[1]);

		for (auto it = fields.begin(); it != fields.end(); ++it)
		{
			if (mrpt::system::strCmpI(it->first, literal_part))
			{
				it->second = value_part;
			}
		}

		line_cnt++;
	}
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::saveGraph(const std::string* fname_in) const
{
	MRPT_START;

	// what's the name of the file to be saved?
	std::string fname;
	if (fname_in)
	{
		fname = *fname_in;
	}
	else
	{
		fname = "output_graph.graph";
	}

	MRPT_LOG_INFO_STREAM(
		"Saving generated graph in VERTEX/EDGE format: " << fname);
	m_graph.saveToTextFile(fname);

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::save3DScene(const std::string* fname_in) const
{
	MRPT_START;
	ASSERTDEBMSG_(
		m_enable_visuals,
		"\nsave3DScene was called even though enable_visuals flag is "
		"off.\nExiting...\n");
	using namespace mrpt::opengl;

	COpenGLScene::Ptr scene = m_win->get3DSceneAndLock();

	if (!scene)
	{
		MRPT_LOG_ERROR_STREAM(
			"Could not fetch 3D Scene. Display window must already be closed.");
		return;
	}

	// TODO - what else is there to be excluded from the scene?
	// remove the CPlanarLaserScan if it exists
	{
		CPlanarLaserScan::Ptr laser_scan;
		for (; (laser_scan = scene->getByClass<CPlanarLaserScan>());)
		{
			MRPT_LOG_DEBUG_STREAM(
				"Removing CPlanarLaserScan from generated 3DScene...");
			scene->removeObject(laser_scan);
		}
	}

	// what's the name of the file to be saved?
	std::string fname;
	if (fname_in)
	{
		fname = *fname_in;
	}
	else
	{
		fname = "output_scene.3DScene";
	}

	MRPT_LOG_INFO_STREAM("Saving generated scene to " << fname);
	scene->saveToFile(fname);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::computeSlamMetric(
	mrpt::graphs::TNodeID nodeID, size_t gt_index)
{
	MRPT_START;
	using namespace mrpt::math;
	using namespace mrpt::poses;

	// start updating the metric after a certain number of nodes have been added
	if (m_graph.nodeCount() < 4)
	{
		return;
	}

	// add to the map - keep track of which gt index corresponds to which nodeID
	m_nodeID_to_gt_indices[nodeID] = gt_index;

	// initialize the loop variables only once
	pose_t curr_node_pos;
	pose_t curr_gt_pos;
	pose_t node_delta_pos;
	pose_t gt_delta;
	double trans_diff = 0;
	double rot_diff = 0;

	size_t indices_size = m_nodeID_to_gt_indices.size();

	// recompute the metric from scratch
	m_curr_deformation_energy = 0;

	// first element of map
	auto start_it = m_nodeID_to_gt_indices.begin();
	start_it++;

	// fetch the first node, gt positions separately
	auto prev_it = start_it;
	prev_it--;
	pose_t prev_node_pos = m_graph.nodes[prev_it->first];
	pose_t prev_gt_pos = m_GT_poses[prev_it->second];

	// temporary constraint type
	constraint_t c;

	for (auto index_it = start_it; index_it != m_nodeID_to_gt_indices.end();
		 index_it++)
	{
		curr_node_pos = m_graph.nodes[index_it->first];
		curr_gt_pos = m_GT_poses[index_it->second];

		node_delta_pos = curr_node_pos - prev_node_pos;
		gt_delta = curr_gt_pos - prev_gt_pos;

		trans_diff = gt_delta.distanceTo(node_delta_pos);
		rot_diff = this->accumulateAngleDiffs(gt_delta, node_delta_pos);

		m_curr_deformation_energy += (pow(trans_diff, 2) + pow(rot_diff, 2));
		m_curr_deformation_energy /= indices_size;

		// add it to the overall vector
		m_deformation_energy_vec.push_back(m_curr_deformation_energy);

		prev_node_pos = curr_node_pos;
		prev_gt_pos = curr_gt_pos;
	}

	MRPT_LOG_DEBUG_STREAM(
		"Total deformation energy: " << m_curr_deformation_energy);

	MRPT_END;
}

template <class GRAPH_T>
double CGraphSlamEngine<GRAPH_T>::accumulateAngleDiffs(
	const mrpt::poses::CPose2D& p1, const mrpt::poses::CPose2D& p2)
{
	return mrpt::math::wrapToPi(p1.phi() - p2.phi());
}
template <class GRAPH_T>
double CGraphSlamEngine<GRAPH_T>::accumulateAngleDiffs(
	const mrpt::poses::CPose3D& p1, const mrpt::poses::CPose3D& p2)
{
	using mrpt::math::wrapToPi;
	double res = 0;

	res += wrapToPi(p1.roll() - p2.roll());
	res += wrapToPi(p1.pitch() - p2.pitch());
	res += wrapToPi(p1.yaw() - p2.yaw());

	return res;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::initSlamMetricVisualization()
{
	ASSERTDEB_(m_enable_visuals);

	MRPT_START;
	MRPT_LOG_DEBUG_STREAM("In initializeSlamMetricVisualization...");
	ASSERTDEB_(m_visualize_SLAM_metric);

	// initialize the m_win_plot on the stack
	m_win_plot = std::make_unique<mrpt::gui::CDisplayWindowPlots>(
		"Evolution of SLAM metric - Deformation Energy (1:1000)", 400, 300);

	m_win_plot->setPos(20, 50);
	m_win_plot->clf();
	// just plot the deformation points from scratch every time
	m_win_plot->hold_off();
	m_win_plot->enableMousePanZoom(true);

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::updateSlamMetricVisualization()
{
	MRPT_START;
	ASSERTDEB_(m_enable_visuals);
	ASSERTDEB_(m_win_plot && m_visualize_SLAM_metric);

	// build the X, Y vectors for plotting - use log scale
	std::vector<double> x(m_deformation_energy_vec.size(), 0);
	std::vector<double> y(m_deformation_energy_vec.size(), 0);
	for (size_t i = 0; i != x.size(); i++)
	{
		x[i] = i;
		y[i] = m_deformation_energy_vec[i] * 1000;
	}

	m_win_plot->plot(x, y, "r-1", "Deformation Energy (x1000)");

	// set the limits so that he y-values can be monitored
	// set the xmin limit with respect to xmax, which is constantly growing
	std::vector<double>::const_iterator xmax, ymax;
	xmax = std::max_element(x.begin(), x.end());
	ymax = std::max_element(y.begin(), y.end());

	m_win_plot->axis(
		/*x_min = */ xmax != x.end() ? -(*xmax / 12) : -1,
		/*x_max = */ (xmax != x.end() ? *xmax : 1),
		/*y_min = */ -0.4f,
		/*y_max = */ (ymax != y.end() ? *ymax : 1));

	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::getDescriptiveReport(
	std::string* report_str) const
{
	MRPT_START;
	using namespace std;

	// Summary of Results
	stringstream results_ss;
	results_ss << "Summary: " << std::endl;
	results_ss << this->header_sep << std::endl;
	results_ss << "\tProcessing time: "
			   << m_time_logger.getMeanTime("proc_time") << std::endl;
	;
	results_ss << "\tDataset Grab time: " << m_dataset_grab_time << std::endl;
	results_ss << "\tReal-time capable: "
			   << (m_time_logger.getMeanTime("proc_time") < m_dataset_grab_time
					   ? "TRUE"
					   : "FALSE")
			   << std::endl;
	results_ss << m_edge_counter.getAsString();
	results_ss << "\tNum of Nodes: " << m_graph.nodeCount() << std::endl;
	;

	// Class configuration parameters
	std::string config_params = this->getParamsAsString();

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += results_ss.str();
	*report_str += this->report_sep;

	*report_str += config_params;
	*report_str += this->report_sep;

	*report_str += time_res;
	*report_str += this->report_sep;

	*report_str += output_res;
	*report_str += this->report_sep;

	MRPT_END;
}

template <class GRAPH_T>
bool CGraphSlamEngine<GRAPH_T>::getGraphSlamStats(
	std::map<std::string, int>* node_stats,
	std::map<std::string, int>* edge_stats, mrpt::system::TTimeStamp* timestamp)
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::graphslam::detail;

	ASSERTDEBMSG_(node_stats, "Invalid pointer to node_stats is given");
	ASSERTDEBMSG_(edge_stats, "Invalid pointer to edge_stats is given");

	if (m_nodeID_max == 0)
	{
		return false;
	}

	// fill the node stats
	(*node_stats)["nodes_total"] = m_nodeID_max + 1;

	// fill the edge stats
	for (auto it = m_edge_counter.cbegin(); it != m_edge_counter.cend(); ++it)
	{
		(*edge_stats)[it->first] = it->second;
	}

	(*edge_stats)["loop_closures"] = m_edge_counter.getLoopClosureEdges();
	(*edge_stats)["edges_total"] = m_edge_counter.getTotalNumOfEdges();

	// fill the timestamp
	if (timestamp)
	{
		*timestamp = m_curr_timestamp;
	}

	return true;
	MRPT_END;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::generateReportFiles(
	const std::string& output_dir_fname)
{
	MRPT_START;
	using namespace mrpt::system;
	using namespace mrpt;

	ASSERTDEBMSG_(
		directoryExists(output_dir_fname),
		format(
			"Output directory \"%s\" doesn't exist", output_dir_fname.c_str()));

	MRPT_LOG_INFO_STREAM("Generating detailed class report...");
	std::lock_guard<std::mutex> graph_lock(m_graph_section);

	std::string report_str;
	std::string fname;
	const std::string ext = ".log";

	{  // CGraphSlamEngine
		report_str.clear();
		fname = output_dir_fname + "/" + m_class_name + ext;
		// initialize the output file - refer to the stream through the
		// m_out_streams map
		this->initResultsFile(fname);
		this->getDescriptiveReport(&report_str);
		m_out_streams[fname].printf("%s", report_str.c_str());

		// write the timings into a separate file
		const std::string time_res = m_time_logger.getStatsAsText();
		fname = output_dir_fname + "/" + "timings" + ext;
		this->initResultsFile(fname);
		m_out_streams[fname].printf("%s", time_res.c_str());
	}
	{  // node_registrar
		report_str.clear();
		fname = output_dir_fname + "/" + "node_registrar" + ext;
		this->initResultsFile(fname);
		m_node_reg->getDescriptiveReport(&report_str);
		m_out_streams[fname].printf("%s", report_str.c_str());
	}
	{  // edge_registrar
		report_str.clear();
		fname = output_dir_fname + "/" + "edge_registrar" + ext;
		this->initResultsFile(fname);
		m_edge_reg->getDescriptiveReport(&report_str);
		m_out_streams[fname].printf("%s", report_str.c_str());
	}
	{  // optimizer
		report_str.clear();
		fname = output_dir_fname + "/" + "optimizer" + ext;
		this->initResultsFile(fname);
		m_optimizer->getDescriptiveReport(&report_str);
		m_out_streams[fname].printf("%s", report_str.c_str());
	}

	if (m_use_GT)
	{  // slam evaluation metric
		report_str.clear();
		const std::string desc(
			"# File includes the evolution of the SLAM metric.  Implemented "
			"metric computes the \"deformation energy\" that is needed to "
			"transfer the estimated trajectory into the ground-truth "
			"trajectory (computed as sum of the difference between estimated "
			"trajectory and ground truth consecutive poses See \"A comparison "
			"of SLAM algorithms based on a graph of relations - W.Burgard et "
			"al.\", for more details on the metric.\n");

		fname = output_dir_fname + "/" + "SLAM_evaluation_metric" + ext;
		this->initResultsFile(fname);

		m_out_streams[fname].printf("%s\n", desc.c_str());
		for (auto vec_it = m_deformation_energy_vec.begin();
			 vec_it != m_deformation_energy_vec.end(); ++vec_it)
		{
			m_out_streams[fname].printf("%f\n", *vec_it);
		}
	}

	MRPT_END;
}
template <class GRAPH_T>
mrpt::opengl::CSetOfObjects::Ptr
	CGraphSlamEngine<GRAPH_T>::setCurrentPositionModel(
		const std::string& model_name, const mrpt::img::TColor& model_color,
		const size_t model_size, const pose_t& init_pose)
{
	mrpt::opengl::CSetOfObjects::Ptr model =
		this->initRobotModelVisualization();
	model->setName(model_name);
	model->setColor_u8(model_color);
	model->setScale(model_size);
	model->setPose(init_pose);

	return model;
}

template <class GRAPH_T>
void CGraphSlamEngine<GRAPH_T>::getDeformationEnergyVector(
	std::vector<double>* vec_out) const
{
	*vec_out = m_deformation_energy_vec;
}
}  // namespace mrpt::graphslam
