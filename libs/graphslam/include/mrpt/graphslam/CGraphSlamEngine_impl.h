/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAMENGINE_IMPL_H
#define CGRAPHSLAMENGINE_IMPL_H

namespace mrpt { namespace graphslam {

 // Ctors, Dtors implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CGraphSlamEngine<GRAPH_t>::CGraphSlamEngine(
		const std::string& config_file,
		const std::string rawlog_fname/* ="" */,
		const std::string fname_GT /* ="" */,
		mrpt::graphslam::CWindowManager* win_manager /* = NULL */,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* node_reg /* = NULL */,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* edge_reg /* = NULL */,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* optimizer /* = NULL */
		):
	m_node_registrar(node_reg),
	m_edge_registrar(edge_reg),
	m_optimizer(optimizer),
	m_enable_visuals(win_manager != NULL),
	m_config_fname(config_file),
	m_rawlog_fname(rawlog_fname),
	m_fname_GT(fname_GT),
	m_GT_poses_step(1),
	m_win_manager(win_manager),
	m_odometry_color(0, 0, 255),
	m_GT_color(0, 255, 0),
	m_estimated_traj_color(255, 165, 0),
	m_optimized_map_color(255, 0, 0),
	m_robot_model_size(1),
	m_graph_section("graph_sec"), // give the CCriticalSection a name for easier debugging
	m_class_name("CGraphSlamEngine")
{

	this->initCGraphSlamEngine();
};

template<class GRAPH_t>

CGraphSlamEngine<GRAPH_t>::~CGraphSlamEngine() {
	using namespace mrpt::utils;
	using namespace mrpt;
	using namespace std;

	MRPT_LOG_DEBUG_STREAM << "In Destructor: Deleting CGraphSlamEngine instance...";

	// close all open files
	for (fstreams_out_it it  = m_out_streams.begin(); it != m_out_streams.end();
			++it) {
		if ((it->second)->fileOpenCorrectly()) {
			MRPT_LOG_INFO_STREAM << "Closing file: " << (it->first);
			(it->second)->close();
		}
	}

	// delete m_odometry_poses
	if (m_odometry_poses.size()) {
		MRPT_LOG_DEBUG_STREAM << "Releasing m_odometry_poses vector; size: " << m_odometry_poses.size();
		for (size_t i = 0; i != m_odometry_poses.size(); ++i) {
			delete m_odometry_poses[i];
		}
		MRPT_LOG_DEBUG_STREAM << "Released m_odometry_poses vector";
	}

	// change back the CImage path
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
		MRPT_LOG_DEBUG_STREAM << "Changing back the CImage PATH";
		CImage::IMAGES_PATH_BASE = m_img_prev_path_base;
	}

	// delete the CDisplayWindowPlots object
	if (m_win_plot) {
		MRPT_LOG_DEBUG_STREAM << "Releasing CDisplayWindowPlots...";
		delete m_win_plot;
	}
}


// Member functions implementations
//////////////////////////////////////////////////////////////


template<class GRAPH_t>
typename GRAPH_t::constraint_t::type_value
CGraphSlamEngine<GRAPH_t>::getCurrentRobotPosEstimation() const {
	MRPT_START;

	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
	return m_node_registrar->getCurrentRobotPosEstimation();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::getRobotEstimatedTrajectory(
		mrpt::graphs::CNetworkOfPoses2DInf::global_poses_t* graph_poses) const {
	MRPT_START;

	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
	*graph_poses = m_graph.nodes;

	MRPT_END;
}



template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initCGraphSlamEngine() {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::opengl;
	using namespace std;

	// logger instance properties
	m_time_logger.setName(m_class_name);
	this->logging_enable_keep_record = true;
	this->setLoggerName(m_class_name);

	// If a valid CWindowManager pointer is given then visuals are on.
	if (m_enable_visuals) {
		m_win = m_win_manager->win;
		m_win_observer = m_win_manager->observer;
	}
	else {
		m_win = NULL;
		m_win_observer = NULL;

		MRPT_LOG_WARN_STREAM << "Visualization is off. Running on headless mode";
	}

	// set the CDisplayWindowPlots pointer to null for starters, we don't know if
	// we are using it
	m_win_plot = NULL;

	m_observation_only_dataset = false;
	m_request_to_exit = false;

	// max node number already in the graph
	m_nodeID_max = 0;
	m_graph.root = TNodeID(0);

	m_program_paused  = false;
	m_GT_poses_index  = 0;

	// pass the necessary variables/objects to the deciders/optimizes
	// pass a graph ptr after the instance initialization
	m_node_registrar->setGraphPtr(&m_graph);
	m_edge_registrar->setGraphPtr(&m_graph);
	m_optimizer->setGraphPtr(&m_graph);

	// pass the window manager pointer
	// note: m_win_manager contains a pointer to the CDisplayWindow3D instance
	if (m_enable_visuals) {
		m_node_registrar->setWindowManagerPtr(m_win_manager);
		m_edge_registrar->setWindowManagerPtr(m_win_manager);
		m_optimizer->setWindowManagerPtr(m_win_manager);
		m_edge_counter.setWindowManagerPtr(m_win_manager);
	}

	// pass a lock in case of multithreaded implementation
	m_node_registrar->setCriticalSectionPtr(&m_graph_section);
	m_edge_registrar->setCriticalSectionPtr(&m_graph_section);
	m_optimizer->setCriticalSectionPtr(&m_graph_section);

	// Load the parameters that each one of the self/deciders/optimizer classes
	// needs
	this->loadParams(m_config_fname);

	if (!m_enable_visuals) {
		MRPT_LOG_WARN_STREAM << "Switching all visualization parameters off...";
		m_visualize_odometry_poses       = 0;
		m_visualize_GT                   = 0;
		m_visualize_map                  = 0;
		m_visualize_estimated_trajectory = 0;
		m_visualize_SLAM_metric          = 0;
		m_enable_curr_pos_viewport       = 0;
		m_enable_range_viewport          = 0;
		m_enable_intensity_viewport      = 0;
	}

	m_use_GT = !m_fname_GT.empty();
	if (m_use_GT) {
		if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
			this->alignOpticalWithMRPTFrame();
			this->readGTFileRGBD_TUM(m_fname_GT, &m_GT_poses);
		}
		else if (mrpt::system::strCmpI(m_GT_file_format, "navsimul")) {
			this->readGTFileNavSimulOutput(m_fname_GT, &m_GT_poses);
		}
	}

	// plot the GT related visuals only if ground-truth file is given
	if (!m_use_GT) {
		MRPT_LOG_WARN_STREAM << "Ground truth file was not provided. Switching the related visualization parameters off...";
		m_visualize_GT          = 0;
		m_visualize_SLAM_metric = 0;
	}

	// timestamp
	if (m_enable_visuals) {
		m_win_manager->assignTextMessageParameters(&m_offset_y_timestamp,
				&m_text_index_timestamp);
	}

	// Configuration of various trajectories visualization
	ASSERT_(m_has_read_config);
	if (m_enable_visuals) {
		// odometry visualization
		if (m_visualize_odometry_poses) {
			this->initOdometryVisualization();
		}
		// GT Visualization
		if (m_visualize_GT) {
			this->initGTVisualization();
		}
		// estimated trajectory visualization
		this->initEstimatedTrajectoryVisualization();
		// current robot pose  viewport
		if (m_enable_curr_pos_viewport) {
			this->initCurrPosViewport();
		}
	}

	// change the CImage path in case of RGBD datasets
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
		// keep the last path - change back to it after rawlog parsing
		m_img_prev_path_base = CImage::IMAGES_PATH_BASE;

		std::string rawlog_fname_noext = system::extractFileName(m_rawlog_fname);
		std::string rawlog_dir = system::extractFileDirectory(m_rawlog_fname);
		std::string m_img_external_storage_dir = rawlog_dir + rawlog_fname_noext
			+ "_Images/";
		CImage::IMAGES_PATH_BASE = m_img_external_storage_dir;
	}

	// 3DRangeScans viewports initialization, in case of RGBD datasets
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
		if (m_enable_range_viewport) {
			this->initRangeImageViewport();
		}
		if (m_enable_intensity_viewport) {
			this->initIntensityImageViewport();
		}
	}
	// axis
	if (m_enable_visuals) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CAxisPtr obj = CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10,-10,-10, 10,10,10);
		obj->setName("axis");
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}


	// Add additional keystrokes in the CDisplayWindow3D help textBox
	if (m_enable_visuals) {

		// keystrokes initialization
		m_keystroke_pause_exec           = "p";
		m_keystroke_odometry             = "o";
		m_keystroke_GT                   = "g";
		m_keystroke_estimated_trajectory = "t";
		m_keystroke_map                  = "m";

		// keystrokes registration
		m_win_observer->registerKeystroke(m_keystroke_pause_exec,
				"Pause/Resume program execution");
		m_win_observer->registerKeystroke(m_keystroke_odometry,
				"Toggle Odometry visualization");
		m_win_observer->registerKeystroke(m_keystroke_GT,
				"Toggle Ground-Truth visualization");
		m_win_observer->registerKeystroke(m_keystroke_estimated_trajectory,
				"Toggle Estimated trajectory visualization");
		m_win_observer->registerKeystroke(m_keystroke_map,
				"Toggle Map visualization");
	}

	// register the types of edges
	vector<string> vec_edge_types;
	vec_edge_types.push_back("Odometry");
	vec_edge_types.push_back("ICP2D");
	vec_edge_types.push_back("ICP3D");

	for (vector<string>::const_iterator cit=vec_edge_types.begin();
			cit != vec_edge_types.end(); ++cit) {
		m_edge_counter.addEdgeType(*cit);
	}

	// Visualize the edge statistics
	if (m_enable_visuals) {
		// total edges - loop closure edges
		double offset_y_total_edges, offset_y_loop_closures;
		int text_index_total_edges, text_index_loop_closures;

		m_win_manager->assignTextMessageParameters(&offset_y_total_edges,
				&text_index_total_edges);


		// build each one of these
		map<string, double> name_to_offset_y;
		map<string, int> name_to_text_index;
		for (vector<string>::const_iterator it = vec_edge_types.begin();
				it != vec_edge_types.end();
				++it) {
			m_win_manager->assignTextMessageParameters(
					&name_to_offset_y[*it],
					&name_to_text_index[*it]);
		}

		m_win_manager->assignTextMessageParameters(
				&offset_y_loop_closures,
				&text_index_loop_closures);

		// add all the parameters to the CEdgeCounter object
		m_edge_counter.setTextMessageParams(
				name_to_offset_y, name_to_text_index,
				offset_y_total_edges, text_index_total_edges,
				offset_y_loop_closures, text_index_loop_closures);
	}

	// query node/edge deciders for visual objects initialization
	if (m_enable_visuals) {
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		m_time_logger.enter("Visuals");
		m_node_registrar->initializeVisuals();
		m_edge_registrar->initializeVisuals();
		m_optimizer->initializeVisuals();
		m_time_logger.leave("Visuals");
	}

	m_init_timestamp = INVALID_TIMESTAMP;

	m_gridmap_is_cached = false;
	m_gridmap_cached = mrpt::maps::COccupancyGridMap2D::Create();


	// In case we are given an RGBD TUM Dataset - try and read the info file so
	// that we know how to play back the GT poses.
	try {
		m_info_params.setRawlogFile(m_rawlog_fname);
		m_info_params.parseFile();
		// set the rate at which we read from the GT poses vector
		int num_of_objects = std::atoi(
				m_info_params.fields["Overall number of objects"].c_str());
		m_GT_poses_step = m_GT_poses.size() / num_of_objects;

		MRPT_LOG_INFO_STREAM <<	"Overall number of objects in rawlog: "
			<< num_of_objects;
		MRPT_LOG_INFO_STREAM << "Setting the Ground truth read step to: "
			<< m_GT_poses_step;
	}
	catch (std::exception& e) {
		MRPT_LOG_INFO_STREAM << "RGBD_TUM info file was not found: " << e.what();
	}

	// SLAM evaluation metric
	m_curr_deformation_energy = 0;
	if (m_visualize_SLAM_metric) {
		this->initSlamMetricVisualization();
	}

	MRPT_END;
}

template<class GRAPH_t>
bool CGraphSlamEngine<GRAPH_t>::execGraphSlamStep(
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry) {
	using namespace mrpt::obs;

	CActionCollectionPtr action;
	CSensoryFramePtr observations;

	return this->execGraphSlamStep(action, observations, observation, rawlog_entry);
}

template<class GRAPH_t>
bool CGraphSlamEngine<GRAPH_t>::execGraphSlamStep(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry) {
	MRPT_START;

	using namespace std;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace mrpt::system;

	m_time_logger.enter("proc_time");

	ASSERTMSG_(m_has_read_config,
			format("\nConfig file is not read yet.\nExiting... \n"));
	// good to go..

	// read first measurement independently if we haven't already
	if (m_init_timestamp == INVALID_TIMESTAMP) {
		MRPT_LOG_DEBUG_STREAM << "execGraphSlamStep: first run";

		if (observation.present()) {
			MRPT_LOG_DEBUG_STREAM << "Observation only dataset!";
			m_observation_only_dataset = true; // false by default
			m_init_timestamp = observation->timestamp;
		}
		else {
			MRPT_LOG_DEBUG_STREAM << "Action-observation dataset!";
			m_observation_only_dataset = false;
			if (action->getBestMovementEstimation()) {

				CActionRobotMovement2DPtr robot_move =
					action->getBestMovementEstimation();
				CPosePDFPtr increment = robot_move->poseChange.get_ptr();
				pose_t increment_pose = increment->getMeanVal();
				m_curr_odometry_only_pose += increment_pose;

				m_init_timestamp = robot_move->timestamp;
			}
		}
	}


	// node registration procedure
	bool registered_new_node;
	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		m_time_logger.enter("node_registrar");
		registered_new_node = m_node_registrar->updateState(
				action, observations, observation);
		m_time_logger.leave("node_registrar");
		if (registered_new_node) {
			m_edge_counter.addEdge("Odometry");
			m_nodeID_max++;
		}
	}
	// Edge registration procedure
	// run this so that the decider can be updated with the latest
	// observations even when no new nodes have been added to the graph
	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

		m_time_logger.enter("edge_registrar");
		m_edge_registrar->updateState(
				action,
				observations,
				observation );
		m_time_logger.leave("edge_registrar");

		m_time_logger.enter("optimizer");
		m_optimizer->updateState(
				action,
				observations,
				observation );
		m_time_logger.leave("optimizer");
	}

	// current timestamp - to be filled depending on the format
	mrpt::system::TTimeStamp timestamp;

	if (observation.present()) {
		// Read a single observation from the rawlog
		// (Format #2 rawlog file)

		timestamp = observation->timestamp;

		// odometry
		if (IS_CLASS(observation, CObservationOdometry)) {
			CObservationOdometryPtr obs_odometry =
				static_cast<CObservationOdometryPtr>(observation);

			m_curr_odometry_only_pose = obs_odometry->odometry;
			// add to the odometry vector
			{
				pose_t* odometry_pose = new pose_t;
				*odometry_pose = m_curr_odometry_only_pose;
				m_odometry_poses.push_back(odometry_pose);
			}
		}
		// laser scans
		else if (IS_CLASS(observation, CObservation2DRangeScan)) {
			m_last_laser_scan2D =
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);

		}
		else if (IS_CLASS(observation, CObservation3DRangeScan)) {
			m_last_laser_scan3D =
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);

		}
	}
	else {
		// action, observations should contain a pair of valid data
		// (Format #1 rawlog file)

		// parse the current action
		CActionRobotMovement2DPtr robot_move =
			action->getBestMovementEstimation();
		CPosePDFPtr increment = robot_move->poseChange.get_ptr();
		pose_t increment_pose = increment->getMeanVal();
		m_curr_odometry_only_pose += increment_pose;

		timestamp = robot_move->timestamp;

		// add to the odometry vector
		{
			pose_t* odometry_pose = new pose_t;
			*odometry_pose = m_curr_odometry_only_pose;
			m_odometry_poses.push_back(odometry_pose);
		}

		// get the last laser scan
		m_last_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();

	} // ELSE FORMAT #1 - Action/Observations

	if (registered_new_node) {

		// update the global position of the nodes
		{
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
			m_time_logger.enter("dijkstra_nodes_estimation");
			m_graph.dijkstra_nodes_estimate();
			m_time_logger.leave("dijkstra_nodes_estimation");
		}

		// keep track of the laser scans so that I can later visualize the map
		m_nodes_to_laser_scans2D[m_nodeID_max] = m_last_laser_scan2D;

		if (m_enable_visuals && m_visualize_map) {
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
			bool full_update = m_edge_registrar->justInsertedLoopClosure();
			this->updateMapVisualization(m_graph, m_nodes_to_laser_scans2D, full_update);
		}

		// query node/edge deciders for visual objects update
		if (m_enable_visuals) {
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
			m_time_logger.enter("Visuals");
			m_node_registrar->updateVisuals();
			m_edge_registrar->updateVisuals();
			m_optimizer->updateVisuals();
			m_time_logger.leave("Visuals");
		}

		// update the edge counter
		std::map<std::string, int> edge_types_to_nums;
		m_edge_registrar->getEdgesStats(&edge_types_to_nums);
		if (edge_types_to_nums.size()) {
			for (std::map<std::string, int>::const_iterator it =
					edge_types_to_nums.begin(); it != edge_types_to_nums.end();
					++it) {

				// loop closure
				if (mrpt::system::strCmpI(it->first, "lc")) {
					m_edge_counter.setLoopClosureEdgesManually(it->second);
				}
				else {
					m_edge_counter.setEdgesManually(it->first, it->second);
				}
			}
		}
		// update the graph visualization

		if (m_enable_curr_pos_viewport) {
			updateCurrPosViewport();
		}
		// update visualization of estimated trajectory
		if (m_enable_visuals) {
			bool full_update = true;
			m_time_logger.enter("Visuals");
			this->updateEstimatedTrajectoryVisualization(full_update);
			m_time_logger.leave("Visuals");
		}

		// refine the SLAM metric  and update its corresponding visualization
		// This is done only when the GT is available.
		if (m_use_GT) {
			m_time_logger.enter("SLAM_metric");
			this->computeSlamMetric(m_nodeID_max, m_GT_poses_index);
			m_time_logger.leave("SLAM_metric");
			if (m_visualize_SLAM_metric) {
				m_time_logger.enter("Visuals");
				this->updateSlamMetricVisualization();
				m_time_logger.leave("Visuals");
			}
		}

		// mark the gridmap outdated
		m_gridmap_is_cached = false;

	} // IF REGISTERED_NEW_NODE

	//
	// Visualization related actions
	//
	m_time_logger.enter("Visuals");
	// Timestamp textMessage
	// Use the dataset timestamp otherwise fallback to
	// mrpt::system::getCurrentTime
	if (m_enable_visuals) {
		if (timestamp != INVALID_TIMESTAMP) {
			m_win_manager->addTextMessage(5,-m_offset_y_timestamp,
					format("Simulated time: %s", timeToString(timestamp).c_str()),
					TColorf(1.0, 1.0, 1.0),
					/* unique_index = */ m_text_index_timestamp );
		}
		else {
			m_win_manager->addTextMessage(5,-m_offset_y_timestamp,
					format("Wall time: %s", timeToString(mrpt::system::getCurrentTime()).c_str()),
					TColorf(1.0, 1.0, 1.0),
					/* unique_index = */ m_text_index_timestamp );
		}
	}

	// Odometry visualization
	if (m_visualize_odometry_poses && m_odometry_poses.size()) {
		this->updateOdometryVisualization();
	}

	// ensure that the GT is visualized at the same rate as the SLAM procedure
	// handle RGBD-TUM datasets manually. Advance the GT index accordingly
	if (m_use_GT) {
		if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) { // 1/loop
		  if (m_enable_visuals) {
			  this->updateGTVisualization(); // I have already taken care of the step
			}
			m_GT_poses_index += m_GT_poses_step;
		}
		else if (mrpt::system::strCmpI(m_GT_file_format, "navsimul")) {
			if (m_observation_only_dataset) { // 1/2loops
				MRPT_LOG_DEBUG_STREAM <<
					"observation_only_dataset: Updating GTVisualization";
				if (rawlog_entry % 2 == 0) {
		      if (m_enable_visuals) {
			      this->updateGTVisualization(); // I have already taken care of the step
			    }
					m_GT_poses_index += m_GT_poses_step;
					MRPT_LOG_DEBUG_STREAM << "rawlog_entry%2==0 " << std::endl;
				}
			}
			else { // 1/loop
				// get both action and observation at a single step - same rate as GT
				MRPT_LOG_DEBUG_STREAM <<
					"action-observations dataset: Updating GTVisualization";
		    if (m_enable_visuals) {
			    this->updateGTVisualization(); // I have already taken care of the step
			  }
				m_GT_poses_index += m_GT_poses_step;
			}
		}
	}

	// 3DRangeScans viewports update
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
		if (m_enable_range_viewport && !m_last_laser_scan3D.null()) {
			this->updateRangeImageViewport();
		}

		if (m_enable_intensity_viewport && !m_last_laser_scan3D.null()) {
			this->updateIntensityImageViewport();
		}
	}

	// Query for events and take corresponding actions
	if (m_enable_visuals) {
		this->queryObserverForEvents();
	}
	m_time_logger.leave("Visuals");

	m_dataset_grab_time = mrpt::system::timeDifference(
			m_init_timestamp,
			timestamp);
	m_time_logger.leave("proc_time");

	return !m_request_to_exit;
	MRPT_END;
} // END OF EXECGRAPHSLAM
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::getOccupancyGridMap2D(
		mrpt::maps::COccupancyGridMap2D* map_ptr,
		mrpt::system::TTimeStamp* acquisition_time /* = NULL */) const {
	MRPT_START;
	ASSERT_(map_ptr);

	if (!m_gridmap_is_cached){
		this->computeOccupancyGridMap2D();
	}

	map_ptr->copyMapContentFrom(*m_gridmap_cached);

	// fill the timestamp if this is given
	if (acquisition_time) {
		*acquisition_time = m_gridmap_acq_time;
	}
	MRPT_END;
}


template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::computeOccupancyGridMap2D() const {
	MRPT_START;
	using namespace std;
	using namespace mrpt::maps;
	using namespace mrpt::utils;
	using namespace mrpt::poses;

	MRPT_LOG_DEBUG_STREAM << "Computing the occupancy gridmap...";
	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

	// set the map parameters
	mrpt::maps::COccupancyGridMap2D gridmap(
			/* min_x = */ -20.0f,
			/* float max_x = */ 20.0f,
			/* float min_y = */ -20.0f,
			/* float max_y = */ 20.0f,
			/* float resolution = */ 0.05f);

	// observation insertion options
  gridmap.insertionOptions.maxOccupancyUpdateCertainty = 0.8f;

  gridmap.insertionOptions.maxDistanceInsertion = 15;
  gridmap.insertionOptions.wideningBeamsWithDistance = true;
  gridmap.insertionOptions.decimation = 2;

	// traverse all the nodes - add their laser scans at their corresponding poses
	for (std::map<mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr>::const_iterator
			it = m_nodes_to_laser_scans2D.begin();
			it != m_nodes_to_laser_scans2D.end(); ++it) {

		TNodeID curr_node = it->first;
		mrpt::obs::CObservation2DRangeScanPtr curr_laser_scan = it->second;
		pose_t curr_pose;

		bool laser_scan_exists = !curr_laser_scan.null();

		bool pose_found = true;
		typename mrpt::graphs::CNetworkOfPoses2DInf::global_poses_t::const_iterator
			pose_search = m_graph.nodes.find(curr_node);
		if (pose_search != m_graph.nodes.end()) {
			curr_pose = pose_search->second;
		}
		else {
			pose_found = false;
		}

		if (laser_scan_exists && pose_found) {
			CPose3D pose_3d(curr_pose);
			gridmap.insertObservation(curr_laser_scan.pointer(), &pose_3d);
		}
	}

	m_gridmap_cached->copyMapContentFrom(gridmap);
	m_gridmap_is_cached = true;
	m_gridmap_acq_time = mrpt::system::now();

	MRPT_LOG_DEBUG_STREAM << "Computed the occupancy gridmap successfully.";
	MRPT_END;
}



template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::loadParams(
		const std::string& fname) {
	MRPT_START;
	using namespace mrpt::utils;

	ASSERTMSG_(mrpt::system::fileExists(fname),
			mrpt::format("\nConfiguration file not found: \n%s\n", fname.c_str()));

	MRPT_LOG_INFO_STREAM << "Reading the .ini file... ";

	CConfigFile cfg_file(fname);

	// Section: GeneralConfiguration
	// ////////////////////////////////
	m_user_decides_about_output_dir = cfg_file.read_bool(
			"GeneralConfiguration",
			"user_decides_about_output_dir",
			false, false);
	m_GT_file_format = cfg_file.read_string(
			"GeneralConfiguration",
			"ground_truth_file_format",
			"NavSimul", false);

	// Minimum verbosity level of the logger
	int min_verbosity_level = cfg_file.read_int(
			"GeneralConfiguration",
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	// Section: VisualizationParameters
	// ////////////////////////////////

	// map visualization
	m_visualize_map = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_map",
			true, false);

	// odometry-only visualization
	m_visualize_odometry_poses = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_odometry_poses",
			true, false);
	m_visualize_estimated_trajectory = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_estimated_trajectory",
			true, false);

	// GT configuration visualization
	m_visualize_GT = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_ground_truth",
			true, false);
	// SLAM metric plot
	m_visualize_SLAM_metric = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_SLAM_metric",
			true, false);

	// Viewports flags
	m_enable_curr_pos_viewport = cfg_file.read_bool(
			"VisualizationParameters",
			"enable_curr_pos_viewport",
			true, false);
	m_enable_range_viewport = cfg_file.read_bool(
			"VisualizationParameters",
			"enable_range_viewport",
			false, false);
	m_enable_intensity_viewport = cfg_file.read_bool(
			"VisualizationParameters",
			"enable_intensity_viewport",
			false, false);

	m_node_registrar->loadParams(fname);
	m_edge_registrar->loadParams(fname);
	m_optimizer->loadParams(fname);

	m_has_read_config = true;
	MRPT_END;
}
template<class GRAPH_t>
std::string CGraphSlamEngine<GRAPH_t>::getParamsAsString() const {
	MRPT_START;

	std::string str;
	this->getParamsAsString(&str);
	return str;

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::getParamsAsString(
		std::string* params_out) const {
	MRPT_START;
	ASSERT_(m_has_read_config);
	using namespace std;

	stringstream ss_out;

	ss_out << "\n------------[ Graphslam_engine Problem Parameters ]------------"
		<< std::endl;
	ss_out << "Config filename                 = "
		<< m_config_fname << std::endl;

	ss_out << "Ground Truth File format        = "
		<< m_GT_file_format << std::endl;
	ss_out << "Ground Truth filename           = "
		<< m_fname_GT << std::endl;

	ss_out << "Visualize odometry              = "
		<< ( m_visualize_odometry_poses ? "TRUE" : "FALSE" ) << std::endl;
	ss_out << "Visualize estimated trajectory  = "
		<< ( m_visualize_estimated_trajectory ? "TRUE" : "FALSE" ) << std::endl;
	ss_out << "Visualize map                   = "
		<< ( m_visualize_map ? "TRUE" : "FALSE" ) << std::endl;
	ss_out << "Visualize Ground Truth          = "
		<< ( m_visualize_GT ? "TRUE" : "FALSE" ) << std::endl;

	ss_out << "Visualize SLAM metric plot      = "
		<< ( m_visualize_SLAM_metric ? "TRUE" : "FALSE" ) << std::endl;

	ss_out << "Enable curr. position viewport  = "
		<< ( m_enable_curr_pos_viewport ? "TRUE" : "FALSE" ) << endl;
	ss_out << "Enable range img viewport       = "
		<< ( m_enable_range_viewport ? "TRUE" : "FALSE" ) << endl;
	ss_out << "Enable intensity img viewport   = "
		<< ( m_enable_intensity_viewport ? "TRUE" : "FALSE" ) << endl;

	ss_out << "-----------------------------------------------------------"
		<< std::endl;
	ss_out << std::endl;

	// copy the stringstream contents to the passed in string
	*params_out = ss_out.str();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::printParams() const {
	MRPT_START;
	std::cout << getParamsAsString();

	m_node_registrar->printParams();
	m_edge_registrar->printParams();
	m_optimizer->printParams();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initOutputDir(
		const std::string& output_dir_fname /* = graphslam_results */) {
	MRPT_START;
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::system;

	MRPT_LOG_INFO_STREAM << "Setting up output directory: " << output_dir_fname;

	// current time vars - handy in the rest of the function.
	mrpt::system::TTimeStamp cur_date(getCurrentTime());
	string cur_date_str(dateTimeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));

	ASSERTMSG_(m_has_read_config,
			"\nCannot initialize output directory, \nMake sure that you have parsed the configuration file first\n");

	// Determine what to do with existing results if previous output directory
	// exists
	if (directoryExists(output_dir_fname)) {
		int answer_int;
		if (m_user_decides_about_output_dir) {
			stringstream question;
			string answer;

			question << "Directory exists. Choose between the "
				<< "following options" << std::endl;
			question << "\t 1: Rename current folder and start new "
				<< "output directory (default)" << std::endl;
			question << "\t 2: Remove existing contents and continue execution "
				<< std::endl;
			question << "\t 3: Handle potential conflict manually "
				"(Halts program execution)" << std::endl;
			question << "\t [ 1 | 2 | 3 ] --> ";
			std::cout << question.str();

			getline(cin, answer);
			answer = mrpt::system::trim(answer);
			answer_int = atoi(&answer[0]);
		}
		else {
			answer_int = 2;
		}

		switch (answer_int)
		{
			case 2:
				{
					MRPT_LOG_INFO_STREAM << "Deleting existing files...";
					// purge directory
					deleteFilesInDirectory(output_dir_fname,
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
					MRPT_LOG_INFO_STREAM << "Renaming directory to: " << dst_fname;
					string* error_msg = NULL;
					bool did_rename = renameFile(output_dir_fname,
							dst_fname,
							error_msg);
					ASSERTMSG_(did_rename,
							format("\nError while trying to rename the output directory: %s",
								error_msg->c_str()) );
					break;
				}
		} // SWITCH (ANSWER_INT)
	} // IF DIRECTORY EXISTS..

	// Now rebuild the directory from scratch
	MRPT_LOG_INFO_STREAM << "Creating the new directory structure...";
	string cur_fname;

	// debug_fname
	createDirectory(output_dir_fname);
	MRPT_LOG_INFO_STREAM << "Finished initializing output directory.";

	MRPT_END;
} // end of initOutputDir

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initResultsFile(
		const std::string& fname) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;
	using namespace mrpt::system;

	MRPT_LOG_INFO_STREAM << "Setting up file: " << fname;

	// current time vars - handy in the rest of the function.
	mrpt::system::TTimeStamp cur_date(getCurrentTime());
	string cur_date_str(dateTimeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));

	m_out_streams[fname] = new CFileOutputStream(fname);
	ASSERTMSG_(m_out_streams[fname]->fileOpenCorrectly(),
			mrpt::format("\nError while trying to open %s\n", fname.c_str()) );

	const std::string sep(80, '#');

	m_out_streams[fname]->printf("# Mobile Robot Programming Toolkit (MRPT)\n");
	m_out_streams[fname]->printf("# http::/www.mrpt.org\n");
	m_out_streams[fname]->printf("# GraphSlamEngine Application\n");
	m_out_streams[fname]->printf("# Automatically generated file - %s\n", cur_date_str.c_str());
	m_out_streams[fname]->printf("%s\n\n", sep.c_str());

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initRangeImageViewport() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_range;

	viewp_range = scene->createViewport("viewp_range");
	double x,y,h,w;
	m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp_range->setViewportPosition(x, y, h, w);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::updateRangeImageViewport() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::math;
	using namespace mrpt::opengl;

	if (m_last_laser_scan3D->hasRangeImage) {

	// TODO - make this a static class member - or at least a private member of the class
	CMatrixFloat range2D;
	mrpt::utils::CImage img;

	// load the image if not already loaded..
	m_last_laser_scan3D->load();
	range2D = m_last_laser_scan3D->rangeImage * (1.0f/5.0); // TODO - without the magic number?
	img.setFromMatrix(range2D);

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_range = scene->getViewport("viewp_range");
	viewp_range->setImageView(img);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	}

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initIntensityImageViewport() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_intensity;

	viewp_intensity = scene->createViewport("viewp_intensity");
	double x, y, w, h;
	m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp_intensity->setViewportPosition(x, y, w, h);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::updateIntensityImageViewport() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;

	if (m_last_laser_scan3D->hasIntensityImage) {
	mrpt::utils::CImage img ;

	// load the image if not already loaded..
	m_last_laser_scan3D->load();
	img = m_last_laser_scan3D->intensityImage;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_intensity = scene->getViewport("viewp_intensity");
	viewp_intensity->setImageView(img);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	}

	MRPT_END;
}


template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initCurrPosViewport() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;
	using namespace mrpt::utils;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp= scene->createViewport("curr_robot_pose_viewport");
	// Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
	viewp->setCloneView("main");
	double x,y,h,w;
	m_win_manager->assignViewportParameters(&x, &y, &w, &h);
	viewp->setViewportPosition(x, y, h, w);
	viewp->setTransparent(false);
	viewp->getCamera().setAzimuthDegrees(90);
	viewp->getCamera().setElevationDegrees(90);
	viewp->setCustomBackgroundColor(TColorf(205, 193, 197, /*alpha = */ 255));
	viewp->getCamera().setZoomDistance(30);
	viewp->getCamera().setOrthogonal();

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}



template<class GRAPH_t>
inline void CGraphSlamEngine<GRAPH_t>::updateCurrPosViewport() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	using namespace mrpt::poses;

	ASSERT_(m_enable_visuals);

	pose_t curr_robot_pose = this->getCurrentRobotPosEstimation();

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp = scene->getViewport("curr_robot_pose_viewport");
	viewp->getCamera().setPointingAt(CPose3D(curr_robot_pose));

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	MRPT_LOG_DEBUG_STREAM << "Updated the \"current_pos\" viewport";

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::readGTFileNavSimulOutput(
		const std::string& fname_GT,
		std::vector<pose_t>* gt_poses,
		std::vector<mrpt::system::TTimeStamp>* gt_timestamps /* = NULL */) {
	MRPT_START;
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::system;

	// make sure file exists
	ASSERTMSG_(fileExists(fname_GT),
			format("\nGround-truth file %s was not found.\n"
				"Either specify a valid ground-truth filename or set set the "
				"m_visualize_GT flag to false\n", fname_GT.c_str()));

	CFileInputStream file_GT(fname_GT);
	ASSERT_(file_GT.fileOpenCorrectly() &&
			"\nreadGTFileNavSimulOutput: Couldn't open GT file\n");
	ASSERTMSG_(gt_poses, "No valid std::vector<pose_t>* was given");

	string curr_line;

	// parse the file - get timestamp and pose and fill in the pose_t vector
	for (size_t line_num = 0; file_GT.readLine(curr_line); line_num++) {
		vector<string> curr_tokens;
		system::tokenize(curr_line, " ", curr_tokens);

		// check the current pose dimensions
		ASSERTMSG_(curr_tokens.size() == constraint_t::state_length + 1,
				"\nGround Truth File doesn't seem to contain data as generated by the "
				"GridMapNavSimul application.\n Either specify the GT file format or set "
				"visualize_ground_truth to false in the .ini file\n");

		// timestamp
		if (gt_timestamps) {
			mrpt::system::TTimeStamp timestamp(atof(curr_tokens[0].c_str()));
			gt_timestamps->push_back(timestamp);
		}

		// pose
		pose_t curr_pose(
				atof(curr_tokens[1].c_str()),
				atof(curr_tokens[2].c_str()),
				atof(curr_tokens[3].c_str()) );
		gt_poses->push_back(curr_pose);
	}

	file_GT.close();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::readGTFileRGBD_TUM(
		const std::string& fname_GT,
		std::vector<pose_t>* gt_poses,
		std::vector<mrpt::system::TTimeStamp>* gt_timestamps/*= NULL */) {
	MRPT_START;
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace mrpt::system;

	// make sure file exists
	ASSERTMSG_(fileExists(fname_GT),
			format("\nGround-truth file %s was not found.\n"
				"Either specify a valid ground-truth filename or set set the "
				"m_visualize_GT flag to false\n", fname_GT.c_str()));

	CFileInputStream file_GT(fname_GT);
	ASSERTMSG_(file_GT.fileOpenCorrectly(),
			"\nreadGTFileRGBD_TUM: Couldn't openGT file\n");
	ASSERTMSG_(gt_poses, "No valid std::vector<pose_t>* was given");

	string curr_line;

	// move to the first non-commented immediately - comments before this..
	for (size_t i = 0; file_GT.readLine(curr_line) ; i++) {
		if (curr_line.at(0) != '#') {
			break;
		}
	}

	// handle the first pose seperately
	// make sure that the ground-truth starts at 0.
	pose_t pose_diff;
	vector<string> curr_tokens;
	mrpt::system::tokenize(curr_line, " ", curr_tokens);

	// check the current pose dimensions
	ASSERTMSG_(curr_tokens.size() == 8,
			"\nGround Truth File doesn't seem to contain data as specified in RGBD_TUM related "
			"datasets.\n Either specify correct the GT file format or set "
			"visualize_ground_truth to false in the .ini file\n");

	// quaternion
	CQuaternionDouble quat;
	quat.r(atof(curr_tokens[7].c_str()));
	quat.x(atof(curr_tokens[4].c_str()));
	quat.y(atof(curr_tokens[5].c_str()));
	quat.z(atof(curr_tokens[6].c_str()));
	double r,p,y;
	quat.rpy(r, p, y);

	CVectorDouble curr_coords(3);
	curr_coords[0] = atof(curr_tokens[1].c_str());
	curr_coords[1] = atof(curr_tokens[2].c_str());
	curr_coords[2] = atof(curr_tokens[3].c_str());

	// initial pose
	pose_t curr_pose(
			curr_coords[0],
			curr_coords[1],
			y);
	//pose_t curr_pose(0, 0, 0);

	pose_diff = curr_pose;

	// parse the file - get timestamp and pose and fill in the pose_t vector
	for (; file_GT.readLine(curr_line) ;) {
		vector<string> curr_tokens;
		system::tokenize(curr_line, " ", curr_tokens);
		ASSERTMSG_(curr_tokens.size() == 8,
				"\nGround Truth File doesn't seem to contain data as specified in RGBD_TUM related "
				"datasets.\n Either specify correct the GT file format or set "
				"visualize_ground_truth to false in the .ini file\n");

		// timestamp
		if (gt_timestamps) {
			mrpt::system::TTimeStamp timestamp(atof(curr_tokens[0].c_str()));
			gt_timestamps->push_back(timestamp);
		}

		// quaternion
		CQuaternionDouble quat;
		quat.r(atof(curr_tokens[7].c_str()));
		quat.x(atof(curr_tokens[4].c_str()));
		quat.y(atof(curr_tokens[5].c_str()));
		quat.z(atof(curr_tokens[6].c_str()));
		quat.rpy(r, p, y);

		// pose
		CVectorDouble curr_coords(3);
		curr_coords[0] = atof(curr_tokens[1].c_str());
		curr_coords[1] = atof(curr_tokens[2].c_str());
		curr_coords[2] = atof(curr_tokens[3].c_str());

		// current ground-truth pose
		pose_t curr_pose(
				curr_coords[0],
				curr_coords[1],
				y);

		curr_pose.x() -= pose_diff.x();
		curr_pose.y() -= pose_diff.y();
		curr_pose.phi() -= pose_diff.phi();
		//curr_pose += -pose_diff;
		gt_poses->push_back(curr_pose);
	}

	file_GT.close();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::alignOpticalWithMRPTFrame() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	// aligning GT (optical) frame with the MRPT frame
	// Set the rotation matrix from the corresponding RPY angles
	// MRPT Frame: X->forward; Y->Left; Z->Upward
	// Optical Frame: X->Right; Y->Downward; Z->Forward
	ASSERT_(m_has_read_config);
	// rotz
	double anglez = DEG2RAD(0.0);
	const double tmpz[] = {
		cos(anglez),     -sin(anglez), 0,
		sin(anglez),     cos(anglez),  0,
		0,               0,            1  };
	CMatrixDouble rotz(3, 3, tmpz);

	// roty
	double angley = DEG2RAD(0.0);
	//double angley = DEG2RAD(90.0);
	const double tmpy[] = {
		cos(angley),      0,      sin(angley),
		0,                1,      0,
		-sin(angley),     0,      cos(angley)  };
	CMatrixDouble roty(3, 3, tmpy);

	// rotx
	//double anglex = DEG2RAD(-90.0);
	double anglex = DEG2RAD(0.0);
	const double tmpx[] = {
		1,        0,               0,
		0,        cos(anglex),     -sin(anglex),
		0,        sin(anglex),     cos(anglex)  };
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

	MRPT_LOG_DEBUG_STREAM << ss_out;

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::queryObserverForEvents() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	ASSERTMSG_(m_win_observer,
			"\nqueryObserverForEvents method was called even though no Observer object was provided\n");
	using namespace mrpt::utils;

	std::map<std::string, bool> events_occurred;
	m_win_observer->returnEventsStruct(&events_occurred);
	m_request_to_exit = events_occurred.find("Ctrl+c")->second;

	// odometry visualization
	if (events_occurred[m_keystroke_odometry]) {
		this->toggleOdometryVisualization();
	}
	// GT visualization
	if (events_occurred[m_keystroke_GT]) {
		this->toggleGTVisualization();
	}
	// Map visualization
	if (events_occurred[m_keystroke_map]) {
		this->toggleMapVisualization();
	}
	// Estimated Trajectory Visualization
	if (events_occurred[m_keystroke_estimated_trajectory]) {
		this->toggleEstimatedTrajectoryVisualization();
	}
	// pause/resume program execution
	if (events_occurred[m_keystroke_pause_exec]) {
		m_program_paused = !m_program_paused;

		// message to the user
		if (m_program_paused) {
			MRPT_LOG_INFO_STREAM << "Program is paused. "
				<< "Press \"p\" or \"P\" in the dipslay window to resume";
		}
		else {
			MRPT_LOG_INFO_STREAM << "Program resumed.";
		}

		while (m_program_paused) {
			mrpt::system::sleep(1000);
			this->queryObserverForEvents();
		}
	}

	// notify the deciders/optimizer of any events they may be interested in
	MRPT_LOG_DEBUG_STREAM << "Notifying deciders/optimizer for events";
	m_node_registrar->notifyOfWindowEvents(events_occurred);
	m_edge_registrar->notifyOfWindowEvents(events_occurred);
	m_optimizer->notifyOfWindowEvents(events_occurred);

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::toggleOdometryVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::utils;
	using namespace mrpt::opengl;
	MRPT_LOG_INFO_STREAM << "Toggling Odometry visualization...";

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (m_visualize_odometry_poses) {
		CRenderizablePtr obj = scene->getByName("odometry_poses_cloud");
		obj->setVisibility(!obj->isVisible());

		obj = scene->getByName("robot_odometry_poses");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		dumpVisibilityErrorMsg("visualize_odometry_poses");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::toggleGTVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	MRPT_LOG_INFO_STREAM << "Toggling Ground Truth visualization";

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (m_visualize_GT) {

		CRenderizablePtr obj = scene->getByName("GT_cloud");
		obj->setVisibility(!obj->isVisible());

		obj = scene->getByName("robot_GT");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		dumpVisibilityErrorMsg("visualize_ground_truth");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::toggleMapVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace std;
	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	MRPT_LOG_INFO_STREAM << "Toggling Map visualization... ";

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	// get total number of nodes
	int num_of_nodes;
	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		num_of_nodes = m_graph.nodeCount();
	}

	// name of gui object
	stringstream scan_name("");

	for (int node_cnt = 0; node_cnt  != num_of_nodes; ++node_cnt) {
		// build the name of the potential corresponding object in the scene
		scan_name.str("");
		scan_name << "laser_scan_";
		scan_name << node_cnt;

		CRenderizablePtr obj = scene->getByName(scan_name.str());
		// current node may not have laserScans => may not have corresponding obj
		if (obj) {
			obj->setVisibility(!obj->isVisible());
		}
	}
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::toggleEstimatedTrajectoryVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	MRPT_LOG_INFO_STREAM << "Toggling Estimated Trajectory visualization... ";

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (m_visualize_estimated_trajectory) {

		CRenderizablePtr obj = scene->getByName("estimated_traj_setoflines");
		obj->setVisibility(!obj->isVisible());

		obj = scene->getByName("robot_estimated_traj");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		dumpVisibilityErrorMsg("visualize_estimated_trajectory");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time /* = 500 milliseconds */) {
	MRPT_START;
	using namespace mrpt::utils;

	MRPT_LOG_ERROR_STREAM << "Cannot toggle visibility of specified object.\n"
			<< "Make sure that the corresponding visualization flag ("
			<< viz_flag
			<< ") is set to true in the .ini file.";
	mrpt::system::sleep(sleep_time);

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::updateMapVisualization(
		const GRAPH_t& gr,
		std::map<mrpt::utils::TNodeID,
		mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans2D,
		bool full_update /*= false */) {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace std;
	using namespace mrpt::poses;

	CTicTac map_update_timer;
	map_update_timer.Tic();

	// get set of nodes to run the update for
	std::set<mrpt::utils::TNodeID> nodes_set;
	{
		if (full_update) {
			// for all the nodes get the node position and the corresponding laser scan
			// if they were recorded and visualize them
			m_graph.getAllNodes(nodes_set);
			MRPT_LOG_INFO_STREAM << "Executing full update of the map";

		} // IF FULL UPDATE
		else { // add only current CSimplePointMap
			nodes_set.insert(m_nodeID_max);
		} // if PARTIAL_UPDATE
	}

	// for all the nodes in the previously populated set
	for (std::set<mrpt::utils::TNodeID>::const_iterator
			node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		pose_t scan_pose = m_graph.nodes.find(*node_it)->second;

		// name of gui object
		stringstream scan_name("");
		scan_name << "laser_scan_";
		scan_name << *node_it;

		// get the node laser scan
		CObservation2DRangeScanPtr scan_content;
		std::map<mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr>::const_iterator search =
				m_nodes_to_laser_scans2D.find(*node_it);

		// make sure that the laser scan exists and is valid
		if (search != m_nodes_to_laser_scans2D.end() && !(search->second.null())) {
			scan_content = search->second;

			CObservation2DRangeScan scan_decimated;
			this->decimateLaserScan(*scan_content, &scan_decimated,
					/*keep_every_n_entries = */ 5);

			// if the scan doesn't already exist, add it to the scene, otherwise just
			// adjust its pose
			COpenGLScenePtr scene = m_win->get3DSceneAndLock();
			CRenderizablePtr obj = scene->getByName(scan_name.str());

			CSetOfObjectsPtr scan_obj;
			if (obj.null()) {
				MRPT_LOG_DEBUG_STREAM << "CSetOfObjects for nodeID " << *node_it <<
					"doesn't exist. Creating it...";

				scan_obj = CSetOfObjects::Create();

				// creating and inserting the observation in the CSetOfObjects
				mrpt::maps::CSimplePointsMap m;
				m.insertObservation(&scan_decimated);
				m.getAs3DObject(scan_obj);

				scan_obj->setName(scan_name.str());
				scan_obj->setColor_u8(m_optimized_map_color);

				// set the visibility of the object the same value as the visibility of
				// the previous - Needed for proper toggling of the visibility of the
				// whole map
				{
					stringstream prev_scan_name("");
					prev_scan_name << "laser_scan_" << *node_it - 1;
					CRenderizablePtr prev_obj = scene->getByName(prev_scan_name.str());
					if (prev_obj) {
						scan_obj->setVisibility(prev_obj->isVisible());
					}
				}

				scene->insert(scan_obj);
			}
			else {
				scan_obj = static_cast<CSetOfObjectsPtr>(obj);
			}

			// finally set the pose correctly - as computed by graphSLAM
			scan_obj->setPose(CPose3D(scan_pose));

			m_win->unlockAccess3DScene();
			m_win->forceRepaint();
		}
		else {
			MRPT_LOG_DEBUG_STREAM << "Laser scans of NodeID " << *node_it <<
				"are  empty/invalid";
		}
	}


	double elapsed_time = map_update_timer.Tac();
	MRPT_LOG_DEBUG_STREAM << "updateMapVisualization took: "
		<< elapsed_time << "s";
	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::decimateLaserScan(
		mrpt::obs::CObservation2DRangeScan& laser_scan_in,
		mrpt::obs::CObservation2DRangeScan* laser_scan_out,
		const int keep_every_n_entries /*= 2*/) {
	MRPT_START;
	using namespace mrpt::utils;

	size_t scan_size = laser_scan_in.scan.size();

	// assign the decimated scans, ranges
	std::vector<float> new_scan(scan_size); // Was [], but can't use non-constant argument with arrays
	std::vector<char> new_validRange(scan_size);
	size_t new_scan_size = 0;
	for (size_t i=0; i != scan_size; i++) {
		if (i % keep_every_n_entries == 0) {
			new_scan[new_scan_size] = laser_scan_in.scan[i];
			new_validRange[new_scan_size] = laser_scan_in.validRange[i];
			new_scan_size++;
		}
	}
	laser_scan_out->loadFromVectors(new_scan_size, &new_scan[0], &new_validRange[0]);

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initGTVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::utils;
	using namespace mrpt::opengl;

	// assertions
	ASSERT_(m_has_read_config);
	ASSERT_(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was provided");

	// point cloud
	CPointCloudPtr GT_cloud = CPointCloud::Create();
	GT_cloud->setPointSize(1.0);
	GT_cloud->enablePointSmooth();
	GT_cloud->enableColorFromX(false);
	GT_cloud->enableColorFromY(false);
	GT_cloud->enableColorFromZ(false);
	GT_cloud->setColor_u8(m_GT_color);
	GT_cloud->setName("GT_cloud");

	// robot model
	CSetOfObjectsPtr robot_model = stock_objects::RobotPioneer();
	pose_t initial_pose;
	robot_model->setPose(initial_pose);
	robot_model->setName("robot_GT");
	robot_model->setColor_u8(m_GT_color);
	robot_model->setScale(m_robot_model_size);

	// insert them to the scene
	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	scene->insert(GT_cloud);
	scene->insert(robot_model);
	m_win->unlockAccess3DScene();

	m_win_manager->assignTextMessageParameters(
			/* offset_y*		= */ &m_offset_y_GT,
			/* text_index* = */ &m_text_index_GT);
	m_win_manager->addTextMessage(5,-m_offset_y_GT,
			mrpt::format("Ground truth path"),
			TColorf(m_GT_color),
			/* unique_index = */ m_text_index_GT );

	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::updateGTVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;

	// add to the GT PointCloud and visualize it
	// check that GT vector is not depleted
	if (m_visualize_GT &&
			m_GT_poses_index < m_GT_poses.size()) {
		ASSERT_(m_win &&
				"Visualization of data was requested but no CDisplayWindow3D pointer was given");

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("GT_cloud");
		CPointCloudPtr GT_cloud = static_cast<CPointCloudPtr>(obj);

		// add the latest GT pose
		pose_t gt_pose = m_GT_poses[m_GT_poses_index];
		GT_cloud->insertPoint(
				gt_pose.x(),
				gt_pose.y(),
				0 );

		// robot model of GT trajectory
		obj = scene->getByName("robot_GT");
		CSetOfObjectsPtr robot_obj = static_cast<CSetOfObjectsPtr>(obj);
		robot_obj->setPose(gt_pose);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initOdometryVisualization() {
	MRPT_START;
	ASSERT_(m_has_read_config);
	ASSERT_(m_enable_visuals);
	using namespace mrpt::utils;
	using namespace mrpt::opengl;

	// point cloud
	CPointCloudPtr odometry_poses_cloud = CPointCloud::Create();
	odometry_poses_cloud->setPointSize(1.0);
	odometry_poses_cloud->enablePointSmooth();
	odometry_poses_cloud->enableColorFromX(false);
	odometry_poses_cloud->enableColorFromY(false);
	odometry_poses_cloud->enableColorFromZ(false);
	odometry_poses_cloud->setColor_u8(m_odometry_color);
	odometry_poses_cloud->setName("odometry_poses_cloud");

	// robot model
	CSetOfObjectsPtr robot_model = stock_objects::RobotPioneer();
	pose_t initial_pose;
	robot_model->setPose(initial_pose);
	robot_model->setName("robot_odometry_poses");
	robot_model->setColor_u8(m_odometry_color);
	robot_model->setScale(m_robot_model_size);

	// insert them to the scene
	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	scene->insert(odometry_poses_cloud);
	scene->insert(robot_model);
	m_win->unlockAccess3DScene();

	m_win_manager->assignTextMessageParameters(
			/* offset_y* = */ &m_offset_y_odometry,
			/* text_index* = */ &m_text_index_odometry);
	m_win_manager->addTextMessage(5,-m_offset_y_odometry,
			mrpt::format("Odometry path"),
			TColorf(m_odometry_color),
			/* unique_index = */ m_text_index_odometry );

	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::updateOdometryVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	ASSERTMSG_(m_win,
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");
	using namespace mrpt::opengl;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	// point cloud
	CRenderizablePtr obj = scene->getByName("odometry_poses_cloud");
	CPointCloudPtr odometry_poses_cloud = static_cast<CPointCloudPtr>(obj);
	pose_t* odometry_pose = m_odometry_poses.back();

	odometry_poses_cloud->insertPoint(
			odometry_pose->x(),
			odometry_pose->y(),
			0 );

	// robot model
	obj = scene->getByName("robot_odometry_poses");
	CSetOfObjectsPtr robot_obj = static_cast<CSetOfObjectsPtr>(obj);
	robot_obj->setPose(*odometry_pose);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initEstimatedTrajectoryVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;
	using namespace mrpt::utils;

	// SetOfLines
	CSetOfLinesPtr estimated_traj_setoflines = CSetOfLines::Create();
	estimated_traj_setoflines->setColor_u8(m_estimated_traj_color);
	estimated_traj_setoflines->setLineWidth(0.5);
	estimated_traj_setoflines->setName("estimated_traj_setoflines");
	// append a dummy line so that you can later use append using
	// CSetOfLines::appendLienStrip method.
	estimated_traj_setoflines->appendLine(
			/* 1st */ 0, 0, 0,
			/* 2nd */ 0, 0, 0);

	// robot model
	CSetOfObjectsPtr robot_model = stock_objects::RobotPioneer();
	pose_t initial_pose;
	robot_model->setPose(initial_pose);
	robot_model->setName("robot_estimated_traj");
	robot_model->setColor_u8(m_estimated_traj_color);
	robot_model->setScale(m_robot_model_size);

	// insert objects in the graph
	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	if (m_visualize_estimated_trajectory) {
		scene->insert(estimated_traj_setoflines);
	}
	scene->insert(robot_model);
	m_win->unlockAccess3DScene();

	if (m_visualize_estimated_trajectory) {
		m_win_manager->assignTextMessageParameters( /* offset_y* = */ &m_offset_y_estimated_traj,
				/* text_index* = */ &m_text_index_estimated_traj);
		m_win_manager->addTextMessage(5,-m_offset_y_estimated_traj,
				mrpt::format("Estimated trajectory"),
				TColorf(m_estimated_traj_color),
				/* unique_index = */ m_text_index_estimated_traj );
	}


	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::
updateEstimatedTrajectoryVisualization(bool full_update) {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	using namespace mrpt::opengl;

	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
	ASSERT_(m_graph.nodeCount() != 0);

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	CRenderizablePtr obj;
	if (m_visualize_estimated_trajectory) {
		// set of lines
		obj = scene->getByName("estimated_traj_setoflines");
		CSetOfLinesPtr estimated_traj_setoflines = static_cast<CSetOfLinesPtr>(obj);

		// gather set of nodes for which to append lines - all of the nodes in the
		// graph or just the last inserted..
		std::set<mrpt::utils::TNodeID> nodes_set;
		{
			if (full_update) {
				m_graph.getAllNodes(nodes_set);
				estimated_traj_setoflines->clear();
				estimated_traj_setoflines->appendLine(
						/* 1st */ 0, 0, 0,
						/* 2nd */ 0, 0, 0);
			}
			else {
				nodes_set.insert(m_graph.nodeCount()-1);
			}

		}
		// append line for each node in the set
		for (std::set<mrpt::utils::TNodeID>::const_iterator
				nodeID_it = nodes_set.begin();
				nodeID_it != nodes_set.end(); ++nodeID_it) {

			estimated_traj_setoflines->appendLineStrip(
					m_graph.nodes[*nodeID_it].x(),
					m_graph.nodes[*nodeID_it].y(),
					0.05f);
		}
	}

	// robot model
	// set the robot position to the last recorded pose in the graph
	obj = scene->getByName("robot_estimated_traj");
	CSetOfObjectsPtr robot_obj = static_cast<CSetOfObjectsPtr>(obj);
	pose_t curr_estimated_pos = m_graph.nodes[m_graph.nodeCount()-1];
	robot_obj->setPose(curr_estimated_pos);


	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	MRPT_END;
}

// TRGBDInfoFileParams
// ////////////////////////////////
template<class GRAPH_t>
CGraphSlamEngine<GRAPH_t>::
TRGBDInfoFileParams::TRGBDInfoFileParams(const std::string& rawlog_fname) {

	this->setRawlogFile(rawlog_fname);
	this->initTRGBDInfoFileParams();
}
template<class GRAPH_t>
CGraphSlamEngine<GRAPH_t>::
TRGBDInfoFileParams::TRGBDInfoFileParams() {
	this->initTRGBDInfoFileParams();
}
template<class GRAPH_t>
CGraphSlamEngine<GRAPH_t>::
TRGBDInfoFileParams::~TRGBDInfoFileParams() { }

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::TRGBDInfoFileParams::setRawlogFile(
		const std::string& rawlog_fname) {

	// get the correct info filename from the rawlog_fname
	std::string dir = mrpt::system::extractFileDirectory(rawlog_fname);
	std::string rawlog_filename = mrpt::system::extractFileName(rawlog_fname);
	std::string name_prefix = "rawlog_";
	std::string name_suffix = "_info.txt";
	info_fname = dir + name_prefix + rawlog_filename + name_suffix;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::
TRGBDInfoFileParams::initTRGBDInfoFileParams() {
	// fields to use
	fields["Overall number of objects"] = "";
	fields["Observations format"] = "";
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::TRGBDInfoFileParams::parseFile() {
	ASSERT_FILE_EXISTS_(info_fname);
	using namespace std;
	using namespace mrpt::utils;

	// open file
	CFileInputStream info_file(info_fname);
	ASSERTMSG_(info_file.fileOpenCorrectly(),
			"\nTRGBDInfoFileParams::parseFile: Couldn't open info file\n");

	string curr_line;
	size_t line_cnt = 0;

	// parse until you find an empty line.
	while (true) {
		info_file.readLine(curr_line);
		line_cnt++;
		if (curr_line.size() == 0)
			break;
	}

	// parse the meaningful data
	while (info_file.readLine(curr_line)) {
		// split current line at ":"
		vector<string> curr_tokens;
		mrpt::system::tokenize(curr_line, ":", curr_tokens);

		ASSERT_EQUAL_(curr_tokens.size(), 2);

		// evaluate the name. if name in info struct then fill the corresponding
		// info struct parameter with the value_part in the file.
		std::string literal_part = mrpt::system::trim(curr_tokens[0]);
		std::string value_part   = mrpt::system::trim(curr_tokens[1]);

		for (std::map<std::string, std::string>::iterator it = fields.begin();
				it != fields.end(); ++it) {
			if (mrpt::system::strCmpI(it->first, literal_part)) {
				it->second = value_part;
			}
		}

		line_cnt++;
	}

}
////////////////////////////////////////////////////////////////////////////////

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::saveGraph(
		const std::string* fname_in /*= NULL */) const {
	MRPT_START;
	using namespace mrpt::utils;

	// what's the name of the file to be saved?
	std::string fname;
	if (fname_in) {
		fname = *fname_in;
	}
	else {
		fname = "output_graph.graph";
	}

	MRPT_LOG_INFO_STREAM << "Saving generated graph in VERTEX/EDGE format: " << fname;
	m_graph.saveToTextFile(fname);

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::save3DScene(
		const std::string* fname_in /* = NULL */) const {
	MRPT_START;
	ASSERTMSG_(m_enable_visuals,
			"\nsave3DScene was called even though enable_visuals flag is off.\nExiting...\n");
	using namespace mrpt::opengl;
	using namespace mrpt::utils;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	// TODO - what else is there to be excluded from the scene?
	// remove the CPlanarLaserScan if it exists
	{
		CPlanarLaserScanPtr laser_scan;
		for (; (laser_scan = scene->getByClass<CPlanarLaserScan>()) ;) {
			MRPT_LOG_DEBUG_STREAM << "Removing CPlanarLaserScan from generated 3DScene...";
			scene->removeObject(laser_scan);
		}
	}

	// what's the name of the file to be saved?
	std::string fname;
	if (fname_in) {
		fname = *fname_in;
	}
	else {
		fname = "output_scene.3DScene";
	}

	MRPT_LOG_INFO_STREAM << "Saving generated scene to " << fname;
	scene->saveToFile(fname);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::computeSlamMetric(mrpt::utils::TNodeID nodeID, size_t gt_index) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	// start updating the metric after a certain number of nodes have been added
	if ( m_graph.nodeCount() < 4 ) {
		return;
	}

	// add to the map - keep track of which gt index corresponds to which nodeID
	m_nodeID_to_gt_indices[nodeID] = gt_index;

	// initialize the loop variables only once
	pose_t curr_node_pos;
	pose_t curr_gt_pos;
	pose_t node_delta;
	pose_t gt_delta;
	double trans_diff;
	double rot_diff;

	size_t indices_size = m_nodeID_to_gt_indices.size();

	// recompute the metric from scratch
	m_curr_deformation_energy = 0;

	// first element of map
	//std::map<mrpt::utils::TNodeID, size_t>::const_iterator start_it =
		//std::next(m_nodeID_to_gt_indices.begin(), 1);
	std::map<mrpt::utils::TNodeID, size_t>::const_iterator start_it =
		m_nodeID_to_gt_indices.begin();
	start_it++;


	// fetch the first node, gt positions separately
	//std::map<mrpt::utils::TNodeID, size_t>::const_iterator prev_it = std::prev(start_it, 1);
	std::map<mrpt::utils::TNodeID, size_t>::const_iterator prev_it = start_it;
	prev_it--;
	pose_t prev_node_pos = m_graph.nodes[prev_it->first];
	pose_t prev_gt_pos = m_GT_poses[prev_it->second];

	for (std::map<mrpt::utils::TNodeID, size_t>::const_iterator
			index_it = start_it;
			index_it != m_nodeID_to_gt_indices.end();
			index_it++) {
		curr_node_pos = m_graph.nodes[index_it->first];
		curr_gt_pos = m_GT_poses[index_it->second];

		node_delta = curr_node_pos - prev_node_pos;
		gt_delta = curr_gt_pos - prev_gt_pos;

		trans_diff = gt_delta.distanceTo(node_delta);
		rot_diff = wrapToPi(gt_delta.phi() - node_delta.phi());

		m_curr_deformation_energy += (pow(trans_diff, 2) + pow(rot_diff, 2));
		m_curr_deformation_energy /= indices_size;

		// add it to the overall vector
		m_deformation_energy_vec.push_back(m_curr_deformation_energy);

		prev_node_pos = curr_node_pos;
		prev_gt_pos = curr_gt_pos;
	}

	MRPT_LOG_DEBUG_STREAM << "Total deformation energy: "
		<< m_curr_deformation_energy;

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::initSlamMetricVisualization() {
	ASSERT_(m_enable_visuals);
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::gui;

	MRPT_START;
	MRPT_LOG_DEBUG_STREAM << "In initializeSlamMetricVisualization...";
	ASSERT_(m_visualize_SLAM_metric);

	// initialize the m_win_plot on the stack
	m_win_plot = new CDisplayWindowPlots("Evolution of SLAM metric - Deformation Energy (1:1000)", 400, 300);


	m_win_plot->setPos(20, 50);
	m_win_plot->clf();
	// just plot the deformation points from scratch every time
	m_win_plot->hold_off();
	m_win_plot->enableMousePanZoom(true);

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::updateSlamMetricVisualization() {
	MRPT_START;
	ASSERT_(m_enable_visuals);
	ASSERT_(m_win_plot && m_visualize_SLAM_metric);

	// build the X, Y vectors for plotting - use log scale
	std::vector<double> x(m_deformation_energy_vec.size(), 0);
	std::vector<double> y(m_deformation_energy_vec.size(), 0);
	for (size_t i = 0; i != x.size(); i++)  {
		x[i] = i;
		y[i] = m_deformation_energy_vec[i]*1000;
	}

	m_win_plot->plot(x, y, "r-1",
			/*plotName = */ "Deformation Energy (x1000)");

	// set the limits so that he y-values can be monitored
	// set the xmin limit with respect to xmax, which is constantly growing
	std::vector<double>::const_iterator xmax, ymax;
	xmax = std::max_element(x.begin(), x.end());
	ymax = std::max_element(y.begin(), y.end());

	m_win_plot->axis(/*x_min = */ xmax != x.end()? -(*xmax/12) : -1,
			/*x_max = */ (xmax != x.end()? *xmax : 1),
			/*y_min = */ -0.4f,
			/*y_max = */ (ymax != y.end()? *ymax : 1) ) ;


	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;
	using namespace std;
	using namespace mrpt::utils;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Summary of Results
	stringstream results_ss;
	results_ss << "Summary: " << std::endl;
	results_ss << header_sep << std::endl;
	results_ss << "\tProcessing time: " <<
		m_time_logger.getMeanTime("proc_time") << std::endl;;
	results_ss << "\tDataset Grab time: " << m_dataset_grab_time << std::endl;
	results_ss << "\tReal-time capable: " <<
		(m_time_logger.getMeanTime("proc_time") < m_dataset_grab_time ?
		 "TRUE": "FALSE") << std::endl;
	results_ss << m_edge_counter.getAsString();
	results_ss << "\tNum of Nodes: " << m_graph.nodeCount() << std::endl;;

	// Class configuration parameters
	std::string config_params = this->getParamsAsString();

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += results_ss.str();
	*report_str += report_sep;

	*report_str += config_params;
	*report_str += report_sep;

	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}

template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::generateReportFiles(
		const std::string& output_dir_fname) {
	MRPT_START;
	using namespace mrpt::utils;
	// first initialize the directory where the files are to be placed
	this->initOutputDir(output_dir_fname);

	MRPT_LOG_INFO_STREAM << "Generating detailed class report...";
	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

	std::string report_str;
	std::string fname;
	const std::string ext = ".log";

	{ // CGraphSlamEngine
		report_str.clear();
		fname = output_dir_fname + "/" + m_class_name + ext;
		// initialize the output file - refer to the stream through the
		// m_out_streams map
		this->initResultsFile(fname);

		// write the actual content
		this->getDescriptiveReport(&report_str);
		m_out_streams[fname]->printf("%s", report_str.c_str());
	}
	{ // node_registrar
		report_str.clear();
		fname = output_dir_fname + "/" + "node_registrar" + ext;
		this->initResultsFile(fname);
		m_node_registrar->getDescriptiveReport(&report_str);
		m_out_streams[fname]->printf("%s", report_str.c_str());
	}
	{ // edge_registrar
		report_str.clear();
		fname = output_dir_fname + "/" + "edge_registrar" + ext;
		this->initResultsFile(fname);
		m_edge_registrar->getDescriptiveReport(&report_str);
		m_out_streams[fname]->printf("%s", report_str.c_str());
	}
	{ // optimizer
		report_str.clear();
		fname = output_dir_fname + "/" + "optimizer" + ext;
		this->initResultsFile(fname);
		m_optimizer->getDescriptiveReport(&report_str);
		m_out_streams[fname]->printf("%s", report_str.c_str());
	}

	if (m_use_GT) { // slam evaluation metric
		report_str.clear();
		const std::string desc("# File includes the evolution of the SLAM metric.  Implemented metric computes the \"deformation energy\" that is needed to transfer the estimated trajectory into the ground-truth trajectory (computed as sum of the difference between estimated trajectory and ground truth consecutive poses See \"A comparison of SLAM algorithms based on a graph of relations - W.Burgard et al.\", for more details on the metric.\n");

		fname = output_dir_fname + "/" + "SLAM_evaluation_metric" + ext;
		this->initResultsFile(fname);

		m_out_streams[fname]->printf("%s\n", desc.c_str());
		for (std::vector<double>::const_iterator vec_it = m_deformation_energy_vec.begin();
				vec_it != m_deformation_energy_vec.end(); ++vec_it) {
			m_out_streams[fname]->printf("%f\n", *vec_it);
		}
	}

	MRPT_END;
}


// TODO - check this
template<class GRAPH_t>
void CGraphSlamEngine<GRAPH_t>::getDeformationEnergyVector(
		std::vector<double>* vec_out) const {
	MRPT_START;

	ASSERT_(vec_out);
	*vec_out = m_deformation_energy_vec;

	MRPT_END;
}

} } // end of namespaces

#endif /* end of include guard: CGRAPHSLAMENGINE_IMPL_H */
