/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAMENGINE_IMPL_H
#define CGRAPHSLAMENGINE_IMPL_H

using namespace mrpt::graphslam;

// todo - remove these
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::graphslam;

using namespace std;

// Ctors, Dtors implementations
//////////////////////////////////////////////////////////////

template< class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::CGraphSlamEngine_t(
		const string& config_file,
		CDisplayWindow3D* win /* = NULL */,
		CWindowObserver* win_observer /* = NULL */,
		std::string rawlog_fname /* = "" */,
		std::string fname_GT /* = "" */ ):
	m_config_fname(config_file),
	m_rawlog_fname(rawlog_fname),
	m_fname_GT(fname_GT),
	m_GT_poses_step(1),
	m_win(win),
	m_win_observer(win_observer),
	m_win_manager(m_win, m_win_observer), // pass window and observer in Ctor
	m_odometry_color(0, 0, 255),
	m_GT_color(0, 255, 0),
	m_estimated_traj_color(255, 165, 0),
	m_robot_model_size(3),
	m_graph_section("graph_sec")
{

	this->initCGraphSlamEngine();
};

template< class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::~CGraphSlamEngine_t() {
	MRPT_START;

	VERBOSE_COUT << "In Destructor: Deleting CGraphSlamEngine_t instance..."
		<< std::endl;

	// close all open files
	for (fstreams_out_it it  = m_out_streams.begin(); it != m_out_streams.end();
			++it) {
		if ((it->second)->fileOpenCorrectly()) {
			VERBOSE_COUT << "Closing file: " << (it->first).c_str() << std::endl;
			(it->second)->close();
		}
	}

	// delete m_odometry_poses
	if (m_odometry_poses.size())
	VERBOSE_COUT << "Releasing m_odometry_poses vector" << std::endl;
	for (int i = 0; i < m_odometry_poses.size(); ++i) {
		delete m_odometry_poses[i];
	}

	MRPT_END;
}


// Member functions implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initCGraphSlamEngine() {
	MRPT_START;

	/**
	 * Parameters validation
	 */
	ASSERT_(!(!m_win && m_win_observer) &&
			"CObsever was provided even though no CDisplayWindow3D was not");

	/**
	 * Initialization of various member variables
	 */

	// check for duplicated edges every..
	m_num_of_edges_for_collapse = 100;

	m_is3D = constraint_t::is_3D_val;
	m_observation_only_rawlog = false;

	// max node number already in the graph
	m_nodeID_max = 0;
	m_graph.root = TNodeID(0);

	// pass a graph ptr after the instance initialization
	m_node_registrar.setGraphPtr(&m_graph);
	m_edge_registrar.setGraphPtr(&m_graph);
	m_optimizer.setGraphPtr(&m_graph);

	// pass the window manager ptr after the instance initialization.
	// m_win_manager contains a pointer to the CDisplayWindow3D instance
	m_node_registrar.setWindowManagerPtr(&m_win_manager);
	m_edge_registrar.setWindowManagerPtr(&m_win_manager);
	m_optimizer.setWindowManagerPtr(&m_win_manager);

	// pass a lock in case of multithreaded implementation
	m_node_registrar.setCriticalSectionPtr(&m_graph_section);
	m_edge_registrar.setCriticalSectionPtr(&m_graph_section);
	m_optimizer.setCriticalSectionPtr(&m_graph_section);

	// Calling of initalization-relevant functions
	this->readConfigFile(m_config_fname);

	this->initOutputDir();
	this->printProblemParams();
	//mrpt::system::pause();

	// pass the rawlog filename after the instance initialization
	m_node_registrar.setRawlogFname(m_rawlog_fname);
	m_edge_registrar.setRawlogFname(m_rawlog_fname);
	m_optimizer.setRawlogFname(m_rawlog_fname);


	/**
	 * Visualization-related parameters initialization
	 */

	if (!m_win) {
		m_visualize_odometry_poses = 0;
		m_visualize_GT = 0;
		m_visualize_map = 0;
		m_enable_curr_pos_viewport = 0;
		m_enable_range_viewport = 0;
		m_enable_intensity_viewport = 0;
	}

	// timestamp
	m_win_manager.assignTextMessageParameters(&m_offset_y_timestamp,
			&m_text_index_timestamp);


	// aligning GT (optical) frame with the MRPT frame
	// Set the rotation matrix from the corresponding RPY angles
	// MRPT Frame: X->forward; Y->Left; Z->Upward
	// Optical Frame: X->Right; Y->Downward; Z->Forward
	ASSERT_(m_has_read_config);
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum") && m_visualize_GT) {
		// rotz
		double anglez = DEG2RAD(0);
		const double tmpz[] = {
			cos(anglez),     -sin(anglez), 0,
			sin(anglez),     cos(anglez),  0,
			0,               0,            1  };
		CMatrixDouble rotz(3, 3, tmpz);

		// roty
		//double angley = DEG2RAD(-90);
		double angley = DEG2RAD(0);
		const double tmpy[] = {
			cos(angley),      0,      sin(angley),
			0,                1,      0,
			-sin(angley),     0,      cos(angley)  };
		CMatrixDouble roty(3, 3, tmpy);

		// rotx
		//double anglex = DEG2RAD(-90);
		double anglex = DEG2RAD(0);
		const double tmpx[] = {
			1,        0,               0,
			0,        cos(anglex),     -sin(anglex),
			0,        sin(anglex),     cos(anglex)  };
		CMatrixDouble rotx(3, 3, tmpx);

		cout << "Constructing the rotation matrix for the GroundTruth Data..." 
			<< endl;
		m_rot_TUM_to_MRPT = rotz * roty * rotx;

		cout << "Rotation matrices for optical=>MRPT transformation" << endl;
		cout << "rotz: " << endl << rotz << endl;
		cout << "roty: " << endl << roty << endl;
		cout << "rotx: " << endl << rotx << endl;
		cout << "Full rotation matrix: " << endl << m_rot_TUM_to_MRPT << endl;
	}

	// Configuration of various trajectories visualization
	ASSERT_(m_has_read_config);
	if (m_win) {
		// odometry visualization
		if (m_visualize_odometry_poses) {
			this->initOdometryVisualization();
		}
		// GT Visualization
		if (m_visualize_GT) {
			if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
				this->readGTFileRGBD_TUM(m_fname_GT, &m_GT_poses);
			}
			else if (mrpt::system::strCmpI(m_GT_file_format, "navsimul")) {
				this->readGTFileNavSimulOutput(m_fname_GT, &m_GT_poses);
			}

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
		m_img_prev_path_base = CImage::IMAGES_PATH_BASE;

		std::string rawlog_fname_noext = system::extractFileName(m_rawlog_fname);
		std::string rawlog_dir = system::extractFileDirectory(m_rawlog_fname);
		std::string m_img_external_storage_dir = rawlog_dir + rawlog_fname_noext + "_Images/";
		CImage::IMAGES_PATH_BASE = m_img_external_storage_dir;
	}

	// 3DRangeScans viewports initialization
	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
		if (m_enable_range_viewport) {
			this->initRangeImageViewport();
		}
		if (m_enable_intensity_viewport) {
			this->initIntensityImageViewport();
		}
	}
	// axis
	if (m_win) {
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

	// keystrokes initialization
	m_keystroke_pause_exec = "p";
	m_keystroke_odometry = "o";
	m_keystroke_GT = "g";
	m_keystroke_estimated_trajectory = "t";
	m_keystroke_map = "m";

	// Add additional keystrokes in the CDisplayWindow3D message box
	if (m_win_observer) { 
		m_win_observer->registerKeystroke(m_keystroke_pause_exec,
				"Pause program execution");
		m_win_observer->registerKeystroke(m_keystroke_odometry,
				"Toggle Odometry visualization");
		m_win_observer->registerKeystroke(m_keystroke_GT,
				"Toggle Ground-Truth visualization");
		m_win_observer->registerKeystroke(m_keystroke_estimated_trajectory,
				"Toggle Estimated trajectory visualization");
		m_win_observer->registerKeystroke(m_keystroke_map,
				"Toggle Map visualization");
	}

	if (m_win) {
		m_edge_counter.setVisualizationWindow(m_win);
	}

	// register the types of edges that are going to be displayed in the
	// visualization window
	{
		// total edges / loop closures
		double offset_y_total_edges, offset_y_loop_closures;
		int text_index_total_edges, text_index_loop_closures;

		m_win_manager.assignTextMessageParameters(&offset_y_total_edges,
				&text_index_total_edges);

		// register all the edge types
		vector<string> vec_edge_types;
		vec_edge_types.push_back("Odometry");m_edge_counter.addEdgeType("Odometry");
		vec_edge_types.push_back("ICP2D"); m_edge_counter.addEdgeType("ICP2D");
		vec_edge_types.push_back("ICP3D"); m_edge_counter.addEdgeType("ICP3D");

		// build each one of these
		map<string, double> name_to_offset_y;
		map<string, int> name_to_text_index;
		for (vector<string>::const_iterator it = vec_edge_types.begin();
				it != vec_edge_types.end();
				++it) {
			m_win_manager.assignTextMessageParameters(&name_to_offset_y[*it],
					&name_to_text_index[*it]);
		}

		m_win_manager.assignTextMessageParameters(&offset_y_loop_closures,
				&text_index_loop_closures);

		if (m_win) {
			// add all the parameters to the CEdgeCounter_t object
			m_edge_counter.setTextMessageParams(name_to_offset_y, name_to_text_index,
					offset_y_total_edges, text_index_total_edges,
					offset_y_loop_closures, text_index_loop_closures);
			m_edge_counter.setWindowManagerPtr(&m_win_manager);
		}
	}

	// query node/edge deciders for visual objects initialization
	if (m_win) {
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		m_node_registrar.initializeVisuals();
		m_edge_registrar.initializeVisuals();
		m_optimizer.initializeVisuals();
	}

	m_autozoom_active = true;
	m_request_to_exit = false;
	m_GT_poses_index = 0;




	// In case we are given an RGBD TUM Dataset - try and read the info file so
	// that we know how to play back the GT poses.
	try {
		m_info_params.setRawlogFile(m_rawlog_fname);
		m_info_params.parseFile();
		// set the rate at which we read from the GT poses vector
		int num_of_objects = std::atoi(
				m_info_params.fields["Overall number of objects"].c_str());
		m_GT_poses_step = m_GT_poses.size() / num_of_objects;

		VERBOSE_COUT << "Overall number of objects in rawlog: " << num_of_objects << endl;
		VERBOSE_COUT << "Setting the Ground truth read step to: " << m_GT_poses_step << endl;
	}
	catch (std::exception& e) {
		VERBOSE_COUT << "RGBD_TUM info file was not found." << endl;
	}

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
bool CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::parseRawlogFile() {
	MRPT_START;

	if (!m_has_read_config)
		THROW_EXCEPTION(
				std::endl
				<< "Config file has not been provided yet.\n" << "Exiting..."
				<< std::endl);
	if (!mrpt::system::fileExists(m_rawlog_fname))
		THROW_EXCEPTION(
				std::endl
				<< "parseRawlogFile: Rawlog file "
				<< m_rawlog_fname << " was not found."
				<< std::endl);
 	// good to go..

	/**
	 * Variables initialization
	 */

	CFileGZInputStream rawlog_file(m_rawlog_fname);
	CActionCollectionPtr action;
	CSensoryFramePtr observations;
	CObservationPtr observation;
	size_t curr_rawlog_entry = 0;

	{
		CRawlog::getActionObservationPairOrObservation(
				rawlog_file,
				action,	// Possible out var: Action of a a pair action / obs
				observations,	// Possible out var: obs's of a pair actin		 / obs
				observation, // Possible out var
				curr_rawlog_entry );
		if (observation.present()) {
			m_observation_only_rawlog = true; // false by default
		}
		else {
			m_observation_only_rawlog = false;
		}
	}

	// current timestamp - to be filled depending on the format
	TTimeStamp timestamp;
	pose_t curr_odometry_only_pose; // defaults to all 0s

	// Read the rest of the rawlog file
	while (CRawlog::getActionObservationPairOrObservation(
				rawlog_file,
				action,	// Possible out var: Action of a a pair action / obs
				observations,	// Possible out var: obs's of a pair actin		 / obs
				observation, // Possible out var
				curr_rawlog_entry ) ) {

		// node registration procedure
		bool registered_new_node;
		{
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
			//cout << "m_graph.nodeCount(): " << m_graph.nodeCount() << endl;
			registered_new_node = m_node_registrar.updateDeciderState(
					action, observations, observation);
			if (registered_new_node) {
				m_edge_counter.addEdge("Odometry");
				m_nodeID_max++;
			}
		}
		// Edge registration procedure
		// run this so that the decider can be updated with the latest
		// obsesrvations even when no new nodes have been added to the graph
		{
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

			m_edge_registrar.updateDeciderState(
					action,
					observations,
					observation );

			m_optimizer.updateOptimizerState(
					action,
					observations,
					observation );
		}
		std::cout << "After updating state of edge decider, optimizer... " 
			<< std::endl;

		if (observation.present()) {
			// Read a single observation from the rawlog
			// (Format #2 rawlog file)

			timestamp = observation->timestamp;

			// odometry
			if (IS_CLASS(observation, CObservationOdometry)) {
				CObservationOdometryPtr obs_odometry =
					static_cast<CObservationOdometryPtr>(observation);

				curr_odometry_only_pose = obs_odometry->odometry;
				// add to the odometry vector
				{
					pose_t* odometry_pose = new pose_t;
					*odometry_pose = curr_odometry_only_pose;
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

			// Camera images
			else if (IS_CLASS(observation, CObservationImage)) {
				//std::cout << "Processing Camera Image. " << std::endl;

			}
		}
		else {
			// action, observations should contain a pair of valid data
			// (Format #1 rawlog file)

			// parse the current action
			CActionRobotMovement2DPtr robot_move =
				action->getBestMovementEstimation();
			CPosePDFPtr increment = robot_move->poseChange;
			pose_t increment_pose = increment->getMeanVal();
			curr_odometry_only_pose += increment_pose;

			timestamp = robot_move->timestamp;

			// add to the odometry vector
			{
				pose_t* odometry_pose = new pose_t;
				*odometry_pose = curr_odometry_only_pose;
				m_odometry_poses.push_back(odometry_pose);
			}

			// get the last laser scan
			m_last_laser_scan2D =
				observations->getObservationByClass<CObservation2DRangeScan>();

		} // ELSE FORMAT #1 - Action/Observations

		if (registered_new_node) {
			// keep track of the laser scans so that I can later visualize the map
			m_nodes_to_laser_scans2D[m_nodeID_max] = m_last_laser_scan2D;
			m_nodes_to_laser_scans3D[m_nodeID_max] = m_last_laser_scan3D;

			if (m_win && m_visualize_map) {
				mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
				bool full_update = m_edge_registrar.justInsertedLoopClosure();
				this->updateMapVisualization(m_graph, m_nodes_to_laser_scans2D, full_update);
			}


			// query node/edge deciders for visual objects update
			if (m_win) {
				mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
				m_node_registrar.updateVisuals();
				m_edge_registrar.updateVisuals();
				m_optimizer.updateVisuals();
			}

			// update the edge counter
			std::map<const std::string, int> edge_types_to_nums;
			m_edge_registrar.getEdgesStats(&edge_types_to_nums);
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

			// update the global position of the nodes
			{
				mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
				m_graph.dijkstra_nodes_estimate();
			}

			// update visualization of estimated trajectory
			if (m_win) {
				bool full_update = true;
				this->updateEstimatedTrajectoryVisualization(full_update);
			}

		} // IF REGISTERED_NEW_NODE

		// Timestamp textMessage
		// Use the dataset timestamp otherwise fallback to mrpt::system::now()
		if (m_win) {
			if (timestamp != INVALID_TIMESTAMP) {
				m_win_manager.addTextMessage(5,-m_offset_y_timestamp,
						format("Simulated time: %s", timeLocalToString(timestamp).c_str()),
						TColorf(1.0, 1.0, 1.0),
						/* unique_index = */ m_text_index_timestamp );
			}
			else {
				m_win_manager.addTextMessage(5,-m_offset_y_timestamp,
						format("Wall time: %s", timeLocalToString(system::now()).c_str()),
						TColorf(1.0, 1.0, 1.0),
						/* unique_index = */ m_text_index_timestamp );
			}
		}

		// Odometry visualization
		if (m_visualize_odometry_poses && m_odometry_poses.size()) {
			this->updateOdometryVisualization();
		}

		// ensure that the GT is visualized at the same rate as the SLAM procedure
		// handle RGBD-TUM datasets manually
		if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
				this->updateGTVisualization(); // I have already taken care of the step
		}
		else if (mrpt::system::strCmpI(m_GT_file_format, "navsimul")) {
			if (m_observation_only_rawlog) {
				if (curr_rawlog_entry % 2 == 0) {
					this->updateGTVisualization();
				}
			}
			else {
				this->updateGTVisualization(); // get both action and observation at a single step - same rate as GT
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

		// Query for events and take coresponding actions
		if (m_win && m_win_observer) {
			this->queryObserverForEvents();
		}

		if (m_request_to_exit) {
			std::cout << "Halting execution... " << std::endl;

			rawlog_file.close();
			return false; // exit the parseRawlogFile method
		}

		// Reduce edges
		if (m_edge_counter.getTotalNumOfEdges() % m_num_of_edges_for_collapse == 0) {
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

			int removed_edges = m_graph.collapseDuplicatedEdges();
			m_edge_counter.setRemovedEdges(removed_edges);
		}

	} // WHILE CRAWLOG FILE
	rawlog_file.close();

	// exiting actions
	if (m_save_graph) { this->saveGraph(); }
	// remove the CPlanarLaserScan if it exists
	{
		COpenGLScenePtr& scene = m_win->get3DSceneAndLock();
		CPlanarLaserScanPtr laser_scan;
		for (; laser_scan = scene->getByClass<CPlanarLaserScan>() ;) {
			std::cout << "Removing CPlanarlaserScan from generated 3DScene..." << std::endl;
			scene->removeObject(laser_scan);
		}

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}
	
	if (m_save_3DScene) { this->save3DScene(); }

	if (mrpt::system::strCmpI(m_GT_file_format, "rgbd_tum")) {
		CImage::IMAGES_PATH_BASE = m_img_prev_path_base;
	}
	return true; // function execution completed successfully
	MRPT_END;
} // END OF FUNCTION

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::readConfigFile(
		const string& fname) {
	MRPT_START;

	VERBOSE_COUT << "Reading the .ini file... " << std::endl;

	// validation of .ini fname
	if (!mrpt::system::fileExists(fname)) {
		THROW_EXCEPTION(
				std::endl
				<< "Configuration file not found: "
				<< fname
				<< std::endl);
	}
	CConfigFile cfg_file(fname);

	if (m_rawlog_fname.empty()) {
		m_rawlog_fname = cfg_file.read_string(
				/*section_name = */ "GeneralConfiguration",
				/*var_name = */ "rawlog_file",
				/*default_value = */ "", /*failIfNotFound = */ true);
	}
	ASSERT_FILE_EXISTS_(m_rawlog_fname)

	if (m_fname_GT.empty()) {
		m_fname_GT = m_rawlog_fname + ".GT.txt"; // default output of GridmapNavSimul tool
	}


	// Section: GeneralConfiguration
	// ////////////////////////////////
	m_output_dir_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"output_dir_fname",
			"graphslam_engine_results", false);
	m_user_decides_about_output_dir = cfg_file.read_bool(
			"GeneralConfiguration",
			"user_decides_about_output_dir",
			true, false);
	m_save_graph = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_graph",
			true, false);
	m_save_3DScene = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_3DScene",
			true, false);
	m_save_graph_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_graph_fname",
			"output_graph.graph", false);
	m_save_3DScene_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_3DScene_fname",
			"scene.3DScene", false);
	m_GT_file_format = cfg_file.read_string(
			"GeneralConfiguration",
			"ground_truth_file_format",
			"NavSimul", false);

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

	m_node_registrar.loadParams(m_config_fname);
	m_edge_registrar.loadParams(m_config_fname);
	m_optimizer.loadParams(m_config_fname);

	m_has_read_config = true;
	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::printProblemParams() const {
	MRPT_START;

	ASSERT_(m_has_read_config);

	stringstream ss_out;

	ss_out << "------------[ Graphslamm_engine Problem Parameters ]------------"
		<< std::endl;
	ss_out << "Graphslam_engine: Problem Parameters " << std::endl;
	ss_out << "Config filename                 = "
		<< m_config_fname << std::endl;
	ss_out << "Rawlog filename                 = "
		<< m_rawlog_fname << std::endl;
	ss_out << "User decides about output dir?  = "
		<< m_user_decides_about_output_dir << std::endl;
	ss_out << "Output directory                = "
		<< m_output_dir_fname << std::endl;

	ss_out << "Generated .graph file?          = "
		<< m_save_graph << std::endl;
	ss_out << "Generate .3DScene file?         = "
		<< m_save_3DScene << std::endl;
	if (m_save_graph) {
		ss_out << "Generated .graph filename       = "
			<< m_save_graph_fname << std::endl;
	}
	if (m_save_3DScene) {
		ss_out << "Generated .3DScene filename     = "
			<< m_save_3DScene_fname << std::endl;
	}

	ss_out << "Ground Truth File format        = "
		<< m_GT_file_format << std::endl;
	ss_out << "Ground Truth filename           = "
		<< m_fname_GT << std::endl;
	ss_out << "Visualize odometry              = "
		<< m_visualize_odometry_poses << std::endl;
	ss_out << "Visualize estimated trajectory  = "
		<< m_visualize_estimated_trajectory << std::endl;
	ss_out << "Visualize map                   = "
		<< m_visualize_map << std::endl;
	ss_out << "Visualize Ground Truth          = "
		<< m_visualize_GT<< std::endl;
	ss_out << "Enable curr. position viewport  = " 
		<< m_enable_curr_pos_viewport << endl;
	ss_out << "Enable range img viewport       = " 
		<< m_enable_range_viewport << endl;
	ss_out << "Enable intensity img viewport   = " 
		<< m_enable_intensity_viewport << endl;
	ss_out << "-----------------------------------------------------------"
		<< std::endl;
	ss_out << std::endl;

	std::cout << ss_out.str(); ss_out.str("");

	m_node_registrar.printParams();
	m_edge_registrar.printParams();
	m_optimizer.printParams();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initOutputDir() {
	MRPT_START;

	VERBOSE_COUT << "Setting up Output directory: " << m_output_dir_fname << std::endl;

	// current time vars - handy in the rest of the function.
	TTimeStamp cur_date(getCurrentTime());
	string cur_date_str(dateTimeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));


	if (!m_has_read_config) {
		THROW_EXCEPTION(
				std::endl
				<< "Cannot initialize output directory. "
				<< "Make sure you have parsed the configuration file first"
				<< std::endl);
	}
	else {
		// Determine what to do with existing results if previous output directory
		// exists
		if (directoryExists(m_output_dir_fname)) {
			int answer_int;
			if (m_user_decides_about_output_dir) {
				/**
				 * Give the user 3 choices.
				 * - Remove the current directory contents
				 * - Rename (and keep) the current directory contents
				 */
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
				case 2: {
									VERBOSE_COUT << "Deleting existing files..." << std::endl;
									// purge directory
									deleteFilesInDirectory(m_output_dir_fname,
											/*deleteDirectoryAsWell = */ true);
									break;
								}
				case 3: {
									// Exit gracefully - call Dtor implicitly
									return;
								}
				case 1:
				default: {
									 // rename the whole directory to DATE_TIME_${OUTPUT_DIR_NAME}
									 string dst_fname = m_output_dir_fname + cur_date_validstr;
									 VERBOSE_COUT << "Renaming directory to: " << dst_fname << std::endl;
									 string* error_msg = NULL;
									 bool did_rename = renameFile(m_output_dir_fname,
											 dst_fname,
											 error_msg);
									 if (!did_rename) {
										 THROW_EXCEPTION(
												 std::endl
												 << "Error while trying to rename the output directory:"
												 << *error_msg
												 << std::endl);
									 }
									 break;
								 }
			} // SWITCH (ANSWER_INT)
		} // IF DIRECTORY EXISTS..

		// Now rebuild the directory from scratch
		VERBOSE_COUT << "Creating the new directory structure..." << std::endl;
		string cur_fname;

		// debug_fname
		createDirectory(m_output_dir_fname);
		VERBOSE_COUT << "Finished initializing output directory." << std::endl;
	}

	MRPT_END;
} // end of initOutputDir

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initResultsFile(
		const string& fname) {
	MRPT_START;

	VERBOSE_COUT << "Setting up file: " << fname << std::endl;

	// current time vars - handy in the rest of the function.
	TTimeStamp cur_date(getCurrentTime());
	string cur_date_str(dateTimeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));

	m_out_streams[fname] = new CFileOutputStream(fname);
	if (m_out_streams[fname]->fileOpenCorrectly()) {
		m_out_streams[fname]->printf("GraphSlamEngine Application\n");
		m_out_streams[fname]->printf("%s\n", cur_date_str.c_str());
		m_out_streams[fname]->printf("---------------------------------------------");
	}
	else {
		THROW_EXCEPTION(
				std::endl
				<< "Error while trying to open "
				<<	fname
				<< std::endl);
	}

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initRangeImageViewport() {
	MRPT_START;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	opengl::COpenGLViewportPtr viewp_range;

	viewp_range = scene->createViewport("viewp_range");
	double x,y,h,w;
	m_win_manager.assignViewportParameters(&x, &y, &w, &h);
	viewp_range->setViewportPosition(x, y, h, w);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::updateRangeImageViewport() {
	MRPT_START;

	if (m_last_laser_scan3D->hasRangeImage) {

	// TODO - make this a static class member - or at least a private member of the class
	CMatrixFloat range2D;
	mrpt::utils::CImage img;

	// load the image if not already loaded..
	m_last_laser_scan3D->load();
	range2D = m_last_laser_scan3D->rangeImage * (1.0/5.0); // TODO - without the magic number?
	img.setFromMatrix(range2D);

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp_range = scene->getViewport("viewp_range");
	viewp_range->setImageView(img);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	}

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initIntensityImageViewport() {
	MRPT_START;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	opengl::COpenGLViewportPtr viewp_intensity;

	viewp_intensity = scene->createViewport("viewp_intensity");
	double x, y, w, h;
	m_win_manager.assignViewportParameters(&x, &y, &w, &h);
	viewp_intensity->setViewportPosition(x, y, w, h);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	
	MRPT_END;
}
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::updateIntensityImageViewport() {
	MRPT_START;

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


template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initCurrPosViewport() {
	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	COpenGLViewportPtr viewp= scene->createViewport("curr_robot_pose_viewport");
	// Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
	viewp->setCloneView("main");
	double x,y,h,w;
	m_win_manager.assignViewportParameters(&x, &y, &w, &h);
	viewp->setViewportPosition(x, y, h, w);
	viewp->setTransparent(false);
	viewp->getCamera().setAzimuthDegrees(90);
	viewp->getCamera().setElevationDegrees(90);
	viewp->setCustomBackgroundColor(TColorf(205, 193, 197, /*alpha = */ 255));
	viewp->getCamera().setZoomDistance(30);
	viewp->getCamera().setOrthogonal();

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
}



template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
inline void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::updateCurrPosViewport() {
	MRPT_START;

	ASSERT_(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	pose_t curr_robot_pose;
	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		// get the last added pose
		curr_robot_pose = m_graph.nodes.find(m_graph.nodeCount()-1)->second;
	}

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	COpenGLViewportPtr viewp = scene->getViewport("curr_robot_pose_viewport");
	viewp->getCamera().setPointingAt(CPose3D(curr_robot_pose));

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	//VERBOSE_COUT << "Updated the \"current_pos\" viewport" << std::endl;

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::readGTFileNavSimulOutput(
		const std::string& fname_GT,
		vector<pose_t>* gt_poses,
		vector<TTimeStamp>* gt_timestamps /* = NULL */) {
	MRPT_START;

	VERBOSE_COUT << "Parsing the ground truth textfile.." << std::endl;
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
			TTimeStamp timestamp(atof(curr_tokens[0].c_str()));
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

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::readGTFileRGBD_TUM(
		const std::string& fname_GT,
		std::vector<pose_t>* gt_poses,
		std::vector<mrpt::system::TTimeStamp>* gt_timestamps/*= NULL */) {
	MRPT_START;

	VERBOSE_COUT << "Parsing the ground truth textfile.." << std::endl;

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
	system::tokenize(curr_line, " ", curr_tokens);

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

	CVectorDouble curr_coords_optical_fr(3);
	curr_coords_optical_fr[0] = atof(curr_tokens[1].c_str());
	curr_coords_optical_fr[1] = atof(curr_tokens[2].c_str());
	curr_coords_optical_fr[2] = atof(curr_tokens[3].c_str());

	// coordinates - MRPT reference frame
	CVectorDouble curr_coords_MRPT_fr;
	m_rot_TUM_to_MRPT.multiply_Ab(curr_coords_optical_fr, curr_coords_MRPT_fr);


	// TODO Add GT pose orientation
	// initial pose
	pose_t curr_pose(
			curr_coords_MRPT_fr[0],
			curr_coords_MRPT_fr[1],
			y);

	pose_diff = curr_pose;
	cout << "Initial GT pose: " << pose_diff << endl;


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
			TTimeStamp timestamp(atof(curr_tokens[0].c_str()));
			gt_timestamps->push_back(timestamp);
		}

		// pose
		// quaternion
		CQuaternionDouble quat;
		quat.r(atof(curr_tokens[7].c_str()));
		quat.x(atof(curr_tokens[4].c_str()));
		quat.y(atof(curr_tokens[5].c_str()));
		quat.z(atof(curr_tokens[6].c_str()));
		quat.rpy(r, p, y);

		CVectorDouble curr_coords_optical_fr(3);
		curr_coords_optical_fr[0] = atof(curr_tokens[1].c_str());
		curr_coords_optical_fr[1] = atof(curr_tokens[2].c_str());
		curr_coords_optical_fr[2] = atof(curr_tokens[3].c_str());

		// coordinates - MRPT reference frame
		CVectorDouble curr_coords_MRPT_fr;
		m_rot_TUM_to_MRPT.multiply_Ab(curr_coords_optical_fr, curr_coords_MRPT_fr);

		// TODO Add GT pose orientation
		// initial pose
		pose_t curr_pose(
				curr_coords_MRPT_fr[0],
				curr_coords_MRPT_fr[1],
				y);

		//curr_pose = curr_pose - pose_diff - do it one-by-one, not a vector
		//operation as it contains the angle;
		curr_pose.x() -= pose_diff.x();
		curr_pose.y() -= pose_diff.y();
		curr_pose.phi() -= pose_diff.phi();
		gt_poses->push_back(curr_pose);
		//cout << "GT pose: " << curr_pose << endl;
	}
	cout << "Size of gt_poses vector: " << gt_poses->size() << endl;
	//mrpt::system::pause();

	file_GT.close();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::autofitObjectInView(
		const CSetOfObjectsPtr& graph_obj) {
	MRPT_START;

	ASSERT_(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	CGridPlaneXYPtr obj_grid = graph_obj->CSetOfObjects::getByClass<CGridPlaneXY>();
	if (obj_grid) {
		float x_min,x_max, y_min,y_max;
		obj_grid->getPlaneLimits(x_min,x_max, y_min,y_max);
		const float z_min = obj_grid->getPlaneZcoord();
		m_win->setCameraPointingToPoint( 0.5*(x_min+x_max), 0.5*(y_min+y_max), z_min );
		m_win->setCameraZoom( 2.0f * std::max(10.0f, std::max(x_max-x_min, y_max-y_min) ) );
	}
	m_win->setCameraAzimuthDeg(60);
	m_win->setCameraElevationDeg(75);
	m_win->setCameraProjective(true);

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::queryObserverForEvents() {
	MRPT_START;

	ASSERT_(m_win);
	ASSERTMSG_(m_win_observer,
			"queryObserverForEvents method was called even though no Observer "
			"object was provided");

	std::map<std::string, bool> events_occurred;
	m_win_observer->returnEventsStruct(&events_occurred);
	m_autozoom_active = !events_occurred["mouse_clicked"];
	m_request_to_exit = events_occurred["Ctrl+c"];

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
	// pause program execution
	if (events_occurred[m_keystroke_pause_exec]) {
		std::cout << "*** Program paused.. Press any key in the *terminal* window to continue***" << std::endl;
		mrpt::system::pause();
	}

	// notify the deciders/optimizer of any events they may be interested in
	m_node_registrar.notifyOfWindowEvents(events_occurred);
	m_edge_registrar.notifyOfWindowEvents(events_occurred);
	m_optimizer.notifyOfWindowEvents(events_occurred);

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::toggleOdometryVisualization() {
	MRPT_START;
	std::cout << "Toggling Odometry visualization..." << std::endl;
 
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
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::toggleGTVisualization() {
	MRPT_START;
	std::cout << "Toggling Ground Truth visualization" << std::endl;

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
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::toggleMapVisualization() {
	MRPT_START;
	std::cout << "Toggling Map visualization... " << std::endl;

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
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::toggleEstimatedTrajectoryVisualization() {
	MRPT_START;
	std::cout << "Toggling Estimated Trajectory visualization... " << std::endl;

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
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time /* = 500 milliseconds */) {
	MRPT_START;
	
	std::cout << format("Cannot toggle visibility of specified object.\n "
			"Make sure that the corresponding visualization flag ( %s "
			") is set to true in the .ini file.\n", 
			viz_flag.c_str()).c_str() << std::endl;
	mrpt::system::sleep(sleep_time);

	MRPT_END;
}


template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::updateMapVisualization(
		const GRAPH_t& gr,
		std::map<const mrpt::utils::TNodeID,
		mrpt::obs::CObservation2DRangeScanPtr> m_nodes_to_laser_scans2D,
		bool full_update /*= false */) {
	MRPT_START;

	CTicTac map_update_timer;
	map_update_timer.Tic();

	ASSERT_(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	//cout << "Updating the map" << endl;

	// TODO - move it out
	mrpt::utils::TColor m_optimized_map_color(255, 0, 0);

	// get set of nodes to run the update for
	std::set<mrpt::utils::TNodeID> nodes_set;
	{
		if (full_update) {
			// for all the nodes get the node position and the corresponding laser scan
			// if they were recorded and visualize them
			m_graph.getAllNodes(nodes_set);
			std::cout << "Executing full update of the map" << std::endl;

		} // IF FULL UPDATE
		else { // add only current CSimplePointMap
			nodes_set.insert(m_nodeID_max);
		} // if PARTIAL_UPDATE
	}

	// for all the nodes in the previously populated set
	for (set<mrpt::utils::TNodeID>::const_iterator
			node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		// get the node pose - thread safe
		// TODO just *find* it 
		pose_t scan_pose = m_graph.nodes[*node_it];

		// name of gui object
		stringstream scan_name("");
		scan_name << "laser_scan_";
		scan_name << *node_it;

		// get the node laser scan
		CObservation2DRangeScanPtr scan_content;
		std::map<const mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr>::const_iterator search =
				m_nodes_to_laser_scans2D.find(*node_it);

		// make sure that the laser scan exists and is valid
		if (search != m_nodes_to_laser_scans2D.end() && !(search->second.null())) {
			scan_content = search->second;

			CObservation2DRangeScan scan_decimated;
			this->decimateLaserScan(*scan_content,
					&scan_decimated,
					/*keep_every_n_entries = */ 5);

			// if the scan doesn't already exist, add it to the scene, otherwise just
			// adjust its pose
			COpenGLScenePtr scene = m_win->get3DSceneAndLock();
			CRenderizablePtr obj = scene->getByName(scan_name.str());

			CSetOfObjectsPtr scan_obj;
			if (obj.null()) {
				//cout << "CSetOfObjects for nodeID " << *node_it << " doesn't exist. "
					//<< "Creating it.." << endl;

				scan_obj = CSetOfObjects::Create();

				// creating and inserting the observation in the CSetOfObjects
				CSimplePointsMap m;
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
			//cout << "Laser scans of NodeID " << *node_it << " are empty/invalid." << endl;
		}
	}


	//double elapsed_time = map_update_timer.Tac();
	//std::cout << "updateMapVisualization took: " << elapsed_time << " s" << std::endl;
	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::decimateLaserScan(
		mrpt::obs::CObservation2DRangeScan& laser_scan_in,
		mrpt::obs::CObservation2DRangeScan* laser_scan_out,
		const int keep_every_n_entries /*= 2*/) {
	MRPT_START;

	size_t scan_size = laser_scan_in.scan.size();

	std::vector<float> new_scan;
	std::vector<char> new_validRange;
	for (size_t i=0; i != scan_size; i++) {
		if (i % keep_every_n_entries == 0) {
			new_scan.push_back(laser_scan_in.scan[i]);
			new_validRange.push_back(laser_scan_in.validRange[i]);
		}
	}

	// assign the decimated scans, ranges
	laser_scan_out->scan = new_scan;
	laser_scan_out->validRange = new_validRange;

	scan_size = laser_scan_out->scan.size();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initGTVisualization() {
	MRPT_START;

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
	opengl::CSetOfObjectsPtr robot_model = stock_objects::RobotPioneer();
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

	m_win_manager.assignTextMessageParameters(
			/* offset_y*		= */ &m_offset_y_GT,
			/* text_index* = */ &m_text_index_GT);
	m_win_manager.addTextMessage(5,-m_offset_y_GT,
			format("Ground truth path"),
			TColorf(m_GT_color),
			/* unique_index = */ m_text_index_GT );

	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::updateGTVisualization() {
	MRPT_START;

	// add to the GT PointCloud and visualize it
	// check that GT vector is not depleted
	if (m_visualize_GT &&
			m_GT_poses_index < m_GT_poses.size()) {
		ASSERT_(m_win &&
				"Visualization of data was requested but no CDisplayWindow3D pointer was given");
		//cout << "Updating the GT visualization" << endl;

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("GT_cloud");
		CPointCloudPtr GT_cloud = static_cast<CPointCloudPtr>(obj);

		// add the latest GT pose
		pose_t gt_pose = m_GT_poses[m_GT_poses_index];
		m_GT_poses_index += m_GT_poses_step;
		GT_cloud->insertPoint(
				gt_pose.x(),
				gt_pose.y(),
				0 );
		//cout << "m_GT_poses_index: " << m_GT_poses_index << " / " << m_GT_poses.size() << endl;

		// robot model of GT trajectory
		obj = scene->getByName("robot_GT");
		CSetOfObjectsPtr robot_obj = static_cast<CSetOfObjectsPtr>(obj);
		robot_obj->setPose(gt_pose);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initOdometryVisualization() {
	MRPT_START;

	ASSERT_(m_has_read_config);
	ASSERT_(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

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
	opengl::CSetOfObjectsPtr robot_model = stock_objects::RobotPioneer();
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

	m_win_manager.assignTextMessageParameters(
			/* offset_y* = */ &m_offset_y_odometry,
			/* text_index* = */ &m_text_index_odometry);
	m_win_manager.addTextMessage(5,-m_offset_y_odometry,
			format("Odometry path"),
			TColorf(m_odometry_color),
			/* unique_index = */ m_text_index_odometry );

	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::updateOdometryVisualization() {
	MRPT_START;

	ASSERT_(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	//cout << "Updating the odometry visualization" << endl;
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

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::initEstimatedTrajectoryVisualization() {
	MRPT_START;

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
	opengl::CSetOfObjectsPtr robot_model = stock_objects::RobotPioneer();
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
		m_win_manager.assignTextMessageParameters( /* offset_y* = */ &m_offset_y_estimated_traj,
				/* text_index* = */ &m_text_index_estimated_traj);
		m_win_manager.addTextMessage(5,-m_offset_y_estimated_traj,
				format("Estimated trajectory"),
				TColorf(m_estimated_traj_color),
				/* unique_index = */ m_text_index_estimated_traj );
	}


	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::
updateEstimatedTrajectoryVisualization(bool full_update) {
	MRPT_START;

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
		for (set<mrpt::utils::TNodeID>::const_iterator
				nodeID_it = nodes_set.begin();
				nodeID_it != nodes_set.end(); ++nodeID_it) {

			estimated_traj_setoflines->appendLineStrip(
					m_graph.nodes[*nodeID_it].x(),
					m_graph.nodes[*nodeID_it].y(),
					0.05);
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
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::
TRGBDInfoFileParams::TRGBDInfoFileParams(std::string rawlog_fname) {

	this->setRawlogFile(rawlog_fname);
	this->initTRGBDInfoFileParams();
}
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::
TRGBDInfoFileParams::TRGBDInfoFileParams() {
	this->initTRGBDInfoFileParams();
}
template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::TRGBDInfoFileParams::setRawlogFile(
		std::string rawlog_fname) {

	// get the correct info filename from the rawlog_fname
	std::string dir = mrpt::system::extractFileDirectory(rawlog_fname);
	std::string rawlog_filename = mrpt::system::extractFileName(rawlog_fname);
	std::string name_prefix = "rawlog_";
	std::string name_suffix = "_info.txt";
	info_fname = dir + name_prefix + rawlog_filename + name_suffix;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::
TRGBDInfoFileParams::initTRGBDInfoFileParams() {
	// fields to use
	fields["Overall number of objects"] = "";
	fields["Observations format"] = "";
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::TRGBDInfoFileParams::parseFile() {
	ASSERT_FILE_EXISTS_(info_fname);

	// open file
	CFileInputStream info_file(info_fname);
	ASSERTMSG_(info_file.fileOpenCorrectly(),
			"\nTRGBDInfoFileParams::parseFile: Couldn't open info file\n");

	string curr_line;
	size_t line_cnt;

	// parse until you find an empty line.
	while (true) {
		info_file.readLine(curr_line);
		line_cnt++;
		if (curr_line.size() == 0)
			break;
	}

	// parse the meaningfull data
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

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::saveGraph() const {
	MRPT_START;
	ASSERT_(m_has_read_config);

	std::string fname = m_output_dir_fname + "/" + m_save_graph_fname;
	saveGraph(fname);

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::saveGraph(const std::string& fname) const {
	MRPT_START;
	// assertions are handled in the caller function

	m_graph.saveToTextFile(fname);
	VERBOSE_COUT << "Saved graph to text file: " << fname <<
		" successfully." << std::endl;

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::save3DScene() const {
	MRPT_START;
	ASSERT_(m_has_read_config);
	ASSERT_(m_win);

	std::string fname = m_output_dir_fname + "/" + m_save_3DScene_fname;
	save3DScene(fname);

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRAR, class EDGE_REGISTRAR, class OPTIMIZER>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRAR, EDGE_REGISTRAR, OPTIMIZER>::save3DScene(const std::string& fname) const {
	MRPT_START;
	// assertions are handled in the caller function

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	scene->saveToFile(fname);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	VERBOSE_COUT << "Saved 3DScene to external file: " << fname <<
		" successfully." << std::endl;

	MRPT_END;
}


#endif /* end of include guard: CGRAPHSLAMENGINE_IMPL_H */
