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
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::graphslam;

using namespace supplementary_funs;

using namespace std;

// Ctors, Dtors implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::CGraphSlamEngine_t(
		const string& config_file,
		CDisplayWindow3D* win /* = NULL */,
		CWindowObserver* win_observer /* = NULL */,
		string rawlog_fname /* = "" */ ):
	m_config_fname(config_file),
	m_rawlog_fname(rawlog_fname),
	m_win(win),
	kOffsetYStep(20.0), // textMessage vertical text position
	kIndexTextStep(1), // textMessage index
	m_odometry_color(0, 0, 1),
	m_GT_color(0, 1, 0),
	m_win_observer(win_observer)
{
	
	this->initGraphSlamEngine();
};

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::~CGraphSlamEngine_t() {
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
	for (fstreams_in_it it  = m_in_streams.begin(); it != m_in_streams.end(); 
			++it) {
		if ((it->second)->fileOpenCorrectly()) {
			VERBOSE_COUT << "Closing file: " << (it->first).c_str() << std::endl;
			(it->second)->close();
		}
	}

	MRPT_END;
}


// Member functions implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::initGraphSlamEngine() {
	MRPT_START;

	/** 
	 * Parameters validation
	 */
	assert(!(!m_win && m_win_observer) && 
			"CObsever was provided even though no CDisplayWindow3D was not");

	/**
	 * Initialization of various member variables
	 */
	// check for duplicated edges every..
	m_num_of_edges_for_collapse = 100;

	m_is3D = constraint_t::is_3D_val;

	// initialize the necessary maps for graph information
	graph_to_name[&m_graph] = "optimized_graph";
	graph_to_viz_params[&m_graph] = &m_optimized_graph_viz_params;

	// max node number already in the graph
	m_nodeID_max = 0;
	m_graph.root = TNodeID(0);

	// pass a graph ptr after the instance initialization
	m_node_registrator.setGraphPtr(&m_graph);
	m_edge_registrator.setGraphPtr(&m_graph);

	// pass a cdisplaywindowptr after the instance initialization
	m_node_registrator.setCDisplayWindowPtr(m_win);
	m_edge_registrator.setCDisplayWindowPtr(m_win);

	// Calling of initalization-relevant functions
	this->readConfigFile(m_config_fname);

	this->initOutputDir();
	this->printProblemParams();

	//mrpt::system::pause();

	/**
	 * Visualization-related parameters initialization
	 */

	if (!m_win) {
		m_visualize_optimized_graph = 0;
		m_visualize_odometry_poses = 0;
		m_visualize_GT = 0;
	}

	// Current Text Position
	m_curr_offset_y = 30.0;
	m_curr_text_index = 1;

	// timestamp
	this->assignTextMessageParameters(&m_offset_y_timestamp,
			&m_text_index_timestamp);

	// optimized graph
	assert(m_has_read_config);
	if (m_visualize_optimized_graph) {
		this->assignTextMessageParameters( /* offset_y*	= */ &m_offset_y_graph,
				/* text_index* = */ &m_text_index_graph );
	}


	// odometry visualization
	assert(m_has_read_config);
	if (m_visualize_odometry_poses) {
		assert(m_win && 
				"Visualization of data was requested but no CDisplayWindow3D pointer was given");

		this->assignTextMessageParameters( /* offset_y* = */ &m_offset_y_odometry,
				/* text_index* = */ &m_text_index_odometry);

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CPointCloudPtr odometry_poses_cloud = CPointCloud::Create();
		scene->insert(odometry_poses_cloud);

		odometry_poses_cloud->setPointSize(2.0);
		odometry_poses_cloud->enablePointSmooth();
		odometry_poses_cloud->enableColorFromX(false);
		odometry_poses_cloud->enableColorFromY(false);
		odometry_poses_cloud->enableColorFromZ(false);
		odometry_poses_cloud->setColor(m_odometry_color);
		odometry_poses_cloud->setName("odometry_poses_cloud");

		m_win->unlockAccess3DScene();

		m_win->addTextMessage(5,-m_offset_y_odometry,
				format("Odometry path"),
				m_odometry_color,
				m_font_name, m_font_size, // font name & size
				mrpt::opengl::NICE,
				/* unique_index = */ m_text_index_odometry );

		m_win->forceRepaint();
	}

	// GT visualization
	assert(m_has_read_config);
	if (m_visualize_GT) {
		assert(m_win && 
				"Visualization of data was requested but no CDisplayWindow3D pointer was given");

		if (m_fname_GT.empty()) {
			THROW_EXCEPTION("Visualization of Ground Truth is TRUE, but no ground"
					<< " truth file was specified");
		}
		this->BuildGroundTruthMap(m_fname_GT);

		this->assignTextMessageParameters( /* offset_y*		= */ &m_offset_y_GT,
				/* text_index* = */ &m_text_index_GT);

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CPointCloudPtr GT_cloud = CPointCloud::Create();
		scene->insert(GT_cloud);

		GT_cloud->setPointSize(2.0);
		GT_cloud->enablePointSmooth();
		GT_cloud->enableColorFromX(false);
		GT_cloud->enableColorFromY(false);
		GT_cloud->enableColorFromZ(false);
		GT_cloud->setColor(m_GT_color);
		GT_cloud->setName("GT_cloud");

		m_win->unlockAccess3DScene();

		m_win->addTextMessage(5,-m_offset_y_GT,
				format("Ground truth path"),
				m_GT_color,
				m_font_name, m_font_size, // font name & size
				mrpt::opengl::NICE,
				/* unique_index = */ m_text_index_GT );

		m_win->forceRepaint();
	}

	// second viewport
	if (m_win) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();
		COpenGLViewportPtr viewp= scene->createViewport("curr_robot_pose_viewport");
		// Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
		viewp->setCloneView("main");
		viewp->setViewportPosition(0.78,0.78,0.20,0.20);
		viewp->setTransparent(false);
		viewp->getCamera().setAzimuthDegrees(90);
		viewp->getCamera().setElevationDegrees(90);
		viewp->setCustomBackgroundColor(TColorf(205, 193, 197, /*alpha = */ 255));
		viewp->getCamera().setZoomDistance(30);
		viewp->getCamera().setOrthogonal();

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();

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

	// robot model
	if (m_win) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		opengl::CSetOfObjectsPtr obj = stock_objects::RobotPioneer();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("robot_model");
		obj->setColor_u8(TColor(142, 142, 56));
		obj->setScale(5);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
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

		this->assignTextMessageParameters(&offset_y_total_edges, 
				&text_index_total_edges);
		//std::cout << "in GraphSlamEngine:	" << std::endl
		//<< "offset_y_total_edges: " << offset_y_total_edges << std::endl
		//<< "text_index_total_edges: " << text_index_total_edges << std::endl;

		// register all the edge types
		vector<string> vec_edge_types;
		vec_edge_types.push_back("Odometry");m_edge_counter.addEdgeType("Odometry");
		vec_edge_types.push_back("ICP"); m_edge_counter.addEdgeType("ICP");
		vec_edge_types.push_back("Visual"); m_edge_counter.addEdgeType("Visual");

		// build each one of these
		map<string, double> name_to_offset_y;
		map<string, int> name_to_text_index;
		for (vector<string>::const_iterator it = vec_edge_types.begin(); 
				it != vec_edge_types.end();
				++it) {
			this->assignTextMessageParameters(&name_to_offset_y[*it], 
					&name_to_text_index[*it]);
			//std::cout << "in initGraphSlamEngine: " << std::endl;
			//std::cout << "name: " << *it << " | offset_y: "
			//<< name_to_offset_y[*it] << " | text_index: "
			//<< name_to_text_index[*it] << std::endl;
		}

		this->assignTextMessageParameters(&offset_y_loop_closures, 
				&text_index_loop_closures);
		//std::cout << "in GraphSlamEngine:	" << std::endl
		//<< "offset_y_loop_closures: " << offset_y_loop_closures << std::endl
		//<< "text_index_loop_closures: " << text_index_loop_closures <<std::endl;

		if (m_win) {
			// add all the parameters to the CEdgeCounter_t object
			m_edge_counter.setTextMessageParams(name_to_offset_y, name_to_text_index,
					offset_y_total_edges, text_index_total_edges,
					offset_y_loop_closures, text_index_loop_closures,
					m_font_name, m_font_size);
		}
	}

	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		m_node_registrator.initializeVisuals();
		m_edge_registrator.initializeVisuals();
	}

	// various flags initialization
	m_autozoom_active = true;

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
bool CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::parseRawlogFile() {
	MRPT_START;

	if (!m_has_read_config)
		THROW_EXCEPTION("Config file has not been provided yet.\nExiting...");
	if (!fileExists(m_rawlog_fname))
		THROW_EXCEPTION("parseRawlogFile: Inputted rawlog file ( "
				<< m_rawlog_fname << " ) not found");
 // good to go..

	/**
	 * Variables initialization
	 */

	CFileGZInputStream rawlog_file(m_rawlog_fname);
	CActionCollectionPtr action;
	CSensoryFramePtr observations;
	CObservationPtr observation;
	size_t curr_rawlog_entry = 0;

	// Tracking the PDF of the current position of the robot with regards to the
	// PREVIOUS registered node
	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 1.0 };
	InfMat init_path_uncertainty(tmp);
	pose_t last_pose_inserted; // defaults to all 0s
	constraint_t since_prev_node_PDF(last_pose_inserted, init_path_uncertainty);
	pose_t curr_odometry_only_pose; // defaults to all 0s

	TNodeID from = m_graph.root; // first node shall be the root - 0

	size_t curr_GT_poses_index = 0;

	// Read the first rawlog pair / observation explicitly to register the laser
	// scan of the root node
	{
		CRawlog::getActionObservationPairOrObservation(
				rawlog_file,
				action,	// Possible out var: Action of a a pair action / obs
				observations, // Possible out var: obs's of a pair actin		 / obs
				observation, // Possible out var
				curr_rawlog_entry );

		std::cout << "Updating edge_registrator for nodeID " << from << std::endl;
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		m_edge_registrator.updateDeciderState(
				action, 
				observations,
				observation);
	}

	// Read the rest of the rawlog file
	while (CRawlog::getActionObservationPairOrObservation(
				rawlog_file,
				action,	// Possible out var: Action of a a pair action / obs
				observations,	// Possible out var: obs's of a pair actin		 / obs
				observation, // Possible out var
				curr_rawlog_entry ) ) {

		// process action & observations
		if (observation.present()) {
			// Read a single observation from the rawlog
			// (Format #2 rawlog file)
			//TODO Implement 2nd format
			THROW_EXCEPTION("Observation-only format is not yet supported");
		}
		else {
			// action, observations should contain a pair of valid data
			// (Format #1 rawlog file)

			// parse the current action
			CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
			CPosePDFPtr increment = robot_move->poseChange;
			pose_t increment_pose = increment->getMeanVal();
			curr_odometry_only_pose += increment_pose;

			// Timestamp textMessage
			// Use the dataset timestamp otherwise fallback to mrpt::system::now()
			TTimeStamp	timestamp = robot_move->timestamp;
			if (timestamp != INVALID_TIMESTAMP && m_win) {
				m_win->addTextMessage(5,-m_offset_y_timestamp,
						format("Simulated time: %s", timeLocalToString(timestamp).c_str()),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_timestamp );
			}
			else if (m_win) {
				m_win->addTextMessage(5,-m_offset_y_timestamp,
						format("Wall time: %s", timeLocalToString(system::now()).c_str()),
						TColorf(1.0, 1.0, 1.0),
						m_font_name, m_font_size, // font name & size
						mrpt::opengl::NICE,
						/* unique_index = */ m_text_index_timestamp );
			}

			// add to the odometry PointCloud and visualize it
			{
				// fill in the odometry_poses vector
				pose_t* odometry_pose = new pose_t;
				*odometry_pose = curr_odometry_only_pose;
				m_odometry_poses.push_back(odometry_pose);

				if (m_visualize_odometry_poses) {
					assert(m_win && 
							"Visualization of data was requested but no CDisplayWindow3D pointer was given");

					COpenGLScenePtr scene = m_win->get3DSceneAndLock();

					CRenderizablePtr obj = scene->getByName("odometry_poses_cloud");
					CPointCloudPtr odometry_poses_cloud = static_cast<CPointCloudPtr>(obj);
					odometry_poses_cloud->insertPoint(
							m_odometry_poses.back()->x(),
							m_odometry_poses.back()->y(),
							0 );

					m_win->unlockAccess3DScene();
					m_win->forceRepaint();
				}

				delete odometry_pose;
			}

			// add to the GT PointCloud and visualize it
			// check that GT vector is not depleted
			if (m_visualize_GT && 
					curr_GT_poses_index < m_GT_poses.size()) {
				assert(m_win &&
						"Visualization of data was requested but no CDisplayWindow3D pointer was given");

				COpenGLScenePtr scene = m_win->get3DSceneAndLock();

				CRenderizablePtr obj = scene->getByName("GT_cloud");
				CPointCloudPtr GT_cloud = static_cast<CPointCloudPtr>(obj);

				// get current point by matching the timestamp
				pose_t* curr_pose = m_GT_poses[curr_GT_poses_index++];
				GT_cloud->insertPoint(
						curr_pose->x(),
						curr_pose->y(),
						0 );

				m_win->unlockAccess3DScene();
				m_win->forceRepaint();
			}

			// new node registration procedure
			bool registered_new_node;
			{
				mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
				registered_new_node = m_node_registrator.updateDeciderState(
						action, observations, observation);
				if (registered_new_node) {
					m_edge_counter.addEdge("Odometry");
				}
			}

			if (registered_new_node) {

				// Edge registration procedure
				{ 
					mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
					m_graph.dijkstra_nodes_estimate(); 

					m_edge_registrator.updateDeciderState(
							action,
							observations,
							observation );
				}

				// join the previous optimization thread
				joinThread(m_thread_optimize);

				// optimize the graph - run on a seperate thread
				m_thread_optimize = createThreadFromObjectMethod(/*obj = */this, 
						/* func = */&CGraphSlamEngine_t::optimizeGraph, &m_graph);

				// update the visualization window
				if (m_visualize_optimized_graph) {
					assert(m_win && 
							"Visualization of data was requested but no CDisplayWindow3D pointer was given");

					visualizeGraph(m_graph);
					updateCurPosViewport(m_graph);
				}

				// update robot model
				if (m_win) {
					COpenGLScenePtr scene = m_win->get3DSceneAndLock();

					CRenderizablePtr obj = scene->getByName("robot_model");
					CSetOfObjectsPtr robot_obj = static_cast<CSetOfObjectsPtr>(obj);

					mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

					robot_obj->setPose(m_graph.nodes[m_graph.nodeCount()-1]);

					m_win->unlockAccess3DScene();
				}

				{
					mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
					m_node_registrator.updateVisuals();
					m_edge_registrator.updateVisuals();
				}

				// update the edge counter
				std::map<const std::string, int> edge_types_to_nums;
				m_edge_registrator.getEdgesStats(&edge_types_to_nums);
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
			} // IF REGISTERED_NEW_NODE
		} // ELSE FORMAT #1


		/** 
		 * query for events and take coresponding actions
		 */
		this->queryObserverForEvents();

		if (m_request_to_exit) {
			std::cout << "Halting execution... " << std::endl;
			joinThread(m_thread_optimize);
			return false; // exit the parseRawlogFile method
		}

		/**
		 * Reduce edges
		 */
		if (m_edge_counter.getTotalNumOfEdges() % m_num_of_edges_for_collapse == 0) {
			mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
			//std::cout << "Collapsing duplicated edges..." << std::endl;

			int removed_edges = m_graph.collapseDuplicatedEdges();
			m_edge_counter.setRemovedEdges(removed_edges);
		}

	} // WHILE CRAWLOG FILE
	return true; // function execution completed successfully
MRPT_END;
} // END OF FUNCTION

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::optimizeGraph(GRAPH_t* gr) {
	MRPT_START;

	CTicTac optimization_timer;
	optimization_timer.Tic();

	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

	//std::cout << "In optimizeGraph: threadID: " << getCurrentThreadId()<< std::endl;

	//std::cout << "Executing the graph optimization" << std::endl;
	graphslam::TResultInfoSpaLevMarq	levmarq_info;

	// Execute the optimization
	graphslam::optimize_graph_spa_levmarq(
			*gr,
			levmarq_info,
			NULL,  // List of nodes to optimize. NULL -> all but the root node.
			m_optimization_params,
			&levMarqFeedback<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>); // functor feedback

	double elapsed_time = optimization_timer.Tac();
	//VERBOSE_COUT << "Optimization of graph took: " << elapsed_time << "s" << std::endl;

	MRPT_UNUSED_PARAM(elapsed_time);
	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::visualizeGraph(
		const GRAPH_t& gr) {
	MRPT_START;

	assert(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	//std::cout << "Inside the visualizeGraph function" << std::endl;
	
	mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);

	string gr_name = graph_to_name[&gr];
	const TParametersDouble* viz_params = graph_to_viz_params[&gr];

	// update the graph (clear and rewrite..)
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	// remove previous graph
	CRenderizablePtr prev_object = scene->getByName(gr_name);
	scene->removeObject(prev_object);

	// Insert the new instance of the graph
	CSetOfObjectsPtr graph_obj = graph_tools::graph_visualize(gr, *viz_params);
	graph_obj->setName(gr_name);
	scene->insert(graph_obj);

	m_win->unlockAccess3DScene();
	m_win->addTextMessage(5,-m_offset_y_graph,
			format("Optimized Graph: #nodes %d",
				static_cast<int>(gr.nodeCount())),
			TColorf(0.0, 0.0, 0.0),
			m_font_name, m_font_size, // font name & size
			mrpt::opengl::NICE,
			/* unique_index = */ m_text_index_graph);

	m_win->forceRepaint();

	// Autozoom to the graph?
	if (m_autozoom_active) {
		this->autofitObjectInView(graph_obj);
	}

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::readConfigFile(
		const string& fname) {
	MRPT_START;

	VERBOSE_COUT << "Reading the .ini file... " << std::endl;

	CConfigFile cfg_file(fname);

	if (m_rawlog_fname.empty()) {
		m_rawlog_fname = cfg_file.read_string(
				/*section_name = */ "GeneralConfiguration",
				/*var_name = */ "rawlog_file",
				/*default_value = */ "", /*failIfNotFound = */ true);
	}
	m_fname_GT = m_rawlog_fname + ".GT.txt"; // default output of GridmapNavSimul tool


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
	m_debug_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"debug_fname",
			"debug.log", false);
	m_save_graph_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_graph_fname",
			"poses.log", false);

	// Section: Optimization
	// ////////////////////////////
	m_optimization_params["verbose"] = cfg_file.read_bool(
			"Optimization",
			"verbose",
			0, false);
	m_optimization_params["profiler"] = cfg_file.read_bool(
			"Optimization",
			"profiler",
			0, false);
		m_optimization_params["max_iterations"] = cfg_file.read_double(
			"Optimization",
			"max_iterations",
			100, false);
	m_optimization_params["scale_hessian"] = cfg_file.read_double(
			"Optimization",
			"scale_hessian",
			0.2, false);
	m_optimization_params["tau"] = cfg_file.read_double(
			"Optimization",
			"tau",
			1e-3, false);


	// Section: VisualizationParameters
	// ////////////////////////////////
	// http://reference.mrpt.org/devel/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61

	m_font_name = cfg_file.read_string(
			"VisualizationParameters",
			"font_name",
			"sans", false);
	m_font_size = cfg_file.read_int(
			"VisualizationParameters",
			"font_size",
			12, false);


	// Optimized graph
	m_visualize_optimized_graph = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_optimized_graph",
			1, false);

	m_optimized_graph_viz_params["show_ID_labels"] = cfg_file.read_bool(
			"VisualizationParameters",
			"optimized_show_ID_labels",
			0, false);
	m_optimized_graph_viz_params["show_ground_grid"] = cfg_file.read_double(
			"VisualizationParameters",
			"optimized_show_ground_grid",
			1, false);
	m_optimized_graph_viz_params["show_edges"] = cfg_file.read_bool(
			"VisualizationParameters",
			"optimized_show_edges",
			1, false);
	m_optimized_graph_viz_params["edge_color"] = cfg_file.read_int(
			"VisualizationParameters",
			"optimized_edge_color",
			4286611456, false);
	m_optimized_graph_viz_params["edge_width"] = cfg_file.read_double(
			"VisualizationParameters",
			"optimized_edge_width",
			1.5, false);
	m_optimized_graph_viz_params["show_node_corners"] = cfg_file.read_bool(
			"VisualizationParameters",
			"optimized_show_node_corners",
			1, false);
	m_optimized_graph_viz_params["show_edge_rel_poses"] = cfg_file.read_bool(
			"VisualizationParameters",
			"optimized_show_edge_rel_poses",
			1, false);
	m_optimized_graph_viz_params["edge_rel_poses_color"] = cfg_file.read_int(
			"VisualizationParameters",
			"optimized_edge_rel_poses_color",
			1090486272, false);
	m_optimized_graph_viz_params["nodes_edges_corner_scale"] = cfg_file.read_double(
			"VisualizationParameters",
			"optimized_nodes_edges_corner_scale",
			0.4, false);
	m_optimized_graph_viz_params["nodes_corner_scale"] = cfg_file.read_double(
			"VisualizationParameters",
			"optimized_nodes_corner_scale",
			0.7, false);
	m_optimized_graph_viz_params["point_size"] = cfg_file.read_int(
			"VisualizationParameters",
			"optimized_point_size",
			0, false);
	m_optimized_graph_viz_params["point_color"] = cfg_file.read_int(
			"VisualizationParameters",
			"optimized_point_color",
			10526880, false);

	// odometry-only visualization

	m_visualize_odometry_poses = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_odometry_poses",
			1, false);

	// GT configuration / visualization
	m_visualize_GT = cfg_file.read_bool(
			"VisualizationParameters",
			"visualize_ground_truth",
			1, false);

	this->m_node_registrator.params.loadFromConfigFileName(m_config_fname, 
			"NodeRegistrationDecidersParameters");
	this->m_edge_registrator.params.loadFromConfigFileName(m_config_fname, 
			"EdgeRegistrationDecidersParameters");

	m_has_read_config = true;
	MRPT_END;
}

// TODO - check for new value from above
template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::printProblemParams() const {
	MRPT_START;

	assert(m_has_read_config);

	stringstream ss_out;

	ss_out << "------------[ Graphslamm_engine Problem Parameters ]------------" 
		<< std::endl;
	ss_out << "Graphslam_engine: Problem Parameters " << std::endl;
	ss_out << "Config filename                 = "
		<< m_config_fname << std::endl;
	ss_out << "Rawlog filename                 = "
		<< m_rawlog_fname << std::endl;
	ss_out << "Output directory                = "
		<< m_output_dir_fname << std::endl;
	ss_out << "User decides about output dir   = "
		<< m_user_decides_about_output_dir << std::endl;
	ss_out << "save_graph_fname                = "
		<< m_save_graph_fname << std::endl;
	ss_out << "Visualize odometry              = " 
		<< m_visualize_odometry_poses << std::endl;
	ss_out << "Visualize optimized path        = " 
		<< m_visualize_odometry_poses << std::endl;
	ss_out << "Visualize Ground Truth          = " 
		<< m_visualize_odometry_poses << std::endl;
	ss_out << "Ground Truth filename           = "
		<< m_fname_GT << std::endl;
	ss_out << "-----------------------------------------------------------" 
		<< std::endl;
	ss_out << std::endl;

	std::cout << ss_out.str(); ss_out.str("");

	std::cout << "-----------[ Optimization Parameters ]-----------" << std::endl;
	m_optimization_params.dumpToConsole();
	std::cout << "-----------[ Graph Visualization Parameters ]-----------" << std::endl;
	m_optimized_graph_viz_params.dumpToConsole();

	std::cout << ss_out.str(); ss_out.str("");

	this->m_node_registrator.params.dumpToConsole();
	this->m_edge_registrator.params.dumpToConsole();

	//mrpt::system::pause();

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::initOutputDir() {
	MRPT_START;

	VERBOSE_COUT << "Setting up Output directory: " << m_output_dir_fname << std::endl;

	// current time vars - handy in the rest of the function.
	TTimeStamp cur_date(getCurrentTime());
	string cur_date_str(dateTimeToString(cur_date));
	string cur_date_validstr(fileNameStripInvalidChars(cur_date_str));


	if (!m_has_read_config) {
		THROW_EXCEPTION("Cannot initialize output directory. " <<
				"Make sure you have parsed the configuration file first");
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
						THROW_EXCEPTION("Error while trying to rename the output directory:" <<
								*error_msg);
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
		//if (m_do_debug) {
			//cur_fname = m_output_dir_fname + "/" + m_debug_fname;
			//this->initResultsFile(cur_fname);
		//}

		VERBOSE_COUT << "Finished initializing output directory." << std::endl;
	}

	MRPT_END;
} // end of initOutputDir

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::initResultsFile(
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
		THROW_EXCEPTION("Error while trying to open " <<	fname);
	}

	MRPT_END;
}

/**
 * updateCurPosViewport
 *
 * Udpate the viewport responsible for displaying the graph-building procedure
 * in the estimated position of the robot
 */
template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
inline void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::updateCurPosViewport(
		const GRAPH_t& gr) {
	MRPT_START;

	assert(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	pose_t curr_robot_pose;
	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(&m_graph_section);
		// get the last added pose
		curr_robot_pose = gr.nodes.find(gr.nodeCount()-1)->second; 
	}

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	COpenGLViewportPtr viewp = scene->getViewport("curr_robot_pose_viewport");
	viewp->getCamera().setPointingAt(CPose3D(curr_robot_pose));

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	//VERBOSE_COUT << "Updated the \"current_pos\" viewport" << std::endl;

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::BuildGroundTruthMap(const std::string& fname_GT) {
	MRPT_START;

	VERBOSE_COUT << "Parsing the ground truth file textfile.." << std::endl;
	assert(fileExists(fname_GT) && "Ground truth file was not found.");

	CFileInputStream* file_GT = new CFileInputStream(fname_GT);
	m_in_streams[fname_GT] = file_GT;

	if (file_GT->fileOpenCorrectly()) {
		string curr_line; 

		// parse the file - get timestamp and pose and fill in the pose_t vector
		for (size_t line_num = 0; file_GT->readLine(curr_line); line_num++) {
			vector<string> curr_tokens;
			system::tokenize(curr_line, " ", curr_tokens);

			// check the current pose dimensions
			if (curr_tokens.size() != constraint_t::state_length + 1) {
				THROW_EXCEPTION("Wrong length of curent pose at line " << line_num);
			}
			pose_t *curr_pose = new pose_t(atof(curr_tokens[1].c_str()),
					atof(curr_tokens[2].c_str()),
					atof(curr_tokens[3].c_str()) );
			m_GT_poses.push_back(curr_pose);
		}
	}
	else {
		THROW_EXCEPTION("BuildGroundTruthMap: Can't open GT file (" << fname_GT
				<< ")");
	}

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::autofitObjectInView(
		const CSetOfObjectsPtr& graph_obj) {
	MRPT_START;

	assert(m_win &&
			"Visualization of data was requested but no CDisplayWindow3D pointer was given");

	CGridPlaneXYPtr obj_grid = graph_obj->CSetOfObjects::getByClass<CGridPlaneXY>();
	if (obj_grid) {
		float x_min,x_max, y_min,y_max;
		obj_grid->getPlaneLimits(x_min,x_max, y_min,y_max);
		const float z_min = obj_grid->getPlaneZcoord();
		m_win->setCameraPointingToPoint( 0.5*(x_min+x_max), 0.5*(y_min+y_max), z_min );
		m_win->setCameraZoom( 2.0f * std::max(10.0f, std::max(x_max-x_min, y_max-y_min) ) );
	}
	m_win->setCameraAzimuthDeg(0);
	m_win->setCameraElevationDeg(75);

	MRPT_END;
}

template<class GRAPH_t, class NODE_REGISTRATOR, class EDGE_REGISTRATOR>
void CGraphSlamEngine_t<GRAPH_t, NODE_REGISTRATOR, EDGE_REGISTRATOR>::queryObserverForEvents() {
	MRPT_START;

	assert(m_win_observer &&
			"queryObserverForEvents method was called even though no Observer object was provided");

	const TParameters<bool>* events_occurred = m_win_observer->returnEventsStruct();
	m_autozoom_active = !(*events_occurred)["mouse_clicked"];
	m_request_to_exit = (*events_occurred)["request_to_exit"];

	MRPT_END;
}

#endif /* end of include guard: CGRAPHSLAMENGINE_IMPL_H */

