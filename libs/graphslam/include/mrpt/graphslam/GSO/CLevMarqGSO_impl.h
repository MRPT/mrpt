/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CLEVMARQGSO_IMPL_H
#define CLEVMARQGSO_IMPL_H

namespace mrpt { namespace graphslam { namespace optimizers {

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CLevMarqGSO<GRAPH_t>::CLevMarqGSO()
{
	MRPT_START;

	this->initCLevMarqGSO();

	MRPT_END;
}
template<class GRAPH_t>
CLevMarqGSO<GRAPH_t>::~CLevMarqGSO() {
	// JL Removed MRPT_END since it could launch an exception, not allowed in a dtor
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::initCLevMarqGSO() {
	MRPT_START;

	using namespace mrpt::utils;

	m_graph = NULL;
	m_win_manager = NULL;
	m_win_observer = NULL;
	m_graph_section = NULL;


	m_first_time_call = false;
	m_initialized_visuals = false;
	m_has_read_config = false;
	m_last_total_num_of_nodes = 5;
	m_autozoom_active = true;

	this->setLoggerName("CLevMarqGSO");
	this->logging_enable_keep_record = true;

	MRPT_END;
}

// Member function implementations
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
bool CLevMarqGSO<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	using namespace mrpt::utils;
	this->logFmt(LVL_DEBUG, "In updateOptimizerState... ");

	if (m_graph->nodeCount() > m_last_total_num_of_nodes) {
		m_last_total_num_of_nodes = m_graph->nodeCount();
		registered_new_node = true;

		if (m_first_time_call) {
			opt_params.last_pair_nodes_to_edge = m_graph->edges;
			m_first_time_call = true;
		}


		if (opt_params.optimization_on_second_thread) {
			//join the previous optimization thread
			mrpt::system::joinThread(m_thread_optimize);

			// optimize the graph - run on a seperate thread
			m_thread_optimize = mrpt::system::createThreadFromObjectMethod(
					/*obj = */ this,
					/* func = */ &CLevMarqGSO::optimizeGraph);

		}
		else { // single threaded implementation
			this->_optimizeGraph();
		}

	}

	return true;
	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;
	using namespace mrpt::utils;

	m_graph = graph;

	this->logFmt(mrpt::utils::LVL_DEBUG, "Fetched the graph successfully");

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	MRPT_START;
	ASSERT_(win_manager);

	m_win_manager = win_manager;
	if (m_win_manager) {
		m_win = m_win_manager->win;

		m_win_observer = m_win_manager->observer;
	}
	this->logFmt(mrpt::utils::LVL_DEBUG, "Fetched the CDisplayWindow successfully");

	MRPT_END;
}

template <class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::setCriticalSectionPtr(
		mrpt::synch::CCriticalSection* graph_section) {
	MRPT_START;

	m_graph_section = graph_section;

	this->logFmt(mrpt::utils::LVL_DEBUG, "Fetched the CCRiticalSection successfully");
	MRPT_END;
}


template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	ASSERT_(m_win_manager);
	this->logFmt(mrpt::utils::LVL_DEBUG, "Initializing visuals");

	ASSERTMSG_(m_win,
			"Visualization of data was requested but no CDisplayWindow3D pointer "
			" was given.");
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	ASSERT_(m_has_read_config);

	this->initGraphVisualization();
	this->initOptDistanceVisualization();

	m_initialized_visuals = true;
	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::updateVisuals() {
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	ASSERT_(m_initialized_visuals);
	ASSERTMSG_(m_win,
			"Visualization of data was requested but no CDisplayWindow3D pointer was given.");

	this->updateOptDistanceVisualization();
	this->updateGraphVisualization();

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred)
{
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	using namespace std;

	//// print the keys for debuggging reasons
	//for (std::map<std::string, bool>::const_iterator cit = events_occurred.begin();
			//cit != events_occurred.end(); ++cit) {
		//cout << "\t" << cit->first << " ==> " << cit->second << endl;
	//}

	// I know the keys exists - I put it there explicitly

	// optimization_distance toggling
	if (opt_params.optimization_distance > 0) {
		if (events_occurred.find(
					opt_params.keystroke_optimization_distance)->second) {
			this->toggleOptDistanceVisualization();
		}
	}

	// graph toggling
	if (events_occurred.find(viz_params.keystroke_graph_toggle)->second) {
		this->toggleGraphVisualization();
	}

	// if mouse event, let the user decide about the camera
	if (events_occurred.find("mouse_clicked")->second) {
		MRPT_LOG_DEBUG_STREAM << "Mouse was clicked. Disabling autozoom.";
		m_autozoom_active = false;
	}

	// autofit the graph once
	if (events_occurred.find(viz_params.keystroke_graph_autofit)->second) {
		MRPT_LOG_DEBUG_STREAM << "Autofit button was pressed";
		this->fitGraphInView();
	}

	MRPT_END;
}

template<class GRAPH_t>
inline void CLevMarqGSO<GRAPH_t>::initGraphVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");

	if (viz_params.visualize_optimized_graph) {
		m_win_observer->registerKeystroke(viz_params.keystroke_graph_toggle,
				"Toggle Graph visualization");
		m_win_observer->registerKeystroke(viz_params.keystroke_graph_autofit,
				"Fit Graph in view");

		m_win_manager->assignTextMessageParameters(
				/* offset_y*	= */ &viz_params.offset_y_graph,
				/* text_index* = */ &viz_params.text_index_graph );

	}


	MRPT_END;
}
template<class GRAPH_t>
inline void CLevMarqGSO<GRAPH_t>::updateGraphVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	using namespace mrpt::opengl;
	using namespace mrpt::utils;

	this->logFmt(mrpt::utils::LVL_DEBUG, "In the updateGraphVisualization function");

	// update the graph (clear and rewrite..)
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	// remove previous graph and insert the new instance
	CRenderizablePtr prev_object = scene->getByName("optimized_graph");
	bool prev_visibility = true;
	if (prev_object) { // set the visibility of the graph correctly
		prev_visibility = prev_object->isVisible();
	}
	scene->removeObject(prev_object);

	CSetOfObjectsPtr graph_obj =
		graph_tools::graph_visualize(*m_graph, viz_params.cfg);
	graph_obj->setName("optimized_graph");
	graph_obj->setVisibility(prev_visibility);
	scene->insert(graph_obj);
	m_win->unlockAccess3DScene();

	m_win_manager->addTextMessage(5,-viz_params.offset_y_graph,
			format("Optimized Graph: #nodes %d",
				static_cast<int>(m_graph->nodeCount())),
			TColorf(0.0, 0.0, 0.0),
			/* unique_index = */ viz_params.text_index_graph);

	m_win->forceRepaint();

	if (m_autozoom_active) {
		this->fitGraphInView();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::toggleGraphVisualization() {
	MRPT_START;
	using namespace mrpt::opengl;

	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	CRenderizablePtr graph_obj = scene->getByName("optimized_graph");
	graph_obj->setVisibility(!graph_obj->isVisible());

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::fitGraphInView() {
	MRPT_START;
	using namespace mrpt::opengl;

	ASSERTMSG_(m_win,
			"\nVisualization of data was requested but no CDisplayWindow3D pointer was given\n");

	// first fetch the graph object
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();
	CRenderizablePtr obj = scene->getByName("optimized_graph");
	CSetOfObjectsPtr graph_obj = static_cast<CSetOfObjectsPtr>(obj);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	// autofit it based on its grid
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


template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::initOptDistanceVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	using namespace mrpt::opengl;

	if (opt_params.optimization_distance > 0) {
		m_win_observer->registerKeystroke(opt_params.keystroke_optimization_distance,
				"Toggle optimization distance on/off");

		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CDiskPtr obj = CDisk::Create();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("optimization_distance_disk");
		obj->setColor_u8(opt_params.optimization_distance_color);
		obj->setDiskRadius(opt_params.optimization_distance,
				opt_params.optimization_distance-0.1);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();

		// optimization distance disk - textMessage
		m_win_manager->assignTextMessageParameters(
				&opt_params.offset_y_optimization_distance,
				&opt_params.text_index_optimization_distance);

		m_win_manager->addTextMessage(5,-opt_params.offset_y_optimization_distance,
				format("Radius for graph optimization"),
				mrpt::utils::TColorf(opt_params.optimization_distance_color),
				/* unique_index = */ opt_params.text_index_optimization_distance );
	}

	m_initialized_visuals = true;

	MRPT_END;
}
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::updateOptDistanceVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	using namespace mrpt::opengl;

	// update ICP_max_distance Disk
	if (opt_params.optimization_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("optimization_distance_disk");
		CDiskPtr disk_obj = static_cast<CDiskPtr>(obj);

		disk_obj->setPose(m_graph->nodes[m_graph->nodeCount()-1]);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}

// TODO - implement this
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::toggleOptDistanceVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	using namespace mrpt::opengl;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	CRenderizablePtr obj = scene->getByName("optimization_distance_disk");
	obj->setVisibility(!obj->isVisible());

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}



template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::optimizeGraph() {
	MRPT_START;

	this->logFmt(mrpt::utils::LVL_DEBUG,
				"In optimizeGraph\n\tThreadID: %lu\n\tTrying to grab lock... ",
				mrpt::system::getCurrentThreadId());

	mrpt::synch::CCriticalSectionLocker m_graph_lock(m_graph_section);
	this->_optimizeGraph();

	this->logFmt(mrpt::utils::LVL_DEBUG, "2nd thread grabbed the lock..");

	MRPT_END;
}

// TODO - do something meaningful with these parameters
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::_optimizeGraph() {
	MRPT_START;
	m_time_logger.enter("CLevMarqGSO::_optimizeGraph");
	this->logFmt(mrpt::utils::LVL_DEBUG, "In _optimizeGraph");

	using namespace mrpt::utils;

	CTicTac optimization_timer;
	optimization_timer.Tic();


	// set of nodes for which the optimization procedure will take place
	std::set< mrpt::utils::TNodeID>* nodes_to_optimize;

	// fill in the nodes in certain distance to the current node, only if
	// full_update is not instructed

	bool full_update = opt_params.optimization_distance == -1 || this->checkForLoopClosures();
	if (full_update) {
		nodes_to_optimize = NULL;
		this->logFmt(mrpt::utils::LVL_DEBUG,
				"Commencing with FULL graph optimization... ");
	}
	else {
		nodes_to_optimize = new std::set<mrpt::utils::TNodeID>;

		// I am certain that this shall not be called when nodeCount = 0, since the
		// optimization procedure starts only after certain number of nodes has
		// been added
		this->getNearbyNodesOf(nodes_to_optimize,
				m_graph->nodeCount()-1,
				opt_params.optimization_distance);
		nodes_to_optimize->insert(m_graph->nodeCount()-1);
	}

	graphslam::TResultInfoSpaLevMarq	levmarq_info;

	// Execute the optimization
	mrpt::graphslam::optimize_graph_spa_levmarq(
			*m_graph,
			levmarq_info,
			nodes_to_optimize,  // List of nodes to optimize. NULL -> all but the root node.
			opt_params.cfg,
			&CLevMarqGSO<GRAPH_t>::levMarqFeedback); // functor feedback

	double elapsed_time = optimization_timer.Tac();
	this->logFmt(mrpt::utils::LVL_DEBUG,
			"Optimization of graph took: %fs", elapsed_time);

	// deleting the nodes_to_optimize set
	delete nodes_to_optimize;
	nodes_to_optimize = NULL;

	m_time_logger.leave("CLevMarqGSO::_optimizeGraph");
	MRPT_UNUSED_PARAM(elapsed_time);
	MRPT_END;
}

template<class GRAPH_t>
bool CLevMarqGSO<GRAPH_t>::checkForLoopClosures() {
	MRPT_START;

	bool is_loop_closure = false;
	typename GRAPH_t::edges_map_t curr_pair_nodes_to_edge =  m_graph->edges;

	// find the *node pairs* that exist in current but not the last nodes_to_edge
	// map If the distance of any of these pairs is greater than
	// LC_min_nodeid_diff then consider this a loop closure
	typename GRAPH_t::edges_map_t::const_iterator search;
	mrpt::utils::TPairNodeIDs curr_pair;

	for (typename GRAPH_t::edges_map_t::const_iterator it =
			curr_pair_nodes_to_edge.begin(); it != curr_pair_nodes_to_edge.end();
			++it) {
		search = opt_params.last_pair_nodes_to_edge.find(it->first);
		// if current node pair is not found in the last set...
		if (search == opt_params.last_pair_nodes_to_edge.end()) {
			curr_pair = it->first;

			if (std::abs(
						static_cast<int>(curr_pair.first) -
						static_cast<int>(curr_pair.second) ) >
					opt_params.LC_min_nodeid_diff ) {

				this->logFmt(mrpt::utils::LVL_DEBUG, "Registering loop closure... ");
				is_loop_closure = true;
				break; // no need for more iterations
			}
		}
	}

	// update the pair_nodes_to_edge map
	opt_params.last_pair_nodes_to_edge = curr_pair_nodes_to_edge;
	return is_loop_closure;

	MRPT_END;
}

	template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::levMarqFeedback(
		const GRAPH_t &graph,
		const size_t iter,
		const size_t max_iter,
		const double cur_sq_error )
{ }

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::getNearbyNodesOf(
		std::set<mrpt::utils::TNodeID> *nodes_set,
		const mrpt::utils::TNodeID& cur_nodeID,
		double distance ) {
	MRPT_START;

	if (distance > 0) {
		// check all but the last node.
		for (mrpt::utils::TNodeID nodeID = 0; nodeID < m_graph->nodeCount()-1; ++nodeID) {
			double curr_distance = m_graph->nodes[nodeID].distanceTo(
					m_graph->nodes[cur_nodeID]);
			if (curr_distance <= distance) {
				nodes_set->insert(nodeID);
			}
		}
	}
	else { // check against all nodes
		m_graph->getAllNodes(*nodes_set);
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::printParams() const {
	opt_params.dumpToConsole();
	viz_params.dumpToConsole();
}
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	using namespace mrpt::utils;

	opt_params.loadFromConfigFileName(source_fname, "OptimizerParameters");
	viz_params.loadFromConfigFileName(source_fname, "VisualizationParameters");

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	// Minimum verbosity level of the logger
	int min_verbosity_level = source.read_int(
			"OptimizerParameters",
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	this->logFmt(mrpt::utils::LVL_DEBUG, "Successfully loaded Params. ");
	m_has_read_config = true;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "Levenberg Marquardt Optimization Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}


// OptimizationParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CLevMarqGSO<GRAPH_t>::OptimizationParams::OptimizationParams():
	optimization_distance_color(0, 201, 87),
	keystroke_optimization_distance("u")
{ }
template<class GRAPH_t>
CLevMarqGSO<GRAPH_t>::OptimizationParams::~OptimizationParams() {
}
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::OptimizationParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ Levenberg-Marquardt Optimization ]------------------\n");
	out.printf("Optimization on second thread  = %s\n",
			optimization_on_second_thread ? "TRUE" : "FALSE");
	out.printf("Optimize nodes in distance     = %.2f\n", optimization_distance);
	out.printf("Min. node difference for LC    = %d\n", LC_min_nodeid_diff);

	out.printf("%s", cfg.getAsString().c_str());
	std::cout << std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::OptimizationParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {
	MRPT_START;
	optimization_on_second_thread = source.read_bool(
			section,
			"optimization_on_second_thread",
			false, false);
	LC_min_nodeid_diff = source.read_int(
			"GeneralConfiguration",
			"LC_min_nodeid_diff",
			30, false);
	optimization_distance = source.read_double(
			section,
			"optimization_distance",
			5, false);
	// asert the previous value
	ASSERTMSG_(optimization_distance == -1 ||
			optimization_distance > 0,
			format("Invalid value for optimization distance: %.2f",
				optimization_distance) );

	// optimization parameters
	cfg["verbose"] = source.read_bool(
			section,
			"verbose",
			0, false);
	cfg["profiler"] = source.read_bool(
			section,
			"profiler",
			0, false);
	cfg["max_iterations"] = source.read_double(
			section,
			"max_iterations",
			100, false);
	cfg["scale_hessian"] = source.read_double(
			"Optimization",
			"scale_hessian",
			0.2, false);
	cfg["tau"] = source.read_double(
			section,
			"tau",
			1e-3, false);

	MRPT_END;
}

// GraphVisualizationParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CLevMarqGSO<GRAPH_t>::GraphVisualizationParams::GraphVisualizationParams():
	keystroke_graph_toggle("s"),
	keystroke_graph_autofit("a")
{
}
template<class GRAPH_t>
CLevMarqGSO<GRAPH_t>::GraphVisualizationParams::~GraphVisualizationParams() {
}
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::GraphVisualizationParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("-----------[ Graph Visualization Parameters ]-----------\n");
	out.printf("Visualize optimized graph = %s\n",
			visualize_optimized_graph ? "TRUE" : "FALSE");

	out.printf("%s", cfg.getAsString().c_str());

	std::cout << std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CLevMarqGSO<GRAPH_t>::GraphVisualizationParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {
	MRPT_START;
	using namespace utils;

	visualize_optimized_graph = source.read_bool(
			section,
			"visualize_optimized_graph",
			1, false);

	cfg["show_ID_labels"] = source.read_bool(
			section,
			"optimized_show_ID_labels",
			0, false);
	cfg["show_ground_grid"] = source.read_double(
			section,
			"optimized_show_ground_grid",
			1, false);
	cfg["show_edges"] = source.read_bool(
			section,
			"optimized_show_edges",
			1, false);
	cfg["edge_color"] = source.read_int(
			section,
			"optimized_edge_color",
			4286611456, false);
	cfg["edge_width"] = source.read_double(
			section,
			"optimized_edge_width",
			1.5, false);
	cfg["show_node_corners"] = source.read_bool(
			section,
			"optimized_show_node_corners",
			1, false);
	cfg["show_edge_rel_poses"] = source.read_bool(
			section,
			"optimized_show_edge_rel_poses",
			1, false);
	cfg["edge_rel_poses_color"] = source.read_int(
			section,
			"optimized_edge_rel_poses_color",
			1090486272, false);
	cfg["nodes_edges_corner_scale"] = source.read_double(
			section,
			"optimized_nodes_edges_corner_scale",
			0.4, false);
	cfg["nodes_corner_scale"] = source.read_double(
			section,
			"optimized_nodes_corner_scale",
			0.7, false);
	cfg["point_size"] = source.read_int(
			section,
			"optimized_point_size",
			0, false);
	cfg["point_color"] = source.read_int(
			section,
			"optimized_point_color",
			10526880, false);

	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CLEVMARQGSO_IMPL_H */
