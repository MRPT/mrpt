/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLEVMARQGSO_IMPL_H
#define CLEVMARQGSO_IMPL_H

using namespace mrpt::graphslam::optimizers;

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::CLevMarqGSO_t()
{
	MRPT_START;

	this->initCLevMarqGSO_t();

	MRPT_END;
}
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::~CLevMarqGSO_t() {
	MRPT_START;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::initCLevMarqGSO_t() {
	MRPT_START;

	m_graph = NULL;
	m_win_manager = NULL;
	m_win_observer = NULL;
	m_graph_section = NULL;


	m_first_time_call = false;
	m_initialized_visuals = false;
	m_has_read_config = false;
	m_last_total_num_of_nodes = 5;

	MRPT_END;
}

// Member function implementations
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
bool CLevMarqGSO_t<GRAPH_t>::updateOptimizerState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	//cout << "[CLevMarqGSO:] In updateOptimizerState... " << endl;
	
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
					/* func = */ &CLevMarqGSO_t::optimizeGraph);

		}
		else { // single threaded implementation
			this->_optimizeGraph();
		}

	}

	return false;
	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;

	m_graph = graph;

	std::cout << "[CLevMarqGSO:] Fetched the graph successfully"
		<< std::endl;

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setRawlogFname(const std::string& rawlog_fname) {
	MRPT_START;

	MRPT_END;
}


template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setWindowManagerPtr(
		mrpt::gui::CWindowManager_t* win_manager) {
	MRPT_START;

	m_win_manager = win_manager;
	if (m_win_manager) {
		m_win = m_win_manager->win;

		m_win_observer = m_win_manager->observer;

	}
	std::cout << "[CLevMarqGSO:] Fetched the CDisplayWindow successfully"
		<< std::endl;

	MRPT_END;
}

template <class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::setCriticalSectionPtr(
		mrpt::synch::CCriticalSection* graph_section) {
	MRPT_START;

	m_graph_section = graph_section;

	std::cout << "[CLevMarqGSO:] Fetched the CCRiticalSection successfully"
		<< std::endl;
	MRPT_END;
}


template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	std::cout << "CLevMarqGSO:] Initializing visuals" << std::endl;

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
void CLevMarqGSO_t<GRAPH_t>::updateVisuals() {
	MRPT_START;

	ASSERT_(m_initialized_visuals);
	ASSERTMSG_(m_win_manager, "No CWindowManager* is given");
	ASSERTMSG_(m_win,
			"Visualization of data was requested but no CDisplayWindow3D pointer "
			" was given.");

	this->updateOptDistanceVisualization();
	this->updateGraphVisualization();

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool> events_occurred)
{
	MRPT_START;

	// I know the keys exists - I put it there explicitly

	// graph toggling
	if (events_occurred.find(viz_params.keystroke_graph)->second) {
		this->toggleGraphVisualization();
	}

	// optimization_distance toggling
	if (opt_params.optimization_distance > 0) {
		if (events_occurred.find(
					opt_params.keystroke_optimization_distance)->second) {
			this->toggleOptDistanceVisualization();
		}
	}

	MRPT_END;
}

template<class GRAPH_t>
inline void CLevMarqGSO_t<GRAPH_t>::initGraphVisualization() {
	MRPT_START;

	m_win_observer->registerKeystroke("s", "Toggle Graph visualization");

	if (viz_params.visualize_optimized_graph) {
		m_win_manager->assignTextMessageParameters(
				/* offset_y*	= */ &viz_params.offset_y_graph,
				/* text_index* = */ &viz_params.text_index_graph );
	}


	MRPT_END;
}
template<class GRAPH_t>
inline void CLevMarqGSO_t<GRAPH_t>::updateGraphVisualization() {
	MRPT_START;

	std::cout << "[CLevMarqGSO:] In the updateGraphVisualization function" 
		<< std::endl;

	// update the graph (clear and rewrite..)
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	// remove previous graph and insert the its new instance
	CRenderizablePtr prev_object = scene->getByName("optimized_graph");
	bool prev_visibility = true;
	if (prev_object) {
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

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::toggleGraphVisualization() {
	MRPT_START;

	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	CRenderizablePtr graph_obj = scene->getByName("optimized_graph");
	graph_obj->setVisibility(!graph_obj->isVisible());

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::initOptDistanceVisualization() {
	MRPT_START;

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
void CLevMarqGSO_t<GRAPH_t>::updateOptDistanceVisualization() {
	MRPT_START;

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
void CLevMarqGSO_t<GRAPH_t>::toggleOptDistanceVisualization() {
	MRPT_START;

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	CRenderizablePtr obj = scene->getByName("optimization_distance_disk");
	obj->setVisibility(!obj->isVisible());

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	
	MRPT_END;
}



template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::optimizeGraph() {
	MRPT_START;

	std::cout << "[CLevMarqGSO:] In optimizeGraph" << std::endl;
	std::cout << "\tThreadID: " << mrpt::system::getCurrentThreadId() << std::endl;
	std::cout << "\tTrying to grab lock... " << std::endl;

	mrpt::synch::CCriticalSectionLocker m_graph_lock(m_graph_section);
	this->_optimizeGraph();

	std::cout << "\t2nd thread grabbed the lock.." << std::endl;

	MRPT_END;
}

// TODO - do something meaningful with these parameters
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::_optimizeGraph() {
	MRPT_START;
	std::cout << "[CLevMarqGSO:] In _optimizeGraph" << std::endl;

	CTicTac optimization_timer;
	optimization_timer.Tic();

 	
	// set of nodes for which the optimization procedure will take place
 	std::set< mrpt::utils::TNodeID>* nodes_to_optimize;

 	// fill in the nodes in certain distance to the current node, only if
 	// full_update is not instructed

 	bool full_update = opt_params.optimization_distance == -1 || 
 		this->checkForLoopClosures();
 	if (full_update) {
 		nodes_to_optimize = NULL;
		std::cout << "[CLevMarqGSO:] Commencing with FULL graph optimization... "
			<< std::endl;
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

		std::cout << "Nodes to optimize: # " << nodes_to_optimize->size() << std::endl;
		std::cout << "\t< ";
		for (std::set<mrpt::utils::TNodeID>::const_iterator it = 
				nodes_to_optimize->begin(); it != nodes_to_optimize->end(); ++it) {
			std::cout << *it << ", ";
		}
		std::cout << " >" << std::endl;

 	}

	graphslam::TResultInfoSpaLevMarq	levmarq_info;

	// Execute the optimization
	mrpt::graphslam::optimize_graph_spa_levmarq(
			*m_graph,
			levmarq_info,
			nodes_to_optimize,  // List of nodes to optimize. NULL -> all but the root node.
			opt_params.cfg,
			&CLevMarqGSO_t<GRAPH_t>::levMarqFeedback); // functor feedback

	double elapsed_time = optimization_timer.Tac();
	std::cout << "Optimization of graph took: " << elapsed_time << "s" 
		<< std::endl;

	// deleting the nodes_to_optimize set
	delete nodes_to_optimize;
	nodes_to_optimize = NULL;

	MRPT_UNUSED_PARAM(elapsed_time);
	MRPT_END;
}

template<class GRAPH_t>
bool CLevMarqGSO_t<GRAPH_t>::checkForLoopClosures() {
	MRPT_START;

	bool is_loop_closure = false;
	typename GRAPH_t::edges_map_t curr_pair_nodes_to_edge =  m_graph->edges;

	//// print it for verification reasons
	//std::cout << "Last Edges: #" << opt_params.last_pair_nodes_to_edge.size()
		//<< std::endl;
	//std::cout << "Current Edges: #" << curr_pair_nodes_to_edge.size()
		//<< std::endl;

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
			//std::cout << "- New node pair was detected: " << curr_pair.first 
				 //<< " => " << curr_pair.second << std::endl;

			if (std::abs(
						static_cast<int>(curr_pair.first) - 
						static_cast<int>(curr_pair.second) ) > 
			 	 	opt_params.LC_min_nodeid_diff ) {

				std::cout << "[CLevMarq:] Registering loop closure... " << std::endl;
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
void CLevMarqGSO_t<GRAPH_t>::levMarqFeedback(
		const GRAPH_t &graph,
		const size_t iter,
		const size_t max_iter,
		const double cur_sq_error )
{ }

template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::getNearbyNodesOf(
		std::set<mrpt::utils::TNodeID> *nodes_set,
		const mrpt::utils::TNodeID& cur_nodeID,
		double distance ) {
	MRPT_START;

	if (distance > 0) {
		// check all but the last node.
		for (mrpt::utils::TNodeID nodeID = 0; nodeID < m_graph->nodeCount()-1; ++nodeID) {
			double curr_distance = m_graph->nodes[nodeID].distanceTo(
					m_graph->nodes[cur_nodeID]);
			//std::cout << "testing against node: " << nodeID << std::endl;
			//std::cout << "\tcurr_distance: " << curr_distance << std::endl;
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
void CLevMarqGSO_t<GRAPH_t>::printParams() const {
	opt_params.dumpToConsole();
	viz_params.dumpToConsole();
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::loadParams(const std::string& source_fname) {
	opt_params.loadFromConfigFileName(source_fname, "OptimizerParameters");
	viz_params.loadFromConfigFileName(source_fname, "VisualizationParameters");

	m_has_read_config = true;
}


// OptimizationParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::OptimizationParams::OptimizationParams():
	optimization_distance_color(0, 201, 87),
	keystroke_optimization_distance("u")
{ }
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::OptimizationParams::~OptimizationParams() {
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::OptimizationParams::dumpToTextStream(
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
void CLevMarqGSO_t<GRAPH_t>::OptimizationParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;
  optimization_on_second_thread = source.read_bool(
  		section,
  		"optimization_on_second_thread",
  		1, false);
  LC_min_nodeid_diff = source.read_int(
 			"GeneralConfiguration",
 			"LC_min_nodeid_diff",
 			30, false);
  optimization_distance = source.read_double(
  		section,
  		"optimization_distance",
  		-1, false);
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
			200, false);
	cfg["scale_hessian"] = source.read_double(
			"Optimization",
			"scale_hessian",
			0.2, false);
	cfg["tau"] = source.read_double(
			section,
			"tau",
			1e-3, false);

	std::cout << "[CLevMarqGSO:] Successfully loaded Optimization parameters. "
		<< std::endl;

	MRPT_END;
}

// GraphVisualizationParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::GraphVisualizationParams():
	keystroke_graph("s")
{
}
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::~GraphVisualizationParams() {
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::dumpToTextStream(
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
void CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;

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

	std::cout << "[LevMarqGSO:] Successfully loaded GraphVisualization Params. "
		<< std::endl;

	MRPT_END;
}


#endif /* end of include guard: CLEVMARQGSO_IMPL_H */
