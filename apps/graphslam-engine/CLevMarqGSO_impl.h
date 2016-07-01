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
CLevMarqGSO_t<GRAPH_t>::CLevMarqGSO_t() {
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
	m_graph_section = NULL;

	m_initialized_visuals = false;
	m_just_inserted_loop_closure = false;
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

		 // join the previous optimization thread
		 mrpt::system::joinThread(m_thread_optimize);
 
		 // optimize the graph - run on a seperate thread
		 m_thread_optimize = createThreadFromObjectMethod(
				 /*obj = */ this,
				 /* func = */ &CLevMarqGSO_t::optimizeGraph );
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

	if (viz_params.visualize_optimized_graph) {
		m_win_manager->assignTextMessageParameters(
				/* offset_y*	= */ &viz_params.offset_y_graph,
				/* text_index* = */ &viz_params.text_index_graph );
	}

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

	this->updateGraphVisualization();

	MRPT_END;
}

// TODO - implement this
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool> events_occurred)
{

}
template<class GRAPH_t>
inline void CLevMarqGSO_t<GRAPH_t>::updateGraphVisualization() {
	MRPT_START;

	//std::cout << "[CLevMarqGSO:] In the updateGraphVisualization function" 
		//<< std::endl;

	// update the graph (clear and rewrite..)
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	// remove previous graph and insert the its new instance
	CRenderizablePtr prev_object = scene->getByName("optimized_graph");
	scene->removeObject(prev_object);

	CSetOfObjectsPtr graph_obj = graph_tools::graph_visualize(*m_graph, viz_params.cfg);
	graph_obj->setName("optimized_graph");
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

// TODO - do something meaningful with these parameters
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::optimizeGraph() {
	MRPT_START;
	std::cout << "[CLevMarqGSO:] In optimizeGraph method... " << std::endl;

	CTicTac optimization_timer;
	optimization_timer.Tic();

	graphslam::TResultInfoSpaLevMarq	levmarq_info;

	{
		mrpt::synch::CCriticalSectionLocker m_graph_lock(m_graph_section);
		// Execute the optimization
		mrpt::graphslam::optimize_graph_spa_levmarq(
				*m_graph,
				levmarq_info,
				NULL,  // List of nodes to optimize. NULL -> all but the root node.
				opt_params.cfg,
				&CLevMarqGSO_t<GRAPH_t>::levMarqFeedback); // functor feedback
	}

	double elapsed_time = optimization_timer.Tac();
	//VERBOSE_COUT << "Optimization of graph took: " << elapsed_time << "s" << std::endl;

	MRPT_UNUSED_PARAM(elapsed_time);
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
void CLevMarqGSO_t<GRAPH_t>::printParams() const {
	opt_params.dumpToConsole();
	viz_params.dumpToConsole();
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::loadParams(const std::string& source_fname) {
	opt_params.loadFromConfigFileName(source_fname, "OptimizerParameters");
	viz_params.loadFromConfigFileName(source_fname, "VisualizationParameters");
}


// OptimizationParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::OptimizationParams::OptimizationParams() {
}
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::OptimizationParams::~OptimizationParams() {
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::OptimizationParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ Levenberg-Marquardt Optimization ]------------------\n");
	cfg.dumpToConsole();

	std::cout << std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::OptimizationParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;

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
CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::GraphVisualizationParams() {
}
template<class GRAPH_t>
CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::~GraphVisualizationParams() {
}
template<class GRAPH_t>
void CLevMarqGSO_t<GRAPH_t>::GraphVisualizationParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	std::cout << "-----------[ Graph Visualization Parameters ]-----------" << std::endl;

	stringstream ss_out("");
	ss_out << "Visualize optimized graph = "
		<< visualize_optimized_graph << std::endl;
	std::cout << ss_out.str() << std::endl;

	cfg.dumpToConsole();

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
