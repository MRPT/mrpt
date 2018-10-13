/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

namespace mrpt::graphslam::optimizers
{
template <class GRAPH_T>
CLevMarqGSO<GRAPH_T>::CLevMarqGSO()
	: m_optimization_policy(OptimizationPolicy::UseLoopClosures)

{
	this->initializeLoggers("CLevMarqGSO");
}

template <class GRAPH_T>
bool CLevMarqGSO<GRAPH_T>::updateState(
	mrpt::obs::CActionCollection::Ptr action,
	mrpt::obs::CSensoryFrame::Ptr observations,
	mrpt::obs::CObservation::Ptr observation)
{
	MRPT_START;
	if (this->m_graph->nodeCount() > m_last_total_num_of_nodes)
	{
		m_last_total_num_of_nodes = this->m_graph->nodeCount();
		registered_new_node = true;

		if (m_first_time_call)
		{
			opt_params.last_pair_nodes_to_edge = this->m_graph->edges;
			m_first_time_call = true;
		}

		if (opt_params.optimization_on_second_thread)
		{
			// join the previous optimization thread
			m_thread_optimize.join();

			// optimize the graph - run on a seperate thread
			m_thread_optimize = std::thread(&CLevMarqGSO::optimizeGraph, this);
		}
		else
		{  // single threaded implementation
			bool is_full_update = this->checkForFullOptimization();
			this->_optimizeGraph(is_full_update);
		}
	}

	return true;
	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::initializeVisuals()
{
	MRPT_START;
	ASSERTDEB_(m_has_read_config);
	parent::initializeVisuals();

	this->initGraphVisualization();
	this->initOptDistanceVisualization();

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::updateVisuals()
{
	MRPT_START;
	parent::updateVisuals();

	if (opt_params.optimization_distance > 0)
	{
		this->updateOptDistanceVisualization();
	}

	this->updateGraphVisualization();

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::notifyOfWindowEvents(
	const std::map<std::string, bool>& events_occurred)
{
	MRPT_START;
	using namespace std;
	parent::notifyOfWindowEvents(events_occurred);

	// I know the keys exists - I registered them explicitly

	// optimization_distance toggling
	if (opt_params.optimization_distance > 0)
	{
		if (events_occurred.find(opt_params.keystroke_optimization_distance)
				->second)
		{
			this->toggleOptDistanceVisualization();
		}

		if (events_occurred.find(opt_params.keystroke_optimize_graph)->second)
		{
			this->_optimizeGraph(/*is_full_update=*/true);
		}
	}

	// graph toggling
	if (events_occurred.find(viz_params.keystroke_graph_toggle)->second)
	{
		this->toggleGraphVisualization();
	}

	// if mouse event, let the user decide about the camera
	if (events_occurred.find("mouse_clicked")->second)
	{
		MRPT_LOG_DEBUG_STREAM("Mouse was clicked. Disabling autozoom.");
		m_autozoom_active = false;
	}

	// autofit the graph once
	if (events_occurred.find(viz_params.keystroke_graph_autofit)->second)
	{
		MRPT_LOG_DEBUG_STREAM("Autofit button was pressed");
		this->fitGraphInView();
	}

	MRPT_END;
}  // end of notifyOfWindowEvents

template <class GRAPH_T>
inline void CLevMarqGSO<GRAPH_T>::initGraphVisualization()
{
	MRPT_START;
	ASSERTDEBMSG_(this->m_win_manager, "No CWindowManager* is given");

	if (viz_params.visualize_optimized_graph)
	{
		this->m_win_observer->registerKeystroke(
			viz_params.keystroke_graph_toggle, "Toggle Graph visualization");
		this->m_win_observer->registerKeystroke(
			viz_params.keystroke_graph_autofit, "Fit Graph in view");

		this->m_win_manager->assignTextMessageParameters(
			/* offset_y*	= */ &viz_params.offset_y_graph,
			/* text_index* = */ &viz_params.text_index_graph);
	}

	MRPT_END;
}
template <class GRAPH_T>
inline void CLevMarqGSO<GRAPH_T>::updateGraphVisualization()
{
	MRPT_START;
	ASSERTDEBMSG_(this->m_win_manager, "No CWindowManager* is given");
	using namespace mrpt::opengl;

	this->logFmt(
		mrpt::system::LVL_DEBUG, "In the updateGraphVisualization function");

	// update the graph (clear and rewrite..)
	COpenGLScene::Ptr& scene = this->m_win->get3DSceneAndLock();

	// remove previous graph and insert the new instance
	// TODO - make this an incremental proocecure
	CRenderizable::Ptr prev_object = scene->getByName("optimized_graph");
	bool prev_visibility = true;
	if (prev_object)
	{  // set the visibility of the graph correctly
		prev_visibility = prev_object->isVisible();
	}
	scene->removeObject(prev_object);

	// CSetOfObjects::Ptr graph_obj =
	// graph_tools::graph_visualize(*this->m_graph, viz_params.cfg);
	CSetOfObjects::Ptr graph_obj = mrpt::make_aligned_shared<CSetOfObjects>();
	this->m_graph->getAs3DObject(graph_obj, viz_params.cfg);

	graph_obj->setName("optimized_graph");
	graph_obj->setVisibility(prev_visibility);
	scene->insert(graph_obj);
	this->m_win->unlockAccess3DScene();

	this->m_win_manager->addTextMessage(
		5, -viz_params.offset_y_graph,
		format(
			"Optimized Graph: #nodes %d",
			static_cast<int>(this->m_graph->nodeCount())),
		mrpt::img::TColorf(0.0, 0.0, 0.0),
		/* unique_index = */ viz_params.text_index_graph);

	this->m_win->forceRepaint();

	if (m_autozoom_active)
	{
		this->fitGraphInView();
	}

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::toggleGraphVisualization()
{
	MRPT_START;
	using namespace mrpt::opengl;

	COpenGLScene::Ptr& scene = this->m_win->get3DSceneAndLock();

	CRenderizable::Ptr graph_obj = scene->getByName("optimized_graph");
	graph_obj->setVisibility(!graph_obj->isVisible());

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::fitGraphInView()
{
	MRPT_START;
	using namespace mrpt::opengl;

	ASSERTDEBMSG_(
		this->m_win,
		"\nVisualization of data was requested but no CDisplayWindow3D pointer "
		"was given\n");

	// first fetch the graph object
	COpenGLScene::Ptr& scene = this->m_win->get3DSceneAndLock();
	CRenderizable::Ptr obj = scene->getByName("optimized_graph");
	CSetOfObjects::Ptr graph_obj =
		std::dynamic_pointer_cast<CSetOfObjects>(obj);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	// autofit it based on its grid
	CGridPlaneXY::Ptr obj_grid =
		graph_obj->CSetOfObjects::getByClass<CGridPlaneXY>();
	if (obj_grid)
	{
		float x_min, x_max, y_min, y_max;
		obj_grid->getPlaneLimits(x_min, x_max, y_min, y_max);
		const float z_min = obj_grid->getPlaneZcoord();
		this->m_win->setCameraPointingToPoint(
			0.5 * (x_min + x_max), 0.5 * (y_min + y_max), z_min);
		this->m_win->setCameraZoom(
			2.0f * std::max(10.0f, std::max(x_max - x_min, y_max - y_min)));
	}
	this->m_win->setCameraAzimuthDeg(60);
	this->m_win->setCameraElevationDeg(75);
	this->m_win->setCameraProjective(true);

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::initOptDistanceVisualization()
{
	MRPT_START;
	using namespace mrpt::opengl;

	if (opt_params.optimization_distance > 0)
	{
		this->m_win_observer->registerKeystroke(
			opt_params.keystroke_optimization_distance,
			"Toggle optimization distance on/off");

		this->m_win_observer->registerKeystroke(
			opt_params.keystroke_optimize_graph,
			"Manually trigger a full graph optimization");
	}

	pose_t p;
	CRenderizable::Ptr obj = this->initOptDistanceVisualizationInternal(p);
	pose_t initial_pose;
	obj->setPose(initial_pose);
	obj->setName("optimization_distance_obj");

	COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();
	scene->insert(obj);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	// optimization distance disk - textMessage
	this->m_win_manager->assignTextMessageParameters(
		&opt_params.offset_y_optimization_distance,
		&opt_params.text_index_optimization_distance);

	this->m_win_manager->addTextMessage(
		5, -opt_params.offset_y_optimization_distance,
		format("Radius for graph optimization"),
		mrpt::img::TColorf(opt_params.optimization_distance_color),
		/* unique_index = */ opt_params.text_index_optimization_distance);
	MRPT_END;
}

template <class GRAPH_T>
mrpt::opengl::CRenderizable::Ptr
	CLevMarqGSO<GRAPH_T>::initOptDistanceVisualizationInternal(
		const mrpt::poses::CPose2D& p_unused)
{
	using namespace mrpt::opengl;

	CDisk::Ptr obj = mrpt::make_aligned_shared<CDisk>();
	obj->setDiskRadius(
		opt_params.optimization_distance,
		opt_params.optimization_distance - 0.1);
	obj->setColor_u8(opt_params.optimization_distance_color);

	return obj;
}
template <class GRAPH_T>
mrpt::opengl::CRenderizable::Ptr
	CLevMarqGSO<GRAPH_T>::initOptDistanceVisualizationInternal(
		const mrpt::poses::CPose3D& p_unused)
{
	using namespace mrpt::opengl;

	CSphere::Ptr obj = mrpt::make_aligned_shared<CSphere>();
	obj->setRadius(opt_params.optimization_distance);
	obj->setColor_u8(
		opt_params.optimization_distance_color.R,
		opt_params.optimization_distance_color.G,
		opt_params.optimization_distance_color.B,
		/*alpha = */ 60);

	return obj;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::updateOptDistanceVisualization()
{
	MRPT_START;
	ASSERTDEBMSG_(this->m_win_manager, "No CWindowManager* is given");
	using namespace mrpt::opengl;

	// update ICP_max_distance Disk
	COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

	CRenderizable::Ptr obj = scene->getByName("optimization_distance_obj");
	obj->setPose(this->m_graph->nodes.rbegin()->second);

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();
	MRPT_END;
}

// TODO - implement this
template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::toggleOptDistanceVisualization()
{
	MRPT_START;
	using namespace mrpt::opengl;

	COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

	CRenderizable::Ptr obj = scene->getByName("optimization_distance_obj");
	obj->setVisibility(!obj->isVisible());

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::optimizeGraph()
{
	MRPT_START;
	using namespace std;

	MRPT_LOG_DEBUG_STREAM(
		"optimizeGraph:: ThreadID:" << endl
									<< "\t" << std::this_thread::get_id()
									<< endl
									<< "\t"
									<< "Trying to grab lock... ");

	std::lock_guard<std::mutex> graph_lock(*this->m_graph_section);
	this->_optimizeGraph();

	this->logFmt(mrpt::system::LVL_DEBUG, "2nd thread grabbed the lock..");

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::_optimizeGraph(bool is_full_update /*=false*/)
{
	MRPT_START;
	this->m_time_logger.enter("CLevMarqGSO::_optimizeGraph");

	// if less than X nodes exist overall, do not try optimizing
	if (m_min_nodes_for_optimization > this->m_graph->nodes.size())
	{
		return;
	}

	mrpt::system::CTicTac optimization_timer;
	optimization_timer.Tic();

	// set of nodes for which the optimization procedure will take place
	std::set<mrpt::graphs::TNodeID>* nodes_to_optimize;

	// fill in the nodes in certain distance to the current node, only if
	// is_full_update is not instructed
	if (is_full_update)
	{
		// nodes_to_optimize: List of nodes to optimize. nullptr -> all but the
		// root
		// node.
		nodes_to_optimize = nullptr;
	}
	else
	{
		nodes_to_optimize = new std::set<mrpt::graphs::TNodeID>;

		// I am certain that this shall not be called when nodeCount = 0, since
		// the
		// optimization procedure starts only after certain number of nodes has
		// been added
		this->getNearbyNodesOf(
			nodes_to_optimize, this->m_graph->nodeCount() - 1,
			opt_params.optimization_distance);
		nodes_to_optimize->insert(this->m_graph->nodeCount() - 1);
	}

	graphslam::TResultInfoSpaLevMarq levmarq_info;

	// Execute the optimization
	mrpt::graphslam::optimize_graph_spa_levmarq(
		*(this->m_graph), levmarq_info, nodes_to_optimize, opt_params.cfg,
		&CLevMarqGSO<GRAPH_T>::levMarqFeedback);  // functor feedback

	if (is_full_update)
	{
		m_just_fully_optimized_graph = true;
	}
	else
	{
		m_just_fully_optimized_graph = false;
	}

	double elapsed_time = optimization_timer.Tac();
	this->logFmt(
		mrpt::system::LVL_DEBUG, "Optimization of graph took: %fs",
		elapsed_time);

	// deleting the nodes_to_optimize set
	delete nodes_to_optimize;
	nodes_to_optimize = nullptr;

	this->m_time_logger.leave("CLevMarqGSO::_optimizeGraph");
	MRPT_UNUSED_PARAM(elapsed_time);
	MRPT_END;
}  // end of _optimizeGraph

template <class GRAPH_T>
bool CLevMarqGSO<GRAPH_T>::checkForLoopClosures()
{
	MRPT_START;

	bool is_loop_closure = false;
	auto curr_pair_nodes_to_edge = this->m_graph->edges;

	// find the *node pairs* that exist in current but not the last
	// nodes_to_edge
	// map If the distance of any of these pairs is greater than
	// LC_min_nodeid_diff then consider this a loop closure
	typename GRAPH_T::edges_map_t::const_iterator search;
	mrpt::graphs::TPairNodeIDs curr_pair;

	for (auto it = curr_pair_nodes_to_edge.begin();
		 it != curr_pair_nodes_to_edge.end(); ++it)
	{
		search = opt_params.last_pair_nodes_to_edge.find(it->first);
		// if current node pair is not found in the last set...
		if (search == opt_params.last_pair_nodes_to_edge.end())
		{
			curr_pair = it->first;

			if (std::abs(
					static_cast<int>(curr_pair.first) -
					static_cast<int>(curr_pair.second)) >
				opt_params.LC_min_nodeid_diff)
			{
				this->logFmt(
					mrpt::system::LVL_DEBUG, "Registering loop closure... ");
				is_loop_closure = true;
				break;  // no need for more iterations
			}
		}
	}

	// update the pair_nodes_to_edge map
	opt_params.last_pair_nodes_to_edge = curr_pair_nodes_to_edge;
	return is_loop_closure;

	MRPT_END;
}

template <class GRAPH_T>
bool CLevMarqGSO<GRAPH_T>::checkForFullOptimization()
{
	bool is_full_update = false;

	if (opt_params.optimization_distance == -1)
	{  // always optimize fully
		return true;
	}

	bool added_lc = this->checkForLoopClosures();

	// Decide on the LoopClosingAttitude I am in
	if (!added_lc)
	{  // reset both ignored and used counters
		if (m_curr_used_consec_lcs != 0 || m_curr_ignored_consec_lcs != 0)
		{
			MRPT_LOG_DEBUG_STREAM("No new Loop Closure found.");
		}

		m_curr_used_consec_lcs = 0;
		m_curr_ignored_consec_lcs = 0;
		m_optimization_policy = OptimizationPolicy::UseLoopClosures;

		return is_full_update;
	}
	else
	{  // lc found.
		// have I used enough consecutive loop closures?
		bool use_limit_reached =
			m_curr_used_consec_lcs == m_max_used_consec_lcs;
		// have I ignored enough consecutive loop closures?
		bool ignore_limit_reached =
			m_curr_ignored_consec_lcs == m_max_ignored_consec_lcs;

		// Have I reached any of the limits above?
		if (ignore_limit_reached || use_limit_reached)
		{
			m_curr_ignored_consec_lcs = 0;
			m_curr_used_consec_lcs = 0;

			// decide of the my policy on full optimization
			if (ignore_limit_reached)
			{
				m_optimization_policy = OptimizationPolicy::UseLoopClosures;
			}
			if (use_limit_reached)
			{
				m_optimization_policy = OptimizationPolicy::IgnoreLoopClosures;
			}
		}
		else
		{  // no limits reached yet.
			if (m_optimization_policy == OptimizationPolicy::UseLoopClosures)
			{
				m_curr_used_consec_lcs += 1;
			}
			else
			{
				m_curr_ignored_consec_lcs += 1;
			}
		}
	}

	// Decide on whether to fully optimize the graph based on the mode I am in
	if (m_optimization_policy == OptimizationPolicy::IgnoreLoopClosures)
	{
		is_full_update = false;
		MRPT_LOG_WARN_STREAM(
			"*PARTIAL* graph optimization.. ignoring new loop closure");
	}
	else
	{
		is_full_update = true;
		MRPT_LOG_WARN_STREAM("Commencing with *FULL* graph optimization... ");
	}
	return is_full_update;

}  // end of checkForFullOptimization

template <class GRAPH_T>
bool CLevMarqGSO<GRAPH_T>::justFullyOptimizedGraph() const
{
	return m_just_fully_optimized_graph;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::levMarqFeedback(
	const GRAPH_T& graph, const size_t iter, const size_t max_iter,
	const double cur_sq_error)
{
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::getNearbyNodesOf(
	std::set<mrpt::graphs::TNodeID>* nodes_set,
	const mrpt::graphs::TNodeID& cur_nodeID, double distance)
{
	MRPT_START;

	if (distance > 0)
	{
		// check all but the last node.
		for (mrpt::graphs::TNodeID nodeID = 0;
			 nodeID < this->m_graph->nodeCount() - 1; ++nodeID)
		{
			double curr_distance = this->m_graph->nodes[nodeID].distanceTo(
				this->m_graph->nodes[cur_nodeID]);
			if (curr_distance <= distance)
			{
				nodes_set->insert(nodeID);
			}
		}
	}
	else
	{  // check against all nodes
		this->m_graph->getAllNodes(*nodes_set);
	}

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::printParams() const
{
	parent::printParams();

	opt_params.dumpToConsole();
	viz_params.dumpToConsole();
}
template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::loadParams(const std::string& source_fname)
{
	MRPT_START;
	parent::loadParams(source_fname);

	opt_params.loadFromConfigFileName(source_fname, "OptimizerParameters");
	viz_params.loadFromConfigFileName(source_fname, "VisualizationParameters");

	mrpt::config::CConfigFile source(source_fname);

	// TODO - check that these work
	m_max_used_consec_lcs = source.read_int(
		"OptimizerParameters", "max_used_consecutive_loop_closures", 2, false);

	m_max_ignored_consec_lcs = source.read_int(
		"OptimizerParameters", "max_ignored_consecutive_loop_closures", 15,
		false);

	// set the logging level if given by the user
	// Minimum verbosity level of the logger
	int min_verbosity_level =
		source.read_int("OptimizerParameters", "class_verbosity", 1, false);
	this->setMinLoggingLevel(mrpt::system::VerbosityLevel(min_verbosity_level));

	MRPT_LOG_DEBUG("Successfully loaded Params. ");
	m_has_read_config = true;

	MRPT_END;
}

template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::getDescriptiveReport(std::string* report_str) const
{
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "Levenberg Marquardt Optimization Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent::getDescriptiveReport(report_str);

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}

template <class GRAPH_T>
CLevMarqGSO<GRAPH_T>::OptimizationParams::OptimizationParams()
	: optimization_distance_color(0, 201, 87),
	  keystroke_optimization_distance("u"),
	  keystroke_optimize_graph("w")
{
}
template <class GRAPH_T>
CLevMarqGSO<GRAPH_T>::OptimizationParams::~OptimizationParams() = default;
template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::OptimizationParams::dumpToTextStream(
	std::ostream& out) const
{
	MRPT_START;
	out << "-----------[ Levenberg-Marquardt Optimization ] -------\n";
	out << "Optimization on second thread  = "
		<< (optimization_on_second_thread ? "TRUE" : "FALSE") << std::endl;
	out << "Optimize nodes in distance     = " << optimization_distance << "\n";
	out << "Min. node difference for LC    = " << LC_min_nodeid_diff << "\n";
	out << cfg.getAsString() << std::endl;
	MRPT_END;
}
template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::OptimizationParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_START;
	optimization_on_second_thread = source.read_bool(
		section, "optimization_on_second_thread", false, false);
	LC_min_nodeid_diff = source.read_int(
		"GeneralConfiguration", "LC_min_nodeid_diff", 30, false);
	optimization_distance =
		source.read_double(section, "optimization_distance", 5, false);
	// asert the previous value
	ASSERTDEBMSG_(
		optimization_distance == 1 || optimization_distance > 0,
		format(
			"Invalid value for optimization distance: %.2f",
			optimization_distance));

	// optimization parameters
	cfg["verbose"] = source.read_bool(section, "verbose", false, false);
	cfg["profiler"] = source.read_bool(section, "profiler", false, false);
	cfg["max_iterations"] =
		source.read_double(section, "max_iterations", 100, false);
	cfg["scale_hessian"] =
		source.read_double("Optimization", "scale_hessian", 0.2, false);
	cfg["tau"] = source.read_double(section, "tau", 1e-3, false);

	MRPT_END;
}

template <class GRAPH_T>
CLevMarqGSO<GRAPH_T>::GraphVisualizationParams::GraphVisualizationParams()
	: keystroke_graph_toggle("s"), keystroke_graph_autofit("a")
{
}
template <class GRAPH_T>
CLevMarqGSO<GRAPH_T>::GraphVisualizationParams::~GraphVisualizationParams() =
	default;
template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::GraphVisualizationParams::dumpToTextStream(
	std::ostream& out) const
{
	MRPT_START;

	out << mrpt::format(
		"-----------[ Graph Visualization Parameters ]-----------\n");
	out << mrpt::format(
		"Visualize optimized graph = %s\n",
		visualize_optimized_graph ? "TRUE" : "FALSE");

	out << mrpt::format("%s", cfg.getAsString().c_str());

	std::cout << std::endl;

	MRPT_END;
}
template <class GRAPH_T>
void CLevMarqGSO<GRAPH_T>::GraphVisualizationParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_START;

	visualize_optimized_graph =
		source.read_bool(section, "visualize_optimized_graph", true, false);

	cfg["show_ID_labels"] =
		source.read_bool(section, "optimized_show_ID_labels", false, false);
	cfg["show_ground_grid"] =
		source.read_double(section, "optimized_show_ground_grid", 1, false);
	cfg["show_edges"] =
		source.read_bool(section, "optimized_show_edges", true, false);
	cfg["edge_color"] =
		source.read_int(section, "optimized_edge_color", 1500, false);
	cfg["edge_width"] =
		source.read_double(section, "optimized_edge_width", 1.5, false);
	cfg["show_node_corners"] =
		source.read_bool(section, "optimized_show_node_corners", true, false);
	cfg["show_edge_rel_poses"] =
		source.read_bool(section, "optimized_show_edge_rel_poses", true, false);
	cfg["edge_rel_poses_color"] =
		source.read_int(section, "optimized_edge_rel_poses_color", 2000, false);
	cfg["nodes_edges_corner_scale"] = source.read_double(
		section, "optimized_nodes_edges_corner_scale", 0.4, false);
	cfg["nodes_corner_scale"] =
		source.read_double(section, "optimized_nodes_corner_scale", 0.7, false);
	cfg["nodes_point_size"] =
		source.read_int(section, "optimized_nodes_point_size", 5, false);
	cfg["nodes_point_color"] =
		source.read_int(section, "optimized_nodes_point_color", 3000, false);

	MRPT_END;
}
}  // namespace mrpt::graphslam::optimizers
