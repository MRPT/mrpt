/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIAERD_IMPL_H
#define CICPCRITERIAERD_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
// //////////////////////////////////

template<class GRAPH_t>
CICPCriteriaERD<GRAPH_t>::CICPCriteriaERD():
	params(*this), // pass reference to self when initializing the parameters
	m_search_disk_color(142, 142, 56),
	m_laser_scans_color(0, 20, 255),
	m_consecutive_invalid_format_instances_thres(20) // high threshold just to make sure
{
	MRPT_START;

	this->initCICPCriteriaERD();

	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::initCICPCriteriaERD() {
	MRPT_START;
	using namespace mrpt::utils;

	m_win = NULL;
	m_win_manager = NULL;
	m_graph = NULL;

	m_initialized_visuals = false;
	m_just_inserted_loop_closure = false;
	m_is_using_3DScan = false;

	// start ICP constraint registration only when
	// nodeCount > m_last_total_num_of_nodes
	m_last_total_num_of_nodes = 2;

	m_edge_types_to_nums["ICP2D"] = 0;
	m_edge_types_to_nums["ICP3D"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	this->logging_enable_keep_record = true;
	this->setLoggerName("CICPCriteriaERD");
	this->logFmt(LVL_DEBUG, "Initialized class object");

	MRPT_END;
}
template<class GRAPH_t>
CICPCriteriaERD<GRAPH_t>::~CICPCriteriaERD() { }

// Methods implementations
// //////////////////////////////////

template<class GRAPH_t> bool CICPCriteriaERD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt::obs;
	using namespace mrpt::utils;

	// check possible prior node registration
	bool registered_new_node = false;

	if (m_last_total_num_of_nodes < m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = m_graph->nodeCount();
		this->logFmt(LVL_DEBUG, "New node has been registered!");
	}

	if (observation.present()) { // observation-only rawlog format
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
			m_last_laser_scan2D =
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);


			m_is_using_3DScan = false;
		}
		if (IS_CLASS(observation, CObservation3DRangeScan)) {
			m_last_laser_scan3D =
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
			// just load the range/intensity images - CGraphSlanEngine takes care
			// of the path
			m_last_laser_scan3D->load();

			// grab fake 2D range scan for visualization
			this->convert3DTo2DRangeScan(
					/*from = */ m_last_laser_scan3D,
					/*to   = */ &m_fake_laser_scan2D);


			m_is_using_3DScan = true;
		}

		// New node has been registered.
		// add the last laser_scan
		if (registered_new_node) {
			if (!m_last_laser_scan2D.null()) {
				m_nodes_to_laser_scans2D[m_graph->nodeCount()-1] = m_last_laser_scan2D;
				this->logFmt(LVL_DEBUG,
						"Added laser scans of nodeID: %lu",
						m_graph->nodeCount()-1);
			}
			if (!m_last_laser_scan3D.null()) {
				m_nodes_to_laser_scans3D[m_graph->nodeCount()-1] = m_last_laser_scan3D;
				this->logFmt(LVL_DEBUG,
						"Added laser scans of nodeID: %lu",
						m_graph->nodeCount()-1);
			}
		}
	}
	else { // action-observations rawlog format
		// append current laser scan
		m_last_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (registered_new_node && m_last_laser_scan2D) {
			m_nodes_to_laser_scans2D[m_graph->nodeCount()-1] = m_last_laser_scan2D;
		}
	}

	// edge registration procedure - same for both rawlog formats
	if (registered_new_node) {
		// get set of nodes within predefined distance for ICP
		std::set<mrpt::utils::TNodeID> nodes_to_check_ICP;
		this->getNearbyNodesOf(
				&nodes_to_check_ICP,
				m_graph->nodeCount()-1,
				params.ICP_max_distance);
		this->logFmt(LVL_DEBUG,
				"Found * %lu * nodes close to nodeID %lu",
				nodes_to_check_ICP.size(),
				m_graph->nodeCount()-1);

		// reset the loop_closure flag and run registration
		m_just_inserted_loop_closure = false;
		registered_new_node = false;

		if (m_is_using_3DScan) {
			checkRegistrationCondition3D(nodes_to_check_ICP);
		}
		else {
			checkRegistrationCondition2D(nodes_to_check_ICP);
		}
	}

	if (!m_checked_for_usuable_dataset) {
		this->checkIfInvalidDataset(action, observations, observation);
	}

	return true;
	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::checkRegistrationCondition2D(
		const std::set<mrpt::utils::TNodeID>& nodes_set) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::math;

	mrpt::utils::TNodeID curr_nodeID = m_graph->nodeCount()-1;
	CObservation2DRangeScanPtr curr_laser_scan;
	std::map<mrpt::utils::TNodeID,
		mrpt::obs::CObservation2DRangeScanPtr>::const_iterator search;

	// search for curr_laser_scan
	search = m_nodes_to_laser_scans2D.find(curr_nodeID);
	if (search != m_nodes_to_laser_scans2D.end()) {
		curr_laser_scan = search->second;
	}

	// commence only if I have the current laser scan
	if (curr_laser_scan) {
		// try adding ICP constraints with each node in the previous set
		for (std::set<mrpt::utils::TNodeID>::const_iterator
				node_it = nodes_set.begin();
				node_it != nodes_set.end(); ++node_it) {

			// get the ICP edge between current and last node
			constraint_t rel_edge;
			mrpt::slam::CICP::TReturnInfo icp_info;
			CObservation2DRangeScanPtr prev_laser_scan;

			// search for prev_laser_scan
			search = m_nodes_to_laser_scans2D.find(*node_it);
			if (search != m_nodes_to_laser_scans2D.end()) {
				prev_laser_scan = search->second;

				// make use of initial node position difference for the ICP edge
				pose_t initial_pose = m_graph->nodes[curr_nodeID] -
					m_graph->nodes[*node_it];

				m_time_logger.enter("CICPCriteriaERD::getICPEdge");
				this->getICPEdge(
						*prev_laser_scan,
						*curr_laser_scan,
						&rel_edge,
						&initial_pose,
						&icp_info);
				m_time_logger.leave("CICPCriteriaERD::getICPEdge");

				// Debugging statements
				MRPT_LOG_DEBUG_STREAM <<
			">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
				MRPT_LOG_DEBUG_STREAM <<
					"ICP constraint between NON-successive nodes: " << 
					*node_it << " => " << curr_nodeID << 
					std::endl <<
					"\tnIterations = " << icp_info.nIterations <<
					"\tgoodness = " << icp_info.goodness;
				MRPT_LOG_DEBUG_STREAM << "ICP_goodness_thresh: " <<
					params.ICP_goodness_thresh;
				MRPT_LOG_DEBUG_STREAM <<
			"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";


				// criterion for registering a new node
				if (icp_info.goodness > params.ICP_goodness_thresh) {
					this->registerNewEdge(*node_it, curr_nodeID, rel_edge);
					m_edge_types_to_nums["ICP2D"]++;
					// in case of loop closure
					if (absDiff(curr_nodeID, *node_it) >
							params.LC_min_nodeid_diff) {
						m_edge_types_to_nums["LC"]++;
						m_just_inserted_loop_closure = true;
					}
				}
			}
		}
	}

	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::checkRegistrationCondition3D(
		const std::set<mrpt::utils::TNodeID>& nodes_set) {
	MRPT_START;
	using namespace std;
	using namespace mrpt::obs;
	using namespace mrpt::math;

	mrpt::utils::TNodeID curr_nodeID = m_graph->nodeCount()-1;
	CObservation3DRangeScanPtr curr_laser_scan;
	std::map<mrpt::utils::TNodeID,
		mrpt::obs::CObservation3DRangeScanPtr>::const_iterator search;
	// search for curr_laser_scan
	search = m_nodes_to_laser_scans3D.find(curr_nodeID);
	if (search != m_nodes_to_laser_scans3D.end()) {
		curr_laser_scan = search->second;
	}

	// commence only if I have the current laser scan
	if (curr_laser_scan) {
		// try adding ICP constraints with each node in the previous set
		for (set<mrpt::utils::TNodeID>::const_iterator
				node_it = nodes_set.begin();
				node_it != nodes_set.end(); ++node_it) {

			// get the ICP edge between current and last node
			constraint_t rel_edge;
			mrpt::slam::CICP::TReturnInfo icp_info;
			CObservation3DRangeScanPtr prev_laser_scan;

			// search for prev_laser_scan
			search = m_nodes_to_laser_scans3D.find(*node_it);
			if (search != m_nodes_to_laser_scans3D.end()) {
				prev_laser_scan = search->second;

				// TODO - use initial edge estimation
				m_time_logger.enter("CICPCriteriaERD::getICPEdge");
				this->getICPEdge(
						*prev_laser_scan,
						*curr_laser_scan,
						&rel_edge,
						NULL,
						&icp_info);
				m_time_logger.leave("CICPCriteriaERD::getICPEdge");

				// criterion for registering a new node
				if (icp_info.goodness > params.ICP_goodness_thresh) {
					this->registerNewEdge(*node_it, curr_nodeID, rel_edge);
					m_edge_types_to_nums["ICP3D"]++;
					// in case of loop closure
					if (absDiff(curr_nodeID, *node_it) > params.LC_min_nodeid_diff) {
						m_edge_types_to_nums["LC"]++;
						m_just_inserted_loop_closure = true;
					}
				}
			}
		}
	}

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) {
	MRPT_START;
	using namespace mrpt::utils;

	m_graph->insertEdge(from,  to, rel_edge);

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::getNearbyNodesOf(
		std::set<mrpt::utils::TNodeID> *nodes_set,
		const mrpt::utils::TNodeID& cur_nodeID,
		double distance ) {
	MRPT_START;
	using namespace mrpt::utils;

	if (distance > 0) {
		// check all but the last node.
		for (TNodeID nodeID = 0; nodeID < m_graph->nodeCount()-1; ++nodeID) {
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
void CICPCriteriaERD<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;

	m_graph = graph;

	this->logFmt(mrpt::utils::LVL_DEBUG, "Fetched the graph successfully");

	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	ASSERT_(win_manager);
	m_win_manager = win_manager;

	// may still be null..
	if (m_win_manager) {
		m_win = m_win_manager->win;

		m_win_observer = m_win_manager->observer;
	}

}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;
	ASSERT_(m_win_manager);

	// I know the key exists - I put it there explicitly
	if (events_occurred.find(params.keystroke_laser_scans)->second) {
		this->toggleLaserScansVisualization();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::toggleLaserScansVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");

	using namespace mrpt::opengl;

	this->logFmt(mrpt::utils::LVL_INFO, "Toggling LaserScans visualization...");

	COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (params.visualize_laser_scans) {
		CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		dumpVisibilityErrorMsg("visualize_laser_scans");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}


template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::getEdgesStats(
		std::map<std::string, int>* edge_types_to_num) const {
	MRPT_START;

	*edge_types_to_num = m_edge_types_to_nums;

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	ASSERT_(m_win_manager);

	this->logFmt(mrpt::utils::LVL_DEBUG, "Initializing visuals");
	m_time_logger.enter("CICPCriteriaERD::Visuals");
	using namespace mrpt::opengl;

	ASSERTMSG_(params.has_read_config,
			"Configuration parameters aren't loaded yet");
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");
	ASSERTMSG_(m_win_observer, "No CWindowObserver* was provided");

	m_win_observer->registerKeystroke(params.keystroke_laser_scans,
			"Toggle LaserScans Visualization");

	// ICP_max_distance disk
	if (params.ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CDiskPtr obj = CDisk::Create();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("ICP_max_distance");
		obj->setColor_u8(m_search_disk_color);
		obj->setDiskRadius(params.ICP_max_distance, params.ICP_max_distance-0.1);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// laser scan visualization
	if (params.visualize_laser_scans) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CPlanarLaserScanPtr laser_scan_viz = mrpt::opengl::CPlanarLaserScan::Create();
		laser_scan_viz->enablePoints(true);
		laser_scan_viz->enableLine(true);
		laser_scan_viz->enableSurface(true);
		laser_scan_viz->setSurfaceColor(
				m_laser_scans_color.R,
				m_laser_scans_color.G,
				m_laser_scans_color.B,
				m_laser_scans_color.A);

		laser_scan_viz->setName("laser_scan_viz");

		scene->insert(laser_scan_viz);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// max distance disk - textMessage
	if (m_win && m_win_manager && params.ICP_max_distance > 0) {
		m_win_manager->assignTextMessageParameters(
				&m_offset_y_search_disk,
				&m_text_index_search_disk);

		m_win_manager->addTextMessage(5,-m_offset_y_search_disk,
				mrpt::format("ICP Edges search radius"),
				mrpt::utils::TColorf(m_search_disk_color),
				/* unique_index = */ m_text_index_search_disk );
	}

	m_initialized_visuals = true;
	m_time_logger.leave("CICPCriteriaERD::Visuals");
	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::updateVisuals() {
	MRPT_START;
	ASSERT_(m_win_manager);
	ASSERT_(m_initialized_visuals);
	m_time_logger.enter("CICPCriteriaERD::Visuals");
	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	this->logFmt(LVL_DEBUG, "Updating visuals");

	// update ICP_max_distance Disk
	if (m_win && params.ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("ICP_max_distance");
		CDiskPtr disk_obj = static_cast<CDiskPtr>(obj);

		disk_obj->setPose(m_graph->nodes[m_graph->nodeCount()-1]);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	// update laser scan visual
	if (m_win && params.visualize_laser_scans &&
			(!m_last_laser_scan2D.null() || !m_fake_laser_scan2D.null())) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		CPlanarLaserScanPtr laser_scan_viz = static_cast<CPlanarLaserScanPtr>(obj);

		// if fake 2D exists use it
		if (!m_fake_laser_scan2D.null()) {
			laser_scan_viz->setScan(*m_fake_laser_scan2D);
		}
		else {
			laser_scan_viz->setScan(*m_last_laser_scan2D);
		}

		// set the pose of the laser scan
		typename GRAPH_t::global_poses_t::const_iterator search =
			m_graph->nodes.find(m_graph->nodeCount()-1);
		if (search != m_graph->nodes.end()) {
			laser_scan_viz->setPose(m_graph->nodes[m_graph->nodeCount()-1]);
			// put the laser scan *underneath* the graph, so that you can still
			// visualize the loop closures with the nodes ahead
			laser_scan_viz->setPose(CPose3D(
						laser_scan_viz->getPoseX(), laser_scan_viz->getPoseY(), -0.15,
						DEG2RAD(laser_scan_viz->getPoseYaw()),
						DEG2RAD(laser_scan_viz->getPosePitch()),
						DEG2RAD(laser_scan_viz->getPoseRoll())
						));
		}

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	m_time_logger.leave("CICPCriteriaERD::Visuals");
	MRPT_END;
}
template<class GRAPH_t>
bool CICPCriteriaERD<GRAPH_t>::justInsertedLoopClosure() const {
	return m_just_inserted_loop_closure;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	if (observation.present()) { // FORMAT #2
		if (IS_CLASS(observation, CObservation2DRangeScan) ||
				IS_CLASS(observation, CObservation3DRangeScan)) {
			m_checked_for_usuable_dataset = true;
			return;
		}
		else {
			m_consecutive_invalid_format_instances++;
		}
	}
	else {
		// TODO - what if it's in this format but only has odometry information?
		m_checked_for_usuable_dataset = true;
		return;
	}
	if (m_consecutive_invalid_format_instances > m_consecutive_invalid_format_instances_thres) {
		this->logFmt(LVL_ERROR,
				"Can't find usuable data in the given dataset.\nMake sure dataset contains valid CObservation2DRangeScan/CObservation3DRangeScan data.");
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time /* = 500 milliseconds */) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt;

	this->logFmt(LVL_ERROR,
			"Cannot toggle visibility of specified object.\n "
			"Make sure that the corresponding visualization flag ( %s "
			") is set to true in the .ini file.\n",
			viz_flag.c_str());
	mrpt::system::sleep(sleep_time);

	MRPT_END;
}


template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;
	using namespace mrpt::utils;

	params.loadFromConfigFileName(source_fname,
			"EdgeRegistrationDeciderParameters");
	this->logFmt(LVL_DEBUG, "Successfully loaded parameters. ");

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	int min_verbosity_level = source.read_int(
			"EdgeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::printParams() const {
	MRPT_START;

	params.dumpToConsole();

	MRPT_END;
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: " << std::endl;
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



// TParameter
// //////////////////////////////////

template<class GRAPH_t>
CICPCriteriaERD<GRAPH_t>::TParams::TParams(decider_t& d):
	decider(d),
	keystroke_laser_scans("l"),
	has_read_config(false)
{ }

template<class GRAPH_t>
CICPCriteriaERD<GRAPH_t>::TParams::~TParams() {
}

template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ Goodness-based ICP Edge Registration ]------------------\n");
	out.printf("ICP goodness threshold         = %.2f%% \n", ICP_goodness_thresh*100);
	out.printf("ICP max radius for edge search = %.2f\n", ICP_max_distance);
	out.printf("Min. node difference for LC    = %lu\n", LC_min_nodeid_diff);
	out.printf("Visualize laser scans          = %d\n", visualize_laser_scans);
	out.printf("3DScans Image Directory        = %s\n", scans_img_external_dir.c_str());

	decider.range_scanner_t::params.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CICPCriteriaERD<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
		const std::string& section) {
	MRPT_START;

	LC_min_nodeid_diff = source.read_int(
			"GeneralConfiguration",
			"LC_min_nodeid_diff",
			30, false);
	ICP_max_distance = source.read_double(
			section,
			"ICP_max_distance",
			10, false);
	ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
			0.75, false);
	visualize_laser_scans = source.read_bool(
			"VisualizationParameters",
			"visualize_laser_scans",
			true, false);
	scans_img_external_dir = source.read_string(
			section,
			"scan_images_external_directory",
			"./", false);

	// load the icp parameters - from "ICP" section explicitly
	decider.range_scanner_t::params.loadFromConfigFile(source, "ICP");

	has_read_config = true;

	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CICPCRITERIAERD_IMPL_H */
