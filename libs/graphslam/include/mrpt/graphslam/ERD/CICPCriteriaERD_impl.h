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

template<class GRAPH_T>
CICPCriteriaERD<GRAPH_T>::CICPCriteriaERD():
	m_search_disk_color(142, 142, 56),
	m_laser_scans_color(0, 20, 255),
	m_is_using_3DScan(false),
	m_ICP_max_distance_obj_name("ICP_max_distance")
{
	MRPT_START;
	using namespace mrpt::utils;

	this->initializeLoggers("CICPCriteriaERD");

	this->m_last_total_num_nodes = 2;

	this->logFmt(LVL_DEBUG, "Initialized class object");

	MRPT_END;
}
template<class GRAPH_T>
CICPCriteriaERD<GRAPH_T>::~CICPCriteriaERD() { }

// Methods implementations
// //////////////////////////////////

template<class GRAPH_T> bool CICPCriteriaERD<GRAPH_T>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt::obs;
	using namespace mrpt::utils;

	// check possible prior node registration
	bool registered_new_node = false;

	if (this->m_last_total_num_nodes < this->m_graph->nodeCount()) {
		registered_new_node = true;
		this->m_last_total_num_nodes = this->m_graph->nodeCount();
		this->logFmt(LVL_DEBUG, "New node has been registered!");
	}

	if (observation.present()) { // observation-only rawlog format
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
			this->m_last_laser_scan2D =
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);
			m_is_using_3DScan = false;
		}
		if (IS_CLASS(observation, CObservation3DRangeScan)) {
			this->m_last_laser_scan3D =
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
			// just load the range/intensity images - CGraphSlanEngine takes care
			// of the path
			this->m_last_laser_scan3D->load();

			m_is_using_3DScan = true;
		}

		// New node has been registered.
		// add the last laser_scan
		if (registered_new_node) {
			if (!this->m_last_laser_scan2D.null()) {
				this->m_nodes_to_laser_scans2D[this->m_graph->nodeCount()-1] =
					this->m_last_laser_scan2D;
			}
			if (!this->m_last_laser_scan3D.null()) {
				this->m_nodes_to_laser_scans3D[
					this->m_graph->nodeCount()-1] = this->m_last_laser_scan3D;
			}
		}
	}
	else { // action-observations rawlog format
		// append current laser scan
		this->m_last_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (registered_new_node && this->m_last_laser_scan2D) {
			this->m_nodes_to_laser_scans2D[
				this->m_graph->nodeCount()-1] = this->m_last_laser_scan2D;
		}
	}

	// edge registration procedure - same for both rawlog formats
	if (registered_new_node) {
		// get set of nodes within predefined distance for ICP
		std::set<mrpt::utils::TNodeID> nodes_to_check_ICP;
		this->getNearbyNodesOf(
				&nodes_to_check_ICP,
				this->m_graph->nodeCount()-1,
				m_ICP_max_distance);
		this->logFmt(LVL_DEBUG,
				"Found * %lu * nodes close to nodeID %lu",
				nodes_to_check_ICP.size(),
				this->m_graph->nodeCount()-1);

		// reset the loop_closure flag and run registration
		this->m_just_inserted_lc = false;
		registered_new_node = false;

		if (m_is_using_3DScan) {
			checkRegistrationCondition3D(nodes_to_check_ICP);
		}
		else {
			checkRegistrationCondition2D(nodes_to_check_ICP);
		}
	}

	return true;
	MRPT_END;
} // end of updateState

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::checkRegistrationCondition2D(
		const std::set<mrpt::utils::TNodeID>& nodes_set) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::math;

	mrpt::utils::TNodeID curr_nodeID = this->m_graph->nodeCount()-1;
	CObservation2DRangeScanPtr curr_laser_scan;
	typename nodes_to_scans2D_t::const_iterator search;

	// search for curr_laser_scan
	search = this->m_nodes_to_laser_scans2D.find(curr_nodeID);
	if (search != this->m_nodes_to_laser_scans2D.end()) {
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
			search = this->m_nodes_to_laser_scans2D.find(*node_it);
			if (search != this->m_nodes_to_laser_scans2D.end()) {
				prev_laser_scan = search->second;

				// make use of initial node position difference for the ICP edge
				pose_t initial_pose = this->m_graph->nodes[curr_nodeID] -
					this->m_graph->nodes[*node_it];

				this->m_time_logger.enter("ERD::getICPEdge");
				this->_getICPEdge(
						*prev_laser_scan,
						*curr_laser_scan,
						&rel_edge,
						&initial_pose,
						&icp_info);
				this->m_time_logger.leave("ERD::getICPEdge");

				// Debugging statements
				MRPT_LOG_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
				MRPT_LOG_DEBUG_STREAM(
					"ICP constraint between NON-successive nodes: " << 
					*node_it << " => " << curr_nodeID << 
					std::endl <<
					"\tnIterations = " << icp_info.nIterations <<
					"\tgoodness = " << icp_info.goodness);
				MRPT_LOG_DEBUG_STREAM("ICP_goodness_thresh: " <<m_ICP_goodness_thresh);
				MRPT_LOG_DEBUG_STREAM("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");


				// criterion for registering a new node
				if (icp_info.goodness > m_ICP_goodness_thresh) {
					this->registerNewEdge(*node_it, curr_nodeID, rel_edge);
					this->m_edge_types_to_nums["ICP2D"]++;
					// in case of loop closure
					if (absDiff(curr_nodeID, *node_it) >
							m_LC_min_nodeid_diff) {
						this->m_edge_types_to_nums["LC"]++;
						this->m_just_inserted_lc = true;
					}
				}
			}
		}
	}

	MRPT_END;
} // end of checkRegistrationCondition2D

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::checkRegistrationCondition3D(
		const std::set<mrpt::utils::TNodeID>& nodes_set) {
	THROW_EXCEPTION("Not Implemented yet.");
} // end of checkRegistrationCondition3D


template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::getNearbyNodesOf(
		std::set<mrpt::utils::TNodeID> *nodes_set,
		const mrpt::utils::TNodeID& cur_nodeID,
		double distance ) {
	MRPT_START;
	using namespace mrpt::utils;

	if (distance > 0) {
		// check all but the last node.
		for (TNodeID nodeID = 0;
				nodeID < this->m_graph->nodeCount()-1;
				++nodeID) {
			double curr_distance = this->m_graph->nodes[nodeID].distanceTo(
					this->m_graph->nodes[cur_nodeID]);
			if (curr_distance <= distance) {
				nodes_set->insert(nodeID);
			}
		}
	}
	else { // check against all nodes
		this->m_graph->getAllNodes(*nodes_set);
	}

	MRPT_END;
} // end of getNearbyNodesOf

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;
	parent_t::notifyOfWindowEvents(events_occurred);

	MRPT_END;
}

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::initializeVisuals() {
	MRPT_START;
	using namespace mrpt::opengl;
	this->m_time_logger.enter("ERD::Visuals");
	parent_t::initializeVisuals();

	if (m_ICP_max_distance > 0) {
		this->initICPDistanceVizualization();
	}

	// max distance disk - textMessage
	if (this->m_win_manager && m_ICP_max_distance > 0) {
		this->m_win_manager->assignTextMessageParameters(
				&m_offset_y_search_disk,
				&m_text_index_search_disk);

		this->m_win_manager->addTextMessage(5,-m_offset_y_search_disk,
				mrpt::format("ICP Edges search radius"),
				mrpt::utils::TColorf(m_search_disk_color),
				/* unique_index = */ m_text_index_search_disk );
	}

	this->m_time_logger.leave("ERD::Visuals");
	MRPT_END;
} // end of initializeVisuals

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::updateVisuals() {
	MRPT_START;
	this->m_time_logger.enter("ERD::Visuals");
	parent_t::updateVisuals();

	if (m_ICP_max_distance > 0) {
		this->updateICPDistanceVizualization();
	}

	this->m_time_logger.leave("ERD::Visuals");
	MRPT_END;
} // end of updateVisuals

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::initICPDistanceVizualization() {
	using namespace mrpt::opengl;
	COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

	CDiskPtr obj = CDisk::Create();
	pose_t initial_pose;
	obj->setPose(initial_pose);
	obj->setName(m_ICP_max_distance_obj_name);
	obj->setColor_u8(m_search_disk_color);
	obj->setDiskRadius(m_ICP_max_distance, m_ICP_max_distance-0.1);
	scene->insert(obj);

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();
} // end of initICPDistanceVizualization

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::updateICPDistanceVizualization() {
	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace mrpt::poses;

	COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

	CRenderizablePtr obj = scene->getByName(m_ICP_max_distance_obj_name);
	CDiskPtr disk_obj = static_cast<CDiskPtr>(obj);

	disk_obj->setPose(this->m_graph->nodes.at(this->m_graph->nodeCount()-1));

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

} // end of updateICPDistanceVizualization

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::loadParams(const std::string& source_fname) {
	MRPT_START;
	using namespace mrpt::utils;
	parent_t::loadParams(source_fname);

	// set the logging level if given by the user
	CConfigFile source(source_fname);

	m_LC_min_nodeid_diff = source.read_int(
			"GeneralConfiguration",
			"LC_min_nodeid_diff",
			30, false);

	std::string section("CRangeScanEdgeRegistrationDecider");

	int min_verbosity_level = source.read_int(
			section,
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	m_ICP_max_distance = source.read_double(
			section,
			"ICP_max_distance",
			10, false);
	m_ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
			0.75, false);
	m_scans_img_external_dir = source.read_string(
			section,
			"scan_images_external_directory",
			"./", false);

	MRPT_END;
} // end of loadParamas

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::printParams() const {
	MRPT_START;
	parent_t::printParams();

	printf("------------------[ Goodness-based ICP Edge Registration ]------------------\n");
	printf("ICP goodness threshold         = %.2f%% \n", m_ICP_goodness_thresh*100);
	printf("ICP max radius for edge search = %.2f\n", m_ICP_max_distance);
	printf("Min. node difference for LC    = %lu\n", m_LC_min_nodeid_diff);
	// TODO - Do not display it if not in 3D
	printf("3DScans Image Directory        = %s\n", m_scans_img_external_dir.c_str());

	MRPT_END;
} // end of printParams

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent_t::getDescriptiveReport(report_str);

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CICPCRITERIAERD_IMPL_H */
