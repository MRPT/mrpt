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
	m_ICP_max_distance_obj_name("ICP_max_distance")
{
	MRPT_START;
	using namespace mrpt::utils;

	this->initializeLoggers("CICPCriteriaERD");
	this->m_last_total_num_nodes = 2;
	this->logFmt(LVL_DEBUG, "Initialized class object");

	// do not use this since it affects LCs when we use a radius around the latest
	// registered node
	this->m_use_mahal_distance_init_ICP = false;

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
		}
		if (IS_CLASS(observation, CObservation3DRangeScan)) {
			this->m_last_laser_scan3D =
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
			// just load the range/intensity images - CGraphSlanEngine takes care
			// of the path
			this->m_last_laser_scan3D->load();
		}


		// New node has been registered.
		// add the last laser_scan
		if (registered_new_node) {
			if (!this->m_last_laser_scan2D.null()) {
				this->m_nodes_to_laser_scans2D[this->m_graph->nodeCount()-1] =
					this->m_last_laser_scan2D;
			}
			if (!this->m_last_laser_scan3D.null()) {
				this->m_nodes_to_laser_scans3D[this->m_graph->nodeCount()-1] =
					this->m_last_laser_scan3D;
			}
		}
	} // end if observation.present()
	else { // action-observations rawlog format
		// append current laser scan
		this->m_last_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (registered_new_node && this->m_last_laser_scan2D) {
			this->m_nodes_to_laser_scans2D[this->m_graph->nodeCount()-1] =
				this->m_last_laser_scan2D;
		}
	} // end else

	// edge registration procedure - same for both rawlog formats
	if (registered_new_node) {
		// reset the loop_closure flag and run registration
		this->m_just_inserted_lc = false;
		registered_new_node = false;

		this->addScanMatchingEdges(this->m_graph->nodeCount()-1);
	}

	return true;
	MRPT_END;
} // end of updateState

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace std;
	parent_t::registerNewEdge(from, to, rel_edge);

	this->m_edge_types_to_nums["ICP2D"]++;
	//  keep track of the registered edges...
	if (absDiff(to, from) > this->m_LC_min_nodeid_diff)  {
		this->m_edge_types_to_nums["LC"]++;
		this->m_just_inserted_lc = true;
		this->logFmt(LVL_INFO, "\tLoop Closure edge!");
	}
	else {
		this->m_just_inserted_lc = false;
	}
	MRPT_END;
} // end of registerNewEdge



template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::fetchNodeIDsForScanMatching(
		const mrpt::utils::TNodeID& curr_nodeID,
		std::set<mrpt::utils::TNodeID>* nodes_set) {
	this->getNearbyNodesOf(
			this->m_graph->nodeCount()-1,
			nodes_set,
			m_ICP_max_distance);
} // end of fetchNodeIDsForScanMatching

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::getNearbyNodesOf(
		const mrpt::utils::TNodeID& curr_nodeID,
		std::set<mrpt::utils::TNodeID> *nodes_set,
		const double distance) {
	MRPT_START;
	using namespace mrpt::utils;

	if (distance > 0) {
		// check all but the last node.
		for (TNodeID nodeID = 0;
				nodeID < this->m_graph->nodeCount()-1;
				++nodeID) {
			double curr_distance = this->m_graph->nodes[nodeID].distanceTo(
					this->m_graph->nodes[curr_nodeID]);
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
} // end of notifyOfWindowEvents

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

	std::string section("EdgeRegistrationDeciderParameters");

	m_ICP_max_distance = source.read_double(
			section,
			"ICP_max_distance",
			10, false);

	MRPT_END;
} // end of loadParamas

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::printParams() const {
	MRPT_START;
	parent_t::printParams();

	printf("------------------[ Goodness-based ICP Edge Registration ]------------------\n");
	printf("ICP goodness threshold         = %.2f%% \n", this->getGoodnessThresh()*100);
	printf("ICP max radius for edge search = %.2f\n", m_ICP_max_distance);
	printf("Min. node difference for LC    = %lu\n", m_LC_min_nodeid_diff);

	MRPT_END;
} // end of printParams

template<class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::
getDescriptiveReport(std::string* report_str) const {
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
} // end of getDescriptiveReport

} } } // end of namespaces

#endif /* end of include guard: CICPCRITERIAERD_IMPL_H */
