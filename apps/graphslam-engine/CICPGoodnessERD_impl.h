/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPGoodnessERD_IMPL_H
#define CICPGoodnessERD_IMPL_H


using namespace mrpt::graphslam::deciders;

// Ctors, Dtors
// //////////////////////////////////

template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::CICPGoodnessERD_t():
	m_search_disk_color(TColor(142, 142, 56)),
	m_consecutive_invalid_format_instances_thres(20) // large threshold just to make sure
{
	MRPT_START;

	this->initCICPGoodnessERD_t();

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::initCICPGoodnessERD_t() {
	MRPT_START;

	m_win = NULL;
	m_win_manager = NULL;
	m_graph = NULL;

	m_initialized_visuals = false;
	m_just_inserted_loop_closure = false;

	// start ICP constraint registration only when 
	// nodeCount > m_last_total_num_of_nodes
	m_last_total_num_of_nodes = 2;

	m_edge_types_to_nums["ICP"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	std::cout << "CCICPGoodnessERD: Initialized class object" << std::endl;

	MRPT_END;
}
template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::~CICPGoodnessERD_t() { }

// Method implementations
// //////////////////////////////////

template<class GRAPH_t> void CICPGoodnessERD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	
	// check possible prior node registration
	bool registered_new_node = false;

	if (m_last_total_num_of_nodes < m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = m_graph->nodeCount();
		//std::cout << "CICPGoodnessERD: Registered new node. " << std::endl;
	}

	if (observation.present()) { // observation-only rawlog format
			if (IS_CLASS(observation, CObservation2DRangeScan)) {
				m_last_laser_scan =
					static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);
			}

			// add the last laser_scan
			if (registered_new_node && m_last_laser_scan) {
				m_nodes_to_laser_scans[m_graph->nodeCount()-1] = m_last_laser_scan;
				std::cout << "CICPGoodnessERD: Added laser scans of nodeID: "
					<< m_graph->nodeCount()-1 << std::endl;
			}
	}
	else { // action-observations rawlog format
		// append current laser scan
		CObservation2DRangeScanPtr curr_laser_scan =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (registered_new_node && curr_laser_scan) {
			m_nodes_to_laser_scans[m_graph->nodeCount()-1] = curr_laser_scan;
			//std::cout << "Added laser scans of nodeID: "
			//<< m_graph->nodeCount()-1 << std::endl;
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
		std::cout << "Found * " << nodes_to_check_ICP.size() 
			<< " * nodes close to nodeID: " << m_graph->nodeCount()-1 << endl;

		// reset the loop_closure flag and run registration
		m_just_inserted_loop_closure = false;
		registered_new_node = false;
		checkRegistrationCondition(nodes_to_check_ICP);
	}

	if (!m_checked_for_usuable_dataset) {
		this->checkIfInvalidDataset(action, observations, observation);
	}

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::checkRegistrationCondition(
		const std::set<mrpt::utils::TNodeID>& nodes_set) {
	MRPT_START;

	// try adding ICP constraints with each node in the previous set
	for (set<mrpt::utils::TNodeID>::const_iterator 
			node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		// get the ICP edge between current and last node
		constraint_t rel_edge;
		double ICP_goodness = this->getICPEdge(
				*node_it,
				m_graph->nodeCount()-1,
				&rel_edge);

		// criterion for registering a new node
		if (ICP_goodness > params.ICP_goodness_thresh) {
			this->registerNewEdge(
					*node_it, m_graph->nodeCount()-1, rel_edge);
			m_edge_types_to_nums["ICP"]++;
			// in case of loop closure
			if (abs(m_graph->nodeCount()-1 - *node_it) > params.LC_min_nodeid_diff) {
				m_edge_types_to_nums["LC"]++;
				m_just_inserted_loop_closure = true;
			}
		}
	}

	MRPT_END;
}


template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::registerNewEdge(
    const mrpt::utils::TNodeID& from, 
    const mrpt::utils::TNodeID& to,
    const constraint_t& rel_edge ) {
  MRPT_START;

  //cout << "CICPGoodnessERD: Inserting new Edge: " << from << " -> " << to << endl;
	m_graph->insertEdge(from,  to, rel_edge);

	MRPT_END;
}
template<class GRAPH_t>
double CICPGoodnessERD_t<GRAPH_t>::getICPEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		constraint_t* rel_edge ) {
	MRPT_START;

	// get the laser scan measurements - first check if they exist
	CObservation2DRangeScanPtr prev_laser_scan;
	CObservation2DRangeScanPtr curr_laser_scan;

	std::map<const mrpt::utils::TNodeID, 
		mrpt::obs::CObservation2DRangeScanPtr>::const_iterator search =
			m_nodes_to_laser_scans.find(from); 
	if (search != m_nodes_to_laser_scans.end()) {
		prev_laser_scan = search->second;
	}
	else {
		std::cout << "Unable to find laser scan of NodeID: " 
			<< from << std::endl;
		return 0.0;
	}
	search =	m_nodes_to_laser_scans.find(to); 
	if (search != m_nodes_to_laser_scans.end()) {
		curr_laser_scan = search->second;
	}
	else {
		std::cout << "Unable to find laser scan of NodeID: " 
			<< to << std::endl;
		return 0.0;
	}

	// Uses TParams::ICP member variable
	CSimplePointsMap m1,m2;
	float running_time;
	CICP::TReturnInfo info;

	pose_t initial_pose;
	// use the difference of the node positions as an initial alignment
	// estimation (dijkstra_nodes_estimate has to be run from the caller
	// function)
	initial_pose = m_graph->nodes[to] - m_graph->nodes[from];

	m1.insertObservation(&(*prev_laser_scan));
	m2.insertObservation(&(*curr_laser_scan));

	CPosePDFPtr pdf = params.icp.Align(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	// return the edge regardless of the goodness of the alignment
	rel_edge->copyFrom(*pdf);  	
	return info.goodness;

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::getNearbyNodesOf(
		set<TNodeID> *nodes_set,
		const TNodeID& cur_nodeID,
		double distance ) {
	MRPT_START;

	if (distance > 0) {
		// check all but the last node.
		for (TNodeID nodeID = 0; nodeID < m_graph->nodeCount()-1; ++nodeID) {
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
void CICPGoodnessERD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;

	m_graph = graph;

	std::cout << "CICPGoodnessERD: Fetched the graph successfully" 
		<< std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::setWindowManagerPtr(
		mrpt::gui::CWindowManager_t* win_manager) {
	m_win_manager = win_manager;
}
template<class GRAPH_t> void
CICPGoodnessERD_t<GRAPH_t>::setCDisplayWindowPtr(
		mrpt::gui::CDisplayWindow3D* win) {
	MRPT_START;

	m_win = win;

	std::cout << "CICPGoodnessERD: Fetched the CDisplayWindow successfully" 
		<< std::endl;

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::getEdgesStats(
		std::map<const std::string, int>* edge_types_to_nums) {
	MRPT_START;

	*edge_types_to_nums = m_edge_types_to_nums;

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	std::cout << "Initializing CICPGoodnessERD visuals" << std::endl;

	// ICP_max_distance disk
	if (m_win &&  params.ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CDiskPtr obj = CDisk::Create();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("ICP_max_distance");
		obj->setColor(m_search_disk_color);
		obj->setDiskRadius(params.ICP_max_distance, params.ICP_max_distance-0.5);
		scene->insert(obj);

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	m_initialized_visuals = true;

	if (m_win && m_win_manager && params.ICP_max_distance > 0) {
		m_win_manager->assignTextMessageParameters(
				&m_offset_y_search_disk,
				&m_text_index_search_disk);

		m_win_manager->addTextMessage(5,-m_offset_y_search_disk,
				format("ICP Edges search radius"),
				mrpt::utils::TColorf(m_search_disk_color),
				/* unique_index = */ m_text_index_search_disk );
	}

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::updateVisuals() {
	MRPT_START;
	//std::cout << "Updating CICPGoodnessERD visuals" << std::endl;

	// update ICP_max_distance Disk
	if (m_win && params.ICP_max_distance > 0) {
		COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		CRenderizablePtr obj = scene->getByName("ICP_max_distance");
		CDiskPtr disk_obj = static_cast<CDiskPtr>(obj);

		disk_obj->setPose(m_graph->nodes[m_graph->nodeCount()-1]);

		m_win->unlockAccess3DScene();
	}

	MRPT_END;
}
template<class GRAPH_t>
bool CICPGoodnessERD_t<GRAPH_t>::justInsertedLoopClosure() {
	return m_just_inserted_loop_closure;
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);

	if (observation.present()) { // FORMAT #2
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
			m_checked_for_usuable_dataset = true;
			return;
		}
		else {
			m_consecutive_invalid_format_instances++;
		}
	}
	else {
		m_checked_for_usuable_dataset = true;
		return;
	}
	if (m_consecutive_invalid_format_instances > m_consecutive_invalid_format_instances_thres) {
		std::cout << "CICPGoodnessERD: Can't find usuable data in the given dataset." 
			<< std::endl;
		std::cout << "Make sure dataset contains valid CObservation2DRangeScan/CObservation3DRangeScan data." 
			<< std::endl;
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}


// TParameter
// //////////////////////////////////

template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::TParams::TParams() {
}

template<class GRAPH_t>
CICPGoodnessERD_t<GRAPH_t>::TParams::~TParams() {
}

template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ Goodness-based ICP Edge Registration ]------------------\n");
	out.printf("ICP goodness threshold         = %.2f%% \n", ICP_goodness_thresh*100);
	out.printf("ICP max radius for edge search = %.2f\n", ICP_max_distance);
	out.printf("Min. node difference for LC    = %d\n", LC_min_nodeid_diff);
	out.printf("ICP Configuration:\n");

	icp.options.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessERD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
    const std::string& section) {
  MRPT_START;

	ICP_max_distance = source.read_double(
			section,
			"ICP_max_distance",
			10, false);
	ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
	 		0.75, false);
  LC_min_nodeid_diff = source.read_int(
  		section,
 			"LC_min_nodeid_diff",
 			10, false);

	// load the icp parameters - from "ICP" section explicitly
	icp.options.loadFromConfigFile(source, "ICP");

	std::cout << "Successfully loaded CICPGoodnessERD parameters. " 
		<< std::endl;

	MRPT_END;
}

#endif /* end of include guard: CICPGoodnessERD_IMPL_H */
