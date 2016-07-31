/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */


#ifndef CLOOPCLOSERERD_IMPL_H
#define CLOOPCLOSERERD_IMPL_H


namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
// //////////////////////////////////
template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::CLoopCloserERD():
	m_curr_node_covariance_color(160, 160, 160, /*alpha = */255),
	m_consecutive_invalid_format_instances_thres(20), // high threshold just to make sure
	m_class_name("CLoopCloserERD")
{
	MRPT_START;
	this->initCLoopCloserERD();
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initCLoopCloserERD() {
	MRPT_START;

	m_win = NULL;
	m_win_manager = NULL;
	m_graph = NULL;

	m_initialized_visuals = false;
	m_visualize_curr_node_covariance = true;
	m_just_inserted_loop_closure = false;

	// start the edge registration procedure only when this num is surpassed
	// nodeCount > m_last_total_num_of_nodes
	m_threshold_to_start = m_last_total_num_of_nodes = 0;

	m_edge_types_to_nums["ICP2D"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	m_partitions_full_update = false;

	m_time_logger.setName(m_class_name);
	m_out_logger.setName(m_class_name);
	m_out_logger.setLoggingLevel(mrpt::utils::LVL_DEBUG); // defalut level of logger

	m_out_logger.log("Initialized class object");

	MRPT_END;
}
template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::~CLoopCloserERD() {

	// release memory of m_node_optimal_paths map.
	m_out_logger.logFmt("Releasing memory of m_node_optimal_paths map...");
	for (typename std::map<mrpt::utils::TNodeID, TPath*>::iterator it = 
			m_node_optimal_paths.begin(); it != m_node_optimal_paths.end();
			++it) {

		delete it->second;
	}

}



// Methods implementations
// //////////////////////////////////

template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	m_time_logger.enter("updateState");
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	// check possible prior node registration
	bool registered_new_node = false;

	// was a new node registered?
	if (m_last_total_num_of_nodes < m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = m_graph->nodeCount();
		m_out_logger.log("New node has been registered!");
	}

	if (observation.present()) { // observation-only rawlog format
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
			// update last laser scan to use
			m_last_laser_scan2D = 
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);

		}
	}
	else { // action-observations format
		// TODO - going to implement this.
	}


	if (registered_new_node) {
		// register the new node-laserScan pair
		m_nodes_to_laser_scans2D[m_graph->nodeCount()-1] = m_last_laser_scan2D;

		if (m_laser_params.use_scan_matching) {
			// scan match with previous X nodes
			this->addScanMatchingEdges(m_graph->nodeCount()-1);
		}

		// update the partitioned map
		if ((m_graph->nodeCount() % 50) == 0) {
			m_partitions_full_update = true;
		}
		else {
			m_partitions_full_update = false;
		}
		this->updateMapPartitions(m_partitions_full_update);

		// check for loop closures
		partitions_t partitions_for_LC;
		this->checkPartitionsForLC(&partitions_for_LC);
		this->evaluatePartitionsForLC(partitions_for_LC);

		if (m_visualize_curr_node_covariance) {
			this->execDijkstraProjection();
		}

		// TODO - remove this
		if (m_graph->nodeCount() % 30 == 0) {
			
			CMatrixDouble33 consistency_elem;
			this->generatePWConsistencyElement(
					m_graph->nodeCount()-15,
					m_graph->nodeCount()-12,
					m_graph->nodeCount()-4,
					m_graph->nodeCount()-1,
					&consistency_elem);
		}

	}

	m_time_logger.leave("updateState");
	// TODO - remove this
	return false;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::addScanMatchingEdges(mrpt::utils::TNodeID curr_nodeID) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	// get a list of nodes to check ICP against
	std::set<TNodeID> nodes_set;

	// have too few nodes been registered yet?
	if (curr_nodeID < m_laser_params.prev_nodes_for_ICP) {
		for (TNodeID nodeID = 0; nodeID != curr_nodeID; ++nodeID) {
			nodes_set.insert(nodeID);
		}
	}
	else {
		for (TNodeID nodeID = curr_nodeID-1;
				nodeID != curr_nodeID-1 - m_laser_params.prev_nodes_for_ICP;
				--nodeID) {
			nodes_set.insert(nodeID);
		}
	}

	m_out_logger.logFmt("Adding ICP Constraints for nodeID: %lu", curr_nodeID);

	// try adding ICP constraints with each node in the previous set
	for (std::set<TNodeID>::const_iterator node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		constraint_t rel_edge;
		mrpt::slam::CICP::TReturnInfo icp_info;

		this->getICPEdge(
				*node_it,
				curr_nodeID,
				&rel_edge,
				&icp_info);

		// criterion for registering a new node
		if (icp_info.goodness > m_laser_params.ICP_goodness_thresh) {
			this->registerNewEdge(*node_it, curr_nodeID, rel_edge);
		}
	}

	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getICPEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		constraint_t* rel_edge,
		mrpt::slam::CICP::TReturnInfo* icp_info) {
	MRPT_START;
	ASSERT_(rel_edge);
	m_time_logger.enter("getICPEdge");

	using namespace mrpt::obs;
	using namespace mrpt::utils;

	// find the relevant laser scans
	nodes_to_scans2D_t::const_iterator search;
	CObservation2DRangeScanPtr from_laser_scan, to_laser_scan;
	search = m_nodes_to_laser_scans2D.find(from);
	if (search != m_nodes_to_laser_scans2D.end()) {
		from_laser_scan = search->second;
	}
	ASSERT_(from_laser_scan.present());
	search = m_nodes_to_laser_scans2D.find(to);
	if (search != m_nodes_to_laser_scans2D.end()) {
		to_laser_scan = search->second;
	}
	ASSERT_(to_laser_scan.present());

	// make use of initial node position difference for the ICP edge
	pose_t initial_pose = m_graph->nodes.at(to) -
		m_graph->nodes.at(from);

	range_scanner_t::getICPEdge(
			*from_laser_scan,
			*to_laser_scan,
			rel_edge,
			&initial_pose,
			icp_info);

	m_time_logger.leave("getICPEdge");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::checkPartitionsForLC(
		partitions_t* partitions_for_LC) {
	MRPT_START;
	m_time_logger.enter("LoopClosureEvaluation");
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::utils;

	partitions_for_LC->clear();

	int partitionID = 0;
	// for every partition...
	for (partitions_t::const_iterator partitions_it = m_curr_partitions.begin();
			partitions_it != m_curr_partitions.end(); ++partitions_it, ++partitionID)
	{
		// investigate each partition
		size_t prev_nodeID = *(partitions_it->begin());
		for (vector_uint::const_iterator it = partitions_it->begin()+1;
				it != partitions_it->end(); ++it) {
			size_t curr_nodeID = *it;

			// are there consecutive nodes with large difference inside this
			// partition?
			if ((curr_nodeID - prev_nodeID) > m_lc_params.LC_min_nodeid_diff) {
				m_out_logger.log(mrpt::format("Found potential loop closures:\n"
							"\tPartitionID: %d\n"
							"\tPartition: %s\n"
							"\t%lu ==> %lu\n",
							partitionID, getVectorAsString(*partitions_it).c_str(),
							prev_nodeID, curr_nodeID),
						LVL_WARN);
				partitions_for_LC->push_back(*partitions_it);
				continue; // no need to check the rest of the nodes in this partition
			}

			// update the previous node
			prev_nodeID = curr_nodeID;
		}
	}

	m_time_logger.leave("LoopClosureEvaluation");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::evaluatePartitionsForLC(
		const partitions_t& partitions) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace std;

	// for each one of the partitions generate the pair-wise consistency Matrix of
	// the relevant edges and find the submatrix of the msot consistent edges
	// inside it.
	for (partitions_t::const_iterator p_it = partitions.begin();
			p_it != partitions.end();
			++p_it ) {
		CMatrixDouble consist_matrix; // matrix to fill in
		this->generatePWConsistencyMatrix(*p_it, &consist_matrix);

		// evaluate the pair-wise consistency matrix
		// TODO - consult the paper on this

		// TODO - partition is worth it only if nodes of the partition are split
		// into two groups A, B whose node difference is larger than the LC
		// difference
	}


	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::generatePWConsistencyMatrix(
		const vector_uint& partition,
		mrpt::math::CMatrixDouble* constist_matrix) const {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::math;
	// resize the matrix to fit all the pair-wise consistencies
	// TODO

	// fill the matrix

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::generatePWConsistencyElement(
		const mrpt::utils::TNodeID& a1,
		const mrpt::utils::TNodeID& a2,
		const mrpt::utils::TNodeID& b1,
		const mrpt::utils::TNodeID& b2,
		mrpt::math::CMatrixDouble33* consistency_elem ) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::math;

	// get the dijkstra links
	// a1=>a2
	execDijkstraProjection(/*starting_node=*/ a1, /*ending_node=*/ a2);
	TPath* path_a1a2 = this->queryOptimalPath(a2);
	ASSERTMSG_(path_a1a2->getSource() == a1,
			format("\nnodeID %lu is not the source of the optimal path\n%s\n\n",
				a1, path_a1a2->getAsString().c_str()));
	ASSERTMSG_(path_a1a2->getDestination() == a2,
			format("\nnodeID %lu is not the destination of the optimal path\n%s\n\n",
				a2, path_a1a2->getAsString().c_str()));
	// b1=>b2
	execDijkstraProjection(/*starting_node=*/ b1, /*ending_node=*/ b2);
	TPath* path_b1b2 = this->queryOptimalPath(b2);
	ASSERTMSG_(path_b1b2->getSource() == b1,
			format("\nnodeID %lu is not the source of the optimal path\n%s\n\n",
				b1, path_b1b2->getAsString().c_str()));
	ASSERTMSG_(path_b1b2->getDestination() == b2,
			format("\nnodeID %lu is not the destination of the optimal path\n%s\n\n",
				b2, path_b1b2->getAsString().c_str()));


	// get the hypotheses
	constraint_t edge_a2b1, edge_b2a1;
	{
		// TODO - Assert that the hypotheses don't return nans in the information
		// matrix
		mrpt::slam::CICP::TReturnInfo icp_info;
		this->getICPEdge(a2, b1, &edge_a2b1, &icp_info);	// a2=>b1
		cout << "Goodness of a2=>b1: " << icp_info.goodness << endl;
		this->getICPEdge(b2, a1, &edge_b2a1, &icp_info);	// b2=>a1
		cout << "Goodness of b2=>a1: " << icp_info.goodness << endl;
	}

	constraint_t res(path_a1a2->curr_pose_pdf);
	//cout << "a1=>a2: " << endl << res;
	res += edge_a2b1;
	//cout << "a2=>b1: " << endl << edge_a2b1;
	res += path_b1b2->curr_pose_pdf;
	//cout << "b1=>b2: " << endl << path_b1b2->curr_pose_pdf;
	res += edge_b2a1;
	//cout << "b2=>a1: " << endl << edge_b2a1;

	cout << "Resulting Transformation: " << endl;
	cout << res << endl;
	//CMatrixDouble33 mat; res.getInformationMatrix(mat);
	//cout << "Determinant: " << mat.det() << endl;
	
	// get the homogeneous matrix of the translation-rotation
	CMatrixDouble44 T_tmp;
	res.getMeanVal().getHomogeneousMatrix(T_tmp);

	// extract the 2D part of the homogeneous Matrix
	CMatrixDouble33 T;
	T(0,0) = T_tmp(0,0) ; T(0,1) = T_tmp(0,1) ; T(0,2) = T_tmp(0,3) ;
	T(1,0) = T_tmp(1,0) ; T(1,1) = T_tmp(1,1) ; T(1,2) = T_tmp(1,3) ;
	T(2,0) = 0.0        ; T(2,1) = 0.0        ; T(2,2) = 1          ;

	// information matrix
	CMatrixDouble33 inf_mat;
	res.getInformationMatrix(inf_mat);

	//CMatrixDouble33 exponent = T * inf_mat * T.transpose();
	Eigen::MatrixXd exponent(3,3); 
	//exponent = T * inf_mat * T.transpose();
	exponent = T * inf_mat * T.transpose();
	// TODO - normalize it - Should I?
	exponent /= exponent.determinant(); // if matrix ill-conditioned? 
	//exponent /= exponent.maxCoeff(); // TODO - test this.

	ASSERT_(consistency_elem);
	Eigen::MatrixXd consistency_tmp = exponent.exp(); // TODO - Can't directly assign it to CMatrixDouble44 - compilation error
	*consistency_elem = consistency_tmp;

	cout << "T = " << endl << T;
	cout << "Exponent= " << endl << exponent << endl;
	cout << "Exponent.determinant()= " << exponent.determinant() << endl;
	cout << "consistency_elem = " << endl << *consistency_elem;
	
	mrpt::system::pause();

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::execDijkstraProjection(
		mrpt::utils::TNodeID starting_node/*=0*/,
		mrpt::utils::TNodeID ending_node/*=INVALID_NODEID*/) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::utils;
	// for the full algorithm see
	// - Recognizing places using spectrally cllustered local matches - E.Olson,
	// p.6
	
	m_time_logger.enter("Dijkstra Projection");

	// ending_node is either INVALID_NODEID or one of the already registered
	// nodeIDs
	ASSERT_(ending_node == INVALID_NODEID ||
			(ending_node >= 0 && ending_node < m_graph->nodeCount()) );

	m_out_logger.logFmt("Executing Dijkstra Projection starting from nodeID: %lu",
			starting_node);
	// if uncertainties already updated - do nothing
	if (m_graph->nodeCount() < 5) return;

	// keep track of the nodes that I have visited
	std::vector<bool> visited_nodes(m_graph->nodeCount(), false);
	m_node_optimal_paths.clear();

	// get the neighbors of each node
	std::map<TNodeID, std::set<TNodeID>>  neighbors_of;
	m_graph->getAdjacencyMatrix(neighbors_of);

	// initialize a pool of TPaths - draw the minimum-uncertainty path during
	// execution
	std::set<TPath*> pool_of_paths;
	// get the edge to each one of the neighboring nodes of the starting node
	std::set<TNodeID> starting_node_neighbors(neighbors_of.at(starting_node));
	for (std::set<TNodeID>::const_iterator n_it =
			starting_node_neighbors.begin();
			n_it != starting_node_neighbors.end(); ++n_it) {

		TPath* path_between_neighbors = new TPath();
		this->getMinUncertaintyPath(starting_node, *n_it, path_between_neighbors);

		pool_of_paths.insert(path_between_neighbors);
	}
	// just visited the first node
	visited_nodes.at(starting_node) = true;

	//// TODO Remove these - >>>>>>>>>>>>>>>>>>>>
	//// printing the pool for verification
	//cout << "Pool of Paths: " << endl;
	//for (typename std::set<TPath*>::const_iterator it = pool_of_paths.begin();
			//it != pool_of_paths.end(); ++it) {
		//printVector((*it)->nodes_traversed);
	//}
	//cout << "------ Done with the starting node ... ------" << endl;
	//int iters = 0;
	//// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	// for all unvisited nodes
	while ( std::any_of(visited_nodes.begin(), visited_nodes.end(),
			[](bool b) {return !b;} ) ) { // if there is at least one false..

		// if an ending nodeID has been specified, end the method when the path to
		// it is found.
		if (ending_node != INVALID_NODEID) {
			if (visited_nodes.at(ending_node)) {
				return;
			}
		}

		TPath* optimal_path = this->popMinUncertaintyPath(&pool_of_paths);
		TNodeID dest = optimal_path->getDestination();
	
		//// TODO Remove these - >>>>>>>>>>>>>>>>>>>>
		//cout << iters << " " << std::string(40, '>') << endl;
		//cout << "current path Destination: " << dest << endl;
		//// printing the pool for verification
		//cout << "Pool of Paths: " << endl;
		//for (typename std::set<TPath*>::const_iterator it = pool_of_paths.begin();
				//it != pool_of_paths.end(); ++it) {
			//printVector((*it)->nodes_traversed);
		//}
		//cout << "Nodes visited: " << endl;
		//std::vector<int> tmp_vec;
		//for (int i = 0; i != visited_nodes.size(); ++i) {
			//tmp_vec.push_back(i);
		//}
		//printVector(tmp_vec); cout << endl; // indices of numbers
		//printVector(visited_nodes);         // actual flags
		//cout << std::string(40, '<') << " " << iters++ << endl;
		//mrpt::system::pause();
		//// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

		if (!visited_nodes.at(dest)) {
			m_node_optimal_paths[dest] = optimal_path;
			visited_nodes.at(dest)= true;

			// for all the edges leaving this node .. compose the transforms with the
			// current pool of paths.
			this->addToPaths(&pool_of_paths, *optimal_path, neighbors_of.at(dest) );
		}
	}
	//// TODO Remove these - >>>>>>>>>>>>>>>>>>>>
	//cout << "----------- Done with Dijkstra Projection... ----------" << endl;
	//// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	m_time_logger.leave("Dijkstra Projection");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::addToPaths(
		std::set<TPath*>* pool_of_paths,
		const TPath& current_path,
		const std::set<mrpt::utils::TNodeID>& neighbors) const {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	TNodeID node_to_append_from = current_path.getDestination();

	// compose transforms for every neighbor of node_to_append_from *except*
	// for the link connecting node_to_append_from and the second to last node in
	// the current_path
	TNodeID second_to_last_node = current_path.nodes_traversed.rbegin()[1];
	for (std::set<TNodeID>::const_iterator neigh_it = neighbors.begin();
			neigh_it != neighbors.end(); ++neigh_it) {
		if (*neigh_it == second_to_last_node) continue;

		// get the path between node_to_append_from, *node_it
		TPath path_between_nodes;
		this->getMinUncertaintyPath(node_to_append_from, *neigh_it,
				&path_between_nodes);

		// format the path to append
		TPath* path_to_append = new TPath();
		*path_to_append = current_path;
		*path_to_append += path_between_nodes;

		pool_of_paths->insert(path_to_append);
	}

	MRPT_END;
}

template<class GRAPH_t>
typename CLoopCloserERD<GRAPH_t>::TPath*
CLoopCloserERD<GRAPH_t>::queryOptimalPath(const mrpt::utils::TNodeID node) const {
	MRPT_START;

	TPath* path = NULL;
	typename std::map<mrpt::utils::TNodeID, TPath*>::const_iterator search;
	search = m_node_optimal_paths.find(node);
	if (search != m_node_optimal_paths.end()) {
		path = search->second;
	}

	return path;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getMinUncertaintyPath(
		const mrpt::utils::TNodeID from,
		const mrpt::utils::TNodeID to,
		TPath* path_between_nodes) const {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace std;

	ASSERTMSG_(m_graph->edgeExists(from, to) || m_graph->edgeExists(to, from),
			mrpt::format("\nEdge between the provided nodeIDs"
				"(%lu <-> %lu) does not exist\n", from, to) );

	// don't add to the path_between_nodes, just fill it in afterwards
	path_between_nodes->clear(); 
	
	// iterate over all the edges, ignore the ones that are all 0s - find the
	// one that is with the lowest uncertainty
	double curr_determinant = 0;
	// forward edges from -> to
	std::pair<edges_citerator, edges_citerator> fwd_edges_pair =
		m_graph->getEdges(from, to);

	for (edges_citerator edges_it = fwd_edges_pair.first;
			edges_it != fwd_edges_pair.second; ++edges_it) {
		// operate on a temporary object instead of the real edge - otherwise
		// function is non-const
		constraint_t curr_edge;
		curr_edge.copyFrom(edges_it->second);

		// is it all 0s?
		CMatrixDouble33 inf_mat;
		curr_edge.getInformationMatrix(inf_mat);

		if (inf_mat == CMatrixDouble33()) {
			inf_mat.unit();
			curr_edge.cov_inv = inf_mat;
		}

		TPath curr_path(from); // set the starting node
		curr_path.addToPath(to, curr_edge);

		// update the resulting path_between_nodes if its determinant is smaller
		// than the determinant of the current path_between_nodes
		if (curr_determinant < curr_path.getDeterminant()) {
			curr_determinant = curr_path.getDeterminant();
			*path_between_nodes = curr_path;
		}
	}
	// backwards edges to -> from
	std::pair<edges_citerator, edges_citerator> bwd_edges_pair =
		m_graph->getEdges(to, from);

	for (edges_citerator edges_it = bwd_edges_pair.first;
			edges_it != bwd_edges_pair.second; ++edges_it) {
		// operate on a temporary object instead of the real edge - otherwise
		// function is non-const
		constraint_t curr_edge;
		(edges_it->second).inverse(curr_edge);

		// is it all 0s?
		CMatrixDouble33 inf_mat;
		curr_edge.getInformationMatrix(inf_mat);

		if (inf_mat == CMatrixDouble33()) {
			inf_mat.unit();
			curr_edge.cov_inv = inf_mat;
		}

		TPath curr_path(from); // set the starting node
		curr_path.addToPath(to, curr_edge);

		// update the resulting path_between_nodes if its determinant is smaller
		// than the determinant of the current path_between_nodes
		if (curr_determinant < curr_path.getDeterminant()) {
			curr_determinant = curr_path.getDeterminant();
			*path_between_nodes = curr_path;
		}
	}

	// TODO - add meaningfull assertions here...

	MRPT_END;
}

template<class GRAPH_t>
typename CLoopCloserERD<GRAPH_t>::TPath* CLoopCloserERD<GRAPH_t>::
popMinUncertaintyPath(std::set<TPath*>* pool_of_paths) const {
	MRPT_START;
	using namespace std;

	//cout << "Determinants: ";
	TPath* optimal_path = NULL;
	double curr_determinant = 0;
	for (typename std::set<TPath*>::const_iterator it =pool_of_paths->begin();
			it != pool_of_paths->end(); ++it) {
		//cout << (*it)->getDeterminant() << ", ";

		// keep the largest determinant - we are in INFORMATION form.
		if (curr_determinant < (*it)->getDeterminant()) {
			curr_determinant = (*it)->getDeterminant();
			optimal_path = *it;
		}
	}

	ASSERT_(optimal_path);
	pool_of_paths->erase(optimal_path); // erase it from the pool

	return optimal_path;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) {
	MRPT_START;
	using namespace mrpt::utils;

	m_edge_types_to_nums["ICP2D"]++;
	m_out_logger.logFmt("Registering new edge: %lu => %lu\n"
			"rel_edge: \t%s\n"
			"norm:     \t%f\n", from, to, 
			rel_edge.getMeanVal().asString().c_str(),
			rel_edge.getMeanVal().norm());

	if (abs(to - from) > m_lc_params.LC_min_nodeid_diff)  {
		m_edge_types_to_nums["LC"]++;
		m_just_inserted_loop_closure = true;
		m_out_logger.log("\tLoop Closure edge!", LVL_INFO);
	}

	m_graph->insertEdge(from,  to, rel_edge);
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	MRPT_START;
	m_graph = graph;
	m_out_logger.log("Fetched the graph successfully");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setRawlogFname(const std::string& rawlog_fname){
	MRPT_START;

	m_rawlog_fname = rawlog_fname;
	m_out_logger.logFmt("Fetched the rawlog filename successfully: %s",
			m_rawlog_fname.c_str());

	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	m_win_manager = win_manager;

	// may still be null..
	if (m_win_manager) {
		m_win = m_win_manager->win;
		m_win_observer = m_win_manager->observer;

		if (m_win_observer) {
			m_win_observer->registerKeystroke(m_laser_params.keystroke_laser_scans,
					"Toggle LaserScans Visualization");
			m_win_observer->registerKeystroke(m_lc_params.keystroke_map_partitions,
					"Toggle Map Partitions Visualization");

		}

		m_out_logger.log("Fetched the window manager, window observer  successfully.");
	}

}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;

	// laser scans
	if (events_occurred.at(m_laser_params.keystroke_laser_scans)) {
		this->toggleLaserScansVisualization();
	}
	// map partitions
	if (events_occurred.at(m_lc_params.keystroke_map_partitions)) {
		this->toggleMapPartitionsVisualization();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initMapPartitionsVisualization() {
	using namespace mrpt;
	using namespace mrpt::gui;
	using namespace mrpt::math;
	using namespace mrpt::opengl;

	// textmessage
	m_win_manager->assignTextMessageParameters(
			/* offset_y*	= */ &m_lc_params.offset_y_map_partitions,
			/* text_index* = */ &m_lc_params.text_index_map_partitions);

	// just add an empty CSetOfObjects in the scene - going to populate it later
	CSetOfObjectsPtr map_partitions_obj = CSetOfObjects::Create();
	map_partitions_obj->setName("map_partitions");

	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();
	scene->insert(map_partitions_obj);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateMapPartitionsVisualization() {
	using namespace mrpt;
	using namespace mrpt::gui;
	using namespace mrpt::math;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;

	// textmessage
	// ////////////////////////////////////////////////////////////
	std::stringstream title;
	title << "# Partitions: " << m_curr_partitions.size();
	m_win_manager->addTextMessage(5,-m_lc_params.offset_y_map_partitions,
			title.str(),
			mrpt::utils::TColorf(m_lc_params.balloon_std_color),
			/* unique_index = */ m_lc_params.text_index_map_partitions);

	// update the partitioning visualization
	// ////////////////////////////////////////////////////////////
	COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

	// fetch the partitions CSetOfObjects
	CSetOfObjectsPtr map_partitions_obj;
	{
		CRenderizablePtr obj = scene->getByName("map_partitions");
		// do not check for null ptr - must be properly created in the init* method
		map_partitions_obj = static_cast<CSetOfObjectsPtr>(obj);
	}

	int partitionID = 0;
	bool partition_contains_last_node = false;
	for (partitions_t::const_iterator p_it = m_curr_partitions.begin();
			p_it != m_curr_partitions.end(); ++p_it, ++partitionID) {
		m_out_logger.logFmt("Working on Partition #%d", partitionID);
		vector_uint nodes_list = *p_it;

		// fetch the current partition object if it exists - create otherwise
		std::string partition_obj_name = mrpt::format("partition_%d", partitionID);
		std::string balloon_obj_name = mrpt::format("#%d", partitionID);

		CRenderizablePtr obj = map_partitions_obj->getByName(partition_obj_name);
		CSetOfObjectsPtr curr_partition_obj;
		if (obj) {
			m_out_logger.logFmt(
					"\tFetching CSetOfObjects partition object for partition #%d",
					partitionID);
			curr_partition_obj = static_cast<CSetOfObjectsPtr>(obj);
		}
		else {
			m_out_logger.logFmt(
					"\tCreating a new CSetOfObjects partition object for partition #%d",
					partitionID);
			curr_partition_obj = CSetOfObjects::Create();
			curr_partition_obj->setName(partition_obj_name);

			m_out_logger.logFmt("\t\tCreating a new CSphere balloon object");
			CSpherePtr balloon_obj = CSphere::Create();
			balloon_obj->setName(balloon_obj_name);
			balloon_obj->setRadius(m_lc_params.balloon_radius);
			balloon_obj->setColor_u8(m_lc_params.balloon_std_color);
			balloon_obj->enableShowName();

			curr_partition_obj->insert(balloon_obj);

			// set of lines connecting the graph nodes to the balloon
			m_out_logger.logFmt("\t\tCreating set of lines that will connect to the Balloon");
			CSetOfLinesPtr connecting_lines_obj = CSetOfLines::Create();
			connecting_lines_obj->setName("connecting_lines");
			connecting_lines_obj->setColor_u8(m_lc_params.connecting_lines_color);
			connecting_lines_obj->setLineWidth(0.1);

			curr_partition_obj->insert(connecting_lines_obj);

			// add the created CSetOfObjects to the total CSetOfObjects responsible
			// for the map partitioning
			map_partitions_obj->insert(curr_partition_obj);
			m_out_logger.logFmt("\tInserted new CSetOfObjects successfully");
		}
		// up to now the CSetOfObjects exists and the balloon inside it as well..

		std::pair<double, double> centroid_coords;
		this->computeCentroidOfNodesVector(nodes_list, &centroid_coords);

		// finding the partition in which the last node is in
		if (std::find(nodes_list.begin(), nodes_list.end(), m_graph->nodeCount()-1)
				!= nodes_list.end()) {
			partition_contains_last_node = true;
		}
		else {
			partition_contains_last_node = false;
		}
		TPoint3D balloon_location(centroid_coords.first, centroid_coords.second,
				m_lc_params.balloon_elevation);

		m_out_logger.logFmt("\tUpdating the balloon position");
		// set the balloon properties
		CSpherePtr balloon_obj;
		{
			// place the partitions baloon at the centroid elevated by a fixed Z value
			CRenderizablePtr obj = curr_partition_obj->getByName(balloon_obj_name);
			balloon_obj = static_cast<CSpherePtr>(obj);
			balloon_obj->setLocation(balloon_location);
			if (partition_contains_last_node)
				balloon_obj->setColor_u8(m_lc_params.balloon_curr_color);
			else
				balloon_obj->setColor_u8(m_lc_params.balloon_std_color);
		}

		m_out_logger.logFmt("\tUpdating the lines connecting nodes to balloon");
		// set the lines connecting the nodes of the partition to the partition
		// balloon - set it from scratch all the times since the node positions
		// tend to change according to the dijkstra position estimation
		CSetOfLinesPtr connecting_lines_obj;
		{
			// place the partitions baloon at the centroid elevated by a fixed Z value
			CRenderizablePtr obj = curr_partition_obj->getByName("connecting_lines");
			connecting_lines_obj = static_cast<CSetOfLinesPtr>(obj);

			connecting_lines_obj->clear();

			for (vector_uint::const_iterator it = nodes_list.begin();
					it != nodes_list.end(); ++it) {
				CPose3D curr_pose(m_graph->nodes.at(*it));
				TPoint3D curr_node_location(curr_pose);

				TSegment3D connecting_line(curr_node_location, balloon_location);
				connecting_lines_obj->appendLine(connecting_line);
			}

		}
		m_out_logger.logFmt("Done working on partition #%d", partitionID);
	}

	// remove outdated partitions
	// these occur when more partitions existed during the previous visualization
	// update, thus the partitions with higher ID than the maximum partitionID
	// would otherwise remain in the visual as zombie partitions
	size_t prev_size = m_last_partitions.size();
	size_t curr_size = m_curr_partitions.size();
	if (curr_size < prev_size) {
		m_out_logger.logFmt("Removing outdated partitions in visual");
		for (int partitionID = curr_size; partitionID != prev_size; ++partitionID) {
			m_out_logger.logFmt("\tRemoving partition %d", partitionID);
			std::string partition_obj_name = mrpt::format("partition_%d", partitionID);

			CRenderizablePtr obj = map_partitions_obj->getByName(partition_obj_name);
			map_partitions_obj->removeObject(obj);
		}
	}
	m_out_logger.logFmt("Done working on the partitions visualization.");


	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::toggleMapPartitionsVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");
	using namespace mrpt::utils;
	using namespace mrpt::opengl;

	m_out_logger.log("Toggling map partitions  visualization...", LVL_INFO);
	mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (m_lc_params.visualize_map_partitions) {
		mrpt::opengl::CRenderizablePtr obj = scene->getByName("map_partitions");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		this->dumpVisibilityErrorMsg("visualize_map_partitions");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::computeCentroidOfNodesVector(
		const vector_uint& nodes_list,
		std::pair<double, double>* centroid_coords) const {
	MRPT_START;

	// get the poses and find the centroid so that we can place the baloon over
	// and at their center
	double centroid_x = 0;
	double centroid_y = 0;
	for (vector_uint::const_iterator node_it = nodes_list.begin();
			node_it != nodes_list.end(); ++node_it) {
		pose_t curr_node_pos = m_graph->nodes.at(*node_it);
		centroid_x +=  curr_node_pos.x();
		centroid_y +=  curr_node_pos.y();

	}

	// normalize by the size - assign to the given pair
	centroid_coords->first = centroid_x/static_cast<double>(nodes_list.size());
	centroid_coords->second = centroid_y/static_cast<double>(nodes_list.size());

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initLaserScansVisualization() {
	MRPT_START;


	// laser scan visualization
	if (m_laser_params.visualize_laser_scans) {
		mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz = 
			mrpt::opengl::CPlanarLaserScan::Create();
		laser_scan_viz->enablePoints(true);
		laser_scan_viz->enableLine(true);
		laser_scan_viz->enableSurface(true);
		laser_scan_viz->setSurfaceColor(
				m_laser_params.laser_scans_color.R,
				m_laser_params.laser_scans_color.G,
				m_laser_params.laser_scans_color.B,
				m_laser_params.laser_scans_color.A);

		laser_scan_viz->setName("laser_scan_viz");

		scene->insert(laser_scan_viz);
		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateLaserScansVisualization() {
	MRPT_START;

	// update laser scan visual
	if (m_laser_params.visualize_laser_scans && !m_last_laser_scan2D.null()) {
		mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

		mrpt::opengl::CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz =
			static_cast<mrpt::opengl::CPlanarLaserScanPtr>(obj);

		laser_scan_viz->setScan(*m_last_laser_scan2D);

		// set the pose of the laser scan
		typename GRAPH_t::global_poses_t::const_iterator search =
			m_graph->nodes.find(m_graph->nodeCount()-1);
		if (search != m_graph->nodes.end()) {
			laser_scan_viz->setPose(m_graph->nodes[m_graph->nodeCount()-1]);
			// put the laser scan underneath the graph, so that you can still
			// visualize the loop closures with the nodes ahead
			laser_scan_viz->setPose(mrpt::poses::CPose3D(
						laser_scan_viz->getPoseX(), laser_scan_viz->getPoseY(), -0.3,
						mrpt::utils::DEG2RAD(laser_scan_viz->getPoseYaw()),
						mrpt::utils::DEG2RAD(laser_scan_viz->getPosePitch()),
						mrpt::utils::DEG2RAD(laser_scan_viz->getPoseRoll())
						));
		}

		m_win->unlockAccess3DScene();
		m_win->forceRepaint();
	}

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::toggleLaserScansVisualization() {
	MRPT_START;
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");
	using namespace mrpt::utils;

	m_out_logger.log("Toggling LaserScans visualization...", LVL_INFO);

	mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();

	if (m_laser_params.visualize_laser_scans) {
		mrpt::opengl::CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		this->dumpVisibilityErrorMsg("visualize_laser_scans");
	}

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getEdgesStats(
		std::map<const std::string, int>* edge_types_to_num) const {
	MRPT_START;
	*edge_types_to_num = m_edge_types_to_nums;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	m_out_logger.log("Initializing visuals");
	m_time_logger.enter("Visuals");

	ASSERTMSG_(m_laser_params.has_read_config,
			"Configuration parameters aren't loaded yet");
	ASSERTMSG_(m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(m_win_manager, "No CWindowManager* was provided");
	ASSERTMSG_(m_win_observer, "No CWindowObserver* was provided");

	if (m_laser_params.visualize_laser_scans) {
		this->initLaserScansVisualization();
	}
	if (m_lc_params.visualize_map_partitions) {
		this->initMapPartitionsVisualization();
	}

	if (m_visualize_curr_node_covariance) {
		this->initCurrCovarianceVisualization();
	}

	m_initialized_visuals = true;
	m_time_logger.leave("Visuals");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateVisuals() {
	MRPT_START;
	ASSERT_(m_initialized_visuals);
	m_out_logger.log("Updating visuals");
	m_time_logger.enter("Visuals");

	if (m_laser_params.visualize_laser_scans) {
	this->updateLaserScansVisualization();
	}
	if (m_lc_params.visualize_map_partitions) {
		this->updateMapPartitionsVisualization();
	}
	if (m_visualize_curr_node_covariance) {
		this->updateCurrCovarianceVisualization();
	}

	m_time_logger.leave("Visuals");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initCurrCovarianceVisualization() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::opengl;

	// text message for covariance ellipsis
	m_win_manager->assignTextMessageParameters(
			/* offset_y*	= */ &m_offset_y_curr_node_covariance,
			/* text_index* = */ &m_text_index_curr_node_covariance);

	std::string title("Position uncertainty");
	m_win_manager->addTextMessage(5,-m_offset_y_curr_node_covariance,
			title,
			mrpt::utils::TColorf(m_curr_node_covariance_color),
			/* unique_index = */ m_text_index_curr_node_covariance);


	// covariance ellipsis
	CEllipsoidPtr cov_ellipsis_obj = CEllipsoid::Create();
	cov_ellipsis_obj->setName("cov_ellipsis_obj");
	cov_ellipsis_obj->setColor_u8(m_curr_node_covariance_color);
	cov_ellipsis_obj->setLocation(0, 0, 0);
	//cov_ellipsis_obj->setQuantiles(2.0);

	mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	scene->insert(cov_ellipsis_obj);
	m_win->unlockAccess3DScene();
	m_win->forceRepaint();
	
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateCurrCovarianceVisualization() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::opengl;
	using namespace mrpt::gui;

	// get the optimal path to the current node
	mrpt::utils::TNodeID curr_node = m_graph->nodeCount()-1;
	TPath* path =	queryOptimalPath(curr_node);
	if (!path) return;


	ASSERT_(path);
	CMatrixDouble33 mat;
	path->curr_pose_pdf.getCovariance(mat);
	pose_t curr_position = m_graph->nodes.at(curr_node);

	stringstream ss_mat; ss_mat << mat;
	m_out_logger.logFmt("In updateCurrCovarianceVisualization\n"
			"Covariance matrix:\n%s\n"
			"determinant : %f", ss_mat.str().c_str(), mat.det() );

	mrpt::opengl::COpenGLScenePtr scene = m_win->get3DSceneAndLock();
	CRenderizablePtr obj = scene->getByName("cov_ellipsis_obj");
	CEllipsoidPtr cov_ellipsis_obj = static_cast<CEllipsoidPtr>(obj);

	// TODO - try setting the pose using the curr_pose_pdf.getMeanVal()..
	// set the pose and corresponding covariance matrix of the ellipsis
	cov_ellipsis_obj->setLocation(curr_position.x(), curr_position.y(), 0);
	//pose_t loc = path->curr_pose_pdf.getMeanVal();
	//cov_ellipsis_obj->setLocation(loc.x(), loc.y(), 0);
	cov_ellipsis_obj->setCovMatrix(mat, 2);

	m_win->unlockAccess3DScene();
	m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::justInsertedLoopClosure() const {
	return m_just_inserted_loop_closure;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt::obs;

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
		// TODO - what if it's in this format but only has odometry information?
		m_checked_for_usuable_dataset = true;
		return;
	}
	if (m_consecutive_invalid_format_instances > 
			m_consecutive_invalid_format_instances_thres) {
		m_out_logger.log("Can't find usuable data in the given dataset.\nMake sure dataset contains valid CObservation2DRangeScan/CObservation3DRangeScan data.",
				mrpt::utils::LVL_ERROR);
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time /* = 500 milliseconds */) {
	MRPT_START;

	m_out_logger.log(format("Cannot toggle visibility of specified object.\n "
			"Make sure that the corresponding visualization flag ( %s "
			") is set to true in the .ini file.\n",
			viz_flag.c_str()).c_str(), mrpt::utils::LVL_ERROR);
	mrpt::system::sleep(sleep_time);

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	m_partitioner.options.loadFromConfigFileName(source_fname,
			"EdgeRegistrationDeciderParameters");
	m_laser_params.loadFromConfigFileName(source_fname,
			"EdgeRegistrationDeciderParameters");
	m_lc_params.loadFromConfigFileName(source_fname,
			"EdgeRegistrationDeciderParameters");
	range_scanner_t::params.loadFromConfigFileName(source_fname, "ICP");

	// set the logging level if given by the user
	mrpt::utils::CConfigFile source(source_fname);
	int min_verbosity_level = source.read_int(
			"EdgeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	m_out_logger.setMinLoggingLevel(mrpt::utils::VerbosityLevel(min_verbosity_level));

	m_out_logger.log("Successfully loaded parameters. ");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::printParams() const {
	MRPT_START;

	std::cout << "------------------[Pair-wise Consistency of ICP Edges - Registration Procedure Summary]------------------" << std::endl;

	m_partitioner.options.dumpToConsole();
	m_laser_params.dumpToConsole();
	m_lc_params.dumpToConsole();
	range_scanner_t::params.dumpToConsole();

	m_out_logger.log("Printed the relevant parameters");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	std::stringstream class_props_ss;
	class_props_ss << "Pair-wise Consistency of ICP Edges - Registration Procedure Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = m_out_logger.getAsString();

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

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateMapPartitions(bool full_update /* = false */) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	m_time_logger.enter("updateMapPartitions");
	
 nodes_to_scans2D_t nodes_to_scans;
	if (full_update) {
		m_out_logger.log("updateMapPartitions: Full partitioning of map was issued", LVL_INFO);

		// clear the existing partitions and recompute the partitioned map for all
		// the nodes
		m_partitioner.clear();
		nodes_to_scans = m_nodes_to_laser_scans2D;
	}
	else {
		// just use the last node-laser scan pair
		nodes_to_scans[0] = m_nodes_to_laser_scans2D.at(m_graph->nodeCount()-1);
	}

	// for each one of the above nodes - add its position and correspoding
	// laserScan to the partitioner object
	for (nodes_to_scans2D_t::const_iterator it = nodes_to_scans.begin();
			it != nodes_to_scans.end(); ++it) {
		if ((it->second).null()) { continue; } // if laserScan invalud go to next...


		// pose
		pose_t curr_pose = m_graph->nodes.at(it->first);
		mrpt::poses::CPosePDFPtr posePDF(new constraint_t(curr_pose));

		// laser scan
		mrpt::obs::CSensoryFramePtr sf = mrpt::obs::CSensoryFrame::Create();
		sf->insert(it->second);

		m_partitioner.addMapFrame(sf, posePDF);
	}

	// update the last partitions list
	size_t n = m_curr_partitions.size();
	m_last_partitions.resize(n);
	for (int i = 0; i < n; i++)	{
		m_last_partitions[i] = m_curr_partitions[i];
	}
	//update current partitions list
	m_partitioner.updatePartitions(m_curr_partitions);

	m_out_logger.log("Updated map partitions successfully.");
	m_time_logger.leave("updateMapPartitions");
	MRPT_END;
}


template<class GRAPH_t>
template<class T>
void CLoopCloserERD<GRAPH_t>::printVectorOfVectors(const T& t) {
	int i = 0;
	for (typename T::const_iterator it = t.begin(); it  != t.end(); ++i, ++it) {
		printf("Vector %d/%lu:\n\t", i, t.size());
		CLoopCloserERD<GRAPH_t>::printVector(*it);
	}
}

template<class GRAPH_t>
template<class T>
void CLoopCloserERD<GRAPH_t>::printVector(const T& t) {
	std::cout << CLoopCloserERD<GRAPH_t>::getVectorAsString(t) << std::endl;
}

template<class GRAPH_t>
template<class T>
std::string CLoopCloserERD<GRAPH_t>::getVectorAsString(const T& t) {
	using namespace std;
	stringstream ss;
	for (typename T::const_iterator it = t.begin(); it != t.end(); ++it) {
		ss << *it << ", ";
	}
	return ss.str();
}

// TLaserParams
// //////////////////////////////////


template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TLaserParams::TLaserParams():
	laser_scans_color(0, 20, 255),
	keystroke_laser_scans("l"),
	has_read_config(false)
{ }

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TLaserParams::~TLaserParams() { }

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TLaserParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("Use scan-matching constraints               = %s\n",
			use_scan_matching? "TRUE": "FALSE");
	out.printf("ICP goodness threshold                      = %.2f%% \n",
			ICP_goodness_thresh*100);
	out.printf("Num. of previous nodes to check ICP against =  %d\n",
			prev_nodes_for_ICP);
	out.printf("Visualize laser scans                       = %s\n",
			visualize_laser_scans? "TRUE": "FALSE");

	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TLaserParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
		const std::string& section) {
	MRPT_START;

	use_scan_matching = source.read_bool(
			section,
			"use_scan_matching",
			true, false);
	ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
			0.75, false);
	prev_nodes_for_ICP = source.read_int( // how many nodes to check ICP against
			section,
			"prev_nodes_for_ICP",
			10, false);
	visualize_laser_scans = source.read_bool(
			"VisualizationParameters",
			"visualize_laser_scans",
			true, false);


	has_read_config = true;
	MRPT_END;
}
// TLoopClosureParams
// //////////////////////////////////


template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TLoopClosureParams::TLoopClosureParams():
	keystroke_map_partitions("b"),
	balloon_elevation(3),
	balloon_radius(0.5),
	balloon_std_color(153, 0, 153),
	balloon_curr_color(102, 0, 102),
	connecting_lines_color(balloon_std_color),
	has_read_config(false)
{ 
}

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TLoopClosureParams::~TLoopClosureParams() { }

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TLoopClosureParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("Min. node difference for LC = %d\n", LC_min_nodeid_diff);
	out.printf("Visualize map partitions    = %s\n", visualize_map_partitions?
			"TRUE": "FALSE");

	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TLoopClosureParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
		const std::string& section) {
	MRPT_START;

	LC_min_nodeid_diff = source.read_int(
			"GeneralConfiguration",
			"LC_min_nodeid_diff",
			30, false);
	visualize_map_partitions = source.read_bool(
			"VisualizationParameters",
			"visualize_map_partitions",
			true, false);

	has_read_config = true;
	MRPT_END;
}

// TPath
// //////////////////////////////////

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TPath::TPath() {
	this->clear();
}
template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TPath::TPath(mrpt::utils::TNodeID starting_node) {
	this->clear();

	nodes_traversed.push_back(starting_node);
}
template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TPath::~TPath() { }

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TPath::clear() {
	using namespace mrpt;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	// clear the vector of traversed nodes
	nodes_traversed.clear();

	// clear the relative edge
	curr_pose_pdf.mean = pose_t();
	// by default the information matrix is set to the unit matrix
	CMatrixDouble33 init_path_mat; init_path_mat.unit();
	// put a really large number - we are certain of this position
	init_path_mat *= 10000; // justify this..
	curr_pose_pdf.cov_inv = init_path_mat;

	determinant_updated = false;
	determinant_cached = 0;

}

template<class GRAPH_t>
typename CLoopCloserERD<GRAPH_t>::TPath& CLoopCloserERD<GRAPH_t>::TPath::
operator+=(const CLoopCloserERD<GRAPH_t>::TPath& other) {
	MRPT_START;

	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	// other should start where this ends
	ASSERTMSG_(other.nodes_traversed.begin()[0] ==
			this->nodes_traversed.rbegin()[0],
			"\"other\" instance must start from the nodeID that this "
			"TPath has ended.");
	ASSERTMSG_(other.nodes_traversed.size(),
			"\"other\" instance doesn't have an initialized nodes traversal list");
	ASSERTMSG_(this->nodes_traversed.size(),
			"\"this\" instance doesn't have an initialized nodes traversal list");

	//////// TODO Remove these - >>>>>>>>>>>>>>>>>>>>>
	//cout << string(20, '-') << "Aggregating 2 paths.."
		//<< string(20, '-') << endl;
	//this->dumpToConsole(); other.dumpToConsole();
	////// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	// aggregate the two gaussian - mean & information matrix
	this->curr_pose_pdf += other.curr_pose_pdf;

	// add the traversed nodes
	this->nodes_traversed.insert(
			this->nodes_traversed.end(),
			other.nodes_traversed.begin()+1,
			other.nodes_traversed.end());

	////// TODO Remove these - >>>>>>>>>>>>>>>>>>>>>
	//cout << std::string(10, '%') << endl << "AFTER Aggregation..." << endl;
	//this->dumpToConsole();
	//cout << string(50, '-') << endl;
	//mrpt::system::pause();
	////// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

	determinant_updated = false;
	return *this;

	MRPT_END;
}
template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::TPath::operator==(
		const CLoopCloserERD<GRAPH_t>::TPath& other) {
		MRPT_START;

		// check if the traversed nodes are the same as well as the
		// CPoseGaussianInfs are the same..
		return ( this->nodes_traversed == other.nodes_traversed &&
				this->curr_pose_pdf == other.curr_pose_pdf );
		
		MRPT_END;
}
template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::TPath::operator!=(
		const CLoopCloserERD<GRAPH_t>::TPath& other) {
		MRPT_START;

		return !(*this == other);

		MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TPath::addToPath(
		mrpt::utils::TNodeID node, constraint_t edge) {
	using namespace std;

	// update the path
	curr_pose_pdf += edge;

	// update the traversed nodes
	nodes_traversed.push_back(node);

	determinant_updated = false;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TPath::loadFromConfigFile( 
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TPath::dumpToTextStream(
		mrpt::utils::CStream &out) const {

	out.printf("%s\n", this->getAsString().c_str());

}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TPath::getAsString(std::string* str) const{
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace std;

	stringstream ss;
	string header_sep(30, '=');

	ss << "Path properties: " << endl;
	ss << header_sep << endl << endl;

	ss << "- CPosePDFGaussianInf: "
		<< (this->isGaussianInfType()?  "TRUE" : "FALSE") << endl;
	ss << "- Nodes list: \n\t< " <<
		CLoopCloserERD<GRAPH_t>::getVectorAsString(nodes_traversed)
		<< "\b\b>" << endl;

	ss << endl;
	ss << curr_pose_pdf << endl;
	ss << endl;

	CMatrixDouble33 mat;
	if (this->isGaussianType()) {
		curr_pose_pdf.getCovariance(mat);
	}
	else if (this->isGaussianInfType()) {
		curr_pose_pdf.getInformationMatrix(mat);
	}
	ss << "Determinant: " << mat.det();

	*str = ss.str();
}
template<class GRAPH_t>
std::string CLoopCloserERD<GRAPH_t>::TPath::getAsString() const {
	std::string s;
	this->getAsString(&s);
	return s;
}

template<class GRAPH_t>
mrpt::utils::TNodeID CLoopCloserERD<GRAPH_t>::TPath::getSource() const {
	return nodes_traversed.at(0);
}
template<class GRAPH_t>
mrpt::utils::TNodeID CLoopCloserERD<GRAPH_t>::TPath::getDestination() const {
	return nodes_traversed.back();
}

template<class GRAPH_t>
double CLoopCloserERD<GRAPH_t>::TPath::getDeterminant() {
	MRPT_START;

	using namespace mrpt::math;
	using namespace std;

	// if determinant is up-to-date then return the cached version...
	if (determinant_updated) return determinant_cached;

	CMatrixDouble33 mat;
	if (this->isGaussianInfType()) {
		curr_pose_pdf.getInformationMatrix(mat);
	}
	else if (this->isGaussianType()) {
		curr_pose_pdf.getCovariance(mat);
	}
	double determinant = mat.det();

	// update the cached version
	determinant_cached = determinant;
	determinant_updated = true;


	return determinant;

	MRPT_END;
}

template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::TPath::hasLowerUncertaintyThan(
		const TPath& other) const {
	ASSERT_((this->isGaussianInfType() && other->isGaussianInfType()) ||
			(this->isGaussianType() && other->isGaussianType()) );

	// If we are talking about information form matrices, the *higher* the 
	// determinant the better.
	// if we are talking about covariances then the *lower* the determinant the
	// better.
	bool has_lower = false;
	if (this->isGaussianInfType()) {
		has_lower = this->getDeterminant() > other->getDeterminant();
	}
	else if (this->isGaussianType()) {
		has_lower = this->getDeterminant() < other->getDeterminant();
	}

	return has_lower;
}

template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::TPath::isGaussianInfType() const {
	using namespace mrpt::poses;
	return curr_pose_pdf.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf);
}
template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::TPath::isGaussianType() const {
	using namespace mrpt::poses;
	return curr_pose_pdf.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian);
}

} } } // end of namespaces

#endif /* end of include guard: CLOOPCLOSERERD_IMPL_H */
