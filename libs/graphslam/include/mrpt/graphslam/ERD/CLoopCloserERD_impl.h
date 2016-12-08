/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
	m_curr_node_covariance_color(160, 160, 160, /*alpha = */255)
{
	this->initCLoopCloserERD();
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initCLoopCloserERD() {
	MRPT_START;
	this->initializeLoggers("CLoopCloserERD");

	m_visualize_curr_node_covariance = false;

	// start the edge registration procedure only when this num is surpassed
	// nodeCount > m_last_total_num_of_nodes
	m_threshold_to_start = m_last_total_num_of_nodes = 0;

	m_edge_types_to_nums["ICP2D"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	m_partitions_full_update = false;

	MRPT_LOG_DEBUG_STREAM << "Initialized class object";
	MRPT_END;
}

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::~CLoopCloserERD() {
	using namespace mrpt::graphslam;

	// release memory of m_node_optimal_paths map.
	MRPT_LOG_DEBUG_STREAM << "Releasing memory of m_node_optimal_paths map...";
	for (typename std::map<
			mrpt::utils::TNodeID,
			TUncertaintyPath<GRAPH_t>*>::iterator
			it = m_node_optimal_paths.begin();
			it != m_node_optimal_paths.end();
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
	this->m_time_logger.enter("updateState");
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	// check possible prior node registration
	bool registered_new_node = false;

	// was a new node registered?
	if (m_last_total_num_of_nodes < this->m_graph->nodeCount()) {
		registered_new_node = true;
		m_last_total_num_of_nodes = this->m_graph->nodeCount();
		MRPT_LOG_DEBUG_STREAM << "New node has been registered!";
	}

	// update last laser scan to use
	if (observation.present()) { // observation-only rawlog format
		if (IS_CLASS(observation, CObservation2DRangeScan)) {
			m_last_laser_scan2D = 
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);

		}
	}
	else { // action-observations format
		// action part

		// observation part
		CObservationPtr curr_observation =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (curr_observation) {
			m_last_laser_scan2D =
				observations->getObservationByClass<CObservation2DRangeScan>();
		}
	}


	if (registered_new_node) {
		// register the new node-laserScan pair
		m_nodes_to_laser_scans2D[this->m_graph->nodeCount()-1] =
			m_last_laser_scan2D;

		if (m_laser_params.use_scan_matching) {
			// scan match with previous X nodes
			this->addScanMatchingEdges(this->m_graph->nodeCount()-1);
		}

		// update the partitioned map
		m_partitions_full_update = (
				(this->m_graph->nodeCount() % m_lc_params.full_partition_per_nodes) == 0 ||
				m_just_inserted_loop_closure)
			?  true: false;
		this->updateMapPartitions(m_partitions_full_update);

		// check for loop closures
		partitions_t partitions_for_LC;
		this->checkPartitionsForLC(&partitions_for_LC);
		this->evaluatePartitionsForLC(partitions_for_LC);

		if (m_visualize_curr_node_covariance) {
			this->execDijkstraProjection();
		}

	}

	this->m_time_logger.leave("updateState");
	return true;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::addScanMatchingEdges(
		mrpt::utils::TNodeID curr_nodeID) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::math;

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

	MRPT_LOG_DEBUG_STREAM("Adding ICP Constraints for nodeID: " <<
		curr_nodeID);

	// try adding ICP constraints with each node in the previous set
	for (std::set<TNodeID>::const_iterator node_it = nodes_set.begin();
			node_it != nodes_set.end(); ++node_it) {

		constraint_t rel_edge;
		mrpt::slam::CICP::TReturnInfo icp_info;

		MRPT_LOG_DEBUG_STREAM("Fetching laser scan for nodes: " << *node_it << "
				==> " << curr_nodeID);

		bool success = this->getICPEdge(
				*node_it,
				curr_nodeID,
				&rel_edge,
				&icp_info);
		if (!success) continue;

		// keep track of the recorded goodness values
		// TODO - rethink on these condition.
		if (!isNaN(icp_info.goodness) || icp_info.goodness != 0) {
			m_laser_params.goodness_threshold_win.addNewMeasurement(icp_info.goodness);
		}
		double goodness_thresh = m_laser_params.goodness_threshold_win.getMedian()*0.9;
		bool accept_goodness = icp_info.goodness > goodness_thresh;
		MRPT_LOG_DEBUG_STREAM("Curr. Goodness: " << icp_info.goodness
				<< "|\t Threshold: " << goodness_thresh << " => " << (accept_goodness? "ACCEPT" : "REJECT"));

		// make sure that the suggested edge makes sense with regards to current
		// graph config - check against the current position difference
		bool accept_mahal_distance = this->mahalanobisDistanceOdometryToICPEdge(
				*node_it, curr_nodeID, rel_edge);

		// criterion for registering a new node
		if (accept_goodness && accept_mahal_distance) {
			this->registerNewEdge(*node_it, curr_nodeID, rel_edge);
		}
	}

	MRPT_END;
}
template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::getICPEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		constraint_t* rel_edge,
		mrpt::slam::CICP::TReturnInfo* icp_info) {
	MRPT_START;
	ASSERT_(rel_edge);
	this->m_time_logger.enter("getICPEdge");

	using namespace mrpt::obs;
	using namespace mrpt::utils;

	// fetch the relevant laser scans
	nodes_to_scans2D_t::const_iterator search;
	CObservation2DRangeScanPtr from_laser_scan, to_laser_scan;
	search = m_nodes_to_laser_scans2D.find(from);
	if (search != m_nodes_to_laser_scans2D.end()) {
		from_laser_scan = search->second;
	}
	search = m_nodes_to_laser_scans2D.find(to);
	if (search != m_nodes_to_laser_scans2D.end()) {
		to_laser_scan = search->second;
	}

	// what if invalid Laser Scans?
	//ASSERT_(from_laser_scan.present());
	//ASSERT_(to_laser_scan.present());
	if (!from_laser_scan.present() || !to_laser_scan.present()) {
		MRPT_LOG_DEBUG_STREAM(
			"Either node #" << from <<
			"or node #" << to <<
			"doesn't contain a valid LaserScan. Ignoring this...");
		return false;
	}

	// make use of initial node position difference for the ICP edge
	pose_t initial_estim = this->m_graph->nodes.at(to) -
		this->m_graph->nodes.at(from);

	range_scanner_t::getICPEdge(
			*from_laser_scan,
			*to_laser_scan,
			rel_edge,
			&initial_estim,
			icp_info);

	this->m_time_logger.leave("getICPEdge");
	return true;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::checkPartitionsForLC(
		partitions_t* partitions_for_LC) {
	MRPT_START;
	this->m_time_logger.enter("LoopClosureEvaluation");

	using namespace std;
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	ASSERT_(partitions_for_LC);
	partitions_for_LC->clear();

	// keep track of the previous nodes list of every partition. If this is not
	// changed - do not mark it as potential for loop closure
	map<int, vector_uint>::iterator finder;
	// reset the previous list if full partitioning was issued
	if (m_partitions_full_update) {
		m_partitionID_to_prev_nodes_list.clear();
	}

	int partitionID = 0;
	// for every partition...
	for (partitions_t::const_iterator
			partitions_it = m_curr_partitions.begin();
			partitions_it != m_curr_partitions.end();
			++partitions_it, ++partitionID)
	{
		// check whether the last registered node is in the currently traversed
		// partition - if not, ignore it.
		if (m_lc_params.LC_check_curr_partition_only) {
			bool curr_node_in_curr_partition =
				((find(partitions_it->begin(),
							 partitions_it->end(),
							 this->m_graph->nodeCount()-1))
				 != partitions_it->end());
			if (!curr_node_in_curr_partition) {
				continue;
			}
		}

		// keep track of the previous nodes list
		finder = m_partitionID_to_prev_nodes_list.find(partitionID);
		if (finder == m_partitionID_to_prev_nodes_list.end()) {
 			// nodes list is not registered yet
			m_partitionID_to_prev_nodes_list.insert(make_pair(partitionID, *partitions_it));
		}
		else {
			if (*partitions_it == finder->second) {
				MRPT_LOG_DEBUG_STREAM << "Partition " << partitionID
					<< " remained unchanged. ";
				continue; // same list as before.. no need to check this...
			}
			else { // list was changed  - update the previous nodes list
				MRPT_LOG_DEBUG_STREAM << "Partition " << partitionID << " CHANGED. ";
				finder->second = *partitions_it;
			}
		}


		// investigate each partition
		int curr_node_index = 1;
		size_t prev_nodeID = *(partitions_it->begin());
		for (vector_uint::const_iterator it = partitions_it->begin()+1;
				it != partitions_it->end(); ++it, ++curr_node_index) {
			size_t curr_nodeID = *it;

			// are there consecutive nodes with large difference inside this
			// partition? Are these nodes enough to consider LC?
			if ((curr_nodeID - prev_nodeID) > m_lc_params.LC_min_nodeid_diff) {
				// there is at least one divergent node..

				int num_after_nodes = partitions_it->size() - curr_node_index;
				int num_before_nodes = partitions_it->size() - num_after_nodes;
				if (num_after_nodes >= m_lc_params.LC_min_remote_nodes &&
						num_before_nodes >= m_lc_params.LC_min_remote_nodes ) { // at least X LC nodes
					MRPT_LOG_WARN_STREAM(
							"Found potential loop closures:" << endl <<
							"\tPartitionID: " << partitionID << endl <<
							"\tPartition: " << getVectorAsString(*partitions_it).c_str() << endl
							<< "\t" << prev_nodeID << " ==> " << curr_nodeID << endl <<
							"\tNumber of LC nodes: " << num_after_nodes);
					partitions_for_LC->push_back(*partitions_it);
					break; // no need to check the rest of the nodes in this partition
				}
			}

			// update the previous node
			prev_nodeID = curr_nodeID;
		}
		this->logFmt(LVL_DEBUG, "Successfully checked partition: %d", partitionID);
	}

	this->m_time_logger.leave("LoopClosureEvaluation");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::evaluatePartitionsForLC(
		const partitions_t& partitions) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::utils;
	using namespace mrpt::graphslam::detail;
	using namespace std;
	this->m_time_logger.enter("LoopClosureEvaluation");

	if (partitions.size() == 0) return;

	std::string header_sep(80, '-');
	this->logFmt(LVL_DEBUG, "Evaluating partitions for loop closures...\n%s\n",
			header_sep.c_str());

	for (partitions_t::const_iterator p_it = partitions.begin();
			p_it != partitions.end();
			++p_it ) {

		vector_uint partition(*p_it);

		// split the partition to groups
		vector_uint groupA, groupB;
		this->splitPartitionToGroups(partition,
				&groupA, &groupB,
				/*max_nodes_in_group=*/5);
		// generate the hypothesis pool
		// use a hypothesis ID with which the consistency matrix will then be
		// formed
		int hypothesis_counter = 0;
		int invalid_hypotheses = 0;
		std::map<
			std::pair<TNodeID, TNodeID>,
			TGraphSlamHypothesis<GRAPH_t>*> nodeIDs_to_hypots;
		{
			for (vector_uint::const_iterator
					b_it = groupB.begin();
					b_it != groupB.end();
					++b_it) {
				for (vector_uint::const_iterator
						a_it = groupA.begin();
						a_it != groupA.end();
						++a_it) {
					mrpt::slam::CICP::TReturnInfo icp_info;
					// by default hypotheses will direct bi => ai; If the hypothesis is
					// traversed the opposite way take the opposite of the constraint
					TGraphSlamHypothesis<GRAPH_t>* hypot = new TGraphSlamHypothesis<GRAPH_t>;
					hypot->from = *b_it;
					hypot->to = *a_it;
					hypot->id = hypothesis_counter++;
					bool found_edge = this->getICPEdge(
							*b_it,
							*a_it,
							&(hypot->edge),
							&icp_info);
					hypot->goodness = icp_info.goodness; // goodness related to the edge
					//cout << "Goodness: " << icp_info.goodness << endl;
					//cout << hypot->getAsString() << endl;
					// Mark as invalid, do not use it from now on...
					if (!found_edge || hypot->goodness == 0) {
						hypot->is_valid = false;
						invalid_hypotheses++;
					}
					nodeIDs_to_hypots[make_pair(*b_it, *a_it)] = hypot;
					this->logFmt(LVL_DEBUG, "%s", hypot->getAsString().c_str());
				}
			}
			MRPT_LOG_DEBUG_STREAM(
				"Generated pool of hypotheses...\tnodeIDs_to_hypots.size() = "
				<< nodeIDs_to_hypots.size()
				<< "\tinvalid hypotheses: " << invalid_hypotheses);
		}
		//mrpt::system::pause();

		// compute the pair-wise consistency for groups of hypotheses between
		// groups A, B
		std::map<
			std::pair<TGraphSlamHypothesis<GRAPH_t>*,
								TGraphSlamHypothesis<GRAPH_t>*>,
			double> hypots_to_consistencies;

		for (vector_uint::const_iterator
				b_out_it = groupB.begin(); b_out_it != groupB.end(); ++b_out_it) {
			TNodeID b1 = *b_out_it;

			for (vector_uint::const_iterator
					b_in_it = b_out_it+1; b_in_it != groupB.end(); ++b_in_it) {
				TNodeID b2 = *b_in_it;

				for (vector_uint::const_iterator
						a_out_it = groupA.begin(); a_out_it != groupA.end(); ++a_out_it) {
					TNodeID a1 = *a_out_it;
					TGraphSlamHypothesis<GRAPH_t>* h_b2a1 =
						nodeIDs_to_hypots.at(make_pair(b2, a1));

					for (vector_uint::const_iterator
							a_in_it = a_out_it+1; a_in_it != groupA.end(); ++a_in_it) {
						TNodeID a2 = *a_in_it;
						TGraphSlamHypothesis<GRAPH_t>* h_b1a2 =
							nodeIDs_to_hypots.at(make_pair(b1, a2));

						double consistency;
						bool hypots_are_valid = (h_b2a1->is_valid && h_b1a2->is_valid &&
								h_b2a1->goodness > 0.25 && h_b1a2->goodness > 0.25);

						if (hypots_are_valid) { //  skip the ones that don't look good
							// keep the consistency element based on the hypotheses that it
							// was generated by - direction of the hypothesis is by default
							// bi=>ai. If the opposite is needed, it is handled by the
							// calling function
							consistency = this->generatePWConsistencyElement(
									a1,a2,b1,b2,
									nodeIDs_to_hypots);
						}
						else {
							consistency = 0;
						}
						MRPT_LOG_DEBUG_STREAM <<
							"Adding hypotheses consistency for nodeIDs: " <<
							b1 << ", " << b2 << ", " << a1 << ", " << a2 <<
							" => " << consistency << endl;
						hypots_to_consistencies[make_pair(h_b2a1, h_b1a2)] = consistency;
					}
				}
			}
		}
		this->logFmt(LVL_DEBUG,
				"Generated map of hypothesis pairs to corresponding consistency elements");
		//mrpt::system::pause();

		// generate the pair-wise consistency matrix of the relevant edges and find
		// the submatrix of the most consistent hypotheses inside it.
		CMatrixDouble consist_matrix(hypothesis_counter, hypothesis_counter);
		for (typename std::map<
					std::pair<TGraphSlamHypothesis<GRAPH_t>*,
										TGraphSlamHypothesis<GRAPH_t>*>,
					double>::const_iterator
				it = hypots_to_consistencies.begin();
				it != hypots_to_consistencies.end();
				++it)  {
			int id1 = it->first.first->id;
			int id2 = it->first.second->id;
			double consistency_elem = it->second;
			consist_matrix(id1, id2) = consist_matrix(id2, id1) = consistency_elem;
			MRPT_LOG_DEBUG_STREAM << "id1 = " << id1 << "\t|" << "id2 = " << id2 <<
				"\t|" << "consistency_elem = " << consistency_elem;
		}
		MRPT_LOG_DEBUG_STREAM("Row count of consist_matrix: " <<
			consist_matrix.getRowCount());

		// evaluate the pair-wise consistency matrix
		// compute dominant eigenvector
		dynamic_vector<double> u;
		bool valid_lambda_ratio =
			this->computeDominantEigenVector(
					consist_matrix, &u,
					/*use_power_method=*/ false);
		if (!valid_lambda_ratio) continue;

		//cout << "Dominant eigenvector: " << u.transpose() << endl;

		// discretize the indicator vector - maximize the dot product of
		// w_unit .* u
		dynamic_vector<double> w(u.size(), 0); // discretized  indicator vector
		double dot_product = 0;
		for (int i = 0; i != w.size(); ++i) {
			stringstream ss;

			// make the necessary change and see if the dot product increases
			w(i) = 1;
			double potential_dot_product = ((w.transpose() * u) / w.squaredNorm()).value();
			ss << mrpt::format("current: %f | potential_dot_product: %f",
					dot_product, potential_dot_product);
			if (potential_dot_product > dot_product) {
				ss << " ==>  ACCEPT";
				dot_product =	potential_dot_product;
			}
			else {
				ss << " ==>  REJECT";
				w(i) = 0; // revert the change
			}
			ss << endl;
			MRPT_LOG_DEBUG_STREAM << ss.str();
		}
		cout << "Outcome of discretization: " << w.transpose() << endl;
		//mrpt::system::pause();

		// register the indicated hypotheses
		if (!w.isZero()) {
			MRPT_LOG_DEBUG_STREAM << "Registering Hypotheses...";

			for (int wi = 0; wi != w.size(); ++wi) {
				if (w(wi) == 1)  {
					// search through the potential hypotheses, find the one with the
					// correct ID and register it.
					typename std::map<
						std::pair<TNodeID, TNodeID>,
						TGraphSlamHypothesis<GRAPH_t>*>::const_iterator h_it;
					for (h_it = nodeIDs_to_hypots.begin();
							h_it != nodeIDs_to_hypots.end();
							++h_it) {
						if (h_it->second->id == wi) {
							this->registerHypothesis(*(h_it->second));
							break;
						}
					}
					ASSERTMSG_(h_it != nodeIDs_to_hypots.end(),
							format("Hypothesis %d was not found", wi));
				}
			}
		}
		//mrpt::system::pause();


		// delete the hypotheses - generated in the heap...
		MRPT_LOG_DEBUG_STREAM << "Deleting the generated hypotheses pool..." ;
		for (typename std::map<std::pair<TNodeID, TNodeID>,
					TGraphSlamHypothesis<GRAPH_t>*>::const_iterator
					it = nodeIDs_to_hypots.begin();
					it != nodeIDs_to_hypots.end();
					++it) {
			delete it->second;
		}

	}

	MRPT_LOG_DEBUG_STREAM << "\n" << header_sep;
	this->m_time_logger.leave("LoopClosureEvaluation");

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::splitPartitionToGroups(
		vector_uint& partition,
		vector_uint* groupA,
		vector_uint* groupB,
		int max_nodes_in_group/*=5*/) {

	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	// assertions
	ASSERTMSG_(groupA, "Pointer to groupA is not valid");
	ASSERTMSG_(groupB, "Pointer to groupB is not valid");
	ASSERTMSG_(max_nodes_in_group == -1 || max_nodes_in_group == 5,
			format("Value %d not permitted for max_nodes_in_group",
				max_nodes_in_group));

	// find a large difference between successive nodeIDs - that's where the cut
	// is going to be
	TNodeID prev_nodeID = 0;
	size_t index_to_split = 1;
	for (vector_uint::const_iterator it = partition.begin()+1;
			it != partition.end(); ++it, ++index_to_split) {
		TNodeID curr_nodeID = *it;

		if ((curr_nodeID - prev_nodeID) > m_lc_params.LC_min_nodeid_diff) {
			break;
		}
		// update the last nodeID
		prev_nodeID = curr_nodeID;
	}
	ASSERT_(partition.size() > index_to_split);

	// Fill the groups
	*groupA = vector_uint(partition.begin(), partition.begin() + index_to_split);
	*groupB = vector_uint(partition.begin() + index_to_split, partition.end());
	if (max_nodes_in_group != -1) { // limit the number of nodes in each group
		// groupA
		if (groupA->size() > max_nodes_in_group) {
			*groupA = vector_uint(groupA->begin(),
					groupA->begin() + max_nodes_in_group);
		}
		// groupB
		if (groupB->size() > max_nodes_in_group) {
			*groupB = vector_uint(groupB->end() - max_nodes_in_group,
					groupB->end());
		}
	}

	MRPT_LOG_DEBUG_STREAM << "groupA:\n\t" << getVectorAsString(*groupA) <<
		" - size: " << groupA->size() << endl;
	MRPT_LOG_DEBUG_STREAM << "groupB:\n\t" << getVectorAsString(*groupB) <<
		" - size: " << groupB->size() << endl;
	//mrpt::system::pause();
}



template<class GRAPH_t>
bool CLoopCloserERD<GRAPH_t>::computeDominantEigenVector(
		const mrpt::math::CMatrixDouble& consist_matrix,
		mrpt::math::dynamic_vector<double>* eigvec,
		bool use_power_method/*=true*/) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace std;
	ASSERT_(eigvec);

	this->m_time_logger.enter("DominantEigenvectorComputation");

	double lambda1, lambda2; // eigenvalues to use
	bool valid_lambda_ratio = false;

	if (use_power_method) {
		THROW_EXCEPTION(
				"\nPower method for computing the first two eigenvectors/eigenvalues hasn't been implemented yet\n");
	}
	else { // call to eigenVectors method
		CMatrixDouble eigvecs, eigvals;
		consist_matrix.eigenVectors(eigvecs, eigvals);

		// assert that the eivenvectors, eigenvalues, consistency matrix are of the
		// same size
		if (!(eigvecs.size() == eigvals.size()) &&
				(consist_matrix.size() == eigvals.size())) {
			MRPT_LOG_ERROR_STREAM("Sizes of eigvecs (" << eigvecs.size() << ")"
				<< "eigvals (" << eigvals.size() << ")" <<
				"consist_matrix (" << consist_matrix.size() << ")" << "don't match.");
				ASSERT_(false);
		}

		eigvecs.extractCol(eigvecs.getColCount()-1, *eigvec);
		lambda1 = eigvals(eigvals.getRowCount()-1, eigvals.getColCount()-1);
		lambda2 = eigvals(eigvals.getRowCount()-2, eigvals.getColCount()-2);
	}

	// I don't care about the sign of the eigenvecttor element
	for (int i = 0; i != eigvec->size(); ++i) {
		(*eigvec)(i) = abs((*eigvec)(i));
	}

	// check the ratio of the two eigenvalues - reject hypotheses set if ratio
	// smaller than threshold
	double curr_lambda_ratio = lambda1 / lambda2;
	stringstream ss;
	ss << "lambda1 = " << lambda1 << " | lambda2 = " << lambda2 << endl;

	valid_lambda_ratio = (curr_lambda_ratio > m_lc_params.LC_eigenvalues_ratio_thresh || lambda2 != 0);
	if (!valid_lambda_ratio) {
		ss << "Current Lambda ratio: " << curr_lambda_ratio;
		ss << "| Threshold ratio: " << m_lc_params.LC_eigenvalues_ratio_thresh;
		ss << "| Lambda threshold not passed or lambda2 = 0!" << endl;
	}
	MRPT_LOG_DEBUG_STREAM << ss.str();

	this->m_time_logger.leave("DominantEigenvectorComputation");
	return valid_lambda_ratio;

	MRPT_END;
}


template<class GRAPH_t>
double CLoopCloserERD<GRAPH_t>::generatePWConsistencyElement(
		const mrpt::utils::TNodeID& a1,
		const mrpt::utils::TNodeID& a2,
		const mrpt::utils::TNodeID& b1,
		const mrpt::utils::TNodeID& b2,
		const typename std::map<
			std::pair<
				mrpt::utils::TNodeID,
				mrpt::utils::TNodeID>,
			mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_t>*>& nodeIDs_to_hypots) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::utils;
	using namespace mrpt::graphslam;
	using namespace mrpt::graphslam::detail;

	// get the dijkstra links
	// a1=>a2
	execDijkstraProjection(/*starting_node=*/ a1, /*ending_node=*/ a2);
	TUncertaintyPath<GRAPH_t>* path_a1a2 = this->queryOptimalPath(a2);
	ASSERTMSG_(path_a1a2->getSource() == a1,
			format("\nnodeID %lu is not the source of the optimal path\n%s\n\n",
				a1, path_a1a2->getAsString().c_str()));
	ASSERTMSG_(path_a1a2->getDestination() == a2,
			format("\nnodeID %lu is not the destination of the optimal path\n%s\n\n",
				a2, path_a1a2->getAsString().c_str()));
	// b1=>b2
	execDijkstraProjection(/*starting_node=*/ b1, /*ending_node=*/ b2);
	TUncertaintyPath<GRAPH_t>* path_b1b2 = this->queryOptimalPath(b2);
	ASSERTMSG_(path_b1b2->getSource() == b1,
			format("\nnodeID %lu is not the source of the optimal path\n%s\n\n",
				b1, path_b1b2->getAsString().c_str()));
	ASSERTMSG_(path_b1b2->getDestination() == b2,
			format("\nnodeID %lu is not the destination of the optimal path\n%s\n\n",
				b2, path_b1b2->getAsString().c_str()));

	// get the edges of the hypotheses
	// by default hypotheses are stored bi => ai
	std::pair<TNodeID, TNodeID> curr_pair;
	constraint_t edge_a2b1, edge_b2a1;
	typename std::map<
		std::pair<TNodeID, TNodeID>,
		mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_t>*>::const_iterator search;
	{
		// Backwards edge: a2=>b1
		curr_pair = make_pair(b1, a2);
		search = nodeIDs_to_hypots.find(curr_pair);
		ASSERTMSG_(search != nodeIDs_to_hypots.end(),
				format("Hypothesis (b1= ) %lu => (a2= ) %lu was not found", b1, a2) );
		(search->second->edge).inverse(edge_a2b1);

		// forward edge b2=>a1
		curr_pair = make_pair(b2, a1);
		search = nodeIDs_to_hypots.find(curr_pair);
		ASSERTMSG_(search != nodeIDs_to_hypots.end(),
				format("Hypothesis (b2= ) %lu => (a1= ) %lu was not found", b1, a2) );
		edge_b2a1 = search->second->edge;
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
	
	// get the vector of the corresponding transformation - [x, y, phi] form
	dynamic_vector<double> T;
	res.getMeanVal().getAsVector(T);

	// information matrix
	CMatrixDouble33 cov_mat;
	res.getCovariance(cov_mat);

	// there has to be an error with the initial Olson formula - p.15.
	// There must be a minus in the exponent and the covariance matrix instead of
	// the information matrix.
	double exponent = (-T.transpose() * cov_mat * T).value();
	double consistency_elem = exp(exponent);

	//cout << "T = " << endl << T << endl;
	//cout << "exponent = " << exponent << endl;
	//cout << "consistency_elem = " << consistency_elem << endl;
	//mrpt::system::pause();

	return consistency_elem;
	MRPT_END;
}

template<class GRAPH_t>
mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_t>*
CLoopCloserERD<GRAPH_t>::findHypotWithEnds(
		std::vector< mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_t>* >& vec_hypots,
				mrpt::utils::TNodeID from, mrpt::utils::TNodeID to) {

	using namespace mrpt::graphslam::detail;


	for (typename std::vector<TGraphSlamHypothesis<GRAPH_t>*>::const_iterator v_cit =
			vec_hypots.begin();
			v_cit != vec_hypots.end();
			v_cit++) {

		if (v_cit->hasEnds(from, to)) {
			return *v_cit;
		}
	}

	// not found.
	return NULL;

}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::execDijkstraProjection(
		mrpt::utils::TNodeID starting_node/*=0*/,
		mrpt::utils::TNodeID ending_node/*=INVALID_NODEID*/) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	// for the full algorithm see
	// - Recognizing places using spectrally clustered local matches - E.Olson,
	// p.6

	this->m_time_logger.enter("Dijkstra Projection");

	// ending_node is either INVALID_NODEID or one of the already registered
	// nodeIDs
	ASSERT_(ending_node == INVALID_NODEID ||
			(ending_node >= 0 && ending_node < this->m_graph->nodeCount()) );
	ASSERTMSG_(starting_node != ending_node, "Starting and Ending nodes coincede");

	if (this->m_graph->nodeCount() < 5) return;

	// debugging message
	stringstream ss_debug("");
	ss_debug << "Executing Dijkstra Projection: " << starting_node << " => ";
	if (ending_node == INVALID_NODEID) {
		ss_debug << "..." << endl;
	}
	else {
		ss_debug << ending_node <<endl;
	}
	MRPT_LOG_DEBUG_STREAM(ss_debug.str());

	// keep track of the nodes that I have visited
	std::vector<bool> visited_nodes(this->m_graph->nodeCount(), false);
	m_node_optimal_paths.clear();

	// get the neighbors of each node
	std::map<TNodeID, std::set<TNodeID> > neighbors_of;
	this->m_graph->getAdjacencyMatrix(neighbors_of);

	// initialize a pool of TUncertaintyPaths - draw the minimum-uncertainty path during
	// execution
	std::set<TUncertaintyPath<GRAPH_t>*> pool_of_paths;
	// get the edge to each one of the neighboring nodes of the starting node
	std::set<TNodeID> starting_node_neighbors(neighbors_of.at(starting_node));
	for (std::set<TNodeID>::const_iterator n_it =
			starting_node_neighbors.begin();
			n_it != starting_node_neighbors.end(); ++n_it) {

		TUncertaintyPath<GRAPH_t>* path_between_neighbors =
			new TUncertaintyPath<GRAPH_t>();
		this->getMinUncertaintyPath(starting_node, *n_it, path_between_neighbors);

		pool_of_paths.insert(path_between_neighbors);
	}
	// just visited the first node
	visited_nodes.at(starting_node) = true;

	//// TODO Remove these - >>>>>>>>>>>>>>>>>>>>
	//// printing the pool for verification
	//cout << "Pool of Paths: " << endl;
	//for (typename std::set<TUncertaintyPath<GRAPH_t>*>::const_iterator it = pool_of_paths.begin();
			//it != pool_of_paths.end(); ++it) {
		//printVector((*it)->nodes_traversed);
	//}
	//cout << "------ Done with the starting node ... ------" << endl;
	//int iters = 0;
	//// TODO Remove these - <<<<<<<<<<<<<<<<<<<<< vvvUNCOMMENT BELOW AS WELLvvv

	while (true) {

		// if there is at least one false, exit loop
		for (std::vector<bool>::const_iterator it = visited_nodes.begin();
				it != visited_nodes.end(); ++it) {
			if (! *it) {
				break;
			}
		}

		// if an ending nodeID has been specified, end the method when the path to
		// it is found.
		if (ending_node != INVALID_NODEID) {
			if (visited_nodes.at(ending_node)) {
				MRPT_LOG_DEBUG_STREAM <<
					"----------- Done with Dijkstra Projection... ----------";
				this->m_time_logger.leave("Dijkstra Projection");
				return;
			}
		}

		TUncertaintyPath<GRAPH_t>* optimal_path =
			this->popMinUncertaintyPath(&pool_of_paths);
		TNodeID dest = optimal_path->getDestination();

		//// TODO Remove these - >>>>>>>>>>>>>>>>>>>> ^^^UNCOMMENT ABOVE AS WELL^^^
		//cout << iters << " " << std::string(40, '>') << endl;
		//cout << "current path Destination: " << dest << endl;
		//// printing the pool for verification
		//cout << "Pool of Paths: " << endl;
		//for (typename std::set<TUncertaintyPath<GRAPH_t>*>::const_iterator
				//it = pool_of_paths.begin();
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

	MRPT_LOG_DEBUG_STREAM <<
		"----------- Done with Dijkstra Projection... ----------";
	this->m_time_logger.leave("Dijkstra Projection");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::addToPaths(
		std::set<typename mrpt::graphslam::TUncertaintyPath<GRAPH_t>*>* pool_of_paths,
		const mrpt::graphslam::TUncertaintyPath<GRAPH_t>& current_path,
		const std::set<mrpt::utils::TNodeID>& neighbors) const {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;
	using namespace mrpt::graphslam;

	TNodeID node_to_append_from = current_path.getDestination();

	// compose transforms for every neighbor of node_to_append_from *except*
	// for the link connecting node_to_append_from and the second to last node in
	// the current_path
	TNodeID second_to_last_node = current_path.nodes_traversed.rbegin()[1];
	for (std::set<TNodeID>::const_iterator neigh_it = neighbors.begin();
			neigh_it != neighbors.end(); ++neigh_it) {
		if (*neigh_it == second_to_last_node) continue;

		// get the path between node_to_append_from, *node_it
		TUncertaintyPath<GRAPH_t> path_between_nodes;
		this->getMinUncertaintyPath(node_to_append_from, *neigh_it,
				&path_between_nodes);

		// format the path to append
		TUncertaintyPath<GRAPH_t>* path_to_append =
			new TUncertaintyPath<GRAPH_t>();
		*path_to_append = current_path;
		*path_to_append += path_between_nodes;

		pool_of_paths->insert(path_to_append);
	}

	MRPT_END;
}

template<class GRAPH_t>
typename mrpt::graphslam::TUncertaintyPath<GRAPH_t>*
CLoopCloserERD<GRAPH_t>::queryOptimalPath(const mrpt::utils::TNodeID node) const {
	MRPT_START;

	TUncertaintyPath<GRAPH_t>* path = NULL;
	typename std::map<mrpt::utils::TNodeID,
					 TUncertaintyPath<GRAPH_t>*>::const_iterator search;
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
		TUncertaintyPath<GRAPH_t>* path_between_nodes) const {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace std;

	ASSERTMSG_(
			this->m_graph->edgeExists(from, to) ||
			this->m_graph->edgeExists(to, from),
			mrpt::format("\nEdge between the provided nodeIDs"
				"(%lu <-> %lu) does not exist\n", from, to) );
	ASSERT_(path_between_nodes);

	//cout << "getMinUncertaintyPath: " << from << " => " << to << endl;

	// don't add to the path_between_nodes, just fill it in afterwards
	path_between_nodes->clear(); 
	
	// iterate over all the edges, ignore the ones that are all 0s - find the
	// one that is with the lowest uncertainty
	double curr_determinant = 0;
	// forward edges from -> to
	std::pair<edges_citerator, edges_citerator> fwd_edges_pair =
		this->m_graph->getEdges(from, to);

	//cout << "Forward edges: " << endl;
	//for (edges_citerator e_it = fwd_edges_pair.first; e_it != fwd_edges_pair.second;
			//++e_it) {
		//cout << e_it->second << endl;
	//}

	for (edges_citerator edges_it = fwd_edges_pair.first;
			edges_it != fwd_edges_pair.second; ++edges_it) {
		// operate on a temporary object instead of the real edge - otherwise
		// function is non-const
		constraint_t curr_edge;
		curr_edge.copyFrom(edges_it->second);

		// is it all 0s?
		CMatrixDouble33 inf_mat;
		curr_edge.getInformationMatrix(inf_mat);

		if (inf_mat == CMatrixDouble33() || isNaN(inf_mat(0,0))) {
			inf_mat.unit();
			curr_edge.cov_inv = inf_mat;
		}

		TUncertaintyPath<GRAPH_t> curr_path(from); // set the starting node
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
		this->m_graph->getEdges(to, from);

	//cout << "Backwards edges: " << endl;
	//for (edges_citerator e_it = bwd_edges_pair.first; e_it != bwd_edges_pair.second;
			//++e_it) {
		//cout << e_it->second << endl;
	//}

	for (edges_citerator edges_it = bwd_edges_pair.first;
			edges_it != bwd_edges_pair.second; ++edges_it) {
		// operate on a temporary object instead of the real edge - otherwise
		// function is non-const
		constraint_t curr_edge;
		(edges_it->second).inverse(curr_edge);

		// is it all 0s?
		CMatrixDouble33 inf_mat;
		curr_edge.getInformationMatrix(inf_mat);

		if (inf_mat == CMatrixDouble33() || isNaN(inf_mat(0,0))) {
			inf_mat.unit();
			curr_edge.cov_inv = inf_mat;
		}

		TUncertaintyPath<GRAPH_t> curr_path(from); // set the starting node
		curr_path.addToPath(to, curr_edge);

		// update the resulting path_between_nodes if its determinant is smaller
		// than the determinant of the current path_between_nodes
		if (curr_determinant < curr_path.getDeterminant()) {
			curr_determinant = curr_path.getDeterminant();
			*path_between_nodes = curr_path;
		}
	}

	MRPT_END;
}

template<class GRAPH_t>
TUncertaintyPath<GRAPH_t>* CLoopCloserERD<GRAPH_t>::
popMinUncertaintyPath(
		typename std::set<TUncertaintyPath<GRAPH_t>*>* pool_of_paths) const {
	MRPT_START;
	using namespace std;

	//cout << "Determinants: ";
	TUncertaintyPath<GRAPH_t>* optimal_path = NULL;
	double curr_determinant = 0;
	for (typename std::set<TUncertaintyPath<GRAPH_t>*>::const_iterator
			it = pool_of_paths->begin();
			it != pool_of_paths->end();
			++it) {
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
bool CLoopCloserERD<GRAPH_t>::mahalanobisDistanceOdometryToICPEdge(
		const mrpt::utils::TNodeID& from, const mrpt::utils::TNodeID& to, const
		constraint_t& rel_edge) {
	MRPT_START;

	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::utils;

	// mean difference
	pose_t initial_estim = this->m_graph->nodes.at(to) - this->m_graph->nodes.at(from);
	dynamic_vector<double> mean_diff;
	(rel_edge.getMeanVal()-initial_estim).getAsVector(mean_diff);
	
	// covariance matrix
	CMatrixDouble33 cov_mat; rel_edge.getCovariance(cov_mat);

	// mahalanobis distance computation
	double mahal_distance = mrpt::math::mahalanobisDistance2(mean_diff, cov_mat);
	bool mahal_distance_null = isNaN(mahal_distance);
	if (!mahal_distance_null) {
		m_laser_params.mahal_distance_ICP_odom_win.addNewMeasurement(mahal_distance);
	}

	//double threshold = m_laser_params.mahal_distance_ICP_odom_win.getMean() +
		//2*m_laser_params.mahal_distance_ICP_odom_win.getStdDev();
	double threshold = m_laser_params.mahal_distance_ICP_odom_win.getMedian()*4;
	bool accept_edge = (threshold >= mahal_distance && !mahal_distance_null) ? true : false;

	//cout << "Suggested Edge: " << rel_edge.getMeanVal() << "|\tInitial Estim.: " << initial_estim
		//<< "|\tMahalanobis Dist: " << mahal_distance << "|\tThresh.: " << threshold
		//<< " => " << (accept_edge? "ACCEPT": "REJECT") << endl;

	return accept_edge;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::registerHypothesis(
		const mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_t>& h) {
	MRPT_LOG_DEBUG_STREAM << "Registering hypothesis: " <<
		h.getAsString(/*oneline=*/ true);
	this->registerNewEdge(h.from, h.to, h.edge);
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt::math;
	using namespace std;
	parent::registerNewEdge(from, to, rel_edge);

	//  keep track of the registered edges...
	m_edge_types_to_nums["ICP2D"]++;

	//  keep track of the registered edges...
	if (absDiff(to, from) > m_lc_params.LC_min_nodeid_diff)  {
		m_edge_types_to_nums["LC"]++;
		m_just_inserted_loop_closure = true;
		this->logFmt(LVL_INFO, "\tLoop Closure edge!");
	}
	else {
		m_just_inserted_loop_closure = false;
	}

	//  actuall registration
	this->m_graph->insertEdge(from,  to, rel_edge);

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	// call parent class method
	parent::setWindowManagerPtr(win_manager);

	using namespace mrpt::utils;

	// may still be null..
	if (this->m_win_manager) {

		if (this->m_win_observer) {
			this->m_win_observer->registerKeystroke(m_laser_params.keystroke_laser_scans,
					"Toggle LaserScans Visualization");
			this->m_win_observer->registerKeystroke(m_lc_params.keystroke_map_partitions,
					"Toggle Map Partitions Visualization");

		}

		this->logFmt(LVL_DEBUG,
				"Fetched the window manager, window observer  successfully.");
	}

}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;
	parent::notifyOfWindowEvents(events_occurred);

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

	// textmessage - display the number of partitions
	if (!m_lc_params.LC_check_curr_partition_only) {
		this->m_win_manager->assignTextMessageParameters(
				/* offset_y*	= */ &m_lc_params.offset_y_map_partitions,
				/* text_index* = */ &m_lc_params.text_index_map_partitions);
	}

	// just add an empty CSetOfObjects in the scene - going to populate it later
	CSetOfObjectsPtr map_partitions_obj = CSetOfObjects::Create();
	map_partitions_obj->setName("map_partitions");

	COpenGLScenePtr& scene = this->m_win->get3DSceneAndLock();
	scene->insert(map_partitions_obj);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

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
	if (!m_lc_params.LC_check_curr_partition_only) {
		std::stringstream title;
		title << "# Partitions: " << m_curr_partitions.size();
		this->m_win_manager->addTextMessage(5,-m_lc_params.offset_y_map_partitions,
				title.str(),
				mrpt::utils::TColorf(m_lc_params.balloon_std_color),
				/* unique_index = */ m_lc_params.text_index_map_partitions);
	}

	// update the partitioning visualization
	// ////////////////////////////////////////////////////////////
	COpenGLScenePtr& scene = this->m_win->get3DSceneAndLock();

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

		MRPT_LOG_DEBUG_STREAM << "Working on Partition #" << partitionID;
		vector_uint nodes_list = *p_it;

		// finding the partition in which the last node is in
		if (std::find(nodes_list.begin(), nodes_list.end(), this->m_graph->nodeCount()-1)
				!= nodes_list.end()) {
			partition_contains_last_node = true;
		}
		else {
			partition_contains_last_node = false;
		}

		// fetch the current partition object if it exists - create otherwise
		std::string partition_obj_name = mrpt::format("partition_%d", partitionID);
		std::string balloon_obj_name = mrpt::format("#%d", partitionID);

		CRenderizablePtr obj = map_partitions_obj->getByName(partition_obj_name);
		CSetOfObjectsPtr curr_partition_obj;
		if (obj) {
			MRPT_LOG_DEBUG_STREAM << 
					"\tFetching CSetOfObjects partition object for partition #" <<
					partitionID;
			curr_partition_obj = static_cast<CSetOfObjectsPtr>(obj);
			if (m_lc_params.LC_check_curr_partition_only) { // make all but the last partition invisible
				curr_partition_obj->setVisibility(partition_contains_last_node); 
			}
		}
		else {
			MRPT_LOG_DEBUG_STREAM <<
				"\tCreating a new CSetOfObjects partition object for partition #" <<
				partitionID;
			curr_partition_obj = CSetOfObjects::Create();
			curr_partition_obj->setName(partition_obj_name);
			if (m_lc_params.LC_check_curr_partition_only) {
 				// make all but the last partition invisible
				curr_partition_obj->setVisibility(partition_contains_last_node); 
			}

			MRPT_LOG_DEBUG_STREAM << "\t\tCreating a new CSphere balloon object";
			CSpherePtr balloon_obj = CSphere::Create();
			balloon_obj->setName(balloon_obj_name);
			balloon_obj->setRadius(m_lc_params.balloon_radius);
			balloon_obj->setColor_u8(m_lc_params.balloon_std_color);
			balloon_obj->enableShowName();

			curr_partition_obj->insert(balloon_obj);

			// set of lines connecting the graph nodes to the balloon
			MRPT_LOG_DEBUG_STREAM << "\t\tCreating set of lines that will connect to the Balloon";
			CSetOfLinesPtr connecting_lines_obj = CSetOfLines::Create();
			connecting_lines_obj->setName("connecting_lines");
			connecting_lines_obj->setColor_u8(m_lc_params.connecting_lines_color);
			connecting_lines_obj->setLineWidth(0.1f);

			curr_partition_obj->insert(connecting_lines_obj);

			// add the created CSetOfObjects to the total CSetOfObjects responsible
			// for the map partitioning
			map_partitions_obj->insert(curr_partition_obj);
			MRPT_LOG_DEBUG_STREAM << "\tInserted new CSetOfObjects successfully";
		}
		// up to now the CSetOfObjects exists and the balloon inside it as well..

		std::pair<double, double> centroid_coords;
		this->computeCentroidOfNodesVector(nodes_list, &centroid_coords);

		TPoint3D balloon_location(centroid_coords.first, centroid_coords.second,
				m_lc_params.balloon_elevation);

		MRPT_LOG_DEBUG_STREAM << "\tUpdating the balloon position";
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

		MRPT_LOG_DEBUG_STREAM << "\tUpdating the lines connecting nodes to balloon";
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
				CPose3D curr_pose(this->m_graph->nodes.at(*it));
				TPoint3D curr_node_location(curr_pose);

				TSegment3D connecting_line(curr_node_location, balloon_location);
				connecting_lines_obj->appendLine(connecting_line);
			}

		}
		MRPT_LOG_DEBUG_STREAM << "Done working on partition #" << partitionID;
	}

	// remove outdated partitions
	// these occur when more partitions existed during the previous visualization
	// update, thus the partitions with higher ID than the maximum partitionID
	// would otherwise remain in the visual as zombie partitions
	size_t prev_size = m_last_partitions.size();
	size_t curr_size = m_curr_partitions.size();
	if (curr_size < prev_size) {
		MRPT_LOG_DEBUG_STREAM("Removing outdated partitions in visual");
		for (size_t partitionID = curr_size; partitionID != prev_size; ++partitionID) {
			MRPT_LOG_DEBUG_STREAM("\tRemoving partition " << partitionID);
			std::string partition_obj_name = mrpt::format(
					"partition_%lu",
					static_cast<unsigned long>(partitionID));

			CRenderizablePtr obj = map_partitions_obj->getByName(partition_obj_name);
			map_partitions_obj->removeObject(obj);
		}
	}
	MRPT_LOG_DEBUG_STREAM << "Done working on the partitions visualization.";


	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::toggleMapPartitionsVisualization() {
	MRPT_START;
	ASSERTMSG_(this->m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(this->m_win_manager, "No CWindowManager* was provided");
	using namespace mrpt::utils;
	using namespace mrpt::opengl;

	this->logFmt(LVL_INFO, "Toggling map partitions  visualization...");
	mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

	if (m_lc_params.visualize_map_partitions) {
		mrpt::opengl::CRenderizablePtr obj = scene->getByName("map_partitions");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		this->dumpVisibilityErrorMsg("visualize_map_partitions");
	}

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

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
		pose_t curr_node_pos = this->m_graph->nodes.at(*node_it);
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
		mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

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
		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateLaserScansVisualization() {
	MRPT_START;

	// update laser scan visual
	if (m_laser_params.visualize_laser_scans && !m_last_laser_scan2D.null()) {
		mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

		mrpt::opengl::CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		mrpt::opengl::CPlanarLaserScanPtr laser_scan_viz =
			static_cast<mrpt::opengl::CPlanarLaserScanPtr>(obj);

		laser_scan_viz->setScan(*m_last_laser_scan2D);

		// set the pose of the laser scan
		typename GRAPH_t::global_poses_t::const_iterator search =
			this->m_graph->nodes.find(this->m_graph->nodeCount()-1);
		if (search != this->m_graph->nodes.end()) {
			laser_scan_viz->setPose(this->m_graph->nodes[this->m_graph->nodeCount()-1]);
			// put the laser scan underneath the graph, so that you can still
			// visualize the loop closures with the nodes ahead
			laser_scan_viz->setPose(mrpt::poses::CPose3D(
						laser_scan_viz->getPoseX(), laser_scan_viz->getPoseY(), -0.15,
						mrpt::utils::DEG2RAD(laser_scan_viz->getPoseYaw()),
						mrpt::utils::DEG2RAD(laser_scan_viz->getPosePitch()),
						mrpt::utils::DEG2RAD(laser_scan_viz->getPoseRoll())
						));
		}

		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::toggleLaserScansVisualization() {
	MRPT_START;
	ASSERTMSG_(this->m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(this->m_win_manager, "No CWindowManager* was provided");
	using namespace mrpt::utils;

	this->logFmt(LVL_INFO, "Toggling LaserScans visualization...");

	mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();

	if (m_laser_params.visualize_laser_scans) {
		mrpt::opengl::CRenderizablePtr obj = scene->getByName("laser_scan_viz");
		obj->setVisibility(!obj->isVisible());
	}
	else {
		this->dumpVisibilityErrorMsg("visualize_laser_scans");
	}

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::getEdgesStats(
		std::map<std::string, int>* edge_types_to_num) const {
	MRPT_START;
	*edge_types_to_num = m_edge_types_to_nums;
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initializeVisuals() {
	MRPT_START;
	parent::initializeVisuals();
	MRPT_LOG_DEBUG_STREAM << "Initializing visuals";
	this->m_time_logger.enter("Visuals");

	ASSERTMSG_(m_laser_params.has_read_config,
			"Configuration parameters aren't loaded yet");
	if (m_laser_params.visualize_laser_scans) {
		this->initLaserScansVisualization();
	}
	if (m_lc_params.visualize_map_partitions) {
		this->initMapPartitionsVisualization();
	}

	if (m_visualize_curr_node_covariance) {
		this->initCurrCovarianceVisualization();
	}

	this->m_time_logger.leave("Visuals");
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateVisuals() {
	MRPT_START;
	parent::updateVisuals();
	MRPT_LOG_DEBUG_STREAM << "Updating visuals";
	this->m_time_logger.enter("Visuals");

	if (m_laser_params.visualize_laser_scans) {
	this->updateLaserScansVisualization();
	}
	if (m_lc_params.visualize_map_partitions) {
		this->updateMapPartitionsVisualization();
	}
	if (m_visualize_curr_node_covariance) {
		this->updateCurrCovarianceVisualization();
	}

	this->m_time_logger.leave("Visuals");
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::initCurrCovarianceVisualization() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::opengl;

	// text message for covariance ellipsis
	this->m_win_manager->assignTextMessageParameters(
			/* offset_y*	= */ &m_offset_y_curr_node_covariance,
			/* text_index* = */ &m_text_index_curr_node_covariance);

	std::string title("Position uncertainty");
	this->m_win_manager->addTextMessage(5,-m_offset_y_curr_node_covariance,
			title,
			mrpt::utils::TColorf(m_curr_node_covariance_color),
			/* unique_index = */ m_text_index_curr_node_covariance);


	// covariance ellipsis
	CEllipsoidPtr cov_ellipsis_obj = CEllipsoid::Create();
	cov_ellipsis_obj->setName("cov_ellipsis_obj");
	cov_ellipsis_obj->setColor_u8(m_curr_node_covariance_color);
	cov_ellipsis_obj->setLocation(0, 0, 0);
	//cov_ellipsis_obj->setQuantiles(2.0);

	mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();
	scene->insert(cov_ellipsis_obj);
	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();
	
	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateCurrCovarianceVisualization() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::opengl;
	using namespace mrpt::gui;
	using namespace mrpt::utils;

	// get the optimal path to the current node
	mrpt::utils::TNodeID curr_node = this->m_graph->nodeCount()-1;
	TUncertaintyPath<GRAPH_t>* path =	queryOptimalPath(curr_node);
	if (!path) return;

	ASSERT_(path);
	CMatrixDouble33 mat;
	path->curr_pose_pdf.getCovariance(mat);
	pose_t curr_position = this->m_graph->nodes.at(curr_node);

	stringstream ss_mat; ss_mat << mat;
	this->logFmt(LVL_DEBUG, "In updateCurrCovarianceVisualization\n"
			"Covariance matrix:\n%s\n"
			"determinant : %f", ss_mat.str().c_str(), mat.det() );

	mrpt::opengl::COpenGLScenePtr scene = this->m_win->get3DSceneAndLock();
	CRenderizablePtr obj = scene->getByName("cov_ellipsis_obj");
	CEllipsoidPtr cov_ellipsis_obj = static_cast<CEllipsoidPtr>(obj);

	// set the pose and corresponding covariance matrix of the ellipsis
	cov_ellipsis_obj->setLocation(curr_position.x(), curr_position.y(), 0);
	//pose_t loc = path->curr_pose_pdf.getMeanVal();
	//cov_ellipsis_obj->setLocation(loc.x(), loc.y(), 0);
	cov_ellipsis_obj->setCovMatrix(mat, 2);

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
}

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time /* = 500 milliseconds */) {
	MRPT_START;

	this->logFmt(mrpt::utils::LVL_ERROR,
			"Cannot toggle visibility of specified object.\n "
			"Make sure that the corresponding visualization flag ( %s "
			") is set to true in the .ini file.\n",
			viz_flag.c_str());
	mrpt::system::sleep(sleep_time);

	MRPT_END;
}


template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;
	parent::loadParams(source_fname);

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
	this->setMinLoggingLevel(mrpt::utils::VerbosityLevel(min_verbosity_level));

	MRPT_LOG_DEBUG_STREAM << "Successfully loaded parameters. ";
	MRPT_END;
}
template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::printParams() const {
	MRPT_START;
	parent::printParams();

	std::cout << "------------------[Pair-wise Consistency of ICP Edges - Registration Procedure Summary]------------------" << std::endl;

	m_partitioner.options.dumpToConsole();
	m_laser_params.dumpToConsole();
	m_lc_params.dumpToConsole();
	range_scanner_t::params.dumpToConsole();

	MRPT_LOG_DEBUG_STREAM << "Printed the relevant parameters";
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

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::updateMapPartitions(bool full_update /* = false */) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	this->m_time_logger.enter("updateMapPartitions");

 	// Initialize the nodeIDs => LaserScans map
 	nodes_to_scans2D_t nodes_to_scans;
	if (full_update) {
		this->logFmt(LVL_INFO,
				"updateMapPartitions: Full partitioning of map was issued");

		// clear the existing partitions and recompute the partitioned map for all
		// the nodes
		m_partitioner.clear();
		nodes_to_scans = m_nodes_to_laser_scans2D;
	}
	else {
		// just use the last node-laser scan pair
		nodes_to_scans.insert(
				make_pair(
					this->m_graph->root, m_nodes_to_laser_scans2D.at(this->m_graph->nodeCount()-1)));
	}

	// for each one of the above nodes - add its position and correspoding
	// laserScan to the partitioner object
	for (nodes_to_scans2D_t::const_iterator it = nodes_to_scans.begin();
			it != nodes_to_scans.end(); ++it) {
		if ((it->second).null()) { continue; } // if laserScan invalid go to next...

		// find pose of node, if it exists...
		typename GRAPH_t::global_poses_t::const_iterator search;
		search = this->m_graph->nodes.find(it->first);
		if (search == m_graph->nodes.end()) {
			MRPT_LOG_WARN_STREAM("Couldn't find pose for nodeID " << it->first);
			continue;
		}

		pose_t curr_pose = search->second;
		mrpt::poses::CPosePDFPtr posePDF(new constraint_t(curr_pose));

		// laser scan
		mrpt::obs::CSensoryFramePtr sf = mrpt::obs::CSensoryFrame::Create();
		sf->insert(it->second);

		m_partitioner.addMapFrame(sf, posePDF);
	}

	// update the last partitions list
	size_t n = m_curr_partitions.size();
	m_last_partitions.resize(n);
	for (size_t i = 0; i < n; i++)	{
		m_last_partitions[i] = m_curr_partitions[i];
	}
	//update current partitions list
	m_partitioner.updatePartitions(m_curr_partitions);

	MRPT_LOG_DEBUG_STREAM << "Updated map partitions successfully.";
	this->m_time_logger.leave("updateMapPartitions");
	MRPT_END;
}

// TLaserParams
// //////////////////////////////////

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TLaserParams::TLaserParams():
	laser_scans_color(0, 20, 255),
	keystroke_laser_scans("l"),
	has_read_config(false)
{
	mahal_distance_ICP_odom_win.resizeWindow(200); // use the last X mahalanobis distance values
	goodness_threshold_win.resizeWindow(200); // use the last X ICP values
}

template<class GRAPH_t>
CLoopCloserERD<GRAPH_t>::TLaserParams::~TLaserParams() { }

template<class GRAPH_t>
void CLoopCloserERD<GRAPH_t>::TLaserParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("Use scan-matching constraints               = %s\n",
			use_scan_matching? "TRUE": "FALSE");
	out.printf("Num. of previous nodes to check ICP against =  %lu\n",
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
	using namespace std;

	stringstream ss;
	ss << "Min. node difference for loop closure                 = " <<
		LC_min_nodeid_diff << endl;
	ss << "Remote NodeIDs to consider the potential loop closure = " <<
		LC_min_remote_nodes << endl;
	ss << "Min EigenValues ratio for accepting a hypotheses set  = " <<
		LC_eigenvalues_ratio_thresh << endl;
	ss << "Check only current node's partition for loop closures = " <<
		(LC_check_curr_partition_only? "TRUE": "FALSE") << endl;
	ss << "New registered nodes required for full partitioning   = " <<
		full_partition_per_nodes << endl;
	ss << "Visualize map partitions                              = " <<
		(visualize_map_partitions?  "TRUE": "FALSE") << endl;

	out.printf("%s", ss.str().c_str());

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
	LC_min_remote_nodes = source.read_int(
			section,
			"LC_min_remote_nodes",
			3, false);
	LC_eigenvalues_ratio_thresh = source.read_double(
			section,
			"LC_eigenvalues_ratio_thresh",
			2, false);
	LC_check_curr_partition_only = source.read_bool(
			section,
			"LC_check_curr_partition_only",
			true, false);
	full_partition_per_nodes = source.read_int(
			section,
			"full_partition_per_nodes",
			50, false);
	visualize_map_partitions = source.read_bool(
			"VisualizationParameters",
			"visualize_map_partitions",
			true, false);

	has_read_config = true;
	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CLOOPCLOSERERD_IMPL_H */
