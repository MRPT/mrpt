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
template<class GRAPH_T>
CLoopCloserERD<GRAPH_T>::CLoopCloserERD():
	m_visualize_curr_node_covariance(false),
	m_curr_node_covariance_color(160, 160, 160, /*alpha = */255),
	m_partitions_full_update(false),
	m_is_first_time_node_reg(true),
	m_dijkstra_node_count_thresh(3)
{
	this->initializeLoggers("CLoopCloserERD");
	MRPT_LOG_DEBUG_STREAM("Initialized class object");
}

template<class GRAPH_T>
CLoopCloserERD<GRAPH_T>::~CLoopCloserERD() {
	using namespace mrpt::graphslam;

	// release memory of m_node_optimal_paths map.
	MRPT_LOG_DEBUG_STREAM("Releasing memory of m_node_optimal_paths map...");
	for (typename std::map<
			mrpt::utils::TNodeID,
			path_t*>::iterator
			it = m_node_optimal_paths.begin();
			it != m_node_optimal_paths.end();
			++it) {

		delete it->second;
	}
}

// Methods implementations
// //////////////////////////////////

template<class GRAPH_T>
bool CLoopCloserERD<GRAPH_T>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	this->m_time_logger.enter("updateState");
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::obs::utils;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	// Track the last recorded laser scan
	{
		CObservation2DRangeScanPtr scan =
			getObservation<CObservation2DRangeScan>(observations, observation);
		if (scan.present()) { this->m_last_laser_scan2D = scan; }
	}

	if (!m_first_laser_scan.present()) {
		m_first_laser_scan = this->m_last_laser_scan2D;
	}

	// check possible prior node registration
	size_t num_registered = absDiff(
			this->m_last_total_num_nodes, this->m_graph->nodeCount());
	bool registered_new_node = num_registered > 0;

	if (registered_new_node) {
		MRPT_LOG_DEBUG_STREAM("New node has been registered in the graph!");
		registered_new_node = true;

		// either single node registration, or double node registration for the
		// first time only, unless we override this check.
		if (!this->m_override_registered_nodes_check) {
			if (!((num_registered == 1) ^ (num_registered == 2 && m_is_first_time_node_reg))) {
				MRPT_LOG_ERROR_STREAM(
					"Invalid number of registered nodes since last call to updateStates| "
					"Found \"" << num_registered << "\" new nodes.");
				THROW_EXCEPTION("Invalid number of registered nodes.");
			}
		}

		// first time call:
		// NRD should have registered *2* nodes; one for the root node and one for
		// the first added constraint. Add the first measurement taken to the root
		// and the second as usual
		if (m_is_first_time_node_reg) {
			MRPT_LOG_WARN_STREAM("Assigning first laserScan to the graph root node.");
			this->m_nodes_to_laser_scans2D[this->m_graph->root] = m_first_laser_scan;
			m_is_first_time_node_reg = false;
		}

		// register the new node-laserScan pair
		this->m_nodes_to_laser_scans2D[this->m_graph->nodeCount()-1] =
			this->m_last_laser_scan2D;

		if (m_use_scan_matching) {
			// scan match with previous X nodes
			this->addScanMatchingEdges(this->m_graph->nodeCount()-1);
		}

		// update partitioning scheme with the latest pose/measurement
		// UPDATE: This is not needed. Partitions aren't affected at all and also
		// this slows down execution considerably
		//m_partitions_full_update = (
				//(this->m_graph->nodeCount() %
				 //m_lc_params.full_partition_per_nodes) == 0 ||
				//this->m_just_inserted_lc) ?  true: false;

		this->updateMapPartitions(/*full_update = */ false,
				/* is_first_time_node_reg = */ num_registered == 2);

		// check for loop closures
		partitions_t partitions_for_LC;
		this->checkPartitionsForLC(&partitions_for_LC);
		this->evaluatePartitionsForLC(partitions_for_LC);

		if (m_visualize_curr_node_covariance) {
			this->execDijkstraProjection();
		}

		this->m_last_total_num_nodes = this->m_graph->nodeCount();
	}

	this->m_time_logger.leave("updateState");
	return true;
	MRPT_END;
} // end of updateStates

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::checkPartitionsForLC(
		partitions_t* partitions_for_LC) {
	MRPT_START;
	this->m_time_logger.enter("LoopClosureEvaluation");

	using namespace std;
	using namespace mrpt;
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
				//MRPT_LOG_DEBUG_STREAM("Partition " << partitionID
					//<< " remained unchanged. ");
				continue; // same list as before.. no need to check this...
			}
			else { // list was changed  - update the previous nodes list
				//MRPT_LOG_DEBUG_STREAM("Partition " << partitionID << " CHANGED. ");
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
							"\tPartition: " << getSTLContainerAsString(*partitions_it).c_str() << endl
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

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::evaluatePartitionsForLC(
		const partitions_t& partitions) {
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::graphslam::detail;
	using namespace mrpt::math;
	using namespace std;
	this->m_time_logger.enter("LoopClosureEvaluation");

	if (partitions.size() == 0) return;

	this->logFmt(LVL_DEBUG, "Evaluating partitions for loop closures...\n%s\n",
			this->header_sep.c_str());

	// for each partition to be evaulated...
	for (partitions_t::const_iterator p_it = partitions.begin();
			p_it != partitions.end();
			++p_it ) {

		vector_uint partition(*p_it);

		// split the partition to groups
		vector_uint groupA, groupB;
		this->splitPartitionToGroups(partition,
				&groupA, &groupB,
				/*max_nodes_in_group=*/5);

		// generate hypotheses pool
		hypotsp_t hypots_pool;
		this->generateHypotsPool(groupA, groupB, &hypots_pool);

		// compute the pair-wise consistency matrix
		CMatrixDouble consist_matrix(hypots_pool.size(), hypots_pool.size());
		this->generatePWConsistenciesMatrix(
				groupA, groupB, hypots_pool, &consist_matrix);

		// evaluate resulting matrix - fill valid hypotheses
		hypotsp_t valid_hypots;
		this->evalPWConsistenciesMatrix(
				consist_matrix,
				hypots_pool,
				&valid_hypots);

		// registering the indicated/valid hypotheses
		if (valid_hypots.size()) {
			MRPT_LOG_WARN_STREAM("Registering Hypotheses...");
			for (typename hypotsp_t::iterator
					it = valid_hypots.begin(); it != valid_hypots.end(); ++it) {
				this->registerHypothesis(**it);
			}
		}
		// delete all hypotheses - generated in the heap...
		MRPT_LOG_DEBUG_STREAM("Deleting the generated hypotheses pool..." );
		for (typename hypotsp_t::iterator
				it = hypots_pool.begin(); it != hypots_pool.end(); ++it) {
			delete *it;
		}
	} // for each partition

	MRPT_LOG_DEBUG_STREAM("\n" << this->header_sep);
	this->m_time_logger.leave("LoopClosureEvaluation");

	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::evalPWConsistenciesMatrix(
		const mrpt::math::CMatrixDouble& consist_matrix,
		const hypotsp_t& hypots_pool,
		hypotsp_t* valid_hypots) {
	MRPT_START;
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	ASSERT_(valid_hypots);
	valid_hypots->clear();

	// evaluate the pair-wise consistency matrix
	// compute dominant eigenvector
	dynamic_vector<double> u;
	bool valid_lambda_ratio =
		this->computeDominantEigenVector(
				consist_matrix, &u,
				/*use_power_method=*/ false);
	if (!valid_lambda_ratio) return;

	//cout << "Dominant eigenvector: " << u.transpose() << endl;

	// discretize the indicator vector - maximize the dot product of
	// w_unit .* u
	ASSERT_(u.size());
	dynamic_vector<double> w(u.size(), 0); // discretized  indicator vector
	double dot_product = 0;
	for (int i = 0; i != w.size(); ++i) {
		// stream for debugging reasons
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
		//MRPT_LOG_DEBUG_STREAM(ss.str());
	}
	cout << "Outcome of discretization: " << w.transpose() << endl;
	//mrpt::system::pause();

	// Current hypothesis is to be registered.
	if (!w.isZero()) {
		for (int wi = 0; wi != w.size(); ++wi) {
			if (w(wi) == 1)  {
				// search through the potential hypotheses, find the one with the
				// correct ID and register it.
				valid_hypots->push_back(this->findHypotByID(hypots_pool, wi));
			}
		}
	}
	//mrpt::system::pause();
	
	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::splitPartitionToGroups(
		vector_uint& partition,
		vector_uint* groupA,
		vector_uint* groupB,
		int max_nodes_in_group/*=5*/) {
	MRPT_START;

	using namespace mrpt;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	// assertions
	ASSERTMSG_(groupA, "Pointer to groupA is not valid");
	ASSERTMSG_(groupB, "Pointer to groupB is not valid");
	ASSERTMSG_(max_nodes_in_group == -1 || max_nodes_in_group > 0,
			format(
				"Value %d not permitted for max_nodes_in_group"
				"Either use a positive integer, "
				"or -1 for non-restrictive partition size",
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
	// limit the number of nodes in each group
	if (max_nodes_in_group != -1) {
		// groupA
		if (groupA->size() > static_cast<size_t>(max_nodes_in_group)) {
			*groupA = vector_uint(groupA->begin(),
					groupA->begin() + max_nodes_in_group);
		}
		// groupB
		if (groupB->size() > static_cast<size_t>(max_nodes_in_group)) {
			*groupB = vector_uint(groupB->end() - max_nodes_in_group,
					groupB->end());
		}
	}

	//mrpt::system::pause();
	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::generateHypotsPool(
		const vector_uint& groupA,
		const vector_uint& groupB,
		hypotsp_t* generated_hypots,
		const TGenerateHypotsPoolAdParams* ad_params/*=NULL*/) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt;

	ASSERTMSG_(generated_hypots,
			"generateHypotsPool: Given hypotsp_t pointer is invalid.");
	generated_hypots->clear();

	MRPT_LOG_DEBUG_STREAM("Generating hypotheses for groups: " << endl);
	MRPT_LOG_DEBUG_STREAM("- groupA:\t" << getSTLContainerAsString(groupA) <<
		" - size: " << groupA.size() << endl);
	MRPT_LOG_DEBUG_STREAM("- groupB:\t" << getSTLContainerAsString(groupB) <<
		" - size: " << groupB.size() << endl);


	// verify that the number of laserScans is the same as the number of poses if
	// the TGenerateHyptsPoolAdParams is used
	// formulate into function and pass vector_uint and ad_params->group
	// TODO
	if (ad_params) {
		const typename TGenerateHypotsPoolAdParams::group_t& params =
			ad_params->groupA_params;
		if (params.size()) {
			size_t nodes_count = groupA.size();

			// map should have same size
			ASSERTMSG_(nodes_count == params.size(),
					format("Size mismatch between nodeIDs in group [%lu]"
						" and corresponding properties map [%lu]",
						nodes_count,
						params.size()));

		}
	}


	// use a hypothesis ID with which the consistency matrix will then be
	// formed
	int hypot_counter = 0;
	int invalid_hypots = 0; // just for keeping track of them.
	{
		// iterate over all the nodes in both groups
		for (vector_uint::const_iterator // B - from
				b_it = groupB.begin();
				b_it != groupB.end();
				++b_it) {
			for (vector_uint::const_iterator // A - to
					a_it = groupA.begin();
					a_it != groupA.end();
					++a_it) {
				// by default hypotheses will direct bi => ai; If the hypothesis is
				// traversed the opposite way, take the opposite of the constraint
				hypot_t* hypot = new hypot_t;
				hypot->from = *b_it;
				hypot->to = *a_it;
				hypot->id = hypot_counter++;

				// [from] *b_it ====[edge]===> [to]  *a_it

				// Fetch and set the pose and LaserScan of from, to nodeIDs
				//
				// even if icp_ad_params NULL, it will be handled appropriately by the
				// getICPEdge fun.
				//bool from_success, to_success;
				TGetICPEdgeAdParams* icp_ad_params = NULL;
				if (ad_params) {

					icp_ad_params = new TGetICPEdgeAdParams;
					this->fillNodePropsFromGroupParams(
							*b_it, ad_params->groupB_params, &icp_ad_params->from_params);
					this->fillNodePropsFromGroupParams(
							*a_it, ad_params->groupA_params, &icp_ad_params->to_params);

					//MRPT_LOG_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
					//MRPT_LOG_DEBUG_STREAM("From nodeID (other): " << *b_it);
					//MRPT_LOG_DEBUG_STREAM("From params (other): "
						//<< icp_ad_params->from_params.getAsString());
					//MRPT_LOG_DEBUG_STREAM("from_success: " << (from_success? "TRUE" : "FALSE"));
					//MRPT_LOG_DEBUG_STREAM("**********");
					//MRPT_LOG_DEBUG_STREAM("To nodeID (own)   : " << *a_it);
					//MRPT_LOG_DEBUG_STREAM("To params (own)   : "
						//<< icp_ad_params->to_params.getAsString());
					//MRPT_LOG_DEBUG_STREAM("to_success: " << (to_success? "TRUE" : "FALSE"));
					//MRPT_LOG_DEBUG_STREAM("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
				}

				// fetch the ICP constraint bi => ai
				mrpt::slam::CICP::TReturnInfo icp_info;
				constraint_t edge;
				bool found_edge = this->getICPEdge(
						*b_it,
						*a_it,
						&edge,
						&icp_info,
						icp_ad_params);

				hypot->setEdge(edge);
				hypot->goodness = icp_info.goodness; // goodness related to the edge

				// Check if invalid
				//
				// Goodness Threshold
				double goodness_thresh =
					this->m_goodness_threshold_win.getMedian() *
					m_lc_icp_constraint_factor;
				bool accept_goodness = icp_info.goodness > goodness_thresh;
				MRPT_LOG_DEBUG_STREAM( "generateHypotsPool:\nCurr. Goodness: " << icp_info.goodness
					<< "|\t Threshold: " << goodness_thresh << " => " <<
					(accept_goodness? "ACCEPT" : "REJECT") << endl);

				if (!found_edge || !accept_goodness) {
					hypot->is_valid = false;
					invalid_hypots++;
				}
				generated_hypots->push_back(hypot);
				MRPT_LOG_DEBUG_STREAM(hypot->getAsString());

				// delete pointer to getICPEdge additional parameters if they were
				// initialized
				delete icp_ad_params;
			}
		}
		MRPT_LOG_DEBUG_STREAM(
			"Generated pool of hypotheses...\tsize = "
			<< generated_hypots->size()
			<< "\tinvalid hypotheses: " << invalid_hypots);
	}
	//mrpt::system::pause();

	MRPT_END;
} // end of generateHypotsPool

template<class GRAPH_T>
bool CLoopCloserERD<GRAPH_T>::computeDominantEigenVector(
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
	bool is_valid_lambda_ratio = false;

	if (use_power_method) {
		THROW_EXCEPTION(
				"\nPower method for computing the first two eigenvectors/eigenvalues hasn't been implemented yet\n");
	}
	else { // call to eigenVectors method
		CMatrixDouble eigvecs, eigvals;
		consist_matrix.eigenVectors(eigvecs, eigvals);

		// assert that the eivenvectors, eigenvalues, consistency matrix are of the
		// same size
		ASSERTMSG_(
				eigvecs.size() == eigvals.size() &&
				consist_matrix.size() == eigvals.size(),
				mrpt::format(
					"Size of eigvecs \"%lu\","
					"eigvalues \"%lu\","
					"consist_matrix \"%lu\" don't match",
					static_cast<unsigned long>(eigvecs.size()),
					static_cast<unsigned long>(eigvals.size()),
					static_cast<unsigned long>(consist_matrix.size())));

		eigvecs.extractCol(eigvecs.getColCount()-1, *eigvec);
		lambda1 = eigvals(eigvals.getRowCount()-1, eigvals.getColCount()-1);
		lambda2 = eigvals(eigvals.getRowCount()-2, eigvals.getColCount()-2);
	}

	// I don't care about the sign of the eigenvector element
	for (int i = 0; i != eigvec->size(); ++i) {
		(*eigvec)(i) = abs((*eigvec)(i));
	}

	// check the ratio of the two eigenvalues - reject hypotheses set if ratio
	// smaller than threshold
	if (approximatelyEqual(0.0, lambda2, /**limit = */ 0.00001)) {
		MRPT_LOG_ERROR_STREAM("Bad lambda2 value: " << lambda2
			<< " => Skipping current evaluation." << endl);
		return false;
	}
	double curr_lambda_ratio = lambda1 / lambda2;
	MRPT_LOG_DEBUG_STREAM(
			"lambda1 = " << lambda1 << " | lambda2 = " << lambda2
		<< "| ratio = " << curr_lambda_ratio << endl);

	is_valid_lambda_ratio =
		(curr_lambda_ratio > m_lc_params.LC_eigenvalues_ratio_thresh);

	this->m_time_logger.leave("DominantEigenvectorComputation");
	return is_valid_lambda_ratio;

	MRPT_END;
} // end of computeDominantEigenVector

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::generatePWConsistenciesMatrix(
		const vector_uint& groupA,
		const vector_uint& groupB,
		const hypotsp_t& hypots_pool,
		mrpt::math::CMatrixDouble* consist_matrix,
		const paths_t* groupA_opt_paths/*=NULL*/,
		const paths_t* groupB_opt_paths/*=NULL*/) {
	MRPT_START;

	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::utils;
	using namespace std;
	ASSERTMSG_(consist_matrix, "Invalid pointer to the Consistency matrix is given");
	ASSERTMSG_(
			static_cast<unsigned long>(consist_matrix->rows()) == hypots_pool.size() &&
			static_cast<unsigned long>(consist_matrix->rows()) == hypots_pool.size(),
			"Consistency matrix dimensions aren't equal to the hypotheses pool size");

	MRPT_LOG_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl
		<< "In generatePWConsistencyMatrix:\n"
		<< "\tgroupA: " << getSTLContainerAsString(groupA) << endl
		<< "\tgroupB: " << getSTLContainerAsString(groupB) << endl
		<< "\tHypots pool Size: " << hypots_pool.size());

	// b1
	for (vector_uint::const_iterator
			b1_it = groupB.begin(); b1_it != groupB.end(); ++b1_it) {
		TNodeID b1 = *b1_it;

		// b2
		for (vector_uint::const_iterator
				b2_it = b1_it+1; b2_it != groupB.end(); ++b2_it) {
			TNodeID b2 = *b2_it;

			// a1
			for (vector_uint::const_iterator
					a1_it = groupA.begin(); a1_it != groupA.end(); ++a1_it) {

				TNodeID a1 = *a1_it;
				hypot_t* hypot_b2_a1 = this->findHypotByEnds(hypots_pool, b2, a1);
				//MRPT_LOG_DEBUG_STREAM("hypot_b2_a1: " << hypot_b2_a1->getAsString());

				// a2
				for (vector_uint::const_iterator
						a2_it = a1_it+1; a2_it != groupA.end(); ++a2_it) {
					TNodeID a2 = *a2_it;
					hypot_t* hypot_b1_a2 = this->findHypotByEnds(hypots_pool, b1, a2);
					//MRPT_LOG_DEBUG_STREAM("hypot_b1_a2: " << hypot_b1_a2->getAsString());

					double consistency;

					// compute consistency element
					if (hypot_b2_a1->is_valid && hypot_b1_a2->is_valid) {
						// extract vector of hypotheses that connect the given nodes,
						// instead of passing the whole hypothesis pool.
						hypotsp_t extracted_hypots;
						extracted_hypots.push_back(hypot_b2_a1);
						extracted_hypots.push_back(hypot_b1_a2);

						paths_t* curr_opt_paths=NULL;
						if (groupA_opt_paths || groupB_opt_paths) { // fill curr_opt_paths
							curr_opt_paths = new paths_t();
						}

						// decide on additional optimal paths
						if (curr_opt_paths) {
							// groupA
							if (groupA_opt_paths) { // a1 -> a2 optimal path
								const path_t* p = this->findPathByEnds(
										*groupA_opt_paths, a1, a2, /*throw_exc=*/ true);
								curr_opt_paths->push_back(*p);
							}
							else { // empty
								curr_opt_paths->push_back(path_t());
							}

							if (groupB_opt_paths) { // b1 -> b2 optimal path
								const path_t* p = this->findPathByEnds(
										*groupB_opt_paths, b1, b2, /*throw_exc=*/ true);
								curr_opt_paths->push_back(*p);
							}
							else { // empty
								curr_opt_paths->push_back(path_t());
							}

						}

						consistency = this->generatePWConsistencyElement(
								a1,a2,b1,b2,
								extracted_hypots,
								curr_opt_paths);

						delete curr_opt_paths;
					}
					else { //  null those that don't look good
						consistency = 0;
					}

					//MRPT_LOG_DEBUG_STREAM(
						//"Adding hypothesis consistency for nodeIDs: "
						//<< "[b1] " << b1 << ", [b2] -> " << b2
						//<< ", [a1] -> " << a1 << ", [a2] -> " << a2
						//<< " ==> " << consistency << endl);

					// fill the PW consistency matrix corresponding element - symmetrical
					int id1 = hypot_b2_a1->id;
					int id2 = hypot_b1_a2->id;

					(*consist_matrix)(id1, id2) = consistency;
					(*consist_matrix)(id2, id1) = consistency;

					//MRPT_LOG_DEBUG_STREAM("id1 = " << id1 << "\t|"
						//<< "id2 = " << id2 << "\t|"
						//<< "consistency = " << consistency);
				}
			}
		}
	}

	//MRPT_LOG_WARN_STREAM("Consistency matrix:" << endl
		//<< this->header_sep << endl
		//<< *consist_matrix << endl);

	MRPT_END;
} // end of generatePWConsistenciesMatrix


template<class GRAPH_T>
double CLoopCloserERD<GRAPH_T>::generatePWConsistencyElement(
		const mrpt::utils::TNodeID& a1,
		const mrpt::utils::TNodeID& a2,
		const mrpt::utils::TNodeID& b1,
		const mrpt::utils::TNodeID& b2,
		const hypotsp_t& hypots,
		const paths_t* opt_paths/*=NULL*/) {
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::math;
	using namespace mrpt::utils;
	using namespace mrpt::graphslam;
	using namespace mrpt::graphslam::detail;

	//MRPT_LOG_DEBUG_STREAM("In generatePWConsistencyElement.\n"
		//<< "\t[" << a1 << ", " << a2 << "]\n"
		//<< "\t[" << b1 << ", " << b2 << "]");
	//MRPT_LOG_DEBUG_STREAM("a1->a2 optimal path:"
		//<< (opt_paths && !opt_paths->begin()->isEmpty()?
				//opt_paths->begin()->getAsString() :
				//"NONE"));
	//MRPT_LOG_DEBUG_STREAM("b1->b2 optimal path: "
		//<< (opt_paths && !opt_paths->rbegin()->isEmpty()?
				//opt_paths->rbegin()->getAsString() :
				//"NONE"));

	// standard size assertions
	ASSERT_(hypots.size() == 2);
	if (opt_paths) { ASSERT_(opt_paths->size() == 2); }

	//
	// get the Dijkstra links
	//
	// a1 ==> a2
	const path_t* path_a1_a2;
	if (!opt_paths || opt_paths->begin()->isEmpty()) {
		MRPT_LOG_DEBUG_STREAM("Running dijkstra [a1] " << a1 << " => [a2] "<< a2);
		execDijkstraProjection(/*starting_node=*/ a1, /*ending_node=*/ a2);
		path_a1_a2 = this->queryOptimalPath(a2);
	}
	else { // fetch the path from the opt_paths arg
		// TODO dubious practice
		path_a1_a2 = &(*opt_paths->begin());
	}
	ASSERT_(path_a1_a2);
	path_a1_a2->assertIsBetweenNodeIDs(/*start=*/a1, /*end*/a2);

	// b1 ==> b2
	const path_t* path_b1_b2;
	if (!opt_paths || opt_paths->rend()->isEmpty()) {
		MRPT_LOG_DEBUG_STREAM("Running djkstra [b1] " << b1 << " => [b2] "<< b2);
		execDijkstraProjection(/*starting_node=*/ b1, /*ending_node=*/ b2);
		path_b1_b2 = this->queryOptimalPath(b2);
	}
	else { // fetch the path from the opt_paths arg
		path_b1_b2 = &(*opt_paths->rbegin());
	}
	ASSERT_(path_b1_b2);
	path_b1_b2->assertIsBetweenNodeIDs(/*start=*/b1, /*end*/b2);
	// get the edges of the hypotheses
	// by default hypotheses are stored bi => ai
	//
	// Backwards edge: a2=>b1
	hypot_t* hypot_b1_a2 = this->findHypotByEnds(hypots, b1, a2);
	// forward edge b2=>a1
	hypot_t* hypot_b2_a1 = this->findHypotByEnds(hypots, b2, a1);

	// Composition of Poses
	// Order : a1 ==> a2 ==> b1 ==> b2 ==> a1
	constraint_t res_transform(path_a1_a2->curr_pose_pdf);
	res_transform += hypot_b1_a2->getInverseEdge();
	res_transform += path_b1_b2->curr_pose_pdf;
	res_transform += hypot_b2_a1->getEdge();

	MRPT_LOG_DEBUG_STREAM("\n-----------Resulting Transformation----------- Hypots: #"
		<< hypot_b1_a2->id << ", #" << hypot_b2_a1->id << endl
		<< "a1 --> a2 => b1 --> b2 => a1: "
		<< a1 << " --> " << a2 << " => " << b1 << " --> " << b2 << " => " << a1 << endl
		<< res_transform << endl << endl
		<< "DIJKSTRA: " << a1 << " --> " << a2 << ": " << path_a1_a2->curr_pose_pdf << endl
		<< "DIJKSTRA: " << b1 << " --> " << b2 << ": " << path_b1_b2->curr_pose_pdf << endl
		<< "hypot_b1_a2(inv):\n" << hypot_b1_a2->getInverseEdge() << endl
		<< "hypot_b2_a1:\n" << hypot_b2_a1->getEdge() << endl);

	// get the vector of the corresponding transformation - [x, y, phi] form
	dynamic_vector<double> T;
	res_transform.getMeanVal().getAsVector(T);

	// information matrix
	CMatrixDouble33 cov_mat;
	res_transform.getCovariance(cov_mat);

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
} // end of generatePWConsistencyElement

template<class GRAPH_T>
const mrpt::graphslam::TUncertaintyPath<GRAPH_T>*
CLoopCloserERD<GRAPH_T>::findPathByEnds(
				const paths_t& vec_paths,
				const mrpt::utils::TNodeID& src,
				const mrpt::utils::TNodeID& dst,
				bool throw_exc/*=true*/) {
	using namespace mrpt;
	
	ASSERT_(vec_paths.size());
	const path_t* res = NULL;

	for (typename paths_t::const_iterator
			cit = vec_paths.begin();
			cit != vec_paths.end();
			++cit) {

		if (cit->getSource() == src && cit->getDestination() == dst) {
			res = &(*cit);
			break;
		}
	}
	if (throw_exc && !res) {
		THROW_EXCEPTION(
				format("Path for %lu => %lu is not found. Exiting...\n",
					static_cast<unsigned long>(src),
					static_cast<unsigned long>(dst)));
	}

	return res;
}

template<class GRAPH_T>
mrpt::graphs::detail::THypothesis<GRAPH_T>*
CLoopCloserERD<GRAPH_T>::findHypotByEnds(
		const hypotsp_t& vec_hypots,
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		bool throw_exc/*=true*/) {
	using namespace mrpt::graphslam::detail;
	using namespace std;

	for (typename hypotsp_t::const_iterator v_cit =
			vec_hypots.begin();
			v_cit != vec_hypots.end();
			v_cit++) {

		if ((*v_cit)->hasEnds(from, to)) {
			//cout << "findHypotByEnds: Found hypot " << from
				//<< " => " << to << " : " << (*v_cit)->getAsString() << endl;
			return *v_cit;
		}
	}

	// not found.
	if (throw_exc) {
 		throw HypothesisNotFoundException(from, to);
 	}
 	else {
		return NULL;
	}
} // end of findHypotByEnds

template<class GRAPH_T>
mrpt::graphs::detail::THypothesis<GRAPH_T>*
CLoopCloserERD<GRAPH_T>::findHypotByID(
		const hypotsp_t& vec_hypots,
		const size_t& id,
		bool throw_exc/*=true*/) {
	using namespace mrpt::graphslam::detail;

	for (typename hypotsp_t::const_iterator v_cit =
			vec_hypots.begin();
			v_cit != vec_hypots.end();
			v_cit++) {

		if ((*v_cit)->id == id) {
			return *v_cit;
		}
	}

	// not found.
	if (throw_exc) {
 		throw HypothesisNotFoundException(id);
 	}
 	else {
		return NULL;
	}
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::execDijkstraProjection(
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
	const std::string dijkstra_end =
		"----------- Done with Dijkstra Projection... ----------";

	// ending_node is either INVALID_NODEID or one of the already registered
	// nodeIDs
	ASSERT_(ending_node == INVALID_NODEID ||
			(ending_node >= 0 && ending_node < this->m_graph->nodeCount()) );
	ASSERTMSG_(starting_node != ending_node, "Starting and Ending nodes coincede");

	// debugging message
	stringstream ss_debug("");
	ss_debug << "Executing Dijkstra Projection: " << starting_node << " => ";
	if (ending_node == INVALID_NODEID) {
		ss_debug << "..." << endl;
	}
	else {
		ss_debug << ending_node <<endl;
	}

	if (this->m_graph->nodeCount() < m_dijkstra_node_count_thresh) {
		return;
	}

	// keep track of the nodes that I have visited
	std::vector<bool> visited_nodes(this->m_graph->nodeCount(), false);
	m_node_optimal_paths.clear();

	// get the neighbors of each node
	std::map<TNodeID, std::set<TNodeID> > neighbors_of;
	this->m_graph->getAdjacencyMatrix(neighbors_of);

	// initialize a pool of TUncertaintyPaths - draw the minimum-uncertainty path during
	// execution
	std::set<path_t*> pool_of_paths;
	// get the edge to each one of the neighboring nodes of the starting node
	std::set<TNodeID> starting_node_neighbors(neighbors_of.at(starting_node));
	for (std::set<TNodeID>::const_iterator n_it =
			starting_node_neighbors.begin();
			n_it != starting_node_neighbors.end(); ++n_it) {

		path_t* path_between_neighbors =
			new path_t();
		this->getMinUncertaintyPath(starting_node, *n_it, path_between_neighbors);

		pool_of_paths.insert(path_between_neighbors);
	}
	// just visited the first node
	visited_nodes.at(starting_node) = true;

	//// TODO Remove these - >>>>>>>>>>>>>>>>>>>>
	//// printing the pool for verification
	//cout << "Pool of Paths: " << endl;
	//for (typename std::set<path_t*>::const_iterator it = pool_of_paths.begin();
			//it != pool_of_paths.end(); ++it) {
		//printSTLContainer((*it)->nodes_traversed);
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
				//MRPT_LOG_DEBUG_STREAM(dijkstra_end);
				this->m_time_logger.leave("Dijkstra Projection");
				return;
			}
		}

		path_t* optimal_path =
			this->popMinUncertaintyPath(&pool_of_paths);
		TNodeID dest = optimal_path->getDestination();

		//// TODO Remove these - >>>>>>>>>>>>>>>>>>>> ^^^UNCOMMENT ABOVE AS WELL^^^
		//cout << iters << " " << std::string(40, '>') << endl;
		//cout << "current path Destination: " << dest << endl;
		//// printing the pool for verification
		//cout << "Pool of Paths: " << endl;
		//for (typename std::set<path_t*>::const_iterator
				//it = pool_of_paths.begin();
				//it != pool_of_paths.end(); ++it) {
			//printSTLContainer((*it)->nodes_traversed);
		//}
		//cout << "Nodes visited: " << endl;
		//std::vector<int> tmp_vec;
		//for (int i = 0; i != visited_nodes.size(); ++i) {
			//tmp_vec.push_back(i);
		//}
		//printSTLContainer(tmp_vec); cout << endl; // indices of numbers
		//printSTLContainer(visited_nodes);         // actual flags
		//cout << std::string(40, '<') << " " << iters++ << endl;
		//mrpt::system::pause();
		//// TODO Remove these - <<<<<<<<<<<<<<<<<<<<<

		if (!visited_nodes.at(dest)) {
			m_node_optimal_paths[dest] = optimal_path;
			visited_nodes.at(dest)= true;

			// for all the edges leaving this node .. compose the transforms with the
			// current pool of paths.
			this->addToPaths(&pool_of_paths, *optimal_path, neighbors_of.at(dest));
		}
	}

	//MRPT_LOG_DEBUG_STREAM(dijkstra_end);
	this->m_time_logger.leave("Dijkstra Projection");
	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::addToPaths(
		std::set<path_t*>* pool_of_paths,
		const path_t& current_path,
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
		path_t path_between_nodes;
		this->getMinUncertaintyPath(node_to_append_from, *neigh_it,
				&path_between_nodes);

		// format the path to append
		path_t* path_to_append =
			new path_t();
		*path_to_append = current_path;
		*path_to_append += path_between_nodes;

		pool_of_paths->insert(path_to_append);
	}

	MRPT_END;
}

template<class GRAPH_T>
typename mrpt::graphslam::TUncertaintyPath<GRAPH_T>*
CLoopCloserERD<GRAPH_T>::queryOptimalPath(const mrpt::utils::TNodeID node) const {
	using namespace std;

	path_t* path = NULL;
	typename std::map<mrpt::utils::TNodeID,
					 path_t*>::const_iterator search;
	search = m_node_optimal_paths.find(node);
	if (search != m_node_optimal_paths.end()) {
		path = search->second;
	}

	//MRPT_LOG_DEBUG_STREAM("Queried optimal path for nodeID: " << node
		//<< " ==> Path: " << (path? "Found" : "NOT Found"));
	return path;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::getMinUncertaintyPath(
		const mrpt::utils::TNodeID from,
		const mrpt::utils::TNodeID to,
		path_t* path_between_nodes) const {
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

		path_t curr_path(from); // set the starting node
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

		path_t curr_path(from); // set the starting node
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

template<class GRAPH_T>
TUncertaintyPath<GRAPH_T>* CLoopCloserERD<GRAPH_T>::
popMinUncertaintyPath(
		typename std::set<path_t*>* pool_of_paths) const {
	MRPT_START;
	using namespace std;

	//cout << "Determinants: ";
	path_t* optimal_path = NULL;
	double curr_determinant = 0;
	for (typename std::set<path_t*>::const_iterator
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

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::registerHypothesis(const hypot_t& hypot) {
	//MRPT_LOG_DEBUG_STREAM("Registering hypothesis: " <<
		//hypot.getAsString([>oneline=<] true));
	this->registerNewEdge(hypot.from, hypot.to, hypot.getEdge());
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::registerNewEdge(
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
	if (absDiff(to, from) > m_lc_params.LC_min_nodeid_diff)  {
		this->m_edge_types_to_nums["LC"]++;
		this->m_just_inserted_lc = true;
		this->logFmt(LVL_INFO, "\tLoop Closure edge!");
	}
	else {
		this->m_just_inserted_lc = false;
	}
	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	MRPT_START;
	parent_t::notifyOfWindowEvents(events_occurred);

	// map partitions
	if (events_occurred.at(m_lc_params.keystroke_map_partitions)) {
		this->toggleMapPartitionsVisualization();
	}

	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::initMapPartitionsVisualization() {
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

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::updateMapPartitionsVisualization() {
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
	bool found_last_node = false; // last node must exist in one partition at all cost TODO
	MRPT_LOG_DEBUG_STREAM("Searching for the partition of the last nodeID: "
		<< (this->m_graph->nodeCount()-1));

	for (partitions_t::const_iterator p_it = m_curr_partitions.begin();
			p_it != m_curr_partitions.end(); ++p_it, ++partitionID) {

		//MRPT_LOG_DEBUG_STREAM("Working on Partition #" << partitionID);
		vector_uint nodes_list = *p_it;

		// finding the partition in which the last node is in
		if (std::find(
					nodes_list.begin(),
					nodes_list.end(),
					this->m_graph->nodeCount()-1) != nodes_list.end()) {
			partition_contains_last_node = true;

			found_last_node = true;
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
			//MRPT_LOG_DEBUG_STREAM(
					//"\tFetching CSetOfObjects partition object for partition #" <<
					//partitionID);
			curr_partition_obj = static_cast<CSetOfObjectsPtr>(obj);
			if (m_lc_params.LC_check_curr_partition_only) { // make all but the last partition invisible
				curr_partition_obj->setVisibility(partition_contains_last_node); 
			}
		}
		else {
			MRPT_LOG_DEBUG_STREAM(
				"\tCreating a new CSetOfObjects partition object for partition #" <<
				partitionID);
			curr_partition_obj = CSetOfObjects::Create();
			curr_partition_obj->setName(partition_obj_name);
			if (m_lc_params.LC_check_curr_partition_only) {
 				// make all but the last partition invisible
				curr_partition_obj->setVisibility(partition_contains_last_node); 
			}

			//MRPT_LOG_DEBUG_STREAM("\t\tCreating a new CSphere balloon object");
			CSpherePtr balloon_obj = CSphere::Create();
			balloon_obj->setName(balloon_obj_name);
			balloon_obj->setRadius(m_lc_params.balloon_radius);
			balloon_obj->setColor_u8(m_lc_params.balloon_std_color);
			balloon_obj->enableShowName();

			curr_partition_obj->insert(balloon_obj);

			// set of lines connecting the graph nodes to the balloon
			//MRPT_LOG_DEBUG_STREAM(
				//"\t\tCreating set of lines that will connect to the Balloon");
			CSetOfLinesPtr connecting_lines_obj = CSetOfLines::Create();
			connecting_lines_obj->setName("connecting_lines");
			connecting_lines_obj->setColor_u8(m_lc_params.connecting_lines_color);
			connecting_lines_obj->setLineWidth(0.1f);

			curr_partition_obj->insert(connecting_lines_obj);

			// add the created CSetOfObjects to the total CSetOfObjects responsible
			// for the map partitioning
			map_partitions_obj->insert(curr_partition_obj);
			//MRPT_LOG_DEBUG_STREAM("\tInserted new CSetOfObjects successfully");
		}
		// up to now the CSetOfObjects exists and the balloon inside it as well..

		std::pair<double, double> centroid_coords;
		this->computeCentroidOfNodesVector(nodes_list, &centroid_coords);

		TPoint3D balloon_location(centroid_coords.first, centroid_coords.second,
				m_lc_params.balloon_elevation);

		//MRPT_LOG_DEBUG_STREAM("\tUpdating the balloon position");
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

		//MRPT_LOG_DEBUG_STREAM("\tUpdating the lines connecting nodes to balloon");
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
		//MRPT_LOG_DEBUG_STREAM("Done working on partition #" << partitionID);
	}

	if (!found_last_node) {
		MRPT_LOG_ERROR_STREAM("Last inserted nodeID was not found in any partition.");
		ASSERT_(found_last_node);
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
			if (!obj) {
				MRPT_LOG_ERROR_STREAM( "Partition : " << partition_obj_name
				<< " was not found.");
				ASSERT_(!obj);
			}
			map_partitions_obj->removeObject(obj);
		}
	}
	MRPT_LOG_DEBUG_STREAM("Done working on the partitions visualization.");


	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();
} // end of updateMapPartitionsVisualization

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::toggleMapPartitionsVisualization() {
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
} // end of toggleMapPartitionsVisualization

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::computeCentroidOfNodesVector(
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
} // end of computeCentroidOfNodesVector

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::initializeVisuals() {
	MRPT_START;
	parent_t::initializeVisuals();
	this->m_time_logger.enter("ERD::Visuals");

	if (m_lc_params.visualize_map_partitions) {
		this->initMapPartitionsVisualization();
	}

	if (m_visualize_curr_node_covariance) {
		this->initCurrCovarianceVisualization();
	}

	// keystrokes
	this->m_win_observer->registerKeystroke(
			m_lc_params.keystroke_map_partitions,
			"Toggle Map Partitions Visualization");

	this->m_time_logger.leave("ERD::Visuals");
	MRPT_END;
}
template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::updateVisuals() {
	MRPT_START;
	parent_t::updateVisuals();
	//MRPT_LOG_DEBUG_STREAM("Updating visuals");
	this->m_time_logger.enter("ERD::Visuals");

	if (m_lc_params.visualize_map_partitions) {
		this->updateMapPartitionsVisualization();
	}
	if (m_visualize_curr_node_covariance) {
		this->updateCurrCovarianceVisualization();
	}

	this->m_time_logger.leave("ERD::Visuals");
	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::initCurrCovarianceVisualization() {
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

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::updateCurrCovarianceVisualization() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::math;
	using namespace mrpt::opengl;
	using namespace mrpt::gui;
	using namespace mrpt::utils;

	// get the optimal path to the current node
	mrpt::utils::TNodeID curr_node = this->m_graph->nodeCount()-1;
	path_t* path =	queryOptimalPath(curr_node);
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

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::loadParams(const std::string& source_fname) {
	MRPT_START;
	parent_t::loadParams(source_fname);

	std::string section ="EdgeRegistrationDeciderParameters";

	m_partitioner.options.loadFromConfigFileName(source_fname,
			section);
	m_lc_params.loadFromConfigFileName(source_fname,
			section);

	mrpt::utils::CConfigFile source(source_fname);

	m_use_scan_matching = source.read_bool(
			section,
			"use_scan_matching",
			true, false);

	m_lc_icp_constraint_factor = source.read_double(
			section,
			"lc_icp_constraint_factor",
			0.70, false);


	// set the logging level if given by the user
	int min_verbosity_level = source.read_int(
			section,
			"class_verbosity",
			1, false);


	this->setMinLoggingLevel(mrpt::utils::VerbosityLevel(min_verbosity_level));
	MRPT_END;
}
template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::printParams() const {
	MRPT_START;
	using namespace std;

	cout << "------------------[Pair-wise Consistency of ICP Edges - Registration Procedure Summary]------------------" << endl;

	parent_t::printParams();
	m_partitioner.options.dumpToConsole();
	m_lc_params.dumpToConsole();

	cout << "Use scan-matching constraints      : "
		<< (m_use_scan_matching? "TRUE": "FALSE") << endl;
	cout << "Loop-closure ICP Constraint factor :  "
		<< m_lc_icp_constraint_factor << endl;

	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;

	// Report on graph
	std::stringstream class_props_ss;
	class_props_ss <<
		"Pair-wise Consistency of ICP Edges - Registration Procedure Summary: " <<
		std::endl;
	class_props_ss << this->header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent_t::getDescriptiveReport(report_str);

	*report_str += class_props_ss.str();
	*report_str += this->report_sep;

	*report_str += time_res;
	*report_str += this->report_sep;

	*report_str += output_res;
	*report_str += this->report_sep;

	MRPT_END;
} // end of getDescriptiveReport

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::getCurrPartitions(partitions_t* partitions_out) const {
	ASSERT_(partitions_out);
	*partitions_out = this->getCurrPartitions();
}

template<class GRAPH_T>
const std::vector<mrpt::vector_uint>&
CLoopCloserERD<GRAPH_T>::getCurrPartitions() const {
	return m_curr_partitions;
}

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::updateMapPartitions(
		bool full_update /* = false */,
		bool is_first_time_node_reg /* = false */) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace mrpt::math;
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
		nodes_to_scans = this->m_nodes_to_laser_scans2D;
	}
	else {
		// if registering measurement for root node as well...
		if (is_first_time_node_reg) {
			nodes_to_scans.insert(
					make_pair(this->m_graph->root,
						this->m_nodes_to_laser_scans2D.at(this->m_graph->root)));
		}

		// just use the last node-laser scan pair
		nodes_to_scans.insert(
				make_pair(
					this->m_graph->nodeCount()-1,
					this->m_nodes_to_laser_scans2D.at(this->m_graph->nodeCount()-1)));
	}

	// TODO - Should always exist.
	// for each one of the above nodes - add its position and correspoding
	// laserScan to the partitioner object
	for (typename nodes_to_scans2D_t::const_iterator
			it = nodes_to_scans.begin();
			it != nodes_to_scans.end(); ++it) {
		if ((it->second).null()) { // if laserScan invalid go to next...
			MRPT_LOG_WARN_STREAM(
				"nodeID \"" << it->first << "\" has invalid laserScan");
			continue;
		}

		// find pose of node, if it exists...
		// TODO - investigate this case. Why should this be happening?
		typename GRAPH_T::global_poses_t::const_iterator search;
		search = this->m_graph->nodes.find(it->first);
		if (search == this->m_graph->nodes.end()) {
			MRPT_LOG_WARN_STREAM("Couldn't find pose for nodeID " << it->first);
			continue;
		}

		// pose
		pose_t curr_pose = search->second;
		mrpt::poses::CPosePDFPtr posePDF(new constraint_t(curr_pose));

		// laser scan
		mrpt::obs::CSensoryFramePtr sf = mrpt::obs::CSensoryFrame::Create();
		sf->insert(it->second);

		m_partitioner.addMapFrame(sf, posePDF);
		//mrpt::system::pause();
	}

	// update the last partitions list
	size_t curr_size = m_curr_partitions.size();
	m_last_partitions.resize(curr_size);
	for (size_t i = 0; i < curr_size; i++)	{
		m_last_partitions[i] = m_curr_partitions[i];
	}
	//update current partitions list
	m_partitioner.updatePartitions(m_curr_partitions);

	MRPT_LOG_DEBUG_STREAM("Updated map partitions successfully.");
	this->m_time_logger.leave("updateMapPartitions");
	MRPT_END;
} // end of updateMapPartitions

// TLoopClosureParams
// //////////////////////////////////


template<class GRAPH_T>
CLoopCloserERD<GRAPH_T>::TLoopClosureParams::TLoopClosureParams():
	keystroke_map_partitions("b"),
	balloon_elevation(3),
	balloon_radius(0.5),
	balloon_std_color(153, 0, 153),
	balloon_curr_color(62, 0, 80),
	connecting_lines_color(balloon_std_color),
	has_read_config(false)
{
}

template<class GRAPH_T>
CLoopCloserERD<GRAPH_T>::TLoopClosureParams::~TLoopClosureParams() { }

template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::TLoopClosureParams::dumpToTextStream(
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
	ss << "Visualize map partitions                              = " <<
		(visualize_map_partitions?  "TRUE": "FALSE") << endl;

	out.printf("%s", ss.str().c_str());

	MRPT_END;
}
template<class GRAPH_T>
void CLoopCloserERD<GRAPH_T>::TLoopClosureParams::loadFromConfigFile(
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
	visualize_map_partitions = source.read_bool(
			"VisualizationParameters",
			"visualize_map_partitions",
			true, false);

	has_read_config = true;
	MRPT_END;
}

} } } // end of namespaces

#endif /* end of include guard: CLOOPCLOSERERD_IMPL_H */
