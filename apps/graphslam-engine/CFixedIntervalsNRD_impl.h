/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CFIXEDINTERVALSNRD_IMPL_H
#define CFIXEDINTERVALSNRD_IMPL_H

using namespace mrpt::graphslam::deciders;

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::CFixedIntervalsNRD_t():
	m_consecutive_invalid_format_instances_thres(20) // large threshold just to make sure
{
	this->initCFixedIntervalsNRD_t();
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::initCFixedIntervalsNRD_t() {

	m_win = NULL;
	m_graph = NULL;

	m_prev_registered_node = INVALID_NODEID;

	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 1.0 };
	InfMat init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	m_checked_for_usuable_dataset = false;
	m_consecutive_invalid_format_instances = 0;

	m_logger.setName("CFixedIntervalsNRD");
	m_logger.setLoggingLevel(LVL_DEBUG);
	m_logger.setMinLoggingLevel(LVL_INFO);

	m_logger.log("IntervalsNRD: Initialized class object", LVL_DEBUG);
}
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::~CFixedIntervalsNRD_t() { }

// Member function implementations
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;

	//cout << "in updateDeciderState..." << endl;
	// don't use the measurements in this implementation
	MRPT_UNUSED_PARAM(observations);

	if (observation.present()) { // FORMAT #2
		m_observation_only_rawlog = true;

		if (IS_CLASS(observation, CObservationOdometry)) {

			CObservationOdometryPtr obs_odometry = 
				static_cast<CObservationOdometryPtr>(observation);
			// not incremental - gives the absolute odometry reading
			m_curr_odometry_only_pose = obs_odometry->odometry;
			m_logger.log(mrpt::format("Current odometry-only pose: %s", m_curr_odometry_only_pose.asString().c_str()),
						LVL_DEBUG);

			// I don't have any information about the covariane of the move in
			// observation-only format
			m_since_prev_node_PDF.mean =
				m_curr_odometry_only_pose - m_last_odometry_only_pose;
		}
	} // IF FORMAT #2 - observation-only
	else { // FORMAT #1
		m_observation_only_rawlog = false;

		mrpt::obs::CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
		mrpt::poses::CPosePDFPtr increment = robot_move->poseChange;
		pose_t increment_pose = increment->getMeanVal();
		InfMat increment_inf_mat;
		increment->getInformationMatrix(increment_inf_mat);

		// update the relative PDF of the path since the LAST node was inserted
		constraint_t incremental_constraint(increment_pose, increment_inf_mat);
		m_since_prev_node_PDF += incremental_constraint;
	} // ELSE - FORMAT #1

	m_curr_estimated_pose = m_graph->nodes[m_prev_registered_node] + 
		m_since_prev_node_PDF.getMeanVal();

	bool registered = this->checkRegistrationCondition();

	if (registered) {
		if (m_observation_only_rawlog) {
			// keep track of the odometry-only pose_t at the last inserted graph node
			m_last_odometry_only_pose = m_curr_odometry_only_pose;
		}
		m_since_prev_node_PDF = constraint_t();
	}

	if (!m_checked_for_usuable_dataset) {
		this->checkIfInvalidDataset(action, observations, observation );
	}

	return registered;

	MRPT_END;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::checkRegistrationCondition() {
	MRPT_START;

	bool registered = false;

	//cout << "in checkRegistrationCondition..." << endl;

	pose_t last_pose_inserted = m_graph->nodes[m_prev_registered_node];

	// odometry criterion
	if ( (last_pose_inserted.distanceTo(m_curr_estimated_pose) 
				> params.registration_max_distance) ||
			(fabs(wrapToPi(last_pose_inserted.phi() - m_curr_estimated_pose.phi())) 
			 > params.registration_max_angle ) ) {

		// register the new node
		registered = true;
		this->registerNewNode();
	}

	return registered;

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::registerNewNode() {
	m_logger.log("In registerNewNode...", LVL_DEBUG);

	mrpt::utils::TNodeID from = m_prev_registered_node;
	mrpt::utils::TNodeID to = ++m_prev_registered_node;

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
  m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	m_logger.log(mrpt::format("Registered new node:\n\t%lu => %lu\n\tEdge: %s",
				from, to, m_since_prev_node_PDF.getMeanVal().asString().c_str()), LVL_DEBUG);

}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	m_graph = graph;

	// get the last registrered node + corresponding pose - root
	m_prev_registered_node = m_graph->root;

	m_logger.log("Fetched the graph successfully", LVL_DEBUG);
}

template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::checkIfInvalidDataset(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {
	MRPT_START;

	MRPT_UNUSED_PARAM(observations);
	MRPT_UNUSED_PARAM(action);

	if (observation.present()) { // FORMAT #2
		if (IS_CLASS(observation, CObservationOdometry)) {
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
		m_logger.log("Can't find usuable data in the given dataset.\nMake sure dataset contains valid odometry data.",
				LVL_ERROR);
		mrpt::system::sleep(5000);
		m_checked_for_usuable_dataset = true;
	}

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	params.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");
	m_logger.log("Successfully loaded parameters.", LVL_DEBUG);

	MRPT_END;
}

template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::printParams() const {
	MRPT_START;
	params.dumpToConsole();

	MRPT_END;
}

// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::TParams::TParams() {
}
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::TParams::~TParams() {
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	double max_angle_deg = RAD2DEG(registration_max_angle);

	out.printf("------------------[ Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n", registration_max_distance);
	out.printf("Max angle for registration    = %.2f deg\n", max_angle_deg);

	MRPT_END;
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;
	registration_max_distance = source.read_double( section,
			"registration_max_distance",
			5 /* meter */, false);
	registration_max_angle = source.read_double( section,
			"registration_max_angle",
			60 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

	MRPT_END;
}

#endif /* end of include guard: CFIXEDINTERVALSNRD_IMPL_H */
