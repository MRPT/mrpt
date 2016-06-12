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
CFixedIntervalsNRD_t<GRAPH_t>::CFixedIntervalsNRD_t(GRAPH_t* graph):
	m_graph(graph)
{
	this->initCFixedIntervalsNRD_t();
}
template<class GRAPH_t>
CFixedIntervalsNRD_t<GRAPH_t>::CFixedIntervalsNRD_t() {
	this->initCFixedIntervalsNRD_t();
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::initCFixedIntervalsNRD_t() {

	m_win = NULL;
	m_graph = NULL;

	m_prev_registered_node = INVALID_NODEID;

	// Tracking the PDF of the current position of the robot with regards to the
	// PREVIOUS registered node
	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 1.0 };
	InfMat m_init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = m_init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	std::cout << "CFixedIntervalsNRD: Initialized class object" << std::endl;
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

	// don't use the measurements in this implementation
	MRPT_UNUSED_PARAM(observations);

	//cout << "in updateDeciderState..." << endl;

	if (observation.present()) { // FORMAT #2
		// TODO - implement this
	}
	else { // FORMAT #1
		// TODO - add input validation for the CActionRobotMovement2D -
		// GetRuntimeClass?
		mrpt::obs::CActionRobotMovement2DPtr robot_move = action->getBestMovementEstimation();
		mrpt::poses::CPosePDFPtr increment = robot_move->poseChange;
		pose_t increment_pose = increment->getMeanVal();
		InfMat increment_inf_mat;
		increment->getInformationMatrix(increment_inf_mat);

		// update the relative PDF of the path since the LAST node was inserted
		constraint_t incremental_constraint(increment_pose, increment_inf_mat);
		m_since_prev_node_PDF += incremental_constraint;

	}
	m_curr_estimated_pose = m_graph->nodes[m_prev_registered_node] + 
		m_since_prev_node_PDF.getMeanVal();

	bool registered = this->checkRegistrationCondition();
	return registered;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::checkRegistrationCondition() {
	bool registered = false;

	//cout << "in checkRegistrationCondition..." << endl;

	pose_t last_pose_inserted = m_graph->nodes[m_prev_registered_node];

	// odometry criterion
	if ( (last_pose_inserted.distanceTo(m_curr_estimated_pose) 
				> params.registration_max_distance) ||
			(fabs(wrapToPi(last_pose_inserted.phi() - m_curr_estimated_pose.phi())) 
			 > params.registration_max_angle ) ) {

		// register the new node
		registered = this->registerNewNode();
	}

	return registered;
}

template<class GRAPH_t>
bool CFixedIntervalsNRD_t<GRAPH_t>::registerNewNode() {
	bool registered = true; // By default it should be able to register the node

	//cout << "in registerNewNode..." << endl;

	mrpt::utils::TNodeID from = m_prev_registered_node;
	mrpt::utils::TNodeID to = ++m_prev_registered_node;

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
  m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	// reset the relative PDF
	m_since_prev_node_PDF.cov_inv = m_init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	return registered;
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	m_graph = graph;

	// get the last registrered node + corresponding pose - root
	m_prev_registered_node = m_graph->root;

	std::cout << "CFixedIntervalsNRD: Fetched the graph successfully" 
		<< std::endl;
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
	double max_angle_deg = RAD2DEG(registration_max_angle);

	out.printf("------------------[ Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n", registration_max_distance);
	out.printf("Max angle for registration    = %.2f deg\n", max_angle_deg);
}
template<class GRAPH_t>
void CFixedIntervalsNRD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
	registration_max_distance = source.read_double( section,
			"registration_max_distance",
			5 /* meter */, false);
	registration_max_angle = source.read_double( section,
			"registration_max_angle",
			60 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

}

#endif /* end of include guard: CFIXEDINTERVALSNRD_IMPL_H */
