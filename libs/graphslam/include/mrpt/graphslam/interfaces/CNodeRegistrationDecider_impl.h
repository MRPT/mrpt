#ifndef CNODEREGISTRATIONDECIDER_IMPL_H
#define CNODEREGISTRATIONDECIDER_IMPL_H

using namespace mrpt::graphslam::deciders;
using namespace std;

#include <sstream>

// Implementation of classes defined in the CNodeRegistrationDecider class
// template.
//
template<class GRAPH_T>
CNodeRegistrationDecider<GRAPH_T>::CNodeRegistrationDecider():
	m_prev_registered_nodeID(INVALID_NODEID) {
		using namespace mrpt::poses;

		m_init_inf_mat.unit();
		m_init_inf_mat *= 10000;
	
}

template<class GRAPH_T>
CNodeRegistrationDecider<GRAPH_T>::~CNodeRegistrationDecider() {
}

template<class GRAPH_T>
void CNodeRegistrationDecider<GRAPH_T>::getDescriptiveReport(
		std::string* report_str) const {
	MRPT_START;

	stringstream ss("");
	parent_t::getDescriptiveReport(report_str);

	ss << "Node Registration Decider Strategy [NRD]: " << endl;
	*report_str += ss.str();

	MRPT_END;
}

template<class GRAPH_T>
bool CNodeRegistrationDecider<GRAPH_T>::checkRegistrationCondition() {
	return false;
}

template<class GRAPH_T>
bool CNodeRegistrationDecider<GRAPH_T>::registerNewNodeAtEnd(
		const typename GRAPH_T::constraint_t constraint) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	// register the initial node if it doesn't exist.
	if (this->m_prev_registered_nodeID == INVALID_NODEID) { // root
		MRPT_LOG_WARN_STREAM << "Registering root node..." << endl;
		this->m_graph->nodes[this->m_graph->root] =
			this->getCurrentRobotPosEstimation();
		this->m_prev_registered_nodeID = this->m_graph->root;
	}

	TNodeID from = this->m_prev_registered_nodeID;
	TNodeID to = from + 1;

	
	this->m_graph->nodes[to] = this->getCurrentRobotPosEstimation();
	this->m_graph->insertEdgeAtEnd(from, to, constraint);

	m_prev_registered_nodeID++;

	MRPT_LOG_DEBUG_STREAM << "Registered new node:" << endl <<
		"\t" << from << " => " << to << endl <<
		"\tEdge: " << constraint.getMeanVal().asString();

	return true;
	MRPT_END;
}


template<class GRAPH_T>
typename GRAPH_T::global_pose_t
CNodeRegistrationDecider<GRAPH_T>::addNodeAnnotsToPose(
		const global_pose_t& pose) const {return pose;}

#endif /* end of include guard: CNODEREGISTRATIONDECIDER_IMPL_H */
