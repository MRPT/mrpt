#ifndef CNODEREGISTRATIONDECIDER_IMPL_H
#define CNODEREGISTRATIONDECIDER_IMPL_H

using namespace mrpt::graphslam::deciders;
using namespace std;

#include <sstream>

// Implementation of classes defined in the CNodeRegistrationDecider class
// template.
//
template<class GRAPH_t>
CNodeRegistrationDecider<GRAPH_t>::CNodeRegistrationDecider():
	m_prev_registered_node(INVALID_NODEID) {

		m_init_inf_mat.unit();
		m_init_inf_mat *= 10000;
	
}

template<class GRAPH_t>
CNodeRegistrationDecider<GRAPH_t>::~CNodeRegistrationDecider() {
}

template<class GRAPH_t>
void CNodeRegistrationDecider<GRAPH_t>::getDescriptiveReport(
		std::string* report_str) const {
	MRPT_START;

	stringstream ss("");
	parent::getDescriptiveReport(report_str);

	ss << "Node Registration Decider Strategy [NRD]: " << endl;
	*report_str += ss.str();

	MRPT_END;
}

template<class GRAPH_t>
bool CNodeRegistrationDecider<GRAPH_t>::checkRegistrationCondition() {
	return false;
}

template<class GRAPH_t>
bool CNodeRegistrationDecider<GRAPH_t>::registerNewNode(
		const typename GRAPH_t::constraint_t constraint) {
	MRPT_START;
	using namespace mrpt::utils;
	using namespace std;

	// register the initial node if it doesn't exist.
	if (this->m_prev_registered_node == INVALID_NODEID) {
		MRPT_LOG_WARN_STREAM << "Registering root node..." << endl;
		//mrpt::system::pause();
		this->m_graph->nodes[this->m_graph->root] = pose_t();
		this->m_prev_registered_node = this->m_graph->root;
	}

	TNodeID from = this->m_prev_registered_node;
	TNodeID to = from + 1;

	this->m_graph->nodes[to] = this->getCurrentRobotPosEstimation();
	this->m_graph->insertEdgeAtEnd(from, to, constraint);

	m_prev_registered_node++;

	MRPT_LOG_DEBUG_STREAM << "Registered new node:" << endl <<
		"\t" << from << " => " << to << endl <<
		"\tEdge: " << constraint.getMeanVal().asString();

	return true;
	MRPT_END;
}


#endif /* end of include guard: CNODEREGISTRATIONDECIDER_IMPL_H */
