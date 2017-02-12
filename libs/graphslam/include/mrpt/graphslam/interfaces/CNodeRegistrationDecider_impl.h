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
	// Runs only once.
	if (this->m_prev_registered_nodeID == INVALID_NODEID) { // root
		MRPT_LOG_WARN_STREAM << "Registering root node..." << endl;
		global_pose_t tmp_pose =
			this->getCurrentRobotPosEstimation();
		this->addNodeAnnotsToPose(&tmp_pose);

		// make sure that this pair hasn't been registered yet.
		std::pair<typename GRAPH_T::global_poses_t::const_iterator, bool> res =
			this->m_graph->nodes.insert(make_pair(
						this->m_graph->root, tmp_pose));
		ASSERTMSG_(res.second, 
				mrpt::format(
					"nodeID \"%lu\" with pose \"%s\" seems to be already registered.",
					this->m_graph->root, tmp_pose.asString().c_str()));

		this->m_prev_registered_nodeID = this->m_graph->root;
	}

	// FROM nodeID
	TNodeID from = this->m_prev_registered_nodeID;
	// TO nodeID
	// In all cases this is going to be ONE AFTER the last registered nodeID
	TNodeID to = this->m_graph->nodeCount();

	// add the new pose.
	{
		global_pose_t tmp_pose = 
			this->getCurrentRobotPosEstimation();
		this->addNodeAnnotsToPose(&tmp_pose);

		// make sure that this pair hasn't been registered yet.
		std::pair<typename GRAPH_T::global_poses_t::const_iterator, bool> res =
			this->m_graph->nodes.insert(make_pair(
						to, tmp_pose));
		ASSERTMSG_(res.second, 
				mrpt::format(
					"nodeID \"%lu\" with pose \"%s\" seems to be already registered.",
					to, tmp_pose.asString().c_str()));
		this->m_graph->insertEdgeAtEnd(from, to, constraint);
	}

	m_prev_registered_nodeID = to;

	MRPT_LOG_DEBUG_STREAM << "Registered new node:" << endl <<
		"\t" << from << " => " << to << endl <<
		"\tEdge: " << constraint.getMeanVal().asString();

	return true;
	MRPT_END;
}

template<class GRAPH_T>
void CNodeRegistrationDecider<GRAPH_T>::addNodeAnnotsToPose(
		global_pose_t* pose) const { }

#endif /* end of include guard: CNODEREGISTRATIONDECIDER_IMPL_H */
