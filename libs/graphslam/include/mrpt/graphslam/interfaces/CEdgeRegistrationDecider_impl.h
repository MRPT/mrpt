#ifndef CEDGEREGISTRATIONDECIDER_IMPL_H
#define CEDGEREGISTRATIONDECIDER_IMPL_H

using namespace mrpt::graphslam::deciders;
using namespace std;

#include <sstream>

// Implementation of classes defined in the CNodeRegistrationDecider class
// template.
//

template<class GRAPH_T>
CEdgeRegistrationDecider<GRAPH_T>::CEdgeRegistrationDecider():
	m_just_inserted_lc(false),
 	m_override_registered_nodes_check(false)
{ }

template<class GRAPH_T>
CEdgeRegistrationDecider<GRAPH_T>::~CEdgeRegistrationDecider() { }

template<class GRAPH_T>
void CEdgeRegistrationDecider<GRAPH_T>::getDescriptiveReport(std::string* report_str) const {
	stringstream ss("");
	parent::getDescriptiveReport(report_str);

	ss << "Edge Registration Decider Strategy [ERD]: " << endl;
	*report_str += ss.str();
}

template<class GRAPH_T>
void CEdgeRegistrationDecider<GRAPH_T>::registerNewEdge(
    const mrpt::utils::TNodeID& from,
    const mrpt::utils::TNodeID& to,
    const constraint_t& rel_edge) {
  using namespace std;

  MRPT_LOG_DEBUG_STREAM( "Registering new edge: " << from << " => "
		<< to << endl << "\tRelative Edge: " << rel_edge.getMeanVal().asString()
		<< "\tNorm: " << rel_edge.getMeanVal().norm());

}



#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_IMPL_H */
