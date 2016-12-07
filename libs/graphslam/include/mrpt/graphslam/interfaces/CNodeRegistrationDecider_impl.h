#ifndef CNODEREGISTRATIONDECIDER_IMPL_H
#define CNODEREGISTRATIONDECIDER_IMPL_H

using namespace mrpt::graphslam::deciders;
using namespace std;

#include <sstream>

// Implementation of classes defined in the CNodeRegistrationDecider class
// template.

template<class GRAPH_t>
void CNodeRegistrationDecider<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	stringstream ss("");
	parent::getDescriptiveReport(report_str);

	ss << "Node Registration Decider Strategy [NRD]: " << endl;
	*report_str += ss.str();
}

template<class GRAPH_t>
bool CNodeRegistrationDecider<GRAPH_t>::checkRegistrationCondition() {
	return false;
}



#endif /* end of include guard: CNODEREGISTRATIONDECIDER_IMPL_H */
