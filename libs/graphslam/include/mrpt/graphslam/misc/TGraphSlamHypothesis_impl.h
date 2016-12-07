#ifndef TGRAPHSLAMHYPOTHESIS_IMPL_H
#define TGRAPHSLAMHYPOTHESIS_IMPL_H

// implementation file for the TGraphSlamHypothesis class template

namespace mrpt { namespace graphslam { namespace detail {

template<class GRAPH_t>
TGraphSlamHypothesis<GRAPH_t>::TGraphSlamHypothesis():
	is_valid(true),
	meas_source(mrpt::graphslam::detail::SSOM_Laser) {}

template<class GRAPH_t>
TGraphSlamHypothesis<GRAPH_t>::~TGraphSlamHypothesis() { }


template<class GRAPH_t>
std::string TGraphSlamHypothesis<GRAPH_t>::getAsString(
		bool oneline/*=true*/) const {
	std::string str;
	this->getAsString(&str, oneline);
	return str;
}

template<class GRAPH_t>
void TGraphSlamHypothesis<GRAPH_t>::getAsString(
		std::string* str,
		bool oneline/*=true*/) const {
	ASSERTMSG_(str, "Given string pointer is not valid");

	using namespace std;

	stringstream ss;
	if (!oneline) { // multiline report
		ss << "Hypothesis #" << id << endl;
		ss << from << " => " << to << endl;
		ss << edge << endl;
	}
	else {
		ss << "Hypothesis #" << id << "|\t ";
		ss << from << " => " << to << "|\t ";
		ss << edge.getMeanVal().asString(); 
		ss << "|\tgoodness: " << goodness;
		ss << "|\tvalid: " << is_valid;
	}

	*str = ss.str();
}


} } } // end of namespaces

#endif /* end of include guard: TGRAPHSLAMHYPOTHESIS_IMPL_H */
