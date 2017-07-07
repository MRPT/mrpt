/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef THYPOTHESIS_IMPL_H
#define THYPOTHESIS_IMPL_H

// implementation file for the THypothesis struct template

namespace mrpt { namespace graphs { namespace detail {

template<class GRAPH_T>
THypothesis<GRAPH_T>::THypothesis():
	is_valid(true),
	goodness(0) {}

template<class GRAPH_T>
THypothesis<GRAPH_T>::~THypothesis() { }


template<class GRAPH_T>
std::string THypothesis<GRAPH_T>::getAsString(
		bool oneline/*=true*/) const {
	std::string str;
	this->getAsString(&str, oneline);
	return str;
}

template<class GRAPH_T>
void THypothesis<GRAPH_T>::getAsString(
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
		ss << "Hypothesis #" << id << "| ";
		ss << from << " => " << to << "| ";
		ss << edge.getMeanVal().asString();
		ss << "|goodness: " << goodness;
		ss << "|valid: " << is_valid;
	}

	*str = ss.str();
}

// TODO - test these
template<class GRAPH_T>
bool THypothesis<GRAPH_T>::sameEndsWith(
		const self_t& other) const {
	return (this->from == other.from && this->to == other.to);
}

template<class GRAPH_T>
bool THypothesis<GRAPH_T>::hasEnds(
		mrpt::utils::TNodeID from_in,
		mrpt::utils::TNodeID to_in) const {
	return (this->from == from_in && this->to == to_in);
}

template<class GRAPH_T>
void THypothesis<GRAPH_T>::getEdge(constraint_t* edge) const {
	ASSERT_(edge);
	edge->copyFrom(this->edge);
}

template<class GRAPH_T>
typename GRAPH_T::constraint_t
THypothesis<GRAPH_T>::getEdge() const {

	return this->edge;
}

template<class GRAPH_T>
void THypothesis<GRAPH_T>::setEdge(const constraint_t& edge) {
	this->edge.copyFrom(edge);
}

template<class GRAPH_T>
void THypothesis<GRAPH_T>::getInverseEdge(constraint_t* edge) const {
	ASSERT_(edge);
	this->edge.inverse(*edge);
}

template<class GRAPH_T>
typename GRAPH_T::constraint_t
THypothesis<GRAPH_T>::getInverseEdge() const {
	constraint_t inverse_edge;
	this->getInverseEdge(&inverse_edge);

	return inverse_edge;
}

template<class GRAPH_T>
void THypothesis<GRAPH_T>::inverseHypothesis() {
	// inverse the start/end nodes
	mrpt::utils::TNodeID tmp = from;
	from = to;
	to = tmp;

	// inverse the edge
	constraint_t edge_tmp = this->getInverseEdge();
	this->edge.copyFrom(edge_tmp);
}

template<class GRAPH_T>
bool THypothesis<GRAPH_T>::operator<(const self_t& other) const {
	return this->id < other.id;
}


} } } // end of namespaces

#endif /* end of include guard: THYPOTHESIS_IMPL_H */
