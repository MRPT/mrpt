/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

// implementation file for the THypothesis struct template

namespace mrpt::graphs::detail
{
template <class GRAPH_T>
THypothesis<GRAPH_T>::THypothesis() = default;

template <class GRAPH_T>
THypothesis<GRAPH_T>::~THypothesis() = default;

template <class GRAPH_T>
std::string THypothesis<GRAPH_T>::getAsString(bool oneline /*=true*/) const
{
	std::string str;
	this->getAsString(&str, oneline);
	return str;
}

template <class GRAPH_T>
void THypothesis<GRAPH_T>::getAsString(
	std::string* str, bool oneline /*=true*/) const
{
	ASSERTMSG_(str, "Given string pointer is not valid");

	using namespace std;

	stringstream ss;
	if (!oneline)
	{  // multiline report
		ss << "Hypothesis #" << id << endl;
		ss << from << " => " << to << endl;
		ss << edge << endl;
	}
	else
	{
		ss << "Hypothesis #" << id << "| ";
		ss << from << " => " << to << "| ";
		ss << edge.getMeanVal().asString();
		ss << "|goodness: " << goodness;
		ss << "|valid: " << is_valid;
	}

	*str = ss.str();
}

// TODO - test these
template <class GRAPH_T>
bool THypothesis<GRAPH_T>::sameEndsWith(const self_t& other) const
{
	return (this->from == other.from && this->to == other.to);
}

template <class GRAPH_T>
bool THypothesis<GRAPH_T>::hasEnds(
	mrpt::graphs::TNodeID from_in, mrpt::graphs::TNodeID to_in) const
{
	return (this->from == from_in && this->to == to_in);
}

template <class GRAPH_T>
void THypothesis<GRAPH_T>::getEdge(constraint_t* out_edge) const
{
	ASSERT_(out_edge);
	out_edge->copyFrom(edge);
}

template <class GRAPH_T>
typename GRAPH_T::constraint_t THypothesis<GRAPH_T>::getEdge() const
{
	return this->edge;
}

template <class GRAPH_T>
void THypothesis<GRAPH_T>::setEdge(const constraint_t& in_edge)
{
	edge.copyFrom(in_edge);
}

template <class GRAPH_T>
void THypothesis<GRAPH_T>::getInverseEdge(constraint_t* out_edge) const
{
	ASSERT_(out_edge);
	edge.inverse(*out_edge);
}

template <class GRAPH_T>
typename GRAPH_T::constraint_t THypothesis<GRAPH_T>::getInverseEdge() const
{
	constraint_t inverse_edge;
	this->getInverseEdge(&inverse_edge);

	return inverse_edge;
}

template <class GRAPH_T>
void THypothesis<GRAPH_T>::inverseHypothesis()
{
	// inverse the start/end nodes
	mrpt::graphs::TNodeID tmp = from;
	from = to;
	to = tmp;

	// inverse the edge
	constraint_t edge_tmp = this->getInverseEdge();
	this->edge.copyFrom(edge_tmp);
}

template <class GRAPH_T>
bool THypothesis<GRAPH_T>::operator<(const self_t& other) const
{
	return this->id < other.id;
}
}  // namespace mrpt::graphs::detail
