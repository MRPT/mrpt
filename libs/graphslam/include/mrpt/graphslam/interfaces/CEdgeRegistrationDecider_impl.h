/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once
#include <sstream>

namespace mrpt::graphslam::deciders
{
template <class GRAPH_T>
void CEdgeRegistrationDecider<GRAPH_T>::getDescriptiveReport(
	std::string* report_str) const
{
	std::stringstream ss("");
	parent::getDescriptiveReport(report_str);

	ss << "Edge Registration Decider Strategy [ERD]: \n";
	*report_str += ss.str();
}

template <class GRAPH_T>
void CEdgeRegistrationDecider<GRAPH_T>::registerNewEdge(
	const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
	const constraint_t& rel_edge)
{
	MRPT_LOG_DEBUG_STREAM(
		"Registering new edge: " << from << " => " << to << "\n"
								 << "\tRelative Edge: "
								 << rel_edge.getMeanVal().asString()
								 << "\tNorm: " << rel_edge.getMeanVal().norm());
}
}  // namespace mrpt::graphslam::deciders
