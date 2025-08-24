/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include <sstream>

namespace mrpt::graphslam::deciders
{
template <class GRAPH_T>
void CEdgeRegistrationDecider<GRAPH_T>::getDescriptiveReport(std::string* report_str) const
{
  std::stringstream ss("");
  parent::getDescriptiveReport(report_str);

  ss << "Edge Registration Decider Strategy [ERD]: \n";
  *report_str += ss.str();
}

template <class GRAPH_T>
void CEdgeRegistrationDecider<GRAPH_T>::registerNewEdge(
    const mrpt::graphs::TNodeID& from,
    const mrpt::graphs::TNodeID& to,
    const constraint_t& rel_edge)
{
  MRPT_LOG_DEBUG_STREAM(
      "Registering new edge: " << from << " => " << to << "\n"
                               << "\tRelative Edge: " << rel_edge.getMeanVal().asString()
                               << "\tNorm: " << rel_edge.getMeanVal().norm());
}
}  // namespace mrpt::graphslam::deciders
