/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once
#include <cstdint>
#include <utility>  // pair

namespace mrpt::graphs
{
/** A generic numeric type for unique IDs of nodes or entities */
using TNodeID = uint64_t;
/** A pair of node IDs */
using TPairNodeIDs = std::pair<TNodeID, TNodeID>;
#define INVALID_NODEID static_cast<mrpt::graphs::TNodeID>(-1)
}  // namespace mrpt::graphs
