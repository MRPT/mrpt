/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once
#include <cstdint>
#include <utility>	// pair

namespace mrpt::graphs
{
/** A generic numeric type for unique IDs of nodes or entities */
using TNodeID = uint64_t;

/** A pair of node IDs */
using TPairNodeIDs = std::pair<TNodeID, TNodeID>;

constexpr static mrpt::graphs::TNodeID INVALID_NODEID =
	static_cast<mrpt::graphs::TNodeID>(-1);

}  // namespace mrpt::graphs
