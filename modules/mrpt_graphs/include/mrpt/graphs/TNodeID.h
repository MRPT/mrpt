/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include <cstdint>
#include <utility>  // pair

namespace mrpt::graphs
{
/** A generic numeric type for unique IDs of nodes or entities */
using TNodeID = uint64_t;

/** A pair of node IDs */
using TPairNodeIDs = std::pair<TNodeID, TNodeID>;

constexpr static mrpt::graphs::TNodeID INVALID_NODEID = static_cast<mrpt::graphs::TNodeID>(-1);

}  // namespace mrpt::graphs
