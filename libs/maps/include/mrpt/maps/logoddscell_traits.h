/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdint>

namespace mrpt::maps::detail
{
template <typename TCELL>
struct logoddscell_traits;
// Specializations:
template <>
struct logoddscell_traits<int8_t>
{
	static constexpr int8_t CELLTYPE_MIN = -127;
	static constexpr int8_t CELLTYPE_MAX = 127;
	static constexpr int8_t P2LTABLE_SIZE = CELLTYPE_MAX;
	static constexpr std::size_t LOGODDS_LUT_ENTRIES = 1 << 8;
};
template <>
struct logoddscell_traits<int16_t>
{
	static constexpr int16_t CELLTYPE_MIN = -32767;
	static constexpr int16_t CELLTYPE_MAX = 32767;
	static constexpr int16_t P2LTABLE_SIZE = CELLTYPE_MAX;
	static constexpr std::size_t LOGODDS_LUT_ENTRIES = 1 << 16;
};
}  // namespace mrpt::maps::detail
