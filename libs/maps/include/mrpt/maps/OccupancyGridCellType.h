/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <mrpt/config.h>
#if (                                                \
	!defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) &&   \
	!defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)) || \
	(defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) &&   \
	 defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS))
#error One of OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS or OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS must be defined.
#endif

namespace mrpt::maps
{
/** The type of the map cells. \ingroup mrpt_maps_grp */
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
struct OccGridCellTraits
{
	using cellType = int8_t;
	using cellTypeUnsigned = uint8_t;
};
#else
struct OccGridCellTraits
{
	using cellType = int16_t;
	using cellTypeUnsigned = uint16_t;
};
#endif
}  // namespace mrpt::maps
