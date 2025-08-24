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

#include <mrpt/maps/config.h>

#include <cstdint>
#if (                                                \
    !defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) &&   \
    !defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)) || \
    (defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) && defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS))
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
