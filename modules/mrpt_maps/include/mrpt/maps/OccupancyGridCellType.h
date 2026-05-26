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

namespace mrpt::maps
{
/** The type of the map cells.
 * Cell size is always 8 bits (int8_t). The 16-bit compile-time switch was
 * removed in MRPT 3.x; historical 16-bit serialized streams can still be read.
 * \ingroup mrpt_maps_grp */
struct OccGridCellTraits
{
  using cellType = int8_t;
  using cellTypeUnsigned = uint8_t;
};
}  // namespace mrpt::maps
