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
