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

#include <cstring>
#include <type_traits>

namespace mrpt::system
{
/** \addtogroup mrpt_memory Memory utilities
 * Header: `#include <mrpt/system/memory.h>`.
 * Library: \ref mrpt_system_grp
 * \ingroup mrpt_system_grp
 *  @{ */

/** Returns the memory occupied by this process, in bytes */
unsigned long getMemoryUsage();

/** @} */

/** \addtogroup mrpt_memory Memory utilities
 *  @{ */
template <std::size_t alignment, typename T, typename = std::enable_if_t<std::is_pointer<T>::value>>
bool is_aligned(T ptr)
{
  return alignment == 0 || reinterpret_cast<std::size_t>(ptr) % alignment == 0;
}
/** @} */

}  // namespace mrpt::system
