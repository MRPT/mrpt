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

namespace mrpt::random
{
/** A compiler and platform-independent uniform distribution generator.
 *  \tparam URGB Can be a C++11 std random generator, or
 *          mrpt::random::Generator_MT19937
 *\ingroup mrpt_random_grp
 */
template <class URBG>
uint64_t portable_uniform_distribution(URBG&& g, uint64_t min, uint64_t max)
{
  if (min == max)
  {
    return min;
  }

  const uint64_t sample = (static_cast<uint64_t>(g()) << 32) | g();
  return min + (sample % (max - min));
}

}  // namespace mrpt::random
