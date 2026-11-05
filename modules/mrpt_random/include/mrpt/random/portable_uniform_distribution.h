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

namespace mrpt::random
{
/** A compiler and platform-independent uniform distribution generator.
 *  Returns an integer in the closed range [min, max], built from two
 *  consecutive 32-bit outputs of the generator (higher bits first).
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

  // Two separate statements, so the call order does not depend on the
  // compiler:
  const auto hi = static_cast<uint64_t>(static_cast<uint32_t>(g()));
  const auto lo = static_cast<uint64_t>(static_cast<uint32_t>(g()));
  const uint64_t sample = (hi << 32) | lo;
  const uint64_t span = max - min + 1;  // 0 means the full 64-bit range
  return span == 0 ? sample : min + (sample % span);
}

}  // namespace mrpt::random
