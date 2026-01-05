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

#include <mrpt/random/portable_uniform_distribution.h>

#include <cstdint>
#include <random>   // uniform_int_distribution
#include <utility>  // std::swap

namespace mrpt::random
{
/** Uniform shuffle a sequence.
 *\ingroup mrpt_random_grp
 */
template <class RandomIt, class URBG>
void shuffle(RandomIt first, RandomIt last, URBG&& g)
{
  const auto n = static_cast<int64_t>(last - first);
  if (n == 0)
  {
    return;
  }
  for (int64_t i = static_cast<int64_t>(n) - 1; i > 0; --i)
  {
    const auto idx = portable_uniform_distribution(g, 0, static_cast<uint64_t>(i));
    std::swap(first[i], first[static_cast<typename RandomIt::difference_type>(idx)]);
  }
}

/** Uniform shuffle a sequence.
 *\ingroup mrpt_random_grp
 */
template <class RandomIt>
void shuffle(RandomIt first, RandomIt last)
{
  std::random_device rd;  // used for random seed
  std::mt19937 g(rd());
  mrpt::random::shuffle(first, last, g);
}

/** Shuffle the first N elements of a sequence. Note that elements at positions
 *  [N:end] may also change if they are randomly picked for permutation with the
 *  first [0,N-1] elements.
 * \ingroup mrpt_random_grp
 * \note [New in MRPT 2.4.0]
 */
template <class RandomIt, class URBG>
void partial_shuffle(RandomIt first, RandomIt last, URBG&& g, size_t N)
{
  const uint64_t n = static_cast<uint64_t>(last - first);
  if (n == 0 || N == 0)
  {
    return;
  }
  const uint64_t n_1 = n - 1;
  for (uint64_t i = 0; i < n && i < N; ++i)
  {
    const auto idx = portable_uniform_distribution(g, i, n_1);
    std::swap(
        first[static_cast<typename RandomIt::difference_type>(i)],
        first[static_cast<typename RandomIt::difference_type>(idx)]);
  }
}

}  // namespace mrpt::random
