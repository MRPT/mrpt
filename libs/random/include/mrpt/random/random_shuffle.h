/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/random/portable_uniform_distribution.h>

#include <cstdint>
#include <iterator>	 // iterator_traits
#include <random>  // uniform_int_distribution
#include <utility>	// std::swap

namespace mrpt::random
{
/** Uniform shuffle a sequence.
 *\ingroup mrpt_random_grp
 */
template <class RandomIt, class URBG>
void shuffle(RandomIt first, RandomIt last, URBG&& g)
{
	const uint64_t n = last - first;
	for (int64_t i = static_cast<int64_t>(n) - 1; i > 0; --i)
		std::swap(first[i], first[portable_uniform_distribution(g, 0, i)]);
}

/** Uniform shuffle a sequence.
 *\ingroup mrpt_random_grp
 */
template <class RandomIt>
void shuffle(RandomIt first, RandomIt last)
{
	std::random_device rd;	// used for random seed
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
	const int64_t n = static_cast<int64_t>(last - first);
	const int64_t n_1 = n - 1;
	for (int64_t i = 0; i < n && i < static_cast<int64_t>(N); ++i)
		std::swap(first[i], first[portable_uniform_distribution(g, i, n_1)]);
}

}  // namespace mrpt::random
