/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <utility>  // std::swap
#include <iterator>  // iterator_traits
#include <random>  // uniform_int_distribution

namespace mrpt
{
namespace random
{
/** Uniform shuffle a sequence.
 *\ingroup mrpt_random_grp
 */
template <class RandomIt, class URBG>
void shuffle(RandomIt first, RandomIt last, URBG&& g)
{
	typedef typename std::iterator_traits<RandomIt>::difference_type diff_t;
	typedef std::uniform_int_distribution<diff_t> distr_t;
	typedef typename distr_t::param_type param_t;
	distr_t D;
	diff_t n = last - first;
	for (diff_t i = n - 1; i > 0; --i)
		std::swap(first[i], first[D(g, param_t(0, i))]);
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

}  // namespace random
}  // namespace mrpt
