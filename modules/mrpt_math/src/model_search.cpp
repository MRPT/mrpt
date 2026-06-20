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

#include <mrpt/math/model_search.h>
#include <mrpt/random/random_shuffle.h>

using namespace mrpt;
using namespace mrpt::math;

//----------------------------------------------------------------------
//! Select random (unique) indices from the 0..p_size sequence
void ModelSearch::pickRandomIndex(size_t p_size, size_t p_pick, std::vector<size_t>& p_ind)
{
  ASSERT_(p_size >= p_pick);

  std::vector<size_t> a(p_size);
  for (size_t i = 0; i < p_size; i++) a[i] = i;

  mrpt::random::shuffle(a.begin(), a.end());
  p_ind.resize(p_pick);
  for (size_t i = 0; i < p_pick; i++) p_ind[i] = a[i];
}

//----------------------------------------------------------------------
//! Select random (unique) indices from the set.
//! The set is destroyed during pick
void ModelSearch::pickRandomIndex(std::set<size_t> p_set, size_t p_pick, std::vector<size_t>& p_ind)
{
  p_ind.resize(p_pick);
  std::vector<size_t> inds(p_set.begin(), p_set.end());

  mrpt::random::shuffle(inds.begin(), inds.end());
  p_ind.resize(p_pick);
  for (size_t i = 0; i < p_pick; i++) p_ind[i] = inds[i];
}
