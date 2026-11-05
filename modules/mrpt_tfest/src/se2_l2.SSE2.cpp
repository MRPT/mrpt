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

#include <mrpt/core/config.h>  // MRPT_ARCH_INTEL_COMPATIBLE

#include "se2_l2_internal.h"

#if MRPT_ARCH_INTEL_COMPATIBLE

#include <mrpt/core/SSE_types.h>
#include <mrpt/core/exceptions.h>

using namespace mrpt::tfest;

internal::se2_l2_impl_return_t<double> internal::se2_l2_impl_SSE2(
    const TMatchingPairList& in_correspondences)
{
  // SSE2 vectorized version, with two double-precision lanes. It performs the
  // same operations, in the same order, as the non-vectorized version:
  // both point sets are shifted by a pivot (the first pair) to avoid
  // cancellation errors for points far from the origin.
  const size_t N = in_correspondences.size();
  ASSERT_(N >= 2);
  const double N_inv = 1.0 / static_cast<double>(N);

  internal::se2_l2_impl_return_t<double> ret;

  const auto& p0 = in_correspondences[0];
  // Note: _mm_set_pd(hi, lo)
  const __m128d pivot_a =
      _mm_set_pd(static_cast<double>(p0.global.y), static_cast<double>(p0.global.x));
  const __m128d pivot_b =
      _mm_set_pd(static_cast<double>(p0.local.y), static_cast<double>(p0.local.x));

  __m128d sum_a = _mm_setzero_pd();        // [ SumXa SumYa ]
  __m128d sum_b = _mm_setzero_pd();        // [ SumXb SumYb ]
  __m128d sum_ab = _mm_setzero_pd();       // [ Sxx Syy ]
  __m128d sum_ab_swap = _mm_setzero_pd();  // [ Sxy Syx ]

  for (const auto& c : in_correspondences)
  {
    // a = [ xa ya ], b = [ xb yb ], b_swap = [ yb xb ]
    const __m128d a = _mm_sub_pd(
        _mm_set_pd(static_cast<double>(c.global.y), static_cast<double>(c.global.x)), pivot_a);
    const __m128d b = _mm_sub_pd(
        _mm_set_pd(static_cast<double>(c.local.y), static_cast<double>(c.local.x)), pivot_b);
    const __m128d b_swap = _mm_shuffle_pd(b, b, 0x1);

    sum_a = _mm_add_pd(sum_a, a);
    sum_b = _mm_add_pd(sum_b, b);
    sum_ab = _mm_add_pd(sum_ab, _mm_mul_pd(a, b));
    sum_ab_swap = _mm_add_pd(sum_ab_swap, _mm_mul_pd(a, b_swap));
  }

  alignas(16) double sums_a[2];
  alignas(16) double sums_b[2];
  alignas(16) double sums_ab[2];
  alignas(16) double sums_ab_swap[2];
  _mm_store_pd(sums_a, sum_a);
  _mm_store_pd(sums_b, sum_b);
  _mm_store_pd(sums_ab, sum_ab);
  _mm_store_pd(sums_ab_swap, sum_ab_swap);

  const double SumXa = sums_a[0];
  const double SumYa = sums_a[1];
  const double SumXb = sums_b[0];
  const double SumYb = sums_b[1];
  const double Sxx = sums_ab[0];
  const double Syy = sums_ab[1];
  const double Sxy = sums_ab_swap[0];
  const double Syx = sums_ab_swap[1];

  ret.mean_x_a = static_cast<double>(p0.global.x) + SumXa * N_inv;
  ret.mean_y_a = static_cast<double>(p0.global.y) + SumYa * N_inv;
  ret.mean_x_b = static_cast<double>(p0.local.x) + SumXb * N_inv;
  ret.mean_y_b = static_cast<double>(p0.local.y) + SumYb * N_inv;

  // Auxiliary variables Ax,Ay:
  ret.Ax = static_cast<double>(N) * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
  ret.Ay = SumXa * SumYb + static_cast<double>(N) * (Syx - Sxy) - SumXb * SumYa;

  return ret;
}

#endif  // MRPT_ARCH_INTEL_COMPATIBLE
