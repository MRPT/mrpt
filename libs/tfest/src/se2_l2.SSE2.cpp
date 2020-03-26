/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "tfest-precomp.h"  // Precompiled headers

#include <mrpt/config.h>
#include "se2_l2_internal.h"

#if MRPT_ARCH_INTEL_COMPATIBLE

#include <mrpt/core/SSE_types.h>
#include <mrpt/core/exceptions.h>

using namespace mrpt::tfest;

internal::se2_l2_impl_return_t<float> internal::se2_l2_impl_SSE2(
	const TMatchingPairList& in_correspondences)
{
	// SSE vectorized version:
	const size_t N = in_correspondences.size();
	ASSERT_(N >= 2);
	const float N_inv = 1.0f / N;  // For efficiency, keep this value.

	// Ensure correct types:
	static_assert(sizeof(TMatchingPair::this_x) == sizeof(float));
	static_assert(sizeof(TMatchingPair::other_x) == sizeof(float));

	internal::se2_l2_impl_return_t<float> ret;

	__m128 sum_a_xyz = _mm_setzero_ps();  // All 4 zeros (0.0f)
	__m128 sum_b_xyz = _mm_setzero_ps();  // All 4 zeros (0.0f)

	//   [ f0     f1      f2      f3  ]
	//    xa*xb  ya*yb   xa*yb  xb*ya
	__m128 sum_ab_xyz = _mm_setzero_ps();  // All 4 zeros (0.0f)

	for (const auto& in_correspondence : in_correspondences)
	{
		// Get the pair of points in the correspondence:
		//   a_xyyx = [   xa     ay   |   xa    ya ]
		//   b_xyyx = [   xb     yb   |   yb    xb ]
		//      (product)
		//            [  xa*xb  ya*yb   xa*yb  xb*ya
		//                LO0    LO1     HI2    HI3
		// Note: _MM_SHUFFLE(hi3,hi2,lo1,lo0)
		const __m128 a_xyz =
			_mm_loadu_ps(&in_correspondence.this_x);  // *Unaligned* load
		const __m128 b_xyz =
			_mm_loadu_ps(&in_correspondence.other_x);  // *Unaligned* load

		const auto a_xyxy =
			_mm_shuffle_ps(a_xyz, a_xyz, _MM_SHUFFLE(1, 0, 1, 0));
		const auto b_xyyx =
			_mm_shuffle_ps(b_xyz, b_xyz, _MM_SHUFFLE(0, 1, 1, 0));

		// Compute the terms:
		sum_a_xyz = _mm_add_ps(sum_a_xyz, a_xyz);
		sum_b_xyz = _mm_add_ps(sum_b_xyz, b_xyz);

		//   [ f0     f1      f2      f3  ]
		//    xa*xb  ya*yb   xa*yb  xb*ya
		sum_ab_xyz = _mm_add_ps(sum_ab_xyz, _mm_mul_ps(a_xyxy, b_xyyx));
	}

	alignas(MRPT_MAX_STATIC_ALIGN_BYTES) float sums_a[4], sums_b[4];
	_mm_store_ps(sums_a, sum_a_xyz);
	_mm_store_ps(sums_b, sum_b_xyz);

	float SumXa = sums_a[0];
	float SumYa = sums_a[1];
	float SumXb = sums_b[0];
	float SumYb = sums_b[1];

	// Compute all four means:
	const __m128 Ninv_4val =
		_mm_set1_ps(N_inv);  // load 4 copies of the same value
	sum_a_xyz = _mm_mul_ps(sum_a_xyz, Ninv_4val);
	sum_b_xyz = _mm_mul_ps(sum_b_xyz, Ninv_4val);

	// means_a[0]: mean_x_a
	// means_a[1]: mean_y_a
	// means_b[0]: mean_x_b
	// means_b[1]: mean_y_b
	alignas(MRPT_MAX_STATIC_ALIGN_BYTES) float means_a[4], means_b[4];
	_mm_store_ps(means_a, sum_a_xyz);
	_mm_store_ps(means_b, sum_b_xyz);

	ret.mean_x_a = means_a[0];
	ret.mean_y_a = means_a[1];
	ret.mean_x_b = means_b[0];
	ret.mean_y_b = means_b[1];

	//      Sxx   Syy     Sxy    Syx
	//    xa*xb  ya*yb   xa*yb  xb*ya
	alignas(MRPT_MAX_STATIC_ALIGN_BYTES) float cross_sums[4];
	_mm_store_ps(cross_sums, sum_ab_xyz);

	float Sxx = cross_sums[0];
	float Syy = cross_sums[1];
	float Sxy = cross_sums[2];
	float Syx = cross_sums[3];

	// Auxiliary variables Ax,Ay:
	ret.Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
	ret.Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;

	return ret;
}

#endif  // MRPT_ARCH_INTEL_COMPATIBLE
