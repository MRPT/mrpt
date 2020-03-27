/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config.h>
#include <mrpt/tfest/TMatchingPair.h>

namespace mrpt::tfest::internal
{
template <typename T = float>
struct se2_l2_impl_return_t
{
	T mean_x_a, mean_y_a, mean_x_b, mean_y_b;
	T Ax, Ay;
};

#if MRPT_ARCH_INTEL_COMPATIBLE
extern se2_l2_impl_return_t<float> se2_l2_impl_SSE2(
	const TMatchingPairList& in_correspondences);
#endif

}  // namespace mrpt::tfest::internal
