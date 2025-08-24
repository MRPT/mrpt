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

#include <mrpt/core/config.h>  // MRPT_ARCH_INTEL_COMPATIBLE
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
extern se2_l2_impl_return_t<float> se2_l2_impl_SSE2(const TMatchingPairList& in_correspondences);
#endif

}  // namespace mrpt::tfest::internal
