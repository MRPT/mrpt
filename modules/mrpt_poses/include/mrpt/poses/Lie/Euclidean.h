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

#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt::poses::Lie
{
/** Traits for Euclidean R^N space.
 * \ingroup mrpt_poses_lie_grp
 */
template <unsigned int N>
struct Euclidean;

template <unsigned int N>
struct EuclideanBase
{
  constexpr static mrpt::math::matrix_dim_t DOFs = N;
  using tangent_vector = mrpt::math::CVectorFixedDouble<DOFs>;
};

template <>
struct Euclidean<2> : public EuclideanBase<2>
{
  using type = mrpt::poses::CPoint2D;
  using light_type = mrpt::math::TPoint2D;
};

template <>
struct Euclidean<3> : public EuclideanBase<3>
{
  using type = mrpt::poses::CPoint3D;
  using light_type = mrpt::math::TPoint3D;
};

}  // namespace mrpt::poses::Lie
