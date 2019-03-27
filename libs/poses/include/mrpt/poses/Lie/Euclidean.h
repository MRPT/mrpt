/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/lightweight_geom_data.h>
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
	constexpr static size_t DOFs = N;
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
