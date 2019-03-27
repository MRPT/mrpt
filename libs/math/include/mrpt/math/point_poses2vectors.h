/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstddef>  // size_t
#include <type_traits>  // enable_if_t

namespace mrpt::math
{
struct TPoseOrPoint;

/** \name Container initializer from pose classes
 * @{
 */

/** Conversion of poses to MRPT containers (vector/matrix) */
template <class CONTAINER, class POINT_OR_POSE>
CONTAINER& containerFromPoseOrPoint(CONTAINER& C, const POINT_OR_POSE& p)
{
	const size_t DIMS = POINT_OR_POSE::static_size;
	C.resize(DIMS, 1);
	for (size_t i = 0; i < DIMS; i++)
		C(i, 0) = static_cast<typename CONTAINER::Scalar>(p[i]);
	return C;
}

#define MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(_CLASS_)                          \
	template <                                                                \
		class TPOSE, typename = std::enable_if_t<                             \
						 std::is_base_of_v<mrpt::math::TPoseOrPoint, TPOSE>>> \
	explicit inline _CLASS_(const TPOSE& p)                                   \
	{                                                                         \
		mrpt::math::containerFromPoseOrPoint(*this, p);                       \
	}                                                                         \
	template <class CPOSE, int = CPOSE::is_3D_val>                            \
	explicit inline _CLASS_(const CPOSE& p)                                   \
	{                                                                         \
		mrpt::math::containerFromPoseOrPoint(*this, p);                       \
	}

/** @} */

}  // namespace mrpt::math
