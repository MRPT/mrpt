/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>
#include <algorithm>  // max()
#include <optional>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/** A bounding box defined by the 3D points at each minimum-maximum corner.
 */
template <typename T>
struct TBoundingBox_
{
	TBoundingBox_() = default;

	/** Ctor from min-max corners.
	 * A bounding box may have a zero volume if max==min.
	 * It is ilegal for a coordinate of the `max` vector to be smaller than its
	 * `min` counterpart, in which case an exception will be thrown.
	 */
	TBoundingBox_(
		const mrpt::math::TPoint3D_<T>& Min,
		const mrpt::math::TPoint3D_<T>& Max)
		: min(Min), max(Max)
	{
		ASSERT_(max.x >= min.x && max.y >= min.y && max.z >= min.z);
	}

	/** The corners of the bounding box */
	mrpt::math::TPoint3D_<T> min, max;

	/** Returns the volume of the box */
	T volume() const
	{
		return (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
	}

	/** Returns the intersection of this bounding box with "b", or std::nullopt
	 * if no intersection exists. */
	std::optional<TBoundingBox_<T>> intersection(
		const TBoundingBox_<T>& b) const
	{
		if (b.min.x > max.x || b.min.y > max.y || b.min.z > max.z ||
			b.max.x < min.x || b.max.y < min.y || b.max.z < min.z)
			return {};

		return {TBoundingBox_<T>(
			{std::max(min.x, b.min.x), std::max(min.y, b.min.y),
			 std::max(min.z, b.min.z)},
			{std::min(max.x, b.max.x), std::min(max.y, b.max.y),
			 std::min(max.z, b.max.z)})};
	}
};

/** A bounding box defined by the 3D points at each minimum-maximum corner.
 * \sa mrpt::math::TPoint3D, mrpt::math::TPoint3Df
 */
using TBoundingBox = TBoundingBox_<double>;
using TBoundingBoxf = TBoundingBox_<float>;

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TBoundingBoxf& bb);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TBoundingBoxf& bb);

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TBoundingBox& bb);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TBoundingBox& bb);

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TBoundingBox, mrpt::math)
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TBoundingBoxf, mrpt::math)
}  // namespace mrpt::typemeta
