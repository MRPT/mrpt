/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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
	enum class CTOR_FLAGS
	{
		None = 0,
		AllowUnordered
	};

	TBoundingBox_() = default;

	/** Ctor from min-max corners.
	 * A bounding box may have a zero volume if max==min.
	 * It is ilegal for a coordinate of the `max` vector to be smaller than its
	 * `min` counterpart, in which case an exception will be thrown, except if
	 * the flag CTOR_FLAGS::AllowUnordered is passed.
	 */
	TBoundingBox_(
		const mrpt::math::TPoint3D_<T>& Min,
		const mrpt::math::TPoint3D_<T>& Max,
		const CTOR_FLAGS f = CTOR_FLAGS::None)
		: min(Min), max(Max)
	{
		if (f != CTOR_FLAGS::AllowUnordered)
			ASSERT_(max.x >= min.x && max.y >= min.y && max.z >= min.z);
	}

	/** The corners of the bounding box */
	mrpt::math::TPoint3D_<T> min, max;

	/** Initialize with min=+Infinity, max=-Infinity. This is useful as an
	 * initial value before processing a list of points to keep their
	 * minimum/maximum. */
	static TBoundingBox_<T> PlusMinusInfinity()
	{
		const T i = std::numeric_limits<T>::max();
		return {{i, i, i}, {-i, -i, -i}, CTOR_FLAGS::AllowUnordered};
	}

	/** Construct a bounding box from two points, by selecting their minimum and
	 * maximum (x,y,z) coordinates, so the point coordinates do not need to be
	 * already sorted by the user as one being the minimum and maximum corners.
	 * \note (New in MRPT 2.5.6)
	 */
	static TBoundingBox_<T> FromUnsortedPoints(
		const mrpt::math::TPoint3D_<T>& pt1,
		const mrpt::math::TPoint3D_<T>& pt2)
	{
		TBoundingBox_<T> r;
		r.min.x = std::min(pt1.x, pt2.x);
		r.min.y = std::min(pt1.y, pt2.y);
		r.min.z = std::min(pt1.z, pt2.z);
		r.max.x = std::max(pt1.x, pt2.x);
		r.max.y = std::max(pt1.y, pt2.y);
		r.max.z = std::max(pt1.z, pt2.z);
		return r;
	}

	/** Returns the volume of the box */
	T volume() const
	{
		return (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
	}

	/** Returns the intersection of this bounding box with "b", or std::nullopt
	 * if no intersection exists.
	 * Note that borders are enlarged by "epsilon" before to testing for
	 * intersection to handle numerical innacuracies, for example on planar
	 * bounding boxes with a fixed "z".
	 */
	std::optional<TBoundingBox_<T>> intersection(
		const TBoundingBox_<T>& b, const T epsilon = static_cast<T>(1e-4)) const
	{
		if (b.min.x - epsilon > max.x || b.min.y - epsilon > max.y ||
			b.min.z - epsilon > max.z || b.max.x + epsilon < min.x ||
			b.max.y + epsilon < min.y || b.max.z + epsilon < min.z)
			return {};

		return {TBoundingBox_<T>(
			{std::max(min.x, b.min.x), std::max(min.y, b.min.y),
			 std::max(min.z, b.min.z)},
			{std::min(max.x, b.max.x), std::min(max.y, b.max.y),
			 std::min(max.z, b.max.z)},
			CTOR_FLAGS::AllowUnordered)};
	}

	/** Returns the union of this bounding box with "b", i.e. a new bounding box
	 * comprising both `this` and `b` */
	TBoundingBox_<T> unionWith(const TBoundingBox_<T>& b) const
	{
		return {TBoundingBox_<T>(
			{std::min(min.x, b.min.x), std::min(min.y, b.min.y),
			 std::min(min.z, b.min.z)},
			{std::max(max.x, b.max.x), std::max(max.y, b.max.y),
			 std::max(max.z, b.max.z)})};
	}

	/** Expands the box limits to include the given point */
	void updateWithPoint(const mrpt::math::TPoint3D_<T>& p)
	{
		mrpt::keep_min(min.x, p.x);
		mrpt::keep_min(min.y, p.y);
		mrpt::keep_min(min.z, p.z);

		mrpt::keep_max(max.x, p.x);
		mrpt::keep_max(max.y, p.y);
		mrpt::keep_max(max.z, p.z);
	}

	/** Returns true if the point lies within the bounding box (including the
	 * exact border)
	 * \note (New in MRPT 2.3.3)
	 */
	bool containsPoint(const mrpt::math::TPoint3D_<T>& p) const
	{
		return p.x >= min.x && p.y >= min.y && p.z >= min.z && p.x <= max.x &&
			p.y <= max.y && p.z <= max.z;
	}

	/** Returns a new bounding box, transforming `this` from local coordinates
	 * to global coordinates, as if `this` was given with respect to `pose`, ie:
	 *
	 * \code
	 *  return.min = pose \oplus this->min
	 *  return.max = pose \oplus this->max
	 * \endcode
	 * \tparam POSE_T Can be mrpt::poses::CPose3D, or mrpt::math::TPose3D
	 *
	 * \note If a rotation exists, the output bounding box will no longer be an
	 * accurate representation of the actual 3D box.
	 */
	template <typename POSE_T>
	TBoundingBox_<T> compose(const POSE_T& pose) const
	{
		return FromUnsortedPoints(
			pose.composePoint(min), pose.composePoint(max));
	}

	/** Returns a new bounding box, transforming `this` from global coordinates
	 * to local coordinates with respect to `pose`, ie:
	 *
	 * \code
	 *  return.min = this->min \ominus pose
	 *  return.max = this->max \ominus pose
	 * \endcode
	 * \tparam POSE_T Can be mrpt::poses::CPose3D, or mrpt::math::TPose3D
	 *
	 * \note If a rotation exists, the output bounding box will no longer be an
	 * accurate representation of the actual 3D box.
	 */
	template <typename POSE_T>
	TBoundingBox_<T> inverseCompose(const POSE_T& pose) const
	{
		return FromUnsortedPoints(
			pose.inverseComposePoint(min), pose.inverseComposePoint(max));
	}

	/** Print bounding box as a string with format
	 * "(minx,miny,minz)-(maxx,maxy,maxz)"
	 *
	 * \note Do not inherit from mrpt::Stringifyable to avoid virtual class
	 * table and keeping the class trivially-copiable.
	 */
	std::string asString() const
	{
		std::string s = min.asString();
		s += "-";
		s += max.asString();
		return s;
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
