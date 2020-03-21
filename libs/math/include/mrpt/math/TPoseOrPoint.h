/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>  // Used in all derived classes
#include <iosfwd>  // std::ostream
#include <stdexcept>
#include <type_traits>

namespace mrpt::math
{
/** Base type of all TPoseXX and TPointXX classes in mrpt::math.
 * Useful for type traits. No virtual methods at all.
 * \ingroup geometry_grp
 */
struct TPoseOrPoint
{
};

/** Forward declarations of all mrpt::math classes related to poses and points
 */
template <typename T>
struct TPoint2D_;
using TPoint2D = TPoint2D_<double>;
using TPoint2Df = TPoint2D_<float>;

template <typename T>
struct TPoint3D_;
using TPoint3D = TPoint3D_<double>;
using TPoint3Df = TPoint3D_<float>;

struct TPose2D;
struct TPose3D;
struct TPose3DQuat;
struct TPoseOrPoint;
struct TTwist2D;
struct TTwist3D;
template <class T>
class CQuaternion;

struct TSegment2D;
struct TSegment3D;
struct TLine2D;
struct TLine3D;
class TPolygon3D;
class TPolygon2D;
struct TObject3D;
struct TObject2D;
struct TPlane;

namespace internal
{
/** Provided for STL and matrices/vectors compatibility */
template <typename Derived>
struct ProvideStaticResize
{
	constexpr std::size_t rows() const { return Derived::static_size; }
	constexpr std::size_t cols() const { return 1; }
	constexpr std::size_t size() const { return Derived::static_size; }

	/** throws if attempted to resize to incorrect length */
	void resize(std::size_t n) { ASSERT_EQUAL_(n, Derived::static_size); }
};
}  // namespace internal

/** Text streaming function */
template <
	class PoseOrPoint, typename = std::enable_if_t<std::is_base_of_v<
						   mrpt::math::TPoseOrPoint, PoseOrPoint>>>
std::ostream& operator<<(std::ostream& o, const PoseOrPoint& p)
{
	o << p.asString();
	return o;
}

/** Binary streaming function */
template <
	class PoseOrPoint, typename = std::enable_if_t<std::is_base_of_v<
						   mrpt::math::TPoseOrPoint, PoseOrPoint>>>
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, PoseOrPoint& o)
{
	for (int i = 0; i < o.static_size; i++) in >> o[i];
	return in;
}

/** Binary streaming function */
template <
	class PoseOrPoint, typename = std::enable_if_t<std::is_base_of_v<
						   mrpt::math::TPoseOrPoint, PoseOrPoint>>>
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const PoseOrPoint& o)
{
	for (int i = 0; i < o.static_size; i++) out << o[i];
	return out;
}

/**
 * Object type identifier for TPoint2D or TPoint3D.
 * \sa TObject2D,TObject3D
 */
static constexpr unsigned char GEOMETRIC_TYPE_POINT = 0;
/**
 * Object type identifier for TSegment2D or TSegment3D.
 * \sa TObject2D,TObject3D
 */
static constexpr unsigned char GEOMETRIC_TYPE_SEGMENT = 1;
/**
 * Object type identifier for TLine2D or TLine3D.
 * \sa TObject2D,TObject3D
 */
static constexpr unsigned char GEOMETRIC_TYPE_LINE = 2;
/**
 * Object type identifier for TPolygon2D or TPolygon3D.
 * \sa TObject2D,TObject3D
 */
static constexpr unsigned char GEOMETRIC_TYPE_POLYGON = 3;
/**
 * Object type identifier for TPlane.
 * \sa TObject3D
 */
static constexpr unsigned char GEOMETRIC_TYPE_PLANE = 4;
/**
 * Object type identifier for empty TObject2D or TObject3D.
 * \sa TObject2D,TObject3D
 */
static constexpr unsigned char GEOMETRIC_TYPE_UNDEFINED = 255;

}  // namespace mrpt::math
