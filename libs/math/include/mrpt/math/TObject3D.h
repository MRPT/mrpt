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
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPoseOrPoint.h>

#include <variant>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/** A variant type for any lightweight 3D type: point, segment, line, plane,
 * polygon. Use provided helper method, or directly access the variant `data`.
 *
 * \sa TPoint3D,TSegment3D,TLine3D,TPlane,TPolygon3D
 */
struct TObject3D
{
	using variant_t = std::variant<
		std::monostate, TPoint3D, TSegment3D, TLine3D, TPolygon3D, TPlane>;

	variant_t data;

	TObject3D() = default;
	~TObject3D() = default;

	/** Constructor from point, segment, line, polygon, or plane */
	template <typename T>
	static TObject3D From(const T& p)
	{
		TObject3D o;
		o.data = p;
		return o;
	}

	/** Checks whether content is a point. */
	bool isPoint() const { return std::holds_alternative<TPoint3D>(data); }

	/**Checks whether content is a segment. */
	bool isSegment() const { return std::holds_alternative<TSegment3D>(data); }
	/**
	 * Checks whether content is a line.
	 */
	bool isLine() const { return std::holds_alternative<TLine3D>(data); }
	/**
	 * Checks whether content is a polygon.
	 */
	bool isPolygon() const { return std::holds_alternative<TPolygon3D>(data); }
	/**
	 * Checks whether content is a plane.
	 */
	bool isPlane() const { return std::holds_alternative<TPlane>(data); }

	bool empty() const { return std::holds_alternative<std::monostate>(data); }

	/**
	 *  Gets the content as a given expected type (an exception will be thrown
	 * if type is wrong, check type first).
	 */
	template <typename T>
	const T& getAs() const
	{
		return std::get<T>(data);
	}

	/// \overload
	template <typename T>
	T& getAs()
	{
		return std::get<T>(data);
	}

	/** returns true if the objects is a point, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getPoint(TPoint3D& out) const
	{
		if (!isPoint()) return false;
		out = getAs<TPoint3D>();
		return true;
	}
	/** returns true if the objects is a segment, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getSegment(TSegment3D& out) const
	{
		if (!isSegment()) return false;
		out = getAs<TSegment3D>();
		return true;
	}
	/** returns true if the objects is a line, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getLine(TLine3D& out) const
	{
		if (!isLine()) return false;
		out = getAs<TLine3D>();
		return true;
	}
	/** returns true if the objects is a TPolygon3D, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getPolygon(TPolygon3D& out) const
	{
		if (!isPolygon()) return false;
		out = getAs<TPolygon3D>();
		return true;
	}
	/** returns true if the objects is a TPlane, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getPlane(TPlane& out) const
	{
		if (!isPlane()) return false;
		out = getAs<TPlane>();
		return true;
	}

	/** Gets a string with the type and the parameters of the object. `empty` if
	 * not defined. \note New in MRPT 2.3.0 */
	std::string asString() const;

	/**
	 * Cast into 2D space.
	 * \throw std::logic_error if the 3D object loses its properties when
	 * projecting into 2D space (for example, it's a plane or a vertical line).
	 */
	TObject2D generate2DObject() const;
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject3D& o);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject3D& o);

/** Textual print stream operator. \sa TObject3D::asString() */
inline std::ostream& operator<<(
	std::ostream& o, const mrpt::math::TObject3D& obj)
{
	o << obj.asString();
	return o;
}

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TObject3D, mrpt::math)

}  // namespace mrpt::typemeta
