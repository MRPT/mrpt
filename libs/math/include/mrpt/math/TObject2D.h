/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/TSegment2D.h>

#include <variant>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/** A variant type for any lightweight 2D type: point, segment, line, polygon.
 * Use provided helper method, or directly access the variant `data`.
 *
 * \sa TPoint2D,TSegment2D,TLine2D,TPolygon2D
 */
struct TObject2D
{
	using variant_t =
		std::variant<std::monostate, TPoint2D, TSegment2D, TLine2D, TPolygon2D>;

	variant_t data;

	TObject2D() = default;
	~TObject2D() = default;

	/** Constructor from point, segment, line, polygon. */
	template <typename T>
	static TObject2D From(const T& p)
	{
		TObject2D o;
		o.data = p;
		return o;
	}

	/** Checks whether content is a point. */
	bool isPoint() const { return std::holds_alternative<TPoint2D>(data); }

	/**Checks whether content is a segment. */
	bool isSegment() const { return std::holds_alternative<TSegment2D>(data); }
	/**
	 * Checks whether content is a line.
	 */
	bool isLine() const { return std::holds_alternative<TLine2D>(data); }
	/**
	 * Checks whether content is a polygon.
	 */
	bool isPolygon() const { return std::holds_alternative<TPolygon2D>(data); }

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
	bool getPoint(TPoint2D& out) const
	{
		if (!isPoint()) return false;
		out = getAs<TPoint2D>();
		return true;
	}
	/** returns true if the objects is a segment, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getSegment(TSegment2D& out) const
	{
		if (!isSegment()) return false;
		out = getAs<TSegment2D>();
		return true;
	}
	/** returns true if the objects is a line, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getLine(TLine2D& out) const
	{
		if (!isLine()) return false;
		out = getAs<TLine2D>();
		return true;
	}
	/** returns true if the objects is a TPolygon2D, and retrieves its value in
	 * `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards
	 * compatibility.
	 */
	bool getPolygon(TPolygon2D& out) const
	{
		if (!isPolygon()) return false;
		out = getAs<TPolygon2D>();
		return true;
	}

	/** Gets a string with the type and the parameters of the object. `empty` if
	 * not defined. \note New in MRPT 2.3.0 */
	std::string asString() const;

	/** Cast into 3D space. */
	TObject3D generate3DObject() const;
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject2D& o);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject2D& o);

/** Textual print stream operator. \sa TObject2D::asString() */
inline std::ostream& operator<<(
	std::ostream& o, const mrpt::math::TObject2D& obj)
{
	o << obj.asString();
	return o;
}

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TObject2D, mrpt::math)
}  // namespace mrpt::typemeta
