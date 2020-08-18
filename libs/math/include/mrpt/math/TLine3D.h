/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <array>

namespace mrpt::math
{
/** 3D line, represented by a base point and a director vector.
 * \sa TLine2D,TSegment3D,TPlane,TPolygon3D,TPoint3D
 */
struct TLine3D
{
   public:
	/** Fast default constructor. Initializes to all zeros. */
	TLine3D() = default;

	/** Constructor from two points, through which the line will pass.
	 * \throw std::logic_error if both points are the same.
	 */
	TLine3D(const TPoint3D& p1, const TPoint3D& p2);
	/** Constructor from 3D segment  */
	explicit TLine3D(const TSegment3D& s);

	/** Constructor from 2D object. Zeroes the z. */
	explicit TLine3D(const TLine2D& l);

	/** Static constructor from a point and a director vector.
	 * \note [New in MRPT 2.0.4]
	 */
	static TLine3D FromPointAndDirector(
		const TPoint3D& basePoint, const TVector3D& directorVector);

	/** Static constructor from two points.
	 * \note [New in MRPT 2.0.4]
	 */
	static TLine3D FromTwoPoints(const TPoint3D& p1, const TPoint3D& p2);

	/** Base point */
	TPoint3D pBase;
	/** Director vector */
	TVector3D director{.0, .0, .0};
	/** Check whether a point is inside the line */
	bool contains(const TPoint3D& point) const;
	/**
	 * Distance between the line and a point.
	 */
	double distance(const TPoint3D& point) const;
	/**
	 * Unitarize director vector.
	 */
	void unitarize();
	/** Get director vector */
	void getDirectorVector(double (&vector)[3]) const
	{
		for (size_t i = 0; i < 3; i++) vector[i] = director[i];
	}
	/** Get director vector (may be NOT unitary if not set so by the user) \sa
	 * getUnitaryDirectorVector(), unitarize() */
	inline const TVector3D& getDirectorVector() const { return director; }

	/**
	 * Unitarize and then get director vector.
	 */
	void getUnitaryDirectorVector(double (&vector)[3])
	{
		unitarize();
		getDirectorVector(vector);
	}
	/**
	 * Project into 2D space, discarding the Z coordinate.
	 * \throw std::logic_error if the line's director vector is orthogonal to
	 * the XY plane.
	 */
	void generate2DObject(TLine2D& l) const;

	/** Returns "P=[x,y,z] u=[ux,uy,uz]"
	 * \note [New in MRPT 2.1.0]
	 */
	std::string asString() const;
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TLine3D& l);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TLine3D& l);

/** Text streaming function */
inline std::ostream& operator<<(std::ostream& o, const TLine3D& p)
{
	o << p.asString();
	return o;
}

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TLine3D, mrpt::math)
}  // namespace mrpt::typemeta
