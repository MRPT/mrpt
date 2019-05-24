/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
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
	/** Base point */
	TPoint3D pBase;
	/** Director vector */
	std::array<double, 3> director{{.0, .0, .0}};
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
	/**
	 * Get director vector.
	 */
	void getDirectorVector(double (&vector)[3]) const
	{
		for (size_t i = 0; i < 3; i++) vector[i] = director[i];
	}
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
	/**
	 * Constructor from two points, through which the line will pass.
	 * \throw std::logic_error if both points are the same.
	 */
	TLine3D(const TPoint3D& p1, const TPoint3D& p2);
	/**
	 * Constructor from 3D segment.
	 */
	explicit TLine3D(const TSegment3D& s);
	/**
	 * Fast default constructor. Initializes to garbage.
	 */
	TLine3D() = default;
	/** Constructor from 2D object. Zeroes the z. */
	explicit TLine3D(const TLine2D& l);
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TLine3D& l);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TLine3D& l);

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TLine3D, mrpt::math)
}  // namespace mrpt::typemeta
