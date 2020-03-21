/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoseOrPoint.h>
#include <array>

namespace mrpt::math
{
/** 2D line without bounds, represented by its equation \f$Ax+By+C=0\f$.
 * \sa TLine3D,TSegment2D,TPolygon2D,TPoint2D
 */
struct TLine2D
{
   public:
	/** Line coefficients, stored as an array: \f$\left[A,B,C\right]\f$ */
	std::array<double, 3> coefs{{0, 0, 0}};
	/**
	 * Evaluate point in the line's equation.
	 */
	double evaluatePoint(const TPoint2D& point) const;
	/**
	 * Check whether a point is inside the line.
	 */
	bool contains(const TPoint2D& point) const;
	/**
	 * Distance from a given point.
	 */
	double distance(const TPoint2D& point) const;
	/**
	 * Distance with sign from a given point (sign indicates side).
	 */
	double signedDistance(const TPoint2D& point) const;
	/**
	 * Get line's normal vector.
	 */
	void getNormalVector(double (&vector)[2]) const;
	/**
	 * Unitarize line's normal vector.
	 */
	void unitarize();
	/**
	 * Get line's normal vector after unitarizing line.
	 */
	void getUnitaryNormalVector(double (&vector)[2])
	{
		unitarize();
		getNormalVector(vector);
	}
	/**
	 * Get line's director vector.
	 */
	void getDirectorVector(double (&vector)[2]) const;
	/**
	 * Unitarize line and then get director vector.
	 */
	void getUnitaryDirectorVector(double (&vector)[2])
	{
		unitarize();
		getDirectorVector(vector);
	}
	/**
	 * Project into 3D space, setting the z to 0.
	 */
	void generate3DObject(TLine3D& l) const;
	/**
	 * Constructor from two points, through which the line will pass.
	 * \throw logic_error if both points are the same
	 */
	TLine2D(const TPoint2D& p1, const TPoint2D& p2);
	/**
	 * Constructor from a segment.
	 */
	explicit TLine2D(const TSegment2D& s);
	/**
	 * Fast default constructor. Initializes to garbage.
	 */
	TLine2D() = default;
	/**
	 * Constructor from line's coefficients.
	 */
	constexpr TLine2D(double A, double B, double C) : coefs{A, B, C} {}
	/**
	 * Construction from 3D object, discarding the Z.
	 * \throw std::logic_error if the line is normal to the XY plane.
	 */
	explicit TLine2D(const TLine3D& l);
	void getAsPose2D(TPose2D& outPose) const;
	void getAsPose2DForcingOrigin(
		const TPoint2D& origin, TPose2D& outPose) const;
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TLine2D& l);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TLine2D& l);

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TLine2D, mrpt::math)
}  // namespace mrpt::typemeta
