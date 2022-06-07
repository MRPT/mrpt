/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/TSegment3D.h>

#include <array>
#include <iosfwd>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/**
 * 3D Plane, represented by its equation \f$Ax+By+Cz+D=0\f$
 * \sa TSegment3D,TLine3D,TPolygon3D,TPoint3D
 */
struct TPlane
{
   public:
	/** Fast default constructor (uninitialized coefficients). */
	TPlane() = default;

	/** Constructor from plane coefficients */
	constexpr TPlane(double A, double B, double C, double D) : coefs{A, B, C, D}
	{
	}

	/** Constructor from an array of coefficients (A,B,C,D). */
	TPlane(const double (&vec)[4])
	{
		for (size_t i = 0; i < 4; i++)
			coefs[i] = vec[i];
	}

	/** Defines a plane which contains these three points.
	 * \throw std::logic_error if the points are linearly dependants.
	 * \sa mrpt::math::getRegressionPlane()
	 */
	TPlane(const TPoint3D& p1, const TPoint3D& p2, const TPoint3D& p3);

	/** \note [New in MRPT 2.1.0]
	 * \sa mrpt::math::getRegressionPlane()
	 */
	static TPlane From3Points(
		const TPoint3D& p1, const TPoint3D& p2, const TPoint3D& p3)
	{
		return {p1, p2, p3};
	}

	/** Defines a plane given a point and a normal vector (must not be unit).
	 * \throw std::logic_error if the normal vector is null
	 */
	TPlane(const TPoint3D& p1, const TVector3D& normal);

	/** \note [New in MRPT 2.1.0] */
	static TPlane FromPointAndNormal(
		const TPoint3D& p1, const TVector3D& normal)
	{
		return {p1, normal};
	}

	/** Defines a plane which contains this point and this line.
	 * \throw std::logic_error if the point is inside the line.
	 */
	TPlane(const TPoint3D& p1, const TLine3D& r2);

	/** Defines a plane which contains this point and this line.
	 * \throw std::logic_error if the point is inside the line.
	 * \note [New in MRPT 2.1.0]
	 */
	static TPlane FromPointAndLine(const TPoint3D& p1, const TLine3D& r)
	{
		return {p1, r};
	}

	/** Defines a plane which contains the two lines.
	 * \throw std::logic_error if the lines do not cross.
	 */
	TPlane(const TLine3D& r1, const TLine3D& r2);

	/** Defines a plane which contains the two lines.
	 * \throw std::logic_error if the lines do not cross.
	 */
	static TPlane FromTwoLines(const TLine3D& r1, const TLine3D& r2)
	{
		return {r1, r2};
	}

	/** Plane coefficients, stored as an array: \f$\left[A,B,C,D\right]\f$ */
	std::array<double, 4> coefs{{.0, .0, .0, .0}};
	/** Evaluate a point in the plane's equation */
	double evaluatePoint(const TPoint3D& point) const;
	/**
	 * Check whether a point is contained into the plane.
	 */
	bool contains(const TPoint3D& point) const;
	/**
	 * Check whether a segment is fully contained into the plane.
	 */
	bool contains(const TSegment3D& segment) const
	{
		return contains(segment.point1) && contains(segment.point2);
	}
	/**
	 * Check whether a line is fully contained into the plane.
	 */
	bool contains(const TLine3D& line) const;

	/** Absolute distance to 3D point */
	double distance(const TPoint3D& point) const;

	/** Signed distance (positive on the normal vector side) to 3D point.
	 *  \note (New in MRPT 2.4.9)
	 */
	double signedDistance(const TPoint3D& point) const;

	/**
	 * Distance to 3D line. Will be zero if the line is not parallel to the
	 * plane.
	 */
	double distance(const TLine3D& line) const;
	/** Get plane's normal vector */
	TVector3D getNormalVector() const;
	/**
	 * Unitarize normal vector.
	 */
	void unitarize();
	void getAsPose3D(mrpt::math::TPose3D& outPose) const;
	void getAsPose3DForcingOrigin(const TPoint3D& center, TPose3D& pose) const;
	TPose3D getAsPose3DForcingOrigin(const TPoint3D& center) const;
	/** Get normal vector */
	TVector3D getUnitaryNormalVector() const;

	/** Returns "[A, B, C, D]"
	 * \note [New in MRPT 2.1.0]
	 */
	std::string asString() const;
};

using TPlane3D = TPlane;

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TPlane& p);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TPlane& p);

/** Text streaming function */
std::ostream& operator<<(std::ostream& o, const TPlane& p);

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPlane, mrpt::math)

}  // namespace mrpt::typemeta
